
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
                    ulong id = id__L(value);
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
                    ulong id = id__f(value);
                    BitUtils.set_bits(id, 7, data, 276);
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
                    BitUtils.set_bits(id, 3, data, 283);
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
                    ulong id = id__f(value);
                    BitUtils.set_bits(id, 7, data, 276);
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
                    BitUtils.set_bits(id, 3, data, 283);
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
                    ulong id = id__f(value);
                    BitUtils.set_bits(id, 7, data, 260);
                }
            }
        }
        new class ALTITUDE : GroundControl.ALTITUDE
        {
            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            /**
            *This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the
            *	local altitude change). The only guarantee on this field is that it will never be reset and is consistent
            *	within a flight. The recommended value for this field is the uncorrected barometric altitude at boot
            *	time. This altitude will also drift and vary between flights*/
            public float altitude_monotonic
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            /**
            *This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events
            *	like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints
            *	are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL
            *	by default and not the WGS84 altitude*/
            public float altitude_amsl
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            /**
            *This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference
            *	to the coordinate origin (0, 0, 0). It is up-positive*/
            public float altitude_local
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float altitude_relative //This is the altitude above the home position. It resets on each change of the current home position
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            /**
            *This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller
            *	than -1000 should be interpreted as unknown*/
            public float altitude_terrain
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            /**
            *This is not the altitude, but the clear space below the system according to the fused clearance estimate.
            *	It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving
            *	target. A negative value indicates no measurement available*/
            public float bottom_clearance
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }
        }
        new class RESOURCE_REQUEST : GroundControl.RESOURCE_REQUEST
        {
            public byte request_id //Request ID. This ID should be re-used when sending back URI contents
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte uri_type //The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            /**
            *The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
            *	on the URI type enum*/
            public byte[] uri
            {
                get {return uri_GET(new byte[120], 0);}
            }
            /**
            *The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
            *	on the URI type enum*/
            public byte[]uri_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 2, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public byte transfer_type //The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  122, 1));}
            }

            /**
            *The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
            *	has a storage associated (e.g. MAVLink FTP)*/
            public byte[] storage
            {
                get {return storage_GET(new byte[120], 0);}
            }
            /**
            *The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
            *	has a storage associated (e.g. MAVLink FTP)*/
            public byte[]storage_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 123, dst_max = pos + 120; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class SCALED_PRESSURE3 : GroundControl.SCALED_PRESSURE3
        {
            public uint time_boot_ms //Timestamp (milliseconds since system boot)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public float press_abs //Absolute pressure (hectopascal)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float press_diff //Differential pressure 1 (hectopascal)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public short temperature //Temperature measurement (0.01 degrees celsius)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
            }
        }
        new class FOLLOW_TARGET : GroundControl.FOLLOW_TARGET
        {
            public ulong timestamp //Timestamp in milliseconds since system boot
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public ulong custom_state //button states or switches of a tracker device
            {
                get {  return (BitUtils.get_bytes(data,  8, 8));}
            }

            public byte est_capabilities //bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
            }

            public int lat //Latitude (WGS84), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  17, 4));}
            }

            public int lon //Longitude (WGS84), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  21, 4));}
            }

            public float alt //AMSL, in meters
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  25, 4)));}
            }

            public float[] vel //target velocity (0,0,0) for unknown
            {
                get {return vel_GET(new float[3], 0);}
            }
            public float[]vel_GET(float[] dst_ch, int pos)  //target velocity (0,0,0) for unknown
            {
                for(int BYTE = 29, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public float[] acc //linear target acceleration (0,0,0) for unknown
            {
                get {return acc_GET(new float[3], 0);}
            }
            public float[]acc_GET(float[] dst_ch, int pos)  //linear target acceleration (0,0,0) for unknown
            {
                for(int BYTE = 41, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public float[] attitude_q //(1 0 0 0 for unknown)
            {
                get {return attitude_q_GET(new float[4], 0);}
            }
            public float[]attitude_q_GET(float[] dst_ch, int pos)  //(1 0 0 0 for unknown)
            {
                for(int BYTE = 53, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public float[] rates //(0 0 0 for unknown)
            {
                get {return rates_GET(new float[3], 0);}
            }
            public float[]rates_GET(float[] dst_ch, int pos)  //(0 0 0 for unknown)
            {
                for(int BYTE = 69, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public float[] position_cov //eph epv
            {
                get {return position_cov_GET(new float[3], 0);}
            }
            public float[]position_cov_GET(float[] dst_ch, int pos)  //eph epv
            {
                for(int BYTE = 81, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
        }
        new class CONTROL_SYSTEM_STATE : GroundControl.CONTROL_SYSTEM_STATE
        {
            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public float x_acc //X acceleration in body frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float y_acc //Y acceleration in body frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float z_acc //Z acceleration in body frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float x_vel //X velocity in body frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float y_vel //Y velocity in body frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float z_vel //Z velocity in body frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float x_pos //X position in local frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float y_pos //Y position in local frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
            }

            public float z_pos //Z position in local frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
            }

            public float airspeed //Airspeed, set to -1 if unknown
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
            }

            public float[] vel_variance //Variance of body velocity estimate
            {
                get {return vel_variance_GET(new float[3], 0);}
            }
            public float[]vel_variance_GET(float[] dst_ch, int pos)  //Variance of body velocity estimate
            {
                for(int BYTE = 48, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public float[] pos_variance //Variance in local position
            {
                get {return pos_variance_GET(new float[3], 0);}
            }
            public float[]pos_variance_GET(float[] dst_ch, int pos)  //Variance in local position
            {
                for(int BYTE = 60, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public float[] q //The attitude, represented as Quaternion
            {
                get {return q_GET(new float[4], 0);}
            }
            public float[]q_GET(float[] dst_ch, int pos)  //The attitude, represented as Quaternion
            {
                for(int BYTE = 72, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public float roll_rate //Angular rate in roll axis
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  88, 4)));}
            }

            public float pitch_rate //Angular rate in pitch axis
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  92, 4)));}
            }

            public float yaw_rate //Angular rate in yaw axis
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  96, 4)));}
            }
        }
        new class BATTERY_STATUS : GroundControl.BATTERY_STATUS
        {
            /**
            *Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
            *	should have the UINT16_MAX value*/
            public ushort[] voltages
            {
                get {return voltages_GET(new ushort[10], 0);}
            }
            /**
            *Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
            *	should have the UINT16_MAX value*/
            public ushort[]voltages_GET(ushort[] dst_ch, int pos)
            {
                for(int BYTE = 0, dst_max = pos + 10; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public byte id //Battery ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  20, 1));}
            }

            public short temperature //Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  21, 2));}
            }

            public short current_battery //Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the curren
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  23, 2));}
            }

            public int current_consumed //Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimat
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  25, 4));}
            }

            /**
            *Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide
            *	energy consumption estimat*/
            public int energy_consumed
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  29, 4));}
            }

            public sbyte battery_remaining //Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  33, 1));}
            }

            public MAV_BATTERY_FUNCTION battery_function //Function of the battery
            {
                get {  return (MAV_BATTERY_FUNCTION)(0 +  BitUtils.get_bits(data, 272, 3));}
            }

            public MAV_BATTERY_TYPE type //Type (chemistry) of the battery
            {
                get {  return (MAV_BATTERY_TYPE)(0 +  BitUtils.get_bits(data, 275, 3));}
            }
        }
        new class AUTOPILOT_VERSION : GroundControl.AUTOPILOT_VERSION
        {
            public ushort vendor_id //ID of the board vendor
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort product_id //ID of the product
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public uint flight_sw_version //Firmware version number
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
            }

            public uint middleware_sw_version //Middleware version number
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  8, 4));}
            }

            public uint os_sw_version //Operating system version number
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  12, 4));}
            }

            public uint board_version //HW / board version (last 8 bytes should be silicon ID, if any)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  16, 4));}
            }

            public ulong uid //UID if provided by hardware (see uid2)
            {
                get {  return (BitUtils.get_bytes(data,  20, 8));}
            }

            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	should allow to identify the commit using the main version number even for very large code bases*/
            public byte[] flight_custom_version
            {
                get {return flight_custom_version_GET(new byte[8], 0);}
            }
            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	should allow to identify the commit using the main version number even for very large code bases*/
            public byte[]flight_custom_version_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 28, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	should allow to identify the commit using the main version number even for very large code bases*/
            public byte[] middleware_custom_version
            {
                get {return middleware_custom_version_GET(new byte[8], 0);}
            }
            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	should allow to identify the commit using the main version number even for very large code bases*/
            public byte[]middleware_custom_version_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 36, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	should allow to identify the commit using the main version number even for very large code bases*/
            public byte[] os_custom_version
            {
                get {return os_custom_version_GET(new byte[8], 0);}
            }
            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	should allow to identify the commit using the main version number even for very large code bases*/
            public byte[]os_custom_version_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 44, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public MAV_PROTOCOL_CAPABILITY capabilities //bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
            {
                get {  return (MAV_PROTOCOL_CAPABILITY)(1 +  BitUtils.get_bits(data, 416, 17));}
            }
            /**
            *UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
            *	use uid*/
            public byte[] uid2_TRY(Inside ph)
            {
                if(ph.field_bit !=  433 && !try_visit_field(ph, 433)) return null;
                return uid2_GET(ph, new byte[ph.items], 0);
            }
            /**
            *UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
            *	use uid*/
            public byte[]uid2_GET(Inside ph, byte[] dst_ch, int pos)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + 18; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public int uid2_LEN()
            {
                return 18;
            }
        }
        new class LANDING_TARGET : GroundControl.LANDING_TARGET
        {
            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public byte target_num //The ID of the target if multiple targets are present
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public float angle_x //X-axis angular offset (in radians) of the target from the center of the image
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  9, 4)));}
            }

            public float angle_y //Y-axis angular offset (in radians) of the target from the center of the image
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  13, 4)));}
            }

            public float distance //Distance to the target from the vehicle in meters
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  17, 4)));}
            }

            public float size_x //Size in radians of target along x-axis
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  21, 4)));}
            }

            public float size_y //Size in radians of target along y-axis
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  25, 4)));}
            }

            public MAV_FRAME frame //MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
            {
                get {  return (MAV_FRAME)(0 +  BitUtils.get_bits(data, 232, 4));}
            }

            public LANDING_TARGET_TYPE type //LANDING_TARGET_TYPE enum specifying the type of landing target
            {
                get {  return (LANDING_TARGET_TYPE)(0 +  BitUtils.get_bits(data, 236, 2));}
            }
            public float x_TRY(Inside ph)//X Position of the landing target on MAV_FRAME
            {
                if(ph.field_bit !=  238 && !try_visit_field(ph, 238)) return 0;
                return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  ph.BYTE, 4)));
            }
            public float y_TRY(Inside ph)//Y Position of the landing target on MAV_FRAME
            {
                if(ph.field_bit !=  239 && !try_visit_field(ph, 239)) return 0;
                return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  ph.BYTE, 4)));
            }
            public float z_TRY(Inside ph)//Z Position of the landing target on MAV_FRAME
            {
                if(ph.field_bit !=  240 && !try_visit_field(ph, 240)) return 0;
                return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  ph.BYTE, 4)));
            }
            public float[] q_TRY(Inside ph)//Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                if(ph.field_bit !=  241 && !try_visit_field(ph, 241)) return null;
                return q_GET(ph, new float[ph.items], 0);
            }
            public float[]q_GET(Inside ph, float[] dst_ch, int pos) //Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            {
                for(int BYTE = ph.BYTE, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public int q_LEN()
            {
                return 4;
            }/**
*Boolean indicating known position (1) or default unkown position (0), for validation of positioning of
*	the landing targe*/
            public byte position_valid_TRY(Inside ph)
            {
                if(ph.field_bit !=  242 && !try_visit_field(ph, 242)) return 0;
                return (byte)((byte) BitUtils.get_bytes(data,  ph.BYTE, 1));
            }
        }
        new class NAV_FILTER_BIAS : GroundControl.NAV_FILTER_BIAS
        {
            public ulong usec //Timestamp (microseconds)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public float accel_0 //b_f[0]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float accel_1 //b_f[1]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float accel_2 //b_f[2]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float gyro_0 //b_f[0]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float gyro_1 //b_f[1]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float gyro_2 //b_f[2]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }
        }
        new class RADIO_CALIBRATION : GroundControl.RADIO_CALIBRATION
        {
            public ushort[] aileron //Aileron setpoints: left, center, right
            {
                get {return aileron_GET(new ushort[3], 0);}
            }
            public ushort[]aileron_GET(ushort[] dst_ch, int pos)  //Aileron setpoints: left, center, right
            {
                for(int BYTE = 0, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public ushort[] elevator //Elevator setpoints: nose down, center, nose up
            {
                get {return elevator_GET(new ushort[3], 0);}
            }
            public ushort[]elevator_GET(ushort[] dst_ch, int pos)  //Elevator setpoints: nose down, center, nose up
            {
                for(int BYTE = 6, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public ushort[] rudder //Rudder setpoints: nose left, center, nose right
            {
                get {return rudder_GET(new ushort[3], 0);}
            }
            public ushort[]rudder_GET(ushort[] dst_ch, int pos)  //Rudder setpoints: nose left, center, nose right
            {
                for(int BYTE = 12, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public ushort[] gyro //Tail gyro mode/gain setpoints: heading hold, rate mode
            {
                get {return gyro_GET(new ushort[2], 0);}
            }
            public ushort[]gyro_GET(ushort[] dst_ch, int pos)  //Tail gyro mode/gain setpoints: heading hold, rate mode
            {
                for(int BYTE = 18, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public ushort[] pitch //Pitch curve setpoints (every 25%)
            {
                get {return pitch_GET(new ushort[5], 0);}
            }
            public ushort[]pitch_GET(ushort[] dst_ch, int pos)  //Pitch curve setpoints (every 25%)
            {
                for(int BYTE = 22, dst_max = pos + 5; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public ushort[] throttle //Throttle curve setpoints (every 25%)
            {
                get {return throttle_GET(new ushort[5], 0);}
            }
            public ushort[]throttle_GET(ushort[] dst_ch, int pos)  //Throttle curve setpoints (every 25%)
            {
                for(int BYTE = 32, dst_max = pos + 5; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
        }
        new class UALBERTA_SYS_STATUS : GroundControl.UALBERTA_SYS_STATUS
        {
            public byte mode //System mode, see UALBERTA_AUTOPILOT_MODE ENUM
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte nav_mode //Navigation mode, see UALBERTA_NAV_MODE ENUM
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte pilot //Pilot mode, see UALBERTA_PILOT_MODE
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }
        }
        new class ESTIMATOR_STATUS : GroundControl.ESTIMATOR_STATUS
        {
            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public float vel_ratio //Velocity innovation test ratio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float pos_horiz_ratio //Horizontal position innovation test ratio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float pos_vert_ratio //Vertical position innovation test ratio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float mag_ratio //Magnetometer innovation test ratio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float hagl_ratio //Height above terrain innovation test ratio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float tas_ratio //True airspeed innovation test ratio
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float pos_horiz_accuracy //Horizontal position 1-STD accuracy relative to the EKF local origin (m)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float pos_vert_accuracy //Vertical position 1-STD accuracy relative to the EKF local origin (m)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
            }

            public ESTIMATOR_STATUS_FLAGS flags //Integer bitmask indicating which EKF outputs are valid. See definition for ESTIMATOR_STATUS_FLAGS.
            {
                get {  return (ESTIMATOR_STATUS_FLAGS)(1 +  BitUtils.get_bits(data, 320, 11));}
            }
        }
        new class WIND_COV : GroundControl.WIND_COV
        {
            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public float wind_x //Wind in X (NED) direction in m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float wind_y //Wind in Y (NED) direction in m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float wind_z //Wind in Z (NED) direction in m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float var_horiz //Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float var_vert //Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float wind_alt //AMSL altitude (m) this measurement was taken at
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float horiz_accuracy //Horizontal speed 1-STD accuracy
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float vert_accuracy //Vertical speed 1-STD accuracy
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
            }
        }
        new class GPS_INPUT : GroundControl.GPS_INPUT
        {
            public ushort time_week //GPS week number
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public uint time_week_ms //GPS time (milliseconds from start of GPS week)
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
            }

            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  6, 8));}
            }

            public byte gps_id //ID of the GPS for multiple GPS inputs
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  14, 1));}
            }

            public byte fix_type //0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  15, 1));}
            }

            public int lat //Latitude (WGS84), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  16, 4));}
            }

            public int lon //Longitude (WGS84), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  20, 4));}
            }

            public float alt //Altitude (AMSL, not WGS84), in m (positive for up)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float hdop //GPS HDOP horizontal dilution of position in m
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float vdop //GPS VDOP vertical dilution of position in m
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float vn //GPS velocity in m/s in NORTH direction in earth-fixed NED frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
            }

            public float ve //GPS velocity in m/s in EAST direction in earth-fixed NED frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
            }

            public float vd //GPS velocity in m/s in DOWN direction in earth-fixed NED frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
            }

            public float speed_accuracy //GPS speed accuracy in m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  48, 4)));}
            }

            public float horiz_accuracy //GPS horizontal accuracy in m
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  52, 4)));}
            }

            public float vert_accuracy //GPS vertical accuracy in m
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  56, 4)));}
            }

            public byte satellites_visible //Number of satellites visible.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  60, 1));}
            }

            public GPS_INPUT_IGNORE_FLAGS ignore_flags //Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided
            {
                get {  return (GPS_INPUT_IGNORE_FLAGS)(1 +  BitUtils.get_bits(data, 488, 8));}
            }
        }
        new class GPS_RTCM_DATA : GroundControl.GPS_RTCM_DATA
        {
            /**
            *LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for
            *	the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed
            *	on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer,
            *	while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered
            *	fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment
            *	with a non full payload is received. This management is used to ensure that normal GPS operation doesn't
            *	corrupt RTCM data, and to recover from a unreliable transport delivery order*/
            public byte flags
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte len //data length
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte[] data_ //RTCM message (may be fragmented)
            {
                get {return data__GET(new byte[180], 0);}
            }
            public byte[]data__GET(byte[] dst_ch, int pos)  //RTCM message (may be fragmented)
            {
                for(int BYTE = 2, dst_max = pos + 180; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class HIGH_LATENCY : GroundControl.HIGH_LATENCY
        {
            public ushort heading //heading (centidegrees)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort wp_distance //distance to target (meters)
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public uint custom_mode //A bitfield for use for autopilot-specific flags.
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
            }

            public short roll //roll (centidegrees)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
            }

            public short pitch //pitch (centidegrees)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  10, 2));}
            }

            public sbyte throttle //throttle (percentage)
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  12, 1));}
            }

            public short heading_sp //heading setpoint (centidegrees)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  13, 2));}
            }

            public int latitude //Latitude, expressed as degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  15, 4));}
            }

            public int longitude //Longitude, expressed as degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  19, 4));}
            }

            public short altitude_amsl //Altitude above mean sea level (meters)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  23, 2));}
            }

            public short altitude_sp //Altitude setpoint relative to the home position (meters)
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  25, 2));}
            }

            public byte airspeed //airspeed (m/s)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  27, 1));}
            }

            public byte airspeed_sp //airspeed setpoint (m/s)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  28, 1));}
            }

            public byte groundspeed //groundspeed (m/s)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  29, 1));}
            }

            public sbyte climb_rate //climb rate (m/s)
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  30, 1));}
            }

            public byte gps_nsat //Number of satellites visible. If unknown, set to 255
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  31, 1));}
            }

            public byte battery_remaining //Remaining battery (percentage)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  32, 1));}
            }

            public sbyte temperature //Autopilot temperature (degrees C)
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  33, 1));}
            }

            public sbyte temperature_air //Air temperature (degrees C) from airspeed sensor
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  34, 1));}
            }

            /**
            *failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS,
            *	bit3:GCS, bit4:fence*/
            public byte failsafe
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  35, 1));}
            }

            public byte wp_num //current waypoint number
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  36, 1));}
            }

            public MAV_MODE_FLAG base_mode //System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
            {
                get {  return (MAV_MODE_FLAG)(1 +  BitUtils.get_bits(data, 296, 8));}
            }

            public MAV_LANDED_STATE landed_state //The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
            {
                get {  return (MAV_LANDED_STATE)(0 +  BitUtils.get_bits(data, 304, 3));}
            }

            public GPS_FIX_TYPE gps_fix_type //See the GPS_FIX_TYPE enum.
            {
                get {  return (GPS_FIX_TYPE)(0 +  BitUtils.get_bits(data, 307, 4));}
            }
        }
        new class VIBRATION : GroundControl.VIBRATION
        {
            public uint clipping_0 //first accelerometer clipping count
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public uint clipping_1 //second accelerometer clipping count
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
            }

            public uint clipping_2 //third accelerometer clipping count
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  8, 4));}
            }

            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  12, 8));}
            }

            public float vibration_x //Vibration levels on X-axis
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float vibration_y //Vibration levels on Y-axis
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float vibration_z //Vibration levels on Z-axis
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }
        }
        new class HOME_POSITION : GroundControl.HOME_POSITION
        {
            public int latitude //Latitude (WGS84), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  0, 4));}
            }

            public int longitude //Longitude (WGS84, in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  4, 4));}
            }

            public int altitude //Altitude (AMSL), in meters * 1000 (positive for up)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  8, 4));}
            }

            public float x //Local X position of this position in the local coordinate frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float y //Local Y position of this position in the local coordinate frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float z //Local Z position of this position in the local coordinate frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            /**
            *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
            *	and slope of the groun*/
            public float[] q
            {
                get {return q_GET(new float[4], 0);}
            }
            /**
            *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
            *	and slope of the groun*/
            public float[]q_GET(float[] dst_ch, int pos)
            {
                for(int BYTE = 24, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            /**
            *Local X position of the end of the approach vector. Multicopters should set this position based on their
            *	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	from the threshold / touchdown zone*/
            public float approach_x
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
            }

            /**
            *Local Y position of the end of the approach vector. Multicopters should set this position based on their
            *	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	from the threshold / touchdown zone*/
            public float approach_y
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
            }

            /**
            *Local Z position of the end of the approach vector. Multicopters should set this position based on their
            *	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	from the threshold / touchdown zone*/
            public float approach_z
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  48, 4)));}
            }
            public ulong time_usec_TRY(Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                if(ph.field_bit !=  416 && !try_visit_field(ph, 416)) return 0;
                return (BitUtils.get_bytes(data,  ph.BYTE, 8));
            }
        }
        new class SET_HOME_POSITION : GroundControl.SET_HOME_POSITION
        {
            public byte target_system //System ID.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public int latitude //Latitude (WGS84), in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  1, 4));}
            }

            public int longitude //Longitude (WGS84, in degrees * 1E7
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  5, 4));}
            }

            public int altitude //Altitude (AMSL), in meters * 1000 (positive for up)
            {
                get {  return (int)((int) BitUtils.get_bytes(data,  9, 4));}
            }

            public float x //Local X position of this position in the local coordinate frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  13, 4)));}
            }

            public float y //Local Y position of this position in the local coordinate frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  17, 4)));}
            }

            public float z //Local Z position of this position in the local coordinate frame
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  21, 4)));}
            }

            /**
            *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
            *	and slope of the groun*/
            public float[] q
            {
                get {return q_GET(new float[4], 0);}
            }
            /**
            *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
            *	and slope of the groun*/
            public float[]q_GET(float[] dst_ch, int pos)
            {
                for(int BYTE = 25, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            /**
            *Local X position of the end of the approach vector. Multicopters should set this position based on their
            *	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	from the threshold / touchdown zone*/
            public float approach_x
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  41, 4)));}
            }

            /**
            *Local Y position of the end of the approach vector. Multicopters should set this position based on their
            *	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	from the threshold / touchdown zone*/
            public float approach_y
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  45, 4)));}
            }

            /**
            *Local Z position of the end of the approach vector. Multicopters should set this position based on their
            *	takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	from the threshold / touchdown zone*/
            public float approach_z
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  49, 4)));}
            }
            public ulong time_usec_TRY(Inside ph)//Timestamp (microseconds since UNIX epoch or microseconds since system boot)
            {
                if(ph.field_bit !=  424 && !try_visit_field(ph, 424)) return 0;
                return (BitUtils.get_bytes(data,  ph.BYTE, 8));
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

            public void OnALTITUDEReceive_direct(Channel src, Inside ph, ALTITUDE pack) {OnALTITUDEReceive(this, ph,  pack);}
            public event ALTITUDEReceiveHandler OnALTITUDEReceive;
            public delegate void ALTITUDEReceiveHandler(Channel src, Inside ph, ALTITUDE pack);
            public void OnRESOURCE_REQUESTReceive_direct(Channel src, Inside ph, RESOURCE_REQUEST pack) {OnRESOURCE_REQUESTReceive(this, ph,  pack);}
            public event RESOURCE_REQUESTReceiveHandler OnRESOURCE_REQUESTReceive;
            public delegate void RESOURCE_REQUESTReceiveHandler(Channel src, Inside ph, RESOURCE_REQUEST pack);
            public void OnSCALED_PRESSURE3Receive_direct(Channel src, Inside ph, SCALED_PRESSURE3 pack) {OnSCALED_PRESSURE3Receive(this, ph,  pack);}
            public event SCALED_PRESSURE3ReceiveHandler OnSCALED_PRESSURE3Receive;
            public delegate void SCALED_PRESSURE3ReceiveHandler(Channel src, Inside ph, SCALED_PRESSURE3 pack);
            public void OnFOLLOW_TARGETReceive_direct(Channel src, Inside ph, FOLLOW_TARGET pack) {OnFOLLOW_TARGETReceive(this, ph,  pack);}
            public event FOLLOW_TARGETReceiveHandler OnFOLLOW_TARGETReceive;
            public delegate void FOLLOW_TARGETReceiveHandler(Channel src, Inside ph, FOLLOW_TARGET pack);
            public void OnCONTROL_SYSTEM_STATEReceive_direct(Channel src, Inside ph, CONTROL_SYSTEM_STATE pack) {OnCONTROL_SYSTEM_STATEReceive(this, ph,  pack);}
            public event CONTROL_SYSTEM_STATEReceiveHandler OnCONTROL_SYSTEM_STATEReceive;
            public delegate void CONTROL_SYSTEM_STATEReceiveHandler(Channel src, Inside ph, CONTROL_SYSTEM_STATE pack);
            public void OnBATTERY_STATUSReceive_direct(Channel src, Inside ph, BATTERY_STATUS pack) {OnBATTERY_STATUSReceive(this, ph,  pack);}
            public event BATTERY_STATUSReceiveHandler OnBATTERY_STATUSReceive;
            public delegate void BATTERY_STATUSReceiveHandler(Channel src, Inside ph, BATTERY_STATUS pack);
            public void OnAUTOPILOT_VERSIONReceive_direct(Channel src, Inside ph, AUTOPILOT_VERSION pack) {OnAUTOPILOT_VERSIONReceive(this, ph,  pack);}
            public event AUTOPILOT_VERSIONReceiveHandler OnAUTOPILOT_VERSIONReceive;
            public delegate void AUTOPILOT_VERSIONReceiveHandler(Channel src, Inside ph, AUTOPILOT_VERSION pack);
            public void OnLANDING_TARGETReceive_direct(Channel src, Inside ph, LANDING_TARGET pack) {OnLANDING_TARGETReceive(this, ph,  pack);}
            public event LANDING_TARGETReceiveHandler OnLANDING_TARGETReceive;
            public delegate void LANDING_TARGETReceiveHandler(Channel src, Inside ph, LANDING_TARGET pack);
            public void OnNAV_FILTER_BIASReceive_direct(Channel src, Inside ph, NAV_FILTER_BIAS pack) {OnNAV_FILTER_BIASReceive(this, ph,  pack);}
            public event NAV_FILTER_BIASReceiveHandler OnNAV_FILTER_BIASReceive;
            public delegate void NAV_FILTER_BIASReceiveHandler(Channel src, Inside ph, NAV_FILTER_BIAS pack);
            public void OnRADIO_CALIBRATIONReceive_direct(Channel src, Inside ph, RADIO_CALIBRATION pack) {OnRADIO_CALIBRATIONReceive(this, ph,  pack);}
            public event RADIO_CALIBRATIONReceiveHandler OnRADIO_CALIBRATIONReceive;
            public delegate void RADIO_CALIBRATIONReceiveHandler(Channel src, Inside ph, RADIO_CALIBRATION pack);
            public void OnUALBERTA_SYS_STATUSReceive_direct(Channel src, Inside ph, UALBERTA_SYS_STATUS pack) {OnUALBERTA_SYS_STATUSReceive(this, ph,  pack);}
            public event UALBERTA_SYS_STATUSReceiveHandler OnUALBERTA_SYS_STATUSReceive;
            public delegate void UALBERTA_SYS_STATUSReceiveHandler(Channel src, Inside ph, UALBERTA_SYS_STATUS pack);
            public void OnESTIMATOR_STATUSReceive_direct(Channel src, Inside ph, ESTIMATOR_STATUS pack) {OnESTIMATOR_STATUSReceive(this, ph,  pack);}
            public event ESTIMATOR_STATUSReceiveHandler OnESTIMATOR_STATUSReceive;
            public delegate void ESTIMATOR_STATUSReceiveHandler(Channel src, Inside ph, ESTIMATOR_STATUS pack);
            public void OnWIND_COVReceive_direct(Channel src, Inside ph, WIND_COV pack) {OnWIND_COVReceive(this, ph,  pack);}
            public event WIND_COVReceiveHandler OnWIND_COVReceive;
            public delegate void WIND_COVReceiveHandler(Channel src, Inside ph, WIND_COV pack);
            public void OnGPS_INPUTReceive_direct(Channel src, Inside ph, GPS_INPUT pack) {OnGPS_INPUTReceive(this, ph,  pack);}
            public event GPS_INPUTReceiveHandler OnGPS_INPUTReceive;
            public delegate void GPS_INPUTReceiveHandler(Channel src, Inside ph, GPS_INPUT pack);
            public void OnGPS_RTCM_DATAReceive_direct(Channel src, Inside ph, GPS_RTCM_DATA pack) {OnGPS_RTCM_DATAReceive(this, ph,  pack);}
            public event GPS_RTCM_DATAReceiveHandler OnGPS_RTCM_DATAReceive;
            public delegate void GPS_RTCM_DATAReceiveHandler(Channel src, Inside ph, GPS_RTCM_DATA pack);
            public void OnHIGH_LATENCYReceive_direct(Channel src, Inside ph, HIGH_LATENCY pack) {OnHIGH_LATENCYReceive(this, ph,  pack);}
            public event HIGH_LATENCYReceiveHandler OnHIGH_LATENCYReceive;
            public delegate void HIGH_LATENCYReceiveHandler(Channel src, Inside ph, HIGH_LATENCY pack);
            public void OnVIBRATIONReceive_direct(Channel src, Inside ph, VIBRATION pack) {OnVIBRATIONReceive(this, ph,  pack);}
            public event VIBRATIONReceiveHandler OnVIBRATIONReceive;
            public delegate void VIBRATIONReceiveHandler(Channel src, Inside ph, VIBRATION pack);
            public void OnHOME_POSITIONReceive_direct(Channel src, Inside ph, HOME_POSITION pack) {OnHOME_POSITIONReceive(this, ph,  pack);}
            public event HOME_POSITIONReceiveHandler OnHOME_POSITIONReceive;
            public delegate void HOME_POSITIONReceiveHandler(Channel src, Inside ph, HOME_POSITION pack);
            public void OnSET_HOME_POSITIONReceive_direct(Channel src, Inside ph, SET_HOME_POSITION pack) {OnSET_HOME_POSITIONReceive(this, ph,  pack);}
            public event SET_HOME_POSITIONReceiveHandler OnSET_HOME_POSITIONReceive;
            public delegate void SET_HOME_POSITIONReceiveHandler(Channel src, Inside ph, SET_HOME_POSITION pack);
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
                    case 141:
                        if(pack == null) return new ALTITUDE();
                        OnALTITUDEReceive(this, ph, (ALTITUDE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 142:
                        if(pack == null) return new RESOURCE_REQUEST();
                        OnRESOURCE_REQUESTReceive(this, ph, (RESOURCE_REQUEST) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 143:
                        if(pack == null) return new SCALED_PRESSURE3();
                        OnSCALED_PRESSURE3Receive(this, ph, (SCALED_PRESSURE3) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 144:
                        if(pack == null) return new FOLLOW_TARGET();
                        OnFOLLOW_TARGETReceive(this, ph, (FOLLOW_TARGET) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 146:
                        if(pack == null) return new CONTROL_SYSTEM_STATE();
                        OnCONTROL_SYSTEM_STATEReceive(this, ph, (CONTROL_SYSTEM_STATE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 147:
                        if(pack == null) return new BATTERY_STATUS();
                        OnBATTERY_STATUSReceive(this, ph, (BATTERY_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 148:
                        if(pack == null) return new AUTOPILOT_VERSION();
                        OnAUTOPILOT_VERSIONReceive(this, ph, (AUTOPILOT_VERSION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 149:
                        if(pack == null) return new LANDING_TARGET();
                        OnLANDING_TARGETReceive(this, ph, (LANDING_TARGET) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 220:
                        if(pack == null) return new NAV_FILTER_BIAS();
                        OnNAV_FILTER_BIASReceive(this, ph, (NAV_FILTER_BIAS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 221:
                        if(pack == null) return new RADIO_CALIBRATION();
                        OnRADIO_CALIBRATIONReceive(this, ph, (RADIO_CALIBRATION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 222:
                        if(pack == null) return new UALBERTA_SYS_STATUS();
                        OnUALBERTA_SYS_STATUSReceive(this, ph, (UALBERTA_SYS_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 230:
                        if(pack == null) return new ESTIMATOR_STATUS();
                        OnESTIMATOR_STATUSReceive(this, ph, (ESTIMATOR_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 231:
                        if(pack == null) return new WIND_COV();
                        OnWIND_COVReceive(this, ph, (WIND_COV) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 232:
                        if(pack == null) return new GPS_INPUT();
                        OnGPS_INPUTReceive(this, ph, (GPS_INPUT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 233:
                        if(pack == null) return new GPS_RTCM_DATA();
                        OnGPS_RTCM_DATAReceive(this, ph, (GPS_RTCM_DATA) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 234:
                        if(pack == null) return new HIGH_LATENCY();
                        OnHIGH_LATENCYReceive(this, ph, (HIGH_LATENCY) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 241:
                        if(pack == null) return new VIBRATION();
                        OnVIBRATIONReceive(this, ph, (VIBRATION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 242:
                        if(pack == null) return new HOME_POSITION();
                        OnHOME_POSITIONReceive(this, ph, (HOME_POSITION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 243:
                        if(pack == null) return new SET_HOME_POSITION();
                        OnSET_HOME_POSITIONReceive(this, ph, (SET_HOME_POSITION) pack);//no any host channels can receive this pack. Handle it with test channel handler
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
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED));
                Debug.Assert(pack.autopilot == MAV_AUTOPILOT.MAV_AUTOPILOT_PPZ);
                Debug.Assert(pack.custom_mode == (uint)2907467331U);
                Debug.Assert(pack.system_status == MAV_STATE.MAV_STATE_POWEROFF);
                Debug.Assert(pack.type == MAV_TYPE.MAV_TYPE_TRICOPTER);
                Debug.Assert(pack.mavlink_version == (byte)(byte)64);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.type = MAV_TYPE.MAV_TYPE_TRICOPTER;
            p0.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_PPZ;
            p0.mavlink_version = (byte)(byte)64;
            p0.custom_mode = (uint)2907467331U;
            p0.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
            p0.system_status = MAV_STATE.MAV_STATE_POWEROFF;
            SMP_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)61509);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)62182);
                Debug.Assert(pack.current_battery == (short)(short)16668);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)48932);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 39);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)57433);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
                Debug.Assert(pack.load == (ushort)(ushort)1685);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)58030);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)46570);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)19869);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.errors_comm = (ushort)(ushort)46570;
            p1.errors_count2 = (ushort)(ushort)62182;
            p1.current_battery = (short)(short)16668;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL);
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.battery_remaining = (sbyte)(sbyte) - 39;
            p1.errors_count4 = (ushort)(ushort)61509;
            p1.errors_count1 = (ushort)(ushort)58030;
            p1.load = (ushort)(ushort)1685;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.drop_rate_comm = (ushort)(ushort)19869;
            p1.voltage_battery = (ushort)(ushort)57433;
            p1.errors_count3 = (ushort)(ushort)48932;
            SMP_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)2715853294463646730L);
                Debug.Assert(pack.time_boot_ms == (uint)2965878040U);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)2715853294463646730L;
            p2.time_boot_ms = (uint)2965878040U;
            SMP_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -2.9918047E38F);
                Debug.Assert(pack.afx == (float)1.801083E38F);
                Debug.Assert(pack.yaw == (float)2.986508E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.y == (float)2.6247764E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)3881);
                Debug.Assert(pack.vx == (float)2.508486E38F);
                Debug.Assert(pack.yaw_rate == (float)2.7251052E38F);
                Debug.Assert(pack.x == (float)1.4299583E38F);
                Debug.Assert(pack.afz == (float) -2.040573E38F);
                Debug.Assert(pack.time_boot_ms == (uint)723283423U);
                Debug.Assert(pack.vz == (float)8.759572E37F);
                Debug.Assert(pack.vy == (float) -2.6631917E38F);
                Debug.Assert(pack.afy == (float)6.652096E37F);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.yaw_rate = (float)2.7251052E38F;
            p3.vx = (float)2.508486E38F;
            p3.x = (float)1.4299583E38F;
            p3.z = (float) -2.9918047E38F;
            p3.type_mask = (ushort)(ushort)3881;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p3.afz = (float) -2.040573E38F;
            p3.afy = (float)6.652096E37F;
            p3.afx = (float)1.801083E38F;
            p3.time_boot_ms = (uint)723283423U;
            p3.yaw = (float)2.986508E38F;
            p3.y = (float)2.6247764E38F;
            p3.vy = (float) -2.6631917E38F;
            p3.vz = (float)8.759572E37F;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)77);
                Debug.Assert(pack.time_usec == (ulong)5870159160017509167L);
                Debug.Assert(pack.target_component == (byte)(byte)142);
                Debug.Assert(pack.seq == (uint)4160479312U);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.target_system = (byte)(byte)77;
            p4.target_component = (byte)(byte)142;
            p4.seq = (uint)4160479312U;
            p4.time_usec = (ulong)5870159160017509167L;
            SMP_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.version == (byte)(byte)248);
                Debug.Assert(pack.passkey_LEN(ph) == 19);
                Debug.Assert(pack.passkey_TRY(ph).Equals("cvehyxwjxcbjogekrNg"));
                Debug.Assert(pack.control_request == (byte)(byte)242);
                Debug.Assert(pack.target_system == (byte)(byte)237);
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.version = (byte)(byte)248;
            p5.passkey_SET("cvehyxwjxcbjogekrNg", PH) ;
            p5.target_system = (byte)(byte)237;
            p5.control_request = (byte)(byte)242;
            SMP_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ack == (byte)(byte)99);
                Debug.Assert(pack.control_request == (byte)(byte)98);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)133);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.control_request = (byte)(byte)98;
            p6.gcs_system_id = (byte)(byte)133;
            p6.ack = (byte)(byte)99;
            SMP_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 23);
                Debug.Assert(pack.key_TRY(ph).Equals("diqokzdofoRKgKivusapNlv"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("diqokzdofoRKgKivusapNlv", PH) ;
            SMP_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.custom_mode == (uint)1840155351U);
                Debug.Assert(pack.base_mode == MAV_MODE.MAV_MODE_TEST_ARMED);
                Debug.Assert(pack.target_system == (byte)(byte)170);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.base_mode = MAV_MODE.MAV_MODE_TEST_ARMED;
            p11.target_system = (byte)(byte)170;
            p11.custom_mode = (uint)1840155351U;
            SMP_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("r"));
                Debug.Assert(pack.target_component == (byte)(byte)99);
                Debug.Assert(pack.param_index == (short)(short) -17749);
                Debug.Assert(pack.target_system == (byte)(byte)50);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.param_index = (short)(short) -17749;
            p20.param_id_SET("r", PH) ;
            p20.target_system = (byte)(byte)50;
            p20.target_component = (byte)(byte)99;
            SMP_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)16);
                Debug.Assert(pack.target_system == (byte)(byte)90);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)90;
            p21.target_component = (byte)(byte)16;
            SMP_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 11);
                Debug.Assert(pack.param_id_TRY(ph).Equals("zfwpdokmtbr"));
                Debug.Assert(pack.param_value == (float) -5.7794767E37F);
                Debug.Assert(pack.param_index == (ushort)(ushort)5345);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8);
                Debug.Assert(pack.param_count == (ushort)(ushort)2255);
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8;
            p22.param_count = (ushort)(ushort)2255;
            p22.param_id_SET("zfwpdokmtbr", PH) ;
            p22.param_index = (ushort)(ushort)5345;
            p22.param_value = (float) -5.7794767E37F;
            SMP_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)24);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
                Debug.Assert(pack.param_value == (float)8.325689E37F);
                Debug.Assert(pack.target_system == (byte)(byte)181);
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("fsx"));
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.target_system = (byte)(byte)181;
            p23.target_component = (byte)(byte)24;
            p23.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32;
            p23.param_id_SET("fsx", PH) ;
            p23.param_value = (float)8.325689E37F;
            SMP_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (int) -1164269824);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)3012336229U);
                Debug.Assert(pack.cog == (ushort)(ushort)51364);
                Debug.Assert(pack.lon == (int) -1416850623);
                Debug.Assert(pack.vel == (ushort)(ushort)28718);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)4050073917U);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -1674276290);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)1457239004U);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)4043328374U);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
                Debug.Assert(pack.satellites_visible == (byte)(byte)8);
                Debug.Assert(pack.lat == (int) -1423672111);
                Debug.Assert(pack.time_usec == (ulong)8309315111956905428L);
                Debug.Assert(pack.epv == (ushort)(ushort)61322);
                Debug.Assert(pack.eph == (ushort)(ushort)42517);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.v_acc_SET((uint)4043328374U, PH) ;
            p24.lat = (int) -1423672111;
            p24.lon = (int) -1416850623;
            p24.vel = (ushort)(ushort)28718;
            p24.eph = (ushort)(ushort)42517;
            p24.vel_acc_SET((uint)4050073917U, PH) ;
            p24.epv = (ushort)(ushort)61322;
            p24.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_PPP;
            p24.satellites_visible = (byte)(byte)8;
            p24.time_usec = (ulong)8309315111956905428L;
            p24.alt_ellipsoid_SET((int) -1674276290, PH) ;
            p24.hdg_acc_SET((uint)3012336229U, PH) ;
            p24.alt = (int) -1164269824;
            p24.cog = (ushort)(ushort)51364;
            p24.h_acc_SET((uint)1457239004U, PH) ;
            SMP_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)119, (byte)212, (byte)4, (byte)226, (byte)205, (byte)194, (byte)80, (byte)30, (byte)115, (byte)35, (byte)44, (byte)181, (byte)243, (byte)149, (byte)238, (byte)189, (byte)239, (byte)248, (byte)110, (byte)117}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)246);
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)207, (byte)221, (byte)189, (byte)112, (byte)167, (byte)102, (byte)236, (byte)168, (byte)108, (byte)248, (byte)76, (byte)5, (byte)215, (byte)112, (byte)99, (byte)144, (byte)146, (byte)9, (byte)128, (byte)21}));
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)240, (byte)178, (byte)12, (byte)218, (byte)235, (byte)20, (byte)129, (byte)236, (byte)34, (byte)197, (byte)233, (byte)67, (byte)118, (byte)30, (byte)247, (byte)73, (byte)132, (byte)18, (byte)73, (byte)70}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)180, (byte)61, (byte)98, (byte)6, (byte)239, (byte)166, (byte)73, (byte)193, (byte)244, (byte)165, (byte)14, (byte)28, (byte)147, (byte)155, (byte)110, (byte)170, (byte)216, (byte)192, (byte)230, (byte)203}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)80, (byte)101, (byte)86, (byte)80, (byte)107, (byte)23, (byte)130, (byte)85, (byte)247, (byte)42, (byte)144, (byte)242, (byte)174, (byte)216, (byte)109, (byte)157, (byte)77, (byte)141, (byte)250, (byte)118}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_prn_SET(new byte[] {(byte)119, (byte)212, (byte)4, (byte)226, (byte)205, (byte)194, (byte)80, (byte)30, (byte)115, (byte)35, (byte)44, (byte)181, (byte)243, (byte)149, (byte)238, (byte)189, (byte)239, (byte)248, (byte)110, (byte)117}, 0) ;
            p25.satellites_visible = (byte)(byte)246;
            p25.satellite_elevation_SET(new byte[] {(byte)240, (byte)178, (byte)12, (byte)218, (byte)235, (byte)20, (byte)129, (byte)236, (byte)34, (byte)197, (byte)233, (byte)67, (byte)118, (byte)30, (byte)247, (byte)73, (byte)132, (byte)18, (byte)73, (byte)70}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)80, (byte)101, (byte)86, (byte)80, (byte)107, (byte)23, (byte)130, (byte)85, (byte)247, (byte)42, (byte)144, (byte)242, (byte)174, (byte)216, (byte)109, (byte)157, (byte)77, (byte)141, (byte)250, (byte)118}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)180, (byte)61, (byte)98, (byte)6, (byte)239, (byte)166, (byte)73, (byte)193, (byte)244, (byte)165, (byte)14, (byte)28, (byte)147, (byte)155, (byte)110, (byte)170, (byte)216, (byte)192, (byte)230, (byte)203}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)207, (byte)221, (byte)189, (byte)112, (byte)167, (byte)102, (byte)236, (byte)168, (byte)108, (byte)248, (byte)76, (byte)5, (byte)215, (byte)112, (byte)99, (byte)144, (byte)146, (byte)9, (byte)128, (byte)21}, 0) ;
            SMP_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1806255295U);
                Debug.Assert(pack.yacc == (short)(short)9078);
                Debug.Assert(pack.xacc == (short)(short) -24916);
                Debug.Assert(pack.ygyro == (short)(short) -27876);
                Debug.Assert(pack.zacc == (short)(short)22168);
                Debug.Assert(pack.ymag == (short)(short)25133);
                Debug.Assert(pack.zgyro == (short)(short) -5971);
                Debug.Assert(pack.xmag == (short)(short) -23444);
                Debug.Assert(pack.zmag == (short)(short)28715);
                Debug.Assert(pack.xgyro == (short)(short) -31494);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.xgyro = (short)(short) -31494;
            p26.time_boot_ms = (uint)1806255295U;
            p26.xmag = (short)(short) -23444;
            p26.zgyro = (short)(short) -5971;
            p26.zacc = (short)(short)22168;
            p26.zmag = (short)(short)28715;
            p26.yacc = (short)(short)9078;
            p26.xacc = (short)(short) -24916;
            p26.ymag = (short)(short)25133;
            p26.ygyro = (short)(short) -27876;
            SMP_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (short)(short)13346);
                Debug.Assert(pack.zmag == (short)(short) -14285);
                Debug.Assert(pack.ygyro == (short)(short) -26032);
                Debug.Assert(pack.yacc == (short)(short) -11677);
                Debug.Assert(pack.xgyro == (short)(short)9735);
                Debug.Assert(pack.time_usec == (ulong)3685014187640538178L);
                Debug.Assert(pack.xacc == (short)(short) -16703);
                Debug.Assert(pack.zacc == (short)(short)12865);
                Debug.Assert(pack.ymag == (short)(short) -25089);
                Debug.Assert(pack.xmag == (short)(short)25127);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.ymag = (short)(short) -25089;
            p27.xmag = (short)(short)25127;
            p27.xgyro = (short)(short)9735;
            p27.time_usec = (ulong)3685014187640538178L;
            p27.xacc = (short)(short) -16703;
            p27.zgyro = (short)(short)13346;
            p27.zmag = (short)(short) -14285;
            p27.ygyro = (short)(short) -26032;
            p27.zacc = (short)(short)12865;
            p27.yacc = (short)(short) -11677;
            SMP_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff1 == (short)(short) -29063);
                Debug.Assert(pack.time_usec == (ulong)6794350941429952314L);
                Debug.Assert(pack.temperature == (short)(short)10997);
                Debug.Assert(pack.press_diff2 == (short)(short)19954);
                Debug.Assert(pack.press_abs == (short)(short) -4798);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_abs = (short)(short) -4798;
            p28.press_diff1 = (short)(short) -29063;
            p28.time_usec = (ulong)6794350941429952314L;
            p28.press_diff2 = (short)(short)19954;
            p28.temperature = (short)(short)10997;
            SMP_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)5986);
                Debug.Assert(pack.press_diff == (float)1.2553257E38F);
                Debug.Assert(pack.press_abs == (float) -1.5602543E38F);
                Debug.Assert(pack.time_boot_ms == (uint)225075477U);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.press_diff = (float)1.2553257E38F;
            p29.time_boot_ms = (uint)225075477U;
            p29.press_abs = (float) -1.5602543E38F;
            p29.temperature = (short)(short)5986;
            SMP_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float) -1.951461E38F);
                Debug.Assert(pack.yawspeed == (float) -2.9401282E38F);
                Debug.Assert(pack.yaw == (float) -2.3427781E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4058156047U);
                Debug.Assert(pack.roll == (float) -3.0729617E38F);
                Debug.Assert(pack.pitch == (float) -2.8869252E38F);
                Debug.Assert(pack.pitchspeed == (float) -2.0395654E38F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.pitch = (float) -2.8869252E38F;
            p30.rollspeed = (float) -1.951461E38F;
            p30.roll = (float) -3.0729617E38F;
            p30.yaw = (float) -2.3427781E38F;
            p30.pitchspeed = (float) -2.0395654E38F;
            p30.time_boot_ms = (uint)4058156047U;
            p30.yawspeed = (float) -2.9401282E38F;
            SMP_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q3 == (float)2.6127277E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1899527446U);
                Debug.Assert(pack.q2 == (float) -2.0740926E38F);
                Debug.Assert(pack.q4 == (float)2.1864121E38F);
                Debug.Assert(pack.yawspeed == (float) -2.0252753E38F);
                Debug.Assert(pack.rollspeed == (float)1.7931727E38F);
                Debug.Assert(pack.pitchspeed == (float) -1.487168E38F);
                Debug.Assert(pack.q1 == (float)2.2196039E38F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.pitchspeed = (float) -1.487168E38F;
            p31.q2 = (float) -2.0740926E38F;
            p31.q4 = (float)2.1864121E38F;
            p31.time_boot_ms = (uint)1899527446U;
            p31.rollspeed = (float)1.7931727E38F;
            p31.yawspeed = (float) -2.0252753E38F;
            p31.q1 = (float)2.2196039E38F;
            p31.q3 = (float)2.6127277E38F;
            SMP_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -1.8570599E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3068356836U);
                Debug.Assert(pack.x == (float) -1.2292607E38F);
                Debug.Assert(pack.vz == (float)8.831562E37F);
                Debug.Assert(pack.vy == (float)2.0358738E38F);
                Debug.Assert(pack.vx == (float)2.027246E38F);
                Debug.Assert(pack.z == (float)1.2909744E38F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.x = (float) -1.2292607E38F;
            p32.time_boot_ms = (uint)3068356836U;
            p32.vy = (float)2.0358738E38F;
            p32.vz = (float)8.831562E37F;
            p32.vx = (float)2.027246E38F;
            p32.y = (float) -1.8570599E38F;
            p32.z = (float)1.2909744E38F;
            SMP_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3243939126U);
                Debug.Assert(pack.relative_alt == (int) -1250479132);
                Debug.Assert(pack.hdg == (ushort)(ushort)32074);
                Debug.Assert(pack.alt == (int) -268774350);
                Debug.Assert(pack.vx == (short)(short)18719);
                Debug.Assert(pack.vy == (short)(short) -7922);
                Debug.Assert(pack.lon == (int)561149432);
                Debug.Assert(pack.vz == (short)(short)17964);
                Debug.Assert(pack.lat == (int) -1589982324);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.lat = (int) -1589982324;
            p33.alt = (int) -268774350;
            p33.vy = (short)(short) -7922;
            p33.vx = (short)(short)18719;
            p33.vz = (short)(short)17964;
            p33.hdg = (ushort)(ushort)32074;
            p33.relative_alt = (int) -1250479132;
            p33.lon = (int)561149432;
            p33.time_boot_ms = (uint)3243939126U;
            SMP_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan1_scaled == (short)(short)14249);
                Debug.Assert(pack.chan8_scaled == (short)(short)20743);
                Debug.Assert(pack.time_boot_ms == (uint)2708432070U);
                Debug.Assert(pack.rssi == (byte)(byte)171);
                Debug.Assert(pack.chan6_scaled == (short)(short) -1110);
                Debug.Assert(pack.chan5_scaled == (short)(short)7240);
                Debug.Assert(pack.port == (byte)(byte)29);
                Debug.Assert(pack.chan3_scaled == (short)(short) -17980);
                Debug.Assert(pack.chan4_scaled == (short)(short)23746);
                Debug.Assert(pack.chan2_scaled == (short)(short)5013);
                Debug.Assert(pack.chan7_scaled == (short)(short) -14176);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.rssi = (byte)(byte)171;
            p34.chan2_scaled = (short)(short)5013;
            p34.chan1_scaled = (short)(short)14249;
            p34.chan7_scaled = (short)(short) -14176;
            p34.time_boot_ms = (uint)2708432070U;
            p34.chan8_scaled = (short)(short)20743;
            p34.port = (byte)(byte)29;
            p34.chan3_scaled = (short)(short) -17980;
            p34.chan5_scaled = (short)(short)7240;
            p34.chan6_scaled = (short)(short) -1110;
            p34.chan4_scaled = (short)(short)23746;
            SMP_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rssi == (byte)(byte)106);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)11150);
                Debug.Assert(pack.port == (byte)(byte)222);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)62790);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)41059);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)34792);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)24091);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)39573);
                Debug.Assert(pack.time_boot_ms == (uint)1345012935U);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)38407);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)64640);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.port = (byte)(byte)222;
            p35.rssi = (byte)(byte)106;
            p35.chan2_raw = (ushort)(ushort)11150;
            p35.chan7_raw = (ushort)(ushort)41059;
            p35.chan5_raw = (ushort)(ushort)64640;
            p35.chan4_raw = (ushort)(ushort)34792;
            p35.chan8_raw = (ushort)(ushort)39573;
            p35.chan6_raw = (ushort)(ushort)38407;
            p35.time_boot_ms = (uint)1345012935U;
            p35.chan3_raw = (ushort)(ushort)62790;
            p35.chan1_raw = (ushort)(ushort)24091;
            SMP_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)46461);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)28153);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)48683);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)21392);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)54928);
                Debug.Assert(pack.time_usec == (uint)3791276685U);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)17901);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)60322);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)14822);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)11667);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)18783);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)60933);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)25841);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)3677);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)5612);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)15347);
                Debug.Assert(pack.port == (byte)(byte)109);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)64033);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo11_raw_SET((ushort)(ushort)14822, PH) ;
            p36.servo15_raw_SET((ushort)(ushort)21392, PH) ;
            p36.servo6_raw = (ushort)(ushort)15347;
            p36.time_usec = (uint)3791276685U;
            p36.servo14_raw_SET((ushort)(ushort)28153, PH) ;
            p36.port = (byte)(byte)109;
            p36.servo8_raw = (ushort)(ushort)3677;
            p36.servo12_raw_SET((ushort)(ushort)5612, PH) ;
            p36.servo3_raw = (ushort)(ushort)25841;
            p36.servo13_raw_SET((ushort)(ushort)11667, PH) ;
            p36.servo16_raw_SET((ushort)(ushort)60933, PH) ;
            p36.servo5_raw = (ushort)(ushort)64033;
            p36.servo7_raw = (ushort)(ushort)17901;
            p36.servo2_raw = (ushort)(ushort)48683;
            p36.servo9_raw_SET((ushort)(ushort)46461, PH) ;
            p36.servo1_raw = (ushort)(ushort)54928;
            p36.servo10_raw_SET((ushort)(ushort)18783, PH) ;
            p36.servo4_raw = (ushort)(ushort)60322;
            SMP_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)149);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.end_index == (short)(short) -31969);
                Debug.Assert(pack.target_system == (byte)(byte)213);
                Debug.Assert(pack.start_index == (short)(short) -32433);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.target_system = (byte)(byte)213;
            p37.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p37.end_index = (short)(short) -31969;
            p37.target_component = (byte)(byte)149;
            p37.start_index = (short)(short) -32433;
            SMP_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end_index == (short)(short)21066);
                Debug.Assert(pack.start_index == (short)(short)25636);
                Debug.Assert(pack.target_system == (byte)(byte)228);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)67);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p38.end_index = (short)(short)21066;
            p38.target_component = (byte)(byte)67;
            p38.start_index = (short)(short)25636;
            p38.target_system = (byte)(byte)228;
            SMP_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.current == (byte)(byte)137);
                Debug.Assert(pack.z == (float)4.0059726E37F);
                Debug.Assert(pack.target_system == (byte)(byte)166);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH);
                Debug.Assert(pack.seq == (ushort)(ushort)13935);
                Debug.Assert(pack.param2 == (float) -9.781305E37F);
                Debug.Assert(pack.x == (float)1.1191515E37F);
                Debug.Assert(pack.autocontinue == (byte)(byte)244);
                Debug.Assert(pack.target_component == (byte)(byte)236);
                Debug.Assert(pack.param1 == (float)2.5167907E38F);
                Debug.Assert(pack.param4 == (float) -2.002102E38F);
                Debug.Assert(pack.param3 == (float) -3.385304E38F);
                Debug.Assert(pack.y == (float)8.61671E37F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.target_component = (byte)(byte)236;
            p39.seq = (ushort)(ushort)13935;
            p39.param1 = (float)2.5167907E38F;
            p39.z = (float)4.0059726E37F;
            p39.target_system = (byte)(byte)166;
            p39.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p39.autocontinue = (byte)(byte)244;
            p39.param4 = (float) -2.002102E38F;
            p39.param3 = (float) -3.385304E38F;
            p39.x = (float)1.1191515E37F;
            p39.param2 = (float) -9.781305E37F;
            p39.y = (float)8.61671E37F;
            p39.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p39.current = (byte)(byte)137;
            p39.command = MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH;
            SMP_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)19);
                Debug.Assert(pack.target_system == (byte)(byte)113);
                Debug.Assert(pack.seq == (ushort)(ushort)52461);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p40.seq = (ushort)(ushort)52461;
            p40.target_system = (byte)(byte)113;
            p40.target_component = (byte)(byte)19;
            SMP_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)54);
                Debug.Assert(pack.target_component == (byte)(byte)227);
                Debug.Assert(pack.target_system == (byte)(byte)46);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.seq = (ushort)(ushort)54;
            p41.target_system = (byte)(byte)46;
            p41.target_component = (byte)(byte)227;
            SMP_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)62235);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)62235;
            SMP_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)122);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)23);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p43.target_component = (byte)(byte)23;
            p43.target_system = (byte)(byte)122;
            SMP_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (ushort)(ushort)31728);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_component == (byte)(byte)177);
                Debug.Assert(pack.target_system == (byte)(byte)148);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p44.count = (ushort)(ushort)31728;
            p44.target_component = (byte)(byte)177;
            p44.target_system = (byte)(byte)148;
            SMP_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)224);
                Debug.Assert(pack.target_system == (byte)(byte)91);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_system = (byte)(byte)91;
            p45.target_component = (byte)(byte)224;
            p45.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            SMP_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)41027);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)41027;
            SMP_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_system == (byte)(byte)14);
                Debug.Assert(pack.type == MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED);
                Debug.Assert(pack.target_component == (byte)(byte)154);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p47.target_system = (byte)(byte)14;
            p47.type = MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED;
            p47.target_component = (byte)(byte)154;
            SMP_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (int)1287801970);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)2876560914933827728L);
                Debug.Assert(pack.altitude == (int)1097756334);
                Debug.Assert(pack.target_system == (byte)(byte)31);
                Debug.Assert(pack.latitude == (int)2025408864);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.latitude = (int)2025408864;
            p48.altitude = (int)1097756334;
            p48.target_system = (byte)(byte)31;
            p48.time_usec_SET((ulong)2876560914933827728L, PH) ;
            p48.longitude = (int)1287801970;
            SMP_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int)36713901);
                Debug.Assert(pack.longitude == (int) -1053991593);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)5754772510629278234L);
                Debug.Assert(pack.latitude == (int) -1738707273);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.time_usec_SET((ulong)5754772510629278234L, PH) ;
            p49.altitude = (int)36713901;
            p49.latitude = (int) -1738707273;
            p49.longitude = (int) -1053991593;
            SMP_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)24);
                Debug.Assert(pack.param_index == (short)(short)9758);
                Debug.Assert(pack.param_value0 == (float) -3.0349273E38F);
                Debug.Assert(pack.param_value_min == (float)1.7597655E38F);
                Debug.Assert(pack.scale == (float)2.0820392E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 2);
                Debug.Assert(pack.param_id_TRY(ph).Equals("Es"));
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)118);
                Debug.Assert(pack.param_value_max == (float) -3.3744666E38F);
                Debug.Assert(pack.target_component == (byte)(byte)161);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.param_index = (short)(short)9758;
            p50.target_component = (byte)(byte)161;
            p50.target_system = (byte)(byte)24;
            p50.param_value_max = (float) -3.3744666E38F;
            p50.param_value_min = (float)1.7597655E38F;
            p50.param_id_SET("Es", PH) ;
            p50.scale = (float)2.0820392E38F;
            p50.parameter_rc_channel_index = (byte)(byte)118;
            p50.param_value0 = (float) -3.0349273E38F;
            SMP_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)43);
                Debug.Assert(pack.target_component == (byte)(byte)210);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.seq == (ushort)(ushort)61768);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p51.target_system = (byte)(byte)43;
            p51.target_component = (byte)(byte)210;
            p51.seq = (ushort)(ushort)61768;
            SMP_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2y == (float) -1.6632197E38F);
                Debug.Assert(pack.p2z == (float) -2.9752157E38F);
                Debug.Assert(pack.target_component == (byte)(byte)6);
                Debug.Assert(pack.p1x == (float) -2.543238E37F);
                Debug.Assert(pack.target_system == (byte)(byte)22);
                Debug.Assert(pack.p1y == (float) -2.8736404E38F);
                Debug.Assert(pack.p2x == (float)3.141758E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.p1z == (float)3.088451E38F);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p1z = (float)3.088451E38F;
            p54.p1y = (float) -2.8736404E38F;
            p54.target_system = (byte)(byte)22;
            p54.p2x = (float)3.141758E38F;
            p54.target_component = (byte)(byte)6;
            p54.p2z = (float) -2.9752157E38F;
            p54.p1x = (float) -2.543238E37F;
            p54.frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p54.p2y = (float) -1.6632197E38F;
            SMP_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1y == (float)2.95159E38F);
                Debug.Assert(pack.p2z == (float) -2.9422067E38F);
                Debug.Assert(pack.p1z == (float)3.1794509E38F);
                Debug.Assert(pack.p1x == (float) -1.8428405E38F);
                Debug.Assert(pack.p2x == (float) -3.279409E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.p2y == (float) -1.8274181E38F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.frame = MAV_FRAME.MAV_FRAME_MISSION;
            p55.p1z = (float)3.1794509E38F;
            p55.p2x = (float) -3.279409E38F;
            p55.p2y = (float) -1.8274181E38F;
            p55.p1x = (float) -1.8428405E38F;
            p55.p2z = (float) -2.9422067E38F;
            p55.p1y = (float)2.95159E38F;
            SMP_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-1.2274479E38F, -1.7768614E38F, -1.1523265E38F, -2.9607562E38F, 3.3590145E38F, -2.5019717E38F, -2.5641367E38F, 2.019859E38F, -6.6851324E37F}));
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.2831481E38F, -1.3385864E38F, 1.1182073E37F, -5.352374E37F}));
                Debug.Assert(pack.pitchspeed == (float) -7.981641E37F);
                Debug.Assert(pack.yawspeed == (float)2.2162181E38F);
                Debug.Assert(pack.time_usec == (ulong)3553649971823118507L);
                Debug.Assert(pack.rollspeed == (float)2.9736438E38F);
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.covariance_SET(new float[] {-1.2274479E38F, -1.7768614E38F, -1.1523265E38F, -2.9607562E38F, 3.3590145E38F, -2.5019717E38F, -2.5641367E38F, 2.019859E38F, -6.6851324E37F}, 0) ;
            p61.q_SET(new float[] {-3.2831481E38F, -1.3385864E38F, 1.1182073E37F, -5.352374E37F}, 0) ;
            p61.yawspeed = (float)2.2162181E38F;
            p61.rollspeed = (float)2.9736438E38F;
            p61.pitchspeed = (float) -7.981641E37F;
            p61.time_usec = (ulong)3553649971823118507L;
            SMP_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt_error == (float) -1.0795242E38F);
                Debug.Assert(pack.aspd_error == (float)3.362008E38F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)31935);
                Debug.Assert(pack.nav_pitch == (float) -1.2671298E38F);
                Debug.Assert(pack.target_bearing == (short)(short)7353);
                Debug.Assert(pack.xtrack_error == (float) -2.6981884E38F);
                Debug.Assert(pack.nav_roll == (float) -1.9910854E38F);
                Debug.Assert(pack.nav_bearing == (short)(short) -28852);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_bearing = (short)(short) -28852;
            p62.xtrack_error = (float) -2.6981884E38F;
            p62.aspd_error = (float)3.362008E38F;
            p62.nav_pitch = (float) -1.2671298E38F;
            p62.alt_error = (float) -1.0795242E38F;
            p62.wp_dist = (ushort)(ushort)31935;
            p62.nav_roll = (float) -1.9910854E38F;
            p62.target_bearing = (short)(short)7353;
            SMP_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -513854239);
                Debug.Assert(pack.lat == (int)998467333);
                Debug.Assert(pack.time_usec == (ulong)3981350061777386843L);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {3.3188855E38F, 3.0906417E38F, 4.4446954E37F, 8.297438E37F, 1.0171782E38F, 3.355114E38F, 2.9745764E38F, 1.9627101E38F, 8.4613294E37F, 5.108182E37F, -7.554214E37F, 3.1749715E38F, -2.2525514E38F, 1.1832131E38F, 1.6532637E37F, 1.6187799E37F, 2.3024427E38F, -2.186175E38F, 1.243951E38F, 2.1681758E38F, -5.3377273E37F, 1.5274638E38F, -3.168521E38F, -6.277332E37F, 1.769477E38F, 2.9969388E38F, 1.6347375E38F, -3.0376805E38F, -1.7616699E38F, 1.9568479E38F, -5.7207667E37F, 6.846906E37F, -9.309464E37F, -6.6420906E37F, 8.092482E37F, 1.8802664E38F}));
                Debug.Assert(pack.relative_alt == (int)14333419);
                Debug.Assert(pack.vz == (float) -8.868607E37F);
                Debug.Assert(pack.vx == (float) -2.6139205E38F);
                Debug.Assert(pack.alt == (int)1224655309);
                Debug.Assert(pack.vy == (float) -3.1584644E38F);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.relative_alt = (int)14333419;
            p63.covariance_SET(new float[] {3.3188855E38F, 3.0906417E38F, 4.4446954E37F, 8.297438E37F, 1.0171782E38F, 3.355114E38F, 2.9745764E38F, 1.9627101E38F, 8.4613294E37F, 5.108182E37F, -7.554214E37F, 3.1749715E38F, -2.2525514E38F, 1.1832131E38F, 1.6532637E37F, 1.6187799E37F, 2.3024427E38F, -2.186175E38F, 1.243951E38F, 2.1681758E38F, -5.3377273E37F, 1.5274638E38F, -3.168521E38F, -6.277332E37F, 1.769477E38F, 2.9969388E38F, 1.6347375E38F, -3.0376805E38F, -1.7616699E38F, 1.9568479E38F, -5.7207667E37F, 6.846906E37F, -9.309464E37F, -6.6420906E37F, 8.092482E37F, 1.8802664E38F}, 0) ;
            p63.lat = (int)998467333;
            p63.vy = (float) -3.1584644E38F;
            p63.alt = (int)1224655309;
            p63.vz = (float) -8.868607E37F;
            p63.lon = (int) -513854239;
            p63.vx = (float) -2.6139205E38F;
            p63.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE;
            p63.time_usec = (ulong)3981350061777386843L;
            SMP_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)7296083632666462215L);
                Debug.Assert(pack.ay == (float) -1.3753429E38F);
                Debug.Assert(pack.vx == (float) -3.752791E37F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.6919366E38F, 5.928852E37F, -1.3597611E38F, 1.2589204E38F, -9.441754E37F, -2.7472324E36F, 2.0630592E38F, -1.1101665E38F, 1.2824976E37F, 1.9217076E38F, 1.9639642E38F, 2.2791952E38F, -3.539978E37F, -2.7672634E38F, -5.940688E36F, 7.8141674E37F, 8.985411E37F, 4.3359933E37F, 1.1325563E37F, 2.5517468E38F, 1.7542051E38F, 2.7386483E37F, -1.7108924E38F, 2.7131933E38F, -1.838038E38F, 6.4184684E37F, -1.1387731E38F, 2.5612223E38F, -3.6406745E37F, 2.5218361E38F, -1.3618522E38F, -2.355278E38F, -3.0250358E38F, 3.3314789E38F, 7.7788015E37F, 1.5223345E38F, 1.8384491E38F, -7.9104845E37F, 3.195314E38F, 3.0038597E38F, -2.9112002E38F, 1.6888728E38F, -1.4327412E38F, -5.4505077E37F, -2.3114631E38F}));
                Debug.Assert(pack.az == (float)1.8796358E38F);
                Debug.Assert(pack.y == (float)2.985568E38F);
                Debug.Assert(pack.vy == (float)1.3785608E38F);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
                Debug.Assert(pack.z == (float)2.0797348E38F);
                Debug.Assert(pack.vz == (float) -1.3120407E38F);
                Debug.Assert(pack.x == (float)5.0498003E37F);
                Debug.Assert(pack.ax == (float)1.4127158E38F);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.vy = (float)1.3785608E38F;
            p64.az = (float)1.8796358E38F;
            p64.ax = (float)1.4127158E38F;
            p64.covariance_SET(new float[] {1.6919366E38F, 5.928852E37F, -1.3597611E38F, 1.2589204E38F, -9.441754E37F, -2.7472324E36F, 2.0630592E38F, -1.1101665E38F, 1.2824976E37F, 1.9217076E38F, 1.9639642E38F, 2.2791952E38F, -3.539978E37F, -2.7672634E38F, -5.940688E36F, 7.8141674E37F, 8.985411E37F, 4.3359933E37F, 1.1325563E37F, 2.5517468E38F, 1.7542051E38F, 2.7386483E37F, -1.7108924E38F, 2.7131933E38F, -1.838038E38F, 6.4184684E37F, -1.1387731E38F, 2.5612223E38F, -3.6406745E37F, 2.5218361E38F, -1.3618522E38F, -2.355278E38F, -3.0250358E38F, 3.3314789E38F, 7.7788015E37F, 1.5223345E38F, 1.8384491E38F, -7.9104845E37F, 3.195314E38F, 3.0038597E38F, -2.9112002E38F, 1.6888728E38F, -1.4327412E38F, -5.4505077E37F, -2.3114631E38F}, 0) ;
            p64.y = (float)2.985568E38F;
            p64.vz = (float) -1.3120407E38F;
            p64.time_usec = (ulong)7296083632666462215L;
            p64.vx = (float) -3.752791E37F;
            p64.z = (float)2.0797348E38F;
            p64.ay = (float) -1.3753429E38F;
            p64.x = (float)5.0498003E37F;
            p64.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS;
            SMP_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)34715);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)19982);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)3846);
                Debug.Assert(pack.time_boot_ms == (uint)2189218440U);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)52473);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)16463);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)24099);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)21361);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)32433);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)26574);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)15291);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)32080);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)43481);
                Debug.Assert(pack.rssi == (byte)(byte)81);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)21710);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)40619);
                Debug.Assert(pack.chancount == (byte)(byte)207);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)42868);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)40525);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)15259);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)40098);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan12_raw = (ushort)(ushort)15259;
            p65.chan9_raw = (ushort)(ushort)21361;
            p65.chan1_raw = (ushort)(ushort)15291;
            p65.chan8_raw = (ushort)(ushort)40525;
            p65.chan16_raw = (ushort)(ushort)3846;
            p65.chan6_raw = (ushort)(ushort)16463;
            p65.chan5_raw = (ushort)(ushort)26574;
            p65.chan3_raw = (ushort)(ushort)24099;
            p65.chan10_raw = (ushort)(ushort)42868;
            p65.chancount = (byte)(byte)207;
            p65.chan13_raw = (ushort)(ushort)40619;
            p65.time_boot_ms = (uint)2189218440U;
            p65.chan11_raw = (ushort)(ushort)32433;
            p65.chan15_raw = (ushort)(ushort)52473;
            p65.chan17_raw = (ushort)(ushort)19982;
            p65.chan4_raw = (ushort)(ushort)43481;
            p65.chan7_raw = (ushort)(ushort)32080;
            p65.chan18_raw = (ushort)(ushort)40098;
            p65.chan14_raw = (ushort)(ushort)34715;
            p65.chan2_raw = (ushort)(ushort)21710;
            p65.rssi = (byte)(byte)81;
            SMP_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.req_stream_id == (byte)(byte)228);
                Debug.Assert(pack.target_system == (byte)(byte)222);
                Debug.Assert(pack.start_stop == (byte)(byte)180);
                Debug.Assert(pack.target_component == (byte)(byte)220);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)4913);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.start_stop = (byte)(byte)180;
            p66.target_component = (byte)(byte)220;
            p66.req_stream_id = (byte)(byte)228;
            p66.target_system = (byte)(byte)222;
            p66.req_message_rate = (ushort)(ushort)4913;
            SMP_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_rate == (ushort)(ushort)59693);
                Debug.Assert(pack.on_off == (byte)(byte)198);
                Debug.Assert(pack.stream_id == (byte)(byte)175);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.stream_id = (byte)(byte)175;
            p67.on_off = (byte)(byte)198;
            p67.message_rate = (ushort)(ushort)59693;
            SMP_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target == (byte)(byte)184);
                Debug.Assert(pack.r == (short)(short) -10997);
                Debug.Assert(pack.buttons == (ushort)(ushort)30296);
                Debug.Assert(pack.z == (short)(short) -16091);
                Debug.Assert(pack.y == (short)(short)18691);
                Debug.Assert(pack.x == (short)(short)32239);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.r = (short)(short) -10997;
            p69.buttons = (ushort)(ushort)30296;
            p69.z = (short)(short) -16091;
            p69.target = (byte)(byte)184;
            p69.y = (short)(short)18691;
            p69.x = (short)(short)32239;
            SMP_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)8992);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)32413);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)46762);
                Debug.Assert(pack.target_system == (byte)(byte)111);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)43737);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)13908);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)22701);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)32247);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)47745);
                Debug.Assert(pack.target_component == (byte)(byte)184);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan2_raw = (ushort)(ushort)32247;
            p70.chan5_raw = (ushort)(ushort)8992;
            p70.chan4_raw = (ushort)(ushort)46762;
            p70.chan1_raw = (ushort)(ushort)32413;
            p70.chan6_raw = (ushort)(ushort)43737;
            p70.chan7_raw = (ushort)(ushort)13908;
            p70.chan3_raw = (ushort)(ushort)47745;
            p70.target_system = (byte)(byte)111;
            p70.chan8_raw = (ushort)(ushort)22701;
            p70.target_component = (byte)(byte)184;
            SMP_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT);
                Debug.Assert(pack.x == (int)2133186193);
                Debug.Assert(pack.z == (float)3.2896592E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)121);
                Debug.Assert(pack.y == (int)894253227);
                Debug.Assert(pack.target_system == (byte)(byte)159);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.seq == (ushort)(ushort)55765);
                Debug.Assert(pack.target_component == (byte)(byte)180);
                Debug.Assert(pack.current == (byte)(byte)167);
                Debug.Assert(pack.param2 == (float)2.078558E38F);
                Debug.Assert(pack.param4 == (float) -8.383749E37F);
                Debug.Assert(pack.param1 == (float) -1.6962284E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_NED);
                Debug.Assert(pack.param3 == (float)1.8297891E38F);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.param1 = (float) -1.6962284E38F;
            p73.y = (int)894253227;
            p73.param2 = (float)2.078558E38F;
            p73.frame = MAV_FRAME.MAV_FRAME_BODY_NED;
            p73.param3 = (float)1.8297891E38F;
            p73.autocontinue = (byte)(byte)121;
            p73.current = (byte)(byte)167;
            p73.seq = (ushort)(ushort)55765;
            p73.target_system = (byte)(byte)159;
            p73.x = (int)2133186193;
            p73.z = (float)3.2896592E38F;
            p73.target_component = (byte)(byte)180;
            p73.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p73.param4 = (float) -8.383749E37F;
            p73.command = MAV_CMD.MAV_CMD_NAV_FENCE_RETURN_POINT;
            SMP_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (float)9.665922E37F);
                Debug.Assert(pack.groundspeed == (float)3.2068532E38F);
                Debug.Assert(pack.heading == (short)(short) -17411);
                Debug.Assert(pack.climb == (float)6.152224E37F);
                Debug.Assert(pack.throttle == (ushort)(ushort)19093);
                Debug.Assert(pack.airspeed == (float) -2.6032487E38F);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.climb = (float)6.152224E37F;
            p74.airspeed = (float) -2.6032487E38F;
            p74.alt = (float)9.665922E37F;
            p74.heading = (short)(short) -17411;
            p74.groundspeed = (float)3.2068532E38F;
            p74.throttle = (ushort)(ushort)19093;
            SMP_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.target_system == (byte)(byte)242);
                Debug.Assert(pack.z == (float) -3.1442941E38F);
                Debug.Assert(pack.current == (byte)(byte)226);
                Debug.Assert(pack.param3 == (float) -1.2159286E38F);
                Debug.Assert(pack.param1 == (float) -5.6346557E37F);
                Debug.Assert(pack.x == (int) -66418381);
                Debug.Assert(pack.param4 == (float)1.6433502E38F);
                Debug.Assert(pack.param2 == (float) -1.5384954E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)247);
                Debug.Assert(pack.y == (int) -1752209127);
                Debug.Assert(pack.target_component == (byte)(byte)79);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.current = (byte)(byte)226;
            p75.command = MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT;
            p75.param4 = (float)1.6433502E38F;
            p75.target_system = (byte)(byte)242;
            p75.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p75.param2 = (float) -1.5384954E38F;
            p75.x = (int) -66418381;
            p75.param1 = (float) -5.6346557E37F;
            p75.y = (int) -1752209127;
            p75.target_component = (byte)(byte)79;
            p75.autocontinue = (byte)(byte)247;
            p75.z = (float) -3.1442941E38F;
            p75.param3 = (float) -1.2159286E38F;
            SMP_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param2 == (float)1.0694599E38F);
                Debug.Assert(pack.target_system == (byte)(byte)206);
                Debug.Assert(pack.param4 == (float)3.8365998E37F);
                Debug.Assert(pack.param5 == (float) -2.4401812E38F);
                Debug.Assert(pack.param3 == (float)5.487187E37F);
                Debug.Assert(pack.target_component == (byte)(byte)50);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_RALLY_POINT);
                Debug.Assert(pack.confirmation == (byte)(byte)171);
                Debug.Assert(pack.param6 == (float) -1.111926E38F);
                Debug.Assert(pack.param1 == (float) -4.9866424E37F);
                Debug.Assert(pack.param7 == (float) -3.0301462E38F);
            };
            GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.param5 = (float) -2.4401812E38F;
            p76.target_system = (byte)(byte)206;
            p76.param3 = (float)5.487187E37F;
            p76.param7 = (float) -3.0301462E38F;
            p76.param6 = (float) -1.111926E38F;
            p76.target_component = (byte)(byte)50;
            p76.param1 = (float) -4.9866424E37F;
            p76.confirmation = (byte)(byte)171;
            p76.param2 = (float)1.0694599E38F;
            p76.param4 = (float)3.8365998E37F;
            p76.command = MAV_CMD.MAV_CMD_NAV_RALLY_POINT;
            CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)232);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)41);
                Debug.Assert(pack.result == MAV_RESULT.MAV_RESULT_ACCEPTED);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)171);
                Debug.Assert(pack.result_param2_TRY(ph) == (int) -226068676);
            };
            GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.result = MAV_RESULT.MAV_RESULT_ACCEPTED;
            p77.command = MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION;
            p77.target_system_SET((byte)(byte)171, PH) ;
            p77.target_component_SET((byte)(byte)232, PH) ;
            p77.progress_SET((byte)(byte)41, PH) ;
            p77.result_param2_SET((int) -226068676, PH) ;
            CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)3.3960761E38F);
                Debug.Assert(pack.roll == (float)6.645314E37F);
                Debug.Assert(pack.mode_switch == (byte)(byte)234);
                Debug.Assert(pack.time_boot_ms == (uint)2938144918U);
                Debug.Assert(pack.pitch == (float)9.266159E37F);
                Debug.Assert(pack.thrust == (float)2.6813901E38F);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)93);
            };
            GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.time_boot_ms = (uint)2938144918U;
            p81.manual_override_switch = (byte)(byte)93;
            p81.thrust = (float)2.6813901E38F;
            p81.roll = (float)6.645314E37F;
            p81.pitch = (float)9.266159E37F;
            p81.yaw = (float)3.3960761E38F;
            p81.mode_switch = (byte)(byte)234;
            CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.thrust == (float) -3.2513617E38F);
                Debug.Assert(pack.body_roll_rate == (float) -2.421258E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.4808431E37F, -9.040835E37F, 2.7167233E38F, -3.1878244E38F}));
                Debug.Assert(pack.type_mask == (byte)(byte)30);
                Debug.Assert(pack.time_boot_ms == (uint)4125803415U);
                Debug.Assert(pack.body_pitch_rate == (float) -3.0414034E37F);
                Debug.Assert(pack.body_yaw_rate == (float) -1.5637172E38F);
                Debug.Assert(pack.target_system == (byte)(byte)20);
                Debug.Assert(pack.target_component == (byte)(byte)29);
            };
            GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.target_system = (byte)(byte)20;
            p82.time_boot_ms = (uint)4125803415U;
            p82.q_SET(new float[] {1.4808431E37F, -9.040835E37F, 2.7167233E38F, -3.1878244E38F}, 0) ;
            p82.body_roll_rate = (float) -2.421258E38F;
            p82.body_pitch_rate = (float) -3.0414034E37F;
            p82.thrust = (float) -3.2513617E38F;
            p82.body_yaw_rate = (float) -1.5637172E38F;
            p82.target_component = (byte)(byte)29;
            p82.type_mask = (byte)(byte)30;
            CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.2449541E38F, -2.441175E38F, 7.413751E37F, 1.9366242E37F}));
                Debug.Assert(pack.type_mask == (byte)(byte)53);
                Debug.Assert(pack.thrust == (float)1.8874322E38F);
                Debug.Assert(pack.body_pitch_rate == (float)2.3305756E38F);
                Debug.Assert(pack.body_roll_rate == (float)8.1613546E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1767164973U);
                Debug.Assert(pack.body_yaw_rate == (float)6.4468886E37F);
            };
            GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.type_mask = (byte)(byte)53;
            p83.q_SET(new float[] {3.2449541E38F, -2.441175E38F, 7.413751E37F, 1.9366242E37F}, 0) ;
            p83.body_roll_rate = (float)8.1613546E37F;
            p83.time_boot_ms = (uint)1767164973U;
            p83.body_yaw_rate = (float)6.4468886E37F;
            p83.body_pitch_rate = (float)2.3305756E38F;
            p83.thrust = (float)1.8874322E38F;
            CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)193);
                Debug.Assert(pack.vy == (float) -2.4319212E38F);
                Debug.Assert(pack.z == (float)1.6415245E38F);
                Debug.Assert(pack.x == (float)1.4214773E38F);
                Debug.Assert(pack.vx == (float)1.0051564E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)29190);
                Debug.Assert(pack.afz == (float)2.6977203E38F);
                Debug.Assert(pack.yaw_rate == (float) -2.6142375E38F);
                Debug.Assert(pack.target_system == (byte)(byte)216);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.yaw == (float) -8.4100585E37F);
                Debug.Assert(pack.afy == (float)3.2602142E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2000564077U);
                Debug.Assert(pack.y == (float)2.2811945E38F);
                Debug.Assert(pack.vz == (float) -2.1034721E37F);
                Debug.Assert(pack.afx == (float) -2.7761579E38F);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p84.afy = (float)3.2602142E38F;
            p84.vy = (float) -2.4319212E38F;
            p84.x = (float)1.4214773E38F;
            p84.yaw = (float) -8.4100585E37F;
            p84.target_component = (byte)(byte)193;
            p84.z = (float)1.6415245E38F;
            p84.time_boot_ms = (uint)2000564077U;
            p84.vz = (float) -2.1034721E37F;
            p84.yaw_rate = (float) -2.6142375E38F;
            p84.vx = (float)1.0051564E38F;
            p84.y = (float)2.2811945E38F;
            p84.target_system = (byte)(byte)216;
            p84.afz = (float)2.6977203E38F;
            p84.afx = (float) -2.7761579E38F;
            p84.type_mask = (ushort)(ushort)29190;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)40);
                Debug.Assert(pack.lon_int == (int)1949333989);
                Debug.Assert(pack.vy == (float)1.0629649E38F);
                Debug.Assert(pack.lat_int == (int) -970226684);
                Debug.Assert(pack.yaw_rate == (float)1.6966998E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.vz == (float) -7.986721E37F);
                Debug.Assert(pack.alt == (float)2.0944845E38F);
                Debug.Assert(pack.target_system == (byte)(byte)160);
                Debug.Assert(pack.type_mask == (ushort)(ushort)54813);
                Debug.Assert(pack.afz == (float) -3.0024158E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2555381169U);
                Debug.Assert(pack.yaw == (float) -1.2051357E38F);
                Debug.Assert(pack.afy == (float)2.0357521E38F);
                Debug.Assert(pack.vx == (float) -3.2761633E38F);
                Debug.Assert(pack.afx == (float) -1.7958331E38F);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.vy = (float)1.0629649E38F;
            p86.afx = (float) -1.7958331E38F;
            p86.afy = (float)2.0357521E38F;
            p86.alt = (float)2.0944845E38F;
            p86.vx = (float) -3.2761633E38F;
            p86.afz = (float) -3.0024158E38F;
            p86.yaw_rate = (float)1.6966998E38F;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p86.type_mask = (ushort)(ushort)54813;
            p86.time_boot_ms = (uint)2555381169U;
            p86.lon_int = (int)1949333989;
            p86.vz = (float) -7.986721E37F;
            p86.yaw = (float) -1.2051357E38F;
            p86.target_system = (byte)(byte)160;
            p86.target_component = (byte)(byte)40;
            p86.lat_int = (int) -970226684;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afx == (float) -3.0186904E38F);
                Debug.Assert(pack.alt == (float) -3.5718981E37F);
                Debug.Assert(pack.lat_int == (int)629654762);
                Debug.Assert(pack.type_mask == (ushort)(ushort)28544);
                Debug.Assert(pack.vx == (float) -1.4264963E38F);
                Debug.Assert(pack.lon_int == (int) -694479815);
                Debug.Assert(pack.yaw_rate == (float) -2.7573427E38F);
                Debug.Assert(pack.yaw == (float) -1.822879E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.vz == (float)2.1783918E38F);
                Debug.Assert(pack.vy == (float) -2.2146049E38F);
                Debug.Assert(pack.afy == (float) -3.0012066E38F);
                Debug.Assert(pack.afz == (float) -2.8567896E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1312325871U);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.yaw_rate = (float) -2.7573427E38F;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_MISSION;
            p87.type_mask = (ushort)(ushort)28544;
            p87.afz = (float) -2.8567896E38F;
            p87.lat_int = (int)629654762;
            p87.time_boot_ms = (uint)1312325871U;
            p87.vx = (float) -1.4264963E38F;
            p87.yaw = (float) -1.822879E38F;
            p87.afx = (float) -3.0186904E38F;
            p87.alt = (float) -3.5718981E37F;
            p87.vz = (float)2.1783918E38F;
            p87.afy = (float) -3.0012066E38F;
            p87.vy = (float) -2.2146049E38F;
            p87.lon_int = (int) -694479815;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)2.4748497E38F);
                Debug.Assert(pack.yaw == (float)1.895166E38F);
                Debug.Assert(pack.roll == (float)1.486272E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1901936700U);
                Debug.Assert(pack.y == (float)2.4214282E38F);
                Debug.Assert(pack.pitch == (float)1.4888628E38F);
                Debug.Assert(pack.x == (float) -2.694584E38F);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.x = (float) -2.694584E38F;
            p89.pitch = (float)1.4888628E38F;
            p89.time_boot_ms = (uint)1901936700U;
            p89.y = (float)2.4214282E38F;
            p89.yaw = (float)1.895166E38F;
            p89.z = (float)2.4748497E38F;
            p89.roll = (float)1.486272E38F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (short)(short) -8450);
                Debug.Assert(pack.xacc == (short)(short) -6661);
                Debug.Assert(pack.alt == (int) -1723267497);
                Debug.Assert(pack.roll == (float)2.0628069E37F);
                Debug.Assert(pack.time_usec == (ulong)4354620599062163467L);
                Debug.Assert(pack.rollspeed == (float) -3.889436E37F);
                Debug.Assert(pack.zacc == (short)(short) -25957);
                Debug.Assert(pack.pitch == (float) -3.3606953E38F);
                Debug.Assert(pack.vz == (short)(short) -28611);
                Debug.Assert(pack.yacc == (short)(short) -24489);
                Debug.Assert(pack.lon == (int)931350998);
                Debug.Assert(pack.yaw == (float)3.239962E38F);
                Debug.Assert(pack.lat == (int) -1945481888);
                Debug.Assert(pack.vy == (short)(short)18112);
                Debug.Assert(pack.yawspeed == (float) -2.1941355E38F);
                Debug.Assert(pack.pitchspeed == (float) -3.3963396E38F);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.xacc = (short)(short) -6661;
            p90.pitch = (float) -3.3606953E38F;
            p90.alt = (int) -1723267497;
            p90.lat = (int) -1945481888;
            p90.vx = (short)(short) -8450;
            p90.vy = (short)(short)18112;
            p90.time_usec = (ulong)4354620599062163467L;
            p90.pitchspeed = (float) -3.3963396E38F;
            p90.zacc = (short)(short) -25957;
            p90.lon = (int)931350998;
            p90.yacc = (short)(short) -24489;
            p90.yawspeed = (float) -2.1941355E38F;
            p90.vz = (short)(short) -28611;
            p90.rollspeed = (float) -3.889436E37F;
            p90.yaw = (float)3.239962E38F;
            p90.roll = (float)2.0628069E37F;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw_rudder == (float) -4.010439E37F);
                Debug.Assert(pack.time_usec == (ulong)1334621750025292062L);
                Debug.Assert(pack.nav_mode == (byte)(byte)182);
                Debug.Assert(pack.roll_ailerons == (float) -1.283516E38F);
                Debug.Assert(pack.aux3 == (float)1.5030622E38F);
                Debug.Assert(pack.pitch_elevator == (float)5.4399E37F);
                Debug.Assert(pack.throttle == (float)1.8028123E38F);
                Debug.Assert(pack.aux1 == (float) -2.3875258E38F);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_MANUAL_ARMED);
                Debug.Assert(pack.aux4 == (float) -3.3917485E38F);
                Debug.Assert(pack.aux2 == (float)9.032306E37F);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.aux3 = (float)1.5030622E38F;
            p91.aux4 = (float) -3.3917485E38F;
            p91.nav_mode = (byte)(byte)182;
            p91.aux2 = (float)9.032306E37F;
            p91.mode = MAV_MODE.MAV_MODE_MANUAL_ARMED;
            p91.yaw_rudder = (float) -4.010439E37F;
            p91.throttle = (float)1.8028123E38F;
            p91.roll_ailerons = (float) -1.283516E38F;
            p91.time_usec = (ulong)1334621750025292062L;
            p91.pitch_elevator = (float)5.4399E37F;
            p91.aux1 = (float) -2.3875258E38F;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)969);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)29038);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)46848);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)36509);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)29239);
                Debug.Assert(pack.time_usec == (ulong)3384095830091386969L);
                Debug.Assert(pack.rssi == (byte)(byte)193);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)27712);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)7249);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)18201);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)130);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)63177);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)65314);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)22202);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan1_raw = (ushort)(ushort)969;
            p92.rssi = (byte)(byte)193;
            p92.chan5_raw = (ushort)(ushort)29038;
            p92.chan8_raw = (ushort)(ushort)130;
            p92.chan4_raw = (ushort)(ushort)63177;
            p92.chan2_raw = (ushort)(ushort)27712;
            p92.chan3_raw = (ushort)(ushort)7249;
            p92.chan6_raw = (ushort)(ushort)22202;
            p92.chan7_raw = (ushort)(ushort)65314;
            p92.chan10_raw = (ushort)(ushort)46848;
            p92.chan11_raw = (ushort)(ushort)29239;
            p92.chan9_raw = (ushort)(ushort)36509;
            p92.time_usec = (ulong)3384095830091386969L;
            p92.chan12_raw = (ushort)(ushort)18201;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_AUTO_ARMED);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-4.0884323E37F, -2.5998522E38F, 8.629123E37F, -1.5594935E38F, -2.9524716E38F, -1.0435244E38F, 1.3426906E38F, 5.977611E37F, 2.5177082E38F, -2.9522658E38F, -1.1945407E38F, 2.1436258E38F, 2.1546872E38F, 3.6655646E37F, 1.817584E38F, 8.778207E37F}));
                Debug.Assert(pack.flags == (ulong)5799663447220576196L);
                Debug.Assert(pack.time_usec == (ulong)8435660386144912141L);
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.controls_SET(new float[] {-4.0884323E37F, -2.5998522E38F, 8.629123E37F, -1.5594935E38F, -2.9524716E38F, -1.0435244E38F, 1.3426906E38F, 5.977611E37F, 2.5177082E38F, -2.9522658E38F, -1.1945407E38F, 2.1436258E38F, 2.1546872E38F, 3.6655646E37F, 1.817584E38F, 8.778207E37F}, 0) ;
            p93.mode = MAV_MODE.MAV_MODE_AUTO_ARMED;
            p93.flags = (ulong)5799663447220576196L;
            p93.time_usec = (ulong)8435660386144912141L;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ground_distance == (float)5.4483045E37F);
                Debug.Assert(pack.quality == (byte)(byte)146);
                Debug.Assert(pack.time_usec == (ulong)6318106183510002083L);
                Debug.Assert(pack.flow_comp_m_x == (float)1.2497696E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)31);
                Debug.Assert(pack.flow_comp_m_y == (float)2.0643635E38F);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)1.4791812E37F);
                Debug.Assert(pack.flow_x == (short)(short) -10475);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float)3.1587908E38F);
                Debug.Assert(pack.flow_y == (short)(short) -28849);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_rate_x_SET((float)1.4791812E37F, PH) ;
            p100.flow_rate_y_SET((float)3.1587908E38F, PH) ;
            p100.flow_comp_m_x = (float)1.2497696E38F;
            p100.time_usec = (ulong)6318106183510002083L;
            p100.flow_comp_m_y = (float)2.0643635E38F;
            p100.flow_y = (short)(short) -28849;
            p100.flow_x = (short)(short) -10475;
            p100.quality = (byte)(byte)146;
            p100.ground_distance = (float)5.4483045E37F;
            p100.sensor_id = (byte)(byte)31;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)2.208393E38F);
                Debug.Assert(pack.yaw == (float) -3.0598865E38F);
                Debug.Assert(pack.roll == (float)3.0175656E38F);
                Debug.Assert(pack.pitch == (float) -2.8122683E38F);
                Debug.Assert(pack.x == (float) -3.2181586E38F);
                Debug.Assert(pack.usec == (ulong)2171691075050586469L);
                Debug.Assert(pack.y == (float)1.0269763E38F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.x = (float) -3.2181586E38F;
            p101.usec = (ulong)2171691075050586469L;
            p101.yaw = (float) -3.0598865E38F;
            p101.pitch = (float) -2.8122683E38F;
            p101.roll = (float)3.0175656E38F;
            p101.z = (float)2.208393E38F;
            p101.y = (float)1.0269763E38F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)2.5889255E38F);
                Debug.Assert(pack.x == (float) -1.8633006E38F);
                Debug.Assert(pack.y == (float) -3.5239133E37F);
                Debug.Assert(pack.yaw == (float)2.4479132E38F);
                Debug.Assert(pack.pitch == (float)2.3098166E38F);
                Debug.Assert(pack.usec == (ulong)8191665729040605119L);
                Debug.Assert(pack.roll == (float)3.3017134E38F);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)8191665729040605119L;
            p102.x = (float) -1.8633006E38F;
            p102.y = (float) -3.5239133E37F;
            p102.roll = (float)3.3017134E38F;
            p102.z = (float)2.5889255E38F;
            p102.yaw = (float)2.4479132E38F;
            p102.pitch = (float)2.3098166E38F;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)2.5542369E38F);
                Debug.Assert(pack.y == (float) -1.6673683E38F);
                Debug.Assert(pack.usec == (ulong)3244605955430145858L);
                Debug.Assert(pack.z == (float)1.041965E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.z = (float)1.041965E38F;
            p103.usec = (ulong)3244605955430145858L;
            p103.x = (float)2.5542369E38F;
            p103.y = (float) -1.6673683E38F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -4.7715474E37F);
                Debug.Assert(pack.roll == (float)1.6215973E38F);
                Debug.Assert(pack.pitch == (float)2.6272205E38F);
                Debug.Assert(pack.z == (float) -1.2611363E37F);
                Debug.Assert(pack.usec == (ulong)3133370547333820803L);
                Debug.Assert(pack.y == (float) -1.4029781E38F);
                Debug.Assert(pack.x == (float) -2.7022808E38F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.roll = (float)1.6215973E38F;
            p104.z = (float) -1.2611363E37F;
            p104.x = (float) -2.7022808E38F;
            p104.usec = (ulong)3133370547333820803L;
            p104.pitch = (float)2.6272205E38F;
            p104.yaw = (float) -4.7715474E37F;
            p104.y = (float) -1.4029781E38F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (float)3.6146453E37F);
                Debug.Assert(pack.zgyro == (float) -2.7153335E38F);
                Debug.Assert(pack.ygyro == (float) -2.6997388E38F);
                Debug.Assert(pack.zmag == (float)1.0395104E38F);
                Debug.Assert(pack.xgyro == (float)1.2287351E37F);
                Debug.Assert(pack.temperature == (float) -1.5303467E38F);
                Debug.Assert(pack.diff_pressure == (float)7.4978044E37F);
                Debug.Assert(pack.abs_pressure == (float)3.0091847E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)54757);
                Debug.Assert(pack.xmag == (float) -1.3210645E38F);
                Debug.Assert(pack.time_usec == (ulong)2392949204721021412L);
                Debug.Assert(pack.ymag == (float) -1.6881959E38F);
                Debug.Assert(pack.yacc == (float)7.9282767E36F);
                Debug.Assert(pack.pressure_alt == (float)2.8931628E38F);
                Debug.Assert(pack.zacc == (float) -2.22676E38F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.xacc = (float)3.6146453E37F;
            p105.zgyro = (float) -2.7153335E38F;
            p105.fields_updated = (ushort)(ushort)54757;
            p105.abs_pressure = (float)3.0091847E38F;
            p105.ymag = (float) -1.6881959E38F;
            p105.temperature = (float) -1.5303467E38F;
            p105.ygyro = (float) -2.6997388E38F;
            p105.time_usec = (ulong)2392949204721021412L;
            p105.xmag = (float) -1.3210645E38F;
            p105.diff_pressure = (float)7.4978044E37F;
            p105.yacc = (float)7.9282767E36F;
            p105.pressure_alt = (float)2.8931628E38F;
            p105.zacc = (float) -2.22676E38F;
            p105.xgyro = (float)1.2287351E37F;
            p105.zmag = (float)1.0395104E38F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_xgyro == (float) -1.1492128E38F);
                Debug.Assert(pack.integration_time_us == (uint)3054272576U);
                Debug.Assert(pack.quality == (byte)(byte)219);
                Debug.Assert(pack.sensor_id == (byte)(byte)69);
                Debug.Assert(pack.temperature == (short)(short) -31896);
                Debug.Assert(pack.distance == (float)9.887454E37F);
                Debug.Assert(pack.integrated_ygyro == (float)3.191393E37F);
                Debug.Assert(pack.integrated_x == (float) -1.4589204E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)2188176291U);
                Debug.Assert(pack.integrated_y == (float)2.074789E38F);
                Debug.Assert(pack.time_usec == (ulong)967970985779588863L);
                Debug.Assert(pack.integrated_zgyro == (float) -6.4796087E37F);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_delta_distance_us = (uint)2188176291U;
            p106.quality = (byte)(byte)219;
            p106.integrated_xgyro = (float) -1.1492128E38F;
            p106.integrated_zgyro = (float) -6.4796087E37F;
            p106.integration_time_us = (uint)3054272576U;
            p106.integrated_ygyro = (float)3.191393E37F;
            p106.integrated_y = (float)2.074789E38F;
            p106.distance = (float)9.887454E37F;
            p106.sensor_id = (byte)(byte)69;
            p106.time_usec = (ulong)967970985779588863L;
            p106.integrated_x = (float) -1.4589204E38F;
            p106.temperature = (short)(short) -31896;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (float) -1.6427555E38F);
                Debug.Assert(pack.yacc == (float)3.0666086E37F);
                Debug.Assert(pack.zacc == (float) -2.735148E38F);
                Debug.Assert(pack.ygyro == (float)3.0846537E38F);
                Debug.Assert(pack.time_usec == (ulong)3943973024488754839L);
                Debug.Assert(pack.abs_pressure == (float) -9.566974E37F);
                Debug.Assert(pack.zgyro == (float) -1.5047974E38F);
                Debug.Assert(pack.diff_pressure == (float)3.275346E38F);
                Debug.Assert(pack.zmag == (float) -1.5925328E38F);
                Debug.Assert(pack.xacc == (float) -3.389878E38F);
                Debug.Assert(pack.fields_updated == (uint)1104777956U);
                Debug.Assert(pack.temperature == (float) -3.224068E38F);
                Debug.Assert(pack.pressure_alt == (float)8.654795E37F);
                Debug.Assert(pack.ymag == (float)2.5409768E38F);
                Debug.Assert(pack.xmag == (float) -3.3107056E37F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.temperature = (float) -3.224068E38F;
            p107.zacc = (float) -2.735148E38F;
            p107.zmag = (float) -1.5925328E38F;
            p107.xgyro = (float) -1.6427555E38F;
            p107.pressure_alt = (float)8.654795E37F;
            p107.fields_updated = (uint)1104777956U;
            p107.ymag = (float)2.5409768E38F;
            p107.ygyro = (float)3.0846537E38F;
            p107.xacc = (float) -3.389878E38F;
            p107.xmag = (float) -3.3107056E37F;
            p107.zgyro = (float) -1.5047974E38F;
            p107.yacc = (float)3.0666086E37F;
            p107.time_usec = (ulong)3943973024488754839L;
            p107.abs_pressure = (float) -9.566974E37F;
            p107.diff_pressure = (float)3.275346E38F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q3 == (float)2.8179898E37F);
                Debug.Assert(pack.ve == (float) -1.7189819E38F);
                Debug.Assert(pack.xgyro == (float)3.016089E38F);
                Debug.Assert(pack.lat == (float)2.9322614E38F);
                Debug.Assert(pack.q2 == (float) -9.230121E37F);
                Debug.Assert(pack.std_dev_horz == (float)1.8066433E38F);
                Debug.Assert(pack.lon == (float)7.759188E37F);
                Debug.Assert(pack.yaw == (float)2.536161E37F);
                Debug.Assert(pack.pitch == (float) -2.6059732E38F);
                Debug.Assert(pack.alt == (float) -3.3895174E37F);
                Debug.Assert(pack.vn == (float)9.970537E37F);
                Debug.Assert(pack.std_dev_vert == (float) -2.5794923E38F);
                Debug.Assert(pack.q4 == (float) -4.2116013E37F);
                Debug.Assert(pack.zacc == (float) -2.7038638E38F);
                Debug.Assert(pack.zgyro == (float)3.1851368E38F);
                Debug.Assert(pack.xacc == (float) -3.1544388E38F);
                Debug.Assert(pack.ygyro == (float)8.029413E37F);
                Debug.Assert(pack.q1 == (float) -3.6047866E37F);
                Debug.Assert(pack.roll == (float)2.512917E38F);
                Debug.Assert(pack.vd == (float) -3.012671E38F);
                Debug.Assert(pack.yacc == (float) -3.2376873E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.alt = (float) -3.3895174E37F;
            p108.zacc = (float) -2.7038638E38F;
            p108.std_dev_vert = (float) -2.5794923E38F;
            p108.q4 = (float) -4.2116013E37F;
            p108.xgyro = (float)3.016089E38F;
            p108.lat = (float)2.9322614E38F;
            p108.xacc = (float) -3.1544388E38F;
            p108.vd = (float) -3.012671E38F;
            p108.std_dev_horz = (float)1.8066433E38F;
            p108.ve = (float) -1.7189819E38F;
            p108.pitch = (float) -2.6059732E38F;
            p108.q1 = (float) -3.6047866E37F;
            p108.q2 = (float) -9.230121E37F;
            p108.lon = (float)7.759188E37F;
            p108.ygyro = (float)8.029413E37F;
            p108.yaw = (float)2.536161E37F;
            p108.zgyro = (float)3.1851368E38F;
            p108.yacc = (float) -3.2376873E38F;
            p108.q3 = (float)2.8179898E37F;
            p108.vn = (float)9.970537E37F;
            p108.roll = (float)2.512917E38F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fixed_ == (ushort)(ushort)56590);
                Debug.Assert(pack.noise == (byte)(byte)211);
                Debug.Assert(pack.txbuf == (byte)(byte)196);
                Debug.Assert(pack.remnoise == (byte)(byte)254);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)28945);
                Debug.Assert(pack.remrssi == (byte)(byte)70);
                Debug.Assert(pack.rssi == (byte)(byte)59);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.noise = (byte)(byte)211;
            p109.remrssi = (byte)(byte)70;
            p109.rssi = (byte)(byte)59;
            p109.fixed_ = (ushort)(ushort)56590;
            p109.remnoise = (byte)(byte)254;
            p109.txbuf = (byte)(byte)196;
            p109.rxerrors = (ushort)(ushort)28945;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)10, (byte)83, (byte)82, (byte)210, (byte)87, (byte)51, (byte)58, (byte)137, (byte)72, (byte)44, (byte)129, (byte)8, (byte)100, (byte)89, (byte)208, (byte)62, (byte)197, (byte)69, (byte)104, (byte)220, (byte)205, (byte)115, (byte)207, (byte)0, (byte)200, (byte)131, (byte)240, (byte)133, (byte)192, (byte)8, (byte)246, (byte)70, (byte)32, (byte)209, (byte)81, (byte)46, (byte)151, (byte)4, (byte)240, (byte)148, (byte)75, (byte)150, (byte)188, (byte)189, (byte)56, (byte)120, (byte)141, (byte)154, (byte)238, (byte)239, (byte)145, (byte)56, (byte)118, (byte)109, (byte)74, (byte)47, (byte)101, (byte)128, (byte)118, (byte)218, (byte)164, (byte)72, (byte)133, (byte)164, (byte)123, (byte)242, (byte)29, (byte)48, (byte)254, (byte)164, (byte)56, (byte)54, (byte)214, (byte)13, (byte)83, (byte)18, (byte)251, (byte)144, (byte)57, (byte)222, (byte)216, (byte)60, (byte)78, (byte)179, (byte)175, (byte)5, (byte)86, (byte)201, (byte)199, (byte)112, (byte)237, (byte)43, (byte)4, (byte)83, (byte)87, (byte)230, (byte)205, (byte)128, (byte)43, (byte)155, (byte)68, (byte)223, (byte)78, (byte)19, (byte)18, (byte)67, (byte)60, (byte)140, (byte)58, (byte)57, (byte)205, (byte)210, (byte)213, (byte)145, (byte)117, (byte)201, (byte)65, (byte)46, (byte)201, (byte)222, (byte)131, (byte)185, (byte)229, (byte)49, (byte)242, (byte)70, (byte)21, (byte)20, (byte)76, (byte)214, (byte)46, (byte)132, (byte)109, (byte)1, (byte)122, (byte)119, (byte)124, (byte)152, (byte)63, (byte)95, (byte)184, (byte)166, (byte)53, (byte)199, (byte)203, (byte)91, (byte)91, (byte)195, (byte)37, (byte)231, (byte)106, (byte)232, (byte)167, (byte)178, (byte)182, (byte)177, (byte)202, (byte)1, (byte)192, (byte)225, (byte)69, (byte)100, (byte)98, (byte)137, (byte)120, (byte)101, (byte)26, (byte)52, (byte)203, (byte)217, (byte)239, (byte)232, (byte)98, (byte)250, (byte)225, (byte)211, (byte)105, (byte)142, (byte)23, (byte)139, (byte)223, (byte)40, (byte)23, (byte)17, (byte)228, (byte)241, (byte)143, (byte)137, (byte)109, (byte)210, (byte)116, (byte)143, (byte)157, (byte)44, (byte)3, (byte)68, (byte)212, (byte)136, (byte)215, (byte)252, (byte)157, (byte)3, (byte)82, (byte)51, (byte)155, (byte)240, (byte)166, (byte)130, (byte)246, (byte)129, (byte)197, (byte)214, (byte)234, (byte)59, (byte)178, (byte)242, (byte)174, (byte)15, (byte)169, (byte)154, (byte)99, (byte)234, (byte)29, (byte)131, (byte)235, (byte)152, (byte)238, (byte)138, (byte)49, (byte)61, (byte)7, (byte)83, (byte)153, (byte)179, (byte)49, (byte)178, (byte)191, (byte)38, (byte)104, (byte)99, (byte)135, (byte)233, (byte)235, (byte)12, (byte)26, (byte)67, (byte)7, (byte)252, (byte)244, (byte)168, (byte)241}));
                Debug.Assert(pack.target_component == (byte)(byte)116);
                Debug.Assert(pack.target_system == (byte)(byte)138);
                Debug.Assert(pack.target_network == (byte)(byte)211);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)211;
            p110.target_component = (byte)(byte)116;
            p110.target_system = (byte)(byte)138;
            p110.payload_SET(new byte[] {(byte)10, (byte)83, (byte)82, (byte)210, (byte)87, (byte)51, (byte)58, (byte)137, (byte)72, (byte)44, (byte)129, (byte)8, (byte)100, (byte)89, (byte)208, (byte)62, (byte)197, (byte)69, (byte)104, (byte)220, (byte)205, (byte)115, (byte)207, (byte)0, (byte)200, (byte)131, (byte)240, (byte)133, (byte)192, (byte)8, (byte)246, (byte)70, (byte)32, (byte)209, (byte)81, (byte)46, (byte)151, (byte)4, (byte)240, (byte)148, (byte)75, (byte)150, (byte)188, (byte)189, (byte)56, (byte)120, (byte)141, (byte)154, (byte)238, (byte)239, (byte)145, (byte)56, (byte)118, (byte)109, (byte)74, (byte)47, (byte)101, (byte)128, (byte)118, (byte)218, (byte)164, (byte)72, (byte)133, (byte)164, (byte)123, (byte)242, (byte)29, (byte)48, (byte)254, (byte)164, (byte)56, (byte)54, (byte)214, (byte)13, (byte)83, (byte)18, (byte)251, (byte)144, (byte)57, (byte)222, (byte)216, (byte)60, (byte)78, (byte)179, (byte)175, (byte)5, (byte)86, (byte)201, (byte)199, (byte)112, (byte)237, (byte)43, (byte)4, (byte)83, (byte)87, (byte)230, (byte)205, (byte)128, (byte)43, (byte)155, (byte)68, (byte)223, (byte)78, (byte)19, (byte)18, (byte)67, (byte)60, (byte)140, (byte)58, (byte)57, (byte)205, (byte)210, (byte)213, (byte)145, (byte)117, (byte)201, (byte)65, (byte)46, (byte)201, (byte)222, (byte)131, (byte)185, (byte)229, (byte)49, (byte)242, (byte)70, (byte)21, (byte)20, (byte)76, (byte)214, (byte)46, (byte)132, (byte)109, (byte)1, (byte)122, (byte)119, (byte)124, (byte)152, (byte)63, (byte)95, (byte)184, (byte)166, (byte)53, (byte)199, (byte)203, (byte)91, (byte)91, (byte)195, (byte)37, (byte)231, (byte)106, (byte)232, (byte)167, (byte)178, (byte)182, (byte)177, (byte)202, (byte)1, (byte)192, (byte)225, (byte)69, (byte)100, (byte)98, (byte)137, (byte)120, (byte)101, (byte)26, (byte)52, (byte)203, (byte)217, (byte)239, (byte)232, (byte)98, (byte)250, (byte)225, (byte)211, (byte)105, (byte)142, (byte)23, (byte)139, (byte)223, (byte)40, (byte)23, (byte)17, (byte)228, (byte)241, (byte)143, (byte)137, (byte)109, (byte)210, (byte)116, (byte)143, (byte)157, (byte)44, (byte)3, (byte)68, (byte)212, (byte)136, (byte)215, (byte)252, (byte)157, (byte)3, (byte)82, (byte)51, (byte)155, (byte)240, (byte)166, (byte)130, (byte)246, (byte)129, (byte)197, (byte)214, (byte)234, (byte)59, (byte)178, (byte)242, (byte)174, (byte)15, (byte)169, (byte)154, (byte)99, (byte)234, (byte)29, (byte)131, (byte)235, (byte)152, (byte)238, (byte)138, (byte)49, (byte)61, (byte)7, (byte)83, (byte)153, (byte)179, (byte)49, (byte)178, (byte)191, (byte)38, (byte)104, (byte)99, (byte)135, (byte)233, (byte)235, (byte)12, (byte)26, (byte)67, (byte)7, (byte)252, (byte)244, (byte)168, (byte)241}, 0) ;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ts1 == (long) -4024182146625090512L);
                Debug.Assert(pack.tc1 == (long)989514208660801766L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long) -4024182146625090512L;
            p111.tc1 = (long)989514208660801766L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)1895496455U);
                Debug.Assert(pack.time_usec == (ulong)6036552787125856395L);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)6036552787125856395L;
            p112.seq = (uint)1895496455U;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ve == (short)(short) -22534);
                Debug.Assert(pack.fix_type == (byte)(byte)177);
                Debug.Assert(pack.satellites_visible == (byte)(byte)189);
                Debug.Assert(pack.epv == (ushort)(ushort)36144);
                Debug.Assert(pack.lat == (int)1814977987);
                Debug.Assert(pack.alt == (int)521101770);
                Debug.Assert(pack.vn == (short)(short) -25231);
                Debug.Assert(pack.time_usec == (ulong)5355102775878181514L);
                Debug.Assert(pack.vd == (short)(short)25012);
                Debug.Assert(pack.eph == (ushort)(ushort)46364);
                Debug.Assert(pack.vel == (ushort)(ushort)31477);
                Debug.Assert(pack.cog == (ushort)(ushort)15660);
                Debug.Assert(pack.lon == (int)1188748272);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.lat = (int)1814977987;
            p113.vel = (ushort)(ushort)31477;
            p113.time_usec = (ulong)5355102775878181514L;
            p113.fix_type = (byte)(byte)177;
            p113.satellites_visible = (byte)(byte)189;
            p113.lon = (int)1188748272;
            p113.alt = (int)521101770;
            p113.ve = (short)(short) -22534;
            p113.cog = (ushort)(ushort)15660;
            p113.eph = (ushort)(ushort)46364;
            p113.vd = (short)(short)25012;
            p113.epv = (ushort)(ushort)36144;
            p113.vn = (short)(short) -25231;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_zgyro == (float)2.7267724E38F);
                Debug.Assert(pack.quality == (byte)(byte)125);
                Debug.Assert(pack.sensor_id == (byte)(byte)188);
                Debug.Assert(pack.integrated_xgyro == (float)1.2016327E38F);
                Debug.Assert(pack.distance == (float) -1.197108E38F);
                Debug.Assert(pack.integrated_x == (float)3.0694703E38F);
                Debug.Assert(pack.integrated_y == (float) -2.7120387E38F);
                Debug.Assert(pack.temperature == (short)(short)21197);
                Debug.Assert(pack.integration_time_us == (uint)1312452945U);
                Debug.Assert(pack.time_delta_distance_us == (uint)627689303U);
                Debug.Assert(pack.time_usec == (ulong)210903230406645186L);
                Debug.Assert(pack.integrated_ygyro == (float)2.906174E38F);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.sensor_id = (byte)(byte)188;
            p114.integrated_ygyro = (float)2.906174E38F;
            p114.integrated_zgyro = (float)2.7267724E38F;
            p114.time_usec = (ulong)210903230406645186L;
            p114.temperature = (short)(short)21197;
            p114.integrated_y = (float) -2.7120387E38F;
            p114.integrated_xgyro = (float)1.2016327E38F;
            p114.integration_time_us = (uint)1312452945U;
            p114.quality = (byte)(byte)125;
            p114.distance = (float) -1.197108E38F;
            p114.time_delta_distance_us = (uint)627689303U;
            p114.integrated_x = (float)3.0694703E38F;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short) -26419);
                Debug.Assert(pack.pitchspeed == (float) -1.6089604E38F);
                Debug.Assert(pack.yawspeed == (float)2.0003557E38F);
                Debug.Assert(pack.yacc == (short)(short)342);
                Debug.Assert(pack.vx == (short)(short) -5331);
                Debug.Assert(pack.vy == (short)(short)23918);
                Debug.Assert(pack.rollspeed == (float)2.989503E38F);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {-3.2467114E38F, -1.1478857E38F, 1.3062663E38F, -2.727927E38F}));
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)24220);
                Debug.Assert(pack.lat == (int) -1304590391);
                Debug.Assert(pack.time_usec == (ulong)6872229183287101571L);
                Debug.Assert(pack.zacc == (short)(short)14497);
                Debug.Assert(pack.alt == (int) -1545475207);
                Debug.Assert(pack.lon == (int)490002358);
                Debug.Assert(pack.vz == (short)(short) -20304);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)39187);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.true_airspeed = (ushort)(ushort)39187;
            p115.time_usec = (ulong)6872229183287101571L;
            p115.lon = (int)490002358;
            p115.vx = (short)(short) -5331;
            p115.pitchspeed = (float) -1.6089604E38F;
            p115.yacc = (short)(short)342;
            p115.zacc = (short)(short)14497;
            p115.vy = (short)(short)23918;
            p115.vz = (short)(short) -20304;
            p115.alt = (int) -1545475207;
            p115.rollspeed = (float)2.989503E38F;
            p115.xacc = (short)(short) -26419;
            p115.yawspeed = (float)2.0003557E38F;
            p115.attitude_quaternion_SET(new float[] {-3.2467114E38F, -1.1478857E38F, 1.3062663E38F, -2.727927E38F}, 0) ;
            p115.lat = (int) -1304590391;
            p115.ind_airspeed = (ushort)(ushort)24220;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (short)(short)29221);
                Debug.Assert(pack.xmag == (short)(short) -32609);
                Debug.Assert(pack.time_boot_ms == (uint)2129670338U);
                Debug.Assert(pack.zmag == (short)(short) -16417);
                Debug.Assert(pack.zacc == (short)(short)1305);
                Debug.Assert(pack.xacc == (short)(short) -29264);
                Debug.Assert(pack.xgyro == (short)(short) -2998);
                Debug.Assert(pack.yacc == (short)(short) -19115);
                Debug.Assert(pack.ygyro == (short)(short)24835);
                Debug.Assert(pack.ymag == (short)(short) -28417);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.yacc = (short)(short) -19115;
            p116.xacc = (short)(short) -29264;
            p116.zgyro = (short)(short)29221;
            p116.xgyro = (short)(short) -2998;
            p116.zmag = (short)(short) -16417;
            p116.ygyro = (short)(short)24835;
            p116.time_boot_ms = (uint)2129670338U;
            p116.xmag = (short)(short) -32609;
            p116.zacc = (short)(short)1305;
            p116.ymag = (short)(short) -28417;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start == (ushort)(ushort)32396);
                Debug.Assert(pack.target_system == (byte)(byte)99);
                Debug.Assert(pack.end == (ushort)(ushort)59049);
                Debug.Assert(pack.target_component == (byte)(byte)20);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.start = (ushort)(ushort)32396;
            p117.target_system = (byte)(byte)99;
            p117.end = (ushort)(ushort)59049;
            p117.target_component = (byte)(byte)20;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)34968);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)15554);
                Debug.Assert(pack.size == (uint)1540446378U);
                Debug.Assert(pack.num_logs == (ushort)(ushort)11546);
                Debug.Assert(pack.time_utc == (uint)2532408541U);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.num_logs = (ushort)(ushort)11546;
            p118.time_utc = (uint)2532408541U;
            p118.id = (ushort)(ushort)34968;
            p118.size = (uint)1540446378U;
            p118.last_log_num = (ushort)(ushort)15554;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)74);
                Debug.Assert(pack.id == (ushort)(ushort)42726);
                Debug.Assert(pack.count == (uint)2121116422U);
                Debug.Assert(pack.target_system == (byte)(byte)195);
                Debug.Assert(pack.ofs == (uint)647900904U);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_component = (byte)(byte)74;
            p119.ofs = (uint)647900904U;
            p119.target_system = (byte)(byte)195;
            p119.count = (uint)2121116422U;
            p119.id = (ushort)(ushort)42726;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)341312927U);
                Debug.Assert(pack.count == (byte)(byte)150);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)121, (byte)204, (byte)144, (byte)62, (byte)120, (byte)95, (byte)67, (byte)132, (byte)201, (byte)165, (byte)55, (byte)27, (byte)227, (byte)51, (byte)39, (byte)215, (byte)191, (byte)192, (byte)96, (byte)164, (byte)138, (byte)241, (byte)126, (byte)89, (byte)58, (byte)47, (byte)235, (byte)176, (byte)107, (byte)103, (byte)68, (byte)99, (byte)81, (byte)94, (byte)159, (byte)121, (byte)126, (byte)173, (byte)133, (byte)176, (byte)94, (byte)180, (byte)52, (byte)10, (byte)12, (byte)90, (byte)150, (byte)73, (byte)72, (byte)99, (byte)128, (byte)150, (byte)183, (byte)218, (byte)135, (byte)129, (byte)251, (byte)151, (byte)164, (byte)0, (byte)118, (byte)221, (byte)129, (byte)136, (byte)249, (byte)83, (byte)168, (byte)45, (byte)136, (byte)74, (byte)49, (byte)88, (byte)95, (byte)158, (byte)223, (byte)220, (byte)107, (byte)12, (byte)14, (byte)169, (byte)131, (byte)48, (byte)121, (byte)202, (byte)56, (byte)72, (byte)143, (byte)64, (byte)176, (byte)173}));
                Debug.Assert(pack.id == (ushort)(ushort)7556);
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)7556;
            p120.data__SET(new byte[] {(byte)121, (byte)204, (byte)144, (byte)62, (byte)120, (byte)95, (byte)67, (byte)132, (byte)201, (byte)165, (byte)55, (byte)27, (byte)227, (byte)51, (byte)39, (byte)215, (byte)191, (byte)192, (byte)96, (byte)164, (byte)138, (byte)241, (byte)126, (byte)89, (byte)58, (byte)47, (byte)235, (byte)176, (byte)107, (byte)103, (byte)68, (byte)99, (byte)81, (byte)94, (byte)159, (byte)121, (byte)126, (byte)173, (byte)133, (byte)176, (byte)94, (byte)180, (byte)52, (byte)10, (byte)12, (byte)90, (byte)150, (byte)73, (byte)72, (byte)99, (byte)128, (byte)150, (byte)183, (byte)218, (byte)135, (byte)129, (byte)251, (byte)151, (byte)164, (byte)0, (byte)118, (byte)221, (byte)129, (byte)136, (byte)249, (byte)83, (byte)168, (byte)45, (byte)136, (byte)74, (byte)49, (byte)88, (byte)95, (byte)158, (byte)223, (byte)220, (byte)107, (byte)12, (byte)14, (byte)169, (byte)131, (byte)48, (byte)121, (byte)202, (byte)56, (byte)72, (byte)143, (byte)64, (byte)176, (byte)173}, 0) ;
            p120.ofs = (uint)341312927U;
            p120.count = (byte)(byte)150;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)185);
                Debug.Assert(pack.target_system == (byte)(byte)14);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_component = (byte)(byte)185;
            p121.target_system = (byte)(byte)14;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)47);
                Debug.Assert(pack.target_system == (byte)(byte)190);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_component = (byte)(byte)47;
            p122.target_system = (byte)(byte)190;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)248);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)220, (byte)71, (byte)5, (byte)141, (byte)4, (byte)165, (byte)123, (byte)134, (byte)159, (byte)177, (byte)139, (byte)181, (byte)63, (byte)24, (byte)14, (byte)66, (byte)1, (byte)83, (byte)9, (byte)13, (byte)166, (byte)113, (byte)38, (byte)191, (byte)204, (byte)183, (byte)221, (byte)253, (byte)80, (byte)10, (byte)174, (byte)16, (byte)12, (byte)119, (byte)175, (byte)123, (byte)254, (byte)215, (byte)3, (byte)196, (byte)77, (byte)152, (byte)36, (byte)199, (byte)145, (byte)209, (byte)158, (byte)227, (byte)42, (byte)236, (byte)109, (byte)153, (byte)218, (byte)120, (byte)115, (byte)201, (byte)251, (byte)160, (byte)216, (byte)207, (byte)199, (byte)9, (byte)122, (byte)234, (byte)148, (byte)217, (byte)35, (byte)136, (byte)107, (byte)157, (byte)236, (byte)31, (byte)213, (byte)218, (byte)15, (byte)51, (byte)96, (byte)79, (byte)27, (byte)7, (byte)40, (byte)26, (byte)95, (byte)244, (byte)48, (byte)217, (byte)63, (byte)69, (byte)177, (byte)83, (byte)179, (byte)235, (byte)228, (byte)164, (byte)131, (byte)104, (byte)18, (byte)127, (byte)188, (byte)240, (byte)109, (byte)190, (byte)42, (byte)146, (byte)235, (byte)105, (byte)41, (byte)182, (byte)128, (byte)47}));
                Debug.Assert(pack.target_system == (byte)(byte)185);
                Debug.Assert(pack.len == (byte)(byte)194);
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.data__SET(new byte[] {(byte)220, (byte)71, (byte)5, (byte)141, (byte)4, (byte)165, (byte)123, (byte)134, (byte)159, (byte)177, (byte)139, (byte)181, (byte)63, (byte)24, (byte)14, (byte)66, (byte)1, (byte)83, (byte)9, (byte)13, (byte)166, (byte)113, (byte)38, (byte)191, (byte)204, (byte)183, (byte)221, (byte)253, (byte)80, (byte)10, (byte)174, (byte)16, (byte)12, (byte)119, (byte)175, (byte)123, (byte)254, (byte)215, (byte)3, (byte)196, (byte)77, (byte)152, (byte)36, (byte)199, (byte)145, (byte)209, (byte)158, (byte)227, (byte)42, (byte)236, (byte)109, (byte)153, (byte)218, (byte)120, (byte)115, (byte)201, (byte)251, (byte)160, (byte)216, (byte)207, (byte)199, (byte)9, (byte)122, (byte)234, (byte)148, (byte)217, (byte)35, (byte)136, (byte)107, (byte)157, (byte)236, (byte)31, (byte)213, (byte)218, (byte)15, (byte)51, (byte)96, (byte)79, (byte)27, (byte)7, (byte)40, (byte)26, (byte)95, (byte)244, (byte)48, (byte)217, (byte)63, (byte)69, (byte)177, (byte)83, (byte)179, (byte)235, (byte)228, (byte)164, (byte)131, (byte)104, (byte)18, (byte)127, (byte)188, (byte)240, (byte)109, (byte)190, (byte)42, (byte)146, (byte)235, (byte)105, (byte)41, (byte)182, (byte)128, (byte)47}, 0) ;
            p123.target_system = (byte)(byte)185;
            p123.len = (byte)(byte)194;
            p123.target_component = (byte)(byte)248;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel == (ushort)(ushort)42768);
                Debug.Assert(pack.satellites_visible == (byte)(byte)213);
                Debug.Assert(pack.alt == (int)44474474);
                Debug.Assert(pack.lat == (int)1318680390);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
                Debug.Assert(pack.dgps_numch == (byte)(byte)155);
                Debug.Assert(pack.time_usec == (ulong)488443397418496847L);
                Debug.Assert(pack.dgps_age == (uint)3592436888U);
                Debug.Assert(pack.epv == (ushort)(ushort)38074);
                Debug.Assert(pack.lon == (int)955170324);
                Debug.Assert(pack.cog == (ushort)(ushort)59338);
                Debug.Assert(pack.eph == (ushort)(ushort)49553);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.vel = (ushort)(ushort)42768;
            p124.alt = (int)44474474;
            p124.lon = (int)955170324;
            p124.dgps_age = (uint)3592436888U;
            p124.satellites_visible = (byte)(byte)213;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS;
            p124.time_usec = (ulong)488443397418496847L;
            p124.cog = (ushort)(ushort)59338;
            p124.dgps_numch = (byte)(byte)155;
            p124.lat = (int)1318680390;
            p124.epv = (ushort)(ushort)38074;
            p124.eph = (ushort)(ushort)49553;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vcc == (ushort)(ushort)3265);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID));
                Debug.Assert(pack.Vservo == (ushort)(ushort)30306);
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID);
            p125.Vcc = (ushort)(ushort)3265;
            p125.Vservo = (ushort)(ushort)30306;
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (byte)(byte)169);
                Debug.Assert(pack.timeout == (ushort)(ushort)56309);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND));
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)69, (byte)138, (byte)120, (byte)159, (byte)153, (byte)154, (byte)183, (byte)55, (byte)132, (byte)158, (byte)63, (byte)27, (byte)95, (byte)167, (byte)66, (byte)45, (byte)238, (byte)152, (byte)199, (byte)160, (byte)118, (byte)105, (byte)171, (byte)184, (byte)159, (byte)155, (byte)42, (byte)80, (byte)211, (byte)106, (byte)15, (byte)63, (byte)196, (byte)40, (byte)22, (byte)89, (byte)219, (byte)221, (byte)16, (byte)57, (byte)201, (byte)125, (byte)226, (byte)181, (byte)101, (byte)236, (byte)230, (byte)225, (byte)5, (byte)174, (byte)60, (byte)91, (byte)170, (byte)93, (byte)109, (byte)242, (byte)218, (byte)159, (byte)45, (byte)165, (byte)160, (byte)229, (byte)38, (byte)72, (byte)84, (byte)132, (byte)53, (byte)115, (byte)28, (byte)241}));
                Debug.Assert(pack.baudrate == (uint)1571198558U);
                Debug.Assert(pack.device == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.timeout = (ushort)(ushort)56309;
            p126.baudrate = (uint)1571198558U;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND);
            p126.data__SET(new byte[] {(byte)69, (byte)138, (byte)120, (byte)159, (byte)153, (byte)154, (byte)183, (byte)55, (byte)132, (byte)158, (byte)63, (byte)27, (byte)95, (byte)167, (byte)66, (byte)45, (byte)238, (byte)152, (byte)199, (byte)160, (byte)118, (byte)105, (byte)171, (byte)184, (byte)159, (byte)155, (byte)42, (byte)80, (byte)211, (byte)106, (byte)15, (byte)63, (byte)196, (byte)40, (byte)22, (byte)89, (byte)219, (byte)221, (byte)16, (byte)57, (byte)201, (byte)125, (byte)226, (byte)181, (byte)101, (byte)236, (byte)230, (byte)225, (byte)5, (byte)174, (byte)60, (byte)91, (byte)170, (byte)93, (byte)109, (byte)242, (byte)218, (byte)159, (byte)45, (byte)165, (byte)160, (byte)229, (byte)38, (byte)72, (byte)84, (byte)132, (byte)53, (byte)115, (byte)28, (byte)241}, 0) ;
            p126.count = (byte)(byte)169;
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rtk_rate == (byte)(byte)241);
                Debug.Assert(pack.time_last_baseline_ms == (uint)1540514738U);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)151);
                Debug.Assert(pack.baseline_c_mm == (int)246466974);
                Debug.Assert(pack.baseline_b_mm == (int)776876312);
                Debug.Assert(pack.baseline_a_mm == (int) -1700610251);
                Debug.Assert(pack.tow == (uint)3070990198U);
                Debug.Assert(pack.wn == (ushort)(ushort)9076);
                Debug.Assert(pack.accuracy == (uint)3726776424U);
                Debug.Assert(pack.nsats == (byte)(byte)13);
                Debug.Assert(pack.iar_num_hypotheses == (int) -1631011446);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)248);
                Debug.Assert(pack.rtk_health == (byte)(byte)61);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.rtk_health = (byte)(byte)61;
            p127.iar_num_hypotheses = (int) -1631011446;
            p127.rtk_receiver_id = (byte)(byte)248;
            p127.accuracy = (uint)3726776424U;
            p127.baseline_c_mm = (int)246466974;
            p127.baseline_b_mm = (int)776876312;
            p127.nsats = (byte)(byte)13;
            p127.wn = (ushort)(ushort)9076;
            p127.time_last_baseline_ms = (uint)1540514738U;
            p127.baseline_a_mm = (int) -1700610251;
            p127.tow = (uint)3070990198U;
            p127.rtk_rate = (byte)(byte)241;
            p127.baseline_coords_type = (byte)(byte)151;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nsats == (byte)(byte)98);
                Debug.Assert(pack.rtk_rate == (byte)(byte)135);
                Debug.Assert(pack.wn == (ushort)(ushort)50449);
                Debug.Assert(pack.tow == (uint)3087133595U);
                Debug.Assert(pack.iar_num_hypotheses == (int) -205284);
                Debug.Assert(pack.baseline_c_mm == (int)347472177);
                Debug.Assert(pack.time_last_baseline_ms == (uint)1743694892U);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)198);
                Debug.Assert(pack.rtk_health == (byte)(byte)68);
                Debug.Assert(pack.baseline_a_mm == (int) -2092153309);
                Debug.Assert(pack.accuracy == (uint)1113196501U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)12);
                Debug.Assert(pack.baseline_b_mm == (int)1607390622);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.iar_num_hypotheses = (int) -205284;
            p128.baseline_coords_type = (byte)(byte)198;
            p128.rtk_health = (byte)(byte)68;
            p128.baseline_b_mm = (int)1607390622;
            p128.wn = (ushort)(ushort)50449;
            p128.time_last_baseline_ms = (uint)1743694892U;
            p128.nsats = (byte)(byte)98;
            p128.rtk_receiver_id = (byte)(byte)12;
            p128.rtk_rate = (byte)(byte)135;
            p128.baseline_c_mm = (int)347472177;
            p128.baseline_a_mm = (int) -2092153309;
            p128.accuracy = (uint)1113196501U;
            p128.tow = (uint)3087133595U;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ymag == (short)(short) -22259);
                Debug.Assert(pack.zmag == (short)(short) -2017);
                Debug.Assert(pack.ygyro == (short)(short) -12398);
                Debug.Assert(pack.xacc == (short)(short)10225);
                Debug.Assert(pack.yacc == (short)(short) -31984);
                Debug.Assert(pack.zgyro == (short)(short) -24426);
                Debug.Assert(pack.xmag == (short)(short)25980);
                Debug.Assert(pack.time_boot_ms == (uint)2745040414U);
                Debug.Assert(pack.zacc == (short)(short)15591);
                Debug.Assert(pack.xgyro == (short)(short)7887);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.xgyro = (short)(short)7887;
            p129.ygyro = (short)(short) -12398;
            p129.zmag = (short)(short) -2017;
            p129.xacc = (short)(short)10225;
            p129.zacc = (short)(short)15591;
            p129.yacc = (short)(short) -31984;
            p129.zgyro = (short)(short) -24426;
            p129.time_boot_ms = (uint)2745040414U;
            p129.ymag = (short)(short) -22259;
            p129.xmag = (short)(short)25980;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.jpg_quality == (byte)(byte)223);
                Debug.Assert(pack.packets == (ushort)(ushort)64842);
                Debug.Assert(pack.size == (uint)744977224U);
                Debug.Assert(pack.height == (ushort)(ushort)65035);
                Debug.Assert(pack.type == (byte)(byte)201);
                Debug.Assert(pack.payload == (byte)(byte)116);
                Debug.Assert(pack.width == (ushort)(ushort)20063);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.payload = (byte)(byte)116;
            p130.size = (uint)744977224U;
            p130.height = (ushort)(ushort)65035;
            p130.jpg_quality = (byte)(byte)223;
            p130.type = (byte)(byte)201;
            p130.packets = (ushort)(ushort)64842;
            p130.width = (ushort)(ushort)20063;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)202, (byte)246, (byte)118, (byte)188, (byte)45, (byte)89, (byte)96, (byte)101, (byte)196, (byte)64, (byte)73, (byte)54, (byte)32, (byte)1, (byte)44, (byte)201, (byte)0, (byte)121, (byte)187, (byte)156, (byte)183, (byte)24, (byte)31, (byte)186, (byte)53, (byte)187, (byte)195, (byte)31, (byte)3, (byte)203, (byte)27, (byte)37, (byte)204, (byte)174, (byte)243, (byte)101, (byte)255, (byte)101, (byte)27, (byte)83, (byte)172, (byte)14, (byte)72, (byte)51, (byte)5, (byte)25, (byte)39, (byte)218, (byte)66, (byte)212, (byte)34, (byte)163, (byte)68, (byte)95, (byte)21, (byte)112, (byte)49, (byte)219, (byte)78, (byte)238, (byte)253, (byte)253, (byte)187, (byte)42, (byte)106, (byte)153, (byte)9, (byte)40, (byte)129, (byte)26, (byte)14, (byte)31, (byte)148, (byte)98, (byte)112, (byte)56, (byte)220, (byte)18, (byte)147, (byte)116, (byte)132, (byte)210, (byte)99, (byte)66, (byte)83, (byte)208, (byte)5, (byte)79, (byte)209, (byte)229, (byte)140, (byte)108, (byte)198, (byte)218, (byte)250, (byte)102, (byte)196, (byte)198, (byte)96, (byte)186, (byte)37, (byte)203, (byte)107, (byte)114, (byte)18, (byte)138, (byte)216, (byte)105, (byte)5, (byte)56, (byte)119, (byte)165, (byte)255, (byte)176, (byte)159, (byte)209, (byte)115, (byte)135, (byte)16, (byte)180, (byte)117, (byte)8, (byte)62, (byte)125, (byte)143, (byte)211, (byte)57, (byte)88, (byte)55, (byte)55, (byte)43, (byte)229, (byte)154, (byte)115, (byte)130, (byte)95, (byte)7, (byte)19, (byte)116, (byte)218, (byte)169, (byte)108, (byte)105, (byte)133, (byte)211, (byte)111, (byte)193, (byte)108, (byte)159, (byte)236, (byte)28, (byte)70, (byte)45, (byte)247, (byte)79, (byte)78, (byte)169, (byte)146, (byte)18, (byte)26, (byte)31, (byte)204, (byte)252, (byte)109, (byte)58, (byte)74, (byte)15, (byte)187, (byte)104, (byte)204, (byte)11, (byte)35, (byte)156, (byte)206, (byte)49, (byte)88, (byte)81, (byte)229, (byte)134, (byte)249, (byte)24, (byte)249, (byte)21, (byte)118, (byte)15, (byte)249, (byte)80, (byte)125, (byte)104, (byte)49, (byte)159, (byte)42, (byte)201, (byte)70, (byte)193, (byte)114, (byte)65, (byte)239, (byte)129, (byte)59, (byte)142, (byte)21, (byte)203, (byte)216, (byte)202, (byte)197, (byte)159, (byte)0, (byte)44, (byte)57, (byte)174, (byte)211, (byte)81, (byte)168, (byte)216, (byte)175, (byte)203, (byte)165, (byte)228, (byte)49, (byte)185, (byte)234, (byte)170, (byte)219, (byte)144, (byte)31, (byte)24, (byte)45, (byte)225, (byte)206, (byte)149, (byte)206, (byte)96, (byte)145, (byte)14, (byte)127, (byte)181, (byte)233, (byte)94, (byte)7, (byte)5, (byte)143, (byte)126, (byte)10, (byte)36, (byte)210, (byte)181, (byte)21, (byte)140, (byte)172, (byte)158, (byte)48, (byte)66}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)31271);
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)31271;
            p131.data__SET(new byte[] {(byte)202, (byte)246, (byte)118, (byte)188, (byte)45, (byte)89, (byte)96, (byte)101, (byte)196, (byte)64, (byte)73, (byte)54, (byte)32, (byte)1, (byte)44, (byte)201, (byte)0, (byte)121, (byte)187, (byte)156, (byte)183, (byte)24, (byte)31, (byte)186, (byte)53, (byte)187, (byte)195, (byte)31, (byte)3, (byte)203, (byte)27, (byte)37, (byte)204, (byte)174, (byte)243, (byte)101, (byte)255, (byte)101, (byte)27, (byte)83, (byte)172, (byte)14, (byte)72, (byte)51, (byte)5, (byte)25, (byte)39, (byte)218, (byte)66, (byte)212, (byte)34, (byte)163, (byte)68, (byte)95, (byte)21, (byte)112, (byte)49, (byte)219, (byte)78, (byte)238, (byte)253, (byte)253, (byte)187, (byte)42, (byte)106, (byte)153, (byte)9, (byte)40, (byte)129, (byte)26, (byte)14, (byte)31, (byte)148, (byte)98, (byte)112, (byte)56, (byte)220, (byte)18, (byte)147, (byte)116, (byte)132, (byte)210, (byte)99, (byte)66, (byte)83, (byte)208, (byte)5, (byte)79, (byte)209, (byte)229, (byte)140, (byte)108, (byte)198, (byte)218, (byte)250, (byte)102, (byte)196, (byte)198, (byte)96, (byte)186, (byte)37, (byte)203, (byte)107, (byte)114, (byte)18, (byte)138, (byte)216, (byte)105, (byte)5, (byte)56, (byte)119, (byte)165, (byte)255, (byte)176, (byte)159, (byte)209, (byte)115, (byte)135, (byte)16, (byte)180, (byte)117, (byte)8, (byte)62, (byte)125, (byte)143, (byte)211, (byte)57, (byte)88, (byte)55, (byte)55, (byte)43, (byte)229, (byte)154, (byte)115, (byte)130, (byte)95, (byte)7, (byte)19, (byte)116, (byte)218, (byte)169, (byte)108, (byte)105, (byte)133, (byte)211, (byte)111, (byte)193, (byte)108, (byte)159, (byte)236, (byte)28, (byte)70, (byte)45, (byte)247, (byte)79, (byte)78, (byte)169, (byte)146, (byte)18, (byte)26, (byte)31, (byte)204, (byte)252, (byte)109, (byte)58, (byte)74, (byte)15, (byte)187, (byte)104, (byte)204, (byte)11, (byte)35, (byte)156, (byte)206, (byte)49, (byte)88, (byte)81, (byte)229, (byte)134, (byte)249, (byte)24, (byte)249, (byte)21, (byte)118, (byte)15, (byte)249, (byte)80, (byte)125, (byte)104, (byte)49, (byte)159, (byte)42, (byte)201, (byte)70, (byte)193, (byte)114, (byte)65, (byte)239, (byte)129, (byte)59, (byte)142, (byte)21, (byte)203, (byte)216, (byte)202, (byte)197, (byte)159, (byte)0, (byte)44, (byte)57, (byte)174, (byte)211, (byte)81, (byte)168, (byte)216, (byte)175, (byte)203, (byte)165, (byte)228, (byte)49, (byte)185, (byte)234, (byte)170, (byte)219, (byte)144, (byte)31, (byte)24, (byte)45, (byte)225, (byte)206, (byte)149, (byte)206, (byte)96, (byte)145, (byte)14, (byte)127, (byte)181, (byte)233, (byte)94, (byte)7, (byte)5, (byte)143, (byte)126, (byte)10, (byte)36, (byte)210, (byte)181, (byte)21, (byte)140, (byte)172, (byte)158, (byte)48, (byte)66}, 0) ;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (byte)(byte)31);
                Debug.Assert(pack.type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
                Debug.Assert(pack.time_boot_ms == (uint)3874781679U);
                Debug.Assert(pack.covariance == (byte)(byte)238);
                Debug.Assert(pack.current_distance == (ushort)(ushort)14106);
                Debug.Assert(pack.orientation == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_YAW_135);
                Debug.Assert(pack.max_distance == (ushort)(ushort)27014);
                Debug.Assert(pack.min_distance == (ushort)(ushort)27269);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.current_distance = (ushort)(ushort)14106;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_YAW_135;
            p132.min_distance = (ushort)(ushort)27269;
            p132.max_distance = (ushort)(ushort)27014;
            p132.id = (byte)(byte)31;
            p132.covariance = (byte)(byte)238;
            p132.time_boot_ms = (uint)3874781679U;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)27325);
                Debug.Assert(pack.mask == (ulong)1117938589318359621L);
                Debug.Assert(pack.lat == (int)240065537);
                Debug.Assert(pack.lon == (int) -1812604334);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.mask = (ulong)1117938589318359621L;
            p133.lon = (int) -1812604334;
            p133.grid_spacing = (ushort)(ushort)27325;
            p133.lat = (int)240065537;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)57544);
                Debug.Assert(pack.lon == (int)1545685853);
                Debug.Assert(pack.lat == (int)310226264);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -29319, (short)646, (short) -22439, (short) -31694, (short)22391, (short) -14801, (short) -9045, (short)10312, (short)19609, (short)20631, (short) -5056, (short)16424, (short) -11003, (short)13207, (short) -7440, (short)28751}));
                Debug.Assert(pack.gridbit == (byte)(byte)224);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.grid_spacing = (ushort)(ushort)57544;
            p134.lon = (int)1545685853;
            p134.gridbit = (byte)(byte)224;
            p134.data__SET(new short[] {(short) -29319, (short)646, (short) -22439, (short) -31694, (short)22391, (short) -14801, (short) -9045, (short)10312, (short)19609, (short)20631, (short) -5056, (short)16424, (short) -11003, (short)13207, (short) -7440, (short)28751}, 0) ;
            p134.lat = (int)310226264;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)889117932);
                Debug.Assert(pack.lat == (int)605243677);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)605243677;
            p135.lon = (int)889117932;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pending == (ushort)(ushort)62583);
                Debug.Assert(pack.loaded == (ushort)(ushort)6602);
                Debug.Assert(pack.terrain_height == (float)1.6528649E38F);
                Debug.Assert(pack.spacing == (ushort)(ushort)14774);
                Debug.Assert(pack.lon == (int) -7268721);
                Debug.Assert(pack.current_height == (float) -5.7188576E37F);
                Debug.Assert(pack.lat == (int) -1291227362);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.pending = (ushort)(ushort)62583;
            p136.loaded = (ushort)(ushort)6602;
            p136.terrain_height = (float)1.6528649E38F;
            p136.current_height = (float) -5.7188576E37F;
            p136.spacing = (ushort)(ushort)14774;
            p136.lon = (int) -7268721;
            p136.lat = (int) -1291227362;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float)1.3595048E38F);
                Debug.Assert(pack.temperature == (short)(short) -19146);
                Debug.Assert(pack.time_boot_ms == (uint)2918311973U);
                Debug.Assert(pack.press_abs == (float)2.7880085E38F);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.press_diff = (float)1.3595048E38F;
            p137.time_boot_ms = (uint)2918311973U;
            p137.temperature = (short)(short) -19146;
            p137.press_abs = (float)2.7880085E38F;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -2.1024281E38F);
                Debug.Assert(pack.z == (float)2.7872356E37F);
                Debug.Assert(pack.y == (float)1.2386465E38F);
                Debug.Assert(pack.time_usec == (ulong)5997092993126042227L);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-6.3465855E37F, 4.602602E37F, 6.206797E37F, 1.3288698E38F}));
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.x = (float) -2.1024281E38F;
            p138.q_SET(new float[] {-6.3465855E37F, 4.602602E37F, 6.206797E37F, 1.3288698E38F}, 0) ;
            p138.z = (float)2.7872356E37F;
            p138.time_usec = (ulong)5997092993126042227L;
            p138.y = (float)1.2386465E38F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)232);
                Debug.Assert(pack.target_component == (byte)(byte)244);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {4.592811E36F, 2.7442555E38F, 3.0049276E38F, -2.5500863E38F, 2.3517133E38F, 7.0814263E37F, 6.1131685E37F, 4.2235958E37F}));
                Debug.Assert(pack.time_usec == (ulong)6608063341871860171L);
                Debug.Assert(pack.target_system == (byte)(byte)56);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)6608063341871860171L;
            p139.group_mlx = (byte)(byte)232;
            p139.target_system = (byte)(byte)56;
            p139.target_component = (byte)(byte)244;
            p139.controls_SET(new float[] {4.592811E36F, 2.7442555E38F, 3.0049276E38F, -2.5500863E38F, 2.3517133E38F, 7.0814263E37F, 6.1131685E37F, 4.2235958E37F}, 0) ;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {3.252877E38F, 1.8296098E38F, 3.2847642E38F, -1.4869103E38F, -1.1478215E38F, 8.632443E37F, 4.698761E37F, -2.1040023E38F}));
                Debug.Assert(pack.group_mlx == (byte)(byte)106);
                Debug.Assert(pack.time_usec == (ulong)6685793768440646225L);
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.group_mlx = (byte)(byte)106;
            p140.controls_SET(new float[] {3.252877E38F, 1.8296098E38F, 3.2847642E38F, -1.4869103E38F, -1.1478215E38F, 8.632443E37F, 4.698761E37F, -2.1040023E38F}, 0) ;
            p140.time_usec = (ulong)6685793768440646225L;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_monotonic == (float)4.2989423E36F);
                Debug.Assert(pack.bottom_clearance == (float)1.8750236E38F);
                Debug.Assert(pack.altitude_local == (float) -1.929231E37F);
                Debug.Assert(pack.altitude_terrain == (float) -1.0291335E38F);
                Debug.Assert(pack.altitude_amsl == (float) -3.0255702E38F);
                Debug.Assert(pack.time_usec == (ulong)3136055308045942667L);
                Debug.Assert(pack.altitude_relative == (float)1.2413114E38F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)3136055308045942667L;
            p141.altitude_monotonic = (float)4.2989423E36F;
            p141.bottom_clearance = (float)1.8750236E38F;
            p141.altitude_relative = (float)1.2413114E38F;
            p141.altitude_local = (float) -1.929231E37F;
            p141.altitude_terrain = (float) -1.0291335E38F;
            p141.altitude_amsl = (float) -3.0255702E38F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)52, (byte)132, (byte)255, (byte)79, (byte)200, (byte)1, (byte)255, (byte)113, (byte)182, (byte)187, (byte)185, (byte)9, (byte)16, (byte)71, (byte)99, (byte)71, (byte)58, (byte)218, (byte)15, (byte)28, (byte)224, (byte)149, (byte)70, (byte)233, (byte)70, (byte)181, (byte)63, (byte)155, (byte)142, (byte)9, (byte)241, (byte)185, (byte)43, (byte)236, (byte)64, (byte)106, (byte)150, (byte)255, (byte)196, (byte)203, (byte)170, (byte)239, (byte)95, (byte)9, (byte)199, (byte)18, (byte)91, (byte)210, (byte)58, (byte)215, (byte)197, (byte)58, (byte)153, (byte)192, (byte)142, (byte)203, (byte)243, (byte)66, (byte)123, (byte)128, (byte)94, (byte)91, (byte)38, (byte)226, (byte)231, (byte)84, (byte)253, (byte)8, (byte)191, (byte)38, (byte)200, (byte)232, (byte)171, (byte)13, (byte)114, (byte)19, (byte)197, (byte)131, (byte)172, (byte)112, (byte)156, (byte)95, (byte)53, (byte)244, (byte)108, (byte)173, (byte)212, (byte)84, (byte)67, (byte)65, (byte)141, (byte)189, (byte)253, (byte)43, (byte)90, (byte)194, (byte)17, (byte)129, (byte)127, (byte)3, (byte)73, (byte)39, (byte)49, (byte)205, (byte)126, (byte)26, (byte)161, (byte)122, (byte)11, (byte)76, (byte)217, (byte)177, (byte)165, (byte)229, (byte)158, (byte)252, (byte)247, (byte)121, (byte)21, (byte)112}));
                Debug.Assert(pack.request_id == (byte)(byte)185);
                Debug.Assert(pack.transfer_type == (byte)(byte)150);
                Debug.Assert(pack.uri_type == (byte)(byte)152);
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)116, (byte)206, (byte)97, (byte)231, (byte)71, (byte)170, (byte)40, (byte)188, (byte)143, (byte)224, (byte)180, (byte)56, (byte)80, (byte)171, (byte)60, (byte)189, (byte)102, (byte)49, (byte)28, (byte)80, (byte)101, (byte)38, (byte)29, (byte)113, (byte)104, (byte)191, (byte)45, (byte)45, (byte)41, (byte)141, (byte)210, (byte)242, (byte)170, (byte)153, (byte)151, (byte)210, (byte)69, (byte)152, (byte)38, (byte)191, (byte)175, (byte)223, (byte)245, (byte)222, (byte)236, (byte)119, (byte)97, (byte)51, (byte)187, (byte)46, (byte)83, (byte)71, (byte)24, (byte)204, (byte)92, (byte)108, (byte)133, (byte)86, (byte)222, (byte)244, (byte)24, (byte)128, (byte)208, (byte)79, (byte)224, (byte)250, (byte)163, (byte)163, (byte)207, (byte)136, (byte)211, (byte)177, (byte)145, (byte)4, (byte)44, (byte)62, (byte)172, (byte)249, (byte)99, (byte)173, (byte)245, (byte)157, (byte)50, (byte)47, (byte)135, (byte)150, (byte)229, (byte)9, (byte)66, (byte)71, (byte)172, (byte)142, (byte)218, (byte)166, (byte)164, (byte)31, (byte)97, (byte)149, (byte)246, (byte)64, (byte)214, (byte)143, (byte)119, (byte)175, (byte)232, (byte)206, (byte)246, (byte)45, (byte)99, (byte)7, (byte)217, (byte)71, (byte)147, (byte)20, (byte)98, (byte)180, (byte)176, (byte)44, (byte)54, (byte)163}));
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.uri_SET(new byte[] {(byte)52, (byte)132, (byte)255, (byte)79, (byte)200, (byte)1, (byte)255, (byte)113, (byte)182, (byte)187, (byte)185, (byte)9, (byte)16, (byte)71, (byte)99, (byte)71, (byte)58, (byte)218, (byte)15, (byte)28, (byte)224, (byte)149, (byte)70, (byte)233, (byte)70, (byte)181, (byte)63, (byte)155, (byte)142, (byte)9, (byte)241, (byte)185, (byte)43, (byte)236, (byte)64, (byte)106, (byte)150, (byte)255, (byte)196, (byte)203, (byte)170, (byte)239, (byte)95, (byte)9, (byte)199, (byte)18, (byte)91, (byte)210, (byte)58, (byte)215, (byte)197, (byte)58, (byte)153, (byte)192, (byte)142, (byte)203, (byte)243, (byte)66, (byte)123, (byte)128, (byte)94, (byte)91, (byte)38, (byte)226, (byte)231, (byte)84, (byte)253, (byte)8, (byte)191, (byte)38, (byte)200, (byte)232, (byte)171, (byte)13, (byte)114, (byte)19, (byte)197, (byte)131, (byte)172, (byte)112, (byte)156, (byte)95, (byte)53, (byte)244, (byte)108, (byte)173, (byte)212, (byte)84, (byte)67, (byte)65, (byte)141, (byte)189, (byte)253, (byte)43, (byte)90, (byte)194, (byte)17, (byte)129, (byte)127, (byte)3, (byte)73, (byte)39, (byte)49, (byte)205, (byte)126, (byte)26, (byte)161, (byte)122, (byte)11, (byte)76, (byte)217, (byte)177, (byte)165, (byte)229, (byte)158, (byte)252, (byte)247, (byte)121, (byte)21, (byte)112}, 0) ;
            p142.storage_SET(new byte[] {(byte)116, (byte)206, (byte)97, (byte)231, (byte)71, (byte)170, (byte)40, (byte)188, (byte)143, (byte)224, (byte)180, (byte)56, (byte)80, (byte)171, (byte)60, (byte)189, (byte)102, (byte)49, (byte)28, (byte)80, (byte)101, (byte)38, (byte)29, (byte)113, (byte)104, (byte)191, (byte)45, (byte)45, (byte)41, (byte)141, (byte)210, (byte)242, (byte)170, (byte)153, (byte)151, (byte)210, (byte)69, (byte)152, (byte)38, (byte)191, (byte)175, (byte)223, (byte)245, (byte)222, (byte)236, (byte)119, (byte)97, (byte)51, (byte)187, (byte)46, (byte)83, (byte)71, (byte)24, (byte)204, (byte)92, (byte)108, (byte)133, (byte)86, (byte)222, (byte)244, (byte)24, (byte)128, (byte)208, (byte)79, (byte)224, (byte)250, (byte)163, (byte)163, (byte)207, (byte)136, (byte)211, (byte)177, (byte)145, (byte)4, (byte)44, (byte)62, (byte)172, (byte)249, (byte)99, (byte)173, (byte)245, (byte)157, (byte)50, (byte)47, (byte)135, (byte)150, (byte)229, (byte)9, (byte)66, (byte)71, (byte)172, (byte)142, (byte)218, (byte)166, (byte)164, (byte)31, (byte)97, (byte)149, (byte)246, (byte)64, (byte)214, (byte)143, (byte)119, (byte)175, (byte)232, (byte)206, (byte)246, (byte)45, (byte)99, (byte)7, (byte)217, (byte)71, (byte)147, (byte)20, (byte)98, (byte)180, (byte)176, (byte)44, (byte)54, (byte)163}, 0) ;
            p142.request_id = (byte)(byte)185;
            p142.uri_type = (byte)(byte)152;
            p142.transfer_type = (byte)(byte)150;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)23636);
                Debug.Assert(pack.press_diff == (float)3.1651647E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2938960421U);
                Debug.Assert(pack.press_abs == (float)3.2169478E38F);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.press_abs = (float)3.2169478E38F;
            p143.temperature = (short)(short)23636;
            p143.time_boot_ms = (uint)2938960421U;
            p143.press_diff = (float)3.1651647E38F;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.timestamp == (ulong)7571394919025453099L);
                Debug.Assert(pack.lon == (int) -249925114);
                Debug.Assert(pack.custom_state == (ulong)1101689478236906751L);
                Debug.Assert(pack.alt == (float)6.274197E37F);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-8.748616E37F, -2.0631184E38F, 2.026527E38F}));
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {1.8213965E38F, 2.7322706E38F, 2.5268475E38F}));
                Debug.Assert(pack.est_capabilities == (byte)(byte)137);
                Debug.Assert(pack.lat == (int)281220229);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {1.8809049E38F, 3.1758672E38F, 3.1233532E38F, 9.336999E37F}));
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-4.536789E37F, 1.7004774E38F, -2.0443622E38F}));
                Debug.Assert(pack.acc.SequenceEqual(new float[] {1.8132813E38F, -1.5702266E38F, -5.2390123E37F}));
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.est_capabilities = (byte)(byte)137;
            p144.custom_state = (ulong)1101689478236906751L;
            p144.timestamp = (ulong)7571394919025453099L;
            p144.alt = (float)6.274197E37F;
            p144.vel_SET(new float[] {-8.748616E37F, -2.0631184E38F, 2.026527E38F}, 0) ;
            p144.lon = (int) -249925114;
            p144.acc_SET(new float[] {1.8132813E38F, -1.5702266E38F, -5.2390123E37F}, 0) ;
            p144.position_cov_SET(new float[] {1.8213965E38F, 2.7322706E38F, 2.5268475E38F}, 0) ;
            p144.lat = (int)281220229;
            p144.attitude_q_SET(new float[] {1.8809049E38F, 3.1758672E38F, 3.1233532E38F, 9.336999E37F}, 0) ;
            p144.rates_SET(new float[] {-4.536789E37F, 1.7004774E38F, -2.0443622E38F}, 0) ;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x_acc == (float) -1.3207687E38F);
                Debug.Assert(pack.pitch_rate == (float)2.589606E38F);
                Debug.Assert(pack.time_usec == (ulong)8809773706219061629L);
                Debug.Assert(pack.y_acc == (float)2.465469E38F);
                Debug.Assert(pack.z_acc == (float)8.734438E35F);
                Debug.Assert(pack.y_pos == (float) -1.9090512E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.4751196E38F, -1.713269E38F, -3.07834E37F, -5.81004E36F}));
                Debug.Assert(pack.y_vel == (float) -2.7253036E38F);
                Debug.Assert(pack.x_pos == (float)2.002215E38F);
                Debug.Assert(pack.airspeed == (float) -1.3689274E38F);
                Debug.Assert(pack.z_vel == (float)2.7243339E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {1.3544069E38F, 2.6814903E38F, 1.0224293E38F}));
                Debug.Assert(pack.roll_rate == (float) -3.3495215E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {-9.789871E37F, -1.1717759E38F, -2.1492292E38F}));
                Debug.Assert(pack.z_pos == (float)1.1988601E38F);
                Debug.Assert(pack.yaw_rate == (float)9.053531E37F);
                Debug.Assert(pack.x_vel == (float) -2.2706196E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.x_vel = (float) -2.2706196E38F;
            p146.y_pos = (float) -1.9090512E38F;
            p146.roll_rate = (float) -3.3495215E38F;
            p146.yaw_rate = (float)9.053531E37F;
            p146.y_acc = (float)2.465469E38F;
            p146.x_acc = (float) -1.3207687E38F;
            p146.z_pos = (float)1.1988601E38F;
            p146.x_pos = (float)2.002215E38F;
            p146.pitch_rate = (float)2.589606E38F;
            p146.time_usec = (ulong)8809773706219061629L;
            p146.q_SET(new float[] {-2.4751196E38F, -1.713269E38F, -3.07834E37F, -5.81004E36F}, 0) ;
            p146.z_vel = (float)2.7243339E38F;
            p146.y_vel = (float) -2.7253036E38F;
            p146.airspeed = (float) -1.3689274E38F;
            p146.vel_variance_SET(new float[] {-9.789871E37F, -1.1717759E38F, -2.1492292E38F}, 0) ;
            p146.z_acc = (float)8.734438E35F;
            p146.pos_variance_SET(new float[] {1.3544069E38F, 2.6814903E38F, 1.0224293E38F}, 0) ;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_consumed == (int) -429343377);
                Debug.Assert(pack.battery_function == MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
                Debug.Assert(pack.type == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 84);
                Debug.Assert(pack.temperature == (short)(short)15598);
                Debug.Assert(pack.energy_consumed == (int)2092888924);
                Debug.Assert(pack.id == (byte)(byte)142);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)31364, (ushort)22144, (ushort)19000, (ushort)64059, (ushort)46709, (ushort)29621, (ushort)26459, (ushort)350, (ushort)46965, (ushort)34538}));
                Debug.Assert(pack.current_battery == (short)(short)25409);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO;
            p147.current_battery = (short)(short)25409;
            p147.temperature = (short)(short)15598;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD;
            p147.voltages_SET(new ushort[] {(ushort)31364, (ushort)22144, (ushort)19000, (ushort)64059, (ushort)46709, (ushort)29621, (ushort)26459, (ushort)350, (ushort)46965, (ushort)34538}, 0) ;
            p147.id = (byte)(byte)142;
            p147.battery_remaining = (sbyte)(sbyte) - 84;
            p147.current_consumed = (int) -429343377;
            p147.energy_consumed = (int)2092888924;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uid == (ulong)3115112514676728605L);
                Debug.Assert(pack.os_sw_version == (uint)1881326053U);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)240, (byte)6, (byte)201, (byte)11, (byte)201, (byte)232, (byte)109, (byte)21}));
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)122, (byte)81, (byte)12, (byte)136, (byte)107, (byte)100, (byte)245, (byte)162, (byte)20, (byte)28, (byte)224, (byte)246, (byte)169, (byte)119, (byte)176, (byte)11, (byte)24, (byte)87}));
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)17, (byte)108, (byte)156, (byte)148, (byte)35, (byte)13, (byte)43, (byte)250}));
                Debug.Assert(pack.vendor_id == (ushort)(ushort)1278);
                Debug.Assert(pack.middleware_sw_version == (uint)2224591382U);
                Debug.Assert(pack.board_version == (uint)1109501589U);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)242, (byte)167, (byte)195, (byte)234, (byte)59, (byte)57, (byte)71, (byte)125}));
                Debug.Assert(pack.product_id == (ushort)(ushort)31501);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE));
                Debug.Assert(pack.flight_sw_version == (uint)974998234U);
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.os_sw_version = (uint)1881326053U;
            p148.board_version = (uint)1109501589U;
            p148.product_id = (ushort)(ushort)31501;
            p148.middleware_custom_version_SET(new byte[] {(byte)240, (byte)6, (byte)201, (byte)11, (byte)201, (byte)232, (byte)109, (byte)21}, 0) ;
            p148.flight_sw_version = (uint)974998234U;
            p148.uid2_SET(new byte[] {(byte)122, (byte)81, (byte)12, (byte)136, (byte)107, (byte)100, (byte)245, (byte)162, (byte)20, (byte)28, (byte)224, (byte)246, (byte)169, (byte)119, (byte)176, (byte)11, (byte)24, (byte)87}, 0, PH) ;
            p148.uid = (ulong)3115112514676728605L;
            p148.vendor_id = (ushort)(ushort)1278;
            p148.flight_custom_version_SET(new byte[] {(byte)242, (byte)167, (byte)195, (byte)234, (byte)59, (byte)57, (byte)71, (byte)125}, 0) ;
            p148.middleware_sw_version = (uint)2224591382U;
            p148.os_custom_version_SET(new byte[] {(byte)17, (byte)108, (byte)156, (byte)148, (byte)35, (byte)13, (byte)43, (byte)250}, 0) ;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE);
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y_TRY(ph) == (float)1.2590755E38F);
                Debug.Assert(pack.size_y == (float) -4.8943244E37F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)57);
                Debug.Assert(pack.z_TRY(ph) == (float)1.7575499E38F);
                Debug.Assert(pack.size_x == (float)9.066239E37F);
                Debug.Assert(pack.distance == (float) -2.0018957E38F);
                Debug.Assert(pack.target_num == (byte)(byte)62);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {-3.0005476E38F, -2.1784348E38F, 2.9447931E38F, 2.6315879E38F}));
                Debug.Assert(pack.type == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
                Debug.Assert(pack.x_TRY(ph) == (float)3.0437613E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.angle_x == (float) -2.1289851E38F);
                Debug.Assert(pack.angle_y == (float) -2.3681001E38F);
                Debug.Assert(pack.time_usec == (ulong)1062377294293410829L);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.target_num = (byte)(byte)62;
            p149.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p149.x_SET((float)3.0437613E38F, PH) ;
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.size_y = (float) -4.8943244E37F;
            p149.time_usec = (ulong)1062377294293410829L;
            p149.z_SET((float)1.7575499E38F, PH) ;
            p149.distance = (float) -2.0018957E38F;
            p149.angle_y = (float) -2.3681001E38F;
            p149.q_SET(new float[] {-3.0005476E38F, -2.1784348E38F, 2.9447931E38F, 2.6315879E38F}, 0, PH) ;
            p149.y_SET((float)1.2590755E38F, PH) ;
            p149.position_valid_SET((byte)(byte)57, PH) ;
            p149.size_x = (float)9.066239E37F;
            p149.angle_x = (float) -2.1289851E38F;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAV_FILTER_BIASReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.accel_2 == (float)2.6601066E38F);
                Debug.Assert(pack.accel_0 == (float)2.4467942E38F);
                Debug.Assert(pack.accel_1 == (float) -9.985348E37F);
                Debug.Assert(pack.usec == (ulong)834070569810540933L);
                Debug.Assert(pack.gyro_2 == (float)1.3090458E38F);
                Debug.Assert(pack.gyro_1 == (float) -1.141581E38F);
                Debug.Assert(pack.gyro_0 == (float)1.1166877E38F);
            };
            GroundControl.NAV_FILTER_BIAS p220 = CommunicationChannel.new_NAV_FILTER_BIAS();
            PH.setPack(p220);
            p220.gyro_1 = (float) -1.141581E38F;
            p220.accel_2 = (float)2.6601066E38F;
            p220.accel_0 = (float)2.4467942E38F;
            p220.usec = (ulong)834070569810540933L;
            p220.gyro_0 = (float)1.1166877E38F;
            p220.accel_1 = (float) -9.985348E37F;
            p220.gyro_2 = (float)1.3090458E38F;
            CommunicationChannel.instance.send(p220);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRADIO_CALIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aileron.SequenceEqual(new ushort[] {(ushort)43622, (ushort)46184, (ushort)34123}));
                Debug.Assert(pack.elevator.SequenceEqual(new ushort[] {(ushort)29460, (ushort)21958, (ushort)50555}));
                Debug.Assert(pack.gyro.SequenceEqual(new ushort[] {(ushort)1386, (ushort)35863}));
                Debug.Assert(pack.pitch.SequenceEqual(new ushort[] {(ushort)41094, (ushort)42527, (ushort)28718, (ushort)52202, (ushort)3680}));
                Debug.Assert(pack.rudder.SequenceEqual(new ushort[] {(ushort)65467, (ushort)23752, (ushort)42230}));
                Debug.Assert(pack.throttle.SequenceEqual(new ushort[] {(ushort)45082, (ushort)3346, (ushort)30967, (ushort)12423, (ushort)24659}));
            };
            GroundControl.RADIO_CALIBRATION p221 = CommunicationChannel.new_RADIO_CALIBRATION();
            PH.setPack(p221);
            p221.throttle_SET(new ushort[] {(ushort)45082, (ushort)3346, (ushort)30967, (ushort)12423, (ushort)24659}, 0) ;
            p221.pitch_SET(new ushort[] {(ushort)41094, (ushort)42527, (ushort)28718, (ushort)52202, (ushort)3680}, 0) ;
            p221.elevator_SET(new ushort[] {(ushort)29460, (ushort)21958, (ushort)50555}, 0) ;
            p221.gyro_SET(new ushort[] {(ushort)1386, (ushort)35863}, 0) ;
            p221.aileron_SET(new ushort[] {(ushort)43622, (ushort)46184, (ushort)34123}, 0) ;
            p221.rudder_SET(new ushort[] {(ushort)65467, (ushort)23752, (ushort)42230}, 0) ;
            CommunicationChannel.instance.send(p221);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUALBERTA_SYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode == (byte)(byte)23);
                Debug.Assert(pack.pilot == (byte)(byte)236);
                Debug.Assert(pack.nav_mode == (byte)(byte)252);
            };
            GroundControl.UALBERTA_SYS_STATUS p222 = CommunicationChannel.new_UALBERTA_SYS_STATUS();
            PH.setPack(p222);
            p222.pilot = (byte)(byte)236;
            p222.nav_mode = (byte)(byte)252;
            p222.mode = (byte)(byte)23;
            CommunicationChannel.instance.send(p222);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hagl_ratio == (float)3.1309631E38F);
                Debug.Assert(pack.time_usec == (ulong)7362393548704095027L);
                Debug.Assert(pack.pos_vert_ratio == (float)2.3903302E38F);
                Debug.Assert(pack.vel_ratio == (float) -5.5045304E37F);
                Debug.Assert(pack.pos_horiz_ratio == (float) -2.5961162E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS));
                Debug.Assert(pack.pos_vert_accuracy == (float) -2.7241556E38F);
                Debug.Assert(pack.tas_ratio == (float)8.758376E37F);
                Debug.Assert(pack.mag_ratio == (float)7.879822E37F);
                Debug.Assert(pack.pos_horiz_accuracy == (float)2.5696176E38F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.pos_horiz_ratio = (float) -2.5961162E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS);
            p230.mag_ratio = (float)7.879822E37F;
            p230.time_usec = (ulong)7362393548704095027L;
            p230.vel_ratio = (float) -5.5045304E37F;
            p230.tas_ratio = (float)8.758376E37F;
            p230.hagl_ratio = (float)3.1309631E38F;
            p230.pos_vert_accuracy = (float) -2.7241556E38F;
            p230.pos_horiz_accuracy = (float)2.5696176E38F;
            p230.pos_vert_ratio = (float)2.3903302E38F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)1590395683981138347L);
                Debug.Assert(pack.wind_alt == (float) -8.2393263E37F);
                Debug.Assert(pack.var_horiz == (float) -2.7465733E38F);
                Debug.Assert(pack.wind_y == (float) -3.3291857E38F);
                Debug.Assert(pack.vert_accuracy == (float)3.1971558E38F);
                Debug.Assert(pack.wind_z == (float) -1.3077108E37F);
                Debug.Assert(pack.wind_x == (float)1.8057985E38F);
                Debug.Assert(pack.var_vert == (float) -2.794966E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -1.1292784E38F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.var_horiz = (float) -2.7465733E38F;
            p231.wind_y = (float) -3.3291857E38F;
            p231.wind_x = (float)1.8057985E38F;
            p231.wind_z = (float) -1.3077108E37F;
            p231.time_usec = (ulong)1590395683981138347L;
            p231.var_vert = (float) -2.794966E38F;
            p231.wind_alt = (float) -8.2393263E37F;
            p231.horiz_accuracy = (float) -1.1292784E38F;
            p231.vert_accuracy = (float)3.1971558E38F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vert_accuracy == (float)3.2878694E37F);
                Debug.Assert(pack.gps_id == (byte)(byte)131);
                Debug.Assert(pack.vdop == (float) -1.9888189E38F);
                Debug.Assert(pack.hdop == (float) -2.9618141E38F);
                Debug.Assert(pack.time_usec == (ulong)3885061752769583300L);
                Debug.Assert(pack.alt == (float) -2.1013361E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -2.6509398E37F);
                Debug.Assert(pack.fix_type == (byte)(byte)26);
                Debug.Assert(pack.lat == (int) -1527505070);
                Debug.Assert(pack.vd == (float) -3.1328021E38F);
                Debug.Assert(pack.time_week_ms == (uint)2905833927U);
                Debug.Assert(pack.vn == (float) -1.1475747E38F);
                Debug.Assert(pack.lon == (int) -1924810033);
                Debug.Assert(pack.time_week == (ushort)(ushort)45411);
                Debug.Assert(pack.satellites_visible == (byte)(byte)203);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT));
                Debug.Assert(pack.speed_accuracy == (float)1.8916965E38F);
                Debug.Assert(pack.ve == (float) -1.5066295E38F);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.gps_id = (byte)(byte)131;
            p232.time_week = (ushort)(ushort)45411;
            p232.vert_accuracy = (float)3.2878694E37F;
            p232.vdop = (float) -1.9888189E38F;
            p232.hdop = (float) -2.9618141E38F;
            p232.alt = (float) -2.1013361E38F;
            p232.vd = (float) -3.1328021E38F;
            p232.speed_accuracy = (float)1.8916965E38F;
            p232.vn = (float) -1.1475747E38F;
            p232.time_week_ms = (uint)2905833927U;
            p232.lon = (int) -1924810033;
            p232.lat = (int) -1527505070;
            p232.time_usec = (ulong)3885061752769583300L;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT);
            p232.ve = (float) -1.5066295E38F;
            p232.horiz_accuracy = (float) -2.6509398E37F;
            p232.fix_type = (byte)(byte)26;
            p232.satellites_visible = (byte)(byte)203;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)37, (byte)69, (byte)133, (byte)148, (byte)85, (byte)194, (byte)185, (byte)50, (byte)200, (byte)103, (byte)149, (byte)96, (byte)49, (byte)28, (byte)171, (byte)75, (byte)206, (byte)156, (byte)221, (byte)18, (byte)166, (byte)115, (byte)119, (byte)35, (byte)51, (byte)183, (byte)85, (byte)48, (byte)108, (byte)122, (byte)43, (byte)73, (byte)15, (byte)251, (byte)84, (byte)143, (byte)37, (byte)61, (byte)246, (byte)28, (byte)88, (byte)74, (byte)142, (byte)53, (byte)85, (byte)139, (byte)166, (byte)126, (byte)164, (byte)113, (byte)179, (byte)244, (byte)192, (byte)43, (byte)170, (byte)81, (byte)80, (byte)26, (byte)0, (byte)233, (byte)107, (byte)176, (byte)188, (byte)54, (byte)79, (byte)87, (byte)233, (byte)11, (byte)63, (byte)218, (byte)110, (byte)120, (byte)193, (byte)134, (byte)138, (byte)170, (byte)106, (byte)172, (byte)109, (byte)254, (byte)163, (byte)51, (byte)60, (byte)54, (byte)133, (byte)158, (byte)155, (byte)98, (byte)249, (byte)171, (byte)232, (byte)158, (byte)147, (byte)242, (byte)248, (byte)195, (byte)90, (byte)83, (byte)87, (byte)135, (byte)26, (byte)73, (byte)206, (byte)179, (byte)242, (byte)33, (byte)191, (byte)97, (byte)23, (byte)173, (byte)163, (byte)31, (byte)127, (byte)190, (byte)31, (byte)235, (byte)175, (byte)232, (byte)26, (byte)48, (byte)198, (byte)166, (byte)149, (byte)4, (byte)190, (byte)79, (byte)120, (byte)96, (byte)114, (byte)231, (byte)218, (byte)164, (byte)154, (byte)180, (byte)224, (byte)196, (byte)103, (byte)189, (byte)14, (byte)195, (byte)19, (byte)26, (byte)237, (byte)140, (byte)105, (byte)132, (byte)171, (byte)4, (byte)138, (byte)163, (byte)66, (byte)184, (byte)78, (byte)199, (byte)82, (byte)72, (byte)92, (byte)60, (byte)230, (byte)3, (byte)121, (byte)199, (byte)184, (byte)212, (byte)45, (byte)87, (byte)109, (byte)154, (byte)40, (byte)8, (byte)165, (byte)244, (byte)193, (byte)208, (byte)213, (byte)121, (byte)121, (byte)123, (byte)73, (byte)3}));
                Debug.Assert(pack.len == (byte)(byte)230);
                Debug.Assert(pack.flags == (byte)(byte)236);
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)236;
            p233.data__SET(new byte[] {(byte)37, (byte)69, (byte)133, (byte)148, (byte)85, (byte)194, (byte)185, (byte)50, (byte)200, (byte)103, (byte)149, (byte)96, (byte)49, (byte)28, (byte)171, (byte)75, (byte)206, (byte)156, (byte)221, (byte)18, (byte)166, (byte)115, (byte)119, (byte)35, (byte)51, (byte)183, (byte)85, (byte)48, (byte)108, (byte)122, (byte)43, (byte)73, (byte)15, (byte)251, (byte)84, (byte)143, (byte)37, (byte)61, (byte)246, (byte)28, (byte)88, (byte)74, (byte)142, (byte)53, (byte)85, (byte)139, (byte)166, (byte)126, (byte)164, (byte)113, (byte)179, (byte)244, (byte)192, (byte)43, (byte)170, (byte)81, (byte)80, (byte)26, (byte)0, (byte)233, (byte)107, (byte)176, (byte)188, (byte)54, (byte)79, (byte)87, (byte)233, (byte)11, (byte)63, (byte)218, (byte)110, (byte)120, (byte)193, (byte)134, (byte)138, (byte)170, (byte)106, (byte)172, (byte)109, (byte)254, (byte)163, (byte)51, (byte)60, (byte)54, (byte)133, (byte)158, (byte)155, (byte)98, (byte)249, (byte)171, (byte)232, (byte)158, (byte)147, (byte)242, (byte)248, (byte)195, (byte)90, (byte)83, (byte)87, (byte)135, (byte)26, (byte)73, (byte)206, (byte)179, (byte)242, (byte)33, (byte)191, (byte)97, (byte)23, (byte)173, (byte)163, (byte)31, (byte)127, (byte)190, (byte)31, (byte)235, (byte)175, (byte)232, (byte)26, (byte)48, (byte)198, (byte)166, (byte)149, (byte)4, (byte)190, (byte)79, (byte)120, (byte)96, (byte)114, (byte)231, (byte)218, (byte)164, (byte)154, (byte)180, (byte)224, (byte)196, (byte)103, (byte)189, (byte)14, (byte)195, (byte)19, (byte)26, (byte)237, (byte)140, (byte)105, (byte)132, (byte)171, (byte)4, (byte)138, (byte)163, (byte)66, (byte)184, (byte)78, (byte)199, (byte)82, (byte)72, (byte)92, (byte)60, (byte)230, (byte)3, (byte)121, (byte)199, (byte)184, (byte)212, (byte)45, (byte)87, (byte)109, (byte)154, (byte)40, (byte)8, (byte)165, (byte)244, (byte)193, (byte)208, (byte)213, (byte)121, (byte)121, (byte)123, (byte)73, (byte)3}, 0) ;
            p233.len = (byte)(byte)230;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (short)(short) -29866);
                Debug.Assert(pack.latitude == (int) -667510329);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
                Debug.Assert(pack.throttle == (sbyte)(sbyte) - 49);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)35963);
                Debug.Assert(pack.wp_num == (byte)(byte)15);
                Debug.Assert(pack.altitude_sp == (short)(short)5564);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
                Debug.Assert(pack.failsafe == (byte)(byte)4);
                Debug.Assert(pack.gps_fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
                Debug.Assert(pack.gps_nsat == (byte)(byte)112);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte)8);
                Debug.Assert(pack.battery_remaining == (byte)(byte)104);
                Debug.Assert(pack.groundspeed == (byte)(byte)152);
                Debug.Assert(pack.heading_sp == (short)(short)14370);
                Debug.Assert(pack.temperature == (sbyte)(sbyte)92);
                Debug.Assert(pack.pitch == (short)(short)10817);
                Debug.Assert(pack.longitude == (int) -1733398983);
                Debug.Assert(pack.airspeed == (byte)(byte)8);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)9);
                Debug.Assert(pack.altitude_amsl == (short)(short)18503);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)23);
                Debug.Assert(pack.heading == (ushort)(ushort)3450);
                Debug.Assert(pack.custom_mode == (uint)3141956678U);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.temperature_air = (sbyte)(sbyte)23;
            p234.battery_remaining = (byte)(byte)104;
            p234.temperature = (sbyte)(sbyte)92;
            p234.altitude_sp = (short)(short)5564;
            p234.custom_mode = (uint)3141956678U;
            p234.airspeed_sp = (byte)(byte)9;
            p234.pitch = (short)(short)10817;
            p234.climb_rate = (sbyte)(sbyte)8;
            p234.roll = (short)(short) -29866;
            p234.wp_distance = (ushort)(ushort)35963;
            p234.altitude_amsl = (short)(short)18503;
            p234.heading_sp = (short)(short)14370;
            p234.wp_num = (byte)(byte)15;
            p234.groundspeed = (byte)(byte)152;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            p234.latitude = (int) -667510329;
            p234.failsafe = (byte)(byte)4;
            p234.longitude = (int) -1733398983;
            p234.heading = (ushort)(ushort)3450;
            p234.gps_nsat = (byte)(byte)112;
            p234.throttle = (sbyte)(sbyte) - 49;
            p234.airspeed = (byte)(byte)8;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.clipping_1 == (uint)3847215244U);
                Debug.Assert(pack.vibration_x == (float)1.0774877E38F);
                Debug.Assert(pack.time_usec == (ulong)7639057198091649877L);
                Debug.Assert(pack.vibration_y == (float) -2.7713347E38F);
                Debug.Assert(pack.clipping_0 == (uint)3441359698U);
                Debug.Assert(pack.clipping_2 == (uint)1007988786U);
                Debug.Assert(pack.vibration_z == (float)2.5328807E38F);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.clipping_0 = (uint)3441359698U;
            p241.time_usec = (ulong)7639057198091649877L;
            p241.vibration_z = (float)2.5328807E38F;
            p241.clipping_1 = (uint)3847215244U;
            p241.vibration_y = (float) -2.7713347E38F;
            p241.clipping_2 = (uint)1007988786U;
            p241.vibration_x = (float)1.0774877E38F;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)2.2762962E38F);
                Debug.Assert(pack.approach_y == (float)4.7981823E37F);
                Debug.Assert(pack.approach_z == (float) -2.2885436E38F);
                Debug.Assert(pack.longitude == (int)1920535260);
                Debug.Assert(pack.latitude == (int)1402818503);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.4847049E38F, 3.3533847E38F, -2.6396453E38F, 1.5142306E38F}));
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)4434538217076958442L);
                Debug.Assert(pack.z == (float)2.3135726E37F);
                Debug.Assert(pack.x == (float) -2.7962054E38F);
                Debug.Assert(pack.altitude == (int) -1323379148);
                Debug.Assert(pack.approach_x == (float) -1.203279E38F);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.longitude = (int)1920535260;
            p242.approach_y = (float)4.7981823E37F;
            p242.q_SET(new float[] {1.4847049E38F, 3.3533847E38F, -2.6396453E38F, 1.5142306E38F}, 0) ;
            p242.time_usec_SET((ulong)4434538217076958442L, PH) ;
            p242.x = (float) -2.7962054E38F;
            p242.approach_x = (float) -1.203279E38F;
            p242.altitude = (int) -1323379148;
            p242.latitude = (int)1402818503;
            p242.approach_z = (float) -2.2885436E38F;
            p242.y = (float)2.2762962E38F;
            p242.z = (float)2.3135726E37F;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -1.6403385E38F);
                Debug.Assert(pack.altitude == (int)315165720);
                Debug.Assert(pack.approach_x == (float) -3.1025531E38F);
                Debug.Assert(pack.y == (float)2.854841E38F);
                Debug.Assert(pack.latitude == (int)1087785506);
                Debug.Assert(pack.longitude == (int)651813182);
                Debug.Assert(pack.approach_y == (float) -1.814022E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7438809458664452244L);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.1264048E38F, -3.9798195E37F, 2.7178728E37F, 2.0105776E38F}));
                Debug.Assert(pack.x == (float)2.6698036E38F);
                Debug.Assert(pack.approach_z == (float)1.0323436E38F);
                Debug.Assert(pack.target_system == (byte)(byte)30);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.approach_x = (float) -3.1025531E38F;
            p243.x = (float)2.6698036E38F;
            p243.target_system = (byte)(byte)30;
            p243.approach_y = (float) -1.814022E38F;
            p243.approach_z = (float)1.0323436E38F;
            p243.y = (float)2.854841E38F;
            p243.time_usec_SET((ulong)7438809458664452244L, PH) ;
            p243.longitude = (int)651813182;
            p243.altitude = (int)315165720;
            p243.q_SET(new float[] {1.1264048E38F, -3.9798195E37F, 2.7178728E37F, 2.0105776E38F}, 0) ;
            p243.z = (float) -1.6403385E38F;
            p243.latitude = (int)1087785506;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)54559);
                Debug.Assert(pack.interval_us == (int)714354242);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)54559;
            p244.interval_us = (int)714354242;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_MC;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -268241756);
                Debug.Assert(pack.heading == (ushort)(ushort)13311);
                Debug.Assert(pack.emitter_type == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_PARACHUTE);
                Debug.Assert(pack.altitude == (int)1408395524);
                Debug.Assert(pack.tslc == (byte)(byte)132);
                Debug.Assert(pack.ver_velocity == (short)(short) -11799);
                Debug.Assert(pack.altitude_type == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
                Debug.Assert(pack.lon == (int) -2042400877);
                Debug.Assert(pack.squawk == (ushort)(ushort)55209);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)59320);
                Debug.Assert(pack.callsign_LEN(ph) == 8);
                Debug.Assert(pack.callsign_TRY(ph).Equals("untcnchi"));
                Debug.Assert(pack.ICAO_address == (uint)1131517004U);
                Debug.Assert(pack.flags == (ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                                            ADSB_FLAGS.ADSB_FLAGS_SIMULATED));
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.hor_velocity = (ushort)(ushort)59320;
            p246.squawk = (ushort)(ushort)55209;
            p246.callsign_SET("untcnchi", PH) ;
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_PARACHUTE;
            p246.lon = (int) -2042400877;
            p246.lat = (int) -268241756;
            p246.ICAO_address = (uint)1131517004U;
            p246.altitude = (int)1408395524;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.tslc = (byte)(byte)132;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                          ADSB_FLAGS.ADSB_FLAGS_SIMULATED);
            p246.heading = (ushort)(ushort)13311;
            p246.ver_velocity = (short)(short) -11799;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.threat_level == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
                Debug.Assert(pack.action == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR);
                Debug.Assert(pack.time_to_minimum_delta == (float)1.306277E38F);
                Debug.Assert(pack.src_ == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
                Debug.Assert(pack.id == (uint)216444106U);
                Debug.Assert(pack.horizontal_minimum_delta == (float) -4.648292E37F);
                Debug.Assert(pack.altitude_minimum_delta == (float)5.904869E37F);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.horizontal_minimum_delta = (float) -4.648292E37F;
            p247.altitude_minimum_delta = (float)5.904869E37F;
            p247.threat_level = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH;
            p247.id = (uint)216444106U;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR;
            p247.time_to_minimum_delta = (float)1.306277E38F;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)6);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)51, (byte)38, (byte)51, (byte)215, (byte)81, (byte)75, (byte)27, (byte)236, (byte)248, (byte)42, (byte)70, (byte)233, (byte)132, (byte)62, (byte)227, (byte)170, (byte)103, (byte)186, (byte)56, (byte)224, (byte)206, (byte)187, (byte)247, (byte)98, (byte)55, (byte)245, (byte)0, (byte)63, (byte)60, (byte)102, (byte)185, (byte)149, (byte)185, (byte)62, (byte)90, (byte)30, (byte)177, (byte)54, (byte)32, (byte)209, (byte)138, (byte)168, (byte)33, (byte)163, (byte)231, (byte)161, (byte)78, (byte)254, (byte)48, (byte)41, (byte)15, (byte)167, (byte)165, (byte)99, (byte)33, (byte)245, (byte)230, (byte)174, (byte)53, (byte)185, (byte)46, (byte)196, (byte)213, (byte)182, (byte)181, (byte)65, (byte)136, (byte)157, (byte)58, (byte)204, (byte)145, (byte)139, (byte)191, (byte)194, (byte)200, (byte)116, (byte)43, (byte)215, (byte)116, (byte)79, (byte)68, (byte)23, (byte)116, (byte)232, (byte)111, (byte)228, (byte)3, (byte)13, (byte)6, (byte)170, (byte)117, (byte)212, (byte)48, (byte)178, (byte)15, (byte)199, (byte)128, (byte)220, (byte)246, (byte)233, (byte)39, (byte)51, (byte)5, (byte)70, (byte)243, (byte)215, (byte)104, (byte)169, (byte)69, (byte)57, (byte)89, (byte)244, (byte)27, (byte)104, (byte)129, (byte)38, (byte)144, (byte)161, (byte)231, (byte)92, (byte)3, (byte)144, (byte)54, (byte)187, (byte)132, (byte)119, (byte)70, (byte)137, (byte)193, (byte)146, (byte)171, (byte)3, (byte)68, (byte)211, (byte)47, (byte)29, (byte)15, (byte)138, (byte)110, (byte)66, (byte)83, (byte)209, (byte)49, (byte)140, (byte)168, (byte)225, (byte)12, (byte)148, (byte)42, (byte)117, (byte)20, (byte)64, (byte)114, (byte)108, (byte)19, (byte)145, (byte)245, (byte)242, (byte)180, (byte)26, (byte)224, (byte)28, (byte)9, (byte)32, (byte)250, (byte)162, (byte)213, (byte)66, (byte)144, (byte)6, (byte)107, (byte)164, (byte)200, (byte)69, (byte)169, (byte)160, (byte)106, (byte)75, (byte)192, (byte)239, (byte)75, (byte)43, (byte)237, (byte)220, (byte)91, (byte)10, (byte)8, (byte)136, (byte)86, (byte)172, (byte)11, (byte)54, (byte)240, (byte)22, (byte)147, (byte)72, (byte)87, (byte)238, (byte)195, (byte)178, (byte)139, (byte)169, (byte)23, (byte)244, (byte)41, (byte)138, (byte)120, (byte)173, (byte)63, (byte)107, (byte)72, (byte)53, (byte)132, (byte)82, (byte)154, (byte)161, (byte)19, (byte)77, (byte)68, (byte)137, (byte)182, (byte)112, (byte)230, (byte)58, (byte)191, (byte)1, (byte)1, (byte)15, (byte)209, (byte)184, (byte)68, (byte)204, (byte)7, (byte)144, (byte)96, (byte)93, (byte)79, (byte)89, (byte)58, (byte)4, (byte)30, (byte)141, (byte)109, (byte)21, (byte)100, (byte)198, (byte)143, (byte)90, (byte)152}));
                Debug.Assert(pack.target_system == (byte)(byte)110);
                Debug.Assert(pack.message_type == (ushort)(ushort)41050);
                Debug.Assert(pack.target_network == (byte)(byte)83);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)83;
            p248.message_type = (ushort)(ushort)41050;
            p248.payload_SET(new byte[] {(byte)51, (byte)38, (byte)51, (byte)215, (byte)81, (byte)75, (byte)27, (byte)236, (byte)248, (byte)42, (byte)70, (byte)233, (byte)132, (byte)62, (byte)227, (byte)170, (byte)103, (byte)186, (byte)56, (byte)224, (byte)206, (byte)187, (byte)247, (byte)98, (byte)55, (byte)245, (byte)0, (byte)63, (byte)60, (byte)102, (byte)185, (byte)149, (byte)185, (byte)62, (byte)90, (byte)30, (byte)177, (byte)54, (byte)32, (byte)209, (byte)138, (byte)168, (byte)33, (byte)163, (byte)231, (byte)161, (byte)78, (byte)254, (byte)48, (byte)41, (byte)15, (byte)167, (byte)165, (byte)99, (byte)33, (byte)245, (byte)230, (byte)174, (byte)53, (byte)185, (byte)46, (byte)196, (byte)213, (byte)182, (byte)181, (byte)65, (byte)136, (byte)157, (byte)58, (byte)204, (byte)145, (byte)139, (byte)191, (byte)194, (byte)200, (byte)116, (byte)43, (byte)215, (byte)116, (byte)79, (byte)68, (byte)23, (byte)116, (byte)232, (byte)111, (byte)228, (byte)3, (byte)13, (byte)6, (byte)170, (byte)117, (byte)212, (byte)48, (byte)178, (byte)15, (byte)199, (byte)128, (byte)220, (byte)246, (byte)233, (byte)39, (byte)51, (byte)5, (byte)70, (byte)243, (byte)215, (byte)104, (byte)169, (byte)69, (byte)57, (byte)89, (byte)244, (byte)27, (byte)104, (byte)129, (byte)38, (byte)144, (byte)161, (byte)231, (byte)92, (byte)3, (byte)144, (byte)54, (byte)187, (byte)132, (byte)119, (byte)70, (byte)137, (byte)193, (byte)146, (byte)171, (byte)3, (byte)68, (byte)211, (byte)47, (byte)29, (byte)15, (byte)138, (byte)110, (byte)66, (byte)83, (byte)209, (byte)49, (byte)140, (byte)168, (byte)225, (byte)12, (byte)148, (byte)42, (byte)117, (byte)20, (byte)64, (byte)114, (byte)108, (byte)19, (byte)145, (byte)245, (byte)242, (byte)180, (byte)26, (byte)224, (byte)28, (byte)9, (byte)32, (byte)250, (byte)162, (byte)213, (byte)66, (byte)144, (byte)6, (byte)107, (byte)164, (byte)200, (byte)69, (byte)169, (byte)160, (byte)106, (byte)75, (byte)192, (byte)239, (byte)75, (byte)43, (byte)237, (byte)220, (byte)91, (byte)10, (byte)8, (byte)136, (byte)86, (byte)172, (byte)11, (byte)54, (byte)240, (byte)22, (byte)147, (byte)72, (byte)87, (byte)238, (byte)195, (byte)178, (byte)139, (byte)169, (byte)23, (byte)244, (byte)41, (byte)138, (byte)120, (byte)173, (byte)63, (byte)107, (byte)72, (byte)53, (byte)132, (byte)82, (byte)154, (byte)161, (byte)19, (byte)77, (byte)68, (byte)137, (byte)182, (byte)112, (byte)230, (byte)58, (byte)191, (byte)1, (byte)1, (byte)15, (byte)209, (byte)184, (byte)68, (byte)204, (byte)7, (byte)144, (byte)96, (byte)93, (byte)79, (byte)89, (byte)58, (byte)4, (byte)30, (byte)141, (byte)109, (byte)21, (byte)100, (byte)198, (byte)143, (byte)90, (byte)152}, 0) ;
            p248.target_system = (byte)(byte)110;
            p248.target_component = (byte)(byte)6;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ver == (byte)(byte)61);
                Debug.Assert(pack.address == (ushort)(ushort)6439);
                Debug.Assert(pack.type == (byte)(byte)239);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte)40, (sbyte)23, (sbyte)8, (sbyte) - 107, (sbyte) - 110, (sbyte) - 124, (sbyte) - 15, (sbyte) - 9, (sbyte)10, (sbyte)83, (sbyte) - 99, (sbyte) - 98, (sbyte)115, (sbyte) - 1, (sbyte)107, (sbyte) - 39, (sbyte) - 47, (sbyte) - 43, (sbyte)87, (sbyte)123, (sbyte) - 1, (sbyte) - 33, (sbyte)42, (sbyte) - 125, (sbyte) - 57, (sbyte) - 22, (sbyte)92, (sbyte) - 118, (sbyte)82, (sbyte) - 102, (sbyte)97, (sbyte)116}));
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.type = (byte)(byte)239;
            p249.value_SET(new sbyte[] {(sbyte)40, (sbyte)23, (sbyte)8, (sbyte) - 107, (sbyte) - 110, (sbyte) - 124, (sbyte) - 15, (sbyte) - 9, (sbyte)10, (sbyte)83, (sbyte) - 99, (sbyte) - 98, (sbyte)115, (sbyte) - 1, (sbyte)107, (sbyte) - 39, (sbyte) - 47, (sbyte) - 43, (sbyte)87, (sbyte)123, (sbyte) - 1, (sbyte) - 33, (sbyte)42, (sbyte) - 125, (sbyte) - 57, (sbyte) - 22, (sbyte)92, (sbyte) - 118, (sbyte)82, (sbyte) - 102, (sbyte)97, (sbyte)116}, 0) ;
            p249.ver = (byte)(byte)61;
            p249.address = (ushort)(ushort)6439;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -2.2208456E38F);
                Debug.Assert(pack.time_usec == (ulong)7780878800839784616L);
                Debug.Assert(pack.y == (float)9.519753E37F);
                Debug.Assert(pack.x == (float) -1.5748046E38F);
                Debug.Assert(pack.name_LEN(ph) == 7);
                Debug.Assert(pack.name_TRY(ph).Equals("ctlzrwr"));
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.x = (float) -1.5748046E38F;
            p250.name_SET("ctlzrwr", PH) ;
            p250.y = (float)9.519753E37F;
            p250.time_usec = (ulong)7780878800839784616L;
            p250.z = (float) -2.2208456E38F;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2608957034U);
                Debug.Assert(pack.value == (float) -6.51323E37F);
                Debug.Assert(pack.name_LEN(ph) == 6);
                Debug.Assert(pack.name_TRY(ph).Equals("pshvhj"));
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.value = (float) -6.51323E37F;
            p251.time_boot_ms = (uint)2608957034U;
            p251.name_SET("pshvhj", PH) ;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1438021894U);
                Debug.Assert(pack.value == (int)1223926746);
                Debug.Assert(pack.name_LEN(ph) == 3);
                Debug.Assert(pack.name_TRY(ph).Equals("vWj"));
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.name_SET("vWj", PH) ;
            p252.value = (int)1223926746;
            p252.time_boot_ms = (uint)1438021894U;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 2);
                Debug.Assert(pack.text_TRY(ph).Equals("Eo"));
                Debug.Assert(pack.severity == MAV_SEVERITY.MAV_SEVERITY_ERROR);
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_ERROR;
            p253.text_SET("Eo", PH) ;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3062482532U);
                Debug.Assert(pack.ind == (byte)(byte)93);
                Debug.Assert(pack.value == (float)7.5302325E37F);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)3062482532U;
            p254.value = (float)7.5302325E37F;
            p254.ind = (byte)(byte)93;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.initial_timestamp == (ulong)8390850949540024736L);
                Debug.Assert(pack.target_system == (byte)(byte)3);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)8, (byte)33, (byte)213, (byte)64, (byte)97, (byte)92, (byte)89, (byte)222, (byte)87, (byte)210, (byte)114, (byte)40, (byte)140, (byte)116, (byte)38, (byte)99, (byte)112, (byte)141, (byte)54, (byte)33, (byte)164, (byte)84, (byte)137, (byte)86, (byte)43, (byte)210, (byte)228, (byte)156, (byte)160, (byte)140, (byte)62, (byte)193}));
                Debug.Assert(pack.target_component == (byte)(byte)235);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.initial_timestamp = (ulong)8390850949540024736L;
            p256.target_system = (byte)(byte)3;
            p256.target_component = (byte)(byte)235;
            p256.secret_key_SET(new byte[] {(byte)8, (byte)33, (byte)213, (byte)64, (byte)97, (byte)92, (byte)89, (byte)222, (byte)87, (byte)210, (byte)114, (byte)40, (byte)140, (byte)116, (byte)38, (byte)99, (byte)112, (byte)141, (byte)54, (byte)33, (byte)164, (byte)84, (byte)137, (byte)86, (byte)43, (byte)210, (byte)228, (byte)156, (byte)160, (byte)140, (byte)62, (byte)193}, 0) ;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.state == (byte)(byte)144);
                Debug.Assert(pack.time_boot_ms == (uint)3044537662U);
                Debug.Assert(pack.last_change_ms == (uint)155085940U);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.state = (byte)(byte)144;
            p257.last_change_ms = (uint)155085940U;
            p257.time_boot_ms = (uint)3044537662U;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)115);
                Debug.Assert(pack.target_component == (byte)(byte)82);
                Debug.Assert(pack.tune_LEN(ph) == 21);
                Debug.Assert(pack.tune_TRY(ph).Equals("ntXPrnsnvYYwpmmrnkDei"));
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)115;
            p258.target_component = (byte)(byte)82;
            p258.tune_SET("ntXPrnsnvYYwpmmrnkDei", PH) ;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.focal_length == (float)1.9410737E38F);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 25);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("qofvwfielRvqgeskgvuqfdtay"));
                Debug.Assert(pack.sensor_size_v == (float)2.4537677E38F);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)218, (byte)28, (byte)82, (byte)40, (byte)56, (byte)37, (byte)154, (byte)70, (byte)108, (byte)10, (byte)4, (byte)78, (byte)236, (byte)70, (byte)149, (byte)9, (byte)81, (byte)117, (byte)86, (byte)154, (byte)117, (byte)238, (byte)92, (byte)3, (byte)33, (byte)119, (byte)254, (byte)79, (byte)165, (byte)10, (byte)101, (byte)105}));
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES));
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)7577);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)45603);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)33068);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)183, (byte)123, (byte)89, (byte)234, (byte)204, (byte)39, (byte)161, (byte)106, (byte)248, (byte)215, (byte)251, (byte)115, (byte)201, (byte)35, (byte)218, (byte)25, (byte)183, (byte)238, (byte)198, (byte)202, (byte)74, (byte)255, (byte)23, (byte)156, (byte)38, (byte)195, (byte)104, (byte)161, (byte)62, (byte)109, (byte)135, (byte)119}));
                Debug.Assert(pack.sensor_size_h == (float) -1.3557945E38F);
                Debug.Assert(pack.lens_id == (byte)(byte)206);
                Debug.Assert(pack.firmware_version == (uint)3796528287U);
                Debug.Assert(pack.time_boot_ms == (uint)3782369794U);
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES);
            p259.cam_definition_uri_SET("qofvwfielRvqgeskgvuqfdtay", PH) ;
            p259.resolution_v = (ushort)(ushort)33068;
            p259.time_boot_ms = (uint)3782369794U;
            p259.sensor_size_v = (float)2.4537677E38F;
            p259.sensor_size_h = (float) -1.3557945E38F;
            p259.firmware_version = (uint)3796528287U;
            p259.cam_definition_version = (ushort)(ushort)7577;
            p259.focal_length = (float)1.9410737E38F;
            p259.resolution_h = (ushort)(ushort)45603;
            p259.model_name_SET(new byte[] {(byte)183, (byte)123, (byte)89, (byte)234, (byte)204, (byte)39, (byte)161, (byte)106, (byte)248, (byte)215, (byte)251, (byte)115, (byte)201, (byte)35, (byte)218, (byte)25, (byte)183, (byte)238, (byte)198, (byte)202, (byte)74, (byte)255, (byte)23, (byte)156, (byte)38, (byte)195, (byte)104, (byte)161, (byte)62, (byte)109, (byte)135, (byte)119}, 0) ;
            p259.lens_id = (byte)(byte)206;
            p259.vendor_name_SET(new byte[] {(byte)218, (byte)28, (byte)82, (byte)40, (byte)56, (byte)37, (byte)154, (byte)70, (byte)108, (byte)10, (byte)4, (byte)78, (byte)236, (byte)70, (byte)149, (byte)9, (byte)81, (byte)117, (byte)86, (byte)154, (byte)117, (byte)238, (byte)92, (byte)3, (byte)33, (byte)119, (byte)254, (byte)79, (byte)165, (byte)10, (byte)101, (byte)105}, 0) ;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == CAMERA_MODE.CAMERA_MODE_IMAGE);
                Debug.Assert(pack.time_boot_ms == (uint)2969042203U);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = CAMERA_MODE.CAMERA_MODE_IMAGE;
            p260.time_boot_ms = (uint)2969042203U;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.total_capacity == (float) -3.1410352E38F);
                Debug.Assert(pack.available_capacity == (float) -1.7210121E38F);
                Debug.Assert(pack.storage_id == (byte)(byte)121);
                Debug.Assert(pack.read_speed == (float)2.666822E38F);
                Debug.Assert(pack.write_speed == (float) -1.0029672E38F);
                Debug.Assert(pack.storage_count == (byte)(byte)96);
                Debug.Assert(pack.status == (byte)(byte)201);
                Debug.Assert(pack.time_boot_ms == (uint)559169981U);
                Debug.Assert(pack.used_capacity == (float) -2.4376832E38F);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.available_capacity = (float) -1.7210121E38F;
            p261.total_capacity = (float) -3.1410352E38F;
            p261.read_speed = (float)2.666822E38F;
            p261.status = (byte)(byte)201;
            p261.write_speed = (float) -1.0029672E38F;
            p261.storage_id = (byte)(byte)121;
            p261.used_capacity = (float) -2.4376832E38F;
            p261.storage_count = (byte)(byte)96;
            p261.time_boot_ms = (uint)559169981U;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.video_status == (byte)(byte)252);
                Debug.Assert(pack.image_interval == (float) -1.957306E38F);
                Debug.Assert(pack.image_status == (byte)(byte)187);
                Debug.Assert(pack.available_capacity == (float) -2.0345275E38F);
                Debug.Assert(pack.time_boot_ms == (uint)152471666U);
                Debug.Assert(pack.recording_time_ms == (uint)1222994489U);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.available_capacity = (float) -2.0345275E38F;
            p262.video_status = (byte)(byte)252;
            p262.image_status = (byte)(byte)187;
            p262.image_interval = (float) -1.957306E38F;
            p262.time_boot_ms = (uint)152471666U;
            p262.recording_time_ms = (uint)1222994489U;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.capture_result == (sbyte)(sbyte) - 12);
                Debug.Assert(pack.file_url_LEN(ph) == 141);
                Debug.Assert(pack.file_url_TRY(ph).Equals("mfnhHuvlDrhxhwijnyujajjprhpyswdwkarxflcpvqhclfljgxpxiqsrphfycAnthgQujfbpzyywlydhznzadhQbfcvwnhuzgmjiwrpvjhctqbqhzwiezicgipuqzzzTfateegeTppfvg"));
                Debug.Assert(pack.image_index == (int) -1929343780);
                Debug.Assert(pack.camera_id == (byte)(byte)105);
                Debug.Assert(pack.time_boot_ms == (uint)2124070279U);
                Debug.Assert(pack.relative_alt == (int)1127630921);
                Debug.Assert(pack.alt == (int) -1162632248);
                Debug.Assert(pack.time_utc == (ulong)3388841926845139422L);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.6538799E38F, -1.7846306E38F, -5.8403685E37F, 2.18037E38F}));
                Debug.Assert(pack.lon == (int)1354860933);
                Debug.Assert(pack.lat == (int)2046750284);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.lat = (int)2046750284;
            p263.camera_id = (byte)(byte)105;
            p263.time_boot_ms = (uint)2124070279U;
            p263.image_index = (int) -1929343780;
            p263.file_url_SET("mfnhHuvlDrhxhwijnyujajjprhpyswdwkarxflcpvqhclfljgxpxiqsrphfycAnthgQujfbpzyywlydhznzadhQbfcvwnhuzgmjiwrpvjhctqbqhzwiezicgipuqzzzTfateegeTppfvg", PH) ;
            p263.capture_result = (sbyte)(sbyte) - 12;
            p263.alt = (int) -1162632248;
            p263.time_utc = (ulong)3388841926845139422L;
            p263.relative_alt = (int)1127630921;
            p263.q_SET(new float[] {2.6538799E38F, -1.7846306E38F, -5.8403685E37F, 2.18037E38F}, 0) ;
            p263.lon = (int)1354860933;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2754829300U);
                Debug.Assert(pack.arming_time_utc == (ulong)517065658031682275L);
                Debug.Assert(pack.flight_uuid == (ulong)1196607106730207200L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)2596653778007276528L);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.takeoff_time_utc = (ulong)2596653778007276528L;
            p264.time_boot_ms = (uint)2754829300U;
            p264.flight_uuid = (ulong)1196607106730207200L;
            p264.arming_time_utc = (ulong)517065658031682275L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float) -2.6045115E38F);
                Debug.Assert(pack.yaw == (float)2.3865721E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2508101245U);
                Debug.Assert(pack.pitch == (float) -2.6925085E38F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.yaw = (float)2.3865721E38F;
            p265.roll = (float) -2.6045115E38F;
            p265.pitch = (float) -2.6925085E38F;
            p265.time_boot_ms = (uint)2508101245U;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)21);
                Debug.Assert(pack.sequence == (ushort)(ushort)5388);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)129, (byte)43, (byte)176, (byte)254, (byte)200, (byte)129, (byte)193, (byte)171, (byte)221, (byte)233, (byte)77, (byte)83, (byte)215, (byte)108, (byte)162, (byte)35, (byte)131, (byte)221, (byte)24, (byte)226, (byte)203, (byte)192, (byte)79, (byte)211, (byte)55, (byte)157, (byte)14, (byte)226, (byte)174, (byte)57, (byte)57, (byte)229, (byte)198, (byte)120, (byte)126, (byte)233, (byte)102, (byte)77, (byte)136, (byte)116, (byte)250, (byte)203, (byte)186, (byte)46, (byte)230, (byte)189, (byte)147, (byte)46, (byte)177, (byte)114, (byte)56, (byte)144, (byte)53, (byte)173, (byte)255, (byte)131, (byte)202, (byte)152, (byte)88, (byte)97, (byte)76, (byte)143, (byte)209, (byte)134, (byte)29, (byte)237, (byte)143, (byte)89, (byte)162, (byte)37, (byte)122, (byte)51, (byte)95, (byte)210, (byte)224, (byte)181, (byte)54, (byte)197, (byte)21, (byte)25, (byte)139, (byte)45, (byte)56, (byte)3, (byte)179, (byte)170, (byte)150, (byte)16, (byte)20, (byte)159, (byte)162, (byte)60, (byte)81, (byte)146, (byte)118, (byte)96, (byte)196, (byte)212, (byte)35, (byte)255, (byte)137, (byte)42, (byte)78, (byte)22, (byte)185, (byte)205, (byte)49, (byte)118, (byte)52, (byte)253, (byte)255, (byte)6, (byte)176, (byte)110, (byte)93, (byte)21, (byte)237, (byte)141, (byte)25, (byte)113, (byte)157, (byte)105, (byte)222, (byte)74, (byte)65, (byte)155, (byte)39, (byte)38, (byte)87, (byte)107, (byte)50, (byte)102, (byte)66, (byte)190, (byte)230, (byte)214, (byte)76, (byte)112, (byte)249, (byte)58, (byte)168, (byte)242, (byte)7, (byte)60, (byte)169, (byte)254, (byte)180, (byte)247, (byte)81, (byte)243, (byte)81, (byte)183, (byte)201, (byte)174, (byte)193, (byte)30, (byte)171, (byte)182, (byte)236, (byte)137, (byte)136, (byte)83, (byte)65, (byte)170, (byte)73, (byte)247, (byte)167, (byte)251, (byte)220, (byte)115, (byte)90, (byte)104, (byte)122, (byte)248, (byte)61, (byte)106, (byte)246, (byte)61, (byte)131, (byte)155, (byte)174, (byte)186, (byte)90, (byte)18, (byte)71, (byte)109, (byte)197, (byte)170, (byte)52, (byte)6, (byte)26, (byte)67, (byte)40, (byte)124, (byte)13, (byte)52, (byte)245, (byte)212, (byte)183, (byte)174, (byte)113, (byte)140, (byte)155, (byte)1, (byte)227, (byte)135, (byte)240, (byte)180, (byte)97, (byte)212, (byte)79, (byte)134, (byte)99, (byte)30, (byte)253, (byte)15, (byte)103, (byte)91, (byte)41, (byte)221, (byte)9, (byte)103, (byte)111, (byte)155, (byte)238, (byte)43, (byte)31, (byte)34, (byte)98, (byte)248, (byte)14, (byte)51, (byte)119, (byte)111, (byte)207, (byte)246, (byte)84, (byte)125, (byte)197, (byte)13, (byte)234, (byte)209, (byte)62, (byte)115, (byte)246, (byte)10, (byte)149, (byte)226, (byte)252}));
                Debug.Assert(pack.target_system == (byte)(byte)98);
                Debug.Assert(pack.length == (byte)(byte)106);
                Debug.Assert(pack.first_message_offset == (byte)(byte)167);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)98;
            p266.target_component = (byte)(byte)21;
            p266.length = (byte)(byte)106;
            p266.first_message_offset = (byte)(byte)167;
            p266.data__SET(new byte[] {(byte)129, (byte)43, (byte)176, (byte)254, (byte)200, (byte)129, (byte)193, (byte)171, (byte)221, (byte)233, (byte)77, (byte)83, (byte)215, (byte)108, (byte)162, (byte)35, (byte)131, (byte)221, (byte)24, (byte)226, (byte)203, (byte)192, (byte)79, (byte)211, (byte)55, (byte)157, (byte)14, (byte)226, (byte)174, (byte)57, (byte)57, (byte)229, (byte)198, (byte)120, (byte)126, (byte)233, (byte)102, (byte)77, (byte)136, (byte)116, (byte)250, (byte)203, (byte)186, (byte)46, (byte)230, (byte)189, (byte)147, (byte)46, (byte)177, (byte)114, (byte)56, (byte)144, (byte)53, (byte)173, (byte)255, (byte)131, (byte)202, (byte)152, (byte)88, (byte)97, (byte)76, (byte)143, (byte)209, (byte)134, (byte)29, (byte)237, (byte)143, (byte)89, (byte)162, (byte)37, (byte)122, (byte)51, (byte)95, (byte)210, (byte)224, (byte)181, (byte)54, (byte)197, (byte)21, (byte)25, (byte)139, (byte)45, (byte)56, (byte)3, (byte)179, (byte)170, (byte)150, (byte)16, (byte)20, (byte)159, (byte)162, (byte)60, (byte)81, (byte)146, (byte)118, (byte)96, (byte)196, (byte)212, (byte)35, (byte)255, (byte)137, (byte)42, (byte)78, (byte)22, (byte)185, (byte)205, (byte)49, (byte)118, (byte)52, (byte)253, (byte)255, (byte)6, (byte)176, (byte)110, (byte)93, (byte)21, (byte)237, (byte)141, (byte)25, (byte)113, (byte)157, (byte)105, (byte)222, (byte)74, (byte)65, (byte)155, (byte)39, (byte)38, (byte)87, (byte)107, (byte)50, (byte)102, (byte)66, (byte)190, (byte)230, (byte)214, (byte)76, (byte)112, (byte)249, (byte)58, (byte)168, (byte)242, (byte)7, (byte)60, (byte)169, (byte)254, (byte)180, (byte)247, (byte)81, (byte)243, (byte)81, (byte)183, (byte)201, (byte)174, (byte)193, (byte)30, (byte)171, (byte)182, (byte)236, (byte)137, (byte)136, (byte)83, (byte)65, (byte)170, (byte)73, (byte)247, (byte)167, (byte)251, (byte)220, (byte)115, (byte)90, (byte)104, (byte)122, (byte)248, (byte)61, (byte)106, (byte)246, (byte)61, (byte)131, (byte)155, (byte)174, (byte)186, (byte)90, (byte)18, (byte)71, (byte)109, (byte)197, (byte)170, (byte)52, (byte)6, (byte)26, (byte)67, (byte)40, (byte)124, (byte)13, (byte)52, (byte)245, (byte)212, (byte)183, (byte)174, (byte)113, (byte)140, (byte)155, (byte)1, (byte)227, (byte)135, (byte)240, (byte)180, (byte)97, (byte)212, (byte)79, (byte)134, (byte)99, (byte)30, (byte)253, (byte)15, (byte)103, (byte)91, (byte)41, (byte)221, (byte)9, (byte)103, (byte)111, (byte)155, (byte)238, (byte)43, (byte)31, (byte)34, (byte)98, (byte)248, (byte)14, (byte)51, (byte)119, (byte)111, (byte)207, (byte)246, (byte)84, (byte)125, (byte)197, (byte)13, (byte)234, (byte)209, (byte)62, (byte)115, (byte)246, (byte)10, (byte)149, (byte)226, (byte)252}, 0) ;
            p266.sequence = (ushort)(ushort)5388;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)106);
                Debug.Assert(pack.length == (byte)(byte)137);
                Debug.Assert(pack.sequence == (ushort)(ushort)7563);
                Debug.Assert(pack.first_message_offset == (byte)(byte)135);
                Debug.Assert(pack.target_system == (byte)(byte)125);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)189, (byte)228, (byte)93, (byte)126, (byte)138, (byte)151, (byte)48, (byte)176, (byte)134, (byte)157, (byte)230, (byte)219, (byte)238, (byte)221, (byte)217, (byte)75, (byte)117, (byte)153, (byte)101, (byte)15, (byte)120, (byte)177, (byte)164, (byte)177, (byte)232, (byte)145, (byte)10, (byte)115, (byte)41, (byte)49, (byte)192, (byte)136, (byte)220, (byte)227, (byte)12, (byte)204, (byte)133, (byte)115, (byte)88, (byte)29, (byte)163, (byte)154, (byte)38, (byte)101, (byte)136, (byte)140, (byte)239, (byte)210, (byte)156, (byte)92, (byte)55, (byte)201, (byte)43, (byte)164, (byte)72, (byte)229, (byte)166, (byte)217, (byte)189, (byte)123, (byte)154, (byte)227, (byte)157, (byte)11, (byte)6, (byte)110, (byte)35, (byte)137, (byte)52, (byte)25, (byte)213, (byte)205, (byte)40, (byte)21, (byte)206, (byte)137, (byte)185, (byte)180, (byte)114, (byte)97, (byte)245, (byte)214, (byte)179, (byte)153, (byte)24, (byte)169, (byte)135, (byte)29, (byte)64, (byte)174, (byte)56, (byte)84, (byte)105, (byte)205, (byte)213, (byte)134, (byte)37, (byte)157, (byte)13, (byte)249, (byte)154, (byte)138, (byte)7, (byte)255, (byte)172, (byte)111, (byte)107, (byte)252, (byte)242, (byte)151, (byte)198, (byte)112, (byte)210, (byte)94, (byte)209, (byte)23, (byte)29, (byte)190, (byte)142, (byte)132, (byte)229, (byte)96, (byte)80, (byte)96, (byte)104, (byte)27, (byte)202, (byte)170, (byte)152, (byte)149, (byte)52, (byte)185, (byte)134, (byte)208, (byte)197, (byte)106, (byte)112, (byte)92, (byte)190, (byte)55, (byte)78, (byte)230, (byte)4, (byte)186, (byte)238, (byte)150, (byte)105, (byte)200, (byte)62, (byte)142, (byte)88, (byte)43, (byte)16, (byte)178, (byte)110, (byte)246, (byte)1, (byte)66, (byte)158, (byte)107, (byte)0, (byte)211, (byte)7, (byte)19, (byte)27, (byte)204, (byte)149, (byte)121, (byte)76, (byte)191, (byte)57, (byte)207, (byte)207, (byte)219, (byte)64, (byte)165, (byte)167, (byte)219, (byte)26, (byte)91, (byte)104, (byte)127, (byte)215, (byte)178, (byte)151, (byte)181, (byte)203, (byte)77, (byte)7, (byte)237, (byte)117, (byte)55, (byte)118, (byte)201, (byte)105, (byte)195, (byte)201, (byte)207, (byte)109, (byte)96, (byte)98, (byte)249, (byte)245, (byte)50, (byte)214, (byte)92, (byte)170, (byte)13, (byte)137, (byte)193, (byte)95, (byte)251, (byte)225, (byte)114, (byte)110, (byte)250, (byte)172, (byte)161, (byte)206, (byte)242, (byte)125, (byte)113, (byte)240, (byte)174, (byte)214, (byte)89, (byte)127, (byte)198, (byte)172, (byte)179, (byte)2, (byte)103, (byte)70, (byte)74, (byte)68, (byte)118, (byte)164, (byte)113, (byte)17, (byte)89, (byte)14, (byte)4, (byte)24, (byte)248, (byte)82, (byte)166, (byte)140, (byte)247, (byte)228}));
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.sequence = (ushort)(ushort)7563;
            p267.data__SET(new byte[] {(byte)189, (byte)228, (byte)93, (byte)126, (byte)138, (byte)151, (byte)48, (byte)176, (byte)134, (byte)157, (byte)230, (byte)219, (byte)238, (byte)221, (byte)217, (byte)75, (byte)117, (byte)153, (byte)101, (byte)15, (byte)120, (byte)177, (byte)164, (byte)177, (byte)232, (byte)145, (byte)10, (byte)115, (byte)41, (byte)49, (byte)192, (byte)136, (byte)220, (byte)227, (byte)12, (byte)204, (byte)133, (byte)115, (byte)88, (byte)29, (byte)163, (byte)154, (byte)38, (byte)101, (byte)136, (byte)140, (byte)239, (byte)210, (byte)156, (byte)92, (byte)55, (byte)201, (byte)43, (byte)164, (byte)72, (byte)229, (byte)166, (byte)217, (byte)189, (byte)123, (byte)154, (byte)227, (byte)157, (byte)11, (byte)6, (byte)110, (byte)35, (byte)137, (byte)52, (byte)25, (byte)213, (byte)205, (byte)40, (byte)21, (byte)206, (byte)137, (byte)185, (byte)180, (byte)114, (byte)97, (byte)245, (byte)214, (byte)179, (byte)153, (byte)24, (byte)169, (byte)135, (byte)29, (byte)64, (byte)174, (byte)56, (byte)84, (byte)105, (byte)205, (byte)213, (byte)134, (byte)37, (byte)157, (byte)13, (byte)249, (byte)154, (byte)138, (byte)7, (byte)255, (byte)172, (byte)111, (byte)107, (byte)252, (byte)242, (byte)151, (byte)198, (byte)112, (byte)210, (byte)94, (byte)209, (byte)23, (byte)29, (byte)190, (byte)142, (byte)132, (byte)229, (byte)96, (byte)80, (byte)96, (byte)104, (byte)27, (byte)202, (byte)170, (byte)152, (byte)149, (byte)52, (byte)185, (byte)134, (byte)208, (byte)197, (byte)106, (byte)112, (byte)92, (byte)190, (byte)55, (byte)78, (byte)230, (byte)4, (byte)186, (byte)238, (byte)150, (byte)105, (byte)200, (byte)62, (byte)142, (byte)88, (byte)43, (byte)16, (byte)178, (byte)110, (byte)246, (byte)1, (byte)66, (byte)158, (byte)107, (byte)0, (byte)211, (byte)7, (byte)19, (byte)27, (byte)204, (byte)149, (byte)121, (byte)76, (byte)191, (byte)57, (byte)207, (byte)207, (byte)219, (byte)64, (byte)165, (byte)167, (byte)219, (byte)26, (byte)91, (byte)104, (byte)127, (byte)215, (byte)178, (byte)151, (byte)181, (byte)203, (byte)77, (byte)7, (byte)237, (byte)117, (byte)55, (byte)118, (byte)201, (byte)105, (byte)195, (byte)201, (byte)207, (byte)109, (byte)96, (byte)98, (byte)249, (byte)245, (byte)50, (byte)214, (byte)92, (byte)170, (byte)13, (byte)137, (byte)193, (byte)95, (byte)251, (byte)225, (byte)114, (byte)110, (byte)250, (byte)172, (byte)161, (byte)206, (byte)242, (byte)125, (byte)113, (byte)240, (byte)174, (byte)214, (byte)89, (byte)127, (byte)198, (byte)172, (byte)179, (byte)2, (byte)103, (byte)70, (byte)74, (byte)68, (byte)118, (byte)164, (byte)113, (byte)17, (byte)89, (byte)14, (byte)4, (byte)24, (byte)248, (byte)82, (byte)166, (byte)140, (byte)247, (byte)228}, 0) ;
            p267.target_component = (byte)(byte)106;
            p267.length = (byte)(byte)137;
            p267.first_message_offset = (byte)(byte)135;
            p267.target_system = (byte)(byte)125;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sequence == (ushort)(ushort)45165);
                Debug.Assert(pack.target_component == (byte)(byte)145);
                Debug.Assert(pack.target_system == (byte)(byte)111);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)111;
            p268.target_component = (byte)(byte)145;
            p268.sequence = (ushort)(ushort)45165;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.framerate == (float) -3.3546262E38F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)7880);
                Debug.Assert(pack.status == (byte)(byte)171);
                Debug.Assert(pack.bitrate == (uint)3927115823U);
                Debug.Assert(pack.camera_id == (byte)(byte)170);
                Debug.Assert(pack.rotation == (ushort)(ushort)51919);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)52849);
                Debug.Assert(pack.uri_LEN(ph) == 76);
                Debug.Assert(pack.uri_TRY(ph).Equals("NRuufczejnpOcsrdtofcqfiPkfdjwpgeygdqvikxjlqcjhbxdMZBiyvnsWsmcwztguwActbDUrBk"));
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.uri_SET("NRuufczejnpOcsrdtofcqfiPkfdjwpgeygdqvikxjlqcjhbxdMZBiyvnsWsmcwztguwActbDUrBk", PH) ;
            p269.camera_id = (byte)(byte)170;
            p269.resolution_h = (ushort)(ushort)7880;
            p269.status = (byte)(byte)171;
            p269.rotation = (ushort)(ushort)51919;
            p269.bitrate = (uint)3927115823U;
            p269.resolution_v = (ushort)(ushort)52849;
            p269.framerate = (float) -3.3546262E38F;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)117);
                Debug.Assert(pack.bitrate == (uint)2317145001U);
                Debug.Assert(pack.uri_LEN(ph) == 122);
                Debug.Assert(pack.uri_TRY(ph).Equals("lyndrxegQqfycgipJvdkcxnpiajpkegibksnfSvvlntpoAokdczrepqjCZnjuszlbnqtoviBywwzHqPymTqiijcwvcxWqhhuesZuawibgaubhilpkprhxhzcqn"));
                Debug.Assert(pack.camera_id == (byte)(byte)251);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)18684);
                Debug.Assert(pack.rotation == (ushort)(ushort)53968);
                Debug.Assert(pack.framerate == (float)3.0592226E38F);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)20510);
                Debug.Assert(pack.target_system == (byte)(byte)3);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)3;
            p270.uri_SET("lyndrxegQqfycgipJvdkcxnpiajpkegibksnfSvvlntpoAokdczrepqjCZnjuszlbnqtoviBywwzHqPymTqiijcwvcxWqhhuesZuawibgaubhilpkprhxhzcqn", PH) ;
            p270.resolution_h = (ushort)(ushort)18684;
            p270.bitrate = (uint)2317145001U;
            p270.target_component = (byte)(byte)117;
            p270.camera_id = (byte)(byte)251;
            p270.resolution_v = (ushort)(ushort)20510;
            p270.rotation = (ushort)(ushort)53968;
            p270.framerate = (float)3.0592226E38F;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 13);
                Debug.Assert(pack.password_TRY(ph).Equals("yompyqwrzldac"));
                Debug.Assert(pack.ssid_LEN(ph) == 28);
                Debug.Assert(pack.ssid_TRY(ph).Equals("jltplwwguxqmspcpZrszcznbclzb"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("jltplwwguxqmspcpZrszcznbclzb", PH) ;
            p299.password_SET("yompyqwrzldac", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)162, (byte)16, (byte)25, (byte)125, (byte)9, (byte)99, (byte)31, (byte)22}));
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)94, (byte)124, (byte)179, (byte)98, (byte)41, (byte)21, (byte)196, (byte)221}));
                Debug.Assert(pack.version == (ushort)(ushort)43828);
                Debug.Assert(pack.max_version == (ushort)(ushort)16148);
                Debug.Assert(pack.min_version == (ushort)(ushort)12916);
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)43828;
            p300.max_version = (ushort)(ushort)16148;
            p300.spec_version_hash_SET(new byte[] {(byte)94, (byte)124, (byte)179, (byte)98, (byte)41, (byte)21, (byte)196, (byte)221}, 0) ;
            p300.min_version = (ushort)(ushort)12916;
            p300.library_version_hash_SET(new byte[] {(byte)162, (byte)16, (byte)25, (byte)125, (byte)9, (byte)99, (byte)31, (byte)22}, 0) ;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uptime_sec == (uint)2483890354U);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)41580);
                Debug.Assert(pack.time_usec == (ulong)6727924443687742308L);
                Debug.Assert(pack.mode == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL);
                Debug.Assert(pack.health == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
                Debug.Assert(pack.sub_mode == (byte)(byte)146);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL;
            p310.uptime_sec = (uint)2483890354U;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK;
            p310.vendor_specific_status_code = (ushort)(ushort)41580;
            p310.time_usec = (ulong)6727924443687742308L;
            p310.sub_mode = (byte)(byte)146;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 13);
                Debug.Assert(pack.name_TRY(ph).Equals("uzswacxqbvntq"));
                Debug.Assert(pack.sw_version_minor == (byte)(byte)167);
                Debug.Assert(pack.sw_vcs_commit == (uint)830677239U);
                Debug.Assert(pack.sw_version_major == (byte)(byte)219);
                Debug.Assert(pack.time_usec == (ulong)851925168844881938L);
                Debug.Assert(pack.hw_version_major == (byte)(byte)102);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)188);
                Debug.Assert(pack.uptime_sec == (uint)2721217723U);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)63, (byte)26, (byte)189, (byte)116, (byte)129, (byte)196, (byte)47, (byte)128, (byte)230, (byte)43, (byte)34, (byte)5, (byte)115, (byte)47, (byte)25, (byte)4}));
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.uptime_sec = (uint)2721217723U;
            p311.name_SET("uzswacxqbvntq", PH) ;
            p311.hw_version_major = (byte)(byte)102;
            p311.sw_version_minor = (byte)(byte)167;
            p311.hw_version_minor = (byte)(byte)188;
            p311.sw_version_major = (byte)(byte)219;
            p311.sw_vcs_commit = (uint)830677239U;
            p311.hw_unique_id_SET(new byte[] {(byte)63, (byte)26, (byte)189, (byte)116, (byte)129, (byte)196, (byte)47, (byte)128, (byte)230, (byte)43, (byte)34, (byte)5, (byte)115, (byte)47, (byte)25, (byte)4}, 0) ;
            p311.time_usec = (ulong)851925168844881938L;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (short)(short) -21046);
                Debug.Assert(pack.target_system == (byte)(byte)241);
                Debug.Assert(pack.param_id_LEN(ph) == 9);
                Debug.Assert(pack.param_id_TRY(ph).Equals("qkwoSetca"));
                Debug.Assert(pack.target_component == (byte)(byte)36);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)241;
            p320.param_index = (short)(short) -21046;
            p320.param_id_SET("qkwoSetca", PH) ;
            p320.target_component = (byte)(byte)36;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)116);
                Debug.Assert(pack.target_system == (byte)(byte)15);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)15;
            p321.target_component = (byte)(byte)116;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("infnyoKy"));
                Debug.Assert(pack.param_count == (ushort)(ushort)22310);
                Debug.Assert(pack.param_value_LEN(ph) == 29);
                Debug.Assert(pack.param_value_TRY(ph).Equals("mktrmMmvZrscxattltirlotkzmgsf"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
                Debug.Assert(pack.param_index == (ushort)(ushort)22283);
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_index = (ushort)(ushort)22283;
            p322.param_id_SET("infnyoKy", PH) ;
            p322.param_value_SET("mktrmMmvZrscxattltirlotkzmgsf", PH) ;
            p322.param_count = (ushort)(ushort)22310;
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)130);
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
                Debug.Assert(pack.param_value_LEN(ph) == 19);
                Debug.Assert(pack.param_value_TRY(ph).Equals("weRfzldfuvQzudorsuj"));
                Debug.Assert(pack.target_system == (byte)(byte)227);
                Debug.Assert(pack.param_id_LEN(ph) == 13);
                Debug.Assert(pack.param_id_TRY(ph).Equals("shbsvmmlxvbYm"));
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_value_SET("weRfzldfuvQzudorsuj", PH) ;
            p323.target_component = (byte)(byte)130;
            p323.target_system = (byte)(byte)227;
            p323.param_id_SET("shbsvmmlxvbYm", PH) ;
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 126);
                Debug.Assert(pack.param_value_TRY(ph).Equals("tpftmyfNksbcsuvdmszbysxmiaDxezmacvdwwckzbhwehottxeqdEmgowctxlaonAeynvgqNiHsewajhUcMhgrorbCsovkHxnjcegawsPahglGumtnaRsbfweegGgo"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
                Debug.Assert(pack.param_result == PARAM_ACK.PARAM_ACK_FAILED);
                Debug.Assert(pack.param_id_LEN(ph) == 14);
                Debug.Assert(pack.param_id_TRY(ph).Equals("zucuoqnbmbfhTH"));
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_result = PARAM_ACK.PARAM_ACK_FAILED;
            p324.param_id_SET("zucuoqnbmbfhTH", PH) ;
            p324.param_value_SET("tpftmyfNksbcsuvdmszbysxmiaDxezmacvdwwckzbhwehottxeqdEmgowctxlaonAeynvgqNiHsewajhUcMhgrorbCsovkHxnjcegawsPahglGumtnaRsbfweegGgo", PH) ;
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)7872763023241089962L);
                Debug.Assert(pack.max_distance == (ushort)(ushort)13205);
                Debug.Assert(pack.increment == (byte)(byte)110);
                Debug.Assert(pack.min_distance == (ushort)(ushort)13868);
                Debug.Assert(pack.sensor_type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)62905, (ushort)35582, (ushort)23225, (ushort)5189, (ushort)28546, (ushort)19604, (ushort)57158, (ushort)34321, (ushort)38392, (ushort)3986, (ushort)35983, (ushort)24323, (ushort)14795, (ushort)11677, (ushort)39624, (ushort)51838, (ushort)35156, (ushort)36880, (ushort)31165, (ushort)35699, (ushort)56506, (ushort)17632, (ushort)36702, (ushort)7173, (ushort)20498, (ushort)53970, (ushort)8580, (ushort)15942, (ushort)114, (ushort)31124, (ushort)36959, (ushort)40043, (ushort)53164, (ushort)51285, (ushort)33765, (ushort)16270, (ushort)7650, (ushort)1335, (ushort)41389, (ushort)55827, (ushort)19727, (ushort)47294, (ushort)42672, (ushort)1460, (ushort)27588, (ushort)55912, (ushort)25091, (ushort)27562, (ushort)27596, (ushort)64938, (ushort)63518, (ushort)20747, (ushort)36113, (ushort)32270, (ushort)57261, (ushort)60518, (ushort)6300, (ushort)60770, (ushort)39327, (ushort)42381, (ushort)63550, (ushort)49327, (ushort)56111, (ushort)36020, (ushort)19111, (ushort)44787, (ushort)14709, (ushort)9427, (ushort)42991, (ushort)16353, (ushort)10024, (ushort)45098}));
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.increment = (byte)(byte)110;
            p330.distances_SET(new ushort[] {(ushort)62905, (ushort)35582, (ushort)23225, (ushort)5189, (ushort)28546, (ushort)19604, (ushort)57158, (ushort)34321, (ushort)38392, (ushort)3986, (ushort)35983, (ushort)24323, (ushort)14795, (ushort)11677, (ushort)39624, (ushort)51838, (ushort)35156, (ushort)36880, (ushort)31165, (ushort)35699, (ushort)56506, (ushort)17632, (ushort)36702, (ushort)7173, (ushort)20498, (ushort)53970, (ushort)8580, (ushort)15942, (ushort)114, (ushort)31124, (ushort)36959, (ushort)40043, (ushort)53164, (ushort)51285, (ushort)33765, (ushort)16270, (ushort)7650, (ushort)1335, (ushort)41389, (ushort)55827, (ushort)19727, (ushort)47294, (ushort)42672, (ushort)1460, (ushort)27588, (ushort)55912, (ushort)25091, (ushort)27562, (ushort)27596, (ushort)64938, (ushort)63518, (ushort)20747, (ushort)36113, (ushort)32270, (ushort)57261, (ushort)60518, (ushort)6300, (ushort)60770, (ushort)39327, (ushort)42381, (ushort)63550, (ushort)49327, (ushort)56111, (ushort)36020, (ushort)19111, (ushort)44787, (ushort)14709, (ushort)9427, (ushort)42991, (ushort)16353, (ushort)10024, (ushort)45098}, 0) ;
            p330.time_usec = (ulong)7872763023241089962L;
            p330.min_distance = (ushort)(ushort)13868;
            p330.max_distance = (ushort)(ushort)13205;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
        }
    }
}