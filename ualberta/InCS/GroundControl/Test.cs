
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
                    ulong id = id__l(value);
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
                    ulong id = id__l(value);
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
                    ulong id = id__k(value);
                    BitUtils.set_bits(id, 7, data, 276);
                }
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = id__l(value);
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
                    ulong id = id__l(value);
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
                    ulong id = id__l(value);
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
                    ulong id = id__l(value);
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
                    ulong id = id__l(value);
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
                    ulong id = id__l(value);
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
                    ulong id = id__l(value);
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
                    ulong id = id__k(value);
                    BitUtils.set_bits(id, 7, data, 276);
                }
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = id__l(value);
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
                    ulong id = id__k(value);
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
                Debug.Assert(pack.system_status == MAV_STATE.MAV_STATE_UNINIT);
                Debug.Assert(pack.autopilot == MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY);
                Debug.Assert(pack.custom_mode == (uint)3099237087U);
                Debug.Assert(pack.type == MAV_TYPE.MAV_TYPE_GENERIC);
                Debug.Assert(pack.mavlink_version == (byte)(byte)110);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED));
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.type = MAV_TYPE.MAV_TYPE_GENERIC;
            p0.mavlink_version = (byte)(byte)110;
            p0.custom_mode = (uint)3099237087U;
            p0.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY;
            p0.system_status = MAV_STATE.MAV_STATE_UNINIT;
            p0.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
            SMP_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_battery == (short)(short) -24047);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
                Debug.Assert(pack.errors_comm == (ushort)(ushort)34509);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 47);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)56550);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)61279);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)26361);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)37339);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)61083);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)29144);
                Debug.Assert(pack.load == (ushort)(ushort)20266);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING));
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.drop_rate_comm = (ushort)(ushort)29144;
            p1.errors_count2 = (ushort)(ushort)56550;
            p1.errors_count1 = (ushort)(ushort)61279;
            p1.errors_comm = (ushort)(ushort)34509;
            p1.battery_remaining = (sbyte)(sbyte) - 47;
            p1.voltage_battery = (ushort)(ushort)37339;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.errors_count4 = (ushort)(ushort)61083;
            p1.load = (ushort)(ushort)20266;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.current_battery = (short)(short) -24047;
            p1.errors_count3 = (ushort)(ushort)26361;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING);
            SMP_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)745091741223207945L);
                Debug.Assert(pack.time_boot_ms == (uint)82234964U);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)745091741223207945L;
            p2.time_boot_ms = (uint)82234964U;
            SMP_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (ushort)(ushort)21703);
                Debug.Assert(pack.yaw_rate == (float)1.5978933E38F);
                Debug.Assert(pack.time_boot_ms == (uint)439049401U);
                Debug.Assert(pack.z == (float) -1.6118693E38F);
                Debug.Assert(pack.afy == (float)3.286154E38F);
                Debug.Assert(pack.afz == (float) -9.186087E37F);
                Debug.Assert(pack.vx == (float)4.7643816E37F);
                Debug.Assert(pack.y == (float) -9.4289E37F);
                Debug.Assert(pack.yaw == (float)2.9303707E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.afx == (float) -1.0443383E38F);
                Debug.Assert(pack.vy == (float)1.1966787E38F);
                Debug.Assert(pack.x == (float) -2.418836E38F);
                Debug.Assert(pack.vz == (float)2.5298262E37F);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.yaw = (float)2.9303707E38F;
            p3.x = (float) -2.418836E38F;
            p3.afx = (float) -1.0443383E38F;
            p3.type_mask = (ushort)(ushort)21703;
            p3.time_boot_ms = (uint)439049401U;
            p3.vy = (float)1.1966787E38F;
            p3.vx = (float)4.7643816E37F;
            p3.y = (float) -9.4289E37F;
            p3.vz = (float)2.5298262E37F;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p3.yaw_rate = (float)1.5978933E38F;
            p3.afy = (float)3.286154E38F;
            p3.z = (float) -1.6118693E38F;
            p3.afz = (float) -9.186087E37F;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)1099678165U);
                Debug.Assert(pack.time_usec == (ulong)3003261658404177374L);
                Debug.Assert(pack.target_system == (byte)(byte)41);
                Debug.Assert(pack.target_component == (byte)(byte)225);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.time_usec = (ulong)3003261658404177374L;
            p4.target_system = (byte)(byte)41;
            p4.target_component = (byte)(byte)225;
            p4.seq = (uint)1099678165U;
            SMP_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.control_request == (byte)(byte)226);
                Debug.Assert(pack.version == (byte)(byte)83);
                Debug.Assert(pack.passkey_LEN(ph) == 13);
                Debug.Assert(pack.passkey_TRY(ph).Equals("lzjsddruudtpy"));
                Debug.Assert(pack.target_system == (byte)(byte)32);
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.passkey_SET("lzjsddruudtpy", PH) ;
            p5.control_request = (byte)(byte)226;
            p5.target_system = (byte)(byte)32;
            p5.version = (byte)(byte)83;
            SMP_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ack == (byte)(byte)2);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)138);
                Debug.Assert(pack.control_request == (byte)(byte)210);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.control_request = (byte)(byte)210;
            p6.gcs_system_id = (byte)(byte)138;
            p6.ack = (byte)(byte)2;
            SMP_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 21);
                Debug.Assert(pack.key_TRY(ph).Equals("uZacthizjgZycvayzqEGk"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("uZacthizjgZycvayzqEGk", PH) ;
            SMP_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)240);
                Debug.Assert(pack.base_mode == MAV_MODE.MAV_MODE_GUIDED_ARMED);
                Debug.Assert(pack.custom_mode == (uint)3979006271U);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)240;
            p11.custom_mode = (uint)3979006271U;
            p11.base_mode = MAV_MODE.MAV_MODE_GUIDED_ARMED;
            SMP_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)52);
                Debug.Assert(pack.target_system == (byte)(byte)23);
                Debug.Assert(pack.param_index == (short)(short) -24812);
                Debug.Assert(pack.param_id_LEN(ph) == 10);
                Debug.Assert(pack.param_id_TRY(ph).Equals("oopkusuhof"));
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.param_id_SET("oopkusuhof", PH) ;
            p20.target_component = (byte)(byte)52;
            p20.param_index = (short)(short) -24812;
            p20.target_system = (byte)(byte)23;
            SMP_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)6);
                Debug.Assert(pack.target_component == (byte)(byte)20);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)20;
            p21.target_system = (byte)(byte)6;
            SMP_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64);
                Debug.Assert(pack.param_index == (ushort)(ushort)16281);
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("TeqxHoLu"));
                Debug.Assert(pack.param_count == (ushort)(ushort)42344);
                Debug.Assert(pack.param_value == (float)1.858932E38F);
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_value = (float)1.858932E38F;
            p22.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64;
            p22.param_count = (ushort)(ushort)42344;
            p22.param_id_SET("TeqxHoLu", PH) ;
            p22.param_index = (ushort)(ushort)16281;
            SMP_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value == (float) -2.662045E38F);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("lkjsu"));
                Debug.Assert(pack.target_component == (byte)(byte)10);
                Debug.Assert(pack.target_system == (byte)(byte)83);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.target_system = (byte)(byte)83;
            p23.param_id_SET("lkjsu", PH) ;
            p23.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64;
            p23.target_component = (byte)(byte)10;
            p23.param_value = (float) -2.662045E38F;
            SMP_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)1820403494);
                Debug.Assert(pack.time_usec == (ulong)8863479591129586615L);
                Debug.Assert(pack.satellites_visible == (byte)(byte)246);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)1978645044U);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)2125452642U);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -1841976163);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
                Debug.Assert(pack.epv == (ushort)(ushort)8218);
                Debug.Assert(pack.lat == (int)1832861025);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)3055302900U);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)1818267880U);
                Debug.Assert(pack.eph == (ushort)(ushort)31448);
                Debug.Assert(pack.alt == (int)1019268893);
                Debug.Assert(pack.vel == (ushort)(ushort)36239);
                Debug.Assert(pack.cog == (ushort)(ushort)13487);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.alt_ellipsoid_SET((int) -1841976163, PH) ;
            p24.h_acc_SET((uint)1818267880U, PH) ;
            p24.time_usec = (ulong)8863479591129586615L;
            p24.v_acc_SET((uint)1978645044U, PH) ;
            p24.vel_acc_SET((uint)2125452642U, PH) ;
            p24.alt = (int)1019268893;
            p24.lat = (int)1832861025;
            p24.hdg_acc_SET((uint)3055302900U, PH) ;
            p24.vel = (ushort)(ushort)36239;
            p24.cog = (ushort)(ushort)13487;
            p24.eph = (ushort)(ushort)31448;
            p24.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS;
            p24.satellites_visible = (byte)(byte)246;
            p24.epv = (ushort)(ushort)8218;
            p24.lon = (int)1820403494;
            SMP_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)195, (byte)27, (byte)159, (byte)59, (byte)2, (byte)136, (byte)255, (byte)7, (byte)80, (byte)139, (byte)212, (byte)152, (byte)136, (byte)212, (byte)22, (byte)110, (byte)5, (byte)69, (byte)121, (byte)162}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)169, (byte)2, (byte)16, (byte)193, (byte)29, (byte)22, (byte)186, (byte)102, (byte)226, (byte)101, (byte)232, (byte)102, (byte)96, (byte)236, (byte)34, (byte)177, (byte)171, (byte)231, (byte)246, (byte)211}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)247);
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)166, (byte)177, (byte)66, (byte)237, (byte)5, (byte)197, (byte)199, (byte)106, (byte)146, (byte)30, (byte)50, (byte)253, (byte)194, (byte)228, (byte)213, (byte)183, (byte)50, (byte)193, (byte)249, (byte)17}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)196, (byte)228, (byte)117, (byte)27, (byte)92, (byte)16, (byte)182, (byte)140, (byte)223, (byte)129, (byte)165, (byte)210, (byte)21, (byte)190, (byte)25, (byte)141, (byte)181, (byte)21, (byte)12, (byte)51}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)43, (byte)111, (byte)51, (byte)182, (byte)235, (byte)157, (byte)100, (byte)217, (byte)201, (byte)74, (byte)134, (byte)231, (byte)170, (byte)119, (byte)136, (byte)221, (byte)233, (byte)112, (byte)116, (byte)152}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_elevation_SET(new byte[] {(byte)166, (byte)177, (byte)66, (byte)237, (byte)5, (byte)197, (byte)199, (byte)106, (byte)146, (byte)30, (byte)50, (byte)253, (byte)194, (byte)228, (byte)213, (byte)183, (byte)50, (byte)193, (byte)249, (byte)17}, 0) ;
            p25.satellites_visible = (byte)(byte)247;
            p25.satellite_prn_SET(new byte[] {(byte)196, (byte)228, (byte)117, (byte)27, (byte)92, (byte)16, (byte)182, (byte)140, (byte)223, (byte)129, (byte)165, (byte)210, (byte)21, (byte)190, (byte)25, (byte)141, (byte)181, (byte)21, (byte)12, (byte)51}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)195, (byte)27, (byte)159, (byte)59, (byte)2, (byte)136, (byte)255, (byte)7, (byte)80, (byte)139, (byte)212, (byte)152, (byte)136, (byte)212, (byte)22, (byte)110, (byte)5, (byte)69, (byte)121, (byte)162}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)43, (byte)111, (byte)51, (byte)182, (byte)235, (byte)157, (byte)100, (byte)217, (byte)201, (byte)74, (byte)134, (byte)231, (byte)170, (byte)119, (byte)136, (byte)221, (byte)233, (byte)112, (byte)116, (byte)152}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)169, (byte)2, (byte)16, (byte)193, (byte)29, (byte)22, (byte)186, (byte)102, (byte)226, (byte)101, (byte)232, (byte)102, (byte)96, (byte)236, (byte)34, (byte)177, (byte)171, (byte)231, (byte)246, (byte)211}, 0) ;
            SMP_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (short)(short) -1614);
                Debug.Assert(pack.zacc == (short)(short)4175);
                Debug.Assert(pack.xacc == (short)(short)18253);
                Debug.Assert(pack.yacc == (short)(short) -31816);
                Debug.Assert(pack.ygyro == (short)(short)992);
                Debug.Assert(pack.ymag == (short)(short)11064);
                Debug.Assert(pack.zgyro == (short)(short) -8372);
                Debug.Assert(pack.xmag == (short)(short) -14332);
                Debug.Assert(pack.zmag == (short)(short)17059);
                Debug.Assert(pack.time_boot_ms == (uint)3901736226U);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.zgyro = (short)(short) -8372;
            p26.ygyro = (short)(short)992;
            p26.zmag = (short)(short)17059;
            p26.xgyro = (short)(short) -1614;
            p26.ymag = (short)(short)11064;
            p26.yacc = (short)(short) -31816;
            p26.time_boot_ms = (uint)3901736226U;
            p26.xacc = (short)(short)18253;
            p26.zacc = (short)(short)4175;
            p26.xmag = (short)(short) -14332;
            SMP_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short) -23484);
                Debug.Assert(pack.xacc == (short)(short) -11);
                Debug.Assert(pack.zmag == (short)(short)31671);
                Debug.Assert(pack.xmag == (short)(short)5140);
                Debug.Assert(pack.ymag == (short)(short) -8226);
                Debug.Assert(pack.xgyro == (short)(short)21748);
                Debug.Assert(pack.ygyro == (short)(short) -25624);
                Debug.Assert(pack.zgyro == (short)(short) -30343);
                Debug.Assert(pack.time_usec == (ulong)9113662181412242372L);
                Debug.Assert(pack.zacc == (short)(short) -11922);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.zgyro = (short)(short) -30343;
            p27.xmag = (short)(short)5140;
            p27.time_usec = (ulong)9113662181412242372L;
            p27.zmag = (short)(short)31671;
            p27.ygyro = (short)(short) -25624;
            p27.zacc = (short)(short) -11922;
            p27.ymag = (short)(short) -8226;
            p27.yacc = (short)(short) -23484;
            p27.xgyro = (short)(short)21748;
            p27.xacc = (short)(short) -11;
            SMP_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)6056094420270497277L);
                Debug.Assert(pack.press_abs == (short)(short) -4166);
                Debug.Assert(pack.press_diff1 == (short)(short)29404);
                Debug.Assert(pack.temperature == (short)(short)32064);
                Debug.Assert(pack.press_diff2 == (short)(short) -25815);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_diff1 = (short)(short)29404;
            p28.press_abs = (short)(short) -4166;
            p28.temperature = (short)(short)32064;
            p28.press_diff2 = (short)(short) -25815;
            p28.time_usec = (ulong)6056094420270497277L;
            SMP_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float) -1.121363E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2623027563U);
                Debug.Assert(pack.press_abs == (float) -2.1161368E38F);
                Debug.Assert(pack.temperature == (short)(short) -18840);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.temperature = (short)(short) -18840;
            p29.press_abs = (float) -2.1161368E38F;
            p29.press_diff = (float) -1.121363E38F;
            p29.time_boot_ms = (uint)2623027563U;
            SMP_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float)9.216956E37F);
                Debug.Assert(pack.yawspeed == (float)2.817988E38F);
                Debug.Assert(pack.pitchspeed == (float)3.6372818E37F);
                Debug.Assert(pack.time_boot_ms == (uint)383219182U);
                Debug.Assert(pack.roll == (float) -2.8804123E38F);
                Debug.Assert(pack.yaw == (float) -2.364847E38F);
                Debug.Assert(pack.rollspeed == (float)1.937119E38F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.pitchspeed = (float)3.6372818E37F;
            p30.pitch = (float)9.216956E37F;
            p30.yaw = (float) -2.364847E38F;
            p30.time_boot_ms = (uint)383219182U;
            p30.rollspeed = (float)1.937119E38F;
            p30.yawspeed = (float)2.817988E38F;
            p30.roll = (float) -2.8804123E38F;
            SMP_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q2 == (float)1.372317E38F);
                Debug.Assert(pack.yawspeed == (float)8.0319787E37F);
                Debug.Assert(pack.rollspeed == (float) -3.116638E38F);
                Debug.Assert(pack.q4 == (float)2.3200022E38F);
                Debug.Assert(pack.q3 == (float) -6.852501E37F);
                Debug.Assert(pack.pitchspeed == (float)1.7127063E38F);
                Debug.Assert(pack.q1 == (float)2.2853672E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2689000910U);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.q3 = (float) -6.852501E37F;
            p31.pitchspeed = (float)1.7127063E38F;
            p31.rollspeed = (float) -3.116638E38F;
            p31.yawspeed = (float)8.0319787E37F;
            p31.q4 = (float)2.3200022E38F;
            p31.time_boot_ms = (uint)2689000910U;
            p31.q1 = (float)2.2853672E38F;
            p31.q2 = (float)1.372317E38F;
            SMP_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)8.926531E37F);
                Debug.Assert(pack.y == (float) -1.041946E38F);
                Debug.Assert(pack.vx == (float)3.2862454E38F);
                Debug.Assert(pack.z == (float)3.113202E38F);
                Debug.Assert(pack.vy == (float)2.5335072E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2576190013U);
                Debug.Assert(pack.vz == (float)8.2634096E37F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.z = (float)3.113202E38F;
            p32.y = (float) -1.041946E38F;
            p32.vz = (float)8.2634096E37F;
            p32.vy = (float)2.5335072E38F;
            p32.x = (float)8.926531E37F;
            p32.time_boot_ms = (uint)2576190013U;
            p32.vx = (float)3.2862454E38F;
            SMP_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hdg == (ushort)(ushort)50424);
                Debug.Assert(pack.vz == (short)(short)8937);
                Debug.Assert(pack.vx == (short)(short)4491);
                Debug.Assert(pack.lon == (int)29125077);
                Debug.Assert(pack.vy == (short)(short)25995);
                Debug.Assert(pack.relative_alt == (int) -230332889);
                Debug.Assert(pack.lat == (int) -927600562);
                Debug.Assert(pack.alt == (int) -417322704);
                Debug.Assert(pack.time_boot_ms == (uint)3574527429U);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.lat = (int) -927600562;
            p33.alt = (int) -417322704;
            p33.vz = (short)(short)8937;
            p33.relative_alt = (int) -230332889;
            p33.hdg = (ushort)(ushort)50424;
            p33.vy = (short)(short)25995;
            p33.vx = (short)(short)4491;
            p33.lon = (int)29125077;
            p33.time_boot_ms = (uint)3574527429U;
            SMP_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan1_scaled == (short)(short)22490);
                Debug.Assert(pack.chan3_scaled == (short)(short)17386);
                Debug.Assert(pack.rssi == (byte)(byte)162);
                Debug.Assert(pack.chan4_scaled == (short)(short) -24759);
                Debug.Assert(pack.chan2_scaled == (short)(short) -10785);
                Debug.Assert(pack.chan6_scaled == (short)(short)28742);
                Debug.Assert(pack.time_boot_ms == (uint)2747226027U);
                Debug.Assert(pack.port == (byte)(byte)136);
                Debug.Assert(pack.chan5_scaled == (short)(short)30485);
                Debug.Assert(pack.chan8_scaled == (short)(short)17960);
                Debug.Assert(pack.chan7_scaled == (short)(short) -10794);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan3_scaled = (short)(short)17386;
            p34.chan2_scaled = (short)(short) -10785;
            p34.rssi = (byte)(byte)162;
            p34.chan1_scaled = (short)(short)22490;
            p34.chan7_scaled = (short)(short) -10794;
            p34.chan8_scaled = (short)(short)17960;
            p34.chan6_scaled = (short)(short)28742;
            p34.time_boot_ms = (uint)2747226027U;
            p34.chan4_scaled = (short)(short) -24759;
            p34.port = (byte)(byte)136;
            p34.chan5_scaled = (short)(short)30485;
            SMP_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)51614);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)4184);
                Debug.Assert(pack.rssi == (byte)(byte)189);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)38878);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)58092);
                Debug.Assert(pack.time_boot_ms == (uint)383852287U);
                Debug.Assert(pack.port == (byte)(byte)172);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)40023);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)3213);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)3178);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)55067);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan3_raw = (ushort)(ushort)40023;
            p35.time_boot_ms = (uint)383852287U;
            p35.rssi = (byte)(byte)189;
            p35.chan5_raw = (ushort)(ushort)51614;
            p35.chan4_raw = (ushort)(ushort)38878;
            p35.chan6_raw = (ushort)(ushort)3213;
            p35.chan2_raw = (ushort)(ushort)58092;
            p35.chan8_raw = (ushort)(ushort)4184;
            p35.port = (byte)(byte)172;
            p35.chan1_raw = (ushort)(ushort)3178;
            p35.chan7_raw = (ushort)(ushort)55067;
            SMP_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (uint)1398963672U);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)16846);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)15282);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)63298);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)59161);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)55316);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)45689);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)38589);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)26370);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)9797);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)41968);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)2192);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)1572);
                Debug.Assert(pack.port == (byte)(byte)196);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)4720);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)64606);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)44950);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)63621);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo7_raw = (ushort)(ushort)38589;
            p36.servo1_raw = (ushort)(ushort)41968;
            p36.servo15_raw_SET((ushort)(ushort)9797, PH) ;
            p36.servo14_raw_SET((ushort)(ushort)44950, PH) ;
            p36.servo2_raw = (ushort)(ushort)63621;
            p36.time_usec = (uint)1398963672U;
            p36.servo16_raw_SET((ushort)(ushort)16846, PH) ;
            p36.servo8_raw = (ushort)(ushort)4720;
            p36.servo3_raw = (ushort)(ushort)59161;
            p36.servo11_raw_SET((ushort)(ushort)55316, PH) ;
            p36.servo9_raw_SET((ushort)(ushort)1572, PH) ;
            p36.servo6_raw = (ushort)(ushort)64606;
            p36.port = (byte)(byte)196;
            p36.servo5_raw = (ushort)(ushort)63298;
            p36.servo13_raw_SET((ushort)(ushort)45689, PH) ;
            p36.servo10_raw_SET((ushort)(ushort)26370, PH) ;
            p36.servo4_raw = (ushort)(ushort)15282;
            p36.servo12_raw_SET((ushort)(ushort)2192, PH) ;
            SMP_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)98);
                Debug.Assert(pack.start_index == (short)(short) -26238);
                Debug.Assert(pack.target_component == (byte)(byte)12);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.end_index == (short)(short) -23657);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.target_component = (byte)(byte)12;
            p37.start_index = (short)(short) -26238;
            p37.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p37.target_system = (byte)(byte)98;
            p37.end_index = (short)(short) -23657;
            SMP_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_index == (short)(short)29393);
                Debug.Assert(pack.target_system == (byte)(byte)165);
                Debug.Assert(pack.target_component == (byte)(byte)234);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.end_index == (short)(short) -29799);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.target_component = (byte)(byte)234;
            p38.target_system = (byte)(byte)165;
            p38.start_index = (short)(short)29393;
            p38.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p38.end_index = (short)(short) -29799;
            SMP_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param2 == (float) -1.6233263E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE);
                Debug.Assert(pack.seq == (ushort)(ushort)23357);
                Debug.Assert(pack.current == (byte)(byte)78);
                Debug.Assert(pack.x == (float)3.2030971E38F);
                Debug.Assert(pack.param3 == (float) -1.6481438E38F);
                Debug.Assert(pack.y == (float)1.0705774E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)213);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)131);
                Debug.Assert(pack.param4 == (float)8.876371E37F);
                Debug.Assert(pack.param1 == (float) -2.3050342E38F);
                Debug.Assert(pack.target_component == (byte)(byte)181);
                Debug.Assert(pack.z == (float) -6.9919977E37F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.z = (float) -6.9919977E37F;
            p39.current = (byte)(byte)78;
            p39.param2 = (float) -1.6233263E38F;
            p39.x = (float)3.2030971E38F;
            p39.target_component = (byte)(byte)181;
            p39.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p39.param3 = (float) -1.6481438E38F;
            p39.autocontinue = (byte)(byte)213;
            p39.command = MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
            p39.param1 = (float) -2.3050342E38F;
            p39.param4 = (float)8.876371E37F;
            p39.seq = (ushort)(ushort)23357;
            p39.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p39.y = (float)1.0705774E38F;
            p39.target_system = (byte)(byte)131;
            SMP_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)43659);
                Debug.Assert(pack.target_component == (byte)(byte)128);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_system == (byte)(byte)225);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p40.target_system = (byte)(byte)225;
            p40.target_component = (byte)(byte)128;
            p40.seq = (ushort)(ushort)43659;
            SMP_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)57753);
                Debug.Assert(pack.target_component == (byte)(byte)234);
                Debug.Assert(pack.target_system == (byte)(byte)198);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_component = (byte)(byte)234;
            p41.target_system = (byte)(byte)198;
            p41.seq = (ushort)(ushort)57753;
            SMP_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)55647);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)55647;
            SMP_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)212);
                Debug.Assert(pack.target_system == (byte)(byte)149);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p43.target_component = (byte)(byte)212;
            p43.target_system = (byte)(byte)149;
            SMP_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_system == (byte)(byte)247);
                Debug.Assert(pack.target_component == (byte)(byte)117);
                Debug.Assert(pack.count == (ushort)(ushort)62451);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.target_component = (byte)(byte)117;
            p44.target_system = (byte)(byte)247;
            p44.count = (ushort)(ushort)62451;
            p44.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            SMP_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)156);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)55);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_system = (byte)(byte)156;
            p45.target_component = (byte)(byte)55;
            p45.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            SMP_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)63255);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)63255;
            SMP_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)118);
                Debug.Assert(pack.target_component == (byte)(byte)179);
                Debug.Assert(pack.type == MAV_MISSION_RESULT.MAV_MISSION_INVALID_SEQUENCE);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.type = MAV_MISSION_RESULT.MAV_MISSION_INVALID_SEQUENCE;
            p47.target_system = (byte)(byte)118;
            p47.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p47.target_component = (byte)(byte)179;
            SMP_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)1816778198142174456L);
                Debug.Assert(pack.altitude == (int) -1220834949);
                Debug.Assert(pack.target_system == (byte)(byte)179);
                Debug.Assert(pack.latitude == (int) -516643517);
                Debug.Assert(pack.longitude == (int) -391532819);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.altitude = (int) -1220834949;
            p48.longitude = (int) -391532819;
            p48.latitude = (int) -516643517;
            p48.time_usec_SET((ulong)1816778198142174456L, PH) ;
            p48.target_system = (byte)(byte)179;
            SMP_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (int)190786557);
                Debug.Assert(pack.altitude == (int)1816916467);
                Debug.Assert(pack.latitude == (int) -113080615);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)1039883518374551422L);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.altitude = (int)1816916467;
            p49.time_usec_SET((ulong)1039883518374551422L, PH) ;
            p49.longitude = (int)190786557;
            p49.latitude = (int) -113080615;
            SMP_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)118);
                Debug.Assert(pack.param_id_LEN(ph) == 11);
                Debug.Assert(pack.param_id_TRY(ph).Equals("qgymdpuvVcA"));
                Debug.Assert(pack.param_index == (short)(short) -30810);
                Debug.Assert(pack.param_value0 == (float) -1.2471515E38F);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)211);
                Debug.Assert(pack.param_value_min == (float)1.149861E38F);
                Debug.Assert(pack.param_value_max == (float)3.2115336E38F);
                Debug.Assert(pack.scale == (float) -2.9530908E38F);
                Debug.Assert(pack.target_component == (byte)(byte)188);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.param_id_SET("qgymdpuvVcA", PH) ;
            p50.parameter_rc_channel_index = (byte)(byte)211;
            p50.target_component = (byte)(byte)188;
            p50.param_value_max = (float)3.2115336E38F;
            p50.param_value0 = (float) -1.2471515E38F;
            p50.param_value_min = (float)1.149861E38F;
            p50.target_system = (byte)(byte)118;
            p50.param_index = (short)(short) -30810;
            p50.scale = (float) -2.9530908E38F;
            SMP_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)72);
                Debug.Assert(pack.target_system == (byte)(byte)191);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.seq == (ushort)(ushort)7385);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.seq = (ushort)(ushort)7385;
            p51.target_system = (byte)(byte)191;
            p51.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p51.target_component = (byte)(byte)72;
            SMP_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2z == (float)1.8672656E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.p1z == (float) -8.863006E37F);
                Debug.Assert(pack.target_system == (byte)(byte)79);
                Debug.Assert(pack.p2y == (float) -2.541134E38F);
                Debug.Assert(pack.target_component == (byte)(byte)73);
                Debug.Assert(pack.p1y == (float) -2.7152285E38F);
                Debug.Assert(pack.p1x == (float)1.8704106E38F);
                Debug.Assert(pack.p2x == (float) -1.1058615E38F);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.target_component = (byte)(byte)73;
            p54.p2z = (float)1.8672656E38F;
            p54.p2y = (float) -2.541134E38F;
            p54.p2x = (float) -1.1058615E38F;
            p54.p1x = (float)1.8704106E38F;
            p54.p1y = (float) -2.7152285E38F;
            p54.target_system = (byte)(byte)79;
            p54.p1z = (float) -8.863006E37F;
            p54.frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            SMP_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1z == (float)9.415132E35F);
                Debug.Assert(pack.p1x == (float)2.791358E38F);
                Debug.Assert(pack.p1y == (float)1.9502064E37F);
                Debug.Assert(pack.p2x == (float)9.36965E37F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.p2y == (float) -2.177531E38F);
                Debug.Assert(pack.p2z == (float) -2.5591985E38F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p2y = (float) -2.177531E38F;
            p55.p1y = (float)1.9502064E37F;
            p55.p2z = (float) -2.5591985E38F;
            p55.p2x = (float)9.36965E37F;
            p55.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p55.p1x = (float)2.791358E38F;
            p55.p1z = (float)9.415132E35F;
            SMP_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-4.8456943E37F, -2.266776E38F, 1.7640595E38F, -1.0995179E37F, -1.9012534E38F, 2.2452966E38F, -4.5998776E37F, 1.9968466E38F, 4.1122218E37F}));
                Debug.Assert(pack.rollspeed == (float)1.2741172E38F);
                Debug.Assert(pack.time_usec == (ulong)7360190645781335538L);
                Debug.Assert(pack.pitchspeed == (float)3.690548E37F);
                Debug.Assert(pack.yawspeed == (float)1.4465112E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.7000737E38F, -2.3122963E38F, 2.9804768E38F, -1.1924353E38F}));
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.rollspeed = (float)1.2741172E38F;
            p61.pitchspeed = (float)3.690548E37F;
            p61.time_usec = (ulong)7360190645781335538L;
            p61.q_SET(new float[] {2.7000737E38F, -2.3122963E38F, 2.9804768E38F, -1.1924353E38F}, 0) ;
            p61.covariance_SET(new float[] {-4.8456943E37F, -2.266776E38F, 1.7640595E38F, -1.0995179E37F, -1.9012534E38F, 2.2452966E38F, -4.5998776E37F, 1.9968466E38F, 4.1122218E37F}, 0) ;
            p61.yawspeed = (float)1.4465112E38F;
            SMP_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wp_dist == (ushort)(ushort)64273);
                Debug.Assert(pack.target_bearing == (short)(short)11132);
                Debug.Assert(pack.aspd_error == (float) -1.832659E38F);
                Debug.Assert(pack.xtrack_error == (float) -2.0214723E38F);
                Debug.Assert(pack.alt_error == (float)3.3242275E38F);
                Debug.Assert(pack.nav_pitch == (float) -1.8550048E38F);
                Debug.Assert(pack.nav_roll == (float)2.0089159E38F);
                Debug.Assert(pack.nav_bearing == (short)(short)21091);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.aspd_error = (float) -1.832659E38F;
            p62.nav_bearing = (short)(short)21091;
            p62.xtrack_error = (float) -2.0214723E38F;
            p62.target_bearing = (short)(short)11132;
            p62.wp_dist = (ushort)(ushort)64273;
            p62.nav_pitch = (float) -1.8550048E38F;
            p62.nav_roll = (float)2.0089159E38F;
            p62.alt_error = (float)3.3242275E38F;
            SMP_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
                Debug.Assert(pack.relative_alt == (int) -681977245);
                Debug.Assert(pack.lon == (int) -938891953);
                Debug.Assert(pack.alt == (int)1812625562);
                Debug.Assert(pack.lat == (int) -1337154627);
                Debug.Assert(pack.vx == (float) -1.6420781E37F);
                Debug.Assert(pack.vy == (float) -2.6258104E38F);
                Debug.Assert(pack.vz == (float) -6.839168E37F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-6.9681055E37F, 8.855898E37F, 1.6919084E38F, -1.5944749E38F, 1.5107338E38F, -2.0214522E38F, -3.1978644E38F, 1.0981411E38F, -9.431672E35F, 3.39883E38F, 2.12529E37F, -1.4945646E38F, -1.6063795E37F, 3.0755727E38F, -3.0976111E38F, -2.758549E38F, -4.8153995E37F, 1.0554896E38F, 3.2536297E38F, -1.9504837E38F, -1.7631667E38F, -4.6468775E37F, -2.4558542E38F, 1.1278903E37F, -1.9558526E38F, 2.3602275E38F, -2.496958E37F, 2.9570928E38F, -3.2622684E38F, 2.4489785E38F, -5.0278117E35F, 1.2597069E38F, 2.4410653E38F, -2.910326E38F, -3.176143E37F, -3.3651716E38F}));
                Debug.Assert(pack.time_usec == (ulong)6215835760350864360L);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.relative_alt = (int) -681977245;
            p63.time_usec = (ulong)6215835760350864360L;
            p63.lon = (int) -938891953;
            p63.lat = (int) -1337154627;
            p63.vx = (float) -1.6420781E37F;
            p63.alt = (int)1812625562;
            p63.vz = (float) -6.839168E37F;
            p63.vy = (float) -2.6258104E38F;
            p63.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS;
            p63.covariance_SET(new float[] {-6.9681055E37F, 8.855898E37F, 1.6919084E38F, -1.5944749E38F, 1.5107338E38F, -2.0214522E38F, -3.1978644E38F, 1.0981411E38F, -9.431672E35F, 3.39883E38F, 2.12529E37F, -1.4945646E38F, -1.6063795E37F, 3.0755727E38F, -3.0976111E38F, -2.758549E38F, -4.8153995E37F, 1.0554896E38F, 3.2536297E38F, -1.9504837E38F, -1.7631667E38F, -4.6468775E37F, -2.4558542E38F, 1.1278903E37F, -1.9558526E38F, 2.3602275E38F, -2.496958E37F, 2.9570928E38F, -3.2622684E38F, 2.4489785E38F, -5.0278117E35F, 1.2597069E38F, 2.4410653E38F, -2.910326E38F, -3.176143E37F, -3.3651716E38F}, 0) ;
            SMP_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)3.2709024E38F);
                Debug.Assert(pack.y == (float)1.8349393E38F);
                Debug.Assert(pack.ay == (float)2.919742E38F);
                Debug.Assert(pack.az == (float)9.465265E37F);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
                Debug.Assert(pack.ax == (float)1.3942995E38F);
                Debug.Assert(pack.x == (float)3.1943996E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.0213898E38F, 3.1191809E38F, -3.2823814E38F, -1.8672506E38F, -3.242265E38F, -1.8007659E37F, 1.5402042E38F, -1.6964273E38F, -2.4384853E38F, 2.3592844E38F, 1.8764283E38F, 8.6064E37F, -1.8911793E38F, 8.4451694E37F, 4.500345E37F, 2.8880932E38F, -1.3280783E38F, 1.9790973E38F, -2.4545865E38F, -1.5295999E38F, -5.8154313E37F, 8.738746E37F, 3.081532E38F, 4.634858E37F, 3.342786E38F, 2.2677239E38F, -3.1611721E38F, -1.1952997E37F, -2.092994E38F, 5.6301814E36F, -2.7999307E38F, -2.7431543E37F, 1.9040753E38F, 1.5756508E38F, 2.7044555E38F, 1.7330183E37F, -2.440926E38F, 3.3854593E38F, 7.605234E37F, -2.3596466E38F, 2.1439272E38F, 3.0369838E37F, -2.5521502E38F, 1.4166615E38F, -5.89125E37F}));
                Debug.Assert(pack.vy == (float) -1.6902076E38F);
                Debug.Assert(pack.vx == (float) -2.2914093E38F);
                Debug.Assert(pack.vz == (float) -3.239033E38F);
                Debug.Assert(pack.time_usec == (ulong)5455717984591587121L);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.az = (float)9.465265E37F;
            p64.covariance_SET(new float[] {-2.0213898E38F, 3.1191809E38F, -3.2823814E38F, -1.8672506E38F, -3.242265E38F, -1.8007659E37F, 1.5402042E38F, -1.6964273E38F, -2.4384853E38F, 2.3592844E38F, 1.8764283E38F, 8.6064E37F, -1.8911793E38F, 8.4451694E37F, 4.500345E37F, 2.8880932E38F, -1.3280783E38F, 1.9790973E38F, -2.4545865E38F, -1.5295999E38F, -5.8154313E37F, 8.738746E37F, 3.081532E38F, 4.634858E37F, 3.342786E38F, 2.2677239E38F, -3.1611721E38F, -1.1952997E37F, -2.092994E38F, 5.6301814E36F, -2.7999307E38F, -2.7431543E37F, 1.9040753E38F, 1.5756508E38F, 2.7044555E38F, 1.7330183E37F, -2.440926E38F, 3.3854593E38F, 7.605234E37F, -2.3596466E38F, 2.1439272E38F, 3.0369838E37F, -2.5521502E38F, 1.4166615E38F, -5.89125E37F}, 0) ;
            p64.ay = (float)2.919742E38F;
            p64.y = (float)1.8349393E38F;
            p64.time_usec = (ulong)5455717984591587121L;
            p64.x = (float)3.1943996E38F;
            p64.vz = (float) -3.239033E38F;
            p64.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO;
            p64.vx = (float) -2.2914093E38F;
            p64.ax = (float)1.3942995E38F;
            p64.vy = (float) -1.6902076E38F;
            p64.z = (float)3.2709024E38F;
            SMP_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)12482);
                Debug.Assert(pack.rssi == (byte)(byte)20);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)31969);
                Debug.Assert(pack.chancount == (byte)(byte)15);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)48450);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)45193);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)64273);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)31474);
                Debug.Assert(pack.time_boot_ms == (uint)804552536U);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)65185);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)32108);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)29326);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)5302);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)6347);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)57469);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)2375);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)6455);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)31373);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)59416);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)33462);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)62495);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan1_raw = (ushort)(ushort)5302;
            p65.chan7_raw = (ushort)(ushort)6455;
            p65.chan15_raw = (ushort)(ushort)57469;
            p65.chan14_raw = (ushort)(ushort)31969;
            p65.chancount = (byte)(byte)15;
            p65.chan17_raw = (ushort)(ushort)33462;
            p65.rssi = (byte)(byte)20;
            p65.chan16_raw = (ushort)(ushort)32108;
            p65.chan9_raw = (ushort)(ushort)12482;
            p65.chan6_raw = (ushort)(ushort)62495;
            p65.chan3_raw = (ushort)(ushort)6347;
            p65.chan8_raw = (ushort)(ushort)64273;
            p65.chan2_raw = (ushort)(ushort)2375;
            p65.chan5_raw = (ushort)(ushort)29326;
            p65.chan10_raw = (ushort)(ushort)31373;
            p65.chan11_raw = (ushort)(ushort)31474;
            p65.chan13_raw = (ushort)(ushort)48450;
            p65.chan12_raw = (ushort)(ushort)65185;
            p65.time_boot_ms = (uint)804552536U;
            p65.chan18_raw = (ushort)(ushort)59416;
            p65.chan4_raw = (ushort)(ushort)45193;
            SMP_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_stop == (byte)(byte)143);
                Debug.Assert(pack.req_stream_id == (byte)(byte)102);
                Debug.Assert(pack.target_component == (byte)(byte)248);
                Debug.Assert(pack.target_system == (byte)(byte)94);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)22445);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_component = (byte)(byte)248;
            p66.req_stream_id = (byte)(byte)102;
            p66.req_message_rate = (ushort)(ushort)22445;
            p66.start_stop = (byte)(byte)143;
            p66.target_system = (byte)(byte)94;
            SMP_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_rate == (ushort)(ushort)24465);
                Debug.Assert(pack.stream_id == (byte)(byte)189);
                Debug.Assert(pack.on_off == (byte)(byte)179);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.stream_id = (byte)(byte)189;
            p67.on_off = (byte)(byte)179;
            p67.message_rate = (ushort)(ushort)24465;
            SMP_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (short)(short) -11528);
                Debug.Assert(pack.z == (short)(short) -590);
                Debug.Assert(pack.r == (short)(short)9012);
                Debug.Assert(pack.y == (short)(short)32610);
                Debug.Assert(pack.target == (byte)(byte)51);
                Debug.Assert(pack.buttons == (ushort)(ushort)32539);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.y = (short)(short)32610;
            p69.z = (short)(short) -590;
            p69.x = (short)(short) -11528;
            p69.target = (byte)(byte)51;
            p69.buttons = (ushort)(ushort)32539;
            p69.r = (short)(short)9012;
            SMP_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)37862);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)25409);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)62233);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)35728);
                Debug.Assert(pack.target_system == (byte)(byte)117);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)36596);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)23635);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)37914);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)21648);
                Debug.Assert(pack.target_component == (byte)(byte)221);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.target_component = (byte)(byte)221;
            p70.chan7_raw = (ushort)(ushort)25409;
            p70.chan8_raw = (ushort)(ushort)37914;
            p70.target_system = (byte)(byte)117;
            p70.chan1_raw = (ushort)(ushort)37862;
            p70.chan2_raw = (ushort)(ushort)21648;
            p70.chan4_raw = (ushort)(ushort)36596;
            p70.chan5_raw = (ushort)(ushort)62233;
            p70.chan3_raw = (ushort)(ushort)35728;
            p70.chan6_raw = (ushort)(ushort)23635;
            SMP_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.autocontinue == (byte)(byte)140);
                Debug.Assert(pack.target_component == (byte)(byte)197);
                Debug.Assert(pack.param3 == (float)3.4830437E37F);
                Debug.Assert(pack.seq == (ushort)(ushort)50548);
                Debug.Assert(pack.target_system == (byte)(byte)182);
                Debug.Assert(pack.y == (int)212198638);
                Debug.Assert(pack.x == (int) -1005883518);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL);
                Debug.Assert(pack.current == (byte)(byte)179);
                Debug.Assert(pack.param4 == (float) -4.5687385E37F);
                Debug.Assert(pack.param2 == (float)4.44326E37F);
                Debug.Assert(pack.param1 == (float)1.7912376E37F);
                Debug.Assert(pack.z == (float) -1.6404094E38F);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.x = (int) -1005883518;
            p73.autocontinue = (byte)(byte)140;
            p73.target_system = (byte)(byte)182;
            p73.z = (float) -1.6404094E38F;
            p73.seq = (ushort)(ushort)50548;
            p73.param2 = (float)4.44326E37F;
            p73.param3 = (float)3.4830437E37F;
            p73.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p73.target_component = (byte)(byte)197;
            p73.param4 = (float) -4.5687385E37F;
            p73.command = MAV_CMD.MAV_CMD_GET_MESSAGE_INTERVAL;
            p73.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p73.current = (byte)(byte)179;
            p73.param1 = (float)1.7912376E37F;
            p73.y = (int)212198638;
            SMP_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.throttle == (ushort)(ushort)59911);
                Debug.Assert(pack.climb == (float) -2.960736E38F);
                Debug.Assert(pack.heading == (short)(short) -24866);
                Debug.Assert(pack.groundspeed == (float)3.4164624E37F);
                Debug.Assert(pack.alt == (float) -1.659476E38F);
                Debug.Assert(pack.airspeed == (float) -1.761039E36F);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.heading = (short)(short) -24866;
            p74.airspeed = (float) -1.761039E36F;
            p74.throttle = (ushort)(ushort)59911;
            p74.alt = (float) -1.659476E38F;
            p74.climb = (float) -2.960736E38F;
            p74.groundspeed = (float)3.4164624E37F;
            SMP_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (int)1507537493);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.z == (float)1.6357184E38F);
                Debug.Assert(pack.x == (int)906888111);
                Debug.Assert(pack.target_system == (byte)(byte)142);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_LAND_START);
                Debug.Assert(pack.param1 == (float)1.4651391E38F);
                Debug.Assert(pack.param4 == (float)2.127473E38F);
                Debug.Assert(pack.param3 == (float)4.3743717E37F);
                Debug.Assert(pack.target_component == (byte)(byte)224);
                Debug.Assert(pack.autocontinue == (byte)(byte)79);
                Debug.Assert(pack.current == (byte)(byte)211);
                Debug.Assert(pack.param2 == (float) -3.0630978E38F);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.param2 = (float) -3.0630978E38F;
            p75.current = (byte)(byte)211;
            p75.y = (int)1507537493;
            p75.target_component = (byte)(byte)224;
            p75.param1 = (float)1.4651391E38F;
            p75.param3 = (float)4.3743717E37F;
            p75.command = MAV_CMD.MAV_CMD_DO_LAND_START;
            p75.z = (float)1.6357184E38F;
            p75.target_system = (byte)(byte)142;
            p75.param4 = (float)2.127473E38F;
            p75.autocontinue = (byte)(byte)79;
            p75.x = (int)906888111;
            p75.frame = MAV_FRAME.MAV_FRAME_MISSION;
            SMP_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param1 == (float)1.2464877E38F);
                Debug.Assert(pack.param6 == (float) -2.5694825E38F);
                Debug.Assert(pack.confirmation == (byte)(byte)31);
                Debug.Assert(pack.param4 == (float) -7.715634E36F);
                Debug.Assert(pack.param5 == (float)1.8432682E38F);
                Debug.Assert(pack.param3 == (float)1.9137946E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN);
                Debug.Assert(pack.target_system == (byte)(byte)82);
                Debug.Assert(pack.param2 == (float) -6.49761E36F);
                Debug.Assert(pack.param7 == (float) -7.2223916E37F);
                Debug.Assert(pack.target_component == (byte)(byte)236);
            };
            GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.param6 = (float) -2.5694825E38F;
            p76.param2 = (float) -6.49761E36F;
            p76.param1 = (float)1.2464877E38F;
            p76.target_system = (byte)(byte)82;
            p76.param3 = (float)1.9137946E38F;
            p76.param7 = (float) -7.2223916E37F;
            p76.param5 = (float)1.8432682E38F;
            p76.target_component = (byte)(byte)236;
            p76.confirmation = (byte)(byte)31;
            p76.command = MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
            p76.param4 = (float) -7.715634E36F;
            CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)250);
                Debug.Assert(pack.result_param2_TRY(ph) == (int)514134148);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_LOITER_TURNS);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)42);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)149);
                Debug.Assert(pack.result == MAV_RESULT.MAV_RESULT_UNSUPPORTED);
            };
            GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.command = MAV_CMD.MAV_CMD_NAV_LOITER_TURNS;
            p77.progress_SET((byte)(byte)42, PH) ;
            p77.result = MAV_RESULT.MAV_RESULT_UNSUPPORTED;
            p77.result_param2_SET((int)514134148, PH) ;
            p77.target_component_SET((byte)(byte)149, PH) ;
            p77.target_system_SET((byte)(byte)250, PH) ;
            CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)77558237U);
                Debug.Assert(pack.mode_switch == (byte)(byte)64);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)232);
                Debug.Assert(pack.thrust == (float) -2.0471957E38F);
                Debug.Assert(pack.pitch == (float) -2.0862525E38F);
                Debug.Assert(pack.yaw == (float) -2.8440555E38F);
                Debug.Assert(pack.roll == (float) -2.654951E38F);
            };
            GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.mode_switch = (byte)(byte)64;
            p81.roll = (float) -2.654951E38F;
            p81.thrust = (float) -2.0471957E38F;
            p81.pitch = (float) -2.0862525E38F;
            p81.time_boot_ms = (uint)77558237U;
            p81.yaw = (float) -2.8440555E38F;
            p81.manual_override_switch = (byte)(byte)232;
            CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.thrust == (float) -3.1594187E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.7448275E38F, -1.1974416E38F, -2.0557987E38F, -1.0178167E38F}));
                Debug.Assert(pack.body_roll_rate == (float) -3.2968677E37F);
                Debug.Assert(pack.target_component == (byte)(byte)25);
                Debug.Assert(pack.body_yaw_rate == (float) -5.5061636E37F);
                Debug.Assert(pack.time_boot_ms == (uint)922449903U);
                Debug.Assert(pack.target_system == (byte)(byte)94);
                Debug.Assert(pack.type_mask == (byte)(byte)245);
                Debug.Assert(pack.body_pitch_rate == (float) -3.2125674E37F);
            };
            GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.body_yaw_rate = (float) -5.5061636E37F;
            p82.time_boot_ms = (uint)922449903U;
            p82.type_mask = (byte)(byte)245;
            p82.body_pitch_rate = (float) -3.2125674E37F;
            p82.target_system = (byte)(byte)94;
            p82.thrust = (float) -3.1594187E38F;
            p82.body_roll_rate = (float) -3.2968677E37F;
            p82.target_component = (byte)(byte)25;
            p82.q_SET(new float[] {-1.7448275E38F, -1.1974416E38F, -2.0557987E38F, -1.0178167E38F}, 0) ;
            CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.978574E38F, -2.7078181E38F, -2.022161E38F, 2.6973913E38F}));
                Debug.Assert(pack.body_yaw_rate == (float)1.4369437E38F);
                Debug.Assert(pack.time_boot_ms == (uint)256114046U);
                Debug.Assert(pack.body_roll_rate == (float) -4.4632953E37F);
                Debug.Assert(pack.type_mask == (byte)(byte)137);
                Debug.Assert(pack.body_pitch_rate == (float) -1.9854784E37F);
                Debug.Assert(pack.thrust == (float) -8.1005363E37F);
            };
            GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.body_pitch_rate = (float) -1.9854784E37F;
            p83.time_boot_ms = (uint)256114046U;
            p83.q_SET(new float[] {-2.978574E38F, -2.7078181E38F, -2.022161E38F, 2.6973913E38F}, 0) ;
            p83.thrust = (float) -8.1005363E37F;
            p83.type_mask = (byte)(byte)137;
            p83.body_yaw_rate = (float)1.4369437E38F;
            p83.body_roll_rate = (float) -4.4632953E37F;
            CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float) -1.2637453E37F);
                Debug.Assert(pack.y == (float) -1.7811734E37F);
                Debug.Assert(pack.z == (float)1.2711655E38F);
                Debug.Assert(pack.x == (float)3.9669683E37F);
                Debug.Assert(pack.vy == (float) -2.3379213E38F);
                Debug.Assert(pack.yaw == (float) -1.9065463E38F);
                Debug.Assert(pack.afy == (float) -6.4198187E37F);
                Debug.Assert(pack.afz == (float)7.5304307E37F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.yaw_rate == (float) -1.826218E38F);
                Debug.Assert(pack.time_boot_ms == (uint)454007807U);
                Debug.Assert(pack.target_system == (byte)(byte)224);
                Debug.Assert(pack.afx == (float) -6.862996E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)28223);
                Debug.Assert(pack.target_component == (byte)(byte)189);
                Debug.Assert(pack.vz == (float) -1.5014021E38F);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.y = (float) -1.7811734E37F;
            p84.time_boot_ms = (uint)454007807U;
            p84.vz = (float) -1.5014021E38F;
            p84.target_component = (byte)(byte)189;
            p84.vx = (float) -1.2637453E37F;
            p84.z = (float)1.2711655E38F;
            p84.yaw_rate = (float) -1.826218E38F;
            p84.yaw = (float) -1.9065463E38F;
            p84.afy = (float) -6.4198187E37F;
            p84.x = (float)3.9669683E37F;
            p84.vy = (float) -2.3379213E38F;
            p84.type_mask = (ushort)(ushort)28223;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p84.afx = (float) -6.862996E37F;
            p84.target_system = (byte)(byte)224;
            p84.afz = (float)7.5304307E37F;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)179);
                Debug.Assert(pack.yaw == (float)2.5884026E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.lat_int == (int)1069170404);
                Debug.Assert(pack.time_boot_ms == (uint)2218022070U);
                Debug.Assert(pack.afx == (float) -1.7762257E38F);
                Debug.Assert(pack.vx == (float) -7.49135E37F);
                Debug.Assert(pack.vy == (float) -4.8711797E37F);
                Debug.Assert(pack.vz == (float) -9.973442E37F);
                Debug.Assert(pack.afy == (float) -6.2436533E37F);
                Debug.Assert(pack.afz == (float)1.3779678E38F);
                Debug.Assert(pack.target_component == (byte)(byte)127);
                Debug.Assert(pack.type_mask == (ushort)(ushort)33746);
                Debug.Assert(pack.lon_int == (int) -195593425);
                Debug.Assert(pack.yaw_rate == (float) -8.3234344E37F);
                Debug.Assert(pack.alt == (float)2.5693132E38F);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.target_system = (byte)(byte)179;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p86.yaw_rate = (float) -8.3234344E37F;
            p86.vx = (float) -7.49135E37F;
            p86.time_boot_ms = (uint)2218022070U;
            p86.yaw = (float)2.5884026E38F;
            p86.afz = (float)1.3779678E38F;
            p86.vy = (float) -4.8711797E37F;
            p86.lat_int = (int)1069170404;
            p86.lon_int = (int) -195593425;
            p86.target_component = (byte)(byte)127;
            p86.alt = (float)2.5693132E38F;
            p86.afx = (float) -1.7762257E38F;
            p86.afy = (float) -6.2436533E37F;
            p86.vz = (float) -9.973442E37F;
            p86.type_mask = (ushort)(ushort)33746;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float) -2.3690493E38F);
                Debug.Assert(pack.yaw == (float)1.2575406E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)7032);
                Debug.Assert(pack.lon_int == (int) -1301746106);
                Debug.Assert(pack.alt == (float) -2.8503858E38F);
                Debug.Assert(pack.afx == (float)2.5794645E38F);
                Debug.Assert(pack.lat_int == (int) -265550027);
                Debug.Assert(pack.time_boot_ms == (uint)3438286939U);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.yaw_rate == (float)7.2747786E37F);
                Debug.Assert(pack.afy == (float) -3.6804225E37F);
                Debug.Assert(pack.afz == (float) -1.4500777E38F);
                Debug.Assert(pack.vy == (float) -1.2342604E38F);
                Debug.Assert(pack.vx == (float) -2.6105003E38F);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.yaw_rate = (float)7.2747786E37F;
            p87.lon_int = (int) -1301746106;
            p87.afx = (float)2.5794645E38F;
            p87.alt = (float) -2.8503858E38F;
            p87.afz = (float) -1.4500777E38F;
            p87.vy = (float) -1.2342604E38F;
            p87.lat_int = (int) -265550027;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p87.yaw = (float)1.2575406E38F;
            p87.afy = (float) -3.6804225E37F;
            p87.vx = (float) -2.6105003E38F;
            p87.vz = (float) -2.3690493E38F;
            p87.time_boot_ms = (uint)3438286939U;
            p87.type_mask = (ushort)(ushort)7032;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)3.5235685E37F);
                Debug.Assert(pack.roll == (float) -1.9879913E37F);
                Debug.Assert(pack.z == (float)3.1956003E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2091917284U);
                Debug.Assert(pack.x == (float) -2.2045622E38F);
                Debug.Assert(pack.pitch == (float)1.4704338E38F);
                Debug.Assert(pack.y == (float) -1.7455685E38F);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.yaw = (float)3.5235685E37F;
            p89.roll = (float) -1.9879913E37F;
            p89.x = (float) -2.2045622E38F;
            p89.time_boot_ms = (uint)2091917284U;
            p89.pitch = (float)1.4704338E38F;
            p89.y = (float) -1.7455685E38F;
            p89.z = (float)3.1956003E38F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -1703286816);
                Debug.Assert(pack.yacc == (short)(short)16856);
                Debug.Assert(pack.vy == (short)(short) -28710);
                Debug.Assert(pack.vz == (short)(short)15427);
                Debug.Assert(pack.xacc == (short)(short) -12955);
                Debug.Assert(pack.vx == (short)(short)7831);
                Debug.Assert(pack.zacc == (short)(short)22186);
                Debug.Assert(pack.pitchspeed == (float)1.2212762E38F);
                Debug.Assert(pack.yawspeed == (float)2.0566061E38F);
                Debug.Assert(pack.time_usec == (ulong)1216365129194782575L);
                Debug.Assert(pack.lat == (int)1827937368);
                Debug.Assert(pack.alt == (int)1740859391);
                Debug.Assert(pack.yaw == (float) -1.0076288E38F);
                Debug.Assert(pack.roll == (float) -2.171373E38F);
                Debug.Assert(pack.rollspeed == (float) -2.7447424E36F);
                Debug.Assert(pack.pitch == (float)1.8667577E38F);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.alt = (int)1740859391;
            p90.rollspeed = (float) -2.7447424E36F;
            p90.pitchspeed = (float)1.2212762E38F;
            p90.yacc = (short)(short)16856;
            p90.vx = (short)(short)7831;
            p90.zacc = (short)(short)22186;
            p90.time_usec = (ulong)1216365129194782575L;
            p90.roll = (float) -2.171373E38F;
            p90.lon = (int) -1703286816;
            p90.lat = (int)1827937368;
            p90.yawspeed = (float)2.0566061E38F;
            p90.xacc = (short)(short) -12955;
            p90.yaw = (float) -1.0076288E38F;
            p90.vy = (short)(short) -28710;
            p90.vz = (short)(short)15427;
            p90.pitch = (float)1.8667577E38F;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.throttle == (float)3.0256658E38F);
                Debug.Assert(pack.pitch_elevator == (float)3.3576507E38F);
                Debug.Assert(pack.yaw_rudder == (float) -1.8997334E38F);
                Debug.Assert(pack.roll_ailerons == (float)2.6225328E38F);
                Debug.Assert(pack.nav_mode == (byte)(byte)216);
                Debug.Assert(pack.aux3 == (float) -3.275821E38F);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
                Debug.Assert(pack.aux2 == (float) -2.1173345E38F);
                Debug.Assert(pack.aux4 == (float)2.4881948E37F);
                Debug.Assert(pack.aux1 == (float)1.3778411E38F);
                Debug.Assert(pack.time_usec == (ulong)7625487667716620694L);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.aux2 = (float) -2.1173345E38F;
            p91.roll_ailerons = (float)2.6225328E38F;
            p91.aux4 = (float)2.4881948E37F;
            p91.pitch_elevator = (float)3.3576507E38F;
            p91.aux3 = (float) -3.275821E38F;
            p91.throttle = (float)3.0256658E38F;
            p91.time_usec = (ulong)7625487667716620694L;
            p91.mode = MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            p91.nav_mode = (byte)(byte)216;
            p91.aux1 = (float)1.3778411E38F;
            p91.yaw_rudder = (float) -1.8997334E38F;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)1689626358378864628L);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)25608);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)3932);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)21547);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)26741);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)50981);
                Debug.Assert(pack.rssi == (byte)(byte)13);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)33257);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)5602);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)7208);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)54318);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)43828);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)28645);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)56518);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)1689626358378864628L;
            p92.chan3_raw = (ushort)(ushort)43828;
            p92.chan2_raw = (ushort)(ushort)28645;
            p92.chan6_raw = (ushort)(ushort)26741;
            p92.chan7_raw = (ushort)(ushort)50981;
            p92.chan4_raw = (ushort)(ushort)21547;
            p92.chan10_raw = (ushort)(ushort)33257;
            p92.chan8_raw = (ushort)(ushort)7208;
            p92.chan12_raw = (ushort)(ushort)5602;
            p92.chan9_raw = (ushort)(ushort)56518;
            p92.chan5_raw = (ushort)(ushort)25608;
            p92.chan1_raw = (ushort)(ushort)54318;
            p92.chan11_raw = (ushort)(ushort)3932;
            p92.rssi = (byte)(byte)13;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (ulong)8423444040280578931L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {2.6053666E38F, 2.6419532E38F, 1.4346613E38F, 2.3845605E38F, 2.713739E38F, -1.2163253E38F, 9.629237E37F, -1.3816454E38F, 1.6920185E38F, -2.6484594E35F, 2.5031587E38F, -8.902218E36F, 5.3528823E37F, -4.617216E37F, -2.9178147E38F, 1.3744726E37F}));
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
                Debug.Assert(pack.time_usec == (ulong)237645240223042565L);
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.controls_SET(new float[] {2.6053666E38F, 2.6419532E38F, 1.4346613E38F, 2.3845605E38F, 2.713739E38F, -1.2163253E38F, 9.629237E37F, -1.3816454E38F, 1.6920185E38F, -2.6484594E35F, 2.5031587E38F, -8.902218E36F, 5.3528823E37F, -4.617216E37F, -2.9178147E38F, 1.3744726E37F}, 0) ;
            p93.flags = (ulong)8423444040280578931L;
            p93.mode = MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            p93.time_usec = (ulong)237645240223042565L;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_id == (byte)(byte)234);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float) -2.3220402E38F);
                Debug.Assert(pack.flow_comp_m_y == (float) -3.2637523E38F);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float) -3.127472E38F);
                Debug.Assert(pack.time_usec == (ulong)4565427467410028340L);
                Debug.Assert(pack.quality == (byte)(byte)141);
                Debug.Assert(pack.flow_x == (short)(short)6808);
                Debug.Assert(pack.flow_comp_m_x == (float)3.2524288E38F);
                Debug.Assert(pack.flow_y == (short)(short)13078);
                Debug.Assert(pack.ground_distance == (float) -1.7812852E38F);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.ground_distance = (float) -1.7812852E38F;
            p100.flow_rate_x_SET((float) -2.3220402E38F, PH) ;
            p100.sensor_id = (byte)(byte)234;
            p100.flow_comp_m_x = (float)3.2524288E38F;
            p100.quality = (byte)(byte)141;
            p100.flow_y = (short)(short)13078;
            p100.time_usec = (ulong)4565427467410028340L;
            p100.flow_x = (short)(short)6808;
            p100.flow_comp_m_y = (float) -3.2637523E38F;
            p100.flow_rate_y_SET((float) -3.127472E38F, PH) ;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)9.010168E37F);
                Debug.Assert(pack.pitch == (float)3.779417E37F);
                Debug.Assert(pack.usec == (ulong)8035594603781867690L);
                Debug.Assert(pack.z == (float) -1.8097116E38F);
                Debug.Assert(pack.y == (float)1.1107819E38F);
                Debug.Assert(pack.roll == (float) -3.3974724E38F);
                Debug.Assert(pack.yaw == (float) -3.2906398E38F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.roll = (float) -3.3974724E38F;
            p101.x = (float)9.010168E37F;
            p101.z = (float) -1.8097116E38F;
            p101.y = (float)1.1107819E38F;
            p101.usec = (ulong)8035594603781867690L;
            p101.yaw = (float) -3.2906398E38F;
            p101.pitch = (float)3.779417E37F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -8.244721E37F);
                Debug.Assert(pack.z == (float) -6.7557593E37F);
                Debug.Assert(pack.usec == (ulong)6926973321359843700L);
                Debug.Assert(pack.x == (float)1.534901E38F);
                Debug.Assert(pack.yaw == (float) -2.3190867E38F);
                Debug.Assert(pack.roll == (float)1.0957894E38F);
                Debug.Assert(pack.y == (float) -2.1747228E37F);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.y = (float) -2.1747228E37F;
            p102.usec = (ulong)6926973321359843700L;
            p102.roll = (float)1.0957894E38F;
            p102.yaw = (float) -2.3190867E38F;
            p102.x = (float)1.534901E38F;
            p102.z = (float) -6.7557593E37F;
            p102.pitch = (float) -8.244721E37F;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -2.5502574E38F);
                Debug.Assert(pack.y == (float)1.7528718E38F);
                Debug.Assert(pack.x == (float)2.474349E38F);
                Debug.Assert(pack.usec == (ulong)5914874007039582821L);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.z = (float) -2.5502574E38F;
            p103.x = (float)2.474349E38F;
            p103.y = (float)1.7528718E38F;
            p103.usec = (ulong)5914874007039582821L;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)1.1945748E38F);
                Debug.Assert(pack.roll == (float)1.3511124E38F);
                Debug.Assert(pack.x == (float) -7.4621353E37F);
                Debug.Assert(pack.pitch == (float)7.8704613E37F);
                Debug.Assert(pack.z == (float)3.9020775E37F);
                Debug.Assert(pack.y == (float) -3.1731943E38F);
                Debug.Assert(pack.usec == (ulong)6189099516652092375L);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)6189099516652092375L;
            p104.yaw = (float)1.1945748E38F;
            p104.z = (float)3.9020775E37F;
            p104.roll = (float)1.3511124E38F;
            p104.pitch = (float)7.8704613E37F;
            p104.y = (float) -3.1731943E38F;
            p104.x = (float) -7.4621353E37F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (float) -6.5051676E37F);
                Debug.Assert(pack.abs_pressure == (float) -2.7417429E38F);
                Debug.Assert(pack.pressure_alt == (float)2.567284E38F);
                Debug.Assert(pack.xgyro == (float)3.1700183E38F);
                Debug.Assert(pack.zmag == (float) -1.4092911E38F);
                Debug.Assert(pack.time_usec == (ulong)5866615550403952717L);
                Debug.Assert(pack.zgyro == (float)1.6364904E38F);
                Debug.Assert(pack.temperature == (float) -3.2521872E38F);
                Debug.Assert(pack.ygyro == (float)1.3129748E38F);
                Debug.Assert(pack.ymag == (float)1.7514392E38F);
                Debug.Assert(pack.diff_pressure == (float) -3.4527717E37F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)15749);
                Debug.Assert(pack.zacc == (float)3.1372949E38F);
                Debug.Assert(pack.yacc == (float) -2.9542658E38F);
                Debug.Assert(pack.xacc == (float)2.322014E37F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.zacc = (float)3.1372949E38F;
            p105.diff_pressure = (float) -3.4527717E37F;
            p105.pressure_alt = (float)2.567284E38F;
            p105.yacc = (float) -2.9542658E38F;
            p105.xacc = (float)2.322014E37F;
            p105.xmag = (float) -6.5051676E37F;
            p105.zgyro = (float)1.6364904E38F;
            p105.temperature = (float) -3.2521872E38F;
            p105.fields_updated = (ushort)(ushort)15749;
            p105.xgyro = (float)3.1700183E38F;
            p105.ygyro = (float)1.3129748E38F;
            p105.ymag = (float)1.7514392E38F;
            p105.time_usec = (ulong)5866615550403952717L;
            p105.abs_pressure = (float) -2.7417429E38F;
            p105.zmag = (float) -1.4092911E38F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.quality == (byte)(byte)222);
                Debug.Assert(pack.time_delta_distance_us == (uint)3871928903U);
                Debug.Assert(pack.integrated_xgyro == (float) -6.0987974E37F);
                Debug.Assert(pack.distance == (float) -4.438955E37F);
                Debug.Assert(pack.integrated_ygyro == (float)8.383037E37F);
                Debug.Assert(pack.integrated_zgyro == (float)1.3871656E37F);
                Debug.Assert(pack.time_usec == (ulong)2069844211376652553L);
                Debug.Assert(pack.integration_time_us == (uint)2759182466U);
                Debug.Assert(pack.integrated_x == (float) -2.8572064E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)114);
                Debug.Assert(pack.integrated_y == (float)2.1092012E38F);
                Debug.Assert(pack.temperature == (short)(short)28181);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.integrated_x = (float) -2.8572064E38F;
            p106.integrated_zgyro = (float)1.3871656E37F;
            p106.quality = (byte)(byte)222;
            p106.sensor_id = (byte)(byte)114;
            p106.time_usec = (ulong)2069844211376652553L;
            p106.integrated_ygyro = (float)8.383037E37F;
            p106.time_delta_distance_us = (uint)3871928903U;
            p106.integrated_y = (float)2.1092012E38F;
            p106.integration_time_us = (uint)2759182466U;
            p106.integrated_xgyro = (float) -6.0987974E37F;
            p106.distance = (float) -4.438955E37F;
            p106.temperature = (short)(short)28181;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (float) -2.8181905E38F);
                Debug.Assert(pack.temperature == (float)1.4411205E38F);
                Debug.Assert(pack.ygyro == (float)2.6703932E38F);
                Debug.Assert(pack.fields_updated == (uint)4010894448U);
                Debug.Assert(pack.abs_pressure == (float) -1.2844695E38F);
                Debug.Assert(pack.yacc == (float)1.662808E38F);
                Debug.Assert(pack.zacc == (float) -1.542068E38F);
                Debug.Assert(pack.ymag == (float) -1.318426E38F);
                Debug.Assert(pack.diff_pressure == (float)3.1387853E37F);
                Debug.Assert(pack.pressure_alt == (float)1.2361674E38F);
                Debug.Assert(pack.time_usec == (ulong)2345230455764436223L);
                Debug.Assert(pack.zmag == (float)8.292079E37F);
                Debug.Assert(pack.zgyro == (float)1.4080473E38F);
                Debug.Assert(pack.xgyro == (float)6.9874777E37F);
                Debug.Assert(pack.xacc == (float)3.6339093E36F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.xmag = (float) -2.8181905E38F;
            p107.yacc = (float)1.662808E38F;
            p107.xacc = (float)3.6339093E36F;
            p107.xgyro = (float)6.9874777E37F;
            p107.diff_pressure = (float)3.1387853E37F;
            p107.time_usec = (ulong)2345230455764436223L;
            p107.zmag = (float)8.292079E37F;
            p107.zacc = (float) -1.542068E38F;
            p107.temperature = (float)1.4411205E38F;
            p107.ymag = (float) -1.318426E38F;
            p107.zgyro = (float)1.4080473E38F;
            p107.ygyro = (float)2.6703932E38F;
            p107.abs_pressure = (float) -1.2844695E38F;
            p107.pressure_alt = (float)1.2361674E38F;
            p107.fields_updated = (uint)4010894448U;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vn == (float) -1.1319383E38F);
                Debug.Assert(pack.q1 == (float)2.5816343E38F);
                Debug.Assert(pack.q2 == (float)1.293552E36F);
                Debug.Assert(pack.xgyro == (float)1.3430532E38F);
                Debug.Assert(pack.roll == (float) -1.0773513E38F);
                Debug.Assert(pack.pitch == (float) -2.1598957E38F);
                Debug.Assert(pack.vd == (float) -2.4919803E37F);
                Debug.Assert(pack.ygyro == (float) -1.102668E38F);
                Debug.Assert(pack.q3 == (float) -2.0320572E37F);
                Debug.Assert(pack.alt == (float)2.256648E38F);
                Debug.Assert(pack.lon == (float)2.9157173E38F);
                Debug.Assert(pack.std_dev_vert == (float)2.9458993E38F);
                Debug.Assert(pack.xacc == (float)2.3796615E38F);
                Debug.Assert(pack.yaw == (float)9.1411514E36F);
                Debug.Assert(pack.zgyro == (float)1.7465382E38F);
                Debug.Assert(pack.lat == (float)1.7891126E37F);
                Debug.Assert(pack.yacc == (float) -2.5107047E38F);
                Debug.Assert(pack.q4 == (float) -3.5774895E37F);
                Debug.Assert(pack.zacc == (float)3.2849496E38F);
                Debug.Assert(pack.ve == (float) -8.835976E36F);
                Debug.Assert(pack.std_dev_horz == (float) -3.292825E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.zgyro = (float)1.7465382E38F;
            p108.alt = (float)2.256648E38F;
            p108.ve = (float) -8.835976E36F;
            p108.std_dev_vert = (float)2.9458993E38F;
            p108.zacc = (float)3.2849496E38F;
            p108.lat = (float)1.7891126E37F;
            p108.xacc = (float)2.3796615E38F;
            p108.q4 = (float) -3.5774895E37F;
            p108.vn = (float) -1.1319383E38F;
            p108.q2 = (float)1.293552E36F;
            p108.pitch = (float) -2.1598957E38F;
            p108.vd = (float) -2.4919803E37F;
            p108.ygyro = (float) -1.102668E38F;
            p108.yacc = (float) -2.5107047E38F;
            p108.q3 = (float) -2.0320572E37F;
            p108.xgyro = (float)1.3430532E38F;
            p108.q1 = (float)2.5816343E38F;
            p108.roll = (float) -1.0773513E38F;
            p108.std_dev_horz = (float) -3.292825E38F;
            p108.yaw = (float)9.1411514E36F;
            p108.lon = (float)2.9157173E38F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rssi == (byte)(byte)68);
                Debug.Assert(pack.remrssi == (byte)(byte)247);
                Debug.Assert(pack.remnoise == (byte)(byte)199);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)62055);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)44863);
                Debug.Assert(pack.txbuf == (byte)(byte)175);
                Debug.Assert(pack.noise == (byte)(byte)141);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rssi = (byte)(byte)68;
            p109.fixed_ = (ushort)(ushort)44863;
            p109.remnoise = (byte)(byte)199;
            p109.noise = (byte)(byte)141;
            p109.remrssi = (byte)(byte)247;
            p109.rxerrors = (ushort)(ushort)62055;
            p109.txbuf = (byte)(byte)175;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_network == (byte)(byte)228);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)178, (byte)224, (byte)13, (byte)214, (byte)94, (byte)83, (byte)193, (byte)24, (byte)173, (byte)243, (byte)166, (byte)1, (byte)253, (byte)136, (byte)246, (byte)211, (byte)136, (byte)96, (byte)254, (byte)146, (byte)77, (byte)24, (byte)149, (byte)255, (byte)251, (byte)147, (byte)245, (byte)116, (byte)98, (byte)234, (byte)159, (byte)238, (byte)195, (byte)86, (byte)82, (byte)211, (byte)10, (byte)145, (byte)159, (byte)47, (byte)169, (byte)175, (byte)140, (byte)201, (byte)65, (byte)243, (byte)139, (byte)239, (byte)251, (byte)209, (byte)182, (byte)110, (byte)237, (byte)188, (byte)120, (byte)222, (byte)13, (byte)201, (byte)62, (byte)251, (byte)35, (byte)69, (byte)75, (byte)110, (byte)166, (byte)19, (byte)104, (byte)110, (byte)190, (byte)28, (byte)9, (byte)147, (byte)205, (byte)125, (byte)148, (byte)187, (byte)193, (byte)104, (byte)238, (byte)9, (byte)205, (byte)240, (byte)48, (byte)102, (byte)246, (byte)174, (byte)71, (byte)97, (byte)80, (byte)212, (byte)62, (byte)3, (byte)58, (byte)27, (byte)233, (byte)227, (byte)38, (byte)23, (byte)22, (byte)245, (byte)64, (byte)189, (byte)29, (byte)222, (byte)239, (byte)75, (byte)255, (byte)88, (byte)117, (byte)60, (byte)47, (byte)70, (byte)53, (byte)201, (byte)180, (byte)124, (byte)89, (byte)86, (byte)197, (byte)171, (byte)61, (byte)133, (byte)31, (byte)12, (byte)85, (byte)191, (byte)86, (byte)217, (byte)225, (byte)75, (byte)94, (byte)130, (byte)13, (byte)2, (byte)113, (byte)16, (byte)1, (byte)69, (byte)78, (byte)217, (byte)46, (byte)144, (byte)201, (byte)218, (byte)79, (byte)195, (byte)214, (byte)157, (byte)185, (byte)175, (byte)110, (byte)124, (byte)61, (byte)43, (byte)252, (byte)101, (byte)51, (byte)183, (byte)67, (byte)118, (byte)238, (byte)191, (byte)239, (byte)191, (byte)84, (byte)12, (byte)65, (byte)42, (byte)196, (byte)39, (byte)151, (byte)161, (byte)161, (byte)14, (byte)37, (byte)140, (byte)211, (byte)184, (byte)110, (byte)246, (byte)113, (byte)26, (byte)151, (byte)88, (byte)170, (byte)106, (byte)109, (byte)62, (byte)3, (byte)238, (byte)149, (byte)190, (byte)13, (byte)10, (byte)124, (byte)42, (byte)195, (byte)225, (byte)152, (byte)97, (byte)164, (byte)49, (byte)139, (byte)58, (byte)200, (byte)81, (byte)35, (byte)105, (byte)151, (byte)167, (byte)158, (byte)178, (byte)215, (byte)31, (byte)184, (byte)198, (byte)143, (byte)46, (byte)147, (byte)201, (byte)17, (byte)136, (byte)25, (byte)186, (byte)223, (byte)30, (byte)34, (byte)209, (byte)217, (byte)50, (byte)145, (byte)170, (byte)119, (byte)220, (byte)25, (byte)51, (byte)223, (byte)99, (byte)62, (byte)186, (byte)71, (byte)128, (byte)232, (byte)155, (byte)16, (byte)67, (byte)19, (byte)144, (byte)84, (byte)129, (byte)165}));
                Debug.Assert(pack.target_system == (byte)(byte)128);
                Debug.Assert(pack.target_component == (byte)(byte)229);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_system = (byte)(byte)128;
            p110.target_component = (byte)(byte)229;
            p110.payload_SET(new byte[] {(byte)178, (byte)224, (byte)13, (byte)214, (byte)94, (byte)83, (byte)193, (byte)24, (byte)173, (byte)243, (byte)166, (byte)1, (byte)253, (byte)136, (byte)246, (byte)211, (byte)136, (byte)96, (byte)254, (byte)146, (byte)77, (byte)24, (byte)149, (byte)255, (byte)251, (byte)147, (byte)245, (byte)116, (byte)98, (byte)234, (byte)159, (byte)238, (byte)195, (byte)86, (byte)82, (byte)211, (byte)10, (byte)145, (byte)159, (byte)47, (byte)169, (byte)175, (byte)140, (byte)201, (byte)65, (byte)243, (byte)139, (byte)239, (byte)251, (byte)209, (byte)182, (byte)110, (byte)237, (byte)188, (byte)120, (byte)222, (byte)13, (byte)201, (byte)62, (byte)251, (byte)35, (byte)69, (byte)75, (byte)110, (byte)166, (byte)19, (byte)104, (byte)110, (byte)190, (byte)28, (byte)9, (byte)147, (byte)205, (byte)125, (byte)148, (byte)187, (byte)193, (byte)104, (byte)238, (byte)9, (byte)205, (byte)240, (byte)48, (byte)102, (byte)246, (byte)174, (byte)71, (byte)97, (byte)80, (byte)212, (byte)62, (byte)3, (byte)58, (byte)27, (byte)233, (byte)227, (byte)38, (byte)23, (byte)22, (byte)245, (byte)64, (byte)189, (byte)29, (byte)222, (byte)239, (byte)75, (byte)255, (byte)88, (byte)117, (byte)60, (byte)47, (byte)70, (byte)53, (byte)201, (byte)180, (byte)124, (byte)89, (byte)86, (byte)197, (byte)171, (byte)61, (byte)133, (byte)31, (byte)12, (byte)85, (byte)191, (byte)86, (byte)217, (byte)225, (byte)75, (byte)94, (byte)130, (byte)13, (byte)2, (byte)113, (byte)16, (byte)1, (byte)69, (byte)78, (byte)217, (byte)46, (byte)144, (byte)201, (byte)218, (byte)79, (byte)195, (byte)214, (byte)157, (byte)185, (byte)175, (byte)110, (byte)124, (byte)61, (byte)43, (byte)252, (byte)101, (byte)51, (byte)183, (byte)67, (byte)118, (byte)238, (byte)191, (byte)239, (byte)191, (byte)84, (byte)12, (byte)65, (byte)42, (byte)196, (byte)39, (byte)151, (byte)161, (byte)161, (byte)14, (byte)37, (byte)140, (byte)211, (byte)184, (byte)110, (byte)246, (byte)113, (byte)26, (byte)151, (byte)88, (byte)170, (byte)106, (byte)109, (byte)62, (byte)3, (byte)238, (byte)149, (byte)190, (byte)13, (byte)10, (byte)124, (byte)42, (byte)195, (byte)225, (byte)152, (byte)97, (byte)164, (byte)49, (byte)139, (byte)58, (byte)200, (byte)81, (byte)35, (byte)105, (byte)151, (byte)167, (byte)158, (byte)178, (byte)215, (byte)31, (byte)184, (byte)198, (byte)143, (byte)46, (byte)147, (byte)201, (byte)17, (byte)136, (byte)25, (byte)186, (byte)223, (byte)30, (byte)34, (byte)209, (byte)217, (byte)50, (byte)145, (byte)170, (byte)119, (byte)220, (byte)25, (byte)51, (byte)223, (byte)99, (byte)62, (byte)186, (byte)71, (byte)128, (byte)232, (byte)155, (byte)16, (byte)67, (byte)19, (byte)144, (byte)84, (byte)129, (byte)165}, 0) ;
            p110.target_network = (byte)(byte)228;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tc1 == (long)4303840233321652604L);
                Debug.Assert(pack.ts1 == (long)2228574293100582486L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long)4303840233321652604L;
            p111.ts1 = (long)2228574293100582486L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)4187550465806738518L);
                Debug.Assert(pack.seq == (uint)2392957450U);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)4187550465806738518L;
            p112.seq = (uint)2392957450U;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)4180083650127133658L);
                Debug.Assert(pack.satellites_visible == (byte)(byte)100);
                Debug.Assert(pack.eph == (ushort)(ushort)59878);
                Debug.Assert(pack.vd == (short)(short)11188);
                Debug.Assert(pack.cog == (ushort)(ushort)54525);
                Debug.Assert(pack.ve == (short)(short)31558);
                Debug.Assert(pack.lon == (int)1092189279);
                Debug.Assert(pack.vel == (ushort)(ushort)40145);
                Debug.Assert(pack.lat == (int)243304329);
                Debug.Assert(pack.epv == (ushort)(ushort)46049);
                Debug.Assert(pack.alt == (int)565273369);
                Debug.Assert(pack.fix_type == (byte)(byte)156);
                Debug.Assert(pack.vn == (short)(short) -24003);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.vd = (short)(short)11188;
            p113.vn = (short)(short) -24003;
            p113.time_usec = (ulong)4180083650127133658L;
            p113.eph = (ushort)(ushort)59878;
            p113.lat = (int)243304329;
            p113.cog = (ushort)(ushort)54525;
            p113.alt = (int)565273369;
            p113.vel = (ushort)(ushort)40145;
            p113.ve = (short)(short)31558;
            p113.lon = (int)1092189279;
            p113.fix_type = (byte)(byte)156;
            p113.satellites_visible = (byte)(byte)100;
            p113.epv = (ushort)(ushort)46049;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_ygyro == (float)1.3618426E38F);
                Debug.Assert(pack.integrated_xgyro == (float)1.0278287E38F);
                Debug.Assert(pack.integrated_y == (float) -2.695477E38F);
                Debug.Assert(pack.quality == (byte)(byte)151);
                Debug.Assert(pack.integrated_zgyro == (float) -2.8070565E38F);
                Debug.Assert(pack.distance == (float)2.4997601E38F);
                Debug.Assert(pack.temperature == (short)(short) -6733);
                Debug.Assert(pack.time_delta_distance_us == (uint)1105911349U);
                Debug.Assert(pack.integrated_x == (float) -3.359304E38F);
                Debug.Assert(pack.integration_time_us == (uint)4092730742U);
                Debug.Assert(pack.sensor_id == (byte)(byte)252);
                Debug.Assert(pack.time_usec == (ulong)8587019912637257506L);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)8587019912637257506L;
            p114.time_delta_distance_us = (uint)1105911349U;
            p114.integration_time_us = (uint)4092730742U;
            p114.sensor_id = (byte)(byte)252;
            p114.distance = (float)2.4997601E38F;
            p114.quality = (byte)(byte)151;
            p114.integrated_x = (float) -3.359304E38F;
            p114.integrated_xgyro = (float)1.0278287E38F;
            p114.integrated_zgyro = (float) -2.8070565E38F;
            p114.integrated_y = (float) -2.695477E38F;
            p114.temperature = (short)(short) -6733;
            p114.integrated_ygyro = (float)1.3618426E38F;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (short)(short) -4021);
                Debug.Assert(pack.vz == (short)(short)29751);
                Debug.Assert(pack.time_usec == (ulong)4358359060199786886L);
                Debug.Assert(pack.rollspeed == (float)1.4194989E38F);
                Debug.Assert(pack.yawspeed == (float)2.3792838E37F);
                Debug.Assert(pack.yacc == (short)(short)2440);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)10352);
                Debug.Assert(pack.lat == (int) -504124152);
                Debug.Assert(pack.lon == (int)2013941430);
                Debug.Assert(pack.pitchspeed == (float) -2.0401666E37F);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)24780);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {-3.37858E38F, -1.0204306E38F, -1.2731319E38F, -9.822288E37F}));
                Debug.Assert(pack.vy == (short)(short) -25162);
                Debug.Assert(pack.vx == (short)(short) -10381);
                Debug.Assert(pack.alt == (int) -1009899779);
                Debug.Assert(pack.xacc == (short)(short) -20053);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.vz = (short)(short)29751;
            p115.time_usec = (ulong)4358359060199786886L;
            p115.yacc = (short)(short)2440;
            p115.vx = (short)(short) -10381;
            p115.yawspeed = (float)2.3792838E37F;
            p115.attitude_quaternion_SET(new float[] {-3.37858E38F, -1.0204306E38F, -1.2731319E38F, -9.822288E37F}, 0) ;
            p115.vy = (short)(short) -25162;
            p115.rollspeed = (float)1.4194989E38F;
            p115.xacc = (short)(short) -20053;
            p115.pitchspeed = (float) -2.0401666E37F;
            p115.lat = (int) -504124152;
            p115.true_airspeed = (ushort)(ushort)24780;
            p115.ind_airspeed = (ushort)(ushort)10352;
            p115.lon = (int)2013941430;
            p115.zacc = (short)(short) -4021;
            p115.alt = (int) -1009899779;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (short)(short)18805);
                Debug.Assert(pack.xacc == (short)(short) -3171);
                Debug.Assert(pack.xgyro == (short)(short)23158);
                Debug.Assert(pack.zacc == (short)(short)25084);
                Debug.Assert(pack.zgyro == (short)(short)29080);
                Debug.Assert(pack.yacc == (short)(short) -3529);
                Debug.Assert(pack.zmag == (short)(short)6417);
                Debug.Assert(pack.ygyro == (short)(short)7020);
                Debug.Assert(pack.time_boot_ms == (uint)3582041083U);
                Debug.Assert(pack.ymag == (short)(short)21372);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.zacc = (short)(short)25084;
            p116.ymag = (short)(short)21372;
            p116.zmag = (short)(short)6417;
            p116.zgyro = (short)(short)29080;
            p116.xgyro = (short)(short)23158;
            p116.time_boot_ms = (uint)3582041083U;
            p116.yacc = (short)(short) -3529;
            p116.xmag = (short)(short)18805;
            p116.xacc = (short)(short) -3171;
            p116.ygyro = (short)(short)7020;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)144);
                Debug.Assert(pack.start == (ushort)(ushort)63824);
                Debug.Assert(pack.target_component == (byte)(byte)0);
                Debug.Assert(pack.end == (ushort)(ushort)52399);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.end = (ushort)(ushort)52399;
            p117.target_system = (byte)(byte)144;
            p117.target_component = (byte)(byte)0;
            p117.start = (ushort)(ushort)63824;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_log_num == (ushort)(ushort)63702);
                Debug.Assert(pack.num_logs == (ushort)(ushort)27102);
                Debug.Assert(pack.time_utc == (uint)3468776366U);
                Debug.Assert(pack.size == (uint)1882309992U);
                Debug.Assert(pack.id == (ushort)(ushort)61911);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)61911;
            p118.last_log_num = (ushort)(ushort)63702;
            p118.size = (uint)1882309992U;
            p118.num_logs = (ushort)(ushort)27102;
            p118.time_utc = (uint)3468776366U;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)37507);
                Debug.Assert(pack.count == (uint)3054431055U);
                Debug.Assert(pack.target_system == (byte)(byte)158);
                Debug.Assert(pack.target_component == (byte)(byte)13);
                Debug.Assert(pack.ofs == (uint)1363689333U);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_component = (byte)(byte)13;
            p119.target_system = (byte)(byte)158;
            p119.id = (ushort)(ushort)37507;
            p119.count = (uint)3054431055U;
            p119.ofs = (uint)1363689333U;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)3676736048U);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)244, (byte)252, (byte)72, (byte)53, (byte)104, (byte)89, (byte)66, (byte)117, (byte)178, (byte)49, (byte)45, (byte)201, (byte)224, (byte)218, (byte)228, (byte)79, (byte)194, (byte)148, (byte)187, (byte)49, (byte)83, (byte)2, (byte)135, (byte)236, (byte)113, (byte)245, (byte)35, (byte)59, (byte)115, (byte)208, (byte)59, (byte)147, (byte)176, (byte)82, (byte)209, (byte)41, (byte)171, (byte)141, (byte)71, (byte)123, (byte)155, (byte)161, (byte)243, (byte)82, (byte)233, (byte)240, (byte)5, (byte)176, (byte)163, (byte)186, (byte)25, (byte)196, (byte)250, (byte)194, (byte)211, (byte)72, (byte)179, (byte)134, (byte)106, (byte)45, (byte)107, (byte)62, (byte)170, (byte)225, (byte)112, (byte)58, (byte)208, (byte)76, (byte)234, (byte)215, (byte)145, (byte)189, (byte)170, (byte)186, (byte)218, (byte)245, (byte)215, (byte)7, (byte)149, (byte)18, (byte)243, (byte)194, (byte)213, (byte)175, (byte)124, (byte)21, (byte)7, (byte)119, (byte)55, (byte)214}));
                Debug.Assert(pack.id == (ushort)(ushort)13487);
                Debug.Assert(pack.count == (byte)(byte)181);
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.ofs = (uint)3676736048U;
            p120.id = (ushort)(ushort)13487;
            p120.data__SET(new byte[] {(byte)244, (byte)252, (byte)72, (byte)53, (byte)104, (byte)89, (byte)66, (byte)117, (byte)178, (byte)49, (byte)45, (byte)201, (byte)224, (byte)218, (byte)228, (byte)79, (byte)194, (byte)148, (byte)187, (byte)49, (byte)83, (byte)2, (byte)135, (byte)236, (byte)113, (byte)245, (byte)35, (byte)59, (byte)115, (byte)208, (byte)59, (byte)147, (byte)176, (byte)82, (byte)209, (byte)41, (byte)171, (byte)141, (byte)71, (byte)123, (byte)155, (byte)161, (byte)243, (byte)82, (byte)233, (byte)240, (byte)5, (byte)176, (byte)163, (byte)186, (byte)25, (byte)196, (byte)250, (byte)194, (byte)211, (byte)72, (byte)179, (byte)134, (byte)106, (byte)45, (byte)107, (byte)62, (byte)170, (byte)225, (byte)112, (byte)58, (byte)208, (byte)76, (byte)234, (byte)215, (byte)145, (byte)189, (byte)170, (byte)186, (byte)218, (byte)245, (byte)215, (byte)7, (byte)149, (byte)18, (byte)243, (byte)194, (byte)213, (byte)175, (byte)124, (byte)21, (byte)7, (byte)119, (byte)55, (byte)214}, 0) ;
            p120.count = (byte)(byte)181;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)222);
                Debug.Assert(pack.target_component == (byte)(byte)179);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_component = (byte)(byte)179;
            p121.target_system = (byte)(byte)222;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)7);
                Debug.Assert(pack.target_system == (byte)(byte)241);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)241;
            p122.target_component = (byte)(byte)7;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)78);
                Debug.Assert(pack.len == (byte)(byte)29);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)172, (byte)9, (byte)159, (byte)21, (byte)147, (byte)102, (byte)135, (byte)152, (byte)225, (byte)228, (byte)136, (byte)152, (byte)66, (byte)21, (byte)20, (byte)219, (byte)80, (byte)30, (byte)163, (byte)47, (byte)157, (byte)233, (byte)243, (byte)174, (byte)85, (byte)105, (byte)222, (byte)218, (byte)171, (byte)143, (byte)52, (byte)41, (byte)207, (byte)136, (byte)72, (byte)185, (byte)48, (byte)56, (byte)78, (byte)65, (byte)30, (byte)111, (byte)218, (byte)80, (byte)117, (byte)71, (byte)12, (byte)227, (byte)16, (byte)15, (byte)104, (byte)59, (byte)28, (byte)197, (byte)23, (byte)214, (byte)2, (byte)7, (byte)159, (byte)50, (byte)17, (byte)183, (byte)106, (byte)164, (byte)14, (byte)182, (byte)181, (byte)82, (byte)137, (byte)98, (byte)216, (byte)190, (byte)127, (byte)32, (byte)14, (byte)4, (byte)109, (byte)251, (byte)163, (byte)213, (byte)191, (byte)165, (byte)30, (byte)179, (byte)235, (byte)225, (byte)219, (byte)21, (byte)70, (byte)45, (byte)26, (byte)156, (byte)131, (byte)77, (byte)46, (byte)82, (byte)200, (byte)206, (byte)135, (byte)174, (byte)102, (byte)111, (byte)214, (byte)170, (byte)170, (byte)212, (byte)211, (byte)251, (byte)229, (byte)204}));
                Debug.Assert(pack.target_component == (byte)(byte)29);
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.data__SET(new byte[] {(byte)172, (byte)9, (byte)159, (byte)21, (byte)147, (byte)102, (byte)135, (byte)152, (byte)225, (byte)228, (byte)136, (byte)152, (byte)66, (byte)21, (byte)20, (byte)219, (byte)80, (byte)30, (byte)163, (byte)47, (byte)157, (byte)233, (byte)243, (byte)174, (byte)85, (byte)105, (byte)222, (byte)218, (byte)171, (byte)143, (byte)52, (byte)41, (byte)207, (byte)136, (byte)72, (byte)185, (byte)48, (byte)56, (byte)78, (byte)65, (byte)30, (byte)111, (byte)218, (byte)80, (byte)117, (byte)71, (byte)12, (byte)227, (byte)16, (byte)15, (byte)104, (byte)59, (byte)28, (byte)197, (byte)23, (byte)214, (byte)2, (byte)7, (byte)159, (byte)50, (byte)17, (byte)183, (byte)106, (byte)164, (byte)14, (byte)182, (byte)181, (byte)82, (byte)137, (byte)98, (byte)216, (byte)190, (byte)127, (byte)32, (byte)14, (byte)4, (byte)109, (byte)251, (byte)163, (byte)213, (byte)191, (byte)165, (byte)30, (byte)179, (byte)235, (byte)225, (byte)219, (byte)21, (byte)70, (byte)45, (byte)26, (byte)156, (byte)131, (byte)77, (byte)46, (byte)82, (byte)200, (byte)206, (byte)135, (byte)174, (byte)102, (byte)111, (byte)214, (byte)170, (byte)170, (byte)212, (byte)211, (byte)251, (byte)229, (byte)204}, 0) ;
            p123.len = (byte)(byte)29;
            p123.target_system = (byte)(byte)78;
            p123.target_component = (byte)(byte)29;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.dgps_age == (uint)3308848179U);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
                Debug.Assert(pack.eph == (ushort)(ushort)31460);
                Debug.Assert(pack.lat == (int)946363498);
                Debug.Assert(pack.satellites_visible == (byte)(byte)194);
                Debug.Assert(pack.alt == (int)1681524608);
                Debug.Assert(pack.epv == (ushort)(ushort)2249);
                Debug.Assert(pack.vel == (ushort)(ushort)52691);
                Debug.Assert(pack.lon == (int) -1763065111);
                Debug.Assert(pack.time_usec == (ulong)2238924857055865569L);
                Debug.Assert(pack.dgps_numch == (byte)(byte)114);
                Debug.Assert(pack.cog == (ushort)(ushort)45505);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)2238924857055865569L;
            p124.eph = (ushort)(ushort)31460;
            p124.cog = (ushort)(ushort)45505;
            p124.dgps_numch = (byte)(byte)114;
            p124.vel = (ushort)(ushort)52691;
            p124.alt = (int)1681524608;
            p124.lon = (int) -1763065111;
            p124.dgps_age = (uint)3308848179U;
            p124.epv = (ushort)(ushort)2249;
            p124.lat = (int)946363498;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT;
            p124.satellites_visible = (byte)(byte)194;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vcc == (ushort)(ushort)26468);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT));
                Debug.Assert(pack.Vservo == (ushort)(ushort)47475);
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)26468;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT);
            p125.Vservo = (ushort)(ushort)47475;
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.timeout == (ushort)(ushort)4903);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING));
                Debug.Assert(pack.device == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
                Debug.Assert(pack.baudrate == (uint)3413551684U);
                Debug.Assert(pack.count == (byte)(byte)238);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)240, (byte)66, (byte)106, (byte)190, (byte)204, (byte)9, (byte)151, (byte)67, (byte)230, (byte)23, (byte)75, (byte)249, (byte)156, (byte)151, (byte)211, (byte)76, (byte)213, (byte)199, (byte)98, (byte)145, (byte)22, (byte)200, (byte)89, (byte)112, (byte)38, (byte)126, (byte)245, (byte)84, (byte)0, (byte)184, (byte)124, (byte)11, (byte)148, (byte)87, (byte)159, (byte)41, (byte)254, (byte)113, (byte)108, (byte)113, (byte)63, (byte)231, (byte)225, (byte)46, (byte)56, (byte)44, (byte)180, (byte)150, (byte)122, (byte)205, (byte)60, (byte)166, (byte)218, (byte)219, (byte)84, (byte)165, (byte)176, (byte)76, (byte)65, (byte)157, (byte)35, (byte)156, (byte)116, (byte)82, (byte)112, (byte)243, (byte)126, (byte)250, (byte)118, (byte)159}));
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING);
            p126.baudrate = (uint)3413551684U;
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2;
            p126.count = (byte)(byte)238;
            p126.timeout = (ushort)(ushort)4903;
            p126.data__SET(new byte[] {(byte)240, (byte)66, (byte)106, (byte)190, (byte)204, (byte)9, (byte)151, (byte)67, (byte)230, (byte)23, (byte)75, (byte)249, (byte)156, (byte)151, (byte)211, (byte)76, (byte)213, (byte)199, (byte)98, (byte)145, (byte)22, (byte)200, (byte)89, (byte)112, (byte)38, (byte)126, (byte)245, (byte)84, (byte)0, (byte)184, (byte)124, (byte)11, (byte)148, (byte)87, (byte)159, (byte)41, (byte)254, (byte)113, (byte)108, (byte)113, (byte)63, (byte)231, (byte)225, (byte)46, (byte)56, (byte)44, (byte)180, (byte)150, (byte)122, (byte)205, (byte)60, (byte)166, (byte)218, (byte)219, (byte)84, (byte)165, (byte)176, (byte)76, (byte)65, (byte)157, (byte)35, (byte)156, (byte)116, (byte)82, (byte)112, (byte)243, (byte)126, (byte)250, (byte)118, (byte)159}, 0) ;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rtk_rate == (byte)(byte)11);
                Debug.Assert(pack.baseline_b_mm == (int) -442546333);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)174);
                Debug.Assert(pack.nsats == (byte)(byte)61);
                Debug.Assert(pack.accuracy == (uint)2853809498U);
                Debug.Assert(pack.time_last_baseline_ms == (uint)1617846196U);
                Debug.Assert(pack.wn == (ushort)(ushort)18084);
                Debug.Assert(pack.tow == (uint)1409510573U);
                Debug.Assert(pack.rtk_health == (byte)(byte)100);
                Debug.Assert(pack.baseline_a_mm == (int) -1035411673);
                Debug.Assert(pack.iar_num_hypotheses == (int)868477734);
                Debug.Assert(pack.baseline_c_mm == (int)2078550280);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)191);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.baseline_b_mm = (int) -442546333;
            p127.iar_num_hypotheses = (int)868477734;
            p127.baseline_c_mm = (int)2078550280;
            p127.nsats = (byte)(byte)61;
            p127.rtk_receiver_id = (byte)(byte)191;
            p127.time_last_baseline_ms = (uint)1617846196U;
            p127.baseline_coords_type = (byte)(byte)174;
            p127.rtk_rate = (byte)(byte)11;
            p127.rtk_health = (byte)(byte)100;
            p127.tow = (uint)1409510573U;
            p127.wn = (ushort)(ushort)18084;
            p127.accuracy = (uint)2853809498U;
            p127.baseline_a_mm = (int) -1035411673;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rtk_health == (byte)(byte)189);
                Debug.Assert(pack.baseline_b_mm == (int)205686576);
                Debug.Assert(pack.time_last_baseline_ms == (uint)2404733214U);
                Debug.Assert(pack.rtk_rate == (byte)(byte)48);
                Debug.Assert(pack.iar_num_hypotheses == (int) -409513554);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)166);
                Debug.Assert(pack.tow == (uint)3115664653U);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)12);
                Debug.Assert(pack.nsats == (byte)(byte)188);
                Debug.Assert(pack.baseline_c_mm == (int) -285595624);
                Debug.Assert(pack.baseline_a_mm == (int) -1312882397);
                Debug.Assert(pack.wn == (ushort)(ushort)34528);
                Debug.Assert(pack.accuracy == (uint)874939467U);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.rtk_rate = (byte)(byte)48;
            p128.baseline_coords_type = (byte)(byte)12;
            p128.accuracy = (uint)874939467U;
            p128.rtk_receiver_id = (byte)(byte)166;
            p128.tow = (uint)3115664653U;
            p128.baseline_b_mm = (int)205686576;
            p128.nsats = (byte)(byte)188;
            p128.baseline_a_mm = (int) -1312882397;
            p128.rtk_health = (byte)(byte)189;
            p128.baseline_c_mm = (int) -285595624;
            p128.wn = (ushort)(ushort)34528;
            p128.time_last_baseline_ms = (uint)2404733214U;
            p128.iar_num_hypotheses = (int) -409513554;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)201149105U);
                Debug.Assert(pack.xgyro == (short)(short) -25662);
                Debug.Assert(pack.zacc == (short)(short)18339);
                Debug.Assert(pack.ygyro == (short)(short) -22184);
                Debug.Assert(pack.yacc == (short)(short) -9922);
                Debug.Assert(pack.xacc == (short)(short)11553);
                Debug.Assert(pack.xmag == (short)(short) -14977);
                Debug.Assert(pack.ymag == (short)(short) -13653);
                Debug.Assert(pack.zmag == (short)(short) -27708);
                Debug.Assert(pack.zgyro == (short)(short) -7376);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.xacc = (short)(short)11553;
            p129.zacc = (short)(short)18339;
            p129.yacc = (short)(short) -9922;
            p129.time_boot_ms = (uint)201149105U;
            p129.zmag = (short)(short) -27708;
            p129.xmag = (short)(short) -14977;
            p129.xgyro = (short)(short) -25662;
            p129.ymag = (short)(short) -13653;
            p129.ygyro = (short)(short) -22184;
            p129.zgyro = (short)(short) -7376;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.width == (ushort)(ushort)14065);
                Debug.Assert(pack.jpg_quality == (byte)(byte)197);
                Debug.Assert(pack.type == (byte)(byte)204);
                Debug.Assert(pack.height == (ushort)(ushort)14179);
                Debug.Assert(pack.packets == (ushort)(ushort)39136);
                Debug.Assert(pack.size == (uint)4002021047U);
                Debug.Assert(pack.payload == (byte)(byte)22);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)204;
            p130.payload = (byte)(byte)22;
            p130.packets = (ushort)(ushort)39136;
            p130.jpg_quality = (byte)(byte)197;
            p130.size = (uint)4002021047U;
            p130.height = (ushort)(ushort)14179;
            p130.width = (ushort)(ushort)14065;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seqnr == (ushort)(ushort)31644);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)122, (byte)163, (byte)121, (byte)60, (byte)3, (byte)238, (byte)61, (byte)67, (byte)204, (byte)204, (byte)189, (byte)36, (byte)112, (byte)72, (byte)96, (byte)49, (byte)222, (byte)75, (byte)35, (byte)48, (byte)150, (byte)82, (byte)91, (byte)107, (byte)192, (byte)104, (byte)98, (byte)229, (byte)212, (byte)131, (byte)150, (byte)253, (byte)242, (byte)161, (byte)36, (byte)183, (byte)244, (byte)135, (byte)180, (byte)64, (byte)15, (byte)150, (byte)32, (byte)57, (byte)80, (byte)172, (byte)138, (byte)96, (byte)94, (byte)122, (byte)149, (byte)56, (byte)201, (byte)198, (byte)160, (byte)205, (byte)36, (byte)17, (byte)45, (byte)113, (byte)214, (byte)17, (byte)10, (byte)193, (byte)78, (byte)191, (byte)46, (byte)79, (byte)91, (byte)6, (byte)121, (byte)227, (byte)211, (byte)70, (byte)119, (byte)60, (byte)43, (byte)24, (byte)23, (byte)90, (byte)75, (byte)108, (byte)252, (byte)1, (byte)89, (byte)149, (byte)170, (byte)234, (byte)201, (byte)223, (byte)227, (byte)31, (byte)199, (byte)23, (byte)57, (byte)104, (byte)60, (byte)100, (byte)164, (byte)228, (byte)119, (byte)3, (byte)249, (byte)47, (byte)164, (byte)202, (byte)86, (byte)103, (byte)12, (byte)125, (byte)24, (byte)47, (byte)176, (byte)71, (byte)205, (byte)0, (byte)118, (byte)62, (byte)200, (byte)186, (byte)213, (byte)110, (byte)102, (byte)138, (byte)114, (byte)76, (byte)58, (byte)52, (byte)168, (byte)51, (byte)43, (byte)28, (byte)223, (byte)236, (byte)115, (byte)110, (byte)61, (byte)229, (byte)162, (byte)207, (byte)72, (byte)233, (byte)51, (byte)163, (byte)75, (byte)128, (byte)134, (byte)213, (byte)146, (byte)118, (byte)142, (byte)97, (byte)160, (byte)208, (byte)189, (byte)155, (byte)226, (byte)8, (byte)115, (byte)174, (byte)103, (byte)161, (byte)200, (byte)183, (byte)45, (byte)144, (byte)242, (byte)110, (byte)35, (byte)152, (byte)212, (byte)91, (byte)182, (byte)26, (byte)73, (byte)115, (byte)216, (byte)61, (byte)103, (byte)169, (byte)133, (byte)44, (byte)69, (byte)133, (byte)114, (byte)87, (byte)143, (byte)231, (byte)16, (byte)240, (byte)101, (byte)201, (byte)197, (byte)234, (byte)209, (byte)39, (byte)87, (byte)173, (byte)103, (byte)87, (byte)162, (byte)10, (byte)92, (byte)241, (byte)98, (byte)69, (byte)12, (byte)130, (byte)90, (byte)222, (byte)143, (byte)31, (byte)148, (byte)125, (byte)203, (byte)80, (byte)78, (byte)150, (byte)218, (byte)133, (byte)157, (byte)157, (byte)121, (byte)222, (byte)165, (byte)176, (byte)245, (byte)204, (byte)80, (byte)201, (byte)167, (byte)97, (byte)93, (byte)109, (byte)96, (byte)244, (byte)73, (byte)88, (byte)104, (byte)192, (byte)212, (byte)177, (byte)9, (byte)170, (byte)46, (byte)159, (byte)195, (byte)151, (byte)251, (byte)71, (byte)67, (byte)176, (byte)117}));
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.data__SET(new byte[] {(byte)122, (byte)163, (byte)121, (byte)60, (byte)3, (byte)238, (byte)61, (byte)67, (byte)204, (byte)204, (byte)189, (byte)36, (byte)112, (byte)72, (byte)96, (byte)49, (byte)222, (byte)75, (byte)35, (byte)48, (byte)150, (byte)82, (byte)91, (byte)107, (byte)192, (byte)104, (byte)98, (byte)229, (byte)212, (byte)131, (byte)150, (byte)253, (byte)242, (byte)161, (byte)36, (byte)183, (byte)244, (byte)135, (byte)180, (byte)64, (byte)15, (byte)150, (byte)32, (byte)57, (byte)80, (byte)172, (byte)138, (byte)96, (byte)94, (byte)122, (byte)149, (byte)56, (byte)201, (byte)198, (byte)160, (byte)205, (byte)36, (byte)17, (byte)45, (byte)113, (byte)214, (byte)17, (byte)10, (byte)193, (byte)78, (byte)191, (byte)46, (byte)79, (byte)91, (byte)6, (byte)121, (byte)227, (byte)211, (byte)70, (byte)119, (byte)60, (byte)43, (byte)24, (byte)23, (byte)90, (byte)75, (byte)108, (byte)252, (byte)1, (byte)89, (byte)149, (byte)170, (byte)234, (byte)201, (byte)223, (byte)227, (byte)31, (byte)199, (byte)23, (byte)57, (byte)104, (byte)60, (byte)100, (byte)164, (byte)228, (byte)119, (byte)3, (byte)249, (byte)47, (byte)164, (byte)202, (byte)86, (byte)103, (byte)12, (byte)125, (byte)24, (byte)47, (byte)176, (byte)71, (byte)205, (byte)0, (byte)118, (byte)62, (byte)200, (byte)186, (byte)213, (byte)110, (byte)102, (byte)138, (byte)114, (byte)76, (byte)58, (byte)52, (byte)168, (byte)51, (byte)43, (byte)28, (byte)223, (byte)236, (byte)115, (byte)110, (byte)61, (byte)229, (byte)162, (byte)207, (byte)72, (byte)233, (byte)51, (byte)163, (byte)75, (byte)128, (byte)134, (byte)213, (byte)146, (byte)118, (byte)142, (byte)97, (byte)160, (byte)208, (byte)189, (byte)155, (byte)226, (byte)8, (byte)115, (byte)174, (byte)103, (byte)161, (byte)200, (byte)183, (byte)45, (byte)144, (byte)242, (byte)110, (byte)35, (byte)152, (byte)212, (byte)91, (byte)182, (byte)26, (byte)73, (byte)115, (byte)216, (byte)61, (byte)103, (byte)169, (byte)133, (byte)44, (byte)69, (byte)133, (byte)114, (byte)87, (byte)143, (byte)231, (byte)16, (byte)240, (byte)101, (byte)201, (byte)197, (byte)234, (byte)209, (byte)39, (byte)87, (byte)173, (byte)103, (byte)87, (byte)162, (byte)10, (byte)92, (byte)241, (byte)98, (byte)69, (byte)12, (byte)130, (byte)90, (byte)222, (byte)143, (byte)31, (byte)148, (byte)125, (byte)203, (byte)80, (byte)78, (byte)150, (byte)218, (byte)133, (byte)157, (byte)157, (byte)121, (byte)222, (byte)165, (byte)176, (byte)245, (byte)204, (byte)80, (byte)201, (byte)167, (byte)97, (byte)93, (byte)109, (byte)96, (byte)244, (byte)73, (byte)88, (byte)104, (byte)192, (byte)212, (byte)177, (byte)9, (byte)170, (byte)46, (byte)159, (byte)195, (byte)151, (byte)251, (byte)71, (byte)67, (byte)176, (byte)117}, 0) ;
            p131.seqnr = (ushort)(ushort)31644;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)833560302U);
                Debug.Assert(pack.id == (byte)(byte)237);
                Debug.Assert(pack.orientation == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_YAW_135);
                Debug.Assert(pack.covariance == (byte)(byte)79);
                Debug.Assert(pack.min_distance == (ushort)(ushort)63895);
                Debug.Assert(pack.current_distance == (ushort)(ushort)21519);
                Debug.Assert(pack.max_distance == (ushort)(ushort)34169);
                Debug.Assert(pack.type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.covariance = (byte)(byte)79;
            p132.max_distance = (ushort)(ushort)34169;
            p132.id = (byte)(byte)237;
            p132.current_distance = (ushort)(ushort)21519;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p132.time_boot_ms = (uint)833560302U;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_YAW_135;
            p132.min_distance = (ushort)(ushort)63895;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -607129897);
                Debug.Assert(pack.lon == (int)1240461832);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)21819);
                Debug.Assert(pack.mask == (ulong)7070616437446618047L);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.mask = (ulong)7070616437446618047L;
            p133.lat = (int) -607129897;
            p133.grid_spacing = (ushort)(ushort)21819;
            p133.lon = (int)1240461832;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short)8856, (short) -20742, (short) -18251, (short)23030, (short)18085, (short)31326, (short) -11164, (short)21400, (short) -12403, (short) -19049, (short)12922, (short) -9827, (short) -3757, (short) -31229, (short) -29464, (short)13463}));
                Debug.Assert(pack.gridbit == (byte)(byte)65);
                Debug.Assert(pack.lon == (int) -1270136673);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)38737);
                Debug.Assert(pack.lat == (int)1448288689);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int)1448288689;
            p134.gridbit = (byte)(byte)65;
            p134.data__SET(new short[] {(short)8856, (short) -20742, (short) -18251, (short)23030, (short)18085, (short)31326, (short) -11164, (short)21400, (short) -12403, (short) -19049, (short)12922, (short) -9827, (short) -3757, (short) -31229, (short) -29464, (short)13463}, 0) ;
            p134.grid_spacing = (ushort)(ushort)38737;
            p134.lon = (int) -1270136673;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -1637960360);
                Debug.Assert(pack.lat == (int)116332431);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)116332431;
            p135.lon = (int) -1637960360;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pending == (ushort)(ushort)25023);
                Debug.Assert(pack.loaded == (ushort)(ushort)15158);
                Debug.Assert(pack.lon == (int)1310575859);
                Debug.Assert(pack.lat == (int)1377491167);
                Debug.Assert(pack.current_height == (float)1.2652686E38F);
                Debug.Assert(pack.terrain_height == (float)2.1038315E38F);
                Debug.Assert(pack.spacing == (ushort)(ushort)42988);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lon = (int)1310575859;
            p136.lat = (int)1377491167;
            p136.terrain_height = (float)2.1038315E38F;
            p136.spacing = (ushort)(ushort)42988;
            p136.current_height = (float)1.2652686E38F;
            p136.loaded = (ushort)(ushort)15158;
            p136.pending = (ushort)(ushort)25023;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -17065);
                Debug.Assert(pack.press_diff == (float)2.3464202E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1235841374U);
                Debug.Assert(pack.press_abs == (float)2.1307097E38F);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.press_diff = (float)2.3464202E37F;
            p137.press_abs = (float)2.1307097E38F;
            p137.temperature = (short)(short) -17065;
            p137.time_boot_ms = (uint)1235841374U;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -1.880264E38F);
                Debug.Assert(pack.x == (float) -1.7112786E38F);
                Debug.Assert(pack.time_usec == (ulong)1038792588552967421L);
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.562939E37F, 1.4654947E38F, -2.7911762E38F, -1.623752E38F}));
                Debug.Assert(pack.z == (float) -8.034374E37F);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.x = (float) -1.7112786E38F;
            p138.z = (float) -8.034374E37F;
            p138.y = (float) -1.880264E38F;
            p138.q_SET(new float[] {3.562939E37F, 1.4654947E38F, -2.7911762E38F, -1.623752E38F}, 0) ;
            p138.time_usec = (ulong)1038792588552967421L;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)118);
                Debug.Assert(pack.time_usec == (ulong)5972197592624344928L);
                Debug.Assert(pack.target_component == (byte)(byte)92);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.2551838E38F, -1.7524752E38F, -1.3346344E38F, -3.0268734E38F, -7.1784473E37F, 1.3836797E38F, -1.6984403E38F, -2.1601336E38F}));
                Debug.Assert(pack.target_system == (byte)(byte)39);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)5972197592624344928L;
            p139.target_system = (byte)(byte)39;
            p139.controls_SET(new float[] {-1.2551838E38F, -1.7524752E38F, -1.3346344E38F, -3.0268734E38F, -7.1784473E37F, 1.3836797E38F, -1.6984403E38F, -2.1601336E38F}, 0) ;
            p139.group_mlx = (byte)(byte)118;
            p139.target_component = (byte)(byte)92;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)112);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-7.572448E36F, -2.4892644E38F, -3.733777E37F, 1.5963875E38F, 2.975889E38F, 2.3890048E38F, -2.3470318E38F, -6.041428E37F}));
                Debug.Assert(pack.time_usec == (ulong)960546566422315189L);
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.group_mlx = (byte)(byte)112;
            p140.time_usec = (ulong)960546566422315189L;
            p140.controls_SET(new float[] {-7.572448E36F, -2.4892644E38F, -3.733777E37F, 1.5963875E38F, 2.975889E38F, 2.3890048E38F, -2.3470318E38F, -6.041428E37F}, 0) ;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_local == (float) -2.5192189E38F);
                Debug.Assert(pack.altitude_monotonic == (float)3.3094674E38F);
                Debug.Assert(pack.time_usec == (ulong)649448912081914583L);
                Debug.Assert(pack.bottom_clearance == (float) -3.0308794E38F);
                Debug.Assert(pack.altitude_terrain == (float) -3.2601527E38F);
                Debug.Assert(pack.altitude_relative == (float)4.039964E37F);
                Debug.Assert(pack.altitude_amsl == (float)3.0080215E38F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_local = (float) -2.5192189E38F;
            p141.altitude_monotonic = (float)3.3094674E38F;
            p141.bottom_clearance = (float) -3.0308794E38F;
            p141.altitude_terrain = (float) -3.2601527E38F;
            p141.time_usec = (ulong)649448912081914583L;
            p141.altitude_relative = (float)4.039964E37F;
            p141.altitude_amsl = (float)3.0080215E38F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri_type == (byte)(byte)14);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)246, (byte)238, (byte)247, (byte)205, (byte)217, (byte)143, (byte)157, (byte)21, (byte)237, (byte)151, (byte)158, (byte)119, (byte)95, (byte)23, (byte)2, (byte)133, (byte)114, (byte)48, (byte)188, (byte)99, (byte)242, (byte)42, (byte)65, (byte)22, (byte)188, (byte)100, (byte)76, (byte)126, (byte)23, (byte)50, (byte)24, (byte)12, (byte)7, (byte)115, (byte)71, (byte)61, (byte)84, (byte)160, (byte)229, (byte)60, (byte)171, (byte)57, (byte)250, (byte)9, (byte)244, (byte)177, (byte)34, (byte)12, (byte)244, (byte)8, (byte)26, (byte)85, (byte)139, (byte)224, (byte)206, (byte)53, (byte)198, (byte)254, (byte)8, (byte)170, (byte)5, (byte)22, (byte)114, (byte)55, (byte)32, (byte)166, (byte)66, (byte)149, (byte)43, (byte)221, (byte)40, (byte)186, (byte)35, (byte)114, (byte)100, (byte)60, (byte)213, (byte)69, (byte)253, (byte)99, (byte)21, (byte)39, (byte)183, (byte)77, (byte)123, (byte)16, (byte)196, (byte)150, (byte)163, (byte)83, (byte)37, (byte)147, (byte)86, (byte)87, (byte)36, (byte)152, (byte)19, (byte)90, (byte)127, (byte)172, (byte)154, (byte)96, (byte)164, (byte)67, (byte)109, (byte)36, (byte)11, (byte)201, (byte)177, (byte)147, (byte)194, (byte)209, (byte)174, (byte)100, (byte)164, (byte)255, (byte)142, (byte)84, (byte)95, (byte)83}));
                Debug.Assert(pack.transfer_type == (byte)(byte)125);
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)189, (byte)145, (byte)154, (byte)192, (byte)79, (byte)227, (byte)238, (byte)232, (byte)57, (byte)230, (byte)224, (byte)108, (byte)202, (byte)140, (byte)222, (byte)254, (byte)47, (byte)168, (byte)216, (byte)160, (byte)44, (byte)71, (byte)175, (byte)172, (byte)250, (byte)250, (byte)42, (byte)64, (byte)64, (byte)168, (byte)72, (byte)177, (byte)36, (byte)170, (byte)135, (byte)83, (byte)249, (byte)188, (byte)15, (byte)172, (byte)125, (byte)191, (byte)236, (byte)100, (byte)91, (byte)12, (byte)166, (byte)245, (byte)96, (byte)67, (byte)40, (byte)178, (byte)5, (byte)68, (byte)171, (byte)124, (byte)210, (byte)186, (byte)36, (byte)125, (byte)184, (byte)65, (byte)154, (byte)97, (byte)51, (byte)7, (byte)3, (byte)249, (byte)43, (byte)91, (byte)202, (byte)159, (byte)207, (byte)49, (byte)78, (byte)92, (byte)170, (byte)221, (byte)96, (byte)191, (byte)201, (byte)110, (byte)216, (byte)50, (byte)15, (byte)151, (byte)210, (byte)107, (byte)175, (byte)58, (byte)154, (byte)160, (byte)77, (byte)211, (byte)107, (byte)100, (byte)224, (byte)61, (byte)34, (byte)158, (byte)59, (byte)124, (byte)219, (byte)171, (byte)26, (byte)55, (byte)106, (byte)173, (byte)156, (byte)145, (byte)234, (byte)148, (byte)234, (byte)10, (byte)60, (byte)3, (byte)93, (byte)126, (byte)242, (byte)251}));
                Debug.Assert(pack.request_id == (byte)(byte)184);
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.transfer_type = (byte)(byte)125;
            p142.request_id = (byte)(byte)184;
            p142.storage_SET(new byte[] {(byte)189, (byte)145, (byte)154, (byte)192, (byte)79, (byte)227, (byte)238, (byte)232, (byte)57, (byte)230, (byte)224, (byte)108, (byte)202, (byte)140, (byte)222, (byte)254, (byte)47, (byte)168, (byte)216, (byte)160, (byte)44, (byte)71, (byte)175, (byte)172, (byte)250, (byte)250, (byte)42, (byte)64, (byte)64, (byte)168, (byte)72, (byte)177, (byte)36, (byte)170, (byte)135, (byte)83, (byte)249, (byte)188, (byte)15, (byte)172, (byte)125, (byte)191, (byte)236, (byte)100, (byte)91, (byte)12, (byte)166, (byte)245, (byte)96, (byte)67, (byte)40, (byte)178, (byte)5, (byte)68, (byte)171, (byte)124, (byte)210, (byte)186, (byte)36, (byte)125, (byte)184, (byte)65, (byte)154, (byte)97, (byte)51, (byte)7, (byte)3, (byte)249, (byte)43, (byte)91, (byte)202, (byte)159, (byte)207, (byte)49, (byte)78, (byte)92, (byte)170, (byte)221, (byte)96, (byte)191, (byte)201, (byte)110, (byte)216, (byte)50, (byte)15, (byte)151, (byte)210, (byte)107, (byte)175, (byte)58, (byte)154, (byte)160, (byte)77, (byte)211, (byte)107, (byte)100, (byte)224, (byte)61, (byte)34, (byte)158, (byte)59, (byte)124, (byte)219, (byte)171, (byte)26, (byte)55, (byte)106, (byte)173, (byte)156, (byte)145, (byte)234, (byte)148, (byte)234, (byte)10, (byte)60, (byte)3, (byte)93, (byte)126, (byte)242, (byte)251}, 0) ;
            p142.uri_SET(new byte[] {(byte)246, (byte)238, (byte)247, (byte)205, (byte)217, (byte)143, (byte)157, (byte)21, (byte)237, (byte)151, (byte)158, (byte)119, (byte)95, (byte)23, (byte)2, (byte)133, (byte)114, (byte)48, (byte)188, (byte)99, (byte)242, (byte)42, (byte)65, (byte)22, (byte)188, (byte)100, (byte)76, (byte)126, (byte)23, (byte)50, (byte)24, (byte)12, (byte)7, (byte)115, (byte)71, (byte)61, (byte)84, (byte)160, (byte)229, (byte)60, (byte)171, (byte)57, (byte)250, (byte)9, (byte)244, (byte)177, (byte)34, (byte)12, (byte)244, (byte)8, (byte)26, (byte)85, (byte)139, (byte)224, (byte)206, (byte)53, (byte)198, (byte)254, (byte)8, (byte)170, (byte)5, (byte)22, (byte)114, (byte)55, (byte)32, (byte)166, (byte)66, (byte)149, (byte)43, (byte)221, (byte)40, (byte)186, (byte)35, (byte)114, (byte)100, (byte)60, (byte)213, (byte)69, (byte)253, (byte)99, (byte)21, (byte)39, (byte)183, (byte)77, (byte)123, (byte)16, (byte)196, (byte)150, (byte)163, (byte)83, (byte)37, (byte)147, (byte)86, (byte)87, (byte)36, (byte)152, (byte)19, (byte)90, (byte)127, (byte)172, (byte)154, (byte)96, (byte)164, (byte)67, (byte)109, (byte)36, (byte)11, (byte)201, (byte)177, (byte)147, (byte)194, (byte)209, (byte)174, (byte)100, (byte)164, (byte)255, (byte)142, (byte)84, (byte)95, (byte)83}, 0) ;
            p142.uri_type = (byte)(byte)14;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)26615);
                Debug.Assert(pack.time_boot_ms == (uint)825303524U);
                Debug.Assert(pack.press_diff == (float) -7.8710094E37F);
                Debug.Assert(pack.press_abs == (float) -1.7439655E37F);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.temperature = (short)(short)26615;
            p143.press_abs = (float) -1.7439655E37F;
            p143.time_boot_ms = (uint)825303524U;
            p143.press_diff = (float) -7.8710094E37F;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rates.SequenceEqual(new float[] {2.6432117E38F, -3.116346E37F, -3.0770198E38F}));
                Debug.Assert(pack.timestamp == (ulong)305516155697094969L);
                Debug.Assert(pack.custom_state == (ulong)6895105612637917212L);
                Debug.Assert(pack.lat == (int)1392122627);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {-1.0486999E38F, -2.1438961E38F, 6.091394E37F}));
                Debug.Assert(pack.est_capabilities == (byte)(byte)70);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-2.2929086E38F, -1.0978313E38F, 7.8904116E37F, 1.0158515E38F}));
                Debug.Assert(pack.lon == (int)632164881);
                Debug.Assert(pack.alt == (float) -8.420493E37F);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-4.0210055E37F, 3.2700676E38F, -2.525008E38F}));
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {1.1975729E38F, 3.2568084E38F, -3.0807226E38F}));
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.est_capabilities = (byte)(byte)70;
            p144.rates_SET(new float[] {2.6432117E38F, -3.116346E37F, -3.0770198E38F}, 0) ;
            p144.vel_SET(new float[] {-4.0210055E37F, 3.2700676E38F, -2.525008E38F}, 0) ;
            p144.lat = (int)1392122627;
            p144.acc_SET(new float[] {-1.0486999E38F, -2.1438961E38F, 6.091394E37F}, 0) ;
            p144.lon = (int)632164881;
            p144.alt = (float) -8.420493E37F;
            p144.custom_state = (ulong)6895105612637917212L;
            p144.attitude_q_SET(new float[] {-2.2929086E38F, -1.0978313E38F, 7.8904116E37F, 1.0158515E38F}, 0) ;
            p144.timestamp = (ulong)305516155697094969L;
            p144.position_cov_SET(new float[] {1.1975729E38F, 3.2568084E38F, -3.0807226E38F}, 0) ;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x_vel == (float)2.7834054E38F);
                Debug.Assert(pack.x_pos == (float)1.2285815E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.089417E38F, -2.056843E38F, 8.300163E37F, 2.722973E38F}));
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {7.293369E37F, 1.1886142E38F, -2.5080664E38F}));
                Debug.Assert(pack.pitch_rate == (float) -2.5242954E38F);
                Debug.Assert(pack.roll_rate == (float)9.628324E37F);
                Debug.Assert(pack.time_usec == (ulong)6932192500148157289L);
                Debug.Assert(pack.z_acc == (float)1.9276382E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {-9.112999E37F, -1.1087241E38F, -2.7300294E38F}));
                Debug.Assert(pack.z_vel == (float)1.1273363E37F);
                Debug.Assert(pack.yaw_rate == (float)2.4162996E38F);
                Debug.Assert(pack.x_acc == (float)1.7940388E38F);
                Debug.Assert(pack.airspeed == (float)2.6071575E38F);
                Debug.Assert(pack.y_vel == (float) -5.119033E37F);
                Debug.Assert(pack.y_pos == (float)1.4368518E38F);
                Debug.Assert(pack.z_pos == (float) -1.704202E38F);
                Debug.Assert(pack.y_acc == (float) -2.7127368E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.z_vel = (float)1.1273363E37F;
            p146.z_pos = (float) -1.704202E38F;
            p146.time_usec = (ulong)6932192500148157289L;
            p146.q_SET(new float[] {-3.089417E38F, -2.056843E38F, 8.300163E37F, 2.722973E38F}, 0) ;
            p146.z_acc = (float)1.9276382E38F;
            p146.y_acc = (float) -2.7127368E38F;
            p146.y_vel = (float) -5.119033E37F;
            p146.yaw_rate = (float)2.4162996E38F;
            p146.pitch_rate = (float) -2.5242954E38F;
            p146.y_pos = (float)1.4368518E38F;
            p146.airspeed = (float)2.6071575E38F;
            p146.pos_variance_SET(new float[] {7.293369E37F, 1.1886142E38F, -2.5080664E38F}, 0) ;
            p146.x_acc = (float)1.7940388E38F;
            p146.x_pos = (float)1.2285815E38F;
            p146.x_vel = (float)2.7834054E38F;
            p146.roll_rate = (float)9.628324E37F;
            p146.vel_variance_SET(new float[] {-9.112999E37F, -1.1087241E38F, -2.7300294E38F}, 0) ;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.battery_function == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION);
                Debug.Assert(pack.current_battery == (short)(short)20127);
                Debug.Assert(pack.id == (byte)(byte)102);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)41399, (ushort)6065, (ushort)26274, (ushort)15603, (ushort)64544, (ushort)58550, (ushort)10693, (ushort)28088, (ushort)41503, (ushort)14580}));
                Debug.Assert(pack.current_consumed == (int) -1721603709);
                Debug.Assert(pack.energy_consumed == (int) -2008214111);
                Debug.Assert(pack.temperature == (short)(short)30724);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 30);
                Debug.Assert(pack.type == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.current_consumed = (int) -1721603709;
            p147.battery_remaining = (sbyte)(sbyte) - 30;
            p147.id = (byte)(byte)102;
            p147.temperature = (short)(short)30724;
            p147.voltages_SET(new ushort[] {(ushort)41399, (ushort)6065, (ushort)26274, (ushort)15603, (ushort)64544, (ushort)58550, (ushort)10693, (ushort)28088, (ushort)41503, (ushort)14580}, 0) ;
            p147.current_battery = (short)(short)20127;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH;
            p147.energy_consumed = (int) -2008214111;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.board_version == (uint)3176316180U);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)72, (byte)202, (byte)164, (byte)192, (byte)72, (byte)160, (byte)229, (byte)142}));
                Debug.Assert(pack.os_sw_version == (uint)3158239508U);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)59566);
                Debug.Assert(pack.middleware_sw_version == (uint)3733040676U);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)203, (byte)98, (byte)98, (byte)128, (byte)52, (byte)77, (byte)3, (byte)199}));
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE));
                Debug.Assert(pack.flight_sw_version == (uint)2642709413U);
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)196, (byte)202, (byte)163, (byte)212, (byte)244, (byte)221, (byte)89, (byte)186}));
                Debug.Assert(pack.product_id == (ushort)(ushort)9867);
                Debug.Assert(pack.uid == (ulong)1553030996283200424L);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)8, (byte)1, (byte)133, (byte)213, (byte)131, (byte)241, (byte)99, (byte)88, (byte)14, (byte)111, (byte)189, (byte)88, (byte)241, (byte)62, (byte)79, (byte)232, (byte)76, (byte)187}));
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.flight_custom_version_SET(new byte[] {(byte)203, (byte)98, (byte)98, (byte)128, (byte)52, (byte)77, (byte)3, (byte)199}, 0) ;
            p148.middleware_custom_version_SET(new byte[] {(byte)72, (byte)202, (byte)164, (byte)192, (byte)72, (byte)160, (byte)229, (byte)142}, 0) ;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE);
            p148.middleware_sw_version = (uint)3733040676U;
            p148.uid = (ulong)1553030996283200424L;
            p148.board_version = (uint)3176316180U;
            p148.os_custom_version_SET(new byte[] {(byte)196, (byte)202, (byte)163, (byte)212, (byte)244, (byte)221, (byte)89, (byte)186}, 0) ;
            p148.os_sw_version = (uint)3158239508U;
            p148.flight_sw_version = (uint)2642709413U;
            p148.vendor_id = (ushort)(ushort)59566;
            p148.product_id = (ushort)(ushort)9867;
            p148.uid2_SET(new byte[] {(byte)8, (byte)1, (byte)133, (byte)213, (byte)131, (byte)241, (byte)99, (byte)88, (byte)14, (byte)111, (byte)189, (byte)88, (byte)241, (byte)62, (byte)79, (byte)232, (byte)76, (byte)187}, 0, PH) ;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z_TRY(ph) == (float)1.4647317E38F);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {1.1109632E38F, -3.3548495E37F, -1.4981052E38F, 2.3279858E38F}));
                Debug.Assert(pack.time_usec == (ulong)443057008729414159L);
                Debug.Assert(pack.y_TRY(ph) == (float) -2.5753395E38F);
                Debug.Assert(pack.x_TRY(ph) == (float) -2.6914908E38F);
                Debug.Assert(pack.target_num == (byte)(byte)104);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.angle_y == (float)2.6755776E37F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)205);
                Debug.Assert(pack.distance == (float)1.9949186E37F);
                Debug.Assert(pack.size_y == (float)5.7299496E37F);
                Debug.Assert(pack.angle_x == (float)2.7465526E38F);
                Debug.Assert(pack.size_x == (float) -7.1376036E37F);
                Debug.Assert(pack.type == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.z_SET((float)1.4647317E38F, PH) ;
            p149.size_x = (float) -7.1376036E37F;
            p149.target_num = (byte)(byte)104;
            p149.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p149.angle_y = (float)2.6755776E37F;
            p149.size_y = (float)5.7299496E37F;
            p149.y_SET((float) -2.5753395E38F, PH) ;
            p149.time_usec = (ulong)443057008729414159L;
            p149.distance = (float)1.9949186E37F;
            p149.position_valid_SET((byte)(byte)205, PH) ;
            p149.x_SET((float) -2.6914908E38F, PH) ;
            p149.q_SET(new float[] {1.1109632E38F, -3.3548495E37F, -1.4981052E38F, 2.3279858E38F}, 0, PH) ;
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON;
            p149.angle_x = (float)2.7465526E38F;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAV_FILTER_BIASReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.accel_2 == (float) -3.0380415E38F);
                Debug.Assert(pack.accel_1 == (float)1.0010084E37F);
                Debug.Assert(pack.accel_0 == (float)2.781946E38F);
                Debug.Assert(pack.usec == (ulong)3581133942877618657L);
                Debug.Assert(pack.gyro_0 == (float)2.3167325E38F);
                Debug.Assert(pack.gyro_1 == (float) -1.0565687E38F);
                Debug.Assert(pack.gyro_2 == (float)2.6682646E38F);
            };
            GroundControl.NAV_FILTER_BIAS p220 = CommunicationChannel.new_NAV_FILTER_BIAS();
            PH.setPack(p220);
            p220.gyro_0 = (float)2.3167325E38F;
            p220.usec = (ulong)3581133942877618657L;
            p220.accel_1 = (float)1.0010084E37F;
            p220.gyro_1 = (float) -1.0565687E38F;
            p220.gyro_2 = (float)2.6682646E38F;
            p220.accel_0 = (float)2.781946E38F;
            p220.accel_2 = (float) -3.0380415E38F;
            CommunicationChannel.instance.send(p220);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRADIO_CALIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.elevator.SequenceEqual(new ushort[] {(ushort)31013, (ushort)63727, (ushort)58931}));
                Debug.Assert(pack.gyro.SequenceEqual(new ushort[] {(ushort)55366, (ushort)56346}));
                Debug.Assert(pack.throttle.SequenceEqual(new ushort[] {(ushort)42356, (ushort)58, (ushort)40001, (ushort)27787, (ushort)32876}));
                Debug.Assert(pack.aileron.SequenceEqual(new ushort[] {(ushort)41497, (ushort)37898, (ushort)63407}));
                Debug.Assert(pack.pitch.SequenceEqual(new ushort[] {(ushort)41764, (ushort)18779, (ushort)37000, (ushort)54664, (ushort)8598}));
                Debug.Assert(pack.rudder.SequenceEqual(new ushort[] {(ushort)26021, (ushort)1225, (ushort)53982}));
            };
            GroundControl.RADIO_CALIBRATION p221 = CommunicationChannel.new_RADIO_CALIBRATION();
            PH.setPack(p221);
            p221.aileron_SET(new ushort[] {(ushort)41497, (ushort)37898, (ushort)63407}, 0) ;
            p221.throttle_SET(new ushort[] {(ushort)42356, (ushort)58, (ushort)40001, (ushort)27787, (ushort)32876}, 0) ;
            p221.pitch_SET(new ushort[] {(ushort)41764, (ushort)18779, (ushort)37000, (ushort)54664, (ushort)8598}, 0) ;
            p221.rudder_SET(new ushort[] {(ushort)26021, (ushort)1225, (ushort)53982}, 0) ;
            p221.elevator_SET(new ushort[] {(ushort)31013, (ushort)63727, (ushort)58931}, 0) ;
            p221.gyro_SET(new ushort[] {(ushort)55366, (ushort)56346}, 0) ;
            CommunicationChannel.instance.send(p221);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUALBERTA_SYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode == (byte)(byte)46);
                Debug.Assert(pack.nav_mode == (byte)(byte)71);
                Debug.Assert(pack.pilot == (byte)(byte)60);
            };
            GroundControl.UALBERTA_SYS_STATUS p222 = CommunicationChannel.new_UALBERTA_SYS_STATUS();
            PH.setPack(p222);
            p222.mode = (byte)(byte)46;
            p222.nav_mode = (byte)(byte)71;
            p222.pilot = (byte)(byte)60;
            CommunicationChannel.instance.send(p222);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tas_ratio == (float)1.3474477E38F);
                Debug.Assert(pack.time_usec == (ulong)2770767011712506121L);
                Debug.Assert(pack.pos_vert_accuracy == (float) -1.0979997E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT));
                Debug.Assert(pack.pos_vert_ratio == (float) -4.3338494E37F);
                Debug.Assert(pack.mag_ratio == (float) -3.0110856E38F);
                Debug.Assert(pack.pos_horiz_ratio == (float) -1.07164E38F);
                Debug.Assert(pack.vel_ratio == (float) -1.1895681E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float)3.2217997E37F);
                Debug.Assert(pack.hagl_ratio == (float) -3.0098216E38F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.pos_vert_accuracy = (float) -1.0979997E38F;
            p230.vel_ratio = (float) -1.1895681E38F;
            p230.mag_ratio = (float) -3.0110856E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT);
            p230.tas_ratio = (float)1.3474477E38F;
            p230.time_usec = (ulong)2770767011712506121L;
            p230.pos_horiz_accuracy = (float)3.2217997E37F;
            p230.pos_vert_ratio = (float) -4.3338494E37F;
            p230.hagl_ratio = (float) -3.0098216E38F;
            p230.pos_horiz_ratio = (float) -1.07164E38F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.var_horiz == (float) -2.9582692E38F);
                Debug.Assert(pack.time_usec == (ulong)5859636228109094282L);
                Debug.Assert(pack.vert_accuracy == (float)3.169252E38F);
                Debug.Assert(pack.wind_x == (float) -2.0226465E38F);
                Debug.Assert(pack.wind_alt == (float) -1.7517157E38F);
                Debug.Assert(pack.var_vert == (float)3.287616E38F);
                Debug.Assert(pack.wind_z == (float) -1.9780258E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -5.5786045E34F);
                Debug.Assert(pack.wind_y == (float)2.0628784E38F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.wind_x = (float) -2.0226465E38F;
            p231.wind_y = (float)2.0628784E38F;
            p231.vert_accuracy = (float)3.169252E38F;
            p231.wind_alt = (float) -1.7517157E38F;
            p231.var_vert = (float)3.287616E38F;
            p231.time_usec = (ulong)5859636228109094282L;
            p231.wind_z = (float) -1.9780258E38F;
            p231.horiz_accuracy = (float) -5.5786045E34F;
            p231.var_horiz = (float) -2.9582692E38F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY));
                Debug.Assert(pack.speed_accuracy == (float) -4.0397497E37F);
                Debug.Assert(pack.alt == (float)9.703375E37F);
                Debug.Assert(pack.ve == (float)2.0681176E37F);
                Debug.Assert(pack.lat == (int) -887467602);
                Debug.Assert(pack.time_week_ms == (uint)3485084607U);
                Debug.Assert(pack.lon == (int)246397997);
                Debug.Assert(pack.vdop == (float) -5.07151E37F);
                Debug.Assert(pack.time_week == (ushort)(ushort)34824);
                Debug.Assert(pack.horiz_accuracy == (float) -2.1304702E38F);
                Debug.Assert(pack.vert_accuracy == (float)1.0979249E38F);
                Debug.Assert(pack.fix_type == (byte)(byte)21);
                Debug.Assert(pack.vd == (float) -5.811284E37F);
                Debug.Assert(pack.time_usec == (ulong)4958597669297906585L);
                Debug.Assert(pack.satellites_visible == (byte)(byte)251);
                Debug.Assert(pack.vn == (float) -1.751828E38F);
                Debug.Assert(pack.gps_id == (byte)(byte)151);
                Debug.Assert(pack.hdop == (float) -9.419194E37F);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.gps_id = (byte)(byte)151;
            p232.lat = (int) -887467602;
            p232.time_usec = (ulong)4958597669297906585L;
            p232.satellites_visible = (byte)(byte)251;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY);
            p232.alt = (float)9.703375E37F;
            p232.fix_type = (byte)(byte)21;
            p232.time_week = (ushort)(ushort)34824;
            p232.speed_accuracy = (float) -4.0397497E37F;
            p232.vd = (float) -5.811284E37F;
            p232.ve = (float)2.0681176E37F;
            p232.vdop = (float) -5.07151E37F;
            p232.vn = (float) -1.751828E38F;
            p232.lon = (int)246397997;
            p232.hdop = (float) -9.419194E37F;
            p232.vert_accuracy = (float)1.0979249E38F;
            p232.horiz_accuracy = (float) -2.1304702E38F;
            p232.time_week_ms = (uint)3485084607U;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)105, (byte)1, (byte)191, (byte)19, (byte)115, (byte)140, (byte)22, (byte)19, (byte)53, (byte)33, (byte)197, (byte)153, (byte)38, (byte)210, (byte)54, (byte)215, (byte)94, (byte)52, (byte)121, (byte)38, (byte)67, (byte)103, (byte)56, (byte)20, (byte)196, (byte)211, (byte)237, (byte)61, (byte)215, (byte)107, (byte)101, (byte)58, (byte)160, (byte)124, (byte)108, (byte)192, (byte)185, (byte)102, (byte)124, (byte)203, (byte)16, (byte)25, (byte)241, (byte)90, (byte)217, (byte)254, (byte)140, (byte)200, (byte)10, (byte)24, (byte)179, (byte)207, (byte)201, (byte)185, (byte)44, (byte)248, (byte)174, (byte)163, (byte)42, (byte)254, (byte)89, (byte)32, (byte)217, (byte)166, (byte)46, (byte)109, (byte)81, (byte)199, (byte)200, (byte)199, (byte)57, (byte)180, (byte)7, (byte)147, (byte)209, (byte)207, (byte)31, (byte)49, (byte)109, (byte)0, (byte)14, (byte)119, (byte)12, (byte)179, (byte)18, (byte)26, (byte)254, (byte)133, (byte)55, (byte)163, (byte)216, (byte)238, (byte)33, (byte)63, (byte)51, (byte)129, (byte)37, (byte)197, (byte)152, (byte)38, (byte)17, (byte)7, (byte)216, (byte)86, (byte)106, (byte)36, (byte)97, (byte)0, (byte)172, (byte)18, (byte)186, (byte)148, (byte)164, (byte)160, (byte)48, (byte)82, (byte)125, (byte)171, (byte)186, (byte)193, (byte)132, (byte)29, (byte)112, (byte)80, (byte)69, (byte)31, (byte)40, (byte)110, (byte)48, (byte)247, (byte)32, (byte)235, (byte)133, (byte)190, (byte)21, (byte)175, (byte)41, (byte)216, (byte)94, (byte)155, (byte)160, (byte)12, (byte)125, (byte)162, (byte)132, (byte)40, (byte)145, (byte)232, (byte)129, (byte)163, (byte)51, (byte)152, (byte)174, (byte)11, (byte)47, (byte)229, (byte)113, (byte)226, (byte)109, (byte)202, (byte)20, (byte)145, (byte)230, (byte)38, (byte)213, (byte)140, (byte)93, (byte)174, (byte)33, (byte)8, (byte)39, (byte)42, (byte)33, (byte)77, (byte)220, (byte)5, (byte)144, (byte)152, (byte)90, (byte)254}));
                Debug.Assert(pack.flags == (byte)(byte)54);
                Debug.Assert(pack.len == (byte)(byte)76);
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.data__SET(new byte[] {(byte)105, (byte)1, (byte)191, (byte)19, (byte)115, (byte)140, (byte)22, (byte)19, (byte)53, (byte)33, (byte)197, (byte)153, (byte)38, (byte)210, (byte)54, (byte)215, (byte)94, (byte)52, (byte)121, (byte)38, (byte)67, (byte)103, (byte)56, (byte)20, (byte)196, (byte)211, (byte)237, (byte)61, (byte)215, (byte)107, (byte)101, (byte)58, (byte)160, (byte)124, (byte)108, (byte)192, (byte)185, (byte)102, (byte)124, (byte)203, (byte)16, (byte)25, (byte)241, (byte)90, (byte)217, (byte)254, (byte)140, (byte)200, (byte)10, (byte)24, (byte)179, (byte)207, (byte)201, (byte)185, (byte)44, (byte)248, (byte)174, (byte)163, (byte)42, (byte)254, (byte)89, (byte)32, (byte)217, (byte)166, (byte)46, (byte)109, (byte)81, (byte)199, (byte)200, (byte)199, (byte)57, (byte)180, (byte)7, (byte)147, (byte)209, (byte)207, (byte)31, (byte)49, (byte)109, (byte)0, (byte)14, (byte)119, (byte)12, (byte)179, (byte)18, (byte)26, (byte)254, (byte)133, (byte)55, (byte)163, (byte)216, (byte)238, (byte)33, (byte)63, (byte)51, (byte)129, (byte)37, (byte)197, (byte)152, (byte)38, (byte)17, (byte)7, (byte)216, (byte)86, (byte)106, (byte)36, (byte)97, (byte)0, (byte)172, (byte)18, (byte)186, (byte)148, (byte)164, (byte)160, (byte)48, (byte)82, (byte)125, (byte)171, (byte)186, (byte)193, (byte)132, (byte)29, (byte)112, (byte)80, (byte)69, (byte)31, (byte)40, (byte)110, (byte)48, (byte)247, (byte)32, (byte)235, (byte)133, (byte)190, (byte)21, (byte)175, (byte)41, (byte)216, (byte)94, (byte)155, (byte)160, (byte)12, (byte)125, (byte)162, (byte)132, (byte)40, (byte)145, (byte)232, (byte)129, (byte)163, (byte)51, (byte)152, (byte)174, (byte)11, (byte)47, (byte)229, (byte)113, (byte)226, (byte)109, (byte)202, (byte)20, (byte)145, (byte)230, (byte)38, (byte)213, (byte)140, (byte)93, (byte)174, (byte)33, (byte)8, (byte)39, (byte)42, (byte)33, (byte)77, (byte)220, (byte)5, (byte)144, (byte)152, (byte)90, (byte)254}, 0) ;
            p233.flags = (byte)(byte)54;
            p233.len = (byte)(byte)76;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.custom_mode == (uint)1283045353U);
                Debug.Assert(pack.longitude == (int) -332465978);
                Debug.Assert(pack.wp_num == (byte)(byte)99);
                Debug.Assert(pack.throttle == (sbyte)(sbyte) - 87);
                Debug.Assert(pack.roll == (short)(short) -16268);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED));
                Debug.Assert(pack.altitude_sp == (short)(short)23730);
                Debug.Assert(pack.heading == (ushort)(ushort)16370);
                Debug.Assert(pack.gps_nsat == (byte)(byte)157);
                Debug.Assert(pack.temperature == (sbyte)(sbyte)14);
                Debug.Assert(pack.latitude == (int)1490095017);
                Debug.Assert(pack.gps_fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)89);
                Debug.Assert(pack.battery_remaining == (byte)(byte)82);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
                Debug.Assert(pack.heading_sp == (short)(short) -1168);
                Debug.Assert(pack.pitch == (short)(short)21377);
                Debug.Assert(pack.failsafe == (byte)(byte)97);
                Debug.Assert(pack.airspeed == (byte)(byte)183);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 127);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)48872);
                Debug.Assert(pack.altitude_amsl == (short)(short)24862);
                Debug.Assert(pack.groundspeed == (byte)(byte)181);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte) - 38);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.longitude = (int) -332465978;
            p234.latitude = (int)1490095017;
            p234.groundspeed = (byte)(byte)181;
            p234.altitude_sp = (short)(short)23730;
            p234.roll = (short)(short) -16268;
            p234.wp_num = (byte)(byte)99;
            p234.temperature_air = (sbyte)(sbyte) - 38;
            p234.climb_rate = (sbyte)(sbyte) - 127;
            p234.airspeed_sp = (byte)(byte)89;
            p234.throttle = (sbyte)(sbyte) - 87;
            p234.failsafe = (byte)(byte)97;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p234.wp_distance = (ushort)(ushort)48872;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            p234.airspeed = (byte)(byte)183;
            p234.custom_mode = (uint)1283045353U;
            p234.gps_nsat = (byte)(byte)157;
            p234.heading = (ushort)(ushort)16370;
            p234.temperature = (sbyte)(sbyte)14;
            p234.heading_sp = (short)(short) -1168;
            p234.altitude_amsl = (short)(short)24862;
            p234.pitch = (short)(short)21377;
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
            p234.battery_remaining = (byte)(byte)82;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.clipping_1 == (uint)1266971074U);
                Debug.Assert(pack.time_usec == (ulong)6677133222670614612L);
                Debug.Assert(pack.vibration_y == (float) -1.3813128E38F);
                Debug.Assert(pack.clipping_2 == (uint)3827562867U);
                Debug.Assert(pack.vibration_x == (float) -2.8537028E38F);
                Debug.Assert(pack.clipping_0 == (uint)1486307659U);
                Debug.Assert(pack.vibration_z == (float)2.7974998E38F);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)6677133222670614612L;
            p241.vibration_x = (float) -2.8537028E38F;
            p241.vibration_y = (float) -1.3813128E38F;
            p241.clipping_2 = (uint)3827562867U;
            p241.clipping_1 = (uint)1266971074U;
            p241.vibration_z = (float)2.7974998E38F;
            p241.clipping_0 = (uint)1486307659U;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -2.1301333E38F);
                Debug.Assert(pack.latitude == (int) -1711184342);
                Debug.Assert(pack.x == (float) -3.1760063E38F);
                Debug.Assert(pack.altitude == (int) -2107423340);
                Debug.Assert(pack.y == (float) -1.6306695E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)1839550224487328555L);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.4187952E38F, 2.0830542E38F, 3.7238834E37F, 1.9739699E38F}));
                Debug.Assert(pack.approach_y == (float)1.623458E38F);
                Debug.Assert(pack.approach_x == (float) -2.2717166E35F);
                Debug.Assert(pack.approach_z == (float) -1.1979244E38F);
                Debug.Assert(pack.longitude == (int)1696573780);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.longitude = (int)1696573780;
            p242.x = (float) -3.1760063E38F;
            p242.q_SET(new float[] {1.4187952E38F, 2.0830542E38F, 3.7238834E37F, 1.9739699E38F}, 0) ;
            p242.y = (float) -1.6306695E38F;
            p242.latitude = (int) -1711184342;
            p242.altitude = (int) -2107423340;
            p242.z = (float) -2.1301333E38F;
            p242.time_usec_SET((ulong)1839550224487328555L, PH) ;
            p242.approach_y = (float)1.623458E38F;
            p242.approach_z = (float) -1.1979244E38F;
            p242.approach_x = (float) -2.2717166E35F;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.approach_z == (float) -2.131203E38F);
                Debug.Assert(pack.y == (float) -3.2811422E37F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)1765690949289704943L);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.2380564E38F, 3.1070913E38F, -3.0142652E38F, 2.0129314E38F}));
                Debug.Assert(pack.altitude == (int)403213048);
                Debug.Assert(pack.x == (float)1.4218408E38F);
                Debug.Assert(pack.target_system == (byte)(byte)52);
                Debug.Assert(pack.latitude == (int)1383909789);
                Debug.Assert(pack.approach_y == (float)1.1026717E38F);
                Debug.Assert(pack.longitude == (int) -1521966218);
                Debug.Assert(pack.z == (float) -7.214656E37F);
                Debug.Assert(pack.approach_x == (float) -1.313595E38F);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.altitude = (int)403213048;
            p243.target_system = (byte)(byte)52;
            p243.y = (float) -3.2811422E37F;
            p243.approach_x = (float) -1.313595E38F;
            p243.z = (float) -7.214656E37F;
            p243.x = (float)1.4218408E38F;
            p243.approach_z = (float) -2.131203E38F;
            p243.approach_y = (float)1.1026717E38F;
            p243.q_SET(new float[] {1.2380564E38F, 3.1070913E38F, -3.0142652E38F, 2.0129314E38F}, 0) ;
            p243.time_usec_SET((ulong)1765690949289704943L, PH) ;
            p243.longitude = (int) -1521966218;
            p243.latitude = (int)1383909789;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.interval_us == (int)1890153949);
                Debug.Assert(pack.message_id == (ushort)(ushort)58632);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int)1890153949;
            p244.message_id = (ushort)(ushort)58632;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.squawk == (ushort)(ushort)22568);
                Debug.Assert(pack.tslc == (byte)(byte)105);
                Debug.Assert(pack.ICAO_address == (uint)167949748U);
                Debug.Assert(pack.altitude == (int) -1704206325);
                Debug.Assert(pack.altitude_type == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
                Debug.Assert(pack.callsign_LEN(ph) == 4);
                Debug.Assert(pack.callsign_TRY(ph).Equals("wcpX"));
                Debug.Assert(pack.flags == (ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                                            ADSB_FLAGS.ADSB_FLAGS_SIMULATED |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING));
                Debug.Assert(pack.emitter_type == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_NO_INFO);
                Debug.Assert(pack.heading == (ushort)(ushort)56538);
                Debug.Assert(pack.lon == (int)1156416882);
                Debug.Assert(pack.ver_velocity == (short)(short) -15199);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)4661);
                Debug.Assert(pack.lat == (int)714507772);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.hor_velocity = (ushort)(ushort)4661;
            p246.callsign_SET("wcpX", PH) ;
            p246.lon = (int)1156416882;
            p246.ICAO_address = (uint)167949748U;
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_NO_INFO;
            p246.squawk = (ushort)(ushort)22568;
            p246.altitude = (int) -1704206325;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                          ADSB_FLAGS.ADSB_FLAGS_SIMULATED |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING);
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.heading = (ushort)(ushort)56538;
            p246.ver_velocity = (short)(short) -15199;
            p246.tslc = (byte)(byte)105;
            p246.lat = (int)714507772;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_to_minimum_delta == (float)2.157782E38F);
                Debug.Assert(pack.horizontal_minimum_delta == (float) -1.1451636E38F);
                Debug.Assert(pack.threat_level == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
                Debug.Assert(pack.altitude_minimum_delta == (float)2.9574197E38F);
                Debug.Assert(pack.action == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
                Debug.Assert(pack.id == (uint)3642844027U);
                Debug.Assert(pack.src_ == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.threat_level = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH;
            p247.altitude_minimum_delta = (float)2.9574197E38F;
            p247.time_to_minimum_delta = (float)2.157782E38F;
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.id = (uint)3642844027U;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND;
            p247.horizontal_minimum_delta = (float) -1.1451636E38F;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)67);
                Debug.Assert(pack.message_type == (ushort)(ushort)39383);
                Debug.Assert(pack.target_component == (byte)(byte)100);
                Debug.Assert(pack.target_network == (byte)(byte)238);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)84, (byte)117, (byte)66, (byte)108, (byte)67, (byte)101, (byte)54, (byte)112, (byte)184, (byte)132, (byte)145, (byte)254, (byte)238, (byte)39, (byte)175, (byte)244, (byte)140, (byte)161, (byte)149, (byte)102, (byte)167, (byte)50, (byte)60, (byte)205, (byte)145, (byte)47, (byte)245, (byte)13, (byte)133, (byte)114, (byte)110, (byte)126, (byte)181, (byte)76, (byte)188, (byte)15, (byte)148, (byte)235, (byte)216, (byte)162, (byte)130, (byte)167, (byte)194, (byte)33, (byte)179, (byte)56, (byte)28, (byte)151, (byte)182, (byte)139, (byte)151, (byte)71, (byte)163, (byte)221, (byte)58, (byte)193, (byte)71, (byte)4, (byte)18, (byte)210, (byte)126, (byte)120, (byte)19, (byte)28, (byte)89, (byte)173, (byte)206, (byte)202, (byte)225, (byte)43, (byte)42, (byte)241, (byte)174, (byte)168, (byte)128, (byte)6, (byte)156, (byte)252, (byte)152, (byte)162, (byte)193, (byte)192, (byte)80, (byte)232, (byte)25, (byte)84, (byte)164, (byte)184, (byte)207, (byte)4, (byte)69, (byte)214, (byte)123, (byte)126, (byte)209, (byte)204, (byte)1, (byte)105, (byte)138, (byte)79, (byte)73, (byte)232, (byte)136, (byte)7, (byte)42, (byte)63, (byte)209, (byte)187, (byte)150, (byte)242, (byte)64, (byte)64, (byte)131, (byte)230, (byte)214, (byte)51, (byte)1, (byte)247, (byte)86, (byte)117, (byte)12, (byte)142, (byte)175, (byte)195, (byte)141, (byte)175, (byte)240, (byte)247, (byte)53, (byte)176, (byte)221, (byte)245, (byte)194, (byte)207, (byte)139, (byte)91, (byte)158, (byte)89, (byte)190, (byte)244, (byte)114, (byte)31, (byte)206, (byte)12, (byte)152, (byte)225, (byte)25, (byte)25, (byte)58, (byte)121, (byte)78, (byte)253, (byte)213, (byte)43, (byte)205, (byte)195, (byte)41, (byte)106, (byte)9, (byte)67, (byte)201, (byte)130, (byte)239, (byte)166, (byte)7, (byte)194, (byte)182, (byte)64, (byte)166, (byte)49, (byte)89, (byte)33, (byte)42, (byte)60, (byte)53, (byte)126, (byte)228, (byte)220, (byte)132, (byte)247, (byte)125, (byte)234, (byte)217, (byte)248, (byte)218, (byte)203, (byte)56, (byte)243, (byte)45, (byte)161, (byte)168, (byte)35, (byte)137, (byte)23, (byte)6, (byte)177, (byte)9, (byte)56, (byte)71, (byte)228, (byte)46, (byte)160, (byte)253, (byte)147, (byte)66, (byte)165, (byte)40, (byte)192, (byte)244, (byte)78, (byte)239, (byte)179, (byte)184, (byte)184, (byte)102, (byte)107, (byte)191, (byte)59, (byte)48, (byte)167, (byte)44, (byte)236, (byte)193, (byte)0, (byte)68, (byte)71, (byte)7, (byte)249, (byte)212, (byte)12, (byte)235, (byte)207, (byte)241, (byte)234, (byte)101, (byte)117, (byte)216, (byte)117, (byte)184, (byte)206, (byte)222, (byte)222, (byte)255, (byte)169, (byte)32, (byte)149, (byte)149, (byte)54, (byte)54}));
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.message_type = (ushort)(ushort)39383;
            p248.target_system = (byte)(byte)67;
            p248.payload_SET(new byte[] {(byte)84, (byte)117, (byte)66, (byte)108, (byte)67, (byte)101, (byte)54, (byte)112, (byte)184, (byte)132, (byte)145, (byte)254, (byte)238, (byte)39, (byte)175, (byte)244, (byte)140, (byte)161, (byte)149, (byte)102, (byte)167, (byte)50, (byte)60, (byte)205, (byte)145, (byte)47, (byte)245, (byte)13, (byte)133, (byte)114, (byte)110, (byte)126, (byte)181, (byte)76, (byte)188, (byte)15, (byte)148, (byte)235, (byte)216, (byte)162, (byte)130, (byte)167, (byte)194, (byte)33, (byte)179, (byte)56, (byte)28, (byte)151, (byte)182, (byte)139, (byte)151, (byte)71, (byte)163, (byte)221, (byte)58, (byte)193, (byte)71, (byte)4, (byte)18, (byte)210, (byte)126, (byte)120, (byte)19, (byte)28, (byte)89, (byte)173, (byte)206, (byte)202, (byte)225, (byte)43, (byte)42, (byte)241, (byte)174, (byte)168, (byte)128, (byte)6, (byte)156, (byte)252, (byte)152, (byte)162, (byte)193, (byte)192, (byte)80, (byte)232, (byte)25, (byte)84, (byte)164, (byte)184, (byte)207, (byte)4, (byte)69, (byte)214, (byte)123, (byte)126, (byte)209, (byte)204, (byte)1, (byte)105, (byte)138, (byte)79, (byte)73, (byte)232, (byte)136, (byte)7, (byte)42, (byte)63, (byte)209, (byte)187, (byte)150, (byte)242, (byte)64, (byte)64, (byte)131, (byte)230, (byte)214, (byte)51, (byte)1, (byte)247, (byte)86, (byte)117, (byte)12, (byte)142, (byte)175, (byte)195, (byte)141, (byte)175, (byte)240, (byte)247, (byte)53, (byte)176, (byte)221, (byte)245, (byte)194, (byte)207, (byte)139, (byte)91, (byte)158, (byte)89, (byte)190, (byte)244, (byte)114, (byte)31, (byte)206, (byte)12, (byte)152, (byte)225, (byte)25, (byte)25, (byte)58, (byte)121, (byte)78, (byte)253, (byte)213, (byte)43, (byte)205, (byte)195, (byte)41, (byte)106, (byte)9, (byte)67, (byte)201, (byte)130, (byte)239, (byte)166, (byte)7, (byte)194, (byte)182, (byte)64, (byte)166, (byte)49, (byte)89, (byte)33, (byte)42, (byte)60, (byte)53, (byte)126, (byte)228, (byte)220, (byte)132, (byte)247, (byte)125, (byte)234, (byte)217, (byte)248, (byte)218, (byte)203, (byte)56, (byte)243, (byte)45, (byte)161, (byte)168, (byte)35, (byte)137, (byte)23, (byte)6, (byte)177, (byte)9, (byte)56, (byte)71, (byte)228, (byte)46, (byte)160, (byte)253, (byte)147, (byte)66, (byte)165, (byte)40, (byte)192, (byte)244, (byte)78, (byte)239, (byte)179, (byte)184, (byte)184, (byte)102, (byte)107, (byte)191, (byte)59, (byte)48, (byte)167, (byte)44, (byte)236, (byte)193, (byte)0, (byte)68, (byte)71, (byte)7, (byte)249, (byte)212, (byte)12, (byte)235, (byte)207, (byte)241, (byte)234, (byte)101, (byte)117, (byte)216, (byte)117, (byte)184, (byte)206, (byte)222, (byte)222, (byte)255, (byte)169, (byte)32, (byte)149, (byte)149, (byte)54, (byte)54}, 0) ;
            p248.target_component = (byte)(byte)100;
            p248.target_network = (byte)(byte)238;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ver == (byte)(byte)95);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte) - 39, (sbyte) - 60, (sbyte) - 71, (sbyte) - 103, (sbyte) - 94, (sbyte) - 84, (sbyte)53, (sbyte)19, (sbyte) - 98, (sbyte) - 107, (sbyte) - 38, (sbyte) - 16, (sbyte) - 22, (sbyte) - 87, (sbyte)15, (sbyte) - 90, (sbyte)4, (sbyte)107, (sbyte)49, (sbyte)82, (sbyte)18, (sbyte) - 87, (sbyte) - 65, (sbyte) - 57, (sbyte) - 43, (sbyte) - 36, (sbyte)41, (sbyte)39, (sbyte) - 12, (sbyte) - 76, (sbyte) - 15, (sbyte) - 36}));
                Debug.Assert(pack.type == (byte)(byte)180);
                Debug.Assert(pack.address == (ushort)(ushort)64673);
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)64673;
            p249.value_SET(new sbyte[] {(sbyte) - 39, (sbyte) - 60, (sbyte) - 71, (sbyte) - 103, (sbyte) - 94, (sbyte) - 84, (sbyte)53, (sbyte)19, (sbyte) - 98, (sbyte) - 107, (sbyte) - 38, (sbyte) - 16, (sbyte) - 22, (sbyte) - 87, (sbyte)15, (sbyte) - 90, (sbyte)4, (sbyte)107, (sbyte)49, (sbyte)82, (sbyte)18, (sbyte) - 87, (sbyte) - 65, (sbyte) - 57, (sbyte) - 43, (sbyte) - 36, (sbyte)41, (sbyte)39, (sbyte) - 12, (sbyte) - 76, (sbyte) - 15, (sbyte) - 36}, 0) ;
            p249.type = (byte)(byte)180;
            p249.ver = (byte)(byte)95;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -4.356381E37F);
                Debug.Assert(pack.time_usec == (ulong)5092225006306294555L);
                Debug.Assert(pack.name_LEN(ph) == 10);
                Debug.Assert(pack.name_TRY(ph).Equals("gvcfthmjem"));
                Debug.Assert(pack.x == (float)9.615135E37F);
                Debug.Assert(pack.y == (float)1.0038265E38F);
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.time_usec = (ulong)5092225006306294555L;
            p250.x = (float)9.615135E37F;
            p250.y = (float)1.0038265E38F;
            p250.name_SET("gvcfthmjem", PH) ;
            p250.z = (float) -4.356381E37F;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 4);
                Debug.Assert(pack.name_TRY(ph).Equals("xunx"));
                Debug.Assert(pack.value == (float) -2.5964206E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3270515926U);
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.name_SET("xunx", PH) ;
            p251.value = (float) -2.5964206E38F;
            p251.time_boot_ms = (uint)3270515926U;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (int)2067713384);
                Debug.Assert(pack.name_LEN(ph) == 3);
                Debug.Assert(pack.name_TRY(ph).Equals("jsl"));
                Debug.Assert(pack.time_boot_ms == (uint)228931520U);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.value = (int)2067713384;
            p252.name_SET("jsl", PH) ;
            p252.time_boot_ms = (uint)228931520U;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 36);
                Debug.Assert(pack.text_TRY(ph).Equals("NtadkYjtnnOzetydlipavvlvjebmyzuiaojv"));
                Debug.Assert(pack.severity == MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_CRITICAL;
            p253.text_SET("NtadkYjtnnOzetydlipavvlvjebmyzuiaojv", PH) ;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float)2.1650592E38F);
                Debug.Assert(pack.ind == (byte)(byte)64);
                Debug.Assert(pack.time_boot_ms == (uint)2860105096U);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)2860105096U;
            p254.ind = (byte)(byte)64;
            p254.value = (float)2.1650592E38F;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.initial_timestamp == (ulong)949277796597334923L);
                Debug.Assert(pack.target_system == (byte)(byte)153);
                Debug.Assert(pack.target_component == (byte)(byte)107);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)254, (byte)211, (byte)72, (byte)190, (byte)137, (byte)163, (byte)100, (byte)218, (byte)215, (byte)179, (byte)93, (byte)127, (byte)156, (byte)35, (byte)134, (byte)227, (byte)165, (byte)206, (byte)28, (byte)123, (byte)177, (byte)205, (byte)200, (byte)15, (byte)39, (byte)195, (byte)30, (byte)153, (byte)199, (byte)199, (byte)179, (byte)55}));
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)153;
            p256.initial_timestamp = (ulong)949277796597334923L;
            p256.secret_key_SET(new byte[] {(byte)254, (byte)211, (byte)72, (byte)190, (byte)137, (byte)163, (byte)100, (byte)218, (byte)215, (byte)179, (byte)93, (byte)127, (byte)156, (byte)35, (byte)134, (byte)227, (byte)165, (byte)206, (byte)28, (byte)123, (byte)177, (byte)205, (byte)200, (byte)15, (byte)39, (byte)195, (byte)30, (byte)153, (byte)199, (byte)199, (byte)179, (byte)55}, 0) ;
            p256.target_component = (byte)(byte)107;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3568919892U);
                Debug.Assert(pack.last_change_ms == (uint)3152721501U);
                Debug.Assert(pack.state == (byte)(byte)253);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.state = (byte)(byte)253;
            p257.last_change_ms = (uint)3152721501U;
            p257.time_boot_ms = (uint)3568919892U;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)22);
                Debug.Assert(pack.tune_LEN(ph) == 4);
                Debug.Assert(pack.tune_TRY(ph).Equals("qrms"));
                Debug.Assert(pack.target_component == (byte)(byte)140);
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.tune_SET("qrms", PH) ;
            p258.target_component = (byte)(byte)140;
            p258.target_system = (byte)(byte)22;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 18);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("Iniaspsqyhsapjrqrp"));
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)221, (byte)43, (byte)219, (byte)124, (byte)82, (byte)154, (byte)96, (byte)47, (byte)143, (byte)60, (byte)153, (byte)98, (byte)145, (byte)93, (byte)130, (byte)167, (byte)59, (byte)148, (byte)102, (byte)39, (byte)230, (byte)40, (byte)161, (byte)224, (byte)96, (byte)204, (byte)64, (byte)250, (byte)224, (byte)194, (byte)10, (byte)18}));
                Debug.Assert(pack.sensor_size_v == (float)1.1329809E38F);
                Debug.Assert(pack.sensor_size_h == (float) -1.5711498E38F);
                Debug.Assert(pack.time_boot_ms == (uint)498719244U);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)7993);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)17101);
                Debug.Assert(pack.focal_length == (float)1.5841929E38F);
                Debug.Assert(pack.firmware_version == (uint)3459088210U);
                Debug.Assert(pack.lens_id == (byte)(byte)164);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)57412);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE));
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)57, (byte)152, (byte)135, (byte)165, (byte)230, (byte)163, (byte)80, (byte)25, (byte)11, (byte)117, (byte)214, (byte)94, (byte)84, (byte)219, (byte)184, (byte)187, (byte)8, (byte)98, (byte)192, (byte)0, (byte)250, (byte)52, (byte)146, (byte)19, (byte)84, (byte)222, (byte)211, (byte)48, (byte)237, (byte)202, (byte)215, (byte)112}));
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.model_name_SET(new byte[] {(byte)221, (byte)43, (byte)219, (byte)124, (byte)82, (byte)154, (byte)96, (byte)47, (byte)143, (byte)60, (byte)153, (byte)98, (byte)145, (byte)93, (byte)130, (byte)167, (byte)59, (byte)148, (byte)102, (byte)39, (byte)230, (byte)40, (byte)161, (byte)224, (byte)96, (byte)204, (byte)64, (byte)250, (byte)224, (byte)194, (byte)10, (byte)18}, 0) ;
            p259.cam_definition_uri_SET("Iniaspsqyhsapjrqrp", PH) ;
            p259.time_boot_ms = (uint)498719244U;
            p259.vendor_name_SET(new byte[] {(byte)57, (byte)152, (byte)135, (byte)165, (byte)230, (byte)163, (byte)80, (byte)25, (byte)11, (byte)117, (byte)214, (byte)94, (byte)84, (byte)219, (byte)184, (byte)187, (byte)8, (byte)98, (byte)192, (byte)0, (byte)250, (byte)52, (byte)146, (byte)19, (byte)84, (byte)222, (byte)211, (byte)48, (byte)237, (byte)202, (byte)215, (byte)112}, 0) ;
            p259.lens_id = (byte)(byte)164;
            p259.cam_definition_version = (ushort)(ushort)7993;
            p259.resolution_v = (ushort)(ushort)57412;
            p259.sensor_size_v = (float)1.1329809E38F;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
            p259.resolution_h = (ushort)(ushort)17101;
            p259.focal_length = (float)1.5841929E38F;
            p259.firmware_version = (uint)3459088210U;
            p259.sensor_size_h = (float) -1.5711498E38F;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == CAMERA_MODE.CAMERA_MODE_IMAGE);
                Debug.Assert(pack.time_boot_ms == (uint)2788027558U);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)2788027558U;
            p260.mode_id = CAMERA_MODE.CAMERA_MODE_IMAGE;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.storage_count == (byte)(byte)114);
                Debug.Assert(pack.status == (byte)(byte)123);
                Debug.Assert(pack.used_capacity == (float)1.2881701E38F);
                Debug.Assert(pack.write_speed == (float) -2.4581175E38F);
                Debug.Assert(pack.storage_id == (byte)(byte)237);
                Debug.Assert(pack.read_speed == (float) -7.216223E37F);
                Debug.Assert(pack.total_capacity == (float)2.5794635E38F);
                Debug.Assert(pack.available_capacity == (float)2.087346E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2129243012U);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.write_speed = (float) -2.4581175E38F;
            p261.read_speed = (float) -7.216223E37F;
            p261.used_capacity = (float)1.2881701E38F;
            p261.time_boot_ms = (uint)2129243012U;
            p261.storage_count = (byte)(byte)114;
            p261.total_capacity = (float)2.5794635E38F;
            p261.status = (byte)(byte)123;
            p261.storage_id = (byte)(byte)237;
            p261.available_capacity = (float)2.087346E38F;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)968516876U);
                Debug.Assert(pack.video_status == (byte)(byte)176);
                Debug.Assert(pack.image_status == (byte)(byte)181);
                Debug.Assert(pack.image_interval == (float)1.4049889E37F);
                Debug.Assert(pack.recording_time_ms == (uint)3020346217U);
                Debug.Assert(pack.available_capacity == (float)9.587671E37F);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.recording_time_ms = (uint)3020346217U;
            p262.time_boot_ms = (uint)968516876U;
            p262.image_status = (byte)(byte)181;
            p262.video_status = (byte)(byte)176;
            p262.image_interval = (float)1.4049889E37F;
            p262.available_capacity = (float)9.587671E37F;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -913036734);
                Debug.Assert(pack.lon == (int)371659982);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte) - 113);
                Debug.Assert(pack.relative_alt == (int)844058050);
                Debug.Assert(pack.alt == (int) -1561021652);
                Debug.Assert(pack.image_index == (int) -2138538984);
                Debug.Assert(pack.file_url_LEN(ph) == 8);
                Debug.Assert(pack.file_url_TRY(ph).Equals("cpzkekad"));
                Debug.Assert(pack.camera_id == (byte)(byte)105);
                Debug.Assert(pack.time_boot_ms == (uint)716540424U);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-8.643218E37F, 2.6832984E37F, -3.9005016E37F, -2.5684135E38F}));
                Debug.Assert(pack.time_utc == (ulong)5531956375643129850L);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.lon = (int)371659982;
            p263.capture_result = (sbyte)(sbyte) - 113;
            p263.image_index = (int) -2138538984;
            p263.lat = (int) -913036734;
            p263.relative_alt = (int)844058050;
            p263.time_utc = (ulong)5531956375643129850L;
            p263.q_SET(new float[] {-8.643218E37F, 2.6832984E37F, -3.9005016E37F, -2.5684135E38F}, 0) ;
            p263.alt = (int) -1561021652;
            p263.time_boot_ms = (uint)716540424U;
            p263.camera_id = (byte)(byte)105;
            p263.file_url_SET("cpzkekad", PH) ;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)4090390131U);
                Debug.Assert(pack.takeoff_time_utc == (ulong)2422557407617634658L);
                Debug.Assert(pack.flight_uuid == (ulong)5025941196033266769L);
                Debug.Assert(pack.arming_time_utc == (ulong)3911129874678466456L);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)4090390131U;
            p264.flight_uuid = (ulong)5025941196033266769L;
            p264.arming_time_utc = (ulong)3911129874678466456L;
            p264.takeoff_time_utc = (ulong)2422557407617634658L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2436008597U);
                Debug.Assert(pack.roll == (float)8.756488E37F);
                Debug.Assert(pack.yaw == (float)1.2166001E38F);
                Debug.Assert(pack.pitch == (float)1.6647678E38F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.yaw = (float)1.2166001E38F;
            p265.pitch = (float)1.6647678E38F;
            p265.roll = (float)8.756488E37F;
            p265.time_boot_ms = (uint)2436008597U;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)102);
                Debug.Assert(pack.first_message_offset == (byte)(byte)108);
                Debug.Assert(pack.length == (byte)(byte)70);
                Debug.Assert(pack.sequence == (ushort)(ushort)42862);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)128, (byte)231, (byte)238, (byte)216, (byte)214, (byte)85, (byte)250, (byte)107, (byte)231, (byte)129, (byte)186, (byte)170, (byte)147, (byte)243, (byte)193, (byte)79, (byte)176, (byte)158, (byte)185, (byte)219, (byte)17, (byte)211, (byte)178, (byte)29, (byte)160, (byte)54, (byte)195, (byte)33, (byte)48, (byte)29, (byte)131, (byte)85, (byte)111, (byte)30, (byte)214, (byte)237, (byte)216, (byte)25, (byte)43, (byte)210, (byte)128, (byte)97, (byte)10, (byte)119, (byte)221, (byte)227, (byte)143, (byte)73, (byte)204, (byte)194, (byte)70, (byte)122, (byte)210, (byte)138, (byte)208, (byte)128, (byte)183, (byte)113, (byte)132, (byte)183, (byte)73, (byte)49, (byte)37, (byte)150, (byte)210, (byte)170, (byte)180, (byte)168, (byte)13, (byte)234, (byte)244, (byte)226, (byte)235, (byte)135, (byte)9, (byte)18, (byte)190, (byte)148, (byte)232, (byte)223, (byte)97, (byte)176, (byte)164, (byte)10, (byte)125, (byte)17, (byte)1, (byte)252, (byte)71, (byte)99, (byte)163, (byte)76, (byte)47, (byte)43, (byte)89, (byte)27, (byte)64, (byte)126, (byte)12, (byte)110, (byte)155, (byte)64, (byte)184, (byte)208, (byte)77, (byte)232, (byte)165, (byte)240, (byte)40, (byte)192, (byte)151, (byte)122, (byte)138, (byte)157, (byte)123, (byte)185, (byte)60, (byte)161, (byte)222, (byte)145, (byte)146, (byte)154, (byte)211, (byte)102, (byte)46, (byte)59, (byte)237, (byte)0, (byte)245, (byte)41, (byte)247, (byte)128, (byte)195, (byte)221, (byte)76, (byte)230, (byte)238, (byte)235, (byte)208, (byte)82, (byte)228, (byte)86, (byte)235, (byte)212, (byte)56, (byte)26, (byte)76, (byte)20, (byte)250, (byte)178, (byte)136, (byte)184, (byte)16, (byte)51, (byte)96, (byte)177, (byte)213, (byte)125, (byte)56, (byte)128, (byte)79, (byte)161, (byte)135, (byte)191, (byte)162, (byte)70, (byte)87, (byte)235, (byte)6, (byte)85, (byte)95, (byte)111, (byte)109, (byte)77, (byte)61, (byte)250, (byte)242, (byte)175, (byte)54, (byte)137, (byte)68, (byte)250, (byte)215, (byte)108, (byte)165, (byte)172, (byte)146, (byte)119, (byte)214, (byte)43, (byte)141, (byte)180, (byte)6, (byte)134, (byte)173, (byte)195, (byte)42, (byte)223, (byte)135, (byte)211, (byte)138, (byte)209, (byte)137, (byte)90, (byte)241, (byte)171, (byte)1, (byte)31, (byte)72, (byte)209, (byte)74, (byte)182, (byte)55, (byte)165, (byte)94, (byte)168, (byte)153, (byte)171, (byte)118, (byte)120, (byte)85, (byte)17, (byte)181, (byte)233, (byte)106, (byte)94, (byte)240, (byte)119, (byte)179, (byte)146, (byte)158, (byte)163, (byte)145, (byte)50, (byte)206, (byte)103, (byte)154, (byte)157, (byte)23, (byte)76, (byte)219, (byte)154, (byte)219, (byte)15, (byte)82, (byte)245, (byte)66, (byte)68, (byte)237}));
                Debug.Assert(pack.target_system == (byte)(byte)75);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_component = (byte)(byte)102;
            p266.target_system = (byte)(byte)75;
            p266.length = (byte)(byte)70;
            p266.first_message_offset = (byte)(byte)108;
            p266.data__SET(new byte[] {(byte)128, (byte)231, (byte)238, (byte)216, (byte)214, (byte)85, (byte)250, (byte)107, (byte)231, (byte)129, (byte)186, (byte)170, (byte)147, (byte)243, (byte)193, (byte)79, (byte)176, (byte)158, (byte)185, (byte)219, (byte)17, (byte)211, (byte)178, (byte)29, (byte)160, (byte)54, (byte)195, (byte)33, (byte)48, (byte)29, (byte)131, (byte)85, (byte)111, (byte)30, (byte)214, (byte)237, (byte)216, (byte)25, (byte)43, (byte)210, (byte)128, (byte)97, (byte)10, (byte)119, (byte)221, (byte)227, (byte)143, (byte)73, (byte)204, (byte)194, (byte)70, (byte)122, (byte)210, (byte)138, (byte)208, (byte)128, (byte)183, (byte)113, (byte)132, (byte)183, (byte)73, (byte)49, (byte)37, (byte)150, (byte)210, (byte)170, (byte)180, (byte)168, (byte)13, (byte)234, (byte)244, (byte)226, (byte)235, (byte)135, (byte)9, (byte)18, (byte)190, (byte)148, (byte)232, (byte)223, (byte)97, (byte)176, (byte)164, (byte)10, (byte)125, (byte)17, (byte)1, (byte)252, (byte)71, (byte)99, (byte)163, (byte)76, (byte)47, (byte)43, (byte)89, (byte)27, (byte)64, (byte)126, (byte)12, (byte)110, (byte)155, (byte)64, (byte)184, (byte)208, (byte)77, (byte)232, (byte)165, (byte)240, (byte)40, (byte)192, (byte)151, (byte)122, (byte)138, (byte)157, (byte)123, (byte)185, (byte)60, (byte)161, (byte)222, (byte)145, (byte)146, (byte)154, (byte)211, (byte)102, (byte)46, (byte)59, (byte)237, (byte)0, (byte)245, (byte)41, (byte)247, (byte)128, (byte)195, (byte)221, (byte)76, (byte)230, (byte)238, (byte)235, (byte)208, (byte)82, (byte)228, (byte)86, (byte)235, (byte)212, (byte)56, (byte)26, (byte)76, (byte)20, (byte)250, (byte)178, (byte)136, (byte)184, (byte)16, (byte)51, (byte)96, (byte)177, (byte)213, (byte)125, (byte)56, (byte)128, (byte)79, (byte)161, (byte)135, (byte)191, (byte)162, (byte)70, (byte)87, (byte)235, (byte)6, (byte)85, (byte)95, (byte)111, (byte)109, (byte)77, (byte)61, (byte)250, (byte)242, (byte)175, (byte)54, (byte)137, (byte)68, (byte)250, (byte)215, (byte)108, (byte)165, (byte)172, (byte)146, (byte)119, (byte)214, (byte)43, (byte)141, (byte)180, (byte)6, (byte)134, (byte)173, (byte)195, (byte)42, (byte)223, (byte)135, (byte)211, (byte)138, (byte)209, (byte)137, (byte)90, (byte)241, (byte)171, (byte)1, (byte)31, (byte)72, (byte)209, (byte)74, (byte)182, (byte)55, (byte)165, (byte)94, (byte)168, (byte)153, (byte)171, (byte)118, (byte)120, (byte)85, (byte)17, (byte)181, (byte)233, (byte)106, (byte)94, (byte)240, (byte)119, (byte)179, (byte)146, (byte)158, (byte)163, (byte)145, (byte)50, (byte)206, (byte)103, (byte)154, (byte)157, (byte)23, (byte)76, (byte)219, (byte)154, (byte)219, (byte)15, (byte)82, (byte)245, (byte)66, (byte)68, (byte)237}, 0) ;
            p266.sequence = (ushort)(ushort)42862;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)156);
                Debug.Assert(pack.first_message_offset == (byte)(byte)34);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)220, (byte)183, (byte)150, (byte)60, (byte)137, (byte)209, (byte)97, (byte)20, (byte)161, (byte)235, (byte)130, (byte)67, (byte)154, (byte)159, (byte)80, (byte)110, (byte)176, (byte)240, (byte)239, (byte)72, (byte)92, (byte)93, (byte)183, (byte)60, (byte)46, (byte)93, (byte)218, (byte)235, (byte)237, (byte)35, (byte)180, (byte)27, (byte)183, (byte)43, (byte)237, (byte)88, (byte)127, (byte)38, (byte)71, (byte)41, (byte)109, (byte)167, (byte)252, (byte)183, (byte)162, (byte)41, (byte)54, (byte)246, (byte)82, (byte)55, (byte)155, (byte)168, (byte)208, (byte)194, (byte)188, (byte)149, (byte)206, (byte)178, (byte)53, (byte)73, (byte)28, (byte)11, (byte)201, (byte)208, (byte)208, (byte)119, (byte)78, (byte)53, (byte)39, (byte)93, (byte)25, (byte)207, (byte)6, (byte)10, (byte)237, (byte)113, (byte)247, (byte)24, (byte)91, (byte)186, (byte)200, (byte)158, (byte)36, (byte)59, (byte)76, (byte)248, (byte)229, (byte)220, (byte)165, (byte)64, (byte)145, (byte)180, (byte)34, (byte)131, (byte)18, (byte)75, (byte)197, (byte)45, (byte)203, (byte)62, (byte)184, (byte)47, (byte)228, (byte)30, (byte)185, (byte)132, (byte)186, (byte)254, (byte)54, (byte)240, (byte)16, (byte)76, (byte)229, (byte)123, (byte)46, (byte)168, (byte)98, (byte)94, (byte)187, (byte)8, (byte)115, (byte)132, (byte)88, (byte)246, (byte)26, (byte)227, (byte)234, (byte)12, (byte)133, (byte)23, (byte)24, (byte)145, (byte)122, (byte)182, (byte)106, (byte)141, (byte)255, (byte)255, (byte)53, (byte)155, (byte)84, (byte)176, (byte)127, (byte)242, (byte)18, (byte)35, (byte)131, (byte)81, (byte)52, (byte)253, (byte)4, (byte)165, (byte)66, (byte)253, (byte)115, (byte)187, (byte)116, (byte)89, (byte)160, (byte)83, (byte)161, (byte)204, (byte)42, (byte)152, (byte)39, (byte)205, (byte)238, (byte)7, (byte)227, (byte)124, (byte)192, (byte)61, (byte)106, (byte)161, (byte)163, (byte)62, (byte)227, (byte)179, (byte)114, (byte)154, (byte)150, (byte)213, (byte)244, (byte)39, (byte)74, (byte)64, (byte)212, (byte)142, (byte)46, (byte)145, (byte)215, (byte)126, (byte)64, (byte)191, (byte)111, (byte)99, (byte)135, (byte)146, (byte)34, (byte)198, (byte)209, (byte)95, (byte)129, (byte)58, (byte)199, (byte)31, (byte)203, (byte)235, (byte)99, (byte)138, (byte)204, (byte)98, (byte)3, (byte)114, (byte)140, (byte)192, (byte)122, (byte)46, (byte)143, (byte)8, (byte)149, (byte)104, (byte)22, (byte)177, (byte)156, (byte)105, (byte)202, (byte)235, (byte)137, (byte)113, (byte)1, (byte)153, (byte)110, (byte)90, (byte)160, (byte)57, (byte)223, (byte)128, (byte)31, (byte)142, (byte)246, (byte)225, (byte)108, (byte)161, (byte)206, (byte)47, (byte)44, (byte)142, (byte)105}));
                Debug.Assert(pack.sequence == (ushort)(ushort)57153);
                Debug.Assert(pack.target_component == (byte)(byte)164);
                Debug.Assert(pack.length == (byte)(byte)233);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_component = (byte)(byte)164;
            p267.sequence = (ushort)(ushort)57153;
            p267.target_system = (byte)(byte)156;
            p267.first_message_offset = (byte)(byte)34;
            p267.length = (byte)(byte)233;
            p267.data__SET(new byte[] {(byte)220, (byte)183, (byte)150, (byte)60, (byte)137, (byte)209, (byte)97, (byte)20, (byte)161, (byte)235, (byte)130, (byte)67, (byte)154, (byte)159, (byte)80, (byte)110, (byte)176, (byte)240, (byte)239, (byte)72, (byte)92, (byte)93, (byte)183, (byte)60, (byte)46, (byte)93, (byte)218, (byte)235, (byte)237, (byte)35, (byte)180, (byte)27, (byte)183, (byte)43, (byte)237, (byte)88, (byte)127, (byte)38, (byte)71, (byte)41, (byte)109, (byte)167, (byte)252, (byte)183, (byte)162, (byte)41, (byte)54, (byte)246, (byte)82, (byte)55, (byte)155, (byte)168, (byte)208, (byte)194, (byte)188, (byte)149, (byte)206, (byte)178, (byte)53, (byte)73, (byte)28, (byte)11, (byte)201, (byte)208, (byte)208, (byte)119, (byte)78, (byte)53, (byte)39, (byte)93, (byte)25, (byte)207, (byte)6, (byte)10, (byte)237, (byte)113, (byte)247, (byte)24, (byte)91, (byte)186, (byte)200, (byte)158, (byte)36, (byte)59, (byte)76, (byte)248, (byte)229, (byte)220, (byte)165, (byte)64, (byte)145, (byte)180, (byte)34, (byte)131, (byte)18, (byte)75, (byte)197, (byte)45, (byte)203, (byte)62, (byte)184, (byte)47, (byte)228, (byte)30, (byte)185, (byte)132, (byte)186, (byte)254, (byte)54, (byte)240, (byte)16, (byte)76, (byte)229, (byte)123, (byte)46, (byte)168, (byte)98, (byte)94, (byte)187, (byte)8, (byte)115, (byte)132, (byte)88, (byte)246, (byte)26, (byte)227, (byte)234, (byte)12, (byte)133, (byte)23, (byte)24, (byte)145, (byte)122, (byte)182, (byte)106, (byte)141, (byte)255, (byte)255, (byte)53, (byte)155, (byte)84, (byte)176, (byte)127, (byte)242, (byte)18, (byte)35, (byte)131, (byte)81, (byte)52, (byte)253, (byte)4, (byte)165, (byte)66, (byte)253, (byte)115, (byte)187, (byte)116, (byte)89, (byte)160, (byte)83, (byte)161, (byte)204, (byte)42, (byte)152, (byte)39, (byte)205, (byte)238, (byte)7, (byte)227, (byte)124, (byte)192, (byte)61, (byte)106, (byte)161, (byte)163, (byte)62, (byte)227, (byte)179, (byte)114, (byte)154, (byte)150, (byte)213, (byte)244, (byte)39, (byte)74, (byte)64, (byte)212, (byte)142, (byte)46, (byte)145, (byte)215, (byte)126, (byte)64, (byte)191, (byte)111, (byte)99, (byte)135, (byte)146, (byte)34, (byte)198, (byte)209, (byte)95, (byte)129, (byte)58, (byte)199, (byte)31, (byte)203, (byte)235, (byte)99, (byte)138, (byte)204, (byte)98, (byte)3, (byte)114, (byte)140, (byte)192, (byte)122, (byte)46, (byte)143, (byte)8, (byte)149, (byte)104, (byte)22, (byte)177, (byte)156, (byte)105, (byte)202, (byte)235, (byte)137, (byte)113, (byte)1, (byte)153, (byte)110, (byte)90, (byte)160, (byte)57, (byte)223, (byte)128, (byte)31, (byte)142, (byte)246, (byte)225, (byte)108, (byte)161, (byte)206, (byte)47, (byte)44, (byte)142, (byte)105}, 0) ;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sequence == (ushort)(ushort)53085);
                Debug.Assert(pack.target_system == (byte)(byte)3);
                Debug.Assert(pack.target_component == (byte)(byte)143);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.sequence = (ushort)(ushort)53085;
            p268.target_component = (byte)(byte)143;
            p268.target_system = (byte)(byte)3;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_v == (ushort)(ushort)9279);
                Debug.Assert(pack.framerate == (float) -2.6874755E38F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)10050);
                Debug.Assert(pack.rotation == (ushort)(ushort)6358);
                Debug.Assert(pack.camera_id == (byte)(byte)29);
                Debug.Assert(pack.status == (byte)(byte)225);
                Debug.Assert(pack.uri_LEN(ph) == 170);
                Debug.Assert(pack.uri_TRY(ph).Equals("kbsvblwwgEmEwtjaggsNmvameXrlmJtFafwnciuDrsinnqwksqfvxTexxhriqcznhutcqvxostmlFrvxDpcodfHynncqbrshWuwohblnjdnaaydbqemubpqwypoEBbwsAdwYzXrgcrwclopXulrlcltbrgalskfRrlforGYzOt"));
                Debug.Assert(pack.bitrate == (uint)4057565648U);
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.resolution_v = (ushort)(ushort)9279;
            p269.rotation = (ushort)(ushort)6358;
            p269.framerate = (float) -2.6874755E38F;
            p269.resolution_h = (ushort)(ushort)10050;
            p269.status = (byte)(byte)225;
            p269.uri_SET("kbsvblwwgEmEwtjaggsNmvameXrlmJtFafwnciuDrsinnqwksqfvxTexxhriqcznhutcqvxostmlFrvxDpcodfHynncqbrshWuwohblnjdnaaydbqemubpqwypoEBbwsAdwYzXrgcrwclopXulrlcltbrgalskfRrlforGYzOt", PH) ;
            p269.bitrate = (uint)4057565648U;
            p269.camera_id = (byte)(byte)29;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)135);
                Debug.Assert(pack.framerate == (float) -2.4793437E38F);
                Debug.Assert(pack.bitrate == (uint)3953587559U);
                Debug.Assert(pack.uri_LEN(ph) == 79);
                Debug.Assert(pack.uri_TRY(ph).Equals("egwympZEkmucAgUjvgvhocvpsgwCofzzykesranqxhtduxzncrnoXbswnVwbbfuzwzytqwipQgufbeI"));
                Debug.Assert(pack.resolution_h == (ushort)(ushort)44568);
                Debug.Assert(pack.camera_id == (byte)(byte)148);
                Debug.Assert(pack.target_component == (byte)(byte)135);
                Debug.Assert(pack.rotation == (ushort)(ushort)17052);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)22925);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)135;
            p270.uri_SET("egwympZEkmucAgUjvgvhocvpsgwCofzzykesranqxhtduxzncrnoXbswnVwbbfuzwzytqwipQgufbeI", PH) ;
            p270.framerate = (float) -2.4793437E38F;
            p270.resolution_h = (ushort)(ushort)44568;
            p270.bitrate = (uint)3953587559U;
            p270.camera_id = (byte)(byte)148;
            p270.target_component = (byte)(byte)135;
            p270.rotation = (ushort)(ushort)17052;
            p270.resolution_v = (ushort)(ushort)22925;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 13);
                Debug.Assert(pack.password_TRY(ph).Equals("pwhvbCttzdztf"));
                Debug.Assert(pack.ssid_LEN(ph) == 31);
                Debug.Assert(pack.ssid_TRY(ph).Equals("yofyqywcdPubifezatxviIdpiuipgtD"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.password_SET("pwhvbCttzdztf", PH) ;
            p299.ssid_SET("yofyqywcdPubifezatxviIdpiuipgtD", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_version == (ushort)(ushort)47840);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)160, (byte)248, (byte)52, (byte)28, (byte)147, (byte)8, (byte)172, (byte)225}));
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)36, (byte)211, (byte)85, (byte)119, (byte)114, (byte)0, (byte)106, (byte)189}));
                Debug.Assert(pack.version == (ushort)(ushort)34709);
                Debug.Assert(pack.max_version == (ushort)(ushort)40082);
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.min_version = (ushort)(ushort)47840;
            p300.spec_version_hash_SET(new byte[] {(byte)160, (byte)248, (byte)52, (byte)28, (byte)147, (byte)8, (byte)172, (byte)225}, 0) ;
            p300.library_version_hash_SET(new byte[] {(byte)36, (byte)211, (byte)85, (byte)119, (byte)114, (byte)0, (byte)106, (byte)189}, 0) ;
            p300.max_version = (ushort)(ushort)40082;
            p300.version = (ushort)(ushort)34709;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.health == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
                Debug.Assert(pack.sub_mode == (byte)(byte)169);
                Debug.Assert(pack.time_usec == (ulong)6027781442254043237L);
                Debug.Assert(pack.uptime_sec == (uint)4206653503U);
                Debug.Assert(pack.mode == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)43362);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.vendor_specific_status_code = (ushort)(ushort)43362;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR;
            p310.sub_mode = (byte)(byte)169;
            p310.uptime_sec = (uint)4206653503U;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE;
            p310.time_usec = (ulong)6027781442254043237L;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sw_vcs_commit == (uint)3718793229U);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)55);
                Debug.Assert(pack.uptime_sec == (uint)1612982609U);
                Debug.Assert(pack.name_LEN(ph) == 40);
                Debug.Assert(pack.name_TRY(ph).Equals("vvxergtYwsvZjqSxggHxxiwdyilvmpxtcWBgCKlq"));
                Debug.Assert(pack.time_usec == (ulong)1136732904850459090L);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)141, (byte)78, (byte)179, (byte)145, (byte)115, (byte)142, (byte)254, (byte)65, (byte)138, (byte)12, (byte)161, (byte)132, (byte)105, (byte)152, (byte)45, (byte)194}));
                Debug.Assert(pack.sw_version_minor == (byte)(byte)156);
                Debug.Assert(pack.hw_version_major == (byte)(byte)185);
                Debug.Assert(pack.sw_version_major == (byte)(byte)251);
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)1136732904850459090L;
            p311.sw_version_major = (byte)(byte)251;
            p311.hw_version_minor = (byte)(byte)55;
            p311.hw_unique_id_SET(new byte[] {(byte)141, (byte)78, (byte)179, (byte)145, (byte)115, (byte)142, (byte)254, (byte)65, (byte)138, (byte)12, (byte)161, (byte)132, (byte)105, (byte)152, (byte)45, (byte)194}, 0) ;
            p311.sw_vcs_commit = (uint)3718793229U;
            p311.hw_version_major = (byte)(byte)185;
            p311.name_SET("vvxergtYwsvZjqSxggHxxiwdyilvmpxtcWBgCKlq", PH) ;
            p311.uptime_sec = (uint)1612982609U;
            p311.sw_version_minor = (byte)(byte)156;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)97);
                Debug.Assert(pack.target_component == (byte)(byte)3);
                Debug.Assert(pack.param_id_LEN(ph) == 9);
                Debug.Assert(pack.param_id_TRY(ph).Equals("aoyiBaovx"));
                Debug.Assert(pack.param_index == (short)(short) -29456);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_component = (byte)(byte)3;
            p320.target_system = (byte)(byte)97;
            p320.param_id_SET("aoyiBaovx", PH) ;
            p320.param_index = (short)(short) -29456;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)106);
                Debug.Assert(pack.target_system == (byte)(byte)117);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_component = (byte)(byte)106;
            p321.target_system = (byte)(byte)117;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_count == (ushort)(ushort)29528);
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
                Debug.Assert(pack.param_index == (ushort)(ushort)12167);
                Debug.Assert(pack.param_id_LEN(ph) == 11);
                Debug.Assert(pack.param_id_TRY(ph).Equals("pjvTrqwoqku"));
                Debug.Assert(pack.param_value_LEN(ph) == 111);
                Debug.Assert(pack.param_value_TRY(ph).Equals("bpqoihvbpjgqofthqyholjmoczpjspWWbgyaystgucbbsfkrvlxpfMyecqhkxvlwzwvfgmkilyicijdmdiugJcsJblbfgyebivpXfvmkiabcwZa"));
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("pjvTrqwoqku", PH) ;
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64;
            p322.param_index = (ushort)(ushort)12167;
            p322.param_count = (ushort)(ushort)29528;
            p322.param_value_SET("bpqoihvbpjgqofthqyholjmoczpjspWWbgyaystgucbbsfkrvlxpfMyecqhkxvlwzwvfgmkilyicijdmdiugJcsJblbfgyebivpXfvmkiabcwZa", PH) ;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 84);
                Debug.Assert(pack.param_value_TRY(ph).Equals("jrQgnFwjvryyixspdzmkgujgghbaazciezsXFagxhmypbmUnfdgwhgYbryzZzfgtafbhmbkMffxfjyjadazv"));
                Debug.Assert(pack.target_component == (byte)(byte)253);
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("ndqxdepv"));
                Debug.Assert(pack.target_system == (byte)(byte)35);
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)35;
            p323.target_component = (byte)(byte)253;
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            p323.param_value_SET("jrQgnFwjvryyixspdzmkgujgghbaazciezsXFagxhmypbmUnfdgwhgYbryzZzfgtafbhmbkMffxfjyjadazv", PH) ;
            p323.param_id_SET("ndqxdepv", PH) ;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_result == PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("gjcqm"));
                Debug.Assert(pack.param_value_LEN(ph) == 50);
                Debug.Assert(pack.param_value_TRY(ph).Equals("ozipsFpoupmnlempJgsukkkhxrzphzzwtwZaRtwmtqTyvahmmr"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_result = PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED;
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            p324.param_value_SET("ozipsFpoupmnlempJgsukkkhxrzphzzwtwZaRtwmtqTyvahmmr", PH) ;
            p324.param_id_SET("gjcqm", PH) ;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)5197720308631204762L);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)55741, (ushort)45577, (ushort)18599, (ushort)7546, (ushort)7623, (ushort)55960, (ushort)49039, (ushort)33770, (ushort)21651, (ushort)5879, (ushort)6047, (ushort)3552, (ushort)11679, (ushort)49973, (ushort)28231, (ushort)46919, (ushort)27841, (ushort)54929, (ushort)13824, (ushort)4824, (ushort)60949, (ushort)59269, (ushort)13584, (ushort)48067, (ushort)40555, (ushort)19094, (ushort)32367, (ushort)11001, (ushort)44043, (ushort)5458, (ushort)38215, (ushort)34120, (ushort)47681, (ushort)35836, (ushort)45691, (ushort)23923, (ushort)44837, (ushort)31311, (ushort)53434, (ushort)64985, (ushort)62537, (ushort)11506, (ushort)5981, (ushort)60638, (ushort)60551, (ushort)64410, (ushort)49663, (ushort)55995, (ushort)23140, (ushort)58701, (ushort)33908, (ushort)46911, (ushort)9375, (ushort)23868, (ushort)50395, (ushort)55966, (ushort)36474, (ushort)58849, (ushort)26720, (ushort)43020, (ushort)26305, (ushort)49509, (ushort)13117, (ushort)56516, (ushort)30919, (ushort)4689, (ushort)45226, (ushort)28110, (ushort)59799, (ushort)55302, (ushort)13968, (ushort)19811}));
                Debug.Assert(pack.increment == (byte)(byte)236);
                Debug.Assert(pack.min_distance == (ushort)(ushort)38119);
                Debug.Assert(pack.sensor_type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
                Debug.Assert(pack.max_distance == (ushort)(ushort)55481);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.distances_SET(new ushort[] {(ushort)55741, (ushort)45577, (ushort)18599, (ushort)7546, (ushort)7623, (ushort)55960, (ushort)49039, (ushort)33770, (ushort)21651, (ushort)5879, (ushort)6047, (ushort)3552, (ushort)11679, (ushort)49973, (ushort)28231, (ushort)46919, (ushort)27841, (ushort)54929, (ushort)13824, (ushort)4824, (ushort)60949, (ushort)59269, (ushort)13584, (ushort)48067, (ushort)40555, (ushort)19094, (ushort)32367, (ushort)11001, (ushort)44043, (ushort)5458, (ushort)38215, (ushort)34120, (ushort)47681, (ushort)35836, (ushort)45691, (ushort)23923, (ushort)44837, (ushort)31311, (ushort)53434, (ushort)64985, (ushort)62537, (ushort)11506, (ushort)5981, (ushort)60638, (ushort)60551, (ushort)64410, (ushort)49663, (ushort)55995, (ushort)23140, (ushort)58701, (ushort)33908, (ushort)46911, (ushort)9375, (ushort)23868, (ushort)50395, (ushort)55966, (ushort)36474, (ushort)58849, (ushort)26720, (ushort)43020, (ushort)26305, (ushort)49509, (ushort)13117, (ushort)56516, (ushort)30919, (ushort)4689, (ushort)45226, (ushort)28110, (ushort)59799, (ushort)55302, (ushort)13968, (ushort)19811}, 0) ;
            p330.time_usec = (ulong)5197720308631204762L;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p330.increment = (byte)(byte)236;
            p330.max_distance = (ushort)(ushort)55481;
            p330.min_distance = (ushort)(ushort)38119;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
        }
    }
}