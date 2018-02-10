
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
                    ulong id = id__N(value);
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
                        case MAV_CMD.MAV_CMD_RESET_MPPT:
                            id = 127;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL:
                            id = 128;
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
                        case MAV_CMD.MAV_CMD_RESET_MPPT:
                            id = 127;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL:
                            id = 128;
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
                        case MAV_CMD.MAV_CMD_RESET_MPPT:
                            id = 127;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL:
                            id = 128;
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
                        case MAV_CMD.MAV_CMD_RESET_MPPT:
                            id = 127;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL:
                            id = 128;
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
                        case MAV_CMD.MAV_CMD_RESET_MPPT:
                            id = 127;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL:
                            id = 128;
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
        new class SENS_POWER : GroundControl.SENS_POWER
        {
            public float adc121_vspb_volt //Power board voltage sensor reading in volts
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float adc121_cspb_amp //Power board current sensor reading in amps
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float adc121_cs1_amp //Board current sensor 1 reading in amps
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float adc121_cs2_amp //Board current sensor 2 reading in amps
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }
        }
        new class SENS_MPPT : GroundControl.SENS_MPPT
        {
            public ushort mppt1_pwm //MPPT1 pwm
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort mppt2_pwm //MPPT2 pwm
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort mppt3_pwm //MPPT3 pwm
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public ulong mppt_timestamp //MPPT last timestamp
            {
                get {  return (BitUtils.get_bytes(data,  6, 8));}
            }

            public float mppt1_volt //MPPT1 voltage
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
            }

            public float mppt1_amp //MPPT1 current
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
            }

            public byte mppt1_status //MPPT1 status
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  22, 1));}
            }

            public float mppt2_volt //MPPT2 voltage
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  23, 4)));}
            }

            public float mppt2_amp //MPPT2 current
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  27, 4)));}
            }

            public byte mppt2_status //MPPT2 status
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  31, 1));}
            }

            public float mppt3_volt //MPPT3 voltage
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float mppt3_amp //MPPT3 current
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
            }

            public byte mppt3_status //MPPT3 status
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  40, 1));}
            }
        }
        new class ASLCTRL_DATA : GroundControl.ASLCTRL_DATA
        {
            public ulong timestamp //Timestamp
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public byte aslctrl_mode //ASLCTRL control-mode (manual, stabilized, auto, etc...)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public float h //See sourcecode for a description of these values...
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  9, 4)));}
            }

            public float hRef //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  13, 4)));}
            }

            public float hRef_t //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  17, 4)));}
            }

            public float PitchAngle //Pitch angle [deg]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  21, 4)));}
            }

            public float PitchAngleRef //Pitch angle reference[deg]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  25, 4)));}
            }

            public float q //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  29, 4)));}
            }

            public float qRef //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  33, 4)));}
            }

            public float uElev //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  37, 4)));}
            }

            public float uThrot //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  41, 4)));}
            }

            public float uThrot2 //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  45, 4)));}
            }

            public float nZ //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  49, 4)));}
            }

            public float AirspeedRef //Airspeed reference [m/s]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  53, 4)));}
            }

            public byte SpoilersEngaged //null
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  57, 1));}
            }

            public float YawAngle //Yaw angle [deg]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  58, 4)));}
            }

            public float YawAngleRef //Yaw angle reference[deg]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  62, 4)));}
            }

            public float RollAngle //Roll angle [deg]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  66, 4)));}
            }

            public float RollAngleRef //Roll angle reference[deg]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  70, 4)));}
            }

            public float p //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  74, 4)));}
            }

            public float pRef //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  78, 4)));}
            }

            public float r //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  82, 4)));}
            }

            public float rRef //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  86, 4)));}
            }

            public float uAil //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  90, 4)));}
            }

            public float uRud //null
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  94, 4)));}
            }
        }
        new class ASLCTRL_DEBUG : GroundControl.ASLCTRL_DEBUG
        {
            public uint i32_1 //Debug data
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
            }

            public byte i8_1 //Debug data
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte i8_2 //Debug data
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public float f_1 //Debug data
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
            }

            public float f_2 //Debug data
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
            }

            public float f_3 //Debug data
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
            }

            public float f_4 //Debug data
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
            }

            public float f_5 //Debug data
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
            }

            public float f_6 //Debug data
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  26, 4)));}
            }

            public float f_7 //Debug data
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  30, 4)));}
            }

            public float f_8 //Debug data
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  34, 4)));}
            }
        }
        new class ASLUAV_STATUS : GroundControl.ASLUAV_STATUS
        {
            public byte LED_status //Status of the position-indicator LEDs
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte SATCOM_status //Status of the IRIDIUM satellite communication system
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte[] Servo_status //Status vector for up to 8 servos
            {
                get {return Servo_status_GET(new byte[8], 0);}
            }
            public byte[]Servo_status_GET(byte[] dst_ch, int pos)  //Status vector for up to 8 servos
            {
                for(int BYTE = 2, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public float Motor_rpm //Motor RPM
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
            }
        }
        new class EKF_EXT : GroundControl.EKF_EXT
        {
            public ulong timestamp //Time since system start [us]
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public float Windspeed //Magnitude of wind velocity (in lateral inertial plane) [m/s]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float WindDir //Wind heading angle from North [rad]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float WindZ //Z (Down) component of inertial wind velocity [m/s]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float Airspeed //Magnitude of air velocity [m/s]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float beta //Sideslip angle [rad]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float alpha //Angle of attack [rad]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }
        }
        new class ASL_OBCTRL : GroundControl.ASL_OBCTRL
        {
            public ulong timestamp //Time since system start [us]
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public float uElev //Elevator command [~]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float uThrot //Throttle command [~]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float uThrot2 //Throttle 2 command [~]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float uAilL //Left aileron command [~]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float uAilR //Right aileron command [~]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float uRud //Rudder command [~]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public byte obctrl_status //Off-board computer status
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  32, 1));}
            }
        }
        new class SENS_ATMOS : GroundControl.SENS_ATMOS
        {
            public float TempAmbient //Ambient temperature [degrees Celsius]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float Humidity //Relative humidity [%]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }
        }
        new class SENS_BATMON : GroundControl.SENS_BATMON
        {
            public ushort voltage //Battery pack voltage in [mV]
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort batterystatus //Battery monitor status report bits in Hex
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public ushort serialnumber //Battery monitor serial number in Hex
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  4, 2));}
            }

            public ushort hostfetcontrol //Battery monitor sensor host FET control in Hex
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  6, 2));}
            }

            public ushort cellvoltage1 //Battery pack cell 1 voltage in [mV]
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  8, 2));}
            }

            public ushort cellvoltage2 //Battery pack cell 2 voltage in [mV]
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  10, 2));}
            }

            public ushort cellvoltage3 //Battery pack cell 3 voltage in [mV]
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  12, 2));}
            }

            public ushort cellvoltage4 //Battery pack cell 4 voltage in [mV]
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  14, 2));}
            }

            public ushort cellvoltage5 //Battery pack cell 5 voltage in [mV]
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  16, 2));}
            }

            public ushort cellvoltage6 //Battery pack cell 6 voltage in [mV]
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  18, 2));}
            }

            public float temperature //Battery pack temperature in [deg C]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public short current //Battery pack current in [mA]
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  24, 2));}
            }

            public byte SoC //Battery pack state-of-charge
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  26, 1));}
            }
        }
        new class FW_SOARING_DATA : GroundControl.FW_SOARING_DATA
        {
            public ulong timestamp //Timestamp [ms]
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public ulong timestampModeChanged //Timestamp since last mode change[ms]
            {
                get {  return (BitUtils.get_bytes(data,  8, 8));}
            }

            public float xW //Thermal core updraft strength [m/s]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float xR //Thermal radius [m]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float xLat //Thermal center latitude [deg]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float xLon //Thermal center longitude [deg]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }

            public float VarW //Variance W
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  32, 4)));}
            }

            public float VarR //Variance R
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  36, 4)));}
            }

            public float VarLat //Variance Lat
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
            }

            public float VarLon //Variance Lon
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
            }

            public float LoiterRadius //Suggested loiter radius [m]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  48, 4)));}
            }

            public float LoiterDirection //Suggested loiter direction
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  52, 4)));}
            }

            public float DistToSoarPoint //Distance to soar point [m]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  56, 4)));}
            }

            public float vSinkExp //Expected sink rate at current airspeed, roll and throttle [m/s]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  60, 4)));}
            }

            public float z1_LocalUpdraftSpeed //Measurement / updraft speed at current/local airplane position [m/s]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  64, 4)));}
            }

            public float z2_DeltaRoll //Measurement / roll angle tracking error [deg]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  68, 4)));}
            }

            public float z1_exp //Expected measurement 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  72, 4)));}
            }

            public float z2_exp //Expected measurement 2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  76, 4)));}
            }

            public float ThermalGSNorth //Thermal drift (from estimator prediction step only) [m/s]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  80, 4)));}
            }

            public float ThermalGSEast //Thermal drift (from estimator prediction step only) [m/s]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  84, 4)));}
            }

            public float TSE_dot //Total specific energy change (filtered) [m/s]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  88, 4)));}
            }

            public float DebugVar1 //Debug variable 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  92, 4)));}
            }

            public float DebugVar2 //Debug variable 2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  96, 4)));}
            }

            public byte ControlMode //Control Mode [-]
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  100, 1));}
            }

            public byte valid //Data valid [-]
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  101, 1));}
            }
        }
        new class SENSORPOD_STATUS : GroundControl.SENSORPOD_STATUS
        {
            public ushort free_space //Free space available in recordings directory in [Gb] * 1e2
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ulong timestamp //Timestamp in linuxtime [ms] (since 1.1.1970)
            {
                get {  return (BitUtils.get_bytes(data,  2, 8));}
            }

            public byte visensor_rate_1 //Rate of ROS topic 1
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
            }

            public byte visensor_rate_2 //Rate of ROS topic 2
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  11, 1));}
            }

            public byte visensor_rate_3 //Rate of ROS topic 3
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  12, 1));}
            }

            public byte visensor_rate_4 //Rate of ROS topic 4
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  13, 1));}
            }

            public byte recording_nodes_count //Number of recording nodes
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  14, 1));}
            }

            public byte cpu_temp //Temperature of sensorpod CPU in [deg C]
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  15, 1));}
            }
        }
        new class SENS_POWER_BOARD : GroundControl.SENS_POWER_BOARD
        {
            public ulong timestamp //Timestamp
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public byte pwr_brd_status //Power board status register
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public byte pwr_brd_led_status //Power board leds status
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
            }

            public float pwr_brd_system_volt //Power board system voltage
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
            }

            public float pwr_brd_servo_volt //Power board servo voltage
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
            }

            public float pwr_brd_mot_l_amp //Power board left motor current sensor
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
            }

            public float pwr_brd_mot_r_amp //Power board right motor current sensor
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
            }

            public float pwr_brd_servo_1_amp //Power board servo1 current sensor
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  26, 4)));}
            }

            public float pwr_brd_servo_2_amp //Power board servo1 current sensor
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  30, 4)));}
            }

            public float pwr_brd_servo_3_amp //Power board servo1 current sensor
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  34, 4)));}
            }

            public float pwr_brd_servo_4_amp //Power board servo1 current sensor
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  38, 4)));}
            }

            public float pwr_brd_aux_amp //Power board aux current sensor
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  42, 4)));}
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

            public void OnAUTOPILOT_VERSIONReceive_direct(Channel src, Inside ph, AUTOPILOT_VERSION pack) {OnAUTOPILOT_VERSIONReceive(this, ph,  pack);}
            public event AUTOPILOT_VERSIONReceiveHandler OnAUTOPILOT_VERSIONReceive;
            public delegate void AUTOPILOT_VERSIONReceiveHandler(Channel src, Inside ph, AUTOPILOT_VERSION pack);
            public void OnLANDING_TARGETReceive_direct(Channel src, Inside ph, LANDING_TARGET pack) {OnLANDING_TARGETReceive(this, ph,  pack);}
            public event LANDING_TARGETReceiveHandler OnLANDING_TARGETReceive;
            public delegate void LANDING_TARGETReceiveHandler(Channel src, Inside ph, LANDING_TARGET pack);
            public void OnSENS_POWERReceive_direct(Channel src, Inside ph, SENS_POWER pack) {OnSENS_POWERReceive(this, ph,  pack);}
            public event SENS_POWERReceiveHandler OnSENS_POWERReceive;
            public delegate void SENS_POWERReceiveHandler(Channel src, Inside ph, SENS_POWER pack);
            public void OnSENS_MPPTReceive_direct(Channel src, Inside ph, SENS_MPPT pack) {OnSENS_MPPTReceive(this, ph,  pack);}
            public event SENS_MPPTReceiveHandler OnSENS_MPPTReceive;
            public delegate void SENS_MPPTReceiveHandler(Channel src, Inside ph, SENS_MPPT pack);
            public void OnASLCTRL_DATAReceive_direct(Channel src, Inside ph, ASLCTRL_DATA pack) {OnASLCTRL_DATAReceive(this, ph,  pack);}
            public event ASLCTRL_DATAReceiveHandler OnASLCTRL_DATAReceive;
            public delegate void ASLCTRL_DATAReceiveHandler(Channel src, Inside ph, ASLCTRL_DATA pack);
            public void OnASLCTRL_DEBUGReceive_direct(Channel src, Inside ph, ASLCTRL_DEBUG pack) {OnASLCTRL_DEBUGReceive(this, ph,  pack);}
            public event ASLCTRL_DEBUGReceiveHandler OnASLCTRL_DEBUGReceive;
            public delegate void ASLCTRL_DEBUGReceiveHandler(Channel src, Inside ph, ASLCTRL_DEBUG pack);
            public void OnASLUAV_STATUSReceive_direct(Channel src, Inside ph, ASLUAV_STATUS pack) {OnASLUAV_STATUSReceive(this, ph,  pack);}
            public event ASLUAV_STATUSReceiveHandler OnASLUAV_STATUSReceive;
            public delegate void ASLUAV_STATUSReceiveHandler(Channel src, Inside ph, ASLUAV_STATUS pack);
            public void OnEKF_EXTReceive_direct(Channel src, Inside ph, EKF_EXT pack) {OnEKF_EXTReceive(this, ph,  pack);}
            public event EKF_EXTReceiveHandler OnEKF_EXTReceive;
            public delegate void EKF_EXTReceiveHandler(Channel src, Inside ph, EKF_EXT pack);
            public void OnASL_OBCTRLReceive_direct(Channel src, Inside ph, ASL_OBCTRL pack) {OnASL_OBCTRLReceive(this, ph,  pack);}
            public event ASL_OBCTRLReceiveHandler OnASL_OBCTRLReceive;
            public delegate void ASL_OBCTRLReceiveHandler(Channel src, Inside ph, ASL_OBCTRL pack);
            public void OnSENS_ATMOSReceive_direct(Channel src, Inside ph, SENS_ATMOS pack) {OnSENS_ATMOSReceive(this, ph,  pack);}
            public event SENS_ATMOSReceiveHandler OnSENS_ATMOSReceive;
            public delegate void SENS_ATMOSReceiveHandler(Channel src, Inside ph, SENS_ATMOS pack);
            public void OnSENS_BATMONReceive_direct(Channel src, Inside ph, SENS_BATMON pack) {OnSENS_BATMONReceive(this, ph,  pack);}
            public event SENS_BATMONReceiveHandler OnSENS_BATMONReceive;
            public delegate void SENS_BATMONReceiveHandler(Channel src, Inside ph, SENS_BATMON pack);
            public void OnFW_SOARING_DATAReceive_direct(Channel src, Inside ph, FW_SOARING_DATA pack) {OnFW_SOARING_DATAReceive(this, ph,  pack);}
            public event FW_SOARING_DATAReceiveHandler OnFW_SOARING_DATAReceive;
            public delegate void FW_SOARING_DATAReceiveHandler(Channel src, Inside ph, FW_SOARING_DATA pack);
            public void OnSENSORPOD_STATUSReceive_direct(Channel src, Inside ph, SENSORPOD_STATUS pack) {OnSENSORPOD_STATUSReceive(this, ph,  pack);}
            public event SENSORPOD_STATUSReceiveHandler OnSENSORPOD_STATUSReceive;
            public delegate void SENSORPOD_STATUSReceiveHandler(Channel src, Inside ph, SENSORPOD_STATUS pack);
            public void OnSENS_POWER_BOARDReceive_direct(Channel src, Inside ph, SENS_POWER_BOARD pack) {OnSENS_POWER_BOARDReceive(this, ph,  pack);}
            public event SENS_POWER_BOARDReceiveHandler OnSENS_POWER_BOARDReceive;
            public delegate void SENS_POWER_BOARDReceiveHandler(Channel src, Inside ph, SENS_POWER_BOARD pack);
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
                    case 76:
                        if(pack == null) return new COMMAND_LONG();
                        break;
                    case 77:
                        if(pack == null) return new COMMAND_ACK();
                        break;
                    case 81:
                        if(pack == null) return new MANUAL_SETPOINT();
                        break;
                    case 148:
                        if(pack == null) return new AUTOPILOT_VERSION();
                        OnAUTOPILOT_VERSIONReceive(this, ph, (AUTOPILOT_VERSION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 149:
                        if(pack == null) return new LANDING_TARGET();
                        OnLANDING_TARGETReceive(this, ph, (LANDING_TARGET) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 201:
                        if(pack == null) return new SENS_POWER();
                        OnSENS_POWERReceive(this, ph, (SENS_POWER) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 202:
                        if(pack == null) return new SENS_MPPT();
                        OnSENS_MPPTReceive(this, ph, (SENS_MPPT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 203:
                        if(pack == null) return new ASLCTRL_DATA();
                        OnASLCTRL_DATAReceive(this, ph, (ASLCTRL_DATA) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 204:
                        if(pack == null) return new ASLCTRL_DEBUG();
                        OnASLCTRL_DEBUGReceive(this, ph, (ASLCTRL_DEBUG) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 205:
                        if(pack == null) return new ASLUAV_STATUS();
                        OnASLUAV_STATUSReceive(this, ph, (ASLUAV_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 206:
                        if(pack == null) return new EKF_EXT();
                        OnEKF_EXTReceive(this, ph, (EKF_EXT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 207:
                        if(pack == null) return new ASL_OBCTRL();
                        OnASL_OBCTRLReceive(this, ph, (ASL_OBCTRL) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 208:
                        if(pack == null) return new SENS_ATMOS();
                        OnSENS_ATMOSReceive(this, ph, (SENS_ATMOS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 209:
                        if(pack == null) return new SENS_BATMON();
                        OnSENS_BATMONReceive(this, ph, (SENS_BATMON) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 210:
                        if(pack == null) return new FW_SOARING_DATA();
                        OnFW_SOARING_DATAReceive(this, ph, (FW_SOARING_DATA) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 211:
                        if(pack == null) return new SENSORPOD_STATUS();
                        OnSENSORPOD_STATUSReceive(this, ph, (SENSORPOD_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 212:
                        if(pack == null) return new SENS_POWER_BOARD();
                        OnSENS_POWER_BOARDReceive(this, ph, (SENS_POWER_BOARD) pack);//no any host channels can receive this pack. Handle it with test channel handler
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
                Debug.Assert(pack.type == MAV_TYPE.MAV_TYPE_VTOL_RESERVED2);
                Debug.Assert(pack.custom_mode == (uint)1145710965U);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED));
                Debug.Assert(pack.autopilot == MAV_AUTOPILOT.MAV_AUTOPILOT_OPENPILOT);
                Debug.Assert(pack.mavlink_version == (byte)(byte)254);
                Debug.Assert(pack.system_status == MAV_STATE.MAV_STATE_CRITICAL);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.mavlink_version = (byte)(byte)254;
            p0.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_OPENPILOT;
            p0.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED);
            p0.system_status = MAV_STATE.MAV_STATE_CRITICAL;
            p0.custom_mode = (uint)1145710965U;
            p0.type = MAV_TYPE.MAV_TYPE_VTOL_RESERVED2;
            ADV_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)41042);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW));
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 49);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)9045);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION));
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)3610);
                Debug.Assert(pack.current_battery == (short)(short)29830);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)10090);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)12981);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
                Debug.Assert(pack.errors_comm == (ushort)(ushort)18768);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)9653);
                Debug.Assert(pack.load == (ushort)(ushort)27193);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.errors_count4 = (ushort)(ushort)10090;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL);
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW);
            p1.load = (ushort)(ushort)27193;
            p1.voltage_battery = (ushort)(ushort)12981;
            p1.errors_count2 = (ushort)(ushort)9653;
            p1.current_battery = (short)(short)29830;
            p1.errors_count1 = (ushort)(ushort)41042;
            p1.battery_remaining = (sbyte)(sbyte) - 49;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION);
            p1.drop_rate_comm = (ushort)(ushort)9045;
            p1.errors_comm = (ushort)(ushort)18768;
            p1.errors_count3 = (ushort)(ushort)3610;
            ADV_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)9180574165099708802L);
                Debug.Assert(pack.time_boot_ms == (uint)660066827U);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_boot_ms = (uint)660066827U;
            p2.time_unix_usec = (ulong)9180574165099708802L;
            ADV_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float) -6.274386E37F);
                Debug.Assert(pack.yaw_rate == (float)2.8394477E38F);
                Debug.Assert(pack.z == (float) -1.1255473E38F);
                Debug.Assert(pack.afz == (float) -1.7136867E38F);
                Debug.Assert(pack.x == (float) -6.044187E37F);
                Debug.Assert(pack.afy == (float)7.109599E37F);
                Debug.Assert(pack.yaw == (float)1.5908656E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.type_mask == (ushort)(ushort)30987);
                Debug.Assert(pack.vz == (float)5.7705167E35F);
                Debug.Assert(pack.afx == (float) -3.0570654E38F);
                Debug.Assert(pack.y == (float)1.5548077E37F);
                Debug.Assert(pack.time_boot_ms == (uint)4018950585U);
                Debug.Assert(pack.vy == (float) -4.664156E36F);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.z = (float) -1.1255473E38F;
            p3.afz = (float) -1.7136867E38F;
            p3.yaw = (float)1.5908656E38F;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p3.x = (float) -6.044187E37F;
            p3.vx = (float) -6.274386E37F;
            p3.afx = (float) -3.0570654E38F;
            p3.vy = (float) -4.664156E36F;
            p3.time_boot_ms = (uint)4018950585U;
            p3.vz = (float)5.7705167E35F;
            p3.y = (float)1.5548077E37F;
            p3.afy = (float)7.109599E37F;
            p3.type_mask = (ushort)(ushort)30987;
            p3.yaw_rate = (float)2.8394477E38F;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)799443257U);
                Debug.Assert(pack.time_usec == (ulong)6723815129884433875L);
                Debug.Assert(pack.target_component == (byte)(byte)171);
                Debug.Assert(pack.target_system == (byte)(byte)145);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.target_component = (byte)(byte)171;
            p4.seq = (uint)799443257U;
            p4.time_usec = (ulong)6723815129884433875L;
            p4.target_system = (byte)(byte)145;
            ADV_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.version == (byte)(byte)176);
                Debug.Assert(pack.target_system == (byte)(byte)78);
                Debug.Assert(pack.control_request == (byte)(byte)223);
                Debug.Assert(pack.passkey_LEN(ph) == 19);
                Debug.Assert(pack.passkey_TRY(ph).Equals("pzcwiomnbtZeKJwjbqe"));
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.control_request = (byte)(byte)223;
            p5.passkey_SET("pzcwiomnbtZeKJwjbqe", PH) ;
            p5.target_system = (byte)(byte)78;
            p5.version = (byte)(byte)176;
            ADV_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.control_request == (byte)(byte)212);
                Debug.Assert(pack.ack == (byte)(byte)160);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)123);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.ack = (byte)(byte)160;
            p6.gcs_system_id = (byte)(byte)123;
            p6.control_request = (byte)(byte)212;
            ADV_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 9);
                Debug.Assert(pack.key_TRY(ph).Equals("wxiipicim"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("wxiipicim", PH) ;
            ADV_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.custom_mode == (uint)2829076812U);
                Debug.Assert(pack.target_system == (byte)(byte)183);
                Debug.Assert(pack.base_mode == MAV_MODE.MAV_MODE_AUTO_ARMED);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)183;
            p11.custom_mode = (uint)2829076812U;
            p11.base_mode = MAV_MODE.MAV_MODE_AUTO_ARMED;
            ADV_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("m"));
                Debug.Assert(pack.target_system == (byte)(byte)199);
                Debug.Assert(pack.target_component == (byte)(byte)187);
                Debug.Assert(pack.param_index == (short)(short) -18672);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_component = (byte)(byte)187;
            p20.param_index = (short)(short) -18672;
            p20.target_system = (byte)(byte)199;
            p20.param_id_SET("m", PH) ;
            ADV_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)237);
                Debug.Assert(pack.target_system == (byte)(byte)194);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)194;
            p21.target_component = (byte)(byte)237;
            ADV_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value == (float)1.9001721E38F);
                Debug.Assert(pack.param_count == (ushort)(ushort)45452);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
                Debug.Assert(pack.param_index == (ushort)(ushort)47079);
                Debug.Assert(pack.param_id_LEN(ph) == 2);
                Debug.Assert(pack.param_id_TRY(ph).Equals("Hu"));
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16;
            p22.param_id_SET("Hu", PH) ;
            p22.param_value = (float)1.9001721E38F;
            p22.param_index = (ushort)(ushort)47079;
            p22.param_count = (ushort)(ushort)45452;
            ADV_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value == (float)1.3053551E38F);
                Debug.Assert(pack.target_component == (byte)(byte)21);
                Debug.Assert(pack.target_system == (byte)(byte)254);
                Debug.Assert(pack.param_id_LEN(ph) == 13);
                Debug.Assert(pack.param_id_TRY(ph).Equals("oziUijrizoFvq"));
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.param_value = (float)1.3053551E38F;
            p23.target_component = (byte)(byte)21;
            p23.param_id_SET("oziUijrizoFvq", PH) ;
            p23.target_system = (byte)(byte)254;
            p23.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8;
            ADV_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)1594228512595066036L);
                Debug.Assert(pack.eph == (ushort)(ushort)63005);
                Debug.Assert(pack.cog == (ushort)(ushort)40132);
                Debug.Assert(pack.lat == (int)1938545581);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)2655668328U);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int)1171514355);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)2069671203U);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)1166895895U);
                Debug.Assert(pack.alt == (int) -147124228);
                Debug.Assert(pack.satellites_visible == (byte)(byte)192);
                Debug.Assert(pack.epv == (ushort)(ushort)58415);
                Debug.Assert(pack.lon == (int)1541592925);
                Debug.Assert(pack.vel == (ushort)(ushort)59294);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)1751814014U);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.alt_ellipsoid_SET((int)1171514355, PH) ;
            p24.lon = (int)1541592925;
            p24.hdg_acc_SET((uint)1166895895U, PH) ;
            p24.lat = (int)1938545581;
            p24.epv = (ushort)(ushort)58415;
            p24.time_usec = (ulong)1594228512595066036L;
            p24.alt = (int) -147124228;
            p24.v_acc_SET((uint)2069671203U, PH) ;
            p24.cog = (ushort)(ushort)40132;
            p24.h_acc_SET((uint)1751814014U, PH) ;
            p24.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p24.eph = (ushort)(ushort)63005;
            p24.vel = (ushort)(ushort)59294;
            p24.vel_acc_SET((uint)2655668328U, PH) ;
            p24.satellites_visible = (byte)(byte)192;
            ADV_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)122, (byte)143, (byte)44, (byte)153, (byte)143, (byte)10, (byte)50, (byte)41, (byte)109, (byte)220, (byte)149, (byte)243, (byte)82, (byte)18, (byte)157, (byte)54, (byte)169, (byte)205, (byte)194, (byte)93}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)250, (byte)191, (byte)173, (byte)139, (byte)186, (byte)199, (byte)105, (byte)32, (byte)184, (byte)240, (byte)193, (byte)187, (byte)204, (byte)47, (byte)180, (byte)71, (byte)225, (byte)232, (byte)61, (byte)240}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)128, (byte)105, (byte)153, (byte)247, (byte)146, (byte)165, (byte)208, (byte)221, (byte)156, (byte)93, (byte)237, (byte)70, (byte)88, (byte)46, (byte)96, (byte)153, (byte)145, (byte)252, (byte)77, (byte)193}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)87, (byte)178, (byte)20, (byte)57, (byte)231, (byte)94, (byte)14, (byte)116, (byte)183, (byte)65, (byte)98, (byte)66, (byte)164, (byte)176, (byte)195, (byte)143, (byte)38, (byte)20, (byte)66, (byte)81}));
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)158, (byte)207, (byte)250, (byte)192, (byte)86, (byte)167, (byte)246, (byte)88, (byte)144, (byte)232, (byte)228, (byte)196, (byte)210, (byte)106, (byte)62, (byte)163, (byte)211, (byte)136, (byte)181, (byte)117}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)28);
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_snr_SET(new byte[] {(byte)122, (byte)143, (byte)44, (byte)153, (byte)143, (byte)10, (byte)50, (byte)41, (byte)109, (byte)220, (byte)149, (byte)243, (byte)82, (byte)18, (byte)157, (byte)54, (byte)169, (byte)205, (byte)194, (byte)93}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)87, (byte)178, (byte)20, (byte)57, (byte)231, (byte)94, (byte)14, (byte)116, (byte)183, (byte)65, (byte)98, (byte)66, (byte)164, (byte)176, (byte)195, (byte)143, (byte)38, (byte)20, (byte)66, (byte)81}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)128, (byte)105, (byte)153, (byte)247, (byte)146, (byte)165, (byte)208, (byte)221, (byte)156, (byte)93, (byte)237, (byte)70, (byte)88, (byte)46, (byte)96, (byte)153, (byte)145, (byte)252, (byte)77, (byte)193}, 0) ;
            p25.satellites_visible = (byte)(byte)28;
            p25.satellite_used_SET(new byte[] {(byte)250, (byte)191, (byte)173, (byte)139, (byte)186, (byte)199, (byte)105, (byte)32, (byte)184, (byte)240, (byte)193, (byte)187, (byte)204, (byte)47, (byte)180, (byte)71, (byte)225, (byte)232, (byte)61, (byte)240}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)158, (byte)207, (byte)250, (byte)192, (byte)86, (byte)167, (byte)246, (byte)88, (byte)144, (byte)232, (byte)228, (byte)196, (byte)210, (byte)106, (byte)62, (byte)163, (byte)211, (byte)136, (byte)181, (byte)117}, 0) ;
            ADV_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)898089670U);
                Debug.Assert(pack.yacc == (short)(short)14260);
                Debug.Assert(pack.xacc == (short)(short) -14538);
                Debug.Assert(pack.ygyro == (short)(short)25061);
                Debug.Assert(pack.xmag == (short)(short)19756);
                Debug.Assert(pack.xgyro == (short)(short) -1806);
                Debug.Assert(pack.zmag == (short)(short) -24072);
                Debug.Assert(pack.zacc == (short)(short) -32663);
                Debug.Assert(pack.zgyro == (short)(short)17042);
                Debug.Assert(pack.ymag == (short)(short) -1512);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.ymag = (short)(short) -1512;
            p26.zmag = (short)(short) -24072;
            p26.xgyro = (short)(short) -1806;
            p26.time_boot_ms = (uint)898089670U;
            p26.xmag = (short)(short)19756;
            p26.zacc = (short)(short) -32663;
            p26.ygyro = (short)(short)25061;
            p26.xacc = (short)(short) -14538;
            p26.yacc = (short)(short)14260;
            p26.zgyro = (short)(short)17042;
            ADV_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short)3896);
                Debug.Assert(pack.yacc == (short)(short)31076);
                Debug.Assert(pack.xgyro == (short)(short) -20620);
                Debug.Assert(pack.xmag == (short)(short)2048);
                Debug.Assert(pack.zmag == (short)(short) -25851);
                Debug.Assert(pack.zacc == (short)(short)28019);
                Debug.Assert(pack.ymag == (short)(short)11466);
                Debug.Assert(pack.ygyro == (short)(short) -24045);
                Debug.Assert(pack.zgyro == (short)(short)19295);
                Debug.Assert(pack.time_usec == (ulong)78869929020673366L);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.zacc = (short)(short)28019;
            p27.time_usec = (ulong)78869929020673366L;
            p27.zgyro = (short)(short)19295;
            p27.ymag = (short)(short)11466;
            p27.yacc = (short)(short)31076;
            p27.zmag = (short)(short) -25851;
            p27.xacc = (short)(short)3896;
            p27.xmag = (short)(short)2048;
            p27.ygyro = (short)(short) -24045;
            p27.xgyro = (short)(short) -20620;
            ADV_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff1 == (short)(short)17218);
                Debug.Assert(pack.press_diff2 == (short)(short)31745);
                Debug.Assert(pack.time_usec == (ulong)4584903027903886706L);
                Debug.Assert(pack.press_abs == (short)(short) -20653);
                Debug.Assert(pack.temperature == (short)(short)9446);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_diff1 = (short)(short)17218;
            p28.temperature = (short)(short)9446;
            p28.time_usec = (ulong)4584903027903886706L;
            p28.press_abs = (short)(short) -20653;
            p28.press_diff2 = (short)(short)31745;
            ADV_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float) -2.3915549E38F);
                Debug.Assert(pack.time_boot_ms == (uint)196049523U);
                Debug.Assert(pack.press_diff == (float) -3.2261566E38F);
                Debug.Assert(pack.temperature == (short)(short)11414);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.temperature = (short)(short)11414;
            p29.press_abs = (float) -2.3915549E38F;
            p29.time_boot_ms = (uint)196049523U;
            p29.press_diff = (float) -3.2261566E38F;
            ADV_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -3.2892598E38F);
                Debug.Assert(pack.yaw == (float)3.1705732E38F);
                Debug.Assert(pack.pitchspeed == (float) -3.469917E37F);
                Debug.Assert(pack.roll == (float)3.2259698E38F);
                Debug.Assert(pack.yawspeed == (float) -1.6974837E38F);
                Debug.Assert(pack.rollspeed == (float)1.2686789E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1696925984U);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.roll = (float)3.2259698E38F;
            p30.pitch = (float) -3.2892598E38F;
            p30.yawspeed = (float) -1.6974837E38F;
            p30.yaw = (float)3.1705732E38F;
            p30.time_boot_ms = (uint)1696925984U;
            p30.pitchspeed = (float) -3.469917E37F;
            p30.rollspeed = (float)1.2686789E38F;
            ADV_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float)2.2780219E38F);
                Debug.Assert(pack.q3 == (float)2.9290513E38F);
                Debug.Assert(pack.yawspeed == (float) -3.8405632E37F);
                Debug.Assert(pack.q2 == (float) -3.393823E38F);
                Debug.Assert(pack.q4 == (float)2.520517E38F);
                Debug.Assert(pack.pitchspeed == (float) -1.4405198E38F);
                Debug.Assert(pack.q1 == (float) -2.2026748E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3723381258U);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.q1 = (float) -2.2026748E38F;
            p31.rollspeed = (float)2.2780219E38F;
            p31.time_boot_ms = (uint)3723381258U;
            p31.q4 = (float)2.520517E38F;
            p31.q2 = (float) -3.393823E38F;
            p31.pitchspeed = (float) -1.4405198E38F;
            p31.yawspeed = (float) -3.8405632E37F;
            p31.q3 = (float)2.9290513E38F;
            ADV_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float)7.481952E35F);
                Debug.Assert(pack.y == (float)9.202538E37F);
                Debug.Assert(pack.x == (float)1.5049601E38F);
                Debug.Assert(pack.z == (float) -2.5684299E38F);
                Debug.Assert(pack.time_boot_ms == (uint)993990221U);
                Debug.Assert(pack.vx == (float)3.3632789E38F);
                Debug.Assert(pack.vz == (float)2.7421875E38F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.vx = (float)3.3632789E38F;
            p32.vz = (float)2.7421875E38F;
            p32.y = (float)9.202538E37F;
            p32.time_boot_ms = (uint)993990221U;
            p32.vy = (float)7.481952E35F;
            p32.z = (float) -2.5684299E38F;
            p32.x = (float)1.5049601E38F;
            ADV_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)1267297775);
                Debug.Assert(pack.lat == (int)436685477);
                Debug.Assert(pack.alt == (int)612259277);
                Debug.Assert(pack.vx == (short)(short)19721);
                Debug.Assert(pack.vy == (short)(short)12540);
                Debug.Assert(pack.hdg == (ushort)(ushort)29745);
                Debug.Assert(pack.relative_alt == (int)1530539851);
                Debug.Assert(pack.time_boot_ms == (uint)2931613573U);
                Debug.Assert(pack.vz == (short)(short)14146);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.lon = (int)1267297775;
            p33.hdg = (ushort)(ushort)29745;
            p33.vx = (short)(short)19721;
            p33.time_boot_ms = (uint)2931613573U;
            p33.vz = (short)(short)14146;
            p33.relative_alt = (int)1530539851;
            p33.vy = (short)(short)12540;
            p33.alt = (int)612259277;
            p33.lat = (int)436685477;
            ADV_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.port == (byte)(byte)239);
                Debug.Assert(pack.chan8_scaled == (short)(short) -9203);
                Debug.Assert(pack.chan7_scaled == (short)(short)16148);
                Debug.Assert(pack.chan6_scaled == (short)(short) -31420);
                Debug.Assert(pack.chan2_scaled == (short)(short)14290);
                Debug.Assert(pack.time_boot_ms == (uint)985800794U);
                Debug.Assert(pack.chan4_scaled == (short)(short)667);
                Debug.Assert(pack.chan3_scaled == (short)(short)1968);
                Debug.Assert(pack.rssi == (byte)(byte)128);
                Debug.Assert(pack.chan5_scaled == (short)(short)32531);
                Debug.Assert(pack.chan1_scaled == (short)(short) -4143);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan4_scaled = (short)(short)667;
            p34.rssi = (byte)(byte)128;
            p34.chan2_scaled = (short)(short)14290;
            p34.chan8_scaled = (short)(short) -9203;
            p34.chan6_scaled = (short)(short) -31420;
            p34.chan7_scaled = (short)(short)16148;
            p34.chan5_scaled = (short)(short)32531;
            p34.port = (byte)(byte)239;
            p34.chan3_scaled = (short)(short)1968;
            p34.chan1_scaled = (short)(short) -4143;
            p34.time_boot_ms = (uint)985800794U;
            ADV_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)43204);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)45134);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)14848);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)34742);
                Debug.Assert(pack.port == (byte)(byte)153);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)46156);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)50276);
                Debug.Assert(pack.time_boot_ms == (uint)3662101041U);
                Debug.Assert(pack.rssi == (byte)(byte)56);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)11555);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)55455);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan5_raw = (ushort)(ushort)50276;
            p35.chan2_raw = (ushort)(ushort)11555;
            p35.chan7_raw = (ushort)(ushort)55455;
            p35.chan1_raw = (ushort)(ushort)34742;
            p35.port = (byte)(byte)153;
            p35.time_boot_ms = (uint)3662101041U;
            p35.chan3_raw = (ushort)(ushort)46156;
            p35.chan6_raw = (ushort)(ushort)43204;
            p35.chan4_raw = (ushort)(ushort)14848;
            p35.rssi = (byte)(byte)56;
            p35.chan8_raw = (ushort)(ushort)45134;
            ADV_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)40135);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)10973);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)51021);
                Debug.Assert(pack.port == (byte)(byte)218);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)14248);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)20577);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)24729);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)24792);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)32796);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)12164);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)8980);
                Debug.Assert(pack.time_usec == (uint)2092856850U);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)57758);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)58094);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)14912);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)43450);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)21416);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)21098);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo6_raw = (ushort)(ushort)51021;
            p36.servo8_raw = (ushort)(ushort)21098;
            p36.servo3_raw = (ushort)(ushort)21416;
            p36.servo16_raw_SET((ushort)(ushort)43450, PH) ;
            p36.servo14_raw_SET((ushort)(ushort)10973, PH) ;
            p36.servo1_raw = (ushort)(ushort)14248;
            p36.servo2_raw = (ushort)(ushort)20577;
            p36.servo4_raw = (ushort)(ushort)12164;
            p36.servo5_raw = (ushort)(ushort)24792;
            p36.servo7_raw = (ushort)(ushort)57758;
            p36.servo9_raw_SET((ushort)(ushort)24729, PH) ;
            p36.time_usec = (uint)2092856850U;
            p36.servo11_raw_SET((ushort)(ushort)32796, PH) ;
            p36.port = (byte)(byte)218;
            p36.servo12_raw_SET((ushort)(ushort)8980, PH) ;
            p36.servo15_raw_SET((ushort)(ushort)40135, PH) ;
            p36.servo10_raw_SET((ushort)(ushort)14912, PH) ;
            p36.servo13_raw_SET((ushort)(ushort)58094, PH) ;
            ADV_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.end_index == (short)(short) -31879);
                Debug.Assert(pack.target_component == (byte)(byte)106);
                Debug.Assert(pack.target_system == (byte)(byte)40);
                Debug.Assert(pack.start_index == (short)(short)25323);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.start_index = (short)(short)25323;
            p37.end_index = (short)(short) -31879;
            p37.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p37.target_system = (byte)(byte)40;
            p37.target_component = (byte)(byte)106;
            ADV_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_index == (short)(short)7193);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.end_index == (short)(short)6181);
                Debug.Assert(pack.target_component == (byte)(byte)207);
                Debug.Assert(pack.target_system == (byte)(byte)200);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p38.target_system = (byte)(byte)200;
            p38.target_component = (byte)(byte)207;
            p38.start_index = (short)(short)7193;
            p38.end_index = (short)(short)6181;
            ADV_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -3.138571E38F);
                Debug.Assert(pack.current == (byte)(byte)98);
                Debug.Assert(pack.param4 == (float) -2.1439278E37F);
                Debug.Assert(pack.y == (float)2.972902E38F);
                Debug.Assert(pack.param1 == (float) -1.9804964E38F);
                Debug.Assert(pack.seq == (ushort)(ushort)15517);
                Debug.Assert(pack.target_system == (byte)(byte)0);
                Debug.Assert(pack.param2 == (float)2.9585582E38F);
                Debug.Assert(pack.x == (float) -1.6832763E38F);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.param3 == (float)3.2757016E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)45);
                Debug.Assert(pack.target_component == (byte)(byte)142);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p39.y = (float)2.972902E38F;
            p39.target_system = (byte)(byte)0;
            p39.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p39.param4 = (float) -2.1439278E37F;
            p39.autocontinue = (byte)(byte)45;
            p39.command = MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO;
            p39.seq = (ushort)(ushort)15517;
            p39.target_component = (byte)(byte)142;
            p39.x = (float) -1.6832763E38F;
            p39.param3 = (float)3.2757016E38F;
            p39.param1 = (float) -1.9804964E38F;
            p39.param2 = (float)2.9585582E38F;
            p39.current = (byte)(byte)98;
            p39.z = (float) -3.138571E38F;
            ADV_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)8065);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)131);
                Debug.Assert(pack.target_system == (byte)(byte)80);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.seq = (ushort)(ushort)8065;
            p40.target_system = (byte)(byte)80;
            p40.target_component = (byte)(byte)131;
            p40.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            ADV_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)25);
                Debug.Assert(pack.target_system == (byte)(byte)31);
                Debug.Assert(pack.seq == (ushort)(ushort)6142);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_component = (byte)(byte)25;
            p41.target_system = (byte)(byte)31;
            p41.seq = (ushort)(ushort)6142;
            ADV_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)21920);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)21920;
            ADV_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_system == (byte)(byte)74);
                Debug.Assert(pack.target_component == (byte)(byte)83);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p43.target_system = (byte)(byte)74;
            p43.target_component = (byte)(byte)83;
            ADV_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)77);
                Debug.Assert(pack.count == (ushort)(ushort)28140);
                Debug.Assert(pack.target_system == (byte)(byte)92);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.count = (ushort)(ushort)28140;
            p44.target_system = (byte)(byte)92;
            p44.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p44.target_component = (byte)(byte)77;
            ADV_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)251);
                Debug.Assert(pack.target_component == (byte)(byte)12);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p45.target_component = (byte)(byte)12;
            p45.target_system = (byte)(byte)251;
            ADV_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)40404);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)40404;
            ADV_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)146);
                Debug.Assert(pack.type == MAV_MISSION_RESULT.MAV_MISSION_ERROR);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)2);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.target_system = (byte)(byte)146;
            p47.target_component = (byte)(byte)2;
            p47.type = MAV_MISSION_RESULT.MAV_MISSION_ERROR;
            p47.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            ADV_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (int)1821264630);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)5419406838516125495L);
                Debug.Assert(pack.latitude == (int)1287968069);
                Debug.Assert(pack.target_system == (byte)(byte)246);
                Debug.Assert(pack.altitude == (int)2088509118);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.longitude = (int)1821264630;
            p48.target_system = (byte)(byte)246;
            p48.time_usec_SET((ulong)5419406838516125495L, PH) ;
            p48.latitude = (int)1287968069;
            p48.altitude = (int)2088509118;
            ADV_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)8965494295075647257L);
                Debug.Assert(pack.altitude == (int)1803878097);
                Debug.Assert(pack.longitude == (int)1331282039);
                Debug.Assert(pack.latitude == (int) -1851857634);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.altitude = (int)1803878097;
            p49.time_usec_SET((ulong)8965494295075647257L, PH) ;
            p49.latitude = (int) -1851857634;
            p49.longitude = (int)1331282039;
            ADV_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_max == (float) -2.3929951E38F);
                Debug.Assert(pack.param_value0 == (float) -1.9219494E38F);
                Debug.Assert(pack.target_system == (byte)(byte)175);
                Debug.Assert(pack.target_component == (byte)(byte)6);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)135);
                Debug.Assert(pack.param_value_min == (float)1.0561939E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 12);
                Debug.Assert(pack.param_id_TRY(ph).Equals("bunbbkzbfljk"));
                Debug.Assert(pack.param_index == (short)(short)17278);
                Debug.Assert(pack.scale == (float)6.789017E36F);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.target_component = (byte)(byte)6;
            p50.target_system = (byte)(byte)175;
            p50.scale = (float)6.789017E36F;
            p50.param_value0 = (float) -1.9219494E38F;
            p50.param_index = (short)(short)17278;
            p50.param_id_SET("bunbbkzbfljk", PH) ;
            p50.param_value_max = (float) -2.3929951E38F;
            p50.param_value_min = (float)1.0561939E38F;
            p50.parameter_rc_channel_index = (byte)(byte)135;
            ADV_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)86);
                Debug.Assert(pack.target_component == (byte)(byte)97);
                Debug.Assert(pack.seq == (ushort)(ushort)48468);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p51.target_component = (byte)(byte)97;
            p51.target_system = (byte)(byte)86;
            p51.seq = (ushort)(ushort)48468;
            ADV_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)48);
                Debug.Assert(pack.p2z == (float)1.6764546E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.target_system == (byte)(byte)232);
                Debug.Assert(pack.p1x == (float) -3.379592E38F);
                Debug.Assert(pack.p2x == (float)3.0694833E38F);
                Debug.Assert(pack.p2y == (float)3.4424566E37F);
                Debug.Assert(pack.p1z == (float) -1.0690115E38F);
                Debug.Assert(pack.p1y == (float) -3.4008833E38F);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p1z = (float) -1.0690115E38F;
            p54.p2x = (float)3.0694833E38F;
            p54.target_component = (byte)(byte)48;
            p54.frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p54.target_system = (byte)(byte)232;
            p54.p1y = (float) -3.4008833E38F;
            p54.p2y = (float)3.4424566E37F;
            p54.p2z = (float)1.6764546E38F;
            p54.p1x = (float) -3.379592E38F;
            ADV_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1x == (float)4.2383794E37F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.p2z == (float) -2.3256963E38F);
                Debug.Assert(pack.p2y == (float) -1.494824E38F);
                Debug.Assert(pack.p2x == (float)2.2073768E38F);
                Debug.Assert(pack.p1z == (float) -1.5844953E38F);
                Debug.Assert(pack.p1y == (float)1.2958299E38F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p2z = (float) -2.3256963E38F;
            p55.p2x = (float)2.2073768E38F;
            p55.p1x = (float)4.2383794E37F;
            p55.p1y = (float)1.2958299E38F;
            p55.p1z = (float) -1.5844953E38F;
            p55.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p55.p2y = (float) -1.494824E38F;
            ADV_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-1.4165657E38F, 3.2781627E38F, 1.0654053E38F, -1.6552284E37F, -3.9708775E37F, -2.6567174E38F, 3.0590305E38F, 8.868773E37F, -1.5422127E38F}));
                Debug.Assert(pack.pitchspeed == (float) -2.6168957E38F);
                Debug.Assert(pack.yawspeed == (float) -6.420179E36F);
                Debug.Assert(pack.time_usec == (ulong)4541968835317212548L);
                Debug.Assert(pack.rollspeed == (float) -2.91214E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-9.526561E37F, 2.0215597E38F, 2.5859125E38F, 2.5402414E38F}));
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.q_SET(new float[] {-9.526561E37F, 2.0215597E38F, 2.5859125E38F, 2.5402414E38F}, 0) ;
            p61.time_usec = (ulong)4541968835317212548L;
            p61.pitchspeed = (float) -2.6168957E38F;
            p61.yawspeed = (float) -6.420179E36F;
            p61.covariance_SET(new float[] {-1.4165657E38F, 3.2781627E38F, 1.0654053E38F, -1.6552284E37F, -3.9708775E37F, -2.6567174E38F, 3.0590305E38F, 8.868773E37F, -1.5422127E38F}, 0) ;
            p61.rollspeed = (float) -2.91214E38F;
            ADV_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xtrack_error == (float)1.9312533E38F);
                Debug.Assert(pack.aspd_error == (float)3.0296202E38F);
                Debug.Assert(pack.nav_roll == (float) -2.8672754E38F);
                Debug.Assert(pack.nav_pitch == (float) -1.4210366E38F);
                Debug.Assert(pack.target_bearing == (short)(short) -13426);
                Debug.Assert(pack.nav_bearing == (short)(short)17006);
                Debug.Assert(pack.alt_error == (float)2.4585477E37F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)23597);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.alt_error = (float)2.4585477E37F;
            p62.nav_pitch = (float) -1.4210366E38F;
            p62.nav_roll = (float) -2.8672754E38F;
            p62.target_bearing = (short)(short) -13426;
            p62.xtrack_error = (float)1.9312533E38F;
            p62.aspd_error = (float)3.0296202E38F;
            p62.nav_bearing = (short)(short)17006;
            p62.wp_dist = (ushort)(ushort)23597;
            ADV_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float) -1.996477E38F);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
                Debug.Assert(pack.lon == (int)1225575986);
                Debug.Assert(pack.relative_alt == (int) -409233033);
                Debug.Assert(pack.vx == (float) -3.1227583E38F);
                Debug.Assert(pack.alt == (int) -2014869051);
                Debug.Assert(pack.vz == (float)3.1132696E38F);
                Debug.Assert(pack.lat == (int) -1667137143);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {6.582693E37F, 4.319763E37F, -5.4509995E37F, 3.235493E38F, -3.1417298E38F, -1.2804871E38F, -4.5174783E36F, -2.1145672E38F, -2.5720503E38F, -3.1201733E38F, -2.5793226E37F, -1.2029728E37F, -2.1933775E38F, 9.453248E37F, 7.5516634E37F, 2.1511913E38F, 1.7707817E38F, -1.2394361E38F, -2.199802E38F, 3.3811207E38F, 2.0369727E38F, 2.6264572E37F, -9.640791E37F, 1.3089386E38F, -1.4673183E38F, -2.3854413E36F, 5.181513E37F, 2.3327337E38F, 3.2755574E38F, 7.2101284E37F, 2.6610998E38F, 1.7597351E38F, 2.0040513E38F, -2.4431874E38F, -2.9550159E38F, -1.0177889E38F}));
                Debug.Assert(pack.time_usec == (ulong)8930557068506160805L);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.lon = (int)1225575986;
            p63.alt = (int) -2014869051;
            p63.covariance_SET(new float[] {6.582693E37F, 4.319763E37F, -5.4509995E37F, 3.235493E38F, -3.1417298E38F, -1.2804871E38F, -4.5174783E36F, -2.1145672E38F, -2.5720503E38F, -3.1201733E38F, -2.5793226E37F, -1.2029728E37F, -2.1933775E38F, 9.453248E37F, 7.5516634E37F, 2.1511913E38F, 1.7707817E38F, -1.2394361E38F, -2.199802E38F, 3.3811207E38F, 2.0369727E38F, 2.6264572E37F, -9.640791E37F, 1.3089386E38F, -1.4673183E38F, -2.3854413E36F, 5.181513E37F, 2.3327337E38F, 3.2755574E38F, 7.2101284E37F, 2.6610998E38F, 1.7597351E38F, 2.0040513E38F, -2.4431874E38F, -2.9550159E38F, -1.0177889E38F}, 0) ;
            p63.vz = (float)3.1132696E38F;
            p63.vx = (float) -3.1227583E38F;
            p63.lat = (int) -1667137143;
            p63.relative_alt = (int) -409233033;
            p63.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
            p63.time_usec = (ulong)8930557068506160805L;
            p63.vy = (float) -1.996477E38F;
            ADV_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
                Debug.Assert(pack.z == (float) -3.9277345E37F);
                Debug.Assert(pack.ay == (float)2.6084355E38F);
                Debug.Assert(pack.vz == (float)1.6328391E38F);
                Debug.Assert(pack.x == (float) -1.7990917E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-6.5168356E37F, 1.3133014E38F, -1.0223945E38F, -9.836805E37F, 1.2252978E37F, -2.6167793E38F, 2.6470307E38F, -1.2814076E38F, 3.4001233E38F, 1.7414203E38F, 1.5935281E38F, 3.3972486E38F, 3.2166141E38F, -3.910096E37F, 1.574702E38F, -2.8628804E38F, 6.971358E37F, 1.905197E38F, -1.594007E38F, 2.982144E38F, 2.8144131E38F, -6.6043253E37F, -1.4833326E38F, 6.9330505E36F, 2.9758503E38F, 3.0581093E38F, -2.062306E37F, 6.4951385E37F, -2.699158E38F, 2.3545685E38F, -2.4993318E38F, 1.0980527E38F, 2.584722E38F, -8.139718E37F, -1.4511268E38F, 2.4004378E38F, 3.1010034E38F, 3.733345E36F, 3.3584484E38F, 1.5263354E37F, -1.7563559E38F, -2.5774622E38F, -3.0696228E38F, 1.6863715E38F, -1.2701657E38F}));
                Debug.Assert(pack.vy == (float)2.3888659E38F);
                Debug.Assert(pack.time_usec == (ulong)6790391966198905812L);
                Debug.Assert(pack.ax == (float) -7.2197354E36F);
                Debug.Assert(pack.az == (float)9.084457E37F);
                Debug.Assert(pack.vx == (float)2.9932326E38F);
                Debug.Assert(pack.y == (float) -1.657821E38F);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.z = (float) -3.9277345E37F;
            p64.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO;
            p64.ax = (float) -7.2197354E36F;
            p64.ay = (float)2.6084355E38F;
            p64.az = (float)9.084457E37F;
            p64.vy = (float)2.3888659E38F;
            p64.time_usec = (ulong)6790391966198905812L;
            p64.vz = (float)1.6328391E38F;
            p64.covariance_SET(new float[] {-6.5168356E37F, 1.3133014E38F, -1.0223945E38F, -9.836805E37F, 1.2252978E37F, -2.6167793E38F, 2.6470307E38F, -1.2814076E38F, 3.4001233E38F, 1.7414203E38F, 1.5935281E38F, 3.3972486E38F, 3.2166141E38F, -3.910096E37F, 1.574702E38F, -2.8628804E38F, 6.971358E37F, 1.905197E38F, -1.594007E38F, 2.982144E38F, 2.8144131E38F, -6.6043253E37F, -1.4833326E38F, 6.9330505E36F, 2.9758503E38F, 3.0581093E38F, -2.062306E37F, 6.4951385E37F, -2.699158E38F, 2.3545685E38F, -2.4993318E38F, 1.0980527E38F, 2.584722E38F, -8.139718E37F, -1.4511268E38F, 2.4004378E38F, 3.1010034E38F, 3.733345E36F, 3.3584484E38F, 1.5263354E37F, -1.7563559E38F, -2.5774622E38F, -3.0696228E38F, 1.6863715E38F, -1.2701657E38F}, 0) ;
            p64.vx = (float)2.9932326E38F;
            p64.x = (float) -1.7990917E38F;
            p64.y = (float) -1.657821E38F;
            ADV_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)12900);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)49935);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)61924);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)63169);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)36359);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)40297);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)14792);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)34656);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)28803);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)31820);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)62236);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)29708);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)1209);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)46603);
                Debug.Assert(pack.time_boot_ms == (uint)570199851U);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)49332);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)27253);
                Debug.Assert(pack.rssi == (byte)(byte)133);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)20411);
                Debug.Assert(pack.chancount == (byte)(byte)30);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)38470);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan13_raw = (ushort)(ushort)46603;
            p65.rssi = (byte)(byte)133;
            p65.chan3_raw = (ushort)(ushort)62236;
            p65.chan9_raw = (ushort)(ushort)27253;
            p65.chan12_raw = (ushort)(ushort)20411;
            p65.chan17_raw = (ushort)(ushort)63169;
            p65.chan2_raw = (ushort)(ushort)1209;
            p65.chan15_raw = (ushort)(ushort)28803;
            p65.chan7_raw = (ushort)(ushort)29708;
            p65.chan1_raw = (ushort)(ushort)31820;
            p65.chan8_raw = (ushort)(ushort)12900;
            p65.time_boot_ms = (uint)570199851U;
            p65.chan18_raw = (ushort)(ushort)40297;
            p65.chancount = (byte)(byte)30;
            p65.chan11_raw = (ushort)(ushort)38470;
            p65.chan5_raw = (ushort)(ushort)34656;
            p65.chan4_raw = (ushort)(ushort)14792;
            p65.chan14_raw = (ushort)(ushort)36359;
            p65.chan6_raw = (ushort)(ushort)49935;
            p65.chan16_raw = (ushort)(ushort)61924;
            p65.chan10_raw = (ushort)(ushort)49332;
            ADV_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_stop == (byte)(byte)135);
                Debug.Assert(pack.req_stream_id == (byte)(byte)149);
                Debug.Assert(pack.target_system == (byte)(byte)58);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)16619);
                Debug.Assert(pack.target_component == (byte)(byte)208);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_system = (byte)(byte)58;
            p66.target_component = (byte)(byte)208;
            p66.req_message_rate = (ushort)(ushort)16619;
            p66.start_stop = (byte)(byte)135;
            p66.req_stream_id = (byte)(byte)149;
            ADV_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_rate == (ushort)(ushort)14296);
                Debug.Assert(pack.stream_id == (byte)(byte)125);
                Debug.Assert(pack.on_off == (byte)(byte)219);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.message_rate = (ushort)(ushort)14296;
            p67.on_off = (byte)(byte)219;
            p67.stream_id = (byte)(byte)125;
            ADV_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target == (byte)(byte)175);
                Debug.Assert(pack.y == (short)(short)17771);
                Debug.Assert(pack.r == (short)(short) -3083);
                Debug.Assert(pack.buttons == (ushort)(ushort)55387);
                Debug.Assert(pack.z == (short)(short) -16508);
                Debug.Assert(pack.x == (short)(short)27210);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.z = (short)(short) -16508;
            p69.target = (byte)(byte)175;
            p69.x = (short)(short)27210;
            p69.y = (short)(short)17771;
            p69.r = (short)(short) -3083;
            p69.buttons = (ushort)(ushort)55387;
            ADV_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)20989);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)29910);
                Debug.Assert(pack.target_component == (byte)(byte)50);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)9657);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)1773);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)5031);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)17598);
                Debug.Assert(pack.target_system == (byte)(byte)84);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)31818);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)42326);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan8_raw = (ushort)(ushort)5031;
            p70.chan1_raw = (ushort)(ushort)29910;
            p70.chan7_raw = (ushort)(ushort)9657;
            p70.target_system = (byte)(byte)84;
            p70.chan2_raw = (ushort)(ushort)17598;
            p70.target_component = (byte)(byte)50;
            p70.chan5_raw = (ushort)(ushort)20989;
            p70.chan3_raw = (ushort)(ushort)31818;
            p70.chan4_raw = (ushort)(ushort)1773;
            p70.chan6_raw = (ushort)(ushort)42326;
            ADV_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param2 == (float) -1.4139652E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.target_component == (byte)(byte)195);
                Debug.Assert(pack.target_system == (byte)(byte)180);
                Debug.Assert(pack.z == (float)1.3664759E38F);
                Debug.Assert(pack.param4 == (float)3.0589742E38F);
                Debug.Assert(pack.seq == (ushort)(ushort)41388);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.param1 == (float) -2.6045352E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)207);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION);
                Debug.Assert(pack.x == (int) -618708540);
                Debug.Assert(pack.y == (int) -1245845368);
                Debug.Assert(pack.current == (byte)(byte)17);
                Debug.Assert(pack.param3 == (float)3.0823926E38F);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p73.current = (byte)(byte)17;
            p73.target_system = (byte)(byte)180;
            p73.param4 = (float)3.0589742E38F;
            p73.param3 = (float)3.0823926E38F;
            p73.y = (int) -1245845368;
            p73.command = MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION;
            p73.x = (int) -618708540;
            p73.target_component = (byte)(byte)195;
            p73.param1 = (float) -2.6045352E38F;
            p73.seq = (ushort)(ushort)41388;
            p73.param2 = (float) -1.4139652E38F;
            p73.z = (float)1.3664759E38F;
            p73.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p73.autocontinue = (byte)(byte)207;
            ADV_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (float) -4.332903E37F);
                Debug.Assert(pack.climb == (float)2.0753986E38F);
                Debug.Assert(pack.heading == (short)(short)6583);
                Debug.Assert(pack.throttle == (ushort)(ushort)47318);
                Debug.Assert(pack.airspeed == (float) -2.0522063E37F);
                Debug.Assert(pack.groundspeed == (float) -3.0112292E38F);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.alt = (float) -4.332903E37F;
            p74.groundspeed = (float) -3.0112292E38F;
            p74.climb = (float)2.0753986E38F;
            p74.airspeed = (float) -2.0522063E37F;
            p74.heading = (short)(short)6583;
            p74.throttle = (ushort)(ushort)47318;
            ADV_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current == (byte)(byte)167);
                Debug.Assert(pack.y == (int) -1785637445);
                Debug.Assert(pack.param2 == (float) -2.5776312E38F);
                Debug.Assert(pack.target_component == (byte)(byte)200);
                Debug.Assert(pack.z == (float)2.5169527E38F);
                Debug.Assert(pack.x == (int)22538342);
                Debug.Assert(pack.param3 == (float) -2.4512193E38F);
                Debug.Assert(pack.param4 == (float) -2.700657E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.param1 == (float)2.3182164E36F);
                Debug.Assert(pack.autocontinue == (byte)(byte)144);
                Debug.Assert(pack.target_system == (byte)(byte)186);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_MOTOR_TEST);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.param1 = (float)2.3182164E36F;
            p75.param2 = (float) -2.5776312E38F;
            p75.y = (int) -1785637445;
            p75.command = MAV_CMD.MAV_CMD_DO_MOTOR_TEST;
            p75.target_component = (byte)(byte)200;
            p75.autocontinue = (byte)(byte)144;
            p75.param4 = (float) -2.700657E38F;
            p75.current = (byte)(byte)167;
            p75.frame = MAV_FRAME.MAV_FRAME_MISSION;
            p75.x = (int)22538342;
            p75.target_system = (byte)(byte)186;
            p75.param3 = (float) -2.4512193E38F;
            p75.z = (float)2.5169527E38F;
            ADV_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param7 == (float)3.3793137E38F);
                Debug.Assert(pack.param2 == (float) -1.7020956E38F);
                Debug.Assert(pack.param6 == (float)5.804631E37F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_SET_ROI);
                Debug.Assert(pack.param5 == (float)1.4642207E38F);
                Debug.Assert(pack.param3 == (float)1.1718878E38F);
                Debug.Assert(pack.target_component == (byte)(byte)225);
                Debug.Assert(pack.param1 == (float)2.4731654E38F);
                Debug.Assert(pack.param4 == (float)2.794328E38F);
                Debug.Assert(pack.confirmation == (byte)(byte)4);
                Debug.Assert(pack.target_system == (byte)(byte)63);
            };
            COMMAND_LONG p76 = new COMMAND_LONG();
            PH.setPack(p76);
            p76.param6 = (float)5.804631E37F;
            p76.command = MAV_CMD.MAV_CMD_DO_SET_ROI;
            p76.confirmation = (byte)(byte)4;
            p76.param5 = (float)1.4642207E38F;
            p76.param2 = (float) -1.7020956E38F;
            p76.target_system = (byte)(byte)63;
            p76.param1 = (float)2.4731654E38F;
            p76.param4 = (float)2.794328E38F;
            p76.param7 = (float)3.3793137E38F;
            p76.param3 = (float)1.1718878E38F;
            p76.target_component = (byte)(byte)225;
            ADV_TEST_CH.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)235);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)126);
                Debug.Assert(pack.result == MAV_RESULT.MAV_RESULT_UNSUPPORTED);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)254);
                Debug.Assert(pack.result_param2_TRY(ph) == (int) -720689250);
            };
            COMMAND_ACK p77 = new COMMAND_ACK();
            PH.setPack(p77);
            p77.result_param2_SET((int) -720689250, PH) ;
            p77.target_system_SET((byte)(byte)235, PH) ;
            p77.result = MAV_RESULT.MAV_RESULT_UNSUPPORTED;
            p77.command = MAV_CMD.MAV_CMD_DO_PAUSE_CONTINUE;
            p77.target_component_SET((byte)(byte)254, PH) ;
            p77.progress_SET((byte)(byte)126, PH) ;
            ADV_TEST_CH.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1427425485U);
                Debug.Assert(pack.thrust == (float) -2.2974638E38F);
                Debug.Assert(pack.pitch == (float) -1.5358047E37F);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)235);
                Debug.Assert(pack.mode_switch == (byte)(byte)9);
                Debug.Assert(pack.yaw == (float) -1.9793924E38F);
                Debug.Assert(pack.roll == (float)6.973825E37F);
            };
            MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.pitch = (float) -1.5358047E37F;
            p81.time_boot_ms = (uint)1427425485U;
            p81.manual_override_switch = (byte)(byte)235;
            p81.yaw = (float) -1.9793924E38F;
            p81.thrust = (float) -2.2974638E38F;
            p81.roll = (float)6.973825E37F;
            p81.mode_switch = (byte)(byte)9;
            ADV_TEST_CH.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)180);
                Debug.Assert(pack.type_mask == (byte)(byte)186);
                Debug.Assert(pack.time_boot_ms == (uint)2557410039U);
                Debug.Assert(pack.body_yaw_rate == (float)2.8986744E38F);
                Debug.Assert(pack.body_roll_rate == (float)3.299242E38F);
                Debug.Assert(pack.target_component == (byte)(byte)207);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.1844273E38F, -1.6475465E38F, -1.6688146E38F, 9.0344584E36F}));
                Debug.Assert(pack.thrust == (float)2.2128304E38F);
                Debug.Assert(pack.body_pitch_rate == (float)2.4404704E38F);
            };
            GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.time_boot_ms = (uint)2557410039U;
            p82.target_system = (byte)(byte)180;
            p82.type_mask = (byte)(byte)186;
            p82.body_yaw_rate = (float)2.8986744E38F;
            p82.thrust = (float)2.2128304E38F;
            p82.target_component = (byte)(byte)207;
            p82.body_pitch_rate = (float)2.4404704E38F;
            p82.body_roll_rate = (float)3.299242E38F;
            p82.q_SET(new float[] {-2.1844273E38F, -1.6475465E38F, -1.6688146E38F, 9.0344584E36F}, 0) ;
            CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.thrust == (float) -5.0337204E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.2935493E38F, -1.3438538E38F, -1.8836527E38F, 1.127432E38F}));
                Debug.Assert(pack.body_yaw_rate == (float)1.1801593E38F);
                Debug.Assert(pack.body_pitch_rate == (float)2.2317816E38F);
                Debug.Assert(pack.body_roll_rate == (float)1.0116976E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)182);
                Debug.Assert(pack.time_boot_ms == (uint)3564241710U);
            };
            GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.body_yaw_rate = (float)1.1801593E38F;
            p83.q_SET(new float[] {-2.2935493E38F, -1.3438538E38F, -1.8836527E38F, 1.127432E38F}, 0) ;
            p83.body_pitch_rate = (float)2.2317816E38F;
            p83.time_boot_ms = (uint)3564241710U;
            p83.thrust = (float) -5.0337204E37F;
            p83.body_roll_rate = (float)1.0116976E38F;
            p83.type_mask = (byte)(byte)182;
            CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)3.1247301E38F);
                Debug.Assert(pack.vy == (float)2.7580323E38F);
                Debug.Assert(pack.target_system == (byte)(byte)185);
                Debug.Assert(pack.yaw_rate == (float) -9.978119E37F);
                Debug.Assert(pack.vx == (float)5.223354E36F);
                Debug.Assert(pack.z == (float) -6.1964354E37F);
                Debug.Assert(pack.afy == (float)1.4024765E38F);
                Debug.Assert(pack.vz == (float)3.2549456E38F);
                Debug.Assert(pack.y == (float)1.3070923E38F);
                Debug.Assert(pack.target_component == (byte)(byte)135);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.x == (float) -1.4683147E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2235275209U);
                Debug.Assert(pack.type_mask == (ushort)(ushort)23287);
                Debug.Assert(pack.afz == (float)3.3588908E38F);
                Debug.Assert(pack.afx == (float)1.7111247E38F);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.vz = (float)3.2549456E38F;
            p84.y = (float)1.3070923E38F;
            p84.afx = (float)1.7111247E38F;
            p84.yaw_rate = (float) -9.978119E37F;
            p84.target_system = (byte)(byte)185;
            p84.type_mask = (ushort)(ushort)23287;
            p84.afy = (float)1.4024765E38F;
            p84.vx = (float)5.223354E36F;
            p84.z = (float) -6.1964354E37F;
            p84.yaw = (float)3.1247301E38F;
            p84.time_boot_ms = (uint)2235275209U;
            p84.vy = (float)2.7580323E38F;
            p84.x = (float) -1.4683147E38F;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p84.afz = (float)3.3588908E38F;
            p84.target_component = (byte)(byte)135;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -1.8549623E38F);
                Debug.Assert(pack.yaw_rate == (float)9.427655E37F);
                Debug.Assert(pack.lat_int == (int) -367787320);
                Debug.Assert(pack.afz == (float)2.6614767E38F);
                Debug.Assert(pack.lon_int == (int)1161726550);
                Debug.Assert(pack.target_component == (byte)(byte)177);
                Debug.Assert(pack.afx == (float)2.0980873E38F);
                Debug.Assert(pack.vx == (float) -3.2696974E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.vy == (float) -5.341575E37F);
                Debug.Assert(pack.alt == (float) -2.6196379E38F);
                Debug.Assert(pack.target_system == (byte)(byte)245);
                Debug.Assert(pack.time_boot_ms == (uint)746064435U);
                Debug.Assert(pack.afy == (float)2.1825007E38F);
                Debug.Assert(pack.vz == (float)2.8335548E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)59469);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.afz = (float)2.6614767E38F;
            p86.vy = (float) -5.341575E37F;
            p86.type_mask = (ushort)(ushort)59469;
            p86.yaw = (float) -1.8549623E38F;
            p86.time_boot_ms = (uint)746064435U;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p86.lat_int = (int) -367787320;
            p86.target_system = (byte)(byte)245;
            p86.vx = (float) -3.2696974E38F;
            p86.afy = (float)2.1825007E38F;
            p86.target_component = (byte)(byte)177;
            p86.yaw_rate = (float)9.427655E37F;
            p86.alt = (float) -2.6196379E38F;
            p86.vz = (float)2.8335548E38F;
            p86.afx = (float)2.0980873E38F;
            p86.lon_int = (int)1161726550;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afy == (float) -3.1758682E38F);
                Debug.Assert(pack.lat_int == (int)1241247510);
                Debug.Assert(pack.afz == (float) -2.092771E38F);
                Debug.Assert(pack.yaw == (float) -2.2276185E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3173834600U);
                Debug.Assert(pack.vx == (float)2.7801965E38F);
                Debug.Assert(pack.vz == (float)2.2066542E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)55050);
                Debug.Assert(pack.lon_int == (int)1983164477);
                Debug.Assert(pack.vy == (float) -2.4117475E38F);
                Debug.Assert(pack.afx == (float)6.3819266E37F);
                Debug.Assert(pack.yaw_rate == (float) -3.2016562E37F);
                Debug.Assert(pack.alt == (float)2.6773783E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.type_mask = (ushort)(ushort)55050;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p87.vx = (float)2.7801965E38F;
            p87.vz = (float)2.2066542E37F;
            p87.afz = (float) -2.092771E38F;
            p87.yaw = (float) -2.2276185E38F;
            p87.vy = (float) -2.4117475E38F;
            p87.lon_int = (int)1983164477;
            p87.lat_int = (int)1241247510;
            p87.afx = (float)6.3819266E37F;
            p87.afy = (float) -3.1758682E38F;
            p87.time_boot_ms = (uint)3173834600U;
            p87.yaw_rate = (float) -3.2016562E37F;
            p87.alt = (float)2.6773783E38F;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2014493138U);
                Debug.Assert(pack.yaw == (float) -1.1561938E38F);
                Debug.Assert(pack.z == (float) -5.6188385E37F);
                Debug.Assert(pack.pitch == (float) -1.1873834E38F);
                Debug.Assert(pack.y == (float)2.6006734E37F);
                Debug.Assert(pack.x == (float)9.502251E37F);
                Debug.Assert(pack.roll == (float)5.0614495E37F);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.x = (float)9.502251E37F;
            p89.roll = (float)5.0614495E37F;
            p89.yaw = (float) -1.1561938E38F;
            p89.pitch = (float) -1.1873834E38F;
            p89.z = (float) -5.6188385E37F;
            p89.y = (float)2.6006734E37F;
            p89.time_boot_ms = (uint)2014493138U;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float)1.9169254E38F);
                Debug.Assert(pack.vx == (short)(short) -6416);
                Debug.Assert(pack.yaw == (float) -1.9692457E38F);
                Debug.Assert(pack.vz == (short)(short) -11996);
                Debug.Assert(pack.vy == (short)(short)25865);
                Debug.Assert(pack.yawspeed == (float) -2.6827843E38F);
                Debug.Assert(pack.xacc == (short)(short) -19431);
                Debug.Assert(pack.time_usec == (ulong)2288445691459195576L);
                Debug.Assert(pack.lat == (int) -1420979873);
                Debug.Assert(pack.yacc == (short)(short)21240);
                Debug.Assert(pack.roll == (float) -1.2847029E38F);
                Debug.Assert(pack.pitchspeed == (float)4.1333464E37F);
                Debug.Assert(pack.pitch == (float) -2.1516751E37F);
                Debug.Assert(pack.alt == (int) -1015293981);
                Debug.Assert(pack.zacc == (short)(short)24802);
                Debug.Assert(pack.lon == (int) -439217025);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.vz = (short)(short) -11996;
            p90.vx = (short)(short) -6416;
            p90.vy = (short)(short)25865;
            p90.yawspeed = (float) -2.6827843E38F;
            p90.pitch = (float) -2.1516751E37F;
            p90.yaw = (float) -1.9692457E38F;
            p90.pitchspeed = (float)4.1333464E37F;
            p90.roll = (float) -1.2847029E38F;
            p90.rollspeed = (float)1.9169254E38F;
            p90.zacc = (short)(short)24802;
            p90.yacc = (short)(short)21240;
            p90.alt = (int) -1015293981;
            p90.xacc = (short)(short) -19431;
            p90.lon = (int) -439217025;
            p90.time_usec = (ulong)2288445691459195576L;
            p90.lat = (int) -1420979873;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll_ailerons == (float)2.7854367E38F);
                Debug.Assert(pack.aux3 == (float)3.4234494E37F);
                Debug.Assert(pack.yaw_rudder == (float)8.870236E37F);
                Debug.Assert(pack.aux4 == (float) -2.293637E38F);
                Debug.Assert(pack.aux1 == (float)2.164342E38F);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
                Debug.Assert(pack.nav_mode == (byte)(byte)103);
                Debug.Assert(pack.pitch_elevator == (float)9.999386E37F);
                Debug.Assert(pack.time_usec == (ulong)1490828613079102946L);
                Debug.Assert(pack.aux2 == (float) -2.5969572E37F);
                Debug.Assert(pack.throttle == (float) -5.9104194E37F);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.roll_ailerons = (float)2.7854367E38F;
            p91.aux4 = (float) -2.293637E38F;
            p91.aux1 = (float)2.164342E38F;
            p91.yaw_rudder = (float)8.870236E37F;
            p91.aux3 = (float)3.4234494E37F;
            p91.pitch_elevator = (float)9.999386E37F;
            p91.aux2 = (float) -2.5969572E37F;
            p91.mode = MAV_MODE.MAV_MODE_STABILIZE_DISARMED;
            p91.nav_mode = (byte)(byte)103;
            p91.throttle = (float) -5.9104194E37F;
            p91.time_usec = (ulong)1490828613079102946L;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)5009);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)46665);
                Debug.Assert(pack.rssi == (byte)(byte)124);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)27524);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)56549);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)52896);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)16214);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)41185);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)40527);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)35877);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)9105);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)26906);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)13417);
                Debug.Assert(pack.time_usec == (ulong)8814913709813905665L);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan9_raw = (ushort)(ushort)9105;
            p92.time_usec = (ulong)8814913709813905665L;
            p92.chan3_raw = (ushort)(ushort)26906;
            p92.chan6_raw = (ushort)(ushort)16214;
            p92.chan12_raw = (ushort)(ushort)5009;
            p92.chan2_raw = (ushort)(ushort)52896;
            p92.chan8_raw = (ushort)(ushort)41185;
            p92.chan11_raw = (ushort)(ushort)46665;
            p92.chan7_raw = (ushort)(ushort)56549;
            p92.rssi = (byte)(byte)124;
            p92.chan5_raw = (ushort)(ushort)13417;
            p92.chan4_raw = (ushort)(ushort)35877;
            p92.chan10_raw = (ushort)(ushort)40527;
            p92.chan1_raw = (ushort)(ushort)27524;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)6151054989257247401L);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_MANUAL_ARMED);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-2.6520413E38F, -2.6876203E38F, -2.0838947E38F, -2.7119022E38F, -2.0833629E38F, 1.7055442E38F, 2.915691E38F, -4.596814E37F, -1.6513039E38F, -1.817208E37F, -6.342975E37F, 6.494029E36F, 2.5535984E38F, 1.4036224E38F, -3.2251999E38F, -9.309588E37F}));
                Debug.Assert(pack.flags == (ulong)2837511087090256163L);
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.mode = MAV_MODE.MAV_MODE_MANUAL_ARMED;
            p93.time_usec = (ulong)6151054989257247401L;
            p93.flags = (ulong)2837511087090256163L;
            p93.controls_SET(new float[] {-2.6520413E38F, -2.6876203E38F, -2.0838947E38F, -2.7119022E38F, -2.0833629E38F, 1.7055442E38F, 2.915691E38F, -4.596814E37F, -1.6513039E38F, -1.817208E37F, -6.342975E37F, 6.494029E36F, 2.5535984E38F, 1.4036224E38F, -3.2251999E38F, -9.309588E37F}, 0) ;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)4032116250858615547L);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float) -1.3251515E38F);
                Debug.Assert(pack.flow_y == (short)(short) -18357);
                Debug.Assert(pack.sensor_id == (byte)(byte)194);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float)2.2669505E38F);
                Debug.Assert(pack.ground_distance == (float)3.3979528E38F);
                Debug.Assert(pack.flow_comp_m_x == (float)9.572817E37F);
                Debug.Assert(pack.flow_x == (short)(short) -19834);
                Debug.Assert(pack.quality == (byte)(byte)79);
                Debug.Assert(pack.flow_comp_m_y == (float)6.0843897E37F);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_rate_x_SET((float) -1.3251515E38F, PH) ;
            p100.flow_rate_y_SET((float)2.2669505E38F, PH) ;
            p100.flow_comp_m_y = (float)6.0843897E37F;
            p100.time_usec = (ulong)4032116250858615547L;
            p100.flow_x = (short)(short) -19834;
            p100.ground_distance = (float)3.3979528E38F;
            p100.flow_y = (short)(short) -18357;
            p100.flow_comp_m_x = (float)9.572817E37F;
            p100.sensor_id = (byte)(byte)194;
            p100.quality = (byte)(byte)79;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -1.1151377E38F);
                Debug.Assert(pack.y == (float) -1.1624824E38F);
                Debug.Assert(pack.roll == (float) -1.1165359E38F);
                Debug.Assert(pack.pitch == (float) -2.2924337E38F);
                Debug.Assert(pack.usec == (ulong)9061985174529881068L);
                Debug.Assert(pack.yaw == (float) -2.6282133E38F);
                Debug.Assert(pack.z == (float) -2.070289E38F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.yaw = (float) -2.6282133E38F;
            p101.roll = (float) -1.1165359E38F;
            p101.usec = (ulong)9061985174529881068L;
            p101.y = (float) -1.1624824E38F;
            p101.z = (float) -2.070289E38F;
            p101.pitch = (float) -2.2924337E38F;
            p101.x = (float) -1.1151377E38F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -9.809589E37F);
                Debug.Assert(pack.pitch == (float)3.0808084E38F);
                Debug.Assert(pack.yaw == (float)1.6815966E38F);
                Debug.Assert(pack.roll == (float) -1.3194801E38F);
                Debug.Assert(pack.usec == (ulong)8540531968982663036L);
                Debug.Assert(pack.z == (float) -3.0164561E38F);
                Debug.Assert(pack.x == (float) -1.6659699E38F);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.roll = (float) -1.3194801E38F;
            p102.usec = (ulong)8540531968982663036L;
            p102.y = (float) -9.809589E37F;
            p102.x = (float) -1.6659699E38F;
            p102.z = (float) -3.0164561E38F;
            p102.yaw = (float)1.6815966E38F;
            p102.pitch = (float)3.0808084E38F;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)997695161404932000L);
                Debug.Assert(pack.z == (float) -1.833913E38F);
                Debug.Assert(pack.y == (float)1.2808254E38F);
                Debug.Assert(pack.x == (float)1.8699382E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)997695161404932000L;
            p103.y = (float)1.2808254E38F;
            p103.z = (float) -1.833913E38F;
            p103.x = (float)1.8699382E38F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -1.3257005E38F);
                Debug.Assert(pack.y == (float) -3.1854403E38F);
                Debug.Assert(pack.usec == (ulong)7819267061800666602L);
                Debug.Assert(pack.yaw == (float)2.1222211E38F);
                Debug.Assert(pack.z == (float)2.853872E38F);
                Debug.Assert(pack.x == (float)1.2771946E37F);
                Debug.Assert(pack.roll == (float) -8.592246E36F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.x = (float)1.2771946E37F;
            p104.y = (float) -3.1854403E38F;
            p104.roll = (float) -8.592246E36F;
            p104.z = (float)2.853872E38F;
            p104.yaw = (float)2.1222211E38F;
            p104.pitch = (float) -1.3257005E38F;
            p104.usec = (ulong)7819267061800666602L;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (float) -1.0419149E38F);
                Debug.Assert(pack.abs_pressure == (float) -2.6698726E38F);
                Debug.Assert(pack.ymag == (float)3.3063713E38F);
                Debug.Assert(pack.time_usec == (ulong)2642661882598826067L);
                Debug.Assert(pack.ygyro == (float)3.243639E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)62149);
                Debug.Assert(pack.yacc == (float) -4.7260687E37F);
                Debug.Assert(pack.pressure_alt == (float)1.044369E38F);
                Debug.Assert(pack.zacc == (float)1.28455E37F);
                Debug.Assert(pack.diff_pressure == (float) -2.6009091E38F);
                Debug.Assert(pack.temperature == (float)1.4469902E38F);
                Debug.Assert(pack.xgyro == (float)3.1985447E38F);
                Debug.Assert(pack.xacc == (float)3.0388501E38F);
                Debug.Assert(pack.xmag == (float) -3.3906364E38F);
                Debug.Assert(pack.zmag == (float) -3.2671715E38F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.zmag = (float) -3.2671715E38F;
            p105.ygyro = (float)3.243639E38F;
            p105.xmag = (float) -3.3906364E38F;
            p105.yacc = (float) -4.7260687E37F;
            p105.pressure_alt = (float)1.044369E38F;
            p105.xacc = (float)3.0388501E38F;
            p105.time_usec = (ulong)2642661882598826067L;
            p105.xgyro = (float)3.1985447E38F;
            p105.fields_updated = (ushort)(ushort)62149;
            p105.zgyro = (float) -1.0419149E38F;
            p105.zacc = (float)1.28455E37F;
            p105.diff_pressure = (float) -2.6009091E38F;
            p105.ymag = (float)3.3063713E38F;
            p105.temperature = (float)1.4469902E38F;
            p105.abs_pressure = (float) -2.6698726E38F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.quality == (byte)(byte)44);
                Debug.Assert(pack.distance == (float) -2.1591434E38F);
                Debug.Assert(pack.time_usec == (ulong)296057624642834505L);
                Debug.Assert(pack.integrated_zgyro == (float) -2.8768241E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)2899269298U);
                Debug.Assert(pack.sensor_id == (byte)(byte)66);
                Debug.Assert(pack.integrated_ygyro == (float) -6.63829E37F);
                Debug.Assert(pack.integrated_x == (float) -3.1114742E38F);
                Debug.Assert(pack.temperature == (short)(short)28039);
                Debug.Assert(pack.integrated_y == (float)7.541935E36F);
                Debug.Assert(pack.integration_time_us == (uint)1452853546U);
                Debug.Assert(pack.integrated_xgyro == (float)7.146315E37F);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.quality = (byte)(byte)44;
            p106.time_usec = (ulong)296057624642834505L;
            p106.temperature = (short)(short)28039;
            p106.sensor_id = (byte)(byte)66;
            p106.integrated_xgyro = (float)7.146315E37F;
            p106.time_delta_distance_us = (uint)2899269298U;
            p106.integrated_ygyro = (float) -6.63829E37F;
            p106.integrated_y = (float)7.541935E36F;
            p106.integration_time_us = (uint)1452853546U;
            p106.distance = (float) -2.1591434E38F;
            p106.integrated_x = (float) -3.1114742E38F;
            p106.integrated_zgyro = (float) -2.8768241E38F;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (float) -4.1560683E37F);
                Debug.Assert(pack.yacc == (float) -5.5079064E36F);
                Debug.Assert(pack.zgyro == (float) -1.5084754E38F);
                Debug.Assert(pack.temperature == (float)7.8981523E37F);
                Debug.Assert(pack.xgyro == (float) -1.4323955E38F);
                Debug.Assert(pack.xacc == (float)3.194802E38F);
                Debug.Assert(pack.zmag == (float)3.3050817E38F);
                Debug.Assert(pack.xmag == (float)8.4797656E37F);
                Debug.Assert(pack.diff_pressure == (float)1.9440116E38F);
                Debug.Assert(pack.fields_updated == (uint)3984637129U);
                Debug.Assert(pack.pressure_alt == (float) -3.3385548E38F);
                Debug.Assert(pack.time_usec == (ulong)4851144582994435946L);
                Debug.Assert(pack.zacc == (float) -2.7963557E38F);
                Debug.Assert(pack.abs_pressure == (float) -3.1953526E36F);
                Debug.Assert(pack.ymag == (float) -7.796982E37F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.pressure_alt = (float) -3.3385548E38F;
            p107.xgyro = (float) -1.4323955E38F;
            p107.fields_updated = (uint)3984637129U;
            p107.diff_pressure = (float)1.9440116E38F;
            p107.xacc = (float)3.194802E38F;
            p107.zacc = (float) -2.7963557E38F;
            p107.zmag = (float)3.3050817E38F;
            p107.temperature = (float)7.8981523E37F;
            p107.time_usec = (ulong)4851144582994435946L;
            p107.abs_pressure = (float) -3.1953526E36F;
            p107.ymag = (float) -7.796982E37F;
            p107.zgyro = (float) -1.5084754E38F;
            p107.xmag = (float)8.4797656E37F;
            p107.ygyro = (float) -4.1560683E37F;
            p107.yacc = (float) -5.5079064E36F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vd == (float) -2.5663122E38F);
                Debug.Assert(pack.yaw == (float) -2.0317188E38F);
                Debug.Assert(pack.std_dev_horz == (float) -2.8622622E38F);
                Debug.Assert(pack.std_dev_vert == (float) -1.0319614E38F);
                Debug.Assert(pack.q4 == (float) -2.713331E38F);
                Debug.Assert(pack.yacc == (float) -3.3486277E38F);
                Debug.Assert(pack.lat == (float)1.0185587E37F);
                Debug.Assert(pack.pitch == (float) -1.6228161E38F);
                Debug.Assert(pack.q3 == (float) -1.4525151E38F);
                Debug.Assert(pack.zacc == (float)1.781033E38F);
                Debug.Assert(pack.alt == (float)8.0958144E36F);
                Debug.Assert(pack.ve == (float)2.6835255E38F);
                Debug.Assert(pack.xgyro == (float) -3.0356788E38F);
                Debug.Assert(pack.zgyro == (float)3.2828285E38F);
                Debug.Assert(pack.ygyro == (float)2.4294879E38F);
                Debug.Assert(pack.vn == (float) -2.9949846E38F);
                Debug.Assert(pack.xacc == (float) -1.4199818E38F);
                Debug.Assert(pack.lon == (float)3.2408445E38F);
                Debug.Assert(pack.roll == (float) -1.8368821E38F);
                Debug.Assert(pack.q1 == (float) -1.6786225E38F);
                Debug.Assert(pack.q2 == (float) -1.6712166E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.std_dev_horz = (float) -2.8622622E38F;
            p108.xacc = (float) -1.4199818E38F;
            p108.yaw = (float) -2.0317188E38F;
            p108.ve = (float)2.6835255E38F;
            p108.zacc = (float)1.781033E38F;
            p108.zgyro = (float)3.2828285E38F;
            p108.alt = (float)8.0958144E36F;
            p108.lon = (float)3.2408445E38F;
            p108.q2 = (float) -1.6712166E38F;
            p108.q1 = (float) -1.6786225E38F;
            p108.lat = (float)1.0185587E37F;
            p108.vd = (float) -2.5663122E38F;
            p108.xgyro = (float) -3.0356788E38F;
            p108.yacc = (float) -3.3486277E38F;
            p108.vn = (float) -2.9949846E38F;
            p108.ygyro = (float)2.4294879E38F;
            p108.q4 = (float) -2.713331E38F;
            p108.std_dev_vert = (float) -1.0319614E38F;
            p108.roll = (float) -1.8368821E38F;
            p108.q3 = (float) -1.4525151E38F;
            p108.pitch = (float) -1.6228161E38F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.remrssi == (byte)(byte)174);
                Debug.Assert(pack.txbuf == (byte)(byte)233);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)62833);
                Debug.Assert(pack.noise == (byte)(byte)181);
                Debug.Assert(pack.remnoise == (byte)(byte)3);
                Debug.Assert(pack.rssi == (byte)(byte)158);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)57370);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.txbuf = (byte)(byte)233;
            p109.rxerrors = (ushort)(ushort)57370;
            p109.fixed_ = (ushort)(ushort)62833;
            p109.rssi = (byte)(byte)158;
            p109.noise = (byte)(byte)181;
            p109.remnoise = (byte)(byte)3;
            p109.remrssi = (byte)(byte)174;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)1);
                Debug.Assert(pack.target_network == (byte)(byte)28);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)201, (byte)206, (byte)173, (byte)14, (byte)81, (byte)75, (byte)213, (byte)146, (byte)43, (byte)69, (byte)230, (byte)36, (byte)9, (byte)155, (byte)105, (byte)81, (byte)219, (byte)170, (byte)183, (byte)119, (byte)235, (byte)122, (byte)173, (byte)140, (byte)84, (byte)106, (byte)11, (byte)20, (byte)199, (byte)242, (byte)108, (byte)126, (byte)113, (byte)113, (byte)98, (byte)114, (byte)94, (byte)217, (byte)16, (byte)70, (byte)113, (byte)70, (byte)200, (byte)202, (byte)43, (byte)43, (byte)45, (byte)25, (byte)37, (byte)204, (byte)203, (byte)228, (byte)40, (byte)154, (byte)242, (byte)197, (byte)18, (byte)108, (byte)30, (byte)251, (byte)110, (byte)116, (byte)186, (byte)2, (byte)16, (byte)134, (byte)139, (byte)175, (byte)211, (byte)52, (byte)189, (byte)50, (byte)42, (byte)84, (byte)239, (byte)64, (byte)19, (byte)171, (byte)9, (byte)66, (byte)42, (byte)38, (byte)142, (byte)133, (byte)80, (byte)74, (byte)166, (byte)0, (byte)180, (byte)39, (byte)126, (byte)46, (byte)68, (byte)205, (byte)40, (byte)101, (byte)224, (byte)246, (byte)169, (byte)31, (byte)128, (byte)26, (byte)151, (byte)175, (byte)160, (byte)223, (byte)224, (byte)115, (byte)163, (byte)53, (byte)141, (byte)214, (byte)245, (byte)79, (byte)83, (byte)188, (byte)83, (byte)94, (byte)68, (byte)112, (byte)115, (byte)152, (byte)234, (byte)62, (byte)145, (byte)246, (byte)100, (byte)87, (byte)212, (byte)196, (byte)72, (byte)91, (byte)194, (byte)212, (byte)226, (byte)63, (byte)17, (byte)31, (byte)165, (byte)21, (byte)106, (byte)51, (byte)107, (byte)1, (byte)43, (byte)23, (byte)134, (byte)87, (byte)122, (byte)74, (byte)193, (byte)174, (byte)53, (byte)164, (byte)228, (byte)46, (byte)2, (byte)240, (byte)111, (byte)137, (byte)59, (byte)212, (byte)251, (byte)198, (byte)174, (byte)31, (byte)65, (byte)241, (byte)66, (byte)86, (byte)204, (byte)45, (byte)175, (byte)207, (byte)124, (byte)136, (byte)4, (byte)26, (byte)243, (byte)74, (byte)224, (byte)158, (byte)218, (byte)4, (byte)140, (byte)106, (byte)205, (byte)192, (byte)155, (byte)122, (byte)107, (byte)170, (byte)175, (byte)28, (byte)222, (byte)215, (byte)16, (byte)193, (byte)19, (byte)68, (byte)154, (byte)87, (byte)217, (byte)8, (byte)103, (byte)132, (byte)50, (byte)48, (byte)162, (byte)175, (byte)145, (byte)30, (byte)149, (byte)35, (byte)135, (byte)135, (byte)228, (byte)99, (byte)205, (byte)65, (byte)129, (byte)146, (byte)49, (byte)125, (byte)38, (byte)196, (byte)190, (byte)247, (byte)51, (byte)146, (byte)227, (byte)213, (byte)238, (byte)246, (byte)24, (byte)211, (byte)118, (byte)21, (byte)12, (byte)210, (byte)16, (byte)89, (byte)48, (byte)255, (byte)22, (byte)131, (byte)119, (byte)254, (byte)110, (byte)98, (byte)43}));
                Debug.Assert(pack.target_component == (byte)(byte)93);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)28;
            p110.target_component = (byte)(byte)93;
            p110.target_system = (byte)(byte)1;
            p110.payload_SET(new byte[] {(byte)201, (byte)206, (byte)173, (byte)14, (byte)81, (byte)75, (byte)213, (byte)146, (byte)43, (byte)69, (byte)230, (byte)36, (byte)9, (byte)155, (byte)105, (byte)81, (byte)219, (byte)170, (byte)183, (byte)119, (byte)235, (byte)122, (byte)173, (byte)140, (byte)84, (byte)106, (byte)11, (byte)20, (byte)199, (byte)242, (byte)108, (byte)126, (byte)113, (byte)113, (byte)98, (byte)114, (byte)94, (byte)217, (byte)16, (byte)70, (byte)113, (byte)70, (byte)200, (byte)202, (byte)43, (byte)43, (byte)45, (byte)25, (byte)37, (byte)204, (byte)203, (byte)228, (byte)40, (byte)154, (byte)242, (byte)197, (byte)18, (byte)108, (byte)30, (byte)251, (byte)110, (byte)116, (byte)186, (byte)2, (byte)16, (byte)134, (byte)139, (byte)175, (byte)211, (byte)52, (byte)189, (byte)50, (byte)42, (byte)84, (byte)239, (byte)64, (byte)19, (byte)171, (byte)9, (byte)66, (byte)42, (byte)38, (byte)142, (byte)133, (byte)80, (byte)74, (byte)166, (byte)0, (byte)180, (byte)39, (byte)126, (byte)46, (byte)68, (byte)205, (byte)40, (byte)101, (byte)224, (byte)246, (byte)169, (byte)31, (byte)128, (byte)26, (byte)151, (byte)175, (byte)160, (byte)223, (byte)224, (byte)115, (byte)163, (byte)53, (byte)141, (byte)214, (byte)245, (byte)79, (byte)83, (byte)188, (byte)83, (byte)94, (byte)68, (byte)112, (byte)115, (byte)152, (byte)234, (byte)62, (byte)145, (byte)246, (byte)100, (byte)87, (byte)212, (byte)196, (byte)72, (byte)91, (byte)194, (byte)212, (byte)226, (byte)63, (byte)17, (byte)31, (byte)165, (byte)21, (byte)106, (byte)51, (byte)107, (byte)1, (byte)43, (byte)23, (byte)134, (byte)87, (byte)122, (byte)74, (byte)193, (byte)174, (byte)53, (byte)164, (byte)228, (byte)46, (byte)2, (byte)240, (byte)111, (byte)137, (byte)59, (byte)212, (byte)251, (byte)198, (byte)174, (byte)31, (byte)65, (byte)241, (byte)66, (byte)86, (byte)204, (byte)45, (byte)175, (byte)207, (byte)124, (byte)136, (byte)4, (byte)26, (byte)243, (byte)74, (byte)224, (byte)158, (byte)218, (byte)4, (byte)140, (byte)106, (byte)205, (byte)192, (byte)155, (byte)122, (byte)107, (byte)170, (byte)175, (byte)28, (byte)222, (byte)215, (byte)16, (byte)193, (byte)19, (byte)68, (byte)154, (byte)87, (byte)217, (byte)8, (byte)103, (byte)132, (byte)50, (byte)48, (byte)162, (byte)175, (byte)145, (byte)30, (byte)149, (byte)35, (byte)135, (byte)135, (byte)228, (byte)99, (byte)205, (byte)65, (byte)129, (byte)146, (byte)49, (byte)125, (byte)38, (byte)196, (byte)190, (byte)247, (byte)51, (byte)146, (byte)227, (byte)213, (byte)238, (byte)246, (byte)24, (byte)211, (byte)118, (byte)21, (byte)12, (byte)210, (byte)16, (byte)89, (byte)48, (byte)255, (byte)22, (byte)131, (byte)119, (byte)254, (byte)110, (byte)98, (byte)43}, 0) ;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ts1 == (long)7404558409275228659L);
                Debug.Assert(pack.tc1 == (long)8530140892140204814L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long)7404558409275228659L;
            p111.tc1 = (long)8530140892140204814L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)950998489U);
                Debug.Assert(pack.time_usec == (ulong)2648290674668448605L);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.seq = (uint)950998489U;
            p112.time_usec = (ulong)2648290674668448605L;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fix_type == (byte)(byte)177);
                Debug.Assert(pack.vel == (ushort)(ushort)36500);
                Debug.Assert(pack.vn == (short)(short) -9853);
                Debug.Assert(pack.alt == (int)1928592100);
                Debug.Assert(pack.satellites_visible == (byte)(byte)85);
                Debug.Assert(pack.time_usec == (ulong)4542039950744770198L);
                Debug.Assert(pack.vd == (short)(short) -11790);
                Debug.Assert(pack.epv == (ushort)(ushort)2928);
                Debug.Assert(pack.ve == (short)(short) -23448);
                Debug.Assert(pack.eph == (ushort)(ushort)61614);
                Debug.Assert(pack.lat == (int)110862272);
                Debug.Assert(pack.cog == (ushort)(ushort)42054);
                Debug.Assert(pack.lon == (int)1747073571);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.alt = (int)1928592100;
            p113.eph = (ushort)(ushort)61614;
            p113.satellites_visible = (byte)(byte)85;
            p113.time_usec = (ulong)4542039950744770198L;
            p113.vd = (short)(short) -11790;
            p113.cog = (ushort)(ushort)42054;
            p113.fix_type = (byte)(byte)177;
            p113.lat = (int)110862272;
            p113.epv = (ushort)(ushort)2928;
            p113.ve = (short)(short) -23448;
            p113.lon = (int)1747073571;
            p113.vel = (ushort)(ushort)36500;
            p113.vn = (short)(short) -9853;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_zgyro == (float)2.7438386E38F);
                Debug.Assert(pack.integration_time_us == (uint)1217447563U);
                Debug.Assert(pack.time_usec == (ulong)6292685897223811541L);
                Debug.Assert(pack.sensor_id == (byte)(byte)241);
                Debug.Assert(pack.temperature == (short)(short)30600);
                Debug.Assert(pack.distance == (float)1.8089514E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)900665057U);
                Debug.Assert(pack.integrated_y == (float)2.9349606E38F);
                Debug.Assert(pack.integrated_ygyro == (float) -3.0845637E38F);
                Debug.Assert(pack.quality == (byte)(byte)97);
                Debug.Assert(pack.integrated_xgyro == (float)9.529653E37F);
                Debug.Assert(pack.integrated_x == (float)1.8205586E38F);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.integrated_y = (float)2.9349606E38F;
            p114.integration_time_us = (uint)1217447563U;
            p114.integrated_xgyro = (float)9.529653E37F;
            p114.integrated_x = (float)1.8205586E38F;
            p114.integrated_zgyro = (float)2.7438386E38F;
            p114.distance = (float)1.8089514E38F;
            p114.time_delta_distance_us = (uint)900665057U;
            p114.quality = (byte)(byte)97;
            p114.sensor_id = (byte)(byte)241;
            p114.time_usec = (ulong)6292685897223811541L;
            p114.integrated_ygyro = (float) -3.0845637E38F;
            p114.temperature = (short)(short)30600;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)8183822713733519545L);
                Debug.Assert(pack.rollspeed == (float) -1.3342478E38F);
                Debug.Assert(pack.vz == (short)(short)8953);
                Debug.Assert(pack.lon == (int)1705513655);
                Debug.Assert(pack.vy == (short)(short)12707);
                Debug.Assert(pack.xacc == (short)(short)626);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)51588);
                Debug.Assert(pack.pitchspeed == (float) -1.6368184E38F);
                Debug.Assert(pack.yawspeed == (float) -3.0778035E38F);
                Debug.Assert(pack.alt == (int)1870118422);
                Debug.Assert(pack.vx == (short)(short)12591);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {2.9206071E38F, 3.3999395E38F, -5.6298604E37F, 1.9649708E38F}));
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)17407);
                Debug.Assert(pack.yacc == (short)(short) -4336);
                Debug.Assert(pack.lat == (int) -307094129);
                Debug.Assert(pack.zacc == (short)(short) -5072);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.vx = (short)(short)12591;
            p115.rollspeed = (float) -1.3342478E38F;
            p115.yacc = (short)(short) -4336;
            p115.xacc = (short)(short)626;
            p115.vz = (short)(short)8953;
            p115.attitude_quaternion_SET(new float[] {2.9206071E38F, 3.3999395E38F, -5.6298604E37F, 1.9649708E38F}, 0) ;
            p115.yawspeed = (float) -3.0778035E38F;
            p115.true_airspeed = (ushort)(ushort)51588;
            p115.alt = (int)1870118422;
            p115.vy = (short)(short)12707;
            p115.lat = (int) -307094129;
            p115.lon = (int)1705513655;
            p115.ind_airspeed = (ushort)(ushort)17407;
            p115.time_usec = (ulong)8183822713733519545L;
            p115.pitchspeed = (float) -1.6368184E38F;
            p115.zacc = (short)(short) -5072;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short)4969);
                Debug.Assert(pack.ygyro == (short)(short)16261);
                Debug.Assert(pack.zacc == (short)(short)4623);
                Debug.Assert(pack.ymag == (short)(short)13076);
                Debug.Assert(pack.zgyro == (short)(short) -14370);
                Debug.Assert(pack.zmag == (short)(short) -21481);
                Debug.Assert(pack.xmag == (short)(short) -3508);
                Debug.Assert(pack.yacc == (short)(short)16974);
                Debug.Assert(pack.time_boot_ms == (uint)754051762U);
                Debug.Assert(pack.xgyro == (short)(short) -4062);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.ygyro = (short)(short)16261;
            p116.ymag = (short)(short)13076;
            p116.xmag = (short)(short) -3508;
            p116.zmag = (short)(short) -21481;
            p116.yacc = (short)(short)16974;
            p116.xacc = (short)(short)4969;
            p116.xgyro = (short)(short) -4062;
            p116.zacc = (short)(short)4623;
            p116.time_boot_ms = (uint)754051762U;
            p116.zgyro = (short)(short) -14370;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)88);
                Debug.Assert(pack.start == (ushort)(ushort)21067);
                Debug.Assert(pack.end == (ushort)(ushort)27310);
                Debug.Assert(pack.target_component == (byte)(byte)98);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.end = (ushort)(ushort)27310;
            p117.target_component = (byte)(byte)98;
            p117.start = (ushort)(ushort)21067;
            p117.target_system = (byte)(byte)88;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)52822);
                Debug.Assert(pack.time_utc == (uint)3451380546U);
                Debug.Assert(pack.num_logs == (ushort)(ushort)61076);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)25512);
                Debug.Assert(pack.size == (uint)2633438267U);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.num_logs = (ushort)(ushort)61076;
            p118.time_utc = (uint)3451380546U;
            p118.id = (ushort)(ushort)52822;
            p118.size = (uint)2633438267U;
            p118.last_log_num = (ushort)(ushort)25512;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)143);
                Debug.Assert(pack.target_component == (byte)(byte)192);
                Debug.Assert(pack.id == (ushort)(ushort)23497);
                Debug.Assert(pack.count == (uint)1086946855U);
                Debug.Assert(pack.ofs == (uint)1735796230U);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_component = (byte)(byte)192;
            p119.count = (uint)1086946855U;
            p119.ofs = (uint)1735796230U;
            p119.target_system = (byte)(byte)143;
            p119.id = (ushort)(ushort)23497;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)3809358115U);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)11, (byte)206, (byte)211, (byte)204, (byte)253, (byte)141, (byte)2, (byte)117, (byte)172, (byte)155, (byte)130, (byte)20, (byte)104, (byte)208, (byte)105, (byte)132, (byte)94, (byte)212, (byte)53, (byte)1, (byte)74, (byte)97, (byte)222, (byte)9, (byte)52, (byte)37, (byte)38, (byte)240, (byte)1, (byte)220, (byte)67, (byte)142, (byte)99, (byte)201, (byte)165, (byte)95, (byte)239, (byte)255, (byte)247, (byte)1, (byte)192, (byte)38, (byte)94, (byte)9, (byte)138, (byte)108, (byte)11, (byte)0, (byte)155, (byte)15, (byte)39, (byte)166, (byte)139, (byte)181, (byte)6, (byte)49, (byte)140, (byte)3, (byte)137, (byte)30, (byte)241, (byte)101, (byte)254, (byte)168, (byte)187, (byte)125, (byte)217, (byte)198, (byte)202, (byte)107, (byte)20, (byte)68, (byte)114, (byte)237, (byte)174, (byte)226, (byte)136, (byte)200, (byte)74, (byte)44, (byte)205, (byte)104, (byte)224, (byte)252, (byte)160, (byte)74, (byte)183, (byte)58, (byte)8, (byte)85}));
                Debug.Assert(pack.id == (ushort)(ushort)38013);
                Debug.Assert(pack.count == (byte)(byte)178);
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.data__SET(new byte[] {(byte)11, (byte)206, (byte)211, (byte)204, (byte)253, (byte)141, (byte)2, (byte)117, (byte)172, (byte)155, (byte)130, (byte)20, (byte)104, (byte)208, (byte)105, (byte)132, (byte)94, (byte)212, (byte)53, (byte)1, (byte)74, (byte)97, (byte)222, (byte)9, (byte)52, (byte)37, (byte)38, (byte)240, (byte)1, (byte)220, (byte)67, (byte)142, (byte)99, (byte)201, (byte)165, (byte)95, (byte)239, (byte)255, (byte)247, (byte)1, (byte)192, (byte)38, (byte)94, (byte)9, (byte)138, (byte)108, (byte)11, (byte)0, (byte)155, (byte)15, (byte)39, (byte)166, (byte)139, (byte)181, (byte)6, (byte)49, (byte)140, (byte)3, (byte)137, (byte)30, (byte)241, (byte)101, (byte)254, (byte)168, (byte)187, (byte)125, (byte)217, (byte)198, (byte)202, (byte)107, (byte)20, (byte)68, (byte)114, (byte)237, (byte)174, (byte)226, (byte)136, (byte)200, (byte)74, (byte)44, (byte)205, (byte)104, (byte)224, (byte)252, (byte)160, (byte)74, (byte)183, (byte)58, (byte)8, (byte)85}, 0) ;
            p120.ofs = (uint)3809358115U;
            p120.count = (byte)(byte)178;
            p120.id = (ushort)(ushort)38013;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)56);
                Debug.Assert(pack.target_component == (byte)(byte)51);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_component = (byte)(byte)51;
            p121.target_system = (byte)(byte)56;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)85);
                Debug.Assert(pack.target_system == (byte)(byte)230);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)230;
            p122.target_component = (byte)(byte)85;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)151);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)26, (byte)154, (byte)146, (byte)150, (byte)234, (byte)142, (byte)25, (byte)255, (byte)130, (byte)100, (byte)28, (byte)51, (byte)47, (byte)210, (byte)55, (byte)27, (byte)15, (byte)33, (byte)244, (byte)220, (byte)159, (byte)150, (byte)108, (byte)112, (byte)147, (byte)215, (byte)14, (byte)100, (byte)28, (byte)238, (byte)41, (byte)214, (byte)10, (byte)224, (byte)226, (byte)123, (byte)218, (byte)230, (byte)8, (byte)7, (byte)35, (byte)3, (byte)238, (byte)248, (byte)168, (byte)187, (byte)18, (byte)72, (byte)78, (byte)218, (byte)155, (byte)76, (byte)98, (byte)108, (byte)243, (byte)46, (byte)241, (byte)118, (byte)241, (byte)191, (byte)24, (byte)177, (byte)240, (byte)39, (byte)25, (byte)35, (byte)118, (byte)200, (byte)209, (byte)208, (byte)178, (byte)42, (byte)250, (byte)197, (byte)79, (byte)248, (byte)164, (byte)187, (byte)108, (byte)126, (byte)150, (byte)132, (byte)118, (byte)181, (byte)95, (byte)246, (byte)6, (byte)194, (byte)66, (byte)176, (byte)252, (byte)38, (byte)212, (byte)121, (byte)225, (byte)43, (byte)245, (byte)170, (byte)9, (byte)52, (byte)27, (byte)198, (byte)190, (byte)152, (byte)44, (byte)59, (byte)138, (byte)140, (byte)139, (byte)104}));
                Debug.Assert(pack.len == (byte)(byte)69);
                Debug.Assert(pack.target_component == (byte)(byte)156);
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.len = (byte)(byte)69;
            p123.target_system = (byte)(byte)151;
            p123.target_component = (byte)(byte)156;
            p123.data__SET(new byte[] {(byte)26, (byte)154, (byte)146, (byte)150, (byte)234, (byte)142, (byte)25, (byte)255, (byte)130, (byte)100, (byte)28, (byte)51, (byte)47, (byte)210, (byte)55, (byte)27, (byte)15, (byte)33, (byte)244, (byte)220, (byte)159, (byte)150, (byte)108, (byte)112, (byte)147, (byte)215, (byte)14, (byte)100, (byte)28, (byte)238, (byte)41, (byte)214, (byte)10, (byte)224, (byte)226, (byte)123, (byte)218, (byte)230, (byte)8, (byte)7, (byte)35, (byte)3, (byte)238, (byte)248, (byte)168, (byte)187, (byte)18, (byte)72, (byte)78, (byte)218, (byte)155, (byte)76, (byte)98, (byte)108, (byte)243, (byte)46, (byte)241, (byte)118, (byte)241, (byte)191, (byte)24, (byte)177, (byte)240, (byte)39, (byte)25, (byte)35, (byte)118, (byte)200, (byte)209, (byte)208, (byte)178, (byte)42, (byte)250, (byte)197, (byte)79, (byte)248, (byte)164, (byte)187, (byte)108, (byte)126, (byte)150, (byte)132, (byte)118, (byte)181, (byte)95, (byte)246, (byte)6, (byte)194, (byte)66, (byte)176, (byte)252, (byte)38, (byte)212, (byte)121, (byte)225, (byte)43, (byte)245, (byte)170, (byte)9, (byte)52, (byte)27, (byte)198, (byte)190, (byte)152, (byte)44, (byte)59, (byte)138, (byte)140, (byte)139, (byte)104}, 0) ;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
                Debug.Assert(pack.dgps_age == (uint)1382360775U);
                Debug.Assert(pack.satellites_visible == (byte)(byte)120);
                Debug.Assert(pack.epv == (ushort)(ushort)13242);
                Debug.Assert(pack.cog == (ushort)(ushort)64310);
                Debug.Assert(pack.alt == (int) -1426571532);
                Debug.Assert(pack.lat == (int)1067319480);
                Debug.Assert(pack.lon == (int) -1990162409);
                Debug.Assert(pack.dgps_numch == (byte)(byte)50);
                Debug.Assert(pack.time_usec == (ulong)3489766019485340016L);
                Debug.Assert(pack.eph == (ushort)(ushort)30228);
                Debug.Assert(pack.vel == (ushort)(ushort)51814);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.dgps_numch = (byte)(byte)50;
            p124.lon = (int) -1990162409;
            p124.dgps_age = (uint)1382360775U;
            p124.time_usec = (ulong)3489766019485340016L;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC;
            p124.epv = (ushort)(ushort)13242;
            p124.lat = (int)1067319480;
            p124.satellites_visible = (byte)(byte)120;
            p124.eph = (ushort)(ushort)30228;
            p124.vel = (ushort)(ushort)51814;
            p124.alt = (int) -1426571532;
            p124.cog = (ushort)(ushort)64310;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vservo == (ushort)(ushort)25055);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED));
                Debug.Assert(pack.Vcc == (ushort)(ushort)30944);
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED);
            p125.Vcc = (ushort)(ushort)30944;
            p125.Vservo = (ushort)(ushort)25055;
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)191, (byte)115, (byte)4, (byte)88, (byte)72, (byte)171, (byte)63, (byte)189, (byte)154, (byte)152, (byte)242, (byte)155, (byte)10, (byte)139, (byte)200, (byte)252, (byte)239, (byte)180, (byte)208, (byte)24, (byte)135, (byte)65, (byte)60, (byte)248, (byte)171, (byte)61, (byte)56, (byte)162, (byte)20, (byte)169, (byte)220, (byte)254, (byte)153, (byte)83, (byte)147, (byte)180, (byte)21, (byte)113, (byte)15, (byte)71, (byte)66, (byte)225, (byte)24, (byte)230, (byte)169, (byte)80, (byte)32, (byte)178, (byte)48, (byte)119, (byte)236, (byte)49, (byte)17, (byte)105, (byte)187, (byte)190, (byte)135, (byte)179, (byte)21, (byte)153, (byte)234, (byte)4, (byte)6, (byte)175, (byte)174, (byte)93, (byte)241, (byte)193, (byte)187, (byte)226}));
                Debug.Assert(pack.baudrate == (uint)625979809U);
                Debug.Assert(pack.count == (byte)(byte)151);
                Debug.Assert(pack.timeout == (ushort)(ushort)55748);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI));
                Debug.Assert(pack.device == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.count = (byte)(byte)151;
            p126.timeout = (ushort)(ushort)55748;
            p126.baudrate = (uint)625979809U;
            p126.data__SET(new byte[] {(byte)191, (byte)115, (byte)4, (byte)88, (byte)72, (byte)171, (byte)63, (byte)189, (byte)154, (byte)152, (byte)242, (byte)155, (byte)10, (byte)139, (byte)200, (byte)252, (byte)239, (byte)180, (byte)208, (byte)24, (byte)135, (byte)65, (byte)60, (byte)248, (byte)171, (byte)61, (byte)56, (byte)162, (byte)20, (byte)169, (byte)220, (byte)254, (byte)153, (byte)83, (byte)147, (byte)180, (byte)21, (byte)113, (byte)15, (byte)71, (byte)66, (byte)225, (byte)24, (byte)230, (byte)169, (byte)80, (byte)32, (byte)178, (byte)48, (byte)119, (byte)236, (byte)49, (byte)17, (byte)105, (byte)187, (byte)190, (byte)135, (byte)179, (byte)21, (byte)153, (byte)234, (byte)4, (byte)6, (byte)175, (byte)174, (byte)93, (byte)241, (byte)193, (byte)187, (byte)226}, 0) ;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI);
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_a_mm == (int) -774968610);
                Debug.Assert(pack.rtk_rate == (byte)(byte)162);
                Debug.Assert(pack.baseline_c_mm == (int) -361988782);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)15);
                Debug.Assert(pack.time_last_baseline_ms == (uint)1360783794U);
                Debug.Assert(pack.iar_num_hypotheses == (int)2016225964);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)233);
                Debug.Assert(pack.tow == (uint)484935555U);
                Debug.Assert(pack.nsats == (byte)(byte)73);
                Debug.Assert(pack.baseline_b_mm == (int)549554548);
                Debug.Assert(pack.rtk_health == (byte)(byte)154);
                Debug.Assert(pack.accuracy == (uint)760543679U);
                Debug.Assert(pack.wn == (ushort)(ushort)2023);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.rtk_rate = (byte)(byte)162;
            p127.baseline_a_mm = (int) -774968610;
            p127.rtk_health = (byte)(byte)154;
            p127.time_last_baseline_ms = (uint)1360783794U;
            p127.baseline_b_mm = (int)549554548;
            p127.tow = (uint)484935555U;
            p127.baseline_coords_type = (byte)(byte)15;
            p127.nsats = (byte)(byte)73;
            p127.iar_num_hypotheses = (int)2016225964;
            p127.baseline_c_mm = (int) -361988782;
            p127.rtk_receiver_id = (byte)(byte)233;
            p127.accuracy = (uint)760543679U;
            p127.wn = (ushort)(ushort)2023;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tow == (uint)934305830U);
                Debug.Assert(pack.wn == (ushort)(ushort)9948);
                Debug.Assert(pack.iar_num_hypotheses == (int) -1313505935);
                Debug.Assert(pack.rtk_rate == (byte)(byte)215);
                Debug.Assert(pack.rtk_health == (byte)(byte)149);
                Debug.Assert(pack.baseline_a_mm == (int) -2118391837);
                Debug.Assert(pack.time_last_baseline_ms == (uint)758040748U);
                Debug.Assert(pack.nsats == (byte)(byte)19);
                Debug.Assert(pack.baseline_c_mm == (int)1870139642);
                Debug.Assert(pack.baseline_b_mm == (int)1649809138);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)154);
                Debug.Assert(pack.accuracy == (uint)977585767U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)86);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.rtk_rate = (byte)(byte)215;
            p128.rtk_health = (byte)(byte)149;
            p128.tow = (uint)934305830U;
            p128.nsats = (byte)(byte)19;
            p128.baseline_a_mm = (int) -2118391837;
            p128.baseline_b_mm = (int)1649809138;
            p128.wn = (ushort)(ushort)9948;
            p128.iar_num_hypotheses = (int) -1313505935;
            p128.baseline_coords_type = (byte)(byte)154;
            p128.accuracy = (uint)977585767U;
            p128.time_last_baseline_ms = (uint)758040748U;
            p128.rtk_receiver_id = (byte)(byte)86;
            p128.baseline_c_mm = (int)1870139642;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short) -22474);
                Debug.Assert(pack.time_boot_ms == (uint)2648256093U);
                Debug.Assert(pack.yacc == (short)(short)19129);
                Debug.Assert(pack.ygyro == (short)(short) -25519);
                Debug.Assert(pack.ymag == (short)(short) -24855);
                Debug.Assert(pack.zgyro == (short)(short)14141);
                Debug.Assert(pack.xgyro == (short)(short)7420);
                Debug.Assert(pack.zacc == (short)(short)19233);
                Debug.Assert(pack.xmag == (short)(short)8243);
                Debug.Assert(pack.zmag == (short)(short)28004);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.zmag = (short)(short)28004;
            p129.zacc = (short)(short)19233;
            p129.time_boot_ms = (uint)2648256093U;
            p129.zgyro = (short)(short)14141;
            p129.xmag = (short)(short)8243;
            p129.ymag = (short)(short) -24855;
            p129.yacc = (short)(short)19129;
            p129.xgyro = (short)(short)7420;
            p129.ygyro = (short)(short) -25519;
            p129.xacc = (short)(short) -22474;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)132);
                Debug.Assert(pack.size == (uint)2821841261U);
                Debug.Assert(pack.jpg_quality == (byte)(byte)37);
                Debug.Assert(pack.width == (ushort)(ushort)50793);
                Debug.Assert(pack.packets == (ushort)(ushort)1757);
                Debug.Assert(pack.payload == (byte)(byte)127);
                Debug.Assert(pack.height == (ushort)(ushort)584);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)132;
            p130.packets = (ushort)(ushort)1757;
            p130.jpg_quality = (byte)(byte)37;
            p130.payload = (byte)(byte)127;
            p130.width = (ushort)(ushort)50793;
            p130.size = (uint)2821841261U;
            p130.height = (ushort)(ushort)584;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seqnr == (ushort)(ushort)61651);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)129, (byte)179, (byte)86, (byte)76, (byte)233, (byte)174, (byte)186, (byte)44, (byte)91, (byte)224, (byte)175, (byte)90, (byte)211, (byte)70, (byte)55, (byte)19, (byte)235, (byte)145, (byte)150, (byte)186, (byte)147, (byte)12, (byte)162, (byte)41, (byte)91, (byte)7, (byte)233, (byte)69, (byte)19, (byte)66, (byte)180, (byte)250, (byte)223, (byte)91, (byte)151, (byte)94, (byte)46, (byte)55, (byte)25, (byte)119, (byte)40, (byte)211, (byte)45, (byte)200, (byte)52, (byte)201, (byte)115, (byte)153, (byte)246, (byte)105, (byte)139, (byte)194, (byte)26, (byte)200, (byte)215, (byte)254, (byte)125, (byte)27, (byte)218, (byte)142, (byte)122, (byte)118, (byte)123, (byte)101, (byte)40, (byte)143, (byte)33, (byte)102, (byte)77, (byte)50, (byte)238, (byte)222, (byte)195, (byte)230, (byte)56, (byte)97, (byte)181, (byte)104, (byte)5, (byte)214, (byte)83, (byte)227, (byte)43, (byte)223, (byte)70, (byte)191, (byte)85, (byte)251, (byte)155, (byte)171, (byte)124, (byte)92, (byte)79, (byte)111, (byte)175, (byte)58, (byte)118, (byte)135, (byte)237, (byte)132, (byte)227, (byte)70, (byte)112, (byte)26, (byte)203, (byte)132, (byte)51, (byte)253, (byte)127, (byte)140, (byte)152, (byte)12, (byte)93, (byte)95, (byte)81, (byte)100, (byte)35, (byte)105, (byte)108, (byte)17, (byte)50, (byte)94, (byte)24, (byte)0, (byte)81, (byte)221, (byte)2, (byte)136, (byte)29, (byte)18, (byte)10, (byte)7, (byte)60, (byte)206, (byte)185, (byte)191, (byte)232, (byte)187, (byte)43, (byte)236, (byte)152, (byte)227, (byte)26, (byte)78, (byte)214, (byte)172, (byte)58, (byte)202, (byte)223, (byte)153, (byte)1, (byte)120, (byte)24, (byte)62, (byte)206, (byte)2, (byte)96, (byte)27, (byte)155, (byte)164, (byte)215, (byte)82, (byte)231, (byte)31, (byte)58, (byte)250, (byte)173, (byte)22, (byte)178, (byte)140, (byte)151, (byte)84, (byte)77, (byte)221, (byte)124, (byte)166, (byte)137, (byte)179, (byte)29, (byte)31, (byte)145, (byte)223, (byte)156, (byte)236, (byte)65, (byte)234, (byte)198, (byte)92, (byte)100, (byte)77, (byte)110, (byte)4, (byte)25, (byte)112, (byte)202, (byte)64, (byte)37, (byte)41, (byte)119, (byte)156, (byte)255, (byte)232, (byte)28, (byte)56, (byte)17, (byte)167, (byte)197, (byte)253, (byte)39, (byte)112, (byte)37, (byte)10, (byte)54, (byte)229, (byte)241, (byte)38, (byte)226, (byte)52, (byte)180, (byte)24, (byte)25, (byte)80, (byte)226, (byte)236, (byte)5, (byte)211, (byte)164, (byte)164, (byte)105, (byte)236, (byte)196, (byte)209, (byte)121, (byte)130, (byte)189, (byte)167, (byte)51, (byte)237, (byte)82, (byte)105, (byte)192, (byte)115, (byte)14, (byte)240, (byte)91, (byte)83, (byte)245, (byte)95, (byte)198, (byte)194, (byte)74, (byte)98, (byte)101}));
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.data__SET(new byte[] {(byte)129, (byte)179, (byte)86, (byte)76, (byte)233, (byte)174, (byte)186, (byte)44, (byte)91, (byte)224, (byte)175, (byte)90, (byte)211, (byte)70, (byte)55, (byte)19, (byte)235, (byte)145, (byte)150, (byte)186, (byte)147, (byte)12, (byte)162, (byte)41, (byte)91, (byte)7, (byte)233, (byte)69, (byte)19, (byte)66, (byte)180, (byte)250, (byte)223, (byte)91, (byte)151, (byte)94, (byte)46, (byte)55, (byte)25, (byte)119, (byte)40, (byte)211, (byte)45, (byte)200, (byte)52, (byte)201, (byte)115, (byte)153, (byte)246, (byte)105, (byte)139, (byte)194, (byte)26, (byte)200, (byte)215, (byte)254, (byte)125, (byte)27, (byte)218, (byte)142, (byte)122, (byte)118, (byte)123, (byte)101, (byte)40, (byte)143, (byte)33, (byte)102, (byte)77, (byte)50, (byte)238, (byte)222, (byte)195, (byte)230, (byte)56, (byte)97, (byte)181, (byte)104, (byte)5, (byte)214, (byte)83, (byte)227, (byte)43, (byte)223, (byte)70, (byte)191, (byte)85, (byte)251, (byte)155, (byte)171, (byte)124, (byte)92, (byte)79, (byte)111, (byte)175, (byte)58, (byte)118, (byte)135, (byte)237, (byte)132, (byte)227, (byte)70, (byte)112, (byte)26, (byte)203, (byte)132, (byte)51, (byte)253, (byte)127, (byte)140, (byte)152, (byte)12, (byte)93, (byte)95, (byte)81, (byte)100, (byte)35, (byte)105, (byte)108, (byte)17, (byte)50, (byte)94, (byte)24, (byte)0, (byte)81, (byte)221, (byte)2, (byte)136, (byte)29, (byte)18, (byte)10, (byte)7, (byte)60, (byte)206, (byte)185, (byte)191, (byte)232, (byte)187, (byte)43, (byte)236, (byte)152, (byte)227, (byte)26, (byte)78, (byte)214, (byte)172, (byte)58, (byte)202, (byte)223, (byte)153, (byte)1, (byte)120, (byte)24, (byte)62, (byte)206, (byte)2, (byte)96, (byte)27, (byte)155, (byte)164, (byte)215, (byte)82, (byte)231, (byte)31, (byte)58, (byte)250, (byte)173, (byte)22, (byte)178, (byte)140, (byte)151, (byte)84, (byte)77, (byte)221, (byte)124, (byte)166, (byte)137, (byte)179, (byte)29, (byte)31, (byte)145, (byte)223, (byte)156, (byte)236, (byte)65, (byte)234, (byte)198, (byte)92, (byte)100, (byte)77, (byte)110, (byte)4, (byte)25, (byte)112, (byte)202, (byte)64, (byte)37, (byte)41, (byte)119, (byte)156, (byte)255, (byte)232, (byte)28, (byte)56, (byte)17, (byte)167, (byte)197, (byte)253, (byte)39, (byte)112, (byte)37, (byte)10, (byte)54, (byte)229, (byte)241, (byte)38, (byte)226, (byte)52, (byte)180, (byte)24, (byte)25, (byte)80, (byte)226, (byte)236, (byte)5, (byte)211, (byte)164, (byte)164, (byte)105, (byte)236, (byte)196, (byte)209, (byte)121, (byte)130, (byte)189, (byte)167, (byte)51, (byte)237, (byte)82, (byte)105, (byte)192, (byte)115, (byte)14, (byte)240, (byte)91, (byte)83, (byte)245, (byte)95, (byte)198, (byte)194, (byte)74, (byte)98, (byte)101}, 0) ;
            p131.seqnr = (ushort)(ushort)61651;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2234643283U);
                Debug.Assert(pack.current_distance == (ushort)(ushort)63094);
                Debug.Assert(pack.id == (byte)(byte)149);
                Debug.Assert(pack.covariance == (byte)(byte)23);
                Debug.Assert(pack.max_distance == (ushort)(ushort)61641);
                Debug.Assert(pack.min_distance == (ushort)(ushort)41005);
                Debug.Assert(pack.orientation == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_YAW_45);
                Debug.Assert(pack.type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.min_distance = (ushort)(ushort)41005;
            p132.time_boot_ms = (uint)2234643283U;
            p132.id = (byte)(byte)149;
            p132.max_distance = (ushort)(ushort)61641;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p132.covariance = (byte)(byte)23;
            p132.current_distance = (ushort)(ushort)63094;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_YAW_45;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)5432);
                Debug.Assert(pack.lat == (int) -1179177765);
                Debug.Assert(pack.mask == (ulong)2918282545535353795L);
                Debug.Assert(pack.lon == (int)258964784);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int) -1179177765;
            p133.lon = (int)258964784;
            p133.grid_spacing = (ushort)(ushort)5432;
            p133.mask = (ulong)2918282545535353795L;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)225149159);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)17241);
                Debug.Assert(pack.gridbit == (byte)(byte)190);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short)27524, (short) -16762, (short) -11012, (short) -2254, (short)13233, (short)24236, (short) -28926, (short) -27097, (short)12421, (short)27360, (short)31205, (short) -28825, (short) -29099, (short) -11102, (short) -22116, (short) -23574}));
                Debug.Assert(pack.lon == (int)186522851);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.grid_spacing = (ushort)(ushort)17241;
            p134.lat = (int)225149159;
            p134.gridbit = (byte)(byte)190;
            p134.data__SET(new short[] {(short)27524, (short) -16762, (short) -11012, (short) -2254, (short)13233, (short)24236, (short) -28926, (short) -27097, (short)12421, (short)27360, (short)31205, (short) -28825, (short) -29099, (short) -11102, (short) -22116, (short) -23574}, 0) ;
            p134.lon = (int)186522851;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -1186285127);
                Debug.Assert(pack.lon == (int)1725527550);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int) -1186285127;
            p135.lon = (int)1725527550;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.spacing == (ushort)(ushort)37088);
                Debug.Assert(pack.loaded == (ushort)(ushort)43636);
                Debug.Assert(pack.lat == (int)1846706756);
                Debug.Assert(pack.terrain_height == (float)6.035674E37F);
                Debug.Assert(pack.lon == (int) -2117581624);
                Debug.Assert(pack.current_height == (float)2.2244603E38F);
                Debug.Assert(pack.pending == (ushort)(ushort)45013);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.current_height = (float)2.2244603E38F;
            p136.terrain_height = (float)6.035674E37F;
            p136.lat = (int)1846706756;
            p136.lon = (int) -2117581624;
            p136.pending = (ushort)(ushort)45013;
            p136.loaded = (ushort)(ushort)43636;
            p136.spacing = (ushort)(ushort)37088;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -12964);
                Debug.Assert(pack.press_diff == (float) -3.388298E38F);
                Debug.Assert(pack.press_abs == (float)8.902227E37F);
                Debug.Assert(pack.time_boot_ms == (uint)2575443552U);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)2575443552U;
            p137.press_diff = (float) -3.388298E38F;
            p137.press_abs = (float)8.902227E37F;
            p137.temperature = (short)(short) -12964;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {9.127886E37F, 2.8378628E37F, 2.7286757E38F, -1.2868547E38F}));
                Debug.Assert(pack.z == (float)2.9858246E38F);
                Debug.Assert(pack.x == (float) -1.046671E38F);
                Debug.Assert(pack.y == (float)1.722289E38F);
                Debug.Assert(pack.time_usec == (ulong)5189268742291798821L);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.time_usec = (ulong)5189268742291798821L;
            p138.q_SET(new float[] {9.127886E37F, 2.8378628E37F, 2.7286757E38F, -1.2868547E38F}, 0) ;
            p138.y = (float)1.722289E38F;
            p138.x = (float) -1.046671E38F;
            p138.z = (float)2.9858246E38F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)92);
                Debug.Assert(pack.target_component == (byte)(byte)117);
                Debug.Assert(pack.time_usec == (ulong)258553910231333669L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.8688786E38F, 2.8692466E38F, -1.3141245E38F, 1.4961778E38F, 2.6496174E37F, -2.4558866E38F, 1.8601807E38F, 3.04367E38F}));
                Debug.Assert(pack.target_system == (byte)(byte)113);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.controls_SET(new float[] {-1.8688786E38F, 2.8692466E38F, -1.3141245E38F, 1.4961778E38F, 2.6496174E37F, -2.4558866E38F, 1.8601807E38F, 3.04367E38F}, 0) ;
            p139.group_mlx = (byte)(byte)92;
            p139.target_component = (byte)(byte)117;
            p139.target_system = (byte)(byte)113;
            p139.time_usec = (ulong)258553910231333669L;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)44);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {2.7227798E38F, 5.231897E37F, -2.5888989E38F, 2.4810437E38F, 1.0633187E38F, 1.3944876E38F, 6.707305E37F, 2.7969352E38F}));
                Debug.Assert(pack.time_usec == (ulong)6459612948875640404L);
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)6459612948875640404L;
            p140.group_mlx = (byte)(byte)44;
            p140.controls_SET(new float[] {2.7227798E38F, 5.231897E37F, -2.5888989E38F, 2.4810437E38F, 1.0633187E38F, 1.3944876E38F, 6.707305E37F, 2.7969352E38F}, 0) ;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.bottom_clearance == (float)1.1307524E37F);
                Debug.Assert(pack.time_usec == (ulong)373928220312021001L);
                Debug.Assert(pack.altitude_amsl == (float) -3.1636444E38F);
                Debug.Assert(pack.altitude_monotonic == (float)1.1047299E38F);
                Debug.Assert(pack.altitude_relative == (float)2.9244659E38F);
                Debug.Assert(pack.altitude_local == (float)4.9190735E37F);
                Debug.Assert(pack.altitude_terrain == (float) -1.6850037E38F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_amsl = (float) -3.1636444E38F;
            p141.altitude_monotonic = (float)1.1047299E38F;
            p141.altitude_local = (float)4.9190735E37F;
            p141.bottom_clearance = (float)1.1307524E37F;
            p141.altitude_terrain = (float) -1.6850037E38F;
            p141.altitude_relative = (float)2.9244659E38F;
            p141.time_usec = (ulong)373928220312021001L;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)130, (byte)189, (byte)11, (byte)176, (byte)84, (byte)72, (byte)83, (byte)251, (byte)55, (byte)7, (byte)254, (byte)136, (byte)208, (byte)2, (byte)111, (byte)81, (byte)159, (byte)168, (byte)220, (byte)99, (byte)252, (byte)152, (byte)130, (byte)22, (byte)212, (byte)157, (byte)59, (byte)212, (byte)184, (byte)66, (byte)102, (byte)238, (byte)203, (byte)108, (byte)146, (byte)161, (byte)7, (byte)205, (byte)188, (byte)54, (byte)135, (byte)91, (byte)166, (byte)8, (byte)221, (byte)128, (byte)242, (byte)28, (byte)196, (byte)73, (byte)253, (byte)194, (byte)64, (byte)24, (byte)135, (byte)161, (byte)249, (byte)181, (byte)209, (byte)93, (byte)53, (byte)222, (byte)119, (byte)10, (byte)125, (byte)139, (byte)128, (byte)30, (byte)108, (byte)206, (byte)254, (byte)247, (byte)102, (byte)216, (byte)194, (byte)150, (byte)54, (byte)156, (byte)181, (byte)248, (byte)88, (byte)125, (byte)252, (byte)203, (byte)8, (byte)188, (byte)189, (byte)234, (byte)58, (byte)64, (byte)243, (byte)25, (byte)219, (byte)164, (byte)197, (byte)147, (byte)67, (byte)199, (byte)103, (byte)1, (byte)81, (byte)89, (byte)184, (byte)70, (byte)25, (byte)54, (byte)148, (byte)214, (byte)131, (byte)96, (byte)47, (byte)130, (byte)254, (byte)128, (byte)27, (byte)217, (byte)114, (byte)20, (byte)15, (byte)168}));
                Debug.Assert(pack.transfer_type == (byte)(byte)56);
                Debug.Assert(pack.request_id == (byte)(byte)228);
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)130, (byte)230, (byte)179, (byte)183, (byte)144, (byte)34, (byte)73, (byte)191, (byte)89, (byte)129, (byte)5, (byte)203, (byte)164, (byte)154, (byte)121, (byte)239, (byte)188, (byte)199, (byte)114, (byte)63, (byte)217, (byte)100, (byte)138, (byte)17, (byte)183, (byte)223, (byte)120, (byte)38, (byte)60, (byte)68, (byte)95, (byte)81, (byte)94, (byte)113, (byte)179, (byte)163, (byte)138, (byte)181, (byte)111, (byte)72, (byte)176, (byte)56, (byte)126, (byte)93, (byte)178, (byte)12, (byte)92, (byte)19, (byte)230, (byte)34, (byte)110, (byte)99, (byte)26, (byte)71, (byte)241, (byte)206, (byte)166, (byte)181, (byte)171, (byte)170, (byte)93, (byte)247, (byte)232, (byte)132, (byte)65, (byte)67, (byte)9, (byte)116, (byte)172, (byte)173, (byte)91, (byte)180, (byte)205, (byte)171, (byte)181, (byte)66, (byte)180, (byte)65, (byte)10, (byte)17, (byte)91, (byte)109, (byte)111, (byte)196, (byte)227, (byte)80, (byte)47, (byte)28, (byte)230, (byte)111, (byte)202, (byte)250, (byte)6, (byte)186, (byte)146, (byte)125, (byte)27, (byte)226, (byte)128, (byte)80, (byte)174, (byte)51, (byte)80, (byte)116, (byte)134, (byte)113, (byte)200, (byte)76, (byte)126, (byte)57, (byte)181, (byte)16, (byte)64, (byte)31, (byte)49, (byte)2, (byte)59, (byte)228, (byte)208, (byte)45}));
                Debug.Assert(pack.uri_type == (byte)(byte)135);
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.transfer_type = (byte)(byte)56;
            p142.request_id = (byte)(byte)228;
            p142.uri_type = (byte)(byte)135;
            p142.storage_SET(new byte[] {(byte)130, (byte)230, (byte)179, (byte)183, (byte)144, (byte)34, (byte)73, (byte)191, (byte)89, (byte)129, (byte)5, (byte)203, (byte)164, (byte)154, (byte)121, (byte)239, (byte)188, (byte)199, (byte)114, (byte)63, (byte)217, (byte)100, (byte)138, (byte)17, (byte)183, (byte)223, (byte)120, (byte)38, (byte)60, (byte)68, (byte)95, (byte)81, (byte)94, (byte)113, (byte)179, (byte)163, (byte)138, (byte)181, (byte)111, (byte)72, (byte)176, (byte)56, (byte)126, (byte)93, (byte)178, (byte)12, (byte)92, (byte)19, (byte)230, (byte)34, (byte)110, (byte)99, (byte)26, (byte)71, (byte)241, (byte)206, (byte)166, (byte)181, (byte)171, (byte)170, (byte)93, (byte)247, (byte)232, (byte)132, (byte)65, (byte)67, (byte)9, (byte)116, (byte)172, (byte)173, (byte)91, (byte)180, (byte)205, (byte)171, (byte)181, (byte)66, (byte)180, (byte)65, (byte)10, (byte)17, (byte)91, (byte)109, (byte)111, (byte)196, (byte)227, (byte)80, (byte)47, (byte)28, (byte)230, (byte)111, (byte)202, (byte)250, (byte)6, (byte)186, (byte)146, (byte)125, (byte)27, (byte)226, (byte)128, (byte)80, (byte)174, (byte)51, (byte)80, (byte)116, (byte)134, (byte)113, (byte)200, (byte)76, (byte)126, (byte)57, (byte)181, (byte)16, (byte)64, (byte)31, (byte)49, (byte)2, (byte)59, (byte)228, (byte)208, (byte)45}, 0) ;
            p142.uri_SET(new byte[] {(byte)130, (byte)189, (byte)11, (byte)176, (byte)84, (byte)72, (byte)83, (byte)251, (byte)55, (byte)7, (byte)254, (byte)136, (byte)208, (byte)2, (byte)111, (byte)81, (byte)159, (byte)168, (byte)220, (byte)99, (byte)252, (byte)152, (byte)130, (byte)22, (byte)212, (byte)157, (byte)59, (byte)212, (byte)184, (byte)66, (byte)102, (byte)238, (byte)203, (byte)108, (byte)146, (byte)161, (byte)7, (byte)205, (byte)188, (byte)54, (byte)135, (byte)91, (byte)166, (byte)8, (byte)221, (byte)128, (byte)242, (byte)28, (byte)196, (byte)73, (byte)253, (byte)194, (byte)64, (byte)24, (byte)135, (byte)161, (byte)249, (byte)181, (byte)209, (byte)93, (byte)53, (byte)222, (byte)119, (byte)10, (byte)125, (byte)139, (byte)128, (byte)30, (byte)108, (byte)206, (byte)254, (byte)247, (byte)102, (byte)216, (byte)194, (byte)150, (byte)54, (byte)156, (byte)181, (byte)248, (byte)88, (byte)125, (byte)252, (byte)203, (byte)8, (byte)188, (byte)189, (byte)234, (byte)58, (byte)64, (byte)243, (byte)25, (byte)219, (byte)164, (byte)197, (byte)147, (byte)67, (byte)199, (byte)103, (byte)1, (byte)81, (byte)89, (byte)184, (byte)70, (byte)25, (byte)54, (byte)148, (byte)214, (byte)131, (byte)96, (byte)47, (byte)130, (byte)254, (byte)128, (byte)27, (byte)217, (byte)114, (byte)20, (byte)15, (byte)168}, 0) ;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float) -2.7263244E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2671981134U);
                Debug.Assert(pack.press_diff == (float) -1.6103226E38F);
                Debug.Assert(pack.temperature == (short)(short) -4326);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.press_abs = (float) -2.7263244E38F;
            p143.time_boot_ms = (uint)2671981134U;
            p143.temperature = (short)(short) -4326;
            p143.press_diff = (float) -1.6103226E38F;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {3.261075E37F, -3.0631521E38F, 1.9046535E38F, -3.1504872E38F}));
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-2.1794321E38F, 8.709868E37F, -2.6995471E38F}));
                Debug.Assert(pack.est_capabilities == (byte)(byte)221);
                Debug.Assert(pack.timestamp == (ulong)4963862457569145949L);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {-2.3487355E38F, -9.093787E37F, -3.3980186E38F}));
                Debug.Assert(pack.alt == (float)2.5312719E38F);
                Debug.Assert(pack.custom_state == (ulong)5675752467311262960L);
                Debug.Assert(pack.lon == (int)1521016717);
                Debug.Assert(pack.lat == (int) -1169658911);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-1.3177695E38F, -7.955756E37F, 2.078333E38F}));
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {-1.9664697E38F, 1.8828193E38F, 1.7888842E38F}));
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.acc_SET(new float[] {-2.3487355E38F, -9.093787E37F, -3.3980186E38F}, 0) ;
            p144.lat = (int) -1169658911;
            p144.alt = (float)2.5312719E38F;
            p144.vel_SET(new float[] {-2.1794321E38F, 8.709868E37F, -2.6995471E38F}, 0) ;
            p144.custom_state = (ulong)5675752467311262960L;
            p144.timestamp = (ulong)4963862457569145949L;
            p144.lon = (int)1521016717;
            p144.position_cov_SET(new float[] {-1.9664697E38F, 1.8828193E38F, 1.7888842E38F}, 0) ;
            p144.rates_SET(new float[] {-1.3177695E38F, -7.955756E37F, 2.078333E38F}, 0) ;
            p144.est_capabilities = (byte)(byte)221;
            p144.attitude_q_SET(new float[] {3.261075E37F, -3.0631521E38F, 1.9046535E38F, -3.1504872E38F}, 0) ;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw_rate == (float)2.2502216E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {5.769415E37F, 7.795548E37F, 1.5544045E38F}));
                Debug.Assert(pack.roll_rate == (float) -1.5035368E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.1891207E38F, -5.0096527E37F, -2.994413E38F, -3.3235113E38F}));
                Debug.Assert(pack.time_usec == (ulong)1327959564557086361L);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {1.930367E38F, 5.922667E37F, 3.228537E38F}));
                Debug.Assert(pack.z_pos == (float) -2.960843E38F);
                Debug.Assert(pack.z_acc == (float) -2.4776073E38F);
                Debug.Assert(pack.y_pos == (float) -3.2473738E37F);
                Debug.Assert(pack.y_acc == (float) -3.0269223E38F);
                Debug.Assert(pack.airspeed == (float)3.2153442E38F);
                Debug.Assert(pack.pitch_rate == (float) -1.9521718E38F);
                Debug.Assert(pack.z_vel == (float) -2.1341715E38F);
                Debug.Assert(pack.x_acc == (float)2.0211147E38F);
                Debug.Assert(pack.x_vel == (float) -2.2374768E37F);
                Debug.Assert(pack.y_vel == (float)1.8303382E38F);
                Debug.Assert(pack.x_pos == (float)1.8886864E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.yaw_rate = (float)2.2502216E38F;
            p146.y_pos = (float) -3.2473738E37F;
            p146.roll_rate = (float) -1.5035368E37F;
            p146.airspeed = (float)3.2153442E38F;
            p146.pitch_rate = (float) -1.9521718E38F;
            p146.pos_variance_SET(new float[] {1.930367E38F, 5.922667E37F, 3.228537E38F}, 0) ;
            p146.z_pos = (float) -2.960843E38F;
            p146.z_vel = (float) -2.1341715E38F;
            p146.y_vel = (float)1.8303382E38F;
            p146.q_SET(new float[] {3.1891207E38F, -5.0096527E37F, -2.994413E38F, -3.3235113E38F}, 0) ;
            p146.x_pos = (float)1.8886864E38F;
            p146.vel_variance_SET(new float[] {5.769415E37F, 7.795548E37F, 1.5544045E38F}, 0) ;
            p146.x_acc = (float)2.0211147E38F;
            p146.x_vel = (float) -2.2374768E37F;
            p146.time_usec = (ulong)1327959564557086361L;
            p146.y_acc = (float) -3.0269223E38F;
            p146.z_acc = (float) -2.4776073E38F;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (byte)(byte)224);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)4930, (ushort)18505, (ushort)8829, (ushort)63879, (ushort)38642, (ushort)55885, (ushort)53390, (ushort)25929, (ushort)10766, (ushort)28249}));
                Debug.Assert(pack.battery_function == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION);
                Debug.Assert(pack.type == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO);
                Debug.Assert(pack.temperature == (short)(short) -8963);
                Debug.Assert(pack.current_battery == (short)(short)15020);
                Debug.Assert(pack.current_consumed == (int) -1371667411);
                Debug.Assert(pack.energy_consumed == (int) -731053841);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 30);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.temperature = (short)(short) -8963;
            p147.id = (byte)(byte)224;
            p147.current_consumed = (int) -1371667411;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION;
            p147.battery_remaining = (sbyte)(sbyte) - 30;
            p147.current_battery = (short)(short)15020;
            p147.voltages_SET(new ushort[] {(ushort)4930, (ushort)18505, (ushort)8829, (ushort)63879, (ushort)38642, (ushort)55885, (ushort)53390, (ushort)25929, (ushort)10766, (ushort)28249}, 0) ;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO;
            p147.energy_consumed = (int) -731053841;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flight_sw_version == (uint)1264286848U);
                Debug.Assert(pack.product_id == (ushort)(ushort)53348);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)2582);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION));
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)183, (byte)126, (byte)185, (byte)104, (byte)174, (byte)187, (byte)227, (byte)250}));
                Debug.Assert(pack.middleware_sw_version == (uint)608874947U);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)179, (byte)117, (byte)170, (byte)65, (byte)71, (byte)7, (byte)34, (byte)38}));
                Debug.Assert(pack.uid == (ulong)3673481036722343376L);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)115, (byte)216, (byte)175, (byte)205, (byte)150, (byte)218, (byte)201, (byte)128, (byte)219, (byte)107, (byte)205, (byte)116, (byte)243, (byte)193, (byte)216, (byte)153, (byte)251, (byte)59}));
                Debug.Assert(pack.os_sw_version == (uint)1956372040U);
                Debug.Assert(pack.board_version == (uint)51992362U);
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)155, (byte)15, (byte)48, (byte)41, (byte)98, (byte)154, (byte)107, (byte)105}));
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.os_custom_version_SET(new byte[] {(byte)155, (byte)15, (byte)48, (byte)41, (byte)98, (byte)154, (byte)107, (byte)105}, 0) ;
            p148.vendor_id = (ushort)(ushort)2582;
            p148.flight_sw_version = (uint)1264286848U;
            p148.middleware_sw_version = (uint)608874947U;
            p148.middleware_custom_version_SET(new byte[] {(byte)179, (byte)117, (byte)170, (byte)65, (byte)71, (byte)7, (byte)34, (byte)38}, 0) ;
            p148.flight_custom_version_SET(new byte[] {(byte)183, (byte)126, (byte)185, (byte)104, (byte)174, (byte)187, (byte)227, (byte)250}, 0) ;
            p148.uid = (ulong)3673481036722343376L;
            p148.uid2_SET(new byte[] {(byte)115, (byte)216, (byte)175, (byte)205, (byte)150, (byte)218, (byte)201, (byte)128, (byte)219, (byte)107, (byte)205, (byte)116, (byte)243, (byte)193, (byte)216, (byte)153, (byte)251, (byte)59}, 0, PH) ;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION);
            p148.product_id = (ushort)(ushort)53348;
            p148.os_sw_version = (uint)1956372040U;
            p148.board_version = (uint)51992362U;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x_TRY(ph) == (float) -6.4742957E37F);
                Debug.Assert(pack.y_TRY(ph) == (float)9.376521E37F);
                Debug.Assert(pack.time_usec == (ulong)8747617312849197944L);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {2.9121796E38F, 2.7617571E38F, -1.4797269E38F, 3.1242387E38F}));
                Debug.Assert(pack.size_y == (float) -3.1141322E38F);
                Debug.Assert(pack.angle_x == (float)9.313002E35F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.distance == (float)1.5757917E38F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)0);
                Debug.Assert(pack.z_TRY(ph) == (float) -2.3166903E38F);
                Debug.Assert(pack.target_num == (byte)(byte)42);
                Debug.Assert(pack.size_x == (float) -2.6753679E38F);
                Debug.Assert(pack.angle_y == (float) -7.8094224E37F);
                Debug.Assert(pack.type == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.size_y = (float) -3.1141322E38F;
            p149.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p149.z_SET((float) -2.3166903E38F, PH) ;
            p149.angle_y = (float) -7.8094224E37F;
            p149.distance = (float)1.5757917E38F;
            p149.position_valid_SET((byte)(byte)0, PH) ;
            p149.size_x = (float) -2.6753679E38F;
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.x_SET((float) -6.4742957E37F, PH) ;
            p149.time_usec = (ulong)8747617312849197944L;
            p149.target_num = (byte)(byte)42;
            p149.angle_x = (float)9.313002E35F;
            p149.q_SET(new float[] {2.9121796E38F, 2.7617571E38F, -1.4797269E38F, 3.1242387E38F}, 0, PH) ;
            p149.y_SET((float)9.376521E37F, PH) ;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENS_POWERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.adc121_cspb_amp == (float)9.540742E37F);
                Debug.Assert(pack.adc121_vspb_volt == (float)2.965396E38F);
                Debug.Assert(pack.adc121_cs1_amp == (float) -9.301946E37F);
                Debug.Assert(pack.adc121_cs2_amp == (float) -1.722312E38F);
            };
            GroundControl.SENS_POWER p201 = CommunicationChannel.new_SENS_POWER();
            PH.setPack(p201);
            p201.adc121_cs1_amp = (float) -9.301946E37F;
            p201.adc121_cs2_amp = (float) -1.722312E38F;
            p201.adc121_vspb_volt = (float)2.965396E38F;
            p201.adc121_cspb_amp = (float)9.540742E37F;
            CommunicationChannel.instance.send(p201);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENS_MPPTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mppt3_amp == (float)1.1479878E36F);
                Debug.Assert(pack.mppt2_amp == (float) -2.0932246E38F);
                Debug.Assert(pack.mppt3_pwm == (ushort)(ushort)27065);
                Debug.Assert(pack.mppt2_volt == (float)2.8956649E37F);
                Debug.Assert(pack.mppt1_status == (byte)(byte)2);
                Debug.Assert(pack.mppt1_volt == (float)2.9550374E38F);
                Debug.Assert(pack.mppt_timestamp == (ulong)2735187402110156251L);
                Debug.Assert(pack.mppt3_volt == (float) -1.8634209E37F);
                Debug.Assert(pack.mppt3_status == (byte)(byte)165);
                Debug.Assert(pack.mppt1_pwm == (ushort)(ushort)209);
                Debug.Assert(pack.mppt2_pwm == (ushort)(ushort)60419);
                Debug.Assert(pack.mppt1_amp == (float)2.180514E37F);
                Debug.Assert(pack.mppt2_status == (byte)(byte)113);
            };
            GroundControl.SENS_MPPT p202 = CommunicationChannel.new_SENS_MPPT();
            PH.setPack(p202);
            p202.mppt2_pwm = (ushort)(ushort)60419;
            p202.mppt3_volt = (float) -1.8634209E37F;
            p202.mppt1_volt = (float)2.9550374E38F;
            p202.mppt3_amp = (float)1.1479878E36F;
            p202.mppt2_amp = (float) -2.0932246E38F;
            p202.mppt1_amp = (float)2.180514E37F;
            p202.mppt2_status = (byte)(byte)113;
            p202.mppt3_status = (byte)(byte)165;
            p202.mppt2_volt = (float)2.8956649E37F;
            p202.mppt_timestamp = (ulong)2735187402110156251L;
            p202.mppt1_pwm = (ushort)(ushort)209;
            p202.mppt3_pwm = (ushort)(ushort)27065;
            p202.mppt1_status = (byte)(byte)2;
            CommunicationChannel.instance.send(p202);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnASLCTRL_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uThrot == (float) -2.183973E38F);
                Debug.Assert(pack.PitchAngle == (float) -1.135331E37F);
                Debug.Assert(pack.aslctrl_mode == (byte)(byte)30);
                Debug.Assert(pack.uElev == (float) -3.2774774E38F);
                Debug.Assert(pack.uThrot2 == (float)2.6384113E38F);
                Debug.Assert(pack.hRef == (float) -3.273634E37F);
                Debug.Assert(pack.SpoilersEngaged == (byte)(byte)99);
                Debug.Assert(pack.AirspeedRef == (float) -4.5901623E37F);
                Debug.Assert(pack.pRef == (float) -3.8104316E36F);
                Debug.Assert(pack.YawAngleRef == (float)3.1118145E38F);
                Debug.Assert(pack.YawAngle == (float)1.2341737E38F);
                Debug.Assert(pack.rRef == (float)1.4244577E38F);
                Debug.Assert(pack.h == (float)6.207422E37F);
                Debug.Assert(pack.p == (float) -1.732599E38F);
                Debug.Assert(pack.r == (float) -3.2684884E38F);
                Debug.Assert(pack.uAil == (float) -1.5798078E38F);
                Debug.Assert(pack.nZ == (float)2.9333847E38F);
                Debug.Assert(pack.uRud == (float) -3.229677E38F);
                Debug.Assert(pack.hRef_t == (float) -1.1512489E38F);
                Debug.Assert(pack.PitchAngleRef == (float)2.3952623E37F);
                Debug.Assert(pack.RollAngle == (float) -1.6624401E38F);
                Debug.Assert(pack.RollAngleRef == (float)1.931183E38F);
                Debug.Assert(pack.q == (float) -3.1479815E38F);
                Debug.Assert(pack.qRef == (float)2.5831957E38F);
                Debug.Assert(pack.timestamp == (ulong)5061588116243393927L);
            };
            GroundControl.ASLCTRL_DATA p203 = CommunicationChannel.new_ASLCTRL_DATA();
            PH.setPack(p203);
            p203.hRef = (float) -3.273634E37F;
            p203.r = (float) -3.2684884E38F;
            p203.nZ = (float)2.9333847E38F;
            p203.AirspeedRef = (float) -4.5901623E37F;
            p203.RollAngleRef = (float)1.931183E38F;
            p203.rRef = (float)1.4244577E38F;
            p203.uElev = (float) -3.2774774E38F;
            p203.p = (float) -1.732599E38F;
            p203.uThrot = (float) -2.183973E38F;
            p203.uRud = (float) -3.229677E38F;
            p203.q = (float) -3.1479815E38F;
            p203.uThrot2 = (float)2.6384113E38F;
            p203.SpoilersEngaged = (byte)(byte)99;
            p203.PitchAngle = (float) -1.135331E37F;
            p203.PitchAngleRef = (float)2.3952623E37F;
            p203.YawAngleRef = (float)3.1118145E38F;
            p203.YawAngle = (float)1.2341737E38F;
            p203.hRef_t = (float) -1.1512489E38F;
            p203.timestamp = (ulong)5061588116243393927L;
            p203.pRef = (float) -3.8104316E36F;
            p203.uAil = (float) -1.5798078E38F;
            p203.qRef = (float)2.5831957E38F;
            p203.h = (float)6.207422E37F;
            p203.aslctrl_mode = (byte)(byte)30;
            p203.RollAngle = (float) -1.6624401E38F;
            CommunicationChannel.instance.send(p203);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnASLCTRL_DEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.f_2 == (float)3.231296E38F);
                Debug.Assert(pack.f_3 == (float)2.1313967E38F);
                Debug.Assert(pack.f_4 == (float) -2.6685189E37F);
                Debug.Assert(pack.i32_1 == (uint)3989390656U);
                Debug.Assert(pack.f_6 == (float)1.829994E38F);
                Debug.Assert(pack.i8_1 == (byte)(byte)170);
                Debug.Assert(pack.f_8 == (float)1.8246461E38F);
                Debug.Assert(pack.i8_2 == (byte)(byte)163);
                Debug.Assert(pack.f_1 == (float) -2.3643458E38F);
                Debug.Assert(pack.f_5 == (float)3.1312448E38F);
                Debug.Assert(pack.f_7 == (float)4.5624697E36F);
            };
            GroundControl.ASLCTRL_DEBUG p204 = CommunicationChannel.new_ASLCTRL_DEBUG();
            PH.setPack(p204);
            p204.f_8 = (float)1.8246461E38F;
            p204.i8_1 = (byte)(byte)170;
            p204.i8_2 = (byte)(byte)163;
            p204.f_3 = (float)2.1313967E38F;
            p204.f_2 = (float)3.231296E38F;
            p204.f_1 = (float) -2.3643458E38F;
            p204.f_7 = (float)4.5624697E36F;
            p204.f_4 = (float) -2.6685189E37F;
            p204.f_5 = (float)3.1312448E38F;
            p204.f_6 = (float)1.829994E38F;
            p204.i32_1 = (uint)3989390656U;
            CommunicationChannel.instance.send(p204);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnASLUAV_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Motor_rpm == (float) -2.341079E38F);
                Debug.Assert(pack.SATCOM_status == (byte)(byte)46);
                Debug.Assert(pack.LED_status == (byte)(byte)212);
                Debug.Assert(pack.Servo_status.SequenceEqual(new byte[] {(byte)190, (byte)136, (byte)5, (byte)92, (byte)11, (byte)48, (byte)221, (byte)133}));
            };
            GroundControl.ASLUAV_STATUS p205 = CommunicationChannel.new_ASLUAV_STATUS();
            PH.setPack(p205);
            p205.LED_status = (byte)(byte)212;
            p205.Motor_rpm = (float) -2.341079E38F;
            p205.Servo_status_SET(new byte[] {(byte)190, (byte)136, (byte)5, (byte)92, (byte)11, (byte)48, (byte)221, (byte)133}, 0) ;
            p205.SATCOM_status = (byte)(byte)46;
            CommunicationChannel.instance.send(p205);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnEKF_EXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Airspeed == (float)3.8396848E37F);
                Debug.Assert(pack.WindDir == (float) -2.8673298E37F);
                Debug.Assert(pack.beta == (float)2.9559971E38F);
                Debug.Assert(pack.timestamp == (ulong)2358492641467100257L);
                Debug.Assert(pack.Windspeed == (float)1.6811529E38F);
                Debug.Assert(pack.WindZ == (float)3.2444561E37F);
                Debug.Assert(pack.alpha == (float)1.6367771E38F);
            };
            GroundControl.EKF_EXT p206 = CommunicationChannel.new_EKF_EXT();
            PH.setPack(p206);
            p206.WindDir = (float) -2.8673298E37F;
            p206.beta = (float)2.9559971E38F;
            p206.WindZ = (float)3.2444561E37F;
            p206.Windspeed = (float)1.6811529E38F;
            p206.alpha = (float)1.6367771E38F;
            p206.timestamp = (ulong)2358492641467100257L;
            p206.Airspeed = (float)3.8396848E37F;
            CommunicationChannel.instance.send(p206);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnASL_OBCTRLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uThrot2 == (float) -1.6568675E38F);
                Debug.Assert(pack.timestamp == (ulong)6634539519765524809L);
                Debug.Assert(pack.uElev == (float) -2.9507379E38F);
                Debug.Assert(pack.uAilL == (float) -2.9388548E38F);
                Debug.Assert(pack.uThrot == (float) -2.4639169E38F);
                Debug.Assert(pack.uRud == (float) -4.0048482E37F);
                Debug.Assert(pack.uAilR == (float) -2.4971804E38F);
                Debug.Assert(pack.obctrl_status == (byte)(byte)46);
            };
            GroundControl.ASL_OBCTRL p207 = CommunicationChannel.new_ASL_OBCTRL();
            PH.setPack(p207);
            p207.uRud = (float) -4.0048482E37F;
            p207.uAilR = (float) -2.4971804E38F;
            p207.uElev = (float) -2.9507379E38F;
            p207.uAilL = (float) -2.9388548E38F;
            p207.uThrot2 = (float) -1.6568675E38F;
            p207.obctrl_status = (byte)(byte)46;
            p207.uThrot = (float) -2.4639169E38F;
            p207.timestamp = (ulong)6634539519765524809L;
            CommunicationChannel.instance.send(p207);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENS_ATMOSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.TempAmbient == (float) -1.4576841E38F);
                Debug.Assert(pack.Humidity == (float)2.5921289E38F);
            };
            GroundControl.SENS_ATMOS p208 = CommunicationChannel.new_SENS_ATMOS();
            PH.setPack(p208);
            p208.Humidity = (float)2.5921289E38F;
            p208.TempAmbient = (float) -1.4576841E38F;
            CommunicationChannel.instance.send(p208);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENS_BATMONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cellvoltage1 == (ushort)(ushort)131);
                Debug.Assert(pack.cellvoltage2 == (ushort)(ushort)13180);
                Debug.Assert(pack.temperature == (float)2.1312042E38F);
                Debug.Assert(pack.batterystatus == (ushort)(ushort)46649);
                Debug.Assert(pack.cellvoltage5 == (ushort)(ushort)50624);
                Debug.Assert(pack.cellvoltage3 == (ushort)(ushort)27624);
                Debug.Assert(pack.cellvoltage4 == (ushort)(ushort)13790);
                Debug.Assert(pack.hostfetcontrol == (ushort)(ushort)10812);
                Debug.Assert(pack.voltage == (ushort)(ushort)18847);
                Debug.Assert(pack.cellvoltage6 == (ushort)(ushort)16289);
                Debug.Assert(pack.current == (short)(short) -13998);
                Debug.Assert(pack.serialnumber == (ushort)(ushort)10420);
                Debug.Assert(pack.SoC == (byte)(byte)76);
            };
            GroundControl.SENS_BATMON p209 = CommunicationChannel.new_SENS_BATMON();
            PH.setPack(p209);
            p209.SoC = (byte)(byte)76;
            p209.voltage = (ushort)(ushort)18847;
            p209.temperature = (float)2.1312042E38F;
            p209.cellvoltage2 = (ushort)(ushort)13180;
            p209.cellvoltage6 = (ushort)(ushort)16289;
            p209.batterystatus = (ushort)(ushort)46649;
            p209.cellvoltage4 = (ushort)(ushort)13790;
            p209.cellvoltage3 = (ushort)(ushort)27624;
            p209.hostfetcontrol = (ushort)(ushort)10812;
            p209.cellvoltage5 = (ushort)(ushort)50624;
            p209.current = (short)(short) -13998;
            p209.cellvoltage1 = (ushort)(ushort)131;
            p209.serialnumber = (ushort)(ushort)10420;
            CommunicationChannel.instance.send(p209);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFW_SOARING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.VarW == (float) -1.1239035E38F);
                Debug.Assert(pack.DebugVar1 == (float) -7.856613E37F);
                Debug.Assert(pack.vSinkExp == (float)9.002546E37F);
                Debug.Assert(pack.z2_exp == (float)1.0567513E38F);
                Debug.Assert(pack.ThermalGSNorth == (float)9.025384E37F);
                Debug.Assert(pack.valid == (byte)(byte)198);
                Debug.Assert(pack.xR == (float) -2.4242282E38F);
                Debug.Assert(pack.xW == (float) -3.3035827E38F);
                Debug.Assert(pack.DistToSoarPoint == (float)5.036299E37F);
                Debug.Assert(pack.LoiterDirection == (float) -6.132431E36F);
                Debug.Assert(pack.VarLon == (float) -1.1614555E38F);
                Debug.Assert(pack.xLat == (float)4.660725E37F);
                Debug.Assert(pack.LoiterRadius == (float) -2.4781456E38F);
                Debug.Assert(pack.VarLat == (float)1.0306186E38F);
                Debug.Assert(pack.DebugVar2 == (float)2.1598894E38F);
                Debug.Assert(pack.z2_DeltaRoll == (float) -1.6202732E38F);
                Debug.Assert(pack.VarR == (float) -1.1309322E38F);
                Debug.Assert(pack.ThermalGSEast == (float) -3.1173982E38F);
                Debug.Assert(pack.TSE_dot == (float) -7.930711E37F);
                Debug.Assert(pack.xLon == (float) -3.309348E38F);
                Debug.Assert(pack.timestampModeChanged == (ulong)1220971338414409877L);
                Debug.Assert(pack.z1_exp == (float)1.8186622E38F);
                Debug.Assert(pack.ControlMode == (byte)(byte)53);
                Debug.Assert(pack.z1_LocalUpdraftSpeed == (float)6.62129E37F);
                Debug.Assert(pack.timestamp == (ulong)869427439692295336L);
            };
            GroundControl.FW_SOARING_DATA p210 = CommunicationChannel.new_FW_SOARING_DATA();
            PH.setPack(p210);
            p210.VarLon = (float) -1.1614555E38F;
            p210.xR = (float) -2.4242282E38F;
            p210.vSinkExp = (float)9.002546E37F;
            p210.timestamp = (ulong)869427439692295336L;
            p210.ThermalGSNorth = (float)9.025384E37F;
            p210.z1_LocalUpdraftSpeed = (float)6.62129E37F;
            p210.LoiterDirection = (float) -6.132431E36F;
            p210.VarR = (float) -1.1309322E38F;
            p210.DistToSoarPoint = (float)5.036299E37F;
            p210.z2_DeltaRoll = (float) -1.6202732E38F;
            p210.z1_exp = (float)1.8186622E38F;
            p210.xLon = (float) -3.309348E38F;
            p210.ControlMode = (byte)(byte)53;
            p210.TSE_dot = (float) -7.930711E37F;
            p210.xW = (float) -3.3035827E38F;
            p210.xLat = (float)4.660725E37F;
            p210.DebugVar2 = (float)2.1598894E38F;
            p210.LoiterRadius = (float) -2.4781456E38F;
            p210.DebugVar1 = (float) -7.856613E37F;
            p210.VarW = (float) -1.1239035E38F;
            p210.timestampModeChanged = (ulong)1220971338414409877L;
            p210.VarLat = (float)1.0306186E38F;
            p210.valid = (byte)(byte)198;
            p210.z2_exp = (float)1.0567513E38F;
            p210.ThermalGSEast = (float) -3.1173982E38F;
            CommunicationChannel.instance.send(p210);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENSORPOD_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.timestamp == (ulong)173532880065895855L);
                Debug.Assert(pack.visensor_rate_1 == (byte)(byte)235);
                Debug.Assert(pack.visensor_rate_3 == (byte)(byte)64);
                Debug.Assert(pack.recording_nodes_count == (byte)(byte)208);
                Debug.Assert(pack.free_space == (ushort)(ushort)64074);
                Debug.Assert(pack.visensor_rate_2 == (byte)(byte)101);
                Debug.Assert(pack.cpu_temp == (byte)(byte)37);
                Debug.Assert(pack.visensor_rate_4 == (byte)(byte)177);
            };
            GroundControl.SENSORPOD_STATUS p211 = CommunicationChannel.new_SENSORPOD_STATUS();
            PH.setPack(p211);
            p211.recording_nodes_count = (byte)(byte)208;
            p211.visensor_rate_2 = (byte)(byte)101;
            p211.visensor_rate_4 = (byte)(byte)177;
            p211.visensor_rate_1 = (byte)(byte)235;
            p211.visensor_rate_3 = (byte)(byte)64;
            p211.timestamp = (ulong)173532880065895855L;
            p211.cpu_temp = (byte)(byte)37;
            p211.free_space = (ushort)(ushort)64074;
            CommunicationChannel.instance.send(p211);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENS_POWER_BOARDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pwr_brd_servo_2_amp == (float) -3.5014928E37F);
                Debug.Assert(pack.pwr_brd_system_volt == (float)2.1903883E38F);
                Debug.Assert(pack.pwr_brd_servo_volt == (float)1.2037734E38F);
                Debug.Assert(pack.timestamp == (ulong)3373768307793219433L);
                Debug.Assert(pack.pwr_brd_status == (byte)(byte)124);
                Debug.Assert(pack.pwr_brd_mot_l_amp == (float) -2.009976E38F);
                Debug.Assert(pack.pwr_brd_servo_1_amp == (float)1.7427135E38F);
                Debug.Assert(pack.pwr_brd_led_status == (byte)(byte)23);
                Debug.Assert(pack.pwr_brd_servo_3_amp == (float)7.6785146E37F);
                Debug.Assert(pack.pwr_brd_mot_r_amp == (float)1.970647E38F);
                Debug.Assert(pack.pwr_brd_servo_4_amp == (float) -1.4321905E37F);
                Debug.Assert(pack.pwr_brd_aux_amp == (float) -2.5082735E38F);
            };
            GroundControl.SENS_POWER_BOARD p212 = CommunicationChannel.new_SENS_POWER_BOARD();
            PH.setPack(p212);
            p212.pwr_brd_servo_3_amp = (float)7.6785146E37F;
            p212.pwr_brd_mot_r_amp = (float)1.970647E38F;
            p212.pwr_brd_servo_4_amp = (float) -1.4321905E37F;
            p212.pwr_brd_aux_amp = (float) -2.5082735E38F;
            p212.pwr_brd_servo_1_amp = (float)1.7427135E38F;
            p212.pwr_brd_servo_volt = (float)1.2037734E38F;
            p212.pwr_brd_system_volt = (float)2.1903883E38F;
            p212.timestamp = (ulong)3373768307793219433L;
            p212.pwr_brd_servo_2_amp = (float) -3.5014928E37F;
            p212.pwr_brd_mot_l_amp = (float) -2.009976E38F;
            p212.pwr_brd_led_status = (byte)(byte)23;
            p212.pwr_brd_status = (byte)(byte)124;
            CommunicationChannel.instance.send(p212);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mag_ratio == (float)1.9250954E38F);
                Debug.Assert(pack.time_usec == (ulong)2084714207374573942L);
                Debug.Assert(pack.hagl_ratio == (float)1.2969256E38F);
                Debug.Assert(pack.pos_vert_ratio == (float)8.560504E37F);
                Debug.Assert(pack.vel_ratio == (float)7.444783E37F);
                Debug.Assert(pack.pos_horiz_ratio == (float) -1.8834724E38F);
                Debug.Assert(pack.tas_ratio == (float)2.7111872E38F);
                Debug.Assert(pack.pos_vert_accuracy == (float)8.949732E37F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH));
                Debug.Assert(pack.pos_horiz_accuracy == (float)3.3698996E38F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.pos_vert_accuracy = (float)8.949732E37F;
            p230.tas_ratio = (float)2.7111872E38F;
            p230.pos_vert_ratio = (float)8.560504E37F;
            p230.hagl_ratio = (float)1.2969256E38F;
            p230.pos_horiz_accuracy = (float)3.3698996E38F;
            p230.mag_ratio = (float)1.9250954E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH);
            p230.pos_horiz_ratio = (float) -1.8834724E38F;
            p230.time_usec = (ulong)2084714207374573942L;
            p230.vel_ratio = (float)7.444783E37F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.horiz_accuracy == (float)2.34868E37F);
                Debug.Assert(pack.wind_alt == (float)7.34531E37F);
                Debug.Assert(pack.var_vert == (float) -2.6885656E38F);
                Debug.Assert(pack.vert_accuracy == (float)2.8630668E38F);
                Debug.Assert(pack.time_usec == (ulong)9113192565652974771L);
                Debug.Assert(pack.wind_x == (float) -1.4569758E38F);
                Debug.Assert(pack.var_horiz == (float) -2.0667112E38F);
                Debug.Assert(pack.wind_y == (float)5.7313207E37F);
                Debug.Assert(pack.wind_z == (float)3.974588E37F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.horiz_accuracy = (float)2.34868E37F;
            p231.wind_y = (float)5.7313207E37F;
            p231.time_usec = (ulong)9113192565652974771L;
            p231.wind_x = (float) -1.4569758E38F;
            p231.var_vert = (float) -2.6885656E38F;
            p231.var_horiz = (float) -2.0667112E38F;
            p231.wind_alt = (float)7.34531E37F;
            p231.wind_z = (float)3.974588E37F;
            p231.vert_accuracy = (float)2.8630668E38F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vert_accuracy == (float) -1.6311434E38F);
                Debug.Assert(pack.satellites_visible == (byte)(byte)67);
                Debug.Assert(pack.ve == (float)2.801337E38F);
                Debug.Assert(pack.vd == (float)1.0680512E38F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY));
                Debug.Assert(pack.lon == (int) -1034178888);
                Debug.Assert(pack.time_week_ms == (uint)1974595257U);
                Debug.Assert(pack.horiz_accuracy == (float) -2.3930006E38F);
                Debug.Assert(pack.gps_id == (byte)(byte)6);
                Debug.Assert(pack.fix_type == (byte)(byte)18);
                Debug.Assert(pack.time_usec == (ulong)5818003729088803646L);
                Debug.Assert(pack.alt == (float) -5.5730454E37F);
                Debug.Assert(pack.lat == (int)769958119);
                Debug.Assert(pack.speed_accuracy == (float)1.7583336E38F);
                Debug.Assert(pack.vn == (float) -3.3589297E38F);
                Debug.Assert(pack.time_week == (ushort)(ushort)20344);
                Debug.Assert(pack.hdop == (float) -5.730693E37F);
                Debug.Assert(pack.vdop == (float)2.9930415E38F);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.vert_accuracy = (float) -1.6311434E38F;
            p232.alt = (float) -5.5730454E37F;
            p232.ve = (float)2.801337E38F;
            p232.vdop = (float)2.9930415E38F;
            p232.horiz_accuracy = (float) -2.3930006E38F;
            p232.vn = (float) -3.3589297E38F;
            p232.lat = (int)769958119;
            p232.gps_id = (byte)(byte)6;
            p232.fix_type = (byte)(byte)18;
            p232.time_usec = (ulong)5818003729088803646L;
            p232.time_week = (ushort)(ushort)20344;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY);
            p232.time_week_ms = (uint)1974595257U;
            p232.speed_accuracy = (float)1.7583336E38F;
            p232.satellites_visible = (byte)(byte)67;
            p232.hdop = (float) -5.730693E37F;
            p232.vd = (float)1.0680512E38F;
            p232.lon = (int) -1034178888;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)169);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)34, (byte)33, (byte)20, (byte)86, (byte)236, (byte)53, (byte)245, (byte)71, (byte)181, (byte)87, (byte)138, (byte)21, (byte)95, (byte)88, (byte)144, (byte)140, (byte)123, (byte)82, (byte)20, (byte)218, (byte)243, (byte)132, (byte)168, (byte)7, (byte)86, (byte)244, (byte)187, (byte)248, (byte)83, (byte)62, (byte)0, (byte)69, (byte)102, (byte)193, (byte)89, (byte)177, (byte)148, (byte)215, (byte)224, (byte)96, (byte)47, (byte)227, (byte)255, (byte)149, (byte)9, (byte)211, (byte)232, (byte)171, (byte)187, (byte)154, (byte)88, (byte)155, (byte)206, (byte)172, (byte)98, (byte)177, (byte)41, (byte)159, (byte)89, (byte)17, (byte)125, (byte)238, (byte)129, (byte)141, (byte)183, (byte)109, (byte)22, (byte)103, (byte)230, (byte)161, (byte)194, (byte)103, (byte)89, (byte)147, (byte)91, (byte)163, (byte)51, (byte)222, (byte)187, (byte)47, (byte)251, (byte)8, (byte)22, (byte)253, (byte)53, (byte)147, (byte)220, (byte)58, (byte)211, (byte)208, (byte)161, (byte)88, (byte)189, (byte)48, (byte)43, (byte)163, (byte)174, (byte)243, (byte)75, (byte)223, (byte)196, (byte)95, (byte)236, (byte)144, (byte)225, (byte)123, (byte)111, (byte)161, (byte)247, (byte)143, (byte)202, (byte)43, (byte)145, (byte)229, (byte)152, (byte)34, (byte)59, (byte)2, (byte)117, (byte)145, (byte)107, (byte)91, (byte)221, (byte)123, (byte)174, (byte)18, (byte)26, (byte)246, (byte)198, (byte)41, (byte)234, (byte)38, (byte)56, (byte)30, (byte)173, (byte)110, (byte)28, (byte)178, (byte)154, (byte)26, (byte)216, (byte)124, (byte)109, (byte)91, (byte)121, (byte)241, (byte)212, (byte)118, (byte)202, (byte)102, (byte)178, (byte)90, (byte)30, (byte)189, (byte)111, (byte)138, (byte)55, (byte)226, (byte)56, (byte)202, (byte)19, (byte)222, (byte)42, (byte)231, (byte)10, (byte)27, (byte)43, (byte)213, (byte)174, (byte)175, (byte)46, (byte)206, (byte)130, (byte)162, (byte)71, (byte)194, (byte)241, (byte)200, (byte)16, (byte)76}));
                Debug.Assert(pack.flags == (byte)(byte)65);
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.len = (byte)(byte)169;
            p233.data__SET(new byte[] {(byte)34, (byte)33, (byte)20, (byte)86, (byte)236, (byte)53, (byte)245, (byte)71, (byte)181, (byte)87, (byte)138, (byte)21, (byte)95, (byte)88, (byte)144, (byte)140, (byte)123, (byte)82, (byte)20, (byte)218, (byte)243, (byte)132, (byte)168, (byte)7, (byte)86, (byte)244, (byte)187, (byte)248, (byte)83, (byte)62, (byte)0, (byte)69, (byte)102, (byte)193, (byte)89, (byte)177, (byte)148, (byte)215, (byte)224, (byte)96, (byte)47, (byte)227, (byte)255, (byte)149, (byte)9, (byte)211, (byte)232, (byte)171, (byte)187, (byte)154, (byte)88, (byte)155, (byte)206, (byte)172, (byte)98, (byte)177, (byte)41, (byte)159, (byte)89, (byte)17, (byte)125, (byte)238, (byte)129, (byte)141, (byte)183, (byte)109, (byte)22, (byte)103, (byte)230, (byte)161, (byte)194, (byte)103, (byte)89, (byte)147, (byte)91, (byte)163, (byte)51, (byte)222, (byte)187, (byte)47, (byte)251, (byte)8, (byte)22, (byte)253, (byte)53, (byte)147, (byte)220, (byte)58, (byte)211, (byte)208, (byte)161, (byte)88, (byte)189, (byte)48, (byte)43, (byte)163, (byte)174, (byte)243, (byte)75, (byte)223, (byte)196, (byte)95, (byte)236, (byte)144, (byte)225, (byte)123, (byte)111, (byte)161, (byte)247, (byte)143, (byte)202, (byte)43, (byte)145, (byte)229, (byte)152, (byte)34, (byte)59, (byte)2, (byte)117, (byte)145, (byte)107, (byte)91, (byte)221, (byte)123, (byte)174, (byte)18, (byte)26, (byte)246, (byte)198, (byte)41, (byte)234, (byte)38, (byte)56, (byte)30, (byte)173, (byte)110, (byte)28, (byte)178, (byte)154, (byte)26, (byte)216, (byte)124, (byte)109, (byte)91, (byte)121, (byte)241, (byte)212, (byte)118, (byte)202, (byte)102, (byte)178, (byte)90, (byte)30, (byte)189, (byte)111, (byte)138, (byte)55, (byte)226, (byte)56, (byte)202, (byte)19, (byte)222, (byte)42, (byte)231, (byte)10, (byte)27, (byte)43, (byte)213, (byte)174, (byte)175, (byte)46, (byte)206, (byte)130, (byte)162, (byte)71, (byte)194, (byte)241, (byte)200, (byte)16, (byte)76}, 0) ;
            p233.flags = (byte)(byte)65;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.battery_remaining == (byte)(byte)18);
                Debug.Assert(pack.throttle == (sbyte)(sbyte) - 1);
                Debug.Assert(pack.altitude_amsl == (short)(short) -17290);
                Debug.Assert(pack.altitude_sp == (short)(short) -28337);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)213);
                Debug.Assert(pack.latitude == (int)816284380);
                Debug.Assert(pack.groundspeed == (byte)(byte)19);
                Debug.Assert(pack.gps_nsat == (byte)(byte)70);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED));
                Debug.Assert(pack.roll == (short)(short)19720);
                Debug.Assert(pack.heading_sp == (short)(short) -23462);
                Debug.Assert(pack.failsafe == (byte)(byte)76);
                Debug.Assert(pack.airspeed == (byte)(byte)41);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 49);
                Debug.Assert(pack.longitude == (int)17865973);
                Debug.Assert(pack.pitch == (short)(short)30164);
                Debug.Assert(pack.custom_mode == (uint)3522208645U);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)7515);
                Debug.Assert(pack.gps_fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
                Debug.Assert(pack.heading == (ushort)(ushort)38451);
                Debug.Assert(pack.wp_num == (byte)(byte)152);
                Debug.Assert(pack.temperature == (sbyte)(sbyte)56);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte) - 31);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.airspeed = (byte)(byte)41;
            p234.battery_remaining = (byte)(byte)18;
            p234.failsafe = (byte)(byte)76;
            p234.climb_rate = (sbyte)(sbyte) - 49;
            p234.longitude = (int)17865973;
            p234.gps_nsat = (byte)(byte)70;
            p234.pitch = (short)(short)30164;
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
            p234.altitude_amsl = (short)(short) -17290;
            p234.heading = (ushort)(ushort)38451;
            p234.heading_sp = (short)(short) -23462;
            p234.temperature = (sbyte)(sbyte)56;
            p234.throttle = (sbyte)(sbyte) - 1;
            p234.custom_mode = (uint)3522208645U;
            p234.wp_distance = (ushort)(ushort)7515;
            p234.roll = (short)(short)19720;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            p234.wp_num = (byte)(byte)152;
            p234.airspeed_sp = (byte)(byte)213;
            p234.altitude_sp = (short)(short) -28337;
            p234.groundspeed = (byte)(byte)19;
            p234.temperature_air = (sbyte)(sbyte) - 31;
            p234.latitude = (int)816284380;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)254967561936036135L);
                Debug.Assert(pack.vibration_z == (float) -2.409399E38F);
                Debug.Assert(pack.clipping_1 == (uint)3928743466U);
                Debug.Assert(pack.clipping_2 == (uint)3458425443U);
                Debug.Assert(pack.vibration_x == (float)1.6775329E38F);
                Debug.Assert(pack.vibration_y == (float)3.20331E37F);
                Debug.Assert(pack.clipping_0 == (uint)2515613901U);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.clipping_1 = (uint)3928743466U;
            p241.clipping_0 = (uint)2515613901U;
            p241.time_usec = (ulong)254967561936036135L;
            p241.vibration_z = (float) -2.409399E38F;
            p241.clipping_2 = (uint)3458425443U;
            p241.vibration_y = (float)3.20331E37F;
            p241.vibration_x = (float)1.6775329E38F;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.approach_y == (float)2.305462E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)4625813807233059178L);
                Debug.Assert(pack.latitude == (int)1043413518);
                Debug.Assert(pack.y == (float)7.9998407E37F);
                Debug.Assert(pack.altitude == (int)675015876);
                Debug.Assert(pack.approach_x == (float)3.3329175E38F);
                Debug.Assert(pack.approach_z == (float)2.7058976E38F);
                Debug.Assert(pack.longitude == (int)1405714558);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.676949E37F, 1.8697457E38F, -2.92175E37F, 3.0676646E38F}));
                Debug.Assert(pack.z == (float)2.2835354E38F);
                Debug.Assert(pack.x == (float)1.8429924E38F);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.time_usec_SET((ulong)4625813807233059178L, PH) ;
            p242.approach_x = (float)3.3329175E38F;
            p242.latitude = (int)1043413518;
            p242.z = (float)2.2835354E38F;
            p242.x = (float)1.8429924E38F;
            p242.y = (float)7.9998407E37F;
            p242.altitude = (int)675015876;
            p242.longitude = (int)1405714558;
            p242.approach_z = (float)2.7058976E38F;
            p242.approach_y = (float)2.305462E38F;
            p242.q_SET(new float[] {-2.676949E37F, 1.8697457E38F, -2.92175E37F, 3.0676646E38F}, 0) ;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)2.2687774E38F);
                Debug.Assert(pack.x == (float) -1.2024653E38F);
                Debug.Assert(pack.latitude == (int)828899822);
                Debug.Assert(pack.approach_x == (float) -3.839113E37F);
                Debug.Assert(pack.approach_y == (float)7.873206E37F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)2055273428140145031L);
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.5258474E37F, -2.1298033E38F, -1.0495809E38F, -2.7276239E38F}));
                Debug.Assert(pack.altitude == (int) -425998334);
                Debug.Assert(pack.longitude == (int) -2130485374);
                Debug.Assert(pack.target_system == (byte)(byte)100);
                Debug.Assert(pack.y == (float)8.1280534E37F);
                Debug.Assert(pack.approach_z == (float)2.4223208E37F);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.time_usec_SET((ulong)2055273428140145031L, PH) ;
            p243.altitude = (int) -425998334;
            p243.target_system = (byte)(byte)100;
            p243.y = (float)8.1280534E37F;
            p243.z = (float)2.2687774E38F;
            p243.approach_y = (float)7.873206E37F;
            p243.approach_x = (float) -3.839113E37F;
            p243.latitude = (int)828899822;
            p243.x = (float) -1.2024653E38F;
            p243.q_SET(new float[] {3.5258474E37F, -2.1298033E38F, -1.0495809E38F, -2.7276239E38F}, 0) ;
            p243.approach_z = (float)2.4223208E37F;
            p243.longitude = (int) -2130485374;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)46793);
                Debug.Assert(pack.interval_us == (int)1848537603);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int)1848537603;
            p244.message_id = (ushort)(ushort)46793;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
                Debug.Assert(pack.vtol_state == MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_MC;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tslc == (byte)(byte)159);
                Debug.Assert(pack.flags == (ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE));
                Debug.Assert(pack.lat == (int) -1927128341);
                Debug.Assert(pack.lon == (int)1107629897);
                Debug.Assert(pack.ver_velocity == (short)(short)13344);
                Debug.Assert(pack.squawk == (ushort)(ushort)54300);
                Debug.Assert(pack.altitude == (int)1397783534);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)46956);
                Debug.Assert(pack.altitude_type == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
                Debug.Assert(pack.emitter_type == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHT);
                Debug.Assert(pack.heading == (ushort)(ushort)38787);
                Debug.Assert(pack.callsign_LEN(ph) == 7);
                Debug.Assert(pack.callsign_TRY(ph).Equals("ridbjit"));
                Debug.Assert(pack.ICAO_address == (uint)4095463728U);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.squawk = (ushort)(ushort)54300;
            p246.lat = (int) -1927128341;
            p246.ver_velocity = (short)(short)13344;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.heading = (ushort)(ushort)38787;
            p246.hor_velocity = (ushort)(ushort)46956;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE);
            p246.callsign_SET("ridbjit", PH) ;
            p246.lon = (int)1107629897;
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHT;
            p246.altitude = (int)1397783534;
            p246.ICAO_address = (uint)4095463728U;
            p246.tslc = (byte)(byte)159;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.threat_level == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
                Debug.Assert(pack.time_to_minimum_delta == (float) -2.874965E38F);
                Debug.Assert(pack.horizontal_minimum_delta == (float)3.3802587E38F);
                Debug.Assert(pack.id == (uint)913695031U);
                Debug.Assert(pack.altitude_minimum_delta == (float)1.5716278E38F);
                Debug.Assert(pack.action == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY);
                Debug.Assert(pack.src_ == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.altitude_minimum_delta = (float)1.5716278E38F;
            p247.id = (uint)913695031U;
            p247.threat_level = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE;
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.horizontal_minimum_delta = (float)3.3802587E38F;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY;
            p247.time_to_minimum_delta = (float) -2.874965E38F;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)191);
                Debug.Assert(pack.target_network == (byte)(byte)247);
                Debug.Assert(pack.message_type == (ushort)(ushort)48293);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)146, (byte)7, (byte)224, (byte)136, (byte)64, (byte)212, (byte)212, (byte)239, (byte)82, (byte)3, (byte)163, (byte)8, (byte)215, (byte)126, (byte)213, (byte)26, (byte)122, (byte)42, (byte)173, (byte)67, (byte)186, (byte)120, (byte)99, (byte)156, (byte)147, (byte)89, (byte)93, (byte)199, (byte)187, (byte)142, (byte)183, (byte)141, (byte)45, (byte)245, (byte)190, (byte)33, (byte)157, (byte)27, (byte)166, (byte)167, (byte)146, (byte)192, (byte)66, (byte)56, (byte)151, (byte)88, (byte)240, (byte)218, (byte)34, (byte)184, (byte)49, (byte)17, (byte)179, (byte)187, (byte)50, (byte)163, (byte)202, (byte)56, (byte)140, (byte)190, (byte)177, (byte)73, (byte)23, (byte)192, (byte)48, (byte)126, (byte)82, (byte)240, (byte)145, (byte)242, (byte)198, (byte)217, (byte)180, (byte)6, (byte)38, (byte)227, (byte)220, (byte)189, (byte)241, (byte)6, (byte)182, (byte)217, (byte)225, (byte)50, (byte)121, (byte)201, (byte)231, (byte)164, (byte)45, (byte)58, (byte)102, (byte)98, (byte)44, (byte)123, (byte)45, (byte)101, (byte)111, (byte)62, (byte)217, (byte)179, (byte)77, (byte)224, (byte)171, (byte)145, (byte)44, (byte)4, (byte)2, (byte)1, (byte)109, (byte)78, (byte)26, (byte)16, (byte)171, (byte)175, (byte)28, (byte)10, (byte)79, (byte)66, (byte)94, (byte)7, (byte)126, (byte)48, (byte)235, (byte)157, (byte)134, (byte)152, (byte)176, (byte)2, (byte)73, (byte)95, (byte)12, (byte)18, (byte)220, (byte)238, (byte)237, (byte)175, (byte)9, (byte)99, (byte)30, (byte)250, (byte)63, (byte)95, (byte)100, (byte)50, (byte)185, (byte)130, (byte)3, (byte)170, (byte)92, (byte)58, (byte)251, (byte)136, (byte)99, (byte)3, (byte)136, (byte)22, (byte)255, (byte)41, (byte)40, (byte)23, (byte)245, (byte)164, (byte)112, (byte)97, (byte)89, (byte)113, (byte)188, (byte)231, (byte)138, (byte)141, (byte)21, (byte)249, (byte)62, (byte)65, (byte)169, (byte)27, (byte)254, (byte)254, (byte)207, (byte)20, (byte)6, (byte)58, (byte)160, (byte)215, (byte)219, (byte)9, (byte)55, (byte)140, (byte)251, (byte)57, (byte)169, (byte)12, (byte)102, (byte)89, (byte)56, (byte)219, (byte)218, (byte)121, (byte)81, (byte)67, (byte)0, (byte)24, (byte)214, (byte)21, (byte)191, (byte)136, (byte)54, (byte)250, (byte)113, (byte)37, (byte)176, (byte)172, (byte)199, (byte)129, (byte)247, (byte)101, (byte)133, (byte)161, (byte)216, (byte)54, (byte)87, (byte)196, (byte)192, (byte)21, (byte)209, (byte)83, (byte)160, (byte)56, (byte)211, (byte)104, (byte)110, (byte)41, (byte)246, (byte)128, (byte)161, (byte)18, (byte)42, (byte)127, (byte)129, (byte)25, (byte)12, (byte)17, (byte)145, (byte)217, (byte)81, (byte)110, (byte)235, (byte)116, (byte)104}));
                Debug.Assert(pack.target_component == (byte)(byte)184);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.payload_SET(new byte[] {(byte)146, (byte)7, (byte)224, (byte)136, (byte)64, (byte)212, (byte)212, (byte)239, (byte)82, (byte)3, (byte)163, (byte)8, (byte)215, (byte)126, (byte)213, (byte)26, (byte)122, (byte)42, (byte)173, (byte)67, (byte)186, (byte)120, (byte)99, (byte)156, (byte)147, (byte)89, (byte)93, (byte)199, (byte)187, (byte)142, (byte)183, (byte)141, (byte)45, (byte)245, (byte)190, (byte)33, (byte)157, (byte)27, (byte)166, (byte)167, (byte)146, (byte)192, (byte)66, (byte)56, (byte)151, (byte)88, (byte)240, (byte)218, (byte)34, (byte)184, (byte)49, (byte)17, (byte)179, (byte)187, (byte)50, (byte)163, (byte)202, (byte)56, (byte)140, (byte)190, (byte)177, (byte)73, (byte)23, (byte)192, (byte)48, (byte)126, (byte)82, (byte)240, (byte)145, (byte)242, (byte)198, (byte)217, (byte)180, (byte)6, (byte)38, (byte)227, (byte)220, (byte)189, (byte)241, (byte)6, (byte)182, (byte)217, (byte)225, (byte)50, (byte)121, (byte)201, (byte)231, (byte)164, (byte)45, (byte)58, (byte)102, (byte)98, (byte)44, (byte)123, (byte)45, (byte)101, (byte)111, (byte)62, (byte)217, (byte)179, (byte)77, (byte)224, (byte)171, (byte)145, (byte)44, (byte)4, (byte)2, (byte)1, (byte)109, (byte)78, (byte)26, (byte)16, (byte)171, (byte)175, (byte)28, (byte)10, (byte)79, (byte)66, (byte)94, (byte)7, (byte)126, (byte)48, (byte)235, (byte)157, (byte)134, (byte)152, (byte)176, (byte)2, (byte)73, (byte)95, (byte)12, (byte)18, (byte)220, (byte)238, (byte)237, (byte)175, (byte)9, (byte)99, (byte)30, (byte)250, (byte)63, (byte)95, (byte)100, (byte)50, (byte)185, (byte)130, (byte)3, (byte)170, (byte)92, (byte)58, (byte)251, (byte)136, (byte)99, (byte)3, (byte)136, (byte)22, (byte)255, (byte)41, (byte)40, (byte)23, (byte)245, (byte)164, (byte)112, (byte)97, (byte)89, (byte)113, (byte)188, (byte)231, (byte)138, (byte)141, (byte)21, (byte)249, (byte)62, (byte)65, (byte)169, (byte)27, (byte)254, (byte)254, (byte)207, (byte)20, (byte)6, (byte)58, (byte)160, (byte)215, (byte)219, (byte)9, (byte)55, (byte)140, (byte)251, (byte)57, (byte)169, (byte)12, (byte)102, (byte)89, (byte)56, (byte)219, (byte)218, (byte)121, (byte)81, (byte)67, (byte)0, (byte)24, (byte)214, (byte)21, (byte)191, (byte)136, (byte)54, (byte)250, (byte)113, (byte)37, (byte)176, (byte)172, (byte)199, (byte)129, (byte)247, (byte)101, (byte)133, (byte)161, (byte)216, (byte)54, (byte)87, (byte)196, (byte)192, (byte)21, (byte)209, (byte)83, (byte)160, (byte)56, (byte)211, (byte)104, (byte)110, (byte)41, (byte)246, (byte)128, (byte)161, (byte)18, (byte)42, (byte)127, (byte)129, (byte)25, (byte)12, (byte)17, (byte)145, (byte)217, (byte)81, (byte)110, (byte)235, (byte)116, (byte)104}, 0) ;
            p248.target_component = (byte)(byte)184;
            p248.target_system = (byte)(byte)191;
            p248.message_type = (ushort)(ushort)48293;
            p248.target_network = (byte)(byte)247;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.address == (ushort)(ushort)37904);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte) - 42, (sbyte) - 17, (sbyte)54, (sbyte)85, (sbyte)4, (sbyte) - 109, (sbyte)120, (sbyte)20, (sbyte)70, (sbyte)112, (sbyte)122, (sbyte)17, (sbyte)69, (sbyte) - 53, (sbyte) - 56, (sbyte)51, (sbyte)61, (sbyte)80, (sbyte) - 122, (sbyte) - 64, (sbyte)1, (sbyte)14, (sbyte)117, (sbyte)44, (sbyte)100, (sbyte)36, (sbyte) - 63, (sbyte)101, (sbyte) - 101, (sbyte) - 123, (sbyte) - 32, (sbyte) - 88}));
                Debug.Assert(pack.ver == (byte)(byte)59);
                Debug.Assert(pack.type == (byte)(byte)225);
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.ver = (byte)(byte)59;
            p249.value_SET(new sbyte[] {(sbyte) - 42, (sbyte) - 17, (sbyte)54, (sbyte)85, (sbyte)4, (sbyte) - 109, (sbyte)120, (sbyte)20, (sbyte)70, (sbyte)112, (sbyte)122, (sbyte)17, (sbyte)69, (sbyte) - 53, (sbyte) - 56, (sbyte)51, (sbyte)61, (sbyte)80, (sbyte) - 122, (sbyte) - 64, (sbyte)1, (sbyte)14, (sbyte)117, (sbyte)44, (sbyte)100, (sbyte)36, (sbyte) - 63, (sbyte)101, (sbyte) - 101, (sbyte) - 123, (sbyte) - 32, (sbyte) - 88}, 0) ;
            p249.type = (byte)(byte)225;
            p249.address = (ushort)(ushort)37904;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)1.6772307E38F);
                Debug.Assert(pack.time_usec == (ulong)567992197361132995L);
                Debug.Assert(pack.name_LEN(ph) == 2);
                Debug.Assert(pack.name_TRY(ph).Equals("ud"));
                Debug.Assert(pack.x == (float) -2.2030242E38F);
                Debug.Assert(pack.z == (float)2.274517E38F);
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.name_SET("ud", PH) ;
            p250.x = (float) -2.2030242E38F;
            p250.time_usec = (ulong)567992197361132995L;
            p250.y = (float)1.6772307E38F;
            p250.z = (float)2.274517E38F;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 8);
                Debug.Assert(pack.name_TRY(ph).Equals("pxiqmzEp"));
                Debug.Assert(pack.time_boot_ms == (uint)2049423699U);
                Debug.Assert(pack.value == (float) -4.334234E37F);
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.name_SET("pxiqmzEp", PH) ;
            p251.value = (float) -4.334234E37F;
            p251.time_boot_ms = (uint)2049423699U;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 1);
                Debug.Assert(pack.name_TRY(ph).Equals("f"));
                Debug.Assert(pack.time_boot_ms == (uint)1666212654U);
                Debug.Assert(pack.value == (int)728972601);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.name_SET("f", PH) ;
            p252.time_boot_ms = (uint)1666212654U;
            p252.value = (int)728972601;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 4);
                Debug.Assert(pack.text_TRY(ph).Equals("Tvqh"));
                Debug.Assert(pack.severity == MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_EMERGENCY;
            p253.text_SET("Tvqh", PH) ;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1618935001U);
                Debug.Assert(pack.value == (float)3.1210204E37F);
                Debug.Assert(pack.ind == (byte)(byte)210);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.ind = (byte)(byte)210;
            p254.value = (float)3.1210204E37F;
            p254.time_boot_ms = (uint)1618935001U;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)9, (byte)252, (byte)169, (byte)252, (byte)79, (byte)115, (byte)26, (byte)142, (byte)111, (byte)185, (byte)251, (byte)220, (byte)99, (byte)209, (byte)45, (byte)153, (byte)176, (byte)223, (byte)118, (byte)44, (byte)54, (byte)95, (byte)239, (byte)161, (byte)112, (byte)115, (byte)182, (byte)7, (byte)90, (byte)133, (byte)60, (byte)227}));
                Debug.Assert(pack.initial_timestamp == (ulong)2382039211384579579L);
                Debug.Assert(pack.target_system == (byte)(byte)21);
                Debug.Assert(pack.target_component == (byte)(byte)130);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_component = (byte)(byte)130;
            p256.target_system = (byte)(byte)21;
            p256.secret_key_SET(new byte[] {(byte)9, (byte)252, (byte)169, (byte)252, (byte)79, (byte)115, (byte)26, (byte)142, (byte)111, (byte)185, (byte)251, (byte)220, (byte)99, (byte)209, (byte)45, (byte)153, (byte)176, (byte)223, (byte)118, (byte)44, (byte)54, (byte)95, (byte)239, (byte)161, (byte)112, (byte)115, (byte)182, (byte)7, (byte)90, (byte)133, (byte)60, (byte)227}, 0) ;
            p256.initial_timestamp = (ulong)2382039211384579579L;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.state == (byte)(byte)213);
                Debug.Assert(pack.last_change_ms == (uint)53480722U);
                Debug.Assert(pack.time_boot_ms == (uint)1056166059U);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.state = (byte)(byte)213;
            p257.time_boot_ms = (uint)1056166059U;
            p257.last_change_ms = (uint)53480722U;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)153);
                Debug.Assert(pack.target_component == (byte)(byte)53);
                Debug.Assert(pack.tune_LEN(ph) == 3);
                Debug.Assert(pack.tune_TRY(ph).Equals("vsi"));
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_component = (byte)(byte)53;
            p258.tune_SET("vsi", PH) ;
            p258.target_system = (byte)(byte)153;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)244, (byte)248, (byte)14, (byte)33, (byte)88, (byte)245, (byte)169, (byte)160, (byte)250, (byte)12, (byte)208, (byte)131, (byte)186, (byte)8, (byte)73, (byte)51, (byte)191, (byte)166, (byte)77, (byte)220, (byte)17, (byte)158, (byte)160, (byte)90, (byte)230, (byte)206, (byte)234, (byte)48, (byte)121, (byte)231, (byte)227, (byte)142}));
                Debug.Assert(pack.sensor_size_v == (float) -1.3695435E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3452602725U);
                Debug.Assert(pack.sensor_size_h == (float) -3.3849644E38F);
                Debug.Assert(pack.firmware_version == (uint)1213188021U);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)36965);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE));
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 77);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("wayqpgxyhxhtfegfnwcUfoudhyNakwbtkiXdiziviuJvgsnaxavwyweuyugikhnjdvBnGsyLgxXgp"));
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)147, (byte)68, (byte)174, (byte)97, (byte)235, (byte)39, (byte)192, (byte)133, (byte)113, (byte)214, (byte)48, (byte)169, (byte)183, (byte)20, (byte)114, (byte)201, (byte)51, (byte)251, (byte)110, (byte)62, (byte)216, (byte)193, (byte)156, (byte)176, (byte)183, (byte)4, (byte)42, (byte)118, (byte)151, (byte)90, (byte)168, (byte)101}));
                Debug.Assert(pack.focal_length == (float)2.7426976E38F);
                Debug.Assert(pack.lens_id == (byte)(byte)202);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)42846);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)24134);
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
            p259.lens_id = (byte)(byte)202;
            p259.model_name_SET(new byte[] {(byte)244, (byte)248, (byte)14, (byte)33, (byte)88, (byte)245, (byte)169, (byte)160, (byte)250, (byte)12, (byte)208, (byte)131, (byte)186, (byte)8, (byte)73, (byte)51, (byte)191, (byte)166, (byte)77, (byte)220, (byte)17, (byte)158, (byte)160, (byte)90, (byte)230, (byte)206, (byte)234, (byte)48, (byte)121, (byte)231, (byte)227, (byte)142}, 0) ;
            p259.sensor_size_v = (float) -1.3695435E38F;
            p259.sensor_size_h = (float) -3.3849644E38F;
            p259.cam_definition_uri_SET("wayqpgxyhxhtfegfnwcUfoudhyNakwbtkiXdiziviuJvgsnaxavwyweuyugikhnjdvBnGsyLgxXgp", PH) ;
            p259.resolution_v = (ushort)(ushort)36965;
            p259.resolution_h = (ushort)(ushort)42846;
            p259.cam_definition_version = (ushort)(ushort)24134;
            p259.vendor_name_SET(new byte[] {(byte)147, (byte)68, (byte)174, (byte)97, (byte)235, (byte)39, (byte)192, (byte)133, (byte)113, (byte)214, (byte)48, (byte)169, (byte)183, (byte)20, (byte)114, (byte)201, (byte)51, (byte)251, (byte)110, (byte)62, (byte)216, (byte)193, (byte)156, (byte)176, (byte)183, (byte)4, (byte)42, (byte)118, (byte)151, (byte)90, (byte)168, (byte)101}, 0) ;
            p259.time_boot_ms = (uint)3452602725U;
            p259.firmware_version = (uint)1213188021U;
            p259.focal_length = (float)2.7426976E38F;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == CAMERA_MODE.CAMERA_MODE_VIDEO);
                Debug.Assert(pack.time_boot_ms == (uint)3513389185U);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)3513389185U;
            p260.mode_id = CAMERA_MODE.CAMERA_MODE_VIDEO;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.total_capacity == (float) -2.0582865E38F);
                Debug.Assert(pack.used_capacity == (float) -1.8000628E38F);
                Debug.Assert(pack.write_speed == (float)5.1564355E37F);
                Debug.Assert(pack.status == (byte)(byte)125);
                Debug.Assert(pack.time_boot_ms == (uint)3796279521U);
                Debug.Assert(pack.storage_count == (byte)(byte)189);
                Debug.Assert(pack.storage_id == (byte)(byte)239);
                Debug.Assert(pack.available_capacity == (float) -1.0387847E38F);
                Debug.Assert(pack.read_speed == (float) -1.3531858E38F);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.total_capacity = (float) -2.0582865E38F;
            p261.write_speed = (float)5.1564355E37F;
            p261.available_capacity = (float) -1.0387847E38F;
            p261.used_capacity = (float) -1.8000628E38F;
            p261.storage_count = (byte)(byte)189;
            p261.status = (byte)(byte)125;
            p261.read_speed = (float) -1.3531858E38F;
            p261.time_boot_ms = (uint)3796279521U;
            p261.storage_id = (byte)(byte)239;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.recording_time_ms == (uint)3773312309U);
                Debug.Assert(pack.image_status == (byte)(byte)36);
                Debug.Assert(pack.available_capacity == (float)1.0890248E38F);
                Debug.Assert(pack.image_interval == (float)8.0437243E37F);
                Debug.Assert(pack.video_status == (byte)(byte)77);
                Debug.Assert(pack.time_boot_ms == (uint)3548604093U);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.image_interval = (float)8.0437243E37F;
            p262.time_boot_ms = (uint)3548604093U;
            p262.available_capacity = (float)1.0890248E38F;
            p262.video_status = (byte)(byte)77;
            p262.image_status = (byte)(byte)36;
            p262.recording_time_ms = (uint)3773312309U;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_utc == (ulong)701620836289831655L);
                Debug.Assert(pack.relative_alt == (int)1067645259);
                Debug.Assert(pack.file_url_LEN(ph) == 22);
                Debug.Assert(pack.file_url_TRY(ph).Equals("xydphaxcmjdmopEwtuurnn"));
                Debug.Assert(pack.q.SequenceEqual(new float[] {5.964842E37F, 4.7648735E37F, 1.2683243E38F, -1.1187215E38F}));
                Debug.Assert(pack.camera_id == (byte)(byte)33);
                Debug.Assert(pack.lat == (int) -786853229);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte) - 48);
                Debug.Assert(pack.lon == (int)1089337698);
                Debug.Assert(pack.time_boot_ms == (uint)3380606711U);
                Debug.Assert(pack.alt == (int)1104343029);
                Debug.Assert(pack.image_index == (int) -154507840);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.capture_result = (sbyte)(sbyte) - 48;
            p263.file_url_SET("xydphaxcmjdmopEwtuurnn", PH) ;
            p263.camera_id = (byte)(byte)33;
            p263.time_boot_ms = (uint)3380606711U;
            p263.relative_alt = (int)1067645259;
            p263.lat = (int) -786853229;
            p263.alt = (int)1104343029;
            p263.q_SET(new float[] {5.964842E37F, 4.7648735E37F, 1.2683243E38F, -1.1187215E38F}, 0) ;
            p263.lon = (int)1089337698;
            p263.time_utc = (ulong)701620836289831655L;
            p263.image_index = (int) -154507840;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.takeoff_time_utc == (ulong)1727216300141930461L);
                Debug.Assert(pack.time_boot_ms == (uint)168851276U);
                Debug.Assert(pack.flight_uuid == (ulong)2170582482597930932L);
                Debug.Assert(pack.arming_time_utc == (ulong)5122437785612411935L);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.flight_uuid = (ulong)2170582482597930932L;
            p264.time_boot_ms = (uint)168851276U;
            p264.arming_time_utc = (ulong)5122437785612411935L;
            p264.takeoff_time_utc = (ulong)1727216300141930461L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -2.8290004E38F);
                Debug.Assert(pack.pitch == (float) -1.2183974E38F);
                Debug.Assert(pack.roll == (float)2.9440118E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3726542602U);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.pitch = (float) -1.2183974E38F;
            p265.yaw = (float) -2.8290004E38F;
            p265.time_boot_ms = (uint)3726542602U;
            p265.roll = (float)2.9440118E37F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)165);
                Debug.Assert(pack.target_system == (byte)(byte)116);
                Debug.Assert(pack.first_message_offset == (byte)(byte)88);
                Debug.Assert(pack.length == (byte)(byte)38);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)224, (byte)214, (byte)81, (byte)98, (byte)177, (byte)221, (byte)161, (byte)165, (byte)97, (byte)147, (byte)84, (byte)198, (byte)242, (byte)122, (byte)30, (byte)170, (byte)242, (byte)238, (byte)174, (byte)230, (byte)234, (byte)212, (byte)173, (byte)194, (byte)134, (byte)110, (byte)167, (byte)79, (byte)229, (byte)17, (byte)160, (byte)8, (byte)251, (byte)179, (byte)249, (byte)198, (byte)84, (byte)166, (byte)79, (byte)6, (byte)107, (byte)132, (byte)70, (byte)76, (byte)96, (byte)64, (byte)163, (byte)138, (byte)53, (byte)169, (byte)143, (byte)239, (byte)101, (byte)144, (byte)151, (byte)178, (byte)42, (byte)165, (byte)103, (byte)135, (byte)157, (byte)46, (byte)18, (byte)198, (byte)246, (byte)59, (byte)8, (byte)121, (byte)113, (byte)137, (byte)110, (byte)249, (byte)231, (byte)214, (byte)218, (byte)100, (byte)162, (byte)103, (byte)32, (byte)182, (byte)64, (byte)105, (byte)104, (byte)163, (byte)225, (byte)198, (byte)199, (byte)14, (byte)100, (byte)116, (byte)241, (byte)130, (byte)158, (byte)23, (byte)108, (byte)203, (byte)133, (byte)153, (byte)49, (byte)17, (byte)141, (byte)234, (byte)223, (byte)37, (byte)77, (byte)236, (byte)202, (byte)161, (byte)221, (byte)254, (byte)176, (byte)34, (byte)167, (byte)254, (byte)231, (byte)121, (byte)111, (byte)217, (byte)207, (byte)76, (byte)239, (byte)244, (byte)40, (byte)136, (byte)13, (byte)26, (byte)196, (byte)233, (byte)237, (byte)24, (byte)136, (byte)125, (byte)101, (byte)243, (byte)243, (byte)62, (byte)63, (byte)13, (byte)227, (byte)207, (byte)214, (byte)233, (byte)49, (byte)14, (byte)92, (byte)96, (byte)131, (byte)146, (byte)7, (byte)101, (byte)197, (byte)8, (byte)135, (byte)123, (byte)156, (byte)69, (byte)243, (byte)128, (byte)5, (byte)224, (byte)245, (byte)216, (byte)222, (byte)8, (byte)139, (byte)211, (byte)56, (byte)176, (byte)1, (byte)167, (byte)101, (byte)12, (byte)209, (byte)17, (byte)157, (byte)186, (byte)238, (byte)170, (byte)183, (byte)245, (byte)12, (byte)228, (byte)171, (byte)132, (byte)25, (byte)247, (byte)69, (byte)21, (byte)68, (byte)204, (byte)54, (byte)48, (byte)52, (byte)97, (byte)10, (byte)94, (byte)167, (byte)205, (byte)198, (byte)77, (byte)113, (byte)131, (byte)170, (byte)227, (byte)180, (byte)162, (byte)111, (byte)176, (byte)16, (byte)30, (byte)189, (byte)43, (byte)244, (byte)62, (byte)17, (byte)237, (byte)241, (byte)68, (byte)35, (byte)213, (byte)115, (byte)48, (byte)159, (byte)226, (byte)108, (byte)50, (byte)79, (byte)221, (byte)10, (byte)52, (byte)78, (byte)149, (byte)11, (byte)238, (byte)211, (byte)105, (byte)132, (byte)172, (byte)107, (byte)57, (byte)38, (byte)142, (byte)254, (byte)134, (byte)161, (byte)135, (byte)253, (byte)163, (byte)82}));
                Debug.Assert(pack.sequence == (ushort)(ushort)37412);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_system = (byte)(byte)116;
            p266.first_message_offset = (byte)(byte)88;
            p266.data__SET(new byte[] {(byte)224, (byte)214, (byte)81, (byte)98, (byte)177, (byte)221, (byte)161, (byte)165, (byte)97, (byte)147, (byte)84, (byte)198, (byte)242, (byte)122, (byte)30, (byte)170, (byte)242, (byte)238, (byte)174, (byte)230, (byte)234, (byte)212, (byte)173, (byte)194, (byte)134, (byte)110, (byte)167, (byte)79, (byte)229, (byte)17, (byte)160, (byte)8, (byte)251, (byte)179, (byte)249, (byte)198, (byte)84, (byte)166, (byte)79, (byte)6, (byte)107, (byte)132, (byte)70, (byte)76, (byte)96, (byte)64, (byte)163, (byte)138, (byte)53, (byte)169, (byte)143, (byte)239, (byte)101, (byte)144, (byte)151, (byte)178, (byte)42, (byte)165, (byte)103, (byte)135, (byte)157, (byte)46, (byte)18, (byte)198, (byte)246, (byte)59, (byte)8, (byte)121, (byte)113, (byte)137, (byte)110, (byte)249, (byte)231, (byte)214, (byte)218, (byte)100, (byte)162, (byte)103, (byte)32, (byte)182, (byte)64, (byte)105, (byte)104, (byte)163, (byte)225, (byte)198, (byte)199, (byte)14, (byte)100, (byte)116, (byte)241, (byte)130, (byte)158, (byte)23, (byte)108, (byte)203, (byte)133, (byte)153, (byte)49, (byte)17, (byte)141, (byte)234, (byte)223, (byte)37, (byte)77, (byte)236, (byte)202, (byte)161, (byte)221, (byte)254, (byte)176, (byte)34, (byte)167, (byte)254, (byte)231, (byte)121, (byte)111, (byte)217, (byte)207, (byte)76, (byte)239, (byte)244, (byte)40, (byte)136, (byte)13, (byte)26, (byte)196, (byte)233, (byte)237, (byte)24, (byte)136, (byte)125, (byte)101, (byte)243, (byte)243, (byte)62, (byte)63, (byte)13, (byte)227, (byte)207, (byte)214, (byte)233, (byte)49, (byte)14, (byte)92, (byte)96, (byte)131, (byte)146, (byte)7, (byte)101, (byte)197, (byte)8, (byte)135, (byte)123, (byte)156, (byte)69, (byte)243, (byte)128, (byte)5, (byte)224, (byte)245, (byte)216, (byte)222, (byte)8, (byte)139, (byte)211, (byte)56, (byte)176, (byte)1, (byte)167, (byte)101, (byte)12, (byte)209, (byte)17, (byte)157, (byte)186, (byte)238, (byte)170, (byte)183, (byte)245, (byte)12, (byte)228, (byte)171, (byte)132, (byte)25, (byte)247, (byte)69, (byte)21, (byte)68, (byte)204, (byte)54, (byte)48, (byte)52, (byte)97, (byte)10, (byte)94, (byte)167, (byte)205, (byte)198, (byte)77, (byte)113, (byte)131, (byte)170, (byte)227, (byte)180, (byte)162, (byte)111, (byte)176, (byte)16, (byte)30, (byte)189, (byte)43, (byte)244, (byte)62, (byte)17, (byte)237, (byte)241, (byte)68, (byte)35, (byte)213, (byte)115, (byte)48, (byte)159, (byte)226, (byte)108, (byte)50, (byte)79, (byte)221, (byte)10, (byte)52, (byte)78, (byte)149, (byte)11, (byte)238, (byte)211, (byte)105, (byte)132, (byte)172, (byte)107, (byte)57, (byte)38, (byte)142, (byte)254, (byte)134, (byte)161, (byte)135, (byte)253, (byte)163, (byte)82}, 0) ;
            p266.target_component = (byte)(byte)165;
            p266.sequence = (ushort)(ushort)37412;
            p266.length = (byte)(byte)38;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.first_message_offset == (byte)(byte)157);
                Debug.Assert(pack.length == (byte)(byte)176);
                Debug.Assert(pack.target_system == (byte)(byte)13);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)166, (byte)218, (byte)198, (byte)23, (byte)102, (byte)235, (byte)19, (byte)242, (byte)252, (byte)255, (byte)29, (byte)207, (byte)198, (byte)22, (byte)72, (byte)230, (byte)178, (byte)22, (byte)208, (byte)122, (byte)249, (byte)141, (byte)73, (byte)81, (byte)113, (byte)22, (byte)231, (byte)157, (byte)172, (byte)108, (byte)82, (byte)220, (byte)119, (byte)174, (byte)176, (byte)17, (byte)184, (byte)57, (byte)209, (byte)247, (byte)24, (byte)213, (byte)219, (byte)194, (byte)2, (byte)76, (byte)95, (byte)232, (byte)122, (byte)253, (byte)151, (byte)52, (byte)141, (byte)115, (byte)1, (byte)105, (byte)240, (byte)123, (byte)83, (byte)90, (byte)190, (byte)152, (byte)98, (byte)89, (byte)161, (byte)194, (byte)63, (byte)35, (byte)202, (byte)254, (byte)91, (byte)163, (byte)98, (byte)231, (byte)70, (byte)174, (byte)46, (byte)98, (byte)255, (byte)229, (byte)43, (byte)216, (byte)160, (byte)204, (byte)128, (byte)101, (byte)75, (byte)66, (byte)103, (byte)248, (byte)165, (byte)4, (byte)95, (byte)145, (byte)184, (byte)154, (byte)73, (byte)36, (byte)116, (byte)201, (byte)47, (byte)53, (byte)164, (byte)251, (byte)77, (byte)236, (byte)126, (byte)246, (byte)131, (byte)235, (byte)143, (byte)27, (byte)215, (byte)250, (byte)143, (byte)221, (byte)231, (byte)10, (byte)53, (byte)17, (byte)223, (byte)192, (byte)149, (byte)89, (byte)150, (byte)221, (byte)199, (byte)21, (byte)39, (byte)29, (byte)160, (byte)205, (byte)169, (byte)143, (byte)113, (byte)153, (byte)105, (byte)55, (byte)30, (byte)159, (byte)128, (byte)241, (byte)24, (byte)204, (byte)234, (byte)11, (byte)159, (byte)153, (byte)29, (byte)3, (byte)134, (byte)193, (byte)252, (byte)89, (byte)176, (byte)138, (byte)170, (byte)87, (byte)216, (byte)160, (byte)120, (byte)157, (byte)71, (byte)39, (byte)161, (byte)223, (byte)190, (byte)236, (byte)42, (byte)250, (byte)57, (byte)22, (byte)99, (byte)142, (byte)190, (byte)215, (byte)136, (byte)55, (byte)75, (byte)151, (byte)85, (byte)75, (byte)78, (byte)68, (byte)94, (byte)114, (byte)97, (byte)156, (byte)143, (byte)217, (byte)148, (byte)232, (byte)157, (byte)188, (byte)202, (byte)49, (byte)178, (byte)87, (byte)99, (byte)200, (byte)9, (byte)188, (byte)226, (byte)254, (byte)101, (byte)159, (byte)42, (byte)201, (byte)40, (byte)7, (byte)32, (byte)81, (byte)174, (byte)95, (byte)252, (byte)184, (byte)125, (byte)193, (byte)55, (byte)194, (byte)139, (byte)71, (byte)89, (byte)191, (byte)236, (byte)39, (byte)37, (byte)84, (byte)181, (byte)115, (byte)191, (byte)70, (byte)154, (byte)49, (byte)133, (byte)21, (byte)43, (byte)29, (byte)202, (byte)226, (byte)98, (byte)178, (byte)239, (byte)146, (byte)182, (byte)252, (byte)14, (byte)86, (byte)126}));
                Debug.Assert(pack.sequence == (ushort)(ushort)57877);
                Debug.Assert(pack.target_component == (byte)(byte)188);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.sequence = (ushort)(ushort)57877;
            p267.target_component = (byte)(byte)188;
            p267.data__SET(new byte[] {(byte)166, (byte)218, (byte)198, (byte)23, (byte)102, (byte)235, (byte)19, (byte)242, (byte)252, (byte)255, (byte)29, (byte)207, (byte)198, (byte)22, (byte)72, (byte)230, (byte)178, (byte)22, (byte)208, (byte)122, (byte)249, (byte)141, (byte)73, (byte)81, (byte)113, (byte)22, (byte)231, (byte)157, (byte)172, (byte)108, (byte)82, (byte)220, (byte)119, (byte)174, (byte)176, (byte)17, (byte)184, (byte)57, (byte)209, (byte)247, (byte)24, (byte)213, (byte)219, (byte)194, (byte)2, (byte)76, (byte)95, (byte)232, (byte)122, (byte)253, (byte)151, (byte)52, (byte)141, (byte)115, (byte)1, (byte)105, (byte)240, (byte)123, (byte)83, (byte)90, (byte)190, (byte)152, (byte)98, (byte)89, (byte)161, (byte)194, (byte)63, (byte)35, (byte)202, (byte)254, (byte)91, (byte)163, (byte)98, (byte)231, (byte)70, (byte)174, (byte)46, (byte)98, (byte)255, (byte)229, (byte)43, (byte)216, (byte)160, (byte)204, (byte)128, (byte)101, (byte)75, (byte)66, (byte)103, (byte)248, (byte)165, (byte)4, (byte)95, (byte)145, (byte)184, (byte)154, (byte)73, (byte)36, (byte)116, (byte)201, (byte)47, (byte)53, (byte)164, (byte)251, (byte)77, (byte)236, (byte)126, (byte)246, (byte)131, (byte)235, (byte)143, (byte)27, (byte)215, (byte)250, (byte)143, (byte)221, (byte)231, (byte)10, (byte)53, (byte)17, (byte)223, (byte)192, (byte)149, (byte)89, (byte)150, (byte)221, (byte)199, (byte)21, (byte)39, (byte)29, (byte)160, (byte)205, (byte)169, (byte)143, (byte)113, (byte)153, (byte)105, (byte)55, (byte)30, (byte)159, (byte)128, (byte)241, (byte)24, (byte)204, (byte)234, (byte)11, (byte)159, (byte)153, (byte)29, (byte)3, (byte)134, (byte)193, (byte)252, (byte)89, (byte)176, (byte)138, (byte)170, (byte)87, (byte)216, (byte)160, (byte)120, (byte)157, (byte)71, (byte)39, (byte)161, (byte)223, (byte)190, (byte)236, (byte)42, (byte)250, (byte)57, (byte)22, (byte)99, (byte)142, (byte)190, (byte)215, (byte)136, (byte)55, (byte)75, (byte)151, (byte)85, (byte)75, (byte)78, (byte)68, (byte)94, (byte)114, (byte)97, (byte)156, (byte)143, (byte)217, (byte)148, (byte)232, (byte)157, (byte)188, (byte)202, (byte)49, (byte)178, (byte)87, (byte)99, (byte)200, (byte)9, (byte)188, (byte)226, (byte)254, (byte)101, (byte)159, (byte)42, (byte)201, (byte)40, (byte)7, (byte)32, (byte)81, (byte)174, (byte)95, (byte)252, (byte)184, (byte)125, (byte)193, (byte)55, (byte)194, (byte)139, (byte)71, (byte)89, (byte)191, (byte)236, (byte)39, (byte)37, (byte)84, (byte)181, (byte)115, (byte)191, (byte)70, (byte)154, (byte)49, (byte)133, (byte)21, (byte)43, (byte)29, (byte)202, (byte)226, (byte)98, (byte)178, (byte)239, (byte)146, (byte)182, (byte)252, (byte)14, (byte)86, (byte)126}, 0) ;
            p267.first_message_offset = (byte)(byte)157;
            p267.length = (byte)(byte)176;
            p267.target_system = (byte)(byte)13;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)237);
                Debug.Assert(pack.sequence == (ushort)(ushort)63810);
                Debug.Assert(pack.target_system == (byte)(byte)180);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_component = (byte)(byte)237;
            p268.target_system = (byte)(byte)180;
            p268.sequence = (ushort)(ushort)63810;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.status == (byte)(byte)253);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)65390);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)15505);
                Debug.Assert(pack.bitrate == (uint)2049334668U);
                Debug.Assert(pack.framerate == (float)1.8848731E38F);
                Debug.Assert(pack.uri_LEN(ph) == 182);
                Debug.Assert(pack.uri_TRY(ph).Equals("ufqukllhwjyedgrnicpfhdzzbhshhdQeXelfPDEmfwrHywhxhvskzOwygdjioiuhfnWtysUKukenruyejeZwkxonfylxjnbfamqoakdhxughufpxykEreofogqafafbdcvltghdDmldumjckggstwlygdfwowlxjwKuefwwscqsujwfekObrhi"));
                Debug.Assert(pack.rotation == (ushort)(ushort)54617);
                Debug.Assert(pack.camera_id == (byte)(byte)141);
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.framerate = (float)1.8848731E38F;
            p269.camera_id = (byte)(byte)141;
            p269.rotation = (ushort)(ushort)54617;
            p269.resolution_v = (ushort)(ushort)65390;
            p269.status = (byte)(byte)253;
            p269.bitrate = (uint)2049334668U;
            p269.resolution_h = (ushort)(ushort)15505;
            p269.uri_SET("ufqukllhwjyedgrnicpfhdzzbhshhdQeXelfPDEmfwrHywhxhvskzOwygdjioiuhfnWtysUKukenruyejeZwkxonfylxjnbfamqoakdhxughufpxykEreofogqafafbdcvltghdDmldumjckggstwlygdfwowlxjwKuefwwscqsujwfekObrhi", PH) ;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_v == (ushort)(ushort)17138);
                Debug.Assert(pack.framerate == (float) -1.0235866E37F);
                Debug.Assert(pack.target_system == (byte)(byte)71);
                Debug.Assert(pack.uri_LEN(ph) == 24);
                Debug.Assert(pack.uri_TRY(ph).Equals("lajzmuamNjhehwvmKcsiKmin"));
                Debug.Assert(pack.resolution_h == (ushort)(ushort)48696);
                Debug.Assert(pack.rotation == (ushort)(ushort)55393);
                Debug.Assert(pack.bitrate == (uint)664873903U);
                Debug.Assert(pack.target_component == (byte)(byte)32);
                Debug.Assert(pack.camera_id == (byte)(byte)111);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.resolution_h = (ushort)(ushort)48696;
            p270.target_system = (byte)(byte)71;
            p270.rotation = (ushort)(ushort)55393;
            p270.camera_id = (byte)(byte)111;
            p270.resolution_v = (ushort)(ushort)17138;
            p270.framerate = (float) -1.0235866E37F;
            p270.uri_SET("lajzmuamNjhehwvmKcsiKmin", PH) ;
            p270.bitrate = (uint)664873903U;
            p270.target_component = (byte)(byte)32;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ssid_LEN(ph) == 4);
                Debug.Assert(pack.ssid_TRY(ph).Equals("tnNm"));
                Debug.Assert(pack.password_LEN(ph) == 35);
                Debug.Assert(pack.password_TRY(ph).Equals("ACrgqsnmgjhjedgpgyfppxlYbocMywkryxt"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.password_SET("ACrgqsnmgjhjedgpgyfppxlYbocMywkryxt", PH) ;
            p299.ssid_SET("tnNm", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_version == (ushort)(ushort)11808);
                Debug.Assert(pack.max_version == (ushort)(ushort)16380);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)67, (byte)223, (byte)9, (byte)49, (byte)232, (byte)94, (byte)205, (byte)244}));
                Debug.Assert(pack.version == (ushort)(ushort)1497);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)165, (byte)115, (byte)164, (byte)5, (byte)172, (byte)23, (byte)134, (byte)107}));
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.max_version = (ushort)(ushort)16380;
            p300.library_version_hash_SET(new byte[] {(byte)165, (byte)115, (byte)164, (byte)5, (byte)172, (byte)23, (byte)134, (byte)107}, 0) ;
            p300.min_version = (ushort)(ushort)11808;
            p300.spec_version_hash_SET(new byte[] {(byte)67, (byte)223, (byte)9, (byte)49, (byte)232, (byte)94, (byte)205, (byte)244}, 0) ;
            p300.version = (ushort)(ushort)1497;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)6390099169165646808L);
                Debug.Assert(pack.sub_mode == (byte)(byte)137);
                Debug.Assert(pack.mode == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)23363);
                Debug.Assert(pack.uptime_sec == (uint)1707704257U);
                Debug.Assert(pack.health == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE;
            p310.vendor_specific_status_code = (ushort)(ushort)23363;
            p310.time_usec = (ulong)6390099169165646808L;
            p310.sub_mode = (byte)(byte)137;
            p310.uptime_sec = (uint)1707704257U;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sw_vcs_commit == (uint)2996977504U);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)248);
                Debug.Assert(pack.sw_version_major == (byte)(byte)120);
                Debug.Assert(pack.uptime_sec == (uint)3972778743U);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)215, (byte)191, (byte)230, (byte)40, (byte)161, (byte)209, (byte)118, (byte)18, (byte)46, (byte)58, (byte)203, (byte)104, (byte)31, (byte)52, (byte)218, (byte)221}));
                Debug.Assert(pack.hw_version_major == (byte)(byte)229);
                Debug.Assert(pack.time_usec == (ulong)7454283302173802084L);
                Debug.Assert(pack.name_LEN(ph) == 73);
                Debug.Assert(pack.name_TRY(ph).Equals("pmnnsvDkwmqoxlpzOwoywfhfrnpyjyhwffzYixxjwGpGukbfgHmgzvzgebcjibpnjjjiuajge"));
                Debug.Assert(pack.hw_version_minor == (byte)(byte)145);
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.sw_version_minor = (byte)(byte)248;
            p311.sw_vcs_commit = (uint)2996977504U;
            p311.uptime_sec = (uint)3972778743U;
            p311.name_SET("pmnnsvDkwmqoxlpzOwoywfhfrnpyjyhwffzYixxjwGpGukbfgHmgzvzgebcjibpnjjjiuajge", PH) ;
            p311.time_usec = (ulong)7454283302173802084L;
            p311.hw_version_minor = (byte)(byte)145;
            p311.hw_version_major = (byte)(byte)229;
            p311.sw_version_major = (byte)(byte)120;
            p311.hw_unique_id_SET(new byte[] {(byte)215, (byte)191, (byte)230, (byte)40, (byte)161, (byte)209, (byte)118, (byte)18, (byte)46, (byte)58, (byte)203, (byte)104, (byte)31, (byte)52, (byte)218, (byte)221}, 0) ;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)173);
                Debug.Assert(pack.param_index == (short)(short)32160);
                Debug.Assert(pack.target_component == (byte)(byte)48);
                Debug.Assert(pack.param_id_LEN(ph) == 6);
                Debug.Assert(pack.param_id_TRY(ph).Equals("jhhipN"));
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_index = (short)(short)32160;
            p320.target_component = (byte)(byte)48;
            p320.target_system = (byte)(byte)173;
            p320.param_id_SET("jhhipN", PH) ;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)212);
                Debug.Assert(pack.target_system == (byte)(byte)245);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)245;
            p321.target_component = (byte)(byte)212;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 16);
                Debug.Assert(pack.param_id_TRY(ph).Equals("ydKJjjyueyQimagt"));
                Debug.Assert(pack.param_index == (ushort)(ushort)22645);
                Debug.Assert(pack.param_count == (ushort)(ushort)25577);
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
                Debug.Assert(pack.param_value_LEN(ph) == 6);
                Debug.Assert(pack.param_value_TRY(ph).Equals("rduoft"));
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16;
            p322.param_index = (ushort)(ushort)22645;
            p322.param_count = (ushort)(ushort)25577;
            p322.param_value_SET("rduoft", PH) ;
            p322.param_id_SET("ydKJjjyueyQimagt", PH) ;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)133);
                Debug.Assert(pack.param_value_LEN(ph) == 95);
                Debug.Assert(pack.param_value_TRY(ph).Equals("oLDvdwkWvushkgnysdiaBqgnKVfrIiXbeetflgykkqijPkmesgeceyqemiwlopoqzjdufxSYibkgukhrkievXyoewhadcSf"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
                Debug.Assert(pack.target_system == (byte)(byte)214);
                Debug.Assert(pack.param_id_LEN(ph) == 14);
                Debug.Assert(pack.param_id_TRY(ph).Equals("sJhkumixygaajh"));
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32;
            p323.param_value_SET("oLDvdwkWvushkgnysdiaBqgnKVfrIiXbeetflgykkqijPkmesgeceyqemiwlopoqzjdufxSYibkgukhrkievXyoewhadcSf", PH) ;
            p323.param_id_SET("sJhkumixygaajh", PH) ;
            p323.target_system = (byte)(byte)214;
            p323.target_component = (byte)(byte)133;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
                Debug.Assert(pack.param_value_LEN(ph) == 64);
                Debug.Assert(pack.param_value_TRY(ph).Equals("xrvcweqvasqaueJuqXmabdsBhdsyrmjHlhcElmsxzfzzqezvqtaiwghjgciiSbsp"));
                Debug.Assert(pack.param_id_LEN(ph) == 10);
                Debug.Assert(pack.param_id_TRY(ph).Equals("fwyogpruim"));
                Debug.Assert(pack.param_result == PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_result = PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED;
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM;
            p324.param_value_SET("xrvcweqvasqaueJuqXmabdsBhdsyrmjHlhcElmsxzfzzqezvqtaiwghjgciiSbsp", PH) ;
            p324.param_id_SET("fwyogpruim", PH) ;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
                Debug.Assert(pack.time_usec == (ulong)5065219838022091715L);
                Debug.Assert(pack.max_distance == (ushort)(ushort)52223);
                Debug.Assert(pack.increment == (byte)(byte)209);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)6050, (ushort)8241, (ushort)43889, (ushort)6525, (ushort)7588, (ushort)58546, (ushort)41195, (ushort)2563, (ushort)54677, (ushort)53085, (ushort)41533, (ushort)43633, (ushort)12262, (ushort)45499, (ushort)30407, (ushort)57184, (ushort)16278, (ushort)33986, (ushort)39759, (ushort)57441, (ushort)54638, (ushort)8270, (ushort)901, (ushort)16654, (ushort)44717, (ushort)55219, (ushort)33536, (ushort)47122, (ushort)14152, (ushort)2279, (ushort)63434, (ushort)23218, (ushort)41299, (ushort)32081, (ushort)13938, (ushort)57857, (ushort)48217, (ushort)32685, (ushort)15901, (ushort)27929, (ushort)51576, (ushort)16229, (ushort)49199, (ushort)13289, (ushort)43548, (ushort)38624, (ushort)48993, (ushort)45454, (ushort)50044, (ushort)22870, (ushort)16336, (ushort)5621, (ushort)32695, (ushort)27327, (ushort)47308, (ushort)63215, (ushort)48590, (ushort)37224, (ushort)49755, (ushort)55325, (ushort)20022, (ushort)8327, (ushort)56846, (ushort)24455, (ushort)20880, (ushort)12779, (ushort)27097, (ushort)8293, (ushort)18554, (ushort)60224, (ushort)6885, (ushort)46399}));
                Debug.Assert(pack.min_distance == (ushort)(ushort)62193);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.min_distance = (ushort)(ushort)62193;
            p330.increment = (byte)(byte)209;
            p330.max_distance = (ushort)(ushort)52223;
            p330.time_usec = (ulong)5065219838022091715L;
            p330.distances_SET(new ushort[] {(ushort)6050, (ushort)8241, (ushort)43889, (ushort)6525, (ushort)7588, (ushort)58546, (ushort)41195, (ushort)2563, (ushort)54677, (ushort)53085, (ushort)41533, (ushort)43633, (ushort)12262, (ushort)45499, (ushort)30407, (ushort)57184, (ushort)16278, (ushort)33986, (ushort)39759, (ushort)57441, (ushort)54638, (ushort)8270, (ushort)901, (ushort)16654, (ushort)44717, (ushort)55219, (ushort)33536, (ushort)47122, (ushort)14152, (ushort)2279, (ushort)63434, (ushort)23218, (ushort)41299, (ushort)32081, (ushort)13938, (ushort)57857, (ushort)48217, (ushort)32685, (ushort)15901, (ushort)27929, (ushort)51576, (ushort)16229, (ushort)49199, (ushort)13289, (ushort)43548, (ushort)38624, (ushort)48993, (ushort)45454, (ushort)50044, (ushort)22870, (ushort)16336, (ushort)5621, (ushort)32695, (ushort)27327, (ushort)47308, (ushort)63215, (ushort)48590, (ushort)37224, (ushort)49755, (ushort)55325, (ushort)20022, (ushort)8327, (ushort)56846, (ushort)24455, (ushort)20880, (ushort)12779, (ushort)27097, (ushort)8293, (ushort)18554, (ushort)60224, (ushort)6885, (ushort)46399}, 0) ;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
        }
    }
}