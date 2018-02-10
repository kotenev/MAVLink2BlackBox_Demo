
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
                    ulong id = id__C(value);
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
                        default:
                            throw  new ArgumentException("Unknown enum " + value);
                    }
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
                        default:
                            throw  new ArgumentException("Unknown enum " + value);
                    }
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
                        default:
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 7, data, 260);
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
                        default:
                            throw  new ArgumentException("Unknown enum " + value);
                    }
                    BitUtils.set_bits(id, 7, data, 248);
                }
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
        new class ARRAY_TEST_0 : GroundControl.ARRAY_TEST_0
        {
            public ushort[] ar_u16 //Value array
            {
                get {return ar_u16_GET(new ushort[4], 0);}
            }
            public ushort[]ar_u16_GET(ushort[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 0, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public uint[] ar_u32 //Value array
            {
                get {return ar_u32_GET(new uint[4], 0);}
            }
            public uint[]ar_u32_GET(uint[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 8, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (uint)((uint) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }
            public byte v1 //Stub field
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  24, 1));}
            }

            public sbyte[] ar_i8 //Value array
            {
                get {return ar_i8_GET(new sbyte[4], 0);}
            }
            public sbyte[]ar_i8_GET(sbyte[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 25, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (sbyte)((sbyte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public byte[] ar_u8 //Value array
            {
                get {return ar_u8_GET(new byte[4], 0);}
            }
            public byte[]ar_u8_GET(byte[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 29, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
        }
        new class ARRAY_TEST_1 : GroundControl.ARRAY_TEST_1
        {
            public uint[] ar_u32 //Value array
            {
                get {return ar_u32_GET(new uint[4], 0);}
            }
            public uint[]ar_u32_GET(uint[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 0, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (uint)((uint) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }
        }
        new class ARRAY_TEST_3 : GroundControl.ARRAY_TEST_3
        {
            public uint[] ar_u32 //Value array
            {
                get {return ar_u32_GET(new uint[4], 0);}
            }
            public uint[]ar_u32_GET(uint[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 0, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (uint)((uint) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }
            public byte v //Stub field
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
            }
        }
        new class ARRAY_TEST_4 : GroundControl.ARRAY_TEST_4
        {
            public uint[] ar_u32 //Value array
            {
                get {return ar_u32_GET(new uint[4], 0);}
            }
            public uint[]ar_u32_GET(uint[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 0, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (uint)((uint) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }
            public byte v //Stub field
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  16, 1));}
            }
        }
        new class ARRAY_TEST_5 : GroundControl.ARRAY_TEST_5
        {
            public string c1_TRY(Inside ph)//Value array
            {
                if(ph.field_bit !=  0 && !try_visit_field(ph, 0)  ||  !try_visit_item(ph, 0)) return null;
                return new string(c1_GET(ph, new char[ph.items], 0));
            }
            public char[]c1_GET(Inside ph, char[] dst_ch, int pos) //Value array
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int c1_LEN(Inside ph)
            {
                return (ph.field_bit !=  0 && !try_visit_field(ph, 0)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            } public string c2_TRY(Inside ph)//Value array
            {
                if(ph.field_bit !=  1 && !try_visit_field(ph, 1)  ||  !try_visit_item(ph, 0)) return null;
                return new string(c2_GET(ph, new char[ph.items], 0));
            }
            public char[]c2_GET(Inside ph, char[] dst_ch, int pos) //Value array
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int c2_LEN(Inside ph)
            {
                return (ph.field_bit !=  1 && !try_visit_field(ph, 1)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class ARRAY_TEST_6 : GroundControl.ARRAY_TEST_6
        {
            public ushort v2 //Stub field
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public ushort[] ar_u16 //Value array
            {
                get {return ar_u16_GET(new ushort[2], 0);}
            }
            public ushort[]ar_u16_GET(ushort[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 2, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public uint v3 //Stub field
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  6, 4));}
            }

            public uint[] ar_u32 //Value array
            {
                get {return ar_u32_GET(new uint[2], 0);}
            }
            public uint[]ar_u32_GET(uint[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 10, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (uint)((uint) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }
            public byte v1 //Stub field
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  18, 1));}
            }

            public int[] ar_i32 //Value array
            {
                get {return ar_i32_GET(new int[2], 0);}
            }
            public int[]ar_i32_GET(int[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 19, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (int)((int) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }
            public short[] ar_i16 //Value array
            {
                get {return ar_i16_GET(new short[2], 0);}
            }
            public short[]ar_i16_GET(short[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 27, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (short)((short) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public byte[] ar_u8 //Value array
            {
                get {return ar_u8_GET(new byte[2], 0);}
            }
            public byte[]ar_u8_GET(byte[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 31, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public sbyte[] ar_i8 //Value array
            {
                get {return ar_i8_GET(new sbyte[2], 0);}
            }
            public sbyte[]ar_i8_GET(sbyte[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 33, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (sbyte)((sbyte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public double[] ar_d //Value array
            {
                get {return ar_d_GET(new double[2], 0);}
            }
            public double[]ar_d_GET(double[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 35, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 8)
                    dst_ch[pos] = (double)(BitConverter.Int64BitsToDouble(BitUtils.get_bytes(data, BYTE, 8)));
                return dst_ch;
            }
            public float[] ar_f //Value array
            {
                get {return ar_f_GET(new float[2], 0);}
            }
            public float[]ar_f_GET(float[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 51, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            } public string ar_c_TRY(Inside ph)//Value array
            {
                if(ph.field_bit !=  472 && !try_visit_field(ph, 472)  ||  !try_visit_item(ph, 0)) return null;
                return new string(ar_c_GET(ph, new char[ph.items], 0));
            }
            public char[]ar_c_GET(Inside ph, char[] dst_ch, int pos) //Value array
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int ar_c_LEN(Inside ph)
            {
                return (ph.field_bit !=  472 && !try_visit_field(ph, 472)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class ARRAY_TEST_7 : GroundControl.ARRAY_TEST_7
        {
            public ushort[] ar_u16 //Value array
            {
                get {return ar_u16_GET(new ushort[2], 0);}
            }
            public ushort[]ar_u16_GET(ushort[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 0, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public uint[] ar_u32 //Value array
            {
                get {return ar_u32_GET(new uint[2], 0);}
            }
            public uint[]ar_u32_GET(uint[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 4, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (uint)((uint) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }
            public double[] ar_d //Value array
            {
                get {return ar_d_GET(new double[2], 0);}
            }
            public double[]ar_d_GET(double[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 12, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 8)
                    dst_ch[pos] = (double)(BitConverter.Int64BitsToDouble(BitUtils.get_bytes(data, BYTE, 8)));
                return dst_ch;
            }
            public float[] ar_f //Value array
            {
                get {return ar_f_GET(new float[2], 0);}
            }
            public float[]ar_f_GET(float[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 28, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            public int[] ar_i32 //Value array
            {
                get {return ar_i32_GET(new int[2], 0);}
            }
            public int[]ar_i32_GET(int[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 36, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (int)((int) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }
            public short[] ar_i16 //Value array
            {
                get {return ar_i16_GET(new short[2], 0);}
            }
            public short[]ar_i16_GET(short[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 44, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (short)((short) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public byte[] ar_u8 //Value array
            {
                get {return ar_u8_GET(new byte[2], 0);}
            }
            public byte[]ar_u8_GET(byte[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 48, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public sbyte[] ar_i8 //Value array
            {
                get {return ar_i8_GET(new sbyte[2], 0);}
            }
            public sbyte[]ar_i8_GET(sbyte[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 50, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (sbyte)((sbyte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            } public string ar_c_TRY(Inside ph)//Value array
            {
                if(ph.field_bit !=  416 && !try_visit_field(ph, 416)  ||  !try_visit_item(ph, 0)) return null;
                return new string(ar_c_GET(ph, new char[ph.items], 0));
            }
            public char[]ar_c_GET(Inside ph, char[] dst_ch, int pos) //Value array
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int ar_c_LEN(Inside ph)
            {
                return (ph.field_bit !=  416 && !try_visit_field(ph, 416)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class ARRAY_TEST_8 : GroundControl.ARRAY_TEST_8
        {
            public ushort[] ar_u16 //Value array
            {
                get {return ar_u16_GET(new ushort[2], 0);}
            }
            public ushort[]ar_u16_GET(ushort[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 0, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public uint v3 //Stub field
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  4, 4));}
            }

            public double[] ar_d //Value array
            {
                get {return ar_d_GET(new double[2], 0);}
            }
            public double[]ar_d_GET(double[] dst_ch, int pos)  //Value array
            {
                for(int BYTE = 8, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 8)
                    dst_ch[pos] = (double)(BitConverter.Int64BitsToDouble(BitUtils.get_bytes(data, BYTE, 8)));
                return dst_ch;
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
            public void OnARRAY_TEST_0Receive_direct(Channel src, Inside ph, ARRAY_TEST_0 pack) {OnARRAY_TEST_0Receive(this, ph,  pack);}
            public event ARRAY_TEST_0ReceiveHandler OnARRAY_TEST_0Receive;
            public delegate void ARRAY_TEST_0ReceiveHandler(Channel src, Inside ph, ARRAY_TEST_0 pack);
            public void OnARRAY_TEST_1Receive_direct(Channel src, Inside ph, ARRAY_TEST_1 pack) {OnARRAY_TEST_1Receive(this, ph,  pack);}
            public event ARRAY_TEST_1ReceiveHandler OnARRAY_TEST_1Receive;
            public delegate void ARRAY_TEST_1ReceiveHandler(Channel src, Inside ph, ARRAY_TEST_1 pack);
            public void OnARRAY_TEST_3Receive_direct(Channel src, Inside ph, ARRAY_TEST_3 pack) {OnARRAY_TEST_3Receive(this, ph,  pack);}
            public event ARRAY_TEST_3ReceiveHandler OnARRAY_TEST_3Receive;
            public delegate void ARRAY_TEST_3ReceiveHandler(Channel src, Inside ph, ARRAY_TEST_3 pack);
            public void OnARRAY_TEST_4Receive_direct(Channel src, Inside ph, ARRAY_TEST_4 pack) {OnARRAY_TEST_4Receive(this, ph,  pack);}
            public event ARRAY_TEST_4ReceiveHandler OnARRAY_TEST_4Receive;
            public delegate void ARRAY_TEST_4ReceiveHandler(Channel src, Inside ph, ARRAY_TEST_4 pack);
            public void OnARRAY_TEST_5Receive_direct(Channel src, Inside ph, ARRAY_TEST_5 pack) {OnARRAY_TEST_5Receive(this, ph,  pack);}
            public event ARRAY_TEST_5ReceiveHandler OnARRAY_TEST_5Receive;
            public delegate void ARRAY_TEST_5ReceiveHandler(Channel src, Inside ph, ARRAY_TEST_5 pack);
            public void OnARRAY_TEST_6Receive_direct(Channel src, Inside ph, ARRAY_TEST_6 pack) {OnARRAY_TEST_6Receive(this, ph,  pack);}
            public event ARRAY_TEST_6ReceiveHandler OnARRAY_TEST_6Receive;
            public delegate void ARRAY_TEST_6ReceiveHandler(Channel src, Inside ph, ARRAY_TEST_6 pack);
            public void OnARRAY_TEST_7Receive_direct(Channel src, Inside ph, ARRAY_TEST_7 pack) {OnARRAY_TEST_7Receive(this, ph,  pack);}
            public event ARRAY_TEST_7ReceiveHandler OnARRAY_TEST_7Receive;
            public delegate void ARRAY_TEST_7ReceiveHandler(Channel src, Inside ph, ARRAY_TEST_7 pack);
            public void OnARRAY_TEST_8Receive_direct(Channel src, Inside ph, ARRAY_TEST_8 pack) {OnARRAY_TEST_8Receive(this, ph,  pack);}
            public event ARRAY_TEST_8ReceiveHandler OnARRAY_TEST_8Receive;
            public delegate void ARRAY_TEST_8ReceiveHandler(Channel src, Inside ph, ARRAY_TEST_8 pack);
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
                    case 150:
                        if(pack == null) return new ARRAY_TEST_0();
                        OnARRAY_TEST_0Receive(this, ph, (ARRAY_TEST_0) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 151:
                        if(pack == null) return new ARRAY_TEST_1();
                        OnARRAY_TEST_1Receive(this, ph, (ARRAY_TEST_1) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 153:
                        if(pack == null) return new ARRAY_TEST_3();
                        OnARRAY_TEST_3Receive(this, ph, (ARRAY_TEST_3) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 154:
                        if(pack == null) return new ARRAY_TEST_4();
                        OnARRAY_TEST_4Receive(this, ph, (ARRAY_TEST_4) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 155:
                        if(pack == null) return new ARRAY_TEST_5();
                        OnARRAY_TEST_5Receive(this, ph, (ARRAY_TEST_5) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 156:
                        if(pack == null) return new ARRAY_TEST_6();
                        OnARRAY_TEST_6Receive(this, ph, (ARRAY_TEST_6) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 157:
                        if(pack == null) return new ARRAY_TEST_7();
                        OnARRAY_TEST_7Receive(this, ph, (ARRAY_TEST_7) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 158:
                        if(pack == null) return new ARRAY_TEST_8();
                        OnARRAY_TEST_8Receive(this, ph, (ARRAY_TEST_8) pack);//no any host channels can receive this pack. Handle it with test channel handler
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
                Debug.Assert(pack.type == MAV_TYPE.MAV_TYPE_PARAFOIL);
                Debug.Assert(pack.mavlink_version == (byte)(byte)107);
                Debug.Assert(pack.system_status == MAV_STATE.MAV_STATE_EMERGENCY);
                Debug.Assert(pack.custom_mode == (uint)2054639992U);
                Debug.Assert(pack.autopilot == MAV_AUTOPILOT.MAV_AUTOPILOT_INVALID);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED));
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED);
            p0.system_status = MAV_STATE.MAV_STATE_EMERGENCY;
            p0.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_INVALID;
            p0.custom_mode = (uint)2054639992U;
            p0.mavlink_version = (byte)(byte)107;
            p0.type = MAV_TYPE.MAV_TYPE_PARAFOIL;
            ADV_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)20030);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)26346);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)4778);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO));
                Debug.Assert(pack.current_battery == (short)(short)25893);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)46911);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)36097);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)53321);
                Debug.Assert(pack.load == (ushort)(ushort)8392);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 55);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)32933);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.load = (ushort)(ushort)8392;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO);
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.drop_rate_comm = (ushort)(ushort)26346;
            p1.battery_remaining = (sbyte)(sbyte) - 55;
            p1.voltage_battery = (ushort)(ushort)20030;
            p1.errors_count3 = (ushort)(ushort)46911;
            p1.errors_count1 = (ushort)(ushort)4778;
            p1.errors_count4 = (ushort)(ushort)53321;
            p1.errors_count2 = (ushort)(ushort)32933;
            p1.errors_comm = (ushort)(ushort)36097;
            p1.current_battery = (short)(short)25893;
            ADV_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)1859178270092229662L);
                Debug.Assert(pack.time_boot_ms == (uint)1059635601U);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)1859178270092229662L;
            p2.time_boot_ms = (uint)1059635601U;
            ADV_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)280139503U);
                Debug.Assert(pack.y == (float)3.0093792E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.z == (float)1.2275614E38F);
                Debug.Assert(pack.vz == (float)2.751815E38F);
                Debug.Assert(pack.vy == (float) -1.2764991E38F);
                Debug.Assert(pack.yaw_rate == (float)1.5704667E38F);
                Debug.Assert(pack.vx == (float)8.762531E37F);
                Debug.Assert(pack.afx == (float)3.2684574E38F);
                Debug.Assert(pack.afz == (float) -2.1165327E37F);
                Debug.Assert(pack.yaw == (float) -9.462302E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)54476);
                Debug.Assert(pack.x == (float)2.5837135E38F);
                Debug.Assert(pack.afy == (float)5.8087026E37F);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.yaw_rate = (float)1.5704667E38F;
            p3.z = (float)1.2275614E38F;
            p3.afx = (float)3.2684574E38F;
            p3.afy = (float)5.8087026E37F;
            p3.vz = (float)2.751815E38F;
            p3.y = (float)3.0093792E38F;
            p3.x = (float)2.5837135E38F;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p3.vy = (float) -1.2764991E38F;
            p3.afz = (float) -2.1165327E37F;
            p3.time_boot_ms = (uint)280139503U;
            p3.type_mask = (ushort)(ushort)54476;
            p3.vx = (float)8.762531E37F;
            p3.yaw = (float) -9.462302E37F;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)13);
                Debug.Assert(pack.time_usec == (ulong)8510924523317562101L);
                Debug.Assert(pack.seq == (uint)3103423027U);
                Debug.Assert(pack.target_component == (byte)(byte)80);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.target_system = (byte)(byte)13;
            p4.target_component = (byte)(byte)80;
            p4.seq = (uint)3103423027U;
            p4.time_usec = (ulong)8510924523317562101L;
            ADV_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.passkey_LEN(ph) == 5);
                Debug.Assert(pack.passkey_TRY(ph).Equals("ediso"));
                Debug.Assert(pack.version == (byte)(byte)31);
                Debug.Assert(pack.control_request == (byte)(byte)101);
                Debug.Assert(pack.target_system == (byte)(byte)184);
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.control_request = (byte)(byte)101;
            p5.version = (byte)(byte)31;
            p5.target_system = (byte)(byte)184;
            p5.passkey_SET("ediso", PH) ;
            ADV_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ack == (byte)(byte)95);
                Debug.Assert(pack.control_request == (byte)(byte)196);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)158);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.control_request = (byte)(byte)196;
            p6.ack = (byte)(byte)95;
            p6.gcs_system_id = (byte)(byte)158;
            ADV_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 6);
                Debug.Assert(pack.key_TRY(ph).Equals("yjDiJG"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("yjDiJG", PH) ;
            ADV_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)121);
                Debug.Assert(pack.custom_mode == (uint)1447707016U);
                Debug.Assert(pack.base_mode == MAV_MODE.MAV_MODE_PREFLIGHT);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.custom_mode = (uint)1447707016U;
            p11.target_system = (byte)(byte)121;
            p11.base_mode = MAV_MODE.MAV_MODE_PREFLIGHT;
            ADV_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 6);
                Debug.Assert(pack.param_id_TRY(ph).Equals("rpZqel"));
                Debug.Assert(pack.target_system == (byte)(byte)115);
                Debug.Assert(pack.param_index == (short)(short)17563);
                Debug.Assert(pack.target_component == (byte)(byte)152);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_component = (byte)(byte)152;
            p20.param_index = (short)(short)17563;
            p20.param_id_SET("rpZqel", PH) ;
            p20.target_system = (byte)(byte)115;
            ADV_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)143);
                Debug.Assert(pack.target_system == (byte)(byte)119);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)119;
            p21.target_component = (byte)(byte)143;
            ADV_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_count == (ushort)(ushort)22621);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32);
                Debug.Assert(pack.param_id_LEN(ph) == 15);
                Debug.Assert(pack.param_id_TRY(ph).Equals("FjgrqwfdOksddhx"));
                Debug.Assert(pack.param_value == (float)1.0582484E38F);
                Debug.Assert(pack.param_index == (ushort)(ushort)62397);
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32;
            p22.param_id_SET("FjgrqwfdOksddhx", PH) ;
            p22.param_index = (ushort)(ushort)62397;
            p22.param_count = (ushort)(ushort)22621;
            p22.param_value = (float)1.0582484E38F;
            ADV_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)247);
                Debug.Assert(pack.target_system == (byte)(byte)93);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64);
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("Tce"));
                Debug.Assert(pack.param_value == (float) -3.0965822E38F);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.target_system = (byte)(byte)93;
            p23.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64;
            p23.param_value = (float) -3.0965822E38F;
            p23.param_id_SET("Tce", PH) ;
            p23.target_component = (byte)(byte)247;
            ADV_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
                Debug.Assert(pack.lon == (int)1943955648);
                Debug.Assert(pack.eph == (ushort)(ushort)48522);
                Debug.Assert(pack.alt == (int) -448204953);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int)1324148689);
                Debug.Assert(pack.lat == (int)2069294562);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)2896069585U);
                Debug.Assert(pack.vel == (ushort)(ushort)44066);
                Debug.Assert(pack.epv == (ushort)(ushort)45851);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)3968723035U);
                Debug.Assert(pack.cog == (ushort)(ushort)41974);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)2722743943U);
                Debug.Assert(pack.time_usec == (ulong)3023751429529024417L);
                Debug.Assert(pack.satellites_visible == (byte)(byte)198);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)1150877071U);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.alt = (int) -448204953;
            p24.vel_acc_SET((uint)1150877071U, PH) ;
            p24.eph = (ushort)(ushort)48522;
            p24.hdg_acc_SET((uint)2896069585U, PH) ;
            p24.time_usec = (ulong)3023751429529024417L;
            p24.h_acc_SET((uint)2722743943U, PH) ;
            p24.vel = (ushort)(ushort)44066;
            p24.epv = (ushort)(ushort)45851;
            p24.satellites_visible = (byte)(byte)198;
            p24.cog = (ushort)(ushort)41974;
            p24.v_acc_SET((uint)3968723035U, PH) ;
            p24.lat = (int)2069294562;
            p24.lon = (int)1943955648;
            p24.alt_ellipsoid_SET((int)1324148689, PH) ;
            p24.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_PPP;
            ADV_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)24, (byte)215, (byte)65, (byte)97, (byte)20, (byte)242, (byte)238, (byte)181, (byte)132, (byte)83, (byte)201, (byte)86, (byte)138, (byte)252, (byte)209, (byte)137, (byte)55, (byte)233, (byte)192, (byte)124}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)237, (byte)42, (byte)248, (byte)238, (byte)204, (byte)0, (byte)35, (byte)22, (byte)181, (byte)46, (byte)58, (byte)246, (byte)238, (byte)41, (byte)208, (byte)207, (byte)51, (byte)70, (byte)72, (byte)192}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)62);
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)2, (byte)49, (byte)40, (byte)133, (byte)96, (byte)66, (byte)8, (byte)174, (byte)143, (byte)182, (byte)97, (byte)138, (byte)202, (byte)52, (byte)121, (byte)183, (byte)159, (byte)231, (byte)170, (byte)243}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)203, (byte)196, (byte)67, (byte)197, (byte)15, (byte)60, (byte)234, (byte)237, (byte)144, (byte)127, (byte)77, (byte)12, (byte)151, (byte)190, (byte)109, (byte)139, (byte)159, (byte)19, (byte)220, (byte)56}));
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)33, (byte)72, (byte)106, (byte)234, (byte)67, (byte)214, (byte)205, (byte)105, (byte)28, (byte)121, (byte)69, (byte)116, (byte)229, (byte)94, (byte)133, (byte)41, (byte)32, (byte)229, (byte)143, (byte)4}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_prn_SET(new byte[] {(byte)2, (byte)49, (byte)40, (byte)133, (byte)96, (byte)66, (byte)8, (byte)174, (byte)143, (byte)182, (byte)97, (byte)138, (byte)202, (byte)52, (byte)121, (byte)183, (byte)159, (byte)231, (byte)170, (byte)243}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)33, (byte)72, (byte)106, (byte)234, (byte)67, (byte)214, (byte)205, (byte)105, (byte)28, (byte)121, (byte)69, (byte)116, (byte)229, (byte)94, (byte)133, (byte)41, (byte)32, (byte)229, (byte)143, (byte)4}, 0) ;
            p25.satellites_visible = (byte)(byte)62;
            p25.satellite_azimuth_SET(new byte[] {(byte)203, (byte)196, (byte)67, (byte)197, (byte)15, (byte)60, (byte)234, (byte)237, (byte)144, (byte)127, (byte)77, (byte)12, (byte)151, (byte)190, (byte)109, (byte)139, (byte)159, (byte)19, (byte)220, (byte)56}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)24, (byte)215, (byte)65, (byte)97, (byte)20, (byte)242, (byte)238, (byte)181, (byte)132, (byte)83, (byte)201, (byte)86, (byte)138, (byte)252, (byte)209, (byte)137, (byte)55, (byte)233, (byte)192, (byte)124}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)237, (byte)42, (byte)248, (byte)238, (byte)204, (byte)0, (byte)35, (byte)22, (byte)181, (byte)46, (byte)58, (byte)246, (byte)238, (byte)41, (byte)208, (byte)207, (byte)51, (byte)70, (byte)72, (byte)192}, 0) ;
            ADV_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (short)(short) -22935);
                Debug.Assert(pack.ygyro == (short)(short) -247);
                Debug.Assert(pack.yacc == (short)(short)7071);
                Debug.Assert(pack.time_boot_ms == (uint)2335720304U);
                Debug.Assert(pack.zgyro == (short)(short) -13686);
                Debug.Assert(pack.ymag == (short)(short)14861);
                Debug.Assert(pack.zacc == (short)(short) -2047);
                Debug.Assert(pack.xacc == (short)(short) -9639);
                Debug.Assert(pack.xmag == (short)(short)2317);
                Debug.Assert(pack.zmag == (short)(short)9246);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.yacc = (short)(short)7071;
            p26.time_boot_ms = (uint)2335720304U;
            p26.xmag = (short)(short)2317;
            p26.xacc = (short)(short) -9639;
            p26.xgyro = (short)(short) -22935;
            p26.ymag = (short)(short)14861;
            p26.ygyro = (short)(short) -247;
            p26.zacc = (short)(short) -2047;
            p26.zmag = (short)(short)9246;
            p26.zgyro = (short)(short) -13686;
            ADV_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short) -10974);
                Debug.Assert(pack.xmag == (short)(short) -16516);
                Debug.Assert(pack.ygyro == (short)(short)24659);
                Debug.Assert(pack.time_usec == (ulong)8506533635886027868L);
                Debug.Assert(pack.xgyro == (short)(short)18946);
                Debug.Assert(pack.zgyro == (short)(short)11916);
                Debug.Assert(pack.yacc == (short)(short) -28378);
                Debug.Assert(pack.zmag == (short)(short)17849);
                Debug.Assert(pack.zacc == (short)(short)31863);
                Debug.Assert(pack.ymag == (short)(short) -7304);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.xgyro = (short)(short)18946;
            p27.zacc = (short)(short)31863;
            p27.ygyro = (short)(short)24659;
            p27.zmag = (short)(short)17849;
            p27.ymag = (short)(short) -7304;
            p27.xacc = (short)(short) -10974;
            p27.yacc = (short)(short) -28378;
            p27.time_usec = (ulong)8506533635886027868L;
            p27.xmag = (short)(short) -16516;
            p27.zgyro = (short)(short)11916;
            ADV_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff1 == (short)(short) -17619);
                Debug.Assert(pack.temperature == (short)(short)20357);
                Debug.Assert(pack.press_diff2 == (short)(short) -14872);
                Debug.Assert(pack.press_abs == (short)(short)18112);
                Debug.Assert(pack.time_usec == (ulong)7598071483920980801L);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_diff2 = (short)(short) -14872;
            p28.press_diff1 = (short)(short) -17619;
            p28.temperature = (short)(short)20357;
            p28.time_usec = (ulong)7598071483920980801L;
            p28.press_abs = (short)(short)18112;
            ADV_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -7986);
                Debug.Assert(pack.press_diff == (float) -5.2253024E36F);
                Debug.Assert(pack.press_abs == (float) -4.6857356E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1257380662U);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.time_boot_ms = (uint)1257380662U;
            p29.press_diff = (float) -5.2253024E36F;
            p29.press_abs = (float) -4.6857356E37F;
            p29.temperature = (short)(short) -7986;
            ADV_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float)2.5625147E38F);
                Debug.Assert(pack.rollspeed == (float)1.4180985E38F);
                Debug.Assert(pack.roll == (float) -1.9453904E38F);
                Debug.Assert(pack.yaw == (float) -2.2055403E38F);
                Debug.Assert(pack.yawspeed == (float)2.8109213E38F);
                Debug.Assert(pack.pitchspeed == (float) -1.872976E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4091480350U);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.time_boot_ms = (uint)4091480350U;
            p30.rollspeed = (float)1.4180985E38F;
            p30.roll = (float) -1.9453904E38F;
            p30.yawspeed = (float)2.8109213E38F;
            p30.yaw = (float) -2.2055403E38F;
            p30.pitch = (float)2.5625147E38F;
            p30.pitchspeed = (float) -1.872976E38F;
            ADV_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q4 == (float)6.724507E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3933129731U);
                Debug.Assert(pack.q1 == (float)7.102819E37F);
                Debug.Assert(pack.rollspeed == (float) -1.7864103E38F);
                Debug.Assert(pack.q3 == (float)1.0948356E38F);
                Debug.Assert(pack.pitchspeed == (float)3.1744354E38F);
                Debug.Assert(pack.q2 == (float)2.991536E38F);
                Debug.Assert(pack.yawspeed == (float) -9.49792E37F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.yawspeed = (float) -9.49792E37F;
            p31.pitchspeed = (float)3.1744354E38F;
            p31.rollspeed = (float) -1.7864103E38F;
            p31.q1 = (float)7.102819E37F;
            p31.q4 = (float)6.724507E37F;
            p31.q3 = (float)1.0948356E38F;
            p31.time_boot_ms = (uint)3933129731U;
            p31.q2 = (float)2.991536E38F;
            ADV_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)2.4177924E38F);
                Debug.Assert(pack.y == (float)2.0515063E38F);
                Debug.Assert(pack.vz == (float) -1.172887E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1327693193U);
                Debug.Assert(pack.x == (float) -3.5992743E37F);
                Debug.Assert(pack.vx == (float)2.2398202E38F);
                Debug.Assert(pack.vy == (float) -2.6573186E38F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.vy = (float) -2.6573186E38F;
            p32.z = (float)2.4177924E38F;
            p32.vx = (float)2.2398202E38F;
            p32.vz = (float) -1.172887E38F;
            p32.time_boot_ms = (uint)1327693193U;
            p32.y = (float)2.0515063E38F;
            p32.x = (float) -3.5992743E37F;
            ADV_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1543868897);
                Debug.Assert(pack.vz == (short)(short) -17437);
                Debug.Assert(pack.lon == (int) -1391052241);
                Debug.Assert(pack.vx == (short)(short)2899);
                Debug.Assert(pack.alt == (int) -1135905480);
                Debug.Assert(pack.time_boot_ms == (uint)2981528652U);
                Debug.Assert(pack.vy == (short)(short)20815);
                Debug.Assert(pack.hdg == (ushort)(ushort)60917);
                Debug.Assert(pack.relative_alt == (int)704447757);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.vx = (short)(short)2899;
            p33.vz = (short)(short) -17437;
            p33.alt = (int) -1135905480;
            p33.lat = (int)1543868897;
            p33.vy = (short)(short)20815;
            p33.hdg = (ushort)(ushort)60917;
            p33.time_boot_ms = (uint)2981528652U;
            p33.lon = (int) -1391052241;
            p33.relative_alt = (int)704447757;
            ADV_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.port == (byte)(byte)77);
                Debug.Assert(pack.chan4_scaled == (short)(short) -3257);
                Debug.Assert(pack.rssi == (byte)(byte)180);
                Debug.Assert(pack.chan7_scaled == (short)(short)5397);
                Debug.Assert(pack.chan6_scaled == (short)(short)25348);
                Debug.Assert(pack.chan5_scaled == (short)(short) -7550);
                Debug.Assert(pack.chan8_scaled == (short)(short) -24074);
                Debug.Assert(pack.chan2_scaled == (short)(short) -28149);
                Debug.Assert(pack.chan3_scaled == (short)(short)24135);
                Debug.Assert(pack.chan1_scaled == (short)(short) -13858);
                Debug.Assert(pack.time_boot_ms == (uint)2175864070U);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.port = (byte)(byte)77;
            p34.chan8_scaled = (short)(short) -24074;
            p34.chan7_scaled = (short)(short)5397;
            p34.chan2_scaled = (short)(short) -28149;
            p34.chan3_scaled = (short)(short)24135;
            p34.chan6_scaled = (short)(short)25348;
            p34.chan4_scaled = (short)(short) -3257;
            p34.chan1_scaled = (short)(short) -13858;
            p34.time_boot_ms = (uint)2175864070U;
            p34.rssi = (byte)(byte)180;
            p34.chan5_scaled = (short)(short) -7550;
            ADV_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)542599341U);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)50694);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)4307);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)4510);
                Debug.Assert(pack.rssi == (byte)(byte)159);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)23039);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)26618);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)17600);
                Debug.Assert(pack.port == (byte)(byte)140);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)41021);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)28320);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan3_raw = (ushort)(ushort)17600;
            p35.chan8_raw = (ushort)(ushort)41021;
            p35.time_boot_ms = (uint)542599341U;
            p35.chan7_raw = (ushort)(ushort)4510;
            p35.chan4_raw = (ushort)(ushort)4307;
            p35.chan5_raw = (ushort)(ushort)26618;
            p35.chan2_raw = (ushort)(ushort)50694;
            p35.chan6_raw = (ushort)(ushort)23039;
            p35.port = (byte)(byte)140;
            p35.chan1_raw = (ushort)(ushort)28320;
            p35.rssi = (byte)(byte)159;
            ADV_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)1440);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)27660);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)44732);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)25068);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)57187);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)41877);
                Debug.Assert(pack.port == (byte)(byte)96);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)38418);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)33188);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)47455);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)21068);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)18150);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)50199);
                Debug.Assert(pack.time_usec == (uint)2457495061U);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)25789);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)53660);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)3409);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)40551);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo2_raw = (ushort)(ushort)21068;
            p36.servo9_raw_SET((ushort)(ushort)47455, PH) ;
            p36.servo15_raw_SET((ushort)(ushort)53660, PH) ;
            p36.servo11_raw_SET((ushort)(ushort)25068, PH) ;
            p36.servo16_raw_SET((ushort)(ushort)38418, PH) ;
            p36.servo6_raw = (ushort)(ushort)27660;
            p36.servo8_raw = (ushort)(ushort)57187;
            p36.servo4_raw = (ushort)(ushort)44732;
            p36.servo13_raw_SET((ushort)(ushort)41877, PH) ;
            p36.servo7_raw = (ushort)(ushort)3409;
            p36.time_usec = (uint)2457495061U;
            p36.servo14_raw_SET((ushort)(ushort)18150, PH) ;
            p36.servo3_raw = (ushort)(ushort)25789;
            p36.port = (byte)(byte)96;
            p36.servo12_raw_SET((ushort)(ushort)33188, PH) ;
            p36.servo5_raw = (ushort)(ushort)50199;
            p36.servo10_raw_SET((ushort)(ushort)1440, PH) ;
            p36.servo1_raw = (ushort)(ushort)40551;
            ADV_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_index == (short)(short) -175);
                Debug.Assert(pack.target_component == (byte)(byte)18);
                Debug.Assert(pack.end_index == (short)(short)13906);
                Debug.Assert(pack.target_system == (byte)(byte)146);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p37.start_index = (short)(short) -175;
            p37.target_component = (byte)(byte)18;
            p37.end_index = (short)(short)13906;
            p37.target_system = (byte)(byte)146;
            ADV_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)216);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.start_index == (short)(short)21141);
                Debug.Assert(pack.target_system == (byte)(byte)94);
                Debug.Assert(pack.end_index == (short)(short) -11531);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.target_component = (byte)(byte)216;
            p38.start_index = (short)(short)21141;
            p38.target_system = (byte)(byte)94;
            p38.end_index = (short)(short) -11531;
            p38.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            ADV_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)3.1740334E38F);
                Debug.Assert(pack.target_component == (byte)(byte)191);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_ROI);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.seq == (ushort)(ushort)22422);
                Debug.Assert(pack.y == (float) -2.7263197E38F);
                Debug.Assert(pack.target_system == (byte)(byte)30);
                Debug.Assert(pack.param3 == (float) -1.8058867E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)2);
                Debug.Assert(pack.param4 == (float)2.5342707E38F);
                Debug.Assert(pack.current == (byte)(byte)105);
                Debug.Assert(pack.param1 == (float)1.9025576E37F);
                Debug.Assert(pack.z == (float) -2.1883523E37F);
                Debug.Assert(pack.param2 == (float)3.0871308E38F);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.command = MAV_CMD.MAV_CMD_NAV_ROI;
            p39.target_component = (byte)(byte)191;
            p39.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p39.z = (float) -2.1883523E37F;
            p39.autocontinue = (byte)(byte)2;
            p39.frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p39.y = (float) -2.7263197E38F;
            p39.param2 = (float)3.0871308E38F;
            p39.param1 = (float)1.9025576E37F;
            p39.param4 = (float)2.5342707E38F;
            p39.param3 = (float) -1.8058867E38F;
            p39.current = (byte)(byte)105;
            p39.seq = (ushort)(ushort)22422;
            p39.x = (float)3.1740334E38F;
            p39.target_system = (byte)(byte)30;
            ADV_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.seq == (ushort)(ushort)23104);
                Debug.Assert(pack.target_system == (byte)(byte)4);
                Debug.Assert(pack.target_component == (byte)(byte)161);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_system = (byte)(byte)4;
            p40.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p40.seq = (ushort)(ushort)23104;
            p40.target_component = (byte)(byte)161;
            ADV_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)32556);
                Debug.Assert(pack.target_system == (byte)(byte)237);
                Debug.Assert(pack.target_component == (byte)(byte)36);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_component = (byte)(byte)36;
            p41.seq = (ushort)(ushort)32556;
            p41.target_system = (byte)(byte)237;
            ADV_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)16349);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)16349;
            ADV_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)226);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_component == (byte)(byte)63);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_system = (byte)(byte)226;
            p43.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p43.target_component = (byte)(byte)63;
            ADV_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.count == (ushort)(ushort)31953);
                Debug.Assert(pack.target_component == (byte)(byte)33);
                Debug.Assert(pack.target_system == (byte)(byte)26);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.target_system = (byte)(byte)26;
            p44.count = (ushort)(ushort)31953;
            p44.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p44.target_component = (byte)(byte)33;
            ADV_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)241);
                Debug.Assert(pack.target_system == (byte)(byte)184);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_system = (byte)(byte)184;
            p45.target_component = (byte)(byte)241;
            p45.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            ADV_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)43902);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)43902;
            ADV_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)116);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.type == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM2);
                Debug.Assert(pack.target_system == (byte)(byte)56);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.target_component = (byte)(byte)116;
            p47.type = MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM2;
            p47.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p47.target_system = (byte)(byte)56;
            ADV_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int)54327943);
                Debug.Assert(pack.target_system == (byte)(byte)73);
                Debug.Assert(pack.latitude == (int) -981149590);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7119480398906480813L);
                Debug.Assert(pack.longitude == (int)216879535);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.time_usec_SET((ulong)7119480398906480813L, PH) ;
            p48.latitude = (int) -981149590;
            p48.altitude = (int)54327943;
            p48.target_system = (byte)(byte)73;
            p48.longitude = (int)216879535;
            ADV_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int) -35105391);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)5359887837027652363L);
                Debug.Assert(pack.latitude == (int)360274561);
                Debug.Assert(pack.longitude == (int)1628695635);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.longitude = (int)1628695635;
            p49.latitude = (int)360274561;
            p49.altitude = (int) -35105391;
            p49.time_usec_SET((ulong)5359887837027652363L, PH) ;
            ADV_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.scale == (float)2.8294846E38F);
                Debug.Assert(pack.target_system == (byte)(byte)71);
                Debug.Assert(pack.param_index == (short)(short) -15530);
                Debug.Assert(pack.target_component == (byte)(byte)102);
                Debug.Assert(pack.param_value_max == (float) -1.7162456E38F);
                Debug.Assert(pack.param_value_min == (float)2.125997E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("kvPadbam"));
                Debug.Assert(pack.param_value0 == (float)2.410692E38F);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)83);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.parameter_rc_channel_index = (byte)(byte)83;
            p50.param_value0 = (float)2.410692E38F;
            p50.param_value_max = (float) -1.7162456E38F;
            p50.scale = (float)2.8294846E38F;
            p50.param_value_min = (float)2.125997E38F;
            p50.param_id_SET("kvPadbam", PH) ;
            p50.target_system = (byte)(byte)71;
            p50.target_component = (byte)(byte)102;
            p50.param_index = (short)(short) -15530;
            ADV_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)228);
                Debug.Assert(pack.target_component == (byte)(byte)63);
                Debug.Assert(pack.seq == (ushort)(ushort)1689);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p51.target_component = (byte)(byte)63;
            p51.seq = (ushort)(ushort)1689;
            p51.target_system = (byte)(byte)228;
            ADV_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2x == (float) -1.2192646E38F);
                Debug.Assert(pack.p1x == (float)2.2042789E38F);
                Debug.Assert(pack.p2z == (float)2.1598028E38F);
                Debug.Assert(pack.p1y == (float) -2.3923688E38F);
                Debug.Assert(pack.target_system == (byte)(byte)3);
                Debug.Assert(pack.target_component == (byte)(byte)161);
                Debug.Assert(pack.p2y == (float) -2.534516E37F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.p1z == (float) -9.553788E37F);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p1x = (float)2.2042789E38F;
            p54.p1z = (float) -9.553788E37F;
            p54.p1y = (float) -2.3923688E38F;
            p54.target_component = (byte)(byte)161;
            p54.p2z = (float)2.1598028E38F;
            p54.p2x = (float) -1.2192646E38F;
            p54.p2y = (float) -2.534516E37F;
            p54.frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p54.target_system = (byte)(byte)3;
            ADV_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2x == (float) -2.9539238E38F);
                Debug.Assert(pack.p1y == (float)9.410489E37F);
                Debug.Assert(pack.p2z == (float) -2.2665295E38F);
                Debug.Assert(pack.p1x == (float)2.2338127E38F);
                Debug.Assert(pack.p2y == (float) -1.7970312E38F);
                Debug.Assert(pack.p1z == (float) -2.0749718E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p1x = (float)2.2338127E38F;
            p55.p1z = (float) -2.0749718E38F;
            p55.p1y = (float)9.410489E37F;
            p55.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p55.p2z = (float) -2.2665295E38F;
            p55.p2y = (float) -1.7970312E38F;
            p55.p2x = (float) -2.9539238E38F;
            ADV_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitchspeed == (float)9.259327E37F);
                Debug.Assert(pack.yawspeed == (float)2.88465E38F);
                Debug.Assert(pack.rollspeed == (float) -2.784303E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {2.5716313E38F, -6.884668E37F, 6.5915484E37F, -1.2845475E38F, -3.0782676E38F, 2.847775E38F, 4.9474466E37F, -1.6835134E38F, 2.8836017E38F}));
                Debug.Assert(pack.time_usec == (ulong)7818346453199285554L);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.7923115E38F, -1.7292227E38F, 1.8092717E38F, -9.946874E37F}));
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.q_SET(new float[] {-1.7923115E38F, -1.7292227E38F, 1.8092717E38F, -9.946874E37F}, 0) ;
            p61.rollspeed = (float) -2.784303E38F;
            p61.yawspeed = (float)2.88465E38F;
            p61.time_usec = (ulong)7818346453199285554L;
            p61.pitchspeed = (float)9.259327E37F;
            p61.covariance_SET(new float[] {2.5716313E38F, -6.884668E37F, 6.5915484E37F, -1.2845475E38F, -3.0782676E38F, 2.847775E38F, 4.9474466E37F, -1.6835134E38F, 2.8836017E38F}, 0) ;
            ADV_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aspd_error == (float)3.3044305E38F);
                Debug.Assert(pack.nav_pitch == (float) -9.638767E36F);
                Debug.Assert(pack.xtrack_error == (float)2.4585298E38F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)61917);
                Debug.Assert(pack.nav_roll == (float) -2.5576916E38F);
                Debug.Assert(pack.nav_bearing == (short)(short) -5861);
                Debug.Assert(pack.alt_error == (float)3.3269265E38F);
                Debug.Assert(pack.target_bearing == (short)(short)26151);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.aspd_error = (float)3.3044305E38F;
            p62.xtrack_error = (float)2.4585298E38F;
            p62.target_bearing = (short)(short)26151;
            p62.wp_dist = (ushort)(ushort)61917;
            p62.nav_pitch = (float) -9.638767E36F;
            p62.nav_roll = (float) -2.5576916E38F;
            p62.nav_bearing = (short)(short) -5861;
            p62.alt_error = (float)3.3269265E38F;
            ADV_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-3.183637E38F, 2.7726748E38F, 1.8929843E38F, -4.4909347E37F, -1.8767484E38F, 9.350352E37F, -2.565933E38F, -9.815537E37F, -2.7682264E38F, 8.600573E37F, 2.8349398E37F, 2.520231E38F, -4.8569095E37F, -7.0328317E37F, -4.4364415E36F, 2.9114239E38F, 1.6109989E38F, 1.705614E38F, 1.1345691E38F, 1.0243503E38F, 1.6239125E37F, -7.576456E37F, -3.2468562E38F, -2.2464525E38F, 6.3233703E37F, -2.983354E36F, 2.1544629E38F, 2.990323E38F, -7.2441263E37F, 2.0991874E38F, -9.815008E37F, -1.782332E38F, 1.8843959E38F, -8.518311E37F, 3.2908662E38F, -1.3665683E38F}));
                Debug.Assert(pack.vy == (float)1.0777858E38F);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
                Debug.Assert(pack.time_usec == (ulong)43912032367534502L);
                Debug.Assert(pack.vx == (float)2.280536E37F);
                Debug.Assert(pack.lon == (int) -1129580828);
                Debug.Assert(pack.relative_alt == (int)1815804397);
                Debug.Assert(pack.lat == (int) -1417057797);
                Debug.Assert(pack.vz == (float) -3.3068873E38F);
                Debug.Assert(pack.alt == (int) -141239902);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.covariance_SET(new float[] {-3.183637E38F, 2.7726748E38F, 1.8929843E38F, -4.4909347E37F, -1.8767484E38F, 9.350352E37F, -2.565933E38F, -9.815537E37F, -2.7682264E38F, 8.600573E37F, 2.8349398E37F, 2.520231E38F, -4.8569095E37F, -7.0328317E37F, -4.4364415E36F, 2.9114239E38F, 1.6109989E38F, 1.705614E38F, 1.1345691E38F, 1.0243503E38F, 1.6239125E37F, -7.576456E37F, -3.2468562E38F, -2.2464525E38F, 6.3233703E37F, -2.983354E36F, 2.1544629E38F, 2.990323E38F, -7.2441263E37F, 2.0991874E38F, -9.815008E37F, -1.782332E38F, 1.8843959E38F, -8.518311E37F, 3.2908662E38F, -1.3665683E38F}, 0) ;
            p63.lat = (int) -1417057797;
            p63.lon = (int) -1129580828;
            p63.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS;
            p63.time_usec = (ulong)43912032367534502L;
            p63.alt = (int) -141239902;
            p63.vy = (float)1.0777858E38F;
            p63.vz = (float) -3.3068873E38F;
            p63.vx = (float)2.280536E37F;
            p63.relative_alt = (int)1815804397;
            ADV_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float) -9.855693E37F);
                Debug.Assert(pack.vz == (float) -2.8455342E38F);
                Debug.Assert(pack.x == (float)6.8446967E37F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-1.642774E38F, 1.03224795E37F, -7.3945726E37F, -2.123304E38F, -6.10875E37F, 2.4611467E38F, 2.4504307E38F, 1.3971863E38F, -9.170092E37F, -3.1647236E38F, 1.8636813E38F, -1.3427588E38F, -7.212906E36F, -1.816018E38F, 1.647626E38F, 1.205318E38F, 7.1768455E37F, 1.1795264E38F, -2.7774886E37F, -8.655964E36F, 2.96148E38F, 9.163798E37F, -1.836259E38F, -2.7352379E38F, 2.0045821E38F, -1.0947145E38F, -3.0876665E38F, -7.1617797E37F, -1.0866641E38F, -3.0353693E38F, -1.8894442E38F, 6.341273E37F, 2.3144164E38F, 2.5691978E38F, 3.095271E38F, 3.2693695E38F, 3.7242738E37F, 1.2799182E38F, 1.1962276E37F, 2.644666E37F, 2.171227E38F, 3.0621715E38F, -2.8159517E38F, -5.4791266E37F, -1.3556385E37F}));
                Debug.Assert(pack.ay == (float) -1.831138E38F);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
                Debug.Assert(pack.ax == (float)3.0097013E38F);
                Debug.Assert(pack.z == (float)1.7429585E38F);
                Debug.Assert(pack.az == (float) -6.489869E37F);
                Debug.Assert(pack.vx == (float) -2.4992608E38F);
                Debug.Assert(pack.time_usec == (ulong)6087942157589817797L);
                Debug.Assert(pack.y == (float) -5.2816764E37F);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.az = (float) -6.489869E37F;
            p64.x = (float)6.8446967E37F;
            p64.vx = (float) -2.4992608E38F;
            p64.vz = (float) -2.8455342E38F;
            p64.ay = (float) -1.831138E38F;
            p64.ax = (float)3.0097013E38F;
            p64.time_usec = (ulong)6087942157589817797L;
            p64.vy = (float) -9.855693E37F;
            p64.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
            p64.z = (float)1.7429585E38F;
            p64.covariance_SET(new float[] {-1.642774E38F, 1.03224795E37F, -7.3945726E37F, -2.123304E38F, -6.10875E37F, 2.4611467E38F, 2.4504307E38F, 1.3971863E38F, -9.170092E37F, -3.1647236E38F, 1.8636813E38F, -1.3427588E38F, -7.212906E36F, -1.816018E38F, 1.647626E38F, 1.205318E38F, 7.1768455E37F, 1.1795264E38F, -2.7774886E37F, -8.655964E36F, 2.96148E38F, 9.163798E37F, -1.836259E38F, -2.7352379E38F, 2.0045821E38F, -1.0947145E38F, -3.0876665E38F, -7.1617797E37F, -1.0866641E38F, -3.0353693E38F, -1.8894442E38F, 6.341273E37F, 2.3144164E38F, 2.5691978E38F, 3.095271E38F, 3.2693695E38F, 3.7242738E37F, 1.2799182E38F, 1.1962276E37F, 2.644666E37F, 2.171227E38F, 3.0621715E38F, -2.8159517E38F, -5.4791266E37F, -1.3556385E37F}, 0) ;
            p64.y = (float) -5.2816764E37F;
            ADV_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)51728);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)23606);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)45743);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)9222);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)1149);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)46588);
                Debug.Assert(pack.chancount == (byte)(byte)132);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)56568);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)9358);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)17321);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)8412);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)32618);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)47104);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)28630);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)27512);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)57642);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)38526);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)11100);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)52956);
                Debug.Assert(pack.rssi == (byte)(byte)214);
                Debug.Assert(pack.time_boot_ms == (uint)1099002404U);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan18_raw = (ushort)(ushort)47104;
            p65.chan14_raw = (ushort)(ushort)51728;
            p65.chan5_raw = (ushort)(ushort)1149;
            p65.chan3_raw = (ushort)(ushort)57642;
            p65.chan11_raw = (ushort)(ushort)32618;
            p65.chan6_raw = (ushort)(ushort)17321;
            p65.chan13_raw = (ushort)(ushort)38526;
            p65.chan17_raw = (ushort)(ushort)8412;
            p65.chan16_raw = (ushort)(ushort)52956;
            p65.chan15_raw = (ushort)(ushort)45743;
            p65.chan1_raw = (ushort)(ushort)56568;
            p65.chancount = (byte)(byte)132;
            p65.rssi = (byte)(byte)214;
            p65.chan2_raw = (ushort)(ushort)28630;
            p65.chan12_raw = (ushort)(ushort)11100;
            p65.chan9_raw = (ushort)(ushort)46588;
            p65.chan7_raw = (ushort)(ushort)9222;
            p65.time_boot_ms = (uint)1099002404U;
            p65.chan4_raw = (ushort)(ushort)9358;
            p65.chan8_raw = (ushort)(ushort)23606;
            p65.chan10_raw = (ushort)(ushort)27512;
            ADV_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.req_stream_id == (byte)(byte)46);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)20704);
                Debug.Assert(pack.target_component == (byte)(byte)122);
                Debug.Assert(pack.target_system == (byte)(byte)41);
                Debug.Assert(pack.start_stop == (byte)(byte)144);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.req_message_rate = (ushort)(ushort)20704;
            p66.req_stream_id = (byte)(byte)46;
            p66.start_stop = (byte)(byte)144;
            p66.target_component = (byte)(byte)122;
            p66.target_system = (byte)(byte)41;
            ADV_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_rate == (ushort)(ushort)9620);
                Debug.Assert(pack.on_off == (byte)(byte)223);
                Debug.Assert(pack.stream_id == (byte)(byte)42);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.on_off = (byte)(byte)223;
            p67.message_rate = (ushort)(ushort)9620;
            p67.stream_id = (byte)(byte)42;
            ADV_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (short)(short) -15737);
                Debug.Assert(pack.target == (byte)(byte)120);
                Debug.Assert(pack.r == (short)(short)25279);
                Debug.Assert(pack.z == (short)(short) -9305);
                Debug.Assert(pack.x == (short)(short)27641);
                Debug.Assert(pack.buttons == (ushort)(ushort)61317);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.r = (short)(short)25279;
            p69.buttons = (ushort)(ushort)61317;
            p69.z = (short)(short) -9305;
            p69.y = (short)(short) -15737;
            p69.target = (byte)(byte)120;
            p69.x = (short)(short)27641;
            ADV_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)34478);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)53343);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)43826);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)51787);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)13425);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)62842);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)58538);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)64142);
                Debug.Assert(pack.target_system == (byte)(byte)32);
                Debug.Assert(pack.target_component == (byte)(byte)204);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan6_raw = (ushort)(ushort)64142;
            p70.chan2_raw = (ushort)(ushort)58538;
            p70.chan1_raw = (ushort)(ushort)51787;
            p70.chan7_raw = (ushort)(ushort)43826;
            p70.chan4_raw = (ushort)(ushort)53343;
            p70.chan8_raw = (ushort)(ushort)13425;
            p70.target_component = (byte)(byte)204;
            p70.target_system = (byte)(byte)32;
            p70.chan5_raw = (ushort)(ushort)34478;
            p70.chan3_raw = (ushort)(ushort)62842;
            ADV_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param1 == (float)1.5096376E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)70);
                Debug.Assert(pack.z == (float) -1.2623614E38F);
                Debug.Assert(pack.current == (byte)(byte)59);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.y == (int)309542508);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT);
                Debug.Assert(pack.param3 == (float) -2.7688256E38F);
                Debug.Assert(pack.param2 == (float) -3.0970708E38F);
                Debug.Assert(pack.seq == (ushort)(ushort)54848);
                Debug.Assert(pack.x == (int)84986889);
                Debug.Assert(pack.param4 == (float)1.3687013E38F);
                Debug.Assert(pack.target_component == (byte)(byte)106);
                Debug.Assert(pack.target_system == (byte)(byte)8);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p73.param3 = (float) -2.7688256E38F;
            p73.target_component = (byte)(byte)106;
            p73.z = (float) -1.2623614E38F;
            p73.seq = (ushort)(ushort)54848;
            p73.target_system = (byte)(byte)8;
            p73.param4 = (float)1.3687013E38F;
            p73.y = (int)309542508;
            p73.command = MAV_CMD.MAV_CMD_DO_INVERTED_FLIGHT;
            p73.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p73.x = (int)84986889;
            p73.autocontinue = (byte)(byte)70;
            p73.param1 = (float)1.5096376E38F;
            p73.param2 = (float) -3.0970708E38F;
            p73.current = (byte)(byte)59;
            ADV_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (float)1.5418213E38F);
                Debug.Assert(pack.groundspeed == (float) -7.709023E37F);
                Debug.Assert(pack.throttle == (ushort)(ushort)60174);
                Debug.Assert(pack.heading == (short)(short)12175);
                Debug.Assert(pack.climb == (float) -2.025883E38F);
                Debug.Assert(pack.airspeed == (float)2.9847493E38F);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.throttle = (ushort)(ushort)60174;
            p74.climb = (float) -2.025883E38F;
            p74.heading = (short)(short)12175;
            p74.alt = (float)1.5418213E38F;
            p74.groundspeed = (float) -7.709023E37F;
            p74.airspeed = (float)2.9847493E38F;
            ADV_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param1 == (float)1.0513495E38F);
                Debug.Assert(pack.y == (int) -1948024514);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_NED);
                Debug.Assert(pack.current == (byte)(byte)81);
                Debug.Assert(pack.z == (float)2.2167124E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)177);
                Debug.Assert(pack.param2 == (float) -5.7116934E37F);
                Debug.Assert(pack.param4 == (float)3.1664235E38F);
                Debug.Assert(pack.x == (int)405633501);
                Debug.Assert(pack.target_system == (byte)(byte)130);
                Debug.Assert(pack.param3 == (float) -2.640834E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION);
                Debug.Assert(pack.target_component == (byte)(byte)142);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.target_system = (byte)(byte)130;
            p75.y = (int) -1948024514;
            p75.frame = MAV_FRAME.MAV_FRAME_BODY_NED;
            p75.param3 = (float) -2.640834E38F;
            p75.z = (float)2.2167124E38F;
            p75.autocontinue = (byte)(byte)177;
            p75.command = MAV_CMD.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
            p75.param4 = (float)3.1664235E38F;
            p75.x = (int)405633501;
            p75.current = (byte)(byte)81;
            p75.param1 = (float)1.0513495E38F;
            p75.target_component = (byte)(byte)142;
            p75.param2 = (float) -5.7116934E37F;
            ADV_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param6 == (float) -2.2413903E37F);
                Debug.Assert(pack.param1 == (float)1.5378653E38F);
                Debug.Assert(pack.target_system == (byte)(byte)220);
                Debug.Assert(pack.target_component == (byte)(byte)143);
                Debug.Assert(pack.param4 == (float) -2.0311016E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_DELAY);
                Debug.Assert(pack.param5 == (float)5.4314407E37F);
                Debug.Assert(pack.param2 == (float) -1.9569844E38F);
                Debug.Assert(pack.param3 == (float)2.596207E38F);
                Debug.Assert(pack.confirmation == (byte)(byte)112);
                Debug.Assert(pack.param7 == (float)2.5338228E38F);
            };
            COMMAND_LONG p76 = new COMMAND_LONG();
            PH.setPack(p76);
            p76.command = MAV_CMD.MAV_CMD_NAV_DELAY;
            p76.param5 = (float)5.4314407E37F;
            p76.target_component = (byte)(byte)143;
            p76.param1 = (float)1.5378653E38F;
            p76.confirmation = (byte)(byte)112;
            p76.param7 = (float)2.5338228E38F;
            p76.param2 = (float) -1.9569844E38F;
            p76.param3 = (float)2.596207E38F;
            p76.target_system = (byte)(byte)220;
            p76.param4 = (float) -2.0311016E38F;
            p76.param6 = (float) -2.2413903E37F;
            ADV_TEST_CH.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)30);
                Debug.Assert(pack.result_param2_TRY(ph) == (int)235661383);
                Debug.Assert(pack.result == MAV_RESULT.MAV_RESULT_FAILED);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)36);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_DELAY);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)225);
            };
            GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.command = MAV_CMD.MAV_CMD_NAV_DELAY;
            p77.result_param2_SET((int)235661383, PH) ;
            p77.target_component_SET((byte)(byte)225, PH) ;
            p77.progress_SET((byte)(byte)30, PH) ;
            p77.result = MAV_RESULT.MAV_RESULT_FAILED;
            p77.target_system_SET((byte)(byte)36, PH) ;
            CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -3.633118E37F);
                Debug.Assert(pack.mode_switch == (byte)(byte)195);
                Debug.Assert(pack.time_boot_ms == (uint)1826527084U);
                Debug.Assert(pack.roll == (float) -2.0324747E38F);
                Debug.Assert(pack.pitch == (float) -3.9418995E37F);
                Debug.Assert(pack.thrust == (float) -2.5171657E38F);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)133);
            };
            GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.pitch = (float) -3.9418995E37F;
            p81.thrust = (float) -2.5171657E38F;
            p81.manual_override_switch = (byte)(byte)133;
            p81.mode_switch = (byte)(byte)195;
            p81.roll = (float) -2.0324747E38F;
            p81.time_boot_ms = (uint)1826527084U;
            p81.yaw = (float) -3.633118E37F;
            CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)218);
                Debug.Assert(pack.target_system == (byte)(byte)179);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.7937944E38F, 1.2084313E38F, 9.855383E37F, 1.9482782E38F}));
                Debug.Assert(pack.time_boot_ms == (uint)1371172181U);
                Debug.Assert(pack.type_mask == (byte)(byte)133);
                Debug.Assert(pack.body_yaw_rate == (float)5.1124993E37F);
                Debug.Assert(pack.body_roll_rate == (float)3.9090012E37F);
                Debug.Assert(pack.body_pitch_rate == (float)4.763179E37F);
                Debug.Assert(pack.thrust == (float) -5.122587E36F);
            };
            GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.body_pitch_rate = (float)4.763179E37F;
            p82.target_component = (byte)(byte)218;
            p82.q_SET(new float[] {-2.7937944E38F, 1.2084313E38F, 9.855383E37F, 1.9482782E38F}, 0) ;
            p82.type_mask = (byte)(byte)133;
            p82.body_roll_rate = (float)3.9090012E37F;
            p82.target_system = (byte)(byte)179;
            p82.body_yaw_rate = (float)5.1124993E37F;
            p82.thrust = (float) -5.122587E36F;
            p82.time_boot_ms = (uint)1371172181U;
            CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2847800386U);
                Debug.Assert(pack.body_pitch_rate == (float) -1.8821086E38F);
                Debug.Assert(pack.body_roll_rate == (float)1.7219005E38F);
                Debug.Assert(pack.body_yaw_rate == (float)1.4427189E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)89);
                Debug.Assert(pack.thrust == (float) -5.1338825E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {7.0009143E37F, -1.4813914E38F, -2.5482686E38F, -3.2230112E38F}));
            };
            GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.thrust = (float) -5.1338825E37F;
            p83.time_boot_ms = (uint)2847800386U;
            p83.body_pitch_rate = (float) -1.8821086E38F;
            p83.body_roll_rate = (float)1.7219005E38F;
            p83.type_mask = (byte)(byte)89;
            p83.body_yaw_rate = (float)1.4427189E38F;
            p83.q_SET(new float[] {7.0009143E37F, -1.4813914E38F, -2.5482686E38F, -3.2230112E38F}, 0) ;
            CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw_rate == (float) -1.24042E37F);
                Debug.Assert(pack.vz == (float)3.0462512E38F);
                Debug.Assert(pack.vy == (float) -8.132068E37F);
                Debug.Assert(pack.target_component == (byte)(byte)99);
                Debug.Assert(pack.yaw == (float) -3.3326634E38F);
                Debug.Assert(pack.y == (float) -2.3655038E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.afy == (float)3.30989E38F);
                Debug.Assert(pack.time_boot_ms == (uint)741197867U);
                Debug.Assert(pack.target_system == (byte)(byte)67);
                Debug.Assert(pack.x == (float)9.178287E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)44118);
                Debug.Assert(pack.z == (float)1.9222487E38F);
                Debug.Assert(pack.afz == (float) -2.429385E38F);
                Debug.Assert(pack.vx == (float)2.9427764E38F);
                Debug.Assert(pack.afx == (float)1.7916815E38F);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.vy = (float) -8.132068E37F;
            p84.target_component = (byte)(byte)99;
            p84.type_mask = (ushort)(ushort)44118;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p84.afx = (float)1.7916815E38F;
            p84.afy = (float)3.30989E38F;
            p84.yaw_rate = (float) -1.24042E37F;
            p84.yaw = (float) -3.3326634E38F;
            p84.time_boot_ms = (uint)741197867U;
            p84.afz = (float) -2.429385E38F;
            p84.x = (float)9.178287E37F;
            p84.vz = (float)3.0462512E38F;
            p84.target_system = (byte)(byte)67;
            p84.vx = (float)2.9427764E38F;
            p84.y = (float) -2.3655038E38F;
            p84.z = (float)1.9222487E38F;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw_rate == (float) -2.129773E38F);
                Debug.Assert(pack.lon_int == (int) -257801738);
                Debug.Assert(pack.alt == (float)9.235787E36F);
                Debug.Assert(pack.target_system == (byte)(byte)18);
                Debug.Assert(pack.type_mask == (ushort)(ushort)65215);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.afz == (float)1.4800351E38F);
                Debug.Assert(pack.target_component == (byte)(byte)79);
                Debug.Assert(pack.afx == (float)8.524019E37F);
                Debug.Assert(pack.yaw == (float) -2.9809307E38F);
                Debug.Assert(pack.afy == (float)1.0575675E38F);
                Debug.Assert(pack.lat_int == (int) -1577447160);
                Debug.Assert(pack.time_boot_ms == (uint)1937417346U);
                Debug.Assert(pack.vz == (float)1.892333E38F);
                Debug.Assert(pack.vy == (float) -2.6966638E38F);
                Debug.Assert(pack.vx == (float) -1.881339E38F);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.vz = (float)1.892333E38F;
            p86.vx = (float) -1.881339E38F;
            p86.afz = (float)1.4800351E38F;
            p86.target_component = (byte)(byte)79;
            p86.lat_int = (int) -1577447160;
            p86.time_boot_ms = (uint)1937417346U;
            p86.yaw_rate = (float) -2.129773E38F;
            p86.alt = (float)9.235787E36F;
            p86.vy = (float) -2.6966638E38F;
            p86.afx = (float)8.524019E37F;
            p86.yaw = (float) -2.9809307E38F;
            p86.lon_int = (int) -257801738;
            p86.target_system = (byte)(byte)18;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p86.afy = (float)1.0575675E38F;
            p86.type_mask = (ushort)(ushort)65215;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float)1.2125199E38F);
                Debug.Assert(pack.afy == (float)5.645162E37F);
                Debug.Assert(pack.lat_int == (int) -1712790926);
                Debug.Assert(pack.afz == (float) -6.3705634E37F);
                Debug.Assert(pack.alt == (float)1.248326E38F);
                Debug.Assert(pack.vz == (float) -1.1802798E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)57063);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.lon_int == (int) -2082702011);
                Debug.Assert(pack.yaw == (float) -2.5476832E38F);
                Debug.Assert(pack.yaw_rate == (float)1.937378E38F);
                Debug.Assert(pack.vy == (float) -6.5197755E37F);
                Debug.Assert(pack.afx == (float)3.3836692E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1794874889U);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.lat_int = (int) -1712790926;
            p87.vx = (float)1.2125199E38F;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p87.lon_int = (int) -2082702011;
            p87.yaw_rate = (float)1.937378E38F;
            p87.alt = (float)1.248326E38F;
            p87.yaw = (float) -2.5476832E38F;
            p87.vy = (float) -6.5197755E37F;
            p87.vz = (float) -1.1802798E38F;
            p87.type_mask = (ushort)(ushort)57063;
            p87.afy = (float)5.645162E37F;
            p87.afx = (float)3.3836692E38F;
            p87.time_boot_ms = (uint)1794874889U;
            p87.afz = (float) -6.3705634E37F;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -2.7258709E38F);
                Debug.Assert(pack.yaw == (float) -6.5570835E37F);
                Debug.Assert(pack.time_boot_ms == (uint)2881297740U);
                Debug.Assert(pack.roll == (float)1.2589546E38F);
                Debug.Assert(pack.y == (float) -1.0669503E38F);
                Debug.Assert(pack.pitch == (float)1.3480053E38F);
                Debug.Assert(pack.x == (float)1.862723E38F);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.x = (float)1.862723E38F;
            p89.time_boot_ms = (uint)2881297740U;
            p89.pitch = (float)1.3480053E38F;
            p89.y = (float) -1.0669503E38F;
            p89.z = (float) -2.7258709E38F;
            p89.yaw = (float) -6.5570835E37F;
            p89.roll = (float)1.2589546E38F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -1431284722);
                Debug.Assert(pack.zacc == (short)(short) -20563);
                Debug.Assert(pack.pitch == (float) -3.3706304E38F);
                Debug.Assert(pack.vx == (short)(short)20981);
                Debug.Assert(pack.yawspeed == (float) -2.6300132E38F);
                Debug.Assert(pack.vy == (short)(short) -31419);
                Debug.Assert(pack.pitchspeed == (float)2.3160632E38F);
                Debug.Assert(pack.roll == (float) -9.758386E37F);
                Debug.Assert(pack.alt == (int) -1880729953);
                Debug.Assert(pack.rollspeed == (float)4.717466E37F);
                Debug.Assert(pack.yacc == (short)(short) -24450);
                Debug.Assert(pack.vz == (short)(short)28458);
                Debug.Assert(pack.lat == (int)457642845);
                Debug.Assert(pack.time_usec == (ulong)2391263760240289489L);
                Debug.Assert(pack.yaw == (float) -2.7887062E38F);
                Debug.Assert(pack.xacc == (short)(short)20350);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.vy = (short)(short) -31419;
            p90.yaw = (float) -2.7887062E38F;
            p90.alt = (int) -1880729953;
            p90.pitchspeed = (float)2.3160632E38F;
            p90.lon = (int) -1431284722;
            p90.pitch = (float) -3.3706304E38F;
            p90.time_usec = (ulong)2391263760240289489L;
            p90.lat = (int)457642845;
            p90.vx = (short)(short)20981;
            p90.yawspeed = (float) -2.6300132E38F;
            p90.zacc = (short)(short) -20563;
            p90.yacc = (short)(short) -24450;
            p90.xacc = (short)(short)20350;
            p90.roll = (float) -9.758386E37F;
            p90.vz = (short)(short)28458;
            p90.rollspeed = (float)4.717466E37F;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aux4 == (float)9.230367E37F);
                Debug.Assert(pack.aux3 == (float) -1.943056E38F);
                Debug.Assert(pack.aux2 == (float) -3.1587547E38F);
                Debug.Assert(pack.time_usec == (ulong)841308636823495239L);
                Debug.Assert(pack.yaw_rudder == (float)4.986777E36F);
                Debug.Assert(pack.throttle == (float)3.1174816E38F);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_PREFLIGHT);
                Debug.Assert(pack.nav_mode == (byte)(byte)165);
                Debug.Assert(pack.pitch_elevator == (float)1.1702644E38F);
                Debug.Assert(pack.aux1 == (float) -2.938347E38F);
                Debug.Assert(pack.roll_ailerons == (float) -9.181048E37F);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.roll_ailerons = (float) -9.181048E37F;
            p91.aux3 = (float) -1.943056E38F;
            p91.mode = MAV_MODE.MAV_MODE_PREFLIGHT;
            p91.aux2 = (float) -3.1587547E38F;
            p91.aux1 = (float) -2.938347E38F;
            p91.pitch_elevator = (float)1.1702644E38F;
            p91.throttle = (float)3.1174816E38F;
            p91.aux4 = (float)9.230367E37F;
            p91.yaw_rudder = (float)4.986777E36F;
            p91.time_usec = (ulong)841308636823495239L;
            p91.nav_mode = (byte)(byte)165;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)7218550241149047564L);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)20083);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)21248);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)41895);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)31124);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)63361);
                Debug.Assert(pack.rssi == (byte)(byte)75);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)48317);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)62637);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)45442);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)60719);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)2205);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)30739);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)6908);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan9_raw = (ushort)(ushort)62637;
            p92.chan10_raw = (ushort)(ushort)48317;
            p92.chan1_raw = (ushort)(ushort)6908;
            p92.chan6_raw = (ushort)(ushort)21248;
            p92.chan12_raw = (ushort)(ushort)45442;
            p92.chan11_raw = (ushort)(ushort)63361;
            p92.time_usec = (ulong)7218550241149047564L;
            p92.chan7_raw = (ushort)(ushort)41895;
            p92.chan8_raw = (ushort)(ushort)31124;
            p92.chan4_raw = (ushort)(ushort)30739;
            p92.rssi = (byte)(byte)75;
            p92.chan2_raw = (ushort)(ushort)60719;
            p92.chan3_raw = (ushort)(ushort)2205;
            p92.chan5_raw = (ushort)(ushort)20083;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)920244806035190608L);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_TEST_ARMED);
                Debug.Assert(pack.flags == (ulong)2474599255701574337L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.7832327E38F, -2.3270007E38F, -1.8738736E38F, 2.1275544E38F, -3.393218E38F, -3.119845E38F, -1.6870164E38F, 3.1362633E38F, 2.215545E38F, -4.6787356E37F, 2.1546774E38F, 2.45792E38F, -3.045427E38F, 4.0454873E37F, -2.8091435E38F, 4.9623187E37F}));
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.controls_SET(new float[] {-1.7832327E38F, -2.3270007E38F, -1.8738736E38F, 2.1275544E38F, -3.393218E38F, -3.119845E38F, -1.6870164E38F, 3.1362633E38F, 2.215545E38F, -4.6787356E37F, 2.1546774E38F, 2.45792E38F, -3.045427E38F, 4.0454873E37F, -2.8091435E38F, 4.9623187E37F}, 0) ;
            p93.flags = (ulong)2474599255701574337L;
            p93.mode = MAV_MODE.MAV_MODE_TEST_ARMED;
            p93.time_usec = (ulong)920244806035190608L;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_y == (short)(short)10008);
                Debug.Assert(pack.ground_distance == (float)1.4625003E38F);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float) -3.2561873E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)137);
                Debug.Assert(pack.flow_x == (short)(short)14035);
                Debug.Assert(pack.quality == (byte)(byte)18);
                Debug.Assert(pack.flow_comp_m_x == (float)2.6225427E38F);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float) -1.3767631E38F);
                Debug.Assert(pack.flow_comp_m_y == (float)3.1227498E38F);
                Debug.Assert(pack.time_usec == (ulong)4853474123326474827L);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.quality = (byte)(byte)18;
            p100.flow_rate_x_SET((float) -1.3767631E38F, PH) ;
            p100.flow_comp_m_y = (float)3.1227498E38F;
            p100.time_usec = (ulong)4853474123326474827L;
            p100.flow_comp_m_x = (float)2.6225427E38F;
            p100.ground_distance = (float)1.4625003E38F;
            p100.flow_y = (short)(short)10008;
            p100.flow_rate_y_SET((float) -3.2561873E38F, PH) ;
            p100.flow_x = (short)(short)14035;
            p100.sensor_id = (byte)(byte)137;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)2.8095295E38F);
                Debug.Assert(pack.yaw == (float)1.9415807E38F);
                Debug.Assert(pack.x == (float) -1.0985851E38F);
                Debug.Assert(pack.roll == (float) -3.3388E37F);
                Debug.Assert(pack.usec == (ulong)1826829392949447860L);
                Debug.Assert(pack.z == (float) -2.0701381E38F);
                Debug.Assert(pack.pitch == (float)2.9957732E38F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.z = (float) -2.0701381E38F;
            p101.x = (float) -1.0985851E38F;
            p101.pitch = (float)2.9957732E38F;
            p101.roll = (float) -3.3388E37F;
            p101.y = (float)2.8095295E38F;
            p101.usec = (ulong)1826829392949447860L;
            p101.yaw = (float)1.9415807E38F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float)8.3719073E37F);
                Debug.Assert(pack.roll == (float)3.0399401E38F);
                Debug.Assert(pack.y == (float)4.830772E37F);
                Debug.Assert(pack.x == (float) -2.3959937E38F);
                Debug.Assert(pack.yaw == (float) -2.8764726E38F);
                Debug.Assert(pack.z == (float) -3.2384745E38F);
                Debug.Assert(pack.usec == (ulong)6225701883200705226L);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.y = (float)4.830772E37F;
            p102.yaw = (float) -2.8764726E38F;
            p102.x = (float) -2.3959937E38F;
            p102.pitch = (float)8.3719073E37F;
            p102.z = (float) -3.2384745E38F;
            p102.roll = (float)3.0399401E38F;
            p102.usec = (ulong)6225701883200705226L;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)1.909542E38F);
                Debug.Assert(pack.x == (float)2.1548138E37F);
                Debug.Assert(pack.usec == (ulong)4549347872179800500L);
                Debug.Assert(pack.z == (float) -1.8810686E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.z = (float) -1.8810686E38F;
            p103.x = (float)2.1548138E37F;
            p103.usec = (ulong)4549347872179800500L;
            p103.y = (float)1.909542E38F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)2.3749882E38F);
                Debug.Assert(pack.roll == (float) -1.8407672E38F);
                Debug.Assert(pack.yaw == (float)3.2766594E38F);
                Debug.Assert(pack.usec == (ulong)1640841526201101820L);
                Debug.Assert(pack.z == (float)1.4712869E38F);
                Debug.Assert(pack.pitch == (float)6.25658E37F);
                Debug.Assert(pack.y == (float) -1.2808837E38F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.z = (float)1.4712869E38F;
            p104.x = (float)2.3749882E38F;
            p104.y = (float) -1.2808837E38F;
            p104.usec = (ulong)1640841526201101820L;
            p104.roll = (float) -1.8407672E38F;
            p104.yaw = (float)3.2766594E38F;
            p104.pitch = (float)6.25658E37F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (float)1.7046434E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)19372);
                Debug.Assert(pack.yacc == (float) -1.9645553E37F);
                Debug.Assert(pack.zmag == (float) -2.2077046E38F);
                Debug.Assert(pack.ygyro == (float)8.669853E37F);
                Debug.Assert(pack.xmag == (float) -3.2428871E38F);
                Debug.Assert(pack.zacc == (float)2.217961E38F);
                Debug.Assert(pack.zgyro == (float) -2.2356955E37F);
                Debug.Assert(pack.ymag == (float)2.3973849E38F);
                Debug.Assert(pack.pressure_alt == (float) -3.262901E38F);
                Debug.Assert(pack.time_usec == (ulong)1973712386790891946L);
                Debug.Assert(pack.xgyro == (float) -1.140498E38F);
                Debug.Assert(pack.diff_pressure == (float) -1.2281339E38F);
                Debug.Assert(pack.abs_pressure == (float) -3.1616537E36F);
                Debug.Assert(pack.temperature == (float) -5.646481E37F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.ygyro = (float)8.669853E37F;
            p105.xgyro = (float) -1.140498E38F;
            p105.pressure_alt = (float) -3.262901E38F;
            p105.ymag = (float)2.3973849E38F;
            p105.time_usec = (ulong)1973712386790891946L;
            p105.yacc = (float) -1.9645553E37F;
            p105.zmag = (float) -2.2077046E38F;
            p105.zacc = (float)2.217961E38F;
            p105.xacc = (float)1.7046434E38F;
            p105.diff_pressure = (float) -1.2281339E38F;
            p105.abs_pressure = (float) -3.1616537E36F;
            p105.fields_updated = (ushort)(ushort)19372;
            p105.xmag = (float) -3.2428871E38F;
            p105.zgyro = (float) -2.2356955E37F;
            p105.temperature = (float) -5.646481E37F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_y == (float) -1.6961779E38F);
                Debug.Assert(pack.quality == (byte)(byte)119);
                Debug.Assert(pack.distance == (float)3.3422156E38F);
                Debug.Assert(pack.integrated_x == (float)3.3019361E38F);
                Debug.Assert(pack.integrated_ygyro == (float)2.7077873E38F);
                Debug.Assert(pack.integration_time_us == (uint)1178433936U);
                Debug.Assert(pack.time_usec == (ulong)8341161530957655628L);
                Debug.Assert(pack.integrated_xgyro == (float)7.91749E37F);
                Debug.Assert(pack.time_delta_distance_us == (uint)1689135807U);
                Debug.Assert(pack.temperature == (short)(short) -28092);
                Debug.Assert(pack.sensor_id == (byte)(byte)147);
                Debug.Assert(pack.integrated_zgyro == (float)1.096674E38F);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.distance = (float)3.3422156E38F;
            p106.integrated_xgyro = (float)7.91749E37F;
            p106.integrated_ygyro = (float)2.7077873E38F;
            p106.quality = (byte)(byte)119;
            p106.integration_time_us = (uint)1178433936U;
            p106.time_usec = (ulong)8341161530957655628L;
            p106.integrated_y = (float) -1.6961779E38F;
            p106.integrated_x = (float)3.3019361E38F;
            p106.time_delta_distance_us = (uint)1689135807U;
            p106.temperature = (short)(short) -28092;
            p106.sensor_id = (byte)(byte)147;
            p106.integrated_zgyro = (float)1.096674E38F;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pressure_alt == (float)1.599319E37F);
                Debug.Assert(pack.yacc == (float) -5.8123834E37F);
                Debug.Assert(pack.xacc == (float) -8.822263E37F);
                Debug.Assert(pack.ygyro == (float)6.584713E37F);
                Debug.Assert(pack.xgyro == (float)2.8576185E38F);
                Debug.Assert(pack.abs_pressure == (float) -2.4519383E38F);
                Debug.Assert(pack.xmag == (float)1.1011195E38F);
                Debug.Assert(pack.zmag == (float)1.0472723E38F);
                Debug.Assert(pack.temperature == (float)5.2095095E37F);
                Debug.Assert(pack.zacc == (float)3.272089E38F);
                Debug.Assert(pack.ymag == (float) -8.542765E37F);
                Debug.Assert(pack.fields_updated == (uint)4131352010U);
                Debug.Assert(pack.diff_pressure == (float)9.71781E37F);
                Debug.Assert(pack.zgyro == (float)1.8389805E38F);
                Debug.Assert(pack.time_usec == (ulong)3671829007039185374L);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.zgyro = (float)1.8389805E38F;
            p107.ymag = (float) -8.542765E37F;
            p107.pressure_alt = (float)1.599319E37F;
            p107.fields_updated = (uint)4131352010U;
            p107.ygyro = (float)6.584713E37F;
            p107.diff_pressure = (float)9.71781E37F;
            p107.yacc = (float) -5.8123834E37F;
            p107.time_usec = (ulong)3671829007039185374L;
            p107.zmag = (float)1.0472723E38F;
            p107.xmag = (float)1.1011195E38F;
            p107.xacc = (float) -8.822263E37F;
            p107.xgyro = (float)2.8576185E38F;
            p107.temperature = (float)5.2095095E37F;
            p107.zacc = (float)3.272089E38F;
            p107.abs_pressure = (float) -2.4519383E38F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vn == (float) -2.4066992E38F);
                Debug.Assert(pack.xacc == (float) -5.5115465E37F);
                Debug.Assert(pack.lon == (float)9.392408E37F);
                Debug.Assert(pack.std_dev_vert == (float) -2.2419155E38F);
                Debug.Assert(pack.lat == (float) -2.6484117E38F);
                Debug.Assert(pack.zacc == (float)7.486265E37F);
                Debug.Assert(pack.vd == (float)1.0221831E38F);
                Debug.Assert(pack.q3 == (float) -1.7711304E38F);
                Debug.Assert(pack.std_dev_horz == (float)2.976615E37F);
                Debug.Assert(pack.ve == (float) -1.350197E38F);
                Debug.Assert(pack.q2 == (float)2.4275825E38F);
                Debug.Assert(pack.pitch == (float)3.2123142E38F);
                Debug.Assert(pack.ygyro == (float)3.0406174E38F);
                Debug.Assert(pack.q4 == (float) -1.5423051E38F);
                Debug.Assert(pack.roll == (float)6.7375665E37F);
                Debug.Assert(pack.xgyro == (float)1.3526151E37F);
                Debug.Assert(pack.alt == (float) -1.815105E37F);
                Debug.Assert(pack.yacc == (float) -3.0548102E38F);
                Debug.Assert(pack.zgyro == (float)9.985624E37F);
                Debug.Assert(pack.q1 == (float)2.253103E37F);
                Debug.Assert(pack.yaw == (float)1.8647743E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.lon = (float)9.392408E37F;
            p108.lat = (float) -2.6484117E38F;
            p108.yaw = (float)1.8647743E38F;
            p108.pitch = (float)3.2123142E38F;
            p108.std_dev_horz = (float)2.976615E37F;
            p108.zgyro = (float)9.985624E37F;
            p108.std_dev_vert = (float) -2.2419155E38F;
            p108.q3 = (float) -1.7711304E38F;
            p108.alt = (float) -1.815105E37F;
            p108.q2 = (float)2.4275825E38F;
            p108.q1 = (float)2.253103E37F;
            p108.yacc = (float) -3.0548102E38F;
            p108.roll = (float)6.7375665E37F;
            p108.xgyro = (float)1.3526151E37F;
            p108.xacc = (float) -5.5115465E37F;
            p108.zacc = (float)7.486265E37F;
            p108.q4 = (float) -1.5423051E38F;
            p108.ygyro = (float)3.0406174E38F;
            p108.vd = (float)1.0221831E38F;
            p108.vn = (float) -2.4066992E38F;
            p108.ve = (float) -1.350197E38F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.remrssi == (byte)(byte)118);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)56695);
                Debug.Assert(pack.txbuf == (byte)(byte)156);
                Debug.Assert(pack.rssi == (byte)(byte)241);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)56203);
                Debug.Assert(pack.remnoise == (byte)(byte)243);
                Debug.Assert(pack.noise == (byte)(byte)232);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.remrssi = (byte)(byte)118;
            p109.txbuf = (byte)(byte)156;
            p109.fixed_ = (ushort)(ushort)56203;
            p109.rssi = (byte)(byte)241;
            p109.remnoise = (byte)(byte)243;
            p109.noise = (byte)(byte)232;
            p109.rxerrors = (ushort)(ushort)56695;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)118);
                Debug.Assert(pack.target_network == (byte)(byte)109);
                Debug.Assert(pack.target_system == (byte)(byte)146);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)79, (byte)38, (byte)226, (byte)36, (byte)146, (byte)170, (byte)7, (byte)227, (byte)98, (byte)211, (byte)48, (byte)87, (byte)220, (byte)19, (byte)188, (byte)113, (byte)12, (byte)35, (byte)187, (byte)245, (byte)173, (byte)121, (byte)145, (byte)40, (byte)206, (byte)172, (byte)85, (byte)171, (byte)16, (byte)212, (byte)166, (byte)93, (byte)132, (byte)237, (byte)117, (byte)68, (byte)129, (byte)33, (byte)173, (byte)119, (byte)223, (byte)91, (byte)113, (byte)41, (byte)126, (byte)165, (byte)207, (byte)25, (byte)103, (byte)38, (byte)171, (byte)236, (byte)150, (byte)229, (byte)176, (byte)30, (byte)238, (byte)123, (byte)62, (byte)255, (byte)166, (byte)255, (byte)46, (byte)45, (byte)217, (byte)54, (byte)104, (byte)30, (byte)61, (byte)39, (byte)133, (byte)173, (byte)252, (byte)7, (byte)205, (byte)57, (byte)104, (byte)169, (byte)189, (byte)155, (byte)40, (byte)192, (byte)187, (byte)61, (byte)94, (byte)140, (byte)4, (byte)79, (byte)194, (byte)149, (byte)6, (byte)239, (byte)77, (byte)19, (byte)155, (byte)167, (byte)58, (byte)216, (byte)20, (byte)125, (byte)183, (byte)136, (byte)1, (byte)95, (byte)95, (byte)3, (byte)203, (byte)80, (byte)101, (byte)189, (byte)137, (byte)120, (byte)99, (byte)33, (byte)49, (byte)98, (byte)75, (byte)237, (byte)86, (byte)158, (byte)213, (byte)5, (byte)254, (byte)54, (byte)227, (byte)223, (byte)184, (byte)251, (byte)6, (byte)78, (byte)246, (byte)205, (byte)172, (byte)113, (byte)31, (byte)226, (byte)5, (byte)240, (byte)27, (byte)232, (byte)230, (byte)214, (byte)1, (byte)250, (byte)76, (byte)119, (byte)147, (byte)46, (byte)42, (byte)236, (byte)166, (byte)82, (byte)238, (byte)7, (byte)54, (byte)195, (byte)22, (byte)218, (byte)213, (byte)132, (byte)58, (byte)114, (byte)238, (byte)49, (byte)110, (byte)231, (byte)185, (byte)238, (byte)105, (byte)47, (byte)210, (byte)31, (byte)230, (byte)182, (byte)122, (byte)169, (byte)153, (byte)83, (byte)176, (byte)69, (byte)42, (byte)46, (byte)231, (byte)248, (byte)174, (byte)101, (byte)194, (byte)22, (byte)92, (byte)75, (byte)158, (byte)13, (byte)80, (byte)163, (byte)121, (byte)201, (byte)149, (byte)37, (byte)132, (byte)73, (byte)54, (byte)9, (byte)213, (byte)69, (byte)253, (byte)144, (byte)138, (byte)217, (byte)25, (byte)130, (byte)144, (byte)76, (byte)237, (byte)1, (byte)97, (byte)245, (byte)107, (byte)105, (byte)174, (byte)54, (byte)207, (byte)245, (byte)24, (byte)176, (byte)61, (byte)50, (byte)160, (byte)50, (byte)141, (byte)175, (byte)203, (byte)219, (byte)204, (byte)0, (byte)88, (byte)12, (byte)80, (byte)177, (byte)54, (byte)252, (byte)222, (byte)187, (byte)238, (byte)176, (byte)110, (byte)91, (byte)3, (byte)89, (byte)226, (byte)103, (byte)98}));
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.payload_SET(new byte[] {(byte)79, (byte)38, (byte)226, (byte)36, (byte)146, (byte)170, (byte)7, (byte)227, (byte)98, (byte)211, (byte)48, (byte)87, (byte)220, (byte)19, (byte)188, (byte)113, (byte)12, (byte)35, (byte)187, (byte)245, (byte)173, (byte)121, (byte)145, (byte)40, (byte)206, (byte)172, (byte)85, (byte)171, (byte)16, (byte)212, (byte)166, (byte)93, (byte)132, (byte)237, (byte)117, (byte)68, (byte)129, (byte)33, (byte)173, (byte)119, (byte)223, (byte)91, (byte)113, (byte)41, (byte)126, (byte)165, (byte)207, (byte)25, (byte)103, (byte)38, (byte)171, (byte)236, (byte)150, (byte)229, (byte)176, (byte)30, (byte)238, (byte)123, (byte)62, (byte)255, (byte)166, (byte)255, (byte)46, (byte)45, (byte)217, (byte)54, (byte)104, (byte)30, (byte)61, (byte)39, (byte)133, (byte)173, (byte)252, (byte)7, (byte)205, (byte)57, (byte)104, (byte)169, (byte)189, (byte)155, (byte)40, (byte)192, (byte)187, (byte)61, (byte)94, (byte)140, (byte)4, (byte)79, (byte)194, (byte)149, (byte)6, (byte)239, (byte)77, (byte)19, (byte)155, (byte)167, (byte)58, (byte)216, (byte)20, (byte)125, (byte)183, (byte)136, (byte)1, (byte)95, (byte)95, (byte)3, (byte)203, (byte)80, (byte)101, (byte)189, (byte)137, (byte)120, (byte)99, (byte)33, (byte)49, (byte)98, (byte)75, (byte)237, (byte)86, (byte)158, (byte)213, (byte)5, (byte)254, (byte)54, (byte)227, (byte)223, (byte)184, (byte)251, (byte)6, (byte)78, (byte)246, (byte)205, (byte)172, (byte)113, (byte)31, (byte)226, (byte)5, (byte)240, (byte)27, (byte)232, (byte)230, (byte)214, (byte)1, (byte)250, (byte)76, (byte)119, (byte)147, (byte)46, (byte)42, (byte)236, (byte)166, (byte)82, (byte)238, (byte)7, (byte)54, (byte)195, (byte)22, (byte)218, (byte)213, (byte)132, (byte)58, (byte)114, (byte)238, (byte)49, (byte)110, (byte)231, (byte)185, (byte)238, (byte)105, (byte)47, (byte)210, (byte)31, (byte)230, (byte)182, (byte)122, (byte)169, (byte)153, (byte)83, (byte)176, (byte)69, (byte)42, (byte)46, (byte)231, (byte)248, (byte)174, (byte)101, (byte)194, (byte)22, (byte)92, (byte)75, (byte)158, (byte)13, (byte)80, (byte)163, (byte)121, (byte)201, (byte)149, (byte)37, (byte)132, (byte)73, (byte)54, (byte)9, (byte)213, (byte)69, (byte)253, (byte)144, (byte)138, (byte)217, (byte)25, (byte)130, (byte)144, (byte)76, (byte)237, (byte)1, (byte)97, (byte)245, (byte)107, (byte)105, (byte)174, (byte)54, (byte)207, (byte)245, (byte)24, (byte)176, (byte)61, (byte)50, (byte)160, (byte)50, (byte)141, (byte)175, (byte)203, (byte)219, (byte)204, (byte)0, (byte)88, (byte)12, (byte)80, (byte)177, (byte)54, (byte)252, (byte)222, (byte)187, (byte)238, (byte)176, (byte)110, (byte)91, (byte)3, (byte)89, (byte)226, (byte)103, (byte)98}, 0) ;
            p110.target_component = (byte)(byte)118;
            p110.target_network = (byte)(byte)109;
            p110.target_system = (byte)(byte)146;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tc1 == (long) -8627237657312746414L);
                Debug.Assert(pack.ts1 == (long) -8850281226321870935L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long) -8850281226321870935L;
            p111.tc1 = (long) -8627237657312746414L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3574524183584992960L);
                Debug.Assert(pack.seq == (uint)519267286U);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.seq = (uint)519267286U;
            p112.time_usec = (ulong)3574524183584992960L;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ve == (short)(short)27986);
                Debug.Assert(pack.eph == (ushort)(ushort)27069);
                Debug.Assert(pack.alt == (int)1496037753);
                Debug.Assert(pack.vel == (ushort)(ushort)9008);
                Debug.Assert(pack.time_usec == (ulong)6486284063940449629L);
                Debug.Assert(pack.lon == (int)1983951555);
                Debug.Assert(pack.satellites_visible == (byte)(byte)231);
                Debug.Assert(pack.epv == (ushort)(ushort)18683);
                Debug.Assert(pack.cog == (ushort)(ushort)46669);
                Debug.Assert(pack.lat == (int) -2036570024);
                Debug.Assert(pack.vd == (short)(short) -32528);
                Debug.Assert(pack.fix_type == (byte)(byte)91);
                Debug.Assert(pack.vn == (short)(short)11515);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.cog = (ushort)(ushort)46669;
            p113.ve = (short)(short)27986;
            p113.alt = (int)1496037753;
            p113.time_usec = (ulong)6486284063940449629L;
            p113.eph = (ushort)(ushort)27069;
            p113.vd = (short)(short) -32528;
            p113.fix_type = (byte)(byte)91;
            p113.lon = (int)1983951555;
            p113.vel = (ushort)(ushort)9008;
            p113.satellites_visible = (byte)(byte)231;
            p113.lat = (int) -2036570024;
            p113.epv = (ushort)(ushort)18683;
            p113.vn = (short)(short)11515;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_delta_distance_us == (uint)646067803U);
                Debug.Assert(pack.integrated_x == (float) -1.5060616E38F);
                Debug.Assert(pack.integrated_zgyro == (float)1.3872468E38F);
                Debug.Assert(pack.integrated_xgyro == (float) -1.818487E38F);
                Debug.Assert(pack.integration_time_us == (uint)1126811378U);
                Debug.Assert(pack.sensor_id == (byte)(byte)205);
                Debug.Assert(pack.time_usec == (ulong)5610800861500076564L);
                Debug.Assert(pack.temperature == (short)(short)6040);
                Debug.Assert(pack.integrated_ygyro == (float)5.086842E37F);
                Debug.Assert(pack.quality == (byte)(byte)181);
                Debug.Assert(pack.distance == (float)1.5690978E37F);
                Debug.Assert(pack.integrated_y == (float) -3.2009222E38F);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.distance = (float)1.5690978E37F;
            p114.integrated_y = (float) -3.2009222E38F;
            p114.integrated_xgyro = (float) -1.818487E38F;
            p114.time_usec = (ulong)5610800861500076564L;
            p114.time_delta_distance_us = (uint)646067803U;
            p114.temperature = (short)(short)6040;
            p114.integration_time_us = (uint)1126811378U;
            p114.integrated_zgyro = (float)1.3872468E38F;
            p114.quality = (byte)(byte)181;
            p114.integrated_x = (float) -1.5060616E38F;
            p114.integrated_ygyro = (float)5.086842E37F;
            p114.sensor_id = (byte)(byte)205;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitchspeed == (float) -8.965522E37F);
                Debug.Assert(pack.lon == (int) -997326994);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {1.9663944E38F, 1.1096279E38F, -5.000646E37F, -1.848359E38F}));
                Debug.Assert(pack.vx == (short)(short)19965);
                Debug.Assert(pack.vy == (short)(short) -9534);
                Debug.Assert(pack.lat == (int)1008142199);
                Debug.Assert(pack.vz == (short)(short) -12680);
                Debug.Assert(pack.xacc == (short)(short) -27703);
                Debug.Assert(pack.rollspeed == (float)2.74285E37F);
                Debug.Assert(pack.yacc == (short)(short)21283);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)13399);
                Debug.Assert(pack.yawspeed == (float)1.3528565E38F);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)53735);
                Debug.Assert(pack.time_usec == (ulong)5315830984479881601L);
                Debug.Assert(pack.alt == (int)255089648);
                Debug.Assert(pack.zacc == (short)(short)7730);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)5315830984479881601L;
            p115.pitchspeed = (float) -8.965522E37F;
            p115.alt = (int)255089648;
            p115.yacc = (short)(short)21283;
            p115.true_airspeed = (ushort)(ushort)53735;
            p115.vz = (short)(short) -12680;
            p115.vy = (short)(short) -9534;
            p115.lat = (int)1008142199;
            p115.yawspeed = (float)1.3528565E38F;
            p115.vx = (short)(short)19965;
            p115.xacc = (short)(short) -27703;
            p115.rollspeed = (float)2.74285E37F;
            p115.attitude_quaternion_SET(new float[] {1.9663944E38F, 1.1096279E38F, -5.000646E37F, -1.848359E38F}, 0) ;
            p115.ind_airspeed = (ushort)(ushort)13399;
            p115.lon = (int) -997326994;
            p115.zacc = (short)(short)7730;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (short)(short)19394);
                Debug.Assert(pack.xacc == (short)(short) -26916);
                Debug.Assert(pack.xmag == (short)(short) -7397);
                Debug.Assert(pack.ymag == (short)(short) -4248);
                Debug.Assert(pack.zmag == (short)(short)25835);
                Debug.Assert(pack.time_boot_ms == (uint)3677282047U);
                Debug.Assert(pack.xgyro == (short)(short)1217);
                Debug.Assert(pack.zgyro == (short)(short)77);
                Debug.Assert(pack.yacc == (short)(short) -19498);
                Debug.Assert(pack.ygyro == (short)(short) -29992);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.zgyro = (short)(short)77;
            p116.zacc = (short)(short)19394;
            p116.ymag = (short)(short) -4248;
            p116.time_boot_ms = (uint)3677282047U;
            p116.ygyro = (short)(short) -29992;
            p116.zmag = (short)(short)25835;
            p116.xacc = (short)(short) -26916;
            p116.xgyro = (short)(short)1217;
            p116.yacc = (short)(short) -19498;
            p116.xmag = (short)(short) -7397;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start == (ushort)(ushort)55338);
                Debug.Assert(pack.target_component == (byte)(byte)102);
                Debug.Assert(pack.end == (ushort)(ushort)35106);
                Debug.Assert(pack.target_system == (byte)(byte)104);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.end = (ushort)(ushort)35106;
            p117.target_component = (byte)(byte)102;
            p117.start = (ushort)(ushort)55338;
            p117.target_system = (byte)(byte)104;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_log_num == (ushort)(ushort)61372);
                Debug.Assert(pack.id == (ushort)(ushort)56991);
                Debug.Assert(pack.time_utc == (uint)4116317274U);
                Debug.Assert(pack.num_logs == (ushort)(ushort)51692);
                Debug.Assert(pack.size == (uint)3413667205U);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.time_utc = (uint)4116317274U;
            p118.size = (uint)3413667205U;
            p118.last_log_num = (ushort)(ushort)61372;
            p118.id = (ushort)(ushort)56991;
            p118.num_logs = (ushort)(ushort)51692;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (uint)3036332686U);
                Debug.Assert(pack.target_component == (byte)(byte)88);
                Debug.Assert(pack.ofs == (uint)1071251611U);
                Debug.Assert(pack.id == (ushort)(ushort)7998);
                Debug.Assert(pack.target_system == (byte)(byte)137);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)137;
            p119.ofs = (uint)1071251611U;
            p119.target_component = (byte)(byte)88;
            p119.id = (ushort)(ushort)7998;
            p119.count = (uint)3036332686U;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)53548);
                Debug.Assert(pack.ofs == (uint)3775633950U);
                Debug.Assert(pack.count == (byte)(byte)131);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)34, (byte)245, (byte)118, (byte)88, (byte)107, (byte)204, (byte)134, (byte)180, (byte)41, (byte)62, (byte)189, (byte)112, (byte)241, (byte)235, (byte)76, (byte)89, (byte)208, (byte)47, (byte)221, (byte)55, (byte)31, (byte)7, (byte)191, (byte)189, (byte)23, (byte)181, (byte)113, (byte)139, (byte)237, (byte)169, (byte)243, (byte)62, (byte)26, (byte)55, (byte)186, (byte)79, (byte)170, (byte)75, (byte)101, (byte)182, (byte)63, (byte)163, (byte)230, (byte)196, (byte)93, (byte)224, (byte)234, (byte)218, (byte)144, (byte)197, (byte)130, (byte)219, (byte)171, (byte)50, (byte)230, (byte)179, (byte)183, (byte)76, (byte)8, (byte)121, (byte)140, (byte)67, (byte)35, (byte)21, (byte)146, (byte)63, (byte)154, (byte)182, (byte)87, (byte)146, (byte)220, (byte)22, (byte)138, (byte)149, (byte)227, (byte)192, (byte)180, (byte)188, (byte)37, (byte)149, (byte)236, (byte)66, (byte)244, (byte)69, (byte)234, (byte)73, (byte)212, (byte)121, (byte)45, (byte)176}));
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.data__SET(new byte[] {(byte)34, (byte)245, (byte)118, (byte)88, (byte)107, (byte)204, (byte)134, (byte)180, (byte)41, (byte)62, (byte)189, (byte)112, (byte)241, (byte)235, (byte)76, (byte)89, (byte)208, (byte)47, (byte)221, (byte)55, (byte)31, (byte)7, (byte)191, (byte)189, (byte)23, (byte)181, (byte)113, (byte)139, (byte)237, (byte)169, (byte)243, (byte)62, (byte)26, (byte)55, (byte)186, (byte)79, (byte)170, (byte)75, (byte)101, (byte)182, (byte)63, (byte)163, (byte)230, (byte)196, (byte)93, (byte)224, (byte)234, (byte)218, (byte)144, (byte)197, (byte)130, (byte)219, (byte)171, (byte)50, (byte)230, (byte)179, (byte)183, (byte)76, (byte)8, (byte)121, (byte)140, (byte)67, (byte)35, (byte)21, (byte)146, (byte)63, (byte)154, (byte)182, (byte)87, (byte)146, (byte)220, (byte)22, (byte)138, (byte)149, (byte)227, (byte)192, (byte)180, (byte)188, (byte)37, (byte)149, (byte)236, (byte)66, (byte)244, (byte)69, (byte)234, (byte)73, (byte)212, (byte)121, (byte)45, (byte)176}, 0) ;
            p120.count = (byte)(byte)131;
            p120.id = (ushort)(ushort)53548;
            p120.ofs = (uint)3775633950U;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)40);
                Debug.Assert(pack.target_system == (byte)(byte)195);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_component = (byte)(byte)40;
            p121.target_system = (byte)(byte)195;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)146);
                Debug.Assert(pack.target_component == (byte)(byte)218);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_component = (byte)(byte)218;
            p122.target_system = (byte)(byte)146;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)249);
                Debug.Assert(pack.len == (byte)(byte)169);
                Debug.Assert(pack.target_component == (byte)(byte)220);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)117, (byte)150, (byte)237, (byte)116, (byte)200, (byte)36, (byte)237, (byte)150, (byte)182, (byte)63, (byte)233, (byte)160, (byte)55, (byte)177, (byte)136, (byte)248, (byte)219, (byte)241, (byte)98, (byte)158, (byte)88, (byte)207, (byte)42, (byte)63, (byte)124, (byte)94, (byte)69, (byte)51, (byte)236, (byte)241, (byte)99, (byte)84, (byte)6, (byte)69, (byte)66, (byte)47, (byte)9, (byte)98, (byte)167, (byte)102, (byte)113, (byte)165, (byte)108, (byte)130, (byte)103, (byte)118, (byte)196, (byte)110, (byte)45, (byte)107, (byte)165, (byte)86, (byte)152, (byte)248, (byte)234, (byte)183, (byte)89, (byte)67, (byte)49, (byte)215, (byte)251, (byte)218, (byte)220, (byte)10, (byte)197, (byte)144, (byte)52, (byte)8, (byte)245, (byte)4, (byte)15, (byte)116, (byte)185, (byte)54, (byte)55, (byte)45, (byte)163, (byte)241, (byte)6, (byte)119, (byte)189, (byte)111, (byte)192, (byte)245, (byte)61, (byte)46, (byte)75, (byte)30, (byte)159, (byte)107, (byte)127, (byte)93, (byte)165, (byte)78, (byte)242, (byte)122, (byte)237, (byte)211, (byte)207, (byte)41, (byte)104, (byte)163, (byte)9, (byte)248, (byte)12, (byte)23, (byte)146, (byte)68, (byte)161, (byte)14}));
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.len = (byte)(byte)169;
            p123.data__SET(new byte[] {(byte)117, (byte)150, (byte)237, (byte)116, (byte)200, (byte)36, (byte)237, (byte)150, (byte)182, (byte)63, (byte)233, (byte)160, (byte)55, (byte)177, (byte)136, (byte)248, (byte)219, (byte)241, (byte)98, (byte)158, (byte)88, (byte)207, (byte)42, (byte)63, (byte)124, (byte)94, (byte)69, (byte)51, (byte)236, (byte)241, (byte)99, (byte)84, (byte)6, (byte)69, (byte)66, (byte)47, (byte)9, (byte)98, (byte)167, (byte)102, (byte)113, (byte)165, (byte)108, (byte)130, (byte)103, (byte)118, (byte)196, (byte)110, (byte)45, (byte)107, (byte)165, (byte)86, (byte)152, (byte)248, (byte)234, (byte)183, (byte)89, (byte)67, (byte)49, (byte)215, (byte)251, (byte)218, (byte)220, (byte)10, (byte)197, (byte)144, (byte)52, (byte)8, (byte)245, (byte)4, (byte)15, (byte)116, (byte)185, (byte)54, (byte)55, (byte)45, (byte)163, (byte)241, (byte)6, (byte)119, (byte)189, (byte)111, (byte)192, (byte)245, (byte)61, (byte)46, (byte)75, (byte)30, (byte)159, (byte)107, (byte)127, (byte)93, (byte)165, (byte)78, (byte)242, (byte)122, (byte)237, (byte)211, (byte)207, (byte)41, (byte)104, (byte)163, (byte)9, (byte)248, (byte)12, (byte)23, (byte)146, (byte)68, (byte)161, (byte)14}, 0) ;
            p123.target_component = (byte)(byte)220;
            p123.target_system = (byte)(byte)249;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.eph == (ushort)(ushort)4368);
                Debug.Assert(pack.cog == (ushort)(ushort)19566);
                Debug.Assert(pack.lon == (int)2069065727);
                Debug.Assert(pack.vel == (ushort)(ushort)30999);
                Debug.Assert(pack.lat == (int)885592216);
                Debug.Assert(pack.alt == (int)2001470255);
                Debug.Assert(pack.satellites_visible == (byte)(byte)47);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
                Debug.Assert(pack.epv == (ushort)(ushort)54065);
                Debug.Assert(pack.dgps_numch == (byte)(byte)156);
                Debug.Assert(pack.dgps_age == (uint)862342201U);
                Debug.Assert(pack.time_usec == (ulong)4311385103511085233L);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p124.time_usec = (ulong)4311385103511085233L;
            p124.eph = (ushort)(ushort)4368;
            p124.dgps_age = (uint)862342201U;
            p124.vel = (ushort)(ushort)30999;
            p124.lon = (int)2069065727;
            p124.epv = (ushort)(ushort)54065;
            p124.dgps_numch = (byte)(byte)156;
            p124.satellites_visible = (byte)(byte)47;
            p124.cog = (ushort)(ushort)19566;
            p124.alt = (int)2001470255;
            p124.lat = (int)885592216;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED));
                Debug.Assert(pack.Vcc == (ushort)(ushort)14569);
                Debug.Assert(pack.Vservo == (ushort)(ushort)20541);
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vservo = (ushort)(ushort)20541;
            p125.Vcc = (ushort)(ushort)14569;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED);
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baudrate == (uint)4000537325U);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                                            SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE));
                Debug.Assert(pack.device == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)194, (byte)125, (byte)45, (byte)102, (byte)162, (byte)183, (byte)219, (byte)211, (byte)218, (byte)169, (byte)78, (byte)195, (byte)216, (byte)158, (byte)152, (byte)147, (byte)123, (byte)186, (byte)122, (byte)46, (byte)117, (byte)240, (byte)119, (byte)81, (byte)93, (byte)159, (byte)205, (byte)172, (byte)61, (byte)247, (byte)245, (byte)194, (byte)38, (byte)105, (byte)28, (byte)31, (byte)75, (byte)178, (byte)44, (byte)171, (byte)55, (byte)80, (byte)205, (byte)11, (byte)80, (byte)126, (byte)59, (byte)133, (byte)15, (byte)30, (byte)120, (byte)141, (byte)137, (byte)76, (byte)48, (byte)253, (byte)76, (byte)72, (byte)15, (byte)177, (byte)83, (byte)120, (byte)11, (byte)28, (byte)63, (byte)80, (byte)161, (byte)127, (byte)212, (byte)138}));
                Debug.Assert(pack.timeout == (ushort)(ushort)17654);
                Debug.Assert(pack.count == (byte)(byte)39);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
            p126.timeout = (ushort)(ushort)17654;
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1;
            p126.baudrate = (uint)4000537325U;
            p126.count = (byte)(byte)39;
            p126.data__SET(new byte[] {(byte)194, (byte)125, (byte)45, (byte)102, (byte)162, (byte)183, (byte)219, (byte)211, (byte)218, (byte)169, (byte)78, (byte)195, (byte)216, (byte)158, (byte)152, (byte)147, (byte)123, (byte)186, (byte)122, (byte)46, (byte)117, (byte)240, (byte)119, (byte)81, (byte)93, (byte)159, (byte)205, (byte)172, (byte)61, (byte)247, (byte)245, (byte)194, (byte)38, (byte)105, (byte)28, (byte)31, (byte)75, (byte)178, (byte)44, (byte)171, (byte)55, (byte)80, (byte)205, (byte)11, (byte)80, (byte)126, (byte)59, (byte)133, (byte)15, (byte)30, (byte)120, (byte)141, (byte)137, (byte)76, (byte)48, (byte)253, (byte)76, (byte)72, (byte)15, (byte)177, (byte)83, (byte)120, (byte)11, (byte)28, (byte)63, (byte)80, (byte)161, (byte)127, (byte)212, (byte)138}, 0) ;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wn == (ushort)(ushort)56648);
                Debug.Assert(pack.accuracy == (uint)4267675037U);
                Debug.Assert(pack.rtk_rate == (byte)(byte)249);
                Debug.Assert(pack.tow == (uint)2984624427U);
                Debug.Assert(pack.iar_num_hypotheses == (int)1740093633);
                Debug.Assert(pack.baseline_c_mm == (int)2049276961);
                Debug.Assert(pack.nsats == (byte)(byte)125);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)55);
                Debug.Assert(pack.rtk_health == (byte)(byte)47);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)43);
                Debug.Assert(pack.baseline_b_mm == (int) -1131178188);
                Debug.Assert(pack.baseline_a_mm == (int) -6126418);
                Debug.Assert(pack.time_last_baseline_ms == (uint)2771786548U);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.accuracy = (uint)4267675037U;
            p127.rtk_receiver_id = (byte)(byte)55;
            p127.rtk_health = (byte)(byte)47;
            p127.baseline_c_mm = (int)2049276961;
            p127.rtk_rate = (byte)(byte)249;
            p127.wn = (ushort)(ushort)56648;
            p127.nsats = (byte)(byte)125;
            p127.iar_num_hypotheses = (int)1740093633;
            p127.tow = (uint)2984624427U;
            p127.baseline_a_mm = (int) -6126418;
            p127.time_last_baseline_ms = (uint)2771786548U;
            p127.baseline_coords_type = (byte)(byte)43;
            p127.baseline_b_mm = (int) -1131178188;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_b_mm == (int) -622939133);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)230);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)74);
                Debug.Assert(pack.rtk_health == (byte)(byte)214);
                Debug.Assert(pack.baseline_c_mm == (int) -664428423);
                Debug.Assert(pack.iar_num_hypotheses == (int) -53212182);
                Debug.Assert(pack.time_last_baseline_ms == (uint)4156091416U);
                Debug.Assert(pack.wn == (ushort)(ushort)33520);
                Debug.Assert(pack.nsats == (byte)(byte)193);
                Debug.Assert(pack.baseline_a_mm == (int)1662958844);
                Debug.Assert(pack.rtk_rate == (byte)(byte)77);
                Debug.Assert(pack.accuracy == (uint)2706748838U);
                Debug.Assert(pack.tow == (uint)2666129898U);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.baseline_c_mm = (int) -664428423;
            p128.accuracy = (uint)2706748838U;
            p128.nsats = (byte)(byte)193;
            p128.wn = (ushort)(ushort)33520;
            p128.baseline_coords_type = (byte)(byte)74;
            p128.rtk_receiver_id = (byte)(byte)230;
            p128.iar_num_hypotheses = (int) -53212182;
            p128.baseline_b_mm = (int) -622939133;
            p128.rtk_health = (byte)(byte)214;
            p128.baseline_a_mm = (int)1662958844;
            p128.time_last_baseline_ms = (uint)4156091416U;
            p128.rtk_rate = (byte)(byte)77;
            p128.tow = (uint)2666129898U;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (short)(short) -20330);
                Debug.Assert(pack.yacc == (short)(short)3836);
                Debug.Assert(pack.zmag == (short)(short) -30844);
                Debug.Assert(pack.zacc == (short)(short)4127);
                Debug.Assert(pack.ygyro == (short)(short)21295);
                Debug.Assert(pack.ymag == (short)(short) -14251);
                Debug.Assert(pack.zgyro == (short)(short)22255);
                Debug.Assert(pack.time_boot_ms == (uint)1605174799U);
                Debug.Assert(pack.xmag == (short)(short)25203);
                Debug.Assert(pack.xacc == (short)(short) -16556);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.xgyro = (short)(short) -20330;
            p129.zgyro = (short)(short)22255;
            p129.yacc = (short)(short)3836;
            p129.xmag = (short)(short)25203;
            p129.ymag = (short)(short) -14251;
            p129.zmag = (short)(short) -30844;
            p129.xacc = (short)(short) -16556;
            p129.ygyro = (short)(short)21295;
            p129.time_boot_ms = (uint)1605174799U;
            p129.zacc = (short)(short)4127;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.size == (uint)1273601551U);
                Debug.Assert(pack.payload == (byte)(byte)138);
                Debug.Assert(pack.packets == (ushort)(ushort)19811);
                Debug.Assert(pack.width == (ushort)(ushort)9217);
                Debug.Assert(pack.height == (ushort)(ushort)6531);
                Debug.Assert(pack.jpg_quality == (byte)(byte)225);
                Debug.Assert(pack.type == (byte)(byte)125);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.packets = (ushort)(ushort)19811;
            p130.payload = (byte)(byte)138;
            p130.size = (uint)1273601551U;
            p130.height = (ushort)(ushort)6531;
            p130.type = (byte)(byte)125;
            p130.width = (ushort)(ushort)9217;
            p130.jpg_quality = (byte)(byte)225;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seqnr == (ushort)(ushort)58091);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)163, (byte)92, (byte)111, (byte)36, (byte)66, (byte)9, (byte)186, (byte)233, (byte)24, (byte)72, (byte)59, (byte)80, (byte)161, (byte)147, (byte)116, (byte)245, (byte)96, (byte)65, (byte)63, (byte)178, (byte)167, (byte)59, (byte)94, (byte)42, (byte)10, (byte)40, (byte)15, (byte)131, (byte)225, (byte)19, (byte)149, (byte)118, (byte)204, (byte)127, (byte)111, (byte)60, (byte)55, (byte)167, (byte)33, (byte)63, (byte)118, (byte)161, (byte)51, (byte)30, (byte)161, (byte)137, (byte)157, (byte)126, (byte)104, (byte)202, (byte)198, (byte)130, (byte)64, (byte)193, (byte)109, (byte)169, (byte)247, (byte)106, (byte)237, (byte)159, (byte)14, (byte)176, (byte)34, (byte)214, (byte)186, (byte)168, (byte)32, (byte)153, (byte)16, (byte)153, (byte)6, (byte)43, (byte)27, (byte)88, (byte)250, (byte)245, (byte)54, (byte)224, (byte)108, (byte)178, (byte)95, (byte)74, (byte)154, (byte)205, (byte)84, (byte)219, (byte)100, (byte)228, (byte)100, (byte)30, (byte)22, (byte)158, (byte)7, (byte)209, (byte)164, (byte)160, (byte)202, (byte)61, (byte)29, (byte)86, (byte)168, (byte)227, (byte)43, (byte)4, (byte)24, (byte)150, (byte)90, (byte)19, (byte)201, (byte)5, (byte)226, (byte)201, (byte)168, (byte)254, (byte)62, (byte)46, (byte)72, (byte)88, (byte)92, (byte)5, (byte)170, (byte)211, (byte)185, (byte)6, (byte)11, (byte)98, (byte)221, (byte)41, (byte)191, (byte)153, (byte)23, (byte)19, (byte)199, (byte)231, (byte)150, (byte)249, (byte)137, (byte)202, (byte)4, (byte)218, (byte)132, (byte)199, (byte)79, (byte)112, (byte)230, (byte)181, (byte)8, (byte)125, (byte)156, (byte)230, (byte)219, (byte)135, (byte)62, (byte)72, (byte)238, (byte)205, (byte)125, (byte)221, (byte)59, (byte)67, (byte)103, (byte)99, (byte)107, (byte)3, (byte)56, (byte)189, (byte)80, (byte)178, (byte)65, (byte)145, (byte)86, (byte)234, (byte)11, (byte)176, (byte)232, (byte)71, (byte)83, (byte)125, (byte)175, (byte)128, (byte)151, (byte)229, (byte)159, (byte)232, (byte)220, (byte)241, (byte)228, (byte)165, (byte)165, (byte)206, (byte)74, (byte)79, (byte)158, (byte)65, (byte)114, (byte)90, (byte)112, (byte)45, (byte)237, (byte)183, (byte)74, (byte)122, (byte)197, (byte)188, (byte)133, (byte)29, (byte)17, (byte)246, (byte)236, (byte)254, (byte)252, (byte)178, (byte)236, (byte)240, (byte)90, (byte)125, (byte)3, (byte)102, (byte)79, (byte)122, (byte)108, (byte)152, (byte)14, (byte)147, (byte)105, (byte)132, (byte)239, (byte)200, (byte)243, (byte)46, (byte)205, (byte)90, (byte)88, (byte)85, (byte)12, (byte)67, (byte)236, (byte)173, (byte)244, (byte)121, (byte)29, (byte)252, (byte)218, (byte)76, (byte)11, (byte)23, (byte)0, (byte)255, (byte)86, (byte)44, (byte)252, (byte)33, (byte)237}));
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)58091;
            p131.data__SET(new byte[] {(byte)163, (byte)92, (byte)111, (byte)36, (byte)66, (byte)9, (byte)186, (byte)233, (byte)24, (byte)72, (byte)59, (byte)80, (byte)161, (byte)147, (byte)116, (byte)245, (byte)96, (byte)65, (byte)63, (byte)178, (byte)167, (byte)59, (byte)94, (byte)42, (byte)10, (byte)40, (byte)15, (byte)131, (byte)225, (byte)19, (byte)149, (byte)118, (byte)204, (byte)127, (byte)111, (byte)60, (byte)55, (byte)167, (byte)33, (byte)63, (byte)118, (byte)161, (byte)51, (byte)30, (byte)161, (byte)137, (byte)157, (byte)126, (byte)104, (byte)202, (byte)198, (byte)130, (byte)64, (byte)193, (byte)109, (byte)169, (byte)247, (byte)106, (byte)237, (byte)159, (byte)14, (byte)176, (byte)34, (byte)214, (byte)186, (byte)168, (byte)32, (byte)153, (byte)16, (byte)153, (byte)6, (byte)43, (byte)27, (byte)88, (byte)250, (byte)245, (byte)54, (byte)224, (byte)108, (byte)178, (byte)95, (byte)74, (byte)154, (byte)205, (byte)84, (byte)219, (byte)100, (byte)228, (byte)100, (byte)30, (byte)22, (byte)158, (byte)7, (byte)209, (byte)164, (byte)160, (byte)202, (byte)61, (byte)29, (byte)86, (byte)168, (byte)227, (byte)43, (byte)4, (byte)24, (byte)150, (byte)90, (byte)19, (byte)201, (byte)5, (byte)226, (byte)201, (byte)168, (byte)254, (byte)62, (byte)46, (byte)72, (byte)88, (byte)92, (byte)5, (byte)170, (byte)211, (byte)185, (byte)6, (byte)11, (byte)98, (byte)221, (byte)41, (byte)191, (byte)153, (byte)23, (byte)19, (byte)199, (byte)231, (byte)150, (byte)249, (byte)137, (byte)202, (byte)4, (byte)218, (byte)132, (byte)199, (byte)79, (byte)112, (byte)230, (byte)181, (byte)8, (byte)125, (byte)156, (byte)230, (byte)219, (byte)135, (byte)62, (byte)72, (byte)238, (byte)205, (byte)125, (byte)221, (byte)59, (byte)67, (byte)103, (byte)99, (byte)107, (byte)3, (byte)56, (byte)189, (byte)80, (byte)178, (byte)65, (byte)145, (byte)86, (byte)234, (byte)11, (byte)176, (byte)232, (byte)71, (byte)83, (byte)125, (byte)175, (byte)128, (byte)151, (byte)229, (byte)159, (byte)232, (byte)220, (byte)241, (byte)228, (byte)165, (byte)165, (byte)206, (byte)74, (byte)79, (byte)158, (byte)65, (byte)114, (byte)90, (byte)112, (byte)45, (byte)237, (byte)183, (byte)74, (byte)122, (byte)197, (byte)188, (byte)133, (byte)29, (byte)17, (byte)246, (byte)236, (byte)254, (byte)252, (byte)178, (byte)236, (byte)240, (byte)90, (byte)125, (byte)3, (byte)102, (byte)79, (byte)122, (byte)108, (byte)152, (byte)14, (byte)147, (byte)105, (byte)132, (byte)239, (byte)200, (byte)243, (byte)46, (byte)205, (byte)90, (byte)88, (byte)85, (byte)12, (byte)67, (byte)236, (byte)173, (byte)244, (byte)121, (byte)29, (byte)252, (byte)218, (byte)76, (byte)11, (byte)23, (byte)0, (byte)255, (byte)86, (byte)44, (byte)252, (byte)33, (byte)237}, 0) ;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance == (byte)(byte)21);
                Debug.Assert(pack.id == (byte)(byte)32);
                Debug.Assert(pack.current_distance == (ushort)(ushort)10979);
                Debug.Assert(pack.max_distance == (ushort)(ushort)59133);
                Debug.Assert(pack.min_distance == (ushort)(ushort)718);
                Debug.Assert(pack.type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
                Debug.Assert(pack.time_boot_ms == (uint)2676629750U);
                Debug.Assert(pack.orientation == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_315);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.min_distance = (ushort)(ushort)718;
            p132.id = (byte)(byte)32;
            p132.max_distance = (ushort)(ushort)59133;
            p132.covariance = (byte)(byte)21;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p132.time_boot_ms = (uint)2676629750U;
            p132.current_distance = (ushort)(ushort)10979;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_315;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)55908);
                Debug.Assert(pack.lon == (int) -1413356975);
                Debug.Assert(pack.lat == (int) -1649569045);
                Debug.Assert(pack.mask == (ulong)5270404087713041907L);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lon = (int) -1413356975;
            p133.grid_spacing = (ushort)(ushort)55908;
            p133.lat = (int) -1649569045;
            p133.mask = (ulong)5270404087713041907L;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gridbit == (byte)(byte)151);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short)20997, (short) -5081, (short)16489, (short) -16582, (short)9765, (short) -1814, (short)26934, (short)9874, (short)25092, (short)17269, (short)28003, (short) -15139, (short)928, (short)19231, (short)28895, (short)32247}));
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)19970);
                Debug.Assert(pack.lat == (int) -2138246871);
                Debug.Assert(pack.lon == (int) -652132695);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.gridbit = (byte)(byte)151;
            p134.data__SET(new short[] {(short)20997, (short) -5081, (short)16489, (short) -16582, (short)9765, (short) -1814, (short)26934, (short)9874, (short)25092, (short)17269, (short)28003, (short) -15139, (short)928, (short)19231, (short)28895, (short)32247}, 0) ;
            p134.lon = (int) -652132695;
            p134.lat = (int) -2138246871;
            p134.grid_spacing = (ushort)(ushort)19970;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1355847727);
                Debug.Assert(pack.lon == (int)1644182386);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)1355847727;
            p135.lon = (int)1644182386;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.spacing == (ushort)(ushort)5323);
                Debug.Assert(pack.pending == (ushort)(ushort)33408);
                Debug.Assert(pack.terrain_height == (float) -2.406481E38F);
                Debug.Assert(pack.loaded == (ushort)(ushort)7263);
                Debug.Assert(pack.current_height == (float) -1.0697297E38F);
                Debug.Assert(pack.lon == (int) -300428296);
                Debug.Assert(pack.lat == (int) -351460667);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.terrain_height = (float) -2.406481E38F;
            p136.loaded = (ushort)(ushort)7263;
            p136.current_height = (float) -1.0697297E38F;
            p136.lat = (int) -351460667;
            p136.lon = (int) -300428296;
            p136.pending = (ushort)(ushort)33408;
            p136.spacing = (ushort)(ushort)5323;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)11189);
                Debug.Assert(pack.time_boot_ms == (uint)1234874102U);
                Debug.Assert(pack.press_abs == (float)3.2582817E38F);
                Debug.Assert(pack.press_diff == (float) -1.7036762E38F);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.temperature = (short)(short)11189;
            p137.press_abs = (float)3.2582817E38F;
            p137.time_boot_ms = (uint)1234874102U;
            p137.press_diff = (float) -1.7036762E38F;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -3.3683929E38F);
                Debug.Assert(pack.x == (float)2.5247243E38F);
                Debug.Assert(pack.time_usec == (ulong)5559829842808982974L);
                Debug.Assert(pack.z == (float)1.2361121E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {4.351219E37F, -8.0143264E37F, 2.0528533E38F, 1.8699068E37F}));
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.y = (float) -3.3683929E38F;
            p138.time_usec = (ulong)5559829842808982974L;
            p138.q_SET(new float[] {4.351219E37F, -8.0143264E37F, 2.0528533E38F, 1.8699068E37F}, 0) ;
            p138.x = (float)2.5247243E38F;
            p138.z = (float)1.2361121E38F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)197);
                Debug.Assert(pack.group_mlx == (byte)(byte)220);
                Debug.Assert(pack.target_component == (byte)(byte)117);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {1.2976265E38F, -1.7548325E38F, 3.237091E38F, 1.5507245E38F, -2.1162035E38F, -1.4291541E38F, 7.592523E37F, -1.3271981E38F}));
                Debug.Assert(pack.time_usec == (ulong)3310071750895658965L);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.target_component = (byte)(byte)117;
            p139.target_system = (byte)(byte)197;
            p139.time_usec = (ulong)3310071750895658965L;
            p139.controls_SET(new float[] {1.2976265E38F, -1.7548325E38F, 3.237091E38F, 1.5507245E38F, -2.1162035E38F, -1.4291541E38F, 7.592523E37F, -1.3271981E38F}, 0) ;
            p139.group_mlx = (byte)(byte)220;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)6170170416755634025L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {1.0713192E38F, 9.443337E37F, -1.6261449E38F, 6.2586293E37F, 1.755984E37F, 2.9668019E37F, 2.9144456E38F, 2.533877E36F}));
                Debug.Assert(pack.group_mlx == (byte)(byte)91);
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.group_mlx = (byte)(byte)91;
            p140.time_usec = (ulong)6170170416755634025L;
            p140.controls_SET(new float[] {1.0713192E38F, 9.443337E37F, -1.6261449E38F, 6.2586293E37F, 1.755984E37F, 2.9668019E37F, 2.9144456E38F, 2.533877E36F}, 0) ;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_terrain == (float) -3.0538304E37F);
                Debug.Assert(pack.altitude_relative == (float)2.790409E38F);
                Debug.Assert(pack.altitude_amsl == (float)2.5164373E37F);
                Debug.Assert(pack.bottom_clearance == (float) -7.64533E37F);
                Debug.Assert(pack.altitude_monotonic == (float)1.9051551E38F);
                Debug.Assert(pack.altitude_local == (float)2.1474954E38F);
                Debug.Assert(pack.time_usec == (ulong)2446554688312600142L);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_local = (float)2.1474954E38F;
            p141.altitude_amsl = (float)2.5164373E37F;
            p141.altitude_terrain = (float) -3.0538304E37F;
            p141.altitude_relative = (float)2.790409E38F;
            p141.bottom_clearance = (float) -7.64533E37F;
            p141.time_usec = (ulong)2446554688312600142L;
            p141.altitude_monotonic = (float)1.9051551E38F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)251, (byte)149, (byte)165, (byte)2, (byte)154, (byte)116, (byte)196, (byte)185, (byte)28, (byte)89, (byte)88, (byte)104, (byte)6, (byte)175, (byte)161, (byte)39, (byte)61, (byte)93, (byte)80, (byte)11, (byte)25, (byte)251, (byte)187, (byte)50, (byte)170, (byte)115, (byte)66, (byte)226, (byte)219, (byte)78, (byte)30, (byte)6, (byte)133, (byte)127, (byte)243, (byte)147, (byte)199, (byte)196, (byte)183, (byte)46, (byte)74, (byte)225, (byte)31, (byte)154, (byte)18, (byte)206, (byte)243, (byte)156, (byte)189, (byte)52, (byte)250, (byte)217, (byte)198, (byte)154, (byte)251, (byte)143, (byte)105, (byte)223, (byte)224, (byte)94, (byte)113, (byte)226, (byte)6, (byte)40, (byte)250, (byte)218, (byte)31, (byte)187, (byte)184, (byte)162, (byte)28, (byte)113, (byte)160, (byte)210, (byte)252, (byte)1, (byte)79, (byte)244, (byte)23, (byte)7, (byte)185, (byte)58, (byte)8, (byte)20, (byte)24, (byte)151, (byte)99, (byte)142, (byte)60, (byte)119, (byte)46, (byte)143, (byte)29, (byte)18, (byte)141, (byte)102, (byte)34, (byte)62, (byte)83, (byte)167, (byte)143, (byte)178, (byte)165, (byte)119, (byte)85, (byte)2, (byte)45, (byte)138, (byte)21, (byte)104, (byte)121, (byte)162, (byte)36, (byte)155, (byte)213, (byte)3, (byte)49, (byte)6, (byte)124, (byte)29}));
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)240, (byte)244, (byte)117, (byte)196, (byte)62, (byte)178, (byte)41, (byte)135, (byte)70, (byte)213, (byte)204, (byte)62, (byte)230, (byte)87, (byte)36, (byte)230, (byte)147, (byte)236, (byte)180, (byte)50, (byte)99, (byte)85, (byte)87, (byte)91, (byte)191, (byte)37, (byte)166, (byte)72, (byte)242, (byte)122, (byte)122, (byte)146, (byte)114, (byte)2, (byte)136, (byte)12, (byte)29, (byte)32, (byte)187, (byte)31, (byte)106, (byte)65, (byte)206, (byte)105, (byte)249, (byte)186, (byte)253, (byte)8, (byte)43, (byte)5, (byte)235, (byte)203, (byte)120, (byte)189, (byte)190, (byte)32, (byte)12, (byte)62, (byte)193, (byte)131, (byte)124, (byte)172, (byte)2, (byte)41, (byte)157, (byte)82, (byte)35, (byte)213, (byte)201, (byte)103, (byte)123, (byte)214, (byte)226, (byte)216, (byte)12, (byte)228, (byte)60, (byte)21, (byte)186, (byte)145, (byte)47, (byte)25, (byte)248, (byte)147, (byte)177, (byte)240, (byte)120, (byte)28, (byte)179, (byte)217, (byte)97, (byte)251, (byte)219, (byte)172, (byte)221, (byte)20, (byte)162, (byte)212, (byte)38, (byte)162, (byte)225, (byte)73, (byte)196, (byte)59, (byte)155, (byte)154, (byte)227, (byte)165, (byte)174, (byte)37, (byte)12, (byte)91, (byte)226, (byte)101, (byte)215, (byte)117, (byte)178, (byte)110, (byte)112, (byte)146}));
                Debug.Assert(pack.request_id == (byte)(byte)185);
                Debug.Assert(pack.transfer_type == (byte)(byte)82);
                Debug.Assert(pack.uri_type == (byte)(byte)99);
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.transfer_type = (byte)(byte)82;
            p142.request_id = (byte)(byte)185;
            p142.uri_SET(new byte[] {(byte)240, (byte)244, (byte)117, (byte)196, (byte)62, (byte)178, (byte)41, (byte)135, (byte)70, (byte)213, (byte)204, (byte)62, (byte)230, (byte)87, (byte)36, (byte)230, (byte)147, (byte)236, (byte)180, (byte)50, (byte)99, (byte)85, (byte)87, (byte)91, (byte)191, (byte)37, (byte)166, (byte)72, (byte)242, (byte)122, (byte)122, (byte)146, (byte)114, (byte)2, (byte)136, (byte)12, (byte)29, (byte)32, (byte)187, (byte)31, (byte)106, (byte)65, (byte)206, (byte)105, (byte)249, (byte)186, (byte)253, (byte)8, (byte)43, (byte)5, (byte)235, (byte)203, (byte)120, (byte)189, (byte)190, (byte)32, (byte)12, (byte)62, (byte)193, (byte)131, (byte)124, (byte)172, (byte)2, (byte)41, (byte)157, (byte)82, (byte)35, (byte)213, (byte)201, (byte)103, (byte)123, (byte)214, (byte)226, (byte)216, (byte)12, (byte)228, (byte)60, (byte)21, (byte)186, (byte)145, (byte)47, (byte)25, (byte)248, (byte)147, (byte)177, (byte)240, (byte)120, (byte)28, (byte)179, (byte)217, (byte)97, (byte)251, (byte)219, (byte)172, (byte)221, (byte)20, (byte)162, (byte)212, (byte)38, (byte)162, (byte)225, (byte)73, (byte)196, (byte)59, (byte)155, (byte)154, (byte)227, (byte)165, (byte)174, (byte)37, (byte)12, (byte)91, (byte)226, (byte)101, (byte)215, (byte)117, (byte)178, (byte)110, (byte)112, (byte)146}, 0) ;
            p142.uri_type = (byte)(byte)99;
            p142.storage_SET(new byte[] {(byte)251, (byte)149, (byte)165, (byte)2, (byte)154, (byte)116, (byte)196, (byte)185, (byte)28, (byte)89, (byte)88, (byte)104, (byte)6, (byte)175, (byte)161, (byte)39, (byte)61, (byte)93, (byte)80, (byte)11, (byte)25, (byte)251, (byte)187, (byte)50, (byte)170, (byte)115, (byte)66, (byte)226, (byte)219, (byte)78, (byte)30, (byte)6, (byte)133, (byte)127, (byte)243, (byte)147, (byte)199, (byte)196, (byte)183, (byte)46, (byte)74, (byte)225, (byte)31, (byte)154, (byte)18, (byte)206, (byte)243, (byte)156, (byte)189, (byte)52, (byte)250, (byte)217, (byte)198, (byte)154, (byte)251, (byte)143, (byte)105, (byte)223, (byte)224, (byte)94, (byte)113, (byte)226, (byte)6, (byte)40, (byte)250, (byte)218, (byte)31, (byte)187, (byte)184, (byte)162, (byte)28, (byte)113, (byte)160, (byte)210, (byte)252, (byte)1, (byte)79, (byte)244, (byte)23, (byte)7, (byte)185, (byte)58, (byte)8, (byte)20, (byte)24, (byte)151, (byte)99, (byte)142, (byte)60, (byte)119, (byte)46, (byte)143, (byte)29, (byte)18, (byte)141, (byte)102, (byte)34, (byte)62, (byte)83, (byte)167, (byte)143, (byte)178, (byte)165, (byte)119, (byte)85, (byte)2, (byte)45, (byte)138, (byte)21, (byte)104, (byte)121, (byte)162, (byte)36, (byte)155, (byte)213, (byte)3, (byte)49, (byte)6, (byte)124, (byte)29}, 0) ;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float)2.180209E38F);
                Debug.Assert(pack.press_abs == (float) -1.5475343E38F);
                Debug.Assert(pack.temperature == (short)(short)15903);
                Debug.Assert(pack.time_boot_ms == (uint)3973867627U);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.press_diff = (float)2.180209E38F;
            p143.temperature = (short)(short)15903;
            p143.time_boot_ms = (uint)3973867627U;
            p143.press_abs = (float) -1.5475343E38F;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)1675369714);
                Debug.Assert(pack.lat == (int) -1727562888);
                Debug.Assert(pack.est_capabilities == (byte)(byte)146);
                Debug.Assert(pack.custom_state == (ulong)5312173335919343352L);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {3.2709766E37F, -1.0734763E38F, 3.2163845E38F}));
                Debug.Assert(pack.alt == (float) -3.1825277E38F);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-2.3723756E38F, 1.0445795E38F, 2.3175052E38F, 1.5698017E38F}));
                Debug.Assert(pack.timestamp == (ulong)1656568961027504552L);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {-1.36914E37F, 1.843431E38F, 1.3982864E38F}));
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-1.5778018E38F, -2.8668851E38F, 2.6683715E38F}));
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {-6.9717654E37F, 1.8826974E38F, 3.1716555E37F}));
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.vel_SET(new float[] {3.2709766E37F, -1.0734763E38F, 3.2163845E38F}, 0) ;
            p144.attitude_q_SET(new float[] {-2.3723756E38F, 1.0445795E38F, 2.3175052E38F, 1.5698017E38F}, 0) ;
            p144.position_cov_SET(new float[] {-6.9717654E37F, 1.8826974E38F, 3.1716555E37F}, 0) ;
            p144.lat = (int) -1727562888;
            p144.est_capabilities = (byte)(byte)146;
            p144.lon = (int)1675369714;
            p144.custom_state = (ulong)5312173335919343352L;
            p144.timestamp = (ulong)1656568961027504552L;
            p144.alt = (float) -3.1825277E38F;
            p144.acc_SET(new float[] {-1.36914E37F, 1.843431E38F, 1.3982864E38F}, 0) ;
            p144.rates_SET(new float[] {-1.5778018E38F, -2.8668851E38F, 2.6683715E38F}, 0) ;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {-1.9806424E38F, 7.855697E37F, -2.8412613E38F}));
                Debug.Assert(pack.yaw_rate == (float)2.220047E38F);
                Debug.Assert(pack.z_vel == (float)4.4817123E37F);
                Debug.Assert(pack.x_vel == (float)2.4785918E38F);
                Debug.Assert(pack.x_acc == (float) -3.0567354E38F);
                Debug.Assert(pack.y_vel == (float) -1.976815E37F);
                Debug.Assert(pack.pitch_rate == (float) -2.964993E38F);
                Debug.Assert(pack.airspeed == (float)2.5785987E38F);
                Debug.Assert(pack.z_acc == (float)2.3133327E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.8673245E37F, 2.5206871E38F, -2.0299656E38F, -3.2332828E38F}));
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {-3.3077951E38F, 1.9846612E38F, 2.3229053E37F}));
                Debug.Assert(pack.roll_rate == (float)1.879889E38F);
                Debug.Assert(pack.x_pos == (float)2.5496413E38F);
                Debug.Assert(pack.y_acc == (float)8.483835E37F);
                Debug.Assert(pack.y_pos == (float) -3.0933904E38F);
                Debug.Assert(pack.time_usec == (ulong)2046374825927222367L);
                Debug.Assert(pack.z_pos == (float)1.165864E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.y_pos = (float) -3.0933904E38F;
            p146.airspeed = (float)2.5785987E38F;
            p146.z_vel = (float)4.4817123E37F;
            p146.q_SET(new float[] {-1.8673245E37F, 2.5206871E38F, -2.0299656E38F, -3.2332828E38F}, 0) ;
            p146.yaw_rate = (float)2.220047E38F;
            p146.roll_rate = (float)1.879889E38F;
            p146.y_acc = (float)8.483835E37F;
            p146.z_pos = (float)1.165864E38F;
            p146.z_acc = (float)2.3133327E38F;
            p146.y_vel = (float) -1.976815E37F;
            p146.time_usec = (ulong)2046374825927222367L;
            p146.pos_variance_SET(new float[] {-1.9806424E38F, 7.855697E37F, -2.8412613E38F}, 0) ;
            p146.vel_variance_SET(new float[] {-3.3077951E38F, 1.9846612E38F, 2.3229053E37F}, 0) ;
            p146.x_vel = (float)2.4785918E38F;
            p146.pitch_rate = (float) -2.964993E38F;
            p146.x_acc = (float) -3.0567354E38F;
            p146.x_pos = (float)2.5496413E38F;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.battery_function == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS);
                Debug.Assert(pack.current_battery == (short)(short)6690);
                Debug.Assert(pack.id == (byte)(byte)155);
                Debug.Assert(pack.current_consumed == (int)1635085883);
                Debug.Assert(pack.temperature == (short)(short)13272);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)88);
                Debug.Assert(pack.type == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION);
                Debug.Assert(pack.energy_consumed == (int)404152253);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)58268, (ushort)23319, (ushort)64899, (ushort)7597, (ushort)61869, (ushort)48090, (ushort)5834, (ushort)29644, (ushort)38766, (ushort)41443}));
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.voltages_SET(new ushort[] {(ushort)58268, (ushort)23319, (ushort)64899, (ushort)7597, (ushort)61869, (ushort)48090, (ushort)5834, (ushort)29644, (ushort)38766, (ushort)41443}, 0) ;
            p147.energy_consumed = (int)404152253;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION;
            p147.battery_remaining = (sbyte)(sbyte)88;
            p147.current_consumed = (int)1635085883;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS;
            p147.temperature = (short)(short)13272;
            p147.current_battery = (short)(short)6690;
            p147.id = (byte)(byte)155;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION));
                Debug.Assert(pack.vendor_id == (ushort)(ushort)9774);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)200, (byte)108, (byte)165, (byte)23, (byte)56, (byte)119, (byte)151, (byte)123, (byte)143, (byte)232, (byte)70, (byte)131, (byte)192, (byte)15, (byte)196, (byte)220, (byte)22, (byte)19}));
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)90, (byte)150, (byte)118, (byte)55, (byte)229, (byte)70, (byte)23, (byte)28}));
                Debug.Assert(pack.middleware_sw_version == (uint)2894331039U);
                Debug.Assert(pack.flight_sw_version == (uint)492077432U);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)82, (byte)26, (byte)191, (byte)155, (byte)73, (byte)220, (byte)180, (byte)3}));
                Debug.Assert(pack.uid == (ulong)4074311807305187011L);
                Debug.Assert(pack.os_sw_version == (uint)3913005882U);
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)109, (byte)26, (byte)231, (byte)101, (byte)111, (byte)77, (byte)181, (byte)158}));
                Debug.Assert(pack.product_id == (ushort)(ushort)59543);
                Debug.Assert(pack.board_version == (uint)234215831U);
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.uid2_SET(new byte[] {(byte)200, (byte)108, (byte)165, (byte)23, (byte)56, (byte)119, (byte)151, (byte)123, (byte)143, (byte)232, (byte)70, (byte)131, (byte)192, (byte)15, (byte)196, (byte)220, (byte)22, (byte)19}, 0, PH) ;
            p148.flight_custom_version_SET(new byte[] {(byte)82, (byte)26, (byte)191, (byte)155, (byte)73, (byte)220, (byte)180, (byte)3}, 0) ;
            p148.middleware_sw_version = (uint)2894331039U;
            p148.uid = (ulong)4074311807305187011L;
            p148.board_version = (uint)234215831U;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION);
            p148.os_sw_version = (uint)3913005882U;
            p148.middleware_custom_version_SET(new byte[] {(byte)90, (byte)150, (byte)118, (byte)55, (byte)229, (byte)70, (byte)23, (byte)28}, 0) ;
            p148.flight_sw_version = (uint)492077432U;
            p148.vendor_id = (ushort)(ushort)9774;
            p148.os_custom_version_SET(new byte[] {(byte)109, (byte)26, (byte)231, (byte)101, (byte)111, (byte)77, (byte)181, (byte)158}, 0) ;
            p148.product_id = (ushort)(ushort)59543;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)159);
                Debug.Assert(pack.angle_y == (float)2.9237162E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.size_x == (float)3.069038E37F);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {1.5254956E38F, 1.894727E38F, -4.4137444E37F, 1.9214166E38F}));
                Debug.Assert(pack.time_usec == (ulong)8055361197649062581L);
                Debug.Assert(pack.angle_x == (float)2.7834035E38F);
                Debug.Assert(pack.target_num == (byte)(byte)232);
                Debug.Assert(pack.type == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
                Debug.Assert(pack.x_TRY(ph) == (float)2.9714105E38F);
                Debug.Assert(pack.z_TRY(ph) == (float) -7.31412E37F);
                Debug.Assert(pack.y_TRY(ph) == (float) -1.1947192E38F);
                Debug.Assert(pack.size_y == (float)2.8844757E38F);
                Debug.Assert(pack.distance == (float) -2.497313E38F);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.size_y = (float)2.8844757E38F;
            p149.target_num = (byte)(byte)232;
            p149.size_x = (float)3.069038E37F;
            p149.position_valid_SET((byte)(byte)159, PH) ;
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON;
            p149.q_SET(new float[] {1.5254956E38F, 1.894727E38F, -4.4137444E37F, 1.9214166E38F}, 0, PH) ;
            p149.angle_x = (float)2.7834035E38F;
            p149.distance = (float) -2.497313E38F;
            p149.time_usec = (ulong)8055361197649062581L;
            p149.x_SET((float)2.9714105E38F, PH) ;
            p149.frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p149.angle_y = (float)2.9237162E38F;
            p149.y_SET((float) -1.1947192E38F, PH) ;
            p149.z_SET((float) -7.31412E37F, PH) ;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_0Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {2870407392U, 1922319282U, 3101102550U, 2714122761U}));
                Debug.Assert(pack.ar_i8.SequenceEqual(new sbyte[] {(sbyte) - 98, (sbyte)33, (sbyte)84, (sbyte)90}));
                Debug.Assert(pack.ar_u16.SequenceEqual(new ushort[] {(ushort)40015, (ushort)9351, (ushort)18338, (ushort)55157}));
                Debug.Assert(pack.ar_u8.SequenceEqual(new byte[] {(byte)164, (byte)73, (byte)102, (byte)173}));
                Debug.Assert(pack.v1 == (byte)(byte)140);
            };
            GroundControl.ARRAY_TEST_0 p150 = CommunicationChannel.new_ARRAY_TEST_0();
            PH.setPack(p150);
            p150.v1 = (byte)(byte)140;
            p150.ar_u32_SET(new uint[] {2870407392U, 1922319282U, 3101102550U, 2714122761U}, 0) ;
            p150.ar_i8_SET(new sbyte[] {(sbyte) - 98, (sbyte)33, (sbyte)84, (sbyte)90}, 0) ;
            p150.ar_u8_SET(new byte[] {(byte)164, (byte)73, (byte)102, (byte)173}, 0) ;
            p150.ar_u16_SET(new ushort[] {(ushort)40015, (ushort)9351, (ushort)18338, (ushort)55157}, 0) ;
            CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_1Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {2921913438U, 4280550064U, 2721364500U, 537983813U}));
            };
            GroundControl.ARRAY_TEST_1 p151 = CommunicationChannel.new_ARRAY_TEST_1();
            PH.setPack(p151);
            p151.ar_u32_SET(new uint[] {2921913438U, 4280550064U, 2721364500U, 537983813U}, 0) ;
            CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.v == (byte)(byte)3);
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {593869636U, 1581524493U, 687804930U, 3726295789U}));
            };
            GroundControl.ARRAY_TEST_3 p153 = CommunicationChannel.new_ARRAY_TEST_3();
            PH.setPack(p153);
            p153.v = (byte)(byte)3;
            p153.ar_u32_SET(new uint[] {593869636U, 1581524493U, 687804930U, 3726295789U}, 0) ;
            CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_4Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.v == (byte)(byte)81);
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {1650514527U, 404121474U, 4257643368U, 2372755512U}));
            };
            GroundControl.ARRAY_TEST_4 p154 = CommunicationChannel.new_ARRAY_TEST_4();
            PH.setPack(p154);
            p154.v = (byte)(byte)81;
            p154.ar_u32_SET(new uint[] {1650514527U, 404121474U, 4257643368U, 2372755512U}, 0) ;
            CommunicationChannel.instance.send(p154);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_5Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.c1_LEN(ph) == 2);
                Debug.Assert(pack.c1_TRY(ph).Equals("gl"));
                Debug.Assert(pack.c2_LEN(ph) == 2);
                Debug.Assert(pack.c2_TRY(ph).Equals("ey"));
            };
            GroundControl.ARRAY_TEST_5 p155 = CommunicationChannel.new_ARRAY_TEST_5();
            PH.setPack(p155);
            p155.c2_SET("ey", PH) ;
            p155.c1_SET("gl", PH) ;
            CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_6Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.v3 == (uint)157239079U);
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {3719089674U, 1726113284U}));
                Debug.Assert(pack.ar_f.SequenceEqual(new float[] {2.3616426E38F, 3.0453446E38F}));
                Debug.Assert(pack.ar_i32.SequenceEqual(new int[] {2019502974, -1123309590}));
                Debug.Assert(pack.ar_u8.SequenceEqual(new byte[] {(byte)186, (byte)24}));
                Debug.Assert(pack.v2 == (ushort)(ushort)2198);
                Debug.Assert(pack.ar_u16.SequenceEqual(new ushort[] {(ushort)3380, (ushort)30604}));
                Debug.Assert(pack.v1 == (byte)(byte)86);
                Debug.Assert(pack.ar_d.SequenceEqual(new double[] {1.4133919085845568E308, -1.7615120905610782E308}));
                Debug.Assert(pack.ar_i8.SequenceEqual(new sbyte[] {(sbyte) - 59, (sbyte) - 103}));
                Debug.Assert(pack.ar_i16.SequenceEqual(new short[] {(short) -30313, (short)24774}));
                Debug.Assert(pack.ar_c_LEN(ph) == 17);
                Debug.Assert(pack.ar_c_TRY(ph).Equals("pvpAueigqlwcmbfDi"));
            };
            GroundControl.ARRAY_TEST_6 p156 = CommunicationChannel.new_ARRAY_TEST_6();
            PH.setPack(p156);
            p156.ar_i32_SET(new int[] {2019502974, -1123309590}, 0) ;
            p156.v3 = (uint)157239079U;
            p156.ar_f_SET(new float[] {2.3616426E38F, 3.0453446E38F}, 0) ;
            p156.ar_d_SET(new double[] {1.4133919085845568E308, -1.7615120905610782E308}, 0) ;
            p156.v2 = (ushort)(ushort)2198;
            p156.ar_i16_SET(new short[] {(short) -30313, (short)24774}, 0) ;
            p156.ar_i8_SET(new sbyte[] {(sbyte) - 59, (sbyte) - 103}, 0) ;
            p156.ar_u32_SET(new uint[] {3719089674U, 1726113284U}, 0) ;
            p156.v1 = (byte)(byte)86;
            p156.ar_u16_SET(new ushort[] {(ushort)3380, (ushort)30604}, 0) ;
            p156.ar_u8_SET(new byte[] {(byte)186, (byte)24}, 0) ;
            p156.ar_c_SET("pvpAueigqlwcmbfDi", PH) ;
            CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_7Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ar_d.SequenceEqual(new double[] {-4.123906108548528E307, 1.7830161985409582E308}));
                Debug.Assert(pack.ar_c_LEN(ph) == 17);
                Debug.Assert(pack.ar_c_TRY(ph).Equals("bgKggxsiTpgidvlkw"));
                Debug.Assert(pack.ar_i16.SequenceEqual(new short[] {(short)1238, (short) -16814}));
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {3487138365U, 511146331U}));
                Debug.Assert(pack.ar_i8.SequenceEqual(new sbyte[] {(sbyte) - 10, (sbyte)17}));
                Debug.Assert(pack.ar_f.SequenceEqual(new float[] {-9.893704E36F, -5.2672343E37F}));
                Debug.Assert(pack.ar_i32.SequenceEqual(new int[] {747863239, 1105756371}));
                Debug.Assert(pack.ar_u8.SequenceEqual(new byte[] {(byte)118, (byte)62}));
                Debug.Assert(pack.ar_u16.SequenceEqual(new ushort[] {(ushort)9130, (ushort)25207}));
            };
            GroundControl.ARRAY_TEST_7 p157 = CommunicationChannel.new_ARRAY_TEST_7();
            PH.setPack(p157);
            p157.ar_c_SET("bgKggxsiTpgidvlkw", PH) ;
            p157.ar_i8_SET(new sbyte[] {(sbyte) - 10, (sbyte)17}, 0) ;
            p157.ar_i32_SET(new int[] {747863239, 1105756371}, 0) ;
            p157.ar_u32_SET(new uint[] {3487138365U, 511146331U}, 0) ;
            p157.ar_d_SET(new double[] {-4.123906108548528E307, 1.7830161985409582E308}, 0) ;
            p157.ar_u8_SET(new byte[] {(byte)118, (byte)62}, 0) ;
            p157.ar_i16_SET(new short[] {(short)1238, (short) -16814}, 0) ;
            p157.ar_u16_SET(new ushort[] {(ushort)9130, (ushort)25207}, 0) ;
            p157.ar_f_SET(new float[] {-9.893704E36F, -5.2672343E37F}, 0) ;
            CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_8Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ar_d.SequenceEqual(new double[] {-9.928551265152998E307, -1.6003929611404818E308}));
                Debug.Assert(pack.ar_u16.SequenceEqual(new ushort[] {(ushort)35957, (ushort)46112}));
                Debug.Assert(pack.v3 == (uint)749353934U);
            };
            GroundControl.ARRAY_TEST_8 p158 = CommunicationChannel.new_ARRAY_TEST_8();
            PH.setPack(p158);
            p158.ar_d_SET(new double[] {-9.928551265152998E307, -1.6003929611404818E308}, 0) ;
            p158.ar_u16_SET(new ushort[] {(ushort)35957, (ushort)46112}, 0) ;
            p158.v3 = (uint)749353934U;
            CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3886929943250750964L);
                Debug.Assert(pack.vel_ratio == (float)1.533663E38F);
                Debug.Assert(pack.hagl_ratio == (float) -1.4371794E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS));
                Debug.Assert(pack.pos_horiz_ratio == (float)6.4275245E37F);
                Debug.Assert(pack.mag_ratio == (float) -1.3698829E38F);
                Debug.Assert(pack.pos_vert_accuracy == (float) -7.1372324E37F);
                Debug.Assert(pack.pos_vert_ratio == (float)2.8546719E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float)3.364186E38F);
                Debug.Assert(pack.tas_ratio == (float) -1.2197299E38F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.hagl_ratio = (float) -1.4371794E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS);
            p230.pos_horiz_ratio = (float)6.4275245E37F;
            p230.pos_horiz_accuracy = (float)3.364186E38F;
            p230.time_usec = (ulong)3886929943250750964L;
            p230.vel_ratio = (float)1.533663E38F;
            p230.pos_vert_accuracy = (float) -7.1372324E37F;
            p230.mag_ratio = (float) -1.3698829E38F;
            p230.tas_ratio = (float) -1.2197299E38F;
            p230.pos_vert_ratio = (float)2.8546719E38F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wind_z == (float) -1.0953506E38F);
                Debug.Assert(pack.wind_x == (float)2.7176279E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -3.153619E38F);
                Debug.Assert(pack.wind_y == (float)2.5796317E38F);
                Debug.Assert(pack.time_usec == (ulong)6618352287644710141L);
                Debug.Assert(pack.var_vert == (float) -1.6078704E37F);
                Debug.Assert(pack.wind_alt == (float) -4.8477333E36F);
                Debug.Assert(pack.var_horiz == (float)1.5255004E38F);
                Debug.Assert(pack.vert_accuracy == (float)6.5515266E37F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.var_horiz = (float)1.5255004E38F;
            p231.time_usec = (ulong)6618352287644710141L;
            p231.wind_alt = (float) -4.8477333E36F;
            p231.wind_y = (float)2.5796317E38F;
            p231.horiz_accuracy = (float) -3.153619E38F;
            p231.wind_z = (float) -1.0953506E38F;
            p231.vert_accuracy = (float)6.5515266E37F;
            p231.var_vert = (float) -1.6078704E37F;
            p231.wind_x = (float)2.7176279E38F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_week_ms == (uint)475803563U);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP));
                Debug.Assert(pack.gps_id == (byte)(byte)126);
                Debug.Assert(pack.lon == (int) -1992883175);
                Debug.Assert(pack.vd == (float) -1.028238E38F);
                Debug.Assert(pack.alt == (float) -1.874102E38F);
                Debug.Assert(pack.vdop == (float) -1.9082431E38F);
                Debug.Assert(pack.vert_accuracy == (float) -9.027435E37F);
                Debug.Assert(pack.horiz_accuracy == (float) -2.3745702E38F);
                Debug.Assert(pack.time_week == (ushort)(ushort)37783);
                Debug.Assert(pack.vn == (float) -3.1097258E38F);
                Debug.Assert(pack.satellites_visible == (byte)(byte)1);
                Debug.Assert(pack.fix_type == (byte)(byte)251);
                Debug.Assert(pack.lat == (int)1265174865);
                Debug.Assert(pack.hdop == (float)6.440734E37F);
                Debug.Assert(pack.speed_accuracy == (float)8.90794E37F);
                Debug.Assert(pack.ve == (float) -2.9770503E38F);
                Debug.Assert(pack.time_usec == (ulong)6025268221137203008L);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.alt = (float) -1.874102E38F;
            p232.time_week_ms = (uint)475803563U;
            p232.time_week = (ushort)(ushort)37783;
            p232.vdop = (float) -1.9082431E38F;
            p232.vert_accuracy = (float) -9.027435E37F;
            p232.fix_type = (byte)(byte)251;
            p232.gps_id = (byte)(byte)126;
            p232.lon = (int) -1992883175;
            p232.time_usec = (ulong)6025268221137203008L;
            p232.vn = (float) -3.1097258E38F;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP);
            p232.horiz_accuracy = (float) -2.3745702E38F;
            p232.hdop = (float)6.440734E37F;
            p232.vd = (float) -1.028238E38F;
            p232.lat = (int)1265174865;
            p232.satellites_visible = (byte)(byte)1;
            p232.speed_accuracy = (float)8.90794E37F;
            p232.ve = (float) -2.9770503E38F;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)242);
                Debug.Assert(pack.flags == (byte)(byte)17);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)23, (byte)62, (byte)200, (byte)144, (byte)8, (byte)75, (byte)249, (byte)250, (byte)40, (byte)229, (byte)217, (byte)143, (byte)240, (byte)20, (byte)174, (byte)134, (byte)129, (byte)193, (byte)154, (byte)203, (byte)74, (byte)89, (byte)201, (byte)101, (byte)117, (byte)122, (byte)180, (byte)14, (byte)215, (byte)33, (byte)146, (byte)218, (byte)77, (byte)185, (byte)100, (byte)54, (byte)210, (byte)191, (byte)234, (byte)72, (byte)157, (byte)20, (byte)65, (byte)62, (byte)52, (byte)0, (byte)55, (byte)223, (byte)207, (byte)118, (byte)201, (byte)216, (byte)67, (byte)77, (byte)151, (byte)13, (byte)35, (byte)132, (byte)130, (byte)181, (byte)17, (byte)135, (byte)182, (byte)2, (byte)203, (byte)135, (byte)238, (byte)155, (byte)163, (byte)251, (byte)184, (byte)47, (byte)188, (byte)41, (byte)67, (byte)250, (byte)142, (byte)98, (byte)251, (byte)75, (byte)109, (byte)152, (byte)208, (byte)225, (byte)166, (byte)97, (byte)211, (byte)160, (byte)244, (byte)229, (byte)194, (byte)234, (byte)36, (byte)193, (byte)3, (byte)142, (byte)222, (byte)90, (byte)228, (byte)54, (byte)119, (byte)70, (byte)117, (byte)193, (byte)32, (byte)37, (byte)66, (byte)180, (byte)16, (byte)184, (byte)245, (byte)242, (byte)38, (byte)237, (byte)76, (byte)7, (byte)68, (byte)132, (byte)48, (byte)123, (byte)28, (byte)249, (byte)26, (byte)69, (byte)112, (byte)228, (byte)57, (byte)162, (byte)218, (byte)68, (byte)250, (byte)50, (byte)126, (byte)200, (byte)61, (byte)24, (byte)13, (byte)49, (byte)139, (byte)148, (byte)215, (byte)11, (byte)91, (byte)239, (byte)50, (byte)237, (byte)122, (byte)189, (byte)243, (byte)216, (byte)103, (byte)44, (byte)5, (byte)170, (byte)72, (byte)103, (byte)31, (byte)117, (byte)17, (byte)183, (byte)59, (byte)122, (byte)103, (byte)52, (byte)71, (byte)107, (byte)232, (byte)108, (byte)149, (byte)253, (byte)130, (byte)181, (byte)86, (byte)231, (byte)239, (byte)190, (byte)54, (byte)53, (byte)186, (byte)100}));
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.len = (byte)(byte)242;
            p233.data__SET(new byte[] {(byte)23, (byte)62, (byte)200, (byte)144, (byte)8, (byte)75, (byte)249, (byte)250, (byte)40, (byte)229, (byte)217, (byte)143, (byte)240, (byte)20, (byte)174, (byte)134, (byte)129, (byte)193, (byte)154, (byte)203, (byte)74, (byte)89, (byte)201, (byte)101, (byte)117, (byte)122, (byte)180, (byte)14, (byte)215, (byte)33, (byte)146, (byte)218, (byte)77, (byte)185, (byte)100, (byte)54, (byte)210, (byte)191, (byte)234, (byte)72, (byte)157, (byte)20, (byte)65, (byte)62, (byte)52, (byte)0, (byte)55, (byte)223, (byte)207, (byte)118, (byte)201, (byte)216, (byte)67, (byte)77, (byte)151, (byte)13, (byte)35, (byte)132, (byte)130, (byte)181, (byte)17, (byte)135, (byte)182, (byte)2, (byte)203, (byte)135, (byte)238, (byte)155, (byte)163, (byte)251, (byte)184, (byte)47, (byte)188, (byte)41, (byte)67, (byte)250, (byte)142, (byte)98, (byte)251, (byte)75, (byte)109, (byte)152, (byte)208, (byte)225, (byte)166, (byte)97, (byte)211, (byte)160, (byte)244, (byte)229, (byte)194, (byte)234, (byte)36, (byte)193, (byte)3, (byte)142, (byte)222, (byte)90, (byte)228, (byte)54, (byte)119, (byte)70, (byte)117, (byte)193, (byte)32, (byte)37, (byte)66, (byte)180, (byte)16, (byte)184, (byte)245, (byte)242, (byte)38, (byte)237, (byte)76, (byte)7, (byte)68, (byte)132, (byte)48, (byte)123, (byte)28, (byte)249, (byte)26, (byte)69, (byte)112, (byte)228, (byte)57, (byte)162, (byte)218, (byte)68, (byte)250, (byte)50, (byte)126, (byte)200, (byte)61, (byte)24, (byte)13, (byte)49, (byte)139, (byte)148, (byte)215, (byte)11, (byte)91, (byte)239, (byte)50, (byte)237, (byte)122, (byte)189, (byte)243, (byte)216, (byte)103, (byte)44, (byte)5, (byte)170, (byte)72, (byte)103, (byte)31, (byte)117, (byte)17, (byte)183, (byte)59, (byte)122, (byte)103, (byte)52, (byte)71, (byte)107, (byte)232, (byte)108, (byte)149, (byte)253, (byte)130, (byte)181, (byte)86, (byte)231, (byte)239, (byte)190, (byte)54, (byte)53, (byte)186, (byte)100}, 0) ;
            p233.flags = (byte)(byte)17;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.heading_sp == (short)(short)4935);
                Debug.Assert(pack.failsafe == (byte)(byte)151);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)14945);
                Debug.Assert(pack.altitude_amsl == (short)(short)10979);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)64);
                Debug.Assert(pack.temperature == (sbyte)(sbyte)88);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED));
                Debug.Assert(pack.pitch == (short)(short) -31572);
                Debug.Assert(pack.battery_remaining == (byte)(byte)243);
                Debug.Assert(pack.altitude_sp == (short)(short)17648);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)45);
                Debug.Assert(pack.latitude == (int) -1818862564);
                Debug.Assert(pack.roll == (short)(short)24457);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
                Debug.Assert(pack.airspeed == (byte)(byte)4);
                Debug.Assert(pack.gps_nsat == (byte)(byte)38);
                Debug.Assert(pack.longitude == (int)2135721299);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)28);
                Debug.Assert(pack.gps_fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 74);
                Debug.Assert(pack.groundspeed == (byte)(byte)47);
                Debug.Assert(pack.heading == (ushort)(ushort)44780);
                Debug.Assert(pack.custom_mode == (uint)3355388318U);
                Debug.Assert(pack.wp_num == (byte)(byte)118);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND;
            p234.temperature = (sbyte)(sbyte)88;
            p234.airspeed_sp = (byte)(byte)45;
            p234.battery_remaining = (byte)(byte)243;
            p234.roll = (short)(short)24457;
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED);
            p234.latitude = (int) -1818862564;
            p234.wp_distance = (ushort)(ushort)14945;
            p234.custom_mode = (uint)3355388318U;
            p234.throttle = (sbyte)(sbyte)28;
            p234.wp_num = (byte)(byte)118;
            p234.longitude = (int)2135721299;
            p234.heading_sp = (short)(short)4935;
            p234.altitude_sp = (short)(short)17648;
            p234.climb_rate = (sbyte)(sbyte) - 74;
            p234.temperature_air = (sbyte)(sbyte)64;
            p234.failsafe = (byte)(byte)151;
            p234.airspeed = (byte)(byte)4;
            p234.pitch = (short)(short) -31572;
            p234.heading = (ushort)(ushort)44780;
            p234.groundspeed = (byte)(byte)47;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC;
            p234.gps_nsat = (byte)(byte)38;
            p234.altitude_amsl = (short)(short)10979;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vibration_x == (float)2.178309E38F);
                Debug.Assert(pack.vibration_z == (float) -9.974794E37F);
                Debug.Assert(pack.vibration_y == (float) -7.5513916E37F);
                Debug.Assert(pack.clipping_2 == (uint)2391524750U);
                Debug.Assert(pack.clipping_1 == (uint)4089102332U);
                Debug.Assert(pack.time_usec == (ulong)2716485800322856973L);
                Debug.Assert(pack.clipping_0 == (uint)2611190760U);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.clipping_0 = (uint)2611190760U;
            p241.vibration_z = (float) -9.974794E37F;
            p241.vibration_x = (float)2.178309E38F;
            p241.vibration_y = (float) -7.5513916E37F;
            p241.clipping_2 = (uint)2391524750U;
            p241.time_usec = (ulong)2716485800322856973L;
            p241.clipping_1 = (uint)4089102332U;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.approach_z == (float) -1.807974E38F);
                Debug.Assert(pack.latitude == (int)941537995);
                Debug.Assert(pack.y == (float)2.8359465E38F);
                Debug.Assert(pack.z == (float) -3.0182169E38F);
                Debug.Assert(pack.x == (float) -1.0735581E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7905251363439504466L);
                Debug.Assert(pack.approach_y == (float)2.5512935E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-8.865906E37F, -3.3669593E38F, 1.3207945E38F, 2.6564227E38F}));
                Debug.Assert(pack.approach_x == (float) -2.2833213E38F);
                Debug.Assert(pack.altitude == (int) -1851520241);
                Debug.Assert(pack.longitude == (int)2003758918);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int)941537995;
            p242.altitude = (int) -1851520241;
            p242.z = (float) -3.0182169E38F;
            p242.y = (float)2.8359465E38F;
            p242.approach_y = (float)2.5512935E38F;
            p242.q_SET(new float[] {-8.865906E37F, -3.3669593E38F, 1.3207945E38F, 2.6564227E38F}, 0) ;
            p242.approach_x = (float) -2.2833213E38F;
            p242.approach_z = (float) -1.807974E38F;
            p242.longitude = (int)2003758918;
            p242.x = (float) -1.0735581E38F;
            p242.time_usec_SET((ulong)7905251363439504466L, PH) ;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.5384024E38F, 2.2424425E38F, 2.4591667E38F, -2.2855475E38F}));
                Debug.Assert(pack.y == (float)5.5582524E37F);
                Debug.Assert(pack.approach_y == (float) -2.1726124E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)208148633650136142L);
                Debug.Assert(pack.x == (float)2.3414882E38F);
                Debug.Assert(pack.latitude == (int)1600409005);
                Debug.Assert(pack.z == (float) -1.7988763E38F);
                Debug.Assert(pack.approach_z == (float)1.783944E38F);
                Debug.Assert(pack.target_system == (byte)(byte)211);
                Debug.Assert(pack.altitude == (int) -250107837);
                Debug.Assert(pack.longitude == (int)76477790);
                Debug.Assert(pack.approach_x == (float) -7.507945E37F);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.z = (float) -1.7988763E38F;
            p243.target_system = (byte)(byte)211;
            p243.altitude = (int) -250107837;
            p243.longitude = (int)76477790;
            p243.approach_y = (float) -2.1726124E38F;
            p243.approach_z = (float)1.783944E38F;
            p243.x = (float)2.3414882E38F;
            p243.approach_x = (float) -7.507945E37F;
            p243.time_usec_SET((ulong)208148633650136142L, PH) ;
            p243.latitude = (int)1600409005;
            p243.y = (float)5.5582524E37F;
            p243.q_SET(new float[] {2.5384024E38F, 2.2424425E38F, 2.4591667E38F, -2.2855475E38F}, 0) ;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.interval_us == (int) -1642818892);
                Debug.Assert(pack.message_id == (ushort)(ushort)15743);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)15743;
            p244.interval_us = (int) -1642818892;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND;
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_FW;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1580028611);
                Debug.Assert(pack.callsign_LEN(ph) == 5);
                Debug.Assert(pack.callsign_TRY(ph).Equals("toiuc"));
                Debug.Assert(pack.ver_velocity == (short)(short) -21993);
                Debug.Assert(pack.ICAO_address == (uint)666187178U);
                Debug.Assert(pack.tslc == (byte)(byte)223);
                Debug.Assert(pack.altitude_type == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)35657);
                Debug.Assert(pack.emitter_type == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ULTRA_LIGHT);
                Debug.Assert(pack.squawk == (ushort)(ushort)22350);
                Debug.Assert(pack.heading == (ushort)(ushort)61282);
                Debug.Assert(pack.flags == (ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE));
                Debug.Assert(pack.lon == (int) -900811281);
                Debug.Assert(pack.altitude == (int) -1467811106);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.squawk = (ushort)(ushort)22350;
            p246.heading = (ushort)(ushort)61282;
            p246.callsign_SET("toiuc", PH) ;
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ULTRA_LIGHT;
            p246.lon = (int) -900811281;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE);
            p246.ver_velocity = (short)(short) -21993;
            p246.ICAO_address = (uint)666187178U;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.hor_velocity = (ushort)(ushort)35657;
            p246.lat = (int)1580028611;
            p246.tslc = (byte)(byte)223;
            p246.altitude = (int) -1467811106;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.action == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT);
                Debug.Assert(pack.src_ == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
                Debug.Assert(pack.time_to_minimum_delta == (float) -2.9490417E38F);
                Debug.Assert(pack.horizontal_minimum_delta == (float) -1.8355232E38F);
                Debug.Assert(pack.id == (uint)3601393793U);
                Debug.Assert(pack.altitude_minimum_delta == (float)2.7352219E37F);
                Debug.Assert(pack.threat_level == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.altitude_minimum_delta = (float)2.7352219E37F;
            p247.time_to_minimum_delta = (float) -2.9490417E38F;
            p247.id = (uint)3601393793U;
            p247.horizontal_minimum_delta = (float) -1.8355232E38F;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT;
            p247.threat_level = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)76, (byte)40, (byte)238, (byte)123, (byte)224, (byte)248, (byte)252, (byte)189, (byte)3, (byte)190, (byte)110, (byte)48, (byte)18, (byte)216, (byte)121, (byte)64, (byte)19, (byte)199, (byte)129, (byte)249, (byte)126, (byte)146, (byte)173, (byte)229, (byte)116, (byte)242, (byte)101, (byte)1, (byte)48, (byte)197, (byte)16, (byte)139, (byte)46, (byte)5, (byte)177, (byte)199, (byte)72, (byte)51, (byte)72, (byte)142, (byte)26, (byte)232, (byte)211, (byte)2, (byte)105, (byte)23, (byte)139, (byte)75, (byte)206, (byte)66, (byte)80, (byte)101, (byte)176, (byte)40, (byte)198, (byte)6, (byte)44, (byte)176, (byte)175, (byte)191, (byte)17, (byte)239, (byte)69, (byte)211, (byte)69, (byte)126, (byte)255, (byte)91, (byte)205, (byte)48, (byte)233, (byte)216, (byte)142, (byte)68, (byte)105, (byte)20, (byte)22, (byte)214, (byte)213, (byte)214, (byte)208, (byte)245, (byte)25, (byte)133, (byte)187, (byte)248, (byte)13, (byte)21, (byte)11, (byte)50, (byte)136, (byte)235, (byte)132, (byte)45, (byte)196, (byte)235, (byte)139, (byte)219, (byte)130, (byte)115, (byte)1, (byte)164, (byte)205, (byte)2, (byte)20, (byte)61, (byte)11, (byte)213, (byte)46, (byte)156, (byte)93, (byte)85, (byte)78, (byte)2, (byte)90, (byte)13, (byte)195, (byte)69, (byte)20, (byte)39, (byte)80, (byte)33, (byte)187, (byte)196, (byte)95, (byte)104, (byte)152, (byte)4, (byte)48, (byte)51, (byte)54, (byte)140, (byte)202, (byte)59, (byte)4, (byte)199, (byte)171, (byte)53, (byte)58, (byte)187, (byte)247, (byte)237, (byte)102, (byte)169, (byte)216, (byte)236, (byte)250, (byte)33, (byte)86, (byte)212, (byte)4, (byte)129, (byte)247, (byte)18, (byte)1, (byte)126, (byte)228, (byte)7, (byte)192, (byte)127, (byte)254, (byte)82, (byte)124, (byte)176, (byte)67, (byte)10, (byte)81, (byte)72, (byte)54, (byte)10, (byte)71, (byte)52, (byte)179, (byte)249, (byte)61, (byte)56, (byte)225, (byte)67, (byte)132, (byte)203, (byte)209, (byte)16, (byte)254, (byte)249, (byte)198, (byte)104, (byte)23, (byte)191, (byte)133, (byte)213, (byte)106, (byte)45, (byte)115, (byte)189, (byte)98, (byte)61, (byte)4, (byte)72, (byte)167, (byte)183, (byte)158, (byte)226, (byte)195, (byte)56, (byte)227, (byte)16, (byte)22, (byte)29, (byte)154, (byte)51, (byte)210, (byte)147, (byte)199, (byte)217, (byte)120, (byte)9, (byte)87, (byte)242, (byte)23, (byte)134, (byte)158, (byte)235, (byte)166, (byte)228, (byte)163, (byte)69, (byte)15, (byte)152, (byte)128, (byte)186, (byte)208, (byte)175, (byte)63, (byte)66, (byte)113, (byte)215, (byte)219, (byte)44, (byte)250, (byte)83, (byte)163, (byte)56, (byte)151, (byte)185, (byte)183, (byte)180, (byte)53, (byte)227, (byte)60}));
                Debug.Assert(pack.target_network == (byte)(byte)233);
                Debug.Assert(pack.message_type == (ushort)(ushort)5817);
                Debug.Assert(pack.target_system == (byte)(byte)24);
                Debug.Assert(pack.target_component == (byte)(byte)187);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_component = (byte)(byte)187;
            p248.payload_SET(new byte[] {(byte)76, (byte)40, (byte)238, (byte)123, (byte)224, (byte)248, (byte)252, (byte)189, (byte)3, (byte)190, (byte)110, (byte)48, (byte)18, (byte)216, (byte)121, (byte)64, (byte)19, (byte)199, (byte)129, (byte)249, (byte)126, (byte)146, (byte)173, (byte)229, (byte)116, (byte)242, (byte)101, (byte)1, (byte)48, (byte)197, (byte)16, (byte)139, (byte)46, (byte)5, (byte)177, (byte)199, (byte)72, (byte)51, (byte)72, (byte)142, (byte)26, (byte)232, (byte)211, (byte)2, (byte)105, (byte)23, (byte)139, (byte)75, (byte)206, (byte)66, (byte)80, (byte)101, (byte)176, (byte)40, (byte)198, (byte)6, (byte)44, (byte)176, (byte)175, (byte)191, (byte)17, (byte)239, (byte)69, (byte)211, (byte)69, (byte)126, (byte)255, (byte)91, (byte)205, (byte)48, (byte)233, (byte)216, (byte)142, (byte)68, (byte)105, (byte)20, (byte)22, (byte)214, (byte)213, (byte)214, (byte)208, (byte)245, (byte)25, (byte)133, (byte)187, (byte)248, (byte)13, (byte)21, (byte)11, (byte)50, (byte)136, (byte)235, (byte)132, (byte)45, (byte)196, (byte)235, (byte)139, (byte)219, (byte)130, (byte)115, (byte)1, (byte)164, (byte)205, (byte)2, (byte)20, (byte)61, (byte)11, (byte)213, (byte)46, (byte)156, (byte)93, (byte)85, (byte)78, (byte)2, (byte)90, (byte)13, (byte)195, (byte)69, (byte)20, (byte)39, (byte)80, (byte)33, (byte)187, (byte)196, (byte)95, (byte)104, (byte)152, (byte)4, (byte)48, (byte)51, (byte)54, (byte)140, (byte)202, (byte)59, (byte)4, (byte)199, (byte)171, (byte)53, (byte)58, (byte)187, (byte)247, (byte)237, (byte)102, (byte)169, (byte)216, (byte)236, (byte)250, (byte)33, (byte)86, (byte)212, (byte)4, (byte)129, (byte)247, (byte)18, (byte)1, (byte)126, (byte)228, (byte)7, (byte)192, (byte)127, (byte)254, (byte)82, (byte)124, (byte)176, (byte)67, (byte)10, (byte)81, (byte)72, (byte)54, (byte)10, (byte)71, (byte)52, (byte)179, (byte)249, (byte)61, (byte)56, (byte)225, (byte)67, (byte)132, (byte)203, (byte)209, (byte)16, (byte)254, (byte)249, (byte)198, (byte)104, (byte)23, (byte)191, (byte)133, (byte)213, (byte)106, (byte)45, (byte)115, (byte)189, (byte)98, (byte)61, (byte)4, (byte)72, (byte)167, (byte)183, (byte)158, (byte)226, (byte)195, (byte)56, (byte)227, (byte)16, (byte)22, (byte)29, (byte)154, (byte)51, (byte)210, (byte)147, (byte)199, (byte)217, (byte)120, (byte)9, (byte)87, (byte)242, (byte)23, (byte)134, (byte)158, (byte)235, (byte)166, (byte)228, (byte)163, (byte)69, (byte)15, (byte)152, (byte)128, (byte)186, (byte)208, (byte)175, (byte)63, (byte)66, (byte)113, (byte)215, (byte)219, (byte)44, (byte)250, (byte)83, (byte)163, (byte)56, (byte)151, (byte)185, (byte)183, (byte)180, (byte)53, (byte)227, (byte)60}, 0) ;
            p248.target_system = (byte)(byte)24;
            p248.target_network = (byte)(byte)233;
            p248.message_type = (ushort)(ushort)5817;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.address == (ushort)(ushort)21571);
                Debug.Assert(pack.type == (byte)(byte)116);
                Debug.Assert(pack.ver == (byte)(byte)192);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte)70, (sbyte) - 45, (sbyte) - 19, (sbyte) - 49, (sbyte)82, (sbyte)18, (sbyte) - 22, (sbyte)14, (sbyte) - 40, (sbyte) - 98, (sbyte) - 8, (sbyte)19, (sbyte) - 74, (sbyte) - 72, (sbyte) - 59, (sbyte)36, (sbyte)39, (sbyte) - 107, (sbyte)114, (sbyte)16, (sbyte)120, (sbyte)29, (sbyte) - 85, (sbyte) - 30, (sbyte)67, (sbyte)73, (sbyte)37, (sbyte)127, (sbyte)6, (sbyte) - 11, (sbyte) - 48, (sbyte) - 8}));
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.value_SET(new sbyte[] {(sbyte)70, (sbyte) - 45, (sbyte) - 19, (sbyte) - 49, (sbyte)82, (sbyte)18, (sbyte) - 22, (sbyte)14, (sbyte) - 40, (sbyte) - 98, (sbyte) - 8, (sbyte)19, (sbyte) - 74, (sbyte) - 72, (sbyte) - 59, (sbyte)36, (sbyte)39, (sbyte) - 107, (sbyte)114, (sbyte)16, (sbyte)120, (sbyte)29, (sbyte) - 85, (sbyte) - 30, (sbyte)67, (sbyte)73, (sbyte)37, (sbyte)127, (sbyte)6, (sbyte) - 11, (sbyte) - 48, (sbyte) - 8}, 0) ;
            p249.address = (ushort)(ushort)21571;
            p249.type = (byte)(byte)116;
            p249.ver = (byte)(byte)192;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)5855877853186006078L);
                Debug.Assert(pack.name_LEN(ph) == 7);
                Debug.Assert(pack.name_TRY(ph).Equals("qmduacs"));
                Debug.Assert(pack.y == (float)3.3384641E38F);
                Debug.Assert(pack.z == (float)2.5032846E38F);
                Debug.Assert(pack.x == (float)2.3917352E37F);
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.time_usec = (ulong)5855877853186006078L;
            p250.name_SET("qmduacs", PH) ;
            p250.y = (float)3.3384641E38F;
            p250.z = (float)2.5032846E38F;
            p250.x = (float)2.3917352E37F;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float) -7.3300015E37F);
                Debug.Assert(pack.time_boot_ms == (uint)2851420991U);
                Debug.Assert(pack.name_LEN(ph) == 6);
                Debug.Assert(pack.name_TRY(ph).Equals("epShsR"));
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.name_SET("epShsR", PH) ;
            p251.time_boot_ms = (uint)2851420991U;
            p251.value = (float) -7.3300015E37F;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 4);
                Debug.Assert(pack.name_TRY(ph).Equals("qegq"));
                Debug.Assert(pack.time_boot_ms == (uint)3606107234U);
                Debug.Assert(pack.value == (int) -1587402588);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.value = (int) -1587402588;
            p252.time_boot_ms = (uint)3606107234U;
            p252.name_SET("qegq", PH) ;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 28);
                Debug.Assert(pack.text_TRY(ph).Equals("QduygvBQxiyzxqeitgqfctilerHv"));
                Debug.Assert(pack.severity == MAV_SEVERITY.MAV_SEVERITY_WARNING);
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_WARNING;
            p253.text_SET("QduygvBQxiyzxqeitgqfctilerHv", PH) ;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ind == (byte)(byte)203);
                Debug.Assert(pack.value == (float)2.7855354E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4222221491U);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.value = (float)2.7855354E38F;
            p254.time_boot_ms = (uint)4222221491U;
            p254.ind = (byte)(byte)203;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)23);
                Debug.Assert(pack.target_component == (byte)(byte)39);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)72, (byte)28, (byte)81, (byte)82, (byte)20, (byte)48, (byte)67, (byte)24, (byte)76, (byte)88, (byte)103, (byte)34, (byte)65, (byte)178, (byte)13, (byte)64, (byte)239, (byte)116, (byte)1, (byte)220, (byte)95, (byte)51, (byte)3, (byte)19, (byte)89, (byte)39, (byte)255, (byte)240, (byte)128, (byte)79, (byte)66, (byte)45}));
                Debug.Assert(pack.initial_timestamp == (ulong)6746306062112959326L);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.secret_key_SET(new byte[] {(byte)72, (byte)28, (byte)81, (byte)82, (byte)20, (byte)48, (byte)67, (byte)24, (byte)76, (byte)88, (byte)103, (byte)34, (byte)65, (byte)178, (byte)13, (byte)64, (byte)239, (byte)116, (byte)1, (byte)220, (byte)95, (byte)51, (byte)3, (byte)19, (byte)89, (byte)39, (byte)255, (byte)240, (byte)128, (byte)79, (byte)66, (byte)45}, 0) ;
            p256.target_system = (byte)(byte)23;
            p256.target_component = (byte)(byte)39;
            p256.initial_timestamp = (ulong)6746306062112959326L;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3149884222U);
                Debug.Assert(pack.state == (byte)(byte)239);
                Debug.Assert(pack.last_change_ms == (uint)1762186546U);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)3149884222U;
            p257.last_change_ms = (uint)1762186546U;
            p257.state = (byte)(byte)239;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)170);
                Debug.Assert(pack.target_component == (byte)(byte)2);
                Debug.Assert(pack.tune_LEN(ph) == 19);
                Debug.Assert(pack.tune_TRY(ph).Equals("zxemqRjzedIrwLgxycA"));
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)170;
            p258.tune_SET("zxemqRjzedIrwLgxycA", PH) ;
            p258.target_component = (byte)(byte)2;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.firmware_version == (uint)2700589134U);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)31752);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 23);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("gtztcejbtnrouszKrniVhyb"));
                Debug.Assert(pack.focal_length == (float)2.546033E38F);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)146, (byte)80, (byte)179, (byte)31, (byte)157, (byte)5, (byte)178, (byte)199, (byte)170, (byte)95, (byte)138, (byte)149, (byte)141, (byte)172, (byte)126, (byte)108, (byte)111, (byte)129, (byte)34, (byte)61, (byte)68, (byte)196, (byte)74, (byte)183, (byte)41, (byte)139, (byte)175, (byte)53, (byte)32, (byte)185, (byte)18, (byte)53}));
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)70, (byte)189, (byte)111, (byte)169, (byte)162, (byte)248, (byte)139, (byte)52, (byte)2, (byte)129, (byte)156, (byte)177, (byte)190, (byte)196, (byte)197, (byte)205, (byte)198, (byte)19, (byte)99, (byte)106, (byte)139, (byte)201, (byte)131, (byte)213, (byte)33, (byte)250, (byte)251, (byte)104, (byte)44, (byte)66, (byte)149, (byte)131}));
                Debug.Assert(pack.sensor_size_v == (float)8.132187E37F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)12777);
                Debug.Assert(pack.lens_id == (byte)(byte)8);
                Debug.Assert(pack.sensor_size_h == (float) -5.4701446E37F);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE));
                Debug.Assert(pack.time_boot_ms == (uint)1212816216U);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)28321);
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.firmware_version = (uint)2700589134U;
            p259.cam_definition_uri_SET("gtztcejbtnrouszKrniVhyb", PH) ;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE);
            p259.resolution_h = (ushort)(ushort)12777;
            p259.sensor_size_v = (float)8.132187E37F;
            p259.focal_length = (float)2.546033E38F;
            p259.model_name_SET(new byte[] {(byte)146, (byte)80, (byte)179, (byte)31, (byte)157, (byte)5, (byte)178, (byte)199, (byte)170, (byte)95, (byte)138, (byte)149, (byte)141, (byte)172, (byte)126, (byte)108, (byte)111, (byte)129, (byte)34, (byte)61, (byte)68, (byte)196, (byte)74, (byte)183, (byte)41, (byte)139, (byte)175, (byte)53, (byte)32, (byte)185, (byte)18, (byte)53}, 0) ;
            p259.time_boot_ms = (uint)1212816216U;
            p259.cam_definition_version = (ushort)(ushort)28321;
            p259.vendor_name_SET(new byte[] {(byte)70, (byte)189, (byte)111, (byte)169, (byte)162, (byte)248, (byte)139, (byte)52, (byte)2, (byte)129, (byte)156, (byte)177, (byte)190, (byte)196, (byte)197, (byte)205, (byte)198, (byte)19, (byte)99, (byte)106, (byte)139, (byte)201, (byte)131, (byte)213, (byte)33, (byte)250, (byte)251, (byte)104, (byte)44, (byte)66, (byte)149, (byte)131}, 0) ;
            p259.lens_id = (byte)(byte)8;
            p259.sensor_size_h = (float) -5.4701446E37F;
            p259.resolution_v = (ushort)(ushort)31752;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3113243801U);
                Debug.Assert(pack.mode_id == CAMERA_MODE.CAMERA_MODE_VIDEO);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = CAMERA_MODE.CAMERA_MODE_VIDEO;
            p260.time_boot_ms = (uint)3113243801U;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.write_speed == (float) -2.348011E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2902866339U);
                Debug.Assert(pack.available_capacity == (float)2.9030452E38F);
                Debug.Assert(pack.used_capacity == (float)1.8179276E38F);
                Debug.Assert(pack.read_speed == (float) -2.3610976E38F);
                Debug.Assert(pack.storage_id == (byte)(byte)68);
                Debug.Assert(pack.status == (byte)(byte)9);
                Debug.Assert(pack.storage_count == (byte)(byte)133);
                Debug.Assert(pack.total_capacity == (float) -3.0642697E38F);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.available_capacity = (float)2.9030452E38F;
            p261.storage_count = (byte)(byte)133;
            p261.used_capacity = (float)1.8179276E38F;
            p261.write_speed = (float) -2.348011E38F;
            p261.status = (byte)(byte)9;
            p261.read_speed = (float) -2.3610976E38F;
            p261.time_boot_ms = (uint)2902866339U;
            p261.total_capacity = (float) -3.0642697E38F;
            p261.storage_id = (byte)(byte)68;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.recording_time_ms == (uint)3433223384U);
                Debug.Assert(pack.video_status == (byte)(byte)224);
                Debug.Assert(pack.available_capacity == (float) -3.4021262E38F);
                Debug.Assert(pack.image_interval == (float) -1.3493493E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3930520696U);
                Debug.Assert(pack.image_status == (byte)(byte)97);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)3930520696U;
            p262.available_capacity = (float) -3.4021262E38F;
            p262.image_interval = (float) -1.3493493E38F;
            p262.image_status = (byte)(byte)97;
            p262.recording_time_ms = (uint)3433223384U;
            p262.video_status = (byte)(byte)224;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_utc == (ulong)6476610942117833386L);
                Debug.Assert(pack.lat == (int) -133777668);
                Debug.Assert(pack.file_url_LEN(ph) == 193);
                Debug.Assert(pack.file_url_TRY(ph).Equals("aqqjhathpvgnogazogsOnbDtybQgykHfkhkkizzkbiJtndvdkjuyyfpyvejnbzvplehxtwdthklkubNwmyrxyqkkldzBjsWsaswhsyoukigaryxipehRudfxOlnfqkkvjiijccalymbmbsvFjpyAuisfdcxfqKDTrhtlfoafgoMegvwigcntjkkgwhogpjIgf"));
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)6);
                Debug.Assert(pack.time_boot_ms == (uint)3584255752U);
                Debug.Assert(pack.camera_id == (byte)(byte)194);
                Debug.Assert(pack.lon == (int) -268770957);
                Debug.Assert(pack.relative_alt == (int)958810155);
                Debug.Assert(pack.image_index == (int) -289662431);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.2565193E38F, 1.6073526E37F, -8.391806E37F, -1.622605E37F}));
                Debug.Assert(pack.alt == (int)1383793167);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.camera_id = (byte)(byte)194;
            p263.capture_result = (sbyte)(sbyte)6;
            p263.relative_alt = (int)958810155;
            p263.lon = (int) -268770957;
            p263.time_utc = (ulong)6476610942117833386L;
            p263.time_boot_ms = (uint)3584255752U;
            p263.q_SET(new float[] {-1.2565193E38F, 1.6073526E37F, -8.391806E37F, -1.622605E37F}, 0) ;
            p263.file_url_SET("aqqjhathpvgnogazogsOnbDtybQgykHfkhkkizzkbiJtndvdkjuyyfpyvejnbzvplehxtwdthklkubNwmyrxyqkkldzBjsWsaswhsyoukigaryxipehRudfxOlnfqkkvjiijccalymbmbsvFjpyAuisfdcxfqKDTrhtlfoafgoMegvwigcntjkkgwhogpjIgf", PH) ;
            p263.alt = (int)1383793167;
            p263.image_index = (int) -289662431;
            p263.lat = (int) -133777668;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.takeoff_time_utc == (ulong)1073581891388460974L);
                Debug.Assert(pack.time_boot_ms == (uint)205908307U);
                Debug.Assert(pack.flight_uuid == (ulong)6873054794188477211L);
                Debug.Assert(pack.arming_time_utc == (ulong)1506970778987469L);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.arming_time_utc = (ulong)1506970778987469L;
            p264.takeoff_time_utc = (ulong)1073581891388460974L;
            p264.flight_uuid = (ulong)6873054794188477211L;
            p264.time_boot_ms = (uint)205908307U;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -1.4862132E38F);
                Debug.Assert(pack.time_boot_ms == (uint)940217068U);
                Debug.Assert(pack.roll == (float)2.1446973E38F);
                Debug.Assert(pack.yaw == (float)6.908654E37F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.yaw = (float)6.908654E37F;
            p265.time_boot_ms = (uint)940217068U;
            p265.roll = (float)2.1446973E38F;
            p265.pitch = (float) -1.4862132E38F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sequence == (ushort)(ushort)37987);
                Debug.Assert(pack.target_component == (byte)(byte)127);
                Debug.Assert(pack.target_system == (byte)(byte)77);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)250, (byte)161, (byte)166, (byte)147, (byte)151, (byte)180, (byte)162, (byte)26, (byte)61, (byte)133, (byte)3, (byte)254, (byte)253, (byte)170, (byte)253, (byte)194, (byte)215, (byte)158, (byte)48, (byte)169, (byte)132, (byte)107, (byte)163, (byte)86, (byte)39, (byte)254, (byte)31, (byte)104, (byte)114, (byte)207, (byte)41, (byte)250, (byte)226, (byte)174, (byte)234, (byte)1, (byte)57, (byte)218, (byte)214, (byte)143, (byte)160, (byte)160, (byte)21, (byte)192, (byte)78, (byte)173, (byte)177, (byte)48, (byte)223, (byte)94, (byte)200, (byte)26, (byte)169, (byte)74, (byte)126, (byte)187, (byte)48, (byte)165, (byte)192, (byte)62, (byte)20, (byte)4, (byte)216, (byte)194, (byte)9, (byte)97, (byte)36, (byte)136, (byte)126, (byte)131, (byte)24, (byte)220, (byte)187, (byte)145, (byte)75, (byte)101, (byte)153, (byte)56, (byte)58, (byte)63, (byte)122, (byte)209, (byte)129, (byte)211, (byte)66, (byte)69, (byte)134, (byte)134, (byte)90, (byte)103, (byte)2, (byte)209, (byte)158, (byte)114, (byte)204, (byte)246, (byte)30, (byte)218, (byte)99, (byte)99, (byte)60, (byte)133, (byte)164, (byte)164, (byte)123, (byte)216, (byte)48, (byte)218, (byte)107, (byte)247, (byte)147, (byte)210, (byte)198, (byte)79, (byte)120, (byte)146, (byte)93, (byte)99, (byte)55, (byte)233, (byte)249, (byte)233, (byte)16, (byte)105, (byte)85, (byte)119, (byte)0, (byte)51, (byte)2, (byte)53, (byte)61, (byte)164, (byte)202, (byte)217, (byte)31, (byte)220, (byte)174, (byte)235, (byte)39, (byte)136, (byte)152, (byte)44, (byte)246, (byte)216, (byte)81, (byte)22, (byte)10, (byte)143, (byte)124, (byte)35, (byte)146, (byte)196, (byte)37, (byte)57, (byte)227, (byte)87, (byte)92, (byte)0, (byte)62, (byte)37, (byte)115, (byte)182, (byte)224, (byte)172, (byte)35, (byte)108, (byte)113, (byte)178, (byte)61, (byte)196, (byte)45, (byte)70, (byte)240, (byte)141, (byte)186, (byte)237, (byte)143, (byte)227, (byte)223, (byte)195, (byte)142, (byte)46, (byte)85, (byte)120, (byte)32, (byte)108, (byte)31, (byte)118, (byte)60, (byte)190, (byte)82, (byte)41, (byte)5, (byte)254, (byte)137, (byte)50, (byte)8, (byte)10, (byte)143, (byte)132, (byte)102, (byte)96, (byte)198, (byte)59, (byte)88, (byte)60, (byte)0, (byte)2, (byte)149, (byte)33, (byte)92, (byte)6, (byte)143, (byte)250, (byte)32, (byte)223, (byte)143, (byte)22, (byte)10, (byte)179, (byte)252, (byte)211, (byte)231, (byte)94, (byte)189, (byte)211, (byte)229, (byte)255, (byte)238, (byte)106, (byte)184, (byte)32, (byte)237, (byte)179, (byte)209, (byte)75, (byte)32, (byte)143, (byte)21, (byte)99, (byte)237, (byte)47, (byte)114, (byte)246, (byte)137, (byte)240, (byte)9, (byte)178, (byte)172}));
                Debug.Assert(pack.first_message_offset == (byte)(byte)210);
                Debug.Assert(pack.length == (byte)(byte)67);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_component = (byte)(byte)127;
            p266.target_system = (byte)(byte)77;
            p266.length = (byte)(byte)67;
            p266.first_message_offset = (byte)(byte)210;
            p266.data__SET(new byte[] {(byte)250, (byte)161, (byte)166, (byte)147, (byte)151, (byte)180, (byte)162, (byte)26, (byte)61, (byte)133, (byte)3, (byte)254, (byte)253, (byte)170, (byte)253, (byte)194, (byte)215, (byte)158, (byte)48, (byte)169, (byte)132, (byte)107, (byte)163, (byte)86, (byte)39, (byte)254, (byte)31, (byte)104, (byte)114, (byte)207, (byte)41, (byte)250, (byte)226, (byte)174, (byte)234, (byte)1, (byte)57, (byte)218, (byte)214, (byte)143, (byte)160, (byte)160, (byte)21, (byte)192, (byte)78, (byte)173, (byte)177, (byte)48, (byte)223, (byte)94, (byte)200, (byte)26, (byte)169, (byte)74, (byte)126, (byte)187, (byte)48, (byte)165, (byte)192, (byte)62, (byte)20, (byte)4, (byte)216, (byte)194, (byte)9, (byte)97, (byte)36, (byte)136, (byte)126, (byte)131, (byte)24, (byte)220, (byte)187, (byte)145, (byte)75, (byte)101, (byte)153, (byte)56, (byte)58, (byte)63, (byte)122, (byte)209, (byte)129, (byte)211, (byte)66, (byte)69, (byte)134, (byte)134, (byte)90, (byte)103, (byte)2, (byte)209, (byte)158, (byte)114, (byte)204, (byte)246, (byte)30, (byte)218, (byte)99, (byte)99, (byte)60, (byte)133, (byte)164, (byte)164, (byte)123, (byte)216, (byte)48, (byte)218, (byte)107, (byte)247, (byte)147, (byte)210, (byte)198, (byte)79, (byte)120, (byte)146, (byte)93, (byte)99, (byte)55, (byte)233, (byte)249, (byte)233, (byte)16, (byte)105, (byte)85, (byte)119, (byte)0, (byte)51, (byte)2, (byte)53, (byte)61, (byte)164, (byte)202, (byte)217, (byte)31, (byte)220, (byte)174, (byte)235, (byte)39, (byte)136, (byte)152, (byte)44, (byte)246, (byte)216, (byte)81, (byte)22, (byte)10, (byte)143, (byte)124, (byte)35, (byte)146, (byte)196, (byte)37, (byte)57, (byte)227, (byte)87, (byte)92, (byte)0, (byte)62, (byte)37, (byte)115, (byte)182, (byte)224, (byte)172, (byte)35, (byte)108, (byte)113, (byte)178, (byte)61, (byte)196, (byte)45, (byte)70, (byte)240, (byte)141, (byte)186, (byte)237, (byte)143, (byte)227, (byte)223, (byte)195, (byte)142, (byte)46, (byte)85, (byte)120, (byte)32, (byte)108, (byte)31, (byte)118, (byte)60, (byte)190, (byte)82, (byte)41, (byte)5, (byte)254, (byte)137, (byte)50, (byte)8, (byte)10, (byte)143, (byte)132, (byte)102, (byte)96, (byte)198, (byte)59, (byte)88, (byte)60, (byte)0, (byte)2, (byte)149, (byte)33, (byte)92, (byte)6, (byte)143, (byte)250, (byte)32, (byte)223, (byte)143, (byte)22, (byte)10, (byte)179, (byte)252, (byte)211, (byte)231, (byte)94, (byte)189, (byte)211, (byte)229, (byte)255, (byte)238, (byte)106, (byte)184, (byte)32, (byte)237, (byte)179, (byte)209, (byte)75, (byte)32, (byte)143, (byte)21, (byte)99, (byte)237, (byte)47, (byte)114, (byte)246, (byte)137, (byte)240, (byte)9, (byte)178, (byte)172}, 0) ;
            p266.sequence = (ushort)(ushort)37987;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)33);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)145, (byte)250, (byte)20, (byte)111, (byte)248, (byte)176, (byte)42, (byte)43, (byte)232, (byte)89, (byte)81, (byte)85, (byte)162, (byte)72, (byte)230, (byte)82, (byte)8, (byte)15, (byte)203, (byte)96, (byte)176, (byte)163, (byte)51, (byte)198, (byte)38, (byte)215, (byte)35, (byte)206, (byte)247, (byte)92, (byte)108, (byte)30, (byte)164, (byte)235, (byte)146, (byte)52, (byte)110, (byte)31, (byte)196, (byte)76, (byte)254, (byte)43, (byte)136, (byte)37, (byte)204, (byte)154, (byte)166, (byte)131, (byte)170, (byte)245, (byte)225, (byte)203, (byte)136, (byte)93, (byte)100, (byte)30, (byte)135, (byte)126, (byte)245, (byte)127, (byte)97, (byte)208, (byte)155, (byte)80, (byte)71, (byte)71, (byte)62, (byte)139, (byte)191, (byte)162, (byte)178, (byte)225, (byte)174, (byte)108, (byte)211, (byte)89, (byte)94, (byte)185, (byte)221, (byte)203, (byte)146, (byte)211, (byte)7, (byte)106, (byte)18, (byte)211, (byte)131, (byte)22, (byte)63, (byte)28, (byte)255, (byte)29, (byte)169, (byte)56, (byte)167, (byte)65, (byte)178, (byte)62, (byte)250, (byte)185, (byte)15, (byte)41, (byte)165, (byte)55, (byte)60, (byte)27, (byte)56, (byte)254, (byte)251, (byte)254, (byte)85, (byte)62, (byte)226, (byte)152, (byte)49, (byte)236, (byte)184, (byte)192, (byte)143, (byte)196, (byte)130, (byte)165, (byte)193, (byte)237, (byte)47, (byte)113, (byte)175, (byte)75, (byte)62, (byte)172, (byte)68, (byte)255, (byte)178, (byte)184, (byte)78, (byte)144, (byte)171, (byte)16, (byte)50, (byte)4, (byte)18, (byte)81, (byte)157, (byte)120, (byte)128, (byte)68, (byte)210, (byte)159, (byte)59, (byte)15, (byte)206, (byte)27, (byte)68, (byte)169, (byte)69, (byte)105, (byte)7, (byte)200, (byte)243, (byte)214, (byte)62, (byte)94, (byte)222, (byte)5, (byte)147, (byte)89, (byte)138, (byte)199, (byte)150, (byte)92, (byte)177, (byte)100, (byte)144, (byte)226, (byte)130, (byte)157, (byte)199, (byte)158, (byte)181, (byte)200, (byte)74, (byte)49, (byte)251, (byte)133, (byte)123, (byte)99, (byte)190, (byte)73, (byte)72, (byte)134, (byte)85, (byte)114, (byte)169, (byte)196, (byte)193, (byte)141, (byte)34, (byte)205, (byte)75, (byte)18, (byte)95, (byte)83, (byte)159, (byte)109, (byte)203, (byte)211, (byte)246, (byte)213, (byte)37, (byte)127, (byte)110, (byte)35, (byte)207, (byte)96, (byte)110, (byte)22, (byte)180, (byte)132, (byte)121, (byte)233, (byte)39, (byte)139, (byte)162, (byte)120, (byte)36, (byte)241, (byte)56, (byte)220, (byte)61, (byte)151, (byte)34, (byte)194, (byte)78, (byte)82, (byte)128, (byte)158, (byte)41, (byte)207, (byte)188, (byte)129, (byte)74, (byte)237, (byte)35, (byte)154, (byte)51, (byte)84, (byte)118, (byte)2, (byte)94}));
                Debug.Assert(pack.sequence == (ushort)(ushort)39355);
                Debug.Assert(pack.first_message_offset == (byte)(byte)38);
                Debug.Assert(pack.target_component == (byte)(byte)68);
                Debug.Assert(pack.length == (byte)(byte)128);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.data__SET(new byte[] {(byte)145, (byte)250, (byte)20, (byte)111, (byte)248, (byte)176, (byte)42, (byte)43, (byte)232, (byte)89, (byte)81, (byte)85, (byte)162, (byte)72, (byte)230, (byte)82, (byte)8, (byte)15, (byte)203, (byte)96, (byte)176, (byte)163, (byte)51, (byte)198, (byte)38, (byte)215, (byte)35, (byte)206, (byte)247, (byte)92, (byte)108, (byte)30, (byte)164, (byte)235, (byte)146, (byte)52, (byte)110, (byte)31, (byte)196, (byte)76, (byte)254, (byte)43, (byte)136, (byte)37, (byte)204, (byte)154, (byte)166, (byte)131, (byte)170, (byte)245, (byte)225, (byte)203, (byte)136, (byte)93, (byte)100, (byte)30, (byte)135, (byte)126, (byte)245, (byte)127, (byte)97, (byte)208, (byte)155, (byte)80, (byte)71, (byte)71, (byte)62, (byte)139, (byte)191, (byte)162, (byte)178, (byte)225, (byte)174, (byte)108, (byte)211, (byte)89, (byte)94, (byte)185, (byte)221, (byte)203, (byte)146, (byte)211, (byte)7, (byte)106, (byte)18, (byte)211, (byte)131, (byte)22, (byte)63, (byte)28, (byte)255, (byte)29, (byte)169, (byte)56, (byte)167, (byte)65, (byte)178, (byte)62, (byte)250, (byte)185, (byte)15, (byte)41, (byte)165, (byte)55, (byte)60, (byte)27, (byte)56, (byte)254, (byte)251, (byte)254, (byte)85, (byte)62, (byte)226, (byte)152, (byte)49, (byte)236, (byte)184, (byte)192, (byte)143, (byte)196, (byte)130, (byte)165, (byte)193, (byte)237, (byte)47, (byte)113, (byte)175, (byte)75, (byte)62, (byte)172, (byte)68, (byte)255, (byte)178, (byte)184, (byte)78, (byte)144, (byte)171, (byte)16, (byte)50, (byte)4, (byte)18, (byte)81, (byte)157, (byte)120, (byte)128, (byte)68, (byte)210, (byte)159, (byte)59, (byte)15, (byte)206, (byte)27, (byte)68, (byte)169, (byte)69, (byte)105, (byte)7, (byte)200, (byte)243, (byte)214, (byte)62, (byte)94, (byte)222, (byte)5, (byte)147, (byte)89, (byte)138, (byte)199, (byte)150, (byte)92, (byte)177, (byte)100, (byte)144, (byte)226, (byte)130, (byte)157, (byte)199, (byte)158, (byte)181, (byte)200, (byte)74, (byte)49, (byte)251, (byte)133, (byte)123, (byte)99, (byte)190, (byte)73, (byte)72, (byte)134, (byte)85, (byte)114, (byte)169, (byte)196, (byte)193, (byte)141, (byte)34, (byte)205, (byte)75, (byte)18, (byte)95, (byte)83, (byte)159, (byte)109, (byte)203, (byte)211, (byte)246, (byte)213, (byte)37, (byte)127, (byte)110, (byte)35, (byte)207, (byte)96, (byte)110, (byte)22, (byte)180, (byte)132, (byte)121, (byte)233, (byte)39, (byte)139, (byte)162, (byte)120, (byte)36, (byte)241, (byte)56, (byte)220, (byte)61, (byte)151, (byte)34, (byte)194, (byte)78, (byte)82, (byte)128, (byte)158, (byte)41, (byte)207, (byte)188, (byte)129, (byte)74, (byte)237, (byte)35, (byte)154, (byte)51, (byte)84, (byte)118, (byte)2, (byte)94}, 0) ;
            p267.first_message_offset = (byte)(byte)38;
            p267.sequence = (ushort)(ushort)39355;
            p267.target_component = (byte)(byte)68;
            p267.target_system = (byte)(byte)33;
            p267.length = (byte)(byte)128;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)197);
                Debug.Assert(pack.target_component == (byte)(byte)3);
                Debug.Assert(pack.sequence == (ushort)(ushort)52492);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_component = (byte)(byte)3;
            p268.target_system = (byte)(byte)197;
            p268.sequence = (ushort)(ushort)52492;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_v == (ushort)(ushort)41327);
                Debug.Assert(pack.camera_id == (byte)(byte)123);
                Debug.Assert(pack.rotation == (ushort)(ushort)13424);
                Debug.Assert(pack.bitrate == (uint)956710848U);
                Debug.Assert(pack.status == (byte)(byte)251);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)31299);
                Debug.Assert(pack.framerate == (float) -2.4320116E38F);
                Debug.Assert(pack.uri_LEN(ph) == 198);
                Debug.Assert(pack.uri_TRY(ph).Equals("cvflgpawyalCrCszlsmuhmfoswmjvowZuqoMPldgYkmgheugcppcuSnhvosloxrabawMclzkrarXfxpncxfepcraspnnejiyfkrmaulgtxMeieshzxdFyniuFogkcBmIHtxAwmmggxfjqahliqfnzboeeBryasoufdoxjtqhzcdfmhyyokbdrmqKpsifydebwwKoqs"));
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.resolution_h = (ushort)(ushort)31299;
            p269.camera_id = (byte)(byte)123;
            p269.rotation = (ushort)(ushort)13424;
            p269.bitrate = (uint)956710848U;
            p269.uri_SET("cvflgpawyalCrCszlsmuhmfoswmjvowZuqoMPldgYkmgheugcppcuSnhvosloxrabawMclzkrarXfxpncxfepcraspnnejiyfkrmaulgtxMeieshzxdFyniuFogkcBmIHtxAwmmggxfjqahliqfnzboeeBryasoufdoxjtqhzcdfmhyyokbdrmqKpsifydebwwKoqs", PH) ;
            p269.framerate = (float) -2.4320116E38F;
            p269.resolution_v = (ushort)(ushort)41327;
            p269.status = (byte)(byte)251;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.camera_id == (byte)(byte)191);
                Debug.Assert(pack.target_system == (byte)(byte)88);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)3623);
                Debug.Assert(pack.target_component == (byte)(byte)242);
                Debug.Assert(pack.rotation == (ushort)(ushort)48979);
                Debug.Assert(pack.framerate == (float) -1.2314998E38F);
                Debug.Assert(pack.uri_LEN(ph) == 61);
                Debug.Assert(pack.uri_TRY(ph).Equals("jqhyMqrrfmkfonfvtnvFpjJYjfwydbAiwdDEojkTUnvCvbogwwdlkdwlebprm"));
                Debug.Assert(pack.bitrate == (uint)2034532633U);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)2812);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.bitrate = (uint)2034532633U;
            p270.framerate = (float) -1.2314998E38F;
            p270.camera_id = (byte)(byte)191;
            p270.uri_SET("jqhyMqrrfmkfonfvtnvFpjJYjfwydbAiwdDEojkTUnvCvbogwwdlkdwlebprm", PH) ;
            p270.target_system = (byte)(byte)88;
            p270.resolution_v = (ushort)(ushort)2812;
            p270.target_component = (byte)(byte)242;
            p270.resolution_h = (ushort)(ushort)3623;
            p270.rotation = (ushort)(ushort)48979;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ssid_LEN(ph) == 7);
                Debug.Assert(pack.ssid_TRY(ph).Equals("bfpxhjv"));
                Debug.Assert(pack.password_LEN(ph) == 4);
                Debug.Assert(pack.password_TRY(ph).Equals("zmpc"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.password_SET("zmpc", PH) ;
            p299.ssid_SET("bfpxhjv", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.version == (ushort)(ushort)42414);
                Debug.Assert(pack.min_version == (ushort)(ushort)31380);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)225, (byte)118, (byte)55, (byte)101, (byte)27, (byte)108, (byte)81, (byte)158}));
                Debug.Assert(pack.max_version == (ushort)(ushort)36701);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)207, (byte)178, (byte)111, (byte)179, (byte)145, (byte)86, (byte)149, (byte)190}));
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.spec_version_hash_SET(new byte[] {(byte)225, (byte)118, (byte)55, (byte)101, (byte)27, (byte)108, (byte)81, (byte)158}, 0) ;
            p300.version = (ushort)(ushort)42414;
            p300.library_version_hash_SET(new byte[] {(byte)207, (byte)178, (byte)111, (byte)179, (byte)145, (byte)86, (byte)149, (byte)190}, 0) ;
            p300.min_version = (ushort)(ushort)31380;
            p300.max_version = (ushort)(ushort)36701;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)6985144005817574864L);
                Debug.Assert(pack.sub_mode == (byte)(byte)218);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)55053);
                Debug.Assert(pack.health == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
                Debug.Assert(pack.mode == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL);
                Debug.Assert(pack.uptime_sec == (uint)871275366U);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL;
            p310.vendor_specific_status_code = (ushort)(ushort)55053;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL;
            p310.time_usec = (ulong)6985144005817574864L;
            p310.uptime_sec = (uint)871275366U;
            p310.sub_mode = (byte)(byte)218;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)7269474424117104591L);
                Debug.Assert(pack.hw_version_major == (byte)(byte)38);
                Debug.Assert(pack.name_LEN(ph) == 33);
                Debug.Assert(pack.name_TRY(ph).Equals("OyouYDcitbsplncxyqxsFgKxKXyLszhnk"));
                Debug.Assert(pack.hw_version_minor == (byte)(byte)250);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)2);
                Debug.Assert(pack.sw_version_major == (byte)(byte)68);
                Debug.Assert(pack.uptime_sec == (uint)1451908952U);
                Debug.Assert(pack.sw_vcs_commit == (uint)4241497830U);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)153, (byte)125, (byte)80, (byte)93, (byte)122, (byte)174, (byte)143, (byte)232, (byte)186, (byte)221, (byte)189, (byte)11, (byte)145, (byte)229, (byte)16, (byte)241}));
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.hw_version_major = (byte)(byte)38;
            p311.sw_vcs_commit = (uint)4241497830U;
            p311.sw_version_minor = (byte)(byte)2;
            p311.sw_version_major = (byte)(byte)68;
            p311.name_SET("OyouYDcitbsplncxyqxsFgKxKXyLszhnk", PH) ;
            p311.uptime_sec = (uint)1451908952U;
            p311.hw_unique_id_SET(new byte[] {(byte)153, (byte)125, (byte)80, (byte)93, (byte)122, (byte)174, (byte)143, (byte)232, (byte)186, (byte)221, (byte)189, (byte)11, (byte)145, (byte)229, (byte)16, (byte)241}, 0) ;
            p311.hw_version_minor = (byte)(byte)250;
            p311.time_usec = (ulong)7269474424117104591L;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (short)(short) -20211);
                Debug.Assert(pack.target_component == (byte)(byte)24);
                Debug.Assert(pack.param_id_LEN(ph) == 4);
                Debug.Assert(pack.param_id_TRY(ph).Equals("inqk"));
                Debug.Assert(pack.target_system == (byte)(byte)227);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_component = (byte)(byte)24;
            p320.param_index = (short)(short) -20211;
            p320.target_system = (byte)(byte)227;
            p320.param_id_SET("inqk", PH) ;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)6);
                Debug.Assert(pack.target_component == (byte)(byte)246);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_component = (byte)(byte)246;
            p321.target_system = (byte)(byte)6;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 93);
                Debug.Assert(pack.param_value_TRY(ph).Equals("nmkTpvltdijFyvLShsndodQedrzwybwshjkkWwzwlzttjisafsdjjnoaefzpOwmermzIvvkxjfwFpungnRLililralfcu"));
                Debug.Assert(pack.param_index == (ushort)(ushort)15399);
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("bczwzxUq"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
                Debug.Assert(pack.param_count == (ushort)(ushort)4217);
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_index = (ushort)(ushort)15399;
            p322.param_value_SET("nmkTpvltdijFyvLShsndodQedrzwybwshjkkWwzwlzttjisafsdjjnoaefzpOwmermzIvvkxjfwFpungnRLililralfcu", PH) ;
            p322.param_count = (ushort)(ushort)4217;
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            p322.param_id_SET("bczwzxUq", PH) ;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("rUa"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
                Debug.Assert(pack.target_component == (byte)(byte)79);
                Debug.Assert(pack.param_value_LEN(ph) == 56);
                Debug.Assert(pack.param_value_TRY(ph).Equals("nsesKZonjzfpzmktxblyqCmitcoqhpsdeakRptJBajwtxQzcfegrfoyb"));
                Debug.Assert(pack.target_system == (byte)(byte)125);
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            p323.param_value_SET("nsesKZonjzfpzmktxblyqCmitcoqhpsdeakRptJBajwtxQzcfegrfoyb", PH) ;
            p323.target_system = (byte)(byte)125;
            p323.param_id_SET("rUa", PH) ;
            p323.target_component = (byte)(byte)79;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
                Debug.Assert(pack.param_value_LEN(ph) == 68);
                Debug.Assert(pack.param_value_TRY(ph).Equals("ftGrozgnrtGtqwvsDwvthddqdOVglprzeMmzwtecszhypbryfobbgvjgvrexmrzdfYre"));
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("Gvr"));
                Debug.Assert(pack.param_result == PARAM_ACK.PARAM_ACK_IN_PROGRESS);
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_result = PARAM_ACK.PARAM_ACK_IN_PROGRESS;
            p324.param_id_SET("Gvr", PH) ;
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32;
            p324.param_value_SET("ftGrozgnrtGtqwvsDwvthddqdOVglprzeMmzwtecszhypbryfobbgvjgvrexmrzdfYre", PH) ;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.increment == (byte)(byte)181);
                Debug.Assert(pack.sensor_type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)5097, (ushort)32230, (ushort)27473, (ushort)6189, (ushort)63035, (ushort)53223, (ushort)9773, (ushort)45167, (ushort)47573, (ushort)17150, (ushort)52859, (ushort)48357, (ushort)49549, (ushort)15508, (ushort)17229, (ushort)53106, (ushort)14625, (ushort)40589, (ushort)14544, (ushort)8666, (ushort)54269, (ushort)13242, (ushort)10371, (ushort)22015, (ushort)64097, (ushort)46442, (ushort)65391, (ushort)45781, (ushort)63722, (ushort)53924, (ushort)17615, (ushort)31570, (ushort)47123, (ushort)41402, (ushort)15420, (ushort)20231, (ushort)26238, (ushort)21047, (ushort)45845, (ushort)4212, (ushort)42545, (ushort)19207, (ushort)34809, (ushort)25874, (ushort)49872, (ushort)24773, (ushort)19655, (ushort)20935, (ushort)58745, (ushort)19895, (ushort)63469, (ushort)34245, (ushort)5780, (ushort)41503, (ushort)55639, (ushort)21707, (ushort)58924, (ushort)62778, (ushort)35487, (ushort)47114, (ushort)51659, (ushort)5814, (ushort)2561, (ushort)4000, (ushort)17476, (ushort)8077, (ushort)36991, (ushort)12362, (ushort)56823, (ushort)17617, (ushort)18799, (ushort)37582}));
                Debug.Assert(pack.min_distance == (ushort)(ushort)17384);
                Debug.Assert(pack.time_usec == (ulong)3769943202468157613L);
                Debug.Assert(pack.max_distance == (ushort)(ushort)26536);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.time_usec = (ulong)3769943202468157613L;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p330.increment = (byte)(byte)181;
            p330.max_distance = (ushort)(ushort)26536;
            p330.distances_SET(new ushort[] {(ushort)5097, (ushort)32230, (ushort)27473, (ushort)6189, (ushort)63035, (ushort)53223, (ushort)9773, (ushort)45167, (ushort)47573, (ushort)17150, (ushort)52859, (ushort)48357, (ushort)49549, (ushort)15508, (ushort)17229, (ushort)53106, (ushort)14625, (ushort)40589, (ushort)14544, (ushort)8666, (ushort)54269, (ushort)13242, (ushort)10371, (ushort)22015, (ushort)64097, (ushort)46442, (ushort)65391, (ushort)45781, (ushort)63722, (ushort)53924, (ushort)17615, (ushort)31570, (ushort)47123, (ushort)41402, (ushort)15420, (ushort)20231, (ushort)26238, (ushort)21047, (ushort)45845, (ushort)4212, (ushort)42545, (ushort)19207, (ushort)34809, (ushort)25874, (ushort)49872, (ushort)24773, (ushort)19655, (ushort)20935, (ushort)58745, (ushort)19895, (ushort)63469, (ushort)34245, (ushort)5780, (ushort)41503, (ushort)55639, (ushort)21707, (ushort)58924, (ushort)62778, (ushort)35487, (ushort)47114, (ushort)51659, (ushort)5814, (ushort)2561, (ushort)4000, (ushort)17476, (ushort)8077, (ushort)36991, (ushort)12362, (ushort)56823, (ushort)17617, (ushort)18799, (ushort)37582}, 0) ;
            p330.min_distance = (ushort)(ushort)17384;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
        }
    }
}