
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
                    ulong id = id__P(value);
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
                    ulong id = id__Q(value);
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
                    ulong id = id__Q(value);
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
                    ulong id = id__X(value);
                    BitUtils.set_bits(id, 8, data, 276);
                }
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = id__Q(value);
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
                    ulong id = id__Q(value);
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
                    ulong id = id__Q(value);
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
                    ulong id = id__Q(value);
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
                    ulong id = id__Q(value);
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
                    ulong id = id__Q(value);
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
                    ulong id = id__Q(value);
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
                    ulong id = id__X(value);
                    BitUtils.set_bits(id, 8, data, 276);
                }
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = id__Q(value);
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
                    ulong id = id__X(value);
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
                    ulong id = id__X(value);
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
                    ulong id = id__X(value);
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
                Debug.Assert(pack.mavlink_version == (byte)(byte)132);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED));
                Debug.Assert(pack.custom_mode == (uint)3114048290U);
                Debug.Assert(pack.system_status == MAV_STATE.MAV_STATE_POWEROFF);
                Debug.Assert(pack.autopilot == MAV_AUTOPILOT.MAV_AUTOPILOT_FP);
                Debug.Assert(pack.type == MAV_TYPE.MAV_TYPE_VTOL_QUADROTOR);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.mavlink_version = (byte)(byte)132;
            p0.system_status = MAV_STATE.MAV_STATE_POWEROFF;
            p0.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED);
            p0.custom_mode = (uint)3114048290U;
            p0.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_FP;
            p0.type = MAV_TYPE.MAV_TYPE_VTOL_QUADROTOR;
            ADV_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)5357);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)2593);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)86);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)35205);
                Debug.Assert(pack.load == (ushort)(ushort)39996);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)56361);
                Debug.Assert(pack.current_battery == (short)(short)32664);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)31642);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)36204);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS));
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION));
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)12639);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.errors_count1 = (ushort)(ushort)31642;
            p1.load = (ushort)(ushort)39996;
            p1.errors_count4 = (ushort)(ushort)56361;
            p1.drop_rate_comm = (ushort)(ushort)5357;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.errors_count2 = (ushort)(ushort)2593;
            p1.voltage_battery = (ushort)(ushort)35205;
            p1.errors_comm = (ushort)(ushort)36204;
            p1.current_battery = (short)(short)32664;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION);
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);
            p1.battery_remaining = (sbyte)(sbyte)86;
            p1.errors_count3 = (ushort)(ushort)12639;
            ADV_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3801169914U);
                Debug.Assert(pack.time_unix_usec == (ulong)6832316909165226784L);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_boot_ms = (uint)3801169914U;
            p2.time_unix_usec = (ulong)6832316909165226784L;
            ADV_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float)8.545855E37F);
                Debug.Assert(pack.yaw == (float)4.5484237E37F);
                Debug.Assert(pack.y == (float)2.2475895E38F);
                Debug.Assert(pack.z == (float)1.4819655E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.type_mask == (ushort)(ushort)48753);
                Debug.Assert(pack.afz == (float)1.4065835E38F);
                Debug.Assert(pack.vx == (float) -3.0756224E38F);
                Debug.Assert(pack.vz == (float)2.9941953E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3034909083U);
                Debug.Assert(pack.afy == (float)7.504443E37F);
                Debug.Assert(pack.yaw_rate == (float) -2.7231007E38F);
                Debug.Assert(pack.afx == (float)3.9620678E37F);
                Debug.Assert(pack.x == (float) -3.3881414E38F);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.y = (float)2.2475895E38F;
            p3.vx = (float) -3.0756224E38F;
            p3.afx = (float)3.9620678E37F;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_MISSION;
            p3.afz = (float)1.4065835E38F;
            p3.afy = (float)7.504443E37F;
            p3.yaw_rate = (float) -2.7231007E38F;
            p3.x = (float) -3.3881414E38F;
            p3.yaw = (float)4.5484237E37F;
            p3.time_boot_ms = (uint)3034909083U;
            p3.vz = (float)2.9941953E37F;
            p3.vy = (float)8.545855E37F;
            p3.z = (float)1.4819655E38F;
            p3.type_mask = (ushort)(ushort)48753;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)3042251231U);
                Debug.Assert(pack.target_system == (byte)(byte)161);
                Debug.Assert(pack.time_usec == (ulong)7004349849322469778L);
                Debug.Assert(pack.target_component == (byte)(byte)9);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.target_component = (byte)(byte)9;
            p4.seq = (uint)3042251231U;
            p4.time_usec = (ulong)7004349849322469778L;
            p4.target_system = (byte)(byte)161;
            ADV_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.control_request == (byte)(byte)47);
                Debug.Assert(pack.target_system == (byte)(byte)168);
                Debug.Assert(pack.passkey_LEN(ph) == 21);
                Debug.Assert(pack.passkey_TRY(ph).Equals("kfqnywpdknkimkOyvjnrv"));
                Debug.Assert(pack.version == (byte)(byte)180);
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.control_request = (byte)(byte)47;
            p5.passkey_SET("kfqnywpdknkimkOyvjnrv", PH) ;
            p5.version = (byte)(byte)180;
            p5.target_system = (byte)(byte)168;
            ADV_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ack == (byte)(byte)103);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)233);
                Debug.Assert(pack.control_request == (byte)(byte)243);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.ack = (byte)(byte)103;
            p6.control_request = (byte)(byte)243;
            p6.gcs_system_id = (byte)(byte)233;
            ADV_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 32);
                Debug.Assert(pack.key_TRY(ph).Equals("rrLgxteviUMbFzhuRhgdFfwepwNkzasQ"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("rrLgxteviUMbFzhuRhgdFfwepwNkzasQ", PH) ;
            ADV_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)199);
                Debug.Assert(pack.base_mode == MAV_MODE.MAV_MODE_AUTO_ARMED);
                Debug.Assert(pack.custom_mode == (uint)3067755287U);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)199;
            p11.custom_mode = (uint)3067755287U;
            p11.base_mode = MAV_MODE.MAV_MODE_AUTO_ARMED;
            ADV_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)144);
                Debug.Assert(pack.target_system == (byte)(byte)99);
                Debug.Assert(pack.param_id_LEN(ph) == 13);
                Debug.Assert(pack.param_id_TRY(ph).Equals("eaaaxkuOeibjS"));
                Debug.Assert(pack.param_index == (short)(short) -19496);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_component = (byte)(byte)144;
            p20.param_id_SET("eaaaxkuOeibjS", PH) ;
            p20.param_index = (short)(short) -19496;
            p20.target_system = (byte)(byte)99;
            ADV_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)97);
                Debug.Assert(pack.target_component == (byte)(byte)101);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)97;
            p21.target_component = (byte)(byte)101;
            ADV_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (ushort)(ushort)51443);
                Debug.Assert(pack.param_count == (ushort)(ushort)3568);
                Debug.Assert(pack.param_value == (float)2.5191767E38F);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32);
                Debug.Assert(pack.param_id_LEN(ph) == 12);
                Debug.Assert(pack.param_id_TRY(ph).Equals("csyanmnyvppj"));
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_index = (ushort)(ushort)51443;
            p22.param_id_SET("csyanmnyvppj", PH) ;
            p22.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32;
            p22.param_value = (float)2.5191767E38F;
            p22.param_count = (ushort)(ushort)3568;
            ADV_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
                Debug.Assert(pack.target_system == (byte)(byte)81);
                Debug.Assert(pack.param_value == (float) -4.8509556E37F);
                Debug.Assert(pack.param_id_LEN(ph) == 12);
                Debug.Assert(pack.param_id_TRY(ph).Equals("eeourrcvilQd"));
                Debug.Assert(pack.target_component == (byte)(byte)211);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.param_id_SET("eeourrcvilQd", PH) ;
            p23.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16;
            p23.target_system = (byte)(byte)81;
            p23.target_component = (byte)(byte)211;
            p23.param_value = (float) -4.8509556E37F;
            ADV_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)906905045U);
                Debug.Assert(pack.epv == (ushort)(ushort)62146);
                Debug.Assert(pack.lat == (int)1161788654);
                Debug.Assert(pack.lon == (int)117648630);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)3272288070U);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)1011539302U);
                Debug.Assert(pack.satellites_visible == (byte)(byte)201);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)4049409864U);
                Debug.Assert(pack.vel == (ushort)(ushort)8103);
                Debug.Assert(pack.eph == (ushort)(ushort)25147);
                Debug.Assert(pack.alt == (int) -1950361546);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -1967773680);
                Debug.Assert(pack.time_usec == (ulong)2275662834140614803L);
                Debug.Assert(pack.cog == (ushort)(ushort)59867);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.v_acc_SET((uint)4049409864U, PH) ;
            p24.lat = (int)1161788654;
            p24.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p24.epv = (ushort)(ushort)62146;
            p24.cog = (ushort)(ushort)59867;
            p24.time_usec = (ulong)2275662834140614803L;
            p24.lon = (int)117648630;
            p24.eph = (ushort)(ushort)25147;
            p24.alt = (int) -1950361546;
            p24.hdg_acc_SET((uint)906905045U, PH) ;
            p24.vel_acc_SET((uint)1011539302U, PH) ;
            p24.satellites_visible = (byte)(byte)201;
            p24.vel = (ushort)(ushort)8103;
            p24.h_acc_SET((uint)3272288070U, PH) ;
            p24.alt_ellipsoid_SET((int) -1967773680, PH) ;
            ADV_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)21, (byte)135, (byte)239, (byte)255, (byte)190, (byte)146, (byte)226, (byte)250, (byte)141, (byte)206, (byte)220, (byte)122, (byte)204, (byte)183, (byte)154, (byte)157, (byte)143, (byte)177, (byte)171, (byte)237}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)103, (byte)99, (byte)190, (byte)65, (byte)133, (byte)163, (byte)210, (byte)241, (byte)179, (byte)12, (byte)70, (byte)207, (byte)64, (byte)35, (byte)183, (byte)250, (byte)10, (byte)142, (byte)164, (byte)72}));
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)4, (byte)4, (byte)101, (byte)26, (byte)211, (byte)129, (byte)125, (byte)115, (byte)148, (byte)112, (byte)67, (byte)71, (byte)253, (byte)108, (byte)45, (byte)12, (byte)51, (byte)163, (byte)17, (byte)124}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)189, (byte)17, (byte)57, (byte)90, (byte)177, (byte)240, (byte)64, (byte)184, (byte)151, (byte)251, (byte)93, (byte)240, (byte)102, (byte)184, (byte)40, (byte)113, (byte)8, (byte)45, (byte)109, (byte)99}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)227, (byte)135, (byte)238, (byte)78, (byte)151, (byte)234, (byte)61, (byte)221, (byte)154, (byte)47, (byte)57, (byte)62, (byte)174, (byte)225, (byte)19, (byte)134, (byte)61, (byte)12, (byte)2, (byte)208}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)69);
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_prn_SET(new byte[] {(byte)103, (byte)99, (byte)190, (byte)65, (byte)133, (byte)163, (byte)210, (byte)241, (byte)179, (byte)12, (byte)70, (byte)207, (byte)64, (byte)35, (byte)183, (byte)250, (byte)10, (byte)142, (byte)164, (byte)72}, 0) ;
            p25.satellites_visible = (byte)(byte)69;
            p25.satellite_elevation_SET(new byte[] {(byte)4, (byte)4, (byte)101, (byte)26, (byte)211, (byte)129, (byte)125, (byte)115, (byte)148, (byte)112, (byte)67, (byte)71, (byte)253, (byte)108, (byte)45, (byte)12, (byte)51, (byte)163, (byte)17, (byte)124}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)189, (byte)17, (byte)57, (byte)90, (byte)177, (byte)240, (byte)64, (byte)184, (byte)151, (byte)251, (byte)93, (byte)240, (byte)102, (byte)184, (byte)40, (byte)113, (byte)8, (byte)45, (byte)109, (byte)99}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)21, (byte)135, (byte)239, (byte)255, (byte)190, (byte)146, (byte)226, (byte)250, (byte)141, (byte)206, (byte)220, (byte)122, (byte)204, (byte)183, (byte)154, (byte)157, (byte)143, (byte)177, (byte)171, (byte)237}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)227, (byte)135, (byte)238, (byte)78, (byte)151, (byte)234, (byte)61, (byte)221, (byte)154, (byte)47, (byte)57, (byte)62, (byte)174, (byte)225, (byte)19, (byte)134, (byte)61, (byte)12, (byte)2, (byte)208}, 0) ;
            ADV_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (short)(short)4775);
                Debug.Assert(pack.zmag == (short)(short) -32497);
                Debug.Assert(pack.zacc == (short)(short)2701);
                Debug.Assert(pack.xacc == (short)(short)6199);
                Debug.Assert(pack.yacc == (short)(short)7264);
                Debug.Assert(pack.time_boot_ms == (uint)4009405996U);
                Debug.Assert(pack.zgyro == (short)(short) -11372);
                Debug.Assert(pack.xmag == (short)(short)17108);
                Debug.Assert(pack.ymag == (short)(short) -21545);
                Debug.Assert(pack.xgyro == (short)(short) -18448);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.xacc = (short)(short)6199;
            p26.xmag = (short)(short)17108;
            p26.zmag = (short)(short) -32497;
            p26.xgyro = (short)(short) -18448;
            p26.ymag = (short)(short) -21545;
            p26.zacc = (short)(short)2701;
            p26.time_boot_ms = (uint)4009405996U;
            p26.zgyro = (short)(short) -11372;
            p26.ygyro = (short)(short)4775;
            p26.yacc = (short)(short)7264;
            ADV_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ymag == (short)(short) -31242);
                Debug.Assert(pack.yacc == (short)(short)4357);
                Debug.Assert(pack.xgyro == (short)(short)9824);
                Debug.Assert(pack.ygyro == (short)(short)16950);
                Debug.Assert(pack.xacc == (short)(short)3823);
                Debug.Assert(pack.zgyro == (short)(short) -27844);
                Debug.Assert(pack.time_usec == (ulong)3120744835027884410L);
                Debug.Assert(pack.xmag == (short)(short)20832);
                Debug.Assert(pack.zmag == (short)(short) -28789);
                Debug.Assert(pack.zacc == (short)(short) -6558);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.zacc = (short)(short) -6558;
            p27.zgyro = (short)(short) -27844;
            p27.ygyro = (short)(short)16950;
            p27.xacc = (short)(short)3823;
            p27.time_usec = (ulong)3120744835027884410L;
            p27.zmag = (short)(short) -28789;
            p27.xmag = (short)(short)20832;
            p27.ymag = (short)(short) -31242;
            p27.xgyro = (short)(short)9824;
            p27.yacc = (short)(short)4357;
            ADV_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff1 == (short)(short) -2511);
                Debug.Assert(pack.temperature == (short)(short) -4771);
                Debug.Assert(pack.press_abs == (short)(short) -2526);
                Debug.Assert(pack.time_usec == (ulong)5196047067437756558L);
                Debug.Assert(pack.press_diff2 == (short)(short) -21417);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_diff1 = (short)(short) -2511;
            p28.temperature = (short)(short) -4771;
            p28.press_abs = (short)(short) -2526;
            p28.time_usec = (ulong)5196047067437756558L;
            p28.press_diff2 = (short)(short) -21417;
            ADV_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float) -4.423121E37F);
                Debug.Assert(pack.temperature == (short)(short) -10790);
                Debug.Assert(pack.time_boot_ms == (uint)3963756996U);
                Debug.Assert(pack.press_diff == (float)9.569638E37F);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.press_abs = (float) -4.423121E37F;
            p29.temperature = (short)(short) -10790;
            p29.press_diff = (float)9.569638E37F;
            p29.time_boot_ms = (uint)3963756996U;
            ADV_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float) -3.3679349E38F);
                Debug.Assert(pack.yawspeed == (float)1.1858872E38F);
                Debug.Assert(pack.pitch == (float) -8.714111E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3780753233U);
                Debug.Assert(pack.roll == (float)1.8167226E38F);
                Debug.Assert(pack.pitchspeed == (float)3.1787487E38F);
                Debug.Assert(pack.yaw == (float)1.1178591E38F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.pitch = (float) -8.714111E37F;
            p30.yaw = (float)1.1178591E38F;
            p30.rollspeed = (float) -3.3679349E38F;
            p30.yawspeed = (float)1.1858872E38F;
            p30.time_boot_ms = (uint)3780753233U;
            p30.pitchspeed = (float)3.1787487E38F;
            p30.roll = (float)1.8167226E38F;
            ADV_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q3 == (float) -1.5968005E38F);
                Debug.Assert(pack.pitchspeed == (float)1.1354479E38F);
                Debug.Assert(pack.q2 == (float)1.174559E38F);
                Debug.Assert(pack.q1 == (float) -2.9480604E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2953182570U);
                Debug.Assert(pack.rollspeed == (float) -2.4134285E38F);
                Debug.Assert(pack.yawspeed == (float)1.7581472E38F);
                Debug.Assert(pack.q4 == (float)3.1604602E38F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.q1 = (float) -2.9480604E38F;
            p31.time_boot_ms = (uint)2953182570U;
            p31.pitchspeed = (float)1.1354479E38F;
            p31.q4 = (float)3.1604602E38F;
            p31.q3 = (float) -1.5968005E38F;
            p31.q2 = (float)1.174559E38F;
            p31.rollspeed = (float) -2.4134285E38F;
            p31.yawspeed = (float)1.7581472E38F;
            ADV_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3074569518U);
                Debug.Assert(pack.y == (float)7.169172E37F);
                Debug.Assert(pack.vx == (float) -3.01101E38F);
                Debug.Assert(pack.z == (float)1.0340599E38F);
                Debug.Assert(pack.vy == (float) -2.420008E38F);
                Debug.Assert(pack.vz == (float) -2.6234583E38F);
                Debug.Assert(pack.x == (float) -1.1019282E36F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.vz = (float) -2.6234583E38F;
            p32.y = (float)7.169172E37F;
            p32.vx = (float) -3.01101E38F;
            p32.vy = (float) -2.420008E38F;
            p32.z = (float)1.0340599E38F;
            p32.time_boot_ms = (uint)3074569518U;
            p32.x = (float) -1.1019282E36F;
            ADV_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hdg == (ushort)(ushort)30411);
                Debug.Assert(pack.lon == (int) -1726714588);
                Debug.Assert(pack.lat == (int) -2059125362);
                Debug.Assert(pack.relative_alt == (int) -336808940);
                Debug.Assert(pack.vy == (short)(short) -17806);
                Debug.Assert(pack.alt == (int) -2043005769);
                Debug.Assert(pack.time_boot_ms == (uint)1338716173U);
                Debug.Assert(pack.vz == (short)(short)23290);
                Debug.Assert(pack.vx == (short)(short) -22864);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.vx = (short)(short) -22864;
            p33.time_boot_ms = (uint)1338716173U;
            p33.vy = (short)(short) -17806;
            p33.relative_alt = (int) -336808940;
            p33.lat = (int) -2059125362;
            p33.lon = (int) -1726714588;
            p33.hdg = (ushort)(ushort)30411;
            p33.alt = (int) -2043005769;
            p33.vz = (short)(short)23290;
            ADV_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan1_scaled == (short)(short)31163);
                Debug.Assert(pack.rssi == (byte)(byte)87);
                Debug.Assert(pack.chan2_scaled == (short)(short) -23219);
                Debug.Assert(pack.port == (byte)(byte)92);
                Debug.Assert(pack.chan8_scaled == (short)(short) -18128);
                Debug.Assert(pack.chan5_scaled == (short)(short)28135);
                Debug.Assert(pack.chan6_scaled == (short)(short) -1883);
                Debug.Assert(pack.time_boot_ms == (uint)2518961399U);
                Debug.Assert(pack.chan3_scaled == (short)(short)26208);
                Debug.Assert(pack.chan4_scaled == (short)(short) -13144);
                Debug.Assert(pack.chan7_scaled == (short)(short) -6572);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.port = (byte)(byte)92;
            p34.chan3_scaled = (short)(short)26208;
            p34.chan7_scaled = (short)(short) -6572;
            p34.time_boot_ms = (uint)2518961399U;
            p34.chan8_scaled = (short)(short) -18128;
            p34.chan4_scaled = (short)(short) -13144;
            p34.chan2_scaled = (short)(short) -23219;
            p34.chan5_scaled = (short)(short)28135;
            p34.chan1_scaled = (short)(short)31163;
            p34.chan6_scaled = (short)(short) -1883;
            p34.rssi = (byte)(byte)87;
            ADV_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.port == (byte)(byte)244);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)45398);
                Debug.Assert(pack.time_boot_ms == (uint)908807311U);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)17956);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)38740);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)32143);
                Debug.Assert(pack.rssi == (byte)(byte)86);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)17311);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)40224);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)30226);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)53874);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.rssi = (byte)(byte)86;
            p35.port = (byte)(byte)244;
            p35.chan6_raw = (ushort)(ushort)32143;
            p35.chan2_raw = (ushort)(ushort)53874;
            p35.time_boot_ms = (uint)908807311U;
            p35.chan1_raw = (ushort)(ushort)30226;
            p35.chan3_raw = (ushort)(ushort)40224;
            p35.chan7_raw = (ushort)(ushort)38740;
            p35.chan5_raw = (ushort)(ushort)45398;
            p35.chan8_raw = (ushort)(ushort)17956;
            p35.chan4_raw = (ushort)(ushort)17311;
            ADV_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.port == (byte)(byte)212);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)34254);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)43988);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)41927);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)40090);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)61110);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)47448);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)41657);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)19076);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)2380);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)40320);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)32752);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)14520);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)7173);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)58210);
                Debug.Assert(pack.time_usec == (uint)2323633222U);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)50544);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)34170);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo2_raw = (ushort)(ushort)40090;
            p36.servo6_raw = (ushort)(ushort)40320;
            p36.servo5_raw = (ushort)(ushort)50544;
            p36.servo4_raw = (ushort)(ushort)19076;
            p36.port = (byte)(byte)212;
            p36.servo14_raw_SET((ushort)(ushort)14520, PH) ;
            p36.servo16_raw_SET((ushort)(ushort)34254, PH) ;
            p36.time_usec = (uint)2323633222U;
            p36.servo7_raw = (ushort)(ushort)2380;
            p36.servo8_raw = (ushort)(ushort)34170;
            p36.servo1_raw = (ushort)(ushort)47448;
            p36.servo13_raw_SET((ushort)(ushort)41927, PH) ;
            p36.servo11_raw_SET((ushort)(ushort)7173, PH) ;
            p36.servo3_raw = (ushort)(ushort)58210;
            p36.servo9_raw_SET((ushort)(ushort)32752, PH) ;
            p36.servo12_raw_SET((ushort)(ushort)61110, PH) ;
            p36.servo15_raw_SET((ushort)(ushort)43988, PH) ;
            p36.servo10_raw_SET((ushort)(ushort)41657, PH) ;
            ADV_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end_index == (short)(short) -14576);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_component == (byte)(byte)54);
                Debug.Assert(pack.start_index == (short)(short)12564);
                Debug.Assert(pack.target_system == (byte)(byte)193);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.end_index = (short)(short) -14576;
            p37.start_index = (short)(short)12564;
            p37.target_system = (byte)(byte)193;
            p37.target_component = (byte)(byte)54;
            p37.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            ADV_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_index == (short)(short)22878);
                Debug.Assert(pack.target_component == (byte)(byte)170);
                Debug.Assert(pack.target_system == (byte)(byte)17);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.end_index == (short)(short)10630);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.end_index = (short)(short)10630;
            p38.start_index = (short)(short)22878;
            p38.target_system = (byte)(byte)17;
            p38.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p38.target_component = (byte)(byte)170;
            ADV_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param4 == (float) -8.84482E37F);
                Debug.Assert(pack.target_component == (byte)(byte)17);
                Debug.Assert(pack.seq == (ushort)(ushort)51269);
                Debug.Assert(pack.param2 == (float) -1.8072266E38F);
                Debug.Assert(pack.z == (float)1.7073517E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)28);
                Debug.Assert(pack.target_system == (byte)(byte)103);
                Debug.Assert(pack.current == (byte)(byte)33);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.param3 == (float)1.8395614E38F);
                Debug.Assert(pack.param1 == (float)2.294739E38F);
                Debug.Assert(pack.y == (float)8.795664E37F);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE);
                Debug.Assert(pack.x == (float)2.1005447E38F);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.x = (float)2.1005447E38F;
            p39.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p39.command = MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE;
            p39.param2 = (float) -1.8072266E38F;
            p39.param4 = (float) -8.84482E37F;
            p39.param1 = (float)2.294739E38F;
            p39.target_system = (byte)(byte)103;
            p39.autocontinue = (byte)(byte)28;
            p39.param3 = (float)1.8395614E38F;
            p39.seq = (ushort)(ushort)51269;
            p39.z = (float)1.7073517E38F;
            p39.y = (float)8.795664E37F;
            p39.current = (byte)(byte)33;
            p39.target_component = (byte)(byte)17;
            p39.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            ADV_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_component == (byte)(byte)243);
                Debug.Assert(pack.target_system == (byte)(byte)141);
                Debug.Assert(pack.seq == (ushort)(ushort)9164);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_component = (byte)(byte)243;
            p40.seq = (ushort)(ushort)9164;
            p40.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p40.target_system = (byte)(byte)141;
            ADV_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)12217);
                Debug.Assert(pack.target_system == (byte)(byte)114);
                Debug.Assert(pack.target_component == (byte)(byte)34);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_system = (byte)(byte)114;
            p41.seq = (ushort)(ushort)12217;
            p41.target_component = (byte)(byte)34;
            ADV_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)7215);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)7215;
            ADV_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)179);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)3);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p43.target_component = (byte)(byte)3;
            p43.target_system = (byte)(byte)179;
            ADV_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)101);
                Debug.Assert(pack.target_component == (byte)(byte)247);
                Debug.Assert(pack.count == (ushort)(ushort)24562);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.target_system = (byte)(byte)101;
            p44.target_component = (byte)(byte)247;
            p44.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p44.count = (ushort)(ushort)24562;
            ADV_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)136);
                Debug.Assert(pack.target_system == (byte)(byte)17);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p45.target_component = (byte)(byte)136;
            p45.target_system = (byte)(byte)17;
            ADV_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)33454);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)33454;
            ADV_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == MAV_MISSION_RESULT.MAV_MISSION_DENIED);
                Debug.Assert(pack.target_component == (byte)(byte)28);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_system == (byte)(byte)182);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p47.target_component = (byte)(byte)28;
            p47.target_system = (byte)(byte)182;
            p47.type = MAV_MISSION_RESULT.MAV_MISSION_DENIED;
            ADV_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)6356418742685607219L);
                Debug.Assert(pack.longitude == (int) -507646601);
                Debug.Assert(pack.target_system == (byte)(byte)96);
                Debug.Assert(pack.latitude == (int)852152468);
                Debug.Assert(pack.altitude == (int) -1616382781);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.altitude = (int) -1616382781;
            p48.latitude = (int)852152468;
            p48.time_usec_SET((ulong)6356418742685607219L, PH) ;
            p48.target_system = (byte)(byte)96;
            p48.longitude = (int) -507646601;
            ADV_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (int) -444170786);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7645711108956261165L);
                Debug.Assert(pack.latitude == (int) -94119251);
                Debug.Assert(pack.altitude == (int)188361836);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.longitude = (int) -444170786;
            p49.latitude = (int) -94119251;
            p49.altitude = (int)188361836;
            p49.time_usec_SET((ulong)7645711108956261165L, PH) ;
            ADV_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)4);
                Debug.Assert(pack.target_system == (byte)(byte)200);
                Debug.Assert(pack.param_value_min == (float) -2.0365537E38F);
                Debug.Assert(pack.param_value0 == (float)2.1403246E38F);
                Debug.Assert(pack.param_index == (short)(short) -24839);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)227);
                Debug.Assert(pack.scale == (float)2.1662964E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 9);
                Debug.Assert(pack.param_id_TRY(ph).Equals("elvtgeBuv"));
                Debug.Assert(pack.param_value_max == (float)1.7509082E38F);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.parameter_rc_channel_index = (byte)(byte)227;
            p50.param_id_SET("elvtgeBuv", PH) ;
            p50.target_system = (byte)(byte)200;
            p50.param_value_max = (float)1.7509082E38F;
            p50.scale = (float)2.1662964E38F;
            p50.param_value0 = (float)2.1403246E38F;
            p50.param_index = (short)(short) -24839;
            p50.target_component = (byte)(byte)4;
            p50.param_value_min = (float) -2.0365537E38F;
            ADV_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)237);
                Debug.Assert(pack.seq == (ushort)(ushort)20626);
                Debug.Assert(pack.target_system == (byte)(byte)100);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p51.target_component = (byte)(byte)237;
            p51.target_system = (byte)(byte)100;
            p51.seq = (ushort)(ushort)20626;
            ADV_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2x == (float) -1.3954105E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.target_system == (byte)(byte)20);
                Debug.Assert(pack.p1z == (float)1.8518586E38F);
                Debug.Assert(pack.p2z == (float) -1.1325597E37F);
                Debug.Assert(pack.p1y == (float)2.1595233E38F);
                Debug.Assert(pack.target_component == (byte)(byte)143);
                Debug.Assert(pack.p1x == (float)1.8617601E38F);
                Debug.Assert(pack.p2y == (float)4.0628727E37F);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p2y = (float)4.0628727E37F;
            p54.p1z = (float)1.8518586E38F;
            p54.p1y = (float)2.1595233E38F;
            p54.frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p54.p2x = (float) -1.3954105E38F;
            p54.target_system = (byte)(byte)20;
            p54.target_component = (byte)(byte)143;
            p54.p1x = (float)1.8617601E38F;
            p54.p2z = (float) -1.1325597E37F;
            ADV_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.p2x == (float) -2.9935567E38F);
                Debug.Assert(pack.p1y == (float)3.3234756E38F);
                Debug.Assert(pack.p2y == (float) -5.749409E37F);
                Debug.Assert(pack.p2z == (float)8.653006E37F);
                Debug.Assert(pack.p1x == (float)2.2113268E38F);
                Debug.Assert(pack.p1z == (float) -1.7711632E38F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p1y = (float)3.3234756E38F;
            p55.p1x = (float)2.2113268E38F;
            p55.p2z = (float)8.653006E37F;
            p55.p1z = (float) -1.7711632E38F;
            p55.frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p55.p2x = (float) -2.9935567E38F;
            p55.p2y = (float) -5.749409E37F;
            ADV_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.4845416E38F, -3.268262E38F, 2.1505145E38F, -2.476784E37F}));
                Debug.Assert(pack.yawspeed == (float)3.297833E38F);
                Debug.Assert(pack.time_usec == (ulong)226282271792254629L);
                Debug.Assert(pack.rollspeed == (float) -2.7201458E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.9437595E38F, -3.2687983E38F, 1.5656001E38F, -2.4495234E38F, -8.922687E37F, -9.917495E37F, 7.3359883E37F, -2.6862212E38F, 8.819404E37F}));
                Debug.Assert(pack.pitchspeed == (float)1.2359829E38F);
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.time_usec = (ulong)226282271792254629L;
            p61.q_SET(new float[] {-1.4845416E38F, -3.268262E38F, 2.1505145E38F, -2.476784E37F}, 0) ;
            p61.rollspeed = (float) -2.7201458E38F;
            p61.yawspeed = (float)3.297833E38F;
            p61.pitchspeed = (float)1.2359829E38F;
            p61.covariance_SET(new float[] {-2.9437595E38F, -3.2687983E38F, 1.5656001E38F, -2.4495234E38F, -8.922687E37F, -9.917495E37F, 7.3359883E37F, -2.6862212E38F, 8.819404E37F}, 0) ;
            ADV_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wp_dist == (ushort)(ushort)394);
                Debug.Assert(pack.target_bearing == (short)(short) -768);
                Debug.Assert(pack.xtrack_error == (float)8.0018157E37F);
                Debug.Assert(pack.aspd_error == (float)2.79198E38F);
                Debug.Assert(pack.alt_error == (float)5.6707087E37F);
                Debug.Assert(pack.nav_pitch == (float)1.1697167E37F);
                Debug.Assert(pack.nav_roll == (float)4.5839767E37F);
                Debug.Assert(pack.nav_bearing == (short)(short) -17402);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_roll = (float)4.5839767E37F;
            p62.aspd_error = (float)2.79198E38F;
            p62.nav_bearing = (short)(short) -17402;
            p62.wp_dist = (ushort)(ushort)394;
            p62.target_bearing = (short)(short) -768;
            p62.alt_error = (float)5.6707087E37F;
            p62.xtrack_error = (float)8.0018157E37F;
            p62.nav_pitch = (float)1.1697167E37F;
            ADV_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float)2.5082435E38F);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
                Debug.Assert(pack.relative_alt == (int) -1247298723);
                Debug.Assert(pack.lon == (int) -604228520);
                Debug.Assert(pack.vy == (float)2.1502693E38F);
                Debug.Assert(pack.vx == (float) -2.0477088E38F);
                Debug.Assert(pack.lat == (int) -1079058708);
                Debug.Assert(pack.alt == (int)70401604);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.3492444E38F, 3.3662445E38F, 2.4317554E38F, -3.4006695E38F, 5.698855E37F, -7.9657227E37F, 2.4888529E38F, 1.3345113E38F, -1.0963076E38F, 1.80833E38F, -2.4721187E38F, 3.0846898E37F, -2.1651422E37F, -3.0342712E38F, -1.0597426E38F, -9.862887E37F, 5.72686E37F, 1.4878743E38F, -2.2315119E38F, 2.7506178E38F, -6.780041E37F, 1.4914181E38F, 1.4731457E38F, 2.549553E38F, -2.7118707E38F, 8.381032E36F, 2.3663187E38F, -1.3786192E38F, -1.2595646E38F, 3.3263247E38F, 2.369019E38F, 2.1395431E38F, -2.4123975E38F, 1.2827902E38F, 2.5509063E38F, 7.8392416E37F}));
                Debug.Assert(pack.time_usec == (ulong)3103166785005411573L);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.lat = (int) -1079058708;
            p63.time_usec = (ulong)3103166785005411573L;
            p63.vx = (float) -2.0477088E38F;
            p63.covariance_SET(new float[] {-2.3492444E38F, 3.3662445E38F, 2.4317554E38F, -3.4006695E38F, 5.698855E37F, -7.9657227E37F, 2.4888529E38F, 1.3345113E38F, -1.0963076E38F, 1.80833E38F, -2.4721187E38F, 3.0846898E37F, -2.1651422E37F, -3.0342712E38F, -1.0597426E38F, -9.862887E37F, 5.72686E37F, 1.4878743E38F, -2.2315119E38F, 2.7506178E38F, -6.780041E37F, 1.4914181E38F, 1.4731457E38F, 2.549553E38F, -2.7118707E38F, 8.381032E36F, 2.3663187E38F, -1.3786192E38F, -1.2595646E38F, 3.3263247E38F, 2.369019E38F, 2.1395431E38F, -2.4123975E38F, 1.2827902E38F, 2.5509063E38F, 7.8392416E37F}, 0) ;
            p63.lon = (int) -604228520;
            p63.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO;
            p63.relative_alt = (int) -1247298723;
            p63.alt = (int)70401604;
            p63.vz = (float)2.5082435E38F;
            p63.vy = (float)2.1502693E38F;
            ADV_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float) -3.0748141E38F);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
                Debug.Assert(pack.ay == (float) -1.3208508E38F);
                Debug.Assert(pack.time_usec == (ulong)6236971304466069710L);
                Debug.Assert(pack.vx == (float)7.3709283E37F);
                Debug.Assert(pack.az == (float)2.7916942E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {3.0481874E38F, -2.9797535E38F, -1.0073621E38F, 1.1056842E38F, 1.1511764E38F, 2.903925E38F, -2.9418615E36F, -2.0635368E38F, 1.7886775E38F, 2.0093866E38F, -2.072272E38F, -3.0382202E38F, 2.5043267E38F, 1.4472604E38F, 1.3354327E38F, 1.5273909E38F, 1.1956094E38F, 7.753096E37F, -3.982619E37F, -2.6321501E38F, 2.2034826E38F, -1.02126014E37F, -3.2019974E38F, -1.1198577E38F, 2.9696178E38F, 8.746367E37F, -1.5116114E38F, -8.836079E37F, 8.361689E37F, 1.1337074E38F, -1.166618E37F, -1.2971801E38F, -1.2388018E38F, -1.7339028E38F, -1.4237064E37F, 8.0829687E37F, -2.629013E38F, -2.7970575E38F, -3.8725366E36F, 1.1747828E37F, 6.5417246E37F, 3.0801115E38F, 1.8055214E38F, 3.2543286E38F, 2.1852128E38F}));
                Debug.Assert(pack.x == (float) -1.03795916E37F);
                Debug.Assert(pack.vz == (float)2.6938129E38F);
                Debug.Assert(pack.ax == (float)3.2728954E38F);
                Debug.Assert(pack.z == (float) -3.3081014E38F);
                Debug.Assert(pack.y == (float) -2.0303335E38F);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.ay = (float) -1.3208508E38F;
            p64.z = (float) -3.3081014E38F;
            p64.time_usec = (ulong)6236971304466069710L;
            p64.vz = (float)2.6938129E38F;
            p64.vy = (float) -3.0748141E38F;
            p64.vx = (float)7.3709283E37F;
            p64.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS;
            p64.x = (float) -1.03795916E37F;
            p64.az = (float)2.7916942E38F;
            p64.y = (float) -2.0303335E38F;
            p64.ax = (float)3.2728954E38F;
            p64.covariance_SET(new float[] {3.0481874E38F, -2.9797535E38F, -1.0073621E38F, 1.1056842E38F, 1.1511764E38F, 2.903925E38F, -2.9418615E36F, -2.0635368E38F, 1.7886775E38F, 2.0093866E38F, -2.072272E38F, -3.0382202E38F, 2.5043267E38F, 1.4472604E38F, 1.3354327E38F, 1.5273909E38F, 1.1956094E38F, 7.753096E37F, -3.982619E37F, -2.6321501E38F, 2.2034826E38F, -1.02126014E37F, -3.2019974E38F, -1.1198577E38F, 2.9696178E38F, 8.746367E37F, -1.5116114E38F, -8.836079E37F, 8.361689E37F, 1.1337074E38F, -1.166618E37F, -1.2971801E38F, -1.2388018E38F, -1.7339028E38F, -1.4237064E37F, 8.0829687E37F, -2.629013E38F, -2.7970575E38F, -3.8725366E36F, 1.1747828E37F, 6.5417246E37F, 3.0801115E38F, 1.8055214E38F, 3.2543286E38F, 2.1852128E38F}, 0) ;
            ADV_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)64001);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)52894);
                Debug.Assert(pack.rssi == (byte)(byte)25);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)10359);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)63525);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)28461);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)3321);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)51364);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)7983);
                Debug.Assert(pack.chancount == (byte)(byte)190);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)43736);
                Debug.Assert(pack.time_boot_ms == (uint)3756794816U);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)16926);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)49131);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)48249);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)19976);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)38189);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)1268);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)4485);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)48792);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)44457);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan18_raw = (ushort)(ushort)43736;
            p65.chan4_raw = (ushort)(ushort)28461;
            p65.chan10_raw = (ushort)(ushort)1268;
            p65.chan6_raw = (ushort)(ushort)19976;
            p65.chan16_raw = (ushort)(ushort)7983;
            p65.chan7_raw = (ushort)(ushort)63525;
            p65.chan11_raw = (ushort)(ushort)10359;
            p65.chan1_raw = (ushort)(ushort)44457;
            p65.time_boot_ms = (uint)3756794816U;
            p65.chan2_raw = (ushort)(ushort)48249;
            p65.chan17_raw = (ushort)(ushort)64001;
            p65.chan9_raw = (ushort)(ushort)38189;
            p65.chancount = (byte)(byte)190;
            p65.chan13_raw = (ushort)(ushort)48792;
            p65.chan3_raw = (ushort)(ushort)51364;
            p65.chan12_raw = (ushort)(ushort)16926;
            p65.rssi = (byte)(byte)25;
            p65.chan5_raw = (ushort)(ushort)52894;
            p65.chan15_raw = (ushort)(ushort)49131;
            p65.chan14_raw = (ushort)(ushort)4485;
            p65.chan8_raw = (ushort)(ushort)3321;
            ADV_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)153);
                Debug.Assert(pack.target_system == (byte)(byte)198);
                Debug.Assert(pack.req_stream_id == (byte)(byte)169);
                Debug.Assert(pack.start_stop == (byte)(byte)25);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)18799);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_system = (byte)(byte)198;
            p66.start_stop = (byte)(byte)25;
            p66.req_stream_id = (byte)(byte)169;
            p66.req_message_rate = (ushort)(ushort)18799;
            p66.target_component = (byte)(byte)153;
            ADV_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.stream_id == (byte)(byte)43);
                Debug.Assert(pack.message_rate == (ushort)(ushort)38760);
                Debug.Assert(pack.on_off == (byte)(byte)216);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.on_off = (byte)(byte)216;
            p67.stream_id = (byte)(byte)43;
            p67.message_rate = (ushort)(ushort)38760;
            ADV_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (short)(short)23745);
                Debug.Assert(pack.r == (short)(short)4664);
                Debug.Assert(pack.x == (short)(short)14380);
                Debug.Assert(pack.target == (byte)(byte)169);
                Debug.Assert(pack.buttons == (ushort)(ushort)5459);
                Debug.Assert(pack.z == (short)(short) -5349);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.buttons = (ushort)(ushort)5459;
            p69.target = (byte)(byte)169;
            p69.z = (short)(short) -5349;
            p69.y = (short)(short)23745;
            p69.r = (short)(short)4664;
            p69.x = (short)(short)14380;
            ADV_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)48534);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)52239);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)50937);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)58648);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)43466);
                Debug.Assert(pack.target_system == (byte)(byte)107);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)22281);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)33211);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)15860);
                Debug.Assert(pack.target_component == (byte)(byte)216);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan4_raw = (ushort)(ushort)33211;
            p70.target_system = (byte)(byte)107;
            p70.chan6_raw = (ushort)(ushort)43466;
            p70.chan3_raw = (ushort)(ushort)50937;
            p70.target_component = (byte)(byte)216;
            p70.chan7_raw = (ushort)(ushort)22281;
            p70.chan2_raw = (ushort)(ushort)15860;
            p70.chan8_raw = (ushort)(ushort)58648;
            p70.chan5_raw = (ushort)(ushort)52239;
            p70.chan1_raw = (ushort)(ushort)48534;
            ADV_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)108);
                Debug.Assert(pack.param3 == (float)2.2501441E38F);
                Debug.Assert(pack.param2 == (float)1.3500071E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_NED);
                Debug.Assert(pack.seq == (ushort)(ushort)23167);
                Debug.Assert(pack.current == (byte)(byte)13);
                Debug.Assert(pack.y == (int)189557154);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_system == (byte)(byte)75);
                Debug.Assert(pack.z == (float) -1.241117E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS);
                Debug.Assert(pack.param4 == (float)2.6471824E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)102);
                Debug.Assert(pack.x == (int) -41923352);
                Debug.Assert(pack.param1 == (float) -3.0935467E38F);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p73.target_system = (byte)(byte)75;
            p73.frame = MAV_FRAME.MAV_FRAME_BODY_NED;
            p73.target_component = (byte)(byte)108;
            p73.z = (float) -1.241117E38F;
            p73.seq = (ushort)(ushort)23167;
            p73.y = (int)189557154;
            p73.command = MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS;
            p73.param1 = (float) -3.0935467E38F;
            p73.param3 = (float)2.2501441E38F;
            p73.param4 = (float)2.6471824E38F;
            p73.x = (int) -41923352;
            p73.current = (byte)(byte)13;
            p73.param2 = (float)1.3500071E38F;
            p73.autocontinue = (byte)(byte)102;
            ADV_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (float)3.1165431E38F);
                Debug.Assert(pack.heading == (short)(short) -13987);
                Debug.Assert(pack.climb == (float) -1.1133203E38F);
                Debug.Assert(pack.airspeed == (float)2.1919567E38F);
                Debug.Assert(pack.groundspeed == (float)3.6951554E37F);
                Debug.Assert(pack.throttle == (ushort)(ushort)31883);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.heading = (short)(short) -13987;
            p74.throttle = (ushort)(ushort)31883;
            p74.climb = (float) -1.1133203E38F;
            p74.alt = (float)3.1165431E38F;
            p74.airspeed = (float)2.1919567E38F;
            p74.groundspeed = (float)3.6951554E37F;
            ADV_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)243);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.param2 == (float)8.591396E36F);
                Debug.Assert(pack.current == (byte)(byte)229);
                Debug.Assert(pack.target_system == (byte)(byte)156);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_PANORAMA_CREATE);
                Debug.Assert(pack.y == (int) -449667258);
                Debug.Assert(pack.param1 == (float) -3.3330504E38F);
                Debug.Assert(pack.z == (float)1.733922E38F);
                Debug.Assert(pack.param4 == (float)1.2835547E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)236);
                Debug.Assert(pack.param3 == (float)1.880478E38F);
                Debug.Assert(pack.x == (int) -1988800197);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.param4 = (float)1.2835547E38F;
            p75.frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p75.current = (byte)(byte)229;
            p75.autocontinue = (byte)(byte)236;
            p75.param2 = (float)8.591396E36F;
            p75.target_system = (byte)(byte)156;
            p75.command = MAV_CMD.MAV_CMD_PANORAMA_CREATE;
            p75.param1 = (float) -3.3330504E38F;
            p75.param3 = (float)1.880478E38F;
            p75.x = (int) -1988800197;
            p75.y = (int) -449667258;
            p75.target_component = (byte)(byte)243;
            p75.z = (float)1.733922E38F;
            ADV_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)70);
                Debug.Assert(pack.target_system == (byte)(byte)22);
                Debug.Assert(pack.param3 == (float) -3.2821807E38F);
                Debug.Assert(pack.param2 == (float)1.8371782E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_PANORAMA_CREATE);
                Debug.Assert(pack.param4 == (float) -1.8085003E38F);
                Debug.Assert(pack.param1 == (float)2.8363429E38F);
                Debug.Assert(pack.param5 == (float)2.4161605E38F);
                Debug.Assert(pack.param7 == (float)3.293651E38F);
                Debug.Assert(pack.confirmation == (byte)(byte)199);
                Debug.Assert(pack.param6 == (float)2.5660579E38F);
            };
            COMMAND_LONG p76 = new COMMAND_LONG();
            PH.setPack(p76);
            p76.param6 = (float)2.5660579E38F;
            p76.param1 = (float)2.8363429E38F;
            p76.param7 = (float)3.293651E38F;
            p76.target_component = (byte)(byte)70;
            p76.confirmation = (byte)(byte)199;
            p76.param3 = (float) -3.2821807E38F;
            p76.param2 = (float)1.8371782E38F;
            p76.param4 = (float) -1.8085003E38F;
            p76.command = MAV_CMD.MAV_CMD_PANORAMA_CREATE;
            p76.target_system = (byte)(byte)22;
            p76.param5 = (float)2.4161605E38F;
            ADV_TEST_CH.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)159);
                Debug.Assert(pack.result_param2_TRY(ph) == (int)1414666087);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)139);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_GO_AROUND);
                Debug.Assert(pack.result == MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)185);
            };
            COMMAND_ACK p77 = new COMMAND_ACK();
            PH.setPack(p77);
            p77.target_component_SET((byte)(byte)139, PH) ;
            p77.progress_SET((byte)(byte)159, PH) ;
            p77.command = MAV_CMD.MAV_CMD_DO_GO_AROUND;
            p77.target_system_SET((byte)(byte)185, PH) ;
            p77.result = MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED;
            p77.result_param2_SET((int)1414666087, PH) ;
            ADV_TEST_CH.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.manual_override_switch == (byte)(byte)254);
                Debug.Assert(pack.roll == (float) -3.2361296E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3948297199U);
                Debug.Assert(pack.yaw == (float) -6.8884864E37F);
                Debug.Assert(pack.thrust == (float)1.2122586E38F);
                Debug.Assert(pack.mode_switch == (byte)(byte)136);
                Debug.Assert(pack.pitch == (float) -2.9874902E38F);
            };
            MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.roll = (float) -3.2361296E38F;
            p81.time_boot_ms = (uint)3948297199U;
            p81.thrust = (float)1.2122586E38F;
            p81.pitch = (float) -2.9874902E38F;
            p81.manual_override_switch = (byte)(byte)254;
            p81.yaw = (float) -6.8884864E37F;
            p81.mode_switch = (byte)(byte)136;
            ADV_TEST_CH.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)616786817U);
                Debug.Assert(pack.body_yaw_rate == (float)1.7476817E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)230);
                Debug.Assert(pack.body_pitch_rate == (float)9.724794E37F);
                Debug.Assert(pack.target_system == (byte)(byte)181);
                Debug.Assert(pack.target_component == (byte)(byte)71);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.2855339E37F, -1.9150047E38F, -2.7844266E38F, -1.3340795E38F}));
                Debug.Assert(pack.thrust == (float)1.3938809E38F);
                Debug.Assert(pack.body_roll_rate == (float) -3.2737728E38F);
            };
            GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.target_system = (byte)(byte)181;
            p82.body_roll_rate = (float) -3.2737728E38F;
            p82.target_component = (byte)(byte)71;
            p82.type_mask = (byte)(byte)230;
            p82.body_pitch_rate = (float)9.724794E37F;
            p82.time_boot_ms = (uint)616786817U;
            p82.q_SET(new float[] {-1.2855339E37F, -1.9150047E38F, -2.7844266E38F, -1.3340795E38F}, 0) ;
            p82.body_yaw_rate = (float)1.7476817E38F;
            p82.thrust = (float)1.3938809E38F;
            CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_pitch_rate == (float)2.6010502E37F);
                Debug.Assert(pack.time_boot_ms == (uint)2480603296U);
                Debug.Assert(pack.type_mask == (byte)(byte)98);
                Debug.Assert(pack.thrust == (float)1.8542528E38F);
                Debug.Assert(pack.body_roll_rate == (float)1.4968772E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.595441E38F, 2.0659494E38F, 1.4972765E38F, 6.2744106E37F}));
                Debug.Assert(pack.body_yaw_rate == (float) -2.6112469E38F);
            };
            GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.body_yaw_rate = (float) -2.6112469E38F;
            p83.thrust = (float)1.8542528E38F;
            p83.time_boot_ms = (uint)2480603296U;
            p83.type_mask = (byte)(byte)98;
            p83.q_SET(new float[] {2.595441E38F, 2.0659494E38F, 1.4972765E38F, 6.2744106E37F}, 0) ;
            p83.body_pitch_rate = (float)2.6010502E37F;
            p83.body_roll_rate = (float)1.4968772E38F;
            CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float)4.0889893E37F);
                Debug.Assert(pack.vx == (float)1.5501588E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)23624);
                Debug.Assert(pack.target_component == (byte)(byte)10);
                Debug.Assert(pack.vz == (float) -2.4083287E38F);
                Debug.Assert(pack.z == (float) -2.706412E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1617543351U);
                Debug.Assert(pack.yaw == (float) -1.2850328E38F);
                Debug.Assert(pack.yaw_rate == (float) -3.3151106E38F);
                Debug.Assert(pack.afx == (float)3.3042262E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.afz == (float)7.29009E37F);
                Debug.Assert(pack.y == (float) -1.9286158E38F);
                Debug.Assert(pack.target_system == (byte)(byte)224);
                Debug.Assert(pack.x == (float) -2.6634698E38F);
                Debug.Assert(pack.afy == (float)9.462702E37F);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.type_mask = (ushort)(ushort)23624;
            p84.target_component = (byte)(byte)10;
            p84.target_system = (byte)(byte)224;
            p84.vx = (float)1.5501588E37F;
            p84.vz = (float) -2.4083287E38F;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p84.yaw = (float) -1.2850328E38F;
            p84.time_boot_ms = (uint)1617543351U;
            p84.afy = (float)9.462702E37F;
            p84.z = (float) -2.706412E38F;
            p84.vy = (float)4.0889893E37F;
            p84.x = (float) -2.6634698E38F;
            p84.y = (float) -1.9286158E38F;
            p84.afx = (float)3.3042262E38F;
            p84.yaw_rate = (float) -3.3151106E38F;
            p84.afz = (float)7.29009E37F;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)4);
                Debug.Assert(pack.vx == (float)2.4697057E38F);
                Debug.Assert(pack.alt == (float) -3.110976E37F);
                Debug.Assert(pack.vy == (float) -5.2017944E36F);
                Debug.Assert(pack.lat_int == (int)640912685);
                Debug.Assert(pack.type_mask == (ushort)(ushort)47462);
                Debug.Assert(pack.afy == (float) -4.729842E37F);
                Debug.Assert(pack.yaw_rate == (float) -3.3805327E38F);
                Debug.Assert(pack.lon_int == (int) -149433311);
                Debug.Assert(pack.afz == (float)1.0434727E38F);
                Debug.Assert(pack.target_component == (byte)(byte)175);
                Debug.Assert(pack.time_boot_ms == (uint)3034779180U);
                Debug.Assert(pack.yaw == (float) -8.761873E37F);
                Debug.Assert(pack.vz == (float) -2.5433693E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_BODY_NED);
                Debug.Assert(pack.afx == (float)4.0239203E37F);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.vx = (float)2.4697057E38F;
            p86.afy = (float) -4.729842E37F;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_BODY_NED;
            p86.vz = (float) -2.5433693E38F;
            p86.afz = (float)1.0434727E38F;
            p86.target_component = (byte)(byte)175;
            p86.vy = (float) -5.2017944E36F;
            p86.yaw = (float) -8.761873E37F;
            p86.afx = (float)4.0239203E37F;
            p86.lat_int = (int)640912685;
            p86.alt = (float) -3.110976E37F;
            p86.type_mask = (ushort)(ushort)47462;
            p86.lon_int = (int) -149433311;
            p86.target_system = (byte)(byte)4;
            p86.time_boot_ms = (uint)3034779180U;
            p86.yaw_rate = (float) -3.3805327E38F;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)4010593493U);
                Debug.Assert(pack.yaw_rate == (float) -6.9488377E37F);
                Debug.Assert(pack.lat_int == (int) -808200140);
                Debug.Assert(pack.vx == (float) -2.0751619E38F);
                Debug.Assert(pack.vy == (float)2.232445E38F);
                Debug.Assert(pack.afy == (float) -2.4871256E38F);
                Debug.Assert(pack.yaw == (float) -2.8560488E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.alt == (float)1.2395018E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)54340);
                Debug.Assert(pack.lon_int == (int)1150439655);
                Debug.Assert(pack.afx == (float)1.502527E38F);
                Debug.Assert(pack.afz == (float)1.0138522E38F);
                Debug.Assert(pack.vz == (float) -2.765234E38F);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.yaw = (float) -2.8560488E38F;
            p87.afz = (float)1.0138522E38F;
            p87.vz = (float) -2.765234E38F;
            p87.yaw_rate = (float) -6.9488377E37F;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p87.lat_int = (int) -808200140;
            p87.type_mask = (ushort)(ushort)54340;
            p87.time_boot_ms = (uint)4010593493U;
            p87.lon_int = (int)1150439655;
            p87.afy = (float) -2.4871256E38F;
            p87.vx = (float) -2.0751619E38F;
            p87.afx = (float)1.502527E38F;
            p87.vy = (float)2.232445E38F;
            p87.alt = (float)1.2395018E38F;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)3.3054862E38F);
                Debug.Assert(pack.pitch == (float)2.605315E38F);
                Debug.Assert(pack.x == (float) -1.9618869E37F);
                Debug.Assert(pack.z == (float)2.0448283E38F);
                Debug.Assert(pack.yaw == (float) -1.910497E38F);
                Debug.Assert(pack.roll == (float) -1.9450717E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3413369986U);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)3413369986U;
            p89.pitch = (float)2.605315E38F;
            p89.yaw = (float) -1.910497E38F;
            p89.y = (float)3.3054862E38F;
            p89.x = (float) -1.9618869E37F;
            p89.z = (float)2.0448283E38F;
            p89.roll = (float) -1.9450717E38F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float)2.6803403E38F);
                Debug.Assert(pack.alt == (int) -1650652282);
                Debug.Assert(pack.rollspeed == (float) -3.301023E37F);
                Debug.Assert(pack.yacc == (short)(short)24721);
                Debug.Assert(pack.yaw == (float)3.4353727E35F);
                Debug.Assert(pack.zacc == (short)(short) -12949);
                Debug.Assert(pack.time_usec == (ulong)2061648294226759266L);
                Debug.Assert(pack.vz == (short)(short)8217);
                Debug.Assert(pack.xacc == (short)(short)31269);
                Debug.Assert(pack.vx == (short)(short) -14982);
                Debug.Assert(pack.lat == (int) -822064895);
                Debug.Assert(pack.pitchspeed == (float)1.761075E38F);
                Debug.Assert(pack.roll == (float)1.1399015E38F);
                Debug.Assert(pack.yawspeed == (float)1.602865E38F);
                Debug.Assert(pack.vy == (short)(short)23851);
                Debug.Assert(pack.lon == (int) -2146970209);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.yacc = (short)(short)24721;
            p90.roll = (float)1.1399015E38F;
            p90.vx = (short)(short) -14982;
            p90.time_usec = (ulong)2061648294226759266L;
            p90.vz = (short)(short)8217;
            p90.lon = (int) -2146970209;
            p90.pitchspeed = (float)1.761075E38F;
            p90.zacc = (short)(short) -12949;
            p90.xacc = (short)(short)31269;
            p90.rollspeed = (float) -3.301023E37F;
            p90.yawspeed = (float)1.602865E38F;
            p90.lat = (int) -822064895;
            p90.alt = (int) -1650652282;
            p90.vy = (short)(short)23851;
            p90.yaw = (float)3.4353727E35F;
            p90.pitch = (float)2.6803403E38F;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll_ailerons == (float) -6.2358535E36F);
                Debug.Assert(pack.aux1 == (float) -2.8750044E38F);
                Debug.Assert(pack.aux4 == (float) -1.7131407E38F);
                Debug.Assert(pack.time_usec == (ulong)3879411639198115681L);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_TEST_ARMED);
                Debug.Assert(pack.aux2 == (float)1.9448283E38F);
                Debug.Assert(pack.pitch_elevator == (float)3.0732157E36F);
                Debug.Assert(pack.nav_mode == (byte)(byte)147);
                Debug.Assert(pack.throttle == (float) -1.1880921E38F);
                Debug.Assert(pack.aux3 == (float)3.462711E37F);
                Debug.Assert(pack.yaw_rudder == (float)7.356545E37F);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.yaw_rudder = (float)7.356545E37F;
            p91.nav_mode = (byte)(byte)147;
            p91.time_usec = (ulong)3879411639198115681L;
            p91.aux1 = (float) -2.8750044E38F;
            p91.pitch_elevator = (float)3.0732157E36F;
            p91.mode = MAV_MODE.MAV_MODE_TEST_ARMED;
            p91.aux2 = (float)1.9448283E38F;
            p91.aux4 = (float) -1.7131407E38F;
            p91.roll_ailerons = (float) -6.2358535E36F;
            p91.throttle = (float) -1.1880921E38F;
            p91.aux3 = (float)3.462711E37F;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)29056);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)18347);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)57678);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)33042);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)53625);
                Debug.Assert(pack.rssi == (byte)(byte)226);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)39245);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)16410);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)11617);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)60710);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)42456);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)49238);
                Debug.Assert(pack.time_usec == (ulong)1761884781047750604L);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)52230);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan6_raw = (ushort)(ushort)60710;
            p92.chan11_raw = (ushort)(ushort)29056;
            p92.chan1_raw = (ushort)(ushort)33042;
            p92.rssi = (byte)(byte)226;
            p92.chan2_raw = (ushort)(ushort)57678;
            p92.chan8_raw = (ushort)(ushort)52230;
            p92.chan7_raw = (ushort)(ushort)18347;
            p92.chan10_raw = (ushort)(ushort)16410;
            p92.chan3_raw = (ushort)(ushort)42456;
            p92.time_usec = (ulong)1761884781047750604L;
            p92.chan9_raw = (ushort)(ushort)53625;
            p92.chan4_raw = (ushort)(ushort)11617;
            p92.chan5_raw = (ushort)(ushort)39245;
            p92.chan12_raw = (ushort)(ushort)49238;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (ulong)6632784378427911754L);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
                Debug.Assert(pack.time_usec == (ulong)1054048133076126491L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.4999011E38F, -6.6251766E37F, -2.2830554E38F, -3.0008803E38F, 2.1401464E37F, -5.476025E37F, 2.3505526E38F, -2.6824444E38F, -2.3658786E38F, -1.0316851E38F, 1.1090421E38F, -2.0395135E38F, 2.963778E38F, 2.770072E38F, -1.773689E38F, 1.6081424E38F}));
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.controls_SET(new float[] {-1.4999011E38F, -6.6251766E37F, -2.2830554E38F, -3.0008803E38F, 2.1401464E37F, -5.476025E37F, 2.3505526E38F, -2.6824444E38F, -2.3658786E38F, -1.0316851E38F, 1.1090421E38F, -2.0395135E38F, 2.963778E38F, 2.770072E38F, -1.773689E38F, 1.6081424E38F}, 0) ;
            p93.mode = MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            p93.time_usec = (ulong)1054048133076126491L;
            p93.flags = (ulong)6632784378427911754L;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)317607895675781957L);
                Debug.Assert(pack.quality == (byte)(byte)185);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float) -1.5635816E38F);
                Debug.Assert(pack.flow_y == (short)(short)24164);
                Debug.Assert(pack.ground_distance == (float) -2.3531041E38F);
                Debug.Assert(pack.flow_x == (short)(short)17583);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float) -2.6160998E38F);
                Debug.Assert(pack.flow_comp_m_x == (float) -2.5589067E38F);
                Debug.Assert(pack.flow_comp_m_y == (float) -2.2647505E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)254);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.sensor_id = (byte)(byte)254;
            p100.flow_y = (short)(short)24164;
            p100.quality = (byte)(byte)185;
            p100.ground_distance = (float) -2.3531041E38F;
            p100.flow_comp_m_x = (float) -2.5589067E38F;
            p100.flow_comp_m_y = (float) -2.2647505E38F;
            p100.flow_rate_y_SET((float) -2.6160998E38F, PH) ;
            p100.flow_x = (short)(short)17583;
            p100.time_usec = (ulong)317607895675781957L;
            p100.flow_rate_x_SET((float) -1.5635816E38F, PH) ;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)2.9561612E38F);
                Debug.Assert(pack.z == (float)1.9834663E38F);
                Debug.Assert(pack.yaw == (float) -2.230347E38F);
                Debug.Assert(pack.roll == (float)1.5641055E38F);
                Debug.Assert(pack.x == (float)5.379959E37F);
                Debug.Assert(pack.usec == (ulong)6560488234159783099L);
                Debug.Assert(pack.pitch == (float)1.1757912E37F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.z = (float)1.9834663E38F;
            p101.y = (float)2.9561612E38F;
            p101.pitch = (float)1.1757912E37F;
            p101.roll = (float)1.5641055E38F;
            p101.usec = (ulong)6560488234159783099L;
            p101.x = (float)5.379959E37F;
            p101.yaw = (float) -2.230347E38F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)1.9865714E38F);
                Debug.Assert(pack.x == (float) -5.2092484E37F);
                Debug.Assert(pack.usec == (ulong)3229281792098888136L);
                Debug.Assert(pack.yaw == (float)2.266891E38F);
                Debug.Assert(pack.pitch == (float)1.2846136E38F);
                Debug.Assert(pack.y == (float) -3.694508E37F);
                Debug.Assert(pack.z == (float) -2.495411E38F);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.y = (float) -3.694508E37F;
            p102.usec = (ulong)3229281792098888136L;
            p102.z = (float) -2.495411E38F;
            p102.pitch = (float)1.2846136E38F;
            p102.roll = (float)1.9865714E38F;
            p102.yaw = (float)2.266891E38F;
            p102.x = (float) -5.2092484E37F;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)1.9240379E37F);
                Debug.Assert(pack.z == (float) -3.3400806E38F);
                Debug.Assert(pack.usec == (ulong)5054912509995082530L);
                Debug.Assert(pack.x == (float)8.704894E37F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.z = (float) -3.3400806E38F;
            p103.usec = (ulong)5054912509995082530L;
            p103.y = (float)1.9240379E37F;
            p103.x = (float)8.704894E37F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float)1.5216381E38F);
                Debug.Assert(pack.yaw == (float) -2.5739522E38F);
                Debug.Assert(pack.y == (float)1.7501069E38F);
                Debug.Assert(pack.roll == (float)1.7709489E38F);
                Debug.Assert(pack.usec == (ulong)9085247345558867107L);
                Debug.Assert(pack.x == (float) -1.6528747E38F);
                Debug.Assert(pack.z == (float)2.5445516E38F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.y = (float)1.7501069E38F;
            p104.z = (float)2.5445516E38F;
            p104.pitch = (float)1.5216381E38F;
            p104.roll = (float)1.7709489E38F;
            p104.x = (float) -1.6528747E38F;
            p104.usec = (ulong)9085247345558867107L;
            p104.yaw = (float) -2.5739522E38F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (float) -2.2831006E38F);
                Debug.Assert(pack.abs_pressure == (float) -5.7674634E37F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)45919);
                Debug.Assert(pack.pressure_alt == (float)1.8386029E38F);
                Debug.Assert(pack.xmag == (float) -1.8578932E37F);
                Debug.Assert(pack.zacc == (float)1.3622653E38F);
                Debug.Assert(pack.ymag == (float)2.0860213E38F);
                Debug.Assert(pack.yacc == (float)2.1142343E38F);
                Debug.Assert(pack.time_usec == (ulong)2397465346273247013L);
                Debug.Assert(pack.diff_pressure == (float) -3.2302573E38F);
                Debug.Assert(pack.zmag == (float)3.2223507E36F);
                Debug.Assert(pack.ygyro == (float)1.5670623E38F);
                Debug.Assert(pack.xacc == (float)1.9883826E38F);
                Debug.Assert(pack.temperature == (float)2.5505816E38F);
                Debug.Assert(pack.zgyro == (float) -1.5429184E38F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.abs_pressure = (float) -5.7674634E37F;
            p105.fields_updated = (ushort)(ushort)45919;
            p105.temperature = (float)2.5505816E38F;
            p105.ygyro = (float)1.5670623E38F;
            p105.xmag = (float) -1.8578932E37F;
            p105.zgyro = (float) -1.5429184E38F;
            p105.time_usec = (ulong)2397465346273247013L;
            p105.ymag = (float)2.0860213E38F;
            p105.diff_pressure = (float) -3.2302573E38F;
            p105.zacc = (float)1.3622653E38F;
            p105.yacc = (float)2.1142343E38F;
            p105.zmag = (float)3.2223507E36F;
            p105.xacc = (float)1.9883826E38F;
            p105.pressure_alt = (float)1.8386029E38F;
            p105.xgyro = (float) -2.2831006E38F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -12760);
                Debug.Assert(pack.time_delta_distance_us == (uint)2767119279U);
                Debug.Assert(pack.integrated_ygyro == (float) -5.380028E37F);
                Debug.Assert(pack.integrated_xgyro == (float)1.004925E38F);
                Debug.Assert(pack.integrated_y == (float)3.2205678E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)6);
                Debug.Assert(pack.integrated_zgyro == (float) -2.5278995E38F);
                Debug.Assert(pack.quality == (byte)(byte)198);
                Debug.Assert(pack.distance == (float) -1.0197798E38F);
                Debug.Assert(pack.integration_time_us == (uint)3504570523U);
                Debug.Assert(pack.integrated_x == (float) -2.1860586E38F);
                Debug.Assert(pack.time_usec == (ulong)1607395341185173552L);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.integration_time_us = (uint)3504570523U;
            p106.temperature = (short)(short) -12760;
            p106.time_delta_distance_us = (uint)2767119279U;
            p106.sensor_id = (byte)(byte)6;
            p106.integrated_xgyro = (float)1.004925E38F;
            p106.integrated_x = (float) -2.1860586E38F;
            p106.integrated_ygyro = (float) -5.380028E37F;
            p106.integrated_y = (float)3.2205678E38F;
            p106.quality = (byte)(byte)198;
            p106.time_usec = (ulong)1607395341185173552L;
            p106.integrated_zgyro = (float) -2.5278995E38F;
            p106.distance = (float) -1.0197798E38F;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (float)2.5259968E38F);
                Debug.Assert(pack.yacc == (float) -6.0839643E37F);
                Debug.Assert(pack.xgyro == (float) -2.3698675E38F);
                Debug.Assert(pack.xmag == (float)2.3752916E38F);
                Debug.Assert(pack.pressure_alt == (float)1.1064659E38F);
                Debug.Assert(pack.zgyro == (float)2.5157284E37F);
                Debug.Assert(pack.temperature == (float) -3.8649047E37F);
                Debug.Assert(pack.ymag == (float)2.5584992E38F);
                Debug.Assert(pack.diff_pressure == (float)1.440859E38F);
                Debug.Assert(pack.abs_pressure == (float)3.1230075E38F);
                Debug.Assert(pack.time_usec == (ulong)4738482621523529340L);
                Debug.Assert(pack.xacc == (float)1.7309702E38F);
                Debug.Assert(pack.zacc == (float)2.7299758E38F);
                Debug.Assert(pack.zmag == (float) -1.5133499E38F);
                Debug.Assert(pack.fields_updated == (uint)3129524538U);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.xacc = (float)1.7309702E38F;
            p107.pressure_alt = (float)1.1064659E38F;
            p107.ygyro = (float)2.5259968E38F;
            p107.abs_pressure = (float)3.1230075E38F;
            p107.xgyro = (float) -2.3698675E38F;
            p107.diff_pressure = (float)1.440859E38F;
            p107.ymag = (float)2.5584992E38F;
            p107.zacc = (float)2.7299758E38F;
            p107.xmag = (float)2.3752916E38F;
            p107.yacc = (float) -6.0839643E37F;
            p107.time_usec = (ulong)4738482621523529340L;
            p107.zmag = (float) -1.5133499E38F;
            p107.zgyro = (float)2.5157284E37F;
            p107.fields_updated = (uint)3129524538U;
            p107.temperature = (float) -3.8649047E37F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (float) -3.1751106E38F);
                Debug.Assert(pack.yaw == (float)2.0911657E38F);
                Debug.Assert(pack.q1 == (float)1.3689597E38F);
                Debug.Assert(pack.zacc == (float) -1.8163135E38F);
                Debug.Assert(pack.zgyro == (float)2.4143053E38F);
                Debug.Assert(pack.ygyro == (float)3.1899468E38F);
                Debug.Assert(pack.q2 == (float) -2.9322322E38F);
                Debug.Assert(pack.std_dev_horz == (float) -8.1548257E37F);
                Debug.Assert(pack.q3 == (float)1.4390921E38F);
                Debug.Assert(pack.roll == (float)8.071909E37F);
                Debug.Assert(pack.std_dev_vert == (float)2.001128E38F);
                Debug.Assert(pack.alt == (float)2.7987975E38F);
                Debug.Assert(pack.ve == (float)2.0714827E38F);
                Debug.Assert(pack.lat == (float) -2.251126E38F);
                Debug.Assert(pack.xacc == (float)1.0591006E38F);
                Debug.Assert(pack.yacc == (float) -3.3004953E38F);
                Debug.Assert(pack.vn == (float)3.3596605E38F);
                Debug.Assert(pack.vd == (float) -1.3428956E38F);
                Debug.Assert(pack.xgyro == (float)3.3064815E37F);
                Debug.Assert(pack.pitch == (float) -3.1116295E38F);
                Debug.Assert(pack.q4 == (float) -2.9411782E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.zacc = (float) -1.8163135E38F;
            p108.xacc = (float)1.0591006E38F;
            p108.q1 = (float)1.3689597E38F;
            p108.alt = (float)2.7987975E38F;
            p108.pitch = (float) -3.1116295E38F;
            p108.roll = (float)8.071909E37F;
            p108.vn = (float)3.3596605E38F;
            p108.q4 = (float) -2.9411782E38F;
            p108.lon = (float) -3.1751106E38F;
            p108.lat = (float) -2.251126E38F;
            p108.xgyro = (float)3.3064815E37F;
            p108.vd = (float) -1.3428956E38F;
            p108.std_dev_vert = (float)2.001128E38F;
            p108.std_dev_horz = (float) -8.1548257E37F;
            p108.yaw = (float)2.0911657E38F;
            p108.yacc = (float) -3.3004953E38F;
            p108.zgyro = (float)2.4143053E38F;
            p108.ygyro = (float)3.1899468E38F;
            p108.q2 = (float) -2.9322322E38F;
            p108.q3 = (float)1.4390921E38F;
            p108.ve = (float)2.0714827E38F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.txbuf == (byte)(byte)160);
                Debug.Assert(pack.rssi == (byte)(byte)167);
                Debug.Assert(pack.remrssi == (byte)(byte)85);
                Debug.Assert(pack.noise == (byte)(byte)154);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)16531);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)43007);
                Debug.Assert(pack.remnoise == (byte)(byte)109);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rxerrors = (ushort)(ushort)16531;
            p109.fixed_ = (ushort)(ushort)43007;
            p109.remrssi = (byte)(byte)85;
            p109.txbuf = (byte)(byte)160;
            p109.remnoise = (byte)(byte)109;
            p109.noise = (byte)(byte)154;
            p109.rssi = (byte)(byte)167;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)202);
                Debug.Assert(pack.target_component == (byte)(byte)229);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)164, (byte)4, (byte)4, (byte)121, (byte)239, (byte)3, (byte)24, (byte)121, (byte)44, (byte)81, (byte)221, (byte)68, (byte)156, (byte)234, (byte)236, (byte)190, (byte)34, (byte)227, (byte)128, (byte)107, (byte)88, (byte)123, (byte)181, (byte)219, (byte)179, (byte)92, (byte)240, (byte)160, (byte)195, (byte)242, (byte)112, (byte)221, (byte)67, (byte)247, (byte)88, (byte)11, (byte)15, (byte)110, (byte)82, (byte)28, (byte)21, (byte)28, (byte)69, (byte)7, (byte)219, (byte)150, (byte)108, (byte)180, (byte)13, (byte)64, (byte)158, (byte)55, (byte)221, (byte)10, (byte)87, (byte)99, (byte)101, (byte)234, (byte)110, (byte)232, (byte)104, (byte)133, (byte)70, (byte)111, (byte)44, (byte)231, (byte)247, (byte)141, (byte)14, (byte)137, (byte)199, (byte)121, (byte)37, (byte)117, (byte)236, (byte)43, (byte)152, (byte)235, (byte)172, (byte)93, (byte)47, (byte)238, (byte)93, (byte)181, (byte)195, (byte)187, (byte)46, (byte)115, (byte)172, (byte)109, (byte)168, (byte)150, (byte)117, (byte)127, (byte)175, (byte)215, (byte)188, (byte)35, (byte)63, (byte)230, (byte)9, (byte)23, (byte)191, (byte)168, (byte)164, (byte)33, (byte)108, (byte)230, (byte)185, (byte)219, (byte)254, (byte)48, (byte)159, (byte)208, (byte)113, (byte)121, (byte)19, (byte)210, (byte)44, (byte)10, (byte)43, (byte)184, (byte)98, (byte)29, (byte)28, (byte)65, (byte)240, (byte)29, (byte)140, (byte)138, (byte)33, (byte)186, (byte)112, (byte)113, (byte)116, (byte)62, (byte)92, (byte)106, (byte)89, (byte)42, (byte)71, (byte)122, (byte)114, (byte)229, (byte)152, (byte)24, (byte)216, (byte)14, (byte)57, (byte)12, (byte)176, (byte)165, (byte)83, (byte)39, (byte)58, (byte)231, (byte)42, (byte)187, (byte)160, (byte)245, (byte)115, (byte)93, (byte)121, (byte)139, (byte)246, (byte)97, (byte)73, (byte)192, (byte)192, (byte)155, (byte)240, (byte)228, (byte)143, (byte)178, (byte)241, (byte)24, (byte)67, (byte)200, (byte)143, (byte)253, (byte)11, (byte)154, (byte)39, (byte)208, (byte)103, (byte)113, (byte)179, (byte)88, (byte)246, (byte)68, (byte)144, (byte)171, (byte)139, (byte)179, (byte)40, (byte)191, (byte)250, (byte)110, (byte)6, (byte)191, (byte)128, (byte)242, (byte)59, (byte)240, (byte)50, (byte)98, (byte)10, (byte)188, (byte)234, (byte)216, (byte)183, (byte)191, (byte)3, (byte)20, (byte)192, (byte)6, (byte)109, (byte)26, (byte)68, (byte)244, (byte)119, (byte)175, (byte)159, (byte)255, (byte)46, (byte)145, (byte)240, (byte)244, (byte)223, (byte)84, (byte)143, (byte)21, (byte)203, (byte)73, (byte)135, (byte)103, (byte)226, (byte)79, (byte)126, (byte)236, (byte)77, (byte)111, (byte)187, (byte)255, (byte)40, (byte)149, (byte)71, (byte)6, (byte)21, (byte)124, (byte)253}));
                Debug.Assert(pack.target_network == (byte)(byte)54);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_component = (byte)(byte)229;
            p110.target_system = (byte)(byte)202;
            p110.payload_SET(new byte[] {(byte)164, (byte)4, (byte)4, (byte)121, (byte)239, (byte)3, (byte)24, (byte)121, (byte)44, (byte)81, (byte)221, (byte)68, (byte)156, (byte)234, (byte)236, (byte)190, (byte)34, (byte)227, (byte)128, (byte)107, (byte)88, (byte)123, (byte)181, (byte)219, (byte)179, (byte)92, (byte)240, (byte)160, (byte)195, (byte)242, (byte)112, (byte)221, (byte)67, (byte)247, (byte)88, (byte)11, (byte)15, (byte)110, (byte)82, (byte)28, (byte)21, (byte)28, (byte)69, (byte)7, (byte)219, (byte)150, (byte)108, (byte)180, (byte)13, (byte)64, (byte)158, (byte)55, (byte)221, (byte)10, (byte)87, (byte)99, (byte)101, (byte)234, (byte)110, (byte)232, (byte)104, (byte)133, (byte)70, (byte)111, (byte)44, (byte)231, (byte)247, (byte)141, (byte)14, (byte)137, (byte)199, (byte)121, (byte)37, (byte)117, (byte)236, (byte)43, (byte)152, (byte)235, (byte)172, (byte)93, (byte)47, (byte)238, (byte)93, (byte)181, (byte)195, (byte)187, (byte)46, (byte)115, (byte)172, (byte)109, (byte)168, (byte)150, (byte)117, (byte)127, (byte)175, (byte)215, (byte)188, (byte)35, (byte)63, (byte)230, (byte)9, (byte)23, (byte)191, (byte)168, (byte)164, (byte)33, (byte)108, (byte)230, (byte)185, (byte)219, (byte)254, (byte)48, (byte)159, (byte)208, (byte)113, (byte)121, (byte)19, (byte)210, (byte)44, (byte)10, (byte)43, (byte)184, (byte)98, (byte)29, (byte)28, (byte)65, (byte)240, (byte)29, (byte)140, (byte)138, (byte)33, (byte)186, (byte)112, (byte)113, (byte)116, (byte)62, (byte)92, (byte)106, (byte)89, (byte)42, (byte)71, (byte)122, (byte)114, (byte)229, (byte)152, (byte)24, (byte)216, (byte)14, (byte)57, (byte)12, (byte)176, (byte)165, (byte)83, (byte)39, (byte)58, (byte)231, (byte)42, (byte)187, (byte)160, (byte)245, (byte)115, (byte)93, (byte)121, (byte)139, (byte)246, (byte)97, (byte)73, (byte)192, (byte)192, (byte)155, (byte)240, (byte)228, (byte)143, (byte)178, (byte)241, (byte)24, (byte)67, (byte)200, (byte)143, (byte)253, (byte)11, (byte)154, (byte)39, (byte)208, (byte)103, (byte)113, (byte)179, (byte)88, (byte)246, (byte)68, (byte)144, (byte)171, (byte)139, (byte)179, (byte)40, (byte)191, (byte)250, (byte)110, (byte)6, (byte)191, (byte)128, (byte)242, (byte)59, (byte)240, (byte)50, (byte)98, (byte)10, (byte)188, (byte)234, (byte)216, (byte)183, (byte)191, (byte)3, (byte)20, (byte)192, (byte)6, (byte)109, (byte)26, (byte)68, (byte)244, (byte)119, (byte)175, (byte)159, (byte)255, (byte)46, (byte)145, (byte)240, (byte)244, (byte)223, (byte)84, (byte)143, (byte)21, (byte)203, (byte)73, (byte)135, (byte)103, (byte)226, (byte)79, (byte)126, (byte)236, (byte)77, (byte)111, (byte)187, (byte)255, (byte)40, (byte)149, (byte)71, (byte)6, (byte)21, (byte)124, (byte)253}, 0) ;
            p110.target_network = (byte)(byte)54;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tc1 == (long)5432718062829882524L);
                Debug.Assert(pack.ts1 == (long) -8640649756285572464L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long) -8640649756285572464L;
            p111.tc1 = (long)5432718062829882524L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)5834498603645056040L);
                Debug.Assert(pack.seq == (uint)633525069U);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)5834498603645056040L;
            p112.seq = (uint)633525069U;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellites_visible == (byte)(byte)194);
                Debug.Assert(pack.vd == (short)(short)2722);
                Debug.Assert(pack.fix_type == (byte)(byte)78);
                Debug.Assert(pack.vn == (short)(short) -11837);
                Debug.Assert(pack.ve == (short)(short)8000);
                Debug.Assert(pack.vel == (ushort)(ushort)56979);
                Debug.Assert(pack.alt == (int)1599188626);
                Debug.Assert(pack.lat == (int) -1493625185);
                Debug.Assert(pack.eph == (ushort)(ushort)61374);
                Debug.Assert(pack.cog == (ushort)(ushort)11517);
                Debug.Assert(pack.lon == (int)92505382);
                Debug.Assert(pack.time_usec == (ulong)3711857393345757050L);
                Debug.Assert(pack.epv == (ushort)(ushort)7651);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.ve = (short)(short)8000;
            p113.alt = (int)1599188626;
            p113.satellites_visible = (byte)(byte)194;
            p113.epv = (ushort)(ushort)7651;
            p113.vn = (short)(short) -11837;
            p113.time_usec = (ulong)3711857393345757050L;
            p113.vel = (ushort)(ushort)56979;
            p113.eph = (ushort)(ushort)61374;
            p113.cog = (ushort)(ushort)11517;
            p113.vd = (short)(short)2722;
            p113.fix_type = (byte)(byte)78;
            p113.lat = (int) -1493625185;
            p113.lon = (int)92505382;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integration_time_us == (uint)1554311726U);
                Debug.Assert(pack.integrated_ygyro == (float) -1.2673236E38F);
                Debug.Assert(pack.temperature == (short)(short) -11283);
                Debug.Assert(pack.quality == (byte)(byte)129);
                Debug.Assert(pack.integrated_zgyro == (float) -5.889908E37F);
                Debug.Assert(pack.distance == (float) -2.4019733E37F);
                Debug.Assert(pack.integrated_x == (float) -3.3490242E38F);
                Debug.Assert(pack.integrated_xgyro == (float) -1.2164668E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)86248890U);
                Debug.Assert(pack.time_usec == (ulong)1528316891790221720L);
                Debug.Assert(pack.integrated_y == (float)2.2146353E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)178);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.integrated_xgyro = (float) -1.2164668E38F;
            p114.integrated_y = (float)2.2146353E38F;
            p114.integrated_x = (float) -3.3490242E38F;
            p114.distance = (float) -2.4019733E37F;
            p114.time_usec = (ulong)1528316891790221720L;
            p114.temperature = (short)(short) -11283;
            p114.integrated_ygyro = (float) -1.2673236E38F;
            p114.integration_time_us = (uint)1554311726U;
            p114.quality = (byte)(byte)129;
            p114.time_delta_distance_us = (uint)86248890U;
            p114.sensor_id = (byte)(byte)178;
            p114.integrated_zgyro = (float) -5.889908E37F;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (short)(short) -22165);
                Debug.Assert(pack.pitchspeed == (float) -2.5749278E38F);
                Debug.Assert(pack.yawspeed == (float) -1.3081845E38F);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)12827);
                Debug.Assert(pack.vx == (short)(short)32411);
                Debug.Assert(pack.lat == (int) -1480380205);
                Debug.Assert(pack.alt == (int)988880858);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {-2.679702E37F, 1.4941513E38F, -2.4137743E38F, -9.435997E37F}));
                Debug.Assert(pack.vz == (short)(short)2129);
                Debug.Assert(pack.zacc == (short)(short) -20496);
                Debug.Assert(pack.time_usec == (ulong)756408622338044593L);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)2394);
                Debug.Assert(pack.yacc == (short)(short) -26345);
                Debug.Assert(pack.rollspeed == (float) -8.525139E37F);
                Debug.Assert(pack.xacc == (short)(short)18400);
                Debug.Assert(pack.lon == (int)1469850551);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.true_airspeed = (ushort)(ushort)2394;
            p115.yacc = (short)(short) -26345;
            p115.zacc = (short)(short) -20496;
            p115.vy = (short)(short) -22165;
            p115.xacc = (short)(short)18400;
            p115.lon = (int)1469850551;
            p115.ind_airspeed = (ushort)(ushort)12827;
            p115.vz = (short)(short)2129;
            p115.lat = (int) -1480380205;
            p115.rollspeed = (float) -8.525139E37F;
            p115.pitchspeed = (float) -2.5749278E38F;
            p115.vx = (short)(short)32411;
            p115.alt = (int)988880858;
            p115.yawspeed = (float) -1.3081845E38F;
            p115.attitude_quaternion_SET(new float[] {-2.679702E37F, 1.4941513E38F, -2.4137743E38F, -9.435997E37F}, 0) ;
            p115.time_usec = (ulong)756408622338044593L;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (short)(short)25636);
                Debug.Assert(pack.zmag == (short)(short) -9435);
                Debug.Assert(pack.yacc == (short)(short)366);
                Debug.Assert(pack.time_boot_ms == (uint)3284744125U);
                Debug.Assert(pack.ymag == (short)(short)4420);
                Debug.Assert(pack.xgyro == (short)(short)884);
                Debug.Assert(pack.zgyro == (short)(short)29290);
                Debug.Assert(pack.xmag == (short)(short)31911);
                Debug.Assert(pack.ygyro == (short)(short)24097);
                Debug.Assert(pack.xacc == (short)(short)18970);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.ygyro = (short)(short)24097;
            p116.zgyro = (short)(short)29290;
            p116.zacc = (short)(short)25636;
            p116.xacc = (short)(short)18970;
            p116.ymag = (short)(short)4420;
            p116.zmag = (short)(short) -9435;
            p116.time_boot_ms = (uint)3284744125U;
            p116.xmag = (short)(short)31911;
            p116.yacc = (short)(short)366;
            p116.xgyro = (short)(short)884;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)211);
                Debug.Assert(pack.end == (ushort)(ushort)42001);
                Debug.Assert(pack.start == (ushort)(ushort)40934);
                Debug.Assert(pack.target_system == (byte)(byte)178);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.end = (ushort)(ushort)42001;
            p117.start = (ushort)(ushort)40934;
            p117.target_component = (byte)(byte)211;
            p117.target_system = (byte)(byte)178;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.size == (uint)3952766864U);
                Debug.Assert(pack.num_logs == (ushort)(ushort)59462);
                Debug.Assert(pack.id == (ushort)(ushort)27135);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)31809);
                Debug.Assert(pack.time_utc == (uint)3308479964U);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.size = (uint)3952766864U;
            p118.id = (ushort)(ushort)27135;
            p118.last_log_num = (ushort)(ushort)31809;
            p118.num_logs = (ushort)(ushort)59462;
            p118.time_utc = (uint)3308479964U;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)48);
                Debug.Assert(pack.target_system == (byte)(byte)230);
                Debug.Assert(pack.id == (ushort)(ushort)9880);
                Debug.Assert(pack.count == (uint)3813732206U);
                Debug.Assert(pack.ofs == (uint)1260166965U);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_component = (byte)(byte)48;
            p119.target_system = (byte)(byte)230;
            p119.ofs = (uint)1260166965U;
            p119.id = (ushort)(ushort)9880;
            p119.count = (uint)3813732206U;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)4177682867U);
                Debug.Assert(pack.count == (byte)(byte)171);
                Debug.Assert(pack.id == (ushort)(ushort)30396);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)52, (byte)196, (byte)156, (byte)210, (byte)221, (byte)219, (byte)223, (byte)145, (byte)107, (byte)72, (byte)10, (byte)26, (byte)139, (byte)26, (byte)121, (byte)83, (byte)80, (byte)138, (byte)217, (byte)4, (byte)54, (byte)94, (byte)232, (byte)110, (byte)36, (byte)223, (byte)133, (byte)172, (byte)71, (byte)99, (byte)106, (byte)121, (byte)51, (byte)13, (byte)92, (byte)172, (byte)22, (byte)123, (byte)232, (byte)41, (byte)55, (byte)115, (byte)204, (byte)37, (byte)89, (byte)145, (byte)245, (byte)37, (byte)161, (byte)137, (byte)88, (byte)147, (byte)165, (byte)155, (byte)49, (byte)5, (byte)167, (byte)17, (byte)113, (byte)34, (byte)49, (byte)84, (byte)89, (byte)129, (byte)210, (byte)44, (byte)152, (byte)113, (byte)159, (byte)48, (byte)247, (byte)83, (byte)138, (byte)103, (byte)24, (byte)219, (byte)240, (byte)237, (byte)138, (byte)78, (byte)172, (byte)76, (byte)196, (byte)69, (byte)77, (byte)157, (byte)101, (byte)41, (byte)223, (byte)249}));
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.count = (byte)(byte)171;
            p120.ofs = (uint)4177682867U;
            p120.id = (ushort)(ushort)30396;
            p120.data__SET(new byte[] {(byte)52, (byte)196, (byte)156, (byte)210, (byte)221, (byte)219, (byte)223, (byte)145, (byte)107, (byte)72, (byte)10, (byte)26, (byte)139, (byte)26, (byte)121, (byte)83, (byte)80, (byte)138, (byte)217, (byte)4, (byte)54, (byte)94, (byte)232, (byte)110, (byte)36, (byte)223, (byte)133, (byte)172, (byte)71, (byte)99, (byte)106, (byte)121, (byte)51, (byte)13, (byte)92, (byte)172, (byte)22, (byte)123, (byte)232, (byte)41, (byte)55, (byte)115, (byte)204, (byte)37, (byte)89, (byte)145, (byte)245, (byte)37, (byte)161, (byte)137, (byte)88, (byte)147, (byte)165, (byte)155, (byte)49, (byte)5, (byte)167, (byte)17, (byte)113, (byte)34, (byte)49, (byte)84, (byte)89, (byte)129, (byte)210, (byte)44, (byte)152, (byte)113, (byte)159, (byte)48, (byte)247, (byte)83, (byte)138, (byte)103, (byte)24, (byte)219, (byte)240, (byte)237, (byte)138, (byte)78, (byte)172, (byte)76, (byte)196, (byte)69, (byte)77, (byte)157, (byte)101, (byte)41, (byte)223, (byte)249}, 0) ;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)168);
                Debug.Assert(pack.target_system == (byte)(byte)171);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)171;
            p121.target_component = (byte)(byte)168;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)212);
                Debug.Assert(pack.target_component == (byte)(byte)189);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_component = (byte)(byte)189;
            p122.target_system = (byte)(byte)212;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)35, (byte)210, (byte)28, (byte)97, (byte)10, (byte)128, (byte)138, (byte)132, (byte)76, (byte)62, (byte)16, (byte)154, (byte)65, (byte)10, (byte)8, (byte)17, (byte)134, (byte)98, (byte)127, (byte)185, (byte)229, (byte)242, (byte)156, (byte)226, (byte)8, (byte)97, (byte)252, (byte)213, (byte)207, (byte)194, (byte)122, (byte)157, (byte)31, (byte)234, (byte)30, (byte)126, (byte)5, (byte)237, (byte)56, (byte)33, (byte)110, (byte)47, (byte)103, (byte)170, (byte)233, (byte)109, (byte)158, (byte)46, (byte)183, (byte)173, (byte)92, (byte)114, (byte)1, (byte)253, (byte)228, (byte)11, (byte)145, (byte)93, (byte)72, (byte)29, (byte)43, (byte)228, (byte)241, (byte)206, (byte)217, (byte)61, (byte)194, (byte)174, (byte)236, (byte)199, (byte)54, (byte)211, (byte)193, (byte)216, (byte)43, (byte)142, (byte)154, (byte)169, (byte)106, (byte)37, (byte)130, (byte)164, (byte)248, (byte)182, (byte)158, (byte)154, (byte)100, (byte)224, (byte)209, (byte)12, (byte)131, (byte)91, (byte)49, (byte)153, (byte)29, (byte)43, (byte)40, (byte)13, (byte)199, (byte)18, (byte)159, (byte)146, (byte)121, (byte)251, (byte)131, (byte)82, (byte)37, (byte)185, (byte)93, (byte)78}));
                Debug.Assert(pack.target_component == (byte)(byte)67);
                Debug.Assert(pack.len == (byte)(byte)97);
                Debug.Assert(pack.target_system == (byte)(byte)217);
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.len = (byte)(byte)97;
            p123.target_system = (byte)(byte)217;
            p123.data__SET(new byte[] {(byte)35, (byte)210, (byte)28, (byte)97, (byte)10, (byte)128, (byte)138, (byte)132, (byte)76, (byte)62, (byte)16, (byte)154, (byte)65, (byte)10, (byte)8, (byte)17, (byte)134, (byte)98, (byte)127, (byte)185, (byte)229, (byte)242, (byte)156, (byte)226, (byte)8, (byte)97, (byte)252, (byte)213, (byte)207, (byte)194, (byte)122, (byte)157, (byte)31, (byte)234, (byte)30, (byte)126, (byte)5, (byte)237, (byte)56, (byte)33, (byte)110, (byte)47, (byte)103, (byte)170, (byte)233, (byte)109, (byte)158, (byte)46, (byte)183, (byte)173, (byte)92, (byte)114, (byte)1, (byte)253, (byte)228, (byte)11, (byte)145, (byte)93, (byte)72, (byte)29, (byte)43, (byte)228, (byte)241, (byte)206, (byte)217, (byte)61, (byte)194, (byte)174, (byte)236, (byte)199, (byte)54, (byte)211, (byte)193, (byte)216, (byte)43, (byte)142, (byte)154, (byte)169, (byte)106, (byte)37, (byte)130, (byte)164, (byte)248, (byte)182, (byte)158, (byte)154, (byte)100, (byte)224, (byte)209, (byte)12, (byte)131, (byte)91, (byte)49, (byte)153, (byte)29, (byte)43, (byte)40, (byte)13, (byte)199, (byte)18, (byte)159, (byte)146, (byte)121, (byte)251, (byte)131, (byte)82, (byte)37, (byte)185, (byte)93, (byte)78}, 0) ;
            p123.target_component = (byte)(byte)67;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)1965866342156362817L);
                Debug.Assert(pack.cog == (ushort)(ushort)15369);
                Debug.Assert(pack.eph == (ushort)(ushort)28174);
                Debug.Assert(pack.lon == (int) -1171191950);
                Debug.Assert(pack.alt == (int)1758589694);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
                Debug.Assert(pack.dgps_numch == (byte)(byte)249);
                Debug.Assert(pack.satellites_visible == (byte)(byte)14);
                Debug.Assert(pack.vel == (ushort)(ushort)52315);
                Debug.Assert(pack.epv == (ushort)(ushort)31943);
                Debug.Assert(pack.lat == (int) -1193418895);
                Debug.Assert(pack.dgps_age == (uint)1719284170U);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.dgps_age = (uint)1719284170U;
            p124.dgps_numch = (byte)(byte)249;
            p124.time_usec = (ulong)1965866342156362817L;
            p124.eph = (ushort)(ushort)28174;
            p124.lat = (int) -1193418895;
            p124.alt = (int)1758589694;
            p124.cog = (ushort)(ushort)15369;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p124.epv = (ushort)(ushort)31943;
            p124.satellites_visible = (byte)(byte)14;
            p124.vel = (ushort)(ushort)52315;
            p124.lon = (int) -1171191950;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vservo == (ushort)(ushort)59802);
                Debug.Assert(pack.Vcc == (ushort)(ushort)13141);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID));
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID);
            p125.Vservo = (ushort)(ushort)59802;
            p125.Vcc = (ushort)(ushort)13141;
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)203, (byte)48, (byte)14, (byte)9, (byte)71, (byte)4, (byte)212, (byte)100, (byte)193, (byte)74, (byte)218, (byte)55, (byte)139, (byte)217, (byte)40, (byte)165, (byte)188, (byte)186, (byte)49, (byte)253, (byte)219, (byte)172, (byte)197, (byte)37, (byte)155, (byte)219, (byte)184, (byte)156, (byte)154, (byte)223, (byte)253, (byte)112, (byte)47, (byte)209, (byte)195, (byte)237, (byte)127, (byte)89, (byte)49, (byte)78, (byte)225, (byte)6, (byte)136, (byte)219, (byte)135, (byte)34, (byte)46, (byte)189, (byte)65, (byte)235, (byte)203, (byte)51, (byte)101, (byte)62, (byte)229, (byte)30, (byte)10, (byte)230, (byte)129, (byte)228, (byte)167, (byte)0, (byte)197, (byte)173, (byte)32, (byte)218, (byte)253, (byte)7, (byte)1, (byte)133}));
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE));
                Debug.Assert(pack.count == (byte)(byte)222);
                Debug.Assert(pack.timeout == (ushort)(ushort)30158);
                Debug.Assert(pack.baudrate == (uint)1048408332U);
                Debug.Assert(pack.device == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.timeout = (ushort)(ushort)30158;
            p126.baudrate = (uint)1048408332U;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
            p126.count = (byte)(byte)222;
            p126.data__SET(new byte[] {(byte)203, (byte)48, (byte)14, (byte)9, (byte)71, (byte)4, (byte)212, (byte)100, (byte)193, (byte)74, (byte)218, (byte)55, (byte)139, (byte)217, (byte)40, (byte)165, (byte)188, (byte)186, (byte)49, (byte)253, (byte)219, (byte)172, (byte)197, (byte)37, (byte)155, (byte)219, (byte)184, (byte)156, (byte)154, (byte)223, (byte)253, (byte)112, (byte)47, (byte)209, (byte)195, (byte)237, (byte)127, (byte)89, (byte)49, (byte)78, (byte)225, (byte)6, (byte)136, (byte)219, (byte)135, (byte)34, (byte)46, (byte)189, (byte)65, (byte)235, (byte)203, (byte)51, (byte)101, (byte)62, (byte)229, (byte)30, (byte)10, (byte)230, (byte)129, (byte)228, (byte)167, (byte)0, (byte)197, (byte)173, (byte)32, (byte)218, (byte)253, (byte)7, (byte)1, (byte)133}, 0) ;
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nsats == (byte)(byte)109);
                Debug.Assert(pack.baseline_c_mm == (int)1884421383);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)216);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)120);
                Debug.Assert(pack.iar_num_hypotheses == (int) -1526959633);
                Debug.Assert(pack.wn == (ushort)(ushort)26089);
                Debug.Assert(pack.time_last_baseline_ms == (uint)2081917138U);
                Debug.Assert(pack.tow == (uint)762752315U);
                Debug.Assert(pack.baseline_b_mm == (int) -2050203090);
                Debug.Assert(pack.rtk_health == (byte)(byte)56);
                Debug.Assert(pack.baseline_a_mm == (int)1115380230);
                Debug.Assert(pack.accuracy == (uint)1801397615U);
                Debug.Assert(pack.rtk_rate == (byte)(byte)220);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.baseline_b_mm = (int) -2050203090;
            p127.rtk_receiver_id = (byte)(byte)216;
            p127.wn = (ushort)(ushort)26089;
            p127.baseline_a_mm = (int)1115380230;
            p127.tow = (uint)762752315U;
            p127.baseline_c_mm = (int)1884421383;
            p127.nsats = (byte)(byte)109;
            p127.rtk_rate = (byte)(byte)220;
            p127.time_last_baseline_ms = (uint)2081917138U;
            p127.iar_num_hypotheses = (int) -1526959633;
            p127.rtk_health = (byte)(byte)56;
            p127.baseline_coords_type = (byte)(byte)120;
            p127.accuracy = (uint)1801397615U;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_a_mm == (int)135946515);
                Debug.Assert(pack.rtk_health == (byte)(byte)173);
                Debug.Assert(pack.baseline_c_mm == (int)458941189);
                Debug.Assert(pack.nsats == (byte)(byte)73);
                Debug.Assert(pack.time_last_baseline_ms == (uint)1555624574U);
                Debug.Assert(pack.rtk_rate == (byte)(byte)108);
                Debug.Assert(pack.iar_num_hypotheses == (int) -711883294);
                Debug.Assert(pack.baseline_b_mm == (int)757739271);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)85);
                Debug.Assert(pack.accuracy == (uint)180611798U);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)180);
                Debug.Assert(pack.tow == (uint)2386716641U);
                Debug.Assert(pack.wn == (ushort)(ushort)56514);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.rtk_rate = (byte)(byte)108;
            p128.tow = (uint)2386716641U;
            p128.iar_num_hypotheses = (int) -711883294;
            p128.baseline_a_mm = (int)135946515;
            p128.accuracy = (uint)180611798U;
            p128.wn = (ushort)(ushort)56514;
            p128.nsats = (byte)(byte)73;
            p128.rtk_receiver_id = (byte)(byte)85;
            p128.baseline_coords_type = (byte)(byte)180;
            p128.rtk_health = (byte)(byte)173;
            p128.baseline_c_mm = (int)458941189;
            p128.time_last_baseline_ms = (uint)1555624574U;
            p128.baseline_b_mm = (int)757739271;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short)18036);
                Debug.Assert(pack.xmag == (short)(short) -25172);
                Debug.Assert(pack.time_boot_ms == (uint)2312443934U);
                Debug.Assert(pack.ymag == (short)(short)9572);
                Debug.Assert(pack.zacc == (short)(short)14681);
                Debug.Assert(pack.zmag == (short)(short)2477);
                Debug.Assert(pack.xgyro == (short)(short) -30813);
                Debug.Assert(pack.ygyro == (short)(short) -8630);
                Debug.Assert(pack.xacc == (short)(short) -3721);
                Debug.Assert(pack.zgyro == (short)(short)17401);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)2312443934U;
            p129.yacc = (short)(short)18036;
            p129.xmag = (short)(short) -25172;
            p129.ygyro = (short)(short) -8630;
            p129.zgyro = (short)(short)17401;
            p129.ymag = (short)(short)9572;
            p129.zmag = (short)(short)2477;
            p129.zacc = (short)(short)14681;
            p129.xgyro = (short)(short) -30813;
            p129.xacc = (short)(short) -3721;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.width == (ushort)(ushort)47484);
                Debug.Assert(pack.size == (uint)3930784732U);
                Debug.Assert(pack.jpg_quality == (byte)(byte)221);
                Debug.Assert(pack.packets == (ushort)(ushort)49874);
                Debug.Assert(pack.payload == (byte)(byte)105);
                Debug.Assert(pack.type == (byte)(byte)123);
                Debug.Assert(pack.height == (ushort)(ushort)56350);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.packets = (ushort)(ushort)49874;
            p130.payload = (byte)(byte)105;
            p130.jpg_quality = (byte)(byte)221;
            p130.type = (byte)(byte)123;
            p130.size = (uint)3930784732U;
            p130.height = (ushort)(ushort)56350;
            p130.width = (ushort)(ushort)47484;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)62, (byte)149, (byte)55, (byte)185, (byte)92, (byte)18, (byte)132, (byte)71, (byte)27, (byte)45, (byte)53, (byte)216, (byte)171, (byte)117, (byte)246, (byte)0, (byte)181, (byte)149, (byte)137, (byte)127, (byte)255, (byte)168, (byte)156, (byte)90, (byte)154, (byte)100, (byte)100, (byte)230, (byte)33, (byte)195, (byte)21, (byte)228, (byte)38, (byte)34, (byte)10, (byte)143, (byte)208, (byte)220, (byte)133, (byte)208, (byte)78, (byte)184, (byte)89, (byte)223, (byte)20, (byte)210, (byte)224, (byte)29, (byte)36, (byte)85, (byte)18, (byte)246, (byte)171, (byte)12, (byte)93, (byte)194, (byte)216, (byte)160, (byte)248, (byte)202, (byte)15, (byte)127, (byte)112, (byte)101, (byte)187, (byte)53, (byte)36, (byte)245, (byte)181, (byte)91, (byte)68, (byte)231, (byte)36, (byte)85, (byte)82, (byte)170, (byte)151, (byte)30, (byte)106, (byte)40, (byte)192, (byte)57, (byte)107, (byte)163, (byte)28, (byte)161, (byte)184, (byte)10, (byte)112, (byte)66, (byte)204, (byte)179, (byte)170, (byte)160, (byte)188, (byte)172, (byte)244, (byte)84, (byte)119, (byte)120, (byte)29, (byte)239, (byte)145, (byte)33, (byte)117, (byte)42, (byte)151, (byte)128, (byte)103, (byte)245, (byte)182, (byte)196, (byte)61, (byte)100, (byte)65, (byte)157, (byte)8, (byte)195, (byte)212, (byte)74, (byte)40, (byte)252, (byte)65, (byte)64, (byte)67, (byte)209, (byte)249, (byte)107, (byte)109, (byte)128, (byte)160, (byte)187, (byte)36, (byte)244, (byte)134, (byte)182, (byte)118, (byte)148, (byte)13, (byte)232, (byte)139, (byte)144, (byte)164, (byte)116, (byte)138, (byte)202, (byte)66, (byte)176, (byte)189, (byte)57, (byte)103, (byte)76, (byte)46, (byte)128, (byte)178, (byte)187, (byte)58, (byte)254, (byte)98, (byte)152, (byte)198, (byte)255, (byte)222, (byte)249, (byte)124, (byte)237, (byte)63, (byte)227, (byte)84, (byte)32, (byte)135, (byte)40, (byte)142, (byte)185, (byte)233, (byte)124, (byte)200, (byte)70, (byte)27, (byte)107, (byte)234, (byte)84, (byte)253, (byte)134, (byte)241, (byte)126, (byte)231, (byte)248, (byte)60, (byte)159, (byte)16, (byte)130, (byte)82, (byte)47, (byte)244, (byte)184, (byte)64, (byte)200, (byte)174, (byte)181, (byte)90, (byte)255, (byte)54, (byte)28, (byte)218, (byte)140, (byte)55, (byte)196, (byte)71, (byte)47, (byte)190, (byte)59, (byte)37, (byte)127, (byte)75, (byte)135, (byte)78, (byte)160, (byte)48, (byte)33, (byte)154, (byte)67, (byte)146, (byte)76, (byte)131, (byte)119, (byte)178, (byte)100, (byte)52, (byte)157, (byte)190, (byte)6, (byte)252, (byte)2, (byte)142, (byte)74, (byte)5, (byte)60, (byte)218, (byte)165, (byte)179, (byte)133, (byte)25, (byte)169, (byte)111, (byte)17, (byte)44, (byte)207, (byte)112, (byte)11, (byte)77, (byte)235, (byte)195}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)61047);
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.data__SET(new byte[] {(byte)62, (byte)149, (byte)55, (byte)185, (byte)92, (byte)18, (byte)132, (byte)71, (byte)27, (byte)45, (byte)53, (byte)216, (byte)171, (byte)117, (byte)246, (byte)0, (byte)181, (byte)149, (byte)137, (byte)127, (byte)255, (byte)168, (byte)156, (byte)90, (byte)154, (byte)100, (byte)100, (byte)230, (byte)33, (byte)195, (byte)21, (byte)228, (byte)38, (byte)34, (byte)10, (byte)143, (byte)208, (byte)220, (byte)133, (byte)208, (byte)78, (byte)184, (byte)89, (byte)223, (byte)20, (byte)210, (byte)224, (byte)29, (byte)36, (byte)85, (byte)18, (byte)246, (byte)171, (byte)12, (byte)93, (byte)194, (byte)216, (byte)160, (byte)248, (byte)202, (byte)15, (byte)127, (byte)112, (byte)101, (byte)187, (byte)53, (byte)36, (byte)245, (byte)181, (byte)91, (byte)68, (byte)231, (byte)36, (byte)85, (byte)82, (byte)170, (byte)151, (byte)30, (byte)106, (byte)40, (byte)192, (byte)57, (byte)107, (byte)163, (byte)28, (byte)161, (byte)184, (byte)10, (byte)112, (byte)66, (byte)204, (byte)179, (byte)170, (byte)160, (byte)188, (byte)172, (byte)244, (byte)84, (byte)119, (byte)120, (byte)29, (byte)239, (byte)145, (byte)33, (byte)117, (byte)42, (byte)151, (byte)128, (byte)103, (byte)245, (byte)182, (byte)196, (byte)61, (byte)100, (byte)65, (byte)157, (byte)8, (byte)195, (byte)212, (byte)74, (byte)40, (byte)252, (byte)65, (byte)64, (byte)67, (byte)209, (byte)249, (byte)107, (byte)109, (byte)128, (byte)160, (byte)187, (byte)36, (byte)244, (byte)134, (byte)182, (byte)118, (byte)148, (byte)13, (byte)232, (byte)139, (byte)144, (byte)164, (byte)116, (byte)138, (byte)202, (byte)66, (byte)176, (byte)189, (byte)57, (byte)103, (byte)76, (byte)46, (byte)128, (byte)178, (byte)187, (byte)58, (byte)254, (byte)98, (byte)152, (byte)198, (byte)255, (byte)222, (byte)249, (byte)124, (byte)237, (byte)63, (byte)227, (byte)84, (byte)32, (byte)135, (byte)40, (byte)142, (byte)185, (byte)233, (byte)124, (byte)200, (byte)70, (byte)27, (byte)107, (byte)234, (byte)84, (byte)253, (byte)134, (byte)241, (byte)126, (byte)231, (byte)248, (byte)60, (byte)159, (byte)16, (byte)130, (byte)82, (byte)47, (byte)244, (byte)184, (byte)64, (byte)200, (byte)174, (byte)181, (byte)90, (byte)255, (byte)54, (byte)28, (byte)218, (byte)140, (byte)55, (byte)196, (byte)71, (byte)47, (byte)190, (byte)59, (byte)37, (byte)127, (byte)75, (byte)135, (byte)78, (byte)160, (byte)48, (byte)33, (byte)154, (byte)67, (byte)146, (byte)76, (byte)131, (byte)119, (byte)178, (byte)100, (byte)52, (byte)157, (byte)190, (byte)6, (byte)252, (byte)2, (byte)142, (byte)74, (byte)5, (byte)60, (byte)218, (byte)165, (byte)179, (byte)133, (byte)25, (byte)169, (byte)111, (byte)17, (byte)44, (byte)207, (byte)112, (byte)11, (byte)77, (byte)235, (byte)195}, 0) ;
            p131.seqnr = (ushort)(ushort)61047;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_distance == (ushort)(ushort)12249);
                Debug.Assert(pack.min_distance == (ushort)(ushort)51077);
                Debug.Assert(pack.orientation == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_PITCH_90);
                Debug.Assert(pack.time_boot_ms == (uint)2855456392U);
                Debug.Assert(pack.id == (byte)(byte)206);
                Debug.Assert(pack.covariance == (byte)(byte)215);
                Debug.Assert(pack.max_distance == (ushort)(ushort)14619);
                Debug.Assert(pack.type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)2855456392U;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_PITCH_90;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p132.current_distance = (ushort)(ushort)12249;
            p132.covariance = (byte)(byte)215;
            p132.min_distance = (ushort)(ushort)51077;
            p132.id = (byte)(byte)206;
            p132.max_distance = (ushort)(ushort)14619;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)43811);
                Debug.Assert(pack.lat == (int) -1772570283);
                Debug.Assert(pack.lon == (int)1581973101);
                Debug.Assert(pack.mask == (ulong)7654087143288600870L);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.grid_spacing = (ushort)(ushort)43811;
            p133.lat = (int) -1772570283;
            p133.mask = (ulong)7654087143288600870L;
            p133.lon = (int)1581973101;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -1560416666);
                Debug.Assert(pack.lat == (int)244521209);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)6331);
                Debug.Assert(pack.gridbit == (byte)(byte)81);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -10572, (short) -32199, (short)25130, (short)10484, (short) -149, (short)4056, (short) -21536, (short)8698, (short)4231, (short) -4664, (short)26410, (short) -7279, (short)7046, (short)28812, (short) -12614, (short)32578}));
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.data__SET(new short[] {(short) -10572, (short) -32199, (short)25130, (short)10484, (short) -149, (short)4056, (short) -21536, (short)8698, (short)4231, (short) -4664, (short)26410, (short) -7279, (short)7046, (short)28812, (short) -12614, (short)32578}, 0) ;
            p134.lat = (int)244521209;
            p134.gridbit = (byte)(byte)81;
            p134.grid_spacing = (ushort)(ushort)6331;
            p134.lon = (int) -1560416666;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)738704459);
                Debug.Assert(pack.lat == (int)547397345);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lon = (int)738704459;
            p135.lat = (int)547397345;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)980454093);
                Debug.Assert(pack.spacing == (ushort)(ushort)24668);
                Debug.Assert(pack.current_height == (float)2.2899895E38F);
                Debug.Assert(pack.loaded == (ushort)(ushort)37645);
                Debug.Assert(pack.terrain_height == (float) -2.4551311E38F);
                Debug.Assert(pack.lon == (int)298101687);
                Debug.Assert(pack.pending == (ushort)(ushort)4875);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.spacing = (ushort)(ushort)24668;
            p136.terrain_height = (float) -2.4551311E38F;
            p136.lon = (int)298101687;
            p136.pending = (ushort)(ushort)4875;
            p136.loaded = (ushort)(ushort)37645;
            p136.current_height = (float)2.2899895E38F;
            p136.lat = (int)980454093;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3668909381U);
                Debug.Assert(pack.temperature == (short)(short)2337);
                Debug.Assert(pack.press_diff == (float)1.7910155E38F);
                Debug.Assert(pack.press_abs == (float) -2.7432452E38F);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.temperature = (short)(short)2337;
            p137.time_boot_ms = (uint)3668909381U;
            p137.press_abs = (float) -2.7432452E38F;
            p137.press_diff = (float)1.7910155E38F;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)2.1019065E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.0882427E38F, -2.8201891E38F, -2.9321754E38F, -2.8808398E38F}));
                Debug.Assert(pack.y == (float)2.9496917E38F);
                Debug.Assert(pack.time_usec == (ulong)9177047564935103923L);
                Debug.Assert(pack.z == (float) -1.5900118E38F);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.x = (float)2.1019065E38F;
            p138.y = (float)2.9496917E38F;
            p138.q_SET(new float[] {-3.0882427E38F, -2.8201891E38F, -2.9321754E38F, -2.8808398E38F}, 0) ;
            p138.time_usec = (ulong)9177047564935103923L;
            p138.z = (float) -1.5900118E38F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)169);
                Debug.Assert(pack.time_usec == (ulong)7690513432987813157L);
                Debug.Assert(pack.group_mlx == (byte)(byte)144);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {2.5709765E38F, -2.3969537E38F, 5.8368434E37F, 8.572963E37F, 3.1242022E38F, 2.133514E38F, 2.7765854E38F, -2.9731452E37F}));
                Debug.Assert(pack.target_system == (byte)(byte)146);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)7690513432987813157L;
            p139.target_component = (byte)(byte)169;
            p139.group_mlx = (byte)(byte)144;
            p139.controls_SET(new float[] {2.5709765E38F, -2.3969537E38F, 5.8368434E37F, 8.572963E37F, 3.1242022E38F, 2.133514E38F, 2.7765854E38F, -2.9731452E37F}, 0) ;
            p139.target_system = (byte)(byte)146;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)83);
                Debug.Assert(pack.time_usec == (ulong)5097146081506954895L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {3.490798E37F, -1.7574033E38F, 2.67135E38F, -3.3732765E38F, 5.988135E37F, 2.0498936E38F, -2.6473088E38F, 8.801689E37F}));
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.group_mlx = (byte)(byte)83;
            p140.time_usec = (ulong)5097146081506954895L;
            p140.controls_SET(new float[] {3.490798E37F, -1.7574033E38F, 2.67135E38F, -3.3732765E38F, 5.988135E37F, 2.0498936E38F, -2.6473088E38F, 8.801689E37F}, 0) ;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_relative == (float)2.68396E38F);
                Debug.Assert(pack.altitude_terrain == (float)1.4515622E38F);
                Debug.Assert(pack.time_usec == (ulong)1712354078514709585L);
                Debug.Assert(pack.altitude_amsl == (float)3.2413228E38F);
                Debug.Assert(pack.bottom_clearance == (float)1.8371908E38F);
                Debug.Assert(pack.altitude_local == (float) -2.8964056E38F);
                Debug.Assert(pack.altitude_monotonic == (float)1.9541404E38F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_local = (float) -2.8964056E38F;
            p141.altitude_terrain = (float)1.4515622E38F;
            p141.altitude_amsl = (float)3.2413228E38F;
            p141.altitude_relative = (float)2.68396E38F;
            p141.bottom_clearance = (float)1.8371908E38F;
            p141.altitude_monotonic = (float)1.9541404E38F;
            p141.time_usec = (ulong)1712354078514709585L;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)87, (byte)27, (byte)73, (byte)174, (byte)150, (byte)83, (byte)8, (byte)215, (byte)86, (byte)172, (byte)186, (byte)255, (byte)46, (byte)115, (byte)2, (byte)60, (byte)197, (byte)28, (byte)141, (byte)215, (byte)102, (byte)232, (byte)18, (byte)21, (byte)80, (byte)103, (byte)231, (byte)182, (byte)127, (byte)80, (byte)74, (byte)94, (byte)192, (byte)184, (byte)244, (byte)153, (byte)92, (byte)137, (byte)151, (byte)151, (byte)27, (byte)182, (byte)112, (byte)197, (byte)35, (byte)66, (byte)239, (byte)244, (byte)165, (byte)126, (byte)137, (byte)95, (byte)22, (byte)69, (byte)143, (byte)171, (byte)216, (byte)30, (byte)133, (byte)99, (byte)31, (byte)91, (byte)18, (byte)107, (byte)253, (byte)198, (byte)209, (byte)36, (byte)249, (byte)136, (byte)45, (byte)125, (byte)125, (byte)21, (byte)100, (byte)22, (byte)80, (byte)189, (byte)237, (byte)138, (byte)161, (byte)159, (byte)164, (byte)135, (byte)249, (byte)113, (byte)19, (byte)113, (byte)5, (byte)18, (byte)175, (byte)64, (byte)241, (byte)147, (byte)75, (byte)53, (byte)106, (byte)2, (byte)32, (byte)62, (byte)125, (byte)34, (byte)243, (byte)131, (byte)198, (byte)228, (byte)127, (byte)143, (byte)13, (byte)202, (byte)53, (byte)184, (byte)229, (byte)189, (byte)136, (byte)129, (byte)240, (byte)106, (byte)230, (byte)81}));
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)83, (byte)159, (byte)14, (byte)35, (byte)209, (byte)24, (byte)251, (byte)140, (byte)62, (byte)247, (byte)222, (byte)140, (byte)124, (byte)152, (byte)61, (byte)240, (byte)254, (byte)63, (byte)23, (byte)228, (byte)177, (byte)121, (byte)197, (byte)188, (byte)115, (byte)94, (byte)9, (byte)23, (byte)4, (byte)70, (byte)65, (byte)193, (byte)204, (byte)66, (byte)74, (byte)69, (byte)209, (byte)40, (byte)116, (byte)119, (byte)144, (byte)141, (byte)223, (byte)5, (byte)35, (byte)121, (byte)7, (byte)53, (byte)40, (byte)153, (byte)144, (byte)107, (byte)48, (byte)197, (byte)229, (byte)249, (byte)25, (byte)113, (byte)148, (byte)187, (byte)119, (byte)43, (byte)174, (byte)37, (byte)108, (byte)249, (byte)114, (byte)185, (byte)0, (byte)123, (byte)43, (byte)59, (byte)53, (byte)238, (byte)28, (byte)3, (byte)243, (byte)68, (byte)98, (byte)203, (byte)249, (byte)247, (byte)72, (byte)103, (byte)152, (byte)25, (byte)132, (byte)202, (byte)247, (byte)247, (byte)193, (byte)120, (byte)16, (byte)103, (byte)196, (byte)125, (byte)136, (byte)179, (byte)195, (byte)169, (byte)123, (byte)23, (byte)53, (byte)167, (byte)158, (byte)49, (byte)87, (byte)241, (byte)136, (byte)11, (byte)228, (byte)187, (byte)86, (byte)86, (byte)66, (byte)147, (byte)236, (byte)127, (byte)153, (byte)134}));
                Debug.Assert(pack.request_id == (byte)(byte)121);
                Debug.Assert(pack.uri_type == (byte)(byte)119);
                Debug.Assert(pack.transfer_type == (byte)(byte)29);
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.uri_type = (byte)(byte)119;
            p142.uri_SET(new byte[] {(byte)83, (byte)159, (byte)14, (byte)35, (byte)209, (byte)24, (byte)251, (byte)140, (byte)62, (byte)247, (byte)222, (byte)140, (byte)124, (byte)152, (byte)61, (byte)240, (byte)254, (byte)63, (byte)23, (byte)228, (byte)177, (byte)121, (byte)197, (byte)188, (byte)115, (byte)94, (byte)9, (byte)23, (byte)4, (byte)70, (byte)65, (byte)193, (byte)204, (byte)66, (byte)74, (byte)69, (byte)209, (byte)40, (byte)116, (byte)119, (byte)144, (byte)141, (byte)223, (byte)5, (byte)35, (byte)121, (byte)7, (byte)53, (byte)40, (byte)153, (byte)144, (byte)107, (byte)48, (byte)197, (byte)229, (byte)249, (byte)25, (byte)113, (byte)148, (byte)187, (byte)119, (byte)43, (byte)174, (byte)37, (byte)108, (byte)249, (byte)114, (byte)185, (byte)0, (byte)123, (byte)43, (byte)59, (byte)53, (byte)238, (byte)28, (byte)3, (byte)243, (byte)68, (byte)98, (byte)203, (byte)249, (byte)247, (byte)72, (byte)103, (byte)152, (byte)25, (byte)132, (byte)202, (byte)247, (byte)247, (byte)193, (byte)120, (byte)16, (byte)103, (byte)196, (byte)125, (byte)136, (byte)179, (byte)195, (byte)169, (byte)123, (byte)23, (byte)53, (byte)167, (byte)158, (byte)49, (byte)87, (byte)241, (byte)136, (byte)11, (byte)228, (byte)187, (byte)86, (byte)86, (byte)66, (byte)147, (byte)236, (byte)127, (byte)153, (byte)134}, 0) ;
            p142.storage_SET(new byte[] {(byte)87, (byte)27, (byte)73, (byte)174, (byte)150, (byte)83, (byte)8, (byte)215, (byte)86, (byte)172, (byte)186, (byte)255, (byte)46, (byte)115, (byte)2, (byte)60, (byte)197, (byte)28, (byte)141, (byte)215, (byte)102, (byte)232, (byte)18, (byte)21, (byte)80, (byte)103, (byte)231, (byte)182, (byte)127, (byte)80, (byte)74, (byte)94, (byte)192, (byte)184, (byte)244, (byte)153, (byte)92, (byte)137, (byte)151, (byte)151, (byte)27, (byte)182, (byte)112, (byte)197, (byte)35, (byte)66, (byte)239, (byte)244, (byte)165, (byte)126, (byte)137, (byte)95, (byte)22, (byte)69, (byte)143, (byte)171, (byte)216, (byte)30, (byte)133, (byte)99, (byte)31, (byte)91, (byte)18, (byte)107, (byte)253, (byte)198, (byte)209, (byte)36, (byte)249, (byte)136, (byte)45, (byte)125, (byte)125, (byte)21, (byte)100, (byte)22, (byte)80, (byte)189, (byte)237, (byte)138, (byte)161, (byte)159, (byte)164, (byte)135, (byte)249, (byte)113, (byte)19, (byte)113, (byte)5, (byte)18, (byte)175, (byte)64, (byte)241, (byte)147, (byte)75, (byte)53, (byte)106, (byte)2, (byte)32, (byte)62, (byte)125, (byte)34, (byte)243, (byte)131, (byte)198, (byte)228, (byte)127, (byte)143, (byte)13, (byte)202, (byte)53, (byte)184, (byte)229, (byte)189, (byte)136, (byte)129, (byte)240, (byte)106, (byte)230, (byte)81}, 0) ;
            p142.transfer_type = (byte)(byte)29;
            p142.request_id = (byte)(byte)121;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float) -5.4571365E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3066788370U);
                Debug.Assert(pack.temperature == (short)(short) -21193);
                Debug.Assert(pack.press_abs == (float)8.241416E37F);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.press_diff = (float) -5.4571365E37F;
            p143.time_boot_ms = (uint)3066788370U;
            p143.press_abs = (float)8.241416E37F;
            p143.temperature = (short)(short) -21193;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.est_capabilities == (byte)(byte)55);
                Debug.Assert(pack.custom_state == (ulong)1281676553241844270L);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {-2.4670971E38F, 1.398905E38F, -9.292252E37F}));
                Debug.Assert(pack.acc.SequenceEqual(new float[] {-2.01419E38F, -1.45651E38F, -4.9689723E37F}));
                Debug.Assert(pack.vel.SequenceEqual(new float[] {2.9043402E37F, 7.326609E37F, 2.3615038E37F}));
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-1.9135543E38F, -1.1762494E38F, -1.3513716E38F, 2.9366856E38F}));
                Debug.Assert(pack.rates.SequenceEqual(new float[] {3.0499923E38F, -2.5184868E37F, -3.7440355E37F}));
                Debug.Assert(pack.timestamp == (ulong)869206501012814152L);
                Debug.Assert(pack.alt == (float)3.2019029E38F);
                Debug.Assert(pack.lat == (int)653171820);
                Debug.Assert(pack.lon == (int)792248969);
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.vel_SET(new float[] {2.9043402E37F, 7.326609E37F, 2.3615038E37F}, 0) ;
            p144.lat = (int)653171820;
            p144.position_cov_SET(new float[] {-2.4670971E38F, 1.398905E38F, -9.292252E37F}, 0) ;
            p144.acc_SET(new float[] {-2.01419E38F, -1.45651E38F, -4.9689723E37F}, 0) ;
            p144.timestamp = (ulong)869206501012814152L;
            p144.rates_SET(new float[] {3.0499923E38F, -2.5184868E37F, -3.7440355E37F}, 0) ;
            p144.attitude_q_SET(new float[] {-1.9135543E38F, -1.1762494E38F, -1.3513716E38F, 2.9366856E38F}, 0) ;
            p144.custom_state = (ulong)1281676553241844270L;
            p144.alt = (float)3.2019029E38F;
            p144.est_capabilities = (byte)(byte)55;
            p144.lon = (int)792248969;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z_vel == (float)4.663825E37F);
                Debug.Assert(pack.x_acc == (float) -1.4691152E38F);
                Debug.Assert(pack.time_usec == (ulong)5139833216317261099L);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {-2.3207593E38F, 3.0159253E38F, 1.5912767E38F}));
                Debug.Assert(pack.y_pos == (float) -7.778515E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-4.745372E37F, -1.5023329E38F, -3.2172244E38F, -2.4866486E38F}));
                Debug.Assert(pack.roll_rate == (float) -2.9920893E38F);
                Debug.Assert(pack.pitch_rate == (float)2.3231312E38F);
                Debug.Assert(pack.yaw_rate == (float) -2.2741962E38F);
                Debug.Assert(pack.airspeed == (float)2.4866488E38F);
                Debug.Assert(pack.x_vel == (float) -2.5915664E38F);
                Debug.Assert(pack.z_acc == (float)4.339252E37F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {-9.215485E36F, -1.3851941E38F, -1.0351525E38F}));
                Debug.Assert(pack.z_pos == (float)2.5111678E38F);
                Debug.Assert(pack.y_vel == (float) -3.349107E38F);
                Debug.Assert(pack.x_pos == (float)8.734502E37F);
                Debug.Assert(pack.y_acc == (float)2.586056E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.pitch_rate = (float)2.3231312E38F;
            p146.q_SET(new float[] {-4.745372E37F, -1.5023329E38F, -3.2172244E38F, -2.4866486E38F}, 0) ;
            p146.time_usec = (ulong)5139833216317261099L;
            p146.z_pos = (float)2.5111678E38F;
            p146.vel_variance_SET(new float[] {-9.215485E36F, -1.3851941E38F, -1.0351525E38F}, 0) ;
            p146.x_acc = (float) -1.4691152E38F;
            p146.roll_rate = (float) -2.9920893E38F;
            p146.y_acc = (float)2.586056E38F;
            p146.y_vel = (float) -3.349107E38F;
            p146.pos_variance_SET(new float[] {-2.3207593E38F, 3.0159253E38F, 1.5912767E38F}, 0) ;
            p146.x_vel = (float) -2.5915664E38F;
            p146.x_pos = (float)8.734502E37F;
            p146.airspeed = (float)2.4866488E38F;
            p146.z_acc = (float)4.339252E37F;
            p146.yaw_rate = (float) -2.2741962E38F;
            p146.y_pos = (float) -7.778515E37F;
            p146.z_vel = (float)4.663825E37F;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_consumed == (int)1855170024);
                Debug.Assert(pack.temperature == (short)(short)11262);
                Debug.Assert(pack.energy_consumed == (int) -1506404516);
                Debug.Assert(pack.type == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
                Debug.Assert(pack.current_battery == (short)(short)13412);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 84);
                Debug.Assert(pack.battery_function == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)31278, (ushort)30781, (ushort)12561, (ushort)56111, (ushort)3344, (ushort)39380, (ushort)41918, (ushort)31164, (ushort)39275, (ushort)49094}));
                Debug.Assert(pack.id == (byte)(byte)129);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)129;
            p147.temperature = (short)(short)11262;
            p147.energy_consumed = (int) -1506404516;
            p147.current_consumed = (int)1855170024;
            p147.battery_remaining = (sbyte)(sbyte) - 84;
            p147.voltages_SET(new ushort[] {(ushort)31278, (ushort)30781, (ushort)12561, (ushort)56111, (ushort)3344, (ushort)39380, (ushort)41918, (ushort)31164, (ushort)39275, (ushort)49094}, 0) ;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN;
            p147.current_battery = (short)(short)13412;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flight_sw_version == (uint)2655816191U);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)163, (byte)237, (byte)112, (byte)153, (byte)197, (byte)30, (byte)57, (byte)42, (byte)49, (byte)245, (byte)13, (byte)225, (byte)203, (byte)30, (byte)19, (byte)64, (byte)215, (byte)248}));
                Debug.Assert(pack.product_id == (ushort)(ushort)13186);
                Debug.Assert(pack.middleware_sw_version == (uint)4254818329U);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)48, (byte)66, (byte)74, (byte)165, (byte)251, (byte)48, (byte)209, (byte)25}));
                Debug.Assert(pack.vendor_id == (ushort)(ushort)40451);
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)250, (byte)30, (byte)175, (byte)107, (byte)111, (byte)80, (byte)86, (byte)60}));
                Debug.Assert(pack.board_version == (uint)3568313802U);
                Debug.Assert(pack.os_sw_version == (uint)3221244837U);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)209, (byte)52, (byte)48, (byte)154, (byte)137, (byte)45, (byte)86, (byte)220}));
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT));
                Debug.Assert(pack.uid == (ulong)8729203579769268148L);
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.flight_custom_version_SET(new byte[] {(byte)48, (byte)66, (byte)74, (byte)165, (byte)251, (byte)48, (byte)209, (byte)25}, 0) ;
            p148.uid = (ulong)8729203579769268148L;
            p148.os_custom_version_SET(new byte[] {(byte)250, (byte)30, (byte)175, (byte)107, (byte)111, (byte)80, (byte)86, (byte)60}, 0) ;
            p148.flight_sw_version = (uint)2655816191U;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT);
            p148.uid2_SET(new byte[] {(byte)163, (byte)237, (byte)112, (byte)153, (byte)197, (byte)30, (byte)57, (byte)42, (byte)49, (byte)245, (byte)13, (byte)225, (byte)203, (byte)30, (byte)19, (byte)64, (byte)215, (byte)248}, 0, PH) ;
            p148.middleware_custom_version_SET(new byte[] {(byte)209, (byte)52, (byte)48, (byte)154, (byte)137, (byte)45, (byte)86, (byte)220}, 0) ;
            p148.os_sw_version = (uint)3221244837U;
            p148.middleware_sw_version = (uint)4254818329U;
            p148.vendor_id = (ushort)(ushort)40451;
            p148.product_id = (ushort)(ushort)13186;
            p148.board_version = (uint)3568313802U;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.size_x == (float) -1.4516743E38F);
                Debug.Assert(pack.y_TRY(ph) == (float)2.3801574E36F);
                Debug.Assert(pack.time_usec == (ulong)1617072845724089825L);
                Debug.Assert(pack.distance == (float)2.869514E38F);
                Debug.Assert(pack.size_y == (float)1.7968783E38F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)171);
                Debug.Assert(pack.x_TRY(ph) == (float) -1.1847632E38F);
                Debug.Assert(pack.target_num == (byte)(byte)185);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {-2.5425071E38F, 1.3987728E38F, 9.761158E37F, 2.920717E38F}));
                Debug.Assert(pack.angle_y == (float) -2.0097401E37F);
                Debug.Assert(pack.angle_x == (float)2.5920214E38F);
                Debug.Assert(pack.type == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.z_TRY(ph) == (float)2.8663385E38F);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.x_SET((float) -1.1847632E38F, PH) ;
            p149.angle_y = (float) -2.0097401E37F;
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON;
            p149.z_SET((float)2.8663385E38F, PH) ;
            p149.time_usec = (ulong)1617072845724089825L;
            p149.y_SET((float)2.3801574E36F, PH) ;
            p149.target_num = (byte)(byte)185;
            p149.size_y = (float)1.7968783E38F;
            p149.size_x = (float) -1.4516743E38F;
            p149.position_valid_SET((byte)(byte)171, PH) ;
            p149.q_SET(new float[] {-2.5425071E38F, 1.3987728E38F, 9.761158E37F, 2.920717E38F}, 0, PH) ;
            p149.distance = (float)2.869514E38F;
            p149.frame = MAV_FRAME.MAV_FRAME_MISSION;
            p149.angle_x = (float)2.5920214E38F;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENS_POWERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.adc121_cs1_amp == (float) -3.3010985E38F);
                Debug.Assert(pack.adc121_vspb_volt == (float)5.221397E37F);
                Debug.Assert(pack.adc121_cs2_amp == (float) -2.2912783E38F);
                Debug.Assert(pack.adc121_cspb_amp == (float) -1.1691115E38F);
            };
            GroundControl.SENS_POWER p201 = CommunicationChannel.new_SENS_POWER();
            PH.setPack(p201);
            p201.adc121_cspb_amp = (float) -1.1691115E38F;
            p201.adc121_vspb_volt = (float)5.221397E37F;
            p201.adc121_cs1_amp = (float) -3.3010985E38F;
            p201.adc121_cs2_amp = (float) -2.2912783E38F;
            CommunicationChannel.instance.send(p201);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENS_MPPTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mppt1_status == (byte)(byte)180);
                Debug.Assert(pack.mppt1_pwm == (ushort)(ushort)38556);
                Debug.Assert(pack.mppt2_pwm == (ushort)(ushort)32123);
                Debug.Assert(pack.mppt3_amp == (float)4.687891E37F);
                Debug.Assert(pack.mppt2_amp == (float) -2.341066E38F);
                Debug.Assert(pack.mppt1_amp == (float)4.587742E37F);
                Debug.Assert(pack.mppt1_volt == (float) -9.727338E37F);
                Debug.Assert(pack.mppt3_status == (byte)(byte)150);
                Debug.Assert(pack.mppt2_status == (byte)(byte)199);
                Debug.Assert(pack.mppt_timestamp == (ulong)8760676680209294021L);
                Debug.Assert(pack.mppt3_pwm == (ushort)(ushort)10197);
                Debug.Assert(pack.mppt3_volt == (float)3.1682081E38F);
                Debug.Assert(pack.mppt2_volt == (float) -1.3286105E38F);
            };
            GroundControl.SENS_MPPT p202 = CommunicationChannel.new_SENS_MPPT();
            PH.setPack(p202);
            p202.mppt2_status = (byte)(byte)199;
            p202.mppt2_volt = (float) -1.3286105E38F;
            p202.mppt1_volt = (float) -9.727338E37F;
            p202.mppt3_pwm = (ushort)(ushort)10197;
            p202.mppt3_volt = (float)3.1682081E38F;
            p202.mppt1_amp = (float)4.587742E37F;
            p202.mppt1_status = (byte)(byte)180;
            p202.mppt2_amp = (float) -2.341066E38F;
            p202.mppt3_amp = (float)4.687891E37F;
            p202.mppt_timestamp = (ulong)8760676680209294021L;
            p202.mppt3_status = (byte)(byte)150;
            p202.mppt2_pwm = (ushort)(ushort)32123;
            p202.mppt1_pwm = (ushort)(ushort)38556;
            CommunicationChannel.instance.send(p202);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnASLCTRL_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uElev == (float)1.8669613E38F);
                Debug.Assert(pack.h == (float) -9.271929E37F);
                Debug.Assert(pack.PitchAngle == (float)2.369694E38F);
                Debug.Assert(pack.PitchAngleRef == (float) -3.1646258E38F);
                Debug.Assert(pack.hRef == (float) -3.6216907E37F);
                Debug.Assert(pack.RollAngleRef == (float)1.5634116E38F);
                Debug.Assert(pack.RollAngle == (float)1.0517192E38F);
                Debug.Assert(pack.r == (float) -2.6041932E38F);
                Debug.Assert(pack.uAil == (float) -1.8755716E38F);
                Debug.Assert(pack.SpoilersEngaged == (byte)(byte)32);
                Debug.Assert(pack.p == (float) -7.8580606E37F);
                Debug.Assert(pack.q == (float)3.1787767E38F);
                Debug.Assert(pack.AirspeedRef == (float)2.9585075E38F);
                Debug.Assert(pack.hRef_t == (float) -2.1553011E38F);
                Debug.Assert(pack.YawAngle == (float) -9.141222E37F);
                Debug.Assert(pack.uThrot2 == (float)2.2283229E38F);
                Debug.Assert(pack.rRef == (float) -3.2366438E38F);
                Debug.Assert(pack.YawAngleRef == (float) -2.0322007E38F);
                Debug.Assert(pack.nZ == (float) -3.4731788E37F);
                Debug.Assert(pack.timestamp == (ulong)273521509112900099L);
                Debug.Assert(pack.pRef == (float) -7.058789E37F);
                Debug.Assert(pack.uThrot == (float) -6.06081E37F);
                Debug.Assert(pack.aslctrl_mode == (byte)(byte)56);
                Debug.Assert(pack.qRef == (float) -2.0219775E38F);
                Debug.Assert(pack.uRud == (float) -1.3930936E37F);
            };
            GroundControl.ASLCTRL_DATA p203 = CommunicationChannel.new_ASLCTRL_DATA();
            PH.setPack(p203);
            p203.uAil = (float) -1.8755716E38F;
            p203.pRef = (float) -7.058789E37F;
            p203.r = (float) -2.6041932E38F;
            p203.RollAngle = (float)1.0517192E38F;
            p203.aslctrl_mode = (byte)(byte)56;
            p203.uThrot = (float) -6.06081E37F;
            p203.YawAngle = (float) -9.141222E37F;
            p203.hRef_t = (float) -2.1553011E38F;
            p203.uRud = (float) -1.3930936E37F;
            p203.uThrot2 = (float)2.2283229E38F;
            p203.p = (float) -7.8580606E37F;
            p203.nZ = (float) -3.4731788E37F;
            p203.YawAngleRef = (float) -2.0322007E38F;
            p203.PitchAngle = (float)2.369694E38F;
            p203.h = (float) -9.271929E37F;
            p203.uElev = (float)1.8669613E38F;
            p203.qRef = (float) -2.0219775E38F;
            p203.q = (float)3.1787767E38F;
            p203.SpoilersEngaged = (byte)(byte)32;
            p203.PitchAngleRef = (float) -3.1646258E38F;
            p203.hRef = (float) -3.6216907E37F;
            p203.rRef = (float) -3.2366438E38F;
            p203.AirspeedRef = (float)2.9585075E38F;
            p203.RollAngleRef = (float)1.5634116E38F;
            p203.timestamp = (ulong)273521509112900099L;
            CommunicationChannel.instance.send(p203);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnASLCTRL_DEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.i32_1 == (uint)154294060U);
                Debug.Assert(pack.f_2 == (float)5.0643427E37F);
                Debug.Assert(pack.f_7 == (float)2.5198247E38F);
                Debug.Assert(pack.f_6 == (float)1.7457378E38F);
                Debug.Assert(pack.i8_1 == (byte)(byte)21);
                Debug.Assert(pack.f_8 == (float) -2.7894696E38F);
                Debug.Assert(pack.f_5 == (float) -8.514714E37F);
                Debug.Assert(pack.f_4 == (float)1.1632315E38F);
                Debug.Assert(pack.i8_2 == (byte)(byte)16);
                Debug.Assert(pack.f_3 == (float)6.8048757E37F);
                Debug.Assert(pack.f_1 == (float) -1.7561516E38F);
            };
            GroundControl.ASLCTRL_DEBUG p204 = CommunicationChannel.new_ASLCTRL_DEBUG();
            PH.setPack(p204);
            p204.f_1 = (float) -1.7561516E38F;
            p204.i32_1 = (uint)154294060U;
            p204.f_7 = (float)2.5198247E38F;
            p204.f_8 = (float) -2.7894696E38F;
            p204.f_4 = (float)1.1632315E38F;
            p204.f_2 = (float)5.0643427E37F;
            p204.f_5 = (float) -8.514714E37F;
            p204.f_6 = (float)1.7457378E38F;
            p204.i8_2 = (byte)(byte)16;
            p204.i8_1 = (byte)(byte)21;
            p204.f_3 = (float)6.8048757E37F;
            CommunicationChannel.instance.send(p204);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnASLUAV_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Motor_rpm == (float) -3.280412E38F);
                Debug.Assert(pack.LED_status == (byte)(byte)147);
                Debug.Assert(pack.Servo_status.SequenceEqual(new byte[] {(byte)249, (byte)157, (byte)147, (byte)61, (byte)210, (byte)177, (byte)75, (byte)216}));
                Debug.Assert(pack.SATCOM_status == (byte)(byte)105);
            };
            GroundControl.ASLUAV_STATUS p205 = CommunicationChannel.new_ASLUAV_STATUS();
            PH.setPack(p205);
            p205.Motor_rpm = (float) -3.280412E38F;
            p205.Servo_status_SET(new byte[] {(byte)249, (byte)157, (byte)147, (byte)61, (byte)210, (byte)177, (byte)75, (byte)216}, 0) ;
            p205.LED_status = (byte)(byte)147;
            p205.SATCOM_status = (byte)(byte)105;
            CommunicationChannel.instance.send(p205);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnEKF_EXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.WindDir == (float) -2.8949503E38F);
                Debug.Assert(pack.beta == (float)8.349425E37F);
                Debug.Assert(pack.alpha == (float)3.0068778E38F);
                Debug.Assert(pack.Windspeed == (float)2.0937312E38F);
                Debug.Assert(pack.Airspeed == (float)1.0793232E38F);
                Debug.Assert(pack.WindZ == (float) -2.3735005E38F);
                Debug.Assert(pack.timestamp == (ulong)1993181284455547070L);
            };
            GroundControl.EKF_EXT p206 = CommunicationChannel.new_EKF_EXT();
            PH.setPack(p206);
            p206.WindDir = (float) -2.8949503E38F;
            p206.alpha = (float)3.0068778E38F;
            p206.Windspeed = (float)2.0937312E38F;
            p206.beta = (float)8.349425E37F;
            p206.timestamp = (ulong)1993181284455547070L;
            p206.Airspeed = (float)1.0793232E38F;
            p206.WindZ = (float) -2.3735005E38F;
            CommunicationChannel.instance.send(p206);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnASL_OBCTRLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uThrot2 == (float) -2.8229634E38F);
                Debug.Assert(pack.timestamp == (ulong)5254640492514917551L);
                Debug.Assert(pack.uAilR == (float) -3.3741003E38F);
                Debug.Assert(pack.uAilL == (float) -2.3982126E38F);
                Debug.Assert(pack.uRud == (float)9.504707E37F);
                Debug.Assert(pack.uElev == (float) -2.8238025E38F);
                Debug.Assert(pack.uThrot == (float)2.9104063E38F);
                Debug.Assert(pack.obctrl_status == (byte)(byte)224);
            };
            GroundControl.ASL_OBCTRL p207 = CommunicationChannel.new_ASL_OBCTRL();
            PH.setPack(p207);
            p207.uAilL = (float) -2.3982126E38F;
            p207.uElev = (float) -2.8238025E38F;
            p207.timestamp = (ulong)5254640492514917551L;
            p207.obctrl_status = (byte)(byte)224;
            p207.uRud = (float)9.504707E37F;
            p207.uThrot = (float)2.9104063E38F;
            p207.uAilR = (float) -3.3741003E38F;
            p207.uThrot2 = (float) -2.8229634E38F;
            CommunicationChannel.instance.send(p207);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENS_ATMOSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Humidity == (float)2.869636E38F);
                Debug.Assert(pack.TempAmbient == (float) -2.2223112E38F);
            };
            GroundControl.SENS_ATMOS p208 = CommunicationChannel.new_SENS_ATMOS();
            PH.setPack(p208);
            p208.Humidity = (float)2.869636E38F;
            p208.TempAmbient = (float) -2.2223112E38F;
            CommunicationChannel.instance.send(p208);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENS_BATMONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cellvoltage6 == (ushort)(ushort)23810);
                Debug.Assert(pack.batterystatus == (ushort)(ushort)63588);
                Debug.Assert(pack.cellvoltage1 == (ushort)(ushort)55294);
                Debug.Assert(pack.hostfetcontrol == (ushort)(ushort)44192);
                Debug.Assert(pack.temperature == (float)1.5613533E38F);
                Debug.Assert(pack.voltage == (ushort)(ushort)1229);
                Debug.Assert(pack.SoC == (byte)(byte)59);
                Debug.Assert(pack.cellvoltage2 == (ushort)(ushort)27099);
                Debug.Assert(pack.cellvoltage3 == (ushort)(ushort)26472);
                Debug.Assert(pack.current == (short)(short) -1218);
                Debug.Assert(pack.cellvoltage4 == (ushort)(ushort)62322);
                Debug.Assert(pack.cellvoltage5 == (ushort)(ushort)2677);
                Debug.Assert(pack.serialnumber == (ushort)(ushort)6826);
            };
            GroundControl.SENS_BATMON p209 = CommunicationChannel.new_SENS_BATMON();
            PH.setPack(p209);
            p209.cellvoltage1 = (ushort)(ushort)55294;
            p209.current = (short)(short) -1218;
            p209.cellvoltage3 = (ushort)(ushort)26472;
            p209.voltage = (ushort)(ushort)1229;
            p209.batterystatus = (ushort)(ushort)63588;
            p209.serialnumber = (ushort)(ushort)6826;
            p209.temperature = (float)1.5613533E38F;
            p209.cellvoltage6 = (ushort)(ushort)23810;
            p209.hostfetcontrol = (ushort)(ushort)44192;
            p209.cellvoltage2 = (ushort)(ushort)27099;
            p209.cellvoltage5 = (ushort)(ushort)2677;
            p209.SoC = (byte)(byte)59;
            p209.cellvoltage4 = (ushort)(ushort)62322;
            CommunicationChannel.instance.send(p209);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFW_SOARING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z2_DeltaRoll == (float) -1.4076246E37F);
                Debug.Assert(pack.LoiterRadius == (float)1.159964E38F);
                Debug.Assert(pack.xLon == (float) -3.198239E38F);
                Debug.Assert(pack.DebugVar1 == (float)5.980986E37F);
                Debug.Assert(pack.VarR == (float) -2.2625006E38F);
                Debug.Assert(pack.z1_exp == (float)3.3850877E38F);
                Debug.Assert(pack.xLat == (float) -2.7429915E38F);
                Debug.Assert(pack.timestamp == (ulong)2568497473381882039L);
                Debug.Assert(pack.DebugVar2 == (float)9.412364E37F);
                Debug.Assert(pack.VarW == (float) -2.9388443E38F);
                Debug.Assert(pack.ThermalGSNorth == (float) -1.4025126E38F);
                Debug.Assert(pack.TSE_dot == (float) -1.3348864E38F);
                Debug.Assert(pack.VarLon == (float) -3.080117E38F);
                Debug.Assert(pack.valid == (byte)(byte)32);
                Debug.Assert(pack.z2_exp == (float)1.9953975E38F);
                Debug.Assert(pack.DistToSoarPoint == (float) -9.695087E37F);
                Debug.Assert(pack.VarLat == (float)3.2739933E38F);
                Debug.Assert(pack.vSinkExp == (float)2.5917269E38F);
                Debug.Assert(pack.LoiterDirection == (float) -1.7823928E38F);
                Debug.Assert(pack.ControlMode == (byte)(byte)129);
                Debug.Assert(pack.z1_LocalUpdraftSpeed == (float)5.1330114E37F);
                Debug.Assert(pack.ThermalGSEast == (float) -2.8725218E38F);
                Debug.Assert(pack.xW == (float) -2.2136078E38F);
                Debug.Assert(pack.timestampModeChanged == (ulong)8095525426644489008L);
                Debug.Assert(pack.xR == (float) -3.1640622E38F);
            };
            GroundControl.FW_SOARING_DATA p210 = CommunicationChannel.new_FW_SOARING_DATA();
            PH.setPack(p210);
            p210.xLat = (float) -2.7429915E38F;
            p210.DebugVar1 = (float)5.980986E37F;
            p210.TSE_dot = (float) -1.3348864E38F;
            p210.z2_exp = (float)1.9953975E38F;
            p210.xLon = (float) -3.198239E38F;
            p210.ThermalGSNorth = (float) -1.4025126E38F;
            p210.xR = (float) -3.1640622E38F;
            p210.xW = (float) -2.2136078E38F;
            p210.VarLat = (float)3.2739933E38F;
            p210.z1_LocalUpdraftSpeed = (float)5.1330114E37F;
            p210.LoiterDirection = (float) -1.7823928E38F;
            p210.VarW = (float) -2.9388443E38F;
            p210.valid = (byte)(byte)32;
            p210.z2_DeltaRoll = (float) -1.4076246E37F;
            p210.VarR = (float) -2.2625006E38F;
            p210.ControlMode = (byte)(byte)129;
            p210.DebugVar2 = (float)9.412364E37F;
            p210.timestampModeChanged = (ulong)8095525426644489008L;
            p210.z1_exp = (float)3.3850877E38F;
            p210.VarLon = (float) -3.080117E38F;
            p210.vSinkExp = (float)2.5917269E38F;
            p210.LoiterRadius = (float)1.159964E38F;
            p210.ThermalGSEast = (float) -2.8725218E38F;
            p210.timestamp = (ulong)2568497473381882039L;
            p210.DistToSoarPoint = (float) -9.695087E37F;
            CommunicationChannel.instance.send(p210);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENSORPOD_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.timestamp == (ulong)3424124738381259515L);
                Debug.Assert(pack.visensor_rate_1 == (byte)(byte)37);
                Debug.Assert(pack.visensor_rate_2 == (byte)(byte)236);
                Debug.Assert(pack.visensor_rate_4 == (byte)(byte)2);
                Debug.Assert(pack.visensor_rate_3 == (byte)(byte)46);
                Debug.Assert(pack.free_space == (ushort)(ushort)65420);
                Debug.Assert(pack.cpu_temp == (byte)(byte)134);
                Debug.Assert(pack.recording_nodes_count == (byte)(byte)74);
            };
            GroundControl.SENSORPOD_STATUS p211 = CommunicationChannel.new_SENSORPOD_STATUS();
            PH.setPack(p211);
            p211.recording_nodes_count = (byte)(byte)74;
            p211.cpu_temp = (byte)(byte)134;
            p211.free_space = (ushort)(ushort)65420;
            p211.visensor_rate_2 = (byte)(byte)236;
            p211.visensor_rate_4 = (byte)(byte)2;
            p211.timestamp = (ulong)3424124738381259515L;
            p211.visensor_rate_1 = (byte)(byte)37;
            p211.visensor_rate_3 = (byte)(byte)46;
            CommunicationChannel.instance.send(p211);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENS_POWER_BOARDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pwr_brd_servo_volt == (float) -2.6754445E38F);
                Debug.Assert(pack.pwr_brd_status == (byte)(byte)42);
                Debug.Assert(pack.timestamp == (ulong)4886937017616027114L);
                Debug.Assert(pack.pwr_brd_aux_amp == (float)3.1561735E38F);
                Debug.Assert(pack.pwr_brd_servo_2_amp == (float) -1.0279555E38F);
                Debug.Assert(pack.pwr_brd_servo_3_amp == (float) -2.418125E38F);
                Debug.Assert(pack.pwr_brd_led_status == (byte)(byte)133);
                Debug.Assert(pack.pwr_brd_servo_1_amp == (float)1.4334048E38F);
                Debug.Assert(pack.pwr_brd_servo_4_amp == (float) -2.793305E38F);
                Debug.Assert(pack.pwr_brd_system_volt == (float)1.264307E37F);
                Debug.Assert(pack.pwr_brd_mot_r_amp == (float) -7.0053374E37F);
                Debug.Assert(pack.pwr_brd_mot_l_amp == (float)2.2181508E38F);
            };
            GroundControl.SENS_POWER_BOARD p212 = CommunicationChannel.new_SENS_POWER_BOARD();
            PH.setPack(p212);
            p212.pwr_brd_system_volt = (float)1.264307E37F;
            p212.pwr_brd_aux_amp = (float)3.1561735E38F;
            p212.pwr_brd_servo_volt = (float) -2.6754445E38F;
            p212.pwr_brd_led_status = (byte)(byte)133;
            p212.pwr_brd_servo_2_amp = (float) -1.0279555E38F;
            p212.pwr_brd_mot_r_amp = (float) -7.0053374E37F;
            p212.timestamp = (ulong)4886937017616027114L;
            p212.pwr_brd_status = (byte)(byte)42;
            p212.pwr_brd_servo_3_amp = (float) -2.418125E38F;
            p212.pwr_brd_servo_4_amp = (float) -2.793305E38F;
            p212.pwr_brd_servo_1_amp = (float)1.4334048E38F;
            p212.pwr_brd_mot_l_amp = (float)2.2181508E38F;
            CommunicationChannel.instance.send(p212);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pos_vert_ratio == (float) -2.0525608E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float)1.3137298E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE));
                Debug.Assert(pack.vel_ratio == (float)1.5899702E38F);
                Debug.Assert(pack.pos_vert_accuracy == (float)1.9536532E38F);
                Debug.Assert(pack.hagl_ratio == (float)5.3818147E37F);
                Debug.Assert(pack.time_usec == (ulong)3400694253598437804L);
                Debug.Assert(pack.tas_ratio == (float) -2.1746583E38F);
                Debug.Assert(pack.mag_ratio == (float) -1.8223274E38F);
                Debug.Assert(pack.pos_horiz_ratio == (float)2.9841274E38F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.tas_ratio = (float) -2.1746583E38F;
            p230.pos_horiz_accuracy = (float)1.3137298E38F;
            p230.vel_ratio = (float)1.5899702E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE);
            p230.pos_horiz_ratio = (float)2.9841274E38F;
            p230.mag_ratio = (float) -1.8223274E38F;
            p230.hagl_ratio = (float)5.3818147E37F;
            p230.pos_vert_accuracy = (float)1.9536532E38F;
            p230.time_usec = (ulong)3400694253598437804L;
            p230.pos_vert_ratio = (float) -2.0525608E38F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vert_accuracy == (float) -3.3029555E36F);
                Debug.Assert(pack.time_usec == (ulong)9176178892705979822L);
                Debug.Assert(pack.var_vert == (float) -5.7807697E37F);
                Debug.Assert(pack.wind_z == (float) -6.9452066E37F);
                Debug.Assert(pack.wind_alt == (float)2.7021536E38F);
                Debug.Assert(pack.wind_y == (float) -2.179485E37F);
                Debug.Assert(pack.horiz_accuracy == (float)1.6995674E37F);
                Debug.Assert(pack.wind_x == (float) -8.29556E37F);
                Debug.Assert(pack.var_horiz == (float)1.5034739E38F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.horiz_accuracy = (float)1.6995674E37F;
            p231.wind_x = (float) -8.29556E37F;
            p231.vert_accuracy = (float) -3.3029555E36F;
            p231.wind_alt = (float)2.7021536E38F;
            p231.var_vert = (float) -5.7807697E37F;
            p231.var_horiz = (float)1.5034739E38F;
            p231.time_usec = (ulong)9176178892705979822L;
            p231.wind_z = (float) -6.9452066E37F;
            p231.wind_y = (float) -2.179485E37F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vert_accuracy == (float)2.705055E38F);
                Debug.Assert(pack.time_usec == (ulong)9025773388342488087L);
                Debug.Assert(pack.hdop == (float) -2.3904187E38F);
                Debug.Assert(pack.ve == (float)1.4289788E38F);
                Debug.Assert(pack.vdop == (float) -2.5411602E38F);
                Debug.Assert(pack.satellites_visible == (byte)(byte)187);
                Debug.Assert(pack.time_week == (ushort)(ushort)63534);
                Debug.Assert(pack.lon == (int)2112260093);
                Debug.Assert(pack.fix_type == (byte)(byte)13);
                Debug.Assert(pack.horiz_accuracy == (float)3.3926291E38F);
                Debug.Assert(pack.lat == (int) -1082953361);
                Debug.Assert(pack.alt == (float) -4.256152E37F);
                Debug.Assert(pack.time_week_ms == (uint)258994350U);
                Debug.Assert(pack.speed_accuracy == (float)1.3806493E38F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY));
                Debug.Assert(pack.vd == (float)1.0484023E38F);
                Debug.Assert(pack.gps_id == (byte)(byte)149);
                Debug.Assert(pack.vn == (float)1.6894803E38F);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY);
            p232.hdop = (float) -2.3904187E38F;
            p232.vdop = (float) -2.5411602E38F;
            p232.vert_accuracy = (float)2.705055E38F;
            p232.fix_type = (byte)(byte)13;
            p232.time_week_ms = (uint)258994350U;
            p232.alt = (float) -4.256152E37F;
            p232.vn = (float)1.6894803E38F;
            p232.time_week = (ushort)(ushort)63534;
            p232.speed_accuracy = (float)1.3806493E38F;
            p232.lon = (int)2112260093;
            p232.lat = (int) -1082953361;
            p232.satellites_visible = (byte)(byte)187;
            p232.gps_id = (byte)(byte)149;
            p232.ve = (float)1.4289788E38F;
            p232.time_usec = (ulong)9025773388342488087L;
            p232.vd = (float)1.0484023E38F;
            p232.horiz_accuracy = (float)3.3926291E38F;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)239);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)32, (byte)187, (byte)15, (byte)192, (byte)216, (byte)139, (byte)85, (byte)35, (byte)92, (byte)66, (byte)158, (byte)12, (byte)166, (byte)55, (byte)137, (byte)155, (byte)148, (byte)144, (byte)101, (byte)37, (byte)126, (byte)172, (byte)141, (byte)192, (byte)27, (byte)23, (byte)94, (byte)17, (byte)77, (byte)164, (byte)117, (byte)240, (byte)171, (byte)67, (byte)123, (byte)48, (byte)43, (byte)58, (byte)156, (byte)188, (byte)156, (byte)91, (byte)82, (byte)33, (byte)143, (byte)237, (byte)105, (byte)88, (byte)143, (byte)54, (byte)180, (byte)164, (byte)236, (byte)27, (byte)53, (byte)69, (byte)215, (byte)21, (byte)78, (byte)122, (byte)82, (byte)70, (byte)34, (byte)78, (byte)233, (byte)115, (byte)0, (byte)103, (byte)109, (byte)55, (byte)23, (byte)197, (byte)143, (byte)60, (byte)200, (byte)49, (byte)179, (byte)120, (byte)136, (byte)108, (byte)160, (byte)149, (byte)20, (byte)195, (byte)83, (byte)196, (byte)235, (byte)168, (byte)241, (byte)239, (byte)154, (byte)83, (byte)143, (byte)109, (byte)191, (byte)196, (byte)61, (byte)176, (byte)68, (byte)204, (byte)33, (byte)155, (byte)134, (byte)240, (byte)223, (byte)163, (byte)128, (byte)124, (byte)172, (byte)48, (byte)120, (byte)175, (byte)84, (byte)173, (byte)158, (byte)60, (byte)202, (byte)75, (byte)161, (byte)129, (byte)123, (byte)254, (byte)89, (byte)112, (byte)24, (byte)205, (byte)82, (byte)124, (byte)91, (byte)244, (byte)80, (byte)98, (byte)15, (byte)76, (byte)32, (byte)194, (byte)252, (byte)205, (byte)186, (byte)179, (byte)214, (byte)176, (byte)239, (byte)4, (byte)55, (byte)111, (byte)128, (byte)182, (byte)178, (byte)9, (byte)118, (byte)188, (byte)17, (byte)127, (byte)152, (byte)219, (byte)82, (byte)200, (byte)124, (byte)28, (byte)23, (byte)240, (byte)177, (byte)24, (byte)56, (byte)45, (byte)33, (byte)79, (byte)154, (byte)199, (byte)116, (byte)13, (byte)32, (byte)31, (byte)212, (byte)144, (byte)36, (byte)39, (byte)101, (byte)238}));
                Debug.Assert(pack.flags == (byte)(byte)39);
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.len = (byte)(byte)239;
            p233.flags = (byte)(byte)39;
            p233.data__SET(new byte[] {(byte)32, (byte)187, (byte)15, (byte)192, (byte)216, (byte)139, (byte)85, (byte)35, (byte)92, (byte)66, (byte)158, (byte)12, (byte)166, (byte)55, (byte)137, (byte)155, (byte)148, (byte)144, (byte)101, (byte)37, (byte)126, (byte)172, (byte)141, (byte)192, (byte)27, (byte)23, (byte)94, (byte)17, (byte)77, (byte)164, (byte)117, (byte)240, (byte)171, (byte)67, (byte)123, (byte)48, (byte)43, (byte)58, (byte)156, (byte)188, (byte)156, (byte)91, (byte)82, (byte)33, (byte)143, (byte)237, (byte)105, (byte)88, (byte)143, (byte)54, (byte)180, (byte)164, (byte)236, (byte)27, (byte)53, (byte)69, (byte)215, (byte)21, (byte)78, (byte)122, (byte)82, (byte)70, (byte)34, (byte)78, (byte)233, (byte)115, (byte)0, (byte)103, (byte)109, (byte)55, (byte)23, (byte)197, (byte)143, (byte)60, (byte)200, (byte)49, (byte)179, (byte)120, (byte)136, (byte)108, (byte)160, (byte)149, (byte)20, (byte)195, (byte)83, (byte)196, (byte)235, (byte)168, (byte)241, (byte)239, (byte)154, (byte)83, (byte)143, (byte)109, (byte)191, (byte)196, (byte)61, (byte)176, (byte)68, (byte)204, (byte)33, (byte)155, (byte)134, (byte)240, (byte)223, (byte)163, (byte)128, (byte)124, (byte)172, (byte)48, (byte)120, (byte)175, (byte)84, (byte)173, (byte)158, (byte)60, (byte)202, (byte)75, (byte)161, (byte)129, (byte)123, (byte)254, (byte)89, (byte)112, (byte)24, (byte)205, (byte)82, (byte)124, (byte)91, (byte)244, (byte)80, (byte)98, (byte)15, (byte)76, (byte)32, (byte)194, (byte)252, (byte)205, (byte)186, (byte)179, (byte)214, (byte)176, (byte)239, (byte)4, (byte)55, (byte)111, (byte)128, (byte)182, (byte)178, (byte)9, (byte)118, (byte)188, (byte)17, (byte)127, (byte)152, (byte)219, (byte)82, (byte)200, (byte)124, (byte)28, (byte)23, (byte)240, (byte)177, (byte)24, (byte)56, (byte)45, (byte)33, (byte)79, (byte)154, (byte)199, (byte)116, (byte)13, (byte)32, (byte)31, (byte)212, (byte)144, (byte)36, (byte)39, (byte)101, (byte)238}, 0) ;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
                Debug.Assert(pack.custom_mode == (uint)4115337523U);
                Debug.Assert(pack.gps_fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
                Debug.Assert(pack.heading_sp == (short)(short) -13411);
                Debug.Assert(pack.battery_remaining == (byte)(byte)246);
                Debug.Assert(pack.roll == (short)(short)887);
                Debug.Assert(pack.latitude == (int) -804147319);
                Debug.Assert(pack.failsafe == (byte)(byte)45);
                Debug.Assert(pack.groundspeed == (byte)(byte)111);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte) - 114);
                Debug.Assert(pack.temperature == (sbyte)(sbyte) - 91);
                Debug.Assert(pack.throttle == (sbyte)(sbyte) - 93);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)1989);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED));
                Debug.Assert(pack.airspeed_sp == (byte)(byte)157);
                Debug.Assert(pack.heading == (ushort)(ushort)10241);
                Debug.Assert(pack.wp_num == (byte)(byte)35);
                Debug.Assert(pack.airspeed == (byte)(byte)85);
                Debug.Assert(pack.altitude_amsl == (short)(short) -18358);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 20);
                Debug.Assert(pack.altitude_sp == (short)(short)20674);
                Debug.Assert(pack.gps_nsat == (byte)(byte)121);
                Debug.Assert(pack.longitude == (int) -1801626927);
                Debug.Assert(pack.pitch == (short)(short) -19484);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.temperature_air = (sbyte)(sbyte) - 114;
            p234.altitude_sp = (short)(short)20674;
            p234.failsafe = (byte)(byte)45;
            p234.heading = (ushort)(ushort)10241;
            p234.throttle = (sbyte)(sbyte) - 93;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            p234.airspeed_sp = (byte)(byte)157;
            p234.climb_rate = (sbyte)(sbyte) - 20;
            p234.pitch = (short)(short) -19484;
            p234.groundspeed = (byte)(byte)111;
            p234.latitude = (int) -804147319;
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
            p234.longitude = (int) -1801626927;
            p234.airspeed = (byte)(byte)85;
            p234.heading_sp = (short)(short) -13411;
            p234.altitude_amsl = (short)(short) -18358;
            p234.temperature = (sbyte)(sbyte) - 91;
            p234.wp_num = (byte)(byte)35;
            p234.battery_remaining = (byte)(byte)246;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX;
            p234.wp_distance = (ushort)(ushort)1989;
            p234.roll = (short)(short)887;
            p234.custom_mode = (uint)4115337523U;
            p234.gps_nsat = (byte)(byte)121;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.clipping_0 == (uint)2020991538U);
                Debug.Assert(pack.vibration_z == (float)1.629437E38F);
                Debug.Assert(pack.time_usec == (ulong)6655392377730624145L);
                Debug.Assert(pack.clipping_2 == (uint)206188163U);
                Debug.Assert(pack.clipping_1 == (uint)30938606U);
                Debug.Assert(pack.vibration_y == (float) -2.543987E38F);
                Debug.Assert(pack.vibration_x == (float) -8.1406143E37F);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.vibration_z = (float)1.629437E38F;
            p241.vibration_x = (float) -8.1406143E37F;
            p241.clipping_1 = (uint)30938606U;
            p241.vibration_y = (float) -2.543987E38F;
            p241.clipping_0 = (uint)2020991538U;
            p241.time_usec = (ulong)6655392377730624145L;
            p241.clipping_2 = (uint)206188163U;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int)1822410779);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)4546560617307335987L);
                Debug.Assert(pack.y == (float)2.8469923E38F);
                Debug.Assert(pack.latitude == (int)1064504165);
                Debug.Assert(pack.z == (float)3.3109733E37F);
                Debug.Assert(pack.x == (float) -1.647038E38F);
                Debug.Assert(pack.approach_z == (float)1.8145078E38F);
                Debug.Assert(pack.approach_x == (float) -2.8101242E38F);
                Debug.Assert(pack.approach_y == (float)3.7844274E37F);
                Debug.Assert(pack.longitude == (int)1275009739);
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.0585864E38F, -9.458744E36F, 1.9526949E38F, 1.0904987E38F}));
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.latitude = (int)1064504165;
            p242.approach_y = (float)3.7844274E37F;
            p242.approach_z = (float)1.8145078E38F;
            p242.time_usec_SET((ulong)4546560617307335987L, PH) ;
            p242.z = (float)3.3109733E37F;
            p242.q_SET(new float[] {3.0585864E38F, -9.458744E36F, 1.9526949E38F, 1.0904987E38F}, 0) ;
            p242.y = (float)2.8469923E38F;
            p242.longitude = (int)1275009739;
            p242.approach_x = (float) -2.8101242E38F;
            p242.x = (float) -1.647038E38F;
            p242.altitude = (int)1822410779;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int)1362321922);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)5967952676338205627L);
                Debug.Assert(pack.altitude == (int) -179010470);
                Debug.Assert(pack.approach_y == (float)1.1142259E38F);
                Debug.Assert(pack.y == (float)1.1705867E38F);
                Debug.Assert(pack.z == (float) -2.0286732E38F);
                Debug.Assert(pack.x == (float) -2.5310616E38F);
                Debug.Assert(pack.longitude == (int)1490967451);
                Debug.Assert(pack.q.SequenceEqual(new float[] {6.739715E37F, 1.5706974E38F, -2.2951662E38F, -2.4686633E38F}));
                Debug.Assert(pack.approach_x == (float)3.95517E37F);
                Debug.Assert(pack.target_system == (byte)(byte)192);
                Debug.Assert(pack.approach_z == (float)1.187469E38F);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.approach_z = (float)1.187469E38F;
            p243.time_usec_SET((ulong)5967952676338205627L, PH) ;
            p243.latitude = (int)1362321922;
            p243.x = (float) -2.5310616E38F;
            p243.altitude = (int) -179010470;
            p243.y = (float)1.1705867E38F;
            p243.z = (float) -2.0286732E38F;
            p243.q_SET(new float[] {6.739715E37F, 1.5706974E38F, -2.2951662E38F, -2.4686633E38F}, 0) ;
            p243.approach_y = (float)1.1142259E38F;
            p243.target_system = (byte)(byte)192;
            p243.approach_x = (float)3.95517E37F;
            p243.longitude = (int)1490967451;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.interval_us == (int)2030579852);
                Debug.Assert(pack.message_id == (ushort)(ushort)15283);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)15283;
            p244.interval_us = (int)2030579852;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_FW;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int) -107552228);
                Debug.Assert(pack.ICAO_address == (uint)1861927878U);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)6986);
                Debug.Assert(pack.ver_velocity == (short)(short) -21747);
                Debug.Assert(pack.heading == (ushort)(ushort)48242);
                Debug.Assert(pack.lat == (int)677432012);
                Debug.Assert(pack.flags == (ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                                            ADSB_FLAGS.ADSB_FLAGS_SIMULATED));
                Debug.Assert(pack.emitter_type == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE);
                Debug.Assert(pack.tslc == (byte)(byte)128);
                Debug.Assert(pack.callsign_LEN(ph) == 9);
                Debug.Assert(pack.callsign_TRY(ph).Equals("dzZpfasbh"));
                Debug.Assert(pack.squawk == (ushort)(ushort)62360);
                Debug.Assert(pack.lon == (int) -452306476);
                Debug.Assert(pack.altitude_type == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.lat = (int)677432012;
            p246.tslc = (byte)(byte)128;
            p246.ICAO_address = (uint)1861927878U;
            p246.altitude = (int) -107552228;
            p246.lon = (int) -452306476;
            p246.heading = (ushort)(ushort)48242;
            p246.squawk = (ushort)(ushort)62360;
            p246.hor_velocity = (ushort)(ushort)6986;
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                          ADSB_FLAGS.ADSB_FLAGS_SIMULATED);
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.callsign_SET("dzZpfasbh", PH) ;
            p246.ver_velocity = (short)(short) -21747;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.horizontal_minimum_delta == (float)1.1183953E37F);
                Debug.Assert(pack.time_to_minimum_delta == (float) -1.7367738E38F);
                Debug.Assert(pack.altitude_minimum_delta == (float)2.6375562E38F);
                Debug.Assert(pack.action == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR);
                Debug.Assert(pack.id == (uint)1224045675U);
                Debug.Assert(pack.src_ == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
                Debug.Assert(pack.threat_level == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.horizontal_minimum_delta = (float)1.1183953E37F;
            p247.altitude_minimum_delta = (float)2.6375562E38F;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR;
            p247.id = (uint)1224045675U;
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.time_to_minimum_delta = (float) -1.7367738E38F;
            p247.threat_level = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)46);
                Debug.Assert(pack.target_component == (byte)(byte)254);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)69, (byte)1, (byte)108, (byte)126, (byte)240, (byte)24, (byte)225, (byte)253, (byte)244, (byte)241, (byte)98, (byte)82, (byte)121, (byte)146, (byte)3, (byte)60, (byte)180, (byte)14, (byte)100, (byte)85, (byte)223, (byte)250, (byte)18, (byte)135, (byte)138, (byte)22, (byte)52, (byte)246, (byte)58, (byte)204, (byte)167, (byte)41, (byte)63, (byte)145, (byte)117, (byte)198, (byte)10, (byte)112, (byte)99, (byte)114, (byte)39, (byte)204, (byte)161, (byte)46, (byte)243, (byte)96, (byte)42, (byte)151, (byte)114, (byte)84, (byte)10, (byte)191, (byte)64, (byte)197, (byte)38, (byte)203, (byte)234, (byte)218, (byte)245, (byte)188, (byte)227, (byte)209, (byte)189, (byte)100, (byte)193, (byte)26, (byte)108, (byte)128, (byte)172, (byte)143, (byte)189, (byte)175, (byte)70, (byte)151, (byte)204, (byte)92, (byte)137, (byte)7, (byte)32, (byte)70, (byte)31, (byte)34, (byte)69, (byte)253, (byte)109, (byte)182, (byte)125, (byte)55, (byte)195, (byte)187, (byte)78, (byte)83, (byte)7, (byte)81, (byte)234, (byte)98, (byte)215, (byte)211, (byte)63, (byte)229, (byte)82, (byte)202, (byte)244, (byte)53, (byte)79, (byte)83, (byte)40, (byte)233, (byte)163, (byte)56, (byte)126, (byte)210, (byte)120, (byte)229, (byte)123, (byte)223, (byte)105, (byte)194, (byte)66, (byte)168, (byte)150, (byte)48, (byte)33, (byte)109, (byte)162, (byte)110, (byte)185, (byte)125, (byte)71, (byte)246, (byte)227, (byte)200, (byte)98, (byte)132, (byte)254, (byte)168, (byte)19, (byte)229, (byte)68, (byte)174, (byte)208, (byte)142, (byte)160, (byte)205, (byte)91, (byte)82, (byte)229, (byte)131, (byte)219, (byte)197, (byte)69, (byte)148, (byte)184, (byte)1, (byte)11, (byte)228, (byte)196, (byte)25, (byte)118, (byte)178, (byte)160, (byte)129, (byte)73, (byte)12, (byte)228, (byte)199, (byte)106, (byte)28, (byte)46, (byte)246, (byte)63, (byte)250, (byte)251, (byte)54, (byte)227, (byte)167, (byte)210, (byte)45, (byte)83, (byte)81, (byte)212, (byte)8, (byte)254, (byte)194, (byte)170, (byte)218, (byte)136, (byte)82, (byte)159, (byte)171, (byte)114, (byte)177, (byte)255, (byte)26, (byte)92, (byte)100, (byte)51, (byte)133, (byte)16, (byte)252, (byte)166, (byte)200, (byte)124, (byte)65, (byte)94, (byte)54, (byte)12, (byte)43, (byte)105, (byte)158, (byte)13, (byte)103, (byte)29, (byte)121, (byte)186, (byte)20, (byte)156, (byte)251, (byte)176, (byte)109, (byte)229, (byte)255, (byte)246, (byte)215, (byte)163, (byte)229, (byte)184, (byte)201, (byte)50, (byte)38, (byte)151, (byte)131, (byte)226, (byte)121, (byte)59, (byte)75, (byte)196, (byte)239, (byte)24, (byte)141, (byte)38, (byte)132, (byte)95, (byte)8, (byte)184, (byte)9, (byte)154, (byte)111, (byte)108}));
                Debug.Assert(pack.message_type == (ushort)(ushort)3465);
                Debug.Assert(pack.target_network == (byte)(byte)136);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_system = (byte)(byte)46;
            p248.target_component = (byte)(byte)254;
            p248.payload_SET(new byte[] {(byte)69, (byte)1, (byte)108, (byte)126, (byte)240, (byte)24, (byte)225, (byte)253, (byte)244, (byte)241, (byte)98, (byte)82, (byte)121, (byte)146, (byte)3, (byte)60, (byte)180, (byte)14, (byte)100, (byte)85, (byte)223, (byte)250, (byte)18, (byte)135, (byte)138, (byte)22, (byte)52, (byte)246, (byte)58, (byte)204, (byte)167, (byte)41, (byte)63, (byte)145, (byte)117, (byte)198, (byte)10, (byte)112, (byte)99, (byte)114, (byte)39, (byte)204, (byte)161, (byte)46, (byte)243, (byte)96, (byte)42, (byte)151, (byte)114, (byte)84, (byte)10, (byte)191, (byte)64, (byte)197, (byte)38, (byte)203, (byte)234, (byte)218, (byte)245, (byte)188, (byte)227, (byte)209, (byte)189, (byte)100, (byte)193, (byte)26, (byte)108, (byte)128, (byte)172, (byte)143, (byte)189, (byte)175, (byte)70, (byte)151, (byte)204, (byte)92, (byte)137, (byte)7, (byte)32, (byte)70, (byte)31, (byte)34, (byte)69, (byte)253, (byte)109, (byte)182, (byte)125, (byte)55, (byte)195, (byte)187, (byte)78, (byte)83, (byte)7, (byte)81, (byte)234, (byte)98, (byte)215, (byte)211, (byte)63, (byte)229, (byte)82, (byte)202, (byte)244, (byte)53, (byte)79, (byte)83, (byte)40, (byte)233, (byte)163, (byte)56, (byte)126, (byte)210, (byte)120, (byte)229, (byte)123, (byte)223, (byte)105, (byte)194, (byte)66, (byte)168, (byte)150, (byte)48, (byte)33, (byte)109, (byte)162, (byte)110, (byte)185, (byte)125, (byte)71, (byte)246, (byte)227, (byte)200, (byte)98, (byte)132, (byte)254, (byte)168, (byte)19, (byte)229, (byte)68, (byte)174, (byte)208, (byte)142, (byte)160, (byte)205, (byte)91, (byte)82, (byte)229, (byte)131, (byte)219, (byte)197, (byte)69, (byte)148, (byte)184, (byte)1, (byte)11, (byte)228, (byte)196, (byte)25, (byte)118, (byte)178, (byte)160, (byte)129, (byte)73, (byte)12, (byte)228, (byte)199, (byte)106, (byte)28, (byte)46, (byte)246, (byte)63, (byte)250, (byte)251, (byte)54, (byte)227, (byte)167, (byte)210, (byte)45, (byte)83, (byte)81, (byte)212, (byte)8, (byte)254, (byte)194, (byte)170, (byte)218, (byte)136, (byte)82, (byte)159, (byte)171, (byte)114, (byte)177, (byte)255, (byte)26, (byte)92, (byte)100, (byte)51, (byte)133, (byte)16, (byte)252, (byte)166, (byte)200, (byte)124, (byte)65, (byte)94, (byte)54, (byte)12, (byte)43, (byte)105, (byte)158, (byte)13, (byte)103, (byte)29, (byte)121, (byte)186, (byte)20, (byte)156, (byte)251, (byte)176, (byte)109, (byte)229, (byte)255, (byte)246, (byte)215, (byte)163, (byte)229, (byte)184, (byte)201, (byte)50, (byte)38, (byte)151, (byte)131, (byte)226, (byte)121, (byte)59, (byte)75, (byte)196, (byte)239, (byte)24, (byte)141, (byte)38, (byte)132, (byte)95, (byte)8, (byte)184, (byte)9, (byte)154, (byte)111, (byte)108}, 0) ;
            p248.message_type = (ushort)(ushort)3465;
            p248.target_network = (byte)(byte)136;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ver == (byte)(byte)127);
                Debug.Assert(pack.type == (byte)(byte)203);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte)112, (sbyte)52, (sbyte) - 73, (sbyte)97, (sbyte)121, (sbyte) - 52, (sbyte) - 92, (sbyte)58, (sbyte)42, (sbyte) - 93, (sbyte)99, (sbyte) - 61, (sbyte)55, (sbyte) - 111, (sbyte)67, (sbyte) - 70, (sbyte) - 71, (sbyte)86, (sbyte)20, (sbyte) - 51, (sbyte) - 9, (sbyte) - 11, (sbyte)86, (sbyte) - 17, (sbyte)35, (sbyte)94, (sbyte) - 96, (sbyte)122, (sbyte) - 75, (sbyte) - 67, (sbyte) - 115, (sbyte) - 106}));
                Debug.Assert(pack.address == (ushort)(ushort)1050);
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.ver = (byte)(byte)127;
            p249.value_SET(new sbyte[] {(sbyte)112, (sbyte)52, (sbyte) - 73, (sbyte)97, (sbyte)121, (sbyte) - 52, (sbyte) - 92, (sbyte)58, (sbyte)42, (sbyte) - 93, (sbyte)99, (sbyte) - 61, (sbyte)55, (sbyte) - 111, (sbyte)67, (sbyte) - 70, (sbyte) - 71, (sbyte)86, (sbyte)20, (sbyte) - 51, (sbyte) - 9, (sbyte) - 11, (sbyte)86, (sbyte) - 17, (sbyte)35, (sbyte)94, (sbyte) - 96, (sbyte)122, (sbyte) - 75, (sbyte) - 67, (sbyte) - 115, (sbyte) - 106}, 0) ;
            p249.address = (ushort)(ushort)1050;
            p249.type = (byte)(byte)203;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3288645799809418677L);
                Debug.Assert(pack.z == (float)2.3435935E38F);
                Debug.Assert(pack.y == (float) -2.5381359E38F);
                Debug.Assert(pack.x == (float) -4.817951E37F);
                Debug.Assert(pack.name_LEN(ph) == 5);
                Debug.Assert(pack.name_TRY(ph).Equals("kweab"));
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.time_usec = (ulong)3288645799809418677L;
            p250.name_SET("kweab", PH) ;
            p250.z = (float)2.3435935E38F;
            p250.x = (float) -4.817951E37F;
            p250.y = (float) -2.5381359E38F;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float) -1.4237075E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4139383616U);
                Debug.Assert(pack.name_LEN(ph) == 2);
                Debug.Assert(pack.name_TRY(ph).Equals("nm"));
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.value = (float) -1.4237075E38F;
            p251.name_SET("nm", PH) ;
            p251.time_boot_ms = (uint)4139383616U;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 3);
                Debug.Assert(pack.name_TRY(ph).Equals("frx"));
                Debug.Assert(pack.value == (int)9382471);
                Debug.Assert(pack.time_boot_ms == (uint)3879952053U);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.name_SET("frx", PH) ;
            p252.value = (int)9382471;
            p252.time_boot_ms = (uint)3879952053U;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.severity == MAV_SEVERITY.MAV_SEVERITY_NOTICE);
                Debug.Assert(pack.text_LEN(ph) == 24);
                Debug.Assert(pack.text_TRY(ph).Equals("obfgfccbfNzyhaipPmbvgtwl"));
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_NOTICE;
            p253.text_SET("obfgfccbfNzyhaipPmbvgtwl", PH) ;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)4007033053U);
                Debug.Assert(pack.value == (float) -3.4789563E37F);
                Debug.Assert(pack.ind == (byte)(byte)64);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.value = (float) -3.4789563E37F;
            p254.ind = (byte)(byte)64;
            p254.time_boot_ms = (uint)4007033053U;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.initial_timestamp == (ulong)3603562087637710506L);
                Debug.Assert(pack.target_system == (byte)(byte)134);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)20, (byte)44, (byte)39, (byte)32, (byte)67, (byte)30, (byte)48, (byte)172, (byte)195, (byte)105, (byte)215, (byte)147, (byte)14, (byte)46, (byte)43, (byte)183, (byte)26, (byte)64, (byte)204, (byte)211, (byte)173, (byte)79, (byte)18, (byte)215, (byte)186, (byte)25, (byte)17, (byte)37, (byte)169, (byte)102, (byte)158, (byte)255}));
                Debug.Assert(pack.target_component == (byte)(byte)124);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)134;
            p256.target_component = (byte)(byte)124;
            p256.initial_timestamp = (ulong)3603562087637710506L;
            p256.secret_key_SET(new byte[] {(byte)20, (byte)44, (byte)39, (byte)32, (byte)67, (byte)30, (byte)48, (byte)172, (byte)195, (byte)105, (byte)215, (byte)147, (byte)14, (byte)46, (byte)43, (byte)183, (byte)26, (byte)64, (byte)204, (byte)211, (byte)173, (byte)79, (byte)18, (byte)215, (byte)186, (byte)25, (byte)17, (byte)37, (byte)169, (byte)102, (byte)158, (byte)255}, 0) ;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_change_ms == (uint)2274081414U);
                Debug.Assert(pack.state == (byte)(byte)129);
                Debug.Assert(pack.time_boot_ms == (uint)4001438669U);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.last_change_ms = (uint)2274081414U;
            p257.time_boot_ms = (uint)4001438669U;
            p257.state = (byte)(byte)129;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tune_LEN(ph) == 19);
                Debug.Assert(pack.tune_TRY(ph).Equals("hxxeBylutacvcihFNam"));
                Debug.Assert(pack.target_system == (byte)(byte)249);
                Debug.Assert(pack.target_component == (byte)(byte)184);
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)249;
            p258.tune_SET("hxxeBylutacvcihFNam", PH) ;
            p258.target_component = (byte)(byte)184;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3973943306U);
                Debug.Assert(pack.sensor_size_h == (float)2.4875828E38F);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)30115);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)5565);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)45, (byte)225, (byte)238, (byte)54, (byte)202, (byte)172, (byte)89, (byte)190, (byte)76, (byte)220, (byte)9, (byte)249, (byte)16, (byte)83, (byte)79, (byte)165, (byte)127, (byte)181, (byte)6, (byte)70, (byte)141, (byte)102, (byte)138, (byte)19, (byte)167, (byte)242, (byte)254, (byte)192, (byte)164, (byte)73, (byte)22, (byte)179}));
                Debug.Assert(pack.firmware_version == (uint)1899374806U);
                Debug.Assert(pack.lens_id == (byte)(byte)189);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 36);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("jsRmqgbJmCzzljcvccRcajdtvdtacdtinbmr"));
                Debug.Assert(pack.sensor_size_v == (float)9.707102E37F);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO));
                Debug.Assert(pack.focal_length == (float) -1.6309388E38F);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)98, (byte)210, (byte)97, (byte)108, (byte)149, (byte)247, (byte)156, (byte)118, (byte)2, (byte)3, (byte)31, (byte)34, (byte)24, (byte)11, (byte)43, (byte)175, (byte)131, (byte)47, (byte)180, (byte)159, (byte)68, (byte)246, (byte)72, (byte)194, (byte)144, (byte)154, (byte)37, (byte)90, (byte)166, (byte)29, (byte)11, (byte)35}));
                Debug.Assert(pack.resolution_v == (ushort)(ushort)28596);
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.vendor_name_SET(new byte[] {(byte)45, (byte)225, (byte)238, (byte)54, (byte)202, (byte)172, (byte)89, (byte)190, (byte)76, (byte)220, (byte)9, (byte)249, (byte)16, (byte)83, (byte)79, (byte)165, (byte)127, (byte)181, (byte)6, (byte)70, (byte)141, (byte)102, (byte)138, (byte)19, (byte)167, (byte)242, (byte)254, (byte)192, (byte)164, (byte)73, (byte)22, (byte)179}, 0) ;
            p259.firmware_version = (uint)1899374806U;
            p259.model_name_SET(new byte[] {(byte)98, (byte)210, (byte)97, (byte)108, (byte)149, (byte)247, (byte)156, (byte)118, (byte)2, (byte)3, (byte)31, (byte)34, (byte)24, (byte)11, (byte)43, (byte)175, (byte)131, (byte)47, (byte)180, (byte)159, (byte)68, (byte)246, (byte)72, (byte)194, (byte)144, (byte)154, (byte)37, (byte)90, (byte)166, (byte)29, (byte)11, (byte)35}, 0) ;
            p259.time_boot_ms = (uint)3973943306U;
            p259.cam_definition_uri_SET("jsRmqgbJmCzzljcvccRcajdtvdtacdtinbmr", PH) ;
            p259.sensor_size_h = (float)2.4875828E38F;
            p259.cam_definition_version = (ushort)(ushort)30115;
            p259.lens_id = (byte)(byte)189;
            p259.resolution_h = (ushort)(ushort)5565;
            p259.resolution_v = (ushort)(ushort)28596;
            p259.focal_length = (float) -1.6309388E38F;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
            p259.sensor_size_v = (float)9.707102E37F;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1318901460U);
                Debug.Assert(pack.mode_id == CAMERA_MODE.CAMERA_MODE_IMAGE);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = CAMERA_MODE.CAMERA_MODE_IMAGE;
            p260.time_boot_ms = (uint)1318901460U;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.storage_id == (byte)(byte)122);
                Debug.Assert(pack.used_capacity == (float)2.9826099E38F);
                Debug.Assert(pack.read_speed == (float)1.6132176E38F);
                Debug.Assert(pack.available_capacity == (float) -1.5444297E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3421277732U);
                Debug.Assert(pack.total_capacity == (float)2.560642E38F);
                Debug.Assert(pack.status == (byte)(byte)14);
                Debug.Assert(pack.write_speed == (float) -2.9628937E38F);
                Debug.Assert(pack.storage_count == (byte)(byte)173);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.storage_id = (byte)(byte)122;
            p261.write_speed = (float) -2.9628937E38F;
            p261.storage_count = (byte)(byte)173;
            p261.time_boot_ms = (uint)3421277732U;
            p261.read_speed = (float)1.6132176E38F;
            p261.available_capacity = (float) -1.5444297E38F;
            p261.status = (byte)(byte)14;
            p261.used_capacity = (float)2.9826099E38F;
            p261.total_capacity = (float)2.560642E38F;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.video_status == (byte)(byte)123);
                Debug.Assert(pack.available_capacity == (float)1.1307973E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2262461032U);
                Debug.Assert(pack.recording_time_ms == (uint)3923272316U);
                Debug.Assert(pack.image_interval == (float) -1.5036807E38F);
                Debug.Assert(pack.image_status == (byte)(byte)104);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.image_interval = (float) -1.5036807E38F;
            p262.available_capacity = (float)1.1307973E38F;
            p262.image_status = (byte)(byte)104;
            p262.video_status = (byte)(byte)123;
            p262.time_boot_ms = (uint)2262461032U;
            p262.recording_time_ms = (uint)3923272316U;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (int)44138912);
                Debug.Assert(pack.time_boot_ms == (uint)1528472491U);
                Debug.Assert(pack.relative_alt == (int) -1077885269);
                Debug.Assert(pack.camera_id == (byte)(byte)17);
                Debug.Assert(pack.lat == (int) -2081211820);
                Debug.Assert(pack.time_utc == (ulong)6408639064341845417L);
                Debug.Assert(pack.file_url_LEN(ph) == 142);
                Debug.Assert(pack.file_url_TRY(ph).Equals("ajhlfpsbavuuirsozhzKfdurajhtbnkylnfwhbgvuiwvwjonnbnesrzjezktqusSwvatrtsexMOusuYvfhvCduegkrzpbtsnEthpxPumauhfgemmgtnojOOzbbcobqqigMhxjdhjvcipaw"));
                Debug.Assert(pack.lon == (int) -248100235);
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.1473126E38F, -2.7482602E38F, -2.6757916E38F, 5.0161624E37F}));
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)112);
                Debug.Assert(pack.image_index == (int)246379019);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.q_SET(new float[] {3.1473126E38F, -2.7482602E38F, -2.6757916E38F, 5.0161624E37F}, 0) ;
            p263.time_boot_ms = (uint)1528472491U;
            p263.lon = (int) -248100235;
            p263.file_url_SET("ajhlfpsbavuuirsozhzKfdurajhtbnkylnfwhbgvuiwvwjonnbnesrzjezktqusSwvatrtsexMOusuYvfhvCduegkrzpbtsnEthpxPumauhfgemmgtnojOOzbbcobqqigMhxjdhjvcipaw", PH) ;
            p263.image_index = (int)246379019;
            p263.camera_id = (byte)(byte)17;
            p263.relative_alt = (int) -1077885269;
            p263.time_utc = (ulong)6408639064341845417L;
            p263.alt = (int)44138912;
            p263.capture_result = (sbyte)(sbyte)112;
            p263.lat = (int) -2081211820;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)618089417U);
                Debug.Assert(pack.arming_time_utc == (ulong)1758881033868944637L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)5875099325331209123L);
                Debug.Assert(pack.flight_uuid == (ulong)203930987476903343L);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.arming_time_utc = (ulong)1758881033868944637L;
            p264.flight_uuid = (ulong)203930987476903343L;
            p264.takeoff_time_utc = (ulong)5875099325331209123L;
            p264.time_boot_ms = (uint)618089417U;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3527931748U);
                Debug.Assert(pack.roll == (float)1.1232302E38F);
                Debug.Assert(pack.yaw == (float) -1.9262993E38F);
                Debug.Assert(pack.pitch == (float) -2.5231731E38F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.roll = (float)1.1232302E38F;
            p265.pitch = (float) -2.5231731E38F;
            p265.time_boot_ms = (uint)3527931748U;
            p265.yaw = (float) -1.9262993E38F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)18);
                Debug.Assert(pack.length == (byte)(byte)214);
                Debug.Assert(pack.sequence == (ushort)(ushort)38045);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)116, (byte)194, (byte)188, (byte)209, (byte)141, (byte)220, (byte)252, (byte)107, (byte)33, (byte)125, (byte)24, (byte)45, (byte)33, (byte)2, (byte)69, (byte)85, (byte)65, (byte)219, (byte)249, (byte)158, (byte)57, (byte)188, (byte)183, (byte)177, (byte)117, (byte)125, (byte)179, (byte)246, (byte)139, (byte)226, (byte)237, (byte)155, (byte)43, (byte)218, (byte)54, (byte)234, (byte)30, (byte)238, (byte)157, (byte)7, (byte)239, (byte)199, (byte)241, (byte)41, (byte)117, (byte)157, (byte)146, (byte)20, (byte)208, (byte)76, (byte)145, (byte)118, (byte)119, (byte)175, (byte)189, (byte)232, (byte)128, (byte)109, (byte)242, (byte)118, (byte)172, (byte)237, (byte)185, (byte)1, (byte)123, (byte)235, (byte)230, (byte)134, (byte)169, (byte)2, (byte)157, (byte)202, (byte)246, (byte)220, (byte)150, (byte)189, (byte)24, (byte)100, (byte)98, (byte)192, (byte)10, (byte)93, (byte)100, (byte)108, (byte)138, (byte)124, (byte)125, (byte)31, (byte)78, (byte)2, (byte)157, (byte)59, (byte)117, (byte)147, (byte)31, (byte)51, (byte)82, (byte)162, (byte)118, (byte)93, (byte)248, (byte)19, (byte)244, (byte)81, (byte)3, (byte)205, (byte)96, (byte)162, (byte)241, (byte)249, (byte)93, (byte)183, (byte)239, (byte)243, (byte)95, (byte)49, (byte)133, (byte)140, (byte)46, (byte)10, (byte)91, (byte)75, (byte)30, (byte)230, (byte)226, (byte)246, (byte)85, (byte)93, (byte)18, (byte)168, (byte)225, (byte)218, (byte)239, (byte)86, (byte)21, (byte)1, (byte)209, (byte)77, (byte)223, (byte)200, (byte)54, (byte)210, (byte)201, (byte)136, (byte)11, (byte)51, (byte)146, (byte)150, (byte)30, (byte)19, (byte)232, (byte)43, (byte)155, (byte)191, (byte)88, (byte)101, (byte)73, (byte)169, (byte)193, (byte)144, (byte)100, (byte)134, (byte)144, (byte)32, (byte)194, (byte)84, (byte)88, (byte)217, (byte)242, (byte)43, (byte)30, (byte)142, (byte)199, (byte)184, (byte)131, (byte)240, (byte)209, (byte)205, (byte)169, (byte)62, (byte)20, (byte)170, (byte)16, (byte)170, (byte)20, (byte)126, (byte)130, (byte)251, (byte)18, (byte)163, (byte)21, (byte)248, (byte)117, (byte)5, (byte)221, (byte)78, (byte)173, (byte)114, (byte)199, (byte)36, (byte)99, (byte)237, (byte)171, (byte)145, (byte)172, (byte)254, (byte)248, (byte)138, (byte)126, (byte)195, (byte)4, (byte)69, (byte)108, (byte)54, (byte)202, (byte)18, (byte)159, (byte)101, (byte)192, (byte)104, (byte)243, (byte)221, (byte)122, (byte)152, (byte)55, (byte)89, (byte)138, (byte)10, (byte)114, (byte)167, (byte)116, (byte)186, (byte)97, (byte)122, (byte)206, (byte)15, (byte)229, (byte)98, (byte)154, (byte)255, (byte)245, (byte)85, (byte)137, (byte)241, (byte)59, (byte)137, (byte)161, (byte)84, (byte)185}));
                Debug.Assert(pack.first_message_offset == (byte)(byte)221);
                Debug.Assert(pack.target_system == (byte)(byte)109);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.sequence = (ushort)(ushort)38045;
            p266.target_component = (byte)(byte)18;
            p266.first_message_offset = (byte)(byte)221;
            p266.target_system = (byte)(byte)109;
            p266.length = (byte)(byte)214;
            p266.data__SET(new byte[] {(byte)116, (byte)194, (byte)188, (byte)209, (byte)141, (byte)220, (byte)252, (byte)107, (byte)33, (byte)125, (byte)24, (byte)45, (byte)33, (byte)2, (byte)69, (byte)85, (byte)65, (byte)219, (byte)249, (byte)158, (byte)57, (byte)188, (byte)183, (byte)177, (byte)117, (byte)125, (byte)179, (byte)246, (byte)139, (byte)226, (byte)237, (byte)155, (byte)43, (byte)218, (byte)54, (byte)234, (byte)30, (byte)238, (byte)157, (byte)7, (byte)239, (byte)199, (byte)241, (byte)41, (byte)117, (byte)157, (byte)146, (byte)20, (byte)208, (byte)76, (byte)145, (byte)118, (byte)119, (byte)175, (byte)189, (byte)232, (byte)128, (byte)109, (byte)242, (byte)118, (byte)172, (byte)237, (byte)185, (byte)1, (byte)123, (byte)235, (byte)230, (byte)134, (byte)169, (byte)2, (byte)157, (byte)202, (byte)246, (byte)220, (byte)150, (byte)189, (byte)24, (byte)100, (byte)98, (byte)192, (byte)10, (byte)93, (byte)100, (byte)108, (byte)138, (byte)124, (byte)125, (byte)31, (byte)78, (byte)2, (byte)157, (byte)59, (byte)117, (byte)147, (byte)31, (byte)51, (byte)82, (byte)162, (byte)118, (byte)93, (byte)248, (byte)19, (byte)244, (byte)81, (byte)3, (byte)205, (byte)96, (byte)162, (byte)241, (byte)249, (byte)93, (byte)183, (byte)239, (byte)243, (byte)95, (byte)49, (byte)133, (byte)140, (byte)46, (byte)10, (byte)91, (byte)75, (byte)30, (byte)230, (byte)226, (byte)246, (byte)85, (byte)93, (byte)18, (byte)168, (byte)225, (byte)218, (byte)239, (byte)86, (byte)21, (byte)1, (byte)209, (byte)77, (byte)223, (byte)200, (byte)54, (byte)210, (byte)201, (byte)136, (byte)11, (byte)51, (byte)146, (byte)150, (byte)30, (byte)19, (byte)232, (byte)43, (byte)155, (byte)191, (byte)88, (byte)101, (byte)73, (byte)169, (byte)193, (byte)144, (byte)100, (byte)134, (byte)144, (byte)32, (byte)194, (byte)84, (byte)88, (byte)217, (byte)242, (byte)43, (byte)30, (byte)142, (byte)199, (byte)184, (byte)131, (byte)240, (byte)209, (byte)205, (byte)169, (byte)62, (byte)20, (byte)170, (byte)16, (byte)170, (byte)20, (byte)126, (byte)130, (byte)251, (byte)18, (byte)163, (byte)21, (byte)248, (byte)117, (byte)5, (byte)221, (byte)78, (byte)173, (byte)114, (byte)199, (byte)36, (byte)99, (byte)237, (byte)171, (byte)145, (byte)172, (byte)254, (byte)248, (byte)138, (byte)126, (byte)195, (byte)4, (byte)69, (byte)108, (byte)54, (byte)202, (byte)18, (byte)159, (byte)101, (byte)192, (byte)104, (byte)243, (byte)221, (byte)122, (byte)152, (byte)55, (byte)89, (byte)138, (byte)10, (byte)114, (byte)167, (byte)116, (byte)186, (byte)97, (byte)122, (byte)206, (byte)15, (byte)229, (byte)98, (byte)154, (byte)255, (byte)245, (byte)85, (byte)137, (byte)241, (byte)59, (byte)137, (byte)161, (byte)84, (byte)185}, 0) ;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sequence == (ushort)(ushort)25811);
                Debug.Assert(pack.target_component == (byte)(byte)242);
                Debug.Assert(pack.target_system == (byte)(byte)249);
                Debug.Assert(pack.length == (byte)(byte)50);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)92, (byte)238, (byte)254, (byte)151, (byte)192, (byte)234, (byte)170, (byte)238, (byte)160, (byte)221, (byte)248, (byte)113, (byte)216, (byte)82, (byte)244, (byte)67, (byte)255, (byte)138, (byte)2, (byte)221, (byte)245, (byte)47, (byte)159, (byte)236, (byte)80, (byte)194, (byte)206, (byte)33, (byte)177, (byte)170, (byte)30, (byte)179, (byte)39, (byte)60, (byte)205, (byte)167, (byte)50, (byte)157, (byte)115, (byte)86, (byte)243, (byte)55, (byte)113, (byte)59, (byte)213, (byte)216, (byte)117, (byte)115, (byte)236, (byte)140, (byte)73, (byte)160, (byte)130, (byte)29, (byte)246, (byte)209, (byte)99, (byte)197, (byte)36, (byte)95, (byte)20, (byte)90, (byte)82, (byte)234, (byte)130, (byte)69, (byte)45, (byte)99, (byte)216, (byte)217, (byte)5, (byte)188, (byte)18, (byte)179, (byte)143, (byte)68, (byte)111, (byte)222, (byte)220, (byte)250, (byte)247, (byte)86, (byte)31, (byte)13, (byte)64, (byte)12, (byte)201, (byte)232, (byte)55, (byte)41, (byte)65, (byte)13, (byte)36, (byte)177, (byte)22, (byte)8, (byte)144, (byte)122, (byte)30, (byte)233, (byte)130, (byte)74, (byte)2, (byte)130, (byte)123, (byte)168, (byte)68, (byte)151, (byte)241, (byte)196, (byte)245, (byte)43, (byte)192, (byte)43, (byte)142, (byte)120, (byte)198, (byte)10, (byte)197, (byte)189, (byte)231, (byte)232, (byte)246, (byte)56, (byte)219, (byte)60, (byte)242, (byte)22, (byte)88, (byte)42, (byte)126, (byte)2, (byte)243, (byte)33, (byte)126, (byte)143, (byte)205, (byte)114, (byte)96, (byte)125, (byte)224, (byte)131, (byte)68, (byte)180, (byte)193, (byte)20, (byte)139, (byte)187, (byte)157, (byte)113, (byte)145, (byte)63, (byte)52, (byte)82, (byte)199, (byte)160, (byte)161, (byte)254, (byte)70, (byte)96, (byte)133, (byte)56, (byte)120, (byte)110, (byte)99, (byte)71, (byte)201, (byte)161, (byte)100, (byte)146, (byte)237, (byte)140, (byte)32, (byte)217, (byte)11, (byte)218, (byte)225, (byte)173, (byte)113, (byte)54, (byte)5, (byte)146, (byte)223, (byte)83, (byte)128, (byte)73, (byte)143, (byte)213, (byte)148, (byte)36, (byte)22, (byte)51, (byte)20, (byte)141, (byte)211, (byte)253, (byte)170, (byte)130, (byte)247, (byte)35, (byte)228, (byte)132, (byte)215, (byte)109, (byte)246, (byte)56, (byte)193, (byte)101, (byte)232, (byte)233, (byte)210, (byte)148, (byte)201, (byte)148, (byte)104, (byte)222, (byte)211, (byte)136, (byte)96, (byte)160, (byte)39, (byte)174, (byte)223, (byte)55, (byte)179, (byte)183, (byte)146, (byte)77, (byte)56, (byte)67, (byte)37, (byte)137, (byte)248, (byte)87, (byte)128, (byte)75, (byte)206, (byte)234, (byte)241, (byte)21, (byte)11, (byte)227, (byte)215, (byte)30, (byte)204, (byte)76, (byte)53, (byte)26, (byte)27}));
                Debug.Assert(pack.first_message_offset == (byte)(byte)122);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.first_message_offset = (byte)(byte)122;
            p267.data__SET(new byte[] {(byte)92, (byte)238, (byte)254, (byte)151, (byte)192, (byte)234, (byte)170, (byte)238, (byte)160, (byte)221, (byte)248, (byte)113, (byte)216, (byte)82, (byte)244, (byte)67, (byte)255, (byte)138, (byte)2, (byte)221, (byte)245, (byte)47, (byte)159, (byte)236, (byte)80, (byte)194, (byte)206, (byte)33, (byte)177, (byte)170, (byte)30, (byte)179, (byte)39, (byte)60, (byte)205, (byte)167, (byte)50, (byte)157, (byte)115, (byte)86, (byte)243, (byte)55, (byte)113, (byte)59, (byte)213, (byte)216, (byte)117, (byte)115, (byte)236, (byte)140, (byte)73, (byte)160, (byte)130, (byte)29, (byte)246, (byte)209, (byte)99, (byte)197, (byte)36, (byte)95, (byte)20, (byte)90, (byte)82, (byte)234, (byte)130, (byte)69, (byte)45, (byte)99, (byte)216, (byte)217, (byte)5, (byte)188, (byte)18, (byte)179, (byte)143, (byte)68, (byte)111, (byte)222, (byte)220, (byte)250, (byte)247, (byte)86, (byte)31, (byte)13, (byte)64, (byte)12, (byte)201, (byte)232, (byte)55, (byte)41, (byte)65, (byte)13, (byte)36, (byte)177, (byte)22, (byte)8, (byte)144, (byte)122, (byte)30, (byte)233, (byte)130, (byte)74, (byte)2, (byte)130, (byte)123, (byte)168, (byte)68, (byte)151, (byte)241, (byte)196, (byte)245, (byte)43, (byte)192, (byte)43, (byte)142, (byte)120, (byte)198, (byte)10, (byte)197, (byte)189, (byte)231, (byte)232, (byte)246, (byte)56, (byte)219, (byte)60, (byte)242, (byte)22, (byte)88, (byte)42, (byte)126, (byte)2, (byte)243, (byte)33, (byte)126, (byte)143, (byte)205, (byte)114, (byte)96, (byte)125, (byte)224, (byte)131, (byte)68, (byte)180, (byte)193, (byte)20, (byte)139, (byte)187, (byte)157, (byte)113, (byte)145, (byte)63, (byte)52, (byte)82, (byte)199, (byte)160, (byte)161, (byte)254, (byte)70, (byte)96, (byte)133, (byte)56, (byte)120, (byte)110, (byte)99, (byte)71, (byte)201, (byte)161, (byte)100, (byte)146, (byte)237, (byte)140, (byte)32, (byte)217, (byte)11, (byte)218, (byte)225, (byte)173, (byte)113, (byte)54, (byte)5, (byte)146, (byte)223, (byte)83, (byte)128, (byte)73, (byte)143, (byte)213, (byte)148, (byte)36, (byte)22, (byte)51, (byte)20, (byte)141, (byte)211, (byte)253, (byte)170, (byte)130, (byte)247, (byte)35, (byte)228, (byte)132, (byte)215, (byte)109, (byte)246, (byte)56, (byte)193, (byte)101, (byte)232, (byte)233, (byte)210, (byte)148, (byte)201, (byte)148, (byte)104, (byte)222, (byte)211, (byte)136, (byte)96, (byte)160, (byte)39, (byte)174, (byte)223, (byte)55, (byte)179, (byte)183, (byte)146, (byte)77, (byte)56, (byte)67, (byte)37, (byte)137, (byte)248, (byte)87, (byte)128, (byte)75, (byte)206, (byte)234, (byte)241, (byte)21, (byte)11, (byte)227, (byte)215, (byte)30, (byte)204, (byte)76, (byte)53, (byte)26, (byte)27}, 0) ;
            p267.target_component = (byte)(byte)242;
            p267.target_system = (byte)(byte)249;
            p267.length = (byte)(byte)50;
            p267.sequence = (ushort)(ushort)25811;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)70);
                Debug.Assert(pack.sequence == (ushort)(ushort)62466);
                Debug.Assert(pack.target_component == (byte)(byte)13);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_component = (byte)(byte)13;
            p268.sequence = (ushort)(ushort)62466;
            p268.target_system = (byte)(byte)70;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_v == (ushort)(ushort)10401);
                Debug.Assert(pack.status == (byte)(byte)57);
                Debug.Assert(pack.uri_LEN(ph) == 103);
                Debug.Assert(pack.uri_TRY(ph).Equals("mvmccailujitIjlcpLafoHAuLwiOmulykwchvItIsonsatvbinpkubqrhopymuQipjqfBnfhyculglVugcgMrkeolamospouTlealka"));
                Debug.Assert(pack.camera_id == (byte)(byte)20);
                Debug.Assert(pack.framerate == (float)1.9469922E36F);
                Debug.Assert(pack.rotation == (ushort)(ushort)43085);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)19559);
                Debug.Assert(pack.bitrate == (uint)1523727660U);
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.status = (byte)(byte)57;
            p269.framerate = (float)1.9469922E36F;
            p269.uri_SET("mvmccailujitIjlcpLafoHAuLwiOmulykwchvItIsonsatvbinpkubqrhopymuQipjqfBnfhyculglVugcgMrkeolamospouTlealka", PH) ;
            p269.rotation = (ushort)(ushort)43085;
            p269.resolution_h = (ushort)(ushort)19559;
            p269.camera_id = (byte)(byte)20;
            p269.bitrate = (uint)1523727660U;
            p269.resolution_v = (ushort)(ushort)10401;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_h == (ushort)(ushort)50711);
                Debug.Assert(pack.framerate == (float) -3.8676438E37F);
                Debug.Assert(pack.camera_id == (byte)(byte)126);
                Debug.Assert(pack.uri_LEN(ph) == 116);
                Debug.Assert(pack.uri_TRY(ph).Equals("rxuoderjlvcXrnzxdhvnvhYugarqwwjfsgwgGrckpqoWnpkIiCosrioczdavnipygodIwdtkdophsdymjlduwehuNndyyjooEpqrgiuDjywqwcsmwbxr"));
                Debug.Assert(pack.resolution_v == (ushort)(ushort)39398);
                Debug.Assert(pack.bitrate == (uint)778519728U);
                Debug.Assert(pack.rotation == (ushort)(ushort)38677);
                Debug.Assert(pack.target_system == (byte)(byte)123);
                Debug.Assert(pack.target_component == (byte)(byte)167);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.framerate = (float) -3.8676438E37F;
            p270.rotation = (ushort)(ushort)38677;
            p270.resolution_h = (ushort)(ushort)50711;
            p270.camera_id = (byte)(byte)126;
            p270.uri_SET("rxuoderjlvcXrnzxdhvnvhYugarqwwjfsgwgGrckpqoWnpkIiCosrioczdavnipygodIwdtkdophsdymjlduwehuNndyyjooEpqrgiuDjywqwcsmwbxr", PH) ;
            p270.target_component = (byte)(byte)167;
            p270.bitrate = (uint)778519728U;
            p270.target_system = (byte)(byte)123;
            p270.resolution_v = (ushort)(ushort)39398;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 27);
                Debug.Assert(pack.password_TRY(ph).Equals("osgagfqkxhiAgvwguStusqohjrU"));
                Debug.Assert(pack.ssid_LEN(ph) == 4);
                Debug.Assert(pack.ssid_TRY(ph).Equals("rezi"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.password_SET("osgagfqkxhiAgvwguStusqohjrU", PH) ;
            p299.ssid_SET("rezi", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)187, (byte)112, (byte)13, (byte)167, (byte)182, (byte)101, (byte)1, (byte)92}));
                Debug.Assert(pack.max_version == (ushort)(ushort)55201);
                Debug.Assert(pack.min_version == (ushort)(ushort)21653);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)124, (byte)226, (byte)74, (byte)241, (byte)24, (byte)148, (byte)176, (byte)74}));
                Debug.Assert(pack.version == (ushort)(ushort)5804);
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.spec_version_hash_SET(new byte[] {(byte)124, (byte)226, (byte)74, (byte)241, (byte)24, (byte)148, (byte)176, (byte)74}, 0) ;
            p300.version = (ushort)(ushort)5804;
            p300.max_version = (ushort)(ushort)55201;
            p300.min_version = (ushort)(ushort)21653;
            p300.library_version_hash_SET(new byte[] {(byte)187, (byte)112, (byte)13, (byte)167, (byte)182, (byte)101, (byte)1, (byte)92}, 0) ;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.health == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)46096);
                Debug.Assert(pack.sub_mode == (byte)(byte)181);
                Debug.Assert(pack.uptime_sec == (uint)150726296U);
                Debug.Assert(pack.mode == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
                Debug.Assert(pack.time_usec == (ulong)5457368553182539464L);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING;
            p310.uptime_sec = (uint)150726296U;
            p310.vendor_specific_status_code = (ushort)(ushort)46096;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE;
            p310.sub_mode = (byte)(byte)181;
            p310.time_usec = (ulong)5457368553182539464L;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sw_version_minor == (byte)(byte)195);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)194);
                Debug.Assert(pack.sw_version_major == (byte)(byte)49);
                Debug.Assert(pack.name_LEN(ph) == 60);
                Debug.Assert(pack.name_TRY(ph).Equals("ackEcfcpmyndmzfsyCgejyikRuxgjfvjlyCjpmXvTcmLwRmJuqnvfhhttfht"));
                Debug.Assert(pack.hw_version_major == (byte)(byte)166);
                Debug.Assert(pack.sw_vcs_commit == (uint)3925065654U);
                Debug.Assert(pack.uptime_sec == (uint)1047700028U);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)87, (byte)171, (byte)211, (byte)194, (byte)13, (byte)254, (byte)220, (byte)6, (byte)18, (byte)30, (byte)77, (byte)191, (byte)97, (byte)203, (byte)186, (byte)35}));
                Debug.Assert(pack.time_usec == (ulong)6541876561518364193L);
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.sw_version_major = (byte)(byte)49;
            p311.hw_unique_id_SET(new byte[] {(byte)87, (byte)171, (byte)211, (byte)194, (byte)13, (byte)254, (byte)220, (byte)6, (byte)18, (byte)30, (byte)77, (byte)191, (byte)97, (byte)203, (byte)186, (byte)35}, 0) ;
            p311.sw_vcs_commit = (uint)3925065654U;
            p311.name_SET("ackEcfcpmyndmzfsyCgejyikRuxgjfvjlyCjpmXvTcmLwRmJuqnvfhhttfht", PH) ;
            p311.sw_version_minor = (byte)(byte)195;
            p311.hw_version_minor = (byte)(byte)194;
            p311.time_usec = (ulong)6541876561518364193L;
            p311.uptime_sec = (uint)1047700028U;
            p311.hw_version_major = (byte)(byte)166;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)191);
                Debug.Assert(pack.target_component == (byte)(byte)18);
                Debug.Assert(pack.param_index == (short)(short) -5578);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("wUevk"));
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_id_SET("wUevk", PH) ;
            p320.target_component = (byte)(byte)18;
            p320.param_index = (short)(short) -5578;
            p320.target_system = (byte)(byte)191;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)7);
                Debug.Assert(pack.target_component == (byte)(byte)224);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)7;
            p321.target_component = (byte)(byte)224;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
                Debug.Assert(pack.param_id_LEN(ph) == 7);
                Debug.Assert(pack.param_id_TRY(ph).Equals("rotcyfa"));
                Debug.Assert(pack.param_index == (ushort)(ushort)60938);
                Debug.Assert(pack.param_value_LEN(ph) == 91);
                Debug.Assert(pack.param_value_TRY(ph).Equals("aaAkqcyynnzwKfwwqfwqygQsuvetrbdocwrnmrTygdsyGvgMoyyhcdvsbjnzzoMbjmhnCafcqubszmukfllvqtfvvli"));
                Debug.Assert(pack.param_count == (ushort)(ushort)52294);
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8;
            p322.param_count = (ushort)(ushort)52294;
            p322.param_index = (ushort)(ushort)60938;
            p322.param_value_SET("aaAkqcyynnzwKfwwqfwqygQsuvetrbdocwrnmrTygdsyGvgMoyyhcdvsbjnzzoMbjmhnCafcqubszmukfllvqtfvvli", PH) ;
            p322.param_id_SET("rotcyfa", PH) ;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)21);
                Debug.Assert(pack.param_id_LEN(ph) == 6);
                Debug.Assert(pack.param_id_TRY(ph).Equals("yxxtqb"));
                Debug.Assert(pack.param_value_LEN(ph) == 18);
                Debug.Assert(pack.param_value_TRY(ph).Equals("xkOwphbatdxBsavqmx"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
                Debug.Assert(pack.target_system == (byte)(byte)62);
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            p323.param_id_SET("yxxtqb", PH) ;
            p323.param_value_SET("xkOwphbatdxBsavqmx", PH) ;
            p323.target_system = (byte)(byte)62;
            p323.target_component = (byte)(byte)21;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 16);
                Debug.Assert(pack.param_id_TRY(ph).Equals("bagmdfikiewpvemi"));
                Debug.Assert(pack.param_value_LEN(ph) == 9);
                Debug.Assert(pack.param_value_TRY(ph).Equals("oeybnDoWu"));
                Debug.Assert(pack.param_result == PARAM_ACK.PARAM_ACK_FAILED);
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_result = PARAM_ACK.PARAM_ACK_FAILED;
            p324.param_id_SET("bagmdfikiewpvemi", PH) ;
            p324.param_value_SET("oeybnDoWu", PH) ;
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.max_distance == (ushort)(ushort)16889);
                Debug.Assert(pack.sensor_type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
                Debug.Assert(pack.min_distance == (ushort)(ushort)35818);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)56951, (ushort)46953, (ushort)38570, (ushort)18983, (ushort)19786, (ushort)4636, (ushort)41811, (ushort)43538, (ushort)60557, (ushort)18129, (ushort)6374, (ushort)35146, (ushort)14863, (ushort)57178, (ushort)31666, (ushort)6016, (ushort)17002, (ushort)9793, (ushort)62613, (ushort)34674, (ushort)12545, (ushort)11613, (ushort)43335, (ushort)22064, (ushort)63617, (ushort)26528, (ushort)28277, (ushort)39039, (ushort)58828, (ushort)34053, (ushort)16121, (ushort)35248, (ushort)28711, (ushort)30856, (ushort)33289, (ushort)64819, (ushort)39840, (ushort)51014, (ushort)58490, (ushort)4606, (ushort)33393, (ushort)52240, (ushort)6922, (ushort)34572, (ushort)26824, (ushort)15629, (ushort)41630, (ushort)37085, (ushort)58557, (ushort)3247, (ushort)42912, (ushort)5039, (ushort)59539, (ushort)10888, (ushort)12357, (ushort)2845, (ushort)31161, (ushort)63542, (ushort)18251, (ushort)25861, (ushort)7328, (ushort)31407, (ushort)3663, (ushort)62799, (ushort)23242, (ushort)22042, (ushort)28513, (ushort)33896, (ushort)27972, (ushort)20586, (ushort)49855, (ushort)64754}));
                Debug.Assert(pack.time_usec == (ulong)8460765820885463504L);
                Debug.Assert(pack.increment == (byte)(byte)164);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.max_distance = (ushort)(ushort)16889;
            p330.distances_SET(new ushort[] {(ushort)56951, (ushort)46953, (ushort)38570, (ushort)18983, (ushort)19786, (ushort)4636, (ushort)41811, (ushort)43538, (ushort)60557, (ushort)18129, (ushort)6374, (ushort)35146, (ushort)14863, (ushort)57178, (ushort)31666, (ushort)6016, (ushort)17002, (ushort)9793, (ushort)62613, (ushort)34674, (ushort)12545, (ushort)11613, (ushort)43335, (ushort)22064, (ushort)63617, (ushort)26528, (ushort)28277, (ushort)39039, (ushort)58828, (ushort)34053, (ushort)16121, (ushort)35248, (ushort)28711, (ushort)30856, (ushort)33289, (ushort)64819, (ushort)39840, (ushort)51014, (ushort)58490, (ushort)4606, (ushort)33393, (ushort)52240, (ushort)6922, (ushort)34572, (ushort)26824, (ushort)15629, (ushort)41630, (ushort)37085, (ushort)58557, (ushort)3247, (ushort)42912, (ushort)5039, (ushort)59539, (ushort)10888, (ushort)12357, (ushort)2845, (ushort)31161, (ushort)63542, (ushort)18251, (ushort)25861, (ushort)7328, (ushort)31407, (ushort)3663, (ushort)62799, (ushort)23242, (ushort)22042, (ushort)28513, (ushort)33896, (ushort)27972, (ushort)20586, (ushort)49855, (ushort)64754}, 0) ;
            p330.increment = (byte)(byte)164;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p330.time_usec = (ulong)8460765820885463504L;
            p330.min_distance = (ushort)(ushort)35818;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
        }
    }
}