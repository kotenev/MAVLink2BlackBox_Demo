
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
                    ulong id = id__i(value);
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
                    ulong id = id__D(value);
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
                    ulong id = id__D(value);
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
                    ulong id = id__D(value);
                    BitUtils.set_bits(id, 7, data, 260);
                }
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
        new class SCRIPT_ITEM : GroundControl.SCRIPT_ITEM
        {
            public ushort seq //Sequence
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
            public string name_TRY(Inside ph)//The name of the mission script, NULL terminated.
            {
                if(ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) return null;
                return new string(name_GET(ph, new char[ph.items], 0));
            }
            public char[]name_GET(Inside ph, char[] dst_ch, int pos) //The name of the mission script, NULL terminated.
            {
                for(int BYTE = ph.BYTE, dst_max = pos + ph.items; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (char)((char) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public int name_LEN(Inside ph)
            {
                return (ph.field_bit !=  32 && !try_visit_field(ph, 32)  ||  !try_visit_item(ph, 0)) ? 0 : ph.items;
            }
        }
        new class SCRIPT_REQUEST : GroundControl.SCRIPT_REQUEST
        {
            public ushort seq //Sequence
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
        }
        new class SCRIPT_REQUEST_LIST : GroundControl.SCRIPT_REQUEST_LIST
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
        new class SCRIPT_COUNT : GroundControl.SCRIPT_COUNT
        {
            public ushort count //Number of script items in the sequence
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
        }
        new class SCRIPT_CURRENT : GroundControl.SCRIPT_CURRENT
        {
            public ushort seq //Active Sequence
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
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
            public void OnSCRIPT_ITEMReceive_direct(Channel src, Inside ph, SCRIPT_ITEM pack) {OnSCRIPT_ITEMReceive(this, ph,  pack);}
            public event SCRIPT_ITEMReceiveHandler OnSCRIPT_ITEMReceive;
            public delegate void SCRIPT_ITEMReceiveHandler(Channel src, Inside ph, SCRIPT_ITEM pack);
            public void OnSCRIPT_REQUESTReceive_direct(Channel src, Inside ph, SCRIPT_REQUEST pack) {OnSCRIPT_REQUESTReceive(this, ph,  pack);}
            public event SCRIPT_REQUESTReceiveHandler OnSCRIPT_REQUESTReceive;
            public delegate void SCRIPT_REQUESTReceiveHandler(Channel src, Inside ph, SCRIPT_REQUEST pack);
            public void OnSCRIPT_REQUEST_LISTReceive_direct(Channel src, Inside ph, SCRIPT_REQUEST_LIST pack) {OnSCRIPT_REQUEST_LISTReceive(this, ph,  pack);}
            public event SCRIPT_REQUEST_LISTReceiveHandler OnSCRIPT_REQUEST_LISTReceive;
            public delegate void SCRIPT_REQUEST_LISTReceiveHandler(Channel src, Inside ph, SCRIPT_REQUEST_LIST pack);
            public void OnSCRIPT_COUNTReceive_direct(Channel src, Inside ph, SCRIPT_COUNT pack) {OnSCRIPT_COUNTReceive(this, ph,  pack);}
            public event SCRIPT_COUNTReceiveHandler OnSCRIPT_COUNTReceive;
            public delegate void SCRIPT_COUNTReceiveHandler(Channel src, Inside ph, SCRIPT_COUNT pack);
            public void OnSCRIPT_CURRENTReceive_direct(Channel src, Inside ph, SCRIPT_CURRENT pack) {OnSCRIPT_CURRENTReceive(this, ph,  pack);}
            public event SCRIPT_CURRENTReceiveHandler OnSCRIPT_CURRENTReceive;
            public delegate void SCRIPT_CURRENTReceiveHandler(Channel src, Inside ph, SCRIPT_CURRENT pack);
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
                    case 180:
                        if(pack == null) return new SCRIPT_ITEM();
                        OnSCRIPT_ITEMReceive(this, ph, (SCRIPT_ITEM) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 181:
                        if(pack == null) return new SCRIPT_REQUEST();
                        OnSCRIPT_REQUESTReceive(this, ph, (SCRIPT_REQUEST) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 182:
                        if(pack == null) return new SCRIPT_REQUEST_LIST();
                        OnSCRIPT_REQUEST_LISTReceive(this, ph, (SCRIPT_REQUEST_LIST) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 183:
                        if(pack == null) return new SCRIPT_COUNT();
                        OnSCRIPT_COUNTReceive(this, ph, (SCRIPT_COUNT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 184:
                        if(pack == null) return new SCRIPT_CURRENT();
                        OnSCRIPT_CURRENTReceive(this, ph, (SCRIPT_CURRENT) pack);//no any host channels can receive this pack. Handle it with test channel handler
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
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
                Debug.Assert(pack.custom_mode == (uint)199566580U);
                Debug.Assert(pack.mavlink_version == (byte)(byte)149);
                Debug.Assert(pack.autopilot == MAV_AUTOPILOT.MAV_AUTOPILOT_UDB);
                Debug.Assert(pack.system_status == MAV_STATE.MAV_STATE_BOOT);
                Debug.Assert(pack.type == MAV_TYPE.MAV_TYPE_AIRSHIP);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.mavlink_version = (byte)(byte)149;
            p0.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
            p0.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_UDB;
            p0.system_status = MAV_STATE.MAV_STATE_BOOT;
            p0.custom_mode = (uint)199566580U;
            p0.type = MAV_TYPE.MAV_TYPE_AIRSHIP;
            SMP_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.load == (ushort)(ushort)3237);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)28868);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)12971);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)37487);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)51590);
                Debug.Assert(pack.current_battery == (short)(short)6587);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)36658);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 70);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW));
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)34054);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)3522);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION));
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.errors_count4 = (ushort)(ushort)28868;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.errors_comm = (ushort)(ushort)51590;
            p1.voltage_battery = (ushort)(ushort)34054;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION);
            p1.drop_rate_comm = (ushort)(ushort)12971;
            p1.battery_remaining = (sbyte)(sbyte) - 70;
            p1.load = (ushort)(ushort)3237;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW);
            p1.errors_count2 = (ushort)(ushort)37487;
            p1.errors_count3 = (ushort)(ushort)3522;
            p1.current_battery = (short)(short)6587;
            p1.errors_count1 = (ushort)(ushort)36658;
            SMP_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1911451661U);
                Debug.Assert(pack.time_unix_usec == (ulong)3441565658515298845L);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)3441565658515298845L;
            p2.time_boot_ms = (uint)1911451661U;
            SMP_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afx == (float)4.663648E37F);
                Debug.Assert(pack.yaw == (float) -2.3117177E38F);
                Debug.Assert(pack.vx == (float)5.9409576E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)22934);
                Debug.Assert(pack.y == (float)7.207902E36F);
                Debug.Assert(pack.x == (float)2.6799014E38F);
                Debug.Assert(pack.vy == (float)1.6507919E38F);
                Debug.Assert(pack.yaw_rate == (float) -2.1618132E38F);
                Debug.Assert(pack.afy == (float)2.388495E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.time_boot_ms == (uint)683113947U);
                Debug.Assert(pack.afz == (float) -1.1645462E38F);
                Debug.Assert(pack.z == (float)5.856585E37F);
                Debug.Assert(pack.vz == (float) -3.2990518E38F);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.y = (float)7.207902E36F;
            p3.vz = (float) -3.2990518E38F;
            p3.type_mask = (ushort)(ushort)22934;
            p3.x = (float)2.6799014E38F;
            p3.time_boot_ms = (uint)683113947U;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p3.afx = (float)4.663648E37F;
            p3.afz = (float) -1.1645462E38F;
            p3.yaw = (float) -2.3117177E38F;
            p3.vx = (float)5.9409576E37F;
            p3.z = (float)5.856585E37F;
            p3.vy = (float)1.6507919E38F;
            p3.yaw_rate = (float) -2.1618132E38F;
            p3.afy = (float)2.388495E38F;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)81);
                Debug.Assert(pack.target_system == (byte)(byte)245);
                Debug.Assert(pack.time_usec == (ulong)711988300982528350L);
                Debug.Assert(pack.seq == (uint)2222361229U);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.time_usec = (ulong)711988300982528350L;
            p4.seq = (uint)2222361229U;
            p4.target_system = (byte)(byte)245;
            p4.target_component = (byte)(byte)81;
            SMP_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.passkey_LEN(ph) == 24);
                Debug.Assert(pack.passkey_TRY(ph).Equals("vlnjnhfxThhpfkosoDnwhJwa"));
                Debug.Assert(pack.control_request == (byte)(byte)246);
                Debug.Assert(pack.target_system == (byte)(byte)204);
                Debug.Assert(pack.version == (byte)(byte)170);
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.passkey_SET("vlnjnhfxThhpfkosoDnwhJwa", PH) ;
            p5.version = (byte)(byte)170;
            p5.control_request = (byte)(byte)246;
            p5.target_system = (byte)(byte)204;
            SMP_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.control_request == (byte)(byte)203);
                Debug.Assert(pack.ack == (byte)(byte)190);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)163);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.ack = (byte)(byte)190;
            p6.gcs_system_id = (byte)(byte)163;
            p6.control_request = (byte)(byte)203;
            SMP_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 17);
                Debug.Assert(pack.key_TRY(ph).Equals("ungmghRehczibmevd"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("ungmghRehczibmevd", PH) ;
            SMP_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)200);
                Debug.Assert(pack.base_mode == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
                Debug.Assert(pack.custom_mode == (uint)1349077318U);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)200;
            p11.custom_mode = (uint)1349077318U;
            p11.base_mode = MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            SMP_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (short)(short) -9285);
                Debug.Assert(pack.target_system == (byte)(byte)168);
                Debug.Assert(pack.target_component == (byte)(byte)80);
                Debug.Assert(pack.param_id_LEN(ph) == 7);
                Debug.Assert(pack.param_id_TRY(ph).Equals("rfwzsfh"));
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.param_id_SET("rfwzsfh", PH) ;
            p20.param_index = (short)(short) -9285;
            p20.target_system = (byte)(byte)168;
            p20.target_component = (byte)(byte)80;
            SMP_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)60);
                Debug.Assert(pack.target_component == (byte)(byte)160);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)160;
            p21.target_system = (byte)(byte)60;
            SMP_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64);
                Debug.Assert(pack.param_id_LEN(ph) == 16);
                Debug.Assert(pack.param_id_TRY(ph).Equals("dyodphewwgzxngui"));
                Debug.Assert(pack.param_value == (float) -1.2298826E38F);
                Debug.Assert(pack.param_count == (ushort)(ushort)35274);
                Debug.Assert(pack.param_index == (ushort)(ushort)54431);
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_id_SET("dyodphewwgzxngui", PH) ;
            p22.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64;
            p22.param_count = (ushort)(ushort)35274;
            p22.param_value = (float) -1.2298826E38F;
            p22.param_index = (ushort)(ushort)54431;
            SMP_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)254);
                Debug.Assert(pack.param_value == (float) -1.8706323E37F);
                Debug.Assert(pack.target_system == (byte)(byte)104);
                Debug.Assert(pack.param_id_LEN(ph) == 15);
                Debug.Assert(pack.param_id_TRY(ph).Equals("egqulndxdpqcmsv"));
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.param_value = (float) -1.8706323E37F;
            p23.target_component = (byte)(byte)254;
            p23.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32;
            p23.param_id_SET("egqulndxdpqcmsv", PH) ;
            p23.target_system = (byte)(byte)104;
            SMP_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.epv == (ushort)(ushort)20548);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)2951133906U);
                Debug.Assert(pack.vel == (ushort)(ushort)5234);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)3323578909U);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -330710775);
                Debug.Assert(pack.lon == (int) -1342055565);
                Debug.Assert(pack.lat == (int) -831510723);
                Debug.Assert(pack.eph == (ushort)(ushort)57890);
                Debug.Assert(pack.satellites_visible == (byte)(byte)61);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)3752307297U);
                Debug.Assert(pack.cog == (ushort)(ushort)44195);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)2762736904U);
                Debug.Assert(pack.alt == (int)965499467);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
                Debug.Assert(pack.time_usec == (ulong)1997257311216839877L);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.cog = (ushort)(ushort)44195;
            p24.eph = (ushort)(ushort)57890;
            p24.lon = (int) -1342055565;
            p24.alt = (int)965499467;
            p24.lat = (int) -831510723;
            p24.vel_acc_SET((uint)2762736904U, PH) ;
            p24.hdg_acc_SET((uint)3323578909U, PH) ;
            p24.alt_ellipsoid_SET((int) -330710775, PH) ;
            p24.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX;
            p24.h_acc_SET((uint)3752307297U, PH) ;
            p24.vel = (ushort)(ushort)5234;
            p24.satellites_visible = (byte)(byte)61;
            p24.epv = (ushort)(ushort)20548;
            p24.time_usec = (ulong)1997257311216839877L;
            p24.v_acc_SET((uint)2951133906U, PH) ;
            SMP_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)36, (byte)184, (byte)157, (byte)174, (byte)180, (byte)109, (byte)78, (byte)36, (byte)16, (byte)28, (byte)243, (byte)126, (byte)47, (byte)47, (byte)205, (byte)49, (byte)207, (byte)45, (byte)240, (byte)53}));
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)14, (byte)193, (byte)22, (byte)101, (byte)34, (byte)110, (byte)82, (byte)57, (byte)16, (byte)98, (byte)221, (byte)249, (byte)11, (byte)214, (byte)237, (byte)13, (byte)148, (byte)33, (byte)215, (byte)121}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)222, (byte)23, (byte)128, (byte)7, (byte)98, (byte)126, (byte)213, (byte)239, (byte)168, (byte)50, (byte)193, (byte)253, (byte)216, (byte)242, (byte)100, (byte)254, (byte)23, (byte)44, (byte)255, (byte)237}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)171);
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)161, (byte)45, (byte)133, (byte)86, (byte)222, (byte)146, (byte)142, (byte)51, (byte)253, (byte)237, (byte)237, (byte)163, (byte)46, (byte)194, (byte)50, (byte)197, (byte)20, (byte)98, (byte)20, (byte)65}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)92, (byte)210, (byte)132, (byte)243, (byte)153, (byte)22, (byte)62, (byte)201, (byte)132, (byte)136, (byte)134, (byte)27, (byte)6, (byte)145, (byte)219, (byte)156, (byte)196, (byte)10, (byte)28, (byte)23}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_azimuth_SET(new byte[] {(byte)222, (byte)23, (byte)128, (byte)7, (byte)98, (byte)126, (byte)213, (byte)239, (byte)168, (byte)50, (byte)193, (byte)253, (byte)216, (byte)242, (byte)100, (byte)254, (byte)23, (byte)44, (byte)255, (byte)237}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)161, (byte)45, (byte)133, (byte)86, (byte)222, (byte)146, (byte)142, (byte)51, (byte)253, (byte)237, (byte)237, (byte)163, (byte)46, (byte)194, (byte)50, (byte)197, (byte)20, (byte)98, (byte)20, (byte)65}, 0) ;
            p25.satellites_visible = (byte)(byte)171;
            p25.satellite_prn_SET(new byte[] {(byte)36, (byte)184, (byte)157, (byte)174, (byte)180, (byte)109, (byte)78, (byte)36, (byte)16, (byte)28, (byte)243, (byte)126, (byte)47, (byte)47, (byte)205, (byte)49, (byte)207, (byte)45, (byte)240, (byte)53}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)14, (byte)193, (byte)22, (byte)101, (byte)34, (byte)110, (byte)82, (byte)57, (byte)16, (byte)98, (byte)221, (byte)249, (byte)11, (byte)214, (byte)237, (byte)13, (byte)148, (byte)33, (byte)215, (byte)121}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)92, (byte)210, (byte)132, (byte)243, (byte)153, (byte)22, (byte)62, (byte)201, (byte)132, (byte)136, (byte)134, (byte)27, (byte)6, (byte)145, (byte)219, (byte)156, (byte)196, (byte)10, (byte)28, (byte)23}, 0) ;
            SMP_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (short)(short) -8024);
                Debug.Assert(pack.xmag == (short)(short) -29431);
                Debug.Assert(pack.zmag == (short)(short)24063);
                Debug.Assert(pack.zacc == (short)(short)3731);
                Debug.Assert(pack.ymag == (short)(short)8091);
                Debug.Assert(pack.yacc == (short)(short)9985);
                Debug.Assert(pack.xgyro == (short)(short) -6643);
                Debug.Assert(pack.xacc == (short)(short)6291);
                Debug.Assert(pack.ygyro == (short)(short) -14323);
                Debug.Assert(pack.time_boot_ms == (uint)1549604491U);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.xmag = (short)(short) -29431;
            p26.ymag = (short)(short)8091;
            p26.yacc = (short)(short)9985;
            p26.zgyro = (short)(short) -8024;
            p26.zacc = (short)(short)3731;
            p26.xacc = (short)(short)6291;
            p26.xgyro = (short)(short) -6643;
            p26.ygyro = (short)(short) -14323;
            p26.time_boot_ms = (uint)1549604491U;
            p26.zmag = (short)(short)24063;
            SMP_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (short)(short) -15743);
                Debug.Assert(pack.zgyro == (short)(short)4606);
                Debug.Assert(pack.time_usec == (ulong)4714097179649704518L);
                Debug.Assert(pack.zmag == (short)(short) -544);
                Debug.Assert(pack.ymag == (short)(short) -4019);
                Debug.Assert(pack.ygyro == (short)(short)615);
                Debug.Assert(pack.xmag == (short)(short) -31910);
                Debug.Assert(pack.xacc == (short)(short) -2071);
                Debug.Assert(pack.zacc == (short)(short) -7644);
                Debug.Assert(pack.yacc == (short)(short)19053);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.zmag = (short)(short) -544;
            p27.zgyro = (short)(short)4606;
            p27.time_usec = (ulong)4714097179649704518L;
            p27.zacc = (short)(short) -7644;
            p27.xmag = (short)(short) -31910;
            p27.yacc = (short)(short)19053;
            p27.ymag = (short)(short) -4019;
            p27.xacc = (short)(short) -2071;
            p27.xgyro = (short)(short) -15743;
            p27.ygyro = (short)(short)615;
            SMP_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (short)(short)31810);
                Debug.Assert(pack.time_usec == (ulong)4124769252965833927L);
                Debug.Assert(pack.press_diff2 == (short)(short) -17514);
                Debug.Assert(pack.temperature == (short)(short)27432);
                Debug.Assert(pack.press_diff1 == (short)(short)27465);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_abs = (short)(short)31810;
            p28.press_diff1 = (short)(short)27465;
            p28.press_diff2 = (short)(short) -17514;
            p28.time_usec = (ulong)4124769252965833927L;
            p28.temperature = (short)(short)27432;
            SMP_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)19650);
                Debug.Assert(pack.press_abs == (float) -1.0358694E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1472045015U);
                Debug.Assert(pack.press_diff == (float)1.4686084E38F);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.press_abs = (float) -1.0358694E38F;
            p29.temperature = (short)(short)19650;
            p29.press_diff = (float)1.4686084E38F;
            p29.time_boot_ms = (uint)1472045015U;
            SMP_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)654450633U);
                Debug.Assert(pack.pitchspeed == (float)1.7329735E38F);
                Debug.Assert(pack.yawspeed == (float) -2.4557485E38F);
                Debug.Assert(pack.roll == (float)1.0415862E38F);
                Debug.Assert(pack.pitch == (float)2.8830887E38F);
                Debug.Assert(pack.rollspeed == (float)2.891174E38F);
                Debug.Assert(pack.yaw == (float)9.491451E37F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.roll = (float)1.0415862E38F;
            p30.yaw = (float)9.491451E37F;
            p30.pitch = (float)2.8830887E38F;
            p30.pitchspeed = (float)1.7329735E38F;
            p30.yawspeed = (float) -2.4557485E38F;
            p30.time_boot_ms = (uint)654450633U;
            p30.rollspeed = (float)2.891174E38F;
            SMP_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1759464990U);
                Debug.Assert(pack.q2 == (float)2.1490036E38F);
                Debug.Assert(pack.rollspeed == (float) -1.3101632E37F);
                Debug.Assert(pack.pitchspeed == (float) -1.6733308E38F);
                Debug.Assert(pack.q4 == (float) -3.0894919E38F);
                Debug.Assert(pack.yawspeed == (float) -2.9717411E38F);
                Debug.Assert(pack.q1 == (float)2.2373696E38F);
                Debug.Assert(pack.q3 == (float)2.1600284E38F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.pitchspeed = (float) -1.6733308E38F;
            p31.rollspeed = (float) -1.3101632E37F;
            p31.q3 = (float)2.1600284E38F;
            p31.q1 = (float)2.2373696E38F;
            p31.q4 = (float) -3.0894919E38F;
            p31.q2 = (float)2.1490036E38F;
            p31.yawspeed = (float) -2.9717411E38F;
            p31.time_boot_ms = (uint)1759464990U;
            SMP_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -2.7040762E38F);
                Debug.Assert(pack.x == (float)7.2061657E37F);
                Debug.Assert(pack.vy == (float) -2.4527506E38F);
                Debug.Assert(pack.vx == (float) -3.288133E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2332855704U);
                Debug.Assert(pack.y == (float)1.4226811E38F);
                Debug.Assert(pack.vz == (float)2.0741285E37F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.vz = (float)2.0741285E37F;
            p32.y = (float)1.4226811E38F;
            p32.time_boot_ms = (uint)2332855704U;
            p32.x = (float)7.2061657E37F;
            p32.vx = (float) -3.288133E38F;
            p32.z = (float) -2.7040762E38F;
            p32.vy = (float) -2.4527506E38F;
            SMP_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.relative_alt == (int) -1365970536);
                Debug.Assert(pack.vy == (short)(short) -31314);
                Debug.Assert(pack.hdg == (ushort)(ushort)25178);
                Debug.Assert(pack.vx == (short)(short)21229);
                Debug.Assert(pack.lat == (int)1475043220);
                Debug.Assert(pack.time_boot_ms == (uint)3436231070U);
                Debug.Assert(pack.vz == (short)(short)10193);
                Debug.Assert(pack.alt == (int)1646447415);
                Debug.Assert(pack.lon == (int)57445118);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.vx = (short)(short)21229;
            p33.lon = (int)57445118;
            p33.time_boot_ms = (uint)3436231070U;
            p33.lat = (int)1475043220;
            p33.vz = (short)(short)10193;
            p33.vy = (short)(short) -31314;
            p33.relative_alt = (int) -1365970536;
            p33.alt = (int)1646447415;
            p33.hdg = (ushort)(ushort)25178;
            SMP_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2789413877U);
                Debug.Assert(pack.chan3_scaled == (short)(short) -15471);
                Debug.Assert(pack.chan2_scaled == (short)(short)26066);
                Debug.Assert(pack.port == (byte)(byte)115);
                Debug.Assert(pack.chan7_scaled == (short)(short) -9187);
                Debug.Assert(pack.chan1_scaled == (short)(short)13180);
                Debug.Assert(pack.chan6_scaled == (short)(short)13760);
                Debug.Assert(pack.chan8_scaled == (short)(short)9981);
                Debug.Assert(pack.rssi == (byte)(byte)24);
                Debug.Assert(pack.chan4_scaled == (short)(short) -26978);
                Debug.Assert(pack.chan5_scaled == (short)(short)12691);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan8_scaled = (short)(short)9981;
            p34.chan2_scaled = (short)(short)26066;
            p34.chan5_scaled = (short)(short)12691;
            p34.chan3_scaled = (short)(short) -15471;
            p34.port = (byte)(byte)115;
            p34.chan6_scaled = (short)(short)13760;
            p34.chan7_scaled = (short)(short) -9187;
            p34.chan4_scaled = (short)(short) -26978;
            p34.time_boot_ms = (uint)2789413877U;
            p34.rssi = (byte)(byte)24;
            p34.chan1_scaled = (short)(short)13180;
            SMP_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)19441);
                Debug.Assert(pack.rssi == (byte)(byte)95);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)35066);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)46112);
                Debug.Assert(pack.port == (byte)(byte)12);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)12584);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)3132);
                Debug.Assert(pack.time_boot_ms == (uint)441376599U);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)13302);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)13233);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)45181);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan6_raw = (ushort)(ushort)45181;
            p35.port = (byte)(byte)12;
            p35.chan8_raw = (ushort)(ushort)35066;
            p35.chan2_raw = (ushort)(ushort)13302;
            p35.time_boot_ms = (uint)441376599U;
            p35.chan3_raw = (ushort)(ushort)46112;
            p35.chan4_raw = (ushort)(ushort)3132;
            p35.chan7_raw = (ushort)(ushort)19441;
            p35.chan1_raw = (ushort)(ushort)13233;
            p35.rssi = (byte)(byte)95;
            p35.chan5_raw = (ushort)(ushort)12584;
            SMP_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)14309);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)60522);
                Debug.Assert(pack.time_usec == (uint)1816021604U);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)51020);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)25953);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)61790);
                Debug.Assert(pack.port == (byte)(byte)14);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)33630);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)21842);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)1839);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)4739);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)42740);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)26509);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)51510);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)36586);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)29741);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)59801);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)26972);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo14_raw_SET((ushort)(ushort)29741, PH) ;
            p36.servo8_raw = (ushort)(ushort)51510;
            p36.servo2_raw = (ushort)(ushort)14309;
            p36.servo1_raw = (ushort)(ushort)61790;
            p36.servo7_raw = (ushort)(ushort)59801;
            p36.servo6_raw = (ushort)(ushort)25953;
            p36.servo10_raw_SET((ushort)(ushort)26972, PH) ;
            p36.servo11_raw_SET((ushort)(ushort)21842, PH) ;
            p36.servo3_raw = (ushort)(ushort)60522;
            p36.servo4_raw = (ushort)(ushort)33630;
            p36.servo9_raw_SET((ushort)(ushort)51020, PH) ;
            p36.servo13_raw_SET((ushort)(ushort)4739, PH) ;
            p36.port = (byte)(byte)14;
            p36.servo16_raw_SET((ushort)(ushort)42740, PH) ;
            p36.servo15_raw_SET((ushort)(ushort)1839, PH) ;
            p36.time_usec = (uint)1816021604U;
            p36.servo12_raw_SET((ushort)(ushort)36586, PH) ;
            p36.servo5_raw = (ushort)(ushort)26509;
            SMP_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)173);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.end_index == (short)(short)11275);
                Debug.Assert(pack.target_system == (byte)(byte)166);
                Debug.Assert(pack.start_index == (short)(short)14448);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.target_system = (byte)(byte)166;
            p37.target_component = (byte)(byte)173;
            p37.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p37.start_index = (short)(short)14448;
            p37.end_index = (short)(short)11275;
            SMP_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)214);
                Debug.Assert(pack.target_component == (byte)(byte)81);
                Debug.Assert(pack.start_index == (short)(short) -17786);
                Debug.Assert(pack.end_index == (short)(short)11513);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.target_component = (byte)(byte)81;
            p38.target_system = (byte)(byte)214;
            p38.start_index = (short)(short) -17786;
            p38.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p38.end_index = (short)(short)11513;
            SMP_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)220);
                Debug.Assert(pack.autocontinue == (byte)(byte)146);
                Debug.Assert(pack.param2 == (float) -5.546204E37F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_STORAGE_FORMAT);
                Debug.Assert(pack.current == (byte)(byte)128);
                Debug.Assert(pack.param3 == (float) -1.6677881E38F);
                Debug.Assert(pack.seq == (ushort)(ushort)20506);
                Debug.Assert(pack.target_component == (byte)(byte)87);
                Debug.Assert(pack.param4 == (float)2.5425144E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.param1 == (float)1.7872776E38F);
                Debug.Assert(pack.z == (float) -2.574652E38F);
                Debug.Assert(pack.y == (float)2.5336476E38F);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.x == (float)1.6943672E38F);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.target_system = (byte)(byte)220;
            p39.z = (float) -2.574652E38F;
            p39.current = (byte)(byte)128;
            p39.param1 = (float)1.7872776E38F;
            p39.command = MAV_CMD.MAV_CMD_STORAGE_FORMAT;
            p39.x = (float)1.6943672E38F;
            p39.autocontinue = (byte)(byte)146;
            p39.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p39.target_component = (byte)(byte)87;
            p39.y = (float)2.5336476E38F;
            p39.seq = (ushort)(ushort)20506;
            p39.frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p39.param3 = (float) -1.6677881E38F;
            p39.param2 = (float) -5.546204E37F;
            p39.param4 = (float)2.5425144E38F;
            SMP_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_system == (byte)(byte)63);
                Debug.Assert(pack.seq == (ushort)(ushort)58676);
                Debug.Assert(pack.target_component == (byte)(byte)63);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_system = (byte)(byte)63;
            p40.seq = (ushort)(ushort)58676;
            p40.target_component = (byte)(byte)63;
            p40.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            SMP_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)13);
                Debug.Assert(pack.seq == (ushort)(ushort)42227);
                Debug.Assert(pack.target_component == (byte)(byte)239);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.seq = (ushort)(ushort)42227;
            p41.target_component = (byte)(byte)239;
            p41.target_system = (byte)(byte)13;
            SMP_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)34106);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)34106;
            SMP_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)97);
                Debug.Assert(pack.target_system == (byte)(byte)204);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_system = (byte)(byte)204;
            p43.target_component = (byte)(byte)97;
            p43.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            SMP_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (ushort)(ushort)17635);
                Debug.Assert(pack.target_component == (byte)(byte)214);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)169);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.count = (ushort)(ushort)17635;
            p44.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p44.target_system = (byte)(byte)169;
            p44.target_component = (byte)(byte)214;
            SMP_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)92);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)217);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_system = (byte)(byte)92;
            p45.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p45.target_component = (byte)(byte)217;
            SMP_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)41224);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)41224;
            SMP_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM7);
                Debug.Assert(pack.target_system == (byte)(byte)78);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_component == (byte)(byte)232);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.type = MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM7;
            p47.target_component = (byte)(byte)232;
            p47.target_system = (byte)(byte)78;
            p47.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            SMP_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)35);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)8253273802084302556L);
                Debug.Assert(pack.longitude == (int)1607177056);
                Debug.Assert(pack.altitude == (int) -1345191958);
                Debug.Assert(pack.latitude == (int)1675014274);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.time_usec_SET((ulong)8253273802084302556L, PH) ;
            p48.longitude = (int)1607177056;
            p48.latitude = (int)1675014274;
            p48.target_system = (byte)(byte)35;
            p48.altitude = (int) -1345191958;
            SMP_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)3550602016537254463L);
                Debug.Assert(pack.longitude == (int) -437291262);
                Debug.Assert(pack.latitude == (int)2076380037);
                Debug.Assert(pack.altitude == (int)1130697282);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.latitude = (int)2076380037;
            p49.altitude = (int)1130697282;
            p49.longitude = (int) -437291262;
            p49.time_usec_SET((ulong)3550602016537254463L, PH) ;
            SMP_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value0 == (float) -2.9383555E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("s"));
                Debug.Assert(pack.param_index == (short)(short)27610);
                Debug.Assert(pack.target_component == (byte)(byte)121);
                Debug.Assert(pack.target_system == (byte)(byte)179);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)232);
                Debug.Assert(pack.param_value_min == (float)3.2990015E38F);
                Debug.Assert(pack.param_value_max == (float) -3.0934472E38F);
                Debug.Assert(pack.scale == (float) -3.355389E38F);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.scale = (float) -3.355389E38F;
            p50.target_system = (byte)(byte)179;
            p50.param_value_min = (float)3.2990015E38F;
            p50.target_component = (byte)(byte)121;
            p50.param_value_max = (float) -3.0934472E38F;
            p50.param_id_SET("s", PH) ;
            p50.param_value0 = (float) -2.9383555E38F;
            p50.param_index = (short)(short)27610;
            p50.parameter_rc_channel_index = (byte)(byte)232;
            SMP_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_system == (byte)(byte)78);
                Debug.Assert(pack.seq == (ushort)(ushort)4054);
                Debug.Assert(pack.target_component == (byte)(byte)114);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.seq = (ushort)(ushort)4054;
            p51.target_system = (byte)(byte)78;
            p51.target_component = (byte)(byte)114;
            p51.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            SMP_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)120);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.p1x == (float) -2.213943E38F);
                Debug.Assert(pack.p1y == (float)8.5351904E36F);
                Debug.Assert(pack.p2z == (float)9.122131E36F);
                Debug.Assert(pack.target_component == (byte)(byte)211);
                Debug.Assert(pack.p2y == (float) -2.776223E38F);
                Debug.Assert(pack.p1z == (float)1.0661773E38F);
                Debug.Assert(pack.p2x == (float)1.2781134E37F);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p2y = (float) -2.776223E38F;
            p54.p2x = (float)1.2781134E37F;
            p54.p1x = (float) -2.213943E38F;
            p54.target_component = (byte)(byte)211;
            p54.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p54.p1z = (float)1.0661773E38F;
            p54.p2z = (float)9.122131E36F;
            p54.p1y = (float)8.5351904E36F;
            p54.target_system = (byte)(byte)120;
            SMP_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1x == (float) -2.2496379E38F);
                Debug.Assert(pack.p1z == (float) -2.0329152E38F);
                Debug.Assert(pack.p2x == (float) -2.370329E38F);
                Debug.Assert(pack.p2y == (float)5.057221E37F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.p2z == (float) -2.7779356E38F);
                Debug.Assert(pack.p1y == (float)3.1108154E38F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p55.p2y = (float)5.057221E37F;
            p55.p1x = (float) -2.2496379E38F;
            p55.p1z = (float) -2.0329152E38F;
            p55.p2z = (float) -2.7779356E38F;
            p55.p1y = (float)3.1108154E38F;
            p55.p2x = (float) -2.370329E38F;
            SMP_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float)2.2291642E38F);
                Debug.Assert(pack.pitchspeed == (float) -1.9083052E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-6.07007E37F, 2.8764992E38F, 1.5735691E38F, -2.463311E38F, -2.0877193E38F, 2.9400902E37F, 2.0448742E38F, -1.2291974E38F, 3.834277E37F}));
                Debug.Assert(pack.time_usec == (ulong)2590448642640147726L);
                Debug.Assert(pack.yawspeed == (float)2.9112929E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.1571254E38F, -5.026209E37F, 1.0207218E38F, 1.1646013E38F}));
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.time_usec = (ulong)2590448642640147726L;
            p61.q_SET(new float[] {1.1571254E38F, -5.026209E37F, 1.0207218E38F, 1.1646013E38F}, 0) ;
            p61.rollspeed = (float)2.2291642E38F;
            p61.yawspeed = (float)2.9112929E38F;
            p61.pitchspeed = (float) -1.9083052E38F;
            p61.covariance_SET(new float[] {-6.07007E37F, 2.8764992E38F, 1.5735691E38F, -2.463311E38F, -2.0877193E38F, 2.9400902E37F, 2.0448742E38F, -1.2291974E38F, 3.834277E37F}, 0) ;
            SMP_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aspd_error == (float)2.365464E38F);
                Debug.Assert(pack.nav_roll == (float) -2.5167627E38F);
                Debug.Assert(pack.nav_bearing == (short)(short)10493);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)10481);
                Debug.Assert(pack.xtrack_error == (float)7.045894E37F);
                Debug.Assert(pack.target_bearing == (short)(short) -31745);
                Debug.Assert(pack.nav_pitch == (float)9.275122E37F);
                Debug.Assert(pack.alt_error == (float)1.2571317E38F);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.xtrack_error = (float)7.045894E37F;
            p62.wp_dist = (ushort)(ushort)10481;
            p62.aspd_error = (float)2.365464E38F;
            p62.nav_bearing = (short)(short)10493;
            p62.nav_pitch = (float)9.275122E37F;
            p62.alt_error = (float)1.2571317E38F;
            p62.target_bearing = (short)(short) -31745;
            p62.nav_roll = (float) -2.5167627E38F;
            SMP_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float)1.5047409E38F);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
                Debug.Assert(pack.vx == (float)3.1559843E38F);
                Debug.Assert(pack.alt == (int) -244620419);
                Debug.Assert(pack.lat == (int)200566372);
                Debug.Assert(pack.relative_alt == (int) -1508019062);
                Debug.Assert(pack.time_usec == (ulong)2915278011569342224L);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.9334997E38F, -2.064282E38F, 9.91857E37F, -1.3769079E38F, 1.0868976E38F, 1.8121716E38F, -3.6129175E37F, 5.8330283E37F, 1.8774544E38F, 2.6089115E38F, -3.016079E38F, 5.252887E37F, -1.4388971E38F, 8.162977E37F, 1.9751578E38F, -1.9905923E38F, -1.4773038E38F, 1.4817476E38F, -1.9154856E38F, 2.2737985E38F, 2.9262675E37F, -4.4969474E37F, 5.8425727E37F, 3.7125042E37F, 8.2988516E37F, 2.8271292E38F, 2.218279E38F, -2.5551366E38F, -2.2565039E38F, 1.9899774E38F, 8.862683E37F, -6.58347E37F, 2.9635813E38F, 1.1700649E38F, 3.0181351E38F, 2.8439056E38F}));
                Debug.Assert(pack.vz == (float) -2.771929E38F);
                Debug.Assert(pack.lon == (int) -1468483447);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.relative_alt = (int) -1508019062;
            p63.covariance_SET(new float[] {-2.9334997E38F, -2.064282E38F, 9.91857E37F, -1.3769079E38F, 1.0868976E38F, 1.8121716E38F, -3.6129175E37F, 5.8330283E37F, 1.8774544E38F, 2.6089115E38F, -3.016079E38F, 5.252887E37F, -1.4388971E38F, 8.162977E37F, 1.9751578E38F, -1.9905923E38F, -1.4773038E38F, 1.4817476E38F, -1.9154856E38F, 2.2737985E38F, 2.9262675E37F, -4.4969474E37F, 5.8425727E37F, 3.7125042E37F, 8.2988516E37F, 2.8271292E38F, 2.218279E38F, -2.5551366E38F, -2.2565039E38F, 1.9899774E38F, 8.862683E37F, -6.58347E37F, 2.9635813E38F, 1.1700649E38F, 3.0181351E38F, 2.8439056E38F}, 0) ;
            p63.lat = (int)200566372;
            p63.time_usec = (ulong)2915278011569342224L;
            p63.vz = (float) -2.771929E38F;
            p63.vy = (float)1.5047409E38F;
            p63.alt = (int) -244620419;
            p63.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO;
            p63.lon = (int) -1468483447;
            p63.vx = (float)3.1559843E38F;
            SMP_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float)4.321333E37F);
                Debug.Assert(pack.vy == (float) -3.245818E38F);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
                Debug.Assert(pack.time_usec == (ulong)4960730757090297011L);
                Debug.Assert(pack.ax == (float)2.1815088E38F);
                Debug.Assert(pack.az == (float) -1.9544361E38F);
                Debug.Assert(pack.x == (float)8.691735E37F);
                Debug.Assert(pack.ay == (float)5.8911567E37F);
                Debug.Assert(pack.vx == (float)4.6078334E37F);
                Debug.Assert(pack.y == (float) -4.382439E37F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.4331564E38F, -1.2543588E38F, -1.906511E37F, 1.1782779E38F, 1.3154594E38F, -1.7812018E37F, 1.8737874E38F, -1.3420915E38F, 1.3314628E38F, 6.4850945E36F, 1.2198031E38F, -9.595194E37F, 3.3185724E38F, -5.723231E37F, 1.6203388E38F, 7.2116293E37F, -4.5524607E36F, 1.8547148E38F, 3.001574E38F, 2.4371954E38F, -5.663318E37F, 3.1724737E38F, 3.3450395E38F, 3.2408544E38F, -8.4164313E37F, -1.0652039E38F, -1.0788604E38F, 3.2470393E38F, 1.3944296E38F, -1.6997757E38F, 1.954295E38F, -9.53064E37F, 2.425347E37F, 3.2719922E38F, -3.3633831E38F, 2.888717E37F, -1.4864822E38F, 1.5858986E38F, 1.894671E38F, 2.3128384E37F, -1.795924E38F, 6.3718366E37F, 2.3559477E38F, 1.3355048E38F, 2.0798169E38F}));
                Debug.Assert(pack.z == (float)5.5145727E37F);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.z = (float)5.5145727E37F;
            p64.az = (float) -1.9544361E38F;
            p64.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS;
            p64.y = (float) -4.382439E37F;
            p64.time_usec = (ulong)4960730757090297011L;
            p64.vz = (float)4.321333E37F;
            p64.covariance_SET(new float[] {1.4331564E38F, -1.2543588E38F, -1.906511E37F, 1.1782779E38F, 1.3154594E38F, -1.7812018E37F, 1.8737874E38F, -1.3420915E38F, 1.3314628E38F, 6.4850945E36F, 1.2198031E38F, -9.595194E37F, 3.3185724E38F, -5.723231E37F, 1.6203388E38F, 7.2116293E37F, -4.5524607E36F, 1.8547148E38F, 3.001574E38F, 2.4371954E38F, -5.663318E37F, 3.1724737E38F, 3.3450395E38F, 3.2408544E38F, -8.4164313E37F, -1.0652039E38F, -1.0788604E38F, 3.2470393E38F, 1.3944296E38F, -1.6997757E38F, 1.954295E38F, -9.53064E37F, 2.425347E37F, 3.2719922E38F, -3.3633831E38F, 2.888717E37F, -1.4864822E38F, 1.5858986E38F, 1.894671E38F, 2.3128384E37F, -1.795924E38F, 6.3718366E37F, 2.3559477E38F, 1.3355048E38F, 2.0798169E38F}, 0) ;
            p64.vy = (float) -3.245818E38F;
            p64.ay = (float)5.8911567E37F;
            p64.ax = (float)2.1815088E38F;
            p64.vx = (float)4.6078334E37F;
            p64.x = (float)8.691735E37F;
            SMP_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)52019);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)59432);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)35947);
                Debug.Assert(pack.time_boot_ms == (uint)1706330228U);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)7852);
                Debug.Assert(pack.chancount == (byte)(byte)129);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)16583);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)57924);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)14870);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)18345);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)8835);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)46748);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)19909);
                Debug.Assert(pack.rssi == (byte)(byte)198);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)34018);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)26411);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)50252);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)44367);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)21627);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)53600);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)48521);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan16_raw = (ushort)(ushort)57924;
            p65.time_boot_ms = (uint)1706330228U;
            p65.chan8_raw = (ushort)(ushort)18345;
            p65.chan11_raw = (ushort)(ushort)34018;
            p65.chan6_raw = (ushort)(ushort)50252;
            p65.chan9_raw = (ushort)(ushort)44367;
            p65.chan15_raw = (ushort)(ushort)8835;
            p65.chan13_raw = (ushort)(ushort)59432;
            p65.chancount = (byte)(byte)129;
            p65.chan14_raw = (ushort)(ushort)19909;
            p65.chan4_raw = (ushort)(ushort)48521;
            p65.chan17_raw = (ushort)(ushort)52019;
            p65.chan3_raw = (ushort)(ushort)16583;
            p65.chan12_raw = (ushort)(ushort)35947;
            p65.chan7_raw = (ushort)(ushort)46748;
            p65.chan2_raw = (ushort)(ushort)21627;
            p65.chan5_raw = (ushort)(ushort)26411;
            p65.chan10_raw = (ushort)(ushort)14870;
            p65.chan18_raw = (ushort)(ushort)53600;
            p65.chan1_raw = (ushort)(ushort)7852;
            p65.rssi = (byte)(byte)198;
            SMP_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)58);
                Debug.Assert(pack.target_system == (byte)(byte)0);
                Debug.Assert(pack.start_stop == (byte)(byte)86);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)49253);
                Debug.Assert(pack.req_stream_id == (byte)(byte)214);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_component = (byte)(byte)58;
            p66.req_message_rate = (ushort)(ushort)49253;
            p66.target_system = (byte)(byte)0;
            p66.req_stream_id = (byte)(byte)214;
            p66.start_stop = (byte)(byte)86;
            SMP_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_rate == (ushort)(ushort)23762);
                Debug.Assert(pack.on_off == (byte)(byte)39);
                Debug.Assert(pack.stream_id == (byte)(byte)71);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.on_off = (byte)(byte)39;
            p67.stream_id = (byte)(byte)71;
            p67.message_rate = (ushort)(ushort)23762;
            SMP_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target == (byte)(byte)75);
                Debug.Assert(pack.r == (short)(short) -30504);
                Debug.Assert(pack.buttons == (ushort)(ushort)51579);
                Debug.Assert(pack.y == (short)(short) -2708);
                Debug.Assert(pack.z == (short)(short) -16266);
                Debug.Assert(pack.x == (short)(short) -28441);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.z = (short)(short) -16266;
            p69.y = (short)(short) -2708;
            p69.buttons = (ushort)(ushort)51579;
            p69.x = (short)(short) -28441;
            p69.r = (short)(short) -30504;
            p69.target = (byte)(byte)75;
            SMP_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)625);
                Debug.Assert(pack.target_component == (byte)(byte)140);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)3989);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)15267);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)47193);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)44067);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)20672);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)56206);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)52980);
                Debug.Assert(pack.target_system == (byte)(byte)177);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan4_raw = (ushort)(ushort)15267;
            p70.chan8_raw = (ushort)(ushort)44067;
            p70.chan2_raw = (ushort)(ushort)56206;
            p70.chan3_raw = (ushort)(ushort)625;
            p70.chan1_raw = (ushort)(ushort)20672;
            p70.chan5_raw = (ushort)(ushort)47193;
            p70.target_component = (byte)(byte)140;
            p70.chan6_raw = (ushort)(ushort)52980;
            p70.target_system = (byte)(byte)177;
            p70.chan7_raw = (ushort)(ushort)3989;
            SMP_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current == (byte)(byte)248);
                Debug.Assert(pack.target_system == (byte)(byte)129);
                Debug.Assert(pack.param1 == (float) -3.3878843E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_NED);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)55);
                Debug.Assert(pack.y == (int) -1122394647);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_GUIDED_MASTER);
                Debug.Assert(pack.param2 == (float)5.014051E37F);
                Debug.Assert(pack.param4 == (float) -3.1874093E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)216);
                Debug.Assert(pack.seq == (ushort)(ushort)5132);
                Debug.Assert(pack.z == (float) -1.1317225E38F);
                Debug.Assert(pack.x == (int)1445080433);
                Debug.Assert(pack.param3 == (float) -2.230299E38F);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.z = (float) -1.1317225E38F;
            p73.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p73.param1 = (float) -3.3878843E38F;
            p73.param2 = (float)5.014051E37F;
            p73.target_component = (byte)(byte)55;
            p73.command = MAV_CMD.MAV_CMD_DO_GUIDED_MASTER;
            p73.param3 = (float) -2.230299E38F;
            p73.seq = (ushort)(ushort)5132;
            p73.target_system = (byte)(byte)129;
            p73.frame = MAV_FRAME.MAV_FRAME_BODY_NED;
            p73.current = (byte)(byte)248;
            p73.autocontinue = (byte)(byte)216;
            p73.x = (int)1445080433;
            p73.param4 = (float) -3.1874093E38F;
            p73.y = (int) -1122394647;
            SMP_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.climb == (float) -1.2663821E38F);
                Debug.Assert(pack.heading == (short)(short) -12183);
                Debug.Assert(pack.alt == (float) -1.8129608E38F);
                Debug.Assert(pack.airspeed == (float) -1.7254992E38F);
                Debug.Assert(pack.groundspeed == (float)2.478117E37F);
                Debug.Assert(pack.throttle == (ushort)(ushort)47698);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.alt = (float) -1.8129608E38F;
            p74.groundspeed = (float)2.478117E37F;
            p74.heading = (short)(short) -12183;
            p74.climb = (float) -1.2663821E38F;
            p74.throttle = (ushort)(ushort)47698;
            p74.airspeed = (float) -1.7254992E38F;
            SMP_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (int)452270425);
                Debug.Assert(pack.target_component == (byte)(byte)38);
                Debug.Assert(pack.param4 == (float) -4.8949603E37F);
                Debug.Assert(pack.param2 == (float)4.401915E37F);
                Debug.Assert(pack.param3 == (float)1.651076E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)94);
                Debug.Assert(pack.z == (float)1.6054104E37F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION);
                Debug.Assert(pack.target_system == (byte)(byte)172);
                Debug.Assert(pack.y == (int)1210020507);
                Debug.Assert(pack.current == (byte)(byte)137);
                Debug.Assert(pack.param1 == (float)2.3210678E38F);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.param3 = (float)1.651076E38F;
            p75.param4 = (float) -4.8949603E37F;
            p75.frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p75.command = MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION;
            p75.y = (int)1210020507;
            p75.target_system = (byte)(byte)172;
            p75.param1 = (float)2.3210678E38F;
            p75.autocontinue = (byte)(byte)94;
            p75.z = (float)1.6054104E37F;
            p75.target_component = (byte)(byte)38;
            p75.x = (int)452270425;
            p75.param2 = (float)4.401915E37F;
            p75.current = (byte)(byte)137;
            SMP_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param3 == (float)2.2524628E38F);
                Debug.Assert(pack.confirmation == (byte)(byte)222);
                Debug.Assert(pack.param1 == (float) -2.9583527E38F);
                Debug.Assert(pack.param4 == (float)2.8999803E38F);
                Debug.Assert(pack.param2 == (float) -7.8827235E37F);
                Debug.Assert(pack.param7 == (float) -7.3837235E37F);
                Debug.Assert(pack.target_component == (byte)(byte)97);
                Debug.Assert(pack.param5 == (float)7.8931284E37F);
                Debug.Assert(pack.target_system == (byte)(byte)11);
                Debug.Assert(pack.param6 == (float) -2.1338547E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_USER_2);
            };
            GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.param6 = (float) -2.1338547E38F;
            p76.param5 = (float)7.8931284E37F;
            p76.param7 = (float) -7.3837235E37F;
            p76.param4 = (float)2.8999803E38F;
            p76.confirmation = (byte)(byte)222;
            p76.command = MAV_CMD.MAV_CMD_USER_2;
            p76.param1 = (float) -2.9583527E38F;
            p76.param3 = (float)2.2524628E38F;
            p76.target_system = (byte)(byte)11;
            p76.target_component = (byte)(byte)97;
            p76.param2 = (float) -7.8827235E37F;
            CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_STORAGE_FORMAT);
                Debug.Assert(pack.result_param2_TRY(ph) == (int)1240416203);
                Debug.Assert(pack.result == MAV_RESULT.MAV_RESULT_FAILED);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)16);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)120);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)119);
            };
            GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.progress_SET((byte)(byte)120, PH) ;
            p77.target_component_SET((byte)(byte)16, PH) ;
            p77.result = MAV_RESULT.MAV_RESULT_FAILED;
            p77.target_system_SET((byte)(byte)119, PH) ;
            p77.result_param2_SET((int)1240416203, PH) ;
            p77.command = MAV_CMD.MAV_CMD_STORAGE_FORMAT;
            CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1474089057U);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)77);
                Debug.Assert(pack.pitch == (float) -2.775161E38F);
                Debug.Assert(pack.mode_switch == (byte)(byte)198);
                Debug.Assert(pack.thrust == (float) -2.3333421E38F);
                Debug.Assert(pack.roll == (float) -9.359158E37F);
                Debug.Assert(pack.yaw == (float) -1.0832672E38F);
            };
            GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.time_boot_ms = (uint)1474089057U;
            p81.manual_override_switch = (byte)(byte)77;
            p81.thrust = (float) -2.3333421E38F;
            p81.mode_switch = (byte)(byte)198;
            p81.pitch = (float) -2.775161E38F;
            p81.roll = (float) -9.359158E37F;
            p81.yaw = (float) -1.0832672E38F;
            CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.thrust == (float) -1.4468435E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)74);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.7741553E38F, -9.946852E37F, -1.6696093E38F, 3.7569693E37F}));
                Debug.Assert(pack.body_roll_rate == (float)2.2784158E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3867219476U);
                Debug.Assert(pack.body_pitch_rate == (float) -2.9317006E38F);
                Debug.Assert(pack.target_system == (byte)(byte)158);
                Debug.Assert(pack.body_yaw_rate == (float) -1.3774058E38F);
                Debug.Assert(pack.target_component == (byte)(byte)181);
            };
            GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.time_boot_ms = (uint)3867219476U;
            p82.target_component = (byte)(byte)181;
            p82.target_system = (byte)(byte)158;
            p82.body_roll_rate = (float)2.2784158E38F;
            p82.q_SET(new float[] {-1.7741553E38F, -9.946852E37F, -1.6696093E38F, 3.7569693E37F}, 0) ;
            p82.type_mask = (byte)(byte)74;
            p82.body_yaw_rate = (float) -1.3774058E38F;
            p82.body_pitch_rate = (float) -2.9317006E38F;
            p82.thrust = (float) -1.4468435E38F;
            CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)585079561U);
                Debug.Assert(pack.body_yaw_rate == (float)8.1449676E36F);
                Debug.Assert(pack.body_pitch_rate == (float)4.9433845E37F);
                Debug.Assert(pack.body_roll_rate == (float)1.1994675E38F);
                Debug.Assert(pack.thrust == (float)1.7538508E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)51);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.2393975E38F, 1.438916E38F, 7.916046E37F, -1.5719684E38F}));
            };
            GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.body_pitch_rate = (float)4.9433845E37F;
            p83.q_SET(new float[] {-2.2393975E38F, 1.438916E38F, 7.916046E37F, -1.5719684E38F}, 0) ;
            p83.body_yaw_rate = (float)8.1449676E36F;
            p83.thrust = (float)1.7538508E38F;
            p83.body_roll_rate = (float)1.1994675E38F;
            p83.type_mask = (byte)(byte)51;
            p83.time_boot_ms = (uint)585079561U;
            CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (ushort)(ushort)57742);
                Debug.Assert(pack.x == (float)2.3604918E38F);
                Debug.Assert(pack.z == (float)2.9315998E38F);
                Debug.Assert(pack.time_boot_ms == (uint)539729196U);
                Debug.Assert(pack.afy == (float) -3.1220028E38F);
                Debug.Assert(pack.afx == (float)1.3946893E37F);
                Debug.Assert(pack.vy == (float)2.4121586E38F);
                Debug.Assert(pack.target_component == (byte)(byte)133);
                Debug.Assert(pack.vz == (float) -1.4821626E37F);
                Debug.Assert(pack.afz == (float) -3.1143234E38F);
                Debug.Assert(pack.y == (float) -2.387997E37F);
                Debug.Assert(pack.yaw == (float)1.7970426E38F);
                Debug.Assert(pack.target_system == (byte)(byte)9);
                Debug.Assert(pack.yaw_rate == (float)1.1896363E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.vx == (float) -1.0387039E38F);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.y = (float) -2.387997E37F;
            p84.vx = (float) -1.0387039E38F;
            p84.vz = (float) -1.4821626E37F;
            p84.vy = (float)2.4121586E38F;
            p84.yaw_rate = (float)1.1896363E38F;
            p84.target_system = (byte)(byte)9;
            p84.x = (float)2.3604918E38F;
            p84.afx = (float)1.3946893E37F;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_MISSION;
            p84.time_boot_ms = (uint)539729196U;
            p84.type_mask = (ushort)(ushort)57742;
            p84.yaw = (float)1.7970426E38F;
            p84.target_component = (byte)(byte)133;
            p84.afz = (float) -3.1143234E38F;
            p84.afy = (float) -3.1220028E38F;
            p84.z = (float)2.9315998E38F;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (ushort)(ushort)5614);
                Debug.Assert(pack.afz == (float)2.3282582E38F);
                Debug.Assert(pack.lon_int == (int) -1535445583);
                Debug.Assert(pack.yaw_rate == (float)3.266184E38F);
                Debug.Assert(pack.afx == (float) -2.604497E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.target_component == (byte)(byte)239);
                Debug.Assert(pack.target_system == (byte)(byte)114);
                Debug.Assert(pack.vz == (float)2.2362277E38F);
                Debug.Assert(pack.alt == (float) -1.3358491E38F);
                Debug.Assert(pack.afy == (float)4.8402044E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1977299513U);
                Debug.Assert(pack.yaw == (float)5.300247E37F);
                Debug.Assert(pack.vx == (float)2.7608223E38F);
                Debug.Assert(pack.vy == (float)2.452006E38F);
                Debug.Assert(pack.lat_int == (int) -1511889147);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.vz = (float)2.2362277E38F;
            p86.type_mask = (ushort)(ushort)5614;
            p86.yaw_rate = (float)3.266184E38F;
            p86.time_boot_ms = (uint)1977299513U;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p86.alt = (float) -1.3358491E38F;
            p86.vx = (float)2.7608223E38F;
            p86.yaw = (float)5.300247E37F;
            p86.target_component = (byte)(byte)239;
            p86.lon_int = (int) -1535445583;
            p86.vy = (float)2.452006E38F;
            p86.target_system = (byte)(byte)114;
            p86.afz = (float)2.3282582E38F;
            p86.afx = (float) -2.604497E38F;
            p86.lat_int = (int) -1511889147;
            p86.afy = (float)4.8402044E37F;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw_rate == (float)6.850351E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)25860);
                Debug.Assert(pack.lon_int == (int) -775240418);
                Debug.Assert(pack.lat_int == (int)1233887245);
                Debug.Assert(pack.alt == (float)2.4471636E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.afy == (float) -2.4805746E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3819062527U);
                Debug.Assert(pack.afz == (float) -1.1175435E38F);
                Debug.Assert(pack.vx == (float) -7.9707527E37F);
                Debug.Assert(pack.afx == (float) -1.2406919E38F);
                Debug.Assert(pack.vy == (float)1.6146697E38F);
                Debug.Assert(pack.vz == (float) -4.0960747E37F);
                Debug.Assert(pack.yaw == (float)9.148254E37F);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.type_mask = (ushort)(ushort)25860;
            p87.lon_int = (int) -775240418;
            p87.afy = (float) -2.4805746E38F;
            p87.afx = (float) -1.2406919E38F;
            p87.vy = (float)1.6146697E38F;
            p87.afz = (float) -1.1175435E38F;
            p87.vz = (float) -4.0960747E37F;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p87.lat_int = (int)1233887245;
            p87.yaw_rate = (float)6.850351E37F;
            p87.yaw = (float)9.148254E37F;
            p87.vx = (float) -7.9707527E37F;
            p87.time_boot_ms = (uint)3819062527U;
            p87.alt = (float)2.4471636E38F;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float) -1.4008081E38F);
                Debug.Assert(pack.y == (float)2.0542152E38F);
                Debug.Assert(pack.pitch == (float) -1.4353041E38F);
                Debug.Assert(pack.yaw == (float) -4.5732833E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1820256562U);
                Debug.Assert(pack.x == (float) -1.253681E38F);
                Debug.Assert(pack.z == (float)2.632059E37F);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.x = (float) -1.253681E38F;
            p89.z = (float)2.632059E37F;
            p89.y = (float)2.0542152E38F;
            p89.time_boot_ms = (uint)1820256562U;
            p89.pitch = (float) -1.4353041E38F;
            p89.yaw = (float) -4.5732833E37F;
            p89.roll = (float) -1.4008081E38F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yawspeed == (float)2.595933E38F);
                Debug.Assert(pack.rollspeed == (float)1.5378325E38F);
                Debug.Assert(pack.pitchspeed == (float) -2.2809656E37F);
                Debug.Assert(pack.vy == (short)(short) -9287);
                Debug.Assert(pack.pitch == (float) -1.9487299E38F);
                Debug.Assert(pack.yaw == (float)7.4972294E37F);
                Debug.Assert(pack.vz == (short)(short) -32650);
                Debug.Assert(pack.lat == (int) -758215833);
                Debug.Assert(pack.time_usec == (ulong)4992664011117878150L);
                Debug.Assert(pack.roll == (float)7.731E37F);
                Debug.Assert(pack.zacc == (short)(short)8013);
                Debug.Assert(pack.xacc == (short)(short) -24299);
                Debug.Assert(pack.vx == (short)(short)13606);
                Debug.Assert(pack.lon == (int)1675050003);
                Debug.Assert(pack.yacc == (short)(short) -6024);
                Debug.Assert(pack.alt == (int)37016798);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.xacc = (short)(short) -24299;
            p90.time_usec = (ulong)4992664011117878150L;
            p90.lon = (int)1675050003;
            p90.alt = (int)37016798;
            p90.vx = (short)(short)13606;
            p90.rollspeed = (float)1.5378325E38F;
            p90.vz = (short)(short) -32650;
            p90.pitchspeed = (float) -2.2809656E37F;
            p90.roll = (float)7.731E37F;
            p90.yacc = (short)(short) -6024;
            p90.zacc = (short)(short)8013;
            p90.yawspeed = (float)2.595933E38F;
            p90.vy = (short)(short) -9287;
            p90.yaw = (float)7.4972294E37F;
            p90.lat = (int) -758215833;
            p90.pitch = (float) -1.9487299E38F;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aux3 == (float)3.355464E38F);
                Debug.Assert(pack.aux1 == (float) -6.154799E37F);
                Debug.Assert(pack.throttle == (float) -1.1718058E38F);
                Debug.Assert(pack.pitch_elevator == (float) -1.883912E38F);
                Debug.Assert(pack.time_usec == (ulong)955696925452454249L);
                Debug.Assert(pack.aux2 == (float)6.6653124E37F);
                Debug.Assert(pack.nav_mode == (byte)(byte)242);
                Debug.Assert(pack.roll_ailerons == (float)8.3355623E37F);
                Debug.Assert(pack.yaw_rudder == (float) -5.130223E37F);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
                Debug.Assert(pack.aux4 == (float) -3.3005967E38F);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.yaw_rudder = (float) -5.130223E37F;
            p91.mode = MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            p91.pitch_elevator = (float) -1.883912E38F;
            p91.throttle = (float) -1.1718058E38F;
            p91.aux3 = (float)3.355464E38F;
            p91.roll_ailerons = (float)8.3355623E37F;
            p91.aux1 = (float) -6.154799E37F;
            p91.nav_mode = (byte)(byte)242;
            p91.time_usec = (ulong)955696925452454249L;
            p91.aux4 = (float) -3.3005967E38F;
            p91.aux2 = (float)6.6653124E37F;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)14788);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)61251);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)64119);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)22682);
                Debug.Assert(pack.rssi == (byte)(byte)246);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)28603);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)11570);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)32914);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)3806);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)40063);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)43796);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)30582);
                Debug.Assert(pack.time_usec == (ulong)7520471604146622750L);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)21825);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan10_raw = (ushort)(ushort)14788;
            p92.chan7_raw = (ushort)(ushort)40063;
            p92.rssi = (byte)(byte)246;
            p92.chan5_raw = (ushort)(ushort)21825;
            p92.chan4_raw = (ushort)(ushort)3806;
            p92.time_usec = (ulong)7520471604146622750L;
            p92.chan8_raw = (ushort)(ushort)22682;
            p92.chan11_raw = (ushort)(ushort)11570;
            p92.chan12_raw = (ushort)(ushort)28603;
            p92.chan2_raw = (ushort)(ushort)32914;
            p92.chan3_raw = (ushort)(ushort)30582;
            p92.chan1_raw = (ushort)(ushort)64119;
            p92.chan9_raw = (ushort)(ushort)61251;
            p92.chan6_raw = (ushort)(ushort)43796;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)4745707117034491263L);
                Debug.Assert(pack.flags == (ulong)672288566607337707L);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_MANUAL_DISARMED);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-3.3701278E38F, -2.1668418E38F, 6.0978806E37F, 5.1436733E37F, 1.022981E38F, -2.8303965E38F, -1.345852E38F, 1.0183503E38F, -1.9034112E37F, 2.269568E38F, 1.8371306E38F, -8.583943E36F, -4.4664893E37F, 2.824962E38F, -2.6855715E38F, 2.3777604E38F}));
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.controls_SET(new float[] {-3.3701278E38F, -2.1668418E38F, 6.0978806E37F, 5.1436733E37F, 1.022981E38F, -2.8303965E38F, -1.345852E38F, 1.0183503E38F, -1.9034112E37F, 2.269568E38F, 1.8371306E38F, -8.583943E36F, -4.4664893E37F, 2.824962E38F, -2.6855715E38F, 2.3777604E38F}, 0) ;
            p93.mode = MAV_MODE.MAV_MODE_MANUAL_DISARMED;
            p93.flags = (ulong)672288566607337707L;
            p93.time_usec = (ulong)4745707117034491263L;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_comp_m_y == (float)4.2664003E37F);
                Debug.Assert(pack.sensor_id == (byte)(byte)150);
                Debug.Assert(pack.flow_x == (short)(short)31000);
                Debug.Assert(pack.flow_y == (short)(short)30992);
                Debug.Assert(pack.quality == (byte)(byte)210);
                Debug.Assert(pack.ground_distance == (float) -2.207648E37F);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float)5.18168E37F);
                Debug.Assert(pack.time_usec == (ulong)2428747655879942211L);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)2.5705102E38F);
                Debug.Assert(pack.flow_comp_m_x == (float) -3.3745155E38F);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_comp_m_x = (float) -3.3745155E38F;
            p100.ground_distance = (float) -2.207648E37F;
            p100.flow_rate_y_SET((float)5.18168E37F, PH) ;
            p100.quality = (byte)(byte)210;
            p100.flow_x = (short)(short)31000;
            p100.flow_comp_m_y = (float)4.2664003E37F;
            p100.flow_y = (short)(short)30992;
            p100.time_usec = (ulong)2428747655879942211L;
            p100.sensor_id = (byte)(byte)150;
            p100.flow_rate_x_SET((float)2.5705102E38F, PH) ;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)5693013840826178822L);
                Debug.Assert(pack.pitch == (float) -3.0840029E38F);
                Debug.Assert(pack.x == (float)1.352791E38F);
                Debug.Assert(pack.roll == (float) -2.3911892E38F);
                Debug.Assert(pack.yaw == (float)1.0825374E38F);
                Debug.Assert(pack.z == (float)4.790658E37F);
                Debug.Assert(pack.y == (float)8.628058E37F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.x = (float)1.352791E38F;
            p101.roll = (float) -2.3911892E38F;
            p101.pitch = (float) -3.0840029E38F;
            p101.usec = (ulong)5693013840826178822L;
            p101.y = (float)8.628058E37F;
            p101.z = (float)4.790658E37F;
            p101.yaw = (float)1.0825374E38F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)2969153057077759101L);
                Debug.Assert(pack.y == (float) -7.143317E37F);
                Debug.Assert(pack.pitch == (float)3.2218458E38F);
                Debug.Assert(pack.yaw == (float)2.5140736E38F);
                Debug.Assert(pack.x == (float)5.612241E37F);
                Debug.Assert(pack.z == (float) -1.040197E38F);
                Debug.Assert(pack.roll == (float)8.3632275E37F);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.x = (float)5.612241E37F;
            p102.pitch = (float)3.2218458E38F;
            p102.usec = (ulong)2969153057077759101L;
            p102.roll = (float)8.3632275E37F;
            p102.y = (float) -7.143317E37F;
            p102.z = (float) -1.040197E38F;
            p102.yaw = (float)2.5140736E38F;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)2.0716853E38F);
                Debug.Assert(pack.usec == (ulong)6741965015659549191L);
                Debug.Assert(pack.y == (float)9.799394E37F);
                Debug.Assert(pack.z == (float)2.8461591E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)6741965015659549191L;
            p103.z = (float)2.8461591E38F;
            p103.y = (float)9.799394E37F;
            p103.x = (float)2.0716853E38F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -1.619167E38F);
                Debug.Assert(pack.pitch == (float)2.3789149E38F);
                Debug.Assert(pack.y == (float)2.4772655E38F);
                Debug.Assert(pack.roll == (float)3.7019444E37F);
                Debug.Assert(pack.x == (float)2.2827039E38F);
                Debug.Assert(pack.usec == (ulong)657473473565028509L);
                Debug.Assert(pack.yaw == (float) -1.1277722E38F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.y = (float)2.4772655E38F;
            p104.pitch = (float)2.3789149E38F;
            p104.roll = (float)3.7019444E37F;
            p104.x = (float)2.2827039E38F;
            p104.usec = (ulong)657473473565028509L;
            p104.z = (float) -1.619167E38F;
            p104.yaw = (float) -1.1277722E38F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (float) -3.1483174E38F);
                Debug.Assert(pack.abs_pressure == (float)1.6269156E38F);
                Debug.Assert(pack.zacc == (float)1.1665353E38F);
                Debug.Assert(pack.xacc == (float)1.313281E38F);
                Debug.Assert(pack.zmag == (float)3.6204588E37F);
                Debug.Assert(pack.zgyro == (float) -8.272968E37F);
                Debug.Assert(pack.time_usec == (ulong)3449343132888633934L);
                Debug.Assert(pack.ymag == (float)2.7107436E38F);
                Debug.Assert(pack.diff_pressure == (float)1.4784259E38F);
                Debug.Assert(pack.ygyro == (float) -2.4277776E37F);
                Debug.Assert(pack.pressure_alt == (float)3.2828392E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)36472);
                Debug.Assert(pack.temperature == (float) -3.2281743E38F);
                Debug.Assert(pack.xgyro == (float) -6.360554E37F);
                Debug.Assert(pack.xmag == (float)2.1361272E38F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.diff_pressure = (float)1.4784259E38F;
            p105.xacc = (float)1.313281E38F;
            p105.fields_updated = (ushort)(ushort)36472;
            p105.xmag = (float)2.1361272E38F;
            p105.yacc = (float) -3.1483174E38F;
            p105.ymag = (float)2.7107436E38F;
            p105.zgyro = (float) -8.272968E37F;
            p105.temperature = (float) -3.2281743E38F;
            p105.ygyro = (float) -2.4277776E37F;
            p105.pressure_alt = (float)3.2828392E38F;
            p105.xgyro = (float) -6.360554E37F;
            p105.time_usec = (ulong)3449343132888633934L;
            p105.zmag = (float)3.6204588E37F;
            p105.zacc = (float)1.1665353E38F;
            p105.abs_pressure = (float)1.6269156E38F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_delta_distance_us == (uint)3309990832U);
                Debug.Assert(pack.integrated_zgyro == (float) -3.2277995E37F);
                Debug.Assert(pack.temperature == (short)(short) -30816);
                Debug.Assert(pack.integrated_xgyro == (float) -1.8181104E37F);
                Debug.Assert(pack.quality == (byte)(byte)57);
                Debug.Assert(pack.integration_time_us == (uint)473042037U);
                Debug.Assert(pack.sensor_id == (byte)(byte)82);
                Debug.Assert(pack.integrated_y == (float)3.1805613E38F);
                Debug.Assert(pack.integrated_ygyro == (float) -1.1447333E38F);
                Debug.Assert(pack.time_usec == (ulong)8438886371552925899L);
                Debug.Assert(pack.distance == (float)2.3599018E38F);
                Debug.Assert(pack.integrated_x == (float)1.0998566E37F);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.temperature = (short)(short) -30816;
            p106.integration_time_us = (uint)473042037U;
            p106.distance = (float)2.3599018E38F;
            p106.integrated_zgyro = (float) -3.2277995E37F;
            p106.integrated_y = (float)3.1805613E38F;
            p106.time_usec = (ulong)8438886371552925899L;
            p106.time_delta_distance_us = (uint)3309990832U;
            p106.integrated_x = (float)1.0998566E37F;
            p106.integrated_xgyro = (float) -1.8181104E37F;
            p106.quality = (byte)(byte)57;
            p106.sensor_id = (byte)(byte)82;
            p106.integrated_ygyro = (float) -1.1447333E38F;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (float) -5.321416E37F);
                Debug.Assert(pack.zmag == (float)4.5952518E36F);
                Debug.Assert(pack.xmag == (float) -2.7394033E38F);
                Debug.Assert(pack.fields_updated == (uint)4086334091U);
                Debug.Assert(pack.xgyro == (float)6.859855E37F);
                Debug.Assert(pack.time_usec == (ulong)8889605133399732260L);
                Debug.Assert(pack.temperature == (float)1.5359592E38F);
                Debug.Assert(pack.pressure_alt == (float) -2.7360135E38F);
                Debug.Assert(pack.ygyro == (float) -2.5659816E38F);
                Debug.Assert(pack.zacc == (float)2.1713713E38F);
                Debug.Assert(pack.yacc == (float) -3.1616334E38F);
                Debug.Assert(pack.ymag == (float)2.3293873E38F);
                Debug.Assert(pack.xacc == (float) -1.0680539E38F);
                Debug.Assert(pack.abs_pressure == (float) -1.8818707E38F);
                Debug.Assert(pack.diff_pressure == (float)8.355678E36F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.zmag = (float)4.5952518E36F;
            p107.ygyro = (float) -2.5659816E38F;
            p107.zgyro = (float) -5.321416E37F;
            p107.xmag = (float) -2.7394033E38F;
            p107.yacc = (float) -3.1616334E38F;
            p107.diff_pressure = (float)8.355678E36F;
            p107.time_usec = (ulong)8889605133399732260L;
            p107.zacc = (float)2.1713713E38F;
            p107.abs_pressure = (float) -1.8818707E38F;
            p107.ymag = (float)2.3293873E38F;
            p107.fields_updated = (uint)4086334091U;
            p107.xgyro = (float)6.859855E37F;
            p107.xacc = (float) -1.0680539E38F;
            p107.pressure_alt = (float) -2.7360135E38F;
            p107.temperature = (float)1.5359592E38F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)1.8174501E38F);
                Debug.Assert(pack.zgyro == (float)3.3423074E38F);
                Debug.Assert(pack.roll == (float)9.095791E37F);
                Debug.Assert(pack.xacc == (float) -3.380712E38F);
                Debug.Assert(pack.q1 == (float)9.861256E37F);
                Debug.Assert(pack.alt == (float)1.6731905E38F);
                Debug.Assert(pack.pitch == (float) -3.864397E37F);
                Debug.Assert(pack.zacc == (float) -2.0217869E38F);
                Debug.Assert(pack.ve == (float)2.0389566E38F);
                Debug.Assert(pack.vd == (float) -2.1303834E38F);
                Debug.Assert(pack.q2 == (float) -2.6334295E37F);
                Debug.Assert(pack.ygyro == (float)1.4751196E38F);
                Debug.Assert(pack.xgyro == (float)3.2044226E38F);
                Debug.Assert(pack.q4 == (float) -9.396215E37F);
                Debug.Assert(pack.q3 == (float)2.1671175E38F);
                Debug.Assert(pack.vn == (float) -1.10789E38F);
                Debug.Assert(pack.yacc == (float) -3.6014235E37F);
                Debug.Assert(pack.std_dev_horz == (float) -3.3296468E38F);
                Debug.Assert(pack.lon == (float)5.818655E37F);
                Debug.Assert(pack.std_dev_vert == (float) -1.8762357E38F);
                Debug.Assert(pack.lat == (float) -6.8969086E37F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.ygyro = (float)1.4751196E38F;
            p108.q2 = (float) -2.6334295E37F;
            p108.vn = (float) -1.10789E38F;
            p108.yaw = (float)1.8174501E38F;
            p108.roll = (float)9.095791E37F;
            p108.ve = (float)2.0389566E38F;
            p108.lat = (float) -6.8969086E37F;
            p108.q3 = (float)2.1671175E38F;
            p108.zacc = (float) -2.0217869E38F;
            p108.xgyro = (float)3.2044226E38F;
            p108.pitch = (float) -3.864397E37F;
            p108.xacc = (float) -3.380712E38F;
            p108.std_dev_horz = (float) -3.3296468E38F;
            p108.lon = (float)5.818655E37F;
            p108.q4 = (float) -9.396215E37F;
            p108.std_dev_vert = (float) -1.8762357E38F;
            p108.vd = (float) -2.1303834E38F;
            p108.alt = (float)1.6731905E38F;
            p108.q1 = (float)9.861256E37F;
            p108.yacc = (float) -3.6014235E37F;
            p108.zgyro = (float)3.3423074E38F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.noise == (byte)(byte)63);
                Debug.Assert(pack.rssi == (byte)(byte)253);
                Debug.Assert(pack.txbuf == (byte)(byte)238);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)48837);
                Debug.Assert(pack.remrssi == (byte)(byte)241);
                Debug.Assert(pack.remnoise == (byte)(byte)227);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)17171);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.fixed_ = (ushort)(ushort)48837;
            p109.rssi = (byte)(byte)253;
            p109.noise = (byte)(byte)63;
            p109.rxerrors = (ushort)(ushort)17171;
            p109.txbuf = (byte)(byte)238;
            p109.remrssi = (byte)(byte)241;
            p109.remnoise = (byte)(byte)227;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)216);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)61, (byte)55, (byte)19, (byte)187, (byte)248, (byte)249, (byte)63, (byte)104, (byte)228, (byte)222, (byte)96, (byte)180, (byte)32, (byte)227, (byte)45, (byte)168, (byte)12, (byte)159, (byte)251, (byte)167, (byte)196, (byte)130, (byte)192, (byte)108, (byte)10, (byte)252, (byte)22, (byte)27, (byte)178, (byte)27, (byte)14, (byte)39, (byte)10, (byte)203, (byte)229, (byte)54, (byte)229, (byte)208, (byte)11, (byte)87, (byte)87, (byte)173, (byte)84, (byte)216, (byte)230, (byte)10, (byte)179, (byte)208, (byte)160, (byte)33, (byte)254, (byte)190, (byte)212, (byte)102, (byte)254, (byte)81, (byte)211, (byte)24, (byte)64, (byte)215, (byte)102, (byte)156, (byte)157, (byte)193, (byte)109, (byte)232, (byte)35, (byte)8, (byte)245, (byte)219, (byte)162, (byte)176, (byte)93, (byte)24, (byte)245, (byte)69, (byte)104, (byte)218, (byte)189, (byte)117, (byte)100, (byte)139, (byte)218, (byte)183, (byte)14, (byte)84, (byte)157, (byte)26, (byte)15, (byte)130, (byte)182, (byte)200, (byte)93, (byte)99, (byte)26, (byte)16, (byte)254, (byte)32, (byte)118, (byte)158, (byte)56, (byte)241, (byte)236, (byte)227, (byte)198, (byte)240, (byte)121, (byte)66, (byte)20, (byte)127, (byte)31, (byte)237, (byte)78, (byte)39, (byte)213, (byte)196, (byte)198, (byte)87, (byte)234, (byte)127, (byte)147, (byte)162, (byte)17, (byte)176, (byte)122, (byte)70, (byte)240, (byte)183, (byte)200, (byte)182, (byte)18, (byte)208, (byte)157, (byte)203, (byte)239, (byte)58, (byte)122, (byte)100, (byte)102, (byte)218, (byte)173, (byte)121, (byte)66, (byte)207, (byte)219, (byte)231, (byte)2, (byte)80, (byte)252, (byte)72, (byte)207, (byte)246, (byte)70, (byte)70, (byte)119, (byte)66, (byte)54, (byte)29, (byte)247, (byte)178, (byte)25, (byte)67, (byte)24, (byte)24, (byte)147, (byte)234, (byte)125, (byte)230, (byte)160, (byte)247, (byte)204, (byte)50, (byte)194, (byte)149, (byte)149, (byte)115, (byte)79, (byte)20, (byte)4, (byte)129, (byte)34, (byte)83, (byte)232, (byte)29, (byte)42, (byte)125, (byte)3, (byte)179, (byte)117, (byte)106, (byte)199, (byte)214, (byte)190, (byte)78, (byte)177, (byte)198, (byte)155, (byte)181, (byte)173, (byte)164, (byte)69, (byte)112, (byte)158, (byte)41, (byte)81, (byte)51, (byte)195, (byte)112, (byte)64, (byte)41, (byte)137, (byte)187, (byte)177, (byte)61, (byte)89, (byte)128, (byte)68, (byte)32, (byte)38, (byte)102, (byte)178, (byte)16, (byte)234, (byte)1, (byte)246, (byte)201, (byte)20, (byte)125, (byte)209, (byte)253, (byte)182, (byte)1, (byte)86, (byte)202, (byte)212, (byte)185, (byte)207, (byte)230, (byte)130, (byte)74, (byte)32, (byte)16, (byte)244, (byte)177, (byte)189, (byte)63, (byte)169, (byte)138, (byte)84, (byte)51, (byte)95}));
                Debug.Assert(pack.target_system == (byte)(byte)54);
                Debug.Assert(pack.target_network == (byte)(byte)166);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_network = (byte)(byte)166;
            p110.target_system = (byte)(byte)54;
            p110.payload_SET(new byte[] {(byte)61, (byte)55, (byte)19, (byte)187, (byte)248, (byte)249, (byte)63, (byte)104, (byte)228, (byte)222, (byte)96, (byte)180, (byte)32, (byte)227, (byte)45, (byte)168, (byte)12, (byte)159, (byte)251, (byte)167, (byte)196, (byte)130, (byte)192, (byte)108, (byte)10, (byte)252, (byte)22, (byte)27, (byte)178, (byte)27, (byte)14, (byte)39, (byte)10, (byte)203, (byte)229, (byte)54, (byte)229, (byte)208, (byte)11, (byte)87, (byte)87, (byte)173, (byte)84, (byte)216, (byte)230, (byte)10, (byte)179, (byte)208, (byte)160, (byte)33, (byte)254, (byte)190, (byte)212, (byte)102, (byte)254, (byte)81, (byte)211, (byte)24, (byte)64, (byte)215, (byte)102, (byte)156, (byte)157, (byte)193, (byte)109, (byte)232, (byte)35, (byte)8, (byte)245, (byte)219, (byte)162, (byte)176, (byte)93, (byte)24, (byte)245, (byte)69, (byte)104, (byte)218, (byte)189, (byte)117, (byte)100, (byte)139, (byte)218, (byte)183, (byte)14, (byte)84, (byte)157, (byte)26, (byte)15, (byte)130, (byte)182, (byte)200, (byte)93, (byte)99, (byte)26, (byte)16, (byte)254, (byte)32, (byte)118, (byte)158, (byte)56, (byte)241, (byte)236, (byte)227, (byte)198, (byte)240, (byte)121, (byte)66, (byte)20, (byte)127, (byte)31, (byte)237, (byte)78, (byte)39, (byte)213, (byte)196, (byte)198, (byte)87, (byte)234, (byte)127, (byte)147, (byte)162, (byte)17, (byte)176, (byte)122, (byte)70, (byte)240, (byte)183, (byte)200, (byte)182, (byte)18, (byte)208, (byte)157, (byte)203, (byte)239, (byte)58, (byte)122, (byte)100, (byte)102, (byte)218, (byte)173, (byte)121, (byte)66, (byte)207, (byte)219, (byte)231, (byte)2, (byte)80, (byte)252, (byte)72, (byte)207, (byte)246, (byte)70, (byte)70, (byte)119, (byte)66, (byte)54, (byte)29, (byte)247, (byte)178, (byte)25, (byte)67, (byte)24, (byte)24, (byte)147, (byte)234, (byte)125, (byte)230, (byte)160, (byte)247, (byte)204, (byte)50, (byte)194, (byte)149, (byte)149, (byte)115, (byte)79, (byte)20, (byte)4, (byte)129, (byte)34, (byte)83, (byte)232, (byte)29, (byte)42, (byte)125, (byte)3, (byte)179, (byte)117, (byte)106, (byte)199, (byte)214, (byte)190, (byte)78, (byte)177, (byte)198, (byte)155, (byte)181, (byte)173, (byte)164, (byte)69, (byte)112, (byte)158, (byte)41, (byte)81, (byte)51, (byte)195, (byte)112, (byte)64, (byte)41, (byte)137, (byte)187, (byte)177, (byte)61, (byte)89, (byte)128, (byte)68, (byte)32, (byte)38, (byte)102, (byte)178, (byte)16, (byte)234, (byte)1, (byte)246, (byte)201, (byte)20, (byte)125, (byte)209, (byte)253, (byte)182, (byte)1, (byte)86, (byte)202, (byte)212, (byte)185, (byte)207, (byte)230, (byte)130, (byte)74, (byte)32, (byte)16, (byte)244, (byte)177, (byte)189, (byte)63, (byte)169, (byte)138, (byte)84, (byte)51, (byte)95}, 0) ;
            p110.target_component = (byte)(byte)216;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tc1 == (long) -2250520728854476888L);
                Debug.Assert(pack.ts1 == (long) -2578409410815444663L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long) -2578409410815444663L;
            p111.tc1 = (long) -2250520728854476888L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)4435293315454084249L);
                Debug.Assert(pack.seq == (uint)1396424151U);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)4435293315454084249L;
            p112.seq = (uint)1396424151U;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellites_visible == (byte)(byte)199);
                Debug.Assert(pack.cog == (ushort)(ushort)59557);
                Debug.Assert(pack.ve == (short)(short) -20548);
                Debug.Assert(pack.vn == (short)(short) -19142);
                Debug.Assert(pack.lat == (int) -1161487742);
                Debug.Assert(pack.lon == (int)1080690657);
                Debug.Assert(pack.vd == (short)(short)7260);
                Debug.Assert(pack.alt == (int)1451020655);
                Debug.Assert(pack.fix_type == (byte)(byte)84);
                Debug.Assert(pack.vel == (ushort)(ushort)13086);
                Debug.Assert(pack.epv == (ushort)(ushort)20280);
                Debug.Assert(pack.eph == (ushort)(ushort)33994);
                Debug.Assert(pack.time_usec == (ulong)5835909347209951542L);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.vn = (short)(short) -19142;
            p113.lon = (int)1080690657;
            p113.alt = (int)1451020655;
            p113.cog = (ushort)(ushort)59557;
            p113.lat = (int) -1161487742;
            p113.eph = (ushort)(ushort)33994;
            p113.fix_type = (byte)(byte)84;
            p113.vel = (ushort)(ushort)13086;
            p113.ve = (short)(short) -20548;
            p113.epv = (ushort)(ushort)20280;
            p113.satellites_visible = (byte)(byte)199;
            p113.vd = (short)(short)7260;
            p113.time_usec = (ulong)5835909347209951542L;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)1912);
                Debug.Assert(pack.time_delta_distance_us == (uint)1056338389U);
                Debug.Assert(pack.integrated_y == (float)1.5328361E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)137);
                Debug.Assert(pack.quality == (byte)(byte)83);
                Debug.Assert(pack.integration_time_us == (uint)1469052360U);
                Debug.Assert(pack.distance == (float) -2.5574845E38F);
                Debug.Assert(pack.integrated_x == (float) -2.1551242E37F);
                Debug.Assert(pack.integrated_zgyro == (float) -2.2563408E38F);
                Debug.Assert(pack.integrated_xgyro == (float)1.6695209E38F);
                Debug.Assert(pack.time_usec == (ulong)2352441622045595236L);
                Debug.Assert(pack.integrated_ygyro == (float) -7.1589363E36F);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)2352441622045595236L;
            p114.temperature = (short)(short)1912;
            p114.integrated_zgyro = (float) -2.2563408E38F;
            p114.integrated_xgyro = (float)1.6695209E38F;
            p114.time_delta_distance_us = (uint)1056338389U;
            p114.distance = (float) -2.5574845E38F;
            p114.integration_time_us = (uint)1469052360U;
            p114.integrated_ygyro = (float) -7.1589363E36F;
            p114.quality = (byte)(byte)83;
            p114.integrated_y = (float)1.5328361E38F;
            p114.sensor_id = (byte)(byte)137;
            p114.integrated_x = (float) -2.1551242E37F;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -705085943);
                Debug.Assert(pack.zacc == (short)(short) -1326);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)43950);
                Debug.Assert(pack.xacc == (short)(short)9455);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)22308);
                Debug.Assert(pack.alt == (int)1741038740);
                Debug.Assert(pack.rollspeed == (float) -2.9852271E38F);
                Debug.Assert(pack.time_usec == (ulong)7743959122049166062L);
                Debug.Assert(pack.pitchspeed == (float) -1.6252151E38F);
                Debug.Assert(pack.vy == (short)(short) -8533);
                Debug.Assert(pack.yacc == (short)(short) -28595);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {-1.7660227E38F, -3.5404224E37F, -1.5403676E38F, 2.5896364E38F}));
                Debug.Assert(pack.vz == (short)(short)30155);
                Debug.Assert(pack.lat == (int) -1606325616);
                Debug.Assert(pack.vx == (short)(short) -14622);
                Debug.Assert(pack.yawspeed == (float) -2.20896E38F);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.vy = (short)(short) -8533;
            p115.pitchspeed = (float) -1.6252151E38F;
            p115.vx = (short)(short) -14622;
            p115.zacc = (short)(short) -1326;
            p115.lon = (int) -705085943;
            p115.attitude_quaternion_SET(new float[] {-1.7660227E38F, -3.5404224E37F, -1.5403676E38F, 2.5896364E38F}, 0) ;
            p115.xacc = (short)(short)9455;
            p115.yawspeed = (float) -2.20896E38F;
            p115.rollspeed = (float) -2.9852271E38F;
            p115.ind_airspeed = (ushort)(ushort)22308;
            p115.vz = (short)(short)30155;
            p115.lat = (int) -1606325616;
            p115.time_usec = (ulong)7743959122049166062L;
            p115.true_airspeed = (ushort)(ushort)43950;
            p115.yacc = (short)(short) -28595;
            p115.alt = (int)1741038740;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (short)(short) -13184);
                Debug.Assert(pack.zacc == (short)(short) -22623);
                Debug.Assert(pack.zgyro == (short)(short) -18045);
                Debug.Assert(pack.time_boot_ms == (uint)2035849460U);
                Debug.Assert(pack.zmag == (short)(short) -4804);
                Debug.Assert(pack.ygyro == (short)(short)1399);
                Debug.Assert(pack.yacc == (short)(short) -8718);
                Debug.Assert(pack.xmag == (short)(short)75);
                Debug.Assert(pack.xacc == (short)(short) -4755);
                Debug.Assert(pack.ymag == (short)(short)10834);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.time_boot_ms = (uint)2035849460U;
            p116.xacc = (short)(short) -4755;
            p116.ygyro = (short)(short)1399;
            p116.xmag = (short)(short)75;
            p116.xgyro = (short)(short) -13184;
            p116.zacc = (short)(short) -22623;
            p116.ymag = (short)(short)10834;
            p116.yacc = (short)(short) -8718;
            p116.zgyro = (short)(short) -18045;
            p116.zmag = (short)(short) -4804;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start == (ushort)(ushort)25657);
                Debug.Assert(pack.target_component == (byte)(byte)197);
                Debug.Assert(pack.end == (ushort)(ushort)30789);
                Debug.Assert(pack.target_system == (byte)(byte)192);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.end = (ushort)(ushort)30789;
            p117.target_system = (byte)(byte)192;
            p117.target_component = (byte)(byte)197;
            p117.start = (ushort)(ushort)25657;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_log_num == (ushort)(ushort)5119);
                Debug.Assert(pack.time_utc == (uint)43235241U);
                Debug.Assert(pack.num_logs == (ushort)(ushort)9475);
                Debug.Assert(pack.size == (uint)3061114589U);
                Debug.Assert(pack.id == (ushort)(ushort)57771);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.last_log_num = (ushort)(ushort)5119;
            p118.size = (uint)3061114589U;
            p118.time_utc = (uint)43235241U;
            p118.num_logs = (ushort)(ushort)9475;
            p118.id = (ushort)(ushort)57771;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)31930);
                Debug.Assert(pack.target_system == (byte)(byte)48);
                Debug.Assert(pack.target_component == (byte)(byte)166);
                Debug.Assert(pack.ofs == (uint)1005008954U);
                Debug.Assert(pack.count == (uint)3170078820U);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_component = (byte)(byte)166;
            p119.target_system = (byte)(byte)48;
            p119.count = (uint)3170078820U;
            p119.ofs = (uint)1005008954U;
            p119.id = (ushort)(ushort)31930;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)2149883960U);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)49, (byte)11, (byte)49, (byte)157, (byte)116, (byte)205, (byte)25, (byte)138, (byte)11, (byte)89, (byte)87, (byte)24, (byte)127, (byte)81, (byte)11, (byte)92, (byte)92, (byte)221, (byte)181, (byte)77, (byte)54, (byte)28, (byte)147, (byte)22, (byte)254, (byte)195, (byte)242, (byte)7, (byte)53, (byte)165, (byte)120, (byte)61, (byte)53, (byte)4, (byte)206, (byte)26, (byte)56, (byte)237, (byte)87, (byte)121, (byte)212, (byte)147, (byte)5, (byte)114, (byte)85, (byte)216, (byte)8, (byte)49, (byte)66, (byte)3, (byte)198, (byte)90, (byte)193, (byte)122, (byte)212, (byte)153, (byte)188, (byte)133, (byte)16, (byte)22, (byte)90, (byte)178, (byte)61, (byte)26, (byte)133, (byte)201, (byte)154, (byte)187, (byte)198, (byte)135, (byte)129, (byte)249, (byte)14, (byte)224, (byte)102, (byte)53, (byte)141, (byte)121, (byte)250, (byte)98, (byte)56, (byte)59, (byte)139, (byte)119, (byte)128, (byte)228, (byte)154, (byte)46, (byte)15, (byte)77}));
                Debug.Assert(pack.id == (ushort)(ushort)61353);
                Debug.Assert(pack.count == (byte)(byte)201);
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.data__SET(new byte[] {(byte)49, (byte)11, (byte)49, (byte)157, (byte)116, (byte)205, (byte)25, (byte)138, (byte)11, (byte)89, (byte)87, (byte)24, (byte)127, (byte)81, (byte)11, (byte)92, (byte)92, (byte)221, (byte)181, (byte)77, (byte)54, (byte)28, (byte)147, (byte)22, (byte)254, (byte)195, (byte)242, (byte)7, (byte)53, (byte)165, (byte)120, (byte)61, (byte)53, (byte)4, (byte)206, (byte)26, (byte)56, (byte)237, (byte)87, (byte)121, (byte)212, (byte)147, (byte)5, (byte)114, (byte)85, (byte)216, (byte)8, (byte)49, (byte)66, (byte)3, (byte)198, (byte)90, (byte)193, (byte)122, (byte)212, (byte)153, (byte)188, (byte)133, (byte)16, (byte)22, (byte)90, (byte)178, (byte)61, (byte)26, (byte)133, (byte)201, (byte)154, (byte)187, (byte)198, (byte)135, (byte)129, (byte)249, (byte)14, (byte)224, (byte)102, (byte)53, (byte)141, (byte)121, (byte)250, (byte)98, (byte)56, (byte)59, (byte)139, (byte)119, (byte)128, (byte)228, (byte)154, (byte)46, (byte)15, (byte)77}, 0) ;
            p120.id = (ushort)(ushort)61353;
            p120.count = (byte)(byte)201;
            p120.ofs = (uint)2149883960U;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)231);
                Debug.Assert(pack.target_component == (byte)(byte)19);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)231;
            p121.target_component = (byte)(byte)19;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)187);
                Debug.Assert(pack.target_component == (byte)(byte)35);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_component = (byte)(byte)35;
            p122.target_system = (byte)(byte)187;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)177, (byte)59, (byte)37, (byte)234, (byte)38, (byte)250, (byte)115, (byte)184, (byte)2, (byte)104, (byte)144, (byte)83, (byte)125, (byte)76, (byte)19, (byte)116, (byte)252, (byte)187, (byte)227, (byte)201, (byte)243, (byte)57, (byte)194, (byte)154, (byte)14, (byte)24, (byte)57, (byte)82, (byte)8, (byte)158, (byte)235, (byte)155, (byte)155, (byte)224, (byte)191, (byte)237, (byte)2, (byte)138, (byte)89, (byte)176, (byte)135, (byte)223, (byte)145, (byte)46, (byte)36, (byte)50, (byte)243, (byte)159, (byte)178, (byte)143, (byte)253, (byte)59, (byte)7, (byte)183, (byte)157, (byte)72, (byte)110, (byte)41, (byte)192, (byte)77, (byte)38, (byte)183, (byte)136, (byte)82, (byte)105, (byte)19, (byte)24, (byte)69, (byte)252, (byte)171, (byte)217, (byte)153, (byte)180, (byte)210, (byte)185, (byte)6, (byte)189, (byte)100, (byte)48, (byte)224, (byte)232, (byte)41, (byte)207, (byte)73, (byte)220, (byte)88, (byte)201, (byte)166, (byte)204, (byte)140, (byte)199, (byte)119, (byte)108, (byte)111, (byte)202, (byte)240, (byte)207, (byte)153, (byte)228, (byte)32, (byte)33, (byte)14, (byte)129, (byte)123, (byte)129, (byte)45, (byte)140, (byte)110, (byte)29, (byte)100}));
                Debug.Assert(pack.len == (byte)(byte)51);
                Debug.Assert(pack.target_system == (byte)(byte)115);
                Debug.Assert(pack.target_component == (byte)(byte)11);
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.len = (byte)(byte)51;
            p123.target_component = (byte)(byte)11;
            p123.target_system = (byte)(byte)115;
            p123.data__SET(new byte[] {(byte)177, (byte)59, (byte)37, (byte)234, (byte)38, (byte)250, (byte)115, (byte)184, (byte)2, (byte)104, (byte)144, (byte)83, (byte)125, (byte)76, (byte)19, (byte)116, (byte)252, (byte)187, (byte)227, (byte)201, (byte)243, (byte)57, (byte)194, (byte)154, (byte)14, (byte)24, (byte)57, (byte)82, (byte)8, (byte)158, (byte)235, (byte)155, (byte)155, (byte)224, (byte)191, (byte)237, (byte)2, (byte)138, (byte)89, (byte)176, (byte)135, (byte)223, (byte)145, (byte)46, (byte)36, (byte)50, (byte)243, (byte)159, (byte)178, (byte)143, (byte)253, (byte)59, (byte)7, (byte)183, (byte)157, (byte)72, (byte)110, (byte)41, (byte)192, (byte)77, (byte)38, (byte)183, (byte)136, (byte)82, (byte)105, (byte)19, (byte)24, (byte)69, (byte)252, (byte)171, (byte)217, (byte)153, (byte)180, (byte)210, (byte)185, (byte)6, (byte)189, (byte)100, (byte)48, (byte)224, (byte)232, (byte)41, (byte)207, (byte)73, (byte)220, (byte)88, (byte)201, (byte)166, (byte)204, (byte)140, (byte)199, (byte)119, (byte)108, (byte)111, (byte)202, (byte)240, (byte)207, (byte)153, (byte)228, (byte)32, (byte)33, (byte)14, (byte)129, (byte)123, (byte)129, (byte)45, (byte)140, (byte)110, (byte)29, (byte)100}, 0) ;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.eph == (ushort)(ushort)50212);
                Debug.Assert(pack.satellites_visible == (byte)(byte)77);
                Debug.Assert(pack.cog == (ushort)(ushort)31187);
                Debug.Assert(pack.alt == (int) -1519946993);
                Debug.Assert(pack.lon == (int)1174292466);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
                Debug.Assert(pack.dgps_age == (uint)1313069128U);
                Debug.Assert(pack.dgps_numch == (byte)(byte)136);
                Debug.Assert(pack.vel == (ushort)(ushort)8307);
                Debug.Assert(pack.lat == (int) -1755390574);
                Debug.Assert(pack.epv == (ushort)(ushort)53673);
                Debug.Assert(pack.time_usec == (ulong)4051830991082771428L);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.eph = (ushort)(ushort)50212;
            p124.vel = (ushort)(ushort)8307;
            p124.alt = (int) -1519946993;
            p124.cog = (ushort)(ushort)31187;
            p124.lat = (int) -1755390574;
            p124.satellites_visible = (byte)(byte)77;
            p124.time_usec = (ulong)4051830991082771428L;
            p124.dgps_numch = (byte)(byte)136;
            p124.dgps_age = (uint)1313069128U;
            p124.epv = (ushort)(ushort)53673;
            p124.lon = (int)1174292466;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vservo == (ushort)(ushort)6003);
                Debug.Assert(pack.Vcc == (ushort)(ushort)14057);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT));
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)14057;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT);
            p125.Vservo = (ushort)(ushort)6003;
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)139, (byte)198, (byte)199, (byte)106, (byte)177, (byte)238, (byte)184, (byte)6, (byte)71, (byte)251, (byte)251, (byte)117, (byte)154, (byte)235, (byte)20, (byte)48, (byte)212, (byte)132, (byte)174, (byte)92, (byte)56, (byte)30, (byte)124, (byte)110, (byte)32, (byte)22, (byte)202, (byte)168, (byte)251, (byte)19, (byte)152, (byte)30, (byte)18, (byte)249, (byte)28, (byte)86, (byte)240, (byte)6, (byte)26, (byte)83, (byte)15, (byte)81, (byte)195, (byte)88, (byte)76, (byte)163, (byte)60, (byte)48, (byte)255, (byte)100, (byte)123, (byte)169, (byte)237, (byte)9, (byte)155, (byte)62, (byte)197, (byte)94, (byte)131, (byte)110, (byte)36, (byte)185, (byte)126, (byte)178, (byte)75, (byte)91, (byte)153, (byte)204, (byte)23, (byte)173}));
                Debug.Assert(pack.device == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
                Debug.Assert(pack.count == (byte)(byte)64);
                Debug.Assert(pack.baudrate == (uint)2099914438U);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND));
                Debug.Assert(pack.timeout == (ushort)(ushort)31126);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.baudrate = (uint)2099914438U;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND);
            p126.data__SET(new byte[] {(byte)139, (byte)198, (byte)199, (byte)106, (byte)177, (byte)238, (byte)184, (byte)6, (byte)71, (byte)251, (byte)251, (byte)117, (byte)154, (byte)235, (byte)20, (byte)48, (byte)212, (byte)132, (byte)174, (byte)92, (byte)56, (byte)30, (byte)124, (byte)110, (byte)32, (byte)22, (byte)202, (byte)168, (byte)251, (byte)19, (byte)152, (byte)30, (byte)18, (byte)249, (byte)28, (byte)86, (byte)240, (byte)6, (byte)26, (byte)83, (byte)15, (byte)81, (byte)195, (byte)88, (byte)76, (byte)163, (byte)60, (byte)48, (byte)255, (byte)100, (byte)123, (byte)169, (byte)237, (byte)9, (byte)155, (byte)62, (byte)197, (byte)94, (byte)131, (byte)110, (byte)36, (byte)185, (byte)126, (byte)178, (byte)75, (byte)91, (byte)153, (byte)204, (byte)23, (byte)173}, 0) ;
            p126.timeout = (ushort)(ushort)31126;
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1;
            p126.count = (byte)(byte)64;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_b_mm == (int) -2021126651);
                Debug.Assert(pack.accuracy == (uint)1118814496U);
                Debug.Assert(pack.tow == (uint)1118903762U);
                Debug.Assert(pack.rtk_rate == (byte)(byte)39);
                Debug.Assert(pack.nsats == (byte)(byte)0);
                Debug.Assert(pack.baseline_a_mm == (int) -1575162392);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)13);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)214);
                Debug.Assert(pack.wn == (ushort)(ushort)40209);
                Debug.Assert(pack.iar_num_hypotheses == (int)794030772);
                Debug.Assert(pack.baseline_c_mm == (int)140121335);
                Debug.Assert(pack.time_last_baseline_ms == (uint)2553562096U);
                Debug.Assert(pack.rtk_health == (byte)(byte)248);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.baseline_b_mm = (int) -2021126651;
            p127.baseline_a_mm = (int) -1575162392;
            p127.accuracy = (uint)1118814496U;
            p127.rtk_receiver_id = (byte)(byte)214;
            p127.tow = (uint)1118903762U;
            p127.iar_num_hypotheses = (int)794030772;
            p127.nsats = (byte)(byte)0;
            p127.baseline_coords_type = (byte)(byte)13;
            p127.wn = (ushort)(ushort)40209;
            p127.baseline_c_mm = (int)140121335;
            p127.rtk_rate = (byte)(byte)39;
            p127.rtk_health = (byte)(byte)248;
            p127.time_last_baseline_ms = (uint)2553562096U;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tow == (uint)3164367150U);
                Debug.Assert(pack.rtk_health == (byte)(byte)115);
                Debug.Assert(pack.nsats == (byte)(byte)72);
                Debug.Assert(pack.baseline_a_mm == (int) -21720434);
                Debug.Assert(pack.wn == (ushort)(ushort)52946);
                Debug.Assert(pack.baseline_b_mm == (int)1055950532);
                Debug.Assert(pack.accuracy == (uint)3958975303U);
                Debug.Assert(pack.rtk_rate == (byte)(byte)50);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)138);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)168);
                Debug.Assert(pack.time_last_baseline_ms == (uint)2580601575U);
                Debug.Assert(pack.baseline_c_mm == (int)154401529);
                Debug.Assert(pack.iar_num_hypotheses == (int) -1254544685);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.iar_num_hypotheses = (int) -1254544685;
            p128.baseline_coords_type = (byte)(byte)138;
            p128.time_last_baseline_ms = (uint)2580601575U;
            p128.baseline_a_mm = (int) -21720434;
            p128.baseline_c_mm = (int)154401529;
            p128.rtk_rate = (byte)(byte)50;
            p128.baseline_b_mm = (int)1055950532;
            p128.rtk_health = (byte)(byte)115;
            p128.tow = (uint)3164367150U;
            p128.accuracy = (uint)3958975303U;
            p128.rtk_receiver_id = (byte)(byte)168;
            p128.nsats = (byte)(byte)72;
            p128.wn = (ushort)(ushort)52946;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (short)(short) -7562);
                Debug.Assert(pack.time_boot_ms == (uint)3693128242U);
                Debug.Assert(pack.ygyro == (short)(short) -6896);
                Debug.Assert(pack.xmag == (short)(short)28514);
                Debug.Assert(pack.xgyro == (short)(short)25775);
                Debug.Assert(pack.ymag == (short)(short) -3467);
                Debug.Assert(pack.xacc == (short)(short) -9163);
                Debug.Assert(pack.zacc == (short)(short) -15582);
                Debug.Assert(pack.yacc == (short)(short) -2551);
                Debug.Assert(pack.zmag == (short)(short)3019);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.xmag = (short)(short)28514;
            p129.zacc = (short)(short) -15582;
            p129.xacc = (short)(short) -9163;
            p129.xgyro = (short)(short)25775;
            p129.ygyro = (short)(short) -6896;
            p129.zgyro = (short)(short) -7562;
            p129.ymag = (short)(short) -3467;
            p129.time_boot_ms = (uint)3693128242U;
            p129.zmag = (short)(short)3019;
            p129.yacc = (short)(short) -2551;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.height == (ushort)(ushort)35042);
                Debug.Assert(pack.payload == (byte)(byte)223);
                Debug.Assert(pack.type == (byte)(byte)219);
                Debug.Assert(pack.jpg_quality == (byte)(byte)22);
                Debug.Assert(pack.size == (uint)4004028646U);
                Debug.Assert(pack.packets == (ushort)(ushort)1614);
                Debug.Assert(pack.width == (ushort)(ushort)6829);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.width = (ushort)(ushort)6829;
            p130.size = (uint)4004028646U;
            p130.payload = (byte)(byte)223;
            p130.type = (byte)(byte)219;
            p130.jpg_quality = (byte)(byte)22;
            p130.packets = (ushort)(ushort)1614;
            p130.height = (ushort)(ushort)35042;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)184, (byte)15, (byte)120, (byte)69, (byte)219, (byte)172, (byte)215, (byte)163, (byte)180, (byte)96, (byte)246, (byte)194, (byte)173, (byte)66, (byte)245, (byte)133, (byte)22, (byte)9, (byte)80, (byte)137, (byte)46, (byte)181, (byte)28, (byte)138, (byte)215, (byte)119, (byte)127, (byte)8, (byte)7, (byte)66, (byte)62, (byte)172, (byte)131, (byte)31, (byte)147, (byte)74, (byte)166, (byte)250, (byte)80, (byte)3, (byte)228, (byte)185, (byte)157, (byte)70, (byte)199, (byte)133, (byte)81, (byte)115, (byte)103, (byte)88, (byte)248, (byte)93, (byte)181, (byte)114, (byte)135, (byte)131, (byte)156, (byte)5, (byte)190, (byte)160, (byte)179, (byte)40, (byte)131, (byte)40, (byte)140, (byte)73, (byte)232, (byte)254, (byte)70, (byte)125, (byte)15, (byte)142, (byte)116, (byte)26, (byte)6, (byte)173, (byte)63, (byte)221, (byte)29, (byte)109, (byte)227, (byte)202, (byte)215, (byte)126, (byte)249, (byte)46, (byte)155, (byte)73, (byte)240, (byte)213, (byte)73, (byte)53, (byte)236, (byte)242, (byte)245, (byte)102, (byte)2, (byte)42, (byte)101, (byte)144, (byte)111, (byte)24, (byte)113, (byte)57, (byte)22, (byte)207, (byte)97, (byte)73, (byte)112, (byte)48, (byte)239, (byte)182, (byte)63, (byte)100, (byte)36, (byte)67, (byte)250, (byte)208, (byte)252, (byte)77, (byte)251, (byte)249, (byte)203, (byte)228, (byte)248, (byte)85, (byte)123, (byte)224, (byte)163, (byte)135, (byte)211, (byte)167, (byte)1, (byte)210, (byte)174, (byte)180, (byte)17, (byte)81, (byte)2, (byte)253, (byte)57, (byte)222, (byte)237, (byte)119, (byte)3, (byte)17, (byte)6, (byte)252, (byte)208, (byte)193, (byte)178, (byte)186, (byte)157, (byte)222, (byte)150, (byte)120, (byte)157, (byte)159, (byte)226, (byte)149, (byte)77, (byte)109, (byte)11, (byte)124, (byte)61, (byte)106, (byte)86, (byte)67, (byte)190, (byte)174, (byte)234, (byte)210, (byte)28, (byte)141, (byte)220, (byte)159, (byte)222, (byte)215, (byte)38, (byte)22, (byte)140, (byte)65, (byte)102, (byte)53, (byte)188, (byte)132, (byte)99, (byte)81, (byte)190, (byte)96, (byte)25, (byte)209, (byte)14, (byte)65, (byte)95, (byte)235, (byte)230, (byte)99, (byte)19, (byte)248, (byte)245, (byte)41, (byte)66, (byte)171, (byte)245, (byte)43, (byte)16, (byte)105, (byte)51, (byte)247, (byte)89, (byte)165, (byte)14, (byte)79, (byte)67, (byte)248, (byte)100, (byte)36, (byte)11, (byte)46, (byte)158, (byte)128, (byte)239, (byte)134, (byte)178, (byte)249, (byte)170, (byte)253, (byte)93, (byte)141, (byte)150, (byte)163, (byte)26, (byte)75, (byte)118, (byte)125, (byte)139, (byte)156, (byte)35, (byte)164, (byte)202, (byte)165, (byte)27, (byte)47, (byte)234, (byte)75, (byte)15, (byte)54, (byte)233, (byte)36, (byte)142, (byte)74, (byte)168}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)3864);
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)3864;
            p131.data__SET(new byte[] {(byte)184, (byte)15, (byte)120, (byte)69, (byte)219, (byte)172, (byte)215, (byte)163, (byte)180, (byte)96, (byte)246, (byte)194, (byte)173, (byte)66, (byte)245, (byte)133, (byte)22, (byte)9, (byte)80, (byte)137, (byte)46, (byte)181, (byte)28, (byte)138, (byte)215, (byte)119, (byte)127, (byte)8, (byte)7, (byte)66, (byte)62, (byte)172, (byte)131, (byte)31, (byte)147, (byte)74, (byte)166, (byte)250, (byte)80, (byte)3, (byte)228, (byte)185, (byte)157, (byte)70, (byte)199, (byte)133, (byte)81, (byte)115, (byte)103, (byte)88, (byte)248, (byte)93, (byte)181, (byte)114, (byte)135, (byte)131, (byte)156, (byte)5, (byte)190, (byte)160, (byte)179, (byte)40, (byte)131, (byte)40, (byte)140, (byte)73, (byte)232, (byte)254, (byte)70, (byte)125, (byte)15, (byte)142, (byte)116, (byte)26, (byte)6, (byte)173, (byte)63, (byte)221, (byte)29, (byte)109, (byte)227, (byte)202, (byte)215, (byte)126, (byte)249, (byte)46, (byte)155, (byte)73, (byte)240, (byte)213, (byte)73, (byte)53, (byte)236, (byte)242, (byte)245, (byte)102, (byte)2, (byte)42, (byte)101, (byte)144, (byte)111, (byte)24, (byte)113, (byte)57, (byte)22, (byte)207, (byte)97, (byte)73, (byte)112, (byte)48, (byte)239, (byte)182, (byte)63, (byte)100, (byte)36, (byte)67, (byte)250, (byte)208, (byte)252, (byte)77, (byte)251, (byte)249, (byte)203, (byte)228, (byte)248, (byte)85, (byte)123, (byte)224, (byte)163, (byte)135, (byte)211, (byte)167, (byte)1, (byte)210, (byte)174, (byte)180, (byte)17, (byte)81, (byte)2, (byte)253, (byte)57, (byte)222, (byte)237, (byte)119, (byte)3, (byte)17, (byte)6, (byte)252, (byte)208, (byte)193, (byte)178, (byte)186, (byte)157, (byte)222, (byte)150, (byte)120, (byte)157, (byte)159, (byte)226, (byte)149, (byte)77, (byte)109, (byte)11, (byte)124, (byte)61, (byte)106, (byte)86, (byte)67, (byte)190, (byte)174, (byte)234, (byte)210, (byte)28, (byte)141, (byte)220, (byte)159, (byte)222, (byte)215, (byte)38, (byte)22, (byte)140, (byte)65, (byte)102, (byte)53, (byte)188, (byte)132, (byte)99, (byte)81, (byte)190, (byte)96, (byte)25, (byte)209, (byte)14, (byte)65, (byte)95, (byte)235, (byte)230, (byte)99, (byte)19, (byte)248, (byte)245, (byte)41, (byte)66, (byte)171, (byte)245, (byte)43, (byte)16, (byte)105, (byte)51, (byte)247, (byte)89, (byte)165, (byte)14, (byte)79, (byte)67, (byte)248, (byte)100, (byte)36, (byte)11, (byte)46, (byte)158, (byte)128, (byte)239, (byte)134, (byte)178, (byte)249, (byte)170, (byte)253, (byte)93, (byte)141, (byte)150, (byte)163, (byte)26, (byte)75, (byte)118, (byte)125, (byte)139, (byte)156, (byte)35, (byte)164, (byte)202, (byte)165, (byte)27, (byte)47, (byte)234, (byte)75, (byte)15, (byte)54, (byte)233, (byte)36, (byte)142, (byte)74, (byte)168}, 0) ;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_distance == (ushort)(ushort)39917);
                Debug.Assert(pack.type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
                Debug.Assert(pack.current_distance == (ushort)(ushort)53140);
                Debug.Assert(pack.time_boot_ms == (uint)2108770768U);
                Debug.Assert(pack.orientation == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_45);
                Debug.Assert(pack.covariance == (byte)(byte)38);
                Debug.Assert(pack.id == (byte)(byte)139);
                Debug.Assert(pack.max_distance == (ushort)(ushort)23000);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.time_boot_ms = (uint)2108770768U;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_45;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p132.id = (byte)(byte)139;
            p132.current_distance = (ushort)(ushort)53140;
            p132.max_distance = (ushort)(ushort)23000;
            p132.covariance = (byte)(byte)38;
            p132.min_distance = (ushort)(ushort)39917;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)43955);
                Debug.Assert(pack.mask == (ulong)4103437744012585123L);
                Debug.Assert(pack.lon == (int) -1272668396);
                Debug.Assert(pack.lat == (int)1109986511);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int)1109986511;
            p133.grid_spacing = (ushort)(ushort)43955;
            p133.mask = (ulong)4103437744012585123L;
            p133.lon = (int) -1272668396;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gridbit == (byte)(byte)187);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -14820, (short)4060, (short) -4868, (short)24764, (short)27426, (short) -12702, (short) -26029, (short) -14846, (short)13669, (short) -31882, (short)17270, (short)14482, (short) -9053, (short) -6306, (short) -14888, (short) -27318}));
                Debug.Assert(pack.lon == (int)1820155829);
                Debug.Assert(pack.lat == (int)2004109810);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)34778);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lon = (int)1820155829;
            p134.gridbit = (byte)(byte)187;
            p134.lat = (int)2004109810;
            p134.grid_spacing = (ushort)(ushort)34778;
            p134.data__SET(new short[] {(short) -14820, (short)4060, (short) -4868, (short)24764, (short)27426, (short) -12702, (short) -26029, (short) -14846, (short)13669, (short) -31882, (short)17270, (short)14482, (short) -9053, (short) -6306, (short) -14888, (short) -27318}, 0) ;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -238797023);
                Debug.Assert(pack.lat == (int)976982228);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lon = (int) -238797023;
            p135.lat = (int)976982228;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pending == (ushort)(ushort)8854);
                Debug.Assert(pack.terrain_height == (float)1.9731825E38F);
                Debug.Assert(pack.spacing == (ushort)(ushort)29201);
                Debug.Assert(pack.lat == (int) -1136079927);
                Debug.Assert(pack.loaded == (ushort)(ushort)6411);
                Debug.Assert(pack.current_height == (float)2.4393393E37F);
                Debug.Assert(pack.lon == (int) -306297182);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lon = (int) -306297182;
            p136.current_height = (float)2.4393393E37F;
            p136.pending = (ushort)(ushort)8854;
            p136.terrain_height = (float)1.9731825E38F;
            p136.lat = (int) -1136079927;
            p136.loaded = (ushort)(ushort)6411;
            p136.spacing = (ushort)(ushort)29201;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3010149257U);
                Debug.Assert(pack.press_diff == (float)1.05460474E37F);
                Debug.Assert(pack.press_abs == (float)2.7127577E38F);
                Debug.Assert(pack.temperature == (short)(short) -7061);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.temperature = (short)(short) -7061;
            p137.press_diff = (float)1.05460474E37F;
            p137.press_abs = (float)2.7127577E38F;
            p137.time_boot_ms = (uint)3010149257U;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)1.5140158E38F);
                Debug.Assert(pack.time_usec == (ulong)6242572084195206876L);
                Debug.Assert(pack.x == (float)7.6985947E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {6.1793E36F, -1.0939251E38F, -4.086893E37F, 1.832561E38F}));
                Debug.Assert(pack.y == (float)2.0712076E38F);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.q_SET(new float[] {6.1793E36F, -1.0939251E38F, -4.086893E37F, 1.832561E38F}, 0) ;
            p138.y = (float)2.0712076E38F;
            p138.time_usec = (ulong)6242572084195206876L;
            p138.x = (float)7.6985947E37F;
            p138.z = (float)1.5140158E38F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)133);
                Debug.Assert(pack.group_mlx == (byte)(byte)192);
                Debug.Assert(pack.time_usec == (ulong)3524736263519653361L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {1.4353176E38F, -2.223272E38F, -3.9102108E37F, 1.8876926E38F, 2.4785873E38F, -3.211507E38F, 1.977313E38F, 3.4109202E37F}));
                Debug.Assert(pack.target_component == (byte)(byte)139);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)3524736263519653361L;
            p139.target_system = (byte)(byte)133;
            p139.target_component = (byte)(byte)139;
            p139.controls_SET(new float[] {1.4353176E38F, -2.223272E38F, -3.9102108E37F, 1.8876926E38F, 2.4785873E38F, -3.211507E38F, 1.977313E38F, 3.4109202E37F}, 0) ;
            p139.group_mlx = (byte)(byte)192;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.6878291E38F, -8.827059E37F, -5.905521E37F, -7.9545973E37F, -2.8737274E37F, -1.4503207E38F, 3.1248567E38F, 1.8167881E38F}));
                Debug.Assert(pack.time_usec == (ulong)3276341847507770448L);
                Debug.Assert(pack.group_mlx == (byte)(byte)18);
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.group_mlx = (byte)(byte)18;
            p140.controls_SET(new float[] {-1.6878291E38F, -8.827059E37F, -5.905521E37F, -7.9545973E37F, -2.8737274E37F, -1.4503207E38F, 3.1248567E38F, 1.8167881E38F}, 0) ;
            p140.time_usec = (ulong)3276341847507770448L;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_relative == (float) -2.7039892E38F);
                Debug.Assert(pack.altitude_monotonic == (float) -2.6242832E38F);
                Debug.Assert(pack.bottom_clearance == (float) -3.3663866E37F);
                Debug.Assert(pack.altitude_local == (float)1.1575594E38F);
                Debug.Assert(pack.altitude_amsl == (float)2.3580226E38F);
                Debug.Assert(pack.altitude_terrain == (float) -1.176725E38F);
                Debug.Assert(pack.time_usec == (ulong)2737059829264937335L);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_amsl = (float)2.3580226E38F;
            p141.altitude_local = (float)1.1575594E38F;
            p141.time_usec = (ulong)2737059829264937335L;
            p141.bottom_clearance = (float) -3.3663866E37F;
            p141.altitude_monotonic = (float) -2.6242832E38F;
            p141.altitude_terrain = (float) -1.176725E38F;
            p141.altitude_relative = (float) -2.7039892E38F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.request_id == (byte)(byte)148);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)29, (byte)31, (byte)202, (byte)104, (byte)237, (byte)20, (byte)25, (byte)183, (byte)177, (byte)127, (byte)27, (byte)190, (byte)48, (byte)102, (byte)139, (byte)164, (byte)28, (byte)70, (byte)0, (byte)128, (byte)240, (byte)187, (byte)214, (byte)154, (byte)74, (byte)229, (byte)239, (byte)176, (byte)184, (byte)130, (byte)156, (byte)117, (byte)41, (byte)38, (byte)192, (byte)25, (byte)197, (byte)13, (byte)43, (byte)56, (byte)156, (byte)255, (byte)152, (byte)152, (byte)248, (byte)179, (byte)203, (byte)234, (byte)3, (byte)135, (byte)64, (byte)228, (byte)114, (byte)240, (byte)67, (byte)104, (byte)64, (byte)97, (byte)183, (byte)15, (byte)153, (byte)24, (byte)75, (byte)61, (byte)246, (byte)38, (byte)146, (byte)210, (byte)162, (byte)88, (byte)24, (byte)182, (byte)6, (byte)127, (byte)91, (byte)203, (byte)104, (byte)179, (byte)156, (byte)250, (byte)108, (byte)26, (byte)240, (byte)41, (byte)237, (byte)74, (byte)17, (byte)179, (byte)148, (byte)117, (byte)210, (byte)101, (byte)142, (byte)79, (byte)210, (byte)96, (byte)73, (byte)86, (byte)99, (byte)80, (byte)217, (byte)186, (byte)170, (byte)126, (byte)26, (byte)103, (byte)167, (byte)107, (byte)223, (byte)249, (byte)30, (byte)133, (byte)124, (byte)185, (byte)140, (byte)199, (byte)203, (byte)57, (byte)92, (byte)201}));
                Debug.Assert(pack.uri_type == (byte)(byte)223);
                Debug.Assert(pack.transfer_type == (byte)(byte)238);
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)102, (byte)10, (byte)179, (byte)184, (byte)179, (byte)231, (byte)79, (byte)197, (byte)184, (byte)162, (byte)50, (byte)58, (byte)118, (byte)114, (byte)173, (byte)230, (byte)225, (byte)0, (byte)71, (byte)39, (byte)167, (byte)4, (byte)246, (byte)86, (byte)113, (byte)148, (byte)208, (byte)165, (byte)136, (byte)202, (byte)75, (byte)137, (byte)26, (byte)190, (byte)76, (byte)133, (byte)65, (byte)127, (byte)65, (byte)201, (byte)71, (byte)179, (byte)116, (byte)112, (byte)204, (byte)181, (byte)199, (byte)2, (byte)10, (byte)119, (byte)121, (byte)10, (byte)212, (byte)164, (byte)161, (byte)17, (byte)72, (byte)163, (byte)155, (byte)94, (byte)36, (byte)75, (byte)139, (byte)191, (byte)176, (byte)231, (byte)64, (byte)29, (byte)196, (byte)215, (byte)126, (byte)103, (byte)23, (byte)121, (byte)214, (byte)38, (byte)139, (byte)6, (byte)36, (byte)168, (byte)151, (byte)168, (byte)97, (byte)253, (byte)218, (byte)93, (byte)162, (byte)124, (byte)22, (byte)161, (byte)112, (byte)246, (byte)61, (byte)209, (byte)253, (byte)6, (byte)121, (byte)157, (byte)67, (byte)116, (byte)181, (byte)57, (byte)15, (byte)188, (byte)234, (byte)213, (byte)192, (byte)251, (byte)217, (byte)99, (byte)212, (byte)155, (byte)131, (byte)50, (byte)91, (byte)101, (byte)36, (byte)169, (byte)53, (byte)95}));
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.transfer_type = (byte)(byte)238;
            p142.request_id = (byte)(byte)148;
            p142.uri_SET(new byte[] {(byte)29, (byte)31, (byte)202, (byte)104, (byte)237, (byte)20, (byte)25, (byte)183, (byte)177, (byte)127, (byte)27, (byte)190, (byte)48, (byte)102, (byte)139, (byte)164, (byte)28, (byte)70, (byte)0, (byte)128, (byte)240, (byte)187, (byte)214, (byte)154, (byte)74, (byte)229, (byte)239, (byte)176, (byte)184, (byte)130, (byte)156, (byte)117, (byte)41, (byte)38, (byte)192, (byte)25, (byte)197, (byte)13, (byte)43, (byte)56, (byte)156, (byte)255, (byte)152, (byte)152, (byte)248, (byte)179, (byte)203, (byte)234, (byte)3, (byte)135, (byte)64, (byte)228, (byte)114, (byte)240, (byte)67, (byte)104, (byte)64, (byte)97, (byte)183, (byte)15, (byte)153, (byte)24, (byte)75, (byte)61, (byte)246, (byte)38, (byte)146, (byte)210, (byte)162, (byte)88, (byte)24, (byte)182, (byte)6, (byte)127, (byte)91, (byte)203, (byte)104, (byte)179, (byte)156, (byte)250, (byte)108, (byte)26, (byte)240, (byte)41, (byte)237, (byte)74, (byte)17, (byte)179, (byte)148, (byte)117, (byte)210, (byte)101, (byte)142, (byte)79, (byte)210, (byte)96, (byte)73, (byte)86, (byte)99, (byte)80, (byte)217, (byte)186, (byte)170, (byte)126, (byte)26, (byte)103, (byte)167, (byte)107, (byte)223, (byte)249, (byte)30, (byte)133, (byte)124, (byte)185, (byte)140, (byte)199, (byte)203, (byte)57, (byte)92, (byte)201}, 0) ;
            p142.storage_SET(new byte[] {(byte)102, (byte)10, (byte)179, (byte)184, (byte)179, (byte)231, (byte)79, (byte)197, (byte)184, (byte)162, (byte)50, (byte)58, (byte)118, (byte)114, (byte)173, (byte)230, (byte)225, (byte)0, (byte)71, (byte)39, (byte)167, (byte)4, (byte)246, (byte)86, (byte)113, (byte)148, (byte)208, (byte)165, (byte)136, (byte)202, (byte)75, (byte)137, (byte)26, (byte)190, (byte)76, (byte)133, (byte)65, (byte)127, (byte)65, (byte)201, (byte)71, (byte)179, (byte)116, (byte)112, (byte)204, (byte)181, (byte)199, (byte)2, (byte)10, (byte)119, (byte)121, (byte)10, (byte)212, (byte)164, (byte)161, (byte)17, (byte)72, (byte)163, (byte)155, (byte)94, (byte)36, (byte)75, (byte)139, (byte)191, (byte)176, (byte)231, (byte)64, (byte)29, (byte)196, (byte)215, (byte)126, (byte)103, (byte)23, (byte)121, (byte)214, (byte)38, (byte)139, (byte)6, (byte)36, (byte)168, (byte)151, (byte)168, (byte)97, (byte)253, (byte)218, (byte)93, (byte)162, (byte)124, (byte)22, (byte)161, (byte)112, (byte)246, (byte)61, (byte)209, (byte)253, (byte)6, (byte)121, (byte)157, (byte)67, (byte)116, (byte)181, (byte)57, (byte)15, (byte)188, (byte)234, (byte)213, (byte)192, (byte)251, (byte)217, (byte)99, (byte)212, (byte)155, (byte)131, (byte)50, (byte)91, (byte)101, (byte)36, (byte)169, (byte)53, (byte)95}, 0) ;
            p142.uri_type = (byte)(byte)223;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1818755479U);
                Debug.Assert(pack.temperature == (short)(short)17840);
                Debug.Assert(pack.press_diff == (float)1.4658865E38F);
                Debug.Assert(pack.press_abs == (float) -3.3085178E38F);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)1818755479U;
            p143.temperature = (short)(short)17840;
            p143.press_abs = (float) -3.3085178E38F;
            p143.press_diff = (float)1.4658865E38F;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-5.634434E36F, 5.14012E37F, 3.336122E38F, 1.4325252E38F}));
                Debug.Assert(pack.timestamp == (ulong)6247355304316009520L);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {3.6980007E37F, 2.3236977E38F, -1.9100645E38F}));
                Debug.Assert(pack.vel.SequenceEqual(new float[] {1.924134E38F, -7.178127E37F, -5.9593827E37F}));
                Debug.Assert(pack.lon == (int)1258729695);
                Debug.Assert(pack.lat == (int)505068879);
                Debug.Assert(pack.est_capabilities == (byte)(byte)75);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {1.9768983E38F, 3.1382082E38F, -2.507563E38F}));
                Debug.Assert(pack.custom_state == (ulong)517558073389455725L);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {5.7824156E37F, 1.035517E38F, 2.7189394E38F}));
                Debug.Assert(pack.alt == (float)1.7558249E38F);
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.timestamp = (ulong)6247355304316009520L;
            p144.acc_SET(new float[] {5.7824156E37F, 1.035517E38F, 2.7189394E38F}, 0) ;
            p144.rates_SET(new float[] {3.6980007E37F, 2.3236977E38F, -1.9100645E38F}, 0) ;
            p144.lat = (int)505068879;
            p144.vel_SET(new float[] {1.924134E38F, -7.178127E37F, -5.9593827E37F}, 0) ;
            p144.lon = (int)1258729695;
            p144.alt = (float)1.7558249E38F;
            p144.est_capabilities = (byte)(byte)75;
            p144.attitude_q_SET(new float[] {-5.634434E36F, 5.14012E37F, 3.336122E38F, 1.4325252E38F}, 0) ;
            p144.custom_state = (ulong)517558073389455725L;
            p144.position_cov_SET(new float[] {1.9768983E38F, 3.1382082E38F, -2.507563E38F}, 0) ;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {-2.2415275E38F, 1.6010186E38F, -2.6909448E38F}));
                Debug.Assert(pack.y_acc == (float)2.944387E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {4.0348604E37F, 1.4018833E38F, 3.3108446E38F}));
                Debug.Assert(pack.pitch_rate == (float)5.340555E36F);
                Debug.Assert(pack.y_pos == (float) -1.7312582E38F);
                Debug.Assert(pack.x_vel == (float)2.7482734E38F);
                Debug.Assert(pack.time_usec == (ulong)2235394099247187017L);
                Debug.Assert(pack.x_pos == (float)1.1520695E38F);
                Debug.Assert(pack.yaw_rate == (float) -5.258972E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.1570035E38F, 1.3102574E37F, 1.2883991E38F, 3.6668523E37F}));
                Debug.Assert(pack.y_vel == (float) -3.8285938E37F);
                Debug.Assert(pack.airspeed == (float)2.3466334E38F);
                Debug.Assert(pack.x_acc == (float) -2.9452286E38F);
                Debug.Assert(pack.roll_rate == (float) -3.3740364E38F);
                Debug.Assert(pack.z_pos == (float)5.938286E37F);
                Debug.Assert(pack.z_acc == (float)3.2915049E38F);
                Debug.Assert(pack.z_vel == (float)2.1996829E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.q_SET(new float[] {-3.1570035E38F, 1.3102574E37F, 1.2883991E38F, 3.6668523E37F}, 0) ;
            p146.z_vel = (float)2.1996829E38F;
            p146.airspeed = (float)2.3466334E38F;
            p146.x_pos = (float)1.1520695E38F;
            p146.z_acc = (float)3.2915049E38F;
            p146.z_pos = (float)5.938286E37F;
            p146.pitch_rate = (float)5.340555E36F;
            p146.y_vel = (float) -3.8285938E37F;
            p146.x_acc = (float) -2.9452286E38F;
            p146.pos_variance_SET(new float[] {4.0348604E37F, 1.4018833E38F, 3.3108446E38F}, 0) ;
            p146.yaw_rate = (float) -5.258972E37F;
            p146.time_usec = (ulong)2235394099247187017L;
            p146.x_vel = (float)2.7482734E38F;
            p146.y_pos = (float) -1.7312582E38F;
            p146.y_acc = (float)2.944387E38F;
            p146.roll_rate = (float) -3.3740364E38F;
            p146.vel_variance_SET(new float[] {-2.2415275E38F, 1.6010186E38F, -2.6909448E38F}, 0) ;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)28301);
                Debug.Assert(pack.energy_consumed == (int)1823834736);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 40);
                Debug.Assert(pack.type == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)48112, (ushort)34620, (ushort)55869, (ushort)58083, (ushort)14780, (ushort)7664, (ushort)24975, (ushort)49440, (ushort)40744, (ushort)49664}));
                Debug.Assert(pack.current_consumed == (int) -759525276);
                Debug.Assert(pack.battery_function == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION);
                Debug.Assert(pack.id == (byte)(byte)118);
                Debug.Assert(pack.current_battery == (short)(short) -10367);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)118;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE;
            p147.battery_remaining = (sbyte)(sbyte) - 40;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION;
            p147.current_battery = (short)(short) -10367;
            p147.voltages_SET(new ushort[] {(ushort)48112, (ushort)34620, (ushort)55869, (ushort)58083, (ushort)14780, (ushort)7664, (ushort)24975, (ushort)49440, (ushort)40744, (ushort)49664}, 0) ;
            p147.energy_consumed = (int)1823834736;
            p147.temperature = (short)(short)28301;
            p147.current_consumed = (int) -759525276;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)40, (byte)76, (byte)112, (byte)92, (byte)248, (byte)48, (byte)145, (byte)158}));
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)178, (byte)13, (byte)56, (byte)223, (byte)75, (byte)181, (byte)192, (byte)10}));
                Debug.Assert(pack.os_sw_version == (uint)2714811121U);
                Debug.Assert(pack.product_id == (ushort)(ushort)53962);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE));
                Debug.Assert(pack.uid == (ulong)8666015712667331687L);
                Debug.Assert(pack.flight_sw_version == (uint)3608290135U);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)27633);
                Debug.Assert(pack.middleware_sw_version == (uint)2614170993U);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)140, (byte)31, (byte)74, (byte)52, (byte)122, (byte)106, (byte)63, (byte)60, (byte)15, (byte)115, (byte)6, (byte)192, (byte)14, (byte)134, (byte)42, (byte)75, (byte)34, (byte)27}));
                Debug.Assert(pack.board_version == (uint)1813772739U);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)224, (byte)115, (byte)108, (byte)187, (byte)177, (byte)16, (byte)77, (byte)192}));
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.flight_sw_version = (uint)3608290135U;
            p148.board_version = (uint)1813772739U;
            p148.product_id = (ushort)(ushort)53962;
            p148.uid = (ulong)8666015712667331687L;
            p148.uid2_SET(new byte[] {(byte)140, (byte)31, (byte)74, (byte)52, (byte)122, (byte)106, (byte)63, (byte)60, (byte)15, (byte)115, (byte)6, (byte)192, (byte)14, (byte)134, (byte)42, (byte)75, (byte)34, (byte)27}, 0, PH) ;
            p148.flight_custom_version_SET(new byte[] {(byte)40, (byte)76, (byte)112, (byte)92, (byte)248, (byte)48, (byte)145, (byte)158}, 0) ;
            p148.os_sw_version = (uint)2714811121U;
            p148.middleware_custom_version_SET(new byte[] {(byte)224, (byte)115, (byte)108, (byte)187, (byte)177, (byte)16, (byte)77, (byte)192}, 0) ;
            p148.vendor_id = (ushort)(ushort)27633;
            p148.middleware_sw_version = (uint)2614170993U;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE);
            p148.os_custom_version_SET(new byte[] {(byte)178, (byte)13, (byte)56, (byte)223, (byte)75, (byte)181, (byte)192, (byte)10}, 0) ;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)126);
                Debug.Assert(pack.y_TRY(ph) == (float) -2.621107E38F);
                Debug.Assert(pack.z_TRY(ph) == (float)2.875583E38F);
                Debug.Assert(pack.distance == (float)8.775462E37F);
                Debug.Assert(pack.target_num == (byte)(byte)16);
                Debug.Assert(pack.type == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
                Debug.Assert(pack.size_y == (float)2.2899995E38F);
                Debug.Assert(pack.x_TRY(ph) == (float)3.320516E38F);
                Debug.Assert(pack.size_x == (float)1.5589214E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.angle_y == (float) -3.2004028E38F);
                Debug.Assert(pack.time_usec == (ulong)1008303957015725311L);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {-4.6486477E37F, 2.0702043E38F, -3.2837234E37F, 7.58602E36F}));
                Debug.Assert(pack.angle_x == (float)9.445518E37F);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.size_x = (float)1.5589214E38F;
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON;
            p149.y_SET((float) -2.621107E38F, PH) ;
            p149.angle_x = (float)9.445518E37F;
            p149.size_y = (float)2.2899995E38F;
            p149.position_valid_SET((byte)(byte)126, PH) ;
            p149.x_SET((float)3.320516E38F, PH) ;
            p149.time_usec = (ulong)1008303957015725311L;
            p149.angle_y = (float) -3.2004028E38F;
            p149.frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p149.distance = (float)8.775462E37F;
            p149.q_SET(new float[] {-4.6486477E37F, 2.0702043E38F, -3.2837234E37F, 7.58602E36F}, 0, PH) ;
            p149.target_num = (byte)(byte)16;
            p149.z_SET((float)2.875583E38F, PH) ;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCRIPT_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)47433);
                Debug.Assert(pack.target_component == (byte)(byte)115);
                Debug.Assert(pack.target_system == (byte)(byte)121);
                Debug.Assert(pack.name_LEN(ph) == 33);
                Debug.Assert(pack.name_TRY(ph).Equals("izqbynxszPbxyejvmbvtdbecivwdtmemj"));
            };
            GroundControl.SCRIPT_ITEM p180 = CommunicationChannel.new_SCRIPT_ITEM();
            PH.setPack(p180);
            p180.target_system = (byte)(byte)121;
            p180.target_component = (byte)(byte)115;
            p180.seq = (ushort)(ushort)47433;
            p180.name_SET("izqbynxszPbxyejvmbvtdbecivwdtmemj", PH) ;
            CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCRIPT_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)229);
                Debug.Assert(pack.target_component == (byte)(byte)131);
                Debug.Assert(pack.seq == (ushort)(ushort)52066);
            };
            GroundControl.SCRIPT_REQUEST p181 = CommunicationChannel.new_SCRIPT_REQUEST();
            PH.setPack(p181);
            p181.seq = (ushort)(ushort)52066;
            p181.target_component = (byte)(byte)131;
            p181.target_system = (byte)(byte)229;
            CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCRIPT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)112);
                Debug.Assert(pack.target_system == (byte)(byte)18);
            };
            GroundControl.SCRIPT_REQUEST_LIST p182 = CommunicationChannel.new_SCRIPT_REQUEST_LIST();
            PH.setPack(p182);
            p182.target_system = (byte)(byte)18;
            p182.target_component = (byte)(byte)112;
            CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCRIPT_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (ushort)(ushort)46676);
                Debug.Assert(pack.target_system == (byte)(byte)194);
                Debug.Assert(pack.target_component == (byte)(byte)9);
            };
            GroundControl.SCRIPT_COUNT p183 = CommunicationChannel.new_SCRIPT_COUNT();
            PH.setPack(p183);
            p183.target_component = (byte)(byte)9;
            p183.target_system = (byte)(byte)194;
            p183.count = (ushort)(ushort)46676;
            CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCRIPT_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)4922);
            };
            GroundControl.SCRIPT_CURRENT p184 = CommunicationChannel.new_SCRIPT_CURRENT();
            PH.setPack(p184);
            p184.seq = (ushort)(ushort)4922;
            CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel_ratio == (float)3.251025E38F);
                Debug.Assert(pack.pos_vert_ratio == (float) -1.7514394E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float) -1.3942636E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH));
                Debug.Assert(pack.tas_ratio == (float)2.8982357E38F);
                Debug.Assert(pack.time_usec == (ulong)330580798803688282L);
                Debug.Assert(pack.hagl_ratio == (float)1.161796E38F);
                Debug.Assert(pack.mag_ratio == (float)2.9124725E38F);
                Debug.Assert(pack.pos_horiz_ratio == (float)7.657138E37F);
                Debug.Assert(pack.pos_vert_accuracy == (float) -5.7338585E37F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.vel_ratio = (float)3.251025E38F;
            p230.pos_vert_ratio = (float) -1.7514394E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_REL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH);
            p230.tas_ratio = (float)2.8982357E38F;
            p230.pos_vert_accuracy = (float) -5.7338585E37F;
            p230.pos_horiz_ratio = (float)7.657138E37F;
            p230.mag_ratio = (float)2.9124725E38F;
            p230.hagl_ratio = (float)1.161796E38F;
            p230.pos_horiz_accuracy = (float) -1.3942636E38F;
            p230.time_usec = (ulong)330580798803688282L;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.var_horiz == (float)6.867117E37F);
                Debug.Assert(pack.var_vert == (float)2.3285496E38F);
                Debug.Assert(pack.wind_z == (float) -2.4144288E38F);
                Debug.Assert(pack.horiz_accuracy == (float)9.582476E37F);
                Debug.Assert(pack.time_usec == (ulong)8818468897756950967L);
                Debug.Assert(pack.wind_y == (float) -2.6662152E38F);
                Debug.Assert(pack.vert_accuracy == (float)2.100943E38F);
                Debug.Assert(pack.wind_x == (float)3.5806875E37F);
                Debug.Assert(pack.wind_alt == (float) -2.2525849E38F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.wind_alt = (float) -2.2525849E38F;
            p231.vert_accuracy = (float)2.100943E38F;
            p231.wind_z = (float) -2.4144288E38F;
            p231.time_usec = (ulong)8818468897756950967L;
            p231.horiz_accuracy = (float)9.582476E37F;
            p231.wind_y = (float) -2.6662152E38F;
            p231.var_horiz = (float)6.867117E37F;
            p231.wind_x = (float)3.5806875E37F;
            p231.var_vert = (float)2.3285496E38F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.horiz_accuracy == (float) -2.9321233E38F);
                Debug.Assert(pack.time_usec == (ulong)1950775329717644237L);
                Debug.Assert(pack.vn == (float) -2.6244846E38F);
                Debug.Assert(pack.vd == (float)9.3991545E36F);
                Debug.Assert(pack.hdop == (float)1.7907922E38F);
                Debug.Assert(pack.vert_accuracy == (float)1.0360167E38F);
                Debug.Assert(pack.gps_id == (byte)(byte)69);
                Debug.Assert(pack.time_week == (ushort)(ushort)31627);
                Debug.Assert(pack.speed_accuracy == (float)3.0914702E38F);
                Debug.Assert(pack.alt == (float) -1.2393371E37F);
                Debug.Assert(pack.ve == (float) -2.3654883E38F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY));
                Debug.Assert(pack.lon == (int) -1891862114);
                Debug.Assert(pack.vdop == (float)1.5740986E38F);
                Debug.Assert(pack.time_week_ms == (uint)3712010578U);
                Debug.Assert(pack.fix_type == (byte)(byte)180);
                Debug.Assert(pack.lat == (int) -1591557108);
                Debug.Assert(pack.satellites_visible == (byte)(byte)66);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.vdop = (float)1.5740986E38F;
            p232.vert_accuracy = (float)1.0360167E38F;
            p232.gps_id = (byte)(byte)69;
            p232.alt = (float) -1.2393371E37F;
            p232.vn = (float) -2.6244846E38F;
            p232.speed_accuracy = (float)3.0914702E38F;
            p232.fix_type = (byte)(byte)180;
            p232.lon = (int) -1891862114;
            p232.vd = (float)9.3991545E36F;
            p232.ve = (float) -2.3654883E38F;
            p232.lat = (int) -1591557108;
            p232.time_week = (ushort)(ushort)31627;
            p232.hdop = (float)1.7907922E38F;
            p232.time_week_ms = (uint)3712010578U;
            p232.horiz_accuracy = (float) -2.9321233E38F;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY);
            p232.time_usec = (ulong)1950775329717644237L;
            p232.satellites_visible = (byte)(byte)66;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)21, (byte)67, (byte)161, (byte)123, (byte)36, (byte)190, (byte)108, (byte)27, (byte)231, (byte)0, (byte)206, (byte)95, (byte)21, (byte)116, (byte)142, (byte)85, (byte)252, (byte)27, (byte)177, (byte)144, (byte)49, (byte)137, (byte)237, (byte)206, (byte)71, (byte)174, (byte)214, (byte)148, (byte)94, (byte)177, (byte)180, (byte)141, (byte)103, (byte)120, (byte)35, (byte)134, (byte)18, (byte)92, (byte)186, (byte)195, (byte)37, (byte)88, (byte)42, (byte)121, (byte)81, (byte)29, (byte)37, (byte)47, (byte)79, (byte)192, (byte)122, (byte)11, (byte)107, (byte)255, (byte)100, (byte)20, (byte)41, (byte)29, (byte)230, (byte)216, (byte)39, (byte)25, (byte)6, (byte)197, (byte)121, (byte)212, (byte)192, (byte)246, (byte)132, (byte)80, (byte)104, (byte)225, (byte)117, (byte)38, (byte)77, (byte)69, (byte)181, (byte)7, (byte)103, (byte)50, (byte)44, (byte)212, (byte)246, (byte)110, (byte)104, (byte)161, (byte)248, (byte)106, (byte)55, (byte)254, (byte)228, (byte)88, (byte)156, (byte)4, (byte)214, (byte)177, (byte)92, (byte)6, (byte)235, (byte)24, (byte)152, (byte)52, (byte)197, (byte)75, (byte)124, (byte)146, (byte)2, (byte)125, (byte)1, (byte)34, (byte)49, (byte)21, (byte)127, (byte)199, (byte)152, (byte)66, (byte)197, (byte)59, (byte)57, (byte)117, (byte)188, (byte)72, (byte)37, (byte)161, (byte)135, (byte)204, (byte)201, (byte)39, (byte)151, (byte)152, (byte)193, (byte)96, (byte)32, (byte)151, (byte)53, (byte)146, (byte)66, (byte)145, (byte)55, (byte)238, (byte)115, (byte)142, (byte)98, (byte)176, (byte)84, (byte)233, (byte)63, (byte)86, (byte)36, (byte)222, (byte)223, (byte)232, (byte)42, (byte)132, (byte)202, (byte)83, (byte)151, (byte)243, (byte)240, (byte)166, (byte)22, (byte)213, (byte)42, (byte)21, (byte)63, (byte)5, (byte)37, (byte)183, (byte)15, (byte)153, (byte)21, (byte)40, (byte)68, (byte)255, (byte)184, (byte)184, (byte)155, (byte)157, (byte)175, (byte)113}));
                Debug.Assert(pack.len == (byte)(byte)240);
                Debug.Assert(pack.flags == (byte)(byte)232);
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)232;
            p233.data__SET(new byte[] {(byte)21, (byte)67, (byte)161, (byte)123, (byte)36, (byte)190, (byte)108, (byte)27, (byte)231, (byte)0, (byte)206, (byte)95, (byte)21, (byte)116, (byte)142, (byte)85, (byte)252, (byte)27, (byte)177, (byte)144, (byte)49, (byte)137, (byte)237, (byte)206, (byte)71, (byte)174, (byte)214, (byte)148, (byte)94, (byte)177, (byte)180, (byte)141, (byte)103, (byte)120, (byte)35, (byte)134, (byte)18, (byte)92, (byte)186, (byte)195, (byte)37, (byte)88, (byte)42, (byte)121, (byte)81, (byte)29, (byte)37, (byte)47, (byte)79, (byte)192, (byte)122, (byte)11, (byte)107, (byte)255, (byte)100, (byte)20, (byte)41, (byte)29, (byte)230, (byte)216, (byte)39, (byte)25, (byte)6, (byte)197, (byte)121, (byte)212, (byte)192, (byte)246, (byte)132, (byte)80, (byte)104, (byte)225, (byte)117, (byte)38, (byte)77, (byte)69, (byte)181, (byte)7, (byte)103, (byte)50, (byte)44, (byte)212, (byte)246, (byte)110, (byte)104, (byte)161, (byte)248, (byte)106, (byte)55, (byte)254, (byte)228, (byte)88, (byte)156, (byte)4, (byte)214, (byte)177, (byte)92, (byte)6, (byte)235, (byte)24, (byte)152, (byte)52, (byte)197, (byte)75, (byte)124, (byte)146, (byte)2, (byte)125, (byte)1, (byte)34, (byte)49, (byte)21, (byte)127, (byte)199, (byte)152, (byte)66, (byte)197, (byte)59, (byte)57, (byte)117, (byte)188, (byte)72, (byte)37, (byte)161, (byte)135, (byte)204, (byte)201, (byte)39, (byte)151, (byte)152, (byte)193, (byte)96, (byte)32, (byte)151, (byte)53, (byte)146, (byte)66, (byte)145, (byte)55, (byte)238, (byte)115, (byte)142, (byte)98, (byte)176, (byte)84, (byte)233, (byte)63, (byte)86, (byte)36, (byte)222, (byte)223, (byte)232, (byte)42, (byte)132, (byte)202, (byte)83, (byte)151, (byte)243, (byte)240, (byte)166, (byte)22, (byte)213, (byte)42, (byte)21, (byte)63, (byte)5, (byte)37, (byte)183, (byte)15, (byte)153, (byte)21, (byte)40, (byte)68, (byte)255, (byte)184, (byte)184, (byte)155, (byte)157, (byte)175, (byte)113}, 0) ;
            p233.len = (byte)(byte)240;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.custom_mode == (uint)165025789U);
                Debug.Assert(pack.temperature == (sbyte)(sbyte) - 27);
                Debug.Assert(pack.pitch == (short)(short)1941);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)61);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)124);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte)63);
                Debug.Assert(pack.airspeed == (byte)(byte)78);
                Debug.Assert(pack.altitude_amsl == (short)(short)508);
                Debug.Assert(pack.battery_remaining == (byte)(byte)120);
                Debug.Assert(pack.failsafe == (byte)(byte)34);
                Debug.Assert(pack.wp_num == (byte)(byte)78);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)122);
                Debug.Assert(pack.heading == (ushort)(ushort)20059);
                Debug.Assert(pack.gps_nsat == (byte)(byte)134);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)61595);
                Debug.Assert(pack.roll == (short)(short) -825);
                Debug.Assert(pack.longitude == (int) -361635505);
                Debug.Assert(pack.altitude_sp == (short)(short)31898);
                Debug.Assert(pack.gps_fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
                Debug.Assert(pack.groundspeed == (byte)(byte)144);
                Debug.Assert(pack.heading_sp == (short)(short)6876);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED));
                Debug.Assert(pack.latitude == (int) -396523839);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.gps_nsat = (byte)(byte)134;
            p234.heading = (ushort)(ushort)20059;
            p234.custom_mode = (uint)165025789U;
            p234.roll = (short)(short) -825;
            p234.throttle = (sbyte)(sbyte)122;
            p234.airspeed = (byte)(byte)78;
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
            p234.climb_rate = (sbyte)(sbyte)63;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            p234.groundspeed = (byte)(byte)144;
            p234.battery_remaining = (byte)(byte)120;
            p234.altitude_sp = (short)(short)31898;
            p234.temperature_air = (sbyte)(sbyte)124;
            p234.wp_num = (byte)(byte)78;
            p234.heading_sp = (short)(short)6876;
            p234.airspeed_sp = (byte)(byte)61;
            p234.altitude_amsl = (short)(short)508;
            p234.pitch = (short)(short)1941;
            p234.failsafe = (byte)(byte)34;
            p234.longitude = (int) -361635505;
            p234.temperature = (sbyte)(sbyte) - 27;
            p234.wp_distance = (ushort)(ushort)61595;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS;
            p234.latitude = (int) -396523839;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)9131704950309145217L);
                Debug.Assert(pack.vibration_x == (float) -2.2898E36F);
                Debug.Assert(pack.clipping_1 == (uint)2217230186U);
                Debug.Assert(pack.vibration_z == (float) -2.3823777E38F);
                Debug.Assert(pack.clipping_0 == (uint)2808278325U);
                Debug.Assert(pack.clipping_2 == (uint)2609781375U);
                Debug.Assert(pack.vibration_y == (float) -2.1014569E37F);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.vibration_z = (float) -2.3823777E38F;
            p241.clipping_2 = (uint)2609781375U;
            p241.clipping_1 = (uint)2217230186U;
            p241.vibration_x = (float) -2.2898E36F;
            p241.time_usec = (ulong)9131704950309145217L;
            p241.vibration_y = (float) -2.1014569E37F;
            p241.clipping_0 = (uint)2808278325U;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7238566152702933610L);
                Debug.Assert(pack.longitude == (int)748015622);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-5.800739E37F, 1.6072012E38F, -4.0864083E36F, -1.1645884E38F}));
                Debug.Assert(pack.approach_x == (float) -6.6281814E37F);
                Debug.Assert(pack.approach_y == (float)2.0505757E38F);
                Debug.Assert(pack.z == (float) -1.9157975E38F);
                Debug.Assert(pack.y == (float)6.301447E37F);
                Debug.Assert(pack.altitude == (int)366719262);
                Debug.Assert(pack.x == (float)9.804761E37F);
                Debug.Assert(pack.latitude == (int) -1303916376);
                Debug.Assert(pack.approach_z == (float) -6.6946793E37F);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.y = (float)6.301447E37F;
            p242.x = (float)9.804761E37F;
            p242.approach_z = (float) -6.6946793E37F;
            p242.longitude = (int)748015622;
            p242.q_SET(new float[] {-5.800739E37F, 1.6072012E38F, -4.0864083E36F, -1.1645884E38F}, 0) ;
            p242.approach_y = (float)2.0505757E38F;
            p242.altitude = (int)366719262;
            p242.approach_x = (float) -6.6281814E37F;
            p242.latitude = (int) -1303916376;
            p242.z = (float) -1.9157975E38F;
            p242.time_usec_SET((ulong)7238566152702933610L, PH) ;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -2.3024458E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.273765E38F, -3.8876154E37F, -1.0935228E38F, 5.4326835E37F}));
                Debug.Assert(pack.approach_y == (float)2.6512742E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)6236631244181950487L);
                Debug.Assert(pack.z == (float) -3.1997467E38F);
                Debug.Assert(pack.longitude == (int) -2057699920);
                Debug.Assert(pack.x == (float) -7.041437E37F);
                Debug.Assert(pack.latitude == (int)631414176);
                Debug.Assert(pack.altitude == (int)487376851);
                Debug.Assert(pack.approach_x == (float)1.6977859E38F);
                Debug.Assert(pack.target_system == (byte)(byte)30);
                Debug.Assert(pack.approach_z == (float)8.017851E37F);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.altitude = (int)487376851;
            p243.z = (float) -3.1997467E38F;
            p243.approach_z = (float)8.017851E37F;
            p243.time_usec_SET((ulong)6236631244181950487L, PH) ;
            p243.q_SET(new float[] {-1.273765E38F, -3.8876154E37F, -1.0935228E38F, 5.4326835E37F}, 0) ;
            p243.y = (float) -2.3024458E38F;
            p243.target_system = (byte)(byte)30;
            p243.longitude = (int) -2057699920;
            p243.x = (float) -7.041437E37F;
            p243.approach_x = (float)1.6977859E38F;
            p243.approach_y = (float)2.6512742E38F;
            p243.latitude = (int)631414176;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)44409);
                Debug.Assert(pack.interval_us == (int)828908040);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int)828908040;
            p244.message_id = (ushort)(ushort)44409;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
                Debug.Assert(pack.vtol_state == MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_FW;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK));
                Debug.Assert(pack.ver_velocity == (short)(short) -19693);
                Debug.Assert(pack.ICAO_address == (uint)246449084U);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)55744);
                Debug.Assert(pack.altitude_type == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
                Debug.Assert(pack.altitude == (int) -598569504);
                Debug.Assert(pack.squawk == (ushort)(ushort)33048);
                Debug.Assert(pack.lat == (int)1249766933);
                Debug.Assert(pack.callsign_LEN(ph) == 4);
                Debug.Assert(pack.callsign_TRY(ph).Equals("hxam"));
                Debug.Assert(pack.tslc == (byte)(byte)163);
                Debug.Assert(pack.heading == (ushort)(ushort)61873);
                Debug.Assert(pack.lon == (int)325505019);
                Debug.Assert(pack.emitter_type == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SMALL);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.heading = (ushort)(ushort)61873;
            p246.altitude = (int) -598569504;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK);
            p246.callsign_SET("hxam", PH) ;
            p246.ICAO_address = (uint)246449084U;
            p246.ver_velocity = (short)(short) -19693;
            p246.squawk = (ushort)(ushort)33048;
            p246.lon = (int)325505019;
            p246.lat = (int)1249766933;
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SMALL;
            p246.tslc = (byte)(byte)163;
            p246.hor_velocity = (ushort)(ushort)55744;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_minimum_delta == (float)2.3927098E38F);
                Debug.Assert(pack.src_ == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
                Debug.Assert(pack.action == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER);
                Debug.Assert(pack.id == (uint)77829642U);
                Debug.Assert(pack.time_to_minimum_delta == (float)9.480917E37F);
                Debug.Assert(pack.horizontal_minimum_delta == (float)2.5718878E38F);
                Debug.Assert(pack.threat_level == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.time_to_minimum_delta = (float)9.480917E37F;
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.id = (uint)77829642U;
            p247.altitude_minimum_delta = (float)2.3927098E38F;
            p247.horizontal_minimum_delta = (float)2.5718878E38F;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER;
            p247.threat_level = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)97);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)252, (byte)55, (byte)59, (byte)93, (byte)43, (byte)31, (byte)144, (byte)204, (byte)103, (byte)118, (byte)178, (byte)177, (byte)203, (byte)193, (byte)238, (byte)117, (byte)130, (byte)170, (byte)102, (byte)69, (byte)53, (byte)39, (byte)83, (byte)63, (byte)83, (byte)251, (byte)196, (byte)60, (byte)174, (byte)167, (byte)99, (byte)221, (byte)26, (byte)211, (byte)49, (byte)173, (byte)38, (byte)241, (byte)17, (byte)43, (byte)0, (byte)108, (byte)170, (byte)109, (byte)120, (byte)130, (byte)228, (byte)243, (byte)108, (byte)152, (byte)133, (byte)13, (byte)145, (byte)94, (byte)75, (byte)195, (byte)107, (byte)118, (byte)21, (byte)168, (byte)37, (byte)151, (byte)85, (byte)76, (byte)235, (byte)127, (byte)17, (byte)167, (byte)205, (byte)43, (byte)213, (byte)27, (byte)154, (byte)83, (byte)190, (byte)123, (byte)80, (byte)216, (byte)64, (byte)213, (byte)191, (byte)216, (byte)118, (byte)51, (byte)207, (byte)173, (byte)113, (byte)47, (byte)44, (byte)14, (byte)143, (byte)85, (byte)17, (byte)117, (byte)65, (byte)109, (byte)229, (byte)136, (byte)76, (byte)47, (byte)32, (byte)164, (byte)168, (byte)0, (byte)199, (byte)195, (byte)237, (byte)177, (byte)129, (byte)126, (byte)71, (byte)231, (byte)108, (byte)97, (byte)118, (byte)252, (byte)102, (byte)201, (byte)163, (byte)100, (byte)165, (byte)166, (byte)106, (byte)23, (byte)116, (byte)85, (byte)65, (byte)135, (byte)131, (byte)88, (byte)251, (byte)14, (byte)54, (byte)152, (byte)116, (byte)167, (byte)196, (byte)117, (byte)15, (byte)244, (byte)160, (byte)184, (byte)83, (byte)52, (byte)170, (byte)146, (byte)157, (byte)158, (byte)223, (byte)119, (byte)95, (byte)120, (byte)14, (byte)183, (byte)227, (byte)225, (byte)88, (byte)155, (byte)126, (byte)52, (byte)210, (byte)238, (byte)253, (byte)32, (byte)232, (byte)164, (byte)97, (byte)253, (byte)60, (byte)151, (byte)155, (byte)101, (byte)162, (byte)115, (byte)149, (byte)17, (byte)89, (byte)8, (byte)141, (byte)239, (byte)213, (byte)29, (byte)183, (byte)132, (byte)130, (byte)190, (byte)123, (byte)124, (byte)253, (byte)198, (byte)9, (byte)76, (byte)82, (byte)3, (byte)237, (byte)191, (byte)128, (byte)112, (byte)14, (byte)129, (byte)68, (byte)154, (byte)137, (byte)181, (byte)107, (byte)81, (byte)92, (byte)27, (byte)64, (byte)193, (byte)209, (byte)205, (byte)118, (byte)242, (byte)54, (byte)182, (byte)180, (byte)169, (byte)16, (byte)125, (byte)128, (byte)12, (byte)103, (byte)37, (byte)165, (byte)2, (byte)181, (byte)195, (byte)21, (byte)198, (byte)133, (byte)21, (byte)170, (byte)193, (byte)251, (byte)71, (byte)167, (byte)169, (byte)144, (byte)105, (byte)147, (byte)191, (byte)62, (byte)76, (byte)152, (byte)122, (byte)84, (byte)50, (byte)188}));
                Debug.Assert(pack.target_component == (byte)(byte)178);
                Debug.Assert(pack.target_network == (byte)(byte)184);
                Debug.Assert(pack.message_type == (ushort)(ushort)38564);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)184;
            p248.target_component = (byte)(byte)178;
            p248.message_type = (ushort)(ushort)38564;
            p248.payload_SET(new byte[] {(byte)252, (byte)55, (byte)59, (byte)93, (byte)43, (byte)31, (byte)144, (byte)204, (byte)103, (byte)118, (byte)178, (byte)177, (byte)203, (byte)193, (byte)238, (byte)117, (byte)130, (byte)170, (byte)102, (byte)69, (byte)53, (byte)39, (byte)83, (byte)63, (byte)83, (byte)251, (byte)196, (byte)60, (byte)174, (byte)167, (byte)99, (byte)221, (byte)26, (byte)211, (byte)49, (byte)173, (byte)38, (byte)241, (byte)17, (byte)43, (byte)0, (byte)108, (byte)170, (byte)109, (byte)120, (byte)130, (byte)228, (byte)243, (byte)108, (byte)152, (byte)133, (byte)13, (byte)145, (byte)94, (byte)75, (byte)195, (byte)107, (byte)118, (byte)21, (byte)168, (byte)37, (byte)151, (byte)85, (byte)76, (byte)235, (byte)127, (byte)17, (byte)167, (byte)205, (byte)43, (byte)213, (byte)27, (byte)154, (byte)83, (byte)190, (byte)123, (byte)80, (byte)216, (byte)64, (byte)213, (byte)191, (byte)216, (byte)118, (byte)51, (byte)207, (byte)173, (byte)113, (byte)47, (byte)44, (byte)14, (byte)143, (byte)85, (byte)17, (byte)117, (byte)65, (byte)109, (byte)229, (byte)136, (byte)76, (byte)47, (byte)32, (byte)164, (byte)168, (byte)0, (byte)199, (byte)195, (byte)237, (byte)177, (byte)129, (byte)126, (byte)71, (byte)231, (byte)108, (byte)97, (byte)118, (byte)252, (byte)102, (byte)201, (byte)163, (byte)100, (byte)165, (byte)166, (byte)106, (byte)23, (byte)116, (byte)85, (byte)65, (byte)135, (byte)131, (byte)88, (byte)251, (byte)14, (byte)54, (byte)152, (byte)116, (byte)167, (byte)196, (byte)117, (byte)15, (byte)244, (byte)160, (byte)184, (byte)83, (byte)52, (byte)170, (byte)146, (byte)157, (byte)158, (byte)223, (byte)119, (byte)95, (byte)120, (byte)14, (byte)183, (byte)227, (byte)225, (byte)88, (byte)155, (byte)126, (byte)52, (byte)210, (byte)238, (byte)253, (byte)32, (byte)232, (byte)164, (byte)97, (byte)253, (byte)60, (byte)151, (byte)155, (byte)101, (byte)162, (byte)115, (byte)149, (byte)17, (byte)89, (byte)8, (byte)141, (byte)239, (byte)213, (byte)29, (byte)183, (byte)132, (byte)130, (byte)190, (byte)123, (byte)124, (byte)253, (byte)198, (byte)9, (byte)76, (byte)82, (byte)3, (byte)237, (byte)191, (byte)128, (byte)112, (byte)14, (byte)129, (byte)68, (byte)154, (byte)137, (byte)181, (byte)107, (byte)81, (byte)92, (byte)27, (byte)64, (byte)193, (byte)209, (byte)205, (byte)118, (byte)242, (byte)54, (byte)182, (byte)180, (byte)169, (byte)16, (byte)125, (byte)128, (byte)12, (byte)103, (byte)37, (byte)165, (byte)2, (byte)181, (byte)195, (byte)21, (byte)198, (byte)133, (byte)21, (byte)170, (byte)193, (byte)251, (byte)71, (byte)167, (byte)169, (byte)144, (byte)105, (byte)147, (byte)191, (byte)62, (byte)76, (byte)152, (byte)122, (byte)84, (byte)50, (byte)188}, 0) ;
            p248.target_system = (byte)(byte)97;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)200);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte) - 97, (sbyte)113, (sbyte) - 62, (sbyte) - 110, (sbyte) - 25, (sbyte) - 101, (sbyte)12, (sbyte)106, (sbyte) - 22, (sbyte)102, (sbyte)105, (sbyte) - 53, (sbyte) - 32, (sbyte)7, (sbyte)54, (sbyte)55, (sbyte) - 39, (sbyte)13, (sbyte)76, (sbyte) - 30, (sbyte) - 15, (sbyte) - 124, (sbyte) - 51, (sbyte)77, (sbyte) - 32, (sbyte) - 103, (sbyte)36, (sbyte)31, (sbyte) - 71, (sbyte)99, (sbyte)91, (sbyte)21}));
                Debug.Assert(pack.address == (ushort)(ushort)8717);
                Debug.Assert(pack.ver == (byte)(byte)167);
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.value_SET(new sbyte[] {(sbyte) - 97, (sbyte)113, (sbyte) - 62, (sbyte) - 110, (sbyte) - 25, (sbyte) - 101, (sbyte)12, (sbyte)106, (sbyte) - 22, (sbyte)102, (sbyte)105, (sbyte) - 53, (sbyte) - 32, (sbyte)7, (sbyte)54, (sbyte)55, (sbyte) - 39, (sbyte)13, (sbyte)76, (sbyte) - 30, (sbyte) - 15, (sbyte) - 124, (sbyte) - 51, (sbyte)77, (sbyte) - 32, (sbyte) - 103, (sbyte)36, (sbyte)31, (sbyte) - 71, (sbyte)99, (sbyte)91, (sbyte)21}, 0) ;
            p249.address = (ushort)(ushort)8717;
            p249.ver = (byte)(byte)167;
            p249.type = (byte)(byte)200;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 5);
                Debug.Assert(pack.name_TRY(ph).Equals("Kjibf"));
                Debug.Assert(pack.z == (float)1.4301817E37F);
                Debug.Assert(pack.x == (float)3.3541459E38F);
                Debug.Assert(pack.y == (float)1.7120132E38F);
                Debug.Assert(pack.time_usec == (ulong)6458215909397900937L);
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.z = (float)1.4301817E37F;
            p250.y = (float)1.7120132E38F;
            p250.time_usec = (ulong)6458215909397900937L;
            p250.x = (float)3.3541459E38F;
            p250.name_SET("Kjibf", PH) ;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float) -1.222718E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3663394950U);
                Debug.Assert(pack.name_LEN(ph) == 3);
                Debug.Assert(pack.name_TRY(ph).Equals("uyi"));
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)3663394950U;
            p251.value = (float) -1.222718E38F;
            p251.name_SET("uyi", PH) ;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2094798454U);
                Debug.Assert(pack.value == (int)1112569885);
                Debug.Assert(pack.name_LEN(ph) == 1);
                Debug.Assert(pack.name_TRY(ph).Equals("U"));
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.name_SET("U", PH) ;
            p252.value = (int)1112569885;
            p252.time_boot_ms = (uint)2094798454U;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.severity == MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
                Debug.Assert(pack.text_LEN(ph) == 34);
                Debug.Assert(pack.text_TRY(ph).Equals("qbtENayuzzpdvxnvogruIVwvvbrqynychx"));
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("qbtENayuzzpdvxnvogruIVwvvbrqynychx", PH) ;
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_CRITICAL;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ind == (byte)(byte)235);
                Debug.Assert(pack.time_boot_ms == (uint)2375168392U);
                Debug.Assert(pack.value == (float) -9.373018E37F);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.ind = (byte)(byte)235;
            p254.value = (float) -9.373018E37F;
            p254.time_boot_ms = (uint)2375168392U;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)149, (byte)41, (byte)63, (byte)51, (byte)11, (byte)45, (byte)192, (byte)133, (byte)180, (byte)220, (byte)63, (byte)51, (byte)115, (byte)19, (byte)124, (byte)251, (byte)40, (byte)7, (byte)196, (byte)240, (byte)63, (byte)192, (byte)219, (byte)96, (byte)129, (byte)42, (byte)170, (byte)15, (byte)238, (byte)54, (byte)206, (byte)168}));
                Debug.Assert(pack.target_system == (byte)(byte)163);
                Debug.Assert(pack.initial_timestamp == (ulong)1284415789229449201L);
                Debug.Assert(pack.target_component == (byte)(byte)81);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.secret_key_SET(new byte[] {(byte)149, (byte)41, (byte)63, (byte)51, (byte)11, (byte)45, (byte)192, (byte)133, (byte)180, (byte)220, (byte)63, (byte)51, (byte)115, (byte)19, (byte)124, (byte)251, (byte)40, (byte)7, (byte)196, (byte)240, (byte)63, (byte)192, (byte)219, (byte)96, (byte)129, (byte)42, (byte)170, (byte)15, (byte)238, (byte)54, (byte)206, (byte)168}, 0) ;
            p256.target_system = (byte)(byte)163;
            p256.initial_timestamp = (ulong)1284415789229449201L;
            p256.target_component = (byte)(byte)81;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_change_ms == (uint)2707681416U);
                Debug.Assert(pack.state == (byte)(byte)46);
                Debug.Assert(pack.time_boot_ms == (uint)556397548U);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.last_change_ms = (uint)2707681416U;
            p257.time_boot_ms = (uint)556397548U;
            p257.state = (byte)(byte)46;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)203);
                Debug.Assert(pack.target_component == (byte)(byte)173);
                Debug.Assert(pack.tune_LEN(ph) == 28);
                Debug.Assert(pack.tune_TRY(ph).Equals("omdbtalexdofwdrlwWlfkylwyqgh"));
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)203;
            p258.target_component = (byte)(byte)173;
            p258.tune_SET("omdbtalexdofwdrlwWlfkylwyqgh", PH) ;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_size_h == (float)1.2113457E38F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)32906);
                Debug.Assert(pack.time_boot_ms == (uint)837413487U);
                Debug.Assert(pack.focal_length == (float) -1.3403604E38F);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)243, (byte)206, (byte)222, (byte)196, (byte)168, (byte)229, (byte)45, (byte)40, (byte)240, (byte)104, (byte)230, (byte)162, (byte)225, (byte)241, (byte)250, (byte)143, (byte)193, (byte)119, (byte)247, (byte)243, (byte)91, (byte)102, (byte)197, (byte)91, (byte)40, (byte)187, (byte)61, (byte)85, (byte)46, (byte)196, (byte)97, (byte)107}));
                Debug.Assert(pack.firmware_version == (uint)2992107238U);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)49792);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)57352);
                Debug.Assert(pack.lens_id == (byte)(byte)196);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE));
                Debug.Assert(pack.sensor_size_v == (float) -1.5156879E37F);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 45);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("qCqdYoccahtqymfmzgeuozHupgdkkxtdzSavhuebyzkms"));
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)26, (byte)52, (byte)95, (byte)243, (byte)159, (byte)67, (byte)228, (byte)189, (byte)167, (byte)144, (byte)110, (byte)65, (byte)212, (byte)109, (byte)17, (byte)6, (byte)245, (byte)240, (byte)194, (byte)185, (byte)238, (byte)131, (byte)76, (byte)27, (byte)115, (byte)222, (byte)74, (byte)127, (byte)152, (byte)227, (byte)161, (byte)3}));
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.resolution_v = (ushort)(ushort)57352;
            p259.model_name_SET(new byte[] {(byte)26, (byte)52, (byte)95, (byte)243, (byte)159, (byte)67, (byte)228, (byte)189, (byte)167, (byte)144, (byte)110, (byte)65, (byte)212, (byte)109, (byte)17, (byte)6, (byte)245, (byte)240, (byte)194, (byte)185, (byte)238, (byte)131, (byte)76, (byte)27, (byte)115, (byte)222, (byte)74, (byte)127, (byte)152, (byte)227, (byte)161, (byte)3}, 0) ;
            p259.firmware_version = (uint)2992107238U;
            p259.sensor_size_h = (float)1.2113457E38F;
            p259.cam_definition_version = (ushort)(ushort)49792;
            p259.sensor_size_v = (float) -1.5156879E37F;
            p259.cam_definition_uri_SET("qCqdYoccahtqymfmzgeuozHupgdkkxtdzSavhuebyzkms", PH) ;
            p259.time_boot_ms = (uint)837413487U;
            p259.vendor_name_SET(new byte[] {(byte)243, (byte)206, (byte)222, (byte)196, (byte)168, (byte)229, (byte)45, (byte)40, (byte)240, (byte)104, (byte)230, (byte)162, (byte)225, (byte)241, (byte)250, (byte)143, (byte)193, (byte)119, (byte)247, (byte)243, (byte)91, (byte)102, (byte)197, (byte)91, (byte)40, (byte)187, (byte)61, (byte)85, (byte)46, (byte)196, (byte)97, (byte)107}, 0) ;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
            p259.lens_id = (byte)(byte)196;
            p259.resolution_h = (ushort)(ushort)32906;
            p259.focal_length = (float) -1.3403604E38F;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == CAMERA_MODE.CAMERA_MODE_IMAGE);
                Debug.Assert(pack.time_boot_ms == (uint)3813667255U);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)3813667255U;
            p260.mode_id = CAMERA_MODE.CAMERA_MODE_IMAGE;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.storage_count == (byte)(byte)248);
                Debug.Assert(pack.used_capacity == (float) -4.114331E37F);
                Debug.Assert(pack.available_capacity == (float) -1.2856103E38F);
                Debug.Assert(pack.read_speed == (float) -1.6979251E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3086596579U);
                Debug.Assert(pack.storage_id == (byte)(byte)149);
                Debug.Assert(pack.total_capacity == (float) -1.771525E38F);
                Debug.Assert(pack.status == (byte)(byte)135);
                Debug.Assert(pack.write_speed == (float) -7.3618885E36F);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.available_capacity = (float) -1.2856103E38F;
            p261.time_boot_ms = (uint)3086596579U;
            p261.read_speed = (float) -1.6979251E38F;
            p261.write_speed = (float) -7.3618885E36F;
            p261.storage_id = (byte)(byte)149;
            p261.status = (byte)(byte)135;
            p261.storage_count = (byte)(byte)248;
            p261.total_capacity = (float) -1.771525E38F;
            p261.used_capacity = (float) -4.114331E37F;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.recording_time_ms == (uint)809954415U);
                Debug.Assert(pack.image_status == (byte)(byte)243);
                Debug.Assert(pack.image_interval == (float)1.2041477E38F);
                Debug.Assert(pack.video_status == (byte)(byte)196);
                Debug.Assert(pack.available_capacity == (float) -3.2299472E38F);
                Debug.Assert(pack.time_boot_ms == (uint)919199040U);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.recording_time_ms = (uint)809954415U;
            p262.video_status = (byte)(byte)196;
            p262.image_status = (byte)(byte)243;
            p262.available_capacity = (float) -3.2299472E38F;
            p262.image_interval = (float)1.2041477E38F;
            p262.time_boot_ms = (uint)919199040U;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -746712649);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.908118E38F, -1.0228553E38F, -2.340494E38F, 1.671167E37F}));
                Debug.Assert(pack.capture_result == (sbyte)(sbyte) - 25);
                Debug.Assert(pack.camera_id == (byte)(byte)114);
                Debug.Assert(pack.time_boot_ms == (uint)1966400953U);
                Debug.Assert(pack.time_utc == (ulong)7053516085486252873L);
                Debug.Assert(pack.lat == (int) -1990819574);
                Debug.Assert(pack.image_index == (int)1607422076);
                Debug.Assert(pack.relative_alt == (int)1162223354);
                Debug.Assert(pack.alt == (int)1773292796);
                Debug.Assert(pack.file_url_LEN(ph) == 116);
                Debug.Assert(pack.file_url_TRY(ph).Equals("RwbneiwbYiiknNgajfkchHwuoTwAopjinnhbfXajqdezgekhljkcfpgcerpegwqhzvlmbcdobvcjfmexpzMpcasfvkygabbrgcCxjQBjnftQqdivyqwS"));
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.lat = (int) -1990819574;
            p263.file_url_SET("RwbneiwbYiiknNgajfkchHwuoTwAopjinnhbfXajqdezgekhljkcfpgcerpegwqhzvlmbcdobvcjfmexpzMpcasfvkygabbrgcCxjQBjnftQqdivyqwS", PH) ;
            p263.time_utc = (ulong)7053516085486252873L;
            p263.lon = (int) -746712649;
            p263.image_index = (int)1607422076;
            p263.camera_id = (byte)(byte)114;
            p263.capture_result = (sbyte)(sbyte) - 25;
            p263.q_SET(new float[] {-2.908118E38F, -1.0228553E38F, -2.340494E38F, 1.671167E37F}, 0) ;
            p263.alt = (int)1773292796;
            p263.relative_alt = (int)1162223354;
            p263.time_boot_ms = (uint)1966400953U;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.arming_time_utc == (ulong)2593235313510820076L);
                Debug.Assert(pack.flight_uuid == (ulong)4774929083826370299L);
                Debug.Assert(pack.time_boot_ms == (uint)247373719U);
                Debug.Assert(pack.takeoff_time_utc == (ulong)7109441014271509843L);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.arming_time_utc = (ulong)2593235313510820076L;
            p264.time_boot_ms = (uint)247373719U;
            p264.takeoff_time_utc = (ulong)7109441014271509843L;
            p264.flight_uuid = (ulong)4774929083826370299L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2946597956U);
                Debug.Assert(pack.roll == (float) -2.5517567E38F);
                Debug.Assert(pack.pitch == (float) -1.966627E38F);
                Debug.Assert(pack.yaw == (float)1.9547746E38F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.pitch = (float) -1.966627E38F;
            p265.roll = (float) -2.5517567E38F;
            p265.yaw = (float)1.9547746E38F;
            p265.time_boot_ms = (uint)2946597956U;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)60, (byte)232, (byte)90, (byte)149, (byte)29, (byte)141, (byte)81, (byte)178, (byte)88, (byte)75, (byte)88, (byte)158, (byte)216, (byte)232, (byte)198, (byte)103, (byte)135, (byte)25, (byte)177, (byte)142, (byte)230, (byte)27, (byte)1, (byte)203, (byte)254, (byte)119, (byte)176, (byte)92, (byte)167, (byte)140, (byte)114, (byte)190, (byte)121, (byte)220, (byte)141, (byte)208, (byte)133, (byte)56, (byte)62, (byte)179, (byte)204, (byte)65, (byte)142, (byte)140, (byte)116, (byte)237, (byte)87, (byte)130, (byte)7, (byte)159, (byte)152, (byte)78, (byte)13, (byte)138, (byte)153, (byte)190, (byte)147, (byte)86, (byte)220, (byte)220, (byte)203, (byte)32, (byte)144, (byte)14, (byte)221, (byte)27, (byte)246, (byte)248, (byte)113, (byte)99, (byte)17, (byte)128, (byte)134, (byte)215, (byte)252, (byte)49, (byte)94, (byte)197, (byte)195, (byte)250, (byte)253, (byte)230, (byte)252, (byte)77, (byte)65, (byte)103, (byte)34, (byte)183, (byte)82, (byte)69, (byte)221, (byte)84, (byte)189, (byte)81, (byte)58, (byte)30, (byte)254, (byte)242, (byte)81, (byte)239, (byte)33, (byte)135, (byte)227, (byte)100, (byte)121, (byte)147, (byte)214, (byte)34, (byte)149, (byte)90, (byte)121, (byte)5, (byte)140, (byte)192, (byte)231, (byte)160, (byte)136, (byte)134, (byte)72, (byte)96, (byte)53, (byte)246, (byte)160, (byte)249, (byte)138, (byte)49, (byte)41, (byte)231, (byte)35, (byte)167, (byte)149, (byte)169, (byte)97, (byte)152, (byte)236, (byte)61, (byte)142, (byte)141, (byte)214, (byte)102, (byte)140, (byte)223, (byte)146, (byte)85, (byte)127, (byte)76, (byte)37, (byte)68, (byte)111, (byte)48, (byte)242, (byte)238, (byte)91, (byte)191, (byte)67, (byte)45, (byte)176, (byte)193, (byte)33, (byte)38, (byte)160, (byte)174, (byte)162, (byte)58, (byte)254, (byte)28, (byte)164, (byte)218, (byte)1, (byte)121, (byte)152, (byte)9, (byte)238, (byte)10, (byte)177, (byte)148, (byte)247, (byte)64, (byte)152, (byte)187, (byte)161, (byte)42, (byte)254, (byte)178, (byte)101, (byte)200, (byte)5, (byte)9, (byte)96, (byte)113, (byte)235, (byte)247, (byte)137, (byte)136, (byte)238, (byte)39, (byte)182, (byte)194, (byte)253, (byte)148, (byte)11, (byte)191, (byte)123, (byte)49, (byte)72, (byte)85, (byte)39, (byte)106, (byte)148, (byte)203, (byte)16, (byte)190, (byte)132, (byte)23, (byte)190, (byte)44, (byte)224, (byte)104, (byte)240, (byte)153, (byte)127, (byte)78, (byte)179, (byte)213, (byte)207, (byte)121, (byte)226, (byte)228, (byte)185, (byte)24, (byte)206, (byte)209, (byte)47, (byte)123, (byte)31, (byte)1, (byte)51, (byte)129, (byte)100, (byte)98, (byte)129, (byte)159, (byte)171, (byte)118, (byte)102, (byte)229, (byte)150, (byte)126, (byte)207}));
                Debug.Assert(pack.target_system == (byte)(byte)26);
                Debug.Assert(pack.length == (byte)(byte)140);
                Debug.Assert(pack.first_message_offset == (byte)(byte)95);
                Debug.Assert(pack.sequence == (ushort)(ushort)30312);
                Debug.Assert(pack.target_component == (byte)(byte)25);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.sequence = (ushort)(ushort)30312;
            p266.first_message_offset = (byte)(byte)95;
            p266.target_system = (byte)(byte)26;
            p266.data__SET(new byte[] {(byte)60, (byte)232, (byte)90, (byte)149, (byte)29, (byte)141, (byte)81, (byte)178, (byte)88, (byte)75, (byte)88, (byte)158, (byte)216, (byte)232, (byte)198, (byte)103, (byte)135, (byte)25, (byte)177, (byte)142, (byte)230, (byte)27, (byte)1, (byte)203, (byte)254, (byte)119, (byte)176, (byte)92, (byte)167, (byte)140, (byte)114, (byte)190, (byte)121, (byte)220, (byte)141, (byte)208, (byte)133, (byte)56, (byte)62, (byte)179, (byte)204, (byte)65, (byte)142, (byte)140, (byte)116, (byte)237, (byte)87, (byte)130, (byte)7, (byte)159, (byte)152, (byte)78, (byte)13, (byte)138, (byte)153, (byte)190, (byte)147, (byte)86, (byte)220, (byte)220, (byte)203, (byte)32, (byte)144, (byte)14, (byte)221, (byte)27, (byte)246, (byte)248, (byte)113, (byte)99, (byte)17, (byte)128, (byte)134, (byte)215, (byte)252, (byte)49, (byte)94, (byte)197, (byte)195, (byte)250, (byte)253, (byte)230, (byte)252, (byte)77, (byte)65, (byte)103, (byte)34, (byte)183, (byte)82, (byte)69, (byte)221, (byte)84, (byte)189, (byte)81, (byte)58, (byte)30, (byte)254, (byte)242, (byte)81, (byte)239, (byte)33, (byte)135, (byte)227, (byte)100, (byte)121, (byte)147, (byte)214, (byte)34, (byte)149, (byte)90, (byte)121, (byte)5, (byte)140, (byte)192, (byte)231, (byte)160, (byte)136, (byte)134, (byte)72, (byte)96, (byte)53, (byte)246, (byte)160, (byte)249, (byte)138, (byte)49, (byte)41, (byte)231, (byte)35, (byte)167, (byte)149, (byte)169, (byte)97, (byte)152, (byte)236, (byte)61, (byte)142, (byte)141, (byte)214, (byte)102, (byte)140, (byte)223, (byte)146, (byte)85, (byte)127, (byte)76, (byte)37, (byte)68, (byte)111, (byte)48, (byte)242, (byte)238, (byte)91, (byte)191, (byte)67, (byte)45, (byte)176, (byte)193, (byte)33, (byte)38, (byte)160, (byte)174, (byte)162, (byte)58, (byte)254, (byte)28, (byte)164, (byte)218, (byte)1, (byte)121, (byte)152, (byte)9, (byte)238, (byte)10, (byte)177, (byte)148, (byte)247, (byte)64, (byte)152, (byte)187, (byte)161, (byte)42, (byte)254, (byte)178, (byte)101, (byte)200, (byte)5, (byte)9, (byte)96, (byte)113, (byte)235, (byte)247, (byte)137, (byte)136, (byte)238, (byte)39, (byte)182, (byte)194, (byte)253, (byte)148, (byte)11, (byte)191, (byte)123, (byte)49, (byte)72, (byte)85, (byte)39, (byte)106, (byte)148, (byte)203, (byte)16, (byte)190, (byte)132, (byte)23, (byte)190, (byte)44, (byte)224, (byte)104, (byte)240, (byte)153, (byte)127, (byte)78, (byte)179, (byte)213, (byte)207, (byte)121, (byte)226, (byte)228, (byte)185, (byte)24, (byte)206, (byte)209, (byte)47, (byte)123, (byte)31, (byte)1, (byte)51, (byte)129, (byte)100, (byte)98, (byte)129, (byte)159, (byte)171, (byte)118, (byte)102, (byte)229, (byte)150, (byte)126, (byte)207}, 0) ;
            p266.target_component = (byte)(byte)25;
            p266.length = (byte)(byte)140;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)57, (byte)71, (byte)0, (byte)76, (byte)223, (byte)45, (byte)3, (byte)240, (byte)102, (byte)84, (byte)103, (byte)106, (byte)44, (byte)78, (byte)161, (byte)151, (byte)223, (byte)236, (byte)249, (byte)82, (byte)101, (byte)140, (byte)171, (byte)125, (byte)123, (byte)146, (byte)154, (byte)62, (byte)27, (byte)20, (byte)42, (byte)218, (byte)240, (byte)192, (byte)235, (byte)111, (byte)159, (byte)143, (byte)117, (byte)12, (byte)114, (byte)136, (byte)4, (byte)121, (byte)13, (byte)35, (byte)82, (byte)172, (byte)235, (byte)134, (byte)6, (byte)85, (byte)0, (byte)107, (byte)223, (byte)152, (byte)227, (byte)72, (byte)36, (byte)208, (byte)20, (byte)16, (byte)10, (byte)114, (byte)26, (byte)1, (byte)128, (byte)230, (byte)60, (byte)170, (byte)89, (byte)58, (byte)208, (byte)204, (byte)244, (byte)200, (byte)31, (byte)108, (byte)110, (byte)10, (byte)163, (byte)129, (byte)235, (byte)120, (byte)55, (byte)130, (byte)77, (byte)216, (byte)67, (byte)49, (byte)9, (byte)48, (byte)200, (byte)117, (byte)32, (byte)208, (byte)77, (byte)8, (byte)50, (byte)186, (byte)214, (byte)231, (byte)46, (byte)119, (byte)193, (byte)84, (byte)186, (byte)200, (byte)32, (byte)167, (byte)192, (byte)84, (byte)175, (byte)22, (byte)132, (byte)254, (byte)91, (byte)117, (byte)252, (byte)218, (byte)68, (byte)230, (byte)66, (byte)45, (byte)117, (byte)37, (byte)193, (byte)153, (byte)23, (byte)63, (byte)52, (byte)213, (byte)181, (byte)112, (byte)73, (byte)1, (byte)11, (byte)168, (byte)15, (byte)23, (byte)176, (byte)96, (byte)111, (byte)77, (byte)255, (byte)196, (byte)92, (byte)22, (byte)40, (byte)33, (byte)27, (byte)195, (byte)138, (byte)185, (byte)36, (byte)0, (byte)14, (byte)195, (byte)0, (byte)14, (byte)109, (byte)49, (byte)166, (byte)202, (byte)186, (byte)139, (byte)188, (byte)101, (byte)113, (byte)170, (byte)199, (byte)118, (byte)186, (byte)0, (byte)20, (byte)163, (byte)1, (byte)215, (byte)87, (byte)177, (byte)167, (byte)181, (byte)196, (byte)133, (byte)84, (byte)148, (byte)89, (byte)78, (byte)159, (byte)211, (byte)177, (byte)112, (byte)211, (byte)181, (byte)180, (byte)189, (byte)60, (byte)154, (byte)73, (byte)175, (byte)228, (byte)208, (byte)211, (byte)129, (byte)222, (byte)222, (byte)128, (byte)156, (byte)57, (byte)98, (byte)115, (byte)69, (byte)41, (byte)20, (byte)177, (byte)8, (byte)187, (byte)178, (byte)127, (byte)75, (byte)169, (byte)205, (byte)169, (byte)22, (byte)218, (byte)22, (byte)145, (byte)119, (byte)254, (byte)225, (byte)169, (byte)82, (byte)149, (byte)97, (byte)26, (byte)207, (byte)177, (byte)244, (byte)44, (byte)209, (byte)223, (byte)89, (byte)195, (byte)186, (byte)80, (byte)129, (byte)90, (byte)226, (byte)52}));
                Debug.Assert(pack.sequence == (ushort)(ushort)56310);
                Debug.Assert(pack.target_component == (byte)(byte)174);
                Debug.Assert(pack.target_system == (byte)(byte)60);
                Debug.Assert(pack.length == (byte)(byte)32);
                Debug.Assert(pack.first_message_offset == (byte)(byte)216);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.sequence = (ushort)(ushort)56310;
            p267.first_message_offset = (byte)(byte)216;
            p267.target_system = (byte)(byte)60;
            p267.target_component = (byte)(byte)174;
            p267.length = (byte)(byte)32;
            p267.data__SET(new byte[] {(byte)57, (byte)71, (byte)0, (byte)76, (byte)223, (byte)45, (byte)3, (byte)240, (byte)102, (byte)84, (byte)103, (byte)106, (byte)44, (byte)78, (byte)161, (byte)151, (byte)223, (byte)236, (byte)249, (byte)82, (byte)101, (byte)140, (byte)171, (byte)125, (byte)123, (byte)146, (byte)154, (byte)62, (byte)27, (byte)20, (byte)42, (byte)218, (byte)240, (byte)192, (byte)235, (byte)111, (byte)159, (byte)143, (byte)117, (byte)12, (byte)114, (byte)136, (byte)4, (byte)121, (byte)13, (byte)35, (byte)82, (byte)172, (byte)235, (byte)134, (byte)6, (byte)85, (byte)0, (byte)107, (byte)223, (byte)152, (byte)227, (byte)72, (byte)36, (byte)208, (byte)20, (byte)16, (byte)10, (byte)114, (byte)26, (byte)1, (byte)128, (byte)230, (byte)60, (byte)170, (byte)89, (byte)58, (byte)208, (byte)204, (byte)244, (byte)200, (byte)31, (byte)108, (byte)110, (byte)10, (byte)163, (byte)129, (byte)235, (byte)120, (byte)55, (byte)130, (byte)77, (byte)216, (byte)67, (byte)49, (byte)9, (byte)48, (byte)200, (byte)117, (byte)32, (byte)208, (byte)77, (byte)8, (byte)50, (byte)186, (byte)214, (byte)231, (byte)46, (byte)119, (byte)193, (byte)84, (byte)186, (byte)200, (byte)32, (byte)167, (byte)192, (byte)84, (byte)175, (byte)22, (byte)132, (byte)254, (byte)91, (byte)117, (byte)252, (byte)218, (byte)68, (byte)230, (byte)66, (byte)45, (byte)117, (byte)37, (byte)193, (byte)153, (byte)23, (byte)63, (byte)52, (byte)213, (byte)181, (byte)112, (byte)73, (byte)1, (byte)11, (byte)168, (byte)15, (byte)23, (byte)176, (byte)96, (byte)111, (byte)77, (byte)255, (byte)196, (byte)92, (byte)22, (byte)40, (byte)33, (byte)27, (byte)195, (byte)138, (byte)185, (byte)36, (byte)0, (byte)14, (byte)195, (byte)0, (byte)14, (byte)109, (byte)49, (byte)166, (byte)202, (byte)186, (byte)139, (byte)188, (byte)101, (byte)113, (byte)170, (byte)199, (byte)118, (byte)186, (byte)0, (byte)20, (byte)163, (byte)1, (byte)215, (byte)87, (byte)177, (byte)167, (byte)181, (byte)196, (byte)133, (byte)84, (byte)148, (byte)89, (byte)78, (byte)159, (byte)211, (byte)177, (byte)112, (byte)211, (byte)181, (byte)180, (byte)189, (byte)60, (byte)154, (byte)73, (byte)175, (byte)228, (byte)208, (byte)211, (byte)129, (byte)222, (byte)222, (byte)128, (byte)156, (byte)57, (byte)98, (byte)115, (byte)69, (byte)41, (byte)20, (byte)177, (byte)8, (byte)187, (byte)178, (byte)127, (byte)75, (byte)169, (byte)205, (byte)169, (byte)22, (byte)218, (byte)22, (byte)145, (byte)119, (byte)254, (byte)225, (byte)169, (byte)82, (byte)149, (byte)97, (byte)26, (byte)207, (byte)177, (byte)244, (byte)44, (byte)209, (byte)223, (byte)89, (byte)195, (byte)186, (byte)80, (byte)129, (byte)90, (byte)226, (byte)52}, 0) ;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)111);
                Debug.Assert(pack.sequence == (ushort)(ushort)36132);
                Debug.Assert(pack.target_system == (byte)(byte)92);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_component = (byte)(byte)111;
            p268.sequence = (ushort)(ushort)36132;
            p268.target_system = (byte)(byte)92;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.framerate == (float) -2.0332158E38F);
                Debug.Assert(pack.camera_id == (byte)(byte)67);
                Debug.Assert(pack.rotation == (ushort)(ushort)1827);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)37441);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)45728);
                Debug.Assert(pack.bitrate == (uint)2828991035U);
                Debug.Assert(pack.uri_LEN(ph) == 220);
                Debug.Assert(pack.uri_TRY(ph).Equals("ufnchybhiyqBpeewwnejMpvmhlzpTjoxrnifnnbwlmngsuixbhePncwwSuqfeleyqNaeNjnkpykvnaVxiGTyolgTglwpoezkauvtbmottihkbggzgrszawesbiimTgjlhgdcetuzulykSZmjgfatiElqybpsmztpOnsogscXjepzipfoUifBVZgrjbuciTdAxticaeqzifAwpzQphbikkqwlkqze"));
                Debug.Assert(pack.status == (byte)(byte)201);
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.resolution_v = (ushort)(ushort)45728;
            p269.status = (byte)(byte)201;
            p269.bitrate = (uint)2828991035U;
            p269.uri_SET("ufnchybhiyqBpeewwnejMpvmhlzpTjoxrnifnnbwlmngsuixbhePncwwSuqfeleyqNaeNjnkpykvnaVxiGTyolgTglwpoezkauvtbmottihkbggzgrszawesbiimTgjlhgdcetuzulykSZmjgfatiElqybpsmztpOnsogscXjepzipfoUifBVZgrjbuciTdAxticaeqzifAwpzQphbikkqwlkqze", PH) ;
            p269.rotation = (ushort)(ushort)1827;
            p269.framerate = (float) -2.0332158E38F;
            p269.resolution_h = (ushort)(ushort)37441;
            p269.camera_id = (byte)(byte)67;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.bitrate == (uint)2468956755U);
                Debug.Assert(pack.camera_id == (byte)(byte)41);
                Debug.Assert(pack.target_component == (byte)(byte)98);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)13807);
                Debug.Assert(pack.target_system == (byte)(byte)216);
                Debug.Assert(pack.rotation == (ushort)(ushort)19763);
                Debug.Assert(pack.uri_LEN(ph) == 83);
                Debug.Assert(pack.uri_TRY(ph).Equals("naorDgmdemzbjrdIkVtfgkgasryueoHgzjAhlJuzgTpjxQlWjcvJizybhoykupxnlfvGhnQwyhvkmmhczwh"));
                Debug.Assert(pack.framerate == (float) -2.09869E38F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)62406);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.resolution_h = (ushort)(ushort)62406;
            p270.resolution_v = (ushort)(ushort)13807;
            p270.target_component = (byte)(byte)98;
            p270.target_system = (byte)(byte)216;
            p270.uri_SET("naorDgmdemzbjrdIkVtfgkgasryueoHgzjAhlJuzgTpjxQlWjcvJizybhoykupxnlfvGhnQwyhvkmmhczwh", PH) ;
            p270.framerate = (float) -2.09869E38F;
            p270.rotation = (ushort)(ushort)19763;
            p270.camera_id = (byte)(byte)41;
            p270.bitrate = (uint)2468956755U;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 48);
                Debug.Assert(pack.password_TRY(ph).Equals("oiuhZroiggnvongqzwzzofxEqtvlhbdsdkdfrbmncazmlRyw"));
                Debug.Assert(pack.ssid_LEN(ph) == 13);
                Debug.Assert(pack.ssid_TRY(ph).Equals("phfipiqkxofgz"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.password_SET("oiuhZroiggnvongqzwzzofxEqtvlhbdsdkdfrbmncazmlRyw", PH) ;
            p299.ssid_SET("phfipiqkxofgz", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.max_version == (ushort)(ushort)48422);
                Debug.Assert(pack.version == (ushort)(ushort)19183);
                Debug.Assert(pack.min_version == (ushort)(ushort)44277);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)48, (byte)86, (byte)90, (byte)71, (byte)19, (byte)158, (byte)159, (byte)51}));
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)135, (byte)86, (byte)148, (byte)75, (byte)238, (byte)110, (byte)222, (byte)115}));
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.spec_version_hash_SET(new byte[] {(byte)135, (byte)86, (byte)148, (byte)75, (byte)238, (byte)110, (byte)222, (byte)115}, 0) ;
            p300.version = (ushort)(ushort)19183;
            p300.library_version_hash_SET(new byte[] {(byte)48, (byte)86, (byte)90, (byte)71, (byte)19, (byte)158, (byte)159, (byte)51}, 0) ;
            p300.max_version = (ushort)(ushort)48422;
            p300.min_version = (ushort)(ushort)44277;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.health == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
                Debug.Assert(pack.time_usec == (ulong)5316331003319831016L);
                Debug.Assert(pack.sub_mode == (byte)(byte)21);
                Debug.Assert(pack.uptime_sec == (uint)1281474407U);
                Debug.Assert(pack.mode == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)32545);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)5316331003319831016L;
            p310.uptime_sec = (uint)1281474407U;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL;
            p310.vendor_specific_status_code = (ushort)(ushort)32545;
            p310.sub_mode = (byte)(byte)21;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sw_vcs_commit == (uint)3060521983U);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)103, (byte)32, (byte)8, (byte)248, (byte)22, (byte)211, (byte)70, (byte)67, (byte)194, (byte)46, (byte)129, (byte)204, (byte)155, (byte)240, (byte)99, (byte)42}));
                Debug.Assert(pack.hw_version_minor == (byte)(byte)77);
                Debug.Assert(pack.sw_version_major == (byte)(byte)254);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)133);
                Debug.Assert(pack.uptime_sec == (uint)2718135644U);
                Debug.Assert(pack.hw_version_major == (byte)(byte)100);
                Debug.Assert(pack.time_usec == (ulong)3378891532963783732L);
                Debug.Assert(pack.name_LEN(ph) == 17);
                Debug.Assert(pack.name_TRY(ph).Equals("juiagqngrejryIAzi"));
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.sw_vcs_commit = (uint)3060521983U;
            p311.uptime_sec = (uint)2718135644U;
            p311.sw_version_minor = (byte)(byte)133;
            p311.sw_version_major = (byte)(byte)254;
            p311.name_SET("juiagqngrejryIAzi", PH) ;
            p311.hw_unique_id_SET(new byte[] {(byte)103, (byte)32, (byte)8, (byte)248, (byte)22, (byte)211, (byte)70, (byte)67, (byte)194, (byte)46, (byte)129, (byte)204, (byte)155, (byte)240, (byte)99, (byte)42}, 0) ;
            p311.hw_version_major = (byte)(byte)100;
            p311.hw_version_minor = (byte)(byte)77;
            p311.time_usec = (ulong)3378891532963783732L;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (short)(short)15104);
                Debug.Assert(pack.param_id_LEN(ph) == 2);
                Debug.Assert(pack.param_id_TRY(ph).Equals("fb"));
                Debug.Assert(pack.target_component == (byte)(byte)120);
                Debug.Assert(pack.target_system == (byte)(byte)21);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_id_SET("fb", PH) ;
            p320.target_system = (byte)(byte)21;
            p320.param_index = (short)(short)15104;
            p320.target_component = (byte)(byte)120;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)163);
                Debug.Assert(pack.target_component == (byte)(byte)230);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_component = (byte)(byte)230;
            p321.target_system = (byte)(byte)163;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 2);
                Debug.Assert(pack.param_id_TRY(ph).Equals("Vo"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
                Debug.Assert(pack.param_count == (ushort)(ushort)61211);
                Debug.Assert(pack.param_value_LEN(ph) == 125);
                Debug.Assert(pack.param_value_TRY(ph).Equals("xtrtfonuEauwuegGgxcIknlhcXvsulzmtmxFgtifbwksYkovGskpzosqqOoizdusmuteslaUgzgfGdAvjnayEmisphzoprhkgmwztgzexhzcZeRiqzecrsdFgpvtk"));
                Debug.Assert(pack.param_index == (ushort)(ushort)29363);
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_value_SET("xtrtfonuEauwuegGgxcIknlhcXvsulzmtmxFgtifbwksYkovGskpzosqqOoizdusmuteslaUgzgfGdAvjnayEmisphzoprhkgmwztgzexhzcZeRiqzecrsdFgpvtk", PH) ;
            p322.param_id_SET("Vo", PH) ;
            p322.param_count = (ushort)(ushort)61211;
            p322.param_index = (ushort)(ushort)29363;
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
                Debug.Assert(pack.param_id_LEN(ph) == 4);
                Debug.Assert(pack.param_id_TRY(ph).Equals("uyvj"));
                Debug.Assert(pack.param_value_LEN(ph) == 103);
                Debug.Assert(pack.param_value_TRY(ph).Equals("ftnfeqcprsdoxkwSpddxuclavkxbhLuvpkromyjcvpZqYcelgaglgjfbhriunwdeixzmsxjubruqhgrtscqximalgueIbupqmuSbjrK"));
                Debug.Assert(pack.target_component == (byte)(byte)171);
                Debug.Assert(pack.target_system == (byte)(byte)92);
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_value_SET("ftnfeqcprsdoxkwSpddxuclavkxbhLuvpkromyjcvpZqYcelgaglgjfbhriunwdeixzmsxjubruqhgrtscqximalgueIbupqmuSbjrK", PH) ;
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            p323.target_component = (byte)(byte)171;
            p323.target_system = (byte)(byte)92;
            p323.param_id_SET("uyvj", PH) ;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_result == PARAM_ACK.PARAM_ACK_IN_PROGRESS);
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
                Debug.Assert(pack.param_id_LEN(ph) == 2);
                Debug.Assert(pack.param_id_TRY(ph).Equals("oj"));
                Debug.Assert(pack.param_value_LEN(ph) == 14);
                Debug.Assert(pack.param_value_TRY(ph).Equals("gxqfqsaizmhrrq"));
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_value_SET("gxqfqsaizmhrrq", PH) ;
            p324.param_result = PARAM_ACK.PARAM_ACK_IN_PROGRESS;
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8;
            p324.param_id_SET("oj", PH) ;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)8872944643187210871L);
                Debug.Assert(pack.max_distance == (ushort)(ushort)3350);
                Debug.Assert(pack.min_distance == (ushort)(ushort)24857);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)20286, (ushort)7054, (ushort)25776, (ushort)38126, (ushort)43236, (ushort)17874, (ushort)5497, (ushort)9706, (ushort)31759, (ushort)6002, (ushort)21717, (ushort)5988, (ushort)37143, (ushort)40506, (ushort)338, (ushort)60562, (ushort)57327, (ushort)40013, (ushort)17384, (ushort)9602, (ushort)64324, (ushort)25119, (ushort)24317, (ushort)33975, (ushort)38987, (ushort)56250, (ushort)44111, (ushort)55252, (ushort)24853, (ushort)13246, (ushort)37260, (ushort)58506, (ushort)46544, (ushort)46070, (ushort)15302, (ushort)59314, (ushort)27668, (ushort)37700, (ushort)20479, (ushort)65421, (ushort)13686, (ushort)17254, (ushort)44014, (ushort)3212, (ushort)20681, (ushort)58374, (ushort)23779, (ushort)47009, (ushort)7722, (ushort)44885, (ushort)34902, (ushort)47424, (ushort)23798, (ushort)11876, (ushort)40243, (ushort)21560, (ushort)53622, (ushort)62447, (ushort)13916, (ushort)43657, (ushort)48175, (ushort)9555, (ushort)44028, (ushort)2897, (ushort)18127, (ushort)62984, (ushort)59806, (ushort)33979, (ushort)41180, (ushort)31354, (ushort)12757, (ushort)41635}));
                Debug.Assert(pack.increment == (byte)(byte)156);
                Debug.Assert(pack.sensor_type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.distances_SET(new ushort[] {(ushort)20286, (ushort)7054, (ushort)25776, (ushort)38126, (ushort)43236, (ushort)17874, (ushort)5497, (ushort)9706, (ushort)31759, (ushort)6002, (ushort)21717, (ushort)5988, (ushort)37143, (ushort)40506, (ushort)338, (ushort)60562, (ushort)57327, (ushort)40013, (ushort)17384, (ushort)9602, (ushort)64324, (ushort)25119, (ushort)24317, (ushort)33975, (ushort)38987, (ushort)56250, (ushort)44111, (ushort)55252, (ushort)24853, (ushort)13246, (ushort)37260, (ushort)58506, (ushort)46544, (ushort)46070, (ushort)15302, (ushort)59314, (ushort)27668, (ushort)37700, (ushort)20479, (ushort)65421, (ushort)13686, (ushort)17254, (ushort)44014, (ushort)3212, (ushort)20681, (ushort)58374, (ushort)23779, (ushort)47009, (ushort)7722, (ushort)44885, (ushort)34902, (ushort)47424, (ushort)23798, (ushort)11876, (ushort)40243, (ushort)21560, (ushort)53622, (ushort)62447, (ushort)13916, (ushort)43657, (ushort)48175, (ushort)9555, (ushort)44028, (ushort)2897, (ushort)18127, (ushort)62984, (ushort)59806, (ushort)33979, (ushort)41180, (ushort)31354, (ushort)12757, (ushort)41635}, 0) ;
            p330.time_usec = (ulong)8872944643187210871L;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p330.increment = (byte)(byte)156;
            p330.max_distance = (ushort)(ushort)3350;
            p330.min_distance = (ushort)(ushort)24857;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
        }
    }
}