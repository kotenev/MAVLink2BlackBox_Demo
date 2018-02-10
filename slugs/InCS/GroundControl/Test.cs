
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
                    ulong id = id__u(value);
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
                        case MAV_CMD.MAV_CMD_DO_NOTHING:
                            id = 110;
                            break;
                        case MAV_CMD.MAV_CMD_RETURN_TO_BASE:
                            id = 111;
                            break;
                        case MAV_CMD.MAV_CMD_STOP_RETURN_TO_BASE:
                            id = 112;
                            break;
                        case MAV_CMD.MAV_CMD_TURN_LIGHT:
                            id = 113;
                            break;
                        case MAV_CMD.MAV_CMD_GET_MID_LEVEL_COMMANDS:
                            id = 114;
                            break;
                        case MAV_CMD.MAV_CMD_MIDLEVEL_STORAGE:
                            id = 115;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                            id = 116;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                            id = 117;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                            id = 118;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                            id = 119;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                            id = 120;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                            id = 121;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                            id = 122;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                            id = 123;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                            id = 124;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                            id = 125;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                            id = 126;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                            id = 127;
                            break;
                        case MAV_CMD.MAV_CMD_USER_1:
                            id = 128;
                            break;
                        case MAV_CMD.MAV_CMD_USER_2:
                            id = 129;
                            break;
                        case MAV_CMD.MAV_CMD_USER_3:
                            id = 130;
                            break;
                        case MAV_CMD.MAV_CMD_USER_4:
                            id = 131;
                            break;
                        case MAV_CMD.MAV_CMD_USER_5:
                            id = 132;
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
                        case MAV_CMD.MAV_CMD_DO_NOTHING:
                            id = 110;
                            break;
                        case MAV_CMD.MAV_CMD_RETURN_TO_BASE:
                            id = 111;
                            break;
                        case MAV_CMD.MAV_CMD_STOP_RETURN_TO_BASE:
                            id = 112;
                            break;
                        case MAV_CMD.MAV_CMD_TURN_LIGHT:
                            id = 113;
                            break;
                        case MAV_CMD.MAV_CMD_GET_MID_LEVEL_COMMANDS:
                            id = 114;
                            break;
                        case MAV_CMD.MAV_CMD_MIDLEVEL_STORAGE:
                            id = 115;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                            id = 116;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                            id = 117;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                            id = 118;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                            id = 119;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                            id = 120;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                            id = 121;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                            id = 122;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                            id = 123;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                            id = 124;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                            id = 125;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                            id = 126;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                            id = 127;
                            break;
                        case MAV_CMD.MAV_CMD_USER_1:
                            id = 128;
                            break;
                        case MAV_CMD.MAV_CMD_USER_2:
                            id = 129;
                            break;
                        case MAV_CMD.MAV_CMD_USER_3:
                            id = 130;
                            break;
                        case MAV_CMD.MAV_CMD_USER_4:
                            id = 131;
                            break;
                        case MAV_CMD.MAV_CMD_USER_5:
                            id = 132;
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
                        case MAV_CMD.MAV_CMD_DO_NOTHING:
                            id = 110;
                            break;
                        case MAV_CMD.MAV_CMD_RETURN_TO_BASE:
                            id = 111;
                            break;
                        case MAV_CMD.MAV_CMD_STOP_RETURN_TO_BASE:
                            id = 112;
                            break;
                        case MAV_CMD.MAV_CMD_TURN_LIGHT:
                            id = 113;
                            break;
                        case MAV_CMD.MAV_CMD_GET_MID_LEVEL_COMMANDS:
                            id = 114;
                            break;
                        case MAV_CMD.MAV_CMD_MIDLEVEL_STORAGE:
                            id = 115;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                            id = 116;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                            id = 117;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                            id = 118;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                            id = 119;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                            id = 120;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                            id = 121;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                            id = 122;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                            id = 123;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                            id = 124;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                            id = 125;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                            id = 126;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                            id = 127;
                            break;
                        case MAV_CMD.MAV_CMD_USER_1:
                            id = 128;
                            break;
                        case MAV_CMD.MAV_CMD_USER_2:
                            id = 129;
                            break;
                        case MAV_CMD.MAV_CMD_USER_3:
                            id = 130;
                            break;
                        case MAV_CMD.MAV_CMD_USER_4:
                            id = 131;
                            break;
                        case MAV_CMD.MAV_CMD_USER_5:
                            id = 132;
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
                        case MAV_CMD.MAV_CMD_DO_NOTHING:
                            id = 110;
                            break;
                        case MAV_CMD.MAV_CMD_RETURN_TO_BASE:
                            id = 111;
                            break;
                        case MAV_CMD.MAV_CMD_STOP_RETURN_TO_BASE:
                            id = 112;
                            break;
                        case MAV_CMD.MAV_CMD_TURN_LIGHT:
                            id = 113;
                            break;
                        case MAV_CMD.MAV_CMD_GET_MID_LEVEL_COMMANDS:
                            id = 114;
                            break;
                        case MAV_CMD.MAV_CMD_MIDLEVEL_STORAGE:
                            id = 115;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                            id = 116;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                            id = 117;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                            id = 118;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                            id = 119;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                            id = 120;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                            id = 121;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                            id = 122;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                            id = 123;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                            id = 124;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                            id = 125;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                            id = 126;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                            id = 127;
                            break;
                        case MAV_CMD.MAV_CMD_USER_1:
                            id = 128;
                            break;
                        case MAV_CMD.MAV_CMD_USER_2:
                            id = 129;
                            break;
                        case MAV_CMD.MAV_CMD_USER_3:
                            id = 130;
                            break;
                        case MAV_CMD.MAV_CMD_USER_4:
                            id = 131;
                            break;
                        case MAV_CMD.MAV_CMD_USER_5:
                            id = 132;
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
                        case MAV_CMD.MAV_CMD_DO_NOTHING:
                            id = 110;
                            break;
                        case MAV_CMD.MAV_CMD_RETURN_TO_BASE:
                            id = 111;
                            break;
                        case MAV_CMD.MAV_CMD_STOP_RETURN_TO_BASE:
                            id = 112;
                            break;
                        case MAV_CMD.MAV_CMD_TURN_LIGHT:
                            id = 113;
                            break;
                        case MAV_CMD.MAV_CMD_GET_MID_LEVEL_COMMANDS:
                            id = 114;
                            break;
                        case MAV_CMD.MAV_CMD_MIDLEVEL_STORAGE:
                            id = 115;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_PREPARE_DEPLOY:
                            id = 116;
                            break;
                        case MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY:
                            id = 117;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_1:
                            id = 118;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_2:
                            id = 119;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_3:
                            id = 120;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_4:
                            id = 121;
                            break;
                        case MAV_CMD.MAV_CMD_WAYPOINT_USER_5:
                            id = 122;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_1:
                            id = 123;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_2:
                            id = 124;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_3:
                            id = 125;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_4:
                            id = 126;
                            break;
                        case MAV_CMD.MAV_CMD_SPATIAL_USER_5:
                            id = 127;
                            break;
                        case MAV_CMD.MAV_CMD_USER_1:
                            id = 128;
                            break;
                        case MAV_CMD.MAV_CMD_USER_2:
                            id = 129;
                            break;
                        case MAV_CMD.MAV_CMD_USER_3:
                            id = 130;
                            break;
                        case MAV_CMD.MAV_CMD_USER_4:
                            id = 131;
                            break;
                        case MAV_CMD.MAV_CMD_USER_5:
                            id = 132;
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
        new class CPU_LOAD : GroundControl.CPU_LOAD
        {
            public ushort batVolt //Battery Voltage in millivolts
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte sensLoad //Sensor DSC Load
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte ctrlLoad //Control DSC Load
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }
        }
        new class SENSOR_BIAS : GroundControl.SENSOR_BIAS
        {
            public float axBias //Accelerometer X bias (m/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float ayBias //Accelerometer Y bias (m/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float azBias //Accelerometer Z bias (m/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float gxBias //Gyro X bias (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float gyBias //Gyro Y bias (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float gzBias //Gyro Z bias (rad/s)
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }
        }
        new class DIAGNOSTIC : GroundControl.DIAGNOSTIC
        {
            public float diagFl1 //Diagnostic float 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float diagFl2 //Diagnostic float 2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float diagFl3 //Diagnostic float 3
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public short diagSh1 //Diagnostic short 1
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  12, 2));}
            }

            public short diagSh2 //Diagnostic short 2
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  14, 2));}
            }

            public short diagSh3 //Diagnostic short 3
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  16, 2));}
            }
        }
        new class SLUGS_NAVIGATION : GroundControl.SLUGS_NAVIGATION
        {
            public ushort h_c //Commanded altitude in 0.1 m
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public float u_m //Measured Airspeed prior to the nav filter in m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  2, 4)));}
            }

            public float phi_c //Commanded Roll
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
            }

            public float theta_c //Commanded Pitch
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
            }

            public float psiDot_c //Commanded Turn rate
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
            }

            public float ay_body //Y component of the body acceleration
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
            }

            public float totalDist //Total Distance to Run on this leg of Navigation
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
            }

            public float dist2Go //Remaining distance to Run on this leg of Navigation
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  26, 4)));}
            }

            public byte fromWP //Origin WP
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  30, 1));}
            }

            public byte toWP //Destination WP
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  31, 1));}
            }
        }
        new class DATA_LOG : GroundControl.DATA_LOG
        {
            public float fl_1 //Log value 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float fl_2 //Log value 2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public float fl_3 //Log value 3
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float fl_4 //Log value 4
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float fl_5 //Log value 5
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float fl_6 //Log value 6
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }
        }
        new class GPS_DATE_TIME : GroundControl.GPS_DATE_TIME
        {
            public byte year //Year reported by Gps
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte month //Month reported by Gps
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte day //Day reported by Gps
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte hour //Hour reported by Gps
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte min //Min reported by Gps
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public byte sec //Sec reported by Gps
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  5, 1));}
            }

            public byte clockStat //Clock Status. See table 47 page 211 OEMStar Manual
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public byte visSat //Visible satellites reported by Gps
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
            }

            public byte useSat //Used satellites in Solution
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public byte GppGl //GPS+GLONASS satellites in Solution
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
            }

            public byte sigUsedMask //GPS and GLONASS usage mask (bit 0 GPS_used? bit_4 GLONASS_used?)
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
            }

            public byte percentUsed //Percent used GPS
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  11, 1));}
            }
        }
        new class MID_LVL_CMDS : GroundControl.MID_LVL_CMDS
        {
            public byte target //The system setting the commands
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public float hCommand //Commanded Altitude in meters
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  1, 4)));}
            }

            public float uCommand //Commanded Airspeed in m/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  5, 4)));}
            }

            public float rCommand //Commanded Turnrate in rad/s
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  9, 4)));}
            }
        }
        new class CTRL_SRFC_PT : GroundControl.CTRL_SRFC_PT
        {
            public ushort bitfieldPt //Bitfield containing the passthrough configuration, see CONTROL_SURFACE_FLAG ENUM.
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public byte target //The system setting the commands
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }
        }
        new class SLUGS_CAMERA_ORDER : GroundControl.SLUGS_CAMERA_ORDER
        {
            public byte target //The system reporting the action
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public sbyte pan //Order the mount to pan: -1 left, 0 No pan motion, +1 right
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  1, 1));}
            }

            public sbyte tilt //Order the mount to tilt: -1 down, 0 No tilt motion, +1 up
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  2, 1));}
            }

            public sbyte zoom //Order the zoom values 0 to 10
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  3, 1));}
            }

            /**
            *Orders the camera mount to move home. The other fields are ignored when this field is set. 1: move home,
            *	0 ignore*/
            public sbyte moveHome
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  4, 1));}
            }
        }
        new class CONTROL_SURFACE : GroundControl.CONTROL_SURFACE
        {
            public byte target //The system setting the commands
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte idSurface //ID control surface send 0: throttle 1: aileron 2: elevator 3: rudder
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public float mControl //Pending
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  2, 4)));}
            }

            public float bControl //Order to origin
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
            }
        }
        new class SLUGS_MOBILE_LOCATION : GroundControl.SLUGS_MOBILE_LOCATION
        {
            public byte target //The system reporting the action
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public float latitude //Mobile Latitude
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  1, 4)));}
            }

            public float longitude //Mobile Longitude
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  5, 4)));}
            }
        }
        new class SLUGS_CONFIGURATION_CAMERA : GroundControl.SLUGS_CONFIGURATION_CAMERA
        {
            public byte target //The system setting the commands
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte idOrder //ID 0: brightness 1: aperture 2: iris 3: ICR 4: backlight
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte order //1: up/on 2: down/off 3: auto/reset/no action
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }
        }
        new class ISR_LOCATION : GroundControl.ISR_LOCATION
        {
            public byte target //The system reporting the action
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public float latitude //ISR Latitude
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  1, 4)));}
            }

            public float longitude //ISR Longitude
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  5, 4)));}
            }

            public float height //ISR Height
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  9, 4)));}
            }

            public byte option1 //Option 1
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  13, 1));}
            }

            public byte option2 //Option 2
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  14, 1));}
            }

            public byte option3 //Option 3
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  15, 1));}
            }
        }
        new class VOLT_SENSOR : GroundControl.VOLT_SENSOR
        {
            public ushort voltage //Voltage in uS of PWM. 0 uS = 0V, 20 uS = 21.5V
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            /**
            *Depends on the value of r2Type (0) Current consumption in uS of PWM, 20 uS = 90Amp (1) Distance in cm
            *	(2) Distance in cm (3) Absolute valu*/
            public ushort reading2
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  2, 2));}
            }

            public byte r2Type //It is the value of reading 2: 0 - Current, 1 - Foreward Sonar, 2 - Back Sonar, 3 - RPM
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }
        }
        new class PTZ_STATUS : GroundControl.PTZ_STATUS
        {
            public byte zoom //The actual Zoom Value
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public short pan //The Pan value in 10ths of degree
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  1, 2));}
            }

            public short tilt //The Tilt value in 10ths of degree
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  3, 2));}
            }
        }
        new class UAV_STATUS : GroundControl.UAV_STATUS
        {
            public byte target //The ID system reporting the action
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public float latitude //Latitude UAV
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  1, 4)));}
            }

            public float longitude //Longitude UAV
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  5, 4)));}
            }

            public float altitude //Altitude UAV
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  9, 4)));}
            }

            public float speed //Speed UAV
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  13, 4)));}
            }

            public float course //Course UAV
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  17, 4)));}
            }
        }
        new class STATUS_GPS : GroundControl.STATUS_GPS
        {
            public ushort csFails //Number of times checksum has failed
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            /**
            *The quality indicator, 0=fix not available or invalid, 1=GPS fix, 2=C/A differential GPS, 6=Dead reckoning
            *	mode, 7=Manual input mode (fixed position), 8=Simulator mode, 9= WAAS*/
            public byte gpsQuality
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
            }

            public byte msgsType //Indicates if GN, GL or GP messages are being received
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  3, 1));}
            }

            public byte posStatus //A = data valid, V = data invalid
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  4, 1));}
            }

            public float magVar //Magnetic variation, degrees
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  5, 4)));}
            }

            /**
            *Magnetic variation direction E/W. Easterly variation (E) subtracts from True course and Westerly variation
            *	(W) adds to True cours*/
            public sbyte magDir
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  9, 1));}
            }

            /**
            *Positioning system mode indicator. A - Autonomous;D-Differential; E-Estimated (dead reckoning) mode;M-Manual
            *	input; N-Data not vali*/
            public byte modeInd
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
            }
        }
        new class NOVATEL_DIAG : GroundControl.NOVATEL_DIAG
        {
            public ushort csFails //Times the CRC has failed since boot
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public uint receiverStatus //Status Bitfield. See table 69 page 350 Novatel OEMstar Manual
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  2, 4));}
            }

            public byte timeStatus //The Time Status. See Table 8 page 27 Novatel OEMStar Manual
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  6, 1));}
            }

            public byte solStatus //solution Status. See table 44 page 197
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  7, 1));}
            }

            public byte posType //position type. See table 43 page 196
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public byte velType //velocity type. See table 43 page 196
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
            }

            public float posSolAge //Age of the position solution in seconds
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
            }
        }
        new class SENSOR_DIAG : GroundControl.SENSOR_DIAG
        {
            public float float1 //Float field 1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  0, 4)));}
            }

            public float float2 //Float field 2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  4, 4)));}
            }

            public short int1 //Int 16 field 1
            {
                get {  return (short)((short) BitUtils.get_bytes(data,  8, 2));}
            }

            public sbyte char1 //Int 8 field 1
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  10, 1));}
            }
        }
        new class BOOT : GroundControl.BOOT
        {
            public uint version //The onboard software version
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  0, 4));}
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

            public void OnCPU_LOADReceive_direct(Channel src, Inside ph, CPU_LOAD pack) {OnCPU_LOADReceive(this, ph,  pack);}
            public event CPU_LOADReceiveHandler OnCPU_LOADReceive;
            public delegate void CPU_LOADReceiveHandler(Channel src, Inside ph, CPU_LOAD pack);
            public void OnSENSOR_BIASReceive_direct(Channel src, Inside ph, SENSOR_BIAS pack) {OnSENSOR_BIASReceive(this, ph,  pack);}
            public event SENSOR_BIASReceiveHandler OnSENSOR_BIASReceive;
            public delegate void SENSOR_BIASReceiveHandler(Channel src, Inside ph, SENSOR_BIAS pack);
            public void OnDIAGNOSTICReceive_direct(Channel src, Inside ph, DIAGNOSTIC pack) {OnDIAGNOSTICReceive(this, ph,  pack);}
            public event DIAGNOSTICReceiveHandler OnDIAGNOSTICReceive;
            public delegate void DIAGNOSTICReceiveHandler(Channel src, Inside ph, DIAGNOSTIC pack);
            public void OnSLUGS_NAVIGATIONReceive_direct(Channel src, Inside ph, SLUGS_NAVIGATION pack) {OnSLUGS_NAVIGATIONReceive(this, ph,  pack);}
            public event SLUGS_NAVIGATIONReceiveHandler OnSLUGS_NAVIGATIONReceive;
            public delegate void SLUGS_NAVIGATIONReceiveHandler(Channel src, Inside ph, SLUGS_NAVIGATION pack);
            public void OnDATA_LOGReceive_direct(Channel src, Inside ph, DATA_LOG pack) {OnDATA_LOGReceive(this, ph,  pack);}
            public event DATA_LOGReceiveHandler OnDATA_LOGReceive;
            public delegate void DATA_LOGReceiveHandler(Channel src, Inside ph, DATA_LOG pack);
            public void OnGPS_DATE_TIMEReceive_direct(Channel src, Inside ph, GPS_DATE_TIME pack) {OnGPS_DATE_TIMEReceive(this, ph,  pack);}
            public event GPS_DATE_TIMEReceiveHandler OnGPS_DATE_TIMEReceive;
            public delegate void GPS_DATE_TIMEReceiveHandler(Channel src, Inside ph, GPS_DATE_TIME pack);
            public void OnMID_LVL_CMDSReceive_direct(Channel src, Inside ph, MID_LVL_CMDS pack) {OnMID_LVL_CMDSReceive(this, ph,  pack);}
            public event MID_LVL_CMDSReceiveHandler OnMID_LVL_CMDSReceive;
            public delegate void MID_LVL_CMDSReceiveHandler(Channel src, Inside ph, MID_LVL_CMDS pack);
            public void OnCTRL_SRFC_PTReceive_direct(Channel src, Inside ph, CTRL_SRFC_PT pack) {OnCTRL_SRFC_PTReceive(this, ph,  pack);}
            public event CTRL_SRFC_PTReceiveHandler OnCTRL_SRFC_PTReceive;
            public delegate void CTRL_SRFC_PTReceiveHandler(Channel src, Inside ph, CTRL_SRFC_PT pack);
            public void OnSLUGS_CAMERA_ORDERReceive_direct(Channel src, Inside ph, SLUGS_CAMERA_ORDER pack) {OnSLUGS_CAMERA_ORDERReceive(this, ph,  pack);}
            public event SLUGS_CAMERA_ORDERReceiveHandler OnSLUGS_CAMERA_ORDERReceive;
            public delegate void SLUGS_CAMERA_ORDERReceiveHandler(Channel src, Inside ph, SLUGS_CAMERA_ORDER pack);
            public void OnCONTROL_SURFACEReceive_direct(Channel src, Inside ph, CONTROL_SURFACE pack) {OnCONTROL_SURFACEReceive(this, ph,  pack);}
            public event CONTROL_SURFACEReceiveHandler OnCONTROL_SURFACEReceive;
            public delegate void CONTROL_SURFACEReceiveHandler(Channel src, Inside ph, CONTROL_SURFACE pack);
            public void OnSLUGS_MOBILE_LOCATIONReceive_direct(Channel src, Inside ph, SLUGS_MOBILE_LOCATION pack) {OnSLUGS_MOBILE_LOCATIONReceive(this, ph,  pack);}
            public event SLUGS_MOBILE_LOCATIONReceiveHandler OnSLUGS_MOBILE_LOCATIONReceive;
            public delegate void SLUGS_MOBILE_LOCATIONReceiveHandler(Channel src, Inside ph, SLUGS_MOBILE_LOCATION pack);
            public void OnSLUGS_CONFIGURATION_CAMERAReceive_direct(Channel src, Inside ph, SLUGS_CONFIGURATION_CAMERA pack) {OnSLUGS_CONFIGURATION_CAMERAReceive(this, ph,  pack);}
            public event SLUGS_CONFIGURATION_CAMERAReceiveHandler OnSLUGS_CONFIGURATION_CAMERAReceive;
            public delegate void SLUGS_CONFIGURATION_CAMERAReceiveHandler(Channel src, Inside ph, SLUGS_CONFIGURATION_CAMERA pack);
            public void OnISR_LOCATIONReceive_direct(Channel src, Inside ph, ISR_LOCATION pack) {OnISR_LOCATIONReceive(this, ph,  pack);}
            public event ISR_LOCATIONReceiveHandler OnISR_LOCATIONReceive;
            public delegate void ISR_LOCATIONReceiveHandler(Channel src, Inside ph, ISR_LOCATION pack);
            public void OnVOLT_SENSORReceive_direct(Channel src, Inside ph, VOLT_SENSOR pack) {OnVOLT_SENSORReceive(this, ph,  pack);}
            public event VOLT_SENSORReceiveHandler OnVOLT_SENSORReceive;
            public delegate void VOLT_SENSORReceiveHandler(Channel src, Inside ph, VOLT_SENSOR pack);
            public void OnPTZ_STATUSReceive_direct(Channel src, Inside ph, PTZ_STATUS pack) {OnPTZ_STATUSReceive(this, ph,  pack);}
            public event PTZ_STATUSReceiveHandler OnPTZ_STATUSReceive;
            public delegate void PTZ_STATUSReceiveHandler(Channel src, Inside ph, PTZ_STATUS pack);
            public void OnUAV_STATUSReceive_direct(Channel src, Inside ph, UAV_STATUS pack) {OnUAV_STATUSReceive(this, ph,  pack);}
            public event UAV_STATUSReceiveHandler OnUAV_STATUSReceive;
            public delegate void UAV_STATUSReceiveHandler(Channel src, Inside ph, UAV_STATUS pack);
            public void OnSTATUS_GPSReceive_direct(Channel src, Inside ph, STATUS_GPS pack) {OnSTATUS_GPSReceive(this, ph,  pack);}
            public event STATUS_GPSReceiveHandler OnSTATUS_GPSReceive;
            public delegate void STATUS_GPSReceiveHandler(Channel src, Inside ph, STATUS_GPS pack);
            public void OnNOVATEL_DIAGReceive_direct(Channel src, Inside ph, NOVATEL_DIAG pack) {OnNOVATEL_DIAGReceive(this, ph,  pack);}
            public event NOVATEL_DIAGReceiveHandler OnNOVATEL_DIAGReceive;
            public delegate void NOVATEL_DIAGReceiveHandler(Channel src, Inside ph, NOVATEL_DIAG pack);
            public void OnSENSOR_DIAGReceive_direct(Channel src, Inside ph, SENSOR_DIAG pack) {OnSENSOR_DIAGReceive(this, ph,  pack);}
            public event SENSOR_DIAGReceiveHandler OnSENSOR_DIAGReceive;
            public delegate void SENSOR_DIAGReceiveHandler(Channel src, Inside ph, SENSOR_DIAG pack);
            public void OnBOOTReceive_direct(Channel src, Inside ph, BOOT pack) {OnBOOTReceive(this, ph,  pack);}
            public event BOOTReceiveHandler OnBOOTReceive;
            public delegate void BOOTReceiveHandler(Channel src, Inside ph, BOOT pack);
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
                    case 82:
                        if(pack == null) return new SET_ATTITUDE_TARGET();
                        break;
                    case 83:
                        if(pack == null) return new ATTITUDE_TARGET();
                        break;
                    case 170:
                        if(pack == null) return new CPU_LOAD();
                        OnCPU_LOADReceive(this, ph, (CPU_LOAD) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 172:
                        if(pack == null) return new SENSOR_BIAS();
                        OnSENSOR_BIASReceive(this, ph, (SENSOR_BIAS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 173:
                        if(pack == null) return new DIAGNOSTIC();
                        OnDIAGNOSTICReceive(this, ph, (DIAGNOSTIC) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 176:
                        if(pack == null) return new SLUGS_NAVIGATION();
                        OnSLUGS_NAVIGATIONReceive(this, ph, (SLUGS_NAVIGATION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 177:
                        if(pack == null) return new DATA_LOG();
                        OnDATA_LOGReceive(this, ph, (DATA_LOG) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 179:
                        if(pack == null) return new GPS_DATE_TIME();
                        OnGPS_DATE_TIMEReceive(this, ph, (GPS_DATE_TIME) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 180:
                        if(pack == null) return new MID_LVL_CMDS();
                        OnMID_LVL_CMDSReceive(this, ph, (MID_LVL_CMDS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 181:
                        if(pack == null) return new CTRL_SRFC_PT();
                        OnCTRL_SRFC_PTReceive(this, ph, (CTRL_SRFC_PT) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 184:
                        if(pack == null) return new SLUGS_CAMERA_ORDER();
                        OnSLUGS_CAMERA_ORDERReceive(this, ph, (SLUGS_CAMERA_ORDER) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 185:
                        if(pack == null) return new CONTROL_SURFACE();
                        OnCONTROL_SURFACEReceive(this, ph, (CONTROL_SURFACE) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 186:
                        if(pack == null) return new SLUGS_MOBILE_LOCATION();
                        OnSLUGS_MOBILE_LOCATIONReceive(this, ph, (SLUGS_MOBILE_LOCATION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 188:
                        if(pack == null) return new SLUGS_CONFIGURATION_CAMERA();
                        OnSLUGS_CONFIGURATION_CAMERAReceive(this, ph, (SLUGS_CONFIGURATION_CAMERA) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 189:
                        if(pack == null) return new ISR_LOCATION();
                        OnISR_LOCATIONReceive(this, ph, (ISR_LOCATION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 191:
                        if(pack == null) return new VOLT_SENSOR();
                        OnVOLT_SENSORReceive(this, ph, (VOLT_SENSOR) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 192:
                        if(pack == null) return new PTZ_STATUS();
                        OnPTZ_STATUSReceive(this, ph, (PTZ_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 193:
                        if(pack == null) return new UAV_STATUS();
                        OnUAV_STATUSReceive(this, ph, (UAV_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 194:
                        if(pack == null) return new STATUS_GPS();
                        OnSTATUS_GPSReceive(this, ph, (STATUS_GPS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 195:
                        if(pack == null) return new NOVATEL_DIAG();
                        OnNOVATEL_DIAGReceive(this, ph, (NOVATEL_DIAG) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 196:
                        if(pack == null) return new SENSOR_DIAG();
                        OnSENSOR_DIAGReceive(this, ph, (SENSOR_DIAG) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 197:
                        if(pack == null) return new BOOT();
                        OnBOOTReceive(this, ph, (BOOT) pack);//no any host channels can receive this pack. Handle it with test channel handler
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
                Debug.Assert(pack.system_status == MAV_STATE.MAV_STATE_POWEROFF);
                Debug.Assert(pack.custom_mode == (uint)296074469U);
                Debug.Assert(pack.mavlink_version == (byte)(byte)158);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
                Debug.Assert(pack.autopilot == MAV_AUTOPILOT.MAV_AUTOPILOT_PX4);
                Debug.Assert(pack.type == MAV_TYPE.MAV_TYPE_GENERIC);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.system_status = MAV_STATE.MAV_STATE_POWEROFF;
            p0.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
            p0.type = MAV_TYPE.MAV_TYPE_GENERIC;
            p0.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_PX4;
            p0.mavlink_version = (byte)(byte)158;
            p0.custom_mode = (uint)296074469U;
            SMP_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)49291);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)6543);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)36754);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)52054);
                Debug.Assert(pack.load == (ushort)(ushort)64127);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)42057);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)56193);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 61);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY));
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)32736);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
                Debug.Assert(pack.current_battery == (short)(short)28797);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL);
            p1.battery_remaining = (sbyte)(sbyte) - 61;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.errors_count2 = (ushort)(ushort)49291;
            p1.errors_comm = (ushort)(ushort)36754;
            p1.load = (ushort)(ushort)64127;
            p1.current_battery = (short)(short)28797;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY);
            p1.errors_count4 = (ushort)(ushort)32736;
            p1.errors_count3 = (ushort)(ushort)52054;
            p1.errors_count1 = (ushort)(ushort)42057;
            p1.drop_rate_comm = (ushort)(ushort)6543;
            p1.voltage_battery = (ushort)(ushort)56193;
            SMP_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)4656843228248737413L);
                Debug.Assert(pack.time_boot_ms == (uint)969203210U);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)4656843228248737413L;
            p2.time_boot_ms = (uint)969203210U;
            SMP_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2377498796U);
                Debug.Assert(pack.type_mask == (ushort)(ushort)18926);
                Debug.Assert(pack.afz == (float) -2.3675793E38F);
                Debug.Assert(pack.afx == (float)8.729284E37F);
                Debug.Assert(pack.afy == (float) -1.3657283E38F);
                Debug.Assert(pack.vy == (float)4.5793376E37F);
                Debug.Assert(pack.z == (float)3.7067064E37F);
                Debug.Assert(pack.x == (float)2.6645247E38F);
                Debug.Assert(pack.yaw_rate == (float) -3.165457E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.vz == (float) -1.9852725E38F);
                Debug.Assert(pack.yaw == (float) -1.4061955E38F);
                Debug.Assert(pack.vx == (float)3.2152361E38F);
                Debug.Assert(pack.y == (float) -1.9025886E38F);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.afx = (float)8.729284E37F;
            p3.x = (float)2.6645247E38F;
            p3.time_boot_ms = (uint)2377498796U;
            p3.afy = (float) -1.3657283E38F;
            p3.z = (float)3.7067064E37F;
            p3.vy = (float)4.5793376E37F;
            p3.vx = (float)3.2152361E38F;
            p3.y = (float) -1.9025886E38F;
            p3.vz = (float) -1.9852725E38F;
            p3.yaw = (float) -1.4061955E38F;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p3.afz = (float) -2.3675793E38F;
            p3.type_mask = (ushort)(ushort)18926;
            p3.yaw_rate = (float) -3.165457E38F;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3938218705278483744L);
                Debug.Assert(pack.target_component == (byte)(byte)240);
                Debug.Assert(pack.target_system == (byte)(byte)41);
                Debug.Assert(pack.seq == (uint)3994065136U);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.target_component = (byte)(byte)240;
            p4.target_system = (byte)(byte)41;
            p4.seq = (uint)3994065136U;
            p4.time_usec = (ulong)3938218705278483744L;
            SMP_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.version == (byte)(byte)130);
                Debug.Assert(pack.passkey_LEN(ph) == 14);
                Debug.Assert(pack.passkey_TRY(ph).Equals("dngeBjetbbDsio"));
                Debug.Assert(pack.target_system == (byte)(byte)25);
                Debug.Assert(pack.control_request == (byte)(byte)150);
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.passkey_SET("dngeBjetbbDsio", PH) ;
            p5.control_request = (byte)(byte)150;
            p5.target_system = (byte)(byte)25;
            p5.version = (byte)(byte)130;
            SMP_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gcs_system_id == (byte)(byte)37);
                Debug.Assert(pack.ack == (byte)(byte)135);
                Debug.Assert(pack.control_request == (byte)(byte)182);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.control_request = (byte)(byte)182;
            p6.ack = (byte)(byte)135;
            p6.gcs_system_id = (byte)(byte)37;
            SMP_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 4);
                Debug.Assert(pack.key_TRY(ph).Equals("hWXp"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("hWXp", PH) ;
            SMP_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.base_mode == MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
                Debug.Assert(pack.target_system == (byte)(byte)78);
                Debug.Assert(pack.custom_mode == (uint)1990362235U);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.base_mode = MAV_MODE.MAV_MODE_STABILIZE_DISARMED;
            p11.custom_mode = (uint)1990362235U;
            p11.target_system = (byte)(byte)78;
            SMP_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 12);
                Debug.Assert(pack.param_id_TRY(ph).Equals("zckwuanvfygw"));
                Debug.Assert(pack.target_system == (byte)(byte)117);
                Debug.Assert(pack.param_index == (short)(short)21098);
                Debug.Assert(pack.target_component == (byte)(byte)107);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_component = (byte)(byte)107;
            p20.param_id_SET("zckwuanvfygw", PH) ;
            p20.param_index = (short)(short)21098;
            p20.target_system = (byte)(byte)117;
            SMP_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)185);
                Debug.Assert(pack.target_system == (byte)(byte)109);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)109;
            p21.target_component = (byte)(byte)185;
            SMP_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value == (float)1.232969E38F);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
                Debug.Assert(pack.param_count == (ushort)(ushort)38393);
                Debug.Assert(pack.param_index == (ushort)(ushort)21870);
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("WfzouIlw"));
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32;
            p22.param_id_SET("WfzouIlw", PH) ;
            p22.param_index = (ushort)(ushort)21870;
            p22.param_value = (float)1.232969E38F;
            p22.param_count = (ushort)(ushort)38393;
            SMP_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value == (float)2.9272776E38F);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32);
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("c"));
                Debug.Assert(pack.target_component == (byte)(byte)162);
                Debug.Assert(pack.target_system == (byte)(byte)251);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.target_component = (byte)(byte)162;
            p23.param_id_SET("c", PH) ;
            p23.param_value = (float)2.9272776E38F;
            p23.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32;
            p23.target_system = (byte)(byte)251;
            SMP_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)4103044493U);
                Debug.Assert(pack.eph == (ushort)(ushort)61583);
                Debug.Assert(pack.time_usec == (ulong)4253885585610638646L);
                Debug.Assert(pack.lon == (int) -766896997);
                Debug.Assert(pack.epv == (ushort)(ushort)21807);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)3522534091U);
                Debug.Assert(pack.alt == (int)1797219328);
                Debug.Assert(pack.vel == (ushort)(ushort)54577);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -986804837);
                Debug.Assert(pack.satellites_visible == (byte)(byte)66);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
                Debug.Assert(pack.lat == (int)1692181206);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)3867434960U);
                Debug.Assert(pack.cog == (ushort)(ushort)25038);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)1876058078U);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.vel_acc_SET((uint)4103044493U, PH) ;
            p24.alt_ellipsoid_SET((int) -986804837, PH) ;
            p24.eph = (ushort)(ushort)61583;
            p24.lat = (int)1692181206;
            p24.alt = (int)1797219328;
            p24.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_PPP;
            p24.epv = (ushort)(ushort)21807;
            p24.v_acc_SET((uint)1876058078U, PH) ;
            p24.hdg_acc_SET((uint)3522534091U, PH) ;
            p24.h_acc_SET((uint)3867434960U, PH) ;
            p24.vel = (ushort)(ushort)54577;
            p24.cog = (ushort)(ushort)25038;
            p24.satellites_visible = (byte)(byte)66;
            p24.time_usec = (ulong)4253885585610638646L;
            p24.lon = (int) -766896997;
            SMP_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)180, (byte)242, (byte)91, (byte)179, (byte)3, (byte)251, (byte)79, (byte)228, (byte)246, (byte)222, (byte)161, (byte)180, (byte)72, (byte)204, (byte)199, (byte)146, (byte)229, (byte)168, (byte)137, (byte)12}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)61, (byte)183, (byte)242, (byte)255, (byte)62, (byte)88, (byte)133, (byte)18, (byte)54, (byte)69, (byte)123, (byte)151, (byte)122, (byte)204, (byte)190, (byte)230, (byte)123, (byte)134, (byte)64, (byte)135}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)63, (byte)55, (byte)58, (byte)13, (byte)49, (byte)97, (byte)198, (byte)210, (byte)252, (byte)4, (byte)12, (byte)128, (byte)7, (byte)2, (byte)70, (byte)254, (byte)111, (byte)250, (byte)139, (byte)94}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)185);
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)51, (byte)163, (byte)3, (byte)13, (byte)145, (byte)166, (byte)153, (byte)140, (byte)233, (byte)185, (byte)37, (byte)157, (byte)53, (byte)250, (byte)30, (byte)36, (byte)7, (byte)4, (byte)252, (byte)96}));
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)5, (byte)29, (byte)67, (byte)187, (byte)57, (byte)213, (byte)37, (byte)163, (byte)14, (byte)182, (byte)133, (byte)154, (byte)114, (byte)71, (byte)249, (byte)93, (byte)1, (byte)6, (byte)58, (byte)253}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_prn_SET(new byte[] {(byte)51, (byte)163, (byte)3, (byte)13, (byte)145, (byte)166, (byte)153, (byte)140, (byte)233, (byte)185, (byte)37, (byte)157, (byte)53, (byte)250, (byte)30, (byte)36, (byte)7, (byte)4, (byte)252, (byte)96}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)61, (byte)183, (byte)242, (byte)255, (byte)62, (byte)88, (byte)133, (byte)18, (byte)54, (byte)69, (byte)123, (byte)151, (byte)122, (byte)204, (byte)190, (byte)230, (byte)123, (byte)134, (byte)64, (byte)135}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)63, (byte)55, (byte)58, (byte)13, (byte)49, (byte)97, (byte)198, (byte)210, (byte)252, (byte)4, (byte)12, (byte)128, (byte)7, (byte)2, (byte)70, (byte)254, (byte)111, (byte)250, (byte)139, (byte)94}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)180, (byte)242, (byte)91, (byte)179, (byte)3, (byte)251, (byte)79, (byte)228, (byte)246, (byte)222, (byte)161, (byte)180, (byte)72, (byte)204, (byte)199, (byte)146, (byte)229, (byte)168, (byte)137, (byte)12}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)5, (byte)29, (byte)67, (byte)187, (byte)57, (byte)213, (byte)37, (byte)163, (byte)14, (byte)182, (byte)133, (byte)154, (byte)114, (byte)71, (byte)249, (byte)93, (byte)1, (byte)6, (byte)58, (byte)253}, 0) ;
            p25.satellites_visible = (byte)(byte)185;
            SMP_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short)4513);
                Debug.Assert(pack.xmag == (short)(short) -63);
                Debug.Assert(pack.zmag == (short)(short) -20607);
                Debug.Assert(pack.zgyro == (short)(short) -2528);
                Debug.Assert(pack.xgyro == (short)(short) -7901);
                Debug.Assert(pack.ygyro == (short)(short)7889);
                Debug.Assert(pack.xacc == (short)(short)25801);
                Debug.Assert(pack.ymag == (short)(short) -22658);
                Debug.Assert(pack.time_boot_ms == (uint)3469157156U);
                Debug.Assert(pack.zacc == (short)(short)30840);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.ygyro = (short)(short)7889;
            p26.xgyro = (short)(short) -7901;
            p26.time_boot_ms = (uint)3469157156U;
            p26.zgyro = (short)(short) -2528;
            p26.ymag = (short)(short) -22658;
            p26.zacc = (short)(short)30840;
            p26.xmag = (short)(short) -63;
            p26.zmag = (short)(short) -20607;
            p26.xacc = (short)(short)25801;
            p26.yacc = (short)(short)4513;
            SMP_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (short)(short) -17146);
                Debug.Assert(pack.zgyro == (short)(short) -5200);
                Debug.Assert(pack.xgyro == (short)(short) -3210);
                Debug.Assert(pack.xacc == (short)(short) -31838);
                Debug.Assert(pack.zacc == (short)(short) -20377);
                Debug.Assert(pack.time_usec == (ulong)6704357853996095180L);
                Debug.Assert(pack.ymag == (short)(short) -16774);
                Debug.Assert(pack.ygyro == (short)(short)73);
                Debug.Assert(pack.yacc == (short)(short)22784);
                Debug.Assert(pack.zmag == (short)(short) -2306);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.xacc = (short)(short) -31838;
            p27.zmag = (short)(short) -2306;
            p27.zgyro = (short)(short) -5200;
            p27.xgyro = (short)(short) -3210;
            p27.ygyro = (short)(short)73;
            p27.ymag = (short)(short) -16774;
            p27.zacc = (short)(short) -20377;
            p27.yacc = (short)(short)22784;
            p27.xmag = (short)(short) -17146;
            p27.time_usec = (ulong)6704357853996095180L;
            SMP_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff1 == (short)(short)16415);
                Debug.Assert(pack.press_diff2 == (short)(short) -23591);
                Debug.Assert(pack.time_usec == (ulong)7569013984163462980L);
                Debug.Assert(pack.press_abs == (short)(short) -21373);
                Debug.Assert(pack.temperature == (short)(short)14686);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.time_usec = (ulong)7569013984163462980L;
            p28.press_abs = (short)(short) -21373;
            p28.press_diff2 = (short)(short) -23591;
            p28.temperature = (short)(short)14686;
            p28.press_diff1 = (short)(short)16415;
            SMP_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float) -2.8441611E38F);
                Debug.Assert(pack.press_diff == (float) -5.9195424E37F);
                Debug.Assert(pack.temperature == (short)(short) -30744);
                Debug.Assert(pack.time_boot_ms == (uint)1643697414U);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.press_diff = (float) -5.9195424E37F;
            p29.press_abs = (float) -2.8441611E38F;
            p29.time_boot_ms = (uint)1643697414U;
            p29.temperature = (short)(short) -30744;
            SMP_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yawspeed == (float)2.087541E38F);
                Debug.Assert(pack.pitchspeed == (float)9.046526E37F);
                Debug.Assert(pack.yaw == (float) -1.9401703E38F);
                Debug.Assert(pack.rollspeed == (float)6.017576E37F);
                Debug.Assert(pack.pitch == (float) -1.6581255E38F);
                Debug.Assert(pack.roll == (float)2.6582187E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2496001953U);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.yaw = (float) -1.9401703E38F;
            p30.time_boot_ms = (uint)2496001953U;
            p30.roll = (float)2.6582187E38F;
            p30.rollspeed = (float)6.017576E37F;
            p30.pitchspeed = (float)9.046526E37F;
            p30.pitch = (float) -1.6581255E38F;
            p30.yawspeed = (float)2.087541E38F;
            SMP_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q2 == (float)8.709208E37F);
                Debug.Assert(pack.q4 == (float) -4.5260603E37F);
                Debug.Assert(pack.pitchspeed == (float)2.8276705E38F);
                Debug.Assert(pack.q3 == (float) -7.1295494E37F);
                Debug.Assert(pack.yawspeed == (float)3.3198473E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2916999038U);
                Debug.Assert(pack.q1 == (float) -1.5441338E38F);
                Debug.Assert(pack.rollspeed == (float)3.3822567E38F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.yawspeed = (float)3.3198473E38F;
            p31.q1 = (float) -1.5441338E38F;
            p31.q4 = (float) -4.5260603E37F;
            p31.pitchspeed = (float)2.8276705E38F;
            p31.q2 = (float)8.709208E37F;
            p31.q3 = (float) -7.1295494E37F;
            p31.rollspeed = (float)3.3822567E38F;
            p31.time_boot_ms = (uint)2916999038U;
            SMP_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -4.0534403E37F);
                Debug.Assert(pack.vy == (float)2.6853756E37F);
                Debug.Assert(pack.vx == (float) -2.7555653E38F);
                Debug.Assert(pack.vz == (float) -2.046861E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2839810971U);
                Debug.Assert(pack.y == (float) -3.1347296E38F);
                Debug.Assert(pack.z == (float)8.403313E37F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.vy = (float)2.6853756E37F;
            p32.y = (float) -3.1347296E38F;
            p32.vx = (float) -2.7555653E38F;
            p32.z = (float)8.403313E37F;
            p32.x = (float) -4.0534403E37F;
            p32.time_boot_ms = (uint)2839810971U;
            p32.vz = (float) -2.046861E38F;
            SMP_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hdg == (ushort)(ushort)42784);
                Debug.Assert(pack.lat == (int) -524840132);
                Debug.Assert(pack.vy == (short)(short) -8350);
                Debug.Assert(pack.relative_alt == (int) -1973692373);
                Debug.Assert(pack.vx == (short)(short)19494);
                Debug.Assert(pack.alt == (int) -675051478);
                Debug.Assert(pack.vz == (short)(short) -22405);
                Debug.Assert(pack.time_boot_ms == (uint)1523060287U);
                Debug.Assert(pack.lon == (int) -1616180760);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.hdg = (ushort)(ushort)42784;
            p33.lon = (int) -1616180760;
            p33.relative_alt = (int) -1973692373;
            p33.lat = (int) -524840132;
            p33.vx = (short)(short)19494;
            p33.vy = (short)(short) -8350;
            p33.time_boot_ms = (uint)1523060287U;
            p33.vz = (short)(short) -22405;
            p33.alt = (int) -675051478;
            SMP_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan6_scaled == (short)(short)25627);
                Debug.Assert(pack.chan1_scaled == (short)(short)20323);
                Debug.Assert(pack.time_boot_ms == (uint)568245331U);
                Debug.Assert(pack.port == (byte)(byte)76);
                Debug.Assert(pack.rssi == (byte)(byte)22);
                Debug.Assert(pack.chan7_scaled == (short)(short)6624);
                Debug.Assert(pack.chan2_scaled == (short)(short) -18251);
                Debug.Assert(pack.chan5_scaled == (short)(short) -1461);
                Debug.Assert(pack.chan3_scaled == (short)(short) -11248);
                Debug.Assert(pack.chan4_scaled == (short)(short) -12459);
                Debug.Assert(pack.chan8_scaled == (short)(short) -2451);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan8_scaled = (short)(short) -2451;
            p34.chan5_scaled = (short)(short) -1461;
            p34.chan4_scaled = (short)(short) -12459;
            p34.chan1_scaled = (short)(short)20323;
            p34.chan6_scaled = (short)(short)25627;
            p34.rssi = (byte)(byte)22;
            p34.port = (byte)(byte)76;
            p34.chan7_scaled = (short)(short)6624;
            p34.time_boot_ms = (uint)568245331U;
            p34.chan2_scaled = (short)(short) -18251;
            p34.chan3_scaled = (short)(short) -11248;
            SMP_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2640839348U);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)8717);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)53555);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)18781);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)38235);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)7128);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)41157);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)26574);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)38072);
                Debug.Assert(pack.rssi == (byte)(byte)131);
                Debug.Assert(pack.port == (byte)(byte)3);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan8_raw = (ushort)(ushort)18781;
            p35.chan5_raw = (ushort)(ushort)7128;
            p35.chan1_raw = (ushort)(ushort)53555;
            p35.chan6_raw = (ushort)(ushort)38072;
            p35.chan7_raw = (ushort)(ushort)41157;
            p35.chan2_raw = (ushort)(ushort)26574;
            p35.chan4_raw = (ushort)(ushort)8717;
            p35.time_boot_ms = (uint)2640839348U;
            p35.chan3_raw = (ushort)(ushort)38235;
            p35.rssi = (byte)(byte)131;
            p35.port = (byte)(byte)3;
            SMP_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)13417);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)880);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)5250);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)5259);
                Debug.Assert(pack.port == (byte)(byte)32);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)33797);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)4758);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)6230);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)39144);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)64162);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)9613);
                Debug.Assert(pack.time_usec == (uint)471687223U);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)25808);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)8530);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)9094);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)32096);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)38432);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)25906);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo7_raw = (ushort)(ushort)9613;
            p36.servo1_raw = (ushort)(ushort)38432;
            p36.servo16_raw_SET((ushort)(ushort)13417, PH) ;
            p36.servo12_raw_SET((ushort)(ushort)880, PH) ;
            p36.servo6_raw = (ushort)(ushort)6230;
            p36.servo9_raw_SET((ushort)(ushort)25906, PH) ;
            p36.servo11_raw_SET((ushort)(ushort)64162, PH) ;
            p36.port = (byte)(byte)32;
            p36.servo15_raw_SET((ushort)(ushort)9094, PH) ;
            p36.servo2_raw = (ushort)(ushort)5250;
            p36.servo5_raw = (ushort)(ushort)39144;
            p36.servo8_raw = (ushort)(ushort)32096;
            p36.servo4_raw = (ushort)(ushort)4758;
            p36.servo10_raw_SET((ushort)(ushort)25808, PH) ;
            p36.servo13_raw_SET((ushort)(ushort)8530, PH) ;
            p36.time_usec = (uint)471687223U;
            p36.servo3_raw = (ushort)(ushort)33797;
            p36.servo14_raw_SET((ushort)(ushort)5259, PH) ;
            SMP_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)143);
                Debug.Assert(pack.target_component == (byte)(byte)11);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.end_index == (short)(short) -5587);
                Debug.Assert(pack.start_index == (short)(short)1931);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.end_index = (short)(short) -5587;
            p37.start_index = (short)(short)1931;
            p37.target_system = (byte)(byte)143;
            p37.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p37.target_component = (byte)(byte)11;
            SMP_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)197);
                Debug.Assert(pack.target_component == (byte)(byte)136);
                Debug.Assert(pack.start_index == (short)(short) -8063);
                Debug.Assert(pack.end_index == (short)(short) -18530);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.end_index = (short)(short) -18530;
            p38.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p38.target_system = (byte)(byte)197;
            p38.start_index = (short)(short) -8063;
            p38.target_component = (byte)(byte)136;
            SMP_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param4 == (float) -1.0480006E38F);
                Debug.Assert(pack.param1 == (float) -2.9997477E38F);
                Debug.Assert(pack.param3 == (float) -8.708806E37F);
                Debug.Assert(pack.current == (byte)(byte)226);
                Debug.Assert(pack.target_system == (byte)(byte)180);
                Debug.Assert(pack.autocontinue == (byte)(byte)13);
                Debug.Assert(pack.y == (float) -2.0073622E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_NOTHING);
                Debug.Assert(pack.seq == (ushort)(ushort)16278);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.target_component == (byte)(byte)39);
                Debug.Assert(pack.x == (float)1.6550755E38F);
                Debug.Assert(pack.z == (float) -8.883204E37F);
                Debug.Assert(pack.param2 == (float) -2.7697431E38F);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.target_component = (byte)(byte)39;
            p39.param1 = (float) -2.9997477E38F;
            p39.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p39.z = (float) -8.883204E37F;
            p39.param3 = (float) -8.708806E37F;
            p39.autocontinue = (byte)(byte)13;
            p39.frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p39.x = (float)1.6550755E38F;
            p39.current = (byte)(byte)226;
            p39.param4 = (float) -1.0480006E38F;
            p39.target_system = (byte)(byte)180;
            p39.seq = (ushort)(ushort)16278;
            p39.y = (float) -2.0073622E38F;
            p39.param2 = (float) -2.7697431E38F;
            p39.command = MAV_CMD.MAV_CMD_DO_NOTHING;
            SMP_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)123);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)131);
                Debug.Assert(pack.seq == (ushort)(ushort)12258);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_component = (byte)(byte)123;
            p40.target_system = (byte)(byte)131;
            p40.seq = (ushort)(ushort)12258;
            p40.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            SMP_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)81);
                Debug.Assert(pack.target_component == (byte)(byte)134);
                Debug.Assert(pack.seq == (ushort)(ushort)26634);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.seq = (ushort)(ushort)26634;
            p41.target_component = (byte)(byte)134;
            p41.target_system = (byte)(byte)81;
            SMP_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)51673);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)51673;
            SMP_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)140);
                Debug.Assert(pack.target_component == (byte)(byte)169);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_component = (byte)(byte)169;
            p43.target_system = (byte)(byte)140;
            p43.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            SMP_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (ushort)(ushort)42946);
                Debug.Assert(pack.target_component == (byte)(byte)116);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_system == (byte)(byte)25);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.target_system = (byte)(byte)25;
            p44.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p44.count = (ushort)(ushort)42946;
            p44.target_component = (byte)(byte)116;
            SMP_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)182);
                Debug.Assert(pack.target_component == (byte)(byte)71);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p45.target_system = (byte)(byte)182;
            p45.target_component = (byte)(byte)71;
            SMP_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)9759);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)9759;
            SMP_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == MAV_MISSION_RESULT.MAV_MISSION_DENIED);
                Debug.Assert(pack.target_system == (byte)(byte)13);
                Debug.Assert(pack.target_component == (byte)(byte)57);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.target_system = (byte)(byte)13;
            p47.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p47.target_component = (byte)(byte)57;
            p47.type = MAV_MISSION_RESULT.MAV_MISSION_DENIED;
            SMP_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int) -1610740559);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)8818943817933335252L);
                Debug.Assert(pack.longitude == (int) -1950442763);
                Debug.Assert(pack.latitude == (int)2076667827);
                Debug.Assert(pack.target_system == (byte)(byte)229);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.altitude = (int) -1610740559;
            p48.target_system = (byte)(byte)229;
            p48.time_usec_SET((ulong)8818943817933335252L, PH) ;
            p48.longitude = (int) -1950442763;
            p48.latitude = (int)2076667827;
            SMP_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int)586354657);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)9008512629321973985L);
                Debug.Assert(pack.latitude == (int)1638428304);
                Debug.Assert(pack.longitude == (int)1053210456);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.time_usec_SET((ulong)9008512629321973985L, PH) ;
            p49.altitude = (int)586354657;
            p49.latitude = (int)1638428304;
            p49.longitude = (int)1053210456;
            SMP_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (short)(short)23245);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)217);
                Debug.Assert(pack.param_value0 == (float) -2.5753196E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 9);
                Debug.Assert(pack.param_id_TRY(ph).Equals("Kgevqyaxh"));
                Debug.Assert(pack.param_value_max == (float)3.0511863E38F);
                Debug.Assert(pack.target_component == (byte)(byte)171);
                Debug.Assert(pack.target_system == (byte)(byte)222);
                Debug.Assert(pack.param_value_min == (float)1.6462577E38F);
                Debug.Assert(pack.scale == (float) -2.515884E38F);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.param_index = (short)(short)23245;
            p50.scale = (float) -2.515884E38F;
            p50.parameter_rc_channel_index = (byte)(byte)217;
            p50.param_value_min = (float)1.6462577E38F;
            p50.param_value0 = (float) -2.5753196E38F;
            p50.param_id_SET("Kgevqyaxh", PH) ;
            p50.target_component = (byte)(byte)171;
            p50.target_system = (byte)(byte)222;
            p50.param_value_max = (float)3.0511863E38F;
            SMP_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)193);
                Debug.Assert(pack.seq == (ushort)(ushort)19452);
                Debug.Assert(pack.target_component == (byte)(byte)188);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p51.target_system = (byte)(byte)193;
            p51.seq = (ushort)(ushort)19452;
            p51.target_component = (byte)(byte)188;
            SMP_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2y == (float)4.0246467E37F);
                Debug.Assert(pack.p1y == (float) -5.151307E37F);
                Debug.Assert(pack.p1z == (float)1.1591814E38F);
                Debug.Assert(pack.p2z == (float) -3.236374E38F);
                Debug.Assert(pack.p1x == (float)1.1162929E38F);
                Debug.Assert(pack.p2x == (float) -2.1396267E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.target_component == (byte)(byte)229);
                Debug.Assert(pack.target_system == (byte)(byte)182);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p1y = (float) -5.151307E37F;
            p54.p2x = (float) -2.1396267E38F;
            p54.target_component = (byte)(byte)229;
            p54.p1z = (float)1.1591814E38F;
            p54.p2z = (float) -3.236374E38F;
            p54.target_system = (byte)(byte)182;
            p54.p2y = (float)4.0246467E37F;
            p54.p1x = (float)1.1162929E38F;
            p54.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            SMP_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2y == (float)1.907944E38F);
                Debug.Assert(pack.p1z == (float) -2.678908E38F);
                Debug.Assert(pack.p2z == (float) -3.9940418E37F);
                Debug.Assert(pack.p1y == (float) -2.707648E38F);
                Debug.Assert(pack.p1x == (float) -9.333217E37F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.p2x == (float)2.651615E38F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p2x = (float)2.651615E38F;
            p55.p2z = (float) -3.9940418E37F;
            p55.p2y = (float)1.907944E38F;
            p55.p1x = (float) -9.333217E37F;
            p55.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p55.p1y = (float) -2.707648E38F;
            p55.p1z = (float) -2.678908E38F;
            SMP_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.1224837E38F, -2.1778608E38F, 2.3498186E38F, 2.7004469E38F}));
                Debug.Assert(pack.time_usec == (ulong)2974963161611643419L);
                Debug.Assert(pack.yawspeed == (float) -3.270026E38F);
                Debug.Assert(pack.rollspeed == (float)3.4022091E38F);
                Debug.Assert(pack.pitchspeed == (float)2.8010726E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-1.1714038E38F, 1.0220768E38F, 2.0696564E38F, 2.2193627E38F, 6.5893087E37F, -7.4285907E37F, 3.1409177E37F, -3.3242218E37F, 2.3997413E38F}));
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.pitchspeed = (float)2.8010726E38F;
            p61.rollspeed = (float)3.4022091E38F;
            p61.time_usec = (ulong)2974963161611643419L;
            p61.yawspeed = (float) -3.270026E38F;
            p61.covariance_SET(new float[] {-1.1714038E38F, 1.0220768E38F, 2.0696564E38F, 2.2193627E38F, 6.5893087E37F, -7.4285907E37F, 3.1409177E37F, -3.3242218E37F, 2.3997413E38F}, 0) ;
            p61.q_SET(new float[] {-1.1224837E38F, -2.1778608E38F, 2.3498186E38F, 2.7004469E38F}, 0) ;
            SMP_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_bearing == (short)(short)6628);
                Debug.Assert(pack.aspd_error == (float) -2.6900505E38F);
                Debug.Assert(pack.nav_pitch == (float) -1.5782919E38F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)21122);
                Debug.Assert(pack.nav_bearing == (short)(short) -3854);
                Debug.Assert(pack.nav_roll == (float) -2.2642722E38F);
                Debug.Assert(pack.xtrack_error == (float)3.3305568E38F);
                Debug.Assert(pack.alt_error == (float)3.219126E38F);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.aspd_error = (float) -2.6900505E38F;
            p62.xtrack_error = (float)3.3305568E38F;
            p62.target_bearing = (short)(short)6628;
            p62.alt_error = (float)3.219126E38F;
            p62.nav_pitch = (float) -1.5782919E38F;
            p62.nav_bearing = (short)(short) -3854;
            p62.nav_roll = (float) -2.2642722E38F;
            p62.wp_dist = (ushort)(ushort)21122;
            SMP_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.9711395E38F, -1.804724E38F, 1.2545098E38F, 5.228298E37F, -2.341027E38F, 3.3878424E37F, 6.9018317E37F, -3.6338642E37F, 3.1602736E38F, 2.0804396E38F, -3.640478E37F, 3.172882E38F, 3.160152E38F, -3.2654994E38F, 1.888452E38F, 1.0330394E38F, 4.7907417E37F, 3.467939E37F, -1.5818848E38F, -1.2267193E38F, 2.5748513E38F, -2.5008049E38F, 7.1798726E37F, 2.0019517E38F, -1.8176887E38F, -2.2501313E38F, -3.3555437E38F, -1.6636233E38F, -2.5516764E38F, -3.1285363E38F, -8.036756E37F, -2.693989E38F, -2.1512201E38F, -2.5285226E38F, 1.9061045E37F, 1.2658461E38F}));
                Debug.Assert(pack.time_usec == (ulong)5610820651086063716L);
                Debug.Assert(pack.vx == (float) -3.0570735E38F);
                Debug.Assert(pack.alt == (int) -1171743334);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
                Debug.Assert(pack.vy == (float)3.1932796E38F);
                Debug.Assert(pack.lat == (int)1745220475);
                Debug.Assert(pack.vz == (float) -1.5425322E38F);
                Debug.Assert(pack.relative_alt == (int) -1697856472);
                Debug.Assert(pack.lon == (int) -525073417);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.alt = (int) -1171743334;
            p63.lon = (int) -525073417;
            p63.lat = (int)1745220475;
            p63.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
            p63.covariance_SET(new float[] {1.9711395E38F, -1.804724E38F, 1.2545098E38F, 5.228298E37F, -2.341027E38F, 3.3878424E37F, 6.9018317E37F, -3.6338642E37F, 3.1602736E38F, 2.0804396E38F, -3.640478E37F, 3.172882E38F, 3.160152E38F, -3.2654994E38F, 1.888452E38F, 1.0330394E38F, 4.7907417E37F, 3.467939E37F, -1.5818848E38F, -1.2267193E38F, 2.5748513E38F, -2.5008049E38F, 7.1798726E37F, 2.0019517E38F, -1.8176887E38F, -2.2501313E38F, -3.3555437E38F, -1.6636233E38F, -2.5516764E38F, -3.1285363E38F, -8.036756E37F, -2.693989E38F, -2.1512201E38F, -2.5285226E38F, 1.9061045E37F, 1.2658461E38F}, 0) ;
            p63.relative_alt = (int) -1697856472;
            p63.time_usec = (ulong)5610820651086063716L;
            p63.vz = (float) -1.5425322E38F;
            p63.vy = (float)3.1932796E38F;
            p63.vx = (float) -3.0570735E38F;
            SMP_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.az == (float)3.1697887E38F);
                Debug.Assert(pack.vz == (float) -1.6943033E38F);
                Debug.Assert(pack.time_usec == (ulong)6669944094307538421L);
                Debug.Assert(pack.ax == (float) -6.641081E37F);
                Debug.Assert(pack.ay == (float)1.6816818E38F);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
                Debug.Assert(pack.vy == (float) -3.0777082E38F);
                Debug.Assert(pack.z == (float)2.6595168E37F);
                Debug.Assert(pack.x == (float)2.7581177E38F);
                Debug.Assert(pack.vx == (float) -5.809471E37F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.1566042E38F, 1.1292756E38F, 1.8264471E37F, 2.9467064E38F, 1.1193922E38F, -3.8400755E37F, -2.6535472E38F, 1.2699531E38F, -2.2289875E38F, 5.8160773E37F, 2.8389938E38F, -9.776191E37F, 1.1237971E38F, -2.3803345E38F, -4.744778E37F, 6.106561E37F, -1.2927136E38F, 1.4108431E38F, -3.9705355E37F, -1.4339501E38F, -4.4635544E37F, -3.2576193E38F, 2.766908E37F, 1.1432277E38F, -1.3007546E38F, 3.1685495E37F, -3.2994463E38F, 3.3149335E38F, 1.2183928E38F, -2.1181052E38F, -3.0319675E38F, 2.670327E38F, 1.5576447E38F, 1.2906192E38F, 1.689903E38F, -2.0645672E38F, -6.972835E37F, -1.1778665E38F, -4.0151966E37F, -9.494712E37F, 2.6135909E38F, -1.9824378E38F, 2.2846735E38F, -2.8560163E37F, -2.9882725E38F}));
                Debug.Assert(pack.y == (float)1.0489843E38F);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.y = (float)1.0489843E38F;
            p64.vy = (float) -3.0777082E38F;
            p64.vz = (float) -1.6943033E38F;
            p64.time_usec = (ulong)6669944094307538421L;
            p64.covariance_SET(new float[] {1.1566042E38F, 1.1292756E38F, 1.8264471E37F, 2.9467064E38F, 1.1193922E38F, -3.8400755E37F, -2.6535472E38F, 1.2699531E38F, -2.2289875E38F, 5.8160773E37F, 2.8389938E38F, -9.776191E37F, 1.1237971E38F, -2.3803345E38F, -4.744778E37F, 6.106561E37F, -1.2927136E38F, 1.4108431E38F, -3.9705355E37F, -1.4339501E38F, -4.4635544E37F, -3.2576193E38F, 2.766908E37F, 1.1432277E38F, -1.3007546E38F, 3.1685495E37F, -3.2994463E38F, 3.3149335E38F, 1.2183928E38F, -2.1181052E38F, -3.0319675E38F, 2.670327E38F, 1.5576447E38F, 1.2906192E38F, 1.689903E38F, -2.0645672E38F, -6.972835E37F, -1.1778665E38F, -4.0151966E37F, -9.494712E37F, 2.6135909E38F, -1.9824378E38F, 2.2846735E38F, -2.8560163E37F, -2.9882725E38F}, 0) ;
            p64.ay = (float)1.6816818E38F;
            p64.vx = (float) -5.809471E37F;
            p64.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS;
            p64.az = (float)3.1697887E38F;
            p64.x = (float)2.7581177E38F;
            p64.ax = (float) -6.641081E37F;
            p64.z = (float)2.6595168E37F;
            SMP_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)50537);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)5526);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)17060);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)175);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)40068);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)6456);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)28405);
                Debug.Assert(pack.chancount == (byte)(byte)17);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)41469);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)46762);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)27733);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)6812);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)46070);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)21);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)14786);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)37042);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)48721);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)41152);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)33701);
                Debug.Assert(pack.time_boot_ms == (uint)3069960186U);
                Debug.Assert(pack.rssi == (byte)(byte)140);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan15_raw = (ushort)(ushort)21;
            p65.chan16_raw = (ushort)(ushort)37042;
            p65.chan6_raw = (ushort)(ushort)175;
            p65.chan12_raw = (ushort)(ushort)46762;
            p65.chan8_raw = (ushort)(ushort)46070;
            p65.chan4_raw = (ushort)(ushort)41469;
            p65.chan7_raw = (ushort)(ushort)41152;
            p65.chan14_raw = (ushort)(ushort)50537;
            p65.chan10_raw = (ushort)(ushort)6456;
            p65.chan17_raw = (ushort)(ushort)14786;
            p65.chan18_raw = (ushort)(ushort)17060;
            p65.chan1_raw = (ushort)(ushort)33701;
            p65.chan2_raw = (ushort)(ushort)6812;
            p65.rssi = (byte)(byte)140;
            p65.chancount = (byte)(byte)17;
            p65.chan13_raw = (ushort)(ushort)5526;
            p65.time_boot_ms = (uint)3069960186U;
            p65.chan9_raw = (ushort)(ushort)40068;
            p65.chan11_raw = (ushort)(ushort)48721;
            p65.chan5_raw = (ushort)(ushort)27733;
            p65.chan3_raw = (ushort)(ushort)28405;
            SMP_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)45);
                Debug.Assert(pack.req_stream_id == (byte)(byte)243);
                Debug.Assert(pack.target_component == (byte)(byte)154);
                Debug.Assert(pack.start_stop == (byte)(byte)182);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)64899);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.req_message_rate = (ushort)(ushort)64899;
            p66.start_stop = (byte)(byte)182;
            p66.target_component = (byte)(byte)154;
            p66.target_system = (byte)(byte)45;
            p66.req_stream_id = (byte)(byte)243;
            SMP_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.stream_id == (byte)(byte)66);
                Debug.Assert(pack.on_off == (byte)(byte)36);
                Debug.Assert(pack.message_rate == (ushort)(ushort)35812);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.stream_id = (byte)(byte)66;
            p67.message_rate = (ushort)(ushort)35812;
            p67.on_off = (byte)(byte)36;
            SMP_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target == (byte)(byte)234);
                Debug.Assert(pack.y == (short)(short)2952);
                Debug.Assert(pack.z == (short)(short) -13019);
                Debug.Assert(pack.x == (short)(short)241);
                Debug.Assert(pack.buttons == (ushort)(ushort)61654);
                Debug.Assert(pack.r == (short)(short)8837);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.target = (byte)(byte)234;
            p69.buttons = (ushort)(ushort)61654;
            p69.y = (short)(short)2952;
            p69.x = (short)(short)241;
            p69.r = (short)(short)8837;
            p69.z = (short)(short) -13019;
            SMP_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)44309);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)9536);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)30465);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)27539);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)46482);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)57330);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)31185);
                Debug.Assert(pack.target_system == (byte)(byte)168);
                Debug.Assert(pack.target_component == (byte)(byte)220);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)7752);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan2_raw = (ushort)(ushort)44309;
            p70.chan8_raw = (ushort)(ushort)30465;
            p70.chan5_raw = (ushort)(ushort)7752;
            p70.target_system = (byte)(byte)168;
            p70.chan7_raw = (ushort)(ushort)57330;
            p70.chan1_raw = (ushort)(ushort)9536;
            p70.chan6_raw = (ushort)(ushort)27539;
            p70.chan3_raw = (ushort)(ushort)31185;
            p70.chan4_raw = (ushort)(ushort)46482;
            p70.target_component = (byte)(byte)220;
            SMP_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current == (byte)(byte)159);
                Debug.Assert(pack.y == (int)1384630170);
                Debug.Assert(pack.x == (int) -1524428987);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_LOITER_TURNS);
                Debug.Assert(pack.seq == (ushort)(ushort)2720);
                Debug.Assert(pack.param2 == (float)2.2076677E38F);
                Debug.Assert(pack.z == (float) -3.920733E37F);
                Debug.Assert(pack.target_component == (byte)(byte)210);
                Debug.Assert(pack.param3 == (float)3.3854467E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)108);
                Debug.Assert(pack.target_system == (byte)(byte)137);
                Debug.Assert(pack.param1 == (float)8.608076E37F);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_NED);
                Debug.Assert(pack.param4 == (float) -2.0228302E38F);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.param1 = (float)8.608076E37F;
            p73.autocontinue = (byte)(byte)108;
            p73.command = MAV_CMD.MAV_CMD_NAV_LOITER_TURNS;
            p73.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p73.param2 = (float)2.2076677E38F;
            p73.seq = (ushort)(ushort)2720;
            p73.frame = MAV_FRAME.MAV_FRAME_BODY_NED;
            p73.param4 = (float) -2.0228302E38F;
            p73.z = (float) -3.920733E37F;
            p73.y = (int)1384630170;
            p73.current = (byte)(byte)159;
            p73.target_system = (byte)(byte)137;
            p73.target_component = (byte)(byte)210;
            p73.x = (int) -1524428987;
            p73.param3 = (float)3.3854467E38F;
            SMP_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.climb == (float)8.388973E37F);
                Debug.Assert(pack.heading == (short)(short) -12159);
                Debug.Assert(pack.groundspeed == (float)9.834119E37F);
                Debug.Assert(pack.airspeed == (float)1.2518336E38F);
                Debug.Assert(pack.alt == (float) -2.8385455E38F);
                Debug.Assert(pack.throttle == (ushort)(ushort)65434);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.alt = (float) -2.8385455E38F;
            p74.climb = (float)8.388973E37F;
            p74.airspeed = (float)1.2518336E38F;
            p74.groundspeed = (float)9.834119E37F;
            p74.throttle = (ushort)(ushort)65434;
            p74.heading = (short)(short) -12159;
            SMP_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (int) -15028967);
                Debug.Assert(pack.param4 == (float)2.0075533E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.param3 == (float)8.218694E37F);
                Debug.Assert(pack.target_system == (byte)(byte)65);
                Debug.Assert(pack.z == (float)1.6646255E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)46);
                Debug.Assert(pack.x == (int) -1540578074);
                Debug.Assert(pack.param1 == (float)8.536618E37F);
                Debug.Assert(pack.current == (byte)(byte)248);
                Debug.Assert(pack.param2 == (float)2.6880681E38F);
                Debug.Assert(pack.target_component == (byte)(byte)173);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.target_component = (byte)(byte)173;
            p75.param1 = (float)8.536618E37F;
            p75.y = (int) -15028967;
            p75.x = (int) -1540578074;
            p75.param4 = (float)2.0075533E38F;
            p75.command = MAV_CMD.MAV_CMD_DO_CONTROL_VIDEO;
            p75.current = (byte)(byte)248;
            p75.param3 = (float)8.218694E37F;
            p75.autocontinue = (byte)(byte)46;
            p75.z = (float)1.6646255E38F;
            p75.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p75.param2 = (float)2.6880681E38F;
            p75.target_system = (byte)(byte)65;
            SMP_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param2 == (float) -1.1159468E38F);
                Debug.Assert(pack.target_component == (byte)(byte)92);
                Debug.Assert(pack.param3 == (float)3.600634E37F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_CONDITION_LAST);
                Debug.Assert(pack.param7 == (float) -3.2890335E38F);
                Debug.Assert(pack.param5 == (float) -8.268554E37F);
                Debug.Assert(pack.param4 == (float) -5.434697E36F);
                Debug.Assert(pack.confirmation == (byte)(byte)144);
                Debug.Assert(pack.param6 == (float)1.0279465E38F);
                Debug.Assert(pack.param1 == (float)1.8902163E38F);
                Debug.Assert(pack.target_system == (byte)(byte)203);
            };
            COMMAND_LONG p76 = new COMMAND_LONG();
            PH.setPack(p76);
            p76.param3 = (float)3.600634E37F;
            p76.command = MAV_CMD.MAV_CMD_CONDITION_LAST;
            p76.param6 = (float)1.0279465E38F;
            p76.target_system = (byte)(byte)203;
            p76.param5 = (float) -8.268554E37F;
            p76.confirmation = (byte)(byte)144;
            p76.param7 = (float) -3.2890335E38F;
            p76.param4 = (float) -5.434697E36F;
            p76.param1 = (float)1.8902163E38F;
            p76.param2 = (float) -1.1159468E38F;
            p76.target_component = (byte)(byte)92;
            SMP_TEST_CH.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result == MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)161);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_PATHPLANNING);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)228);
                Debug.Assert(pack.result_param2_TRY(ph) == (int) -1336811872);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)47);
            };
            COMMAND_ACK p77 = new COMMAND_ACK();
            PH.setPack(p77);
            p77.progress_SET((byte)(byte)228, PH) ;
            p77.target_system_SET((byte)(byte)161, PH) ;
            p77.command = MAV_CMD.MAV_CMD_NAV_PATHPLANNING;
            p77.result_param2_SET((int) -1336811872, PH) ;
            p77.result = MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED;
            p77.target_component_SET((byte)(byte)47, PH) ;
            SMP_TEST_CH.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -2.6613941E38F);
                Debug.Assert(pack.mode_switch == (byte)(byte)87);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)97);
                Debug.Assert(pack.time_boot_ms == (uint)2892222807U);
                Debug.Assert(pack.roll == (float)6.7064315E37F);
                Debug.Assert(pack.thrust == (float)1.5525174E38F);
                Debug.Assert(pack.pitch == (float) -1.5957788E38F);
            };
            MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.roll = (float)6.7064315E37F;
            p81.yaw = (float) -2.6613941E38F;
            p81.thrust = (float)1.5525174E38F;
            p81.time_boot_ms = (uint)2892222807U;
            p81.manual_override_switch = (byte)(byte)97;
            p81.mode_switch = (byte)(byte)87;
            p81.pitch = (float) -1.5957788E38F;
            SMP_TEST_CH.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.thrust == (float) -1.1026127E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)146);
                Debug.Assert(pack.body_pitch_rate == (float) -2.7638712E38F);
                Debug.Assert(pack.body_yaw_rate == (float)1.6641946E38F);
                Debug.Assert(pack.body_roll_rate == (float)9.280702E37F);
                Debug.Assert(pack.target_component == (byte)(byte)153);
                Debug.Assert(pack.q.SequenceEqual(new float[] {8.2792365E36F, 9.516262E37F, -3.1294515E38F, 3.412985E37F}));
                Debug.Assert(pack.target_system == (byte)(byte)21);
                Debug.Assert(pack.time_boot_ms == (uint)1581071161U);
            };
            SET_ATTITUDE_TARGET p82 = new SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.body_yaw_rate = (float)1.6641946E38F;
            p82.target_component = (byte)(byte)153;
            p82.q_SET(new float[] {8.2792365E36F, 9.516262E37F, -3.1294515E38F, 3.412985E37F}, 0) ;
            p82.type_mask = (byte)(byte)146;
            p82.thrust = (float) -1.1026127E38F;
            p82.body_roll_rate = (float)9.280702E37F;
            p82.body_pitch_rate = (float) -2.7638712E38F;
            p82.time_boot_ms = (uint)1581071161U;
            p82.target_system = (byte)(byte)21;
            SMP_TEST_CH.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.0862335E38F, -3.090145E38F, -2.7572707E38F, -2.2296392E38F}));
                Debug.Assert(pack.body_pitch_rate == (float)3.306654E38F);
                Debug.Assert(pack.thrust == (float)1.3822447E37F);
                Debug.Assert(pack.type_mask == (byte)(byte)53);
                Debug.Assert(pack.body_roll_rate == (float)2.5106084E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2677720986U);
                Debug.Assert(pack.body_yaw_rate == (float) -1.8586709E38F);
            };
            ATTITUDE_TARGET p83 = new ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.q_SET(new float[] {-3.0862335E38F, -3.090145E38F, -2.7572707E38F, -2.2296392E38F}, 0) ;
            p83.type_mask = (byte)(byte)53;
            p83.body_roll_rate = (float)2.5106084E38F;
            p83.time_boot_ms = (uint)2677720986U;
            p83.body_yaw_rate = (float) -1.8586709E38F;
            p83.body_pitch_rate = (float)3.306654E38F;
            p83.thrust = (float)1.3822447E37F;
            SMP_TEST_CH.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float)4.1271963E37F);
                Debug.Assert(pack.target_component == (byte)(byte)39);
                Debug.Assert(pack.yaw == (float)2.4311155E38F);
                Debug.Assert(pack.y == (float)2.881423E38F);
                Debug.Assert(pack.vy == (float) -3.210904E38F);
                Debug.Assert(pack.z == (float)2.4049794E38F);
                Debug.Assert(pack.afz == (float) -2.8191513E38F);
                Debug.Assert(pack.x == (float) -1.3650021E38F);
                Debug.Assert(pack.target_system == (byte)(byte)23);
                Debug.Assert(pack.vz == (float)2.9150068E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)20636);
                Debug.Assert(pack.afx == (float)8.857831E37F);
                Debug.Assert(pack.time_boot_ms == (uint)2908182233U);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_BODY_NED);
                Debug.Assert(pack.yaw_rate == (float)1.9399954E38F);
                Debug.Assert(pack.afy == (float)2.442182E38F);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.afz = (float) -2.8191513E38F;
            p84.yaw = (float)2.4311155E38F;
            p84.afy = (float)2.442182E38F;
            p84.y = (float)2.881423E38F;
            p84.z = (float)2.4049794E38F;
            p84.target_component = (byte)(byte)39;
            p84.vz = (float)2.9150068E38F;
            p84.target_system = (byte)(byte)23;
            p84.vy = (float) -3.210904E38F;
            p84.x = (float) -1.3650021E38F;
            p84.time_boot_ms = (uint)2908182233U;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_BODY_NED;
            p84.type_mask = (ushort)(ushort)20636;
            p84.yaw_rate = (float)1.9399954E38F;
            p84.vx = (float)4.1271963E37F;
            p84.afx = (float)8.857831E37F;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw_rate == (float) -9.497425E37F);
                Debug.Assert(pack.afz == (float)1.2147647E38F);
                Debug.Assert(pack.alt == (float)7.066517E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3127316686U);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.afy == (float) -9.602977E37F);
                Debug.Assert(pack.vz == (float) -1.6736669E38F);
                Debug.Assert(pack.vy == (float) -5.689586E37F);
                Debug.Assert(pack.lat_int == (int)150407471);
                Debug.Assert(pack.type_mask == (ushort)(ushort)49102);
                Debug.Assert(pack.target_component == (byte)(byte)180);
                Debug.Assert(pack.afx == (float)2.7581702E38F);
                Debug.Assert(pack.lon_int == (int) -913140284);
                Debug.Assert(pack.vx == (float)1.990929E38F);
                Debug.Assert(pack.target_system == (byte)(byte)104);
                Debug.Assert(pack.yaw == (float)1.8264363E37F);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.yaw = (float)1.8264363E37F;
            p86.afz = (float)1.2147647E38F;
            p86.lat_int = (int)150407471;
            p86.time_boot_ms = (uint)3127316686U;
            p86.type_mask = (ushort)(ushort)49102;
            p86.alt = (float)7.066517E37F;
            p86.yaw_rate = (float) -9.497425E37F;
            p86.target_component = (byte)(byte)180;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p86.vx = (float)1.990929E38F;
            p86.target_system = (byte)(byte)104;
            p86.afy = (float) -9.602977E37F;
            p86.afx = (float)2.7581702E38F;
            p86.vz = (float) -1.6736669E38F;
            p86.vy = (float) -5.689586E37F;
            p86.lon_int = (int) -913140284;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afz == (float) -1.6708947E38F);
                Debug.Assert(pack.vx == (float) -1.2923594E38F);
                Debug.Assert(pack.yaw_rate == (float) -5.262813E37F);
                Debug.Assert(pack.alt == (float)1.8150152E38F);
                Debug.Assert(pack.vz == (float) -2.9066253E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)35935);
                Debug.Assert(pack.time_boot_ms == (uint)2630514600U);
                Debug.Assert(pack.afx == (float) -8.588928E37F);
                Debug.Assert(pack.afy == (float)9.256725E37F);
                Debug.Assert(pack.lon_int == (int) -1092309010);
                Debug.Assert(pack.vy == (float)1.5792692E38F);
                Debug.Assert(pack.lat_int == (int)855975742);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.yaw == (float) -7.4537434E37F);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.vz = (float) -2.9066253E38F;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p87.afx = (float) -8.588928E37F;
            p87.yaw = (float) -7.4537434E37F;
            p87.afz = (float) -1.6708947E38F;
            p87.type_mask = (ushort)(ushort)35935;
            p87.afy = (float)9.256725E37F;
            p87.yaw_rate = (float) -5.262813E37F;
            p87.vx = (float) -1.2923594E38F;
            p87.time_boot_ms = (uint)2630514600U;
            p87.vy = (float)1.5792692E38F;
            p87.alt = (float)1.8150152E38F;
            p87.lat_int = (int)855975742;
            p87.lon_int = (int) -1092309010;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -5.77434E37F);
                Debug.Assert(pack.roll == (float)5.782311E37F);
                Debug.Assert(pack.z == (float)1.6998314E38F);
                Debug.Assert(pack.y == (float)2.0371376E38F);
                Debug.Assert(pack.x == (float)1.4341044E38F);
                Debug.Assert(pack.yaw == (float)3.2296336E38F);
                Debug.Assert(pack.time_boot_ms == (uint)872856973U);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.pitch = (float) -5.77434E37F;
            p89.yaw = (float)3.2296336E38F;
            p89.x = (float)1.4341044E38F;
            p89.time_boot_ms = (uint)872856973U;
            p89.y = (float)2.0371376E38F;
            p89.roll = (float)5.782311E37F;
            p89.z = (float)1.6998314E38F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short)13564);
                Debug.Assert(pack.yawspeed == (float) -3.1060786E38F);
                Debug.Assert(pack.roll == (float)1.1396285E38F);
                Debug.Assert(pack.yaw == (float) -1.4330618E38F);
                Debug.Assert(pack.time_usec == (ulong)7036083261441716306L);
                Debug.Assert(pack.lat == (int) -1926073333);
                Debug.Assert(pack.rollspeed == (float)1.1557907E37F);
                Debug.Assert(pack.xacc == (short)(short)24660);
                Debug.Assert(pack.pitchspeed == (float)1.1904998E38F);
                Debug.Assert(pack.zacc == (short)(short)19968);
                Debug.Assert(pack.vy == (short)(short) -14703);
                Debug.Assert(pack.vz == (short)(short)15657);
                Debug.Assert(pack.pitch == (float)3.0803747E38F);
                Debug.Assert(pack.vx == (short)(short) -12547);
                Debug.Assert(pack.lon == (int) -889417486);
                Debug.Assert(pack.alt == (int)963935682);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.xacc = (short)(short)24660;
            p90.rollspeed = (float)1.1557907E37F;
            p90.alt = (int)963935682;
            p90.zacc = (short)(short)19968;
            p90.pitch = (float)3.0803747E38F;
            p90.pitchspeed = (float)1.1904998E38F;
            p90.lat = (int) -1926073333;
            p90.time_usec = (ulong)7036083261441716306L;
            p90.yaw = (float) -1.4330618E38F;
            p90.vy = (short)(short) -14703;
            p90.yacc = (short)(short)13564;
            p90.lon = (int) -889417486;
            p90.yawspeed = (float) -3.1060786E38F;
            p90.vz = (short)(short)15657;
            p90.vx = (short)(short) -12547;
            p90.roll = (float)1.1396285E38F;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)2433989736776747270L);
                Debug.Assert(pack.nav_mode == (byte)(byte)189);
                Debug.Assert(pack.pitch_elevator == (float) -5.0659755E37F);
                Debug.Assert(pack.aux4 == (float)3.1170768E38F);
                Debug.Assert(pack.aux2 == (float) -8.76318E37F);
                Debug.Assert(pack.aux3 == (float) -9.400826E37F);
                Debug.Assert(pack.yaw_rudder == (float) -2.447577E38F);
                Debug.Assert(pack.aux1 == (float) -1.1117427E38F);
                Debug.Assert(pack.throttle == (float)1.3427414E38F);
                Debug.Assert(pack.roll_ailerons == (float)2.8080372E37F);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.pitch_elevator = (float) -5.0659755E37F;
            p91.aux3 = (float) -9.400826E37F;
            p91.aux2 = (float) -8.76318E37F;
            p91.nav_mode = (byte)(byte)189;
            p91.throttle = (float)1.3427414E38F;
            p91.roll_ailerons = (float)2.8080372E37F;
            p91.time_usec = (ulong)2433989736776747270L;
            p91.aux1 = (float) -1.1117427E38F;
            p91.mode = MAV_MODE.MAV_MODE_STABILIZE_DISARMED;
            p91.yaw_rudder = (float) -2.447577E38F;
            p91.aux4 = (float)3.1170768E38F;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)40481);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)51343);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)48087);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)40874);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)20878);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)51020);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)11415);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)244);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)37002);
                Debug.Assert(pack.time_usec == (ulong)7567532002799346120L);
                Debug.Assert(pack.rssi == (byte)(byte)69);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)39237);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)17475);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)56062);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan9_raw = (ushort)(ushort)40481;
            p92.chan6_raw = (ushort)(ushort)51343;
            p92.rssi = (byte)(byte)69;
            p92.chan5_raw = (ushort)(ushort)17475;
            p92.chan1_raw = (ushort)(ushort)244;
            p92.time_usec = (ulong)7567532002799346120L;
            p92.chan10_raw = (ushort)(ushort)39237;
            p92.chan3_raw = (ushort)(ushort)20878;
            p92.chan11_raw = (ushort)(ushort)11415;
            p92.chan7_raw = (ushort)(ushort)51020;
            p92.chan2_raw = (ushort)(ushort)56062;
            p92.chan4_raw = (ushort)(ushort)40874;
            p92.chan8_raw = (ushort)(ushort)37002;
            p92.chan12_raw = (ushort)(ushort)48087;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3562797681707106103L);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_MANUAL_DISARMED);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {5.3148873E37F, 3.2926322E38F, 1.2786741E38F, -1.1972957E38F, -2.6237574E37F, -2.2848555E37F, -3.3164705E38F, -2.0150497E38F, -3.9819988E37F, -8.917548E37F, 1.5153374E38F, 1.0449394E38F, -4.8566463E37F, 2.0647856E38F, -3.0066263E38F, 2.1098314E38F}));
                Debug.Assert(pack.flags == (ulong)2925731049236427594L);
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.time_usec = (ulong)3562797681707106103L;
            p93.flags = (ulong)2925731049236427594L;
            p93.controls_SET(new float[] {5.3148873E37F, 3.2926322E38F, 1.2786741E38F, -1.1972957E38F, -2.6237574E37F, -2.2848555E37F, -3.3164705E38F, -2.0150497E38F, -3.9819988E37F, -8.917548E37F, 1.5153374E38F, 1.0449394E38F, -4.8566463E37F, 2.0647856E38F, -3.0066263E38F, 2.1098314E38F}, 0) ;
            p93.mode = MAV_MODE.MAV_MODE_MANUAL_DISARMED;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_y == (short)(short)3169);
                Debug.Assert(pack.flow_x == (short)(short) -29568);
                Debug.Assert(pack.quality == (byte)(byte)116);
                Debug.Assert(pack.time_usec == (ulong)1237296248208075500L);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float)2.7451724E38F);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)1.90694E38F);
                Debug.Assert(pack.flow_comp_m_y == (float) -2.9909682E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)56);
                Debug.Assert(pack.ground_distance == (float)2.242054E38F);
                Debug.Assert(pack.flow_comp_m_x == (float) -3.0453182E38F);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.quality = (byte)(byte)116;
            p100.flow_comp_m_y = (float) -2.9909682E38F;
            p100.sensor_id = (byte)(byte)56;
            p100.flow_y = (short)(short)3169;
            p100.time_usec = (ulong)1237296248208075500L;
            p100.flow_rate_x_SET((float)1.90694E38F, PH) ;
            p100.flow_comp_m_x = (float) -3.0453182E38F;
            p100.flow_rate_y_SET((float)2.7451724E38F, PH) ;
            p100.ground_distance = (float)2.242054E38F;
            p100.flow_x = (short)(short) -29568;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -2.733308E38F);
                Debug.Assert(pack.roll == (float)3.2622144E38F);
                Debug.Assert(pack.yaw == (float)3.0672944E38F);
                Debug.Assert(pack.usec == (ulong)7243177089130281163L);
                Debug.Assert(pack.x == (float)1.9939431E36F);
                Debug.Assert(pack.z == (float) -3.2387708E38F);
                Debug.Assert(pack.y == (float) -3.1752074E38F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.yaw = (float)3.0672944E38F;
            p101.usec = (ulong)7243177089130281163L;
            p101.roll = (float)3.2622144E38F;
            p101.pitch = (float) -2.733308E38F;
            p101.x = (float)1.9939431E36F;
            p101.z = (float) -3.2387708E38F;
            p101.y = (float) -3.1752074E38F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -2.3869538E38F);
                Debug.Assert(pack.usec == (ulong)6596743538773243760L);
                Debug.Assert(pack.roll == (float)1.4935476E38F);
                Debug.Assert(pack.x == (float) -9.600918E37F);
                Debug.Assert(pack.yaw == (float) -5.3109946E37F);
                Debug.Assert(pack.z == (float)2.9441122E38F);
                Debug.Assert(pack.y == (float) -2.2044452E38F);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.pitch = (float) -2.3869538E38F;
            p102.z = (float)2.9441122E38F;
            p102.x = (float) -9.600918E37F;
            p102.yaw = (float) -5.3109946E37F;
            p102.roll = (float)1.4935476E38F;
            p102.y = (float) -2.2044452E38F;
            p102.usec = (ulong)6596743538773243760L;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)3.3474705E38F);
                Debug.Assert(pack.x == (float)1.074275E37F);
                Debug.Assert(pack.z == (float)1.8755412E38F);
                Debug.Assert(pack.usec == (ulong)3480785857056935834L);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.y = (float)3.3474705E38F;
            p103.usec = (ulong)3480785857056935834L;
            p103.z = (float)1.8755412E38F;
            p103.x = (float)1.074275E37F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)9.28373E37F);
                Debug.Assert(pack.roll == (float)1.1841825E37F);
                Debug.Assert(pack.z == (float) -2.0868082E38F);
                Debug.Assert(pack.y == (float)1.6491926E38F);
                Debug.Assert(pack.usec == (ulong)3683124225809442764L);
                Debug.Assert(pack.pitch == (float)2.2550553E38F);
                Debug.Assert(pack.x == (float)1.6671316E38F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.yaw = (float)9.28373E37F;
            p104.usec = (ulong)3683124225809442764L;
            p104.x = (float)1.6671316E38F;
            p104.z = (float) -2.0868082E38F;
            p104.y = (float)1.6491926E38F;
            p104.pitch = (float)2.2550553E38F;
            p104.roll = (float)1.1841825E37F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.diff_pressure == (float)1.7184949E38F);
                Debug.Assert(pack.zgyro == (float)9.714891E37F);
                Debug.Assert(pack.xacc == (float)1.3335186E38F);
                Debug.Assert(pack.xmag == (float)1.7920308E38F);
                Debug.Assert(pack.abs_pressure == (float) -1.8379828E38F);
                Debug.Assert(pack.temperature == (float) -3.0737734E38F);
                Debug.Assert(pack.yacc == (float) -1.2908968E38F);
                Debug.Assert(pack.time_usec == (ulong)1783804921349912250L);
                Debug.Assert(pack.xgyro == (float) -1.5264384E38F);
                Debug.Assert(pack.ymag == (float) -1.0654348E38F);
                Debug.Assert(pack.ygyro == (float)1.140396E37F);
                Debug.Assert(pack.pressure_alt == (float)8.488007E37F);
                Debug.Assert(pack.zacc == (float)1.4015502E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)44751);
                Debug.Assert(pack.zmag == (float)3.1641202E38F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.pressure_alt = (float)8.488007E37F;
            p105.time_usec = (ulong)1783804921349912250L;
            p105.ymag = (float) -1.0654348E38F;
            p105.temperature = (float) -3.0737734E38F;
            p105.zgyro = (float)9.714891E37F;
            p105.zacc = (float)1.4015502E38F;
            p105.ygyro = (float)1.140396E37F;
            p105.fields_updated = (ushort)(ushort)44751;
            p105.zmag = (float)3.1641202E38F;
            p105.diff_pressure = (float)1.7184949E38F;
            p105.xgyro = (float) -1.5264384E38F;
            p105.xmag = (float)1.7920308E38F;
            p105.yacc = (float) -1.2908968E38F;
            p105.abs_pressure = (float) -1.8379828E38F;
            p105.xacc = (float)1.3335186E38F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.distance == (float)3.068688E38F);
                Debug.Assert(pack.integrated_y == (float)2.0319076E38F);
                Debug.Assert(pack.integrated_zgyro == (float)1.7176142E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)1231465846U);
                Debug.Assert(pack.sensor_id == (byte)(byte)107);
                Debug.Assert(pack.integrated_x == (float)2.4191201E38F);
                Debug.Assert(pack.quality == (byte)(byte)172);
                Debug.Assert(pack.integrated_ygyro == (float)1.3314905E38F);
                Debug.Assert(pack.integration_time_us == (uint)2149397339U);
                Debug.Assert(pack.integrated_xgyro == (float)4.942596E37F);
                Debug.Assert(pack.temperature == (short)(short) -5285);
                Debug.Assert(pack.time_usec == (ulong)5009089813062805914L);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.integrated_y = (float)2.0319076E38F;
            p106.integrated_x = (float)2.4191201E38F;
            p106.temperature = (short)(short) -5285;
            p106.integrated_ygyro = (float)1.3314905E38F;
            p106.sensor_id = (byte)(byte)107;
            p106.distance = (float)3.068688E38F;
            p106.integrated_zgyro = (float)1.7176142E38F;
            p106.quality = (byte)(byte)172;
            p106.integration_time_us = (uint)2149397339U;
            p106.time_delta_distance_us = (uint)1231465846U;
            p106.integrated_xgyro = (float)4.942596E37F;
            p106.time_usec = (ulong)5009089813062805914L;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.abs_pressure == (float)6.7265947E37F);
                Debug.Assert(pack.ymag == (float)2.3380187E38F);
                Debug.Assert(pack.xgyro == (float) -8.113954E37F);
                Debug.Assert(pack.zgyro == (float) -1.5046233E38F);
                Debug.Assert(pack.diff_pressure == (float)2.7111022E38F);
                Debug.Assert(pack.fields_updated == (uint)1190838817U);
                Debug.Assert(pack.zacc == (float) -2.4457661E38F);
                Debug.Assert(pack.zmag == (float)7.2364935E37F);
                Debug.Assert(pack.pressure_alt == (float)1.0859221E38F);
                Debug.Assert(pack.yacc == (float) -2.7061983E38F);
                Debug.Assert(pack.xmag == (float) -5.0802604E37F);
                Debug.Assert(pack.temperature == (float)1.814446E38F);
                Debug.Assert(pack.xacc == (float) -3.41846E37F);
                Debug.Assert(pack.time_usec == (ulong)7153211947196189336L);
                Debug.Assert(pack.ygyro == (float) -7.912154E37F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.temperature = (float)1.814446E38F;
            p107.yacc = (float) -2.7061983E38F;
            p107.zgyro = (float) -1.5046233E38F;
            p107.zacc = (float) -2.4457661E38F;
            p107.ymag = (float)2.3380187E38F;
            p107.ygyro = (float) -7.912154E37F;
            p107.xgyro = (float) -8.113954E37F;
            p107.time_usec = (ulong)7153211947196189336L;
            p107.diff_pressure = (float)2.7111022E38F;
            p107.pressure_alt = (float)1.0859221E38F;
            p107.xmag = (float) -5.0802604E37F;
            p107.fields_updated = (uint)1190838817U;
            p107.xacc = (float) -3.41846E37F;
            p107.abs_pressure = (float)6.7265947E37F;
            p107.zmag = (float)7.2364935E37F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vd == (float)1.7733876E38F);
                Debug.Assert(pack.std_dev_horz == (float)1.186033E38F);
                Debug.Assert(pack.lat == (float)7.181184E37F);
                Debug.Assert(pack.zacc == (float)2.3398548E38F);
                Debug.Assert(pack.q1 == (float) -3.1982482E38F);
                Debug.Assert(pack.lon == (float)2.0402992E37F);
                Debug.Assert(pack.alt == (float) -1.5457293E38F);
                Debug.Assert(pack.vn == (float) -6.1556545E37F);
                Debug.Assert(pack.yacc == (float) -2.5297858E38F);
                Debug.Assert(pack.q4 == (float)3.9021853E37F);
                Debug.Assert(pack.ve == (float)1.6799091E38F);
                Debug.Assert(pack.std_dev_vert == (float)3.3804714E38F);
                Debug.Assert(pack.xacc == (float)3.0750912E38F);
                Debug.Assert(pack.q2 == (float) -1.769731E38F);
                Debug.Assert(pack.yaw == (float)2.9269277E37F);
                Debug.Assert(pack.xgyro == (float)2.56612E38F);
                Debug.Assert(pack.roll == (float)1.2386317E37F);
                Debug.Assert(pack.pitch == (float) -2.9549863E37F);
                Debug.Assert(pack.ygyro == (float)2.1134662E38F);
                Debug.Assert(pack.q3 == (float) -9.513213E37F);
                Debug.Assert(pack.zgyro == (float)3.292137E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.zacc = (float)2.3398548E38F;
            p108.q4 = (float)3.9021853E37F;
            p108.q2 = (float) -1.769731E38F;
            p108.yacc = (float) -2.5297858E38F;
            p108.std_dev_horz = (float)1.186033E38F;
            p108.xgyro = (float)2.56612E38F;
            p108.lon = (float)2.0402992E37F;
            p108.ve = (float)1.6799091E38F;
            p108.lat = (float)7.181184E37F;
            p108.std_dev_vert = (float)3.3804714E38F;
            p108.q1 = (float) -3.1982482E38F;
            p108.ygyro = (float)2.1134662E38F;
            p108.q3 = (float) -9.513213E37F;
            p108.vd = (float)1.7733876E38F;
            p108.zgyro = (float)3.292137E38F;
            p108.xacc = (float)3.0750912E38F;
            p108.pitch = (float) -2.9549863E37F;
            p108.vn = (float) -6.1556545E37F;
            p108.roll = (float)1.2386317E37F;
            p108.alt = (float) -1.5457293E38F;
            p108.yaw = (float)2.9269277E37F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rxerrors == (ushort)(ushort)39731);
                Debug.Assert(pack.rssi == (byte)(byte)143);
                Debug.Assert(pack.remrssi == (byte)(byte)139);
                Debug.Assert(pack.noise == (byte)(byte)62);
                Debug.Assert(pack.remnoise == (byte)(byte)125);
                Debug.Assert(pack.txbuf == (byte)(byte)85);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)58721);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.remnoise = (byte)(byte)125;
            p109.remrssi = (byte)(byte)139;
            p109.rssi = (byte)(byte)143;
            p109.noise = (byte)(byte)62;
            p109.rxerrors = (ushort)(ushort)39731;
            p109.txbuf = (byte)(byte)85;
            p109.fixed_ = (ushort)(ushort)58721;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)189);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)9, (byte)118, (byte)31, (byte)116, (byte)37, (byte)223, (byte)20, (byte)248, (byte)245, (byte)242, (byte)243, (byte)46, (byte)171, (byte)230, (byte)155, (byte)175, (byte)135, (byte)174, (byte)0, (byte)87, (byte)194, (byte)22, (byte)166, (byte)174, (byte)238, (byte)239, (byte)250, (byte)60, (byte)132, (byte)114, (byte)218, (byte)108, (byte)197, (byte)75, (byte)13, (byte)71, (byte)181, (byte)216, (byte)103, (byte)180, (byte)72, (byte)10, (byte)243, (byte)86, (byte)3, (byte)101, (byte)115, (byte)187, (byte)45, (byte)197, (byte)147, (byte)136, (byte)231, (byte)134, (byte)46, (byte)227, (byte)109, (byte)5, (byte)11, (byte)16, (byte)51, (byte)123, (byte)177, (byte)100, (byte)78, (byte)235, (byte)46, (byte)151, (byte)17, (byte)155, (byte)110, (byte)90, (byte)54, (byte)167, (byte)163, (byte)61, (byte)144, (byte)255, (byte)203, (byte)28, (byte)170, (byte)44, (byte)255, (byte)247, (byte)147, (byte)159, (byte)2, (byte)214, (byte)64, (byte)216, (byte)238, (byte)137, (byte)113, (byte)185, (byte)191, (byte)82, (byte)168, (byte)89, (byte)202, (byte)139, (byte)238, (byte)236, (byte)76, (byte)123, (byte)188, (byte)81, (byte)143, (byte)121, (byte)40, (byte)2, (byte)100, (byte)221, (byte)195, (byte)9, (byte)44, (byte)115, (byte)61, (byte)220, (byte)29, (byte)180, (byte)237, (byte)89, (byte)89, (byte)229, (byte)19, (byte)239, (byte)188, (byte)58, (byte)201, (byte)151, (byte)191, (byte)248, (byte)105, (byte)143, (byte)211, (byte)134, (byte)225, (byte)252, (byte)13, (byte)27, (byte)238, (byte)143, (byte)149, (byte)202, (byte)235, (byte)68, (byte)162, (byte)155, (byte)72, (byte)164, (byte)200, (byte)128, (byte)255, (byte)237, (byte)137, (byte)125, (byte)153, (byte)51, (byte)25, (byte)128, (byte)57, (byte)219, (byte)203, (byte)102, (byte)159, (byte)246, (byte)124, (byte)9, (byte)14, (byte)58, (byte)84, (byte)72, (byte)111, (byte)21, (byte)136, (byte)142, (byte)225, (byte)167, (byte)15, (byte)163, (byte)113, (byte)196, (byte)72, (byte)219, (byte)23, (byte)76, (byte)69, (byte)67, (byte)229, (byte)159, (byte)41, (byte)191, (byte)34, (byte)246, (byte)48, (byte)81, (byte)244, (byte)122, (byte)204, (byte)247, (byte)28, (byte)176, (byte)173, (byte)60, (byte)29, (byte)197, (byte)206, (byte)106, (byte)182, (byte)218, (byte)123, (byte)204, (byte)29, (byte)73, (byte)179, (byte)183, (byte)226, (byte)220, (byte)172, (byte)162, (byte)87, (byte)67, (byte)96, (byte)232, (byte)247, (byte)126, (byte)227, (byte)241, (byte)144, (byte)137, (byte)204, (byte)49, (byte)183, (byte)243, (byte)12, (byte)48, (byte)249, (byte)234, (byte)124, (byte)137, (byte)203, (byte)179, (byte)211, (byte)202, (byte)91, (byte)60, (byte)42, (byte)29, (byte)116, (byte)226, (byte)111}));
                Debug.Assert(pack.target_network == (byte)(byte)250);
                Debug.Assert(pack.target_system == (byte)(byte)229);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_system = (byte)(byte)229;
            p110.target_component = (byte)(byte)189;
            p110.target_network = (byte)(byte)250;
            p110.payload_SET(new byte[] {(byte)9, (byte)118, (byte)31, (byte)116, (byte)37, (byte)223, (byte)20, (byte)248, (byte)245, (byte)242, (byte)243, (byte)46, (byte)171, (byte)230, (byte)155, (byte)175, (byte)135, (byte)174, (byte)0, (byte)87, (byte)194, (byte)22, (byte)166, (byte)174, (byte)238, (byte)239, (byte)250, (byte)60, (byte)132, (byte)114, (byte)218, (byte)108, (byte)197, (byte)75, (byte)13, (byte)71, (byte)181, (byte)216, (byte)103, (byte)180, (byte)72, (byte)10, (byte)243, (byte)86, (byte)3, (byte)101, (byte)115, (byte)187, (byte)45, (byte)197, (byte)147, (byte)136, (byte)231, (byte)134, (byte)46, (byte)227, (byte)109, (byte)5, (byte)11, (byte)16, (byte)51, (byte)123, (byte)177, (byte)100, (byte)78, (byte)235, (byte)46, (byte)151, (byte)17, (byte)155, (byte)110, (byte)90, (byte)54, (byte)167, (byte)163, (byte)61, (byte)144, (byte)255, (byte)203, (byte)28, (byte)170, (byte)44, (byte)255, (byte)247, (byte)147, (byte)159, (byte)2, (byte)214, (byte)64, (byte)216, (byte)238, (byte)137, (byte)113, (byte)185, (byte)191, (byte)82, (byte)168, (byte)89, (byte)202, (byte)139, (byte)238, (byte)236, (byte)76, (byte)123, (byte)188, (byte)81, (byte)143, (byte)121, (byte)40, (byte)2, (byte)100, (byte)221, (byte)195, (byte)9, (byte)44, (byte)115, (byte)61, (byte)220, (byte)29, (byte)180, (byte)237, (byte)89, (byte)89, (byte)229, (byte)19, (byte)239, (byte)188, (byte)58, (byte)201, (byte)151, (byte)191, (byte)248, (byte)105, (byte)143, (byte)211, (byte)134, (byte)225, (byte)252, (byte)13, (byte)27, (byte)238, (byte)143, (byte)149, (byte)202, (byte)235, (byte)68, (byte)162, (byte)155, (byte)72, (byte)164, (byte)200, (byte)128, (byte)255, (byte)237, (byte)137, (byte)125, (byte)153, (byte)51, (byte)25, (byte)128, (byte)57, (byte)219, (byte)203, (byte)102, (byte)159, (byte)246, (byte)124, (byte)9, (byte)14, (byte)58, (byte)84, (byte)72, (byte)111, (byte)21, (byte)136, (byte)142, (byte)225, (byte)167, (byte)15, (byte)163, (byte)113, (byte)196, (byte)72, (byte)219, (byte)23, (byte)76, (byte)69, (byte)67, (byte)229, (byte)159, (byte)41, (byte)191, (byte)34, (byte)246, (byte)48, (byte)81, (byte)244, (byte)122, (byte)204, (byte)247, (byte)28, (byte)176, (byte)173, (byte)60, (byte)29, (byte)197, (byte)206, (byte)106, (byte)182, (byte)218, (byte)123, (byte)204, (byte)29, (byte)73, (byte)179, (byte)183, (byte)226, (byte)220, (byte)172, (byte)162, (byte)87, (byte)67, (byte)96, (byte)232, (byte)247, (byte)126, (byte)227, (byte)241, (byte)144, (byte)137, (byte)204, (byte)49, (byte)183, (byte)243, (byte)12, (byte)48, (byte)249, (byte)234, (byte)124, (byte)137, (byte)203, (byte)179, (byte)211, (byte)202, (byte)91, (byte)60, (byte)42, (byte)29, (byte)116, (byte)226, (byte)111}, 0) ;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tc1 == (long) -3050880388828675657L);
                Debug.Assert(pack.ts1 == (long)597429451395632566L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -3050880388828675657L;
            p111.ts1 = (long)597429451395632566L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)5299139618792617051L);
                Debug.Assert(pack.seq == (uint)2409700760U);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.seq = (uint)2409700760U;
            p112.time_usec = (ulong)5299139618792617051L;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.epv == (ushort)(ushort)52043);
                Debug.Assert(pack.alt == (int) -1610758519);
                Debug.Assert(pack.satellites_visible == (byte)(byte)20);
                Debug.Assert(pack.cog == (ushort)(ushort)8382);
                Debug.Assert(pack.vel == (ushort)(ushort)33883);
                Debug.Assert(pack.lat == (int)716517681);
                Debug.Assert(pack.vn == (short)(short)17305);
                Debug.Assert(pack.fix_type == (byte)(byte)23);
                Debug.Assert(pack.eph == (ushort)(ushort)3430);
                Debug.Assert(pack.lon == (int)1629612377);
                Debug.Assert(pack.time_usec == (ulong)2625768239183335250L);
                Debug.Assert(pack.vd == (short)(short) -4323);
                Debug.Assert(pack.ve == (short)(short)16008);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.ve = (short)(short)16008;
            p113.satellites_visible = (byte)(byte)20;
            p113.alt = (int) -1610758519;
            p113.lon = (int)1629612377;
            p113.eph = (ushort)(ushort)3430;
            p113.fix_type = (byte)(byte)23;
            p113.vel = (ushort)(ushort)33883;
            p113.cog = (ushort)(ushort)8382;
            p113.time_usec = (ulong)2625768239183335250L;
            p113.vn = (short)(short)17305;
            p113.vd = (short)(short) -4323;
            p113.lat = (int)716517681;
            p113.epv = (ushort)(ushort)52043;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_y == (float)1.7196569E38F);
                Debug.Assert(pack.integration_time_us == (uint)2473121275U);
                Debug.Assert(pack.sensor_id == (byte)(byte)138);
                Debug.Assert(pack.temperature == (short)(short) -15573);
                Debug.Assert(pack.integrated_zgyro == (float) -3.1841657E38F);
                Debug.Assert(pack.quality == (byte)(byte)224);
                Debug.Assert(pack.time_delta_distance_us == (uint)966309819U);
                Debug.Assert(pack.distance == (float)2.6375986E38F);
                Debug.Assert(pack.time_usec == (ulong)1943627791171071058L);
                Debug.Assert(pack.integrated_xgyro == (float)1.5598568E38F);
                Debug.Assert(pack.integrated_x == (float) -2.0844259E37F);
                Debug.Assert(pack.integrated_ygyro == (float)2.0180152E38F);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_delta_distance_us = (uint)966309819U;
            p114.integrated_ygyro = (float)2.0180152E38F;
            p114.integrated_xgyro = (float)1.5598568E38F;
            p114.quality = (byte)(byte)224;
            p114.integrated_y = (float)1.7196569E38F;
            p114.temperature = (short)(short) -15573;
            p114.time_usec = (ulong)1943627791171071058L;
            p114.distance = (float)2.6375986E38F;
            p114.integrated_x = (float) -2.0844259E37F;
            p114.sensor_id = (byte)(byte)138;
            p114.integrated_zgyro = (float) -3.1841657E38F;
            p114.integration_time_us = (uint)2473121275U;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (short)(short)19365);
                Debug.Assert(pack.vy == (short)(short)21715);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {3.2495765E38F, 2.2541199E38F, 1.0461072E38F, 2.9769178E38F}));
                Debug.Assert(pack.yacc == (short)(short)9471);
                Debug.Assert(pack.yawspeed == (float)3.6147222E37F);
                Debug.Assert(pack.xacc == (short)(short)538);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)5530);
                Debug.Assert(pack.alt == (int) -1839896043);
                Debug.Assert(pack.lon == (int) -1945810793);
                Debug.Assert(pack.vx == (short)(short) -3851);
                Debug.Assert(pack.rollspeed == (float) -2.1988254E38F);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)23275);
                Debug.Assert(pack.lat == (int) -20644058);
                Debug.Assert(pack.pitchspeed == (float) -1.3996045E38F);
                Debug.Assert(pack.time_usec == (ulong)2283524773552226493L);
                Debug.Assert(pack.vz == (short)(short)15714);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.ind_airspeed = (ushort)(ushort)23275;
            p115.vz = (short)(short)15714;
            p115.lat = (int) -20644058;
            p115.rollspeed = (float) -2.1988254E38F;
            p115.yawspeed = (float)3.6147222E37F;
            p115.alt = (int) -1839896043;
            p115.vy = (short)(short)21715;
            p115.lon = (int) -1945810793;
            p115.zacc = (short)(short)19365;
            p115.true_airspeed = (ushort)(ushort)5530;
            p115.vx = (short)(short) -3851;
            p115.xacc = (short)(short)538;
            p115.yacc = (short)(short)9471;
            p115.pitchspeed = (float) -1.3996045E38F;
            p115.attitude_quaternion_SET(new float[] {3.2495765E38F, 2.2541199E38F, 1.0461072E38F, 2.9769178E38F}, 0) ;
            p115.time_usec = (ulong)2283524773552226493L;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short)26637);
                Debug.Assert(pack.zmag == (short)(short)15467);
                Debug.Assert(pack.zgyro == (short)(short)25964);
                Debug.Assert(pack.xacc == (short)(short) -3908);
                Debug.Assert(pack.ygyro == (short)(short) -31707);
                Debug.Assert(pack.xgyro == (short)(short)6692);
                Debug.Assert(pack.time_boot_ms == (uint)4168609964U);
                Debug.Assert(pack.xmag == (short)(short) -28344);
                Debug.Assert(pack.zacc == (short)(short)3357);
                Debug.Assert(pack.ymag == (short)(short) -25480);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.ygyro = (short)(short) -31707;
            p116.ymag = (short)(short) -25480;
            p116.zgyro = (short)(short)25964;
            p116.xgyro = (short)(short)6692;
            p116.xacc = (short)(short) -3908;
            p116.zmag = (short)(short)15467;
            p116.zacc = (short)(short)3357;
            p116.time_boot_ms = (uint)4168609964U;
            p116.yacc = (short)(short)26637;
            p116.xmag = (short)(short) -28344;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start == (ushort)(ushort)14846);
                Debug.Assert(pack.end == (ushort)(ushort)16689);
                Debug.Assert(pack.target_system == (byte)(byte)193);
                Debug.Assert(pack.target_component == (byte)(byte)255);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.start = (ushort)(ushort)14846;
            p117.end = (ushort)(ushort)16689;
            p117.target_system = (byte)(byte)193;
            p117.target_component = (byte)(byte)255;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_log_num == (ushort)(ushort)19385);
                Debug.Assert(pack.num_logs == (ushort)(ushort)27615);
                Debug.Assert(pack.size == (uint)3134757737U);
                Debug.Assert(pack.time_utc == (uint)850399608U);
                Debug.Assert(pack.id == (ushort)(ushort)24943);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.time_utc = (uint)850399608U;
            p118.num_logs = (ushort)(ushort)27615;
            p118.last_log_num = (ushort)(ushort)19385;
            p118.size = (uint)3134757737U;
            p118.id = (ushort)(ushort)24943;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (uint)2207001778U);
                Debug.Assert(pack.target_system == (byte)(byte)106);
                Debug.Assert(pack.ofs == (uint)554756469U);
                Debug.Assert(pack.id == (ushort)(ushort)16906);
                Debug.Assert(pack.target_component == (byte)(byte)77);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)106;
            p119.ofs = (uint)554756469U;
            p119.count = (uint)2207001778U;
            p119.target_component = (byte)(byte)77;
            p119.id = (ushort)(ushort)16906;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)28011);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)86, (byte)157, (byte)174, (byte)251, (byte)5, (byte)22, (byte)45, (byte)216, (byte)94, (byte)145, (byte)8, (byte)115, (byte)224, (byte)182, (byte)192, (byte)41, (byte)203, (byte)61, (byte)120, (byte)173, (byte)252, (byte)161, (byte)159, (byte)201, (byte)155, (byte)43, (byte)130, (byte)151, (byte)38, (byte)230, (byte)176, (byte)176, (byte)111, (byte)99, (byte)147, (byte)162, (byte)92, (byte)170, (byte)178, (byte)126, (byte)109, (byte)111, (byte)154, (byte)206, (byte)242, (byte)39, (byte)12, (byte)33, (byte)17, (byte)45, (byte)117, (byte)68, (byte)255, (byte)211, (byte)114, (byte)16, (byte)6, (byte)248, (byte)136, (byte)198, (byte)159, (byte)13, (byte)179, (byte)105, (byte)230, (byte)240, (byte)226, (byte)185, (byte)119, (byte)121, (byte)14, (byte)33, (byte)200, (byte)202, (byte)114, (byte)17, (byte)50, (byte)137, (byte)132, (byte)66, (byte)141, (byte)113, (byte)147, (byte)85, (byte)245, (byte)156, (byte)130, (byte)58, (byte)88, (byte)121}));
                Debug.Assert(pack.ofs == (uint)1949827843U);
                Debug.Assert(pack.count == (byte)(byte)207);
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.data__SET(new byte[] {(byte)86, (byte)157, (byte)174, (byte)251, (byte)5, (byte)22, (byte)45, (byte)216, (byte)94, (byte)145, (byte)8, (byte)115, (byte)224, (byte)182, (byte)192, (byte)41, (byte)203, (byte)61, (byte)120, (byte)173, (byte)252, (byte)161, (byte)159, (byte)201, (byte)155, (byte)43, (byte)130, (byte)151, (byte)38, (byte)230, (byte)176, (byte)176, (byte)111, (byte)99, (byte)147, (byte)162, (byte)92, (byte)170, (byte)178, (byte)126, (byte)109, (byte)111, (byte)154, (byte)206, (byte)242, (byte)39, (byte)12, (byte)33, (byte)17, (byte)45, (byte)117, (byte)68, (byte)255, (byte)211, (byte)114, (byte)16, (byte)6, (byte)248, (byte)136, (byte)198, (byte)159, (byte)13, (byte)179, (byte)105, (byte)230, (byte)240, (byte)226, (byte)185, (byte)119, (byte)121, (byte)14, (byte)33, (byte)200, (byte)202, (byte)114, (byte)17, (byte)50, (byte)137, (byte)132, (byte)66, (byte)141, (byte)113, (byte)147, (byte)85, (byte)245, (byte)156, (byte)130, (byte)58, (byte)88, (byte)121}, 0) ;
            p120.ofs = (uint)1949827843U;
            p120.count = (byte)(byte)207;
            p120.id = (ushort)(ushort)28011;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)40);
                Debug.Assert(pack.target_component == (byte)(byte)33);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_component = (byte)(byte)33;
            p121.target_system = (byte)(byte)40;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)204);
                Debug.Assert(pack.target_component == (byte)(byte)121);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)204;
            p122.target_component = (byte)(byte)121;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)107);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)66, (byte)103, (byte)199, (byte)120, (byte)106, (byte)240, (byte)251, (byte)18, (byte)129, (byte)235, (byte)78, (byte)251, (byte)180, (byte)147, (byte)159, (byte)38, (byte)200, (byte)48, (byte)122, (byte)62, (byte)249, (byte)110, (byte)102, (byte)131, (byte)182, (byte)195, (byte)220, (byte)176, (byte)148, (byte)215, (byte)90, (byte)170, (byte)0, (byte)212, (byte)98, (byte)178, (byte)225, (byte)254, (byte)110, (byte)75, (byte)251, (byte)173, (byte)231, (byte)37, (byte)189, (byte)0, (byte)65, (byte)96, (byte)159, (byte)192, (byte)150, (byte)73, (byte)227, (byte)137, (byte)84, (byte)243, (byte)236, (byte)87, (byte)94, (byte)179, (byte)33, (byte)158, (byte)237, (byte)141, (byte)4, (byte)154, (byte)227, (byte)147, (byte)57, (byte)187, (byte)39, (byte)79, (byte)0, (byte)139, (byte)176, (byte)245, (byte)134, (byte)176, (byte)255, (byte)109, (byte)220, (byte)75, (byte)100, (byte)39, (byte)230, (byte)145, (byte)241, (byte)169, (byte)128, (byte)249, (byte)100, (byte)235, (byte)143, (byte)95, (byte)74, (byte)197, (byte)136, (byte)50, (byte)143, (byte)130, (byte)220, (byte)253, (byte)119, (byte)62, (byte)241, (byte)31, (byte)186, (byte)183, (byte)190, (byte)82}));
                Debug.Assert(pack.target_system == (byte)(byte)41);
                Debug.Assert(pack.len == (byte)(byte)106);
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.data__SET(new byte[] {(byte)66, (byte)103, (byte)199, (byte)120, (byte)106, (byte)240, (byte)251, (byte)18, (byte)129, (byte)235, (byte)78, (byte)251, (byte)180, (byte)147, (byte)159, (byte)38, (byte)200, (byte)48, (byte)122, (byte)62, (byte)249, (byte)110, (byte)102, (byte)131, (byte)182, (byte)195, (byte)220, (byte)176, (byte)148, (byte)215, (byte)90, (byte)170, (byte)0, (byte)212, (byte)98, (byte)178, (byte)225, (byte)254, (byte)110, (byte)75, (byte)251, (byte)173, (byte)231, (byte)37, (byte)189, (byte)0, (byte)65, (byte)96, (byte)159, (byte)192, (byte)150, (byte)73, (byte)227, (byte)137, (byte)84, (byte)243, (byte)236, (byte)87, (byte)94, (byte)179, (byte)33, (byte)158, (byte)237, (byte)141, (byte)4, (byte)154, (byte)227, (byte)147, (byte)57, (byte)187, (byte)39, (byte)79, (byte)0, (byte)139, (byte)176, (byte)245, (byte)134, (byte)176, (byte)255, (byte)109, (byte)220, (byte)75, (byte)100, (byte)39, (byte)230, (byte)145, (byte)241, (byte)169, (byte)128, (byte)249, (byte)100, (byte)235, (byte)143, (byte)95, (byte)74, (byte)197, (byte)136, (byte)50, (byte)143, (byte)130, (byte)220, (byte)253, (byte)119, (byte)62, (byte)241, (byte)31, (byte)186, (byte)183, (byte)190, (byte)82}, 0) ;
            p123.target_system = (byte)(byte)41;
            p123.len = (byte)(byte)106;
            p123.target_component = (byte)(byte)107;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.eph == (ushort)(ushort)38595);
                Debug.Assert(pack.dgps_age == (uint)611239715U);
                Debug.Assert(pack.lon == (int) -722088459);
                Debug.Assert(pack.satellites_visible == (byte)(byte)213);
                Debug.Assert(pack.lat == (int)1916801991);
                Debug.Assert(pack.cog == (ushort)(ushort)16550);
                Debug.Assert(pack.dgps_numch == (byte)(byte)101);
                Debug.Assert(pack.epv == (ushort)(ushort)23992);
                Debug.Assert(pack.alt == (int)762692380);
                Debug.Assert(pack.time_usec == (ulong)4598990416576897135L);
                Debug.Assert(pack.vel == (ushort)(ushort)23439);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.lat = (int)1916801991;
            p124.time_usec = (ulong)4598990416576897135L;
            p124.epv = (ushort)(ushort)23992;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS;
            p124.vel = (ushort)(ushort)23439;
            p124.alt = (int)762692380;
            p124.dgps_age = (uint)611239715U;
            p124.lon = (int) -722088459;
            p124.satellites_visible = (byte)(byte)213;
            p124.dgps_numch = (byte)(byte)101;
            p124.cog = (ushort)(ushort)16550;
            p124.eph = (ushort)(ushort)38595;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT));
                Debug.Assert(pack.Vcc == (ushort)(ushort)2403);
                Debug.Assert(pack.Vservo == (ushort)(ushort)59821);
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT);
            p125.Vcc = (ushort)(ushort)2403;
            p125.Vservo = (ushort)(ushort)59821;
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (byte)(byte)210);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING));
                Debug.Assert(pack.device == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2);
                Debug.Assert(pack.baudrate == (uint)3403170442U);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)55, (byte)94, (byte)23, (byte)247, (byte)37, (byte)20, (byte)168, (byte)45, (byte)213, (byte)17, (byte)134, (byte)36, (byte)114, (byte)87, (byte)147, (byte)70, (byte)94, (byte)74, (byte)193, (byte)29, (byte)123, (byte)177, (byte)167, (byte)2, (byte)148, (byte)106, (byte)39, (byte)180, (byte)146, (byte)16, (byte)10, (byte)105, (byte)230, (byte)192, (byte)8, (byte)59, (byte)8, (byte)80, (byte)240, (byte)153, (byte)142, (byte)182, (byte)129, (byte)15, (byte)45, (byte)194, (byte)220, (byte)127, (byte)99, (byte)163, (byte)123, (byte)145, (byte)152, (byte)138, (byte)198, (byte)223, (byte)25, (byte)20, (byte)254, (byte)22, (byte)201, (byte)58, (byte)227, (byte)117, (byte)246, (byte)62, (byte)95, (byte)233, (byte)122, (byte)56}));
                Debug.Assert(pack.timeout == (ushort)(ushort)51940);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.baudrate = (uint)3403170442U;
            p126.timeout = (ushort)(ushort)51940;
            p126.data__SET(new byte[] {(byte)55, (byte)94, (byte)23, (byte)247, (byte)37, (byte)20, (byte)168, (byte)45, (byte)213, (byte)17, (byte)134, (byte)36, (byte)114, (byte)87, (byte)147, (byte)70, (byte)94, (byte)74, (byte)193, (byte)29, (byte)123, (byte)177, (byte)167, (byte)2, (byte)148, (byte)106, (byte)39, (byte)180, (byte)146, (byte)16, (byte)10, (byte)105, (byte)230, (byte)192, (byte)8, (byte)59, (byte)8, (byte)80, (byte)240, (byte)153, (byte)142, (byte)182, (byte)129, (byte)15, (byte)45, (byte)194, (byte)220, (byte)127, (byte)99, (byte)163, (byte)123, (byte)145, (byte)152, (byte)138, (byte)198, (byte)223, (byte)25, (byte)20, (byte)254, (byte)22, (byte)201, (byte)58, (byte)227, (byte)117, (byte)246, (byte)62, (byte)95, (byte)233, (byte)122, (byte)56}, 0) ;
            p126.count = (byte)(byte)210;
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING);
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_b_mm == (int) -242123268);
                Debug.Assert(pack.iar_num_hypotheses == (int)424201824);
                Debug.Assert(pack.wn == (ushort)(ushort)2728);
                Debug.Assert(pack.tow == (uint)1451100313U);
                Debug.Assert(pack.baseline_a_mm == (int) -137982952);
                Debug.Assert(pack.time_last_baseline_ms == (uint)2179348634U);
                Debug.Assert(pack.rtk_rate == (byte)(byte)152);
                Debug.Assert(pack.nsats == (byte)(byte)218);
                Debug.Assert(pack.accuracy == (uint)416565526U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)185);
                Debug.Assert(pack.rtk_health == (byte)(byte)2);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)186);
                Debug.Assert(pack.baseline_c_mm == (int)1556827897);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.baseline_coords_type = (byte)(byte)186;
            p127.baseline_b_mm = (int) -242123268;
            p127.wn = (ushort)(ushort)2728;
            p127.accuracy = (uint)416565526U;
            p127.baseline_c_mm = (int)1556827897;
            p127.iar_num_hypotheses = (int)424201824;
            p127.rtk_health = (byte)(byte)2;
            p127.nsats = (byte)(byte)218;
            p127.time_last_baseline_ms = (uint)2179348634U;
            p127.rtk_rate = (byte)(byte)152;
            p127.rtk_receiver_id = (byte)(byte)185;
            p127.baseline_a_mm = (int) -137982952;
            p127.tow = (uint)1451100313U;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_b_mm == (int)1803280503);
                Debug.Assert(pack.accuracy == (uint)893727221U);
                Debug.Assert(pack.baseline_c_mm == (int)326020067);
                Debug.Assert(pack.iar_num_hypotheses == (int)1386678580);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)161);
                Debug.Assert(pack.rtk_rate == (byte)(byte)245);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)182);
                Debug.Assert(pack.nsats == (byte)(byte)152);
                Debug.Assert(pack.rtk_health == (byte)(byte)117);
                Debug.Assert(pack.tow == (uint)2301084683U);
                Debug.Assert(pack.wn == (ushort)(ushort)33404);
                Debug.Assert(pack.baseline_a_mm == (int) -489615704);
                Debug.Assert(pack.time_last_baseline_ms == (uint)656610759U);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.rtk_health = (byte)(byte)117;
            p128.baseline_a_mm = (int) -489615704;
            p128.tow = (uint)2301084683U;
            p128.accuracy = (uint)893727221U;
            p128.time_last_baseline_ms = (uint)656610759U;
            p128.rtk_rate = (byte)(byte)245;
            p128.baseline_b_mm = (int)1803280503;
            p128.rtk_receiver_id = (byte)(byte)161;
            p128.wn = (ushort)(ushort)33404;
            p128.nsats = (byte)(byte)152;
            p128.iar_num_hypotheses = (int)1386678580;
            p128.baseline_coords_type = (byte)(byte)182;
            p128.baseline_c_mm = (int)326020067;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (short)(short) -2946);
                Debug.Assert(pack.zacc == (short)(short) -29005);
                Debug.Assert(pack.ygyro == (short)(short) -11652);
                Debug.Assert(pack.ymag == (short)(short)20175);
                Debug.Assert(pack.zmag == (short)(short)32267);
                Debug.Assert(pack.zgyro == (short)(short) -15705);
                Debug.Assert(pack.time_boot_ms == (uint)3724342842U);
                Debug.Assert(pack.xacc == (short)(short) -31984);
                Debug.Assert(pack.yacc == (short)(short) -6086);
                Debug.Assert(pack.xmag == (short)(short) -13594);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.xacc = (short)(short) -31984;
            p129.zmag = (short)(short)32267;
            p129.yacc = (short)(short) -6086;
            p129.time_boot_ms = (uint)3724342842U;
            p129.xgyro = (short)(short) -2946;
            p129.xmag = (short)(short) -13594;
            p129.zgyro = (short)(short) -15705;
            p129.zacc = (short)(short) -29005;
            p129.ygyro = (short)(short) -11652;
            p129.ymag = (short)(short)20175;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload == (byte)(byte)132);
                Debug.Assert(pack.type == (byte)(byte)212);
                Debug.Assert(pack.width == (ushort)(ushort)45226);
                Debug.Assert(pack.height == (ushort)(ushort)49836);
                Debug.Assert(pack.size == (uint)3313152499U);
                Debug.Assert(pack.jpg_quality == (byte)(byte)226);
                Debug.Assert(pack.packets == (ushort)(ushort)30890);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.jpg_quality = (byte)(byte)226;
            p130.height = (ushort)(ushort)49836;
            p130.width = (ushort)(ushort)45226;
            p130.payload = (byte)(byte)132;
            p130.type = (byte)(byte)212;
            p130.packets = (ushort)(ushort)30890;
            p130.size = (uint)3313152499U;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)69, (byte)62, (byte)106, (byte)84, (byte)225, (byte)114, (byte)197, (byte)42, (byte)250, (byte)160, (byte)143, (byte)188, (byte)191, (byte)50, (byte)17, (byte)223, (byte)124, (byte)128, (byte)94, (byte)130, (byte)70, (byte)41, (byte)33, (byte)112, (byte)181, (byte)100, (byte)17, (byte)133, (byte)223, (byte)39, (byte)155, (byte)137, (byte)100, (byte)220, (byte)116, (byte)135, (byte)213, (byte)11, (byte)48, (byte)225, (byte)65, (byte)56, (byte)52, (byte)77, (byte)158, (byte)94, (byte)73, (byte)212, (byte)55, (byte)89, (byte)199, (byte)27, (byte)129, (byte)106, (byte)104, (byte)101, (byte)35, (byte)110, (byte)231, (byte)162, (byte)50, (byte)107, (byte)141, (byte)56, (byte)164, (byte)145, (byte)17, (byte)197, (byte)43, (byte)132, (byte)39, (byte)137, (byte)250, (byte)108, (byte)155, (byte)157, (byte)113, (byte)95, (byte)202, (byte)88, (byte)224, (byte)211, (byte)201, (byte)12, (byte)228, (byte)67, (byte)102, (byte)231, (byte)140, (byte)35, (byte)108, (byte)64, (byte)115, (byte)57, (byte)230, (byte)245, (byte)63, (byte)22, (byte)84, (byte)218, (byte)37, (byte)171, (byte)41, (byte)123, (byte)61, (byte)241, (byte)39, (byte)51, (byte)227, (byte)45, (byte)127, (byte)205, (byte)114, (byte)249, (byte)62, (byte)159, (byte)124, (byte)7, (byte)167, (byte)160, (byte)22, (byte)243, (byte)148, (byte)9, (byte)246, (byte)158, (byte)211, (byte)98, (byte)130, (byte)52, (byte)140, (byte)36, (byte)141, (byte)13, (byte)160, (byte)134, (byte)250, (byte)63, (byte)116, (byte)191, (byte)214, (byte)30, (byte)204, (byte)219, (byte)224, (byte)43, (byte)250, (byte)179, (byte)214, (byte)31, (byte)243, (byte)213, (byte)199, (byte)28, (byte)97, (byte)220, (byte)46, (byte)108, (byte)250, (byte)222, (byte)85, (byte)49, (byte)153, (byte)225, (byte)141, (byte)251, (byte)39, (byte)175, (byte)93, (byte)154, (byte)161, (byte)162, (byte)12, (byte)73, (byte)113, (byte)45, (byte)214, (byte)165, (byte)28, (byte)40, (byte)134, (byte)232, (byte)214, (byte)191, (byte)65, (byte)77, (byte)250, (byte)2, (byte)110, (byte)153, (byte)87, (byte)2, (byte)74, (byte)16, (byte)82, (byte)247, (byte)146, (byte)109, (byte)233, (byte)203, (byte)163, (byte)78, (byte)26, (byte)54, (byte)211, (byte)178, (byte)61, (byte)177, (byte)226, (byte)94, (byte)65, (byte)4, (byte)178, (byte)41, (byte)235, (byte)0, (byte)98, (byte)207, (byte)82, (byte)209, (byte)230, (byte)242, (byte)246, (byte)192, (byte)245, (byte)113, (byte)126, (byte)144, (byte)199, (byte)251, (byte)237, (byte)3, (byte)144, (byte)237, (byte)46, (byte)149, (byte)71, (byte)2, (byte)210, (byte)233, (byte)202, (byte)237, (byte)167, (byte)198, (byte)42, (byte)201, (byte)211, (byte)71, (byte)137, (byte)42, (byte)139, (byte)169, (byte)212}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)8789);
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)8789;
            p131.data__SET(new byte[] {(byte)69, (byte)62, (byte)106, (byte)84, (byte)225, (byte)114, (byte)197, (byte)42, (byte)250, (byte)160, (byte)143, (byte)188, (byte)191, (byte)50, (byte)17, (byte)223, (byte)124, (byte)128, (byte)94, (byte)130, (byte)70, (byte)41, (byte)33, (byte)112, (byte)181, (byte)100, (byte)17, (byte)133, (byte)223, (byte)39, (byte)155, (byte)137, (byte)100, (byte)220, (byte)116, (byte)135, (byte)213, (byte)11, (byte)48, (byte)225, (byte)65, (byte)56, (byte)52, (byte)77, (byte)158, (byte)94, (byte)73, (byte)212, (byte)55, (byte)89, (byte)199, (byte)27, (byte)129, (byte)106, (byte)104, (byte)101, (byte)35, (byte)110, (byte)231, (byte)162, (byte)50, (byte)107, (byte)141, (byte)56, (byte)164, (byte)145, (byte)17, (byte)197, (byte)43, (byte)132, (byte)39, (byte)137, (byte)250, (byte)108, (byte)155, (byte)157, (byte)113, (byte)95, (byte)202, (byte)88, (byte)224, (byte)211, (byte)201, (byte)12, (byte)228, (byte)67, (byte)102, (byte)231, (byte)140, (byte)35, (byte)108, (byte)64, (byte)115, (byte)57, (byte)230, (byte)245, (byte)63, (byte)22, (byte)84, (byte)218, (byte)37, (byte)171, (byte)41, (byte)123, (byte)61, (byte)241, (byte)39, (byte)51, (byte)227, (byte)45, (byte)127, (byte)205, (byte)114, (byte)249, (byte)62, (byte)159, (byte)124, (byte)7, (byte)167, (byte)160, (byte)22, (byte)243, (byte)148, (byte)9, (byte)246, (byte)158, (byte)211, (byte)98, (byte)130, (byte)52, (byte)140, (byte)36, (byte)141, (byte)13, (byte)160, (byte)134, (byte)250, (byte)63, (byte)116, (byte)191, (byte)214, (byte)30, (byte)204, (byte)219, (byte)224, (byte)43, (byte)250, (byte)179, (byte)214, (byte)31, (byte)243, (byte)213, (byte)199, (byte)28, (byte)97, (byte)220, (byte)46, (byte)108, (byte)250, (byte)222, (byte)85, (byte)49, (byte)153, (byte)225, (byte)141, (byte)251, (byte)39, (byte)175, (byte)93, (byte)154, (byte)161, (byte)162, (byte)12, (byte)73, (byte)113, (byte)45, (byte)214, (byte)165, (byte)28, (byte)40, (byte)134, (byte)232, (byte)214, (byte)191, (byte)65, (byte)77, (byte)250, (byte)2, (byte)110, (byte)153, (byte)87, (byte)2, (byte)74, (byte)16, (byte)82, (byte)247, (byte)146, (byte)109, (byte)233, (byte)203, (byte)163, (byte)78, (byte)26, (byte)54, (byte)211, (byte)178, (byte)61, (byte)177, (byte)226, (byte)94, (byte)65, (byte)4, (byte)178, (byte)41, (byte)235, (byte)0, (byte)98, (byte)207, (byte)82, (byte)209, (byte)230, (byte)242, (byte)246, (byte)192, (byte)245, (byte)113, (byte)126, (byte)144, (byte)199, (byte)251, (byte)237, (byte)3, (byte)144, (byte)237, (byte)46, (byte)149, (byte)71, (byte)2, (byte)210, (byte)233, (byte)202, (byte)237, (byte)167, (byte)198, (byte)42, (byte)201, (byte)211, (byte)71, (byte)137, (byte)42, (byte)139, (byte)169, (byte)212}, 0) ;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.orientation == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_YAW_90);
                Debug.Assert(pack.current_distance == (ushort)(ushort)5836);
                Debug.Assert(pack.covariance == (byte)(byte)33);
                Debug.Assert(pack.type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
                Debug.Assert(pack.min_distance == (ushort)(ushort)51827);
                Debug.Assert(pack.time_boot_ms == (uint)1695599852U);
                Debug.Assert(pack.id == (byte)(byte)193);
                Debug.Assert(pack.max_distance == (ushort)(ushort)45126);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.max_distance = (ushort)(ushort)45126;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_YAW_90;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN;
            p132.min_distance = (ushort)(ushort)51827;
            p132.time_boot_ms = (uint)1695599852U;
            p132.current_distance = (ushort)(ushort)5836;
            p132.covariance = (byte)(byte)33;
            p132.id = (byte)(byte)193;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)9107);
                Debug.Assert(pack.lat == (int)1475712023);
                Debug.Assert(pack.mask == (ulong)1076197605127073212L);
                Debug.Assert(pack.lon == (int)1432402131);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lon = (int)1432402131;
            p133.mask = (ulong)1076197605127073212L;
            p133.lat = (int)1475712023;
            p133.grid_spacing = (ushort)(ushort)9107;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)13036);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -30906, (short)2377, (short)3511, (short)17015, (short)7455, (short)28229, (short) -24290, (short)17023, (short) -27873, (short)25048, (short) -31631, (short) -30981, (short) -13961, (short)31761, (short) -21345, (short)29520}));
                Debug.Assert(pack.lon == (int)245556257);
                Debug.Assert(pack.gridbit == (byte)(byte)27);
                Debug.Assert(pack.lat == (int)246204547);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.grid_spacing = (ushort)(ushort)13036;
            p134.lon = (int)245556257;
            p134.data__SET(new short[] {(short) -30906, (short)2377, (short)3511, (short)17015, (short)7455, (short)28229, (short) -24290, (short)17023, (short) -27873, (short)25048, (short) -31631, (short) -30981, (short) -13961, (short)31761, (short) -21345, (short)29520}, 0) ;
            p134.gridbit = (byte)(byte)27;
            p134.lat = (int)246204547;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)928616935);
                Debug.Assert(pack.lon == (int)693768367);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)928616935;
            p135.lon = (int)693768367;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_height == (float) -1.9130787E38F);
                Debug.Assert(pack.lat == (int)463453157);
                Debug.Assert(pack.terrain_height == (float)3.4879707E36F);
                Debug.Assert(pack.loaded == (ushort)(ushort)36512);
                Debug.Assert(pack.spacing == (ushort)(ushort)59092);
                Debug.Assert(pack.lon == (int)690358844);
                Debug.Assert(pack.pending == (ushort)(ushort)28761);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lon = (int)690358844;
            p136.lat = (int)463453157;
            p136.terrain_height = (float)3.4879707E36F;
            p136.current_height = (float) -1.9130787E38F;
            p136.spacing = (ushort)(ushort)59092;
            p136.pending = (ushort)(ushort)28761;
            p136.loaded = (ushort)(ushort)36512;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)15503);
                Debug.Assert(pack.time_boot_ms == (uint)177728526U);
                Debug.Assert(pack.press_abs == (float) -3.0216215E38F);
                Debug.Assert(pack.press_diff == (float) -1.4045524E38F);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.press_diff = (float) -1.4045524E38F;
            p137.temperature = (short)(short)15503;
            p137.press_abs = (float) -3.0216215E38F;
            p137.time_boot_ms = (uint)177728526U;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)2.3900167E38F);
                Debug.Assert(pack.x == (float)3.0103292E38F);
                Debug.Assert(pack.time_usec == (ulong)3993426828527193839L);
                Debug.Assert(pack.y == (float) -2.6857291E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-4.0096283E37F, 1.600411E38F, 2.638079E38F, -3.1998343E38F}));
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.z = (float)2.3900167E38F;
            p138.q_SET(new float[] {-4.0096283E37F, 1.600411E38F, 2.638079E38F, -3.1998343E38F}, 0) ;
            p138.y = (float) -2.6857291E38F;
            p138.x = (float)3.0103292E38F;
            p138.time_usec = (ulong)3993426828527193839L;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {2.6740205E38F, 2.5233332E38F, 7.5138047E37F, 1.3590459E38F, 2.3915675E38F, -1.0927153E38F, 2.4375144E38F, 2.8539385E38F}));
                Debug.Assert(pack.time_usec == (ulong)1893022606806543123L);
                Debug.Assert(pack.group_mlx == (byte)(byte)203);
                Debug.Assert(pack.target_component == (byte)(byte)254);
                Debug.Assert(pack.target_system == (byte)(byte)29);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.target_system = (byte)(byte)29;
            p139.group_mlx = (byte)(byte)203;
            p139.time_usec = (ulong)1893022606806543123L;
            p139.controls_SET(new float[] {2.6740205E38F, 2.5233332E38F, 7.5138047E37F, 1.3590459E38F, 2.3915675E38F, -1.0927153E38F, 2.4375144E38F, 2.8539385E38F}, 0) ;
            p139.target_component = (byte)(byte)254;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-2.3568113E38F, 2.352719E38F, -2.4733978E37F, 1.705637E38F, -2.017591E38F, -2.0194177E38F, -1.6119404E38F, 3.293538E38F}));
                Debug.Assert(pack.time_usec == (ulong)6747112309222739377L);
                Debug.Assert(pack.group_mlx == (byte)(byte)41);
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)6747112309222739377L;
            p140.group_mlx = (byte)(byte)41;
            p140.controls_SET(new float[] {-2.3568113E38F, 2.352719E38F, -2.4733978E37F, 1.705637E38F, -2.017591E38F, -2.0194177E38F, -1.6119404E38F, 3.293538E38F}, 0) ;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.bottom_clearance == (float) -7.3728293E37F);
                Debug.Assert(pack.time_usec == (ulong)4196262057593763951L);
                Debug.Assert(pack.altitude_amsl == (float)5.034152E37F);
                Debug.Assert(pack.altitude_monotonic == (float) -2.7270833E38F);
                Debug.Assert(pack.altitude_terrain == (float)2.7025266E37F);
                Debug.Assert(pack.altitude_relative == (float)3.1115143E37F);
                Debug.Assert(pack.altitude_local == (float) -2.1054145E38F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)4196262057593763951L;
            p141.altitude_relative = (float)3.1115143E37F;
            p141.altitude_monotonic = (float) -2.7270833E38F;
            p141.bottom_clearance = (float) -7.3728293E37F;
            p141.altitude_local = (float) -2.1054145E38F;
            p141.altitude_amsl = (float)5.034152E37F;
            p141.altitude_terrain = (float)2.7025266E37F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.transfer_type == (byte)(byte)67);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)148, (byte)200, (byte)50, (byte)61, (byte)226, (byte)8, (byte)98, (byte)52, (byte)235, (byte)60, (byte)144, (byte)4, (byte)23, (byte)132, (byte)125, (byte)50, (byte)248, (byte)67, (byte)152, (byte)216, (byte)59, (byte)155, (byte)179, (byte)7, (byte)104, (byte)81, (byte)191, (byte)219, (byte)79, (byte)197, (byte)65, (byte)169, (byte)91, (byte)163, (byte)126, (byte)0, (byte)130, (byte)241, (byte)224, (byte)184, (byte)85, (byte)27, (byte)132, (byte)231, (byte)133, (byte)56, (byte)62, (byte)189, (byte)123, (byte)24, (byte)62, (byte)44, (byte)166, (byte)119, (byte)116, (byte)192, (byte)20, (byte)159, (byte)67, (byte)129, (byte)250, (byte)7, (byte)182, (byte)248, (byte)149, (byte)250, (byte)8, (byte)155, (byte)247, (byte)185, (byte)117, (byte)104, (byte)77, (byte)191, (byte)108, (byte)114, (byte)25, (byte)255, (byte)169, (byte)227, (byte)144, (byte)168, (byte)205, (byte)75, (byte)147, (byte)142, (byte)82, (byte)103, (byte)83, (byte)133, (byte)17, (byte)220, (byte)74, (byte)30, (byte)52, (byte)104, (byte)213, (byte)101, (byte)118, (byte)170, (byte)82, (byte)87, (byte)176, (byte)30, (byte)165, (byte)114, (byte)148, (byte)85, (byte)205, (byte)204, (byte)214, (byte)41, (byte)11, (byte)202, (byte)159, (byte)100, (byte)146, (byte)38, (byte)214, (byte)225}));
                Debug.Assert(pack.request_id == (byte)(byte)149);
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)4, (byte)207, (byte)239, (byte)107, (byte)11, (byte)54, (byte)122, (byte)58, (byte)128, (byte)208, (byte)180, (byte)154, (byte)244, (byte)238, (byte)45, (byte)130, (byte)57, (byte)155, (byte)207, (byte)17, (byte)253, (byte)5, (byte)89, (byte)77, (byte)81, (byte)248, (byte)83, (byte)90, (byte)184, (byte)63, (byte)90, (byte)151, (byte)120, (byte)60, (byte)155, (byte)54, (byte)155, (byte)39, (byte)122, (byte)12, (byte)118, (byte)233, (byte)120, (byte)166, (byte)247, (byte)55, (byte)26, (byte)44, (byte)200, (byte)125, (byte)123, (byte)165, (byte)148, (byte)174, (byte)213, (byte)14, (byte)229, (byte)83, (byte)133, (byte)246, (byte)243, (byte)95, (byte)157, (byte)26, (byte)172, (byte)197, (byte)227, (byte)94, (byte)20, (byte)163, (byte)26, (byte)250, (byte)199, (byte)245, (byte)234, (byte)6, (byte)226, (byte)44, (byte)31, (byte)228, (byte)63, (byte)59, (byte)188, (byte)98, (byte)118, (byte)159, (byte)158, (byte)180, (byte)253, (byte)58, (byte)130, (byte)238, (byte)179, (byte)170, (byte)151, (byte)169, (byte)44, (byte)16, (byte)18, (byte)208, (byte)189, (byte)178, (byte)22, (byte)241, (byte)77, (byte)184, (byte)123, (byte)23, (byte)191, (byte)14, (byte)2, (byte)23, (byte)231, (byte)107, (byte)161, (byte)222, (byte)40, (byte)229, (byte)105, (byte)22}));
                Debug.Assert(pack.uri_type == (byte)(byte)183);
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.storage_SET(new byte[] {(byte)4, (byte)207, (byte)239, (byte)107, (byte)11, (byte)54, (byte)122, (byte)58, (byte)128, (byte)208, (byte)180, (byte)154, (byte)244, (byte)238, (byte)45, (byte)130, (byte)57, (byte)155, (byte)207, (byte)17, (byte)253, (byte)5, (byte)89, (byte)77, (byte)81, (byte)248, (byte)83, (byte)90, (byte)184, (byte)63, (byte)90, (byte)151, (byte)120, (byte)60, (byte)155, (byte)54, (byte)155, (byte)39, (byte)122, (byte)12, (byte)118, (byte)233, (byte)120, (byte)166, (byte)247, (byte)55, (byte)26, (byte)44, (byte)200, (byte)125, (byte)123, (byte)165, (byte)148, (byte)174, (byte)213, (byte)14, (byte)229, (byte)83, (byte)133, (byte)246, (byte)243, (byte)95, (byte)157, (byte)26, (byte)172, (byte)197, (byte)227, (byte)94, (byte)20, (byte)163, (byte)26, (byte)250, (byte)199, (byte)245, (byte)234, (byte)6, (byte)226, (byte)44, (byte)31, (byte)228, (byte)63, (byte)59, (byte)188, (byte)98, (byte)118, (byte)159, (byte)158, (byte)180, (byte)253, (byte)58, (byte)130, (byte)238, (byte)179, (byte)170, (byte)151, (byte)169, (byte)44, (byte)16, (byte)18, (byte)208, (byte)189, (byte)178, (byte)22, (byte)241, (byte)77, (byte)184, (byte)123, (byte)23, (byte)191, (byte)14, (byte)2, (byte)23, (byte)231, (byte)107, (byte)161, (byte)222, (byte)40, (byte)229, (byte)105, (byte)22}, 0) ;
            p142.uri_type = (byte)(byte)183;
            p142.request_id = (byte)(byte)149;
            p142.uri_SET(new byte[] {(byte)148, (byte)200, (byte)50, (byte)61, (byte)226, (byte)8, (byte)98, (byte)52, (byte)235, (byte)60, (byte)144, (byte)4, (byte)23, (byte)132, (byte)125, (byte)50, (byte)248, (byte)67, (byte)152, (byte)216, (byte)59, (byte)155, (byte)179, (byte)7, (byte)104, (byte)81, (byte)191, (byte)219, (byte)79, (byte)197, (byte)65, (byte)169, (byte)91, (byte)163, (byte)126, (byte)0, (byte)130, (byte)241, (byte)224, (byte)184, (byte)85, (byte)27, (byte)132, (byte)231, (byte)133, (byte)56, (byte)62, (byte)189, (byte)123, (byte)24, (byte)62, (byte)44, (byte)166, (byte)119, (byte)116, (byte)192, (byte)20, (byte)159, (byte)67, (byte)129, (byte)250, (byte)7, (byte)182, (byte)248, (byte)149, (byte)250, (byte)8, (byte)155, (byte)247, (byte)185, (byte)117, (byte)104, (byte)77, (byte)191, (byte)108, (byte)114, (byte)25, (byte)255, (byte)169, (byte)227, (byte)144, (byte)168, (byte)205, (byte)75, (byte)147, (byte)142, (byte)82, (byte)103, (byte)83, (byte)133, (byte)17, (byte)220, (byte)74, (byte)30, (byte)52, (byte)104, (byte)213, (byte)101, (byte)118, (byte)170, (byte)82, (byte)87, (byte)176, (byte)30, (byte)165, (byte)114, (byte)148, (byte)85, (byte)205, (byte)204, (byte)214, (byte)41, (byte)11, (byte)202, (byte)159, (byte)100, (byte)146, (byte)38, (byte)214, (byte)225}, 0) ;
            p142.transfer_type = (byte)(byte)67;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float)2.304168E38F);
                Debug.Assert(pack.temperature == (short)(short)7651);
                Debug.Assert(pack.time_boot_ms == (uint)2436975315U);
                Debug.Assert(pack.press_diff == (float)1.609706E38F);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.press_diff = (float)1.609706E38F;
            p143.press_abs = (float)2.304168E38F;
            p143.temperature = (short)(short)7651;
            p143.time_boot_ms = (uint)2436975315U;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.acc.SequenceEqual(new float[] {1.0179862E38F, 2.7677206E38F, -1.6290653E38F}));
                Debug.Assert(pack.est_capabilities == (byte)(byte)120);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {2.8756354E38F, -7.3442813E37F, 1.8883347E38F}));
                Debug.Assert(pack.alt == (float) -2.4773892E38F);
                Debug.Assert(pack.lat == (int)988099034);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {3.2280848E38F, -2.6661834E38F, 1.5585887E38F}));
                Debug.Assert(pack.timestamp == (ulong)2563371392411183808L);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-1.4116408E38F, 6.7458413E36F, -3.0622802E38F, -1.355933E38F}));
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-2.8142977E38F, 4.1629947E37F, -1.4069958E38F}));
                Debug.Assert(pack.custom_state == (ulong)5086272140936770996L);
                Debug.Assert(pack.lon == (int)1573254842);
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.lat = (int)988099034;
            p144.vel_SET(new float[] {-2.8142977E38F, 4.1629947E37F, -1.4069958E38F}, 0) ;
            p144.custom_state = (ulong)5086272140936770996L;
            p144.position_cov_SET(new float[] {2.8756354E38F, -7.3442813E37F, 1.8883347E38F}, 0) ;
            p144.lon = (int)1573254842;
            p144.est_capabilities = (byte)(byte)120;
            p144.alt = (float) -2.4773892E38F;
            p144.rates_SET(new float[] {3.2280848E38F, -2.6661834E38F, 1.5585887E38F}, 0) ;
            p144.acc_SET(new float[] {1.0179862E38F, 2.7677206E38F, -1.6290653E38F}, 0) ;
            p144.attitude_q_SET(new float[] {-1.4116408E38F, 6.7458413E36F, -3.0622802E38F, -1.355933E38F}, 0) ;
            p144.timestamp = (ulong)2563371392411183808L;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z_acc == (float) -8.355184E37F);
                Debug.Assert(pack.y_pos == (float) -1.5095466E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {1.826526E38F, -2.3945806E38F, 1.1227193E38F}));
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {-5.292242E37F, -2.039707E38F, -6.0160147E37F}));
                Debug.Assert(pack.time_usec == (ulong)2897683914487352038L);
                Debug.Assert(pack.roll_rate == (float) -4.434993E37F);
                Debug.Assert(pack.x_vel == (float)1.7886518E38F);
                Debug.Assert(pack.z_pos == (float) -2.7509108E36F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.3767508E38F, 2.0501082E38F, 1.769766E38F, 1.1166024E38F}));
                Debug.Assert(pack.z_vel == (float) -8.332932E36F);
                Debug.Assert(pack.airspeed == (float) -3.2799307E38F);
                Debug.Assert(pack.x_pos == (float) -2.5496695E38F);
                Debug.Assert(pack.pitch_rate == (float)4.778503E37F);
                Debug.Assert(pack.y_vel == (float) -4.5621244E37F);
                Debug.Assert(pack.y_acc == (float) -2.0541244E38F);
                Debug.Assert(pack.x_acc == (float)4.912937E37F);
                Debug.Assert(pack.yaw_rate == (float) -3.2811994E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.roll_rate = (float) -4.434993E37F;
            p146.vel_variance_SET(new float[] {1.826526E38F, -2.3945806E38F, 1.1227193E38F}, 0) ;
            p146.z_acc = (float) -8.355184E37F;
            p146.z_vel = (float) -8.332932E36F;
            p146.airspeed = (float) -3.2799307E38F;
            p146.pitch_rate = (float)4.778503E37F;
            p146.yaw_rate = (float) -3.2811994E38F;
            p146.x_vel = (float)1.7886518E38F;
            p146.y_vel = (float) -4.5621244E37F;
            p146.x_pos = (float) -2.5496695E38F;
            p146.x_acc = (float)4.912937E37F;
            p146.y_pos = (float) -1.5095466E38F;
            p146.q_SET(new float[] {2.3767508E38F, 2.0501082E38F, 1.769766E38F, 1.1166024E38F}, 0) ;
            p146.pos_variance_SET(new float[] {-5.292242E37F, -2.039707E38F, -6.0160147E37F}, 0) ;
            p146.time_usec = (ulong)2897683914487352038L;
            p146.z_pos = (float) -2.7509108E36F;
            p146.y_acc = (float) -2.0541244E38F;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.battery_function == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
                Debug.Assert(pack.temperature == (short)(short) -14026);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 116);
                Debug.Assert(pack.current_battery == (short)(short)29745);
                Debug.Assert(pack.current_consumed == (int) -1540172208);
                Debug.Assert(pack.energy_consumed == (int)691215127);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)37864, (ushort)47619, (ushort)26992, (ushort)65485, (ushort)25113, (ushort)62560, (ushort)6608, (ushort)11754, (ushort)39053, (ushort)22818}));
                Debug.Assert(pack.id == (byte)(byte)137);
                Debug.Assert(pack.type == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.battery_remaining = (sbyte)(sbyte) - 116;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION;
            p147.temperature = (short)(short) -14026;
            p147.id = (byte)(byte)137;
            p147.energy_consumed = (int)691215127;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL;
            p147.current_battery = (short)(short)29745;
            p147.current_consumed = (int) -1540172208;
            p147.voltages_SET(new ushort[] {(ushort)37864, (ushort)47619, (ushort)26992, (ushort)65485, (ushort)25113, (ushort)62560, (ushort)6608, (ushort)11754, (ushort)39053, (ushort)22818}, 0) ;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.product_id == (ushort)(ushort)48049);
                Debug.Assert(pack.uid == (ulong)8865644197333276328L);
                Debug.Assert(pack.os_sw_version == (uint)2406077300U);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)236, (byte)102, (byte)150, (byte)104, (byte)43, (byte)172, (byte)39, (byte)118}));
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)252, (byte)97, (byte)67, (byte)178, (byte)7, (byte)122, (byte)71, (byte)15}));
                Debug.Assert(pack.vendor_id == (ushort)(ushort)59785);
                Debug.Assert(pack.middleware_sw_version == (uint)1355545300U);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION));
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)83, (byte)88, (byte)128, (byte)147, (byte)208, (byte)213, (byte)109, (byte)182}));
                Debug.Assert(pack.board_version == (uint)4035659376U);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)4, (byte)240, (byte)91, (byte)131, (byte)193, (byte)252, (byte)11, (byte)228, (byte)149, (byte)153, (byte)27, (byte)209, (byte)113, (byte)65, (byte)122, (byte)78, (byte)105, (byte)228}));
                Debug.Assert(pack.flight_sw_version == (uint)1888375369U);
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.vendor_id = (ushort)(ushort)59785;
            p148.os_custom_version_SET(new byte[] {(byte)83, (byte)88, (byte)128, (byte)147, (byte)208, (byte)213, (byte)109, (byte)182}, 0) ;
            p148.board_version = (uint)4035659376U;
            p148.uid2_SET(new byte[] {(byte)4, (byte)240, (byte)91, (byte)131, (byte)193, (byte)252, (byte)11, (byte)228, (byte)149, (byte)153, (byte)27, (byte)209, (byte)113, (byte)65, (byte)122, (byte)78, (byte)105, (byte)228}, 0, PH) ;
            p148.middleware_custom_version_SET(new byte[] {(byte)236, (byte)102, (byte)150, (byte)104, (byte)43, (byte)172, (byte)39, (byte)118}, 0) ;
            p148.product_id = (ushort)(ushort)48049;
            p148.flight_custom_version_SET(new byte[] {(byte)252, (byte)97, (byte)67, (byte)178, (byte)7, (byte)122, (byte)71, (byte)15}, 0) ;
            p148.middleware_sw_version = (uint)1355545300U;
            p148.os_sw_version = (uint)2406077300U;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION);
            p148.flight_sw_version = (uint)1888375369U;
            p148.uid = (ulong)8865644197333276328L;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.size_x == (float)8.367167E37F);
                Debug.Assert(pack.size_y == (float) -2.3335539E38F);
                Debug.Assert(pack.angle_y == (float)1.5087613E37F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)150);
                Debug.Assert(pack.x_TRY(ph) == (float) -2.2905377E38F);
                Debug.Assert(pack.angle_x == (float)1.9123323E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.time_usec == (ulong)3912845751444930660L);
                Debug.Assert(pack.target_num == (byte)(byte)35);
                Debug.Assert(pack.type == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
                Debug.Assert(pack.y_TRY(ph) == (float) -2.3306876E38F);
                Debug.Assert(pack.z_TRY(ph) == (float) -2.4983452E38F);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {1.1369813E37F, -3.1093293E38F, 3.3185681E38F, -7.5809385E37F}));
                Debug.Assert(pack.distance == (float) -2.2616759E38F);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.angle_y = (float)1.5087613E37F;
            p149.position_valid_SET((byte)(byte)150, PH) ;
            p149.target_num = (byte)(byte)35;
            p149.x_SET((float) -2.2905377E38F, PH) ;
            p149.size_x = (float)8.367167E37F;
            p149.z_SET((float) -2.4983452E38F, PH) ;
            p149.time_usec = (ulong)3912845751444930660L;
            p149.angle_x = (float)1.9123323E38F;
            p149.frame = MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p149.distance = (float) -2.2616759E38F;
            p149.size_y = (float) -2.3335539E38F;
            p149.q_SET(new float[] {1.1369813E37F, -3.1093293E38F, 3.3185681E38F, -7.5809385E37F}, 0, PH) ;
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON;
            p149.y_SET((float) -2.3306876E38F, PH) ;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnCPU_LOADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensLoad == (byte)(byte)11);
                Debug.Assert(pack.ctrlLoad == (byte)(byte)183);
                Debug.Assert(pack.batVolt == (ushort)(ushort)54262);
            };
            GroundControl.CPU_LOAD p170 = CommunicationChannel.new_CPU_LOAD();
            PH.setPack(p170);
            p170.sensLoad = (byte)(byte)11;
            p170.batVolt = (ushort)(ushort)54262;
            p170.ctrlLoad = (byte)(byte)183;
            CommunicationChannel.instance.send(p170);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSENSOR_BIASReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.axBias == (float) -2.7367812E38F);
                Debug.Assert(pack.ayBias == (float)7.0971736E37F);
                Debug.Assert(pack.gzBias == (float) -2.8287998E38F);
                Debug.Assert(pack.gyBias == (float) -8.623264E37F);
                Debug.Assert(pack.azBias == (float) -1.834913E38F);
                Debug.Assert(pack.gxBias == (float) -2.5154058E38F);
            };
            GroundControl.SENSOR_BIAS p172 = CommunicationChannel.new_SENSOR_BIAS();
            PH.setPack(p172);
            p172.axBias = (float) -2.7367812E38F;
            p172.gzBias = (float) -2.8287998E38F;
            p172.gyBias = (float) -8.623264E37F;
            p172.azBias = (float) -1.834913E38F;
            p172.gxBias = (float) -2.5154058E38F;
            p172.ayBias = (float)7.0971736E37F;
            CommunicationChannel.instance.send(p172);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDIAGNOSTICReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.diagSh1 == (short)(short) -21293);
                Debug.Assert(pack.diagFl2 == (float) -2.1480725E38F);
                Debug.Assert(pack.diagSh2 == (short)(short)32354);
                Debug.Assert(pack.diagFl3 == (float) -1.3384289E38F);
                Debug.Assert(pack.diagSh3 == (short)(short) -1522);
                Debug.Assert(pack.diagFl1 == (float)2.5376552E38F);
            };
            GroundControl.DIAGNOSTIC p173 = CommunicationChannel.new_DIAGNOSTIC();
            PH.setPack(p173);
            p173.diagSh2 = (short)(short)32354;
            p173.diagFl3 = (float) -1.3384289E38F;
            p173.diagSh3 = (short)(short) -1522;
            p173.diagFl2 = (float) -2.1480725E38F;
            p173.diagFl1 = (float)2.5376552E38F;
            p173.diagSh1 = (short)(short) -21293;
            CommunicationChannel.instance.send(p173);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSLUGS_NAVIGATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.psiDot_c == (float) -3.1429908E38F);
                Debug.Assert(pack.totalDist == (float)3.2212146E38F);
                Debug.Assert(pack.dist2Go == (float) -1.6499743E38F);
                Debug.Assert(pack.toWP == (byte)(byte)159);
                Debug.Assert(pack.phi_c == (float) -1.2540342E38F);
                Debug.Assert(pack.u_m == (float)2.1368472E38F);
                Debug.Assert(pack.ay_body == (float) -1.5195099E38F);
                Debug.Assert(pack.fromWP == (byte)(byte)201);
                Debug.Assert(pack.h_c == (ushort)(ushort)8727);
                Debug.Assert(pack.theta_c == (float)1.9944395E38F);
            };
            GroundControl.SLUGS_NAVIGATION p176 = CommunicationChannel.new_SLUGS_NAVIGATION();
            PH.setPack(p176);
            p176.phi_c = (float) -1.2540342E38F;
            p176.fromWP = (byte)(byte)201;
            p176.ay_body = (float) -1.5195099E38F;
            p176.psiDot_c = (float) -3.1429908E38F;
            p176.toWP = (byte)(byte)159;
            p176.dist2Go = (float) -1.6499743E38F;
            p176.theta_c = (float)1.9944395E38F;
            p176.h_c = (ushort)(ushort)8727;
            p176.u_m = (float)2.1368472E38F;
            p176.totalDist = (float)3.2212146E38F;
            CommunicationChannel.instance.send(p176);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDATA_LOGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fl_4 == (float) -1.0896843E38F);
                Debug.Assert(pack.fl_2 == (float)3.0000675E38F);
                Debug.Assert(pack.fl_5 == (float) -2.261023E38F);
                Debug.Assert(pack.fl_6 == (float)1.3583911E38F);
                Debug.Assert(pack.fl_1 == (float) -1.0259672E38F);
                Debug.Assert(pack.fl_3 == (float)5.947711E37F);
            };
            GroundControl.DATA_LOG p177 = CommunicationChannel.new_DATA_LOG();
            PH.setPack(p177);
            p177.fl_6 = (float)1.3583911E38F;
            p177.fl_1 = (float) -1.0259672E38F;
            p177.fl_5 = (float) -2.261023E38F;
            p177.fl_2 = (float)3.0000675E38F;
            p177.fl_3 = (float)5.947711E37F;
            p177.fl_4 = (float) -1.0896843E38F;
            CommunicationChannel.instance.send(p177);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_DATE_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.day == (byte)(byte)154);
                Debug.Assert(pack.hour == (byte)(byte)210);
                Debug.Assert(pack.month == (byte)(byte)156);
                Debug.Assert(pack.sec == (byte)(byte)50);
                Debug.Assert(pack.useSat == (byte)(byte)36);
                Debug.Assert(pack.min == (byte)(byte)113);
                Debug.Assert(pack.visSat == (byte)(byte)239);
                Debug.Assert(pack.percentUsed == (byte)(byte)127);
                Debug.Assert(pack.GppGl == (byte)(byte)60);
                Debug.Assert(pack.clockStat == (byte)(byte)194);
                Debug.Assert(pack.year == (byte)(byte)23);
                Debug.Assert(pack.sigUsedMask == (byte)(byte)230);
            };
            GroundControl.GPS_DATE_TIME p179 = CommunicationChannel.new_GPS_DATE_TIME();
            PH.setPack(p179);
            p179.min = (byte)(byte)113;
            p179.useSat = (byte)(byte)36;
            p179.hour = (byte)(byte)210;
            p179.day = (byte)(byte)154;
            p179.clockStat = (byte)(byte)194;
            p179.sigUsedMask = (byte)(byte)230;
            p179.sec = (byte)(byte)50;
            p179.GppGl = (byte)(byte)60;
            p179.month = (byte)(byte)156;
            p179.year = (byte)(byte)23;
            p179.visSat = (byte)(byte)239;
            p179.percentUsed = (byte)(byte)127;
            CommunicationChannel.instance.send(p179);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMID_LVL_CMDSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target == (byte)(byte)254);
                Debug.Assert(pack.rCommand == (float) -2.0306787E38F);
                Debug.Assert(pack.hCommand == (float)2.7563052E38F);
                Debug.Assert(pack.uCommand == (float)1.1879875E38F);
            };
            GroundControl.MID_LVL_CMDS p180 = CommunicationChannel.new_MID_LVL_CMDS();
            PH.setPack(p180);
            p180.rCommand = (float) -2.0306787E38F;
            p180.hCommand = (float)2.7563052E38F;
            p180.uCommand = (float)1.1879875E38F;
            p180.target = (byte)(byte)254;
            CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCTRL_SRFC_PTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target == (byte)(byte)233);
                Debug.Assert(pack.bitfieldPt == (ushort)(ushort)36821);
            };
            GroundControl.CTRL_SRFC_PT p181 = CommunicationChannel.new_CTRL_SRFC_PT();
            PH.setPack(p181);
            p181.target = (byte)(byte)233;
            p181.bitfieldPt = (ushort)(ushort)36821;
            CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSLUGS_CAMERA_ORDERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target == (byte)(byte)135);
                Debug.Assert(pack.tilt == (sbyte)(sbyte)41);
                Debug.Assert(pack.moveHome == (sbyte)(sbyte) - 68);
                Debug.Assert(pack.zoom == (sbyte)(sbyte)12);
                Debug.Assert(pack.pan == (sbyte)(sbyte)57);
            };
            GroundControl.SLUGS_CAMERA_ORDER p184 = CommunicationChannel.new_SLUGS_CAMERA_ORDER();
            PH.setPack(p184);
            p184.moveHome = (sbyte)(sbyte) - 68;
            p184.pan = (sbyte)(sbyte)57;
            p184.zoom = (sbyte)(sbyte)12;
            p184.target = (byte)(byte)135;
            p184.tilt = (sbyte)(sbyte)41;
            CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCONTROL_SURFACEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.idSurface == (byte)(byte)27);
                Debug.Assert(pack.bControl == (float) -9.632056E37F);
                Debug.Assert(pack.target == (byte)(byte)57);
                Debug.Assert(pack.mControl == (float)1.2426239E38F);
            };
            GroundControl.CONTROL_SURFACE p185 = CommunicationChannel.new_CONTROL_SURFACE();
            PH.setPack(p185);
            p185.bControl = (float) -9.632056E37F;
            p185.target = (byte)(byte)57;
            p185.mControl = (float)1.2426239E38F;
            p185.idSurface = (byte)(byte)27;
            CommunicationChannel.instance.send(p185);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSLUGS_MOBILE_LOCATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (float) -1.4590122E37F);
                Debug.Assert(pack.target == (byte)(byte)144);
                Debug.Assert(pack.latitude == (float) -1.9212805E38F);
            };
            GroundControl.SLUGS_MOBILE_LOCATION p186 = CommunicationChannel.new_SLUGS_MOBILE_LOCATION();
            PH.setPack(p186);
            p186.longitude = (float) -1.4590122E37F;
            p186.target = (byte)(byte)144;
            p186.latitude = (float) -1.9212805E38F;
            CommunicationChannel.instance.send(p186);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSLUGS_CONFIGURATION_CAMERAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target == (byte)(byte)162);
                Debug.Assert(pack.idOrder == (byte)(byte)135);
                Debug.Assert(pack.order == (byte)(byte)190);
            };
            GroundControl.SLUGS_CONFIGURATION_CAMERA p188 = CommunicationChannel.new_SLUGS_CONFIGURATION_CAMERA();
            PH.setPack(p188);
            p188.target = (byte)(byte)162;
            p188.order = (byte)(byte)190;
            p188.idOrder = (byte)(byte)135;
            CommunicationChannel.instance.send(p188);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnISR_LOCATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (float) -4.79458E37F);
                Debug.Assert(pack.height == (float)1.807581E38F);
                Debug.Assert(pack.option1 == (byte)(byte)250);
                Debug.Assert(pack.longitude == (float)1.8183277E37F);
                Debug.Assert(pack.option2 == (byte)(byte)237);
                Debug.Assert(pack.target == (byte)(byte)237);
                Debug.Assert(pack.option3 == (byte)(byte)241);
            };
            GroundControl.ISR_LOCATION p189 = CommunicationChannel.new_ISR_LOCATION();
            PH.setPack(p189);
            p189.option2 = (byte)(byte)237;
            p189.height = (float)1.807581E38F;
            p189.longitude = (float)1.8183277E37F;
            p189.option1 = (byte)(byte)250;
            p189.option3 = (byte)(byte)241;
            p189.target = (byte)(byte)237;
            p189.latitude = (float) -4.79458E37F;
            CommunicationChannel.instance.send(p189);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVOLT_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.reading2 == (ushort)(ushort)35048);
                Debug.Assert(pack.r2Type == (byte)(byte)176);
                Debug.Assert(pack.voltage == (ushort)(ushort)15761);
            };
            GroundControl.VOLT_SENSOR p191 = CommunicationChannel.new_VOLT_SENSOR();
            PH.setPack(p191);
            p191.r2Type = (byte)(byte)176;
            p191.reading2 = (ushort)(ushort)35048;
            p191.voltage = (ushort)(ushort)15761;
            CommunicationChannel.instance.send(p191);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPTZ_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zoom == (byte)(byte)115);
                Debug.Assert(pack.tilt == (short)(short)10761);
                Debug.Assert(pack.pan == (short)(short) -4950);
            };
            GroundControl.PTZ_STATUS p192 = CommunicationChannel.new_PTZ_STATUS();
            PH.setPack(p192);
            p192.zoom = (byte)(byte)115;
            p192.pan = (short)(short) -4950;
            p192.tilt = (short)(short)10761;
            CommunicationChannel.instance.send(p192);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAV_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.course == (float) -9.527016E37F);
                Debug.Assert(pack.latitude == (float)4.075752E37F);
                Debug.Assert(pack.target == (byte)(byte)96);
                Debug.Assert(pack.altitude == (float) -3.2672817E37F);
                Debug.Assert(pack.longitude == (float)4.3914657E37F);
                Debug.Assert(pack.speed == (float)1.0047282E38F);
            };
            GroundControl.UAV_STATUS p193 = CommunicationChannel.new_UAV_STATUS();
            PH.setPack(p193);
            p193.latitude = (float)4.075752E37F;
            p193.target = (byte)(byte)96;
            p193.longitude = (float)4.3914657E37F;
            p193.course = (float) -9.527016E37F;
            p193.speed = (float)1.0047282E38F;
            p193.altitude = (float) -3.2672817E37F;
            CommunicationChannel.instance.send(p193);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTATUS_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.posStatus == (byte)(byte)206);
                Debug.Assert(pack.msgsType == (byte)(byte)98);
                Debug.Assert(pack.magDir == (sbyte)(sbyte) - 104);
                Debug.Assert(pack.magVar == (float) -2.2034159E38F);
                Debug.Assert(pack.csFails == (ushort)(ushort)39521);
                Debug.Assert(pack.gpsQuality == (byte)(byte)132);
                Debug.Assert(pack.modeInd == (byte)(byte)9);
            };
            GroundControl.STATUS_GPS p194 = CommunicationChannel.new_STATUS_GPS();
            PH.setPack(p194);
            p194.gpsQuality = (byte)(byte)132;
            p194.posStatus = (byte)(byte)206;
            p194.magDir = (sbyte)(sbyte) - 104;
            p194.magVar = (float) -2.2034159E38F;
            p194.msgsType = (byte)(byte)98;
            p194.csFails = (ushort)(ushort)39521;
            p194.modeInd = (byte)(byte)9;
            CommunicationChannel.instance.send(p194);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNOVATEL_DIAGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.solStatus == (byte)(byte)126);
                Debug.Assert(pack.posType == (byte)(byte)180);
                Debug.Assert(pack.csFails == (ushort)(ushort)4103);
                Debug.Assert(pack.posSolAge == (float) -1.0325112E37F);
                Debug.Assert(pack.timeStatus == (byte)(byte)234);
                Debug.Assert(pack.velType == (byte)(byte)106);
                Debug.Assert(pack.receiverStatus == (uint)2092049765U);
            };
            GroundControl.NOVATEL_DIAG p195 = CommunicationChannel.new_NOVATEL_DIAG();
            PH.setPack(p195);
            p195.posType = (byte)(byte)180;
            p195.receiverStatus = (uint)2092049765U;
            p195.csFails = (ushort)(ushort)4103;
            p195.timeStatus = (byte)(byte)234;
            p195.posSolAge = (float) -1.0325112E37F;
            p195.solStatus = (byte)(byte)126;
            p195.velType = (byte)(byte)106;
            CommunicationChannel.instance.send(p195);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSENSOR_DIAGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.char1 == (sbyte)(sbyte)87);
                Debug.Assert(pack.int1 == (short)(short)3016);
                Debug.Assert(pack.float1 == (float) -1.2432714E38F);
                Debug.Assert(pack.float2 == (float) -2.8236481E38F);
            };
            GroundControl.SENSOR_DIAG p196 = CommunicationChannel.new_SENSOR_DIAG();
            PH.setPack(p196);
            p196.float2 = (float) -2.8236481E38F;
            p196.float1 = (float) -1.2432714E38F;
            p196.int1 = (short)(short)3016;
            p196.char1 = (sbyte)(sbyte)87;
            CommunicationChannel.instance.send(p196);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBOOTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.version == (uint)3599835463U);
            };
            GroundControl.BOOT p197 = CommunicationChannel.new_BOOT();
            PH.setPack(p197);
            p197.version = (uint)3599835463U;
            CommunicationChannel.instance.send(p197);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            CommunicationChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hagl_ratio == (float) -9.887143E37F);
                Debug.Assert(pack.tas_ratio == (float) -3.0893584E38F);
                Debug.Assert(pack.pos_horiz_ratio == (float)3.0635428E38F);
                Debug.Assert(pack.pos_vert_accuracy == (float) -1.9992953E38F);
                Debug.Assert(pack.vel_ratio == (float)9.516803E37F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH));
                Debug.Assert(pack.pos_horiz_accuracy == (float) -7.417716E37F);
                Debug.Assert(pack.mag_ratio == (float)1.7542319E38F);
                Debug.Assert(pack.pos_vert_ratio == (float) -6.2208594E37F);
                Debug.Assert(pack.time_usec == (ulong)8239018343674637839L);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.hagl_ratio = (float) -9.887143E37F;
            p230.vel_ratio = (float)9.516803E37F;
            p230.pos_vert_accuracy = (float) -1.9992953E38F;
            p230.pos_horiz_accuracy = (float) -7.417716E37F;
            p230.pos_vert_ratio = (float) -6.2208594E37F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH);
            p230.time_usec = (ulong)8239018343674637839L;
            p230.pos_horiz_ratio = (float)3.0635428E38F;
            p230.tas_ratio = (float) -3.0893584E38F;
            p230.mag_ratio = (float)1.7542319E38F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)2615653922120648571L);
                Debug.Assert(pack.var_horiz == (float)1.4276354E38F);
                Debug.Assert(pack.wind_z == (float) -1.5926364E38F);
                Debug.Assert(pack.wind_alt == (float)2.1905785E38F);
                Debug.Assert(pack.horiz_accuracy == (float)1.0213431E38F);
                Debug.Assert(pack.wind_y == (float) -2.9168113E38F);
                Debug.Assert(pack.var_vert == (float)2.5873996E38F);
                Debug.Assert(pack.vert_accuracy == (float)1.6036814E38F);
                Debug.Assert(pack.wind_x == (float)2.8433093E38F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.wind_y = (float) -2.9168113E38F;
            p231.var_vert = (float)2.5873996E38F;
            p231.wind_x = (float)2.8433093E38F;
            p231.time_usec = (ulong)2615653922120648571L;
            p231.wind_alt = (float)2.1905785E38F;
            p231.vert_accuracy = (float)1.6036814E38F;
            p231.horiz_accuracy = (float)1.0213431E38F;
            p231.wind_z = (float) -1.5926364E38F;
            p231.var_horiz = (float)1.4276354E38F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fix_type == (byte)(byte)151);
                Debug.Assert(pack.ve == (float)1.1006469E38F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT));
                Debug.Assert(pack.vd == (float)1.2224466E38F);
                Debug.Assert(pack.lon == (int) -1053526398);
                Debug.Assert(pack.hdop == (float)9.74019E37F);
                Debug.Assert(pack.lat == (int) -787134269);
                Debug.Assert(pack.satellites_visible == (byte)(byte)83);
                Debug.Assert(pack.alt == (float) -2.938449E38F);
                Debug.Assert(pack.time_usec == (ulong)1745890004828496775L);
                Debug.Assert(pack.horiz_accuracy == (float)2.6881537E38F);
                Debug.Assert(pack.time_week == (ushort)(ushort)20374);
                Debug.Assert(pack.speed_accuracy == (float)2.0730902E38F);
                Debug.Assert(pack.vert_accuracy == (float) -1.015047E38F);
                Debug.Assert(pack.time_week_ms == (uint)4099689659U);
                Debug.Assert(pack.gps_id == (byte)(byte)254);
                Debug.Assert(pack.vdop == (float)3.1467323E38F);
                Debug.Assert(pack.vn == (float) -5.5624737E37F);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT);
            p232.time_week_ms = (uint)4099689659U;
            p232.time_week = (ushort)(ushort)20374;
            p232.lon = (int) -1053526398;
            p232.fix_type = (byte)(byte)151;
            p232.vn = (float) -5.5624737E37F;
            p232.vd = (float)1.2224466E38F;
            p232.vert_accuracy = (float) -1.015047E38F;
            p232.lat = (int) -787134269;
            p232.vdop = (float)3.1467323E38F;
            p232.satellites_visible = (byte)(byte)83;
            p232.horiz_accuracy = (float)2.6881537E38F;
            p232.speed_accuracy = (float)2.0730902E38F;
            p232.gps_id = (byte)(byte)254;
            p232.hdop = (float)9.74019E37F;
            p232.ve = (float)1.1006469E38F;
            p232.time_usec = (ulong)1745890004828496775L;
            p232.alt = (float) -2.938449E38F;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)11);
                Debug.Assert(pack.flags == (byte)(byte)172);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)46, (byte)20, (byte)36, (byte)75, (byte)29, (byte)178, (byte)96, (byte)169, (byte)206, (byte)218, (byte)72, (byte)215, (byte)57, (byte)92, (byte)94, (byte)216, (byte)33, (byte)177, (byte)252, (byte)51, (byte)131, (byte)236, (byte)127, (byte)161, (byte)227, (byte)201, (byte)170, (byte)7, (byte)131, (byte)196, (byte)121, (byte)13, (byte)15, (byte)75, (byte)127, (byte)78, (byte)120, (byte)67, (byte)247, (byte)249, (byte)42, (byte)59, (byte)152, (byte)207, (byte)80, (byte)108, (byte)172, (byte)2, (byte)90, (byte)181, (byte)162, (byte)81, (byte)130, (byte)145, (byte)105, (byte)137, (byte)48, (byte)184, (byte)122, (byte)38, (byte)222, (byte)191, (byte)211, (byte)215, (byte)118, (byte)221, (byte)77, (byte)45, (byte)135, (byte)61, (byte)197, (byte)96, (byte)243, (byte)26, (byte)195, (byte)107, (byte)137, (byte)152, (byte)8, (byte)20, (byte)5, (byte)46, (byte)213, (byte)187, (byte)21, (byte)26, (byte)113, (byte)69, (byte)253, (byte)254, (byte)80, (byte)252, (byte)159, (byte)31, (byte)227, (byte)248, (byte)61, (byte)39, (byte)103, (byte)209, (byte)24, (byte)250, (byte)84, (byte)94, (byte)128, (byte)234, (byte)114, (byte)89, (byte)220, (byte)139, (byte)80, (byte)30, (byte)238, (byte)38, (byte)126, (byte)173, (byte)192, (byte)219, (byte)124, (byte)13, (byte)82, (byte)160, (byte)207, (byte)146, (byte)68, (byte)147, (byte)153, (byte)164, (byte)148, (byte)115, (byte)255, (byte)217, (byte)226, (byte)25, (byte)142, (byte)122, (byte)34, (byte)107, (byte)19, (byte)82, (byte)29, (byte)117, (byte)63, (byte)85, (byte)5, (byte)23, (byte)132, (byte)65, (byte)162, (byte)190, (byte)3, (byte)185, (byte)191, (byte)10, (byte)79, (byte)211, (byte)113, (byte)102, (byte)106, (byte)94, (byte)225, (byte)170, (byte)243, (byte)42, (byte)232, (byte)181, (byte)130, (byte)128, (byte)7, (byte)105, (byte)196, (byte)115, (byte)109, (byte)73, (byte)195, (byte)185, (byte)74, (byte)218, (byte)144, (byte)181}));
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.data__SET(new byte[] {(byte)46, (byte)20, (byte)36, (byte)75, (byte)29, (byte)178, (byte)96, (byte)169, (byte)206, (byte)218, (byte)72, (byte)215, (byte)57, (byte)92, (byte)94, (byte)216, (byte)33, (byte)177, (byte)252, (byte)51, (byte)131, (byte)236, (byte)127, (byte)161, (byte)227, (byte)201, (byte)170, (byte)7, (byte)131, (byte)196, (byte)121, (byte)13, (byte)15, (byte)75, (byte)127, (byte)78, (byte)120, (byte)67, (byte)247, (byte)249, (byte)42, (byte)59, (byte)152, (byte)207, (byte)80, (byte)108, (byte)172, (byte)2, (byte)90, (byte)181, (byte)162, (byte)81, (byte)130, (byte)145, (byte)105, (byte)137, (byte)48, (byte)184, (byte)122, (byte)38, (byte)222, (byte)191, (byte)211, (byte)215, (byte)118, (byte)221, (byte)77, (byte)45, (byte)135, (byte)61, (byte)197, (byte)96, (byte)243, (byte)26, (byte)195, (byte)107, (byte)137, (byte)152, (byte)8, (byte)20, (byte)5, (byte)46, (byte)213, (byte)187, (byte)21, (byte)26, (byte)113, (byte)69, (byte)253, (byte)254, (byte)80, (byte)252, (byte)159, (byte)31, (byte)227, (byte)248, (byte)61, (byte)39, (byte)103, (byte)209, (byte)24, (byte)250, (byte)84, (byte)94, (byte)128, (byte)234, (byte)114, (byte)89, (byte)220, (byte)139, (byte)80, (byte)30, (byte)238, (byte)38, (byte)126, (byte)173, (byte)192, (byte)219, (byte)124, (byte)13, (byte)82, (byte)160, (byte)207, (byte)146, (byte)68, (byte)147, (byte)153, (byte)164, (byte)148, (byte)115, (byte)255, (byte)217, (byte)226, (byte)25, (byte)142, (byte)122, (byte)34, (byte)107, (byte)19, (byte)82, (byte)29, (byte)117, (byte)63, (byte)85, (byte)5, (byte)23, (byte)132, (byte)65, (byte)162, (byte)190, (byte)3, (byte)185, (byte)191, (byte)10, (byte)79, (byte)211, (byte)113, (byte)102, (byte)106, (byte)94, (byte)225, (byte)170, (byte)243, (byte)42, (byte)232, (byte)181, (byte)130, (byte)128, (byte)7, (byte)105, (byte)196, (byte)115, (byte)109, (byte)73, (byte)195, (byte)185, (byte)74, (byte)218, (byte)144, (byte)181}, 0) ;
            p233.len = (byte)(byte)11;
            p233.flags = (byte)(byte)172;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (sbyte)(sbyte)40);
                Debug.Assert(pack.battery_remaining == (byte)(byte)21);
                Debug.Assert(pack.pitch == (short)(short)28615);
                Debug.Assert(pack.wp_num == (byte)(byte)134);
                Debug.Assert(pack.roll == (short)(short) -19742);
                Debug.Assert(pack.airspeed == (byte)(byte)226);
                Debug.Assert(pack.latitude == (int) -841149756);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)93);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 14);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)207);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)4);
                Debug.Assert(pack.altitude_amsl == (short)(short)25619);
                Debug.Assert(pack.longitude == (int) -1848622109);
                Debug.Assert(pack.heading == (ushort)(ushort)13477);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
                Debug.Assert(pack.gps_fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED));
                Debug.Assert(pack.wp_distance == (ushort)(ushort)14169);
                Debug.Assert(pack.altitude_sp == (short)(short) -6002);
                Debug.Assert(pack.failsafe == (byte)(byte)97);
                Debug.Assert(pack.gps_nsat == (byte)(byte)74);
                Debug.Assert(pack.custom_mode == (uint)1230397708U);
                Debug.Assert(pack.heading_sp == (short)(short) -1723);
                Debug.Assert(pack.groundspeed == (byte)(byte)159);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.battery_remaining = (byte)(byte)21;
            p234.throttle = (sbyte)(sbyte)4;
            p234.temperature_air = (sbyte)(sbyte)93;
            p234.heading = (ushort)(ushort)13477;
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
            p234.gps_nsat = (byte)(byte)74;
            p234.pitch = (short)(short)28615;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            p234.altitude_sp = (short)(short) -6002;
            p234.wp_num = (byte)(byte)134;
            p234.custom_mode = (uint)1230397708U;
            p234.airspeed = (byte)(byte)226;
            p234.airspeed_sp = (byte)(byte)207;
            p234.latitude = (int) -841149756;
            p234.wp_distance = (ushort)(ushort)14169;
            p234.temperature = (sbyte)(sbyte)40;
            p234.roll = (short)(short) -19742;
            p234.groundspeed = (byte)(byte)159;
            p234.heading_sp = (short)(short) -1723;
            p234.climb_rate = (sbyte)(sbyte) - 14;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_PPP;
            p234.longitude = (int) -1848622109;
            p234.altitude_amsl = (short)(short)25619;
            p234.failsafe = (byte)(byte)97;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vibration_x == (float)2.775815E38F);
                Debug.Assert(pack.clipping_1 == (uint)3241184989U);
                Debug.Assert(pack.vibration_y == (float) -2.6436437E38F);
                Debug.Assert(pack.time_usec == (ulong)6591279445320304929L);
                Debug.Assert(pack.clipping_0 == (uint)1098349622U);
                Debug.Assert(pack.vibration_z == (float) -3.220741E38F);
                Debug.Assert(pack.clipping_2 == (uint)2643606972U);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.time_usec = (ulong)6591279445320304929L;
            p241.vibration_y = (float) -2.6436437E38F;
            p241.clipping_2 = (uint)2643606972U;
            p241.clipping_1 = (uint)3241184989U;
            p241.vibration_x = (float)2.775815E38F;
            p241.clipping_0 = (uint)1098349622U;
            p241.vibration_z = (float) -3.220741E38F;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int)1803317029);
                Debug.Assert(pack.latitude == (int) -415743851);
                Debug.Assert(pack.approach_x == (float) -6.912907E37F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)2093294942884906846L);
                Debug.Assert(pack.approach_y == (float)2.7839956E38F);
                Debug.Assert(pack.y == (float) -1.3141552E38F);
                Debug.Assert(pack.longitude == (int)734004401);
                Debug.Assert(pack.q.SequenceEqual(new float[] {8.562995E37F, -3.354694E38F, -1.9249865E38F, 2.312389E38F}));
                Debug.Assert(pack.z == (float)9.140796E37F);
                Debug.Assert(pack.approach_z == (float)3.1662025E37F);
                Debug.Assert(pack.x == (float)4.0745082E37F);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.time_usec_SET((ulong)2093294942884906846L, PH) ;
            p242.z = (float)9.140796E37F;
            p242.y = (float) -1.3141552E38F;
            p242.approach_y = (float)2.7839956E38F;
            p242.x = (float)4.0745082E37F;
            p242.latitude = (int) -415743851;
            p242.q_SET(new float[] {8.562995E37F, -3.354694E38F, -1.9249865E38F, 2.312389E38F}, 0) ;
            p242.longitude = (int)734004401;
            p242.approach_z = (float)3.1662025E37F;
            p242.altitude = (int)1803317029;
            p242.approach_x = (float) -6.912907E37F;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.approach_y == (float)2.2531924E38F);
                Debug.Assert(pack.x == (float)2.9857796E38F);
                Debug.Assert(pack.latitude == (int) -47583620);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)2297943942216696245L);
                Debug.Assert(pack.approach_x == (float)2.1209886E38F);
                Debug.Assert(pack.target_system == (byte)(byte)160);
                Debug.Assert(pack.approach_z == (float)5.5600737E37F);
                Debug.Assert(pack.y == (float) -1.178613E38F);
                Debug.Assert(pack.longitude == (int) -965455693);
                Debug.Assert(pack.altitude == (int) -294638526);
                Debug.Assert(pack.z == (float) -1.7014784E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.0799128E37F, -1.6232251E38F, -2.9941236E38F, -3.2549608E38F}));
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.q_SET(new float[] {-3.0799128E37F, -1.6232251E38F, -2.9941236E38F, -3.2549608E38F}, 0) ;
            p243.approach_x = (float)2.1209886E38F;
            p243.approach_y = (float)2.2531924E38F;
            p243.time_usec_SET((ulong)2297943942216696245L, PH) ;
            p243.longitude = (int) -965455693;
            p243.x = (float)2.9857796E38F;
            p243.target_system = (byte)(byte)160;
            p243.approach_z = (float)5.5600737E37F;
            p243.latitude = (int) -47583620;
            p243.y = (float) -1.178613E38F;
            p243.z = (float) -1.7014784E38F;
            p243.altitude = (int) -294638526;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)24592);
                Debug.Assert(pack.interval_us == (int) -1986811639);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int) -1986811639;
            p244.message_id = (ushort)(ushort)24592;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
                Debug.Assert(pack.vtol_state == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int)1798060936);
                Debug.Assert(pack.lon == (int) -225058634);
                Debug.Assert(pack.flags == (ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN));
                Debug.Assert(pack.lat == (int)441483271);
                Debug.Assert(pack.ICAO_address == (uint)2725524681U);
                Debug.Assert(pack.squawk == (ushort)(ushort)37496);
                Debug.Assert(pack.tslc == (byte)(byte)59);
                Debug.Assert(pack.ver_velocity == (short)(short)18571);
                Debug.Assert(pack.callsign_LEN(ph) == 2);
                Debug.Assert(pack.callsign_TRY(ph).Equals("dk"));
                Debug.Assert(pack.emitter_type == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_PARACHUTE);
                Debug.Assert(pack.heading == (ushort)(ushort)22394);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)23067);
                Debug.Assert(pack.altitude_type == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN);
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.altitude = (int)1798060936;
            p246.callsign_SET("dk", PH) ;
            p246.lat = (int)441483271;
            p246.tslc = (byte)(byte)59;
            p246.lon = (int) -225058634;
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_PARACHUTE;
            p246.ver_velocity = (short)(short)18571;
            p246.hor_velocity = (ushort)(ushort)23067;
            p246.ICAO_address = (uint)2725524681U;
            p246.heading = (ushort)(ushort)22394;
            p246.squawk = (ushort)(ushort)37496;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_to_minimum_delta == (float) -3.2309045E38F);
                Debug.Assert(pack.threat_level == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
                Debug.Assert(pack.altitude_minimum_delta == (float)3.2810572E38F);
                Debug.Assert(pack.horizontal_minimum_delta == (float) -1.9309116E38F);
                Debug.Assert(pack.action == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY);
                Debug.Assert(pack.src_ == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
                Debug.Assert(pack.id == (uint)1436100608U);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.time_to_minimum_delta = (float) -3.2309045E38F;
            p247.altitude_minimum_delta = (float)3.2810572E38F;
            p247.horizontal_minimum_delta = (float) -1.9309116E38F;
            p247.threat_level = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY;
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.id = (uint)1436100608U;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)228);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)67, (byte)253, (byte)176, (byte)77, (byte)140, (byte)22, (byte)0, (byte)0, (byte)252, (byte)151, (byte)128, (byte)91, (byte)207, (byte)103, (byte)232, (byte)58, (byte)162, (byte)18, (byte)146, (byte)143, (byte)157, (byte)84, (byte)199, (byte)49, (byte)46, (byte)101, (byte)232, (byte)148, (byte)217, (byte)247, (byte)13, (byte)72, (byte)200, (byte)253, (byte)151, (byte)69, (byte)37, (byte)180, (byte)238, (byte)70, (byte)232, (byte)29, (byte)176, (byte)80, (byte)20, (byte)139, (byte)36, (byte)157, (byte)227, (byte)212, (byte)181, (byte)46, (byte)174, (byte)244, (byte)252, (byte)77, (byte)24, (byte)82, (byte)116, (byte)231, (byte)143, (byte)8, (byte)168, (byte)70, (byte)34, (byte)144, (byte)252, (byte)72, (byte)117, (byte)203, (byte)47, (byte)74, (byte)67, (byte)200, (byte)131, (byte)120, (byte)55, (byte)172, (byte)91, (byte)211, (byte)72, (byte)83, (byte)19, (byte)145, (byte)161, (byte)174, (byte)193, (byte)118, (byte)88, (byte)133, (byte)92, (byte)30, (byte)150, (byte)118, (byte)131, (byte)101, (byte)183, (byte)20, (byte)210, (byte)178, (byte)66, (byte)113, (byte)176, (byte)176, (byte)188, (byte)220, (byte)183, (byte)244, (byte)206, (byte)110, (byte)231, (byte)155, (byte)120, (byte)109, (byte)211, (byte)17, (byte)140, (byte)15, (byte)178, (byte)41, (byte)193, (byte)223, (byte)228, (byte)177, (byte)252, (byte)40, (byte)112, (byte)73, (byte)109, (byte)200, (byte)235, (byte)37, (byte)71, (byte)249, (byte)220, (byte)22, (byte)192, (byte)120, (byte)246, (byte)194, (byte)191, (byte)182, (byte)78, (byte)173, (byte)251, (byte)146, (byte)8, (byte)41, (byte)50, (byte)214, (byte)195, (byte)116, (byte)106, (byte)231, (byte)40, (byte)149, (byte)240, (byte)63, (byte)38, (byte)230, (byte)40, (byte)125, (byte)237, (byte)41, (byte)244, (byte)39, (byte)250, (byte)143, (byte)249, (byte)208, (byte)132, (byte)57, (byte)51, (byte)65, (byte)186, (byte)68, (byte)92, (byte)246, (byte)6, (byte)241, (byte)143, (byte)125, (byte)234, (byte)90, (byte)65, (byte)34, (byte)90, (byte)33, (byte)248, (byte)95, (byte)19, (byte)96, (byte)166, (byte)252, (byte)241, (byte)181, (byte)118, (byte)204, (byte)223, (byte)209, (byte)117, (byte)47, (byte)17, (byte)69, (byte)14, (byte)146, (byte)11, (byte)231, (byte)166, (byte)21, (byte)8, (byte)39, (byte)75, (byte)94, (byte)32, (byte)38, (byte)62, (byte)128, (byte)219, (byte)156, (byte)101, (byte)182, (byte)68, (byte)42, (byte)223, (byte)233, (byte)157, (byte)66, (byte)187, (byte)18, (byte)176, (byte)149, (byte)71, (byte)204, (byte)237, (byte)165, (byte)217, (byte)173, (byte)63, (byte)88, (byte)184, (byte)169, (byte)149, (byte)63, (byte)118, (byte)239, (byte)124, (byte)237, (byte)12}));
                Debug.Assert(pack.target_network == (byte)(byte)167);
                Debug.Assert(pack.message_type == (ushort)(ushort)24131);
                Debug.Assert(pack.target_component == (byte)(byte)202);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_component = (byte)(byte)202;
            p248.target_network = (byte)(byte)167;
            p248.message_type = (ushort)(ushort)24131;
            p248.target_system = (byte)(byte)228;
            p248.payload_SET(new byte[] {(byte)67, (byte)253, (byte)176, (byte)77, (byte)140, (byte)22, (byte)0, (byte)0, (byte)252, (byte)151, (byte)128, (byte)91, (byte)207, (byte)103, (byte)232, (byte)58, (byte)162, (byte)18, (byte)146, (byte)143, (byte)157, (byte)84, (byte)199, (byte)49, (byte)46, (byte)101, (byte)232, (byte)148, (byte)217, (byte)247, (byte)13, (byte)72, (byte)200, (byte)253, (byte)151, (byte)69, (byte)37, (byte)180, (byte)238, (byte)70, (byte)232, (byte)29, (byte)176, (byte)80, (byte)20, (byte)139, (byte)36, (byte)157, (byte)227, (byte)212, (byte)181, (byte)46, (byte)174, (byte)244, (byte)252, (byte)77, (byte)24, (byte)82, (byte)116, (byte)231, (byte)143, (byte)8, (byte)168, (byte)70, (byte)34, (byte)144, (byte)252, (byte)72, (byte)117, (byte)203, (byte)47, (byte)74, (byte)67, (byte)200, (byte)131, (byte)120, (byte)55, (byte)172, (byte)91, (byte)211, (byte)72, (byte)83, (byte)19, (byte)145, (byte)161, (byte)174, (byte)193, (byte)118, (byte)88, (byte)133, (byte)92, (byte)30, (byte)150, (byte)118, (byte)131, (byte)101, (byte)183, (byte)20, (byte)210, (byte)178, (byte)66, (byte)113, (byte)176, (byte)176, (byte)188, (byte)220, (byte)183, (byte)244, (byte)206, (byte)110, (byte)231, (byte)155, (byte)120, (byte)109, (byte)211, (byte)17, (byte)140, (byte)15, (byte)178, (byte)41, (byte)193, (byte)223, (byte)228, (byte)177, (byte)252, (byte)40, (byte)112, (byte)73, (byte)109, (byte)200, (byte)235, (byte)37, (byte)71, (byte)249, (byte)220, (byte)22, (byte)192, (byte)120, (byte)246, (byte)194, (byte)191, (byte)182, (byte)78, (byte)173, (byte)251, (byte)146, (byte)8, (byte)41, (byte)50, (byte)214, (byte)195, (byte)116, (byte)106, (byte)231, (byte)40, (byte)149, (byte)240, (byte)63, (byte)38, (byte)230, (byte)40, (byte)125, (byte)237, (byte)41, (byte)244, (byte)39, (byte)250, (byte)143, (byte)249, (byte)208, (byte)132, (byte)57, (byte)51, (byte)65, (byte)186, (byte)68, (byte)92, (byte)246, (byte)6, (byte)241, (byte)143, (byte)125, (byte)234, (byte)90, (byte)65, (byte)34, (byte)90, (byte)33, (byte)248, (byte)95, (byte)19, (byte)96, (byte)166, (byte)252, (byte)241, (byte)181, (byte)118, (byte)204, (byte)223, (byte)209, (byte)117, (byte)47, (byte)17, (byte)69, (byte)14, (byte)146, (byte)11, (byte)231, (byte)166, (byte)21, (byte)8, (byte)39, (byte)75, (byte)94, (byte)32, (byte)38, (byte)62, (byte)128, (byte)219, (byte)156, (byte)101, (byte)182, (byte)68, (byte)42, (byte)223, (byte)233, (byte)157, (byte)66, (byte)187, (byte)18, (byte)176, (byte)149, (byte)71, (byte)204, (byte)237, (byte)165, (byte)217, (byte)173, (byte)63, (byte)88, (byte)184, (byte)169, (byte)149, (byte)63, (byte)118, (byte)239, (byte)124, (byte)237, (byte)12}, 0) ;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.address == (ushort)(ushort)20263);
                Debug.Assert(pack.type == (byte)(byte)237);
                Debug.Assert(pack.ver == (byte)(byte)225);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte)66, (sbyte)34, (sbyte) - 54, (sbyte) - 76, (sbyte)76, (sbyte)106, (sbyte) - 37, (sbyte) - 2, (sbyte)125, (sbyte)79, (sbyte) - 58, (sbyte)64, (sbyte) - 80, (sbyte) - 97, (sbyte) - 124, (sbyte) - 101, (sbyte) - 78, (sbyte)98, (sbyte) - 21, (sbyte) - 80, (sbyte)22, (sbyte)52, (sbyte)13, (sbyte) - 85, (sbyte) - 90, (sbyte)15, (sbyte) - 16, (sbyte)123, (sbyte) - 50, (sbyte) - 2, (sbyte)71, (sbyte)43}));
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)20263;
            p249.type = (byte)(byte)237;
            p249.ver = (byte)(byte)225;
            p249.value_SET(new sbyte[] {(sbyte)66, (sbyte)34, (sbyte) - 54, (sbyte) - 76, (sbyte)76, (sbyte)106, (sbyte) - 37, (sbyte) - 2, (sbyte)125, (sbyte)79, (sbyte) - 58, (sbyte)64, (sbyte) - 80, (sbyte) - 97, (sbyte) - 124, (sbyte) - 101, (sbyte) - 78, (sbyte)98, (sbyte) - 21, (sbyte) - 80, (sbyte)22, (sbyte)52, (sbyte)13, (sbyte) - 85, (sbyte) - 90, (sbyte)15, (sbyte) - 16, (sbyte)123, (sbyte) - 50, (sbyte) - 2, (sbyte)71, (sbyte)43}, 0) ;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)3.7390782E37F);
                Debug.Assert(pack.name_LEN(ph) == 1);
                Debug.Assert(pack.name_TRY(ph).Equals("o"));
                Debug.Assert(pack.x == (float)1.6060047E38F);
                Debug.Assert(pack.time_usec == (ulong)5575727660120984174L);
                Debug.Assert(pack.z == (float)2.014763E38F);
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.z = (float)2.014763E38F;
            p250.name_SET("o", PH) ;
            p250.time_usec = (ulong)5575727660120984174L;
            p250.y = (float)3.7390782E37F;
            p250.x = (float)1.6060047E38F;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float)2.5931777E38F);
                Debug.Assert(pack.name_LEN(ph) == 4);
                Debug.Assert(pack.name_TRY(ph).Equals("uilg"));
                Debug.Assert(pack.time_boot_ms == (uint)711127236U);
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.name_SET("uilg", PH) ;
            p251.value = (float)2.5931777E38F;
            p251.time_boot_ms = (uint)711127236U;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (int)1736736444);
                Debug.Assert(pack.time_boot_ms == (uint)2019823843U);
                Debug.Assert(pack.name_LEN(ph) == 3);
                Debug.Assert(pack.name_TRY(ph).Equals("ckb"));
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.name_SET("ckb", PH) ;
            p252.time_boot_ms = (uint)2019823843U;
            p252.value = (int)1736736444;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 19);
                Debug.Assert(pack.text_TRY(ph).Equals("ThmpzofuDqtttgcevfe"));
                Debug.Assert(pack.severity == MAV_SEVERITY.MAV_SEVERITY_INFO);
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_INFO;
            p253.text_SET("ThmpzofuDqtttgcevfe", PH) ;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float)2.9664486E38F);
                Debug.Assert(pack.ind == (byte)(byte)28);
                Debug.Assert(pack.time_boot_ms == (uint)1459563012U);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.value = (float)2.9664486E38F;
            p254.time_boot_ms = (uint)1459563012U;
            p254.ind = (byte)(byte)28;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)114, (byte)159, (byte)162, (byte)161, (byte)156, (byte)183, (byte)136, (byte)153, (byte)159, (byte)78, (byte)74, (byte)206, (byte)214, (byte)52, (byte)155, (byte)185, (byte)237, (byte)17, (byte)34, (byte)132, (byte)104, (byte)210, (byte)104, (byte)25, (byte)252, (byte)21, (byte)254, (byte)239, (byte)209, (byte)228, (byte)216, (byte)177}));
                Debug.Assert(pack.initial_timestamp == (ulong)4198126199341126296L);
                Debug.Assert(pack.target_component == (byte)(byte)195);
                Debug.Assert(pack.target_system == (byte)(byte)230);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.initial_timestamp = (ulong)4198126199341126296L;
            p256.secret_key_SET(new byte[] {(byte)114, (byte)159, (byte)162, (byte)161, (byte)156, (byte)183, (byte)136, (byte)153, (byte)159, (byte)78, (byte)74, (byte)206, (byte)214, (byte)52, (byte)155, (byte)185, (byte)237, (byte)17, (byte)34, (byte)132, (byte)104, (byte)210, (byte)104, (byte)25, (byte)252, (byte)21, (byte)254, (byte)239, (byte)209, (byte)228, (byte)216, (byte)177}, 0) ;
            p256.target_component = (byte)(byte)195;
            p256.target_system = (byte)(byte)230;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.state == (byte)(byte)180);
                Debug.Assert(pack.time_boot_ms == (uint)4084508324U);
                Debug.Assert(pack.last_change_ms == (uint)2030276308U);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.last_change_ms = (uint)2030276308U;
            p257.time_boot_ms = (uint)4084508324U;
            p257.state = (byte)(byte)180;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)93);
                Debug.Assert(pack.tune_LEN(ph) == 15);
                Debug.Assert(pack.tune_TRY(ph).Equals("tsfmgcyhrcsdjeg"));
                Debug.Assert(pack.target_component == (byte)(byte)10);
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.tune_SET("tsfmgcyhrcsdjeg", PH) ;
            p258.target_system = (byte)(byte)93;
            p258.target_component = (byte)(byte)10;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.firmware_version == (uint)389663427U);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)59285);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)50, (byte)112, (byte)108, (byte)185, (byte)186, (byte)108, (byte)63, (byte)205, (byte)75, (byte)12, (byte)120, (byte)242, (byte)218, (byte)82, (byte)25, (byte)39, (byte)205, (byte)253, (byte)227, (byte)205, (byte)24, (byte)98, (byte)185, (byte)211, (byte)58, (byte)202, (byte)58, (byte)40, (byte)175, (byte)127, (byte)255, (byte)173}));
                Debug.Assert(pack.time_boot_ms == (uint)4055048608U);
                Debug.Assert(pack.focal_length == (float) -5.349022E37F);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)20190);
                Debug.Assert(pack.sensor_size_h == (float) -2.4539939E38F);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE));
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)149, (byte)242, (byte)121, (byte)142, (byte)194, (byte)241, (byte)190, (byte)24, (byte)169, (byte)229, (byte)166, (byte)116, (byte)242, (byte)246, (byte)211, (byte)135, (byte)135, (byte)99, (byte)172, (byte)51, (byte)164, (byte)243, (byte)1, (byte)84, (byte)214, (byte)180, (byte)160, (byte)195, (byte)242, (byte)225, (byte)53, (byte)216}));
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)39968);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 109);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("qtiLeZvbffzvBqBOgryfffhpowpghrnwXlMriiufybrhjkpcqvegylqucjwreekabgldgorvxkrathdhzUubqalwzjtizrhbtrpxjsryrjwdk"));
                Debug.Assert(pack.lens_id == (byte)(byte)137);
                Debug.Assert(pack.sensor_size_v == (float)2.1635068E38F);
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.sensor_size_h = (float) -2.4539939E38F;
            p259.lens_id = (byte)(byte)137;
            p259.cam_definition_version = (ushort)(ushort)39968;
            p259.sensor_size_v = (float)2.1635068E38F;
            p259.resolution_v = (ushort)(ushort)20190;
            p259.vendor_name_SET(new byte[] {(byte)149, (byte)242, (byte)121, (byte)142, (byte)194, (byte)241, (byte)190, (byte)24, (byte)169, (byte)229, (byte)166, (byte)116, (byte)242, (byte)246, (byte)211, (byte)135, (byte)135, (byte)99, (byte)172, (byte)51, (byte)164, (byte)243, (byte)1, (byte)84, (byte)214, (byte)180, (byte)160, (byte)195, (byte)242, (byte)225, (byte)53, (byte)216}, 0) ;
            p259.time_boot_ms = (uint)4055048608U;
            p259.model_name_SET(new byte[] {(byte)50, (byte)112, (byte)108, (byte)185, (byte)186, (byte)108, (byte)63, (byte)205, (byte)75, (byte)12, (byte)120, (byte)242, (byte)218, (byte)82, (byte)25, (byte)39, (byte)205, (byte)253, (byte)227, (byte)205, (byte)24, (byte)98, (byte)185, (byte)211, (byte)58, (byte)202, (byte)58, (byte)40, (byte)175, (byte)127, (byte)255, (byte)173}, 0) ;
            p259.focal_length = (float) -5.349022E37F;
            p259.cam_definition_uri_SET("qtiLeZvbffzvBqBOgryfffhpowpghrnwXlMriiufybrhjkpcqvegylqucjwreekabgldgorvxkrathdhzUubqalwzjtizrhbtrpxjsryrjwdk", PH) ;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE);
            p259.resolution_h = (ushort)(ushort)59285;
            p259.firmware_version = (uint)389663427U;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3653884513U);
                Debug.Assert(pack.mode_id == CAMERA_MODE.CAMERA_MODE_VIDEO);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)3653884513U;
            p260.mode_id = CAMERA_MODE.CAMERA_MODE_VIDEO;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.available_capacity == (float) -9.359971E37F);
                Debug.Assert(pack.write_speed == (float)2.1823412E38F);
                Debug.Assert(pack.used_capacity == (float)1.7387017E38F);
                Debug.Assert(pack.total_capacity == (float) -2.9125794E38F);
                Debug.Assert(pack.storage_count == (byte)(byte)158);
                Debug.Assert(pack.time_boot_ms == (uint)3973964824U);
                Debug.Assert(pack.status == (byte)(byte)75);
                Debug.Assert(pack.read_speed == (float) -4.6219916E36F);
                Debug.Assert(pack.storage_id == (byte)(byte)246);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.storage_count = (byte)(byte)158;
            p261.write_speed = (float)2.1823412E38F;
            p261.total_capacity = (float) -2.9125794E38F;
            p261.available_capacity = (float) -9.359971E37F;
            p261.status = (byte)(byte)75;
            p261.used_capacity = (float)1.7387017E38F;
            p261.read_speed = (float) -4.6219916E36F;
            p261.storage_id = (byte)(byte)246;
            p261.time_boot_ms = (uint)3973964824U;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.available_capacity == (float) -1.1066283E38F);
                Debug.Assert(pack.video_status == (byte)(byte)222);
                Debug.Assert(pack.image_interval == (float) -1.9118406E38F);
                Debug.Assert(pack.recording_time_ms == (uint)1520026929U);
                Debug.Assert(pack.time_boot_ms == (uint)3747709502U);
                Debug.Assert(pack.image_status == (byte)(byte)73);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.image_status = (byte)(byte)73;
            p262.video_status = (byte)(byte)222;
            p262.time_boot_ms = (uint)3747709502U;
            p262.available_capacity = (float) -1.1066283E38F;
            p262.image_interval = (float) -1.9118406E38F;
            p262.recording_time_ms = (uint)1520026929U;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_utc == (ulong)7416180108333973051L);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.0698883E38F, -2.2748676E38F, -2.4631597E38F, 2.273021E38F}));
                Debug.Assert(pack.image_index == (int)693299381);
                Debug.Assert(pack.lon == (int)1426640262);
                Debug.Assert(pack.lat == (int)1946691523);
                Debug.Assert(pack.camera_id == (byte)(byte)70);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)100);
                Debug.Assert(pack.alt == (int) -1297004083);
                Debug.Assert(pack.time_boot_ms == (uint)2705273590U);
                Debug.Assert(pack.relative_alt == (int) -213155453);
                Debug.Assert(pack.file_url_LEN(ph) == 60);
                Debug.Assert(pack.file_url_TRY(ph).Equals("wnozpvclwpmiXzwlcmDaepoeGetcqlsvfmQoFprRdtktoabMctAqdTrfjhwx"));
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.camera_id = (byte)(byte)70;
            p263.file_url_SET("wnozpvclwpmiXzwlcmDaepoeGetcqlsvfmQoFprRdtktoabMctAqdTrfjhwx", PH) ;
            p263.alt = (int) -1297004083;
            p263.capture_result = (sbyte)(sbyte)100;
            p263.lon = (int)1426640262;
            p263.q_SET(new float[] {2.0698883E38F, -2.2748676E38F, -2.4631597E38F, 2.273021E38F}, 0) ;
            p263.relative_alt = (int) -213155453;
            p263.time_utc = (ulong)7416180108333973051L;
            p263.image_index = (int)693299381;
            p263.time_boot_ms = (uint)2705273590U;
            p263.lat = (int)1946691523;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.takeoff_time_utc == (ulong)6318652473831132046L);
                Debug.Assert(pack.time_boot_ms == (uint)1403317268U);
                Debug.Assert(pack.flight_uuid == (ulong)7362991757783861294L);
                Debug.Assert(pack.arming_time_utc == (ulong)7409125111284610870L);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.flight_uuid = (ulong)7362991757783861294L;
            p264.time_boot_ms = (uint)1403317268U;
            p264.arming_time_utc = (ulong)7409125111284610870L;
            p264.takeoff_time_utc = (ulong)6318652473831132046L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3227116519U);
                Debug.Assert(pack.yaw == (float)4.638751E37F);
                Debug.Assert(pack.roll == (float)7.02057E37F);
                Debug.Assert(pack.pitch == (float)2.667593E38F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.roll = (float)7.02057E37F;
            p265.time_boot_ms = (uint)3227116519U;
            p265.pitch = (float)2.667593E38F;
            p265.yaw = (float)4.638751E37F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sequence == (ushort)(ushort)19229);
                Debug.Assert(pack.target_system == (byte)(byte)122);
                Debug.Assert(pack.first_message_offset == (byte)(byte)214);
                Debug.Assert(pack.target_component == (byte)(byte)121);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)194, (byte)216, (byte)59, (byte)236, (byte)221, (byte)114, (byte)49, (byte)44, (byte)77, (byte)192, (byte)59, (byte)96, (byte)241, (byte)250, (byte)114, (byte)24, (byte)219, (byte)17, (byte)59, (byte)105, (byte)244, (byte)204, (byte)71, (byte)75, (byte)77, (byte)20, (byte)20, (byte)16, (byte)181, (byte)110, (byte)190, (byte)209, (byte)209, (byte)187, (byte)55, (byte)81, (byte)95, (byte)24, (byte)95, (byte)177, (byte)28, (byte)137, (byte)85, (byte)156, (byte)240, (byte)59, (byte)89, (byte)161, (byte)118, (byte)27, (byte)177, (byte)80, (byte)149, (byte)215, (byte)59, (byte)70, (byte)228, (byte)53, (byte)93, (byte)212, (byte)46, (byte)170, (byte)208, (byte)95, (byte)248, (byte)122, (byte)217, (byte)129, (byte)13, (byte)65, (byte)201, (byte)20, (byte)118, (byte)44, (byte)148, (byte)207, (byte)111, (byte)127, (byte)50, (byte)115, (byte)143, (byte)216, (byte)46, (byte)243, (byte)94, (byte)141, (byte)17, (byte)167, (byte)82, (byte)167, (byte)67, (byte)238, (byte)123, (byte)107, (byte)154, (byte)15, (byte)95, (byte)208, (byte)166, (byte)237, (byte)221, (byte)78, (byte)133, (byte)35, (byte)1, (byte)63, (byte)196, (byte)191, (byte)167, (byte)89, (byte)203, (byte)239, (byte)240, (byte)67, (byte)63, (byte)219, (byte)181, (byte)151, (byte)179, (byte)52, (byte)234, (byte)168, (byte)180, (byte)38, (byte)185, (byte)198, (byte)83, (byte)74, (byte)95, (byte)119, (byte)224, (byte)25, (byte)69, (byte)169, (byte)221, (byte)32, (byte)55, (byte)11, (byte)102, (byte)179, (byte)170, (byte)223, (byte)3, (byte)141, (byte)64, (byte)141, (byte)62, (byte)21, (byte)226, (byte)26, (byte)199, (byte)93, (byte)235, (byte)187, (byte)206, (byte)17, (byte)6, (byte)199, (byte)117, (byte)49, (byte)84, (byte)180, (byte)122, (byte)83, (byte)32, (byte)114, (byte)62, (byte)200, (byte)184, (byte)240, (byte)246, (byte)21, (byte)80, (byte)155, (byte)126, (byte)39, (byte)30, (byte)158, (byte)20, (byte)44, (byte)119, (byte)78, (byte)224, (byte)211, (byte)105, (byte)85, (byte)216, (byte)159, (byte)115, (byte)0, (byte)220, (byte)141, (byte)98, (byte)232, (byte)23, (byte)112, (byte)56, (byte)67, (byte)153, (byte)200, (byte)16, (byte)245, (byte)22, (byte)95, (byte)164, (byte)163, (byte)132, (byte)155, (byte)161, (byte)90, (byte)25, (byte)208, (byte)236, (byte)169, (byte)150, (byte)155, (byte)176, (byte)205, (byte)242, (byte)102, (byte)45, (byte)169, (byte)55, (byte)234, (byte)254, (byte)163, (byte)166, (byte)195, (byte)28, (byte)96, (byte)98, (byte)146, (byte)211, (byte)87, (byte)223, (byte)170, (byte)25, (byte)195, (byte)220, (byte)47, (byte)25, (byte)208, (byte)130, (byte)161, (byte)68, (byte)195, (byte)36, (byte)32, (byte)178}));
                Debug.Assert(pack.length == (byte)(byte)172);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.data__SET(new byte[] {(byte)194, (byte)216, (byte)59, (byte)236, (byte)221, (byte)114, (byte)49, (byte)44, (byte)77, (byte)192, (byte)59, (byte)96, (byte)241, (byte)250, (byte)114, (byte)24, (byte)219, (byte)17, (byte)59, (byte)105, (byte)244, (byte)204, (byte)71, (byte)75, (byte)77, (byte)20, (byte)20, (byte)16, (byte)181, (byte)110, (byte)190, (byte)209, (byte)209, (byte)187, (byte)55, (byte)81, (byte)95, (byte)24, (byte)95, (byte)177, (byte)28, (byte)137, (byte)85, (byte)156, (byte)240, (byte)59, (byte)89, (byte)161, (byte)118, (byte)27, (byte)177, (byte)80, (byte)149, (byte)215, (byte)59, (byte)70, (byte)228, (byte)53, (byte)93, (byte)212, (byte)46, (byte)170, (byte)208, (byte)95, (byte)248, (byte)122, (byte)217, (byte)129, (byte)13, (byte)65, (byte)201, (byte)20, (byte)118, (byte)44, (byte)148, (byte)207, (byte)111, (byte)127, (byte)50, (byte)115, (byte)143, (byte)216, (byte)46, (byte)243, (byte)94, (byte)141, (byte)17, (byte)167, (byte)82, (byte)167, (byte)67, (byte)238, (byte)123, (byte)107, (byte)154, (byte)15, (byte)95, (byte)208, (byte)166, (byte)237, (byte)221, (byte)78, (byte)133, (byte)35, (byte)1, (byte)63, (byte)196, (byte)191, (byte)167, (byte)89, (byte)203, (byte)239, (byte)240, (byte)67, (byte)63, (byte)219, (byte)181, (byte)151, (byte)179, (byte)52, (byte)234, (byte)168, (byte)180, (byte)38, (byte)185, (byte)198, (byte)83, (byte)74, (byte)95, (byte)119, (byte)224, (byte)25, (byte)69, (byte)169, (byte)221, (byte)32, (byte)55, (byte)11, (byte)102, (byte)179, (byte)170, (byte)223, (byte)3, (byte)141, (byte)64, (byte)141, (byte)62, (byte)21, (byte)226, (byte)26, (byte)199, (byte)93, (byte)235, (byte)187, (byte)206, (byte)17, (byte)6, (byte)199, (byte)117, (byte)49, (byte)84, (byte)180, (byte)122, (byte)83, (byte)32, (byte)114, (byte)62, (byte)200, (byte)184, (byte)240, (byte)246, (byte)21, (byte)80, (byte)155, (byte)126, (byte)39, (byte)30, (byte)158, (byte)20, (byte)44, (byte)119, (byte)78, (byte)224, (byte)211, (byte)105, (byte)85, (byte)216, (byte)159, (byte)115, (byte)0, (byte)220, (byte)141, (byte)98, (byte)232, (byte)23, (byte)112, (byte)56, (byte)67, (byte)153, (byte)200, (byte)16, (byte)245, (byte)22, (byte)95, (byte)164, (byte)163, (byte)132, (byte)155, (byte)161, (byte)90, (byte)25, (byte)208, (byte)236, (byte)169, (byte)150, (byte)155, (byte)176, (byte)205, (byte)242, (byte)102, (byte)45, (byte)169, (byte)55, (byte)234, (byte)254, (byte)163, (byte)166, (byte)195, (byte)28, (byte)96, (byte)98, (byte)146, (byte)211, (byte)87, (byte)223, (byte)170, (byte)25, (byte)195, (byte)220, (byte)47, (byte)25, (byte)208, (byte)130, (byte)161, (byte)68, (byte)195, (byte)36, (byte)32, (byte)178}, 0) ;
            p266.target_system = (byte)(byte)122;
            p266.target_component = (byte)(byte)121;
            p266.sequence = (ushort)(ushort)19229;
            p266.length = (byte)(byte)172;
            p266.first_message_offset = (byte)(byte)214;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)174);
                Debug.Assert(pack.first_message_offset == (byte)(byte)157);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)223, (byte)107, (byte)244, (byte)60, (byte)222, (byte)177, (byte)91, (byte)173, (byte)230, (byte)61, (byte)167, (byte)163, (byte)90, (byte)208, (byte)178, (byte)225, (byte)171, (byte)119, (byte)89, (byte)220, (byte)23, (byte)204, (byte)11, (byte)102, (byte)48, (byte)166, (byte)187, (byte)29, (byte)77, (byte)43, (byte)36, (byte)108, (byte)192, (byte)22, (byte)159, (byte)21, (byte)72, (byte)34, (byte)142, (byte)161, (byte)228, (byte)187, (byte)227, (byte)120, (byte)26, (byte)151, (byte)199, (byte)101, (byte)190, (byte)35, (byte)45, (byte)201, (byte)58, (byte)29, (byte)126, (byte)194, (byte)1, (byte)171, (byte)165, (byte)142, (byte)159, (byte)151, (byte)30, (byte)99, (byte)117, (byte)136, (byte)237, (byte)83, (byte)117, (byte)102, (byte)128, (byte)61, (byte)75, (byte)81, (byte)186, (byte)214, (byte)193, (byte)123, (byte)57, (byte)117, (byte)105, (byte)72, (byte)171, (byte)233, (byte)244, (byte)51, (byte)88, (byte)140, (byte)205, (byte)241, (byte)51, (byte)229, (byte)251, (byte)181, (byte)174, (byte)13, (byte)90, (byte)125, (byte)102, (byte)75, (byte)32, (byte)202, (byte)118, (byte)197, (byte)190, (byte)25, (byte)181, (byte)25, (byte)93, (byte)220, (byte)166, (byte)78, (byte)192, (byte)221, (byte)8, (byte)50, (byte)90, (byte)205, (byte)193, (byte)105, (byte)190, (byte)35, (byte)193, (byte)120, (byte)42, (byte)49, (byte)11, (byte)156, (byte)115, (byte)122, (byte)6, (byte)42, (byte)200, (byte)128, (byte)179, (byte)180, (byte)126, (byte)50, (byte)116, (byte)59, (byte)205, (byte)57, (byte)80, (byte)59, (byte)23, (byte)132, (byte)28, (byte)212, (byte)39, (byte)32, (byte)1, (byte)64, (byte)179, (byte)166, (byte)189, (byte)157, (byte)13, (byte)37, (byte)172, (byte)243, (byte)102, (byte)113, (byte)157, (byte)187, (byte)73, (byte)145, (byte)33, (byte)54, (byte)244, (byte)207, (byte)74, (byte)1, (byte)226, (byte)155, (byte)113, (byte)174, (byte)64, (byte)127, (byte)228, (byte)47, (byte)28, (byte)218, (byte)238, (byte)41, (byte)203, (byte)13, (byte)126, (byte)118, (byte)76, (byte)154, (byte)126, (byte)100, (byte)191, (byte)49, (byte)29, (byte)41, (byte)184, (byte)106, (byte)191, (byte)111, (byte)184, (byte)115, (byte)212, (byte)23, (byte)61, (byte)61, (byte)77, (byte)50, (byte)161, (byte)99, (byte)198, (byte)196, (byte)183, (byte)49, (byte)216, (byte)124, (byte)221, (byte)117, (byte)67, (byte)199, (byte)204, (byte)233, (byte)250, (byte)249, (byte)68, (byte)68, (byte)130, (byte)112, (byte)36, (byte)80, (byte)98, (byte)114, (byte)229, (byte)95, (byte)153, (byte)195, (byte)9, (byte)92, (byte)177, (byte)91, (byte)90, (byte)227, (byte)92, (byte)191, (byte)229, (byte)107, (byte)129, (byte)5, (byte)67}));
                Debug.Assert(pack.length == (byte)(byte)102);
                Debug.Assert(pack.target_system == (byte)(byte)222);
                Debug.Assert(pack.sequence == (ushort)(ushort)23948);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_system = (byte)(byte)222;
            p267.data__SET(new byte[] {(byte)223, (byte)107, (byte)244, (byte)60, (byte)222, (byte)177, (byte)91, (byte)173, (byte)230, (byte)61, (byte)167, (byte)163, (byte)90, (byte)208, (byte)178, (byte)225, (byte)171, (byte)119, (byte)89, (byte)220, (byte)23, (byte)204, (byte)11, (byte)102, (byte)48, (byte)166, (byte)187, (byte)29, (byte)77, (byte)43, (byte)36, (byte)108, (byte)192, (byte)22, (byte)159, (byte)21, (byte)72, (byte)34, (byte)142, (byte)161, (byte)228, (byte)187, (byte)227, (byte)120, (byte)26, (byte)151, (byte)199, (byte)101, (byte)190, (byte)35, (byte)45, (byte)201, (byte)58, (byte)29, (byte)126, (byte)194, (byte)1, (byte)171, (byte)165, (byte)142, (byte)159, (byte)151, (byte)30, (byte)99, (byte)117, (byte)136, (byte)237, (byte)83, (byte)117, (byte)102, (byte)128, (byte)61, (byte)75, (byte)81, (byte)186, (byte)214, (byte)193, (byte)123, (byte)57, (byte)117, (byte)105, (byte)72, (byte)171, (byte)233, (byte)244, (byte)51, (byte)88, (byte)140, (byte)205, (byte)241, (byte)51, (byte)229, (byte)251, (byte)181, (byte)174, (byte)13, (byte)90, (byte)125, (byte)102, (byte)75, (byte)32, (byte)202, (byte)118, (byte)197, (byte)190, (byte)25, (byte)181, (byte)25, (byte)93, (byte)220, (byte)166, (byte)78, (byte)192, (byte)221, (byte)8, (byte)50, (byte)90, (byte)205, (byte)193, (byte)105, (byte)190, (byte)35, (byte)193, (byte)120, (byte)42, (byte)49, (byte)11, (byte)156, (byte)115, (byte)122, (byte)6, (byte)42, (byte)200, (byte)128, (byte)179, (byte)180, (byte)126, (byte)50, (byte)116, (byte)59, (byte)205, (byte)57, (byte)80, (byte)59, (byte)23, (byte)132, (byte)28, (byte)212, (byte)39, (byte)32, (byte)1, (byte)64, (byte)179, (byte)166, (byte)189, (byte)157, (byte)13, (byte)37, (byte)172, (byte)243, (byte)102, (byte)113, (byte)157, (byte)187, (byte)73, (byte)145, (byte)33, (byte)54, (byte)244, (byte)207, (byte)74, (byte)1, (byte)226, (byte)155, (byte)113, (byte)174, (byte)64, (byte)127, (byte)228, (byte)47, (byte)28, (byte)218, (byte)238, (byte)41, (byte)203, (byte)13, (byte)126, (byte)118, (byte)76, (byte)154, (byte)126, (byte)100, (byte)191, (byte)49, (byte)29, (byte)41, (byte)184, (byte)106, (byte)191, (byte)111, (byte)184, (byte)115, (byte)212, (byte)23, (byte)61, (byte)61, (byte)77, (byte)50, (byte)161, (byte)99, (byte)198, (byte)196, (byte)183, (byte)49, (byte)216, (byte)124, (byte)221, (byte)117, (byte)67, (byte)199, (byte)204, (byte)233, (byte)250, (byte)249, (byte)68, (byte)68, (byte)130, (byte)112, (byte)36, (byte)80, (byte)98, (byte)114, (byte)229, (byte)95, (byte)153, (byte)195, (byte)9, (byte)92, (byte)177, (byte)91, (byte)90, (byte)227, (byte)92, (byte)191, (byte)229, (byte)107, (byte)129, (byte)5, (byte)67}, 0) ;
            p267.target_component = (byte)(byte)174;
            p267.first_message_offset = (byte)(byte)157;
            p267.sequence = (ushort)(ushort)23948;
            p267.length = (byte)(byte)102;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)165);
                Debug.Assert(pack.sequence == (ushort)(ushort)12562);
                Debug.Assert(pack.target_component == (byte)(byte)141);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_component = (byte)(byte)141;
            p268.sequence = (ushort)(ushort)12562;
            p268.target_system = (byte)(byte)165;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri_LEN(ph) == 119);
                Debug.Assert(pack.uri_TRY(ph).Equals("EroudhijqmpfzckxwdoehfkkoppyrehaqtlficclpXrvwqsgxbjwgkugxgpsTkpejjmtysyHnpqlqxbimlpvPEdcmJjllddvkOecjJydBgUcviuordgqqXv"));
                Debug.Assert(pack.camera_id == (byte)(byte)186);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)60466);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)13368);
                Debug.Assert(pack.framerate == (float) -9.616615E37F);
                Debug.Assert(pack.rotation == (ushort)(ushort)40173);
                Debug.Assert(pack.status == (byte)(byte)88);
                Debug.Assert(pack.bitrate == (uint)1125805625U);
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.status = (byte)(byte)88;
            p269.uri_SET("EroudhijqmpfzckxwdoehfkkoppyrehaqtlficclpXrvwqsgxbjwgkugxgpsTkpejjmtysyHnpqlqxbimlpvPEdcmJjllddvkOecjJydBgUcviuordgqqXv", PH) ;
            p269.bitrate = (uint)1125805625U;
            p269.resolution_h = (ushort)(ushort)13368;
            p269.resolution_v = (ushort)(ushort)60466;
            p269.camera_id = (byte)(byte)186;
            p269.framerate = (float) -9.616615E37F;
            p269.rotation = (ushort)(ushort)40173;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri_LEN(ph) == 170);
                Debug.Assert(pack.uri_TRY(ph).Equals("hzvzsgvmbMlbfwpohcrggfzirgqpcytogsyfldaelgyNVyhkiEPoSrzijhdfeoObblctabGphIrumoTtJmrWvnmyfzdddrvowomqMoxjvpvzqoSfmmnlnlhgdwqxfbchpbYsUjvzxouHeqqqPiRcdqqlearruyujJdvqtlbmwh"));
                Debug.Assert(pack.framerate == (float) -1.1150249E38F);
                Debug.Assert(pack.target_system == (byte)(byte)93);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)32677);
                Debug.Assert(pack.camera_id == (byte)(byte)240);
                Debug.Assert(pack.bitrate == (uint)148530694U);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)44497);
                Debug.Assert(pack.rotation == (ushort)(ushort)16823);
                Debug.Assert(pack.target_component == (byte)(byte)249);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.bitrate = (uint)148530694U;
            p270.camera_id = (byte)(byte)240;
            p270.resolution_v = (ushort)(ushort)44497;
            p270.framerate = (float) -1.1150249E38F;
            p270.uri_SET("hzvzsgvmbMlbfwpohcrggfzirgqpcytogsyfldaelgyNVyhkiEPoSrzijhdfeoObblctabGphIrumoTtJmrWvnmyfzdddrvowomqMoxjvpvzqoSfmmnlnlhgdwqxfbchpbYsUjvzxouHeqqqPiRcdqqlearruyujJdvqtlbmwh", PH) ;
            p270.target_system = (byte)(byte)93;
            p270.rotation = (ushort)(ushort)16823;
            p270.resolution_h = (ushort)(ushort)32677;
            p270.target_component = (byte)(byte)249;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 23);
                Debug.Assert(pack.password_TRY(ph).Equals("pvkwhzwkqfvbdwvbbljaoin"));
                Debug.Assert(pack.ssid_LEN(ph) == 29);
                Debug.Assert(pack.ssid_TRY(ph).Equals("nvcfhqliuogduAjXhfudkktcjjtmo"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("nvcfhqliuogduAjXhfudkktcjjtmo", PH) ;
            p299.password_SET("pvkwhzwkqfvbdwvbbljaoin", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_version == (ushort)(ushort)15137);
                Debug.Assert(pack.max_version == (ushort)(ushort)31352);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)86, (byte)162, (byte)181, (byte)138, (byte)179, (byte)158, (byte)83, (byte)55}));
                Debug.Assert(pack.version == (ushort)(ushort)9979);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)197, (byte)155, (byte)252, (byte)34, (byte)121, (byte)160, (byte)3, (byte)162}));
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.library_version_hash_SET(new byte[] {(byte)86, (byte)162, (byte)181, (byte)138, (byte)179, (byte)158, (byte)83, (byte)55}, 0) ;
            p300.spec_version_hash_SET(new byte[] {(byte)197, (byte)155, (byte)252, (byte)34, (byte)121, (byte)160, (byte)3, (byte)162}, 0) ;
            p300.version = (ushort)(ushort)9979;
            p300.min_version = (ushort)(ushort)15137;
            p300.max_version = (ushort)(ushort)31352;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uptime_sec == (uint)3358150852U);
                Debug.Assert(pack.mode == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)10794);
                Debug.Assert(pack.sub_mode == (byte)(byte)143);
                Debug.Assert(pack.health == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
                Debug.Assert(pack.time_usec == (ulong)4208009780418927517L);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.uptime_sec = (uint)3358150852U;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING;
            p310.time_usec = (ulong)4208009780418927517L;
            p310.sub_mode = (byte)(byte)143;
            p310.vendor_specific_status_code = (ushort)(ushort)10794;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sw_vcs_commit == (uint)2621005554U);
                Debug.Assert(pack.sw_version_major == (byte)(byte)39);
                Debug.Assert(pack.hw_version_major == (byte)(byte)214);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)188);
                Debug.Assert(pack.name_LEN(ph) == 61);
                Debug.Assert(pack.name_TRY(ph).Equals("bcsbwkmTnjnAxiqaeXhjyqvaidqcmJcvkjWgjFxxaolRvgiPDkqwtzHybvhoz"));
                Debug.Assert(pack.time_usec == (ulong)1229928980589654872L);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)7);
                Debug.Assert(pack.uptime_sec == (uint)1670094804U);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)198, (byte)27, (byte)66, (byte)58, (byte)117, (byte)190, (byte)197, (byte)2, (byte)101, (byte)248, (byte)246, (byte)160, (byte)143, (byte)91, (byte)113, (byte)220}));
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.time_usec = (ulong)1229928980589654872L;
            p311.hw_version_minor = (byte)(byte)7;
            p311.hw_version_major = (byte)(byte)214;
            p311.hw_unique_id_SET(new byte[] {(byte)198, (byte)27, (byte)66, (byte)58, (byte)117, (byte)190, (byte)197, (byte)2, (byte)101, (byte)248, (byte)246, (byte)160, (byte)143, (byte)91, (byte)113, (byte)220}, 0) ;
            p311.sw_vcs_commit = (uint)2621005554U;
            p311.sw_version_minor = (byte)(byte)188;
            p311.uptime_sec = (uint)1670094804U;
            p311.name_SET("bcsbwkmTnjnAxiqaeXhjyqvaidqcmJcvkjWgjFxxaolRvgiPDkqwtzHybvhoz", PH) ;
            p311.sw_version_major = (byte)(byte)39;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)193);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("rNmwF"));
                Debug.Assert(pack.param_index == (short)(short) -2366);
                Debug.Assert(pack.target_component == (byte)(byte)136);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)193;
            p320.target_component = (byte)(byte)136;
            p320.param_id_SET("rNmwF", PH) ;
            p320.param_index = (short)(short) -2366;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)163);
                Debug.Assert(pack.target_component == (byte)(byte)255);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_component = (byte)(byte)255;
            p321.target_system = (byte)(byte)163;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (ushort)(ushort)6445);
                Debug.Assert(pack.param_value_LEN(ph) == 36);
                Debug.Assert(pack.param_value_TRY(ph).Equals("ubfhzvrVFsxmPnqfmrqhfyMcSLeshwkqvjpL"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
                Debug.Assert(pack.param_id_LEN(ph) == 11);
                Debug.Assert(pack.param_id_TRY(ph).Equals("qgNltrtavjn"));
                Debug.Assert(pack.param_count == (ushort)(ushort)21784);
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_value_SET("ubfhzvrVFsxmPnqfmrqhfyMcSLeshwkqvjpL", PH) ;
            p322.param_id_SET("qgNltrtavjn", PH) ;
            p322.param_count = (ushort)(ushort)21784;
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            p322.param_index = (ushort)(ushort)6445;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 7);
                Debug.Assert(pack.param_id_TRY(ph).Equals("cvrMlmy"));
                Debug.Assert(pack.param_value_LEN(ph) == 22);
                Debug.Assert(pack.param_value_TRY(ph).Equals("cnQsxrjehvqRyzsdBwioGh"));
                Debug.Assert(pack.target_system == (byte)(byte)83);
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64);
                Debug.Assert(pack.target_component == (byte)(byte)171);
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)83;
            p323.param_id_SET("cvrMlmy", PH) ;
            p323.target_component = (byte)(byte)171;
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64;
            p323.param_value_SET("cnQsxrjehvqRyzsdBwioGh", PH) ;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 79);
                Debug.Assert(pack.param_value_TRY(ph).Equals("peqyOtdklLRniasdintwghmqAqycYjysbUewnnfauKhquUalyahCOoxjiqfhyptidhpdhrrueHrlqiv"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
                Debug.Assert(pack.param_id_LEN(ph) == 6);
                Debug.Assert(pack.param_id_TRY(ph).Equals("dlkhth"));
                Debug.Assert(pack.param_result == PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_value_SET("peqyOtdklLRniasdintwghmqAqycYjysbUewnnfauKhquUalyahCOoxjiqfhyptidhpdhrrueHrlqiv", PH) ;
            p324.param_id_SET("dlkhth", PH) ;
            p324.param_result = PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED;
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.increment == (byte)(byte)136);
                Debug.Assert(pack.min_distance == (ushort)(ushort)37816);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)2949, (ushort)56913, (ushort)7047, (ushort)52101, (ushort)59457, (ushort)8681, (ushort)31759, (ushort)8884, (ushort)32820, (ushort)811, (ushort)64413, (ushort)55846, (ushort)19203, (ushort)46039, (ushort)46555, (ushort)43947, (ushort)31566, (ushort)58234, (ushort)9480, (ushort)32629, (ushort)28068, (ushort)7768, (ushort)13630, (ushort)15583, (ushort)24142, (ushort)16452, (ushort)37899, (ushort)32393, (ushort)4775, (ushort)5372, (ushort)22851, (ushort)33465, (ushort)49408, (ushort)28129, (ushort)34611, (ushort)63960, (ushort)57456, (ushort)18503, (ushort)17545, (ushort)12872, (ushort)44781, (ushort)13328, (ushort)56786, (ushort)32656, (ushort)13429, (ushort)44628, (ushort)28252, (ushort)11783, (ushort)43821, (ushort)22857, (ushort)21030, (ushort)45382, (ushort)2368, (ushort)42574, (ushort)22062, (ushort)17043, (ushort)50395, (ushort)45943, (ushort)52119, (ushort)62419, (ushort)5251, (ushort)61642, (ushort)47198, (ushort)22404, (ushort)6737, (ushort)47994, (ushort)55622, (ushort)50486, (ushort)11000, (ushort)53676, (ushort)4719, (ushort)55100}));
                Debug.Assert(pack.time_usec == (ulong)994051420756891460L);
                Debug.Assert(pack.max_distance == (ushort)(ushort)32358);
                Debug.Assert(pack.sensor_type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.max_distance = (ushort)(ushort)32358;
            p330.time_usec = (ulong)994051420756891460L;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p330.increment = (byte)(byte)136;
            p330.distances_SET(new ushort[] {(ushort)2949, (ushort)56913, (ushort)7047, (ushort)52101, (ushort)59457, (ushort)8681, (ushort)31759, (ushort)8884, (ushort)32820, (ushort)811, (ushort)64413, (ushort)55846, (ushort)19203, (ushort)46039, (ushort)46555, (ushort)43947, (ushort)31566, (ushort)58234, (ushort)9480, (ushort)32629, (ushort)28068, (ushort)7768, (ushort)13630, (ushort)15583, (ushort)24142, (ushort)16452, (ushort)37899, (ushort)32393, (ushort)4775, (ushort)5372, (ushort)22851, (ushort)33465, (ushort)49408, (ushort)28129, (ushort)34611, (ushort)63960, (ushort)57456, (ushort)18503, (ushort)17545, (ushort)12872, (ushort)44781, (ushort)13328, (ushort)56786, (ushort)32656, (ushort)13429, (ushort)44628, (ushort)28252, (ushort)11783, (ushort)43821, (ushort)22857, (ushort)21030, (ushort)45382, (ushort)2368, (ushort)42574, (ushort)22062, (ushort)17043, (ushort)50395, (ushort)45943, (ushort)52119, (ushort)62419, (ushort)5251, (ushort)61642, (ushort)47198, (ushort)22404, (ushort)6737, (ushort)47994, (ushort)55622, (ushort)50486, (ushort)11000, (ushort)53676, (ushort)4719, (ushort)55100}, 0) ;
            p330.min_distance = (ushort)(ushort)37816;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
        }
    }
}