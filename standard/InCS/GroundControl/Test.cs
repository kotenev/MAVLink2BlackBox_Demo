
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
                    ulong id = id__Q(value);
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
                    ulong id = id__x(value);
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
                    ulong id = id__x(value);
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
        new class SET_ACTUATOR_CONTROL_TARGET : GroundControl.SET_ACTUATOR_CONTROL_TARGET
        {
            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            /**
            *Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
            *	this field to difference between instances*/
            public byte group_mlx
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            public byte target_system //System ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  9, 1));}
            }

            public byte target_component //Component ID
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  10, 1));}
            }

            /**
            *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
            *	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
            *	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
            *	mixer to repurpose them as generic outputs*/
            public float[] controls
            {
                get {return controls_GET(new float[8], 0);}
            }
            /**
            *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
            *	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
            *	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
            *	mixer to repurpose them as generic outputs*/
            public float[]controls_GET(float[] dst_ch, int pos)
            {
                for(int BYTE = 11, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
        }
        new class ACTUATOR_CONTROL_TARGET : GroundControl.ACTUATOR_CONTROL_TARGET
        {
            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            /**
            *Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
            *	this field to difference between instances*/
            public byte group_mlx
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            /**
            *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
            *	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
            *	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
            *	mixer to repurpose them as generic outputs*/
            public float[] controls
            {
                get {return controls_GET(new float[8], 0);}
            }
            /**
            *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
            *	motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
            *	(index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
            *	mixer to repurpose them as generic outputs*/
            public float[]controls_GET(float[] dst_ch, int pos)
            {
                for(int BYTE = 9, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
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

            public void OnSET_ACTUATOR_CONTROL_TARGETReceive_direct(Channel src, Inside ph, SET_ACTUATOR_CONTROL_TARGET pack) {OnSET_ACTUATOR_CONTROL_TARGETReceive(this, ph,  pack);}
            public event SET_ACTUATOR_CONTROL_TARGETReceiveHandler OnSET_ACTUATOR_CONTROL_TARGETReceive;
            public delegate void SET_ACTUATOR_CONTROL_TARGETReceiveHandler(Channel src, Inside ph, SET_ACTUATOR_CONTROL_TARGET pack);
            public void OnACTUATOR_CONTROL_TARGETReceive_direct(Channel src, Inside ph, ACTUATOR_CONTROL_TARGET pack) {OnACTUATOR_CONTROL_TARGETReceive(this, ph,  pack);}
            public event ACTUATOR_CONTROL_TARGETReceiveHandler OnACTUATOR_CONTROL_TARGETReceive;
            public delegate void ACTUATOR_CONTROL_TARGETReceiveHandler(Channel src, Inside ph, ACTUATOR_CONTROL_TARGET pack);
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
                    case 139:
                        if(pack == null) return new SET_ACTUATOR_CONTROL_TARGET();
                        OnSET_ACTUATOR_CONTROL_TARGETReceive(this, ph, (SET_ACTUATOR_CONTROL_TARGET) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 140:
                        if(pack == null) return new ACTUATOR_CONTROL_TARGET();
                        OnACTUATOR_CONTROL_TARGETReceive(this, ph, (ACTUATOR_CONTROL_TARGET) pack);//no any host channels can receive this pack. Handle it with test channel handler
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
                Debug.Assert(pack.custom_mode == (uint)714081811U);
                Debug.Assert(pack.system_status == MAV_STATE.MAV_STATE_CRITICAL);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED));
                Debug.Assert(pack.type == MAV_TYPE.MAV_TYPE_SUBMARINE);
                Debug.Assert(pack.autopilot == MAV_AUTOPILOT.MAV_AUTOPILOT_ARDUPILOTMEGA);
                Debug.Assert(pack.mavlink_version == (byte)(byte)90);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
            p0.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_ARDUPILOTMEGA;
            p0.system_status = MAV_STATE.MAV_STATE_CRITICAL;
            p0.custom_mode = (uint)714081811U;
            p0.type = MAV_TYPE.MAV_TYPE_SUBMARINE;
            p0.mavlink_version = (byte)(byte)90;
            ADV_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION));
                Debug.Assert(pack.errors_comm == (ushort)(ushort)44649);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)52233);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)43360);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)54459);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 20);
                Debug.Assert(pack.current_battery == (short)(short)28147);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)5295);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)57398);
                Debug.Assert(pack.load == (ushort)(ushort)24666);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)63305);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.errors_count3 = (ushort)(ushort)52233;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.errors_count1 = (ushort)(ushort)63305;
            p1.battery_remaining = (sbyte)(sbyte) - 20;
            p1.errors_count2 = (ushort)(ushort)57398;
            p1.errors_count4 = (ushort)(ushort)5295;
            p1.drop_rate_comm = (ushort)(ushort)54459;
            p1.errors_comm = (ushort)(ushort)44649;
            p1.current_battery = (short)(short)28147;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION);
            p1.load = (ushort)(ushort)24666;
            p1.voltage_battery = (ushort)(ushort)43360;
            ADV_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)178211183U);
                Debug.Assert(pack.time_unix_usec == (ulong)5748539598731326480L);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)5748539598731326480L;
            p2.time_boot_ms = (uint)178211183U;
            ADV_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -3.0368868E38F);
                Debug.Assert(pack.vz == (float)2.1074227E38F);
                Debug.Assert(pack.afz == (float) -2.7865493E38F);
                Debug.Assert(pack.x == (float) -8.584879E37F);
                Debug.Assert(pack.yaw == (float) -1.1298226E38F);
                Debug.Assert(pack.vy == (float)5.787589E37F);
                Debug.Assert(pack.y == (float) -3.9900269E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)14771);
                Debug.Assert(pack.time_boot_ms == (uint)2820772365U);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.yaw_rate == (float)1.480807E37F);
                Debug.Assert(pack.afy == (float) -3.2466276E38F);
                Debug.Assert(pack.afx == (float)1.7017335E38F);
                Debug.Assert(pack.vx == (float) -1.6740882E38F);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.x = (float) -8.584879E37F;
            p3.time_boot_ms = (uint)2820772365U;
            p3.z = (float) -3.0368868E38F;
            p3.afx = (float)1.7017335E38F;
            p3.afy = (float) -3.2466276E38F;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p3.yaw = (float) -1.1298226E38F;
            p3.type_mask = (ushort)(ushort)14771;
            p3.vz = (float)2.1074227E38F;
            p3.yaw_rate = (float)1.480807E37F;
            p3.y = (float) -3.9900269E37F;
            p3.afz = (float) -2.7865493E38F;
            p3.vx = (float) -1.6740882E38F;
            p3.vy = (float)5.787589E37F;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)160);
                Debug.Assert(pack.time_usec == (ulong)1710374182484471412L);
                Debug.Assert(pack.seq == (uint)1390840204U);
                Debug.Assert(pack.target_component == (byte)(byte)127);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.target_system = (byte)(byte)160;
            p4.time_usec = (ulong)1710374182484471412L;
            p4.target_component = (byte)(byte)127;
            p4.seq = (uint)1390840204U;
            ADV_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.passkey_LEN(ph) == 19);
                Debug.Assert(pack.passkey_TRY(ph).Equals("uuwyalvvzpfkzqQoDii"));
                Debug.Assert(pack.target_system == (byte)(byte)134);
                Debug.Assert(pack.version == (byte)(byte)3);
                Debug.Assert(pack.control_request == (byte)(byte)157);
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.passkey_SET("uuwyalvvzpfkzqQoDii", PH) ;
            p5.control_request = (byte)(byte)157;
            p5.target_system = (byte)(byte)134;
            p5.version = (byte)(byte)3;
            ADV_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ack == (byte)(byte)225);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)156);
                Debug.Assert(pack.control_request == (byte)(byte)43);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.ack = (byte)(byte)225;
            p6.control_request = (byte)(byte)43;
            p6.gcs_system_id = (byte)(byte)156;
            ADV_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 31);
                Debug.Assert(pack.key_TRY(ph).Equals("qhbccHulcwxajbmpobuhseaYyrfbnrD"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("qhbccHulcwxajbmpobuhseaYyrfbnrD", PH) ;
            ADV_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)119);
                Debug.Assert(pack.base_mode == MAV_MODE.MAV_MODE_STABILIZE_ARMED);
                Debug.Assert(pack.custom_mode == (uint)3092945672U);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)119;
            p11.custom_mode = (uint)3092945672U;
            p11.base_mode = MAV_MODE.MAV_MODE_STABILIZE_ARMED;
            ADV_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 9);
                Debug.Assert(pack.param_id_TRY(ph).Equals("hVJkvfkvc"));
                Debug.Assert(pack.target_component == (byte)(byte)40);
                Debug.Assert(pack.target_system == (byte)(byte)150);
                Debug.Assert(pack.param_index == (short)(short) -25274);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_system = (byte)(byte)150;
            p20.param_id_SET("hVJkvfkvc", PH) ;
            p20.target_component = (byte)(byte)40;
            p20.param_index = (short)(short) -25274;
            ADV_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)207);
                Debug.Assert(pack.target_component == (byte)(byte)219);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)219;
            p21.target_system = (byte)(byte)207;
            ADV_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("wcfjOpfm"));
                Debug.Assert(pack.param_index == (ushort)(ushort)7711);
                Debug.Assert(pack.param_value == (float)1.3393929E38F);
                Debug.Assert(pack.param_count == (ushort)(ushort)20174);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64);
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_count = (ushort)(ushort)20174;
            p22.param_value = (float)1.3393929E38F;
            p22.param_index = (ushort)(ushort)7711;
            p22.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64;
            p22.param_id_SET("wcfjOpfm", PH) ;
            ADV_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)184);
                Debug.Assert(pack.param_id_LEN(ph) == 7);
                Debug.Assert(pack.param_id_TRY(ph).Equals("dxhtkom"));
                Debug.Assert(pack.param_value == (float) -8.654398E37F);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
                Debug.Assert(pack.target_component == (byte)(byte)0);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32;
            p23.target_component = (byte)(byte)0;
            p23.param_value = (float) -8.654398E37F;
            p23.target_system = (byte)(byte)184;
            p23.param_id_SET("dxhtkom", PH) ;
            ADV_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (int)92671287);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -1054714594);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)2978228451U);
                Debug.Assert(pack.epv == (ushort)(ushort)52126);
                Debug.Assert(pack.satellites_visible == (byte)(byte)12);
                Debug.Assert(pack.cog == (ushort)(ushort)11634);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)4251270547U);
                Debug.Assert(pack.lon == (int) -2002247065);
                Debug.Assert(pack.lat == (int)897535433);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
                Debug.Assert(pack.time_usec == (ulong)7891344934651972000L);
                Debug.Assert(pack.vel == (ushort)(ushort)54356);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)1935706816U);
                Debug.Assert(pack.eph == (ushort)(ushort)39271);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)2982118447U);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.time_usec = (ulong)7891344934651972000L;
            p24.alt_ellipsoid_SET((int) -1054714594, PH) ;
            p24.eph = (ushort)(ushort)39271;
            p24.hdg_acc_SET((uint)2978228451U, PH) ;
            p24.lon = (int) -2002247065;
            p24.h_acc_SET((uint)2982118447U, PH) ;
            p24.lat = (int)897535433;
            p24.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p24.satellites_visible = (byte)(byte)12;
            p24.epv = (ushort)(ushort)52126;
            p24.vel_acc_SET((uint)4251270547U, PH) ;
            p24.vel = (ushort)(ushort)54356;
            p24.cog = (ushort)(ushort)11634;
            p24.v_acc_SET((uint)1935706816U, PH) ;
            p24.alt = (int)92671287;
            ADV_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellites_visible == (byte)(byte)171);
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)238, (byte)155, (byte)159, (byte)148, (byte)234, (byte)116, (byte)110, (byte)63, (byte)116, (byte)139, (byte)185, (byte)5, (byte)206, (byte)17, (byte)218, (byte)41, (byte)76, (byte)146, (byte)12, (byte)93}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)165, (byte)140, (byte)13, (byte)46, (byte)249, (byte)45, (byte)39, (byte)252, (byte)28, (byte)141, (byte)29, (byte)237, (byte)18, (byte)158, (byte)17, (byte)69, (byte)102, (byte)48, (byte)124, (byte)222}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)58, (byte)170, (byte)165, (byte)190, (byte)13, (byte)198, (byte)168, (byte)192, (byte)195, (byte)165, (byte)35, (byte)48, (byte)15, (byte)85, (byte)105, (byte)3, (byte)204, (byte)26, (byte)128, (byte)52}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)22, (byte)66, (byte)12, (byte)156, (byte)92, (byte)193, (byte)236, (byte)119, (byte)177, (byte)179, (byte)184, (byte)161, (byte)245, (byte)125, (byte)36, (byte)33, (byte)126, (byte)202, (byte)154, (byte)64}));
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)229, (byte)220, (byte)134, (byte)153, (byte)71, (byte)165, (byte)108, (byte)170, (byte)180, (byte)157, (byte)90, (byte)190, (byte)138, (byte)18, (byte)116, (byte)211, (byte)75, (byte)146, (byte)108, (byte)17}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_prn_SET(new byte[] {(byte)165, (byte)140, (byte)13, (byte)46, (byte)249, (byte)45, (byte)39, (byte)252, (byte)28, (byte)141, (byte)29, (byte)237, (byte)18, (byte)158, (byte)17, (byte)69, (byte)102, (byte)48, (byte)124, (byte)222}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)229, (byte)220, (byte)134, (byte)153, (byte)71, (byte)165, (byte)108, (byte)170, (byte)180, (byte)157, (byte)90, (byte)190, (byte)138, (byte)18, (byte)116, (byte)211, (byte)75, (byte)146, (byte)108, (byte)17}, 0) ;
            p25.satellites_visible = (byte)(byte)171;
            p25.satellite_used_SET(new byte[] {(byte)238, (byte)155, (byte)159, (byte)148, (byte)234, (byte)116, (byte)110, (byte)63, (byte)116, (byte)139, (byte)185, (byte)5, (byte)206, (byte)17, (byte)218, (byte)41, (byte)76, (byte)146, (byte)12, (byte)93}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)58, (byte)170, (byte)165, (byte)190, (byte)13, (byte)198, (byte)168, (byte)192, (byte)195, (byte)165, (byte)35, (byte)48, (byte)15, (byte)85, (byte)105, (byte)3, (byte)204, (byte)26, (byte)128, (byte)52}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)22, (byte)66, (byte)12, (byte)156, (byte)92, (byte)193, (byte)236, (byte)119, (byte)177, (byte)179, (byte)184, (byte)161, (byte)245, (byte)125, (byte)36, (byte)33, (byte)126, (byte)202, (byte)154, (byte)64}, 0) ;
            ADV_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (short)(short)1955);
                Debug.Assert(pack.xgyro == (short)(short)21461);
                Debug.Assert(pack.yacc == (short)(short)29070);
                Debug.Assert(pack.xmag == (short)(short) -204);
                Debug.Assert(pack.ygyro == (short)(short)12411);
                Debug.Assert(pack.zacc == (short)(short)13149);
                Debug.Assert(pack.xacc == (short)(short)19038);
                Debug.Assert(pack.ymag == (short)(short)25352);
                Debug.Assert(pack.time_boot_ms == (uint)2459776392U);
                Debug.Assert(pack.zmag == (short)(short)27348);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.zgyro = (short)(short)1955;
            p26.time_boot_ms = (uint)2459776392U;
            p26.ymag = (short)(short)25352;
            p26.xgyro = (short)(short)21461;
            p26.zmag = (short)(short)27348;
            p26.ygyro = (short)(short)12411;
            p26.xmag = (short)(short) -204;
            p26.zacc = (short)(short)13149;
            p26.yacc = (short)(short)29070;
            p26.xacc = (short)(short)19038;
            ADV_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)7991738080472519608L);
                Debug.Assert(pack.xmag == (short)(short) -23062);
                Debug.Assert(pack.zgyro == (short)(short)29573);
                Debug.Assert(pack.xacc == (short)(short) -13281);
                Debug.Assert(pack.ygyro == (short)(short)19352);
                Debug.Assert(pack.xgyro == (short)(short)4587);
                Debug.Assert(pack.yacc == (short)(short) -3627);
                Debug.Assert(pack.zacc == (short)(short)16391);
                Debug.Assert(pack.ymag == (short)(short) -23149);
                Debug.Assert(pack.zmag == (short)(short) -19540);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.zacc = (short)(short)16391;
            p27.xgyro = (short)(short)4587;
            p27.xmag = (short)(short) -23062;
            p27.ymag = (short)(short) -23149;
            p27.yacc = (short)(short) -3627;
            p27.zgyro = (short)(short)29573;
            p27.zmag = (short)(short) -19540;
            p27.xacc = (short)(short) -13281;
            p27.time_usec = (ulong)7991738080472519608L;
            p27.ygyro = (short)(short)19352;
            ADV_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)2736924112386308893L);
                Debug.Assert(pack.temperature == (short)(short) -11186);
                Debug.Assert(pack.press_diff2 == (short)(short)20600);
                Debug.Assert(pack.press_abs == (short)(short) -16854);
                Debug.Assert(pack.press_diff1 == (short)(short) -9140);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_diff1 = (short)(short) -9140;
            p28.time_usec = (ulong)2736924112386308893L;
            p28.press_abs = (short)(short) -16854;
            p28.press_diff2 = (short)(short)20600;
            p28.temperature = (short)(short) -11186;
            ADV_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float)2.5119097E38F);
                Debug.Assert(pack.press_diff == (float)1.4700378E38F);
                Debug.Assert(pack.temperature == (short)(short)14855);
                Debug.Assert(pack.time_boot_ms == (uint)3937426604U);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.temperature = (short)(short)14855;
            p29.time_boot_ms = (uint)3937426604U;
            p29.press_abs = (float)2.5119097E38F;
            p29.press_diff = (float)1.4700378E38F;
            ADV_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float)2.3517937E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1166552045U);
                Debug.Assert(pack.yawspeed == (float)1.7500235E38F);
                Debug.Assert(pack.yaw == (float)2.5896076E38F);
                Debug.Assert(pack.roll == (float)9.328947E37F);
                Debug.Assert(pack.pitch == (float)1.2173069E38F);
                Debug.Assert(pack.pitchspeed == (float)1.2075038E38F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.yaw = (float)2.5896076E38F;
            p30.pitch = (float)1.2173069E38F;
            p30.yawspeed = (float)1.7500235E38F;
            p30.rollspeed = (float)2.3517937E38F;
            p30.pitchspeed = (float)1.2075038E38F;
            p30.time_boot_ms = (uint)1166552045U;
            p30.roll = (float)9.328947E37F;
            ADV_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitchspeed == (float) -1.0675252E38F);
                Debug.Assert(pack.rollspeed == (float)3.3250962E37F);
                Debug.Assert(pack.time_boot_ms == (uint)425720004U);
                Debug.Assert(pack.yawspeed == (float) -2.0898877E38F);
                Debug.Assert(pack.q1 == (float)2.2481057E38F);
                Debug.Assert(pack.q4 == (float) -5.0755797E37F);
                Debug.Assert(pack.q3 == (float) -2.7000869E38F);
                Debug.Assert(pack.q2 == (float) -2.4745033E38F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.q4 = (float) -5.0755797E37F;
            p31.pitchspeed = (float) -1.0675252E38F;
            p31.q1 = (float)2.2481057E38F;
            p31.rollspeed = (float)3.3250962E37F;
            p31.q3 = (float) -2.7000869E38F;
            p31.q2 = (float) -2.4745033E38F;
            p31.time_boot_ms = (uint)425720004U;
            p31.yawspeed = (float) -2.0898877E38F;
            ADV_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1120846438U);
                Debug.Assert(pack.y == (float) -2.3445603E37F);
                Debug.Assert(pack.x == (float)5.1936203E37F);
                Debug.Assert(pack.vx == (float)4.615635E37F);
                Debug.Assert(pack.vz == (float) -9.116947E37F);
                Debug.Assert(pack.vy == (float)2.3480617E38F);
                Debug.Assert(pack.z == (float) -2.5632889E38F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.time_boot_ms = (uint)1120846438U;
            p32.y = (float) -2.3445603E37F;
            p32.vx = (float)4.615635E37F;
            p32.z = (float) -2.5632889E38F;
            p32.vy = (float)2.3480617E38F;
            p32.vz = (float) -9.116947E37F;
            p32.x = (float)5.1936203E37F;
            ADV_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -1583049933);
                Debug.Assert(pack.relative_alt == (int)135834565);
                Debug.Assert(pack.vy == (short)(short) -13412);
                Debug.Assert(pack.time_boot_ms == (uint)2738844775U);
                Debug.Assert(pack.hdg == (ushort)(ushort)20379);
                Debug.Assert(pack.vz == (short)(short) -3947);
                Debug.Assert(pack.lat == (int)2109527395);
                Debug.Assert(pack.alt == (int) -364317556);
                Debug.Assert(pack.vx == (short)(short)20660);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.vy = (short)(short) -13412;
            p33.vz = (short)(short) -3947;
            p33.alt = (int) -364317556;
            p33.vx = (short)(short)20660;
            p33.time_boot_ms = (uint)2738844775U;
            p33.hdg = (ushort)(ushort)20379;
            p33.lat = (int)2109527395;
            p33.lon = (int) -1583049933;
            p33.relative_alt = (int)135834565;
            ADV_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rssi == (byte)(byte)99);
                Debug.Assert(pack.chan1_scaled == (short)(short) -5823);
                Debug.Assert(pack.chan8_scaled == (short)(short) -15334);
                Debug.Assert(pack.chan3_scaled == (short)(short)12924);
                Debug.Assert(pack.chan2_scaled == (short)(short)30816);
                Debug.Assert(pack.port == (byte)(byte)97);
                Debug.Assert(pack.chan4_scaled == (short)(short)28903);
                Debug.Assert(pack.chan5_scaled == (short)(short)27633);
                Debug.Assert(pack.chan6_scaled == (short)(short)28540);
                Debug.Assert(pack.chan7_scaled == (short)(short)17175);
                Debug.Assert(pack.time_boot_ms == (uint)1017180303U);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan1_scaled = (short)(short) -5823;
            p34.chan5_scaled = (short)(short)27633;
            p34.chan6_scaled = (short)(short)28540;
            p34.chan3_scaled = (short)(short)12924;
            p34.chan4_scaled = (short)(short)28903;
            p34.chan8_scaled = (short)(short) -15334;
            p34.time_boot_ms = (uint)1017180303U;
            p34.port = (byte)(byte)97;
            p34.rssi = (byte)(byte)99;
            p34.chan7_scaled = (short)(short)17175;
            p34.chan2_scaled = (short)(short)30816;
            ADV_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)35522);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)15922);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)22315);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)31683);
                Debug.Assert(pack.rssi == (byte)(byte)126);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)56756);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)47030);
                Debug.Assert(pack.time_boot_ms == (uint)1470201556U);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)13677);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)61390);
                Debug.Assert(pack.port == (byte)(byte)176);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.rssi = (byte)(byte)126;
            p35.chan1_raw = (ushort)(ushort)61390;
            p35.chan2_raw = (ushort)(ushort)35522;
            p35.chan3_raw = (ushort)(ushort)56756;
            p35.chan5_raw = (ushort)(ushort)15922;
            p35.chan4_raw = (ushort)(ushort)31683;
            p35.chan6_raw = (ushort)(ushort)13677;
            p35.port = (byte)(byte)176;
            p35.chan8_raw = (ushort)(ushort)22315;
            p35.chan7_raw = (ushort)(ushort)47030;
            p35.time_boot_ms = (uint)1470201556U;
            ADV_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)51405);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)55100);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)17869);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)65358);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)24590);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)32880);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)31660);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)38088);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)22083);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)22192);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)34811);
                Debug.Assert(pack.time_usec == (uint)4287498303U);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)40605);
                Debug.Assert(pack.port == (byte)(byte)182);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)17171);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)26278);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)18650);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)11705);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo5_raw = (ushort)(ushort)40605;
            p36.servo10_raw_SET((ushort)(ushort)32880, PH) ;
            p36.servo1_raw = (ushort)(ushort)38088;
            p36.servo11_raw_SET((ushort)(ushort)34811, PH) ;
            p36.servo15_raw_SET((ushort)(ushort)18650, PH) ;
            p36.servo16_raw_SET((ushort)(ushort)51405, PH) ;
            p36.servo3_raw = (ushort)(ushort)17869;
            p36.servo12_raw_SET((ushort)(ushort)24590, PH) ;
            p36.time_usec = (uint)4287498303U;
            p36.servo13_raw_SET((ushort)(ushort)65358, PH) ;
            p36.servo8_raw = (ushort)(ushort)22192;
            p36.servo6_raw = (ushort)(ushort)26278;
            p36.servo7_raw = (ushort)(ushort)31660;
            p36.servo9_raw_SET((ushort)(ushort)11705, PH) ;
            p36.port = (byte)(byte)182;
            p36.servo14_raw_SET((ushort)(ushort)22083, PH) ;
            p36.servo2_raw = (ushort)(ushort)55100;
            p36.servo4_raw = (ushort)(ushort)17171;
            ADV_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_index == (short)(short) -4575);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.end_index == (short)(short)711);
                Debug.Assert(pack.target_component == (byte)(byte)179);
                Debug.Assert(pack.target_system == (byte)(byte)6);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.target_system = (byte)(byte)6;
            p37.start_index = (short)(short) -4575;
            p37.end_index = (short)(short)711;
            p37.target_component = (byte)(byte)179;
            p37.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            ADV_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)103);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.start_index == (short)(short)28482);
                Debug.Assert(pack.end_index == (short)(short)4578);
                Debug.Assert(pack.target_system == (byte)(byte)30);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.start_index = (short)(short)28482;
            p38.target_system = (byte)(byte)30;
            p38.end_index = (short)(short)4578;
            p38.target_component = (byte)(byte)103;
            p38.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            ADV_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current == (byte)(byte)175);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS);
                Debug.Assert(pack.param1 == (float) -3.273208E37F);
                Debug.Assert(pack.seq == (ushort)(ushort)64121);
                Debug.Assert(pack.autocontinue == (byte)(byte)200);
                Debug.Assert(pack.param4 == (float)2.9337652E38F);
                Debug.Assert(pack.z == (float) -1.6377632E38F);
                Debug.Assert(pack.target_component == (byte)(byte)26);
                Debug.Assert(pack.target_system == (byte)(byte)141);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.x == (float)3.3513962E38F);
                Debug.Assert(pack.y == (float)1.6272882E38F);
                Debug.Assert(pack.param2 == (float) -1.841436E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.param3 == (float) -2.8278573E38F);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.x = (float)3.3513962E38F;
            p39.param3 = (float) -2.8278573E38F;
            p39.autocontinue = (byte)(byte)200;
            p39.param1 = (float) -3.273208E37F;
            p39.y = (float)1.6272882E38F;
            p39.seq = (ushort)(ushort)64121;
            p39.param4 = (float)2.9337652E38F;
            p39.frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p39.current = (byte)(byte)175;
            p39.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p39.command = MAV_CMD.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
            p39.target_system = (byte)(byte)141;
            p39.z = (float) -1.6377632E38F;
            p39.target_component = (byte)(byte)26;
            p39.param2 = (float) -1.841436E38F;
            ADV_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)28462);
                Debug.Assert(pack.target_component == (byte)(byte)175);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_system == (byte)(byte)235);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_system = (byte)(byte)235;
            p40.target_component = (byte)(byte)175;
            p40.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p40.seq = (ushort)(ushort)28462;
            ADV_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)58679);
                Debug.Assert(pack.target_system == (byte)(byte)248);
                Debug.Assert(pack.target_component == (byte)(byte)190);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.seq = (ushort)(ushort)58679;
            p41.target_system = (byte)(byte)248;
            p41.target_component = (byte)(byte)190;
            ADV_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)13812);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)13812;
            ADV_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)73);
                Debug.Assert(pack.target_component == (byte)(byte)122);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p43.target_component = (byte)(byte)122;
            p43.target_system = (byte)(byte)73;
            ADV_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (ushort)(ushort)38399);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)232);
                Debug.Assert(pack.target_system == (byte)(byte)98);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.count = (ushort)(ushort)38399;
            p44.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p44.target_system = (byte)(byte)98;
            p44.target_component = (byte)(byte)232;
            ADV_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)52);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_system == (byte)(byte)47);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_system = (byte)(byte)47;
            p45.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p45.target_component = (byte)(byte)52;
            ADV_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)47935);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)47935;
            ADV_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)103);
                Debug.Assert(pack.target_component == (byte)(byte)6);
                Debug.Assert(pack.type == MAV_MISSION_RESULT.MAV_MISSION_ERROR);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.target_system = (byte)(byte)103;
            p47.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p47.target_component = (byte)(byte)6;
            p47.type = MAV_MISSION_RESULT.MAV_MISSION_ERROR;
            ADV_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int)1366793164);
                Debug.Assert(pack.latitude == (int)590597583);
                Debug.Assert(pack.target_system == (byte)(byte)112);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)8272498377763473579L);
                Debug.Assert(pack.longitude == (int) -71085637);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.longitude = (int) -71085637;
            p48.target_system = (byte)(byte)112;
            p48.latitude = (int)590597583;
            p48.altitude = (int)1366793164;
            p48.time_usec_SET((ulong)8272498377763473579L, PH) ;
            ADV_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int)182122581);
                Debug.Assert(pack.altitude == (int) -2144709500);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7956505346087184441L);
                Debug.Assert(pack.longitude == (int)2128041144);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.altitude = (int) -2144709500;
            p49.time_usec_SET((ulong)7956505346087184441L, PH) ;
            p49.latitude = (int)182122581;
            p49.longitude = (int)2128041144;
            ADV_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value0 == (float) -4.0610237E37F);
                Debug.Assert(pack.param_value_min == (float) -1.5770763E38F);
                Debug.Assert(pack.target_system == (byte)(byte)118);
                Debug.Assert(pack.target_component == (byte)(byte)28);
                Debug.Assert(pack.scale == (float)9.398886E37F);
                Debug.Assert(pack.param_index == (short)(short)31781);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)124);
                Debug.Assert(pack.param_value_max == (float)1.1588918E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 14);
                Debug.Assert(pack.param_id_TRY(ph).Equals("dHvnziwmpzkhmt"));
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.target_system = (byte)(byte)118;
            p50.param_value_max = (float)1.1588918E38F;
            p50.param_index = (short)(short)31781;
            p50.param_id_SET("dHvnziwmpzkhmt", PH) ;
            p50.target_component = (byte)(byte)28;
            p50.param_value0 = (float) -4.0610237E37F;
            p50.parameter_rc_channel_index = (byte)(byte)124;
            p50.scale = (float)9.398886E37F;
            p50.param_value_min = (float) -1.5770763E38F;
            ADV_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)244);
                Debug.Assert(pack.seq == (ushort)(ushort)63217);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)241);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_system = (byte)(byte)241;
            p51.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p51.seq = (ushort)(ushort)63217;
            p51.target_component = (byte)(byte)244;
            ADV_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.p1z == (float)6.159156E37F);
                Debug.Assert(pack.p1x == (float)5.8423993E37F);
                Debug.Assert(pack.p2z == (float)5.207937E37F);
                Debug.Assert(pack.p2x == (float)3.3468183E38F);
                Debug.Assert(pack.target_system == (byte)(byte)92);
                Debug.Assert(pack.p2y == (float)2.1170838E38F);
                Debug.Assert(pack.target_component == (byte)(byte)170);
                Debug.Assert(pack.p1y == (float)2.5950605E37F);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p2y = (float)2.1170838E38F;
            p54.p1y = (float)2.5950605E37F;
            p54.frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p54.p2z = (float)5.207937E37F;
            p54.p1x = (float)5.8423993E37F;
            p54.target_component = (byte)(byte)170;
            p54.target_system = (byte)(byte)92;
            p54.p2x = (float)3.3468183E38F;
            p54.p1z = (float)6.159156E37F;
            ADV_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2y == (float) -2.2881527E38F);
                Debug.Assert(pack.p2x == (float) -2.4729817E38F);
                Debug.Assert(pack.p1x == (float)1.750703E38F);
                Debug.Assert(pack.p2z == (float)3.1050907E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.p1y == (float) -1.9098775E38F);
                Debug.Assert(pack.p1z == (float) -3.3159614E38F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p2x = (float) -2.4729817E38F;
            p55.p2y = (float) -2.2881527E38F;
            p55.p1z = (float) -3.3159614E38F;
            p55.p1x = (float)1.750703E38F;
            p55.frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p55.p1y = (float) -1.9098775E38F;
            p55.p2z = (float)3.1050907E38F;
            ADV_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float)2.3066101E38F);
                Debug.Assert(pack.pitchspeed == (float) -1.016485E38F);
                Debug.Assert(pack.time_usec == (ulong)5208483366679418303L);
                Debug.Assert(pack.yawspeed == (float)5.185059E36F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {2.78437E38F, 2.2959742E38F, 1.9506567E38F, -2.4989855E38F, -6.9001183E37F, 2.387317E37F, 2.572183E38F, -1.517309E37F, 2.2287922E38F}));
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.3567091E38F, 1.0535636E38F, 1.3352502E37F, -1.1484714E38F}));
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.yawspeed = (float)5.185059E36F;
            p61.rollspeed = (float)2.3066101E38F;
            p61.time_usec = (ulong)5208483366679418303L;
            p61.covariance_SET(new float[] {2.78437E38F, 2.2959742E38F, 1.9506567E38F, -2.4989855E38F, -6.9001183E37F, 2.387317E37F, 2.572183E38F, -1.517309E37F, 2.2287922E38F}, 0) ;
            p61.q_SET(new float[] {2.3567091E38F, 1.0535636E38F, 1.3352502E37F, -1.1484714E38F}, 0) ;
            p61.pitchspeed = (float) -1.016485E38F;
            ADV_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nav_pitch == (float) -5.346703E37F);
                Debug.Assert(pack.aspd_error == (float) -1.6154128E38F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)53459);
                Debug.Assert(pack.xtrack_error == (float)2.8045215E37F);
                Debug.Assert(pack.nav_roll == (float)3.0122098E38F);
                Debug.Assert(pack.target_bearing == (short)(short)25876);
                Debug.Assert(pack.alt_error == (float)2.0729395E38F);
                Debug.Assert(pack.nav_bearing == (short)(short)1789);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_bearing = (short)(short)1789;
            p62.xtrack_error = (float)2.8045215E37F;
            p62.wp_dist = (ushort)(ushort)53459;
            p62.nav_roll = (float)3.0122098E38F;
            p62.aspd_error = (float) -1.6154128E38F;
            p62.alt_error = (float)2.0729395E38F;
            p62.target_bearing = (short)(short)25876;
            p62.nav_pitch = (float) -5.346703E37F;
            ADV_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float) -5.1877485E37F);
                Debug.Assert(pack.lon == (int)1287012808);
                Debug.Assert(pack.time_usec == (ulong)5502747694130743659L);
                Debug.Assert(pack.lat == (int) -877402552);
                Debug.Assert(pack.vx == (float) -2.4031197E38F);
                Debug.Assert(pack.alt == (int)1338398685);
                Debug.Assert(pack.vz == (float) -2.7300283E37F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {8.665739E37F, -3.2775843E38F, -6.009413E37F, 2.0487215E38F, -7.09326E37F, -1.4079602E38F, -1.884714E38F, -2.7646642E38F, -5.5737857E37F, -1.7157499E38F, 1.3030798E38F, -7.3820264E37F, -2.6630449E38F, 2.611418E38F, 1.8806282E38F, 1.4542496E38F, 2.6564799E38F, 6.8606163E37F, 1.8599789E37F, 1.8479786E38F, 1.6466609E38F, 2.7550283E38F, -2.0105336E38F, 1.085371E38F, -3.008768E38F, -2.617487E37F, 1.0176481E38F, 5.564757E37F, 1.418937E38F, -1.4668621E37F, -2.0431932E38F, 3.1095354E38F, 3.151824E38F, -1.995747E38F, -6.3347097E37F, 3.011484E37F}));
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
                Debug.Assert(pack.relative_alt == (int)740813295);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.covariance_SET(new float[] {8.665739E37F, -3.2775843E38F, -6.009413E37F, 2.0487215E38F, -7.09326E37F, -1.4079602E38F, -1.884714E38F, -2.7646642E38F, -5.5737857E37F, -1.7157499E38F, 1.3030798E38F, -7.3820264E37F, -2.6630449E38F, 2.611418E38F, 1.8806282E38F, 1.4542496E38F, 2.6564799E38F, 6.8606163E37F, 1.8599789E37F, 1.8479786E38F, 1.6466609E38F, 2.7550283E38F, -2.0105336E38F, 1.085371E38F, -3.008768E38F, -2.617487E37F, 1.0176481E38F, 5.564757E37F, 1.418937E38F, -1.4668621E37F, -2.0431932E38F, 3.1095354E38F, 3.151824E38F, -1.995747E38F, -6.3347097E37F, 3.011484E37F}, 0) ;
            p63.vx = (float) -2.4031197E38F;
            p63.relative_alt = (int)740813295;
            p63.vy = (float) -5.1877485E37F;
            p63.vz = (float) -2.7300283E37F;
            p63.lon = (int)1287012808;
            p63.alt = (int)1338398685;
            p63.lat = (int) -877402552;
            p63.time_usec = (ulong)5502747694130743659L;
            p63.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS;
            ADV_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -3.1909569E38F);
                Debug.Assert(pack.vy == (float)6.7298916E37F);
                Debug.Assert(pack.time_usec == (ulong)1239487312247741430L);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.827366E38F, 2.5028674E38F, -1.2578101E37F, -2.1086915E38F, -1.474216E38F, 1.838626E38F, 3.0958652E38F, 2.6444406E37F, -8.014626E37F, 4.9697202E36F, -2.8922718E38F, -2.7834106E38F, -2.0558999E38F, -2.4187637E38F, -1.1887875E38F, -8.068669E37F, 2.0276868E38F, -2.2335912E38F, 2.359948E38F, 2.2460162E38F, -2.9545988E38F, -1.6123585E38F, 1.0738692E38F, -3.0458749E37F, -2.9095644E38F, -1.123157E38F, 1.2691823E37F, 2.0530652E38F, 3.1558638E38F, 3.0680264E38F, 2.5927893E38F, -3.372824E37F, 3.2701268E38F, -3.9840545E37F, 1.1651722E38F, 9.762601E37F, -1.2919418E38F, -3.1576872E38F, -6.9152825E37F, 8.603691E37F, 3.210446E38F, -4.517515E37F, 1.9575132E38F, -1.3413708E37F, 2.2177413E38F}));
                Debug.Assert(pack.az == (float)2.4980096E38F);
                Debug.Assert(pack.vz == (float) -1.0143175E38F);
                Debug.Assert(pack.ay == (float) -2.8199715E38F);
                Debug.Assert(pack.z == (float)1.2942337E38F);
                Debug.Assert(pack.ax == (float) -2.8657522E38F);
                Debug.Assert(pack.y == (float)3.730253E36F);
                Debug.Assert(pack.vx == (float) -2.2570677E38F);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.vx = (float) -2.2570677E38F;
            p64.vz = (float) -1.0143175E38F;
            p64.y = (float)3.730253E36F;
            p64.vy = (float)6.7298916E37F;
            p64.covariance_SET(new float[] {-2.827366E38F, 2.5028674E38F, -1.2578101E37F, -2.1086915E38F, -1.474216E38F, 1.838626E38F, 3.0958652E38F, 2.6444406E37F, -8.014626E37F, 4.9697202E36F, -2.8922718E38F, -2.7834106E38F, -2.0558999E38F, -2.4187637E38F, -1.1887875E38F, -8.068669E37F, 2.0276868E38F, -2.2335912E38F, 2.359948E38F, 2.2460162E38F, -2.9545988E38F, -1.6123585E38F, 1.0738692E38F, -3.0458749E37F, -2.9095644E38F, -1.123157E38F, 1.2691823E37F, 2.0530652E38F, 3.1558638E38F, 3.0680264E38F, 2.5927893E38F, -3.372824E37F, 3.2701268E38F, -3.9840545E37F, 1.1651722E38F, 9.762601E37F, -1.2919418E38F, -3.1576872E38F, -6.9152825E37F, 8.603691E37F, 3.210446E38F, -4.517515E37F, 1.9575132E38F, -1.3413708E37F, 2.2177413E38F}, 0) ;
            p64.ax = (float) -2.8657522E38F;
            p64.x = (float) -3.1909569E38F;
            p64.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO;
            p64.time_usec = (ulong)1239487312247741430L;
            p64.z = (float)1.2942337E38F;
            p64.ay = (float) -2.8199715E38F;
            p64.az = (float)2.4980096E38F;
            ADV_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)13184);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)31237);
                Debug.Assert(pack.time_boot_ms == (uint)904709815U);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)47321);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)39397);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)16042);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)15457);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)7810);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)49229);
                Debug.Assert(pack.rssi == (byte)(byte)219);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)50128);
                Debug.Assert(pack.chancount == (byte)(byte)195);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)61426);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)54711);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)51943);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)27497);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)59642);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)31156);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)19792);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)7293);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)54004);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan3_raw = (ushort)(ushort)31156;
            p65.chan14_raw = (ushort)(ushort)47321;
            p65.chan4_raw = (ushort)(ushort)31237;
            p65.chan9_raw = (ushort)(ushort)39397;
            p65.chan18_raw = (ushort)(ushort)13184;
            p65.chan7_raw = (ushort)(ushort)15457;
            p65.chan15_raw = (ushort)(ushort)59642;
            p65.chan6_raw = (ushort)(ushort)54711;
            p65.chan1_raw = (ushort)(ushort)16042;
            p65.rssi = (byte)(byte)219;
            p65.chan5_raw = (ushort)(ushort)7810;
            p65.chancount = (byte)(byte)195;
            p65.chan8_raw = (ushort)(ushort)49229;
            p65.chan12_raw = (ushort)(ushort)51943;
            p65.chan10_raw = (ushort)(ushort)50128;
            p65.chan13_raw = (ushort)(ushort)61426;
            p65.time_boot_ms = (uint)904709815U;
            p65.chan11_raw = (ushort)(ushort)19792;
            p65.chan17_raw = (ushort)(ushort)7293;
            p65.chan2_raw = (ushort)(ushort)54004;
            p65.chan16_raw = (ushort)(ushort)27497;
            ADV_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.req_stream_id == (byte)(byte)199);
                Debug.Assert(pack.start_stop == (byte)(byte)149);
                Debug.Assert(pack.target_system == (byte)(byte)81);
                Debug.Assert(pack.target_component == (byte)(byte)145);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)64444);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_system = (byte)(byte)81;
            p66.req_message_rate = (ushort)(ushort)64444;
            p66.req_stream_id = (byte)(byte)199;
            p66.target_component = (byte)(byte)145;
            p66.start_stop = (byte)(byte)149;
            ADV_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.stream_id == (byte)(byte)215);
                Debug.Assert(pack.on_off == (byte)(byte)36);
                Debug.Assert(pack.message_rate == (ushort)(ushort)24657);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.on_off = (byte)(byte)36;
            p67.stream_id = (byte)(byte)215;
            p67.message_rate = (ushort)(ushort)24657;
            ADV_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target == (byte)(byte)224);
                Debug.Assert(pack.z == (short)(short) -7993);
                Debug.Assert(pack.x == (short)(short) -18516);
                Debug.Assert(pack.y == (short)(short) -30350);
                Debug.Assert(pack.r == (short)(short) -26536);
                Debug.Assert(pack.buttons == (ushort)(ushort)10440);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.z = (short)(short) -7993;
            p69.y = (short)(short) -30350;
            p69.buttons = (ushort)(ushort)10440;
            p69.x = (short)(short) -18516;
            p69.r = (short)(short) -26536;
            p69.target = (byte)(byte)224;
            ADV_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)45968);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)48973);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)35614);
                Debug.Assert(pack.target_system == (byte)(byte)79);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)39132);
                Debug.Assert(pack.target_component == (byte)(byte)162);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)34026);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)39347);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)35681);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)39427);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.target_component = (byte)(byte)162;
            p70.chan6_raw = (ushort)(ushort)48973;
            p70.target_system = (byte)(byte)79;
            p70.chan1_raw = (ushort)(ushort)35614;
            p70.chan2_raw = (ushort)(ushort)39132;
            p70.chan3_raw = (ushort)(ushort)39427;
            p70.chan7_raw = (ushort)(ushort)45968;
            p70.chan5_raw = (ushort)(ushort)35681;
            p70.chan4_raw = (ushort)(ushort)34026;
            p70.chan8_raw = (ushort)(ushort)39347;
            ADV_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.current == (byte)(byte)218);
                Debug.Assert(pack.param2 == (float)3.3957488E38F);
                Debug.Assert(pack.z == (float)7.137677E37F);
                Debug.Assert(pack.param4 == (float) -1.3362906E38F);
                Debug.Assert(pack.param3 == (float)1.4268909E38F);
                Debug.Assert(pack.target_component == (byte)(byte)122);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.autocontinue == (byte)(byte)13);
                Debug.Assert(pack.y == (int) -825983266);
                Debug.Assert(pack.x == (int)1594682719);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_CONDITION_DISTANCE);
                Debug.Assert(pack.param1 == (float) -1.276072E38F);
                Debug.Assert(pack.seq == (ushort)(ushort)3765);
                Debug.Assert(pack.target_system == (byte)(byte)207);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.x = (int)1594682719;
            p73.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p73.target_system = (byte)(byte)207;
            p73.autocontinue = (byte)(byte)13;
            p73.param2 = (float)3.3957488E38F;
            p73.current = (byte)(byte)218;
            p73.z = (float)7.137677E37F;
            p73.frame = MAV_FRAME.MAV_FRAME_MISSION;
            p73.param1 = (float) -1.276072E38F;
            p73.param3 = (float)1.4268909E38F;
            p73.command = MAV_CMD.MAV_CMD_CONDITION_DISTANCE;
            p73.param4 = (float) -1.3362906E38F;
            p73.target_component = (byte)(byte)122;
            p73.seq = (ushort)(ushort)3765;
            p73.y = (int) -825983266;
            ADV_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.throttle == (ushort)(ushort)50569);
                Debug.Assert(pack.heading == (short)(short) -28392);
                Debug.Assert(pack.groundspeed == (float)9.521557E37F);
                Debug.Assert(pack.alt == (float)1.1661201E38F);
                Debug.Assert(pack.climb == (float)6.4456823E37F);
                Debug.Assert(pack.airspeed == (float)2.2654525E38F);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.groundspeed = (float)9.521557E37F;
            p74.climb = (float)6.4456823E37F;
            p74.airspeed = (float)2.2654525E38F;
            p74.alt = (float)1.1661201E38F;
            p74.heading = (short)(short) -28392;
            p74.throttle = (ushort)(ushort)50569;
            ADV_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.autocontinue == (byte)(byte)60);
                Debug.Assert(pack.param3 == (float)1.0556689E38F);
                Debug.Assert(pack.y == (int) -639222413);
                Debug.Assert(pack.param4 == (float)3.3190088E38F);
                Debug.Assert(pack.x == (int) -264887733);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_WAYPOINT_USER_4);
                Debug.Assert(pack.param2 == (float)4.1364235E37F);
                Debug.Assert(pack.target_component == (byte)(byte)219);
                Debug.Assert(pack.target_system == (byte)(byte)180);
                Debug.Assert(pack.param1 == (float) -9.189578E37F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.z == (float)2.36534E36F);
                Debug.Assert(pack.current == (byte)(byte)89);
            };
            GroundControl.COMMAND_INT p75 = CommunicationChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.command = MAV_CMD.MAV_CMD_WAYPOINT_USER_4;
            p75.y = (int) -639222413;
            p75.param4 = (float)3.3190088E38F;
            p75.param3 = (float)1.0556689E38F;
            p75.z = (float)2.36534E36F;
            p75.param1 = (float) -9.189578E37F;
            p75.x = (int) -264887733;
            p75.target_component = (byte)(byte)219;
            p75.target_system = (byte)(byte)180;
            p75.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p75.param2 = (float)4.1364235E37F;
            p75.current = (byte)(byte)89;
            p75.autocontinue = (byte)(byte)60;
            CommunicationChannel.instance.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param4 == (float) -1.1066383E38F);
                Debug.Assert(pack.param6 == (float) -3.2085512E37F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE);
                Debug.Assert(pack.param1 == (float) -2.1778168E38F);
                Debug.Assert(pack.target_system == (byte)(byte)139);
                Debug.Assert(pack.confirmation == (byte)(byte)126);
                Debug.Assert(pack.param7 == (float)1.7981894E38F);
                Debug.Assert(pack.param5 == (float) -1.5836505E38F);
                Debug.Assert(pack.target_component == (byte)(byte)17);
                Debug.Assert(pack.param3 == (float)5.6496297E37F);
                Debug.Assert(pack.param2 == (float)2.094921E38F);
            };
            GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.command = MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE;
            p76.param1 = (float) -2.1778168E38F;
            p76.param4 = (float) -1.1066383E38F;
            p76.target_component = (byte)(byte)17;
            p76.param2 = (float)2.094921E38F;
            p76.param5 = (float) -1.5836505E38F;
            p76.param3 = (float)5.6496297E37F;
            p76.param7 = (float)1.7981894E38F;
            p76.target_system = (byte)(byte)139;
            p76.confirmation = (byte)(byte)126;
            p76.param6 = (float) -3.2085512E37F;
            CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result == MAV_RESULT.MAV_RESULT_DENIED);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)56);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)210);
                Debug.Assert(pack.result_param2_TRY(ph) == (int) -1167050496);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)117);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_CONDITION_YAW);
            };
            GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.progress_SET((byte)(byte)210, PH) ;
            p77.result = MAV_RESULT.MAV_RESULT_DENIED;
            p77.target_component_SET((byte)(byte)117, PH) ;
            p77.target_system_SET((byte)(byte)56, PH) ;
            p77.result_param2_SET((int) -1167050496, PH) ;
            p77.command = MAV_CMD.MAV_CMD_CONDITION_YAW;
            CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -6.083423E37F);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)144);
                Debug.Assert(pack.mode_switch == (byte)(byte)25);
                Debug.Assert(pack.yaw == (float)1.1058223E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1823851859U);
                Debug.Assert(pack.roll == (float)1.4230613E38F);
                Debug.Assert(pack.thrust == (float) -1.3107058E38F);
            };
            GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.pitch = (float) -6.083423E37F;
            p81.thrust = (float) -1.3107058E38F;
            p81.mode_switch = (byte)(byte)25;
            p81.time_boot_ms = (uint)1823851859U;
            p81.yaw = (float)1.1058223E38F;
            p81.roll = (float)1.4230613E38F;
            p81.manual_override_switch = (byte)(byte)144;
            CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)48);
                Debug.Assert(pack.body_pitch_rate == (float) -3.1873764E38F);
                Debug.Assert(pack.thrust == (float) -3.1760424E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)25);
                Debug.Assert(pack.body_roll_rate == (float) -1.4877746E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2883149177U);
                Debug.Assert(pack.target_component == (byte)(byte)202);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-7.8617814E37F, 2.3991162E38F, 2.3209461E38F, 1.1666193E38F}));
                Debug.Assert(pack.body_yaw_rate == (float) -1.75606E38F);
            };
            GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.body_roll_rate = (float) -1.4877746E38F;
            p82.thrust = (float) -3.1760424E38F;
            p82.body_yaw_rate = (float) -1.75606E38F;
            p82.body_pitch_rate = (float) -3.1873764E38F;
            p82.type_mask = (byte)(byte)25;
            p82.target_system = (byte)(byte)48;
            p82.q_SET(new float[] {-7.8617814E37F, 2.3991162E38F, 2.3209461E38F, 1.1666193E38F}, 0) ;
            p82.time_boot_ms = (uint)2883149177U;
            p82.target_component = (byte)(byte)202;
            CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (byte)(byte)229);
                Debug.Assert(pack.time_boot_ms == (uint)3611792583U);
                Debug.Assert(pack.q.SequenceEqual(new float[] {4.057594E37F, 2.1906485E38F, 6.4325605E36F, 2.6097011E38F}));
                Debug.Assert(pack.body_yaw_rate == (float)3.2136918E37F);
                Debug.Assert(pack.body_roll_rate == (float) -2.5838084E38F);
                Debug.Assert(pack.thrust == (float) -9.425874E37F);
                Debug.Assert(pack.body_pitch_rate == (float) -4.557823E37F);
            };
            GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.body_yaw_rate = (float)3.2136918E37F;
            p83.body_pitch_rate = (float) -4.557823E37F;
            p83.time_boot_ms = (uint)3611792583U;
            p83.q_SET(new float[] {4.057594E37F, 2.1906485E38F, 6.4325605E36F, 2.6097011E38F}, 0) ;
            p83.body_roll_rate = (float) -2.5838084E38F;
            p83.thrust = (float) -9.425874E37F;
            p83.type_mask = (byte)(byte)229;
            CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afz == (float) -3.2622763E38F);
                Debug.Assert(pack.z == (float) -2.2442324E38F);
                Debug.Assert(pack.time_boot_ms == (uint)93793495U);
                Debug.Assert(pack.vz == (float)1.8632375E38F);
                Debug.Assert(pack.afy == (float) -9.032192E37F);
                Debug.Assert(pack.yaw_rate == (float) -1.1435286E37F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.target_component == (byte)(byte)84);
                Debug.Assert(pack.x == (float) -2.084642E38F);
                Debug.Assert(pack.y == (float)3.1498516E37F);
                Debug.Assert(pack.vy == (float)1.5008239E38F);
                Debug.Assert(pack.afx == (float)1.6114293E38F);
                Debug.Assert(pack.yaw == (float)2.2711338E38F);
                Debug.Assert(pack.vx == (float)1.4442008E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)12527);
                Debug.Assert(pack.target_system == (byte)(byte)223);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.vx = (float)1.4442008E38F;
            p84.afx = (float)1.6114293E38F;
            p84.y = (float)3.1498516E37F;
            p84.vz = (float)1.8632375E38F;
            p84.z = (float) -2.2442324E38F;
            p84.type_mask = (ushort)(ushort)12527;
            p84.target_system = (byte)(byte)223;
            p84.vy = (float)1.5008239E38F;
            p84.time_boot_ms = (uint)93793495U;
            p84.target_component = (byte)(byte)84;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p84.yaw = (float)2.2711338E38F;
            p84.x = (float) -2.084642E38F;
            p84.afz = (float) -3.2622763E38F;
            p84.afy = (float) -9.032192E37F;
            p84.yaw_rate = (float) -1.1435286E37F;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float) -3.4285144E37F);
                Debug.Assert(pack.lon_int == (int)1008327934);
                Debug.Assert(pack.vx == (float) -2.1822767E38F);
                Debug.Assert(pack.yaw_rate == (float) -4.57144E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)19105);
                Debug.Assert(pack.afz == (float)7.3483657E37F);
                Debug.Assert(pack.lat_int == (int) -952852506);
                Debug.Assert(pack.afx == (float)2.8323272E38F);
                Debug.Assert(pack.afy == (float)2.051118E38F);
                Debug.Assert(pack.yaw == (float) -2.9204323E38F);
                Debug.Assert(pack.target_system == (byte)(byte)18);
                Debug.Assert(pack.alt == (float)3.3420864E38F);
                Debug.Assert(pack.vz == (float)1.9977456E37F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.time_boot_ms == (uint)4008574548U);
                Debug.Assert(pack.target_component == (byte)(byte)70);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.type_mask = (ushort)(ushort)19105;
            p86.vy = (float) -3.4285144E37F;
            p86.target_component = (byte)(byte)70;
            p86.target_system = (byte)(byte)18;
            p86.afz = (float)7.3483657E37F;
            p86.afy = (float)2.051118E38F;
            p86.yaw = (float) -2.9204323E38F;
            p86.yaw_rate = (float) -4.57144E37F;
            p86.lon_int = (int)1008327934;
            p86.lat_int = (int) -952852506;
            p86.afx = (float)2.8323272E38F;
            p86.alt = (float)3.3420864E38F;
            p86.vx = (float) -2.1822767E38F;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p86.time_boot_ms = (uint)4008574548U;
            p86.vz = (float)1.9977456E37F;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float) -3.085891E38F);
                Debug.Assert(pack.afy == (float) -1.6182722E37F);
                Debug.Assert(pack.lat_int == (int) -1333381302);
                Debug.Assert(pack.lon_int == (int)1796922362);
                Debug.Assert(pack.afx == (float) -3.1234957E38F);
                Debug.Assert(pack.time_boot_ms == (uint)621988819U);
                Debug.Assert(pack.yaw == (float) -4.99936E37F);
                Debug.Assert(pack.vx == (float)3.022919E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.alt == (float)1.1486749E38F);
                Debug.Assert(pack.yaw_rate == (float) -1.5433973E37F);
                Debug.Assert(pack.vz == (float) -2.0174373E38F);
                Debug.Assert(pack.afz == (float)1.8301423E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)14046);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.vy = (float) -3.085891E38F;
            p87.alt = (float)1.1486749E38F;
            p87.lat_int = (int) -1333381302;
            p87.lon_int = (int)1796922362;
            p87.yaw = (float) -4.99936E37F;
            p87.afx = (float) -3.1234957E38F;
            p87.afz = (float)1.8301423E38F;
            p87.vx = (float)3.022919E38F;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p87.time_boot_ms = (uint)621988819U;
            p87.afy = (float) -1.6182722E37F;
            p87.yaw_rate = (float) -1.5433973E37F;
            p87.vz = (float) -2.0174373E38F;
            p87.type_mask = (ushort)(ushort)14046;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2254768691U);
                Debug.Assert(pack.yaw == (float) -7.392114E37F);
                Debug.Assert(pack.roll == (float) -2.0659553E38F);
                Debug.Assert(pack.x == (float) -1.5319793E38F);
                Debug.Assert(pack.pitch == (float) -3.0982328E38F);
                Debug.Assert(pack.z == (float)2.8150976E38F);
                Debug.Assert(pack.y == (float)4.902566E37F);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.y = (float)4.902566E37F;
            p89.roll = (float) -2.0659553E38F;
            p89.yaw = (float) -7.392114E37F;
            p89.pitch = (float) -3.0982328E38F;
            p89.z = (float)2.8150976E38F;
            p89.time_boot_ms = (uint)2254768691U;
            p89.x = (float) -1.5319793E38F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -1.145991E38F);
                Debug.Assert(pack.alt == (int) -2145969180);
                Debug.Assert(pack.roll == (float)5.4536347E37F);
                Debug.Assert(pack.zacc == (short)(short)30107);
                Debug.Assert(pack.pitchspeed == (float) -2.6972443E38F);
                Debug.Assert(pack.rollspeed == (float) -3.2920182E38F);
                Debug.Assert(pack.lon == (int)793482080);
                Debug.Assert(pack.yaw == (float) -1.2311294E38F);
                Debug.Assert(pack.lat == (int) -2089260041);
                Debug.Assert(pack.vx == (short)(short)30081);
                Debug.Assert(pack.vy == (short)(short) -26923);
                Debug.Assert(pack.yacc == (short)(short)7598);
                Debug.Assert(pack.time_usec == (ulong)1998175844980635751L);
                Debug.Assert(pack.yawspeed == (float)1.0572889E38F);
                Debug.Assert(pack.vz == (short)(short) -5228);
                Debug.Assert(pack.xacc == (short)(short) -1279);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.pitch = (float) -1.145991E38F;
            p90.roll = (float)5.4536347E37F;
            p90.vx = (short)(short)30081;
            p90.vz = (short)(short) -5228;
            p90.rollspeed = (float) -3.2920182E38F;
            p90.pitchspeed = (float) -2.6972443E38F;
            p90.yaw = (float) -1.2311294E38F;
            p90.lat = (int) -2089260041;
            p90.yawspeed = (float)1.0572889E38F;
            p90.yacc = (short)(short)7598;
            p90.vy = (short)(short) -26923;
            p90.alt = (int) -2145969180;
            p90.time_usec = (ulong)1998175844980635751L;
            p90.zacc = (short)(short)30107;
            p90.xacc = (short)(short) -1279;
            p90.lon = (int)793482080;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aux3 == (float)1.4338565E38F);
                Debug.Assert(pack.nav_mode == (byte)(byte)58);
                Debug.Assert(pack.aux1 == (float)2.7214246E38F);
                Debug.Assert(pack.throttle == (float) -1.3420369E38F);
                Debug.Assert(pack.aux2 == (float)3.432448E37F);
                Debug.Assert(pack.pitch_elevator == (float)3.25688E38F);
                Debug.Assert(pack.roll_ailerons == (float)2.8658246E37F);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_TEST_ARMED);
                Debug.Assert(pack.time_usec == (ulong)2013875573800775113L);
                Debug.Assert(pack.yaw_rudder == (float)1.6738467E38F);
                Debug.Assert(pack.aux4 == (float) -3.0386236E38F);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.aux1 = (float)2.7214246E38F;
            p91.nav_mode = (byte)(byte)58;
            p91.yaw_rudder = (float)1.6738467E38F;
            p91.aux3 = (float)1.4338565E38F;
            p91.throttle = (float) -1.3420369E38F;
            p91.aux2 = (float)3.432448E37F;
            p91.time_usec = (ulong)2013875573800775113L;
            p91.aux4 = (float) -3.0386236E38F;
            p91.pitch_elevator = (float)3.25688E38F;
            p91.roll_ailerons = (float)2.8658246E37F;
            p91.mode = MAV_MODE.MAV_MODE_TEST_ARMED;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)58793);
                Debug.Assert(pack.time_usec == (ulong)4207835245388656594L);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)777);
                Debug.Assert(pack.rssi == (byte)(byte)182);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)65215);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)51140);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)55613);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)18469);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)59116);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)14549);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)39555);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)5427);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)13092);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)38495);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan1_raw = (ushort)(ushort)14549;
            p92.chan7_raw = (ushort)(ushort)51140;
            p92.chan5_raw = (ushort)(ushort)13092;
            p92.chan4_raw = (ushort)(ushort)58793;
            p92.chan2_raw = (ushort)(ushort)59116;
            p92.rssi = (byte)(byte)182;
            p92.chan8_raw = (ushort)(ushort)38495;
            p92.chan12_raw = (ushort)(ushort)777;
            p92.time_usec = (ulong)4207835245388656594L;
            p92.chan3_raw = (ushort)(ushort)65215;
            p92.chan11_raw = (ushort)(ushort)39555;
            p92.chan6_raw = (ushort)(ushort)5427;
            p92.chan9_raw = (ushort)(ushort)18469;
            p92.chan10_raw = (ushort)(ushort)55613;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {9.948451E37F, -2.4991926E38F, 2.0462061E38F, -1.9931996E38F, -2.9480868E38F, -1.7282418E38F, -2.8648042E38F, -3.2132687E38F, -1.913963E38F, -5.1353626E37F, -1.7294458E38F, -3.239816E38F, 1.9923772E38F, -1.9453074E38F, -2.3228685E38F, -1.8217372E38F}));
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_AUTO_ARMED);
                Debug.Assert(pack.time_usec == (ulong)3166692993019025755L);
                Debug.Assert(pack.flags == (ulong)3796470254240389255L);
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.controls_SET(new float[] {9.948451E37F, -2.4991926E38F, 2.0462061E38F, -1.9931996E38F, -2.9480868E38F, -1.7282418E38F, -2.8648042E38F, -3.2132687E38F, -1.913963E38F, -5.1353626E37F, -1.7294458E38F, -3.239816E38F, 1.9923772E38F, -1.9453074E38F, -2.3228685E38F, -1.8217372E38F}, 0) ;
            p93.mode = MAV_MODE.MAV_MODE_AUTO_ARMED;
            p93.flags = (ulong)3796470254240389255L;
            p93.time_usec = (ulong)3166692993019025755L;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)616859837472847320L);
                Debug.Assert(pack.flow_comp_m_y == (float) -3.0233785E38F);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float)2.0649357E38F);
                Debug.Assert(pack.flow_y == (short)(short) -6720);
                Debug.Assert(pack.ground_distance == (float)4.4238435E37F);
                Debug.Assert(pack.flow_comp_m_x == (float) -8.359354E37F);
                Debug.Assert(pack.flow_x == (short)(short)8670);
                Debug.Assert(pack.sensor_id == (byte)(byte)233);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)1.4766529E38F);
                Debug.Assert(pack.quality == (byte)(byte)239);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.quality = (byte)(byte)239;
            p100.sensor_id = (byte)(byte)233;
            p100.flow_comp_m_x = (float) -8.359354E37F;
            p100.flow_rate_y_SET((float)2.0649357E38F, PH) ;
            p100.flow_x = (short)(short)8670;
            p100.flow_comp_m_y = (float) -3.0233785E38F;
            p100.ground_distance = (float)4.4238435E37F;
            p100.flow_y = (short)(short) -6720;
            p100.flow_rate_x_SET((float)1.4766529E38F, PH) ;
            p100.time_usec = (ulong)616859837472847320L;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float)1.0978955E37F);
                Debug.Assert(pack.y == (float) -3.165014E38F);
                Debug.Assert(pack.yaw == (float) -2.972577E38F);
                Debug.Assert(pack.usec == (ulong)5240219118471688826L);
                Debug.Assert(pack.z == (float)3.2329757E38F);
                Debug.Assert(pack.roll == (float) -2.0754138E38F);
                Debug.Assert(pack.x == (float) -1.0610068E38F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.y = (float) -3.165014E38F;
            p101.yaw = (float) -2.972577E38F;
            p101.pitch = (float)1.0978955E37F;
            p101.x = (float) -1.0610068E38F;
            p101.roll = (float) -2.0754138E38F;
            p101.z = (float)3.2329757E38F;
            p101.usec = (ulong)5240219118471688826L;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -1.9776692E38F);
                Debug.Assert(pack.pitch == (float) -1.1399507E38F);
                Debug.Assert(pack.y == (float) -2.585749E37F);
                Debug.Assert(pack.roll == (float)1.3085416E37F);
                Debug.Assert(pack.usec == (ulong)6964771068581876896L);
                Debug.Assert(pack.yaw == (float) -1.3433275E38F);
                Debug.Assert(pack.z == (float) -3.0944211E38F);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.pitch = (float) -1.1399507E38F;
            p102.usec = (ulong)6964771068581876896L;
            p102.x = (float) -1.9776692E38F;
            p102.roll = (float)1.3085416E37F;
            p102.z = (float) -3.0944211E38F;
            p102.y = (float) -2.585749E37F;
            p102.yaw = (float) -1.3433275E38F;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)4.2592954E37F);
                Debug.Assert(pack.usec == (ulong)2764962112215065870L);
                Debug.Assert(pack.z == (float)3.5785632E37F);
                Debug.Assert(pack.x == (float) -2.4585327E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)2764962112215065870L;
            p103.x = (float) -2.4585327E38F;
            p103.z = (float)3.5785632E37F;
            p103.y = (float)4.2592954E37F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -3.1114837E38F);
                Debug.Assert(pack.yaw == (float) -3.921652E36F);
                Debug.Assert(pack.z == (float) -2.416582E38F);
                Debug.Assert(pack.pitch == (float)2.9867653E38F);
                Debug.Assert(pack.roll == (float)4.075723E37F);
                Debug.Assert(pack.usec == (ulong)7623152166682493037L);
                Debug.Assert(pack.y == (float)1.0791186E38F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.z = (float) -2.416582E38F;
            p104.usec = (ulong)7623152166682493037L;
            p104.pitch = (float)2.9867653E38F;
            p104.y = (float)1.0791186E38F;
            p104.roll = (float)4.075723E37F;
            p104.yaw = (float) -3.921652E36F;
            p104.x = (float) -3.1114837E38F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fields_updated == (ushort)(ushort)46036);
                Debug.Assert(pack.abs_pressure == (float) -3.0665538E37F);
                Debug.Assert(pack.zacc == (float) -3.7676353E37F);
                Debug.Assert(pack.ymag == (float)2.8459995E36F);
                Debug.Assert(pack.xacc == (float) -7.36627E37F);
                Debug.Assert(pack.ygyro == (float) -2.7563626E38F);
                Debug.Assert(pack.zmag == (float)2.3777996E38F);
                Debug.Assert(pack.xmag == (float) -9.975175E37F);
                Debug.Assert(pack.xgyro == (float) -3.2186478E38F);
                Debug.Assert(pack.pressure_alt == (float)2.9749904E38F);
                Debug.Assert(pack.diff_pressure == (float)2.1970263E38F);
                Debug.Assert(pack.time_usec == (ulong)6641771924716958684L);
                Debug.Assert(pack.yacc == (float) -3.7430495E37F);
                Debug.Assert(pack.temperature == (float) -1.5300083E38F);
                Debug.Assert(pack.zgyro == (float)2.677454E38F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.ymag = (float)2.8459995E36F;
            p105.xmag = (float) -9.975175E37F;
            p105.pressure_alt = (float)2.9749904E38F;
            p105.ygyro = (float) -2.7563626E38F;
            p105.xacc = (float) -7.36627E37F;
            p105.zacc = (float) -3.7676353E37F;
            p105.abs_pressure = (float) -3.0665538E37F;
            p105.zmag = (float)2.3777996E38F;
            p105.time_usec = (ulong)6641771924716958684L;
            p105.diff_pressure = (float)2.1970263E38F;
            p105.temperature = (float) -1.5300083E38F;
            p105.zgyro = (float)2.677454E38F;
            p105.xgyro = (float) -3.2186478E38F;
            p105.fields_updated = (ushort)(ushort)46036;
            p105.yacc = (float) -3.7430495E37F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integration_time_us == (uint)161233863U);
                Debug.Assert(pack.time_usec == (ulong)2169121802915836516L);
                Debug.Assert(pack.integrated_zgyro == (float) -2.1602277E38F);
                Debug.Assert(pack.quality == (byte)(byte)177);
                Debug.Assert(pack.temperature == (short)(short) -25646);
                Debug.Assert(pack.distance == (float)8.130238E37F);
                Debug.Assert(pack.integrated_y == (float)2.3206573E38F);
                Debug.Assert(pack.integrated_x == (float)2.930305E38F);
                Debug.Assert(pack.integrated_ygyro == (float)7.410387E37F);
                Debug.Assert(pack.sensor_id == (byte)(byte)199);
                Debug.Assert(pack.time_delta_distance_us == (uint)1012321356U);
                Debug.Assert(pack.integrated_xgyro == (float)1.2026934E38F);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.sensor_id = (byte)(byte)199;
            p106.integrated_y = (float)2.3206573E38F;
            p106.time_delta_distance_us = (uint)1012321356U;
            p106.integration_time_us = (uint)161233863U;
            p106.temperature = (short)(short) -25646;
            p106.integrated_ygyro = (float)7.410387E37F;
            p106.integrated_zgyro = (float) -2.1602277E38F;
            p106.integrated_xgyro = (float)1.2026934E38F;
            p106.integrated_x = (float)2.930305E38F;
            p106.quality = (byte)(byte)177;
            p106.time_usec = (ulong)2169121802915836516L;
            p106.distance = (float)8.130238E37F;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (float) -1.4847872E37F);
                Debug.Assert(pack.zmag == (float) -1.6150401E38F);
                Debug.Assert(pack.abs_pressure == (float)1.0998681E38F);
                Debug.Assert(pack.zacc == (float)2.8679765E38F);
                Debug.Assert(pack.xgyro == (float)2.1273167E38F);
                Debug.Assert(pack.temperature == (float) -1.0117223E38F);
                Debug.Assert(pack.ymag == (float) -2.7043053E36F);
                Debug.Assert(pack.yacc == (float) -1.9596284E38F);
                Debug.Assert(pack.xacc == (float) -2.1226179E38F);
                Debug.Assert(pack.fields_updated == (uint)1931591222U);
                Debug.Assert(pack.time_usec == (ulong)4325009505322095917L);
                Debug.Assert(pack.diff_pressure == (float)7.542017E37F);
                Debug.Assert(pack.pressure_alt == (float)2.7778587E38F);
                Debug.Assert(pack.ygyro == (float) -9.608224E37F);
                Debug.Assert(pack.xmag == (float) -2.9866351E38F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.fields_updated = (uint)1931591222U;
            p107.pressure_alt = (float)2.7778587E38F;
            p107.diff_pressure = (float)7.542017E37F;
            p107.xacc = (float) -2.1226179E38F;
            p107.xmag = (float) -2.9866351E38F;
            p107.temperature = (float) -1.0117223E38F;
            p107.zgyro = (float) -1.4847872E37F;
            p107.zacc = (float)2.8679765E38F;
            p107.ymag = (float) -2.7043053E36F;
            p107.yacc = (float) -1.9596284E38F;
            p107.ygyro = (float) -9.608224E37F;
            p107.zmag = (float) -1.6150401E38F;
            p107.abs_pressure = (float)1.0998681E38F;
            p107.time_usec = (ulong)4325009505322095917L;
            p107.xgyro = (float)2.1273167E38F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (float) -1.842366E38F);
                Debug.Assert(pack.xgyro == (float)2.8267815E38F);
                Debug.Assert(pack.yaw == (float)1.5876511E38F);
                Debug.Assert(pack.q3 == (float)2.1197568E38F);
                Debug.Assert(pack.std_dev_vert == (float)2.5255766E38F);
                Debug.Assert(pack.yacc == (float)8.094438E37F);
                Debug.Assert(pack.vd == (float) -2.7357567E38F);
                Debug.Assert(pack.lat == (float)2.0581591E38F);
                Debug.Assert(pack.q4 == (float) -2.8192178E38F);
                Debug.Assert(pack.std_dev_horz == (float)2.324546E38F);
                Debug.Assert(pack.pitch == (float)2.1036522E38F);
                Debug.Assert(pack.ve == (float)2.1627237E38F);
                Debug.Assert(pack.ygyro == (float)7.6253524E37F);
                Debug.Assert(pack.xacc == (float) -2.8209179E38F);
                Debug.Assert(pack.vn == (float)7.0224324E37F);
                Debug.Assert(pack.roll == (float) -5.4782717E37F);
                Debug.Assert(pack.zgyro == (float)1.163867E38F);
                Debug.Assert(pack.alt == (float) -1.8622148E38F);
                Debug.Assert(pack.q2 == (float) -1.9395249E38F);
                Debug.Assert(pack.q1 == (float) -7.497279E37F);
                Debug.Assert(pack.lon == (float)1.770586E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.alt = (float) -1.8622148E38F;
            p108.std_dev_vert = (float)2.5255766E38F;
            p108.pitch = (float)2.1036522E38F;
            p108.lon = (float)1.770586E38F;
            p108.ygyro = (float)7.6253524E37F;
            p108.q2 = (float) -1.9395249E38F;
            p108.xgyro = (float)2.8267815E38F;
            p108.zacc = (float) -1.842366E38F;
            p108.xacc = (float) -2.8209179E38F;
            p108.q1 = (float) -7.497279E37F;
            p108.roll = (float) -5.4782717E37F;
            p108.yaw = (float)1.5876511E38F;
            p108.lat = (float)2.0581591E38F;
            p108.zgyro = (float)1.163867E38F;
            p108.vd = (float) -2.7357567E38F;
            p108.ve = (float)2.1627237E38F;
            p108.vn = (float)7.0224324E37F;
            p108.std_dev_horz = (float)2.324546E38F;
            p108.yacc = (float)8.094438E37F;
            p108.q4 = (float) -2.8192178E38F;
            p108.q3 = (float)2.1197568E38F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fixed_ == (ushort)(ushort)17946);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)50581);
                Debug.Assert(pack.rssi == (byte)(byte)184);
                Debug.Assert(pack.noise == (byte)(byte)88);
                Debug.Assert(pack.remrssi == (byte)(byte)236);
                Debug.Assert(pack.remnoise == (byte)(byte)28);
                Debug.Assert(pack.txbuf == (byte)(byte)173);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.remrssi = (byte)(byte)236;
            p109.fixed_ = (ushort)(ushort)17946;
            p109.rxerrors = (ushort)(ushort)50581;
            p109.txbuf = (byte)(byte)173;
            p109.rssi = (byte)(byte)184;
            p109.noise = (byte)(byte)88;
            p109.remnoise = (byte)(byte)28;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)250, (byte)107, (byte)175, (byte)123, (byte)96, (byte)172, (byte)235, (byte)159, (byte)41, (byte)42, (byte)21, (byte)81, (byte)119, (byte)164, (byte)150, (byte)118, (byte)57, (byte)122, (byte)196, (byte)75, (byte)235, (byte)3, (byte)128, (byte)143, (byte)168, (byte)30, (byte)114, (byte)180, (byte)183, (byte)78, (byte)166, (byte)196, (byte)162, (byte)0, (byte)27, (byte)95, (byte)9, (byte)79, (byte)9, (byte)176, (byte)222, (byte)150, (byte)28, (byte)109, (byte)128, (byte)190, (byte)236, (byte)90, (byte)198, (byte)203, (byte)104, (byte)250, (byte)192, (byte)7, (byte)203, (byte)89, (byte)175, (byte)194, (byte)5, (byte)249, (byte)49, (byte)9, (byte)228, (byte)196, (byte)159, (byte)67, (byte)89, (byte)57, (byte)239, (byte)223, (byte)225, (byte)93, (byte)177, (byte)114, (byte)159, (byte)223, (byte)241, (byte)177, (byte)40, (byte)241, (byte)148, (byte)52, (byte)239, (byte)64, (byte)13, (byte)181, (byte)37, (byte)91, (byte)180, (byte)50, (byte)199, (byte)93, (byte)68, (byte)25, (byte)43, (byte)137, (byte)75, (byte)16, (byte)124, (byte)114, (byte)47, (byte)191, (byte)129, (byte)130, (byte)220, (byte)19, (byte)134, (byte)82, (byte)6, (byte)171, (byte)150, (byte)140, (byte)223, (byte)240, (byte)253, (byte)76, (byte)166, (byte)85, (byte)234, (byte)1, (byte)151, (byte)100, (byte)147, (byte)60, (byte)117, (byte)77, (byte)7, (byte)86, (byte)67, (byte)196, (byte)212, (byte)117, (byte)200, (byte)129, (byte)68, (byte)228, (byte)121, (byte)134, (byte)17, (byte)210, (byte)91, (byte)49, (byte)40, (byte)229, (byte)59, (byte)196, (byte)220, (byte)215, (byte)177, (byte)217, (byte)33, (byte)210, (byte)94, (byte)227, (byte)118, (byte)58, (byte)7, (byte)192, (byte)226, (byte)223, (byte)217, (byte)106, (byte)104, (byte)61, (byte)81, (byte)36, (byte)154, (byte)89, (byte)173, (byte)193, (byte)157, (byte)132, (byte)54, (byte)48, (byte)97, (byte)9, (byte)59, (byte)9, (byte)148, (byte)43, (byte)190, (byte)255, (byte)174, (byte)211, (byte)67, (byte)240, (byte)30, (byte)67, (byte)213, (byte)89, (byte)167, (byte)251, (byte)107, (byte)239, (byte)239, (byte)174, (byte)135, (byte)104, (byte)252, (byte)151, (byte)137, (byte)183, (byte)50, (byte)216, (byte)168, (byte)187, (byte)171, (byte)148, (byte)67, (byte)165, (byte)169, (byte)224, (byte)177, (byte)93, (byte)88, (byte)233, (byte)157, (byte)48, (byte)215, (byte)123, (byte)141, (byte)64, (byte)102, (byte)235, (byte)243, (byte)70, (byte)104, (byte)242, (byte)228, (byte)91, (byte)151, (byte)195, (byte)224, (byte)85, (byte)140, (byte)192, (byte)74, (byte)25, (byte)169, (byte)102, (byte)248, (byte)48, (byte)203, (byte)203, (byte)143, (byte)221, (byte)252, (byte)109, (byte)209, (byte)113, (byte)84}));
                Debug.Assert(pack.target_network == (byte)(byte)174);
                Debug.Assert(pack.target_component == (byte)(byte)20);
                Debug.Assert(pack.target_system == (byte)(byte)224);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.payload_SET(new byte[] {(byte)250, (byte)107, (byte)175, (byte)123, (byte)96, (byte)172, (byte)235, (byte)159, (byte)41, (byte)42, (byte)21, (byte)81, (byte)119, (byte)164, (byte)150, (byte)118, (byte)57, (byte)122, (byte)196, (byte)75, (byte)235, (byte)3, (byte)128, (byte)143, (byte)168, (byte)30, (byte)114, (byte)180, (byte)183, (byte)78, (byte)166, (byte)196, (byte)162, (byte)0, (byte)27, (byte)95, (byte)9, (byte)79, (byte)9, (byte)176, (byte)222, (byte)150, (byte)28, (byte)109, (byte)128, (byte)190, (byte)236, (byte)90, (byte)198, (byte)203, (byte)104, (byte)250, (byte)192, (byte)7, (byte)203, (byte)89, (byte)175, (byte)194, (byte)5, (byte)249, (byte)49, (byte)9, (byte)228, (byte)196, (byte)159, (byte)67, (byte)89, (byte)57, (byte)239, (byte)223, (byte)225, (byte)93, (byte)177, (byte)114, (byte)159, (byte)223, (byte)241, (byte)177, (byte)40, (byte)241, (byte)148, (byte)52, (byte)239, (byte)64, (byte)13, (byte)181, (byte)37, (byte)91, (byte)180, (byte)50, (byte)199, (byte)93, (byte)68, (byte)25, (byte)43, (byte)137, (byte)75, (byte)16, (byte)124, (byte)114, (byte)47, (byte)191, (byte)129, (byte)130, (byte)220, (byte)19, (byte)134, (byte)82, (byte)6, (byte)171, (byte)150, (byte)140, (byte)223, (byte)240, (byte)253, (byte)76, (byte)166, (byte)85, (byte)234, (byte)1, (byte)151, (byte)100, (byte)147, (byte)60, (byte)117, (byte)77, (byte)7, (byte)86, (byte)67, (byte)196, (byte)212, (byte)117, (byte)200, (byte)129, (byte)68, (byte)228, (byte)121, (byte)134, (byte)17, (byte)210, (byte)91, (byte)49, (byte)40, (byte)229, (byte)59, (byte)196, (byte)220, (byte)215, (byte)177, (byte)217, (byte)33, (byte)210, (byte)94, (byte)227, (byte)118, (byte)58, (byte)7, (byte)192, (byte)226, (byte)223, (byte)217, (byte)106, (byte)104, (byte)61, (byte)81, (byte)36, (byte)154, (byte)89, (byte)173, (byte)193, (byte)157, (byte)132, (byte)54, (byte)48, (byte)97, (byte)9, (byte)59, (byte)9, (byte)148, (byte)43, (byte)190, (byte)255, (byte)174, (byte)211, (byte)67, (byte)240, (byte)30, (byte)67, (byte)213, (byte)89, (byte)167, (byte)251, (byte)107, (byte)239, (byte)239, (byte)174, (byte)135, (byte)104, (byte)252, (byte)151, (byte)137, (byte)183, (byte)50, (byte)216, (byte)168, (byte)187, (byte)171, (byte)148, (byte)67, (byte)165, (byte)169, (byte)224, (byte)177, (byte)93, (byte)88, (byte)233, (byte)157, (byte)48, (byte)215, (byte)123, (byte)141, (byte)64, (byte)102, (byte)235, (byte)243, (byte)70, (byte)104, (byte)242, (byte)228, (byte)91, (byte)151, (byte)195, (byte)224, (byte)85, (byte)140, (byte)192, (byte)74, (byte)25, (byte)169, (byte)102, (byte)248, (byte)48, (byte)203, (byte)203, (byte)143, (byte)221, (byte)252, (byte)109, (byte)209, (byte)113, (byte)84}, 0) ;
            p110.target_network = (byte)(byte)174;
            p110.target_component = (byte)(byte)20;
            p110.target_system = (byte)(byte)224;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tc1 == (long)5844488208852771686L);
                Debug.Assert(pack.ts1 == (long) -2641088069258101088L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long) -2641088069258101088L;
            p111.tc1 = (long)5844488208852771686L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)925740567U);
                Debug.Assert(pack.time_usec == (ulong)5770251885980281857L);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)5770251885980281857L;
            p112.seq = (uint)925740567U;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel == (ushort)(ushort)25098);
                Debug.Assert(pack.alt == (int) -1655294878);
                Debug.Assert(pack.lon == (int) -1641012381);
                Debug.Assert(pack.vd == (short)(short)22365);
                Debug.Assert(pack.fix_type == (byte)(byte)179);
                Debug.Assert(pack.vn == (short)(short) -23302);
                Debug.Assert(pack.lat == (int) -1869055175);
                Debug.Assert(pack.satellites_visible == (byte)(byte)124);
                Debug.Assert(pack.cog == (ushort)(ushort)49729);
                Debug.Assert(pack.time_usec == (ulong)128639134051117660L);
                Debug.Assert(pack.epv == (ushort)(ushort)52503);
                Debug.Assert(pack.eph == (ushort)(ushort)39937);
                Debug.Assert(pack.ve == (short)(short)19105);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.cog = (ushort)(ushort)49729;
            p113.vel = (ushort)(ushort)25098;
            p113.satellites_visible = (byte)(byte)124;
            p113.vd = (short)(short)22365;
            p113.time_usec = (ulong)128639134051117660L;
            p113.epv = (ushort)(ushort)52503;
            p113.fix_type = (byte)(byte)179;
            p113.alt = (int) -1655294878;
            p113.vn = (short)(short) -23302;
            p113.ve = (short)(short)19105;
            p113.lon = (int) -1641012381;
            p113.lat = (int) -1869055175;
            p113.eph = (ushort)(ushort)39937;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_xgyro == (float)1.1883009E38F);
                Debug.Assert(pack.integrated_ygyro == (float)2.007813E38F);
                Debug.Assert(pack.time_usec == (ulong)1938601315161706595L);
                Debug.Assert(pack.integrated_x == (float)2.9751893E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)881277567U);
                Debug.Assert(pack.integration_time_us == (uint)3572324963U);
                Debug.Assert(pack.distance == (float) -5.947987E37F);
                Debug.Assert(pack.quality == (byte)(byte)182);
                Debug.Assert(pack.temperature == (short)(short)10706);
                Debug.Assert(pack.integrated_zgyro == (float)1.3387829E37F);
                Debug.Assert(pack.integrated_y == (float) -2.7841534E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)195);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.temperature = (short)(short)10706;
            p114.sensor_id = (byte)(byte)195;
            p114.integration_time_us = (uint)3572324963U;
            p114.integrated_zgyro = (float)1.3387829E37F;
            p114.quality = (byte)(byte)182;
            p114.integrated_xgyro = (float)1.1883009E38F;
            p114.integrated_ygyro = (float)2.007813E38F;
            p114.time_delta_distance_us = (uint)881277567U;
            p114.integrated_y = (float) -2.7841534E38F;
            p114.time_usec = (ulong)1938601315161706595L;
            p114.distance = (float) -5.947987E37F;
            p114.integrated_x = (float)2.9751893E38F;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short)23331);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)45479);
                Debug.Assert(pack.yacc == (short)(short)14179);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {1.6971023E38F, -9.242048E37F, 2.6050713E38F, 2.222349E38F}));
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)13559);
                Debug.Assert(pack.vz == (short)(short) -3948);
                Debug.Assert(pack.lat == (int) -1717985372);
                Debug.Assert(pack.zacc == (short)(short)7935);
                Debug.Assert(pack.rollspeed == (float)1.2841296E38F);
                Debug.Assert(pack.lon == (int)1264780514);
                Debug.Assert(pack.vy == (short)(short) -27007);
                Debug.Assert(pack.time_usec == (ulong)1466765118225801121L);
                Debug.Assert(pack.yawspeed == (float)2.63994E38F);
                Debug.Assert(pack.alt == (int) -1665955920);
                Debug.Assert(pack.vx == (short)(short) -6853);
                Debug.Assert(pack.pitchspeed == (float)3.1324654E38F);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.vz = (short)(short) -3948;
            p115.yawspeed = (float)2.63994E38F;
            p115.lon = (int)1264780514;
            p115.vx = (short)(short) -6853;
            p115.attitude_quaternion_SET(new float[] {1.6971023E38F, -9.242048E37F, 2.6050713E38F, 2.222349E38F}, 0) ;
            p115.pitchspeed = (float)3.1324654E38F;
            p115.ind_airspeed = (ushort)(ushort)13559;
            p115.yacc = (short)(short)14179;
            p115.alt = (int) -1665955920;
            p115.xacc = (short)(short)23331;
            p115.vy = (short)(short) -27007;
            p115.time_usec = (ulong)1466765118225801121L;
            p115.zacc = (short)(short)7935;
            p115.lat = (int) -1717985372;
            p115.true_airspeed = (ushort)(ushort)45479;
            p115.rollspeed = (float)1.2841296E38F;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short)14156);
                Debug.Assert(pack.xacc == (short)(short) -27496);
                Debug.Assert(pack.zgyro == (short)(short) -11547);
                Debug.Assert(pack.xmag == (short)(short)1377);
                Debug.Assert(pack.ymag == (short)(short) -3042);
                Debug.Assert(pack.xgyro == (short)(short) -8787);
                Debug.Assert(pack.zacc == (short)(short) -7901);
                Debug.Assert(pack.zmag == (short)(short) -4509);
                Debug.Assert(pack.time_boot_ms == (uint)3853853947U);
                Debug.Assert(pack.ygyro == (short)(short) -22412);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.ymag = (short)(short) -3042;
            p116.yacc = (short)(short)14156;
            p116.zgyro = (short)(short) -11547;
            p116.xacc = (short)(short) -27496;
            p116.xmag = (short)(short)1377;
            p116.ygyro = (short)(short) -22412;
            p116.zacc = (short)(short) -7901;
            p116.time_boot_ms = (uint)3853853947U;
            p116.xgyro = (short)(short) -8787;
            p116.zmag = (short)(short) -4509;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start == (ushort)(ushort)19281);
                Debug.Assert(pack.end == (ushort)(ushort)36971);
                Debug.Assert(pack.target_component == (byte)(byte)7);
                Debug.Assert(pack.target_system == (byte)(byte)111);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.end = (ushort)(ushort)36971;
            p117.target_system = (byte)(byte)111;
            p117.start = (ushort)(ushort)19281;
            p117.target_component = (byte)(byte)7;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_log_num == (ushort)(ushort)37591);
                Debug.Assert(pack.num_logs == (ushort)(ushort)42982);
                Debug.Assert(pack.time_utc == (uint)2203430695U);
                Debug.Assert(pack.size == (uint)1797298455U);
                Debug.Assert(pack.id == (ushort)(ushort)44047);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.size = (uint)1797298455U;
            p118.time_utc = (uint)2203430695U;
            p118.num_logs = (ushort)(ushort)42982;
            p118.id = (ushort)(ushort)44047;
            p118.last_log_num = (ushort)(ushort)37591;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)12104);
                Debug.Assert(pack.target_system == (byte)(byte)249);
                Debug.Assert(pack.count == (uint)4038133587U);
                Debug.Assert(pack.target_component == (byte)(byte)141);
                Debug.Assert(pack.ofs == (uint)4027141221U);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.count = (uint)4038133587U;
            p119.id = (ushort)(ushort)12104;
            p119.ofs = (uint)4027141221U;
            p119.target_system = (byte)(byte)249;
            p119.target_component = (byte)(byte)141;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)3168172580U);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)171, (byte)59, (byte)220, (byte)44, (byte)135, (byte)198, (byte)89, (byte)4, (byte)108, (byte)213, (byte)107, (byte)169, (byte)2, (byte)136, (byte)238, (byte)211, (byte)232, (byte)20, (byte)157, (byte)161, (byte)156, (byte)170, (byte)248, (byte)67, (byte)232, (byte)73, (byte)253, (byte)39, (byte)40, (byte)221, (byte)150, (byte)82, (byte)63, (byte)216, (byte)96, (byte)5, (byte)19, (byte)220, (byte)82, (byte)195, (byte)244, (byte)186, (byte)200, (byte)140, (byte)233, (byte)70, (byte)243, (byte)20, (byte)251, (byte)168, (byte)250, (byte)192, (byte)230, (byte)110, (byte)248, (byte)63, (byte)231, (byte)190, (byte)157, (byte)194, (byte)218, (byte)123, (byte)207, (byte)138, (byte)157, (byte)123, (byte)56, (byte)86, (byte)174, (byte)84, (byte)159, (byte)180, (byte)217, (byte)148, (byte)248, (byte)79, (byte)59, (byte)240, (byte)224, (byte)251, (byte)104, (byte)137, (byte)20, (byte)139, (byte)11, (byte)30, (byte)7, (byte)222, (byte)136, (byte)11}));
                Debug.Assert(pack.count == (byte)(byte)171);
                Debug.Assert(pack.id == (ushort)(ushort)20368);
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)20368;
            p120.ofs = (uint)3168172580U;
            p120.count = (byte)(byte)171;
            p120.data__SET(new byte[] {(byte)171, (byte)59, (byte)220, (byte)44, (byte)135, (byte)198, (byte)89, (byte)4, (byte)108, (byte)213, (byte)107, (byte)169, (byte)2, (byte)136, (byte)238, (byte)211, (byte)232, (byte)20, (byte)157, (byte)161, (byte)156, (byte)170, (byte)248, (byte)67, (byte)232, (byte)73, (byte)253, (byte)39, (byte)40, (byte)221, (byte)150, (byte)82, (byte)63, (byte)216, (byte)96, (byte)5, (byte)19, (byte)220, (byte)82, (byte)195, (byte)244, (byte)186, (byte)200, (byte)140, (byte)233, (byte)70, (byte)243, (byte)20, (byte)251, (byte)168, (byte)250, (byte)192, (byte)230, (byte)110, (byte)248, (byte)63, (byte)231, (byte)190, (byte)157, (byte)194, (byte)218, (byte)123, (byte)207, (byte)138, (byte)157, (byte)123, (byte)56, (byte)86, (byte)174, (byte)84, (byte)159, (byte)180, (byte)217, (byte)148, (byte)248, (byte)79, (byte)59, (byte)240, (byte)224, (byte)251, (byte)104, (byte)137, (byte)20, (byte)139, (byte)11, (byte)30, (byte)7, (byte)222, (byte)136, (byte)11}, 0) ;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)74);
                Debug.Assert(pack.target_component == (byte)(byte)93);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)74;
            p121.target_component = (byte)(byte)93;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)93);
                Debug.Assert(pack.target_component == (byte)(byte)202);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_component = (byte)(byte)202;
            p122.target_system = (byte)(byte)93;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)22, (byte)180, (byte)146, (byte)189, (byte)227, (byte)51, (byte)40, (byte)251, (byte)142, (byte)163, (byte)41, (byte)142, (byte)38, (byte)91, (byte)97, (byte)152, (byte)11, (byte)240, (byte)129, (byte)56, (byte)246, (byte)100, (byte)104, (byte)106, (byte)230, (byte)84, (byte)146, (byte)10, (byte)119, (byte)64, (byte)15, (byte)103, (byte)16, (byte)58, (byte)208, (byte)251, (byte)215, (byte)206, (byte)16, (byte)153, (byte)245, (byte)220, (byte)136, (byte)180, (byte)51, (byte)95, (byte)162, (byte)237, (byte)242, (byte)250, (byte)146, (byte)56, (byte)194, (byte)11, (byte)124, (byte)172, (byte)133, (byte)211, (byte)244, (byte)20, (byte)228, (byte)115, (byte)220, (byte)165, (byte)181, (byte)214, (byte)237, (byte)83, (byte)36, (byte)208, (byte)77, (byte)203, (byte)69, (byte)49, (byte)214, (byte)87, (byte)138, (byte)63, (byte)121, (byte)242, (byte)212, (byte)38, (byte)210, (byte)99, (byte)223, (byte)77, (byte)214, (byte)97, (byte)194, (byte)57, (byte)27, (byte)238, (byte)213, (byte)205, (byte)134, (byte)153, (byte)115, (byte)138, (byte)151, (byte)136, (byte)144, (byte)115, (byte)94, (byte)103, (byte)94, (byte)72, (byte)52, (byte)214, (byte)53, (byte)176}));
                Debug.Assert(pack.target_component == (byte)(byte)251);
                Debug.Assert(pack.target_system == (byte)(byte)94);
                Debug.Assert(pack.len == (byte)(byte)175);
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.data__SET(new byte[] {(byte)22, (byte)180, (byte)146, (byte)189, (byte)227, (byte)51, (byte)40, (byte)251, (byte)142, (byte)163, (byte)41, (byte)142, (byte)38, (byte)91, (byte)97, (byte)152, (byte)11, (byte)240, (byte)129, (byte)56, (byte)246, (byte)100, (byte)104, (byte)106, (byte)230, (byte)84, (byte)146, (byte)10, (byte)119, (byte)64, (byte)15, (byte)103, (byte)16, (byte)58, (byte)208, (byte)251, (byte)215, (byte)206, (byte)16, (byte)153, (byte)245, (byte)220, (byte)136, (byte)180, (byte)51, (byte)95, (byte)162, (byte)237, (byte)242, (byte)250, (byte)146, (byte)56, (byte)194, (byte)11, (byte)124, (byte)172, (byte)133, (byte)211, (byte)244, (byte)20, (byte)228, (byte)115, (byte)220, (byte)165, (byte)181, (byte)214, (byte)237, (byte)83, (byte)36, (byte)208, (byte)77, (byte)203, (byte)69, (byte)49, (byte)214, (byte)87, (byte)138, (byte)63, (byte)121, (byte)242, (byte)212, (byte)38, (byte)210, (byte)99, (byte)223, (byte)77, (byte)214, (byte)97, (byte)194, (byte)57, (byte)27, (byte)238, (byte)213, (byte)205, (byte)134, (byte)153, (byte)115, (byte)138, (byte)151, (byte)136, (byte)144, (byte)115, (byte)94, (byte)103, (byte)94, (byte)72, (byte)52, (byte)214, (byte)53, (byte)176}, 0) ;
            p123.len = (byte)(byte)175;
            p123.target_system = (byte)(byte)94;
            p123.target_component = (byte)(byte)251;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel == (ushort)(ushort)64585);
                Debug.Assert(pack.satellites_visible == (byte)(byte)198);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
                Debug.Assert(pack.cog == (ushort)(ushort)15991);
                Debug.Assert(pack.alt == (int) -271700193);
                Debug.Assert(pack.lat == (int)1546841980);
                Debug.Assert(pack.lon == (int)91222935);
                Debug.Assert(pack.dgps_numch == (byte)(byte)230);
                Debug.Assert(pack.eph == (ushort)(ushort)58667);
                Debug.Assert(pack.epv == (ushort)(ushort)55388);
                Debug.Assert(pack.dgps_age == (uint)25268105U);
                Debug.Assert(pack.time_usec == (ulong)2983370399638005351L);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.time_usec = (ulong)2983370399638005351L;
            p124.vel = (ushort)(ushort)64585;
            p124.epv = (ushort)(ushort)55388;
            p124.alt = (int) -271700193;
            p124.lat = (int)1546841980;
            p124.dgps_numch = (byte)(byte)230;
            p124.dgps_age = (uint)25268105U;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p124.satellites_visible = (byte)(byte)198;
            p124.cog = (ushort)(ushort)15991;
            p124.eph = (ushort)(ushort)58667;
            p124.lon = (int)91222935;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED));
                Debug.Assert(pack.Vservo == (ushort)(ushort)40697);
                Debug.Assert(pack.Vcc == (ushort)(ushort)1538);
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED);
            p125.Vcc = (ushort)(ushort)1538;
            p125.Vservo = (ushort)(ushort)40697;
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                                            SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND));
                Debug.Assert(pack.timeout == (ushort)(ushort)4572);
                Debug.Assert(pack.baudrate == (uint)2941884141U);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)203, (byte)165, (byte)119, (byte)75, (byte)144, (byte)35, (byte)37, (byte)73, (byte)129, (byte)180, (byte)184, (byte)211, (byte)68, (byte)243, (byte)227, (byte)249, (byte)119, (byte)3, (byte)56, (byte)162, (byte)47, (byte)230, (byte)52, (byte)97, (byte)216, (byte)209, (byte)127, (byte)165, (byte)224, (byte)12, (byte)209, (byte)195, (byte)6, (byte)126, (byte)192, (byte)101, (byte)185, (byte)242, (byte)11, (byte)191, (byte)230, (byte)51, (byte)224, (byte)135, (byte)98, (byte)73, (byte)209, (byte)1, (byte)218, (byte)178, (byte)253, (byte)33, (byte)91, (byte)90, (byte)69, (byte)7, (byte)185, (byte)96, (byte)1, (byte)166, (byte)78, (byte)21, (byte)204, (byte)88, (byte)79, (byte)109, (byte)138, (byte)46, (byte)65, (byte)190}));
                Debug.Assert(pack.device == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
                Debug.Assert(pack.count == (byte)(byte)47);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.timeout = (ushort)(ushort)4572;
            p126.count = (byte)(byte)47;
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1;
            p126.data__SET(new byte[] {(byte)203, (byte)165, (byte)119, (byte)75, (byte)144, (byte)35, (byte)37, (byte)73, (byte)129, (byte)180, (byte)184, (byte)211, (byte)68, (byte)243, (byte)227, (byte)249, (byte)119, (byte)3, (byte)56, (byte)162, (byte)47, (byte)230, (byte)52, (byte)97, (byte)216, (byte)209, (byte)127, (byte)165, (byte)224, (byte)12, (byte)209, (byte)195, (byte)6, (byte)126, (byte)192, (byte)101, (byte)185, (byte)242, (byte)11, (byte)191, (byte)230, (byte)51, (byte)224, (byte)135, (byte)98, (byte)73, (byte)209, (byte)1, (byte)218, (byte)178, (byte)253, (byte)33, (byte)91, (byte)90, (byte)69, (byte)7, (byte)185, (byte)96, (byte)1, (byte)166, (byte)78, (byte)21, (byte)204, (byte)88, (byte)79, (byte)109, (byte)138, (byte)46, (byte)65, (byte)190}, 0) ;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND);
            p126.baudrate = (uint)2941884141U;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_last_baseline_ms == (uint)3840202317U);
                Debug.Assert(pack.rtk_rate == (byte)(byte)197);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)194);
                Debug.Assert(pack.rtk_health == (byte)(byte)44);
                Debug.Assert(pack.baseline_b_mm == (int) -1413771555);
                Debug.Assert(pack.baseline_a_mm == (int)970401409);
                Debug.Assert(pack.iar_num_hypotheses == (int) -89817771);
                Debug.Assert(pack.nsats == (byte)(byte)229);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)141);
                Debug.Assert(pack.wn == (ushort)(ushort)34876);
                Debug.Assert(pack.tow == (uint)781215369U);
                Debug.Assert(pack.baseline_c_mm == (int)2116130118);
                Debug.Assert(pack.accuracy == (uint)3626051066U);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.wn = (ushort)(ushort)34876;
            p127.baseline_a_mm = (int)970401409;
            p127.time_last_baseline_ms = (uint)3840202317U;
            p127.tow = (uint)781215369U;
            p127.baseline_coords_type = (byte)(byte)194;
            p127.nsats = (byte)(byte)229;
            p127.baseline_b_mm = (int) -1413771555;
            p127.rtk_receiver_id = (byte)(byte)141;
            p127.rtk_health = (byte)(byte)44;
            p127.iar_num_hypotheses = (int) -89817771;
            p127.rtk_rate = (byte)(byte)197;
            p127.baseline_c_mm = (int)2116130118;
            p127.accuracy = (uint)3626051066U;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_b_mm == (int)1109245716);
                Debug.Assert(pack.baseline_a_mm == (int) -2096797021);
                Debug.Assert(pack.iar_num_hypotheses == (int)706538628);
                Debug.Assert(pack.rtk_health == (byte)(byte)39);
                Debug.Assert(pack.nsats == (byte)(byte)162);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)115);
                Debug.Assert(pack.baseline_c_mm == (int) -2114551261);
                Debug.Assert(pack.rtk_rate == (byte)(byte)2);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)255);
                Debug.Assert(pack.accuracy == (uint)29905222U);
                Debug.Assert(pack.tow == (uint)3419402674U);
                Debug.Assert(pack.time_last_baseline_ms == (uint)457946237U);
                Debug.Assert(pack.wn == (ushort)(ushort)54987);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.rtk_health = (byte)(byte)39;
            p128.rtk_receiver_id = (byte)(byte)255;
            p128.rtk_rate = (byte)(byte)2;
            p128.baseline_coords_type = (byte)(byte)115;
            p128.nsats = (byte)(byte)162;
            p128.baseline_b_mm = (int)1109245716;
            p128.iar_num_hypotheses = (int)706538628;
            p128.baseline_a_mm = (int) -2096797021;
            p128.wn = (ushort)(ushort)54987;
            p128.baseline_c_mm = (int) -2114551261;
            p128.tow = (uint)3419402674U;
            p128.time_last_baseline_ms = (uint)457946237U;
            p128.accuracy = (uint)29905222U;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (short)(short)20234);
                Debug.Assert(pack.yacc == (short)(short)27214);
                Debug.Assert(pack.xmag == (short)(short)374);
                Debug.Assert(pack.xacc == (short)(short) -1608);
                Debug.Assert(pack.time_boot_ms == (uint)957459516U);
                Debug.Assert(pack.xgyro == (short)(short) -5860);
                Debug.Assert(pack.ymag == (short)(short) -1848);
                Debug.Assert(pack.zmag == (short)(short) -12845);
                Debug.Assert(pack.zacc == (short)(short) -3464);
                Debug.Assert(pack.zgyro == (short)(short) -20723);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.zmag = (short)(short) -12845;
            p129.ygyro = (short)(short)20234;
            p129.xmag = (short)(short)374;
            p129.xgyro = (short)(short) -5860;
            p129.time_boot_ms = (uint)957459516U;
            p129.ymag = (short)(short) -1848;
            p129.zgyro = (short)(short) -20723;
            p129.xacc = (short)(short) -1608;
            p129.yacc = (short)(short)27214;
            p129.zacc = (short)(short) -3464;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.width == (ushort)(ushort)28668);
                Debug.Assert(pack.packets == (ushort)(ushort)48141);
                Debug.Assert(pack.payload == (byte)(byte)191);
                Debug.Assert(pack.height == (ushort)(ushort)65473);
                Debug.Assert(pack.jpg_quality == (byte)(byte)249);
                Debug.Assert(pack.size == (uint)187884610U);
                Debug.Assert(pack.type == (byte)(byte)248);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.height = (ushort)(ushort)65473;
            p130.payload = (byte)(byte)191;
            p130.type = (byte)(byte)248;
            p130.width = (ushort)(ushort)28668;
            p130.packets = (ushort)(ushort)48141;
            p130.jpg_quality = (byte)(byte)249;
            p130.size = (uint)187884610U;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seqnr == (ushort)(ushort)9432);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)248, (byte)186, (byte)39, (byte)155, (byte)175, (byte)177, (byte)151, (byte)227, (byte)178, (byte)156, (byte)0, (byte)65, (byte)224, (byte)78, (byte)109, (byte)1, (byte)96, (byte)146, (byte)154, (byte)244, (byte)73, (byte)214, (byte)14, (byte)228, (byte)142, (byte)191, (byte)26, (byte)131, (byte)55, (byte)250, (byte)174, (byte)115, (byte)208, (byte)102, (byte)113, (byte)2, (byte)255, (byte)184, (byte)153, (byte)138, (byte)183, (byte)8, (byte)158, (byte)224, (byte)181, (byte)201, (byte)217, (byte)97, (byte)136, (byte)114, (byte)207, (byte)199, (byte)141, (byte)5, (byte)189, (byte)204, (byte)160, (byte)27, (byte)158, (byte)190, (byte)152, (byte)64, (byte)45, (byte)186, (byte)237, (byte)238, (byte)213, (byte)21, (byte)62, (byte)53, (byte)201, (byte)32, (byte)69, (byte)114, (byte)210, (byte)121, (byte)247, (byte)248, (byte)153, (byte)196, (byte)24, (byte)82, (byte)184, (byte)250, (byte)216, (byte)43, (byte)226, (byte)113, (byte)76, (byte)8, (byte)227, (byte)53, (byte)75, (byte)232, (byte)209, (byte)125, (byte)0, (byte)71, (byte)137, (byte)12, (byte)73, (byte)46, (byte)5, (byte)162, (byte)96, (byte)135, (byte)1, (byte)50, (byte)69, (byte)13, (byte)53, (byte)122, (byte)240, (byte)232, (byte)167, (byte)18, (byte)91, (byte)15, (byte)2, (byte)177, (byte)133, (byte)146, (byte)159, (byte)83, (byte)32, (byte)142, (byte)154, (byte)189, (byte)67, (byte)106, (byte)207, (byte)136, (byte)95, (byte)130, (byte)137, (byte)25, (byte)47, (byte)35, (byte)163, (byte)59, (byte)207, (byte)117, (byte)28, (byte)83, (byte)75, (byte)34, (byte)19, (byte)187, (byte)237, (byte)192, (byte)177, (byte)6, (byte)137, (byte)231, (byte)44, (byte)17, (byte)105, (byte)130, (byte)142, (byte)230, (byte)162, (byte)189, (byte)192, (byte)102, (byte)240, (byte)160, (byte)12, (byte)25, (byte)67, (byte)128, (byte)35, (byte)130, (byte)131, (byte)91, (byte)104, (byte)84, (byte)85, (byte)190, (byte)120, (byte)131, (byte)67, (byte)137, (byte)171, (byte)25, (byte)127, (byte)171, (byte)76, (byte)179, (byte)240, (byte)2, (byte)139, (byte)5, (byte)200, (byte)224, (byte)3, (byte)90, (byte)96, (byte)232, (byte)28, (byte)248, (byte)145, (byte)84, (byte)132, (byte)148, (byte)223, (byte)200, (byte)43, (byte)110, (byte)224, (byte)245, (byte)165, (byte)90, (byte)172, (byte)112, (byte)86, (byte)80, (byte)171, (byte)232, (byte)88, (byte)117, (byte)23, (byte)186, (byte)194, (byte)189, (byte)159, (byte)255, (byte)103, (byte)191, (byte)157, (byte)228, (byte)7, (byte)2, (byte)38, (byte)38, (byte)21, (byte)145, (byte)217, (byte)251, (byte)41, (byte)166, (byte)230, (byte)136, (byte)148, (byte)30, (byte)157, (byte)33, (byte)110, (byte)12, (byte)34, (byte)28, (byte)206, (byte)228, (byte)254}));
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)9432;
            p131.data__SET(new byte[] {(byte)248, (byte)186, (byte)39, (byte)155, (byte)175, (byte)177, (byte)151, (byte)227, (byte)178, (byte)156, (byte)0, (byte)65, (byte)224, (byte)78, (byte)109, (byte)1, (byte)96, (byte)146, (byte)154, (byte)244, (byte)73, (byte)214, (byte)14, (byte)228, (byte)142, (byte)191, (byte)26, (byte)131, (byte)55, (byte)250, (byte)174, (byte)115, (byte)208, (byte)102, (byte)113, (byte)2, (byte)255, (byte)184, (byte)153, (byte)138, (byte)183, (byte)8, (byte)158, (byte)224, (byte)181, (byte)201, (byte)217, (byte)97, (byte)136, (byte)114, (byte)207, (byte)199, (byte)141, (byte)5, (byte)189, (byte)204, (byte)160, (byte)27, (byte)158, (byte)190, (byte)152, (byte)64, (byte)45, (byte)186, (byte)237, (byte)238, (byte)213, (byte)21, (byte)62, (byte)53, (byte)201, (byte)32, (byte)69, (byte)114, (byte)210, (byte)121, (byte)247, (byte)248, (byte)153, (byte)196, (byte)24, (byte)82, (byte)184, (byte)250, (byte)216, (byte)43, (byte)226, (byte)113, (byte)76, (byte)8, (byte)227, (byte)53, (byte)75, (byte)232, (byte)209, (byte)125, (byte)0, (byte)71, (byte)137, (byte)12, (byte)73, (byte)46, (byte)5, (byte)162, (byte)96, (byte)135, (byte)1, (byte)50, (byte)69, (byte)13, (byte)53, (byte)122, (byte)240, (byte)232, (byte)167, (byte)18, (byte)91, (byte)15, (byte)2, (byte)177, (byte)133, (byte)146, (byte)159, (byte)83, (byte)32, (byte)142, (byte)154, (byte)189, (byte)67, (byte)106, (byte)207, (byte)136, (byte)95, (byte)130, (byte)137, (byte)25, (byte)47, (byte)35, (byte)163, (byte)59, (byte)207, (byte)117, (byte)28, (byte)83, (byte)75, (byte)34, (byte)19, (byte)187, (byte)237, (byte)192, (byte)177, (byte)6, (byte)137, (byte)231, (byte)44, (byte)17, (byte)105, (byte)130, (byte)142, (byte)230, (byte)162, (byte)189, (byte)192, (byte)102, (byte)240, (byte)160, (byte)12, (byte)25, (byte)67, (byte)128, (byte)35, (byte)130, (byte)131, (byte)91, (byte)104, (byte)84, (byte)85, (byte)190, (byte)120, (byte)131, (byte)67, (byte)137, (byte)171, (byte)25, (byte)127, (byte)171, (byte)76, (byte)179, (byte)240, (byte)2, (byte)139, (byte)5, (byte)200, (byte)224, (byte)3, (byte)90, (byte)96, (byte)232, (byte)28, (byte)248, (byte)145, (byte)84, (byte)132, (byte)148, (byte)223, (byte)200, (byte)43, (byte)110, (byte)224, (byte)245, (byte)165, (byte)90, (byte)172, (byte)112, (byte)86, (byte)80, (byte)171, (byte)232, (byte)88, (byte)117, (byte)23, (byte)186, (byte)194, (byte)189, (byte)159, (byte)255, (byte)103, (byte)191, (byte)157, (byte)228, (byte)7, (byte)2, (byte)38, (byte)38, (byte)21, (byte)145, (byte)217, (byte)251, (byte)41, (byte)166, (byte)230, (byte)136, (byte)148, (byte)30, (byte)157, (byte)33, (byte)110, (byte)12, (byte)34, (byte)28, (byte)206, (byte)228, (byte)254}, 0) ;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (byte)(byte)131);
                Debug.Assert(pack.orientation == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_PITCH_180);
                Debug.Assert(pack.time_boot_ms == (uint)2469746655U);
                Debug.Assert(pack.min_distance == (ushort)(ushort)28867);
                Debug.Assert(pack.type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
                Debug.Assert(pack.max_distance == (ushort)(ushort)49244);
                Debug.Assert(pack.current_distance == (ushort)(ushort)9805);
                Debug.Assert(pack.covariance == (byte)(byte)130);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_PITCH_180;
            p132.current_distance = (ushort)(ushort)9805;
            p132.time_boot_ms = (uint)2469746655U;
            p132.id = (byte)(byte)131;
            p132.min_distance = (ushort)(ushort)28867;
            p132.max_distance = (ushort)(ushort)49244;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN;
            p132.covariance = (byte)(byte)130;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -864601271);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)37236);
                Debug.Assert(pack.lat == (int) -1791454407);
                Debug.Assert(pack.mask == (ulong)8450267885778590782L);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.grid_spacing = (ushort)(ushort)37236;
            p133.lon = (int) -864601271;
            p133.lat = (int) -1791454407;
            p133.mask = (ulong)8450267885778590782L;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -597622909);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -28906, (short) -12681, (short)10134, (short) -4881, (short) -16304, (short)22559, (short)15936, (short)30854, (short) -21370, (short) -2658, (short)6667, (short)2874, (short)3020, (short) -19863, (short)1075, (short)11961}));
                Debug.Assert(pack.gridbit == (byte)(byte)59);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)53128);
                Debug.Assert(pack.lat == (int) -1716979228);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lon = (int) -597622909;
            p134.grid_spacing = (ushort)(ushort)53128;
            p134.gridbit = (byte)(byte)59;
            p134.data__SET(new short[] {(short) -28906, (short) -12681, (short)10134, (short) -4881, (short) -16304, (short)22559, (short)15936, (short)30854, (short) -21370, (short) -2658, (short)6667, (short)2874, (short)3020, (short) -19863, (short)1075, (short)11961}, 0) ;
            p134.lat = (int) -1716979228;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)203045843);
                Debug.Assert(pack.lon == (int)135860601);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lon = (int)135860601;
            p135.lat = (int)203045843;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pending == (ushort)(ushort)62885);
                Debug.Assert(pack.lon == (int)1258111193);
                Debug.Assert(pack.spacing == (ushort)(ushort)41500);
                Debug.Assert(pack.lat == (int)884519224);
                Debug.Assert(pack.loaded == (ushort)(ushort)4374);
                Debug.Assert(pack.terrain_height == (float) -3.3938132E38F);
                Debug.Assert(pack.current_height == (float)1.4822788E38F);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lon = (int)1258111193;
            p136.current_height = (float)1.4822788E38F;
            p136.terrain_height = (float) -3.3938132E38F;
            p136.spacing = (ushort)(ushort)41500;
            p136.lat = (int)884519224;
            p136.pending = (ushort)(ushort)62885;
            p136.loaded = (ushort)(ushort)4374;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float)3.0952093E38F);
                Debug.Assert(pack.temperature == (short)(short) -7347);
                Debug.Assert(pack.press_abs == (float)2.8192994E38F);
                Debug.Assert(pack.time_boot_ms == (uint)913934980U);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)913934980U;
            p137.press_diff = (float)3.0952093E38F;
            p137.press_abs = (float)2.8192994E38F;
            p137.temperature = (short)(short) -7347;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)7.817265E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.9922773E38F, 7.8956287E37F, 1.151438E37F, 3.347007E37F}));
                Debug.Assert(pack.x == (float) -6.5913364E37F);
                Debug.Assert(pack.time_usec == (ulong)6508343524074712776L);
                Debug.Assert(pack.z == (float)5.6072704E37F);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.x = (float) -6.5913364E37F;
            p138.time_usec = (ulong)6508343524074712776L;
            p138.q_SET(new float[] {2.9922773E38F, 7.8956287E37F, 1.151438E37F, 3.347007E37F}, 0) ;
            p138.z = (float)5.6072704E37F;
            p138.y = (float)7.817265E37F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)199);
                Debug.Assert(pack.target_system == (byte)(byte)111);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.4864727E37F, 2.9819219E38F, -8.037215E37F, 7.0078225E37F, -2.1819918E38F, 9.681375E37F, -3.271867E38F, 4.9832547E37F}));
                Debug.Assert(pack.time_usec == (ulong)2815127883117891204L);
                Debug.Assert(pack.target_component == (byte)(byte)44);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.group_mlx = (byte)(byte)199;
            p139.time_usec = (ulong)2815127883117891204L;
            p139.target_system = (byte)(byte)111;
            p139.controls_SET(new float[] {-1.4864727E37F, 2.9819219E38F, -8.037215E37F, 7.0078225E37F, -2.1819918E38F, 9.681375E37F, -3.271867E38F, 4.9832547E37F}, 0) ;
            p139.target_component = (byte)(byte)44;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)2170302876458069184L);
                Debug.Assert(pack.group_mlx == (byte)(byte)55);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-6.7628947E37F, 2.5783756E38F, 8.1469603E36F, -1.1284689E38F, -8.143954E37F, -2.8164963E38F, 6.6610076E36F, 3.0668754E38F}));
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.group_mlx = (byte)(byte)55;
            p140.time_usec = (ulong)2170302876458069184L;
            p140.controls_SET(new float[] {-6.7628947E37F, 2.5783756E38F, 8.1469603E36F, -1.1284689E38F, -8.143954E37F, -2.8164963E38F, 6.6610076E36F, 3.0668754E38F}, 0) ;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.bottom_clearance == (float)1.3095089E38F);
                Debug.Assert(pack.altitude_monotonic == (float) -3.9022905E37F);
                Debug.Assert(pack.time_usec == (ulong)8113063585615379842L);
                Debug.Assert(pack.altitude_amsl == (float) -1.365925E38F);
                Debug.Assert(pack.altitude_relative == (float)2.1600683E38F);
                Debug.Assert(pack.altitude_local == (float) -4.0138562E37F);
                Debug.Assert(pack.altitude_terrain == (float)2.1708011E38F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)8113063585615379842L;
            p141.altitude_local = (float) -4.0138562E37F;
            p141.altitude_relative = (float)2.1600683E38F;
            p141.altitude_terrain = (float)2.1708011E38F;
            p141.altitude_monotonic = (float) -3.9022905E37F;
            p141.bottom_clearance = (float)1.3095089E38F;
            p141.altitude_amsl = (float) -1.365925E38F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)149, (byte)146, (byte)151, (byte)233, (byte)244, (byte)217, (byte)170, (byte)205, (byte)204, (byte)236, (byte)114, (byte)217, (byte)62, (byte)102, (byte)165, (byte)243, (byte)198, (byte)144, (byte)176, (byte)117, (byte)126, (byte)14, (byte)110, (byte)133, (byte)15, (byte)156, (byte)75, (byte)34, (byte)78, (byte)214, (byte)221, (byte)11, (byte)72, (byte)178, (byte)145, (byte)200, (byte)239, (byte)209, (byte)177, (byte)108, (byte)243, (byte)56, (byte)86, (byte)102, (byte)115, (byte)226, (byte)77, (byte)70, (byte)73, (byte)142, (byte)186, (byte)218, (byte)52, (byte)186, (byte)100, (byte)196, (byte)200, (byte)40, (byte)141, (byte)168, (byte)191, (byte)12, (byte)224, (byte)171, (byte)242, (byte)23, (byte)6, (byte)106, (byte)148, (byte)65, (byte)66, (byte)86, (byte)166, (byte)137, (byte)116, (byte)224, (byte)72, (byte)253, (byte)207, (byte)118, (byte)223, (byte)3, (byte)174, (byte)118, (byte)57, (byte)79, (byte)98, (byte)177, (byte)209, (byte)21, (byte)40, (byte)45, (byte)90, (byte)81, (byte)132, (byte)105, (byte)71, (byte)72, (byte)235, (byte)0, (byte)88, (byte)174, (byte)65, (byte)241, (byte)183, (byte)249, (byte)102, (byte)237, (byte)1, (byte)246, (byte)177, (byte)198, (byte)22, (byte)17, (byte)204, (byte)145, (byte)112, (byte)217, (byte)3, (byte)11}));
                Debug.Assert(pack.request_id == (byte)(byte)66);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)7, (byte)77, (byte)144, (byte)67, (byte)41, (byte)254, (byte)25, (byte)205, (byte)108, (byte)252, (byte)138, (byte)3, (byte)154, (byte)210, (byte)158, (byte)209, (byte)70, (byte)190, (byte)174, (byte)135, (byte)231, (byte)244, (byte)8, (byte)160, (byte)226, (byte)165, (byte)235, (byte)189, (byte)134, (byte)242, (byte)80, (byte)169, (byte)55, (byte)71, (byte)74, (byte)35, (byte)24, (byte)227, (byte)255, (byte)224, (byte)128, (byte)90, (byte)117, (byte)152, (byte)135, (byte)24, (byte)21, (byte)36, (byte)251, (byte)238, (byte)100, (byte)241, (byte)15, (byte)224, (byte)157, (byte)24, (byte)91, (byte)33, (byte)47, (byte)107, (byte)219, (byte)76, (byte)59, (byte)194, (byte)110, (byte)78, (byte)21, (byte)184, (byte)192, (byte)14, (byte)35, (byte)109, (byte)65, (byte)16, (byte)141, (byte)129, (byte)212, (byte)61, (byte)6, (byte)246, (byte)83, (byte)19, (byte)1, (byte)45, (byte)58, (byte)89, (byte)110, (byte)247, (byte)87, (byte)193, (byte)150, (byte)66, (byte)26, (byte)183, (byte)153, (byte)160, (byte)59, (byte)56, (byte)204, (byte)51, (byte)170, (byte)239, (byte)84, (byte)113, (byte)141, (byte)51, (byte)154, (byte)226, (byte)210, (byte)52, (byte)126, (byte)77, (byte)195, (byte)84, (byte)165, (byte)76, (byte)185, (byte)134, (byte)92, (byte)79}));
                Debug.Assert(pack.transfer_type == (byte)(byte)242);
                Debug.Assert(pack.uri_type == (byte)(byte)54);
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.transfer_type = (byte)(byte)242;
            p142.request_id = (byte)(byte)66;
            p142.uri_SET(new byte[] {(byte)7, (byte)77, (byte)144, (byte)67, (byte)41, (byte)254, (byte)25, (byte)205, (byte)108, (byte)252, (byte)138, (byte)3, (byte)154, (byte)210, (byte)158, (byte)209, (byte)70, (byte)190, (byte)174, (byte)135, (byte)231, (byte)244, (byte)8, (byte)160, (byte)226, (byte)165, (byte)235, (byte)189, (byte)134, (byte)242, (byte)80, (byte)169, (byte)55, (byte)71, (byte)74, (byte)35, (byte)24, (byte)227, (byte)255, (byte)224, (byte)128, (byte)90, (byte)117, (byte)152, (byte)135, (byte)24, (byte)21, (byte)36, (byte)251, (byte)238, (byte)100, (byte)241, (byte)15, (byte)224, (byte)157, (byte)24, (byte)91, (byte)33, (byte)47, (byte)107, (byte)219, (byte)76, (byte)59, (byte)194, (byte)110, (byte)78, (byte)21, (byte)184, (byte)192, (byte)14, (byte)35, (byte)109, (byte)65, (byte)16, (byte)141, (byte)129, (byte)212, (byte)61, (byte)6, (byte)246, (byte)83, (byte)19, (byte)1, (byte)45, (byte)58, (byte)89, (byte)110, (byte)247, (byte)87, (byte)193, (byte)150, (byte)66, (byte)26, (byte)183, (byte)153, (byte)160, (byte)59, (byte)56, (byte)204, (byte)51, (byte)170, (byte)239, (byte)84, (byte)113, (byte)141, (byte)51, (byte)154, (byte)226, (byte)210, (byte)52, (byte)126, (byte)77, (byte)195, (byte)84, (byte)165, (byte)76, (byte)185, (byte)134, (byte)92, (byte)79}, 0) ;
            p142.uri_type = (byte)(byte)54;
            p142.storage_SET(new byte[] {(byte)149, (byte)146, (byte)151, (byte)233, (byte)244, (byte)217, (byte)170, (byte)205, (byte)204, (byte)236, (byte)114, (byte)217, (byte)62, (byte)102, (byte)165, (byte)243, (byte)198, (byte)144, (byte)176, (byte)117, (byte)126, (byte)14, (byte)110, (byte)133, (byte)15, (byte)156, (byte)75, (byte)34, (byte)78, (byte)214, (byte)221, (byte)11, (byte)72, (byte)178, (byte)145, (byte)200, (byte)239, (byte)209, (byte)177, (byte)108, (byte)243, (byte)56, (byte)86, (byte)102, (byte)115, (byte)226, (byte)77, (byte)70, (byte)73, (byte)142, (byte)186, (byte)218, (byte)52, (byte)186, (byte)100, (byte)196, (byte)200, (byte)40, (byte)141, (byte)168, (byte)191, (byte)12, (byte)224, (byte)171, (byte)242, (byte)23, (byte)6, (byte)106, (byte)148, (byte)65, (byte)66, (byte)86, (byte)166, (byte)137, (byte)116, (byte)224, (byte)72, (byte)253, (byte)207, (byte)118, (byte)223, (byte)3, (byte)174, (byte)118, (byte)57, (byte)79, (byte)98, (byte)177, (byte)209, (byte)21, (byte)40, (byte)45, (byte)90, (byte)81, (byte)132, (byte)105, (byte)71, (byte)72, (byte)235, (byte)0, (byte)88, (byte)174, (byte)65, (byte)241, (byte)183, (byte)249, (byte)102, (byte)237, (byte)1, (byte)246, (byte)177, (byte)198, (byte)22, (byte)17, (byte)204, (byte)145, (byte)112, (byte)217, (byte)3, (byte)11}, 0) ;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -16306);
                Debug.Assert(pack.press_abs == (float) -2.059365E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3054885104U);
                Debug.Assert(pack.press_diff == (float)1.6627828E38F);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.temperature = (short)(short) -16306;
            p143.press_abs = (float) -2.059365E38F;
            p143.press_diff = (float)1.6627828E38F;
            p143.time_boot_ms = (uint)3054885104U;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.est_capabilities == (byte)(byte)62);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {-2.6998897E38F, -1.0087525E38F, 3.2090583E38F}));
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-1.5102222E38F, -2.706593E38F, 2.3059503E38F}));
                Debug.Assert(pack.acc.SequenceEqual(new float[] {2.696965E37F, -3.0210854E38F, 1.9736933E37F}));
                Debug.Assert(pack.lat == (int)1086606259);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {1.1822438E37F, -2.4160068E38F, 5.3448814E36F, 2.6963523E38F}));
                Debug.Assert(pack.timestamp == (ulong)8620424798813360022L);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-1.0331958E38F, 2.0713974E37F, 3.1538273E38F}));
                Debug.Assert(pack.lon == (int)1375154764);
                Debug.Assert(pack.custom_state == (ulong)1188611127870537182L);
                Debug.Assert(pack.alt == (float) -9.659575E37F);
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.attitude_q_SET(new float[] {1.1822438E37F, -2.4160068E38F, 5.3448814E36F, 2.6963523E38F}, 0) ;
            p144.acc_SET(new float[] {2.696965E37F, -3.0210854E38F, 1.9736933E37F}, 0) ;
            p144.lat = (int)1086606259;
            p144.custom_state = (ulong)1188611127870537182L;
            p144.position_cov_SET(new float[] {-2.6998897E38F, -1.0087525E38F, 3.2090583E38F}, 0) ;
            p144.vel_SET(new float[] {-1.0331958E38F, 2.0713974E37F, 3.1538273E38F}, 0) ;
            p144.rates_SET(new float[] {-1.5102222E38F, -2.706593E38F, 2.3059503E38F}, 0) ;
            p144.alt = (float) -9.659575E37F;
            p144.timestamp = (ulong)8620424798813360022L;
            p144.lon = (int)1375154764;
            p144.est_capabilities = (byte)(byte)62;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.1431116E38F, 3.187569E38F, 5.681343E37F, 3.2839933E38F}));
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {1.0533577E38F, -2.0108273E38F, -2.207611E38F}));
                Debug.Assert(pack.x_acc == (float)1.3181727E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {2.92409E38F, -2.2215195E38F, 3.1079055E38F}));
                Debug.Assert(pack.y_acc == (float)8.3762077E37F);
                Debug.Assert(pack.z_acc == (float)2.7407533E38F);
                Debug.Assert(pack.x_vel == (float)2.2059449E38F);
                Debug.Assert(pack.y_pos == (float) -3.156102E38F);
                Debug.Assert(pack.x_pos == (float)2.0528857E38F);
                Debug.Assert(pack.y_vel == (float) -7.658295E37F);
                Debug.Assert(pack.pitch_rate == (float) -3.377416E38F);
                Debug.Assert(pack.z_pos == (float)9.236817E37F);
                Debug.Assert(pack.z_vel == (float)2.5700358E37F);
                Debug.Assert(pack.airspeed == (float) -2.9360364E38F);
                Debug.Assert(pack.roll_rate == (float)2.4267183E38F);
                Debug.Assert(pack.time_usec == (ulong)3578760597396180480L);
                Debug.Assert(pack.yaw_rate == (float) -1.749945E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.airspeed = (float) -2.9360364E38F;
            p146.x_vel = (float)2.2059449E38F;
            p146.time_usec = (ulong)3578760597396180480L;
            p146.z_acc = (float)2.7407533E38F;
            p146.x_pos = (float)2.0528857E38F;
            p146.z_pos = (float)9.236817E37F;
            p146.y_acc = (float)8.3762077E37F;
            p146.q_SET(new float[] {-1.1431116E38F, 3.187569E38F, 5.681343E37F, 3.2839933E38F}, 0) ;
            p146.vel_variance_SET(new float[] {1.0533577E38F, -2.0108273E38F, -2.207611E38F}, 0) ;
            p146.y_vel = (float) -7.658295E37F;
            p146.x_acc = (float)1.3181727E38F;
            p146.pos_variance_SET(new float[] {2.92409E38F, -2.2215195E38F, 3.1079055E38F}, 0) ;
            p146.roll_rate = (float)2.4267183E38F;
            p146.yaw_rate = (float) -1.749945E38F;
            p146.y_pos = (float) -3.156102E38F;
            p146.z_vel = (float)2.5700358E37F;
            p146.pitch_rate = (float) -3.377416E38F;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 55);
                Debug.Assert(pack.energy_consumed == (int) -770190260);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)17391, (ushort)44055, (ushort)14950, (ushort)65504, (ushort)10880, (ushort)48001, (ushort)45684, (ushort)27336, (ushort)24940, (ushort)41597}));
                Debug.Assert(pack.battery_function == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS);
                Debug.Assert(pack.type == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
                Debug.Assert(pack.current_battery == (short)(short)21314);
                Debug.Assert(pack.current_consumed == (int) -789096706);
                Debug.Assert(pack.temperature == (short)(short)8970);
                Debug.Assert(pack.id == (byte)(byte)25);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH;
            p147.id = (byte)(byte)25;
            p147.battery_remaining = (sbyte)(sbyte) - 55;
            p147.energy_consumed = (int) -770190260;
            p147.current_battery = (short)(short)21314;
            p147.voltages_SET(new ushort[] {(ushort)17391, (ushort)44055, (ushort)14950, (ushort)65504, (ushort)10880, (ushort)48001, (ushort)45684, (ushort)27336, (ushort)24940, (ushort)41597}, 0) ;
            p147.current_consumed = (int) -789096706;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS;
            p147.temperature = (short)(short)8970;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)164, (byte)52, (byte)123, (byte)0, (byte)208, (byte)232, (byte)161, (byte)152, (byte)64, (byte)70, (byte)27, (byte)200, (byte)76, (byte)181, (byte)115, (byte)53, (byte)127, (byte)70}));
                Debug.Assert(pack.os_sw_version == (uint)1773443148U);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)54, (byte)230, (byte)120, (byte)7, (byte)99, (byte)163, (byte)155, (byte)156}));
                Debug.Assert(pack.middleware_sw_version == (uint)2726046417U);
                Debug.Assert(pack.flight_sw_version == (uint)1898491486U);
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)144, (byte)31, (byte)76, (byte)178, (byte)15, (byte)51, (byte)81, (byte)18}));
                Debug.Assert(pack.uid == (ulong)2967403338186604530L);
                Debug.Assert(pack.board_version == (uint)2552746513U);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)172, (byte)94, (byte)226, (byte)38, (byte)107, (byte)54, (byte)83, (byte)147}));
                Debug.Assert(pack.product_id == (ushort)(ushort)20750);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET));
                Debug.Assert(pack.vendor_id == (ushort)(ushort)39744);
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.uid = (ulong)2967403338186604530L;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET);
            p148.os_sw_version = (uint)1773443148U;
            p148.middleware_custom_version_SET(new byte[] {(byte)54, (byte)230, (byte)120, (byte)7, (byte)99, (byte)163, (byte)155, (byte)156}, 0) ;
            p148.uid2_SET(new byte[] {(byte)164, (byte)52, (byte)123, (byte)0, (byte)208, (byte)232, (byte)161, (byte)152, (byte)64, (byte)70, (byte)27, (byte)200, (byte)76, (byte)181, (byte)115, (byte)53, (byte)127, (byte)70}, 0, PH) ;
            p148.os_custom_version_SET(new byte[] {(byte)144, (byte)31, (byte)76, (byte)178, (byte)15, (byte)51, (byte)81, (byte)18}, 0) ;
            p148.flight_sw_version = (uint)1898491486U;
            p148.vendor_id = (ushort)(ushort)39744;
            p148.flight_custom_version_SET(new byte[] {(byte)172, (byte)94, (byte)226, (byte)38, (byte)107, (byte)54, (byte)83, (byte)147}, 0) ;
            p148.board_version = (uint)2552746513U;
            p148.product_id = (ushort)(ushort)20750;
            p148.middleware_sw_version = (uint)2726046417U;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.x_TRY(ph) == (float) -2.6265485E38F);
                Debug.Assert(pack.angle_y == (float) -1.8449973E38F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)150);
                Debug.Assert(pack.distance == (float)1.7316089E38F);
                Debug.Assert(pack.z_TRY(ph) == (float) -2.5717584E38F);
                Debug.Assert(pack.y_TRY(ph) == (float)3.064834E38F);
                Debug.Assert(pack.size_x == (float)2.0129168E38F);
                Debug.Assert(pack.target_num == (byte)(byte)173);
                Debug.Assert(pack.size_y == (float)2.5366956E38F);
                Debug.Assert(pack.angle_x == (float) -1.4139519E38F);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {1.2041077E38F, -1.9357793E38F, 3.3519095E38F, 7.336532E36F}));
                Debug.Assert(pack.type == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
                Debug.Assert(pack.time_usec == (ulong)6846118903039706590L);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.time_usec = (ulong)6846118903039706590L;
            p149.size_y = (float)2.5366956E38F;
            p149.angle_y = (float) -1.8449973E38F;
            p149.x_SET((float) -2.6265485E38F, PH) ;
            p149.angle_x = (float) -1.4139519E38F;
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.z_SET((float) -2.5717584E38F, PH) ;
            p149.y_SET((float)3.064834E38F, PH) ;
            p149.q_SET(new float[] {1.2041077E38F, -1.9357793E38F, 3.3519095E38F, 7.336532E36F}, 0, PH) ;
            p149.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p149.target_num = (byte)(byte)173;
            p149.position_valid_SET((byte)(byte)150, PH) ;
            p149.size_x = (float)2.0129168E38F;
            p149.distance = (float)1.7316089E38F;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE));
                Debug.Assert(pack.time_usec == (ulong)7346443647417837306L);
                Debug.Assert(pack.pos_horiz_accuracy == (float) -5.8052374E37F);
                Debug.Assert(pack.mag_ratio == (float) -4.2514202E37F);
                Debug.Assert(pack.pos_vert_accuracy == (float)3.868233E37F);
                Debug.Assert(pack.vel_ratio == (float)6.244095E37F);
                Debug.Assert(pack.pos_horiz_ratio == (float)1.8404171E38F);
                Debug.Assert(pack.pos_vert_ratio == (float)2.094035E38F);
                Debug.Assert(pack.hagl_ratio == (float)2.2138485E38F);
                Debug.Assert(pack.tas_ratio == (float) -1.9753905E38F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.pos_horiz_ratio = (float)1.8404171E38F;
            p230.pos_horiz_accuracy = (float) -5.8052374E37F;
            p230.mag_ratio = (float) -4.2514202E37F;
            p230.tas_ratio = (float) -1.9753905E38F;
            p230.pos_vert_accuracy = (float)3.868233E37F;
            p230.time_usec = (ulong)7346443647417837306L;
            p230.pos_vert_ratio = (float)2.094035E38F;
            p230.hagl_ratio = (float)2.2138485E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE);
            p230.vel_ratio = (float)6.244095E37F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vert_accuracy == (float)5.2351196E37F);
                Debug.Assert(pack.var_horiz == (float)1.0261456E38F);
                Debug.Assert(pack.wind_z == (float)1.400046E38F);
                Debug.Assert(pack.time_usec == (ulong)4857779868689167891L);
                Debug.Assert(pack.wind_y == (float) -2.2690175E38F);
                Debug.Assert(pack.horiz_accuracy == (float)3.331899E38F);
                Debug.Assert(pack.wind_x == (float)2.678361E38F);
                Debug.Assert(pack.wind_alt == (float)1.9372226E38F);
                Debug.Assert(pack.var_vert == (float) -3.2002359E38F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.wind_x = (float)2.678361E38F;
            p231.var_vert = (float) -3.2002359E38F;
            p231.wind_y = (float) -2.2690175E38F;
            p231.horiz_accuracy = (float)3.331899E38F;
            p231.wind_z = (float)1.400046E38F;
            p231.vert_accuracy = (float)5.2351196E37F;
            p231.wind_alt = (float)1.9372226E38F;
            p231.time_usec = (ulong)4857779868689167891L;
            p231.var_horiz = (float)1.0261456E38F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hdop == (float) -2.3872226E38F);
                Debug.Assert(pack.horiz_accuracy == (float)2.6356717E38F);
                Debug.Assert(pack.lon == (int)884518440);
                Debug.Assert(pack.vd == (float)6.065281E37F);
                Debug.Assert(pack.time_week_ms == (uint)3096837128U);
                Debug.Assert(pack.alt == (float)3.2113776E38F);
                Debug.Assert(pack.time_usec == (ulong)1275078402169907521L);
                Debug.Assert(pack.vdop == (float) -2.5271895E38F);
                Debug.Assert(pack.speed_accuracy == (float)1.6501969E38F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP));
                Debug.Assert(pack.vert_accuracy == (float)2.2295812E38F);
                Debug.Assert(pack.vn == (float) -3.3323989E38F);
                Debug.Assert(pack.ve == (float) -9.97585E37F);
                Debug.Assert(pack.lat == (int) -1040125734);
                Debug.Assert(pack.gps_id == (byte)(byte)34);
                Debug.Assert(pack.fix_type == (byte)(byte)79);
                Debug.Assert(pack.satellites_visible == (byte)(byte)25);
                Debug.Assert(pack.time_week == (ushort)(ushort)27866);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP);
            p232.vd = (float)6.065281E37F;
            p232.lat = (int) -1040125734;
            p232.time_week_ms = (uint)3096837128U;
            p232.time_usec = (ulong)1275078402169907521L;
            p232.vn = (float) -3.3323989E38F;
            p232.ve = (float) -9.97585E37F;
            p232.vdop = (float) -2.5271895E38F;
            p232.horiz_accuracy = (float)2.6356717E38F;
            p232.vert_accuracy = (float)2.2295812E38F;
            p232.fix_type = (byte)(byte)79;
            p232.hdop = (float) -2.3872226E38F;
            p232.gps_id = (byte)(byte)34;
            p232.lon = (int)884518440;
            p232.time_week = (ushort)(ushort)27866;
            p232.satellites_visible = (byte)(byte)25;
            p232.speed_accuracy = (float)1.6501969E38F;
            p232.alt = (float)3.2113776E38F;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (byte)(byte)15);
                Debug.Assert(pack.len == (byte)(byte)124);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)172, (byte)116, (byte)111, (byte)38, (byte)190, (byte)144, (byte)236, (byte)245, (byte)230, (byte)52, (byte)108, (byte)245, (byte)98, (byte)93, (byte)132, (byte)76, (byte)111, (byte)230, (byte)6, (byte)31, (byte)106, (byte)218, (byte)63, (byte)177, (byte)63, (byte)185, (byte)51, (byte)85, (byte)209, (byte)168, (byte)141, (byte)246, (byte)228, (byte)18, (byte)237, (byte)215, (byte)26, (byte)105, (byte)160, (byte)225, (byte)184, (byte)33, (byte)114, (byte)60, (byte)212, (byte)250, (byte)40, (byte)64, (byte)218, (byte)215, (byte)84, (byte)127, (byte)201, (byte)242, (byte)141, (byte)60, (byte)95, (byte)133, (byte)88, (byte)75, (byte)205, (byte)222, (byte)183, (byte)188, (byte)142, (byte)185, (byte)236, (byte)151, (byte)88, (byte)45, (byte)65, (byte)9, (byte)198, (byte)214, (byte)72, (byte)155, (byte)208, (byte)65, (byte)65, (byte)83, (byte)176, (byte)159, (byte)158, (byte)20, (byte)43, (byte)186, (byte)225, (byte)254, (byte)128, (byte)34, (byte)142, (byte)170, (byte)90, (byte)135, (byte)222, (byte)21, (byte)233, (byte)123, (byte)65, (byte)248, (byte)94, (byte)161, (byte)50, (byte)217, (byte)220, (byte)159, (byte)18, (byte)54, (byte)223, (byte)94, (byte)4, (byte)89, (byte)208, (byte)202, (byte)28, (byte)161, (byte)193, (byte)149, (byte)151, (byte)6, (byte)183, (byte)68, (byte)114, (byte)161, (byte)113, (byte)10, (byte)82, (byte)160, (byte)40, (byte)132, (byte)30, (byte)187, (byte)157, (byte)16, (byte)198, (byte)134, (byte)76, (byte)215, (byte)23, (byte)246, (byte)76, (byte)105, (byte)188, (byte)235, (byte)83, (byte)0, (byte)234, (byte)115, (byte)123, (byte)148, (byte)115, (byte)11, (byte)202, (byte)193, (byte)101, (byte)91, (byte)99, (byte)164, (byte)110, (byte)163, (byte)121, (byte)130, (byte)250, (byte)222, (byte)79, (byte)60, (byte)97, (byte)170, (byte)192, (byte)137, (byte)233, (byte)79, (byte)164, (byte)246, (byte)244, (byte)172, (byte)35, (byte)153, (byte)84, (byte)120}));
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)15;
            p233.data__SET(new byte[] {(byte)172, (byte)116, (byte)111, (byte)38, (byte)190, (byte)144, (byte)236, (byte)245, (byte)230, (byte)52, (byte)108, (byte)245, (byte)98, (byte)93, (byte)132, (byte)76, (byte)111, (byte)230, (byte)6, (byte)31, (byte)106, (byte)218, (byte)63, (byte)177, (byte)63, (byte)185, (byte)51, (byte)85, (byte)209, (byte)168, (byte)141, (byte)246, (byte)228, (byte)18, (byte)237, (byte)215, (byte)26, (byte)105, (byte)160, (byte)225, (byte)184, (byte)33, (byte)114, (byte)60, (byte)212, (byte)250, (byte)40, (byte)64, (byte)218, (byte)215, (byte)84, (byte)127, (byte)201, (byte)242, (byte)141, (byte)60, (byte)95, (byte)133, (byte)88, (byte)75, (byte)205, (byte)222, (byte)183, (byte)188, (byte)142, (byte)185, (byte)236, (byte)151, (byte)88, (byte)45, (byte)65, (byte)9, (byte)198, (byte)214, (byte)72, (byte)155, (byte)208, (byte)65, (byte)65, (byte)83, (byte)176, (byte)159, (byte)158, (byte)20, (byte)43, (byte)186, (byte)225, (byte)254, (byte)128, (byte)34, (byte)142, (byte)170, (byte)90, (byte)135, (byte)222, (byte)21, (byte)233, (byte)123, (byte)65, (byte)248, (byte)94, (byte)161, (byte)50, (byte)217, (byte)220, (byte)159, (byte)18, (byte)54, (byte)223, (byte)94, (byte)4, (byte)89, (byte)208, (byte)202, (byte)28, (byte)161, (byte)193, (byte)149, (byte)151, (byte)6, (byte)183, (byte)68, (byte)114, (byte)161, (byte)113, (byte)10, (byte)82, (byte)160, (byte)40, (byte)132, (byte)30, (byte)187, (byte)157, (byte)16, (byte)198, (byte)134, (byte)76, (byte)215, (byte)23, (byte)246, (byte)76, (byte)105, (byte)188, (byte)235, (byte)83, (byte)0, (byte)234, (byte)115, (byte)123, (byte)148, (byte)115, (byte)11, (byte)202, (byte)193, (byte)101, (byte)91, (byte)99, (byte)164, (byte)110, (byte)163, (byte)121, (byte)130, (byte)250, (byte)222, (byte)79, (byte)60, (byte)97, (byte)170, (byte)192, (byte)137, (byte)233, (byte)79, (byte)164, (byte)246, (byte)244, (byte)172, (byte)35, (byte)153, (byte)84, (byte)120}, 0) ;
            p233.len = (byte)(byte)124;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.airspeed == (byte)(byte)242);
                Debug.Assert(pack.heading == (ushort)(ushort)54091);
                Debug.Assert(pack.altitude_amsl == (short)(short)6779);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)214);
                Debug.Assert(pack.temperature == (sbyte)(sbyte)84);
                Debug.Assert(pack.wp_num == (byte)(byte)151);
                Debug.Assert(pack.failsafe == (byte)(byte)64);
                Debug.Assert(pack.pitch == (short)(short)12889);
                Debug.Assert(pack.heading_sp == (short)(short) -21561);
                Debug.Assert(pack.latitude == (int) -353698506);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 12);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)1666);
                Debug.Assert(pack.longitude == (int) -1355003665);
                Debug.Assert(pack.throttle == (sbyte)(sbyte) - 121);
                Debug.Assert(pack.roll == (short)(short)27854);
                Debug.Assert(pack.altitude_sp == (short)(short) -31942);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
                Debug.Assert(pack.gps_nsat == (byte)(byte)106);
                Debug.Assert(pack.custom_mode == (uint)4070572718U);
                Debug.Assert(pack.gps_fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED));
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)55);
                Debug.Assert(pack.groundspeed == (byte)(byte)25);
                Debug.Assert(pack.battery_remaining == (byte)(byte)127);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.climb_rate = (sbyte)(sbyte) - 12;
            p234.longitude = (int) -1355003665;
            p234.roll = (short)(short)27854;
            p234.pitch = (short)(short)12889;
            p234.custom_mode = (uint)4070572718U;
            p234.wp_distance = (ushort)(ushort)1666;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC;
            p234.heading = (ushort)(ushort)54091;
            p234.latitude = (int) -353698506;
            p234.temperature_air = (sbyte)(sbyte)55;
            p234.groundspeed = (byte)(byte)25;
            p234.altitude_sp = (short)(short) -31942;
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
            p234.altitude_amsl = (short)(short)6779;
            p234.failsafe = (byte)(byte)64;
            p234.throttle = (sbyte)(sbyte) - 121;
            p234.heading_sp = (short)(short) -21561;
            p234.temperature = (sbyte)(sbyte)84;
            p234.wp_num = (byte)(byte)151;
            p234.airspeed = (byte)(byte)242;
            p234.airspeed_sp = (byte)(byte)214;
            p234.battery_remaining = (byte)(byte)127;
            p234.gps_nsat = (byte)(byte)106;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.clipping_1 == (uint)3956157373U);
                Debug.Assert(pack.vibration_y == (float)2.9126542E38F);
                Debug.Assert(pack.time_usec == (ulong)3259059076001613796L);
                Debug.Assert(pack.vibration_x == (float) -2.9805467E38F);
                Debug.Assert(pack.vibration_z == (float)1.7668194E38F);
                Debug.Assert(pack.clipping_2 == (uint)1013363795U);
                Debug.Assert(pack.clipping_0 == (uint)3159779432U);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.clipping_1 = (uint)3956157373U;
            p241.clipping_2 = (uint)1013363795U;
            p241.vibration_z = (float)1.7668194E38F;
            p241.vibration_y = (float)2.9126542E38F;
            p241.clipping_0 = (uint)3159779432U;
            p241.vibration_x = (float) -2.9805467E38F;
            p241.time_usec = (ulong)3259059076001613796L;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -2.7205976E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.0185875E38F, -1.9232951E38F, -2.2485846E38F, -1.6451103E38F}));
                Debug.Assert(pack.x == (float) -3.0963875E38F);
                Debug.Assert(pack.z == (float)2.6006818E38F);
                Debug.Assert(pack.approach_y == (float)9.236247E37F);
                Debug.Assert(pack.longitude == (int) -1332973988);
                Debug.Assert(pack.latitude == (int) -874668999);
                Debug.Assert(pack.approach_x == (float) -3.2341202E37F);
                Debug.Assert(pack.approach_z == (float) -2.4174683E38F);
                Debug.Assert(pack.altitude == (int) -1372069912);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)3242999786573484496L);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.approach_y = (float)9.236247E37F;
            p242.x = (float) -3.0963875E38F;
            p242.approach_x = (float) -3.2341202E37F;
            p242.longitude = (int) -1332973988;
            p242.altitude = (int) -1372069912;
            p242.z = (float)2.6006818E38F;
            p242.time_usec_SET((ulong)3242999786573484496L, PH) ;
            p242.q_SET(new float[] {2.0185875E38F, -1.9232951E38F, -2.2485846E38F, -1.6451103E38F}, 0) ;
            p242.approach_z = (float) -2.4174683E38F;
            p242.latitude = (int) -874668999;
            p242.y = (float) -2.7205976E38F;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int)403403404);
                Debug.Assert(pack.altitude == (int) -979409191);
                Debug.Assert(pack.target_system == (byte)(byte)54);
                Debug.Assert(pack.approach_x == (float) -9.631347E37F);
                Debug.Assert(pack.approach_y == (float)1.085576E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.2125491E38F, 2.691965E38F, -3.3417251E38F, 2.5256074E38F}));
                Debug.Assert(pack.y == (float) -1.1338738E38F);
                Debug.Assert(pack.z == (float)3.2602767E38F);
                Debug.Assert(pack.approach_z == (float) -8.4287346E37F);
                Debug.Assert(pack.x == (float)3.2536646E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)6762608791009525254L);
                Debug.Assert(pack.longitude == (int)590588757);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.approach_x = (float) -9.631347E37F;
            p243.x = (float)3.2536646E38F;
            p243.approach_y = (float)1.085576E38F;
            p243.approach_z = (float) -8.4287346E37F;
            p243.time_usec_SET((ulong)6762608791009525254L, PH) ;
            p243.altitude = (int) -979409191;
            p243.target_system = (byte)(byte)54;
            p243.latitude = (int)403403404;
            p243.y = (float) -1.1338738E38F;
            p243.q_SET(new float[] {3.2125491E38F, 2.691965E38F, -3.3417251E38F, 2.5256074E38F}, 0) ;
            p243.z = (float)3.2602767E38F;
            p243.longitude = (int)590588757;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.interval_us == (int) -587214130);
                Debug.Assert(pack.message_id == (ushort)(ushort)6899);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int) -587214130;
            p244.message_id = (ushort)(ushort)6899;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_type == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
                Debug.Assert(pack.ICAO_address == (uint)567860553U);
                Debug.Assert(pack.tslc == (byte)(byte)233);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)50009);
                Debug.Assert(pack.heading == (ushort)(ushort)57188);
                Debug.Assert(pack.emitter_type == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HIGHLY_MANUV);
                Debug.Assert(pack.lat == (int)1957605057);
                Debug.Assert(pack.callsign_LEN(ph) == 1);
                Debug.Assert(pack.callsign_TRY(ph).Equals("C"));
                Debug.Assert(pack.lon == (int) -1982740998);
                Debug.Assert(pack.ver_velocity == (short)(short) -27888);
                Debug.Assert(pack.flags == (ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                                            ADSB_FLAGS.ADSB_FLAGS_SIMULATED));
                Debug.Assert(pack.altitude == (int)1129812634);
                Debug.Assert(pack.squawk == (ushort)(ushort)19516);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.squawk = (ushort)(ushort)19516;
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HIGHLY_MANUV;
            p246.hor_velocity = (ushort)(ushort)50009;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.lon = (int) -1982740998;
            p246.altitude = (int)1129812634;
            p246.lat = (int)1957605057;
            p246.ICAO_address = (uint)567860553U;
            p246.heading = (ushort)(ushort)57188;
            p246.ver_velocity = (short)(short) -27888;
            p246.tslc = (byte)(byte)233;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                          ADSB_FLAGS.ADSB_FLAGS_SIMULATED);
            p246.callsign_SET("C", PH) ;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.src_ == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
                Debug.Assert(pack.altitude_minimum_delta == (float) -2.7416875E38F);
                Debug.Assert(pack.time_to_minimum_delta == (float) -9.933773E37F);
                Debug.Assert(pack.horizontal_minimum_delta == (float) -3.3577836E37F);
                Debug.Assert(pack.threat_level == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
                Debug.Assert(pack.action == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL);
                Debug.Assert(pack.id == (uint)2858099071U);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL;
            p247.id = (uint)2858099071U;
            p247.altitude_minimum_delta = (float) -2.7416875E38F;
            p247.time_to_minimum_delta = (float) -9.933773E37F;
            p247.threat_level = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH;
            p247.horizontal_minimum_delta = (float) -3.3577836E37F;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)144);
                Debug.Assert(pack.target_network == (byte)(byte)75);
                Debug.Assert(pack.target_component == (byte)(byte)83);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)29, (byte)6, (byte)200, (byte)82, (byte)176, (byte)84, (byte)108, (byte)32, (byte)126, (byte)114, (byte)2, (byte)14, (byte)49, (byte)44, (byte)36, (byte)167, (byte)134, (byte)132, (byte)243, (byte)58, (byte)163, (byte)155, (byte)39, (byte)205, (byte)177, (byte)66, (byte)46, (byte)241, (byte)245, (byte)173, (byte)178, (byte)111, (byte)69, (byte)199, (byte)134, (byte)19, (byte)146, (byte)60, (byte)110, (byte)94, (byte)94, (byte)179, (byte)75, (byte)129, (byte)177, (byte)110, (byte)41, (byte)136, (byte)243, (byte)3, (byte)137, (byte)160, (byte)81, (byte)227, (byte)122, (byte)186, (byte)149, (byte)230, (byte)87, (byte)18, (byte)144, (byte)179, (byte)144, (byte)20, (byte)207, (byte)93, (byte)2, (byte)192, (byte)18, (byte)33, (byte)119, (byte)78, (byte)219, (byte)193, (byte)69, (byte)64, (byte)33, (byte)243, (byte)181, (byte)21, (byte)88, (byte)173, (byte)217, (byte)245, (byte)177, (byte)174, (byte)132, (byte)99, (byte)12, (byte)77, (byte)51, (byte)159, (byte)21, (byte)45, (byte)10, (byte)77, (byte)153, (byte)234, (byte)198, (byte)160, (byte)105, (byte)198, (byte)216, (byte)105, (byte)56, (byte)43, (byte)16, (byte)187, (byte)190, (byte)87, (byte)199, (byte)191, (byte)151, (byte)150, (byte)172, (byte)116, (byte)170, (byte)83, (byte)7, (byte)60, (byte)110, (byte)252, (byte)116, (byte)89, (byte)146, (byte)4, (byte)165, (byte)209, (byte)176, (byte)146, (byte)82, (byte)180, (byte)135, (byte)110, (byte)178, (byte)91, (byte)11, (byte)157, (byte)241, (byte)203, (byte)124, (byte)130, (byte)198, (byte)17, (byte)107, (byte)49, (byte)2, (byte)64, (byte)135, (byte)249, (byte)51, (byte)219, (byte)234, (byte)158, (byte)211, (byte)160, (byte)10, (byte)149, (byte)246, (byte)206, (byte)208, (byte)34, (byte)224, (byte)181, (byte)41, (byte)61, (byte)9, (byte)127, (byte)92, (byte)79, (byte)97, (byte)35, (byte)51, (byte)246, (byte)210, (byte)195, (byte)227, (byte)205, (byte)42, (byte)172, (byte)18, (byte)249, (byte)238, (byte)240, (byte)145, (byte)234, (byte)64, (byte)51, (byte)41, (byte)13, (byte)48, (byte)122, (byte)49, (byte)161, (byte)138, (byte)32, (byte)169, (byte)183, (byte)131, (byte)209, (byte)63, (byte)37, (byte)93, (byte)251, (byte)73, (byte)45, (byte)94, (byte)85, (byte)75, (byte)89, (byte)134, (byte)32, (byte)177, (byte)231, (byte)14, (byte)240, (byte)245, (byte)82, (byte)59, (byte)165, (byte)85, (byte)39, (byte)53, (byte)180, (byte)140, (byte)221, (byte)60, (byte)160, (byte)156, (byte)66, (byte)77, (byte)213, (byte)245, (byte)214, (byte)175, (byte)84, (byte)128, (byte)125, (byte)213, (byte)225, (byte)94, (byte)164, (byte)176, (byte)241, (byte)79, (byte)116, (byte)119, (byte)219, (byte)168}));
                Debug.Assert(pack.message_type == (ushort)(ushort)13110);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.payload_SET(new byte[] {(byte)29, (byte)6, (byte)200, (byte)82, (byte)176, (byte)84, (byte)108, (byte)32, (byte)126, (byte)114, (byte)2, (byte)14, (byte)49, (byte)44, (byte)36, (byte)167, (byte)134, (byte)132, (byte)243, (byte)58, (byte)163, (byte)155, (byte)39, (byte)205, (byte)177, (byte)66, (byte)46, (byte)241, (byte)245, (byte)173, (byte)178, (byte)111, (byte)69, (byte)199, (byte)134, (byte)19, (byte)146, (byte)60, (byte)110, (byte)94, (byte)94, (byte)179, (byte)75, (byte)129, (byte)177, (byte)110, (byte)41, (byte)136, (byte)243, (byte)3, (byte)137, (byte)160, (byte)81, (byte)227, (byte)122, (byte)186, (byte)149, (byte)230, (byte)87, (byte)18, (byte)144, (byte)179, (byte)144, (byte)20, (byte)207, (byte)93, (byte)2, (byte)192, (byte)18, (byte)33, (byte)119, (byte)78, (byte)219, (byte)193, (byte)69, (byte)64, (byte)33, (byte)243, (byte)181, (byte)21, (byte)88, (byte)173, (byte)217, (byte)245, (byte)177, (byte)174, (byte)132, (byte)99, (byte)12, (byte)77, (byte)51, (byte)159, (byte)21, (byte)45, (byte)10, (byte)77, (byte)153, (byte)234, (byte)198, (byte)160, (byte)105, (byte)198, (byte)216, (byte)105, (byte)56, (byte)43, (byte)16, (byte)187, (byte)190, (byte)87, (byte)199, (byte)191, (byte)151, (byte)150, (byte)172, (byte)116, (byte)170, (byte)83, (byte)7, (byte)60, (byte)110, (byte)252, (byte)116, (byte)89, (byte)146, (byte)4, (byte)165, (byte)209, (byte)176, (byte)146, (byte)82, (byte)180, (byte)135, (byte)110, (byte)178, (byte)91, (byte)11, (byte)157, (byte)241, (byte)203, (byte)124, (byte)130, (byte)198, (byte)17, (byte)107, (byte)49, (byte)2, (byte)64, (byte)135, (byte)249, (byte)51, (byte)219, (byte)234, (byte)158, (byte)211, (byte)160, (byte)10, (byte)149, (byte)246, (byte)206, (byte)208, (byte)34, (byte)224, (byte)181, (byte)41, (byte)61, (byte)9, (byte)127, (byte)92, (byte)79, (byte)97, (byte)35, (byte)51, (byte)246, (byte)210, (byte)195, (byte)227, (byte)205, (byte)42, (byte)172, (byte)18, (byte)249, (byte)238, (byte)240, (byte)145, (byte)234, (byte)64, (byte)51, (byte)41, (byte)13, (byte)48, (byte)122, (byte)49, (byte)161, (byte)138, (byte)32, (byte)169, (byte)183, (byte)131, (byte)209, (byte)63, (byte)37, (byte)93, (byte)251, (byte)73, (byte)45, (byte)94, (byte)85, (byte)75, (byte)89, (byte)134, (byte)32, (byte)177, (byte)231, (byte)14, (byte)240, (byte)245, (byte)82, (byte)59, (byte)165, (byte)85, (byte)39, (byte)53, (byte)180, (byte)140, (byte)221, (byte)60, (byte)160, (byte)156, (byte)66, (byte)77, (byte)213, (byte)245, (byte)214, (byte)175, (byte)84, (byte)128, (byte)125, (byte)213, (byte)225, (byte)94, (byte)164, (byte)176, (byte)241, (byte)79, (byte)116, (byte)119, (byte)219, (byte)168}, 0) ;
            p248.target_network = (byte)(byte)75;
            p248.target_component = (byte)(byte)83;
            p248.target_system = (byte)(byte)144;
            p248.message_type = (ushort)(ushort)13110;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)241);
                Debug.Assert(pack.address == (ushort)(ushort)56991);
                Debug.Assert(pack.ver == (byte)(byte)241);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte) - 1, (sbyte)102, (sbyte) - 114, (sbyte) - 76, (sbyte) - 94, (sbyte)120, (sbyte)15, (sbyte)70, (sbyte)56, (sbyte) - 9, (sbyte) - 6, (sbyte)47, (sbyte)54, (sbyte)101, (sbyte)3, (sbyte) - 1, (sbyte)4, (sbyte) - 4, (sbyte) - 49, (sbyte)48, (sbyte) - 58, (sbyte) - 115, (sbyte)25, (sbyte) - 32, (sbyte) - 14, (sbyte) - 65, (sbyte)38, (sbyte) - 38, (sbyte)26, (sbyte) - 88, (sbyte)79, (sbyte) - 9}));
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.ver = (byte)(byte)241;
            p249.address = (ushort)(ushort)56991;
            p249.type = (byte)(byte)241;
            p249.value_SET(new sbyte[] {(sbyte) - 1, (sbyte)102, (sbyte) - 114, (sbyte) - 76, (sbyte) - 94, (sbyte)120, (sbyte)15, (sbyte)70, (sbyte)56, (sbyte) - 9, (sbyte) - 6, (sbyte)47, (sbyte)54, (sbyte)101, (sbyte)3, (sbyte) - 1, (sbyte)4, (sbyte) - 4, (sbyte) - 49, (sbyte)48, (sbyte) - 58, (sbyte) - 115, (sbyte)25, (sbyte) - 32, (sbyte) - 14, (sbyte) - 65, (sbyte)38, (sbyte) - 38, (sbyte)26, (sbyte) - 88, (sbyte)79, (sbyte) - 9}, 0) ;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)1.0345801E38F);
                Debug.Assert(pack.y == (float) -1.3215302E38F);
                Debug.Assert(pack.time_usec == (ulong)7898867505024041397L);
                Debug.Assert(pack.x == (float)2.8749845E38F);
                Debug.Assert(pack.name_LEN(ph) == 2);
                Debug.Assert(pack.name_TRY(ph).Equals("ot"));
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.y = (float) -1.3215302E38F;
            p250.z = (float)1.0345801E38F;
            p250.time_usec = (ulong)7898867505024041397L;
            p250.x = (float)2.8749845E38F;
            p250.name_SET("ot", PH) ;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float)1.6447679E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1563321107U);
                Debug.Assert(pack.name_LEN(ph) == 7);
                Debug.Assert(pack.name_TRY(ph).Equals("nrzxwub"));
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)1563321107U;
            p251.name_SET("nrzxwub", PH) ;
            p251.value = (float)1.6447679E38F;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1968852788U);
                Debug.Assert(pack.name_LEN(ph) == 6);
                Debug.Assert(pack.name_TRY(ph).Equals("lhaxqm"));
                Debug.Assert(pack.value == (int) -2050569627);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)1968852788U;
            p252.value = (int) -2050569627;
            p252.name_SET("lhaxqm", PH) ;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.severity == MAV_SEVERITY.MAV_SEVERITY_NOTICE);
                Debug.Assert(pack.text_LEN(ph) == 46);
                Debug.Assert(pack.text_TRY(ph).Equals("fobRlsoXdhhOwftlxlwQbcvuvqttnsyuysnexjygdhpgTd"));
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("fobRlsoXdhhOwftlxlwQbcvuvqttnsyuysnexjygdhpgTd", PH) ;
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_NOTICE;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2949398342U);
                Debug.Assert(pack.value == (float)2.5750294E38F);
                Debug.Assert(pack.ind == (byte)(byte)172);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)2949398342U;
            p254.ind = (byte)(byte)172;
            p254.value = (float)2.5750294E38F;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)57);
                Debug.Assert(pack.target_system == (byte)(byte)56);
                Debug.Assert(pack.initial_timestamp == (ulong)4576568667108254895L);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)221, (byte)76, (byte)105, (byte)116, (byte)158, (byte)196, (byte)121, (byte)24, (byte)62, (byte)210, (byte)59, (byte)240, (byte)16, (byte)200, (byte)184, (byte)148, (byte)131, (byte)116, (byte)233, (byte)14, (byte)164, (byte)245, (byte)67, (byte)53, (byte)42, (byte)139, (byte)41, (byte)20, (byte)173, (byte)14, (byte)59, (byte)69}));
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.initial_timestamp = (ulong)4576568667108254895L;
            p256.target_system = (byte)(byte)56;
            p256.secret_key_SET(new byte[] {(byte)221, (byte)76, (byte)105, (byte)116, (byte)158, (byte)196, (byte)121, (byte)24, (byte)62, (byte)210, (byte)59, (byte)240, (byte)16, (byte)200, (byte)184, (byte)148, (byte)131, (byte)116, (byte)233, (byte)14, (byte)164, (byte)245, (byte)67, (byte)53, (byte)42, (byte)139, (byte)41, (byte)20, (byte)173, (byte)14, (byte)59, (byte)69}, 0) ;
            p256.target_component = (byte)(byte)57;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2037828523U);
                Debug.Assert(pack.last_change_ms == (uint)3534786475U);
                Debug.Assert(pack.state == (byte)(byte)123);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.last_change_ms = (uint)3534786475U;
            p257.time_boot_ms = (uint)2037828523U;
            p257.state = (byte)(byte)123;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)139);
                Debug.Assert(pack.target_system == (byte)(byte)147);
                Debug.Assert(pack.tune_LEN(ph) == 30);
                Debug.Assert(pack.tune_TRY(ph).Equals("PnpdEnpblupjrxrwuZxdUyfbymgNiY"));
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)147;
            p258.tune_SET("PnpdEnpblupjrxrwuZxdUyfbymgNiY", PH) ;
            p258.target_component = (byte)(byte)139;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_size_h == (float) -9.158567E37F);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)105, (byte)238, (byte)231, (byte)15, (byte)71, (byte)29, (byte)73, (byte)163, (byte)195, (byte)222, (byte)85, (byte)191, (byte)246, (byte)77, (byte)135, (byte)148, (byte)69, (byte)191, (byte)8, (byte)152, (byte)162, (byte)253, (byte)231, (byte)182, (byte)142, (byte)139, (byte)180, (byte)159, (byte)108, (byte)122, (byte)68, (byte)250}));
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)39605);
                Debug.Assert(pack.sensor_size_v == (float) -3.0591293E38F);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 50);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("ngottsvhsidmrbylmWmhjbrUuspXkutepthspjbadpGlvfuZxe"));
                Debug.Assert(pack.time_boot_ms == (uint)2975857248U);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE));
                Debug.Assert(pack.resolution_h == (ushort)(ushort)58799);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)10992);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)233, (byte)35, (byte)142, (byte)128, (byte)200, (byte)248, (byte)202, (byte)94, (byte)111, (byte)138, (byte)34, (byte)120, (byte)122, (byte)197, (byte)58, (byte)117, (byte)22, (byte)79, (byte)12, (byte)84, (byte)78, (byte)59, (byte)131, (byte)166, (byte)69, (byte)221, (byte)87, (byte)33, (byte)51, (byte)226, (byte)172, (byte)35}));
                Debug.Assert(pack.lens_id == (byte)(byte)100);
                Debug.Assert(pack.focal_length == (float) -1.9645804E38F);
                Debug.Assert(pack.firmware_version == (uint)3017067782U);
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.sensor_size_v = (float) -3.0591293E38F;
            p259.vendor_name_SET(new byte[] {(byte)105, (byte)238, (byte)231, (byte)15, (byte)71, (byte)29, (byte)73, (byte)163, (byte)195, (byte)222, (byte)85, (byte)191, (byte)246, (byte)77, (byte)135, (byte)148, (byte)69, (byte)191, (byte)8, (byte)152, (byte)162, (byte)253, (byte)231, (byte)182, (byte)142, (byte)139, (byte)180, (byte)159, (byte)108, (byte)122, (byte)68, (byte)250}, 0) ;
            p259.lens_id = (byte)(byte)100;
            p259.cam_definition_uri_SET("ngottsvhsidmrbylmWmhjbrUuspXkutepthspjbadpGlvfuZxe", PH) ;
            p259.firmware_version = (uint)3017067782U;
            p259.cam_definition_version = (ushort)(ushort)39605;
            p259.resolution_v = (ushort)(ushort)10992;
            p259.model_name_SET(new byte[] {(byte)233, (byte)35, (byte)142, (byte)128, (byte)200, (byte)248, (byte)202, (byte)94, (byte)111, (byte)138, (byte)34, (byte)120, (byte)122, (byte)197, (byte)58, (byte)117, (byte)22, (byte)79, (byte)12, (byte)84, (byte)78, (byte)59, (byte)131, (byte)166, (byte)69, (byte)221, (byte)87, (byte)33, (byte)51, (byte)226, (byte)172, (byte)35}, 0) ;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
            p259.sensor_size_h = (float) -9.158567E37F;
            p259.focal_length = (float) -1.9645804E38F;
            p259.time_boot_ms = (uint)2975857248U;
            p259.resolution_h = (ushort)(ushort)58799;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == CAMERA_MODE.CAMERA_MODE_IMAGE);
                Debug.Assert(pack.time_boot_ms == (uint)1509727804U);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)1509727804U;
            p260.mode_id = CAMERA_MODE.CAMERA_MODE_IMAGE;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.status == (byte)(byte)26);
                Debug.Assert(pack.storage_count == (byte)(byte)87);
                Debug.Assert(pack.used_capacity == (float) -7.3412176E37F);
                Debug.Assert(pack.total_capacity == (float)6.1973334E36F);
                Debug.Assert(pack.time_boot_ms == (uint)1166188037U);
                Debug.Assert(pack.read_speed == (float) -3.3742141E38F);
                Debug.Assert(pack.storage_id == (byte)(byte)186);
                Debug.Assert(pack.available_capacity == (float) -1.125568E38F);
                Debug.Assert(pack.write_speed == (float) -5.084569E37F);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.write_speed = (float) -5.084569E37F;
            p261.read_speed = (float) -3.3742141E38F;
            p261.used_capacity = (float) -7.3412176E37F;
            p261.status = (byte)(byte)26;
            p261.storage_id = (byte)(byte)186;
            p261.available_capacity = (float) -1.125568E38F;
            p261.total_capacity = (float)6.1973334E36F;
            p261.storage_count = (byte)(byte)87;
            p261.time_boot_ms = (uint)1166188037U;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.image_interval == (float)2.114866E38F);
                Debug.Assert(pack.image_status == (byte)(byte)52);
                Debug.Assert(pack.available_capacity == (float)8.366864E37F);
                Debug.Assert(pack.video_status == (byte)(byte)226);
                Debug.Assert(pack.recording_time_ms == (uint)750459239U);
                Debug.Assert(pack.time_boot_ms == (uint)1612952524U);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)1612952524U;
            p262.image_interval = (float)2.114866E38F;
            p262.image_status = (byte)(byte)52;
            p262.recording_time_ms = (uint)750459239U;
            p262.video_status = (byte)(byte)226;
            p262.available_capacity = (float)8.366864E37F;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)681503048);
                Debug.Assert(pack.relative_alt == (int) -793146254);
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.3915838E38F, -2.367993E37F, 6.374487E37F, 6.9777224E37F}));
                Debug.Assert(pack.image_index == (int) -1779307828);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)65);
                Debug.Assert(pack.file_url_LEN(ph) == 115);
                Debug.Assert(pack.file_url_TRY(ph).Equals("rbbxavuddEdMnoEatFlgjghuvqkqJhzmwmlGEbukecyubwddlaeghulhbfnDvqxjKdjddfyhTurewXLkiuaoekwjdjRnrmpqestQgSnmovdQscqphGl"));
                Debug.Assert(pack.time_boot_ms == (uint)1197268223U);
                Debug.Assert(pack.lat == (int)105292322);
                Debug.Assert(pack.time_utc == (ulong)5705572237926914901L);
                Debug.Assert(pack.camera_id == (byte)(byte)159);
                Debug.Assert(pack.alt == (int) -2144160583);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.image_index = (int) -1779307828;
            p263.file_url_SET("rbbxavuddEdMnoEatFlgjghuvqkqJhzmwmlGEbukecyubwddlaeghulhbfnDvqxjKdjddfyhTurewXLkiuaoekwjdjRnrmpqestQgSnmovdQscqphGl", PH) ;
            p263.camera_id = (byte)(byte)159;
            p263.alt = (int) -2144160583;
            p263.time_utc = (ulong)5705572237926914901L;
            p263.lat = (int)105292322;
            p263.time_boot_ms = (uint)1197268223U;
            p263.lon = (int)681503048;
            p263.relative_alt = (int) -793146254;
            p263.capture_result = (sbyte)(sbyte)65;
            p263.q_SET(new float[] {3.3915838E38F, -2.367993E37F, 6.374487E37F, 6.9777224E37F}, 0) ;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flight_uuid == (ulong)7942569804874420420L);
                Debug.Assert(pack.time_boot_ms == (uint)81301516U);
                Debug.Assert(pack.arming_time_utc == (ulong)5070707838395992508L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)8685911647836737823L);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.flight_uuid = (ulong)7942569804874420420L;
            p264.time_boot_ms = (uint)81301516U;
            p264.arming_time_utc = (ulong)5070707838395992508L;
            p264.takeoff_time_utc = (ulong)8685911647836737823L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2359032226U);
                Debug.Assert(pack.pitch == (float)1.3656759E37F);
                Debug.Assert(pack.yaw == (float)2.5057177E36F);
                Debug.Assert(pack.roll == (float) -8.0358146E37F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.roll = (float) -8.0358146E37F;
            p265.yaw = (float)2.5057177E36F;
            p265.time_boot_ms = (uint)2359032226U;
            p265.pitch = (float)1.3656759E37F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)56);
                Debug.Assert(pack.first_message_offset == (byte)(byte)22);
                Debug.Assert(pack.target_component == (byte)(byte)203);
                Debug.Assert(pack.length == (byte)(byte)12);
                Debug.Assert(pack.sequence == (ushort)(ushort)57185);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)34, (byte)248, (byte)230, (byte)130, (byte)139, (byte)100, (byte)12, (byte)105, (byte)75, (byte)76, (byte)250, (byte)208, (byte)161, (byte)105, (byte)48, (byte)49, (byte)144, (byte)88, (byte)222, (byte)76, (byte)249, (byte)33, (byte)42, (byte)109, (byte)141, (byte)171, (byte)66, (byte)124, (byte)69, (byte)75, (byte)6, (byte)13, (byte)60, (byte)8, (byte)4, (byte)52, (byte)213, (byte)52, (byte)68, (byte)32, (byte)18, (byte)254, (byte)24, (byte)177, (byte)165, (byte)86, (byte)175, (byte)204, (byte)218, (byte)0, (byte)36, (byte)97, (byte)54, (byte)84, (byte)207, (byte)156, (byte)46, (byte)145, (byte)182, (byte)45, (byte)115, (byte)130, (byte)94, (byte)112, (byte)75, (byte)48, (byte)65, (byte)7, (byte)9, (byte)72, (byte)122, (byte)172, (byte)228, (byte)214, (byte)246, (byte)99, (byte)216, (byte)161, (byte)54, (byte)65, (byte)140, (byte)197, (byte)23, (byte)238, (byte)39, (byte)253, (byte)109, (byte)236, (byte)202, (byte)34, (byte)11, (byte)74, (byte)102, (byte)1, (byte)207, (byte)193, (byte)36, (byte)130, (byte)246, (byte)97, (byte)231, (byte)115, (byte)88, (byte)7, (byte)111, (byte)150, (byte)149, (byte)37, (byte)251, (byte)144, (byte)206, (byte)171, (byte)55, (byte)170, (byte)250, (byte)97, (byte)133, (byte)76, (byte)169, (byte)52, (byte)146, (byte)98, (byte)182, (byte)244, (byte)70, (byte)96, (byte)42, (byte)134, (byte)167, (byte)104, (byte)252, (byte)61, (byte)103, (byte)129, (byte)25, (byte)78, (byte)26, (byte)178, (byte)68, (byte)221, (byte)180, (byte)92, (byte)109, (byte)171, (byte)158, (byte)10, (byte)62, (byte)188, (byte)156, (byte)194, (byte)229, (byte)179, (byte)188, (byte)164, (byte)219, (byte)117, (byte)122, (byte)114, (byte)42, (byte)217, (byte)85, (byte)29, (byte)90, (byte)244, (byte)196, (byte)159, (byte)226, (byte)181, (byte)250, (byte)250, (byte)229, (byte)20, (byte)14, (byte)38, (byte)207, (byte)155, (byte)238, (byte)104, (byte)194, (byte)149, (byte)95, (byte)227, (byte)184, (byte)15, (byte)190, (byte)85, (byte)178, (byte)142, (byte)147, (byte)44, (byte)234, (byte)200, (byte)130, (byte)140, (byte)170, (byte)128, (byte)183, (byte)137, (byte)133, (byte)45, (byte)247, (byte)240, (byte)33, (byte)37, (byte)180, (byte)171, (byte)98, (byte)149, (byte)103, (byte)175, (byte)209, (byte)54, (byte)213, (byte)153, (byte)8, (byte)81, (byte)205, (byte)33, (byte)143, (byte)136, (byte)182, (byte)192, (byte)152, (byte)177, (byte)55, (byte)132, (byte)123, (byte)88, (byte)41, (byte)31, (byte)236, (byte)100, (byte)97, (byte)115, (byte)107, (byte)196, (byte)115, (byte)31, (byte)55, (byte)69, (byte)21, (byte)16, (byte)116, (byte)142, (byte)53, (byte)207, (byte)121, (byte)254, (byte)95}));
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.first_message_offset = (byte)(byte)22;
            p266.target_component = (byte)(byte)203;
            p266.sequence = (ushort)(ushort)57185;
            p266.data__SET(new byte[] {(byte)34, (byte)248, (byte)230, (byte)130, (byte)139, (byte)100, (byte)12, (byte)105, (byte)75, (byte)76, (byte)250, (byte)208, (byte)161, (byte)105, (byte)48, (byte)49, (byte)144, (byte)88, (byte)222, (byte)76, (byte)249, (byte)33, (byte)42, (byte)109, (byte)141, (byte)171, (byte)66, (byte)124, (byte)69, (byte)75, (byte)6, (byte)13, (byte)60, (byte)8, (byte)4, (byte)52, (byte)213, (byte)52, (byte)68, (byte)32, (byte)18, (byte)254, (byte)24, (byte)177, (byte)165, (byte)86, (byte)175, (byte)204, (byte)218, (byte)0, (byte)36, (byte)97, (byte)54, (byte)84, (byte)207, (byte)156, (byte)46, (byte)145, (byte)182, (byte)45, (byte)115, (byte)130, (byte)94, (byte)112, (byte)75, (byte)48, (byte)65, (byte)7, (byte)9, (byte)72, (byte)122, (byte)172, (byte)228, (byte)214, (byte)246, (byte)99, (byte)216, (byte)161, (byte)54, (byte)65, (byte)140, (byte)197, (byte)23, (byte)238, (byte)39, (byte)253, (byte)109, (byte)236, (byte)202, (byte)34, (byte)11, (byte)74, (byte)102, (byte)1, (byte)207, (byte)193, (byte)36, (byte)130, (byte)246, (byte)97, (byte)231, (byte)115, (byte)88, (byte)7, (byte)111, (byte)150, (byte)149, (byte)37, (byte)251, (byte)144, (byte)206, (byte)171, (byte)55, (byte)170, (byte)250, (byte)97, (byte)133, (byte)76, (byte)169, (byte)52, (byte)146, (byte)98, (byte)182, (byte)244, (byte)70, (byte)96, (byte)42, (byte)134, (byte)167, (byte)104, (byte)252, (byte)61, (byte)103, (byte)129, (byte)25, (byte)78, (byte)26, (byte)178, (byte)68, (byte)221, (byte)180, (byte)92, (byte)109, (byte)171, (byte)158, (byte)10, (byte)62, (byte)188, (byte)156, (byte)194, (byte)229, (byte)179, (byte)188, (byte)164, (byte)219, (byte)117, (byte)122, (byte)114, (byte)42, (byte)217, (byte)85, (byte)29, (byte)90, (byte)244, (byte)196, (byte)159, (byte)226, (byte)181, (byte)250, (byte)250, (byte)229, (byte)20, (byte)14, (byte)38, (byte)207, (byte)155, (byte)238, (byte)104, (byte)194, (byte)149, (byte)95, (byte)227, (byte)184, (byte)15, (byte)190, (byte)85, (byte)178, (byte)142, (byte)147, (byte)44, (byte)234, (byte)200, (byte)130, (byte)140, (byte)170, (byte)128, (byte)183, (byte)137, (byte)133, (byte)45, (byte)247, (byte)240, (byte)33, (byte)37, (byte)180, (byte)171, (byte)98, (byte)149, (byte)103, (byte)175, (byte)209, (byte)54, (byte)213, (byte)153, (byte)8, (byte)81, (byte)205, (byte)33, (byte)143, (byte)136, (byte)182, (byte)192, (byte)152, (byte)177, (byte)55, (byte)132, (byte)123, (byte)88, (byte)41, (byte)31, (byte)236, (byte)100, (byte)97, (byte)115, (byte)107, (byte)196, (byte)115, (byte)31, (byte)55, (byte)69, (byte)21, (byte)16, (byte)116, (byte)142, (byte)53, (byte)207, (byte)121, (byte)254, (byte)95}, 0) ;
            p266.target_system = (byte)(byte)56;
            p266.length = (byte)(byte)12;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)136, (byte)66, (byte)19, (byte)78, (byte)21, (byte)135, (byte)135, (byte)125, (byte)144, (byte)57, (byte)233, (byte)139, (byte)251, (byte)7, (byte)144, (byte)67, (byte)178, (byte)6, (byte)175, (byte)172, (byte)189, (byte)34, (byte)35, (byte)216, (byte)164, (byte)135, (byte)152, (byte)187, (byte)212, (byte)235, (byte)176, (byte)93, (byte)121, (byte)8, (byte)108, (byte)82, (byte)214, (byte)33, (byte)155, (byte)253, (byte)201, (byte)165, (byte)40, (byte)2, (byte)39, (byte)193, (byte)251, (byte)152, (byte)203, (byte)67, (byte)58, (byte)30, (byte)47, (byte)130, (byte)222, (byte)26, (byte)15, (byte)179, (byte)50, (byte)110, (byte)140, (byte)10, (byte)151, (byte)244, (byte)5, (byte)48, (byte)64, (byte)97, (byte)101, (byte)163, (byte)205, (byte)225, (byte)177, (byte)217, (byte)208, (byte)17, (byte)164, (byte)53, (byte)149, (byte)210, (byte)140, (byte)108, (byte)241, (byte)107, (byte)135, (byte)243, (byte)26, (byte)122, (byte)236, (byte)131, (byte)115, (byte)13, (byte)199, (byte)132, (byte)249, (byte)136, (byte)2, (byte)240, (byte)111, (byte)2, (byte)31, (byte)136, (byte)143, (byte)1, (byte)145, (byte)122, (byte)53, (byte)53, (byte)87, (byte)10, (byte)58, (byte)9, (byte)234, (byte)62, (byte)171, (byte)218, (byte)11, (byte)221, (byte)200, (byte)67, (byte)195, (byte)102, (byte)118, (byte)60, (byte)35, (byte)203, (byte)199, (byte)158, (byte)137, (byte)59, (byte)144, (byte)21, (byte)185, (byte)152, (byte)121, (byte)153, (byte)227, (byte)225, (byte)164, (byte)252, (byte)101, (byte)250, (byte)40, (byte)63, (byte)193, (byte)194, (byte)95, (byte)79, (byte)210, (byte)154, (byte)118, (byte)70, (byte)12, (byte)71, (byte)141, (byte)226, (byte)198, (byte)43, (byte)149, (byte)207, (byte)89, (byte)48, (byte)72, (byte)69, (byte)81, (byte)178, (byte)184, (byte)57, (byte)203, (byte)190, (byte)162, (byte)197, (byte)143, (byte)163, (byte)233, (byte)35, (byte)248, (byte)210, (byte)35, (byte)184, (byte)151, (byte)27, (byte)124, (byte)44, (byte)2, (byte)72, (byte)46, (byte)13, (byte)23, (byte)245, (byte)85, (byte)94, (byte)93, (byte)118, (byte)124, (byte)21, (byte)142, (byte)42, (byte)52, (byte)15, (byte)193, (byte)17, (byte)108, (byte)103, (byte)198, (byte)104, (byte)88, (byte)125, (byte)26, (byte)117, (byte)207, (byte)212, (byte)9, (byte)154, (byte)41, (byte)0, (byte)210, (byte)140, (byte)57, (byte)232, (byte)153, (byte)110, (byte)121, (byte)36, (byte)202, (byte)22, (byte)53, (byte)25, (byte)112, (byte)45, (byte)154, (byte)87, (byte)193, (byte)142, (byte)169, (byte)111, (byte)130, (byte)52, (byte)145, (byte)230, (byte)215, (byte)39, (byte)84, (byte)66, (byte)153, (byte)47, (byte)97, (byte)202, (byte)108}));
                Debug.Assert(pack.target_component == (byte)(byte)197);
                Debug.Assert(pack.sequence == (ushort)(ushort)5554);
                Debug.Assert(pack.first_message_offset == (byte)(byte)40);
                Debug.Assert(pack.target_system == (byte)(byte)81);
                Debug.Assert(pack.length == (byte)(byte)150);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.length = (byte)(byte)150;
            p267.target_system = (byte)(byte)81;
            p267.target_component = (byte)(byte)197;
            p267.first_message_offset = (byte)(byte)40;
            p267.sequence = (ushort)(ushort)5554;
            p267.data__SET(new byte[] {(byte)136, (byte)66, (byte)19, (byte)78, (byte)21, (byte)135, (byte)135, (byte)125, (byte)144, (byte)57, (byte)233, (byte)139, (byte)251, (byte)7, (byte)144, (byte)67, (byte)178, (byte)6, (byte)175, (byte)172, (byte)189, (byte)34, (byte)35, (byte)216, (byte)164, (byte)135, (byte)152, (byte)187, (byte)212, (byte)235, (byte)176, (byte)93, (byte)121, (byte)8, (byte)108, (byte)82, (byte)214, (byte)33, (byte)155, (byte)253, (byte)201, (byte)165, (byte)40, (byte)2, (byte)39, (byte)193, (byte)251, (byte)152, (byte)203, (byte)67, (byte)58, (byte)30, (byte)47, (byte)130, (byte)222, (byte)26, (byte)15, (byte)179, (byte)50, (byte)110, (byte)140, (byte)10, (byte)151, (byte)244, (byte)5, (byte)48, (byte)64, (byte)97, (byte)101, (byte)163, (byte)205, (byte)225, (byte)177, (byte)217, (byte)208, (byte)17, (byte)164, (byte)53, (byte)149, (byte)210, (byte)140, (byte)108, (byte)241, (byte)107, (byte)135, (byte)243, (byte)26, (byte)122, (byte)236, (byte)131, (byte)115, (byte)13, (byte)199, (byte)132, (byte)249, (byte)136, (byte)2, (byte)240, (byte)111, (byte)2, (byte)31, (byte)136, (byte)143, (byte)1, (byte)145, (byte)122, (byte)53, (byte)53, (byte)87, (byte)10, (byte)58, (byte)9, (byte)234, (byte)62, (byte)171, (byte)218, (byte)11, (byte)221, (byte)200, (byte)67, (byte)195, (byte)102, (byte)118, (byte)60, (byte)35, (byte)203, (byte)199, (byte)158, (byte)137, (byte)59, (byte)144, (byte)21, (byte)185, (byte)152, (byte)121, (byte)153, (byte)227, (byte)225, (byte)164, (byte)252, (byte)101, (byte)250, (byte)40, (byte)63, (byte)193, (byte)194, (byte)95, (byte)79, (byte)210, (byte)154, (byte)118, (byte)70, (byte)12, (byte)71, (byte)141, (byte)226, (byte)198, (byte)43, (byte)149, (byte)207, (byte)89, (byte)48, (byte)72, (byte)69, (byte)81, (byte)178, (byte)184, (byte)57, (byte)203, (byte)190, (byte)162, (byte)197, (byte)143, (byte)163, (byte)233, (byte)35, (byte)248, (byte)210, (byte)35, (byte)184, (byte)151, (byte)27, (byte)124, (byte)44, (byte)2, (byte)72, (byte)46, (byte)13, (byte)23, (byte)245, (byte)85, (byte)94, (byte)93, (byte)118, (byte)124, (byte)21, (byte)142, (byte)42, (byte)52, (byte)15, (byte)193, (byte)17, (byte)108, (byte)103, (byte)198, (byte)104, (byte)88, (byte)125, (byte)26, (byte)117, (byte)207, (byte)212, (byte)9, (byte)154, (byte)41, (byte)0, (byte)210, (byte)140, (byte)57, (byte)232, (byte)153, (byte)110, (byte)121, (byte)36, (byte)202, (byte)22, (byte)53, (byte)25, (byte)112, (byte)45, (byte)154, (byte)87, (byte)193, (byte)142, (byte)169, (byte)111, (byte)130, (byte)52, (byte)145, (byte)230, (byte)215, (byte)39, (byte)84, (byte)66, (byte)153, (byte)47, (byte)97, (byte)202, (byte)108}, 0) ;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)15);
                Debug.Assert(pack.target_component == (byte)(byte)217);
                Debug.Assert(pack.sequence == (ushort)(ushort)29809);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_component = (byte)(byte)217;
            p268.target_system = (byte)(byte)15;
            p268.sequence = (ushort)(ushort)29809;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.camera_id == (byte)(byte)134);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)12119);
                Debug.Assert(pack.bitrate == (uint)1752788982U);
                Debug.Assert(pack.framerate == (float) -1.482548E38F);
                Debug.Assert(pack.status == (byte)(byte)239);
                Debug.Assert(pack.uri_LEN(ph) == 134);
                Debug.Assert(pack.uri_TRY(ph).Equals("xfbinNkbjsrgkpztuzoaIoqZylucsxewKRrJqvHyjzrgnijDytlwgmzrtrDnzenvaliarydtlmudRcdacdrkBEpedoCsbNircjzsbiJgsvuwiwChyibGihmcoqnbRldrwsqowl"));
                Debug.Assert(pack.rotation == (ushort)(ushort)37886);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)61695);
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.rotation = (ushort)(ushort)37886;
            p269.framerate = (float) -1.482548E38F;
            p269.status = (byte)(byte)239;
            p269.uri_SET("xfbinNkbjsrgkpztuzoaIoqZylucsxewKRrJqvHyjzrgnijDytlwgmzrtrDnzenvaliarydtlmudRcdacdrkBEpedoCsbNircjzsbiJgsvuwiwChyibGihmcoqnbRldrwsqowl", PH) ;
            p269.resolution_h = (ushort)(ushort)61695;
            p269.resolution_v = (ushort)(ushort)12119;
            p269.bitrate = (uint)1752788982U;
            p269.camera_id = (byte)(byte)134;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_h == (ushort)(ushort)30332);
                Debug.Assert(pack.rotation == (ushort)(ushort)27703);
                Debug.Assert(pack.target_component == (byte)(byte)1);
                Debug.Assert(pack.uri_LEN(ph) == 31);
                Debug.Assert(pack.uri_TRY(ph).Equals("kgsgwxflqwbgmepkjqPpdmapmwNqurq"));
                Debug.Assert(pack.framerate == (float)1.0757497E38F);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)35701);
                Debug.Assert(pack.target_system == (byte)(byte)54);
                Debug.Assert(pack.bitrate == (uint)1121768708U);
                Debug.Assert(pack.camera_id == (byte)(byte)29);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.framerate = (float)1.0757497E38F;
            p270.resolution_h = (ushort)(ushort)30332;
            p270.resolution_v = (ushort)(ushort)35701;
            p270.target_system = (byte)(byte)54;
            p270.rotation = (ushort)(ushort)27703;
            p270.camera_id = (byte)(byte)29;
            p270.target_component = (byte)(byte)1;
            p270.bitrate = (uint)1121768708U;
            p270.uri_SET("kgsgwxflqwbgmepkjqPpdmapmwNqurq", PH) ;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 4);
                Debug.Assert(pack.password_TRY(ph).Equals("Hjsx"));
                Debug.Assert(pack.ssid_LEN(ph) == 20);
                Debug.Assert(pack.ssid_TRY(ph).Equals("qqqfernbppbfqmwlsdrl"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.password_SET("Hjsx", PH) ;
            p299.ssid_SET("qqqfernbppbfqmwlsdrl", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)127, (byte)247, (byte)106, (byte)41, (byte)13, (byte)20, (byte)152, (byte)112}));
                Debug.Assert(pack.max_version == (ushort)(ushort)27706);
                Debug.Assert(pack.min_version == (ushort)(ushort)31699);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)56, (byte)8, (byte)228, (byte)55, (byte)196, (byte)25, (byte)254, (byte)190}));
                Debug.Assert(pack.version == (ushort)(ushort)36088);
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.version = (ushort)(ushort)36088;
            p300.max_version = (ushort)(ushort)27706;
            p300.library_version_hash_SET(new byte[] {(byte)56, (byte)8, (byte)228, (byte)55, (byte)196, (byte)25, (byte)254, (byte)190}, 0) ;
            p300.spec_version_hash_SET(new byte[] {(byte)127, (byte)247, (byte)106, (byte)41, (byte)13, (byte)20, (byte)152, (byte)112}, 0) ;
            p300.min_version = (ushort)(ushort)31699;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uptime_sec == (uint)3472810960U);
                Debug.Assert(pack.time_usec == (ulong)4620573012797512977L);
                Debug.Assert(pack.health == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
                Debug.Assert(pack.sub_mode == (byte)(byte)185);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)45979);
                Debug.Assert(pack.mode == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)4620573012797512977L;
            p310.vendor_specific_status_code = (ushort)(ushort)45979;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL;
            p310.uptime_sec = (uint)3472810960U;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION;
            p310.sub_mode = (byte)(byte)185;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uptime_sec == (uint)2899545380U);
                Debug.Assert(pack.sw_version_major == (byte)(byte)41);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)49, (byte)228, (byte)3, (byte)175, (byte)21, (byte)154, (byte)89, (byte)190, (byte)124, (byte)77, (byte)187, (byte)117, (byte)226, (byte)100, (byte)20, (byte)204}));
                Debug.Assert(pack.hw_version_minor == (byte)(byte)75);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)80);
                Debug.Assert(pack.time_usec == (ulong)693093795025832540L);
                Debug.Assert(pack.name_LEN(ph) == 27);
                Debug.Assert(pack.name_TRY(ph).Equals("oluAtnmrvfdplatbbcupfvbmgds"));
                Debug.Assert(pack.sw_vcs_commit == (uint)3891723882U);
                Debug.Assert(pack.hw_version_major == (byte)(byte)108);
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.uptime_sec = (uint)2899545380U;
            p311.sw_version_major = (byte)(byte)41;
            p311.sw_version_minor = (byte)(byte)80;
            p311.time_usec = (ulong)693093795025832540L;
            p311.hw_unique_id_SET(new byte[] {(byte)49, (byte)228, (byte)3, (byte)175, (byte)21, (byte)154, (byte)89, (byte)190, (byte)124, (byte)77, (byte)187, (byte)117, (byte)226, (byte)100, (byte)20, (byte)204}, 0) ;
            p311.hw_version_minor = (byte)(byte)75;
            p311.name_SET("oluAtnmrvfdplatbbcupfvbmgds", PH) ;
            p311.sw_vcs_commit = (uint)3891723882U;
            p311.hw_version_major = (byte)(byte)108;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)119);
                Debug.Assert(pack.target_system == (byte)(byte)23);
                Debug.Assert(pack.param_id_LEN(ph) == 4);
                Debug.Assert(pack.param_id_TRY(ph).Equals("sxrn"));
                Debug.Assert(pack.param_index == (short)(short) -32309);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_index = (short)(short) -32309;
            p320.target_component = (byte)(byte)119;
            p320.target_system = (byte)(byte)23;
            p320.param_id_SET("sxrn", PH) ;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)74);
                Debug.Assert(pack.target_system == (byte)(byte)159);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)159;
            p321.target_component = (byte)(byte)74;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 36);
                Debug.Assert(pack.param_value_TRY(ph).Equals("vFitDwslfhqkthPcyhyfyxpladbvtiiwsopj"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
                Debug.Assert(pack.param_count == (ushort)(ushort)27492);
                Debug.Assert(pack.param_index == (ushort)(ushort)13133);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("zzneB"));
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_index = (ushort)(ushort)13133;
            p322.param_count = (ushort)(ushort)27492;
            p322.param_id_SET("zzneB", PH) ;
            p322.param_value_SET("vFitDwslfhqkthPcyhyfyxpladbvtiiwsopj", PH) ;
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 7);
                Debug.Assert(pack.param_id_TRY(ph).Equals("rmtenqt"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
                Debug.Assert(pack.param_value_LEN(ph) == 17);
                Debug.Assert(pack.param_value_TRY(ph).Equals("jwozqveujoAxesgon"));
                Debug.Assert(pack.target_system == (byte)(byte)155);
                Debug.Assert(pack.target_component == (byte)(byte)59);
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_value_SET("jwozqveujoAxesgon", PH) ;
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8;
            p323.param_id_SET("rmtenqt", PH) ;
            p323.target_system = (byte)(byte)155;
            p323.target_component = (byte)(byte)59;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
                Debug.Assert(pack.param_value_LEN(ph) == 13);
                Debug.Assert(pack.param_value_TRY(ph).Equals("wdpocvqmmkqba"));
                Debug.Assert(pack.param_result == PARAM_ACK.PARAM_ACK_FAILED);
                Debug.Assert(pack.param_id_LEN(ph) == 11);
                Debug.Assert(pack.param_id_TRY(ph).Equals("rmayvffulzn"));
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_value_SET("wdpocvqmmkqba", PH) ;
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8;
            p324.param_id_SET("rmayvffulzn", PH) ;
            p324.param_result = PARAM_ACK.PARAM_ACK_FAILED;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_distance == (ushort)(ushort)63888);
                Debug.Assert(pack.max_distance == (ushort)(ushort)5766);
                Debug.Assert(pack.sensor_type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
                Debug.Assert(pack.increment == (byte)(byte)247);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)804, (ushort)56617, (ushort)18754, (ushort)44028, (ushort)10453, (ushort)57205, (ushort)46482, (ushort)9942, (ushort)62466, (ushort)41676, (ushort)19598, (ushort)47172, (ushort)57120, (ushort)7009, (ushort)23784, (ushort)50506, (ushort)57251, (ushort)23888, (ushort)47809, (ushort)51535, (ushort)28736, (ushort)61643, (ushort)26169, (ushort)446, (ushort)8463, (ushort)6419, (ushort)30404, (ushort)38082, (ushort)46735, (ushort)9976, (ushort)52227, (ushort)25697, (ushort)50440, (ushort)54112, (ushort)27438, (ushort)45394, (ushort)22090, (ushort)44379, (ushort)60934, (ushort)18118, (ushort)50772, (ushort)43669, (ushort)9958, (ushort)39847, (ushort)27898, (ushort)40662, (ushort)40078, (ushort)56240, (ushort)54654, (ushort)65081, (ushort)28454, (ushort)22622, (ushort)50016, (ushort)55839, (ushort)41636, (ushort)1073, (ushort)27120, (ushort)48169, (ushort)54166, (ushort)32172, (ushort)53364, (ushort)56949, (ushort)13669, (ushort)12352, (ushort)55478, (ushort)21578, (ushort)3008, (ushort)3534, (ushort)49856, (ushort)16930, (ushort)61971, (ushort)36316}));
                Debug.Assert(pack.time_usec == (ulong)5167617087540817612L);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.distances_SET(new ushort[] {(ushort)804, (ushort)56617, (ushort)18754, (ushort)44028, (ushort)10453, (ushort)57205, (ushort)46482, (ushort)9942, (ushort)62466, (ushort)41676, (ushort)19598, (ushort)47172, (ushort)57120, (ushort)7009, (ushort)23784, (ushort)50506, (ushort)57251, (ushort)23888, (ushort)47809, (ushort)51535, (ushort)28736, (ushort)61643, (ushort)26169, (ushort)446, (ushort)8463, (ushort)6419, (ushort)30404, (ushort)38082, (ushort)46735, (ushort)9976, (ushort)52227, (ushort)25697, (ushort)50440, (ushort)54112, (ushort)27438, (ushort)45394, (ushort)22090, (ushort)44379, (ushort)60934, (ushort)18118, (ushort)50772, (ushort)43669, (ushort)9958, (ushort)39847, (ushort)27898, (ushort)40662, (ushort)40078, (ushort)56240, (ushort)54654, (ushort)65081, (ushort)28454, (ushort)22622, (ushort)50016, (ushort)55839, (ushort)41636, (ushort)1073, (ushort)27120, (ushort)48169, (ushort)54166, (ushort)32172, (ushort)53364, (ushort)56949, (ushort)13669, (ushort)12352, (ushort)55478, (ushort)21578, (ushort)3008, (ushort)3534, (ushort)49856, (ushort)16930, (ushort)61971, (ushort)36316}, 0) ;
            p330.time_usec = (ulong)5167617087540817612L;
            p330.min_distance = (ushort)(ushort)63888;
            p330.max_distance = (ushort)(ushort)5766;
            p330.increment = (byte)(byte)247;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
        }
    }
}