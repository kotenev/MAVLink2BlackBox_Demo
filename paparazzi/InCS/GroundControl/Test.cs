
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
                    ulong id = id__I(value);
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
                    ulong id = id__h(value);
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
                    ulong id = id__h(value);
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
                    ulong id = id__B(value);
                    BitUtils.set_bits(id, 7, data, 276);
                }
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = id__h(value);
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
                    ulong id = id__h(value);
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
                    ulong id = id__h(value);
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
                    ulong id = id__h(value);
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
                    ulong id = id__h(value);
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
                    ulong id = id__h(value);
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
                    ulong id = id__h(value);
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
                    ulong id = id__B(value);
                    BitUtils.set_bits(id, 7, data, 276);
                }
            }

            public MAV_MISSION_TYPE mission_type //Mission type, see MAV_MISSION_TYPE
            {
                set
                {
                    ulong id = id__h(value);
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
                    ulong id = id__B(value);
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
                Debug.Assert(pack.system_status == MAV_STATE.MAV_STATE_CALIBRATING);
                Debug.Assert(pack.type == MAV_TYPE.MAV_TYPE_SUBMARINE);
                Debug.Assert(pack.mavlink_version == (byte)(byte)169);
                Debug.Assert(pack.autopilot == MAV_AUTOPILOT.MAV_AUTOPILOT_ASLUAV);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
                Debug.Assert(pack.custom_mode == (uint)3951360099U);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.mavlink_version = (byte)(byte)169;
            p0.type = MAV_TYPE.MAV_TYPE_SUBMARINE;
            p0.custom_mode = (uint)3951360099U;
            p0.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
            p0.system_status = MAV_STATE.MAV_STATE_CALIBRATING;
            p0.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_ASLUAV;
            SMP_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY));
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)127);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH));
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)39722);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)42717);
                Debug.Assert(pack.load == (ushort)(ushort)46544);
                Debug.Assert(pack.current_battery == (short)(short)10116);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)26305);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)43993);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)13011);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)24112);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)26541);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.errors_comm = (ushort)(ushort)42717;
            p1.errors_count1 = (ushort)(ushort)24112;
            p1.voltage_battery = (ushort)(ushort)26541;
            p1.battery_remaining = (sbyte)(sbyte)127;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH);
            p1.errors_count3 = (ushort)(ushort)43993;
            p1.drop_rate_comm = (ushort)(ushort)26305;
            p1.errors_count4 = (ushort)(ushort)39722;
            p1.current_battery = (short)(short)10116;
            p1.errors_count2 = (ushort)(ushort)13011;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY);
            p1.load = (ushort)(ushort)46544;
            SMP_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3903027645U);
                Debug.Assert(pack.time_unix_usec == (ulong)8649245236195008331L);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)8649245236195008331L;
            p2.time_boot_ms = (uint)3903027645U;
            SMP_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afx == (float)1.3599155E38F);
                Debug.Assert(pack.yaw_rate == (float)2.4393595E38F);
                Debug.Assert(pack.y == (float) -8.891937E37F);
                Debug.Assert(pack.vz == (float)2.2598237E38F);
                Debug.Assert(pack.afz == (float) -1.4358898E38F);
                Debug.Assert(pack.time_boot_ms == (uint)777722891U);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.vx == (float)7.8477353E37F);
                Debug.Assert(pack.z == (float) -1.0870551E37F);
                Debug.Assert(pack.vy == (float) -1.9426715E36F);
                Debug.Assert(pack.afy == (float) -2.3101502E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)58497);
                Debug.Assert(pack.x == (float) -2.0090157E38F);
                Debug.Assert(pack.yaw == (float)2.3141442E38F);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.vy = (float) -1.9426715E36F;
            p3.type_mask = (ushort)(ushort)58497;
            p3.yaw = (float)2.3141442E38F;
            p3.afz = (float) -1.4358898E38F;
            p3.yaw_rate = (float)2.4393595E38F;
            p3.z = (float) -1.0870551E37F;
            p3.vz = (float)2.2598237E38F;
            p3.time_boot_ms = (uint)777722891U;
            p3.afy = (float) -2.3101502E38F;
            p3.x = (float) -2.0090157E38F;
            p3.y = (float) -8.891937E37F;
            p3.afx = (float)1.3599155E38F;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p3.vx = (float)7.8477353E37F;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)6);
                Debug.Assert(pack.time_usec == (ulong)2130411113093105758L);
                Debug.Assert(pack.seq == (uint)2049890570U);
                Debug.Assert(pack.target_component == (byte)(byte)204);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.seq = (uint)2049890570U;
            p4.target_system = (byte)(byte)6;
            p4.time_usec = (ulong)2130411113093105758L;
            p4.target_component = (byte)(byte)204;
            SMP_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.version == (byte)(byte)46);
                Debug.Assert(pack.target_system == (byte)(byte)29);
                Debug.Assert(pack.control_request == (byte)(byte)2);
                Debug.Assert(pack.passkey_LEN(ph) == 8);
                Debug.Assert(pack.passkey_TRY(ph).Equals("qwbgbTbc"));
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.version = (byte)(byte)46;
            p5.passkey_SET("qwbgbTbc", PH) ;
            p5.control_request = (byte)(byte)2;
            p5.target_system = (byte)(byte)29;
            SMP_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.control_request == (byte)(byte)52);
                Debug.Assert(pack.ack == (byte)(byte)46);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)177);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.gcs_system_id = (byte)(byte)177;
            p6.ack = (byte)(byte)46;
            p6.control_request = (byte)(byte)52;
            SMP_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 32);
                Debug.Assert(pack.key_TRY(ph).Equals("EugdzxyyvqpqcxfojgwvkdamdxduOBuw"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("EugdzxyyvqpqcxfojgwvkdamdxduOBuw", PH) ;
            SMP_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)157);
                Debug.Assert(pack.custom_mode == (uint)3985812268U);
                Debug.Assert(pack.base_mode == MAV_MODE.MAV_MODE_TEST_ARMED);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)157;
            p11.base_mode = MAV_MODE.MAV_MODE_TEST_ARMED;
            p11.custom_mode = (uint)3985812268U;
            SMP_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)154);
                Debug.Assert(pack.param_index == (short)(short) -2444);
                Debug.Assert(pack.target_system == (byte)(byte)17);
                Debug.Assert(pack.param_id_LEN(ph) == 6);
                Debug.Assert(pack.param_id_TRY(ph).Equals("wdxHyw"));
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_system = (byte)(byte)17;
            p20.param_id_SET("wdxHyw", PH) ;
            p20.param_index = (short)(short) -2444;
            p20.target_component = (byte)(byte)154;
            SMP_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)247);
                Debug.Assert(pack.target_component == (byte)(byte)250);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)250;
            p21.target_system = (byte)(byte)247;
            SMP_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_count == (ushort)(ushort)12666);
                Debug.Assert(pack.param_index == (ushort)(ushort)39985);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8);
                Debug.Assert(pack.param_value == (float) -3.0726214E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("r"));
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_id_SET("r", PH) ;
            p22.param_index = (ushort)(ushort)39985;
            p22.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8;
            p22.param_count = (ushort)(ushort)12666;
            p22.param_value = (float) -3.0726214E38F;
            SMP_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)69);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
                Debug.Assert(pack.param_value == (float) -5.31771E37F);
                Debug.Assert(pack.target_component == (byte)(byte)220);
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("h"));
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.param_value = (float) -5.31771E37F;
            p23.param_id_SET("h", PH) ;
            p23.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16;
            p23.target_system = (byte)(byte)69;
            p23.target_component = (byte)(byte)220;
            SMP_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (int) -788025981);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)3829657917U);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -1762413981);
                Debug.Assert(pack.time_usec == (ulong)3202142437165071942L);
                Debug.Assert(pack.vel == (ushort)(ushort)29384);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)1168642672U);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
                Debug.Assert(pack.eph == (ushort)(ushort)34875);
                Debug.Assert(pack.cog == (ushort)(ushort)8176);
                Debug.Assert(pack.satellites_visible == (byte)(byte)82);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)926444481U);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)1449430644U);
                Debug.Assert(pack.lon == (int)1920851227);
                Debug.Assert(pack.lat == (int) -10056673);
                Debug.Assert(pack.epv == (ushort)(ushort)16161);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.lat = (int) -10056673;
            p24.vel = (ushort)(ushort)29384;
            p24.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT;
            p24.eph = (ushort)(ushort)34875;
            p24.hdg_acc_SET((uint)1449430644U, PH) ;
            p24.satellites_visible = (byte)(byte)82;
            p24.cog = (ushort)(ushort)8176;
            p24.alt_ellipsoid_SET((int) -1762413981, PH) ;
            p24.time_usec = (ulong)3202142437165071942L;
            p24.alt = (int) -788025981;
            p24.lon = (int)1920851227;
            p24.epv = (ushort)(ushort)16161;
            p24.vel_acc_SET((uint)926444481U, PH) ;
            p24.v_acc_SET((uint)3829657917U, PH) ;
            p24.h_acc_SET((uint)1168642672U, PH) ;
            SMP_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)83, (byte)211, (byte)17, (byte)5, (byte)19, (byte)230, (byte)59, (byte)78, (byte)87, (byte)153, (byte)46, (byte)171, (byte)125, (byte)17, (byte)225, (byte)180, (byte)234, (byte)30, (byte)192, (byte)176}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)162, (byte)242, (byte)222, (byte)253, (byte)4, (byte)184, (byte)201, (byte)25, (byte)53, (byte)56, (byte)79, (byte)103, (byte)5, (byte)160, (byte)193, (byte)7, (byte)216, (byte)119, (byte)135, (byte)7}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)38, (byte)201, (byte)159, (byte)250, (byte)153, (byte)20, (byte)55, (byte)142, (byte)217, (byte)72, (byte)24, (byte)243, (byte)38, (byte)18, (byte)77, (byte)164, (byte)169, (byte)239, (byte)184, (byte)214}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)145, (byte)176, (byte)23, (byte)4, (byte)121, (byte)238, (byte)22, (byte)205, (byte)129, (byte)199, (byte)149, (byte)196, (byte)34, (byte)144, (byte)137, (byte)104, (byte)242, (byte)227, (byte)130, (byte)127}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)12);
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)239, (byte)104, (byte)174, (byte)97, (byte)141, (byte)77, (byte)42, (byte)72, (byte)67, (byte)231, (byte)110, (byte)60, (byte)13, (byte)87, (byte)15, (byte)10, (byte)79, (byte)16, (byte)243, (byte)37}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_snr_SET(new byte[] {(byte)145, (byte)176, (byte)23, (byte)4, (byte)121, (byte)238, (byte)22, (byte)205, (byte)129, (byte)199, (byte)149, (byte)196, (byte)34, (byte)144, (byte)137, (byte)104, (byte)242, (byte)227, (byte)130, (byte)127}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)83, (byte)211, (byte)17, (byte)5, (byte)19, (byte)230, (byte)59, (byte)78, (byte)87, (byte)153, (byte)46, (byte)171, (byte)125, (byte)17, (byte)225, (byte)180, (byte)234, (byte)30, (byte)192, (byte)176}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)239, (byte)104, (byte)174, (byte)97, (byte)141, (byte)77, (byte)42, (byte)72, (byte)67, (byte)231, (byte)110, (byte)60, (byte)13, (byte)87, (byte)15, (byte)10, (byte)79, (byte)16, (byte)243, (byte)37}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)38, (byte)201, (byte)159, (byte)250, (byte)153, (byte)20, (byte)55, (byte)142, (byte)217, (byte)72, (byte)24, (byte)243, (byte)38, (byte)18, (byte)77, (byte)164, (byte)169, (byte)239, (byte)184, (byte)214}, 0) ;
            p25.satellites_visible = (byte)(byte)12;
            p25.satellite_prn_SET(new byte[] {(byte)162, (byte)242, (byte)222, (byte)253, (byte)4, (byte)184, (byte)201, (byte)25, (byte)53, (byte)56, (byte)79, (byte)103, (byte)5, (byte)160, (byte)193, (byte)7, (byte)216, (byte)119, (byte)135, (byte)7}, 0) ;
            SMP_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (short)(short) -7095);
                Debug.Assert(pack.xgyro == (short)(short) -4515);
                Debug.Assert(pack.xacc == (short)(short) -14884);
                Debug.Assert(pack.ygyro == (short)(short) -7496);
                Debug.Assert(pack.zgyro == (short)(short)19976);
                Debug.Assert(pack.time_boot_ms == (uint)3159113406U);
                Debug.Assert(pack.yacc == (short)(short) -17525);
                Debug.Assert(pack.ymag == (short)(short)14990);
                Debug.Assert(pack.xmag == (short)(short) -4178);
                Debug.Assert(pack.zmag == (short)(short) -6776);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.time_boot_ms = (uint)3159113406U;
            p26.xacc = (short)(short) -14884;
            p26.zmag = (short)(short) -6776;
            p26.ymag = (short)(short)14990;
            p26.xgyro = (short)(short) -4515;
            p26.zacc = (short)(short) -7095;
            p26.ygyro = (short)(short) -7496;
            p26.yacc = (short)(short) -17525;
            p26.xmag = (short)(short) -4178;
            p26.zgyro = (short)(short)19976;
            SMP_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short) -9499);
                Debug.Assert(pack.ygyro == (short)(short)31153);
                Debug.Assert(pack.time_usec == (ulong)2350985851600867637L);
                Debug.Assert(pack.xacc == (short)(short)17602);
                Debug.Assert(pack.zgyro == (short)(short)737);
                Debug.Assert(pack.xmag == (short)(short) -7989);
                Debug.Assert(pack.zmag == (short)(short) -27426);
                Debug.Assert(pack.ymag == (short)(short)2539);
                Debug.Assert(pack.zacc == (short)(short) -14843);
                Debug.Assert(pack.xgyro == (short)(short)14185);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.ygyro = (short)(short)31153;
            p27.xmag = (short)(short) -7989;
            p27.zacc = (short)(short) -14843;
            p27.xacc = (short)(short)17602;
            p27.zmag = (short)(short) -27426;
            p27.xgyro = (short)(short)14185;
            p27.zgyro = (short)(short)737;
            p27.ymag = (short)(short)2539;
            p27.yacc = (short)(short) -9499;
            p27.time_usec = (ulong)2350985851600867637L;
            SMP_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -9451);
                Debug.Assert(pack.press_diff2 == (short)(short)846);
                Debug.Assert(pack.press_diff1 == (short)(short) -18646);
                Debug.Assert(pack.time_usec == (ulong)8520408026875906386L);
                Debug.Assert(pack.press_abs == (short)(short) -8200);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_diff1 = (short)(short) -18646;
            p28.temperature = (short)(short) -9451;
            p28.time_usec = (ulong)8520408026875906386L;
            p28.press_abs = (short)(short) -8200;
            p28.press_diff2 = (short)(short)846;
            SMP_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)13540);
                Debug.Assert(pack.press_diff == (float) -1.8932966E38F);
                Debug.Assert(pack.press_abs == (float)3.0242971E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3143859584U);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.temperature = (short)(short)13540;
            p29.time_boot_ms = (uint)3143859584U;
            p29.press_abs = (float)3.0242971E38F;
            p29.press_diff = (float) -1.8932966E38F;
            SMP_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yawspeed == (float)3.3314975E38F);
                Debug.Assert(pack.rollspeed == (float) -8.419933E37F);
                Debug.Assert(pack.yaw == (float) -2.1717408E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1874896157U);
                Debug.Assert(pack.pitchspeed == (float) -1.0292633E38F);
                Debug.Assert(pack.pitch == (float)1.8135087E38F);
                Debug.Assert(pack.roll == (float)1.0997331E38F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.pitchspeed = (float) -1.0292633E38F;
            p30.yawspeed = (float)3.3314975E38F;
            p30.time_boot_ms = (uint)1874896157U;
            p30.yaw = (float) -2.1717408E38F;
            p30.roll = (float)1.0997331E38F;
            p30.pitch = (float)1.8135087E38F;
            p30.rollspeed = (float) -8.419933E37F;
            SMP_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yawspeed == (float) -1.0940725E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2133595697U);
                Debug.Assert(pack.q1 == (float) -3.343833E38F);
                Debug.Assert(pack.rollspeed == (float)1.6010453E38F);
                Debug.Assert(pack.q3 == (float)1.7618496E38F);
                Debug.Assert(pack.pitchspeed == (float)2.5826314E38F);
                Debug.Assert(pack.q4 == (float) -9.201932E36F);
                Debug.Assert(pack.q2 == (float) -3.2207467E38F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.q4 = (float) -9.201932E36F;
            p31.q2 = (float) -3.2207467E38F;
            p31.time_boot_ms = (uint)2133595697U;
            p31.q1 = (float) -3.343833E38F;
            p31.q3 = (float)1.7618496E38F;
            p31.pitchspeed = (float)2.5826314E38F;
            p31.yawspeed = (float) -1.0940725E38F;
            p31.rollspeed = (float)1.6010453E38F;
            SMP_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -8.2425913E37F);
                Debug.Assert(pack.y == (float) -1.0634591E38F);
                Debug.Assert(pack.vx == (float) -3.0418619E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4262658531U);
                Debug.Assert(pack.z == (float)1.1890252E38F);
                Debug.Assert(pack.vz == (float)3.3776124E38F);
                Debug.Assert(pack.vy == (float)4.951164E37F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.time_boot_ms = (uint)4262658531U;
            p32.vx = (float) -3.0418619E38F;
            p32.x = (float) -8.2425913E37F;
            p32.vz = (float)3.3776124E38F;
            p32.z = (float)1.1890252E38F;
            p32.vy = (float)4.951164E37F;
            p32.y = (float) -1.0634591E38F;
            SMP_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.relative_alt == (int)720920501);
                Debug.Assert(pack.vy == (short)(short)6084);
                Debug.Assert(pack.vx == (short)(short)18582);
                Debug.Assert(pack.time_boot_ms == (uint)4076535634U);
                Debug.Assert(pack.lat == (int)1473967880);
                Debug.Assert(pack.alt == (int)2014796390);
                Debug.Assert(pack.vz == (short)(short)31445);
                Debug.Assert(pack.lon == (int) -1425095864);
                Debug.Assert(pack.hdg == (ushort)(ushort)20624);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.relative_alt = (int)720920501;
            p33.hdg = (ushort)(ushort)20624;
            p33.vz = (short)(short)31445;
            p33.lon = (int) -1425095864;
            p33.vx = (short)(short)18582;
            p33.lat = (int)1473967880;
            p33.time_boot_ms = (uint)4076535634U;
            p33.alt = (int)2014796390;
            p33.vy = (short)(short)6084;
            SMP_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3045610341U);
                Debug.Assert(pack.chan2_scaled == (short)(short)25101);
                Debug.Assert(pack.port == (byte)(byte)204);
                Debug.Assert(pack.chan8_scaled == (short)(short)6963);
                Debug.Assert(pack.chan5_scaled == (short)(short) -9016);
                Debug.Assert(pack.chan7_scaled == (short)(short) -1155);
                Debug.Assert(pack.chan3_scaled == (short)(short) -30491);
                Debug.Assert(pack.chan1_scaled == (short)(short) -22097);
                Debug.Assert(pack.chan6_scaled == (short)(short)13823);
                Debug.Assert(pack.rssi == (byte)(byte)40);
                Debug.Assert(pack.chan4_scaled == (short)(short) -14088);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan1_scaled = (short)(short) -22097;
            p34.time_boot_ms = (uint)3045610341U;
            p34.chan2_scaled = (short)(short)25101;
            p34.chan3_scaled = (short)(short) -30491;
            p34.chan8_scaled = (short)(short)6963;
            p34.chan4_scaled = (short)(short) -14088;
            p34.port = (byte)(byte)204;
            p34.rssi = (byte)(byte)40;
            p34.chan5_scaled = (short)(short) -9016;
            p34.chan7_scaled = (short)(short) -1155;
            p34.chan6_scaled = (short)(short)13823;
            SMP_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)18197);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)15278);
                Debug.Assert(pack.time_boot_ms == (uint)2007932254U);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)8703);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)44466);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)10974);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)36812);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)56227);
                Debug.Assert(pack.port == (byte)(byte)153);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)5894);
                Debug.Assert(pack.rssi == (byte)(byte)60);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.rssi = (byte)(byte)60;
            p35.port = (byte)(byte)153;
            p35.chan4_raw = (ushort)(ushort)8703;
            p35.time_boot_ms = (uint)2007932254U;
            p35.chan7_raw = (ushort)(ushort)10974;
            p35.chan8_raw = (ushort)(ushort)36812;
            p35.chan1_raw = (ushort)(ushort)56227;
            p35.chan5_raw = (ushort)(ushort)5894;
            p35.chan2_raw = (ushort)(ushort)15278;
            p35.chan3_raw = (ushort)(ushort)44466;
            p35.chan6_raw = (ushort)(ushort)18197;
            SMP_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.port == (byte)(byte)225);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)36638);
                Debug.Assert(pack.time_usec == (uint)4287318916U);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)64954);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)2402);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)20209);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)38601);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)10586);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)35498);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)19170);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)37332);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)4419);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)59367);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)15083);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)25775);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)7419);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)48374);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)47775);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo2_raw = (ushort)(ushort)59367;
            p36.servo1_raw = (ushort)(ushort)15083;
            p36.servo6_raw = (ushort)(ushort)48374;
            p36.servo10_raw_SET((ushort)(ushort)7419, PH) ;
            p36.servo12_raw_SET((ushort)(ushort)38601, PH) ;
            p36.servo13_raw_SET((ushort)(ushort)2402, PH) ;
            p36.servo11_raw_SET((ushort)(ushort)35498, PH) ;
            p36.servo9_raw_SET((ushort)(ushort)36638, PH) ;
            p36.servo8_raw = (ushort)(ushort)19170;
            p36.servo5_raw = (ushort)(ushort)47775;
            p36.servo4_raw = (ushort)(ushort)10586;
            p36.servo7_raw = (ushort)(ushort)37332;
            p36.servo3_raw = (ushort)(ushort)20209;
            p36.time_usec = (uint)4287318916U;
            p36.servo16_raw_SET((ushort)(ushort)25775, PH) ;
            p36.port = (byte)(byte)225;
            p36.servo15_raw_SET((ushort)(ushort)64954, PH) ;
            p36.servo14_raw_SET((ushort)(ushort)4419, PH) ;
            SMP_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)48);
                Debug.Assert(pack.start_index == (short)(short)1140);
                Debug.Assert(pack.target_system == (byte)(byte)67);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.end_index == (short)(short)31404);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.end_index = (short)(short)31404;
            p37.target_system = (byte)(byte)67;
            p37.start_index = (short)(short)1140;
            p37.target_component = (byte)(byte)48;
            p37.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            SMP_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end_index == (short)(short)9201);
                Debug.Assert(pack.target_system == (byte)(byte)28);
                Debug.Assert(pack.target_component == (byte)(byte)79);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.start_index == (short)(short) -8085);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.end_index = (short)(short)9201;
            p38.target_system = (byte)(byte)28;
            p38.target_component = (byte)(byte)79;
            p38.start_index = (short)(short) -8085;
            p38.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            SMP_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)4);
                Debug.Assert(pack.param1 == (float) -2.4032812E38F);
                Debug.Assert(pack.param2 == (float)9.172193E37F);
                Debug.Assert(pack.seq == (ushort)(ushort)46288);
                Debug.Assert(pack.x == (float) -2.4534976E38F);
                Debug.Assert(pack.param4 == (float) -7.8515996E37F);
                Debug.Assert(pack.autocontinue == (byte)(byte)108);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_WAYPOINT);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.y == (float)9.572045E37F);
                Debug.Assert(pack.target_component == (byte)(byte)167);
                Debug.Assert(pack.z == (float) -7.136443E37F);
                Debug.Assert(pack.current == (byte)(byte)12);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.param3 == (float) -1.2472616E38F);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.command = MAV_CMD.MAV_CMD_NAV_WAYPOINT;
            p39.target_system = (byte)(byte)4;
            p39.param2 = (float)9.172193E37F;
            p39.y = (float)9.572045E37F;
            p39.current = (byte)(byte)12;
            p39.param3 = (float) -1.2472616E38F;
            p39.target_component = (byte)(byte)167;
            p39.frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p39.param1 = (float) -2.4032812E38F;
            p39.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p39.autocontinue = (byte)(byte)108;
            p39.param4 = (float) -7.8515996E37F;
            p39.x = (float) -2.4534976E38F;
            p39.seq = (ushort)(ushort)46288;
            p39.z = (float) -7.136443E37F;
            SMP_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_component == (byte)(byte)15);
                Debug.Assert(pack.target_system == (byte)(byte)81);
                Debug.Assert(pack.seq == (ushort)(ushort)5588);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p40.target_system = (byte)(byte)81;
            p40.seq = (ushort)(ushort)5588;
            p40.target_component = (byte)(byte)15;
            SMP_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)26);
                Debug.Assert(pack.target_component == (byte)(byte)205);
                Debug.Assert(pack.seq == (ushort)(ushort)21092);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.seq = (ushort)(ushort)21092;
            p41.target_system = (byte)(byte)26;
            p41.target_component = (byte)(byte)205;
            SMP_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)63623);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)63623;
            SMP_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)224);
                Debug.Assert(pack.target_system == (byte)(byte)154);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_component = (byte)(byte)224;
            p43.target_system = (byte)(byte)154;
            p43.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            SMP_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)83);
                Debug.Assert(pack.count == (ushort)(ushort)33094);
                Debug.Assert(pack.target_component == (byte)(byte)37);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p44.count = (ushort)(ushort)33094;
            p44.target_component = (byte)(byte)37;
            p44.target_system = (byte)(byte)83;
            SMP_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)32);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_system == (byte)(byte)155);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_component = (byte)(byte)32;
            p45.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p45.target_system = (byte)(byte)155;
            SMP_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)44533);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)44533;
            SMP_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == MAV_MISSION_RESULT.MAV_MISSION_ERROR);
                Debug.Assert(pack.target_component == (byte)(byte)187);
                Debug.Assert(pack.target_system == (byte)(byte)209);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.target_component = (byte)(byte)187;
            p47.target_system = (byte)(byte)209;
            p47.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p47.type = MAV_MISSION_RESULT.MAV_MISSION_ERROR;
            SMP_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int) -1917918226);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)3219892629622191061L);
                Debug.Assert(pack.latitude == (int) -479326808);
                Debug.Assert(pack.longitude == (int) -1927772812);
                Debug.Assert(pack.target_system == (byte)(byte)167);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.latitude = (int) -479326808;
            p48.longitude = (int) -1927772812;
            p48.altitude = (int) -1917918226;
            p48.target_system = (byte)(byte)167;
            p48.time_usec_SET((ulong)3219892629622191061L, PH) ;
            SMP_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int)772900307);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)10958947484850122L);
                Debug.Assert(pack.longitude == (int)294508781);
                Debug.Assert(pack.altitude == (int) -860543700);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.latitude = (int)772900307;
            p49.longitude = (int)294508781;
            p49.altitude = (int) -860543700;
            p49.time_usec_SET((ulong)10958947484850122L, PH) ;
            SMP_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_max == (float) -2.5613862E38F);
                Debug.Assert(pack.target_component == (byte)(byte)185);
                Debug.Assert(pack.param_value_min == (float)1.3413383E38F);
                Debug.Assert(pack.target_system == (byte)(byte)8);
                Debug.Assert(pack.param_value0 == (float) -2.5650204E38F);
                Debug.Assert(pack.param_index == (short)(short) -23936);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)179);
                Debug.Assert(pack.param_id_LEN(ph) == 16);
                Debug.Assert(pack.param_id_TRY(ph).Equals("zpwfrtvnzcadefbj"));
                Debug.Assert(pack.scale == (float)2.1680316E38F);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.scale = (float)2.1680316E38F;
            p50.param_id_SET("zpwfrtvnzcadefbj", PH) ;
            p50.param_value_max = (float) -2.5613862E38F;
            p50.param_value_min = (float)1.3413383E38F;
            p50.target_component = (byte)(byte)185;
            p50.param_value0 = (float) -2.5650204E38F;
            p50.parameter_rc_channel_index = (byte)(byte)179;
            p50.param_index = (short)(short) -23936;
            p50.target_system = (byte)(byte)8;
            SMP_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)16338);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)168);
                Debug.Assert(pack.target_system == (byte)(byte)4);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_component = (byte)(byte)168;
            p51.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p51.target_system = (byte)(byte)4;
            p51.seq = (ushort)(ushort)16338;
            SMP_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)228);
                Debug.Assert(pack.p2z == (float) -2.2938462E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.target_component == (byte)(byte)79);
                Debug.Assert(pack.p1y == (float) -2.1610978E38F);
                Debug.Assert(pack.p2x == (float) -1.3310767E38F);
                Debug.Assert(pack.p2y == (float)2.4528743E38F);
                Debug.Assert(pack.p1x == (float) -1.9206326E38F);
                Debug.Assert(pack.p1z == (float)5.4057393E37F);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p2z = (float) -2.2938462E38F;
            p54.p1y = (float) -2.1610978E38F;
            p54.target_system = (byte)(byte)228;
            p54.p2x = (float) -1.3310767E38F;
            p54.target_component = (byte)(byte)79;
            p54.p1z = (float)5.4057393E37F;
            p54.frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p54.p2y = (float)2.4528743E38F;
            p54.p1x = (float) -1.9206326E38F;
            SMP_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.p2z == (float)1.4164851E38F);
                Debug.Assert(pack.p1x == (float) -2.8327744E38F);
                Debug.Assert(pack.p2x == (float)2.243169E38F);
                Debug.Assert(pack.p1y == (float)3.1188547E38F);
                Debug.Assert(pack.p1z == (float)2.2985479E38F);
                Debug.Assert(pack.p2y == (float) -4.4627584E37F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p2x = (float)2.243169E38F;
            p55.p1z = (float)2.2985479E38F;
            p55.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p55.p1x = (float) -2.8327744E38F;
            p55.p2y = (float) -4.4627584E37F;
            p55.p1y = (float)3.1188547E38F;
            p55.p2z = (float)1.4164851E38F;
            SMP_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)7962782767206097960L);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-3.3800262E38F, 7.0918084E37F, -3.492054E37F, -1.6045821E36F, -2.5438494E38F, -2.548035E38F, 8.604719E37F, -7.0606977E37F, 4.4164166E37F}));
                Debug.Assert(pack.pitchspeed == (float) -1.5743892E38F);
                Debug.Assert(pack.yawspeed == (float)7.5945074E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.0591205E36F, -2.896005E38F, -1.8091562E37F, 1.7750256E38F}));
                Debug.Assert(pack.rollspeed == (float)8.790164E37F);
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.pitchspeed = (float) -1.5743892E38F;
            p61.q_SET(new float[] {2.0591205E36F, -2.896005E38F, -1.8091562E37F, 1.7750256E38F}, 0) ;
            p61.yawspeed = (float)7.5945074E37F;
            p61.covariance_SET(new float[] {-3.3800262E38F, 7.0918084E37F, -3.492054E37F, -1.6045821E36F, -2.5438494E38F, -2.548035E38F, 8.604719E37F, -7.0606977E37F, 4.4164166E37F}, 0) ;
            p61.rollspeed = (float)8.790164E37F;
            p61.time_usec = (ulong)7962782767206097960L;
            SMP_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nav_bearing == (short)(short) -18339);
                Debug.Assert(pack.nav_roll == (float) -2.9946643E38F);
                Debug.Assert(pack.nav_pitch == (float) -2.365208E38F);
                Debug.Assert(pack.aspd_error == (float) -3.327789E38F);
                Debug.Assert(pack.target_bearing == (short)(short) -18665);
                Debug.Assert(pack.alt_error == (float)1.1795022E38F);
                Debug.Assert(pack.xtrack_error == (float)2.8602353E38F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)11988);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_pitch = (float) -2.365208E38F;
            p62.aspd_error = (float) -3.327789E38F;
            p62.wp_dist = (ushort)(ushort)11988;
            p62.nav_roll = (float) -2.9946643E38F;
            p62.xtrack_error = (float)2.8602353E38F;
            p62.nav_bearing = (short)(short) -18339;
            p62.alt_error = (float)1.1795022E38F;
            p62.target_bearing = (short)(short) -18665;
            SMP_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.0695069E37F, -3.1104241E37F, 1.2927398E38F, 9.899642E36F, 1.594567E38F, -9.017975E36F, -2.8054438E38F, 2.5908332E38F, 3.2285434E38F, 8.46753E37F, 2.084336E38F, -3.164229E38F, 3.2061581E38F, -1.2842798E38F, -4.0273412E37F, -1.2664353E38F, -2.6630094E38F, 5.264769E37F, -1.579656E38F, 2.5104914E38F, 2.658696E38F, 9.391357E36F, -3.122816E38F, 3.113561E38F, 2.940699E38F, 3.3202193E38F, 1.6993008E38F, 8.634613E37F, -4.985281E37F, 1.3404317E38F, 2.2944596E38F, -2.2581153E38F, -3.3265387E38F, 1.0965983E38F, 1.7707905E38F, 2.8393583E38F}));
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
                Debug.Assert(pack.lat == (int)985846691);
                Debug.Assert(pack.lon == (int) -377300791);
                Debug.Assert(pack.time_usec == (ulong)7976716560880566674L);
                Debug.Assert(pack.vx == (float)1.5739707E38F);
                Debug.Assert(pack.alt == (int) -797576880);
                Debug.Assert(pack.relative_alt == (int) -302824244);
                Debug.Assert(pack.vy == (float) -1.7557912E38F);
                Debug.Assert(pack.vz == (float) -4.7745372E36F);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS;
            p63.vz = (float) -4.7745372E36F;
            p63.alt = (int) -797576880;
            p63.covariance_SET(new float[] {-2.0695069E37F, -3.1104241E37F, 1.2927398E38F, 9.899642E36F, 1.594567E38F, -9.017975E36F, -2.8054438E38F, 2.5908332E38F, 3.2285434E38F, 8.46753E37F, 2.084336E38F, -3.164229E38F, 3.2061581E38F, -1.2842798E38F, -4.0273412E37F, -1.2664353E38F, -2.6630094E38F, 5.264769E37F, -1.579656E38F, 2.5104914E38F, 2.658696E38F, 9.391357E36F, -3.122816E38F, 3.113561E38F, 2.940699E38F, 3.3202193E38F, 1.6993008E38F, 8.634613E37F, -4.985281E37F, 1.3404317E38F, 2.2944596E38F, -2.2581153E38F, -3.3265387E38F, 1.0965983E38F, 1.7707905E38F, 2.8393583E38F}, 0) ;
            p63.relative_alt = (int) -302824244;
            p63.vx = (float)1.5739707E38F;
            p63.lon = (int) -377300791;
            p63.lat = (int)985846691;
            p63.vy = (float) -1.7557912E38F;
            p63.time_usec = (ulong)7976716560880566674L;
            SMP_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -3.3575509E38F);
                Debug.Assert(pack.ay == (float) -5.808315E37F);
                Debug.Assert(pack.az == (float) -2.184028E38F);
                Debug.Assert(pack.y == (float) -2.7021816E38F);
                Debug.Assert(pack.vz == (float)2.032916E38F);
                Debug.Assert(pack.z == (float)1.8179706E38F);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
                Debug.Assert(pack.vy == (float)2.7580211E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {3.3859872E38F, 1.1706317E38F, -2.6931434E38F, -2.256775E38F, 1.4898904E38F, 1.1681014E37F, -3.2059863E38F, 7.893273E37F, -3.0295634E38F, 1.985159E38F, 3.2617838E38F, -1.23402E38F, 2.5184556E38F, -5.8329974E37F, 1.8205416E38F, 2.3891437E38F, 1.4172208E38F, -2.5778316E38F, -1.580054E38F, -3.3345494E38F, 1.7053256E37F, -1.4872392E38F, -2.6277286E37F, 2.1328686E38F, 9.110462E37F, -8.844796E37F, 5.465488E37F, -2.7852745E37F, 2.6908E38F, -1.3348063E38F, -3.2066413E38F, -1.6442205E38F, -1.3969046E38F, -2.9400075E38F, 2.1763782E38F, 2.0976737E38F, 8.776582E37F, -2.7326058E38F, 1.0551595E38F, 3.5650087E37F, -1.75951E38F, 1.0490677E38F, 2.7731548E38F, 2.8636657E38F, -1.8337187E38F}));
                Debug.Assert(pack.ax == (float)2.5917916E38F);
                Debug.Assert(pack.time_usec == (ulong)2561266684529685717L);
                Debug.Assert(pack.vx == (float) -3.05786E38F);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.y = (float) -2.7021816E38F;
            p64.vx = (float) -3.05786E38F;
            p64.z = (float)1.8179706E38F;
            p64.x = (float) -3.3575509E38F;
            p64.vy = (float)2.7580211E38F;
            p64.covariance_SET(new float[] {3.3859872E38F, 1.1706317E38F, -2.6931434E38F, -2.256775E38F, 1.4898904E38F, 1.1681014E37F, -3.2059863E38F, 7.893273E37F, -3.0295634E38F, 1.985159E38F, 3.2617838E38F, -1.23402E38F, 2.5184556E38F, -5.8329974E37F, 1.8205416E38F, 2.3891437E38F, 1.4172208E38F, -2.5778316E38F, -1.580054E38F, -3.3345494E38F, 1.7053256E37F, -1.4872392E38F, -2.6277286E37F, 2.1328686E38F, 9.110462E37F, -8.844796E37F, 5.465488E37F, -2.7852745E37F, 2.6908E38F, -1.3348063E38F, -3.2066413E38F, -1.6442205E38F, -1.3969046E38F, -2.9400075E38F, 2.1763782E38F, 2.0976737E38F, 8.776582E37F, -2.7326058E38F, 1.0551595E38F, 3.5650087E37F, -1.75951E38F, 1.0490677E38F, 2.7731548E38F, 2.8636657E38F, -1.8337187E38F}, 0) ;
            p64.time_usec = (ulong)2561266684529685717L;
            p64.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
            p64.ax = (float)2.5917916E38F;
            p64.az = (float) -2.184028E38F;
            p64.vz = (float)2.032916E38F;
            p64.ay = (float) -5.808315E37F;
            SMP_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)49228);
                Debug.Assert(pack.rssi == (byte)(byte)99);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)18886);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)54580);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)27138);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)16240);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)55693);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)39360);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)11204);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)53359);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)63115);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)59685);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)41542);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)45994);
                Debug.Assert(pack.time_boot_ms == (uint)2229040555U);
                Debug.Assert(pack.chancount == (byte)(byte)163);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)32463);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)28127);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)44677);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)49949);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)44242);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan6_raw = (ushort)(ushort)18886;
            p65.chan10_raw = (ushort)(ushort)41542;
            p65.chan4_raw = (ushort)(ushort)27138;
            p65.chan2_raw = (ushort)(ushort)59685;
            p65.chan9_raw = (ushort)(ushort)45994;
            p65.chan14_raw = (ushort)(ushort)49228;
            p65.chan12_raw = (ushort)(ushort)63115;
            p65.chan11_raw = (ushort)(ushort)28127;
            p65.chan3_raw = (ushort)(ushort)32463;
            p65.chan1_raw = (ushort)(ushort)53359;
            p65.chan8_raw = (ushort)(ushort)55693;
            p65.chan7_raw = (ushort)(ushort)16240;
            p65.time_boot_ms = (uint)2229040555U;
            p65.chan15_raw = (ushort)(ushort)54580;
            p65.chan17_raw = (ushort)(ushort)11204;
            p65.chan18_raw = (ushort)(ushort)49949;
            p65.chan16_raw = (ushort)(ushort)39360;
            p65.chan13_raw = (ushort)(ushort)44677;
            p65.rssi = (byte)(byte)99;
            p65.chancount = (byte)(byte)163;
            p65.chan5_raw = (ushort)(ushort)44242;
            SMP_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)37024);
                Debug.Assert(pack.target_system == (byte)(byte)70);
                Debug.Assert(pack.req_stream_id == (byte)(byte)45);
                Debug.Assert(pack.target_component == (byte)(byte)76);
                Debug.Assert(pack.start_stop == (byte)(byte)9);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.req_message_rate = (ushort)(ushort)37024;
            p66.target_system = (byte)(byte)70;
            p66.req_stream_id = (byte)(byte)45;
            p66.start_stop = (byte)(byte)9;
            p66.target_component = (byte)(byte)76;
            SMP_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.on_off == (byte)(byte)30);
                Debug.Assert(pack.stream_id == (byte)(byte)211);
                Debug.Assert(pack.message_rate == (ushort)(ushort)43529);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.stream_id = (byte)(byte)211;
            p67.on_off = (byte)(byte)30;
            p67.message_rate = (ushort)(ushort)43529;
            SMP_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (short)(short) -27316);
                Debug.Assert(pack.target == (byte)(byte)140);
                Debug.Assert(pack.y == (short)(short) -31708);
                Debug.Assert(pack.x == (short)(short)4534);
                Debug.Assert(pack.buttons == (ushort)(ushort)65444);
                Debug.Assert(pack.r == (short)(short)31328);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.r = (short)(short)31328;
            p69.target = (byte)(byte)140;
            p69.z = (short)(short) -27316;
            p69.buttons = (ushort)(ushort)65444;
            p69.x = (short)(short)4534;
            p69.y = (short)(short) -31708;
            SMP_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)47008);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)62384);
                Debug.Assert(pack.target_system == (byte)(byte)116);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)59912);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)47822);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)27796);
                Debug.Assert(pack.target_component == (byte)(byte)91);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)1635);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)63704);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)46141);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan1_raw = (ushort)(ushort)47008;
            p70.chan6_raw = (ushort)(ushort)62384;
            p70.chan3_raw = (ushort)(ushort)59912;
            p70.chan5_raw = (ushort)(ushort)47822;
            p70.chan8_raw = (ushort)(ushort)1635;
            p70.chan2_raw = (ushort)(ushort)46141;
            p70.chan7_raw = (ushort)(ushort)27796;
            p70.target_system = (byte)(byte)116;
            p70.target_component = (byte)(byte)91;
            p70.chan4_raw = (ushort)(ushort)63704;
            SMP_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current == (byte)(byte)202);
                Debug.Assert(pack.param3 == (float)1.5037416E38F);
                Debug.Assert(pack.z == (float)1.0957642E37F);
                Debug.Assert(pack.y == (int) -1807909704);
                Debug.Assert(pack.param4 == (float)1.4019021E38F);
                Debug.Assert(pack.param2 == (float) -1.9027825E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT);
                Debug.Assert(pack.autocontinue == (byte)(byte)221);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.param1 == (float) -9.579477E37F);
                Debug.Assert(pack.x == (int)717227983);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)150);
                Debug.Assert(pack.target_system == (byte)(byte)248);
                Debug.Assert(pack.seq == (ushort)(ushort)17222);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.param2 = (float) -1.9027825E38F;
            p73.param4 = (float)1.4019021E38F;
            p73.target_component = (byte)(byte)150;
            p73.autocontinue = (byte)(byte)221;
            p73.z = (float)1.0957642E37F;
            p73.x = (int)717227983;
            p73.command = MAV_CMD.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
            p73.y = (int) -1807909704;
            p73.seq = (ushort)(ushort)17222;
            p73.current = (byte)(byte)202;
            p73.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p73.param1 = (float) -9.579477E37F;
            p73.param3 = (float)1.5037416E38F;
            p73.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p73.target_system = (byte)(byte)248;
            SMP_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.airspeed == (float)5.846421E37F);
                Debug.Assert(pack.climb == (float)1.5042421E38F);
                Debug.Assert(pack.heading == (short)(short) -18079);
                Debug.Assert(pack.groundspeed == (float) -1.2891966E38F);
                Debug.Assert(pack.alt == (float) -3.3365755E37F);
                Debug.Assert(pack.throttle == (ushort)(ushort)41910);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.alt = (float) -3.3365755E37F;
            p74.airspeed = (float)5.846421E37F;
            p74.throttle = (ushort)(ushort)41910;
            p74.climb = (float)1.5042421E38F;
            p74.heading = (short)(short) -18079;
            p74.groundspeed = (float) -1.2891966E38F;
            SMP_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (int)1681333874);
                Debug.Assert(pack.autocontinue == (byte)(byte)120);
                Debug.Assert(pack.param2 == (float)3.6430625E37F);
                Debug.Assert(pack.target_system == (byte)(byte)35);
                Debug.Assert(pack.target_component == (byte)(byte)78);
                Debug.Assert(pack.param4 == (float)1.2994832E38F);
                Debug.Assert(pack.current == (byte)(byte)3);
                Debug.Assert(pack.z == (float)3.3571607E38F);
                Debug.Assert(pack.param3 == (float) -4.089885E36F);
                Debug.Assert(pack.param1 == (float) -6.9683626E37F);
                Debug.Assert(pack.y == (int)2066218867);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.z = (float)3.3571607E38F;
            p75.x = (int)1681333874;
            p75.param2 = (float)3.6430625E37F;
            p75.param1 = (float) -6.9683626E37F;
            p75.command = MAV_CMD.MAV_CMD_NAV_SET_YAW_SPEED;
            p75.target_component = (byte)(byte)78;
            p75.y = (int)2066218867;
            p75.autocontinue = (byte)(byte)120;
            p75.frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p75.param4 = (float)1.2994832E38F;
            p75.param3 = (float) -4.089885E36F;
            p75.current = (byte)(byte)3;
            p75.target_system = (byte)(byte)35;
            SMP_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.confirmation == (byte)(byte)178);
                Debug.Assert(pack.param1 == (float)1.4747991E38F);
                Debug.Assert(pack.target_system == (byte)(byte)96);
                Debug.Assert(pack.param6 == (float)5.527714E37F);
                Debug.Assert(pack.target_component == (byte)(byte)95);
                Debug.Assert(pack.param4 == (float) -2.6451195E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY);
                Debug.Assert(pack.param3 == (float)1.7503876E38F);
                Debug.Assert(pack.param7 == (float) -1.6448819E38F);
                Debug.Assert(pack.param5 == (float) -2.9821817E38F);
                Debug.Assert(pack.param2 == (float) -2.6849383E38F);
            };
            GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.target_component = (byte)(byte)95;
            p76.param1 = (float)1.4747991E38F;
            p76.param2 = (float) -2.6849383E38F;
            p76.command = MAV_CMD.MAV_CMD_PAYLOAD_CONTROL_DEPLOY;
            p76.param3 = (float)1.7503876E38F;
            p76.param7 = (float) -1.6448819E38F;
            p76.confirmation = (byte)(byte)178;
            p76.target_system = (byte)(byte)96;
            p76.param5 = (float) -2.9821817E38F;
            p76.param4 = (float) -2.6451195E38F;
            p76.param6 = (float)5.527714E37F;
            CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)6);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION);
                Debug.Assert(pack.result == MAV_RESULT.MAV_RESULT_FAILED);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)125);
                Debug.Assert(pack.result_param2_TRY(ph) == (int) -704791954);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)234);
            };
            GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.result_param2_SET((int) -704791954, PH) ;
            p77.target_component_SET((byte)(byte)6, PH) ;
            p77.command = MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION;
            p77.result = MAV_RESULT.MAV_RESULT_FAILED;
            p77.progress_SET((byte)(byte)125, PH) ;
            p77.target_system_SET((byte)(byte)234, PH) ;
            CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -1.2856652E38F);
                Debug.Assert(pack.pitch == (float) -4.0085315E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3234037241U);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)117);
                Debug.Assert(pack.thrust == (float)2.2103088E38F);
                Debug.Assert(pack.mode_switch == (byte)(byte)226);
                Debug.Assert(pack.roll == (float)1.536852E38F);
            };
            GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.yaw = (float) -1.2856652E38F;
            p81.roll = (float)1.536852E38F;
            p81.time_boot_ms = (uint)3234037241U;
            p81.pitch = (float) -4.0085315E37F;
            p81.mode_switch = (byte)(byte)226;
            p81.manual_override_switch = (byte)(byte)117;
            p81.thrust = (float)2.2103088E38F;
            CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_roll_rate == (float)3.6554685E37F);
                Debug.Assert(pack.body_yaw_rate == (float) -1.485605E38F);
                Debug.Assert(pack.target_system == (byte)(byte)220);
                Debug.Assert(pack.type_mask == (byte)(byte)167);
                Debug.Assert(pack.time_boot_ms == (uint)2564211658U);
                Debug.Assert(pack.thrust == (float)5.9475484E37F);
                Debug.Assert(pack.target_component == (byte)(byte)212);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.022368E38F, 6.327581E37F, 1.721323E38F, 2.7147607E37F}));
                Debug.Assert(pack.body_pitch_rate == (float) -1.8041047E38F);
            };
            GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.body_yaw_rate = (float) -1.485605E38F;
            p82.thrust = (float)5.9475484E37F;
            p82.target_component = (byte)(byte)212;
            p82.type_mask = (byte)(byte)167;
            p82.target_system = (byte)(byte)220;
            p82.time_boot_ms = (uint)2564211658U;
            p82.q_SET(new float[] {-3.022368E38F, 6.327581E37F, 1.721323E38F, 2.7147607E37F}, 0) ;
            p82.body_pitch_rate = (float) -1.8041047E38F;
            p82.body_roll_rate = (float)3.6554685E37F;
            CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_roll_rate == (float)3.2987289E38F);
                Debug.Assert(pack.body_yaw_rate == (float)2.7362644E38F);
                Debug.Assert(pack.body_pitch_rate == (float)5.9116835E37F);
                Debug.Assert(pack.thrust == (float)2.079322E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3493449597U);
                Debug.Assert(pack.type_mask == (byte)(byte)123);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.4985004E38F, 2.1008283E38F, -8.046838E37F, -2.9101218E38F}));
            };
            GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.q_SET(new float[] {-1.4985004E38F, 2.1008283E38F, -8.046838E37F, -2.9101218E38F}, 0) ;
            p83.body_yaw_rate = (float)2.7362644E38F;
            p83.body_pitch_rate = (float)5.9116835E37F;
            p83.thrust = (float)2.079322E38F;
            p83.type_mask = (byte)(byte)123;
            p83.time_boot_ms = (uint)3493449597U;
            p83.body_roll_rate = (float)3.2987289E38F;
            CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float) -1.6612621E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3856890852U);
                Debug.Assert(pack.target_system == (byte)(byte)103);
                Debug.Assert(pack.vy == (float)4.6408683E37F);
                Debug.Assert(pack.yaw_rate == (float)1.66279E38F);
                Debug.Assert(pack.x == (float) -2.3502928E38F);
                Debug.Assert(pack.vz == (float) -1.3165753E38F);
                Debug.Assert(pack.z == (float)2.8878354E38F);
                Debug.Assert(pack.target_component == (byte)(byte)91);
                Debug.Assert(pack.yaw == (float) -2.8867406E38F);
                Debug.Assert(pack.afz == (float)1.143679E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.y == (float)1.6802859E38F);
                Debug.Assert(pack.afy == (float) -3.2687052E38F);
                Debug.Assert(pack.afx == (float) -6.544499E36F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)50135);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.time_boot_ms = (uint)3856890852U;
            p84.type_mask = (ushort)(ushort)50135;
            p84.vx = (float) -1.6612621E38F;
            p84.yaw = (float) -2.8867406E38F;
            p84.afz = (float)1.143679E38F;
            p84.target_component = (byte)(byte)91;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p84.target_system = (byte)(byte)103;
            p84.z = (float)2.8878354E38F;
            p84.yaw_rate = (float)1.66279E38F;
            p84.afy = (float) -3.2687052E38F;
            p84.x = (float) -2.3502928E38F;
            p84.vz = (float) -1.3165753E38F;
            p84.vy = (float)4.6408683E37F;
            p84.y = (float)1.6802859E38F;
            p84.afx = (float) -6.544499E36F;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon_int == (int) -396585074);
                Debug.Assert(pack.lat_int == (int) -966458113);
                Debug.Assert(pack.type_mask == (ushort)(ushort)32611);
                Debug.Assert(pack.vx == (float) -8.774522E37F);
                Debug.Assert(pack.afz == (float)2.8128238E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2870393710U);
                Debug.Assert(pack.target_system == (byte)(byte)97);
                Debug.Assert(pack.alt == (float)1.0720888E38F);
                Debug.Assert(pack.yaw == (float)1.4071658E38F);
                Debug.Assert(pack.afy == (float) -3.2315833E38F);
                Debug.Assert(pack.afx == (float)1.6686373E38F);
                Debug.Assert(pack.yaw_rate == (float) -2.9559353E38F);
                Debug.Assert(pack.vy == (float) -3.0430995E38F);
                Debug.Assert(pack.vz == (float)1.092876E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.target_component == (byte)(byte)183);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.vz = (float)1.092876E38F;
            p86.lat_int = (int) -966458113;
            p86.yaw_rate = (float) -2.9559353E38F;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p86.type_mask = (ushort)(ushort)32611;
            p86.vy = (float) -3.0430995E38F;
            p86.afx = (float)1.6686373E38F;
            p86.target_component = (byte)(byte)183;
            p86.afz = (float)2.8128238E38F;
            p86.vx = (float) -8.774522E37F;
            p86.yaw = (float)1.4071658E38F;
            p86.time_boot_ms = (uint)2870393710U;
            p86.lon_int = (int) -396585074;
            p86.target_system = (byte)(byte)97;
            p86.afy = (float) -3.2315833E38F;
            p86.alt = (float)1.0720888E38F;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw_rate == (float)5.93816E37F);
                Debug.Assert(pack.lon_int == (int)1048654771);
                Debug.Assert(pack.vz == (float)3.0613054E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.vx == (float) -2.5307468E38F);
                Debug.Assert(pack.afz == (float)2.8819497E38F);
                Debug.Assert(pack.vy == (float) -1.1824151E38F);
                Debug.Assert(pack.afx == (float)2.1933333E38F);
                Debug.Assert(pack.yaw == (float) -2.7746316E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2237024032U);
                Debug.Assert(pack.alt == (float)7.1774747E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)8256);
                Debug.Assert(pack.lat_int == (int) -526459659);
                Debug.Assert(pack.afy == (float) -1.1104037E38F);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.afy = (float) -1.1104037E38F;
            p87.time_boot_ms = (uint)2237024032U;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p87.yaw = (float) -2.7746316E38F;
            p87.lon_int = (int)1048654771;
            p87.yaw_rate = (float)5.93816E37F;
            p87.vx = (float) -2.5307468E38F;
            p87.alt = (float)7.1774747E37F;
            p87.afx = (float)2.1933333E38F;
            p87.vy = (float) -1.1824151E38F;
            p87.vz = (float)3.0613054E38F;
            p87.type_mask = (ushort)(ushort)8256;
            p87.afz = (float)2.8819497E38F;
            p87.lat_int = (int) -526459659;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -1.0554479E38F);
                Debug.Assert(pack.z == (float)2.1183845E38F);
                Debug.Assert(pack.roll == (float) -6.511888E37F);
                Debug.Assert(pack.x == (float) -3.3195522E38F);
                Debug.Assert(pack.yaw == (float) -3.1684746E38F);
                Debug.Assert(pack.pitch == (float) -1.4336457E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3482780020U);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.pitch = (float) -1.4336457E38F;
            p89.roll = (float) -6.511888E37F;
            p89.x = (float) -3.3195522E38F;
            p89.y = (float) -1.0554479E38F;
            p89.z = (float)2.1183845E38F;
            p89.time_boot_ms = (uint)3482780020U;
            p89.yaw = (float) -3.1684746E38F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -2.934169E38F);
                Debug.Assert(pack.xacc == (short)(short) -22022);
                Debug.Assert(pack.zacc == (short)(short) -24202);
                Debug.Assert(pack.yaw == (float)2.8548445E38F);
                Debug.Assert(pack.rollspeed == (float) -2.2945993E38F);
                Debug.Assert(pack.vy == (short)(short) -25098);
                Debug.Assert(pack.lat == (int) -1919840346);
                Debug.Assert(pack.time_usec == (ulong)8168669469533441374L);
                Debug.Assert(pack.alt == (int)1148972641);
                Debug.Assert(pack.vz == (short)(short) -18813);
                Debug.Assert(pack.yawspeed == (float) -1.3544387E38F);
                Debug.Assert(pack.lon == (int)1799486652);
                Debug.Assert(pack.pitchspeed == (float) -2.1510576E38F);
                Debug.Assert(pack.yacc == (short)(short)26659);
                Debug.Assert(pack.vx == (short)(short)6723);
                Debug.Assert(pack.roll == (float) -2.3118292E38F);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.zacc = (short)(short) -24202;
            p90.yaw = (float)2.8548445E38F;
            p90.lon = (int)1799486652;
            p90.yacc = (short)(short)26659;
            p90.xacc = (short)(short) -22022;
            p90.pitchspeed = (float) -2.1510576E38F;
            p90.vx = (short)(short)6723;
            p90.roll = (float) -2.3118292E38F;
            p90.yawspeed = (float) -1.3544387E38F;
            p90.rollspeed = (float) -2.2945993E38F;
            p90.time_usec = (ulong)8168669469533441374L;
            p90.lat = (int) -1919840346;
            p90.vz = (short)(short) -18813;
            p90.vy = (short)(short) -25098;
            p90.pitch = (float) -2.934169E38F;
            p90.alt = (int)1148972641;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw_rudder == (float) -2.3800773E38F);
                Debug.Assert(pack.pitch_elevator == (float) -5.00645E37F);
                Debug.Assert(pack.throttle == (float) -1.4999032E38F);
                Debug.Assert(pack.nav_mode == (byte)(byte)152);
                Debug.Assert(pack.aux1 == (float)3.215254E38F);
                Debug.Assert(pack.aux4 == (float)7.757737E37F);
                Debug.Assert(pack.aux3 == (float)2.3439087E38F);
                Debug.Assert(pack.roll_ailerons == (float) -1.7177422E38F);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_PREFLIGHT);
                Debug.Assert(pack.time_usec == (ulong)2384790147189999973L);
                Debug.Assert(pack.aux2 == (float) -3.3108145E38F);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.nav_mode = (byte)(byte)152;
            p91.time_usec = (ulong)2384790147189999973L;
            p91.aux4 = (float)7.757737E37F;
            p91.aux3 = (float)2.3439087E38F;
            p91.throttle = (float) -1.4999032E38F;
            p91.yaw_rudder = (float) -2.3800773E38F;
            p91.pitch_elevator = (float) -5.00645E37F;
            p91.mode = MAV_MODE.MAV_MODE_PREFLIGHT;
            p91.aux2 = (float) -3.3108145E38F;
            p91.aux1 = (float)3.215254E38F;
            p91.roll_ailerons = (float) -1.7177422E38F;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)41312);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)21791);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)43299);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)8985);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)37278);
                Debug.Assert(pack.time_usec == (ulong)5251863495330583593L);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)58691);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)61644);
                Debug.Assert(pack.rssi == (byte)(byte)242);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)34184);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)19681);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)32235);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)29817);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)35240);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan10_raw = (ushort)(ushort)41312;
            p92.rssi = (byte)(byte)242;
            p92.chan2_raw = (ushort)(ushort)35240;
            p92.time_usec = (ulong)5251863495330583593L;
            p92.chan11_raw = (ushort)(ushort)21791;
            p92.chan4_raw = (ushort)(ushort)58691;
            p92.chan9_raw = (ushort)(ushort)19681;
            p92.chan6_raw = (ushort)(ushort)37278;
            p92.chan12_raw = (ushort)(ushort)61644;
            p92.chan7_raw = (ushort)(ushort)8985;
            p92.chan1_raw = (ushort)(ushort)43299;
            p92.chan5_raw = (ushort)(ushort)29817;
            p92.chan8_raw = (ushort)(ushort)34184;
            p92.chan3_raw = (ushort)(ushort)32235;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)2394420848736026351L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {9.036942E37F, 1.6970042E38F, -2.7766803E38F, 2.6723393E38F, -2.9746928E38F, -1.791147E38F, 1.4597058E38F, 2.2610802E37F, 1.361557E38F, -2.0000286E37F, 2.1757673E38F, -2.06331E38F, 1.0524161E38F, 2.608687E38F, 1.871782E37F, -1.8876325E38F}));
                Debug.Assert(pack.flags == (ulong)5239205122262634093L);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_AUTO_DISARMED);
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.flags = (ulong)5239205122262634093L;
            p93.time_usec = (ulong)2394420848736026351L;
            p93.mode = MAV_MODE.MAV_MODE_AUTO_DISARMED;
            p93.controls_SET(new float[] {9.036942E37F, 1.6970042E38F, -2.7766803E38F, 2.6723393E38F, -2.9746928E38F, -1.791147E38F, 1.4597058E38F, 2.2610802E37F, 1.361557E38F, -2.0000286E37F, 2.1757673E38F, -2.06331E38F, 1.0524161E38F, 2.608687E38F, 1.871782E37F, -1.8876325E38F}, 0) ;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float) -3.2112587E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)255);
                Debug.Assert(pack.flow_comp_m_y == (float) -2.1393502E36F);
                Debug.Assert(pack.flow_comp_m_x == (float)9.488067E37F);
                Debug.Assert(pack.flow_y == (short)(short)1295);
                Debug.Assert(pack.flow_x == (short)(short) -21566);
                Debug.Assert(pack.quality == (byte)(byte)215);
                Debug.Assert(pack.time_usec == (ulong)6406146746199440983L);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)2.4512308E38F);
                Debug.Assert(pack.ground_distance == (float) -1.5438795E38F);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_comp_m_x = (float)9.488067E37F;
            p100.flow_x = (short)(short) -21566;
            p100.quality = (byte)(byte)215;
            p100.flow_y = (short)(short)1295;
            p100.flow_rate_x_SET((float)2.4512308E38F, PH) ;
            p100.sensor_id = (byte)(byte)255;
            p100.time_usec = (ulong)6406146746199440983L;
            p100.flow_rate_y_SET((float) -3.2112587E38F, PH) ;
            p100.ground_distance = (float) -1.5438795E38F;
            p100.flow_comp_m_y = (float) -2.1393502E36F;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)2.0831112E38F);
                Debug.Assert(pack.pitch == (float) -2.9833496E38F);
                Debug.Assert(pack.usec == (ulong)6059580326014628640L);
                Debug.Assert(pack.z == (float)5.215886E37F);
                Debug.Assert(pack.x == (float)3.2995661E38F);
                Debug.Assert(pack.y == (float) -3.2631519E38F);
                Debug.Assert(pack.roll == (float)2.9128039E38F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.pitch = (float) -2.9833496E38F;
            p101.x = (float)3.2995661E38F;
            p101.y = (float) -3.2631519E38F;
            p101.usec = (ulong)6059580326014628640L;
            p101.yaw = (float)2.0831112E38F;
            p101.z = (float)5.215886E37F;
            p101.roll = (float)2.9128039E38F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)1.9739435E38F);
                Debug.Assert(pack.roll == (float) -3.2570045E38F);
                Debug.Assert(pack.usec == (ulong)6907423816434252988L);
                Debug.Assert(pack.y == (float)2.5088053E38F);
                Debug.Assert(pack.yaw == (float)1.3445249E38F);
                Debug.Assert(pack.x == (float)2.6369816E38F);
                Debug.Assert(pack.pitch == (float)1.0865113E38F);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.z = (float)1.9739435E38F;
            p102.yaw = (float)1.3445249E38F;
            p102.roll = (float) -3.2570045E38F;
            p102.y = (float)2.5088053E38F;
            p102.pitch = (float)1.0865113E38F;
            p102.x = (float)2.6369816E38F;
            p102.usec = (ulong)6907423816434252988L;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -2.9894564E38F);
                Debug.Assert(pack.usec == (ulong)3702761213755099312L);
                Debug.Assert(pack.x == (float) -1.0784197E38F);
                Debug.Assert(pack.y == (float)3.31911E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.x = (float) -1.0784197E38F;
            p103.y = (float)3.31911E38F;
            p103.usec = (ulong)3702761213755099312L;
            p103.z = (float) -2.9894564E38F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -3.2297275E38F);
                Debug.Assert(pack.roll == (float) -1.3619503E38F);
                Debug.Assert(pack.yaw == (float) -1.5633203E38F);
                Debug.Assert(pack.pitch == (float)2.0507266E38F);
                Debug.Assert(pack.usec == (ulong)3995598412380973701L);
                Debug.Assert(pack.y == (float) -7.730999E37F);
                Debug.Assert(pack.z == (float) -3.1253552E38F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.z = (float) -3.1253552E38F;
            p104.y = (float) -7.730999E37F;
            p104.x = (float) -3.2297275E38F;
            p104.yaw = (float) -1.5633203E38F;
            p104.roll = (float) -1.3619503E38F;
            p104.pitch = (float)2.0507266E38F;
            p104.usec = (ulong)3995598412380973701L;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (float) -1.240959E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)36996);
                Debug.Assert(pack.xmag == (float)1.5302331E38F);
                Debug.Assert(pack.ymag == (float)3.2908992E38F);
                Debug.Assert(pack.abs_pressure == (float)5.0913224E37F);
                Debug.Assert(pack.zgyro == (float) -3.019171E38F);
                Debug.Assert(pack.temperature == (float)3.1353948E38F);
                Debug.Assert(pack.diff_pressure == (float)1.8753325E38F);
                Debug.Assert(pack.xacc == (float)9.946261E37F);
                Debug.Assert(pack.xgyro == (float)9.717645E37F);
                Debug.Assert(pack.yacc == (float)1.5590847E38F);
                Debug.Assert(pack.pressure_alt == (float)1.3732969E38F);
                Debug.Assert(pack.zmag == (float) -3.4495775E37F);
                Debug.Assert(pack.time_usec == (ulong)168381994706770124L);
                Debug.Assert(pack.ygyro == (float)2.3920015E38F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.fields_updated = (ushort)(ushort)36996;
            p105.zmag = (float) -3.4495775E37F;
            p105.ygyro = (float)2.3920015E38F;
            p105.temperature = (float)3.1353948E38F;
            p105.yacc = (float)1.5590847E38F;
            p105.zgyro = (float) -3.019171E38F;
            p105.zacc = (float) -1.240959E38F;
            p105.xgyro = (float)9.717645E37F;
            p105.diff_pressure = (float)1.8753325E38F;
            p105.xacc = (float)9.946261E37F;
            p105.ymag = (float)3.2908992E38F;
            p105.abs_pressure = (float)5.0913224E37F;
            p105.time_usec = (ulong)168381994706770124L;
            p105.xmag = (float)1.5302331E38F;
            p105.pressure_alt = (float)1.3732969E38F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_id == (byte)(byte)63);
                Debug.Assert(pack.integrated_zgyro == (float) -3.3969574E38F);
                Debug.Assert(pack.temperature == (short)(short) -28651);
                Debug.Assert(pack.distance == (float) -3.2462786E38F);
                Debug.Assert(pack.integration_time_us == (uint)3716867884U);
                Debug.Assert(pack.time_usec == (ulong)4911393758515583538L);
                Debug.Assert(pack.integrated_y == (float)4.9965934E37F);
                Debug.Assert(pack.quality == (byte)(byte)194);
                Debug.Assert(pack.integrated_xgyro == (float)2.1475478E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)1242958441U);
                Debug.Assert(pack.integrated_ygyro == (float)2.4568649E38F);
                Debug.Assert(pack.integrated_x == (float)2.2350754E37F);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.integrated_ygyro = (float)2.4568649E38F;
            p106.temperature = (short)(short) -28651;
            p106.time_usec = (ulong)4911393758515583538L;
            p106.time_delta_distance_us = (uint)1242958441U;
            p106.integration_time_us = (uint)3716867884U;
            p106.quality = (byte)(byte)194;
            p106.integrated_y = (float)4.9965934E37F;
            p106.integrated_xgyro = (float)2.1475478E38F;
            p106.integrated_x = (float)2.2350754E37F;
            p106.sensor_id = (byte)(byte)63;
            p106.integrated_zgyro = (float) -3.3969574E38F;
            p106.distance = (float) -3.2462786E38F;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (float) -2.9862676E38F);
                Debug.Assert(pack.fields_updated == (uint)397119610U);
                Debug.Assert(pack.ygyro == (float) -3.0219675E37F);
                Debug.Assert(pack.xgyro == (float) -1.4587886E38F);
                Debug.Assert(pack.xacc == (float) -1.552484E38F);
                Debug.Assert(pack.pressure_alt == (float) -1.3580045E38F);
                Debug.Assert(pack.xmag == (float) -2.6996045E38F);
                Debug.Assert(pack.zacc == (float)1.2537792E38F);
                Debug.Assert(pack.zgyro == (float)1.0366382E37F);
                Debug.Assert(pack.time_usec == (ulong)1425611979009176677L);
                Debug.Assert(pack.diff_pressure == (float)2.4299039E38F);
                Debug.Assert(pack.ymag == (float) -7.214385E37F);
                Debug.Assert(pack.abs_pressure == (float)7.461165E37F);
                Debug.Assert(pack.yacc == (float) -3.1751593E38F);
                Debug.Assert(pack.zmag == (float)2.762412E38F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.zacc = (float)1.2537792E38F;
            p107.xacc = (float) -1.552484E38F;
            p107.fields_updated = (uint)397119610U;
            p107.temperature = (float) -2.9862676E38F;
            p107.pressure_alt = (float) -1.3580045E38F;
            p107.zgyro = (float)1.0366382E37F;
            p107.xgyro = (float) -1.4587886E38F;
            p107.diff_pressure = (float)2.4299039E38F;
            p107.ymag = (float) -7.214385E37F;
            p107.time_usec = (ulong)1425611979009176677L;
            p107.zmag = (float)2.762412E38F;
            p107.yacc = (float) -3.1751593E38F;
            p107.abs_pressure = (float)7.461165E37F;
            p107.xmag = (float) -2.6996045E38F;
            p107.ygyro = (float) -3.0219675E37F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (float) -2.3769568E38F);
                Debug.Assert(pack.vn == (float) -2.7172454E38F);
                Debug.Assert(pack.q2 == (float)2.5790053E38F);
                Debug.Assert(pack.alt == (float) -2.4406111E38F);
                Debug.Assert(pack.yaw == (float) -1.8516314E37F);
                Debug.Assert(pack.std_dev_vert == (float)2.3680405E37F);
                Debug.Assert(pack.xgyro == (float) -2.4496409E38F);
                Debug.Assert(pack.lat == (float) -1.7163744E38F);
                Debug.Assert(pack.std_dev_horz == (float)8.3261533E37F);
                Debug.Assert(pack.ve == (float) -2.3017018E38F);
                Debug.Assert(pack.xacc == (float)1.7394207E38F);
                Debug.Assert(pack.q4 == (float) -2.209622E37F);
                Debug.Assert(pack.lon == (float)2.905597E38F);
                Debug.Assert(pack.roll == (float)6.4409144E37F);
                Debug.Assert(pack.zgyro == (float) -9.262537E36F);
                Debug.Assert(pack.vd == (float)5.074075E36F);
                Debug.Assert(pack.yacc == (float)2.249445E38F);
                Debug.Assert(pack.pitch == (float)1.5759764E37F);
                Debug.Assert(pack.zacc == (float) -2.8274584E38F);
                Debug.Assert(pack.q1 == (float) -1.3729154E38F);
                Debug.Assert(pack.q3 == (float) -1.7353358E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.q4 = (float) -2.209622E37F;
            p108.pitch = (float)1.5759764E37F;
            p108.xgyro = (float) -2.4496409E38F;
            p108.zacc = (float) -2.8274584E38F;
            p108.q2 = (float)2.5790053E38F;
            p108.vn = (float) -2.7172454E38F;
            p108.vd = (float)5.074075E36F;
            p108.ygyro = (float) -2.3769568E38F;
            p108.roll = (float)6.4409144E37F;
            p108.yacc = (float)2.249445E38F;
            p108.lat = (float) -1.7163744E38F;
            p108.q3 = (float) -1.7353358E38F;
            p108.alt = (float) -2.4406111E38F;
            p108.zgyro = (float) -9.262537E36F;
            p108.ve = (float) -2.3017018E38F;
            p108.std_dev_vert = (float)2.3680405E37F;
            p108.yaw = (float) -1.8516314E37F;
            p108.std_dev_horz = (float)8.3261533E37F;
            p108.xacc = (float)1.7394207E38F;
            p108.q1 = (float) -1.3729154E38F;
            p108.lon = (float)2.905597E38F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fixed_ == (ushort)(ushort)28810);
                Debug.Assert(pack.noise == (byte)(byte)187);
                Debug.Assert(pack.remnoise == (byte)(byte)204);
                Debug.Assert(pack.txbuf == (byte)(byte)140);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)62505);
                Debug.Assert(pack.rssi == (byte)(byte)183);
                Debug.Assert(pack.remrssi == (byte)(byte)226);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rxerrors = (ushort)(ushort)62505;
            p109.remnoise = (byte)(byte)204;
            p109.fixed_ = (ushort)(ushort)28810;
            p109.rssi = (byte)(byte)183;
            p109.noise = (byte)(byte)187;
            p109.remrssi = (byte)(byte)226;
            p109.txbuf = (byte)(byte)140;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_network == (byte)(byte)165);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)137, (byte)236, (byte)186, (byte)5, (byte)7, (byte)184, (byte)154, (byte)207, (byte)190, (byte)130, (byte)214, (byte)132, (byte)96, (byte)48, (byte)75, (byte)79, (byte)188, (byte)63, (byte)173, (byte)25, (byte)226, (byte)45, (byte)7, (byte)158, (byte)113, (byte)199, (byte)126, (byte)130, (byte)206, (byte)44, (byte)234, (byte)229, (byte)74, (byte)70, (byte)148, (byte)184, (byte)57, (byte)210, (byte)178, (byte)181, (byte)57, (byte)138, (byte)250, (byte)28, (byte)97, (byte)148, (byte)29, (byte)64, (byte)103, (byte)12, (byte)254, (byte)32, (byte)145, (byte)149, (byte)155, (byte)55, (byte)211, (byte)106, (byte)11, (byte)137, (byte)120, (byte)12, (byte)185, (byte)199, (byte)30, (byte)49, (byte)242, (byte)111, (byte)132, (byte)32, (byte)74, (byte)221, (byte)255, (byte)197, (byte)68, (byte)198, (byte)24, (byte)176, (byte)104, (byte)214, (byte)180, (byte)154, (byte)170, (byte)103, (byte)181, (byte)207, (byte)198, (byte)65, (byte)102, (byte)26, (byte)131, (byte)26, (byte)222, (byte)13, (byte)194, (byte)255, (byte)232, (byte)164, (byte)221, (byte)113, (byte)222, (byte)91, (byte)246, (byte)25, (byte)89, (byte)112, (byte)23, (byte)104, (byte)10, (byte)177, (byte)239, (byte)148, (byte)216, (byte)233, (byte)59, (byte)64, (byte)180, (byte)159, (byte)185, (byte)10, (byte)229, (byte)220, (byte)107, (byte)113, (byte)79, (byte)123, (byte)207, (byte)4, (byte)131, (byte)173, (byte)136, (byte)119, (byte)176, (byte)239, (byte)35, (byte)92, (byte)199, (byte)0, (byte)131, (byte)115, (byte)239, (byte)80, (byte)251, (byte)98, (byte)48, (byte)127, (byte)49, (byte)60, (byte)57, (byte)239, (byte)149, (byte)154, (byte)165, (byte)88, (byte)178, (byte)143, (byte)54, (byte)83, (byte)188, (byte)112, (byte)101, (byte)97, (byte)237, (byte)8, (byte)26, (byte)247, (byte)169, (byte)17, (byte)56, (byte)71, (byte)172, (byte)196, (byte)126, (byte)251, (byte)90, (byte)211, (byte)44, (byte)180, (byte)71, (byte)153, (byte)231, (byte)169, (byte)88, (byte)28, (byte)166, (byte)65, (byte)84, (byte)93, (byte)12, (byte)158, (byte)225, (byte)174, (byte)173, (byte)150, (byte)95, (byte)183, (byte)98, (byte)236, (byte)78, (byte)32, (byte)205, (byte)67, (byte)140, (byte)114, (byte)247, (byte)91, (byte)162, (byte)150, (byte)239, (byte)65, (byte)156, (byte)144, (byte)149, (byte)82, (byte)137, (byte)124, (byte)104, (byte)132, (byte)56, (byte)43, (byte)172, (byte)126, (byte)246, (byte)12, (byte)138, (byte)191, (byte)214, (byte)231, (byte)247, (byte)161, (byte)137, (byte)154, (byte)18, (byte)32, (byte)65, (byte)32, (byte)226, (byte)55, (byte)73, (byte)93, (byte)116, (byte)96, (byte)67, (byte)154, (byte)155, (byte)107, (byte)244, (byte)250, (byte)189, (byte)171, (byte)246}));
                Debug.Assert(pack.target_component == (byte)(byte)98);
                Debug.Assert(pack.target_system == (byte)(byte)70);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_system = (byte)(byte)70;
            p110.target_network = (byte)(byte)165;
            p110.payload_SET(new byte[] {(byte)137, (byte)236, (byte)186, (byte)5, (byte)7, (byte)184, (byte)154, (byte)207, (byte)190, (byte)130, (byte)214, (byte)132, (byte)96, (byte)48, (byte)75, (byte)79, (byte)188, (byte)63, (byte)173, (byte)25, (byte)226, (byte)45, (byte)7, (byte)158, (byte)113, (byte)199, (byte)126, (byte)130, (byte)206, (byte)44, (byte)234, (byte)229, (byte)74, (byte)70, (byte)148, (byte)184, (byte)57, (byte)210, (byte)178, (byte)181, (byte)57, (byte)138, (byte)250, (byte)28, (byte)97, (byte)148, (byte)29, (byte)64, (byte)103, (byte)12, (byte)254, (byte)32, (byte)145, (byte)149, (byte)155, (byte)55, (byte)211, (byte)106, (byte)11, (byte)137, (byte)120, (byte)12, (byte)185, (byte)199, (byte)30, (byte)49, (byte)242, (byte)111, (byte)132, (byte)32, (byte)74, (byte)221, (byte)255, (byte)197, (byte)68, (byte)198, (byte)24, (byte)176, (byte)104, (byte)214, (byte)180, (byte)154, (byte)170, (byte)103, (byte)181, (byte)207, (byte)198, (byte)65, (byte)102, (byte)26, (byte)131, (byte)26, (byte)222, (byte)13, (byte)194, (byte)255, (byte)232, (byte)164, (byte)221, (byte)113, (byte)222, (byte)91, (byte)246, (byte)25, (byte)89, (byte)112, (byte)23, (byte)104, (byte)10, (byte)177, (byte)239, (byte)148, (byte)216, (byte)233, (byte)59, (byte)64, (byte)180, (byte)159, (byte)185, (byte)10, (byte)229, (byte)220, (byte)107, (byte)113, (byte)79, (byte)123, (byte)207, (byte)4, (byte)131, (byte)173, (byte)136, (byte)119, (byte)176, (byte)239, (byte)35, (byte)92, (byte)199, (byte)0, (byte)131, (byte)115, (byte)239, (byte)80, (byte)251, (byte)98, (byte)48, (byte)127, (byte)49, (byte)60, (byte)57, (byte)239, (byte)149, (byte)154, (byte)165, (byte)88, (byte)178, (byte)143, (byte)54, (byte)83, (byte)188, (byte)112, (byte)101, (byte)97, (byte)237, (byte)8, (byte)26, (byte)247, (byte)169, (byte)17, (byte)56, (byte)71, (byte)172, (byte)196, (byte)126, (byte)251, (byte)90, (byte)211, (byte)44, (byte)180, (byte)71, (byte)153, (byte)231, (byte)169, (byte)88, (byte)28, (byte)166, (byte)65, (byte)84, (byte)93, (byte)12, (byte)158, (byte)225, (byte)174, (byte)173, (byte)150, (byte)95, (byte)183, (byte)98, (byte)236, (byte)78, (byte)32, (byte)205, (byte)67, (byte)140, (byte)114, (byte)247, (byte)91, (byte)162, (byte)150, (byte)239, (byte)65, (byte)156, (byte)144, (byte)149, (byte)82, (byte)137, (byte)124, (byte)104, (byte)132, (byte)56, (byte)43, (byte)172, (byte)126, (byte)246, (byte)12, (byte)138, (byte)191, (byte)214, (byte)231, (byte)247, (byte)161, (byte)137, (byte)154, (byte)18, (byte)32, (byte)65, (byte)32, (byte)226, (byte)55, (byte)73, (byte)93, (byte)116, (byte)96, (byte)67, (byte)154, (byte)155, (byte)107, (byte)244, (byte)250, (byte)189, (byte)171, (byte)246}, 0) ;
            p110.target_component = (byte)(byte)98;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ts1 == (long)8844440891249809955L);
                Debug.Assert(pack.tc1 == (long)7263129656246275874L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long)7263129656246275874L;
            p111.ts1 = (long)8844440891249809955L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)3144574508U);
                Debug.Assert(pack.time_usec == (ulong)7415894299925578765L);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)7415894299925578765L;
            p112.seq = (uint)3144574508U;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vd == (short)(short) -6767);
                Debug.Assert(pack.vel == (ushort)(ushort)6822);
                Debug.Assert(pack.cog == (ushort)(ushort)55438);
                Debug.Assert(pack.lon == (int)753889819);
                Debug.Assert(pack.fix_type == (byte)(byte)229);
                Debug.Assert(pack.eph == (ushort)(ushort)59172);
                Debug.Assert(pack.vn == (short)(short)27264);
                Debug.Assert(pack.epv == (ushort)(ushort)86);
                Debug.Assert(pack.alt == (int) -1496086833);
                Debug.Assert(pack.satellites_visible == (byte)(byte)24);
                Debug.Assert(pack.time_usec == (ulong)572463035660906218L);
                Debug.Assert(pack.ve == (short)(short) -31231);
                Debug.Assert(pack.lat == (int)837933104);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.cog = (ushort)(ushort)55438;
            p113.alt = (int) -1496086833;
            p113.vn = (short)(short)27264;
            p113.satellites_visible = (byte)(byte)24;
            p113.vd = (short)(short) -6767;
            p113.ve = (short)(short) -31231;
            p113.eph = (ushort)(ushort)59172;
            p113.epv = (ushort)(ushort)86;
            p113.vel = (ushort)(ushort)6822;
            p113.fix_type = (byte)(byte)229;
            p113.lon = (int)753889819;
            p113.time_usec = (ulong)572463035660906218L;
            p113.lat = (int)837933104;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_y == (float) -1.0026312E38F);
                Debug.Assert(pack.integrated_xgyro == (float) -2.8983502E38F);
                Debug.Assert(pack.integrated_x == (float) -3.5788283E36F);
                Debug.Assert(pack.integrated_zgyro == (float)1.2620152E38F);
                Debug.Assert(pack.distance == (float) -1.0768913E38F);
                Debug.Assert(pack.quality == (byte)(byte)178);
                Debug.Assert(pack.sensor_id == (byte)(byte)51);
                Debug.Assert(pack.temperature == (short)(short) -30117);
                Debug.Assert(pack.time_usec == (ulong)3704260441351715497L);
                Debug.Assert(pack.integrated_ygyro == (float)2.8061278E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)1153421923U);
                Debug.Assert(pack.integration_time_us == (uint)2627724847U);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.integrated_y = (float) -1.0026312E38F;
            p114.integrated_x = (float) -3.5788283E36F;
            p114.integrated_zgyro = (float)1.2620152E38F;
            p114.integration_time_us = (uint)2627724847U;
            p114.distance = (float) -1.0768913E38F;
            p114.integrated_ygyro = (float)2.8061278E38F;
            p114.time_usec = (ulong)3704260441351715497L;
            p114.time_delta_distance_us = (uint)1153421923U;
            p114.sensor_id = (byte)(byte)51;
            p114.temperature = (short)(short) -30117;
            p114.integrated_xgyro = (float) -2.8983502E38F;
            p114.quality = (byte)(byte)178;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (short)(short) -10209);
                Debug.Assert(pack.vz == (short)(short)845);
                Debug.Assert(pack.alt == (int) -708871419);
                Debug.Assert(pack.yacc == (short)(short)2992);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {-1.1404063E38F, -3.0106805E38F, 1.4427985E38F, 1.2867501E38F}));
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)33280);
                Debug.Assert(pack.yawspeed == (float) -2.356347E38F);
                Debug.Assert(pack.vx == (short)(short)18669);
                Debug.Assert(pack.lat == (int) -1888569487);
                Debug.Assert(pack.zacc == (short)(short)11434);
                Debug.Assert(pack.pitchspeed == (float) -3.0557012E38F);
                Debug.Assert(pack.xacc == (short)(short) -6758);
                Debug.Assert(pack.rollspeed == (float)3.6988323E37F);
                Debug.Assert(pack.lon == (int) -370865388);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)22086);
                Debug.Assert(pack.time_usec == (ulong)8451774823121882832L);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.alt = (int) -708871419;
            p115.ind_airspeed = (ushort)(ushort)22086;
            p115.attitude_quaternion_SET(new float[] {-1.1404063E38F, -3.0106805E38F, 1.4427985E38F, 1.2867501E38F}, 0) ;
            p115.vz = (short)(short)845;
            p115.yacc = (short)(short)2992;
            p115.pitchspeed = (float) -3.0557012E38F;
            p115.xacc = (short)(short) -6758;
            p115.rollspeed = (float)3.6988323E37F;
            p115.zacc = (short)(short)11434;
            p115.lon = (int) -370865388;
            p115.vx = (short)(short)18669;
            p115.time_usec = (ulong)8451774823121882832L;
            p115.lat = (int) -1888569487;
            p115.true_airspeed = (ushort)(ushort)33280;
            p115.vy = (short)(short) -10209;
            p115.yawspeed = (float) -2.356347E38F;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (short)(short) -16405);
                Debug.Assert(pack.ygyro == (short)(short)29932);
                Debug.Assert(pack.zmag == (short)(short)2309);
                Debug.Assert(pack.zacc == (short)(short)4298);
                Debug.Assert(pack.xmag == (short)(short) -31562);
                Debug.Assert(pack.ymag == (short)(short)30631);
                Debug.Assert(pack.xacc == (short)(short) -21578);
                Debug.Assert(pack.xgyro == (short)(short)23757);
                Debug.Assert(pack.yacc == (short)(short) -10925);
                Debug.Assert(pack.time_boot_ms == (uint)3882711342U);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.xacc = (short)(short) -21578;
            p116.time_boot_ms = (uint)3882711342U;
            p116.ygyro = (short)(short)29932;
            p116.xgyro = (short)(short)23757;
            p116.ymag = (short)(short)30631;
            p116.zmag = (short)(short)2309;
            p116.yacc = (short)(short) -10925;
            p116.zacc = (short)(short)4298;
            p116.zgyro = (short)(short) -16405;
            p116.xmag = (short)(short) -31562;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end == (ushort)(ushort)50264);
                Debug.Assert(pack.target_component == (byte)(byte)133);
                Debug.Assert(pack.target_system == (byte)(byte)233);
                Debug.Assert(pack.start == (ushort)(ushort)56815);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.start = (ushort)(ushort)56815;
            p117.target_component = (byte)(byte)133;
            p117.end = (ushort)(ushort)50264;
            p117.target_system = (byte)(byte)233;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_utc == (uint)729349689U);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)5678);
                Debug.Assert(pack.size == (uint)2814985046U);
                Debug.Assert(pack.id == (ushort)(ushort)35780);
                Debug.Assert(pack.num_logs == (ushort)(ushort)17079);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.last_log_num = (ushort)(ushort)5678;
            p118.size = (uint)2814985046U;
            p118.time_utc = (uint)729349689U;
            p118.num_logs = (ushort)(ushort)17079;
            p118.id = (ushort)(ushort)35780;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (uint)1996068761U);
                Debug.Assert(pack.target_component == (byte)(byte)223);
                Debug.Assert(pack.target_system == (byte)(byte)113);
                Debug.Assert(pack.ofs == (uint)3178945777U);
                Debug.Assert(pack.id == (ushort)(ushort)24622);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.count = (uint)1996068761U;
            p119.target_component = (byte)(byte)223;
            p119.id = (ushort)(ushort)24622;
            p119.ofs = (uint)3178945777U;
            p119.target_system = (byte)(byte)113;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)75, (byte)226, (byte)205, (byte)98, (byte)255, (byte)220, (byte)77, (byte)59, (byte)169, (byte)208, (byte)206, (byte)221, (byte)16, (byte)214, (byte)210, (byte)25, (byte)247, (byte)151, (byte)131, (byte)137, (byte)135, (byte)19, (byte)53, (byte)252, (byte)150, (byte)204, (byte)26, (byte)125, (byte)203, (byte)162, (byte)244, (byte)41, (byte)211, (byte)176, (byte)208, (byte)20, (byte)19, (byte)216, (byte)65, (byte)167, (byte)176, (byte)151, (byte)40, (byte)106, (byte)97, (byte)23, (byte)191, (byte)141, (byte)164, (byte)108, (byte)177, (byte)128, (byte)60, (byte)72, (byte)30, (byte)79, (byte)62, (byte)106, (byte)147, (byte)249, (byte)197, (byte)95, (byte)51, (byte)85, (byte)167, (byte)235, (byte)182, (byte)94, (byte)109, (byte)54, (byte)13, (byte)120, (byte)161, (byte)202, (byte)232, (byte)76, (byte)97, (byte)175, (byte)109, (byte)45, (byte)107, (byte)120, (byte)144, (byte)13, (byte)86, (byte)151, (byte)230, (byte)107, (byte)98, (byte)166}));
                Debug.Assert(pack.id == (ushort)(ushort)52513);
                Debug.Assert(pack.count == (byte)(byte)46);
                Debug.Assert(pack.ofs == (uint)1209273667U);
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)52513;
            p120.data__SET(new byte[] {(byte)75, (byte)226, (byte)205, (byte)98, (byte)255, (byte)220, (byte)77, (byte)59, (byte)169, (byte)208, (byte)206, (byte)221, (byte)16, (byte)214, (byte)210, (byte)25, (byte)247, (byte)151, (byte)131, (byte)137, (byte)135, (byte)19, (byte)53, (byte)252, (byte)150, (byte)204, (byte)26, (byte)125, (byte)203, (byte)162, (byte)244, (byte)41, (byte)211, (byte)176, (byte)208, (byte)20, (byte)19, (byte)216, (byte)65, (byte)167, (byte)176, (byte)151, (byte)40, (byte)106, (byte)97, (byte)23, (byte)191, (byte)141, (byte)164, (byte)108, (byte)177, (byte)128, (byte)60, (byte)72, (byte)30, (byte)79, (byte)62, (byte)106, (byte)147, (byte)249, (byte)197, (byte)95, (byte)51, (byte)85, (byte)167, (byte)235, (byte)182, (byte)94, (byte)109, (byte)54, (byte)13, (byte)120, (byte)161, (byte)202, (byte)232, (byte)76, (byte)97, (byte)175, (byte)109, (byte)45, (byte)107, (byte)120, (byte)144, (byte)13, (byte)86, (byte)151, (byte)230, (byte)107, (byte)98, (byte)166}, 0) ;
            p120.ofs = (uint)1209273667U;
            p120.count = (byte)(byte)46;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)111);
                Debug.Assert(pack.target_system == (byte)(byte)118);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)118;
            p121.target_component = (byte)(byte)111;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)242);
                Debug.Assert(pack.target_system == (byte)(byte)132);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_component = (byte)(byte)242;
            p122.target_system = (byte)(byte)132;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)90);
                Debug.Assert(pack.target_component == (byte)(byte)84);
                Debug.Assert(pack.target_system == (byte)(byte)56);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)131, (byte)75, (byte)208, (byte)26, (byte)77, (byte)109, (byte)136, (byte)119, (byte)190, (byte)12, (byte)235, (byte)139, (byte)150, (byte)245, (byte)228, (byte)103, (byte)226, (byte)196, (byte)74, (byte)252, (byte)251, (byte)101, (byte)230, (byte)105, (byte)150, (byte)35, (byte)1, (byte)119, (byte)109, (byte)132, (byte)40, (byte)136, (byte)52, (byte)59, (byte)121, (byte)130, (byte)136, (byte)1, (byte)39, (byte)177, (byte)10, (byte)111, (byte)102, (byte)160, (byte)237, (byte)151, (byte)251, (byte)82, (byte)103, (byte)34, (byte)57, (byte)167, (byte)122, (byte)225, (byte)254, (byte)233, (byte)74, (byte)161, (byte)69, (byte)236, (byte)105, (byte)254, (byte)242, (byte)226, (byte)32, (byte)74, (byte)186, (byte)149, (byte)176, (byte)60, (byte)192, (byte)236, (byte)36, (byte)251, (byte)115, (byte)147, (byte)55, (byte)205, (byte)189, (byte)145, (byte)209, (byte)74, (byte)79, (byte)217, (byte)134, (byte)235, (byte)120, (byte)193, (byte)109, (byte)23, (byte)201, (byte)89, (byte)76, (byte)226, (byte)235, (byte)122, (byte)189, (byte)66, (byte)66, (byte)132, (byte)226, (byte)61, (byte)2, (byte)15, (byte)47, (byte)237, (byte)182, (byte)155, (byte)23, (byte)226}));
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.data__SET(new byte[] {(byte)131, (byte)75, (byte)208, (byte)26, (byte)77, (byte)109, (byte)136, (byte)119, (byte)190, (byte)12, (byte)235, (byte)139, (byte)150, (byte)245, (byte)228, (byte)103, (byte)226, (byte)196, (byte)74, (byte)252, (byte)251, (byte)101, (byte)230, (byte)105, (byte)150, (byte)35, (byte)1, (byte)119, (byte)109, (byte)132, (byte)40, (byte)136, (byte)52, (byte)59, (byte)121, (byte)130, (byte)136, (byte)1, (byte)39, (byte)177, (byte)10, (byte)111, (byte)102, (byte)160, (byte)237, (byte)151, (byte)251, (byte)82, (byte)103, (byte)34, (byte)57, (byte)167, (byte)122, (byte)225, (byte)254, (byte)233, (byte)74, (byte)161, (byte)69, (byte)236, (byte)105, (byte)254, (byte)242, (byte)226, (byte)32, (byte)74, (byte)186, (byte)149, (byte)176, (byte)60, (byte)192, (byte)236, (byte)36, (byte)251, (byte)115, (byte)147, (byte)55, (byte)205, (byte)189, (byte)145, (byte)209, (byte)74, (byte)79, (byte)217, (byte)134, (byte)235, (byte)120, (byte)193, (byte)109, (byte)23, (byte)201, (byte)89, (byte)76, (byte)226, (byte)235, (byte)122, (byte)189, (byte)66, (byte)66, (byte)132, (byte)226, (byte)61, (byte)2, (byte)15, (byte)47, (byte)237, (byte)182, (byte)155, (byte)23, (byte)226}, 0) ;
            p123.target_component = (byte)(byte)84;
            p123.target_system = (byte)(byte)56;
            p123.len = (byte)(byte)90;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellites_visible == (byte)(byte)134);
                Debug.Assert(pack.alt == (int) -898295027);
                Debug.Assert(pack.lon == (int)75301191);
                Debug.Assert(pack.time_usec == (ulong)3950809047946986519L);
                Debug.Assert(pack.eph == (ushort)(ushort)20077);
                Debug.Assert(pack.epv == (ushort)(ushort)40131);
                Debug.Assert(pack.lat == (int)765622760);
                Debug.Assert(pack.cog == (ushort)(ushort)38930);
                Debug.Assert(pack.dgps_age == (uint)4125717389U);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC);
                Debug.Assert(pack.vel == (ushort)(ushort)21069);
                Debug.Assert(pack.dgps_numch == (byte)(byte)92);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.lat = (int)765622760;
            p124.epv = (ushort)(ushort)40131;
            p124.alt = (int) -898295027;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_STATIC;
            p124.lon = (int)75301191;
            p124.eph = (ushort)(ushort)20077;
            p124.dgps_numch = (byte)(byte)92;
            p124.vel = (ushort)(ushort)21069;
            p124.satellites_visible = (byte)(byte)134;
            p124.cog = (ushort)(ushort)38930;
            p124.time_usec = (ulong)3950809047946986519L;
            p124.dgps_age = (uint)4125717389U;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT));
                Debug.Assert(pack.Vcc == (ushort)(ushort)63476);
                Debug.Assert(pack.Vservo == (ushort)(ushort)29810);
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)63476;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT);
            p125.Vservo = (ushort)(ushort)29810;
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baudrate == (uint)3116908486U);
                Debug.Assert(pack.timeout == (ushort)(ushort)36636);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)16, (byte)186, (byte)238, (byte)169, (byte)246, (byte)119, (byte)236, (byte)247, (byte)129, (byte)232, (byte)217, (byte)132, (byte)176, (byte)246, (byte)243, (byte)131, (byte)145, (byte)113, (byte)149, (byte)163, (byte)179, (byte)227, (byte)162, (byte)239, (byte)154, (byte)114, (byte)254, (byte)108, (byte)104, (byte)29, (byte)27, (byte)193, (byte)68, (byte)57, (byte)27, (byte)54, (byte)13, (byte)129, (byte)67, (byte)25, (byte)50, (byte)46, (byte)150, (byte)163, (byte)175, (byte)219, (byte)1, (byte)49, (byte)146, (byte)250, (byte)24, (byte)192, (byte)227, (byte)115, (byte)146, (byte)201, (byte)32, (byte)197, (byte)82, (byte)226, (byte)27, (byte)63, (byte)99, (byte)221, (byte)131, (byte)250, (byte)196, (byte)190, (byte)26, (byte)170}));
                Debug.Assert(pack.device == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                                            SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                            SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING));
                Debug.Assert(pack.count == (byte)(byte)105);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.timeout = (ushort)(ushort)36636;
            p126.count = (byte)(byte)105;
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING);
            p126.baudrate = (uint)3116908486U;
            p126.data__SET(new byte[] {(byte)16, (byte)186, (byte)238, (byte)169, (byte)246, (byte)119, (byte)236, (byte)247, (byte)129, (byte)232, (byte)217, (byte)132, (byte)176, (byte)246, (byte)243, (byte)131, (byte)145, (byte)113, (byte)149, (byte)163, (byte)179, (byte)227, (byte)162, (byte)239, (byte)154, (byte)114, (byte)254, (byte)108, (byte)104, (byte)29, (byte)27, (byte)193, (byte)68, (byte)57, (byte)27, (byte)54, (byte)13, (byte)129, (byte)67, (byte)25, (byte)50, (byte)46, (byte)150, (byte)163, (byte)175, (byte)219, (byte)1, (byte)49, (byte)146, (byte)250, (byte)24, (byte)192, (byte)227, (byte)115, (byte)146, (byte)201, (byte)32, (byte)197, (byte)82, (byte)226, (byte)27, (byte)63, (byte)99, (byte)221, (byte)131, (byte)250, (byte)196, (byte)190, (byte)26, (byte)170}, 0) ;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_a_mm == (int)1351401153);
                Debug.Assert(pack.wn == (ushort)(ushort)31037);
                Debug.Assert(pack.time_last_baseline_ms == (uint)293410106U);
                Debug.Assert(pack.rtk_health == (byte)(byte)203);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)183);
                Debug.Assert(pack.accuracy == (uint)3393332408U);
                Debug.Assert(pack.iar_num_hypotheses == (int)1212228829);
                Debug.Assert(pack.tow == (uint)1948887592U);
                Debug.Assert(pack.baseline_b_mm == (int)1080252504);
                Debug.Assert(pack.baseline_c_mm == (int)1181884316);
                Debug.Assert(pack.nsats == (byte)(byte)57);
                Debug.Assert(pack.rtk_rate == (byte)(byte)163);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)10);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.tow = (uint)1948887592U;
            p127.wn = (ushort)(ushort)31037;
            p127.accuracy = (uint)3393332408U;
            p127.rtk_receiver_id = (byte)(byte)183;
            p127.baseline_c_mm = (int)1181884316;
            p127.time_last_baseline_ms = (uint)293410106U;
            p127.baseline_a_mm = (int)1351401153;
            p127.nsats = (byte)(byte)57;
            p127.iar_num_hypotheses = (int)1212228829;
            p127.rtk_health = (byte)(byte)203;
            p127.baseline_coords_type = (byte)(byte)10;
            p127.rtk_rate = (byte)(byte)163;
            p127.baseline_b_mm = (int)1080252504;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.iar_num_hypotheses == (int) -1201052361);
                Debug.Assert(pack.baseline_b_mm == (int) -1199495413);
                Debug.Assert(pack.wn == (ushort)(ushort)24874);
                Debug.Assert(pack.rtk_health == (byte)(byte)141);
                Debug.Assert(pack.baseline_c_mm == (int) -286917596);
                Debug.Assert(pack.nsats == (byte)(byte)129);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)175);
                Debug.Assert(pack.baseline_a_mm == (int) -1009391532);
                Debug.Assert(pack.rtk_rate == (byte)(byte)57);
                Debug.Assert(pack.tow == (uint)3435298512U);
                Debug.Assert(pack.accuracy == (uint)4291311565U);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)189);
                Debug.Assert(pack.time_last_baseline_ms == (uint)3489676327U);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.baseline_a_mm = (int) -1009391532;
            p128.baseline_coords_type = (byte)(byte)189;
            p128.baseline_b_mm = (int) -1199495413;
            p128.time_last_baseline_ms = (uint)3489676327U;
            p128.rtk_rate = (byte)(byte)57;
            p128.rtk_receiver_id = (byte)(byte)175;
            p128.wn = (ushort)(ushort)24874;
            p128.rtk_health = (byte)(byte)141;
            p128.accuracy = (uint)4291311565U;
            p128.nsats = (byte)(byte)129;
            p128.iar_num_hypotheses = (int) -1201052361;
            p128.tow = (uint)3435298512U;
            p128.baseline_c_mm = (int) -286917596;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ymag == (short)(short)14647);
                Debug.Assert(pack.zacc == (short)(short) -7403);
                Debug.Assert(pack.xgyro == (short)(short)14117);
                Debug.Assert(pack.time_boot_ms == (uint)2980695536U);
                Debug.Assert(pack.xmag == (short)(short)2571);
                Debug.Assert(pack.ygyro == (short)(short)6083);
                Debug.Assert(pack.zgyro == (short)(short)21555);
                Debug.Assert(pack.yacc == (short)(short)1928);
                Debug.Assert(pack.xacc == (short)(short)27383);
                Debug.Assert(pack.zmag == (short)(short)6768);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.xmag = (short)(short)2571;
            p129.xacc = (short)(short)27383;
            p129.time_boot_ms = (uint)2980695536U;
            p129.xgyro = (short)(short)14117;
            p129.ymag = (short)(short)14647;
            p129.yacc = (short)(short)1928;
            p129.zacc = (short)(short) -7403;
            p129.zgyro = (short)(short)21555;
            p129.ygyro = (short)(short)6083;
            p129.zmag = (short)(short)6768;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)138);
                Debug.Assert(pack.payload == (byte)(byte)249);
                Debug.Assert(pack.packets == (ushort)(ushort)26917);
                Debug.Assert(pack.size == (uint)3526400905U);
                Debug.Assert(pack.jpg_quality == (byte)(byte)97);
                Debug.Assert(pack.height == (ushort)(ushort)2242);
                Debug.Assert(pack.width == (ushort)(ushort)48919);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.size = (uint)3526400905U;
            p130.payload = (byte)(byte)249;
            p130.height = (ushort)(ushort)2242;
            p130.packets = (ushort)(ushort)26917;
            p130.type = (byte)(byte)138;
            p130.jpg_quality = (byte)(byte)97;
            p130.width = (ushort)(ushort)48919;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)152, (byte)230, (byte)137, (byte)168, (byte)187, (byte)232, (byte)182, (byte)10, (byte)196, (byte)237, (byte)38, (byte)91, (byte)10, (byte)183, (byte)59, (byte)89, (byte)218, (byte)77, (byte)137, (byte)34, (byte)33, (byte)159, (byte)198, (byte)194, (byte)196, (byte)137, (byte)98, (byte)112, (byte)3, (byte)202, (byte)145, (byte)101, (byte)61, (byte)88, (byte)203, (byte)27, (byte)164, (byte)55, (byte)202, (byte)108, (byte)2, (byte)180, (byte)64, (byte)59, (byte)202, (byte)229, (byte)185, (byte)164, (byte)98, (byte)241, (byte)81, (byte)173, (byte)216, (byte)104, (byte)179, (byte)207, (byte)188, (byte)128, (byte)11, (byte)100, (byte)75, (byte)159, (byte)54, (byte)16, (byte)115, (byte)94, (byte)251, (byte)5, (byte)161, (byte)190, (byte)15, (byte)29, (byte)51, (byte)154, (byte)251, (byte)138, (byte)208, (byte)210, (byte)174, (byte)116, (byte)109, (byte)1, (byte)120, (byte)51, (byte)52, (byte)4, (byte)98, (byte)108, (byte)197, (byte)150, (byte)207, (byte)158, (byte)182, (byte)1, (byte)56, (byte)89, (byte)105, (byte)131, (byte)206, (byte)120, (byte)234, (byte)242, (byte)10, (byte)172, (byte)124, (byte)50, (byte)252, (byte)89, (byte)60, (byte)217, (byte)13, (byte)45, (byte)81, (byte)140, (byte)76, (byte)7, (byte)108, (byte)222, (byte)31, (byte)247, (byte)119, (byte)13, (byte)210, (byte)66, (byte)36, (byte)116, (byte)231, (byte)32, (byte)124, (byte)43, (byte)194, (byte)89, (byte)248, (byte)151, (byte)33, (byte)213, (byte)116, (byte)87, (byte)227, (byte)218, (byte)126, (byte)2, (byte)226, (byte)99, (byte)190, (byte)214, (byte)29, (byte)13, (byte)108, (byte)205, (byte)100, (byte)238, (byte)172, (byte)75, (byte)89, (byte)28, (byte)224, (byte)242, (byte)52, (byte)17, (byte)104, (byte)200, (byte)101, (byte)175, (byte)212, (byte)90, (byte)224, (byte)12, (byte)208, (byte)117, (byte)63, (byte)255, (byte)188, (byte)202, (byte)184, (byte)203, (byte)198, (byte)60, (byte)144, (byte)128, (byte)243, (byte)0, (byte)89, (byte)203, (byte)143, (byte)146, (byte)132, (byte)54, (byte)130, (byte)50, (byte)81, (byte)195, (byte)236, (byte)208, (byte)144, (byte)133, (byte)160, (byte)75, (byte)162, (byte)26, (byte)125, (byte)95, (byte)125, (byte)162, (byte)34, (byte)85, (byte)235, (byte)95, (byte)152, (byte)113, (byte)46, (byte)63, (byte)15, (byte)115, (byte)160, (byte)161, (byte)221, (byte)181, (byte)124, (byte)48, (byte)187, (byte)70, (byte)51, (byte)192, (byte)62, (byte)42, (byte)222, (byte)217, (byte)1, (byte)238, (byte)3, (byte)73, (byte)72, (byte)104, (byte)237, (byte)5, (byte)17, (byte)152, (byte)31, (byte)23, (byte)175, (byte)191, (byte)47, (byte)0, (byte)36, (byte)42, (byte)50, (byte)1, (byte)69, (byte)25, (byte)187, (byte)24, (byte)59}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)26954);
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)26954;
            p131.data__SET(new byte[] {(byte)152, (byte)230, (byte)137, (byte)168, (byte)187, (byte)232, (byte)182, (byte)10, (byte)196, (byte)237, (byte)38, (byte)91, (byte)10, (byte)183, (byte)59, (byte)89, (byte)218, (byte)77, (byte)137, (byte)34, (byte)33, (byte)159, (byte)198, (byte)194, (byte)196, (byte)137, (byte)98, (byte)112, (byte)3, (byte)202, (byte)145, (byte)101, (byte)61, (byte)88, (byte)203, (byte)27, (byte)164, (byte)55, (byte)202, (byte)108, (byte)2, (byte)180, (byte)64, (byte)59, (byte)202, (byte)229, (byte)185, (byte)164, (byte)98, (byte)241, (byte)81, (byte)173, (byte)216, (byte)104, (byte)179, (byte)207, (byte)188, (byte)128, (byte)11, (byte)100, (byte)75, (byte)159, (byte)54, (byte)16, (byte)115, (byte)94, (byte)251, (byte)5, (byte)161, (byte)190, (byte)15, (byte)29, (byte)51, (byte)154, (byte)251, (byte)138, (byte)208, (byte)210, (byte)174, (byte)116, (byte)109, (byte)1, (byte)120, (byte)51, (byte)52, (byte)4, (byte)98, (byte)108, (byte)197, (byte)150, (byte)207, (byte)158, (byte)182, (byte)1, (byte)56, (byte)89, (byte)105, (byte)131, (byte)206, (byte)120, (byte)234, (byte)242, (byte)10, (byte)172, (byte)124, (byte)50, (byte)252, (byte)89, (byte)60, (byte)217, (byte)13, (byte)45, (byte)81, (byte)140, (byte)76, (byte)7, (byte)108, (byte)222, (byte)31, (byte)247, (byte)119, (byte)13, (byte)210, (byte)66, (byte)36, (byte)116, (byte)231, (byte)32, (byte)124, (byte)43, (byte)194, (byte)89, (byte)248, (byte)151, (byte)33, (byte)213, (byte)116, (byte)87, (byte)227, (byte)218, (byte)126, (byte)2, (byte)226, (byte)99, (byte)190, (byte)214, (byte)29, (byte)13, (byte)108, (byte)205, (byte)100, (byte)238, (byte)172, (byte)75, (byte)89, (byte)28, (byte)224, (byte)242, (byte)52, (byte)17, (byte)104, (byte)200, (byte)101, (byte)175, (byte)212, (byte)90, (byte)224, (byte)12, (byte)208, (byte)117, (byte)63, (byte)255, (byte)188, (byte)202, (byte)184, (byte)203, (byte)198, (byte)60, (byte)144, (byte)128, (byte)243, (byte)0, (byte)89, (byte)203, (byte)143, (byte)146, (byte)132, (byte)54, (byte)130, (byte)50, (byte)81, (byte)195, (byte)236, (byte)208, (byte)144, (byte)133, (byte)160, (byte)75, (byte)162, (byte)26, (byte)125, (byte)95, (byte)125, (byte)162, (byte)34, (byte)85, (byte)235, (byte)95, (byte)152, (byte)113, (byte)46, (byte)63, (byte)15, (byte)115, (byte)160, (byte)161, (byte)221, (byte)181, (byte)124, (byte)48, (byte)187, (byte)70, (byte)51, (byte)192, (byte)62, (byte)42, (byte)222, (byte)217, (byte)1, (byte)238, (byte)3, (byte)73, (byte)72, (byte)104, (byte)237, (byte)5, (byte)17, (byte)152, (byte)31, (byte)23, (byte)175, (byte)191, (byte)47, (byte)0, (byte)36, (byte)42, (byte)50, (byte)1, (byte)69, (byte)25, (byte)187, (byte)24, (byte)59}, 0) ;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.max_distance == (ushort)(ushort)4813);
                Debug.Assert(pack.id == (byte)(byte)138);
                Debug.Assert(pack.orientation == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_PITCH_270);
                Debug.Assert(pack.type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
                Debug.Assert(pack.time_boot_ms == (uint)3770797182U);
                Debug.Assert(pack.covariance == (byte)(byte)222);
                Debug.Assert(pack.min_distance == (ushort)(ushort)30191);
                Debug.Assert(pack.current_distance == (ushort)(ushort)35035);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            p132.time_boot_ms = (uint)3770797182U;
            p132.covariance = (byte)(byte)222;
            p132.id = (byte)(byte)138;
            p132.max_distance = (ushort)(ushort)4813;
            p132.min_distance = (ushort)(ushort)30191;
            p132.current_distance = (ushort)(ushort)35035;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_PITCH_270;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)26954);
                Debug.Assert(pack.lon == (int) -1233226670);
                Debug.Assert(pack.lat == (int)617152712);
                Debug.Assert(pack.mask == (ulong)8629333286092399935L);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lon = (int) -1233226670;
            p133.lat = (int)617152712;
            p133.mask = (ulong)8629333286092399935L;
            p133.grid_spacing = (ushort)(ushort)26954;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)872284712);
                Debug.Assert(pack.gridbit == (byte)(byte)223);
                Debug.Assert(pack.lat == (int) -1336776104);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)9313);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -25386, (short)25047, (short)5734, (short) -15801, (short) -10950, (short) -12187, (short) -27006, (short)31519, (short) -27817, (short) -25216, (short) -31932, (short) -5929, (short) -17535, (short) -7524, (short)26493, (short)5548}));
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.gridbit = (byte)(byte)223;
            p134.lon = (int)872284712;
            p134.data__SET(new short[] {(short) -25386, (short)25047, (short)5734, (short) -15801, (short) -10950, (short) -12187, (short) -27006, (short)31519, (short) -27817, (short) -25216, (short) -31932, (short) -5929, (short) -17535, (short) -7524, (short)26493, (short)5548}, 0) ;
            p134.lat = (int) -1336776104;
            p134.grid_spacing = (ushort)(ushort)9313;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1241364831);
                Debug.Assert(pack.lon == (int)1429596040);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lon = (int)1429596040;
            p135.lat = (int)1241364831;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.terrain_height == (float)2.3532213E38F);
                Debug.Assert(pack.pending == (ushort)(ushort)15031);
                Debug.Assert(pack.lat == (int)933714188);
                Debug.Assert(pack.loaded == (ushort)(ushort)15159);
                Debug.Assert(pack.current_height == (float)1.4072342E38F);
                Debug.Assert(pack.lon == (int) -765006431);
                Debug.Assert(pack.spacing == (ushort)(ushort)20805);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lon = (int) -765006431;
            p136.loaded = (ushort)(ushort)15159;
            p136.spacing = (ushort)(ushort)20805;
            p136.current_height = (float)1.4072342E38F;
            p136.pending = (ushort)(ushort)15031;
            p136.terrain_height = (float)2.3532213E38F;
            p136.lat = (int)933714188;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -6983);
                Debug.Assert(pack.time_boot_ms == (uint)736933188U);
                Debug.Assert(pack.press_abs == (float) -8.442179E37F);
                Debug.Assert(pack.press_diff == (float) -2.0329581E37F);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)736933188U;
            p137.press_abs = (float) -8.442179E37F;
            p137.press_diff = (float) -2.0329581E37F;
            p137.temperature = (short)(short) -6983;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -1.2196927E38F);
                Debug.Assert(pack.x == (float)2.1059368E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.275016E38F, 2.9856005E38F, 1.4126832E38F, 2.5511602E38F}));
                Debug.Assert(pack.time_usec == (ulong)547815662718712043L);
                Debug.Assert(pack.z == (float)6.5690856E37F);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.q_SET(new float[] {-3.275016E38F, 2.9856005E38F, 1.4126832E38F, 2.5511602E38F}, 0) ;
            p138.z = (float)6.5690856E37F;
            p138.time_usec = (ulong)547815662718712043L;
            p138.y = (float) -1.2196927E38F;
            p138.x = (float)2.1059368E38F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)6121724388809668173L);
                Debug.Assert(pack.group_mlx == (byte)(byte)131);
                Debug.Assert(pack.target_component == (byte)(byte)190);
                Debug.Assert(pack.target_system == (byte)(byte)130);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {2.7420132E38F, -2.462336E38F, 2.3230797E38F, -1.1397941E38F, 1.1261636E38F, -8.200592E37F, 2.1906487E38F, 6.839675E37F}));
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.target_system = (byte)(byte)130;
            p139.time_usec = (ulong)6121724388809668173L;
            p139.group_mlx = (byte)(byte)131;
            p139.controls_SET(new float[] {2.7420132E38F, -2.462336E38F, 2.3230797E38F, -1.1397941E38F, 1.1261636E38F, -8.200592E37F, 2.1906487E38F, 6.839675E37F}, 0) ;
            p139.target_component = (byte)(byte)190;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)1775685466121255784L);
                Debug.Assert(pack.group_mlx == (byte)(byte)229);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-2.9209442E38F, -6.4813266E37F, -2.3541752E38F, 3.2972422E38F, -2.5484673E38F, -2.444422E38F, -3.9121087E37F, 9.001309E37F}));
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.group_mlx = (byte)(byte)229;
            p140.time_usec = (ulong)1775685466121255784L;
            p140.controls_SET(new float[] {-2.9209442E38F, -6.4813266E37F, -2.3541752E38F, 3.2972422E38F, -2.5484673E38F, -2.444422E38F, -3.9121087E37F, 9.001309E37F}, 0) ;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)7345432094695344184L);
                Debug.Assert(pack.bottom_clearance == (float) -5.808724E37F);
                Debug.Assert(pack.altitude_relative == (float) -5.98652E37F);
                Debug.Assert(pack.altitude_monotonic == (float) -1.7463222E38F);
                Debug.Assert(pack.altitude_local == (float) -1.1532019E38F);
                Debug.Assert(pack.altitude_terrain == (float)2.4944607E38F);
                Debug.Assert(pack.altitude_amsl == (float)2.7452623E38F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_relative = (float) -5.98652E37F;
            p141.bottom_clearance = (float) -5.808724E37F;
            p141.altitude_terrain = (float)2.4944607E38F;
            p141.altitude_amsl = (float)2.7452623E38F;
            p141.altitude_local = (float) -1.1532019E38F;
            p141.altitude_monotonic = (float) -1.7463222E38F;
            p141.time_usec = (ulong)7345432094695344184L;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)247, (byte)186, (byte)45, (byte)47, (byte)222, (byte)133, (byte)68, (byte)18, (byte)121, (byte)106, (byte)75, (byte)106, (byte)202, (byte)114, (byte)44, (byte)243, (byte)186, (byte)215, (byte)230, (byte)226, (byte)197, (byte)167, (byte)103, (byte)146, (byte)35, (byte)13, (byte)165, (byte)220, (byte)192, (byte)162, (byte)223, (byte)208, (byte)220, (byte)59, (byte)7, (byte)209, (byte)182, (byte)154, (byte)137, (byte)0, (byte)39, (byte)111, (byte)207, (byte)66, (byte)150, (byte)24, (byte)205, (byte)7, (byte)249, (byte)165, (byte)230, (byte)103, (byte)222, (byte)154, (byte)56, (byte)242, (byte)212, (byte)51, (byte)156, (byte)19, (byte)253, (byte)176, (byte)73, (byte)206, (byte)148, (byte)19, (byte)201, (byte)109, (byte)147, (byte)236, (byte)78, (byte)192, (byte)19, (byte)100, (byte)225, (byte)243, (byte)148, (byte)223, (byte)181, (byte)30, (byte)41, (byte)72, (byte)61, (byte)225, (byte)242, (byte)169, (byte)126, (byte)121, (byte)161, (byte)8, (byte)93, (byte)177, (byte)139, (byte)197, (byte)158, (byte)245, (byte)138, (byte)217, (byte)106, (byte)197, (byte)65, (byte)109, (byte)186, (byte)253, (byte)157, (byte)5, (byte)186, (byte)91, (byte)75, (byte)119, (byte)11, (byte)93, (byte)139, (byte)51, (byte)184, (byte)206, (byte)212, (byte)125, (byte)221, (byte)73}));
                Debug.Assert(pack.uri_type == (byte)(byte)100);
                Debug.Assert(pack.request_id == (byte)(byte)50);
                Debug.Assert(pack.transfer_type == (byte)(byte)97);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)142, (byte)6, (byte)239, (byte)94, (byte)173, (byte)10, (byte)229, (byte)202, (byte)50, (byte)58, (byte)154, (byte)120, (byte)138, (byte)198, (byte)11, (byte)182, (byte)253, (byte)255, (byte)216, (byte)97, (byte)83, (byte)21, (byte)45, (byte)144, (byte)39, (byte)76, (byte)101, (byte)8, (byte)51, (byte)101, (byte)52, (byte)134, (byte)22, (byte)175, (byte)2, (byte)22, (byte)132, (byte)150, (byte)160, (byte)90, (byte)209, (byte)61, (byte)97, (byte)203, (byte)10, (byte)3, (byte)64, (byte)159, (byte)91, (byte)35, (byte)206, (byte)67, (byte)126, (byte)194, (byte)188, (byte)113, (byte)75, (byte)1, (byte)47, (byte)104, (byte)252, (byte)221, (byte)226, (byte)202, (byte)8, (byte)200, (byte)101, (byte)73, (byte)113, (byte)2, (byte)253, (byte)127, (byte)22, (byte)63, (byte)112, (byte)136, (byte)115, (byte)229, (byte)140, (byte)187, (byte)79, (byte)26, (byte)22, (byte)125, (byte)11, (byte)234, (byte)47, (byte)131, (byte)250, (byte)136, (byte)233, (byte)126, (byte)199, (byte)141, (byte)249, (byte)178, (byte)237, (byte)106, (byte)192, (byte)0, (byte)156, (byte)163, (byte)107, (byte)169, (byte)127, (byte)212, (byte)16, (byte)158, (byte)160, (byte)58, (byte)213, (byte)127, (byte)139, (byte)16, (byte)69, (byte)31, (byte)249, (byte)16, (byte)68, (byte)132}));
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.storage_SET(new byte[] {(byte)247, (byte)186, (byte)45, (byte)47, (byte)222, (byte)133, (byte)68, (byte)18, (byte)121, (byte)106, (byte)75, (byte)106, (byte)202, (byte)114, (byte)44, (byte)243, (byte)186, (byte)215, (byte)230, (byte)226, (byte)197, (byte)167, (byte)103, (byte)146, (byte)35, (byte)13, (byte)165, (byte)220, (byte)192, (byte)162, (byte)223, (byte)208, (byte)220, (byte)59, (byte)7, (byte)209, (byte)182, (byte)154, (byte)137, (byte)0, (byte)39, (byte)111, (byte)207, (byte)66, (byte)150, (byte)24, (byte)205, (byte)7, (byte)249, (byte)165, (byte)230, (byte)103, (byte)222, (byte)154, (byte)56, (byte)242, (byte)212, (byte)51, (byte)156, (byte)19, (byte)253, (byte)176, (byte)73, (byte)206, (byte)148, (byte)19, (byte)201, (byte)109, (byte)147, (byte)236, (byte)78, (byte)192, (byte)19, (byte)100, (byte)225, (byte)243, (byte)148, (byte)223, (byte)181, (byte)30, (byte)41, (byte)72, (byte)61, (byte)225, (byte)242, (byte)169, (byte)126, (byte)121, (byte)161, (byte)8, (byte)93, (byte)177, (byte)139, (byte)197, (byte)158, (byte)245, (byte)138, (byte)217, (byte)106, (byte)197, (byte)65, (byte)109, (byte)186, (byte)253, (byte)157, (byte)5, (byte)186, (byte)91, (byte)75, (byte)119, (byte)11, (byte)93, (byte)139, (byte)51, (byte)184, (byte)206, (byte)212, (byte)125, (byte)221, (byte)73}, 0) ;
            p142.uri_SET(new byte[] {(byte)142, (byte)6, (byte)239, (byte)94, (byte)173, (byte)10, (byte)229, (byte)202, (byte)50, (byte)58, (byte)154, (byte)120, (byte)138, (byte)198, (byte)11, (byte)182, (byte)253, (byte)255, (byte)216, (byte)97, (byte)83, (byte)21, (byte)45, (byte)144, (byte)39, (byte)76, (byte)101, (byte)8, (byte)51, (byte)101, (byte)52, (byte)134, (byte)22, (byte)175, (byte)2, (byte)22, (byte)132, (byte)150, (byte)160, (byte)90, (byte)209, (byte)61, (byte)97, (byte)203, (byte)10, (byte)3, (byte)64, (byte)159, (byte)91, (byte)35, (byte)206, (byte)67, (byte)126, (byte)194, (byte)188, (byte)113, (byte)75, (byte)1, (byte)47, (byte)104, (byte)252, (byte)221, (byte)226, (byte)202, (byte)8, (byte)200, (byte)101, (byte)73, (byte)113, (byte)2, (byte)253, (byte)127, (byte)22, (byte)63, (byte)112, (byte)136, (byte)115, (byte)229, (byte)140, (byte)187, (byte)79, (byte)26, (byte)22, (byte)125, (byte)11, (byte)234, (byte)47, (byte)131, (byte)250, (byte)136, (byte)233, (byte)126, (byte)199, (byte)141, (byte)249, (byte)178, (byte)237, (byte)106, (byte)192, (byte)0, (byte)156, (byte)163, (byte)107, (byte)169, (byte)127, (byte)212, (byte)16, (byte)158, (byte)160, (byte)58, (byte)213, (byte)127, (byte)139, (byte)16, (byte)69, (byte)31, (byte)249, (byte)16, (byte)68, (byte)132}, 0) ;
            p142.transfer_type = (byte)(byte)97;
            p142.uri_type = (byte)(byte)100;
            p142.request_id = (byte)(byte)50;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float) -6.216349E37F);
                Debug.Assert(pack.temperature == (short)(short)23925);
                Debug.Assert(pack.time_boot_ms == (uint)2624795783U);
                Debug.Assert(pack.press_abs == (float) -1.0276975E38F);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)2624795783U;
            p143.press_diff = (float) -6.216349E37F;
            p143.temperature = (short)(short)23925;
            p143.press_abs = (float) -1.0276975E38F;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)466821194);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {-9.346974E37F, -2.201379E38F, -2.771934E38F}));
                Debug.Assert(pack.acc.SequenceEqual(new float[] {-3.3084083E37F, -2.123631E38F, 2.7297996E38F}));
                Debug.Assert(pack.est_capabilities == (byte)(byte)24);
                Debug.Assert(pack.alt == (float) -1.9214798E38F);
                Debug.Assert(pack.timestamp == (ulong)451590206912110887L);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-1.6275201E37F, 4.367872E37F, -6.1030354E37F}));
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-1.9679675E38F, -7.803294E37F, 1.3668085E38F, -3.6188603E37F}));
                Debug.Assert(pack.custom_state == (ulong)940880629924859261L);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-1.7308775E38F, -2.0297323E38F, -2.6086815E38F}));
                Debug.Assert(pack.lon == (int) -1988605670);
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.lon = (int) -1988605670;
            p144.alt = (float) -1.9214798E38F;
            p144.attitude_q_SET(new float[] {-1.9679675E38F, -7.803294E37F, 1.3668085E38F, -3.6188603E37F}, 0) ;
            p144.vel_SET(new float[] {-1.7308775E38F, -2.0297323E38F, -2.6086815E38F}, 0) ;
            p144.acc_SET(new float[] {-3.3084083E37F, -2.123631E38F, 2.7297996E38F}, 0) ;
            p144.position_cov_SET(new float[] {-9.346974E37F, -2.201379E38F, -2.771934E38F}, 0) ;
            p144.timestamp = (ulong)451590206912110887L;
            p144.custom_state = (ulong)940880629924859261L;
            p144.est_capabilities = (byte)(byte)24;
            p144.rates_SET(new float[] {-1.6275201E37F, 4.367872E37F, -6.1030354E37F}, 0) ;
            p144.lat = (int)466821194;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {3.8910297E37F, -2.3972543E38F, 7.102596E37F}));
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {2.9722875E38F, 1.7853227E37F, -1.4114664E38F}));
                Debug.Assert(pack.y_pos == (float)6.796111E37F);
                Debug.Assert(pack.airspeed == (float)4.793938E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.3259938E38F, 9.78376E37F, 1.295555E38F, 6.562715E37F}));
                Debug.Assert(pack.x_acc == (float)1.1334827E37F);
                Debug.Assert(pack.roll_rate == (float)2.1949549E38F);
                Debug.Assert(pack.yaw_rate == (float)1.308104E38F);
                Debug.Assert(pack.z_vel == (float)3.1859118E38F);
                Debug.Assert(pack.pitch_rate == (float) -1.2300627E38F);
                Debug.Assert(pack.time_usec == (ulong)8006889909058643814L);
                Debug.Assert(pack.z_acc == (float)1.3926137E38F);
                Debug.Assert(pack.x_vel == (float)2.7185797E38F);
                Debug.Assert(pack.z_pos == (float) -3.2225893E38F);
                Debug.Assert(pack.x_pos == (float) -1.3324229E38F);
                Debug.Assert(pack.y_acc == (float) -7.3181043E37F);
                Debug.Assert(pack.y_vel == (float)1.2785645E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.q_SET(new float[] {2.3259938E38F, 9.78376E37F, 1.295555E38F, 6.562715E37F}, 0) ;
            p146.roll_rate = (float)2.1949549E38F;
            p146.pos_variance_SET(new float[] {2.9722875E38F, 1.7853227E37F, -1.4114664E38F}, 0) ;
            p146.y_vel = (float)1.2785645E38F;
            p146.z_acc = (float)1.3926137E38F;
            p146.x_pos = (float) -1.3324229E38F;
            p146.x_vel = (float)2.7185797E38F;
            p146.y_acc = (float) -7.3181043E37F;
            p146.y_pos = (float)6.796111E37F;
            p146.pitch_rate = (float) -1.2300627E38F;
            p146.yaw_rate = (float)1.308104E38F;
            p146.z_pos = (float) -3.2225893E38F;
            p146.time_usec = (ulong)8006889909058643814L;
            p146.z_vel = (float)3.1859118E38F;
            p146.airspeed = (float)4.793938E37F;
            p146.x_acc = (float)1.1334827E37F;
            p146.vel_variance_SET(new float[] {3.8910297E37F, -2.3972543E38F, 7.102596E37F}, 0) ;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_battery == (short)(short) -28153);
                Debug.Assert(pack.battery_function == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)4);
                Debug.Assert(pack.temperature == (short)(short) -30716);
                Debug.Assert(pack.id == (byte)(byte)250);
                Debug.Assert(pack.type == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN);
                Debug.Assert(pack.current_consumed == (int)1145971335);
                Debug.Assert(pack.energy_consumed == (int)1077640092);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)33579, (ushort)28203, (ushort)10656, (ushort)10699, (ushort)39471, (ushort)8681, (ushort)41037, (ushort)44868, (ushort)41626, (ushort)9461}));
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN;
            p147.battery_remaining = (sbyte)(sbyte)4;
            p147.current_consumed = (int)1145971335;
            p147.current_battery = (short)(short) -28153;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION;
            p147.temperature = (short)(short) -30716;
            p147.id = (byte)(byte)250;
            p147.voltages_SET(new ushort[] {(ushort)33579, (ushort)28203, (ushort)10656, (ushort)10699, (ushort)39471, (ushort)8681, (ushort)41037, (ushort)44868, (ushort)41626, (ushort)9461}, 0) ;
            p147.energy_consumed = (int)1077640092;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)135, (byte)58, (byte)242, (byte)47, (byte)220, (byte)192, (byte)33, (byte)61}));
                Debug.Assert(pack.middleware_sw_version == (uint)3313805200U);
                Debug.Assert(pack.uid == (ulong)488505740916515108L);
                Debug.Assert(pack.os_sw_version == (uint)3208292191U);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)19637);
                Debug.Assert(pack.flight_sw_version == (uint)1708001073U);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)38, (byte)252, (byte)18, (byte)157, (byte)175, (byte)77, (byte)195, (byte)158}));
                Debug.Assert(pack.board_version == (uint)3916396742U);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET));
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)3, (byte)189, (byte)157, (byte)162, (byte)169, (byte)182, (byte)174, (byte)31, (byte)124, (byte)171, (byte)150, (byte)100, (byte)154, (byte)246, (byte)182, (byte)158, (byte)168, (byte)53}));
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)44, (byte)104, (byte)36, (byte)33, (byte)179, (byte)85, (byte)156, (byte)253}));
                Debug.Assert(pack.product_id == (ushort)(ushort)14499);
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.os_sw_version = (uint)3208292191U;
            p148.uid2_SET(new byte[] {(byte)3, (byte)189, (byte)157, (byte)162, (byte)169, (byte)182, (byte)174, (byte)31, (byte)124, (byte)171, (byte)150, (byte)100, (byte)154, (byte)246, (byte)182, (byte)158, (byte)168, (byte)53}, 0, PH) ;
            p148.flight_custom_version_SET(new byte[] {(byte)38, (byte)252, (byte)18, (byte)157, (byte)175, (byte)77, (byte)195, (byte)158}, 0) ;
            p148.product_id = (ushort)(ushort)14499;
            p148.uid = (ulong)488505740916515108L;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET);
            p148.middleware_custom_version_SET(new byte[] {(byte)44, (byte)104, (byte)36, (byte)33, (byte)179, (byte)85, (byte)156, (byte)253}, 0) ;
            p148.middleware_sw_version = (uint)3313805200U;
            p148.board_version = (uint)3916396742U;
            p148.flight_sw_version = (uint)1708001073U;
            p148.vendor_id = (ushort)(ushort)19637;
            p148.os_custom_version_SET(new byte[] {(byte)135, (byte)58, (byte)242, (byte)47, (byte)220, (byte)192, (byte)33, (byte)61}, 0) ;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.angle_x == (float)8.045781E37F);
                Debug.Assert(pack.size_x == (float) -1.4029241E38F);
                Debug.Assert(pack.x_TRY(ph) == (float)9.044053E37F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)2);
                Debug.Assert(pack.y_TRY(ph) == (float) -7.456669E36F);
                Debug.Assert(pack.time_usec == (ulong)48801515698792199L);
                Debug.Assert(pack.z_TRY(ph) == (float) -1.0165032E38F);
                Debug.Assert(pack.target_num == (byte)(byte)45);
                Debug.Assert(pack.angle_y == (float) -2.2836764E38F);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {-9.391052E37F, 1.2204381E38F, -9.88854E37F, 3.2781051E38F}));
                Debug.Assert(pack.size_y == (float) -1.9373703E37F);
                Debug.Assert(pack.type == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
                Debug.Assert(pack.distance == (float) -2.5763072E38F);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.angle_y = (float) -2.2836764E38F;
            p149.y_SET((float) -7.456669E36F, PH) ;
            p149.size_y = (float) -1.9373703E37F;
            p149.distance = (float) -2.5763072E38F;
            p149.angle_x = (float)8.045781E37F;
            p149.time_usec = (ulong)48801515698792199L;
            p149.size_x = (float) -1.4029241E38F;
            p149.frame = MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.z_SET((float) -1.0165032E38F, PH) ;
            p149.x_SET((float)9.044053E37F, PH) ;
            p149.target_num = (byte)(byte)45;
            p149.position_valid_SET((byte)(byte)2, PH) ;
            p149.q_SET(new float[] {-9.391052E37F, 1.2204381E38F, -9.88854E37F, 3.2781051E38F}, 0, PH) ;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCRIPT_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 45);
                Debug.Assert(pack.name_TRY(ph).Equals("hzyQiuxwgeeozietyzaZbElnsxnwzqcsegxjqtaquiAfq"));
                Debug.Assert(pack.target_component == (byte)(byte)234);
                Debug.Assert(pack.target_system == (byte)(byte)218);
                Debug.Assert(pack.seq == (ushort)(ushort)44615);
            };
            GroundControl.SCRIPT_ITEM p180 = CommunicationChannel.new_SCRIPT_ITEM();
            PH.setPack(p180);
            p180.name_SET("hzyQiuxwgeeozietyzaZbElnsxnwzqcsegxjqtaquiAfq", PH) ;
            p180.target_component = (byte)(byte)234;
            p180.seq = (ushort)(ushort)44615;
            p180.target_system = (byte)(byte)218;
            CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCRIPT_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)28418);
                Debug.Assert(pack.target_system == (byte)(byte)141);
                Debug.Assert(pack.target_component == (byte)(byte)22);
            };
            GroundControl.SCRIPT_REQUEST p181 = CommunicationChannel.new_SCRIPT_REQUEST();
            PH.setPack(p181);
            p181.target_component = (byte)(byte)22;
            p181.target_system = (byte)(byte)141;
            p181.seq = (ushort)(ushort)28418;
            CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCRIPT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)14);
                Debug.Assert(pack.target_system == (byte)(byte)138);
            };
            GroundControl.SCRIPT_REQUEST_LIST p182 = CommunicationChannel.new_SCRIPT_REQUEST_LIST();
            PH.setPack(p182);
            p182.target_component = (byte)(byte)14;
            p182.target_system = (byte)(byte)138;
            CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCRIPT_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)136);
                Debug.Assert(pack.target_system == (byte)(byte)79);
                Debug.Assert(pack.count == (ushort)(ushort)16447);
            };
            GroundControl.SCRIPT_COUNT p183 = CommunicationChannel.new_SCRIPT_COUNT();
            PH.setPack(p183);
            p183.target_system = (byte)(byte)79;
            p183.target_component = (byte)(byte)136;
            p183.count = (ushort)(ushort)16447;
            CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCRIPT_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)65370);
            };
            GroundControl.SCRIPT_CURRENT p184 = CommunicationChannel.new_SCRIPT_CURRENT();
            PH.setPack(p184);
            p184.seq = (ushort)(ushort)65370;
            CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel_ratio == (float)1.0634925E38F);
                Debug.Assert(pack.mag_ratio == (float)2.1056062E38F);
                Debug.Assert(pack.hagl_ratio == (float)1.992974E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS));
                Debug.Assert(pack.tas_ratio == (float)1.582156E38F);
                Debug.Assert(pack.pos_vert_accuracy == (float) -1.359557E38F);
                Debug.Assert(pack.pos_vert_ratio == (float) -2.7350375E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float) -2.6507205E38F);
                Debug.Assert(pack.pos_horiz_ratio == (float)1.6915733E38F);
                Debug.Assert(pack.time_usec == (ulong)4308434972448125666L);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.pos_vert_ratio = (float) -2.7350375E38F;
            p230.hagl_ratio = (float)1.992974E38F;
            p230.vel_ratio = (float)1.0634925E38F;
            p230.pos_horiz_accuracy = (float) -2.6507205E38F;
            p230.time_usec = (ulong)4308434972448125666L;
            p230.tas_ratio = (float)1.582156E38F;
            p230.pos_vert_accuracy = (float) -1.359557E38F;
            p230.mag_ratio = (float)2.1056062E38F;
            p230.pos_horiz_ratio = (float)1.6915733E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS);
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)6379739322611923426L);
                Debug.Assert(pack.wind_z == (float) -2.163556E38F);
                Debug.Assert(pack.var_horiz == (float) -2.2780866E38F);
                Debug.Assert(pack.vert_accuracy == (float)9.308215E37F);
                Debug.Assert(pack.horiz_accuracy == (float)2.2544813E38F);
                Debug.Assert(pack.wind_alt == (float) -4.826267E37F);
                Debug.Assert(pack.wind_y == (float) -2.8296702E38F);
                Debug.Assert(pack.var_vert == (float)2.1729472E38F);
                Debug.Assert(pack.wind_x == (float) -1.8590317E38F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.wind_x = (float) -1.8590317E38F;
            p231.horiz_accuracy = (float)2.2544813E38F;
            p231.wind_alt = (float) -4.826267E37F;
            p231.wind_y = (float) -2.8296702E38F;
            p231.vert_accuracy = (float)9.308215E37F;
            p231.time_usec = (ulong)6379739322611923426L;
            p231.wind_z = (float) -2.163556E38F;
            p231.var_horiz = (float) -2.2780866E38F;
            p231.var_vert = (float)2.1729472E38F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)4443743613913052895L);
                Debug.Assert(pack.speed_accuracy == (float)4.4331085E37F);
                Debug.Assert(pack.vert_accuracy == (float)1.3378165E38F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ));
                Debug.Assert(pack.vdop == (float)1.7962222E38F);
                Debug.Assert(pack.alt == (float) -1.062079E38F);
                Debug.Assert(pack.time_week_ms == (uint)2283175841U);
                Debug.Assert(pack.fix_type == (byte)(byte)242);
                Debug.Assert(pack.horiz_accuracy == (float) -1.0881428E38F);
                Debug.Assert(pack.hdop == (float)2.7744478E38F);
                Debug.Assert(pack.vd == (float)1.823995E38F);
                Debug.Assert(pack.ve == (float) -2.197593E38F);
                Debug.Assert(pack.vn == (float) -2.9355076E38F);
                Debug.Assert(pack.gps_id == (byte)(byte)106);
                Debug.Assert(pack.lat == (int)2137028542);
                Debug.Assert(pack.lon == (int) -1608997737);
                Debug.Assert(pack.time_week == (ushort)(ushort)55861);
                Debug.Assert(pack.satellites_visible == (byte)(byte)152);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.time_usec = (ulong)4443743613913052895L;
            p232.gps_id = (byte)(byte)106;
            p232.lat = (int)2137028542;
            p232.time_week = (ushort)(ushort)55861;
            p232.vn = (float) -2.9355076E38F;
            p232.vert_accuracy = (float)1.3378165E38F;
            p232.horiz_accuracy = (float) -1.0881428E38F;
            p232.fix_type = (byte)(byte)242;
            p232.vdop = (float)1.7962222E38F;
            p232.speed_accuracy = (float)4.4331085E37F;
            p232.ve = (float) -2.197593E38F;
            p232.vd = (float)1.823995E38F;
            p232.time_week_ms = (uint)2283175841U;
            p232.alt = (float) -1.062079E38F;
            p232.hdop = (float)2.7744478E38F;
            p232.lon = (int) -1608997737;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ);
            p232.satellites_visible = (byte)(byte)152;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)34);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)201, (byte)217, (byte)130, (byte)222, (byte)75, (byte)89, (byte)154, (byte)107, (byte)64, (byte)199, (byte)67, (byte)146, (byte)56, (byte)151, (byte)137, (byte)47, (byte)235, (byte)209, (byte)117, (byte)12, (byte)48, (byte)106, (byte)159, (byte)75, (byte)147, (byte)91, (byte)250, (byte)235, (byte)8, (byte)201, (byte)74, (byte)26, (byte)197, (byte)190, (byte)57, (byte)107, (byte)138, (byte)129, (byte)216, (byte)251, (byte)145, (byte)134, (byte)142, (byte)222, (byte)33, (byte)115, (byte)126, (byte)23, (byte)207, (byte)65, (byte)172, (byte)100, (byte)48, (byte)165, (byte)42, (byte)148, (byte)58, (byte)216, (byte)147, (byte)30, (byte)223, (byte)165, (byte)93, (byte)147, (byte)27, (byte)43, (byte)180, (byte)99, (byte)208, (byte)203, (byte)125, (byte)119, (byte)75, (byte)22, (byte)174, (byte)38, (byte)193, (byte)43, (byte)177, (byte)133, (byte)81, (byte)242, (byte)45, (byte)55, (byte)47, (byte)162, (byte)2, (byte)176, (byte)73, (byte)173, (byte)207, (byte)83, (byte)177, (byte)89, (byte)11, (byte)47, (byte)220, (byte)39, (byte)247, (byte)12, (byte)14, (byte)129, (byte)213, (byte)2, (byte)135, (byte)168, (byte)216, (byte)32, (byte)120, (byte)111, (byte)83, (byte)66, (byte)109, (byte)133, (byte)118, (byte)123, (byte)148, (byte)217, (byte)25, (byte)211, (byte)230, (byte)200, (byte)112, (byte)149, (byte)26, (byte)16, (byte)15, (byte)234, (byte)16, (byte)245, (byte)105, (byte)168, (byte)199, (byte)128, (byte)106, (byte)233, (byte)236, (byte)246, (byte)250, (byte)3, (byte)210, (byte)10, (byte)86, (byte)172, (byte)133, (byte)118, (byte)255, (byte)134, (byte)14, (byte)87, (byte)109, (byte)17, (byte)22, (byte)195, (byte)240, (byte)113, (byte)132, (byte)25, (byte)0, (byte)101, (byte)2, (byte)31, (byte)102, (byte)136, (byte)91, (byte)221, (byte)153, (byte)2, (byte)40, (byte)8, (byte)139, (byte)150, (byte)87, (byte)243, (byte)1, (byte)19, (byte)22, (byte)250, (byte)125, (byte)5}));
                Debug.Assert(pack.flags == (byte)(byte)198);
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.data__SET(new byte[] {(byte)201, (byte)217, (byte)130, (byte)222, (byte)75, (byte)89, (byte)154, (byte)107, (byte)64, (byte)199, (byte)67, (byte)146, (byte)56, (byte)151, (byte)137, (byte)47, (byte)235, (byte)209, (byte)117, (byte)12, (byte)48, (byte)106, (byte)159, (byte)75, (byte)147, (byte)91, (byte)250, (byte)235, (byte)8, (byte)201, (byte)74, (byte)26, (byte)197, (byte)190, (byte)57, (byte)107, (byte)138, (byte)129, (byte)216, (byte)251, (byte)145, (byte)134, (byte)142, (byte)222, (byte)33, (byte)115, (byte)126, (byte)23, (byte)207, (byte)65, (byte)172, (byte)100, (byte)48, (byte)165, (byte)42, (byte)148, (byte)58, (byte)216, (byte)147, (byte)30, (byte)223, (byte)165, (byte)93, (byte)147, (byte)27, (byte)43, (byte)180, (byte)99, (byte)208, (byte)203, (byte)125, (byte)119, (byte)75, (byte)22, (byte)174, (byte)38, (byte)193, (byte)43, (byte)177, (byte)133, (byte)81, (byte)242, (byte)45, (byte)55, (byte)47, (byte)162, (byte)2, (byte)176, (byte)73, (byte)173, (byte)207, (byte)83, (byte)177, (byte)89, (byte)11, (byte)47, (byte)220, (byte)39, (byte)247, (byte)12, (byte)14, (byte)129, (byte)213, (byte)2, (byte)135, (byte)168, (byte)216, (byte)32, (byte)120, (byte)111, (byte)83, (byte)66, (byte)109, (byte)133, (byte)118, (byte)123, (byte)148, (byte)217, (byte)25, (byte)211, (byte)230, (byte)200, (byte)112, (byte)149, (byte)26, (byte)16, (byte)15, (byte)234, (byte)16, (byte)245, (byte)105, (byte)168, (byte)199, (byte)128, (byte)106, (byte)233, (byte)236, (byte)246, (byte)250, (byte)3, (byte)210, (byte)10, (byte)86, (byte)172, (byte)133, (byte)118, (byte)255, (byte)134, (byte)14, (byte)87, (byte)109, (byte)17, (byte)22, (byte)195, (byte)240, (byte)113, (byte)132, (byte)25, (byte)0, (byte)101, (byte)2, (byte)31, (byte)102, (byte)136, (byte)91, (byte)221, (byte)153, (byte)2, (byte)40, (byte)8, (byte)139, (byte)150, (byte)87, (byte)243, (byte)1, (byte)19, (byte)22, (byte)250, (byte)125, (byte)5}, 0) ;
            p233.len = (byte)(byte)34;
            p233.flags = (byte)(byte)198;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.airspeed == (byte)(byte)111);
                Debug.Assert(pack.custom_mode == (uint)3452409810U);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte) - 62);
                Debug.Assert(pack.groundspeed == (byte)(byte)92);
                Debug.Assert(pack.wp_num == (byte)(byte)219);
                Debug.Assert(pack.gps_fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)32);
                Debug.Assert(pack.gps_nsat == (byte)(byte)2);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)41);
                Debug.Assert(pack.roll == (short)(short) -26915);
                Debug.Assert(pack.heading == (ushort)(ushort)35643);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 50);
                Debug.Assert(pack.altitude_amsl == (short)(short)16873);
                Debug.Assert(pack.failsafe == (byte)(byte)106);
                Debug.Assert(pack.heading_sp == (short)(short) -13737);
                Debug.Assert(pack.altitude_sp == (short)(short) -686);
                Debug.Assert(pack.battery_remaining == (byte)(byte)245);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)63843);
                Debug.Assert(pack.latitude == (int)1567546655);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED));
                Debug.Assert(pack.temperature == (sbyte)(sbyte)19);
                Debug.Assert(pack.longitude == (int)1796721553);
                Debug.Assert(pack.pitch == (short)(short) -15163);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.groundspeed = (byte)(byte)92;
            p234.roll = (short)(short) -26915;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_PPP;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            p234.heading = (ushort)(ushort)35643;
            p234.pitch = (short)(short) -15163;
            p234.custom_mode = (uint)3452409810U;
            p234.battery_remaining = (byte)(byte)245;
            p234.temperature = (sbyte)(sbyte)19;
            p234.temperature_air = (sbyte)(sbyte) - 62;
            p234.throttle = (sbyte)(sbyte)32;
            p234.wp_distance = (ushort)(ushort)63843;
            p234.altitude_amsl = (short)(short)16873;
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED);
            p234.longitude = (int)1796721553;
            p234.climb_rate = (sbyte)(sbyte) - 50;
            p234.heading_sp = (short)(short) -13737;
            p234.gps_nsat = (byte)(byte)2;
            p234.airspeed_sp = (byte)(byte)41;
            p234.wp_num = (byte)(byte)219;
            p234.latitude = (int)1567546655;
            p234.failsafe = (byte)(byte)106;
            p234.airspeed = (byte)(byte)111;
            p234.altitude_sp = (short)(short) -686;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.clipping_1 == (uint)2478383191U);
                Debug.Assert(pack.vibration_x == (float) -2.0270765E38F);
                Debug.Assert(pack.vibration_y == (float) -9.994853E37F);
                Debug.Assert(pack.clipping_2 == (uint)2156811404U);
                Debug.Assert(pack.time_usec == (ulong)9136009323512165828L);
                Debug.Assert(pack.clipping_0 == (uint)1581566816U);
                Debug.Assert(pack.vibration_z == (float) -8.518087E37F);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.clipping_1 = (uint)2478383191U;
            p241.vibration_x = (float) -2.0270765E38F;
            p241.time_usec = (ulong)9136009323512165828L;
            p241.vibration_y = (float) -9.994853E37F;
            p241.clipping_0 = (uint)1581566816U;
            p241.clipping_2 = (uint)2156811404U;
            p241.vibration_z = (float) -8.518087E37F;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.approach_x == (float)9.807072E37F);
                Debug.Assert(pack.x == (float) -3.3908932E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.245131E38F, -1.437723E38F, 1.2791834E38F, -2.3471522E38F}));
                Debug.Assert(pack.altitude == (int)1831030367);
                Debug.Assert(pack.longitude == (int) -173212698);
                Debug.Assert(pack.z == (float)2.8093993E38F);
                Debug.Assert(pack.approach_z == (float) -2.8084629E38F);
                Debug.Assert(pack.approach_y == (float)1.5376457E38F);
                Debug.Assert(pack.latitude == (int)1454339517);
                Debug.Assert(pack.y == (float) -3.307815E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7002304137673402873L);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.altitude = (int)1831030367;
            p242.x = (float) -3.3908932E38F;
            p242.longitude = (int) -173212698;
            p242.z = (float)2.8093993E38F;
            p242.time_usec_SET((ulong)7002304137673402873L, PH) ;
            p242.q_SET(new float[] {3.245131E38F, -1.437723E38F, 1.2791834E38F, -2.3471522E38F}, 0) ;
            p242.approach_z = (float) -2.8084629E38F;
            p242.y = (float) -3.307815E38F;
            p242.approach_x = (float)9.807072E37F;
            p242.latitude = (int)1454339517;
            p242.approach_y = (float)1.5376457E38F;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int) -714565516);
                Debug.Assert(pack.approach_x == (float) -1.2580163E38F);
                Debug.Assert(pack.x == (float)2.4871542E38F);
                Debug.Assert(pack.approach_z == (float)1.0714236E38F);
                Debug.Assert(pack.longitude == (int) -1659281712);
                Debug.Assert(pack.y == (float) -9.483984E37F);
                Debug.Assert(pack.target_system == (byte)(byte)31);
                Debug.Assert(pack.approach_y == (float) -2.0879096E38F);
                Debug.Assert(pack.z == (float)1.2466752E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.9164515E38F, -1.9709095E38F, -1.9494452E38F, -4.497302E37F}));
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7984516380750552004L);
                Debug.Assert(pack.latitude == (int)843975769);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.x = (float)2.4871542E38F;
            p243.approach_y = (float) -2.0879096E38F;
            p243.time_usec_SET((ulong)7984516380750552004L, PH) ;
            p243.altitude = (int) -714565516;
            p243.y = (float) -9.483984E37F;
            p243.latitude = (int)843975769;
            p243.approach_z = (float)1.0714236E38F;
            p243.approach_x = (float) -1.2580163E38F;
            p243.longitude = (int) -1659281712;
            p243.target_system = (byte)(byte)31;
            p243.q_SET(new float[] {-2.9164515E38F, -1.9709095E38F, -1.9494452E38F, -4.497302E37F}, 0) ;
            p243.z = (float)1.2466752E37F;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)5075);
                Debug.Assert(pack.interval_us == (int)1920243310);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)5075;
            p244.interval_us = (int)1920243310;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.heading == (ushort)(ushort)60416);
                Debug.Assert(pack.altitude == (int)452931664);
                Debug.Assert(pack.ver_velocity == (short)(short) -10358);
                Debug.Assert(pack.flags == (ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS));
                Debug.Assert(pack.lat == (int)1687641209);
                Debug.Assert(pack.lon == (int)139550796);
                Debug.Assert(pack.altitude_type == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
                Debug.Assert(pack.tslc == (byte)(byte)197);
                Debug.Assert(pack.ICAO_address == (uint)2562665562U);
                Debug.Assert(pack.squawk == (ushort)(ushort)3470);
                Debug.Assert(pack.callsign_LEN(ph) == 9);
                Debug.Assert(pack.callsign_TRY(ph).Equals("brhjiocMx"));
                Debug.Assert(pack.emitter_type == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHT);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)31825);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHT;
            p246.heading = (ushort)(ushort)60416;
            p246.tslc = (byte)(byte)197;
            p246.hor_velocity = (ushort)(ushort)31825;
            p246.altitude = (int)452931664;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS);
            p246.ICAO_address = (uint)2562665562U;
            p246.callsign_SET("brhjiocMx", PH) ;
            p246.lat = (int)1687641209;
            p246.lon = (int)139550796;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.ver_velocity = (short)(short) -10358;
            p246.squawk = (ushort)(ushort)3470;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (uint)2618941818U);
                Debug.Assert(pack.action == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER);
                Debug.Assert(pack.src_ == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
                Debug.Assert(pack.altitude_minimum_delta == (float)1.4172681E38F);
                Debug.Assert(pack.threat_level == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
                Debug.Assert(pack.time_to_minimum_delta == (float) -2.5385634E38F);
                Debug.Assert(pack.horizontal_minimum_delta == (float) -3.2694137E38F);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.time_to_minimum_delta = (float) -2.5385634E38F;
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER;
            p247.id = (uint)2618941818U;
            p247.altitude_minimum_delta = (float)1.4172681E38F;
            p247.threat_level = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH;
            p247.horizontal_minimum_delta = (float) -3.2694137E38F;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)50);
                Debug.Assert(pack.target_network == (byte)(byte)182);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)107, (byte)155, (byte)225, (byte)65, (byte)76, (byte)106, (byte)62, (byte)114, (byte)125, (byte)28, (byte)20, (byte)253, (byte)19, (byte)86, (byte)144, (byte)91, (byte)100, (byte)55, (byte)29, (byte)170, (byte)178, (byte)99, (byte)58, (byte)30, (byte)183, (byte)252, (byte)33, (byte)84, (byte)101, (byte)167, (byte)90, (byte)148, (byte)39, (byte)227, (byte)217, (byte)192, (byte)103, (byte)72, (byte)221, (byte)42, (byte)163, (byte)127, (byte)115, (byte)9, (byte)166, (byte)168, (byte)173, (byte)203, (byte)188, (byte)86, (byte)28, (byte)120, (byte)118, (byte)56, (byte)252, (byte)34, (byte)234, (byte)182, (byte)28, (byte)59, (byte)120, (byte)54, (byte)28, (byte)47, (byte)29, (byte)144, (byte)232, (byte)27, (byte)43, (byte)226, (byte)176, (byte)98, (byte)115, (byte)161, (byte)128, (byte)225, (byte)156, (byte)25, (byte)105, (byte)152, (byte)113, (byte)206, (byte)119, (byte)180, (byte)234, (byte)47, (byte)7, (byte)244, (byte)191, (byte)155, (byte)64, (byte)188, (byte)166, (byte)130, (byte)109, (byte)55, (byte)19, (byte)88, (byte)48, (byte)11, (byte)241, (byte)53, (byte)11, (byte)82, (byte)238, (byte)143, (byte)83, (byte)42, (byte)188, (byte)73, (byte)69, (byte)50, (byte)144, (byte)166, (byte)102, (byte)43, (byte)190, (byte)66, (byte)17, (byte)144, (byte)60, (byte)143, (byte)96, (byte)193, (byte)37, (byte)51, (byte)180, (byte)166, (byte)22, (byte)229, (byte)30, (byte)53, (byte)151, (byte)219, (byte)191, (byte)225, (byte)114, (byte)242, (byte)254, (byte)219, (byte)86, (byte)65, (byte)181, (byte)36, (byte)36, (byte)57, (byte)200, (byte)143, (byte)28, (byte)207, (byte)202, (byte)202, (byte)117, (byte)137, (byte)132, (byte)193, (byte)102, (byte)115, (byte)98, (byte)166, (byte)43, (byte)63, (byte)16, (byte)155, (byte)38, (byte)155, (byte)90, (byte)143, (byte)183, (byte)111, (byte)186, (byte)99, (byte)240, (byte)88, (byte)71, (byte)57, (byte)110, (byte)122, (byte)94, (byte)116, (byte)21, (byte)112, (byte)253, (byte)217, (byte)249, (byte)194, (byte)231, (byte)32, (byte)251, (byte)127, (byte)177, (byte)77, (byte)242, (byte)162, (byte)151, (byte)223, (byte)60, (byte)244, (byte)77, (byte)225, (byte)60, (byte)101, (byte)236, (byte)6, (byte)101, (byte)174, (byte)82, (byte)48, (byte)214, (byte)44, (byte)15, (byte)34, (byte)158, (byte)76, (byte)30, (byte)139, (byte)38, (byte)141, (byte)194, (byte)51, (byte)8, (byte)169, (byte)198, (byte)83, (byte)216, (byte)119, (byte)108, (byte)69, (byte)83, (byte)64, (byte)90, (byte)24, (byte)91, (byte)40, (byte)133, (byte)81, (byte)176, (byte)195, (byte)94, (byte)9, (byte)159, (byte)175, (byte)200, (byte)98, (byte)219, (byte)211, (byte)182, (byte)142, (byte)253}));
                Debug.Assert(pack.target_component == (byte)(byte)105);
                Debug.Assert(pack.message_type == (ushort)(ushort)55403);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)182;
            p248.message_type = (ushort)(ushort)55403;
            p248.target_system = (byte)(byte)50;
            p248.payload_SET(new byte[] {(byte)107, (byte)155, (byte)225, (byte)65, (byte)76, (byte)106, (byte)62, (byte)114, (byte)125, (byte)28, (byte)20, (byte)253, (byte)19, (byte)86, (byte)144, (byte)91, (byte)100, (byte)55, (byte)29, (byte)170, (byte)178, (byte)99, (byte)58, (byte)30, (byte)183, (byte)252, (byte)33, (byte)84, (byte)101, (byte)167, (byte)90, (byte)148, (byte)39, (byte)227, (byte)217, (byte)192, (byte)103, (byte)72, (byte)221, (byte)42, (byte)163, (byte)127, (byte)115, (byte)9, (byte)166, (byte)168, (byte)173, (byte)203, (byte)188, (byte)86, (byte)28, (byte)120, (byte)118, (byte)56, (byte)252, (byte)34, (byte)234, (byte)182, (byte)28, (byte)59, (byte)120, (byte)54, (byte)28, (byte)47, (byte)29, (byte)144, (byte)232, (byte)27, (byte)43, (byte)226, (byte)176, (byte)98, (byte)115, (byte)161, (byte)128, (byte)225, (byte)156, (byte)25, (byte)105, (byte)152, (byte)113, (byte)206, (byte)119, (byte)180, (byte)234, (byte)47, (byte)7, (byte)244, (byte)191, (byte)155, (byte)64, (byte)188, (byte)166, (byte)130, (byte)109, (byte)55, (byte)19, (byte)88, (byte)48, (byte)11, (byte)241, (byte)53, (byte)11, (byte)82, (byte)238, (byte)143, (byte)83, (byte)42, (byte)188, (byte)73, (byte)69, (byte)50, (byte)144, (byte)166, (byte)102, (byte)43, (byte)190, (byte)66, (byte)17, (byte)144, (byte)60, (byte)143, (byte)96, (byte)193, (byte)37, (byte)51, (byte)180, (byte)166, (byte)22, (byte)229, (byte)30, (byte)53, (byte)151, (byte)219, (byte)191, (byte)225, (byte)114, (byte)242, (byte)254, (byte)219, (byte)86, (byte)65, (byte)181, (byte)36, (byte)36, (byte)57, (byte)200, (byte)143, (byte)28, (byte)207, (byte)202, (byte)202, (byte)117, (byte)137, (byte)132, (byte)193, (byte)102, (byte)115, (byte)98, (byte)166, (byte)43, (byte)63, (byte)16, (byte)155, (byte)38, (byte)155, (byte)90, (byte)143, (byte)183, (byte)111, (byte)186, (byte)99, (byte)240, (byte)88, (byte)71, (byte)57, (byte)110, (byte)122, (byte)94, (byte)116, (byte)21, (byte)112, (byte)253, (byte)217, (byte)249, (byte)194, (byte)231, (byte)32, (byte)251, (byte)127, (byte)177, (byte)77, (byte)242, (byte)162, (byte)151, (byte)223, (byte)60, (byte)244, (byte)77, (byte)225, (byte)60, (byte)101, (byte)236, (byte)6, (byte)101, (byte)174, (byte)82, (byte)48, (byte)214, (byte)44, (byte)15, (byte)34, (byte)158, (byte)76, (byte)30, (byte)139, (byte)38, (byte)141, (byte)194, (byte)51, (byte)8, (byte)169, (byte)198, (byte)83, (byte)216, (byte)119, (byte)108, (byte)69, (byte)83, (byte)64, (byte)90, (byte)24, (byte)91, (byte)40, (byte)133, (byte)81, (byte)176, (byte)195, (byte)94, (byte)9, (byte)159, (byte)175, (byte)200, (byte)98, (byte)219, (byte)211, (byte)182, (byte)142, (byte)253}, 0) ;
            p248.target_component = (byte)(byte)105;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)251);
                Debug.Assert(pack.address == (ushort)(ushort)26529);
                Debug.Assert(pack.ver == (byte)(byte)211);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte) - 122, (sbyte)58, (sbyte)20, (sbyte)90, (sbyte) - 26, (sbyte)66, (sbyte)101, (sbyte)20, (sbyte)91, (sbyte)24, (sbyte) - 123, (sbyte)98, (sbyte)0, (sbyte)85, (sbyte) - 125, (sbyte) - 115, (sbyte)100, (sbyte)80, (sbyte)3, (sbyte)66, (sbyte) - 39, (sbyte)111, (sbyte)103, (sbyte) - 39, (sbyte)122, (sbyte)78, (sbyte) - 109, (sbyte) - 2, (sbyte) - 91, (sbyte) - 86, (sbyte) - 53, (sbyte) - 127}));
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.type = (byte)(byte)251;
            p249.ver = (byte)(byte)211;
            p249.address = (ushort)(ushort)26529;
            p249.value_SET(new sbyte[] {(sbyte) - 122, (sbyte)58, (sbyte)20, (sbyte)90, (sbyte) - 26, (sbyte)66, (sbyte)101, (sbyte)20, (sbyte)91, (sbyte)24, (sbyte) - 123, (sbyte)98, (sbyte)0, (sbyte)85, (sbyte) - 125, (sbyte) - 115, (sbyte)100, (sbyte)80, (sbyte)3, (sbyte)66, (sbyte) - 39, (sbyte)111, (sbyte)103, (sbyte) - 39, (sbyte)122, (sbyte)78, (sbyte) - 109, (sbyte) - 2, (sbyte) - 91, (sbyte) - 86, (sbyte) - 53, (sbyte) - 127}, 0) ;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -3.1538068E38F);
                Debug.Assert(pack.y == (float) -1.9612863E38F);
                Debug.Assert(pack.z == (float)1.4134185E38F);
                Debug.Assert(pack.time_usec == (ulong)6107990302878127813L);
                Debug.Assert(pack.name_LEN(ph) == 6);
                Debug.Assert(pack.name_TRY(ph).Equals("lfgsig"));
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.y = (float) -1.9612863E38F;
            p250.time_usec = (ulong)6107990302878127813L;
            p250.x = (float) -3.1538068E38F;
            p250.name_SET("lfgsig", PH) ;
            p250.z = (float)1.4134185E38F;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)196388717U);
                Debug.Assert(pack.name_LEN(ph) == 5);
                Debug.Assert(pack.name_TRY(ph).Equals("heeKj"));
                Debug.Assert(pack.value == (float) -2.3934346E37F);
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.value = (float) -2.3934346E37F;
            p251.time_boot_ms = (uint)196388717U;
            p251.name_SET("heeKj", PH) ;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 6);
                Debug.Assert(pack.name_TRY(ph).Equals("uikzcl"));
                Debug.Assert(pack.time_boot_ms == (uint)2538500910U);
                Debug.Assert(pack.value == (int)1916981237);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.name_SET("uikzcl", PH) ;
            p252.value = (int)1916981237;
            p252.time_boot_ms = (uint)2538500910U;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.severity == MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
                Debug.Assert(pack.text_LEN(ph) == 18);
                Debug.Assert(pack.text_TRY(ph).Equals("ayiPxswwxyxrpisqhj"));
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("ayiPxswwxyxrpisqhj", PH) ;
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_CRITICAL;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ind == (byte)(byte)42);
                Debug.Assert(pack.value == (float)2.0470032E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2219169368U);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.ind = (byte)(byte)42;
            p254.value = (float)2.0470032E38F;
            p254.time_boot_ms = (uint)2219169368U;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)250, (byte)171, (byte)216, (byte)91, (byte)76, (byte)201, (byte)64, (byte)232, (byte)226, (byte)75, (byte)74, (byte)73, (byte)50, (byte)253, (byte)13, (byte)62, (byte)115, (byte)20, (byte)147, (byte)168, (byte)8, (byte)103, (byte)16, (byte)22, (byte)182, (byte)119, (byte)60, (byte)5, (byte)115, (byte)197, (byte)18, (byte)206}));
                Debug.Assert(pack.target_system == (byte)(byte)133);
                Debug.Assert(pack.target_component == (byte)(byte)241);
                Debug.Assert(pack.initial_timestamp == (ulong)657237564657241300L);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.secret_key_SET(new byte[] {(byte)250, (byte)171, (byte)216, (byte)91, (byte)76, (byte)201, (byte)64, (byte)232, (byte)226, (byte)75, (byte)74, (byte)73, (byte)50, (byte)253, (byte)13, (byte)62, (byte)115, (byte)20, (byte)147, (byte)168, (byte)8, (byte)103, (byte)16, (byte)22, (byte)182, (byte)119, (byte)60, (byte)5, (byte)115, (byte)197, (byte)18, (byte)206}, 0) ;
            p256.target_system = (byte)(byte)133;
            p256.target_component = (byte)(byte)241;
            p256.initial_timestamp = (ulong)657237564657241300L;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)224547629U);
                Debug.Assert(pack.last_change_ms == (uint)1930090105U);
                Debug.Assert(pack.state == (byte)(byte)159);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.state = (byte)(byte)159;
            p257.time_boot_ms = (uint)224547629U;
            p257.last_change_ms = (uint)1930090105U;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tune_LEN(ph) == 2);
                Debug.Assert(pack.tune_TRY(ph).Equals("yz"));
                Debug.Assert(pack.target_system == (byte)(byte)113);
                Debug.Assert(pack.target_component == (byte)(byte)71);
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)113;
            p258.target_component = (byte)(byte)71;
            p258.tune_SET("yz", PH) ;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)121, (byte)133, (byte)159, (byte)229, (byte)117, (byte)210, (byte)82, (byte)111, (byte)6, (byte)153, (byte)55, (byte)3, (byte)74, (byte)243, (byte)251, (byte)133, (byte)200, (byte)54, (byte)247, (byte)25, (byte)143, (byte)122, (byte)52, (byte)95, (byte)78, (byte)206, (byte)2, (byte)10, (byte)54, (byte)214, (byte)47, (byte)213}));
                Debug.Assert(pack.sensor_size_v == (float)2.9965145E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3984230529U);
                Debug.Assert(pack.lens_id == (byte)(byte)168);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)56, (byte)174, (byte)11, (byte)139, (byte)52, (byte)109, (byte)38, (byte)214, (byte)126, (byte)93, (byte)190, (byte)75, (byte)45, (byte)10, (byte)162, (byte)9, (byte)243, (byte)189, (byte)85, (byte)222, (byte)129, (byte)255, (byte)194, (byte)235, (byte)125, (byte)133, (byte)53, (byte)76, (byte)91, (byte)23, (byte)99, (byte)116}));
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 7);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("EUXwijx"));
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE));
                Debug.Assert(pack.focal_length == (float)2.757596E38F);
                Debug.Assert(pack.sensor_size_h == (float) -2.8098484E38F);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)40241);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)20209);
                Debug.Assert(pack.firmware_version == (uint)387973378U);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)23767);
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.cam_definition_version = (ushort)(ushort)40241;
            p259.resolution_h = (ushort)(ushort)23767;
            p259.focal_length = (float)2.757596E38F;
            p259.sensor_size_h = (float) -2.8098484E38F;
            p259.model_name_SET(new byte[] {(byte)121, (byte)133, (byte)159, (byte)229, (byte)117, (byte)210, (byte)82, (byte)111, (byte)6, (byte)153, (byte)55, (byte)3, (byte)74, (byte)243, (byte)251, (byte)133, (byte)200, (byte)54, (byte)247, (byte)25, (byte)143, (byte)122, (byte)52, (byte)95, (byte)78, (byte)206, (byte)2, (byte)10, (byte)54, (byte)214, (byte)47, (byte)213}, 0) ;
            p259.vendor_name_SET(new byte[] {(byte)56, (byte)174, (byte)11, (byte)139, (byte)52, (byte)109, (byte)38, (byte)214, (byte)126, (byte)93, (byte)190, (byte)75, (byte)45, (byte)10, (byte)162, (byte)9, (byte)243, (byte)189, (byte)85, (byte)222, (byte)129, (byte)255, (byte)194, (byte)235, (byte)125, (byte)133, (byte)53, (byte)76, (byte)91, (byte)23, (byte)99, (byte)116}, 0) ;
            p259.lens_id = (byte)(byte)168;
            p259.resolution_v = (ushort)(ushort)20209;
            p259.sensor_size_v = (float)2.9965145E38F;
            p259.time_boot_ms = (uint)3984230529U;
            p259.firmware_version = (uint)387973378U;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE);
            p259.cam_definition_uri_SET("EUXwijx", PH) ;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
                Debug.Assert(pack.time_boot_ms == (uint)2634569341U);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)2634569341U;
            p260.mode_id = CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.read_speed == (float)1.9303994E38F);
                Debug.Assert(pack.used_capacity == (float)1.3258166E36F);
                Debug.Assert(pack.time_boot_ms == (uint)3099418681U);
                Debug.Assert(pack.storage_count == (byte)(byte)248);
                Debug.Assert(pack.storage_id == (byte)(byte)244);
                Debug.Assert(pack.status == (byte)(byte)70);
                Debug.Assert(pack.total_capacity == (float)2.0015576E38F);
                Debug.Assert(pack.write_speed == (float) -9.154488E37F);
                Debug.Assert(pack.available_capacity == (float)6.5101474E37F);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.read_speed = (float)1.9303994E38F;
            p261.write_speed = (float) -9.154488E37F;
            p261.storage_id = (byte)(byte)244;
            p261.status = (byte)(byte)70;
            p261.available_capacity = (float)6.5101474E37F;
            p261.storage_count = (byte)(byte)248;
            p261.time_boot_ms = (uint)3099418681U;
            p261.total_capacity = (float)2.0015576E38F;
            p261.used_capacity = (float)1.3258166E36F;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)941629008U);
                Debug.Assert(pack.available_capacity == (float) -1.0221256E38F);
                Debug.Assert(pack.video_status == (byte)(byte)69);
                Debug.Assert(pack.image_interval == (float)7.6283507E37F);
                Debug.Assert(pack.recording_time_ms == (uint)3029215047U);
                Debug.Assert(pack.image_status == (byte)(byte)251);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.recording_time_ms = (uint)3029215047U;
            p262.time_boot_ms = (uint)941629008U;
            p262.available_capacity = (float) -1.0221256E38F;
            p262.video_status = (byte)(byte)69;
            p262.image_status = (byte)(byte)251;
            p262.image_interval = (float)7.6283507E37F;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.camera_id == (byte)(byte)31);
                Debug.Assert(pack.alt == (int)1798589805);
                Debug.Assert(pack.relative_alt == (int) -1114323809);
                Debug.Assert(pack.time_utc == (ulong)713401655494123576L);
                Debug.Assert(pack.file_url_LEN(ph) == 15);
                Debug.Assert(pack.file_url_TRY(ph).Equals("pFyeppHgufsmJsl"));
                Debug.Assert(pack.q.SequenceEqual(new float[] {6.344913E37F, -6.241367E37F, 1.113607E38F, -3.0645613E38F}));
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)105);
                Debug.Assert(pack.time_boot_ms == (uint)4225484127U);
                Debug.Assert(pack.image_index == (int)1378937237);
                Debug.Assert(pack.lon == (int) -112341381);
                Debug.Assert(pack.lat == (int)477817395);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.image_index = (int)1378937237;
            p263.lat = (int)477817395;
            p263.camera_id = (byte)(byte)31;
            p263.file_url_SET("pFyeppHgufsmJsl", PH) ;
            p263.time_boot_ms = (uint)4225484127U;
            p263.capture_result = (sbyte)(sbyte)105;
            p263.q_SET(new float[] {6.344913E37F, -6.241367E37F, 1.113607E38F, -3.0645613E38F}, 0) ;
            p263.lon = (int) -112341381;
            p263.time_utc = (ulong)713401655494123576L;
            p263.alt = (int)1798589805;
            p263.relative_alt = (int) -1114323809;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flight_uuid == (ulong)5792901458600077833L);
                Debug.Assert(pack.arming_time_utc == (ulong)7026784851140756961L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)7872041170978174126L);
                Debug.Assert(pack.time_boot_ms == (uint)1637178558U);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.takeoff_time_utc = (ulong)7872041170978174126L;
            p264.flight_uuid = (ulong)5792901458600077833L;
            p264.time_boot_ms = (uint)1637178558U;
            p264.arming_time_utc = (ulong)7026784851140756961L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -2.0017783E38F);
                Debug.Assert(pack.pitch == (float)2.2858507E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1971600401U);
                Debug.Assert(pack.roll == (float) -2.303804E38F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.yaw = (float) -2.0017783E38F;
            p265.roll = (float) -2.303804E38F;
            p265.time_boot_ms = (uint)1971600401U;
            p265.pitch = (float)2.2858507E38F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sequence == (ushort)(ushort)23383);
                Debug.Assert(pack.target_component == (byte)(byte)254);
                Debug.Assert(pack.length == (byte)(byte)190);
                Debug.Assert(pack.first_message_offset == (byte)(byte)98);
                Debug.Assert(pack.target_system == (byte)(byte)210);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)229, (byte)135, (byte)206, (byte)180, (byte)162, (byte)104, (byte)150, (byte)116, (byte)3, (byte)1, (byte)62, (byte)0, (byte)115, (byte)0, (byte)201, (byte)50, (byte)174, (byte)24, (byte)211, (byte)56, (byte)184, (byte)155, (byte)178, (byte)1, (byte)137, (byte)113, (byte)134, (byte)6, (byte)142, (byte)3, (byte)32, (byte)19, (byte)240, (byte)131, (byte)88, (byte)42, (byte)166, (byte)101, (byte)82, (byte)177, (byte)85, (byte)103, (byte)146, (byte)228, (byte)32, (byte)93, (byte)77, (byte)69, (byte)247, (byte)168, (byte)96, (byte)140, (byte)50, (byte)6, (byte)115, (byte)131, (byte)247, (byte)92, (byte)112, (byte)120, (byte)84, (byte)177, (byte)206, (byte)31, (byte)212, (byte)43, (byte)66, (byte)212, (byte)55, (byte)100, (byte)123, (byte)50, (byte)63, (byte)119, (byte)190, (byte)250, (byte)108, (byte)243, (byte)115, (byte)196, (byte)187, (byte)95, (byte)195, (byte)88, (byte)75, (byte)202, (byte)230, (byte)109, (byte)16, (byte)73, (byte)212, (byte)86, (byte)245, (byte)62, (byte)36, (byte)197, (byte)152, (byte)208, (byte)207, (byte)199, (byte)234, (byte)96, (byte)209, (byte)109, (byte)113, (byte)98, (byte)40, (byte)64, (byte)10, (byte)26, (byte)162, (byte)219, (byte)75, (byte)206, (byte)9, (byte)113, (byte)174, (byte)91, (byte)236, (byte)83, (byte)117, (byte)211, (byte)152, (byte)47, (byte)33, (byte)67, (byte)12, (byte)13, (byte)190, (byte)155, (byte)51, (byte)240, (byte)132, (byte)97, (byte)74, (byte)93, (byte)199, (byte)86, (byte)25, (byte)58, (byte)19, (byte)248, (byte)23, (byte)171, (byte)88, (byte)234, (byte)35, (byte)10, (byte)92, (byte)124, (byte)117, (byte)176, (byte)232, (byte)34, (byte)177, (byte)151, (byte)10, (byte)236, (byte)90, (byte)2, (byte)130, (byte)87, (byte)185, (byte)199, (byte)169, (byte)185, (byte)48, (byte)118, (byte)167, (byte)254, (byte)182, (byte)107, (byte)122, (byte)249, (byte)237, (byte)236, (byte)143, (byte)26, (byte)245, (byte)18, (byte)211, (byte)181, (byte)139, (byte)79, (byte)173, (byte)68, (byte)1, (byte)112, (byte)103, (byte)6, (byte)23, (byte)61, (byte)138, (byte)143, (byte)195, (byte)121, (byte)15, (byte)200, (byte)183, (byte)66, (byte)225, (byte)146, (byte)109, (byte)233, (byte)19, (byte)18, (byte)104, (byte)177, (byte)107, (byte)141, (byte)187, (byte)46, (byte)164, (byte)163, (byte)162, (byte)7, (byte)141, (byte)239, (byte)248, (byte)31, (byte)94, (byte)46, (byte)138, (byte)145, (byte)96, (byte)3, (byte)133, (byte)229, (byte)3, (byte)89, (byte)197, (byte)139, (byte)164, (byte)224, (byte)124, (byte)111, (byte)242, (byte)4, (byte)176, (byte)221, (byte)204, (byte)52, (byte)46, (byte)59, (byte)185, (byte)124, (byte)9, (byte)151, (byte)30}));
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.length = (byte)(byte)190;
            p266.data__SET(new byte[] {(byte)229, (byte)135, (byte)206, (byte)180, (byte)162, (byte)104, (byte)150, (byte)116, (byte)3, (byte)1, (byte)62, (byte)0, (byte)115, (byte)0, (byte)201, (byte)50, (byte)174, (byte)24, (byte)211, (byte)56, (byte)184, (byte)155, (byte)178, (byte)1, (byte)137, (byte)113, (byte)134, (byte)6, (byte)142, (byte)3, (byte)32, (byte)19, (byte)240, (byte)131, (byte)88, (byte)42, (byte)166, (byte)101, (byte)82, (byte)177, (byte)85, (byte)103, (byte)146, (byte)228, (byte)32, (byte)93, (byte)77, (byte)69, (byte)247, (byte)168, (byte)96, (byte)140, (byte)50, (byte)6, (byte)115, (byte)131, (byte)247, (byte)92, (byte)112, (byte)120, (byte)84, (byte)177, (byte)206, (byte)31, (byte)212, (byte)43, (byte)66, (byte)212, (byte)55, (byte)100, (byte)123, (byte)50, (byte)63, (byte)119, (byte)190, (byte)250, (byte)108, (byte)243, (byte)115, (byte)196, (byte)187, (byte)95, (byte)195, (byte)88, (byte)75, (byte)202, (byte)230, (byte)109, (byte)16, (byte)73, (byte)212, (byte)86, (byte)245, (byte)62, (byte)36, (byte)197, (byte)152, (byte)208, (byte)207, (byte)199, (byte)234, (byte)96, (byte)209, (byte)109, (byte)113, (byte)98, (byte)40, (byte)64, (byte)10, (byte)26, (byte)162, (byte)219, (byte)75, (byte)206, (byte)9, (byte)113, (byte)174, (byte)91, (byte)236, (byte)83, (byte)117, (byte)211, (byte)152, (byte)47, (byte)33, (byte)67, (byte)12, (byte)13, (byte)190, (byte)155, (byte)51, (byte)240, (byte)132, (byte)97, (byte)74, (byte)93, (byte)199, (byte)86, (byte)25, (byte)58, (byte)19, (byte)248, (byte)23, (byte)171, (byte)88, (byte)234, (byte)35, (byte)10, (byte)92, (byte)124, (byte)117, (byte)176, (byte)232, (byte)34, (byte)177, (byte)151, (byte)10, (byte)236, (byte)90, (byte)2, (byte)130, (byte)87, (byte)185, (byte)199, (byte)169, (byte)185, (byte)48, (byte)118, (byte)167, (byte)254, (byte)182, (byte)107, (byte)122, (byte)249, (byte)237, (byte)236, (byte)143, (byte)26, (byte)245, (byte)18, (byte)211, (byte)181, (byte)139, (byte)79, (byte)173, (byte)68, (byte)1, (byte)112, (byte)103, (byte)6, (byte)23, (byte)61, (byte)138, (byte)143, (byte)195, (byte)121, (byte)15, (byte)200, (byte)183, (byte)66, (byte)225, (byte)146, (byte)109, (byte)233, (byte)19, (byte)18, (byte)104, (byte)177, (byte)107, (byte)141, (byte)187, (byte)46, (byte)164, (byte)163, (byte)162, (byte)7, (byte)141, (byte)239, (byte)248, (byte)31, (byte)94, (byte)46, (byte)138, (byte)145, (byte)96, (byte)3, (byte)133, (byte)229, (byte)3, (byte)89, (byte)197, (byte)139, (byte)164, (byte)224, (byte)124, (byte)111, (byte)242, (byte)4, (byte)176, (byte)221, (byte)204, (byte)52, (byte)46, (byte)59, (byte)185, (byte)124, (byte)9, (byte)151, (byte)30}, 0) ;
            p266.target_system = (byte)(byte)210;
            p266.target_component = (byte)(byte)254;
            p266.sequence = (ushort)(ushort)23383;
            p266.first_message_offset = (byte)(byte)98;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)121);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)107, (byte)65, (byte)52, (byte)52, (byte)26, (byte)149, (byte)149, (byte)174, (byte)219, (byte)60, (byte)63, (byte)254, (byte)16, (byte)4, (byte)53, (byte)154, (byte)78, (byte)33, (byte)132, (byte)176, (byte)30, (byte)111, (byte)47, (byte)189, (byte)152, (byte)219, (byte)88, (byte)192, (byte)113, (byte)242, (byte)222, (byte)33, (byte)193, (byte)4, (byte)43, (byte)60, (byte)51, (byte)121, (byte)102, (byte)12, (byte)177, (byte)168, (byte)136, (byte)45, (byte)79, (byte)93, (byte)90, (byte)10, (byte)196, (byte)19, (byte)245, (byte)116, (byte)219, (byte)191, (byte)41, (byte)38, (byte)51, (byte)3, (byte)84, (byte)214, (byte)160, (byte)185, (byte)247, (byte)83, (byte)51, (byte)82, (byte)5, (byte)48, (byte)140, (byte)62, (byte)238, (byte)144, (byte)175, (byte)181, (byte)104, (byte)239, (byte)149, (byte)179, (byte)170, (byte)250, (byte)128, (byte)188, (byte)19, (byte)77, (byte)161, (byte)73, (byte)67, (byte)139, (byte)218, (byte)179, (byte)156, (byte)6, (byte)66, (byte)122, (byte)235, (byte)201, (byte)111, (byte)54, (byte)115, (byte)57, (byte)23, (byte)95, (byte)71, (byte)78, (byte)62, (byte)240, (byte)125, (byte)39, (byte)137, (byte)191, (byte)195, (byte)111, (byte)200, (byte)169, (byte)72, (byte)247, (byte)236, (byte)128, (byte)247, (byte)9, (byte)144, (byte)150, (byte)15, (byte)175, (byte)104, (byte)176, (byte)55, (byte)194, (byte)142, (byte)22, (byte)35, (byte)92, (byte)215, (byte)116, (byte)135, (byte)0, (byte)174, (byte)218, (byte)241, (byte)248, (byte)44, (byte)83, (byte)148, (byte)172, (byte)246, (byte)70, (byte)218, (byte)65, (byte)190, (byte)69, (byte)39, (byte)51, (byte)147, (byte)136, (byte)142, (byte)112, (byte)3, (byte)217, (byte)74, (byte)25, (byte)23, (byte)68, (byte)8, (byte)197, (byte)28, (byte)198, (byte)129, (byte)140, (byte)132, (byte)192, (byte)17, (byte)88, (byte)172, (byte)179, (byte)66, (byte)35, (byte)114, (byte)13, (byte)43, (byte)184, (byte)199, (byte)159, (byte)64, (byte)1, (byte)225, (byte)111, (byte)5, (byte)240, (byte)39, (byte)156, (byte)137, (byte)233, (byte)74, (byte)22, (byte)250, (byte)215, (byte)18, (byte)146, (byte)207, (byte)252, (byte)198, (byte)60, (byte)202, (byte)210, (byte)201, (byte)81, (byte)155, (byte)18, (byte)94, (byte)185, (byte)158, (byte)2, (byte)17, (byte)159, (byte)184, (byte)58, (byte)137, (byte)177, (byte)62, (byte)28, (byte)2, (byte)30, (byte)48, (byte)164, (byte)41, (byte)112, (byte)147, (byte)146, (byte)233, (byte)39, (byte)187, (byte)100, (byte)249, (byte)28, (byte)238, (byte)60, (byte)29, (byte)2, (byte)63, (byte)108, (byte)110, (byte)39, (byte)107, (byte)241, (byte)111, (byte)3, (byte)45, (byte)59, (byte)169}));
                Debug.Assert(pack.first_message_offset == (byte)(byte)45);
                Debug.Assert(pack.length == (byte)(byte)165);
                Debug.Assert(pack.sequence == (ushort)(ushort)23232);
                Debug.Assert(pack.target_component == (byte)(byte)17);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.length = (byte)(byte)165;
            p267.sequence = (ushort)(ushort)23232;
            p267.target_system = (byte)(byte)121;
            p267.data__SET(new byte[] {(byte)107, (byte)65, (byte)52, (byte)52, (byte)26, (byte)149, (byte)149, (byte)174, (byte)219, (byte)60, (byte)63, (byte)254, (byte)16, (byte)4, (byte)53, (byte)154, (byte)78, (byte)33, (byte)132, (byte)176, (byte)30, (byte)111, (byte)47, (byte)189, (byte)152, (byte)219, (byte)88, (byte)192, (byte)113, (byte)242, (byte)222, (byte)33, (byte)193, (byte)4, (byte)43, (byte)60, (byte)51, (byte)121, (byte)102, (byte)12, (byte)177, (byte)168, (byte)136, (byte)45, (byte)79, (byte)93, (byte)90, (byte)10, (byte)196, (byte)19, (byte)245, (byte)116, (byte)219, (byte)191, (byte)41, (byte)38, (byte)51, (byte)3, (byte)84, (byte)214, (byte)160, (byte)185, (byte)247, (byte)83, (byte)51, (byte)82, (byte)5, (byte)48, (byte)140, (byte)62, (byte)238, (byte)144, (byte)175, (byte)181, (byte)104, (byte)239, (byte)149, (byte)179, (byte)170, (byte)250, (byte)128, (byte)188, (byte)19, (byte)77, (byte)161, (byte)73, (byte)67, (byte)139, (byte)218, (byte)179, (byte)156, (byte)6, (byte)66, (byte)122, (byte)235, (byte)201, (byte)111, (byte)54, (byte)115, (byte)57, (byte)23, (byte)95, (byte)71, (byte)78, (byte)62, (byte)240, (byte)125, (byte)39, (byte)137, (byte)191, (byte)195, (byte)111, (byte)200, (byte)169, (byte)72, (byte)247, (byte)236, (byte)128, (byte)247, (byte)9, (byte)144, (byte)150, (byte)15, (byte)175, (byte)104, (byte)176, (byte)55, (byte)194, (byte)142, (byte)22, (byte)35, (byte)92, (byte)215, (byte)116, (byte)135, (byte)0, (byte)174, (byte)218, (byte)241, (byte)248, (byte)44, (byte)83, (byte)148, (byte)172, (byte)246, (byte)70, (byte)218, (byte)65, (byte)190, (byte)69, (byte)39, (byte)51, (byte)147, (byte)136, (byte)142, (byte)112, (byte)3, (byte)217, (byte)74, (byte)25, (byte)23, (byte)68, (byte)8, (byte)197, (byte)28, (byte)198, (byte)129, (byte)140, (byte)132, (byte)192, (byte)17, (byte)88, (byte)172, (byte)179, (byte)66, (byte)35, (byte)114, (byte)13, (byte)43, (byte)184, (byte)199, (byte)159, (byte)64, (byte)1, (byte)225, (byte)111, (byte)5, (byte)240, (byte)39, (byte)156, (byte)137, (byte)233, (byte)74, (byte)22, (byte)250, (byte)215, (byte)18, (byte)146, (byte)207, (byte)252, (byte)198, (byte)60, (byte)202, (byte)210, (byte)201, (byte)81, (byte)155, (byte)18, (byte)94, (byte)185, (byte)158, (byte)2, (byte)17, (byte)159, (byte)184, (byte)58, (byte)137, (byte)177, (byte)62, (byte)28, (byte)2, (byte)30, (byte)48, (byte)164, (byte)41, (byte)112, (byte)147, (byte)146, (byte)233, (byte)39, (byte)187, (byte)100, (byte)249, (byte)28, (byte)238, (byte)60, (byte)29, (byte)2, (byte)63, (byte)108, (byte)110, (byte)39, (byte)107, (byte)241, (byte)111, (byte)3, (byte)45, (byte)59, (byte)169}, 0) ;
            p267.target_component = (byte)(byte)17;
            p267.first_message_offset = (byte)(byte)45;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)9);
                Debug.Assert(pack.target_system == (byte)(byte)122);
                Debug.Assert(pack.sequence == (ushort)(ushort)14674);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)122;
            p268.target_component = (byte)(byte)9;
            p268.sequence = (ushort)(ushort)14674;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.camera_id == (byte)(byte)59);
                Debug.Assert(pack.uri_LEN(ph) == 71);
                Debug.Assert(pack.uri_TRY(ph).Equals("ikhdsmjXjkoqfpsockedpdVowHxgwerrbaHgamtgkbzshwnwnckybykauwWucdYaAjqgimc"));
                Debug.Assert(pack.rotation == (ushort)(ushort)41738);
                Debug.Assert(pack.status == (byte)(byte)151);
                Debug.Assert(pack.framerate == (float) -8.881537E37F);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)29710);
                Debug.Assert(pack.bitrate == (uint)1500407154U);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)14856);
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)59;
            p269.resolution_h = (ushort)(ushort)14856;
            p269.rotation = (ushort)(ushort)41738;
            p269.bitrate = (uint)1500407154U;
            p269.status = (byte)(byte)151;
            p269.uri_SET("ikhdsmjXjkoqfpsockedpdVowHxgwerrbaHgamtgkbzshwnwnckybykauwWucdYaAjqgimc", PH) ;
            p269.framerate = (float) -8.881537E37F;
            p269.resolution_v = (ushort)(ushort)29710;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)185);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)40379);
                Debug.Assert(pack.target_component == (byte)(byte)92);
                Debug.Assert(pack.uri_LEN(ph) == 220);
                Debug.Assert(pack.uri_TRY(ph).Equals("bfdufrztshwbiqdflsxykjcbxlvqkLgfKebrbvpiaAvglklbuWvXhNakyzgfppamqsieldpnGxeHvAbacpevnihhvmsoyuqqchorwbboqsdhHqkNgzadrgfPhccwbvtIuuwoUwYdvpdguebkkynwgyhlftpkgffLjpxHbbegfbuylwcnjllbaughmhfmdyplajxtuLbvwiyzutvjxiaaigmpcjpl"));
                Debug.Assert(pack.framerate == (float)1.485923E37F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)11820);
                Debug.Assert(pack.rotation == (ushort)(ushort)61293);
                Debug.Assert(pack.bitrate == (uint)3268619675U);
                Debug.Assert(pack.camera_id == (byte)(byte)201);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.resolution_v = (ushort)(ushort)40379;
            p270.framerate = (float)1.485923E37F;
            p270.camera_id = (byte)(byte)201;
            p270.resolution_h = (ushort)(ushort)11820;
            p270.target_component = (byte)(byte)92;
            p270.rotation = (ushort)(ushort)61293;
            p270.bitrate = (uint)3268619675U;
            p270.target_system = (byte)(byte)185;
            p270.uri_SET("bfdufrztshwbiqdflsxykjcbxlvqkLgfKebrbvpiaAvglklbuWvXhNakyzgfppamqsieldpnGxeHvAbacpevnihhvmsoyuqqchorwbboqsdhHqkNgzadrgfPhccwbvtIuuwoUwYdvpdguebkkynwgyhlftpkgffLjpxHbbegfbuylwcnjllbaughmhfmdyplajxtuLbvwiyzutvjxiaaigmpcjpl", PH) ;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ssid_LEN(ph) == 24);
                Debug.Assert(pack.ssid_TRY(ph).Equals("zdttUaekyfbdUyrqyXfjVtiy"));
                Debug.Assert(pack.password_LEN(ph) == 19);
                Debug.Assert(pack.password_TRY(ph).Equals("DDijfoWyhbvlZrkfffn"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("zdttUaekyfbdUyrqyXfjVtiy", PH) ;
            p299.password_SET("DDijfoWyhbvlZrkfffn", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.max_version == (ushort)(ushort)54015);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)53, (byte)206, (byte)86, (byte)154, (byte)123, (byte)193, (byte)155, (byte)69}));
                Debug.Assert(pack.version == (ushort)(ushort)56822);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)166, (byte)197, (byte)90, (byte)163, (byte)122, (byte)234, (byte)148, (byte)90}));
                Debug.Assert(pack.min_version == (ushort)(ushort)30668);
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.library_version_hash_SET(new byte[] {(byte)166, (byte)197, (byte)90, (byte)163, (byte)122, (byte)234, (byte)148, (byte)90}, 0) ;
            p300.min_version = (ushort)(ushort)30668;
            p300.version = (ushort)(ushort)56822;
            p300.spec_version_hash_SET(new byte[] {(byte)53, (byte)206, (byte)86, (byte)154, (byte)123, (byte)193, (byte)155, (byte)69}, 0) ;
            p300.max_version = (ushort)(ushort)54015;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)51001);
                Debug.Assert(pack.uptime_sec == (uint)766270281U);
                Debug.Assert(pack.sub_mode == (byte)(byte)17);
                Debug.Assert(pack.health == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
                Debug.Assert(pack.time_usec == (ulong)7459439037052308922L);
                Debug.Assert(pack.mode == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK;
            p310.vendor_specific_status_code = (ushort)(ushort)51001;
            p310.uptime_sec = (uint)766270281U;
            p310.sub_mode = (byte)(byte)17;
            p310.time_usec = (ulong)7459439037052308922L;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uptime_sec == (uint)2973617361U);
                Debug.Assert(pack.sw_version_major == (byte)(byte)16);
                Debug.Assert(pack.sw_vcs_commit == (uint)219704259U);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)193);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)254);
                Debug.Assert(pack.name_LEN(ph) == 24);
                Debug.Assert(pack.name_TRY(ph).Equals("yscfzWpbbqfllJqxYFsucqyp"));
                Debug.Assert(pack.hw_version_major == (byte)(byte)39);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)170, (byte)17, (byte)248, (byte)45, (byte)147, (byte)87, (byte)59, (byte)70, (byte)114, (byte)132, (byte)92, (byte)94, (byte)94, (byte)139, (byte)138, (byte)126}));
                Debug.Assert(pack.time_usec == (ulong)4347810467229106123L);
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.uptime_sec = (uint)2973617361U;
            p311.hw_version_minor = (byte)(byte)254;
            p311.sw_vcs_commit = (uint)219704259U;
            p311.sw_version_minor = (byte)(byte)193;
            p311.hw_unique_id_SET(new byte[] {(byte)170, (byte)17, (byte)248, (byte)45, (byte)147, (byte)87, (byte)59, (byte)70, (byte)114, (byte)132, (byte)92, (byte)94, (byte)94, (byte)139, (byte)138, (byte)126}, 0) ;
            p311.hw_version_major = (byte)(byte)39;
            p311.name_SET("yscfzWpbbqfllJqxYFsucqyp", PH) ;
            p311.sw_version_major = (byte)(byte)16;
            p311.time_usec = (ulong)4347810467229106123L;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 4);
                Debug.Assert(pack.param_id_TRY(ph).Equals("hbfw"));
                Debug.Assert(pack.param_index == (short)(short)6663);
                Debug.Assert(pack.target_component == (byte)(byte)116);
                Debug.Assert(pack.target_system == (byte)(byte)223);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)223;
            p320.param_index = (short)(short)6663;
            p320.param_id_SET("hbfw", PH) ;
            p320.target_component = (byte)(byte)116;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)26);
                Debug.Assert(pack.target_component == (byte)(byte)49);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_component = (byte)(byte)49;
            p321.target_system = (byte)(byte)26;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 2);
                Debug.Assert(pack.param_value_TRY(ph).Equals("lk"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
                Debug.Assert(pack.param_count == (ushort)(ushort)351);
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("tlH"));
                Debug.Assert(pack.param_index == (ushort)(ushort)8080);
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_value_SET("lk", PH) ;
            p322.param_index = (ushort)(ushort)8080;
            p322.param_id_SET("tlH", PH) ;
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            p322.param_count = (ushort)(ushort)351;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 115);
                Debug.Assert(pack.param_value_TRY(ph).Equals("ruCjfvjbjaarwtkrfcveimtpuuusuxJnwmtatdkynjzjpmwOcdsmioNjjndwoflNghncovwdwhlyksnkogtxgDwsuuQxzqdnwfisrzQpdnsnbkxzecZ"));
                Debug.Assert(pack.param_id_LEN(ph) == 16);
                Debug.Assert(pack.param_id_TRY(ph).Equals("tiibXkvnruqcbxgn"));
                Debug.Assert(pack.target_component == (byte)(byte)213);
                Debug.Assert(pack.target_system == (byte)(byte)65);
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_id_SET("tiibXkvnruqcbxgn", PH) ;
            p323.target_system = (byte)(byte)65;
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8;
            p323.param_value_SET("ruCjfvjbjaarwtkrfcveimtpuuusuxJnwmtatdkynjzjpmwOcdsmioNjjndwoflNghncovwdwhlyksnkogtxgDwsuuQxzqdnwfisrzQpdnsnbkxzecZ", PH) ;
            p323.target_component = (byte)(byte)213;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("Glh"));
                Debug.Assert(pack.param_result == PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
                Debug.Assert(pack.param_value_LEN(ph) == 44);
                Debug.Assert(pack.param_value_TRY(ph).Equals("nmboqezdugmguvsdssfjduoqfDwTrvbrarjcmbhcfqMe"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_result = PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED;
            p324.param_id_SET("Glh", PH) ;
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            p324.param_value_SET("nmboqezdugmguvsdssfjduoqfDwTrvbrarjcmbhcfqMe", PH) ;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.increment == (byte)(byte)55);
                Debug.Assert(pack.sensor_type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
                Debug.Assert(pack.time_usec == (ulong)2689512971263581990L);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)62837, (ushort)42720, (ushort)4443, (ushort)51670, (ushort)53103, (ushort)31325, (ushort)2758, (ushort)36041, (ushort)5425, (ushort)39939, (ushort)3295, (ushort)59795, (ushort)6711, (ushort)49806, (ushort)6972, (ushort)64765, (ushort)24867, (ushort)16209, (ushort)7328, (ushort)49589, (ushort)17991, (ushort)18859, (ushort)58143, (ushort)15165, (ushort)13415, (ushort)4523, (ushort)23428, (ushort)38139, (ushort)37193, (ushort)17708, (ushort)2862, (ushort)35711, (ushort)38068, (ushort)17187, (ushort)50987, (ushort)11479, (ushort)21685, (ushort)61139, (ushort)61900, (ushort)45853, (ushort)44776, (ushort)48199, (ushort)48106, (ushort)15913, (ushort)47008, (ushort)36900, (ushort)23224, (ushort)55037, (ushort)49833, (ushort)42675, (ushort)16501, (ushort)39409, (ushort)60260, (ushort)59106, (ushort)25562, (ushort)51211, (ushort)36390, (ushort)12636, (ushort)42220, (ushort)38508, (ushort)6132, (ushort)40373, (ushort)23341, (ushort)26676, (ushort)13901, (ushort)3439, (ushort)1510, (ushort)49498, (ushort)44754, (ushort)46895, (ushort)14978, (ushort)39254}));
                Debug.Assert(pack.min_distance == (ushort)(ushort)39693);
                Debug.Assert(pack.max_distance == (ushort)(ushort)62880);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.min_distance = (ushort)(ushort)39693;
            p330.increment = (byte)(byte)55;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN;
            p330.distances_SET(new ushort[] {(ushort)62837, (ushort)42720, (ushort)4443, (ushort)51670, (ushort)53103, (ushort)31325, (ushort)2758, (ushort)36041, (ushort)5425, (ushort)39939, (ushort)3295, (ushort)59795, (ushort)6711, (ushort)49806, (ushort)6972, (ushort)64765, (ushort)24867, (ushort)16209, (ushort)7328, (ushort)49589, (ushort)17991, (ushort)18859, (ushort)58143, (ushort)15165, (ushort)13415, (ushort)4523, (ushort)23428, (ushort)38139, (ushort)37193, (ushort)17708, (ushort)2862, (ushort)35711, (ushort)38068, (ushort)17187, (ushort)50987, (ushort)11479, (ushort)21685, (ushort)61139, (ushort)61900, (ushort)45853, (ushort)44776, (ushort)48199, (ushort)48106, (ushort)15913, (ushort)47008, (ushort)36900, (ushort)23224, (ushort)55037, (ushort)49833, (ushort)42675, (ushort)16501, (ushort)39409, (ushort)60260, (ushort)59106, (ushort)25562, (ushort)51211, (ushort)36390, (ushort)12636, (ushort)42220, (ushort)38508, (ushort)6132, (ushort)40373, (ushort)23341, (ushort)26676, (ushort)13901, (ushort)3439, (ushort)1510, (ushort)49498, (ushort)44754, (ushort)46895, (ushort)14978, (ushort)39254}, 0) ;
            p330.max_distance = (ushort)(ushort)62880;
            p330.time_usec = (ulong)2689512971263581990L;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
        }
    }
}