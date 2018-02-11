
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
                    ulong id = id__s(value);
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
            *	 0 ignore*/
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
            *	 (2) Distance in cm (3) Absolute valu*/
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
            *	 mode, 7=Manual input mode (fixed position), 8=Simulator mode, 9= WAAS*/
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
            *	 (W) adds to True cours*/
            public sbyte magDir
            {
                get {  return (sbyte)((sbyte) BitUtils.get_bytes(data,  9, 1));}
            }

            /**
            *Positioning system mode indicator. A - Autonomous;D-Differential; E-Estimated (dead reckoning) mode;M-Manual
            *	 input; N-Data not vali*/
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
            *	 the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed
            *	 on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer,
            *	 while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered
            *	 fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment
            *	 with a non full payload is received. This management is used to ensure that normal GPS operation doesn't
            *	 corrupt RTCM data, and to recover from a unreliable transport delivery order*/
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
            *	 bit3:GCS, bit4:fence*/
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
            *	 and slope of the groun*/
            public float[] q
            {
                get {return q_GET(new float[4], 0);}
            }
            /**
            *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
            *	 and slope of the groun*/
            public float[]q_GET(float[] dst_ch, int pos)
            {
                for(int BYTE = 24, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            /**
            *Local X position of the end of the approach vector. Multicopters should set this position based on their
            *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	 from the threshold / touchdown zone*/
            public float approach_x
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  40, 4)));}
            }

            /**
            *Local Y position of the end of the approach vector. Multicopters should set this position based on their
            *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	 from the threshold / touchdown zone*/
            public float approach_y
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  44, 4)));}
            }

            /**
            *Local Z position of the end of the approach vector. Multicopters should set this position based on their
            *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	 from the threshold / touchdown zone*/
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
            *	 and slope of the groun*/
            public float[] q
            {
                get {return q_GET(new float[4], 0);}
            }
            /**
            *World to surface normal and heading transformation of the takeoff position. Used to indicate the heading
            *	 and slope of the groun*/
            public float[]q_GET(float[] dst_ch, int pos)
            {
                for(int BYTE = 25, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  BYTE, 4)));
                return dst_ch;
            }
            /**
            *Local X position of the end of the approach vector. Multicopters should set this position based on their
            *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	 from the threshold / touchdown zone*/
            public float approach_x
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  41, 4)));}
            }

            /**
            *Local Y position of the end of the approach vector. Multicopters should set this position based on their
            *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	 from the threshold / touchdown zone*/
            public float approach_y
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  45, 4)));}
            }

            /**
            *Local Z position of the end of the approach vector. Multicopters should set this position based on their
            *	 takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing
            *	 fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened
            *	 from the threshold / touchdown zone*/
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
                get {  return (MAV_COLLISION_THREAT_LEVEL)(0 +  BitUtils.get_bits(data, 132, 3));}
            }
        }
        new class V2_EXTENSION : GroundControl.V2_EXTENSION
        {
            /**
            *A code that identifies the software component that understands this message (analogous to usb device classes
            *	 or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension
            *	 and the corresponding entry should be added to https:github.com/mavlink/mavlink/extension-message-ids.xml.
            *	 Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...).
            *	 Message_types greater than 32767 are considered local experiments and should not be checked in to any
            *	 widely distributed codebase*/
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
            *	 and other fields.  The entire content of this block is opaque unless you understand any the encoding
            *	 message_type.  The particular encoding used can be extension specific and might not always be documented
            *	 as part of the mavlink specification*/
            public byte[] payload
            {
                get {return payload_GET(new byte[249], 0);}
            }
            /**
            *Variable length payload. The length is defined by the remaining message length when subtracting the header
            *	 and other fields.  The entire content of this block is opaque unless you understand any the encoding
            *	 message_type.  The particular encoding used can be extension specific and might not always be documented
            *	 as part of the mavlink specification*/
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
                get {  return (CAMERA_MODE)(0 +  BitUtils.get_bits(data, 32, 3));}
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
            *	 set and capture in progress*/
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
                Debug.Assert(pack.system_status == MAV_STATE.MAV_STATE_ACTIVE);
                Debug.Assert(pack.autopilot == MAV_AUTOPILOT.MAV_AUTOPILOT_SLUGS);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED));
                Debug.Assert(pack.type == MAV_TYPE.MAV_TYPE_FIXED_WING);
                Debug.Assert(pack.mavlink_version == (byte)(byte)124);
                Debug.Assert(pack.custom_mode == (uint)3560604132U);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_SLUGS;
            p0.system_status = MAV_STATE.MAV_STATE_ACTIVE;
            p0.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED);
            p0.type = MAV_TYPE.MAV_TYPE_FIXED_WING;
            p0.mavlink_version = (byte)(byte)124;
            p0.custom_mode = (uint)3560604132U;
            SMP_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)1545);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)47264);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 33);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)38884);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)4578);
                Debug.Assert(pack.current_battery == (short)(short)19439);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)58570);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)5224);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
                Debug.Assert(pack.load == (ushort)(ushort)34093);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)11122);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.errors_count3 = (ushort)(ushort)38884;
            p1.errors_count2 = (ushort)(ushort)4578;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.errors_count1 = (ushort)(ushort)5224;
            p1.drop_rate_comm = (ushort)(ushort)58570;
            p1.errors_comm = (ushort)(ushort)47264;
            p1.voltage_battery = (ushort)(ushort)1545;
            p1.load = (ushort)(ushort)34093;
            p1.errors_count4 = (ushort)(ushort)11122;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL);
            p1.current_battery = (short)(short)19439;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.battery_remaining = (sbyte)(sbyte) - 33;
            SMP_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)701290841919845855L);
                Debug.Assert(pack.time_boot_ms == (uint)2957198293U);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)701290841919845855L;
            p2.time_boot_ms = (uint)2957198293U;
            SMP_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3118888747U);
                Debug.Assert(pack.yaw_rate == (float) -4.565846E37F);
                Debug.Assert(pack.y == (float) -2.072182E38F);
                Debug.Assert(pack.afz == (float)9.287499E37F);
                Debug.Assert(pack.yaw == (float)8.957678E37F);
                Debug.Assert(pack.z == (float)7.566768E37F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.afy == (float) -1.1418364E38F);
                Debug.Assert(pack.vz == (float)1.0077071E38F);
                Debug.Assert(pack.x == (float) -1.4589398E38F);
                Debug.Assert(pack.vx == (float)2.3102011E38F);
                Debug.Assert(pack.afx == (float)2.5753328E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)4162);
                Debug.Assert(pack.vy == (float) -1.0537493E38F);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.yaw_rate = (float) -4.565846E37F;
            p3.z = (float)7.566768E37F;
            p3.afy = (float) -1.1418364E38F;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p3.vx = (float)2.3102011E38F;
            p3.y = (float) -2.072182E38F;
            p3.afz = (float)9.287499E37F;
            p3.time_boot_ms = (uint)3118888747U;
            p3.vz = (float)1.0077071E38F;
            p3.x = (float) -1.4589398E38F;
            p3.afx = (float)2.5753328E38F;
            p3.type_mask = (ushort)(ushort)4162;
            p3.vy = (float) -1.0537493E38F;
            p3.yaw = (float)8.957678E37F;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)5259096908802595064L);
                Debug.Assert(pack.target_system == (byte)(byte)175);
                Debug.Assert(pack.seq == (uint)431225810U);
                Debug.Assert(pack.target_component == (byte)(byte)197);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.target_system = (byte)(byte)175;
            p4.time_usec = (ulong)5259096908802595064L;
            p4.target_component = (byte)(byte)197;
            p4.seq = (uint)431225810U;
            SMP_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.control_request == (byte)(byte)241);
                Debug.Assert(pack.target_system == (byte)(byte)129);
                Debug.Assert(pack.version == (byte)(byte)159);
                Debug.Assert(pack.passkey_LEN(ph) == 23);
                Debug.Assert(pack.passkey_TRY(ph).Equals("pdwoiuoPxsTxarhzqkdQgcG"));
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.target_system = (byte)(byte)129;
            p5.control_request = (byte)(byte)241;
            p5.version = (byte)(byte)159;
            p5.passkey_SET("pdwoiuoPxsTxarhzqkdQgcG", PH) ;
            SMP_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ack == (byte)(byte)79);
                Debug.Assert(pack.control_request == (byte)(byte)68);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)95);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.ack = (byte)(byte)79;
            p6.gcs_system_id = (byte)(byte)95;
            p6.control_request = (byte)(byte)68;
            SMP_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 10);
                Debug.Assert(pack.key_TRY(ph).Equals("jwfeHjkhdf"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("jwfeHjkhdf", PH) ;
            SMP_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)145);
                Debug.Assert(pack.custom_mode == (uint)4193028707U);
                Debug.Assert(pack.base_mode == MAV_MODE.MAV_MODE_TEST_ARMED);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)145;
            p11.base_mode = MAV_MODE.MAV_MODE_TEST_ARMED;
            p11.custom_mode = (uint)4193028707U;
            SMP_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)208);
                Debug.Assert(pack.target_system == (byte)(byte)195);
                Debug.Assert(pack.param_index == (short)(short) -20808);
                Debug.Assert(pack.param_id_LEN(ph) == 4);
                Debug.Assert(pack.param_id_TRY(ph).Equals("xhzp"));
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.param_index = (short)(short) -20808;
            p20.target_system = (byte)(byte)195;
            p20.target_component = (byte)(byte)208;
            p20.param_id_SET("xhzp", PH) ;
            SMP_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)170);
                Debug.Assert(pack.target_system == (byte)(byte)188);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)188;
            p21.target_component = (byte)(byte)170;
            SMP_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_count == (ushort)(ushort)42605);
                Debug.Assert(pack.param_id_LEN(ph) == 4);
                Debug.Assert(pack.param_id_TRY(ph).Equals("dFTx"));
                Debug.Assert(pack.param_index == (ushort)(ushort)3328);
                Debug.Assert(pack.param_value == (float)1.3852073E38F);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8);
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_index = (ushort)(ushort)3328;
            p22.param_id_SET("dFTx", PH) ;
            p22.param_count = (ushort)(ushort)42605;
            p22.param_value = (float)1.3852073E38F;
            p22.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT8;
            SMP_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value == (float) -3.1848074E38F);
                Debug.Assert(pack.target_system == (byte)(byte)166);
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("B"));
                Debug.Assert(pack.target_component == (byte)(byte)250);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.param_value = (float) -3.1848074E38F;
            p23.param_id_SET("B", PH) ;
            p23.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64;
            p23.target_system = (byte)(byte)166;
            p23.target_component = (byte)(byte)250;
            SMP_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)1206163690U);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)1512021431U);
                Debug.Assert(pack.time_usec == (ulong)3420561808591850270L);
                Debug.Assert(pack.lon == (int) -330799857);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)3914395822U);
                Debug.Assert(pack.lat == (int)1315207429);
                Debug.Assert(pack.eph == (ushort)(ushort)322);
                Debug.Assert(pack.satellites_visible == (byte)(byte)142);
                Debug.Assert(pack.cog == (ushort)(ushort)36361);
                Debug.Assert(pack.alt == (int)822352730);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int)1690628406);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)1782261683U);
                Debug.Assert(pack.epv == (ushort)(ushort)10647);
                Debug.Assert(pack.vel == (ushort)(ushort)30464);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.eph = (ushort)(ushort)322;
            p24.lat = (int)1315207429;
            p24.v_acc_SET((uint)1512021431U, PH) ;
            p24.epv = (ushort)(ushort)10647;
            p24.alt = (int)822352730;
            p24.alt_ellipsoid_SET((int)1690628406, PH) ;
            p24.h_acc_SET((uint)3914395822U, PH) ;
            p24.time_usec = (ulong)3420561808591850270L;
            p24.satellites_visible = (byte)(byte)142;
            p24.lon = (int) -330799857;
            p24.cog = (ushort)(ushort)36361;
            p24.vel_acc_SET((uint)1782261683U, PH) ;
            p24.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p24.vel = (ushort)(ushort)30464;
            p24.hdg_acc_SET((uint)1206163690U, PH) ;
            SMP_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)51, (byte)102, (byte)184, (byte)80, (byte)20, (byte)249, (byte)134, (byte)190, (byte)62, (byte)224, (byte)123, (byte)64, (byte)79, (byte)84, (byte)187, (byte)255, (byte)62, (byte)47, (byte)48, (byte)32}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)53, (byte)164, (byte)42, (byte)134, (byte)129, (byte)120, (byte)216, (byte)48, (byte)97, (byte)35, (byte)178, (byte)225, (byte)124, (byte)97, (byte)34, (byte)178, (byte)0, (byte)71, (byte)153, (byte)202}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)110, (byte)39, (byte)208, (byte)23, (byte)78, (byte)74, (byte)189, (byte)82, (byte)11, (byte)101, (byte)73, (byte)94, (byte)168, (byte)17, (byte)182, (byte)248, (byte)4, (byte)54, (byte)222, (byte)157}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)246);
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)99, (byte)107, (byte)110, (byte)38, (byte)111, (byte)137, (byte)140, (byte)113, (byte)72, (byte)29, (byte)95, (byte)65, (byte)222, (byte)105, (byte)227, (byte)102, (byte)153, (byte)34, (byte)192, (byte)27}));
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)108, (byte)145, (byte)242, (byte)212, (byte)197, (byte)159, (byte)63, (byte)247, (byte)28, (byte)139, (byte)14, (byte)206, (byte)211, (byte)253, (byte)78, (byte)247, (byte)96, (byte)197, (byte)174, (byte)194}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_elevation_SET(new byte[] {(byte)108, (byte)145, (byte)242, (byte)212, (byte)197, (byte)159, (byte)63, (byte)247, (byte)28, (byte)139, (byte)14, (byte)206, (byte)211, (byte)253, (byte)78, (byte)247, (byte)96, (byte)197, (byte)174, (byte)194}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)53, (byte)164, (byte)42, (byte)134, (byte)129, (byte)120, (byte)216, (byte)48, (byte)97, (byte)35, (byte)178, (byte)225, (byte)124, (byte)97, (byte)34, (byte)178, (byte)0, (byte)71, (byte)153, (byte)202}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)51, (byte)102, (byte)184, (byte)80, (byte)20, (byte)249, (byte)134, (byte)190, (byte)62, (byte)224, (byte)123, (byte)64, (byte)79, (byte)84, (byte)187, (byte)255, (byte)62, (byte)47, (byte)48, (byte)32}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)110, (byte)39, (byte)208, (byte)23, (byte)78, (byte)74, (byte)189, (byte)82, (byte)11, (byte)101, (byte)73, (byte)94, (byte)168, (byte)17, (byte)182, (byte)248, (byte)4, (byte)54, (byte)222, (byte)157}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)99, (byte)107, (byte)110, (byte)38, (byte)111, (byte)137, (byte)140, (byte)113, (byte)72, (byte)29, (byte)95, (byte)65, (byte)222, (byte)105, (byte)227, (byte)102, (byte)153, (byte)34, (byte)192, (byte)27}, 0) ;
            p25.satellites_visible = (byte)(byte)246;
            SMP_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2611723334U);
                Debug.Assert(pack.zgyro == (short)(short) -4200);
                Debug.Assert(pack.yacc == (short)(short) -23849);
                Debug.Assert(pack.ygyro == (short)(short)12311);
                Debug.Assert(pack.xmag == (short)(short) -22604);
                Debug.Assert(pack.xgyro == (short)(short) -25560);
                Debug.Assert(pack.xacc == (short)(short)8160);
                Debug.Assert(pack.zmag == (short)(short)29862);
                Debug.Assert(pack.ymag == (short)(short)2264);
                Debug.Assert(pack.zacc == (short)(short)31550);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.zgyro = (short)(short) -4200;
            p26.ymag = (short)(short)2264;
            p26.yacc = (short)(short) -23849;
            p26.time_boot_ms = (uint)2611723334U;
            p26.zacc = (short)(short)31550;
            p26.xgyro = (short)(short) -25560;
            p26.xacc = (short)(short)8160;
            p26.zmag = (short)(short)29862;
            p26.ygyro = (short)(short)12311;
            p26.xmag = (short)(short) -22604;
            SMP_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (short)(short) -17218);
                Debug.Assert(pack.xacc == (short)(short) -1674);
                Debug.Assert(pack.xgyro == (short)(short) -15990);
                Debug.Assert(pack.time_usec == (ulong)7963502259320729153L);
                Debug.Assert(pack.zacc == (short)(short)21539);
                Debug.Assert(pack.zgyro == (short)(short)4117);
                Debug.Assert(pack.xmag == (short)(short) -17073);
                Debug.Assert(pack.ymag == (short)(short) -1447);
                Debug.Assert(pack.zmag == (short)(short)30954);
                Debug.Assert(pack.yacc == (short)(short) -3654);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.zmag = (short)(short)30954;
            p27.time_usec = (ulong)7963502259320729153L;
            p27.zgyro = (short)(short)4117;
            p27.zacc = (short)(short)21539;
            p27.xacc = (short)(short) -1674;
            p27.xmag = (short)(short) -17073;
            p27.ygyro = (short)(short) -17218;
            p27.xgyro = (short)(short) -15990;
            p27.ymag = (short)(short) -1447;
            p27.yacc = (short)(short) -3654;
            SMP_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)4311967461198958994L);
                Debug.Assert(pack.press_abs == (short)(short) -17790);
                Debug.Assert(pack.press_diff1 == (short)(short) -23922);
                Debug.Assert(pack.temperature == (short)(short) -9020);
                Debug.Assert(pack.press_diff2 == (short)(short)29915);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_diff1 = (short)(short) -23922;
            p28.temperature = (short)(short) -9020;
            p28.press_abs = (short)(short) -17790;
            p28.time_usec = (ulong)4311967461198958994L;
            p28.press_diff2 = (short)(short)29915;
            SMP_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2706071556U);
                Debug.Assert(pack.press_abs == (float) -1.3715577E38F);
                Debug.Assert(pack.press_diff == (float)2.217893E38F);
                Debug.Assert(pack.temperature == (short)(short)26567);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.temperature = (short)(short)26567;
            p29.press_diff = (float)2.217893E38F;
            p29.press_abs = (float) -1.3715577E38F;
            p29.time_boot_ms = (uint)2706071556U;
            SMP_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yawspeed == (float) -9.983577E37F);
                Debug.Assert(pack.yaw == (float)1.0547214E38F);
                Debug.Assert(pack.roll == (float)3.337787E38F);
                Debug.Assert(pack.rollspeed == (float) -2.5750702E37F);
                Debug.Assert(pack.time_boot_ms == (uint)882704565U);
                Debug.Assert(pack.pitch == (float) -1.5626332E38F);
                Debug.Assert(pack.pitchspeed == (float) -1.9522146E38F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.yawspeed = (float) -9.983577E37F;
            p30.time_boot_ms = (uint)882704565U;
            p30.pitchspeed = (float) -1.9522146E38F;
            p30.roll = (float)3.337787E38F;
            p30.rollspeed = (float) -2.5750702E37F;
            p30.yaw = (float)1.0547214E38F;
            p30.pitch = (float) -1.5626332E38F;
            SMP_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float) -3.3608041E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3087313322U);
                Debug.Assert(pack.q3 == (float)2.4388247E38F);
                Debug.Assert(pack.yawspeed == (float) -1.3321983E38F);
                Debug.Assert(pack.q4 == (float) -1.6133913E38F);
                Debug.Assert(pack.q1 == (float)5.9766465E37F);
                Debug.Assert(pack.q2 == (float)5.153109E37F);
                Debug.Assert(pack.pitchspeed == (float)9.032766E37F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.q3 = (float)2.4388247E38F;
            p31.rollspeed = (float) -3.3608041E37F;
            p31.q1 = (float)5.9766465E37F;
            p31.time_boot_ms = (uint)3087313322U;
            p31.q2 = (float)5.153109E37F;
            p31.q4 = (float) -1.6133913E38F;
            p31.yawspeed = (float) -1.3321983E38F;
            p31.pitchspeed = (float)9.032766E37F;
            SMP_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float)8.424843E37F);
                Debug.Assert(pack.z == (float) -2.32872E38F);
                Debug.Assert(pack.x == (float) -2.5801693E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4123900303U);
                Debug.Assert(pack.y == (float) -2.2051072E38F);
                Debug.Assert(pack.vz == (float)2.2435836E38F);
                Debug.Assert(pack.vy == (float)1.11461E38F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.time_boot_ms = (uint)4123900303U;
            p32.z = (float) -2.32872E38F;
            p32.vz = (float)2.2435836E38F;
            p32.y = (float) -2.2051072E38F;
            p32.vx = (float)8.424843E37F;
            p32.vy = (float)1.11461E38F;
            p32.x = (float) -2.5801693E38F;
            SMP_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hdg == (ushort)(ushort)13292);
                Debug.Assert(pack.vy == (short)(short)20401);
                Debug.Assert(pack.relative_alt == (int)804810153);
                Debug.Assert(pack.time_boot_ms == (uint)2909578858U);
                Debug.Assert(pack.lon == (int)1430169975);
                Debug.Assert(pack.alt == (int) -302842896);
                Debug.Assert(pack.vx == (short)(short)3623);
                Debug.Assert(pack.lat == (int)1522358954);
                Debug.Assert(pack.vz == (short)(short) -32487);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.vy = (short)(short)20401;
            p33.relative_alt = (int)804810153;
            p33.hdg = (ushort)(ushort)13292;
            p33.lon = (int)1430169975;
            p33.vz = (short)(short) -32487;
            p33.alt = (int) -302842896;
            p33.time_boot_ms = (uint)2909578858U;
            p33.vx = (short)(short)3623;
            p33.lat = (int)1522358954;
            SMP_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3640355210U);
                Debug.Assert(pack.rssi == (byte)(byte)209);
                Debug.Assert(pack.chan7_scaled == (short)(short) -25553);
                Debug.Assert(pack.chan2_scaled == (short)(short) -1162);
                Debug.Assert(pack.port == (byte)(byte)50);
                Debug.Assert(pack.chan6_scaled == (short)(short) -14553);
                Debug.Assert(pack.chan8_scaled == (short)(short)20706);
                Debug.Assert(pack.chan5_scaled == (short)(short) -24743);
                Debug.Assert(pack.chan4_scaled == (short)(short) -3361);
                Debug.Assert(pack.chan3_scaled == (short)(short) -567);
                Debug.Assert(pack.chan1_scaled == (short)(short) -24574);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan8_scaled = (short)(short)20706;
            p34.port = (byte)(byte)50;
            p34.chan1_scaled = (short)(short) -24574;
            p34.chan4_scaled = (short)(short) -3361;
            p34.chan6_scaled = (short)(short) -14553;
            p34.time_boot_ms = (uint)3640355210U;
            p34.chan7_scaled = (short)(short) -25553;
            p34.chan2_scaled = (short)(short) -1162;
            p34.chan3_scaled = (short)(short) -567;
            p34.chan5_scaled = (short)(short) -24743;
            p34.rssi = (byte)(byte)209;
            SMP_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)37183);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)24480);
                Debug.Assert(pack.rssi == (byte)(byte)114);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)20297);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)61961);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)18291);
                Debug.Assert(pack.port == (byte)(byte)117);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)47325);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)59839);
                Debug.Assert(pack.time_boot_ms == (uint)709910560U);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)51759);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.rssi = (byte)(byte)114;
            p35.chan3_raw = (ushort)(ushort)37183;
            p35.chan8_raw = (ushort)(ushort)47325;
            p35.port = (byte)(byte)117;
            p35.chan2_raw = (ushort)(ushort)51759;
            p35.chan7_raw = (ushort)(ushort)59839;
            p35.chan1_raw = (ushort)(ushort)18291;
            p35.time_boot_ms = (uint)709910560U;
            p35.chan5_raw = (ushort)(ushort)20297;
            p35.chan6_raw = (ushort)(ushort)61961;
            p35.chan4_raw = (ushort)(ushort)24480;
            SMP_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.port == (byte)(byte)85);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)24223);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)48052);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)47595);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)17311);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)29026);
                Debug.Assert(pack.time_usec == (uint)2132135321U);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)3935);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)5270);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)30396);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)51752);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)15853);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)40881);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)62084);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)12351);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)51631);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)9888);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)41357);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.time_usec = (uint)2132135321U;
            p36.servo16_raw_SET((ushort)(ushort)9888, PH) ;
            p36.servo5_raw = (ushort)(ushort)41357;
            p36.servo7_raw = (ushort)(ushort)29026;
            p36.servo6_raw = (ushort)(ushort)62084;
            p36.servo14_raw_SET((ushort)(ushort)5270, PH) ;
            p36.servo9_raw_SET((ushort)(ushort)51631, PH) ;
            p36.servo10_raw_SET((ushort)(ushort)40881, PH) ;
            p36.servo13_raw_SET((ushort)(ushort)51752, PH) ;
            p36.servo11_raw_SET((ushort)(ushort)30396, PH) ;
            p36.servo12_raw_SET((ushort)(ushort)17311, PH) ;
            p36.servo2_raw = (ushort)(ushort)24223;
            p36.servo8_raw = (ushort)(ushort)15853;
            p36.port = (byte)(byte)85;
            p36.servo4_raw = (ushort)(ushort)48052;
            p36.servo3_raw = (ushort)(ushort)47595;
            p36.servo1_raw = (ushort)(ushort)3935;
            p36.servo15_raw_SET((ushort)(ushort)12351, PH) ;
            SMP_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_system == (byte)(byte)96);
                Debug.Assert(pack.end_index == (short)(short)9300);
                Debug.Assert(pack.target_component == (byte)(byte)93);
                Debug.Assert(pack.start_index == (short)(short) -14994);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.end_index = (short)(short)9300;
            p37.start_index = (short)(short) -14994;
            p37.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p37.target_component = (byte)(byte)93;
            p37.target_system = (byte)(byte)96;
            SMP_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)207);
                Debug.Assert(pack.end_index == (short)(short)13319);
                Debug.Assert(pack.target_system == (byte)(byte)118);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.start_index == (short)(short) -11481);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.end_index = (short)(short)13319;
            p38.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p38.target_component = (byte)(byte)207;
            p38.target_system = (byte)(byte)118;
            p38.start_index = (short)(short) -11481;
            SMP_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)2.5502627E38F);
                Debug.Assert(pack.y == (float)5.076423E37F);
                Debug.Assert(pack.param3 == (float) -8.963002E37F);
                Debug.Assert(pack.x == (float) -1.5761508E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.seq == (ushort)(ushort)44219);
                Debug.Assert(pack.target_component == (byte)(byte)85);
                Debug.Assert(pack.target_system == (byte)(byte)69);
                Debug.Assert(pack.param1 == (float) -3.1388453E38F);
                Debug.Assert(pack.current == (byte)(byte)217);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.param2 == (float) -2.2325188E37F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_REPEAT_RELAY);
                Debug.Assert(pack.autocontinue == (byte)(byte)196);
                Debug.Assert(pack.param4 == (float)1.5354036E38F);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.z = (float)2.5502627E38F;
            p39.y = (float)5.076423E37F;
            p39.x = (float) -1.5761508E38F;
            p39.command = MAV_CMD.MAV_CMD_DO_REPEAT_RELAY;
            p39.seq = (ushort)(ushort)44219;
            p39.autocontinue = (byte)(byte)196;
            p39.param3 = (float) -8.963002E37F;
            p39.target_system = (byte)(byte)69;
            p39.param2 = (float) -2.2325188E37F;
            p39.param1 = (float) -3.1388453E38F;
            p39.param4 = (float)1.5354036E38F;
            p39.frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p39.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p39.target_component = (byte)(byte)85;
            p39.current = (byte)(byte)217;
            SMP_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)21499);
                Debug.Assert(pack.target_component == (byte)(byte)124);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_system == (byte)(byte)169);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p40.target_component = (byte)(byte)124;
            p40.seq = (ushort)(ushort)21499;
            p40.target_system = (byte)(byte)169;
            SMP_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)19700);
                Debug.Assert(pack.target_system == (byte)(byte)101);
                Debug.Assert(pack.target_component == (byte)(byte)5);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_component = (byte)(byte)5;
            p41.target_system = (byte)(byte)101;
            p41.seq = (ushort)(ushort)19700;
            SMP_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)20804);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)20804;
            SMP_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)63);
                Debug.Assert(pack.target_system == (byte)(byte)204);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p43.target_system = (byte)(byte)204;
            p43.target_component = (byte)(byte)63;
            SMP_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.count == (ushort)(ushort)27928);
                Debug.Assert(pack.target_component == (byte)(byte)84);
                Debug.Assert(pack.target_system == (byte)(byte)96);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p44.count = (ushort)(ushort)27928;
            p44.target_component = (byte)(byte)84;
            p44.target_system = (byte)(byte)96;
            SMP_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)92);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)141);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p45.target_system = (byte)(byte)92;
            p45.target_component = (byte)(byte)141;
            SMP_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)40214);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)40214;
            SMP_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE);
                Debug.Assert(pack.target_system == (byte)(byte)104);
                Debug.Assert(pack.target_component == (byte)(byte)80);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p47.target_component = (byte)(byte)80;
            p47.target_system = (byte)(byte)104;
            p47.type = MAV_MISSION_RESULT.MAV_MISSION_NO_SPACE;
            SMP_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int) -654509443);
                Debug.Assert(pack.target_system == (byte)(byte)78);
                Debug.Assert(pack.longitude == (int)1390767808);
                Debug.Assert(pack.altitude == (int)482627016);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)3659943870910085113L);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.altitude = (int)482627016;
            p48.target_system = (byte)(byte)78;
            p48.latitude = (int) -654509443;
            p48.longitude = (int)1390767808;
            p48.time_usec_SET((ulong)3659943870910085113L, PH) ;
            SMP_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)4416479197892841814L);
                Debug.Assert(pack.latitude == (int) -891677179);
                Debug.Assert(pack.altitude == (int)2025579665);
                Debug.Assert(pack.longitude == (int)1088974066);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.longitude = (int)1088974066;
            p49.altitude = (int)2025579665;
            p49.time_usec_SET((ulong)4416479197892841814L, PH) ;
            p49.latitude = (int) -891677179;
            SMP_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value0 == (float)5.620568E37F);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)35);
                Debug.Assert(pack.param_value_max == (float)1.3109248E38F);
                Debug.Assert(pack.target_system == (byte)(byte)131);
                Debug.Assert(pack.scale == (float)2.4091118E38F);
                Debug.Assert(pack.target_component == (byte)(byte)228);
                Debug.Assert(pack.param_id_LEN(ph) == 12);
                Debug.Assert(pack.param_id_TRY(ph).Equals("zfyozhXnxzah"));
                Debug.Assert(pack.param_value_min == (float) -2.820182E37F);
                Debug.Assert(pack.param_index == (short)(short) -29183);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.target_system = (byte)(byte)131;
            p50.param_value_max = (float)1.3109248E38F;
            p50.param_value0 = (float)5.620568E37F;
            p50.target_component = (byte)(byte)228;
            p50.scale = (float)2.4091118E38F;
            p50.param_index = (short)(short) -29183;
            p50.param_id_SET("zfyozhXnxzah", PH) ;
            p50.parameter_rc_channel_index = (byte)(byte)35;
            p50.param_value_min = (float) -2.820182E37F;
            SMP_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)61653);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_component == (byte)(byte)87);
                Debug.Assert(pack.target_system == (byte)(byte)19);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.seq = (ushort)(ushort)61653;
            p51.target_system = (byte)(byte)19;
            p51.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p51.target_component = (byte)(byte)87;
            SMP_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.p2z == (float)3.2994456E38F);
                Debug.Assert(pack.p2x == (float)3.2966603E38F);
                Debug.Assert(pack.p2y == (float) -2.5580143E38F);
                Debug.Assert(pack.p1z == (float)2.8993323E38F);
                Debug.Assert(pack.target_component == (byte)(byte)18);
                Debug.Assert(pack.p1x == (float) -2.732226E38F);
                Debug.Assert(pack.target_system == (byte)(byte)88);
                Debug.Assert(pack.p1y == (float)1.8463027E38F);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p2x = (float)3.2966603E38F;
            p54.p2z = (float)3.2994456E38F;
            p54.p2y = (float) -2.5580143E38F;
            p54.p1z = (float)2.8993323E38F;
            p54.p1y = (float)1.8463027E38F;
            p54.target_system = (byte)(byte)88;
            p54.p1x = (float) -2.732226E38F;
            p54.frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p54.target_component = (byte)(byte)18;
            SMP_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.p2y == (float)2.4931353E38F);
                Debug.Assert(pack.p2z == (float)1.9529174E38F);
                Debug.Assert(pack.p1z == (float) -2.9167354E38F);
                Debug.Assert(pack.p2x == (float)2.9536458E38F);
                Debug.Assert(pack.p1y == (float) -3.3644203E38F);
                Debug.Assert(pack.p1x == (float) -1.5781208E37F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p1z = (float) -2.9167354E38F;
            p55.p2x = (float)2.9536458E38F;
            p55.p2z = (float)1.9529174E38F;
            p55.frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p55.p2y = (float)2.4931353E38F;
            p55.p1y = (float) -3.3644203E38F;
            p55.p1x = (float) -1.5781208E37F;
            SMP_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-1.747896E38F, -1.8950275E38F, 2.878528E38F, 9.745132E37F, -2.942865E38F, -9.330549E37F, 9.751877E37F, 4.216965E37F, 1.7978387E38F}));
                Debug.Assert(pack.rollspeed == (float) -1.8376658E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.3461894E37F, 6.988006E37F, -1.1492251E38F, 9.302844E37F}));
                Debug.Assert(pack.pitchspeed == (float)2.2960878E38F);
                Debug.Assert(pack.time_usec == (ulong)9103695108397476153L);
                Debug.Assert(pack.yawspeed == (float)1.9538177E38F);
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.rollspeed = (float) -1.8376658E38F;
            p61.pitchspeed = (float)2.2960878E38F;
            p61.q_SET(new float[] {3.3461894E37F, 6.988006E37F, -1.1492251E38F, 9.302844E37F}, 0) ;
            p61.yawspeed = (float)1.9538177E38F;
            p61.time_usec = (ulong)9103695108397476153L;
            p61.covariance_SET(new float[] {-1.747896E38F, -1.8950275E38F, 2.878528E38F, 9.745132E37F, -2.942865E38F, -9.330549E37F, 9.751877E37F, 4.216965E37F, 1.7978387E38F}, 0) ;
            SMP_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nav_roll == (float) -3.163042E38F);
                Debug.Assert(pack.xtrack_error == (float) -2.455852E38F);
                Debug.Assert(pack.alt_error == (float) -3.0587675E38F);
                Debug.Assert(pack.nav_pitch == (float)1.952209E38F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)25797);
                Debug.Assert(pack.target_bearing == (short)(short)687);
                Debug.Assert(pack.aspd_error == (float) -8.2302905E35F);
                Debug.Assert(pack.nav_bearing == (short)(short)20395);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.aspd_error = (float) -8.2302905E35F;
            p62.xtrack_error = (float) -2.455852E38F;
            p62.nav_roll = (float) -3.163042E38F;
            p62.alt_error = (float) -3.0587675E38F;
            p62.nav_pitch = (float)1.952209E38F;
            p62.target_bearing = (short)(short)687;
            p62.nav_bearing = (short)(short)20395;
            p62.wp_dist = (ushort)(ushort)25797;
            SMP_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float) -2.3123732E38F);
                Debug.Assert(pack.lat == (int) -1472267391);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.7252345E38F, 1.245533E38F, 2.1795808E38F, 1.8986333E38F, 2.1324996E38F, -1.9816373E38F, 7.772047E37F, -3.85491E37F, -3.0486764E37F, -7.408751E37F, -3.2421868E38F, 1.6312267E38F, 2.7755161E38F, 1.407403E38F, 1.4286256E38F, -4.0188193E37F, 2.9291148E38F, 4.7764076E37F, 1.1215674E38F, -2.496318E38F, -2.7985773E37F, 2.3575717E38F, 3.059448E38F, -1.0646269E38F, -2.5489046E38F, -2.7166782E37F, -3.1572925E38F, 2.336655E38F, -2.4425501E38F, -5.284716E37F, -1.074564E38F, -3.0408153E38F, 1.6708637E38F, 8.065851E37F, 5.2425105E37F, -3.0957013E38F}));
                Debug.Assert(pack.lon == (int) -1476026741);
                Debug.Assert(pack.vy == (float) -8.211424E37F);
                Debug.Assert(pack.time_usec == (ulong)3622301134522885800L);
                Debug.Assert(pack.vx == (float)2.0554776E38F);
                Debug.Assert(pack.relative_alt == (int) -1526437185);
                Debug.Assert(pack.alt == (int) -697114675);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.vz = (float) -2.3123732E38F;
            p63.time_usec = (ulong)3622301134522885800L;
            p63.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS;
            p63.relative_alt = (int) -1526437185;
            p63.lon = (int) -1476026741;
            p63.covariance_SET(new float[] {1.7252345E38F, 1.245533E38F, 2.1795808E38F, 1.8986333E38F, 2.1324996E38F, -1.9816373E38F, 7.772047E37F, -3.85491E37F, -3.0486764E37F, -7.408751E37F, -3.2421868E38F, 1.6312267E38F, 2.7755161E38F, 1.407403E38F, 1.4286256E38F, -4.0188193E37F, 2.9291148E38F, 4.7764076E37F, 1.1215674E38F, -2.496318E38F, -2.7985773E37F, 2.3575717E38F, 3.059448E38F, -1.0646269E38F, -2.5489046E38F, -2.7166782E37F, -3.1572925E38F, 2.336655E38F, -2.4425501E38F, -5.284716E37F, -1.074564E38F, -3.0408153E38F, 1.6708637E38F, 8.065851E37F, 5.2425105E37F, -3.0957013E38F}, 0) ;
            p63.vy = (float) -8.211424E37F;
            p63.vx = (float)2.0554776E38F;
            p63.alt = (int) -697114675;
            p63.lat = (int) -1472267391;
            SMP_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)1.3221332E38F);
                Debug.Assert(pack.y == (float) -1.988579E38F);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
                Debug.Assert(pack.ax == (float) -6.9834233E37F);
                Debug.Assert(pack.vx == (float)1.7379311E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.4515744E38F, -1.9481167E38F, -1.4546451E38F, -1.5565792E38F, 1.3801903E38F, -3.2282294E38F, -1.6336278E38F, -2.1010548E38F, -1.5071801E38F, 1.5070083E38F, -5.241572E37F, -3.045975E38F, 2.9452412E38F, 5.5566303E37F, -7.0007627E37F, -3.2592694E38F, 3.1437968E38F, 2.9570597E38F, -1.0364237E38F, 9.68948E37F, 4.436125E37F, -3.1353754E38F, -7.63978E37F, 1.1114303E38F, 1.9024961E38F, 2.5269039E38F, 1.1779121E38F, -1.1124031E38F, -3.2317973E38F, -2.5275675E38F, 2.5228748E38F, 3.0755475E37F, 2.8132816E38F, 1.7848855E38F, 1.3135925E37F, 9.514902E37F, -1.1964171E38F, -2.3878035E38F, -3.2578235E38F, -2.4719317E38F, 8.295279E37F, -3.0035571E38F, 2.8044975E38F, -2.0088782E38F, -3.3164295E38F}));
                Debug.Assert(pack.x == (float)2.8258658E38F);
                Debug.Assert(pack.az == (float) -5.6309197E37F);
                Debug.Assert(pack.ay == (float) -1.5520977E38F);
                Debug.Assert(pack.vz == (float) -9.907098E37F);
                Debug.Assert(pack.vy == (float) -1.7211624E38F);
                Debug.Assert(pack.time_usec == (ulong)2103882386244005225L);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.ax = (float) -6.9834233E37F;
            p64.vx = (float)1.7379311E38F;
            p64.x = (float)2.8258658E38F;
            p64.vz = (float) -9.907098E37F;
            p64.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE;
            p64.az = (float) -5.6309197E37F;
            p64.vy = (float) -1.7211624E38F;
            p64.z = (float)1.3221332E38F;
            p64.time_usec = (ulong)2103882386244005225L;
            p64.covariance_SET(new float[] {-2.4515744E38F, -1.9481167E38F, -1.4546451E38F, -1.5565792E38F, 1.3801903E38F, -3.2282294E38F, -1.6336278E38F, -2.1010548E38F, -1.5071801E38F, 1.5070083E38F, -5.241572E37F, -3.045975E38F, 2.9452412E38F, 5.5566303E37F, -7.0007627E37F, -3.2592694E38F, 3.1437968E38F, 2.9570597E38F, -1.0364237E38F, 9.68948E37F, 4.436125E37F, -3.1353754E38F, -7.63978E37F, 1.1114303E38F, 1.9024961E38F, 2.5269039E38F, 1.1779121E38F, -1.1124031E38F, -3.2317973E38F, -2.5275675E38F, 2.5228748E38F, 3.0755475E37F, 2.8132816E38F, 1.7848855E38F, 1.3135925E37F, 9.514902E37F, -1.1964171E38F, -2.3878035E38F, -3.2578235E38F, -2.4719317E38F, 8.295279E37F, -3.0035571E38F, 2.8044975E38F, -2.0088782E38F, -3.3164295E38F}, 0) ;
            p64.y = (float) -1.988579E38F;
            p64.ay = (float) -1.5520977E38F;
            SMP_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)28401);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)14045);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)31244);
                Debug.Assert(pack.rssi == (byte)(byte)51);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)19708);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)63568);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)7628);
                Debug.Assert(pack.time_boot_ms == (uint)70845468U);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)54769);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)33929);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)10166);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)11191);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)21832);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)64701);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)47449);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)47823);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)15978);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)6140);
                Debug.Assert(pack.chancount == (byte)(byte)248);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)62707);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)52856);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan17_raw = (ushort)(ushort)54769;
            p65.chan1_raw = (ushort)(ushort)47449;
            p65.chancount = (byte)(byte)248;
            p65.chan12_raw = (ushort)(ushort)21832;
            p65.chan11_raw = (ushort)(ushort)28401;
            p65.chan14_raw = (ushort)(ushort)19708;
            p65.chan9_raw = (ushort)(ushort)63568;
            p65.chan3_raw = (ushort)(ushort)47823;
            p65.chan13_raw = (ushort)(ushort)64701;
            p65.rssi = (byte)(byte)51;
            p65.time_boot_ms = (uint)70845468U;
            p65.chan18_raw = (ushort)(ushort)11191;
            p65.chan16_raw = (ushort)(ushort)33929;
            p65.chan10_raw = (ushort)(ushort)62707;
            p65.chan2_raw = (ushort)(ushort)15978;
            p65.chan7_raw = (ushort)(ushort)52856;
            p65.chan6_raw = (ushort)(ushort)7628;
            p65.chan8_raw = (ushort)(ushort)31244;
            p65.chan5_raw = (ushort)(ushort)10166;
            p65.chan15_raw = (ushort)(ushort)14045;
            p65.chan4_raw = (ushort)(ushort)6140;
            SMP_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_stop == (byte)(byte)69);
                Debug.Assert(pack.target_component == (byte)(byte)85);
                Debug.Assert(pack.req_stream_id == (byte)(byte)211);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)9147);
                Debug.Assert(pack.target_system == (byte)(byte)40);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.req_message_rate = (ushort)(ushort)9147;
            p66.req_stream_id = (byte)(byte)211;
            p66.start_stop = (byte)(byte)69;
            p66.target_system = (byte)(byte)40;
            p66.target_component = (byte)(byte)85;
            SMP_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.stream_id == (byte)(byte)244);
                Debug.Assert(pack.message_rate == (ushort)(ushort)65232);
                Debug.Assert(pack.on_off == (byte)(byte)170);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.message_rate = (ushort)(ushort)65232;
            p67.on_off = (byte)(byte)170;
            p67.stream_id = (byte)(byte)244;
            SMP_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.r == (short)(short) -2554);
                Debug.Assert(pack.target == (byte)(byte)255);
                Debug.Assert(pack.z == (short)(short) -7361);
                Debug.Assert(pack.buttons == (ushort)(ushort)26617);
                Debug.Assert(pack.y == (short)(short)9472);
                Debug.Assert(pack.x == (short)(short) -21605);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.buttons = (ushort)(ushort)26617;
            p69.r = (short)(short) -2554;
            p69.y = (short)(short)9472;
            p69.x = (short)(short) -21605;
            p69.target = (byte)(byte)255;
            p69.z = (short)(short) -7361;
            SMP_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)45062);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)7233);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)60130);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)53809);
                Debug.Assert(pack.target_system == (byte)(byte)173);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)19558);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)4893);
                Debug.Assert(pack.target_component == (byte)(byte)200);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)27544);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)15324);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan4_raw = (ushort)(ushort)4893;
            p70.target_system = (byte)(byte)173;
            p70.chan5_raw = (ushort)(ushort)27544;
            p70.chan2_raw = (ushort)(ushort)19558;
            p70.chan8_raw = (ushort)(ushort)15324;
            p70.chan3_raw = (ushort)(ushort)53809;
            p70.chan7_raw = (ushort)(ushort)45062;
            p70.chan1_raw = (ushort)(ushort)60130;
            p70.target_component = (byte)(byte)200;
            p70.chan6_raw = (ushort)(ushort)7233;
            SMP_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)28);
                Debug.Assert(pack.autocontinue == (byte)(byte)9);
                Debug.Assert(pack.param3 == (float)9.979285E37F);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.current == (byte)(byte)174);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE);
                Debug.Assert(pack.x == (int) -2088127899);
                Debug.Assert(pack.z == (float) -3.0380433E38F);
                Debug.Assert(pack.target_component == (byte)(byte)28);
                Debug.Assert(pack.param4 == (float)2.4901653E38F);
                Debug.Assert(pack.seq == (ushort)(ushort)667);
                Debug.Assert(pack.param2 == (float)1.4285356E38F);
                Debug.Assert(pack.param1 == (float) -1.2293297E38F);
                Debug.Assert(pack.y == (int) -220065196);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.seq = (ushort)(ushort)667;
            p73.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p73.param2 = (float)1.4285356E38F;
            p73.z = (float) -3.0380433E38F;
            p73.x = (int) -2088127899;
            p73.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p73.target_component = (byte)(byte)28;
            p73.y = (int) -220065196;
            p73.param3 = (float)9.979285E37F;
            p73.command = MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
            p73.param1 = (float) -1.2293297E38F;
            p73.param4 = (float)2.4901653E38F;
            p73.autocontinue = (byte)(byte)9;
            p73.current = (byte)(byte)174;
            p73.target_system = (byte)(byte)28;
            SMP_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.groundspeed == (float) -3.1447073E38F);
                Debug.Assert(pack.alt == (float) -1.2045062E38F);
                Debug.Assert(pack.heading == (short)(short)24283);
                Debug.Assert(pack.climb == (float)1.1506147E38F);
                Debug.Assert(pack.throttle == (ushort)(ushort)35016);
                Debug.Assert(pack.airspeed == (float)8.077711E37F);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.airspeed = (float)8.077711E37F;
            p74.groundspeed = (float) -3.1447073E38F;
            p74.throttle = (ushort)(ushort)35016;
            p74.heading = (short)(short)24283;
            p74.alt = (float) -1.2045062E38F;
            p74.climb = (float)1.1506147E38F;
            SMP_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param4 == (float) -2.2187228E38F);
                Debug.Assert(pack.x == (int)1566828630);
                Debug.Assert(pack.target_system == (byte)(byte)192);
                Debug.Assert(pack.autocontinue == (byte)(byte)164);
                Debug.Assert(pack.target_component == (byte)(byte)228);
                Debug.Assert(pack.param2 == (float)8.405518E37F);
                Debug.Assert(pack.param1 == (float) -2.9900438E38F);
                Debug.Assert(pack.z == (float)3.0583886E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_SPATIAL_USER_4);
                Debug.Assert(pack.y == (int)1948843264);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.param3 == (float) -2.6710445E37F);
                Debug.Assert(pack.current == (byte)(byte)45);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.target_system = (byte)(byte)192;
            p75.param4 = (float) -2.2187228E38F;
            p75.z = (float)3.0583886E38F;
            p75.autocontinue = (byte)(byte)164;
            p75.y = (int)1948843264;
            p75.param3 = (float) -2.6710445E37F;
            p75.frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p75.command = MAV_CMD.MAV_CMD_SPATIAL_USER_4;
            p75.target_component = (byte)(byte)228;
            p75.current = (byte)(byte)45;
            p75.param2 = (float)8.405518E37F;
            p75.param1 = (float) -2.9900438E38F;
            p75.x = (int)1566828630;
            SMP_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.confirmation == (byte)(byte)21);
                Debug.Assert(pack.param3 == (float)2.2429108E38F);
                Debug.Assert(pack.param7 == (float) -2.3703042E38F);
                Debug.Assert(pack.target_component == (byte)(byte)25);
                Debug.Assert(pack.param5 == (float) -4.5369737E37F);
                Debug.Assert(pack.param1 == (float) -1.742581E38F);
                Debug.Assert(pack.param2 == (float)3.219837E38F);
                Debug.Assert(pack.target_system == (byte)(byte)189);
                Debug.Assert(pack.param6 == (float) -2.3390918E38F);
                Debug.Assert(pack.param4 == (float) -2.0336174E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION);
            };
            COMMAND_LONG p76 = new COMMAND_LONG();
            PH.setPack(p76);
            p76.param7 = (float) -2.3703042E38F;
            p76.param3 = (float)2.2429108E38F;
            p76.confirmation = (byte)(byte)21;
            p76.param4 = (float) -2.0336174E38F;
            p76.target_system = (byte)(byte)189;
            p76.command = MAV_CMD.MAV_CMD_REQUEST_FLIGHT_INFORMATION;
            p76.param5 = (float) -4.5369737E37F;
            p76.param1 = (float) -1.742581E38F;
            p76.param6 = (float) -2.3390918E38F;
            p76.param2 = (float)3.219837E38F;
            p76.target_component = (byte)(byte)25;
            SMP_TEST_CH.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)148);
                Debug.Assert(pack.result == MAV_RESULT.MAV_RESULT_UNSUPPORTED);
                Debug.Assert(pack.result_param2_TRY(ph) == (int) -1515597252);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)235);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)22);
            };
            COMMAND_ACK p77 = new COMMAND_ACK();
            PH.setPack(p77);
            p77.result = MAV_RESULT.MAV_RESULT_UNSUPPORTED;
            p77.command = MAV_CMD.MAV_CMD_PREFLIGHT_CALIBRATION;
            p77.result_param2_SET((int) -1515597252, PH) ;
            p77.target_system_SET((byte)(byte)148, PH) ;
            p77.target_component_SET((byte)(byte)22, PH) ;
            p77.progress_SET((byte)(byte)235, PH) ;
            SMP_TEST_CH.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)1.8052673E38F);
                Debug.Assert(pack.mode_switch == (byte)(byte)121);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)219);
                Debug.Assert(pack.roll == (float)5.501348E37F);
                Debug.Assert(pack.time_boot_ms == (uint)2909577484U);
                Debug.Assert(pack.thrust == (float)7.7953646E37F);
                Debug.Assert(pack.pitch == (float)1.3140956E36F);
            };
            MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.thrust = (float)7.7953646E37F;
            p81.mode_switch = (byte)(byte)121;
            p81.manual_override_switch = (byte)(byte)219;
            p81.yaw = (float)1.8052673E38F;
            p81.roll = (float)5.501348E37F;
            p81.time_boot_ms = (uint)2909577484U;
            p81.pitch = (float)1.3140956E36F;
            SMP_TEST_CH.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_yaw_rate == (float)1.8157028E38F);
                Debug.Assert(pack.body_roll_rate == (float) -7.051817E37F);
                Debug.Assert(pack.target_system == (byte)(byte)58);
                Debug.Assert(pack.thrust == (float)1.06482E38F);
                Debug.Assert(pack.body_pitch_rate == (float)1.0156135E38F);
                Debug.Assert(pack.target_component == (byte)(byte)27);
                Debug.Assert(pack.time_boot_ms == (uint)3567496107U);
                Debug.Assert(pack.type_mask == (byte)(byte)132);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.9190236E38F, -2.0191232E38F, 4.353252E37F, -1.6966074E38F}));
            };
            SET_ATTITUDE_TARGET p82 = new SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.type_mask = (byte)(byte)132;
            p82.body_yaw_rate = (float)1.8157028E38F;
            p82.target_system = (byte)(byte)58;
            p82.thrust = (float)1.06482E38F;
            p82.q_SET(new float[] {1.9190236E38F, -2.0191232E38F, 4.353252E37F, -1.6966074E38F}, 0) ;
            p82.target_component = (byte)(byte)27;
            p82.body_pitch_rate = (float)1.0156135E38F;
            p82.body_roll_rate = (float) -7.051817E37F;
            p82.time_boot_ms = (uint)3567496107U;
            SMP_TEST_CH.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_yaw_rate == (float)4.2556577E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1515903237U);
                Debug.Assert(pack.thrust == (float) -3.1218092E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.0100553E38F, 1.2204633E37F, 3.105684E38F, 2.8542689E38F}));
                Debug.Assert(pack.body_pitch_rate == (float) -2.641309E38F);
                Debug.Assert(pack.body_roll_rate == (float)2.9916696E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)188);
            };
            ATTITUDE_TARGET p83 = new ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)1515903237U;
            p83.body_pitch_rate = (float) -2.641309E38F;
            p83.body_yaw_rate = (float)4.2556577E37F;
            p83.body_roll_rate = (float)2.9916696E38F;
            p83.q_SET(new float[] {-1.0100553E38F, 1.2204633E37F, 3.105684E38F, 2.8542689E38F}, 0) ;
            p83.thrust = (float) -3.1218092E37F;
            p83.type_mask = (byte)(byte)188;
            SMP_TEST_CH.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afx == (float)1.9182916E38F);
                Debug.Assert(pack.vy == (float)1.3998907E38F);
                Debug.Assert(pack.target_component == (byte)(byte)0);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.afy == (float) -4.662023E37F);
                Debug.Assert(pack.afz == (float)2.9928758E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3975003938U);
                Debug.Assert(pack.z == (float)8.787418E37F);
                Debug.Assert(pack.vz == (float)2.5524987E38F);
                Debug.Assert(pack.yaw == (float) -1.7147027E38F);
                Debug.Assert(pack.y == (float)2.0467823E38F);
                Debug.Assert(pack.target_system == (byte)(byte)171);
                Debug.Assert(pack.vx == (float)2.2831645E38F);
                Debug.Assert(pack.x == (float)3.1617571E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)27883);
                Debug.Assert(pack.yaw_rate == (float)1.2346306E38F);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.z = (float)8.787418E37F;
            p84.vy = (float)1.3998907E38F;
            p84.afz = (float)2.9928758E38F;
            p84.vz = (float)2.5524987E38F;
            p84.yaw = (float) -1.7147027E38F;
            p84.type_mask = (ushort)(ushort)27883;
            p84.x = (float)3.1617571E37F;
            p84.time_boot_ms = (uint)3975003938U;
            p84.vx = (float)2.2831645E38F;
            p84.afy = (float) -4.662023E37F;
            p84.yaw_rate = (float)1.2346306E38F;
            p84.target_component = (byte)(byte)0;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p84.afx = (float)1.9182916E38F;
            p84.y = (float)2.0467823E38F;
            p84.target_system = (byte)(byte)171;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.vy == (float)5.574499E37F);
                Debug.Assert(pack.lat_int == (int)2044939282);
                Debug.Assert(pack.vz == (float) -1.3585711E38F);
                Debug.Assert(pack.vx == (float)3.2876236E38F);
                Debug.Assert(pack.yaw_rate == (float) -1.3797821E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3345353652U);
                Debug.Assert(pack.afy == (float)1.0840965E38F);
                Debug.Assert(pack.alt == (float)2.538299E38F);
                Debug.Assert(pack.afx == (float) -1.9145011E37F);
                Debug.Assert(pack.afz == (float)2.6634483E38F);
                Debug.Assert(pack.target_system == (byte)(byte)46);
                Debug.Assert(pack.target_component == (byte)(byte)89);
                Debug.Assert(pack.lon_int == (int)2088109969);
                Debug.Assert(pack.yaw == (float)2.711753E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)41376);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.type_mask = (ushort)(ushort)41376;
            p86.lon_int = (int)2088109969;
            p86.alt = (float)2.538299E38F;
            p86.vz = (float) -1.3585711E38F;
            p86.target_component = (byte)(byte)89;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p86.afx = (float) -1.9145011E37F;
            p86.time_boot_ms = (uint)3345353652U;
            p86.vx = (float)3.2876236E38F;
            p86.lat_int = (int)2044939282;
            p86.afy = (float)1.0840965E38F;
            p86.afz = (float)2.6634483E38F;
            p86.vy = (float)5.574499E37F;
            p86.yaw = (float)2.711753E38F;
            p86.target_system = (byte)(byte)46;
            p86.yaw_rate = (float) -1.3797821E38F;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat_int == (int) -1074634574);
                Debug.Assert(pack.vy == (float) -2.8384362E38F);
                Debug.Assert(pack.afy == (float) -3.2578618E38F);
                Debug.Assert(pack.yaw == (float)1.2042099E38F);
                Debug.Assert(pack.yaw_rate == (float)2.08301E38F);
                Debug.Assert(pack.alt == (float)3.136791E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3088507716U);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.afz == (float)1.8870273E38F);
                Debug.Assert(pack.lon_int == (int) -476151685);
                Debug.Assert(pack.vx == (float)1.1408892E38F);
                Debug.Assert(pack.afx == (float)1.5805297E37F);
                Debug.Assert(pack.vz == (float) -3.1717687E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)3841);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.lon_int = (int) -476151685;
            p87.alt = (float)3.136791E37F;
            p87.lat_int = (int) -1074634574;
            p87.yaw_rate = (float)2.08301E38F;
            p87.vx = (float)1.1408892E38F;
            p87.time_boot_ms = (uint)3088507716U;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p87.afy = (float) -3.2578618E38F;
            p87.vy = (float) -2.8384362E38F;
            p87.vz = (float) -3.1717687E38F;
            p87.afx = (float)1.5805297E37F;
            p87.afz = (float)1.8870273E38F;
            p87.yaw = (float)1.2042099E38F;
            p87.type_mask = (ushort)(ushort)3841;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)1.349965E38F);
                Debug.Assert(pack.z == (float) -2.5240609E38F);
                Debug.Assert(pack.roll == (float)1.5224307E38F);
                Debug.Assert(pack.y == (float)1.7040311E38F);
                Debug.Assert(pack.pitch == (float)1.5213888E38F);
                Debug.Assert(pack.x == (float) -1.9742885E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3372770713U);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.y = (float)1.7040311E38F;
            p89.pitch = (float)1.5213888E38F;
            p89.time_boot_ms = (uint)3372770713U;
            p89.yaw = (float)1.349965E38F;
            p89.z = (float) -2.5240609E38F;
            p89.x = (float) -1.9742885E38F;
            p89.roll = (float)1.5224307E38F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float) -1.0708445E38F);
                Debug.Assert(pack.vx == (short)(short)6652);
                Debug.Assert(pack.alt == (int) -352234513);
                Debug.Assert(pack.yawspeed == (float)1.1253957E38F);
                Debug.Assert(pack.rollspeed == (float) -1.8214064E38F);
                Debug.Assert(pack.yaw == (float)2.2410584E38F);
                Debug.Assert(pack.xacc == (short)(short)18041);
                Debug.Assert(pack.lon == (int)1744668439);
                Debug.Assert(pack.zacc == (short)(short)22753);
                Debug.Assert(pack.pitchspeed == (float) -1.7478022E38F);
                Debug.Assert(pack.yacc == (short)(short)16718);
                Debug.Assert(pack.lat == (int) -1200936325);
                Debug.Assert(pack.vy == (short)(short) -11500);
                Debug.Assert(pack.pitch == (float)2.2750635E38F);
                Debug.Assert(pack.vz == (short)(short)25458);
                Debug.Assert(pack.time_usec == (ulong)3081861509365925360L);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.pitch = (float)2.2750635E38F;
            p90.zacc = (short)(short)22753;
            p90.time_usec = (ulong)3081861509365925360L;
            p90.rollspeed = (float) -1.8214064E38F;
            p90.yawspeed = (float)1.1253957E38F;
            p90.pitchspeed = (float) -1.7478022E38F;
            p90.xacc = (short)(short)18041;
            p90.lat = (int) -1200936325;
            p90.vy = (short)(short) -11500;
            p90.yacc = (short)(short)16718;
            p90.vz = (short)(short)25458;
            p90.roll = (float) -1.0708445E38F;
            p90.alt = (int) -352234513;
            p90.lon = (int)1744668439;
            p90.vx = (short)(short)6652;
            p90.yaw = (float)2.2410584E38F;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aux3 == (float) -3.3311178E38F);
                Debug.Assert(pack.aux2 == (float) -1.6419241E38F);
                Debug.Assert(pack.yaw_rudder == (float) -3.2375204E38F);
                Debug.Assert(pack.aux1 == (float) -1.3675561E38F);
                Debug.Assert(pack.roll_ailerons == (float)3.2932676E38F);
                Debug.Assert(pack.pitch_elevator == (float) -2.8953535E38F);
                Debug.Assert(pack.nav_mode == (byte)(byte)205);
                Debug.Assert(pack.time_usec == (ulong)8252300411350639779L);
                Debug.Assert(pack.aux4 == (float)4.137805E37F);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
                Debug.Assert(pack.throttle == (float) -5.2107544E37F);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.nav_mode = (byte)(byte)205;
            p91.aux4 = (float)4.137805E37F;
            p91.aux2 = (float) -1.6419241E38F;
            p91.throttle = (float) -5.2107544E37F;
            p91.yaw_rudder = (float) -3.2375204E38F;
            p91.aux1 = (float) -1.3675561E38F;
            p91.mode = MAV_MODE.MAV_MODE_STABILIZE_DISARMED;
            p91.pitch_elevator = (float) -2.8953535E38F;
            p91.time_usec = (ulong)8252300411350639779L;
            p91.aux3 = (float) -3.3311178E38F;
            p91.roll_ailerons = (float)3.2932676E38F;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)23849);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)40264);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)4271);
                Debug.Assert(pack.rssi == (byte)(byte)180);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)58310);
                Debug.Assert(pack.time_usec == (ulong)7745645067510577271L);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)41876);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)20111);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)28772);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)36873);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)31090);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)11212);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)22644);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)1540);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan3_raw = (ushort)(ushort)4271;
            p92.chan12_raw = (ushort)(ushort)36873;
            p92.chan2_raw = (ushort)(ushort)20111;
            p92.chan5_raw = (ushort)(ushort)28772;
            p92.chan4_raw = (ushort)(ushort)41876;
            p92.chan9_raw = (ushort)(ushort)58310;
            p92.chan11_raw = (ushort)(ushort)23849;
            p92.time_usec = (ulong)7745645067510577271L;
            p92.chan8_raw = (ushort)(ushort)31090;
            p92.rssi = (byte)(byte)180;
            p92.chan10_raw = (ushort)(ushort)11212;
            p92.chan1_raw = (ushort)(ushort)1540;
            p92.chan6_raw = (ushort)(ushort)40264;
            p92.chan7_raw = (ushort)(ushort)22644;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (ulong)4177589796765916113L);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_AUTO_DISARMED);
                Debug.Assert(pack.time_usec == (ulong)4585038603659443908L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.7881453E38F, 2.6700421E38F, -1.8348005E38F, 2.7555526E38F, -4.718808E37F, -7.434597E37F, -7.675948E37F, -1.6654009E38F, 1.7029857E38F, 1.0771858E38F, -8.678014E37F, -1.6947621E38F, 2.024919E38F, 6.884058E37F, 9.833589E37F, -1.9993272E38F}));
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.mode = MAV_MODE.MAV_MODE_AUTO_DISARMED;
            p93.controls_SET(new float[] {-1.7881453E38F, 2.6700421E38F, -1.8348005E38F, 2.7555526E38F, -4.718808E37F, -7.434597E37F, -7.675948E37F, -1.6654009E38F, 1.7029857E38F, 1.0771858E38F, -8.678014E37F, -1.6947621E38F, 2.024919E38F, 6.884058E37F, 9.833589E37F, -1.9993272E38F}, 0) ;
            p93.flags = (ulong)4177589796765916113L;
            p93.time_usec = (ulong)4585038603659443908L;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_id == (byte)(byte)66);
                Debug.Assert(pack.flow_y == (short)(short) -4298);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float) -3.0858222E38F);
                Debug.Assert(pack.flow_comp_m_y == (float)1.7717403E38F);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float) -2.9212473E38F);
                Debug.Assert(pack.ground_distance == (float)2.3422824E37F);
                Debug.Assert(pack.flow_x == (short)(short)15833);
                Debug.Assert(pack.flow_comp_m_x == (float) -2.7787605E38F);
                Debug.Assert(pack.quality == (byte)(byte)251);
                Debug.Assert(pack.time_usec == (ulong)7088208466848549673L);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_rate_x_SET((float) -2.9212473E38F, PH) ;
            p100.sensor_id = (byte)(byte)66;
            p100.ground_distance = (float)2.3422824E37F;
            p100.time_usec = (ulong)7088208466848549673L;
            p100.flow_rate_y_SET((float) -3.0858222E38F, PH) ;
            p100.quality = (byte)(byte)251;
            p100.flow_comp_m_x = (float) -2.7787605E38F;
            p100.flow_y = (short)(short) -4298;
            p100.flow_comp_m_y = (float)1.7717403E38F;
            p100.flow_x = (short)(short)15833;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -6.3190603E37F);
                Debug.Assert(pack.usec == (ulong)8646445946132355844L);
                Debug.Assert(pack.z == (float)1.5671516E38F);
                Debug.Assert(pack.y == (float)2.505452E37F);
                Debug.Assert(pack.pitch == (float) -2.906369E38F);
                Debug.Assert(pack.yaw == (float)1.313241E38F);
                Debug.Assert(pack.roll == (float)3.0396014E38F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.z = (float)1.5671516E38F;
            p101.y = (float)2.505452E37F;
            p101.yaw = (float)1.313241E38F;
            p101.usec = (ulong)8646445946132355844L;
            p101.x = (float) -6.3190603E37F;
            p101.roll = (float)3.0396014E38F;
            p101.pitch = (float) -2.906369E38F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)1.1350674E38F);
                Debug.Assert(pack.y == (float)2.5245094E37F);
                Debug.Assert(pack.x == (float) -3.2451841E38F);
                Debug.Assert(pack.usec == (ulong)5952582685767614639L);
                Debug.Assert(pack.roll == (float)1.6963501E37F);
                Debug.Assert(pack.yaw == (float) -9.556294E37F);
                Debug.Assert(pack.pitch == (float)1.0327426E38F);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)5952582685767614639L;
            p102.yaw = (float) -9.556294E37F;
            p102.roll = (float)1.6963501E37F;
            p102.x = (float) -3.2451841E38F;
            p102.y = (float)2.5245094E37F;
            p102.z = (float)1.1350674E38F;
            p102.pitch = (float)1.0327426E38F;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)7.314856E37F);
                Debug.Assert(pack.usec == (ulong)413302398460000842L);
                Debug.Assert(pack.x == (float) -1.6645237E37F);
                Debug.Assert(pack.y == (float)1.2054436E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)413302398460000842L;
            p103.z = (float)7.314856E37F;
            p103.y = (float)1.2054436E38F;
            p103.x = (float) -1.6645237E37F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)9.826418E37F);
                Debug.Assert(pack.usec == (ulong)7722564701354666403L);
                Debug.Assert(pack.roll == (float) -2.5366736E37F);
                Debug.Assert(pack.x == (float) -1.3791642E38F);
                Debug.Assert(pack.yaw == (float)2.0207574E38F);
                Debug.Assert(pack.z == (float)1.8580006E38F);
                Debug.Assert(pack.pitch == (float)2.8892562E38F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.pitch = (float)2.8892562E38F;
            p104.y = (float)9.826418E37F;
            p104.z = (float)1.8580006E38F;
            p104.yaw = (float)2.0207574E38F;
            p104.usec = (ulong)7722564701354666403L;
            p104.x = (float) -1.3791642E38F;
            p104.roll = (float) -2.5366736E37F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zmag == (float) -8.0152265E37F);
                Debug.Assert(pack.pressure_alt == (float)8.682727E37F);
                Debug.Assert(pack.zacc == (float)1.6207655E38F);
                Debug.Assert(pack.abs_pressure == (float) -2.3607356E38F);
                Debug.Assert(pack.temperature == (float) -4.65258E37F);
                Debug.Assert(pack.xmag == (float) -2.7822182E38F);
                Debug.Assert(pack.diff_pressure == (float)2.3922212E37F);
                Debug.Assert(pack.xgyro == (float) -2.7818085E38F);
                Debug.Assert(pack.ygyro == (float) -2.7958926E38F);
                Debug.Assert(pack.yacc == (float)2.1008307E38F);
                Debug.Assert(pack.time_usec == (ulong)1608265099463877649L);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)36209);
                Debug.Assert(pack.zgyro == (float) -2.2045925E37F);
                Debug.Assert(pack.ymag == (float)1.1674089E38F);
                Debug.Assert(pack.xacc == (float)8.511381E37F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.ygyro = (float) -2.7958926E38F;
            p105.zgyro = (float) -2.2045925E37F;
            p105.xmag = (float) -2.7822182E38F;
            p105.zmag = (float) -8.0152265E37F;
            p105.diff_pressure = (float)2.3922212E37F;
            p105.yacc = (float)2.1008307E38F;
            p105.ymag = (float)1.1674089E38F;
            p105.xgyro = (float) -2.7818085E38F;
            p105.xacc = (float)8.511381E37F;
            p105.pressure_alt = (float)8.682727E37F;
            p105.zacc = (float)1.6207655E38F;
            p105.fields_updated = (ushort)(ushort)36209;
            p105.time_usec = (ulong)1608265099463877649L;
            p105.abs_pressure = (float) -2.3607356E38F;
            p105.temperature = (float) -4.65258E37F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_zgyro == (float)2.9600057E38F);
                Debug.Assert(pack.integrated_ygyro == (float) -2.1612043E38F);
                Debug.Assert(pack.integration_time_us == (uint)3312678221U);
                Debug.Assert(pack.integrated_y == (float)1.3636951E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)476875135U);
                Debug.Assert(pack.time_usec == (ulong)9088122627344188499L);
                Debug.Assert(pack.quality == (byte)(byte)56);
                Debug.Assert(pack.integrated_x == (float)2.4039107E38F);
                Debug.Assert(pack.distance == (float)1.103421E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)94);
                Debug.Assert(pack.temperature == (short)(short)28362);
                Debug.Assert(pack.integrated_xgyro == (float)1.3079436E38F);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.integrated_y = (float)1.3636951E38F;
            p106.integrated_zgyro = (float)2.9600057E38F;
            p106.integrated_x = (float)2.4039107E38F;
            p106.integrated_ygyro = (float) -2.1612043E38F;
            p106.integration_time_us = (uint)3312678221U;
            p106.distance = (float)1.103421E38F;
            p106.quality = (byte)(byte)56;
            p106.time_delta_distance_us = (uint)476875135U;
            p106.sensor_id = (byte)(byte)94;
            p106.time_usec = (ulong)9088122627344188499L;
            p106.temperature = (short)(short)28362;
            p106.integrated_xgyro = (float)1.3079436E38F;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fields_updated == (uint)2119433475U);
                Debug.Assert(pack.ymag == (float)1.3763882E38F);
                Debug.Assert(pack.zgyro == (float) -7.553417E37F);
                Debug.Assert(pack.yacc == (float)3.349009E38F);
                Debug.Assert(pack.diff_pressure == (float)1.7836081E38F);
                Debug.Assert(pack.pressure_alt == (float)3.3252428E38F);
                Debug.Assert(pack.xacc == (float)3.892409E37F);
                Debug.Assert(pack.xmag == (float) -2.283941E38F);
                Debug.Assert(pack.ygyro == (float)2.3918472E38F);
                Debug.Assert(pack.zmag == (float)1.9534555E38F);
                Debug.Assert(pack.zacc == (float) -1.8161143E38F);
                Debug.Assert(pack.temperature == (float) -2.7926189E38F);
                Debug.Assert(pack.abs_pressure == (float) -1.7161724E38F);
                Debug.Assert(pack.time_usec == (ulong)4853773558623497743L);
                Debug.Assert(pack.xgyro == (float) -1.6995128E38F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.fields_updated = (uint)2119433475U;
            p107.zgyro = (float) -7.553417E37F;
            p107.xmag = (float) -2.283941E38F;
            p107.zmag = (float)1.9534555E38F;
            p107.yacc = (float)3.349009E38F;
            p107.ygyro = (float)2.3918472E38F;
            p107.zacc = (float) -1.8161143E38F;
            p107.ymag = (float)1.3763882E38F;
            p107.temperature = (float) -2.7926189E38F;
            p107.diff_pressure = (float)1.7836081E38F;
            p107.pressure_alt = (float)3.3252428E38F;
            p107.xacc = (float)3.892409E37F;
            p107.abs_pressure = (float) -1.7161724E38F;
            p107.xgyro = (float) -1.6995128E38F;
            p107.time_usec = (ulong)4853773558623497743L;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.std_dev_vert == (float)2.1099434E38F);
                Debug.Assert(pack.ve == (float)1.4291789E38F);
                Debug.Assert(pack.std_dev_horz == (float)2.409726E38F);
                Debug.Assert(pack.lon == (float) -2.4566121E38F);
                Debug.Assert(pack.yaw == (float) -3.3921799E38F);
                Debug.Assert(pack.q4 == (float)1.2140127E38F);
                Debug.Assert(pack.q1 == (float)1.7959366E38F);
                Debug.Assert(pack.ygyro == (float)3.3175755E38F);
                Debug.Assert(pack.zgyro == (float)2.4254578E37F);
                Debug.Assert(pack.lat == (float) -3.3527766E38F);
                Debug.Assert(pack.xgyro == (float)1.4671795E38F);
                Debug.Assert(pack.alt == (float) -3.058411E38F);
                Debug.Assert(pack.q3 == (float) -1.4817899E37F);
                Debug.Assert(pack.yacc == (float) -1.6722186E38F);
                Debug.Assert(pack.roll == (float)2.3276494E37F);
                Debug.Assert(pack.zacc == (float)8.613168E36F);
                Debug.Assert(pack.q2 == (float) -2.5574575E38F);
                Debug.Assert(pack.vd == (float) -3.8410158E37F);
                Debug.Assert(pack.pitch == (float)5.5606087E37F);
                Debug.Assert(pack.xacc == (float)2.2144124E37F);
                Debug.Assert(pack.vn == (float) -6.2826346E37F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.pitch = (float)5.5606087E37F;
            p108.xacc = (float)2.2144124E37F;
            p108.roll = (float)2.3276494E37F;
            p108.lat = (float) -3.3527766E38F;
            p108.std_dev_horz = (float)2.409726E38F;
            p108.ve = (float)1.4291789E38F;
            p108.std_dev_vert = (float)2.1099434E38F;
            p108.q1 = (float)1.7959366E38F;
            p108.yaw = (float) -3.3921799E38F;
            p108.q2 = (float) -2.5574575E38F;
            p108.vd = (float) -3.8410158E37F;
            p108.alt = (float) -3.058411E38F;
            p108.ygyro = (float)3.3175755E38F;
            p108.yacc = (float) -1.6722186E38F;
            p108.lon = (float) -2.4566121E38F;
            p108.zacc = (float)8.613168E36F;
            p108.q4 = (float)1.2140127E38F;
            p108.xgyro = (float)1.4671795E38F;
            p108.zgyro = (float)2.4254578E37F;
            p108.q3 = (float) -1.4817899E37F;
            p108.vn = (float) -6.2826346E37F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.txbuf == (byte)(byte)69);
                Debug.Assert(pack.rssi == (byte)(byte)23);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)28847);
                Debug.Assert(pack.remnoise == (byte)(byte)35);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)45327);
                Debug.Assert(pack.remrssi == (byte)(byte)148);
                Debug.Assert(pack.noise == (byte)(byte)94);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rxerrors = (ushort)(ushort)45327;
            p109.rssi = (byte)(byte)23;
            p109.remnoise = (byte)(byte)35;
            p109.fixed_ = (ushort)(ushort)28847;
            p109.noise = (byte)(byte)94;
            p109.remrssi = (byte)(byte)148;
            p109.txbuf = (byte)(byte)69;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_network == (byte)(byte)95);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)136, (byte)21, (byte)76, (byte)220, (byte)16, (byte)87, (byte)67, (byte)91, (byte)175, (byte)169, (byte)134, (byte)1, (byte)212, (byte)89, (byte)88, (byte)233, (byte)212, (byte)146, (byte)176, (byte)59, (byte)13, (byte)100, (byte)114, (byte)244, (byte)120, (byte)69, (byte)100, (byte)246, (byte)161, (byte)163, (byte)119, (byte)186, (byte)212, (byte)160, (byte)90, (byte)203, (byte)20, (byte)124, (byte)189, (byte)44, (byte)31, (byte)138, (byte)37, (byte)227, (byte)165, (byte)162, (byte)163, (byte)92, (byte)221, (byte)34, (byte)112, (byte)49, (byte)182, (byte)208, (byte)72, (byte)242, (byte)12, (byte)102, (byte)3, (byte)234, (byte)190, (byte)101, (byte)8, (byte)234, (byte)235, (byte)33, (byte)186, (byte)43, (byte)136, (byte)120, (byte)179, (byte)138, (byte)171, (byte)3, (byte)152, (byte)204, (byte)0, (byte)69, (byte)171, (byte)13, (byte)89, (byte)200, (byte)40, (byte)84, (byte)121, (byte)239, (byte)209, (byte)100, (byte)179, (byte)169, (byte)41, (byte)101, (byte)73, (byte)126, (byte)209, (byte)164, (byte)68, (byte)224, (byte)181, (byte)234, (byte)160, (byte)108, (byte)39, (byte)29, (byte)171, (byte)227, (byte)45, (byte)73, (byte)178, (byte)21, (byte)91, (byte)212, (byte)247, (byte)28, (byte)65, (byte)154, (byte)70, (byte)44, (byte)37, (byte)225, (byte)82, (byte)24, (byte)125, (byte)153, (byte)69, (byte)70, (byte)35, (byte)95, (byte)88, (byte)84, (byte)60, (byte)0, (byte)79, (byte)136, (byte)245, (byte)225, (byte)173, (byte)186, (byte)1, (byte)182, (byte)8, (byte)152, (byte)16, (byte)18, (byte)38, (byte)53, (byte)200, (byte)127, (byte)16, (byte)211, (byte)50, (byte)20, (byte)109, (byte)102, (byte)84, (byte)54, (byte)161, (byte)54, (byte)157, (byte)156, (byte)127, (byte)58, (byte)236, (byte)215, (byte)139, (byte)228, (byte)139, (byte)222, (byte)215, (byte)38, (byte)253, (byte)123, (byte)221, (byte)42, (byte)170, (byte)141, (byte)171, (byte)84, (byte)248, (byte)53, (byte)33, (byte)2, (byte)30, (byte)211, (byte)250, (byte)56, (byte)225, (byte)16, (byte)138, (byte)110, (byte)87, (byte)37, (byte)125, (byte)206, (byte)110, (byte)249, (byte)124, (byte)29, (byte)231, (byte)34, (byte)119, (byte)71, (byte)124, (byte)196, (byte)201, (byte)181, (byte)123, (byte)254, (byte)47, (byte)230, (byte)102, (byte)246, (byte)172, (byte)255, (byte)68, (byte)138, (byte)114, (byte)29, (byte)126, (byte)179, (byte)89, (byte)107, (byte)158, (byte)124, (byte)184, (byte)116, (byte)78, (byte)60, (byte)108, (byte)76, (byte)122, (byte)72, (byte)64, (byte)216, (byte)76, (byte)150, (byte)44, (byte)115, (byte)94, (byte)20, (byte)95, (byte)216, (byte)204, (byte)118, (byte)64, (byte)98, (byte)102, (byte)26, (byte)63, (byte)224, (byte)167}));
                Debug.Assert(pack.target_system == (byte)(byte)20);
                Debug.Assert(pack.target_component == (byte)(byte)191);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_system = (byte)(byte)20;
            p110.target_network = (byte)(byte)95;
            p110.payload_SET(new byte[] {(byte)136, (byte)21, (byte)76, (byte)220, (byte)16, (byte)87, (byte)67, (byte)91, (byte)175, (byte)169, (byte)134, (byte)1, (byte)212, (byte)89, (byte)88, (byte)233, (byte)212, (byte)146, (byte)176, (byte)59, (byte)13, (byte)100, (byte)114, (byte)244, (byte)120, (byte)69, (byte)100, (byte)246, (byte)161, (byte)163, (byte)119, (byte)186, (byte)212, (byte)160, (byte)90, (byte)203, (byte)20, (byte)124, (byte)189, (byte)44, (byte)31, (byte)138, (byte)37, (byte)227, (byte)165, (byte)162, (byte)163, (byte)92, (byte)221, (byte)34, (byte)112, (byte)49, (byte)182, (byte)208, (byte)72, (byte)242, (byte)12, (byte)102, (byte)3, (byte)234, (byte)190, (byte)101, (byte)8, (byte)234, (byte)235, (byte)33, (byte)186, (byte)43, (byte)136, (byte)120, (byte)179, (byte)138, (byte)171, (byte)3, (byte)152, (byte)204, (byte)0, (byte)69, (byte)171, (byte)13, (byte)89, (byte)200, (byte)40, (byte)84, (byte)121, (byte)239, (byte)209, (byte)100, (byte)179, (byte)169, (byte)41, (byte)101, (byte)73, (byte)126, (byte)209, (byte)164, (byte)68, (byte)224, (byte)181, (byte)234, (byte)160, (byte)108, (byte)39, (byte)29, (byte)171, (byte)227, (byte)45, (byte)73, (byte)178, (byte)21, (byte)91, (byte)212, (byte)247, (byte)28, (byte)65, (byte)154, (byte)70, (byte)44, (byte)37, (byte)225, (byte)82, (byte)24, (byte)125, (byte)153, (byte)69, (byte)70, (byte)35, (byte)95, (byte)88, (byte)84, (byte)60, (byte)0, (byte)79, (byte)136, (byte)245, (byte)225, (byte)173, (byte)186, (byte)1, (byte)182, (byte)8, (byte)152, (byte)16, (byte)18, (byte)38, (byte)53, (byte)200, (byte)127, (byte)16, (byte)211, (byte)50, (byte)20, (byte)109, (byte)102, (byte)84, (byte)54, (byte)161, (byte)54, (byte)157, (byte)156, (byte)127, (byte)58, (byte)236, (byte)215, (byte)139, (byte)228, (byte)139, (byte)222, (byte)215, (byte)38, (byte)253, (byte)123, (byte)221, (byte)42, (byte)170, (byte)141, (byte)171, (byte)84, (byte)248, (byte)53, (byte)33, (byte)2, (byte)30, (byte)211, (byte)250, (byte)56, (byte)225, (byte)16, (byte)138, (byte)110, (byte)87, (byte)37, (byte)125, (byte)206, (byte)110, (byte)249, (byte)124, (byte)29, (byte)231, (byte)34, (byte)119, (byte)71, (byte)124, (byte)196, (byte)201, (byte)181, (byte)123, (byte)254, (byte)47, (byte)230, (byte)102, (byte)246, (byte)172, (byte)255, (byte)68, (byte)138, (byte)114, (byte)29, (byte)126, (byte)179, (byte)89, (byte)107, (byte)158, (byte)124, (byte)184, (byte)116, (byte)78, (byte)60, (byte)108, (byte)76, (byte)122, (byte)72, (byte)64, (byte)216, (byte)76, (byte)150, (byte)44, (byte)115, (byte)94, (byte)20, (byte)95, (byte)216, (byte)204, (byte)118, (byte)64, (byte)98, (byte)102, (byte)26, (byte)63, (byte)224, (byte)167}, 0) ;
            p110.target_component = (byte)(byte)191;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ts1 == (long) -6261700039548671123L);
                Debug.Assert(pack.tc1 == (long)7998432200332786388L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long) -6261700039548671123L;
            p111.tc1 = (long)7998432200332786388L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)2441243085U);
                Debug.Assert(pack.time_usec == (ulong)4361233041600779280L);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.seq = (uint)2441243085U;
            p112.time_usec = (ulong)4361233041600779280L;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1480860191);
                Debug.Assert(pack.time_usec == (ulong)6672464153900686369L);
                Debug.Assert(pack.vn == (short)(short)6735);
                Debug.Assert(pack.ve == (short)(short) -7783);
                Debug.Assert(pack.epv == (ushort)(ushort)512);
                Debug.Assert(pack.eph == (ushort)(ushort)43000);
                Debug.Assert(pack.vel == (ushort)(ushort)6873);
                Debug.Assert(pack.fix_type == (byte)(byte)32);
                Debug.Assert(pack.alt == (int)213488787);
                Debug.Assert(pack.cog == (ushort)(ushort)35688);
                Debug.Assert(pack.vd == (short)(short)7680);
                Debug.Assert(pack.lon == (int) -1156199143);
                Debug.Assert(pack.satellites_visible == (byte)(byte)142);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.eph = (ushort)(ushort)43000;
            p113.ve = (short)(short) -7783;
            p113.epv = (ushort)(ushort)512;
            p113.fix_type = (byte)(byte)32;
            p113.satellites_visible = (byte)(byte)142;
            p113.vel = (ushort)(ushort)6873;
            p113.time_usec = (ulong)6672464153900686369L;
            p113.lon = (int) -1156199143;
            p113.cog = (ushort)(ushort)35688;
            p113.vd = (short)(short)7680;
            p113.alt = (int)213488787;
            p113.lat = (int)1480860191;
            p113.vn = (short)(short)6735;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_y == (float)3.136215E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)4066139486U);
                Debug.Assert(pack.sensor_id == (byte)(byte)194);
                Debug.Assert(pack.integration_time_us == (uint)3793667411U);
                Debug.Assert(pack.distance == (float)1.5435911E38F);
                Debug.Assert(pack.integrated_x == (float) -1.6259172E38F);
                Debug.Assert(pack.time_usec == (ulong)934488699625150225L);
                Debug.Assert(pack.temperature == (short)(short) -3144);
                Debug.Assert(pack.integrated_xgyro == (float) -1.8497667E38F);
                Debug.Assert(pack.quality == (byte)(byte)84);
                Debug.Assert(pack.integrated_zgyro == (float) -4.284598E37F);
                Debug.Assert(pack.integrated_ygyro == (float)1.5254783E38F);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.time_usec = (ulong)934488699625150225L;
            p114.sensor_id = (byte)(byte)194;
            p114.integrated_x = (float) -1.6259172E38F;
            p114.integration_time_us = (uint)3793667411U;
            p114.time_delta_distance_us = (uint)4066139486U;
            p114.temperature = (short)(short) -3144;
            p114.integrated_xgyro = (float) -1.8497667E38F;
            p114.quality = (byte)(byte)84;
            p114.integrated_y = (float)3.136215E38F;
            p114.integrated_zgyro = (float) -4.284598E37F;
            p114.distance = (float)1.5435911E38F;
            p114.integrated_ygyro = (float)1.5254783E38F;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (short)(short)25759);
                Debug.Assert(pack.vz == (short)(short) -9600);
                Debug.Assert(pack.time_usec == (ulong)524900614082189723L);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {-3.1629412E38F, -3.2609959E38F, 3.2417152E38F, 1.4765085E38F}));
                Debug.Assert(pack.pitchspeed == (float) -3.1781424E38F);
                Debug.Assert(pack.alt == (int) -1452103274);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)7259);
                Debug.Assert(pack.zacc == (short)(short) -13845);
                Debug.Assert(pack.vy == (short)(short)6869);
                Debug.Assert(pack.lat == (int) -1142463276);
                Debug.Assert(pack.yacc == (short)(short)11726);
                Debug.Assert(pack.xacc == (short)(short)22002);
                Debug.Assert(pack.lon == (int) -921305365);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)23186);
                Debug.Assert(pack.rollspeed == (float) -2.4208415E38F);
                Debug.Assert(pack.yawspeed == (float)4.447996E37F);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)524900614082189723L;
            p115.ind_airspeed = (ushort)(ushort)23186;
            p115.true_airspeed = (ushort)(ushort)7259;
            p115.rollspeed = (float) -2.4208415E38F;
            p115.vy = (short)(short)6869;
            p115.attitude_quaternion_SET(new float[] {-3.1629412E38F, -3.2609959E38F, 3.2417152E38F, 1.4765085E38F}, 0) ;
            p115.vz = (short)(short) -9600;
            p115.vx = (short)(short)25759;
            p115.yawspeed = (float)4.447996E37F;
            p115.pitchspeed = (float) -3.1781424E38F;
            p115.xacc = (short)(short)22002;
            p115.yacc = (short)(short)11726;
            p115.zacc = (short)(short) -13845;
            p115.lon = (int) -921305365;
            p115.alt = (int) -1452103274;
            p115.lat = (int) -1142463276;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (short)(short)454);
                Debug.Assert(pack.ymag == (short)(short) -16048);
                Debug.Assert(pack.xgyro == (short)(short)26268);
                Debug.Assert(pack.xacc == (short)(short) -4666);
                Debug.Assert(pack.zacc == (short)(short)16540);
                Debug.Assert(pack.time_boot_ms == (uint)1753862901U);
                Debug.Assert(pack.ygyro == (short)(short)26558);
                Debug.Assert(pack.xmag == (short)(short) -13340);
                Debug.Assert(pack.zmag == (short)(short) -18807);
                Debug.Assert(pack.yacc == (short)(short) -21362);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.zmag = (short)(short) -18807;
            p116.zacc = (short)(short)16540;
            p116.yacc = (short)(short) -21362;
            p116.xgyro = (short)(short)26268;
            p116.xacc = (short)(short) -4666;
            p116.ymag = (short)(short) -16048;
            p116.time_boot_ms = (uint)1753862901U;
            p116.zgyro = (short)(short)454;
            p116.ygyro = (short)(short)26558;
            p116.xmag = (short)(short) -13340;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start == (ushort)(ushort)24907);
                Debug.Assert(pack.end == (ushort)(ushort)32324);
                Debug.Assert(pack.target_component == (byte)(byte)125);
                Debug.Assert(pack.target_system == (byte)(byte)198);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.target_component = (byte)(byte)125;
            p117.start = (ushort)(ushort)24907;
            p117.end = (ushort)(ushort)32324;
            p117.target_system = (byte)(byte)198;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.num_logs == (ushort)(ushort)42578);
                Debug.Assert(pack.size == (uint)2877124138U);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)35637);
                Debug.Assert(pack.id == (ushort)(ushort)3783);
                Debug.Assert(pack.time_utc == (uint)1282772429U);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.time_utc = (uint)1282772429U;
            p118.num_logs = (ushort)(ushort)42578;
            p118.last_log_num = (ushort)(ushort)35637;
            p118.id = (ushort)(ushort)3783;
            p118.size = (uint)2877124138U;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)50162);
                Debug.Assert(pack.target_component == (byte)(byte)180);
                Debug.Assert(pack.ofs == (uint)451782043U);
                Debug.Assert(pack.count == (uint)398504486U);
                Debug.Assert(pack.target_system == (byte)(byte)77);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.id = (ushort)(ushort)50162;
            p119.ofs = (uint)451782043U;
            p119.target_component = (byte)(byte)180;
            p119.count = (uint)398504486U;
            p119.target_system = (byte)(byte)77;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)80958256U);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)207, (byte)171, (byte)178, (byte)31, (byte)15, (byte)141, (byte)157, (byte)246, (byte)32, (byte)5, (byte)38, (byte)211, (byte)183, (byte)53, (byte)202, (byte)33, (byte)62, (byte)49, (byte)3, (byte)17, (byte)101, (byte)50, (byte)233, (byte)188, (byte)152, (byte)135, (byte)244, (byte)155, (byte)175, (byte)214, (byte)0, (byte)119, (byte)62, (byte)82, (byte)32, (byte)83, (byte)84, (byte)232, (byte)191, (byte)172, (byte)50, (byte)149, (byte)184, (byte)203, (byte)226, (byte)229, (byte)51, (byte)135, (byte)247, (byte)231, (byte)175, (byte)182, (byte)120, (byte)184, (byte)173, (byte)199, (byte)15, (byte)77, (byte)112, (byte)73, (byte)255, (byte)108, (byte)176, (byte)196, (byte)196, (byte)114, (byte)121, (byte)120, (byte)175, (byte)250, (byte)172, (byte)216, (byte)246, (byte)46, (byte)139, (byte)80, (byte)191, (byte)108, (byte)194, (byte)250, (byte)19, (byte)172, (byte)97, (byte)29, (byte)232, (byte)212, (byte)233, (byte)24, (byte)135, (byte)12}));
                Debug.Assert(pack.id == (ushort)(ushort)19618);
                Debug.Assert(pack.count == (byte)(byte)70);
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.count = (byte)(byte)70;
            p120.data__SET(new byte[] {(byte)207, (byte)171, (byte)178, (byte)31, (byte)15, (byte)141, (byte)157, (byte)246, (byte)32, (byte)5, (byte)38, (byte)211, (byte)183, (byte)53, (byte)202, (byte)33, (byte)62, (byte)49, (byte)3, (byte)17, (byte)101, (byte)50, (byte)233, (byte)188, (byte)152, (byte)135, (byte)244, (byte)155, (byte)175, (byte)214, (byte)0, (byte)119, (byte)62, (byte)82, (byte)32, (byte)83, (byte)84, (byte)232, (byte)191, (byte)172, (byte)50, (byte)149, (byte)184, (byte)203, (byte)226, (byte)229, (byte)51, (byte)135, (byte)247, (byte)231, (byte)175, (byte)182, (byte)120, (byte)184, (byte)173, (byte)199, (byte)15, (byte)77, (byte)112, (byte)73, (byte)255, (byte)108, (byte)176, (byte)196, (byte)196, (byte)114, (byte)121, (byte)120, (byte)175, (byte)250, (byte)172, (byte)216, (byte)246, (byte)46, (byte)139, (byte)80, (byte)191, (byte)108, (byte)194, (byte)250, (byte)19, (byte)172, (byte)97, (byte)29, (byte)232, (byte)212, (byte)233, (byte)24, (byte)135, (byte)12}, 0) ;
            p120.id = (ushort)(ushort)19618;
            p120.ofs = (uint)80958256U;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)194);
                Debug.Assert(pack.target_component == (byte)(byte)242);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_component = (byte)(byte)242;
            p121.target_system = (byte)(byte)194;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)100);
                Debug.Assert(pack.target_component == (byte)(byte)193);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_component = (byte)(byte)193;
            p122.target_system = (byte)(byte)100;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)24);
                Debug.Assert(pack.target_system == (byte)(byte)122);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)9, (byte)83, (byte)114, (byte)237, (byte)200, (byte)139, (byte)88, (byte)61, (byte)243, (byte)21, (byte)162, (byte)156, (byte)145, (byte)243, (byte)32, (byte)177, (byte)193, (byte)108, (byte)151, (byte)251, (byte)39, (byte)110, (byte)8, (byte)175, (byte)73, (byte)116, (byte)66, (byte)166, (byte)233, (byte)167, (byte)109, (byte)139, (byte)56, (byte)184, (byte)245, (byte)238, (byte)77, (byte)104, (byte)60, (byte)13, (byte)118, (byte)252, (byte)29, (byte)225, (byte)208, (byte)97, (byte)142, (byte)180, (byte)9, (byte)49, (byte)190, (byte)222, (byte)71, (byte)132, (byte)143, (byte)85, (byte)127, (byte)209, (byte)163, (byte)199, (byte)72, (byte)210, (byte)143, (byte)78, (byte)238, (byte)33, (byte)237, (byte)4, (byte)140, (byte)170, (byte)216, (byte)230, (byte)216, (byte)14, (byte)215, (byte)10, (byte)237, (byte)2, (byte)72, (byte)166, (byte)228, (byte)227, (byte)10, (byte)122, (byte)79, (byte)212, (byte)83, (byte)65, (byte)166, (byte)22, (byte)109, (byte)166, (byte)86, (byte)223, (byte)104, (byte)36, (byte)23, (byte)63, (byte)137, (byte)152, (byte)66, (byte)85, (byte)147, (byte)16, (byte)244, (byte)43, (byte)11, (byte)97, (byte)225, (byte)157}));
                Debug.Assert(pack.len == (byte)(byte)247);
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.len = (byte)(byte)247;
            p123.target_system = (byte)(byte)122;
            p123.data__SET(new byte[] {(byte)9, (byte)83, (byte)114, (byte)237, (byte)200, (byte)139, (byte)88, (byte)61, (byte)243, (byte)21, (byte)162, (byte)156, (byte)145, (byte)243, (byte)32, (byte)177, (byte)193, (byte)108, (byte)151, (byte)251, (byte)39, (byte)110, (byte)8, (byte)175, (byte)73, (byte)116, (byte)66, (byte)166, (byte)233, (byte)167, (byte)109, (byte)139, (byte)56, (byte)184, (byte)245, (byte)238, (byte)77, (byte)104, (byte)60, (byte)13, (byte)118, (byte)252, (byte)29, (byte)225, (byte)208, (byte)97, (byte)142, (byte)180, (byte)9, (byte)49, (byte)190, (byte)222, (byte)71, (byte)132, (byte)143, (byte)85, (byte)127, (byte)209, (byte)163, (byte)199, (byte)72, (byte)210, (byte)143, (byte)78, (byte)238, (byte)33, (byte)237, (byte)4, (byte)140, (byte)170, (byte)216, (byte)230, (byte)216, (byte)14, (byte)215, (byte)10, (byte)237, (byte)2, (byte)72, (byte)166, (byte)228, (byte)227, (byte)10, (byte)122, (byte)79, (byte)212, (byte)83, (byte)65, (byte)166, (byte)22, (byte)109, (byte)166, (byte)86, (byte)223, (byte)104, (byte)36, (byte)23, (byte)63, (byte)137, (byte)152, (byte)66, (byte)85, (byte)147, (byte)16, (byte)244, (byte)43, (byte)11, (byte)97, (byte)225, (byte)157}, 0) ;
            p123.target_component = (byte)(byte)24;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.dgps_numch == (byte)(byte)152);
                Debug.Assert(pack.cog == (ushort)(ushort)44916);
                Debug.Assert(pack.vel == (ushort)(ushort)13361);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
                Debug.Assert(pack.time_usec == (ulong)8029512187689549825L);
                Debug.Assert(pack.alt == (int)1570242154);
                Debug.Assert(pack.epv == (ushort)(ushort)56901);
                Debug.Assert(pack.lon == (int)837082470);
                Debug.Assert(pack.dgps_age == (uint)3275301353U);
                Debug.Assert(pack.lat == (int) -1365357236);
                Debug.Assert(pack.eph == (ushort)(ushort)8496);
                Debug.Assert(pack.satellites_visible == (byte)(byte)12);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.dgps_age = (uint)3275301353U;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX;
            p124.eph = (ushort)(ushort)8496;
            p124.epv = (ushort)(ushort)56901;
            p124.lat = (int) -1365357236;
            p124.satellites_visible = (byte)(byte)12;
            p124.lon = (int)837082470;
            p124.dgps_numch = (byte)(byte)152;
            p124.alt = (int)1570242154;
            p124.vel = (ushort)(ushort)13361;
            p124.cog = (ushort)(ushort)44916;
            p124.time_usec = (ulong)8029512187689549825L;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vcc == (ushort)(ushort)23288);
                Debug.Assert(pack.Vservo == (ushort)(ushort)54715);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT));
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vservo = (ushort)(ushort)54715;
            p125.Vcc = (ushort)(ushort)23288;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT);
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.device == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1);
                Debug.Assert(pack.count == (byte)(byte)68);
                Debug.Assert(pack.timeout == (ushort)(ushort)55895);
                Debug.Assert(pack.baudrate == (uint)3398463142U);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)192, (byte)200, (byte)238, (byte)196, (byte)38, (byte)145, (byte)15, (byte)53, (byte)98, (byte)231, (byte)139, (byte)31, (byte)63, (byte)132, (byte)124, (byte)65, (byte)198, (byte)149, (byte)230, (byte)162, (byte)3, (byte)78, (byte)170, (byte)170, (byte)134, (byte)91, (byte)86, (byte)223, (byte)98, (byte)58, (byte)139, (byte)28, (byte)128, (byte)217, (byte)116, (byte)246, (byte)63, (byte)250, (byte)159, (byte)162, (byte)221, (byte)18, (byte)172, (byte)31, (byte)189, (byte)190, (byte)238, (byte)145, (byte)148, (byte)188, (byte)142, (byte)7, (byte)22, (byte)254, (byte)85, (byte)201, (byte)159, (byte)49, (byte)182, (byte)105, (byte)160, (byte)33, (byte)172, (byte)59, (byte)47, (byte)129, (byte)219, (byte)94, (byte)62, (byte)183}));
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING));
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.count = (byte)(byte)68;
            p126.data__SET(new byte[] {(byte)192, (byte)200, (byte)238, (byte)196, (byte)38, (byte)145, (byte)15, (byte)53, (byte)98, (byte)231, (byte)139, (byte)31, (byte)63, (byte)132, (byte)124, (byte)65, (byte)198, (byte)149, (byte)230, (byte)162, (byte)3, (byte)78, (byte)170, (byte)170, (byte)134, (byte)91, (byte)86, (byte)223, (byte)98, (byte)58, (byte)139, (byte)28, (byte)128, (byte)217, (byte)116, (byte)246, (byte)63, (byte)250, (byte)159, (byte)162, (byte)221, (byte)18, (byte)172, (byte)31, (byte)189, (byte)190, (byte)238, (byte)145, (byte)148, (byte)188, (byte)142, (byte)7, (byte)22, (byte)254, (byte)85, (byte)201, (byte)159, (byte)49, (byte)182, (byte)105, (byte)160, (byte)33, (byte)172, (byte)59, (byte)47, (byte)129, (byte)219, (byte)94, (byte)62, (byte)183}, 0) ;
            p126.timeout = (ushort)(ushort)55895;
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING);
            p126.baudrate = (uint)3398463142U;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_c_mm == (int)2054626170);
                Debug.Assert(pack.rtk_rate == (byte)(byte)229);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)189);
                Debug.Assert(pack.rtk_health == (byte)(byte)165);
                Debug.Assert(pack.iar_num_hypotheses == (int)1012169856);
                Debug.Assert(pack.accuracy == (uint)1677729116U);
                Debug.Assert(pack.tow == (uint)688224685U);
                Debug.Assert(pack.baseline_a_mm == (int)1451387894);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)10);
                Debug.Assert(pack.wn == (ushort)(ushort)34029);
                Debug.Assert(pack.time_last_baseline_ms == (uint)30138261U);
                Debug.Assert(pack.nsats == (byte)(byte)14);
                Debug.Assert(pack.baseline_b_mm == (int) -925653549);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.accuracy = (uint)1677729116U;
            p127.rtk_health = (byte)(byte)165;
            p127.baseline_coords_type = (byte)(byte)189;
            p127.rtk_rate = (byte)(byte)229;
            p127.rtk_receiver_id = (byte)(byte)10;
            p127.iar_num_hypotheses = (int)1012169856;
            p127.wn = (ushort)(ushort)34029;
            p127.baseline_b_mm = (int) -925653549;
            p127.baseline_a_mm = (int)1451387894;
            p127.baseline_c_mm = (int)2054626170;
            p127.nsats = (byte)(byte)14;
            p127.tow = (uint)688224685U;
            p127.time_last_baseline_ms = (uint)30138261U;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_c_mm == (int) -943203750);
                Debug.Assert(pack.rtk_rate == (byte)(byte)207);
                Debug.Assert(pack.wn == (ushort)(ushort)27988);
                Debug.Assert(pack.iar_num_hypotheses == (int) -378462207);
                Debug.Assert(pack.baseline_a_mm == (int) -1638599661);
                Debug.Assert(pack.accuracy == (uint)205946286U);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)160);
                Debug.Assert(pack.time_last_baseline_ms == (uint)2177893653U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)61);
                Debug.Assert(pack.tow == (uint)4194238169U);
                Debug.Assert(pack.baseline_b_mm == (int)1709709627);
                Debug.Assert(pack.rtk_health == (byte)(byte)209);
                Debug.Assert(pack.nsats == (byte)(byte)15);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.baseline_c_mm = (int) -943203750;
            p128.time_last_baseline_ms = (uint)2177893653U;
            p128.baseline_b_mm = (int)1709709627;
            p128.baseline_a_mm = (int) -1638599661;
            p128.rtk_health = (byte)(byte)209;
            p128.rtk_receiver_id = (byte)(byte)61;
            p128.baseline_coords_type = (byte)(byte)160;
            p128.rtk_rate = (byte)(byte)207;
            p128.accuracy = (uint)205946286U;
            p128.wn = (ushort)(ushort)27988;
            p128.nsats = (byte)(byte)15;
            p128.iar_num_hypotheses = (int) -378462207;
            p128.tow = (uint)4194238169U;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (short)(short)2591);
                Debug.Assert(pack.ymag == (short)(short)30578);
                Debug.Assert(pack.time_boot_ms == (uint)319130307U);
                Debug.Assert(pack.zmag == (short)(short)4715);
                Debug.Assert(pack.xacc == (short)(short)5265);
                Debug.Assert(pack.ygyro == (short)(short) -18562);
                Debug.Assert(pack.zacc == (short)(short) -18803);
                Debug.Assert(pack.xgyro == (short)(short)7523);
                Debug.Assert(pack.zgyro == (short)(short)12367);
                Debug.Assert(pack.yacc == (short)(short)1605);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.xmag = (short)(short)2591;
            p129.yacc = (short)(short)1605;
            p129.ymag = (short)(short)30578;
            p129.zmag = (short)(short)4715;
            p129.time_boot_ms = (uint)319130307U;
            p129.ygyro = (short)(short) -18562;
            p129.xgyro = (short)(short)7523;
            p129.xacc = (short)(short)5265;
            p129.zacc = (short)(short) -18803;
            p129.zgyro = (short)(short)12367;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)66);
                Debug.Assert(pack.jpg_quality == (byte)(byte)61);
                Debug.Assert(pack.width == (ushort)(ushort)12314);
                Debug.Assert(pack.height == (ushort)(ushort)51250);
                Debug.Assert(pack.size == (uint)3534167523U);
                Debug.Assert(pack.payload == (byte)(byte)161);
                Debug.Assert(pack.packets == (ushort)(ushort)54718);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.width = (ushort)(ushort)12314;
            p130.packets = (ushort)(ushort)54718;
            p130.jpg_quality = (byte)(byte)61;
            p130.type = (byte)(byte)66;
            p130.size = (uint)3534167523U;
            p130.payload = (byte)(byte)161;
            p130.height = (ushort)(ushort)51250;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seqnr == (ushort)(ushort)9380);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)68, (byte)224, (byte)166, (byte)83, (byte)27, (byte)78, (byte)241, (byte)152, (byte)148, (byte)154, (byte)198, (byte)193, (byte)242, (byte)81, (byte)86, (byte)213, (byte)236, (byte)79, (byte)71, (byte)226, (byte)110, (byte)165, (byte)239, (byte)138, (byte)202, (byte)106, (byte)241, (byte)51, (byte)24, (byte)133, (byte)132, (byte)26, (byte)244, (byte)82, (byte)193, (byte)114, (byte)160, (byte)49, (byte)171, (byte)116, (byte)67, (byte)36, (byte)238, (byte)61, (byte)174, (byte)140, (byte)189, (byte)68, (byte)215, (byte)39, (byte)187, (byte)5, (byte)162, (byte)39, (byte)201, (byte)117, (byte)103, (byte)192, (byte)249, (byte)146, (byte)115, (byte)7, (byte)110, (byte)211, (byte)132, (byte)216, (byte)104, (byte)235, (byte)109, (byte)142, (byte)180, (byte)160, (byte)62, (byte)19, (byte)54, (byte)77, (byte)91, (byte)110, (byte)109, (byte)222, (byte)16, (byte)78, (byte)107, (byte)1, (byte)20, (byte)119, (byte)174, (byte)38, (byte)244, (byte)38, (byte)79, (byte)102, (byte)86, (byte)171, (byte)246, (byte)119, (byte)207, (byte)218, (byte)23, (byte)70, (byte)184, (byte)27, (byte)214, (byte)202, (byte)79, (byte)194, (byte)55, (byte)136, (byte)213, (byte)62, (byte)108, (byte)59, (byte)92, (byte)8, (byte)157, (byte)56, (byte)24, (byte)237, (byte)16, (byte)72, (byte)226, (byte)127, (byte)73, (byte)100, (byte)29, (byte)5, (byte)95, (byte)215, (byte)131, (byte)24, (byte)181, (byte)130, (byte)146, (byte)181, (byte)92, (byte)151, (byte)254, (byte)201, (byte)31, (byte)160, (byte)98, (byte)31, (byte)62, (byte)191, (byte)252, (byte)216, (byte)209, (byte)253, (byte)219, (byte)112, (byte)98, (byte)28, (byte)242, (byte)35, (byte)182, (byte)113, (byte)95, (byte)209, (byte)81, (byte)163, (byte)47, (byte)140, (byte)191, (byte)35, (byte)178, (byte)255, (byte)206, (byte)15, (byte)128, (byte)96, (byte)77, (byte)102, (byte)255, (byte)32, (byte)72, (byte)118, (byte)237, (byte)75, (byte)141, (byte)218, (byte)89, (byte)147, (byte)223, (byte)3, (byte)200, (byte)57, (byte)194, (byte)204, (byte)178, (byte)188, (byte)162, (byte)46, (byte)100, (byte)198, (byte)250, (byte)153, (byte)37, (byte)63, (byte)97, (byte)249, (byte)255, (byte)55, (byte)123, (byte)68, (byte)160, (byte)179, (byte)79, (byte)185, (byte)25, (byte)81, (byte)181, (byte)4, (byte)79, (byte)55, (byte)23, (byte)246, (byte)226, (byte)25, (byte)97, (byte)162, (byte)189, (byte)59, (byte)29, (byte)194, (byte)117, (byte)147, (byte)185, (byte)250, (byte)80, (byte)98, (byte)146, (byte)117, (byte)18, (byte)213, (byte)90, (byte)182, (byte)171, (byte)19, (byte)76, (byte)171, (byte)129, (byte)124, (byte)155, (byte)145, (byte)250, (byte)8, (byte)136, (byte)251, (byte)169, (byte)203, (byte)48, (byte)230, (byte)43}));
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.data__SET(new byte[] {(byte)68, (byte)224, (byte)166, (byte)83, (byte)27, (byte)78, (byte)241, (byte)152, (byte)148, (byte)154, (byte)198, (byte)193, (byte)242, (byte)81, (byte)86, (byte)213, (byte)236, (byte)79, (byte)71, (byte)226, (byte)110, (byte)165, (byte)239, (byte)138, (byte)202, (byte)106, (byte)241, (byte)51, (byte)24, (byte)133, (byte)132, (byte)26, (byte)244, (byte)82, (byte)193, (byte)114, (byte)160, (byte)49, (byte)171, (byte)116, (byte)67, (byte)36, (byte)238, (byte)61, (byte)174, (byte)140, (byte)189, (byte)68, (byte)215, (byte)39, (byte)187, (byte)5, (byte)162, (byte)39, (byte)201, (byte)117, (byte)103, (byte)192, (byte)249, (byte)146, (byte)115, (byte)7, (byte)110, (byte)211, (byte)132, (byte)216, (byte)104, (byte)235, (byte)109, (byte)142, (byte)180, (byte)160, (byte)62, (byte)19, (byte)54, (byte)77, (byte)91, (byte)110, (byte)109, (byte)222, (byte)16, (byte)78, (byte)107, (byte)1, (byte)20, (byte)119, (byte)174, (byte)38, (byte)244, (byte)38, (byte)79, (byte)102, (byte)86, (byte)171, (byte)246, (byte)119, (byte)207, (byte)218, (byte)23, (byte)70, (byte)184, (byte)27, (byte)214, (byte)202, (byte)79, (byte)194, (byte)55, (byte)136, (byte)213, (byte)62, (byte)108, (byte)59, (byte)92, (byte)8, (byte)157, (byte)56, (byte)24, (byte)237, (byte)16, (byte)72, (byte)226, (byte)127, (byte)73, (byte)100, (byte)29, (byte)5, (byte)95, (byte)215, (byte)131, (byte)24, (byte)181, (byte)130, (byte)146, (byte)181, (byte)92, (byte)151, (byte)254, (byte)201, (byte)31, (byte)160, (byte)98, (byte)31, (byte)62, (byte)191, (byte)252, (byte)216, (byte)209, (byte)253, (byte)219, (byte)112, (byte)98, (byte)28, (byte)242, (byte)35, (byte)182, (byte)113, (byte)95, (byte)209, (byte)81, (byte)163, (byte)47, (byte)140, (byte)191, (byte)35, (byte)178, (byte)255, (byte)206, (byte)15, (byte)128, (byte)96, (byte)77, (byte)102, (byte)255, (byte)32, (byte)72, (byte)118, (byte)237, (byte)75, (byte)141, (byte)218, (byte)89, (byte)147, (byte)223, (byte)3, (byte)200, (byte)57, (byte)194, (byte)204, (byte)178, (byte)188, (byte)162, (byte)46, (byte)100, (byte)198, (byte)250, (byte)153, (byte)37, (byte)63, (byte)97, (byte)249, (byte)255, (byte)55, (byte)123, (byte)68, (byte)160, (byte)179, (byte)79, (byte)185, (byte)25, (byte)81, (byte)181, (byte)4, (byte)79, (byte)55, (byte)23, (byte)246, (byte)226, (byte)25, (byte)97, (byte)162, (byte)189, (byte)59, (byte)29, (byte)194, (byte)117, (byte)147, (byte)185, (byte)250, (byte)80, (byte)98, (byte)146, (byte)117, (byte)18, (byte)213, (byte)90, (byte)182, (byte)171, (byte)19, (byte)76, (byte)171, (byte)129, (byte)124, (byte)155, (byte)145, (byte)250, (byte)8, (byte)136, (byte)251, (byte)169, (byte)203, (byte)48, (byte)230, (byte)43}, 0) ;
            p131.seqnr = (ushort)(ushort)9380;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_distance == (ushort)(ushort)1020);
                Debug.Assert(pack.id == (byte)(byte)148);
                Debug.Assert(pack.type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
                Debug.Assert(pack.time_boot_ms == (uint)4232004909U);
                Debug.Assert(pack.max_distance == (ushort)(ushort)58474);
                Debug.Assert(pack.orientation == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_YAW_45);
                Debug.Assert(pack.min_distance == (ushort)(ushort)24030);
                Debug.Assert(pack.covariance == (byte)(byte)148);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.covariance = (byte)(byte)148;
            p132.current_distance = (ushort)(ushort)1020;
            p132.id = (byte)(byte)148;
            p132.time_boot_ms = (uint)4232004909U;
            p132.min_distance = (ushort)(ushort)24030;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_YAW_45;
            p132.max_distance = (ushort)(ushort)58474;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)48266);
                Debug.Assert(pack.mask == (ulong)5253906125960230339L);
                Debug.Assert(pack.lat == (int) -1969265824);
                Debug.Assert(pack.lon == (int)620934893);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lon = (int)620934893;
            p133.lat = (int) -1969265824;
            p133.grid_spacing = (ushort)(ushort)48266;
            p133.mask = (ulong)5253906125960230339L;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gridbit == (byte)(byte)241);
                Debug.Assert(pack.lat == (int) -1242211862);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short)25650, (short)20904, (short) -29064, (short)30947, (short) -19067, (short)766, (short)17505, (short) -5203, (short) -31657, (short)16362, (short)554, (short) -8900, (short) -5488, (short)6521, (short) -22521, (short)17779}));
                Debug.Assert(pack.lon == (int)1969522794);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)50345);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int) -1242211862;
            p134.gridbit = (byte)(byte)241;
            p134.lon = (int)1969522794;
            p134.data__SET(new short[] {(short)25650, (short)20904, (short) -29064, (short)30947, (short) -19067, (short)766, (short)17505, (short) -5203, (short) -31657, (short)16362, (short)554, (short) -8900, (short) -5488, (short)6521, (short) -22521, (short)17779}, 0) ;
            p134.grid_spacing = (ushort)(ushort)50345;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)733028163);
                Debug.Assert(pack.lon == (int) -1802753941);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)733028163;
            p135.lon = (int) -1802753941;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.spacing == (ushort)(ushort)19603);
                Debug.Assert(pack.loaded == (ushort)(ushort)41302);
                Debug.Assert(pack.lon == (int)1046561920);
                Debug.Assert(pack.terrain_height == (float)3.3711507E38F);
                Debug.Assert(pack.current_height == (float)3.2305315E37F);
                Debug.Assert(pack.lat == (int) -878836800);
                Debug.Assert(pack.pending == (ushort)(ushort)54172);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lat = (int) -878836800;
            p136.current_height = (float)3.2305315E37F;
            p136.loaded = (ushort)(ushort)41302;
            p136.terrain_height = (float)3.3711507E38F;
            p136.spacing = (ushort)(ushort)19603;
            p136.pending = (ushort)(ushort)54172;
            p136.lon = (int)1046561920;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)26399);
                Debug.Assert(pack.press_diff == (float) -4.1169126E37F);
                Debug.Assert(pack.press_abs == (float) -1.5353054E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2491127044U);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.temperature = (short)(short)26399;
            p137.press_abs = (float) -1.5353054E38F;
            p137.press_diff = (float) -4.1169126E37F;
            p137.time_boot_ms = (uint)2491127044U;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)1.0437978E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {7.4992724E37F, 7.832138E37F, 1.830992E38F, 1.0674343E38F}));
                Debug.Assert(pack.time_usec == (ulong)955761843356872805L);
                Debug.Assert(pack.z == (float)5.1144584E35F);
                Debug.Assert(pack.y == (float) -3.0845906E38F);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.q_SET(new float[] {7.4992724E37F, 7.832138E37F, 1.830992E38F, 1.0674343E38F}, 0) ;
            p138.time_usec = (ulong)955761843356872805L;
            p138.x = (float)1.0437978E38F;
            p138.y = (float) -3.0845906E38F;
            p138.z = (float)5.1144584E35F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)112);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-8.3912025E37F, -1.9392833E38F, 2.8812652E38F, -3.126742E38F, -1.3295223E38F, -1.6175065E38F, 2.034084E38F, 2.9548923E38F}));
                Debug.Assert(pack.time_usec == (ulong)2661366904432202103L);
                Debug.Assert(pack.target_component == (byte)(byte)27);
                Debug.Assert(pack.group_mlx == (byte)(byte)206);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.controls_SET(new float[] {-8.3912025E37F, -1.9392833E38F, 2.8812652E38F, -3.126742E38F, -1.3295223E38F, -1.6175065E38F, 2.034084E38F, 2.9548923E38F}, 0) ;
            p139.time_usec = (ulong)2661366904432202103L;
            p139.target_system = (byte)(byte)112;
            p139.group_mlx = (byte)(byte)206;
            p139.target_component = (byte)(byte)27;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)215);
                Debug.Assert(pack.time_usec == (ulong)1734569626501866949L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.194478E37F, -2.1043264E37F, -2.4434986E38F, -2.1840226E38F, -2.2539996E38F, -2.7772038E38F, 2.403517E38F, -6.3802817E37F}));
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.controls_SET(new float[] {-1.194478E37F, -2.1043264E37F, -2.4434986E38F, -2.1840226E38F, -2.2539996E38F, -2.7772038E38F, 2.403517E38F, -6.3802817E37F}, 0) ;
            p140.time_usec = (ulong)1734569626501866949L;
            p140.group_mlx = (byte)(byte)215;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_amsl == (float) -2.8403874E38F);
                Debug.Assert(pack.time_usec == (ulong)6032481198803455481L);
                Debug.Assert(pack.altitude_relative == (float)7.949383E37F);
                Debug.Assert(pack.bottom_clearance == (float)8.1019084E37F);
                Debug.Assert(pack.altitude_terrain == (float)6.940451E37F);
                Debug.Assert(pack.altitude_monotonic == (float)1.3139783E38F);
                Debug.Assert(pack.altitude_local == (float)1.2269599E38F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)6032481198803455481L;
            p141.altitude_terrain = (float)6.940451E37F;
            p141.altitude_local = (float)1.2269599E38F;
            p141.bottom_clearance = (float)8.1019084E37F;
            p141.altitude_monotonic = (float)1.3139783E38F;
            p141.altitude_relative = (float)7.949383E37F;
            p141.altitude_amsl = (float) -2.8403874E38F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri_type == (byte)(byte)127);
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)197, (byte)116, (byte)69, (byte)41, (byte)215, (byte)198, (byte)9, (byte)83, (byte)22, (byte)165, (byte)134, (byte)232, (byte)32, (byte)45, (byte)95, (byte)197, (byte)248, (byte)183, (byte)197, (byte)32, (byte)73, (byte)116, (byte)66, (byte)156, (byte)99, (byte)109, (byte)7, (byte)189, (byte)51, (byte)188, (byte)228, (byte)38, (byte)211, (byte)130, (byte)152, (byte)47, (byte)249, (byte)210, (byte)141, (byte)142, (byte)98, (byte)198, (byte)60, (byte)83, (byte)98, (byte)194, (byte)21, (byte)4, (byte)128, (byte)172, (byte)203, (byte)70, (byte)3, (byte)170, (byte)82, (byte)5, (byte)205, (byte)189, (byte)18, (byte)140, (byte)167, (byte)117, (byte)218, (byte)173, (byte)96, (byte)151, (byte)193, (byte)204, (byte)230, (byte)211, (byte)202, (byte)138, (byte)113, (byte)211, (byte)145, (byte)157, (byte)48, (byte)87, (byte)142, (byte)235, (byte)44, (byte)174, (byte)25, (byte)197, (byte)6, (byte)43, (byte)28, (byte)207, (byte)55, (byte)216, (byte)149, (byte)191, (byte)251, (byte)65, (byte)68, (byte)119, (byte)80, (byte)110, (byte)245, (byte)37, (byte)250, (byte)147, (byte)157, (byte)34, (byte)200, (byte)1, (byte)115, (byte)65, (byte)213, (byte)108, (byte)57, (byte)80, (byte)149, (byte)22, (byte)83, (byte)247, (byte)129, (byte)2, (byte)137, (byte)254}));
                Debug.Assert(pack.request_id == (byte)(byte)159);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)211, (byte)253, (byte)167, (byte)52, (byte)85, (byte)5, (byte)241, (byte)180, (byte)158, (byte)249, (byte)248, (byte)160, (byte)28, (byte)90, (byte)240, (byte)11, (byte)216, (byte)54, (byte)242, (byte)218, (byte)118, (byte)19, (byte)244, (byte)49, (byte)188, (byte)48, (byte)90, (byte)12, (byte)215, (byte)110, (byte)132, (byte)4, (byte)164, (byte)62, (byte)186, (byte)221, (byte)46, (byte)224, (byte)74, (byte)67, (byte)214, (byte)227, (byte)58, (byte)195, (byte)60, (byte)148, (byte)93, (byte)66, (byte)102, (byte)58, (byte)102, (byte)250, (byte)127, (byte)179, (byte)203, (byte)220, (byte)233, (byte)174, (byte)151, (byte)241, (byte)215, (byte)5, (byte)145, (byte)146, (byte)248, (byte)29, (byte)24, (byte)141, (byte)201, (byte)19, (byte)161, (byte)137, (byte)87, (byte)35, (byte)169, (byte)40, (byte)32, (byte)8, (byte)154, (byte)128, (byte)237, (byte)103, (byte)47, (byte)233, (byte)117, (byte)218, (byte)145, (byte)103, (byte)237, (byte)104, (byte)117, (byte)112, (byte)51, (byte)141, (byte)7, (byte)178, (byte)57, (byte)158, (byte)67, (byte)20, (byte)110, (byte)168, (byte)133, (byte)29, (byte)230, (byte)135, (byte)151, (byte)240, (byte)60, (byte)132, (byte)216, (byte)84, (byte)190, (byte)60, (byte)170, (byte)12, (byte)230, (byte)40, (byte)165, (byte)179}));
                Debug.Assert(pack.transfer_type == (byte)(byte)220);
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.uri_type = (byte)(byte)127;
            p142.uri_SET(new byte[] {(byte)211, (byte)253, (byte)167, (byte)52, (byte)85, (byte)5, (byte)241, (byte)180, (byte)158, (byte)249, (byte)248, (byte)160, (byte)28, (byte)90, (byte)240, (byte)11, (byte)216, (byte)54, (byte)242, (byte)218, (byte)118, (byte)19, (byte)244, (byte)49, (byte)188, (byte)48, (byte)90, (byte)12, (byte)215, (byte)110, (byte)132, (byte)4, (byte)164, (byte)62, (byte)186, (byte)221, (byte)46, (byte)224, (byte)74, (byte)67, (byte)214, (byte)227, (byte)58, (byte)195, (byte)60, (byte)148, (byte)93, (byte)66, (byte)102, (byte)58, (byte)102, (byte)250, (byte)127, (byte)179, (byte)203, (byte)220, (byte)233, (byte)174, (byte)151, (byte)241, (byte)215, (byte)5, (byte)145, (byte)146, (byte)248, (byte)29, (byte)24, (byte)141, (byte)201, (byte)19, (byte)161, (byte)137, (byte)87, (byte)35, (byte)169, (byte)40, (byte)32, (byte)8, (byte)154, (byte)128, (byte)237, (byte)103, (byte)47, (byte)233, (byte)117, (byte)218, (byte)145, (byte)103, (byte)237, (byte)104, (byte)117, (byte)112, (byte)51, (byte)141, (byte)7, (byte)178, (byte)57, (byte)158, (byte)67, (byte)20, (byte)110, (byte)168, (byte)133, (byte)29, (byte)230, (byte)135, (byte)151, (byte)240, (byte)60, (byte)132, (byte)216, (byte)84, (byte)190, (byte)60, (byte)170, (byte)12, (byte)230, (byte)40, (byte)165, (byte)179}, 0) ;
            p142.transfer_type = (byte)(byte)220;
            p142.storage_SET(new byte[] {(byte)197, (byte)116, (byte)69, (byte)41, (byte)215, (byte)198, (byte)9, (byte)83, (byte)22, (byte)165, (byte)134, (byte)232, (byte)32, (byte)45, (byte)95, (byte)197, (byte)248, (byte)183, (byte)197, (byte)32, (byte)73, (byte)116, (byte)66, (byte)156, (byte)99, (byte)109, (byte)7, (byte)189, (byte)51, (byte)188, (byte)228, (byte)38, (byte)211, (byte)130, (byte)152, (byte)47, (byte)249, (byte)210, (byte)141, (byte)142, (byte)98, (byte)198, (byte)60, (byte)83, (byte)98, (byte)194, (byte)21, (byte)4, (byte)128, (byte)172, (byte)203, (byte)70, (byte)3, (byte)170, (byte)82, (byte)5, (byte)205, (byte)189, (byte)18, (byte)140, (byte)167, (byte)117, (byte)218, (byte)173, (byte)96, (byte)151, (byte)193, (byte)204, (byte)230, (byte)211, (byte)202, (byte)138, (byte)113, (byte)211, (byte)145, (byte)157, (byte)48, (byte)87, (byte)142, (byte)235, (byte)44, (byte)174, (byte)25, (byte)197, (byte)6, (byte)43, (byte)28, (byte)207, (byte)55, (byte)216, (byte)149, (byte)191, (byte)251, (byte)65, (byte)68, (byte)119, (byte)80, (byte)110, (byte)245, (byte)37, (byte)250, (byte)147, (byte)157, (byte)34, (byte)200, (byte)1, (byte)115, (byte)65, (byte)213, (byte)108, (byte)57, (byte)80, (byte)149, (byte)22, (byte)83, (byte)247, (byte)129, (byte)2, (byte)137, (byte)254}, 0) ;
            p142.request_id = (byte)(byte)159;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -1732);
                Debug.Assert(pack.press_abs == (float)2.3528682E38F);
                Debug.Assert(pack.press_diff == (float)2.90308E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3123418775U);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.press_abs = (float)2.3528682E38F;
            p143.press_diff = (float)2.90308E38F;
            p143.temperature = (short)(short) -1732;
            p143.time_boot_ms = (uint)3123418775U;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {1.9312482E38F, -1.0789854E38F, -3.3016593E38F, 2.672763E38F}));
                Debug.Assert(pack.lat == (int)1460454803);
                Debug.Assert(pack.est_capabilities == (byte)(byte)120);
                Debug.Assert(pack.custom_state == (ulong)8097532051879678634L);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {4.29872E37F, 3.3002778E38F, -1.3025924E38F}));
                Debug.Assert(pack.rates.SequenceEqual(new float[] {1.4714158E38F, -2.6360555E38F, 2.0019302E38F}));
                Debug.Assert(pack.alt == (float)9.679389E37F);
                Debug.Assert(pack.lon == (int)1129422807);
                Debug.Assert(pack.timestamp == (ulong)6088606097053119727L);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {1.4527471E38F, -2.8272252E37F, -1.0750625E38F}));
                Debug.Assert(pack.vel.SequenceEqual(new float[] {2.474601E38F, -1.6069547E38F, 8.0376394E36F}));
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.alt = (float)9.679389E37F;
            p144.position_cov_SET(new float[] {1.4527471E38F, -2.8272252E37F, -1.0750625E38F}, 0) ;
            p144.attitude_q_SET(new float[] {1.9312482E38F, -1.0789854E38F, -3.3016593E38F, 2.672763E38F}, 0) ;
            p144.lat = (int)1460454803;
            p144.lon = (int)1129422807;
            p144.timestamp = (ulong)6088606097053119727L;
            p144.est_capabilities = (byte)(byte)120;
            p144.vel_SET(new float[] {2.474601E38F, -1.6069547E38F, 8.0376394E36F}, 0) ;
            p144.rates_SET(new float[] {1.4714158E38F, -2.6360555E38F, 2.0019302E38F}, 0) ;
            p144.custom_state = (ulong)8097532051879678634L;
            p144.acc_SET(new float[] {4.29872E37F, 3.3002778E38F, -1.3025924E38F}, 0) ;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {1.6057425E38F, -2.7142971E38F, 2.5224028E38F}));
                Debug.Assert(pack.y_acc == (float)2.723717E37F);
                Debug.Assert(pack.airspeed == (float)2.7050064E37F);
                Debug.Assert(pack.roll_rate == (float) -1.5884123E37F);
                Debug.Assert(pack.x_vel == (float)1.0788378E38F);
                Debug.Assert(pack.x_acc == (float) -1.7710963E38F);
                Debug.Assert(pack.z_acc == (float) -1.253133E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {-3.3166695E38F, 1.9237369E38F, -3.0919475E38F}));
                Debug.Assert(pack.x_pos == (float)2.9288984E38F);
                Debug.Assert(pack.z_vel == (float)6.95626E37F);
                Debug.Assert(pack.yaw_rate == (float)2.512545E38F);
                Debug.Assert(pack.z_pos == (float) -3.0103807E38F);
                Debug.Assert(pack.y_pos == (float) -2.493528E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.0557082E38F, 1.5413095E38F, -2.2716637E38F, -1.8585865E38F}));
                Debug.Assert(pack.y_vel == (float) -1.6231101E38F);
                Debug.Assert(pack.pitch_rate == (float) -2.1472222E38F);
                Debug.Assert(pack.time_usec == (ulong)7626516642897792621L);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.z_acc = (float) -1.253133E38F;
            p146.x_acc = (float) -1.7710963E38F;
            p146.z_vel = (float)6.95626E37F;
            p146.z_pos = (float) -3.0103807E38F;
            p146.roll_rate = (float) -1.5884123E37F;
            p146.y_acc = (float)2.723717E37F;
            p146.y_pos = (float) -2.493528E38F;
            p146.yaw_rate = (float)2.512545E38F;
            p146.vel_variance_SET(new float[] {-3.3166695E38F, 1.9237369E38F, -3.0919475E38F}, 0) ;
            p146.pos_variance_SET(new float[] {1.6057425E38F, -2.7142971E38F, 2.5224028E38F}, 0) ;
            p146.x_vel = (float)1.0788378E38F;
            p146.y_vel = (float) -1.6231101E38F;
            p146.airspeed = (float)2.7050064E37F;
            p146.time_usec = (ulong)7626516642897792621L;
            p146.x_pos = (float)2.9288984E38F;
            p146.pitch_rate = (float) -2.1472222E38F;
            p146.q_SET(new float[] {1.0557082E38F, 1.5413095E38F, -2.2716637E38F, -1.8585865E38F}, 0) ;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.battery_function == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
                Debug.Assert(pack.energy_consumed == (int) -819470892);
                Debug.Assert(pack.type == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)9594, (ushort)24948, (ushort)26163, (ushort)1479, (ushort)35872, (ushort)40675, (ushort)30443, (ushort)44305, (ushort)24615, (ushort)4809}));
                Debug.Assert(pack.id == (byte)(byte)66);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)16);
                Debug.Assert(pack.temperature == (short)(short) -11910);
                Debug.Assert(pack.current_battery == (short)(short) -14389);
                Debug.Assert(pack.current_consumed == (int) -302745240);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH;
            p147.id = (byte)(byte)66;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL;
            p147.voltages_SET(new ushort[] {(ushort)9594, (ushort)24948, (ushort)26163, (ushort)1479, (ushort)35872, (ushort)40675, (ushort)30443, (ushort)44305, (ushort)24615, (ushort)4809}, 0) ;
            p147.current_battery = (short)(short) -14389;
            p147.temperature = (short)(short) -11910;
            p147.current_consumed = (int) -302745240;
            p147.battery_remaining = (sbyte)(sbyte)16;
            p147.energy_consumed = (int) -819470892;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)76, (byte)84, (byte)147, (byte)198, (byte)107, (byte)98, (byte)40, (byte)245, (byte)203, (byte)10, (byte)13, (byte)149, (byte)212, (byte)138, (byte)129, (byte)161, (byte)210, (byte)176}));
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION));
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)252, (byte)38, (byte)197, (byte)187, (byte)55, (byte)82, (byte)129, (byte)59}));
                Debug.Assert(pack.vendor_id == (ushort)(ushort)18921);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)234, (byte)155, (byte)39, (byte)176, (byte)218, (byte)27, (byte)216, (byte)102}));
                Debug.Assert(pack.os_sw_version == (uint)2948803619U);
                Debug.Assert(pack.flight_sw_version == (uint)319430786U);
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)255, (byte)183, (byte)95, (byte)234, (byte)32, (byte)243, (byte)56, (byte)53}));
                Debug.Assert(pack.product_id == (ushort)(ushort)53421);
                Debug.Assert(pack.board_version == (uint)1636362152U);
                Debug.Assert(pack.middleware_sw_version == (uint)978440780U);
                Debug.Assert(pack.uid == (ulong)7878612844742854332L);
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.flight_sw_version = (uint)319430786U;
            p148.flight_custom_version_SET(new byte[] {(byte)252, (byte)38, (byte)197, (byte)187, (byte)55, (byte)82, (byte)129, (byte)59}, 0) ;
            p148.uid2_SET(new byte[] {(byte)76, (byte)84, (byte)147, (byte)198, (byte)107, (byte)98, (byte)40, (byte)245, (byte)203, (byte)10, (byte)13, (byte)149, (byte)212, (byte)138, (byte)129, (byte)161, (byte)210, (byte)176}, 0, PH) ;
            p148.middleware_sw_version = (uint)978440780U;
            p148.middleware_custom_version_SET(new byte[] {(byte)234, (byte)155, (byte)39, (byte)176, (byte)218, (byte)27, (byte)216, (byte)102}, 0) ;
            p148.product_id = (ushort)(ushort)53421;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION);
            p148.os_sw_version = (uint)2948803619U;
            p148.uid = (ulong)7878612844742854332L;
            p148.board_version = (uint)1636362152U;
            p148.vendor_id = (ushort)(ushort)18921;
            p148.os_custom_version_SET(new byte[] {(byte)255, (byte)183, (byte)95, (byte)234, (byte)32, (byte)243, (byte)56, (byte)53}, 0) ;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.angle_x == (float) -1.0766874E38F);
                Debug.Assert(pack.y_TRY(ph) == (float) -1.1773563E38F);
                Debug.Assert(pack.z_TRY(ph) == (float)1.0904718E38F);
                Debug.Assert(pack.x_TRY(ph) == (float)7.799929E37F);
                Debug.Assert(pack.size_x == (float)2.1470985E38F);
                Debug.Assert(pack.type == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL);
                Debug.Assert(pack.target_num == (byte)(byte)238);
                Debug.Assert(pack.size_y == (float)1.6280019E38F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)91);
                Debug.Assert(pack.time_usec == (ulong)4479096209217537915L);
                Debug.Assert(pack.distance == (float)1.0672612E38F);
                Debug.Assert(pack.angle_y == (float)8.048432E37F);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {-2.7638836E38F, -1.588138E38F, 1.8889884E38F, 2.8048615E38F}));
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_MISSION);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.angle_y = (float)8.048432E37F;
            p149.time_usec = (ulong)4479096209217537915L;
            p149.x_SET((float)7.799929E37F, PH) ;
            p149.frame = MAV_FRAME.MAV_FRAME_MISSION;
            p149.angle_x = (float) -1.0766874E38F;
            p149.z_SET((float)1.0904718E38F, PH) ;
            p149.size_y = (float)1.6280019E38F;
            p149.y_SET((float) -1.1773563E38F, PH) ;
            p149.q_SET(new float[] {-2.7638836E38F, -1.588138E38F, 1.8889884E38F, 2.8048615E38F}, 0, PH) ;
            p149.distance = (float)1.0672612E38F;
            p149.size_x = (float)2.1470985E38F;
            p149.position_valid_SET((byte)(byte)91, PH) ;
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL;
            p149.target_num = (byte)(byte)238;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnCPU_LOADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.batVolt == (ushort)(ushort)40849);
                Debug.Assert(pack.sensLoad == (byte)(byte)133);
                Debug.Assert(pack.ctrlLoad == (byte)(byte)124);
            };
            GroundControl.CPU_LOAD p170 = CommunicationChannel.new_CPU_LOAD();
            PH.setPack(p170);
            p170.ctrlLoad = (byte)(byte)124;
            p170.batVolt = (ushort)(ushort)40849;
            p170.sensLoad = (byte)(byte)133;
            CommunicationChannel.instance.send(p170);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSENSOR_BIASReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gyBias == (float) -1.4777431E38F);
                Debug.Assert(pack.gxBias == (float)3.1431672E38F);
                Debug.Assert(pack.ayBias == (float) -8.509587E37F);
                Debug.Assert(pack.azBias == (float) -1.1832802E38F);
                Debug.Assert(pack.gzBias == (float) -2.5802927E38F);
                Debug.Assert(pack.axBias == (float) -1.6935337E38F);
            };
            GroundControl.SENSOR_BIAS p172 = CommunicationChannel.new_SENSOR_BIAS();
            PH.setPack(p172);
            p172.azBias = (float) -1.1832802E38F;
            p172.ayBias = (float) -8.509587E37F;
            p172.gzBias = (float) -2.5802927E38F;
            p172.gyBias = (float) -1.4777431E38F;
            p172.axBias = (float) -1.6935337E38F;
            p172.gxBias = (float)3.1431672E38F;
            CommunicationChannel.instance.send(p172);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDIAGNOSTICReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.diagFl1 == (float) -1.6227784E37F);
                Debug.Assert(pack.diagSh2 == (short)(short)16984);
                Debug.Assert(pack.diagSh3 == (short)(short) -28616);
                Debug.Assert(pack.diagFl3 == (float) -1.1056156E38F);
                Debug.Assert(pack.diagSh1 == (short)(short)9204);
                Debug.Assert(pack.diagFl2 == (float) -2.8320103E38F);
            };
            GroundControl.DIAGNOSTIC p173 = CommunicationChannel.new_DIAGNOSTIC();
            PH.setPack(p173);
            p173.diagSh2 = (short)(short)16984;
            p173.diagSh1 = (short)(short)9204;
            p173.diagFl2 = (float) -2.8320103E38F;
            p173.diagFl3 = (float) -1.1056156E38F;
            p173.diagFl1 = (float) -1.6227784E37F;
            p173.diagSh3 = (short)(short) -28616;
            CommunicationChannel.instance.send(p173);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSLUGS_NAVIGATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.phi_c == (float) -1.8287188E38F);
                Debug.Assert(pack.h_c == (ushort)(ushort)55217);
                Debug.Assert(pack.toWP == (byte)(byte)43);
                Debug.Assert(pack.ay_body == (float) -1.2622591E38F);
                Debug.Assert(pack.psiDot_c == (float)3.0484405E38F);
                Debug.Assert(pack.theta_c == (float) -1.1579673E38F);
                Debug.Assert(pack.fromWP == (byte)(byte)79);
                Debug.Assert(pack.totalDist == (float) -2.1723617E38F);
                Debug.Assert(pack.u_m == (float) -3.82082E37F);
                Debug.Assert(pack.dist2Go == (float)2.4425327E38F);
            };
            GroundControl.SLUGS_NAVIGATION p176 = CommunicationChannel.new_SLUGS_NAVIGATION();
            PH.setPack(p176);
            p176.toWP = (byte)(byte)43;
            p176.phi_c = (float) -1.8287188E38F;
            p176.fromWP = (byte)(byte)79;
            p176.psiDot_c = (float)3.0484405E38F;
            p176.totalDist = (float) -2.1723617E38F;
            p176.h_c = (ushort)(ushort)55217;
            p176.ay_body = (float) -1.2622591E38F;
            p176.dist2Go = (float)2.4425327E38F;
            p176.theta_c = (float) -1.1579673E38F;
            p176.u_m = (float) -3.82082E37F;
            CommunicationChannel.instance.send(p176);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDATA_LOGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fl_3 == (float)3.3008833E38F);
                Debug.Assert(pack.fl_4 == (float) -3.3254904E37F);
                Debug.Assert(pack.fl_6 == (float)5.391398E37F);
                Debug.Assert(pack.fl_2 == (float)2.7819428E38F);
                Debug.Assert(pack.fl_5 == (float) -2.4977706E37F);
                Debug.Assert(pack.fl_1 == (float) -2.3602963E38F);
            };
            GroundControl.DATA_LOG p177 = CommunicationChannel.new_DATA_LOG();
            PH.setPack(p177);
            p177.fl_3 = (float)3.3008833E38F;
            p177.fl_4 = (float) -3.3254904E37F;
            p177.fl_6 = (float)5.391398E37F;
            p177.fl_1 = (float) -2.3602963E38F;
            p177.fl_5 = (float) -2.4977706E37F;
            p177.fl_2 = (float)2.7819428E38F;
            CommunicationChannel.instance.send(p177);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_DATE_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.useSat == (byte)(byte)150);
                Debug.Assert(pack.day == (byte)(byte)30);
                Debug.Assert(pack.sec == (byte)(byte)161);
                Debug.Assert(pack.year == (byte)(byte)78);
                Debug.Assert(pack.hour == (byte)(byte)125);
                Debug.Assert(pack.min == (byte)(byte)51);
                Debug.Assert(pack.month == (byte)(byte)191);
                Debug.Assert(pack.sigUsedMask == (byte)(byte)11);
                Debug.Assert(pack.clockStat == (byte)(byte)62);
                Debug.Assert(pack.visSat == (byte)(byte)147);
                Debug.Assert(pack.percentUsed == (byte)(byte)50);
                Debug.Assert(pack.GppGl == (byte)(byte)122);
            };
            GroundControl.GPS_DATE_TIME p179 = CommunicationChannel.new_GPS_DATE_TIME();
            PH.setPack(p179);
            p179.sigUsedMask = (byte)(byte)11;
            p179.day = (byte)(byte)30;
            p179.percentUsed = (byte)(byte)50;
            p179.min = (byte)(byte)51;
            p179.month = (byte)(byte)191;
            p179.clockStat = (byte)(byte)62;
            p179.visSat = (byte)(byte)147;
            p179.year = (byte)(byte)78;
            p179.hour = (byte)(byte)125;
            p179.useSat = (byte)(byte)150;
            p179.GppGl = (byte)(byte)122;
            p179.sec = (byte)(byte)161;
            CommunicationChannel.instance.send(p179);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMID_LVL_CMDSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target == (byte)(byte)76);
                Debug.Assert(pack.uCommand == (float) -2.1383826E38F);
                Debug.Assert(pack.rCommand == (float) -1.1476292E38F);
                Debug.Assert(pack.hCommand == (float)2.1234788E38F);
            };
            GroundControl.MID_LVL_CMDS p180 = CommunicationChannel.new_MID_LVL_CMDS();
            PH.setPack(p180);
            p180.uCommand = (float) -2.1383826E38F;
            p180.target = (byte)(byte)76;
            p180.hCommand = (float)2.1234788E38F;
            p180.rCommand = (float) -1.1476292E38F;
            CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCTRL_SRFC_PTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.bitfieldPt == (ushort)(ushort)13111);
                Debug.Assert(pack.target == (byte)(byte)209);
            };
            GroundControl.CTRL_SRFC_PT p181 = CommunicationChannel.new_CTRL_SRFC_PT();
            PH.setPack(p181);
            p181.bitfieldPt = (ushort)(ushort)13111;
            p181.target = (byte)(byte)209;
            CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSLUGS_CAMERA_ORDERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.moveHome == (sbyte)(sbyte) - 93);
                Debug.Assert(pack.zoom == (sbyte)(sbyte) - 64);
                Debug.Assert(pack.tilt == (sbyte)(sbyte) - 25);
                Debug.Assert(pack.pan == (sbyte)(sbyte)54);
                Debug.Assert(pack.target == (byte)(byte)61);
            };
            GroundControl.SLUGS_CAMERA_ORDER p184 = CommunicationChannel.new_SLUGS_CAMERA_ORDER();
            PH.setPack(p184);
            p184.tilt = (sbyte)(sbyte) - 25;
            p184.zoom = (sbyte)(sbyte) - 64;
            p184.target = (byte)(byte)61;
            p184.moveHome = (sbyte)(sbyte) - 93;
            p184.pan = (sbyte)(sbyte)54;
            CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCONTROL_SURFACEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.idSurface == (byte)(byte)216);
                Debug.Assert(pack.bControl == (float)9.614548E36F);
                Debug.Assert(pack.mControl == (float)2.2070493E38F);
                Debug.Assert(pack.target == (byte)(byte)61);
            };
            GroundControl.CONTROL_SURFACE p185 = CommunicationChannel.new_CONTROL_SURFACE();
            PH.setPack(p185);
            p185.idSurface = (byte)(byte)216;
            p185.mControl = (float)2.2070493E38F;
            p185.bControl = (float)9.614548E36F;
            p185.target = (byte)(byte)61;
            CommunicationChannel.instance.send(p185);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSLUGS_MOBILE_LOCATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (float)1.9359594E37F);
                Debug.Assert(pack.latitude == (float) -1.6044644E38F);
                Debug.Assert(pack.target == (byte)(byte)91);
            };
            GroundControl.SLUGS_MOBILE_LOCATION p186 = CommunicationChannel.new_SLUGS_MOBILE_LOCATION();
            PH.setPack(p186);
            p186.latitude = (float) -1.6044644E38F;
            p186.target = (byte)(byte)91;
            p186.longitude = (float)1.9359594E37F;
            CommunicationChannel.instance.send(p186);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSLUGS_CONFIGURATION_CAMERAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target == (byte)(byte)4);
                Debug.Assert(pack.order == (byte)(byte)214);
                Debug.Assert(pack.idOrder == (byte)(byte)153);
            };
            GroundControl.SLUGS_CONFIGURATION_CAMERA p188 = CommunicationChannel.new_SLUGS_CONFIGURATION_CAMERA();
            PH.setPack(p188);
            p188.target = (byte)(byte)4;
            p188.idOrder = (byte)(byte)153;
            p188.order = (byte)(byte)214;
            CommunicationChannel.instance.send(p188);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnISR_LOCATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.option1 == (byte)(byte)100);
                Debug.Assert(pack.longitude == (float) -1.6399399E38F);
                Debug.Assert(pack.target == (byte)(byte)92);
                Debug.Assert(pack.height == (float) -4.813937E37F);
                Debug.Assert(pack.latitude == (float)6.6704753E37F);
                Debug.Assert(pack.option2 == (byte)(byte)199);
                Debug.Assert(pack.option3 == (byte)(byte)226);
            };
            GroundControl.ISR_LOCATION p189 = CommunicationChannel.new_ISR_LOCATION();
            PH.setPack(p189);
            p189.height = (float) -4.813937E37F;
            p189.longitude = (float) -1.6399399E38F;
            p189.option2 = (byte)(byte)199;
            p189.option3 = (byte)(byte)226;
            p189.target = (byte)(byte)92;
            p189.option1 = (byte)(byte)100;
            p189.latitude = (float)6.6704753E37F;
            CommunicationChannel.instance.send(p189);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVOLT_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.voltage == (ushort)(ushort)16954);
                Debug.Assert(pack.r2Type == (byte)(byte)21);
                Debug.Assert(pack.reading2 == (ushort)(ushort)63640);
            };
            GroundControl.VOLT_SENSOR p191 = CommunicationChannel.new_VOLT_SENSOR();
            PH.setPack(p191);
            p191.voltage = (ushort)(ushort)16954;
            p191.r2Type = (byte)(byte)21;
            p191.reading2 = (ushort)(ushort)63640;
            CommunicationChannel.instance.send(p191);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPTZ_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tilt == (short)(short) -10354);
                Debug.Assert(pack.zoom == (byte)(byte)91);
                Debug.Assert(pack.pan == (short)(short) -18533);
            };
            GroundControl.PTZ_STATUS p192 = CommunicationChannel.new_PTZ_STATUS();
            PH.setPack(p192);
            p192.zoom = (byte)(byte)91;
            p192.pan = (short)(short) -18533;
            p192.tilt = (short)(short) -10354;
            CommunicationChannel.instance.send(p192);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAV_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.course == (float) -3.1249508E38F);
                Debug.Assert(pack.target == (byte)(byte)180);
                Debug.Assert(pack.longitude == (float)2.7708544E38F);
                Debug.Assert(pack.latitude == (float)1.798136E38F);
                Debug.Assert(pack.altitude == (float)3.8770683E37F);
                Debug.Assert(pack.speed == (float) -3.0054205E38F);
            };
            GroundControl.UAV_STATUS p193 = CommunicationChannel.new_UAV_STATUS();
            PH.setPack(p193);
            p193.longitude = (float)2.7708544E38F;
            p193.target = (byte)(byte)180;
            p193.altitude = (float)3.8770683E37F;
            p193.latitude = (float)1.798136E38F;
            p193.course = (float) -3.1249508E38F;
            p193.speed = (float) -3.0054205E38F;
            CommunicationChannel.instance.send(p193);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTATUS_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.magDir == (sbyte)(sbyte) - 114);
                Debug.Assert(pack.csFails == (ushort)(ushort)48279);
                Debug.Assert(pack.modeInd == (byte)(byte)107);
                Debug.Assert(pack.msgsType == (byte)(byte)140);
                Debug.Assert(pack.posStatus == (byte)(byte)148);
                Debug.Assert(pack.gpsQuality == (byte)(byte)108);
                Debug.Assert(pack.magVar == (float)2.626941E38F);
            };
            GroundControl.STATUS_GPS p194 = CommunicationChannel.new_STATUS_GPS();
            PH.setPack(p194);
            p194.csFails = (ushort)(ushort)48279;
            p194.msgsType = (byte)(byte)140;
            p194.posStatus = (byte)(byte)148;
            p194.magVar = (float)2.626941E38F;
            p194.modeInd = (byte)(byte)107;
            p194.gpsQuality = (byte)(byte)108;
            p194.magDir = (sbyte)(sbyte) - 114;
            CommunicationChannel.instance.send(p194);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNOVATEL_DIAGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.csFails == (ushort)(ushort)7345);
                Debug.Assert(pack.posType == (byte)(byte)26);
                Debug.Assert(pack.timeStatus == (byte)(byte)208);
                Debug.Assert(pack.posSolAge == (float) -2.1388322E38F);
                Debug.Assert(pack.receiverStatus == (uint)2567435470U);
                Debug.Assert(pack.solStatus == (byte)(byte)150);
                Debug.Assert(pack.velType == (byte)(byte)81);
            };
            GroundControl.NOVATEL_DIAG p195 = CommunicationChannel.new_NOVATEL_DIAG();
            PH.setPack(p195);
            p195.posType = (byte)(byte)26;
            p195.velType = (byte)(byte)81;
            p195.solStatus = (byte)(byte)150;
            p195.receiverStatus = (uint)2567435470U;
            p195.posSolAge = (float) -2.1388322E38F;
            p195.csFails = (ushort)(ushort)7345;
            p195.timeStatus = (byte)(byte)208;
            CommunicationChannel.instance.send(p195);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSENSOR_DIAGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.int1 == (short)(short)24445);
                Debug.Assert(pack.float1 == (float) -2.0756951E38F);
                Debug.Assert(pack.float2 == (float) -7.456972E37F);
                Debug.Assert(pack.char1 == (sbyte)(sbyte)39);
            };
            GroundControl.SENSOR_DIAG p196 = CommunicationChannel.new_SENSOR_DIAG();
            PH.setPack(p196);
            p196.float1 = (float) -2.0756951E38F;
            p196.char1 = (sbyte)(sbyte)39;
            p196.int1 = (short)(short)24445;
            p196.float2 = (float) -7.456972E37F;
            CommunicationChannel.instance.send(p196);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBOOTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.version == (uint)1777929671U);
            };
            GroundControl.BOOT p197 = CommunicationChannel.new_BOOT();
            PH.setPack(p197);
            p197.version = (uint)1777929671U;
            CommunicationChannel.instance.send(p197);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            CommunicationChannel.instance.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tas_ratio == (float) -2.851413E37F);
                Debug.Assert(pack.hagl_ratio == (float)1.5147033E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL));
                Debug.Assert(pack.pos_horiz_accuracy == (float)1.4119958E38F);
                Debug.Assert(pack.pos_vert_accuracy == (float) -5.7212043E37F);
                Debug.Assert(pack.pos_vert_ratio == (float) -2.316532E38F);
                Debug.Assert(pack.vel_ratio == (float) -5.7887533E37F);
                Debug.Assert(pack.mag_ratio == (float)2.7043695E38F);
                Debug.Assert(pack.pos_horiz_ratio == (float)2.9958306E38F);
                Debug.Assert(pack.time_usec == (ulong)594071470057563947L);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.pos_vert_accuracy = (float) -5.7212043E37F;
            p230.vel_ratio = (float) -5.7887533E37F;
            p230.pos_horiz_accuracy = (float)1.4119958E38F;
            p230.pos_vert_ratio = (float) -2.316532E38F;
            p230.pos_horiz_ratio = (float)2.9958306E38F;
            p230.time_usec = (ulong)594071470057563947L;
            p230.mag_ratio = (float)2.7043695E38F;
            p230.hagl_ratio = (float)1.5147033E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL);
            p230.tas_ratio = (float) -2.851413E37F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wind_z == (float) -2.3333377E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -1.4939118E38F);
                Debug.Assert(pack.var_vert == (float)2.5408896E38F);
                Debug.Assert(pack.wind_x == (float) -8.474029E37F);
                Debug.Assert(pack.vert_accuracy == (float)8.4123317E37F);
                Debug.Assert(pack.var_horiz == (float)8.788658E37F);
                Debug.Assert(pack.wind_alt == (float)3.622203E37F);
                Debug.Assert(pack.wind_y == (float)6.180345E37F);
                Debug.Assert(pack.time_usec == (ulong)6586478178913096031L);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.wind_x = (float) -8.474029E37F;
            p231.horiz_accuracy = (float) -1.4939118E38F;
            p231.vert_accuracy = (float)8.4123317E37F;
            p231.var_horiz = (float)8.788658E37F;
            p231.wind_alt = (float)3.622203E37F;
            p231.var_vert = (float)2.5408896E38F;
            p231.time_usec = (ulong)6586478178913096031L;
            p231.wind_z = (float) -2.3333377E38F;
            p231.wind_y = (float)6.180345E37F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (float)1.2344129E38F);
                Debug.Assert(pack.lon == (int)110467984);
                Debug.Assert(pack.horiz_accuracy == (float) -2.388696E38F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT));
                Debug.Assert(pack.time_week == (ushort)(ushort)45168);
                Debug.Assert(pack.vn == (float)2.268811E38F);
                Debug.Assert(pack.time_usec == (ulong)3388728543202134394L);
                Debug.Assert(pack.satellites_visible == (byte)(byte)253);
                Debug.Assert(pack.hdop == (float)4.8804563E37F);
                Debug.Assert(pack.vdop == (float)4.949203E37F);
                Debug.Assert(pack.lat == (int)2049266067);
                Debug.Assert(pack.time_week_ms == (uint)2171533987U);
                Debug.Assert(pack.speed_accuracy == (float) -2.6715002E38F);
                Debug.Assert(pack.vd == (float) -2.4458432E38F);
                Debug.Assert(pack.fix_type == (byte)(byte)230);
                Debug.Assert(pack.gps_id == (byte)(byte)30);
                Debug.Assert(pack.ve == (float)4.5912593E36F);
                Debug.Assert(pack.vert_accuracy == (float)2.4374536E38F);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.speed_accuracy = (float) -2.6715002E38F;
            p232.gps_id = (byte)(byte)30;
            p232.lat = (int)2049266067;
            p232.hdop = (float)4.8804563E37F;
            p232.lon = (int)110467984;
            p232.time_usec = (ulong)3388728543202134394L;
            p232.vert_accuracy = (float)2.4374536E38F;
            p232.vn = (float)2.268811E38F;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT);
            p232.fix_type = (byte)(byte)230;
            p232.alt = (float)1.2344129E38F;
            p232.time_week_ms = (uint)2171533987U;
            p232.horiz_accuracy = (float) -2.388696E38F;
            p232.ve = (float)4.5912593E36F;
            p232.vdop = (float)4.949203E37F;
            p232.satellites_visible = (byte)(byte)253;
            p232.time_week = (ushort)(ushort)45168;
            p232.vd = (float) -2.4458432E38F;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)110);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)214, (byte)237, (byte)67, (byte)199, (byte)124, (byte)83, (byte)226, (byte)202, (byte)130, (byte)42, (byte)119, (byte)171, (byte)107, (byte)20, (byte)32, (byte)167, (byte)66, (byte)56, (byte)123, (byte)30, (byte)255, (byte)0, (byte)207, (byte)94, (byte)64, (byte)116, (byte)219, (byte)52, (byte)71, (byte)33, (byte)253, (byte)164, (byte)21, (byte)120, (byte)43, (byte)249, (byte)6, (byte)112, (byte)253, (byte)95, (byte)102, (byte)226, (byte)95, (byte)123, (byte)113, (byte)57, (byte)17, (byte)67, (byte)132, (byte)38, (byte)147, (byte)158, (byte)136, (byte)143, (byte)209, (byte)221, (byte)159, (byte)163, (byte)251, (byte)64, (byte)218, (byte)74, (byte)131, (byte)95, (byte)160, (byte)100, (byte)115, (byte)71, (byte)130, (byte)195, (byte)119, (byte)146, (byte)177, (byte)102, (byte)23, (byte)75, (byte)97, (byte)105, (byte)193, (byte)241, (byte)226, (byte)72, (byte)87, (byte)178, (byte)189, (byte)146, (byte)87, (byte)158, (byte)36, (byte)67, (byte)16, (byte)230, (byte)106, (byte)64, (byte)255, (byte)167, (byte)114, (byte)29, (byte)48, (byte)138, (byte)157, (byte)197, (byte)195, (byte)52, (byte)217, (byte)25, (byte)64, (byte)127, (byte)148, (byte)100, (byte)58, (byte)65, (byte)255, (byte)84, (byte)49, (byte)236, (byte)152, (byte)139, (byte)131, (byte)184, (byte)167, (byte)47, (byte)237, (byte)129, (byte)203, (byte)235, (byte)159, (byte)93, (byte)47, (byte)28, (byte)242, (byte)127, (byte)83, (byte)151, (byte)67, (byte)133, (byte)114, (byte)146, (byte)47, (byte)227, (byte)38, (byte)28, (byte)116, (byte)133, (byte)73, (byte)124, (byte)197, (byte)154, (byte)216, (byte)167, (byte)126, (byte)107, (byte)99, (byte)242, (byte)219, (byte)188, (byte)190, (byte)214, (byte)170, (byte)14, (byte)124, (byte)210, (byte)53, (byte)169, (byte)125, (byte)44, (byte)212, (byte)246, (byte)175, (byte)4, (byte)248, (byte)195, (byte)164, (byte)145, (byte)211, (byte)132, (byte)163, (byte)154, (byte)29, (byte)18}));
                Debug.Assert(pack.flags == (byte)(byte)8);
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.len = (byte)(byte)110;
            p233.data__SET(new byte[] {(byte)214, (byte)237, (byte)67, (byte)199, (byte)124, (byte)83, (byte)226, (byte)202, (byte)130, (byte)42, (byte)119, (byte)171, (byte)107, (byte)20, (byte)32, (byte)167, (byte)66, (byte)56, (byte)123, (byte)30, (byte)255, (byte)0, (byte)207, (byte)94, (byte)64, (byte)116, (byte)219, (byte)52, (byte)71, (byte)33, (byte)253, (byte)164, (byte)21, (byte)120, (byte)43, (byte)249, (byte)6, (byte)112, (byte)253, (byte)95, (byte)102, (byte)226, (byte)95, (byte)123, (byte)113, (byte)57, (byte)17, (byte)67, (byte)132, (byte)38, (byte)147, (byte)158, (byte)136, (byte)143, (byte)209, (byte)221, (byte)159, (byte)163, (byte)251, (byte)64, (byte)218, (byte)74, (byte)131, (byte)95, (byte)160, (byte)100, (byte)115, (byte)71, (byte)130, (byte)195, (byte)119, (byte)146, (byte)177, (byte)102, (byte)23, (byte)75, (byte)97, (byte)105, (byte)193, (byte)241, (byte)226, (byte)72, (byte)87, (byte)178, (byte)189, (byte)146, (byte)87, (byte)158, (byte)36, (byte)67, (byte)16, (byte)230, (byte)106, (byte)64, (byte)255, (byte)167, (byte)114, (byte)29, (byte)48, (byte)138, (byte)157, (byte)197, (byte)195, (byte)52, (byte)217, (byte)25, (byte)64, (byte)127, (byte)148, (byte)100, (byte)58, (byte)65, (byte)255, (byte)84, (byte)49, (byte)236, (byte)152, (byte)139, (byte)131, (byte)184, (byte)167, (byte)47, (byte)237, (byte)129, (byte)203, (byte)235, (byte)159, (byte)93, (byte)47, (byte)28, (byte)242, (byte)127, (byte)83, (byte)151, (byte)67, (byte)133, (byte)114, (byte)146, (byte)47, (byte)227, (byte)38, (byte)28, (byte)116, (byte)133, (byte)73, (byte)124, (byte)197, (byte)154, (byte)216, (byte)167, (byte)126, (byte)107, (byte)99, (byte)242, (byte)219, (byte)188, (byte)190, (byte)214, (byte)170, (byte)14, (byte)124, (byte)210, (byte)53, (byte)169, (byte)125, (byte)44, (byte)212, (byte)246, (byte)175, (byte)4, (byte)248, (byte)195, (byte)164, (byte)145, (byte)211, (byte)132, (byte)163, (byte)154, (byte)29, (byte)18}, 0) ;
            p233.flags = (byte)(byte)8;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.groundspeed == (byte)(byte)229);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte)14);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)39);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)977);
                Debug.Assert(pack.latitude == (int) -264655771);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)130);
                Debug.Assert(pack.gps_fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
                Debug.Assert(pack.wp_num == (byte)(byte)117);
                Debug.Assert(pack.temperature == (sbyte)(sbyte) - 12);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
                Debug.Assert(pack.airspeed == (byte)(byte)55);
                Debug.Assert(pack.altitude_amsl == (short)(short) -32421);
                Debug.Assert(pack.gps_nsat == (byte)(byte)18);
                Debug.Assert(pack.custom_mode == (uint)2502141133U);
                Debug.Assert(pack.heading == (ushort)(ushort)61471);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)102);
                Debug.Assert(pack.longitude == (int)2124994824);
                Debug.Assert(pack.battery_remaining == (byte)(byte)134);
                Debug.Assert(pack.heading_sp == (short)(short)20890);
                Debug.Assert(pack.pitch == (short)(short)2760);
                Debug.Assert(pack.failsafe == (byte)(byte)220);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
                Debug.Assert(pack.roll == (short)(short)23752);
                Debug.Assert(pack.altitude_sp == (short)(short) -3013);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.gps_nsat = (byte)(byte)18;
            p234.wp_num = (byte)(byte)117;
            p234.roll = (short)(short)23752;
            p234.heading_sp = (short)(short)20890;
            p234.pitch = (short)(short)2760;
            p234.wp_distance = (ushort)(ushort)977;
            p234.temperature_air = (sbyte)(sbyte)39;
            p234.climb_rate = (sbyte)(sbyte)14;
            p234.failsafe = (byte)(byte)220;
            p234.airspeed_sp = (byte)(byte)130;
            p234.altitude_amsl = (short)(short) -32421;
            p234.temperature = (sbyte)(sbyte) - 12;
            p234.throttle = (sbyte)(sbyte)102;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            p234.altitude_sp = (short)(short) -3013;
            p234.longitude = (int)2124994824;
            p234.latitude = (int) -264655771;
            p234.heading = (ushort)(ushort)61471;
            p234.airspeed = (byte)(byte)55;
            p234.custom_mode = (uint)2502141133U;
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
            p234.groundspeed = (byte)(byte)229;
            p234.battery_remaining = (byte)(byte)134;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)5966667999042232209L);
                Debug.Assert(pack.vibration_y == (float) -2.4570306E37F);
                Debug.Assert(pack.clipping_1 == (uint)1702906568U);
                Debug.Assert(pack.vibration_x == (float)1.4484925E37F);
                Debug.Assert(pack.clipping_2 == (uint)3482216535U);
                Debug.Assert(pack.clipping_0 == (uint)1285898621U);
                Debug.Assert(pack.vibration_z == (float) -9.508772E36F);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.vibration_y = (float) -2.4570306E37F;
            p241.clipping_0 = (uint)1285898621U;
            p241.clipping_2 = (uint)3482216535U;
            p241.time_usec = (ulong)5966667999042232209L;
            p241.clipping_1 = (uint)1702906568U;
            p241.vibration_z = (float) -9.508772E36F;
            p241.vibration_x = (float)1.4484925E37F;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.approach_z == (float)3.0336118E38F);
                Debug.Assert(pack.x == (float)2.8619415E38F);
                Debug.Assert(pack.y == (float)2.8344252E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.9807202E38F, 2.5920511E37F, 9.947394E37F, 1.9877477E38F}));
                Debug.Assert(pack.altitude == (int) -1612574784);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)4741052105168093030L);
                Debug.Assert(pack.z == (float) -2.3422333E38F);
                Debug.Assert(pack.latitude == (int) -893118870);
                Debug.Assert(pack.approach_y == (float) -2.6835465E38F);
                Debug.Assert(pack.longitude == (int)795204800);
                Debug.Assert(pack.approach_x == (float) -5.875329E37F);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.time_usec_SET((ulong)4741052105168093030L, PH) ;
            p242.altitude = (int) -1612574784;
            p242.q_SET(new float[] {-2.9807202E38F, 2.5920511E37F, 9.947394E37F, 1.9877477E38F}, 0) ;
            p242.longitude = (int)795204800;
            p242.latitude = (int) -893118870;
            p242.approach_z = (float)3.0336118E38F;
            p242.approach_y = (float) -2.6835465E38F;
            p242.y = (float)2.8344252E38F;
            p242.z = (float) -2.3422333E38F;
            p242.approach_x = (float) -5.875329E37F;
            p242.x = (float)2.8619415E38F;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int)685376754);
                Debug.Assert(pack.z == (float)1.543641E38F);
                Debug.Assert(pack.latitude == (int)910336550);
                Debug.Assert(pack.longitude == (int)522683741);
                Debug.Assert(pack.y == (float)3.0436366E38F);
                Debug.Assert(pack.target_system == (byte)(byte)17);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)4412429253836456492L);
                Debug.Assert(pack.approach_y == (float)6.491216E37F);
                Debug.Assert(pack.x == (float) -1.3534847E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.6976453E38F, -4.0925065E37F, -1.0224318E38F, 1.6139272E38F}));
                Debug.Assert(pack.approach_x == (float)6.6034115E37F);
                Debug.Assert(pack.approach_z == (float)1.454154E38F);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.z = (float)1.543641E38F;
            p243.approach_y = (float)6.491216E37F;
            p243.x = (float) -1.3534847E38F;
            p243.latitude = (int)910336550;
            p243.time_usec_SET((ulong)4412429253836456492L, PH) ;
            p243.longitude = (int)522683741;
            p243.approach_x = (float)6.6034115E37F;
            p243.altitude = (int)685376754;
            p243.target_system = (byte)(byte)17;
            p243.approach_z = (float)1.454154E38F;
            p243.q_SET(new float[] {2.6976453E38F, -4.0925065E37F, -1.0224318E38F, 1.6139272E38F}, 0) ;
            p243.y = (float)3.0436366E38F;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)64593);
                Debug.Assert(pack.interval_us == (int)1190673353);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)64593;
            p244.interval_us = (int)1190673353;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
                Debug.Assert(pack.vtol_state == MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_MC;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.callsign_LEN(ph) == 4);
                Debug.Assert(pack.callsign_TRY(ph).Equals("jyFa"));
                Debug.Assert(pack.lat == (int) -968753301);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)30699);
                Debug.Assert(pack.heading == (ushort)(ushort)54663);
                Debug.Assert(pack.lon == (int) -1773587453);
                Debug.Assert(pack.ver_velocity == (short)(short)7446);
                Debug.Assert(pack.flags == (ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN));
                Debug.Assert(pack.altitude == (int)1962502705);
                Debug.Assert(pack.emitter_type == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SMALL);
                Debug.Assert(pack.altitude_type == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
                Debug.Assert(pack.tslc == (byte)(byte)190);
                Debug.Assert(pack.ICAO_address == (uint)1003848102U);
                Debug.Assert(pack.squawk == (ushort)(ushort)31795);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.ver_velocity = (short)(short)7446;
            p246.lat = (int) -968753301;
            p246.callsign_SET("jyFa", PH) ;
            p246.tslc = (byte)(byte)190;
            p246.hor_velocity = (ushort)(ushort)30699;
            p246.ICAO_address = (uint)1003848102U;
            p246.altitude = (int)1962502705;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN);
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SMALL;
            p246.squawk = (ushort)(ushort)31795;
            p246.heading = (ushort)(ushort)54663;
            p246.lon = (int) -1773587453;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (uint)373580551U);
                Debug.Assert(pack.altitude_minimum_delta == (float)2.644949E38F);
                Debug.Assert(pack.src_ == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
                Debug.Assert(pack.horizontal_minimum_delta == (float)1.860929E38F);
                Debug.Assert(pack.time_to_minimum_delta == (float) -1.3093448E38F);
                Debug.Assert(pack.action == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH |
                                                   MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW));
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.time_to_minimum_delta = (float) -1.3093448E38F;
            p247.altitude_minimum_delta = (float)2.644949E38F;
            p247.horizontal_minimum_delta = (float)1.860929E38F;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH |
                                 MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
            p247.id = (uint)373580551U;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)225);
                Debug.Assert(pack.target_system == (byte)(byte)115);
                Debug.Assert(pack.message_type == (ushort)(ushort)28957);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)127, (byte)16, (byte)233, (byte)192, (byte)111, (byte)146, (byte)215, (byte)115, (byte)5, (byte)106, (byte)58, (byte)7, (byte)188, (byte)32, (byte)203, (byte)170, (byte)101, (byte)26, (byte)55, (byte)48, (byte)2, (byte)192, (byte)49, (byte)224, (byte)60, (byte)166, (byte)76, (byte)247, (byte)83, (byte)139, (byte)71, (byte)213, (byte)84, (byte)158, (byte)21, (byte)110, (byte)13, (byte)214, (byte)70, (byte)6, (byte)14, (byte)104, (byte)43, (byte)172, (byte)16, (byte)247, (byte)185, (byte)94, (byte)179, (byte)232, (byte)131, (byte)152, (byte)132, (byte)65, (byte)241, (byte)43, (byte)27, (byte)13, (byte)36, (byte)194, (byte)127, (byte)179, (byte)182, (byte)129, (byte)200, (byte)137, (byte)50, (byte)248, (byte)220, (byte)48, (byte)148, (byte)194, (byte)130, (byte)214, (byte)108, (byte)58, (byte)89, (byte)13, (byte)72, (byte)17, (byte)40, (byte)22, (byte)203, (byte)179, (byte)59, (byte)113, (byte)139, (byte)16, (byte)214, (byte)42, (byte)84, (byte)68, (byte)247, (byte)57, (byte)194, (byte)66, (byte)117, (byte)88, (byte)43, (byte)152, (byte)185, (byte)149, (byte)32, (byte)97, (byte)74, (byte)71, (byte)41, (byte)53, (byte)37, (byte)108, (byte)174, (byte)51, (byte)159, (byte)103, (byte)124, (byte)172, (byte)82, (byte)129, (byte)34, (byte)221, (byte)65, (byte)75, (byte)203, (byte)253, (byte)14, (byte)227, (byte)130, (byte)52, (byte)183, (byte)101, (byte)248, (byte)138, (byte)58, (byte)123, (byte)239, (byte)210, (byte)222, (byte)149, (byte)37, (byte)139, (byte)46, (byte)236, (byte)24, (byte)25, (byte)181, (byte)207, (byte)17, (byte)253, (byte)0, (byte)211, (byte)66, (byte)81, (byte)33, (byte)21, (byte)202, (byte)137, (byte)86, (byte)135, (byte)206, (byte)43, (byte)40, (byte)22, (byte)210, (byte)247, (byte)243, (byte)187, (byte)110, (byte)100, (byte)237, (byte)232, (byte)116, (byte)60, (byte)83, (byte)82, (byte)91, (byte)102, (byte)16, (byte)38, (byte)216, (byte)75, (byte)12, (byte)11, (byte)139, (byte)180, (byte)221, (byte)49, (byte)186, (byte)234, (byte)8, (byte)26, (byte)202, (byte)115, (byte)137, (byte)37, (byte)146, (byte)95, (byte)91, (byte)143, (byte)62, (byte)209, (byte)108, (byte)252, (byte)129, (byte)142, (byte)250, (byte)97, (byte)201, (byte)183, (byte)88, (byte)245, (byte)136, (byte)207, (byte)216, (byte)224, (byte)245, (byte)84, (byte)253, (byte)129, (byte)178, (byte)43, (byte)187, (byte)210, (byte)140, (byte)66, (byte)250, (byte)217, (byte)160, (byte)217, (byte)247, (byte)31, (byte)181, (byte)15, (byte)103, (byte)138, (byte)115, (byte)34, (byte)77, (byte)5, (byte)117, (byte)73, (byte)181, (byte)2, (byte)183, (byte)49, (byte)56, (byte)125, (byte)230, (byte)171, (byte)223}));
                Debug.Assert(pack.target_network == (byte)(byte)187);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_system = (byte)(byte)115;
            p248.target_component = (byte)(byte)225;
            p248.payload_SET(new byte[] {(byte)127, (byte)16, (byte)233, (byte)192, (byte)111, (byte)146, (byte)215, (byte)115, (byte)5, (byte)106, (byte)58, (byte)7, (byte)188, (byte)32, (byte)203, (byte)170, (byte)101, (byte)26, (byte)55, (byte)48, (byte)2, (byte)192, (byte)49, (byte)224, (byte)60, (byte)166, (byte)76, (byte)247, (byte)83, (byte)139, (byte)71, (byte)213, (byte)84, (byte)158, (byte)21, (byte)110, (byte)13, (byte)214, (byte)70, (byte)6, (byte)14, (byte)104, (byte)43, (byte)172, (byte)16, (byte)247, (byte)185, (byte)94, (byte)179, (byte)232, (byte)131, (byte)152, (byte)132, (byte)65, (byte)241, (byte)43, (byte)27, (byte)13, (byte)36, (byte)194, (byte)127, (byte)179, (byte)182, (byte)129, (byte)200, (byte)137, (byte)50, (byte)248, (byte)220, (byte)48, (byte)148, (byte)194, (byte)130, (byte)214, (byte)108, (byte)58, (byte)89, (byte)13, (byte)72, (byte)17, (byte)40, (byte)22, (byte)203, (byte)179, (byte)59, (byte)113, (byte)139, (byte)16, (byte)214, (byte)42, (byte)84, (byte)68, (byte)247, (byte)57, (byte)194, (byte)66, (byte)117, (byte)88, (byte)43, (byte)152, (byte)185, (byte)149, (byte)32, (byte)97, (byte)74, (byte)71, (byte)41, (byte)53, (byte)37, (byte)108, (byte)174, (byte)51, (byte)159, (byte)103, (byte)124, (byte)172, (byte)82, (byte)129, (byte)34, (byte)221, (byte)65, (byte)75, (byte)203, (byte)253, (byte)14, (byte)227, (byte)130, (byte)52, (byte)183, (byte)101, (byte)248, (byte)138, (byte)58, (byte)123, (byte)239, (byte)210, (byte)222, (byte)149, (byte)37, (byte)139, (byte)46, (byte)236, (byte)24, (byte)25, (byte)181, (byte)207, (byte)17, (byte)253, (byte)0, (byte)211, (byte)66, (byte)81, (byte)33, (byte)21, (byte)202, (byte)137, (byte)86, (byte)135, (byte)206, (byte)43, (byte)40, (byte)22, (byte)210, (byte)247, (byte)243, (byte)187, (byte)110, (byte)100, (byte)237, (byte)232, (byte)116, (byte)60, (byte)83, (byte)82, (byte)91, (byte)102, (byte)16, (byte)38, (byte)216, (byte)75, (byte)12, (byte)11, (byte)139, (byte)180, (byte)221, (byte)49, (byte)186, (byte)234, (byte)8, (byte)26, (byte)202, (byte)115, (byte)137, (byte)37, (byte)146, (byte)95, (byte)91, (byte)143, (byte)62, (byte)209, (byte)108, (byte)252, (byte)129, (byte)142, (byte)250, (byte)97, (byte)201, (byte)183, (byte)88, (byte)245, (byte)136, (byte)207, (byte)216, (byte)224, (byte)245, (byte)84, (byte)253, (byte)129, (byte)178, (byte)43, (byte)187, (byte)210, (byte)140, (byte)66, (byte)250, (byte)217, (byte)160, (byte)217, (byte)247, (byte)31, (byte)181, (byte)15, (byte)103, (byte)138, (byte)115, (byte)34, (byte)77, (byte)5, (byte)117, (byte)73, (byte)181, (byte)2, (byte)183, (byte)49, (byte)56, (byte)125, (byte)230, (byte)171, (byte)223}, 0) ;
            p248.message_type = (ushort)(ushort)28957;
            p248.target_network = (byte)(byte)187;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)2);
                Debug.Assert(pack.ver == (byte)(byte)92);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte) - 76, (sbyte) - 9, (sbyte) - 61, (sbyte) - 60, (sbyte)118, (sbyte)48, (sbyte)83, (sbyte)85, (sbyte) - 118, (sbyte) - 102, (sbyte) - 87, (sbyte) - 10, (sbyte) - 80, (sbyte) - 77, (sbyte) - 34, (sbyte) - 89, (sbyte)116, (sbyte) - 30, (sbyte) - 88, (sbyte) - 23, (sbyte) - 89, (sbyte)55, (sbyte) - 104, (sbyte) - 96, (sbyte) - 73, (sbyte)77, (sbyte)107, (sbyte)103, (sbyte)111, (sbyte)124, (sbyte) - 32, (sbyte) - 87}));
                Debug.Assert(pack.address == (ushort)(ushort)19956);
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.ver = (byte)(byte)92;
            p249.value_SET(new sbyte[] {(sbyte) - 76, (sbyte) - 9, (sbyte) - 61, (sbyte) - 60, (sbyte)118, (sbyte)48, (sbyte)83, (sbyte)85, (sbyte) - 118, (sbyte) - 102, (sbyte) - 87, (sbyte) - 10, (sbyte) - 80, (sbyte) - 77, (sbyte) - 34, (sbyte) - 89, (sbyte)116, (sbyte) - 30, (sbyte) - 88, (sbyte) - 23, (sbyte) - 89, (sbyte)55, (sbyte) - 104, (sbyte) - 96, (sbyte) - 73, (sbyte)77, (sbyte)107, (sbyte)103, (sbyte)111, (sbyte)124, (sbyte) - 32, (sbyte) - 87}, 0) ;
            p249.type = (byte)(byte)2;
            p249.address = (ushort)(ushort)19956;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)1434945336914705626L);
                Debug.Assert(pack.z == (float)2.2285022E38F);
                Debug.Assert(pack.x == (float)4.7917396E37F);
                Debug.Assert(pack.name_LEN(ph) == 10);
                Debug.Assert(pack.name_TRY(ph).Equals("efmdzftLCn"));
                Debug.Assert(pack.y == (float) -2.357719E38F);
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.time_usec = (ulong)1434945336914705626L;
            p250.y = (float) -2.357719E38F;
            p250.z = (float)2.2285022E38F;
            p250.name_SET("efmdzftLCn", PH) ;
            p250.x = (float)4.7917396E37F;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1852309107U);
                Debug.Assert(pack.value == (float) -9.064623E37F);
                Debug.Assert(pack.name_LEN(ph) == 1);
                Debug.Assert(pack.name_TRY(ph).Equals("f"));
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)1852309107U;
            p251.value = (float) -9.064623E37F;
            p251.name_SET("f", PH) ;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (int)642343919);
                Debug.Assert(pack.name_LEN(ph) == 7);
                Debug.Assert(pack.name_TRY(ph).Equals("vsTliAc"));
                Debug.Assert(pack.time_boot_ms == (uint)4138928157U);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.value = (int)642343919;
            p252.name_SET("vsTliAc", PH) ;
            p252.time_boot_ms = (uint)4138928157U;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 22);
                Debug.Assert(pack.text_TRY(ph).Equals("xmtdbzaGkldxyhcbqfxujm"));
                Debug.Assert(pack.severity == MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_CRITICAL;
            p253.text_SET("xmtdbzaGkldxyhcbqfxujm", PH) ;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float)3.127127E38F);
                Debug.Assert(pack.ind == (byte)(byte)140);
                Debug.Assert(pack.time_boot_ms == (uint)3436698718U);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)3436698718U;
            p254.value = (float)3.127127E38F;
            p254.ind = (byte)(byte)140;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)29);
                Debug.Assert(pack.target_system == (byte)(byte)226);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)103, (byte)252, (byte)90, (byte)71, (byte)148, (byte)38, (byte)97, (byte)28, (byte)183, (byte)19, (byte)78, (byte)189, (byte)131, (byte)166, (byte)44, (byte)240, (byte)120, (byte)18, (byte)4, (byte)200, (byte)34, (byte)118, (byte)39, (byte)75, (byte)235, (byte)120, (byte)56, (byte)186, (byte)102, (byte)4, (byte)11, (byte)232}));
                Debug.Assert(pack.initial_timestamp == (ulong)194265937775288866L);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.initial_timestamp = (ulong)194265937775288866L;
            p256.target_component = (byte)(byte)29;
            p256.target_system = (byte)(byte)226;
            p256.secret_key_SET(new byte[] {(byte)103, (byte)252, (byte)90, (byte)71, (byte)148, (byte)38, (byte)97, (byte)28, (byte)183, (byte)19, (byte)78, (byte)189, (byte)131, (byte)166, (byte)44, (byte)240, (byte)120, (byte)18, (byte)4, (byte)200, (byte)34, (byte)118, (byte)39, (byte)75, (byte)235, (byte)120, (byte)56, (byte)186, (byte)102, (byte)4, (byte)11, (byte)232}, 0) ;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_change_ms == (uint)4244699819U);
                Debug.Assert(pack.time_boot_ms == (uint)3818434308U);
                Debug.Assert(pack.state == (byte)(byte)77);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.state = (byte)(byte)77;
            p257.time_boot_ms = (uint)3818434308U;
            p257.last_change_ms = (uint)4244699819U;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tune_LEN(ph) == 26);
                Debug.Assert(pack.tune_TRY(ph).Equals("lgiwjrondzqgbtKsabsjhsoqsJ"));
                Debug.Assert(pack.target_system == (byte)(byte)25);
                Debug.Assert(pack.target_component == (byte)(byte)128);
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_component = (byte)(byte)128;
            p258.tune_SET("lgiwjrondzqgbtKsabsjhsoqsJ", PH) ;
            p258.target_system = (byte)(byte)25;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)170, (byte)114, (byte)77, (byte)179, (byte)193, (byte)170, (byte)194, (byte)212, (byte)185, (byte)72, (byte)160, (byte)42, (byte)246, (byte)73, (byte)71, (byte)124, (byte)142, (byte)58, (byte)75, (byte)63, (byte)177, (byte)188, (byte)51, (byte)194, (byte)2, (byte)194, (byte)51, (byte)60, (byte)231, (byte)93, (byte)101, (byte)115}));
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)36708);
                Debug.Assert(pack.sensor_size_v == (float)2.3244824E38F);
                Debug.Assert(pack.lens_id == (byte)(byte)194);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)38301);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 109);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("fuzefeorvaovfDgggiztiuMxwtpawzhsnevdcygjpascjvuwHvdfnifgtehddgjqefcgqqjmbmGxlypnqlvxEppubFeqthrlxcdkleJnegbmp"));
                Debug.Assert(pack.resolution_v == (ushort)(ushort)1932);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)124, (byte)175, (byte)192, (byte)116, (byte)55, (byte)111, (byte)10, (byte)212, (byte)108, (byte)7, (byte)66, (byte)68, (byte)142, (byte)95, (byte)57, (byte)120, (byte)79, (byte)208, (byte)210, (byte)179, (byte)44, (byte)15, (byte)139, (byte)183, (byte)51, (byte)73, (byte)218, (byte)0, (byte)178, (byte)56, (byte)96, (byte)236}));
                Debug.Assert(pack.focal_length == (float)2.5967628E38F);
                Debug.Assert(pack.sensor_size_h == (float)3.0980174E38F);
                Debug.Assert(pack.time_boot_ms == (uint)538944925U);
                Debug.Assert(pack.firmware_version == (uint)1793299107U);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE));
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.resolution_v = (ushort)(ushort)1932;
            p259.sensor_size_v = (float)2.3244824E38F;
            p259.sensor_size_h = (float)3.0980174E38F;
            p259.cam_definition_uri_SET("fuzefeorvaovfDgggiztiuMxwtpawzhsnevdcygjpascjvuwHvdfnifgtehddgjqefcgqqjmbmGxlypnqlvxEppubFeqthrlxcdkleJnegbmp", PH) ;
            p259.focal_length = (float)2.5967628E38F;
            p259.cam_definition_version = (ushort)(ushort)36708;
            p259.resolution_h = (ushort)(ushort)38301;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE);
            p259.model_name_SET(new byte[] {(byte)124, (byte)175, (byte)192, (byte)116, (byte)55, (byte)111, (byte)10, (byte)212, (byte)108, (byte)7, (byte)66, (byte)68, (byte)142, (byte)95, (byte)57, (byte)120, (byte)79, (byte)208, (byte)210, (byte)179, (byte)44, (byte)15, (byte)139, (byte)183, (byte)51, (byte)73, (byte)218, (byte)0, (byte)178, (byte)56, (byte)96, (byte)236}, 0) ;
            p259.firmware_version = (uint)1793299107U;
            p259.lens_id = (byte)(byte)194;
            p259.time_boot_ms = (uint)538944925U;
            p259.vendor_name_SET(new byte[] {(byte)170, (byte)114, (byte)77, (byte)179, (byte)193, (byte)170, (byte)194, (byte)212, (byte)185, (byte)72, (byte)160, (byte)42, (byte)246, (byte)73, (byte)71, (byte)124, (byte)142, (byte)58, (byte)75, (byte)63, (byte)177, (byte)188, (byte)51, (byte)194, (byte)2, (byte)194, (byte)51, (byte)60, (byte)231, (byte)93, (byte)101, (byte)115}, 0) ;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == (CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY |
                                              CAMERA_MODE.CAMERA_MODE_VIDEO));
                Debug.Assert(pack.time_boot_ms == (uint)1407984265U);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = (CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY |
                            CAMERA_MODE.CAMERA_MODE_VIDEO);
            p260.time_boot_ms = (uint)1407984265U;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.available_capacity == (float)3.5829318E37F);
                Debug.Assert(pack.read_speed == (float) -3.11274E36F);
                Debug.Assert(pack.status == (byte)(byte)27);
                Debug.Assert(pack.time_boot_ms == (uint)1513777808U);
                Debug.Assert(pack.used_capacity == (float)2.7189592E38F);
                Debug.Assert(pack.storage_id == (byte)(byte)101);
                Debug.Assert(pack.write_speed == (float)7.088777E37F);
                Debug.Assert(pack.storage_count == (byte)(byte)113);
                Debug.Assert(pack.total_capacity == (float) -1.7711728E38F);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.total_capacity = (float) -1.7711728E38F;
            p261.read_speed = (float) -3.11274E36F;
            p261.storage_count = (byte)(byte)113;
            p261.status = (byte)(byte)27;
            p261.used_capacity = (float)2.7189592E38F;
            p261.write_speed = (float)7.088777E37F;
            p261.time_boot_ms = (uint)1513777808U;
            p261.storage_id = (byte)(byte)101;
            p261.available_capacity = (float)3.5829318E37F;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.video_status == (byte)(byte)253);
                Debug.Assert(pack.image_interval == (float) -1.5043096E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3823650038U);
                Debug.Assert(pack.available_capacity == (float) -1.3047737E38F);
                Debug.Assert(pack.recording_time_ms == (uint)1426957378U);
                Debug.Assert(pack.image_status == (byte)(byte)216);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.image_status = (byte)(byte)216;
            p262.available_capacity = (float) -1.3047737E38F;
            p262.video_status = (byte)(byte)253;
            p262.time_boot_ms = (uint)3823650038U;
            p262.image_interval = (float) -1.5043096E38F;
            p262.recording_time_ms = (uint)1426957378U;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.camera_id == (byte)(byte)150);
                Debug.Assert(pack.alt == (int) -1695302220);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.687545E38F, 6.011848E37F, 2.4877456E38F, -2.970807E38F}));
                Debug.Assert(pack.relative_alt == (int)1991896298);
                Debug.Assert(pack.lat == (int) -957760842);
                Debug.Assert(pack.time_utc == (ulong)653112115831429222L);
                Debug.Assert(pack.lon == (int) -997914374);
                Debug.Assert(pack.image_index == (int) -1974121530);
                Debug.Assert(pack.time_boot_ms == (uint)375226055U);
                Debug.Assert(pack.file_url_LEN(ph) == 6);
                Debug.Assert(pack.file_url_TRY(ph).Equals("jdisxo"));
                Debug.Assert(pack.capture_result == (sbyte)(sbyte) - 111);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.capture_result = (sbyte)(sbyte) - 111;
            p263.alt = (int) -1695302220;
            p263.q_SET(new float[] {1.687545E38F, 6.011848E37F, 2.4877456E38F, -2.970807E38F}, 0) ;
            p263.relative_alt = (int)1991896298;
            p263.time_boot_ms = (uint)375226055U;
            p263.time_utc = (ulong)653112115831429222L;
            p263.camera_id = (byte)(byte)150;
            p263.image_index = (int) -1974121530;
            p263.lat = (int) -957760842;
            p263.file_url_SET("jdisxo", PH) ;
            p263.lon = (int) -997914374;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)838956824U);
                Debug.Assert(pack.flight_uuid == (ulong)8377178814491474559L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)1501656313451108001L);
                Debug.Assert(pack.arming_time_utc == (ulong)7986135402765881371L);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)838956824U;
            p264.arming_time_utc = (ulong)7986135402765881371L;
            p264.takeoff_time_utc = (ulong)1501656313451108001L;
            p264.flight_uuid = (ulong)8377178814491474559L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)2.5611858E38F);
                Debug.Assert(pack.pitch == (float)2.0031898E38F);
                Debug.Assert(pack.time_boot_ms == (uint)388079175U);
                Debug.Assert(pack.yaw == (float) -3.2418101E38F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)388079175U;
            p265.pitch = (float)2.0031898E38F;
            p265.roll = (float)2.5611858E38F;
            p265.yaw = (float) -3.2418101E38F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)187, (byte)182, (byte)178, (byte)157, (byte)111, (byte)96, (byte)88, (byte)32, (byte)182, (byte)47, (byte)143, (byte)154, (byte)208, (byte)154, (byte)114, (byte)135, (byte)115, (byte)179, (byte)138, (byte)243, (byte)180, (byte)191, (byte)111, (byte)4, (byte)210, (byte)83, (byte)7, (byte)23, (byte)150, (byte)212, (byte)36, (byte)249, (byte)41, (byte)94, (byte)136, (byte)185, (byte)44, (byte)25, (byte)18, (byte)239, (byte)109, (byte)30, (byte)20, (byte)72, (byte)199, (byte)28, (byte)248, (byte)28, (byte)13, (byte)25, (byte)109, (byte)89, (byte)147, (byte)100, (byte)231, (byte)49, (byte)153, (byte)172, (byte)196, (byte)204, (byte)58, (byte)21, (byte)21, (byte)201, (byte)141, (byte)26, (byte)32, (byte)137, (byte)151, (byte)0, (byte)223, (byte)229, (byte)184, (byte)252, (byte)12, (byte)106, (byte)194, (byte)134, (byte)39, (byte)126, (byte)254, (byte)134, (byte)105, (byte)0, (byte)162, (byte)103, (byte)93, (byte)13, (byte)209, (byte)112, (byte)55, (byte)4, (byte)129, (byte)244, (byte)59, (byte)138, (byte)85, (byte)168, (byte)134, (byte)32, (byte)96, (byte)70, (byte)154, (byte)119, (byte)163, (byte)20, (byte)196, (byte)217, (byte)102, (byte)16, (byte)53, (byte)100, (byte)73, (byte)116, (byte)46, (byte)34, (byte)174, (byte)153, (byte)17, (byte)97, (byte)56, (byte)248, (byte)223, (byte)38, (byte)67, (byte)168, (byte)228, (byte)51, (byte)215, (byte)89, (byte)165, (byte)234, (byte)196, (byte)23, (byte)196, (byte)146, (byte)178, (byte)253, (byte)34, (byte)133, (byte)54, (byte)184, (byte)181, (byte)92, (byte)213, (byte)165, (byte)130, (byte)129, (byte)254, (byte)100, (byte)57, (byte)138, (byte)87, (byte)178, (byte)87, (byte)188, (byte)139, (byte)203, (byte)135, (byte)211, (byte)40, (byte)242, (byte)174, (byte)80, (byte)156, (byte)237, (byte)52, (byte)99, (byte)76, (byte)136, (byte)222, (byte)167, (byte)176, (byte)39, (byte)97, (byte)172, (byte)223, (byte)193, (byte)161, (byte)112, (byte)190, (byte)188, (byte)58, (byte)71, (byte)193, (byte)182, (byte)55, (byte)35, (byte)240, (byte)159, (byte)40, (byte)230, (byte)102, (byte)2, (byte)104, (byte)38, (byte)137, (byte)200, (byte)141, (byte)206, (byte)133, (byte)72, (byte)118, (byte)4, (byte)199, (byte)29, (byte)177, (byte)214, (byte)2, (byte)174, (byte)214, (byte)55, (byte)139, (byte)26, (byte)87, (byte)136, (byte)130, (byte)82, (byte)111, (byte)137, (byte)215, (byte)237, (byte)38, (byte)195, (byte)170, (byte)111, (byte)35, (byte)51, (byte)124, (byte)5, (byte)5, (byte)56, (byte)12, (byte)31, (byte)153, (byte)161, (byte)114, (byte)61, (byte)133, (byte)238, (byte)108, (byte)96, (byte)6, (byte)124, (byte)242, (byte)10, (byte)168, (byte)194, (byte)159}));
                Debug.Assert(pack.target_component == (byte)(byte)153);
                Debug.Assert(pack.sequence == (ushort)(ushort)43298);
                Debug.Assert(pack.first_message_offset == (byte)(byte)240);
                Debug.Assert(pack.length == (byte)(byte)166);
                Debug.Assert(pack.target_system == (byte)(byte)176);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.sequence = (ushort)(ushort)43298;
            p266.first_message_offset = (byte)(byte)240;
            p266.target_component = (byte)(byte)153;
            p266.data__SET(new byte[] {(byte)187, (byte)182, (byte)178, (byte)157, (byte)111, (byte)96, (byte)88, (byte)32, (byte)182, (byte)47, (byte)143, (byte)154, (byte)208, (byte)154, (byte)114, (byte)135, (byte)115, (byte)179, (byte)138, (byte)243, (byte)180, (byte)191, (byte)111, (byte)4, (byte)210, (byte)83, (byte)7, (byte)23, (byte)150, (byte)212, (byte)36, (byte)249, (byte)41, (byte)94, (byte)136, (byte)185, (byte)44, (byte)25, (byte)18, (byte)239, (byte)109, (byte)30, (byte)20, (byte)72, (byte)199, (byte)28, (byte)248, (byte)28, (byte)13, (byte)25, (byte)109, (byte)89, (byte)147, (byte)100, (byte)231, (byte)49, (byte)153, (byte)172, (byte)196, (byte)204, (byte)58, (byte)21, (byte)21, (byte)201, (byte)141, (byte)26, (byte)32, (byte)137, (byte)151, (byte)0, (byte)223, (byte)229, (byte)184, (byte)252, (byte)12, (byte)106, (byte)194, (byte)134, (byte)39, (byte)126, (byte)254, (byte)134, (byte)105, (byte)0, (byte)162, (byte)103, (byte)93, (byte)13, (byte)209, (byte)112, (byte)55, (byte)4, (byte)129, (byte)244, (byte)59, (byte)138, (byte)85, (byte)168, (byte)134, (byte)32, (byte)96, (byte)70, (byte)154, (byte)119, (byte)163, (byte)20, (byte)196, (byte)217, (byte)102, (byte)16, (byte)53, (byte)100, (byte)73, (byte)116, (byte)46, (byte)34, (byte)174, (byte)153, (byte)17, (byte)97, (byte)56, (byte)248, (byte)223, (byte)38, (byte)67, (byte)168, (byte)228, (byte)51, (byte)215, (byte)89, (byte)165, (byte)234, (byte)196, (byte)23, (byte)196, (byte)146, (byte)178, (byte)253, (byte)34, (byte)133, (byte)54, (byte)184, (byte)181, (byte)92, (byte)213, (byte)165, (byte)130, (byte)129, (byte)254, (byte)100, (byte)57, (byte)138, (byte)87, (byte)178, (byte)87, (byte)188, (byte)139, (byte)203, (byte)135, (byte)211, (byte)40, (byte)242, (byte)174, (byte)80, (byte)156, (byte)237, (byte)52, (byte)99, (byte)76, (byte)136, (byte)222, (byte)167, (byte)176, (byte)39, (byte)97, (byte)172, (byte)223, (byte)193, (byte)161, (byte)112, (byte)190, (byte)188, (byte)58, (byte)71, (byte)193, (byte)182, (byte)55, (byte)35, (byte)240, (byte)159, (byte)40, (byte)230, (byte)102, (byte)2, (byte)104, (byte)38, (byte)137, (byte)200, (byte)141, (byte)206, (byte)133, (byte)72, (byte)118, (byte)4, (byte)199, (byte)29, (byte)177, (byte)214, (byte)2, (byte)174, (byte)214, (byte)55, (byte)139, (byte)26, (byte)87, (byte)136, (byte)130, (byte)82, (byte)111, (byte)137, (byte)215, (byte)237, (byte)38, (byte)195, (byte)170, (byte)111, (byte)35, (byte)51, (byte)124, (byte)5, (byte)5, (byte)56, (byte)12, (byte)31, (byte)153, (byte)161, (byte)114, (byte)61, (byte)133, (byte)238, (byte)108, (byte)96, (byte)6, (byte)124, (byte)242, (byte)10, (byte)168, (byte)194, (byte)159}, 0) ;
            p266.target_system = (byte)(byte)176;
            p266.length = (byte)(byte)166;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)230);
                Debug.Assert(pack.target_system == (byte)(byte)32);
                Debug.Assert(pack.length == (byte)(byte)0);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)252, (byte)13, (byte)230, (byte)170, (byte)18, (byte)46, (byte)153, (byte)172, (byte)216, (byte)141, (byte)240, (byte)109, (byte)251, (byte)254, (byte)4, (byte)178, (byte)136, (byte)143, (byte)133, (byte)18, (byte)45, (byte)15, (byte)190, (byte)162, (byte)34, (byte)94, (byte)67, (byte)9, (byte)86, (byte)98, (byte)194, (byte)215, (byte)85, (byte)130, (byte)33, (byte)49, (byte)93, (byte)232, (byte)200, (byte)83, (byte)147, (byte)3, (byte)68, (byte)208, (byte)147, (byte)248, (byte)251, (byte)47, (byte)63, (byte)172, (byte)23, (byte)122, (byte)108, (byte)106, (byte)13, (byte)234, (byte)239, (byte)67, (byte)33, (byte)234, (byte)5, (byte)72, (byte)194, (byte)234, (byte)107, (byte)3, (byte)70, (byte)154, (byte)158, (byte)158, (byte)195, (byte)70, (byte)152, (byte)175, (byte)78, (byte)2, (byte)61, (byte)227, (byte)219, (byte)255, (byte)6, (byte)202, (byte)57, (byte)164, (byte)239, (byte)225, (byte)136, (byte)182, (byte)165, (byte)23, (byte)36, (byte)207, (byte)78, (byte)248, (byte)110, (byte)75, (byte)206, (byte)93, (byte)61, (byte)196, (byte)55, (byte)6, (byte)5, (byte)224, (byte)141, (byte)75, (byte)209, (byte)146, (byte)61, (byte)109, (byte)131, (byte)199, (byte)9, (byte)51, (byte)235, (byte)179, (byte)158, (byte)45, (byte)229, (byte)213, (byte)89, (byte)143, (byte)134, (byte)193, (byte)143, (byte)153, (byte)200, (byte)34, (byte)255, (byte)214, (byte)232, (byte)118, (byte)218, (byte)133, (byte)241, (byte)230, (byte)33, (byte)97, (byte)193, (byte)232, (byte)148, (byte)123, (byte)157, (byte)36, (byte)62, (byte)4, (byte)230, (byte)42, (byte)99, (byte)253, (byte)32, (byte)3, (byte)76, (byte)216, (byte)143, (byte)82, (byte)100, (byte)3, (byte)5, (byte)221, (byte)57, (byte)124, (byte)78, (byte)166, (byte)114, (byte)189, (byte)29, (byte)130, (byte)194, (byte)93, (byte)251, (byte)216, (byte)203, (byte)108, (byte)43, (byte)144, (byte)97, (byte)48, (byte)64, (byte)27, (byte)190, (byte)208, (byte)126, (byte)11, (byte)111, (byte)93, (byte)108, (byte)231, (byte)12, (byte)146, (byte)164, (byte)228, (byte)21, (byte)79, (byte)47, (byte)46, (byte)202, (byte)209, (byte)253, (byte)158, (byte)154, (byte)121, (byte)187, (byte)107, (byte)231, (byte)213, (byte)76, (byte)133, (byte)118, (byte)12, (byte)128, (byte)136, (byte)125, (byte)104, (byte)80, (byte)244, (byte)89, (byte)241, (byte)152, (byte)198, (byte)120, (byte)140, (byte)104, (byte)233, (byte)134, (byte)108, (byte)77, (byte)27, (byte)172, (byte)143, (byte)13, (byte)178, (byte)81, (byte)7, (byte)255, (byte)120, (byte)159, (byte)154, (byte)204, (byte)169, (byte)66, (byte)33, (byte)56, (byte)145, (byte)138, (byte)142, (byte)63, (byte)252, (byte)135}));
                Debug.Assert(pack.first_message_offset == (byte)(byte)6);
                Debug.Assert(pack.sequence == (ushort)(ushort)56029);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.data__SET(new byte[] {(byte)252, (byte)13, (byte)230, (byte)170, (byte)18, (byte)46, (byte)153, (byte)172, (byte)216, (byte)141, (byte)240, (byte)109, (byte)251, (byte)254, (byte)4, (byte)178, (byte)136, (byte)143, (byte)133, (byte)18, (byte)45, (byte)15, (byte)190, (byte)162, (byte)34, (byte)94, (byte)67, (byte)9, (byte)86, (byte)98, (byte)194, (byte)215, (byte)85, (byte)130, (byte)33, (byte)49, (byte)93, (byte)232, (byte)200, (byte)83, (byte)147, (byte)3, (byte)68, (byte)208, (byte)147, (byte)248, (byte)251, (byte)47, (byte)63, (byte)172, (byte)23, (byte)122, (byte)108, (byte)106, (byte)13, (byte)234, (byte)239, (byte)67, (byte)33, (byte)234, (byte)5, (byte)72, (byte)194, (byte)234, (byte)107, (byte)3, (byte)70, (byte)154, (byte)158, (byte)158, (byte)195, (byte)70, (byte)152, (byte)175, (byte)78, (byte)2, (byte)61, (byte)227, (byte)219, (byte)255, (byte)6, (byte)202, (byte)57, (byte)164, (byte)239, (byte)225, (byte)136, (byte)182, (byte)165, (byte)23, (byte)36, (byte)207, (byte)78, (byte)248, (byte)110, (byte)75, (byte)206, (byte)93, (byte)61, (byte)196, (byte)55, (byte)6, (byte)5, (byte)224, (byte)141, (byte)75, (byte)209, (byte)146, (byte)61, (byte)109, (byte)131, (byte)199, (byte)9, (byte)51, (byte)235, (byte)179, (byte)158, (byte)45, (byte)229, (byte)213, (byte)89, (byte)143, (byte)134, (byte)193, (byte)143, (byte)153, (byte)200, (byte)34, (byte)255, (byte)214, (byte)232, (byte)118, (byte)218, (byte)133, (byte)241, (byte)230, (byte)33, (byte)97, (byte)193, (byte)232, (byte)148, (byte)123, (byte)157, (byte)36, (byte)62, (byte)4, (byte)230, (byte)42, (byte)99, (byte)253, (byte)32, (byte)3, (byte)76, (byte)216, (byte)143, (byte)82, (byte)100, (byte)3, (byte)5, (byte)221, (byte)57, (byte)124, (byte)78, (byte)166, (byte)114, (byte)189, (byte)29, (byte)130, (byte)194, (byte)93, (byte)251, (byte)216, (byte)203, (byte)108, (byte)43, (byte)144, (byte)97, (byte)48, (byte)64, (byte)27, (byte)190, (byte)208, (byte)126, (byte)11, (byte)111, (byte)93, (byte)108, (byte)231, (byte)12, (byte)146, (byte)164, (byte)228, (byte)21, (byte)79, (byte)47, (byte)46, (byte)202, (byte)209, (byte)253, (byte)158, (byte)154, (byte)121, (byte)187, (byte)107, (byte)231, (byte)213, (byte)76, (byte)133, (byte)118, (byte)12, (byte)128, (byte)136, (byte)125, (byte)104, (byte)80, (byte)244, (byte)89, (byte)241, (byte)152, (byte)198, (byte)120, (byte)140, (byte)104, (byte)233, (byte)134, (byte)108, (byte)77, (byte)27, (byte)172, (byte)143, (byte)13, (byte)178, (byte)81, (byte)7, (byte)255, (byte)120, (byte)159, (byte)154, (byte)204, (byte)169, (byte)66, (byte)33, (byte)56, (byte)145, (byte)138, (byte)142, (byte)63, (byte)252, (byte)135}, 0) ;
            p267.length = (byte)(byte)0;
            p267.target_system = (byte)(byte)32;
            p267.target_component = (byte)(byte)230;
            p267.first_message_offset = (byte)(byte)6;
            p267.sequence = (ushort)(ushort)56029;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)221);
                Debug.Assert(pack.sequence == (ushort)(ushort)8553);
                Debug.Assert(pack.target_system == (byte)(byte)41);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_component = (byte)(byte)221;
            p268.target_system = (byte)(byte)41;
            p268.sequence = (ushort)(ushort)8553;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_v == (ushort)(ushort)14195);
                Debug.Assert(pack.uri_LEN(ph) == 31);
                Debug.Assert(pack.uri_TRY(ph).Equals("bfwIpObclehcopkhjlfdmhxcoxCnfoi"));
                Debug.Assert(pack.status == (byte)(byte)60);
                Debug.Assert(pack.camera_id == (byte)(byte)66);
                Debug.Assert(pack.framerate == (float)3.2621946E38F);
                Debug.Assert(pack.bitrate == (uint)1852455860U);
                Debug.Assert(pack.rotation == (ushort)(ushort)35349);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)23885);
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.rotation = (ushort)(ushort)35349;
            p269.resolution_h = (ushort)(ushort)23885;
            p269.uri_SET("bfwIpObclehcopkhjlfdmhxcoxCnfoi", PH) ;
            p269.resolution_v = (ushort)(ushort)14195;
            p269.status = (byte)(byte)60;
            p269.bitrate = (uint)1852455860U;
            p269.framerate = (float)3.2621946E38F;
            p269.camera_id = (byte)(byte)66;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_h == (ushort)(ushort)49185);
                Debug.Assert(pack.target_component == (byte)(byte)107);
                Debug.Assert(pack.rotation == (ushort)(ushort)59051);
                Debug.Assert(pack.framerate == (float) -3.0579112E38F);
                Debug.Assert(pack.camera_id == (byte)(byte)128);
                Debug.Assert(pack.target_system == (byte)(byte)39);
                Debug.Assert(pack.uri_LEN(ph) == 146);
                Debug.Assert(pack.uri_TRY(ph).Equals("ParzhwKfvaiitgyldhglrjqiFqlrrUrRblAHGQdedYtrtxxumfysfwByrqlbAhltvcibcyxegptzkdsgkejitdQvmzntdyBhmtwvmpoxkwrmaizllqnyqdzcybfbxweaqsqmvbloqdzdjbnqgp"));
                Debug.Assert(pack.bitrate == (uint)746028655U);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)27125);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.camera_id = (byte)(byte)128;
            p270.bitrate = (uint)746028655U;
            p270.resolution_v = (ushort)(ushort)27125;
            p270.uri_SET("ParzhwKfvaiitgyldhglrjqiFqlrrUrRblAHGQdedYtrtxxumfysfwByrqlbAhltvcibcyxegptzkdsgkejitdQvmzntdyBhmtwvmpoxkwrmaizllqnyqdzcybfbxweaqsqmvbloqdzdjbnqgp", PH) ;
            p270.framerate = (float) -3.0579112E38F;
            p270.target_system = (byte)(byte)39;
            p270.resolution_h = (ushort)(ushort)49185;
            p270.rotation = (ushort)(ushort)59051;
            p270.target_component = (byte)(byte)107;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 40);
                Debug.Assert(pack.password_TRY(ph).Equals("jNIsncrifkmzynruyaxdhdhSsKckcXtqrhQbwzgm"));
                Debug.Assert(pack.ssid_LEN(ph) == 15);
                Debug.Assert(pack.ssid_TRY(ph).Equals("oqNvnmxnamfkysW"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("oqNvnmxnamfkysW", PH) ;
            p299.password_SET("jNIsncrifkmzynruyaxdhdhSsKckcXtqrhQbwzgm", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.max_version == (ushort)(ushort)14442);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)206, (byte)115, (byte)245, (byte)247, (byte)177, (byte)114, (byte)62, (byte)250}));
                Debug.Assert(pack.min_version == (ushort)(ushort)60756);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)132, (byte)176, (byte)151, (byte)193, (byte)143, (byte)143, (byte)31, (byte)38}));
                Debug.Assert(pack.version == (ushort)(ushort)30237);
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.min_version = (ushort)(ushort)60756;
            p300.max_version = (ushort)(ushort)14442;
            p300.spec_version_hash_SET(new byte[] {(byte)206, (byte)115, (byte)245, (byte)247, (byte)177, (byte)114, (byte)62, (byte)250}, 0) ;
            p300.version = (ushort)(ushort)30237;
            p300.library_version_hash_SET(new byte[] {(byte)132, (byte)176, (byte)151, (byte)193, (byte)143, (byte)143, (byte)31, (byte)38}, 0) ;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)4311349985298209626L);
                Debug.Assert(pack.sub_mode == (byte)(byte)115);
                Debug.Assert(pack.health == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
                Debug.Assert(pack.uptime_sec == (uint)864038175U);
                Debug.Assert(pack.mode == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)24879);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.time_usec = (ulong)4311349985298209626L;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE;
            p310.sub_mode = (byte)(byte)115;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK;
            p310.uptime_sec = (uint)864038175U;
            p310.vendor_specific_status_code = (ushort)(ushort)24879;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uptime_sec == (uint)834632105U);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)233, (byte)185, (byte)216, (byte)209, (byte)147, (byte)68, (byte)165, (byte)142, (byte)70, (byte)1, (byte)233, (byte)30, (byte)92, (byte)237, (byte)246, (byte)61}));
                Debug.Assert(pack.name_LEN(ph) == 29);
                Debug.Assert(pack.name_TRY(ph).Equals("ctkmgCmqavjdcywpnqjdvxkQnnnlt"));
                Debug.Assert(pack.sw_vcs_commit == (uint)606578510U);
                Debug.Assert(pack.hw_version_major == (byte)(byte)34);
                Debug.Assert(pack.sw_version_major == (byte)(byte)88);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)174);
                Debug.Assert(pack.time_usec == (ulong)2895162825060232503L);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)246);
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.hw_unique_id_SET(new byte[] {(byte)233, (byte)185, (byte)216, (byte)209, (byte)147, (byte)68, (byte)165, (byte)142, (byte)70, (byte)1, (byte)233, (byte)30, (byte)92, (byte)237, (byte)246, (byte)61}, 0) ;
            p311.hw_version_major = (byte)(byte)34;
            p311.sw_version_minor = (byte)(byte)246;
            p311.sw_vcs_commit = (uint)606578510U;
            p311.sw_version_major = (byte)(byte)88;
            p311.hw_version_minor = (byte)(byte)174;
            p311.name_SET("ctkmgCmqavjdcywpnqjdvxkQnnnlt", PH) ;
            p311.uptime_sec = (uint)834632105U;
            p311.time_usec = (ulong)2895162825060232503L;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)152);
                Debug.Assert(pack.target_system == (byte)(byte)91);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("adrku"));
                Debug.Assert(pack.param_index == (short)(short)2647);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_index = (short)(short)2647;
            p320.param_id_SET("adrku", PH) ;
            p320.target_component = (byte)(byte)152;
            p320.target_system = (byte)(byte)91;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)144);
                Debug.Assert(pack.target_component == (byte)(byte)58);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_component = (byte)(byte)58;
            p321.target_system = (byte)(byte)144;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (ushort)(ushort)23583);
                Debug.Assert(pack.param_count == (ushort)(ushort)55242);
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("nvsrzngj"));
                Debug.Assert(pack.param_value_LEN(ph) == 94);
                Debug.Assert(pack.param_value_TRY(ph).Equals("qhsrlkhtvJjgzwpyzhnklzRnwUcjxszzofdogizkrovgtrjaawwlujyhjoqmekutgvdqFkkzhsdrjkcdCykHhajEqkpbfn"));
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_id_SET("nvsrzngj", PH) ;
            p322.param_index = (ushort)(ushort)23583;
            p322.param_value_SET("qhsrlkhtvJjgzwpyzhnklzRnwUcjxszzofdogizkrovgtrjaawwlujyhjoqmekutgvdqFkkzhsdrjkcdCykHhajEqkpbfn", PH) ;
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8;
            p322.param_count = (ushort)(ushort)55242;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_LEN(ph) == 82);
                Debug.Assert(pack.param_value_TRY(ph).Equals("IAdqluyoRfvdzlsunddnmozcCkzrrmdlxovhjhgartehitwUyipOQfdtevppxLiqhaJtFhhVepboIRftia"));
                Debug.Assert(pack.target_system == (byte)(byte)69);
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("txzextht"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
                Debug.Assert(pack.target_component == (byte)(byte)9);
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_value_SET("IAdqluyoRfvdzlsunddnmozcCkzrrmdlxovhjhgartehitwUyipOQfdtevppxLiqhaJtFhhVepboIRftia", PH) ;
            p323.target_component = (byte)(byte)9;
            p323.param_id_SET("txzextht", PH) ;
            p323.target_system = (byte)(byte)69;
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_result == PARAM_ACK.PARAM_ACK_FAILED);
                Debug.Assert(pack.param_value_LEN(ph) == 73);
                Debug.Assert(pack.param_value_TRY(ph).Equals("axxuehhncgfmitqbaWymouvgbgfetbdlYnojtdqBqagvbvurnyUcayPbzbqzsbgqrSlKevgyg"));
                Debug.Assert(pack.param_id_LEN(ph) == 13);
                Debug.Assert(pack.param_id_TRY(ph).Equals("yyUzcvPmyqzmV"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_value_SET("axxuehhncgfmitqbaWymouvgbgfetbdlYnojtdqBqagvbvurnyUcayPbzbqzsbgqrSlKevgyg", PH) ;
            p324.param_id_SET("yyUzcvPmyqzmV", PH) ;
            p324.param_result = PARAM_ACK.PARAM_ACK_FAILED;
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.increment == (byte)(byte)122);
                Debug.Assert(pack.min_distance == (ushort)(ushort)47739);
                Debug.Assert(pack.max_distance == (ushort)(ushort)38811);
                Debug.Assert(pack.sensor_type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)19277, (ushort)8397, (ushort)19336, (ushort)32436, (ushort)53402, (ushort)35574, (ushort)44768, (ushort)27679, (ushort)20264, (ushort)21020, (ushort)12508, (ushort)64434, (ushort)37076, (ushort)39928, (ushort)62703, (ushort)44345, (ushort)40416, (ushort)63338, (ushort)46096, (ushort)4805, (ushort)47192, (ushort)32402, (ushort)56918, (ushort)12105, (ushort)31828, (ushort)23283, (ushort)38145, (ushort)49520, (ushort)46777, (ushort)61879, (ushort)26719, (ushort)6255, (ushort)39012, (ushort)62599, (ushort)22779, (ushort)62580, (ushort)59270, (ushort)28437, (ushort)29541, (ushort)28069, (ushort)10417, (ushort)41834, (ushort)8332, (ushort)15497, (ushort)29325, (ushort)49910, (ushort)53390, (ushort)60415, (ushort)49186, (ushort)17281, (ushort)54196, (ushort)15190, (ushort)28086, (ushort)23840, (ushort)31907, (ushort)20525, (ushort)64938, (ushort)29740, (ushort)60446, (ushort)48539, (ushort)26772, (ushort)25047, (ushort)59643, (ushort)9871, (ushort)57555, (ushort)23100, (ushort)27022, (ushort)40831, (ushort)49697, (ushort)58078, (ushort)53108, (ushort)49998}));
                Debug.Assert(pack.time_usec == (ulong)1932955139629954977L);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p330.max_distance = (ushort)(ushort)38811;
            p330.increment = (byte)(byte)122;
            p330.distances_SET(new ushort[] {(ushort)19277, (ushort)8397, (ushort)19336, (ushort)32436, (ushort)53402, (ushort)35574, (ushort)44768, (ushort)27679, (ushort)20264, (ushort)21020, (ushort)12508, (ushort)64434, (ushort)37076, (ushort)39928, (ushort)62703, (ushort)44345, (ushort)40416, (ushort)63338, (ushort)46096, (ushort)4805, (ushort)47192, (ushort)32402, (ushort)56918, (ushort)12105, (ushort)31828, (ushort)23283, (ushort)38145, (ushort)49520, (ushort)46777, (ushort)61879, (ushort)26719, (ushort)6255, (ushort)39012, (ushort)62599, (ushort)22779, (ushort)62580, (ushort)59270, (ushort)28437, (ushort)29541, (ushort)28069, (ushort)10417, (ushort)41834, (ushort)8332, (ushort)15497, (ushort)29325, (ushort)49910, (ushort)53390, (ushort)60415, (ushort)49186, (ushort)17281, (ushort)54196, (ushort)15190, (ushort)28086, (ushort)23840, (ushort)31907, (ushort)20525, (ushort)64938, (ushort)29740, (ushort)60446, (ushort)48539, (ushort)26772, (ushort)25047, (ushort)59643, (ushort)9871, (ushort)57555, (ushort)23100, (ushort)27022, (ushort)40831, (ushort)49697, (ushort)58078, (ushort)53108, (ushort)49998}, 0) ;
            p330.min_distance = (ushort)(ushort)47739;
            p330.time_usec = (ulong)1932955139629954977L;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
        }
    }
}