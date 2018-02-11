
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
                    ulong id = id__j(value);
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
            *	 should allow to identify the commit using the main version number even for very large code bases*/
            public byte[] flight_custom_version
            {
                get {return flight_custom_version_GET(new byte[8], 0);}
            }
            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	 should allow to identify the commit using the main version number even for very large code bases*/
            public byte[]flight_custom_version_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 28, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	 should allow to identify the commit using the main version number even for very large code bases*/
            public byte[] middleware_custom_version
            {
                get {return middleware_custom_version_GET(new byte[8], 0);}
            }
            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	 should allow to identify the commit using the main version number even for very large code bases*/
            public byte[]middleware_custom_version_GET(byte[] dst_ch, int pos)
            {
                for(int BYTE = 36, dst_max = pos + 8; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	 should allow to identify the commit using the main version number even for very large code bases*/
            public byte[] os_custom_version
            {
                get {return os_custom_version_GET(new byte[8], 0);}
            }
            /**
            *Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but
            *	 should allow to identify the commit using the main version number even for very large code bases*/
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
            *	 use uid*/
            public byte[] uid2_TRY(Inside ph)
            {
                if(ph.field_bit !=  433 && !try_visit_field(ph, 433)) return null;
                return uid2_GET(ph, new byte[ph.items], 0);
            }
            /**
            *UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise
            *	 use uid*/
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
*	 the landing targe*/
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
                Debug.Assert(pack.system_status == MAV_STATE.MAV_STATE_UNINIT);
                Debug.Assert(pack.type == MAV_TYPE.MAV_TYPE_VTOL_RESERVED2);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
                Debug.Assert(pack.custom_mode == (uint)1028423512U);
                Debug.Assert(pack.autopilot == MAV_AUTOPILOT.MAV_AUTOPILOT_AUTOQUAD);
                Debug.Assert(pack.mavlink_version == (byte)(byte)44);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.type = MAV_TYPE.MAV_TYPE_VTOL_RESERVED2;
            p0.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_AUTOQUAD;
            p0.mavlink_version = (byte)(byte)44;
            p0.custom_mode = (uint)1028423512U;
            p0.system_status = MAV_STATE.MAV_STATE_UNINIT;
            p0.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
            ADV_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)14915);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)58830);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)24);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)50552);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)11824);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW));
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)23071);
                Debug.Assert(pack.current_battery == (short)(short)21056);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)11710);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING));
                Debug.Assert(pack.load == (ushort)(ushort)45467);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)18535);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW);
            p1.voltage_battery = (ushort)(ushort)11824;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL);
            p1.errors_count4 = (ushort)(ushort)14915;
            p1.battery_remaining = (sbyte)(sbyte)24;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING);
            p1.errors_comm = (ushort)(ushort)18535;
            p1.load = (ushort)(ushort)45467;
            p1.current_battery = (short)(short)21056;
            p1.errors_count3 = (ushort)(ushort)11710;
            p1.drop_rate_comm = (ushort)(ushort)50552;
            p1.errors_count2 = (ushort)(ushort)23071;
            p1.errors_count1 = (ushort)(ushort)58830;
            ADV_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3873232573U);
                Debug.Assert(pack.time_unix_usec == (ulong)4301519206930240212L);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)4301519206930240212L;
            p2.time_boot_ms = (uint)3873232573U;
            ADV_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -1.4758781E38F);
                Debug.Assert(pack.z == (float)1.1037004E38F);
                Debug.Assert(pack.y == (float) -1.797929E38F);
                Debug.Assert(pack.afz == (float)8.4616545E37F);
                Debug.Assert(pack.time_boot_ms == (uint)867905860U);
                Debug.Assert(pack.afy == (float)2.8753944E38F);
                Debug.Assert(pack.yaw_rate == (float) -1.7794249E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.type_mask == (ushort)(ushort)45489);
                Debug.Assert(pack.vy == (float) -5.6254135E37F);
                Debug.Assert(pack.vz == (float) -1.2745892E38F);
                Debug.Assert(pack.vx == (float) -1.3527786E38F);
                Debug.Assert(pack.afx == (float)3.1840227E38F);
                Debug.Assert(pack.x == (float)2.8756745E38F);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p3.afz = (float)8.4616545E37F;
            p3.yaw = (float) -1.4758781E38F;
            p3.afy = (float)2.8753944E38F;
            p3.y = (float) -1.797929E38F;
            p3.vy = (float) -5.6254135E37F;
            p3.vz = (float) -1.2745892E38F;
            p3.vx = (float) -1.3527786E38F;
            p3.type_mask = (ushort)(ushort)45489;
            p3.yaw_rate = (float) -1.7794249E38F;
            p3.time_boot_ms = (uint)867905860U;
            p3.afx = (float)3.1840227E38F;
            p3.x = (float)2.8756745E38F;
            p3.z = (float)1.1037004E38F;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)132);
                Debug.Assert(pack.time_usec == (ulong)1428469047103837734L);
                Debug.Assert(pack.target_system == (byte)(byte)8);
                Debug.Assert(pack.seq == (uint)2916177504U);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.seq = (uint)2916177504U;
            p4.target_system = (byte)(byte)8;
            p4.time_usec = (ulong)1428469047103837734L;
            p4.target_component = (byte)(byte)132;
            ADV_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)156);
                Debug.Assert(pack.passkey_LEN(ph) == 21);
                Debug.Assert(pack.passkey_TRY(ph).Equals("ndqIuKnwaVnepbhcpcsji"));
                Debug.Assert(pack.version == (byte)(byte)216);
                Debug.Assert(pack.control_request == (byte)(byte)179);
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.target_system = (byte)(byte)156;
            p5.version = (byte)(byte)216;
            p5.passkey_SET("ndqIuKnwaVnepbhcpcsji", PH) ;
            p5.control_request = (byte)(byte)179;
            ADV_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ack == (byte)(byte)250);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)246);
                Debug.Assert(pack.control_request == (byte)(byte)96);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.gcs_system_id = (byte)(byte)246;
            p6.ack = (byte)(byte)250;
            p6.control_request = (byte)(byte)96;
            ADV_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 29);
                Debug.Assert(pack.key_TRY(ph).Equals("kpytuceRjhdrxUnhdbxpaymxpgbqk"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("kpytuceRjhdrxUnhdbxpaymxpgbqk", PH) ;
            ADV_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.base_mode == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
                Debug.Assert(pack.custom_mode == (uint)799210977U);
                Debug.Assert(pack.target_system == (byte)(byte)205);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)205;
            p11.base_mode = MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            p11.custom_mode = (uint)799210977U;
            ADV_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)9);
                Debug.Assert(pack.param_index == (short)(short)6319);
                Debug.Assert(pack.param_id_LEN(ph) == 9);
                Debug.Assert(pack.param_id_TRY(ph).Equals("blgdMxawd"));
                Debug.Assert(pack.target_system == (byte)(byte)75);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_component = (byte)(byte)9;
            p20.param_index = (short)(short)6319;
            p20.target_system = (byte)(byte)75;
            p20.param_id_SET("blgdMxawd", PH) ;
            ADV_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)118);
                Debug.Assert(pack.target_system == (byte)(byte)205);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)205;
            p21.target_component = (byte)(byte)118;
            ADV_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
                Debug.Assert(pack.param_id_LEN(ph) == 10);
                Debug.Assert(pack.param_id_TRY(ph).Equals("heHYiacqwk"));
                Debug.Assert(pack.param_count == (ushort)(ushort)17085);
                Debug.Assert(pack.param_index == (ushort)(ushort)38133);
                Debug.Assert(pack.param_value == (float)2.4595868E38F);
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_id_SET("heHYiacqwk", PH) ;
            p22.param_count = (ushort)(ushort)17085;
            p22.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32;
            p22.param_value = (float)2.4595868E38F;
            p22.param_index = (ushort)(ushort)38133;
            ADV_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value == (float) -2.6964796E38F);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
                Debug.Assert(pack.target_system == (byte)(byte)102);
                Debug.Assert(pack.target_component == (byte)(byte)170);
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("svs"));
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.target_system = (byte)(byte)102;
            p23.target_component = (byte)(byte)170;
            p23.param_id_SET("svs", PH) ;
            p23.param_value = (float) -2.6964796E38F;
            p23.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8;
            ADV_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.epv == (ushort)(ushort)33562);
                Debug.Assert(pack.lon == (int) -1073499253);
                Debug.Assert(pack.lat == (int)2133882438);
                Debug.Assert(pack.cog == (ushort)(ushort)60726);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)836921315U);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)1788900482U);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)2190636408U);
                Debug.Assert(pack.satellites_visible == (byte)(byte)196);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -1108955576);
                Debug.Assert(pack.vel == (ushort)(ushort)8834);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)1265659801U);
                Debug.Assert(pack.eph == (ushort)(ushort)25162);
                Debug.Assert(pack.alt == (int)1726484834);
                Debug.Assert(pack.time_usec == (ulong)1355239579471578517L);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX;
            p24.lon = (int) -1073499253;
            p24.eph = (ushort)(ushort)25162;
            p24.alt = (int)1726484834;
            p24.lat = (int)2133882438;
            p24.vel = (ushort)(ushort)8834;
            p24.hdg_acc_SET((uint)836921315U, PH) ;
            p24.cog = (ushort)(ushort)60726;
            p24.vel_acc_SET((uint)1265659801U, PH) ;
            p24.satellites_visible = (byte)(byte)196;
            p24.time_usec = (ulong)1355239579471578517L;
            p24.h_acc_SET((uint)2190636408U, PH) ;
            p24.epv = (ushort)(ushort)33562;
            p24.v_acc_SET((uint)1788900482U, PH) ;
            p24.alt_ellipsoid_SET((int) -1108955576, PH) ;
            ADV_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)33, (byte)183, (byte)134, (byte)71, (byte)105, (byte)250, (byte)212, (byte)167, (byte)126, (byte)244, (byte)214, (byte)115, (byte)35, (byte)55, (byte)30, (byte)197, (byte)76, (byte)95, (byte)48, (byte)29}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)0, (byte)53, (byte)44, (byte)65, (byte)202, (byte)224, (byte)236, (byte)92, (byte)64, (byte)133, (byte)234, (byte)30, (byte)250, (byte)214, (byte)129, (byte)31, (byte)214, (byte)14, (byte)143, (byte)99}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)112, (byte)58, (byte)206, (byte)171, (byte)80, (byte)193, (byte)142, (byte)25, (byte)115, (byte)75, (byte)179, (byte)5, (byte)217, (byte)202, (byte)166, (byte)118, (byte)156, (byte)143, (byte)254, (byte)66}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)252);
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)155, (byte)31, (byte)187, (byte)141, (byte)32, (byte)203, (byte)129, (byte)39, (byte)7, (byte)68, (byte)138, (byte)104, (byte)159, (byte)163, (byte)187, (byte)233, (byte)16, (byte)191, (byte)29, (byte)161}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)8, (byte)226, (byte)13, (byte)84, (byte)195, (byte)114, (byte)200, (byte)100, (byte)222, (byte)253, (byte)80, (byte)170, (byte)15, (byte)30, (byte)65, (byte)148, (byte)105, (byte)58, (byte)180, (byte)7}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_elevation_SET(new byte[] {(byte)155, (byte)31, (byte)187, (byte)141, (byte)32, (byte)203, (byte)129, (byte)39, (byte)7, (byte)68, (byte)138, (byte)104, (byte)159, (byte)163, (byte)187, (byte)233, (byte)16, (byte)191, (byte)29, (byte)161}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)112, (byte)58, (byte)206, (byte)171, (byte)80, (byte)193, (byte)142, (byte)25, (byte)115, (byte)75, (byte)179, (byte)5, (byte)217, (byte)202, (byte)166, (byte)118, (byte)156, (byte)143, (byte)254, (byte)66}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)8, (byte)226, (byte)13, (byte)84, (byte)195, (byte)114, (byte)200, (byte)100, (byte)222, (byte)253, (byte)80, (byte)170, (byte)15, (byte)30, (byte)65, (byte)148, (byte)105, (byte)58, (byte)180, (byte)7}, 0) ;
            p25.satellites_visible = (byte)(byte)252;
            p25.satellite_azimuth_SET(new byte[] {(byte)0, (byte)53, (byte)44, (byte)65, (byte)202, (byte)224, (byte)236, (byte)92, (byte)64, (byte)133, (byte)234, (byte)30, (byte)250, (byte)214, (byte)129, (byte)31, (byte)214, (byte)14, (byte)143, (byte)99}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)33, (byte)183, (byte)134, (byte)71, (byte)105, (byte)250, (byte)212, (byte)167, (byte)126, (byte)244, (byte)214, (byte)115, (byte)35, (byte)55, (byte)30, (byte)197, (byte)76, (byte)95, (byte)48, (byte)29}, 0) ;
            ADV_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short) -5600);
                Debug.Assert(pack.xmag == (short)(short) -28447);
                Debug.Assert(pack.zgyro == (short)(short) -14790);
                Debug.Assert(pack.time_boot_ms == (uint)3639537090U);
                Debug.Assert(pack.zacc == (short)(short)30234);
                Debug.Assert(pack.xacc == (short)(short)20089);
                Debug.Assert(pack.xgyro == (short)(short)10710);
                Debug.Assert(pack.ymag == (short)(short) -22681);
                Debug.Assert(pack.ygyro == (short)(short) -25830);
                Debug.Assert(pack.zmag == (short)(short)13593);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.xgyro = (short)(short)10710;
            p26.zmag = (short)(short)13593;
            p26.zgyro = (short)(short) -14790;
            p26.yacc = (short)(short) -5600;
            p26.time_boot_ms = (uint)3639537090U;
            p26.zacc = (short)(short)30234;
            p26.ygyro = (short)(short) -25830;
            p26.xacc = (short)(short)20089;
            p26.xmag = (short)(short) -28447;
            p26.ymag = (short)(short) -22681;
            ADV_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (short)(short) -16294);
                Debug.Assert(pack.yacc == (short)(short)23067);
                Debug.Assert(pack.xmag == (short)(short) -20227);
                Debug.Assert(pack.time_usec == (ulong)4839046161658273776L);
                Debug.Assert(pack.zgyro == (short)(short)29230);
                Debug.Assert(pack.zmag == (short)(short)29925);
                Debug.Assert(pack.ygyro == (short)(short) -17937);
                Debug.Assert(pack.xacc == (short)(short) -9524);
                Debug.Assert(pack.zacc == (short)(short)15442);
                Debug.Assert(pack.ymag == (short)(short) -9231);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.ygyro = (short)(short) -17937;
            p27.xgyro = (short)(short) -16294;
            p27.zgyro = (short)(short)29230;
            p27.time_usec = (ulong)4839046161658273776L;
            p27.yacc = (short)(short)23067;
            p27.xmag = (short)(short) -20227;
            p27.zacc = (short)(short)15442;
            p27.xacc = (short)(short) -9524;
            p27.zmag = (short)(short)29925;
            p27.ymag = (short)(short) -9231;
            ADV_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff2 == (short)(short)2553);
                Debug.Assert(pack.time_usec == (ulong)3697218297511672142L);
                Debug.Assert(pack.temperature == (short)(short) -13728);
                Debug.Assert(pack.press_diff1 == (short)(short) -15412);
                Debug.Assert(pack.press_abs == (short)(short)22521);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_diff2 = (short)(short)2553;
            p28.press_abs = (short)(short)22521;
            p28.press_diff1 = (short)(short) -15412;
            p28.temperature = (short)(short) -13728;
            p28.time_usec = (ulong)3697218297511672142L;
            ADV_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -4144);
                Debug.Assert(pack.press_abs == (float)2.991923E38F);
                Debug.Assert(pack.press_diff == (float)1.844176E38F);
                Debug.Assert(pack.time_boot_ms == (uint)41140042U);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.temperature = (short)(short) -4144;
            p29.press_diff = (float)1.844176E38F;
            p29.press_abs = (float)2.991923E38F;
            p29.time_boot_ms = (uint)41140042U;
            ADV_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yawspeed == (float)1.9705E38F);
                Debug.Assert(pack.rollspeed == (float)9.39247E37F);
                Debug.Assert(pack.time_boot_ms == (uint)2213872764U);
                Debug.Assert(pack.roll == (float) -2.8559975E38F);
                Debug.Assert(pack.yaw == (float) -3.2260503E38F);
                Debug.Assert(pack.pitch == (float) -8.0337296E37F);
                Debug.Assert(pack.pitchspeed == (float) -3.116302E38F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.pitchspeed = (float) -3.116302E38F;
            p30.time_boot_ms = (uint)2213872764U;
            p30.yawspeed = (float)1.9705E38F;
            p30.roll = (float) -2.8559975E38F;
            p30.rollspeed = (float)9.39247E37F;
            p30.pitch = (float) -8.0337296E37F;
            p30.yaw = (float) -3.2260503E38F;
            ADV_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float) -1.0074414E38F);
                Debug.Assert(pack.pitchspeed == (float) -2.8389226E38F);
                Debug.Assert(pack.q2 == (float) -1.0310232E38F);
                Debug.Assert(pack.q1 == (float) -9.628459E37F);
                Debug.Assert(pack.q4 == (float)1.2029347E38F);
                Debug.Assert(pack.q3 == (float)1.3383934E38F);
                Debug.Assert(pack.yawspeed == (float)2.2943512E38F);
                Debug.Assert(pack.time_boot_ms == (uint)78600322U);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.rollspeed = (float) -1.0074414E38F;
            p31.q3 = (float)1.3383934E38F;
            p31.q2 = (float) -1.0310232E38F;
            p31.q4 = (float)1.2029347E38F;
            p31.q1 = (float) -9.628459E37F;
            p31.yawspeed = (float)2.2943512E38F;
            p31.pitchspeed = (float) -2.8389226E38F;
            p31.time_boot_ms = (uint)78600322U;
            ADV_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float)2.0045142E38F);
                Debug.Assert(pack.x == (float) -7.5352666E37F);
                Debug.Assert(pack.z == (float) -1.0881621E38F);
                Debug.Assert(pack.vy == (float) -4.6869976E37F);
                Debug.Assert(pack.time_boot_ms == (uint)2722599485U);
                Debug.Assert(pack.vz == (float) -2.2787218E38F);
                Debug.Assert(pack.y == (float) -1.9491767E38F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.time_boot_ms = (uint)2722599485U;
            p32.vz = (float) -2.2787218E38F;
            p32.x = (float) -7.5352666E37F;
            p32.vy = (float) -4.6869976E37F;
            p32.z = (float) -1.0881621E38F;
            p32.vx = (float)2.0045142E38F;
            p32.y = (float) -1.9491767E38F;
            ADV_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -456794310);
                Debug.Assert(pack.hdg == (ushort)(ushort)6449);
                Debug.Assert(pack.vy == (short)(short) -19613);
                Debug.Assert(pack.time_boot_ms == (uint)801412740U);
                Debug.Assert(pack.lon == (int) -393957501);
                Debug.Assert(pack.alt == (int)1189665941);
                Debug.Assert(pack.relative_alt == (int)1865650290);
                Debug.Assert(pack.vx == (short)(short)12311);
                Debug.Assert(pack.vz == (short)(short) -29561);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.alt = (int)1189665941;
            p33.time_boot_ms = (uint)801412740U;
            p33.vz = (short)(short) -29561;
            p33.lon = (int) -393957501;
            p33.hdg = (ushort)(ushort)6449;
            p33.vx = (short)(short)12311;
            p33.relative_alt = (int)1865650290;
            p33.lat = (int) -456794310;
            p33.vy = (short)(short) -19613;
            ADV_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan6_scaled == (short)(short) -31321);
                Debug.Assert(pack.chan7_scaled == (short)(short)32238);
                Debug.Assert(pack.chan3_scaled == (short)(short)931);
                Debug.Assert(pack.chan1_scaled == (short)(short)1387);
                Debug.Assert(pack.rssi == (byte)(byte)114);
                Debug.Assert(pack.time_boot_ms == (uint)3487399779U);
                Debug.Assert(pack.chan8_scaled == (short)(short)21953);
                Debug.Assert(pack.port == (byte)(byte)190);
                Debug.Assert(pack.chan4_scaled == (short)(short)31175);
                Debug.Assert(pack.chan2_scaled == (short)(short)32123);
                Debug.Assert(pack.chan5_scaled == (short)(short)19668);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan2_scaled = (short)(short)32123;
            p34.chan8_scaled = (short)(short)21953;
            p34.chan4_scaled = (short)(short)31175;
            p34.chan7_scaled = (short)(short)32238;
            p34.chan1_scaled = (short)(short)1387;
            p34.port = (byte)(byte)190;
            p34.chan3_scaled = (short)(short)931;
            p34.rssi = (byte)(byte)114;
            p34.chan5_scaled = (short)(short)19668;
            p34.time_boot_ms = (uint)3487399779U;
            p34.chan6_scaled = (short)(short) -31321;
            ADV_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)25067);
                Debug.Assert(pack.port == (byte)(byte)252);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)31797);
                Debug.Assert(pack.rssi == (byte)(byte)14);
                Debug.Assert(pack.time_boot_ms == (uint)1763159087U);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)60252);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)40546);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)8937);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)6839);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)24126);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)55705);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan2_raw = (ushort)(ushort)8937;
            p35.port = (byte)(byte)252;
            p35.chan6_raw = (ushort)(ushort)55705;
            p35.chan3_raw = (ushort)(ushort)40546;
            p35.chan7_raw = (ushort)(ushort)31797;
            p35.chan5_raw = (ushort)(ushort)6839;
            p35.chan1_raw = (ushort)(ushort)60252;
            p35.chan8_raw = (ushort)(ushort)25067;
            p35.rssi = (byte)(byte)14;
            p35.time_boot_ms = (uint)1763159087U;
            p35.chan4_raw = (ushort)(ushort)24126;
            ADV_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)11352);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)1567);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)56744);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)26965);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)58694);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)57622);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)29136);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)17216);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)49257);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)24157);
                Debug.Assert(pack.time_usec == (uint)1042787131U);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)802);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)12298);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)54573);
                Debug.Assert(pack.port == (byte)(byte)185);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)37391);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)50458);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)32994);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo7_raw = (ushort)(ushort)54573;
            p36.servo2_raw = (ushort)(ushort)802;
            p36.servo5_raw = (ushort)(ushort)50458;
            p36.servo3_raw = (ushort)(ushort)24157;
            p36.servo14_raw_SET((ushort)(ushort)1567, PH) ;
            p36.servo11_raw_SET((ushort)(ushort)32994, PH) ;
            p36.servo16_raw_SET((ushort)(ushort)37391, PH) ;
            p36.servo9_raw_SET((ushort)(ushort)11352, PH) ;
            p36.servo4_raw = (ushort)(ushort)58694;
            p36.servo13_raw_SET((ushort)(ushort)49257, PH) ;
            p36.servo1_raw = (ushort)(ushort)57622;
            p36.servo10_raw_SET((ushort)(ushort)56744, PH) ;
            p36.servo12_raw_SET((ushort)(ushort)29136, PH) ;
            p36.servo15_raw_SET((ushort)(ushort)26965, PH) ;
            p36.time_usec = (uint)1042787131U;
            p36.servo8_raw = (ushort)(ushort)12298;
            p36.port = (byte)(byte)185;
            p36.servo6_raw = (ushort)(ushort)17216;
            ADV_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_index == (short)(short)4694);
                Debug.Assert(pack.target_component == (byte)(byte)227);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_system == (byte)(byte)186);
                Debug.Assert(pack.end_index == (short)(short) -24915);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.target_component = (byte)(byte)227;
            p37.start_index = (short)(short)4694;
            p37.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p37.end_index = (short)(short) -24915;
            p37.target_system = (byte)(byte)186;
            ADV_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)16);
                Debug.Assert(pack.start_index == (short)(short)14093);
                Debug.Assert(pack.end_index == (short)(short) -21702);
                Debug.Assert(pack.target_component == (byte)(byte)34);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.end_index = (short)(short) -21702;
            p38.target_system = (byte)(byte)16;
            p38.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p38.target_component = (byte)(byte)34;
            p38.start_index = (short)(short)14093;
            ADV_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param3 == (float)6.006742E36F);
                Debug.Assert(pack.current == (byte)(byte)226);
                Debug.Assert(pack.target_system == (byte)(byte)60);
                Debug.Assert(pack.target_component == (byte)(byte)183);
                Debug.Assert(pack.autocontinue == (byte)(byte)27);
                Debug.Assert(pack.x == (float)2.6748594E38F);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.seq == (ushort)(ushort)36253);
                Debug.Assert(pack.param1 == (float) -2.3534122E38F);
                Debug.Assert(pack.z == (float) -1.5700559E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.y == (float) -2.6866956E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_STORAGE_FORMAT);
                Debug.Assert(pack.param2 == (float)2.6757238E38F);
                Debug.Assert(pack.param4 == (float)3.2890974E38F);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.y = (float) -2.6866956E38F;
            p39.frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p39.param4 = (float)3.2890974E38F;
            p39.seq = (ushort)(ushort)36253;
            p39.current = (byte)(byte)226;
            p39.target_system = (byte)(byte)60;
            p39.param1 = (float) -2.3534122E38F;
            p39.command = MAV_CMD.MAV_CMD_STORAGE_FORMAT;
            p39.param2 = (float)2.6757238E38F;
            p39.target_component = (byte)(byte)183;
            p39.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p39.x = (float)2.6748594E38F;
            p39.param3 = (float)6.006742E36F;
            p39.z = (float) -1.5700559E38F;
            p39.autocontinue = (byte)(byte)27;
            ADV_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)47423);
                Debug.Assert(pack.target_component == (byte)(byte)65);
                Debug.Assert(pack.target_system == (byte)(byte)85);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p40.target_component = (byte)(byte)65;
            p40.seq = (ushort)(ushort)47423;
            p40.target_system = (byte)(byte)85;
            ADV_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)35);
                Debug.Assert(pack.seq == (ushort)(ushort)42407);
                Debug.Assert(pack.target_component == (byte)(byte)136);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_system = (byte)(byte)35;
            p41.target_component = (byte)(byte)136;
            p41.seq = (ushort)(ushort)42407;
            ADV_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)65318);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)65318;
            ADV_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)161);
                Debug.Assert(pack.target_system == (byte)(byte)16);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p43.target_system = (byte)(byte)16;
            p43.target_component = (byte)(byte)161;
            ADV_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)101);
                Debug.Assert(pack.target_system == (byte)(byte)183);
                Debug.Assert(pack.count == (ushort)(ushort)64303);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p44.target_component = (byte)(byte)101;
            p44.target_system = (byte)(byte)183;
            p44.count = (ushort)(ushort)64303;
            ADV_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)116);
                Debug.Assert(pack.target_system == (byte)(byte)172);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p45.target_component = (byte)(byte)116;
            p45.target_system = (byte)(byte)172;
            ADV_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)21699);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)21699;
            ADV_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)135);
                Debug.Assert(pack.target_system == (byte)(byte)47);
                Debug.Assert(pack.type == MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.target_component = (byte)(byte)135;
            p47.type = MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED;
            p47.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p47.target_system = (byte)(byte)47;
            ADV_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int)1163890347);
                Debug.Assert(pack.longitude == (int)2136574393);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)2140525830647779775L);
                Debug.Assert(pack.altitude == (int)1740301523);
                Debug.Assert(pack.target_system == (byte)(byte)72);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.latitude = (int)1163890347;
            p48.time_usec_SET((ulong)2140525830647779775L, PH) ;
            p48.target_system = (byte)(byte)72;
            p48.longitude = (int)2136574393;
            p48.altitude = (int)1740301523;
            ADV_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int) -2015193164);
                Debug.Assert(pack.altitude == (int) -840778513);
                Debug.Assert(pack.longitude == (int)503303260);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)2339161450696741410L);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.time_usec_SET((ulong)2339161450696741410L, PH) ;
            p49.latitude = (int) -2015193164;
            p49.altitude = (int) -840778513;
            p49.longitude = (int)503303260;
            ADV_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)29);
                Debug.Assert(pack.param_value_min == (float)2.9447828E38F);
                Debug.Assert(pack.param_index == (short)(short)8779);
                Debug.Assert(pack.param_value_max == (float)1.8313282E38F);
                Debug.Assert(pack.target_component == (byte)(byte)23);
                Debug.Assert(pack.scale == (float) -2.9510803E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 16);
                Debug.Assert(pack.param_id_TRY(ph).Equals("tzscmqSmUrmuoerw"));
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)62);
                Debug.Assert(pack.param_value0 == (float)2.7969607E38F);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.target_system = (byte)(byte)29;
            p50.param_index = (short)(short)8779;
            p50.target_component = (byte)(byte)23;
            p50.param_id_SET("tzscmqSmUrmuoerw", PH) ;
            p50.param_value_min = (float)2.9447828E38F;
            p50.scale = (float) -2.9510803E38F;
            p50.param_value0 = (float)2.7969607E38F;
            p50.parameter_rc_channel_index = (byte)(byte)62;
            p50.param_value_max = (float)1.8313282E38F;
            ADV_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)253);
                Debug.Assert(pack.seq == (ushort)(ushort)37670);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_component == (byte)(byte)181);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.seq = (ushort)(ushort)37670;
            p51.target_component = (byte)(byte)181;
            p51.target_system = (byte)(byte)253;
            p51.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            ADV_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)250);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.p1y == (float)1.0073265E38F);
                Debug.Assert(pack.p2z == (float) -2.5303531E38F);
                Debug.Assert(pack.target_system == (byte)(byte)247);
                Debug.Assert(pack.p1x == (float)2.6538271E38F);
                Debug.Assert(pack.p2x == (float) -1.731851E38F);
                Debug.Assert(pack.p2y == (float)3.4798365E37F);
                Debug.Assert(pack.p1z == (float) -1.5948115E38F);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p1y = (float)1.0073265E38F;
            p54.p2y = (float)3.4798365E37F;
            p54.target_component = (byte)(byte)250;
            p54.frame = MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p54.p2z = (float) -2.5303531E38F;
            p54.p2x = (float) -1.731851E38F;
            p54.target_system = (byte)(byte)247;
            p54.p1x = (float)2.6538271E38F;
            p54.p1z = (float) -1.5948115E38F;
            ADV_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.p2z == (float) -2.7343144E38F);
                Debug.Assert(pack.p1x == (float) -3.2419671E38F);
                Debug.Assert(pack.p1z == (float)1.9244948E38F);
                Debug.Assert(pack.p1y == (float) -1.5233122E38F);
                Debug.Assert(pack.p2x == (float)2.2124835E38F);
                Debug.Assert(pack.p2y == (float) -1.6597094E38F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p2z = (float) -2.7343144E38F;
            p55.p1x = (float) -3.2419671E38F;
            p55.p1y = (float) -1.5233122E38F;
            p55.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p55.p2x = (float)2.2124835E38F;
            p55.p1z = (float)1.9244948E38F;
            p55.p2y = (float) -1.6597094E38F;
            ADV_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.3717556E38F, -1.1138492E38F, -6.0349483E37F, -2.5081053E38F}));
                Debug.Assert(pack.yawspeed == (float) -1.144226E38F);
                Debug.Assert(pack.time_usec == (ulong)1695374328107588599L);
                Debug.Assert(pack.pitchspeed == (float)1.6045367E38F);
                Debug.Assert(pack.rollspeed == (float) -3.677395E37F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.5808959E38F, -3.0772133E38F, -3.233992E38F, -2.4956738E38F, 1.8423908E38F, -6.6529863E37F, 1.0523636E37F, -3.2787872E38F, 9.274244E37F}));
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.rollspeed = (float) -3.677395E37F;
            p61.time_usec = (ulong)1695374328107588599L;
            p61.pitchspeed = (float)1.6045367E38F;
            p61.q_SET(new float[] {2.3717556E38F, -1.1138492E38F, -6.0349483E37F, -2.5081053E38F}, 0) ;
            p61.yawspeed = (float) -1.144226E38F;
            p61.covariance_SET(new float[] {-2.5808959E38F, -3.0772133E38F, -3.233992E38F, -2.4956738E38F, 1.8423908E38F, -6.6529863E37F, 1.0523636E37F, -3.2787872E38F, 9.274244E37F}, 0) ;
            ADV_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xtrack_error == (float)3.3727909E38F);
                Debug.Assert(pack.nav_roll == (float) -1.4968617E38F);
                Debug.Assert(pack.aspd_error == (float) -5.6564836E37F);
                Debug.Assert(pack.alt_error == (float)5.1825567E37F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)1004);
                Debug.Assert(pack.nav_pitch == (float) -8.870532E37F);
                Debug.Assert(pack.nav_bearing == (short)(short)1922);
                Debug.Assert(pack.target_bearing == (short)(short)30309);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.target_bearing = (short)(short)30309;
            p62.nav_roll = (float) -1.4968617E38F;
            p62.wp_dist = (ushort)(ushort)1004;
            p62.alt_error = (float)5.1825567E37F;
            p62.aspd_error = (float) -5.6564836E37F;
            p62.nav_pitch = (float) -8.870532E37F;
            p62.xtrack_error = (float)3.3727909E38F;
            p62.nav_bearing = (short)(short)1922;
            ADV_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float) -2.571387E38F);
                Debug.Assert(pack.vx == (float) -3.290803E38F);
                Debug.Assert(pack.lon == (int)953564311);
                Debug.Assert(pack.time_usec == (ulong)2115247793876142347L);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
                Debug.Assert(pack.relative_alt == (int) -682828917);
                Debug.Assert(pack.vz == (float) -5.333926E37F);
                Debug.Assert(pack.lat == (int)908590193);
                Debug.Assert(pack.alt == (int)1514836829);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-6.235259E37F, 9.984445E37F, 2.4077028E36F, 1.2563274E38F, 5.207899E37F, 2.8904691E38F, 3.3384749E38F, -1.8106993E38F, 1.3664344E38F, 3.2008356E38F, -2.8704396E38F, -3.3611514E38F, -2.2857203E38F, 7.1738497E37F, -4.9968394E36F, -7.029022E37F, 1.4932513E38F, -1.342033E38F, -2.2299637E38F, 1.6131607E38F, 2.4980615E38F, 1.538145E38F, -1.523583E38F, -8.04382E37F, -2.7892234E38F, 1.4436966E37F, 2.778427E38F, -1.6914572E38F, -5.5009267E37F, 1.1849779E38F, -1.8597686E38F, -3.2135036E38F, -1.420591E38F, 2.2238756E38F, 5.1015245E37F, -3.3835564E37F}));
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.alt = (int)1514836829;
            p63.vz = (float) -5.333926E37F;
            p63.time_usec = (ulong)2115247793876142347L;
            p63.vy = (float) -2.571387E38F;
            p63.covariance_SET(new float[] {-6.235259E37F, 9.984445E37F, 2.4077028E36F, 1.2563274E38F, 5.207899E37F, 2.8904691E38F, 3.3384749E38F, -1.8106993E38F, 1.3664344E38F, 3.2008356E38F, -2.8704396E38F, -3.3611514E38F, -2.2857203E38F, 7.1738497E37F, -4.9968394E36F, -7.029022E37F, 1.4932513E38F, -1.342033E38F, -2.2299637E38F, 1.6131607E38F, 2.4980615E38F, 1.538145E38F, -1.523583E38F, -8.04382E37F, -2.7892234E38F, 1.4436966E37F, 2.778427E38F, -1.6914572E38F, -5.5009267E37F, 1.1849779E38F, -1.8597686E38F, -3.2135036E38F, -1.420591E38F, 2.2238756E38F, 5.1015245E37F, -3.3835564E37F}, 0) ;
            p63.relative_alt = (int) -682828917;
            p63.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS;
            p63.lon = (int)953564311;
            p63.lat = (int)908590193;
            p63.vx = (float) -3.290803E38F;
            ADV_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ax == (float)2.5288538E38F);
                Debug.Assert(pack.vz == (float) -2.0382319E38F);
                Debug.Assert(pack.z == (float)3.9656744E36F);
                Debug.Assert(pack.vx == (float)1.838728E37F);
                Debug.Assert(pack.x == (float)3.0963915E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-8.496631E37F, 2.880843E38F, -2.8475278E38F, -4.0333818E37F, 2.9284733E38F, -3.2404367E37F, 2.704922E38F, -3.4004502E38F, -3.2234803E38F, -3.1266962E37F, 8.927752E36F, 1.2824159E38F, -1.6972076E38F, 1.847751E38F, 2.110931E38F, -1.9199075E38F, -3.4561523E37F, 2.8578485E38F, -3.1386169E38F, 3.1644794E38F, -1.4674257E38F, 7.7023363E37F, -2.4971871E38F, 3.1690389E38F, 3.228968E38F, 1.0014446E38F, -1.01326E38F, -2.8130599E38F, -3.1254435E38F, -3.0602242E38F, -8.080057E37F, -6.2237873E36F, 1.2513158E38F, 2.293472E38F, 7.913142E36F, -3.2302997E38F, -2.6990494E38F, -2.3846668E38F, -1.1437529E38F, -1.072603E37F, 2.5991403E38F, -5.0254582E36F, 2.645195E38F, -1.6405257E37F, 3.0645593E38F}));
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
                Debug.Assert(pack.vy == (float) -8.507613E37F);
                Debug.Assert(pack.ay == (float) -1.1570052E37F);
                Debug.Assert(pack.az == (float)5.872389E37F);
                Debug.Assert(pack.time_usec == (ulong)1743076843000177344L);
                Debug.Assert(pack.y == (float) -8.956339E37F);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.vy = (float) -8.507613E37F;
            p64.az = (float)5.872389E37F;
            p64.covariance_SET(new float[] {-8.496631E37F, 2.880843E38F, -2.8475278E38F, -4.0333818E37F, 2.9284733E38F, -3.2404367E37F, 2.704922E38F, -3.4004502E38F, -3.2234803E38F, -3.1266962E37F, 8.927752E36F, 1.2824159E38F, -1.6972076E38F, 1.847751E38F, 2.110931E38F, -1.9199075E38F, -3.4561523E37F, 2.8578485E38F, -3.1386169E38F, 3.1644794E38F, -1.4674257E38F, 7.7023363E37F, -2.4971871E38F, 3.1690389E38F, 3.228968E38F, 1.0014446E38F, -1.01326E38F, -2.8130599E38F, -3.1254435E38F, -3.0602242E38F, -8.080057E37F, -6.2237873E36F, 1.2513158E38F, 2.293472E38F, 7.913142E36F, -3.2302997E38F, -2.6990494E38F, -2.3846668E38F, -1.1437529E38F, -1.072603E37F, 2.5991403E38F, -5.0254582E36F, 2.645195E38F, -1.6405257E37F, 3.0645593E38F}, 0) ;
            p64.vz = (float) -2.0382319E38F;
            p64.z = (float)3.9656744E36F;
            p64.ay = (float) -1.1570052E37F;
            p64.y = (float) -8.956339E37F;
            p64.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
            p64.vx = (float)1.838728E37F;
            p64.ax = (float)2.5288538E38F;
            p64.x = (float)3.0963915E38F;
            p64.time_usec = (ulong)1743076843000177344L;
            ADV_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)41472);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)54838);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)52668);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)43773);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)38836);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)46224);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)36616);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)23305);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)10203);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)33333);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)27367);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)208);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)18974);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)49880);
                Debug.Assert(pack.time_boot_ms == (uint)271807870U);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)29032);
                Debug.Assert(pack.rssi == (byte)(byte)165);
                Debug.Assert(pack.chancount == (byte)(byte)73);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)18931);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)43955);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)25134);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.time_boot_ms = (uint)271807870U;
            p65.chan14_raw = (ushort)(ushort)10203;
            p65.chan3_raw = (ushort)(ushort)41472;
            p65.chan12_raw = (ushort)(ushort)49880;
            p65.chan13_raw = (ushort)(ushort)36616;
            p65.chan10_raw = (ushort)(ushort)25134;
            p65.chancount = (byte)(byte)73;
            p65.chan9_raw = (ushort)(ushort)52668;
            p65.chan8_raw = (ushort)(ushort)54838;
            p65.chan17_raw = (ushort)(ushort)18974;
            p65.rssi = (byte)(byte)165;
            p65.chan18_raw = (ushort)(ushort)38836;
            p65.chan1_raw = (ushort)(ushort)33333;
            p65.chan11_raw = (ushort)(ushort)46224;
            p65.chan16_raw = (ushort)(ushort)18931;
            p65.chan15_raw = (ushort)(ushort)208;
            p65.chan7_raw = (ushort)(ushort)29032;
            p65.chan4_raw = (ushort)(ushort)23305;
            p65.chan2_raw = (ushort)(ushort)43773;
            p65.chan6_raw = (ushort)(ushort)27367;
            p65.chan5_raw = (ushort)(ushort)43955;
            ADV_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_stop == (byte)(byte)242);
                Debug.Assert(pack.target_component == (byte)(byte)151);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)59493);
                Debug.Assert(pack.req_stream_id == (byte)(byte)179);
                Debug.Assert(pack.target_system == (byte)(byte)203);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_system = (byte)(byte)203;
            p66.start_stop = (byte)(byte)242;
            p66.target_component = (byte)(byte)151;
            p66.req_message_rate = (ushort)(ushort)59493;
            p66.req_stream_id = (byte)(byte)179;
            ADV_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.on_off == (byte)(byte)39);
                Debug.Assert(pack.message_rate == (ushort)(ushort)56767);
                Debug.Assert(pack.stream_id == (byte)(byte)251);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.on_off = (byte)(byte)39;
            p67.stream_id = (byte)(byte)251;
            p67.message_rate = (ushort)(ushort)56767;
            ADV_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target == (byte)(byte)160);
                Debug.Assert(pack.z == (short)(short)17330);
                Debug.Assert(pack.r == (short)(short) -5307);
                Debug.Assert(pack.buttons == (ushort)(ushort)6618);
                Debug.Assert(pack.y == (short)(short)10893);
                Debug.Assert(pack.x == (short)(short)379);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.z = (short)(short)17330;
            p69.buttons = (ushort)(ushort)6618;
            p69.x = (short)(short)379;
            p69.r = (short)(short) -5307;
            p69.target = (byte)(byte)160;
            p69.y = (short)(short)10893;
            ADV_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)27562);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)48199);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)25254);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)12594);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)24111);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)17334);
                Debug.Assert(pack.target_component == (byte)(byte)145);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)19150);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)555);
                Debug.Assert(pack.target_system == (byte)(byte)249);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan5_raw = (ushort)(ushort)27562;
            p70.chan2_raw = (ushort)(ushort)19150;
            p70.chan4_raw = (ushort)(ushort)24111;
            p70.target_component = (byte)(byte)145;
            p70.chan3_raw = (ushort)(ushort)555;
            p70.chan7_raw = (ushort)(ushort)25254;
            p70.chan8_raw = (ushort)(ushort)17334;
            p70.chan6_raw = (ushort)(ushort)48199;
            p70.chan1_raw = (ushort)(ushort)12594;
            p70.target_system = (byte)(byte)249;
            ADV_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param2 == (float)3.6336974E37F);
                Debug.Assert(pack.target_system == (byte)(byte)206);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.seq == (ushort)(ushort)29634);
                Debug.Assert(pack.autocontinue == (byte)(byte)242);
                Debug.Assert(pack.target_component == (byte)(byte)179);
                Debug.Assert(pack.param4 == (float)8.716636E37F);
                Debug.Assert(pack.param1 == (float)3.3235586E38F);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_LAND);
                Debug.Assert(pack.z == (float) -2.5095227E38F);
                Debug.Assert(pack.y == (int) -1279917563);
                Debug.Assert(pack.current == (byte)(byte)111);
                Debug.Assert(pack.param3 == (float)1.4879792E38F);
                Debug.Assert(pack.x == (int) -11410217);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.param1 = (float)3.3235586E38F;
            p73.y = (int) -1279917563;
            p73.param2 = (float)3.6336974E37F;
            p73.current = (byte)(byte)111;
            p73.param4 = (float)8.716636E37F;
            p73.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p73.target_system = (byte)(byte)206;
            p73.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p73.command = MAV_CMD.MAV_CMD_NAV_LAND;
            p73.autocontinue = (byte)(byte)242;
            p73.seq = (ushort)(ushort)29634;
            p73.x = (int) -11410217;
            p73.z = (float) -2.5095227E38F;
            p73.param3 = (float)1.4879792E38F;
            p73.target_component = (byte)(byte)179;
            ADV_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.heading == (short)(short) -15189);
                Debug.Assert(pack.throttle == (ushort)(ushort)2389);
                Debug.Assert(pack.groundspeed == (float) -1.230803E38F);
                Debug.Assert(pack.airspeed == (float)1.8374504E38F);
                Debug.Assert(pack.climb == (float) -1.4271152E37F);
                Debug.Assert(pack.alt == (float)3.0360984E38F);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.alt = (float)3.0360984E38F;
            p74.groundspeed = (float) -1.230803E38F;
            p74.airspeed = (float)1.8374504E38F;
            p74.heading = (short)(short) -15189;
            p74.climb = (float) -1.4271152E37F;
            p74.throttle = (ushort)(ushort)2389;
            ADV_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (int)1713022681);
                Debug.Assert(pack.current == (byte)(byte)27);
                Debug.Assert(pack.autocontinue == (byte)(byte)0);
                Debug.Assert(pack.param3 == (float)2.0994318E37F);
                Debug.Assert(pack.z == (float)4.131478E37F);
                Debug.Assert(pack.param2 == (float)9.355556E37F);
                Debug.Assert(pack.param4 == (float)1.2052431E36F);
                Debug.Assert(pack.param1 == (float)1.412381E38F);
                Debug.Assert(pack.target_component == (byte)(byte)233);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL);
                Debug.Assert(pack.target_system == (byte)(byte)216);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.x == (int)1056824103);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.z = (float)4.131478E37F;
            p75.param2 = (float)9.355556E37F;
            p75.autocontinue = (byte)(byte)0;
            p75.param3 = (float)2.0994318E37F;
            p75.x = (int)1056824103;
            p75.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p75.param4 = (float)1.2052431E36F;
            p75.target_component = (byte)(byte)233;
            p75.target_system = (byte)(byte)216;
            p75.param1 = (float)1.412381E38F;
            p75.current = (byte)(byte)27;
            p75.y = (int)1713022681;
            p75.command = MAV_CMD.MAV_CMD_DO_TRIGGER_CONTROL;
            ADV_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param7 == (float) -3.1740572E38F);
                Debug.Assert(pack.target_component == (byte)(byte)249);
                Debug.Assert(pack.param4 == (float) -1.5508133E38F);
                Debug.Assert(pack.param2 == (float) -2.0509542E38F);
                Debug.Assert(pack.confirmation == (byte)(byte)96);
                Debug.Assert(pack.param5 == (float) -2.1751592E38F);
                Debug.Assert(pack.param6 == (float) -2.443593E38F);
                Debug.Assert(pack.param3 == (float)3.1379867E38F);
                Debug.Assert(pack.target_system == (byte)(byte)218);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL);
                Debug.Assert(pack.param1 == (float)1.206782E38F);
            };
            COMMAND_LONG p76 = new COMMAND_LONG();
            PH.setPack(p76);
            p76.param1 = (float)1.206782E38F;
            p76.param3 = (float)3.1379867E38F;
            p76.param6 = (float) -2.443593E38F;
            p76.param5 = (float) -2.1751592E38F;
            p76.target_system = (byte)(byte)218;
            p76.command = MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL;
            p76.param2 = (float) -2.0509542E38F;
            p76.target_component = (byte)(byte)249;
            p76.param4 = (float) -1.5508133E38F;
            p76.param7 = (float) -3.1740572E38F;
            p76.confirmation = (byte)(byte)96;
            ADV_TEST_CH.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result_param2_TRY(ph) == (int)81581810);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)136);
                Debug.Assert(pack.result == MAV_RESULT.MAV_RESULT_FAILED);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)123);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)186);
            };
            COMMAND_ACK p77 = new COMMAND_ACK();
            PH.setPack(p77);
            p77.result = MAV_RESULT.MAV_RESULT_FAILED;
            p77.command = MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION;
            p77.progress_SET((byte)(byte)136, PH) ;
            p77.result_param2_SET((int)81581810, PH) ;
            p77.target_system_SET((byte)(byte)123, PH) ;
            p77.target_component_SET((byte)(byte)186, PH) ;
            ADV_TEST_CH.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)3.1256112E38F);
                Debug.Assert(pack.thrust == (float)5.411302E37F);
                Debug.Assert(pack.yaw == (float) -3.2881337E38F);
                Debug.Assert(pack.pitch == (float)1.0635389E38F);
                Debug.Assert(pack.mode_switch == (byte)(byte)41);
                Debug.Assert(pack.time_boot_ms == (uint)2123973681U);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)61);
            };
            MANUAL_SETPOINT p81 = new MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.yaw = (float) -3.2881337E38F;
            p81.roll = (float)3.1256112E38F;
            p81.mode_switch = (byte)(byte)41;
            p81.thrust = (float)5.411302E37F;
            p81.time_boot_ms = (uint)2123973681U;
            p81.pitch = (float)1.0635389E38F;
            p81.manual_override_switch = (byte)(byte)61;
            ADV_TEST_CH.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)175);
                Debug.Assert(pack.body_pitch_rate == (float)1.2108674E38F);
                Debug.Assert(pack.target_system == (byte)(byte)150);
                Debug.Assert(pack.thrust == (float)1.9760525E38F);
                Debug.Assert(pack.body_yaw_rate == (float)3.1134574E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.9014816E38F, 2.587489E38F, 2.9758984E38F, -4.331553E37F}));
                Debug.Assert(pack.type_mask == (byte)(byte)249);
                Debug.Assert(pack.body_roll_rate == (float) -1.5152309E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1659239102U);
            };
            GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.thrust = (float)1.9760525E38F;
            p82.target_system = (byte)(byte)150;
            p82.target_component = (byte)(byte)175;
            p82.body_yaw_rate = (float)3.1134574E38F;
            p82.time_boot_ms = (uint)1659239102U;
            p82.body_roll_rate = (float) -1.5152309E38F;
            p82.body_pitch_rate = (float)1.2108674E38F;
            p82.type_mask = (byte)(byte)249;
            p82.q_SET(new float[] {2.9014816E38F, 2.587489E38F, 2.9758984E38F, -4.331553E37F}, 0) ;
            CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (byte)(byte)152);
                Debug.Assert(pack.body_roll_rate == (float)4.142428E37F);
                Debug.Assert(pack.body_yaw_rate == (float)3.7521094E37F);
                Debug.Assert(pack.body_pitch_rate == (float) -1.5960708E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1144009027U);
                Debug.Assert(pack.thrust == (float)1.2327414E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.9193394E38F, -1.5518778E36F, -1.6179045E38F, -1.8513443E38F}));
            };
            GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.body_pitch_rate = (float) -1.5960708E38F;
            p83.body_roll_rate = (float)4.142428E37F;
            p83.thrust = (float)1.2327414E38F;
            p83.body_yaw_rate = (float)3.7521094E37F;
            p83.time_boot_ms = (uint)1144009027U;
            p83.type_mask = (byte)(byte)152;
            p83.q_SET(new float[] {-1.9193394E38F, -1.5518778E36F, -1.6179045E38F, -1.8513443E38F}, 0) ;
            CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afz == (float) -1.8599613E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.time_boot_ms == (uint)3925197538U);
                Debug.Assert(pack.vz == (float)2.7428807E38F);
                Debug.Assert(pack.z == (float) -1.7738115E38F);
                Debug.Assert(pack.target_system == (byte)(byte)41);
                Debug.Assert(pack.yaw_rate == (float)5.3667104E37F);
                Debug.Assert(pack.yaw == (float)6.341603E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)22091);
                Debug.Assert(pack.vy == (float)2.892472E38F);
                Debug.Assert(pack.y == (float)2.8579023E38F);
                Debug.Assert(pack.x == (float) -2.1318025E38F);
                Debug.Assert(pack.target_component == (byte)(byte)148);
                Debug.Assert(pack.afy == (float) -2.7853212E38F);
                Debug.Assert(pack.vx == (float) -3.0907324E38F);
                Debug.Assert(pack.afx == (float)2.9945455E38F);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.y = (float)2.8579023E38F;
            p84.z = (float) -1.7738115E38F;
            p84.afy = (float) -2.7853212E38F;
            p84.target_system = (byte)(byte)41;
            p84.type_mask = (ushort)(ushort)22091;
            p84.yaw_rate = (float)5.3667104E37F;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p84.vz = (float)2.7428807E38F;
            p84.x = (float) -2.1318025E38F;
            p84.vx = (float) -3.0907324E38F;
            p84.afz = (float) -1.8599613E38F;
            p84.afx = (float)2.9945455E38F;
            p84.vy = (float)2.892472E38F;
            p84.yaw = (float)6.341603E37F;
            p84.target_component = (byte)(byte)148;
            p84.time_boot_ms = (uint)3925197538U;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)19);
                Debug.Assert(pack.time_boot_ms == (uint)2587932232U);
                Debug.Assert(pack.afy == (float) -1.01709E38F);
                Debug.Assert(pack.alt == (float)2.4497918E38F);
                Debug.Assert(pack.yaw_rate == (float) -2.7539273E37F);
                Debug.Assert(pack.lon_int == (int)1184385893);
                Debug.Assert(pack.afz == (float) -1.801349E38F);
                Debug.Assert(pack.yaw == (float) -1.5347412E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)47358);
                Debug.Assert(pack.target_component == (byte)(byte)28);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.vx == (float) -1.0768023E38F);
                Debug.Assert(pack.vz == (float)2.3121037E37F);
                Debug.Assert(pack.lat_int == (int)742152685);
                Debug.Assert(pack.vy == (float)2.6328336E38F);
                Debug.Assert(pack.afx == (float)3.5691894E37F);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.afy = (float) -1.01709E38F;
            p86.afz = (float) -1.801349E38F;
            p86.lat_int = (int)742152685;
            p86.type_mask = (ushort)(ushort)47358;
            p86.time_boot_ms = (uint)2587932232U;
            p86.lon_int = (int)1184385893;
            p86.yaw_rate = (float) -2.7539273E37F;
            p86.target_component = (byte)(byte)28;
            p86.vz = (float)2.3121037E37F;
            p86.target_system = (byte)(byte)19;
            p86.afx = (float)3.5691894E37F;
            p86.yaw = (float) -1.5347412E38F;
            p86.alt = (float)2.4497918E38F;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p86.vy = (float)2.6328336E38F;
            p86.vx = (float) -1.0768023E38F;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (ushort)(ushort)35860);
                Debug.Assert(pack.afy == (float)2.6957452E38F);
                Debug.Assert(pack.lon_int == (int)1580185596);
                Debug.Assert(pack.afz == (float)1.7084616E38F);
                Debug.Assert(pack.yaw == (float)3.1577347E38F);
                Debug.Assert(pack.vx == (float) -2.1455717E38F);
                Debug.Assert(pack.yaw_rate == (float) -2.0539215E36F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.afx == (float) -7.7817536E37F);
                Debug.Assert(pack.vz == (float) -1.5724747E38F);
                Debug.Assert(pack.lat_int == (int) -1060640712);
                Debug.Assert(pack.vy == (float) -9.666829E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3150778609U);
                Debug.Assert(pack.alt == (float)2.9765718E38F);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.time_boot_ms = (uint)3150778609U;
            p87.lon_int = (int)1580185596;
            p87.yaw_rate = (float) -2.0539215E36F;
            p87.vy = (float) -9.666829E37F;
            p87.alt = (float)2.9765718E38F;
            p87.afx = (float) -7.7817536E37F;
            p87.type_mask = (ushort)(ushort)35860;
            p87.vx = (float) -2.1455717E38F;
            p87.yaw = (float)3.1577347E38F;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p87.vz = (float) -1.5724747E38F;
            p87.lat_int = (int) -1060640712;
            p87.afz = (float)1.7084616E38F;
            p87.afy = (float)2.6957452E38F;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)2.5655788E38F);
                Debug.Assert(pack.roll == (float)3.358258E38F);
                Debug.Assert(pack.x == (float)1.5112844E38F);
                Debug.Assert(pack.pitch == (float)2.7914315E38F);
                Debug.Assert(pack.y == (float)1.5128541E38F);
                Debug.Assert(pack.yaw == (float)2.2453702E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3360944612U);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.time_boot_ms = (uint)3360944612U;
            p89.pitch = (float)2.7914315E38F;
            p89.y = (float)1.5128541E38F;
            p89.yaw = (float)2.2453702E38F;
            p89.z = (float)2.5655788E38F;
            p89.roll = (float)3.358258E38F;
            p89.x = (float)1.5112844E38F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)5750120024386861722L);
                Debug.Assert(pack.yacc == (short)(short) -17088);
                Debug.Assert(pack.rollspeed == (float) -2.7756437E38F);
                Debug.Assert(pack.vy == (short)(short) -3367);
                Debug.Assert(pack.vz == (short)(short) -28907);
                Debug.Assert(pack.xacc == (short)(short) -18347);
                Debug.Assert(pack.yaw == (float) -3.2863302E38F);
                Debug.Assert(pack.lon == (int) -1732038902);
                Debug.Assert(pack.lat == (int) -598741962);
                Debug.Assert(pack.pitchspeed == (float)1.3519423E38F);
                Debug.Assert(pack.pitch == (float) -6.7832465E37F);
                Debug.Assert(pack.zacc == (short)(short)4298);
                Debug.Assert(pack.roll == (float) -2.4794729E38F);
                Debug.Assert(pack.yawspeed == (float) -8.6202E37F);
                Debug.Assert(pack.vx == (short)(short) -12843);
                Debug.Assert(pack.alt == (int) -1384540715);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.roll = (float) -2.4794729E38F;
            p90.lat = (int) -598741962;
            p90.yacc = (short)(short) -17088;
            p90.time_usec = (ulong)5750120024386861722L;
            p90.lon = (int) -1732038902;
            p90.vz = (short)(short) -28907;
            p90.yaw = (float) -3.2863302E38F;
            p90.pitch = (float) -6.7832465E37F;
            p90.vy = (short)(short) -3367;
            p90.zacc = (short)(short)4298;
            p90.rollspeed = (float) -2.7756437E38F;
            p90.vx = (short)(short) -12843;
            p90.pitchspeed = (float)1.3519423E38F;
            p90.xacc = (short)(short) -18347;
            p90.alt = (int) -1384540715;
            p90.yawspeed = (float) -8.6202E37F;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.throttle == (float) -1.5815699E38F);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_AUTO_DISARMED);
                Debug.Assert(pack.aux3 == (float)5.3827553E37F);
                Debug.Assert(pack.roll_ailerons == (float) -2.5980221E38F);
                Debug.Assert(pack.yaw_rudder == (float) -1.2005037E38F);
                Debug.Assert(pack.pitch_elevator == (float)5.9976885E37F);
                Debug.Assert(pack.aux2 == (float)6.5726294E37F);
                Debug.Assert(pack.aux1 == (float)6.6865324E37F);
                Debug.Assert(pack.nav_mode == (byte)(byte)70);
                Debug.Assert(pack.time_usec == (ulong)6586331543847399256L);
                Debug.Assert(pack.aux4 == (float) -1.955271E38F);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)6586331543847399256L;
            p91.mode = MAV_MODE.MAV_MODE_AUTO_DISARMED;
            p91.throttle = (float) -1.5815699E38F;
            p91.aux3 = (float)5.3827553E37F;
            p91.nav_mode = (byte)(byte)70;
            p91.aux2 = (float)6.5726294E37F;
            p91.yaw_rudder = (float) -1.2005037E38F;
            p91.aux4 = (float) -1.955271E38F;
            p91.aux1 = (float)6.6865324E37F;
            p91.roll_ailerons = (float) -2.5980221E38F;
            p91.pitch_elevator = (float)5.9976885E37F;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)62935);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)62386);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)53113);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)38933);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)28926);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)11059);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)57296);
                Debug.Assert(pack.rssi == (byte)(byte)160);
                Debug.Assert(pack.time_usec == (ulong)1716677507586091755L);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)50925);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)42585);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)30588);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)57071);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)35011);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan7_raw = (ushort)(ushort)62935;
            p92.chan6_raw = (ushort)(ushort)30588;
            p92.chan4_raw = (ushort)(ushort)57071;
            p92.chan8_raw = (ushort)(ushort)11059;
            p92.chan10_raw = (ushort)(ushort)53113;
            p92.chan9_raw = (ushort)(ushort)62386;
            p92.chan1_raw = (ushort)(ushort)50925;
            p92.rssi = (byte)(byte)160;
            p92.chan11_raw = (ushort)(ushort)38933;
            p92.chan3_raw = (ushort)(ushort)35011;
            p92.time_usec = (ulong)1716677507586091755L;
            p92.chan12_raw = (ushort)(ushort)57296;
            p92.chan5_raw = (ushort)(ushort)28926;
            p92.chan2_raw = (ushort)(ushort)42585;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_MANUAL_ARMED);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-2.2094065E38F, 9.154718E37F, -3.0466424E38F, 2.6533412E38F, 8.175461E37F, -1.9350912E38F, 1.3909608E38F, -9.121202E37F, 3.3871823E38F, -2.5976351E38F, 2.1059287E38F, -3.0768196E38F, -3.0552018E38F, 8.343791E37F, 2.8536626E38F, 7.359533E37F}));
                Debug.Assert(pack.flags == (ulong)606962557309130429L);
                Debug.Assert(pack.time_usec == (ulong)5351228893659389172L);
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.mode = MAV_MODE.MAV_MODE_MANUAL_ARMED;
            p93.flags = (ulong)606962557309130429L;
            p93.controls_SET(new float[] {-2.2094065E38F, 9.154718E37F, -3.0466424E38F, 2.6533412E38F, 8.175461E37F, -1.9350912E38F, 1.3909608E38F, -9.121202E37F, 3.3871823E38F, -2.5976351E38F, 2.1059287E38F, -3.0768196E38F, -3.0552018E38F, 8.343791E37F, 2.8536626E38F, 7.359533E37F}, 0) ;
            p93.time_usec = (ulong)5351228893659389172L;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_id == (byte)(byte)214);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)2.0637157E38F);
                Debug.Assert(pack.flow_comp_m_y == (float)3.3101308E38F);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float) -3.0542193E38F);
                Debug.Assert(pack.quality == (byte)(byte)94);
                Debug.Assert(pack.flow_y == (short)(short)26662);
                Debug.Assert(pack.ground_distance == (float) -1.0550185E38F);
                Debug.Assert(pack.flow_comp_m_x == (float)2.2332004E38F);
                Debug.Assert(pack.flow_x == (short)(short) -31965);
                Debug.Assert(pack.time_usec == (ulong)2014299220064092119L);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_rate_x_SET((float)2.0637157E38F, PH) ;
            p100.flow_y = (short)(short)26662;
            p100.time_usec = (ulong)2014299220064092119L;
            p100.flow_rate_y_SET((float) -3.0542193E38F, PH) ;
            p100.flow_x = (short)(short) -31965;
            p100.sensor_id = (byte)(byte)214;
            p100.flow_comp_m_y = (float)3.3101308E38F;
            p100.ground_distance = (float) -1.0550185E38F;
            p100.quality = (byte)(byte)94;
            p100.flow_comp_m_x = (float)2.2332004E38F;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -2.146167E38F);
                Debug.Assert(pack.roll == (float) -1.0463546E38F);
                Debug.Assert(pack.usec == (ulong)570140163417473929L);
                Debug.Assert(pack.x == (float)2.4203926E38F);
                Debug.Assert(pack.z == (float) -3.3656785E38F);
                Debug.Assert(pack.yaw == (float)2.7070313E38F);
                Debug.Assert(pack.pitch == (float)3.5358609E37F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.x = (float)2.4203926E38F;
            p101.yaw = (float)2.7070313E38F;
            p101.pitch = (float)3.5358609E37F;
            p101.usec = (ulong)570140163417473929L;
            p101.z = (float) -3.3656785E38F;
            p101.y = (float) -2.146167E38F;
            p101.roll = (float) -1.0463546E38F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)1.4031697E37F);
                Debug.Assert(pack.pitch == (float) -5.5825146E36F);
                Debug.Assert(pack.usec == (ulong)1897659816297802998L);
                Debug.Assert(pack.x == (float) -2.9382806E38F);
                Debug.Assert(pack.y == (float)1.2416493E38F);
                Debug.Assert(pack.roll == (float) -1.9976457E37F);
                Debug.Assert(pack.yaw == (float) -9.268068E36F);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)1897659816297802998L;
            p102.z = (float)1.4031697E37F;
            p102.yaw = (float) -9.268068E36F;
            p102.y = (float)1.2416493E38F;
            p102.roll = (float) -1.9976457E37F;
            p102.pitch = (float) -5.5825146E36F;
            p102.x = (float) -2.9382806E38F;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -1.7477584E38F);
                Debug.Assert(pack.y == (float) -2.9194863E38F);
                Debug.Assert(pack.usec == (ulong)6161856212312340612L);
                Debug.Assert(pack.z == (float)2.1912472E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.z = (float)2.1912472E38F;
            p103.usec = (ulong)6161856212312340612L;
            p103.x = (float) -1.7477584E38F;
            p103.y = (float) -2.9194863E38F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -3.0566086E38F);
                Debug.Assert(pack.y == (float) -1.2107165E38F);
                Debug.Assert(pack.roll == (float)2.7206589E38F);
                Debug.Assert(pack.z == (float)3.0549933E38F);
                Debug.Assert(pack.x == (float) -4.412125E36F);
                Debug.Assert(pack.pitch == (float) -2.9651508E38F);
                Debug.Assert(pack.usec == (ulong)7142090319460809107L);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.yaw = (float) -3.0566086E38F;
            p104.usec = (ulong)7142090319460809107L;
            p104.pitch = (float) -2.9651508E38F;
            p104.z = (float)3.0549933E38F;
            p104.y = (float) -1.2107165E38F;
            p104.x = (float) -4.412125E36F;
            p104.roll = (float)2.7206589E38F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (float) -1.706056E38F);
                Debug.Assert(pack.abs_pressure == (float) -2.4926357E38F);
                Debug.Assert(pack.pressure_alt == (float)1.2975501E38F);
                Debug.Assert(pack.temperature == (float)4.0615122E37F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)64386);
                Debug.Assert(pack.ygyro == (float)3.2469974E38F);
                Debug.Assert(pack.xacc == (float)2.120863E38F);
                Debug.Assert(pack.time_usec == (ulong)3619080655374205190L);
                Debug.Assert(pack.zacc == (float) -1.2763211E38F);
                Debug.Assert(pack.yacc == (float) -6.037647E36F);
                Debug.Assert(pack.ymag == (float) -2.0614187E38F);
                Debug.Assert(pack.zmag == (float)1.6262297E37F);
                Debug.Assert(pack.diff_pressure == (float)3.282904E38F);
                Debug.Assert(pack.zgyro == (float) -1.3819641E38F);
                Debug.Assert(pack.xmag == (float) -1.0551927E38F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.xgyro = (float) -1.706056E38F;
            p105.abs_pressure = (float) -2.4926357E38F;
            p105.xmag = (float) -1.0551927E38F;
            p105.pressure_alt = (float)1.2975501E38F;
            p105.ymag = (float) -2.0614187E38F;
            p105.zgyro = (float) -1.3819641E38F;
            p105.xacc = (float)2.120863E38F;
            p105.time_usec = (ulong)3619080655374205190L;
            p105.ygyro = (float)3.2469974E38F;
            p105.temperature = (float)4.0615122E37F;
            p105.fields_updated = (ushort)(ushort)64386;
            p105.yacc = (float) -6.037647E36F;
            p105.zmag = (float)1.6262297E37F;
            p105.diff_pressure = (float)3.282904E38F;
            p105.zacc = (float) -1.2763211E38F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_delta_distance_us == (uint)554686026U);
                Debug.Assert(pack.integrated_xgyro == (float)1.6136102E38F);
                Debug.Assert(pack.integrated_y == (float) -2.636459E38F);
                Debug.Assert(pack.integration_time_us == (uint)3467879497U);
                Debug.Assert(pack.integrated_x == (float)7.0627554E37F);
                Debug.Assert(pack.distance == (float)2.6536531E38F);
                Debug.Assert(pack.integrated_ygyro == (float)1.0431413E38F);
                Debug.Assert(pack.quality == (byte)(byte)61);
                Debug.Assert(pack.temperature == (short)(short)14781);
                Debug.Assert(pack.sensor_id == (byte)(byte)143);
                Debug.Assert(pack.time_usec == (ulong)6313341193087901489L);
                Debug.Assert(pack.integrated_zgyro == (float)2.3469433E38F);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.distance = (float)2.6536531E38F;
            p106.time_usec = (ulong)6313341193087901489L;
            p106.quality = (byte)(byte)61;
            p106.temperature = (short)(short)14781;
            p106.integration_time_us = (uint)3467879497U;
            p106.integrated_y = (float) -2.636459E38F;
            p106.integrated_xgyro = (float)1.6136102E38F;
            p106.time_delta_distance_us = (uint)554686026U;
            p106.integrated_ygyro = (float)1.0431413E38F;
            p106.integrated_zgyro = (float)2.3469433E38F;
            p106.integrated_x = (float)7.0627554E37F;
            p106.sensor_id = (byte)(byte)143;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ymag == (float) -2.513699E38F);
                Debug.Assert(pack.temperature == (float)9.162079E35F);
                Debug.Assert(pack.pressure_alt == (float)6.1161723E37F);
                Debug.Assert(pack.xacc == (float) -2.0966095E38F);
                Debug.Assert(pack.yacc == (float) -3.3221084E38F);
                Debug.Assert(pack.time_usec == (ulong)5590575995261822368L);
                Debug.Assert(pack.fields_updated == (uint)3420878851U);
                Debug.Assert(pack.diff_pressure == (float)3.3146005E38F);
                Debug.Assert(pack.xgyro == (float) -2.4769548E38F);
                Debug.Assert(pack.xmag == (float)3.8296754E37F);
                Debug.Assert(pack.abs_pressure == (float)3.0369647E38F);
                Debug.Assert(pack.zgyro == (float)1.2745942E37F);
                Debug.Assert(pack.zmag == (float) -1.8610793E37F);
                Debug.Assert(pack.zacc == (float)5.343559E37F);
                Debug.Assert(pack.ygyro == (float)2.6861478E38F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.yacc = (float) -3.3221084E38F;
            p107.zacc = (float)5.343559E37F;
            p107.time_usec = (ulong)5590575995261822368L;
            p107.xgyro = (float) -2.4769548E38F;
            p107.pressure_alt = (float)6.1161723E37F;
            p107.xacc = (float) -2.0966095E38F;
            p107.diff_pressure = (float)3.3146005E38F;
            p107.temperature = (float)9.162079E35F;
            p107.ymag = (float) -2.513699E38F;
            p107.abs_pressure = (float)3.0369647E38F;
            p107.xmag = (float)3.8296754E37F;
            p107.fields_updated = (uint)3420878851U;
            p107.zmag = (float) -1.8610793E37F;
            p107.ygyro = (float)2.6861478E38F;
            p107.zgyro = (float)1.2745942E37F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float)3.0509845E38F);
                Debug.Assert(pack.ygyro == (float) -9.181302E37F);
                Debug.Assert(pack.alt == (float)2.1321849E38F);
                Debug.Assert(pack.ve == (float)8.52506E37F);
                Debug.Assert(pack.xacc == (float)5.308952E37F);
                Debug.Assert(pack.lat == (float) -1.9272924E38F);
                Debug.Assert(pack.vn == (float) -3.1722137E38F);
                Debug.Assert(pack.yacc == (float) -1.5941221E38F);
                Debug.Assert(pack.q1 == (float) -1.8791364E38F);
                Debug.Assert(pack.q3 == (float)2.5140951E38F);
                Debug.Assert(pack.zacc == (float)2.9491358E38F);
                Debug.Assert(pack.std_dev_horz == (float)3.3599004E38F);
                Debug.Assert(pack.vd == (float)7.328138E37F);
                Debug.Assert(pack.roll == (float) -1.5375204E38F);
                Debug.Assert(pack.xgyro == (float) -1.7290977E38F);
                Debug.Assert(pack.q2 == (float) -1.7023208E37F);
                Debug.Assert(pack.zgyro == (float)1.3539261E38F);
                Debug.Assert(pack.yaw == (float) -3.6939567E37F);
                Debug.Assert(pack.std_dev_vert == (float)6.3520643E37F);
                Debug.Assert(pack.lon == (float)1.6047771E38F);
                Debug.Assert(pack.q4 == (float) -2.3727018E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.alt = (float)2.1321849E38F;
            p108.ve = (float)8.52506E37F;
            p108.yaw = (float) -3.6939567E37F;
            p108.lat = (float) -1.9272924E38F;
            p108.std_dev_horz = (float)3.3599004E38F;
            p108.zacc = (float)2.9491358E38F;
            p108.xgyro = (float) -1.7290977E38F;
            p108.roll = (float) -1.5375204E38F;
            p108.q1 = (float) -1.8791364E38F;
            p108.zgyro = (float)1.3539261E38F;
            p108.q4 = (float) -2.3727018E38F;
            p108.vd = (float)7.328138E37F;
            p108.xacc = (float)5.308952E37F;
            p108.lon = (float)1.6047771E38F;
            p108.vn = (float) -3.1722137E38F;
            p108.std_dev_vert = (float)6.3520643E37F;
            p108.pitch = (float)3.0509845E38F;
            p108.q2 = (float) -1.7023208E37F;
            p108.q3 = (float)2.5140951E38F;
            p108.yacc = (float) -1.5941221E38F;
            p108.ygyro = (float) -9.181302E37F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rxerrors == (ushort)(ushort)8474);
                Debug.Assert(pack.txbuf == (byte)(byte)239);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)64800);
                Debug.Assert(pack.remrssi == (byte)(byte)79);
                Debug.Assert(pack.remnoise == (byte)(byte)243);
                Debug.Assert(pack.noise == (byte)(byte)164);
                Debug.Assert(pack.rssi == (byte)(byte)149);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.txbuf = (byte)(byte)239;
            p109.fixed_ = (ushort)(ushort)64800;
            p109.rssi = (byte)(byte)149;
            p109.rxerrors = (ushort)(ushort)8474;
            p109.noise = (byte)(byte)164;
            p109.remnoise = (byte)(byte)243;
            p109.remrssi = (byte)(byte)79;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)122);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)48, (byte)91, (byte)118, (byte)72, (byte)122, (byte)103, (byte)94, (byte)197, (byte)6, (byte)131, (byte)252, (byte)201, (byte)116, (byte)145, (byte)71, (byte)242, (byte)197, (byte)200, (byte)100, (byte)157, (byte)135, (byte)86, (byte)42, (byte)46, (byte)224, (byte)166, (byte)37, (byte)53, (byte)232, (byte)139, (byte)226, (byte)227, (byte)9, (byte)191, (byte)232, (byte)144, (byte)54, (byte)133, (byte)234, (byte)55, (byte)25, (byte)167, (byte)171, (byte)56, (byte)108, (byte)16, (byte)37, (byte)18, (byte)89, (byte)201, (byte)157, (byte)63, (byte)234, (byte)133, (byte)145, (byte)180, (byte)35, (byte)135, (byte)239, (byte)199, (byte)84, (byte)170, (byte)94, (byte)21, (byte)163, (byte)243, (byte)125, (byte)144, (byte)182, (byte)44, (byte)47, (byte)66, (byte)19, (byte)20, (byte)250, (byte)136, (byte)58, (byte)15, (byte)85, (byte)49, (byte)237, (byte)109, (byte)80, (byte)231, (byte)3, (byte)93, (byte)174, (byte)89, (byte)224, (byte)17, (byte)55, (byte)91, (byte)60, (byte)86, (byte)198, (byte)237, (byte)128, (byte)12, (byte)87, (byte)234, (byte)64, (byte)59, (byte)45, (byte)165, (byte)173, (byte)155, (byte)19, (byte)116, (byte)224, (byte)149, (byte)128, (byte)197, (byte)16, (byte)9, (byte)184, (byte)80, (byte)190, (byte)179, (byte)197, (byte)137, (byte)6, (byte)118, (byte)13, (byte)248, (byte)132, (byte)250, (byte)59, (byte)244, (byte)26, (byte)149, (byte)13, (byte)195, (byte)216, (byte)152, (byte)128, (byte)69, (byte)165, (byte)41, (byte)117, (byte)162, (byte)44, (byte)8, (byte)69, (byte)17, (byte)49, (byte)212, (byte)208, (byte)229, (byte)234, (byte)57, (byte)146, (byte)236, (byte)235, (byte)100, (byte)132, (byte)102, (byte)103, (byte)89, (byte)213, (byte)86, (byte)239, (byte)197, (byte)210, (byte)248, (byte)107, (byte)78, (byte)68, (byte)159, (byte)42, (byte)244, (byte)92, (byte)21, (byte)114, (byte)169, (byte)216, (byte)174, (byte)75, (byte)227, (byte)197, (byte)155, (byte)101, (byte)139, (byte)127, (byte)149, (byte)182, (byte)10, (byte)249, (byte)28, (byte)86, (byte)244, (byte)107, (byte)208, (byte)251, (byte)170, (byte)176, (byte)141, (byte)142, (byte)158, (byte)7, (byte)223, (byte)61, (byte)34, (byte)0, (byte)240, (byte)71, (byte)33, (byte)175, (byte)2, (byte)50, (byte)9, (byte)21, (byte)149, (byte)188, (byte)215, (byte)155, (byte)241, (byte)1, (byte)62, (byte)209, (byte)241, (byte)251, (byte)177, (byte)18, (byte)99, (byte)83, (byte)197, (byte)0, (byte)93, (byte)79, (byte)225, (byte)4, (byte)145, (byte)57, (byte)138, (byte)1, (byte)34, (byte)71, (byte)36, (byte)87, (byte)34, (byte)23, (byte)166, (byte)220, (byte)77, (byte)76, (byte)164, (byte)124, (byte)221, (byte)37, (byte)215, (byte)113}));
                Debug.Assert(pack.target_network == (byte)(byte)55);
                Debug.Assert(pack.target_system == (byte)(byte)127);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.payload_SET(new byte[] {(byte)48, (byte)91, (byte)118, (byte)72, (byte)122, (byte)103, (byte)94, (byte)197, (byte)6, (byte)131, (byte)252, (byte)201, (byte)116, (byte)145, (byte)71, (byte)242, (byte)197, (byte)200, (byte)100, (byte)157, (byte)135, (byte)86, (byte)42, (byte)46, (byte)224, (byte)166, (byte)37, (byte)53, (byte)232, (byte)139, (byte)226, (byte)227, (byte)9, (byte)191, (byte)232, (byte)144, (byte)54, (byte)133, (byte)234, (byte)55, (byte)25, (byte)167, (byte)171, (byte)56, (byte)108, (byte)16, (byte)37, (byte)18, (byte)89, (byte)201, (byte)157, (byte)63, (byte)234, (byte)133, (byte)145, (byte)180, (byte)35, (byte)135, (byte)239, (byte)199, (byte)84, (byte)170, (byte)94, (byte)21, (byte)163, (byte)243, (byte)125, (byte)144, (byte)182, (byte)44, (byte)47, (byte)66, (byte)19, (byte)20, (byte)250, (byte)136, (byte)58, (byte)15, (byte)85, (byte)49, (byte)237, (byte)109, (byte)80, (byte)231, (byte)3, (byte)93, (byte)174, (byte)89, (byte)224, (byte)17, (byte)55, (byte)91, (byte)60, (byte)86, (byte)198, (byte)237, (byte)128, (byte)12, (byte)87, (byte)234, (byte)64, (byte)59, (byte)45, (byte)165, (byte)173, (byte)155, (byte)19, (byte)116, (byte)224, (byte)149, (byte)128, (byte)197, (byte)16, (byte)9, (byte)184, (byte)80, (byte)190, (byte)179, (byte)197, (byte)137, (byte)6, (byte)118, (byte)13, (byte)248, (byte)132, (byte)250, (byte)59, (byte)244, (byte)26, (byte)149, (byte)13, (byte)195, (byte)216, (byte)152, (byte)128, (byte)69, (byte)165, (byte)41, (byte)117, (byte)162, (byte)44, (byte)8, (byte)69, (byte)17, (byte)49, (byte)212, (byte)208, (byte)229, (byte)234, (byte)57, (byte)146, (byte)236, (byte)235, (byte)100, (byte)132, (byte)102, (byte)103, (byte)89, (byte)213, (byte)86, (byte)239, (byte)197, (byte)210, (byte)248, (byte)107, (byte)78, (byte)68, (byte)159, (byte)42, (byte)244, (byte)92, (byte)21, (byte)114, (byte)169, (byte)216, (byte)174, (byte)75, (byte)227, (byte)197, (byte)155, (byte)101, (byte)139, (byte)127, (byte)149, (byte)182, (byte)10, (byte)249, (byte)28, (byte)86, (byte)244, (byte)107, (byte)208, (byte)251, (byte)170, (byte)176, (byte)141, (byte)142, (byte)158, (byte)7, (byte)223, (byte)61, (byte)34, (byte)0, (byte)240, (byte)71, (byte)33, (byte)175, (byte)2, (byte)50, (byte)9, (byte)21, (byte)149, (byte)188, (byte)215, (byte)155, (byte)241, (byte)1, (byte)62, (byte)209, (byte)241, (byte)251, (byte)177, (byte)18, (byte)99, (byte)83, (byte)197, (byte)0, (byte)93, (byte)79, (byte)225, (byte)4, (byte)145, (byte)57, (byte)138, (byte)1, (byte)34, (byte)71, (byte)36, (byte)87, (byte)34, (byte)23, (byte)166, (byte)220, (byte)77, (byte)76, (byte)164, (byte)124, (byte)221, (byte)37, (byte)215, (byte)113}, 0) ;
            p110.target_system = (byte)(byte)127;
            p110.target_network = (byte)(byte)55;
            p110.target_component = (byte)(byte)122;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tc1 == (long) -8446506937197664579L);
                Debug.Assert(pack.ts1 == (long)247132602702149708L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long)247132602702149708L;
            p111.tc1 = (long) -8446506937197664579L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)935513709U);
                Debug.Assert(pack.time_usec == (ulong)1275151654496666452L);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.seq = (uint)935513709U;
            p112.time_usec = (ulong)1275151654496666452L;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.eph == (ushort)(ushort)36354);
                Debug.Assert(pack.vel == (ushort)(ushort)61610);
                Debug.Assert(pack.alt == (int) -125181216);
                Debug.Assert(pack.lat == (int)790522449);
                Debug.Assert(pack.fix_type == (byte)(byte)121);
                Debug.Assert(pack.satellites_visible == (byte)(byte)175);
                Debug.Assert(pack.time_usec == (ulong)94065095094652482L);
                Debug.Assert(pack.vn == (short)(short) -31719);
                Debug.Assert(pack.vd == (short)(short)18047);
                Debug.Assert(pack.cog == (ushort)(ushort)31381);
                Debug.Assert(pack.epv == (ushort)(ushort)62930);
                Debug.Assert(pack.lon == (int)1612780122);
                Debug.Assert(pack.ve == (short)(short)18455);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.eph = (ushort)(ushort)36354;
            p113.vel = (ushort)(ushort)61610;
            p113.vd = (short)(short)18047;
            p113.lon = (int)1612780122;
            p113.cog = (ushort)(ushort)31381;
            p113.vn = (short)(short) -31719;
            p113.fix_type = (byte)(byte)121;
            p113.lat = (int)790522449;
            p113.satellites_visible = (byte)(byte)175;
            p113.epv = (ushort)(ushort)62930;
            p113.ve = (short)(short)18455;
            p113.alt = (int) -125181216;
            p113.time_usec = (ulong)94065095094652482L;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_xgyro == (float) -1.9436708E38F);
                Debug.Assert(pack.integrated_x == (float) -1.0833781E36F);
                Debug.Assert(pack.integration_time_us == (uint)1259343475U);
                Debug.Assert(pack.quality == (byte)(byte)138);
                Debug.Assert(pack.sensor_id == (byte)(byte)92);
                Debug.Assert(pack.integrated_ygyro == (float)1.9828637E38F);
                Debug.Assert(pack.distance == (float) -2.187749E36F);
                Debug.Assert(pack.integrated_zgyro == (float)1.9225643E38F);
                Debug.Assert(pack.time_usec == (ulong)6761181990089857276L);
                Debug.Assert(pack.temperature == (short)(short) -23341);
                Debug.Assert(pack.time_delta_distance_us == (uint)2370080771U);
                Debug.Assert(pack.integrated_y == (float) -2.1737549E38F);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.integration_time_us = (uint)1259343475U;
            p114.time_delta_distance_us = (uint)2370080771U;
            p114.integrated_y = (float) -2.1737549E38F;
            p114.quality = (byte)(byte)138;
            p114.integrated_zgyro = (float)1.9225643E38F;
            p114.integrated_ygyro = (float)1.9828637E38F;
            p114.sensor_id = (byte)(byte)92;
            p114.distance = (float) -2.187749E36F;
            p114.integrated_x = (float) -1.0833781E36F;
            p114.integrated_xgyro = (float) -1.9436708E38F;
            p114.temperature = (short)(short) -23341;
            p114.time_usec = (ulong)6761181990089857276L;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short)20966);
                Debug.Assert(pack.time_usec == (ulong)8737528080584864899L);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)30060);
                Debug.Assert(pack.lon == (int)1086573938);
                Debug.Assert(pack.rollspeed == (float)1.20235E38F);
                Debug.Assert(pack.vy == (short)(short)30620);
                Debug.Assert(pack.yawspeed == (float) -1.5681308E38F);
                Debug.Assert(pack.yacc == (short)(short)6953);
                Debug.Assert(pack.vx == (short)(short) -12874);
                Debug.Assert(pack.pitchspeed == (float) -2.3918287E38F);
                Debug.Assert(pack.zacc == (short)(short) -9444);
                Debug.Assert(pack.vz == (short)(short)21646);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {-2.9967483E38F, -2.9684982E38F, 2.1770747E38F, 1.5469373E38F}));
                Debug.Assert(pack.lat == (int) -327590787);
                Debug.Assert(pack.alt == (int)1545806027);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)224);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.pitchspeed = (float) -2.3918287E38F;
            p115.rollspeed = (float)1.20235E38F;
            p115.vz = (short)(short)21646;
            p115.vy = (short)(short)30620;
            p115.ind_airspeed = (ushort)(ushort)224;
            p115.lat = (int) -327590787;
            p115.attitude_quaternion_SET(new float[] {-2.9967483E38F, -2.9684982E38F, 2.1770747E38F, 1.5469373E38F}, 0) ;
            p115.zacc = (short)(short) -9444;
            p115.alt = (int)1545806027;
            p115.xacc = (short)(short)20966;
            p115.lon = (int)1086573938;
            p115.time_usec = (ulong)8737528080584864899L;
            p115.vx = (short)(short) -12874;
            p115.yawspeed = (float) -1.5681308E38F;
            p115.yacc = (short)(short)6953;
            p115.true_airspeed = (ushort)(ushort)30060;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short)24534);
                Debug.Assert(pack.time_boot_ms == (uint)1997839095U);
                Debug.Assert(pack.zmag == (short)(short)23001);
                Debug.Assert(pack.xgyro == (short)(short)16672);
                Debug.Assert(pack.yacc == (short)(short)20854);
                Debug.Assert(pack.zacc == (short)(short)5865);
                Debug.Assert(pack.ygyro == (short)(short) -6669);
                Debug.Assert(pack.zgyro == (short)(short) -19566);
                Debug.Assert(pack.xmag == (short)(short)23091);
                Debug.Assert(pack.ymag == (short)(short) -26498);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.zgyro = (short)(short) -19566;
            p116.zacc = (short)(short)5865;
            p116.xacc = (short)(short)24534;
            p116.xgyro = (short)(short)16672;
            p116.time_boot_ms = (uint)1997839095U;
            p116.yacc = (short)(short)20854;
            p116.ygyro = (short)(short) -6669;
            p116.zmag = (short)(short)23001;
            p116.ymag = (short)(short) -26498;
            p116.xmag = (short)(short)23091;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end == (ushort)(ushort)49837);
                Debug.Assert(pack.target_system == (byte)(byte)99);
                Debug.Assert(pack.start == (ushort)(ushort)54292);
                Debug.Assert(pack.target_component == (byte)(byte)136);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.end = (ushort)(ushort)49837;
            p117.start = (ushort)(ushort)54292;
            p117.target_component = (byte)(byte)136;
            p117.target_system = (byte)(byte)99;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)15796);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)2961);
                Debug.Assert(pack.num_logs == (ushort)(ushort)3426);
                Debug.Assert(pack.time_utc == (uint)3714289956U);
                Debug.Assert(pack.size == (uint)2380948762U);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)15796;
            p118.last_log_num = (ushort)(ushort)2961;
            p118.time_utc = (uint)3714289956U;
            p118.size = (uint)2380948762U;
            p118.num_logs = (ushort)(ushort)3426;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)1948764182U);
                Debug.Assert(pack.count == (uint)1061026388U);
                Debug.Assert(pack.target_system == (byte)(byte)238);
                Debug.Assert(pack.id == (ushort)(ushort)32760);
                Debug.Assert(pack.target_component == (byte)(byte)90);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)238;
            p119.count = (uint)1061026388U;
            p119.target_component = (byte)(byte)90;
            p119.id = (ushort)(ushort)32760;
            p119.ofs = (uint)1948764182U;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (byte)(byte)97);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)157, (byte)236, (byte)242, (byte)100, (byte)212, (byte)107, (byte)15, (byte)172, (byte)54, (byte)65, (byte)82, (byte)68, (byte)49, (byte)123, (byte)63, (byte)69, (byte)70, (byte)205, (byte)108, (byte)204, (byte)200, (byte)199, (byte)2, (byte)51, (byte)228, (byte)234, (byte)166, (byte)243, (byte)151, (byte)199, (byte)174, (byte)190, (byte)25, (byte)216, (byte)234, (byte)166, (byte)205, (byte)96, (byte)167, (byte)245, (byte)235, (byte)75, (byte)38, (byte)94, (byte)159, (byte)211, (byte)17, (byte)118, (byte)32, (byte)118, (byte)254, (byte)39, (byte)2, (byte)56, (byte)11, (byte)21, (byte)150, (byte)87, (byte)84, (byte)25, (byte)91, (byte)249, (byte)223, (byte)252, (byte)207, (byte)72, (byte)8, (byte)206, (byte)121, (byte)203, (byte)253, (byte)230, (byte)200, (byte)214, (byte)7, (byte)44, (byte)235, (byte)109, (byte)3, (byte)105, (byte)210, (byte)7, (byte)225, (byte)73, (byte)188, (byte)80, (byte)201, (byte)127, (byte)63, (byte)67}));
                Debug.Assert(pack.ofs == (uint)4006188873U);
                Debug.Assert(pack.id == (ushort)(ushort)37582);
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.data__SET(new byte[] {(byte)157, (byte)236, (byte)242, (byte)100, (byte)212, (byte)107, (byte)15, (byte)172, (byte)54, (byte)65, (byte)82, (byte)68, (byte)49, (byte)123, (byte)63, (byte)69, (byte)70, (byte)205, (byte)108, (byte)204, (byte)200, (byte)199, (byte)2, (byte)51, (byte)228, (byte)234, (byte)166, (byte)243, (byte)151, (byte)199, (byte)174, (byte)190, (byte)25, (byte)216, (byte)234, (byte)166, (byte)205, (byte)96, (byte)167, (byte)245, (byte)235, (byte)75, (byte)38, (byte)94, (byte)159, (byte)211, (byte)17, (byte)118, (byte)32, (byte)118, (byte)254, (byte)39, (byte)2, (byte)56, (byte)11, (byte)21, (byte)150, (byte)87, (byte)84, (byte)25, (byte)91, (byte)249, (byte)223, (byte)252, (byte)207, (byte)72, (byte)8, (byte)206, (byte)121, (byte)203, (byte)253, (byte)230, (byte)200, (byte)214, (byte)7, (byte)44, (byte)235, (byte)109, (byte)3, (byte)105, (byte)210, (byte)7, (byte)225, (byte)73, (byte)188, (byte)80, (byte)201, (byte)127, (byte)63, (byte)67}, 0) ;
            p120.count = (byte)(byte)97;
            p120.ofs = (uint)4006188873U;
            p120.id = (ushort)(ushort)37582;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)243);
                Debug.Assert(pack.target_component == (byte)(byte)169);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_component = (byte)(byte)169;
            p121.target_system = (byte)(byte)243;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)176);
                Debug.Assert(pack.target_component == (byte)(byte)113);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)176;
            p122.target_component = (byte)(byte)113;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)206);
                Debug.Assert(pack.target_component == (byte)(byte)24);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)94, (byte)26, (byte)10, (byte)206, (byte)18, (byte)167, (byte)22, (byte)94, (byte)226, (byte)112, (byte)89, (byte)198, (byte)58, (byte)73, (byte)145, (byte)154, (byte)154, (byte)106, (byte)240, (byte)249, (byte)118, (byte)210, (byte)117, (byte)219, (byte)45, (byte)175, (byte)86, (byte)50, (byte)53, (byte)13, (byte)166, (byte)77, (byte)200, (byte)25, (byte)253, (byte)188, (byte)75, (byte)16, (byte)234, (byte)126, (byte)2, (byte)168, (byte)78, (byte)170, (byte)198, (byte)160, (byte)51, (byte)58, (byte)119, (byte)53, (byte)136, (byte)123, (byte)183, (byte)143, (byte)246, (byte)99, (byte)248, (byte)115, (byte)141, (byte)239, (byte)10, (byte)140, (byte)207, (byte)27, (byte)104, (byte)117, (byte)81, (byte)56, (byte)250, (byte)172, (byte)133, (byte)50, (byte)224, (byte)228, (byte)229, (byte)193, (byte)90, (byte)80, (byte)55, (byte)78, (byte)209, (byte)43, (byte)200, (byte)169, (byte)164, (byte)100, (byte)154, (byte)30, (byte)94, (byte)186, (byte)45, (byte)172, (byte)189, (byte)199, (byte)183, (byte)71, (byte)201, (byte)105, (byte)141, (byte)171, (byte)192, (byte)248, (byte)161, (byte)110, (byte)199, (byte)177, (byte)48, (byte)234, (byte)216, (byte)37}));
                Debug.Assert(pack.len == (byte)(byte)39);
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.data__SET(new byte[] {(byte)94, (byte)26, (byte)10, (byte)206, (byte)18, (byte)167, (byte)22, (byte)94, (byte)226, (byte)112, (byte)89, (byte)198, (byte)58, (byte)73, (byte)145, (byte)154, (byte)154, (byte)106, (byte)240, (byte)249, (byte)118, (byte)210, (byte)117, (byte)219, (byte)45, (byte)175, (byte)86, (byte)50, (byte)53, (byte)13, (byte)166, (byte)77, (byte)200, (byte)25, (byte)253, (byte)188, (byte)75, (byte)16, (byte)234, (byte)126, (byte)2, (byte)168, (byte)78, (byte)170, (byte)198, (byte)160, (byte)51, (byte)58, (byte)119, (byte)53, (byte)136, (byte)123, (byte)183, (byte)143, (byte)246, (byte)99, (byte)248, (byte)115, (byte)141, (byte)239, (byte)10, (byte)140, (byte)207, (byte)27, (byte)104, (byte)117, (byte)81, (byte)56, (byte)250, (byte)172, (byte)133, (byte)50, (byte)224, (byte)228, (byte)229, (byte)193, (byte)90, (byte)80, (byte)55, (byte)78, (byte)209, (byte)43, (byte)200, (byte)169, (byte)164, (byte)100, (byte)154, (byte)30, (byte)94, (byte)186, (byte)45, (byte)172, (byte)189, (byte)199, (byte)183, (byte)71, (byte)201, (byte)105, (byte)141, (byte)171, (byte)192, (byte)248, (byte)161, (byte)110, (byte)199, (byte)177, (byte)48, (byte)234, (byte)216, (byte)37}, 0) ;
            p123.target_component = (byte)(byte)24;
            p123.len = (byte)(byte)39;
            p123.target_system = (byte)(byte)206;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellites_visible == (byte)(byte)210);
                Debug.Assert(pack.eph == (ushort)(ushort)44498);
                Debug.Assert(pack.alt == (int) -1024909079);
                Debug.Assert(pack.time_usec == (ulong)7253689355834222905L);
                Debug.Assert(pack.lon == (int)1906220590);
                Debug.Assert(pack.dgps_age == (uint)1873607088U);
                Debug.Assert(pack.dgps_numch == (byte)(byte)10);
                Debug.Assert(pack.lat == (int)376377521);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
                Debug.Assert(pack.cog == (ushort)(ushort)39244);
                Debug.Assert(pack.vel == (ushort)(ushort)40659);
                Debug.Assert(pack.epv == (ushort)(ushort)50599);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.cog = (ushort)(ushort)39244;
            p124.alt = (int) -1024909079;
            p124.lat = (int)376377521;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED;
            p124.satellites_visible = (byte)(byte)210;
            p124.lon = (int)1906220590;
            p124.eph = (ushort)(ushort)44498;
            p124.dgps_age = (uint)1873607088U;
            p124.time_usec = (ulong)7253689355834222905L;
            p124.vel = (ushort)(ushort)40659;
            p124.epv = (ushort)(ushort)50599;
            p124.dgps_numch = (byte)(byte)10;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED));
                Debug.Assert(pack.Vservo == (ushort)(ushort)34033);
                Debug.Assert(pack.Vcc == (ushort)(ushort)35048);
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)35048;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED);
            p125.Vservo = (ushort)(ushort)34033;
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.timeout == (ushort)(ushort)23878);
                Debug.Assert(pack.baudrate == (uint)356026069U);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)71, (byte)157, (byte)77, (byte)223, (byte)103, (byte)38, (byte)216, (byte)177, (byte)238, (byte)164, (byte)170, (byte)121, (byte)139, (byte)72, (byte)2, (byte)221, (byte)191, (byte)235, (byte)152, (byte)68, (byte)112, (byte)57, (byte)185, (byte)216, (byte)127, (byte)126, (byte)48, (byte)117, (byte)140, (byte)235, (byte)65, (byte)53, (byte)142, (byte)33, (byte)147, (byte)150, (byte)11, (byte)118, (byte)209, (byte)67, (byte)10, (byte)71, (byte)142, (byte)36, (byte)31, (byte)167, (byte)35, (byte)47, (byte)2, (byte)56, (byte)45, (byte)129, (byte)187, (byte)61, (byte)92, (byte)86, (byte)169, (byte)67, (byte)37, (byte)192, (byte)202, (byte)56, (byte)214, (byte)46, (byte)252, (byte)231, (byte)40, (byte)164, (byte)164, (byte)62}));
                Debug.Assert(pack.count == (byte)(byte)255);
                Debug.Assert(pack.device == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                                            SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE));
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2;
            p126.baudrate = (uint)356026069U;
            p126.data__SET(new byte[] {(byte)71, (byte)157, (byte)77, (byte)223, (byte)103, (byte)38, (byte)216, (byte)177, (byte)238, (byte)164, (byte)170, (byte)121, (byte)139, (byte)72, (byte)2, (byte)221, (byte)191, (byte)235, (byte)152, (byte)68, (byte)112, (byte)57, (byte)185, (byte)216, (byte)127, (byte)126, (byte)48, (byte)117, (byte)140, (byte)235, (byte)65, (byte)53, (byte)142, (byte)33, (byte)147, (byte)150, (byte)11, (byte)118, (byte)209, (byte)67, (byte)10, (byte)71, (byte)142, (byte)36, (byte)31, (byte)167, (byte)35, (byte)47, (byte)2, (byte)56, (byte)45, (byte)129, (byte)187, (byte)61, (byte)92, (byte)86, (byte)169, (byte)67, (byte)37, (byte)192, (byte)202, (byte)56, (byte)214, (byte)46, (byte)252, (byte)231, (byte)40, (byte)164, (byte)164, (byte)62}, 0) ;
            p126.timeout = (ushort)(ushort)23878;
            p126.count = (byte)(byte)255;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_c_mm == (int)1049917049);
                Debug.Assert(pack.rtk_rate == (byte)(byte)36);
                Debug.Assert(pack.iar_num_hypotheses == (int) -1611737432);
                Debug.Assert(pack.tow == (uint)71103244U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)246);
                Debug.Assert(pack.baseline_a_mm == (int) -1994831300);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)146);
                Debug.Assert(pack.time_last_baseline_ms == (uint)250234997U);
                Debug.Assert(pack.rtk_health == (byte)(byte)56);
                Debug.Assert(pack.wn == (ushort)(ushort)34238);
                Debug.Assert(pack.accuracy == (uint)931381023U);
                Debug.Assert(pack.baseline_b_mm == (int) -713952516);
                Debug.Assert(pack.nsats == (byte)(byte)181);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.rtk_rate = (byte)(byte)36;
            p127.baseline_coords_type = (byte)(byte)146;
            p127.nsats = (byte)(byte)181;
            p127.baseline_a_mm = (int) -1994831300;
            p127.rtk_health = (byte)(byte)56;
            p127.baseline_c_mm = (int)1049917049;
            p127.accuracy = (uint)931381023U;
            p127.rtk_receiver_id = (byte)(byte)246;
            p127.iar_num_hypotheses = (int) -1611737432;
            p127.time_last_baseline_ms = (uint)250234997U;
            p127.tow = (uint)71103244U;
            p127.baseline_b_mm = (int) -713952516;
            p127.wn = (ushort)(ushort)34238;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baseline_a_mm == (int)1289167660);
                Debug.Assert(pack.iar_num_hypotheses == (int) -1039935478);
                Debug.Assert(pack.tow == (uint)3672913863U);
                Debug.Assert(pack.nsats == (byte)(byte)21);
                Debug.Assert(pack.rtk_rate == (byte)(byte)92);
                Debug.Assert(pack.baseline_c_mm == (int)1476554504);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)110);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)48);
                Debug.Assert(pack.time_last_baseline_ms == (uint)2208759291U);
                Debug.Assert(pack.accuracy == (uint)2273236601U);
                Debug.Assert(pack.wn == (ushort)(ushort)3034);
                Debug.Assert(pack.rtk_health == (byte)(byte)120);
                Debug.Assert(pack.baseline_b_mm == (int)1387640358);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.baseline_c_mm = (int)1476554504;
            p128.iar_num_hypotheses = (int) -1039935478;
            p128.rtk_receiver_id = (byte)(byte)110;
            p128.time_last_baseline_ms = (uint)2208759291U;
            p128.baseline_a_mm = (int)1289167660;
            p128.accuracy = (uint)2273236601U;
            p128.tow = (uint)3672913863U;
            p128.baseline_coords_type = (byte)(byte)48;
            p128.nsats = (byte)(byte)21;
            p128.rtk_rate = (byte)(byte)92;
            p128.rtk_health = (byte)(byte)120;
            p128.wn = (ushort)(ushort)3034;
            p128.baseline_b_mm = (int)1387640358;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zmag == (short)(short)4520);
                Debug.Assert(pack.xmag == (short)(short)7547);
                Debug.Assert(pack.time_boot_ms == (uint)3693758641U);
                Debug.Assert(pack.xacc == (short)(short) -31773);
                Debug.Assert(pack.yacc == (short)(short)8774);
                Debug.Assert(pack.xgyro == (short)(short) -10754);
                Debug.Assert(pack.zgyro == (short)(short) -17623);
                Debug.Assert(pack.ymag == (short)(short)1892);
                Debug.Assert(pack.zacc == (short)(short)10731);
                Debug.Assert(pack.ygyro == (short)(short) -7547);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.ymag = (short)(short)1892;
            p129.xacc = (short)(short) -31773;
            p129.yacc = (short)(short)8774;
            p129.xmag = (short)(short)7547;
            p129.zacc = (short)(short)10731;
            p129.ygyro = (short)(short) -7547;
            p129.zgyro = (short)(short) -17623;
            p129.zmag = (short)(short)4520;
            p129.time_boot_ms = (uint)3693758641U;
            p129.xgyro = (short)(short) -10754;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.width == (ushort)(ushort)7169);
                Debug.Assert(pack.size == (uint)264133203U);
                Debug.Assert(pack.height == (ushort)(ushort)14807);
                Debug.Assert(pack.jpg_quality == (byte)(byte)57);
                Debug.Assert(pack.type == (byte)(byte)206);
                Debug.Assert(pack.payload == (byte)(byte)167);
                Debug.Assert(pack.packets == (ushort)(ushort)32226);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.width = (ushort)(ushort)7169;
            p130.packets = (ushort)(ushort)32226;
            p130.payload = (byte)(byte)167;
            p130.size = (uint)264133203U;
            p130.height = (ushort)(ushort)14807;
            p130.type = (byte)(byte)206;
            p130.jpg_quality = (byte)(byte)57;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)236, (byte)100, (byte)70, (byte)83, (byte)173, (byte)172, (byte)69, (byte)73, (byte)119, (byte)163, (byte)174, (byte)172, (byte)222, (byte)206, (byte)10, (byte)81, (byte)92, (byte)5, (byte)234, (byte)242, (byte)22, (byte)12, (byte)22, (byte)58, (byte)234, (byte)203, (byte)219, (byte)200, (byte)246, (byte)81, (byte)139, (byte)113, (byte)65, (byte)59, (byte)195, (byte)234, (byte)121, (byte)161, (byte)21, (byte)216, (byte)82, (byte)159, (byte)74, (byte)124, (byte)26, (byte)183, (byte)94, (byte)98, (byte)94, (byte)224, (byte)222, (byte)19, (byte)215, (byte)29, (byte)10, (byte)54, (byte)103, (byte)141, (byte)82, (byte)234, (byte)234, (byte)30, (byte)29, (byte)204, (byte)148, (byte)26, (byte)39, (byte)96, (byte)6, (byte)235, (byte)13, (byte)246, (byte)133, (byte)215, (byte)199, (byte)199, (byte)65, (byte)87, (byte)94, (byte)78, (byte)185, (byte)125, (byte)67, (byte)23, (byte)247, (byte)83, (byte)238, (byte)14, (byte)165, (byte)87, (byte)198, (byte)233, (byte)140, (byte)57, (byte)60, (byte)37, (byte)37, (byte)175, (byte)32, (byte)60, (byte)170, (byte)159, (byte)65, (byte)204, (byte)253, (byte)234, (byte)122, (byte)214, (byte)122, (byte)76, (byte)98, (byte)217, (byte)52, (byte)75, (byte)11, (byte)212, (byte)206, (byte)184, (byte)182, (byte)224, (byte)25, (byte)84, (byte)63, (byte)14, (byte)177, (byte)80, (byte)169, (byte)200, (byte)247, (byte)247, (byte)34, (byte)21, (byte)86, (byte)220, (byte)192, (byte)168, (byte)137, (byte)173, (byte)136, (byte)131, (byte)233, (byte)183, (byte)30, (byte)20, (byte)38, (byte)43, (byte)127, (byte)245, (byte)146, (byte)20, (byte)99, (byte)16, (byte)146, (byte)51, (byte)216, (byte)84, (byte)90, (byte)214, (byte)26, (byte)5, (byte)210, (byte)34, (byte)61, (byte)17, (byte)115, (byte)122, (byte)225, (byte)72, (byte)86, (byte)209, (byte)204, (byte)84, (byte)237, (byte)117, (byte)250, (byte)23, (byte)96, (byte)175, (byte)204, (byte)77, (byte)18, (byte)95, (byte)55, (byte)1, (byte)56, (byte)215, (byte)27, (byte)99, (byte)55, (byte)87, (byte)121, (byte)145, (byte)122, (byte)39, (byte)55, (byte)65, (byte)109, (byte)182, (byte)79, (byte)181, (byte)165, (byte)54, (byte)156, (byte)53, (byte)219, (byte)253, (byte)74, (byte)73, (byte)161, (byte)43, (byte)21, (byte)105, (byte)212, (byte)113, (byte)237, (byte)91, (byte)237, (byte)109, (byte)100, (byte)5, (byte)34, (byte)130, (byte)242, (byte)19, (byte)40, (byte)16, (byte)16, (byte)120, (byte)90, (byte)129, (byte)101, (byte)142, (byte)249, (byte)35, (byte)244, (byte)63, (byte)132, (byte)48, (byte)199, (byte)154, (byte)143, (byte)79, (byte)21, (byte)78, (byte)98, (byte)28, (byte)188, (byte)8, (byte)198, (byte)79, (byte)124, (byte)215, (byte)29}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)31225);
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)31225;
            p131.data__SET(new byte[] {(byte)236, (byte)100, (byte)70, (byte)83, (byte)173, (byte)172, (byte)69, (byte)73, (byte)119, (byte)163, (byte)174, (byte)172, (byte)222, (byte)206, (byte)10, (byte)81, (byte)92, (byte)5, (byte)234, (byte)242, (byte)22, (byte)12, (byte)22, (byte)58, (byte)234, (byte)203, (byte)219, (byte)200, (byte)246, (byte)81, (byte)139, (byte)113, (byte)65, (byte)59, (byte)195, (byte)234, (byte)121, (byte)161, (byte)21, (byte)216, (byte)82, (byte)159, (byte)74, (byte)124, (byte)26, (byte)183, (byte)94, (byte)98, (byte)94, (byte)224, (byte)222, (byte)19, (byte)215, (byte)29, (byte)10, (byte)54, (byte)103, (byte)141, (byte)82, (byte)234, (byte)234, (byte)30, (byte)29, (byte)204, (byte)148, (byte)26, (byte)39, (byte)96, (byte)6, (byte)235, (byte)13, (byte)246, (byte)133, (byte)215, (byte)199, (byte)199, (byte)65, (byte)87, (byte)94, (byte)78, (byte)185, (byte)125, (byte)67, (byte)23, (byte)247, (byte)83, (byte)238, (byte)14, (byte)165, (byte)87, (byte)198, (byte)233, (byte)140, (byte)57, (byte)60, (byte)37, (byte)37, (byte)175, (byte)32, (byte)60, (byte)170, (byte)159, (byte)65, (byte)204, (byte)253, (byte)234, (byte)122, (byte)214, (byte)122, (byte)76, (byte)98, (byte)217, (byte)52, (byte)75, (byte)11, (byte)212, (byte)206, (byte)184, (byte)182, (byte)224, (byte)25, (byte)84, (byte)63, (byte)14, (byte)177, (byte)80, (byte)169, (byte)200, (byte)247, (byte)247, (byte)34, (byte)21, (byte)86, (byte)220, (byte)192, (byte)168, (byte)137, (byte)173, (byte)136, (byte)131, (byte)233, (byte)183, (byte)30, (byte)20, (byte)38, (byte)43, (byte)127, (byte)245, (byte)146, (byte)20, (byte)99, (byte)16, (byte)146, (byte)51, (byte)216, (byte)84, (byte)90, (byte)214, (byte)26, (byte)5, (byte)210, (byte)34, (byte)61, (byte)17, (byte)115, (byte)122, (byte)225, (byte)72, (byte)86, (byte)209, (byte)204, (byte)84, (byte)237, (byte)117, (byte)250, (byte)23, (byte)96, (byte)175, (byte)204, (byte)77, (byte)18, (byte)95, (byte)55, (byte)1, (byte)56, (byte)215, (byte)27, (byte)99, (byte)55, (byte)87, (byte)121, (byte)145, (byte)122, (byte)39, (byte)55, (byte)65, (byte)109, (byte)182, (byte)79, (byte)181, (byte)165, (byte)54, (byte)156, (byte)53, (byte)219, (byte)253, (byte)74, (byte)73, (byte)161, (byte)43, (byte)21, (byte)105, (byte)212, (byte)113, (byte)237, (byte)91, (byte)237, (byte)109, (byte)100, (byte)5, (byte)34, (byte)130, (byte)242, (byte)19, (byte)40, (byte)16, (byte)16, (byte)120, (byte)90, (byte)129, (byte)101, (byte)142, (byte)249, (byte)35, (byte)244, (byte)63, (byte)132, (byte)48, (byte)199, (byte)154, (byte)143, (byte)79, (byte)21, (byte)78, (byte)98, (byte)28, (byte)188, (byte)8, (byte)198, (byte)79, (byte)124, (byte)215, (byte)29}, 0) ;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_distance == (ushort)(ushort)47290);
                Debug.Assert(pack.time_boot_ms == (uint)3735854752U);
                Debug.Assert(pack.orientation == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_225);
                Debug.Assert(pack.type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
                Debug.Assert(pack.current_distance == (ushort)(ushort)23660);
                Debug.Assert(pack.max_distance == (ushort)(ushort)53969);
                Debug.Assert(pack.id == (byte)(byte)13);
                Debug.Assert(pack.covariance == (byte)(byte)134);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.max_distance = (ushort)(ushort)53969;
            p132.id = (byte)(byte)13;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN;
            p132.covariance = (byte)(byte)134;
            p132.current_distance = (ushort)(ushort)23660;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_225;
            p132.min_distance = (ushort)(ushort)47290;
            p132.time_boot_ms = (uint)3735854752U;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mask == (ulong)2953603547606503408L);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)38688);
                Debug.Assert(pack.lon == (int)519565070);
                Debug.Assert(pack.lat == (int)1657471421);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.grid_spacing = (ushort)(ushort)38688;
            p133.mask = (ulong)2953603547606503408L;
            p133.lat = (int)1657471421;
            p133.lon = (int)519565070;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -465788084);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -26532, (short)2828, (short)32118, (short)13677, (short)8004, (short)32430, (short)9063, (short) -30592, (short) -4691, (short) -2975, (short)21502, (short)19987, (short)29757, (short) -14398, (short) -25072, (short) -5929}));
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)55190);
                Debug.Assert(pack.lat == (int)2133477632);
                Debug.Assert(pack.gridbit == (byte)(byte)239);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.grid_spacing = (ushort)(ushort)55190;
            p134.lat = (int)2133477632;
            p134.gridbit = (byte)(byte)239;
            p134.lon = (int) -465788084;
            p134.data__SET(new short[] {(short) -26532, (short)2828, (short)32118, (short)13677, (short)8004, (short)32430, (short)9063, (short) -30592, (short) -4691, (short) -2975, (short)21502, (short)19987, (short)29757, (short) -14398, (short) -25072, (short) -5929}, 0) ;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1822504752);
                Debug.Assert(pack.lon == (int) -1217666322);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lon = (int) -1217666322;
            p135.lat = (int)1822504752;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -220471405);
                Debug.Assert(pack.loaded == (ushort)(ushort)58087);
                Debug.Assert(pack.pending == (ushort)(ushort)30485);
                Debug.Assert(pack.spacing == (ushort)(ushort)39187);
                Debug.Assert(pack.current_height == (float)5.6943783E37F);
                Debug.Assert(pack.lon == (int)1072853737);
                Debug.Assert(pack.terrain_height == (float)3.6191693E37F);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.current_height = (float)5.6943783E37F;
            p136.pending = (ushort)(ushort)30485;
            p136.spacing = (ushort)(ushort)39187;
            p136.lat = (int) -220471405;
            p136.lon = (int)1072853737;
            p136.loaded = (ushort)(ushort)58087;
            p136.terrain_height = (float)3.6191693E37F;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float)2.3608062E38F);
                Debug.Assert(pack.press_diff == (float)2.1596795E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4137188386U);
                Debug.Assert(pack.temperature == (short)(short) -14540);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.press_abs = (float)2.3608062E38F;
            p137.press_diff = (float)2.1596795E38F;
            p137.temperature = (short)(short) -14540;
            p137.time_boot_ms = (uint)4137188386U;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {-5.189008E37F, -2.6871231E38F, 3.184791E38F, 6.7591267E37F}));
                Debug.Assert(pack.time_usec == (ulong)7270997561365334509L);
                Debug.Assert(pack.x == (float)3.599361E37F);
                Debug.Assert(pack.z == (float) -1.5464491E38F);
                Debug.Assert(pack.y == (float)2.593864E38F);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.q_SET(new float[] {-5.189008E37F, -2.6871231E38F, 3.184791E38F, 6.7591267E37F}, 0) ;
            p138.x = (float)3.599361E37F;
            p138.time_usec = (ulong)7270997561365334509L;
            p138.z = (float) -1.5464491E38F;
            p138.y = (float)2.593864E38F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)246);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-3.3069337E38F, -2.40241E38F, -7.6637855E37F, 5.9167095E37F, 1.1105901E37F, 4.874259E37F, 2.6678616E38F, 2.8741128E38F}));
                Debug.Assert(pack.time_usec == (ulong)7088160163669616057L);
                Debug.Assert(pack.target_system == (byte)(byte)157);
                Debug.Assert(pack.group_mlx == (byte)(byte)5);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.target_system = (byte)(byte)157;
            p139.target_component = (byte)(byte)246;
            p139.controls_SET(new float[] {-3.3069337E38F, -2.40241E38F, -7.6637855E37F, 5.9167095E37F, 1.1105901E37F, 4.874259E37F, 2.6678616E38F, 2.8741128E38F}, 0) ;
            p139.time_usec = (ulong)7088160163669616057L;
            p139.group_mlx = (byte)(byte)5;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.group_mlx == (byte)(byte)203);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {2.1417815E38F, 8.073801E37F, 1.6815415E38F, -5.6381E36F, -2.4314725E38F, 1.5901381E38F, 7.356704E37F, 1.3961225E38F}));
                Debug.Assert(pack.time_usec == (ulong)7263788876116888825L);
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.group_mlx = (byte)(byte)203;
            p140.time_usec = (ulong)7263788876116888825L;
            p140.controls_SET(new float[] {2.1417815E38F, 8.073801E37F, 1.6815415E38F, -5.6381E36F, -2.4314725E38F, 1.5901381E38F, 7.356704E37F, 1.3961225E38F}, 0) ;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_terrain == (float)7.1364875E37F);
                Debug.Assert(pack.altitude_monotonic == (float)3.160173E38F);
                Debug.Assert(pack.time_usec == (ulong)5797590761120275376L);
                Debug.Assert(pack.altitude_relative == (float)2.3347396E38F);
                Debug.Assert(pack.altitude_amsl == (float)2.714212E38F);
                Debug.Assert(pack.bottom_clearance == (float) -2.6053988E38F);
                Debug.Assert(pack.altitude_local == (float)9.147572E37F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_terrain = (float)7.1364875E37F;
            p141.time_usec = (ulong)5797590761120275376L;
            p141.bottom_clearance = (float) -2.6053988E38F;
            p141.altitude_relative = (float)2.3347396E38F;
            p141.altitude_monotonic = (float)3.160173E38F;
            p141.altitude_local = (float)9.147572E37F;
            p141.altitude_amsl = (float)2.714212E38F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.request_id == (byte)(byte)185);
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)127, (byte)5, (byte)6, (byte)190, (byte)184, (byte)96, (byte)255, (byte)133, (byte)200, (byte)21, (byte)180, (byte)100, (byte)169, (byte)153, (byte)202, (byte)97, (byte)96, (byte)82, (byte)219, (byte)87, (byte)186, (byte)45, (byte)0, (byte)115, (byte)40, (byte)162, (byte)235, (byte)155, (byte)156, (byte)28, (byte)192, (byte)140, (byte)140, (byte)207, (byte)27, (byte)188, (byte)83, (byte)98, (byte)218, (byte)34, (byte)178, (byte)242, (byte)25, (byte)75, (byte)7, (byte)27, (byte)27, (byte)74, (byte)130, (byte)119, (byte)157, (byte)61, (byte)40, (byte)57, (byte)67, (byte)107, (byte)123, (byte)67, (byte)6, (byte)142, (byte)27, (byte)185, (byte)34, (byte)103, (byte)58, (byte)73, (byte)11, (byte)192, (byte)157, (byte)233, (byte)40, (byte)12, (byte)35, (byte)158, (byte)54, (byte)170, (byte)88, (byte)163, (byte)91, (byte)135, (byte)77, (byte)149, (byte)20, (byte)58, (byte)186, (byte)107, (byte)50, (byte)185, (byte)85, (byte)248, (byte)17, (byte)60, (byte)116, (byte)215, (byte)46, (byte)159, (byte)212, (byte)8, (byte)147, (byte)154, (byte)42, (byte)87, (byte)176, (byte)212, (byte)76, (byte)129, (byte)220, (byte)105, (byte)152, (byte)225, (byte)186, (byte)191, (byte)214, (byte)70, (byte)166, (byte)52, (byte)155, (byte)214, (byte)61, (byte)149}));
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)96, (byte)154, (byte)198, (byte)184, (byte)36, (byte)101, (byte)224, (byte)10, (byte)93, (byte)44, (byte)231, (byte)147, (byte)250, (byte)162, (byte)28, (byte)245, (byte)245, (byte)32, (byte)238, (byte)19, (byte)2, (byte)76, (byte)69, (byte)78, (byte)236, (byte)243, (byte)105, (byte)124, (byte)192, (byte)152, (byte)116, (byte)195, (byte)31, (byte)100, (byte)252, (byte)60, (byte)199, (byte)145, (byte)108, (byte)118, (byte)13, (byte)62, (byte)231, (byte)219, (byte)83, (byte)46, (byte)197, (byte)136, (byte)216, (byte)248, (byte)35, (byte)112, (byte)254, (byte)37, (byte)18, (byte)155, (byte)109, (byte)127, (byte)95, (byte)221, (byte)115, (byte)229, (byte)111, (byte)105, (byte)255, (byte)159, (byte)74, (byte)119, (byte)178, (byte)203, (byte)225, (byte)191, (byte)175, (byte)72, (byte)185, (byte)233, (byte)151, (byte)139, (byte)133, (byte)98, (byte)128, (byte)138, (byte)157, (byte)63, (byte)243, (byte)166, (byte)233, (byte)13, (byte)184, (byte)50, (byte)164, (byte)46, (byte)108, (byte)211, (byte)64, (byte)121, (byte)25, (byte)22, (byte)129, (byte)211, (byte)253, (byte)26, (byte)32, (byte)13, (byte)12, (byte)109, (byte)167, (byte)102, (byte)10, (byte)115, (byte)41, (byte)231, (byte)135, (byte)181, (byte)154, (byte)226, (byte)10, (byte)172, (byte)209, (byte)248}));
                Debug.Assert(pack.transfer_type == (byte)(byte)39);
                Debug.Assert(pack.uri_type == (byte)(byte)169);
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)185;
            p142.uri_type = (byte)(byte)169;
            p142.transfer_type = (byte)(byte)39;
            p142.storage_SET(new byte[] {(byte)127, (byte)5, (byte)6, (byte)190, (byte)184, (byte)96, (byte)255, (byte)133, (byte)200, (byte)21, (byte)180, (byte)100, (byte)169, (byte)153, (byte)202, (byte)97, (byte)96, (byte)82, (byte)219, (byte)87, (byte)186, (byte)45, (byte)0, (byte)115, (byte)40, (byte)162, (byte)235, (byte)155, (byte)156, (byte)28, (byte)192, (byte)140, (byte)140, (byte)207, (byte)27, (byte)188, (byte)83, (byte)98, (byte)218, (byte)34, (byte)178, (byte)242, (byte)25, (byte)75, (byte)7, (byte)27, (byte)27, (byte)74, (byte)130, (byte)119, (byte)157, (byte)61, (byte)40, (byte)57, (byte)67, (byte)107, (byte)123, (byte)67, (byte)6, (byte)142, (byte)27, (byte)185, (byte)34, (byte)103, (byte)58, (byte)73, (byte)11, (byte)192, (byte)157, (byte)233, (byte)40, (byte)12, (byte)35, (byte)158, (byte)54, (byte)170, (byte)88, (byte)163, (byte)91, (byte)135, (byte)77, (byte)149, (byte)20, (byte)58, (byte)186, (byte)107, (byte)50, (byte)185, (byte)85, (byte)248, (byte)17, (byte)60, (byte)116, (byte)215, (byte)46, (byte)159, (byte)212, (byte)8, (byte)147, (byte)154, (byte)42, (byte)87, (byte)176, (byte)212, (byte)76, (byte)129, (byte)220, (byte)105, (byte)152, (byte)225, (byte)186, (byte)191, (byte)214, (byte)70, (byte)166, (byte)52, (byte)155, (byte)214, (byte)61, (byte)149}, 0) ;
            p142.uri_SET(new byte[] {(byte)96, (byte)154, (byte)198, (byte)184, (byte)36, (byte)101, (byte)224, (byte)10, (byte)93, (byte)44, (byte)231, (byte)147, (byte)250, (byte)162, (byte)28, (byte)245, (byte)245, (byte)32, (byte)238, (byte)19, (byte)2, (byte)76, (byte)69, (byte)78, (byte)236, (byte)243, (byte)105, (byte)124, (byte)192, (byte)152, (byte)116, (byte)195, (byte)31, (byte)100, (byte)252, (byte)60, (byte)199, (byte)145, (byte)108, (byte)118, (byte)13, (byte)62, (byte)231, (byte)219, (byte)83, (byte)46, (byte)197, (byte)136, (byte)216, (byte)248, (byte)35, (byte)112, (byte)254, (byte)37, (byte)18, (byte)155, (byte)109, (byte)127, (byte)95, (byte)221, (byte)115, (byte)229, (byte)111, (byte)105, (byte)255, (byte)159, (byte)74, (byte)119, (byte)178, (byte)203, (byte)225, (byte)191, (byte)175, (byte)72, (byte)185, (byte)233, (byte)151, (byte)139, (byte)133, (byte)98, (byte)128, (byte)138, (byte)157, (byte)63, (byte)243, (byte)166, (byte)233, (byte)13, (byte)184, (byte)50, (byte)164, (byte)46, (byte)108, (byte)211, (byte)64, (byte)121, (byte)25, (byte)22, (byte)129, (byte)211, (byte)253, (byte)26, (byte)32, (byte)13, (byte)12, (byte)109, (byte)167, (byte)102, (byte)10, (byte)115, (byte)41, (byte)231, (byte)135, (byte)181, (byte)154, (byte)226, (byte)10, (byte)172, (byte)209, (byte)248}, 0) ;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2438193661U);
                Debug.Assert(pack.temperature == (short)(short)26687);
                Debug.Assert(pack.press_abs == (float)3.2353695E38F);
                Debug.Assert(pack.press_diff == (float) -2.2157798E38F);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.press_diff = (float) -2.2157798E38F;
            p143.press_abs = (float)3.2353695E38F;
            p143.temperature = (short)(short)26687;
            p143.time_boot_ms = (uint)2438193661U;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-1.9337515E38F, -1.7863645E38F, -1.3035166E38F, 5.8149395E37F}));
                Debug.Assert(pack.est_capabilities == (byte)(byte)40);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-3.9017423E36F, -1.579282E38F, 2.0252822E38F}));
                Debug.Assert(pack.acc.SequenceEqual(new float[] {2.207607E38F, -1.813178E38F, -8.614779E37F}));
                Debug.Assert(pack.alt == (float)1.2780669E38F);
                Debug.Assert(pack.custom_state == (ulong)5526910530640388102L);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {9.563241E37F, -2.5375613E38F, -2.7704311E38F}));
                Debug.Assert(pack.lat == (int)1133276619);
                Debug.Assert(pack.timestamp == (ulong)4953714322495492775L);
                Debug.Assert(pack.lon == (int) -1039423749);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-1.6687136E38F, -1.4971904E38F, 3.2326305E38F}));
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.lat = (int)1133276619;
            p144.position_cov_SET(new float[] {9.563241E37F, -2.5375613E38F, -2.7704311E38F}, 0) ;
            p144.est_capabilities = (byte)(byte)40;
            p144.vel_SET(new float[] {-1.6687136E38F, -1.4971904E38F, 3.2326305E38F}, 0) ;
            p144.custom_state = (ulong)5526910530640388102L;
            p144.attitude_q_SET(new float[] {-1.9337515E38F, -1.7863645E38F, -1.3035166E38F, 5.8149395E37F}, 0) ;
            p144.rates_SET(new float[] {-3.9017423E36F, -1.579282E38F, 2.0252822E38F}, 0) ;
            p144.acc_SET(new float[] {2.207607E38F, -1.813178E38F, -8.614779E37F}, 0) ;
            p144.lon = (int) -1039423749;
            p144.timestamp = (ulong)4953714322495492775L;
            p144.alt = (float)1.2780669E38F;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z_pos == (float) -9.238708E37F);
                Debug.Assert(pack.y_pos == (float)3.0138527E38F);
                Debug.Assert(pack.x_acc == (float) -1.1913336E38F);
                Debug.Assert(pack.z_acc == (float) -2.0801017E38F);
                Debug.Assert(pack.x_pos == (float)2.0141218E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {-7.9878296E36F, 2.2040602E38F, 6.8983497E37F}));
                Debug.Assert(pack.airspeed == (float) -1.5565084E38F);
                Debug.Assert(pack.x_vel == (float)1.2271237E38F);
                Debug.Assert(pack.pitch_rate == (float)3.234448E38F);
                Debug.Assert(pack.y_vel == (float)3.3743492E37F);
                Debug.Assert(pack.y_acc == (float) -2.8223184E38F);
                Debug.Assert(pack.z_vel == (float)2.0993844E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {1.3276077E38F, -5.782139E37F, -1.87363E38F}));
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.857889E38F, -1.491011E38F, 2.5011136E38F, 1.5053894E38F}));
                Debug.Assert(pack.time_usec == (ulong)3386951965883996914L);
                Debug.Assert(pack.yaw_rate == (float)1.1403597E38F);
                Debug.Assert(pack.roll_rate == (float) -2.7544027E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.yaw_rate = (float)1.1403597E38F;
            p146.pos_variance_SET(new float[] {-7.9878296E36F, 2.2040602E38F, 6.8983497E37F}, 0) ;
            p146.x_pos = (float)2.0141218E38F;
            p146.x_acc = (float) -1.1913336E38F;
            p146.z_acc = (float) -2.0801017E38F;
            p146.x_vel = (float)1.2271237E38F;
            p146.pitch_rate = (float)3.234448E38F;
            p146.time_usec = (ulong)3386951965883996914L;
            p146.z_pos = (float) -9.238708E37F;
            p146.y_vel = (float)3.3743492E37F;
            p146.airspeed = (float) -1.5565084E38F;
            p146.y_acc = (float) -2.8223184E38F;
            p146.y_pos = (float)3.0138527E38F;
            p146.z_vel = (float)2.0993844E38F;
            p146.roll_rate = (float) -2.7544027E38F;
            p146.q_SET(new float[] {-2.857889E38F, -1.491011E38F, 2.5011136E38F, 1.5053894E38F}, 0) ;
            p146.vel_variance_SET(new float[] {1.3276077E38F, -5.782139E37F, -1.87363E38F}, 0) ;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.energy_consumed == (int) -1849555696);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)33657, (ushort)15849, (ushort)32572, (ushort)38557, (ushort)57500, (ushort)53917, (ushort)53002, (ushort)28733, (ushort)10088, (ushort)39499}));
                Debug.Assert(pack.current_battery == (short)(short)15374);
                Debug.Assert(pack.id == (byte)(byte)77);
                Debug.Assert(pack.battery_function == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 1);
                Debug.Assert(pack.type == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION);
                Debug.Assert(pack.current_consumed == (int) -329393096);
                Debug.Assert(pack.temperature == (short)(short) -31424);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN;
            p147.current_consumed = (int) -329393096;
            p147.battery_remaining = (sbyte)(sbyte) - 1;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION;
            p147.id = (byte)(byte)77;
            p147.current_battery = (short)(short)15374;
            p147.energy_consumed = (int) -1849555696;
            p147.voltages_SET(new ushort[] {(ushort)33657, (ushort)15849, (ushort)32572, (ushort)38557, (ushort)57500, (ushort)53917, (ushort)53002, (ushort)28733, (ushort)10088, (ushort)39499}, 0) ;
            p147.temperature = (short)(short) -31424;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uid == (ulong)68834325428001896L);
                Debug.Assert(pack.product_id == (ushort)(ushort)25990);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)125, (byte)12, (byte)83, (byte)190, (byte)98, (byte)180, (byte)75, (byte)46}));
                Debug.Assert(pack.flight_sw_version == (uint)110798225U);
                Debug.Assert(pack.middleware_sw_version == (uint)2481526743U);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE));
                Debug.Assert(pack.vendor_id == (ushort)(ushort)29362);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)7, (byte)3, (byte)5, (byte)119, (byte)120, (byte)127, (byte)181, (byte)248, (byte)237, (byte)165, (byte)55, (byte)178, (byte)203, (byte)86, (byte)127, (byte)69, (byte)14, (byte)80}));
                Debug.Assert(pack.os_sw_version == (uint)2461478439U);
                Debug.Assert(pack.board_version == (uint)3417582752U);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)61, (byte)11, (byte)37, (byte)34, (byte)17, (byte)225, (byte)163, (byte)159}));
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)226, (byte)93, (byte)89, (byte)88, (byte)142, (byte)126, (byte)144, (byte)210}));
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.middleware_custom_version_SET(new byte[] {(byte)61, (byte)11, (byte)37, (byte)34, (byte)17, (byte)225, (byte)163, (byte)159}, 0) ;
            p148.uid2_SET(new byte[] {(byte)7, (byte)3, (byte)5, (byte)119, (byte)120, (byte)127, (byte)181, (byte)248, (byte)237, (byte)165, (byte)55, (byte)178, (byte)203, (byte)86, (byte)127, (byte)69, (byte)14, (byte)80}, 0, PH) ;
            p148.board_version = (uint)3417582752U;
            p148.vendor_id = (ushort)(ushort)29362;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE);
            p148.flight_sw_version = (uint)110798225U;
            p148.middleware_sw_version = (uint)2481526743U;
            p148.os_sw_version = (uint)2461478439U;
            p148.os_custom_version_SET(new byte[] {(byte)226, (byte)93, (byte)89, (byte)88, (byte)142, (byte)126, (byte)144, (byte)210}, 0) ;
            p148.flight_custom_version_SET(new byte[] {(byte)125, (byte)12, (byte)83, (byte)190, (byte)98, (byte)180, (byte)75, (byte)46}, 0) ;
            p148.product_id = (ushort)(ushort)25990;
            p148.uid = (ulong)68834325428001896L;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.angle_x == (float) -1.9740287E38F);
                Debug.Assert(pack.z_TRY(ph) == (float)1.9509723E38F);
                Debug.Assert(pack.target_num == (byte)(byte)37);
                Debug.Assert(pack.type == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
                Debug.Assert(pack.size_y == (float)3.4774207E36F);
                Debug.Assert(pack.size_x == (float)2.617815E38F);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {2.615017E38F, 2.1083972E38F, -1.0540883E38F, -1.5511814E38F}));
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)154);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.angle_y == (float) -1.9352037E38F);
                Debug.Assert(pack.time_usec == (ulong)3107585546543333524L);
                Debug.Assert(pack.distance == (float)6.582582E37F);
                Debug.Assert(pack.y_TRY(ph) == (float)2.0817969E38F);
                Debug.Assert(pack.x_TRY(ph) == (float)8.119567E37F);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.size_y = (float)3.4774207E36F;
            p149.z_SET((float)1.9509723E38F, PH) ;
            p149.position_valid_SET((byte)(byte)154, PH) ;
            p149.time_usec = (ulong)3107585546543333524L;
            p149.angle_x = (float) -1.9740287E38F;
            p149.frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p149.y_SET((float)2.0817969E38F, PH) ;
            p149.angle_y = (float) -1.9352037E38F;
            p149.target_num = (byte)(byte)37;
            p149.size_x = (float)2.617815E38F;
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.distance = (float)6.582582E37F;
            p149.x_SET((float)8.119567E37F, PH) ;
            p149.q_SET(new float[] {2.615017E38F, 2.1083972E38F, -1.0540883E38F, -1.5511814E38F}, 0, PH) ;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENS_POWERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.adc121_vspb_volt == (float)2.917252E38F);
                Debug.Assert(pack.adc121_cspb_amp == (float)5.8735057E36F);
                Debug.Assert(pack.adc121_cs1_amp == (float) -9.730814E36F);
                Debug.Assert(pack.adc121_cs2_amp == (float) -1.9326942E38F);
            };
            GroundControl.SENS_POWER p201 = CommunicationChannel.new_SENS_POWER();
            PH.setPack(p201);
            p201.adc121_vspb_volt = (float)2.917252E38F;
            p201.adc121_cspb_amp = (float)5.8735057E36F;
            p201.adc121_cs2_amp = (float) -1.9326942E38F;
            p201.adc121_cs1_amp = (float) -9.730814E36F;
            CommunicationChannel.instance.send(p201);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENS_MPPTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mppt3_pwm == (ushort)(ushort)35151);
                Debug.Assert(pack.mppt1_status == (byte)(byte)91);
                Debug.Assert(pack.mppt2_amp == (float)1.2066939E38F);
                Debug.Assert(pack.mppt1_volt == (float) -5.757167E37F);
                Debug.Assert(pack.mppt3_volt == (float)3.1441315E38F);
                Debug.Assert(pack.mppt2_volt == (float)2.7628408E38F);
                Debug.Assert(pack.mppt_timestamp == (ulong)3036863990109340621L);
                Debug.Assert(pack.mppt2_pwm == (ushort)(ushort)16567);
                Debug.Assert(pack.mppt3_status == (byte)(byte)75);
                Debug.Assert(pack.mppt2_status == (byte)(byte)99);
                Debug.Assert(pack.mppt1_amp == (float)3.9236435E37F);
                Debug.Assert(pack.mppt3_amp == (float) -2.9260382E38F);
                Debug.Assert(pack.mppt1_pwm == (ushort)(ushort)4863);
            };
            GroundControl.SENS_MPPT p202 = CommunicationChannel.new_SENS_MPPT();
            PH.setPack(p202);
            p202.mppt1_status = (byte)(byte)91;
            p202.mppt3_status = (byte)(byte)75;
            p202.mppt2_status = (byte)(byte)99;
            p202.mppt_timestamp = (ulong)3036863990109340621L;
            p202.mppt2_amp = (float)1.2066939E38F;
            p202.mppt3_volt = (float)3.1441315E38F;
            p202.mppt2_volt = (float)2.7628408E38F;
            p202.mppt3_pwm = (ushort)(ushort)35151;
            p202.mppt3_amp = (float) -2.9260382E38F;
            p202.mppt1_amp = (float)3.9236435E37F;
            p202.mppt1_volt = (float) -5.757167E37F;
            p202.mppt1_pwm = (ushort)(ushort)4863;
            p202.mppt2_pwm = (ushort)(ushort)16567;
            CommunicationChannel.instance.send(p202);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnASLCTRL_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uThrot == (float) -1.7242368E38F);
                Debug.Assert(pack.h == (float) -6.1314196E37F);
                Debug.Assert(pack.PitchAngleRef == (float)3.0799731E38F);
                Debug.Assert(pack.uRud == (float) -2.2658356E38F);
                Debug.Assert(pack.nZ == (float)2.4190873E37F);
                Debug.Assert(pack.rRef == (float)6.494054E37F);
                Debug.Assert(pack.SpoilersEngaged == (byte)(byte)159);
                Debug.Assert(pack.hRef == (float)3.1508421E38F);
                Debug.Assert(pack.p == (float) -1.5102973E38F);
                Debug.Assert(pack.qRef == (float)2.0855418E38F);
                Debug.Assert(pack.timestamp == (ulong)2640203673242862948L);
                Debug.Assert(pack.YawAngle == (float) -3.9544736E37F);
                Debug.Assert(pack.uThrot2 == (float) -2.1624628E37F);
                Debug.Assert(pack.q == (float)1.9422249E38F);
                Debug.Assert(pack.YawAngleRef == (float)2.9624997E38F);
                Debug.Assert(pack.uElev == (float) -1.8976581E38F);
                Debug.Assert(pack.RollAngleRef == (float) -5.2564344E37F);
                Debug.Assert(pack.uAil == (float)5.4032294E37F);
                Debug.Assert(pack.r == (float) -2.3147844E38F);
                Debug.Assert(pack.hRef_t == (float)1.9747439E38F);
                Debug.Assert(pack.aslctrl_mode == (byte)(byte)87);
                Debug.Assert(pack.RollAngle == (float)9.686488E37F);
                Debug.Assert(pack.AirspeedRef == (float)3.362974E38F);
                Debug.Assert(pack.pRef == (float)3.3225528E38F);
                Debug.Assert(pack.PitchAngle == (float) -2.8818814E37F);
            };
            GroundControl.ASLCTRL_DATA p203 = CommunicationChannel.new_ASLCTRL_DATA();
            PH.setPack(p203);
            p203.p = (float) -1.5102973E38F;
            p203.h = (float) -6.1314196E37F;
            p203.PitchAngle = (float) -2.8818814E37F;
            p203.YawAngleRef = (float)2.9624997E38F;
            p203.uThrot2 = (float) -2.1624628E37F;
            p203.aslctrl_mode = (byte)(byte)87;
            p203.timestamp = (ulong)2640203673242862948L;
            p203.uThrot = (float) -1.7242368E38F;
            p203.pRef = (float)3.3225528E38F;
            p203.PitchAngleRef = (float)3.0799731E38F;
            p203.hRef = (float)3.1508421E38F;
            p203.uElev = (float) -1.8976581E38F;
            p203.RollAngleRef = (float) -5.2564344E37F;
            p203.uAil = (float)5.4032294E37F;
            p203.q = (float)1.9422249E38F;
            p203.uRud = (float) -2.2658356E38F;
            p203.SpoilersEngaged = (byte)(byte)159;
            p203.rRef = (float)6.494054E37F;
            p203.hRef_t = (float)1.9747439E38F;
            p203.RollAngle = (float)9.686488E37F;
            p203.qRef = (float)2.0855418E38F;
            p203.YawAngle = (float) -3.9544736E37F;
            p203.nZ = (float)2.4190873E37F;
            p203.r = (float) -2.3147844E38F;
            p203.AirspeedRef = (float)3.362974E38F;
            CommunicationChannel.instance.send(p203);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnASLCTRL_DEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.f_4 == (float)3.1635985E38F);
                Debug.Assert(pack.i8_1 == (byte)(byte)88);
                Debug.Assert(pack.f_7 == (float) -2.8462886E37F);
                Debug.Assert(pack.f_5 == (float) -1.9668903E38F);
                Debug.Assert(pack.f_1 == (float) -2.692163E38F);
                Debug.Assert(pack.i32_1 == (uint)1994601177U);
                Debug.Assert(pack.i8_2 == (byte)(byte)213);
                Debug.Assert(pack.f_3 == (float) -3.1328547E38F);
                Debug.Assert(pack.f_2 == (float) -4.0560235E37F);
                Debug.Assert(pack.f_8 == (float) -1.7502716E38F);
                Debug.Assert(pack.f_6 == (float) -2.9757175E38F);
            };
            GroundControl.ASLCTRL_DEBUG p204 = CommunicationChannel.new_ASLCTRL_DEBUG();
            PH.setPack(p204);
            p204.i8_1 = (byte)(byte)88;
            p204.i8_2 = (byte)(byte)213;
            p204.f_3 = (float) -3.1328547E38F;
            p204.f_2 = (float) -4.0560235E37F;
            p204.f_5 = (float) -1.9668903E38F;
            p204.i32_1 = (uint)1994601177U;
            p204.f_4 = (float)3.1635985E38F;
            p204.f_1 = (float) -2.692163E38F;
            p204.f_7 = (float) -2.8462886E37F;
            p204.f_6 = (float) -2.9757175E38F;
            p204.f_8 = (float) -1.7502716E38F;
            CommunicationChannel.instance.send(p204);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnASLUAV_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Motor_rpm == (float) -4.8266674E37F);
                Debug.Assert(pack.SATCOM_status == (byte)(byte)131);
                Debug.Assert(pack.Servo_status.SequenceEqual(new byte[] {(byte)223, (byte)56, (byte)39, (byte)154, (byte)39, (byte)93, (byte)199, (byte)125}));
                Debug.Assert(pack.LED_status == (byte)(byte)139);
            };
            GroundControl.ASLUAV_STATUS p205 = CommunicationChannel.new_ASLUAV_STATUS();
            PH.setPack(p205);
            p205.LED_status = (byte)(byte)139;
            p205.Servo_status_SET(new byte[] {(byte)223, (byte)56, (byte)39, (byte)154, (byte)39, (byte)93, (byte)199, (byte)125}, 0) ;
            p205.Motor_rpm = (float) -4.8266674E37F;
            p205.SATCOM_status = (byte)(byte)131;
            CommunicationChannel.instance.send(p205);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnEKF_EXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Airspeed == (float) -2.189868E38F);
                Debug.Assert(pack.alpha == (float)6.120866E36F);
                Debug.Assert(pack.WindDir == (float) -5.632869E37F);
                Debug.Assert(pack.Windspeed == (float) -1.3910526E37F);
                Debug.Assert(pack.WindZ == (float) -9.289958E37F);
                Debug.Assert(pack.timestamp == (ulong)7879885964115836863L);
                Debug.Assert(pack.beta == (float)3.2316117E38F);
            };
            GroundControl.EKF_EXT p206 = CommunicationChannel.new_EKF_EXT();
            PH.setPack(p206);
            p206.alpha = (float)6.120866E36F;
            p206.WindDir = (float) -5.632869E37F;
            p206.Windspeed = (float) -1.3910526E37F;
            p206.timestamp = (ulong)7879885964115836863L;
            p206.Airspeed = (float) -2.189868E38F;
            p206.WindZ = (float) -9.289958E37F;
            p206.beta = (float)3.2316117E38F;
            CommunicationChannel.instance.send(p206);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnASL_OBCTRLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.timestamp == (ulong)5667244320734693510L);
                Debug.Assert(pack.uThrot == (float) -2.3222635E38F);
                Debug.Assert(pack.uRud == (float) -1.9440183E38F);
                Debug.Assert(pack.uAilR == (float) -1.1250199E38F);
                Debug.Assert(pack.obctrl_status == (byte)(byte)118);
                Debug.Assert(pack.uThrot2 == (float)2.4405166E38F);
                Debug.Assert(pack.uAilL == (float)2.0402806E38F);
                Debug.Assert(pack.uElev == (float) -2.3342514E38F);
            };
            GroundControl.ASL_OBCTRL p207 = CommunicationChannel.new_ASL_OBCTRL();
            PH.setPack(p207);
            p207.uAilR = (float) -1.1250199E38F;
            p207.uElev = (float) -2.3342514E38F;
            p207.timestamp = (ulong)5667244320734693510L;
            p207.uThrot = (float) -2.3222635E38F;
            p207.uRud = (float) -1.9440183E38F;
            p207.obctrl_status = (byte)(byte)118;
            p207.uThrot2 = (float)2.4405166E38F;
            p207.uAilL = (float)2.0402806E38F;
            CommunicationChannel.instance.send(p207);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENS_ATMOSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Humidity == (float) -1.3163885E38F);
                Debug.Assert(pack.TempAmbient == (float)9.702463E37F);
            };
            GroundControl.SENS_ATMOS p208 = CommunicationChannel.new_SENS_ATMOS();
            PH.setPack(p208);
            p208.TempAmbient = (float)9.702463E37F;
            p208.Humidity = (float) -1.3163885E38F;
            CommunicationChannel.instance.send(p208);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENS_BATMONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cellvoltage1 == (ushort)(ushort)63446);
                Debug.Assert(pack.cellvoltage5 == (ushort)(ushort)3415);
                Debug.Assert(pack.cellvoltage4 == (ushort)(ushort)13666);
                Debug.Assert(pack.temperature == (float)1.7234416E38F);
                Debug.Assert(pack.cellvoltage3 == (ushort)(ushort)10040);
                Debug.Assert(pack.serialnumber == (ushort)(ushort)63726);
                Debug.Assert(pack.cellvoltage6 == (ushort)(ushort)2424);
                Debug.Assert(pack.cellvoltage2 == (ushort)(ushort)56081);
                Debug.Assert(pack.batterystatus == (ushort)(ushort)3475);
                Debug.Assert(pack.voltage == (ushort)(ushort)9716);
                Debug.Assert(pack.hostfetcontrol == (ushort)(ushort)17004);
                Debug.Assert(pack.SoC == (byte)(byte)54);
                Debug.Assert(pack.current == (short)(short) -18312);
            };
            GroundControl.SENS_BATMON p209 = CommunicationChannel.new_SENS_BATMON();
            PH.setPack(p209);
            p209.cellvoltage5 = (ushort)(ushort)3415;
            p209.cellvoltage6 = (ushort)(ushort)2424;
            p209.batterystatus = (ushort)(ushort)3475;
            p209.hostfetcontrol = (ushort)(ushort)17004;
            p209.cellvoltage3 = (ushort)(ushort)10040;
            p209.cellvoltage1 = (ushort)(ushort)63446;
            p209.temperature = (float)1.7234416E38F;
            p209.SoC = (byte)(byte)54;
            p209.serialnumber = (ushort)(ushort)63726;
            p209.voltage = (ushort)(ushort)9716;
            p209.cellvoltage2 = (ushort)(ushort)56081;
            p209.current = (short)(short) -18312;
            p209.cellvoltage4 = (ushort)(ushort)13666;
            CommunicationChannel.instance.send(p209);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFW_SOARING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z2_DeltaRoll == (float)9.352763E37F);
                Debug.Assert(pack.z2_exp == (float) -2.709705E38F);
                Debug.Assert(pack.xLat == (float) -3.2913095E37F);
                Debug.Assert(pack.DistToSoarPoint == (float)2.16172E38F);
                Debug.Assert(pack.LoiterRadius == (float)3.2280631E38F);
                Debug.Assert(pack.ControlMode == (byte)(byte)26);
                Debug.Assert(pack.ThermalGSNorth == (float)1.5642976E38F);
                Debug.Assert(pack.valid == (byte)(byte)134);
                Debug.Assert(pack.DebugVar1 == (float) -1.4892672E38F);
                Debug.Assert(pack.LoiterDirection == (float) -5.0172916E37F);
                Debug.Assert(pack.xW == (float)3.1831437E38F);
                Debug.Assert(pack.ThermalGSEast == (float)2.3131358E38F);
                Debug.Assert(pack.xLon == (float) -2.1675012E38F);
                Debug.Assert(pack.DebugVar2 == (float)3.0746777E37F);
                Debug.Assert(pack.VarLat == (float) -1.5228953E38F);
                Debug.Assert(pack.timestampModeChanged == (ulong)9205293042295472772L);
                Debug.Assert(pack.VarW == (float)2.8077019E38F);
                Debug.Assert(pack.timestamp == (ulong)2130128272425568611L);
                Debug.Assert(pack.z1_LocalUpdraftSpeed == (float) -8.163955E37F);
                Debug.Assert(pack.VarLon == (float) -1.5602289E38F);
                Debug.Assert(pack.TSE_dot == (float)3.1604245E38F);
                Debug.Assert(pack.xR == (float) -3.8744194E36F);
                Debug.Assert(pack.z1_exp == (float)5.0829113E37F);
                Debug.Assert(pack.vSinkExp == (float)1.1359545E38F);
                Debug.Assert(pack.VarR == (float)7.029019E37F);
            };
            GroundControl.FW_SOARING_DATA p210 = CommunicationChannel.new_FW_SOARING_DATA();
            PH.setPack(p210);
            p210.VarR = (float)7.029019E37F;
            p210.DistToSoarPoint = (float)2.16172E38F;
            p210.xR = (float) -3.8744194E36F;
            p210.LoiterRadius = (float)3.2280631E38F;
            p210.VarW = (float)2.8077019E38F;
            p210.ControlMode = (byte)(byte)26;
            p210.xW = (float)3.1831437E38F;
            p210.z1_LocalUpdraftSpeed = (float) -8.163955E37F;
            p210.timestampModeChanged = (ulong)9205293042295472772L;
            p210.LoiterDirection = (float) -5.0172916E37F;
            p210.ThermalGSNorth = (float)1.5642976E38F;
            p210.VarLon = (float) -1.5602289E38F;
            p210.DebugVar2 = (float)3.0746777E37F;
            p210.timestamp = (ulong)2130128272425568611L;
            p210.vSinkExp = (float)1.1359545E38F;
            p210.z2_DeltaRoll = (float)9.352763E37F;
            p210.z2_exp = (float) -2.709705E38F;
            p210.VarLat = (float) -1.5228953E38F;
            p210.ThermalGSEast = (float)2.3131358E38F;
            p210.valid = (byte)(byte)134;
            p210.xLon = (float) -2.1675012E38F;
            p210.TSE_dot = (float)3.1604245E38F;
            p210.z1_exp = (float)5.0829113E37F;
            p210.xLat = (float) -3.2913095E37F;
            p210.DebugVar1 = (float) -1.4892672E38F;
            CommunicationChannel.instance.send(p210);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENSORPOD_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.recording_nodes_count == (byte)(byte)192);
                Debug.Assert(pack.free_space == (ushort)(ushort)35697);
                Debug.Assert(pack.timestamp == (ulong)3162035552295197376L);
                Debug.Assert(pack.visensor_rate_4 == (byte)(byte)59);
                Debug.Assert(pack.visensor_rate_3 == (byte)(byte)173);
                Debug.Assert(pack.visensor_rate_1 == (byte)(byte)105);
                Debug.Assert(pack.visensor_rate_2 == (byte)(byte)237);
                Debug.Assert(pack.cpu_temp == (byte)(byte)110);
            };
            GroundControl.SENSORPOD_STATUS p211 = CommunicationChannel.new_SENSORPOD_STATUS();
            PH.setPack(p211);
            p211.free_space = (ushort)(ushort)35697;
            p211.visensor_rate_4 = (byte)(byte)59;
            p211.recording_nodes_count = (byte)(byte)192;
            p211.visensor_rate_1 = (byte)(byte)105;
            p211.cpu_temp = (byte)(byte)110;
            p211.visensor_rate_2 = (byte)(byte)237;
            p211.timestamp = (ulong)3162035552295197376L;
            p211.visensor_rate_3 = (byte)(byte)173;
            CommunicationChannel.instance.send(p211);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSENS_POWER_BOARDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pwr_brd_status == (byte)(byte)49);
                Debug.Assert(pack.pwr_brd_servo_1_amp == (float)9.978235E37F);
                Debug.Assert(pack.timestamp == (ulong)3850042130545000937L);
                Debug.Assert(pack.pwr_brd_servo_volt == (float) -1.0087256E38F);
                Debug.Assert(pack.pwr_brd_mot_r_amp == (float) -2.6455917E38F);
                Debug.Assert(pack.pwr_brd_mot_l_amp == (float)1.8347005E38F);
                Debug.Assert(pack.pwr_brd_led_status == (byte)(byte)111);
                Debug.Assert(pack.pwr_brd_aux_amp == (float)2.7029061E38F);
                Debug.Assert(pack.pwr_brd_servo_2_amp == (float)2.857652E37F);
                Debug.Assert(pack.pwr_brd_servo_4_amp == (float)1.8053983E38F);
                Debug.Assert(pack.pwr_brd_servo_3_amp == (float)3.0490463E38F);
                Debug.Assert(pack.pwr_brd_system_volt == (float) -1.1686186E38F);
            };
            GroundControl.SENS_POWER_BOARD p212 = CommunicationChannel.new_SENS_POWER_BOARD();
            PH.setPack(p212);
            p212.pwr_brd_servo_4_amp = (float)1.8053983E38F;
            p212.pwr_brd_led_status = (byte)(byte)111;
            p212.pwr_brd_mot_r_amp = (float) -2.6455917E38F;
            p212.pwr_brd_status = (byte)(byte)49;
            p212.pwr_brd_servo_volt = (float) -1.0087256E38F;
            p212.pwr_brd_aux_amp = (float)2.7029061E38F;
            p212.pwr_brd_system_volt = (float) -1.1686186E38F;
            p212.pwr_brd_servo_1_amp = (float)9.978235E37F;
            p212.pwr_brd_servo_3_amp = (float)3.0490463E38F;
            p212.timestamp = (ulong)3850042130545000937L;
            p212.pwr_brd_mot_l_amp = (float)1.8347005E38F;
            p212.pwr_brd_servo_2_amp = (float)2.857652E37F;
            CommunicationChannel.instance.send(p212);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pos_horiz_accuracy == (float) -2.3304913E38F);
                Debug.Assert(pack.mag_ratio == (float)2.2221925E38F);
                Debug.Assert(pack.pos_vert_ratio == (float)2.4648184E38F);
                Debug.Assert(pack.time_usec == (ulong)1268332115685291819L);
                Debug.Assert(pack.vel_ratio == (float)2.466374E38F);
                Debug.Assert(pack.pos_horiz_ratio == (float) -7.1112836E37F);
                Debug.Assert(pack.pos_vert_accuracy == (float)2.5463145E38F);
                Debug.Assert(pack.tas_ratio == (float) -1.1120203E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT));
                Debug.Assert(pack.hagl_ratio == (float)8.0010775E37F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.pos_vert_accuracy = (float)2.5463145E38F;
            p230.tas_ratio = (float) -1.1120203E38F;
            p230.hagl_ratio = (float)8.0010775E37F;
            p230.pos_vert_ratio = (float)2.4648184E38F;
            p230.pos_horiz_ratio = (float) -7.1112836E37F;
            p230.time_usec = (ulong)1268332115685291819L;
            p230.pos_horiz_accuracy = (float) -2.3304913E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT);
            p230.mag_ratio = (float)2.2221925E38F;
            p230.vel_ratio = (float)2.466374E38F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wind_alt == (float) -2.3794688E38F);
                Debug.Assert(pack.var_horiz == (float)1.6509597E38F);
                Debug.Assert(pack.wind_y == (float)1.3085383E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -2.3372143E38F);
                Debug.Assert(pack.wind_x == (float)7.4243593E37F);
                Debug.Assert(pack.vert_accuracy == (float)2.445385E38F);
                Debug.Assert(pack.wind_z == (float) -1.9512422E38F);
                Debug.Assert(pack.var_vert == (float)2.8365216E38F);
                Debug.Assert(pack.time_usec == (ulong)6910686898661902211L);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.horiz_accuracy = (float) -2.3372143E38F;
            p231.wind_alt = (float) -2.3794688E38F;
            p231.wind_z = (float) -1.9512422E38F;
            p231.vert_accuracy = (float)2.445385E38F;
            p231.wind_y = (float)1.3085383E38F;
            p231.time_usec = (ulong)6910686898661902211L;
            p231.wind_x = (float)7.4243593E37F;
            p231.var_horiz = (float)1.6509597E38F;
            p231.var_vert = (float)2.8365216E38F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ve == (float)2.8114328E37F);
                Debug.Assert(pack.gps_id == (byte)(byte)110);
                Debug.Assert(pack.time_week_ms == (uint)3412367738U);
                Debug.Assert(pack.hdop == (float)7.158421E37F);
                Debug.Assert(pack.alt == (float) -1.1295965E38F);
                Debug.Assert(pack.lon == (int)875258721);
                Debug.Assert(pack.speed_accuracy == (float) -2.0224392E38F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY));
                Debug.Assert(pack.vd == (float) -1.5965812E38F);
                Debug.Assert(pack.vdop == (float)2.2962823E38F);
                Debug.Assert(pack.lat == (int) -45157801);
                Debug.Assert(pack.satellites_visible == (byte)(byte)59);
                Debug.Assert(pack.vert_accuracy == (float)9.496076E37F);
                Debug.Assert(pack.time_week == (ushort)(ushort)54310);
                Debug.Assert(pack.horiz_accuracy == (float)2.2076549E38F);
                Debug.Assert(pack.fix_type == (byte)(byte)120);
                Debug.Assert(pack.time_usec == (ulong)526445627029628842L);
                Debug.Assert(pack.vn == (float)3.2193405E38F);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.speed_accuracy = (float) -2.0224392E38F;
            p232.gps_id = (byte)(byte)110;
            p232.vdop = (float)2.2962823E38F;
            p232.vert_accuracy = (float)9.496076E37F;
            p232.vd = (float) -1.5965812E38F;
            p232.time_week = (ushort)(ushort)54310;
            p232.ve = (float)2.8114328E37F;
            p232.time_week_ms = (uint)3412367738U;
            p232.vn = (float)3.2193405E38F;
            p232.lat = (int) -45157801;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY);
            p232.time_usec = (ulong)526445627029628842L;
            p232.alt = (float) -1.1295965E38F;
            p232.fix_type = (byte)(byte)120;
            p232.lon = (int)875258721;
            p232.hdop = (float)7.158421E37F;
            p232.horiz_accuracy = (float)2.2076549E38F;
            p232.satellites_visible = (byte)(byte)59;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (byte)(byte)189);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)119, (byte)64, (byte)65, (byte)129, (byte)185, (byte)153, (byte)251, (byte)132, (byte)182, (byte)31, (byte)147, (byte)109, (byte)75, (byte)183, (byte)85, (byte)220, (byte)15, (byte)36, (byte)151, (byte)184, (byte)25, (byte)130, (byte)247, (byte)186, (byte)20, (byte)12, (byte)238, (byte)71, (byte)25, (byte)62, (byte)222, (byte)249, (byte)129, (byte)238, (byte)61, (byte)125, (byte)10, (byte)125, (byte)54, (byte)117, (byte)242, (byte)126, (byte)185, (byte)254, (byte)8, (byte)131, (byte)198, (byte)227, (byte)128, (byte)211, (byte)45, (byte)204, (byte)120, (byte)216, (byte)93, (byte)13, (byte)74, (byte)6, (byte)193, (byte)238, (byte)32, (byte)204, (byte)203, (byte)127, (byte)87, (byte)19, (byte)83, (byte)119, (byte)64, (byte)128, (byte)180, (byte)157, (byte)81, (byte)152, (byte)99, (byte)141, (byte)102, (byte)27, (byte)174, (byte)198, (byte)96, (byte)40, (byte)104, (byte)37, (byte)65, (byte)218, (byte)140, (byte)251, (byte)246, (byte)245, (byte)8, (byte)158, (byte)1, (byte)145, (byte)2, (byte)12, (byte)96, (byte)55, (byte)72, (byte)241, (byte)61, (byte)204, (byte)2, (byte)200, (byte)173, (byte)220, (byte)138, (byte)156, (byte)16, (byte)83, (byte)183, (byte)10, (byte)155, (byte)50, (byte)245, (byte)42, (byte)188, (byte)165, (byte)215, (byte)186, (byte)255, (byte)200, (byte)22, (byte)85, (byte)135, (byte)8, (byte)24, (byte)44, (byte)179, (byte)254, (byte)3, (byte)236, (byte)67, (byte)33, (byte)34, (byte)69, (byte)127, (byte)4, (byte)89, (byte)160, (byte)19, (byte)118, (byte)62, (byte)224, (byte)23, (byte)230, (byte)68, (byte)88, (byte)22, (byte)21, (byte)2, (byte)1, (byte)63, (byte)30, (byte)216, (byte)217, (byte)62, (byte)205, (byte)185, (byte)32, (byte)135, (byte)62, (byte)196, (byte)232, (byte)185, (byte)208, (byte)93, (byte)106, (byte)189, (byte)105, (byte)234, (byte)224, (byte)4, (byte)229, (byte)134, (byte)232, (byte)171, (byte)32, (byte)194, (byte)233}));
                Debug.Assert(pack.len == (byte)(byte)106);
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.len = (byte)(byte)106;
            p233.flags = (byte)(byte)189;
            p233.data__SET(new byte[] {(byte)119, (byte)64, (byte)65, (byte)129, (byte)185, (byte)153, (byte)251, (byte)132, (byte)182, (byte)31, (byte)147, (byte)109, (byte)75, (byte)183, (byte)85, (byte)220, (byte)15, (byte)36, (byte)151, (byte)184, (byte)25, (byte)130, (byte)247, (byte)186, (byte)20, (byte)12, (byte)238, (byte)71, (byte)25, (byte)62, (byte)222, (byte)249, (byte)129, (byte)238, (byte)61, (byte)125, (byte)10, (byte)125, (byte)54, (byte)117, (byte)242, (byte)126, (byte)185, (byte)254, (byte)8, (byte)131, (byte)198, (byte)227, (byte)128, (byte)211, (byte)45, (byte)204, (byte)120, (byte)216, (byte)93, (byte)13, (byte)74, (byte)6, (byte)193, (byte)238, (byte)32, (byte)204, (byte)203, (byte)127, (byte)87, (byte)19, (byte)83, (byte)119, (byte)64, (byte)128, (byte)180, (byte)157, (byte)81, (byte)152, (byte)99, (byte)141, (byte)102, (byte)27, (byte)174, (byte)198, (byte)96, (byte)40, (byte)104, (byte)37, (byte)65, (byte)218, (byte)140, (byte)251, (byte)246, (byte)245, (byte)8, (byte)158, (byte)1, (byte)145, (byte)2, (byte)12, (byte)96, (byte)55, (byte)72, (byte)241, (byte)61, (byte)204, (byte)2, (byte)200, (byte)173, (byte)220, (byte)138, (byte)156, (byte)16, (byte)83, (byte)183, (byte)10, (byte)155, (byte)50, (byte)245, (byte)42, (byte)188, (byte)165, (byte)215, (byte)186, (byte)255, (byte)200, (byte)22, (byte)85, (byte)135, (byte)8, (byte)24, (byte)44, (byte)179, (byte)254, (byte)3, (byte)236, (byte)67, (byte)33, (byte)34, (byte)69, (byte)127, (byte)4, (byte)89, (byte)160, (byte)19, (byte)118, (byte)62, (byte)224, (byte)23, (byte)230, (byte)68, (byte)88, (byte)22, (byte)21, (byte)2, (byte)1, (byte)63, (byte)30, (byte)216, (byte)217, (byte)62, (byte)205, (byte)185, (byte)32, (byte)135, (byte)62, (byte)196, (byte)232, (byte)185, (byte)208, (byte)93, (byte)106, (byte)189, (byte)105, (byte)234, (byte)224, (byte)4, (byte)229, (byte)134, (byte)232, (byte)171, (byte)32, (byte)194, (byte)233}, 0) ;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_amsl == (short)(short) -4047);
                Debug.Assert(pack.roll == (short)(short)25742);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)77);
                Debug.Assert(pack.gps_nsat == (byte)(byte)198);
                Debug.Assert(pack.latitude == (int) -90890437);
                Debug.Assert(pack.wp_num == (byte)(byte)126);
                Debug.Assert(pack.altitude_sp == (short)(short) -18320);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
                Debug.Assert(pack.longitude == (int)717674267);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)2226);
                Debug.Assert(pack.failsafe == (byte)(byte)38);
                Debug.Assert(pack.temperature == (sbyte)(sbyte)101);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte)80);
                Debug.Assert(pack.heading == (ushort)(ushort)62343);
                Debug.Assert(pack.gps_fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
                Debug.Assert(pack.heading_sp == (short)(short) -2595);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED));
                Debug.Assert(pack.pitch == (short)(short)7980);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)19);
                Debug.Assert(pack.groundspeed == (byte)(byte)231);
                Debug.Assert(pack.battery_remaining == (byte)(byte)31);
                Debug.Assert(pack.airspeed == (byte)(byte)239);
                Debug.Assert(pack.throttle == (sbyte)(sbyte) - 60);
                Debug.Assert(pack.custom_mode == (uint)2811457042U);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.heading_sp = (short)(short) -2595;
            p234.custom_mode = (uint)2811457042U;
            p234.failsafe = (byte)(byte)38;
            p234.wp_distance = (ushort)(ushort)2226;
            p234.latitude = (int) -90890437;
            p234.gps_nsat = (byte)(byte)198;
            p234.groundspeed = (byte)(byte)231;
            p234.temperature_air = (sbyte)(sbyte)19;
            p234.climb_rate = (sbyte)(sbyte)80;
            p234.pitch = (short)(short)7980;
            p234.wp_num = (byte)(byte)126;
            p234.altitude_sp = (short)(short) -18320;
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
            p234.roll = (short)(short)25742;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT;
            p234.throttle = (sbyte)(sbyte) - 60;
            p234.heading = (ushort)(ushort)62343;
            p234.battery_remaining = (byte)(byte)31;
            p234.altitude_amsl = (short)(short) -4047;
            p234.longitude = (int)717674267;
            p234.airspeed = (byte)(byte)239;
            p234.airspeed_sp = (byte)(byte)77;
            p234.temperature = (sbyte)(sbyte)101;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.clipping_0 == (uint)181027624U);
                Debug.Assert(pack.vibration_y == (float)9.0215575E36F);
                Debug.Assert(pack.time_usec == (ulong)1376499410770387646L);
                Debug.Assert(pack.vibration_x == (float) -1.6192752E38F);
                Debug.Assert(pack.clipping_1 == (uint)2910310473U);
                Debug.Assert(pack.clipping_2 == (uint)1510730005U);
                Debug.Assert(pack.vibration_z == (float) -2.5354048E38F);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.clipping_0 = (uint)181027624U;
            p241.time_usec = (ulong)1376499410770387646L;
            p241.vibration_z = (float) -2.5354048E38F;
            p241.clipping_2 = (uint)1510730005U;
            p241.clipping_1 = (uint)2910310473U;
            p241.vibration_x = (float) -1.6192752E38F;
            p241.vibration_y = (float)9.0215575E36F;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.approach_x == (float) -1.7265322E38F);
                Debug.Assert(pack.approach_y == (float) -2.8063845E38F);
                Debug.Assert(pack.x == (float)1.0289709E38F);
                Debug.Assert(pack.altitude == (int) -1132979073);
                Debug.Assert(pack.y == (float) -8.795852E37F);
                Debug.Assert(pack.approach_z == (float) -1.2468535E37F);
                Debug.Assert(pack.longitude == (int) -223513518);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.8564252E38F, -7.1620433E37F, -3.0723328E38F, -2.3077323E37F}));
                Debug.Assert(pack.z == (float)2.6304797E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7899575049699552642L);
                Debug.Assert(pack.latitude == (int)1035965859);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.x = (float)1.0289709E38F;
            p242.q_SET(new float[] {-1.8564252E38F, -7.1620433E37F, -3.0723328E38F, -2.3077323E37F}, 0) ;
            p242.longitude = (int) -223513518;
            p242.y = (float) -8.795852E37F;
            p242.approach_y = (float) -2.8063845E38F;
            p242.z = (float)2.6304797E38F;
            p242.approach_x = (float) -1.7265322E38F;
            p242.altitude = (int) -1132979073;
            p242.approach_z = (float) -1.2468535E37F;
            p242.time_usec_SET((ulong)7899575049699552642L, PH) ;
            p242.latitude = (int)1035965859;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.approach_z == (float)1.3407087E38F);
                Debug.Assert(pack.longitude == (int) -1314599427);
                Debug.Assert(pack.target_system == (byte)(byte)245);
                Debug.Assert(pack.z == (float) -1.8725303E38F);
                Debug.Assert(pack.approach_y == (float) -1.8561579E38F);
                Debug.Assert(pack.altitude == (int)772416431);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)8280038243209033808L);
                Debug.Assert(pack.latitude == (int)1342560524);
                Debug.Assert(pack.x == (float)2.1613893E38F);
                Debug.Assert(pack.approach_x == (float)3.2023122E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.0181859E37F, 2.789236E38F, 3.3218192E38F, 1.7709986E38F}));
                Debug.Assert(pack.y == (float)2.4785683E38F);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.y = (float)2.4785683E38F;
            p243.q_SET(new float[] {1.0181859E37F, 2.789236E38F, 3.3218192E38F, 1.7709986E38F}, 0) ;
            p243.latitude = (int)1342560524;
            p243.longitude = (int) -1314599427;
            p243.x = (float)2.1613893E38F;
            p243.target_system = (byte)(byte)245;
            p243.approach_y = (float) -1.8561579E38F;
            p243.approach_x = (float)3.2023122E38F;
            p243.z = (float) -1.8725303E38F;
            p243.approach_z = (float)1.3407087E38F;
            p243.altitude = (int)772416431;
            p243.time_usec_SET((ulong)8280038243209033808L, PH) ;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.interval_us == (int) -1121626519);
                Debug.Assert(pack.message_id == (ushort)(ushort)51250);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int) -1121626519;
            p244.message_id = (ushort)(ushort)51250;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
                Debug.Assert(pack.vtol_state == MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_MC;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tslc == (byte)(byte)134);
                Debug.Assert(pack.emitter_type == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_NO_INFO);
                Debug.Assert(pack.lon == (int)1331129839);
                Debug.Assert(pack.squawk == (ushort)(ushort)51157);
                Debug.Assert(pack.ICAO_address == (uint)1737433791U);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)28836);
                Debug.Assert(pack.ver_velocity == (short)(short)11809);
                Debug.Assert(pack.altitude == (int)2019338562);
                Debug.Assert(pack.callsign_LEN(ph) == 9);
                Debug.Assert(pack.callsign_TRY(ph).Equals("kyuyvcdxy"));
                Debug.Assert(pack.flags == (ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                                            ADSB_FLAGS.ADSB_FLAGS_SIMULATED));
                Debug.Assert(pack.altitude_type == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
                Debug.Assert(pack.lat == (int) -824464788);
                Debug.Assert(pack.heading == (ushort)(ushort)41703);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.squawk = (ushort)(ushort)51157;
            p246.callsign_SET("kyuyvcdxy", PH) ;
            p246.hor_velocity = (ushort)(ushort)28836;
            p246.ver_velocity = (short)(short)11809;
            p246.ICAO_address = (uint)1737433791U;
            p246.lon = (int)1331129839;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                          ADSB_FLAGS.ADSB_FLAGS_SIMULATED);
            p246.altitude = (int)2019338562;
            p246.tslc = (byte)(byte)134;
            p246.lat = (int) -824464788;
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_NO_INFO;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            p246.heading = (ushort)(ushort)41703;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.horizontal_minimum_delta == (float) -1.1154647E38F);
                Debug.Assert(pack.action == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER);
                Debug.Assert(pack.altitude_minimum_delta == (float) -6.322065E37F);
                Debug.Assert(pack.time_to_minimum_delta == (float)3.1564725E38F);
                Debug.Assert(pack.src_ == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH |
                                                   MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW));
                Debug.Assert(pack.id == (uint)2523632948U);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.horizontal_minimum_delta = (float) -1.1154647E38F;
            p247.time_to_minimum_delta = (float)3.1564725E38F;
            p247.altitude_minimum_delta = (float) -6.322065E37F;
            p247.id = (uint)2523632948U;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH |
                                 MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)162);
                Debug.Assert(pack.message_type == (ushort)(ushort)1953);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)68, (byte)187, (byte)19, (byte)56, (byte)228, (byte)109, (byte)43, (byte)244, (byte)134, (byte)91, (byte)20, (byte)197, (byte)244, (byte)103, (byte)195, (byte)81, (byte)209, (byte)141, (byte)146, (byte)18, (byte)44, (byte)215, (byte)164, (byte)20, (byte)250, (byte)129, (byte)223, (byte)142, (byte)169, (byte)169, (byte)166, (byte)46, (byte)25, (byte)149, (byte)57, (byte)237, (byte)153, (byte)6, (byte)116, (byte)202, (byte)120, (byte)195, (byte)108, (byte)239, (byte)140, (byte)162, (byte)75, (byte)241, (byte)74, (byte)82, (byte)249, (byte)71, (byte)225, (byte)192, (byte)122, (byte)94, (byte)147, (byte)89, (byte)44, (byte)125, (byte)0, (byte)20, (byte)12, (byte)102, (byte)114, (byte)4, (byte)180, (byte)179, (byte)101, (byte)120, (byte)22, (byte)203, (byte)75, (byte)13, (byte)36, (byte)209, (byte)231, (byte)133, (byte)142, (byte)94, (byte)116, (byte)160, (byte)45, (byte)35, (byte)36, (byte)57, (byte)50, (byte)95, (byte)238, (byte)233, (byte)212, (byte)179, (byte)42, (byte)6, (byte)96, (byte)212, (byte)60, (byte)178, (byte)144, (byte)66, (byte)157, (byte)172, (byte)108, (byte)60, (byte)172, (byte)146, (byte)152, (byte)100, (byte)16, (byte)101, (byte)8, (byte)156, (byte)189, (byte)148, (byte)47, (byte)15, (byte)240, (byte)124, (byte)146, (byte)81, (byte)169, (byte)14, (byte)42, (byte)189, (byte)69, (byte)194, (byte)93, (byte)43, (byte)204, (byte)54, (byte)97, (byte)254, (byte)59, (byte)221, (byte)197, (byte)48, (byte)78, (byte)101, (byte)104, (byte)184, (byte)31, (byte)91, (byte)193, (byte)122, (byte)152, (byte)245, (byte)138, (byte)222, (byte)122, (byte)103, (byte)159, (byte)99, (byte)71, (byte)186, (byte)52, (byte)88, (byte)138, (byte)192, (byte)227, (byte)38, (byte)55, (byte)194, (byte)129, (byte)177, (byte)157, (byte)241, (byte)49, (byte)4, (byte)48, (byte)173, (byte)211, (byte)0, (byte)139, (byte)66, (byte)0, (byte)179, (byte)237, (byte)93, (byte)184, (byte)176, (byte)224, (byte)184, (byte)182, (byte)25, (byte)71, (byte)87, (byte)110, (byte)251, (byte)179, (byte)254, (byte)139, (byte)19, (byte)17, (byte)119, (byte)210, (byte)201, (byte)93, (byte)166, (byte)41, (byte)161, (byte)227, (byte)30, (byte)44, (byte)101, (byte)124, (byte)90, (byte)146, (byte)227, (byte)226, (byte)223, (byte)46, (byte)37, (byte)137, (byte)131, (byte)10, (byte)106, (byte)188, (byte)22, (byte)97, (byte)101, (byte)77, (byte)84, (byte)3, (byte)69, (byte)118, (byte)115, (byte)239, (byte)80, (byte)217, (byte)37, (byte)195, (byte)60, (byte)14, (byte)28, (byte)208, (byte)222, (byte)113, (byte)195, (byte)16, (byte)234, (byte)145, (byte)63, (byte)175, (byte)77, (byte)40, (byte)38, (byte)53, (byte)59, (byte)35}));
                Debug.Assert(pack.target_network == (byte)(byte)184);
                Debug.Assert(pack.target_component == (byte)(byte)196);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_component = (byte)(byte)196;
            p248.target_system = (byte)(byte)162;
            p248.message_type = (ushort)(ushort)1953;
            p248.payload_SET(new byte[] {(byte)68, (byte)187, (byte)19, (byte)56, (byte)228, (byte)109, (byte)43, (byte)244, (byte)134, (byte)91, (byte)20, (byte)197, (byte)244, (byte)103, (byte)195, (byte)81, (byte)209, (byte)141, (byte)146, (byte)18, (byte)44, (byte)215, (byte)164, (byte)20, (byte)250, (byte)129, (byte)223, (byte)142, (byte)169, (byte)169, (byte)166, (byte)46, (byte)25, (byte)149, (byte)57, (byte)237, (byte)153, (byte)6, (byte)116, (byte)202, (byte)120, (byte)195, (byte)108, (byte)239, (byte)140, (byte)162, (byte)75, (byte)241, (byte)74, (byte)82, (byte)249, (byte)71, (byte)225, (byte)192, (byte)122, (byte)94, (byte)147, (byte)89, (byte)44, (byte)125, (byte)0, (byte)20, (byte)12, (byte)102, (byte)114, (byte)4, (byte)180, (byte)179, (byte)101, (byte)120, (byte)22, (byte)203, (byte)75, (byte)13, (byte)36, (byte)209, (byte)231, (byte)133, (byte)142, (byte)94, (byte)116, (byte)160, (byte)45, (byte)35, (byte)36, (byte)57, (byte)50, (byte)95, (byte)238, (byte)233, (byte)212, (byte)179, (byte)42, (byte)6, (byte)96, (byte)212, (byte)60, (byte)178, (byte)144, (byte)66, (byte)157, (byte)172, (byte)108, (byte)60, (byte)172, (byte)146, (byte)152, (byte)100, (byte)16, (byte)101, (byte)8, (byte)156, (byte)189, (byte)148, (byte)47, (byte)15, (byte)240, (byte)124, (byte)146, (byte)81, (byte)169, (byte)14, (byte)42, (byte)189, (byte)69, (byte)194, (byte)93, (byte)43, (byte)204, (byte)54, (byte)97, (byte)254, (byte)59, (byte)221, (byte)197, (byte)48, (byte)78, (byte)101, (byte)104, (byte)184, (byte)31, (byte)91, (byte)193, (byte)122, (byte)152, (byte)245, (byte)138, (byte)222, (byte)122, (byte)103, (byte)159, (byte)99, (byte)71, (byte)186, (byte)52, (byte)88, (byte)138, (byte)192, (byte)227, (byte)38, (byte)55, (byte)194, (byte)129, (byte)177, (byte)157, (byte)241, (byte)49, (byte)4, (byte)48, (byte)173, (byte)211, (byte)0, (byte)139, (byte)66, (byte)0, (byte)179, (byte)237, (byte)93, (byte)184, (byte)176, (byte)224, (byte)184, (byte)182, (byte)25, (byte)71, (byte)87, (byte)110, (byte)251, (byte)179, (byte)254, (byte)139, (byte)19, (byte)17, (byte)119, (byte)210, (byte)201, (byte)93, (byte)166, (byte)41, (byte)161, (byte)227, (byte)30, (byte)44, (byte)101, (byte)124, (byte)90, (byte)146, (byte)227, (byte)226, (byte)223, (byte)46, (byte)37, (byte)137, (byte)131, (byte)10, (byte)106, (byte)188, (byte)22, (byte)97, (byte)101, (byte)77, (byte)84, (byte)3, (byte)69, (byte)118, (byte)115, (byte)239, (byte)80, (byte)217, (byte)37, (byte)195, (byte)60, (byte)14, (byte)28, (byte)208, (byte)222, (byte)113, (byte)195, (byte)16, (byte)234, (byte)145, (byte)63, (byte)175, (byte)77, (byte)40, (byte)38, (byte)53, (byte)59, (byte)35}, 0) ;
            p248.target_network = (byte)(byte)184;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte)2, (sbyte)107, (sbyte) - 7, (sbyte)68, (sbyte) - 9, (sbyte)31, (sbyte) - 63, (sbyte)126, (sbyte) - 38, (sbyte)58, (sbyte) - 91, (sbyte)46, (sbyte) - 52, (sbyte)62, (sbyte)71, (sbyte)21, (sbyte)54, (sbyte) - 102, (sbyte) - 40, (sbyte)90, (sbyte) - 44, (sbyte) - 82, (sbyte)96, (sbyte)43, (sbyte) - 108, (sbyte) - 1, (sbyte)74, (sbyte) - 106, (sbyte) - 32, (sbyte) - 71, (sbyte)106, (sbyte)113}));
                Debug.Assert(pack.ver == (byte)(byte)24);
                Debug.Assert(pack.type == (byte)(byte)202);
                Debug.Assert(pack.address == (ushort)(ushort)19270);
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.ver = (byte)(byte)24;
            p249.type = (byte)(byte)202;
            p249.address = (ushort)(ushort)19270;
            p249.value_SET(new sbyte[] {(sbyte)2, (sbyte)107, (sbyte) - 7, (sbyte)68, (sbyte) - 9, (sbyte)31, (sbyte) - 63, (sbyte)126, (sbyte) - 38, (sbyte)58, (sbyte) - 91, (sbyte)46, (sbyte) - 52, (sbyte)62, (sbyte)71, (sbyte)21, (sbyte)54, (sbyte) - 102, (sbyte) - 40, (sbyte)90, (sbyte) - 44, (sbyte) - 82, (sbyte)96, (sbyte)43, (sbyte) - 108, (sbyte) - 1, (sbyte)74, (sbyte) - 106, (sbyte) - 32, (sbyte) - 71, (sbyte)106, (sbyte)113}, 0) ;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)2.6712585E38F);
                Debug.Assert(pack.y == (float) -1.7411106E38F);
                Debug.Assert(pack.time_usec == (ulong)523617196360593503L);
                Debug.Assert(pack.x == (float)3.0986088E38F);
                Debug.Assert(pack.name_LEN(ph) == 4);
                Debug.Assert(pack.name_TRY(ph).Equals("zstn"));
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.x = (float)3.0986088E38F;
            p250.name_SET("zstn", PH) ;
            p250.time_usec = (ulong)523617196360593503L;
            p250.y = (float) -1.7411106E38F;
            p250.z = (float)2.6712585E38F;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float) -9.128943E37F);
                Debug.Assert(pack.name_LEN(ph) == 10);
                Debug.Assert(pack.name_TRY(ph).Equals("rvmyirlEyb"));
                Debug.Assert(pack.time_boot_ms == (uint)3056124756U);
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)3056124756U;
            p251.value = (float) -9.128943E37F;
            p251.name_SET("rvmyirlEyb", PH) ;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1067593764U);
                Debug.Assert(pack.name_LEN(ph) == 9);
                Debug.Assert(pack.name_TRY(ph).Equals("eOjttxbrw"));
                Debug.Assert(pack.value == (int)810408405);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.name_SET("eOjttxbrw", PH) ;
            p252.time_boot_ms = (uint)1067593764U;
            p252.value = (int)810408405;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 23);
                Debug.Assert(pack.text_TRY(ph).Equals("bemrdfmqodrepwsvwsvjyrd"));
                Debug.Assert(pack.severity == MAV_SEVERITY.MAV_SEVERITY_DEBUG);
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("bemrdfmqodrepwsvwsvjyrd", PH) ;
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_DEBUG;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float)1.0564236E38F);
                Debug.Assert(pack.ind == (byte)(byte)86);
                Debug.Assert(pack.time_boot_ms == (uint)3014378780U);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.value = (float)1.0564236E38F;
            p254.ind = (byte)(byte)86;
            p254.time_boot_ms = (uint)3014378780U;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)155);
                Debug.Assert(pack.initial_timestamp == (ulong)8322208968641502297L);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)110, (byte)209, (byte)129, (byte)65, (byte)162, (byte)213, (byte)18, (byte)194, (byte)244, (byte)161, (byte)142, (byte)150, (byte)113, (byte)155, (byte)153, (byte)17, (byte)3, (byte)61, (byte)225, (byte)62, (byte)42, (byte)238, (byte)52, (byte)206, (byte)96, (byte)72, (byte)31, (byte)78, (byte)184, (byte)228, (byte)73, (byte)131}));
                Debug.Assert(pack.target_component == (byte)(byte)173);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.secret_key_SET(new byte[] {(byte)110, (byte)209, (byte)129, (byte)65, (byte)162, (byte)213, (byte)18, (byte)194, (byte)244, (byte)161, (byte)142, (byte)150, (byte)113, (byte)155, (byte)153, (byte)17, (byte)3, (byte)61, (byte)225, (byte)62, (byte)42, (byte)238, (byte)52, (byte)206, (byte)96, (byte)72, (byte)31, (byte)78, (byte)184, (byte)228, (byte)73, (byte)131}, 0) ;
            p256.target_component = (byte)(byte)173;
            p256.initial_timestamp = (ulong)8322208968641502297L;
            p256.target_system = (byte)(byte)155;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_change_ms == (uint)551393501U);
                Debug.Assert(pack.state == (byte)(byte)208);
                Debug.Assert(pack.time_boot_ms == (uint)3815139237U);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)3815139237U;
            p257.state = (byte)(byte)208;
            p257.last_change_ms = (uint)551393501U;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tune_LEN(ph) == 26);
                Debug.Assert(pack.tune_TRY(ph).Equals("uvximbmnpgfijxrfwbdrmzgzlp"));
                Debug.Assert(pack.target_system == (byte)(byte)90);
                Debug.Assert(pack.target_component == (byte)(byte)93);
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_component = (byte)(byte)93;
            p258.tune_SET("uvximbmnpgfijxrfwbdrmzgzlp", PH) ;
            p258.target_system = (byte)(byte)90;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2531877946U);
                Debug.Assert(pack.sensor_size_v == (float)1.7202165E38F);
                Debug.Assert(pack.firmware_version == (uint)3944940344U);
                Debug.Assert(pack.lens_id == (byte)(byte)54);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)158, (byte)159, (byte)201, (byte)223, (byte)34, (byte)117, (byte)129, (byte)244, (byte)217, (byte)103, (byte)45, (byte)192, (byte)164, (byte)78, (byte)117, (byte)118, (byte)193, (byte)113, (byte)80, (byte)13, (byte)149, (byte)250, (byte)249, (byte)45, (byte)71, (byte)189, (byte)147, (byte)195, (byte)197, (byte)229, (byte)250, (byte)60}));
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 111);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("iukSqBwtVhjndwxeysxuvilkimvmhcmenwQbyonLlrkjxuekusplktgfksVuahnpUlsvynmwhbSGmlkeaovnqfdyszdmxHeYsmxabqhironqjXd"));
                Debug.Assert(pack.resolution_v == (ushort)(ushort)44952);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE));
                Debug.Assert(pack.focal_length == (float) -5.0842236E37F);
                Debug.Assert(pack.sensor_size_h == (float) -2.3981345E38F);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)26768);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)245, (byte)122, (byte)142, (byte)117, (byte)193, (byte)125, (byte)153, (byte)6, (byte)218, (byte)249, (byte)75, (byte)192, (byte)106, (byte)58, (byte)154, (byte)50, (byte)229, (byte)203, (byte)95, (byte)219, (byte)112, (byte)193, (byte)136, (byte)111, (byte)190, (byte)205, (byte)92, (byte)190, (byte)208, (byte)84, (byte)129, (byte)90}));
                Debug.Assert(pack.resolution_h == (ushort)(ushort)34168);
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.vendor_name_SET(new byte[] {(byte)245, (byte)122, (byte)142, (byte)117, (byte)193, (byte)125, (byte)153, (byte)6, (byte)218, (byte)249, (byte)75, (byte)192, (byte)106, (byte)58, (byte)154, (byte)50, (byte)229, (byte)203, (byte)95, (byte)219, (byte)112, (byte)193, (byte)136, (byte)111, (byte)190, (byte)205, (byte)92, (byte)190, (byte)208, (byte)84, (byte)129, (byte)90}, 0) ;
            p259.resolution_h = (ushort)(ushort)34168;
            p259.firmware_version = (uint)3944940344U;
            p259.sensor_size_v = (float)1.7202165E38F;
            p259.resolution_v = (ushort)(ushort)44952;
            p259.focal_length = (float) -5.0842236E37F;
            p259.cam_definition_uri_SET("iukSqBwtVhjndwxeysxuvilkimvmhcmenwQbyonLlrkjxuekusplktgfksVuahnpUlsvynmwhbSGmlkeaovnqfdyszdmxHeYsmxabqhironqjXd", PH) ;
            p259.time_boot_ms = (uint)2531877946U;
            p259.lens_id = (byte)(byte)54;
            p259.sensor_size_h = (float) -2.3981345E38F;
            p259.cam_definition_version = (ushort)(ushort)26768;
            p259.model_name_SET(new byte[] {(byte)158, (byte)159, (byte)201, (byte)223, (byte)34, (byte)117, (byte)129, (byte)244, (byte)217, (byte)103, (byte)45, (byte)192, (byte)164, (byte)78, (byte)117, (byte)118, (byte)193, (byte)113, (byte)80, (byte)13, (byte)149, (byte)250, (byte)249, (byte)45, (byte)71, (byte)189, (byte)147, (byte)195, (byte)197, (byte)229, (byte)250, (byte)60}, 0) ;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE);
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == (CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY));
                Debug.Assert(pack.time_boot_ms == (uint)1041446738U);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = (CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
            p260.time_boot_ms = (uint)1041446738U;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2482517519U);
                Debug.Assert(pack.status == (byte)(byte)145);
                Debug.Assert(pack.available_capacity == (float)1.2049233E38F);
                Debug.Assert(pack.read_speed == (float)1.918031E38F);
                Debug.Assert(pack.storage_id == (byte)(byte)194);
                Debug.Assert(pack.storage_count == (byte)(byte)98);
                Debug.Assert(pack.used_capacity == (float)2.3439857E38F);
                Debug.Assert(pack.write_speed == (float)9.73791E36F);
                Debug.Assert(pack.total_capacity == (float) -2.0946277E38F);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.read_speed = (float)1.918031E38F;
            p261.used_capacity = (float)2.3439857E38F;
            p261.storage_count = (byte)(byte)98;
            p261.total_capacity = (float) -2.0946277E38F;
            p261.write_speed = (float)9.73791E36F;
            p261.available_capacity = (float)1.2049233E38F;
            p261.status = (byte)(byte)145;
            p261.storage_id = (byte)(byte)194;
            p261.time_boot_ms = (uint)2482517519U;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)293973744U);
                Debug.Assert(pack.available_capacity == (float)9.426825E37F);
                Debug.Assert(pack.video_status == (byte)(byte)128);
                Debug.Assert(pack.image_status == (byte)(byte)175);
                Debug.Assert(pack.recording_time_ms == (uint)3204331496U);
                Debug.Assert(pack.image_interval == (float)1.9257673E38F);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.image_interval = (float)1.9257673E38F;
            p262.recording_time_ms = (uint)3204331496U;
            p262.available_capacity = (float)9.426825E37F;
            p262.time_boot_ms = (uint)293973744U;
            p262.video_status = (byte)(byte)128;
            p262.image_status = (byte)(byte)175;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.1661541E38F, 2.0003161E37F, -2.7450615E38F, 1.6947186E38F}));
                Debug.Assert(pack.lat == (int)808774110);
                Debug.Assert(pack.image_index == (int) -1278111268);
                Debug.Assert(pack.time_utc == (ulong)3955609997493710597L);
                Debug.Assert(pack.relative_alt == (int)384526769);
                Debug.Assert(pack.time_boot_ms == (uint)2380912048U);
                Debug.Assert(pack.lon == (int) -822372177);
                Debug.Assert(pack.camera_id == (byte)(byte)172);
                Debug.Assert(pack.file_url_LEN(ph) == 34);
                Debug.Assert(pack.file_url_TRY(ph).Equals("kaXawcxxwpqzmfdYtcquwuRbdozuImZwou"));
                Debug.Assert(pack.alt == (int) -2104630363);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte) - 46);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.camera_id = (byte)(byte)172;
            p263.image_index = (int) -1278111268;
            p263.relative_alt = (int)384526769;
            p263.file_url_SET("kaXawcxxwpqzmfdYtcquwuRbdozuImZwou", PH) ;
            p263.lon = (int) -822372177;
            p263.capture_result = (sbyte)(sbyte) - 46;
            p263.time_boot_ms = (uint)2380912048U;
            p263.lat = (int)808774110;
            p263.time_utc = (ulong)3955609997493710597L;
            p263.alt = (int) -2104630363;
            p263.q_SET(new float[] {-3.1661541E38F, 2.0003161E37F, -2.7450615E38F, 1.6947186E38F}, 0) ;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.arming_time_utc == (ulong)2150361959407797909L);
                Debug.Assert(pack.time_boot_ms == (uint)2979466842U);
                Debug.Assert(pack.flight_uuid == (ulong)3585791853558402190L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)7321994193774910422L);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.flight_uuid = (ulong)3585791853558402190L;
            p264.arming_time_utc = (ulong)2150361959407797909L;
            p264.time_boot_ms = (uint)2979466842U;
            p264.takeoff_time_utc = (ulong)7321994193774910422L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)2.334333E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2130395281U);
                Debug.Assert(pack.yaw == (float) -2.2890947E38F);
                Debug.Assert(pack.pitch == (float)1.0106968E38F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.roll = (float)2.334333E38F;
            p265.yaw = (float) -2.2890947E38F;
            p265.pitch = (float)1.0106968E38F;
            p265.time_boot_ms = (uint)2130395281U;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sequence == (ushort)(ushort)40486);
                Debug.Assert(pack.first_message_offset == (byte)(byte)172);
                Debug.Assert(pack.target_system == (byte)(byte)147);
                Debug.Assert(pack.target_component == (byte)(byte)44);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)206, (byte)130, (byte)92, (byte)182, (byte)72, (byte)253, (byte)63, (byte)175, (byte)57, (byte)130, (byte)215, (byte)167, (byte)154, (byte)176, (byte)201, (byte)184, (byte)82, (byte)62, (byte)25, (byte)145, (byte)197, (byte)206, (byte)99, (byte)199, (byte)133, (byte)74, (byte)127, (byte)38, (byte)102, (byte)27, (byte)135, (byte)219, (byte)49, (byte)99, (byte)253, (byte)161, (byte)66, (byte)83, (byte)20, (byte)208, (byte)38, (byte)91, (byte)158, (byte)226, (byte)48, (byte)58, (byte)110, (byte)67, (byte)37, (byte)109, (byte)26, (byte)126, (byte)19, (byte)59, (byte)173, (byte)22, (byte)85, (byte)147, (byte)105, (byte)13, (byte)36, (byte)186, (byte)224, (byte)136, (byte)130, (byte)166, (byte)56, (byte)173, (byte)151, (byte)45, (byte)58, (byte)75, (byte)143, (byte)184, (byte)218, (byte)164, (byte)94, (byte)219, (byte)115, (byte)196, (byte)112, (byte)75, (byte)150, (byte)202, (byte)238, (byte)27, (byte)207, (byte)16, (byte)56, (byte)110, (byte)47, (byte)187, (byte)34, (byte)133, (byte)8, (byte)84, (byte)61, (byte)48, (byte)212, (byte)51, (byte)164, (byte)74, (byte)23, (byte)97, (byte)26, (byte)66, (byte)95, (byte)109, (byte)84, (byte)15, (byte)212, (byte)210, (byte)174, (byte)8, (byte)241, (byte)83, (byte)34, (byte)216, (byte)214, (byte)120, (byte)74, (byte)15, (byte)222, (byte)249, (byte)98, (byte)159, (byte)18, (byte)186, (byte)19, (byte)246, (byte)70, (byte)123, (byte)211, (byte)18, (byte)48, (byte)57, (byte)56, (byte)53, (byte)37, (byte)46, (byte)70, (byte)230, (byte)247, (byte)154, (byte)203, (byte)252, (byte)80, (byte)93, (byte)79, (byte)180, (byte)114, (byte)158, (byte)4, (byte)116, (byte)73, (byte)108, (byte)17, (byte)136, (byte)137, (byte)143, (byte)27, (byte)56, (byte)158, (byte)227, (byte)112, (byte)209, (byte)43, (byte)216, (byte)202, (byte)79, (byte)101, (byte)60, (byte)236, (byte)55, (byte)145, (byte)186, (byte)117, (byte)165, (byte)143, (byte)181, (byte)55, (byte)172, (byte)228, (byte)209, (byte)197, (byte)72, (byte)174, (byte)210, (byte)200, (byte)229, (byte)134, (byte)51, (byte)93, (byte)178, (byte)251, (byte)65, (byte)69, (byte)198, (byte)232, (byte)191, (byte)218, (byte)143, (byte)76, (byte)251, (byte)112, (byte)96, (byte)12, (byte)217, (byte)53, (byte)119, (byte)120, (byte)11, (byte)80, (byte)82, (byte)98, (byte)210, (byte)43, (byte)70, (byte)24, (byte)253, (byte)220, (byte)60, (byte)211, (byte)228, (byte)247, (byte)21, (byte)206, (byte)81, (byte)158, (byte)93, (byte)77, (byte)141, (byte)202, (byte)112, (byte)237, (byte)127, (byte)110, (byte)193, (byte)166, (byte)251, (byte)24, (byte)234, (byte)245, (byte)141, (byte)90, (byte)145, (byte)161, (byte)242, (byte)88}));
                Debug.Assert(pack.length == (byte)(byte)45);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.data__SET(new byte[] {(byte)206, (byte)130, (byte)92, (byte)182, (byte)72, (byte)253, (byte)63, (byte)175, (byte)57, (byte)130, (byte)215, (byte)167, (byte)154, (byte)176, (byte)201, (byte)184, (byte)82, (byte)62, (byte)25, (byte)145, (byte)197, (byte)206, (byte)99, (byte)199, (byte)133, (byte)74, (byte)127, (byte)38, (byte)102, (byte)27, (byte)135, (byte)219, (byte)49, (byte)99, (byte)253, (byte)161, (byte)66, (byte)83, (byte)20, (byte)208, (byte)38, (byte)91, (byte)158, (byte)226, (byte)48, (byte)58, (byte)110, (byte)67, (byte)37, (byte)109, (byte)26, (byte)126, (byte)19, (byte)59, (byte)173, (byte)22, (byte)85, (byte)147, (byte)105, (byte)13, (byte)36, (byte)186, (byte)224, (byte)136, (byte)130, (byte)166, (byte)56, (byte)173, (byte)151, (byte)45, (byte)58, (byte)75, (byte)143, (byte)184, (byte)218, (byte)164, (byte)94, (byte)219, (byte)115, (byte)196, (byte)112, (byte)75, (byte)150, (byte)202, (byte)238, (byte)27, (byte)207, (byte)16, (byte)56, (byte)110, (byte)47, (byte)187, (byte)34, (byte)133, (byte)8, (byte)84, (byte)61, (byte)48, (byte)212, (byte)51, (byte)164, (byte)74, (byte)23, (byte)97, (byte)26, (byte)66, (byte)95, (byte)109, (byte)84, (byte)15, (byte)212, (byte)210, (byte)174, (byte)8, (byte)241, (byte)83, (byte)34, (byte)216, (byte)214, (byte)120, (byte)74, (byte)15, (byte)222, (byte)249, (byte)98, (byte)159, (byte)18, (byte)186, (byte)19, (byte)246, (byte)70, (byte)123, (byte)211, (byte)18, (byte)48, (byte)57, (byte)56, (byte)53, (byte)37, (byte)46, (byte)70, (byte)230, (byte)247, (byte)154, (byte)203, (byte)252, (byte)80, (byte)93, (byte)79, (byte)180, (byte)114, (byte)158, (byte)4, (byte)116, (byte)73, (byte)108, (byte)17, (byte)136, (byte)137, (byte)143, (byte)27, (byte)56, (byte)158, (byte)227, (byte)112, (byte)209, (byte)43, (byte)216, (byte)202, (byte)79, (byte)101, (byte)60, (byte)236, (byte)55, (byte)145, (byte)186, (byte)117, (byte)165, (byte)143, (byte)181, (byte)55, (byte)172, (byte)228, (byte)209, (byte)197, (byte)72, (byte)174, (byte)210, (byte)200, (byte)229, (byte)134, (byte)51, (byte)93, (byte)178, (byte)251, (byte)65, (byte)69, (byte)198, (byte)232, (byte)191, (byte)218, (byte)143, (byte)76, (byte)251, (byte)112, (byte)96, (byte)12, (byte)217, (byte)53, (byte)119, (byte)120, (byte)11, (byte)80, (byte)82, (byte)98, (byte)210, (byte)43, (byte)70, (byte)24, (byte)253, (byte)220, (byte)60, (byte)211, (byte)228, (byte)247, (byte)21, (byte)206, (byte)81, (byte)158, (byte)93, (byte)77, (byte)141, (byte)202, (byte)112, (byte)237, (byte)127, (byte)110, (byte)193, (byte)166, (byte)251, (byte)24, (byte)234, (byte)245, (byte)141, (byte)90, (byte)145, (byte)161, (byte)242, (byte)88}, 0) ;
            p266.target_component = (byte)(byte)44;
            p266.target_system = (byte)(byte)147;
            p266.sequence = (ushort)(ushort)40486;
            p266.first_message_offset = (byte)(byte)172;
            p266.length = (byte)(byte)45;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)21);
                Debug.Assert(pack.length == (byte)(byte)79);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)250, (byte)216, (byte)93, (byte)22, (byte)168, (byte)24, (byte)197, (byte)51, (byte)126, (byte)36, (byte)117, (byte)201, (byte)114, (byte)202, (byte)200, (byte)92, (byte)17, (byte)106, (byte)87, (byte)210, (byte)119, (byte)23, (byte)160, (byte)148, (byte)17, (byte)199, (byte)217, (byte)150, (byte)39, (byte)45, (byte)82, (byte)32, (byte)4, (byte)7, (byte)234, (byte)221, (byte)169, (byte)93, (byte)48, (byte)114, (byte)164, (byte)175, (byte)234, (byte)178, (byte)184, (byte)121, (byte)228, (byte)55, (byte)108, (byte)36, (byte)54, (byte)222, (byte)66, (byte)227, (byte)232, (byte)203, (byte)249, (byte)129, (byte)93, (byte)7, (byte)227, (byte)66, (byte)152, (byte)131, (byte)92, (byte)32, (byte)217, (byte)98, (byte)228, (byte)101, (byte)240, (byte)39, (byte)166, (byte)246, (byte)112, (byte)235, (byte)205, (byte)51, (byte)59, (byte)48, (byte)178, (byte)40, (byte)211, (byte)248, (byte)179, (byte)152, (byte)93, (byte)252, (byte)176, (byte)17, (byte)139, (byte)9, (byte)148, (byte)124, (byte)234, (byte)150, (byte)129, (byte)66, (byte)51, (byte)198, (byte)215, (byte)140, (byte)128, (byte)28, (byte)161, (byte)5, (byte)123, (byte)251, (byte)139, (byte)186, (byte)72, (byte)190, (byte)11, (byte)174, (byte)163, (byte)94, (byte)83, (byte)229, (byte)229, (byte)48, (byte)89, (byte)161, (byte)189, (byte)47, (byte)252, (byte)161, (byte)108, (byte)91, (byte)21, (byte)80, (byte)155, (byte)60, (byte)37, (byte)101, (byte)122, (byte)16, (byte)118, (byte)163, (byte)172, (byte)72, (byte)182, (byte)176, (byte)16, (byte)165, (byte)64, (byte)105, (byte)121, (byte)170, (byte)208, (byte)117, (byte)5, (byte)80, (byte)32, (byte)193, (byte)83, (byte)9, (byte)236, (byte)91, (byte)190, (byte)107, (byte)43, (byte)240, (byte)248, (byte)102, (byte)27, (byte)69, (byte)56, (byte)98, (byte)26, (byte)159, (byte)252, (byte)226, (byte)63, (byte)11, (byte)36, (byte)108, (byte)244, (byte)8, (byte)215, (byte)173, (byte)107, (byte)221, (byte)108, (byte)119, (byte)230, (byte)253, (byte)167, (byte)234, (byte)147, (byte)98, (byte)102, (byte)118, (byte)106, (byte)226, (byte)80, (byte)161, (byte)113, (byte)172, (byte)53, (byte)42, (byte)114, (byte)112, (byte)4, (byte)7, (byte)0, (byte)104, (byte)17, (byte)83, (byte)1, (byte)85, (byte)111, (byte)36, (byte)92, (byte)138, (byte)129, (byte)233, (byte)158, (byte)130, (byte)4, (byte)212, (byte)173, (byte)229, (byte)144, (byte)168, (byte)155, (byte)233, (byte)43, (byte)158, (byte)254, (byte)158, (byte)243, (byte)111, (byte)145, (byte)235, (byte)144, (byte)203, (byte)61, (byte)75, (byte)254, (byte)125, (byte)39, (byte)208, (byte)35, (byte)118, (byte)202, (byte)192, (byte)110, (byte)190, (byte)158}));
                Debug.Assert(pack.target_component == (byte)(byte)151);
                Debug.Assert(pack.sequence == (ushort)(ushort)55640);
                Debug.Assert(pack.first_message_offset == (byte)(byte)182);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.sequence = (ushort)(ushort)55640;
            p267.target_component = (byte)(byte)151;
            p267.length = (byte)(byte)79;
            p267.data__SET(new byte[] {(byte)250, (byte)216, (byte)93, (byte)22, (byte)168, (byte)24, (byte)197, (byte)51, (byte)126, (byte)36, (byte)117, (byte)201, (byte)114, (byte)202, (byte)200, (byte)92, (byte)17, (byte)106, (byte)87, (byte)210, (byte)119, (byte)23, (byte)160, (byte)148, (byte)17, (byte)199, (byte)217, (byte)150, (byte)39, (byte)45, (byte)82, (byte)32, (byte)4, (byte)7, (byte)234, (byte)221, (byte)169, (byte)93, (byte)48, (byte)114, (byte)164, (byte)175, (byte)234, (byte)178, (byte)184, (byte)121, (byte)228, (byte)55, (byte)108, (byte)36, (byte)54, (byte)222, (byte)66, (byte)227, (byte)232, (byte)203, (byte)249, (byte)129, (byte)93, (byte)7, (byte)227, (byte)66, (byte)152, (byte)131, (byte)92, (byte)32, (byte)217, (byte)98, (byte)228, (byte)101, (byte)240, (byte)39, (byte)166, (byte)246, (byte)112, (byte)235, (byte)205, (byte)51, (byte)59, (byte)48, (byte)178, (byte)40, (byte)211, (byte)248, (byte)179, (byte)152, (byte)93, (byte)252, (byte)176, (byte)17, (byte)139, (byte)9, (byte)148, (byte)124, (byte)234, (byte)150, (byte)129, (byte)66, (byte)51, (byte)198, (byte)215, (byte)140, (byte)128, (byte)28, (byte)161, (byte)5, (byte)123, (byte)251, (byte)139, (byte)186, (byte)72, (byte)190, (byte)11, (byte)174, (byte)163, (byte)94, (byte)83, (byte)229, (byte)229, (byte)48, (byte)89, (byte)161, (byte)189, (byte)47, (byte)252, (byte)161, (byte)108, (byte)91, (byte)21, (byte)80, (byte)155, (byte)60, (byte)37, (byte)101, (byte)122, (byte)16, (byte)118, (byte)163, (byte)172, (byte)72, (byte)182, (byte)176, (byte)16, (byte)165, (byte)64, (byte)105, (byte)121, (byte)170, (byte)208, (byte)117, (byte)5, (byte)80, (byte)32, (byte)193, (byte)83, (byte)9, (byte)236, (byte)91, (byte)190, (byte)107, (byte)43, (byte)240, (byte)248, (byte)102, (byte)27, (byte)69, (byte)56, (byte)98, (byte)26, (byte)159, (byte)252, (byte)226, (byte)63, (byte)11, (byte)36, (byte)108, (byte)244, (byte)8, (byte)215, (byte)173, (byte)107, (byte)221, (byte)108, (byte)119, (byte)230, (byte)253, (byte)167, (byte)234, (byte)147, (byte)98, (byte)102, (byte)118, (byte)106, (byte)226, (byte)80, (byte)161, (byte)113, (byte)172, (byte)53, (byte)42, (byte)114, (byte)112, (byte)4, (byte)7, (byte)0, (byte)104, (byte)17, (byte)83, (byte)1, (byte)85, (byte)111, (byte)36, (byte)92, (byte)138, (byte)129, (byte)233, (byte)158, (byte)130, (byte)4, (byte)212, (byte)173, (byte)229, (byte)144, (byte)168, (byte)155, (byte)233, (byte)43, (byte)158, (byte)254, (byte)158, (byte)243, (byte)111, (byte)145, (byte)235, (byte)144, (byte)203, (byte)61, (byte)75, (byte)254, (byte)125, (byte)39, (byte)208, (byte)35, (byte)118, (byte)202, (byte)192, (byte)110, (byte)190, (byte)158}, 0) ;
            p267.target_system = (byte)(byte)21;
            p267.first_message_offset = (byte)(byte)182;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)238);
                Debug.Assert(pack.target_component == (byte)(byte)72);
                Debug.Assert(pack.sequence == (ushort)(ushort)54847);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_component = (byte)(byte)72;
            p268.target_system = (byte)(byte)238;
            p268.sequence = (ushort)(ushort)54847;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.bitrate == (uint)3859400545U);
                Debug.Assert(pack.camera_id == (byte)(byte)192);
                Debug.Assert(pack.rotation == (ushort)(ushort)40840);
                Debug.Assert(pack.status == (byte)(byte)176);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)29352);
                Debug.Assert(pack.uri_LEN(ph) == 124);
                Debug.Assert(pack.uri_TRY(ph).Equals("UhjjcttuabvklVjLlcopdbiQvzCpgewtkpzmfnuwrcfqwutkanjsbmgqJbwfqwqGfPtdukcliehsxqsgehzswqpdekNfmjIoGyuekgJeyxppYafyzbHuikdjivqn"));
                Debug.Assert(pack.resolution_h == (ushort)(ushort)10416);
                Debug.Assert(pack.framerate == (float)1.4417682E38F);
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.bitrate = (uint)3859400545U;
            p269.camera_id = (byte)(byte)192;
            p269.status = (byte)(byte)176;
            p269.framerate = (float)1.4417682E38F;
            p269.rotation = (ushort)(ushort)40840;
            p269.uri_SET("UhjjcttuabvklVjLlcopdbiQvzCpgewtkpzmfnuwrcfqwutkanjsbmgqJbwfqwqGfPtdukcliehsxqsgehzswqpdekNfmjIoGyuekgJeyxppYafyzbHuikdjivqn", PH) ;
            p269.resolution_h = (ushort)(ushort)10416;
            p269.resolution_v = (ushort)(ushort)29352;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.bitrate == (uint)4251109455U);
                Debug.Assert(pack.uri_LEN(ph) == 152);
                Debug.Assert(pack.uri_TRY(ph).Equals("vbrtgynthtthqCtewmhrlohudsskhdcvoJzatfqrCcentJyMamwmdkycbwpfQrsYTigdoIYvupulzyfazDmypwjlnmblxchkutckjugvbygnwgsqfvmwtvnczAQefiwsfxjbErfIcczzZnvEfccurwph"));
                Debug.Assert(pack.target_system == (byte)(byte)126);
                Debug.Assert(pack.camera_id == (byte)(byte)194);
                Debug.Assert(pack.rotation == (ushort)(ushort)65109);
                Debug.Assert(pack.target_component == (byte)(byte)218);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)19426);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)44657);
                Debug.Assert(pack.framerate == (float) -5.7114545E37F);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.resolution_h = (ushort)(ushort)19426;
            p270.bitrate = (uint)4251109455U;
            p270.resolution_v = (ushort)(ushort)44657;
            p270.rotation = (ushort)(ushort)65109;
            p270.framerate = (float) -5.7114545E37F;
            p270.target_component = (byte)(byte)218;
            p270.target_system = (byte)(byte)126;
            p270.uri_SET("vbrtgynthtthqCtewmhrlohudsskhdcvoJzatfqrCcentJyMamwmdkycbwpfQrsYTigdoIYvupulzyfazDmypwjlnmblxchkutckjugvbygnwgsqfvmwtvnczAQefiwsfxjbErfIcczzZnvEfccurwph", PH) ;
            p270.camera_id = (byte)(byte)194;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 58);
                Debug.Assert(pack.password_TRY(ph).Equals("HvyuNtjojyrgwlhiplywVcZhefawdgvRpcliIPqsywrhgprOzvoixbxxUm"));
                Debug.Assert(pack.ssid_LEN(ph) == 29);
                Debug.Assert(pack.ssid_TRY(ph).Equals("eihffcyagcxfdhcYhryWgnetwyYpm"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.password_SET("HvyuNtjojyrgwlhiplywVcZhefawdgvRpcliIPqsywrhgprOzvoixbxxUm", PH) ;
            p299.ssid_SET("eihffcyagcxfdhcYhryWgnetwyYpm", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_version == (ushort)(ushort)15748);
                Debug.Assert(pack.version == (ushort)(ushort)5445);
                Debug.Assert(pack.max_version == (ushort)(ushort)45782);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)28, (byte)141, (byte)87, (byte)52, (byte)83, (byte)110, (byte)160, (byte)67}));
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)19, (byte)221, (byte)236, (byte)235, (byte)32, (byte)79, (byte)91, (byte)147}));
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.max_version = (ushort)(ushort)45782;
            p300.min_version = (ushort)(ushort)15748;
            p300.library_version_hash_SET(new byte[] {(byte)28, (byte)141, (byte)87, (byte)52, (byte)83, (byte)110, (byte)160, (byte)67}, 0) ;
            p300.version = (ushort)(ushort)5445;
            p300.spec_version_hash_SET(new byte[] {(byte)19, (byte)221, (byte)236, (byte)235, (byte)32, (byte)79, (byte)91, (byte)147}, 0) ;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uptime_sec == (uint)3487178959U);
                Debug.Assert(pack.time_usec == (ulong)2667530112798659955L);
                Debug.Assert(pack.sub_mode == (byte)(byte)142);
                Debug.Assert(pack.health == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
                Debug.Assert(pack.mode == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)15243);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.vendor_specific_status_code = (ushort)(ushort)15243;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL;
            p310.sub_mode = (byte)(byte)142;
            p310.uptime_sec = (uint)3487178959U;
            p310.time_usec = (ulong)2667530112798659955L;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hw_version_major == (byte)(byte)131);
                Debug.Assert(pack.time_usec == (ulong)6350160084856248701L);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)0);
                Debug.Assert(pack.sw_version_major == (byte)(byte)93);
                Debug.Assert(pack.sw_vcs_commit == (uint)4076577436U);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)110, (byte)113, (byte)86, (byte)175, (byte)238, (byte)8, (byte)205, (byte)163, (byte)230, (byte)201, (byte)240, (byte)33, (byte)12, (byte)111, (byte)167, (byte)60}));
                Debug.Assert(pack.uptime_sec == (uint)975773086U);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)87);
                Debug.Assert(pack.name_LEN(ph) == 55);
                Debug.Assert(pack.name_TRY(ph).Equals("tuLbRpfdqkScsghyNwcsslrgboficigmavxadTpkplmTpnghhfvLrds"));
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.sw_vcs_commit = (uint)4076577436U;
            p311.time_usec = (ulong)6350160084856248701L;
            p311.name_SET("tuLbRpfdqkScsghyNwcsslrgboficigmavxadTpkplmTpnghhfvLrds", PH) ;
            p311.uptime_sec = (uint)975773086U;
            p311.sw_version_minor = (byte)(byte)87;
            p311.hw_version_minor = (byte)(byte)0;
            p311.hw_version_major = (byte)(byte)131;
            p311.hw_unique_id_SET(new byte[] {(byte)110, (byte)113, (byte)86, (byte)175, (byte)238, (byte)8, (byte)205, (byte)163, (byte)230, (byte)201, (byte)240, (byte)33, (byte)12, (byte)111, (byte)167, (byte)60}, 0) ;
            p311.sw_version_major = (byte)(byte)93;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (short)(short)15949);
                Debug.Assert(pack.target_component == (byte)(byte)194);
                Debug.Assert(pack.target_system == (byte)(byte)145);
                Debug.Assert(pack.param_id_LEN(ph) == 12);
                Debug.Assert(pack.param_id_TRY(ph).Equals("bzaNhbwyavwf"));
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.param_id_SET("bzaNhbwyavwf", PH) ;
            p320.target_component = (byte)(byte)194;
            p320.param_index = (short)(short)15949;
            p320.target_system = (byte)(byte)145;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)135);
                Debug.Assert(pack.target_system == (byte)(byte)23);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_component = (byte)(byte)135;
            p321.target_system = (byte)(byte)23;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
                Debug.Assert(pack.param_id_LEN(ph) == 12);
                Debug.Assert(pack.param_id_TRY(ph).Equals("dsVxwkkwlbea"));
                Debug.Assert(pack.param_value_LEN(ph) == 110);
                Debug.Assert(pack.param_value_TRY(ph).Equals("rclosfkFcxtrjmramQkdzGJnatryllloparnazzjgwgfndwokHdyljwyyztxlpbgtjsezmlgGtDukgxaqzsFtczDrnrVtzxcftCqqmyoVblqqo"));
                Debug.Assert(pack.param_count == (ushort)(ushort)3878);
                Debug.Assert(pack.param_index == (ushort)(ushort)14483);
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            p322.param_id_SET("dsVxwkkwlbea", PH) ;
            p322.param_value_SET("rclosfkFcxtrjmramQkdzGJnatryllloparnazzjgwgfndwokHdyljwyyztxlpbgtjsezmlgGtDukgxaqzsFtczDrnrVtzxcftCqqmyoVblqqo", PH) ;
            p322.param_count = (ushort)(ushort)3878;
            p322.param_index = (ushort)(ushort)14483;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)111);
                Debug.Assert(pack.target_system == (byte)(byte)139);
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("OGlzYiIm"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
                Debug.Assert(pack.param_value_LEN(ph) == 28);
                Debug.Assert(pack.param_value_TRY(ph).Equals("gkdckffjejrZpdcQzDLaqimmfyye"));
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_id_SET("OGlzYiIm", PH) ;
            p323.target_component = (byte)(byte)111;
            p323.param_value_SET("gkdckffjejrZpdcQzDLaqimmfyye", PH) ;
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16;
            p323.target_system = (byte)(byte)139;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
                Debug.Assert(pack.param_result == PARAM_ACK.PARAM_ACK_ACCEPTED);
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("jwr"));
                Debug.Assert(pack.param_value_LEN(ph) == 107);
                Debug.Assert(pack.param_value_TRY(ph).Equals("ohkPacbgfmsioqqIblptohnmzxpuprzslKwutbjkSumJFehjilmwzJpeoWeiespsdwomswrLitZoowyqsfjvubAIecjmrvzgkkujnBhkbur"));
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_value_SET("ohkPacbgfmsioqqIblptohnmzxpuprzslKwutbjkSumJFehjilmwzJpeoWeiespsdwomswrLitZoowyqsfjvubAIecjmrvzgkkujnBhkbur", PH) ;
            p324.param_id_SET("jwr", PH) ;
            p324.param_result = PARAM_ACK.PARAM_ACK_ACCEPTED;
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.increment == (byte)(byte)34);
                Debug.Assert(pack.time_usec == (ulong)7618845281460209145L);
                Debug.Assert(pack.max_distance == (ushort)(ushort)10141);
                Debug.Assert(pack.min_distance == (ushort)(ushort)53613);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)31539, (ushort)14667, (ushort)46787, (ushort)62298, (ushort)33704, (ushort)4707, (ushort)1988, (ushort)55092, (ushort)22586, (ushort)43263, (ushort)5664, (ushort)14884, (ushort)63318, (ushort)13426, (ushort)33178, (ushort)2771, (ushort)56738, (ushort)17879, (ushort)19833, (ushort)6220, (ushort)7655, (ushort)48463, (ushort)18191, (ushort)11314, (ushort)50787, (ushort)43073, (ushort)7804, (ushort)50540, (ushort)62433, (ushort)7133, (ushort)64314, (ushort)13937, (ushort)58332, (ushort)62117, (ushort)9430, (ushort)54408, (ushort)32564, (ushort)196, (ushort)60362, (ushort)20719, (ushort)29697, (ushort)45740, (ushort)41140, (ushort)53723, (ushort)19008, (ushort)25498, (ushort)49513, (ushort)19083, (ushort)42560, (ushort)20691, (ushort)14872, (ushort)33531, (ushort)55818, (ushort)8152, (ushort)20061, (ushort)41003, (ushort)31473, (ushort)37751, (ushort)24355, (ushort)29942, (ushort)44101, (ushort)16755, (ushort)62304, (ushort)40431, (ushort)7689, (ushort)57561, (ushort)6362, (ushort)6863, (ushort)520, (ushort)32015, (ushort)47515, (ushort)56262}));
                Debug.Assert(pack.sensor_type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.max_distance = (ushort)(ushort)10141;
            p330.time_usec = (ulong)7618845281460209145L;
            p330.increment = (byte)(byte)34;
            p330.distances_SET(new ushort[] {(ushort)31539, (ushort)14667, (ushort)46787, (ushort)62298, (ushort)33704, (ushort)4707, (ushort)1988, (ushort)55092, (ushort)22586, (ushort)43263, (ushort)5664, (ushort)14884, (ushort)63318, (ushort)13426, (ushort)33178, (ushort)2771, (ushort)56738, (ushort)17879, (ushort)19833, (ushort)6220, (ushort)7655, (ushort)48463, (ushort)18191, (ushort)11314, (ushort)50787, (ushort)43073, (ushort)7804, (ushort)50540, (ushort)62433, (ushort)7133, (ushort)64314, (ushort)13937, (ushort)58332, (ushort)62117, (ushort)9430, (ushort)54408, (ushort)32564, (ushort)196, (ushort)60362, (ushort)20719, (ushort)29697, (ushort)45740, (ushort)41140, (ushort)53723, (ushort)19008, (ushort)25498, (ushort)49513, (ushort)19083, (ushort)42560, (ushort)20691, (ushort)14872, (ushort)33531, (ushort)55818, (ushort)8152, (ushort)20061, (ushort)41003, (ushort)31473, (ushort)37751, (ushort)24355, (ushort)29942, (ushort)44101, (ushort)16755, (ushort)62304, (ushort)40431, (ushort)7689, (ushort)57561, (ushort)6362, (ushort)6863, (ushort)520, (ushort)32015, (ushort)47515, (ushort)56262}, 0) ;
            p330.min_distance = (ushort)(ushort)53613;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
        }
    }
}