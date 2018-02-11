
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
                    ulong id = id__a(value);
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
                    ulong id = id__k(value);
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
                    ulong id = id__k(value);
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
                    ulong id = id__k(value);
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
            *	 on the URI type enum*/
            public byte[] uri
            {
                get {return uri_GET(new byte[120], 0);}
            }
            /**
            *The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends
            *	 on the URI type enum*/
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
            *	 has a storage associated (e.g. MAVLink FTP)*/
            public byte[] storage
            {
                get {return storage_GET(new byte[120], 0);}
            }
            /**
            *The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type
            *	 has a storage associated (e.g. MAVLink FTP)*/
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
            *	 should have the UINT16_MAX value*/
            public ushort[] voltages
            {
                get {return voltages_GET(new ushort[10], 0);}
            }
            /**
            *Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery
            *	 should have the UINT16_MAX value*/
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
            *	 energy consumption estimat*/
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
                Debug.Assert(pack.type == MAV_TYPE.MAV_TYPE_VTOL_RESERVED3);
                Debug.Assert(pack.autopilot == MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_MISSION_FULL);
                Debug.Assert(pack.custom_mode == (uint)2546856919U);
                Debug.Assert(pack.system_status == MAV_STATE.MAV_STATE_CALIBRATING);
                Debug.Assert(pack.mavlink_version == (byte)(byte)169);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED));
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.custom_mode = (uint)2546856919U;
            p0.system_status = MAV_STATE.MAV_STATE_CALIBRATING;
            p0.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED);
            p0.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_MISSION_FULL;
            p0.type = MAV_TYPE.MAV_TYPE_VTOL_RESERVED3;
            p0.mavlink_version = (byte)(byte)169;
            SMP_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)7704);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)34179);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 20);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)1788);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)51516);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)31981);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)57926);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL));
                Debug.Assert(pack.current_battery == (short)(short)32519);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)37523);
                Debug.Assert(pack.load == (ushort)(ushort)20867);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.current_battery = (short)(short)32519;
            p1.errors_count1 = (ushort)(ushort)57926;
            p1.errors_count3 = (ushort)(ushort)34179;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.errors_count4 = (ushort)(ushort)1788;
            p1.errors_count2 = (ushort)(ushort)7704;
            p1.voltage_battery = (ushort)(ushort)51516;
            p1.errors_comm = (ushort)(ushort)37523;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL);
            p1.battery_remaining = (sbyte)(sbyte) - 20;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL);
            p1.drop_rate_comm = (ushort)(ushort)31981;
            p1.load = (ushort)(ushort)20867;
            SMP_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)858633333U);
                Debug.Assert(pack.time_unix_usec == (ulong)4128278993053187445L);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_boot_ms = (uint)858633333U;
            p2.time_unix_usec = (ulong)4128278993053187445L;
            SMP_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw_rate == (float) -3.755372E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3823545019U);
                Debug.Assert(pack.y == (float)3.1957124E37F);
                Debug.Assert(pack.afz == (float)2.6093945E38F);
                Debug.Assert(pack.vy == (float)2.913825E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.afy == (float)9.692988E37F);
                Debug.Assert(pack.vx == (float) -1.4896094E38F);
                Debug.Assert(pack.z == (float) -3.312505E37F);
                Debug.Assert(pack.x == (float) -3.072909E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)6401);
                Debug.Assert(pack.yaw == (float) -1.1665097E38F);
                Debug.Assert(pack.afx == (float)2.4389857E38F);
                Debug.Assert(pack.vz == (float) -2.7687373E38F);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.afz = (float)2.6093945E38F;
            p3.vx = (float) -1.4896094E38F;
            p3.y = (float)3.1957124E37F;
            p3.time_boot_ms = (uint)3823545019U;
            p3.vz = (float) -2.7687373E38F;
            p3.vy = (float)2.913825E38F;
            p3.x = (float) -3.072909E38F;
            p3.type_mask = (ushort)(ushort)6401;
            p3.afx = (float)2.4389857E38F;
            p3.afy = (float)9.692988E37F;
            p3.z = (float) -3.312505E37F;
            p3.yaw = (float) -1.1665097E38F;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p3.yaw_rate = (float) -3.755372E37F;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)99);
                Debug.Assert(pack.time_usec == (ulong)3347104952245250747L);
                Debug.Assert(pack.seq == (uint)2445910036U);
                Debug.Assert(pack.target_system == (byte)(byte)10);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.seq = (uint)2445910036U;
            p4.time_usec = (ulong)3347104952245250747L;
            p4.target_component = (byte)(byte)99;
            p4.target_system = (byte)(byte)10;
            SMP_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.control_request == (byte)(byte)1);
                Debug.Assert(pack.version == (byte)(byte)113);
                Debug.Assert(pack.target_system == (byte)(byte)242);
                Debug.Assert(pack.passkey_LEN(ph) == 22);
                Debug.Assert(pack.passkey_TRY(ph).Equals("vxDYlqzqvnbnRnutjissmk"));
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.passkey_SET("vxDYlqzqvnbnRnutjissmk", PH) ;
            p5.control_request = (byte)(byte)1;
            p5.target_system = (byte)(byte)242;
            p5.version = (byte)(byte)113;
            SMP_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gcs_system_id == (byte)(byte)185);
                Debug.Assert(pack.ack == (byte)(byte)126);
                Debug.Assert(pack.control_request == (byte)(byte)149);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.ack = (byte)(byte)126;
            p6.control_request = (byte)(byte)149;
            p6.gcs_system_id = (byte)(byte)185;
            SMP_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 6);
                Debug.Assert(pack.key_TRY(ph).Equals("jmgbrp"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("jmgbrp", PH) ;
            SMP_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)53);
                Debug.Assert(pack.base_mode == MAV_MODE.MAV_MODE_STABILIZE_ARMED);
                Debug.Assert(pack.custom_mode == (uint)2879512858U);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.target_system = (byte)(byte)53;
            p11.base_mode = MAV_MODE.MAV_MODE_STABILIZE_ARMED;
            p11.custom_mode = (uint)2879512858U;
            SMP_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)205);
                Debug.Assert(pack.param_index == (short)(short)28151);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("LpuZj"));
                Debug.Assert(pack.target_system == (byte)(byte)251);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_system = (byte)(byte)251;
            p20.param_index = (short)(short)28151;
            p20.target_component = (byte)(byte)205;
            p20.param_id_SET("LpuZj", PH) ;
            SMP_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)183);
                Debug.Assert(pack.target_component == (byte)(byte)157);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)157;
            p21.target_system = (byte)(byte)183;
            SMP_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (ushort)(ushort)34079);
                Debug.Assert(pack.param_count == (ushort)(ushort)53124);
                Debug.Assert(pack.param_id_LEN(ph) == 2);
                Debug.Assert(pack.param_id_TRY(ph).Equals("xm"));
                Debug.Assert(pack.param_value == (float) -2.347286E38F);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_index = (ushort)(ushort)34079;
            p22.param_value = (float) -2.347286E38F;
            p22.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32;
            p22.param_id_SET("xm", PH) ;
            p22.param_count = (ushort)(ushort)53124;
            SMP_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("kgtzgbbl"));
                Debug.Assert(pack.target_system == (byte)(byte)138);
                Debug.Assert(pack.target_component == (byte)(byte)90);
                Debug.Assert(pack.param_value == (float)2.3003145E38F);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.param_id_SET("kgtzgbbl", PH) ;
            p23.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT64;
            p23.param_value = (float)2.3003145E38F;
            p23.target_component = (byte)(byte)90;
            p23.target_system = (byte)(byte)138;
            SMP_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)1757165434U);
                Debug.Assert(pack.vel == (ushort)(ushort)24350);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int)2051217095);
                Debug.Assert(pack.satellites_visible == (byte)(byte)112);
                Debug.Assert(pack.cog == (ushort)(ushort)20446);
                Debug.Assert(pack.eph == (ushort)(ushort)30286);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)3676032635U);
                Debug.Assert(pack.time_usec == (ulong)2953809963724547464L);
                Debug.Assert(pack.epv == (ushort)(ushort)17716);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
                Debug.Assert(pack.alt == (int) -1360995119);
                Debug.Assert(pack.lon == (int)1859136433);
                Debug.Assert(pack.lat == (int) -1584403926);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)2581035930U);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)700408773U);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.h_acc_SET((uint)700408773U, PH) ;
            p24.epv = (ushort)(ushort)17716;
            p24.cog = (ushort)(ushort)20446;
            p24.time_usec = (ulong)2953809963724547464L;
            p24.vel = (ushort)(ushort)24350;
            p24.lon = (int)1859136433;
            p24.satellites_visible = (byte)(byte)112;
            p24.hdg_acc_SET((uint)2581035930U, PH) ;
            p24.vel_acc_SET((uint)1757165434U, PH) ;
            p24.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_PPP;
            p24.alt_ellipsoid_SET((int)2051217095, PH) ;
            p24.alt = (int) -1360995119;
            p24.lat = (int) -1584403926;
            p24.eph = (ushort)(ushort)30286;
            p24.v_acc_SET((uint)3676032635U, PH) ;
            SMP_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)145, (byte)39, (byte)146, (byte)22, (byte)172, (byte)124, (byte)132, (byte)151, (byte)46, (byte)153, (byte)120, (byte)43, (byte)250, (byte)154, (byte)6, (byte)194, (byte)222, (byte)80, (byte)71, (byte)54}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)21, (byte)35, (byte)64, (byte)162, (byte)10, (byte)131, (byte)68, (byte)235, (byte)81, (byte)175, (byte)176, (byte)201, (byte)101, (byte)81, (byte)121, (byte)78, (byte)50, (byte)195, (byte)225, (byte)188}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)88);
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)253, (byte)115, (byte)102, (byte)173, (byte)143, (byte)165, (byte)158, (byte)43, (byte)170, (byte)23, (byte)210, (byte)126, (byte)197, (byte)135, (byte)20, (byte)255, (byte)101, (byte)208, (byte)166, (byte)77}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)79, (byte)8, (byte)151, (byte)228, (byte)116, (byte)231, (byte)244, (byte)95, (byte)29, (byte)198, (byte)162, (byte)33, (byte)51, (byte)229, (byte)59, (byte)4, (byte)73, (byte)17, (byte)205, (byte)252}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)156, (byte)59, (byte)48, (byte)84, (byte)163, (byte)157, (byte)70, (byte)231, (byte)157, (byte)121, (byte)42, (byte)87, (byte)140, (byte)38, (byte)131, (byte)176, (byte)95, (byte)6, (byte)83, (byte)105}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_used_SET(new byte[] {(byte)253, (byte)115, (byte)102, (byte)173, (byte)143, (byte)165, (byte)158, (byte)43, (byte)170, (byte)23, (byte)210, (byte)126, (byte)197, (byte)135, (byte)20, (byte)255, (byte)101, (byte)208, (byte)166, (byte)77}, 0) ;
            p25.satellites_visible = (byte)(byte)88;
            p25.satellite_azimuth_SET(new byte[] {(byte)79, (byte)8, (byte)151, (byte)228, (byte)116, (byte)231, (byte)244, (byte)95, (byte)29, (byte)198, (byte)162, (byte)33, (byte)51, (byte)229, (byte)59, (byte)4, (byte)73, (byte)17, (byte)205, (byte)252}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)156, (byte)59, (byte)48, (byte)84, (byte)163, (byte)157, (byte)70, (byte)231, (byte)157, (byte)121, (byte)42, (byte)87, (byte)140, (byte)38, (byte)131, (byte)176, (byte)95, (byte)6, (byte)83, (byte)105}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)21, (byte)35, (byte)64, (byte)162, (byte)10, (byte)131, (byte)68, (byte)235, (byte)81, (byte)175, (byte)176, (byte)201, (byte)101, (byte)81, (byte)121, (byte)78, (byte)50, (byte)195, (byte)225, (byte)188}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)145, (byte)39, (byte)146, (byte)22, (byte)172, (byte)124, (byte)132, (byte)151, (byte)46, (byte)153, (byte)120, (byte)43, (byte)250, (byte)154, (byte)6, (byte)194, (byte)222, (byte)80, (byte)71, (byte)54}, 0) ;
            SMP_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (short)(short)27618);
                Debug.Assert(pack.zgyro == (short)(short) -25528);
                Debug.Assert(pack.zacc == (short)(short) -1479);
                Debug.Assert(pack.yacc == (short)(short)3609);
                Debug.Assert(pack.ygyro == (short)(short)5222);
                Debug.Assert(pack.xacc == (short)(short)17248);
                Debug.Assert(pack.xgyro == (short)(short)14722);
                Debug.Assert(pack.zmag == (short)(short) -17014);
                Debug.Assert(pack.ymag == (short)(short)26023);
                Debug.Assert(pack.time_boot_ms == (uint)3354897721U);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.yacc = (short)(short)3609;
            p26.zacc = (short)(short) -1479;
            p26.xmag = (short)(short)27618;
            p26.time_boot_ms = (uint)3354897721U;
            p26.ygyro = (short)(short)5222;
            p26.zgyro = (short)(short) -25528;
            p26.zmag = (short)(short) -17014;
            p26.ymag = (short)(short)26023;
            p26.xacc = (short)(short)17248;
            p26.xgyro = (short)(short)14722;
            SMP_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (short)(short)26691);
                Debug.Assert(pack.zacc == (short)(short) -20499);
                Debug.Assert(pack.ygyro == (short)(short) -8582);
                Debug.Assert(pack.xacc == (short)(short) -32732);
                Debug.Assert(pack.zmag == (short)(short) -4532);
                Debug.Assert(pack.yacc == (short)(short)7078);
                Debug.Assert(pack.ymag == (short)(short) -14988);
                Debug.Assert(pack.xmag == (short)(short)23014);
                Debug.Assert(pack.zgyro == (short)(short)31732);
                Debug.Assert(pack.time_usec == (ulong)909233752376504311L);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.zmag = (short)(short) -4532;
            p27.time_usec = (ulong)909233752376504311L;
            p27.yacc = (short)(short)7078;
            p27.zacc = (short)(short) -20499;
            p27.xgyro = (short)(short)26691;
            p27.zgyro = (short)(short)31732;
            p27.xacc = (short)(short) -32732;
            p27.ygyro = (short)(short) -8582;
            p27.ymag = (short)(short) -14988;
            p27.xmag = (short)(short)23014;
            SMP_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (short)(short) -12156);
                Debug.Assert(pack.press_diff1 == (short)(short) -1346);
                Debug.Assert(pack.time_usec == (ulong)2538124893360494276L);
                Debug.Assert(pack.temperature == (short)(short)9233);
                Debug.Assert(pack.press_diff2 == (short)(short)23258);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_diff1 = (short)(short) -1346;
            p28.time_usec = (ulong)2538124893360494276L;
            p28.press_diff2 = (short)(short)23258;
            p28.press_abs = (short)(short) -12156;
            p28.temperature = (short)(short)9233;
            SMP_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1107333502U);
                Debug.Assert(pack.press_diff == (float) -7.514101E37F);
                Debug.Assert(pack.press_abs == (float) -2.2012828E38F);
                Debug.Assert(pack.temperature == (short)(short)29370);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.temperature = (short)(short)29370;
            p29.time_boot_ms = (uint)1107333502U;
            p29.press_diff = (float) -7.514101E37F;
            p29.press_abs = (float) -2.2012828E38F;
            SMP_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)1.8239373E38F);
                Debug.Assert(pack.pitchspeed == (float) -2.5764457E38F);
                Debug.Assert(pack.pitch == (float)2.3231786E38F);
                Debug.Assert(pack.roll == (float)9.908676E36F);
                Debug.Assert(pack.time_boot_ms == (uint)2702363993U);
                Debug.Assert(pack.rollspeed == (float)2.0963862E38F);
                Debug.Assert(pack.yawspeed == (float) -2.909575E38F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.yaw = (float)1.8239373E38F;
            p30.pitchspeed = (float) -2.5764457E38F;
            p30.yawspeed = (float) -2.909575E38F;
            p30.pitch = (float)2.3231786E38F;
            p30.rollspeed = (float)2.0963862E38F;
            p30.time_boot_ms = (uint)2702363993U;
            p30.roll = (float)9.908676E36F;
            SMP_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q1 == (float) -2.320481E38F);
                Debug.Assert(pack.q2 == (float) -1.960743E38F);
                Debug.Assert(pack.q3 == (float)1.176975E38F);
                Debug.Assert(pack.rollspeed == (float)3.1094498E38F);
                Debug.Assert(pack.q4 == (float) -2.0260524E36F);
                Debug.Assert(pack.pitchspeed == (float) -8.840763E36F);
                Debug.Assert(pack.time_boot_ms == (uint)2988786143U);
                Debug.Assert(pack.yawspeed == (float)1.1386536E37F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.pitchspeed = (float) -8.840763E36F;
            p31.time_boot_ms = (uint)2988786143U;
            p31.rollspeed = (float)3.1094498E38F;
            p31.yawspeed = (float)1.1386536E37F;
            p31.q4 = (float) -2.0260524E36F;
            p31.q3 = (float)1.176975E38F;
            p31.q1 = (float) -2.320481E38F;
            p31.q2 = (float) -1.960743E38F;
            SMP_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float) -1.9687368E38F);
                Debug.Assert(pack.y == (float) -2.0262748E38F);
                Debug.Assert(pack.x == (float)4.4495185E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1102496392U);
                Debug.Assert(pack.vx == (float) -2.3382158E38F);
                Debug.Assert(pack.z == (float)1.7003843E38F);
                Debug.Assert(pack.vz == (float) -2.769049E38F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.vx = (float) -2.3382158E38F;
            p32.time_boot_ms = (uint)1102496392U;
            p32.vy = (float) -1.9687368E38F;
            p32.x = (float)4.4495185E37F;
            p32.y = (float) -2.0262748E38F;
            p32.vz = (float) -2.769049E38F;
            p32.z = (float)1.7003843E38F;
            SMP_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hdg == (ushort)(ushort)42927);
                Debug.Assert(pack.time_boot_ms == (uint)431897932U);
                Debug.Assert(pack.vy == (short)(short)8575);
                Debug.Assert(pack.alt == (int)604179967);
                Debug.Assert(pack.relative_alt == (int) -521484269);
                Debug.Assert(pack.lon == (int)1925120480);
                Debug.Assert(pack.vx == (short)(short) -28264);
                Debug.Assert(pack.lat == (int) -1083271156);
                Debug.Assert(pack.vz == (short)(short)21572);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.lat = (int) -1083271156;
            p33.vx = (short)(short) -28264;
            p33.vz = (short)(short)21572;
            p33.relative_alt = (int) -521484269;
            p33.hdg = (ushort)(ushort)42927;
            p33.alt = (int)604179967;
            p33.lon = (int)1925120480;
            p33.vy = (short)(short)8575;
            p33.time_boot_ms = (uint)431897932U;
            SMP_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan5_scaled == (short)(short)16915);
                Debug.Assert(pack.chan3_scaled == (short)(short) -16204);
                Debug.Assert(pack.chan7_scaled == (short)(short)29655);
                Debug.Assert(pack.rssi == (byte)(byte)27);
                Debug.Assert(pack.port == (byte)(byte)96);
                Debug.Assert(pack.chan6_scaled == (short)(short)8785);
                Debug.Assert(pack.chan2_scaled == (short)(short)3761);
                Debug.Assert(pack.time_boot_ms == (uint)4490491U);
                Debug.Assert(pack.chan1_scaled == (short)(short)32394);
                Debug.Assert(pack.chan8_scaled == (short)(short) -14057);
                Debug.Assert(pack.chan4_scaled == (short)(short)29979);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.time_boot_ms = (uint)4490491U;
            p34.chan1_scaled = (short)(short)32394;
            p34.chan3_scaled = (short)(short) -16204;
            p34.port = (byte)(byte)96;
            p34.chan5_scaled = (short)(short)16915;
            p34.chan2_scaled = (short)(short)3761;
            p34.chan8_scaled = (short)(short) -14057;
            p34.chan4_scaled = (short)(short)29979;
            p34.chan6_scaled = (short)(short)8785;
            p34.rssi = (byte)(byte)27;
            p34.chan7_scaled = (short)(short)29655;
            SMP_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)8058);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)25290);
                Debug.Assert(pack.rssi == (byte)(byte)210);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)2111);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)51715);
                Debug.Assert(pack.port == (byte)(byte)116);
                Debug.Assert(pack.time_boot_ms == (uint)814177731U);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)23805);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)45109);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)52526);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)20204);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan2_raw = (ushort)(ushort)20204;
            p35.rssi = (byte)(byte)210;
            p35.chan1_raw = (ushort)(ushort)51715;
            p35.chan8_raw = (ushort)(ushort)23805;
            p35.chan3_raw = (ushort)(ushort)8058;
            p35.chan5_raw = (ushort)(ushort)2111;
            p35.time_boot_ms = (uint)814177731U;
            p35.chan4_raw = (ushort)(ushort)25290;
            p35.port = (byte)(byte)116;
            p35.chan7_raw = (ushort)(ushort)45109;
            p35.chan6_raw = (ushort)(ushort)52526;
            SMP_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)16472);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)34020);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)52311);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)50250);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)8880);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)21571);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)13405);
                Debug.Assert(pack.port == (byte)(byte)50);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)52413);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)62960);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)24689);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)20487);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)7101);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)9023);
                Debug.Assert(pack.time_usec == (uint)4223796079U);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)56523);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)56574);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)18513);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo3_raw = (ushort)(ushort)18513;
            p36.time_usec = (uint)4223796079U;
            p36.servo7_raw = (ushort)(ushort)24689;
            p36.servo4_raw = (ushort)(ushort)9023;
            p36.servo10_raw_SET((ushort)(ushort)56523, PH) ;
            p36.servo5_raw = (ushort)(ushort)34020;
            p36.servo14_raw_SET((ushort)(ushort)21571, PH) ;
            p36.servo6_raw = (ushort)(ushort)20487;
            p36.servo8_raw = (ushort)(ushort)62960;
            p36.servo12_raw_SET((ushort)(ushort)52413, PH) ;
            p36.port = (byte)(byte)50;
            p36.servo2_raw = (ushort)(ushort)56574;
            p36.servo1_raw = (ushort)(ushort)50250;
            p36.servo9_raw_SET((ushort)(ushort)8880, PH) ;
            p36.servo11_raw_SET((ushort)(ushort)13405, PH) ;
            p36.servo13_raw_SET((ushort)(ushort)7101, PH) ;
            p36.servo15_raw_SET((ushort)(ushort)16472, PH) ;
            p36.servo16_raw_SET((ushort)(ushort)52311, PH) ;
            SMP_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.end_index == (short)(short) -1598);
                Debug.Assert(pack.target_system == (byte)(byte)100);
                Debug.Assert(pack.start_index == (short)(short)17513);
                Debug.Assert(pack.target_component == (byte)(byte)166);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p37.end_index = (short)(short) -1598;
            p37.target_component = (byte)(byte)166;
            p37.target_system = (byte)(byte)100;
            p37.start_index = (short)(short)17513;
            SMP_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_index == (short)(short)29652);
                Debug.Assert(pack.target_system == (byte)(byte)124);
                Debug.Assert(pack.end_index == (short)(short)25335);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_component == (byte)(byte)70);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p38.target_component = (byte)(byte)70;
            p38.end_index = (short)(short)25335;
            p38.start_index = (short)(short)29652;
            p38.target_system = (byte)(byte)124;
            SMP_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param3 == (float)1.8194003E38F);
                Debug.Assert(pack.param1 == (float) -1.0736764E38F);
                Debug.Assert(pack.target_component == (byte)(byte)11);
                Debug.Assert(pack.current == (byte)(byte)30);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.x == (float) -2.9534758E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_SET_RELAY);
                Debug.Assert(pack.param2 == (float) -1.841736E38F);
                Debug.Assert(pack.seq == (ushort)(ushort)5810);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.param4 == (float)2.667175E38F);
                Debug.Assert(pack.z == (float)7.2668116E37F);
                Debug.Assert(pack.y == (float)3.0641348E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)248);
                Debug.Assert(pack.target_system == (byte)(byte)242);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.target_component = (byte)(byte)11;
            p39.current = (byte)(byte)30;
            p39.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p39.param2 = (float) -1.841736E38F;
            p39.target_system = (byte)(byte)242;
            p39.param4 = (float)2.667175E38F;
            p39.seq = (ushort)(ushort)5810;
            p39.y = (float)3.0641348E38F;
            p39.param3 = (float)1.8194003E38F;
            p39.autocontinue = (byte)(byte)248;
            p39.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p39.command = MAV_CMD.MAV_CMD_DO_SET_RELAY;
            p39.param1 = (float) -1.0736764E38F;
            p39.z = (float)7.2668116E37F;
            p39.x = (float) -2.9534758E38F;
            SMP_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)78);
                Debug.Assert(pack.target_system == (byte)(byte)17);
                Debug.Assert(pack.seq == (ushort)(ushort)19272);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_component = (byte)(byte)78;
            p40.seq = (ushort)(ushort)19272;
            p40.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p40.target_system = (byte)(byte)17;
            SMP_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)36);
                Debug.Assert(pack.seq == (ushort)(ushort)51911);
                Debug.Assert(pack.target_system == (byte)(byte)128);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.seq = (ushort)(ushort)51911;
            p41.target_component = (byte)(byte)36;
            p41.target_system = (byte)(byte)128;
            SMP_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)2206);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)2206;
            SMP_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)169);
                Debug.Assert(pack.target_component == (byte)(byte)47);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_component = (byte)(byte)47;
            p43.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p43.target_system = (byte)(byte)169;
            SMP_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_system == (byte)(byte)138);
                Debug.Assert(pack.count == (ushort)(ushort)23764);
                Debug.Assert(pack.target_component == (byte)(byte)19);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.target_component = (byte)(byte)19;
            p44.count = (ushort)(ushort)23764;
            p44.target_system = (byte)(byte)138;
            p44.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            SMP_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)17);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)214);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_system = (byte)(byte)17;
            p45.target_component = (byte)(byte)214;
            p45.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            SMP_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)10448);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)10448;
            SMP_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)233);
                Debug.Assert(pack.type == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM5_X);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_system == (byte)(byte)114);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p47.type = MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM5_X;
            p47.target_component = (byte)(byte)233;
            p47.target_system = (byte)(byte)114;
            SMP_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)3776531232668013148L);
                Debug.Assert(pack.latitude == (int)1725777464);
                Debug.Assert(pack.target_system == (byte)(byte)189);
                Debug.Assert(pack.longitude == (int) -1560379753);
                Debug.Assert(pack.altitude == (int)1378603503);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.longitude = (int) -1560379753;
            p48.altitude = (int)1378603503;
            p48.target_system = (byte)(byte)189;
            p48.time_usec_SET((ulong)3776531232668013148L, PH) ;
            p48.latitude = (int)1725777464;
            SMP_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)6066399177611933110L);
                Debug.Assert(pack.latitude == (int)482096982);
                Debug.Assert(pack.altitude == (int) -1138987203);
                Debug.Assert(pack.longitude == (int)1374606221);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.altitude = (int) -1138987203;
            p49.longitude = (int)1374606221;
            p49.time_usec_SET((ulong)6066399177611933110L, PH) ;
            p49.latitude = (int)482096982;
            SMP_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (short)(short)23814);
                Debug.Assert(pack.target_system == (byte)(byte)219);
                Debug.Assert(pack.param_value0 == (float) -1.0830721E38F);
                Debug.Assert(pack.param_value_min == (float) -9.707629E37F);
                Debug.Assert(pack.param_value_max == (float) -1.1337875E38F);
                Debug.Assert(pack.scale == (float)3.4800647E37F);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)164);
                Debug.Assert(pack.param_id_LEN(ph) == 12);
                Debug.Assert(pack.param_id_TRY(ph).Equals("Hlhpycvgnwnd"));
                Debug.Assert(pack.target_component == (byte)(byte)193);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.param_index = (short)(short)23814;
            p50.param_value_max = (float) -1.1337875E38F;
            p50.parameter_rc_channel_index = (byte)(byte)164;
            p50.param_id_SET("Hlhpycvgnwnd", PH) ;
            p50.param_value0 = (float) -1.0830721E38F;
            p50.param_value_min = (float) -9.707629E37F;
            p50.scale = (float)3.4800647E37F;
            p50.target_component = (byte)(byte)193;
            p50.target_system = (byte)(byte)219;
            SMP_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)63157);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_component == (byte)(byte)122);
                Debug.Assert(pack.target_system == (byte)(byte)57);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_system = (byte)(byte)57;
            p51.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p51.target_component = (byte)(byte)122;
            p51.seq = (ushort)(ushort)63157;
            SMP_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.p1z == (float) -9.3324E37F);
                Debug.Assert(pack.target_system == (byte)(byte)179);
                Debug.Assert(pack.target_component == (byte)(byte)158);
                Debug.Assert(pack.p1y == (float) -6.731238E37F);
                Debug.Assert(pack.p2y == (float)2.3070815E38F);
                Debug.Assert(pack.p2z == (float)2.1076156E38F);
                Debug.Assert(pack.p2x == (float)2.2893648E38F);
                Debug.Assert(pack.p1x == (float)1.267392E38F);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p1x = (float)1.267392E38F;
            p54.p2y = (float)2.3070815E38F;
            p54.p2x = (float)2.2893648E38F;
            p54.frame = MAV_FRAME.MAV_FRAME_MISSION;
            p54.p2z = (float)2.1076156E38F;
            p54.target_system = (byte)(byte)179;
            p54.p1z = (float) -9.3324E37F;
            p54.p1y = (float) -6.731238E37F;
            p54.target_component = (byte)(byte)158;
            SMP_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2z == (float)4.772874E36F);
                Debug.Assert(pack.p2y == (float)6.335675E37F);
                Debug.Assert(pack.p1y == (float) -1.1724319E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_NED);
                Debug.Assert(pack.p1x == (float) -3.3964163E38F);
                Debug.Assert(pack.p1z == (float)3.3316634E38F);
                Debug.Assert(pack.p2x == (float) -2.666226E38F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.frame = MAV_FRAME.MAV_FRAME_BODY_NED;
            p55.p2z = (float)4.772874E36F;
            p55.p2y = (float)6.335675E37F;
            p55.p1y = (float) -1.1724319E38F;
            p55.p1z = (float)3.3316634E38F;
            p55.p1x = (float) -3.3964163E38F;
            p55.p2x = (float) -2.666226E38F;
            SMP_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitchspeed == (float)1.9173396E38F);
                Debug.Assert(pack.yawspeed == (float)2.8704378E38F);
                Debug.Assert(pack.rollspeed == (float)1.7212503E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.5099549E38F, -1.5505967E38F, -5.844694E37F, -2.788533E37F, -2.1727148E38F, 3.059512E38F, -1.95368E38F, -2.9884607E38F, 1.2862534E38F}));
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.9445681E38F, -2.9544999E38F, -1.0212017E38F, -1.5441834E38F}));
                Debug.Assert(pack.time_usec == (ulong)1573126471304706038L);
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.rollspeed = (float)1.7212503E38F;
            p61.covariance_SET(new float[] {1.5099549E38F, -1.5505967E38F, -5.844694E37F, -2.788533E37F, -2.1727148E38F, 3.059512E38F, -1.95368E38F, -2.9884607E38F, 1.2862534E38F}, 0) ;
            p61.pitchspeed = (float)1.9173396E38F;
            p61.time_usec = (ulong)1573126471304706038L;
            p61.yawspeed = (float)2.8704378E38F;
            p61.q_SET(new float[] {-1.9445681E38F, -2.9544999E38F, -1.0212017E38F, -1.5441834E38F}, 0) ;
            SMP_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nav_bearing == (short)(short)6648);
                Debug.Assert(pack.nav_pitch == (float)1.2620281E38F);
                Debug.Assert(pack.alt_error == (float)2.5541975E38F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)50397);
                Debug.Assert(pack.target_bearing == (short)(short) -26746);
                Debug.Assert(pack.aspd_error == (float)1.7499355E38F);
                Debug.Assert(pack.xtrack_error == (float) -2.821002E38F);
                Debug.Assert(pack.nav_roll == (float) -2.5027995E38F);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_pitch = (float)1.2620281E38F;
            p62.aspd_error = (float)1.7499355E38F;
            p62.nav_bearing = (short)(short)6648;
            p62.xtrack_error = (float) -2.821002E38F;
            p62.nav_roll = (float) -2.5027995E38F;
            p62.alt_error = (float)2.5541975E38F;
            p62.wp_dist = (ushort)(ushort)50397;
            p62.target_bearing = (short)(short) -26746;
            SMP_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float)2.6014075E38F);
                Debug.Assert(pack.lat == (int)1968333278);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-1.4021304E38F, 9.492473E37F, -3.0143492E38F, -2.5447775E38F, -2.70565E38F, -1.9247664E38F, -2.5225107E38F, 2.733399E37F, -2.5000885E38F, -1.2692542E38F, 2.3720006E38F, -7.7319157E37F, -2.833186E37F, -1.9339373E38F, -1.590888E37F, 6.0591275E37F, -1.8686963E38F, 1.5345746E38F, 3.1164123E38F, -8.895575E37F, -3.1687375E38F, 1.9844298E37F, 2.0050789E38F, 8.729379E37F, -3.431796E37F, 1.6285256E38F, 3.3863992E38F, -2.977473E38F, 6.316603E37F, 9.057747E37F, -1.3195464E38F, 1.8844626E38F, -1.3090717E38F, 1.841336E38F, 9.0384565E36F, -9.560335E37F}));
                Debug.Assert(pack.alt == (int)1568760038);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
                Debug.Assert(pack.lon == (int) -174782809);
                Debug.Assert(pack.vx == (float)2.3631007E38F);
                Debug.Assert(pack.time_usec == (ulong)8783574228242320087L);
                Debug.Assert(pack.vy == (float) -1.5251226E38F);
                Debug.Assert(pack.relative_alt == (int)531058578);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.covariance_SET(new float[] {-1.4021304E38F, 9.492473E37F, -3.0143492E38F, -2.5447775E38F, -2.70565E38F, -1.9247664E38F, -2.5225107E38F, 2.733399E37F, -2.5000885E38F, -1.2692542E38F, 2.3720006E38F, -7.7319157E37F, -2.833186E37F, -1.9339373E38F, -1.590888E37F, 6.0591275E37F, -1.8686963E38F, 1.5345746E38F, 3.1164123E38F, -8.895575E37F, -3.1687375E38F, 1.9844298E37F, 2.0050789E38F, 8.729379E37F, -3.431796E37F, 1.6285256E38F, 3.3863992E38F, -2.977473E38F, 6.316603E37F, 9.057747E37F, -1.3195464E38F, 1.8844626E38F, -1.3090717E38F, 1.841336E38F, 9.0384565E36F, -9.560335E37F}, 0) ;
            p63.relative_alt = (int)531058578;
            p63.vx = (float)2.3631007E38F;
            p63.lat = (int)1968333278;
            p63.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE;
            p63.time_usec = (ulong)8783574228242320087L;
            p63.vy = (float) -1.5251226E38F;
            p63.lon = (int) -174782809;
            p63.alt = (int)1568760038;
            p63.vz = (float)2.6014075E38F;
            SMP_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float) -3.1815093E38F);
                Debug.Assert(pack.ax == (float) -2.6342595E38F);
                Debug.Assert(pack.vz == (float) -5.1371987E36F);
                Debug.Assert(pack.y == (float)2.1121562E38F);
                Debug.Assert(pack.x == (float) -1.2346575E38F);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
                Debug.Assert(pack.ay == (float)2.4731918E38F);
                Debug.Assert(pack.az == (float) -3.382069E38F);
                Debug.Assert(pack.vx == (float)3.0403338E38F);
                Debug.Assert(pack.time_usec == (ulong)685188241849136137L);
                Debug.Assert(pack.z == (float) -1.9713163E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.8558E38F, -2.9626003E38F, -1.2678195E37F, 6.983319E37F, -1.8247881E38F, -1.0412207E38F, -3.126297E38F, 1.3127446E38F, -3.6908707E37F, -3.8574367E37F, -1.9036536E38F, 2.5929805E38F, 1.71183E38F, 2.9329228E38F, -2.6943812E37F, -5.257618E37F, -2.2800075E38F, 1.9705916E37F, 7.910692E37F, -3.2491948E38F, 9.973172E37F, 2.0934049E38F, 1.5775402E38F, 2.6946662E37F, 4.370963E37F, -1.0122353E38F, 1.2747022E38F, 1.2450218E38F, 1.6504956E38F, 2.2758537E38F, 3.2105671E38F, -1.8041443E38F, -5.9518513E37F, 6.3832663E37F, -1.8734671E38F, -4.3490972E36F, 2.8416005E38F, -2.9825584E38F, -4.5090727E37F, 2.6371589E38F, 1.2738919E38F, 2.0421695E38F, -2.9971355E38F, 7.1370666E37F, -1.0212849E38F}));
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.vx = (float)3.0403338E38F;
            p64.z = (float) -1.9713163E38F;
            p64.time_usec = (ulong)685188241849136137L;
            p64.y = (float)2.1121562E38F;
            p64.ay = (float)2.4731918E38F;
            p64.ax = (float) -2.6342595E38F;
            p64.vy = (float) -3.1815093E38F;
            p64.x = (float) -1.2346575E38F;
            p64.vz = (float) -5.1371987E36F;
            p64.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS;
            p64.covariance_SET(new float[] {-2.8558E38F, -2.9626003E38F, -1.2678195E37F, 6.983319E37F, -1.8247881E38F, -1.0412207E38F, -3.126297E38F, 1.3127446E38F, -3.6908707E37F, -3.8574367E37F, -1.9036536E38F, 2.5929805E38F, 1.71183E38F, 2.9329228E38F, -2.6943812E37F, -5.257618E37F, -2.2800075E38F, 1.9705916E37F, 7.910692E37F, -3.2491948E38F, 9.973172E37F, 2.0934049E38F, 1.5775402E38F, 2.6946662E37F, 4.370963E37F, -1.0122353E38F, 1.2747022E38F, 1.2450218E38F, 1.6504956E38F, 2.2758537E38F, 3.2105671E38F, -1.8041443E38F, -5.9518513E37F, 6.3832663E37F, -1.8734671E38F, -4.3490972E36F, 2.8416005E38F, -2.9825584E38F, -4.5090727E37F, 2.6371589E38F, 1.2738919E38F, 2.0421695E38F, -2.9971355E38F, 7.1370666E37F, -1.0212849E38F}, 0) ;
            p64.az = (float) -3.382069E38F;
            SMP_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)41373);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)21367);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)47131);
                Debug.Assert(pack.rssi == (byte)(byte)240);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)60152);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)36878);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)42891);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)12835);
                Debug.Assert(pack.chancount == (byte)(byte)98);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)43264);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)51149);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)34995);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)3195);
                Debug.Assert(pack.time_boot_ms == (uint)957598720U);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)57358);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)21354);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)2438);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)26716);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)29622);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)5230);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)22973);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan11_raw = (ushort)(ushort)22973;
            p65.chan3_raw = (ushort)(ushort)60152;
            p65.chan8_raw = (ushort)(ushort)43264;
            p65.chan14_raw = (ushort)(ushort)21367;
            p65.chan15_raw = (ushort)(ushort)42891;
            p65.chan7_raw = (ushort)(ushort)5230;
            p65.chan1_raw = (ushort)(ushort)21354;
            p65.chan2_raw = (ushort)(ushort)26716;
            p65.chan12_raw = (ushort)(ushort)12835;
            p65.chan16_raw = (ushort)(ushort)29622;
            p65.chan13_raw = (ushort)(ushort)47131;
            p65.time_boot_ms = (uint)957598720U;
            p65.chan9_raw = (ushort)(ushort)3195;
            p65.chan5_raw = (ushort)(ushort)51149;
            p65.chan10_raw = (ushort)(ushort)36878;
            p65.chancount = (byte)(byte)98;
            p65.chan4_raw = (ushort)(ushort)34995;
            p65.rssi = (byte)(byte)240;
            p65.chan17_raw = (ushort)(ushort)2438;
            p65.chan6_raw = (ushort)(ushort)41373;
            p65.chan18_raw = (ushort)(ushort)57358;
            SMP_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.req_stream_id == (byte)(byte)149);
                Debug.Assert(pack.start_stop == (byte)(byte)226);
                Debug.Assert(pack.target_component == (byte)(byte)115);
                Debug.Assert(pack.target_system == (byte)(byte)37);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)51148);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.req_message_rate = (ushort)(ushort)51148;
            p66.target_system = (byte)(byte)37;
            p66.target_component = (byte)(byte)115;
            p66.start_stop = (byte)(byte)226;
            p66.req_stream_id = (byte)(byte)149;
            SMP_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.on_off == (byte)(byte)238);
                Debug.Assert(pack.message_rate == (ushort)(ushort)836);
                Debug.Assert(pack.stream_id == (byte)(byte)127);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.message_rate = (ushort)(ushort)836;
            p67.stream_id = (byte)(byte)127;
            p67.on_off = (byte)(byte)238;
            SMP_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (short)(short)19198);
                Debug.Assert(pack.buttons == (ushort)(ushort)43755);
                Debug.Assert(pack.x == (short)(short)30464);
                Debug.Assert(pack.y == (short)(short)24881);
                Debug.Assert(pack.target == (byte)(byte)248);
                Debug.Assert(pack.r == (short)(short) -12446);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.buttons = (ushort)(ushort)43755;
            p69.x = (short)(short)30464;
            p69.r = (short)(short) -12446;
            p69.target = (byte)(byte)248;
            p69.y = (short)(short)24881;
            p69.z = (short)(short)19198;
            SMP_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)22947);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)10210);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)1510);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)2004);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)26808);
                Debug.Assert(pack.target_component == (byte)(byte)193);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)27164);
                Debug.Assert(pack.target_system == (byte)(byte)145);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)419);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)54929);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.target_system = (byte)(byte)145;
            p70.chan5_raw = (ushort)(ushort)2004;
            p70.chan7_raw = (ushort)(ushort)419;
            p70.chan3_raw = (ushort)(ushort)26808;
            p70.chan8_raw = (ushort)(ushort)27164;
            p70.target_component = (byte)(byte)193;
            p70.chan2_raw = (ushort)(ushort)1510;
            p70.chan1_raw = (ushort)(ushort)10210;
            p70.chan4_raw = (ushort)(ushort)22947;
            p70.chan6_raw = (ushort)(ushort)54929;
            SMP_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (int) -1753746280);
                Debug.Assert(pack.param4 == (float)2.7025114E38F);
                Debug.Assert(pack.param2 == (float)2.6940123E36F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION);
                Debug.Assert(pack.current == (byte)(byte)76);
                Debug.Assert(pack.z == (float) -2.9279161E38F);
                Debug.Assert(pack.x == (int) -1877939226);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.autocontinue == (byte)(byte)183);
                Debug.Assert(pack.param3 == (float) -2.9148932E38F);
                Debug.Assert(pack.seq == (ushort)(ushort)40958);
                Debug.Assert(pack.target_component == (byte)(byte)87);
                Debug.Assert(pack.param1 == (float) -2.4692866E38F);
                Debug.Assert(pack.target_system == (byte)(byte)74);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.y = (int) -1753746280;
            p73.frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p73.param3 = (float) -2.9148932E38F;
            p73.target_system = (byte)(byte)74;
            p73.z = (float) -2.9279161E38F;
            p73.param2 = (float)2.6940123E36F;
            p73.param4 = (float)2.7025114E38F;
            p73.param1 = (float) -2.4692866E38F;
            p73.seq = (ushort)(ushort)40958;
            p73.x = (int) -1877939226;
            p73.target_component = (byte)(byte)87;
            p73.command = MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION;
            p73.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p73.current = (byte)(byte)76;
            p73.autocontinue = (byte)(byte)183;
            SMP_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.climb == (float)2.5566375E38F);
                Debug.Assert(pack.throttle == (ushort)(ushort)53728);
                Debug.Assert(pack.heading == (short)(short) -21403);
                Debug.Assert(pack.groundspeed == (float)2.5504296E38F);
                Debug.Assert(pack.airspeed == (float)3.225676E38F);
                Debug.Assert(pack.alt == (float)1.0972471E38F);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.heading = (short)(short) -21403;
            p74.climb = (float)2.5566375E38F;
            p74.airspeed = (float)3.225676E38F;
            p74.throttle = (ushort)(ushort)53728;
            p74.alt = (float)1.0972471E38F;
            p74.groundspeed = (float)2.5504296E38F;
            SMP_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (int) -1890892533);
                Debug.Assert(pack.target_system == (byte)(byte)58);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_GO_AROUND);
                Debug.Assert(pack.x == (int)549912889);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.param2 == (float) -1.3258103E38F);
                Debug.Assert(pack.param4 == (float)2.8802224E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)159);
                Debug.Assert(pack.param3 == (float)2.1759866E38F);
                Debug.Assert(pack.target_component == (byte)(byte)213);
                Debug.Assert(pack.current == (byte)(byte)231);
                Debug.Assert(pack.z == (float) -1.1662037E38F);
                Debug.Assert(pack.param1 == (float) -1.593464E38F);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.y = (int) -1890892533;
            p75.x = (int)549912889;
            p75.current = (byte)(byte)231;
            p75.autocontinue = (byte)(byte)159;
            p75.command = MAV_CMD.MAV_CMD_DO_GO_AROUND;
            p75.target_component = (byte)(byte)213;
            p75.param3 = (float)2.1759866E38F;
            p75.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p75.param1 = (float) -1.593464E38F;
            p75.param4 = (float)2.8802224E38F;
            p75.param2 = (float) -1.3258103E38F;
            p75.z = (float) -1.1662037E38F;
            p75.target_system = (byte)(byte)58;
            SMP_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param7 == (float)5.8884546E37F);
                Debug.Assert(pack.param3 == (float)3.1533502E38F);
                Debug.Assert(pack.param5 == (float)1.0369694E38F);
                Debug.Assert(pack.confirmation == (byte)(byte)243);
                Debug.Assert(pack.param2 == (float) -3.1630144E38F);
                Debug.Assert(pack.target_component == (byte)(byte)149);
                Debug.Assert(pack.param6 == (float)4.906998E37F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION);
                Debug.Assert(pack.param4 == (float) -2.1514196E37F);
                Debug.Assert(pack.target_system == (byte)(byte)128);
                Debug.Assert(pack.param1 == (float)1.6321998E38F);
            };
            GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.command = MAV_CMD.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
            p76.param7 = (float)5.8884546E37F;
            p76.param4 = (float) -2.1514196E37F;
            p76.param6 = (float)4.906998E37F;
            p76.param3 = (float)3.1533502E38F;
            p76.param2 = (float) -3.1630144E38F;
            p76.confirmation = (byte)(byte)243;
            p76.param1 = (float)1.6321998E38F;
            p76.target_system = (byte)(byte)128;
            p76.target_component = (byte)(byte)149;
            p76.param5 = (float)1.0369694E38F;
            CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)184);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_USER_3);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)23);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)197);
                Debug.Assert(pack.result == MAV_RESULT.MAV_RESULT_UNSUPPORTED);
                Debug.Assert(pack.result_param2_TRY(ph) == (int)732574079);
            };
            GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.target_component_SET((byte)(byte)23, PH) ;
            p77.result_param2_SET((int)732574079, PH) ;
            p77.result = MAV_RESULT.MAV_RESULT_UNSUPPORTED;
            p77.progress_SET((byte)(byte)197, PH) ;
            p77.target_system_SET((byte)(byte)184, PH) ;
            p77.command = MAV_CMD.MAV_CMD_USER_3;
            CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -2.0155267E38F);
                Debug.Assert(pack.mode_switch == (byte)(byte)234);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)90);
                Debug.Assert(pack.thrust == (float)1.0998837E38F);
                Debug.Assert(pack.time_boot_ms == (uint)400722740U);
                Debug.Assert(pack.yaw == (float)2.6956908E38F);
                Debug.Assert(pack.roll == (float)7.279863E37F);
            };
            GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.thrust = (float)1.0998837E38F;
            p81.mode_switch = (byte)(byte)234;
            p81.time_boot_ms = (uint)400722740U;
            p81.yaw = (float)2.6956908E38F;
            p81.roll = (float)7.279863E37F;
            p81.manual_override_switch = (byte)(byte)90;
            p81.pitch = (float) -2.0155267E38F;
            CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.220243E37F, 1.9915643E38F, 3.2816334E38F, -2.7690617E38F}));
                Debug.Assert(pack.type_mask == (byte)(byte)163);
                Debug.Assert(pack.body_roll_rate == (float)2.4851742E38F);
                Debug.Assert(pack.target_component == (byte)(byte)237);
                Debug.Assert(pack.target_system == (byte)(byte)6);
                Debug.Assert(pack.thrust == (float)1.6915958E38F);
                Debug.Assert(pack.body_pitch_rate == (float)2.2559921E37F);
                Debug.Assert(pack.body_yaw_rate == (float) -1.3887841E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1579669592U);
            };
            GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.target_component = (byte)(byte)237;
            p82.target_system = (byte)(byte)6;
            p82.type_mask = (byte)(byte)163;
            p82.body_pitch_rate = (float)2.2559921E37F;
            p82.time_boot_ms = (uint)1579669592U;
            p82.body_roll_rate = (float)2.4851742E38F;
            p82.body_yaw_rate = (float) -1.3887841E38F;
            p82.thrust = (float)1.6915958E38F;
            p82.q_SET(new float[] {-1.220243E37F, 1.9915643E38F, 3.2816334E38F, -2.7690617E38F}, 0) ;
            CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_pitch_rate == (float) -1.6815187E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)172);
                Debug.Assert(pack.time_boot_ms == (uint)1642407761U);
                Debug.Assert(pack.body_roll_rate == (float) -2.8845107E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.9414296E38F, 2.507603E38F, 3.3026257E38F, -3.5798937E37F}));
                Debug.Assert(pack.thrust == (float)9.006939E37F);
                Debug.Assert(pack.body_yaw_rate == (float)2.9202145E38F);
            };
            GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.body_roll_rate = (float) -2.8845107E38F;
            p83.body_pitch_rate = (float) -1.6815187E38F;
            p83.thrust = (float)9.006939E37F;
            p83.body_yaw_rate = (float)2.9202145E38F;
            p83.time_boot_ms = (uint)1642407761U;
            p83.q_SET(new float[] {1.9414296E38F, 2.507603E38F, 3.3026257E38F, -3.5798937E37F}, 0) ;
            p83.type_mask = (byte)(byte)172;
            CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)4128064473U);
                Debug.Assert(pack.target_component == (byte)(byte)108);
                Debug.Assert(pack.yaw_rate == (float)3.2670676E38F);
                Debug.Assert(pack.afz == (float)3.185587E38F);
                Debug.Assert(pack.vy == (float)2.524273E38F);
                Debug.Assert(pack.vx == (float)2.6237583E38F);
                Debug.Assert(pack.x == (float)2.355413E37F);
                Debug.Assert(pack.vz == (float) -1.5139754E38F);
                Debug.Assert(pack.afx == (float) -3.0154956E38F);
                Debug.Assert(pack.yaw == (float)3.1643798E38F);
                Debug.Assert(pack.afy == (float) -2.5961547E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.z == (float) -1.2213236E38F);
                Debug.Assert(pack.y == (float) -1.852261E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)22768);
                Debug.Assert(pack.target_system == (byte)(byte)216);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.vy = (float)2.524273E38F;
            p84.vx = (float)2.6237583E38F;
            p84.afx = (float) -3.0154956E38F;
            p84.y = (float) -1.852261E38F;
            p84.afz = (float)3.185587E38F;
            p84.target_system = (byte)(byte)216;
            p84.z = (float) -1.2213236E38F;
            p84.target_component = (byte)(byte)108;
            p84.yaw = (float)3.1643798E38F;
            p84.type_mask = (ushort)(ushort)22768;
            p84.afy = (float) -2.5961547E38F;
            p84.yaw_rate = (float)3.2670676E38F;
            p84.x = (float)2.355413E37F;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p84.time_boot_ms = (uint)4128064473U;
            p84.vz = (float) -1.5139754E38F;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.vx == (float) -9.088153E36F);
                Debug.Assert(pack.yaw == (float) -1.8725913E38F);
                Debug.Assert(pack.yaw_rate == (float) -2.4004169E38F);
                Debug.Assert(pack.lon_int == (int) -1812295493);
                Debug.Assert(pack.vz == (float) -6.40218E37F);
                Debug.Assert(pack.target_system == (byte)(byte)230);
                Debug.Assert(pack.time_boot_ms == (uint)1849019681U);
                Debug.Assert(pack.afz == (float) -2.6239643E38F);
                Debug.Assert(pack.lat_int == (int)1924161861);
                Debug.Assert(pack.target_component == (byte)(byte)94);
                Debug.Assert(pack.afx == (float)2.0057157E38F);
                Debug.Assert(pack.alt == (float)2.0392298E38F);
                Debug.Assert(pack.afy == (float)1.1922894E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)8002);
                Debug.Assert(pack.vy == (float) -2.5365934E38F);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.afz = (float) -2.6239643E38F;
            p86.vx = (float) -9.088153E36F;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p86.yaw = (float) -1.8725913E38F;
            p86.lat_int = (int)1924161861;
            p86.target_component = (byte)(byte)94;
            p86.vz = (float) -6.40218E37F;
            p86.target_system = (byte)(byte)230;
            p86.lon_int = (int) -1812295493;
            p86.alt = (float)2.0392298E38F;
            p86.time_boot_ms = (uint)1849019681U;
            p86.afy = (float)1.1922894E38F;
            p86.afx = (float)2.0057157E38F;
            p86.yaw_rate = (float) -2.4004169E38F;
            p86.vy = (float) -2.5365934E38F;
            p86.type_mask = (ushort)(ushort)8002;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon_int == (int)1481038547);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.time_boot_ms == (uint)3224205664U);
                Debug.Assert(pack.vy == (float) -1.4379291E38F);
                Debug.Assert(pack.lat_int == (int)428251116);
                Debug.Assert(pack.vz == (float) -1.2839409E38F);
                Debug.Assert(pack.vx == (float)1.4593346E37F);
                Debug.Assert(pack.yaw == (float) -7.273947E35F);
                Debug.Assert(pack.afx == (float) -1.4698516E38F);
                Debug.Assert(pack.alt == (float) -9.204684E37F);
                Debug.Assert(pack.afz == (float) -2.9351628E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)27674);
                Debug.Assert(pack.afy == (float) -1.9462712E38F);
                Debug.Assert(pack.yaw_rate == (float) -2.4459734E38F);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.afz = (float) -2.9351628E37F;
            p87.afx = (float) -1.4698516E38F;
            p87.vx = (float)1.4593346E37F;
            p87.alt = (float) -9.204684E37F;
            p87.yaw = (float) -7.273947E35F;
            p87.time_boot_ms = (uint)3224205664U;
            p87.lat_int = (int)428251116;
            p87.afy = (float) -1.9462712E38F;
            p87.type_mask = (ushort)(ushort)27674;
            p87.vz = (float) -1.2839409E38F;
            p87.lon_int = (int)1481038547;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p87.vy = (float) -1.4379291E38F;
            p87.yaw_rate = (float) -2.4459734E38F;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -1.5584982E38F);
                Debug.Assert(pack.roll == (float) -1.5608354E38F);
                Debug.Assert(pack.y == (float)1.928467E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3729426001U);
                Debug.Assert(pack.z == (float)2.0774297E38F);
                Debug.Assert(pack.x == (float) -2.7646407E38F);
                Debug.Assert(pack.pitch == (float) -3.078991E38F);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.yaw = (float) -1.5584982E38F;
            p89.time_boot_ms = (uint)3729426001U;
            p89.x = (float) -2.7646407E38F;
            p89.z = (float)2.0774297E38F;
            p89.roll = (float) -1.5608354E38F;
            p89.pitch = (float) -3.078991E38F;
            p89.y = (float)1.928467E38F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short) -28014);
                Debug.Assert(pack.alt == (int)349394254);
                Debug.Assert(pack.yawspeed == (float) -3.3832002E38F);
                Debug.Assert(pack.zacc == (short)(short)18487);
                Debug.Assert(pack.rollspeed == (float)3.1232633E38F);
                Debug.Assert(pack.yaw == (float)4.42888E37F);
                Debug.Assert(pack.vy == (short)(short)13453);
                Debug.Assert(pack.yacc == (short)(short)21674);
                Debug.Assert(pack.roll == (float) -1.684752E38F);
                Debug.Assert(pack.lat == (int)1534014252);
                Debug.Assert(pack.vx == (short)(short)463);
                Debug.Assert(pack.pitchspeed == (float)1.3312696E38F);
                Debug.Assert(pack.vz == (short)(short) -8240);
                Debug.Assert(pack.lon == (int) -1477953252);
                Debug.Assert(pack.pitch == (float)6.101612E37F);
                Debug.Assert(pack.time_usec == (ulong)4070491487378117754L);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.xacc = (short)(short) -28014;
            p90.yacc = (short)(short)21674;
            p90.vz = (short)(short) -8240;
            p90.rollspeed = (float)3.1232633E38F;
            p90.vy = (short)(short)13453;
            p90.yawspeed = (float) -3.3832002E38F;
            p90.vx = (short)(short)463;
            p90.lat = (int)1534014252;
            p90.time_usec = (ulong)4070491487378117754L;
            p90.yaw = (float)4.42888E37F;
            p90.roll = (float) -1.684752E38F;
            p90.pitch = (float)6.101612E37F;
            p90.lon = (int) -1477953252;
            p90.zacc = (short)(short)18487;
            p90.alt = (int)349394254;
            p90.pitchspeed = (float)1.3312696E38F;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nav_mode == (byte)(byte)39);
                Debug.Assert(pack.pitch_elevator == (float)7.821627E37F);
                Debug.Assert(pack.throttle == (float)8.3592176E37F);
                Debug.Assert(pack.time_usec == (ulong)2075586044811303396L);
                Debug.Assert(pack.aux2 == (float) -2.1037331E38F);
                Debug.Assert(pack.yaw_rudder == (float) -1.5712644E38F);
                Debug.Assert(pack.aux4 == (float) -1.6161817E38F);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_TEST_DISARMED);
                Debug.Assert(pack.aux1 == (float) -1.5946959E38F);
                Debug.Assert(pack.roll_ailerons == (float) -1.5688435E38F);
                Debug.Assert(pack.aux3 == (float)1.0879543E37F);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.mode = MAV_MODE.MAV_MODE_TEST_DISARMED;
            p91.aux2 = (float) -2.1037331E38F;
            p91.aux1 = (float) -1.5946959E38F;
            p91.aux4 = (float) -1.6161817E38F;
            p91.nav_mode = (byte)(byte)39;
            p91.throttle = (float)8.3592176E37F;
            p91.aux3 = (float)1.0879543E37F;
            p91.yaw_rudder = (float) -1.5712644E38F;
            p91.roll_ailerons = (float) -1.5688435E38F;
            p91.pitch_elevator = (float)7.821627E37F;
            p91.time_usec = (ulong)2075586044811303396L;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)22094);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)25213);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)18010);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)2505);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)14880);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)51265);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)47701);
                Debug.Assert(pack.rssi == (byte)(byte)166);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)6985);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)12258);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)21285);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)54795);
                Debug.Assert(pack.time_usec == (ulong)2221795013751470978L);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)31008);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan8_raw = (ushort)(ushort)18010;
            p92.chan10_raw = (ushort)(ushort)2505;
            p92.chan12_raw = (ushort)(ushort)6985;
            p92.chan2_raw = (ushort)(ushort)31008;
            p92.chan7_raw = (ushort)(ushort)47701;
            p92.chan1_raw = (ushort)(ushort)25213;
            p92.time_usec = (ulong)2221795013751470978L;
            p92.chan5_raw = (ushort)(ushort)14880;
            p92.chan9_raw = (ushort)(ushort)51265;
            p92.chan4_raw = (ushort)(ushort)54795;
            p92.chan3_raw = (ushort)(ushort)21285;
            p92.chan6_raw = (ushort)(ushort)22094;
            p92.chan11_raw = (ushort)(ushort)12258;
            p92.rssi = (byte)(byte)166;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-3.3927689E38F, 3.002278E38F, 8.8523546E36F, 3.1055736E38F, -2.7945463E38F, 8.832522E37F, 1.8556032E38F, -1.7962053E38F, 2.3672115E38F, 1.4878241E38F, 3.0699998E37F, 2.8083848E38F, 2.5265587E38F, 3.2349813E38F, 2.8286617E38F, 2.561817E38F}));
                Debug.Assert(pack.time_usec == (ulong)5578097062127376303L);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_TEST_ARMED);
                Debug.Assert(pack.flags == (ulong)4517566718265640992L);
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.flags = (ulong)4517566718265640992L;
            p93.controls_SET(new float[] {-3.3927689E38F, 3.002278E38F, 8.8523546E36F, 3.1055736E38F, -2.7945463E38F, 8.832522E37F, 1.8556032E38F, -1.7962053E38F, 2.3672115E38F, 1.4878241E38F, 3.0699998E37F, 2.8083848E38F, 2.5265587E38F, 3.2349813E38F, 2.8286617E38F, 2.561817E38F}, 0) ;
            p93.time_usec = (ulong)5578097062127376303L;
            p93.mode = MAV_MODE.MAV_MODE_TEST_ARMED;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_id == (byte)(byte)176);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)1.4624361E38F);
                Debug.Assert(pack.quality == (byte)(byte)44);
                Debug.Assert(pack.time_usec == (ulong)1918151182279808267L);
                Debug.Assert(pack.ground_distance == (float)5.943045E37F);
                Debug.Assert(pack.flow_comp_m_y == (float)2.5763532E38F);
                Debug.Assert(pack.flow_y == (short)(short)17546);
                Debug.Assert(pack.flow_comp_m_x == (float)2.0855595E38F);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float) -2.4037485E38F);
                Debug.Assert(pack.flow_x == (short)(short) -19807);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_rate_y_SET((float) -2.4037485E38F, PH) ;
            p100.flow_rate_x_SET((float)1.4624361E38F, PH) ;
            p100.flow_y = (short)(short)17546;
            p100.flow_x = (short)(short) -19807;
            p100.flow_comp_m_x = (float)2.0855595E38F;
            p100.ground_distance = (float)5.943045E37F;
            p100.flow_comp_m_y = (float)2.5763532E38F;
            p100.quality = (byte)(byte)44;
            p100.time_usec = (ulong)1918151182279808267L;
            p100.sensor_id = (byte)(byte)176;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)1.4443548E38F);
                Debug.Assert(pack.yaw == (float)1.8530713E37F);
                Debug.Assert(pack.z == (float) -3.2145546E38F);
                Debug.Assert(pack.pitch == (float)2.4483716E38F);
                Debug.Assert(pack.y == (float) -2.6201093E38F);
                Debug.Assert(pack.usec == (ulong)7967810172016748611L);
                Debug.Assert(pack.roll == (float) -2.0842125E38F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.y = (float) -2.6201093E38F;
            p101.yaw = (float)1.8530713E37F;
            p101.z = (float) -3.2145546E38F;
            p101.x = (float)1.4443548E38F;
            p101.pitch = (float)2.4483716E38F;
            p101.roll = (float) -2.0842125E38F;
            p101.usec = (ulong)7967810172016748611L;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -2.6687805E37F);
                Debug.Assert(pack.roll == (float)3.2080401E38F);
                Debug.Assert(pack.x == (float) -1.654253E38F);
                Debug.Assert(pack.y == (float)2.7849115E38F);
                Debug.Assert(pack.usec == (ulong)7262667685635876894L);
                Debug.Assert(pack.pitch == (float)2.3322035E37F);
                Debug.Assert(pack.yaw == (float) -6.407374E37F);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.pitch = (float)2.3322035E37F;
            p102.yaw = (float) -6.407374E37F;
            p102.y = (float)2.7849115E38F;
            p102.z = (float) -2.6687805E37F;
            p102.x = (float) -1.654253E38F;
            p102.roll = (float)3.2080401E38F;
            p102.usec = (ulong)7262667685635876894L;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)3088139952008383972L);
                Debug.Assert(pack.y == (float)5.7468017E37F);
                Debug.Assert(pack.z == (float) -1.5465797E38F);
                Debug.Assert(pack.x == (float)2.4861957E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.y = (float)5.7468017E37F;
            p103.usec = (ulong)3088139952008383972L;
            p103.x = (float)2.4861957E38F;
            p103.z = (float) -1.5465797E38F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -1.0807572E38F);
                Debug.Assert(pack.x == (float)1.943164E38F);
                Debug.Assert(pack.z == (float) -2.566072E38F);
                Debug.Assert(pack.usec == (ulong)5098127217013352104L);
                Debug.Assert(pack.roll == (float)1.2088665E38F);
                Debug.Assert(pack.pitch == (float) -9.857976E37F);
                Debug.Assert(pack.yaw == (float) -2.9949258E38F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.pitch = (float) -9.857976E37F;
            p104.x = (float)1.943164E38F;
            p104.roll = (float)1.2088665E38F;
            p104.usec = (ulong)5098127217013352104L;
            p104.y = (float) -1.0807572E38F;
            p104.yaw = (float) -2.9949258E38F;
            p104.z = (float) -2.566072E38F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zmag == (float) -2.5235086E38F);
                Debug.Assert(pack.temperature == (float) -2.3797187E38F);
                Debug.Assert(pack.ymag == (float)3.7019918E37F);
                Debug.Assert(pack.diff_pressure == (float)1.5473523E38F);
                Debug.Assert(pack.ygyro == (float)7.1357426E37F);
                Debug.Assert(pack.xgyro == (float)1.5734197E38F);
                Debug.Assert(pack.xacc == (float)1.8058955E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)33968);
                Debug.Assert(pack.abs_pressure == (float)1.9945628E38F);
                Debug.Assert(pack.xmag == (float) -2.6523579E38F);
                Debug.Assert(pack.zgyro == (float)1.6919431E38F);
                Debug.Assert(pack.time_usec == (ulong)5612826372613717689L);
                Debug.Assert(pack.yacc == (float)3.2333733E38F);
                Debug.Assert(pack.pressure_alt == (float)1.4004448E38F);
                Debug.Assert(pack.zacc == (float)3.6880677E37F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.diff_pressure = (float)1.5473523E38F;
            p105.xgyro = (float)1.5734197E38F;
            p105.fields_updated = (ushort)(ushort)33968;
            p105.ygyro = (float)7.1357426E37F;
            p105.xacc = (float)1.8058955E38F;
            p105.abs_pressure = (float)1.9945628E38F;
            p105.ymag = (float)3.7019918E37F;
            p105.xmag = (float) -2.6523579E38F;
            p105.pressure_alt = (float)1.4004448E38F;
            p105.zgyro = (float)1.6919431E38F;
            p105.zmag = (float) -2.5235086E38F;
            p105.temperature = (float) -2.3797187E38F;
            p105.yacc = (float)3.2333733E38F;
            p105.time_usec = (ulong)5612826372613717689L;
            p105.zacc = (float)3.6880677E37F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_delta_distance_us == (uint)2837189430U);
                Debug.Assert(pack.integrated_y == (float)2.6347424E38F);
                Debug.Assert(pack.integrated_ygyro == (float) -3.8288758E37F);
                Debug.Assert(pack.integrated_x == (float) -1.8094775E38F);
                Debug.Assert(pack.integrated_xgyro == (float)8.4357356E37F);
                Debug.Assert(pack.integration_time_us == (uint)2554888618U);
                Debug.Assert(pack.distance == (float) -6.8266164E37F);
                Debug.Assert(pack.temperature == (short)(short)27596);
                Debug.Assert(pack.integrated_zgyro == (float) -1.3014569E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)82);
                Debug.Assert(pack.time_usec == (ulong)8286191542172961621L);
                Debug.Assert(pack.quality == (byte)(byte)178);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.integrated_ygyro = (float) -3.8288758E37F;
            p106.time_usec = (ulong)8286191542172961621L;
            p106.temperature = (short)(short)27596;
            p106.distance = (float) -6.8266164E37F;
            p106.integrated_zgyro = (float) -1.3014569E38F;
            p106.time_delta_distance_us = (uint)2837189430U;
            p106.integrated_xgyro = (float)8.4357356E37F;
            p106.integration_time_us = (uint)2554888618U;
            p106.integrated_x = (float) -1.8094775E38F;
            p106.quality = (byte)(byte)178;
            p106.sensor_id = (byte)(byte)82;
            p106.integrated_y = (float)2.6347424E38F;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (float)8.147469E37F);
                Debug.Assert(pack.diff_pressure == (float) -8.559022E37F);
                Debug.Assert(pack.abs_pressure == (float)1.6013377E38F);
                Debug.Assert(pack.zacc == (float) -1.5704636E38F);
                Debug.Assert(pack.fields_updated == (uint)2436476580U);
                Debug.Assert(pack.zmag == (float)3.1404374E38F);
                Debug.Assert(pack.ymag == (float)3.249845E38F);
                Debug.Assert(pack.pressure_alt == (float) -3.0072402E37F);
                Debug.Assert(pack.time_usec == (ulong)8227643070009062152L);
                Debug.Assert(pack.zgyro == (float)2.8903638E38F);
                Debug.Assert(pack.xgyro == (float) -3.104818E38F);
                Debug.Assert(pack.xacc == (float)3.973328E37F);
                Debug.Assert(pack.xmag == (float) -2.0294849E38F);
                Debug.Assert(pack.ygyro == (float) -2.0839154E38F);
                Debug.Assert(pack.yacc == (float) -1.9411801E38F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.ygyro = (float) -2.0839154E38F;
            p107.time_usec = (ulong)8227643070009062152L;
            p107.zgyro = (float)2.8903638E38F;
            p107.temperature = (float)8.147469E37F;
            p107.zacc = (float) -1.5704636E38F;
            p107.ymag = (float)3.249845E38F;
            p107.fields_updated = (uint)2436476580U;
            p107.pressure_alt = (float) -3.0072402E37F;
            p107.zmag = (float)3.1404374E38F;
            p107.yacc = (float) -1.9411801E38F;
            p107.diff_pressure = (float) -8.559022E37F;
            p107.abs_pressure = (float)1.6013377E38F;
            p107.xgyro = (float) -3.104818E38F;
            p107.xmag = (float) -2.0294849E38F;
            p107.xacc = (float)3.973328E37F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q2 == (float)2.5835013E38F);
                Debug.Assert(pack.q3 == (float)1.2082958E38F);
                Debug.Assert(pack.std_dev_horz == (float)2.890825E38F);
                Debug.Assert(pack.pitch == (float)1.8988051E38F);
                Debug.Assert(pack.yacc == (float) -2.304093E38F);
                Debug.Assert(pack.vd == (float) -5.130378E37F);
                Debug.Assert(pack.vn == (float) -2.3233772E38F);
                Debug.Assert(pack.alt == (float) -3.1282017E38F);
                Debug.Assert(pack.yaw == (float) -2.6863423E38F);
                Debug.Assert(pack.zacc == (float) -1.1689047E38F);
                Debug.Assert(pack.xgyro == (float) -1.5742109E38F);
                Debug.Assert(pack.std_dev_vert == (float) -1.2055883E38F);
                Debug.Assert(pack.q1 == (float) -2.235849E38F);
                Debug.Assert(pack.ygyro == (float) -7.0035155E37F);
                Debug.Assert(pack.lat == (float) -1.7789866E38F);
                Debug.Assert(pack.q4 == (float) -6.0715413E37F);
                Debug.Assert(pack.ve == (float) -2.9533515E38F);
                Debug.Assert(pack.roll == (float)3.3595941E38F);
                Debug.Assert(pack.lon == (float) -1.6989454E38F);
                Debug.Assert(pack.xacc == (float) -3.088047E38F);
                Debug.Assert(pack.zgyro == (float)8.413934E37F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.alt = (float) -3.1282017E38F;
            p108.lat = (float) -1.7789866E38F;
            p108.q3 = (float)1.2082958E38F;
            p108.yaw = (float) -2.6863423E38F;
            p108.pitch = (float)1.8988051E38F;
            p108.vn = (float) -2.3233772E38F;
            p108.std_dev_vert = (float) -1.2055883E38F;
            p108.yacc = (float) -2.304093E38F;
            p108.xacc = (float) -3.088047E38F;
            p108.lon = (float) -1.6989454E38F;
            p108.q2 = (float)2.5835013E38F;
            p108.q1 = (float) -2.235849E38F;
            p108.zacc = (float) -1.1689047E38F;
            p108.ygyro = (float) -7.0035155E37F;
            p108.roll = (float)3.3595941E38F;
            p108.std_dev_horz = (float)2.890825E38F;
            p108.zgyro = (float)8.413934E37F;
            p108.q4 = (float) -6.0715413E37F;
            p108.ve = (float) -2.9533515E38F;
            p108.xgyro = (float) -1.5742109E38F;
            p108.vd = (float) -5.130378E37F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.txbuf == (byte)(byte)142);
                Debug.Assert(pack.rssi == (byte)(byte)136);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)35884);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)57863);
                Debug.Assert(pack.noise == (byte)(byte)236);
                Debug.Assert(pack.remrssi == (byte)(byte)74);
                Debug.Assert(pack.remnoise == (byte)(byte)138);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.txbuf = (byte)(byte)142;
            p109.noise = (byte)(byte)236;
            p109.remnoise = (byte)(byte)138;
            p109.rssi = (byte)(byte)136;
            p109.remrssi = (byte)(byte)74;
            p109.fixed_ = (ushort)(ushort)35884;
            p109.rxerrors = (ushort)(ushort)57863;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)97);
                Debug.Assert(pack.target_network == (byte)(byte)92);
                Debug.Assert(pack.target_component == (byte)(byte)140);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)226, (byte)166, (byte)225, (byte)103, (byte)178, (byte)84, (byte)78, (byte)200, (byte)105, (byte)48, (byte)78, (byte)57, (byte)90, (byte)249, (byte)24, (byte)201, (byte)171, (byte)242, (byte)8, (byte)8, (byte)190, (byte)1, (byte)148, (byte)136, (byte)115, (byte)174, (byte)130, (byte)58, (byte)89, (byte)90, (byte)40, (byte)179, (byte)254, (byte)79, (byte)7, (byte)232, (byte)143, (byte)92, (byte)107, (byte)199, (byte)85, (byte)237, (byte)252, (byte)101, (byte)151, (byte)150, (byte)239, (byte)116, (byte)104, (byte)222, (byte)97, (byte)41, (byte)38, (byte)10, (byte)233, (byte)213, (byte)164, (byte)49, (byte)19, (byte)10, (byte)225, (byte)215, (byte)191, (byte)212, (byte)206, (byte)100, (byte)216, (byte)40, (byte)22, (byte)80, (byte)78, (byte)168, (byte)37, (byte)81, (byte)184, (byte)251, (byte)135, (byte)115, (byte)26, (byte)194, (byte)231, (byte)215, (byte)141, (byte)72, (byte)151, (byte)37, (byte)230, (byte)251, (byte)185, (byte)143, (byte)61, (byte)56, (byte)214, (byte)7, (byte)230, (byte)173, (byte)83, (byte)135, (byte)44, (byte)20, (byte)80, (byte)89, (byte)12, (byte)35, (byte)164, (byte)41, (byte)81, (byte)137, (byte)241, (byte)121, (byte)140, (byte)18, (byte)47, (byte)98, (byte)224, (byte)223, (byte)18, (byte)109, (byte)113, (byte)149, (byte)77, (byte)222, (byte)234, (byte)240, (byte)153, (byte)88, (byte)115, (byte)224, (byte)13, (byte)253, (byte)69, (byte)69, (byte)196, (byte)35, (byte)240, (byte)78, (byte)107, (byte)243, (byte)26, (byte)122, (byte)167, (byte)241, (byte)183, (byte)116, (byte)254, (byte)184, (byte)67, (byte)240, (byte)224, (byte)68, (byte)9, (byte)63, (byte)43, (byte)222, (byte)133, (byte)212, (byte)3, (byte)135, (byte)161, (byte)19, (byte)250, (byte)31, (byte)44, (byte)137, (byte)46, (byte)102, (byte)166, (byte)249, (byte)81, (byte)61, (byte)44, (byte)204, (byte)147, (byte)113, (byte)145, (byte)229, (byte)184, (byte)174, (byte)118, (byte)140, (byte)238, (byte)156, (byte)63, (byte)234, (byte)210, (byte)70, (byte)80, (byte)176, (byte)240, (byte)57, (byte)225, (byte)5, (byte)29, (byte)12, (byte)240, (byte)128, (byte)202, (byte)10, (byte)39, (byte)252, (byte)231, (byte)200, (byte)47, (byte)59, (byte)56, (byte)29, (byte)168, (byte)23, (byte)111, (byte)96, (byte)111, (byte)144, (byte)218, (byte)33, (byte)82, (byte)230, (byte)133, (byte)36, (byte)157, (byte)172, (byte)74, (byte)57, (byte)152, (byte)219, (byte)252, (byte)74, (byte)145, (byte)112, (byte)175, (byte)93, (byte)225, (byte)145, (byte)232, (byte)189, (byte)232, (byte)64, (byte)54, (byte)2, (byte)57, (byte)74, (byte)195, (byte)106, (byte)153, (byte)170, (byte)134, (byte)194, (byte)174, (byte)144, (byte)253, (byte)225, (byte)163}));
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_component = (byte)(byte)140;
            p110.target_network = (byte)(byte)92;
            p110.payload_SET(new byte[] {(byte)226, (byte)166, (byte)225, (byte)103, (byte)178, (byte)84, (byte)78, (byte)200, (byte)105, (byte)48, (byte)78, (byte)57, (byte)90, (byte)249, (byte)24, (byte)201, (byte)171, (byte)242, (byte)8, (byte)8, (byte)190, (byte)1, (byte)148, (byte)136, (byte)115, (byte)174, (byte)130, (byte)58, (byte)89, (byte)90, (byte)40, (byte)179, (byte)254, (byte)79, (byte)7, (byte)232, (byte)143, (byte)92, (byte)107, (byte)199, (byte)85, (byte)237, (byte)252, (byte)101, (byte)151, (byte)150, (byte)239, (byte)116, (byte)104, (byte)222, (byte)97, (byte)41, (byte)38, (byte)10, (byte)233, (byte)213, (byte)164, (byte)49, (byte)19, (byte)10, (byte)225, (byte)215, (byte)191, (byte)212, (byte)206, (byte)100, (byte)216, (byte)40, (byte)22, (byte)80, (byte)78, (byte)168, (byte)37, (byte)81, (byte)184, (byte)251, (byte)135, (byte)115, (byte)26, (byte)194, (byte)231, (byte)215, (byte)141, (byte)72, (byte)151, (byte)37, (byte)230, (byte)251, (byte)185, (byte)143, (byte)61, (byte)56, (byte)214, (byte)7, (byte)230, (byte)173, (byte)83, (byte)135, (byte)44, (byte)20, (byte)80, (byte)89, (byte)12, (byte)35, (byte)164, (byte)41, (byte)81, (byte)137, (byte)241, (byte)121, (byte)140, (byte)18, (byte)47, (byte)98, (byte)224, (byte)223, (byte)18, (byte)109, (byte)113, (byte)149, (byte)77, (byte)222, (byte)234, (byte)240, (byte)153, (byte)88, (byte)115, (byte)224, (byte)13, (byte)253, (byte)69, (byte)69, (byte)196, (byte)35, (byte)240, (byte)78, (byte)107, (byte)243, (byte)26, (byte)122, (byte)167, (byte)241, (byte)183, (byte)116, (byte)254, (byte)184, (byte)67, (byte)240, (byte)224, (byte)68, (byte)9, (byte)63, (byte)43, (byte)222, (byte)133, (byte)212, (byte)3, (byte)135, (byte)161, (byte)19, (byte)250, (byte)31, (byte)44, (byte)137, (byte)46, (byte)102, (byte)166, (byte)249, (byte)81, (byte)61, (byte)44, (byte)204, (byte)147, (byte)113, (byte)145, (byte)229, (byte)184, (byte)174, (byte)118, (byte)140, (byte)238, (byte)156, (byte)63, (byte)234, (byte)210, (byte)70, (byte)80, (byte)176, (byte)240, (byte)57, (byte)225, (byte)5, (byte)29, (byte)12, (byte)240, (byte)128, (byte)202, (byte)10, (byte)39, (byte)252, (byte)231, (byte)200, (byte)47, (byte)59, (byte)56, (byte)29, (byte)168, (byte)23, (byte)111, (byte)96, (byte)111, (byte)144, (byte)218, (byte)33, (byte)82, (byte)230, (byte)133, (byte)36, (byte)157, (byte)172, (byte)74, (byte)57, (byte)152, (byte)219, (byte)252, (byte)74, (byte)145, (byte)112, (byte)175, (byte)93, (byte)225, (byte)145, (byte)232, (byte)189, (byte)232, (byte)64, (byte)54, (byte)2, (byte)57, (byte)74, (byte)195, (byte)106, (byte)153, (byte)170, (byte)134, (byte)194, (byte)174, (byte)144, (byte)253, (byte)225, (byte)163}, 0) ;
            p110.target_system = (byte)(byte)97;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tc1 == (long) -7496803390271229274L);
                Debug.Assert(pack.ts1 == (long) -314357297360836862L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.ts1 = (long) -314357297360836862L;
            p111.tc1 = (long) -7496803390271229274L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)3043658036U);
                Debug.Assert(pack.time_usec == (ulong)55476996027169734L);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.seq = (uint)3043658036U;
            p112.time_usec = (ulong)55476996027169734L;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellites_visible == (byte)(byte)87);
                Debug.Assert(pack.vn == (short)(short) -25198);
                Debug.Assert(pack.fix_type == (byte)(byte)251);
                Debug.Assert(pack.cog == (ushort)(ushort)25704);
                Debug.Assert(pack.lat == (int)740217320);
                Debug.Assert(pack.ve == (short)(short)28909);
                Debug.Assert(pack.alt == (int)1902565430);
                Debug.Assert(pack.lon == (int) -1542792131);
                Debug.Assert(pack.vd == (short)(short) -2169);
                Debug.Assert(pack.vel == (ushort)(ushort)63336);
                Debug.Assert(pack.time_usec == (ulong)2296997455162266544L);
                Debug.Assert(pack.epv == (ushort)(ushort)45757);
                Debug.Assert(pack.eph == (ushort)(ushort)17537);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.satellites_visible = (byte)(byte)87;
            p113.time_usec = (ulong)2296997455162266544L;
            p113.fix_type = (byte)(byte)251;
            p113.vd = (short)(short) -2169;
            p113.lat = (int)740217320;
            p113.epv = (ushort)(ushort)45757;
            p113.vn = (short)(short) -25198;
            p113.vel = (ushort)(ushort)63336;
            p113.alt = (int)1902565430;
            p113.ve = (short)(short)28909;
            p113.eph = (ushort)(ushort)17537;
            p113.cog = (ushort)(ushort)25704;
            p113.lon = (int) -1542792131;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integration_time_us == (uint)3890240612U);
                Debug.Assert(pack.integrated_x == (float) -2.6596512E38F);
                Debug.Assert(pack.quality == (byte)(byte)243);
                Debug.Assert(pack.temperature == (short)(short)9804);
                Debug.Assert(pack.integrated_zgyro == (float)1.4698473E38F);
                Debug.Assert(pack.integrated_ygyro == (float) -2.9793797E38F);
                Debug.Assert(pack.integrated_y == (float) -5.7100556E37F);
                Debug.Assert(pack.integrated_xgyro == (float) -1.9253093E36F);
                Debug.Assert(pack.sensor_id == (byte)(byte)178);
                Debug.Assert(pack.time_usec == (ulong)6292108980198498987L);
                Debug.Assert(pack.time_delta_distance_us == (uint)325318731U);
                Debug.Assert(pack.distance == (float)2.445294E38F);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.integrated_xgyro = (float) -1.9253093E36F;
            p114.integrated_zgyro = (float)1.4698473E38F;
            p114.temperature = (short)(short)9804;
            p114.time_delta_distance_us = (uint)325318731U;
            p114.integrated_ygyro = (float) -2.9793797E38F;
            p114.sensor_id = (byte)(byte)178;
            p114.quality = (byte)(byte)243;
            p114.integrated_y = (float) -5.7100556E37F;
            p114.time_usec = (ulong)6292108980198498987L;
            p114.integration_time_us = (uint)3890240612U;
            p114.distance = (float)2.445294E38F;
            p114.integrated_x = (float) -2.6596512E38F;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (int) -1129142696);
                Debug.Assert(pack.yacc == (short)(short)807);
                Debug.Assert(pack.zacc == (short)(short) -25995);
                Debug.Assert(pack.rollspeed == (float) -1.3226852E38F);
                Debug.Assert(pack.vx == (short)(short)24106);
                Debug.Assert(pack.xacc == (short)(short) -25847);
                Debug.Assert(pack.lon == (int) -202573149);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {2.7320006E38F, -2.4156624E38F, -1.8804118E38F, 1.3708316E38F}));
                Debug.Assert(pack.yawspeed == (float) -2.2337099E38F);
                Debug.Assert(pack.time_usec == (ulong)7203576477881995973L);
                Debug.Assert(pack.lat == (int)1944110178);
                Debug.Assert(pack.pitchspeed == (float)3.2831714E38F);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)63694);
                Debug.Assert(pack.vz == (short)(short)28993);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)58140);
                Debug.Assert(pack.vy == (short)(short)6387);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.vz = (short)(short)28993;
            p115.time_usec = (ulong)7203576477881995973L;
            p115.lon = (int) -202573149;
            p115.rollspeed = (float) -1.3226852E38F;
            p115.zacc = (short)(short) -25995;
            p115.vx = (short)(short)24106;
            p115.vy = (short)(short)6387;
            p115.ind_airspeed = (ushort)(ushort)63694;
            p115.xacc = (short)(short) -25847;
            p115.yacc = (short)(short)807;
            p115.true_airspeed = (ushort)(ushort)58140;
            p115.attitude_quaternion_SET(new float[] {2.7320006E38F, -2.4156624E38F, -1.8804118E38F, 1.3708316E38F}, 0) ;
            p115.yawspeed = (float) -2.2337099E38F;
            p115.alt = (int) -1129142696;
            p115.lat = (int)1944110178;
            p115.pitchspeed = (float)3.2831714E38F;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (short)(short) -24971);
                Debug.Assert(pack.zmag == (short)(short) -30239);
                Debug.Assert(pack.zacc == (short)(short) -2130);
                Debug.Assert(pack.ymag == (short)(short) -25633);
                Debug.Assert(pack.zgyro == (short)(short)18850);
                Debug.Assert(pack.time_boot_ms == (uint)2677037099U);
                Debug.Assert(pack.yacc == (short)(short) -4355);
                Debug.Assert(pack.xacc == (short)(short)32039);
                Debug.Assert(pack.xmag == (short)(short) -16994);
                Debug.Assert(pack.xgyro == (short)(short) -6828);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.ygyro = (short)(short) -24971;
            p116.ymag = (short)(short) -25633;
            p116.zgyro = (short)(short)18850;
            p116.yacc = (short)(short) -4355;
            p116.xmag = (short)(short) -16994;
            p116.xgyro = (short)(short) -6828;
            p116.zmag = (short)(short) -30239;
            p116.time_boot_ms = (uint)2677037099U;
            p116.xacc = (short)(short)32039;
            p116.zacc = (short)(short) -2130;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)65);
                Debug.Assert(pack.end == (ushort)(ushort)4190);
                Debug.Assert(pack.start == (ushort)(ushort)34400);
                Debug.Assert(pack.target_component == (byte)(byte)25);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.end = (ushort)(ushort)4190;
            p117.target_component = (byte)(byte)25;
            p117.target_system = (byte)(byte)65;
            p117.start = (ushort)(ushort)34400;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.size == (uint)4120753501U);
                Debug.Assert(pack.time_utc == (uint)4084871532U);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)35794);
                Debug.Assert(pack.id == (ushort)(ushort)45287);
                Debug.Assert(pack.num_logs == (ushort)(ushort)50792);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)45287;
            p118.time_utc = (uint)4084871532U;
            p118.num_logs = (ushort)(ushort)50792;
            p118.size = (uint)4120753501U;
            p118.last_log_num = (ushort)(ushort)35794;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)58102);
                Debug.Assert(pack.target_system == (byte)(byte)31);
                Debug.Assert(pack.target_component == (byte)(byte)152);
                Debug.Assert(pack.ofs == (uint)3262853060U);
                Debug.Assert(pack.count == (uint)2862803520U);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_component = (byte)(byte)152;
            p119.ofs = (uint)3262853060U;
            p119.target_system = (byte)(byte)31;
            p119.count = (uint)2862803520U;
            p119.id = (ushort)(ushort)58102;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)9879);
                Debug.Assert(pack.count == (byte)(byte)230);
                Debug.Assert(pack.ofs == (uint)1626983888U);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)207, (byte)183, (byte)196, (byte)18, (byte)210, (byte)246, (byte)234, (byte)244, (byte)106, (byte)66, (byte)83, (byte)94, (byte)185, (byte)153, (byte)185, (byte)147, (byte)1, (byte)222, (byte)1, (byte)78, (byte)170, (byte)67, (byte)173, (byte)138, (byte)178, (byte)171, (byte)218, (byte)100, (byte)26, (byte)194, (byte)87, (byte)156, (byte)9, (byte)151, (byte)24, (byte)197, (byte)124, (byte)248, (byte)239, (byte)92, (byte)72, (byte)114, (byte)246, (byte)225, (byte)159, (byte)197, (byte)64, (byte)115, (byte)146, (byte)150, (byte)164, (byte)95, (byte)100, (byte)153, (byte)234, (byte)125, (byte)130, (byte)100, (byte)26, (byte)12, (byte)222, (byte)131, (byte)79, (byte)68, (byte)255, (byte)77, (byte)156, (byte)39, (byte)244, (byte)86, (byte)38, (byte)108, (byte)158, (byte)87, (byte)117, (byte)71, (byte)159, (byte)119, (byte)182, (byte)218, (byte)105, (byte)162, (byte)30, (byte)133, (byte)13, (byte)87, (byte)229, (byte)172, (byte)4, (byte)182}));
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.data__SET(new byte[] {(byte)207, (byte)183, (byte)196, (byte)18, (byte)210, (byte)246, (byte)234, (byte)244, (byte)106, (byte)66, (byte)83, (byte)94, (byte)185, (byte)153, (byte)185, (byte)147, (byte)1, (byte)222, (byte)1, (byte)78, (byte)170, (byte)67, (byte)173, (byte)138, (byte)178, (byte)171, (byte)218, (byte)100, (byte)26, (byte)194, (byte)87, (byte)156, (byte)9, (byte)151, (byte)24, (byte)197, (byte)124, (byte)248, (byte)239, (byte)92, (byte)72, (byte)114, (byte)246, (byte)225, (byte)159, (byte)197, (byte)64, (byte)115, (byte)146, (byte)150, (byte)164, (byte)95, (byte)100, (byte)153, (byte)234, (byte)125, (byte)130, (byte)100, (byte)26, (byte)12, (byte)222, (byte)131, (byte)79, (byte)68, (byte)255, (byte)77, (byte)156, (byte)39, (byte)244, (byte)86, (byte)38, (byte)108, (byte)158, (byte)87, (byte)117, (byte)71, (byte)159, (byte)119, (byte)182, (byte)218, (byte)105, (byte)162, (byte)30, (byte)133, (byte)13, (byte)87, (byte)229, (byte)172, (byte)4, (byte)182}, 0) ;
            p120.count = (byte)(byte)230;
            p120.ofs = (uint)1626983888U;
            p120.id = (ushort)(ushort)9879;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)80);
                Debug.Assert(pack.target_component == (byte)(byte)59);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)80;
            p121.target_component = (byte)(byte)59;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)79);
                Debug.Assert(pack.target_component == (byte)(byte)181);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)79;
            p122.target_component = (byte)(byte)181;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)108);
                Debug.Assert(pack.len == (byte)(byte)100);
                Debug.Assert(pack.target_component == (byte)(byte)204);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)220, (byte)99, (byte)143, (byte)122, (byte)215, (byte)110, (byte)26, (byte)152, (byte)179, (byte)31, (byte)239, (byte)37, (byte)131, (byte)250, (byte)15, (byte)96, (byte)137, (byte)251, (byte)133, (byte)134, (byte)207, (byte)22, (byte)3, (byte)115, (byte)63, (byte)121, (byte)245, (byte)11, (byte)33, (byte)245, (byte)61, (byte)233, (byte)252, (byte)48, (byte)218, (byte)123, (byte)15, (byte)47, (byte)20, (byte)65, (byte)177, (byte)171, (byte)141, (byte)11, (byte)28, (byte)177, (byte)85, (byte)83, (byte)156, (byte)197, (byte)146, (byte)203, (byte)34, (byte)231, (byte)230, (byte)152, (byte)11, (byte)97, (byte)55, (byte)4, (byte)11, (byte)199, (byte)153, (byte)155, (byte)223, (byte)235, (byte)30, (byte)178, (byte)77, (byte)69, (byte)149, (byte)95, (byte)182, (byte)225, (byte)132, (byte)21, (byte)206, (byte)253, (byte)194, (byte)151, (byte)34, (byte)68, (byte)95, (byte)113, (byte)29, (byte)62, (byte)96, (byte)210, (byte)49, (byte)17, (byte)108, (byte)221, (byte)91, (byte)127, (byte)164, (byte)205, (byte)184, (byte)202, (byte)228, (byte)44, (byte)101, (byte)222, (byte)138, (byte)232, (byte)156, (byte)2, (byte)68, (byte)70, (byte)185, (byte)27}));
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)108;
            p123.target_component = (byte)(byte)204;
            p123.len = (byte)(byte)100;
            p123.data__SET(new byte[] {(byte)220, (byte)99, (byte)143, (byte)122, (byte)215, (byte)110, (byte)26, (byte)152, (byte)179, (byte)31, (byte)239, (byte)37, (byte)131, (byte)250, (byte)15, (byte)96, (byte)137, (byte)251, (byte)133, (byte)134, (byte)207, (byte)22, (byte)3, (byte)115, (byte)63, (byte)121, (byte)245, (byte)11, (byte)33, (byte)245, (byte)61, (byte)233, (byte)252, (byte)48, (byte)218, (byte)123, (byte)15, (byte)47, (byte)20, (byte)65, (byte)177, (byte)171, (byte)141, (byte)11, (byte)28, (byte)177, (byte)85, (byte)83, (byte)156, (byte)197, (byte)146, (byte)203, (byte)34, (byte)231, (byte)230, (byte)152, (byte)11, (byte)97, (byte)55, (byte)4, (byte)11, (byte)199, (byte)153, (byte)155, (byte)223, (byte)235, (byte)30, (byte)178, (byte)77, (byte)69, (byte)149, (byte)95, (byte)182, (byte)225, (byte)132, (byte)21, (byte)206, (byte)253, (byte)194, (byte)151, (byte)34, (byte)68, (byte)95, (byte)113, (byte)29, (byte)62, (byte)96, (byte)210, (byte)49, (byte)17, (byte)108, (byte)221, (byte)91, (byte)127, (byte)164, (byte)205, (byte)184, (byte)202, (byte)228, (byte)44, (byte)101, (byte)222, (byte)138, (byte)232, (byte)156, (byte)2, (byte)68, (byte)70, (byte)185, (byte)27}, 0) ;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -1326790193);
                Debug.Assert(pack.vel == (ushort)(ushort)34159);
                Debug.Assert(pack.cog == (ushort)(ushort)23213);
                Debug.Assert(pack.lat == (int) -1792081510);
                Debug.Assert(pack.time_usec == (ulong)2597909365866561881L);
                Debug.Assert(pack.eph == (ushort)(ushort)57309);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
                Debug.Assert(pack.alt == (int) -965164904);
                Debug.Assert(pack.epv == (ushort)(ushort)52673);
                Debug.Assert(pack.satellites_visible == (byte)(byte)247);
                Debug.Assert(pack.dgps_numch == (byte)(byte)7);
                Debug.Assert(pack.dgps_age == (uint)1007721807U);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.dgps_age = (uint)1007721807U;
            p124.epv = (ushort)(ushort)52673;
            p124.lon = (int) -1326790193;
            p124.satellites_visible = (byte)(byte)247;
            p124.time_usec = (ulong)2597909365866561881L;
            p124.eph = (ushort)(ushort)57309;
            p124.lat = (int) -1792081510;
            p124.alt = (int) -965164904;
            p124.dgps_numch = (byte)(byte)7;
            p124.cog = (ushort)(ushort)23213;
            p124.vel = (ushort)(ushort)34159;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED));
                Debug.Assert(pack.Vservo == (ushort)(ushort)40154);
                Debug.Assert(pack.Vcc == (ushort)(ushort)36620);
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)36620;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED);
            p125.Vservo = (ushort)(ushort)40154;
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING |
                                            SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND));
                Debug.Assert(pack.timeout == (ushort)(ushort)50072);
                Debug.Assert(pack.count == (byte)(byte)66);
                Debug.Assert(pack.baudrate == (uint)3881275185U);
                Debug.Assert(pack.device == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)110, (byte)58, (byte)69, (byte)223, (byte)236, (byte)234, (byte)90, (byte)13, (byte)10, (byte)56, (byte)29, (byte)67, (byte)24, (byte)227, (byte)179, (byte)193, (byte)168, (byte)255, (byte)11, (byte)221, (byte)169, (byte)165, (byte)177, (byte)49, (byte)100, (byte)140, (byte)57, (byte)173, (byte)7, (byte)250, (byte)10, (byte)89, (byte)144, (byte)101, (byte)38, (byte)54, (byte)129, (byte)187, (byte)223, (byte)116, (byte)254, (byte)170, (byte)189, (byte)236, (byte)160, (byte)75, (byte)209, (byte)231, (byte)63, (byte)162, (byte)41, (byte)89, (byte)188, (byte)135, (byte)207, (byte)53, (byte)109, (byte)105, (byte)11, (byte)55, (byte)193, (byte)66, (byte)4, (byte)203, (byte)88, (byte)117, (byte)45, (byte)42, (byte)16, (byte)126}));
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.count = (byte)(byte)66;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND);
            p126.data__SET(new byte[] {(byte)110, (byte)58, (byte)69, (byte)223, (byte)236, (byte)234, (byte)90, (byte)13, (byte)10, (byte)56, (byte)29, (byte)67, (byte)24, (byte)227, (byte)179, (byte)193, (byte)168, (byte)255, (byte)11, (byte)221, (byte)169, (byte)165, (byte)177, (byte)49, (byte)100, (byte)140, (byte)57, (byte)173, (byte)7, (byte)250, (byte)10, (byte)89, (byte)144, (byte)101, (byte)38, (byte)54, (byte)129, (byte)187, (byte)223, (byte)116, (byte)254, (byte)170, (byte)189, (byte)236, (byte)160, (byte)75, (byte)209, (byte)231, (byte)63, (byte)162, (byte)41, (byte)89, (byte)188, (byte)135, (byte)207, (byte)53, (byte)109, (byte)105, (byte)11, (byte)55, (byte)193, (byte)66, (byte)4, (byte)203, (byte)88, (byte)117, (byte)45, (byte)42, (byte)16, (byte)126}, 0) ;
            p126.baudrate = (uint)3881275185U;
            p126.timeout = (ushort)(ushort)50072;
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tow == (uint)3652738529U);
                Debug.Assert(pack.accuracy == (uint)3614362451U);
                Debug.Assert(pack.baseline_c_mm == (int) -492397033);
                Debug.Assert(pack.nsats == (byte)(byte)168);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)45);
                Debug.Assert(pack.baseline_a_mm == (int)1033083473);
                Debug.Assert(pack.wn == (ushort)(ushort)42598);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)59);
                Debug.Assert(pack.baseline_b_mm == (int) -157326138);
                Debug.Assert(pack.iar_num_hypotheses == (int) -492964763);
                Debug.Assert(pack.time_last_baseline_ms == (uint)3066156461U);
                Debug.Assert(pack.rtk_rate == (byte)(byte)102);
                Debug.Assert(pack.rtk_health == (byte)(byte)236);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.tow = (uint)3652738529U;
            p127.accuracy = (uint)3614362451U;
            p127.baseline_a_mm = (int)1033083473;
            p127.baseline_c_mm = (int) -492397033;
            p127.time_last_baseline_ms = (uint)3066156461U;
            p127.rtk_receiver_id = (byte)(byte)45;
            p127.nsats = (byte)(byte)168;
            p127.iar_num_hypotheses = (int) -492964763;
            p127.rtk_rate = (byte)(byte)102;
            p127.rtk_health = (byte)(byte)236;
            p127.baseline_b_mm = (int) -157326138;
            p127.baseline_coords_type = (byte)(byte)59;
            p127.wn = (ushort)(ushort)42598;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_last_baseline_ms == (uint)3572805777U);
                Debug.Assert(pack.tow == (uint)59807063U);
                Debug.Assert(pack.baseline_a_mm == (int)999661890);
                Debug.Assert(pack.rtk_rate == (byte)(byte)135);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)175);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)147);
                Debug.Assert(pack.wn == (ushort)(ushort)8431);
                Debug.Assert(pack.nsats == (byte)(byte)142);
                Debug.Assert(pack.baseline_b_mm == (int) -751023273);
                Debug.Assert(pack.rtk_health == (byte)(byte)80);
                Debug.Assert(pack.baseline_c_mm == (int)1946922670);
                Debug.Assert(pack.accuracy == (uint)2264329288U);
                Debug.Assert(pack.iar_num_hypotheses == (int)1865850353);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.time_last_baseline_ms = (uint)3572805777U;
            p128.tow = (uint)59807063U;
            p128.accuracy = (uint)2264329288U;
            p128.baseline_a_mm = (int)999661890;
            p128.rtk_receiver_id = (byte)(byte)147;
            p128.baseline_b_mm = (int) -751023273;
            p128.rtk_health = (byte)(byte)80;
            p128.wn = (ushort)(ushort)8431;
            p128.iar_num_hypotheses = (int)1865850353;
            p128.baseline_coords_type = (byte)(byte)175;
            p128.baseline_c_mm = (int)1946922670;
            p128.rtk_rate = (byte)(byte)135;
            p128.nsats = (byte)(byte)142;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ymag == (short)(short) -12642);
                Debug.Assert(pack.xgyro == (short)(short)8666);
                Debug.Assert(pack.time_boot_ms == (uint)2079979084U);
                Debug.Assert(pack.ygyro == (short)(short)6369);
                Debug.Assert(pack.yacc == (short)(short) -32697);
                Debug.Assert(pack.xmag == (short)(short)20149);
                Debug.Assert(pack.zacc == (short)(short) -24739);
                Debug.Assert(pack.zmag == (short)(short)6140);
                Debug.Assert(pack.xacc == (short)(short)24534);
                Debug.Assert(pack.zgyro == (short)(short) -9670);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.xacc = (short)(short)24534;
            p129.ymag = (short)(short) -12642;
            p129.zgyro = (short)(short) -9670;
            p129.ygyro = (short)(short)6369;
            p129.zacc = (short)(short) -24739;
            p129.yacc = (short)(short) -32697;
            p129.time_boot_ms = (uint)2079979084U;
            p129.zmag = (short)(short)6140;
            p129.xgyro = (short)(short)8666;
            p129.xmag = (short)(short)20149;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload == (byte)(byte)54);
                Debug.Assert(pack.size == (uint)2707932933U);
                Debug.Assert(pack.jpg_quality == (byte)(byte)228);
                Debug.Assert(pack.height == (ushort)(ushort)3941);
                Debug.Assert(pack.packets == (ushort)(ushort)56810);
                Debug.Assert(pack.width == (ushort)(ushort)48295);
                Debug.Assert(pack.type == (byte)(byte)137);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.jpg_quality = (byte)(byte)228;
            p130.size = (uint)2707932933U;
            p130.height = (ushort)(ushort)3941;
            p130.width = (ushort)(ushort)48295;
            p130.type = (byte)(byte)137;
            p130.packets = (ushort)(ushort)56810;
            p130.payload = (byte)(byte)54;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)182, (byte)23, (byte)166, (byte)47, (byte)174, (byte)100, (byte)64, (byte)17, (byte)52, (byte)82, (byte)148, (byte)114, (byte)201, (byte)15, (byte)24, (byte)252, (byte)90, (byte)0, (byte)243, (byte)130, (byte)194, (byte)104, (byte)230, (byte)76, (byte)65, (byte)251, (byte)137, (byte)26, (byte)13, (byte)21, (byte)70, (byte)209, (byte)40, (byte)56, (byte)116, (byte)90, (byte)40, (byte)107, (byte)110, (byte)158, (byte)198, (byte)18, (byte)120, (byte)46, (byte)254, (byte)14, (byte)161, (byte)92, (byte)213, (byte)166, (byte)34, (byte)186, (byte)253, (byte)152, (byte)158, (byte)118, (byte)59, (byte)234, (byte)236, (byte)131, (byte)75, (byte)54, (byte)195, (byte)223, (byte)7, (byte)149, (byte)150, (byte)234, (byte)208, (byte)123, (byte)33, (byte)180, (byte)71, (byte)21, (byte)229, (byte)104, (byte)17, (byte)55, (byte)71, (byte)143, (byte)112, (byte)116, (byte)229, (byte)174, (byte)103, (byte)138, (byte)13, (byte)72, (byte)99, (byte)214, (byte)181, (byte)212, (byte)83, (byte)215, (byte)209, (byte)177, (byte)142, (byte)14, (byte)42, (byte)227, (byte)69, (byte)21, (byte)248, (byte)233, (byte)215, (byte)26, (byte)147, (byte)58, (byte)104, (byte)11, (byte)170, (byte)57, (byte)103, (byte)47, (byte)62, (byte)198, (byte)5, (byte)85, (byte)171, (byte)215, (byte)17, (byte)209, (byte)242, (byte)103, (byte)198, (byte)212, (byte)24, (byte)146, (byte)169, (byte)98, (byte)85, (byte)229, (byte)101, (byte)207, (byte)56, (byte)158, (byte)58, (byte)215, (byte)152, (byte)181, (byte)162, (byte)27, (byte)9, (byte)18, (byte)178, (byte)1, (byte)97, (byte)111, (byte)243, (byte)71, (byte)81, (byte)217, (byte)236, (byte)34, (byte)99, (byte)235, (byte)46, (byte)84, (byte)143, (byte)93, (byte)253, (byte)224, (byte)226, (byte)40, (byte)172, (byte)168, (byte)29, (byte)113, (byte)86, (byte)2, (byte)211, (byte)231, (byte)8, (byte)134, (byte)173, (byte)100, (byte)196, (byte)55, (byte)92, (byte)64, (byte)250, (byte)248, (byte)193, (byte)103, (byte)145, (byte)24, (byte)236, (byte)216, (byte)122, (byte)112, (byte)208, (byte)71, (byte)160, (byte)192, (byte)31, (byte)189, (byte)11, (byte)73, (byte)71, (byte)135, (byte)184, (byte)166, (byte)171, (byte)230, (byte)189, (byte)166, (byte)187, (byte)144, (byte)19, (byte)5, (byte)134, (byte)111, (byte)135, (byte)102, (byte)198, (byte)252, (byte)216, (byte)17, (byte)143, (byte)206, (byte)179, (byte)72, (byte)130, (byte)173, (byte)218, (byte)32, (byte)98, (byte)195, (byte)211, (byte)46, (byte)139, (byte)162, (byte)61, (byte)107, (byte)242, (byte)3, (byte)112, (byte)179, (byte)85, (byte)9, (byte)209, (byte)63, (byte)114, (byte)133, (byte)36, (byte)193, (byte)159, (byte)42, (byte)168, (byte)5, (byte)30, (byte)230, (byte)158}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)39795);
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)39795;
            p131.data__SET(new byte[] {(byte)182, (byte)23, (byte)166, (byte)47, (byte)174, (byte)100, (byte)64, (byte)17, (byte)52, (byte)82, (byte)148, (byte)114, (byte)201, (byte)15, (byte)24, (byte)252, (byte)90, (byte)0, (byte)243, (byte)130, (byte)194, (byte)104, (byte)230, (byte)76, (byte)65, (byte)251, (byte)137, (byte)26, (byte)13, (byte)21, (byte)70, (byte)209, (byte)40, (byte)56, (byte)116, (byte)90, (byte)40, (byte)107, (byte)110, (byte)158, (byte)198, (byte)18, (byte)120, (byte)46, (byte)254, (byte)14, (byte)161, (byte)92, (byte)213, (byte)166, (byte)34, (byte)186, (byte)253, (byte)152, (byte)158, (byte)118, (byte)59, (byte)234, (byte)236, (byte)131, (byte)75, (byte)54, (byte)195, (byte)223, (byte)7, (byte)149, (byte)150, (byte)234, (byte)208, (byte)123, (byte)33, (byte)180, (byte)71, (byte)21, (byte)229, (byte)104, (byte)17, (byte)55, (byte)71, (byte)143, (byte)112, (byte)116, (byte)229, (byte)174, (byte)103, (byte)138, (byte)13, (byte)72, (byte)99, (byte)214, (byte)181, (byte)212, (byte)83, (byte)215, (byte)209, (byte)177, (byte)142, (byte)14, (byte)42, (byte)227, (byte)69, (byte)21, (byte)248, (byte)233, (byte)215, (byte)26, (byte)147, (byte)58, (byte)104, (byte)11, (byte)170, (byte)57, (byte)103, (byte)47, (byte)62, (byte)198, (byte)5, (byte)85, (byte)171, (byte)215, (byte)17, (byte)209, (byte)242, (byte)103, (byte)198, (byte)212, (byte)24, (byte)146, (byte)169, (byte)98, (byte)85, (byte)229, (byte)101, (byte)207, (byte)56, (byte)158, (byte)58, (byte)215, (byte)152, (byte)181, (byte)162, (byte)27, (byte)9, (byte)18, (byte)178, (byte)1, (byte)97, (byte)111, (byte)243, (byte)71, (byte)81, (byte)217, (byte)236, (byte)34, (byte)99, (byte)235, (byte)46, (byte)84, (byte)143, (byte)93, (byte)253, (byte)224, (byte)226, (byte)40, (byte)172, (byte)168, (byte)29, (byte)113, (byte)86, (byte)2, (byte)211, (byte)231, (byte)8, (byte)134, (byte)173, (byte)100, (byte)196, (byte)55, (byte)92, (byte)64, (byte)250, (byte)248, (byte)193, (byte)103, (byte)145, (byte)24, (byte)236, (byte)216, (byte)122, (byte)112, (byte)208, (byte)71, (byte)160, (byte)192, (byte)31, (byte)189, (byte)11, (byte)73, (byte)71, (byte)135, (byte)184, (byte)166, (byte)171, (byte)230, (byte)189, (byte)166, (byte)187, (byte)144, (byte)19, (byte)5, (byte)134, (byte)111, (byte)135, (byte)102, (byte)198, (byte)252, (byte)216, (byte)17, (byte)143, (byte)206, (byte)179, (byte)72, (byte)130, (byte)173, (byte)218, (byte)32, (byte)98, (byte)195, (byte)211, (byte)46, (byte)139, (byte)162, (byte)61, (byte)107, (byte)242, (byte)3, (byte)112, (byte)179, (byte)85, (byte)9, (byte)209, (byte)63, (byte)114, (byte)133, (byte)36, (byte)193, (byte)159, (byte)42, (byte)168, (byte)5, (byte)30, (byte)230, (byte)158}, 0) ;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
                Debug.Assert(pack.covariance == (byte)(byte)82);
                Debug.Assert(pack.max_distance == (ushort)(ushort)54162);
                Debug.Assert(pack.time_boot_ms == (uint)1490159910U);
                Debug.Assert(pack.orientation == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_45);
                Debug.Assert(pack.current_distance == (ushort)(ushort)57417);
                Debug.Assert(pack.min_distance == (ushort)(ushort)34985);
                Debug.Assert(pack.id == (byte)(byte)65);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.min_distance = (ushort)(ushort)34985;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            p132.max_distance = (ushort)(ushort)54162;
            p132.covariance = (byte)(byte)82;
            p132.id = (byte)(byte)65;
            p132.time_boot_ms = (uint)1490159910U;
            p132.current_distance = (ushort)(ushort)57417;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_45;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mask == (ulong)9147247129066627131L);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)37433);
                Debug.Assert(pack.lat == (int)1065490412);
                Debug.Assert(pack.lon == (int)1007693957);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int)1065490412;
            p133.lon = (int)1007693957;
            p133.mask = (ulong)9147247129066627131L;
            p133.grid_spacing = (ushort)(ushort)37433;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -1767690722);
                Debug.Assert(pack.lat == (int)1912130751);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -21395, (short)20718, (short)12618, (short) -1582, (short)28811, (short) -13357, (short) -21398, (short) -6833, (short) -29828, (short)19412, (short)27428, (short)19329, (short)20598, (short)17027, (short) -32180, (short) -2047}));
                Debug.Assert(pack.gridbit == (byte)(byte)17);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)45192);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.data__SET(new short[] {(short) -21395, (short)20718, (short)12618, (short) -1582, (short)28811, (short) -13357, (short) -21398, (short) -6833, (short) -29828, (short)19412, (short)27428, (short)19329, (short)20598, (short)17027, (short) -32180, (short) -2047}, 0) ;
            p134.grid_spacing = (ushort)(ushort)45192;
            p134.lon = (int) -1767690722;
            p134.gridbit = (byte)(byte)17;
            p134.lat = (int)1912130751;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1731349533);
                Debug.Assert(pack.lon == (int)44510685);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lon = (int)44510685;
            p135.lat = (int)1731349533;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_height == (float)1.7593487E38F);
                Debug.Assert(pack.terrain_height == (float)5.4401946E37F);
                Debug.Assert(pack.loaded == (ushort)(ushort)10374);
                Debug.Assert(pack.lat == (int) -464968636);
                Debug.Assert(pack.spacing == (ushort)(ushort)64395);
                Debug.Assert(pack.lon == (int)1636375004);
                Debug.Assert(pack.pending == (ushort)(ushort)57698);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.current_height = (float)1.7593487E38F;
            p136.pending = (ushort)(ushort)57698;
            p136.spacing = (ushort)(ushort)64395;
            p136.loaded = (ushort)(ushort)10374;
            p136.terrain_height = (float)5.4401946E37F;
            p136.lon = (int)1636375004;
            p136.lat = (int) -464968636;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2257204184U);
                Debug.Assert(pack.press_diff == (float) -7.0273697E37F);
                Debug.Assert(pack.temperature == (short)(short) -10105);
                Debug.Assert(pack.press_abs == (float) -1.1123192E38F);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.time_boot_ms = (uint)2257204184U;
            p137.temperature = (short)(short) -10105;
            p137.press_abs = (float) -1.1123192E38F;
            p137.press_diff = (float) -7.0273697E37F;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -1.7496955E38F);
                Debug.Assert(pack.time_usec == (ulong)2631604467811191026L);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.8283094E38F, -6.893075E37F, 1.7799096E38F, 1.9733196E38F}));
                Debug.Assert(pack.x == (float) -6.680869E37F);
                Debug.Assert(pack.y == (float) -1.6692658E37F);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.y = (float) -1.6692658E37F;
            p138.z = (float) -1.7496955E38F;
            p138.q_SET(new float[] {-2.8283094E38F, -6.893075E37F, 1.7799096E38F, 1.9733196E38F}, 0) ;
            p138.time_usec = (ulong)2631604467811191026L;
            p138.x = (float) -6.680869E37F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)218);
                Debug.Assert(pack.target_component == (byte)(byte)243);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.6277553E38F, 8.277864E36F, 1.1017348E37F, -7.802598E37F, 1.5755271E38F, 6.141107E37F, 1.666451E38F, -2.9235513E38F}));
                Debug.Assert(pack.group_mlx == (byte)(byte)2);
                Debug.Assert(pack.time_usec == (ulong)3068235378642009913L);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.group_mlx = (byte)(byte)2;
            p139.target_system = (byte)(byte)218;
            p139.target_component = (byte)(byte)243;
            p139.controls_SET(new float[] {-1.6277553E38F, 8.277864E36F, 1.1017348E37F, -7.802598E37F, 1.5755271E38F, 6.141107E37F, 1.666451E38F, -2.9235513E38F}, 0) ;
            p139.time_usec = (ulong)3068235378642009913L;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)8516873511587301075L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {9.101052E37F, 2.4444636E38F, -1.548006E38F, 2.1178034E38F, 3.0733797E38F, 1.2838526E38F, 1.9164557E38F, 1.9479403E38F}));
                Debug.Assert(pack.group_mlx == (byte)(byte)183);
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.controls_SET(new float[] {9.101052E37F, 2.4444636E38F, -1.548006E38F, 2.1178034E38F, 3.0733797E38F, 1.2838526E38F, 1.9164557E38F, 1.9479403E38F}, 0) ;
            p140.group_mlx = (byte)(byte)183;
            p140.time_usec = (ulong)8516873511587301075L;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_amsl == (float)2.1670592E38F);
                Debug.Assert(pack.altitude_monotonic == (float)1.3110506E38F);
                Debug.Assert(pack.bottom_clearance == (float)2.0576957E38F);
                Debug.Assert(pack.altitude_local == (float)1.4431773E38F);
                Debug.Assert(pack.altitude_relative == (float) -2.1024192E38F);
                Debug.Assert(pack.altitude_terrain == (float) -2.6082347E38F);
                Debug.Assert(pack.time_usec == (ulong)7526532880240352956L);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_monotonic = (float)1.3110506E38F;
            p141.altitude_terrain = (float) -2.6082347E38F;
            p141.altitude_amsl = (float)2.1670592E38F;
            p141.altitude_local = (float)1.4431773E38F;
            p141.time_usec = (ulong)7526532880240352956L;
            p141.bottom_clearance = (float)2.0576957E38F;
            p141.altitude_relative = (float) -2.1024192E38F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri_type == (byte)(byte)18);
                Debug.Assert(pack.request_id == (byte)(byte)126);
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)46, (byte)35, (byte)199, (byte)139, (byte)246, (byte)22, (byte)154, (byte)130, (byte)203, (byte)166, (byte)109, (byte)131, (byte)150, (byte)19, (byte)206, (byte)196, (byte)4, (byte)28, (byte)193, (byte)130, (byte)87, (byte)115, (byte)231, (byte)131, (byte)121, (byte)227, (byte)171, (byte)170, (byte)135, (byte)35, (byte)215, (byte)55, (byte)165, (byte)225, (byte)81, (byte)247, (byte)231, (byte)114, (byte)110, (byte)129, (byte)144, (byte)57, (byte)91, (byte)55, (byte)141, (byte)14, (byte)73, (byte)13, (byte)203, (byte)63, (byte)103, (byte)55, (byte)250, (byte)175, (byte)167, (byte)15, (byte)72, (byte)59, (byte)119, (byte)227, (byte)48, (byte)37, (byte)4, (byte)50, (byte)202, (byte)34, (byte)152, (byte)40, (byte)235, (byte)113, (byte)236, (byte)207, (byte)136, (byte)230, (byte)255, (byte)68, (byte)130, (byte)128, (byte)143, (byte)11, (byte)127, (byte)224, (byte)85, (byte)222, (byte)236, (byte)241, (byte)194, (byte)27, (byte)8, (byte)42, (byte)198, (byte)104, (byte)220, (byte)89, (byte)50, (byte)191, (byte)154, (byte)184, (byte)0, (byte)67, (byte)84, (byte)163, (byte)218, (byte)247, (byte)37, (byte)236, (byte)170, (byte)47, (byte)43, (byte)11, (byte)58, (byte)19, (byte)168, (byte)19, (byte)13, (byte)247, (byte)64, (byte)28, (byte)104, (byte)140}));
                Debug.Assert(pack.transfer_type == (byte)(byte)38);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)6, (byte)180, (byte)67, (byte)190, (byte)166, (byte)116, (byte)58, (byte)130, (byte)238, (byte)90, (byte)39, (byte)67, (byte)7, (byte)62, (byte)108, (byte)162, (byte)168, (byte)101, (byte)98, (byte)179, (byte)51, (byte)199, (byte)23, (byte)73, (byte)32, (byte)208, (byte)193, (byte)218, (byte)117, (byte)242, (byte)97, (byte)75, (byte)65, (byte)174, (byte)35, (byte)254, (byte)184, (byte)14, (byte)51, (byte)10, (byte)170, (byte)168, (byte)180, (byte)166, (byte)71, (byte)95, (byte)2, (byte)74, (byte)33, (byte)43, (byte)47, (byte)100, (byte)173, (byte)68, (byte)203, (byte)191, (byte)46, (byte)137, (byte)49, (byte)131, (byte)49, (byte)135, (byte)189, (byte)178, (byte)194, (byte)30, (byte)186, (byte)137, (byte)186, (byte)220, (byte)217, (byte)48, (byte)32, (byte)28, (byte)103, (byte)169, (byte)118, (byte)148, (byte)107, (byte)89, (byte)242, (byte)242, (byte)8, (byte)77, (byte)27, (byte)98, (byte)126, (byte)60, (byte)100, (byte)46, (byte)193, (byte)165, (byte)119, (byte)176, (byte)113, (byte)80, (byte)169, (byte)196, (byte)114, (byte)76, (byte)142, (byte)137, (byte)101, (byte)215, (byte)32, (byte)43, (byte)131, (byte)156, (byte)61, (byte)194, (byte)124, (byte)73, (byte)52, (byte)251, (byte)18, (byte)93, (byte)85, (byte)148, (byte)111, (byte)42}));
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)126;
            p142.transfer_type = (byte)(byte)38;
            p142.storage_SET(new byte[] {(byte)46, (byte)35, (byte)199, (byte)139, (byte)246, (byte)22, (byte)154, (byte)130, (byte)203, (byte)166, (byte)109, (byte)131, (byte)150, (byte)19, (byte)206, (byte)196, (byte)4, (byte)28, (byte)193, (byte)130, (byte)87, (byte)115, (byte)231, (byte)131, (byte)121, (byte)227, (byte)171, (byte)170, (byte)135, (byte)35, (byte)215, (byte)55, (byte)165, (byte)225, (byte)81, (byte)247, (byte)231, (byte)114, (byte)110, (byte)129, (byte)144, (byte)57, (byte)91, (byte)55, (byte)141, (byte)14, (byte)73, (byte)13, (byte)203, (byte)63, (byte)103, (byte)55, (byte)250, (byte)175, (byte)167, (byte)15, (byte)72, (byte)59, (byte)119, (byte)227, (byte)48, (byte)37, (byte)4, (byte)50, (byte)202, (byte)34, (byte)152, (byte)40, (byte)235, (byte)113, (byte)236, (byte)207, (byte)136, (byte)230, (byte)255, (byte)68, (byte)130, (byte)128, (byte)143, (byte)11, (byte)127, (byte)224, (byte)85, (byte)222, (byte)236, (byte)241, (byte)194, (byte)27, (byte)8, (byte)42, (byte)198, (byte)104, (byte)220, (byte)89, (byte)50, (byte)191, (byte)154, (byte)184, (byte)0, (byte)67, (byte)84, (byte)163, (byte)218, (byte)247, (byte)37, (byte)236, (byte)170, (byte)47, (byte)43, (byte)11, (byte)58, (byte)19, (byte)168, (byte)19, (byte)13, (byte)247, (byte)64, (byte)28, (byte)104, (byte)140}, 0) ;
            p142.uri_SET(new byte[] {(byte)6, (byte)180, (byte)67, (byte)190, (byte)166, (byte)116, (byte)58, (byte)130, (byte)238, (byte)90, (byte)39, (byte)67, (byte)7, (byte)62, (byte)108, (byte)162, (byte)168, (byte)101, (byte)98, (byte)179, (byte)51, (byte)199, (byte)23, (byte)73, (byte)32, (byte)208, (byte)193, (byte)218, (byte)117, (byte)242, (byte)97, (byte)75, (byte)65, (byte)174, (byte)35, (byte)254, (byte)184, (byte)14, (byte)51, (byte)10, (byte)170, (byte)168, (byte)180, (byte)166, (byte)71, (byte)95, (byte)2, (byte)74, (byte)33, (byte)43, (byte)47, (byte)100, (byte)173, (byte)68, (byte)203, (byte)191, (byte)46, (byte)137, (byte)49, (byte)131, (byte)49, (byte)135, (byte)189, (byte)178, (byte)194, (byte)30, (byte)186, (byte)137, (byte)186, (byte)220, (byte)217, (byte)48, (byte)32, (byte)28, (byte)103, (byte)169, (byte)118, (byte)148, (byte)107, (byte)89, (byte)242, (byte)242, (byte)8, (byte)77, (byte)27, (byte)98, (byte)126, (byte)60, (byte)100, (byte)46, (byte)193, (byte)165, (byte)119, (byte)176, (byte)113, (byte)80, (byte)169, (byte)196, (byte)114, (byte)76, (byte)142, (byte)137, (byte)101, (byte)215, (byte)32, (byte)43, (byte)131, (byte)156, (byte)61, (byte)194, (byte)124, (byte)73, (byte)52, (byte)251, (byte)18, (byte)93, (byte)85, (byte)148, (byte)111, (byte)42}, 0) ;
            p142.uri_type = (byte)(byte)18;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -20780);
                Debug.Assert(pack.time_boot_ms == (uint)2764791075U);
                Debug.Assert(pack.press_diff == (float) -6.5732795E37F);
                Debug.Assert(pack.press_abs == (float)2.192981E38F);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.press_abs = (float)2.192981E38F;
            p143.temperature = (short)(short) -20780;
            p143.time_boot_ms = (uint)2764791075U;
            p143.press_diff = (float) -6.5732795E37F;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.est_capabilities == (byte)(byte)184);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {3.2987108E38F, 3.468651E37F, -2.5744104E38F, 1.9028316E38F}));
                Debug.Assert(pack.timestamp == (ulong)4753149761727159570L);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-1.2351058E38F, -2.9552763E38F, 9.974373E37F}));
                Debug.Assert(pack.lat == (int)951962118);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {3.2042185E38F, -2.110505E38F, 1.2169329E38F}));
                Debug.Assert(pack.custom_state == (ulong)8929388383316329206L);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {-3.1903372E38F, 2.816875E38F, -1.0704916E38F}));
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-1.9944174E38F, 2.6701297E38F, -1.4889939E38F}));
                Debug.Assert(pack.lon == (int) -1507222631);
                Debug.Assert(pack.alt == (float)2.48635E38F);
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.rates_SET(new float[] {-1.9944174E38F, 2.6701297E38F, -1.4889939E38F}, 0) ;
            p144.custom_state = (ulong)8929388383316329206L;
            p144.timestamp = (ulong)4753149761727159570L;
            p144.lat = (int)951962118;
            p144.est_capabilities = (byte)(byte)184;
            p144.lon = (int) -1507222631;
            p144.vel_SET(new float[] {-1.2351058E38F, -2.9552763E38F, 9.974373E37F}, 0) ;
            p144.acc_SET(new float[] {-3.1903372E38F, 2.816875E38F, -1.0704916E38F}, 0) ;
            p144.attitude_q_SET(new float[] {3.2987108E38F, 3.468651E37F, -2.5744104E38F, 1.9028316E38F}, 0) ;
            p144.position_cov_SET(new float[] {3.2042185E38F, -2.110505E38F, 1.2169329E38F}, 0) ;
            p144.alt = (float)2.48635E38F;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z_acc == (float)1.6224295E38F);
                Debug.Assert(pack.z_vel == (float)2.5852984E37F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {-3.9570558E37F, -1.2360833E38F, 6.1166315E36F}));
                Debug.Assert(pack.roll_rate == (float) -2.602863E38F);
                Debug.Assert(pack.yaw_rate == (float) -2.2947648E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {5.041825E37F, 1.0291777E38F, -3.2458295E37F}));
                Debug.Assert(pack.time_usec == (ulong)4929752040188400331L);
                Debug.Assert(pack.x_vel == (float) -4.343794E34F);
                Debug.Assert(pack.x_pos == (float)9.163084E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.3722383E38F, -8.867752E37F, 1.2872317E38F, -5.3048516E37F}));
                Debug.Assert(pack.y_vel == (float)2.2794638E38F);
                Debug.Assert(pack.y_acc == (float)3.3567865E37F);
                Debug.Assert(pack.x_acc == (float)2.0785841E38F);
                Debug.Assert(pack.z_pos == (float)2.2138041E38F);
                Debug.Assert(pack.y_pos == (float)1.4962337E38F);
                Debug.Assert(pack.airspeed == (float)1.3334404E38F);
                Debug.Assert(pack.pitch_rate == (float) -3.3137137E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.x_vel = (float) -4.343794E34F;
            p146.roll_rate = (float) -2.602863E38F;
            p146.z_acc = (float)1.6224295E38F;
            p146.z_pos = (float)2.2138041E38F;
            p146.q_SET(new float[] {-1.3722383E38F, -8.867752E37F, 1.2872317E38F, -5.3048516E37F}, 0) ;
            p146.y_pos = (float)1.4962337E38F;
            p146.vel_variance_SET(new float[] {-3.9570558E37F, -1.2360833E38F, 6.1166315E36F}, 0) ;
            p146.yaw_rate = (float) -2.2947648E38F;
            p146.y_vel = (float)2.2794638E38F;
            p146.airspeed = (float)1.3334404E38F;
            p146.time_usec = (ulong)4929752040188400331L;
            p146.pitch_rate = (float) -3.3137137E38F;
            p146.x_acc = (float)2.0785841E38F;
            p146.z_vel = (float)2.5852984E37F;
            p146.x_pos = (float)9.163084E37F;
            p146.y_acc = (float)3.3567865E37F;
            p146.pos_variance_SET(new float[] {5.041825E37F, 1.0291777E38F, -3.2458295E37F}, 0) ;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_battery == (short)(short) -25064);
                Debug.Assert(pack.id == (byte)(byte)231);
                Debug.Assert(pack.temperature == (short)(short) -32005);
                Debug.Assert(pack.type == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO);
                Debug.Assert(pack.energy_consumed == (int)541189221);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)46);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)53801, (ushort)56422, (ushort)60155, (ushort)17577, (ushort)7943, (ushort)7231, (ushort)16211, (ushort)11769, (ushort)32132, (ushort)22975}));
                Debug.Assert(pack.current_consumed == (int)1640175819);
                Debug.Assert(pack.battery_function == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.current_consumed = (int)1640175819;
            p147.energy_consumed = (int)541189221;
            p147.temperature = (short)(short) -32005;
            p147.id = (byte)(byte)231;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS;
            p147.battery_remaining = (sbyte)(sbyte)46;
            p147.voltages_SET(new ushort[] {(ushort)53801, (ushort)56422, (ushort)60155, (ushort)17577, (ushort)7943, (ushort)7231, (ushort)16211, (ushort)11769, (ushort)32132, (ushort)22975}, 0) ;
            p147.current_battery = (short)(short) -25064;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)53, (byte)147, (byte)194, (byte)228, (byte)146, (byte)208, (byte)172, (byte)63}));
                Debug.Assert(pack.product_id == (ushort)(ushort)19742);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION));
                Debug.Assert(pack.uid == (ulong)2671450623552858675L);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)52886);
                Debug.Assert(pack.middleware_sw_version == (uint)2609600514U);
                Debug.Assert(pack.board_version == (uint)1704767795U);
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)242, (byte)154, (byte)167, (byte)182, (byte)28, (byte)199, (byte)44, (byte)38}));
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)104, (byte)109, (byte)143, (byte)223, (byte)141, (byte)156, (byte)101, (byte)231}));
                Debug.Assert(pack.os_sw_version == (uint)3081817340U);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)33, (byte)86, (byte)39, (byte)93, (byte)7, (byte)104, (byte)141, (byte)86, (byte)92, (byte)105, (byte)157, (byte)172, (byte)156, (byte)39, (byte)23, (byte)167, (byte)201, (byte)96}));
                Debug.Assert(pack.flight_sw_version == (uint)387730981U);
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION);
            p148.os_custom_version_SET(new byte[] {(byte)242, (byte)154, (byte)167, (byte)182, (byte)28, (byte)199, (byte)44, (byte)38}, 0) ;
            p148.flight_sw_version = (uint)387730981U;
            p148.middleware_sw_version = (uint)2609600514U;
            p148.uid2_SET(new byte[] {(byte)33, (byte)86, (byte)39, (byte)93, (byte)7, (byte)104, (byte)141, (byte)86, (byte)92, (byte)105, (byte)157, (byte)172, (byte)156, (byte)39, (byte)23, (byte)167, (byte)201, (byte)96}, 0, PH) ;
            p148.board_version = (uint)1704767795U;
            p148.middleware_custom_version_SET(new byte[] {(byte)53, (byte)147, (byte)194, (byte)228, (byte)146, (byte)208, (byte)172, (byte)63}, 0) ;
            p148.vendor_id = (ushort)(ushort)52886;
            p148.product_id = (ushort)(ushort)19742;
            p148.os_sw_version = (uint)3081817340U;
            p148.uid = (ulong)2671450623552858675L;
            p148.flight_custom_version_SET(new byte[] {(byte)104, (byte)109, (byte)143, (byte)223, (byte)141, (byte)156, (byte)101, (byte)231}, 0) ;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)224);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {2.7203607E38F, -3.3143648E38F, 2.983857E38F, -3.0693297E38F}));
                Debug.Assert(pack.size_x == (float) -7.5032817E37F);
                Debug.Assert(pack.size_y == (float) -2.6368295E38F);
                Debug.Assert(pack.angle_x == (float) -1.5498077E38F);
                Debug.Assert(pack.x_TRY(ph) == (float)2.9849097E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.type == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
                Debug.Assert(pack.target_num == (byte)(byte)11);
                Debug.Assert(pack.z_TRY(ph) == (float)3.3641865E37F);
                Debug.Assert(pack.angle_y == (float)1.674634E36F);
                Debug.Assert(pack.time_usec == (ulong)430705905203944135L);
                Debug.Assert(pack.y_TRY(ph) == (float)1.8806964E38F);
                Debug.Assert(pack.distance == (float)1.9929331E38F);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.angle_x = (float) -1.5498077E38F;
            p149.y_SET((float)1.8806964E38F, PH) ;
            p149.target_num = (byte)(byte)11;
            p149.frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p149.position_valid_SET((byte)(byte)224, PH) ;
            p149.q_SET(new float[] {2.7203607E38F, -3.3143648E38F, 2.983857E38F, -3.0693297E38F}, 0, PH) ;
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON;
            p149.time_usec = (ulong)430705905203944135L;
            p149.x_SET((float)2.9849097E38F, PH) ;
            p149.size_y = (float) -2.6368295E38F;
            p149.size_x = (float) -7.5032817E37F;
            p149.z_SET((float)3.3641865E37F, PH) ;
            p149.angle_y = (float)1.674634E36F;
            p149.distance = (float)1.9929331E38F;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCRIPT_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 14);
                Debug.Assert(pack.name_TRY(ph).Equals("iymtpuriusxtyd"));
                Debug.Assert(pack.seq == (ushort)(ushort)24616);
                Debug.Assert(pack.target_component == (byte)(byte)79);
                Debug.Assert(pack.target_system == (byte)(byte)134);
            };
            GroundControl.SCRIPT_ITEM p180 = CommunicationChannel.new_SCRIPT_ITEM();
            PH.setPack(p180);
            p180.seq = (ushort)(ushort)24616;
            p180.target_system = (byte)(byte)134;
            p180.name_SET("iymtpuriusxtyd", PH) ;
            p180.target_component = (byte)(byte)79;
            CommunicationChannel.instance.send(p180);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCRIPT_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)47);
                Debug.Assert(pack.seq == (ushort)(ushort)46996);
                Debug.Assert(pack.target_system == (byte)(byte)144);
            };
            GroundControl.SCRIPT_REQUEST p181 = CommunicationChannel.new_SCRIPT_REQUEST();
            PH.setPack(p181);
            p181.target_component = (byte)(byte)47;
            p181.target_system = (byte)(byte)144;
            p181.seq = (ushort)(ushort)46996;
            CommunicationChannel.instance.send(p181);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCRIPT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)173);
                Debug.Assert(pack.target_system == (byte)(byte)162);
            };
            GroundControl.SCRIPT_REQUEST_LIST p182 = CommunicationChannel.new_SCRIPT_REQUEST_LIST();
            PH.setPack(p182);
            p182.target_system = (byte)(byte)162;
            p182.target_component = (byte)(byte)173;
            CommunicationChannel.instance.send(p182);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCRIPT_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)219);
                Debug.Assert(pack.target_system == (byte)(byte)212);
                Debug.Assert(pack.count == (ushort)(ushort)28815);
            };
            GroundControl.SCRIPT_COUNT p183 = CommunicationChannel.new_SCRIPT_COUNT();
            PH.setPack(p183);
            p183.target_component = (byte)(byte)219;
            p183.count = (ushort)(ushort)28815;
            p183.target_system = (byte)(byte)212;
            CommunicationChannel.instance.send(p183);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCRIPT_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)51820);
            };
            GroundControl.SCRIPT_CURRENT p184 = CommunicationChannel.new_SCRIPT_CURRENT();
            PH.setPack(p184);
            p184.seq = (ushort)(ushort)51820;
            CommunicationChannel.instance.send(p184);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tas_ratio == (float)2.7096683E38F);
                Debug.Assert(pack.pos_horiz_ratio == (float) -4.8155105E37F);
                Debug.Assert(pack.mag_ratio == (float) -1.8199534E37F);
                Debug.Assert(pack.time_usec == (ulong)7009756072519137412L);
                Debug.Assert(pack.pos_vert_ratio == (float)1.6787467E38F);
                Debug.Assert(pack.vel_ratio == (float) -2.4684206E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float)1.3621321E38F);
                Debug.Assert(pack.hagl_ratio == (float) -1.1135475E38F);
                Debug.Assert(pack.pos_vert_accuracy == (float)2.4715526E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS));
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.pos_horiz_accuracy = (float)1.3621321E38F;
            p230.pos_horiz_ratio = (float) -4.8155105E37F;
            p230.vel_ratio = (float) -2.4684206E38F;
            p230.pos_vert_accuracy = (float)2.4715526E38F;
            p230.time_usec = (ulong)7009756072519137412L;
            p230.pos_vert_ratio = (float)1.6787467E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS);
            p230.tas_ratio = (float)2.7096683E38F;
            p230.mag_ratio = (float) -1.8199534E37F;
            p230.hagl_ratio = (float) -1.1135475E38F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.var_vert == (float)2.7612779E38F);
                Debug.Assert(pack.wind_x == (float) -3.7428461E37F);
                Debug.Assert(pack.wind_z == (float)1.2529556E38F);
                Debug.Assert(pack.wind_y == (float)2.6224312E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -8.4861627E37F);
                Debug.Assert(pack.time_usec == (ulong)7004160283391647472L);
                Debug.Assert(pack.var_horiz == (float) -9.683822E37F);
                Debug.Assert(pack.wind_alt == (float)2.3130126E37F);
                Debug.Assert(pack.vert_accuracy == (float) -5.928946E37F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.var_vert = (float)2.7612779E38F;
            p231.wind_y = (float)2.6224312E38F;
            p231.horiz_accuracy = (float) -8.4861627E37F;
            p231.var_horiz = (float) -9.683822E37F;
            p231.vert_accuracy = (float) -5.928946E37F;
            p231.time_usec = (ulong)7004160283391647472L;
            p231.wind_x = (float) -3.7428461E37F;
            p231.wind_alt = (float)2.3130126E37F;
            p231.wind_z = (float)1.2529556E38F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.speed_accuracy == (float) -3.5663905E37F);
                Debug.Assert(pack.vert_accuracy == (float)2.1983187E38F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY));
                Debug.Assert(pack.hdop == (float)2.5707212E38F);
                Debug.Assert(pack.vd == (float) -3.4825607E37F);
                Debug.Assert(pack.horiz_accuracy == (float)1.2718305E38F);
                Debug.Assert(pack.time_week_ms == (uint)1661038466U);
                Debug.Assert(pack.gps_id == (byte)(byte)119);
                Debug.Assert(pack.vn == (float)4.9443317E37F);
                Debug.Assert(pack.lon == (int)1272426445);
                Debug.Assert(pack.fix_type == (byte)(byte)116);
                Debug.Assert(pack.time_usec == (ulong)3094133285558394790L);
                Debug.Assert(pack.alt == (float) -2.1248075E38F);
                Debug.Assert(pack.ve == (float)2.9712217E38F);
                Debug.Assert(pack.time_week == (ushort)(ushort)43575);
                Debug.Assert(pack.vdop == (float) -1.8799275E38F);
                Debug.Assert(pack.satellites_visible == (byte)(byte)178);
                Debug.Assert(pack.lat == (int) -1769892403);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.fix_type = (byte)(byte)116;
            p232.speed_accuracy = (float) -3.5663905E37F;
            p232.lon = (int)1272426445;
            p232.horiz_accuracy = (float)1.2718305E38F;
            p232.alt = (float) -2.1248075E38F;
            p232.lat = (int) -1769892403;
            p232.hdop = (float)2.5707212E38F;
            p232.vdop = (float) -1.8799275E38F;
            p232.ve = (float)2.9712217E38F;
            p232.time_week = (ushort)(ushort)43575;
            p232.gps_id = (byte)(byte)119;
            p232.time_usec = (ulong)3094133285558394790L;
            p232.vert_accuracy = (float)2.1983187E38F;
            p232.time_week_ms = (uint)1661038466U;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY);
            p232.vd = (float) -3.4825607E37F;
            p232.satellites_visible = (byte)(byte)178;
            p232.vn = (float)4.9443317E37F;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)246);
                Debug.Assert(pack.flags == (byte)(byte)145);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)154, (byte)146, (byte)208, (byte)115, (byte)226, (byte)12, (byte)120, (byte)219, (byte)136, (byte)249, (byte)169, (byte)90, (byte)107, (byte)242, (byte)152, (byte)168, (byte)97, (byte)5, (byte)76, (byte)108, (byte)206, (byte)109, (byte)32, (byte)75, (byte)128, (byte)245, (byte)70, (byte)27, (byte)182, (byte)169, (byte)144, (byte)119, (byte)162, (byte)115, (byte)90, (byte)96, (byte)95, (byte)117, (byte)43, (byte)140, (byte)225, (byte)252, (byte)231, (byte)55, (byte)109, (byte)255, (byte)36, (byte)196, (byte)82, (byte)50, (byte)165, (byte)185, (byte)50, (byte)6, (byte)28, (byte)207, (byte)112, (byte)235, (byte)8, (byte)11, (byte)29, (byte)192, (byte)135, (byte)90, (byte)229, (byte)64, (byte)77, (byte)204, (byte)86, (byte)176, (byte)43, (byte)234, (byte)226, (byte)179, (byte)72, (byte)214, (byte)4, (byte)109, (byte)143, (byte)38, (byte)137, (byte)80, (byte)71, (byte)124, (byte)64, (byte)197, (byte)173, (byte)144, (byte)154, (byte)50, (byte)245, (byte)74, (byte)193, (byte)144, (byte)222, (byte)105, (byte)136, (byte)171, (byte)245, (byte)241, (byte)149, (byte)246, (byte)79, (byte)164, (byte)16, (byte)145, (byte)149, (byte)159, (byte)3, (byte)240, (byte)84, (byte)94, (byte)219, (byte)78, (byte)11, (byte)123, (byte)182, (byte)110, (byte)190, (byte)36, (byte)233, (byte)22, (byte)14, (byte)77, (byte)8, (byte)164, (byte)71, (byte)76, (byte)29, (byte)181, (byte)4, (byte)241, (byte)63, (byte)129, (byte)246, (byte)65, (byte)121, (byte)33, (byte)7, (byte)180, (byte)164, (byte)181, (byte)191, (byte)189, (byte)14, (byte)46, (byte)204, (byte)241, (byte)116, (byte)84, (byte)25, (byte)52, (byte)37, (byte)67, (byte)130, (byte)160, (byte)235, (byte)6, (byte)184, (byte)143, (byte)146, (byte)247, (byte)177, (byte)78, (byte)242, (byte)144, (byte)182, (byte)83, (byte)45, (byte)159, (byte)148, (byte)59, (byte)5, (byte)172, (byte)8, (byte)56, (byte)172, (byte)51, (byte)83, (byte)135}));
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.flags = (byte)(byte)145;
            p233.data__SET(new byte[] {(byte)154, (byte)146, (byte)208, (byte)115, (byte)226, (byte)12, (byte)120, (byte)219, (byte)136, (byte)249, (byte)169, (byte)90, (byte)107, (byte)242, (byte)152, (byte)168, (byte)97, (byte)5, (byte)76, (byte)108, (byte)206, (byte)109, (byte)32, (byte)75, (byte)128, (byte)245, (byte)70, (byte)27, (byte)182, (byte)169, (byte)144, (byte)119, (byte)162, (byte)115, (byte)90, (byte)96, (byte)95, (byte)117, (byte)43, (byte)140, (byte)225, (byte)252, (byte)231, (byte)55, (byte)109, (byte)255, (byte)36, (byte)196, (byte)82, (byte)50, (byte)165, (byte)185, (byte)50, (byte)6, (byte)28, (byte)207, (byte)112, (byte)235, (byte)8, (byte)11, (byte)29, (byte)192, (byte)135, (byte)90, (byte)229, (byte)64, (byte)77, (byte)204, (byte)86, (byte)176, (byte)43, (byte)234, (byte)226, (byte)179, (byte)72, (byte)214, (byte)4, (byte)109, (byte)143, (byte)38, (byte)137, (byte)80, (byte)71, (byte)124, (byte)64, (byte)197, (byte)173, (byte)144, (byte)154, (byte)50, (byte)245, (byte)74, (byte)193, (byte)144, (byte)222, (byte)105, (byte)136, (byte)171, (byte)245, (byte)241, (byte)149, (byte)246, (byte)79, (byte)164, (byte)16, (byte)145, (byte)149, (byte)159, (byte)3, (byte)240, (byte)84, (byte)94, (byte)219, (byte)78, (byte)11, (byte)123, (byte)182, (byte)110, (byte)190, (byte)36, (byte)233, (byte)22, (byte)14, (byte)77, (byte)8, (byte)164, (byte)71, (byte)76, (byte)29, (byte)181, (byte)4, (byte)241, (byte)63, (byte)129, (byte)246, (byte)65, (byte)121, (byte)33, (byte)7, (byte)180, (byte)164, (byte)181, (byte)191, (byte)189, (byte)14, (byte)46, (byte)204, (byte)241, (byte)116, (byte)84, (byte)25, (byte)52, (byte)37, (byte)67, (byte)130, (byte)160, (byte)235, (byte)6, (byte)184, (byte)143, (byte)146, (byte)247, (byte)177, (byte)78, (byte)242, (byte)144, (byte)182, (byte)83, (byte)45, (byte)159, (byte)148, (byte)59, (byte)5, (byte)172, (byte)8, (byte)56, (byte)172, (byte)51, (byte)83, (byte)135}, 0) ;
            p233.len = (byte)(byte)246;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gps_nsat == (byte)(byte)62);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED));
                Debug.Assert(pack.wp_num == (byte)(byte)98);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
                Debug.Assert(pack.battery_remaining == (byte)(byte)100);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)93);
                Debug.Assert(pack.temperature == (sbyte)(sbyte)62);
                Debug.Assert(pack.latitude == (int)453805557);
                Debug.Assert(pack.altitude_amsl == (short)(short)21360);
                Debug.Assert(pack.altitude_sp == (short)(short) -25173);
                Debug.Assert(pack.failsafe == (byte)(byte)24);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte)66);
                Debug.Assert(pack.heading_sp == (short)(short)26807);
                Debug.Assert(pack.pitch == (short)(short) -27387);
                Debug.Assert(pack.gps_fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
                Debug.Assert(pack.custom_mode == (uint)1803652995U);
                Debug.Assert(pack.roll == (short)(short) -2566);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)7579);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)53);
                Debug.Assert(pack.airspeed == (byte)(byte)137);
                Debug.Assert(pack.heading == (ushort)(ushort)44012);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)68);
                Debug.Assert(pack.groundspeed == (byte)(byte)41);
                Debug.Assert(pack.longitude == (int)1882723498);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.airspeed = (byte)(byte)137;
            p234.roll = (short)(short) -2566;
            p234.latitude = (int)453805557;
            p234.failsafe = (byte)(byte)24;
            p234.throttle = (sbyte)(sbyte)68;
            p234.wp_distance = (ushort)(ushort)7579;
            p234.heading_sp = (short)(short)26807;
            p234.airspeed_sp = (byte)(byte)93;
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
            p234.temperature = (sbyte)(sbyte)62;
            p234.gps_nsat = (byte)(byte)62;
            p234.longitude = (int)1882723498;
            p234.custom_mode = (uint)1803652995U;
            p234.heading = (ushort)(ushort)44012;
            p234.temperature_air = (sbyte)(sbyte)53;
            p234.altitude_amsl = (short)(short)21360;
            p234.wp_num = (byte)(byte)98;
            p234.altitude_sp = (short)(short) -25173;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            p234.battery_remaining = (byte)(byte)100;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p234.groundspeed = (byte)(byte)41;
            p234.pitch = (short)(short) -27387;
            p234.climb_rate = (sbyte)(sbyte)66;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vibration_x == (float)3.222784E38F);
                Debug.Assert(pack.clipping_1 == (uint)3970608201U);
                Debug.Assert(pack.clipping_2 == (uint)2480965535U);
                Debug.Assert(pack.vibration_y == (float)3.4398827E37F);
                Debug.Assert(pack.clipping_0 == (uint)4055585632U);
                Debug.Assert(pack.time_usec == (ulong)7537797748067571112L);
                Debug.Assert(pack.vibration_z == (float)5.9403015E37F);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.vibration_y = (float)3.4398827E37F;
            p241.vibration_z = (float)5.9403015E37F;
            p241.clipping_2 = (uint)2480965535U;
            p241.time_usec = (ulong)7537797748067571112L;
            p241.vibration_x = (float)3.222784E38F;
            p241.clipping_0 = (uint)4055585632U;
            p241.clipping_1 = (uint)3970608201U;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -1.5436574E38F);
                Debug.Assert(pack.altitude == (int) -1980876911);
                Debug.Assert(pack.latitude == (int) -389042484);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.951193E38F, 1.708669E38F, 2.131945E38F, -2.6333399E38F}));
                Debug.Assert(pack.z == (float) -1.3430991E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)2193500833810912622L);
                Debug.Assert(pack.approach_y == (float)3.1537843E38F);
                Debug.Assert(pack.longitude == (int)1650403604);
                Debug.Assert(pack.approach_z == (float) -2.8105447E37F);
                Debug.Assert(pack.approach_x == (float) -2.716199E38F);
                Debug.Assert(pack.x == (float)1.1485766E38F);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.altitude = (int) -1980876911;
            p242.x = (float)1.1485766E38F;
            p242.approach_z = (float) -2.8105447E37F;
            p242.latitude = (int) -389042484;
            p242.z = (float) -1.3430991E38F;
            p242.longitude = (int)1650403604;
            p242.y = (float) -1.5436574E38F;
            p242.q_SET(new float[] {-2.951193E38F, 1.708669E38F, 2.131945E38F, -2.6333399E38F}, 0) ;
            p242.approach_y = (float)3.1537843E38F;
            p242.approach_x = (float) -2.716199E38F;
            p242.time_usec_SET((ulong)2193500833810912622L, PH) ;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)3547746346374408792L);
                Debug.Assert(pack.y == (float)1.027252E38F);
                Debug.Assert(pack.x == (float) -6.330866E37F);
                Debug.Assert(pack.longitude == (int) -1661895327);
                Debug.Assert(pack.altitude == (int)832425188);
                Debug.Assert(pack.target_system == (byte)(byte)36);
                Debug.Assert(pack.approach_z == (float) -2.8695275E38F);
                Debug.Assert(pack.z == (float)1.2986649E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.0827258E38F, -7.970297E36F, -3.308876E38F, -3.1184919E38F}));
                Debug.Assert(pack.approach_x == (float) -2.7672328E38F);
                Debug.Assert(pack.latitude == (int) -1414785452);
                Debug.Assert(pack.approach_y == (float) -8.2225776E37F);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.approach_z = (float) -2.8695275E38F;
            p243.latitude = (int) -1414785452;
            p243.y = (float)1.027252E38F;
            p243.target_system = (byte)(byte)36;
            p243.approach_x = (float) -2.7672328E38F;
            p243.z = (float)1.2986649E38F;
            p243.q_SET(new float[] {1.0827258E38F, -7.970297E36F, -3.308876E38F, -3.1184919E38F}, 0) ;
            p243.longitude = (int) -1661895327;
            p243.time_usec_SET((ulong)3547746346374408792L, PH) ;
            p243.x = (float) -6.330866E37F;
            p243.approach_y = (float) -8.2225776E37F;
            p243.altitude = (int)832425188;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)49530);
                Debug.Assert(pack.interval_us == (int) -1002255435);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int) -1002255435;
            p244.message_id = (ushort)(ushort)49530;
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
                Debug.Assert(pack.lon == (int)715970777);
                Debug.Assert(pack.lat == (int) -664631007);
                Debug.Assert(pack.squawk == (ushort)(ushort)62784);
                Debug.Assert(pack.tslc == (byte)(byte)82);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)39564);
                Debug.Assert(pack.altitude == (int)1236832064);
                Debug.Assert(pack.heading == (ushort)(ushort)24807);
                Debug.Assert(pack.altitude_type == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
                Debug.Assert(pack.emitter_type == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED);
                Debug.Assert(pack.ver_velocity == (short)(short)13949);
                Debug.Assert(pack.flags == (ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN));
                Debug.Assert(pack.ICAO_address == (uint)4106129655U);
                Debug.Assert(pack.callsign_LEN(ph) == 6);
                Debug.Assert(pack.callsign_TRY(ph).Equals("wsirex"));
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN);
            p246.ICAO_address = (uint)4106129655U;
            p246.heading = (ushort)(ushort)24807;
            p246.lon = (int)715970777;
            p246.callsign_SET("wsirex", PH) ;
            p246.tslc = (byte)(byte)82;
            p246.hor_velocity = (ushort)(ushort)39564;
            p246.ver_velocity = (short)(short)13949;
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.squawk = (ushort)(ushort)62784;
            p246.altitude = (int)1236832064;
            p246.lat = (int) -664631007;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_to_minimum_delta == (float)3.3569682E38F);
                Debug.Assert(pack.src_ == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW |
                                                   MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE));
                Debug.Assert(pack.altitude_minimum_delta == (float)1.161562E37F);
                Debug.Assert(pack.id == (uint)2770326555U);
                Debug.Assert(pack.horizontal_minimum_delta == (float)4.280121E37F);
                Debug.Assert(pack.action == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.horizontal_minimum_delta = (float)4.280121E37F;
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.time_to_minimum_delta = (float)3.3569682E38F;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL;
            p247.altitude_minimum_delta = (float)1.161562E37F;
            p247.id = (uint)2770326555U;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW |
                                 MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_type == (ushort)(ushort)25270);
                Debug.Assert(pack.target_system == (byte)(byte)73);
                Debug.Assert(pack.target_component == (byte)(byte)117);
                Debug.Assert(pack.target_network == (byte)(byte)182);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)245, (byte)111, (byte)110, (byte)184, (byte)221, (byte)48, (byte)167, (byte)44, (byte)90, (byte)242, (byte)168, (byte)109, (byte)127, (byte)121, (byte)172, (byte)91, (byte)128, (byte)176, (byte)42, (byte)146, (byte)83, (byte)242, (byte)178, (byte)149, (byte)255, (byte)85, (byte)104, (byte)225, (byte)255, (byte)200, (byte)44, (byte)201, (byte)95, (byte)201, (byte)122, (byte)52, (byte)190, (byte)35, (byte)26, (byte)180, (byte)23, (byte)196, (byte)187, (byte)53, (byte)42, (byte)11, (byte)145, (byte)251, (byte)103, (byte)89, (byte)210, (byte)220, (byte)231, (byte)192, (byte)75, (byte)225, (byte)193, (byte)28, (byte)93, (byte)32, (byte)210, (byte)208, (byte)207, (byte)191, (byte)240, (byte)75, (byte)74, (byte)193, (byte)119, (byte)187, (byte)96, (byte)211, (byte)197, (byte)123, (byte)225, (byte)58, (byte)136, (byte)245, (byte)250, (byte)52, (byte)96, (byte)35, (byte)14, (byte)98, (byte)47, (byte)250, (byte)68, (byte)198, (byte)82, (byte)73, (byte)18, (byte)12, (byte)52, (byte)230, (byte)137, (byte)66, (byte)16, (byte)166, (byte)230, (byte)159, (byte)44, (byte)29, (byte)117, (byte)177, (byte)111, (byte)53, (byte)98, (byte)233, (byte)7, (byte)163, (byte)2, (byte)107, (byte)80, (byte)175, (byte)228, (byte)72, (byte)72, (byte)40, (byte)182, (byte)249, (byte)121, (byte)213, (byte)99, (byte)122, (byte)189, (byte)73, (byte)245, (byte)26, (byte)26, (byte)217, (byte)106, (byte)187, (byte)76, (byte)20, (byte)121, (byte)115, (byte)152, (byte)102, (byte)197, (byte)167, (byte)176, (byte)126, (byte)213, (byte)84, (byte)49, (byte)100, (byte)231, (byte)92, (byte)204, (byte)145, (byte)152, (byte)25, (byte)73, (byte)253, (byte)77, (byte)54, (byte)114, (byte)225, (byte)105, (byte)13, (byte)154, (byte)232, (byte)201, (byte)62, (byte)225, (byte)159, (byte)84, (byte)33, (byte)228, (byte)109, (byte)33, (byte)17, (byte)164, (byte)124, (byte)80, (byte)203, (byte)130, (byte)25, (byte)226, (byte)44, (byte)127, (byte)204, (byte)8, (byte)168, (byte)21, (byte)193, (byte)208, (byte)213, (byte)55, (byte)108, (byte)213, (byte)226, (byte)247, (byte)151, (byte)168, (byte)183, (byte)0, (byte)134, (byte)233, (byte)155, (byte)201, (byte)220, (byte)26, (byte)38, (byte)20, (byte)146, (byte)16, (byte)81, (byte)120, (byte)236, (byte)26, (byte)17, (byte)161, (byte)35, (byte)61, (byte)38, (byte)0, (byte)247, (byte)246, (byte)38, (byte)5, (byte)58, (byte)136, (byte)45, (byte)1, (byte)252, (byte)208, (byte)100, (byte)20, (byte)161, (byte)148, (byte)246, (byte)124, (byte)84, (byte)35, (byte)251, (byte)152, (byte)101, (byte)49, (byte)49, (byte)218, (byte)173, (byte)115, (byte)81, (byte)56, (byte)13, (byte)157, (byte)104, (byte)147}));
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.message_type = (ushort)(ushort)25270;
            p248.target_component = (byte)(byte)117;
            p248.target_network = (byte)(byte)182;
            p248.target_system = (byte)(byte)73;
            p248.payload_SET(new byte[] {(byte)245, (byte)111, (byte)110, (byte)184, (byte)221, (byte)48, (byte)167, (byte)44, (byte)90, (byte)242, (byte)168, (byte)109, (byte)127, (byte)121, (byte)172, (byte)91, (byte)128, (byte)176, (byte)42, (byte)146, (byte)83, (byte)242, (byte)178, (byte)149, (byte)255, (byte)85, (byte)104, (byte)225, (byte)255, (byte)200, (byte)44, (byte)201, (byte)95, (byte)201, (byte)122, (byte)52, (byte)190, (byte)35, (byte)26, (byte)180, (byte)23, (byte)196, (byte)187, (byte)53, (byte)42, (byte)11, (byte)145, (byte)251, (byte)103, (byte)89, (byte)210, (byte)220, (byte)231, (byte)192, (byte)75, (byte)225, (byte)193, (byte)28, (byte)93, (byte)32, (byte)210, (byte)208, (byte)207, (byte)191, (byte)240, (byte)75, (byte)74, (byte)193, (byte)119, (byte)187, (byte)96, (byte)211, (byte)197, (byte)123, (byte)225, (byte)58, (byte)136, (byte)245, (byte)250, (byte)52, (byte)96, (byte)35, (byte)14, (byte)98, (byte)47, (byte)250, (byte)68, (byte)198, (byte)82, (byte)73, (byte)18, (byte)12, (byte)52, (byte)230, (byte)137, (byte)66, (byte)16, (byte)166, (byte)230, (byte)159, (byte)44, (byte)29, (byte)117, (byte)177, (byte)111, (byte)53, (byte)98, (byte)233, (byte)7, (byte)163, (byte)2, (byte)107, (byte)80, (byte)175, (byte)228, (byte)72, (byte)72, (byte)40, (byte)182, (byte)249, (byte)121, (byte)213, (byte)99, (byte)122, (byte)189, (byte)73, (byte)245, (byte)26, (byte)26, (byte)217, (byte)106, (byte)187, (byte)76, (byte)20, (byte)121, (byte)115, (byte)152, (byte)102, (byte)197, (byte)167, (byte)176, (byte)126, (byte)213, (byte)84, (byte)49, (byte)100, (byte)231, (byte)92, (byte)204, (byte)145, (byte)152, (byte)25, (byte)73, (byte)253, (byte)77, (byte)54, (byte)114, (byte)225, (byte)105, (byte)13, (byte)154, (byte)232, (byte)201, (byte)62, (byte)225, (byte)159, (byte)84, (byte)33, (byte)228, (byte)109, (byte)33, (byte)17, (byte)164, (byte)124, (byte)80, (byte)203, (byte)130, (byte)25, (byte)226, (byte)44, (byte)127, (byte)204, (byte)8, (byte)168, (byte)21, (byte)193, (byte)208, (byte)213, (byte)55, (byte)108, (byte)213, (byte)226, (byte)247, (byte)151, (byte)168, (byte)183, (byte)0, (byte)134, (byte)233, (byte)155, (byte)201, (byte)220, (byte)26, (byte)38, (byte)20, (byte)146, (byte)16, (byte)81, (byte)120, (byte)236, (byte)26, (byte)17, (byte)161, (byte)35, (byte)61, (byte)38, (byte)0, (byte)247, (byte)246, (byte)38, (byte)5, (byte)58, (byte)136, (byte)45, (byte)1, (byte)252, (byte)208, (byte)100, (byte)20, (byte)161, (byte)148, (byte)246, (byte)124, (byte)84, (byte)35, (byte)251, (byte)152, (byte)101, (byte)49, (byte)49, (byte)218, (byte)173, (byte)115, (byte)81, (byte)56, (byte)13, (byte)157, (byte)104, (byte)147}, 0) ;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ver == (byte)(byte)23);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte) - 93, (sbyte) - 67, (sbyte) - 70, (sbyte) - 82, (sbyte) - 16, (sbyte)30, (sbyte) - 75, (sbyte)100, (sbyte) - 82, (sbyte) - 112, (sbyte) - 98, (sbyte) - 10, (sbyte)121, (sbyte)83, (sbyte) - 4, (sbyte) - 92, (sbyte) - 70, (sbyte) - 85, (sbyte)40, (sbyte) - 64, (sbyte)66, (sbyte) - 28, (sbyte) - 96, (sbyte)7, (sbyte) - 87, (sbyte) - 28, (sbyte) - 106, (sbyte)81, (sbyte) - 28, (sbyte) - 62, (sbyte)80, (sbyte)9}));
                Debug.Assert(pack.type == (byte)(byte)213);
                Debug.Assert(pack.address == (ushort)(ushort)37251);
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.type = (byte)(byte)213;
            p249.value_SET(new sbyte[] {(sbyte) - 93, (sbyte) - 67, (sbyte) - 70, (sbyte) - 82, (sbyte) - 16, (sbyte)30, (sbyte) - 75, (sbyte)100, (sbyte) - 82, (sbyte) - 112, (sbyte) - 98, (sbyte) - 10, (sbyte)121, (sbyte)83, (sbyte) - 4, (sbyte) - 92, (sbyte) - 70, (sbyte) - 85, (sbyte)40, (sbyte) - 64, (sbyte)66, (sbyte) - 28, (sbyte) - 96, (sbyte)7, (sbyte) - 87, (sbyte) - 28, (sbyte) - 106, (sbyte)81, (sbyte) - 28, (sbyte) - 62, (sbyte)80, (sbyte)9}, 0) ;
            p249.ver = (byte)(byte)23;
            p249.address = (ushort)(ushort)37251;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)2.119437E38F);
                Debug.Assert(pack.x == (float) -1.6169785E38F);
                Debug.Assert(pack.name_LEN(ph) == 10);
                Debug.Assert(pack.name_TRY(ph).Equals("ekfeYynjni"));
                Debug.Assert(pack.z == (float) -2.0599578E38F);
                Debug.Assert(pack.time_usec == (ulong)2410419593800107962L);
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.y = (float)2.119437E38F;
            p250.x = (float) -1.6169785E38F;
            p250.z = (float) -2.0599578E38F;
            p250.name_SET("ekfeYynjni", PH) ;
            p250.time_usec = (ulong)2410419593800107962L;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 10);
                Debug.Assert(pack.name_TRY(ph).Equals("elhxpomgol"));
                Debug.Assert(pack.value == (float)2.704502E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1449178210U);
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.name_SET("elhxpomgol", PH) ;
            p251.time_boot_ms = (uint)1449178210U;
            p251.value = (float)2.704502E38F;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 10);
                Debug.Assert(pack.name_TRY(ph).Equals("gwMryqepyc"));
                Debug.Assert(pack.value == (int) -1776831333);
                Debug.Assert(pack.time_boot_ms == (uint)74042698U);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.value = (int) -1776831333;
            p252.name_SET("gwMryqepyc", PH) ;
            p252.time_boot_ms = (uint)74042698U;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 10);
                Debug.Assert(pack.text_TRY(ph).Equals("giefayCtrb"));
                Debug.Assert(pack.severity == MAV_SEVERITY.MAV_SEVERITY_WARNING);
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("giefayCtrb", PH) ;
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_WARNING;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ind == (byte)(byte)196);
                Debug.Assert(pack.value == (float) -1.9914655E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2575211716U);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.ind = (byte)(byte)196;
            p254.time_boot_ms = (uint)2575211716U;
            p254.value = (float) -1.9914655E38F;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)152, (byte)38, (byte)241, (byte)221, (byte)54, (byte)117, (byte)182, (byte)48, (byte)187, (byte)76, (byte)173, (byte)38, (byte)137, (byte)90, (byte)90, (byte)13, (byte)215, (byte)135, (byte)238, (byte)120, (byte)249, (byte)176, (byte)214, (byte)11, (byte)156, (byte)243, (byte)193, (byte)200, (byte)198, (byte)172, (byte)55, (byte)144}));
                Debug.Assert(pack.target_system == (byte)(byte)49);
                Debug.Assert(pack.target_component == (byte)(byte)31);
                Debug.Assert(pack.initial_timestamp == (ulong)9177542924399432823L);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)49;
            p256.target_component = (byte)(byte)31;
            p256.secret_key_SET(new byte[] {(byte)152, (byte)38, (byte)241, (byte)221, (byte)54, (byte)117, (byte)182, (byte)48, (byte)187, (byte)76, (byte)173, (byte)38, (byte)137, (byte)90, (byte)90, (byte)13, (byte)215, (byte)135, (byte)238, (byte)120, (byte)249, (byte)176, (byte)214, (byte)11, (byte)156, (byte)243, (byte)193, (byte)200, (byte)198, (byte)172, (byte)55, (byte)144}, 0) ;
            p256.initial_timestamp = (ulong)9177542924399432823L;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_change_ms == (uint)854713597U);
                Debug.Assert(pack.time_boot_ms == (uint)575566349U);
                Debug.Assert(pack.state == (byte)(byte)136);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.last_change_ms = (uint)854713597U;
            p257.state = (byte)(byte)136;
            p257.time_boot_ms = (uint)575566349U;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)219);
                Debug.Assert(pack.tune_LEN(ph) == 2);
                Debug.Assert(pack.tune_TRY(ph).Equals("iw"));
                Debug.Assert(pack.target_component == (byte)(byte)169);
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_component = (byte)(byte)169;
            p258.tune_SET("iw", PH) ;
            p258.target_system = (byte)(byte)219;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)239, (byte)17, (byte)229, (byte)218, (byte)252, (byte)187, (byte)131, (byte)49, (byte)161, (byte)121, (byte)77, (byte)14, (byte)158, (byte)28, (byte)225, (byte)195, (byte)238, (byte)186, (byte)191, (byte)149, (byte)21, (byte)120, (byte)43, (byte)133, (byte)15, (byte)6, (byte)215, (byte)128, (byte)194, (byte)188, (byte)5, (byte)245}));
                Debug.Assert(pack.resolution_v == (ushort)(ushort)15110);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)34062);
                Debug.Assert(pack.time_boot_ms == (uint)4210547458U);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 125);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("mavjvjDrXemEdbxiuhxdzhwgzfRulaVcxrBlmboyBjYzglaamUEmgymgtryrqkonSqvYaximunjHarhlkahkylvvpmHevgwBdormcjkslehkxUyQdarnlqekuhsvi"));
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE));
                Debug.Assert(pack.sensor_size_v == (float) -1.6249669E38F);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)239, (byte)76, (byte)235, (byte)228, (byte)96, (byte)171, (byte)115, (byte)136, (byte)138, (byte)122, (byte)173, (byte)135, (byte)166, (byte)242, (byte)96, (byte)225, (byte)8, (byte)113, (byte)175, (byte)109, (byte)12, (byte)119, (byte)91, (byte)92, (byte)189, (byte)59, (byte)255, (byte)18, (byte)58, (byte)120, (byte)195, (byte)26}));
                Debug.Assert(pack.resolution_h == (ushort)(ushort)30330);
                Debug.Assert(pack.lens_id == (byte)(byte)70);
                Debug.Assert(pack.firmware_version == (uint)3582829240U);
                Debug.Assert(pack.focal_length == (float)1.2149781E38F);
                Debug.Assert(pack.sensor_size_h == (float) -7.198578E36F);
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE);
            p259.cam_definition_uri_SET("mavjvjDrXemEdbxiuhxdzhwgzfRulaVcxrBlmboyBjYzglaamUEmgymgtryrqkonSqvYaximunjHarhlkahkylvvpmHevgwBdormcjkslehkxUyQdarnlqekuhsvi", PH) ;
            p259.sensor_size_h = (float) -7.198578E36F;
            p259.firmware_version = (uint)3582829240U;
            p259.model_name_SET(new byte[] {(byte)239, (byte)17, (byte)229, (byte)218, (byte)252, (byte)187, (byte)131, (byte)49, (byte)161, (byte)121, (byte)77, (byte)14, (byte)158, (byte)28, (byte)225, (byte)195, (byte)238, (byte)186, (byte)191, (byte)149, (byte)21, (byte)120, (byte)43, (byte)133, (byte)15, (byte)6, (byte)215, (byte)128, (byte)194, (byte)188, (byte)5, (byte)245}, 0) ;
            p259.cam_definition_version = (ushort)(ushort)34062;
            p259.sensor_size_v = (float) -1.6249669E38F;
            p259.resolution_v = (ushort)(ushort)15110;
            p259.resolution_h = (ushort)(ushort)30330;
            p259.lens_id = (byte)(byte)70;
            p259.time_boot_ms = (uint)4210547458U;
            p259.vendor_name_SET(new byte[] {(byte)239, (byte)76, (byte)235, (byte)228, (byte)96, (byte)171, (byte)115, (byte)136, (byte)138, (byte)122, (byte)173, (byte)135, (byte)166, (byte)242, (byte)96, (byte)225, (byte)8, (byte)113, (byte)175, (byte)109, (byte)12, (byte)119, (byte)91, (byte)92, (byte)189, (byte)59, (byte)255, (byte)18, (byte)58, (byte)120, (byte)195, (byte)26}, 0) ;
            p259.focal_length = (float)1.2149781E38F;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2062624403U);
                Debug.Assert(pack.mode_id == (CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY));
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = (CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
            p260.time_boot_ms = (uint)2062624403U;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.storage_id == (byte)(byte)48);
                Debug.Assert(pack.write_speed == (float) -1.7846942E38F);
                Debug.Assert(pack.storage_count == (byte)(byte)214);
                Debug.Assert(pack.available_capacity == (float) -8.1906746E36F);
                Debug.Assert(pack.used_capacity == (float) -1.7162263E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2008285535U);
                Debug.Assert(pack.total_capacity == (float) -2.0537303E38F);
                Debug.Assert(pack.status == (byte)(byte)43);
                Debug.Assert(pack.read_speed == (float)2.661318E38F);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.write_speed = (float) -1.7846942E38F;
            p261.available_capacity = (float) -8.1906746E36F;
            p261.storage_id = (byte)(byte)48;
            p261.total_capacity = (float) -2.0537303E38F;
            p261.status = (byte)(byte)43;
            p261.storage_count = (byte)(byte)214;
            p261.used_capacity = (float) -1.7162263E38F;
            p261.time_boot_ms = (uint)2008285535U;
            p261.read_speed = (float)2.661318E38F;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.available_capacity == (float)1.595546E38F);
                Debug.Assert(pack.image_interval == (float) -1.2697874E38F);
                Debug.Assert(pack.time_boot_ms == (uint)707567530U);
                Debug.Assert(pack.image_status == (byte)(byte)132);
                Debug.Assert(pack.recording_time_ms == (uint)4014456100U);
                Debug.Assert(pack.video_status == (byte)(byte)170);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.time_boot_ms = (uint)707567530U;
            p262.recording_time_ms = (uint)4014456100U;
            p262.available_capacity = (float)1.595546E38F;
            p262.video_status = (byte)(byte)170;
            p262.image_status = (byte)(byte)132;
            p262.image_interval = (float) -1.2697874E38F;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.image_index == (int) -1207238123);
                Debug.Assert(pack.lat == (int)1967913692);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)43);
                Debug.Assert(pack.relative_alt == (int) -516812373);
                Debug.Assert(pack.camera_id == (byte)(byte)120);
                Debug.Assert(pack.alt == (int) -936759508);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.2926376E38F, -1.1421919E38F, 4.3684766E37F, 5.5768955E37F}));
                Debug.Assert(pack.time_utc == (ulong)1931346928557068L);
                Debug.Assert(pack.time_boot_ms == (uint)1909575408U);
                Debug.Assert(pack.file_url_LEN(ph) == 106);
                Debug.Assert(pack.file_url_TRY(ph).Equals("xgcdsbhrylbcnhefAbbqqgblpFuOyeYhtexdphqmxoTkwtyjsqhpiqlvppsmAuzccfazljkpyhgxoovcAxxriTtmdfkpfthfTiwjdnnflv"));
                Debug.Assert(pack.lon == (int) -1960412845);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.image_index = (int) -1207238123;
            p263.lat = (int)1967913692;
            p263.lon = (int) -1960412845;
            p263.q_SET(new float[] {2.2926376E38F, -1.1421919E38F, 4.3684766E37F, 5.5768955E37F}, 0) ;
            p263.time_utc = (ulong)1931346928557068L;
            p263.camera_id = (byte)(byte)120;
            p263.capture_result = (sbyte)(sbyte)43;
            p263.relative_alt = (int) -516812373;
            p263.file_url_SET("xgcdsbhrylbcnhefAbbqqgblpFuOyeYhtexdphqmxoTkwtyjsqhpiqlvppsmAuzccfazljkpyhgxoovcAxxriTtmdfkpfthfTiwjdnnflv", PH) ;
            p263.time_boot_ms = (uint)1909575408U;
            p263.alt = (int) -936759508;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flight_uuid == (ulong)1956385753605545493L);
                Debug.Assert(pack.time_boot_ms == (uint)79705356U);
                Debug.Assert(pack.arming_time_utc == (ulong)2578999886961771471L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)817154757780303369L);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.flight_uuid = (ulong)1956385753605545493L;
            p264.arming_time_utc = (ulong)2578999886961771471L;
            p264.takeoff_time_utc = (ulong)817154757780303369L;
            p264.time_boot_ms = (uint)79705356U;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)3.3055855E38F);
                Debug.Assert(pack.time_boot_ms == (uint)810715587U);
                Debug.Assert(pack.pitch == (float) -3.0615654E38F);
                Debug.Assert(pack.roll == (float)1.8736227E38F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.yaw = (float)3.3055855E38F;
            p265.time_boot_ms = (uint)810715587U;
            p265.pitch = (float) -3.0615654E38F;
            p265.roll = (float)1.8736227E38F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.length == (byte)(byte)141);
                Debug.Assert(pack.sequence == (ushort)(ushort)14915);
                Debug.Assert(pack.target_system == (byte)(byte)129);
                Debug.Assert(pack.target_component == (byte)(byte)216);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)52, (byte)99, (byte)255, (byte)69, (byte)101, (byte)54, (byte)33, (byte)150, (byte)155, (byte)68, (byte)22, (byte)73, (byte)203, (byte)21, (byte)38, (byte)192, (byte)43, (byte)249, (byte)23, (byte)60, (byte)127, (byte)144, (byte)76, (byte)6, (byte)151, (byte)244, (byte)88, (byte)251, (byte)23, (byte)219, (byte)59, (byte)105, (byte)51, (byte)37, (byte)242, (byte)74, (byte)61, (byte)59, (byte)174, (byte)175, (byte)121, (byte)69, (byte)94, (byte)171, (byte)231, (byte)116, (byte)198, (byte)252, (byte)68, (byte)170, (byte)136, (byte)255, (byte)147, (byte)108, (byte)156, (byte)190, (byte)109, (byte)130, (byte)120, (byte)132, (byte)115, (byte)13, (byte)109, (byte)161, (byte)158, (byte)196, (byte)143, (byte)236, (byte)164, (byte)91, (byte)112, (byte)217, (byte)14, (byte)230, (byte)54, (byte)214, (byte)117, (byte)55, (byte)251, (byte)190, (byte)201, (byte)111, (byte)46, (byte)90, (byte)64, (byte)162, (byte)63, (byte)12, (byte)219, (byte)39, (byte)50, (byte)128, (byte)97, (byte)132, (byte)53, (byte)241, (byte)183, (byte)115, (byte)170, (byte)83, (byte)239, (byte)248, (byte)177, (byte)70, (byte)134, (byte)251, (byte)0, (byte)115, (byte)243, (byte)175, (byte)228, (byte)44, (byte)207, (byte)162, (byte)185, (byte)220, (byte)246, (byte)248, (byte)9, (byte)126, (byte)84, (byte)192, (byte)87, (byte)178, (byte)94, (byte)131, (byte)206, (byte)23, (byte)125, (byte)217, (byte)28, (byte)37, (byte)104, (byte)236, (byte)186, (byte)177, (byte)43, (byte)82, (byte)214, (byte)64, (byte)16, (byte)96, (byte)251, (byte)241, (byte)43, (byte)162, (byte)246, (byte)75, (byte)254, (byte)230, (byte)220, (byte)46, (byte)65, (byte)53, (byte)41, (byte)209, (byte)146, (byte)218, (byte)221, (byte)63, (byte)43, (byte)56, (byte)223, (byte)228, (byte)186, (byte)133, (byte)106, (byte)14, (byte)156, (byte)26, (byte)169, (byte)112, (byte)156, (byte)109, (byte)255, (byte)177, (byte)177, (byte)169, (byte)48, (byte)44, (byte)189, (byte)131, (byte)12, (byte)79, (byte)217, (byte)134, (byte)31, (byte)255, (byte)247, (byte)30, (byte)176, (byte)133, (byte)221, (byte)221, (byte)110, (byte)211, (byte)112, (byte)133, (byte)105, (byte)250, (byte)128, (byte)122, (byte)129, (byte)5, (byte)226, (byte)85, (byte)215, (byte)117, (byte)248, (byte)70, (byte)16, (byte)231, (byte)248, (byte)106, (byte)131, (byte)189, (byte)107, (byte)21, (byte)224, (byte)120, (byte)127, (byte)205, (byte)133, (byte)52, (byte)245, (byte)46, (byte)243, (byte)250, (byte)109, (byte)125, (byte)153, (byte)180, (byte)61, (byte)68, (byte)99, (byte)84, (byte)53, (byte)196, (byte)254, (byte)9, (byte)96, (byte)49, (byte)143, (byte)55, (byte)250, (byte)118, (byte)22, (byte)63, (byte)139}));
                Debug.Assert(pack.first_message_offset == (byte)(byte)234);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.sequence = (ushort)(ushort)14915;
            p266.length = (byte)(byte)141;
            p266.data__SET(new byte[] {(byte)52, (byte)99, (byte)255, (byte)69, (byte)101, (byte)54, (byte)33, (byte)150, (byte)155, (byte)68, (byte)22, (byte)73, (byte)203, (byte)21, (byte)38, (byte)192, (byte)43, (byte)249, (byte)23, (byte)60, (byte)127, (byte)144, (byte)76, (byte)6, (byte)151, (byte)244, (byte)88, (byte)251, (byte)23, (byte)219, (byte)59, (byte)105, (byte)51, (byte)37, (byte)242, (byte)74, (byte)61, (byte)59, (byte)174, (byte)175, (byte)121, (byte)69, (byte)94, (byte)171, (byte)231, (byte)116, (byte)198, (byte)252, (byte)68, (byte)170, (byte)136, (byte)255, (byte)147, (byte)108, (byte)156, (byte)190, (byte)109, (byte)130, (byte)120, (byte)132, (byte)115, (byte)13, (byte)109, (byte)161, (byte)158, (byte)196, (byte)143, (byte)236, (byte)164, (byte)91, (byte)112, (byte)217, (byte)14, (byte)230, (byte)54, (byte)214, (byte)117, (byte)55, (byte)251, (byte)190, (byte)201, (byte)111, (byte)46, (byte)90, (byte)64, (byte)162, (byte)63, (byte)12, (byte)219, (byte)39, (byte)50, (byte)128, (byte)97, (byte)132, (byte)53, (byte)241, (byte)183, (byte)115, (byte)170, (byte)83, (byte)239, (byte)248, (byte)177, (byte)70, (byte)134, (byte)251, (byte)0, (byte)115, (byte)243, (byte)175, (byte)228, (byte)44, (byte)207, (byte)162, (byte)185, (byte)220, (byte)246, (byte)248, (byte)9, (byte)126, (byte)84, (byte)192, (byte)87, (byte)178, (byte)94, (byte)131, (byte)206, (byte)23, (byte)125, (byte)217, (byte)28, (byte)37, (byte)104, (byte)236, (byte)186, (byte)177, (byte)43, (byte)82, (byte)214, (byte)64, (byte)16, (byte)96, (byte)251, (byte)241, (byte)43, (byte)162, (byte)246, (byte)75, (byte)254, (byte)230, (byte)220, (byte)46, (byte)65, (byte)53, (byte)41, (byte)209, (byte)146, (byte)218, (byte)221, (byte)63, (byte)43, (byte)56, (byte)223, (byte)228, (byte)186, (byte)133, (byte)106, (byte)14, (byte)156, (byte)26, (byte)169, (byte)112, (byte)156, (byte)109, (byte)255, (byte)177, (byte)177, (byte)169, (byte)48, (byte)44, (byte)189, (byte)131, (byte)12, (byte)79, (byte)217, (byte)134, (byte)31, (byte)255, (byte)247, (byte)30, (byte)176, (byte)133, (byte)221, (byte)221, (byte)110, (byte)211, (byte)112, (byte)133, (byte)105, (byte)250, (byte)128, (byte)122, (byte)129, (byte)5, (byte)226, (byte)85, (byte)215, (byte)117, (byte)248, (byte)70, (byte)16, (byte)231, (byte)248, (byte)106, (byte)131, (byte)189, (byte)107, (byte)21, (byte)224, (byte)120, (byte)127, (byte)205, (byte)133, (byte)52, (byte)245, (byte)46, (byte)243, (byte)250, (byte)109, (byte)125, (byte)153, (byte)180, (byte)61, (byte)68, (byte)99, (byte)84, (byte)53, (byte)196, (byte)254, (byte)9, (byte)96, (byte)49, (byte)143, (byte)55, (byte)250, (byte)118, (byte)22, (byte)63, (byte)139}, 0) ;
            p266.target_component = (byte)(byte)216;
            p266.target_system = (byte)(byte)129;
            p266.first_message_offset = (byte)(byte)234;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sequence == (ushort)(ushort)37476);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)151, (byte)88, (byte)164, (byte)198, (byte)20, (byte)179, (byte)241, (byte)174, (byte)185, (byte)174, (byte)74, (byte)133, (byte)143, (byte)251, (byte)155, (byte)115, (byte)147, (byte)202, (byte)64, (byte)240, (byte)201, (byte)2, (byte)178, (byte)196, (byte)248, (byte)178, (byte)109, (byte)184, (byte)45, (byte)26, (byte)188, (byte)224, (byte)118, (byte)217, (byte)95, (byte)13, (byte)56, (byte)89, (byte)55, (byte)57, (byte)51, (byte)233, (byte)173, (byte)252, (byte)219, (byte)123, (byte)19, (byte)124, (byte)135, (byte)46, (byte)67, (byte)138, (byte)62, (byte)186, (byte)30, (byte)26, (byte)100, (byte)2, (byte)46, (byte)48, (byte)2, (byte)55, (byte)91, (byte)73, (byte)34, (byte)254, (byte)205, (byte)82, (byte)3, (byte)79, (byte)59, (byte)158, (byte)129, (byte)130, (byte)95, (byte)61, (byte)255, (byte)189, (byte)211, (byte)212, (byte)36, (byte)95, (byte)5, (byte)133, (byte)168, (byte)190, (byte)162, (byte)196, (byte)166, (byte)49, (byte)90, (byte)202, (byte)1, (byte)127, (byte)8, (byte)68, (byte)148, (byte)95, (byte)130, (byte)59, (byte)162, (byte)121, (byte)226, (byte)90, (byte)215, (byte)171, (byte)145, (byte)215, (byte)32, (byte)200, (byte)160, (byte)216, (byte)247, (byte)239, (byte)82, (byte)3, (byte)132, (byte)173, (byte)142, (byte)81, (byte)160, (byte)29, (byte)111, (byte)187, (byte)203, (byte)102, (byte)8, (byte)21, (byte)79, (byte)77, (byte)212, (byte)68, (byte)177, (byte)40, (byte)208, (byte)221, (byte)224, (byte)87, (byte)108, (byte)187, (byte)194, (byte)249, (byte)149, (byte)220, (byte)3, (byte)6, (byte)164, (byte)233, (byte)86, (byte)216, (byte)27, (byte)68, (byte)124, (byte)138, (byte)63, (byte)104, (byte)184, (byte)210, (byte)255, (byte)30, (byte)202, (byte)43, (byte)10, (byte)213, (byte)45, (byte)203, (byte)202, (byte)72, (byte)49, (byte)172, (byte)161, (byte)57, (byte)129, (byte)208, (byte)174, (byte)1, (byte)118, (byte)108, (byte)32, (byte)116, (byte)254, (byte)135, (byte)202, (byte)61, (byte)168, (byte)42, (byte)15, (byte)116, (byte)33, (byte)136, (byte)128, (byte)68, (byte)255, (byte)40, (byte)235, (byte)244, (byte)250, (byte)35, (byte)38, (byte)58, (byte)98, (byte)175, (byte)97, (byte)227, (byte)190, (byte)242, (byte)44, (byte)165, (byte)52, (byte)66, (byte)55, (byte)100, (byte)108, (byte)140, (byte)165, (byte)165, (byte)252, (byte)159, (byte)13, (byte)30, (byte)212, (byte)163, (byte)212, (byte)140, (byte)97, (byte)33, (byte)223, (byte)220, (byte)75, (byte)234, (byte)11, (byte)138, (byte)231, (byte)154, (byte)163, (byte)92, (byte)11, (byte)145, (byte)157, (byte)230, (byte)94, (byte)211, (byte)32, (byte)194, (byte)149, (byte)161, (byte)31, (byte)97, (byte)40}));
                Debug.Assert(pack.first_message_offset == (byte)(byte)128);
                Debug.Assert(pack.length == (byte)(byte)235);
                Debug.Assert(pack.target_system == (byte)(byte)95);
                Debug.Assert(pack.target_component == (byte)(byte)248);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.length = (byte)(byte)235;
            p267.data__SET(new byte[] {(byte)151, (byte)88, (byte)164, (byte)198, (byte)20, (byte)179, (byte)241, (byte)174, (byte)185, (byte)174, (byte)74, (byte)133, (byte)143, (byte)251, (byte)155, (byte)115, (byte)147, (byte)202, (byte)64, (byte)240, (byte)201, (byte)2, (byte)178, (byte)196, (byte)248, (byte)178, (byte)109, (byte)184, (byte)45, (byte)26, (byte)188, (byte)224, (byte)118, (byte)217, (byte)95, (byte)13, (byte)56, (byte)89, (byte)55, (byte)57, (byte)51, (byte)233, (byte)173, (byte)252, (byte)219, (byte)123, (byte)19, (byte)124, (byte)135, (byte)46, (byte)67, (byte)138, (byte)62, (byte)186, (byte)30, (byte)26, (byte)100, (byte)2, (byte)46, (byte)48, (byte)2, (byte)55, (byte)91, (byte)73, (byte)34, (byte)254, (byte)205, (byte)82, (byte)3, (byte)79, (byte)59, (byte)158, (byte)129, (byte)130, (byte)95, (byte)61, (byte)255, (byte)189, (byte)211, (byte)212, (byte)36, (byte)95, (byte)5, (byte)133, (byte)168, (byte)190, (byte)162, (byte)196, (byte)166, (byte)49, (byte)90, (byte)202, (byte)1, (byte)127, (byte)8, (byte)68, (byte)148, (byte)95, (byte)130, (byte)59, (byte)162, (byte)121, (byte)226, (byte)90, (byte)215, (byte)171, (byte)145, (byte)215, (byte)32, (byte)200, (byte)160, (byte)216, (byte)247, (byte)239, (byte)82, (byte)3, (byte)132, (byte)173, (byte)142, (byte)81, (byte)160, (byte)29, (byte)111, (byte)187, (byte)203, (byte)102, (byte)8, (byte)21, (byte)79, (byte)77, (byte)212, (byte)68, (byte)177, (byte)40, (byte)208, (byte)221, (byte)224, (byte)87, (byte)108, (byte)187, (byte)194, (byte)249, (byte)149, (byte)220, (byte)3, (byte)6, (byte)164, (byte)233, (byte)86, (byte)216, (byte)27, (byte)68, (byte)124, (byte)138, (byte)63, (byte)104, (byte)184, (byte)210, (byte)255, (byte)30, (byte)202, (byte)43, (byte)10, (byte)213, (byte)45, (byte)203, (byte)202, (byte)72, (byte)49, (byte)172, (byte)161, (byte)57, (byte)129, (byte)208, (byte)174, (byte)1, (byte)118, (byte)108, (byte)32, (byte)116, (byte)254, (byte)135, (byte)202, (byte)61, (byte)168, (byte)42, (byte)15, (byte)116, (byte)33, (byte)136, (byte)128, (byte)68, (byte)255, (byte)40, (byte)235, (byte)244, (byte)250, (byte)35, (byte)38, (byte)58, (byte)98, (byte)175, (byte)97, (byte)227, (byte)190, (byte)242, (byte)44, (byte)165, (byte)52, (byte)66, (byte)55, (byte)100, (byte)108, (byte)140, (byte)165, (byte)165, (byte)252, (byte)159, (byte)13, (byte)30, (byte)212, (byte)163, (byte)212, (byte)140, (byte)97, (byte)33, (byte)223, (byte)220, (byte)75, (byte)234, (byte)11, (byte)138, (byte)231, (byte)154, (byte)163, (byte)92, (byte)11, (byte)145, (byte)157, (byte)230, (byte)94, (byte)211, (byte)32, (byte)194, (byte)149, (byte)161, (byte)31, (byte)97, (byte)40}, 0) ;
            p267.target_system = (byte)(byte)95;
            p267.first_message_offset = (byte)(byte)128;
            p267.target_component = (byte)(byte)248;
            p267.sequence = (ushort)(ushort)37476;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)71);
                Debug.Assert(pack.sequence == (ushort)(ushort)64144);
                Debug.Assert(pack.target_system == (byte)(byte)66);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_component = (byte)(byte)71;
            p268.sequence = (ushort)(ushort)64144;
            p268.target_system = (byte)(byte)66;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.status == (byte)(byte)240);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)25432);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)57915);
                Debug.Assert(pack.camera_id == (byte)(byte)15);
                Debug.Assert(pack.bitrate == (uint)794616748U);
                Debug.Assert(pack.rotation == (ushort)(ushort)29421);
                Debug.Assert(pack.uri_LEN(ph) == 129);
                Debug.Assert(pack.uri_TRY(ph).Equals("FxRzazsjzydbpvwwragweobcpymidvzjhzwyimmkvIjsdqbxTuufxvhvyeyqBtqgxrfftYindlaJtumzlkwAjzdJEtmlshnBqowywsnzdyuXczryxdidTamacongvlxci"));
                Debug.Assert(pack.framerate == (float)3.3033291E38F);
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.resolution_h = (ushort)(ushort)25432;
            p269.status = (byte)(byte)240;
            p269.resolution_v = (ushort)(ushort)57915;
            p269.uri_SET("FxRzazsjzydbpvwwragweobcpymidvzjhzwyimmkvIjsdqbxTuufxvhvyeyqBtqgxrfftYindlaJtumzlkwAjzdJEtmlshnBqowywsnzdyuXczryxdidTamacongvlxci", PH) ;
            p269.camera_id = (byte)(byte)15;
            p269.bitrate = (uint)794616748U;
            p269.rotation = (ushort)(ushort)29421;
            p269.framerate = (float)3.3033291E38F;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)65);
                Debug.Assert(pack.target_system == (byte)(byte)15);
                Debug.Assert(pack.bitrate == (uint)708530833U);
                Debug.Assert(pack.framerate == (float) -2.5912656E38F);
                Debug.Assert(pack.camera_id == (byte)(byte)148);
                Debug.Assert(pack.uri_LEN(ph) == 196);
                Debug.Assert(pack.uri_TRY(ph).Equals("zlrpggcokegltpqghhWkmufVncoxymziumZoxvhpkgyxodeyPGHnaktzlfpecbwpHnccxmffddttvvfTaeneoonabwsoybdagdCuSbchxvulCqbsjabgpcpcimnboluzxJyxrnbsnvlQrqucbaxtnXzOqnxjgHzpddwugGxrdobrkucshcNmzrOrvoEDngwdedjc"));
                Debug.Assert(pack.resolution_v == (ushort)(ushort)19089);
                Debug.Assert(pack.rotation == (ushort)(ushort)61096);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)55300);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.target_system = (byte)(byte)15;
            p270.resolution_h = (ushort)(ushort)55300;
            p270.rotation = (ushort)(ushort)61096;
            p270.camera_id = (byte)(byte)148;
            p270.framerate = (float) -2.5912656E38F;
            p270.uri_SET("zlrpggcokegltpqghhWkmufVncoxymziumZoxvhpkgyxodeyPGHnaktzlfpecbwpHnccxmffddttvvfTaeneoonabwsoybdagdCuSbchxvulCqbsjabgpcpcimnboluzxJyxrnbsnvlQrqucbaxtnXzOqnxjgHzpddwugGxrdobrkucshcNmzrOrvoEDngwdedjc", PH) ;
            p270.target_component = (byte)(byte)65;
            p270.resolution_v = (ushort)(ushort)19089;
            p270.bitrate = (uint)708530833U;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 13);
                Debug.Assert(pack.password_TRY(ph).Equals("bswvsfgbiavzu"));
                Debug.Assert(pack.ssid_LEN(ph) == 2);
                Debug.Assert(pack.ssid_TRY(ph).Equals("tt"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.password_SET("bswvsfgbiavzu", PH) ;
            p299.ssid_SET("tt", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)126, (byte)37, (byte)124, (byte)178, (byte)95, (byte)74, (byte)197, (byte)225}));
                Debug.Assert(pack.version == (ushort)(ushort)33636);
                Debug.Assert(pack.min_version == (ushort)(ushort)60080);
                Debug.Assert(pack.max_version == (ushort)(ushort)25496);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)169, (byte)196, (byte)160, (byte)181, (byte)73, (byte)180, (byte)163, (byte)12}));
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.spec_version_hash_SET(new byte[] {(byte)169, (byte)196, (byte)160, (byte)181, (byte)73, (byte)180, (byte)163, (byte)12}, 0) ;
            p300.library_version_hash_SET(new byte[] {(byte)126, (byte)37, (byte)124, (byte)178, (byte)95, (byte)74, (byte)197, (byte)225}, 0) ;
            p300.min_version = (ushort)(ushort)60080;
            p300.max_version = (ushort)(ushort)25496;
            p300.version = (ushort)(ushort)33636;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sub_mode == (byte)(byte)138);
                Debug.Assert(pack.health == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
                Debug.Assert(pack.time_usec == (ulong)7661220393737620348L);
                Debug.Assert(pack.mode == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)19291);
                Debug.Assert(pack.uptime_sec == (uint)3489138601U);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE;
            p310.time_usec = (ulong)7661220393737620348L;
            p310.uptime_sec = (uint)3489138601U;
            p310.sub_mode = (byte)(byte)138;
            p310.vendor_specific_status_code = (ushort)(ushort)19291;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 70);
                Debug.Assert(pack.name_TRY(ph).Equals("AxmibAOcaogOuvmngkryihZoxmpbnmfpfiyniGytfsfytmezcfdqbnvXettUwEralsvsyY"));
                Debug.Assert(pack.hw_version_minor == (byte)(byte)207);
                Debug.Assert(pack.sw_version_major == (byte)(byte)75);
                Debug.Assert(pack.sw_vcs_commit == (uint)101821352U);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)175, (byte)194, (byte)178, (byte)128, (byte)32, (byte)54, (byte)114, (byte)153, (byte)229, (byte)139, (byte)71, (byte)149, (byte)142, (byte)145, (byte)57, (byte)44}));
                Debug.Assert(pack.sw_version_minor == (byte)(byte)178);
                Debug.Assert(pack.hw_version_major == (byte)(byte)177);
                Debug.Assert(pack.uptime_sec == (uint)1128386143U);
                Debug.Assert(pack.time_usec == (ulong)3395546666856716618L);
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.uptime_sec = (uint)1128386143U;
            p311.sw_vcs_commit = (uint)101821352U;
            p311.sw_version_major = (byte)(byte)75;
            p311.hw_version_minor = (byte)(byte)207;
            p311.hw_unique_id_SET(new byte[] {(byte)175, (byte)194, (byte)178, (byte)128, (byte)32, (byte)54, (byte)114, (byte)153, (byte)229, (byte)139, (byte)71, (byte)149, (byte)142, (byte)145, (byte)57, (byte)44}, 0) ;
            p311.hw_version_major = (byte)(byte)177;
            p311.sw_version_minor = (byte)(byte)178;
            p311.time_usec = (ulong)3395546666856716618L;
            p311.name_SET("AxmibAOcaogOuvmngkryihZoxmpbnmfpfiyniGytfsfytmezcfdqbnvXettUwEralsvsyY", PH) ;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("sidHywru"));
                Debug.Assert(pack.target_component == (byte)(byte)172);
                Debug.Assert(pack.target_system == (byte)(byte)46);
                Debug.Assert(pack.param_index == (short)(short)23583);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)46;
            p320.param_index = (short)(short)23583;
            p320.param_id_SET("sidHywru", PH) ;
            p320.target_component = (byte)(byte)172;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)84);
                Debug.Assert(pack.target_system == (byte)(byte)138);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)138;
            p321.target_component = (byte)(byte)84;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_count == (ushort)(ushort)55820);
                Debug.Assert(pack.param_id_LEN(ph) == 10);
                Debug.Assert(pack.param_id_TRY(ph).Equals("xqcrwrwrah"));
                Debug.Assert(pack.param_value_LEN(ph) == 57);
                Debug.Assert(pack.param_value_TRY(ph).Equals("najqhbaqptaDouwsbcsZgtumgHrlzfoWetemgKyhmuszbxJhvjknwtdys"));
                Debug.Assert(pack.param_index == (ushort)(ushort)5593);
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_value_SET("najqhbaqptaDouwsbcsZgtumgHrlzfoWetemgKyhmuszbxJhvjknwtdys", PH) ;
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            p322.param_index = (ushort)(ushort)5593;
            p322.param_count = (ushort)(ushort)55820;
            p322.param_id_SET("xqcrwrwrah", PH) ;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
                Debug.Assert(pack.target_component == (byte)(byte)20);
                Debug.Assert(pack.param_value_LEN(ph) == 5);
                Debug.Assert(pack.param_value_TRY(ph).Equals("qYAdm"));
                Debug.Assert(pack.target_system == (byte)(byte)224);
                Debug.Assert(pack.param_id_LEN(ph) == 15);
                Debug.Assert(pack.param_id_TRY(ph).Equals("wdsbtpvcNyMIBxs"));
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)224;
            p323.target_component = (byte)(byte)20;
            p323.param_value_SET("qYAdm", PH) ;
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            p323.param_id_SET("wdsbtpvcNyMIBxs", PH) ;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
                Debug.Assert(pack.param_value_LEN(ph) == 50);
                Debug.Assert(pack.param_value_TRY(ph).Equals("atrennjbnfjtydujmsdzcmmfcqcebzlyyuamkNkqgYtiujgctY"));
                Debug.Assert(pack.param_result == PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
                Debug.Assert(pack.param_id_LEN(ph) == 2);
                Debug.Assert(pack.param_id_TRY(ph).Equals("in"));
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("in", PH) ;
            p324.param_result = PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED;
            p324.param_value_SET("atrennjbnfjtydujmsdzcmmfcqcebzlyyuamkNkqgYtiujgctY", PH) ;
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_distance == (ushort)(ushort)41839);
                Debug.Assert(pack.time_usec == (ulong)6025542767379955875L);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)6644, (ushort)65333, (ushort)44340, (ushort)26439, (ushort)37521, (ushort)34513, (ushort)20554, (ushort)52026, (ushort)21038, (ushort)465, (ushort)51974, (ushort)48, (ushort)33194, (ushort)47892, (ushort)9134, (ushort)53877, (ushort)34025, (ushort)1238, (ushort)59651, (ushort)29083, (ushort)4346, (ushort)16107, (ushort)32611, (ushort)13377, (ushort)35528, (ushort)17317, (ushort)52980, (ushort)53506, (ushort)40700, (ushort)36378, (ushort)23230, (ushort)29105, (ushort)59576, (ushort)40371, (ushort)50156, (ushort)25701, (ushort)42084, (ushort)60071, (ushort)43153, (ushort)48193, (ushort)26448, (ushort)52231, (ushort)44248, (ushort)50565, (ushort)23430, (ushort)57028, (ushort)32651, (ushort)45531, (ushort)30723, (ushort)41661, (ushort)57071, (ushort)313, (ushort)52089, (ushort)20730, (ushort)59543, (ushort)51644, (ushort)40274, (ushort)15322, (ushort)43565, (ushort)35642, (ushort)33939, (ushort)21858, (ushort)2157, (ushort)53951, (ushort)58566, (ushort)37069, (ushort)42181, (ushort)57059, (ushort)781, (ushort)48263, (ushort)59789, (ushort)652}));
                Debug.Assert(pack.max_distance == (ushort)(ushort)1589);
                Debug.Assert(pack.increment == (byte)(byte)96);
                Debug.Assert(pack.sensor_type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.max_distance = (ushort)(ushort)1589;
            p330.distances_SET(new ushort[] {(ushort)6644, (ushort)65333, (ushort)44340, (ushort)26439, (ushort)37521, (ushort)34513, (ushort)20554, (ushort)52026, (ushort)21038, (ushort)465, (ushort)51974, (ushort)48, (ushort)33194, (ushort)47892, (ushort)9134, (ushort)53877, (ushort)34025, (ushort)1238, (ushort)59651, (ushort)29083, (ushort)4346, (ushort)16107, (ushort)32611, (ushort)13377, (ushort)35528, (ushort)17317, (ushort)52980, (ushort)53506, (ushort)40700, (ushort)36378, (ushort)23230, (ushort)29105, (ushort)59576, (ushort)40371, (ushort)50156, (ushort)25701, (ushort)42084, (ushort)60071, (ushort)43153, (ushort)48193, (ushort)26448, (ushort)52231, (ushort)44248, (ushort)50565, (ushort)23430, (ushort)57028, (ushort)32651, (ushort)45531, (ushort)30723, (ushort)41661, (ushort)57071, (ushort)313, (ushort)52089, (ushort)20730, (ushort)59543, (ushort)51644, (ushort)40274, (ushort)15322, (ushort)43565, (ushort)35642, (ushort)33939, (ushort)21858, (ushort)2157, (ushort)53951, (ushort)58566, (ushort)37069, (ushort)42181, (ushort)57059, (ushort)781, (ushort)48263, (ushort)59789, (ushort)652}, 0) ;
            p330.min_distance = (ushort)(ushort)41839;
            p330.increment = (byte)(byte)96;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN;
            p330.time_usec = (ulong)6025542767379955875L;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
        }
    }
}