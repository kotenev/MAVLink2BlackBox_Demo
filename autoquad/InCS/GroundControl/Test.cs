
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
                    ulong id = id__o(value);
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
                        case MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION:
                            id = 3;
                            break;
                        case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                            id = 4;
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
                        case MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION:
                            id = 3;
                            break;
                        case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                            id = 4;
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
                    ulong id = id__W(value);
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
                        case MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION:
                            id = 3;
                            break;
                        case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                            id = 4;
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
                        case MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION:
                            id = 3;
                            break;
                        case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                            id = 4;
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
                        case MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION:
                            id = 3;
                            break;
                        case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                            id = 4;
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
                        case MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION:
                            id = 3;
                            break;
                        case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                            id = 4;
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
                        case MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION:
                            id = 3;
                            break;
                        case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                            id = 4;
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
                        case MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION:
                            id = 3;
                            break;
                        case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                            id = 4;
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
                        case MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION:
                            id = 3;
                            break;
                        case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                            id = 4;
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
                    ulong id = id__W(value);
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
                        case MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION:
                            id = 3;
                            break;
                        case MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL:
                            id = 4;
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
        new class ACTUATOR_CONTROL_TARGET : GroundControl.ACTUATOR_CONTROL_TARGET
        {
            public ulong time_usec //Timestamp (micros since boot or Unix epoch)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            /**
            *Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use
            *	 this field to difference between instances*/
            public byte group_mlx
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  8, 1));}
            }

            /**
            *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
            *	 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
            *	 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
            *	 mixer to repurpose them as generic outputs*/
            public float[] controls
            {
                get {return controls_GET(new float[8], 0);}
            }
            /**
            *Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction
            *	 motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0):
            *	 (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through
            *	 mixer to repurpose them as generic outputs*/
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
            *	 local altitude change). The only guarantee on this field is that it will never be reset and is consistent
            *	 within a flight. The recommended value for this field is the uncorrected barometric altitude at boot
            *	 time. This altitude will also drift and vary between flights*/
            public float altitude_monotonic
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            /**
            *This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events
            *	 like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints
            *	 are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL
            *	 by default and not the WGS84 altitude*/
            public float altitude_amsl
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            /**
            *This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference
            *	 to the coordinate origin (0, 0, 0). It is up-positive*/
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
            *	 than -1000 should be interpreted as unknown*/
            public float altitude_terrain
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            /**
            *This is not the altitude, but the clear space below the system according to the fused clearance estimate.
            *	 It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving
            *	 target. A negative value indicates no measurement available*/
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
        new class AQ_TELEMETRY_F : GroundControl.AQ_TELEMETRY_F
        {
            public ushort Index //Index of message
            {
                get {  return (ushort)((ushort) BitUtils.get_bytes(data,  0, 2));}
            }

            public float value1 //value1
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  2, 4)));}
            }

            public float value2 //value2
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  6, 4)));}
            }

            public float value3 //value3
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  10, 4)));}
            }

            public float value4 //value4
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  14, 4)));}
            }

            public float value5 //value5
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  18, 4)));}
            }

            public float value6 //value6
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  22, 4)));}
            }

            public float value7 //value7
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  26, 4)));}
            }

            public float value8 //value8
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  30, 4)));}
            }

            public float value9 //value9
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  34, 4)));}
            }

            public float value10 //value10
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  38, 4)));}
            }

            public float value11 //value11
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  42, 4)));}
            }

            public float value12 //value12
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  46, 4)));}
            }

            public float value13 //value13
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  50, 4)));}
            }

            public float value14 //value14
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  54, 4)));}
            }

            public float value15 //value15
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  58, 4)));}
            }

            public float value16 //value16
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  62, 4)));}
            }

            public float value17 //value17
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  66, 4)));}
            }

            public float value18 //value18
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  70, 4)));}
            }

            public float value19 //value19
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  74, 4)));}
            }

            public float value20 //value20
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  78, 4)));}
            }
        }
        new class AQ_ESC_TELEMETRY : GroundControl.AQ_ESC_TELEMETRY
        {
            public ushort[] status_age //Age of each ESC telemetry reading in ms compared to boot time. A value of 0xFFFF means timeout/no data
            {
                get {return status_age_GET(new ushort[4], 0);}
            }
            public ushort[]status_age_GET(ushort[] dst_ch, int pos)  //Age of each ESC telemetry reading in ms compared to boot time. A value of 0xFFFF means timeout/no data
            {
                for(int BYTE = 0, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public uint time_boot_ms //Timestamp of the component clock since boot time in ms.
            {
                get {  return (uint)((uint) BitUtils.get_bytes(data,  8, 4));}
            }

            public uint[] data0 //Data bits 1-32 for each ESC.
            {
                get {return data0_GET(new uint[4], 0);}
            }
            public uint[]data0_GET(uint[] dst_ch, int pos)  //Data bits 1-32 for each ESC.
            {
                for(int BYTE = 12, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (uint)((uint) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }
            public uint[] data1 //Data bits 33-64 for each ESC.
            {
                get {return data1_GET(new uint[4], 0);}
            }
            public uint[]data1_GET(uint[] dst_ch, int pos)  //Data bits 33-64 for each ESC.
            {
                for(int BYTE = 28, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 4)
                    dst_ch[pos] = (uint)((uint) BitUtils.get_bytes(data,  BYTE, 4));
                return dst_ch;
            }
            public byte seq //Sequence number of message (first set of 4 motors is #1, next 4 is #2, etc).
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  44, 1));}
            }

            public byte num_motors //Total number of active ESCs/motors on the system.
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  45, 1));}
            }

            public byte num_in_seq //Number of active ESCs in this sequence (1 through this many array members will be populated with data
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  46, 1));}
            }

            public byte[] escid //ESC/Motor ID
            {
                get {return escid_GET(new byte[4], 0);}
            }
            public byte[]escid_GET(byte[] dst_ch, int pos)  //ESC/Motor ID
            {
                for(int BYTE = 47, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
                return dst_ch;
            }
            public byte[] data_version //Version of data structure (determines contents).
            {
                get {return data_version_GET(new byte[4], 0);}
            }
            public byte[]data_version_GET(byte[] dst_ch, int pos)  //Version of data structure (determines contents).
            {
                for(int BYTE = 51, dst_max = pos + 4; pos < dst_max ; pos++,  BYTE += 1)
                    dst_ch[pos] = (byte)((byte) BitUtils.get_bytes(data,  BYTE, 1));
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
                get {  return (MAV_COLLISION_THREAT_LEVEL)(0 +  BitUtils.get_bits(data, 132, 2));}
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
            public void OnAQ_TELEMETRY_FReceive_direct(Channel src, Inside ph, AQ_TELEMETRY_F pack) {OnAQ_TELEMETRY_FReceive(this, ph,  pack);}
            public event AQ_TELEMETRY_FReceiveHandler OnAQ_TELEMETRY_FReceive;
            public delegate void AQ_TELEMETRY_FReceiveHandler(Channel src, Inside ph, AQ_TELEMETRY_F pack);
            public void OnAQ_ESC_TELEMETRYReceive_direct(Channel src, Inside ph, AQ_ESC_TELEMETRY pack) {OnAQ_ESC_TELEMETRYReceive(this, ph,  pack);}
            public event AQ_ESC_TELEMETRYReceiveHandler OnAQ_ESC_TELEMETRYReceive;
            public delegate void AQ_ESC_TELEMETRYReceiveHandler(Channel src, Inside ph, AQ_ESC_TELEMETRY pack);
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
                    case 150:
                        if(pack == null) return new AQ_TELEMETRY_F();
                        OnAQ_TELEMETRY_FReceive(this, ph, (AQ_TELEMETRY_F) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 152:
                        if(pack == null) return new AQ_ESC_TELEMETRY();
                        OnAQ_ESC_TELEMETRYReceive(this, ph, (AQ_ESC_TELEMETRY) pack);//no any host channels can receive this pack. Handle it with test channel handler
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
                Debug.Assert(pack.type == MAV_TYPE.MAV_TYPE_GCS);
                Debug.Assert(pack.system_status == MAV_STATE.MAV_STATE_ACTIVE);
                Debug.Assert(pack.custom_mode == (uint)1847799231U);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED));
                Debug.Assert(pack.autopilot == MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC);
                Debug.Assert(pack.mavlink_version == (byte)(byte)169);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED);
            p0.system_status = MAV_STATE.MAV_STATE_ACTIVE;
            p0.mavlink_version = (byte)(byte)169;
            p0.type = MAV_TYPE.MAV_TYPE_GCS;
            p0.custom_mode = (uint)1847799231U;
            p0.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC;
            SMP_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.load == (ushort)(ushort)32749);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)4196);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)23535);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)45527);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)73);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)17252);
                Debug.Assert(pack.current_battery == (short)(short) -9788);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS));
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)53174);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)30466);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)59940);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY));
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.errors_count2 = (ushort)(ushort)30466;
            p1.current_battery = (short)(short) -9788;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.errors_comm = (ushort)(ushort)45527;
            p1.voltage_battery = (ushort)(ushort)4196;
            p1.drop_rate_comm = (ushort)(ushort)53174;
            p1.errors_count3 = (ushort)(ushort)59940;
            p1.errors_count4 = (ushort)(ushort)23535;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY);
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS);
            p1.battery_remaining = (sbyte)(sbyte)73;
            p1.load = (ushort)(ushort)32749;
            p1.errors_count1 = (ushort)(ushort)17252;
            SMP_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)7688970919757714244L);
                Debug.Assert(pack.time_boot_ms == (uint)1779990308U);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_unix_usec = (ulong)7688970919757714244L;
            p2.time_boot_ms = (uint)1779990308U;
            SMP_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float) -4.307473E37F);
                Debug.Assert(pack.vx == (float) -1.1039651E38F);
                Debug.Assert(pack.afy == (float) -1.1870001E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.afx == (float) -2.3842354E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1882989158U);
                Debug.Assert(pack.z == (float) -1.7728643E38F);
                Debug.Assert(pack.x == (float)2.8262114E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)60124);
                Debug.Assert(pack.y == (float) -2.5171691E38F);
                Debug.Assert(pack.afz == (float)1.0787113E38F);
                Debug.Assert(pack.vz == (float) -5.865374E37F);
                Debug.Assert(pack.yaw_rate == (float)7.6820473E37F);
                Debug.Assert(pack.yaw == (float)1.6757283E38F);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.vx = (float) -1.1039651E38F;
            p3.yaw_rate = (float)7.6820473E37F;
            p3.time_boot_ms = (uint)1882989158U;
            p3.type_mask = (ushort)(ushort)60124;
            p3.y = (float) -2.5171691E38F;
            p3.vz = (float) -5.865374E37F;
            p3.z = (float) -1.7728643E38F;
            p3.afy = (float) -1.1870001E38F;
            p3.yaw = (float)1.6757283E38F;
            p3.afx = (float) -2.3842354E38F;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p3.vy = (float) -4.307473E37F;
            p3.afz = (float)1.0787113E38F;
            p3.x = (float)2.8262114E38F;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)2146506481U);
                Debug.Assert(pack.target_component == (byte)(byte)37);
                Debug.Assert(pack.target_system == (byte)(byte)228);
                Debug.Assert(pack.time_usec == (ulong)138637330428108971L);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.seq = (uint)2146506481U;
            p4.target_component = (byte)(byte)37;
            p4.time_usec = (ulong)138637330428108971L;
            p4.target_system = (byte)(byte)228;
            SMP_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)206);
                Debug.Assert(pack.control_request == (byte)(byte)105);
                Debug.Assert(pack.version == (byte)(byte)118);
                Debug.Assert(pack.passkey_LEN(ph) == 13);
                Debug.Assert(pack.passkey_TRY(ph).Equals("ChlvzsuCcicqx"));
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.passkey_SET("ChlvzsuCcicqx", PH) ;
            p5.control_request = (byte)(byte)105;
            p5.target_system = (byte)(byte)206;
            p5.version = (byte)(byte)118;
            SMP_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ack == (byte)(byte)224);
                Debug.Assert(pack.control_request == (byte)(byte)186);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)167);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.ack = (byte)(byte)224;
            p6.gcs_system_id = (byte)(byte)167;
            p6.control_request = (byte)(byte)186;
            SMP_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 8);
                Debug.Assert(pack.key_TRY(ph).Equals("CvxhpKzu"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("CvxhpKzu", PH) ;
            SMP_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.custom_mode == (uint)699151866U);
                Debug.Assert(pack.base_mode == MAV_MODE.MAV_MODE_AUTO_ARMED);
                Debug.Assert(pack.target_system == (byte)(byte)137);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.custom_mode = (uint)699151866U;
            p11.base_mode = MAV_MODE.MAV_MODE_AUTO_ARMED;
            p11.target_system = (byte)(byte)137;
            SMP_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("Dha"));
                Debug.Assert(pack.target_system == (byte)(byte)92);
                Debug.Assert(pack.param_index == (short)(short) -28702);
                Debug.Assert(pack.target_component == (byte)(byte)228);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_system = (byte)(byte)92;
            p20.param_index = (short)(short) -28702;
            p20.target_component = (byte)(byte)228;
            p20.param_id_SET("Dha", PH) ;
            SMP_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)33);
                Debug.Assert(pack.target_system == (byte)(byte)60);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)60;
            p21.target_component = (byte)(byte)33;
            SMP_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_count == (ushort)(ushort)25894);
                Debug.Assert(pack.param_value == (float) -2.9756838E38F);
                Debug.Assert(pack.param_index == (ushort)(ushort)33622);
                Debug.Assert(pack.param_id_LEN(ph) == 12);
                Debug.Assert(pack.param_id_TRY(ph).Equals("ZarbkyJyejhy"));
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_value = (float) -2.9756838E38F;
            p22.param_id_SET("ZarbkyJyejhy", PH) ;
            p22.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16;
            p22.param_index = (ushort)(ushort)33622;
            p22.param_count = (ushort)(ushort)25894;
            SMP_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)11);
                Debug.Assert(pack.param_value == (float) -2.6465567E38F);
                Debug.Assert(pack.target_system == (byte)(byte)139);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64);
                Debug.Assert(pack.param_id_LEN(ph) == 16);
                Debug.Assert(pack.param_id_TRY(ph).Equals("hJwbitUmnqJocqxe"));
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.param_id_SET("hJwbitUmnqJocqxe", PH) ;
            p23.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64;
            p23.target_component = (byte)(byte)11;
            p23.target_system = (byte)(byte)139;
            p23.param_value = (float) -2.6465567E38F;
            SMP_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cog == (ushort)(ushort)12425);
                Debug.Assert(pack.vel == (ushort)(ushort)47250);
                Debug.Assert(pack.epv == (ushort)(ushort)23457);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)3155576065U);
                Debug.Assert(pack.satellites_visible == (byte)(byte)135);
                Debug.Assert(pack.alt == (int) -799012464);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)1485127827U);
                Debug.Assert(pack.lat == (int)841759394);
                Debug.Assert(pack.lon == (int)2014607631);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -371919190);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)609362061U);
                Debug.Assert(pack.time_usec == (ulong)5835349051032461795L);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
                Debug.Assert(pack.eph == (ushort)(ushort)15584);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)2139737612U);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.vel_acc_SET((uint)3155576065U, PH) ;
            p24.eph = (ushort)(ushort)15584;
            p24.vel = (ushort)(ushort)47250;
            p24.h_acc_SET((uint)2139737612U, PH) ;
            p24.epv = (ushort)(ushort)23457;
            p24.v_acc_SET((uint)609362061U, PH) ;
            p24.alt = (int) -799012464;
            p24.lon = (int)2014607631;
            p24.alt_ellipsoid_SET((int) -371919190, PH) ;
            p24.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS;
            p24.satellites_visible = (byte)(byte)135;
            p24.cog = (ushort)(ushort)12425;
            p24.lat = (int)841759394;
            p24.time_usec = (ulong)5835349051032461795L;
            p24.hdg_acc_SET((uint)1485127827U, PH) ;
            SMP_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)43, (byte)81, (byte)190, (byte)228, (byte)126, (byte)46, (byte)183, (byte)109, (byte)247, (byte)36, (byte)142, (byte)203, (byte)23, (byte)54, (byte)68, (byte)11, (byte)148, (byte)68, (byte)39, (byte)237}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)189, (byte)131, (byte)188, (byte)12, (byte)61, (byte)145, (byte)243, (byte)230, (byte)68, (byte)154, (byte)200, (byte)123, (byte)111, (byte)20, (byte)143, (byte)26, (byte)147, (byte)118, (byte)23, (byte)147}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)85, (byte)252, (byte)70, (byte)51, (byte)243, (byte)172, (byte)66, (byte)5, (byte)115, (byte)154, (byte)76, (byte)73, (byte)53, (byte)219, (byte)217, (byte)200, (byte)151, (byte)139, (byte)63, (byte)176}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)166);
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)111, (byte)88, (byte)208, (byte)217, (byte)60, (byte)140, (byte)48, (byte)88, (byte)21, (byte)104, (byte)64, (byte)26, (byte)203, (byte)106, (byte)201, (byte)23, (byte)115, (byte)251, (byte)235, (byte)246}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)40, (byte)29, (byte)220, (byte)104, (byte)113, (byte)179, (byte)185, (byte)57, (byte)82, (byte)50, (byte)169, (byte)188, (byte)63, (byte)214, (byte)75, (byte)14, (byte)190, (byte)102, (byte)55, (byte)32}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_prn_SET(new byte[] {(byte)40, (byte)29, (byte)220, (byte)104, (byte)113, (byte)179, (byte)185, (byte)57, (byte)82, (byte)50, (byte)169, (byte)188, (byte)63, (byte)214, (byte)75, (byte)14, (byte)190, (byte)102, (byte)55, (byte)32}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)111, (byte)88, (byte)208, (byte)217, (byte)60, (byte)140, (byte)48, (byte)88, (byte)21, (byte)104, (byte)64, (byte)26, (byte)203, (byte)106, (byte)201, (byte)23, (byte)115, (byte)251, (byte)235, (byte)246}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)85, (byte)252, (byte)70, (byte)51, (byte)243, (byte)172, (byte)66, (byte)5, (byte)115, (byte)154, (byte)76, (byte)73, (byte)53, (byte)219, (byte)217, (byte)200, (byte)151, (byte)139, (byte)63, (byte)176}, 0) ;
            p25.satellites_visible = (byte)(byte)166;
            p25.satellite_elevation_SET(new byte[] {(byte)43, (byte)81, (byte)190, (byte)228, (byte)126, (byte)46, (byte)183, (byte)109, (byte)247, (byte)36, (byte)142, (byte)203, (byte)23, (byte)54, (byte)68, (byte)11, (byte)148, (byte)68, (byte)39, (byte)237}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)189, (byte)131, (byte)188, (byte)12, (byte)61, (byte)145, (byte)243, (byte)230, (byte)68, (byte)154, (byte)200, (byte)123, (byte)111, (byte)20, (byte)143, (byte)26, (byte)147, (byte)118, (byte)23, (byte)147}, 0) ;
            SMP_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (short)(short) -7453);
                Debug.Assert(pack.zmag == (short)(short)3175);
                Debug.Assert(pack.xacc == (short)(short) -32669);
                Debug.Assert(pack.yacc == (short)(short) -10623);
                Debug.Assert(pack.zacc == (short)(short) -20458);
                Debug.Assert(pack.zgyro == (short)(short) -938);
                Debug.Assert(pack.xmag == (short)(short) -3045);
                Debug.Assert(pack.ymag == (short)(short) -17896);
                Debug.Assert(pack.ygyro == (short)(short)7044);
                Debug.Assert(pack.time_boot_ms == (uint)4150366037U);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.ymag = (short)(short) -17896;
            p26.ygyro = (short)(short)7044;
            p26.xacc = (short)(short) -32669;
            p26.xmag = (short)(short) -3045;
            p26.zacc = (short)(short) -20458;
            p26.time_boot_ms = (uint)4150366037U;
            p26.xgyro = (short)(short) -7453;
            p26.zmag = (short)(short)3175;
            p26.zgyro = (short)(short) -938;
            p26.yacc = (short)(short) -10623;
            SMP_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zmag == (short)(short)15126);
                Debug.Assert(pack.xacc == (short)(short)15381);
                Debug.Assert(pack.zgyro == (short)(short)19738);
                Debug.Assert(pack.time_usec == (ulong)7887420614252128106L);
                Debug.Assert(pack.ygyro == (short)(short) -18916);
                Debug.Assert(pack.yacc == (short)(short)21809);
                Debug.Assert(pack.ymag == (short)(short)5111);
                Debug.Assert(pack.xmag == (short)(short) -30802);
                Debug.Assert(pack.xgyro == (short)(short) -6838);
                Debug.Assert(pack.zacc == (short)(short) -11730);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.xgyro = (short)(short) -6838;
            p27.zmag = (short)(short)15126;
            p27.ygyro = (short)(short) -18916;
            p27.time_usec = (ulong)7887420614252128106L;
            p27.xmag = (short)(short) -30802;
            p27.xacc = (short)(short)15381;
            p27.zgyro = (short)(short)19738;
            p27.yacc = (short)(short)21809;
            p27.zacc = (short)(short) -11730;
            p27.ymag = (short)(short)5111;
            SMP_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)29250);
                Debug.Assert(pack.press_abs == (short)(short)25385);
                Debug.Assert(pack.press_diff1 == (short)(short)15555);
                Debug.Assert(pack.time_usec == (ulong)7004541695902712514L);
                Debug.Assert(pack.press_diff2 == (short)(short)7605);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_diff1 = (short)(short)15555;
            p28.temperature = (short)(short)29250;
            p28.press_diff2 = (short)(short)7605;
            p28.time_usec = (ulong)7004541695902712514L;
            p28.press_abs = (short)(short)25385;
            SMP_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float) -2.5968678E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3891245505U);
                Debug.Assert(pack.press_diff == (float) -7.681622E37F);
                Debug.Assert(pack.temperature == (short)(short) -5465);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.press_diff = (float) -7.681622E37F;
            p29.press_abs = (float) -2.5968678E38F;
            p29.time_boot_ms = (uint)3891245505U;
            p29.temperature = (short)(short) -5465;
            SMP_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitchspeed == (float) -1.6487001E38F);
                Debug.Assert(pack.rollspeed == (float) -2.5450925E38F);
                Debug.Assert(pack.yaw == (float)9.394537E37F);
                Debug.Assert(pack.roll == (float) -2.806125E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1671688958U);
                Debug.Assert(pack.pitch == (float) -3.6581037E37F);
                Debug.Assert(pack.yawspeed == (float) -8.210443E37F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.time_boot_ms = (uint)1671688958U;
            p30.roll = (float) -2.806125E38F;
            p30.yaw = (float)9.394537E37F;
            p30.pitch = (float) -3.6581037E37F;
            p30.yawspeed = (float) -8.210443E37F;
            p30.pitchspeed = (float) -1.6487001E38F;
            p30.rollspeed = (float) -2.5450925E38F;
            SMP_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q3 == (float)2.5672736E38F);
                Debug.Assert(pack.q4 == (float)4.8840767E37F);
                Debug.Assert(pack.q2 == (float)1.7781114E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3007535153U);
                Debug.Assert(pack.rollspeed == (float)1.1216716E38F);
                Debug.Assert(pack.pitchspeed == (float) -2.9736022E38F);
                Debug.Assert(pack.q1 == (float)9.550223E37F);
                Debug.Assert(pack.yawspeed == (float)8.221664E37F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.time_boot_ms = (uint)3007535153U;
            p31.yawspeed = (float)8.221664E37F;
            p31.rollspeed = (float)1.1216716E38F;
            p31.pitchspeed = (float) -2.9736022E38F;
            p31.q3 = (float)2.5672736E38F;
            p31.q4 = (float)4.8840767E37F;
            p31.q2 = (float)1.7781114E38F;
            p31.q1 = (float)9.550223E37F;
            SMP_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -3.046951E38F);
                Debug.Assert(pack.z == (float) -2.2629259E38F);
                Debug.Assert(pack.vx == (float) -1.189847E38F);
                Debug.Assert(pack.vz == (float) -1.4219553E38F);
                Debug.Assert(pack.vy == (float) -2.9827606E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4111276807U);
                Debug.Assert(pack.x == (float) -9.339288E36F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.vx = (float) -1.189847E38F;
            p32.time_boot_ms = (uint)4111276807U;
            p32.vz = (float) -1.4219553E38F;
            p32.vy = (float) -2.9827606E38F;
            p32.z = (float) -2.2629259E38F;
            p32.y = (float) -3.046951E38F;
            p32.x = (float) -9.339288E36F;
            SMP_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (short)(short)17778);
                Debug.Assert(pack.vx == (short)(short)734);
                Debug.Assert(pack.lon == (int) -1791525166);
                Debug.Assert(pack.relative_alt == (int) -401570133);
                Debug.Assert(pack.alt == (int)961487766);
                Debug.Assert(pack.time_boot_ms == (uint)1370342459U);
                Debug.Assert(pack.lat == (int) -453943691);
                Debug.Assert(pack.vz == (short)(short)1861);
                Debug.Assert(pack.hdg == (ushort)(ushort)16537);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.hdg = (ushort)(ushort)16537;
            p33.vz = (short)(short)1861;
            p33.vy = (short)(short)17778;
            p33.time_boot_ms = (uint)1370342459U;
            p33.alt = (int)961487766;
            p33.lat = (int) -453943691;
            p33.vx = (short)(short)734;
            p33.relative_alt = (int) -401570133;
            p33.lon = (int) -1791525166;
            SMP_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan4_scaled == (short)(short)24111);
                Debug.Assert(pack.chan3_scaled == (short)(short)12240);
                Debug.Assert(pack.chan6_scaled == (short)(short) -18502);
                Debug.Assert(pack.chan1_scaled == (short)(short) -6168);
                Debug.Assert(pack.chan7_scaled == (short)(short) -7353);
                Debug.Assert(pack.port == (byte)(byte)226);
                Debug.Assert(pack.rssi == (byte)(byte)165);
                Debug.Assert(pack.chan2_scaled == (short)(short) -29940);
                Debug.Assert(pack.chan5_scaled == (short)(short) -24450);
                Debug.Assert(pack.time_boot_ms == (uint)4143567504U);
                Debug.Assert(pack.chan8_scaled == (short)(short) -7394);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan8_scaled = (short)(short) -7394;
            p34.time_boot_ms = (uint)4143567504U;
            p34.chan2_scaled = (short)(short) -29940;
            p34.rssi = (byte)(byte)165;
            p34.chan7_scaled = (short)(short) -7353;
            p34.chan6_scaled = (short)(short) -18502;
            p34.chan3_scaled = (short)(short)12240;
            p34.port = (byte)(byte)226;
            p34.chan5_scaled = (short)(short) -24450;
            p34.chan1_scaled = (short)(short) -6168;
            p34.chan4_scaled = (short)(short)24111;
            SMP_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)16671);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)16029);
                Debug.Assert(pack.rssi == (byte)(byte)160);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)38319);
                Debug.Assert(pack.time_boot_ms == (uint)3288188878U);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)17890);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)52217);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)8634);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)43399);
                Debug.Assert(pack.port == (byte)(byte)131);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)14181);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan5_raw = (ushort)(ushort)16029;
            p35.chan1_raw = (ushort)(ushort)8634;
            p35.port = (byte)(byte)131;
            p35.chan6_raw = (ushort)(ushort)14181;
            p35.rssi = (byte)(byte)160;
            p35.chan2_raw = (ushort)(ushort)38319;
            p35.chan7_raw = (ushort)(ushort)17890;
            p35.chan4_raw = (ushort)(ushort)16671;
            p35.time_boot_ms = (uint)3288188878U;
            p35.chan3_raw = (ushort)(ushort)43399;
            p35.chan8_raw = (ushort)(ushort)52217;
            SMP_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)24848);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)37915);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)53716);
                Debug.Assert(pack.time_usec == (uint)2339836611U);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)3187);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)27835);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)32662);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)34974);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)21574);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)51125);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)64239);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)2514);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)42358);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)2166);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)40652);
                Debug.Assert(pack.port == (byte)(byte)57);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)58244);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)4619);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo4_raw = (ushort)(ushort)51125;
            p36.servo10_raw_SET((ushort)(ushort)24848, PH) ;
            p36.servo15_raw_SET((ushort)(ushort)42358, PH) ;
            p36.servo2_raw = (ushort)(ushort)27835;
            p36.time_usec = (uint)2339836611U;
            p36.servo11_raw_SET((ushort)(ushort)64239, PH) ;
            p36.servo13_raw_SET((ushort)(ushort)53716, PH) ;
            p36.servo14_raw_SET((ushort)(ushort)40652, PH) ;
            p36.servo12_raw_SET((ushort)(ushort)37915, PH) ;
            p36.servo5_raw = (ushort)(ushort)34974;
            p36.servo1_raw = (ushort)(ushort)32662;
            p36.servo9_raw_SET((ushort)(ushort)58244, PH) ;
            p36.servo16_raw_SET((ushort)(ushort)21574, PH) ;
            p36.servo7_raw = (ushort)(ushort)3187;
            p36.servo8_raw = (ushort)(ushort)2166;
            p36.port = (byte)(byte)57;
            p36.servo3_raw = (ushort)(ushort)4619;
            p36.servo6_raw = (ushort)(ushort)2514;
            SMP_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)145);
                Debug.Assert(pack.target_component == (byte)(byte)116);
                Debug.Assert(pack.start_index == (short)(short) -14116);
                Debug.Assert(pack.end_index == (short)(short) -18972);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.start_index = (short)(short) -14116;
            p37.target_system = (byte)(byte)145;
            p37.mission_type = MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION;
            p37.end_index = (short)(short) -18972;
            p37.target_component = (byte)(byte)116;
            SMP_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)120);
                Debug.Assert(pack.end_index == (short)(short)21747);
                Debug.Assert(pack.target_component == (byte)(byte)126);
                Debug.Assert(pack.start_index == (short)(short)325);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.start_index = (short)(short)325;
            p38.target_component = (byte)(byte)126;
            p38.end_index = (short)(short)21747;
            p38.target_system = (byte)(byte)120;
            p38.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            SMP_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN);
                Debug.Assert(pack.param4 == (float)1.0816037E37F);
                Debug.Assert(pack.z == (float)8.655052E37F);
                Debug.Assert(pack.current == (byte)(byte)130);
                Debug.Assert(pack.autocontinue == (byte)(byte)151);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.param2 == (float) -2.2202797E38F);
                Debug.Assert(pack.target_system == (byte)(byte)88);
                Debug.Assert(pack.seq == (ushort)(ushort)9291);
                Debug.Assert(pack.y == (float) -1.4848886E38F);
                Debug.Assert(pack.target_component == (byte)(byte)96);
                Debug.Assert(pack.x == (float)2.5178003E37F);
                Debug.Assert(pack.param3 == (float)6.134125E37F);
                Debug.Assert(pack.param1 == (float) -1.5132185E38F);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p39.target_component = (byte)(byte)96;
            p39.frame = MAV_FRAME.MAV_FRAME_MISSION;
            p39.x = (float)2.5178003E37F;
            p39.autocontinue = (byte)(byte)151;
            p39.param3 = (float)6.134125E37F;
            p39.param1 = (float) -1.5132185E38F;
            p39.y = (float) -1.4848886E38F;
            p39.param4 = (float)1.0816037E37F;
            p39.seq = (ushort)(ushort)9291;
            p39.param2 = (float) -2.2202797E38F;
            p39.command = MAV_CMD.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
            p39.target_system = (byte)(byte)88;
            p39.z = (float)8.655052E37F;
            p39.current = (byte)(byte)130;
            SMP_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)164);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION);
                Debug.Assert(pack.seq == (ushort)(ushort)8488);
                Debug.Assert(pack.target_component == (byte)(byte)11);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_component = (byte)(byte)11;
            p40.target_system = (byte)(byte)164;
            p40.seq = (ushort)(ushort)8488;
            p40.mission_type = MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION;
            SMP_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)238);
                Debug.Assert(pack.seq == (ushort)(ushort)38638);
                Debug.Assert(pack.target_component == (byte)(byte)17);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_system = (byte)(byte)238;
            p41.seq = (ushort)(ushort)38638;
            p41.target_component = (byte)(byte)17;
            SMP_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)11326);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)11326;
            SMP_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)22);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION);
                Debug.Assert(pack.target_component == (byte)(byte)231);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION;
            p43.target_system = (byte)(byte)22;
            p43.target_component = (byte)(byte)231;
            SMP_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)152);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.count == (ushort)(ushort)2173);
                Debug.Assert(pack.target_system == (byte)(byte)21);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p44.count = (ushort)(ushort)2173;
            p44.target_system = (byte)(byte)21;
            p44.target_component = (byte)(byte)152;
            SMP_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_system == (byte)(byte)198);
                Debug.Assert(pack.target_component == (byte)(byte)90);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p45.target_component = (byte)(byte)90;
            p45.target_system = (byte)(byte)198;
            SMP_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)5113);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)5113;
            SMP_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)134);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION);
                Debug.Assert(pack.type == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM3);
                Debug.Assert(pack.target_system == (byte)(byte)253);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.target_component = (byte)(byte)134;
            p47.type = MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM3;
            p47.target_system = (byte)(byte)253;
            p47.mission_type = MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION;
            SMP_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)80);
                Debug.Assert(pack.latitude == (int)2081642558);
                Debug.Assert(pack.longitude == (int) -1057971016);
                Debug.Assert(pack.altitude == (int) -265150628);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)4865206115454282846L);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.altitude = (int) -265150628;
            p48.latitude = (int)2081642558;
            p48.target_system = (byte)(byte)80;
            p48.longitude = (int) -1057971016;
            p48.time_usec_SET((ulong)4865206115454282846L, PH) ;
            SMP_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)4913310248741098546L);
                Debug.Assert(pack.latitude == (int)230709425);
                Debug.Assert(pack.altitude == (int) -1464672125);
                Debug.Assert(pack.longitude == (int)757958454);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.altitude = (int) -1464672125;
            p49.longitude = (int)757958454;
            p49.latitude = (int)230709425;
            p49.time_usec_SET((ulong)4913310248741098546L, PH) ;
            SMP_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)218);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)202);
                Debug.Assert(pack.target_system == (byte)(byte)74);
                Debug.Assert(pack.param_index == (short)(short)32547);
                Debug.Assert(pack.param_id_LEN(ph) == 10);
                Debug.Assert(pack.param_id_TRY(ph).Equals("rqmNeqoatb"));
                Debug.Assert(pack.param_value_max == (float) -1.5376729E38F);
                Debug.Assert(pack.param_value0 == (float) -3.0898602E38F);
                Debug.Assert(pack.scale == (float) -1.848984E38F);
                Debug.Assert(pack.param_value_min == (float)1.4756247E38F);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.param_value_min = (float)1.4756247E38F;
            p50.scale = (float) -1.848984E38F;
            p50.param_index = (short)(short)32547;
            p50.param_value_max = (float) -1.5376729E38F;
            p50.param_id_SET("rqmNeqoatb", PH) ;
            p50.parameter_rc_channel_index = (byte)(byte)202;
            p50.param_value0 = (float) -3.0898602E38F;
            p50.target_system = (byte)(byte)74;
            p50.target_component = (byte)(byte)218;
            SMP_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION);
                Debug.Assert(pack.target_component == (byte)(byte)225);
                Debug.Assert(pack.seq == (ushort)(ushort)49537);
                Debug.Assert(pack.target_system == (byte)(byte)153);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.mission_type = MAV_MISSION_TYPE.MAV_DATA_STREAM_PROPULSION;
            p51.target_component = (byte)(byte)225;
            p51.target_system = (byte)(byte)153;
            p51.seq = (ushort)(ushort)49537;
            SMP_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1z == (float) -2.9015046E38F);
                Debug.Assert(pack.target_system == (byte)(byte)248);
                Debug.Assert(pack.p2x == (float)1.4295168E38F);
                Debug.Assert(pack.target_component == (byte)(byte)25);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.p2y == (float) -1.9508342E38F);
                Debug.Assert(pack.p1x == (float)3.1577728E38F);
                Debug.Assert(pack.p1y == (float) -1.3507461E38F);
                Debug.Assert(pack.p2z == (float)1.3725151E38F);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.target_system = (byte)(byte)248;
            p54.p2x = (float)1.4295168E38F;
            p54.p2z = (float)1.3725151E38F;
            p54.p1z = (float) -2.9015046E38F;
            p54.target_component = (byte)(byte)25;
            p54.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p54.p1y = (float) -1.3507461E38F;
            p54.p2y = (float) -1.9508342E38F;
            p54.p1x = (float)3.1577728E38F;
            SMP_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.p1x == (float) -1.7132207E38F);
                Debug.Assert(pack.p1y == (float)1.9565196E37F);
                Debug.Assert(pack.p2z == (float) -2.8887658E38F);
                Debug.Assert(pack.p1z == (float) -1.962732E38F);
                Debug.Assert(pack.p2x == (float) -1.1719446E36F);
                Debug.Assert(pack.p2y == (float)1.4713567E38F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p1x = (float) -1.7132207E38F;
            p55.p1z = (float) -1.962732E38F;
            p55.p1y = (float)1.9565196E37F;
            p55.frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p55.p2x = (float) -1.1719446E36F;
            p55.p2y = (float)1.4713567E38F;
            p55.p2z = (float) -2.8887658E38F;
            SMP_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {8.577083E37F, 1.1331293E38F, 2.9249873E38F, -6.5571545E37F, 1.5793603E38F, 1.1638836E38F, 1.7183622E38F, 1.047372E38F, 1.0768215E38F}));
                Debug.Assert(pack.rollspeed == (float)5.690904E37F);
                Debug.Assert(pack.pitchspeed == (float)2.306828E38F);
                Debug.Assert(pack.time_usec == (ulong)4144976017426352772L);
                Debug.Assert(pack.yawspeed == (float)9.357269E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.805823E38F, 2.0323112E38F, 1.0742052E38F, 2.1752408E38F}));
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.yawspeed = (float)9.357269E37F;
            p61.rollspeed = (float)5.690904E37F;
            p61.pitchspeed = (float)2.306828E38F;
            p61.time_usec = (ulong)4144976017426352772L;
            p61.covariance_SET(new float[] {8.577083E37F, 1.1331293E38F, 2.9249873E38F, -6.5571545E37F, 1.5793603E38F, 1.1638836E38F, 1.7183622E38F, 1.047372E38F, 1.0768215E38F}, 0) ;
            p61.q_SET(new float[] {2.805823E38F, 2.0323112E38F, 1.0742052E38F, 2.1752408E38F}, 0) ;
            SMP_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xtrack_error == (float)7.6340865E37F);
                Debug.Assert(pack.aspd_error == (float)6.978496E37F);
                Debug.Assert(pack.nav_pitch == (float)2.3997214E38F);
                Debug.Assert(pack.nav_bearing == (short)(short) -17023);
                Debug.Assert(pack.target_bearing == (short)(short)2280);
                Debug.Assert(pack.alt_error == (float)2.6599094E38F);
                Debug.Assert(pack.nav_roll == (float) -3.2975807E38F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)11827);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.wp_dist = (ushort)(ushort)11827;
            p62.alt_error = (float)2.6599094E38F;
            p62.xtrack_error = (float)7.6340865E37F;
            p62.nav_bearing = (short)(short) -17023;
            p62.target_bearing = (short)(short)2280;
            p62.aspd_error = (float)6.978496E37F;
            p62.nav_roll = (float) -3.2975807E38F;
            p62.nav_pitch = (float)2.3997214E38F;
            SMP_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-1.2440195E38F, 3.3629154E38F, 1.7697684E38F, 2.3551163E38F, 1.5838448E38F, -2.1000253E38F, 1.5708561E38F, -1.7531186E38F, -2.619504E37F, 6.2208757E37F, -1.2589429E38F, 3.4275858E37F, 8.8529416E36F, -3.1669508E38F, 2.7426199E38F, 1.1208883E38F, 3.3502772E38F, -1.9446288E38F, -1.996655E38F, -3.2237416E38F, -2.6540026E38F, 1.4637534E38F, -6.7623333E37F, 7.752638E37F, -1.4897773E38F, 2.0377457E38F, -1.016127E38F, -2.48022E38F, -2.8108345E38F, -1.8833546E38F, -2.2185877E38F, -3.0959052E38F, -1.859528E38F, 2.317439E38F, 1.668799E38F, -3.0733315E38F}));
                Debug.Assert(pack.vx == (float)1.5484061E38F);
                Debug.Assert(pack.lon == (int) -614538262);
                Debug.Assert(pack.lat == (int)2069269534);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
                Debug.Assert(pack.vy == (float) -7.0553067E37F);
                Debug.Assert(pack.vz == (float) -1.1474984E37F);
                Debug.Assert(pack.alt == (int)1278414578);
                Debug.Assert(pack.time_usec == (ulong)3374237167570349028L);
                Debug.Assert(pack.relative_alt == (int) -732552727);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.relative_alt = (int) -732552727;
            p63.time_usec = (ulong)3374237167570349028L;
            p63.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS;
            p63.lon = (int) -614538262;
            p63.covariance_SET(new float[] {-1.2440195E38F, 3.3629154E38F, 1.7697684E38F, 2.3551163E38F, 1.5838448E38F, -2.1000253E38F, 1.5708561E38F, -1.7531186E38F, -2.619504E37F, 6.2208757E37F, -1.2589429E38F, 3.4275858E37F, 8.8529416E36F, -3.1669508E38F, 2.7426199E38F, 1.1208883E38F, 3.3502772E38F, -1.9446288E38F, -1.996655E38F, -3.2237416E38F, -2.6540026E38F, 1.4637534E38F, -6.7623333E37F, 7.752638E37F, -1.4897773E38F, 2.0377457E38F, -1.016127E38F, -2.48022E38F, -2.8108345E38F, -1.8833546E38F, -2.2185877E38F, -3.0959052E38F, -1.859528E38F, 2.317439E38F, 1.668799E38F, -3.0733315E38F}, 0) ;
            p63.alt = (int)1278414578;
            p63.vx = (float)1.5484061E38F;
            p63.vy = (float) -7.0553067E37F;
            p63.vz = (float) -1.1474984E37F;
            p63.lat = (int)2069269534;
            SMP_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)5118486484600797702L);
                Debug.Assert(pack.y == (float)1.9410355E38F);
                Debug.Assert(pack.vy == (float)2.5714033E38F);
                Debug.Assert(pack.ay == (float) -1.8506062E38F);
                Debug.Assert(pack.az == (float) -4.7683986E37F);
                Debug.Assert(pack.vx == (float) -1.8646907E38F);
                Debug.Assert(pack.z == (float) -3.1775482E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {2.003562E38F, 2.117194E38F, 1.3793556E38F, -6.5904724E37F, -9.2782745E36F, 3.6431006E37F, -1.036877E38F, -1.3530932E38F, -2.7351207E38F, -1.4804205E38F, -2.3979978E38F, -9.454955E37F, -2.874311E38F, 1.5949095E38F, -4.532715E37F, 2.9653197E38F, 3.293975E38F, -7.2943313E37F, 1.9259255E38F, 1.9174469E38F, 1.896015E38F, -2.1583597E38F, -1.7065548E38F, 6.4646317E37F, -1.5699977E38F, -8.585519E37F, 2.0596128E38F, 2.211698E38F, -1.0858029E38F, 1.0352082E38F, 5.9043043E37F, 8.457727E37F, -1.4524738E38F, 2.0247226E38F, -1.9239103E38F, -2.3561045E38F, 2.190105E38F, -3.1282885E38F, 4.9873573E37F, -7.924246E37F, 1.776742E38F, 1.9194015E38F, 2.1716423E38F, -2.07235E38F, -3.3589947E37F}));
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
                Debug.Assert(pack.vz == (float)1.8555407E38F);
                Debug.Assert(pack.x == (float) -6.01804E37F);
                Debug.Assert(pack.ax == (float)2.4264948E38F);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS;
            p64.y = (float)1.9410355E38F;
            p64.az = (float) -4.7683986E37F;
            p64.ay = (float) -1.8506062E38F;
            p64.z = (float) -3.1775482E38F;
            p64.vy = (float)2.5714033E38F;
            p64.vx = (float) -1.8646907E38F;
            p64.vz = (float)1.8555407E38F;
            p64.x = (float) -6.01804E37F;
            p64.time_usec = (ulong)5118486484600797702L;
            p64.ax = (float)2.4264948E38F;
            p64.covariance_SET(new float[] {2.003562E38F, 2.117194E38F, 1.3793556E38F, -6.5904724E37F, -9.2782745E36F, 3.6431006E37F, -1.036877E38F, -1.3530932E38F, -2.7351207E38F, -1.4804205E38F, -2.3979978E38F, -9.454955E37F, -2.874311E38F, 1.5949095E38F, -4.532715E37F, 2.9653197E38F, 3.293975E38F, -7.2943313E37F, 1.9259255E38F, 1.9174469E38F, 1.896015E38F, -2.1583597E38F, -1.7065548E38F, 6.4646317E37F, -1.5699977E38F, -8.585519E37F, 2.0596128E38F, 2.211698E38F, -1.0858029E38F, 1.0352082E38F, 5.9043043E37F, 8.457727E37F, -1.4524738E38F, 2.0247226E38F, -1.9239103E38F, -2.3561045E38F, 2.190105E38F, -3.1282885E38F, 4.9873573E37F, -7.924246E37F, 1.776742E38F, 1.9194015E38F, 2.1716423E38F, -2.07235E38F, -3.3589947E37F}, 0) ;
            SMP_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)4016411236U);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)3981);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)56338);
                Debug.Assert(pack.chancount == (byte)(byte)201);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)5765);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)36171);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)50226);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)28475);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)48666);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)22691);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)54696);
                Debug.Assert(pack.rssi == (byte)(byte)37);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)10561);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)59990);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)18375);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)54534);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)13522);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)18439);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)58287);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)23977);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)4833);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan8_raw = (ushort)(ushort)13522;
            p65.chan2_raw = (ushort)(ushort)22691;
            p65.chan14_raw = (ushort)(ushort)48666;
            p65.chan17_raw = (ushort)(ushort)54534;
            p65.chan12_raw = (ushort)(ushort)56338;
            p65.chan5_raw = (ushort)(ushort)23977;
            p65.rssi = (byte)(byte)37;
            p65.chancount = (byte)(byte)201;
            p65.chan18_raw = (ushort)(ushort)50226;
            p65.chan6_raw = (ushort)(ushort)36171;
            p65.chan7_raw = (ushort)(ushort)18439;
            p65.time_boot_ms = (uint)4016411236U;
            p65.chan15_raw = (ushort)(ushort)3981;
            p65.chan11_raw = (ushort)(ushort)54696;
            p65.chan10_raw = (ushort)(ushort)18375;
            p65.chan4_raw = (ushort)(ushort)10561;
            p65.chan13_raw = (ushort)(ushort)4833;
            p65.chan9_raw = (ushort)(ushort)28475;
            p65.chan3_raw = (ushort)(ushort)58287;
            p65.chan1_raw = (ushort)(ushort)5765;
            p65.chan16_raw = (ushort)(ushort)59990;
            SMP_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)38872);
                Debug.Assert(pack.target_component == (byte)(byte)32);
                Debug.Assert(pack.req_stream_id == (byte)(byte)25);
                Debug.Assert(pack.target_system == (byte)(byte)158);
                Debug.Assert(pack.start_stop == (byte)(byte)209);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_component = (byte)(byte)32;
            p66.target_system = (byte)(byte)158;
            p66.start_stop = (byte)(byte)209;
            p66.req_stream_id = (byte)(byte)25;
            p66.req_message_rate = (ushort)(ushort)38872;
            SMP_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_rate == (ushort)(ushort)20555);
                Debug.Assert(pack.on_off == (byte)(byte)84);
                Debug.Assert(pack.stream_id == (byte)(byte)76);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.on_off = (byte)(byte)84;
            p67.stream_id = (byte)(byte)76;
            p67.message_rate = (ushort)(ushort)20555;
            SMP_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.buttons == (ushort)(ushort)56887);
                Debug.Assert(pack.z == (short)(short)3788);
                Debug.Assert(pack.y == (short)(short)12953);
                Debug.Assert(pack.r == (short)(short)30213);
                Debug.Assert(pack.target == (byte)(byte)49);
                Debug.Assert(pack.x == (short)(short) -16092);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.y = (short)(short)12953;
            p69.x = (short)(short) -16092;
            p69.buttons = (ushort)(ushort)56887;
            p69.z = (short)(short)3788;
            p69.target = (byte)(byte)49;
            p69.r = (short)(short)30213;
            SMP_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)39496);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)46667);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)42424);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)55878);
                Debug.Assert(pack.target_system == (byte)(byte)181);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)19357);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)12623);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)26376);
                Debug.Assert(pack.target_component == (byte)(byte)62);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)13925);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan4_raw = (ushort)(ushort)19357;
            p70.chan1_raw = (ushort)(ushort)42424;
            p70.target_component = (byte)(byte)62;
            p70.chan2_raw = (ushort)(ushort)39496;
            p70.target_system = (byte)(byte)181;
            p70.chan3_raw = (ushort)(ushort)46667;
            p70.chan8_raw = (ushort)(ushort)55878;
            p70.chan6_raw = (ushort)(ushort)26376;
            p70.chan7_raw = (ushort)(ushort)13925;
            p70.chan5_raw = (ushort)(ushort)12623;
            SMP_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -1.3916131E38F);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_PATHPLANNING);
                Debug.Assert(pack.current == (byte)(byte)237);
                Debug.Assert(pack.y == (int) -2099211949);
                Debug.Assert(pack.param3 == (float)2.7554252E38F);
                Debug.Assert(pack.param4 == (float) -5.5253994E37F);
                Debug.Assert(pack.x == (int) -1722663826);
                Debug.Assert(pack.param2 == (float)1.1406667E38F);
                Debug.Assert(pack.param1 == (float)3.5500416E37F);
                Debug.Assert(pack.target_system == (byte)(byte)45);
                Debug.Assert(pack.seq == (ushort)(ushort)35260);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.autocontinue == (byte)(byte)181);
                Debug.Assert(pack.target_component == (byte)(byte)70);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.y = (int) -2099211949;
            p73.param3 = (float)2.7554252E38F;
            p73.target_component = (byte)(byte)70;
            p73.frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p73.autocontinue = (byte)(byte)181;
            p73.param4 = (float) -5.5253994E37F;
            p73.target_system = (byte)(byte)45;
            p73.current = (byte)(byte)237;
            p73.command = MAV_CMD.MAV_CMD_NAV_PATHPLANNING;
            p73.param1 = (float)3.5500416E37F;
            p73.seq = (ushort)(ushort)35260;
            p73.x = (int) -1722663826;
            p73.param2 = (float)1.1406667E38F;
            p73.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p73.z = (float) -1.3916131E38F;
            SMP_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (float) -3.3967264E38F);
                Debug.Assert(pack.airspeed == (float)5.7448095E37F);
                Debug.Assert(pack.climb == (float)1.5589242E38F);
                Debug.Assert(pack.heading == (short)(short)9100);
                Debug.Assert(pack.throttle == (ushort)(ushort)5397);
                Debug.Assert(pack.groundspeed == (float) -7.8477303E37F);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.groundspeed = (float) -7.8477303E37F;
            p74.airspeed = (float)5.7448095E37F;
            p74.alt = (float) -3.3967264E38F;
            p74.throttle = (ushort)(ushort)5397;
            p74.heading = (short)(short)9100;
            p74.climb = (float)1.5589242E38F;
            SMP_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param1 == (float) -2.2711076E38F);
                Debug.Assert(pack.z == (float) -1.1973423E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.target_system == (byte)(byte)148);
                Debug.Assert(pack.param4 == (float) -2.3960495E38F);
                Debug.Assert(pack.x == (int)247774637);
                Debug.Assert(pack.y == (int)483706644);
                Debug.Assert(pack.param2 == (float) -2.0381386E38F);
                Debug.Assert(pack.param3 == (float) -1.4510722E38F);
                Debug.Assert(pack.target_component == (byte)(byte)47);
                Debug.Assert(pack.autocontinue == (byte)(byte)18);
                Debug.Assert(pack.current == (byte)(byte)211);
            };
            GroundControl.COMMAND_INT p75 = CommunicationChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.autocontinue = (byte)(byte)18;
            p75.param1 = (float) -2.2711076E38F;
            p75.param2 = (float) -2.0381386E38F;
            p75.param3 = (float) -1.4510722E38F;
            p75.y = (int)483706644;
            p75.target_component = (byte)(byte)47;
            p75.target_system = (byte)(byte)148;
            p75.x = (int)247774637;
            p75.param4 = (float) -2.3960495E38F;
            p75.frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p75.command = MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL_QUAT;
            p75.current = (byte)(byte)211;
            p75.z = (float) -1.1973423E38F;
            CommunicationChannel.instance.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param4 == (float) -3.234236E38F);
                Debug.Assert(pack.param1 == (float) -1.7782008E38F);
                Debug.Assert(pack.target_component == (byte)(byte)27);
                Debug.Assert(pack.param7 == (float)1.1035361E37F);
                Debug.Assert(pack.param5 == (float)5.605276E36F);
                Debug.Assert(pack.target_system == (byte)(byte)183);
                Debug.Assert(pack.confirmation == (byte)(byte)247);
                Debug.Assert(pack.param3 == (float) -7.7471604E37F);
                Debug.Assert(pack.param6 == (float) -3.21294E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION);
                Debug.Assert(pack.param2 == (float) -2.9723096E38F);
            };
            GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.param7 = (float)1.1035361E37F;
            p76.target_component = (byte)(byte)27;
            p76.target_system = (byte)(byte)183;
            p76.confirmation = (byte)(byte)247;
            p76.param1 = (float) -1.7782008E38F;
            p76.param6 = (float) -3.21294E38F;
            p76.param4 = (float) -3.234236E38F;
            p76.command = MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION;
            p76.param5 = (float)5.605276E36F;
            p76.param3 = (float) -7.7471604E37F;
            p76.param2 = (float) -2.9723096E38F;
            CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result == MAV_RESULT.MAV_RESULT_FAILED);
                Debug.Assert(pack.result_param2_TRY(ph) == (int)1479195336);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)208);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)180);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)235);
            };
            GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.target_system_SET((byte)(byte)208, PH) ;
            p77.target_component_SET((byte)(byte)235, PH) ;
            p77.progress_SET((byte)(byte)180, PH) ;
            p77.result_param2_SET((int)1479195336, PH) ;
            p77.result = MAV_RESULT.MAV_RESULT_FAILED;
            p77.command = MAV_CMD.MAV_CMD_REQUEST_CAMERA_INFORMATION;
            CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -4.651784E37F);
                Debug.Assert(pack.time_boot_ms == (uint)448709149U);
                Debug.Assert(pack.mode_switch == (byte)(byte)101);
                Debug.Assert(pack.thrust == (float)1.8374017E36F);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)40);
                Debug.Assert(pack.yaw == (float) -1.0443185E38F);
                Debug.Assert(pack.roll == (float)1.0647E38F);
            };
            GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.manual_override_switch = (byte)(byte)40;
            p81.time_boot_ms = (uint)448709149U;
            p81.mode_switch = (byte)(byte)101;
            p81.roll = (float)1.0647E38F;
            p81.yaw = (float) -1.0443185E38F;
            p81.thrust = (float)1.8374017E36F;
            p81.pitch = (float) -4.651784E37F;
            CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)203);
                Debug.Assert(pack.thrust == (float)2.4034522E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.7175455E38F, 1.2065678E38F, -3.1396659E38F, -1.0828131E38F}));
                Debug.Assert(pack.body_pitch_rate == (float) -1.6056402E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3266348417U);
                Debug.Assert(pack.body_roll_rate == (float)2.574733E38F);
                Debug.Assert(pack.target_system == (byte)(byte)95);
                Debug.Assert(pack.body_yaw_rate == (float)1.9199558E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)4);
            };
            GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.type_mask = (byte)(byte)4;
            p82.body_yaw_rate = (float)1.9199558E38F;
            p82.time_boot_ms = (uint)3266348417U;
            p82.target_system = (byte)(byte)95;
            p82.target_component = (byte)(byte)203;
            p82.body_roll_rate = (float)2.574733E38F;
            p82.thrust = (float)2.4034522E38F;
            p82.q_SET(new float[] {1.7175455E38F, 1.2065678E38F, -3.1396659E38F, -1.0828131E38F}, 0) ;
            p82.body_pitch_rate = (float) -1.6056402E38F;
            CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)69647895U);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-7.499944E37F, -2.234344E38F, 7.30236E37F, -2.9242194E38F}));
                Debug.Assert(pack.type_mask == (byte)(byte)110);
                Debug.Assert(pack.thrust == (float)2.5545956E37F);
                Debug.Assert(pack.body_roll_rate == (float) -1.4971711E38F);
                Debug.Assert(pack.body_pitch_rate == (float) -3.3319258E37F);
                Debug.Assert(pack.body_yaw_rate == (float) -2.1123276E38F);
            };
            GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.time_boot_ms = (uint)69647895U;
            p83.body_pitch_rate = (float) -3.3319258E37F;
            p83.q_SET(new float[] {-7.499944E37F, -2.234344E38F, 7.30236E37F, -2.9242194E38F}, 0) ;
            p83.body_roll_rate = (float) -1.4971711E38F;
            p83.thrust = (float)2.5545956E37F;
            p83.type_mask = (byte)(byte)110;
            p83.body_yaw_rate = (float) -2.1123276E38F;
            CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (ushort)(ushort)61177);
                Debug.Assert(pack.vx == (float) -1.7544238E38F);
                Debug.Assert(pack.yaw == (float) -2.8530414E37F);
                Debug.Assert(pack.afz == (float) -3.166674E38F);
                Debug.Assert(pack.vz == (float)6.286428E37F);
                Debug.Assert(pack.time_boot_ms == (uint)4238717314U);
                Debug.Assert(pack.afx == (float)1.7870188E38F);
                Debug.Assert(pack.target_component == (byte)(byte)100);
                Debug.Assert(pack.x == (float)1.8580296E38F);
                Debug.Assert(pack.yaw_rate == (float) -3.0336327E38F);
                Debug.Assert(pack.y == (float)1.3533892E37F);
                Debug.Assert(pack.z == (float) -1.7565447E38F);
                Debug.Assert(pack.afy == (float)1.6412858E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.target_system == (byte)(byte)52);
                Debug.Assert(pack.vy == (float) -1.7193468E38F);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.yaw = (float) -2.8530414E37F;
            p84.y = (float)1.3533892E37F;
            p84.x = (float)1.8580296E38F;
            p84.z = (float) -1.7565447E38F;
            p84.afy = (float)1.6412858E38F;
            p84.time_boot_ms = (uint)4238717314U;
            p84.afz = (float) -3.166674E38F;
            p84.target_system = (byte)(byte)52;
            p84.target_component = (byte)(byte)100;
            p84.yaw_rate = (float) -3.0336327E38F;
            p84.vy = (float) -1.7193468E38F;
            p84.afx = (float)1.7870188E38F;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p84.vz = (float)6.286428E37F;
            p84.type_mask = (ushort)(ushort)61177;
            p84.vx = (float) -1.7544238E38F;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1403162267U);
                Debug.Assert(pack.vx == (float)3.3222374E38F);
                Debug.Assert(pack.lat_int == (int)1925370883);
                Debug.Assert(pack.afy == (float)1.1057568E38F);
                Debug.Assert(pack.target_system == (byte)(byte)87);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.afz == (float)1.4522837E38F);
                Debug.Assert(pack.yaw_rate == (float) -3.354892E38F);
                Debug.Assert(pack.afx == (float) -2.7086603E37F);
                Debug.Assert(pack.alt == (float) -3.262828E38F);
                Debug.Assert(pack.yaw == (float) -2.362925E38F);
                Debug.Assert(pack.vz == (float) -3.2275839E38F);
                Debug.Assert(pack.lon_int == (int) -487993389);
                Debug.Assert(pack.target_component == (byte)(byte)251);
                Debug.Assert(pack.vy == (float)2.2595588E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)22763);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.lon_int = (int) -487993389;
            p86.vx = (float)3.3222374E38F;
            p86.lat_int = (int)1925370883;
            p86.vy = (float)2.2595588E38F;
            p86.afx = (float) -2.7086603E37F;
            p86.alt = (float) -3.262828E38F;
            p86.yaw = (float) -2.362925E38F;
            p86.yaw_rate = (float) -3.354892E38F;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p86.target_component = (byte)(byte)251;
            p86.target_system = (byte)(byte)87;
            p86.type_mask = (ushort)(ushort)22763;
            p86.afz = (float)1.4522837E38F;
            p86.afy = (float)1.1057568E38F;
            p86.vz = (float) -3.2275839E38F;
            p86.time_boot_ms = (uint)1403162267U;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)5.0003752E36F);
                Debug.Assert(pack.alt == (float) -2.2953932E38F);
                Debug.Assert(pack.vy == (float)1.5621227E38F);
                Debug.Assert(pack.lon_int == (int)636234594);
                Debug.Assert(pack.type_mask == (ushort)(ushort)12256);
                Debug.Assert(pack.afz == (float)1.5202763E38F);
                Debug.Assert(pack.vx == (float) -1.7035843E38F);
                Debug.Assert(pack.afx == (float)3.3605624E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.afy == (float)2.655725E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3189966662U);
                Debug.Assert(pack.yaw_rate == (float)6.4906885E37F);
                Debug.Assert(pack.vz == (float)7.646182E37F);
                Debug.Assert(pack.lat_int == (int) -333874111);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.afx = (float)3.3605624E38F;
            p87.vx = (float) -1.7035843E38F;
            p87.afy = (float)2.655725E38F;
            p87.type_mask = (ushort)(ushort)12256;
            p87.time_boot_ms = (uint)3189966662U;
            p87.yaw_rate = (float)6.4906885E37F;
            p87.yaw = (float)5.0003752E36F;
            p87.afz = (float)1.5202763E38F;
            p87.lat_int = (int) -333874111;
            p87.vy = (float)1.5621227E38F;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p87.alt = (float) -2.2953932E38F;
            p87.lon_int = (int)636234594;
            p87.vz = (float)7.646182E37F;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -2.7515383E37F);
                Debug.Assert(pack.y == (float) -1.372015E38F);
                Debug.Assert(pack.x == (float)2.3416946E38F);
                Debug.Assert(pack.roll == (float) -1.851474E38F);
                Debug.Assert(pack.z == (float) -3.3806834E38F);
                Debug.Assert(pack.pitch == (float) -1.5583018E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1896326969U);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.z = (float) -3.3806834E38F;
            p89.yaw = (float) -2.7515383E37F;
            p89.roll = (float) -1.851474E38F;
            p89.y = (float) -1.372015E38F;
            p89.time_boot_ms = (uint)1896326969U;
            p89.pitch = (float) -1.5583018E38F;
            p89.x = (float)2.3416946E38F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.roll == (float)1.827148E38F);
                Debug.Assert(pack.vx == (short)(short) -5766);
                Debug.Assert(pack.yaw == (float)3.4009048E38F);
                Debug.Assert(pack.zacc == (short)(short) -31749);
                Debug.Assert(pack.yacc == (short)(short) -17358);
                Debug.Assert(pack.xacc == (short)(short) -24693);
                Debug.Assert(pack.pitchspeed == (float) -3.1922184E38F);
                Debug.Assert(pack.rollspeed == (float) -3.728902E37F);
                Debug.Assert(pack.yawspeed == (float) -2.8727528E38F);
                Debug.Assert(pack.time_usec == (ulong)5723119883023753656L);
                Debug.Assert(pack.alt == (int)1363148797);
                Debug.Assert(pack.pitch == (float) -2.7976485E38F);
                Debug.Assert(pack.lat == (int) -1814375321);
                Debug.Assert(pack.lon == (int) -1584718923);
                Debug.Assert(pack.vy == (short)(short) -29485);
                Debug.Assert(pack.vz == (short)(short) -20075);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.alt = (int)1363148797;
            p90.pitch = (float) -2.7976485E38F;
            p90.roll = (float)1.827148E38F;
            p90.yacc = (short)(short) -17358;
            p90.lon = (int) -1584718923;
            p90.lat = (int) -1814375321;
            p90.yawspeed = (float) -2.8727528E38F;
            p90.pitchspeed = (float) -3.1922184E38F;
            p90.yaw = (float)3.4009048E38F;
            p90.vz = (short)(short) -20075;
            p90.vy = (short)(short) -29485;
            p90.vx = (short)(short) -5766;
            p90.zacc = (short)(short) -31749;
            p90.xacc = (short)(short) -24693;
            p90.time_usec = (ulong)5723119883023753656L;
            p90.rollspeed = (float) -3.728902E37F;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aux3 == (float) -3.2609423E38F);
                Debug.Assert(pack.yaw_rudder == (float)7.305814E37F);
                Debug.Assert(pack.throttle == (float)2.6321497E38F);
                Debug.Assert(pack.aux4 == (float) -1.321835E38F);
                Debug.Assert(pack.aux2 == (float)2.4954524E37F);
                Debug.Assert(pack.pitch_elevator == (float) -1.2683494E38F);
                Debug.Assert(pack.nav_mode == (byte)(byte)86);
                Debug.Assert(pack.roll_ailerons == (float)2.8240493E38F);
                Debug.Assert(pack.aux1 == (float)2.8220876E38F);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_PREFLIGHT);
                Debug.Assert(pack.time_usec == (ulong)6084038432936390325L);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.throttle = (float)2.6321497E38F;
            p91.aux3 = (float) -3.2609423E38F;
            p91.time_usec = (ulong)6084038432936390325L;
            p91.mode = MAV_MODE.MAV_MODE_PREFLIGHT;
            p91.roll_ailerons = (float)2.8240493E38F;
            p91.aux4 = (float) -1.321835E38F;
            p91.aux2 = (float)2.4954524E37F;
            p91.yaw_rudder = (float)7.305814E37F;
            p91.nav_mode = (byte)(byte)86;
            p91.aux1 = (float)2.8220876E38F;
            p91.pitch_elevator = (float) -1.2683494E38F;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)65495);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)14495);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)12964);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)31068);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)37507);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)22281);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)60173);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)49120);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)41712);
                Debug.Assert(pack.time_usec == (ulong)7620785592953404806L);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)10341);
                Debug.Assert(pack.rssi == (byte)(byte)83);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)38699);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)23095);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan7_raw = (ushort)(ushort)23095;
            p92.chan11_raw = (ushort)(ushort)22281;
            p92.chan6_raw = (ushort)(ushort)14495;
            p92.chan9_raw = (ushort)(ushort)37507;
            p92.chan2_raw = (ushort)(ushort)31068;
            p92.chan4_raw = (ushort)(ushort)10341;
            p92.chan12_raw = (ushort)(ushort)38699;
            p92.chan5_raw = (ushort)(ushort)41712;
            p92.time_usec = (ulong)7620785592953404806L;
            p92.chan10_raw = (ushort)(ushort)65495;
            p92.chan1_raw = (ushort)(ushort)49120;
            p92.chan3_raw = (ushort)(ushort)60173;
            p92.chan8_raw = (ushort)(ushort)12964;
            p92.rssi = (byte)(byte)83;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)1027423671035033300L);
                Debug.Assert(pack.flags == (ulong)796905261404182770L);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_GUIDED_ARMED);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {3.2347922E38F, 2.8955592E38F, -1.501268E38F, 3.0451365E38F, 4.0701267E37F, -5.0192656E37F, 1.9154075E38F, -2.8161043E38F, -2.266393E38F, 1.1089141E38F, -1.747421E38F, -1.189649E38F, -2.4598993E38F, -2.493496E38F, 3.562616E37F, -1.0214945E38F}));
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.controls_SET(new float[] {3.2347922E38F, 2.8955592E38F, -1.501268E38F, 3.0451365E38F, 4.0701267E37F, -5.0192656E37F, 1.9154075E38F, -2.8161043E38F, -2.266393E38F, 1.1089141E38F, -1.747421E38F, -1.189649E38F, -2.4598993E38F, -2.493496E38F, 3.562616E37F, -1.0214945E38F}, 0) ;
            p93.time_usec = (ulong)1027423671035033300L;
            p93.mode = MAV_MODE.MAV_MODE_GUIDED_ARMED;
            p93.flags = (ulong)796905261404182770L;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)2765377050983077274L);
                Debug.Assert(pack.sensor_id == (byte)(byte)34);
                Debug.Assert(pack.quality == (byte)(byte)228);
                Debug.Assert(pack.flow_comp_m_x == (float)1.9555634E38F);
                Debug.Assert(pack.flow_y == (short)(short) -16395);
                Debug.Assert(pack.flow_comp_m_y == (float) -2.968577E37F);
                Debug.Assert(pack.ground_distance == (float)6.128928E37F);
                Debug.Assert(pack.flow_x == (short)(short)10961);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float)1.0886199E38F);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)2.2388346E38F);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.flow_rate_x_SET((float)2.2388346E38F, PH) ;
            p100.flow_comp_m_x = (float)1.9555634E38F;
            p100.flow_comp_m_y = (float) -2.968577E37F;
            p100.flow_x = (short)(short)10961;
            p100.flow_y = (short)(short) -16395;
            p100.sensor_id = (byte)(byte)34;
            p100.ground_distance = (float)6.128928E37F;
            p100.flow_rate_y_SET((float)1.0886199E38F, PH) ;
            p100.quality = (byte)(byte)228;
            p100.time_usec = (ulong)2765377050983077274L;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)1.3979148E38F);
                Debug.Assert(pack.roll == (float) -4.102645E37F);
                Debug.Assert(pack.yaw == (float) -8.049209E37F);
                Debug.Assert(pack.pitch == (float)2.0228848E37F);
                Debug.Assert(pack.y == (float) -9.297764E37F);
                Debug.Assert(pack.usec == (ulong)928066387325660869L);
                Debug.Assert(pack.z == (float)1.7879321E38F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.pitch = (float)2.0228848E37F;
            p101.y = (float) -9.297764E37F;
            p101.x = (float)1.3979148E38F;
            p101.roll = (float) -4.102645E37F;
            p101.z = (float)1.7879321E38F;
            p101.yaw = (float) -8.049209E37F;
            p101.usec = (ulong)928066387325660869L;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -2.0093678E37F);
                Debug.Assert(pack.z == (float) -8.120953E37F);
                Debug.Assert(pack.y == (float)2.337086E38F);
                Debug.Assert(pack.x == (float) -2.2700166E38F);
                Debug.Assert(pack.usec == (ulong)1618903692820364966L);
                Debug.Assert(pack.roll == (float)2.8915875E38F);
                Debug.Assert(pack.pitch == (float)2.051413E38F);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.x = (float) -2.2700166E38F;
            p102.z = (float) -8.120953E37F;
            p102.yaw = (float) -2.0093678E37F;
            p102.pitch = (float)2.051413E38F;
            p102.roll = (float)2.8915875E38F;
            p102.y = (float)2.337086E38F;
            p102.usec = (ulong)1618903692820364966L;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -1.1545192E37F);
                Debug.Assert(pack.y == (float) -2.3168718E38F);
                Debug.Assert(pack.x == (float)6.313152E37F);
                Debug.Assert(pack.usec == (ulong)6761932409000475772L);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.z = (float) -1.1545192E37F;
            p103.x = (float)6.313152E37F;
            p103.usec = (ulong)6761932409000475772L;
            p103.y = (float) -2.3168718E38F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -6.18481E37F);
                Debug.Assert(pack.y == (float)2.1158537E38F);
                Debug.Assert(pack.roll == (float) -2.0806894E38F);
                Debug.Assert(pack.x == (float)2.5039387E38F);
                Debug.Assert(pack.yaw == (float)2.1861418E38F);
                Debug.Assert(pack.pitch == (float)9.64288E37F);
                Debug.Assert(pack.usec == (ulong)7656428474045858584L);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.roll = (float) -2.0806894E38F;
            p104.usec = (ulong)7656428474045858584L;
            p104.pitch = (float)9.64288E37F;
            p104.x = (float)2.5039387E38F;
            p104.y = (float)2.1158537E38F;
            p104.z = (float) -6.18481E37F;
            p104.yaw = (float)2.1861418E38F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)5803885539092710481L);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)40703);
                Debug.Assert(pack.zgyro == (float) -8.2551156E37F);
                Debug.Assert(pack.ymag == (float)7.8175926E37F);
                Debug.Assert(pack.yacc == (float) -5.7584894E37F);
                Debug.Assert(pack.zacc == (float) -1.6901868E38F);
                Debug.Assert(pack.ygyro == (float) -3.29568E38F);
                Debug.Assert(pack.abs_pressure == (float) -1.2794989E38F);
                Debug.Assert(pack.xgyro == (float)3.135552E37F);
                Debug.Assert(pack.temperature == (float)5.2331654E37F);
                Debug.Assert(pack.pressure_alt == (float)6.1047345E37F);
                Debug.Assert(pack.zmag == (float)7.9546074E37F);
                Debug.Assert(pack.xmag == (float) -4.4823123E35F);
                Debug.Assert(pack.diff_pressure == (float) -1.4217392E37F);
                Debug.Assert(pack.xacc == (float) -2.0149941E38F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.ymag = (float)7.8175926E37F;
            p105.time_usec = (ulong)5803885539092710481L;
            p105.diff_pressure = (float) -1.4217392E37F;
            p105.xacc = (float) -2.0149941E38F;
            p105.xmag = (float) -4.4823123E35F;
            p105.zacc = (float) -1.6901868E38F;
            p105.fields_updated = (ushort)(ushort)40703;
            p105.pressure_alt = (float)6.1047345E37F;
            p105.zgyro = (float) -8.2551156E37F;
            p105.ygyro = (float) -3.29568E38F;
            p105.yacc = (float) -5.7584894E37F;
            p105.temperature = (float)5.2331654E37F;
            p105.zmag = (float)7.9546074E37F;
            p105.xgyro = (float)3.135552E37F;
            p105.abs_pressure = (float) -1.2794989E38F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.quality == (byte)(byte)234);
                Debug.Assert(pack.sensor_id == (byte)(byte)238);
                Debug.Assert(pack.time_delta_distance_us == (uint)1367198390U);
                Debug.Assert(pack.time_usec == (ulong)20204167227478984L);
                Debug.Assert(pack.integration_time_us == (uint)2677018692U);
                Debug.Assert(pack.integrated_zgyro == (float)2.9608328E38F);
                Debug.Assert(pack.integrated_y == (float) -2.640378E38F);
                Debug.Assert(pack.integrated_xgyro == (float) -1.8989976E38F);
                Debug.Assert(pack.integrated_x == (float) -1.717747E37F);
                Debug.Assert(pack.distance == (float)1.3579613E38F);
                Debug.Assert(pack.integrated_ygyro == (float)1.8588875E38F);
                Debug.Assert(pack.temperature == (short)(short) -27009);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.time_delta_distance_us = (uint)1367198390U;
            p106.quality = (byte)(byte)234;
            p106.integrated_x = (float) -1.717747E37F;
            p106.integrated_y = (float) -2.640378E38F;
            p106.sensor_id = (byte)(byte)238;
            p106.integration_time_us = (uint)2677018692U;
            p106.temperature = (short)(short) -27009;
            p106.distance = (float)1.3579613E38F;
            p106.time_usec = (ulong)20204167227478984L;
            p106.integrated_ygyro = (float)1.8588875E38F;
            p106.integrated_zgyro = (float)2.9608328E38F;
            p106.integrated_xgyro = (float) -1.8989976E38F;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fields_updated == (uint)661414072U);
                Debug.Assert(pack.xmag == (float) -1.6562E38F);
                Debug.Assert(pack.ygyro == (float) -1.6838453E38F);
                Debug.Assert(pack.diff_pressure == (float) -1.4478797E38F);
                Debug.Assert(pack.zmag == (float)9.405177E37F);
                Debug.Assert(pack.ymag == (float) -1.8505748E38F);
                Debug.Assert(pack.pressure_alt == (float)6.1979175E37F);
                Debug.Assert(pack.yacc == (float)1.8908003E38F);
                Debug.Assert(pack.abs_pressure == (float) -1.7424642E38F);
                Debug.Assert(pack.zgyro == (float) -1.4307797E38F);
                Debug.Assert(pack.time_usec == (ulong)5490330257755438768L);
                Debug.Assert(pack.xacc == (float)1.0661483E38F);
                Debug.Assert(pack.zacc == (float) -7.3051723E37F);
                Debug.Assert(pack.xgyro == (float) -2.7249367E38F);
                Debug.Assert(pack.temperature == (float)1.9773988E38F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.diff_pressure = (float) -1.4478797E38F;
            p107.fields_updated = (uint)661414072U;
            p107.temperature = (float)1.9773988E38F;
            p107.xmag = (float) -1.6562E38F;
            p107.yacc = (float)1.8908003E38F;
            p107.zgyro = (float) -1.4307797E38F;
            p107.zmag = (float)9.405177E37F;
            p107.xacc = (float)1.0661483E38F;
            p107.time_usec = (ulong)5490330257755438768L;
            p107.xgyro = (float) -2.7249367E38F;
            p107.ymag = (float) -1.8505748E38F;
            p107.pressure_alt = (float)6.1979175E37F;
            p107.ygyro = (float) -1.6838453E38F;
            p107.zacc = (float) -7.3051723E37F;
            p107.abs_pressure = (float) -1.7424642E38F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vn == (float) -7.5268326E37F);
                Debug.Assert(pack.q2 == (float)2.9134935E38F);
                Debug.Assert(pack.lon == (float)9.210851E37F);
                Debug.Assert(pack.q4 == (float)2.3584747E38F);
                Debug.Assert(pack.vd == (float) -2.5773904E38F);
                Debug.Assert(pack.xacc == (float) -2.2097933E38F);
                Debug.Assert(pack.lat == (float) -2.0796492E38F);
                Debug.Assert(pack.xgyro == (float)1.8887483E38F);
                Debug.Assert(pack.pitch == (float)1.2607657E38F);
                Debug.Assert(pack.roll == (float)6.1904084E37F);
                Debug.Assert(pack.q3 == (float)7.527017E37F);
                Debug.Assert(pack.ve == (float)3.0618524E38F);
                Debug.Assert(pack.q1 == (float)5.9561593E37F);
                Debug.Assert(pack.yaw == (float) -2.6173403E38F);
                Debug.Assert(pack.zgyro == (float)1.791215E37F);
                Debug.Assert(pack.ygyro == (float) -2.3609406E38F);
                Debug.Assert(pack.std_dev_vert == (float)3.112272E38F);
                Debug.Assert(pack.zacc == (float)1.3911524E38F);
                Debug.Assert(pack.std_dev_horz == (float) -1.1077931E38F);
                Debug.Assert(pack.yacc == (float) -3.0884757E38F);
                Debug.Assert(pack.alt == (float)1.5860149E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.zgyro = (float)1.791215E37F;
            p108.vd = (float) -2.5773904E38F;
            p108.ygyro = (float) -2.3609406E38F;
            p108.zacc = (float)1.3911524E38F;
            p108.ve = (float)3.0618524E38F;
            p108.lat = (float) -2.0796492E38F;
            p108.q3 = (float)7.527017E37F;
            p108.pitch = (float)1.2607657E38F;
            p108.q4 = (float)2.3584747E38F;
            p108.vn = (float) -7.5268326E37F;
            p108.std_dev_horz = (float) -1.1077931E38F;
            p108.yacc = (float) -3.0884757E38F;
            p108.q1 = (float)5.9561593E37F;
            p108.xgyro = (float)1.8887483E38F;
            p108.std_dev_vert = (float)3.112272E38F;
            p108.roll = (float)6.1904084E37F;
            p108.alt = (float)1.5860149E38F;
            p108.xacc = (float) -2.2097933E38F;
            p108.lon = (float)9.210851E37F;
            p108.q2 = (float)2.9134935E38F;
            p108.yaw = (float) -2.6173403E38F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.txbuf == (byte)(byte)252);
                Debug.Assert(pack.remrssi == (byte)(byte)29);
                Debug.Assert(pack.remnoise == (byte)(byte)168);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)64533);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)49842);
                Debug.Assert(pack.rssi == (byte)(byte)163);
                Debug.Assert(pack.noise == (byte)(byte)128);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.rxerrors = (ushort)(ushort)49842;
            p109.noise = (byte)(byte)128;
            p109.rssi = (byte)(byte)163;
            p109.fixed_ = (ushort)(ushort)64533;
            p109.remnoise = (byte)(byte)168;
            p109.remrssi = (byte)(byte)29;
            p109.txbuf = (byte)(byte)252;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_network == (byte)(byte)216);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)219, (byte)93, (byte)4, (byte)101, (byte)168, (byte)1, (byte)106, (byte)0, (byte)113, (byte)49, (byte)194, (byte)152, (byte)92, (byte)202, (byte)174, (byte)2, (byte)138, (byte)44, (byte)84, (byte)94, (byte)218, (byte)113, (byte)94, (byte)181, (byte)32, (byte)31, (byte)92, (byte)122, (byte)105, (byte)63, (byte)52, (byte)17, (byte)187, (byte)247, (byte)18, (byte)66, (byte)33, (byte)232, (byte)192, (byte)143, (byte)202, (byte)11, (byte)166, (byte)200, (byte)251, (byte)67, (byte)205, (byte)19, (byte)113, (byte)89, (byte)244, (byte)191, (byte)92, (byte)208, (byte)137, (byte)235, (byte)250, (byte)77, (byte)210, (byte)170, (byte)245, (byte)107, (byte)48, (byte)119, (byte)228, (byte)68, (byte)69, (byte)139, (byte)181, (byte)96, (byte)229, (byte)14, (byte)119, (byte)254, (byte)72, (byte)168, (byte)165, (byte)158, (byte)229, (byte)47, (byte)90, (byte)106, (byte)135, (byte)216, (byte)129, (byte)27, (byte)129, (byte)102, (byte)178, (byte)48, (byte)244, (byte)35, (byte)32, (byte)214, (byte)80, (byte)155, (byte)99, (byte)117, (byte)93, (byte)168, (byte)177, (byte)64, (byte)77, (byte)112, (byte)24, (byte)254, (byte)245, (byte)106, (byte)243, (byte)55, (byte)215, (byte)226, (byte)171, (byte)119, (byte)187, (byte)228, (byte)141, (byte)171, (byte)39, (byte)62, (byte)15, (byte)28, (byte)237, (byte)113, (byte)43, (byte)18, (byte)37, (byte)25, (byte)6, (byte)113, (byte)68, (byte)219, (byte)34, (byte)49, (byte)122, (byte)183, (byte)39, (byte)214, (byte)59, (byte)83, (byte)220, (byte)82, (byte)106, (byte)6, (byte)74, (byte)215, (byte)82, (byte)255, (byte)222, (byte)74, (byte)46, (byte)53, (byte)109, (byte)202, (byte)7, (byte)121, (byte)152, (byte)244, (byte)108, (byte)10, (byte)165, (byte)110, (byte)4, (byte)184, (byte)125, (byte)149, (byte)85, (byte)129, (byte)151, (byte)59, (byte)71, (byte)27, (byte)122, (byte)170, (byte)103, (byte)143, (byte)80, (byte)212, (byte)71, (byte)15, (byte)6, (byte)103, (byte)131, (byte)88, (byte)149, (byte)60, (byte)15, (byte)236, (byte)200, (byte)208, (byte)131, (byte)222, (byte)206, (byte)59, (byte)159, (byte)168, (byte)232, (byte)202, (byte)40, (byte)211, (byte)233, (byte)218, (byte)208, (byte)54, (byte)92, (byte)118, (byte)206, (byte)0, (byte)187, (byte)90, (byte)56, (byte)225, (byte)100, (byte)58, (byte)97, (byte)239, (byte)13, (byte)236, (byte)38, (byte)208, (byte)155, (byte)2, (byte)128, (byte)32, (byte)124, (byte)172, (byte)94, (byte)7, (byte)166, (byte)133, (byte)172, (byte)158, (byte)205, (byte)141, (byte)203, (byte)2, (byte)9, (byte)76, (byte)190, (byte)158, (byte)214, (byte)183, (byte)185, (byte)114, (byte)224, (byte)230, (byte)227, (byte)47, (byte)172, (byte)118, (byte)175}));
                Debug.Assert(pack.target_system == (byte)(byte)131);
                Debug.Assert(pack.target_component == (byte)(byte)226);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_system = (byte)(byte)131;
            p110.payload_SET(new byte[] {(byte)219, (byte)93, (byte)4, (byte)101, (byte)168, (byte)1, (byte)106, (byte)0, (byte)113, (byte)49, (byte)194, (byte)152, (byte)92, (byte)202, (byte)174, (byte)2, (byte)138, (byte)44, (byte)84, (byte)94, (byte)218, (byte)113, (byte)94, (byte)181, (byte)32, (byte)31, (byte)92, (byte)122, (byte)105, (byte)63, (byte)52, (byte)17, (byte)187, (byte)247, (byte)18, (byte)66, (byte)33, (byte)232, (byte)192, (byte)143, (byte)202, (byte)11, (byte)166, (byte)200, (byte)251, (byte)67, (byte)205, (byte)19, (byte)113, (byte)89, (byte)244, (byte)191, (byte)92, (byte)208, (byte)137, (byte)235, (byte)250, (byte)77, (byte)210, (byte)170, (byte)245, (byte)107, (byte)48, (byte)119, (byte)228, (byte)68, (byte)69, (byte)139, (byte)181, (byte)96, (byte)229, (byte)14, (byte)119, (byte)254, (byte)72, (byte)168, (byte)165, (byte)158, (byte)229, (byte)47, (byte)90, (byte)106, (byte)135, (byte)216, (byte)129, (byte)27, (byte)129, (byte)102, (byte)178, (byte)48, (byte)244, (byte)35, (byte)32, (byte)214, (byte)80, (byte)155, (byte)99, (byte)117, (byte)93, (byte)168, (byte)177, (byte)64, (byte)77, (byte)112, (byte)24, (byte)254, (byte)245, (byte)106, (byte)243, (byte)55, (byte)215, (byte)226, (byte)171, (byte)119, (byte)187, (byte)228, (byte)141, (byte)171, (byte)39, (byte)62, (byte)15, (byte)28, (byte)237, (byte)113, (byte)43, (byte)18, (byte)37, (byte)25, (byte)6, (byte)113, (byte)68, (byte)219, (byte)34, (byte)49, (byte)122, (byte)183, (byte)39, (byte)214, (byte)59, (byte)83, (byte)220, (byte)82, (byte)106, (byte)6, (byte)74, (byte)215, (byte)82, (byte)255, (byte)222, (byte)74, (byte)46, (byte)53, (byte)109, (byte)202, (byte)7, (byte)121, (byte)152, (byte)244, (byte)108, (byte)10, (byte)165, (byte)110, (byte)4, (byte)184, (byte)125, (byte)149, (byte)85, (byte)129, (byte)151, (byte)59, (byte)71, (byte)27, (byte)122, (byte)170, (byte)103, (byte)143, (byte)80, (byte)212, (byte)71, (byte)15, (byte)6, (byte)103, (byte)131, (byte)88, (byte)149, (byte)60, (byte)15, (byte)236, (byte)200, (byte)208, (byte)131, (byte)222, (byte)206, (byte)59, (byte)159, (byte)168, (byte)232, (byte)202, (byte)40, (byte)211, (byte)233, (byte)218, (byte)208, (byte)54, (byte)92, (byte)118, (byte)206, (byte)0, (byte)187, (byte)90, (byte)56, (byte)225, (byte)100, (byte)58, (byte)97, (byte)239, (byte)13, (byte)236, (byte)38, (byte)208, (byte)155, (byte)2, (byte)128, (byte)32, (byte)124, (byte)172, (byte)94, (byte)7, (byte)166, (byte)133, (byte)172, (byte)158, (byte)205, (byte)141, (byte)203, (byte)2, (byte)9, (byte)76, (byte)190, (byte)158, (byte)214, (byte)183, (byte)185, (byte)114, (byte)224, (byte)230, (byte)227, (byte)47, (byte)172, (byte)118, (byte)175}, 0) ;
            p110.target_network = (byte)(byte)216;
            p110.target_component = (byte)(byte)226;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ts1 == (long)1624431561591153201L);
                Debug.Assert(pack.tc1 == (long) -8176961088253804986L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -8176961088253804986L;
            p111.ts1 = (long)1624431561591153201L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)7178735613800855463L);
                Debug.Assert(pack.seq == (uint)1674373221U);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.seq = (uint)1674373221U;
            p112.time_usec = (ulong)7178735613800855463L;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (int) -1150606126);
                Debug.Assert(pack.vd == (short)(short)7910);
                Debug.Assert(pack.time_usec == (ulong)2222139046500035317L);
                Debug.Assert(pack.vel == (ushort)(ushort)7245);
                Debug.Assert(pack.fix_type == (byte)(byte)102);
                Debug.Assert(pack.lat == (int) -1807952472);
                Debug.Assert(pack.epv == (ushort)(ushort)59048);
                Debug.Assert(pack.ve == (short)(short)6865);
                Debug.Assert(pack.satellites_visible == (byte)(byte)100);
                Debug.Assert(pack.eph == (ushort)(ushort)6809);
                Debug.Assert(pack.cog == (ushort)(ushort)33503);
                Debug.Assert(pack.vn == (short)(short)3957);
                Debug.Assert(pack.lon == (int) -1213316389);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.alt = (int) -1150606126;
            p113.vel = (ushort)(ushort)7245;
            p113.epv = (ushort)(ushort)59048;
            p113.vn = (short)(short)3957;
            p113.cog = (ushort)(ushort)33503;
            p113.satellites_visible = (byte)(byte)100;
            p113.vd = (short)(short)7910;
            p113.fix_type = (byte)(byte)102;
            p113.lat = (int) -1807952472;
            p113.ve = (short)(short)6865;
            p113.time_usec = (ulong)2222139046500035317L;
            p113.lon = (int) -1213316389;
            p113.eph = (ushort)(ushort)6809;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_ygyro == (float)3.0901819E38F);
                Debug.Assert(pack.time_usec == (ulong)816746663290904903L);
                Debug.Assert(pack.time_delta_distance_us == (uint)3616306110U);
                Debug.Assert(pack.distance == (float)3.0599518E38F);
                Debug.Assert(pack.integrated_xgyro == (float) -8.917617E37F);
                Debug.Assert(pack.quality == (byte)(byte)155);
                Debug.Assert(pack.integrated_y == (float)2.7592539E38F);
                Debug.Assert(pack.temperature == (short)(short) -26997);
                Debug.Assert(pack.integration_time_us == (uint)922337039U);
                Debug.Assert(pack.sensor_id == (byte)(byte)230);
                Debug.Assert(pack.integrated_zgyro == (float)1.4476251E38F);
                Debug.Assert(pack.integrated_x == (float) -2.2280093E38F);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.integrated_y = (float)2.7592539E38F;
            p114.integrated_ygyro = (float)3.0901819E38F;
            p114.time_delta_distance_us = (uint)3616306110U;
            p114.time_usec = (ulong)816746663290904903L;
            p114.distance = (float)3.0599518E38F;
            p114.integrated_zgyro = (float)1.4476251E38F;
            p114.quality = (byte)(byte)155;
            p114.integration_time_us = (uint)922337039U;
            p114.sensor_id = (byte)(byte)230;
            p114.integrated_xgyro = (float) -8.917617E37F;
            p114.temperature = (short)(short) -26997;
            p114.integrated_x = (float) -2.2280093E38F;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)34356);
                Debug.Assert(pack.yacc == (short)(short) -20746);
                Debug.Assert(pack.rollspeed == (float) -1.6861078E38F);
                Debug.Assert(pack.yawspeed == (float) -4.2660215E37F);
                Debug.Assert(pack.lat == (int) -1726759602);
                Debug.Assert(pack.alt == (int) -605204943);
                Debug.Assert(pack.vx == (short)(short) -20966);
                Debug.Assert(pack.xacc == (short)(short)9307);
                Debug.Assert(pack.lon == (int) -1237953448);
                Debug.Assert(pack.time_usec == (ulong)4388253998990474849L);
                Debug.Assert(pack.zacc == (short)(short) -1661);
                Debug.Assert(pack.vy == (short)(short) -16062);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {-2.8490488E38F, 1.3912686E38F, 1.7108847E38F, 1.9577507E38F}));
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)22453);
                Debug.Assert(pack.vz == (short)(short) -22285);
                Debug.Assert(pack.pitchspeed == (float)2.1723883E37F);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.alt = (int) -605204943;
            p115.zacc = (short)(short) -1661;
            p115.lat = (int) -1726759602;
            p115.yawspeed = (float) -4.2660215E37F;
            p115.vx = (short)(short) -20966;
            p115.yacc = (short)(short) -20746;
            p115.true_airspeed = (ushort)(ushort)34356;
            p115.vz = (short)(short) -22285;
            p115.lon = (int) -1237953448;
            p115.pitchspeed = (float)2.1723883E37F;
            p115.rollspeed = (float) -1.6861078E38F;
            p115.attitude_quaternion_SET(new float[] {-2.8490488E38F, 1.3912686E38F, 1.7108847E38F, 1.9577507E38F}, 0) ;
            p115.xacc = (short)(short)9307;
            p115.vy = (short)(short) -16062;
            p115.time_usec = (ulong)4388253998990474849L;
            p115.ind_airspeed = (ushort)(ushort)22453;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (short)(short) -17379);
                Debug.Assert(pack.xgyro == (short)(short) -16878);
                Debug.Assert(pack.zgyro == (short)(short)1908);
                Debug.Assert(pack.time_boot_ms == (uint)2152120088U);
                Debug.Assert(pack.xmag == (short)(short) -69);
                Debug.Assert(pack.zmag == (short)(short) -14971);
                Debug.Assert(pack.xacc == (short)(short) -15429);
                Debug.Assert(pack.yacc == (short)(short)1080);
                Debug.Assert(pack.ymag == (short)(short) -19843);
                Debug.Assert(pack.zacc == (short)(short) -27864);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.xacc = (short)(short) -15429;
            p116.xmag = (short)(short) -69;
            p116.ygyro = (short)(short) -17379;
            p116.zmag = (short)(short) -14971;
            p116.time_boot_ms = (uint)2152120088U;
            p116.xgyro = (short)(short) -16878;
            p116.zacc = (short)(short) -27864;
            p116.ymag = (short)(short) -19843;
            p116.yacc = (short)(short)1080;
            p116.zgyro = (short)(short)1908;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)170);
                Debug.Assert(pack.end == (ushort)(ushort)7256);
                Debug.Assert(pack.start == (ushort)(ushort)38257);
                Debug.Assert(pack.target_system == (byte)(byte)79);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.end = (ushort)(ushort)7256;
            p117.target_component = (byte)(byte)170;
            p117.start = (ushort)(ushort)38257;
            p117.target_system = (byte)(byte)79;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)49812);
                Debug.Assert(pack.size == (uint)955935490U);
                Debug.Assert(pack.num_logs == (ushort)(ushort)52781);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)2786);
                Debug.Assert(pack.time_utc == (uint)3714204173U);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.num_logs = (ushort)(ushort)52781;
            p118.size = (uint)955935490U;
            p118.time_utc = (uint)3714204173U;
            p118.id = (ushort)(ushort)49812;
            p118.last_log_num = (ushort)(ushort)2786;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (ushort)(ushort)47050);
                Debug.Assert(pack.target_component == (byte)(byte)48);
                Debug.Assert(pack.ofs == (uint)377363772U);
                Debug.Assert(pack.count == (uint)1632478850U);
                Debug.Assert(pack.target_system == (byte)(byte)42);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_component = (byte)(byte)48;
            p119.ofs = (uint)377363772U;
            p119.target_system = (byte)(byte)42;
            p119.count = (uint)1632478850U;
            p119.id = (ushort)(ushort)47050;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)89, (byte)212, (byte)152, (byte)86, (byte)158, (byte)237, (byte)97, (byte)55, (byte)97, (byte)231, (byte)143, (byte)108, (byte)14, (byte)234, (byte)79, (byte)140, (byte)41, (byte)6, (byte)56, (byte)215, (byte)183, (byte)47, (byte)250, (byte)145, (byte)87, (byte)197, (byte)163, (byte)0, (byte)127, (byte)63, (byte)161, (byte)252, (byte)119, (byte)41, (byte)30, (byte)113, (byte)133, (byte)119, (byte)181, (byte)121, (byte)165, (byte)255, (byte)122, (byte)78, (byte)69, (byte)40, (byte)236, (byte)129, (byte)193, (byte)61, (byte)126, (byte)162, (byte)139, (byte)159, (byte)79, (byte)127, (byte)188, (byte)115, (byte)106, (byte)13, (byte)31, (byte)49, (byte)25, (byte)151, (byte)128, (byte)239, (byte)185, (byte)27, (byte)172, (byte)241, (byte)226, (byte)199, (byte)24, (byte)119, (byte)220, (byte)255, (byte)24, (byte)19, (byte)22, (byte)188, (byte)148, (byte)16, (byte)119, (byte)113, (byte)99, (byte)32, (byte)27, (byte)101, (byte)47, (byte)122}));
                Debug.Assert(pack.count == (byte)(byte)241);
                Debug.Assert(pack.id == (ushort)(ushort)41171);
                Debug.Assert(pack.ofs == (uint)863597189U);
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.count = (byte)(byte)241;
            p120.data__SET(new byte[] {(byte)89, (byte)212, (byte)152, (byte)86, (byte)158, (byte)237, (byte)97, (byte)55, (byte)97, (byte)231, (byte)143, (byte)108, (byte)14, (byte)234, (byte)79, (byte)140, (byte)41, (byte)6, (byte)56, (byte)215, (byte)183, (byte)47, (byte)250, (byte)145, (byte)87, (byte)197, (byte)163, (byte)0, (byte)127, (byte)63, (byte)161, (byte)252, (byte)119, (byte)41, (byte)30, (byte)113, (byte)133, (byte)119, (byte)181, (byte)121, (byte)165, (byte)255, (byte)122, (byte)78, (byte)69, (byte)40, (byte)236, (byte)129, (byte)193, (byte)61, (byte)126, (byte)162, (byte)139, (byte)159, (byte)79, (byte)127, (byte)188, (byte)115, (byte)106, (byte)13, (byte)31, (byte)49, (byte)25, (byte)151, (byte)128, (byte)239, (byte)185, (byte)27, (byte)172, (byte)241, (byte)226, (byte)199, (byte)24, (byte)119, (byte)220, (byte)255, (byte)24, (byte)19, (byte)22, (byte)188, (byte)148, (byte)16, (byte)119, (byte)113, (byte)99, (byte)32, (byte)27, (byte)101, (byte)47, (byte)122}, 0) ;
            p120.id = (ushort)(ushort)41171;
            p120.ofs = (uint)863597189U;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)243);
                Debug.Assert(pack.target_component == (byte)(byte)157);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)243;
            p121.target_component = (byte)(byte)157;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)30);
                Debug.Assert(pack.target_component == (byte)(byte)214);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)30;
            p122.target_component = (byte)(byte)214;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)166);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)2, (byte)138, (byte)160, (byte)168, (byte)248, (byte)22, (byte)128, (byte)125, (byte)245, (byte)68, (byte)248, (byte)146, (byte)131, (byte)148, (byte)70, (byte)133, (byte)35, (byte)86, (byte)165, (byte)62, (byte)204, (byte)7, (byte)71, (byte)211, (byte)177, (byte)107, (byte)113, (byte)2, (byte)154, (byte)37, (byte)52, (byte)249, (byte)214, (byte)85, (byte)99, (byte)254, (byte)119, (byte)61, (byte)103, (byte)212, (byte)239, (byte)69, (byte)117, (byte)193, (byte)170, (byte)182, (byte)0, (byte)245, (byte)17, (byte)88, (byte)190, (byte)206, (byte)64, (byte)219, (byte)34, (byte)229, (byte)163, (byte)97, (byte)47, (byte)23, (byte)150, (byte)235, (byte)30, (byte)171, (byte)152, (byte)152, (byte)81, (byte)11, (byte)121, (byte)26, (byte)203, (byte)71, (byte)26, (byte)222, (byte)168, (byte)75, (byte)18, (byte)60, (byte)70, (byte)139, (byte)73, (byte)33, (byte)3, (byte)118, (byte)126, (byte)136, (byte)183, (byte)155, (byte)114, (byte)83, (byte)218, (byte)149, (byte)221, (byte)57, (byte)33, (byte)248, (byte)76, (byte)30, (byte)124, (byte)141, (byte)32, (byte)141, (byte)70, (byte)80, (byte)212, (byte)2, (byte)29, (byte)209, (byte)139, (byte)171}));
                Debug.Assert(pack.target_component == (byte)(byte)236);
                Debug.Assert(pack.len == (byte)(byte)158);
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.len = (byte)(byte)158;
            p123.data__SET(new byte[] {(byte)2, (byte)138, (byte)160, (byte)168, (byte)248, (byte)22, (byte)128, (byte)125, (byte)245, (byte)68, (byte)248, (byte)146, (byte)131, (byte)148, (byte)70, (byte)133, (byte)35, (byte)86, (byte)165, (byte)62, (byte)204, (byte)7, (byte)71, (byte)211, (byte)177, (byte)107, (byte)113, (byte)2, (byte)154, (byte)37, (byte)52, (byte)249, (byte)214, (byte)85, (byte)99, (byte)254, (byte)119, (byte)61, (byte)103, (byte)212, (byte)239, (byte)69, (byte)117, (byte)193, (byte)170, (byte)182, (byte)0, (byte)245, (byte)17, (byte)88, (byte)190, (byte)206, (byte)64, (byte)219, (byte)34, (byte)229, (byte)163, (byte)97, (byte)47, (byte)23, (byte)150, (byte)235, (byte)30, (byte)171, (byte)152, (byte)152, (byte)81, (byte)11, (byte)121, (byte)26, (byte)203, (byte)71, (byte)26, (byte)222, (byte)168, (byte)75, (byte)18, (byte)60, (byte)70, (byte)139, (byte)73, (byte)33, (byte)3, (byte)118, (byte)126, (byte)136, (byte)183, (byte)155, (byte)114, (byte)83, (byte)218, (byte)149, (byte)221, (byte)57, (byte)33, (byte)248, (byte)76, (byte)30, (byte)124, (byte)141, (byte)32, (byte)141, (byte)70, (byte)80, (byte)212, (byte)2, (byte)29, (byte)209, (byte)139, (byte)171}, 0) ;
            p123.target_component = (byte)(byte)236;
            p123.target_system = (byte)(byte)166;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
                Debug.Assert(pack.lat == (int)1461199122);
                Debug.Assert(pack.vel == (ushort)(ushort)28011);
                Debug.Assert(pack.satellites_visible == (byte)(byte)194);
                Debug.Assert(pack.eph == (ushort)(ushort)257);
                Debug.Assert(pack.lon == (int)450513232);
                Debug.Assert(pack.alt == (int)1526539912);
                Debug.Assert(pack.epv == (ushort)(ushort)44725);
                Debug.Assert(pack.time_usec == (ulong)1959138867014182931L);
                Debug.Assert(pack.dgps_age == (uint)4015272383U);
                Debug.Assert(pack.dgps_numch == (byte)(byte)55);
                Debug.Assert(pack.cog == (ushort)(ushort)35561);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.satellites_visible = (byte)(byte)194;
            p124.dgps_age = (uint)4015272383U;
            p124.alt = (int)1526539912;
            p124.dgps_numch = (byte)(byte)55;
            p124.epv = (ushort)(ushort)44725;
            p124.vel = (ushort)(ushort)28011;
            p124.time_usec = (ulong)1959138867014182931L;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p124.lat = (int)1461199122;
            p124.cog = (ushort)(ushort)35561;
            p124.eph = (ushort)(ushort)257;
            p124.lon = (int)450513232;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vservo == (ushort)(ushort)33997);
                Debug.Assert(pack.Vcc == (ushort)(ushort)21178);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED));
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)21178;
            p125.Vservo = (ushort)(ushort)33997;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED);
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (byte)(byte)215);
                Debug.Assert(pack.device == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL);
                Debug.Assert(pack.baudrate == (uint)3056215309U);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)79, (byte)189, (byte)86, (byte)235, (byte)30, (byte)191, (byte)225, (byte)126, (byte)4, (byte)153, (byte)23, (byte)67, (byte)146, (byte)9, (byte)116, (byte)214, (byte)33, (byte)140, (byte)128, (byte)59, (byte)238, (byte)85, (byte)215, (byte)58, (byte)215, (byte)217, (byte)80, (byte)105, (byte)86, (byte)202, (byte)104, (byte)203, (byte)58, (byte)238, (byte)76, (byte)206, (byte)59, (byte)225, (byte)200, (byte)119, (byte)214, (byte)49, (byte)20, (byte)226, (byte)251, (byte)209, (byte)129, (byte)146, (byte)176, (byte)200, (byte)23, (byte)127, (byte)73, (byte)152, (byte)249, (byte)174, (byte)156, (byte)243, (byte)212, (byte)183, (byte)236, (byte)213, (byte)125, (byte)22, (byte)182, (byte)120, (byte)166, (byte)107, (byte)147, (byte)46}));
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND));
                Debug.Assert(pack.timeout == (ushort)(ushort)12052);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL;
            p126.timeout = (ushort)(ushort)12052;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND);
            p126.count = (byte)(byte)215;
            p126.baudrate = (uint)3056215309U;
            p126.data__SET(new byte[] {(byte)79, (byte)189, (byte)86, (byte)235, (byte)30, (byte)191, (byte)225, (byte)126, (byte)4, (byte)153, (byte)23, (byte)67, (byte)146, (byte)9, (byte)116, (byte)214, (byte)33, (byte)140, (byte)128, (byte)59, (byte)238, (byte)85, (byte)215, (byte)58, (byte)215, (byte)217, (byte)80, (byte)105, (byte)86, (byte)202, (byte)104, (byte)203, (byte)58, (byte)238, (byte)76, (byte)206, (byte)59, (byte)225, (byte)200, (byte)119, (byte)214, (byte)49, (byte)20, (byte)226, (byte)251, (byte)209, (byte)129, (byte)146, (byte)176, (byte)200, (byte)23, (byte)127, (byte)73, (byte)152, (byte)249, (byte)174, (byte)156, (byte)243, (byte)212, (byte)183, (byte)236, (byte)213, (byte)125, (byte)22, (byte)182, (byte)120, (byte)166, (byte)107, (byte)147, (byte)46}, 0) ;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rtk_health == (byte)(byte)211);
                Debug.Assert(pack.rtk_rate == (byte)(byte)135);
                Debug.Assert(pack.time_last_baseline_ms == (uint)2661819111U);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)36);
                Debug.Assert(pack.baseline_a_mm == (int) -1754412343);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)195);
                Debug.Assert(pack.nsats == (byte)(byte)232);
                Debug.Assert(pack.baseline_b_mm == (int)1489192706);
                Debug.Assert(pack.wn == (ushort)(ushort)49414);
                Debug.Assert(pack.iar_num_hypotheses == (int) -968774900);
                Debug.Assert(pack.baseline_c_mm == (int) -1942163847);
                Debug.Assert(pack.accuracy == (uint)1386298188U);
                Debug.Assert(pack.tow == (uint)120964864U);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.baseline_a_mm = (int) -1754412343;
            p127.baseline_c_mm = (int) -1942163847;
            p127.rtk_receiver_id = (byte)(byte)195;
            p127.rtk_health = (byte)(byte)211;
            p127.rtk_rate = (byte)(byte)135;
            p127.accuracy = (uint)1386298188U;
            p127.baseline_coords_type = (byte)(byte)36;
            p127.wn = (ushort)(ushort)49414;
            p127.time_last_baseline_ms = (uint)2661819111U;
            p127.baseline_b_mm = (int)1489192706;
            p127.tow = (uint)120964864U;
            p127.nsats = (byte)(byte)232;
            p127.iar_num_hypotheses = (int) -968774900;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.iar_num_hypotheses == (int) -109994927);
                Debug.Assert(pack.rtk_health == (byte)(byte)211);
                Debug.Assert(pack.rtk_rate == (byte)(byte)131);
                Debug.Assert(pack.time_last_baseline_ms == (uint)3734856539U);
                Debug.Assert(pack.baseline_a_mm == (int) -1439419564);
                Debug.Assert(pack.baseline_c_mm == (int) -1709615558);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)30);
                Debug.Assert(pack.tow == (uint)988376693U);
                Debug.Assert(pack.accuracy == (uint)2014922188U);
                Debug.Assert(pack.nsats == (byte)(byte)108);
                Debug.Assert(pack.wn == (ushort)(ushort)57388);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)197);
                Debug.Assert(pack.baseline_b_mm == (int) -1695931409);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.accuracy = (uint)2014922188U;
            p128.wn = (ushort)(ushort)57388;
            p128.baseline_a_mm = (int) -1439419564;
            p128.rtk_rate = (byte)(byte)131;
            p128.rtk_health = (byte)(byte)211;
            p128.baseline_b_mm = (int) -1695931409;
            p128.rtk_receiver_id = (byte)(byte)197;
            p128.nsats = (byte)(byte)108;
            p128.iar_num_hypotheses = (int) -109994927;
            p128.baseline_c_mm = (int) -1709615558;
            p128.time_last_baseline_ms = (uint)3734856539U;
            p128.tow = (uint)988376693U;
            p128.baseline_coords_type = (byte)(byte)30;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (short)(short)15375);
                Debug.Assert(pack.xacc == (short)(short)28424);
                Debug.Assert(pack.ymag == (short)(short) -7757);
                Debug.Assert(pack.zacc == (short)(short)21762);
                Debug.Assert(pack.xgyro == (short)(short) -9002);
                Debug.Assert(pack.zgyro == (short)(short)20234);
                Debug.Assert(pack.time_boot_ms == (uint)2991650081U);
                Debug.Assert(pack.zmag == (short)(short)21010);
                Debug.Assert(pack.xmag == (short)(short)32);
                Debug.Assert(pack.yacc == (short)(short)5632);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.zacc = (short)(short)21762;
            p129.zgyro = (short)(short)20234;
            p129.xacc = (short)(short)28424;
            p129.xmag = (short)(short)32;
            p129.xgyro = (short)(short) -9002;
            p129.zmag = (short)(short)21010;
            p129.time_boot_ms = (uint)2991650081U;
            p129.ygyro = (short)(short)15375;
            p129.ymag = (short)(short) -7757;
            p129.yacc = (short)(short)5632;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == (byte)(byte)36);
                Debug.Assert(pack.payload == (byte)(byte)28);
                Debug.Assert(pack.packets == (ushort)(ushort)24935);
                Debug.Assert(pack.width == (ushort)(ushort)60528);
                Debug.Assert(pack.height == (ushort)(ushort)31104);
                Debug.Assert(pack.jpg_quality == (byte)(byte)79);
                Debug.Assert(pack.size == (uint)138906111U);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.height = (ushort)(ushort)31104;
            p130.size = (uint)138906111U;
            p130.jpg_quality = (byte)(byte)79;
            p130.packets = (ushort)(ushort)24935;
            p130.payload = (byte)(byte)28;
            p130.type = (byte)(byte)36;
            p130.width = (ushort)(ushort)60528;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)102, (byte)79, (byte)57, (byte)167, (byte)107, (byte)122, (byte)155, (byte)68, (byte)115, (byte)197, (byte)68, (byte)144, (byte)163, (byte)153, (byte)91, (byte)255, (byte)119, (byte)208, (byte)145, (byte)56, (byte)170, (byte)110, (byte)134, (byte)246, (byte)17, (byte)91, (byte)86, (byte)70, (byte)67, (byte)74, (byte)0, (byte)220, (byte)247, (byte)119, (byte)24, (byte)154, (byte)239, (byte)64, (byte)128, (byte)198, (byte)171, (byte)64, (byte)245, (byte)247, (byte)73, (byte)227, (byte)13, (byte)147, (byte)93, (byte)44, (byte)55, (byte)199, (byte)95, (byte)73, (byte)178, (byte)194, (byte)220, (byte)154, (byte)117, (byte)29, (byte)92, (byte)58, (byte)23, (byte)37, (byte)46, (byte)45, (byte)182, (byte)78, (byte)78, (byte)76, (byte)141, (byte)165, (byte)30, (byte)118, (byte)233, (byte)84, (byte)70, (byte)100, (byte)156, (byte)110, (byte)160, (byte)81, (byte)81, (byte)44, (byte)130, (byte)63, (byte)202, (byte)4, (byte)93, (byte)174, (byte)144, (byte)209, (byte)211, (byte)222, (byte)110, (byte)225, (byte)191, (byte)126, (byte)205, (byte)227, (byte)249, (byte)17, (byte)191, (byte)206, (byte)71, (byte)167, (byte)58, (byte)148, (byte)81, (byte)143, (byte)129, (byte)82, (byte)231, (byte)167, (byte)250, (byte)184, (byte)124, (byte)134, (byte)199, (byte)103, (byte)68, (byte)179, (byte)81, (byte)235, (byte)75, (byte)179, (byte)92, (byte)130, (byte)208, (byte)157, (byte)150, (byte)194, (byte)235, (byte)230, (byte)37, (byte)18, (byte)148, (byte)88, (byte)36, (byte)175, (byte)97, (byte)148, (byte)89, (byte)109, (byte)62, (byte)56, (byte)81, (byte)25, (byte)14, (byte)32, (byte)15, (byte)2, (byte)108, (byte)80, (byte)197, (byte)232, (byte)79, (byte)30, (byte)223, (byte)99, (byte)211, (byte)200, (byte)252, (byte)246, (byte)252, (byte)56, (byte)108, (byte)140, (byte)3, (byte)179, (byte)167, (byte)247, (byte)98, (byte)63, (byte)216, (byte)210, (byte)199, (byte)73, (byte)194, (byte)213, (byte)22, (byte)222, (byte)202, (byte)113, (byte)149, (byte)105, (byte)22, (byte)179, (byte)153, (byte)208, (byte)151, (byte)146, (byte)110, (byte)199, (byte)210, (byte)236, (byte)96, (byte)165, (byte)161, (byte)68, (byte)178, (byte)133, (byte)187, (byte)96, (byte)232, (byte)29, (byte)178, (byte)56, (byte)186, (byte)68, (byte)4, (byte)137, (byte)141, (byte)243, (byte)91, (byte)154, (byte)19, (byte)113, (byte)208, (byte)157, (byte)9, (byte)208, (byte)104, (byte)17, (byte)59, (byte)130, (byte)195, (byte)188, (byte)45, (byte)180, (byte)79, (byte)189, (byte)246, (byte)139, (byte)76, (byte)185, (byte)95, (byte)49, (byte)93, (byte)142, (byte)122, (byte)197, (byte)207, (byte)219, (byte)127, (byte)85, (byte)94, (byte)241, (byte)252, (byte)138, (byte)117, (byte)0, (byte)154}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)44017);
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.data__SET(new byte[] {(byte)102, (byte)79, (byte)57, (byte)167, (byte)107, (byte)122, (byte)155, (byte)68, (byte)115, (byte)197, (byte)68, (byte)144, (byte)163, (byte)153, (byte)91, (byte)255, (byte)119, (byte)208, (byte)145, (byte)56, (byte)170, (byte)110, (byte)134, (byte)246, (byte)17, (byte)91, (byte)86, (byte)70, (byte)67, (byte)74, (byte)0, (byte)220, (byte)247, (byte)119, (byte)24, (byte)154, (byte)239, (byte)64, (byte)128, (byte)198, (byte)171, (byte)64, (byte)245, (byte)247, (byte)73, (byte)227, (byte)13, (byte)147, (byte)93, (byte)44, (byte)55, (byte)199, (byte)95, (byte)73, (byte)178, (byte)194, (byte)220, (byte)154, (byte)117, (byte)29, (byte)92, (byte)58, (byte)23, (byte)37, (byte)46, (byte)45, (byte)182, (byte)78, (byte)78, (byte)76, (byte)141, (byte)165, (byte)30, (byte)118, (byte)233, (byte)84, (byte)70, (byte)100, (byte)156, (byte)110, (byte)160, (byte)81, (byte)81, (byte)44, (byte)130, (byte)63, (byte)202, (byte)4, (byte)93, (byte)174, (byte)144, (byte)209, (byte)211, (byte)222, (byte)110, (byte)225, (byte)191, (byte)126, (byte)205, (byte)227, (byte)249, (byte)17, (byte)191, (byte)206, (byte)71, (byte)167, (byte)58, (byte)148, (byte)81, (byte)143, (byte)129, (byte)82, (byte)231, (byte)167, (byte)250, (byte)184, (byte)124, (byte)134, (byte)199, (byte)103, (byte)68, (byte)179, (byte)81, (byte)235, (byte)75, (byte)179, (byte)92, (byte)130, (byte)208, (byte)157, (byte)150, (byte)194, (byte)235, (byte)230, (byte)37, (byte)18, (byte)148, (byte)88, (byte)36, (byte)175, (byte)97, (byte)148, (byte)89, (byte)109, (byte)62, (byte)56, (byte)81, (byte)25, (byte)14, (byte)32, (byte)15, (byte)2, (byte)108, (byte)80, (byte)197, (byte)232, (byte)79, (byte)30, (byte)223, (byte)99, (byte)211, (byte)200, (byte)252, (byte)246, (byte)252, (byte)56, (byte)108, (byte)140, (byte)3, (byte)179, (byte)167, (byte)247, (byte)98, (byte)63, (byte)216, (byte)210, (byte)199, (byte)73, (byte)194, (byte)213, (byte)22, (byte)222, (byte)202, (byte)113, (byte)149, (byte)105, (byte)22, (byte)179, (byte)153, (byte)208, (byte)151, (byte)146, (byte)110, (byte)199, (byte)210, (byte)236, (byte)96, (byte)165, (byte)161, (byte)68, (byte)178, (byte)133, (byte)187, (byte)96, (byte)232, (byte)29, (byte)178, (byte)56, (byte)186, (byte)68, (byte)4, (byte)137, (byte)141, (byte)243, (byte)91, (byte)154, (byte)19, (byte)113, (byte)208, (byte)157, (byte)9, (byte)208, (byte)104, (byte)17, (byte)59, (byte)130, (byte)195, (byte)188, (byte)45, (byte)180, (byte)79, (byte)189, (byte)246, (byte)139, (byte)76, (byte)185, (byte)95, (byte)49, (byte)93, (byte)142, (byte)122, (byte)197, (byte)207, (byte)219, (byte)127, (byte)85, (byte)94, (byte)241, (byte)252, (byte)138, (byte)117, (byte)0, (byte)154}, 0) ;
            p131.seqnr = (ushort)(ushort)44017;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2367284703U);
                Debug.Assert(pack.orientation == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_315);
                Debug.Assert(pack.max_distance == (ushort)(ushort)6146);
                Debug.Assert(pack.id == (byte)(byte)104);
                Debug.Assert(pack.min_distance == (ushort)(ushort)35865);
                Debug.Assert(pack.type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
                Debug.Assert(pack.covariance == (byte)(byte)154);
                Debug.Assert(pack.current_distance == (ushort)(ushort)59638);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.min_distance = (ushort)(ushort)35865;
            p132.covariance = (byte)(byte)154;
            p132.id = (byte)(byte)104;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_315;
            p132.max_distance = (ushort)(ushort)6146;
            p132.current_distance = (ushort)(ushort)59638;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p132.time_boot_ms = (uint)2367284703U;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)1529576683);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)58393);
                Debug.Assert(pack.mask == (ulong)6036489676291836932L);
                Debug.Assert(pack.lat == (int)1949016301);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.lat = (int)1949016301;
            p133.lon = (int)1529576683;
            p133.mask = (ulong)6036489676291836932L;
            p133.grid_spacing = (ushort)(ushort)58393;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gridbit == (byte)(byte)54);
                Debug.Assert(pack.lon == (int) -766005421);
                Debug.Assert(pack.lat == (int)1665124531);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)35807);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -8521, (short) -12672, (short)12558, (short) -25201, (short)18381, (short)21766, (short)17063, (short) -14958, (short) -5161, (short) -23812, (short) -16642, (short)8529, (short)6867, (short) -2162, (short) -26184, (short) -93}));
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.lat = (int)1665124531;
            p134.gridbit = (byte)(byte)54;
            p134.grid_spacing = (ushort)(ushort)35807;
            p134.data__SET(new short[] {(short) -8521, (short) -12672, (short)12558, (short) -25201, (short)18381, (short)21766, (short)17063, (short) -14958, (short) -5161, (short) -23812, (short) -16642, (short)8529, (short)6867, (short) -2162, (short) -26184, (short) -93}, 0) ;
            p134.lon = (int) -766005421;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)882367395);
                Debug.Assert(pack.lon == (int) -1230435096);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lat = (int)882367395;
            p135.lon = (int) -1230435096;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.terrain_height == (float) -9.76052E37F);
                Debug.Assert(pack.lat == (int) -1683983960);
                Debug.Assert(pack.pending == (ushort)(ushort)53075);
                Debug.Assert(pack.loaded == (ushort)(ushort)8655);
                Debug.Assert(pack.lon == (int) -211014138);
                Debug.Assert(pack.current_height == (float)2.6065332E38F);
                Debug.Assert(pack.spacing == (ushort)(ushort)41501);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.lon = (int) -211014138;
            p136.lat = (int) -1683983960;
            p136.current_height = (float)2.6065332E38F;
            p136.pending = (ushort)(ushort)53075;
            p136.spacing = (ushort)(ushort)41501;
            p136.loaded = (ushort)(ushort)8655;
            p136.terrain_height = (float) -9.76052E37F;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -20764);
                Debug.Assert(pack.press_abs == (float)8.598876E37F);
                Debug.Assert(pack.time_boot_ms == (uint)4276540166U);
                Debug.Assert(pack.press_diff == (float) -2.3255306E38F);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.press_abs = (float)8.598876E37F;
            p137.time_boot_ms = (uint)4276540166U;
            p137.temperature = (short)(short) -20764;
            p137.press_diff = (float) -2.3255306E38F;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)1.4651805E38F);
                Debug.Assert(pack.time_usec == (ulong)8467502328440061600L);
                Debug.Assert(pack.y == (float)2.6916354E38F);
                Debug.Assert(pack.z == (float)4.6061504E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.1444085E38F, 2.3274688E38F, 9.718431E37F, -3.3164908E38F}));
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.x = (float)1.4651805E38F;
            p138.q_SET(new float[] {-3.1444085E38F, 2.3274688E38F, 9.718431E37F, -3.3164908E38F}, 0) ;
            p138.time_usec = (ulong)8467502328440061600L;
            p138.y = (float)2.6916354E38F;
            p138.z = (float)4.6061504E37F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.742E38F, -3.3863724E38F, 3.1701167E38F, 8.2352967E37F, 1.3872917E38F, 2.0054265E38F, -8.225412E37F, -3.4923287E37F}));
                Debug.Assert(pack.target_component == (byte)(byte)141);
                Debug.Assert(pack.group_mlx == (byte)(byte)145);
                Debug.Assert(pack.target_system == (byte)(byte)8);
                Debug.Assert(pack.time_usec == (ulong)2056362782514957801L);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.target_component = (byte)(byte)141;
            p139.time_usec = (ulong)2056362782514957801L;
            p139.controls_SET(new float[] {-1.742E38F, -3.3863724E38F, 3.1701167E38F, 8.2352967E37F, 1.3872917E38F, 2.0054265E38F, -8.225412E37F, -3.4923287E37F}, 0) ;
            p139.group_mlx = (byte)(byte)145;
            p139.target_system = (byte)(byte)8;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-2.6906308E38F, -3.003124E38F, 1.814454E38F, 3.1182026E38F, -1.651787E38F, 2.0243066E38F, -1.0614057E38F, -1.0742019E38F}));
                Debug.Assert(pack.time_usec == (ulong)953340955625885173L);
                Debug.Assert(pack.group_mlx == (byte)(byte)100);
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.time_usec = (ulong)953340955625885173L;
            p140.group_mlx = (byte)(byte)100;
            p140.controls_SET(new float[] {-2.6906308E38F, -3.003124E38F, 1.814454E38F, 3.1182026E38F, -1.651787E38F, 2.0243066E38F, -1.0614057E38F, -1.0742019E38F}, 0) ;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_relative == (float)2.7164807E38F);
                Debug.Assert(pack.time_usec == (ulong)78206034293244810L);
                Debug.Assert(pack.altitude_amsl == (float) -1.5471864E38F);
                Debug.Assert(pack.altitude_terrain == (float)2.2748661E38F);
                Debug.Assert(pack.altitude_local == (float) -2.0187054E38F);
                Debug.Assert(pack.altitude_monotonic == (float)2.3363253E38F);
                Debug.Assert(pack.bottom_clearance == (float) -4.402461E37F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.bottom_clearance = (float) -4.402461E37F;
            p141.altitude_amsl = (float) -1.5471864E38F;
            p141.altitude_local = (float) -2.0187054E38F;
            p141.altitude_monotonic = (float)2.3363253E38F;
            p141.time_usec = (ulong)78206034293244810L;
            p141.altitude_terrain = (float)2.2748661E38F;
            p141.altitude_relative = (float)2.7164807E38F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.request_id == (byte)(byte)10);
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)223, (byte)243, (byte)39, (byte)103, (byte)120, (byte)155, (byte)44, (byte)150, (byte)104, (byte)211, (byte)60, (byte)245, (byte)165, (byte)26, (byte)30, (byte)39, (byte)79, (byte)189, (byte)100, (byte)110, (byte)211, (byte)167, (byte)96, (byte)67, (byte)244, (byte)204, (byte)183, (byte)101, (byte)206, (byte)224, (byte)9, (byte)26, (byte)76, (byte)152, (byte)91, (byte)95, (byte)78, (byte)227, (byte)120, (byte)144, (byte)224, (byte)168, (byte)252, (byte)242, (byte)179, (byte)6, (byte)220, (byte)185, (byte)185, (byte)143, (byte)241, (byte)250, (byte)187, (byte)97, (byte)43, (byte)1, (byte)57, (byte)48, (byte)166, (byte)37, (byte)0, (byte)140, (byte)41, (byte)37, (byte)112, (byte)81, (byte)192, (byte)74, (byte)113, (byte)224, (byte)130, (byte)152, (byte)161, (byte)134, (byte)4, (byte)0, (byte)110, (byte)31, (byte)131, (byte)155, (byte)250, (byte)226, (byte)60, (byte)66, (byte)71, (byte)89, (byte)109, (byte)213, (byte)70, (byte)118, (byte)111, (byte)250, (byte)116, (byte)197, (byte)182, (byte)83, (byte)0, (byte)51, (byte)147, (byte)84, (byte)153, (byte)248, (byte)128, (byte)97, (byte)5, (byte)207, (byte)20, (byte)225, (byte)122, (byte)71, (byte)197, (byte)149, (byte)77, (byte)196, (byte)191, (byte)61, (byte)110, (byte)65, (byte)2, (byte)128}));
                Debug.Assert(pack.uri_type == (byte)(byte)77);
                Debug.Assert(pack.transfer_type == (byte)(byte)166);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)86, (byte)126, (byte)188, (byte)252, (byte)113, (byte)156, (byte)7, (byte)209, (byte)84, (byte)102, (byte)249, (byte)93, (byte)220, (byte)183, (byte)11, (byte)109, (byte)14, (byte)88, (byte)59, (byte)113, (byte)126, (byte)28, (byte)80, (byte)84, (byte)252, (byte)18, (byte)72, (byte)63, (byte)83, (byte)93, (byte)33, (byte)5, (byte)134, (byte)20, (byte)209, (byte)194, (byte)5, (byte)178, (byte)223, (byte)33, (byte)169, (byte)32, (byte)80, (byte)170, (byte)59, (byte)133, (byte)59, (byte)128, (byte)248, (byte)108, (byte)124, (byte)246, (byte)255, (byte)191, (byte)118, (byte)106, (byte)217, (byte)138, (byte)148, (byte)34, (byte)28, (byte)77, (byte)47, (byte)92, (byte)248, (byte)59, (byte)9, (byte)171, (byte)97, (byte)86, (byte)35, (byte)49, (byte)253, (byte)57, (byte)196, (byte)113, (byte)163, (byte)192, (byte)107, (byte)60, (byte)105, (byte)23, (byte)148, (byte)159, (byte)18, (byte)159, (byte)109, (byte)243, (byte)146, (byte)11, (byte)88, (byte)80, (byte)137, (byte)176, (byte)100, (byte)103, (byte)166, (byte)201, (byte)67, (byte)151, (byte)194, (byte)117, (byte)118, (byte)28, (byte)62, (byte)243, (byte)244, (byte)101, (byte)228, (byte)217, (byte)177, (byte)26, (byte)209, (byte)140, (byte)17, (byte)173, (byte)115, (byte)217, (byte)103, (byte)65}));
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.transfer_type = (byte)(byte)166;
            p142.storage_SET(new byte[] {(byte)223, (byte)243, (byte)39, (byte)103, (byte)120, (byte)155, (byte)44, (byte)150, (byte)104, (byte)211, (byte)60, (byte)245, (byte)165, (byte)26, (byte)30, (byte)39, (byte)79, (byte)189, (byte)100, (byte)110, (byte)211, (byte)167, (byte)96, (byte)67, (byte)244, (byte)204, (byte)183, (byte)101, (byte)206, (byte)224, (byte)9, (byte)26, (byte)76, (byte)152, (byte)91, (byte)95, (byte)78, (byte)227, (byte)120, (byte)144, (byte)224, (byte)168, (byte)252, (byte)242, (byte)179, (byte)6, (byte)220, (byte)185, (byte)185, (byte)143, (byte)241, (byte)250, (byte)187, (byte)97, (byte)43, (byte)1, (byte)57, (byte)48, (byte)166, (byte)37, (byte)0, (byte)140, (byte)41, (byte)37, (byte)112, (byte)81, (byte)192, (byte)74, (byte)113, (byte)224, (byte)130, (byte)152, (byte)161, (byte)134, (byte)4, (byte)0, (byte)110, (byte)31, (byte)131, (byte)155, (byte)250, (byte)226, (byte)60, (byte)66, (byte)71, (byte)89, (byte)109, (byte)213, (byte)70, (byte)118, (byte)111, (byte)250, (byte)116, (byte)197, (byte)182, (byte)83, (byte)0, (byte)51, (byte)147, (byte)84, (byte)153, (byte)248, (byte)128, (byte)97, (byte)5, (byte)207, (byte)20, (byte)225, (byte)122, (byte)71, (byte)197, (byte)149, (byte)77, (byte)196, (byte)191, (byte)61, (byte)110, (byte)65, (byte)2, (byte)128}, 0) ;
            p142.uri_type = (byte)(byte)77;
            p142.uri_SET(new byte[] {(byte)86, (byte)126, (byte)188, (byte)252, (byte)113, (byte)156, (byte)7, (byte)209, (byte)84, (byte)102, (byte)249, (byte)93, (byte)220, (byte)183, (byte)11, (byte)109, (byte)14, (byte)88, (byte)59, (byte)113, (byte)126, (byte)28, (byte)80, (byte)84, (byte)252, (byte)18, (byte)72, (byte)63, (byte)83, (byte)93, (byte)33, (byte)5, (byte)134, (byte)20, (byte)209, (byte)194, (byte)5, (byte)178, (byte)223, (byte)33, (byte)169, (byte)32, (byte)80, (byte)170, (byte)59, (byte)133, (byte)59, (byte)128, (byte)248, (byte)108, (byte)124, (byte)246, (byte)255, (byte)191, (byte)118, (byte)106, (byte)217, (byte)138, (byte)148, (byte)34, (byte)28, (byte)77, (byte)47, (byte)92, (byte)248, (byte)59, (byte)9, (byte)171, (byte)97, (byte)86, (byte)35, (byte)49, (byte)253, (byte)57, (byte)196, (byte)113, (byte)163, (byte)192, (byte)107, (byte)60, (byte)105, (byte)23, (byte)148, (byte)159, (byte)18, (byte)159, (byte)109, (byte)243, (byte)146, (byte)11, (byte)88, (byte)80, (byte)137, (byte)176, (byte)100, (byte)103, (byte)166, (byte)201, (byte)67, (byte)151, (byte)194, (byte)117, (byte)118, (byte)28, (byte)62, (byte)243, (byte)244, (byte)101, (byte)228, (byte)217, (byte)177, (byte)26, (byte)209, (byte)140, (byte)17, (byte)173, (byte)115, (byte)217, (byte)103, (byte)65}, 0) ;
            p142.request_id = (byte)(byte)10;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float)8.808432E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3878295833U);
                Debug.Assert(pack.press_abs == (float) -2.9394235E38F);
                Debug.Assert(pack.temperature == (short)(short) -7520);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.temperature = (short)(short) -7520;
            p143.press_diff = (float)8.808432E37F;
            p143.time_boot_ms = (uint)3878295833U;
            p143.press_abs = (float) -2.9394235E38F;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)479860927);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {-1.8636723E38F, 2.4659984E38F, 3.0261491E38F}));
                Debug.Assert(pack.custom_state == (ulong)4173224764499218186L);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {3.0271571E38F, -6.6055204E37F, 3.2992286E38F}));
                Debug.Assert(pack.timestamp == (ulong)5319829714032909972L);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {2.7806322E38F, 1.8125665E38F, 2.1986523E38F}));
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-1.1731067E38F, 2.0982226E38F, -1.2518564E38F, 6.2557487E37F}));
                Debug.Assert(pack.alt == (float)1.22473E38F);
                Debug.Assert(pack.est_capabilities == (byte)(byte)203);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {-2.8226863E38F, -2.0098978E38F, -3.2533368E38F}));
                Debug.Assert(pack.lat == (int) -1473425868);
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.lat = (int) -1473425868;
            p144.position_cov_SET(new float[] {-2.8226863E38F, -2.0098978E38F, -3.2533368E38F}, 0) ;
            p144.vel_SET(new float[] {3.0271571E38F, -6.6055204E37F, 3.2992286E38F}, 0) ;
            p144.acc_SET(new float[] {-1.8636723E38F, 2.4659984E38F, 3.0261491E38F}, 0) ;
            p144.est_capabilities = (byte)(byte)203;
            p144.lon = (int)479860927;
            p144.rates_SET(new float[] {2.7806322E38F, 1.8125665E38F, 2.1986523E38F}, 0) ;
            p144.timestamp = (ulong)5319829714032909972L;
            p144.alt = (float)1.22473E38F;
            p144.attitude_q_SET(new float[] {-1.1731067E38F, 2.0982226E38F, -1.2518564E38F, 6.2557487E37F}, 0) ;
            p144.custom_state = (ulong)4173224764499218186L;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x_vel == (float)3.1593139E38F);
                Debug.Assert(pack.yaw_rate == (float)2.4870092E38F);
                Debug.Assert(pack.x_acc == (float) -1.8368472E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {2.7555885E38F, 2.6505442E38F, 2.8516447E38F}));
                Debug.Assert(pack.time_usec == (ulong)7284569024758441484L);
                Debug.Assert(pack.z_pos == (float)2.3967143E38F);
                Debug.Assert(pack.roll_rate == (float) -3.1827212E38F);
                Debug.Assert(pack.y_vel == (float) -2.361752E38F);
                Debug.Assert(pack.y_pos == (float) -1.2233772E38F);
                Debug.Assert(pack.z_vel == (float)2.8765036E37F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {1.6172449E38F, 1.6538054E38F, 5.945928E37F}));
                Debug.Assert(pack.airspeed == (float) -2.5080715E38F);
                Debug.Assert(pack.pitch_rate == (float) -1.2424188E38F);
                Debug.Assert(pack.y_acc == (float) -1.3446541E38F);
                Debug.Assert(pack.x_pos == (float)2.5274306E38F);
                Debug.Assert(pack.z_acc == (float)1.8931678E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-3.2504742E37F, 2.1407014E38F, -2.426472E38F, 1.3890591E38F}));
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.x_pos = (float)2.5274306E38F;
            p146.z_vel = (float)2.8765036E37F;
            p146.roll_rate = (float) -3.1827212E38F;
            p146.z_pos = (float)2.3967143E38F;
            p146.x_acc = (float) -1.8368472E38F;
            p146.airspeed = (float) -2.5080715E38F;
            p146.y_pos = (float) -1.2233772E38F;
            p146.vel_variance_SET(new float[] {2.7555885E38F, 2.6505442E38F, 2.8516447E38F}, 0) ;
            p146.pos_variance_SET(new float[] {1.6172449E38F, 1.6538054E38F, 5.945928E37F}, 0) ;
            p146.y_vel = (float) -2.361752E38F;
            p146.pitch_rate = (float) -1.2424188E38F;
            p146.y_acc = (float) -1.3446541E38F;
            p146.time_usec = (ulong)7284569024758441484L;
            p146.z_acc = (float)1.8931678E38F;
            p146.yaw_rate = (float)2.4870092E38F;
            p146.q_SET(new float[] {-3.2504742E37F, 2.1407014E38F, -2.426472E38F, 1.3890591E38F}, 0) ;
            p146.x_vel = (float)3.1593139E38F;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (byte)(byte)225);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)31018, (ushort)61508, (ushort)50558, (ushort)32320, (ushort)24212, (ushort)47553, (ushort)62871, (ushort)5764, (ushort)42162, (ushort)39259}));
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)64);
                Debug.Assert(pack.temperature == (short)(short) -31655);
                Debug.Assert(pack.battery_function == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION);
                Debug.Assert(pack.type == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION);
                Debug.Assert(pack.current_battery == (short)(short)3772);
                Debug.Assert(pack.current_consumed == (int) -421879735);
                Debug.Assert(pack.energy_consumed == (int)1525921390);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.id = (byte)(byte)225;
            p147.battery_remaining = (sbyte)(sbyte)64;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION;
            p147.current_consumed = (int) -421879735;
            p147.current_battery = (short)(short)3772;
            p147.energy_consumed = (int)1525921390;
            p147.voltages_SET(new ushort[] {(ushort)31018, (ushort)61508, (ushort)50558, (ushort)32320, (ushort)24212, (ushort)47553, (ushort)62871, (ushort)5764, (ushort)42162, (ushort)39259}, 0) ;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION;
            p147.temperature = (short)(short) -31655;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT));
                Debug.Assert(pack.board_version == (uint)2723581931U);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)23, (byte)70, (byte)150, (byte)181, (byte)68, (byte)96, (byte)106, (byte)49, (byte)130, (byte)126, (byte)204, (byte)213, (byte)109, (byte)218, (byte)252, (byte)68, (byte)16, (byte)116}));
                Debug.Assert(pack.product_id == (ushort)(ushort)26711);
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)248, (byte)53, (byte)224, (byte)73, (byte)113, (byte)234, (byte)139, (byte)60}));
                Debug.Assert(pack.os_sw_version == (uint)2376245597U);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)36791);
                Debug.Assert(pack.flight_sw_version == (uint)2652680528U);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)184, (byte)86, (byte)8, (byte)60, (byte)152, (byte)217, (byte)124, (byte)143}));
                Debug.Assert(pack.uid == (ulong)1042906607459135010L);
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)190, (byte)97, (byte)237, (byte)159, (byte)192, (byte)192, (byte)88, (byte)81}));
                Debug.Assert(pack.middleware_sw_version == (uint)4287536354U);
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.product_id = (ushort)(ushort)26711;
            p148.os_custom_version_SET(new byte[] {(byte)190, (byte)97, (byte)237, (byte)159, (byte)192, (byte)192, (byte)88, (byte)81}, 0) ;
            p148.uid = (ulong)1042906607459135010L;
            p148.uid2_SET(new byte[] {(byte)23, (byte)70, (byte)150, (byte)181, (byte)68, (byte)96, (byte)106, (byte)49, (byte)130, (byte)126, (byte)204, (byte)213, (byte)109, (byte)218, (byte)252, (byte)68, (byte)16, (byte)116}, 0, PH) ;
            p148.os_sw_version = (uint)2376245597U;
            p148.vendor_id = (ushort)(ushort)36791;
            p148.flight_custom_version_SET(new byte[] {(byte)248, (byte)53, (byte)224, (byte)73, (byte)113, (byte)234, (byte)139, (byte)60}, 0) ;
            p148.middleware_sw_version = (uint)4287536354U;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT);
            p148.middleware_custom_version_SET(new byte[] {(byte)184, (byte)86, (byte)8, (byte)60, (byte)152, (byte)217, (byte)124, (byte)143}, 0) ;
            p148.flight_sw_version = (uint)2652680528U;
            p148.board_version = (uint)2723581931U;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x_TRY(ph) == (float)9.704979E36F);
                Debug.Assert(pack.size_x == (float)2.410667E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.y_TRY(ph) == (float) -2.5509716E38F);
                Debug.Assert(pack.size_y == (float)7.8710053E37F);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {2.0089447E38F, -6.845422E37F, 2.8065062E38F, 1.29793E38F}));
                Debug.Assert(pack.distance == (float)3.0938702E38F);
                Debug.Assert(pack.z_TRY(ph) == (float) -1.0786763E38F);
                Debug.Assert(pack.target_num == (byte)(byte)61);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)91);
                Debug.Assert(pack.time_usec == (ulong)4064083978929140514L);
                Debug.Assert(pack.type == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
                Debug.Assert(pack.angle_x == (float)8.217951E37F);
                Debug.Assert(pack.angle_y == (float)2.5357317E37F);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.size_y = (float)7.8710053E37F;
            p149.frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p149.time_usec = (ulong)4064083978929140514L;
            p149.z_SET((float) -1.0786763E38F, PH) ;
            p149.x_SET((float)9.704979E36F, PH) ;
            p149.y_SET((float) -2.5509716E38F, PH) ;
            p149.position_valid_SET((byte)(byte)91, PH) ;
            p149.q_SET(new float[] {2.0089447E38F, -6.845422E37F, 2.8065062E38F, 1.29793E38F}, 0, PH) ;
            p149.target_num = (byte)(byte)61;
            p149.size_x = (float)2.410667E38F;
            p149.distance = (float)3.0938702E38F;
            p149.angle_x = (float)8.217951E37F;
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON;
            p149.angle_y = (float)2.5357317E37F;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAQ_TELEMETRY_FReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value10 == (float) -2.3387144E38F);
                Debug.Assert(pack.value5 == (float) -6.959701E37F);
                Debug.Assert(pack.value2 == (float) -2.7380718E38F);
                Debug.Assert(pack.value13 == (float)1.6244923E38F);
                Debug.Assert(pack.value16 == (float)2.937373E38F);
                Debug.Assert(pack.value4 == (float) -1.5015501E37F);
                Debug.Assert(pack.value18 == (float)1.0313826E38F);
                Debug.Assert(pack.value3 == (float)2.4633027E38F);
                Debug.Assert(pack.value17 == (float)7.649432E36F);
                Debug.Assert(pack.value15 == (float) -1.2193688E37F);
                Debug.Assert(pack.value11 == (float) -2.5182595E38F);
                Debug.Assert(pack.value1 == (float) -3.0208544E38F);
                Debug.Assert(pack.value9 == (float) -2.9813735E37F);
                Debug.Assert(pack.value6 == (float) -4.2053774E37F);
                Debug.Assert(pack.value19 == (float) -1.7189262E37F);
                Debug.Assert(pack.value8 == (float) -1.2348767E38F);
                Debug.Assert(pack.value20 == (float)2.4697182E38F);
                Debug.Assert(pack.Index == (ushort)(ushort)24419);
                Debug.Assert(pack.value12 == (float) -2.3215232E38F);
                Debug.Assert(pack.value7 == (float) -9.232923E37F);
                Debug.Assert(pack.value14 == (float)2.307623E38F);
            };
            GroundControl.AQ_TELEMETRY_F p150 = CommunicationChannel.new_AQ_TELEMETRY_F();
            PH.setPack(p150);
            p150.value5 = (float) -6.959701E37F;
            p150.value11 = (float) -2.5182595E38F;
            p150.value19 = (float) -1.7189262E37F;
            p150.value16 = (float)2.937373E38F;
            p150.value10 = (float) -2.3387144E38F;
            p150.value13 = (float)1.6244923E38F;
            p150.value18 = (float)1.0313826E38F;
            p150.value17 = (float)7.649432E36F;
            p150.value6 = (float) -4.2053774E37F;
            p150.value12 = (float) -2.3215232E38F;
            p150.value8 = (float) -1.2348767E38F;
            p150.Index = (ushort)(ushort)24419;
            p150.value2 = (float) -2.7380718E38F;
            p150.value7 = (float) -9.232923E37F;
            p150.value14 = (float)2.307623E38F;
            p150.value15 = (float) -1.2193688E37F;
            p150.value1 = (float) -3.0208544E38F;
            p150.value3 = (float)2.4633027E38F;
            p150.value4 = (float) -1.5015501E37F;
            p150.value20 = (float)2.4697182E38F;
            p150.value9 = (float) -2.9813735E37F;
            CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAQ_ESC_TELEMETRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (byte)(byte)87);
                Debug.Assert(pack.num_in_seq == (byte)(byte)80);
                Debug.Assert(pack.data1.SequenceEqual(new uint[] {1234771093U, 4125426596U, 2167610909U, 2180438312U}));
                Debug.Assert(pack.escid.SequenceEqual(new byte[] {(byte)48, (byte)145, (byte)238, (byte)60}));
                Debug.Assert(pack.data0.SequenceEqual(new uint[] {1236962832U, 799720716U, 2874379180U, 2447791058U}));
                Debug.Assert(pack.time_boot_ms == (uint)1192893473U);
                Debug.Assert(pack.data_version.SequenceEqual(new byte[] {(byte)16, (byte)4, (byte)79, (byte)23}));
                Debug.Assert(pack.status_age.SequenceEqual(new ushort[] {(ushort)52063, (ushort)34273, (ushort)10026, (ushort)4895}));
                Debug.Assert(pack.num_motors == (byte)(byte)228);
            };
            GroundControl.AQ_ESC_TELEMETRY p152 = CommunicationChannel.new_AQ_ESC_TELEMETRY();
            PH.setPack(p152);
            p152.escid_SET(new byte[] {(byte)48, (byte)145, (byte)238, (byte)60}, 0) ;
            p152.num_in_seq = (byte)(byte)80;
            p152.status_age_SET(new ushort[] {(ushort)52063, (ushort)34273, (ushort)10026, (ushort)4895}, 0) ;
            p152.num_motors = (byte)(byte)228;
            p152.data0_SET(new uint[] {1236962832U, 799720716U, 2874379180U, 2447791058U}, 0) ;
            p152.seq = (byte)(byte)87;
            p152.data_version_SET(new byte[] {(byte)16, (byte)4, (byte)79, (byte)23}, 0) ;
            p152.data1_SET(new uint[] {1234771093U, 4125426596U, 2167610909U, 2180438312U}, 0) ;
            p152.time_boot_ms = (uint)1192893473U;
            CommunicationChannel.instance.send(p152);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pos_horiz_accuracy == (float) -6.1431793E37F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ));
                Debug.Assert(pack.pos_vert_accuracy == (float)1.1616468E38F);
                Debug.Assert(pack.vel_ratio == (float) -1.0096244E38F);
                Debug.Assert(pack.mag_ratio == (float)5.6325823E37F);
                Debug.Assert(pack.tas_ratio == (float)8.068164E37F);
                Debug.Assert(pack.hagl_ratio == (float)4.256159E37F);
                Debug.Assert(pack.pos_horiz_ratio == (float) -3.3597818E38F);
                Debug.Assert(pack.pos_vert_ratio == (float) -1.963807E38F);
                Debug.Assert(pack.time_usec == (ulong)7047261961192562642L);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.pos_vert_accuracy = (float)1.1616468E38F;
            p230.pos_vert_ratio = (float) -1.963807E38F;
            p230.time_usec = (ulong)7047261961192562642L;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ);
            p230.pos_horiz_accuracy = (float) -6.1431793E37F;
            p230.hagl_ratio = (float)4.256159E37F;
            p230.mag_ratio = (float)5.6325823E37F;
            p230.tas_ratio = (float)8.068164E37F;
            p230.vel_ratio = (float) -1.0096244E38F;
            p230.pos_horiz_ratio = (float) -3.3597818E38F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wind_y == (float) -6.9483595E37F);
                Debug.Assert(pack.var_horiz == (float)2.487185E38F);
                Debug.Assert(pack.wind_x == (float)2.960481E38F);
                Debug.Assert(pack.wind_z == (float)3.1761947E38F);
                Debug.Assert(pack.time_usec == (ulong)3899577175839220080L);
                Debug.Assert(pack.wind_alt == (float) -1.524427E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -1.6935563E38F);
                Debug.Assert(pack.var_vert == (float) -1.014527E37F);
                Debug.Assert(pack.vert_accuracy == (float)2.2640276E38F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.wind_alt = (float) -1.524427E38F;
            p231.wind_z = (float)3.1761947E38F;
            p231.var_vert = (float) -1.014527E37F;
            p231.horiz_accuracy = (float) -1.6935563E38F;
            p231.time_usec = (ulong)3899577175839220080L;
            p231.var_horiz = (float)2.487185E38F;
            p231.wind_y = (float) -6.9483595E37F;
            p231.wind_x = (float)2.960481E38F;
            p231.vert_accuracy = (float)2.2640276E38F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hdop == (float)4.2885994E37F);
                Debug.Assert(pack.speed_accuracy == (float)2.5702833E38F);
                Debug.Assert(pack.vdop == (float)1.4771855E38F);
                Debug.Assert(pack.fix_type == (byte)(byte)106);
                Debug.Assert(pack.gps_id == (byte)(byte)95);
                Debug.Assert(pack.satellites_visible == (byte)(byte)51);
                Debug.Assert(pack.alt == (float) -2.7012555E38F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT));
                Debug.Assert(pack.ve == (float)3.2602704E38F);
                Debug.Assert(pack.lon == (int) -975512370);
                Debug.Assert(pack.vert_accuracy == (float)3.3486268E38F);
                Debug.Assert(pack.time_week == (ushort)(ushort)8315);
                Debug.Assert(pack.time_week_ms == (uint)778653016U);
                Debug.Assert(pack.lat == (int)1877221707);
                Debug.Assert(pack.horiz_accuracy == (float)1.246005E38F);
                Debug.Assert(pack.vn == (float) -2.9191103E38F);
                Debug.Assert(pack.time_usec == (ulong)7283815368729150088L);
                Debug.Assert(pack.vd == (float) -1.218541E38F);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.lat = (int)1877221707;
            p232.lon = (int) -975512370;
            p232.vd = (float) -1.218541E38F;
            p232.satellites_visible = (byte)(byte)51;
            p232.vn = (float) -2.9191103E38F;
            p232.vdop = (float)1.4771855E38F;
            p232.speed_accuracy = (float)2.5702833E38F;
            p232.gps_id = (byte)(byte)95;
            p232.fix_type = (byte)(byte)106;
            p232.time_usec = (ulong)7283815368729150088L;
            p232.horiz_accuracy = (float)1.246005E38F;
            p232.vert_accuracy = (float)3.3486268E38F;
            p232.ve = (float)3.2602704E38F;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT);
            p232.alt = (float) -2.7012555E38F;
            p232.time_week = (ushort)(ushort)8315;
            p232.time_week_ms = (uint)778653016U;
            p232.hdop = (float)4.2885994E37F;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)221);
                Debug.Assert(pack.flags == (byte)(byte)103);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)103, (byte)187, (byte)237, (byte)226, (byte)162, (byte)205, (byte)90, (byte)248, (byte)106, (byte)52, (byte)46, (byte)225, (byte)189, (byte)155, (byte)139, (byte)170, (byte)117, (byte)48, (byte)217, (byte)37, (byte)82, (byte)240, (byte)195, (byte)226, (byte)221, (byte)200, (byte)60, (byte)67, (byte)113, (byte)194, (byte)79, (byte)74, (byte)7, (byte)79, (byte)73, (byte)36, (byte)105, (byte)88, (byte)9, (byte)181, (byte)146, (byte)33, (byte)165, (byte)83, (byte)195, (byte)159, (byte)104, (byte)246, (byte)81, (byte)19, (byte)245, (byte)11, (byte)55, (byte)198, (byte)216, (byte)15, (byte)162, (byte)59, (byte)195, (byte)47, (byte)151, (byte)58, (byte)134, (byte)205, (byte)250, (byte)207, (byte)145, (byte)145, (byte)65, (byte)240, (byte)170, (byte)185, (byte)131, (byte)118, (byte)145, (byte)231, (byte)25, (byte)163, (byte)101, (byte)74, (byte)110, (byte)170, (byte)56, (byte)27, (byte)167, (byte)96, (byte)81, (byte)23, (byte)106, (byte)144, (byte)132, (byte)72, (byte)241, (byte)176, (byte)102, (byte)193, (byte)27, (byte)251, (byte)185, (byte)90, (byte)32, (byte)237, (byte)137, (byte)43, (byte)5, (byte)249, (byte)39, (byte)206, (byte)77, (byte)186, (byte)142, (byte)242, (byte)241, (byte)117, (byte)169, (byte)191, (byte)144, (byte)229, (byte)224, (byte)230, (byte)217, (byte)85, (byte)97, (byte)82, (byte)253, (byte)207, (byte)203, (byte)67, (byte)220, (byte)89, (byte)88, (byte)62, (byte)207, (byte)92, (byte)95, (byte)151, (byte)191, (byte)65, (byte)245, (byte)211, (byte)111, (byte)175, (byte)194, (byte)78, (byte)102, (byte)194, (byte)50, (byte)101, (byte)225, (byte)143, (byte)146, (byte)166, (byte)4, (byte)22, (byte)25, (byte)26, (byte)121, (byte)173, (byte)84, (byte)84, (byte)255, (byte)117, (byte)102, (byte)199, (byte)61, (byte)158, (byte)193, (byte)222, (byte)196, (byte)248, (byte)245, (byte)48, (byte)243, (byte)71, (byte)213, (byte)201, (byte)176, (byte)124, (byte)160, (byte)16}));
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.len = (byte)(byte)221;
            p233.data__SET(new byte[] {(byte)103, (byte)187, (byte)237, (byte)226, (byte)162, (byte)205, (byte)90, (byte)248, (byte)106, (byte)52, (byte)46, (byte)225, (byte)189, (byte)155, (byte)139, (byte)170, (byte)117, (byte)48, (byte)217, (byte)37, (byte)82, (byte)240, (byte)195, (byte)226, (byte)221, (byte)200, (byte)60, (byte)67, (byte)113, (byte)194, (byte)79, (byte)74, (byte)7, (byte)79, (byte)73, (byte)36, (byte)105, (byte)88, (byte)9, (byte)181, (byte)146, (byte)33, (byte)165, (byte)83, (byte)195, (byte)159, (byte)104, (byte)246, (byte)81, (byte)19, (byte)245, (byte)11, (byte)55, (byte)198, (byte)216, (byte)15, (byte)162, (byte)59, (byte)195, (byte)47, (byte)151, (byte)58, (byte)134, (byte)205, (byte)250, (byte)207, (byte)145, (byte)145, (byte)65, (byte)240, (byte)170, (byte)185, (byte)131, (byte)118, (byte)145, (byte)231, (byte)25, (byte)163, (byte)101, (byte)74, (byte)110, (byte)170, (byte)56, (byte)27, (byte)167, (byte)96, (byte)81, (byte)23, (byte)106, (byte)144, (byte)132, (byte)72, (byte)241, (byte)176, (byte)102, (byte)193, (byte)27, (byte)251, (byte)185, (byte)90, (byte)32, (byte)237, (byte)137, (byte)43, (byte)5, (byte)249, (byte)39, (byte)206, (byte)77, (byte)186, (byte)142, (byte)242, (byte)241, (byte)117, (byte)169, (byte)191, (byte)144, (byte)229, (byte)224, (byte)230, (byte)217, (byte)85, (byte)97, (byte)82, (byte)253, (byte)207, (byte)203, (byte)67, (byte)220, (byte)89, (byte)88, (byte)62, (byte)207, (byte)92, (byte)95, (byte)151, (byte)191, (byte)65, (byte)245, (byte)211, (byte)111, (byte)175, (byte)194, (byte)78, (byte)102, (byte)194, (byte)50, (byte)101, (byte)225, (byte)143, (byte)146, (byte)166, (byte)4, (byte)22, (byte)25, (byte)26, (byte)121, (byte)173, (byte)84, (byte)84, (byte)255, (byte)117, (byte)102, (byte)199, (byte)61, (byte)158, (byte)193, (byte)222, (byte)196, (byte)248, (byte)245, (byte)48, (byte)243, (byte)71, (byte)213, (byte)201, (byte)176, (byte)124, (byte)160, (byte)16}, 0) ;
            p233.flags = (byte)(byte)103;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED));
                Debug.Assert(pack.wp_num == (byte)(byte)84);
                Debug.Assert(pack.battery_remaining == (byte)(byte)209);
                Debug.Assert(pack.roll == (short)(short)2630);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)16503);
                Debug.Assert(pack.altitude_amsl == (short)(short)7927);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte) - 54);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 8);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)205);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)98);
                Debug.Assert(pack.temperature == (sbyte)(sbyte) - 60);
                Debug.Assert(pack.heading_sp == (short)(short) -25262);
                Debug.Assert(pack.custom_mode == (uint)243068935U);
                Debug.Assert(pack.heading == (ushort)(ushort)51941);
                Debug.Assert(pack.gps_nsat == (byte)(byte)20);
                Debug.Assert(pack.longitude == (int) -1767017702);
                Debug.Assert(pack.failsafe == (byte)(byte)17);
                Debug.Assert(pack.airspeed == (byte)(byte)140);
                Debug.Assert(pack.pitch == (short)(short)6719);
                Debug.Assert(pack.latitude == (int) -1307748425);
                Debug.Assert(pack.groundspeed == (byte)(byte)105);
                Debug.Assert(pack.gps_fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
                Debug.Assert(pack.altitude_sp == (short)(short) -5020);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.groundspeed = (byte)(byte)105;
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
            p234.airspeed_sp = (byte)(byte)205;
            p234.climb_rate = (sbyte)(sbyte) - 8;
            p234.heading_sp = (short)(short) -25262;
            p234.altitude_sp = (short)(short) -5020;
            p234.custom_mode = (uint)243068935U;
            p234.gps_nsat = (byte)(byte)20;
            p234.latitude = (int) -1307748425;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX;
            p234.airspeed = (byte)(byte)140;
            p234.pitch = (short)(short)6719;
            p234.heading = (ushort)(ushort)51941;
            p234.altitude_amsl = (short)(short)7927;
            p234.temperature = (sbyte)(sbyte) - 60;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            p234.roll = (short)(short)2630;
            p234.battery_remaining = (byte)(byte)209;
            p234.temperature_air = (sbyte)(sbyte) - 54;
            p234.wp_distance = (ushort)(ushort)16503;
            p234.longitude = (int) -1767017702;
            p234.failsafe = (byte)(byte)17;
            p234.throttle = (sbyte)(sbyte)98;
            p234.wp_num = (byte)(byte)84;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vibration_x == (float)5.729526E37F);
                Debug.Assert(pack.clipping_1 == (uint)182198148U);
                Debug.Assert(pack.clipping_0 == (uint)3073987416U);
                Debug.Assert(pack.vibration_y == (float) -1.9807477E38F);
                Debug.Assert(pack.time_usec == (ulong)1839223983273673905L);
                Debug.Assert(pack.vibration_z == (float)6.012266E37F);
                Debug.Assert(pack.clipping_2 == (uint)4133666137U);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.vibration_y = (float) -1.9807477E38F;
            p241.clipping_1 = (uint)182198148U;
            p241.vibration_x = (float)5.729526E37F;
            p241.time_usec = (ulong)1839223983273673905L;
            p241.clipping_2 = (uint)4133666137U;
            p241.clipping_0 = (uint)3073987416U;
            p241.vibration_z = (float)6.012266E37F;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)4.6822566E37F);
                Debug.Assert(pack.approach_x == (float) -1.5743711E38F);
                Debug.Assert(pack.latitude == (int)500864139);
                Debug.Assert(pack.x == (float)5.1313295E37F);
                Debug.Assert(pack.altitude == (int) -1937193249);
                Debug.Assert(pack.longitude == (int)1338859177);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)6486098098688886346L);
                Debug.Assert(pack.y == (float)1.5767456E38F);
                Debug.Assert(pack.approach_y == (float)4.4497335E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.8491891E38F, -2.8178064E38F, 1.2461588E38F, -2.5559966E38F}));
                Debug.Assert(pack.approach_z == (float) -3.4977418E36F);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.q_SET(new float[] {-2.8491891E38F, -2.8178064E38F, 1.2461588E38F, -2.5559966E38F}, 0) ;
            p242.altitude = (int) -1937193249;
            p242.approach_x = (float) -1.5743711E38F;
            p242.z = (float)4.6822566E37F;
            p242.y = (float)1.5767456E38F;
            p242.approach_y = (float)4.4497335E37F;
            p242.x = (float)5.1313295E37F;
            p242.longitude = (int)1338859177;
            p242.time_usec_SET((ulong)6486098098688886346L, PH) ;
            p242.latitude = (int)500864139;
            p242.approach_z = (float) -3.4977418E36F;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -1.7097305E38F);
                Debug.Assert(pack.altitude == (int) -1727097962);
                Debug.Assert(pack.longitude == (int) -712640311);
                Debug.Assert(pack.approach_y == (float)1.3075684E37F);
                Debug.Assert(pack.latitude == (int) -712578500);
                Debug.Assert(pack.y == (float)2.641867E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.8182394E38F, 6.3896045E37F, 1.7528308E38F, -1.76121E38F}));
                Debug.Assert(pack.x == (float) -9.32417E37F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)1291474241773048374L);
                Debug.Assert(pack.target_system == (byte)(byte)122);
                Debug.Assert(pack.approach_z == (float) -1.7044637E38F);
                Debug.Assert(pack.approach_x == (float) -2.9858666E38F);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.approach_y = (float)1.3075684E37F;
            p243.altitude = (int) -1727097962;
            p243.y = (float)2.641867E38F;
            p243.q_SET(new float[] {2.8182394E38F, 6.3896045E37F, 1.7528308E38F, -1.76121E38F}, 0) ;
            p243.approach_x = (float) -2.9858666E38F;
            p243.target_system = (byte)(byte)122;
            p243.latitude = (int) -712578500;
            p243.approach_z = (float) -1.7044637E38F;
            p243.z = (float) -1.7097305E38F;
            p243.x = (float) -9.32417E37F;
            p243.time_usec_SET((ulong)1291474241773048374L, PH) ;
            p243.longitude = (int) -712640311;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.interval_us == (int) -1313424901);
                Debug.Assert(pack.message_id == (ushort)(ushort)43199);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int) -1313424901;
            p244.message_id = (ushort)(ushort)43199;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)2128480332);
                Debug.Assert(pack.heading == (ushort)(ushort)424);
                Debug.Assert(pack.squawk == (ushort)(ushort)25630);
                Debug.Assert(pack.altitude_type == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
                Debug.Assert(pack.altitude == (int)503708657);
                Debug.Assert(pack.ver_velocity == (short)(short) -2763);
                Debug.Assert(pack.lat == (int)1225385560);
                Debug.Assert(pack.flags == (ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE));
                Debug.Assert(pack.emitter_type == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ROTOCRAFT);
                Debug.Assert(pack.callsign_LEN(ph) == 7);
                Debug.Assert(pack.callsign_TRY(ph).Equals("ibqpslw"));
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)23893);
                Debug.Assert(pack.ICAO_address == (uint)3645438901U);
                Debug.Assert(pack.tslc == (byte)(byte)184);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.hor_velocity = (ushort)(ushort)23893;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE);
            p246.tslc = (byte)(byte)184;
            p246.lon = (int)2128480332;
            p246.lat = (int)1225385560;
            p246.callsign_SET("ibqpslw", PH) ;
            p246.heading = (ushort)(ushort)424;
            p246.ICAO_address = (uint)3645438901U;
            p246.altitude = (int)503708657;
            p246.ver_velocity = (short)(short) -2763;
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_ROTOCRAFT;
            p246.squawk = (ushort)(ushort)25630;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.id == (uint)272333644U);
                Debug.Assert(pack.src_ == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
                Debug.Assert(pack.altitude_minimum_delta == (float)2.910816E38F);
                Debug.Assert(pack.horizontal_minimum_delta == (float)8.4951204E37F);
                Debug.Assert(pack.threat_level == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
                Debug.Assert(pack.time_to_minimum_delta == (float)2.6623064E38F);
                Debug.Assert(pack.action == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY;
            p247.horizontal_minimum_delta = (float)8.4951204E37F;
            p247.altitude_minimum_delta = (float)2.910816E38F;
            p247.id = (uint)272333644U;
            p247.time_to_minimum_delta = (float)2.6623064E38F;
            p247.threat_level = MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_type == (ushort)(ushort)44513);
                Debug.Assert(pack.target_network == (byte)(byte)91);
                Debug.Assert(pack.target_system == (byte)(byte)34);
                Debug.Assert(pack.target_component == (byte)(byte)53);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)14, (byte)49, (byte)25, (byte)153, (byte)20, (byte)242, (byte)5, (byte)114, (byte)170, (byte)198, (byte)179, (byte)223, (byte)175, (byte)221, (byte)209, (byte)161, (byte)64, (byte)148, (byte)228, (byte)52, (byte)4, (byte)76, (byte)95, (byte)41, (byte)75, (byte)255, (byte)204, (byte)157, (byte)80, (byte)83, (byte)204, (byte)165, (byte)22, (byte)87, (byte)102, (byte)22, (byte)107, (byte)97, (byte)135, (byte)22, (byte)68, (byte)155, (byte)74, (byte)156, (byte)5, (byte)194, (byte)154, (byte)187, (byte)175, (byte)206, (byte)190, (byte)10, (byte)73, (byte)243, (byte)60, (byte)195, (byte)201, (byte)200, (byte)226, (byte)21, (byte)145, (byte)242, (byte)156, (byte)250, (byte)214, (byte)159, (byte)230, (byte)145, (byte)105, (byte)163, (byte)58, (byte)148, (byte)142, (byte)243, (byte)46, (byte)78, (byte)236, (byte)251, (byte)53, (byte)23, (byte)60, (byte)204, (byte)225, (byte)44, (byte)217, (byte)132, (byte)96, (byte)99, (byte)129, (byte)200, (byte)215, (byte)162, (byte)104, (byte)177, (byte)183, (byte)220, (byte)128, (byte)6, (byte)140, (byte)118, (byte)212, (byte)63, (byte)213, (byte)119, (byte)189, (byte)10, (byte)127, (byte)125, (byte)70, (byte)159, (byte)96, (byte)223, (byte)234, (byte)218, (byte)93, (byte)152, (byte)165, (byte)233, (byte)14, (byte)252, (byte)100, (byte)172, (byte)225, (byte)79, (byte)243, (byte)118, (byte)48, (byte)121, (byte)39, (byte)120, (byte)170, (byte)7, (byte)2, (byte)55, (byte)71, (byte)210, (byte)113, (byte)54, (byte)125, (byte)52, (byte)159, (byte)11, (byte)18, (byte)243, (byte)133, (byte)75, (byte)103, (byte)1, (byte)3, (byte)242, (byte)3, (byte)227, (byte)153, (byte)206, (byte)21, (byte)135, (byte)240, (byte)50, (byte)22, (byte)60, (byte)50, (byte)100, (byte)71, (byte)227, (byte)104, (byte)69, (byte)125, (byte)129, (byte)196, (byte)52, (byte)135, (byte)47, (byte)110, (byte)112, (byte)121, (byte)202, (byte)14, (byte)136, (byte)208, (byte)38, (byte)73, (byte)154, (byte)32, (byte)38, (byte)249, (byte)224, (byte)147, (byte)33, (byte)105, (byte)145, (byte)175, (byte)225, (byte)213, (byte)210, (byte)20, (byte)6, (byte)196, (byte)121, (byte)14, (byte)36, (byte)192, (byte)13, (byte)11, (byte)51, (byte)218, (byte)141, (byte)187, (byte)178, (byte)251, (byte)58, (byte)56, (byte)232, (byte)79, (byte)3, (byte)62, (byte)14, (byte)220, (byte)200, (byte)3, (byte)112, (byte)178, (byte)131, (byte)35, (byte)216, (byte)129, (byte)162, (byte)199, (byte)174, (byte)25, (byte)136, (byte)31, (byte)79, (byte)201, (byte)94, (byte)33, (byte)125, (byte)34, (byte)42, (byte)60, (byte)247, (byte)35, (byte)243, (byte)129, (byte)83, (byte)91, (byte)37, (byte)2, (byte)191, (byte)220}));
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)91;
            p248.target_system = (byte)(byte)34;
            p248.payload_SET(new byte[] {(byte)14, (byte)49, (byte)25, (byte)153, (byte)20, (byte)242, (byte)5, (byte)114, (byte)170, (byte)198, (byte)179, (byte)223, (byte)175, (byte)221, (byte)209, (byte)161, (byte)64, (byte)148, (byte)228, (byte)52, (byte)4, (byte)76, (byte)95, (byte)41, (byte)75, (byte)255, (byte)204, (byte)157, (byte)80, (byte)83, (byte)204, (byte)165, (byte)22, (byte)87, (byte)102, (byte)22, (byte)107, (byte)97, (byte)135, (byte)22, (byte)68, (byte)155, (byte)74, (byte)156, (byte)5, (byte)194, (byte)154, (byte)187, (byte)175, (byte)206, (byte)190, (byte)10, (byte)73, (byte)243, (byte)60, (byte)195, (byte)201, (byte)200, (byte)226, (byte)21, (byte)145, (byte)242, (byte)156, (byte)250, (byte)214, (byte)159, (byte)230, (byte)145, (byte)105, (byte)163, (byte)58, (byte)148, (byte)142, (byte)243, (byte)46, (byte)78, (byte)236, (byte)251, (byte)53, (byte)23, (byte)60, (byte)204, (byte)225, (byte)44, (byte)217, (byte)132, (byte)96, (byte)99, (byte)129, (byte)200, (byte)215, (byte)162, (byte)104, (byte)177, (byte)183, (byte)220, (byte)128, (byte)6, (byte)140, (byte)118, (byte)212, (byte)63, (byte)213, (byte)119, (byte)189, (byte)10, (byte)127, (byte)125, (byte)70, (byte)159, (byte)96, (byte)223, (byte)234, (byte)218, (byte)93, (byte)152, (byte)165, (byte)233, (byte)14, (byte)252, (byte)100, (byte)172, (byte)225, (byte)79, (byte)243, (byte)118, (byte)48, (byte)121, (byte)39, (byte)120, (byte)170, (byte)7, (byte)2, (byte)55, (byte)71, (byte)210, (byte)113, (byte)54, (byte)125, (byte)52, (byte)159, (byte)11, (byte)18, (byte)243, (byte)133, (byte)75, (byte)103, (byte)1, (byte)3, (byte)242, (byte)3, (byte)227, (byte)153, (byte)206, (byte)21, (byte)135, (byte)240, (byte)50, (byte)22, (byte)60, (byte)50, (byte)100, (byte)71, (byte)227, (byte)104, (byte)69, (byte)125, (byte)129, (byte)196, (byte)52, (byte)135, (byte)47, (byte)110, (byte)112, (byte)121, (byte)202, (byte)14, (byte)136, (byte)208, (byte)38, (byte)73, (byte)154, (byte)32, (byte)38, (byte)249, (byte)224, (byte)147, (byte)33, (byte)105, (byte)145, (byte)175, (byte)225, (byte)213, (byte)210, (byte)20, (byte)6, (byte)196, (byte)121, (byte)14, (byte)36, (byte)192, (byte)13, (byte)11, (byte)51, (byte)218, (byte)141, (byte)187, (byte)178, (byte)251, (byte)58, (byte)56, (byte)232, (byte)79, (byte)3, (byte)62, (byte)14, (byte)220, (byte)200, (byte)3, (byte)112, (byte)178, (byte)131, (byte)35, (byte)216, (byte)129, (byte)162, (byte)199, (byte)174, (byte)25, (byte)136, (byte)31, (byte)79, (byte)201, (byte)94, (byte)33, (byte)125, (byte)34, (byte)42, (byte)60, (byte)247, (byte)35, (byte)243, (byte)129, (byte)83, (byte)91, (byte)37, (byte)2, (byte)191, (byte)220}, 0) ;
            p248.target_component = (byte)(byte)53;
            p248.message_type = (ushort)(ushort)44513;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte)114, (sbyte)16, (sbyte) - 1, (sbyte)46, (sbyte) - 33, (sbyte)61, (sbyte) - 81, (sbyte) - 97, (sbyte) - 59, (sbyte) - 127, (sbyte)110, (sbyte)108, (sbyte)73, (sbyte)108, (sbyte) - 82, (sbyte)125, (sbyte) - 98, (sbyte) - 96, (sbyte)32, (sbyte)10, (sbyte)19, (sbyte)25, (sbyte) - 9, (sbyte) - 26, (sbyte) - 25, (sbyte)58, (sbyte)108, (sbyte) - 16, (sbyte)82, (sbyte) - 7, (sbyte)10, (sbyte)12}));
                Debug.Assert(pack.ver == (byte)(byte)186);
                Debug.Assert(pack.type == (byte)(byte)217);
                Debug.Assert(pack.address == (ushort)(ushort)62);
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.address = (ushort)(ushort)62;
            p249.type = (byte)(byte)217;
            p249.value_SET(new sbyte[] {(sbyte)114, (sbyte)16, (sbyte) - 1, (sbyte)46, (sbyte) - 33, (sbyte)61, (sbyte) - 81, (sbyte) - 97, (sbyte) - 59, (sbyte) - 127, (sbyte)110, (sbyte)108, (sbyte)73, (sbyte)108, (sbyte) - 82, (sbyte)125, (sbyte) - 98, (sbyte) - 96, (sbyte)32, (sbyte)10, (sbyte)19, (sbyte)25, (sbyte) - 9, (sbyte) - 26, (sbyte) - 25, (sbyte)58, (sbyte)108, (sbyte) - 16, (sbyte)82, (sbyte) - 7, (sbyte)10, (sbyte)12}, 0) ;
            p249.ver = (byte)(byte)186;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)3.1948846E38F);
                Debug.Assert(pack.y == (float)7.106929E37F);
                Debug.Assert(pack.z == (float)3.1148203E37F);
                Debug.Assert(pack.time_usec == (ulong)5754775642922395113L);
                Debug.Assert(pack.name_LEN(ph) == 10);
                Debug.Assert(pack.name_TRY(ph).Equals("dwkgculKEQ"));
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.x = (float)3.1948846E38F;
            p250.time_usec = (ulong)5754775642922395113L;
            p250.name_SET("dwkgculKEQ", PH) ;
            p250.y = (float)7.106929E37F;
            p250.z = (float)3.1148203E37F;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 4);
                Debug.Assert(pack.name_TRY(ph).Equals("ggfz"));
                Debug.Assert(pack.value == (float) -7.623425E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3632867003U);
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.name_SET("ggfz", PH) ;
            p251.value = (float) -7.623425E37F;
            p251.time_boot_ms = (uint)3632867003U;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 9);
                Debug.Assert(pack.name_TRY(ph).Equals("vhBpiqFns"));
                Debug.Assert(pack.value == (int) -1152894328);
                Debug.Assert(pack.time_boot_ms == (uint)3359936765U);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)3359936765U;
            p252.value = (int) -1152894328;
            p252.name_SET("vhBpiqFns", PH) ;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.severity == MAV_SEVERITY.MAV_SEVERITY_DEBUG);
                Debug.Assert(pack.text_LEN(ph) == 21);
                Debug.Assert(pack.text_TRY(ph).Equals("umzqqejlejbxluibbqsta"));
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("umzqqejlejbxluibbqsta", PH) ;
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_DEBUG;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2435469124U);
                Debug.Assert(pack.value == (float) -2.7108962E38F);
                Debug.Assert(pack.ind == (byte)(byte)57);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.time_boot_ms = (uint)2435469124U;
            p254.value = (float) -2.7108962E38F;
            p254.ind = (byte)(byte)57;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)133);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)27, (byte)50, (byte)6, (byte)220, (byte)20, (byte)128, (byte)232, (byte)34, (byte)47, (byte)70, (byte)3, (byte)16, (byte)255, (byte)167, (byte)247, (byte)17, (byte)130, (byte)193, (byte)131, (byte)72, (byte)121, (byte)36, (byte)165, (byte)127, (byte)127, (byte)28, (byte)162, (byte)33, (byte)68, (byte)35, (byte)115, (byte)50}));
                Debug.Assert(pack.target_system == (byte)(byte)237);
                Debug.Assert(pack.initial_timestamp == (ulong)7398063129175063624L);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)237;
            p256.secret_key_SET(new byte[] {(byte)27, (byte)50, (byte)6, (byte)220, (byte)20, (byte)128, (byte)232, (byte)34, (byte)47, (byte)70, (byte)3, (byte)16, (byte)255, (byte)167, (byte)247, (byte)17, (byte)130, (byte)193, (byte)131, (byte)72, (byte)121, (byte)36, (byte)165, (byte)127, (byte)127, (byte)28, (byte)162, (byte)33, (byte)68, (byte)35, (byte)115, (byte)50}, 0) ;
            p256.initial_timestamp = (ulong)7398063129175063624L;
            p256.target_component = (byte)(byte)133;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.state == (byte)(byte)13);
                Debug.Assert(pack.time_boot_ms == (uint)2894331619U);
                Debug.Assert(pack.last_change_ms == (uint)4174657459U);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)2894331619U;
            p257.last_change_ms = (uint)4174657459U;
            p257.state = (byte)(byte)13;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tune_LEN(ph) == 21);
                Debug.Assert(pack.tune_TRY(ph).Equals("pdjwldhjbbwqusyurrzsb"));
                Debug.Assert(pack.target_component == (byte)(byte)82);
                Debug.Assert(pack.target_system == (byte)(byte)44);
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_component = (byte)(byte)82;
            p258.target_system = (byte)(byte)44;
            p258.tune_SET("pdjwldhjbbwqusyurrzsb", PH) ;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE));
                Debug.Assert(pack.resolution_h == (ushort)(ushort)39479);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)63, (byte)137, (byte)104, (byte)148, (byte)138, (byte)75, (byte)117, (byte)77, (byte)57, (byte)141, (byte)31, (byte)188, (byte)224, (byte)114, (byte)26, (byte)231, (byte)43, (byte)140, (byte)144, (byte)124, (byte)118, (byte)220, (byte)52, (byte)86, (byte)113, (byte)110, (byte)2, (byte)134, (byte)127, (byte)154, (byte)108, (byte)158}));
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)53, (byte)64, (byte)189, (byte)184, (byte)98, (byte)164, (byte)78, (byte)0, (byte)40, (byte)130, (byte)114, (byte)64, (byte)115, (byte)162, (byte)49, (byte)218, (byte)244, (byte)100, (byte)148, (byte)216, (byte)112, (byte)192, (byte)109, (byte)74, (byte)99, (byte)84, (byte)172, (byte)109, (byte)86, (byte)88, (byte)97, (byte)19}));
                Debug.Assert(pack.sensor_size_v == (float)1.3606105E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3885556479U);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)27010);
                Debug.Assert(pack.lens_id == (byte)(byte)34);
                Debug.Assert(pack.focal_length == (float)1.5454063E38F);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 128);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("vdxeskOowLlcvkmukrmpGomfbeDqnrotzfvlttdrmDpaiygeJujviMkuagubqfjvwLDxjcuxlhkzTfffdubyyrsIoiprDsHrbgubidGohnxhrhjnkvwbAyaFaerivgbo"));
                Debug.Assert(pack.firmware_version == (uint)1935715957U);
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)4345);
                Debug.Assert(pack.sensor_size_h == (float) -9.6794815E36F);
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)3885556479U;
            p259.model_name_SET(new byte[] {(byte)53, (byte)64, (byte)189, (byte)184, (byte)98, (byte)164, (byte)78, (byte)0, (byte)40, (byte)130, (byte)114, (byte)64, (byte)115, (byte)162, (byte)49, (byte)218, (byte)244, (byte)100, (byte)148, (byte)216, (byte)112, (byte)192, (byte)109, (byte)74, (byte)99, (byte)84, (byte)172, (byte)109, (byte)86, (byte)88, (byte)97, (byte)19}, 0) ;
            p259.cam_definition_version = (ushort)(ushort)4345;
            p259.sensor_size_h = (float) -9.6794815E36F;
            p259.firmware_version = (uint)1935715957U;
            p259.focal_length = (float)1.5454063E38F;
            p259.cam_definition_uri_SET("vdxeskOowLlcvkmukrmpGomfbeDqnrotzfvlttdrmDpaiygeJujviMkuagubqfjvwLDxjcuxlhkzTfffdubyyrsIoiprDsHrbgubidGohnxhrhjnkvwbAyaFaerivgbo", PH) ;
            p259.vendor_name_SET(new byte[] {(byte)63, (byte)137, (byte)104, (byte)148, (byte)138, (byte)75, (byte)117, (byte)77, (byte)57, (byte)141, (byte)31, (byte)188, (byte)224, (byte)114, (byte)26, (byte)231, (byte)43, (byte)140, (byte)144, (byte)124, (byte)118, (byte)220, (byte)52, (byte)86, (byte)113, (byte)110, (byte)2, (byte)134, (byte)127, (byte)154, (byte)108, (byte)158}, 0) ;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE);
            p259.lens_id = (byte)(byte)34;
            p259.resolution_h = (ushort)(ushort)39479;
            p259.sensor_size_v = (float)1.3606105E38F;
            p259.resolution_v = (ushort)(ushort)27010;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode_id == CAMERA_MODE.CAMERA_MODE_IMAGE);
                Debug.Assert(pack.time_boot_ms == (uint)3005542633U);
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)3005542633U;
            p260.mode_id = CAMERA_MODE.CAMERA_MODE_IMAGE;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.available_capacity == (float)1.5164371E38F);
                Debug.Assert(pack.storage_id == (byte)(byte)94);
                Debug.Assert(pack.used_capacity == (float) -2.0761156E38F);
                Debug.Assert(pack.total_capacity == (float)1.8922107E38F);
                Debug.Assert(pack.status == (byte)(byte)19);
                Debug.Assert(pack.write_speed == (float) -1.6014174E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3640173402U);
                Debug.Assert(pack.storage_count == (byte)(byte)81);
                Debug.Assert(pack.read_speed == (float) -2.9947371E38F);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.available_capacity = (float)1.5164371E38F;
            p261.storage_count = (byte)(byte)81;
            p261.time_boot_ms = (uint)3640173402U;
            p261.total_capacity = (float)1.8922107E38F;
            p261.status = (byte)(byte)19;
            p261.used_capacity = (float) -2.0761156E38F;
            p261.read_speed = (float) -2.9947371E38F;
            p261.write_speed = (float) -1.6014174E38F;
            p261.storage_id = (byte)(byte)94;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.available_capacity == (float)8.4530623E37F);
                Debug.Assert(pack.recording_time_ms == (uint)3083085518U);
                Debug.Assert(pack.time_boot_ms == (uint)1757281253U);
                Debug.Assert(pack.video_status == (byte)(byte)90);
                Debug.Assert(pack.image_status == (byte)(byte)188);
                Debug.Assert(pack.image_interval == (float) -2.029947E38F);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.recording_time_ms = (uint)3083085518U;
            p262.image_interval = (float) -2.029947E38F;
            p262.time_boot_ms = (uint)1757281253U;
            p262.image_status = (byte)(byte)188;
            p262.video_status = (byte)(byte)90;
            p262.available_capacity = (float)8.4530623E37F;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.image_index == (int) -1312768999);
                Debug.Assert(pack.time_utc == (ulong)8412357898090984855L);
                Debug.Assert(pack.lat == (int)1714369045);
                Debug.Assert(pack.time_boot_ms == (uint)2475853999U);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.1586522E38F, -3.267878E38F, -3.0595548E38F, -2.8293997E37F}));
                Debug.Assert(pack.lon == (int)1616706267);
                Debug.Assert(pack.file_url_LEN(ph) == 169);
                Debug.Assert(pack.file_url_TRY(ph).Equals("FsckknutScbRnrgaebgzKpzbdyyjGBzgbopgkrdodieuameullejyjuSbzbThLsrwlWQbplznxycladgutppfyjBxpxkkcjeuvwhdsuujmlqqaoslrpyfqmaodelEucxbqmmvVpovktatpqsovibUUriHkImfrXtpujRctxgt"));
                Debug.Assert(pack.relative_alt == (int) -900105998);
                Debug.Assert(pack.alt == (int) -1053868423);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte) - 110);
                Debug.Assert(pack.camera_id == (byte)(byte)48);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.q_SET(new float[] {-2.1586522E38F, -3.267878E38F, -3.0595548E38F, -2.8293997E37F}, 0) ;
            p263.image_index = (int) -1312768999;
            p263.time_boot_ms = (uint)2475853999U;
            p263.file_url_SET("FsckknutScbRnrgaebgzKpzbdyyjGBzgbopgkrdodieuameullejyjuSbzbThLsrwlWQbplznxycladgutppfyjBxpxkkcjeuvwhdsuujmlqqaoslrpyfqmaodelEucxbqmmvVpovktatpqsovibUUriHkImfrXtpujRctxgt", PH) ;
            p263.relative_alt = (int) -900105998;
            p263.lat = (int)1714369045;
            p263.lon = (int)1616706267;
            p263.time_utc = (ulong)8412357898090984855L;
            p263.camera_id = (byte)(byte)48;
            p263.capture_result = (sbyte)(sbyte) - 110;
            p263.alt = (int) -1053868423;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.takeoff_time_utc == (ulong)6357957859899640331L);
                Debug.Assert(pack.flight_uuid == (ulong)9024321718819684506L);
                Debug.Assert(pack.arming_time_utc == (ulong)8235175585064863102L);
                Debug.Assert(pack.time_boot_ms == (uint)1999685487U);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)1999685487U;
            p264.flight_uuid = (ulong)9024321718819684506L;
            p264.arming_time_utc = (ulong)8235175585064863102L;
            p264.takeoff_time_utc = (ulong)6357957859899640331L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)3.1093262E38F);
                Debug.Assert(pack.roll == (float) -1.562107E38F);
                Debug.Assert(pack.pitch == (float) -1.8169538E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4266199238U);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.time_boot_ms = (uint)4266199238U;
            p265.yaw = (float)3.1093262E38F;
            p265.pitch = (float) -1.8169538E38F;
            p265.roll = (float) -1.562107E38F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)91, (byte)96, (byte)65, (byte)41, (byte)138, (byte)234, (byte)149, (byte)130, (byte)228, (byte)103, (byte)109, (byte)240, (byte)113, (byte)182, (byte)14, (byte)180, (byte)195, (byte)194, (byte)37, (byte)75, (byte)154, (byte)88, (byte)180, (byte)110, (byte)87, (byte)38, (byte)184, (byte)16, (byte)106, (byte)235, (byte)178, (byte)240, (byte)92, (byte)35, (byte)97, (byte)204, (byte)230, (byte)116, (byte)79, (byte)98, (byte)200, (byte)30, (byte)242, (byte)199, (byte)32, (byte)181, (byte)19, (byte)59, (byte)174, (byte)156, (byte)221, (byte)178, (byte)7, (byte)199, (byte)27, (byte)88, (byte)212, (byte)155, (byte)226, (byte)111, (byte)223, (byte)224, (byte)204, (byte)141, (byte)66, (byte)249, (byte)96, (byte)218, (byte)228, (byte)20, (byte)4, (byte)83, (byte)111, (byte)70, (byte)79, (byte)55, (byte)85, (byte)33, (byte)61, (byte)185, (byte)214, (byte)160, (byte)193, (byte)132, (byte)9, (byte)72, (byte)29, (byte)183, (byte)225, (byte)227, (byte)81, (byte)227, (byte)133, (byte)64, (byte)100, (byte)12, (byte)193, (byte)131, (byte)211, (byte)184, (byte)42, (byte)31, (byte)51, (byte)157, (byte)187, (byte)45, (byte)125, (byte)101, (byte)225, (byte)61, (byte)225, (byte)217, (byte)154, (byte)3, (byte)141, (byte)147, (byte)152, (byte)5, (byte)195, (byte)16, (byte)38, (byte)141, (byte)144, (byte)38, (byte)168, (byte)98, (byte)76, (byte)54, (byte)48, (byte)61, (byte)71, (byte)165, (byte)119, (byte)155, (byte)195, (byte)198, (byte)143, (byte)224, (byte)117, (byte)99, (byte)196, (byte)225, (byte)98, (byte)252, (byte)157, (byte)237, (byte)15, (byte)39, (byte)95, (byte)44, (byte)197, (byte)2, (byte)222, (byte)182, (byte)179, (byte)90, (byte)33, (byte)161, (byte)253, (byte)121, (byte)215, (byte)33, (byte)181, (byte)154, (byte)6, (byte)108, (byte)140, (byte)148, (byte)89, (byte)104, (byte)176, (byte)227, (byte)190, (byte)78, (byte)93, (byte)158, (byte)53, (byte)243, (byte)46, (byte)250, (byte)183, (byte)93, (byte)128, (byte)129, (byte)246, (byte)48, (byte)186, (byte)3, (byte)152, (byte)149, (byte)36, (byte)135, (byte)61, (byte)106, (byte)94, (byte)41, (byte)138, (byte)123, (byte)36, (byte)117, (byte)6, (byte)230, (byte)174, (byte)119, (byte)125, (byte)221, (byte)190, (byte)52, (byte)121, (byte)241, (byte)0, (byte)10, (byte)165, (byte)190, (byte)203, (byte)49, (byte)82, (byte)231, (byte)52, (byte)65, (byte)32, (byte)37, (byte)17, (byte)249, (byte)107, (byte)151, (byte)209, (byte)194, (byte)67, (byte)194, (byte)160, (byte)57, (byte)190, (byte)45, (byte)126, (byte)116, (byte)217, (byte)38, (byte)248, (byte)147, (byte)122, (byte)49, (byte)124, (byte)207, (byte)119, (byte)171, (byte)150, (byte)58, (byte)132}));
                Debug.Assert(pack.sequence == (ushort)(ushort)54658);
                Debug.Assert(pack.target_system == (byte)(byte)203);
                Debug.Assert(pack.length == (byte)(byte)125);
                Debug.Assert(pack.target_component == (byte)(byte)25);
                Debug.Assert(pack.first_message_offset == (byte)(byte)178);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.sequence = (ushort)(ushort)54658;
            p266.data__SET(new byte[] {(byte)91, (byte)96, (byte)65, (byte)41, (byte)138, (byte)234, (byte)149, (byte)130, (byte)228, (byte)103, (byte)109, (byte)240, (byte)113, (byte)182, (byte)14, (byte)180, (byte)195, (byte)194, (byte)37, (byte)75, (byte)154, (byte)88, (byte)180, (byte)110, (byte)87, (byte)38, (byte)184, (byte)16, (byte)106, (byte)235, (byte)178, (byte)240, (byte)92, (byte)35, (byte)97, (byte)204, (byte)230, (byte)116, (byte)79, (byte)98, (byte)200, (byte)30, (byte)242, (byte)199, (byte)32, (byte)181, (byte)19, (byte)59, (byte)174, (byte)156, (byte)221, (byte)178, (byte)7, (byte)199, (byte)27, (byte)88, (byte)212, (byte)155, (byte)226, (byte)111, (byte)223, (byte)224, (byte)204, (byte)141, (byte)66, (byte)249, (byte)96, (byte)218, (byte)228, (byte)20, (byte)4, (byte)83, (byte)111, (byte)70, (byte)79, (byte)55, (byte)85, (byte)33, (byte)61, (byte)185, (byte)214, (byte)160, (byte)193, (byte)132, (byte)9, (byte)72, (byte)29, (byte)183, (byte)225, (byte)227, (byte)81, (byte)227, (byte)133, (byte)64, (byte)100, (byte)12, (byte)193, (byte)131, (byte)211, (byte)184, (byte)42, (byte)31, (byte)51, (byte)157, (byte)187, (byte)45, (byte)125, (byte)101, (byte)225, (byte)61, (byte)225, (byte)217, (byte)154, (byte)3, (byte)141, (byte)147, (byte)152, (byte)5, (byte)195, (byte)16, (byte)38, (byte)141, (byte)144, (byte)38, (byte)168, (byte)98, (byte)76, (byte)54, (byte)48, (byte)61, (byte)71, (byte)165, (byte)119, (byte)155, (byte)195, (byte)198, (byte)143, (byte)224, (byte)117, (byte)99, (byte)196, (byte)225, (byte)98, (byte)252, (byte)157, (byte)237, (byte)15, (byte)39, (byte)95, (byte)44, (byte)197, (byte)2, (byte)222, (byte)182, (byte)179, (byte)90, (byte)33, (byte)161, (byte)253, (byte)121, (byte)215, (byte)33, (byte)181, (byte)154, (byte)6, (byte)108, (byte)140, (byte)148, (byte)89, (byte)104, (byte)176, (byte)227, (byte)190, (byte)78, (byte)93, (byte)158, (byte)53, (byte)243, (byte)46, (byte)250, (byte)183, (byte)93, (byte)128, (byte)129, (byte)246, (byte)48, (byte)186, (byte)3, (byte)152, (byte)149, (byte)36, (byte)135, (byte)61, (byte)106, (byte)94, (byte)41, (byte)138, (byte)123, (byte)36, (byte)117, (byte)6, (byte)230, (byte)174, (byte)119, (byte)125, (byte)221, (byte)190, (byte)52, (byte)121, (byte)241, (byte)0, (byte)10, (byte)165, (byte)190, (byte)203, (byte)49, (byte)82, (byte)231, (byte)52, (byte)65, (byte)32, (byte)37, (byte)17, (byte)249, (byte)107, (byte)151, (byte)209, (byte)194, (byte)67, (byte)194, (byte)160, (byte)57, (byte)190, (byte)45, (byte)126, (byte)116, (byte)217, (byte)38, (byte)248, (byte)147, (byte)122, (byte)49, (byte)124, (byte)207, (byte)119, (byte)171, (byte)150, (byte)58, (byte)132}, 0) ;
            p266.first_message_offset = (byte)(byte)178;
            p266.target_system = (byte)(byte)203;
            p266.target_component = (byte)(byte)25;
            p266.length = (byte)(byte)125;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.length == (byte)(byte)205);
                Debug.Assert(pack.first_message_offset == (byte)(byte)0);
                Debug.Assert(pack.sequence == (ushort)(ushort)47263);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)101, (byte)5, (byte)212, (byte)73, (byte)11, (byte)208, (byte)244, (byte)150, (byte)155, (byte)195, (byte)19, (byte)135, (byte)118, (byte)59, (byte)19, (byte)139, (byte)199, (byte)203, (byte)129, (byte)97, (byte)247, (byte)208, (byte)76, (byte)57, (byte)56, (byte)224, (byte)18, (byte)18, (byte)74, (byte)222, (byte)64, (byte)125, (byte)101, (byte)13, (byte)119, (byte)190, (byte)6, (byte)216, (byte)33, (byte)49, (byte)194, (byte)177, (byte)160, (byte)38, (byte)108, (byte)78, (byte)183, (byte)202, (byte)148, (byte)236, (byte)42, (byte)17, (byte)50, (byte)129, (byte)215, (byte)100, (byte)7, (byte)188, (byte)117, (byte)154, (byte)100, (byte)99, (byte)116, (byte)11, (byte)21, (byte)129, (byte)50, (byte)135, (byte)203, (byte)46, (byte)196, (byte)93, (byte)107, (byte)174, (byte)236, (byte)107, (byte)115, (byte)127, (byte)10, (byte)127, (byte)132, (byte)113, (byte)6, (byte)100, (byte)238, (byte)174, (byte)228, (byte)230, (byte)14, (byte)52, (byte)189, (byte)105, (byte)120, (byte)202, (byte)157, (byte)55, (byte)250, (byte)184, (byte)166, (byte)7, (byte)14, (byte)92, (byte)167, (byte)42, (byte)126, (byte)5, (byte)14, (byte)21, (byte)196, (byte)5, (byte)230, (byte)151, (byte)17, (byte)220, (byte)47, (byte)56, (byte)31, (byte)28, (byte)105, (byte)48, (byte)49, (byte)7, (byte)224, (byte)233, (byte)141, (byte)246, (byte)214, (byte)26, (byte)168, (byte)98, (byte)19, (byte)147, (byte)160, (byte)11, (byte)157, (byte)51, (byte)88, (byte)227, (byte)138, (byte)195, (byte)210, (byte)75, (byte)93, (byte)100, (byte)216, (byte)121, (byte)245, (byte)243, (byte)185, (byte)209, (byte)86, (byte)58, (byte)230, (byte)198, (byte)50, (byte)196, (byte)224, (byte)127, (byte)55, (byte)33, (byte)4, (byte)203, (byte)51, (byte)177, (byte)57, (byte)13, (byte)167, (byte)39, (byte)175, (byte)150, (byte)9, (byte)202, (byte)9, (byte)121, (byte)222, (byte)220, (byte)168, (byte)31, (byte)52, (byte)5, (byte)224, (byte)199, (byte)174, (byte)75, (byte)103, (byte)11, (byte)110, (byte)131, (byte)127, (byte)22, (byte)189, (byte)225, (byte)143, (byte)197, (byte)80, (byte)168, (byte)227, (byte)28, (byte)169, (byte)224, (byte)4, (byte)153, (byte)202, (byte)75, (byte)69, (byte)1, (byte)163, (byte)49, (byte)24, (byte)149, (byte)201, (byte)17, (byte)156, (byte)23, (byte)13, (byte)23, (byte)191, (byte)114, (byte)72, (byte)195, (byte)231, (byte)253, (byte)70, (byte)194, (byte)168, (byte)166, (byte)130, (byte)92, (byte)43, (byte)10, (byte)141, (byte)27, (byte)87, (byte)248, (byte)150, (byte)250, (byte)163, (byte)217, (byte)107, (byte)101, (byte)216, (byte)152, (byte)114, (byte)245, (byte)93, (byte)11, (byte)57, (byte)180, (byte)102}));
                Debug.Assert(pack.target_system == (byte)(byte)171);
                Debug.Assert(pack.target_component == (byte)(byte)171);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.sequence = (ushort)(ushort)47263;
            p267.length = (byte)(byte)205;
            p267.data__SET(new byte[] {(byte)101, (byte)5, (byte)212, (byte)73, (byte)11, (byte)208, (byte)244, (byte)150, (byte)155, (byte)195, (byte)19, (byte)135, (byte)118, (byte)59, (byte)19, (byte)139, (byte)199, (byte)203, (byte)129, (byte)97, (byte)247, (byte)208, (byte)76, (byte)57, (byte)56, (byte)224, (byte)18, (byte)18, (byte)74, (byte)222, (byte)64, (byte)125, (byte)101, (byte)13, (byte)119, (byte)190, (byte)6, (byte)216, (byte)33, (byte)49, (byte)194, (byte)177, (byte)160, (byte)38, (byte)108, (byte)78, (byte)183, (byte)202, (byte)148, (byte)236, (byte)42, (byte)17, (byte)50, (byte)129, (byte)215, (byte)100, (byte)7, (byte)188, (byte)117, (byte)154, (byte)100, (byte)99, (byte)116, (byte)11, (byte)21, (byte)129, (byte)50, (byte)135, (byte)203, (byte)46, (byte)196, (byte)93, (byte)107, (byte)174, (byte)236, (byte)107, (byte)115, (byte)127, (byte)10, (byte)127, (byte)132, (byte)113, (byte)6, (byte)100, (byte)238, (byte)174, (byte)228, (byte)230, (byte)14, (byte)52, (byte)189, (byte)105, (byte)120, (byte)202, (byte)157, (byte)55, (byte)250, (byte)184, (byte)166, (byte)7, (byte)14, (byte)92, (byte)167, (byte)42, (byte)126, (byte)5, (byte)14, (byte)21, (byte)196, (byte)5, (byte)230, (byte)151, (byte)17, (byte)220, (byte)47, (byte)56, (byte)31, (byte)28, (byte)105, (byte)48, (byte)49, (byte)7, (byte)224, (byte)233, (byte)141, (byte)246, (byte)214, (byte)26, (byte)168, (byte)98, (byte)19, (byte)147, (byte)160, (byte)11, (byte)157, (byte)51, (byte)88, (byte)227, (byte)138, (byte)195, (byte)210, (byte)75, (byte)93, (byte)100, (byte)216, (byte)121, (byte)245, (byte)243, (byte)185, (byte)209, (byte)86, (byte)58, (byte)230, (byte)198, (byte)50, (byte)196, (byte)224, (byte)127, (byte)55, (byte)33, (byte)4, (byte)203, (byte)51, (byte)177, (byte)57, (byte)13, (byte)167, (byte)39, (byte)175, (byte)150, (byte)9, (byte)202, (byte)9, (byte)121, (byte)222, (byte)220, (byte)168, (byte)31, (byte)52, (byte)5, (byte)224, (byte)199, (byte)174, (byte)75, (byte)103, (byte)11, (byte)110, (byte)131, (byte)127, (byte)22, (byte)189, (byte)225, (byte)143, (byte)197, (byte)80, (byte)168, (byte)227, (byte)28, (byte)169, (byte)224, (byte)4, (byte)153, (byte)202, (byte)75, (byte)69, (byte)1, (byte)163, (byte)49, (byte)24, (byte)149, (byte)201, (byte)17, (byte)156, (byte)23, (byte)13, (byte)23, (byte)191, (byte)114, (byte)72, (byte)195, (byte)231, (byte)253, (byte)70, (byte)194, (byte)168, (byte)166, (byte)130, (byte)92, (byte)43, (byte)10, (byte)141, (byte)27, (byte)87, (byte)248, (byte)150, (byte)250, (byte)163, (byte)217, (byte)107, (byte)101, (byte)216, (byte)152, (byte)114, (byte)245, (byte)93, (byte)11, (byte)57, (byte)180, (byte)102}, 0) ;
            p267.first_message_offset = (byte)(byte)0;
            p267.target_component = (byte)(byte)171;
            p267.target_system = (byte)(byte)171;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)181);
                Debug.Assert(pack.target_system == (byte)(byte)220);
                Debug.Assert(pack.sequence == (ushort)(ushort)6350);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)220;
            p268.target_component = (byte)(byte)181;
            p268.sequence = (ushort)(ushort)6350;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.framerate == (float)2.661567E38F);
                Debug.Assert(pack.camera_id == (byte)(byte)119);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)4105);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)57113);
                Debug.Assert(pack.status == (byte)(byte)43);
                Debug.Assert(pack.uri_LEN(ph) == 112);
                Debug.Assert(pack.uri_TRY(ph).Equals("tUqAoocpcfolngqdhdaZtygkmnxjrmfnjvwtasxflwuyuujkPukeepkopBpqcgkglXnikcncGdnxdlslrWwKipuifZdrldmyWsnnmlbyRpxasrej"));
                Debug.Assert(pack.bitrate == (uint)3948318626U);
                Debug.Assert(pack.rotation == (ushort)(ushort)17503);
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.camera_id = (byte)(byte)119;
            p269.uri_SET("tUqAoocpcfolngqdhdaZtygkmnxjrmfnjvwtasxflwuyuujkPukeepkopBpqcgkglXnikcncGdnxdlslrWwKipuifZdrldmyWsnnmlbyRpxasrej", PH) ;
            p269.framerate = (float)2.661567E38F;
            p269.bitrate = (uint)3948318626U;
            p269.resolution_h = (ushort)(ushort)4105;
            p269.rotation = (ushort)(ushort)17503;
            p269.status = (byte)(byte)43;
            p269.resolution_v = (ushort)(ushort)57113;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)116);
                Debug.Assert(pack.uri_LEN(ph) == 130);
                Debug.Assert(pack.uri_TRY(ph).Equals("xIovelztlFatOzwxAyzdbEUasvbxhmfiuqmvGwveoihmavKbiwdgYhTdqdrdlhhBdspnsnfHtgphcbmphxaymoksdakpjsuidvnburxvtlpvnkadhnosRwTriddmyquzIb"));
                Debug.Assert(pack.framerate == (float) -2.5867743E38F);
                Debug.Assert(pack.target_system == (byte)(byte)173);
                Debug.Assert(pack.rotation == (ushort)(ushort)8982);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)17521);
                Debug.Assert(pack.camera_id == (byte)(byte)105);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)2073);
                Debug.Assert(pack.bitrate == (uint)866902773U);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.resolution_h = (ushort)(ushort)17521;
            p270.bitrate = (uint)866902773U;
            p270.camera_id = (byte)(byte)105;
            p270.target_system = (byte)(byte)173;
            p270.resolution_v = (ushort)(ushort)2073;
            p270.framerate = (float) -2.5867743E38F;
            p270.rotation = (ushort)(ushort)8982;
            p270.target_component = (byte)(byte)116;
            p270.uri_SET("xIovelztlFatOzwxAyzdbEUasvbxhmfiuqmvGwveoihmavKbiwdgYhTdqdrdlhhBdspnsnfHtgphcbmphxaymoksdakpjsuidvnburxvtlpvnkadhnosRwTriddmyquzIb", PH) ;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 39);
                Debug.Assert(pack.password_TRY(ph).Equals("cocqidcocurbrAuucqHbrOijzblpgfxhjrbgzDb"));
                Debug.Assert(pack.ssid_LEN(ph) == 17);
                Debug.Assert(pack.ssid_TRY(ph).Equals("ntsudlyptglHzvzvc"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.password_SET("cocqidcocurbrAuucqHbrOijzblpgfxhjrbgzDb", PH) ;
            p299.ssid_SET("ntsudlyptglHzvzvc", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_version == (ushort)(ushort)43817);
                Debug.Assert(pack.version == (ushort)(ushort)45477);
                Debug.Assert(pack.max_version == (ushort)(ushort)60897);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)46, (byte)193, (byte)250, (byte)152, (byte)46, (byte)159, (byte)68, (byte)196}));
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)77, (byte)234, (byte)135, (byte)120, (byte)183, (byte)92, (byte)188, (byte)184}));
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.max_version = (ushort)(ushort)60897;
            p300.version = (ushort)(ushort)45477;
            p300.library_version_hash_SET(new byte[] {(byte)46, (byte)193, (byte)250, (byte)152, (byte)46, (byte)159, (byte)68, (byte)196}, 0) ;
            p300.spec_version_hash_SET(new byte[] {(byte)77, (byte)234, (byte)135, (byte)120, (byte)183, (byte)92, (byte)188, (byte)184}, 0) ;
            p300.min_version = (ushort)(ushort)43817;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sub_mode == (byte)(byte)140);
                Debug.Assert(pack.uptime_sec == (uint)4106615037U);
                Debug.Assert(pack.mode == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)60099);
                Debug.Assert(pack.time_usec == (ulong)3280702233508658594L);
                Debug.Assert(pack.health == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_OK;
            p310.vendor_specific_status_code = (ushort)(ushort)60099;
            p310.sub_mode = (byte)(byte)140;
            p310.uptime_sec = (uint)4106615037U;
            p310.time_usec = (ulong)3280702233508658594L;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hw_version_minor == (byte)(byte)160);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)177);
                Debug.Assert(pack.hw_version_major == (byte)(byte)65);
                Debug.Assert(pack.name_LEN(ph) == 8);
                Debug.Assert(pack.name_TRY(ph).Equals("thpqHegv"));
                Debug.Assert(pack.sw_vcs_commit == (uint)1308167308U);
                Debug.Assert(pack.time_usec == (ulong)1561334624274177255L);
                Debug.Assert(pack.sw_version_major == (byte)(byte)207);
                Debug.Assert(pack.uptime_sec == (uint)3020756937U);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)82, (byte)203, (byte)243, (byte)133, (byte)116, (byte)1, (byte)151, (byte)146, (byte)227, (byte)230, (byte)101, (byte)210, (byte)179, (byte)245, (byte)241, (byte)45}));
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.name_SET("thpqHegv", PH) ;
            p311.sw_vcs_commit = (uint)1308167308U;
            p311.hw_version_minor = (byte)(byte)160;
            p311.hw_version_major = (byte)(byte)65;
            p311.sw_version_minor = (byte)(byte)177;
            p311.hw_unique_id_SET(new byte[] {(byte)82, (byte)203, (byte)243, (byte)133, (byte)116, (byte)1, (byte)151, (byte)146, (byte)227, (byte)230, (byte)101, (byte)210, (byte)179, (byte)245, (byte)241, (byte)45}, 0) ;
            p311.sw_version_major = (byte)(byte)207;
            p311.uptime_sec = (uint)3020756937U;
            p311.time_usec = (ulong)1561334624274177255L;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)255);
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("dbyjzpxq"));
                Debug.Assert(pack.target_system == (byte)(byte)183);
                Debug.Assert(pack.param_index == (short)(short)17239);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_system = (byte)(byte)183;
            p320.target_component = (byte)(byte)255;
            p320.param_index = (short)(short)17239;
            p320.param_id_SET("dbyjzpxq", PH) ;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)32);
                Debug.Assert(pack.target_component == (byte)(byte)8);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)32;
            p321.target_component = (byte)(byte)8;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 4);
                Debug.Assert(pack.param_id_TRY(ph).Equals("wigz"));
                Debug.Assert(pack.param_value_LEN(ph) == 66);
                Debug.Assert(pack.param_value_TRY(ph).Equals("qewmtbbDYnnheowoxzoebwssscNeXcbvfgtrphmljzTbgrcecbXMkrzijcJReyqcjn"));
                Debug.Assert(pack.param_index == (ushort)(ushort)10458);
                Debug.Assert(pack.param_count == (ushort)(ushort)4846);
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_count = (ushort)(ushort)4846;
            p322.param_id_SET("wigz", PH) ;
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            p322.param_value_SET("qewmtbbDYnnheowoxzoebwssscNeXcbvfgtrphmljzTbgrcecbXMkrzijcJReyqcjn", PH) ;
            p322.param_index = (ushort)(ushort)10458;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 10);
                Debug.Assert(pack.param_id_TRY(ph).Equals("iwkzmOxjbb"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
                Debug.Assert(pack.target_system == (byte)(byte)203);
                Debug.Assert(pack.target_component == (byte)(byte)181);
                Debug.Assert(pack.param_value_LEN(ph) == 88);
                Debug.Assert(pack.param_value_TRY(ph).Equals("dkqytvqozrhEsDwGvgnvkffwoddcljvavqdxcOvpApptdKlptalqbvrcotmgBzwtjggoilpzxpyxoarpgoAckjkl"));
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            p323.target_system = (byte)(byte)203;
            p323.param_value_SET("dkqytvqozrhEsDwGvgnvkffwoddcljvavqdxcOvpApptdKlptalqbvrcotmgBzwtjggoilpzxpyxoarpgoAckjkl", PH) ;
            p323.param_id_SET("iwkzmOxjbb", PH) ;
            p323.target_component = (byte)(byte)181;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
                Debug.Assert(pack.param_result == PARAM_ACK.PARAM_ACK_IN_PROGRESS);
                Debug.Assert(pack.param_id_LEN(ph) == 11);
                Debug.Assert(pack.param_id_TRY(ph).Equals("bvhqjjumoju"));
                Debug.Assert(pack.param_value_LEN(ph) == 81);
                Debug.Assert(pack.param_value_TRY(ph).Equals("OqNmbjthsxtAorswsiutruuAuaowajJpklaitjfyxnnCttfrunjabktqevjyhdvgeovkdowdnlwrijrgk"));
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            p324.param_result = PARAM_ACK.PARAM_ACK_IN_PROGRESS;
            p324.param_id_SET("bvhqjjumoju", PH) ;
            p324.param_value_SET("OqNmbjthsxtAorswsiutruuAuaowajJpklaitjfyxnnCttfrunjabktqevjyhdvgeovkdowdnlwrijrgk", PH) ;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.increment == (byte)(byte)8);
                Debug.Assert(pack.sensor_type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
                Debug.Assert(pack.max_distance == (ushort)(ushort)1820);
                Debug.Assert(pack.time_usec == (ulong)3473488405695365739L);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)8474, (ushort)48978, (ushort)14052, (ushort)53819, (ushort)25333, (ushort)62029, (ushort)8073, (ushort)61990, (ushort)32546, (ushort)36008, (ushort)28487, (ushort)57795, (ushort)46066, (ushort)37904, (ushort)59865, (ushort)22242, (ushort)46956, (ushort)3147, (ushort)27201, (ushort)1896, (ushort)15628, (ushort)27584, (ushort)58414, (ushort)10402, (ushort)17181, (ushort)35720, (ushort)9740, (ushort)16894, (ushort)21796, (ushort)59613, (ushort)16747, (ushort)58082, (ushort)36967, (ushort)8873, (ushort)36914, (ushort)29923, (ushort)47063, (ushort)34141, (ushort)36805, (ushort)62880, (ushort)61256, (ushort)10515, (ushort)8306, (ushort)53597, (ushort)7810, (ushort)54656, (ushort)48795, (ushort)13356, (ushort)23522, (ushort)40648, (ushort)42742, (ushort)63754, (ushort)52545, (ushort)50097, (ushort)6090, (ushort)36021, (ushort)24783, (ushort)21986, (ushort)20757, (ushort)31184, (ushort)28535, (ushort)56581, (ushort)56263, (ushort)5783, (ushort)52136, (ushort)49178, (ushort)60326, (ushort)34759, (ushort)30378, (ushort)810, (ushort)42543, (ushort)7560}));
                Debug.Assert(pack.min_distance == (ushort)(ushort)29924);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.min_distance = (ushort)(ushort)29924;
            p330.distances_SET(new ushort[] {(ushort)8474, (ushort)48978, (ushort)14052, (ushort)53819, (ushort)25333, (ushort)62029, (ushort)8073, (ushort)61990, (ushort)32546, (ushort)36008, (ushort)28487, (ushort)57795, (ushort)46066, (ushort)37904, (ushort)59865, (ushort)22242, (ushort)46956, (ushort)3147, (ushort)27201, (ushort)1896, (ushort)15628, (ushort)27584, (ushort)58414, (ushort)10402, (ushort)17181, (ushort)35720, (ushort)9740, (ushort)16894, (ushort)21796, (ushort)59613, (ushort)16747, (ushort)58082, (ushort)36967, (ushort)8873, (ushort)36914, (ushort)29923, (ushort)47063, (ushort)34141, (ushort)36805, (ushort)62880, (ushort)61256, (ushort)10515, (ushort)8306, (ushort)53597, (ushort)7810, (ushort)54656, (ushort)48795, (ushort)13356, (ushort)23522, (ushort)40648, (ushort)42742, (ushort)63754, (ushort)52545, (ushort)50097, (ushort)6090, (ushort)36021, (ushort)24783, (ushort)21986, (ushort)20757, (ushort)31184, (ushort)28535, (ushort)56581, (ushort)56263, (ushort)5783, (ushort)52136, (ushort)49178, (ushort)60326, (ushort)34759, (ushort)30378, (ushort)810, (ushort)42543, (ushort)7560}, 0) ;
            p330.max_distance = (ushort)(ushort)1820;
            p330.time_usec = (ulong)3473488405695365739L;
            p330.increment = (byte)(byte)8;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
        }
    }
}