
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
                    ulong id = id__W(value);
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
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED));
                Debug.Assert(pack.custom_mode == (uint)1210026020U);
                Debug.Assert(pack.autopilot == MAV_AUTOPILOT.MAV_AUTOPILOT_ASLUAV);
                Debug.Assert(pack.mavlink_version == (byte)(byte)3);
                Debug.Assert(pack.type == MAV_TYPE.MAV_TYPE_COAXIAL);
                Debug.Assert(pack.system_status == MAV_STATE.MAV_STATE_EMERGENCY);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.custom_mode = (uint)1210026020U;
            p0.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_ASLUAV;
            p0.mavlink_version = (byte)(byte)3;
            p0.type = MAV_TYPE.MAV_TYPE_COAXIAL;
            p0.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED);
            p0.system_status = MAV_STATE.MAV_STATE_EMERGENCY;
            ADV_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)79);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION));
                Debug.Assert(pack.load == (ushort)(ushort)44101);
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)32725);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)4432);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)45224);
                Debug.Assert(pack.current_battery == (short)(short) -24607);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)48942);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)47308);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)36514);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS));
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION));
                Debug.Assert(pack.errors_comm == (ushort)(ushort)37326);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.errors_count3 = (ushort)(ushort)36514;
            p1.errors_count1 = (ushort)(ushort)45224;
            p1.drop_rate_comm = (ushort)(ushort)32725;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION);
            p1.errors_count4 = (ushort)(ushort)4432;
            p1.current_battery = (short)(short) -24607;
            p1.load = (ushort)(ushort)44101;
            p1.errors_comm = (ushort)(ushort)37326;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS);
            p1.battery_remaining = (sbyte)(sbyte)79;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION);
            p1.voltage_battery = (ushort)(ushort)48942;
            p1.errors_count2 = (ushort)(ushort)47308;
            ADV_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3654890946U);
                Debug.Assert(pack.time_unix_usec == (ulong)5262127384915955236L);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_boot_ms = (uint)3654890946U;
            p2.time_unix_usec = (ulong)5262127384915955236L;
            ADV_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float) -1.2725492E38F);
                Debug.Assert(pack.x == (float)1.649142E38F);
                Debug.Assert(pack.vy == (float) -1.1718579E38F);
                Debug.Assert(pack.afx == (float) -2.8052434E38F);
                Debug.Assert(pack.y == (float)2.0171055E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)58491);
                Debug.Assert(pack.afy == (float) -3.2428735E38F);
                Debug.Assert(pack.afz == (float) -1.364654E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.vz == (float)6.7923513E37F);
                Debug.Assert(pack.vx == (float) -2.577862E38F);
                Debug.Assert(pack.yaw == (float)2.4020717E38F);
                Debug.Assert(pack.yaw_rate == (float)2.949823E38F);
                Debug.Assert(pack.time_boot_ms == (uint)520512263U);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.vy = (float) -1.1718579E38F;
            p3.vz = (float)6.7923513E37F;
            p3.afx = (float) -2.8052434E38F;
            p3.afy = (float) -3.2428735E38F;
            p3.y = (float)2.0171055E38F;
            p3.x = (float)1.649142E38F;
            p3.yaw_rate = (float)2.949823E38F;
            p3.afz = (float) -1.364654E38F;
            p3.time_boot_ms = (uint)520512263U;
            p3.yaw = (float)2.4020717E38F;
            p3.z = (float) -1.2725492E38F;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p3.type_mask = (ushort)(ushort)58491;
            p3.vx = (float) -2.577862E38F;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)53);
                Debug.Assert(pack.time_usec == (ulong)3755731550306356804L);
                Debug.Assert(pack.seq == (uint)3081842091U);
                Debug.Assert(pack.target_component == (byte)(byte)44);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.target_system = (byte)(byte)53;
            p4.seq = (uint)3081842091U;
            p4.target_component = (byte)(byte)44;
            p4.time_usec = (ulong)3755731550306356804L;
            ADV_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.control_request == (byte)(byte)180);
                Debug.Assert(pack.target_system == (byte)(byte)207);
                Debug.Assert(pack.passkey_LEN(ph) == 11);
                Debug.Assert(pack.passkey_TRY(ph).Equals("cxvfkjeqvvg"));
                Debug.Assert(pack.version == (byte)(byte)212);
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.passkey_SET("cxvfkjeqvvg", PH) ;
            p5.target_system = (byte)(byte)207;
            p5.control_request = (byte)(byte)180;
            p5.version = (byte)(byte)212;
            ADV_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.control_request == (byte)(byte)115);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)103);
                Debug.Assert(pack.ack == (byte)(byte)19);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.gcs_system_id = (byte)(byte)103;
            p6.ack = (byte)(byte)19;
            p6.control_request = (byte)(byte)115;
            ADV_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 21);
                Debug.Assert(pack.key_TRY(ph).Equals("nskkwPhyjjthlkigumAyj"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("nskkwPhyjjthlkigumAyj", PH) ;
            ADV_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.custom_mode == (uint)2642632504U);
                Debug.Assert(pack.base_mode == MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
                Debug.Assert(pack.target_system == (byte)(byte)5);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.custom_mode = (uint)2642632504U;
            p11.base_mode = MAV_MODE.MAV_MODE_STABILIZE_DISARMED;
            p11.target_system = (byte)(byte)5;
            ADV_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("yzdoq"));
                Debug.Assert(pack.target_component == (byte)(byte)217);
                Debug.Assert(pack.param_index == (short)(short) -2566);
                Debug.Assert(pack.target_system == (byte)(byte)181);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.param_id_SET("yzdoq", PH) ;
            p20.target_component = (byte)(byte)217;
            p20.param_index = (short)(short) -2566;
            p20.target_system = (byte)(byte)181;
            ADV_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)185);
                Debug.Assert(pack.target_component == (byte)(byte)244);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)185;
            p21.target_component = (byte)(byte)244;
            ADV_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value == (float)2.69084E38F);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
                Debug.Assert(pack.param_index == (ushort)(ushort)16461);
                Debug.Assert(pack.param_id_LEN(ph) == 14);
                Debug.Assert(pack.param_id_TRY(ph).Equals("hvzjawwaeuJxdx"));
                Debug.Assert(pack.param_count == (ushort)(ushort)204);
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16;
            p22.param_index = (ushort)(ushort)16461;
            p22.param_id_SET("hvzjawwaeuJxdx", PH) ;
            p22.param_value = (float)2.69084E38F;
            p22.param_count = (ushort)(ushort)204;
            ADV_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
                Debug.Assert(pack.target_system == (byte)(byte)48);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("SpqUg"));
                Debug.Assert(pack.target_component == (byte)(byte)180);
                Debug.Assert(pack.param_value == (float)3.3263458E38F);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.target_system = (byte)(byte)48;
            p23.target_component = (byte)(byte)180;
            p23.param_value = (float)3.3263458E38F;
            p23.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32;
            p23.param_id_SET("SpqUg", PH) ;
            ADV_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (int) -2049310025);
                Debug.Assert(pack.vel == (ushort)(ushort)19945);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int)1626199051);
                Debug.Assert(pack.lon == (int) -1730138115);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)2355920660U);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)852250640U);
                Debug.Assert(pack.satellites_visible == (byte)(byte)196);
                Debug.Assert(pack.time_usec == (ulong)972519746572113791L);
                Debug.Assert(pack.cog == (ushort)(ushort)16328);
                Debug.Assert(pack.eph == (ushort)(ushort)20489);
                Debug.Assert(pack.epv == (ushort)(ushort)47736);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)1479950835U);
                Debug.Assert(pack.lat == (int)501769401);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)1540585957U);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.time_usec = (ulong)972519746572113791L;
            p24.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS;
            p24.eph = (ushort)(ushort)20489;
            p24.cog = (ushort)(ushort)16328;
            p24.epv = (ushort)(ushort)47736;
            p24.alt = (int) -2049310025;
            p24.lat = (int)501769401;
            p24.hdg_acc_SET((uint)1540585957U, PH) ;
            p24.alt_ellipsoid_SET((int)1626199051, PH) ;
            p24.v_acc_SET((uint)2355920660U, PH) ;
            p24.satellites_visible = (byte)(byte)196;
            p24.vel_acc_SET((uint)852250640U, PH) ;
            p24.lon = (int) -1730138115;
            p24.h_acc_SET((uint)1479950835U, PH) ;
            p24.vel = (ushort)(ushort)19945;
            ADV_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)32, (byte)41, (byte)134, (byte)0, (byte)142, (byte)242, (byte)42, (byte)166, (byte)138, (byte)51, (byte)111, (byte)3, (byte)25, (byte)173, (byte)91, (byte)33, (byte)104, (byte)126, (byte)146, (byte)199}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)232, (byte)166, (byte)251, (byte)99, (byte)138, (byte)108, (byte)54, (byte)27, (byte)18, (byte)116, (byte)146, (byte)211, (byte)217, (byte)26, (byte)228, (byte)49, (byte)49, (byte)203, (byte)7, (byte)96}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)116, (byte)219, (byte)55, (byte)160, (byte)115, (byte)246, (byte)146, (byte)186, (byte)156, (byte)181, (byte)102, (byte)147, (byte)25, (byte)149, (byte)163, (byte)104, (byte)92, (byte)68, (byte)205, (byte)128}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)242);
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)180, (byte)237, (byte)15, (byte)250, (byte)226, (byte)16, (byte)54, (byte)227, (byte)224, (byte)249, (byte)250, (byte)126, (byte)173, (byte)210, (byte)207, (byte)214, (byte)45, (byte)0, (byte)128, (byte)60}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)135, (byte)123, (byte)116, (byte)129, (byte)9, (byte)28, (byte)129, (byte)83, (byte)112, (byte)134, (byte)72, (byte)55, (byte)238, (byte)218, (byte)135, (byte)120, (byte)177, (byte)28, (byte)22, (byte)184}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_snr_SET(new byte[] {(byte)232, (byte)166, (byte)251, (byte)99, (byte)138, (byte)108, (byte)54, (byte)27, (byte)18, (byte)116, (byte)146, (byte)211, (byte)217, (byte)26, (byte)228, (byte)49, (byte)49, (byte)203, (byte)7, (byte)96}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)180, (byte)237, (byte)15, (byte)250, (byte)226, (byte)16, (byte)54, (byte)227, (byte)224, (byte)249, (byte)250, (byte)126, (byte)173, (byte)210, (byte)207, (byte)214, (byte)45, (byte)0, (byte)128, (byte)60}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)135, (byte)123, (byte)116, (byte)129, (byte)9, (byte)28, (byte)129, (byte)83, (byte)112, (byte)134, (byte)72, (byte)55, (byte)238, (byte)218, (byte)135, (byte)120, (byte)177, (byte)28, (byte)22, (byte)184}, 0) ;
            p25.satellites_visible = (byte)(byte)242;
            p25.satellite_elevation_SET(new byte[] {(byte)32, (byte)41, (byte)134, (byte)0, (byte)142, (byte)242, (byte)42, (byte)166, (byte)138, (byte)51, (byte)111, (byte)3, (byte)25, (byte)173, (byte)91, (byte)33, (byte)104, (byte)126, (byte)146, (byte)199}, 0) ;
            p25.satellite_used_SET(new byte[] {(byte)116, (byte)219, (byte)55, (byte)160, (byte)115, (byte)246, (byte)146, (byte)186, (byte)156, (byte)181, (byte)102, (byte)147, (byte)25, (byte)149, (byte)163, (byte)104, (byte)92, (byte)68, (byte)205, (byte)128}, 0) ;
            ADV_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (short)(short) -18675);
                Debug.Assert(pack.zmag == (short)(short)25426);
                Debug.Assert(pack.xacc == (short)(short)22481);
                Debug.Assert(pack.zacc == (short)(short)27036);
                Debug.Assert(pack.yacc == (short)(short)27630);
                Debug.Assert(pack.zgyro == (short)(short) -20809);
                Debug.Assert(pack.ymag == (short)(short)17378);
                Debug.Assert(pack.xgyro == (short)(short) -27551);
                Debug.Assert(pack.ygyro == (short)(short) -8327);
                Debug.Assert(pack.time_boot_ms == (uint)2152443333U);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.zmag = (short)(short)25426;
            p26.yacc = (short)(short)27630;
            p26.ygyro = (short)(short) -8327;
            p26.xmag = (short)(short) -18675;
            p26.time_boot_ms = (uint)2152443333U;
            p26.xgyro = (short)(short) -27551;
            p26.xacc = (short)(short)22481;
            p26.zgyro = (short)(short) -20809;
            p26.zacc = (short)(short)27036;
            p26.ymag = (short)(short)17378;
            ADV_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (short)(short)31369);
                Debug.Assert(pack.xgyro == (short)(short)10326);
                Debug.Assert(pack.zmag == (short)(short) -30329);
                Debug.Assert(pack.xmag == (short)(short) -16196);
                Debug.Assert(pack.yacc == (short)(short) -9284);
                Debug.Assert(pack.xacc == (short)(short) -1569);
                Debug.Assert(pack.zacc == (short)(short)7313);
                Debug.Assert(pack.ymag == (short)(short) -16787);
                Debug.Assert(pack.time_usec == (ulong)6569421092077482855L);
                Debug.Assert(pack.ygyro == (short)(short)23228);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.xacc = (short)(short) -1569;
            p27.time_usec = (ulong)6569421092077482855L;
            p27.xgyro = (short)(short)10326;
            p27.zacc = (short)(short)7313;
            p27.ymag = (short)(short) -16787;
            p27.xmag = (short)(short) -16196;
            p27.zmag = (short)(short) -30329;
            p27.yacc = (short)(short) -9284;
            p27.ygyro = (short)(short)23228;
            p27.zgyro = (short)(short)31369;
            ADV_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -2719);
                Debug.Assert(pack.press_diff2 == (short)(short)2064);
                Debug.Assert(pack.press_abs == (short)(short)30184);
                Debug.Assert(pack.time_usec == (ulong)6620471093515319398L);
                Debug.Assert(pack.press_diff1 == (short)(short)23702);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.press_diff2 = (short)(short)2064;
            p28.press_diff1 = (short)(short)23702;
            p28.temperature = (short)(short) -2719;
            p28.time_usec = (ulong)6620471093515319398L;
            p28.press_abs = (short)(short)30184;
            ADV_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1795318199U);
                Debug.Assert(pack.press_abs == (float) -1.5125732E37F);
                Debug.Assert(pack.press_diff == (float)1.893514E38F);
                Debug.Assert(pack.temperature == (short)(short) -4941);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.press_diff = (float)1.893514E38F;
            p29.time_boot_ms = (uint)1795318199U;
            p29.temperature = (short)(short) -4941;
            p29.press_abs = (float) -1.5125732E37F;
            ADV_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float) -2.9988607E38F);
                Debug.Assert(pack.yaw == (float)2.0806773E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4154923032U);
                Debug.Assert(pack.yawspeed == (float)5.820357E37F);
                Debug.Assert(pack.pitch == (float)1.8600878E38F);
                Debug.Assert(pack.roll == (float)2.6854202E38F);
                Debug.Assert(pack.pitchspeed == (float)1.6892877E37F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.pitchspeed = (float)1.6892877E37F;
            p30.yawspeed = (float)5.820357E37F;
            p30.rollspeed = (float) -2.9988607E38F;
            p30.roll = (float)2.6854202E38F;
            p30.yaw = (float)2.0806773E38F;
            p30.time_boot_ms = (uint)4154923032U;
            p30.pitch = (float)1.8600878E38F;
            ADV_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitchspeed == (float) -2.7686684E38F);
                Debug.Assert(pack.q2 == (float)8.630944E37F);
                Debug.Assert(pack.time_boot_ms == (uint)279933690U);
                Debug.Assert(pack.yawspeed == (float) -3.3707036E38F);
                Debug.Assert(pack.rollspeed == (float)7.245491E37F);
                Debug.Assert(pack.q3 == (float)2.559907E38F);
                Debug.Assert(pack.q4 == (float)3.0351174E38F);
                Debug.Assert(pack.q1 == (float) -1.3351031E38F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.time_boot_ms = (uint)279933690U;
            p31.pitchspeed = (float) -2.7686684E38F;
            p31.q4 = (float)3.0351174E38F;
            p31.rollspeed = (float)7.245491E37F;
            p31.yawspeed = (float) -3.3707036E38F;
            p31.q3 = (float)2.559907E38F;
            p31.q1 = (float) -1.3351031E38F;
            p31.q2 = (float)8.630944E37F;
            ADV_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float)8.014842E37F);
                Debug.Assert(pack.z == (float)2.2691925E38F);
                Debug.Assert(pack.vz == (float) -8.03055E37F);
                Debug.Assert(pack.y == (float) -1.1557993E38F);
                Debug.Assert(pack.vy == (float)3.0093867E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3468497998U);
                Debug.Assert(pack.x == (float)1.5778447E38F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.vy = (float)3.0093867E38F;
            p32.vx = (float)8.014842E37F;
            p32.vz = (float) -8.03055E37F;
            p32.x = (float)1.5778447E38F;
            p32.time_boot_ms = (uint)3468497998U;
            p32.z = (float)2.2691925E38F;
            p32.y = (float) -1.1557993E38F;
            ADV_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)736716723U);
                Debug.Assert(pack.lat == (int) -640073289);
                Debug.Assert(pack.lon == (int)102586696);
                Debug.Assert(pack.relative_alt == (int)1080140736);
                Debug.Assert(pack.vy == (short)(short) -13750);
                Debug.Assert(pack.hdg == (ushort)(ushort)31360);
                Debug.Assert(pack.alt == (int)1723225935);
                Debug.Assert(pack.vz == (short)(short)13889);
                Debug.Assert(pack.vx == (short)(short) -9629);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.vx = (short)(short) -9629;
            p33.lat = (int) -640073289;
            p33.hdg = (ushort)(ushort)31360;
            p33.vy = (short)(short) -13750;
            p33.vz = (short)(short)13889;
            p33.time_boot_ms = (uint)736716723U;
            p33.alt = (int)1723225935;
            p33.lon = (int)102586696;
            p33.relative_alt = (int)1080140736;
            ADV_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan6_scaled == (short)(short)30468);
                Debug.Assert(pack.time_boot_ms == (uint)62875643U);
                Debug.Assert(pack.chan2_scaled == (short)(short)10899);
                Debug.Assert(pack.chan4_scaled == (short)(short)10542);
                Debug.Assert(pack.chan8_scaled == (short)(short) -4876);
                Debug.Assert(pack.chan7_scaled == (short)(short)7862);
                Debug.Assert(pack.chan3_scaled == (short)(short)1121);
                Debug.Assert(pack.port == (byte)(byte)163);
                Debug.Assert(pack.chan1_scaled == (short)(short) -20321);
                Debug.Assert(pack.rssi == (byte)(byte)39);
                Debug.Assert(pack.chan5_scaled == (short)(short) -11820);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.port = (byte)(byte)163;
            p34.rssi = (byte)(byte)39;
            p34.chan3_scaled = (short)(short)1121;
            p34.chan2_scaled = (short)(short)10899;
            p34.chan6_scaled = (short)(short)30468;
            p34.time_boot_ms = (uint)62875643U;
            p34.chan1_scaled = (short)(short) -20321;
            p34.chan5_scaled = (short)(short) -11820;
            p34.chan8_scaled = (short)(short) -4876;
            p34.chan7_scaled = (short)(short)7862;
            p34.chan4_scaled = (short)(short)10542;
            ADV_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)10103);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)41367);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)2778);
                Debug.Assert(pack.port == (byte)(byte)229);
                Debug.Assert(pack.time_boot_ms == (uint)3453334257U);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)30017);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)44515);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)56344);
                Debug.Assert(pack.rssi == (byte)(byte)46);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)9545);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)20230);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan8_raw = (ushort)(ushort)30017;
            p35.chan2_raw = (ushort)(ushort)10103;
            p35.port = (byte)(byte)229;
            p35.chan6_raw = (ushort)(ushort)56344;
            p35.chan1_raw = (ushort)(ushort)2778;
            p35.time_boot_ms = (uint)3453334257U;
            p35.rssi = (byte)(byte)46;
            p35.chan3_raw = (ushort)(ushort)44515;
            p35.chan7_raw = (ushort)(ushort)41367;
            p35.chan5_raw = (ushort)(ushort)9545;
            p35.chan4_raw = (ushort)(ushort)20230;
            ADV_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)745);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)33819);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)20880);
                Debug.Assert(pack.time_usec == (uint)3968477115U);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)36578);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)3855);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)63658);
                Debug.Assert(pack.port == (byte)(byte)110);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)47568);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)43242);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)56473);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)27070);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)40254);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)15168);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)22551);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)33758);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)14798);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)30345);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo4_raw = (ushort)(ushort)3855;
            p36.servo3_raw = (ushort)(ushort)43242;
            p36.servo12_raw_SET((ushort)(ushort)47568, PH) ;
            p36.servo14_raw_SET((ushort)(ushort)14798, PH) ;
            p36.servo15_raw_SET((ushort)(ushort)63658, PH) ;
            p36.servo7_raw = (ushort)(ushort)56473;
            p36.servo8_raw = (ushort)(ushort)745;
            p36.servo13_raw_SET((ushort)(ushort)22551, PH) ;
            p36.servo1_raw = (ushort)(ushort)33819;
            p36.time_usec = (uint)3968477115U;
            p36.servo6_raw = (ushort)(ushort)30345;
            p36.port = (byte)(byte)110;
            p36.servo11_raw_SET((ushort)(ushort)20880, PH) ;
            p36.servo5_raw = (ushort)(ushort)15168;
            p36.servo2_raw = (ushort)(ushort)40254;
            p36.servo16_raw_SET((ushort)(ushort)27070, PH) ;
            p36.servo9_raw_SET((ushort)(ushort)33758, PH) ;
            p36.servo10_raw_SET((ushort)(ushort)36578, PH) ;
            ADV_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_index == (short)(short) -5452);
                Debug.Assert(pack.target_system == (byte)(byte)89);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_component == (byte)(byte)122);
                Debug.Assert(pack.end_index == (short)(short) -16348);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.start_index = (short)(short) -5452;
            p37.target_system = (byte)(byte)89;
            p37.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p37.end_index = (short)(short) -16348;
            p37.target_component = (byte)(byte)122;
            ADV_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)167);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.start_index == (short)(short)1071);
                Debug.Assert(pack.end_index == (short)(short)15501);
                Debug.Assert(pack.target_system == (byte)(byte)117);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p38.start_index = (short)(short)1071;
            p38.target_component = (byte)(byte)167;
            p38.end_index = (short)(short)15501;
            p38.target_system = (byte)(byte)117;
            ADV_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.z == (float) -1.8351638E38F);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.current == (byte)(byte)129);
                Debug.Assert(pack.param2 == (float) -9.344716E37F);
                Debug.Assert(pack.x == (float)7.3782625E37F);
                Debug.Assert(pack.param3 == (float) -1.5766392E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)113);
                Debug.Assert(pack.y == (float) -2.5210776E38F);
                Debug.Assert(pack.target_component == (byte)(byte)176);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_USER_1);
                Debug.Assert(pack.param1 == (float) -2.5414097E38F);
                Debug.Assert(pack.param4 == (float) -1.1769354E38F);
                Debug.Assert(pack.target_system == (byte)(byte)253);
                Debug.Assert(pack.seq == (ushort)(ushort)4616);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p39.x = (float)7.3782625E37F;
            p39.y = (float) -2.5210776E38F;
            p39.param3 = (float) -1.5766392E38F;
            p39.target_component = (byte)(byte)176;
            p39.param1 = (float) -2.5414097E38F;
            p39.current = (byte)(byte)129;
            p39.param2 = (float) -9.344716E37F;
            p39.command = MAV_CMD.MAV_CMD_USER_1;
            p39.z = (float) -1.8351638E38F;
            p39.seq = (ushort)(ushort)4616;
            p39.autocontinue = (byte)(byte)113;
            p39.frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p39.param4 = (float) -1.1769354E38F;
            p39.target_system = (byte)(byte)253;
            ADV_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)226);
                Debug.Assert(pack.seq == (ushort)(ushort)48222);
                Debug.Assert(pack.target_system == (byte)(byte)99);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p40.seq = (ushort)(ushort)48222;
            p40.target_component = (byte)(byte)226;
            p40.target_system = (byte)(byte)99;
            ADV_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)50);
                Debug.Assert(pack.seq == (ushort)(ushort)25962);
                Debug.Assert(pack.target_system == (byte)(byte)138);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_component = (byte)(byte)50;
            p41.seq = (ushort)(ushort)25962;
            p41.target_system = (byte)(byte)138;
            ADV_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)33988);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)33988;
            ADV_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)236);
                Debug.Assert(pack.target_component == (byte)(byte)86);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.target_component = (byte)(byte)86;
            p43.target_system = (byte)(byte)236;
            p43.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            ADV_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (ushort)(ushort)40501);
                Debug.Assert(pack.target_system == (byte)(byte)112);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)87);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.target_system = (byte)(byte)112;
            p44.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p44.count = (ushort)(ushort)40501;
            p44.target_component = (byte)(byte)87;
            ADV_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)23);
                Debug.Assert(pack.target_system == (byte)(byte)198);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_component = (byte)(byte)23;
            p45.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p45.target_system = (byte)(byte)198;
            ADV_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)13842);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)13842;
            ADV_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)24);
                Debug.Assert(pack.target_system == (byte)(byte)62);
                Debug.Assert(pack.type == MAV_MISSION_RESULT.MAV_MISSION_INVALID_SEQUENCE);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.type = MAV_MISSION_RESULT.MAV_MISSION_INVALID_SEQUENCE;
            p47.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p47.target_component = (byte)(byte)24;
            p47.target_system = (byte)(byte)62;
            ADV_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int) -2074383762);
                Debug.Assert(pack.longitude == (int) -107324695);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)8593545667907436277L);
                Debug.Assert(pack.target_system == (byte)(byte)6);
                Debug.Assert(pack.latitude == (int)1369703072);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.target_system = (byte)(byte)6;
            p48.altitude = (int) -2074383762;
            p48.latitude = (int)1369703072;
            p48.time_usec_SET((ulong)8593545667907436277L, PH) ;
            p48.longitude = (int) -107324695;
            ADV_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int) -1259778520);
                Debug.Assert(pack.longitude == (int) -204231472);
                Debug.Assert(pack.latitude == (int)438463668);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)6098187653909283929L);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.altitude = (int) -1259778520;
            p49.longitude = (int) -204231472;
            p49.latitude = (int)438463668;
            p49.time_usec_SET((ulong)6098187653909283929L, PH) ;
            ADV_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.scale == (float)2.281836E38F);
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("rai"));
                Debug.Assert(pack.target_system == (byte)(byte)4);
                Debug.Assert(pack.param_value_max == (float) -3.3083093E38F);
                Debug.Assert(pack.param_value_min == (float)2.1139439E37F);
                Debug.Assert(pack.param_value0 == (float) -1.8034733E38F);
                Debug.Assert(pack.target_component == (byte)(byte)250);
                Debug.Assert(pack.param_index == (short)(short) -9082);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)103);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.param_index = (short)(short) -9082;
            p50.target_system = (byte)(byte)4;
            p50.target_component = (byte)(byte)250;
            p50.param_value_max = (float) -3.3083093E38F;
            p50.param_value_min = (float)2.1139439E37F;
            p50.scale = (float)2.281836E38F;
            p50.param_id_SET("rai", PH) ;
            p50.param_value0 = (float) -1.8034733E38F;
            p50.parameter_rc_channel_index = (byte)(byte)103;
            ADV_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.target_system == (byte)(byte)56);
                Debug.Assert(pack.target_component == (byte)(byte)18);
                Debug.Assert(pack.seq == (ushort)(ushort)20480);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.seq = (ushort)(ushort)20480;
            p51.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p51.target_component = (byte)(byte)18;
            p51.target_system = (byte)(byte)56;
            ADV_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.p1x == (float) -1.9520667E37F);
                Debug.Assert(pack.p2z == (float) -2.9230066E38F);
                Debug.Assert(pack.target_system == (byte)(byte)61);
                Debug.Assert(pack.p1y == (float)6.8119076E37F);
                Debug.Assert(pack.p2y == (float)1.3142087E38F);
                Debug.Assert(pack.p2x == (float) -1.2381852E38F);
                Debug.Assert(pack.p1z == (float) -2.8558067E38F);
                Debug.Assert(pack.target_component == (byte)(byte)130);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.target_system = (byte)(byte)61;
            p54.p2x = (float) -1.2381852E38F;
            p54.p1y = (float)6.8119076E37F;
            p54.p1x = (float) -1.9520667E37F;
            p54.frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p54.p2y = (float)1.3142087E38F;
            p54.target_component = (byte)(byte)130;
            p54.p1z = (float) -2.8558067E38F;
            p54.p2z = (float) -2.9230066E38F;
            ADV_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2x == (float) -3.3736586E38F);
                Debug.Assert(pack.p1z == (float) -2.7924394E38F);
                Debug.Assert(pack.p2y == (float) -2.4942508E38F);
                Debug.Assert(pack.p1x == (float)2.7825643E38F);
                Debug.Assert(pack.p1y == (float) -2.0949145E38F);
                Debug.Assert(pack.p2z == (float)6.0655565E37F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_MISSION);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.frame = MAV_FRAME.MAV_FRAME_MISSION;
            p55.p1x = (float)2.7825643E38F;
            p55.p2x = (float) -3.3736586E38F;
            p55.p2y = (float) -2.4942508E38F;
            p55.p1y = (float) -2.0949145E38F;
            p55.p2z = (float)6.0655565E37F;
            p55.p1z = (float) -2.7924394E38F;
            ADV_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float)1.5707159E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.4845441E38F, 1.7544303E38F, -3.3989122E38F, -8.661678E36F}));
                Debug.Assert(pack.pitchspeed == (float)1.617282E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {4.918474E37F, -1.7348216E38F, -2.2133865E38F, 1.758374E38F, -2.7059736E38F, -8.536202E37F, -3.4027703E38F, -6.5792395E37F, 5.845518E37F}));
                Debug.Assert(pack.yawspeed == (float) -2.3170016E38F);
                Debug.Assert(pack.time_usec == (ulong)1664745237762227498L);
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.pitchspeed = (float)1.617282E38F;
            p61.yawspeed = (float) -2.3170016E38F;
            p61.rollspeed = (float)1.5707159E38F;
            p61.time_usec = (ulong)1664745237762227498L;
            p61.q_SET(new float[] {-1.4845441E38F, 1.7544303E38F, -3.3989122E38F, -8.661678E36F}, 0) ;
            p61.covariance_SET(new float[] {4.918474E37F, -1.7348216E38F, -2.2133865E38F, 1.758374E38F, -2.7059736E38F, -8.536202E37F, -3.4027703E38F, -6.5792395E37F, 5.845518E37F}, 0) ;
            ADV_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nav_roll == (float)1.8984457E38F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)2436);
                Debug.Assert(pack.nav_pitch == (float)1.9972786E38F);
                Debug.Assert(pack.xtrack_error == (float) -1.4733566E38F);
                Debug.Assert(pack.nav_bearing == (short)(short)3789);
                Debug.Assert(pack.alt_error == (float)1.0017156E38F);
                Debug.Assert(pack.aspd_error == (float) -1.3990978E38F);
                Debug.Assert(pack.target_bearing == (short)(short) -9769);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_bearing = (short)(short)3789;
            p62.nav_roll = (float)1.8984457E38F;
            p62.xtrack_error = (float) -1.4733566E38F;
            p62.target_bearing = (short)(short) -9769;
            p62.wp_dist = (ushort)(ushort)2436;
            p62.alt_error = (float)1.0017156E38F;
            p62.nav_pitch = (float)1.9972786E38F;
            p62.aspd_error = (float) -1.3990978E38F;
            ADV_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1916144085);
                Debug.Assert(pack.lon == (int) -1695937877);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
                Debug.Assert(pack.alt == (int)2076795822);
                Debug.Assert(pack.time_usec == (ulong)9216898505694095335L);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-1.6270708E38F, -2.2786141E38F, 6.925196E37F, 1.5143368E38F, 5.611342E37F, 2.3098638E38F, -2.576118E38F, 3.3620698E38F, -3.3706095E38F, -2.5993586E37F, 2.0507496E38F, 6.476839E37F, 2.8070399E37F, -2.3920463E38F, 2.7261005E38F, 1.070673E38F, -8.58724E37F, 3.3829704E38F, 3.165528E37F, -2.7013863E38F, -1.8570006E38F, -8.202027E36F, -1.5350479E38F, -8.3873615E37F, 5.639833E37F, 5.510981E37F, 3.2613646E38F, -2.967579E38F, -2.0815188E38F, 2.8697005E38F, 7.1658357E37F, 7.2202387E37F, 1.9919799E38F, 1.804006E38F, -3.6831297E37F, 2.342741E37F}));
                Debug.Assert(pack.vz == (float) -1.3212224E38F);
                Debug.Assert(pack.relative_alt == (int)1560868979);
                Debug.Assert(pack.vy == (float)1.3526017E38F);
                Debug.Assert(pack.vx == (float) -2.622509E38F);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.lon = (int) -1695937877;
            p63.vx = (float) -2.622509E38F;
            p63.covariance_SET(new float[] {-1.6270708E38F, -2.2786141E38F, 6.925196E37F, 1.5143368E38F, 5.611342E37F, 2.3098638E38F, -2.576118E38F, 3.3620698E38F, -3.3706095E38F, -2.5993586E37F, 2.0507496E38F, 6.476839E37F, 2.8070399E37F, -2.3920463E38F, 2.7261005E38F, 1.070673E38F, -8.58724E37F, 3.3829704E38F, 3.165528E37F, -2.7013863E38F, -1.8570006E38F, -8.202027E36F, -1.5350479E38F, -8.3873615E37F, 5.639833E37F, 5.510981E37F, 3.2613646E38F, -2.967579E38F, -2.0815188E38F, 2.8697005E38F, 7.1658357E37F, 7.2202387E37F, 1.9919799E38F, 1.804006E38F, -3.6831297E37F, 2.342741E37F}, 0) ;
            p63.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO;
            p63.relative_alt = (int)1560868979;
            p63.vy = (float)1.3526017E38F;
            p63.vz = (float) -1.3212224E38F;
            p63.alt = (int)2076795822;
            p63.time_usec = (ulong)9216898505694095335L;
            p63.lat = (int)1916144085;
            ADV_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float) -3.1991844E38F);
                Debug.Assert(pack.ay == (float) -3.2198678E38F);
                Debug.Assert(pack.ax == (float) -2.943407E37F);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE);
                Debug.Assert(pack.time_usec == (ulong)5883388365008105699L);
                Debug.Assert(pack.az == (float) -3.0588044E38F);
                Debug.Assert(pack.y == (float)1.1214747E38F);
                Debug.Assert(pack.z == (float)2.5268666E38F);
                Debug.Assert(pack.vz == (float)1.4695924E38F);
                Debug.Assert(pack.vx == (float) -1.6759004E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-1.778168E37F, -1.1700607E38F, -6.220493E37F, -2.8208398E38F, -1.4410542E38F, 4.792944E37F, -2.1199268E38F, 1.9717942E38F, -4.2426544E37F, -2.5000964E38F, -2.3936395E38F, -2.401808E38F, 3.1818669E38F, 5.957806E37F, -6.831846E37F, -2.0037925E38F, -2.5028477E38F, 3.3526618E37F, 2.432923E38F, 1.0531898E38F, 1.1513723E38F, 3.2926437E38F, 2.5241695E37F, 2.4128273E38F, -1.1835917E38F, 2.3407846E38F, -5.2006674E37F, 1.036139E38F, 4.742035E37F, 2.4214656E38F, -2.1087278E38F, 1.0538053E38F, 9.329107E37F, -1.4031562E38F, 4.5870282E36F, -2.5772617E38F, -1.534342E38F, 7.4327653E37F, -1.2316662E37F, -2.0158798E38F, -2.714682E38F, -2.2183352E38F, 3.017409E38F, 1.319521E38F, 1.85897E38F}));
                Debug.Assert(pack.x == (float)1.4075972E38F);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.vx = (float) -1.6759004E38F;
            p64.time_usec = (ulong)5883388365008105699L;
            p64.vy = (float) -3.1991844E38F;
            p64.covariance_SET(new float[] {-1.778168E37F, -1.1700607E38F, -6.220493E37F, -2.8208398E38F, -1.4410542E38F, 4.792944E37F, -2.1199268E38F, 1.9717942E38F, -4.2426544E37F, -2.5000964E38F, -2.3936395E38F, -2.401808E38F, 3.1818669E38F, 5.957806E37F, -6.831846E37F, -2.0037925E38F, -2.5028477E38F, 3.3526618E37F, 2.432923E38F, 1.0531898E38F, 1.1513723E38F, 3.2926437E38F, 2.5241695E37F, 2.4128273E38F, -1.1835917E38F, 2.3407846E38F, -5.2006674E37F, 1.036139E38F, 4.742035E37F, 2.4214656E38F, -2.1087278E38F, 1.0538053E38F, 9.329107E37F, -1.4031562E38F, 4.5870282E36F, -2.5772617E38F, -1.534342E38F, 7.4327653E37F, -1.2316662E37F, -2.0158798E38F, -2.714682E38F, -2.2183352E38F, 3.017409E38F, 1.319521E38F, 1.85897E38F}, 0) ;
            p64.ay = (float) -3.2198678E38F;
            p64.vz = (float)1.4695924E38F;
            p64.ax = (float) -2.943407E37F;
            p64.x = (float)1.4075972E38F;
            p64.z = (float)2.5268666E38F;
            p64.y = (float)1.1214747E38F;
            p64.az = (float) -3.0588044E38F;
            p64.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_NAIVE;
            ADV_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)64778);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)36563);
                Debug.Assert(pack.time_boot_ms == (uint)2661746609U);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)52586);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)40819);
                Debug.Assert(pack.chancount == (byte)(byte)24);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)57764);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)11425);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)7084);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)4416);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)2600);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)41621);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)25055);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)40675);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)21950);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)7316);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)38740);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)8703);
                Debug.Assert(pack.rssi == (byte)(byte)193);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)1999);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)48135);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan8_raw = (ushort)(ushort)11425;
            p65.chan9_raw = (ushort)(ushort)38740;
            p65.chan14_raw = (ushort)(ushort)2600;
            p65.chan1_raw = (ushort)(ushort)7316;
            p65.chan12_raw = (ushort)(ushort)40819;
            p65.chan7_raw = (ushort)(ushort)25055;
            p65.rssi = (byte)(byte)193;
            p65.chan6_raw = (ushort)(ushort)36563;
            p65.chan17_raw = (ushort)(ushort)21950;
            p65.chan11_raw = (ushort)(ushort)40675;
            p65.chan16_raw = (ushort)(ushort)57764;
            p65.chancount = (byte)(byte)24;
            p65.chan10_raw = (ushort)(ushort)4416;
            p65.chan3_raw = (ushort)(ushort)41621;
            p65.chan5_raw = (ushort)(ushort)52586;
            p65.chan2_raw = (ushort)(ushort)64778;
            p65.chan13_raw = (ushort)(ushort)1999;
            p65.chan15_raw = (ushort)(ushort)48135;
            p65.chan4_raw = (ushort)(ushort)8703;
            p65.chan18_raw = (ushort)(ushort)7084;
            p65.time_boot_ms = (uint)2661746609U;
            ADV_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_stop == (byte)(byte)219);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)1654);
                Debug.Assert(pack.target_system == (byte)(byte)17);
                Debug.Assert(pack.req_stream_id == (byte)(byte)210);
                Debug.Assert(pack.target_component == (byte)(byte)106);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.start_stop = (byte)(byte)219;
            p66.target_system = (byte)(byte)17;
            p66.target_component = (byte)(byte)106;
            p66.req_message_rate = (ushort)(ushort)1654;
            p66.req_stream_id = (byte)(byte)210;
            ADV_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.on_off == (byte)(byte)81);
                Debug.Assert(pack.message_rate == (ushort)(ushort)41845);
                Debug.Assert(pack.stream_id == (byte)(byte)34);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.message_rate = (ushort)(ushort)41845;
            p67.on_off = (byte)(byte)81;
            p67.stream_id = (byte)(byte)34;
            ADV_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.buttons == (ushort)(ushort)30415);
                Debug.Assert(pack.target == (byte)(byte)202);
                Debug.Assert(pack.z == (short)(short)23743);
                Debug.Assert(pack.x == (short)(short) -18863);
                Debug.Assert(pack.y == (short)(short)12222);
                Debug.Assert(pack.r == (short)(short) -5213);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.x = (short)(short) -18863;
            p69.r = (short)(short) -5213;
            p69.buttons = (ushort)(ushort)30415;
            p69.z = (short)(short)23743;
            p69.target = (byte)(byte)202;
            p69.y = (short)(short)12222;
            ADV_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)49799);
                Debug.Assert(pack.target_system == (byte)(byte)184);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)48298);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)41953);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)17280);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)5158);
                Debug.Assert(pack.target_component == (byte)(byte)128);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)49286);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)8678);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)11964);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan4_raw = (ushort)(ushort)8678;
            p70.chan1_raw = (ushort)(ushort)11964;
            p70.chan8_raw = (ushort)(ushort)5158;
            p70.chan2_raw = (ushort)(ushort)48298;
            p70.chan7_raw = (ushort)(ushort)49286;
            p70.target_component = (byte)(byte)128;
            p70.target_system = (byte)(byte)184;
            p70.chan5_raw = (ushort)(ushort)41953;
            p70.chan3_raw = (ushort)(ushort)49799;
            p70.chan6_raw = (ushort)(ushort)17280;
            ADV_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)210);
                Debug.Assert(pack.current == (byte)(byte)193);
                Debug.Assert(pack.seq == (ushort)(ushort)36283);
                Debug.Assert(pack.param4 == (float) -1.2533066E38F);
                Debug.Assert(pack.param2 == (float) -3.0714014E38F);
                Debug.Assert(pack.param1 == (float)2.3170521E38F);
                Debug.Assert(pack.z == (float) -3.2042495E38F);
                Debug.Assert(pack.y == (int) -261736278);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS);
                Debug.Assert(pack.target_system == (byte)(byte)96);
                Debug.Assert(pack.autocontinue == (byte)(byte)113);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.x == (int) -1565340473);
                Debug.Assert(pack.param3 == (float)7.3612876E37F);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p73.param1 = (float)2.3170521E38F;
            p73.frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p73.param2 = (float) -3.0714014E38F;
            p73.autocontinue = (byte)(byte)113;
            p73.z = (float) -3.2042495E38F;
            p73.param3 = (float)7.3612876E37F;
            p73.y = (int) -261736278;
            p73.seq = (ushort)(ushort)36283;
            p73.target_system = (byte)(byte)96;
            p73.current = (byte)(byte)193;
            p73.command = MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS;
            p73.x = (int) -1565340473;
            p73.param4 = (float) -1.2533066E38F;
            p73.target_component = (byte)(byte)210;
            ADV_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (float)1.8957843E38F);
                Debug.Assert(pack.climb == (float) -8.533635E37F);
                Debug.Assert(pack.throttle == (ushort)(ushort)28991);
                Debug.Assert(pack.heading == (short)(short) -9294);
                Debug.Assert(pack.groundspeed == (float) -3.3531565E38F);
                Debug.Assert(pack.airspeed == (float)2.58815E38F);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.groundspeed = (float) -3.3531565E38F;
            p74.climb = (float) -8.533635E37F;
            p74.heading = (short)(short) -9294;
            p74.throttle = (ushort)(ushort)28991;
            p74.alt = (float)1.8957843E38F;
            p74.airspeed = (float)2.58815E38F;
            ADV_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (int)2145494417);
                Debug.Assert(pack.param2 == (float)1.945002E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE);
                Debug.Assert(pack.param3 == (float) -9.812846E37F);
                Debug.Assert(pack.current == (byte)(byte)202);
                Debug.Assert(pack.target_component == (byte)(byte)220);
                Debug.Assert(pack.target_system == (byte)(byte)134);
                Debug.Assert(pack.param4 == (float) -1.4185142E38F);
                Debug.Assert(pack.x == (int)1973125689);
                Debug.Assert(pack.autocontinue == (byte)(byte)136);
                Debug.Assert(pack.z == (float)3.0954642E38F);
                Debug.Assert(pack.param1 == (float) -9.721442E37F);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.autocontinue = (byte)(byte)136;
            p75.param1 = (float) -9.721442E37F;
            p75.x = (int)1973125689;
            p75.target_system = (byte)(byte)134;
            p75.y = (int)2145494417;
            p75.param2 = (float)1.945002E38F;
            p75.command = MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
            p75.target_component = (byte)(byte)220;
            p75.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p75.z = (float)3.0954642E38F;
            p75.param3 = (float) -9.812846E37F;
            p75.param4 = (float) -1.4185142E38F;
            p75.current = (byte)(byte)202;
            ADV_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)190);
                Debug.Assert(pack.param6 == (float) -2.9529142E38F);
                Debug.Assert(pack.param7 == (float) -2.85362E38F);
                Debug.Assert(pack.param1 == (float) -9.914128E37F);
                Debug.Assert(pack.confirmation == (byte)(byte)21);
                Debug.Assert(pack.param3 == (float) -3.2527762E38F);
                Debug.Assert(pack.param4 == (float)2.7870057E38F);
                Debug.Assert(pack.param2 == (float)1.7243498E38F);
                Debug.Assert(pack.target_component == (byte)(byte)233);
                Debug.Assert(pack.param5 == (float) -4.0785697E37F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST);
            };
            COMMAND_LONG p76 = new COMMAND_LONG();
            PH.setPack(p76);
            p76.target_component = (byte)(byte)233;
            p76.param2 = (float)1.7243498E38F;
            p76.param6 = (float) -2.9529142E38F;
            p76.target_system = (byte)(byte)190;
            p76.param1 = (float) -9.914128E37F;
            p76.param7 = (float) -2.85362E38F;
            p76.param3 = (float) -3.2527762E38F;
            p76.confirmation = (byte)(byte)21;
            p76.command = MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST;
            p76.param4 = (float)2.7870057E38F;
            p76.param5 = (float) -4.0785697E37F;
            ADV_TEST_CH.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result == MAV_RESULT.MAV_RESULT_DENIED);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)124);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)64);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)82);
                Debug.Assert(pack.result_param2_TRY(ph) == (int)1421419054);
            };
            GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.target_system_SET((byte)(byte)124, PH) ;
            p77.result_param2_SET((int)1421419054, PH) ;
            p77.progress_SET((byte)(byte)82, PH) ;
            p77.result = MAV_RESULT.MAV_RESULT_DENIED;
            p77.target_component_SET((byte)(byte)64, PH) ;
            p77.command = MAV_CMD.MAV_CMD_REQUEST_CAMERA_SETTINGS;
            CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1951168193U);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)211);
                Debug.Assert(pack.yaw == (float)2.5287092E38F);
                Debug.Assert(pack.pitch == (float)2.7909103E38F);
                Debug.Assert(pack.mode_switch == (byte)(byte)188);
                Debug.Assert(pack.roll == (float)7.7589364E37F);
                Debug.Assert(pack.thrust == (float) -2.6170578E38F);
            };
            GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.yaw = (float)2.5287092E38F;
            p81.roll = (float)7.7589364E37F;
            p81.thrust = (float) -2.6170578E38F;
            p81.time_boot_ms = (uint)1951168193U;
            p81.pitch = (float)2.7909103E38F;
            p81.manual_override_switch = (byte)(byte)211;
            p81.mode_switch = (byte)(byte)188;
            CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_yaw_rate == (float)9.978171E37F);
                Debug.Assert(pack.body_roll_rate == (float) -4.867514E37F);
                Debug.Assert(pack.type_mask == (byte)(byte)124);
                Debug.Assert(pack.body_pitch_rate == (float)2.810176E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4120117512U);
                Debug.Assert(pack.target_component == (byte)(byte)162);
                Debug.Assert(pack.target_system == (byte)(byte)25);
                Debug.Assert(pack.thrust == (float) -2.9418737E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.0116721E38F, 3.2580657E38F, 3.025817E38F, 3.1970488E37F}));
            };
            GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.target_component = (byte)(byte)162;
            p82.body_roll_rate = (float) -4.867514E37F;
            p82.thrust = (float) -2.9418737E38F;
            p82.body_yaw_rate = (float)9.978171E37F;
            p82.type_mask = (byte)(byte)124;
            p82.time_boot_ms = (uint)4120117512U;
            p82.q_SET(new float[] {-1.0116721E38F, 3.2580657E38F, 3.025817E38F, 3.1970488E37F}, 0) ;
            p82.target_system = (byte)(byte)25;
            p82.body_pitch_rate = (float)2.810176E38F;
            CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type_mask == (byte)(byte)227);
                Debug.Assert(pack.thrust == (float) -1.6733246E38F);
                Debug.Assert(pack.time_boot_ms == (uint)537334263U);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.8826277E38F, 1.6481388E37F, 6.1474564E37F, -4.592028E37F}));
                Debug.Assert(pack.body_yaw_rate == (float)1.7309081E38F);
                Debug.Assert(pack.body_roll_rate == (float)1.4004346E38F);
                Debug.Assert(pack.body_pitch_rate == (float)3.4016185E38F);
            };
            GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.q_SET(new float[] {2.8826277E38F, 1.6481388E37F, 6.1474564E37F, -4.592028E37F}, 0) ;
            p83.thrust = (float) -1.6733246E38F;
            p83.body_roll_rate = (float)1.4004346E38F;
            p83.time_boot_ms = (uint)537334263U;
            p83.type_mask = (byte)(byte)227;
            p83.body_pitch_rate = (float)3.4016185E38F;
            p83.body_yaw_rate = (float)1.7309081E38F;
            CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)3.2016331E38F);
                Debug.Assert(pack.afy == (float)2.9548299E38F);
                Debug.Assert(pack.z == (float)2.1615148E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)64374);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.afz == (float)9.294083E37F);
                Debug.Assert(pack.yaw_rate == (float) -1.2590474E38F);
                Debug.Assert(pack.target_component == (byte)(byte)207);
                Debug.Assert(pack.vy == (float)1.417632E37F);
                Debug.Assert(pack.target_system == (byte)(byte)232);
                Debug.Assert(pack.time_boot_ms == (uint)3467946265U);
                Debug.Assert(pack.y == (float)7.7750405E36F);
                Debug.Assert(pack.vx == (float) -2.884297E38F);
                Debug.Assert(pack.vz == (float) -5.746596E37F);
                Debug.Assert(pack.yaw == (float)3.2313665E38F);
                Debug.Assert(pack.afx == (float)1.6446269E38F);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.x = (float)3.2016331E38F;
            p84.target_component = (byte)(byte)207;
            p84.type_mask = (ushort)(ushort)64374;
            p84.vx = (float) -2.884297E38F;
            p84.z = (float)2.1615148E38F;
            p84.afx = (float)1.6446269E38F;
            p84.time_boot_ms = (uint)3467946265U;
            p84.vz = (float) -5.746596E37F;
            p84.yaw = (float)3.2313665E38F;
            p84.vy = (float)1.417632E37F;
            p84.yaw_rate = (float) -1.2590474E38F;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p84.afz = (float)9.294083E37F;
            p84.y = (float)7.7750405E36F;
            p84.target_system = (byte)(byte)232;
            p84.afy = (float)2.9548299E38F;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (float) -1.4255716E38F);
                Debug.Assert(pack.afx == (float)2.4071458E38F);
                Debug.Assert(pack.lon_int == (int) -897418617);
                Debug.Assert(pack.lat_int == (int)357006000);
                Debug.Assert(pack.vz == (float)1.3753478E38F);
                Debug.Assert(pack.vx == (float) -2.4900315E38F);
                Debug.Assert(pack.target_system == (byte)(byte)45);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_BODY_NED);
                Debug.Assert(pack.afz == (float) -2.057515E38F);
                Debug.Assert(pack.target_component == (byte)(byte)205);
                Debug.Assert(pack.vy == (float) -1.0194108E38F);
                Debug.Assert(pack.afy == (float) -1.9206291E37F);
                Debug.Assert(pack.yaw_rate == (float) -3.1305552E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)56500);
                Debug.Assert(pack.time_boot_ms == (uint)2247603515U);
                Debug.Assert(pack.yaw == (float) -6.3691685E37F);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.lat_int = (int)357006000;
            p86.vy = (float) -1.0194108E38F;
            p86.vx = (float) -2.4900315E38F;
            p86.target_system = (byte)(byte)45;
            p86.alt = (float) -1.4255716E38F;
            p86.lon_int = (int) -897418617;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_BODY_NED;
            p86.yaw = (float) -6.3691685E37F;
            p86.afx = (float)2.4071458E38F;
            p86.vz = (float)1.3753478E38F;
            p86.target_component = (byte)(byte)205;
            p86.time_boot_ms = (uint)2247603515U;
            p86.afy = (float) -1.9206291E37F;
            p86.yaw_rate = (float) -3.1305552E38F;
            p86.type_mask = (ushort)(ushort)56500;
            p86.afz = (float) -2.057515E38F;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afy == (float) -3.1000446E38F);
                Debug.Assert(pack.afx == (float) -2.808218E38F);
                Debug.Assert(pack.alt == (float) -2.7011172E38F);
                Debug.Assert(pack.vz == (float)3.2494931E38F);
                Debug.Assert(pack.lon_int == (int)573446507);
                Debug.Assert(pack.time_boot_ms == (uint)2355886132U);
                Debug.Assert(pack.vx == (float)4.127579E37F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)58115);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.yaw == (float) -8.470845E37F);
                Debug.Assert(pack.yaw_rate == (float)1.1536404E38F);
                Debug.Assert(pack.afz == (float)2.4030092E38F);
                Debug.Assert(pack.vy == (float) -2.0909055E38F);
                Debug.Assert(pack.lat_int == (int)92516451);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.alt = (float) -2.7011172E38F;
            p87.lat_int = (int)92516451;
            p87.lon_int = (int)573446507;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_MISSION;
            p87.vx = (float)4.127579E37F;
            p87.type_mask = (ushort)(ushort)58115;
            p87.afz = (float)2.4030092E38F;
            p87.yaw = (float) -8.470845E37F;
            p87.vy = (float) -2.0909055E38F;
            p87.afx = (float) -2.808218E38F;
            p87.afy = (float) -3.1000446E38F;
            p87.vz = (float)3.2494931E38F;
            p87.time_boot_ms = (uint)2355886132U;
            p87.yaw_rate = (float)1.1536404E38F;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -4.4808614E37F);
                Debug.Assert(pack.roll == (float) -2.9790692E38F);
                Debug.Assert(pack.y == (float)1.4307661E38F);
                Debug.Assert(pack.pitch == (float)1.7410504E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3704302226U);
                Debug.Assert(pack.x == (float) -3.2916178E37F);
                Debug.Assert(pack.z == (float)5.2371474E37F);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.z = (float)5.2371474E37F;
            p89.y = (float)1.4307661E38F;
            p89.time_boot_ms = (uint)3704302226U;
            p89.roll = (float) -2.9790692E38F;
            p89.x = (float) -3.2916178E37F;
            p89.pitch = (float)1.7410504E38F;
            p89.yaw = (float) -4.4808614E37F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (short)(short) -23122);
                Debug.Assert(pack.roll == (float) -1.2539362E38F);
                Debug.Assert(pack.yacc == (short)(short)9181);
                Debug.Assert(pack.rollspeed == (float) -9.16117E37F);
                Debug.Assert(pack.lat == (int) -1599393394);
                Debug.Assert(pack.vx == (short)(short) -26899);
                Debug.Assert(pack.time_usec == (ulong)3674821361516196246L);
                Debug.Assert(pack.yaw == (float)6.990184E37F);
                Debug.Assert(pack.vy == (short)(short)19885);
                Debug.Assert(pack.lon == (int)2141260248);
                Debug.Assert(pack.zacc == (short)(short) -21079);
                Debug.Assert(pack.alt == (int)1504747875);
                Debug.Assert(pack.xacc == (short)(short)26096);
                Debug.Assert(pack.pitch == (float)3.2147098E38F);
                Debug.Assert(pack.pitchspeed == (float) -2.2158591E38F);
                Debug.Assert(pack.yawspeed == (float)3.3028766E37F);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.roll = (float) -1.2539362E38F;
            p90.yacc = (short)(short)9181;
            p90.vy = (short)(short)19885;
            p90.yaw = (float)6.990184E37F;
            p90.pitch = (float)3.2147098E38F;
            p90.alt = (int)1504747875;
            p90.vz = (short)(short) -23122;
            p90.vx = (short)(short) -26899;
            p90.yawspeed = (float)3.3028766E37F;
            p90.xacc = (short)(short)26096;
            p90.lat = (int) -1599393394;
            p90.lon = (int)2141260248;
            p90.time_usec = (ulong)3674821361516196246L;
            p90.pitchspeed = (float) -2.2158591E38F;
            p90.rollspeed = (float) -9.16117E37F;
            p90.zacc = (short)(short) -21079;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aux3 == (float) -2.898332E38F);
                Debug.Assert(pack.yaw_rudder == (float) -1.625526E38F);
                Debug.Assert(pack.throttle == (float)2.4543861E38F);
                Debug.Assert(pack.roll_ailerons == (float) -9.932539E37F);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
                Debug.Assert(pack.aux4 == (float)2.896118E38F);
                Debug.Assert(pack.pitch_elevator == (float)1.6901776E38F);
                Debug.Assert(pack.time_usec == (ulong)2978602580712064395L);
                Debug.Assert(pack.aux2 == (float) -1.8602797E38F);
                Debug.Assert(pack.nav_mode == (byte)(byte)86);
                Debug.Assert(pack.aux1 == (float) -4.4351885E37F);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.aux3 = (float) -2.898332E38F;
            p91.mode = MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            p91.roll_ailerons = (float) -9.932539E37F;
            p91.pitch_elevator = (float)1.6901776E38F;
            p91.throttle = (float)2.4543861E38F;
            p91.time_usec = (ulong)2978602580712064395L;
            p91.aux4 = (float)2.896118E38F;
            p91.aux1 = (float) -4.4351885E37F;
            p91.aux2 = (float) -1.8602797E38F;
            p91.yaw_rudder = (float) -1.625526E38F;
            p91.nav_mode = (byte)(byte)86;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)44904);
                Debug.Assert(pack.time_usec == (ulong)5561918530974073170L);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)3919);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)25618);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)1279);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)20846);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)32664);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)12555);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)22446);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)45982);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)36251);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)45230);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)17244);
                Debug.Assert(pack.rssi == (byte)(byte)226);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.time_usec = (ulong)5561918530974073170L;
            p92.chan6_raw = (ushort)(ushort)45982;
            p92.chan8_raw = (ushort)(ushort)20846;
            p92.chan10_raw = (ushort)(ushort)25618;
            p92.chan3_raw = (ushort)(ushort)3919;
            p92.chan12_raw = (ushort)(ushort)36251;
            p92.chan11_raw = (ushort)(ushort)44904;
            p92.chan7_raw = (ushort)(ushort)22446;
            p92.chan4_raw = (ushort)(ushort)17244;
            p92.chan2_raw = (ushort)(ushort)1279;
            p92.chan9_raw = (ushort)(ushort)45230;
            p92.rssi = (byte)(byte)226;
            p92.chan1_raw = (ushort)(ushort)32664;
            p92.chan5_raw = (ushort)(ushort)12555;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (ulong)8236996432700704832L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-2.4222199E38F, -2.1047943E38F, 2.5353507E38F, 2.547887E38F, -3.2333155E38F, 1.465788E38F, 2.2468352E38F, -5.780306E37F, -9.529232E37F, -2.2905828E38F, 1.7202757E38F, 1.2038215E38F, 2.909372E38F, -1.8485644E38F, -4.4782207E37F, -3.125047E38F}));
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_GUIDED_DISARMED);
                Debug.Assert(pack.time_usec == (ulong)139392827634537559L);
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.mode = MAV_MODE.MAV_MODE_GUIDED_DISARMED;
            p93.flags = (ulong)8236996432700704832L;
            p93.controls_SET(new float[] {-2.4222199E38F, -2.1047943E38F, 2.5353507E38F, 2.547887E38F, -3.2333155E38F, 1.465788E38F, 2.2468352E38F, -5.780306E37F, -9.529232E37F, -2.2905828E38F, 1.7202757E38F, 1.2038215E38F, 2.909372E38F, -1.8485644E38F, -4.4782207E37F, -3.125047E38F}, 0) ;
            p93.time_usec = (ulong)139392827634537559L;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_y == (short)(short)26057);
                Debug.Assert(pack.flow_comp_m_x == (float) -2.351788E38F);
                Debug.Assert(pack.quality == (byte)(byte)93);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)3.0992927E38F);
                Debug.Assert(pack.ground_distance == (float)1.8373709E37F);
                Debug.Assert(pack.flow_comp_m_y == (float)2.5975323E38F);
                Debug.Assert(pack.time_usec == (ulong)4100510206309614543L);
                Debug.Assert(pack.flow_x == (short)(short)23352);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float)2.1238348E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)212);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.ground_distance = (float)1.8373709E37F;
            p100.flow_y = (short)(short)26057;
            p100.flow_comp_m_y = (float)2.5975323E38F;
            p100.flow_rate_y_SET((float)2.1238348E38F, PH) ;
            p100.flow_rate_x_SET((float)3.0992927E38F, PH) ;
            p100.flow_x = (short)(short)23352;
            p100.sensor_id = (byte)(byte)212;
            p100.time_usec = (ulong)4100510206309614543L;
            p100.flow_comp_m_x = (float) -2.351788E38F;
            p100.quality = (byte)(byte)93;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -1.2530288E38F);
                Debug.Assert(pack.roll == (float) -9.17418E37F);
                Debug.Assert(pack.z == (float)1.3000071E38F);
                Debug.Assert(pack.yaw == (float)1.6926517E38F);
                Debug.Assert(pack.y == (float)4.6799835E37F);
                Debug.Assert(pack.x == (float) -2.8076654E38F);
                Debug.Assert(pack.usec == (ulong)1396126629842634585L);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.x = (float) -2.8076654E38F;
            p101.roll = (float) -9.17418E37F;
            p101.z = (float)1.3000071E38F;
            p101.usec = (ulong)1396126629842634585L;
            p101.y = (float)4.6799835E37F;
            p101.pitch = (float) -1.2530288E38F;
            p101.yaw = (float)1.6926517E38F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)3.691122E37F);
                Debug.Assert(pack.yaw == (float) -2.6073042E38F);
                Debug.Assert(pack.roll == (float)2.6443017E38F);
                Debug.Assert(pack.z == (float) -3.693432E37F);
                Debug.Assert(pack.pitch == (float) -4.5100554E37F);
                Debug.Assert(pack.usec == (ulong)8949882080309136572L);
                Debug.Assert(pack.x == (float) -2.7298178E38F);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.pitch = (float) -4.5100554E37F;
            p102.usec = (ulong)8949882080309136572L;
            p102.roll = (float)2.6443017E38F;
            p102.y = (float)3.691122E37F;
            p102.x = (float) -2.7298178E38F;
            p102.z = (float) -3.693432E37F;
            p102.yaw = (float) -2.6073042E38F;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)7548086254355361679L);
                Debug.Assert(pack.x == (float) -1.852013E38F);
                Debug.Assert(pack.y == (float) -1.2220278E38F);
                Debug.Assert(pack.z == (float) -3.033982E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.z = (float) -3.033982E38F;
            p103.y = (float) -1.2220278E38F;
            p103.x = (float) -1.852013E38F;
            p103.usec = (ulong)7548086254355361679L;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -2.0974717E38F);
                Debug.Assert(pack.pitch == (float)1.2843765E38F);
                Debug.Assert(pack.roll == (float)2.208878E37F);
                Debug.Assert(pack.x == (float)9.3593255E36F);
                Debug.Assert(pack.z == (float)1.2487502E38F);
                Debug.Assert(pack.yaw == (float) -3.2378987E38F);
                Debug.Assert(pack.usec == (ulong)2788260370819011415L);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.z = (float)1.2487502E38F;
            p104.y = (float) -2.0974717E38F;
            p104.pitch = (float)1.2843765E38F;
            p104.usec = (ulong)2788260370819011415L;
            p104.roll = (float)2.208878E37F;
            p104.yaw = (float) -3.2378987E38F;
            p104.x = (float)9.3593255E36F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fields_updated == (ushort)(ushort)55427);
                Debug.Assert(pack.zgyro == (float)2.7058844E38F);
                Debug.Assert(pack.temperature == (float)8.454703E37F);
                Debug.Assert(pack.ygyro == (float) -2.3641274E38F);
                Debug.Assert(pack.pressure_alt == (float)1.2565313E38F);
                Debug.Assert(pack.xgyro == (float) -3.1115644E38F);
                Debug.Assert(pack.diff_pressure == (float) -1.2779558E38F);
                Debug.Assert(pack.ymag == (float)2.4340153E38F);
                Debug.Assert(pack.zmag == (float)1.2410611E38F);
                Debug.Assert(pack.time_usec == (ulong)3065980185825411675L);
                Debug.Assert(pack.yacc == (float) -1.4202843E38F);
                Debug.Assert(pack.xacc == (float)1.6015723E38F);
                Debug.Assert(pack.abs_pressure == (float)1.823769E38F);
                Debug.Assert(pack.zacc == (float)1.1298811E38F);
                Debug.Assert(pack.xmag == (float) -9.360563E37F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.time_usec = (ulong)3065980185825411675L;
            p105.zgyro = (float)2.7058844E38F;
            p105.fields_updated = (ushort)(ushort)55427;
            p105.xmag = (float) -9.360563E37F;
            p105.yacc = (float) -1.4202843E38F;
            p105.abs_pressure = (float)1.823769E38F;
            p105.ygyro = (float) -2.3641274E38F;
            p105.ymag = (float)2.4340153E38F;
            p105.zacc = (float)1.1298811E38F;
            p105.pressure_alt = (float)1.2565313E38F;
            p105.temperature = (float)8.454703E37F;
            p105.zmag = (float)1.2410611E38F;
            p105.diff_pressure = (float) -1.2779558E38F;
            p105.xacc = (float)1.6015723E38F;
            p105.xgyro = (float) -3.1115644E38F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_delta_distance_us == (uint)57347358U);
                Debug.Assert(pack.quality == (byte)(byte)32);
                Debug.Assert(pack.integrated_zgyro == (float) -4.1109536E37F);
                Debug.Assert(pack.integrated_ygyro == (float)2.8693245E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)13);
                Debug.Assert(pack.integrated_x == (float)1.1197644E38F);
                Debug.Assert(pack.temperature == (short)(short) -30920);
                Debug.Assert(pack.integration_time_us == (uint)22654563U);
                Debug.Assert(pack.integrated_y == (float)3.1099879E38F);
                Debug.Assert(pack.distance == (float) -3.2761994E37F);
                Debug.Assert(pack.time_usec == (ulong)5449484122900314704L);
                Debug.Assert(pack.integrated_xgyro == (float)2.0084161E38F);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.integration_time_us = (uint)22654563U;
            p106.distance = (float) -3.2761994E37F;
            p106.integrated_xgyro = (float)2.0084161E38F;
            p106.integrated_ygyro = (float)2.8693245E38F;
            p106.integrated_y = (float)3.1099879E38F;
            p106.integrated_x = (float)1.1197644E38F;
            p106.integrated_zgyro = (float) -4.1109536E37F;
            p106.temperature = (short)(short) -30920;
            p106.quality = (byte)(byte)32;
            p106.time_delta_distance_us = (uint)57347358U;
            p106.sensor_id = (byte)(byte)13;
            p106.time_usec = (ulong)5449484122900314704L;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)1247057713123850642L);
                Debug.Assert(pack.zgyro == (float)2.3370623E38F);
                Debug.Assert(pack.xmag == (float)3.739101E37F);
                Debug.Assert(pack.xgyro == (float)2.322354E38F);
                Debug.Assert(pack.ygyro == (float) -7.4876014E37F);
                Debug.Assert(pack.zacc == (float)2.025501E38F);
                Debug.Assert(pack.abs_pressure == (float) -5.4742796E37F);
                Debug.Assert(pack.xacc == (float)1.8542177E38F);
                Debug.Assert(pack.diff_pressure == (float)1.1153257E38F);
                Debug.Assert(pack.pressure_alt == (float) -2.1211628E38F);
                Debug.Assert(pack.temperature == (float)1.1783811E38F);
                Debug.Assert(pack.ymag == (float) -1.4630448E38F);
                Debug.Assert(pack.yacc == (float) -2.4251349E38F);
                Debug.Assert(pack.fields_updated == (uint)3123738604U);
                Debug.Assert(pack.zmag == (float) -2.5558124E38F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.xacc = (float)1.8542177E38F;
            p107.ymag = (float) -1.4630448E38F;
            p107.xmag = (float)3.739101E37F;
            p107.zmag = (float) -2.5558124E38F;
            p107.zgyro = (float)2.3370623E38F;
            p107.xgyro = (float)2.322354E38F;
            p107.zacc = (float)2.025501E38F;
            p107.time_usec = (ulong)1247057713123850642L;
            p107.ygyro = (float) -7.4876014E37F;
            p107.yacc = (float) -2.4251349E38F;
            p107.diff_pressure = (float)1.1153257E38F;
            p107.abs_pressure = (float) -5.4742796E37F;
            p107.fields_updated = (uint)3123738604U;
            p107.temperature = (float)1.1783811E38F;
            p107.pressure_alt = (float) -2.1211628E38F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q4 == (float)2.0937101E38F);
                Debug.Assert(pack.xgyro == (float)5.0113387E37F);
                Debug.Assert(pack.q3 == (float)2.0977824E38F);
                Debug.Assert(pack.pitch == (float) -1.5339594E38F);
                Debug.Assert(pack.ve == (float) -3.0348962E37F);
                Debug.Assert(pack.zgyro == (float)2.3236768E38F);
                Debug.Assert(pack.yaw == (float)1.6976796E38F);
                Debug.Assert(pack.ygyro == (float)3.1992923E38F);
                Debug.Assert(pack.xacc == (float) -2.6150524E38F);
                Debug.Assert(pack.roll == (float) -9.180947E37F);
                Debug.Assert(pack.q1 == (float)1.8626357E38F);
                Debug.Assert(pack.q2 == (float)3.023835E38F);
                Debug.Assert(pack.vd == (float)2.6356008E37F);
                Debug.Assert(pack.zacc == (float) -3.100947E38F);
                Debug.Assert(pack.vn == (float)2.049852E38F);
                Debug.Assert(pack.yacc == (float)1.8444428E38F);
                Debug.Assert(pack.lon == (float)1.5788318E38F);
                Debug.Assert(pack.std_dev_vert == (float) -2.5702997E38F);
                Debug.Assert(pack.alt == (float) -1.0246824E38F);
                Debug.Assert(pack.lat == (float) -6.2182957E37F);
                Debug.Assert(pack.std_dev_horz == (float) -2.07921E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.xacc = (float) -2.6150524E38F;
            p108.roll = (float) -9.180947E37F;
            p108.ygyro = (float)3.1992923E38F;
            p108.zacc = (float) -3.100947E38F;
            p108.std_dev_horz = (float) -2.07921E38F;
            p108.q1 = (float)1.8626357E38F;
            p108.pitch = (float) -1.5339594E38F;
            p108.yaw = (float)1.6976796E38F;
            p108.q2 = (float)3.023835E38F;
            p108.q3 = (float)2.0977824E38F;
            p108.lon = (float)1.5788318E38F;
            p108.vd = (float)2.6356008E37F;
            p108.ve = (float) -3.0348962E37F;
            p108.std_dev_vert = (float) -2.5702997E38F;
            p108.zgyro = (float)2.3236768E38F;
            p108.lat = (float) -6.2182957E37F;
            p108.vn = (float)2.049852E38F;
            p108.q4 = (float)2.0937101E38F;
            p108.alt = (float) -1.0246824E38F;
            p108.yacc = (float)1.8444428E38F;
            p108.xgyro = (float)5.0113387E37F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rxerrors == (ushort)(ushort)33931);
                Debug.Assert(pack.remnoise == (byte)(byte)133);
                Debug.Assert(pack.remrssi == (byte)(byte)221);
                Debug.Assert(pack.rssi == (byte)(byte)83);
                Debug.Assert(pack.txbuf == (byte)(byte)128);
                Debug.Assert(pack.noise == (byte)(byte)185);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)20899);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.remrssi = (byte)(byte)221;
            p109.rssi = (byte)(byte)83;
            p109.rxerrors = (ushort)(ushort)33931;
            p109.txbuf = (byte)(byte)128;
            p109.fixed_ = (ushort)(ushort)20899;
            p109.noise = (byte)(byte)185;
            p109.remnoise = (byte)(byte)133;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)58);
                Debug.Assert(pack.target_network == (byte)(byte)116);
                Debug.Assert(pack.target_component == (byte)(byte)74);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)8, (byte)33, (byte)63, (byte)72, (byte)83, (byte)200, (byte)248, (byte)196, (byte)81, (byte)78, (byte)153, (byte)165, (byte)34, (byte)246, (byte)223, (byte)236, (byte)89, (byte)11, (byte)107, (byte)136, (byte)140, (byte)132, (byte)205, (byte)204, (byte)2, (byte)5, (byte)211, (byte)227, (byte)24, (byte)135, (byte)227, (byte)131, (byte)70, (byte)180, (byte)113, (byte)199, (byte)143, (byte)37, (byte)26, (byte)126, (byte)181, (byte)255, (byte)49, (byte)180, (byte)100, (byte)228, (byte)235, (byte)151, (byte)121, (byte)208, (byte)81, (byte)142, (byte)137, (byte)103, (byte)136, (byte)79, (byte)22, (byte)31, (byte)140, (byte)249, (byte)25, (byte)68, (byte)173, (byte)100, (byte)117, (byte)157, (byte)166, (byte)91, (byte)180, (byte)157, (byte)177, (byte)126, (byte)217, (byte)95, (byte)109, (byte)71, (byte)4, (byte)238, (byte)234, (byte)94, (byte)73, (byte)222, (byte)200, (byte)228, (byte)186, (byte)181, (byte)96, (byte)65, (byte)28, (byte)172, (byte)215, (byte)32, (byte)76, (byte)177, (byte)64, (byte)203, (byte)13, (byte)249, (byte)245, (byte)134, (byte)157, (byte)84, (byte)150, (byte)187, (byte)192, (byte)140, (byte)200, (byte)122, (byte)39, (byte)71, (byte)246, (byte)121, (byte)134, (byte)112, (byte)129, (byte)67, (byte)106, (byte)25, (byte)53, (byte)187, (byte)242, (byte)130, (byte)65, (byte)37, (byte)41, (byte)108, (byte)1, (byte)118, (byte)4, (byte)16, (byte)71, (byte)6, (byte)23, (byte)237, (byte)215, (byte)172, (byte)92, (byte)12, (byte)101, (byte)204, (byte)27, (byte)33, (byte)171, (byte)136, (byte)58, (byte)110, (byte)40, (byte)12, (byte)168, (byte)213, (byte)60, (byte)12, (byte)60, (byte)150, (byte)10, (byte)46, (byte)151, (byte)169, (byte)110, (byte)200, (byte)190, (byte)94, (byte)253, (byte)146, (byte)76, (byte)2, (byte)44, (byte)98, (byte)14, (byte)22, (byte)56, (byte)241, (byte)241, (byte)213, (byte)205, (byte)99, (byte)90, (byte)157, (byte)113, (byte)43, (byte)252, (byte)164, (byte)42, (byte)22, (byte)143, (byte)10, (byte)104, (byte)44, (byte)184, (byte)149, (byte)89, (byte)172, (byte)85, (byte)132, (byte)106, (byte)151, (byte)79, (byte)115, (byte)113, (byte)9, (byte)230, (byte)167, (byte)78, (byte)209, (byte)213, (byte)142, (byte)215, (byte)218, (byte)95, (byte)93, (byte)179, (byte)182, (byte)22, (byte)89, (byte)234, (byte)141, (byte)252, (byte)34, (byte)195, (byte)62, (byte)8, (byte)236, (byte)88, (byte)111, (byte)114, (byte)248, (byte)230, (byte)135, (byte)146, (byte)149, (byte)52, (byte)37, (byte)96, (byte)18, (byte)10, (byte)0, (byte)123, (byte)189, (byte)138, (byte)167, (byte)13, (byte)36, (byte)147, (byte)186, (byte)240, (byte)168, (byte)133, (byte)176, (byte)254, (byte)249, (byte)59}));
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_component = (byte)(byte)74;
            p110.target_network = (byte)(byte)116;
            p110.target_system = (byte)(byte)58;
            p110.payload_SET(new byte[] {(byte)8, (byte)33, (byte)63, (byte)72, (byte)83, (byte)200, (byte)248, (byte)196, (byte)81, (byte)78, (byte)153, (byte)165, (byte)34, (byte)246, (byte)223, (byte)236, (byte)89, (byte)11, (byte)107, (byte)136, (byte)140, (byte)132, (byte)205, (byte)204, (byte)2, (byte)5, (byte)211, (byte)227, (byte)24, (byte)135, (byte)227, (byte)131, (byte)70, (byte)180, (byte)113, (byte)199, (byte)143, (byte)37, (byte)26, (byte)126, (byte)181, (byte)255, (byte)49, (byte)180, (byte)100, (byte)228, (byte)235, (byte)151, (byte)121, (byte)208, (byte)81, (byte)142, (byte)137, (byte)103, (byte)136, (byte)79, (byte)22, (byte)31, (byte)140, (byte)249, (byte)25, (byte)68, (byte)173, (byte)100, (byte)117, (byte)157, (byte)166, (byte)91, (byte)180, (byte)157, (byte)177, (byte)126, (byte)217, (byte)95, (byte)109, (byte)71, (byte)4, (byte)238, (byte)234, (byte)94, (byte)73, (byte)222, (byte)200, (byte)228, (byte)186, (byte)181, (byte)96, (byte)65, (byte)28, (byte)172, (byte)215, (byte)32, (byte)76, (byte)177, (byte)64, (byte)203, (byte)13, (byte)249, (byte)245, (byte)134, (byte)157, (byte)84, (byte)150, (byte)187, (byte)192, (byte)140, (byte)200, (byte)122, (byte)39, (byte)71, (byte)246, (byte)121, (byte)134, (byte)112, (byte)129, (byte)67, (byte)106, (byte)25, (byte)53, (byte)187, (byte)242, (byte)130, (byte)65, (byte)37, (byte)41, (byte)108, (byte)1, (byte)118, (byte)4, (byte)16, (byte)71, (byte)6, (byte)23, (byte)237, (byte)215, (byte)172, (byte)92, (byte)12, (byte)101, (byte)204, (byte)27, (byte)33, (byte)171, (byte)136, (byte)58, (byte)110, (byte)40, (byte)12, (byte)168, (byte)213, (byte)60, (byte)12, (byte)60, (byte)150, (byte)10, (byte)46, (byte)151, (byte)169, (byte)110, (byte)200, (byte)190, (byte)94, (byte)253, (byte)146, (byte)76, (byte)2, (byte)44, (byte)98, (byte)14, (byte)22, (byte)56, (byte)241, (byte)241, (byte)213, (byte)205, (byte)99, (byte)90, (byte)157, (byte)113, (byte)43, (byte)252, (byte)164, (byte)42, (byte)22, (byte)143, (byte)10, (byte)104, (byte)44, (byte)184, (byte)149, (byte)89, (byte)172, (byte)85, (byte)132, (byte)106, (byte)151, (byte)79, (byte)115, (byte)113, (byte)9, (byte)230, (byte)167, (byte)78, (byte)209, (byte)213, (byte)142, (byte)215, (byte)218, (byte)95, (byte)93, (byte)179, (byte)182, (byte)22, (byte)89, (byte)234, (byte)141, (byte)252, (byte)34, (byte)195, (byte)62, (byte)8, (byte)236, (byte)88, (byte)111, (byte)114, (byte)248, (byte)230, (byte)135, (byte)146, (byte)149, (byte)52, (byte)37, (byte)96, (byte)18, (byte)10, (byte)0, (byte)123, (byte)189, (byte)138, (byte)167, (byte)13, (byte)36, (byte)147, (byte)186, (byte)240, (byte)168, (byte)133, (byte)176, (byte)254, (byte)249, (byte)59}, 0) ;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tc1 == (long) -8633280938329428615L);
                Debug.Assert(pack.ts1 == (long)3095960908192671086L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -8633280938329428615L;
            p111.ts1 = (long)3095960908192671086L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (uint)1282483711U);
                Debug.Assert(pack.time_usec == (ulong)4028400896172588728L);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)4028400896172588728L;
            p112.seq = (uint)1282483711U;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vn == (short)(short) -6037);
                Debug.Assert(pack.lon == (int)1783951673);
                Debug.Assert(pack.vd == (short)(short) -9830);
                Debug.Assert(pack.lat == (int) -2054641292);
                Debug.Assert(pack.cog == (ushort)(ushort)14382);
                Debug.Assert(pack.eph == (ushort)(ushort)59980);
                Debug.Assert(pack.alt == (int)1788387149);
                Debug.Assert(pack.vel == (ushort)(ushort)59584);
                Debug.Assert(pack.satellites_visible == (byte)(byte)133);
                Debug.Assert(pack.fix_type == (byte)(byte)133);
                Debug.Assert(pack.time_usec == (ulong)3029603050710066850L);
                Debug.Assert(pack.epv == (ushort)(ushort)63702);
                Debug.Assert(pack.ve == (short)(short)25768);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.epv = (ushort)(ushort)63702;
            p113.vn = (short)(short) -6037;
            p113.lat = (int) -2054641292;
            p113.time_usec = (ulong)3029603050710066850L;
            p113.satellites_visible = (byte)(byte)133;
            p113.cog = (ushort)(ushort)14382;
            p113.vel = (ushort)(ushort)59584;
            p113.lon = (int)1783951673;
            p113.ve = (short)(short)25768;
            p113.vd = (short)(short) -9830;
            p113.alt = (int)1788387149;
            p113.fix_type = (byte)(byte)133;
            p113.eph = (ushort)(ushort)59980;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_xgyro == (float) -2.2919958E38F);
                Debug.Assert(pack.integrated_ygyro == (float) -3.320607E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)3587278688U);
                Debug.Assert(pack.integrated_x == (float)6.963467E37F);
                Debug.Assert(pack.temperature == (short)(short) -13138);
                Debug.Assert(pack.distance == (float) -1.896042E38F);
                Debug.Assert(pack.quality == (byte)(byte)8);
                Debug.Assert(pack.sensor_id == (byte)(byte)87);
                Debug.Assert(pack.integrated_zgyro == (float)1.0449329E38F);
                Debug.Assert(pack.integration_time_us == (uint)2052055053U);
                Debug.Assert(pack.time_usec == (ulong)5077291133891214194L);
                Debug.Assert(pack.integrated_y == (float) -6.4763377E37F);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.distance = (float) -1.896042E38F;
            p114.integrated_ygyro = (float) -3.320607E38F;
            p114.time_usec = (ulong)5077291133891214194L;
            p114.integrated_x = (float)6.963467E37F;
            p114.integrated_y = (float) -6.4763377E37F;
            p114.time_delta_distance_us = (uint)3587278688U;
            p114.integration_time_us = (uint)2052055053U;
            p114.quality = (byte)(byte)8;
            p114.sensor_id = (byte)(byte)87;
            p114.temperature = (short)(short) -13138;
            p114.integrated_xgyro = (float) -2.2919958E38F;
            p114.integrated_zgyro = (float)1.0449329E38F;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yawspeed == (float)2.9393917E38F);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {2.364656E38F, 2.3429327E38F, -3.309593E38F, -9.012743E37F}));
                Debug.Assert(pack.time_usec == (ulong)4002646461240467008L);
                Debug.Assert(pack.vy == (short)(short)6343);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)61362);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)36334);
                Debug.Assert(pack.xacc == (short)(short)20781);
                Debug.Assert(pack.rollspeed == (float) -2.5681782E38F);
                Debug.Assert(pack.lon == (int) -1962485116);
                Debug.Assert(pack.zacc == (short)(short) -12906);
                Debug.Assert(pack.alt == (int) -124025312);
                Debug.Assert(pack.vz == (short)(short) -23526);
                Debug.Assert(pack.pitchspeed == (float)3.1191517E37F);
                Debug.Assert(pack.yacc == (short)(short) -2705);
                Debug.Assert(pack.vx == (short)(short) -29923);
                Debug.Assert(pack.lat == (int)1039870321);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.time_usec = (ulong)4002646461240467008L;
            p115.lat = (int)1039870321;
            p115.vx = (short)(short) -29923;
            p115.yawspeed = (float)2.9393917E38F;
            p115.yacc = (short)(short) -2705;
            p115.rollspeed = (float) -2.5681782E38F;
            p115.attitude_quaternion_SET(new float[] {2.364656E38F, 2.3429327E38F, -3.309593E38F, -9.012743E37F}, 0) ;
            p115.alt = (int) -124025312;
            p115.pitchspeed = (float)3.1191517E37F;
            p115.zacc = (short)(short) -12906;
            p115.ind_airspeed = (ushort)(ushort)36334;
            p115.vy = (short)(short)6343;
            p115.vz = (short)(short) -23526;
            p115.lon = (int) -1962485116;
            p115.xacc = (short)(short)20781;
            p115.true_airspeed = (ushort)(ushort)61362;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1670447811U);
                Debug.Assert(pack.xgyro == (short)(short) -26740);
                Debug.Assert(pack.xmag == (short)(short)2080);
                Debug.Assert(pack.ymag == (short)(short)11478);
                Debug.Assert(pack.zmag == (short)(short) -5469);
                Debug.Assert(pack.yacc == (short)(short) -3542);
                Debug.Assert(pack.ygyro == (short)(short) -14420);
                Debug.Assert(pack.xacc == (short)(short)26264);
                Debug.Assert(pack.zgyro == (short)(short) -2407);
                Debug.Assert(pack.zacc == (short)(short)2353);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.yacc = (short)(short) -3542;
            p116.xmag = (short)(short)2080;
            p116.zmag = (short)(short) -5469;
            p116.zacc = (short)(short)2353;
            p116.ymag = (short)(short)11478;
            p116.zgyro = (short)(short) -2407;
            p116.xacc = (short)(short)26264;
            p116.time_boot_ms = (uint)1670447811U;
            p116.xgyro = (short)(short) -26740;
            p116.ygyro = (short)(short) -14420;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)176);
                Debug.Assert(pack.end == (ushort)(ushort)1870);
                Debug.Assert(pack.start == (ushort)(ushort)64056);
                Debug.Assert(pack.target_component == (byte)(byte)228);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.end = (ushort)(ushort)1870;
            p117.start = (ushort)(ushort)64056;
            p117.target_component = (byte)(byte)228;
            p117.target_system = (byte)(byte)176;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.num_logs == (ushort)(ushort)60009);
                Debug.Assert(pack.time_utc == (uint)1924659735U);
                Debug.Assert(pack.id == (ushort)(ushort)24111);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)28854);
                Debug.Assert(pack.size == (uint)2485849635U);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.time_utc = (uint)1924659735U;
            p118.last_log_num = (ushort)(ushort)28854;
            p118.size = (uint)2485849635U;
            p118.id = (ushort)(ushort)24111;
            p118.num_logs = (ushort)(ushort)60009;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)168);
                Debug.Assert(pack.id == (ushort)(ushort)32936);
                Debug.Assert(pack.target_component == (byte)(byte)139);
                Debug.Assert(pack.ofs == (uint)406106255U);
                Debug.Assert(pack.count == (uint)3385262486U);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.ofs = (uint)406106255U;
            p119.id = (ushort)(ushort)32936;
            p119.target_system = (byte)(byte)168;
            p119.count = (uint)3385262486U;
            p119.target_component = (byte)(byte)139;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)3941297212U);
                Debug.Assert(pack.count == (byte)(byte)200);
                Debug.Assert(pack.id == (ushort)(ushort)63524);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)213, (byte)87, (byte)164, (byte)23, (byte)207, (byte)43, (byte)191, (byte)43, (byte)106, (byte)113, (byte)217, (byte)162, (byte)170, (byte)166, (byte)120, (byte)205, (byte)42, (byte)120, (byte)36, (byte)126, (byte)231, (byte)158, (byte)51, (byte)163, (byte)26, (byte)69, (byte)98, (byte)193, (byte)18, (byte)179, (byte)244, (byte)217, (byte)171, (byte)38, (byte)109, (byte)123, (byte)210, (byte)176, (byte)101, (byte)148, (byte)217, (byte)58, (byte)253, (byte)98, (byte)209, (byte)161, (byte)102, (byte)245, (byte)57, (byte)117, (byte)189, (byte)1, (byte)14, (byte)26, (byte)45, (byte)185, (byte)211, (byte)155, (byte)75, (byte)151, (byte)32, (byte)107, (byte)7, (byte)34, (byte)203, (byte)191, (byte)140, (byte)110, (byte)199, (byte)175, (byte)119, (byte)187, (byte)58, (byte)185, (byte)253, (byte)163, (byte)87, (byte)250, (byte)53, (byte)125, (byte)71, (byte)250, (byte)193, (byte)8, (byte)147, (byte)91, (byte)99, (byte)153, (byte)226, (byte)5}));
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.count = (byte)(byte)200;
            p120.id = (ushort)(ushort)63524;
            p120.data__SET(new byte[] {(byte)213, (byte)87, (byte)164, (byte)23, (byte)207, (byte)43, (byte)191, (byte)43, (byte)106, (byte)113, (byte)217, (byte)162, (byte)170, (byte)166, (byte)120, (byte)205, (byte)42, (byte)120, (byte)36, (byte)126, (byte)231, (byte)158, (byte)51, (byte)163, (byte)26, (byte)69, (byte)98, (byte)193, (byte)18, (byte)179, (byte)244, (byte)217, (byte)171, (byte)38, (byte)109, (byte)123, (byte)210, (byte)176, (byte)101, (byte)148, (byte)217, (byte)58, (byte)253, (byte)98, (byte)209, (byte)161, (byte)102, (byte)245, (byte)57, (byte)117, (byte)189, (byte)1, (byte)14, (byte)26, (byte)45, (byte)185, (byte)211, (byte)155, (byte)75, (byte)151, (byte)32, (byte)107, (byte)7, (byte)34, (byte)203, (byte)191, (byte)140, (byte)110, (byte)199, (byte)175, (byte)119, (byte)187, (byte)58, (byte)185, (byte)253, (byte)163, (byte)87, (byte)250, (byte)53, (byte)125, (byte)71, (byte)250, (byte)193, (byte)8, (byte)147, (byte)91, (byte)99, (byte)153, (byte)226, (byte)5}, 0) ;
            p120.ofs = (uint)3941297212U;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)219);
                Debug.Assert(pack.target_system == (byte)(byte)72);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_component = (byte)(byte)219;
            p121.target_system = (byte)(byte)72;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)24);
                Debug.Assert(pack.target_system == (byte)(byte)196);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_component = (byte)(byte)24;
            p122.target_system = (byte)(byte)196;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)98);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)125, (byte)187, (byte)39, (byte)135, (byte)142, (byte)226, (byte)151, (byte)247, (byte)36, (byte)202, (byte)196, (byte)130, (byte)154, (byte)25, (byte)117, (byte)221, (byte)225, (byte)65, (byte)120, (byte)169, (byte)116, (byte)99, (byte)188, (byte)44, (byte)158, (byte)224, (byte)17, (byte)6, (byte)172, (byte)17, (byte)4, (byte)112, (byte)121, (byte)150, (byte)32, (byte)247, (byte)128, (byte)220, (byte)9, (byte)68, (byte)145, (byte)219, (byte)84, (byte)136, (byte)156, (byte)224, (byte)37, (byte)83, (byte)5, (byte)190, (byte)197, (byte)221, (byte)65, (byte)165, (byte)92, (byte)219, (byte)16, (byte)143, (byte)34, (byte)196, (byte)96, (byte)64, (byte)162, (byte)255, (byte)247, (byte)160, (byte)196, (byte)246, (byte)120, (byte)17, (byte)148, (byte)137, (byte)87, (byte)205, (byte)152, (byte)132, (byte)95, (byte)178, (byte)42, (byte)106, (byte)215, (byte)188, (byte)114, (byte)110, (byte)81, (byte)244, (byte)57, (byte)178, (byte)178, (byte)18, (byte)170, (byte)240, (byte)229, (byte)153, (byte)248, (byte)204, (byte)80, (byte)241, (byte)47, (byte)178, (byte)225, (byte)176, (byte)121, (byte)230, (byte)94, (byte)178, (byte)52, (byte)91, (byte)38, (byte)151}));
                Debug.Assert(pack.target_system == (byte)(byte)95);
                Debug.Assert(pack.len == (byte)(byte)26);
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.target_system = (byte)(byte)95;
            p123.data__SET(new byte[] {(byte)125, (byte)187, (byte)39, (byte)135, (byte)142, (byte)226, (byte)151, (byte)247, (byte)36, (byte)202, (byte)196, (byte)130, (byte)154, (byte)25, (byte)117, (byte)221, (byte)225, (byte)65, (byte)120, (byte)169, (byte)116, (byte)99, (byte)188, (byte)44, (byte)158, (byte)224, (byte)17, (byte)6, (byte)172, (byte)17, (byte)4, (byte)112, (byte)121, (byte)150, (byte)32, (byte)247, (byte)128, (byte)220, (byte)9, (byte)68, (byte)145, (byte)219, (byte)84, (byte)136, (byte)156, (byte)224, (byte)37, (byte)83, (byte)5, (byte)190, (byte)197, (byte)221, (byte)65, (byte)165, (byte)92, (byte)219, (byte)16, (byte)143, (byte)34, (byte)196, (byte)96, (byte)64, (byte)162, (byte)255, (byte)247, (byte)160, (byte)196, (byte)246, (byte)120, (byte)17, (byte)148, (byte)137, (byte)87, (byte)205, (byte)152, (byte)132, (byte)95, (byte)178, (byte)42, (byte)106, (byte)215, (byte)188, (byte)114, (byte)110, (byte)81, (byte)244, (byte)57, (byte)178, (byte)178, (byte)18, (byte)170, (byte)240, (byte)229, (byte)153, (byte)248, (byte)204, (byte)80, (byte)241, (byte)47, (byte)178, (byte)225, (byte)176, (byte)121, (byte)230, (byte)94, (byte)178, (byte)52, (byte)91, (byte)38, (byte)151}, 0) ;
            p123.target_component = (byte)(byte)98;
            p123.len = (byte)(byte)26;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)986100219);
                Debug.Assert(pack.dgps_numch == (byte)(byte)215);
                Debug.Assert(pack.time_usec == (ulong)6307001570005609184L);
                Debug.Assert(pack.cog == (ushort)(ushort)5102);
                Debug.Assert(pack.eph == (ushort)(ushort)2228);
                Debug.Assert(pack.alt == (int)1635258283);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
                Debug.Assert(pack.dgps_age == (uint)241249669U);
                Debug.Assert(pack.epv == (ushort)(ushort)57146);
                Debug.Assert(pack.vel == (ushort)(ushort)33804);
                Debug.Assert(pack.satellites_visible == (byte)(byte)125);
                Debug.Assert(pack.lat == (int) -383177121);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.dgps_age = (uint)241249669U;
            p124.cog = (ushort)(ushort)5102;
            p124.lon = (int)986100219;
            p124.alt = (int)1635258283;
            p124.time_usec = (ulong)6307001570005609184L;
            p124.vel = (ushort)(ushort)33804;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_PPP;
            p124.dgps_numch = (byte)(byte)215;
            p124.lat = (int) -383177121;
            p124.eph = (ushort)(ushort)2228;
            p124.epv = (ushort)(ushort)57146;
            p124.satellites_visible = (byte)(byte)125;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vservo == (ushort)(ushort)63657);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED));
                Debug.Assert(pack.Vcc == (ushort)(ushort)23379);
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)23379;
            p125.Vservo = (ushort)(ushort)63657;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED);
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.baudrate == (uint)403497628U);
                Debug.Assert(pack.count == (byte)(byte)235);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)34, (byte)233, (byte)142, (byte)66, (byte)2, (byte)167, (byte)197, (byte)81, (byte)183, (byte)51, (byte)105, (byte)179, (byte)233, (byte)45, (byte)108, (byte)53, (byte)55, (byte)168, (byte)161, (byte)220, (byte)178, (byte)139, (byte)182, (byte)174, (byte)38, (byte)80, (byte)98, (byte)113, (byte)198, (byte)21, (byte)35, (byte)0, (byte)57, (byte)99, (byte)130, (byte)92, (byte)179, (byte)183, (byte)90, (byte)34, (byte)245, (byte)148, (byte)33, (byte)145, (byte)42, (byte)43, (byte)240, (byte)200, (byte)113, (byte)42, (byte)183, (byte)159, (byte)28, (byte)91, (byte)74, (byte)116, (byte)45, (byte)201, (byte)37, (byte)227, (byte)155, (byte)49, (byte)0, (byte)137, (byte)248, (byte)7, (byte)132, (byte)180, (byte)58, (byte)105}));
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                                            SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                            SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING));
                Debug.Assert(pack.timeout == (ushort)(ushort)34507);
                Debug.Assert(pack.device == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.timeout = (ushort)(ushort)34507;
            p126.count = (byte)(byte)235;
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1;
            p126.baudrate = (uint)403497628U;
            p126.data__SET(new byte[] {(byte)34, (byte)233, (byte)142, (byte)66, (byte)2, (byte)167, (byte)197, (byte)81, (byte)183, (byte)51, (byte)105, (byte)179, (byte)233, (byte)45, (byte)108, (byte)53, (byte)55, (byte)168, (byte)161, (byte)220, (byte)178, (byte)139, (byte)182, (byte)174, (byte)38, (byte)80, (byte)98, (byte)113, (byte)198, (byte)21, (byte)35, (byte)0, (byte)57, (byte)99, (byte)130, (byte)92, (byte)179, (byte)183, (byte)90, (byte)34, (byte)245, (byte)148, (byte)33, (byte)145, (byte)42, (byte)43, (byte)240, (byte)200, (byte)113, (byte)42, (byte)183, (byte)159, (byte)28, (byte)91, (byte)74, (byte)116, (byte)45, (byte)201, (byte)37, (byte)227, (byte)155, (byte)49, (byte)0, (byte)137, (byte)248, (byte)7, (byte)132, (byte)180, (byte)58, (byte)105}, 0) ;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING);
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wn == (ushort)(ushort)14057);
                Debug.Assert(pack.iar_num_hypotheses == (int)1558217752);
                Debug.Assert(pack.baseline_b_mm == (int) -410141353);
                Debug.Assert(pack.accuracy == (uint)2261632374U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)59);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)155);
                Debug.Assert(pack.tow == (uint)1555863887U);
                Debug.Assert(pack.nsats == (byte)(byte)219);
                Debug.Assert(pack.time_last_baseline_ms == (uint)3485355706U);
                Debug.Assert(pack.rtk_rate == (byte)(byte)124);
                Debug.Assert(pack.rtk_health == (byte)(byte)201);
                Debug.Assert(pack.baseline_c_mm == (int) -820209724);
                Debug.Assert(pack.baseline_a_mm == (int) -1907422778);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.time_last_baseline_ms = (uint)3485355706U;
            p127.rtk_receiver_id = (byte)(byte)59;
            p127.rtk_rate = (byte)(byte)124;
            p127.baseline_b_mm = (int) -410141353;
            p127.iar_num_hypotheses = (int)1558217752;
            p127.accuracy = (uint)2261632374U;
            p127.baseline_a_mm = (int) -1907422778;
            p127.nsats = (byte)(byte)219;
            p127.rtk_health = (byte)(byte)201;
            p127.tow = (uint)1555863887U;
            p127.baseline_coords_type = (byte)(byte)155;
            p127.wn = (ushort)(ushort)14057;
            p127.baseline_c_mm = (int) -820209724;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.nsats == (byte)(byte)60);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)13);
                Debug.Assert(pack.accuracy == (uint)56389812U);
                Debug.Assert(pack.baseline_b_mm == (int)921006489);
                Debug.Assert(pack.tow == (uint)2573683805U);
                Debug.Assert(pack.iar_num_hypotheses == (int) -1678616089);
                Debug.Assert(pack.baseline_a_mm == (int) -508072541);
                Debug.Assert(pack.rtk_health == (byte)(byte)31);
                Debug.Assert(pack.wn == (ushort)(ushort)54902);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)50);
                Debug.Assert(pack.time_last_baseline_ms == (uint)241265131U);
                Debug.Assert(pack.rtk_rate == (byte)(byte)8);
                Debug.Assert(pack.baseline_c_mm == (int) -1067761083);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.accuracy = (uint)56389812U;
            p128.rtk_receiver_id = (byte)(byte)13;
            p128.baseline_b_mm = (int)921006489;
            p128.baseline_coords_type = (byte)(byte)50;
            p128.rtk_rate = (byte)(byte)8;
            p128.baseline_a_mm = (int) -508072541;
            p128.tow = (uint)2573683805U;
            p128.rtk_health = (byte)(byte)31;
            p128.wn = (ushort)(ushort)54902;
            p128.baseline_c_mm = (int) -1067761083;
            p128.nsats = (byte)(byte)60;
            p128.time_last_baseline_ms = (uint)241265131U;
            p128.iar_num_hypotheses = (int) -1678616089;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ymag == (short)(short)15079);
                Debug.Assert(pack.xmag == (short)(short)15794);
                Debug.Assert(pack.yacc == (short)(short) -20140);
                Debug.Assert(pack.zacc == (short)(short) -7861);
                Debug.Assert(pack.time_boot_ms == (uint)2054869150U);
                Debug.Assert(pack.xgyro == (short)(short)11533);
                Debug.Assert(pack.zgyro == (short)(short) -15387);
                Debug.Assert(pack.xacc == (short)(short)11584);
                Debug.Assert(pack.ygyro == (short)(short)11937);
                Debug.Assert(pack.zmag == (short)(short)2654);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.time_boot_ms = (uint)2054869150U;
            p129.ygyro = (short)(short)11937;
            p129.xmag = (short)(short)15794;
            p129.zgyro = (short)(short) -15387;
            p129.zmag = (short)(short)2654;
            p129.xacc = (short)(short)11584;
            p129.xgyro = (short)(short)11533;
            p129.zacc = (short)(short) -7861;
            p129.ymag = (short)(short)15079;
            p129.yacc = (short)(short) -20140;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.size == (uint)2990258565U);
                Debug.Assert(pack.jpg_quality == (byte)(byte)28);
                Debug.Assert(pack.type == (byte)(byte)181);
                Debug.Assert(pack.height == (ushort)(ushort)4076);
                Debug.Assert(pack.payload == (byte)(byte)97);
                Debug.Assert(pack.packets == (ushort)(ushort)54003);
                Debug.Assert(pack.width == (ushort)(ushort)49642);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.type = (byte)(byte)181;
            p130.size = (uint)2990258565U;
            p130.payload = (byte)(byte)97;
            p130.width = (ushort)(ushort)49642;
            p130.jpg_quality = (byte)(byte)28;
            p130.height = (ushort)(ushort)4076;
            p130.packets = (ushort)(ushort)54003;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seqnr == (ushort)(ushort)32618);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)144, (byte)145, (byte)13, (byte)212, (byte)110, (byte)168, (byte)149, (byte)83, (byte)83, (byte)33, (byte)142, (byte)55, (byte)93, (byte)142, (byte)204, (byte)180, (byte)79, (byte)39, (byte)40, (byte)52, (byte)201, (byte)185, (byte)43, (byte)222, (byte)122, (byte)169, (byte)219, (byte)212, (byte)30, (byte)5, (byte)187, (byte)78, (byte)33, (byte)32, (byte)36, (byte)191, (byte)22, (byte)62, (byte)69, (byte)151, (byte)188, (byte)232, (byte)138, (byte)52, (byte)205, (byte)44, (byte)46, (byte)157, (byte)207, (byte)54, (byte)249, (byte)214, (byte)121, (byte)193, (byte)246, (byte)233, (byte)172, (byte)140, (byte)155, (byte)54, (byte)123, (byte)51, (byte)72, (byte)116, (byte)91, (byte)97, (byte)64, (byte)8, (byte)32, (byte)209, (byte)38, (byte)161, (byte)1, (byte)2, (byte)125, (byte)90, (byte)160, (byte)111, (byte)242, (byte)49, (byte)42, (byte)155, (byte)122, (byte)154, (byte)202, (byte)73, (byte)23, (byte)216, (byte)194, (byte)185, (byte)154, (byte)125, (byte)248, (byte)158, (byte)111, (byte)235, (byte)143, (byte)187, (byte)88, (byte)81, (byte)201, (byte)155, (byte)19, (byte)53, (byte)101, (byte)3, (byte)161, (byte)6, (byte)68, (byte)14, (byte)148, (byte)175, (byte)223, (byte)160, (byte)51, (byte)18, (byte)17, (byte)26, (byte)98, (byte)187, (byte)255, (byte)41, (byte)13, (byte)180, (byte)226, (byte)83, (byte)3, (byte)173, (byte)52, (byte)45, (byte)214, (byte)190, (byte)186, (byte)58, (byte)38, (byte)162, (byte)174, (byte)155, (byte)250, (byte)96, (byte)236, (byte)97, (byte)42, (byte)50, (byte)221, (byte)168, (byte)207, (byte)176, (byte)35, (byte)251, (byte)29, (byte)200, (byte)163, (byte)203, (byte)248, (byte)250, (byte)197, (byte)166, (byte)118, (byte)20, (byte)158, (byte)97, (byte)116, (byte)180, (byte)51, (byte)57, (byte)122, (byte)207, (byte)108, (byte)81, (byte)34, (byte)190, (byte)11, (byte)9, (byte)249, (byte)22, (byte)213, (byte)251, (byte)15, (byte)225, (byte)3, (byte)37, (byte)252, (byte)128, (byte)129, (byte)94, (byte)190, (byte)106, (byte)14, (byte)196, (byte)16, (byte)246, (byte)53, (byte)11, (byte)41, (byte)164, (byte)52, (byte)125, (byte)168, (byte)142, (byte)209, (byte)148, (byte)29, (byte)24, (byte)242, (byte)21, (byte)134, (byte)210, (byte)45, (byte)70, (byte)24, (byte)105, (byte)17, (byte)80, (byte)3, (byte)67, (byte)164, (byte)128, (byte)244, (byte)66, (byte)131, (byte)15, (byte)238, (byte)113, (byte)105, (byte)82, (byte)111, (byte)99, (byte)29, (byte)134, (byte)240, (byte)100, (byte)39, (byte)247, (byte)248, (byte)78, (byte)172, (byte)239, (byte)200, (byte)144, (byte)209, (byte)4, (byte)67, (byte)64, (byte)184, (byte)17, (byte)197, (byte)205, (byte)209, (byte)199, (byte)14, (byte)186, (byte)75}));
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)32618;
            p131.data__SET(new byte[] {(byte)144, (byte)145, (byte)13, (byte)212, (byte)110, (byte)168, (byte)149, (byte)83, (byte)83, (byte)33, (byte)142, (byte)55, (byte)93, (byte)142, (byte)204, (byte)180, (byte)79, (byte)39, (byte)40, (byte)52, (byte)201, (byte)185, (byte)43, (byte)222, (byte)122, (byte)169, (byte)219, (byte)212, (byte)30, (byte)5, (byte)187, (byte)78, (byte)33, (byte)32, (byte)36, (byte)191, (byte)22, (byte)62, (byte)69, (byte)151, (byte)188, (byte)232, (byte)138, (byte)52, (byte)205, (byte)44, (byte)46, (byte)157, (byte)207, (byte)54, (byte)249, (byte)214, (byte)121, (byte)193, (byte)246, (byte)233, (byte)172, (byte)140, (byte)155, (byte)54, (byte)123, (byte)51, (byte)72, (byte)116, (byte)91, (byte)97, (byte)64, (byte)8, (byte)32, (byte)209, (byte)38, (byte)161, (byte)1, (byte)2, (byte)125, (byte)90, (byte)160, (byte)111, (byte)242, (byte)49, (byte)42, (byte)155, (byte)122, (byte)154, (byte)202, (byte)73, (byte)23, (byte)216, (byte)194, (byte)185, (byte)154, (byte)125, (byte)248, (byte)158, (byte)111, (byte)235, (byte)143, (byte)187, (byte)88, (byte)81, (byte)201, (byte)155, (byte)19, (byte)53, (byte)101, (byte)3, (byte)161, (byte)6, (byte)68, (byte)14, (byte)148, (byte)175, (byte)223, (byte)160, (byte)51, (byte)18, (byte)17, (byte)26, (byte)98, (byte)187, (byte)255, (byte)41, (byte)13, (byte)180, (byte)226, (byte)83, (byte)3, (byte)173, (byte)52, (byte)45, (byte)214, (byte)190, (byte)186, (byte)58, (byte)38, (byte)162, (byte)174, (byte)155, (byte)250, (byte)96, (byte)236, (byte)97, (byte)42, (byte)50, (byte)221, (byte)168, (byte)207, (byte)176, (byte)35, (byte)251, (byte)29, (byte)200, (byte)163, (byte)203, (byte)248, (byte)250, (byte)197, (byte)166, (byte)118, (byte)20, (byte)158, (byte)97, (byte)116, (byte)180, (byte)51, (byte)57, (byte)122, (byte)207, (byte)108, (byte)81, (byte)34, (byte)190, (byte)11, (byte)9, (byte)249, (byte)22, (byte)213, (byte)251, (byte)15, (byte)225, (byte)3, (byte)37, (byte)252, (byte)128, (byte)129, (byte)94, (byte)190, (byte)106, (byte)14, (byte)196, (byte)16, (byte)246, (byte)53, (byte)11, (byte)41, (byte)164, (byte)52, (byte)125, (byte)168, (byte)142, (byte)209, (byte)148, (byte)29, (byte)24, (byte)242, (byte)21, (byte)134, (byte)210, (byte)45, (byte)70, (byte)24, (byte)105, (byte)17, (byte)80, (byte)3, (byte)67, (byte)164, (byte)128, (byte)244, (byte)66, (byte)131, (byte)15, (byte)238, (byte)113, (byte)105, (byte)82, (byte)111, (byte)99, (byte)29, (byte)134, (byte)240, (byte)100, (byte)39, (byte)247, (byte)248, (byte)78, (byte)172, (byte)239, (byte)200, (byte)144, (byte)209, (byte)4, (byte)67, (byte)64, (byte)184, (byte)17, (byte)197, (byte)205, (byte)209, (byte)199, (byte)14, (byte)186, (byte)75}, 0) ;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
                Debug.Assert(pack.orientation == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_PITCH_180);
                Debug.Assert(pack.max_distance == (ushort)(ushort)56234);
                Debug.Assert(pack.time_boot_ms == (uint)3720104129U);
                Debug.Assert(pack.id == (byte)(byte)75);
                Debug.Assert(pack.covariance == (byte)(byte)17);
                Debug.Assert(pack.current_distance == (ushort)(ushort)3209);
                Debug.Assert(pack.min_distance == (ushort)(ushort)4031);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.current_distance = (ushort)(ushort)3209;
            p132.min_distance = (ushort)(ushort)4031;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_PITCH_180;
            p132.time_boot_ms = (uint)3720104129U;
            p132.id = (byte)(byte)75;
            p132.covariance = (byte)(byte)17;
            p132.max_distance = (ushort)(ushort)56234;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -1378941084);
                Debug.Assert(pack.mask == (ulong)3261532795169407215L);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)44788);
                Debug.Assert(pack.lon == (int) -341410637);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.grid_spacing = (ushort)(ushort)44788;
            p133.lon = (int) -341410637;
            p133.mask = (ulong)3261532795169407215L;
            p133.lat = (int) -1378941084;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short)14603, (short) -16107, (short)22770, (short) -8698, (short) -29779, (short)27172, (short)31962, (short)15923, (short)11383, (short) -8921, (short)17286, (short)22488, (short)3225, (short) -7050, (short)20378, (short)1115}));
                Debug.Assert(pack.gridbit == (byte)(byte)229);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)16075);
                Debug.Assert(pack.lat == (int)519202636);
                Debug.Assert(pack.lon == (int)811932306);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.grid_spacing = (ushort)(ushort)16075;
            p134.data__SET(new short[] {(short)14603, (short) -16107, (short)22770, (short) -8698, (short) -29779, (short)27172, (short)31962, (short)15923, (short)11383, (short) -8921, (short)17286, (short)22488, (short)3225, (short) -7050, (short)20378, (short)1115}, 0) ;
            p134.lon = (int)811932306;
            p134.gridbit = (byte)(byte)229;
            p134.lat = (int)519202636;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)24086859);
                Debug.Assert(pack.lat == (int) -1034741204);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lon = (int)24086859;
            p135.lat = (int) -1034741204;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.terrain_height == (float) -2.394761E38F);
                Debug.Assert(pack.spacing == (ushort)(ushort)48783);
                Debug.Assert(pack.loaded == (ushort)(ushort)16495);
                Debug.Assert(pack.current_height == (float)2.881665E38F);
                Debug.Assert(pack.pending == (ushort)(ushort)55036);
                Debug.Assert(pack.lon == (int)1377387268);
                Debug.Assert(pack.lat == (int) -574099492);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.loaded = (ushort)(ushort)16495;
            p136.lat = (int) -574099492;
            p136.terrain_height = (float) -2.394761E38F;
            p136.pending = (ushort)(ushort)55036;
            p136.current_height = (float)2.881665E38F;
            p136.lon = (int)1377387268;
            p136.spacing = (ushort)(ushort)48783;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -17819);
                Debug.Assert(pack.press_diff == (float)1.1141917E38F);
                Debug.Assert(pack.press_abs == (float)2.6874095E38F);
                Debug.Assert(pack.time_boot_ms == (uint)197784005U);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.press_diff = (float)1.1141917E38F;
            p137.time_boot_ms = (uint)197784005U;
            p137.press_abs = (float)2.6874095E38F;
            p137.temperature = (short)(short) -17819;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.6124536E38F, 1.6267427E38F, 7.230378E37F, -1.0587625E38F}));
                Debug.Assert(pack.x == (float)2.4488207E38F);
                Debug.Assert(pack.z == (float)1.0514666E38F);
                Debug.Assert(pack.y == (float) -9.139052E37F);
                Debug.Assert(pack.time_usec == (ulong)4524744134946997447L);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.z = (float)1.0514666E38F;
            p138.q_SET(new float[] {1.6124536E38F, 1.6267427E38F, 7.230378E37F, -1.0587625E38F}, 0) ;
            p138.time_usec = (ulong)4524744134946997447L;
            p138.x = (float)2.4488207E38F;
            p138.y = (float) -9.139052E37F;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)10);
                Debug.Assert(pack.target_component == (byte)(byte)112);
                Debug.Assert(pack.time_usec == (ulong)3188465710858585899L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {2.532768E37F, 1.7115088E38F, 3.093051E38F, 1.493006E38F, 5.1036156E37F, 2.0787271E38F, 2.1505007E38F, 2.3872323E38F}));
                Debug.Assert(pack.group_mlx == (byte)(byte)251);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.target_system = (byte)(byte)10;
            p139.controls_SET(new float[] {2.532768E37F, 1.7115088E38F, 3.093051E38F, 1.493006E38F, 5.1036156E37F, 2.0787271E38F, 2.1505007E38F, 2.3872323E38F}, 0) ;
            p139.target_component = (byte)(byte)112;
            p139.time_usec = (ulong)3188465710858585899L;
            p139.group_mlx = (byte)(byte)251;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)5590660148973167147L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {1.235132E38F, 1.8937428E38F, 6.233892E37F, 1.2544333E38F, 1.6800493E38F, 3.0285396E38F, -7.9780843E37F, 6.110171E37F}));
                Debug.Assert(pack.group_mlx == (byte)(byte)4);
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.group_mlx = (byte)(byte)4;
            p140.controls_SET(new float[] {1.235132E38F, 1.8937428E38F, 6.233892E37F, 1.2544333E38F, 1.6800493E38F, 3.0285396E38F, -7.9780843E37F, 6.110171E37F}, 0) ;
            p140.time_usec = (ulong)5590660148973167147L;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_monotonic == (float) -2.960969E38F);
                Debug.Assert(pack.time_usec == (ulong)6528186084666948482L);
                Debug.Assert(pack.altitude_relative == (float) -1.8597227E38F);
                Debug.Assert(pack.altitude_local == (float)3.1405782E38F);
                Debug.Assert(pack.altitude_terrain == (float)1.6777442E38F);
                Debug.Assert(pack.altitude_amsl == (float) -4.672046E37F);
                Debug.Assert(pack.bottom_clearance == (float)3.9953634E37F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.altitude_amsl = (float) -4.672046E37F;
            p141.altitude_relative = (float) -1.8597227E38F;
            p141.altitude_terrain = (float)1.6777442E38F;
            p141.altitude_monotonic = (float) -2.960969E38F;
            p141.time_usec = (ulong)6528186084666948482L;
            p141.altitude_local = (float)3.1405782E38F;
            p141.bottom_clearance = (float)3.9953634E37F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)197, (byte)133, (byte)188, (byte)210, (byte)87, (byte)110, (byte)148, (byte)93, (byte)225, (byte)243, (byte)208, (byte)137, (byte)194, (byte)202, (byte)240, (byte)230, (byte)189, (byte)20, (byte)201, (byte)195, (byte)216, (byte)1, (byte)98, (byte)194, (byte)3, (byte)146, (byte)193, (byte)7, (byte)180, (byte)153, (byte)13, (byte)26, (byte)50, (byte)135, (byte)18, (byte)75, (byte)23, (byte)141, (byte)42, (byte)173, (byte)147, (byte)81, (byte)202, (byte)190, (byte)199, (byte)154, (byte)134, (byte)250, (byte)153, (byte)154, (byte)7, (byte)128, (byte)148, (byte)28, (byte)41, (byte)70, (byte)60, (byte)207, (byte)187, (byte)2, (byte)172, (byte)213, (byte)151, (byte)97, (byte)143, (byte)118, (byte)60, (byte)163, (byte)110, (byte)114, (byte)136, (byte)245, (byte)243, (byte)66, (byte)139, (byte)205, (byte)153, (byte)121, (byte)123, (byte)112, (byte)221, (byte)153, (byte)195, (byte)10, (byte)151, (byte)46, (byte)214, (byte)107, (byte)215, (byte)197, (byte)110, (byte)203, (byte)3, (byte)201, (byte)99, (byte)198, (byte)11, (byte)208, (byte)100, (byte)21, (byte)162, (byte)49, (byte)142, (byte)235, (byte)225, (byte)154, (byte)156, (byte)49, (byte)202, (byte)93, (byte)188, (byte)79, (byte)53, (byte)192, (byte)172, (byte)102, (byte)81, (byte)70, (byte)52, (byte)46}));
                Debug.Assert(pack.request_id == (byte)(byte)184);
                Debug.Assert(pack.uri_type == (byte)(byte)244);
                Debug.Assert(pack.transfer_type == (byte)(byte)136);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)93, (byte)179, (byte)105, (byte)199, (byte)181, (byte)126, (byte)237, (byte)149, (byte)157, (byte)125, (byte)59, (byte)139, (byte)217, (byte)65, (byte)86, (byte)205, (byte)58, (byte)225, (byte)83, (byte)145, (byte)6, (byte)179, (byte)151, (byte)102, (byte)83, (byte)137, (byte)91, (byte)150, (byte)61, (byte)143, (byte)117, (byte)208, (byte)201, (byte)165, (byte)236, (byte)126, (byte)37, (byte)12, (byte)31, (byte)121, (byte)221, (byte)59, (byte)31, (byte)192, (byte)222, (byte)121, (byte)100, (byte)209, (byte)150, (byte)1, (byte)29, (byte)201, (byte)157, (byte)228, (byte)130, (byte)217, (byte)118, (byte)187, (byte)77, (byte)58, (byte)200, (byte)179, (byte)107, (byte)174, (byte)202, (byte)219, (byte)133, (byte)151, (byte)91, (byte)155, (byte)187, (byte)6, (byte)243, (byte)147, (byte)152, (byte)33, (byte)174, (byte)67, (byte)158, (byte)47, (byte)117, (byte)17, (byte)9, (byte)168, (byte)96, (byte)35, (byte)103, (byte)192, (byte)157, (byte)44, (byte)77, (byte)168, (byte)201, (byte)24, (byte)227, (byte)12, (byte)202, (byte)235, (byte)247, (byte)64, (byte)113, (byte)254, (byte)193, (byte)46, (byte)136, (byte)202, (byte)140, (byte)3, (byte)248, (byte)143, (byte)200, (byte)149, (byte)152, (byte)238, (byte)94, (byte)156, (byte)184, (byte)148, (byte)198, (byte)252}));
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)184;
            p142.storage_SET(new byte[] {(byte)197, (byte)133, (byte)188, (byte)210, (byte)87, (byte)110, (byte)148, (byte)93, (byte)225, (byte)243, (byte)208, (byte)137, (byte)194, (byte)202, (byte)240, (byte)230, (byte)189, (byte)20, (byte)201, (byte)195, (byte)216, (byte)1, (byte)98, (byte)194, (byte)3, (byte)146, (byte)193, (byte)7, (byte)180, (byte)153, (byte)13, (byte)26, (byte)50, (byte)135, (byte)18, (byte)75, (byte)23, (byte)141, (byte)42, (byte)173, (byte)147, (byte)81, (byte)202, (byte)190, (byte)199, (byte)154, (byte)134, (byte)250, (byte)153, (byte)154, (byte)7, (byte)128, (byte)148, (byte)28, (byte)41, (byte)70, (byte)60, (byte)207, (byte)187, (byte)2, (byte)172, (byte)213, (byte)151, (byte)97, (byte)143, (byte)118, (byte)60, (byte)163, (byte)110, (byte)114, (byte)136, (byte)245, (byte)243, (byte)66, (byte)139, (byte)205, (byte)153, (byte)121, (byte)123, (byte)112, (byte)221, (byte)153, (byte)195, (byte)10, (byte)151, (byte)46, (byte)214, (byte)107, (byte)215, (byte)197, (byte)110, (byte)203, (byte)3, (byte)201, (byte)99, (byte)198, (byte)11, (byte)208, (byte)100, (byte)21, (byte)162, (byte)49, (byte)142, (byte)235, (byte)225, (byte)154, (byte)156, (byte)49, (byte)202, (byte)93, (byte)188, (byte)79, (byte)53, (byte)192, (byte)172, (byte)102, (byte)81, (byte)70, (byte)52, (byte)46}, 0) ;
            p142.uri_SET(new byte[] {(byte)93, (byte)179, (byte)105, (byte)199, (byte)181, (byte)126, (byte)237, (byte)149, (byte)157, (byte)125, (byte)59, (byte)139, (byte)217, (byte)65, (byte)86, (byte)205, (byte)58, (byte)225, (byte)83, (byte)145, (byte)6, (byte)179, (byte)151, (byte)102, (byte)83, (byte)137, (byte)91, (byte)150, (byte)61, (byte)143, (byte)117, (byte)208, (byte)201, (byte)165, (byte)236, (byte)126, (byte)37, (byte)12, (byte)31, (byte)121, (byte)221, (byte)59, (byte)31, (byte)192, (byte)222, (byte)121, (byte)100, (byte)209, (byte)150, (byte)1, (byte)29, (byte)201, (byte)157, (byte)228, (byte)130, (byte)217, (byte)118, (byte)187, (byte)77, (byte)58, (byte)200, (byte)179, (byte)107, (byte)174, (byte)202, (byte)219, (byte)133, (byte)151, (byte)91, (byte)155, (byte)187, (byte)6, (byte)243, (byte)147, (byte)152, (byte)33, (byte)174, (byte)67, (byte)158, (byte)47, (byte)117, (byte)17, (byte)9, (byte)168, (byte)96, (byte)35, (byte)103, (byte)192, (byte)157, (byte)44, (byte)77, (byte)168, (byte)201, (byte)24, (byte)227, (byte)12, (byte)202, (byte)235, (byte)247, (byte)64, (byte)113, (byte)254, (byte)193, (byte)46, (byte)136, (byte)202, (byte)140, (byte)3, (byte)248, (byte)143, (byte)200, (byte)149, (byte)152, (byte)238, (byte)94, (byte)156, (byte)184, (byte)148, (byte)198, (byte)252}, 0) ;
            p142.uri_type = (byte)(byte)244;
            p142.transfer_type = (byte)(byte)136;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1070306583U);
                Debug.Assert(pack.press_diff == (float)3.2326355E37F);
                Debug.Assert(pack.temperature == (short)(short)11784);
                Debug.Assert(pack.press_abs == (float)2.0828554E38F);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)1070306583U;
            p143.temperature = (short)(short)11784;
            p143.press_diff = (float)3.2326355E37F;
            p143.press_abs = (float)2.0828554E38F;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -490747996);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {2.9119857E38F, 8.628502E37F, 4.982595E37F}));
                Debug.Assert(pack.timestamp == (ulong)6957603374566013841L);
                Debug.Assert(pack.alt == (float) -1.927792E38F);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {-1.3294393E38F, -3.123293E38F, 3.2511114E38F}));
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {1.1299739E38F, 9.8926274E36F, 3.1247415E38F, -1.0601271E38F}));
                Debug.Assert(pack.est_capabilities == (byte)(byte)25);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {-6.6232437E37F, 6.1785965E37F, -1.7593893E38F}));
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-2.645423E37F, 1.4772651E38F, -2.9097767E38F}));
                Debug.Assert(pack.custom_state == (ulong)1092904192669204347L);
                Debug.Assert(pack.lat == (int)304752928);
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.attitude_q_SET(new float[] {1.1299739E38F, 9.8926274E36F, 3.1247415E38F, -1.0601271E38F}, 0) ;
            p144.acc_SET(new float[] {-6.6232437E37F, 6.1785965E37F, -1.7593893E38F}, 0) ;
            p144.lat = (int)304752928;
            p144.vel_SET(new float[] {-2.645423E37F, 1.4772651E38F, -2.9097767E38F}, 0) ;
            p144.timestamp = (ulong)6957603374566013841L;
            p144.custom_state = (ulong)1092904192669204347L;
            p144.lon = (int) -490747996;
            p144.alt = (float) -1.927792E38F;
            p144.est_capabilities = (byte)(byte)25;
            p144.position_cov_SET(new float[] {2.9119857E38F, 8.628502E37F, 4.982595E37F}, 0) ;
            p144.rates_SET(new float[] {-1.3294393E38F, -3.123293E38F, 3.2511114E38F}, 0) ;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw_rate == (float)2.331913E38F);
                Debug.Assert(pack.y_acc == (float) -1.6563969E38F);
                Debug.Assert(pack.roll_rate == (float) -9.80415E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.1617872E38F, -3.81956E37F, -3.1197618E38F, 4.391266E37F}));
                Debug.Assert(pack.pitch_rate == (float)1.0685139E38F);
                Debug.Assert(pack.z_pos == (float) -3.213839E38F);
                Debug.Assert(pack.x_pos == (float)3.0174958E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {2.9702435E38F, -3.0475937E38F, -2.365945E38F}));
                Debug.Assert(pack.x_acc == (float)3.176545E37F);
                Debug.Assert(pack.y_pos == (float)1.5808628E38F);
                Debug.Assert(pack.z_vel == (float) -1.1384641E38F);
                Debug.Assert(pack.airspeed == (float)1.926426E38F);
                Debug.Assert(pack.x_vel == (float) -1.5859569E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {6.744107E37F, 2.7645762E38F, 2.5079335E38F}));
                Debug.Assert(pack.time_usec == (ulong)8783249755469182668L);
                Debug.Assert(pack.z_acc == (float)3.3699514E38F);
                Debug.Assert(pack.y_vel == (float) -1.3908887E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.x_pos = (float)3.0174958E38F;
            p146.q_SET(new float[] {-2.1617872E38F, -3.81956E37F, -3.1197618E38F, 4.391266E37F}, 0) ;
            p146.airspeed = (float)1.926426E38F;
            p146.vel_variance_SET(new float[] {2.9702435E38F, -3.0475937E38F, -2.365945E38F}, 0) ;
            p146.pitch_rate = (float)1.0685139E38F;
            p146.x_acc = (float)3.176545E37F;
            p146.pos_variance_SET(new float[] {6.744107E37F, 2.7645762E38F, 2.5079335E38F}, 0) ;
            p146.roll_rate = (float) -9.80415E37F;
            p146.x_vel = (float) -1.5859569E38F;
            p146.yaw_rate = (float)2.331913E38F;
            p146.y_pos = (float)1.5808628E38F;
            p146.z_pos = (float) -3.213839E38F;
            p146.z_vel = (float) -1.1384641E38F;
            p146.y_vel = (float) -1.3908887E38F;
            p146.z_acc = (float)3.3699514E38F;
            p146.y_acc = (float) -1.6563969E38F;
            p146.time_usec = (ulong)8783249755469182668L;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_consumed == (int) -1046263741);
                Debug.Assert(pack.id == (byte)(byte)174);
                Debug.Assert(pack.energy_consumed == (int) -1726509638);
                Debug.Assert(pack.battery_function == MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
                Debug.Assert(pack.current_battery == (short)(short) -3929);
                Debug.Assert(pack.type == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)51448, (ushort)40787, (ushort)58754, (ushort)7221, (ushort)63180, (ushort)17308, (ushort)49926, (ushort)57555, (ushort)63175, (ushort)28515}));
                Debug.Assert(pack.temperature == (short)(short) -6773);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 70);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.temperature = (short)(short) -6773;
            p147.energy_consumed = (int) -1726509638;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH;
            p147.battery_remaining = (sbyte)(sbyte) - 70;
            p147.current_battery = (short)(short) -3929;
            p147.voltages_SET(new ushort[] {(ushort)51448, (ushort)40787, (ushort)58754, (ushort)7221, (ushort)63180, (ushort)17308, (ushort)49926, (ushort)57555, (ushort)63175, (ushort)28515}, 0) ;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD;
            p147.current_consumed = (int) -1046263741;
            p147.id = (byte)(byte)174;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)68, (byte)19, (byte)142, (byte)43, (byte)142, (byte)73, (byte)161, (byte)19}));
                Debug.Assert(pack.uid == (ulong)1031224811274753612L);
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)14, (byte)239, (byte)100, (byte)192, (byte)163, (byte)42, (byte)249, (byte)206}));
                Debug.Assert(pack.middleware_sw_version == (uint)3306114786U);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)51, (byte)31, (byte)21, (byte)179, (byte)242, (byte)96, (byte)47, (byte)227, (byte)242, (byte)110, (byte)249, (byte)130, (byte)183, (byte)182, (byte)125, (byte)29, (byte)211, (byte)137}));
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)161, (byte)153, (byte)23, (byte)210, (byte)164, (byte)144, (byte)102, (byte)98}));
                Debug.Assert(pack.vendor_id == (ushort)(ushort)49082);
                Debug.Assert(pack.board_version == (uint)358652174U);
                Debug.Assert(pack.product_id == (ushort)(ushort)59765);
                Debug.Assert(pack.os_sw_version == (uint)1973497740U);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT));
                Debug.Assert(pack.flight_sw_version == (uint)3484184859U);
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.flight_sw_version = (uint)3484184859U;
            p148.flight_custom_version_SET(new byte[] {(byte)68, (byte)19, (byte)142, (byte)43, (byte)142, (byte)73, (byte)161, (byte)19}, 0) ;
            p148.os_custom_version_SET(new byte[] {(byte)161, (byte)153, (byte)23, (byte)210, (byte)164, (byte)144, (byte)102, (byte)98}, 0) ;
            p148.vendor_id = (ushort)(ushort)49082;
            p148.os_sw_version = (uint)1973497740U;
            p148.middleware_sw_version = (uint)3306114786U;
            p148.product_id = (ushort)(ushort)59765;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT);
            p148.uid2_SET(new byte[] {(byte)51, (byte)31, (byte)21, (byte)179, (byte)242, (byte)96, (byte)47, (byte)227, (byte)242, (byte)110, (byte)249, (byte)130, (byte)183, (byte)182, (byte)125, (byte)29, (byte)211, (byte)137}, 0, PH) ;
            p148.uid = (ulong)1031224811274753612L;
            p148.board_version = (uint)358652174U;
            p148.middleware_custom_version_SET(new byte[] {(byte)14, (byte)239, (byte)100, (byte)192, (byte)163, (byte)42, (byte)249, (byte)206}, 0) ;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_num == (byte)(byte)166);
                Debug.Assert(pack.type == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
                Debug.Assert(pack.x_TRY(ph) == (float) -2.4810993E38F);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {-3.361457E37F, -1.3379476E38F, 3.6748402E37F, -2.9067703E38F}));
                Debug.Assert(pack.time_usec == (ulong)6858826236204586920L);
                Debug.Assert(pack.z_TRY(ph) == (float) -2.8087284E38F);
                Debug.Assert(pack.size_x == (float)2.295289E37F);
                Debug.Assert(pack.y_TRY(ph) == (float) -2.1840555E38F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)52);
                Debug.Assert(pack.angle_x == (float)3.7700902E37F);
                Debug.Assert(pack.angle_y == (float)2.5695627E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.size_y == (float)3.0018607E38F);
                Debug.Assert(pack.distance == (float) -1.454421E38F);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.y_SET((float) -2.1840555E38F, PH) ;
            p149.target_num = (byte)(byte)166;
            p149.time_usec = (ulong)6858826236204586920L;
            p149.q_SET(new float[] {-3.361457E37F, -1.3379476E38F, 3.6748402E37F, -2.9067703E38F}, 0, PH) ;
            p149.size_x = (float)2.295289E37F;
            p149.x_SET((float) -2.4810993E38F, PH) ;
            p149.frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p149.z_SET((float) -2.8087284E38F, PH) ;
            p149.angle_y = (float)2.5695627E38F;
            p149.angle_x = (float)3.7700902E37F;
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON;
            p149.size_y = (float)3.0018607E38F;
            p149.distance = (float) -1.454421E38F;
            p149.position_valid_SET((byte)(byte)52, PH) ;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_0Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ar_i8.SequenceEqual(new sbyte[] {(sbyte)91, (sbyte) - 88, (sbyte)105, (sbyte)105}));
                Debug.Assert(pack.v1 == (byte)(byte)194);
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {4268467720U, 3104113189U, 2832500614U, 1795249193U}));
                Debug.Assert(pack.ar_u16.SequenceEqual(new ushort[] {(ushort)45650, (ushort)45632, (ushort)3596, (ushort)8501}));
                Debug.Assert(pack.ar_u8.SequenceEqual(new byte[] {(byte)133, (byte)114, (byte)86, (byte)98}));
            };
            GroundControl.ARRAY_TEST_0 p150 = CommunicationChannel.new_ARRAY_TEST_0();
            PH.setPack(p150);
            p150.ar_u16_SET(new ushort[] {(ushort)45650, (ushort)45632, (ushort)3596, (ushort)8501}, 0) ;
            p150.ar_u8_SET(new byte[] {(byte)133, (byte)114, (byte)86, (byte)98}, 0) ;
            p150.v1 = (byte)(byte)194;
            p150.ar_i8_SET(new sbyte[] {(sbyte)91, (sbyte) - 88, (sbyte)105, (sbyte)105}, 0) ;
            p150.ar_u32_SET(new uint[] {4268467720U, 3104113189U, 2832500614U, 1795249193U}, 0) ;
            CommunicationChannel.instance.send(p150);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_1Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {1945409886U, 752350772U, 4127100025U, 943060023U}));
            };
            GroundControl.ARRAY_TEST_1 p151 = CommunicationChannel.new_ARRAY_TEST_1();
            PH.setPack(p151);
            p151.ar_u32_SET(new uint[] {1945409886U, 752350772U, 4127100025U, 943060023U}, 0) ;
            CommunicationChannel.instance.send(p151);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.v == (byte)(byte)69);
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {676890831U, 1758136660U, 2879930427U, 123681193U}));
            };
            GroundControl.ARRAY_TEST_3 p153 = CommunicationChannel.new_ARRAY_TEST_3();
            PH.setPack(p153);
            p153.v = (byte)(byte)69;
            p153.ar_u32_SET(new uint[] {676890831U, 1758136660U, 2879930427U, 123681193U}, 0) ;
            CommunicationChannel.instance.send(p153);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_4Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {159125610U, 3123082419U, 206525904U, 553145927U}));
                Debug.Assert(pack.v == (byte)(byte)39);
            };
            GroundControl.ARRAY_TEST_4 p154 = CommunicationChannel.new_ARRAY_TEST_4();
            PH.setPack(p154);
            p154.ar_u32_SET(new uint[] {159125610U, 3123082419U, 206525904U, 553145927U}, 0) ;
            p154.v = (byte)(byte)39;
            CommunicationChannel.instance.send(p154);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_5Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.c2_LEN(ph) == 5);
                Debug.Assert(pack.c2_TRY(ph).Equals("vucfm"));
                Debug.Assert(pack.c1_LEN(ph) == 4);
                Debug.Assert(pack.c1_TRY(ph).Equals("yxyC"));
            };
            GroundControl.ARRAY_TEST_5 p155 = CommunicationChannel.new_ARRAY_TEST_5();
            PH.setPack(p155);
            p155.c1_SET("yxyC", PH) ;
            p155.c2_SET("vucfm", PH) ;
            CommunicationChannel.instance.send(p155);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_6Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ar_d.SequenceEqual(new double[] {-9.262112005657244E307, 1.5425331074736198E308}));
                Debug.Assert(pack.ar_i32.SequenceEqual(new int[] {-1794838991, -77344891}));
                Debug.Assert(pack.v2 == (ushort)(ushort)40554);
                Debug.Assert(pack.v1 == (byte)(byte)198);
                Debug.Assert(pack.ar_u8.SequenceEqual(new byte[] {(byte)249, (byte)96}));
                Debug.Assert(pack.ar_u16.SequenceEqual(new ushort[] {(ushort)14561, (ushort)62416}));
                Debug.Assert(pack.ar_c_LEN(ph) == 20);
                Debug.Assert(pack.ar_c_TRY(ph).Equals("tNnjfnHfosvYLhevzuqh"));
                Debug.Assert(pack.ar_i16.SequenceEqual(new short[] {(short)3790, (short) -3289}));
                Debug.Assert(pack.ar_i8.SequenceEqual(new sbyte[] {(sbyte) - 87, (sbyte)19}));
                Debug.Assert(pack.v3 == (uint)3167129999U);
                Debug.Assert(pack.ar_f.SequenceEqual(new float[] {-1.0078618E38F, -2.505486E38F}));
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {1466227336U, 3870426085U}));
            };
            GroundControl.ARRAY_TEST_6 p156 = CommunicationChannel.new_ARRAY_TEST_6();
            PH.setPack(p156);
            p156.ar_u8_SET(new byte[] {(byte)249, (byte)96}, 0) ;
            p156.ar_i16_SET(new short[] {(short)3790, (short) -3289}, 0) ;
            p156.v1 = (byte)(byte)198;
            p156.ar_i8_SET(new sbyte[] {(sbyte) - 87, (sbyte)19}, 0) ;
            p156.v2 = (ushort)(ushort)40554;
            p156.ar_c_SET("tNnjfnHfosvYLhevzuqh", PH) ;
            p156.ar_i32_SET(new int[] {-1794838991, -77344891}, 0) ;
            p156.ar_d_SET(new double[] {-9.262112005657244E307, 1.5425331074736198E308}, 0) ;
            p156.ar_u32_SET(new uint[] {1466227336U, 3870426085U}, 0) ;
            p156.ar_u16_SET(new ushort[] {(ushort)14561, (ushort)62416}, 0) ;
            p156.ar_f_SET(new float[] {-1.0078618E38F, -2.505486E38F}, 0) ;
            p156.v3 = (uint)3167129999U;
            CommunicationChannel.instance.send(p156);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_7Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ar_c_LEN(ph) == 32);
                Debug.Assert(pack.ar_c_TRY(ph).Equals("edeeWsuizroydxpNlejMykorethfihaO"));
                Debug.Assert(pack.ar_u32.SequenceEqual(new uint[] {1558530986U, 1574168709U}));
                Debug.Assert(pack.ar_d.SequenceEqual(new double[] {1.5776971228064527E308, -8.642742390030827E307}));
                Debug.Assert(pack.ar_f.SequenceEqual(new float[] {-1.6945706E38F, 1.0927585E38F}));
                Debug.Assert(pack.ar_i8.SequenceEqual(new sbyte[] {(sbyte)88, (sbyte) - 85}));
                Debug.Assert(pack.ar_i16.SequenceEqual(new short[] {(short) -20754, (short) -19925}));
                Debug.Assert(pack.ar_u16.SequenceEqual(new ushort[] {(ushort)58172, (ushort)54999}));
                Debug.Assert(pack.ar_i32.SequenceEqual(new int[] {-1202670487, 1782251993}));
                Debug.Assert(pack.ar_u8.SequenceEqual(new byte[] {(byte)111, (byte)86}));
            };
            GroundControl.ARRAY_TEST_7 p157 = CommunicationChannel.new_ARRAY_TEST_7();
            PH.setPack(p157);
            p157.ar_u32_SET(new uint[] {1558530986U, 1574168709U}, 0) ;
            p157.ar_u16_SET(new ushort[] {(ushort)58172, (ushort)54999}, 0) ;
            p157.ar_i8_SET(new sbyte[] {(sbyte)88, (sbyte) - 85}, 0) ;
            p157.ar_c_SET("edeeWsuizroydxpNlejMykorethfihaO", PH) ;
            p157.ar_u8_SET(new byte[] {(byte)111, (byte)86}, 0) ;
            p157.ar_f_SET(new float[] {-1.6945706E38F, 1.0927585E38F}, 0) ;
            p157.ar_i16_SET(new short[] {(short) -20754, (short) -19925}, 0) ;
            p157.ar_d_SET(new double[] {1.5776971228064527E308, -8.642742390030827E307}, 0) ;
            p157.ar_i32_SET(new int[] {-1202670487, 1782251993}, 0) ;
            CommunicationChannel.instance.send(p157);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnARRAY_TEST_8Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.v3 == (uint)1539174875U);
                Debug.Assert(pack.ar_d.SequenceEqual(new double[] {-8.483693008293993E307, -1.3587494701675212E308}));
                Debug.Assert(pack.ar_u16.SequenceEqual(new ushort[] {(ushort)919, (ushort)16004}));
            };
            GroundControl.ARRAY_TEST_8 p158 = CommunicationChannel.new_ARRAY_TEST_8();
            PH.setPack(p158);
            p158.v3 = (uint)1539174875U;
            p158.ar_u16_SET(new ushort[] {(ushort)919, (ushort)16004}, 0) ;
            p158.ar_d_SET(new double[] {-8.483693008293993E307, -1.3587494701675212E308}, 0) ;
            CommunicationChannel.instance.send(p158);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mag_ratio == (float)2.9867233E38F);
                Debug.Assert(pack.time_usec == (ulong)8284506320794375370L);
                Debug.Assert(pack.pos_horiz_ratio == (float)9.672208E37F);
                Debug.Assert(pack.hagl_ratio == (float) -1.8221926E38F);
                Debug.Assert(pack.tas_ratio == (float) -1.9225013E38F);
                Debug.Assert(pack.vel_ratio == (float)5.8680925E37F);
                Debug.Assert(pack.pos_vert_ratio == (float) -1.2985551E38F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS));
                Debug.Assert(pack.pos_horiz_accuracy == (float)1.9558356E38F);
                Debug.Assert(pack.pos_vert_accuracy == (float)2.0281588E38F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.pos_horiz_ratio = (float)9.672208E37F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS);
            p230.hagl_ratio = (float) -1.8221926E38F;
            p230.pos_vert_accuracy = (float)2.0281588E38F;
            p230.mag_ratio = (float)2.9867233E38F;
            p230.vel_ratio = (float)5.8680925E37F;
            p230.pos_vert_ratio = (float) -1.2985551E38F;
            p230.pos_horiz_accuracy = (float)1.9558356E38F;
            p230.time_usec = (ulong)8284506320794375370L;
            p230.tas_ratio = (float) -1.9225013E38F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wind_x == (float) -3.4639133E37F);
                Debug.Assert(pack.wind_y == (float)2.8859242E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -2.2626623E37F);
                Debug.Assert(pack.time_usec == (ulong)1195621651210952177L);
                Debug.Assert(pack.var_horiz == (float)3.7824171E37F);
                Debug.Assert(pack.var_vert == (float) -3.300302E38F);
                Debug.Assert(pack.vert_accuracy == (float)3.792705E37F);
                Debug.Assert(pack.wind_z == (float) -6.249015E37F);
                Debug.Assert(pack.wind_alt == (float) -2.024391E38F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.wind_y = (float)2.8859242E38F;
            p231.vert_accuracy = (float)3.792705E37F;
            p231.wind_alt = (float) -2.024391E38F;
            p231.var_vert = (float) -3.300302E38F;
            p231.wind_z = (float) -6.249015E37F;
            p231.wind_x = (float) -3.4639133E37F;
            p231.time_usec = (ulong)1195621651210952177L;
            p231.horiz_accuracy = (float) -2.2626623E37F;
            p231.var_horiz = (float)3.7824171E37F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vdop == (float) -3.004825E38F);
                Debug.Assert(pack.time_week_ms == (uint)2325256242U);
                Debug.Assert(pack.vn == (float)3.1610764E38F);
                Debug.Assert(pack.vert_accuracy == (float)2.7217049E38F);
                Debug.Assert(pack.time_week == (ushort)(ushort)31732);
                Debug.Assert(pack.hdop == (float) -1.7831719E38F);
                Debug.Assert(pack.time_usec == (ulong)7638561144272031824L);
                Debug.Assert(pack.ve == (float)2.053606E37F);
                Debug.Assert(pack.alt == (float)3.224698E38F);
                Debug.Assert(pack.speed_accuracy == (float)2.3223337E38F);
                Debug.Assert(pack.lon == (int)1122885942);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ));
                Debug.Assert(pack.satellites_visible == (byte)(byte)193);
                Debug.Assert(pack.vd == (float)3.251768E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -1.5984969E38F);
                Debug.Assert(pack.lat == (int) -471601907);
                Debug.Assert(pack.gps_id == (byte)(byte)96);
                Debug.Assert(pack.fix_type == (byte)(byte)114);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.alt = (float)3.224698E38F;
            p232.vdop = (float) -3.004825E38F;
            p232.vn = (float)3.1610764E38F;
            p232.speed_accuracy = (float)2.3223337E38F;
            p232.vert_accuracy = (float)2.7217049E38F;
            p232.lat = (int) -471601907;
            p232.time_week = (ushort)(ushort)31732;
            p232.vd = (float)3.251768E38F;
            p232.ve = (float)2.053606E37F;
            p232.time_usec = (ulong)7638561144272031824L;
            p232.time_week_ms = (uint)2325256242U;
            p232.fix_type = (byte)(byte)114;
            p232.satellites_visible = (byte)(byte)193;
            p232.lon = (int)1122885942;
            p232.horiz_accuracy = (float) -1.5984969E38F;
            p232.hdop = (float) -1.7831719E38F;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ);
            p232.gps_id = (byte)(byte)96;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.len == (byte)(byte)39);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)69, (byte)172, (byte)94, (byte)193, (byte)130, (byte)137, (byte)162, (byte)105, (byte)205, (byte)114, (byte)118, (byte)47, (byte)85, (byte)149, (byte)64, (byte)155, (byte)124, (byte)165, (byte)63, (byte)88, (byte)168, (byte)249, (byte)50, (byte)144, (byte)248, (byte)207, (byte)124, (byte)214, (byte)238, (byte)98, (byte)177, (byte)240, (byte)117, (byte)169, (byte)97, (byte)244, (byte)87, (byte)212, (byte)198, (byte)30, (byte)144, (byte)84, (byte)90, (byte)239, (byte)118, (byte)189, (byte)59, (byte)210, (byte)58, (byte)138, (byte)153, (byte)77, (byte)216, (byte)159, (byte)195, (byte)231, (byte)113, (byte)3, (byte)248, (byte)247, (byte)178, (byte)134, (byte)3, (byte)74, (byte)72, (byte)118, (byte)52, (byte)52, (byte)41, (byte)46, (byte)133, (byte)100, (byte)234, (byte)239, (byte)40, (byte)67, (byte)145, (byte)129, (byte)115, (byte)254, (byte)168, (byte)15, (byte)69, (byte)88, (byte)80, (byte)103, (byte)198, (byte)225, (byte)237, (byte)62, (byte)254, (byte)130, (byte)22, (byte)170, (byte)56, (byte)155, (byte)121, (byte)93, (byte)33, (byte)207, (byte)221, (byte)211, (byte)248, (byte)227, (byte)96, (byte)59, (byte)239, (byte)188, (byte)37, (byte)209, (byte)90, (byte)45, (byte)112, (byte)24, (byte)181, (byte)101, (byte)148, (byte)170, (byte)55, (byte)198, (byte)30, (byte)105, (byte)110, (byte)212, (byte)71, (byte)67, (byte)15, (byte)131, (byte)40, (byte)142, (byte)249, (byte)146, (byte)250, (byte)67, (byte)195, (byte)232, (byte)195, (byte)93, (byte)51, (byte)232, (byte)194, (byte)120, (byte)142, (byte)56, (byte)183, (byte)194, (byte)175, (byte)125, (byte)181, (byte)128, (byte)157, (byte)234, (byte)167, (byte)95, (byte)67, (byte)71, (byte)198, (byte)77, (byte)83, (byte)177, (byte)162, (byte)218, (byte)103, (byte)12, (byte)203, (byte)169, (byte)252, (byte)179, (byte)231, (byte)101, (byte)99, (byte)17, (byte)10, (byte)42, (byte)30, (byte)104, (byte)233, (byte)27, (byte)136, (byte)242}));
                Debug.Assert(pack.flags == (byte)(byte)73);
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.data__SET(new byte[] {(byte)69, (byte)172, (byte)94, (byte)193, (byte)130, (byte)137, (byte)162, (byte)105, (byte)205, (byte)114, (byte)118, (byte)47, (byte)85, (byte)149, (byte)64, (byte)155, (byte)124, (byte)165, (byte)63, (byte)88, (byte)168, (byte)249, (byte)50, (byte)144, (byte)248, (byte)207, (byte)124, (byte)214, (byte)238, (byte)98, (byte)177, (byte)240, (byte)117, (byte)169, (byte)97, (byte)244, (byte)87, (byte)212, (byte)198, (byte)30, (byte)144, (byte)84, (byte)90, (byte)239, (byte)118, (byte)189, (byte)59, (byte)210, (byte)58, (byte)138, (byte)153, (byte)77, (byte)216, (byte)159, (byte)195, (byte)231, (byte)113, (byte)3, (byte)248, (byte)247, (byte)178, (byte)134, (byte)3, (byte)74, (byte)72, (byte)118, (byte)52, (byte)52, (byte)41, (byte)46, (byte)133, (byte)100, (byte)234, (byte)239, (byte)40, (byte)67, (byte)145, (byte)129, (byte)115, (byte)254, (byte)168, (byte)15, (byte)69, (byte)88, (byte)80, (byte)103, (byte)198, (byte)225, (byte)237, (byte)62, (byte)254, (byte)130, (byte)22, (byte)170, (byte)56, (byte)155, (byte)121, (byte)93, (byte)33, (byte)207, (byte)221, (byte)211, (byte)248, (byte)227, (byte)96, (byte)59, (byte)239, (byte)188, (byte)37, (byte)209, (byte)90, (byte)45, (byte)112, (byte)24, (byte)181, (byte)101, (byte)148, (byte)170, (byte)55, (byte)198, (byte)30, (byte)105, (byte)110, (byte)212, (byte)71, (byte)67, (byte)15, (byte)131, (byte)40, (byte)142, (byte)249, (byte)146, (byte)250, (byte)67, (byte)195, (byte)232, (byte)195, (byte)93, (byte)51, (byte)232, (byte)194, (byte)120, (byte)142, (byte)56, (byte)183, (byte)194, (byte)175, (byte)125, (byte)181, (byte)128, (byte)157, (byte)234, (byte)167, (byte)95, (byte)67, (byte)71, (byte)198, (byte)77, (byte)83, (byte)177, (byte)162, (byte)218, (byte)103, (byte)12, (byte)203, (byte)169, (byte)252, (byte)179, (byte)231, (byte)101, (byte)99, (byte)17, (byte)10, (byte)42, (byte)30, (byte)104, (byte)233, (byte)27, (byte)136, (byte)242}, 0) ;
            p233.flags = (byte)(byte)73;
            p233.len = (byte)(byte)39;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (short)(short) -18224);
                Debug.Assert(pack.failsafe == (byte)(byte)162);
                Debug.Assert(pack.altitude_amsl == (short)(short)12317);
                Debug.Assert(pack.gps_fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
                Debug.Assert(pack.latitude == (int)2142526905);
                Debug.Assert(pack.temperature == (sbyte)(sbyte) - 5);
                Debug.Assert(pack.roll == (short)(short)28494);
                Debug.Assert(pack.wp_num == (byte)(byte)237);
                Debug.Assert(pack.heading_sp == (short)(short) -1349);
                Debug.Assert(pack.battery_remaining == (byte)(byte)83);
                Debug.Assert(pack.longitude == (int) -2139372631);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)251);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)83);
                Debug.Assert(pack.custom_mode == (uint)3648183851U);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 66);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED));
                Debug.Assert(pack.wp_distance == (ushort)(ushort)63764);
                Debug.Assert(pack.heading == (ushort)(ushort)11700);
                Debug.Assert(pack.airspeed == (byte)(byte)244);
                Debug.Assert(pack.groundspeed == (byte)(byte)135);
                Debug.Assert(pack.gps_nsat == (byte)(byte)188);
                Debug.Assert(pack.altitude_sp == (short)(short)11870);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)59);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.heading_sp = (short)(short) -1349;
            p234.airspeed_sp = (byte)(byte)251;
            p234.throttle = (sbyte)(sbyte)83;
            p234.temperature_air = (sbyte)(sbyte)59;
            p234.longitude = (int) -2139372631;
            p234.pitch = (short)(short) -18224;
            p234.airspeed = (byte)(byte)244;
            p234.temperature = (sbyte)(sbyte) - 5;
            p234.roll = (short)(short)28494;
            p234.latitude = (int)2142526905;
            p234.wp_distance = (ushort)(ushort)63764;
            p234.failsafe = (byte)(byte)162;
            p234.gps_nsat = (byte)(byte)188;
            p234.groundspeed = (byte)(byte)135;
            p234.climb_rate = (sbyte)(sbyte) - 66;
            p234.altitude_sp = (short)(short)11870;
            p234.custom_mode = (uint)3648183851U;
            p234.battery_remaining = (byte)(byte)83;
            p234.wp_num = (byte)(byte)237;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT;
            p234.altitude_amsl = (short)(short)12317;
            p234.heading = (ushort)(ushort)11700;
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED);
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.clipping_2 == (uint)2206857466U);
                Debug.Assert(pack.vibration_x == (float) -1.520458E38F);
                Debug.Assert(pack.time_usec == (ulong)3164449944994629361L);
                Debug.Assert(pack.vibration_y == (float) -3.0655704E38F);
                Debug.Assert(pack.clipping_0 == (uint)510662408U);
                Debug.Assert(pack.clipping_1 == (uint)3666174025U);
                Debug.Assert(pack.vibration_z == (float) -3.2454153E38F);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.clipping_2 = (uint)2206857466U;
            p241.clipping_0 = (uint)510662408U;
            p241.clipping_1 = (uint)3666174025U;
            p241.vibration_x = (float) -1.520458E38F;
            p241.vibration_z = (float) -3.2454153E38F;
            p241.vibration_y = (float) -3.0655704E38F;
            p241.time_usec = (ulong)3164449944994629361L;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.approach_y == (float) -2.2876595E38F);
                Debug.Assert(pack.altitude == (int)559937047);
                Debug.Assert(pack.longitude == (int) -1943598307);
                Debug.Assert(pack.approach_x == (float) -3.2673183E38F);
                Debug.Assert(pack.approach_z == (float)6.174118E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.3799507E38F, 2.5415965E38F, 1.8848506E38F, 1.6373537E38F}));
                Debug.Assert(pack.y == (float) -1.1899756E38F);
                Debug.Assert(pack.z == (float)3.3349873E38F);
                Debug.Assert(pack.x == (float)2.471904E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)6915914789618010017L);
                Debug.Assert(pack.latitude == (int)398000162);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.approach_x = (float) -3.2673183E38F;
            p242.time_usec_SET((ulong)6915914789618010017L, PH) ;
            p242.longitude = (int) -1943598307;
            p242.z = (float)3.3349873E38F;
            p242.q_SET(new float[] {-2.3799507E38F, 2.5415965E38F, 1.8848506E38F, 1.6373537E38F}, 0) ;
            p242.approach_z = (float)6.174118E37F;
            p242.y = (float) -1.1899756E38F;
            p242.altitude = (int)559937047;
            p242.x = (float)2.471904E38F;
            p242.approach_y = (float) -2.2876595E38F;
            p242.latitude = (int)398000162;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.longitude == (int) -1200816382);
                Debug.Assert(pack.approach_z == (float) -1.6216397E38F);
                Debug.Assert(pack.latitude == (int)424075495);
                Debug.Assert(pack.approach_y == (float)3.2813393E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)1974499679369810710L);
                Debug.Assert(pack.altitude == (int) -494827617);
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.3317131E38F, 6.2456765E37F, 2.0071718E38F, 4.1079883E37F}));
                Debug.Assert(pack.target_system == (byte)(byte)207);
                Debug.Assert(pack.y == (float) -2.734459E38F);
                Debug.Assert(pack.x == (float) -2.817503E38F);
                Debug.Assert(pack.approach_x == (float) -1.1451536E38F);
                Debug.Assert(pack.z == (float) -4.7797775E37F);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.time_usec_SET((ulong)1974499679369810710L, PH) ;
            p243.longitude = (int) -1200816382;
            p243.q_SET(new float[] {3.3317131E38F, 6.2456765E37F, 2.0071718E38F, 4.1079883E37F}, 0) ;
            p243.approach_x = (float) -1.1451536E38F;
            p243.latitude = (int)424075495;
            p243.target_system = (byte)(byte)207;
            p243.x = (float) -2.817503E38F;
            p243.y = (float) -2.734459E38F;
            p243.approach_y = (float)3.2813393E38F;
            p243.altitude = (int) -494827617;
            p243.approach_z = (float) -1.6216397E38F;
            p243.z = (float) -4.7797775E37F;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.interval_us == (int)1030953735);
                Debug.Assert(pack.message_id == (ushort)(ushort)11481);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)11481;
            p244.interval_us = (int)1030953735;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_MC;
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_type == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
                Debug.Assert(pack.altitude == (int) -2042097092);
                Debug.Assert(pack.ICAO_address == (uint)2416219054U);
                Debug.Assert(pack.lon == (int) -331860277);
                Debug.Assert(pack.flags == (ADSB_FLAGS.ADSB_FLAGS_SIMULATED |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY));
                Debug.Assert(pack.callsign_LEN(ph) == 5);
                Debug.Assert(pack.callsign_TRY(ph).Equals("exywl"));
                Debug.Assert(pack.ver_velocity == (short)(short)19570);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)1761);
                Debug.Assert(pack.squawk == (ushort)(ushort)35289);
                Debug.Assert(pack.emitter_type == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LARGE);
                Debug.Assert(pack.heading == (ushort)(ushort)26471);
                Debug.Assert(pack.tslc == (byte)(byte)42);
                Debug.Assert(pack.lat == (int)90460545);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.lat = (int)90460545;
            p246.altitude = (int) -2042097092;
            p246.tslc = (byte)(byte)42;
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LARGE;
            p246.ver_velocity = (short)(short)19570;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_SIMULATED |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_VELOCITY);
            p246.heading = (ushort)(ushort)26471;
            p246.squawk = (ushort)(ushort)35289;
            p246.lon = (int) -331860277;
            p246.ICAO_address = (uint)2416219054U;
            p246.hor_velocity = (ushort)(ushort)1761;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.callsign_SET("exywl", PH) ;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_to_minimum_delta == (float)2.0827449E38F);
                Debug.Assert(pack.altitude_minimum_delta == (float)3.2683324E38F);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW));
                Debug.Assert(pack.action == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR);
                Debug.Assert(pack.src_ == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
                Debug.Assert(pack.id == (uint)3987921644U);
                Debug.Assert(pack.horizontal_minimum_delta == (float)7.9122917E37F);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.id = (uint)3987921644U;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
            p247.horizontal_minimum_delta = (float)7.9122917E37F;
            p247.altitude_minimum_delta = (float)3.2683324E38F;
            p247.time_to_minimum_delta = (float)2.0827449E38F;
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)183, (byte)241, (byte)195, (byte)193, (byte)40, (byte)115, (byte)101, (byte)235, (byte)17, (byte)43, (byte)125, (byte)226, (byte)98, (byte)86, (byte)237, (byte)158, (byte)132, (byte)198, (byte)145, (byte)81, (byte)213, (byte)65, (byte)232, (byte)14, (byte)148, (byte)105, (byte)33, (byte)72, (byte)117, (byte)159, (byte)147, (byte)165, (byte)128, (byte)201, (byte)11, (byte)201, (byte)114, (byte)0, (byte)118, (byte)26, (byte)134, (byte)4, (byte)201, (byte)243, (byte)219, (byte)223, (byte)249, (byte)8, (byte)189, (byte)245, (byte)128, (byte)234, (byte)46, (byte)162, (byte)210, (byte)115, (byte)187, (byte)238, (byte)56, (byte)245, (byte)172, (byte)99, (byte)5, (byte)13, (byte)72, (byte)78, (byte)167, (byte)223, (byte)130, (byte)36, (byte)223, (byte)215, (byte)88, (byte)164, (byte)67, (byte)251, (byte)221, (byte)218, (byte)46, (byte)164, (byte)70, (byte)56, (byte)247, (byte)79, (byte)103, (byte)240, (byte)146, (byte)215, (byte)166, (byte)176, (byte)249, (byte)230, (byte)140, (byte)130, (byte)228, (byte)47, (byte)82, (byte)96, (byte)70, (byte)135, (byte)194, (byte)120, (byte)238, (byte)16, (byte)63, (byte)133, (byte)93, (byte)185, (byte)244, (byte)27, (byte)254, (byte)109, (byte)84, (byte)124, (byte)97, (byte)254, (byte)176, (byte)117, (byte)109, (byte)9, (byte)1, (byte)178, (byte)221, (byte)252, (byte)140, (byte)242, (byte)31, (byte)202, (byte)67, (byte)139, (byte)43, (byte)209, (byte)215, (byte)185, (byte)14, (byte)131, (byte)48, (byte)138, (byte)152, (byte)2, (byte)19, (byte)15, (byte)6, (byte)83, (byte)76, (byte)156, (byte)249, (byte)102, (byte)86, (byte)69, (byte)169, (byte)220, (byte)17, (byte)84, (byte)34, (byte)76, (byte)70, (byte)203, (byte)57, (byte)157, (byte)8, (byte)216, (byte)40, (byte)185, (byte)114, (byte)216, (byte)241, (byte)181, (byte)120, (byte)157, (byte)92, (byte)74, (byte)14, (byte)91, (byte)111, (byte)83, (byte)148, (byte)137, (byte)168, (byte)252, (byte)34, (byte)121, (byte)156, (byte)250, (byte)149, (byte)24, (byte)94, (byte)5, (byte)166, (byte)228, (byte)69, (byte)228, (byte)167, (byte)190, (byte)177, (byte)147, (byte)148, (byte)143, (byte)105, (byte)238, (byte)57, (byte)99, (byte)32, (byte)97, (byte)103, (byte)23, (byte)99, (byte)76, (byte)77, (byte)185, (byte)78, (byte)130, (byte)253, (byte)144, (byte)127, (byte)241, (byte)252, (byte)98, (byte)239, (byte)192, (byte)189, (byte)237, (byte)205, (byte)46, (byte)90, (byte)182, (byte)29, (byte)25, (byte)207, (byte)152, (byte)57, (byte)37, (byte)167, (byte)220, (byte)243, (byte)39, (byte)42, (byte)6, (byte)181, (byte)191, (byte)127, (byte)88, (byte)248, (byte)172, (byte)19, (byte)39, (byte)75, (byte)127, (byte)252}));
                Debug.Assert(pack.target_network == (byte)(byte)252);
                Debug.Assert(pack.target_component == (byte)(byte)97);
                Debug.Assert(pack.target_system == (byte)(byte)75);
                Debug.Assert(pack.message_type == (ushort)(ushort)56227);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_component = (byte)(byte)97;
            p248.message_type = (ushort)(ushort)56227;
            p248.target_network = (byte)(byte)252;
            p248.payload_SET(new byte[] {(byte)183, (byte)241, (byte)195, (byte)193, (byte)40, (byte)115, (byte)101, (byte)235, (byte)17, (byte)43, (byte)125, (byte)226, (byte)98, (byte)86, (byte)237, (byte)158, (byte)132, (byte)198, (byte)145, (byte)81, (byte)213, (byte)65, (byte)232, (byte)14, (byte)148, (byte)105, (byte)33, (byte)72, (byte)117, (byte)159, (byte)147, (byte)165, (byte)128, (byte)201, (byte)11, (byte)201, (byte)114, (byte)0, (byte)118, (byte)26, (byte)134, (byte)4, (byte)201, (byte)243, (byte)219, (byte)223, (byte)249, (byte)8, (byte)189, (byte)245, (byte)128, (byte)234, (byte)46, (byte)162, (byte)210, (byte)115, (byte)187, (byte)238, (byte)56, (byte)245, (byte)172, (byte)99, (byte)5, (byte)13, (byte)72, (byte)78, (byte)167, (byte)223, (byte)130, (byte)36, (byte)223, (byte)215, (byte)88, (byte)164, (byte)67, (byte)251, (byte)221, (byte)218, (byte)46, (byte)164, (byte)70, (byte)56, (byte)247, (byte)79, (byte)103, (byte)240, (byte)146, (byte)215, (byte)166, (byte)176, (byte)249, (byte)230, (byte)140, (byte)130, (byte)228, (byte)47, (byte)82, (byte)96, (byte)70, (byte)135, (byte)194, (byte)120, (byte)238, (byte)16, (byte)63, (byte)133, (byte)93, (byte)185, (byte)244, (byte)27, (byte)254, (byte)109, (byte)84, (byte)124, (byte)97, (byte)254, (byte)176, (byte)117, (byte)109, (byte)9, (byte)1, (byte)178, (byte)221, (byte)252, (byte)140, (byte)242, (byte)31, (byte)202, (byte)67, (byte)139, (byte)43, (byte)209, (byte)215, (byte)185, (byte)14, (byte)131, (byte)48, (byte)138, (byte)152, (byte)2, (byte)19, (byte)15, (byte)6, (byte)83, (byte)76, (byte)156, (byte)249, (byte)102, (byte)86, (byte)69, (byte)169, (byte)220, (byte)17, (byte)84, (byte)34, (byte)76, (byte)70, (byte)203, (byte)57, (byte)157, (byte)8, (byte)216, (byte)40, (byte)185, (byte)114, (byte)216, (byte)241, (byte)181, (byte)120, (byte)157, (byte)92, (byte)74, (byte)14, (byte)91, (byte)111, (byte)83, (byte)148, (byte)137, (byte)168, (byte)252, (byte)34, (byte)121, (byte)156, (byte)250, (byte)149, (byte)24, (byte)94, (byte)5, (byte)166, (byte)228, (byte)69, (byte)228, (byte)167, (byte)190, (byte)177, (byte)147, (byte)148, (byte)143, (byte)105, (byte)238, (byte)57, (byte)99, (byte)32, (byte)97, (byte)103, (byte)23, (byte)99, (byte)76, (byte)77, (byte)185, (byte)78, (byte)130, (byte)253, (byte)144, (byte)127, (byte)241, (byte)252, (byte)98, (byte)239, (byte)192, (byte)189, (byte)237, (byte)205, (byte)46, (byte)90, (byte)182, (byte)29, (byte)25, (byte)207, (byte)152, (byte)57, (byte)37, (byte)167, (byte)220, (byte)243, (byte)39, (byte)42, (byte)6, (byte)181, (byte)191, (byte)127, (byte)88, (byte)248, (byte)172, (byte)19, (byte)39, (byte)75, (byte)127, (byte)252}, 0) ;
            p248.target_system = (byte)(byte)75;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte) - 15, (sbyte) - 5, (sbyte)32, (sbyte)95, (sbyte) - 102, (sbyte)107, (sbyte) - 10, (sbyte) - 91, (sbyte)13, (sbyte) - 78, (sbyte)21, (sbyte)103, (sbyte)95, (sbyte)47, (sbyte)55, (sbyte) - 73, (sbyte)0, (sbyte) - 19, (sbyte) - 74, (sbyte) - 114, (sbyte) - 51, (sbyte) - 9, (sbyte)97, (sbyte) - 71, (sbyte) - 24, (sbyte)86, (sbyte) - 104, (sbyte) - 23, (sbyte)65, (sbyte)43, (sbyte) - 82, (sbyte) - 15}));
                Debug.Assert(pack.type == (byte)(byte)92);
                Debug.Assert(pack.ver == (byte)(byte)108);
                Debug.Assert(pack.address == (ushort)(ushort)35159);
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.ver = (byte)(byte)108;
            p249.value_SET(new sbyte[] {(sbyte) - 15, (sbyte) - 5, (sbyte)32, (sbyte)95, (sbyte) - 102, (sbyte)107, (sbyte) - 10, (sbyte) - 91, (sbyte)13, (sbyte) - 78, (sbyte)21, (sbyte)103, (sbyte)95, (sbyte)47, (sbyte)55, (sbyte) - 73, (sbyte)0, (sbyte) - 19, (sbyte) - 74, (sbyte) - 114, (sbyte) - 51, (sbyte) - 9, (sbyte)97, (sbyte) - 71, (sbyte) - 24, (sbyte)86, (sbyte) - 104, (sbyte) - 23, (sbyte)65, (sbyte)43, (sbyte) - 82, (sbyte) - 15}, 0) ;
            p249.type = (byte)(byte)92;
            p249.address = (ushort)(ushort)35159;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -1.2431195E38F);
                Debug.Assert(pack.y == (float) -2.9955393E38F);
                Debug.Assert(pack.z == (float)1.6084702E38F);
                Debug.Assert(pack.time_usec == (ulong)8589948525888355054L);
                Debug.Assert(pack.name_LEN(ph) == 8);
                Debug.Assert(pack.name_TRY(ph).Equals("bcmwuliq"));
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.time_usec = (ulong)8589948525888355054L;
            p250.y = (float) -2.9955393E38F;
            p250.z = (float)1.6084702E38F;
            p250.x = (float) -1.2431195E38F;
            p250.name_SET("bcmwuliq", PH) ;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2976349840U);
                Debug.Assert(pack.name_LEN(ph) == 4);
                Debug.Assert(pack.name_TRY(ph).Equals("hnub"));
                Debug.Assert(pack.value == (float)1.5388443E38F);
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)2976349840U;
            p251.value = (float)1.5388443E38F;
            p251.name_SET("hnub", PH) ;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 9);
                Debug.Assert(pack.name_TRY(ph).Equals("Mqcfudmjg"));
                Debug.Assert(pack.value == (int) -915512423);
                Debug.Assert(pack.time_boot_ms == (uint)403433609U);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.time_boot_ms = (uint)403433609U;
            p252.value = (int) -915512423;
            p252.name_SET("Mqcfudmjg", PH) ;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.severity == MAV_SEVERITY.MAV_SEVERITY_DEBUG);
                Debug.Assert(pack.text_LEN(ph) == 40);
                Debug.Assert(pack.text_TRY(ph).Equals("wnGlgaxeczuyvnicDcgfplywwCzuwqhqmseufaMv"));
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("wnGlgaxeczuyvnicDcgfplywwCzuwqhqmseufaMv", PH) ;
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_DEBUG;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ind == (byte)(byte)65);
                Debug.Assert(pack.value == (float) -3.144291E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3521979243U);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.ind = (byte)(byte)65;
            p254.time_boot_ms = (uint)3521979243U;
            p254.value = (float) -3.144291E38F;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)235, (byte)34, (byte)50, (byte)152, (byte)205, (byte)134, (byte)180, (byte)194, (byte)102, (byte)175, (byte)134, (byte)130, (byte)133, (byte)132, (byte)119, (byte)64, (byte)88, (byte)61, (byte)119, (byte)138, (byte)56, (byte)154, (byte)1, (byte)189, (byte)84, (byte)108, (byte)131, (byte)91, (byte)22, (byte)31, (byte)244, (byte)114}));
                Debug.Assert(pack.initial_timestamp == (ulong)8498111590539388821L);
                Debug.Assert(pack.target_system == (byte)(byte)144);
                Debug.Assert(pack.target_component == (byte)(byte)119);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.secret_key_SET(new byte[] {(byte)235, (byte)34, (byte)50, (byte)152, (byte)205, (byte)134, (byte)180, (byte)194, (byte)102, (byte)175, (byte)134, (byte)130, (byte)133, (byte)132, (byte)119, (byte)64, (byte)88, (byte)61, (byte)119, (byte)138, (byte)56, (byte)154, (byte)1, (byte)189, (byte)84, (byte)108, (byte)131, (byte)91, (byte)22, (byte)31, (byte)244, (byte)114}, 0) ;
            p256.initial_timestamp = (ulong)8498111590539388821L;
            p256.target_system = (byte)(byte)144;
            p256.target_component = (byte)(byte)119;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2102715637U);
                Debug.Assert(pack.last_change_ms == (uint)3449135937U);
                Debug.Assert(pack.state == (byte)(byte)89);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.last_change_ms = (uint)3449135937U;
            p257.state = (byte)(byte)89;
            p257.time_boot_ms = (uint)2102715637U;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)190);
                Debug.Assert(pack.tune_LEN(ph) == 12);
                Debug.Assert(pack.tune_TRY(ph).Equals("qzxtzfpowehm"));
                Debug.Assert(pack.target_system == (byte)(byte)72);
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.tune_SET("qzxtzfpowehm", PH) ;
            p258.target_system = (byte)(byte)72;
            p258.target_component = (byte)(byte)190;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_v == (ushort)(ushort)51731);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)21, (byte)127, (byte)250, (byte)29, (byte)98, (byte)36, (byte)171, (byte)70, (byte)211, (byte)24, (byte)250, (byte)92, (byte)206, (byte)26, (byte)31, (byte)84, (byte)137, (byte)126, (byte)214, (byte)243, (byte)191, (byte)59, (byte)143, (byte)114, (byte)4, (byte)126, (byte)223, (byte)205, (byte)112, (byte)184, (byte)75, (byte)168}));
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)27639);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE));
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 129);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("uhypijAlrDfsypcxuauyVYRjBzdyKadpcotzcrwaizhjSOaePoprdqtehhexzniYpotmlnPxtcaCsukpnFimrshrmryybqtChytfgzpjehbsiryvxjuzbophdvmqxlyYz"));
                Debug.Assert(pack.sensor_size_v == (float)3.5012994E37F);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)94, (byte)230, (byte)144, (byte)221, (byte)70, (byte)127, (byte)127, (byte)136, (byte)84, (byte)235, (byte)122, (byte)39, (byte)66, (byte)197, (byte)244, (byte)224, (byte)158, (byte)167, (byte)108, (byte)174, (byte)184, (byte)250, (byte)9, (byte)110, (byte)37, (byte)104, (byte)202, (byte)117, (byte)212, (byte)159, (byte)146, (byte)137}));
                Debug.Assert(pack.focal_length == (float)6.6778566E37F);
                Debug.Assert(pack.firmware_version == (uint)1506949337U);
                Debug.Assert(pack.time_boot_ms == (uint)2304012382U);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)8118);
                Debug.Assert(pack.lens_id == (byte)(byte)218);
                Debug.Assert(pack.sensor_size_h == (float)2.5629506E38F);
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.time_boot_ms = (uint)2304012382U;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE);
            p259.resolution_h = (ushort)(ushort)8118;
            p259.model_name_SET(new byte[] {(byte)21, (byte)127, (byte)250, (byte)29, (byte)98, (byte)36, (byte)171, (byte)70, (byte)211, (byte)24, (byte)250, (byte)92, (byte)206, (byte)26, (byte)31, (byte)84, (byte)137, (byte)126, (byte)214, (byte)243, (byte)191, (byte)59, (byte)143, (byte)114, (byte)4, (byte)126, (byte)223, (byte)205, (byte)112, (byte)184, (byte)75, (byte)168}, 0) ;
            p259.cam_definition_uri_SET("uhypijAlrDfsypcxuauyVYRjBzdyKadpcotzcrwaizhjSOaePoprdqtehhexzniYpotmlnPxtcaCsukpnFimrshrmryybqtChytfgzpjehbsiryvxjuzbophdvmqxlyYz", PH) ;
            p259.lens_id = (byte)(byte)218;
            p259.sensor_size_v = (float)3.5012994E37F;
            p259.cam_definition_version = (ushort)(ushort)27639;
            p259.resolution_v = (ushort)(ushort)51731;
            p259.firmware_version = (uint)1506949337U;
            p259.focal_length = (float)6.6778566E37F;
            p259.sensor_size_h = (float)2.5629506E38F;
            p259.vendor_name_SET(new byte[] {(byte)94, (byte)230, (byte)144, (byte)221, (byte)70, (byte)127, (byte)127, (byte)136, (byte)84, (byte)235, (byte)122, (byte)39, (byte)66, (byte)197, (byte)244, (byte)224, (byte)158, (byte)167, (byte)108, (byte)174, (byte)184, (byte)250, (byte)9, (byte)110, (byte)37, (byte)104, (byte)202, (byte)117, (byte)212, (byte)159, (byte)146, (byte)137}, 0) ;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3321825841U);
                Debug.Assert(pack.mode_id == (CAMERA_MODE.CAMERA_MODE_VIDEO |
                                              CAMERA_MODE.CAMERA_MODE_IMAGE));
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = (CAMERA_MODE.CAMERA_MODE_VIDEO |
                            CAMERA_MODE.CAMERA_MODE_IMAGE);
            p260.time_boot_ms = (uint)3321825841U;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)699827408U);
                Debug.Assert(pack.total_capacity == (float) -1.7697926E38F);
                Debug.Assert(pack.used_capacity == (float)3.2670875E38F);
                Debug.Assert(pack.storage_id == (byte)(byte)123);
                Debug.Assert(pack.status == (byte)(byte)255);
                Debug.Assert(pack.write_speed == (float)2.467847E38F);
                Debug.Assert(pack.available_capacity == (float) -1.1667542E38F);
                Debug.Assert(pack.read_speed == (float) -1.4269685E38F);
                Debug.Assert(pack.storage_count == (byte)(byte)66);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.available_capacity = (float) -1.1667542E38F;
            p261.used_capacity = (float)3.2670875E38F;
            p261.storage_id = (byte)(byte)123;
            p261.total_capacity = (float) -1.7697926E38F;
            p261.time_boot_ms = (uint)699827408U;
            p261.write_speed = (float)2.467847E38F;
            p261.storage_count = (byte)(byte)66;
            p261.status = (byte)(byte)255;
            p261.read_speed = (float) -1.4269685E38F;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.recording_time_ms == (uint)626103767U);
                Debug.Assert(pack.image_interval == (float)2.459711E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3615065441U);
                Debug.Assert(pack.image_status == (byte)(byte)26);
                Debug.Assert(pack.available_capacity == (float) -2.4055788E38F);
                Debug.Assert(pack.video_status == (byte)(byte)35);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.video_status = (byte)(byte)35;
            p262.image_status = (byte)(byte)26;
            p262.available_capacity = (float) -2.4055788E38F;
            p262.time_boot_ms = (uint)3615065441U;
            p262.image_interval = (float)2.459711E37F;
            p262.recording_time_ms = (uint)626103767U;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_utc == (ulong)1575404361258668675L);
                Debug.Assert(pack.image_index == (int) -1786119611);
                Debug.Assert(pack.file_url_LEN(ph) == 40);
                Debug.Assert(pack.file_url_TRY(ph).Equals("cfaqetrvieulhEMfRzomszdhgsyxBqxesfspigza"));
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.1912332E38F, -3.0246318E38F, -2.219849E38F, 1.289773E38F}));
                Debug.Assert(pack.time_boot_ms == (uint)3422660234U);
                Debug.Assert(pack.camera_id == (byte)(byte)243);
                Debug.Assert(pack.relative_alt == (int)1330167085);
                Debug.Assert(pack.alt == (int) -1999387652);
                Debug.Assert(pack.lat == (int)1879297149);
                Debug.Assert(pack.lon == (int) -1588415816);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)21);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.relative_alt = (int)1330167085;
            p263.image_index = (int) -1786119611;
            p263.time_utc = (ulong)1575404361258668675L;
            p263.file_url_SET("cfaqetrvieulhEMfRzomszdhgsyxBqxesfspigza", PH) ;
            p263.capture_result = (sbyte)(sbyte)21;
            p263.lon = (int) -1588415816;
            p263.time_boot_ms = (uint)3422660234U;
            p263.q_SET(new float[] {-2.1912332E38F, -3.0246318E38F, -2.219849E38F, 1.289773E38F}, 0) ;
            p263.camera_id = (byte)(byte)243;
            p263.alt = (int) -1999387652;
            p263.lat = (int)1879297149;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.arming_time_utc == (ulong)4499142006411062652L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)3957522449857927844L);
                Debug.Assert(pack.flight_uuid == (ulong)6527785463535890327L);
                Debug.Assert(pack.time_boot_ms == (uint)599784698U);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)599784698U;
            p264.flight_uuid = (ulong)6527785463535890327L;
            p264.arming_time_utc = (ulong)4499142006411062652L;
            p264.takeoff_time_utc = (ulong)3957522449857927844L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -3.0724472E38F);
                Debug.Assert(pack.pitch == (float)2.671108E37F);
                Debug.Assert(pack.time_boot_ms == (uint)2691198875U);
                Debug.Assert(pack.roll == (float)1.8743818E38F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.yaw = (float) -3.0724472E38F;
            p265.roll = (float)1.8743818E38F;
            p265.time_boot_ms = (uint)2691198875U;
            p265.pitch = (float)2.671108E37F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)44);
                Debug.Assert(pack.sequence == (ushort)(ushort)17649);
                Debug.Assert(pack.length == (byte)(byte)74);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)185, (byte)129, (byte)135, (byte)150, (byte)20, (byte)131, (byte)67, (byte)195, (byte)122, (byte)199, (byte)116, (byte)189, (byte)176, (byte)182, (byte)213, (byte)124, (byte)5, (byte)185, (byte)73, (byte)102, (byte)179, (byte)137, (byte)229, (byte)12, (byte)194, (byte)49, (byte)77, (byte)40, (byte)122, (byte)173, (byte)193, (byte)209, (byte)198, (byte)0, (byte)125, (byte)111, (byte)13, (byte)217, (byte)54, (byte)161, (byte)88, (byte)45, (byte)38, (byte)142, (byte)169, (byte)23, (byte)164, (byte)27, (byte)215, (byte)164, (byte)21, (byte)155, (byte)239, (byte)221, (byte)75, (byte)51, (byte)165, (byte)193, (byte)108, (byte)95, (byte)248, (byte)21, (byte)252, (byte)46, (byte)215, (byte)91, (byte)12, (byte)245, (byte)171, (byte)139, (byte)74, (byte)248, (byte)204, (byte)100, (byte)214, (byte)92, (byte)56, (byte)200, (byte)149, (byte)72, (byte)138, (byte)166, (byte)155, (byte)89, (byte)114, (byte)71, (byte)195, (byte)162, (byte)63, (byte)149, (byte)218, (byte)17, (byte)90, (byte)53, (byte)217, (byte)195, (byte)160, (byte)248, (byte)95, (byte)85, (byte)110, (byte)99, (byte)140, (byte)75, (byte)139, (byte)20, (byte)171, (byte)165, (byte)11, (byte)207, (byte)103, (byte)252, (byte)92, (byte)6, (byte)250, (byte)26, (byte)245, (byte)42, (byte)239, (byte)78, (byte)180, (byte)188, (byte)211, (byte)93, (byte)72, (byte)62, (byte)193, (byte)28, (byte)157, (byte)36, (byte)25, (byte)171, (byte)36, (byte)248, (byte)107, (byte)183, (byte)233, (byte)14, (byte)254, (byte)7, (byte)244, (byte)164, (byte)19, (byte)78, (byte)51, (byte)4, (byte)250, (byte)37, (byte)16, (byte)179, (byte)238, (byte)128, (byte)69, (byte)213, (byte)180, (byte)64, (byte)134, (byte)236, (byte)24, (byte)105, (byte)43, (byte)252, (byte)161, (byte)68, (byte)75, (byte)35, (byte)154, (byte)222, (byte)63, (byte)155, (byte)165, (byte)184, (byte)116, (byte)64, (byte)160, (byte)156, (byte)79, (byte)189, (byte)155, (byte)30, (byte)62, (byte)51, (byte)239, (byte)95, (byte)11, (byte)62, (byte)97, (byte)254, (byte)158, (byte)106, (byte)92, (byte)123, (byte)144, (byte)62, (byte)29, (byte)61, (byte)158, (byte)129, (byte)31, (byte)182, (byte)229, (byte)19, (byte)164, (byte)185, (byte)228, (byte)119, (byte)94, (byte)204, (byte)223, (byte)68, (byte)222, (byte)86, (byte)229, (byte)176, (byte)239, (byte)181, (byte)141, (byte)173, (byte)219, (byte)199, (byte)140, (byte)110, (byte)128, (byte)248, (byte)250, (byte)64, (byte)49, (byte)47, (byte)30, (byte)18, (byte)32, (byte)225, (byte)141, (byte)72, (byte)159, (byte)126, (byte)192, (byte)64, (byte)11, (byte)252, (byte)107, (byte)99, (byte)177, (byte)195, (byte)174, (byte)81, (byte)36, (byte)76, (byte)78}));
                Debug.Assert(pack.target_component == (byte)(byte)100);
                Debug.Assert(pack.first_message_offset == (byte)(byte)36);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.first_message_offset = (byte)(byte)36;
            p266.length = (byte)(byte)74;
            p266.target_system = (byte)(byte)44;
            p266.target_component = (byte)(byte)100;
            p266.data__SET(new byte[] {(byte)185, (byte)129, (byte)135, (byte)150, (byte)20, (byte)131, (byte)67, (byte)195, (byte)122, (byte)199, (byte)116, (byte)189, (byte)176, (byte)182, (byte)213, (byte)124, (byte)5, (byte)185, (byte)73, (byte)102, (byte)179, (byte)137, (byte)229, (byte)12, (byte)194, (byte)49, (byte)77, (byte)40, (byte)122, (byte)173, (byte)193, (byte)209, (byte)198, (byte)0, (byte)125, (byte)111, (byte)13, (byte)217, (byte)54, (byte)161, (byte)88, (byte)45, (byte)38, (byte)142, (byte)169, (byte)23, (byte)164, (byte)27, (byte)215, (byte)164, (byte)21, (byte)155, (byte)239, (byte)221, (byte)75, (byte)51, (byte)165, (byte)193, (byte)108, (byte)95, (byte)248, (byte)21, (byte)252, (byte)46, (byte)215, (byte)91, (byte)12, (byte)245, (byte)171, (byte)139, (byte)74, (byte)248, (byte)204, (byte)100, (byte)214, (byte)92, (byte)56, (byte)200, (byte)149, (byte)72, (byte)138, (byte)166, (byte)155, (byte)89, (byte)114, (byte)71, (byte)195, (byte)162, (byte)63, (byte)149, (byte)218, (byte)17, (byte)90, (byte)53, (byte)217, (byte)195, (byte)160, (byte)248, (byte)95, (byte)85, (byte)110, (byte)99, (byte)140, (byte)75, (byte)139, (byte)20, (byte)171, (byte)165, (byte)11, (byte)207, (byte)103, (byte)252, (byte)92, (byte)6, (byte)250, (byte)26, (byte)245, (byte)42, (byte)239, (byte)78, (byte)180, (byte)188, (byte)211, (byte)93, (byte)72, (byte)62, (byte)193, (byte)28, (byte)157, (byte)36, (byte)25, (byte)171, (byte)36, (byte)248, (byte)107, (byte)183, (byte)233, (byte)14, (byte)254, (byte)7, (byte)244, (byte)164, (byte)19, (byte)78, (byte)51, (byte)4, (byte)250, (byte)37, (byte)16, (byte)179, (byte)238, (byte)128, (byte)69, (byte)213, (byte)180, (byte)64, (byte)134, (byte)236, (byte)24, (byte)105, (byte)43, (byte)252, (byte)161, (byte)68, (byte)75, (byte)35, (byte)154, (byte)222, (byte)63, (byte)155, (byte)165, (byte)184, (byte)116, (byte)64, (byte)160, (byte)156, (byte)79, (byte)189, (byte)155, (byte)30, (byte)62, (byte)51, (byte)239, (byte)95, (byte)11, (byte)62, (byte)97, (byte)254, (byte)158, (byte)106, (byte)92, (byte)123, (byte)144, (byte)62, (byte)29, (byte)61, (byte)158, (byte)129, (byte)31, (byte)182, (byte)229, (byte)19, (byte)164, (byte)185, (byte)228, (byte)119, (byte)94, (byte)204, (byte)223, (byte)68, (byte)222, (byte)86, (byte)229, (byte)176, (byte)239, (byte)181, (byte)141, (byte)173, (byte)219, (byte)199, (byte)140, (byte)110, (byte)128, (byte)248, (byte)250, (byte)64, (byte)49, (byte)47, (byte)30, (byte)18, (byte)32, (byte)225, (byte)141, (byte)72, (byte)159, (byte)126, (byte)192, (byte)64, (byte)11, (byte)252, (byte)107, (byte)99, (byte)177, (byte)195, (byte)174, (byte)81, (byte)36, (byte)76, (byte)78}, 0) ;
            p266.sequence = (ushort)(ushort)17649;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.first_message_offset == (byte)(byte)194);
                Debug.Assert(pack.target_component == (byte)(byte)142);
                Debug.Assert(pack.length == (byte)(byte)197);
                Debug.Assert(pack.sequence == (ushort)(ushort)38299);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)182, (byte)219, (byte)169, (byte)208, (byte)241, (byte)158, (byte)155, (byte)236, (byte)118, (byte)44, (byte)9, (byte)125, (byte)230, (byte)150, (byte)167, (byte)91, (byte)180, (byte)37, (byte)120, (byte)51, (byte)217, (byte)74, (byte)237, (byte)98, (byte)134, (byte)249, (byte)2, (byte)86, (byte)253, (byte)137, (byte)128, (byte)22, (byte)195, (byte)162, (byte)5, (byte)174, (byte)11, (byte)225, (byte)203, (byte)22, (byte)202, (byte)137, (byte)212, (byte)222, (byte)214, (byte)118, (byte)228, (byte)3, (byte)2, (byte)125, (byte)130, (byte)105, (byte)81, (byte)212, (byte)209, (byte)86, (byte)235, (byte)63, (byte)146, (byte)177, (byte)188, (byte)169, (byte)219, (byte)227, (byte)18, (byte)249, (byte)212, (byte)39, (byte)8, (byte)18, (byte)163, (byte)225, (byte)172, (byte)9, (byte)52, (byte)63, (byte)137, (byte)136, (byte)10, (byte)166, (byte)78, (byte)58, (byte)199, (byte)42, (byte)180, (byte)22, (byte)235, (byte)224, (byte)67, (byte)232, (byte)211, (byte)32, (byte)240, (byte)166, (byte)128, (byte)77, (byte)200, (byte)139, (byte)178, (byte)186, (byte)216, (byte)57, (byte)79, (byte)110, (byte)174, (byte)212, (byte)35, (byte)77, (byte)255, (byte)137, (byte)167, (byte)135, (byte)246, (byte)37, (byte)4, (byte)17, (byte)22, (byte)239, (byte)224, (byte)103, (byte)217, (byte)3, (byte)102, (byte)231, (byte)226, (byte)124, (byte)184, (byte)230, (byte)99, (byte)231, (byte)182, (byte)228, (byte)102, (byte)245, (byte)159, (byte)110, (byte)236, (byte)210, (byte)196, (byte)149, (byte)149, (byte)157, (byte)196, (byte)38, (byte)194, (byte)235, (byte)1, (byte)90, (byte)126, (byte)184, (byte)135, (byte)128, (byte)173, (byte)2, (byte)141, (byte)114, (byte)15, (byte)232, (byte)6, (byte)190, (byte)6, (byte)210, (byte)155, (byte)169, (byte)244, (byte)39, (byte)137, (byte)85, (byte)41, (byte)152, (byte)199, (byte)79, (byte)15, (byte)8, (byte)142, (byte)193, (byte)239, (byte)42, (byte)203, (byte)253, (byte)25, (byte)169, (byte)92, (byte)129, (byte)139, (byte)142, (byte)59, (byte)148, (byte)174, (byte)80, (byte)249, (byte)227, (byte)166, (byte)236, (byte)13, (byte)73, (byte)93, (byte)177, (byte)227, (byte)122, (byte)42, (byte)254, (byte)202, (byte)135, (byte)209, (byte)94, (byte)168, (byte)2, (byte)143, (byte)87, (byte)234, (byte)181, (byte)77, (byte)5, (byte)30, (byte)146, (byte)8, (byte)181, (byte)89, (byte)96, (byte)67, (byte)72, (byte)128, (byte)139, (byte)203, (byte)160, (byte)36, (byte)186, (byte)54, (byte)221, (byte)95, (byte)211, (byte)118, (byte)77, (byte)100, (byte)38, (byte)163, (byte)202, (byte)119, (byte)143, (byte)157, (byte)91, (byte)18, (byte)149, (byte)155, (byte)99, (byte)9, (byte)237, (byte)141}));
                Debug.Assert(pack.target_system == (byte)(byte)206);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_component = (byte)(byte)142;
            p267.first_message_offset = (byte)(byte)194;
            p267.data__SET(new byte[] {(byte)182, (byte)219, (byte)169, (byte)208, (byte)241, (byte)158, (byte)155, (byte)236, (byte)118, (byte)44, (byte)9, (byte)125, (byte)230, (byte)150, (byte)167, (byte)91, (byte)180, (byte)37, (byte)120, (byte)51, (byte)217, (byte)74, (byte)237, (byte)98, (byte)134, (byte)249, (byte)2, (byte)86, (byte)253, (byte)137, (byte)128, (byte)22, (byte)195, (byte)162, (byte)5, (byte)174, (byte)11, (byte)225, (byte)203, (byte)22, (byte)202, (byte)137, (byte)212, (byte)222, (byte)214, (byte)118, (byte)228, (byte)3, (byte)2, (byte)125, (byte)130, (byte)105, (byte)81, (byte)212, (byte)209, (byte)86, (byte)235, (byte)63, (byte)146, (byte)177, (byte)188, (byte)169, (byte)219, (byte)227, (byte)18, (byte)249, (byte)212, (byte)39, (byte)8, (byte)18, (byte)163, (byte)225, (byte)172, (byte)9, (byte)52, (byte)63, (byte)137, (byte)136, (byte)10, (byte)166, (byte)78, (byte)58, (byte)199, (byte)42, (byte)180, (byte)22, (byte)235, (byte)224, (byte)67, (byte)232, (byte)211, (byte)32, (byte)240, (byte)166, (byte)128, (byte)77, (byte)200, (byte)139, (byte)178, (byte)186, (byte)216, (byte)57, (byte)79, (byte)110, (byte)174, (byte)212, (byte)35, (byte)77, (byte)255, (byte)137, (byte)167, (byte)135, (byte)246, (byte)37, (byte)4, (byte)17, (byte)22, (byte)239, (byte)224, (byte)103, (byte)217, (byte)3, (byte)102, (byte)231, (byte)226, (byte)124, (byte)184, (byte)230, (byte)99, (byte)231, (byte)182, (byte)228, (byte)102, (byte)245, (byte)159, (byte)110, (byte)236, (byte)210, (byte)196, (byte)149, (byte)149, (byte)157, (byte)196, (byte)38, (byte)194, (byte)235, (byte)1, (byte)90, (byte)126, (byte)184, (byte)135, (byte)128, (byte)173, (byte)2, (byte)141, (byte)114, (byte)15, (byte)232, (byte)6, (byte)190, (byte)6, (byte)210, (byte)155, (byte)169, (byte)244, (byte)39, (byte)137, (byte)85, (byte)41, (byte)152, (byte)199, (byte)79, (byte)15, (byte)8, (byte)142, (byte)193, (byte)239, (byte)42, (byte)203, (byte)253, (byte)25, (byte)169, (byte)92, (byte)129, (byte)139, (byte)142, (byte)59, (byte)148, (byte)174, (byte)80, (byte)249, (byte)227, (byte)166, (byte)236, (byte)13, (byte)73, (byte)93, (byte)177, (byte)227, (byte)122, (byte)42, (byte)254, (byte)202, (byte)135, (byte)209, (byte)94, (byte)168, (byte)2, (byte)143, (byte)87, (byte)234, (byte)181, (byte)77, (byte)5, (byte)30, (byte)146, (byte)8, (byte)181, (byte)89, (byte)96, (byte)67, (byte)72, (byte)128, (byte)139, (byte)203, (byte)160, (byte)36, (byte)186, (byte)54, (byte)221, (byte)95, (byte)211, (byte)118, (byte)77, (byte)100, (byte)38, (byte)163, (byte)202, (byte)119, (byte)143, (byte)157, (byte)91, (byte)18, (byte)149, (byte)155, (byte)99, (byte)9, (byte)237, (byte)141}, 0) ;
            p267.target_system = (byte)(byte)206;
            p267.length = (byte)(byte)197;
            p267.sequence = (ushort)(ushort)38299;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)145);
                Debug.Assert(pack.sequence == (ushort)(ushort)15433);
                Debug.Assert(pack.target_system == (byte)(byte)205);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)205;
            p268.sequence = (ushort)(ushort)15433;
            p268.target_component = (byte)(byte)145;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.camera_id == (byte)(byte)20);
                Debug.Assert(pack.framerate == (float)2.9830855E38F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)48940);
                Debug.Assert(pack.bitrate == (uint)100050149U);
                Debug.Assert(pack.status == (byte)(byte)151);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)5405);
                Debug.Assert(pack.uri_LEN(ph) == 174);
                Debug.Assert(pack.uri_TRY(ph).Equals("gndoxsldomzijiCyaOnqejmioqhqGXZhnaOgbxcrtpRrcWpvVdcfwpmlcwjlmuabhpcmibctdetoSzqchracuhwqagsyjocyfqhbnitfjxnzsuvqvxnkjosrwrvNzbswOuwehtdcogZDtvztzMzuvxobrsflmlomeoexvacvzencnz"));
                Debug.Assert(pack.rotation == (ushort)(ushort)17433);
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.uri_SET("gndoxsldomzijiCyaOnqejmioqhqGXZhnaOgbxcrtpRrcWpvVdcfwpmlcwjlmuabhpcmibctdetoSzqchracuhwqagsyjocyfqhbnitfjxnzsuvqvxnkjosrwrvNzbswOuwehtdcogZDtvztzMzuvxobrsflmlomeoexvacvzencnz", PH) ;
            p269.camera_id = (byte)(byte)20;
            p269.framerate = (float)2.9830855E38F;
            p269.resolution_h = (ushort)(ushort)48940;
            p269.rotation = (ushort)(ushort)17433;
            p269.resolution_v = (ushort)(ushort)5405;
            p269.status = (byte)(byte)151;
            p269.bitrate = (uint)100050149U;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.camera_id == (byte)(byte)67);
                Debug.Assert(pack.rotation == (ushort)(ushort)51716);
                Debug.Assert(pack.framerate == (float) -8.14347E37F);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)22466);
                Debug.Assert(pack.target_system == (byte)(byte)190);
                Debug.Assert(pack.target_component == (byte)(byte)153);
                Debug.Assert(pack.uri_LEN(ph) == 65);
                Debug.Assert(pack.uri_TRY(ph).Equals("oFuugFpefugamnurzadtauqcghbjZumgfvwucmyqwofgwkeKteKzwbzasZugyrayx"));
                Debug.Assert(pack.bitrate == (uint)1090035478U);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)17534);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.resolution_h = (ushort)(ushort)17534;
            p270.target_system = (byte)(byte)190;
            p270.uri_SET("oFuugFpefugamnurzadtauqcghbjZumgfvwucmyqwofgwkeKteKzwbzasZugyrayx", PH) ;
            p270.rotation = (ushort)(ushort)51716;
            p270.target_component = (byte)(byte)153;
            p270.bitrate = (uint)1090035478U;
            p270.resolution_v = (ushort)(ushort)22466;
            p270.camera_id = (byte)(byte)67;
            p270.framerate = (float) -8.14347E37F;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 53);
                Debug.Assert(pack.password_TRY(ph).Equals("ymyrothstnShpakmpcvgftlbFHkuRlkvpafhxWhuinVwqPnWitplq"));
                Debug.Assert(pack.ssid_LEN(ph) == 25);
                Debug.Assert(pack.ssid_TRY(ph).Equals("odkuQakNvkhqxlXwkixnzljkd"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("odkuQakNvkhqxlXwkixnzljkd", PH) ;
            p299.password_SET("ymyrothstnShpakmpcvgftlbFHkuRlkvpafhxWhuinVwqPnWitplq", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_version == (ushort)(ushort)10726);
                Debug.Assert(pack.version == (ushort)(ushort)1274);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)19, (byte)249, (byte)60, (byte)5, (byte)27, (byte)3, (byte)131, (byte)94}));
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)233, (byte)103, (byte)134, (byte)166, (byte)4, (byte)84, (byte)194, (byte)105}));
                Debug.Assert(pack.max_version == (ushort)(ushort)22038);
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.library_version_hash_SET(new byte[] {(byte)19, (byte)249, (byte)60, (byte)5, (byte)27, (byte)3, (byte)131, (byte)94}, 0) ;
            p300.version = (ushort)(ushort)1274;
            p300.spec_version_hash_SET(new byte[] {(byte)233, (byte)103, (byte)134, (byte)166, (byte)4, (byte)84, (byte)194, (byte)105}, 0) ;
            p300.min_version = (ushort)(ushort)10726;
            p300.max_version = (ushort)(ushort)22038;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sub_mode == (byte)(byte)71);
                Debug.Assert(pack.mode == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
                Debug.Assert(pack.time_usec == (ulong)1706398778284680117L);
                Debug.Assert(pack.health == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)29166);
                Debug.Assert(pack.uptime_sec == (uint)3344576107U);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING;
            p310.sub_mode = (byte)(byte)71;
            p310.time_usec = (ulong)1706398778284680117L;
            p310.uptime_sec = (uint)3344576107U;
            p310.vendor_specific_status_code = (ushort)(ushort)29166;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)53, (byte)246, (byte)79, (byte)45, (byte)255, (byte)108, (byte)247, (byte)201, (byte)225, (byte)110, (byte)168, (byte)62, (byte)202, (byte)203, (byte)141, (byte)246}));
                Debug.Assert(pack.sw_vcs_commit == (uint)1182458079U);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)65);
                Debug.Assert(pack.hw_version_major == (byte)(byte)19);
                Debug.Assert(pack.sw_version_major == (byte)(byte)101);
                Debug.Assert(pack.uptime_sec == (uint)1493604864U);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)213);
                Debug.Assert(pack.name_LEN(ph) == 4);
                Debug.Assert(pack.name_TRY(ph).Equals("nbgh"));
                Debug.Assert(pack.time_usec == (ulong)5556535534726061542L);
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.hw_version_major = (byte)(byte)19;
            p311.hw_version_minor = (byte)(byte)213;
            p311.name_SET("nbgh", PH) ;
            p311.sw_version_major = (byte)(byte)101;
            p311.time_usec = (ulong)5556535534726061542L;
            p311.hw_unique_id_SET(new byte[] {(byte)53, (byte)246, (byte)79, (byte)45, (byte)255, (byte)108, (byte)247, (byte)201, (byte)225, (byte)110, (byte)168, (byte)62, (byte)202, (byte)203, (byte)141, (byte)246}, 0) ;
            p311.sw_vcs_commit = (uint)1182458079U;
            p311.sw_version_minor = (byte)(byte)65;
            p311.uptime_sec = (uint)1493604864U;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (short)(short)23695);
                Debug.Assert(pack.target_system == (byte)(byte)158);
                Debug.Assert(pack.target_component == (byte)(byte)82);
                Debug.Assert(pack.param_id_LEN(ph) == 6);
                Debug.Assert(pack.param_id_TRY(ph).Equals("AdwIxq"));
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_component = (byte)(byte)82;
            p320.param_index = (short)(short)23695;
            p320.target_system = (byte)(byte)158;
            p320.param_id_SET("AdwIxq", PH) ;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)34);
                Debug.Assert(pack.target_component == (byte)(byte)196);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)34;
            p321.target_component = (byte)(byte)196;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
                Debug.Assert(pack.param_id_LEN(ph) == 10);
                Debug.Assert(pack.param_id_TRY(ph).Equals("Fphzntpmvf"));
                Debug.Assert(pack.param_count == (ushort)(ushort)5752);
                Debug.Assert(pack.param_index == (ushort)(ushort)27835);
                Debug.Assert(pack.param_value_LEN(ph) == 52);
                Debug.Assert(pack.param_value_TRY(ph).Equals("xmoghphifjFdrgBrwosadqchaajpvcudtiepmymHfvmkbabuglja"));
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_count = (ushort)(ushort)5752;
            p322.param_value_SET("xmoghphifjFdrgBrwosadqchaajpvcudtiepmymHfvmkbabuglja", PH) ;
            p322.param_index = (ushort)(ushort)27835;
            p322.param_id_SET("Fphzntpmvf", PH) ;
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)83);
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
                Debug.Assert(pack.param_value_LEN(ph) == 88);
                Debug.Assert(pack.param_value_TRY(ph).Equals("roifvKdyeuxemagdCwszutvztBekcecqjrgCurCnxjiswinypvrmwvbcuwfqcdcfayhyjehhgzroTtvvgpxxstsg"));
                Debug.Assert(pack.target_system == (byte)(byte)53);
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("G"));
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_id_SET("G", PH) ;
            p323.param_value_SET("roifvKdyeuxemagdCwszutvztBekcecqjrgCurCnxjiswinypvrmwvbcuwfqcdcfayhyjehhgzroTtvvgpxxstsg", PH) ;
            p323.target_system = (byte)(byte)53;
            p323.target_component = (byte)(byte)83;
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_result == PARAM_ACK.PARAM_ACK_FAILED);
                Debug.Assert(pack.param_id_LEN(ph) == 11);
                Debug.Assert(pack.param_id_TRY(ph).Equals("nudhbHusvXu"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
                Debug.Assert(pack.param_value_LEN(ph) == 82);
                Debug.Assert(pack.param_value_TRY(ph).Equals("jyexloYtqvdqecqwlwjWcffsghxymsIyaHztoHtdiAmkhqYkKlwzkoudccjwydcidbhbhxjsmNsftrdrxs"));
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_result = PARAM_ACK.PARAM_ACK_FAILED;
            p324.param_value_SET("jyexloYtqvdqecqwlwjWcffsghxymsIyaHztoHtdiAmkhqYkKlwzkoudccjwydcidbhbhxjsmNsftrdrxs", PH) ;
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16;
            p324.param_id_SET("nudhbHusvXu", PH) ;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.max_distance == (ushort)(ushort)18800);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)30488, (ushort)23209, (ushort)63041, (ushort)49133, (ushort)50904, (ushort)2336, (ushort)44436, (ushort)13348, (ushort)59022, (ushort)53207, (ushort)24747, (ushort)13940, (ushort)64234, (ushort)29230, (ushort)4178, (ushort)38416, (ushort)20843, (ushort)51759, (ushort)36527, (ushort)42937, (ushort)15041, (ushort)11954, (ushort)3487, (ushort)40573, (ushort)21360, (ushort)53387, (ushort)63650, (ushort)46124, (ushort)44058, (ushort)36444, (ushort)485, (ushort)1653, (ushort)21363, (ushort)31620, (ushort)6538, (ushort)52742, (ushort)9947, (ushort)23252, (ushort)27000, (ushort)12928, (ushort)6227, (ushort)16782, (ushort)16271, (ushort)28683, (ushort)55719, (ushort)56105, (ushort)36985, (ushort)17255, (ushort)13330, (ushort)43632, (ushort)39120, (ushort)55402, (ushort)14495, (ushort)20515, (ushort)30890, (ushort)16447, (ushort)13024, (ushort)77, (ushort)31920, (ushort)44335, (ushort)2039, (ushort)16592, (ushort)26649, (ushort)39061, (ushort)8269, (ushort)31821, (ushort)62924, (ushort)27991, (ushort)34743, (ushort)31390, (ushort)25200, (ushort)15224}));
                Debug.Assert(pack.increment == (byte)(byte)190);
                Debug.Assert(pack.sensor_type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
                Debug.Assert(pack.time_usec == (ulong)6846905157265759234L);
                Debug.Assert(pack.min_distance == (ushort)(ushort)19898);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.distances_SET(new ushort[] {(ushort)30488, (ushort)23209, (ushort)63041, (ushort)49133, (ushort)50904, (ushort)2336, (ushort)44436, (ushort)13348, (ushort)59022, (ushort)53207, (ushort)24747, (ushort)13940, (ushort)64234, (ushort)29230, (ushort)4178, (ushort)38416, (ushort)20843, (ushort)51759, (ushort)36527, (ushort)42937, (ushort)15041, (ushort)11954, (ushort)3487, (ushort)40573, (ushort)21360, (ushort)53387, (ushort)63650, (ushort)46124, (ushort)44058, (ushort)36444, (ushort)485, (ushort)1653, (ushort)21363, (ushort)31620, (ushort)6538, (ushort)52742, (ushort)9947, (ushort)23252, (ushort)27000, (ushort)12928, (ushort)6227, (ushort)16782, (ushort)16271, (ushort)28683, (ushort)55719, (ushort)56105, (ushort)36985, (ushort)17255, (ushort)13330, (ushort)43632, (ushort)39120, (ushort)55402, (ushort)14495, (ushort)20515, (ushort)30890, (ushort)16447, (ushort)13024, (ushort)77, (ushort)31920, (ushort)44335, (ushort)2039, (ushort)16592, (ushort)26649, (ushort)39061, (ushort)8269, (ushort)31821, (ushort)62924, (ushort)27991, (ushort)34743, (ushort)31390, (ushort)25200, (ushort)15224}, 0) ;
            p330.max_distance = (ushort)(ushort)18800;
            p330.min_distance = (ushort)(ushort)19898;
            p330.increment = (byte)(byte)190;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            p330.time_usec = (ulong)6846905157265759234L;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
        }
    }
}