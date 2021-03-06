
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
                    ulong id = id__b(value);
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
                    ulong id = id__t(value);
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
                    ulong id = id__t(value);
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
            *	 this field to difference between instances*/
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
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED));
                Debug.Assert(pack.type == MAV_TYPE.MAV_TYPE_GCS);
                Debug.Assert(pack.system_status == MAV_STATE.MAV_STATE_POWEROFF);
                Debug.Assert(pack.autopilot == MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY);
                Debug.Assert(pack.mavlink_version == (byte)(byte)148);
                Debug.Assert(pack.custom_mode == (uint)3502864888U);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY;
            p0.mavlink_version = (byte)(byte)148;
            p0.type = MAV_TYPE.MAV_TYPE_GCS;
            p0.custom_mode = (uint)3502864888U;
            p0.system_status = MAV_STATE.MAV_STATE_POWEROFF;
            p0.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED);
            ADV_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.load == (ushort)(ushort)32026);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)60932);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH));
                Debug.Assert(pack.current_battery == (short)(short) -19870);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)18766);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)54027);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)18092);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)31);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)41669);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY));
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)26857);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)37990);
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.errors_count2 = (ushort)(ushort)18092;
            p1.errors_count3 = (ushort)(ushort)54027;
            p1.battery_remaining = (sbyte)(sbyte)31;
            p1.drop_rate_comm = (ushort)(ushort)26857;
            p1.load = (ushort)(ushort)32026;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_GPS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH);
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY);
            p1.voltage_battery = (ushort)(ushort)37990;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.current_battery = (short)(short) -19870;
            p1.errors_count4 = (ushort)(ushort)41669;
            p1.errors_count1 = (ushort)(ushort)18766;
            p1.errors_comm = (ushort)(ushort)60932;
            ADV_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)3373506472791979429L);
                Debug.Assert(pack.time_boot_ms == (uint)1195657553U);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_boot_ms = (uint)1195657553U;
            p2.time_unix_usec = (ulong)3373506472791979429L;
            ADV_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2377011975U);
                Debug.Assert(pack.afy == (float) -9.003748E37F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.y == (float)2.1263062E38F);
                Debug.Assert(pack.afx == (float)1.1286939E37F);
                Debug.Assert(pack.z == (float) -1.2216614E37F);
                Debug.Assert(pack.vz == (float)1.6038515E38F);
                Debug.Assert(pack.vy == (float)1.9478316E38F);
                Debug.Assert(pack.afz == (float)1.3806108E38F);
                Debug.Assert(pack.vx == (float) -2.7370834E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)22802);
                Debug.Assert(pack.yaw == (float)1.3059528E38F);
                Debug.Assert(pack.yaw_rate == (float)2.188346E38F);
                Debug.Assert(pack.x == (float)3.1557123E38F);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.afy = (float) -9.003748E37F;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p3.afx = (float)1.1286939E37F;
            p3.yaw_rate = (float)2.188346E38F;
            p3.x = (float)3.1557123E38F;
            p3.z = (float) -1.2216614E37F;
            p3.type_mask = (ushort)(ushort)22802;
            p3.y = (float)2.1263062E38F;
            p3.vy = (float)1.9478316E38F;
            p3.vz = (float)1.6038515E38F;
            p3.yaw = (float)1.3059528E38F;
            p3.time_boot_ms = (uint)2377011975U;
            p3.vx = (float) -2.7370834E38F;
            p3.afz = (float)1.3806108E38F;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)2668680487587587266L);
                Debug.Assert(pack.target_component == (byte)(byte)203);
                Debug.Assert(pack.seq == (uint)2890898654U);
                Debug.Assert(pack.target_system == (byte)(byte)196);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.time_usec = (ulong)2668680487587587266L;
            p4.target_component = (byte)(byte)203;
            p4.target_system = (byte)(byte)196;
            p4.seq = (uint)2890898654U;
            ADV_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.control_request == (byte)(byte)169);
                Debug.Assert(pack.passkey_LEN(ph) == 9);
                Debug.Assert(pack.passkey_TRY(ph).Equals("rlzyfexbi"));
                Debug.Assert(pack.target_system == (byte)(byte)204);
                Debug.Assert(pack.version == (byte)(byte)213);
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.control_request = (byte)(byte)169;
            p5.version = (byte)(byte)213;
            p5.target_system = (byte)(byte)204;
            p5.passkey_SET("rlzyfexbi", PH) ;
            ADV_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.control_request == (byte)(byte)87);
                Debug.Assert(pack.ack == (byte)(byte)138);
                Debug.Assert(pack.gcs_system_id == (byte)(byte)20);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.control_request = (byte)(byte)87;
            p6.gcs_system_id = (byte)(byte)20;
            p6.ack = (byte)(byte)138;
            ADV_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 8);
                Debug.Assert(pack.key_TRY(ph).Equals("lxhkpizt"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("lxhkpizt", PH) ;
            ADV_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)234);
                Debug.Assert(pack.base_mode == MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
                Debug.Assert(pack.custom_mode == (uint)3965958830U);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.custom_mode = (uint)3965958830U;
            p11.target_system = (byte)(byte)234;
            p11.base_mode = MAV_MODE.MAV_MODE_STABILIZE_DISARMED;
            ADV_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (short)(short)28615);
                Debug.Assert(pack.target_component == (byte)(byte)36);
                Debug.Assert(pack.param_id_LEN(ph) == 13);
                Debug.Assert(pack.param_id_TRY(ph).Equals("voajnnhMlldoi"));
                Debug.Assert(pack.target_system == (byte)(byte)228);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_system = (byte)(byte)228;
            p20.param_id_SET("voajnnhMlldoi", PH) ;
            p20.target_component = (byte)(byte)36;
            p20.param_index = (short)(short)28615;
            ADV_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)165);
                Debug.Assert(pack.target_component == (byte)(byte)42);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_component = (byte)(byte)42;
            p21.target_system = (byte)(byte)165;
            ADV_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_count == (ushort)(ushort)17104);
                Debug.Assert(pack.param_id_LEN(ph) == 14);
                Debug.Assert(pack.param_id_TRY(ph).Equals("hnbkgczeyvUelL"));
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
                Debug.Assert(pack.param_value == (float) -1.3780649E38F);
                Debug.Assert(pack.param_index == (ushort)(ushort)40925);
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32;
            p22.param_id_SET("hnbkgczeyvUelL", PH) ;
            p22.param_value = (float) -1.3780649E38F;
            p22.param_index = (ushort)(ushort)40925;
            p22.param_count = (ushort)(ushort)17104;
            ADV_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value == (float)3.3901906E38F);
                Debug.Assert(pack.target_system == (byte)(byte)246);
                Debug.Assert(pack.target_component == (byte)(byte)218);
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("JjNfVnkx"));
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32);
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32;
            p23.param_id_SET("JjNfVnkx", PH) ;
            p23.param_value = (float)3.3901906E38F;
            p23.target_component = (byte)(byte)218;
            p23.target_system = (byte)(byte)246;
            ADV_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
                Debug.Assert(pack.satellites_visible == (byte)(byte)251);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)1288705829U);
                Debug.Assert(pack.lon == (int)893608302);
                Debug.Assert(pack.alt == (int)1241085978);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int)1901453266);
                Debug.Assert(pack.time_usec == (ulong)3448729978741802366L);
                Debug.Assert(pack.lat == (int)256456919);
                Debug.Assert(pack.vel == (ushort)(ushort)38406);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)1451212309U);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)268067259U);
                Debug.Assert(pack.epv == (ushort)(ushort)26833);
                Debug.Assert(pack.eph == (ushort)(ushort)17122);
                Debug.Assert(pack.cog == (ushort)(ushort)22911);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)20900425U);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.lat = (int)256456919;
            p24.lon = (int)893608302;
            p24.alt_ellipsoid_SET((int)1901453266, PH) ;
            p24.alt = (int)1241085978;
            p24.eph = (ushort)(ushort)17122;
            p24.vel_acc_SET((uint)20900425U, PH) ;
            p24.h_acc_SET((uint)268067259U, PH) ;
            p24.hdg_acc_SET((uint)1288705829U, PH) ;
            p24.satellites_visible = (byte)(byte)251;
            p24.v_acc_SET((uint)1451212309U, PH) ;
            p24.time_usec = (ulong)3448729978741802366L;
            p24.vel = (ushort)(ushort)38406;
            p24.epv = (ushort)(ushort)26833;
            p24.cog = (ushort)(ushort)22911;
            p24.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS;
            ADV_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)172, (byte)150, (byte)136, (byte)135, (byte)248, (byte)71, (byte)19, (byte)129, (byte)248, (byte)168, (byte)184, (byte)195, (byte)222, (byte)215, (byte)209, (byte)211, (byte)55, (byte)222, (byte)214, (byte)243}));
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)156, (byte)161, (byte)35, (byte)9, (byte)72, (byte)29, (byte)194, (byte)4, (byte)168, (byte)6, (byte)151, (byte)127, (byte)126, (byte)127, (byte)122, (byte)238, (byte)187, (byte)39, (byte)182, (byte)91}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)187);
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)181, (byte)137, (byte)22, (byte)152, (byte)112, (byte)213, (byte)71, (byte)116, (byte)46, (byte)65, (byte)97, (byte)237, (byte)154, (byte)203, (byte)86, (byte)47, (byte)9, (byte)62, (byte)133, (byte)47}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)215, (byte)139, (byte)125, (byte)54, (byte)71, (byte)225, (byte)4, (byte)47, (byte)111, (byte)23, (byte)176, (byte)36, (byte)114, (byte)91, (byte)176, (byte)46, (byte)203, (byte)188, (byte)23, (byte)59}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)75, (byte)143, (byte)53, (byte)61, (byte)251, (byte)230, (byte)176, (byte)86, (byte)190, (byte)129, (byte)79, (byte)77, (byte)164, (byte)52, (byte)115, (byte)172, (byte)40, (byte)134, (byte)76, (byte)122}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_prn_SET(new byte[] {(byte)215, (byte)139, (byte)125, (byte)54, (byte)71, (byte)225, (byte)4, (byte)47, (byte)111, (byte)23, (byte)176, (byte)36, (byte)114, (byte)91, (byte)176, (byte)46, (byte)203, (byte)188, (byte)23, (byte)59}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)75, (byte)143, (byte)53, (byte)61, (byte)251, (byte)230, (byte)176, (byte)86, (byte)190, (byte)129, (byte)79, (byte)77, (byte)164, (byte)52, (byte)115, (byte)172, (byte)40, (byte)134, (byte)76, (byte)122}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)172, (byte)150, (byte)136, (byte)135, (byte)248, (byte)71, (byte)19, (byte)129, (byte)248, (byte)168, (byte)184, (byte)195, (byte)222, (byte)215, (byte)209, (byte)211, (byte)55, (byte)222, (byte)214, (byte)243}, 0) ;
            p25.satellites_visible = (byte)(byte)187;
            p25.satellite_used_SET(new byte[] {(byte)181, (byte)137, (byte)22, (byte)152, (byte)112, (byte)213, (byte)71, (byte)116, (byte)46, (byte)65, (byte)97, (byte)237, (byte)154, (byte)203, (byte)86, (byte)47, (byte)9, (byte)62, (byte)133, (byte)47}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)156, (byte)161, (byte)35, (byte)9, (byte)72, (byte)29, (byte)194, (byte)4, (byte)168, (byte)6, (byte)151, (byte)127, (byte)126, (byte)127, (byte)122, (byte)238, (byte)187, (byte)39, (byte)182, (byte)91}, 0) ;
            ADV_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xacc == (short)(short)2558);
                Debug.Assert(pack.zacc == (short)(short) -15592);
                Debug.Assert(pack.time_boot_ms == (uint)3608876360U);
                Debug.Assert(pack.ymag == (short)(short) -18785);
                Debug.Assert(pack.yacc == (short)(short)17958);
                Debug.Assert(pack.zmag == (short)(short) -10448);
                Debug.Assert(pack.xmag == (short)(short) -30150);
                Debug.Assert(pack.zgyro == (short)(short)29657);
                Debug.Assert(pack.xgyro == (short)(short)26653);
                Debug.Assert(pack.ygyro == (short)(short)8063);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.yacc = (short)(short)17958;
            p26.xmag = (short)(short) -30150;
            p26.zgyro = (short)(short)29657;
            p26.ymag = (short)(short) -18785;
            p26.zmag = (short)(short) -10448;
            p26.xacc = (short)(short)2558;
            p26.time_boot_ms = (uint)3608876360U;
            p26.ygyro = (short)(short)8063;
            p26.xgyro = (short)(short)26653;
            p26.zacc = (short)(short) -15592;
            ADV_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zgyro == (short)(short)13736);
                Debug.Assert(pack.zmag == (short)(short)30864);
                Debug.Assert(pack.zacc == (short)(short) -15236);
                Debug.Assert(pack.xmag == (short)(short) -37);
                Debug.Assert(pack.ygyro == (short)(short) -12126);
                Debug.Assert(pack.yacc == (short)(short) -29316);
                Debug.Assert(pack.xacc == (short)(short) -23871);
                Debug.Assert(pack.ymag == (short)(short) -8539);
                Debug.Assert(pack.xgyro == (short)(short)10978);
                Debug.Assert(pack.time_usec == (ulong)1318638057835445866L);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.xmag = (short)(short) -37;
            p27.ygyro = (short)(short) -12126;
            p27.zgyro = (short)(short)13736;
            p27.yacc = (short)(short) -29316;
            p27.ymag = (short)(short) -8539;
            p27.zacc = (short)(short) -15236;
            p27.xgyro = (short)(short)10978;
            p27.xacc = (short)(short) -23871;
            p27.zmag = (short)(short)30864;
            p27.time_usec = (ulong)1318638057835445866L;
            ADV_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short)23104);
                Debug.Assert(pack.press_abs == (short)(short)2612);
                Debug.Assert(pack.press_diff1 == (short)(short) -11270);
                Debug.Assert(pack.press_diff2 == (short)(short) -14993);
                Debug.Assert(pack.time_usec == (ulong)873921896216784258L);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.time_usec = (ulong)873921896216784258L;
            p28.temperature = (short)(short)23104;
            p28.press_diff1 = (short)(short) -11270;
            p28.press_abs = (short)(short)2612;
            p28.press_diff2 = (short)(short) -14993;
            ADV_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)1211101249U);
                Debug.Assert(pack.press_diff == (float) -1.9126492E37F);
                Debug.Assert(pack.temperature == (short)(short)15294);
                Debug.Assert(pack.press_abs == (float) -1.5814851E38F);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.time_boot_ms = (uint)1211101249U;
            p29.press_diff = (float) -1.9126492E37F;
            p29.press_abs = (float) -1.5814851E38F;
            p29.temperature = (short)(short)15294;
            ADV_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float)3.1964065E38F);
                Debug.Assert(pack.yawspeed == (float) -3.8390094E37F);
                Debug.Assert(pack.pitchspeed == (float) -4.2224765E37F);
                Debug.Assert(pack.time_boot_ms == (uint)3933744645U);
                Debug.Assert(pack.roll == (float)6.1826666E37F);
                Debug.Assert(pack.rollspeed == (float)2.975912E37F);
                Debug.Assert(pack.yaw == (float)2.2504974E37F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.pitch = (float)3.1964065E38F;
            p30.rollspeed = (float)2.975912E37F;
            p30.time_boot_ms = (uint)3933744645U;
            p30.yaw = (float)2.2504974E37F;
            p30.yawspeed = (float) -3.8390094E37F;
            p30.pitchspeed = (float) -4.2224765E37F;
            p30.roll = (float)6.1826666E37F;
            ADV_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q4 == (float) -5.571771E36F);
                Debug.Assert(pack.q3 == (float)1.5098282E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2518851041U);
                Debug.Assert(pack.pitchspeed == (float)2.4528429E38F);
                Debug.Assert(pack.rollspeed == (float) -4.6979405E37F);
                Debug.Assert(pack.q2 == (float)1.0456247E37F);
                Debug.Assert(pack.q1 == (float) -1.3742153E38F);
                Debug.Assert(pack.yawspeed == (float) -2.55949E38F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.pitchspeed = (float)2.4528429E38F;
            p31.yawspeed = (float) -2.55949E38F;
            p31.time_boot_ms = (uint)2518851041U;
            p31.q4 = (float) -5.571771E36F;
            p31.rollspeed = (float) -4.6979405E37F;
            p31.q1 = (float) -1.3742153E38F;
            p31.q3 = (float)1.5098282E38F;
            p31.q2 = (float)1.0456247E37F;
            ADV_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vy == (float)3.01159E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1217774485U);
                Debug.Assert(pack.vz == (float)1.626774E38F);
                Debug.Assert(pack.x == (float)1.0705639E38F);
                Debug.Assert(pack.y == (float) -8.036291E37F);
                Debug.Assert(pack.z == (float) -2.7419867E38F);
                Debug.Assert(pack.vx == (float)2.6675557E38F);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.time_boot_ms = (uint)1217774485U;
            p32.y = (float) -8.036291E37F;
            p32.vx = (float)2.6675557E38F;
            p32.vy = (float)3.01159E38F;
            p32.vz = (float)1.626774E38F;
            p32.x = (float)1.0705639E38F;
            p32.z = (float) -2.7419867E38F;
            ADV_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.hdg == (ushort)(ushort)47905);
                Debug.Assert(pack.vz == (short)(short)17456);
                Debug.Assert(pack.alt == (int)50116022);
                Debug.Assert(pack.relative_alt == (int) -492238303);
                Debug.Assert(pack.lon == (int) -2030833167);
                Debug.Assert(pack.time_boot_ms == (uint)2752845315U);
                Debug.Assert(pack.lat == (int) -1311810112);
                Debug.Assert(pack.vx == (short)(short)3913);
                Debug.Assert(pack.vy == (short)(short)9181);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.vx = (short)(short)3913;
            p33.vy = (short)(short)9181;
            p33.hdg = (ushort)(ushort)47905;
            p33.lon = (int) -2030833167;
            p33.relative_alt = (int) -492238303;
            p33.vz = (short)(short)17456;
            p33.time_boot_ms = (uint)2752845315U;
            p33.alt = (int)50116022;
            p33.lat = (int) -1311810112;
            ADV_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan5_scaled == (short)(short) -31478);
                Debug.Assert(pack.chan4_scaled == (short)(short)24671);
                Debug.Assert(pack.chan3_scaled == (short)(short)12657);
                Debug.Assert(pack.chan8_scaled == (short)(short)19835);
                Debug.Assert(pack.time_boot_ms == (uint)2638523830U);
                Debug.Assert(pack.chan7_scaled == (short)(short)11043);
                Debug.Assert(pack.chan6_scaled == (short)(short)19325);
                Debug.Assert(pack.chan2_scaled == (short)(short) -29080);
                Debug.Assert(pack.port == (byte)(byte)96);
                Debug.Assert(pack.chan1_scaled == (short)(short)5040);
                Debug.Assert(pack.rssi == (byte)(byte)121);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.chan2_scaled = (short)(short) -29080;
            p34.port = (byte)(byte)96;
            p34.chan7_scaled = (short)(short)11043;
            p34.chan8_scaled = (short)(short)19835;
            p34.time_boot_ms = (uint)2638523830U;
            p34.rssi = (byte)(byte)121;
            p34.chan3_scaled = (short)(short)12657;
            p34.chan6_scaled = (short)(short)19325;
            p34.chan5_scaled = (short)(short) -31478;
            p34.chan4_scaled = (short)(short)24671;
            p34.chan1_scaled = (short)(short)5040;
            ADV_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)57590);
                Debug.Assert(pack.rssi == (byte)(byte)241);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)59687);
                Debug.Assert(pack.time_boot_ms == (uint)444458867U);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)18161);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)12727);
                Debug.Assert(pack.port == (byte)(byte)56);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)42496);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)42924);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)12429);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)23215);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan1_raw = (ushort)(ushort)42496;
            p35.chan5_raw = (ushort)(ushort)18161;
            p35.chan7_raw = (ushort)(ushort)59687;
            p35.chan3_raw = (ushort)(ushort)57590;
            p35.chan8_raw = (ushort)(ushort)12429;
            p35.chan2_raw = (ushort)(ushort)12727;
            p35.time_boot_ms = (uint)444458867U;
            p35.rssi = (byte)(byte)241;
            p35.port = (byte)(byte)56;
            p35.chan6_raw = (ushort)(ushort)42924;
            p35.chan4_raw = (ushort)(ushort)23215;
            ADV_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)9374);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)32511);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)37585);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)60158);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)12763);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)7886);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)25934);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)64495);
                Debug.Assert(pack.port == (byte)(byte)69);
                Debug.Assert(pack.time_usec == (uint)2775108718U);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)5459);
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)35954);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)41705);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)5777);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)52834);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)1176);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)18976);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)36767);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo9_raw_SET((ushort)(ushort)12763, PH) ;
            p36.servo10_raw_SET((ushort)(ushort)32511, PH) ;
            p36.servo14_raw_SET((ushort)(ushort)5777, PH) ;
            p36.servo4_raw = (ushort)(ushort)18976;
            p36.servo15_raw_SET((ushort)(ushort)9374, PH) ;
            p36.servo3_raw = (ushort)(ushort)37585;
            p36.servo5_raw = (ushort)(ushort)60158;
            p36.time_usec = (uint)2775108718U;
            p36.servo12_raw_SET((ushort)(ushort)35954, PH) ;
            p36.servo13_raw_SET((ushort)(ushort)25934, PH) ;
            p36.servo2_raw = (ushort)(ushort)52834;
            p36.servo7_raw = (ushort)(ushort)64495;
            p36.servo16_raw_SET((ushort)(ushort)1176, PH) ;
            p36.servo11_raw_SET((ushort)(ushort)41705, PH) ;
            p36.port = (byte)(byte)69;
            p36.servo6_raw = (ushort)(ushort)7886;
            p36.servo1_raw = (ushort)(ushort)5459;
            p36.servo8_raw = (ushort)(ushort)36767;
            ADV_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)46);
                Debug.Assert(pack.start_index == (short)(short) -1324);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_system == (byte)(byte)99);
                Debug.Assert(pack.end_index == (short)(short)3098);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p37.target_component = (byte)(byte)46;
            p37.end_index = (short)(short)3098;
            p37.start_index = (short)(short) -1324;
            p37.target_system = (byte)(byte)99;
            ADV_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_index == (short)(short)4520);
                Debug.Assert(pack.target_component == (byte)(byte)32);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.end_index == (short)(short) -11675);
                Debug.Assert(pack.target_system == (byte)(byte)182);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.target_system = (byte)(byte)182;
            p38.target_component = (byte)(byte)32;
            p38.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p38.start_index = (short)(short)4520;
            p38.end_index = (short)(short) -11675;
            ADV_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_CONDITION_LAST);
                Debug.Assert(pack.y == (float) -1.6404362E38F);
                Debug.Assert(pack.autocontinue == (byte)(byte)8);
                Debug.Assert(pack.current == (byte)(byte)211);
                Debug.Assert(pack.param4 == (float) -5.3356464E35F);
                Debug.Assert(pack.param2 == (float)5.303887E37F);
                Debug.Assert(pack.seq == (ushort)(ushort)57547);
                Debug.Assert(pack.param1 == (float)4.2838152E35F);
                Debug.Assert(pack.param3 == (float)1.3224834E38F);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.z == (float) -2.944066E38F);
                Debug.Assert(pack.x == (float)1.2052335E38F);
                Debug.Assert(pack.target_system == (byte)(byte)120);
                Debug.Assert(pack.target_component == (byte)(byte)242);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.y = (float) -1.6404362E38F;
            p39.x = (float)1.2052335E38F;
            p39.target_component = (byte)(byte)242;
            p39.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p39.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            p39.autocontinue = (byte)(byte)8;
            p39.current = (byte)(byte)211;
            p39.z = (float) -2.944066E38F;
            p39.command = MAV_CMD.MAV_CMD_CONDITION_LAST;
            p39.param3 = (float)1.3224834E38F;
            p39.seq = (ushort)(ushort)57547;
            p39.param2 = (float)5.303887E37F;
            p39.param4 = (float) -5.3356464E35F;
            p39.param1 = (float)4.2838152E35F;
            p39.target_system = (byte)(byte)120;
            ADV_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)119);
                Debug.Assert(pack.target_component == (byte)(byte)143);
                Debug.Assert(pack.seq == (ushort)(ushort)25290);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.target_component = (byte)(byte)143;
            p40.seq = (ushort)(ushort)25290;
            p40.target_system = (byte)(byte)119;
            p40.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            ADV_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)33997);
                Debug.Assert(pack.target_component == (byte)(byte)194);
                Debug.Assert(pack.target_system == (byte)(byte)190);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_system = (byte)(byte)190;
            p41.target_component = (byte)(byte)194;
            p41.seq = (ushort)(ushort)33997;
            ADV_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)29214);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)29214;
            ADV_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_system == (byte)(byte)215);
                Debug.Assert(pack.target_component == (byte)(byte)250);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p43.target_system = (byte)(byte)215;
            p43.target_component = (byte)(byte)250;
            ADV_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (ushort)(ushort)17573);
                Debug.Assert(pack.target_system == (byte)(byte)129);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_component == (byte)(byte)232);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.target_component = (byte)(byte)232;
            p44.target_system = (byte)(byte)129;
            p44.count = (ushort)(ushort)17573;
            p44.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            ADV_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)101);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.target_system == (byte)(byte)52);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.target_component = (byte)(byte)101;
            p45.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p45.target_system = (byte)(byte)52;
            ADV_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)6269);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)6269;
            ADV_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)206);
                Debug.Assert(pack.target_system == (byte)(byte)229);
                Debug.Assert(pack.type == MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED_FRAME);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.target_system = (byte)(byte)229;
            p47.target_component = (byte)(byte)206;
            p47.type = MAV_MISSION_RESULT.MAV_MISSION_UNSUPPORTED_FRAME;
            p47.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            ADV_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)35);
                Debug.Assert(pack.longitude == (int) -1432783792);
                Debug.Assert(pack.altitude == (int)1510367100);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)2782462449698012613L);
                Debug.Assert(pack.latitude == (int)1068663113);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.longitude = (int) -1432783792;
            p48.altitude = (int)1510367100;
            p48.target_system = (byte)(byte)35;
            p48.time_usec_SET((ulong)2782462449698012613L, PH) ;
            p48.latitude = (int)1068663113;
            ADV_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)426053214324499166L);
                Debug.Assert(pack.latitude == (int) -2101331421);
                Debug.Assert(pack.longitude == (int) -1790459372);
                Debug.Assert(pack.altitude == (int)1553579915);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.latitude = (int) -2101331421;
            p49.longitude = (int) -1790459372;
            p49.time_usec_SET((ulong)426053214324499166L, PH) ;
            p49.altitude = (int)1553579915;
            ADV_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_max == (float)3.0364363E38F);
                Debug.Assert(pack.scale == (float)7.49752E37F);
                Debug.Assert(pack.param_id_LEN(ph) == 2);
                Debug.Assert(pack.param_id_TRY(ph).Equals("nj"));
                Debug.Assert(pack.param_value_min == (float) -4.679617E37F);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)222);
                Debug.Assert(pack.target_component == (byte)(byte)63);
                Debug.Assert(pack.param_value0 == (float)1.1838733E38F);
                Debug.Assert(pack.param_index == (short)(short)12034);
                Debug.Assert(pack.target_system == (byte)(byte)239);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.target_system = (byte)(byte)239;
            p50.param_index = (short)(short)12034;
            p50.param_value_max = (float)3.0364363E38F;
            p50.param_value_min = (float) -4.679617E37F;
            p50.target_component = (byte)(byte)63;
            p50.param_value0 = (float)1.1838733E38F;
            p50.param_id_SET("nj", PH) ;
            p50.scale = (float)7.49752E37F;
            p50.parameter_rc_channel_index = (byte)(byte)222;
            ADV_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)69);
                Debug.Assert(pack.target_component == (byte)(byte)61);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.seq == (ushort)(ushort)63782);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.target_component = (byte)(byte)61;
            p51.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p51.target_system = (byte)(byte)69;
            p51.seq = (ushort)(ushort)63782;
            ADV_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2x == (float)2.6066245E37F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.target_component == (byte)(byte)149);
                Debug.Assert(pack.p2y == (float)1.8731992E38F);
                Debug.Assert(pack.target_system == (byte)(byte)246);
                Debug.Assert(pack.p1y == (float) -9.358089E37F);
                Debug.Assert(pack.p1x == (float) -1.7731534E38F);
                Debug.Assert(pack.p2z == (float)3.7139928E36F);
                Debug.Assert(pack.p1z == (float)1.0707582E38F);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p2z = (float)3.7139928E36F;
            p54.target_system = (byte)(byte)246;
            p54.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p54.p1z = (float)1.0707582E38F;
            p54.p2x = (float)2.6066245E37F;
            p54.target_component = (byte)(byte)149;
            p54.p1x = (float) -1.7731534E38F;
            p54.p2y = (float)1.8731992E38F;
            p54.p1y = (float) -9.358089E37F;
            ADV_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
                Debug.Assert(pack.p1x == (float)1.8631973E38F);
                Debug.Assert(pack.p2x == (float)2.1517418E38F);
                Debug.Assert(pack.p2y == (float) -2.2876264E38F);
                Debug.Assert(pack.p1y == (float) -9.418954E37F);
                Debug.Assert(pack.p2z == (float) -2.7081307E38F);
                Debug.Assert(pack.p1z == (float)1.0381589E38F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p2z = (float) -2.7081307E38F;
            p55.p2x = (float)2.1517418E38F;
            p55.frame = MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED;
            p55.p1z = (float)1.0381589E38F;
            p55.p1y = (float) -9.418954E37F;
            p55.p1x = (float)1.8631973E38F;
            p55.p2y = (float) -2.2876264E38F;
            ADV_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-2.4762743E38F, -1.8292748E38F, 2.6795012E38F, 1.9649704E38F, -2.4850548E37F, 1.5893233E38F, 1.7117575E38F, 3.2910907E38F, -3.0015337E37F}));
                Debug.Assert(pack.pitchspeed == (float) -3.0738635E36F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {8.597119E37F, -2.0365983E38F, 1.5469343E38F, 2.5984359E38F}));
                Debug.Assert(pack.time_usec == (ulong)3883911208122689311L);
                Debug.Assert(pack.yawspeed == (float) -2.9466009E38F);
                Debug.Assert(pack.rollspeed == (float) -1.7426074E38F);
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.covariance_SET(new float[] {-2.4762743E38F, -1.8292748E38F, 2.6795012E38F, 1.9649704E38F, -2.4850548E37F, 1.5893233E38F, 1.7117575E38F, 3.2910907E38F, -3.0015337E37F}, 0) ;
            p61.time_usec = (ulong)3883911208122689311L;
            p61.yawspeed = (float) -2.9466009E38F;
            p61.q_SET(new float[] {8.597119E37F, -2.0365983E38F, 1.5469343E38F, 2.5984359E38F}, 0) ;
            p61.rollspeed = (float) -1.7426074E38F;
            p61.pitchspeed = (float) -3.0738635E36F;
            ADV_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wp_dist == (ushort)(ushort)35709);
                Debug.Assert(pack.nav_roll == (float)2.8321662E37F);
                Debug.Assert(pack.aspd_error == (float)2.5650443E38F);
                Debug.Assert(pack.xtrack_error == (float)1.4661421E38F);
                Debug.Assert(pack.target_bearing == (short)(short) -32612);
                Debug.Assert(pack.alt_error == (float)1.2628746E38F);
                Debug.Assert(pack.nav_bearing == (short)(short) -16732);
                Debug.Assert(pack.nav_pitch == (float) -1.4120862E38F);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.nav_pitch = (float) -1.4120862E38F;
            p62.nav_bearing = (short)(short) -16732;
            p62.wp_dist = (ushort)(ushort)35709;
            p62.alt_error = (float)1.2628746E38F;
            p62.nav_roll = (float)2.8321662E37F;
            p62.target_bearing = (short)(short) -32612;
            p62.xtrack_error = (float)1.4661421E38F;
            p62.aspd_error = (float)2.5650443E38F;
            ADV_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.relative_alt == (int)424475331);
                Debug.Assert(pack.lat == (int) -2137636562);
                Debug.Assert(pack.alt == (int) -1674561331);
                Debug.Assert(pack.time_usec == (ulong)7408689989930880477L);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {-3.1443765E38F, -2.4165061E38F, -2.1280988E38F, 9.9071445E36F, 1.0541154E38F, 3.090476E38F, -5.7222E37F, -1.1690571E38F, -7.655127E37F, -6.35088E36F, -8.607557E37F, 2.1677334E38F, 2.5045466E38F, 2.4910113E38F, 2.877884E38F, 1.9306988E38F, 1.0758937E38F, -1.7891909E38F, -2.7363155E38F, 1.0692288E38F, -2.6152417E38F, 2.7857806E38F, 6.104237E37F, -1.9255166E38F, 2.113019E38F, -1.6291377E38F, 2.5495597E38F, -2.0566282E38F, -3.2689896E38F, 9.199609E37F, -3.1177623E38F, 1.4347424E38F, -1.0011985E38F, -2.896879E38F, 2.002116E38F, 1.394277E38F}));
                Debug.Assert(pack.vz == (float) -1.4726176E38F);
                Debug.Assert(pack.vx == (float)2.1336103E38F);
                Debug.Assert(pack.vy == (float) -1.9065842E38F);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
                Debug.Assert(pack.lon == (int)629679604);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.vx = (float)2.1336103E38F;
            p63.vz = (float) -1.4726176E38F;
            p63.lat = (int) -2137636562;
            p63.lon = (int)629679604;
            p63.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS;
            p63.alt = (int) -1674561331;
            p63.time_usec = (ulong)7408689989930880477L;
            p63.vy = (float) -1.9065842E38F;
            p63.covariance_SET(new float[] {-3.1443765E38F, -2.4165061E38F, -2.1280988E38F, 9.9071445E36F, 1.0541154E38F, 3.090476E38F, -5.7222E37F, -1.1690571E38F, -7.655127E37F, -6.35088E36F, -8.607557E37F, 2.1677334E38F, 2.5045466E38F, 2.4910113E38F, 2.877884E38F, 1.9306988E38F, 1.0758937E38F, -1.7891909E38F, -2.7363155E38F, 1.0692288E38F, -2.6152417E38F, 2.7857806E38F, 6.104237E37F, -1.9255166E38F, 2.113019E38F, -1.6291377E38F, 2.5495597E38F, -2.0566282E38F, -3.2689896E38F, 9.199609E37F, -3.1177623E38F, 1.4347424E38F, -1.0011985E38F, -2.896879E38F, 2.002116E38F, 1.394277E38F}, 0) ;
            p63.relative_alt = (int)424475331;
            ADV_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.az == (float) -1.0331681E38F);
                Debug.Assert(pack.x == (float) -4.846838E37F);
                Debug.Assert(pack.vy == (float)4.030409E37F);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
                Debug.Assert(pack.z == (float)1.3221324E38F);
                Debug.Assert(pack.vx == (float)2.5479426E38F);
                Debug.Assert(pack.vz == (float)3.0613176E38F);
                Debug.Assert(pack.y == (float)1.2100844E37F);
                Debug.Assert(pack.ay == (float) -3.284566E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {4.9980497E37F, 1.5867939E38F, 2.6073538E36F, 1.8047544E38F, -7.3167855E37F, -6.6616504E37F, 2.5659319E38F, -2.536767E37F, 7.690442E37F, 3.2115086E38F, -2.984992E38F, -1.815824E38F, -1.4064231E37F, 2.0046241E38F, -2.0744494E38F, -3.5511284E37F, -1.7356834E38F, -1.2490115E38F, -1.5140107E38F, -1.6219313E38F, 3.9012576E37F, -1.6221643E38F, 4.9602443E37F, -1.6325585E38F, 2.2251548E38F, 2.3638574E38F, -1.4972851E38F, 3.0584712E38F, -2.579712E38F, -1.1453308E38F, 3.715518E37F, 2.2548078E38F, 4.894906E37F, -9.202356E36F, 1.9740066E38F, -1.325304E37F, -3.5744134E37F, -1.0052769E38F, 1.6289857E38F, 1.9116124E38F, 2.316508E38F, -1.985792E38F, 5.96956E37F, 8.47871E37F, 1.1464857E38F}));
                Debug.Assert(pack.ax == (float)1.1169291E38F);
                Debug.Assert(pack.time_usec == (ulong)1674986482781307524L);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.vz = (float)3.0613176E38F;
            p64.ay = (float) -3.284566E38F;
            p64.vy = (float)4.030409E37F;
            p64.ax = (float)1.1169291E38F;
            p64.vx = (float)2.5479426E38F;
            p64.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
            p64.az = (float) -1.0331681E38F;
            p64.z = (float)1.3221324E38F;
            p64.time_usec = (ulong)1674986482781307524L;
            p64.x = (float) -4.846838E37F;
            p64.y = (float)1.2100844E37F;
            p64.covariance_SET(new float[] {4.9980497E37F, 1.5867939E38F, 2.6073538E36F, 1.8047544E38F, -7.3167855E37F, -6.6616504E37F, 2.5659319E38F, -2.536767E37F, 7.690442E37F, 3.2115086E38F, -2.984992E38F, -1.815824E38F, -1.4064231E37F, 2.0046241E38F, -2.0744494E38F, -3.5511284E37F, -1.7356834E38F, -1.2490115E38F, -1.5140107E38F, -1.6219313E38F, 3.9012576E37F, -1.6221643E38F, 4.9602443E37F, -1.6325585E38F, 2.2251548E38F, 2.3638574E38F, -1.4972851E38F, 3.0584712E38F, -2.579712E38F, -1.1453308E38F, 3.715518E37F, 2.2548078E38F, 4.894906E37F, -9.202356E36F, 1.9740066E38F, -1.325304E37F, -3.5744134E37F, -1.0052769E38F, 1.6289857E38F, 1.9116124E38F, 2.316508E38F, -1.985792E38F, 5.96956E37F, 8.47871E37F, 1.1464857E38F}, 0) ;
            ADV_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)22190);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)26171);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)37640);
                Debug.Assert(pack.time_boot_ms == (uint)2072143202U);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)21159);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)54971);
                Debug.Assert(pack.chancount == (byte)(byte)50);
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)57606);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)33669);
                Debug.Assert(pack.rssi == (byte)(byte)198);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)29516);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)46718);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)44158);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)11068);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)32431);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)57223);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)22262);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)6367);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)20847);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)60509);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)11432);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan6_raw = (ushort)(ushort)37640;
            p65.chan2_raw = (ushort)(ushort)46718;
            p65.chan18_raw = (ushort)(ushort)11432;
            p65.chan3_raw = (ushort)(ushort)11068;
            p65.chan14_raw = (ushort)(ushort)57606;
            p65.chan13_raw = (ushort)(ushort)32431;
            p65.chan11_raw = (ushort)(ushort)6367;
            p65.chan4_raw = (ushort)(ushort)44158;
            p65.rssi = (byte)(byte)198;
            p65.chan5_raw = (ushort)(ushort)29516;
            p65.chan8_raw = (ushort)(ushort)22190;
            p65.chan10_raw = (ushort)(ushort)21159;
            p65.chan7_raw = (ushort)(ushort)20847;
            p65.time_boot_ms = (uint)2072143202U;
            p65.chan16_raw = (ushort)(ushort)60509;
            p65.chan17_raw = (ushort)(ushort)22262;
            p65.chan12_raw = (ushort)(ushort)54971;
            p65.chan15_raw = (ushort)(ushort)57223;
            p65.chan1_raw = (ushort)(ushort)33669;
            p65.chancount = (byte)(byte)50;
            p65.chan9_raw = (ushort)(ushort)26171;
            ADV_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_stop == (byte)(byte)154);
                Debug.Assert(pack.req_stream_id == (byte)(byte)25);
                Debug.Assert(pack.target_component == (byte)(byte)79);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)39011);
                Debug.Assert(pack.target_system == (byte)(byte)70);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_system = (byte)(byte)70;
            p66.start_stop = (byte)(byte)154;
            p66.req_message_rate = (ushort)(ushort)39011;
            p66.target_component = (byte)(byte)79;
            p66.req_stream_id = (byte)(byte)25;
            ADV_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_rate == (ushort)(ushort)56416);
                Debug.Assert(pack.stream_id == (byte)(byte)147);
                Debug.Assert(pack.on_off == (byte)(byte)186);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.message_rate = (ushort)(ushort)56416;
            p67.on_off = (byte)(byte)186;
            p67.stream_id = (byte)(byte)147;
            ADV_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (short)(short)25738);
                Debug.Assert(pack.target == (byte)(byte)25);
                Debug.Assert(pack.r == (short)(short)24350);
                Debug.Assert(pack.x == (short)(short)16009);
                Debug.Assert(pack.z == (short)(short)13248);
                Debug.Assert(pack.buttons == (ushort)(ushort)22437);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.y = (short)(short)25738;
            p69.z = (short)(short)13248;
            p69.target = (byte)(byte)25;
            p69.r = (short)(short)24350;
            p69.x = (short)(short)16009;
            p69.buttons = (ushort)(ushort)22437;
            ADV_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)36200);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)7053);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)39930);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)36073);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)55487);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)4180);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)37003);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)43640);
                Debug.Assert(pack.target_component == (byte)(byte)208);
                Debug.Assert(pack.target_system == (byte)(byte)202);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.chan7_raw = (ushort)(ushort)7053;
            p70.target_system = (byte)(byte)202;
            p70.target_component = (byte)(byte)208;
            p70.chan8_raw = (ushort)(ushort)36200;
            p70.chan1_raw = (ushort)(ushort)37003;
            p70.chan4_raw = (ushort)(ushort)39930;
            p70.chan6_raw = (ushort)(ushort)55487;
            p70.chan3_raw = (ushort)(ushort)36073;
            p70.chan5_raw = (ushort)(ushort)4180;
            p70.chan2_raw = (ushort)(ushort)43640;
            ADV_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param3 == (float) -1.9537974E38F);
                Debug.Assert(pack.seq == (ushort)(ushort)4534);
                Debug.Assert(pack.param1 == (float) -1.9517742E36F);
                Debug.Assert(pack.target_component == (byte)(byte)78);
                Debug.Assert(pack.param2 == (float)1.5818468E38F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_USER_1);
                Debug.Assert(pack.target_system == (byte)(byte)79);
                Debug.Assert(pack.current == (byte)(byte)184);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.autocontinue == (byte)(byte)156);
                Debug.Assert(pack.param4 == (float)9.423428E37F);
                Debug.Assert(pack.y == (int) -915164919);
                Debug.Assert(pack.z == (float) -2.6364277E38F);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.x == (int) -870328294);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.current = (byte)(byte)184;
            p73.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p73.param4 = (float)9.423428E37F;
            p73.x = (int) -870328294;
            p73.param3 = (float) -1.9537974E38F;
            p73.target_component = (byte)(byte)78;
            p73.seq = (ushort)(ushort)4534;
            p73.autocontinue = (byte)(byte)156;
            p73.y = (int) -915164919;
            p73.z = (float) -2.6364277E38F;
            p73.param1 = (float) -1.9517742E36F;
            p73.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p73.param2 = (float)1.5818468E38F;
            p73.command = MAV_CMD.MAV_CMD_USER_1;
            p73.target_system = (byte)(byte)79;
            ADV_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.airspeed == (float)1.6288213E38F);
                Debug.Assert(pack.climb == (float)5.9144373E37F);
                Debug.Assert(pack.throttle == (ushort)(ushort)11458);
                Debug.Assert(pack.groundspeed == (float) -2.614082E38F);
                Debug.Assert(pack.alt == (float) -2.1846021E38F);
                Debug.Assert(pack.heading == (short)(short) -37);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.climb = (float)5.9144373E37F;
            p74.airspeed = (float)1.6288213E38F;
            p74.alt = (float) -2.1846021E38F;
            p74.throttle = (ushort)(ushort)11458;
            p74.groundspeed = (float) -2.614082E38F;
            p74.heading = (short)(short) -37;
            ADV_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(ADV_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current == (byte)(byte)203);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE);
                Debug.Assert(pack.autocontinue == (byte)(byte)122);
                Debug.Assert(pack.param3 == (float) -9.030802E37F);
                Debug.Assert(pack.param4 == (float) -1.2988913E38F);
                Debug.Assert(pack.param1 == (float) -1.4266638E38F);
                Debug.Assert(pack.target_component == (byte)(byte)219);
                Debug.Assert(pack.x == (int) -753106385);
                Debug.Assert(pack.y == (int) -33631919);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_NED);
                Debug.Assert(pack.target_system == (byte)(byte)208);
                Debug.Assert(pack.z == (float) -1.3671068E38F);
                Debug.Assert(pack.param2 == (float) -2.4435655E38F);
            };
            GroundControl.COMMAND_INT p75 = CommunicationChannel.new_COMMAND_INT();
            PH.setPack(p75);
            p75.frame = MAV_FRAME.MAV_FRAME_LOCAL_NED;
            p75.z = (float) -1.3671068E38F;
            p75.param2 = (float) -2.4435655E38F;
            p75.param1 = (float) -1.4266638E38F;
            p75.command = MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE;
            p75.target_system = (byte)(byte)208;
            p75.target_component = (byte)(byte)219;
            p75.y = (int) -33631919;
            p75.param3 = (float) -9.030802E37F;
            p75.current = (byte)(byte)203;
            p75.param4 = (float) -1.2988913E38F;
            p75.autocontinue = (byte)(byte)122;
            p75.x = (int) -753106385;
            CommunicationChannel.instance.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param1 == (float)1.639786E37F);
                Debug.Assert(pack.param7 == (float) -2.4974473E38F);
                Debug.Assert(pack.param2 == (float) -7.3043706E37F);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE);
                Debug.Assert(pack.target_system == (byte)(byte)4);
                Debug.Assert(pack.param4 == (float) -3.246989E38F);
                Debug.Assert(pack.target_component == (byte)(byte)37);
                Debug.Assert(pack.param6 == (float)1.701484E38F);
                Debug.Assert(pack.param3 == (float)1.2444797E38F);
                Debug.Assert(pack.confirmation == (byte)(byte)165);
                Debug.Assert(pack.param5 == (float)1.1512523E38F);
            };
            GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.param4 = (float) -3.246989E38F;
            p76.param1 = (float)1.639786E37F;
            p76.target_component = (byte)(byte)37;
            p76.target_system = (byte)(byte)4;
            p76.param2 = (float) -7.3043706E37F;
            p76.confirmation = (byte)(byte)165;
            p76.param7 = (float) -2.4974473E38F;
            p76.param6 = (float)1.701484E38F;
            p76.command = MAV_CMD.MAV_CMD_DO_MOUNT_CONFIGURE;
            p76.param5 = (float)1.1512523E38F;
            p76.param3 = (float)1.2444797E38F;
            CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.result == MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)46);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)203);
                Debug.Assert(pack.result_param2_TRY(ph) == (int) -117164053);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)230);
            };
            GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.result = MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED;
            p77.command = MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL;
            p77.result_param2_SET((int) -117164053, PH) ;
            p77.target_system_SET((byte)(byte)230, PH) ;
            p77.progress_SET((byte)(byte)203, PH) ;
            p77.target_component_SET((byte)(byte)46, PH) ;
            CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float) -2.4148346E38F);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)244);
                Debug.Assert(pack.thrust == (float)2.72285E38F);
                Debug.Assert(pack.mode_switch == (byte)(byte)176);
                Debug.Assert(pack.yaw == (float)7.931476E37F);
                Debug.Assert(pack.roll == (float)7.7588994E37F);
                Debug.Assert(pack.time_boot_ms == (uint)2015733737U);
            };
            GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.yaw = (float)7.931476E37F;
            p81.thrust = (float)2.72285E38F;
            p81.roll = (float)7.7588994E37F;
            p81.mode_switch = (byte)(byte)176;
            p81.time_boot_ms = (uint)2015733737U;
            p81.manual_override_switch = (byte)(byte)244;
            p81.pitch = (float) -2.4148346E38F;
            CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.body_pitch_rate == (float)8.3478276E37F);
                Debug.Assert(pack.target_component == (byte)(byte)145);
                Debug.Assert(pack.body_yaw_rate == (float)2.592214E38F);
                Debug.Assert(pack.target_system == (byte)(byte)41);
                Debug.Assert(pack.body_roll_rate == (float) -1.1533603E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2775959793U);
                Debug.Assert(pack.thrust == (float) -2.5806936E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)225);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.988682E38F, 1.5793634E38F, -2.0340222E37F, -2.964432E38F}));
            };
            GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.target_component = (byte)(byte)145;
            p82.thrust = (float) -2.5806936E38F;
            p82.body_roll_rate = (float) -1.1533603E38F;
            p82.body_yaw_rate = (float)2.592214E38F;
            p82.q_SET(new float[] {2.988682E38F, 1.5793634E38F, -2.0340222E37F, -2.964432E38F}, 0) ;
            p82.target_system = (byte)(byte)41;
            p82.body_pitch_rate = (float)8.3478276E37F;
            p82.time_boot_ms = (uint)2775959793U;
            p82.type_mask = (byte)(byte)225;
            CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3561403172U);
                Debug.Assert(pack.body_roll_rate == (float) -1.3379818E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)189);
                Debug.Assert(pack.body_pitch_rate == (float) -1.0852935E38F);
                Debug.Assert(pack.body_yaw_rate == (float)2.331361E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {8.4865547E37F, 1.4686824E38F, -2.6966178E38F, 6.6275446E37F}));
                Debug.Assert(pack.thrust == (float) -3.1002933E38F);
            };
            GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.q_SET(new float[] {8.4865547E37F, 1.4686824E38F, -2.6966178E38F, 6.6275446E37F}, 0) ;
            p83.body_roll_rate = (float) -1.3379818E38F;
            p83.type_mask = (byte)(byte)189;
            p83.time_boot_ms = (uint)3561403172U;
            p83.body_pitch_rate = (float) -1.0852935E38F;
            p83.body_yaw_rate = (float)2.331361E38F;
            p83.thrust = (float) -3.1002933E38F;
            CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.afz == (float) -8.1560735E36F);
                Debug.Assert(pack.afy == (float) -3.643591E37F);
                Debug.Assert(pack.vy == (float) -5.1239877E37F);
                Debug.Assert(pack.afx == (float) -2.7195294E38F);
                Debug.Assert(pack.x == (float) -2.9963688E38F);
                Debug.Assert(pack.vz == (float) -2.6174776E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.type_mask == (ushort)(ushort)49337);
                Debug.Assert(pack.z == (float) -2.3879629E38F);
                Debug.Assert(pack.vx == (float) -2.633662E38F);
                Debug.Assert(pack.yaw == (float) -2.1513363E38F);
                Debug.Assert(pack.target_system == (byte)(byte)56);
                Debug.Assert(pack.yaw_rate == (float) -2.724708E38F);
                Debug.Assert(pack.target_component == (byte)(byte)234);
                Debug.Assert(pack.y == (float)2.0353755E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3514611493U);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.target_system = (byte)(byte)56;
            p84.y = (float)2.0353755E38F;
            p84.vx = (float) -2.633662E38F;
            p84.vz = (float) -2.6174776E38F;
            p84.x = (float) -2.9963688E38F;
            p84.yaw_rate = (float) -2.724708E38F;
            p84.type_mask = (ushort)(ushort)49337;
            p84.yaw = (float) -2.1513363E38F;
            p84.afy = (float) -3.643591E37F;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p84.afx = (float) -2.7195294E38F;
            p84.vy = (float) -5.1239877E37F;
            p84.afz = (float) -8.1560735E36F;
            p84.z = (float) -2.3879629E38F;
            p84.target_component = (byte)(byte)234;
            p84.time_boot_ms = (uint)3514611493U;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)16);
                Debug.Assert(pack.yaw_rate == (float)5.8944663E37F);
                Debug.Assert(pack.target_system == (byte)(byte)179);
                Debug.Assert(pack.lon_int == (int) -1704396237);
                Debug.Assert(pack.vy == (float) -1.2984005E38F);
                Debug.Assert(pack.lat_int == (int) -834188889);
                Debug.Assert(pack.time_boot_ms == (uint)2179481763U);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.alt == (float) -5.77567E37F);
                Debug.Assert(pack.vz == (float) -1.2370894E36F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)23398);
                Debug.Assert(pack.yaw == (float)2.581052E38F);
                Debug.Assert(pack.afy == (float) -2.5101603E38F);
                Debug.Assert(pack.afz == (float) -1.0313623E38F);
                Debug.Assert(pack.afx == (float)2.6370284E38F);
                Debug.Assert(pack.vx == (float) -9.03922E37F);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.afy = (float) -2.5101603E38F;
            p86.vz = (float) -1.2370894E36F;
            p86.target_component = (byte)(byte)16;
            p86.vy = (float) -1.2984005E38F;
            p86.afz = (float) -1.0313623E38F;
            p86.afx = (float)2.6370284E38F;
            p86.lon_int = (int) -1704396237;
            p86.time_boot_ms = (uint)2179481763U;
            p86.vx = (float) -9.03922E37F;
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p86.yaw_rate = (float)5.8944663E37F;
            p86.target_system = (byte)(byte)179;
            p86.lat_int = (int) -834188889;
            p86.yaw = (float)2.581052E38F;
            p86.alt = (float) -5.77567E37F;
            p86.type_mask = (ushort)(ushort)23398;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.alt == (float)7.916901E36F);
                Debug.Assert(pack.afx == (float)2.9822991E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2823662789U);
                Debug.Assert(pack.vy == (float)3.7046123E37F);
                Debug.Assert(pack.afy == (float) -3.2230791E38F);
                Debug.Assert(pack.yaw == (float) -3.3144905E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)1281);
                Debug.Assert(pack.vx == (float) -1.3492591E38F);
                Debug.Assert(pack.afz == (float)2.0229856E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_GLOBAL);
                Debug.Assert(pack.vz == (float)8.656144E37F);
                Debug.Assert(pack.lat_int == (int)1701206587);
                Debug.Assert(pack.yaw_rate == (float) -1.1959746E38F);
                Debug.Assert(pack.lon_int == (int)358612914);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_GLOBAL;
            p87.vy = (float)3.7046123E37F;
            p87.afz = (float)2.0229856E38F;
            p87.type_mask = (ushort)(ushort)1281;
            p87.vz = (float)8.656144E37F;
            p87.afy = (float) -3.2230791E38F;
            p87.alt = (float)7.916901E36F;
            p87.lon_int = (int)358612914;
            p87.vx = (float) -1.3492591E38F;
            p87.yaw = (float) -3.3144905E38F;
            p87.yaw_rate = (float) -1.1959746E38F;
            p87.time_boot_ms = (uint)2823662789U;
            p87.lat_int = (int)1701206587;
            p87.afx = (float)2.9822991E38F;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)3.3464956E38F);
                Debug.Assert(pack.time_boot_ms == (uint)715382848U);
                Debug.Assert(pack.x == (float) -2.406718E38F);
                Debug.Assert(pack.z == (float)1.7538279E38F);
                Debug.Assert(pack.roll == (float)1.5415464E38F);
                Debug.Assert(pack.pitch == (float) -3.5723471E37F);
                Debug.Assert(pack.y == (float) -1.594408E38F);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.y = (float) -1.594408E38F;
            p89.time_boot_ms = (uint)715382848U;
            p89.yaw = (float)3.3464956E38F;
            p89.z = (float)1.7538279E38F;
            p89.x = (float) -2.406718E38F;
            p89.roll = (float)1.5415464E38F;
            p89.pitch = (float) -3.5723471E37F;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rollspeed == (float)8.953876E37F);
                Debug.Assert(pack.alt == (int) -1727902046);
                Debug.Assert(pack.time_usec == (ulong)1800812430316171127L);
                Debug.Assert(pack.xacc == (short)(short) -6691);
                Debug.Assert(pack.pitchspeed == (float)9.881967E37F);
                Debug.Assert(pack.pitch == (float) -8.979454E37F);
                Debug.Assert(pack.vy == (short)(short) -23840);
                Debug.Assert(pack.vx == (short)(short)3152);
                Debug.Assert(pack.lat == (int)190699197);
                Debug.Assert(pack.yawspeed == (float) -2.8417528E38F);
                Debug.Assert(pack.zacc == (short)(short)19629);
                Debug.Assert(pack.yaw == (float) -1.6019921E38F);
                Debug.Assert(pack.vz == (short)(short)24803);
                Debug.Assert(pack.lon == (int)1853888342);
                Debug.Assert(pack.yacc == (short)(short)29955);
                Debug.Assert(pack.roll == (float)3.0417809E37F);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.yacc = (short)(short)29955;
            p90.alt = (int) -1727902046;
            p90.roll = (float)3.0417809E37F;
            p90.lon = (int)1853888342;
            p90.pitch = (float) -8.979454E37F;
            p90.rollspeed = (float)8.953876E37F;
            p90.vz = (short)(short)24803;
            p90.yawspeed = (float) -2.8417528E38F;
            p90.vx = (short)(short)3152;
            p90.zacc = (short)(short)19629;
            p90.xacc = (short)(short) -6691;
            p90.time_usec = (ulong)1800812430316171127L;
            p90.vy = (short)(short) -23840;
            p90.pitchspeed = (float)9.881967E37F;
            p90.yaw = (float) -1.6019921E38F;
            p90.lat = (int)190699197;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aux4 == (float) -2.6600192E38F);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_AUTO_ARMED);
                Debug.Assert(pack.pitch_elevator == (float) -1.5708773E38F);
                Debug.Assert(pack.aux2 == (float) -1.1945531E38F);
                Debug.Assert(pack.time_usec == (ulong)1558062029149007529L);
                Debug.Assert(pack.throttle == (float)2.343809E38F);
                Debug.Assert(pack.nav_mode == (byte)(byte)214);
                Debug.Assert(pack.roll_ailerons == (float)6.532246E37F);
                Debug.Assert(pack.aux3 == (float) -1.3591506E38F);
                Debug.Assert(pack.yaw_rudder == (float) -3.285828E38F);
                Debug.Assert(pack.aux1 == (float)3.173707E38F);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.roll_ailerons = (float)6.532246E37F;
            p91.yaw_rudder = (float) -3.285828E38F;
            p91.aux4 = (float) -2.6600192E38F;
            p91.throttle = (float)2.343809E38F;
            p91.aux3 = (float) -1.3591506E38F;
            p91.time_usec = (ulong)1558062029149007529L;
            p91.aux2 = (float) -1.1945531E38F;
            p91.aux1 = (float)3.173707E38F;
            p91.nav_mode = (byte)(byte)214;
            p91.mode = MAV_MODE.MAV_MODE_AUTO_ARMED;
            p91.pitch_elevator = (float) -1.5708773E38F;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)53795);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)26035);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)33254);
                Debug.Assert(pack.time_usec == (ulong)4702899979694267084L);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)58744);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)20009);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)29182);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)7542);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)6505);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)13967);
                Debug.Assert(pack.rssi == (byte)(byte)100);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)52135);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)10189);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)7337);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan2_raw = (ushort)(ushort)52135;
            p92.chan9_raw = (ushort)(ushort)58744;
            p92.chan7_raw = (ushort)(ushort)29182;
            p92.chan8_raw = (ushort)(ushort)13967;
            p92.chan12_raw = (ushort)(ushort)33254;
            p92.rssi = (byte)(byte)100;
            p92.chan11_raw = (ushort)(ushort)7337;
            p92.chan10_raw = (ushort)(ushort)53795;
            p92.chan4_raw = (ushort)(ushort)6505;
            p92.chan1_raw = (ushort)(ushort)20009;
            p92.chan6_raw = (ushort)(ushort)10189;
            p92.chan3_raw = (ushort)(ushort)26035;
            p92.time_usec = (ulong)4702899979694267084L;
            p92.chan5_raw = (ushort)(ushort)7542;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)4979205347144864093L);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
                Debug.Assert(pack.flags == (ulong)4351964504286741630L);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.6533698E38F, -1.3685246E38F, 4.044982E37F, -2.1192419E37F, 1.5342018E38F, -3.22528E38F, 4.6763472E36F, -3.3662486E38F, -3.1218462E38F, 2.5573115E38F, 2.012704E38F, 1.3548588E38F, 1.7419367E37F, -1.7922168E38F, -2.294741E38F, 7.5467734E36F}));
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.controls_SET(new float[] {-1.6533698E38F, -1.3685246E38F, 4.044982E37F, -2.1192419E37F, 1.5342018E38F, -3.22528E38F, 4.6763472E36F, -3.3662486E38F, -3.1218462E38F, 2.5573115E38F, 2.012704E38F, 1.3548588E38F, 1.7419367E37F, -1.7922168E38F, -2.294741E38F, 7.5467734E36F}, 0) ;
            p93.time_usec = (ulong)4979205347144864093L;
            p93.mode = MAV_MODE.MAV_MODE_STABILIZE_DISARMED;
            p93.flags = (ulong)4351964504286741630L;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)4.958621E37F);
                Debug.Assert(pack.flow_x == (short)(short)31314);
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float) -2.3818567E37F);
                Debug.Assert(pack.time_usec == (ulong)647758283493076243L);
                Debug.Assert(pack.flow_comp_m_x == (float) -3.3286973E38F);
                Debug.Assert(pack.ground_distance == (float)1.044051E37F);
                Debug.Assert(pack.quality == (byte)(byte)0);
                Debug.Assert(pack.sensor_id == (byte)(byte)6);
                Debug.Assert(pack.flow_y == (short)(short) -13579);
                Debug.Assert(pack.flow_comp_m_y == (float) -2.412984E37F);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.quality = (byte)(byte)0;
            p100.flow_comp_m_x = (float) -3.3286973E38F;
            p100.flow_x = (short)(short)31314;
            p100.ground_distance = (float)1.044051E37F;
            p100.time_usec = (ulong)647758283493076243L;
            p100.sensor_id = (byte)(byte)6;
            p100.flow_rate_y_SET((float) -2.3818567E37F, PH) ;
            p100.flow_comp_m_y = (float) -2.412984E37F;
            p100.flow_y = (short)(short) -13579;
            p100.flow_rate_x_SET((float)4.958621E37F, PH) ;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -2.3540196E37F);
                Debug.Assert(pack.yaw == (float)2.2061984E38F);
                Debug.Assert(pack.y == (float) -2.0042258E38F);
                Debug.Assert(pack.roll == (float) -9.430205E37F);
                Debug.Assert(pack.z == (float) -2.330062E38F);
                Debug.Assert(pack.pitch == (float) -2.3759133E38F);
                Debug.Assert(pack.usec == (ulong)8701093280840319460L);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.z = (float) -2.330062E38F;
            p101.usec = (ulong)8701093280840319460L;
            p101.x = (float) -2.3540196E37F;
            p101.pitch = (float) -2.3759133E38F;
            p101.roll = (float) -9.430205E37F;
            p101.y = (float) -2.0042258E38F;
            p101.yaw = (float)2.2061984E38F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)1099639394143537482L);
                Debug.Assert(pack.z == (float)6.454771E37F);
                Debug.Assert(pack.pitch == (float) -1.4988326E38F);
                Debug.Assert(pack.yaw == (float)1.2135642E38F);
                Debug.Assert(pack.roll == (float) -2.6955973E38F);
                Debug.Assert(pack.x == (float)2.5234332E38F);
                Debug.Assert(pack.y == (float)7.274217E37F);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)1099639394143537482L;
            p102.z = (float)6.454771E37F;
            p102.y = (float)7.274217E37F;
            p102.pitch = (float) -1.4988326E38F;
            p102.roll = (float) -2.6955973E38F;
            p102.yaw = (float)1.2135642E38F;
            p102.x = (float)2.5234332E38F;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.usec == (ulong)82910047985279939L);
                Debug.Assert(pack.y == (float) -1.0452822E38F);
                Debug.Assert(pack.x == (float)4.569796E37F);
                Debug.Assert(pack.z == (float)2.3954777E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.usec = (ulong)82910047985279939L;
            p103.x = (float)4.569796E37F;
            p103.z = (float)2.3954777E38F;
            p103.y = (float) -1.0452822E38F;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -4.914886E37F);
                Debug.Assert(pack.yaw == (float) -7.38702E37F);
                Debug.Assert(pack.y == (float) -1.3831054E38F);
                Debug.Assert(pack.z == (float)1.5908383E38F);
                Debug.Assert(pack.usec == (ulong)4254926183623931190L);
                Debug.Assert(pack.pitch == (float)7.122084E37F);
                Debug.Assert(pack.roll == (float)7.7947724E37F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.y = (float) -1.3831054E38F;
            p104.roll = (float)7.7947724E37F;
            p104.pitch = (float)7.122084E37F;
            p104.yaw = (float) -7.38702E37F;
            p104.z = (float)1.5908383E38F;
            p104.x = (float) -4.914886E37F;
            p104.usec = (ulong)4254926183623931190L;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.abs_pressure == (float)6.896849E36F);
                Debug.Assert(pack.pressure_alt == (float)3.3786947E38F);
                Debug.Assert(pack.xacc == (float)2.609662E37F);
                Debug.Assert(pack.diff_pressure == (float) -1.5495869E38F);
                Debug.Assert(pack.yacc == (float)5.177128E37F);
                Debug.Assert(pack.xgyro == (float)4.178377E37F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)21531);
                Debug.Assert(pack.zgyro == (float)2.177123E38F);
                Debug.Assert(pack.ymag == (float)1.9429376E38F);
                Debug.Assert(pack.ygyro == (float)2.8301117E38F);
                Debug.Assert(pack.xmag == (float) -2.7336597E38F);
                Debug.Assert(pack.zacc == (float) -1.6756368E38F);
                Debug.Assert(pack.time_usec == (ulong)8140611861464618207L);
                Debug.Assert(pack.zmag == (float) -1.2649654E38F);
                Debug.Assert(pack.temperature == (float)2.5707946E38F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.abs_pressure = (float)6.896849E36F;
            p105.yacc = (float)5.177128E37F;
            p105.xacc = (float)2.609662E37F;
            p105.ygyro = (float)2.8301117E38F;
            p105.fields_updated = (ushort)(ushort)21531;
            p105.pressure_alt = (float)3.3786947E38F;
            p105.diff_pressure = (float) -1.5495869E38F;
            p105.time_usec = (ulong)8140611861464618207L;
            p105.zgyro = (float)2.177123E38F;
            p105.zmag = (float) -1.2649654E38F;
            p105.zacc = (float) -1.6756368E38F;
            p105.temperature = (float)2.5707946E38F;
            p105.xgyro = (float)4.178377E37F;
            p105.ymag = (float)1.9429376E38F;
            p105.xmag = (float) -2.7336597E38F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.quality == (byte)(byte)240);
                Debug.Assert(pack.temperature == (short)(short) -23951);
                Debug.Assert(pack.integrated_x == (float)6.940557E37F);
                Debug.Assert(pack.integrated_xgyro == (float) -2.2704389E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)151);
                Debug.Assert(pack.time_usec == (ulong)2336961080975735122L);
                Debug.Assert(pack.integrated_ygyro == (float)2.6799422E38F);
                Debug.Assert(pack.integrated_zgyro == (float) -1.8638403E38F);
                Debug.Assert(pack.distance == (float) -6.328231E37F);
                Debug.Assert(pack.integration_time_us == (uint)1370060674U);
                Debug.Assert(pack.time_delta_distance_us == (uint)275145624U);
                Debug.Assert(pack.integrated_y == (float)3.3593554E38F);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.integrated_ygyro = (float)2.6799422E38F;
            p106.sensor_id = (byte)(byte)151;
            p106.time_usec = (ulong)2336961080975735122L;
            p106.time_delta_distance_us = (uint)275145624U;
            p106.integrated_y = (float)3.3593554E38F;
            p106.temperature = (short)(short) -23951;
            p106.distance = (float) -6.328231E37F;
            p106.integrated_xgyro = (float) -2.2704389E38F;
            p106.quality = (byte)(byte)240;
            p106.integration_time_us = (uint)1370060674U;
            p106.integrated_zgyro = (float) -1.8638403E38F;
            p106.integrated_x = (float)6.940557E37F;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (float)1.9270437E38F);
                Debug.Assert(pack.diff_pressure == (float) -2.522578E37F);
                Debug.Assert(pack.zgyro == (float) -1.1591332E38F);
                Debug.Assert(pack.abs_pressure == (float) -4.232824E37F);
                Debug.Assert(pack.yacc == (float)1.4902635E38F);
                Debug.Assert(pack.xacc == (float) -8.632598E36F);
                Debug.Assert(pack.time_usec == (ulong)1824370419453878322L);
                Debug.Assert(pack.xmag == (float)1.0612756E38F);
                Debug.Assert(pack.temperature == (float)3.2965029E38F);
                Debug.Assert(pack.xgyro == (float)6.70551E36F);
                Debug.Assert(pack.zmag == (float) -2.4293317E38F);
                Debug.Assert(pack.pressure_alt == (float)2.4582429E38F);
                Debug.Assert(pack.fields_updated == (uint)2767547991U);
                Debug.Assert(pack.ymag == (float)3.6874655E37F);
                Debug.Assert(pack.ygyro == (float)1.024607E38F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.diff_pressure = (float) -2.522578E37F;
            p107.temperature = (float)3.2965029E38F;
            p107.zmag = (float) -2.4293317E38F;
            p107.zacc = (float)1.9270437E38F;
            p107.abs_pressure = (float) -4.232824E37F;
            p107.fields_updated = (uint)2767547991U;
            p107.pressure_alt = (float)2.4582429E38F;
            p107.time_usec = (ulong)1824370419453878322L;
            p107.ymag = (float)3.6874655E37F;
            p107.xmag = (float)1.0612756E38F;
            p107.yacc = (float)1.4902635E38F;
            p107.zgyro = (float) -1.1591332E38F;
            p107.xgyro = (float)6.70551E36F;
            p107.xacc = (float) -8.632598E36F;
            p107.ygyro = (float)1.024607E38F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ve == (float)2.732281E38F);
                Debug.Assert(pack.pitch == (float)1.8229736E37F);
                Debug.Assert(pack.ygyro == (float) -1.0269921E38F);
                Debug.Assert(pack.roll == (float)2.906475E38F);
                Debug.Assert(pack.zacc == (float)2.944436E38F);
                Debug.Assert(pack.lat == (float)5.021522E37F);
                Debug.Assert(pack.xgyro == (float) -1.1634659E38F);
                Debug.Assert(pack.q2 == (float)2.290063E38F);
                Debug.Assert(pack.alt == (float) -5.3138057E37F);
                Debug.Assert(pack.q4 == (float) -2.094149E38F);
                Debug.Assert(pack.q1 == (float)8.3543656E37F);
                Debug.Assert(pack.std_dev_horz == (float) -1.5440523E38F);
                Debug.Assert(pack.xacc == (float)3.0811791E38F);
                Debug.Assert(pack.vd == (float) -6.12906E37F);
                Debug.Assert(pack.yacc == (float)2.2921273E38F);
                Debug.Assert(pack.vn == (float)1.9069899E37F);
                Debug.Assert(pack.std_dev_vert == (float) -2.587361E38F);
                Debug.Assert(pack.lon == (float) -3.0587314E38F);
                Debug.Assert(pack.yaw == (float) -1.5969076E37F);
                Debug.Assert(pack.zgyro == (float) -2.6880438E38F);
                Debug.Assert(pack.q3 == (float) -1.5681639E38F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.yacc = (float)2.2921273E38F;
            p108.q3 = (float) -1.5681639E38F;
            p108.q2 = (float)2.290063E38F;
            p108.zacc = (float)2.944436E38F;
            p108.xacc = (float)3.0811791E38F;
            p108.vd = (float) -6.12906E37F;
            p108.xgyro = (float) -1.1634659E38F;
            p108.lon = (float) -3.0587314E38F;
            p108.alt = (float) -5.3138057E37F;
            p108.q1 = (float)8.3543656E37F;
            p108.ve = (float)2.732281E38F;
            p108.zgyro = (float) -2.6880438E38F;
            p108.ygyro = (float) -1.0269921E38F;
            p108.std_dev_vert = (float) -2.587361E38F;
            p108.vn = (float)1.9069899E37F;
            p108.q4 = (float) -2.094149E38F;
            p108.std_dev_horz = (float) -1.5440523E38F;
            p108.pitch = (float)1.8229736E37F;
            p108.lat = (float)5.021522E37F;
            p108.roll = (float)2.906475E38F;
            p108.yaw = (float) -1.5969076E37F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fixed_ == (ushort)(ushort)14870);
                Debug.Assert(pack.rssi == (byte)(byte)28);
                Debug.Assert(pack.txbuf == (byte)(byte)254);
                Debug.Assert(pack.remnoise == (byte)(byte)160);
                Debug.Assert(pack.noise == (byte)(byte)111);
                Debug.Assert(pack.remrssi == (byte)(byte)205);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)19103);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.txbuf = (byte)(byte)254;
            p109.fixed_ = (ushort)(ushort)14870;
            p109.rssi = (byte)(byte)28;
            p109.rxerrors = (ushort)(ushort)19103;
            p109.noise = (byte)(byte)111;
            p109.remnoise = (byte)(byte)160;
            p109.remrssi = (byte)(byte)205;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)220);
                Debug.Assert(pack.target_network == (byte)(byte)108);
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)220, (byte)85, (byte)186, (byte)154, (byte)93, (byte)168, (byte)207, (byte)40, (byte)34, (byte)20, (byte)230, (byte)134, (byte)118, (byte)149, (byte)108, (byte)212, (byte)69, (byte)3, (byte)45, (byte)60, (byte)134, (byte)168, (byte)179, (byte)208, (byte)248, (byte)76, (byte)28, (byte)50, (byte)170, (byte)123, (byte)217, (byte)25, (byte)224, (byte)199, (byte)195, (byte)186, (byte)145, (byte)191, (byte)91, (byte)136, (byte)88, (byte)20, (byte)100, (byte)151, (byte)184, (byte)93, (byte)161, (byte)111, (byte)65, (byte)87, (byte)30, (byte)219, (byte)137, (byte)7, (byte)193, (byte)213, (byte)91, (byte)14, (byte)154, (byte)154, (byte)90, (byte)73, (byte)51, (byte)146, (byte)58, (byte)252, (byte)105, (byte)223, (byte)95, (byte)167, (byte)245, (byte)45, (byte)144, (byte)225, (byte)181, (byte)167, (byte)55, (byte)217, (byte)95, (byte)131, (byte)220, (byte)43, (byte)197, (byte)247, (byte)174, (byte)6, (byte)217, (byte)88, (byte)89, (byte)201, (byte)192, (byte)225, (byte)150, (byte)189, (byte)143, (byte)155, (byte)178, (byte)216, (byte)157, (byte)27, (byte)155, (byte)18, (byte)97, (byte)171, (byte)14, (byte)125, (byte)87, (byte)220, (byte)211, (byte)32, (byte)44, (byte)123, (byte)29, (byte)22, (byte)135, (byte)52, (byte)248, (byte)1, (byte)227, (byte)72, (byte)118, (byte)28, (byte)106, (byte)68, (byte)36, (byte)3, (byte)35, (byte)161, (byte)245, (byte)157, (byte)101, (byte)175, (byte)186, (byte)115, (byte)121, (byte)106, (byte)27, (byte)123, (byte)122, (byte)126, (byte)147, (byte)11, (byte)192, (byte)230, (byte)152, (byte)106, (byte)169, (byte)19, (byte)160, (byte)182, (byte)146, (byte)119, (byte)34, (byte)193, (byte)50, (byte)154, (byte)137, (byte)25, (byte)156, (byte)80, (byte)231, (byte)17, (byte)229, (byte)50, (byte)246, (byte)26, (byte)59, (byte)101, (byte)170, (byte)47, (byte)42, (byte)77, (byte)168, (byte)76, (byte)210, (byte)250, (byte)133, (byte)190, (byte)253, (byte)137, (byte)9, (byte)66, (byte)22, (byte)30, (byte)119, (byte)138, (byte)204, (byte)218, (byte)244, (byte)170, (byte)97, (byte)12, (byte)57, (byte)230, (byte)5, (byte)200, (byte)194, (byte)182, (byte)244, (byte)185, (byte)29, (byte)108, (byte)210, (byte)78, (byte)172, (byte)212, (byte)255, (byte)74, (byte)113, (byte)131, (byte)82, (byte)167, (byte)194, (byte)178, (byte)164, (byte)220, (byte)235, (byte)51, (byte)110, (byte)6, (byte)8, (byte)145, (byte)83, (byte)15, (byte)97, (byte)154, (byte)13, (byte)86, (byte)16, (byte)227, (byte)144, (byte)88, (byte)54, (byte)69, (byte)205, (byte)162, (byte)170, (byte)144, (byte)168, (byte)72, (byte)175, (byte)208, (byte)152, (byte)194, (byte)32, (byte)67, (byte)121, (byte)23, (byte)222, (byte)33, (byte)121}));
                Debug.Assert(pack.target_component == (byte)(byte)34);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_system = (byte)(byte)220;
            p110.target_component = (byte)(byte)34;
            p110.payload_SET(new byte[] {(byte)220, (byte)85, (byte)186, (byte)154, (byte)93, (byte)168, (byte)207, (byte)40, (byte)34, (byte)20, (byte)230, (byte)134, (byte)118, (byte)149, (byte)108, (byte)212, (byte)69, (byte)3, (byte)45, (byte)60, (byte)134, (byte)168, (byte)179, (byte)208, (byte)248, (byte)76, (byte)28, (byte)50, (byte)170, (byte)123, (byte)217, (byte)25, (byte)224, (byte)199, (byte)195, (byte)186, (byte)145, (byte)191, (byte)91, (byte)136, (byte)88, (byte)20, (byte)100, (byte)151, (byte)184, (byte)93, (byte)161, (byte)111, (byte)65, (byte)87, (byte)30, (byte)219, (byte)137, (byte)7, (byte)193, (byte)213, (byte)91, (byte)14, (byte)154, (byte)154, (byte)90, (byte)73, (byte)51, (byte)146, (byte)58, (byte)252, (byte)105, (byte)223, (byte)95, (byte)167, (byte)245, (byte)45, (byte)144, (byte)225, (byte)181, (byte)167, (byte)55, (byte)217, (byte)95, (byte)131, (byte)220, (byte)43, (byte)197, (byte)247, (byte)174, (byte)6, (byte)217, (byte)88, (byte)89, (byte)201, (byte)192, (byte)225, (byte)150, (byte)189, (byte)143, (byte)155, (byte)178, (byte)216, (byte)157, (byte)27, (byte)155, (byte)18, (byte)97, (byte)171, (byte)14, (byte)125, (byte)87, (byte)220, (byte)211, (byte)32, (byte)44, (byte)123, (byte)29, (byte)22, (byte)135, (byte)52, (byte)248, (byte)1, (byte)227, (byte)72, (byte)118, (byte)28, (byte)106, (byte)68, (byte)36, (byte)3, (byte)35, (byte)161, (byte)245, (byte)157, (byte)101, (byte)175, (byte)186, (byte)115, (byte)121, (byte)106, (byte)27, (byte)123, (byte)122, (byte)126, (byte)147, (byte)11, (byte)192, (byte)230, (byte)152, (byte)106, (byte)169, (byte)19, (byte)160, (byte)182, (byte)146, (byte)119, (byte)34, (byte)193, (byte)50, (byte)154, (byte)137, (byte)25, (byte)156, (byte)80, (byte)231, (byte)17, (byte)229, (byte)50, (byte)246, (byte)26, (byte)59, (byte)101, (byte)170, (byte)47, (byte)42, (byte)77, (byte)168, (byte)76, (byte)210, (byte)250, (byte)133, (byte)190, (byte)253, (byte)137, (byte)9, (byte)66, (byte)22, (byte)30, (byte)119, (byte)138, (byte)204, (byte)218, (byte)244, (byte)170, (byte)97, (byte)12, (byte)57, (byte)230, (byte)5, (byte)200, (byte)194, (byte)182, (byte)244, (byte)185, (byte)29, (byte)108, (byte)210, (byte)78, (byte)172, (byte)212, (byte)255, (byte)74, (byte)113, (byte)131, (byte)82, (byte)167, (byte)194, (byte)178, (byte)164, (byte)220, (byte)235, (byte)51, (byte)110, (byte)6, (byte)8, (byte)145, (byte)83, (byte)15, (byte)97, (byte)154, (byte)13, (byte)86, (byte)16, (byte)227, (byte)144, (byte)88, (byte)54, (byte)69, (byte)205, (byte)162, (byte)170, (byte)144, (byte)168, (byte)72, (byte)175, (byte)208, (byte)152, (byte)194, (byte)32, (byte)67, (byte)121, (byte)23, (byte)222, (byte)33, (byte)121}, 0) ;
            p110.target_network = (byte)(byte)108;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ts1 == (long)843658122359249400L);
                Debug.Assert(pack.tc1 == (long) -1446963093045924756L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long) -1446963093045924756L;
            p111.ts1 = (long)843658122359249400L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3942157645115312879L);
                Debug.Assert(pack.seq == (uint)2680602195U);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.time_usec = (ulong)3942157645115312879L;
            p112.seq = (uint)2680602195U;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ve == (short)(short)5956);
                Debug.Assert(pack.lat == (int) -1308188553);
                Debug.Assert(pack.vd == (short)(short)23475);
                Debug.Assert(pack.cog == (ushort)(ushort)64803);
                Debug.Assert(pack.time_usec == (ulong)3931750979498348156L);
                Debug.Assert(pack.fix_type == (byte)(byte)23);
                Debug.Assert(pack.lon == (int)69665335);
                Debug.Assert(pack.alt == (int)1602143219);
                Debug.Assert(pack.vel == (ushort)(ushort)29730);
                Debug.Assert(pack.satellites_visible == (byte)(byte)132);
                Debug.Assert(pack.eph == (ushort)(ushort)62354);
                Debug.Assert(pack.vn == (short)(short)22499);
                Debug.Assert(pack.epv == (ushort)(ushort)16851);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.ve = (short)(short)5956;
            p113.eph = (ushort)(ushort)62354;
            p113.vn = (short)(short)22499;
            p113.cog = (ushort)(ushort)64803;
            p113.vd = (short)(short)23475;
            p113.epv = (ushort)(ushort)16851;
            p113.alt = (int)1602143219;
            p113.vel = (ushort)(ushort)29730;
            p113.satellites_visible = (byte)(byte)132;
            p113.lat = (int) -1308188553;
            p113.fix_type = (byte)(byte)23;
            p113.time_usec = (ulong)3931750979498348156L;
            p113.lon = (int)69665335;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_zgyro == (float)1.917758E38F);
                Debug.Assert(pack.integration_time_us == (uint)3187521957U);
                Debug.Assert(pack.distance == (float) -5.206934E37F);
                Debug.Assert(pack.sensor_id == (byte)(byte)252);
                Debug.Assert(pack.integrated_x == (float) -1.2784461E38F);
                Debug.Assert(pack.time_usec == (ulong)3451282218526715039L);
                Debug.Assert(pack.integrated_y == (float)3.0658122E38F);
                Debug.Assert(pack.temperature == (short)(short)15873);
                Debug.Assert(pack.integrated_xgyro == (float)7.1827137E37F);
                Debug.Assert(pack.time_delta_distance_us == (uint)3115365334U);
                Debug.Assert(pack.quality == (byte)(byte)102);
                Debug.Assert(pack.integrated_ygyro == (float)3.037429E38F);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.integrated_zgyro = (float)1.917758E38F;
            p114.integration_time_us = (uint)3187521957U;
            p114.integrated_y = (float)3.0658122E38F;
            p114.integrated_x = (float) -1.2784461E38F;
            p114.sensor_id = (byte)(byte)252;
            p114.integrated_ygyro = (float)3.037429E38F;
            p114.time_delta_distance_us = (uint)3115365334U;
            p114.temperature = (short)(short)15873;
            p114.quality = (byte)(byte)102;
            p114.time_usec = (ulong)3451282218526715039L;
            p114.distance = (float) -5.206934E37F;
            p114.integrated_xgyro = (float)7.1827137E37F;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (short)(short)793);
                Debug.Assert(pack.pitchspeed == (float) -2.438522E38F);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)41739);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)8862);
                Debug.Assert(pack.time_usec == (ulong)3999550841153741892L);
                Debug.Assert(pack.yacc == (short)(short) -6065);
                Debug.Assert(pack.lon == (int)717428329);
                Debug.Assert(pack.yawspeed == (float) -5.419717E37F);
                Debug.Assert(pack.vz == (short)(short) -20150);
                Debug.Assert(pack.rollspeed == (float) -3.2757844E38F);
                Debug.Assert(pack.lat == (int)1002485553);
                Debug.Assert(pack.xacc == (short)(short) -8382);
                Debug.Assert(pack.vy == (short)(short)81);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {-2.8745456E38F, -7.24151E37F, 2.1055095E37F, 2.1100062E38F}));
                Debug.Assert(pack.alt == (int) -779678644);
                Debug.Assert(pack.zacc == (short)(short)10390);
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.lon = (int)717428329;
            p115.time_usec = (ulong)3999550841153741892L;
            p115.vz = (short)(short) -20150;
            p115.true_airspeed = (ushort)(ushort)41739;
            p115.ind_airspeed = (ushort)(ushort)8862;
            p115.pitchspeed = (float) -2.438522E38F;
            p115.rollspeed = (float) -3.2757844E38F;
            p115.xacc = (short)(short) -8382;
            p115.zacc = (short)(short)10390;
            p115.lat = (int)1002485553;
            p115.vy = (short)(short)81;
            p115.alt = (int) -779678644;
            p115.attitude_quaternion_SET(new float[] {-2.8745456E38F, -7.24151E37F, 2.1055095E37F, 2.1100062E38F}, 0) ;
            p115.yawspeed = (float) -5.419717E37F;
            p115.vx = (short)(short)793;
            p115.yacc = (short)(short) -6065;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (short)(short) -28193);
                Debug.Assert(pack.xacc == (short)(short)18525);
                Debug.Assert(pack.yacc == (short)(short)6632);
                Debug.Assert(pack.xgyro == (short)(short)15547);
                Debug.Assert(pack.zgyro == (short)(short) -16385);
                Debug.Assert(pack.zmag == (short)(short) -25375);
                Debug.Assert(pack.time_boot_ms == (uint)2557198869U);
                Debug.Assert(pack.zacc == (short)(short) -13747);
                Debug.Assert(pack.xmag == (short)(short) -25459);
                Debug.Assert(pack.ymag == (short)(short) -10507);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.zgyro = (short)(short) -16385;
            p116.xmag = (short)(short) -25459;
            p116.time_boot_ms = (uint)2557198869U;
            p116.ygyro = (short)(short) -28193;
            p116.xgyro = (short)(short)15547;
            p116.yacc = (short)(short)6632;
            p116.zacc = (short)(short) -13747;
            p116.xacc = (short)(short)18525;
            p116.zmag = (short)(short) -25375;
            p116.ymag = (short)(short) -10507;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start == (ushort)(ushort)11648);
                Debug.Assert(pack.end == (ushort)(ushort)2117);
                Debug.Assert(pack.target_component == (byte)(byte)115);
                Debug.Assert(pack.target_system == (byte)(byte)107);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.end = (ushort)(ushort)2117;
            p117.target_system = (byte)(byte)107;
            p117.target_component = (byte)(byte)115;
            p117.start = (ushort)(ushort)11648;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_log_num == (ushort)(ushort)16870);
                Debug.Assert(pack.size == (uint)2273459559U);
                Debug.Assert(pack.time_utc == (uint)569127122U);
                Debug.Assert(pack.num_logs == (ushort)(ushort)37141);
                Debug.Assert(pack.id == (ushort)(ushort)22748);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.id = (ushort)(ushort)22748;
            p118.num_logs = (ushort)(ushort)37141;
            p118.size = (uint)2273459559U;
            p118.last_log_num = (ushort)(ushort)16870;
            p118.time_utc = (uint)569127122U;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)113);
                Debug.Assert(pack.ofs == (uint)1346031486U);
                Debug.Assert(pack.target_system == (byte)(byte)157);
                Debug.Assert(pack.count == (uint)2244149751U);
                Debug.Assert(pack.id == (ushort)(ushort)29036);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.target_system = (byte)(byte)157;
            p119.target_component = (byte)(byte)113;
            p119.ofs = (uint)1346031486U;
            p119.count = (uint)2244149751U;
            p119.id = (ushort)(ushort)29036;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.count == (byte)(byte)250);
                Debug.Assert(pack.ofs == (uint)3572763300U);
                Debug.Assert(pack.id == (ushort)(ushort)29722);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)235, (byte)134, (byte)66, (byte)163, (byte)38, (byte)15, (byte)79, (byte)174, (byte)119, (byte)153, (byte)37, (byte)190, (byte)123, (byte)227, (byte)197, (byte)149, (byte)105, (byte)104, (byte)5, (byte)50, (byte)1, (byte)112, (byte)122, (byte)12, (byte)95, (byte)9, (byte)208, (byte)75, (byte)98, (byte)196, (byte)248, (byte)89, (byte)63, (byte)122, (byte)26, (byte)86, (byte)15, (byte)254, (byte)18, (byte)139, (byte)209, (byte)232, (byte)38, (byte)147, (byte)6, (byte)60, (byte)65, (byte)137, (byte)79, (byte)174, (byte)75, (byte)204, (byte)178, (byte)141, (byte)220, (byte)38, (byte)40, (byte)183, (byte)35, (byte)206, (byte)233, (byte)28, (byte)148, (byte)210, (byte)151, (byte)43, (byte)57, (byte)44, (byte)176, (byte)53, (byte)43, (byte)131, (byte)69, (byte)94, (byte)182, (byte)216, (byte)225, (byte)239, (byte)3, (byte)164, (byte)237, (byte)45, (byte)144, (byte)111, (byte)151, (byte)166, (byte)92, (byte)44, (byte)36, (byte)122}));
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.count = (byte)(byte)250;
            p120.data__SET(new byte[] {(byte)235, (byte)134, (byte)66, (byte)163, (byte)38, (byte)15, (byte)79, (byte)174, (byte)119, (byte)153, (byte)37, (byte)190, (byte)123, (byte)227, (byte)197, (byte)149, (byte)105, (byte)104, (byte)5, (byte)50, (byte)1, (byte)112, (byte)122, (byte)12, (byte)95, (byte)9, (byte)208, (byte)75, (byte)98, (byte)196, (byte)248, (byte)89, (byte)63, (byte)122, (byte)26, (byte)86, (byte)15, (byte)254, (byte)18, (byte)139, (byte)209, (byte)232, (byte)38, (byte)147, (byte)6, (byte)60, (byte)65, (byte)137, (byte)79, (byte)174, (byte)75, (byte)204, (byte)178, (byte)141, (byte)220, (byte)38, (byte)40, (byte)183, (byte)35, (byte)206, (byte)233, (byte)28, (byte)148, (byte)210, (byte)151, (byte)43, (byte)57, (byte)44, (byte)176, (byte)53, (byte)43, (byte)131, (byte)69, (byte)94, (byte)182, (byte)216, (byte)225, (byte)239, (byte)3, (byte)164, (byte)237, (byte)45, (byte)144, (byte)111, (byte)151, (byte)166, (byte)92, (byte)44, (byte)36, (byte)122}, 0) ;
            p120.ofs = (uint)3572763300U;
            p120.id = (ushort)(ushort)29722;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)230);
                Debug.Assert(pack.target_component == (byte)(byte)219);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_system = (byte)(byte)230;
            p121.target_component = (byte)(byte)219;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)164);
                Debug.Assert(pack.target_system == (byte)(byte)179);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_system = (byte)(byte)179;
            p122.target_component = (byte)(byte)164;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)59);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)37, (byte)81, (byte)103, (byte)82, (byte)228, (byte)51, (byte)208, (byte)137, (byte)20, (byte)38, (byte)52, (byte)222, (byte)60, (byte)142, (byte)42, (byte)125, (byte)151, (byte)119, (byte)27, (byte)54, (byte)39, (byte)199, (byte)71, (byte)238, (byte)167, (byte)102, (byte)70, (byte)127, (byte)71, (byte)101, (byte)212, (byte)12, (byte)148, (byte)81, (byte)56, (byte)32, (byte)45, (byte)210, (byte)5, (byte)146, (byte)208, (byte)1, (byte)76, (byte)119, (byte)122, (byte)138, (byte)218, (byte)138, (byte)167, (byte)102, (byte)12, (byte)33, (byte)1, (byte)83, (byte)228, (byte)17, (byte)19, (byte)15, (byte)8, (byte)28, (byte)105, (byte)76, (byte)195, (byte)116, (byte)120, (byte)230, (byte)3, (byte)146, (byte)119, (byte)1, (byte)16, (byte)240, (byte)3, (byte)24, (byte)21, (byte)8, (byte)59, (byte)6, (byte)39, (byte)74, (byte)227, (byte)221, (byte)222, (byte)83, (byte)76, (byte)80, (byte)65, (byte)188, (byte)103, (byte)15, (byte)249, (byte)145, (byte)177, (byte)69, (byte)240, (byte)83, (byte)202, (byte)209, (byte)181, (byte)23, (byte)129, (byte)52, (byte)47, (byte)171, (byte)140, (byte)215, (byte)121, (byte)137, (byte)238, (byte)45}));
                Debug.Assert(pack.target_system == (byte)(byte)97);
                Debug.Assert(pack.len == (byte)(byte)100);
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.data__SET(new byte[] {(byte)37, (byte)81, (byte)103, (byte)82, (byte)228, (byte)51, (byte)208, (byte)137, (byte)20, (byte)38, (byte)52, (byte)222, (byte)60, (byte)142, (byte)42, (byte)125, (byte)151, (byte)119, (byte)27, (byte)54, (byte)39, (byte)199, (byte)71, (byte)238, (byte)167, (byte)102, (byte)70, (byte)127, (byte)71, (byte)101, (byte)212, (byte)12, (byte)148, (byte)81, (byte)56, (byte)32, (byte)45, (byte)210, (byte)5, (byte)146, (byte)208, (byte)1, (byte)76, (byte)119, (byte)122, (byte)138, (byte)218, (byte)138, (byte)167, (byte)102, (byte)12, (byte)33, (byte)1, (byte)83, (byte)228, (byte)17, (byte)19, (byte)15, (byte)8, (byte)28, (byte)105, (byte)76, (byte)195, (byte)116, (byte)120, (byte)230, (byte)3, (byte)146, (byte)119, (byte)1, (byte)16, (byte)240, (byte)3, (byte)24, (byte)21, (byte)8, (byte)59, (byte)6, (byte)39, (byte)74, (byte)227, (byte)221, (byte)222, (byte)83, (byte)76, (byte)80, (byte)65, (byte)188, (byte)103, (byte)15, (byte)249, (byte)145, (byte)177, (byte)69, (byte)240, (byte)83, (byte)202, (byte)209, (byte)181, (byte)23, (byte)129, (byte)52, (byte)47, (byte)171, (byte)140, (byte)215, (byte)121, (byte)137, (byte)238, (byte)45}, 0) ;
            p123.target_component = (byte)(byte)59;
            p123.len = (byte)(byte)100;
            p123.target_system = (byte)(byte)97;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)201490882);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
                Debug.Assert(pack.dgps_numch == (byte)(byte)156);
                Debug.Assert(pack.eph == (ushort)(ushort)27766);
                Debug.Assert(pack.satellites_visible == (byte)(byte)188);
                Debug.Assert(pack.lon == (int)1018254806);
                Debug.Assert(pack.time_usec == (ulong)8754230396296442440L);
                Debug.Assert(pack.dgps_age == (uint)3211940805U);
                Debug.Assert(pack.vel == (ushort)(ushort)18610);
                Debug.Assert(pack.alt == (int) -1127699267);
                Debug.Assert(pack.epv == (ushort)(ushort)52289);
                Debug.Assert(pack.cog == (ushort)(ushort)30943);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.cog = (ushort)(ushort)30943;
            p124.dgps_age = (uint)3211940805U;
            p124.alt = (int) -1127699267;
            p124.eph = (ushort)(ushort)27766;
            p124.satellites_visible = (byte)(byte)188;
            p124.epv = (ushort)(ushort)52289;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
            p124.lon = (int)1018254806;
            p124.lat = (int)201490882;
            p124.dgps_numch = (byte)(byte)156;
            p124.time_usec = (ulong)8754230396296442440L;
            p124.vel = (ushort)(ushort)18610;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vcc == (ushort)(ushort)47941);
                Debug.Assert(pack.Vservo == (ushort)(ushort)2747);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID));
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vcc = (ushort)(ushort)47941;
            p125.Vservo = (ushort)(ushort)2747;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID);
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.device == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
                Debug.Assert(pack.baudrate == (uint)4012147306U);
                Debug.Assert(pack.count == (byte)(byte)133);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)34, (byte)162, (byte)198, (byte)58, (byte)223, (byte)226, (byte)254, (byte)85, (byte)97, (byte)31, (byte)139, (byte)242, (byte)1, (byte)54, (byte)205, (byte)101, (byte)230, (byte)8, (byte)75, (byte)138, (byte)164, (byte)112, (byte)248, (byte)122, (byte)24, (byte)87, (byte)138, (byte)133, (byte)159, (byte)25, (byte)142, (byte)208, (byte)70, (byte)199, (byte)186, (byte)147, (byte)105, (byte)195, (byte)101, (byte)7, (byte)110, (byte)97, (byte)100, (byte)17, (byte)221, (byte)213, (byte)155, (byte)32, (byte)109, (byte)169, (byte)67, (byte)24, (byte)147, (byte)166, (byte)182, (byte)108, (byte)102, (byte)253, (byte)167, (byte)104, (byte)98, (byte)217, (byte)234, (byte)177, (byte)110, (byte)169, (byte)207, (byte)220, (byte)166, (byte)104}));
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY));
                Debug.Assert(pack.timeout == (ushort)(ushort)6587);
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.data__SET(new byte[] {(byte)34, (byte)162, (byte)198, (byte)58, (byte)223, (byte)226, (byte)254, (byte)85, (byte)97, (byte)31, (byte)139, (byte)242, (byte)1, (byte)54, (byte)205, (byte)101, (byte)230, (byte)8, (byte)75, (byte)138, (byte)164, (byte)112, (byte)248, (byte)122, (byte)24, (byte)87, (byte)138, (byte)133, (byte)159, (byte)25, (byte)142, (byte)208, (byte)70, (byte)199, (byte)186, (byte)147, (byte)105, (byte)195, (byte)101, (byte)7, (byte)110, (byte)97, (byte)100, (byte)17, (byte)221, (byte)213, (byte)155, (byte)32, (byte)109, (byte)169, (byte)67, (byte)24, (byte)147, (byte)166, (byte)182, (byte)108, (byte)102, (byte)253, (byte)167, (byte)104, (byte)98, (byte)217, (byte)234, (byte)177, (byte)110, (byte)169, (byte)207, (byte)220, (byte)166, (byte)104}, 0) ;
            p126.baudrate = (uint)4012147306U;
            p126.timeout = (ushort)(ushort)6587;
            p126.count = (byte)(byte)133;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY);
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rtk_rate == (byte)(byte)42);
                Debug.Assert(pack.rtk_health == (byte)(byte)138);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)183);
                Debug.Assert(pack.nsats == (byte)(byte)181);
                Debug.Assert(pack.wn == (ushort)(ushort)33911);
                Debug.Assert(pack.iar_num_hypotheses == (int) -767047171);
                Debug.Assert(pack.baseline_a_mm == (int)149109758);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)234);
                Debug.Assert(pack.accuracy == (uint)267631542U);
                Debug.Assert(pack.baseline_c_mm == (int) -729905750);
                Debug.Assert(pack.baseline_b_mm == (int) -392777503);
                Debug.Assert(pack.time_last_baseline_ms == (uint)4047737379U);
                Debug.Assert(pack.tow == (uint)4166787457U);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.rtk_rate = (byte)(byte)42;
            p127.baseline_b_mm = (int) -392777503;
            p127.nsats = (byte)(byte)181;
            p127.rtk_health = (byte)(byte)138;
            p127.tow = (uint)4166787457U;
            p127.time_last_baseline_ms = (uint)4047737379U;
            p127.baseline_a_mm = (int)149109758;
            p127.accuracy = (uint)267631542U;
            p127.iar_num_hypotheses = (int) -767047171;
            p127.rtk_receiver_id = (byte)(byte)234;
            p127.baseline_coords_type = (byte)(byte)183;
            p127.wn = (ushort)(ushort)33911;
            p127.baseline_c_mm = (int) -729905750;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wn == (ushort)(ushort)23565);
                Debug.Assert(pack.accuracy == (uint)1608167784U);
                Debug.Assert(pack.rtk_health == (byte)(byte)106);
                Debug.Assert(pack.baseline_b_mm == (int) -1775624229);
                Debug.Assert(pack.baseline_a_mm == (int) -1584391571);
                Debug.Assert(pack.tow == (uint)62702347U);
                Debug.Assert(pack.rtk_rate == (byte)(byte)87);
                Debug.Assert(pack.baseline_c_mm == (int)171063088);
                Debug.Assert(pack.time_last_baseline_ms == (uint)3787483093U);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)247);
                Debug.Assert(pack.iar_num_hypotheses == (int)1615870905);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)68);
                Debug.Assert(pack.nsats == (byte)(byte)70);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.baseline_a_mm = (int) -1584391571;
            p128.tow = (uint)62702347U;
            p128.time_last_baseline_ms = (uint)3787483093U;
            p128.baseline_coords_type = (byte)(byte)247;
            p128.rtk_receiver_id = (byte)(byte)68;
            p128.nsats = (byte)(byte)70;
            p128.iar_num_hypotheses = (int)1615870905;
            p128.rtk_rate = (byte)(byte)87;
            p128.baseline_b_mm = (int) -1775624229;
            p128.baseline_c_mm = (int)171063088;
            p128.accuracy = (uint)1608167784U;
            p128.rtk_health = (byte)(byte)106;
            p128.wn = (ushort)(ushort)23565;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xmag == (short)(short) -4664);
                Debug.Assert(pack.zacc == (short)(short) -20149);
                Debug.Assert(pack.xgyro == (short)(short)5649);
                Debug.Assert(pack.xacc == (short)(short)21945);
                Debug.Assert(pack.ymag == (short)(short)5663);
                Debug.Assert(pack.yacc == (short)(short) -726);
                Debug.Assert(pack.time_boot_ms == (uint)2600222238U);
                Debug.Assert(pack.zmag == (short)(short) -816);
                Debug.Assert(pack.zgyro == (short)(short)20214);
                Debug.Assert(pack.ygyro == (short)(short) -25416);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.zgyro = (short)(short)20214;
            p129.xgyro = (short)(short)5649;
            p129.yacc = (short)(short) -726;
            p129.time_boot_ms = (uint)2600222238U;
            p129.zacc = (short)(short) -20149;
            p129.ymag = (short)(short)5663;
            p129.zmag = (short)(short) -816;
            p129.ygyro = (short)(short) -25416;
            p129.xmag = (short)(short) -4664;
            p129.xacc = (short)(short)21945;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.height == (ushort)(ushort)58786);
                Debug.Assert(pack.payload == (byte)(byte)34);
                Debug.Assert(pack.type == (byte)(byte)215);
                Debug.Assert(pack.jpg_quality == (byte)(byte)105);
                Debug.Assert(pack.size == (uint)3212435537U);
                Debug.Assert(pack.width == (ushort)(ushort)39808);
                Debug.Assert(pack.packets == (ushort)(ushort)3254);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.packets = (ushort)(ushort)3254;
            p130.payload = (byte)(byte)34;
            p130.jpg_quality = (byte)(byte)105;
            p130.width = (ushort)(ushort)39808;
            p130.size = (uint)3212435537U;
            p130.height = (ushort)(ushort)58786;
            p130.type = (byte)(byte)215;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seqnr == (ushort)(ushort)59528);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)172, (byte)164, (byte)242, (byte)205, (byte)119, (byte)211, (byte)50, (byte)117, (byte)223, (byte)44, (byte)3, (byte)33, (byte)138, (byte)38, (byte)21, (byte)35, (byte)240, (byte)239, (byte)20, (byte)133, (byte)11, (byte)170, (byte)8, (byte)124, (byte)61, (byte)74, (byte)225, (byte)233, (byte)239, (byte)30, (byte)207, (byte)99, (byte)48, (byte)114, (byte)150, (byte)102, (byte)83, (byte)187, (byte)174, (byte)173, (byte)142, (byte)44, (byte)130, (byte)144, (byte)125, (byte)241, (byte)111, (byte)224, (byte)120, (byte)158, (byte)119, (byte)221, (byte)115, (byte)224, (byte)210, (byte)110, (byte)182, (byte)75, (byte)42, (byte)126, (byte)80, (byte)183, (byte)76, (byte)42, (byte)188, (byte)102, (byte)66, (byte)49, (byte)164, (byte)175, (byte)161, (byte)34, (byte)123, (byte)133, (byte)86, (byte)98, (byte)42, (byte)253, (byte)2, (byte)31, (byte)55, (byte)104, (byte)63, (byte)120, (byte)66, (byte)91, (byte)98, (byte)217, (byte)129, (byte)66, (byte)57, (byte)195, (byte)238, (byte)204, (byte)160, (byte)66, (byte)13, (byte)220, (byte)159, (byte)119, (byte)119, (byte)38, (byte)15, (byte)103, (byte)82, (byte)122, (byte)171, (byte)251, (byte)97, (byte)146, (byte)24, (byte)232, (byte)11, (byte)115, (byte)125, (byte)37, (byte)218, (byte)86, (byte)191, (byte)209, (byte)17, (byte)8, (byte)206, (byte)214, (byte)18, (byte)169, (byte)159, (byte)206, (byte)30, (byte)248, (byte)125, (byte)127, (byte)148, (byte)205, (byte)208, (byte)92, (byte)94, (byte)211, (byte)171, (byte)211, (byte)235, (byte)13, (byte)253, (byte)82, (byte)42, (byte)89, (byte)196, (byte)19, (byte)128, (byte)207, (byte)144, (byte)174, (byte)237, (byte)76, (byte)69, (byte)88, (byte)199, (byte)57, (byte)122, (byte)225, (byte)158, (byte)155, (byte)193, (byte)62, (byte)165, (byte)221, (byte)31, (byte)163, (byte)42, (byte)65, (byte)54, (byte)79, (byte)12, (byte)11, (byte)28, (byte)52, (byte)180, (byte)63, (byte)56, (byte)108, (byte)129, (byte)171, (byte)106, (byte)239, (byte)150, (byte)243, (byte)4, (byte)168, (byte)235, (byte)243, (byte)91, (byte)196, (byte)45, (byte)3, (byte)82, (byte)127, (byte)72, (byte)216, (byte)22, (byte)38, (byte)33, (byte)138, (byte)27, (byte)114, (byte)155, (byte)62, (byte)84, (byte)180, (byte)203, (byte)231, (byte)186, (byte)56, (byte)84, (byte)73, (byte)182, (byte)23, (byte)171, (byte)110, (byte)92, (byte)147, (byte)124, (byte)150, (byte)20, (byte)62, (byte)155, (byte)57, (byte)224, (byte)199, (byte)94, (byte)71, (byte)75, (byte)7, (byte)174, (byte)234, (byte)145, (byte)77, (byte)171, (byte)39, (byte)66, (byte)108, (byte)160, (byte)229, (byte)80, (byte)32, (byte)244, (byte)254, (byte)216, (byte)11, (byte)185, (byte)254, (byte)30, (byte)4, (byte)83}));
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)59528;
            p131.data__SET(new byte[] {(byte)172, (byte)164, (byte)242, (byte)205, (byte)119, (byte)211, (byte)50, (byte)117, (byte)223, (byte)44, (byte)3, (byte)33, (byte)138, (byte)38, (byte)21, (byte)35, (byte)240, (byte)239, (byte)20, (byte)133, (byte)11, (byte)170, (byte)8, (byte)124, (byte)61, (byte)74, (byte)225, (byte)233, (byte)239, (byte)30, (byte)207, (byte)99, (byte)48, (byte)114, (byte)150, (byte)102, (byte)83, (byte)187, (byte)174, (byte)173, (byte)142, (byte)44, (byte)130, (byte)144, (byte)125, (byte)241, (byte)111, (byte)224, (byte)120, (byte)158, (byte)119, (byte)221, (byte)115, (byte)224, (byte)210, (byte)110, (byte)182, (byte)75, (byte)42, (byte)126, (byte)80, (byte)183, (byte)76, (byte)42, (byte)188, (byte)102, (byte)66, (byte)49, (byte)164, (byte)175, (byte)161, (byte)34, (byte)123, (byte)133, (byte)86, (byte)98, (byte)42, (byte)253, (byte)2, (byte)31, (byte)55, (byte)104, (byte)63, (byte)120, (byte)66, (byte)91, (byte)98, (byte)217, (byte)129, (byte)66, (byte)57, (byte)195, (byte)238, (byte)204, (byte)160, (byte)66, (byte)13, (byte)220, (byte)159, (byte)119, (byte)119, (byte)38, (byte)15, (byte)103, (byte)82, (byte)122, (byte)171, (byte)251, (byte)97, (byte)146, (byte)24, (byte)232, (byte)11, (byte)115, (byte)125, (byte)37, (byte)218, (byte)86, (byte)191, (byte)209, (byte)17, (byte)8, (byte)206, (byte)214, (byte)18, (byte)169, (byte)159, (byte)206, (byte)30, (byte)248, (byte)125, (byte)127, (byte)148, (byte)205, (byte)208, (byte)92, (byte)94, (byte)211, (byte)171, (byte)211, (byte)235, (byte)13, (byte)253, (byte)82, (byte)42, (byte)89, (byte)196, (byte)19, (byte)128, (byte)207, (byte)144, (byte)174, (byte)237, (byte)76, (byte)69, (byte)88, (byte)199, (byte)57, (byte)122, (byte)225, (byte)158, (byte)155, (byte)193, (byte)62, (byte)165, (byte)221, (byte)31, (byte)163, (byte)42, (byte)65, (byte)54, (byte)79, (byte)12, (byte)11, (byte)28, (byte)52, (byte)180, (byte)63, (byte)56, (byte)108, (byte)129, (byte)171, (byte)106, (byte)239, (byte)150, (byte)243, (byte)4, (byte)168, (byte)235, (byte)243, (byte)91, (byte)196, (byte)45, (byte)3, (byte)82, (byte)127, (byte)72, (byte)216, (byte)22, (byte)38, (byte)33, (byte)138, (byte)27, (byte)114, (byte)155, (byte)62, (byte)84, (byte)180, (byte)203, (byte)231, (byte)186, (byte)56, (byte)84, (byte)73, (byte)182, (byte)23, (byte)171, (byte)110, (byte)92, (byte)147, (byte)124, (byte)150, (byte)20, (byte)62, (byte)155, (byte)57, (byte)224, (byte)199, (byte)94, (byte)71, (byte)75, (byte)7, (byte)174, (byte)234, (byte)145, (byte)77, (byte)171, (byte)39, (byte)66, (byte)108, (byte)160, (byte)229, (byte)80, (byte)32, (byte)244, (byte)254, (byte)216, (byte)11, (byte)185, (byte)254, (byte)30, (byte)4, (byte)83}, 0) ;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_distance == (ushort)(ushort)43253);
                Debug.Assert(pack.max_distance == (ushort)(ushort)55711);
                Debug.Assert(pack.time_boot_ms == (uint)204252413U);
                Debug.Assert(pack.min_distance == (ushort)(ushort)4391);
                Debug.Assert(pack.covariance == (byte)(byte)165);
                Debug.Assert(pack.type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
                Debug.Assert(pack.orientation == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_135);
                Debug.Assert(pack.id == (byte)(byte)84);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR;
            p132.id = (byte)(byte)84;
            p132.covariance = (byte)(byte)165;
            p132.current_distance = (ushort)(ushort)43253;
            p132.time_boot_ms = (uint)204252413U;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_135;
            p132.max_distance = (ushort)(ushort)55711;
            p132.min_distance = (ushort)(ushort)4391;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -957396210);
                Debug.Assert(pack.lon == (int)688462460);
                Debug.Assert(pack.mask == (ulong)8916280874202878179L);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)11520);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.mask = (ulong)8916280874202878179L;
            p133.grid_spacing = (ushort)(ushort)11520;
            p133.lat = (int) -957396210;
            p133.lon = (int)688462460;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)15409);
                Debug.Assert(pack.lon == (int) -2134155146);
                Debug.Assert(pack.lat == (int) -303719040);
                Debug.Assert(pack.gridbit == (byte)(byte)175);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -21337, (short)30337, (short) -28193, (short)27380, (short) -11983, (short)31371, (short) -5480, (short)7018, (short)27327, (short)8842, (short)9529, (short)27915, (short)19873, (short) -14944, (short)26437, (short)10037}));
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.grid_spacing = (ushort)(ushort)15409;
            p134.lat = (int) -303719040;
            p134.data__SET(new short[] {(short) -21337, (short)30337, (short) -28193, (short)27380, (short) -11983, (short)31371, (short) -5480, (short)7018, (short)27327, (short)8842, (short)9529, (short)27915, (short)19873, (short) -14944, (short)26437, (short)10037}, 0) ;
            p134.gridbit = (byte)(byte)175;
            p134.lon = (int) -2134155146;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -1602554992);
                Debug.Assert(pack.lon == (int)1672779949);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lon = (int)1672779949;
            p135.lat = (int) -1602554992;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pending == (ushort)(ushort)14216);
                Debug.Assert(pack.current_height == (float) -2.3475522E38F);
                Debug.Assert(pack.lat == (int)1180875759);
                Debug.Assert(pack.lon == (int) -1784148066);
                Debug.Assert(pack.loaded == (ushort)(ushort)46527);
                Debug.Assert(pack.spacing == (ushort)(ushort)1341);
                Debug.Assert(pack.terrain_height == (float) -2.446351E37F);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.pending = (ushort)(ushort)14216;
            p136.lon = (int) -1784148066;
            p136.lat = (int)1180875759;
            p136.spacing = (ushort)(ushort)1341;
            p136.current_height = (float) -2.3475522E38F;
            p136.terrain_height = (float) -2.446351E37F;
            p136.loaded = (ushort)(ushort)46527;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float) -2.3986606E38F);
                Debug.Assert(pack.temperature == (short)(short)25341);
                Debug.Assert(pack.time_boot_ms == (uint)4057913821U);
                Debug.Assert(pack.press_abs == (float) -1.8213626E38F);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.press_abs = (float) -1.8213626E38F;
            p137.time_boot_ms = (uint)4057913821U;
            p137.press_diff = (float) -2.3986606E38F;
            p137.temperature = (short)(short)25341;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.611432E38F, -3.1234951E38F, 2.4953426E38F, 1.886388E38F}));
                Debug.Assert(pack.z == (float)2.9264803E38F);
                Debug.Assert(pack.y == (float) -1.9644153E38F);
                Debug.Assert(pack.x == (float)1.416656E38F);
                Debug.Assert(pack.time_usec == (ulong)798079903902286385L);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.x = (float)1.416656E38F;
            p138.z = (float)2.9264803E38F;
            p138.y = (float) -1.9644153E38F;
            p138.time_usec = (ulong)798079903902286385L;
            p138.q_SET(new float[] {-2.611432E38F, -3.1234951E38F, 2.4953426E38F, 1.886388E38F}, 0) ;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.2799589E38F, 2.6768913E38F, 2.9256083E37F, -1.0701191E38F, 3.2895726E38F, 8.837452E37F, -4.158639E37F, -8.664696E37F}));
                Debug.Assert(pack.target_component == (byte)(byte)50);
                Debug.Assert(pack.target_system == (byte)(byte)63);
                Debug.Assert(pack.time_usec == (ulong)3110917761652043385L);
                Debug.Assert(pack.group_mlx == (byte)(byte)249);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.time_usec = (ulong)3110917761652043385L;
            p139.group_mlx = (byte)(byte)249;
            p139.target_component = (byte)(byte)50;
            p139.controls_SET(new float[] {-1.2799589E38F, 2.6768913E38F, 2.9256083E37F, -1.0701191E38F, 3.2895726E38F, 8.837452E37F, -4.158639E37F, -8.664696E37F}, 0) ;
            p139.target_system = (byte)(byte)63;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {2.383176E38F, 6.2104906E37F, 2.8861E38F, 1.272881E38F, 5.7398144E37F, 1.8635944E38F, 2.4273002E38F, -4.096482E36F}));
                Debug.Assert(pack.group_mlx == (byte)(byte)73);
                Debug.Assert(pack.time_usec == (ulong)7001058245241879038L);
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.controls_SET(new float[] {2.383176E38F, 6.2104906E37F, 2.8861E38F, 1.272881E38F, 5.7398144E37F, 1.8635944E38F, 2.4273002E38F, -4.096482E36F}, 0) ;
            p140.group_mlx = (byte)(byte)73;
            p140.time_usec = (ulong)7001058245241879038L;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_relative == (float) -3.1340698E38F);
                Debug.Assert(pack.altitude_monotonic == (float) -2.5858612E38F);
                Debug.Assert(pack.time_usec == (ulong)1203375217083852440L);
                Debug.Assert(pack.altitude_local == (float) -1.2779467E38F);
                Debug.Assert(pack.bottom_clearance == (float)1.6597611E38F);
                Debug.Assert(pack.altitude_terrain == (float) -3.3483686E38F);
                Debug.Assert(pack.altitude_amsl == (float) -1.8216074E37F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.time_usec = (ulong)1203375217083852440L;
            p141.altitude_relative = (float) -3.1340698E38F;
            p141.bottom_clearance = (float)1.6597611E38F;
            p141.altitude_amsl = (float) -1.8216074E37F;
            p141.altitude_local = (float) -1.2779467E38F;
            p141.altitude_terrain = (float) -3.3483686E38F;
            p141.altitude_monotonic = (float) -2.5858612E38F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)32, (byte)212, (byte)138, (byte)38, (byte)149, (byte)104, (byte)17, (byte)172, (byte)37, (byte)17, (byte)46, (byte)67, (byte)117, (byte)220, (byte)177, (byte)30, (byte)96, (byte)129, (byte)189, (byte)2, (byte)102, (byte)132, (byte)96, (byte)181, (byte)142, (byte)227, (byte)39, (byte)172, (byte)66, (byte)187, (byte)161, (byte)203, (byte)153, (byte)166, (byte)132, (byte)32, (byte)15, (byte)236, (byte)16, (byte)79, (byte)254, (byte)196, (byte)167, (byte)166, (byte)119, (byte)208, (byte)146, (byte)7, (byte)153, (byte)240, (byte)160, (byte)154, (byte)84, (byte)141, (byte)84, (byte)222, (byte)51, (byte)254, (byte)88, (byte)113, (byte)14, (byte)249, (byte)202, (byte)193, (byte)226, (byte)26, (byte)75, (byte)9, (byte)93, (byte)168, (byte)74, (byte)18, (byte)234, (byte)99, (byte)47, (byte)88, (byte)93, (byte)197, (byte)142, (byte)221, (byte)22, (byte)81, (byte)9, (byte)34, (byte)205, (byte)26, (byte)6, (byte)30, (byte)174, (byte)94, (byte)211, (byte)137, (byte)121, (byte)16, (byte)65, (byte)240, (byte)195, (byte)233, (byte)240, (byte)27, (byte)54, (byte)144, (byte)213, (byte)80, (byte)243, (byte)214, (byte)179, (byte)2, (byte)105, (byte)244, (byte)15, (byte)43, (byte)98, (byte)245, (byte)12, (byte)57, (byte)77, (byte)233, (byte)145, (byte)104}));
                Debug.Assert(pack.uri_type == (byte)(byte)248);
                Debug.Assert(pack.transfer_type == (byte)(byte)85);
                Debug.Assert(pack.request_id == (byte)(byte)124);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)240, (byte)34, (byte)78, (byte)85, (byte)51, (byte)60, (byte)102, (byte)187, (byte)44, (byte)37, (byte)65, (byte)244, (byte)10, (byte)26, (byte)252, (byte)10, (byte)8, (byte)42, (byte)236, (byte)59, (byte)27, (byte)189, (byte)61, (byte)166, (byte)69, (byte)252, (byte)173, (byte)105, (byte)232, (byte)110, (byte)14, (byte)185, (byte)229, (byte)141, (byte)163, (byte)21, (byte)210, (byte)41, (byte)119, (byte)214, (byte)226, (byte)104, (byte)65, (byte)95, (byte)175, (byte)23, (byte)141, (byte)203, (byte)182, (byte)30, (byte)42, (byte)32, (byte)52, (byte)232, (byte)153, (byte)241, (byte)91, (byte)43, (byte)228, (byte)149, (byte)123, (byte)35, (byte)220, (byte)249, (byte)133, (byte)143, (byte)181, (byte)83, (byte)79, (byte)212, (byte)237, (byte)190, (byte)80, (byte)228, (byte)128, (byte)236, (byte)163, (byte)126, (byte)70, (byte)63, (byte)163, (byte)77, (byte)92, (byte)252, (byte)215, (byte)215, (byte)140, (byte)252, (byte)196, (byte)92, (byte)2, (byte)185, (byte)2, (byte)189, (byte)245, (byte)101, (byte)44, (byte)206, (byte)118, (byte)12, (byte)145, (byte)181, (byte)140, (byte)188, (byte)218, (byte)22, (byte)83, (byte)197, (byte)209, (byte)199, (byte)183, (byte)31, (byte)65, (byte)228, (byte)211, (byte)62, (byte)137, (byte)104, (byte)171, (byte)144}));
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.request_id = (byte)(byte)124;
            p142.storage_SET(new byte[] {(byte)32, (byte)212, (byte)138, (byte)38, (byte)149, (byte)104, (byte)17, (byte)172, (byte)37, (byte)17, (byte)46, (byte)67, (byte)117, (byte)220, (byte)177, (byte)30, (byte)96, (byte)129, (byte)189, (byte)2, (byte)102, (byte)132, (byte)96, (byte)181, (byte)142, (byte)227, (byte)39, (byte)172, (byte)66, (byte)187, (byte)161, (byte)203, (byte)153, (byte)166, (byte)132, (byte)32, (byte)15, (byte)236, (byte)16, (byte)79, (byte)254, (byte)196, (byte)167, (byte)166, (byte)119, (byte)208, (byte)146, (byte)7, (byte)153, (byte)240, (byte)160, (byte)154, (byte)84, (byte)141, (byte)84, (byte)222, (byte)51, (byte)254, (byte)88, (byte)113, (byte)14, (byte)249, (byte)202, (byte)193, (byte)226, (byte)26, (byte)75, (byte)9, (byte)93, (byte)168, (byte)74, (byte)18, (byte)234, (byte)99, (byte)47, (byte)88, (byte)93, (byte)197, (byte)142, (byte)221, (byte)22, (byte)81, (byte)9, (byte)34, (byte)205, (byte)26, (byte)6, (byte)30, (byte)174, (byte)94, (byte)211, (byte)137, (byte)121, (byte)16, (byte)65, (byte)240, (byte)195, (byte)233, (byte)240, (byte)27, (byte)54, (byte)144, (byte)213, (byte)80, (byte)243, (byte)214, (byte)179, (byte)2, (byte)105, (byte)244, (byte)15, (byte)43, (byte)98, (byte)245, (byte)12, (byte)57, (byte)77, (byte)233, (byte)145, (byte)104}, 0) ;
            p142.uri_type = (byte)(byte)248;
            p142.uri_SET(new byte[] {(byte)240, (byte)34, (byte)78, (byte)85, (byte)51, (byte)60, (byte)102, (byte)187, (byte)44, (byte)37, (byte)65, (byte)244, (byte)10, (byte)26, (byte)252, (byte)10, (byte)8, (byte)42, (byte)236, (byte)59, (byte)27, (byte)189, (byte)61, (byte)166, (byte)69, (byte)252, (byte)173, (byte)105, (byte)232, (byte)110, (byte)14, (byte)185, (byte)229, (byte)141, (byte)163, (byte)21, (byte)210, (byte)41, (byte)119, (byte)214, (byte)226, (byte)104, (byte)65, (byte)95, (byte)175, (byte)23, (byte)141, (byte)203, (byte)182, (byte)30, (byte)42, (byte)32, (byte)52, (byte)232, (byte)153, (byte)241, (byte)91, (byte)43, (byte)228, (byte)149, (byte)123, (byte)35, (byte)220, (byte)249, (byte)133, (byte)143, (byte)181, (byte)83, (byte)79, (byte)212, (byte)237, (byte)190, (byte)80, (byte)228, (byte)128, (byte)236, (byte)163, (byte)126, (byte)70, (byte)63, (byte)163, (byte)77, (byte)92, (byte)252, (byte)215, (byte)215, (byte)140, (byte)252, (byte)196, (byte)92, (byte)2, (byte)185, (byte)2, (byte)189, (byte)245, (byte)101, (byte)44, (byte)206, (byte)118, (byte)12, (byte)145, (byte)181, (byte)140, (byte)188, (byte)218, (byte)22, (byte)83, (byte)197, (byte)209, (byte)199, (byte)183, (byte)31, (byte)65, (byte)228, (byte)211, (byte)62, (byte)137, (byte)104, (byte)171, (byte)144}, 0) ;
            p142.transfer_type = (byte)(byte)85;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)692817072U);
                Debug.Assert(pack.temperature == (short)(short)28135);
                Debug.Assert(pack.press_diff == (float)3.3479735E38F);
                Debug.Assert(pack.press_abs == (float) -3.053199E38F);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.press_abs = (float) -3.053199E38F;
            p143.temperature = (short)(short)28135;
            p143.press_diff = (float)3.3479735E38F;
            p143.time_boot_ms = (uint)692817072U;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-2.8448071E38F, 2.4116759E38F, 9.745743E37F, -6.340753E37F}));
                Debug.Assert(pack.timestamp == (ulong)8236764597763937589L);
                Debug.Assert(pack.est_capabilities == (byte)(byte)197);
                Debug.Assert(pack.rates.SequenceEqual(new float[] {1.0677116E38F, 2.0882217E38F, 2.394976E38F}));
                Debug.Assert(pack.custom_state == (ulong)8438448971500824354L);
                Debug.Assert(pack.lon == (int) -889523748);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {3.1833347E38F, 1.8115082E38F, -2.9257504E38F}));
                Debug.Assert(pack.lat == (int)124582271);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {-8.3108466E37F, 3.0234688E38F, -7.529217E37F}));
                Debug.Assert(pack.alt == (float)2.03118E38F);
                Debug.Assert(pack.acc.SequenceEqual(new float[] {-1.7061577E38F, -3.0264253E38F, -1.9497852E38F}));
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.custom_state = (ulong)8438448971500824354L;
            p144.alt = (float)2.03118E38F;
            p144.position_cov_SET(new float[] {3.1833347E38F, 1.8115082E38F, -2.9257504E38F}, 0) ;
            p144.acc_SET(new float[] {-1.7061577E38F, -3.0264253E38F, -1.9497852E38F}, 0) ;
            p144.timestamp = (ulong)8236764597763937589L;
            p144.lon = (int) -889523748;
            p144.rates_SET(new float[] {1.0677116E38F, 2.0882217E38F, 2.394976E38F}, 0) ;
            p144.vel_SET(new float[] {-8.3108466E37F, 3.0234688E38F, -7.529217E37F}, 0) ;
            p144.est_capabilities = (byte)(byte)197;
            p144.attitude_q_SET(new float[] {-2.8448071E38F, 2.4116759E38F, 9.745743E37F, -6.340753E37F}, 0) ;
            p144.lat = (int)124582271;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)4359018725458318478L);
                Debug.Assert(pack.x_acc == (float)3.2638877E38F);
                Debug.Assert(pack.z_pos == (float) -2.7652404E38F);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {1.6202232E38F, -7.052399E37F, 3.5366587E36F}));
                Debug.Assert(pack.z_vel == (float)2.8439684E38F);
                Debug.Assert(pack.pitch_rate == (float) -6.9414767E37F);
                Debug.Assert(pack.roll_rate == (float) -9.003239E37F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {2.995349E38F, 1.3050369E38F, 3.0321933E38F}));
                Debug.Assert(pack.airspeed == (float) -2.5581254E38F);
                Debug.Assert(pack.y_pos == (float)1.8469562E38F);
                Debug.Assert(pack.x_pos == (float) -2.9867345E38F);
                Debug.Assert(pack.z_acc == (float) -4.065127E36F);
                Debug.Assert(pack.yaw_rate == (float) -1.099365E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.194731E38F, -8.660582E37F, 3.39752E37F, 2.3886346E38F}));
                Debug.Assert(pack.x_vel == (float) -2.0744136E37F);
                Debug.Assert(pack.y_acc == (float)2.7141677E37F);
                Debug.Assert(pack.y_vel == (float) -2.1911251E38F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.z_pos = (float) -2.7652404E38F;
            p146.x_pos = (float) -2.9867345E38F;
            p146.y_pos = (float)1.8469562E38F;
            p146.pos_variance_SET(new float[] {2.995349E38F, 1.3050369E38F, 3.0321933E38F}, 0) ;
            p146.x_acc = (float)3.2638877E38F;
            p146.time_usec = (ulong)4359018725458318478L;
            p146.roll_rate = (float) -9.003239E37F;
            p146.z_acc = (float) -4.065127E36F;
            p146.yaw_rate = (float) -1.099365E38F;
            p146.airspeed = (float) -2.5581254E38F;
            p146.x_vel = (float) -2.0744136E37F;
            p146.z_vel = (float)2.8439684E38F;
            p146.pitch_rate = (float) -6.9414767E37F;
            p146.y_vel = (float) -2.1911251E38F;
            p146.y_acc = (float)2.7141677E37F;
            p146.vel_variance_SET(new float[] {1.6202232E38F, -7.052399E37F, 3.5366587E36F}, 0) ;
            p146.q_SET(new float[] {-1.194731E38F, -8.660582E37F, 3.39752E37F, 2.3886346E38F}, 0) ;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)31860, (ushort)58907, (ushort)38842, (ushort)10393, (ushort)7406, (ushort)36932, (ushort)11617, (ushort)39450, (ushort)30772, (ushort)47429}));
                Debug.Assert(pack.energy_consumed == (int)768462295);
                Debug.Assert(pack.current_battery == (short)(short) -7187);
                Debug.Assert(pack.type == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte)38);
                Debug.Assert(pack.battery_function == MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
                Debug.Assert(pack.temperature == (short)(short)15884);
                Debug.Assert(pack.id == (byte)(byte)219);
                Debug.Assert(pack.current_consumed == (int) -1409071656);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.temperature = (short)(short)15884;
            p147.current_battery = (short)(short) -7187;
            p147.id = (byte)(byte)219;
            p147.current_consumed = (int) -1409071656;
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO;
            p147.voltages_SET(new ushort[] {(ushort)31860, (ushort)58907, (ushort)38842, (ushort)10393, (ushort)7406, (ushort)36932, (ushort)11617, (ushort)39450, (ushort)30772, (ushort)47429}, 0) ;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD;
            p147.energy_consumed = (int)768462295;
            p147.battery_remaining = (sbyte)(sbyte)38;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)144, (byte)95, (byte)134, (byte)122, (byte)205, (byte)25, (byte)236, (byte)251, (byte)77, (byte)180, (byte)60, (byte)154, (byte)158, (byte)219, (byte)8, (byte)154, (byte)173, (byte)25}));
                Debug.Assert(pack.middleware_sw_version == (uint)2910555628U);
                Debug.Assert(pack.product_id == (ushort)(ushort)47832);
                Debug.Assert(pack.os_sw_version == (uint)669384835U);
                Debug.Assert(pack.uid == (ulong)5329967165932354051L);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)16335);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE));
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)183, (byte)254, (byte)204, (byte)196, (byte)173, (byte)14, (byte)198, (byte)84}));
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)93, (byte)166, (byte)230, (byte)189, (byte)115, (byte)117, (byte)5, (byte)78}));
                Debug.Assert(pack.flight_sw_version == (uint)1170538084U);
                Debug.Assert(pack.board_version == (uint)93045094U);
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)97, (byte)89, (byte)157, (byte)240, (byte)72, (byte)3, (byte)28, (byte)210}));
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.os_custom_version_SET(new byte[] {(byte)97, (byte)89, (byte)157, (byte)240, (byte)72, (byte)3, (byte)28, (byte)210}, 0) ;
            p148.middleware_custom_version_SET(new byte[] {(byte)183, (byte)254, (byte)204, (byte)196, (byte)173, (byte)14, (byte)198, (byte)84}, 0) ;
            p148.uid = (ulong)5329967165932354051L;
            p148.vendor_id = (ushort)(ushort)16335;
            p148.uid2_SET(new byte[] {(byte)144, (byte)95, (byte)134, (byte)122, (byte)205, (byte)25, (byte)236, (byte)251, (byte)77, (byte)180, (byte)60, (byte)154, (byte)158, (byte)219, (byte)8, (byte)154, (byte)173, (byte)25}, 0, PH) ;
            p148.os_sw_version = (uint)669384835U;
            p148.flight_sw_version = (uint)1170538084U;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FTP |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE);
            p148.product_id = (ushort)(ushort)47832;
            p148.middleware_sw_version = (uint)2910555628U;
            p148.flight_custom_version_SET(new byte[] {(byte)93, (byte)166, (byte)230, (byte)189, (byte)115, (byte)117, (byte)5, (byte)78}, 0) ;
            p148.board_version = (uint)93045094U;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_num == (byte)(byte)54);
                Debug.Assert(pack.size_x == (float)2.4435852E38F);
                Debug.Assert(pack.x_TRY(ph) == (float) -3.852465E37F);
                Debug.Assert(pack.angle_y == (float) -8.010551E37F);
                Debug.Assert(pack.angle_x == (float)3.1796218E38F);
                Debug.Assert(pack.size_y == (float) -2.6417534E38F);
                Debug.Assert(pack.distance == (float)3.0403703E38F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)142);
                Debug.Assert(pack.z_TRY(ph) == (float)3.183737E38F);
                Debug.Assert(pack.y_TRY(ph) == (float)1.0669243E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.time_usec == (ulong)4689260543795877626L);
                Debug.Assert(pack.type == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {-3.2233736E38F, -2.9973844E38F, -3.0082325E37F, -2.7246253E38F}));
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.q_SET(new float[] {-3.2233736E38F, -2.9973844E38F, -3.0082325E37F, -2.7246253E38F}, 0, PH) ;
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON;
            p149.distance = (float)3.0403703E38F;
            p149.z_SET((float)3.183737E38F, PH) ;
            p149.angle_x = (float)3.1796218E38F;
            p149.frame = MAV_FRAME.MAV_FRAME_MISSION;
            p149.target_num = (byte)(byte)54;
            p149.y_SET((float)1.0669243E38F, PH) ;
            p149.time_usec = (ulong)4689260543795877626L;
            p149.x_SET((float) -3.852465E37F, PH) ;
            p149.size_y = (float) -2.6417534E38F;
            p149.position_valid_SET((byte)(byte)142, PH) ;
            p149.size_x = (float)2.4435852E38F;
            p149.angle_y = (float) -8.010551E37F;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tas_ratio == (float) -8.627897E37F);
                Debug.Assert(pack.pos_vert_accuracy == (float)1.3999289E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float) -3.3925364E38F);
                Debug.Assert(pack.time_usec == (ulong)2166538952587999883L);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ));
                Debug.Assert(pack.hagl_ratio == (float)8.1416624E37F);
                Debug.Assert(pack.pos_horiz_ratio == (float)1.098559E38F);
                Debug.Assert(pack.pos_vert_ratio == (float)3.385458E38F);
                Debug.Assert(pack.mag_ratio == (float) -1.7517228E38F);
                Debug.Assert(pack.vel_ratio == (float) -1.0807437E38F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.pos_horiz_accuracy = (float) -3.3925364E38F;
            p230.pos_horiz_ratio = (float)1.098559E38F;
            p230.pos_vert_accuracy = (float)1.3999289E38F;
            p230.vel_ratio = (float) -1.0807437E38F;
            p230.time_usec = (ulong)2166538952587999883L;
            p230.mag_ratio = (float) -1.7517228E38F;
            p230.hagl_ratio = (float)8.1416624E37F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ);
            p230.tas_ratio = (float) -8.627897E37F;
            p230.pos_vert_ratio = (float)3.385458E38F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.wind_alt == (float) -2.8197667E38F);
                Debug.Assert(pack.horiz_accuracy == (float)2.4575908E38F);
                Debug.Assert(pack.var_vert == (float) -6.932402E37F);
                Debug.Assert(pack.time_usec == (ulong)3861555192442383402L);
                Debug.Assert(pack.wind_z == (float)2.4081554E38F);
                Debug.Assert(pack.var_horiz == (float)6.3361127E37F);
                Debug.Assert(pack.wind_x == (float) -1.8703404E37F);
                Debug.Assert(pack.vert_accuracy == (float) -1.6191326E36F);
                Debug.Assert(pack.wind_y == (float) -1.1004295E37F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.vert_accuracy = (float) -1.6191326E36F;
            p231.wind_alt = (float) -2.8197667E38F;
            p231.var_vert = (float) -6.932402E37F;
            p231.time_usec = (ulong)3861555192442383402L;
            p231.var_horiz = (float)6.3361127E37F;
            p231.horiz_accuracy = (float)2.4575908E38F;
            p231.wind_x = (float) -1.8703404E37F;
            p231.wind_z = (float)2.4081554E38F;
            p231.wind_y = (float) -1.1004295E37F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)194962262);
                Debug.Assert(pack.time_usec == (ulong)4670593272663776897L);
                Debug.Assert(pack.time_week == (ushort)(ushort)1464);
                Debug.Assert(pack.alt == (float) -2.740547E38F);
                Debug.Assert(pack.ve == (float) -2.093601E38F);
                Debug.Assert(pack.hdop == (float) -7.758696E37F);
                Debug.Assert(pack.vd == (float) -1.4173083E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -3.289571E38F);
                Debug.Assert(pack.gps_id == (byte)(byte)22);
                Debug.Assert(pack.lat == (int)1307278396);
                Debug.Assert(pack.fix_type == (byte)(byte)43);
                Debug.Assert(pack.vert_accuracy == (float)2.2596716E38F);
                Debug.Assert(pack.satellites_visible == (byte)(byte)203);
                Debug.Assert(pack.speed_accuracy == (float)3.0952125E38F);
                Debug.Assert(pack.vn == (float) -2.209931E38F);
                Debug.Assert(pack.vdop == (float) -3.9431408E37F);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ));
                Debug.Assert(pack.time_week_ms == (uint)1952986431U);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.vn = (float) -2.209931E38F;
            p232.time_week_ms = (uint)1952986431U;
            p232.satellites_visible = (byte)(byte)203;
            p232.alt = (float) -2.740547E38F;
            p232.lat = (int)1307278396;
            p232.lon = (int)194962262;
            p232.vd = (float) -1.4173083E38F;
            p232.time_usec = (ulong)4670593272663776897L;
            p232.speed_accuracy = (float)3.0952125E38F;
            p232.hdop = (float) -7.758696E37F;
            p232.vert_accuracy = (float)2.2596716E38F;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ);
            p232.vdop = (float) -3.9431408E37F;
            p232.time_week = (ushort)(ushort)1464;
            p232.horiz_accuracy = (float) -3.289571E38F;
            p232.gps_id = (byte)(byte)22;
            p232.fix_type = (byte)(byte)43;
            p232.ve = (float) -2.093601E38F;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (byte)(byte)135);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)7, (byte)73, (byte)180, (byte)194, (byte)1, (byte)108, (byte)139, (byte)186, (byte)101, (byte)138, (byte)3, (byte)122, (byte)159, (byte)232, (byte)232, (byte)169, (byte)204, (byte)158, (byte)136, (byte)26, (byte)124, (byte)68, (byte)87, (byte)124, (byte)92, (byte)81, (byte)173, (byte)12, (byte)84, (byte)82, (byte)197, (byte)247, (byte)173, (byte)168, (byte)161, (byte)2, (byte)50, (byte)76, (byte)55, (byte)87, (byte)104, (byte)200, (byte)95, (byte)149, (byte)135, (byte)119, (byte)20, (byte)196, (byte)198, (byte)42, (byte)175, (byte)12, (byte)56, (byte)38, (byte)246, (byte)175, (byte)96, (byte)64, (byte)162, (byte)126, (byte)213, (byte)180, (byte)187, (byte)49, (byte)147, (byte)128, (byte)158, (byte)204, (byte)149, (byte)160, (byte)132, (byte)28, (byte)74, (byte)126, (byte)18, (byte)159, (byte)51, (byte)177, (byte)17, (byte)135, (byte)31, (byte)199, (byte)67, (byte)223, (byte)109, (byte)54, (byte)233, (byte)154, (byte)13, (byte)111, (byte)140, (byte)233, (byte)247, (byte)237, (byte)216, (byte)169, (byte)179, (byte)43, (byte)252, (byte)56, (byte)102, (byte)241, (byte)125, (byte)146, (byte)146, (byte)244, (byte)109, (byte)167, (byte)158, (byte)163, (byte)249, (byte)171, (byte)75, (byte)52, (byte)29, (byte)37, (byte)241, (byte)25, (byte)229, (byte)235, (byte)207, (byte)45, (byte)191, (byte)29, (byte)170, (byte)51, (byte)188, (byte)111, (byte)81, (byte)93, (byte)135, (byte)13, (byte)34, (byte)21, (byte)246, (byte)186, (byte)36, (byte)0, (byte)143, (byte)158, (byte)237, (byte)205, (byte)200, (byte)22, (byte)29, (byte)23, (byte)253, (byte)65, (byte)123, (byte)37, (byte)182, (byte)186, (byte)164, (byte)244, (byte)149, (byte)144, (byte)99, (byte)238, (byte)52, (byte)167, (byte)154, (byte)160, (byte)185, (byte)182, (byte)31, (byte)87, (byte)229, (byte)253, (byte)252, (byte)218, (byte)19, (byte)158, (byte)189, (byte)73, (byte)133, (byte)142, (byte)208, (byte)17, (byte)40, (byte)24}));
                Debug.Assert(pack.len == (byte)(byte)82);
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.len = (byte)(byte)82;
            p233.data__SET(new byte[] {(byte)7, (byte)73, (byte)180, (byte)194, (byte)1, (byte)108, (byte)139, (byte)186, (byte)101, (byte)138, (byte)3, (byte)122, (byte)159, (byte)232, (byte)232, (byte)169, (byte)204, (byte)158, (byte)136, (byte)26, (byte)124, (byte)68, (byte)87, (byte)124, (byte)92, (byte)81, (byte)173, (byte)12, (byte)84, (byte)82, (byte)197, (byte)247, (byte)173, (byte)168, (byte)161, (byte)2, (byte)50, (byte)76, (byte)55, (byte)87, (byte)104, (byte)200, (byte)95, (byte)149, (byte)135, (byte)119, (byte)20, (byte)196, (byte)198, (byte)42, (byte)175, (byte)12, (byte)56, (byte)38, (byte)246, (byte)175, (byte)96, (byte)64, (byte)162, (byte)126, (byte)213, (byte)180, (byte)187, (byte)49, (byte)147, (byte)128, (byte)158, (byte)204, (byte)149, (byte)160, (byte)132, (byte)28, (byte)74, (byte)126, (byte)18, (byte)159, (byte)51, (byte)177, (byte)17, (byte)135, (byte)31, (byte)199, (byte)67, (byte)223, (byte)109, (byte)54, (byte)233, (byte)154, (byte)13, (byte)111, (byte)140, (byte)233, (byte)247, (byte)237, (byte)216, (byte)169, (byte)179, (byte)43, (byte)252, (byte)56, (byte)102, (byte)241, (byte)125, (byte)146, (byte)146, (byte)244, (byte)109, (byte)167, (byte)158, (byte)163, (byte)249, (byte)171, (byte)75, (byte)52, (byte)29, (byte)37, (byte)241, (byte)25, (byte)229, (byte)235, (byte)207, (byte)45, (byte)191, (byte)29, (byte)170, (byte)51, (byte)188, (byte)111, (byte)81, (byte)93, (byte)135, (byte)13, (byte)34, (byte)21, (byte)246, (byte)186, (byte)36, (byte)0, (byte)143, (byte)158, (byte)237, (byte)205, (byte)200, (byte)22, (byte)29, (byte)23, (byte)253, (byte)65, (byte)123, (byte)37, (byte)182, (byte)186, (byte)164, (byte)244, (byte)149, (byte)144, (byte)99, (byte)238, (byte)52, (byte)167, (byte)154, (byte)160, (byte)185, (byte)182, (byte)31, (byte)87, (byte)229, (byte)253, (byte)252, (byte)218, (byte)19, (byte)158, (byte)189, (byte)73, (byte)133, (byte)142, (byte)208, (byte)17, (byte)40, (byte)24}, 0) ;
            p233.flags = (byte)(byte)135;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.groundspeed == (byte)(byte)113);
                Debug.Assert(pack.gps_nsat == (byte)(byte)130);
                Debug.Assert(pack.failsafe == (byte)(byte)152);
                Debug.Assert(pack.custom_mode == (uint)3845131697U);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)188);
                Debug.Assert(pack.throttle == (sbyte)(sbyte) - 56);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
                Debug.Assert(pack.altitude_amsl == (short)(short) -3892);
                Debug.Assert(pack.heading == (ushort)(ushort)35831);
                Debug.Assert(pack.gps_fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
                Debug.Assert(pack.longitude == (int) -1230528529);
                Debug.Assert(pack.temperature == (sbyte)(sbyte) - 24);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte)75);
                Debug.Assert(pack.altitude_sp == (short)(short) -4962);
                Debug.Assert(pack.pitch == (short)(short)25180);
                Debug.Assert(pack.wp_num == (byte)(byte)151);
                Debug.Assert(pack.latitude == (int)683462955);
                Debug.Assert(pack.roll == (short)(short)22783);
                Debug.Assert(pack.battery_remaining == (byte)(byte)39);
                Debug.Assert(pack.heading_sp == (short)(short)3354);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED));
                Debug.Assert(pack.airspeed == (byte)(byte)150);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)9144);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte)119);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.latitude = (int)683462955;
            p234.gps_nsat = (byte)(byte)130;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            p234.airspeed_sp = (byte)(byte)188;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX;
            p234.longitude = (int) -1230528529;
            p234.airspeed = (byte)(byte)150;
            p234.failsafe = (byte)(byte)152;
            p234.throttle = (sbyte)(sbyte) - 56;
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED);
            p234.temperature = (sbyte)(sbyte) - 24;
            p234.temperature_air = (sbyte)(sbyte)75;
            p234.pitch = (short)(short)25180;
            p234.wp_num = (byte)(byte)151;
            p234.heading = (ushort)(ushort)35831;
            p234.altitude_amsl = (short)(short) -3892;
            p234.roll = (short)(short)22783;
            p234.groundspeed = (byte)(byte)113;
            p234.altitude_sp = (short)(short) -4962;
            p234.battery_remaining = (byte)(byte)39;
            p234.custom_mode = (uint)3845131697U;
            p234.heading_sp = (short)(short)3354;
            p234.wp_distance = (ushort)(ushort)9144;
            p234.climb_rate = (sbyte)(sbyte)119;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)7633315532331284387L);
                Debug.Assert(pack.clipping_0 == (uint)3295598026U);
                Debug.Assert(pack.vibration_z == (float)1.2388803E38F);
                Debug.Assert(pack.vibration_y == (float)5.900832E37F);
                Debug.Assert(pack.vibration_x == (float) -2.0813939E37F);
                Debug.Assert(pack.clipping_1 == (uint)1706969443U);
                Debug.Assert(pack.clipping_2 == (uint)2890174335U);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.vibration_x = (float) -2.0813939E37F;
            p241.vibration_z = (float)1.2388803E38F;
            p241.clipping_1 = (uint)1706969443U;
            p241.clipping_0 = (uint)3295598026U;
            p241.time_usec = (ulong)7633315532331284387L;
            p241.vibration_y = (float)5.900832E37F;
            p241.clipping_2 = (uint)2890174335U;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)3093899398752513436L);
                Debug.Assert(pack.approach_y == (float)2.0860509E38F);
                Debug.Assert(pack.latitude == (int)428481399);
                Debug.Assert(pack.z == (float)7.737157E37F);
                Debug.Assert(pack.approach_x == (float) -2.9688369E38F);
                Debug.Assert(pack.approach_z == (float)8.3518095E37F);
                Debug.Assert(pack.longitude == (int) -448251781);
                Debug.Assert(pack.x == (float)5.520942E37F);
                Debug.Assert(pack.y == (float) -8.482176E37F);
                Debug.Assert(pack.altitude == (int)417643152);
                Debug.Assert(pack.q.SequenceEqual(new float[] {1.4888901E38F, -2.6684638E37F, -1.580565E38F, 5.7471643E37F}));
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.x = (float)5.520942E37F;
            p242.time_usec_SET((ulong)3093899398752513436L, PH) ;
            p242.approach_x = (float) -2.9688369E38F;
            p242.approach_z = (float)8.3518095E37F;
            p242.y = (float) -8.482176E37F;
            p242.approach_y = (float)2.0860509E38F;
            p242.q_SET(new float[] {1.4888901E38F, -2.6684638E37F, -1.580565E38F, 5.7471643E37F}, 0) ;
            p242.z = (float)7.737157E37F;
            p242.altitude = (int)417643152;
            p242.latitude = (int)428481399;
            p242.longitude = (int) -448251781;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.latitude == (int)1218422054);
                Debug.Assert(pack.approach_y == (float)1.9798236E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {6.4722016E37F, -1.5925323E38F, -2.4873846E38F, -2.1446747E37F}));
                Debug.Assert(pack.x == (float) -1.1738597E38F);
                Debug.Assert(pack.approach_z == (float)2.1026162E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)8817913196221827341L);
                Debug.Assert(pack.target_system == (byte)(byte)206);
                Debug.Assert(pack.approach_x == (float)2.531371E38F);
                Debug.Assert(pack.y == (float) -1.5392929E38F);
                Debug.Assert(pack.altitude == (int) -1859665619);
                Debug.Assert(pack.longitude == (int) -584103551);
                Debug.Assert(pack.z == (float)2.1755815E38F);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.x = (float) -1.1738597E38F;
            p243.latitude = (int)1218422054;
            p243.approach_x = (float)2.531371E38F;
            p243.y = (float) -1.5392929E38F;
            p243.approach_y = (float)1.9798236E38F;
            p243.target_system = (byte)(byte)206;
            p243.approach_z = (float)2.1026162E38F;
            p243.altitude = (int) -1859665619;
            p243.time_usec_SET((ulong)8817913196221827341L, PH) ;
            p243.q_SET(new float[] {6.4722016E37F, -1.5925323E38F, -2.4873846E38F, -2.1446747E37F}, 0) ;
            p243.longitude = (int) -584103551;
            p243.z = (float)2.1755815E38F;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)31710);
                Debug.Assert(pack.interval_us == (int)1874005702);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.message_id = (ushort)(ushort)31710;
            p244.interval_us = (int)1874005702;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vtol_state == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND;
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.callsign_LEN(ph) == 3);
                Debug.Assert(pack.callsign_TRY(ph).Equals("mhf"));
                Debug.Assert(pack.heading == (ushort)(ushort)44860);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)56886);
                Debug.Assert(pack.ver_velocity == (short)(short) -18071);
                Debug.Assert(pack.altitude_type == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
                Debug.Assert(pack.lat == (int)324412958);
                Debug.Assert(pack.squawk == (ushort)(ushort)56609);
                Debug.Assert(pack.flags == (ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                                            ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS));
                Debug.Assert(pack.emitter_type == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HEAVY);
                Debug.Assert(pack.altitude == (int)977177035);
                Debug.Assert(pack.lon == (int) -1208589747);
                Debug.Assert(pack.tslc == (byte)(byte)205);
                Debug.Assert(pack.ICAO_address == (uint)2546425287U);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.squawk = (ushort)(ushort)56609;
            p246.ver_velocity = (short)(short) -18071;
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HEAVY;
            p246.altitude = (int)977177035;
            p246.hor_velocity = (ushort)(ushort)56886;
            p246.callsign_SET("mhf", PH) ;
            p246.heading = (ushort)(ushort)44860;
            p246.tslc = (byte)(byte)205;
            p246.ICAO_address = (uint)2546425287U;
            p246.lat = (int)324412958;
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                          ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS);
            p246.lon = (int) -1208589747;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.action == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW |
                                                   MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE));
                Debug.Assert(pack.altitude_minimum_delta == (float) -1.0676716E38F);
                Debug.Assert(pack.id == (uint)866741324U);
                Debug.Assert(pack.horizontal_minimum_delta == (float) -1.1681647E38F);
                Debug.Assert(pack.src_ == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
                Debug.Assert(pack.time_to_minimum_delta == (float)1.830753E38F);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT;
            p247.time_to_minimum_delta = (float)1.830753E38F;
            p247.id = (uint)866741324U;
            p247.horizontal_minimum_delta = (float) -1.1681647E38F;
            p247.altitude_minimum_delta = (float) -1.0676716E38F;
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW |
                                 MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)192, (byte)154, (byte)59, (byte)194, (byte)14, (byte)80, (byte)242, (byte)91, (byte)111, (byte)172, (byte)137, (byte)202, (byte)140, (byte)202, (byte)123, (byte)33, (byte)251, (byte)52, (byte)120, (byte)105, (byte)247, (byte)119, (byte)253, (byte)59, (byte)239, (byte)126, (byte)49, (byte)73, (byte)204, (byte)109, (byte)255, (byte)0, (byte)161, (byte)242, (byte)67, (byte)232, (byte)200, (byte)17, (byte)99, (byte)213, (byte)249, (byte)134, (byte)12, (byte)103, (byte)161, (byte)230, (byte)99, (byte)80, (byte)149, (byte)184, (byte)185, (byte)123, (byte)131, (byte)128, (byte)106, (byte)27, (byte)6, (byte)255, (byte)60, (byte)19, (byte)157, (byte)138, (byte)26, (byte)33, (byte)106, (byte)48, (byte)1, (byte)209, (byte)83, (byte)68, (byte)3, (byte)55, (byte)211, (byte)93, (byte)243, (byte)173, (byte)130, (byte)245, (byte)145, (byte)129, (byte)72, (byte)76, (byte)23, (byte)154, (byte)243, (byte)6, (byte)200, (byte)209, (byte)136, (byte)217, (byte)162, (byte)199, (byte)195, (byte)7, (byte)52, (byte)6, (byte)21, (byte)51, (byte)209, (byte)152, (byte)33, (byte)35, (byte)187, (byte)159, (byte)37, (byte)212, (byte)86, (byte)238, (byte)83, (byte)168, (byte)176, (byte)138, (byte)243, (byte)104, (byte)44, (byte)210, (byte)152, (byte)206, (byte)14, (byte)74, (byte)1, (byte)93, (byte)70, (byte)21, (byte)34, (byte)110, (byte)89, (byte)164, (byte)114, (byte)40, (byte)179, (byte)86, (byte)252, (byte)238, (byte)145, (byte)251, (byte)186, (byte)178, (byte)90, (byte)38, (byte)87, (byte)193, (byte)9, (byte)204, (byte)96, (byte)234, (byte)191, (byte)109, (byte)33, (byte)196, (byte)232, (byte)206, (byte)212, (byte)181, (byte)111, (byte)135, (byte)184, (byte)34, (byte)235, (byte)52, (byte)23, (byte)205, (byte)201, (byte)74, (byte)204, (byte)218, (byte)48, (byte)204, (byte)122, (byte)155, (byte)32, (byte)231, (byte)112, (byte)0, (byte)195, (byte)94, (byte)35, (byte)54, (byte)38, (byte)181, (byte)141, (byte)140, (byte)101, (byte)194, (byte)251, (byte)67, (byte)161, (byte)18, (byte)127, (byte)132, (byte)21, (byte)20, (byte)242, (byte)155, (byte)32, (byte)70, (byte)99, (byte)101, (byte)60, (byte)138, (byte)215, (byte)111, (byte)125, (byte)54, (byte)212, (byte)121, (byte)111, (byte)4, (byte)185, (byte)234, (byte)61, (byte)63, (byte)205, (byte)117, (byte)177, (byte)93, (byte)228, (byte)65, (byte)254, (byte)34, (byte)135, (byte)169, (byte)238, (byte)85, (byte)196, (byte)28, (byte)92, (byte)245, (byte)46, (byte)186, (byte)165, (byte)159, (byte)120, (byte)157, (byte)28, (byte)20, (byte)103, (byte)242, (byte)6, (byte)157, (byte)113, (byte)27, (byte)104, (byte)14, (byte)8, (byte)137, (byte)243, (byte)93, (byte)118}));
                Debug.Assert(pack.target_component == (byte)(byte)104);
                Debug.Assert(pack.message_type == (ushort)(ushort)8355);
                Debug.Assert(pack.target_system == (byte)(byte)198);
                Debug.Assert(pack.target_network == (byte)(byte)170);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_system = (byte)(byte)198;
            p248.payload_SET(new byte[] {(byte)192, (byte)154, (byte)59, (byte)194, (byte)14, (byte)80, (byte)242, (byte)91, (byte)111, (byte)172, (byte)137, (byte)202, (byte)140, (byte)202, (byte)123, (byte)33, (byte)251, (byte)52, (byte)120, (byte)105, (byte)247, (byte)119, (byte)253, (byte)59, (byte)239, (byte)126, (byte)49, (byte)73, (byte)204, (byte)109, (byte)255, (byte)0, (byte)161, (byte)242, (byte)67, (byte)232, (byte)200, (byte)17, (byte)99, (byte)213, (byte)249, (byte)134, (byte)12, (byte)103, (byte)161, (byte)230, (byte)99, (byte)80, (byte)149, (byte)184, (byte)185, (byte)123, (byte)131, (byte)128, (byte)106, (byte)27, (byte)6, (byte)255, (byte)60, (byte)19, (byte)157, (byte)138, (byte)26, (byte)33, (byte)106, (byte)48, (byte)1, (byte)209, (byte)83, (byte)68, (byte)3, (byte)55, (byte)211, (byte)93, (byte)243, (byte)173, (byte)130, (byte)245, (byte)145, (byte)129, (byte)72, (byte)76, (byte)23, (byte)154, (byte)243, (byte)6, (byte)200, (byte)209, (byte)136, (byte)217, (byte)162, (byte)199, (byte)195, (byte)7, (byte)52, (byte)6, (byte)21, (byte)51, (byte)209, (byte)152, (byte)33, (byte)35, (byte)187, (byte)159, (byte)37, (byte)212, (byte)86, (byte)238, (byte)83, (byte)168, (byte)176, (byte)138, (byte)243, (byte)104, (byte)44, (byte)210, (byte)152, (byte)206, (byte)14, (byte)74, (byte)1, (byte)93, (byte)70, (byte)21, (byte)34, (byte)110, (byte)89, (byte)164, (byte)114, (byte)40, (byte)179, (byte)86, (byte)252, (byte)238, (byte)145, (byte)251, (byte)186, (byte)178, (byte)90, (byte)38, (byte)87, (byte)193, (byte)9, (byte)204, (byte)96, (byte)234, (byte)191, (byte)109, (byte)33, (byte)196, (byte)232, (byte)206, (byte)212, (byte)181, (byte)111, (byte)135, (byte)184, (byte)34, (byte)235, (byte)52, (byte)23, (byte)205, (byte)201, (byte)74, (byte)204, (byte)218, (byte)48, (byte)204, (byte)122, (byte)155, (byte)32, (byte)231, (byte)112, (byte)0, (byte)195, (byte)94, (byte)35, (byte)54, (byte)38, (byte)181, (byte)141, (byte)140, (byte)101, (byte)194, (byte)251, (byte)67, (byte)161, (byte)18, (byte)127, (byte)132, (byte)21, (byte)20, (byte)242, (byte)155, (byte)32, (byte)70, (byte)99, (byte)101, (byte)60, (byte)138, (byte)215, (byte)111, (byte)125, (byte)54, (byte)212, (byte)121, (byte)111, (byte)4, (byte)185, (byte)234, (byte)61, (byte)63, (byte)205, (byte)117, (byte)177, (byte)93, (byte)228, (byte)65, (byte)254, (byte)34, (byte)135, (byte)169, (byte)238, (byte)85, (byte)196, (byte)28, (byte)92, (byte)245, (byte)46, (byte)186, (byte)165, (byte)159, (byte)120, (byte)157, (byte)28, (byte)20, (byte)103, (byte)242, (byte)6, (byte)157, (byte)113, (byte)27, (byte)104, (byte)14, (byte)8, (byte)137, (byte)243, (byte)93, (byte)118}, 0) ;
            p248.message_type = (ushort)(ushort)8355;
            p248.target_component = (byte)(byte)104;
            p248.target_network = (byte)(byte)170;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ver == (byte)(byte)2);
                Debug.Assert(pack.type == (byte)(byte)202);
                Debug.Assert(pack.address == (ushort)(ushort)26143);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte)10, (sbyte)37, (sbyte) - 118, (sbyte) - 35, (sbyte) - 41, (sbyte) - 105, (sbyte)24, (sbyte) - 115, (sbyte)26, (sbyte) - 43, (sbyte) - 89, (sbyte) - 86, (sbyte) - 30, (sbyte) - 68, (sbyte) - 65, (sbyte)58, (sbyte) - 88, (sbyte)12, (sbyte)11, (sbyte)118, (sbyte) - 40, (sbyte) - 81, (sbyte)104, (sbyte)61, (sbyte)113, (sbyte)80, (sbyte) - 62, (sbyte) - 127, (sbyte) - 17, (sbyte)101, (sbyte)96, (sbyte)60}));
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.value_SET(new sbyte[] {(sbyte)10, (sbyte)37, (sbyte) - 118, (sbyte) - 35, (sbyte) - 41, (sbyte) - 105, (sbyte)24, (sbyte) - 115, (sbyte)26, (sbyte) - 43, (sbyte) - 89, (sbyte) - 86, (sbyte) - 30, (sbyte) - 68, (sbyte) - 65, (sbyte)58, (sbyte) - 88, (sbyte)12, (sbyte)11, (sbyte)118, (sbyte) - 40, (sbyte) - 81, (sbyte)104, (sbyte)61, (sbyte)113, (sbyte)80, (sbyte) - 62, (sbyte) - 127, (sbyte) - 17, (sbyte)101, (sbyte)96, (sbyte)60}, 0) ;
            p249.type = (byte)(byte)202;
            p249.ver = (byte)(byte)2;
            p249.address = (ushort)(ushort)26143;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 6);
                Debug.Assert(pack.name_TRY(ph).Equals("jkaDdf"));
                Debug.Assert(pack.y == (float) -1.3552019E38F);
                Debug.Assert(pack.x == (float) -3.1580036E38F);
                Debug.Assert(pack.z == (float)1.5434684E38F);
                Debug.Assert(pack.time_usec == (ulong)3659663047772347804L);
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.y = (float) -1.3552019E38F;
            p250.z = (float)1.5434684E38F;
            p250.name_SET("jkaDdf", PH) ;
            p250.time_usec = (ulong)3659663047772347804L;
            p250.x = (float) -3.1580036E38F;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3661020268U);
                Debug.Assert(pack.name_LEN(ph) == 6);
                Debug.Assert(pack.name_TRY(ph).Equals("hQysef"));
                Debug.Assert(pack.value == (float)4.596635E36F);
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.time_boot_ms = (uint)3661020268U;
            p251.name_SET("hQysef", PH) ;
            p251.value = (float)4.596635E36F;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.name_LEN(ph) == 9);
                Debug.Assert(pack.name_TRY(ph).Equals("qosTijjlt"));
                Debug.Assert(pack.value == (int)1108393093);
                Debug.Assert(pack.time_boot_ms == (uint)2518747338U);
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.name_SET("qosTijjlt", PH) ;
            p252.value = (int)1108393093;
            p252.time_boot_ms = (uint)2518747338U;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 23);
                Debug.Assert(pack.text_TRY(ph).Equals("NoLTmcacbcuwwodzaxvxxrO"));
                Debug.Assert(pack.severity == MAV_SEVERITY.MAV_SEVERITY_WARNING);
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.text_SET("NoLTmcacbcuwwodzaxvxxrO", PH) ;
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_WARNING;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3764431713U);
                Debug.Assert(pack.ind == (byte)(byte)112);
                Debug.Assert(pack.value == (float)5.098907E37F);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.value = (float)5.098907E37F;
            p254.ind = (byte)(byte)112;
            p254.time_boot_ms = (uint)3764431713U;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.initial_timestamp == (ulong)975100772862209442L);
                Debug.Assert(pack.target_component == (byte)(byte)165);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)98, (byte)146, (byte)73, (byte)28, (byte)75, (byte)134, (byte)248, (byte)90, (byte)57, (byte)58, (byte)12, (byte)126, (byte)110, (byte)149, (byte)184, (byte)83, (byte)28, (byte)17, (byte)83, (byte)41, (byte)37, (byte)52, (byte)90, (byte)23, (byte)151, (byte)29, (byte)157, (byte)72, (byte)132, (byte)115, (byte)147, (byte)46}));
                Debug.Assert(pack.target_system == (byte)(byte)150);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.target_system = (byte)(byte)150;
            p256.initial_timestamp = (ulong)975100772862209442L;
            p256.target_component = (byte)(byte)165;
            p256.secret_key_SET(new byte[] {(byte)98, (byte)146, (byte)73, (byte)28, (byte)75, (byte)134, (byte)248, (byte)90, (byte)57, (byte)58, (byte)12, (byte)126, (byte)110, (byte)149, (byte)184, (byte)83, (byte)28, (byte)17, (byte)83, (byte)41, (byte)37, (byte)52, (byte)90, (byte)23, (byte)151, (byte)29, (byte)157, (byte)72, (byte)132, (byte)115, (byte)147, (byte)46}, 0) ;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3650691740U);
                Debug.Assert(pack.state == (byte)(byte)205);
                Debug.Assert(pack.last_change_ms == (uint)425399274U);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.time_boot_ms = (uint)3650691740U;
            p257.state = (byte)(byte)205;
            p257.last_change_ms = (uint)425399274U;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)31);
                Debug.Assert(pack.target_component == (byte)(byte)141);
                Debug.Assert(pack.tune_LEN(ph) == 5);
                Debug.Assert(pack.tune_TRY(ph).Equals("jjjgw"));
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)31;
            p258.tune_SET("jjjgw", PH) ;
            p258.target_component = (byte)(byte)141;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)48909);
                Debug.Assert(pack.firmware_version == (uint)3191590538U);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)46130);
                Debug.Assert(pack.sensor_size_v == (float) -1.6277E38F);
                Debug.Assert(pack.focal_length == (float) -1.5119638E36F);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)162, (byte)182, (byte)124, (byte)8, (byte)39, (byte)10, (byte)62, (byte)27, (byte)207, (byte)136, (byte)32, (byte)168, (byte)38, (byte)110, (byte)171, (byte)184, (byte)132, (byte)68, (byte)57, (byte)138, (byte)206, (byte)63, (byte)115, (byte)223, (byte)179, (byte)83, (byte)23, (byte)178, (byte)204, (byte)252, (byte)234, (byte)120}));
                Debug.Assert(pack.sensor_size_h == (float) -2.1391079E38F);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO));
                Debug.Assert(pack.lens_id == (byte)(byte)10);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)250, (byte)115, (byte)81, (byte)187, (byte)9, (byte)29, (byte)103, (byte)199, (byte)146, (byte)110, (byte)233, (byte)8, (byte)237, (byte)135, (byte)61, (byte)115, (byte)169, (byte)197, (byte)181, (byte)188, (byte)132, (byte)11, (byte)26, (byte)0, (byte)244, (byte)144, (byte)154, (byte)216, (byte)9, (byte)115, (byte)89, (byte)243}));
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 90);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("qmehufqzizkElfiasrTnqfbvpdcutklyycToKQaNrBbgoXgftdfbbgcRynzcOrblkgcslyuoQSrXurpwhgudyjcaui"));
                Debug.Assert(pack.resolution_h == (ushort)(ushort)21516);
                Debug.Assert(pack.time_boot_ms == (uint)3102810665U);
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.resolution_v = (ushort)(ushort)46130;
            p259.focal_length = (float) -1.5119638E36F;
            p259.time_boot_ms = (uint)3102810665U;
            p259.firmware_version = (uint)3191590538U;
            p259.resolution_h = (ushort)(ushort)21516;
            p259.cam_definition_version = (ushort)(ushort)48909;
            p259.cam_definition_uri_SET("qmehufqzizkElfiasrTnqfbvpdcutklyycToKQaNrBbgoXgftdfbbgcRynzcOrblkgcslyuoQSrXurpwhgudyjcaui", PH) ;
            p259.sensor_size_v = (float) -1.6277E38F;
            p259.model_name_SET(new byte[] {(byte)250, (byte)115, (byte)81, (byte)187, (byte)9, (byte)29, (byte)103, (byte)199, (byte)146, (byte)110, (byte)233, (byte)8, (byte)237, (byte)135, (byte)61, (byte)115, (byte)169, (byte)197, (byte)181, (byte)188, (byte)132, (byte)11, (byte)26, (byte)0, (byte)244, (byte)144, (byte)154, (byte)216, (byte)9, (byte)115, (byte)89, (byte)243}, 0) ;
            p259.lens_id = (byte)(byte)10;
            p259.vendor_name_SET(new byte[] {(byte)162, (byte)182, (byte)124, (byte)8, (byte)39, (byte)10, (byte)62, (byte)27, (byte)207, (byte)136, (byte)32, (byte)168, (byte)38, (byte)110, (byte)171, (byte)184, (byte)132, (byte)68, (byte)57, (byte)138, (byte)206, (byte)63, (byte)115, (byte)223, (byte)179, (byte)83, (byte)23, (byte)178, (byte)204, (byte)252, (byte)234, (byte)120}, 0) ;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
            p259.sensor_size_h = (float) -2.1391079E38F;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2320052691U);
                Debug.Assert(pack.mode_id == (CAMERA_MODE.CAMERA_MODE_IMAGE |
                                              CAMERA_MODE.CAMERA_MODE_VIDEO));
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.mode_id = (CAMERA_MODE.CAMERA_MODE_IMAGE |
                            CAMERA_MODE.CAMERA_MODE_VIDEO);
            p260.time_boot_ms = (uint)2320052691U;
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.write_speed == (float)2.772744E38F);
                Debug.Assert(pack.available_capacity == (float)1.4201691E38F);
                Debug.Assert(pack.used_capacity == (float) -7.3808145E36F);
                Debug.Assert(pack.total_capacity == (float) -2.7688527E38F);
                Debug.Assert(pack.status == (byte)(byte)156);
                Debug.Assert(pack.storage_id == (byte)(byte)104);
                Debug.Assert(pack.time_boot_ms == (uint)1149143637U);
                Debug.Assert(pack.read_speed == (float) -1.3674644E38F);
                Debug.Assert(pack.storage_count == (byte)(byte)157);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.storage_count = (byte)(byte)157;
            p261.storage_id = (byte)(byte)104;
            p261.write_speed = (float)2.772744E38F;
            p261.read_speed = (float) -1.3674644E38F;
            p261.available_capacity = (float)1.4201691E38F;
            p261.total_capacity = (float) -2.7688527E38F;
            p261.status = (byte)(byte)156;
            p261.time_boot_ms = (uint)1149143637U;
            p261.used_capacity = (float) -7.3808145E36F;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.available_capacity == (float)3.0329127E38F);
                Debug.Assert(pack.recording_time_ms == (uint)797658364U);
                Debug.Assert(pack.image_status == (byte)(byte)69);
                Debug.Assert(pack.video_status == (byte)(byte)107);
                Debug.Assert(pack.time_boot_ms == (uint)1766060077U);
                Debug.Assert(pack.image_interval == (float) -2.2549809E38F);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.available_capacity = (float)3.0329127E38F;
            p262.video_status = (byte)(byte)107;
            p262.time_boot_ms = (uint)1766060077U;
            p262.image_interval = (float) -2.2549809E38F;
            p262.recording_time_ms = (uint)797658364U;
            p262.image_status = (byte)(byte)69;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.image_index == (int)2121174689);
                Debug.Assert(pack.lat == (int)1659455935);
                Debug.Assert(pack.file_url_LEN(ph) == 30);
                Debug.Assert(pack.file_url_TRY(ph).Equals("eLnohjcItiosywkeokvWnyawbHkbad"));
                Debug.Assert(pack.lon == (int) -202686024);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.5505617E38F, 2.966462E37F, -1.1369806E38F, -3.204057E38F}));
                Debug.Assert(pack.relative_alt == (int)1247249473);
                Debug.Assert(pack.alt == (int) -262889274);
                Debug.Assert(pack.time_utc == (ulong)612411757608533042L);
                Debug.Assert(pack.time_boot_ms == (uint)1353463514U);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)22);
                Debug.Assert(pack.camera_id == (byte)(byte)5);
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.relative_alt = (int)1247249473;
            p263.lat = (int)1659455935;
            p263.image_index = (int)2121174689;
            p263.lon = (int) -202686024;
            p263.file_url_SET("eLnohjcItiosywkeokvWnyawbHkbad", PH) ;
            p263.q_SET(new float[] {-2.5505617E38F, 2.966462E37F, -1.1369806E38F, -3.204057E38F}, 0) ;
            p263.alt = (int) -262889274;
            p263.time_utc = (ulong)612411757608533042L;
            p263.capture_result = (sbyte)(sbyte)22;
            p263.time_boot_ms = (uint)1353463514U;
            p263.camera_id = (byte)(byte)5;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flight_uuid == (ulong)842021632262643052L);
                Debug.Assert(pack.time_boot_ms == (uint)1714045549U);
                Debug.Assert(pack.arming_time_utc == (ulong)1058623998372478009L);
                Debug.Assert(pack.takeoff_time_utc == (ulong)6097965368325333896L);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)1714045549U;
            p264.arming_time_utc = (ulong)1058623998372478009L;
            p264.takeoff_time_utc = (ulong)6097965368325333896L;
            p264.flight_uuid = (ulong)842021632262643052L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float) -2.6814828E38F);
                Debug.Assert(pack.roll == (float) -2.957452E38F);
                Debug.Assert(pack.time_boot_ms == (uint)744522667U);
                Debug.Assert(pack.pitch == (float)3.8822013E37F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.roll = (float) -2.957452E38F;
            p265.yaw = (float) -2.6814828E38F;
            p265.pitch = (float)3.8822013E37F;
            p265.time_boot_ms = (uint)744522667U;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)6, (byte)142, (byte)75, (byte)173, (byte)23, (byte)35, (byte)92, (byte)145, (byte)235, (byte)96, (byte)160, (byte)149, (byte)53, (byte)226, (byte)0, (byte)29, (byte)242, (byte)200, (byte)187, (byte)21, (byte)190, (byte)72, (byte)86, (byte)14, (byte)0, (byte)109, (byte)12, (byte)72, (byte)164, (byte)71, (byte)10, (byte)163, (byte)88, (byte)221, (byte)152, (byte)203, (byte)99, (byte)235, (byte)76, (byte)103, (byte)152, (byte)159, (byte)102, (byte)11, (byte)99, (byte)70, (byte)179, (byte)194, (byte)236, (byte)30, (byte)1, (byte)126, (byte)205, (byte)164, (byte)216, (byte)21, (byte)126, (byte)215, (byte)90, (byte)136, (byte)211, (byte)218, (byte)16, (byte)17, (byte)194, (byte)206, (byte)96, (byte)239, (byte)160, (byte)250, (byte)7, (byte)231, (byte)193, (byte)133, (byte)59, (byte)62, (byte)19, (byte)82, (byte)126, (byte)246, (byte)120, (byte)95, (byte)12, (byte)58, (byte)180, (byte)181, (byte)216, (byte)251, (byte)243, (byte)70, (byte)125, (byte)100, (byte)1, (byte)193, (byte)236, (byte)228, (byte)34, (byte)127, (byte)116, (byte)165, (byte)98, (byte)238, (byte)179, (byte)187, (byte)254, (byte)3, (byte)200, (byte)150, (byte)55, (byte)127, (byte)253, (byte)136, (byte)171, (byte)159, (byte)255, (byte)217, (byte)252, (byte)32, (byte)185, (byte)193, (byte)185, (byte)234, (byte)46, (byte)30, (byte)149, (byte)220, (byte)126, (byte)205, (byte)11, (byte)39, (byte)145, (byte)63, (byte)21, (byte)237, (byte)219, (byte)18, (byte)242, (byte)117, (byte)229, (byte)144, (byte)173, (byte)239, (byte)107, (byte)41, (byte)188, (byte)219, (byte)108, (byte)38, (byte)131, (byte)36, (byte)141, (byte)99, (byte)77, (byte)141, (byte)80, (byte)113, (byte)24, (byte)109, (byte)84, (byte)84, (byte)243, (byte)117, (byte)230, (byte)57, (byte)42, (byte)26, (byte)133, (byte)92, (byte)119, (byte)234, (byte)89, (byte)213, (byte)84, (byte)148, (byte)217, (byte)21, (byte)13, (byte)200, (byte)134, (byte)145, (byte)91, (byte)197, (byte)66, (byte)110, (byte)35, (byte)124, (byte)94, (byte)212, (byte)36, (byte)82, (byte)161, (byte)24, (byte)223, (byte)85, (byte)196, (byte)226, (byte)92, (byte)173, (byte)229, (byte)69, (byte)165, (byte)200, (byte)203, (byte)126, (byte)132, (byte)201, (byte)246, (byte)60, (byte)33, (byte)2, (byte)87, (byte)193, (byte)111, (byte)23, (byte)215, (byte)191, (byte)100, (byte)146, (byte)217, (byte)111, (byte)80, (byte)79, (byte)246, (byte)199, (byte)74, (byte)171, (byte)213, (byte)7, (byte)215, (byte)14, (byte)168, (byte)247, (byte)241, (byte)159, (byte)99, (byte)43, (byte)96, (byte)246, (byte)207, (byte)72, (byte)135, (byte)78, (byte)206, (byte)0, (byte)166, (byte)236, (byte)123, (byte)122, (byte)4}));
                Debug.Assert(pack.sequence == (ushort)(ushort)57582);
                Debug.Assert(pack.length == (byte)(byte)126);
                Debug.Assert(pack.target_component == (byte)(byte)245);
                Debug.Assert(pack.first_message_offset == (byte)(byte)122);
                Debug.Assert(pack.target_system == (byte)(byte)197);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.data__SET(new byte[] {(byte)6, (byte)142, (byte)75, (byte)173, (byte)23, (byte)35, (byte)92, (byte)145, (byte)235, (byte)96, (byte)160, (byte)149, (byte)53, (byte)226, (byte)0, (byte)29, (byte)242, (byte)200, (byte)187, (byte)21, (byte)190, (byte)72, (byte)86, (byte)14, (byte)0, (byte)109, (byte)12, (byte)72, (byte)164, (byte)71, (byte)10, (byte)163, (byte)88, (byte)221, (byte)152, (byte)203, (byte)99, (byte)235, (byte)76, (byte)103, (byte)152, (byte)159, (byte)102, (byte)11, (byte)99, (byte)70, (byte)179, (byte)194, (byte)236, (byte)30, (byte)1, (byte)126, (byte)205, (byte)164, (byte)216, (byte)21, (byte)126, (byte)215, (byte)90, (byte)136, (byte)211, (byte)218, (byte)16, (byte)17, (byte)194, (byte)206, (byte)96, (byte)239, (byte)160, (byte)250, (byte)7, (byte)231, (byte)193, (byte)133, (byte)59, (byte)62, (byte)19, (byte)82, (byte)126, (byte)246, (byte)120, (byte)95, (byte)12, (byte)58, (byte)180, (byte)181, (byte)216, (byte)251, (byte)243, (byte)70, (byte)125, (byte)100, (byte)1, (byte)193, (byte)236, (byte)228, (byte)34, (byte)127, (byte)116, (byte)165, (byte)98, (byte)238, (byte)179, (byte)187, (byte)254, (byte)3, (byte)200, (byte)150, (byte)55, (byte)127, (byte)253, (byte)136, (byte)171, (byte)159, (byte)255, (byte)217, (byte)252, (byte)32, (byte)185, (byte)193, (byte)185, (byte)234, (byte)46, (byte)30, (byte)149, (byte)220, (byte)126, (byte)205, (byte)11, (byte)39, (byte)145, (byte)63, (byte)21, (byte)237, (byte)219, (byte)18, (byte)242, (byte)117, (byte)229, (byte)144, (byte)173, (byte)239, (byte)107, (byte)41, (byte)188, (byte)219, (byte)108, (byte)38, (byte)131, (byte)36, (byte)141, (byte)99, (byte)77, (byte)141, (byte)80, (byte)113, (byte)24, (byte)109, (byte)84, (byte)84, (byte)243, (byte)117, (byte)230, (byte)57, (byte)42, (byte)26, (byte)133, (byte)92, (byte)119, (byte)234, (byte)89, (byte)213, (byte)84, (byte)148, (byte)217, (byte)21, (byte)13, (byte)200, (byte)134, (byte)145, (byte)91, (byte)197, (byte)66, (byte)110, (byte)35, (byte)124, (byte)94, (byte)212, (byte)36, (byte)82, (byte)161, (byte)24, (byte)223, (byte)85, (byte)196, (byte)226, (byte)92, (byte)173, (byte)229, (byte)69, (byte)165, (byte)200, (byte)203, (byte)126, (byte)132, (byte)201, (byte)246, (byte)60, (byte)33, (byte)2, (byte)87, (byte)193, (byte)111, (byte)23, (byte)215, (byte)191, (byte)100, (byte)146, (byte)217, (byte)111, (byte)80, (byte)79, (byte)246, (byte)199, (byte)74, (byte)171, (byte)213, (byte)7, (byte)215, (byte)14, (byte)168, (byte)247, (byte)241, (byte)159, (byte)99, (byte)43, (byte)96, (byte)246, (byte)207, (byte)72, (byte)135, (byte)78, (byte)206, (byte)0, (byte)166, (byte)236, (byte)123, (byte)122, (byte)4}, 0) ;
            p266.target_component = (byte)(byte)245;
            p266.target_system = (byte)(byte)197;
            p266.first_message_offset = (byte)(byte)122;
            p266.length = (byte)(byte)126;
            p266.sequence = (ushort)(ushort)57582;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)201);
                Debug.Assert(pack.sequence == (ushort)(ushort)36271);
                Debug.Assert(pack.first_message_offset == (byte)(byte)69);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)172, (byte)229, (byte)161, (byte)213, (byte)122, (byte)51, (byte)234, (byte)183, (byte)179, (byte)39, (byte)226, (byte)8, (byte)55, (byte)232, (byte)19, (byte)249, (byte)125, (byte)4, (byte)0, (byte)22, (byte)217, (byte)99, (byte)49, (byte)150, (byte)141, (byte)22, (byte)249, (byte)97, (byte)236, (byte)42, (byte)122, (byte)196, (byte)125, (byte)244, (byte)169, (byte)80, (byte)105, (byte)202, (byte)168, (byte)51, (byte)119, (byte)50, (byte)43, (byte)239, (byte)30, (byte)185, (byte)140, (byte)6, (byte)68, (byte)253, (byte)144, (byte)77, (byte)15, (byte)200, (byte)154, (byte)102, (byte)247, (byte)77, (byte)86, (byte)54, (byte)185, (byte)68, (byte)243, (byte)30, (byte)217, (byte)7, (byte)89, (byte)53, (byte)142, (byte)157, (byte)38, (byte)18, (byte)161, (byte)33, (byte)202, (byte)228, (byte)186, (byte)77, (byte)224, (byte)13, (byte)114, (byte)115, (byte)208, (byte)183, (byte)166, (byte)245, (byte)41, (byte)186, (byte)35, (byte)107, (byte)112, (byte)220, (byte)187, (byte)230, (byte)154, (byte)224, (byte)159, (byte)10, (byte)92, (byte)138, (byte)88, (byte)223, (byte)136, (byte)64, (byte)3, (byte)57, (byte)176, (byte)45, (byte)197, (byte)226, (byte)77, (byte)153, (byte)125, (byte)64, (byte)146, (byte)195, (byte)8, (byte)173, (byte)89, (byte)127, (byte)27, (byte)102, (byte)112, (byte)239, (byte)206, (byte)121, (byte)154, (byte)138, (byte)33, (byte)129, (byte)167, (byte)129, (byte)84, (byte)68, (byte)4, (byte)239, (byte)19, (byte)192, (byte)123, (byte)111, (byte)247, (byte)136, (byte)17, (byte)23, (byte)158, (byte)166, (byte)176, (byte)212, (byte)164, (byte)251, (byte)99, (byte)168, (byte)183, (byte)132, (byte)98, (byte)165, (byte)175, (byte)17, (byte)214, (byte)143, (byte)171, (byte)224, (byte)32, (byte)10, (byte)143, (byte)84, (byte)151, (byte)170, (byte)181, (byte)18, (byte)82, (byte)1, (byte)177, (byte)207, (byte)240, (byte)20, (byte)3, (byte)127, (byte)106, (byte)115, (byte)124, (byte)87, (byte)13, (byte)34, (byte)200, (byte)204, (byte)81, (byte)166, (byte)151, (byte)32, (byte)119, (byte)23, (byte)199, (byte)64, (byte)106, (byte)16, (byte)78, (byte)144, (byte)101, (byte)242, (byte)15, (byte)105, (byte)246, (byte)125, (byte)163, (byte)233, (byte)8, (byte)57, (byte)10, (byte)233, (byte)94, (byte)193, (byte)216, (byte)190, (byte)158, (byte)223, (byte)88, (byte)105, (byte)122, (byte)51, (byte)67, (byte)188, (byte)113, (byte)72, (byte)218, (byte)200, (byte)57, (byte)48, (byte)30, (byte)224, (byte)233, (byte)12, (byte)206, (byte)246, (byte)43, (byte)71, (byte)229, (byte)131, (byte)187, (byte)88, (byte)78, (byte)32, (byte)142, (byte)21, (byte)167, (byte)150, (byte)147, (byte)123, (byte)108}));
                Debug.Assert(pack.target_system == (byte)(byte)19);
                Debug.Assert(pack.length == (byte)(byte)159);
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.target_component = (byte)(byte)201;
            p267.first_message_offset = (byte)(byte)69;
            p267.length = (byte)(byte)159;
            p267.data__SET(new byte[] {(byte)172, (byte)229, (byte)161, (byte)213, (byte)122, (byte)51, (byte)234, (byte)183, (byte)179, (byte)39, (byte)226, (byte)8, (byte)55, (byte)232, (byte)19, (byte)249, (byte)125, (byte)4, (byte)0, (byte)22, (byte)217, (byte)99, (byte)49, (byte)150, (byte)141, (byte)22, (byte)249, (byte)97, (byte)236, (byte)42, (byte)122, (byte)196, (byte)125, (byte)244, (byte)169, (byte)80, (byte)105, (byte)202, (byte)168, (byte)51, (byte)119, (byte)50, (byte)43, (byte)239, (byte)30, (byte)185, (byte)140, (byte)6, (byte)68, (byte)253, (byte)144, (byte)77, (byte)15, (byte)200, (byte)154, (byte)102, (byte)247, (byte)77, (byte)86, (byte)54, (byte)185, (byte)68, (byte)243, (byte)30, (byte)217, (byte)7, (byte)89, (byte)53, (byte)142, (byte)157, (byte)38, (byte)18, (byte)161, (byte)33, (byte)202, (byte)228, (byte)186, (byte)77, (byte)224, (byte)13, (byte)114, (byte)115, (byte)208, (byte)183, (byte)166, (byte)245, (byte)41, (byte)186, (byte)35, (byte)107, (byte)112, (byte)220, (byte)187, (byte)230, (byte)154, (byte)224, (byte)159, (byte)10, (byte)92, (byte)138, (byte)88, (byte)223, (byte)136, (byte)64, (byte)3, (byte)57, (byte)176, (byte)45, (byte)197, (byte)226, (byte)77, (byte)153, (byte)125, (byte)64, (byte)146, (byte)195, (byte)8, (byte)173, (byte)89, (byte)127, (byte)27, (byte)102, (byte)112, (byte)239, (byte)206, (byte)121, (byte)154, (byte)138, (byte)33, (byte)129, (byte)167, (byte)129, (byte)84, (byte)68, (byte)4, (byte)239, (byte)19, (byte)192, (byte)123, (byte)111, (byte)247, (byte)136, (byte)17, (byte)23, (byte)158, (byte)166, (byte)176, (byte)212, (byte)164, (byte)251, (byte)99, (byte)168, (byte)183, (byte)132, (byte)98, (byte)165, (byte)175, (byte)17, (byte)214, (byte)143, (byte)171, (byte)224, (byte)32, (byte)10, (byte)143, (byte)84, (byte)151, (byte)170, (byte)181, (byte)18, (byte)82, (byte)1, (byte)177, (byte)207, (byte)240, (byte)20, (byte)3, (byte)127, (byte)106, (byte)115, (byte)124, (byte)87, (byte)13, (byte)34, (byte)200, (byte)204, (byte)81, (byte)166, (byte)151, (byte)32, (byte)119, (byte)23, (byte)199, (byte)64, (byte)106, (byte)16, (byte)78, (byte)144, (byte)101, (byte)242, (byte)15, (byte)105, (byte)246, (byte)125, (byte)163, (byte)233, (byte)8, (byte)57, (byte)10, (byte)233, (byte)94, (byte)193, (byte)216, (byte)190, (byte)158, (byte)223, (byte)88, (byte)105, (byte)122, (byte)51, (byte)67, (byte)188, (byte)113, (byte)72, (byte)218, (byte)200, (byte)57, (byte)48, (byte)30, (byte)224, (byte)233, (byte)12, (byte)206, (byte)246, (byte)43, (byte)71, (byte)229, (byte)131, (byte)187, (byte)88, (byte)78, (byte)32, (byte)142, (byte)21, (byte)167, (byte)150, (byte)147, (byte)123, (byte)108}, 0) ;
            p267.sequence = (ushort)(ushort)36271;
            p267.target_system = (byte)(byte)19;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)113);
                Debug.Assert(pack.sequence == (ushort)(ushort)49619);
                Debug.Assert(pack.target_component == (byte)(byte)218);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_system = (byte)(byte)113;
            p268.sequence = (ushort)(ushort)49619;
            p268.target_component = (byte)(byte)218;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.resolution_v == (ushort)(ushort)51852);
                Debug.Assert(pack.uri_LEN(ph) == 23);
                Debug.Assert(pack.uri_TRY(ph).Equals("xbsoixTzkbnpjgotoukosuO"));
                Debug.Assert(pack.bitrate == (uint)907696722U);
                Debug.Assert(pack.rotation == (ushort)(ushort)13915);
                Debug.Assert(pack.framerate == (float)9.544466E37F);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)56397);
                Debug.Assert(pack.status == (byte)(byte)179);
                Debug.Assert(pack.camera_id == (byte)(byte)227);
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.framerate = (float)9.544466E37F;
            p269.uri_SET("xbsoixTzkbnpjgotoukosuO", PH) ;
            p269.resolution_v = (ushort)(ushort)51852;
            p269.resolution_h = (ushort)(ushort)56397;
            p269.bitrate = (uint)907696722U;
            p269.camera_id = (byte)(byte)227;
            p269.rotation = (ushort)(ushort)13915;
            p269.status = (byte)(byte)179;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.bitrate == (uint)3574133052U);
                Debug.Assert(pack.target_component == (byte)(byte)221);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)48373);
                Debug.Assert(pack.uri_LEN(ph) == 14);
                Debug.Assert(pack.uri_TRY(ph).Equals("uvbfliirhkhojq"));
                Debug.Assert(pack.camera_id == (byte)(byte)13);
                Debug.Assert(pack.target_system == (byte)(byte)250);
                Debug.Assert(pack.framerate == (float)4.395946E37F);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)61650);
                Debug.Assert(pack.rotation == (ushort)(ushort)51436);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.rotation = (ushort)(ushort)51436;
            p270.uri_SET("uvbfliirhkhojq", PH) ;
            p270.resolution_v = (ushort)(ushort)61650;
            p270.framerate = (float)4.395946E37F;
            p270.bitrate = (uint)3574133052U;
            p270.target_component = (byte)(byte)221;
            p270.target_system = (byte)(byte)250;
            p270.camera_id = (byte)(byte)13;
            p270.resolution_h = (ushort)(ushort)48373;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 50);
                Debug.Assert(pack.password_TRY(ph).Equals("eXzmdjlzysdubpesbgxrirwkygLiygvolJephfnkwcdEWesizf"));
                Debug.Assert(pack.ssid_LEN(ph) == 26);
                Debug.Assert(pack.ssid_TRY(ph).Equals("PPpwqovRjuAroavuchawdvySnO"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("PPpwqovRjuAroavuchawdvySnO", PH) ;
            p299.password_SET("eXzmdjlzysdubpesbgxrirwkygLiygvolJephfnkwcdEWesizf", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.max_version == (ushort)(ushort)31594);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)26, (byte)239, (byte)185, (byte)194, (byte)150, (byte)243, (byte)248, (byte)236}));
                Debug.Assert(pack.min_version == (ushort)(ushort)43724);
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)59, (byte)182, (byte)133, (byte)250, (byte)46, (byte)30, (byte)71, (byte)247}));
                Debug.Assert(pack.version == (ushort)(ushort)43694);
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.max_version = (ushort)(ushort)31594;
            p300.spec_version_hash_SET(new byte[] {(byte)26, (byte)239, (byte)185, (byte)194, (byte)150, (byte)243, (byte)248, (byte)236}, 0) ;
            p300.library_version_hash_SET(new byte[] {(byte)59, (byte)182, (byte)133, (byte)250, (byte)46, (byte)30, (byte)71, (byte)247}, 0) ;
            p300.version = (ushort)(ushort)43694;
            p300.min_version = (ushort)(ushort)43724;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uptime_sec == (uint)3002502445U);
                Debug.Assert(pack.mode == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION);
                Debug.Assert(pack.health == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
                Debug.Assert(pack.time_usec == (ulong)5972406204292063758L);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)44240);
                Debug.Assert(pack.sub_mode == (byte)(byte)51);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.vendor_specific_status_code = (ushort)(ushort)44240;
            p310.sub_mode = (byte)(byte)51;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_INITIALIZATION;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING;
            p310.uptime_sec = (uint)3002502445U;
            p310.time_usec = (ulong)5972406204292063758L;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sw_version_minor == (byte)(byte)247);
                Debug.Assert(pack.hw_version_major == (byte)(byte)71);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)175);
                Debug.Assert(pack.sw_vcs_commit == (uint)3645604935U);
                Debug.Assert(pack.uptime_sec == (uint)1754848453U);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)152, (byte)188, (byte)51, (byte)171, (byte)111, (byte)92, (byte)183, (byte)95, (byte)2, (byte)13, (byte)162, (byte)135, (byte)73, (byte)49, (byte)172, (byte)68}));
                Debug.Assert(pack.sw_version_major == (byte)(byte)249);
                Debug.Assert(pack.time_usec == (ulong)6393886469436745850L);
                Debug.Assert(pack.name_LEN(ph) == 20);
                Debug.Assert(pack.name_TRY(ph).Equals("bqirbbimdjlRffsospvN"));
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.hw_version_minor = (byte)(byte)175;
            p311.time_usec = (ulong)6393886469436745850L;
            p311.sw_vcs_commit = (uint)3645604935U;
            p311.uptime_sec = (uint)1754848453U;
            p311.hw_version_major = (byte)(byte)71;
            p311.name_SET("bqirbbimdjlRffsospvN", PH) ;
            p311.sw_version_minor = (byte)(byte)247;
            p311.hw_unique_id_SET(new byte[] {(byte)152, (byte)188, (byte)51, (byte)171, (byte)111, (byte)92, (byte)183, (byte)95, (byte)2, (byte)13, (byte)162, (byte)135, (byte)73, (byte)49, (byte)172, (byte)68}, 0) ;
            p311.sw_version_major = (byte)(byte)249;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (short)(short)27544);
                Debug.Assert(pack.param_id_LEN(ph) == 5);
                Debug.Assert(pack.param_id_TRY(ph).Equals("vyftj"));
                Debug.Assert(pack.target_system == (byte)(byte)79);
                Debug.Assert(pack.target_component == (byte)(byte)34);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_component = (byte)(byte)34;
            p320.param_index = (short)(short)27544;
            p320.param_id_SET("vyftj", PH) ;
            p320.target_system = (byte)(byte)79;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)0);
                Debug.Assert(pack.target_system == (byte)(byte)253);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_system = (byte)(byte)253;
            p321.target_component = (byte)(byte)0;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (ushort)(ushort)54752);
                Debug.Assert(pack.param_value_LEN(ph) == 57);
                Debug.Assert(pack.param_value_TRY(ph).Equals("oytNRytkBvxhBjtkkpaPwtenorslovamylqczlirzchmfbmuagwuqnLlq"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
                Debug.Assert(pack.param_id_LEN(ph) == 8);
                Debug.Assert(pack.param_id_TRY(ph).Equals("mthxjdqk"));
                Debug.Assert(pack.param_count == (ushort)(ushort)61215);
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM;
            p322.param_id_SET("mthxjdqk", PH) ;
            p322.param_count = (ushort)(ushort)61215;
            p322.param_value_SET("oytNRytkBvxhBjtkkpaPwtenorslovamylqczlirzchmfbmuagwuqnLlq", PH) ;
            p322.param_index = (ushort)(ushort)54752;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)129);
                Debug.Assert(pack.param_value_LEN(ph) == 122);
                Debug.Assert(pack.param_value_TRY(ph).Equals("QgxwZQZrehnldeqndxkeqxvkvQudspgDbqeStcbVxjyahqwzkmmqbnendgzvwyuhheqikudjhdRmybFoPfxwvryoicIgxdygluvegkxyhtygyFspsxjotuKyiz"));
                Debug.Assert(pack.target_component == (byte)(byte)137);
                Debug.Assert(pack.param_id_LEN(ph) == 16);
                Debug.Assert(pack.param_id_TRY(ph).Equals("pifjvoowtlfsgkfg"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.target_system = (byte)(byte)129;
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64;
            p323.target_component = (byte)(byte)137;
            p323.param_value_SET("QgxwZQZrehnldeqndxkeqxvkvQudspgDbqeStcbVxjyahqwzkmmqbnendgzvwyuhheqikudjhdRmybFoPfxwvryoicIgxdygluvegkxyhtygyFspsxjotuKyiz", PH) ;
            p323.param_id_SET("pifjvoowtlfsgkfg", PH) ;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
                Debug.Assert(pack.param_result == PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
                Debug.Assert(pack.param_value_LEN(ph) == 64);
                Debug.Assert(pack.param_value_TRY(ph).Equals("imjpglgljlsqwiamqpmdwtqtJmizjuostivDnafsVtzkvlqatfjjlHjeffcufIhx"));
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("Uey"));
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32;
            p324.param_result = PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED;
            p324.param_value_SET("imjpglgljlsqwiamqpmdwtqtJmizjuostivDnafsVtzkvlqatfjjlHjeffcufIhx", PH) ;
            p324.param_id_SET("Uey", PH) ;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)51962, (ushort)6902, (ushort)102, (ushort)25149, (ushort)30183, (ushort)19499, (ushort)19350, (ushort)50166, (ushort)22028, (ushort)46234, (ushort)3695, (ushort)8375, (ushort)11634, (ushort)56722, (ushort)5032, (ushort)34127, (ushort)42811, (ushort)59627, (ushort)4294, (ushort)11797, (ushort)3297, (ushort)40177, (ushort)31164, (ushort)37290, (ushort)16501, (ushort)19881, (ushort)29917, (ushort)14495, (ushort)35826, (ushort)32441, (ushort)64950, (ushort)56314, (ushort)45067, (ushort)5291, (ushort)30595, (ushort)45630, (ushort)52604, (ushort)34815, (ushort)28946, (ushort)20062, (ushort)33665, (ushort)27535, (ushort)39658, (ushort)49350, (ushort)60373, (ushort)36894, (ushort)37242, (ushort)5441, (ushort)20352, (ushort)6018, (ushort)59125, (ushort)5861, (ushort)44255, (ushort)21078, (ushort)34064, (ushort)54895, (ushort)55231, (ushort)21931, (ushort)46913, (ushort)57098, (ushort)49103, (ushort)22559, (ushort)37594, (ushort)22351, (ushort)47864, (ushort)55130, (ushort)54742, (ushort)11115, (ushort)4049, (ushort)1778, (ushort)23153, (ushort)49238}));
                Debug.Assert(pack.increment == (byte)(byte)16);
                Debug.Assert(pack.sensor_type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
                Debug.Assert(pack.max_distance == (ushort)(ushort)48890);
                Debug.Assert(pack.time_usec == (ulong)4655436327686673715L);
                Debug.Assert(pack.min_distance == (ushort)(ushort)47422);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.increment = (byte)(byte)16;
            p330.time_usec = (ulong)4655436327686673715L;
            p330.min_distance = (ushort)(ushort)47422;
            p330.max_distance = (ushort)(ushort)48890;
            p330.distances_SET(new ushort[] {(ushort)51962, (ushort)6902, (ushort)102, (ushort)25149, (ushort)30183, (ushort)19499, (ushort)19350, (ushort)50166, (ushort)22028, (ushort)46234, (ushort)3695, (ushort)8375, (ushort)11634, (ushort)56722, (ushort)5032, (ushort)34127, (ushort)42811, (ushort)59627, (ushort)4294, (ushort)11797, (ushort)3297, (ushort)40177, (ushort)31164, (ushort)37290, (ushort)16501, (ushort)19881, (ushort)29917, (ushort)14495, (ushort)35826, (ushort)32441, (ushort)64950, (ushort)56314, (ushort)45067, (ushort)5291, (ushort)30595, (ushort)45630, (ushort)52604, (ushort)34815, (ushort)28946, (ushort)20062, (ushort)33665, (ushort)27535, (ushort)39658, (ushort)49350, (ushort)60373, (ushort)36894, (ushort)37242, (ushort)5441, (ushort)20352, (ushort)6018, (ushort)59125, (ushort)5861, (ushort)44255, (ushort)21078, (ushort)34064, (ushort)54895, (ushort)55231, (ushort)21931, (ushort)46913, (ushort)57098, (ushort)49103, (ushort)22559, (ushort)37594, (ushort)22351, (ushort)47864, (ushort)55130, (ushort)54742, (ushort)11115, (ushort)4049, (ushort)1778, (ushort)23153, (ushort)49238}, 0) ;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, ADV_TEST_CH);
        }
    }
}