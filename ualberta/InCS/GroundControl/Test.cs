
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
                    ulong id = id__D(value);
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
                    ulong id = id__C(value);
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
                    ulong id = id__C(value);
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
                    ulong id = id__C(value);
                    BitUtils.set_bits(id, 7, data, 260);
                }
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
        new class NAV_FILTER_BIAS : GroundControl.NAV_FILTER_BIAS
        {
            public ulong usec //Timestamp (microseconds)
            {
                get {  return (BitUtils.get_bytes(data,  0, 8));}
            }

            public float accel_0 //b_f[0]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  8, 4)));}
            }

            public float accel_1 //b_f[1]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  12, 4)));}
            }

            public float accel_2 //b_f[2]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  16, 4)));}
            }

            public float gyro_0 //b_f[0]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  20, 4)));}
            }

            public float gyro_1 //b_f[1]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  24, 4)));}
            }

            public float gyro_2 //b_f[2]
            {
                get {  return (float)(BitUtils.Int32BitsToFloat((uint) BitUtils.get_bytes(data,  28, 4)));}
            }
        }
        new class RADIO_CALIBRATION : GroundControl.RADIO_CALIBRATION
        {
            public ushort[] aileron //Aileron setpoints: left, center, right
            {
                get {return aileron_GET(new ushort[3], 0);}
            }
            public ushort[]aileron_GET(ushort[] dst_ch, int pos)  //Aileron setpoints: left, center, right
            {
                for(int BYTE = 0, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public ushort[] elevator //Elevator setpoints: nose down, center, nose up
            {
                get {return elevator_GET(new ushort[3], 0);}
            }
            public ushort[]elevator_GET(ushort[] dst_ch, int pos)  //Elevator setpoints: nose down, center, nose up
            {
                for(int BYTE = 6, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public ushort[] rudder //Rudder setpoints: nose left, center, nose right
            {
                get {return rudder_GET(new ushort[3], 0);}
            }
            public ushort[]rudder_GET(ushort[] dst_ch, int pos)  //Rudder setpoints: nose left, center, nose right
            {
                for(int BYTE = 12, dst_max = pos + 3; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public ushort[] gyro //Tail gyro mode/gain setpoints: heading hold, rate mode
            {
                get {return gyro_GET(new ushort[2], 0);}
            }
            public ushort[]gyro_GET(ushort[] dst_ch, int pos)  //Tail gyro mode/gain setpoints: heading hold, rate mode
            {
                for(int BYTE = 18, dst_max = pos + 2; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public ushort[] pitch //Pitch curve setpoints (every 25%)
            {
                get {return pitch_GET(new ushort[5], 0);}
            }
            public ushort[]pitch_GET(ushort[] dst_ch, int pos)  //Pitch curve setpoints (every 25%)
            {
                for(int BYTE = 22, dst_max = pos + 5; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
            public ushort[] throttle //Throttle curve setpoints (every 25%)
            {
                get {return throttle_GET(new ushort[5], 0);}
            }
            public ushort[]throttle_GET(ushort[] dst_ch, int pos)  //Throttle curve setpoints (every 25%)
            {
                for(int BYTE = 32, dst_max = pos + 5; pos < dst_max ; pos++,  BYTE += 2)
                    dst_ch[pos] = (ushort)((ushort) BitUtils.get_bytes(data,  BYTE, 2));
                return dst_ch;
            }
        }
        new class UALBERTA_SYS_STATUS : GroundControl.UALBERTA_SYS_STATUS
        {
            public byte mode //System mode, see UALBERTA_AUTOPILOT_MODE ENUM
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  0, 1));}
            }

            public byte nav_mode //Navigation mode, see UALBERTA_NAV_MODE ENUM
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  1, 1));}
            }

            public byte pilot //Pilot mode, see UALBERTA_PILOT_MODE
            {
                get {  return (byte)((byte) BitUtils.get_bytes(data,  2, 1));}
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
            public void OnNAV_FILTER_BIASReceive_direct(Channel src, Inside ph, NAV_FILTER_BIAS pack) {OnNAV_FILTER_BIASReceive(this, ph,  pack);}
            public event NAV_FILTER_BIASReceiveHandler OnNAV_FILTER_BIASReceive;
            public delegate void NAV_FILTER_BIASReceiveHandler(Channel src, Inside ph, NAV_FILTER_BIAS pack);
            public void OnRADIO_CALIBRATIONReceive_direct(Channel src, Inside ph, RADIO_CALIBRATION pack) {OnRADIO_CALIBRATIONReceive(this, ph,  pack);}
            public event RADIO_CALIBRATIONReceiveHandler OnRADIO_CALIBRATIONReceive;
            public delegate void RADIO_CALIBRATIONReceiveHandler(Channel src, Inside ph, RADIO_CALIBRATION pack);
            public void OnUALBERTA_SYS_STATUSReceive_direct(Channel src, Inside ph, UALBERTA_SYS_STATUS pack) {OnUALBERTA_SYS_STATUSReceive(this, ph,  pack);}
            public event UALBERTA_SYS_STATUSReceiveHandler OnUALBERTA_SYS_STATUSReceive;
            public delegate void UALBERTA_SYS_STATUSReceiveHandler(Channel src, Inside ph, UALBERTA_SYS_STATUS pack);
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
                    case 220:
                        if(pack == null) return new NAV_FILTER_BIAS();
                        OnNAV_FILTER_BIASReceive(this, ph, (NAV_FILTER_BIAS) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 221:
                        if(pack == null) return new RADIO_CALIBRATION();
                        OnRADIO_CALIBRATIONReceive(this, ph, (RADIO_CALIBRATION) pack);//no any host channels can receive this pack. Handle it with test channel handler
                        break;
                    case 222:
                        if(pack == null) return new UALBERTA_SYS_STATUS();
                        OnUALBERTA_SYS_STATUSReceive(this, ph, (UALBERTA_SYS_STATUS) pack);//no any host channels can receive this pack. Handle it with test channel handler
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
                Debug.Assert(pack.autopilot == MAV_AUTOPILOT.MAV_AUTOPILOT_OPENPILOT);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
                Debug.Assert(pack.type == MAV_TYPE.MAV_TYPE_AIRSHIP);
                Debug.Assert(pack.mavlink_version == (byte)(byte)197);
                Debug.Assert(pack.system_status == MAV_STATE.MAV_STATE_STANDBY);
                Debug.Assert(pack.custom_mode == (uint)1490160644U);
            };
            HEARTBEAT p0 = new HEARTBEAT();
            PH.setPack(p0);
            p0.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
            p0.mavlink_version = (byte)(byte)197;
            p0.system_status = MAV_STATE.MAV_STATE_STANDBY;
            p0.type = MAV_TYPE.MAV_TYPE_AIRSHIP;
            p0.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_OPENPILOT;
            p0.custom_mode = (uint)1490160644U;
            SMP_TEST_CH.send(p0);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_battery == (short)(short)21827);
                Debug.Assert(pack.errors_count1 == (ushort)(ushort)56435);
                Debug.Assert(pack.errors_count3 == (ushort)(ushort)63756);
                Debug.Assert(pack.errors_count4 == (ushort)(ushort)42922);
                Debug.Assert(pack.voltage_battery == (ushort)(ushort)14814);
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 71);
                Debug.Assert(pack.onboard_control_sensors_enabled == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH));
                Debug.Assert(pack.drop_rate_comm == (ushort)(ushort)21091);
                Debug.Assert(pack.errors_count2 == (ushort)(ushort)17851);
                Debug.Assert(pack.onboard_control_sensors_present == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
                Debug.Assert(pack.load == (ushort)(ushort)10357);
                Debug.Assert(pack.errors_comm == (ushort)(ushort)46359);
                Debug.Assert(pack.onboard_control_sensors_health == (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                             MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE));
            };
            SYS_STATUS p1 = new SYS_STATUS();
            PH.setPack(p1);
            p1.drop_rate_comm = (ushort)(ushort)21091;
            p1.errors_count4 = (ushort)(ushort)42922;
            p1.onboard_control_sensors_present = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_AHRS |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.errors_count3 = (ushort)(ushort)63756;
            p1.battery_remaining = (sbyte)(sbyte) - 71;
            p1.errors_count1 = (ushort)(ushort)56435;
            p1.onboard_control_sensors_enabled = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                  MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH);
            p1.load = (ushort)(ushort)10357;
            p1.current_battery = (short)(short)21827;
            p1.onboard_control_sensors_health = (MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_LOGGING |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY |
                                                 MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
            p1.voltage_battery = (ushort)(ushort)14814;
            p1.errors_comm = (ushort)(ushort)46359;
            p1.errors_count2 = (ushort)(ushort)17851;
            SMP_TEST_CH.send(p1);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSYSTEM_TIMEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_unix_usec == (ulong)1340918520997934046L);
                Debug.Assert(pack.time_boot_ms == (uint)3244576607U);
            };
            SYSTEM_TIME p2 = new SYSTEM_TIME();
            PH.setPack(p2);
            p2.time_boot_ms = (uint)3244576607U;
            p2.time_unix_usec = (ulong)1340918520997934046L;
            SMP_TEST_CH.send(p2);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float)1.4240329E38F);
                Debug.Assert(pack.afy == (float)1.6243734E38F);
                Debug.Assert(pack.yaw_rate == (float)3.2716151E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)21401);
                Debug.Assert(pack.z == (float)1.9801264E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3739702083U);
                Debug.Assert(pack.y == (float)2.9924337E38F);
                Debug.Assert(pack.vy == (float)1.617541E38F);
                Debug.Assert(pack.yaw == (float)2.4462278E38F);
                Debug.Assert(pack.afx == (float) -1.0565379E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.vz == (float) -9.043769E37F);
                Debug.Assert(pack.x == (float)2.124356E38F);
                Debug.Assert(pack.afz == (float)7.2538846E37F);
            };
            GroundControl.POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.new_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p3);
            p3.time_boot_ms = (uint)3739702083U;
            p3.y = (float)2.9924337E38F;
            p3.yaw_rate = (float)3.2716151E38F;
            p3.vx = (float)1.4240329E38F;
            p3.vy = (float)1.617541E38F;
            p3.yaw = (float)2.4462278E38F;
            p3.type_mask = (ushort)(ushort)21401;
            p3.vz = (float) -9.043769E37F;
            p3.afx = (float) -1.0565379E38F;
            p3.x = (float)2.124356E38F;
            p3.coordinate_frame = MAV_FRAME.MAV_FRAME_MISSION;
            p3.z = (float)1.9801264E38F;
            p3.afy = (float)1.6243734E38F;
            p3.afz = (float)7.2538846E37F;
            CommunicationChannel.instance.send(p3);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)4014024200361927528L);
                Debug.Assert(pack.seq == (uint)2799955124U);
                Debug.Assert(pack.target_component == (byte)(byte)120);
                Debug.Assert(pack.target_system == (byte)(byte)216);
            };
            PING p4 = new PING();
            PH.setPack(p4);
            p4.target_component = (byte)(byte)120;
            p4.time_usec = (ulong)4014024200361927528L;
            p4.target_system = (byte)(byte)216;
            p4.seq = (uint)2799955124U;
            SMP_TEST_CH.send(p4);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.version == (byte)(byte)147);
                Debug.Assert(pack.passkey_LEN(ph) == 23);
                Debug.Assert(pack.passkey_TRY(ph).Equals("vujryyyakdupwgpldefHNvD"));
                Debug.Assert(pack.control_request == (byte)(byte)71);
                Debug.Assert(pack.target_system == (byte)(byte)229);
            };
            CHANGE_OPERATOR_CONTROL p5 = new CHANGE_OPERATOR_CONTROL();
            PH.setPack(p5);
            p5.control_request = (byte)(byte)71;
            p5.passkey_SET("vujryyyakdupwgpldefHNvD", PH) ;
            p5.version = (byte)(byte)147;
            p5.target_system = (byte)(byte)229;
            SMP_TEST_CH.send(p5);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCHANGE_OPERATOR_CONTROL_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gcs_system_id == (byte)(byte)10);
                Debug.Assert(pack.control_request == (byte)(byte)67);
                Debug.Assert(pack.ack == (byte)(byte)125);
            };
            CHANGE_OPERATOR_CONTROL_ACK p6 = new CHANGE_OPERATOR_CONTROL_ACK();
            PH.setPack(p6);
            p6.control_request = (byte)(byte)67;
            p6.ack = (byte)(byte)125;
            p6.gcs_system_id = (byte)(byte)10;
            SMP_TEST_CH.send(p6);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnAUTH_KEYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.key_LEN(ph) == 12);
                Debug.Assert(pack.key_TRY(ph).Equals("ZrmeiUgsxkMa"));
            };
            AUTH_KEY p7 = new AUTH_KEY();
            PH.setPack(p7);
            p7.key_SET("ZrmeiUgsxkMa", PH) ;
            SMP_TEST_CH.send(p7);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_MODEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.custom_mode == (uint)3152001851U);
                Debug.Assert(pack.target_system == (byte)(byte)247);
                Debug.Assert(pack.base_mode == MAV_MODE.MAV_MODE_TEST_DISARMED);
            };
            SET_MODE p11 = new SET_MODE();
            PH.setPack(p11);
            p11.custom_mode = (uint)3152001851U;
            p11.base_mode = MAV_MODE.MAV_MODE_TEST_DISARMED;
            p11.target_system = (byte)(byte)247;
            SMP_TEST_CH.send(p11);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (short)(short) -6503);
                Debug.Assert(pack.param_id_LEN(ph) == 15);
                Debug.Assert(pack.param_id_TRY(ph).Equals("slcaRsxkuojuxrp"));
                Debug.Assert(pack.target_system == (byte)(byte)162);
                Debug.Assert(pack.target_component == (byte)(byte)185);
            };
            PARAM_REQUEST_READ p20 = new PARAM_REQUEST_READ();
            PH.setPack(p20);
            p20.target_system = (byte)(byte)162;
            p20.param_index = (short)(short) -6503;
            p20.param_id_SET("slcaRsxkuojuxrp", PH) ;
            p20.target_component = (byte)(byte)185;
            SMP_TEST_CH.send(p20);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)164);
                Debug.Assert(pack.target_system == (byte)(byte)82);
            };
            PARAM_REQUEST_LIST p21 = new PARAM_REQUEST_LIST();
            PH.setPack(p21);
            p21.target_system = (byte)(byte)82;
            p21.target_component = (byte)(byte)164;
            SMP_TEST_CH.send(p21);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value == (float)6.525779E37F);
                Debug.Assert(pack.param_index == (ushort)(ushort)63203);
                Debug.Assert(pack.param_count == (ushort)(ushort)6373);
                Debug.Assert(pack.param_id_LEN(ph) == 4);
                Debug.Assert(pack.param_id_TRY(ph).Equals("axer"));
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64);
            };
            PARAM_VALUE p22 = new PARAM_VALUE();
            PH.setPack(p22);
            p22.param_count = (ushort)(ushort)6373;
            p22.param_id_SET("axer", PH) ;
            p22.param_value = (float)6.525779E37F;
            p22.param_index = (ushort)(ushort)63203;
            p22.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL64;
            SMP_TEST_CH.send(p22);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)93);
                Debug.Assert(pack.target_component == (byte)(byte)165);
                Debug.Assert(pack.param_value == (float) -2.2329199E38F);
                Debug.Assert(pack.param_type == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
                Debug.Assert(pack.param_id_LEN(ph) == 3);
                Debug.Assert(pack.param_id_TRY(ph).Equals("cdY"));
            };
            PARAM_SET p23 = new PARAM_SET();
            PH.setPack(p23);
            p23.target_component = (byte)(byte)165;
            p23.target_system = (byte)(byte)93;
            p23.param_value = (float) -2.2329199E38F;
            p23.param_id_SET("cdY", PH) ;
            p23.param_type = MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8;
            SMP_TEST_CH.send(p23);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RAW_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)909496763);
                Debug.Assert(pack.hdg_acc_TRY(ph) == (uint)399249498U);
                Debug.Assert(pack.alt_ellipsoid_TRY(ph) == (int) -1715019254);
                Debug.Assert(pack.vel_acc_TRY(ph) == (uint)2571194942U);
                Debug.Assert(pack.vel == (ushort)(ushort)26364);
                Debug.Assert(pack.epv == (ushort)(ushort)64489);
                Debug.Assert(pack.alt == (int) -1944379596);
                Debug.Assert(pack.lat == (int) -1331812816);
                Debug.Assert(pack.satellites_visible == (byte)(byte)49);
                Debug.Assert(pack.time_usec == (ulong)3289476438735088175L);
                Debug.Assert(pack.cog == (ushort)(ushort)15810);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
                Debug.Assert(pack.eph == (ushort)(ushort)59729);
                Debug.Assert(pack.v_acc_TRY(ph) == (uint)611382240U);
                Debug.Assert(pack.h_acc_TRY(ph) == (uint)4041390400U);
            };
            GPS_RAW_INT p24 = new GPS_RAW_INT();
            PH.setPack(p24);
            p24.epv = (ushort)(ushort)64489;
            p24.hdg_acc_SET((uint)399249498U, PH) ;
            p24.time_usec = (ulong)3289476438735088175L;
            p24.h_acc_SET((uint)4041390400U, PH) ;
            p24.alt = (int) -1944379596;
            p24.vel_acc_SET((uint)2571194942U, PH) ;
            p24.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX;
            p24.v_acc_SET((uint)611382240U, PH) ;
            p24.lat = (int) -1331812816;
            p24.satellites_visible = (byte)(byte)49;
            p24.alt_ellipsoid_SET((int) -1715019254, PH) ;
            p24.eph = (ushort)(ushort)59729;
            p24.vel = (ushort)(ushort)26364;
            p24.lon = (int)909496763;
            p24.cog = (ushort)(ushort)15810;
            SMP_TEST_CH.send(p24);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.satellite_elevation.SequenceEqual(new byte[] {(byte)7, (byte)112, (byte)92, (byte)25, (byte)115, (byte)128, (byte)193, (byte)68, (byte)13, (byte)232, (byte)50, (byte)190, (byte)40, (byte)254, (byte)103, (byte)56, (byte)142, (byte)89, (byte)24, (byte)229}));
                Debug.Assert(pack.satellite_azimuth.SequenceEqual(new byte[] {(byte)39, (byte)27, (byte)110, (byte)246, (byte)16, (byte)209, (byte)232, (byte)153, (byte)4, (byte)117, (byte)251, (byte)118, (byte)181, (byte)77, (byte)243, (byte)121, (byte)169, (byte)202, (byte)48, (byte)239}));
                Debug.Assert(pack.satellites_visible == (byte)(byte)155);
                Debug.Assert(pack.satellite_snr.SequenceEqual(new byte[] {(byte)42, (byte)218, (byte)198, (byte)226, (byte)151, (byte)224, (byte)222, (byte)202, (byte)241, (byte)233, (byte)143, (byte)203, (byte)70, (byte)228, (byte)247, (byte)190, (byte)137, (byte)99, (byte)97, (byte)1}));
                Debug.Assert(pack.satellite_used.SequenceEqual(new byte[] {(byte)103, (byte)188, (byte)102, (byte)207, (byte)136, (byte)62, (byte)221, (byte)180, (byte)53, (byte)118, (byte)13, (byte)88, (byte)65, (byte)175, (byte)214, (byte)7, (byte)226, (byte)22, (byte)52, (byte)157}));
                Debug.Assert(pack.satellite_prn.SequenceEqual(new byte[] {(byte)112, (byte)2, (byte)47, (byte)212, (byte)40, (byte)160, (byte)1, (byte)207, (byte)101, (byte)146, (byte)113, (byte)69, (byte)43, (byte)82, (byte)28, (byte)80, (byte)174, (byte)55, (byte)236, (byte)31}));
            };
            GPS_STATUS p25 = new GPS_STATUS();
            PH.setPack(p25);
            p25.satellite_used_SET(new byte[] {(byte)103, (byte)188, (byte)102, (byte)207, (byte)136, (byte)62, (byte)221, (byte)180, (byte)53, (byte)118, (byte)13, (byte)88, (byte)65, (byte)175, (byte)214, (byte)7, (byte)226, (byte)22, (byte)52, (byte)157}, 0) ;
            p25.satellite_snr_SET(new byte[] {(byte)42, (byte)218, (byte)198, (byte)226, (byte)151, (byte)224, (byte)222, (byte)202, (byte)241, (byte)233, (byte)143, (byte)203, (byte)70, (byte)228, (byte)247, (byte)190, (byte)137, (byte)99, (byte)97, (byte)1}, 0) ;
            p25.satellite_prn_SET(new byte[] {(byte)112, (byte)2, (byte)47, (byte)212, (byte)40, (byte)160, (byte)1, (byte)207, (byte)101, (byte)146, (byte)113, (byte)69, (byte)43, (byte)82, (byte)28, (byte)80, (byte)174, (byte)55, (byte)236, (byte)31}, 0) ;
            p25.satellite_azimuth_SET(new byte[] {(byte)39, (byte)27, (byte)110, (byte)246, (byte)16, (byte)209, (byte)232, (byte)153, (byte)4, (byte)117, (byte)251, (byte)118, (byte)181, (byte)77, (byte)243, (byte)121, (byte)169, (byte)202, (byte)48, (byte)239}, 0) ;
            p25.satellite_elevation_SET(new byte[] {(byte)7, (byte)112, (byte)92, (byte)25, (byte)115, (byte)128, (byte)193, (byte)68, (byte)13, (byte)232, (byte)50, (byte)190, (byte)40, (byte)254, (byte)103, (byte)56, (byte)142, (byte)89, (byte)24, (byte)229}, 0) ;
            p25.satellites_visible = (byte)(byte)155;
            SMP_TEST_CH.send(p25);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yacc == (short)(short)21456);
                Debug.Assert(pack.zacc == (short)(short) -19237);
                Debug.Assert(pack.ymag == (short)(short) -14613);
                Debug.Assert(pack.xacc == (short)(short) -8180);
                Debug.Assert(pack.zgyro == (short)(short) -29150);
                Debug.Assert(pack.ygyro == (short)(short)9849);
                Debug.Assert(pack.xmag == (short)(short) -419);
                Debug.Assert(pack.time_boot_ms == (uint)2091453887U);
                Debug.Assert(pack.zmag == (short)(short)20659);
                Debug.Assert(pack.xgyro == (short)(short) -8706);
            };
            SCALED_IMU p26 = new SCALED_IMU();
            PH.setPack(p26);
            p26.time_boot_ms = (uint)2091453887U;
            p26.xmag = (short)(short) -419;
            p26.zgyro = (short)(short) -29150;
            p26.zacc = (short)(short) -19237;
            p26.yacc = (short)(short)21456;
            p26.xacc = (short)(short) -8180;
            p26.ymag = (short)(short) -14613;
            p26.ygyro = (short)(short)9849;
            p26.zmag = (short)(short)20659;
            p26.xgyro = (short)(short) -8706;
            SMP_TEST_CH.send(p26);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (short)(short) -3559);
                Debug.Assert(pack.xacc == (short)(short)12478);
                Debug.Assert(pack.time_usec == (ulong)8006161312137697527L);
                Debug.Assert(pack.zgyro == (short)(short)22795);
                Debug.Assert(pack.ymag == (short)(short) -22588);
                Debug.Assert(pack.yacc == (short)(short)29726);
                Debug.Assert(pack.xgyro == (short)(short)17383);
                Debug.Assert(pack.zacc == (short)(short)24269);
                Debug.Assert(pack.zmag == (short)(short) -13630);
                Debug.Assert(pack.xmag == (short)(short) -8035);
            };
            RAW_IMU p27 = new RAW_IMU();
            PH.setPack(p27);
            p27.xmag = (short)(short) -8035;
            p27.xacc = (short)(short)12478;
            p27.ygyro = (short)(short) -3559;
            p27.zgyro = (short)(short)22795;
            p27.xgyro = (short)(short)17383;
            p27.time_usec = (ulong)8006161312137697527L;
            p27.zacc = (short)(short)24269;
            p27.ymag = (short)(short) -22588;
            p27.zmag = (short)(short) -13630;
            p27.yacc = (short)(short)29726;
            SMP_TEST_CH.send(p27);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRAW_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff1 == (short)(short) -31810);
                Debug.Assert(pack.time_usec == (ulong)2410433953071778442L);
                Debug.Assert(pack.press_abs == (short)(short) -3175);
                Debug.Assert(pack.temperature == (short)(short) -30008);
                Debug.Assert(pack.press_diff2 == (short)(short)17321);
            };
            RAW_PRESSURE p28 = new RAW_PRESSURE();
            PH.setPack(p28);
            p28.temperature = (short)(short) -30008;
            p28.press_diff1 = (short)(short) -31810;
            p28.press_abs = (short)(short) -3175;
            p28.time_usec = (ulong)2410433953071778442L;
            p28.press_diff2 = (short)(short)17321;
            SMP_TEST_CH.send(p28);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSUREReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_abs == (float)9.443881E37F);
                Debug.Assert(pack.temperature == (short)(short) -4890);
                Debug.Assert(pack.time_boot_ms == (uint)2298250313U);
                Debug.Assert(pack.press_diff == (float)2.9003648E37F);
            };
            SCALED_PRESSURE p29 = new SCALED_PRESSURE();
            PH.setPack(p29);
            p29.time_boot_ms = (uint)2298250313U;
            p29.press_abs = (float)9.443881E37F;
            p29.temperature = (short)(short) -4890;
            p29.press_diff = (float)2.9003648E37F;
            SMP_TEST_CH.send(p29);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3034386407U);
                Debug.Assert(pack.roll == (float)2.5836843E38F);
                Debug.Assert(pack.yawspeed == (float)9.684464E37F);
                Debug.Assert(pack.rollspeed == (float) -3.3418616E38F);
                Debug.Assert(pack.pitchspeed == (float)6.1149447E37F);
                Debug.Assert(pack.yaw == (float)3.075636E37F);
                Debug.Assert(pack.pitch == (float)8.626706E37F);
            };
            ATTITUDE p30 = new ATTITUDE();
            PH.setPack(p30);
            p30.rollspeed = (float) -3.3418616E38F;
            p30.time_boot_ms = (uint)3034386407U;
            p30.roll = (float)2.5836843E38F;
            p30.yawspeed = (float)9.684464E37F;
            p30.pitchspeed = (float)6.1149447E37F;
            p30.pitch = (float)8.626706E37F;
            p30.yaw = (float)3.075636E37F;
            SMP_TEST_CH.send(p30);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitchspeed == (float)1.1191417E38F);
                Debug.Assert(pack.time_boot_ms == (uint)2016560384U);
                Debug.Assert(pack.rollspeed == (float) -2.0059127E38F);
                Debug.Assert(pack.yawspeed == (float)9.177353E37F);
                Debug.Assert(pack.q3 == (float)2.8981886E38F);
                Debug.Assert(pack.q1 == (float) -2.8176964E38F);
                Debug.Assert(pack.q4 == (float) -2.7130848E38F);
                Debug.Assert(pack.q2 == (float)2.2512759E38F);
            };
            ATTITUDE_QUATERNION p31 = new ATTITUDE_QUATERNION();
            PH.setPack(p31);
            p31.yawspeed = (float)9.177353E37F;
            p31.q2 = (float)2.2512759E38F;
            p31.q3 = (float)2.8981886E38F;
            p31.rollspeed = (float) -2.0059127E38F;
            p31.q1 = (float) -2.8176964E38F;
            p31.time_boot_ms = (uint)2016560384U;
            p31.q4 = (float) -2.7130848E38F;
            p31.pitchspeed = (float)1.1191417E38F;
            SMP_TEST_CH.send(p31);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float)4.674585E37F);
                Debug.Assert(pack.vx == (float) -7.165298E37F);
                Debug.Assert(pack.vy == (float)4.3074628E36F);
                Debug.Assert(pack.x == (float) -2.753817E38F);
                Debug.Assert(pack.vz == (float)7.202317E37F);
                Debug.Assert(pack.z == (float)1.2400957E38F);
                Debug.Assert(pack.time_boot_ms == (uint)779142377U);
            };
            LOCAL_POSITION_NED p32 = new LOCAL_POSITION_NED();
            PH.setPack(p32);
            p32.vz = (float)7.202317E37F;
            p32.x = (float) -2.753817E38F;
            p32.z = (float)1.2400957E38F;
            p32.vy = (float)4.3074628E36F;
            p32.time_boot_ms = (uint)779142377U;
            p32.y = (float)4.674585E37F;
            p32.vx = (float) -7.165298E37F;
            SMP_TEST_CH.send(p32);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -616681241);
                Debug.Assert(pack.relative_alt == (int)1446684193);
                Debug.Assert(pack.vz == (short)(short) -6495);
                Debug.Assert(pack.lon == (int) -1388465667);
                Debug.Assert(pack.alt == (int) -103209076);
                Debug.Assert(pack.vy == (short)(short) -31592);
                Debug.Assert(pack.vx == (short)(short) -9899);
                Debug.Assert(pack.hdg == (ushort)(ushort)12067);
                Debug.Assert(pack.time_boot_ms == (uint)1259863967U);
            };
            GLOBAL_POSITION_INT p33 = new GLOBAL_POSITION_INT();
            PH.setPack(p33);
            p33.relative_alt = (int)1446684193;
            p33.alt = (int) -103209076;
            p33.lat = (int) -616681241;
            p33.vy = (short)(short) -31592;
            p33.vz = (short)(short) -6495;
            p33.hdg = (ushort)(ushort)12067;
            p33.vx = (short)(short) -9899;
            p33.time_boot_ms = (uint)1259863967U;
            p33.lon = (int) -1388465667;
            SMP_TEST_CH.send(p33);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_SCALEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan4_scaled == (short)(short)16576);
                Debug.Assert(pack.chan1_scaled == (short)(short) -30808);
                Debug.Assert(pack.chan3_scaled == (short)(short)17973);
                Debug.Assert(pack.chan5_scaled == (short)(short)32152);
                Debug.Assert(pack.time_boot_ms == (uint)3371975394U);
                Debug.Assert(pack.chan2_scaled == (short)(short)23155);
                Debug.Assert(pack.chan8_scaled == (short)(short) -2154);
                Debug.Assert(pack.rssi == (byte)(byte)38);
                Debug.Assert(pack.port == (byte)(byte)58);
                Debug.Assert(pack.chan7_scaled == (short)(short)6860);
                Debug.Assert(pack.chan6_scaled == (short)(short) -30863);
            };
            RC_CHANNELS_SCALED p34 = new RC_CHANNELS_SCALED();
            PH.setPack(p34);
            p34.time_boot_ms = (uint)3371975394U;
            p34.port = (byte)(byte)58;
            p34.chan6_scaled = (short)(short) -30863;
            p34.chan2_scaled = (short)(short)23155;
            p34.chan8_scaled = (short)(short) -2154;
            p34.chan7_scaled = (short)(short)6860;
            p34.chan3_scaled = (short)(short)17973;
            p34.chan4_scaled = (short)(short)16576;
            p34.rssi = (byte)(byte)38;
            p34.chan5_scaled = (short)(short)32152;
            p34.chan1_scaled = (short)(short) -30808;
            SMP_TEST_CH.send(p34);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)19167);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)57280);
                Debug.Assert(pack.rssi == (byte)(byte)15);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)2463);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)23558);
                Debug.Assert(pack.port == (byte)(byte)76);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)52470);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)33375);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)55914);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)19735);
                Debug.Assert(pack.time_boot_ms == (uint)1969543552U);
            };
            RC_CHANNELS_RAW p35 = new RC_CHANNELS_RAW();
            PH.setPack(p35);
            p35.chan8_raw = (ushort)(ushort)33375;
            p35.chan2_raw = (ushort)(ushort)23558;
            p35.chan4_raw = (ushort)(ushort)19167;
            p35.chan5_raw = (ushort)(ushort)19735;
            p35.chan6_raw = (ushort)(ushort)52470;
            p35.chan1_raw = (ushort)(ushort)2463;
            p35.chan7_raw = (ushort)(ushort)57280;
            p35.chan3_raw = (ushort)(ushort)55914;
            p35.port = (byte)(byte)76;
            p35.time_boot_ms = (uint)1969543552U;
            p35.rssi = (byte)(byte)15;
            SMP_TEST_CH.send(p35);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERVO_OUTPUT_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.servo12_raw_TRY(ph) == (ushort)(ushort)60626);
                Debug.Assert(pack.servo6_raw == (ushort)(ushort)50015);
                Debug.Assert(pack.servo14_raw_TRY(ph) == (ushort)(ushort)63106);
                Debug.Assert(pack.port == (byte)(byte)245);
                Debug.Assert(pack.servo3_raw == (ushort)(ushort)21386);
                Debug.Assert(pack.servo15_raw_TRY(ph) == (ushort)(ushort)15901);
                Debug.Assert(pack.servo1_raw == (ushort)(ushort)54077);
                Debug.Assert(pack.servo2_raw == (ushort)(ushort)13009);
                Debug.Assert(pack.servo11_raw_TRY(ph) == (ushort)(ushort)27452);
                Debug.Assert(pack.servo13_raw_TRY(ph) == (ushort)(ushort)23167);
                Debug.Assert(pack.servo8_raw == (ushort)(ushort)57603);
                Debug.Assert(pack.time_usec == (uint)1290842769U);
                Debug.Assert(pack.servo5_raw == (ushort)(ushort)47000);
                Debug.Assert(pack.servo10_raw_TRY(ph) == (ushort)(ushort)18938);
                Debug.Assert(pack.servo4_raw == (ushort)(ushort)65410);
                Debug.Assert(pack.servo9_raw_TRY(ph) == (ushort)(ushort)16845);
                Debug.Assert(pack.servo16_raw_TRY(ph) == (ushort)(ushort)29676);
                Debug.Assert(pack.servo7_raw == (ushort)(ushort)43911);
            };
            SERVO_OUTPUT_RAW p36 = new SERVO_OUTPUT_RAW();
            PH.setPack(p36);
            p36.servo16_raw_SET((ushort)(ushort)29676, PH) ;
            p36.servo10_raw_SET((ushort)(ushort)18938, PH) ;
            p36.servo3_raw = (ushort)(ushort)21386;
            p36.servo12_raw_SET((ushort)(ushort)60626, PH) ;
            p36.servo2_raw = (ushort)(ushort)13009;
            p36.port = (byte)(byte)245;
            p36.servo9_raw_SET((ushort)(ushort)16845, PH) ;
            p36.time_usec = (uint)1290842769U;
            p36.servo11_raw_SET((ushort)(ushort)27452, PH) ;
            p36.servo14_raw_SET((ushort)(ushort)63106, PH) ;
            p36.servo6_raw = (ushort)(ushort)50015;
            p36.servo7_raw = (ushort)(ushort)43911;
            p36.servo13_raw_SET((ushort)(ushort)23167, PH) ;
            p36.servo1_raw = (ushort)(ushort)54077;
            p36.servo15_raw_SET((ushort)(ushort)15901, PH) ;
            p36.servo5_raw = (ushort)(ushort)47000;
            p36.servo8_raw = (ushort)(ushort)57603;
            p36.servo4_raw = (ushort)(ushort)65410;
            SMP_TEST_CH.send(p36);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)14);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.end_index == (short)(short) -21654);
                Debug.Assert(pack.target_system == (byte)(byte)135);
                Debug.Assert(pack.start_index == (short)(short)9578);
            };
            MISSION_REQUEST_PARTIAL_LIST p37 = new MISSION_REQUEST_PARTIAL_LIST();
            PH.setPack(p37);
            p37.target_component = (byte)(byte)14;
            p37.target_system = (byte)(byte)135;
            p37.start_index = (short)(short)9578;
            p37.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p37.end_index = (short)(short) -21654;
            SMP_TEST_CH.send(p37);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_WRITE_PARTIAL_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
                Debug.Assert(pack.end_index == (short)(short)11216);
                Debug.Assert(pack.start_index == (short)(short)32682);
                Debug.Assert(pack.target_system == (byte)(byte)6);
                Debug.Assert(pack.target_component == (byte)(byte)57);
            };
            MISSION_WRITE_PARTIAL_LIST p38 = new MISSION_WRITE_PARTIAL_LIST();
            PH.setPack(p38);
            p38.target_component = (byte)(byte)57;
            p38.start_index = (short)(short)32682;
            p38.end_index = (short)(short)11216;
            p38.target_system = (byte)(byte)6;
            p38.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY;
            SMP_TEST_CH.send(p38);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)109);
                Debug.Assert(pack.param4 == (float) -1.5468676E38F);
                Debug.Assert(pack.y == (float)1.7409808E38F);
                Debug.Assert(pack.x == (float) -2.8489642E38F);
                Debug.Assert(pack.param3 == (float)2.3245637E37F);
                Debug.Assert(pack.param1 == (float)1.922984E38F);
                Debug.Assert(pack.z == (float) -5.285831E37F);
                Debug.Assert(pack.seq == (ushort)(ushort)23556);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_DO_FOLLOW);
                Debug.Assert(pack.autocontinue == (byte)(byte)181);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
                Debug.Assert(pack.target_component == (byte)(byte)243);
                Debug.Assert(pack.current == (byte)(byte)243);
                Debug.Assert(pack.param2 == (float) -2.4080873E38F);
            };
            MISSION_ITEM p39 = new MISSION_ITEM();
            PH.setPack(p39);
            p39.z = (float) -5.285831E37F;
            p39.param3 = (float)2.3245637E37F;
            p39.y = (float)1.7409808E38F;
            p39.param1 = (float)1.922984E38F;
            p39.param4 = (float) -1.5468676E38F;
            p39.target_system = (byte)(byte)109;
            p39.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
            p39.param2 = (float) -2.4080873E38F;
            p39.seq = (ushort)(ushort)23556;
            p39.target_component = (byte)(byte)243;
            p39.autocontinue = (byte)(byte)181;
            p39.current = (byte)(byte)243;
            p39.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p39.command = MAV_CMD.MAV_CMD_DO_FOLLOW;
            p39.x = (float) -2.8489642E38F;
            SMP_TEST_CH.send(p39);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)6);
                Debug.Assert(pack.seq == (ushort)(ushort)63519);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.target_system == (byte)(byte)62);
            };
            MISSION_REQUEST p40 = new MISSION_REQUEST();
            PH.setPack(p40);
            p40.seq = (ushort)(ushort)63519;
            p40.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            p40.target_component = (byte)(byte)6;
            p40.target_system = (byte)(byte)62;
            SMP_TEST_CH.send(p40);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_SET_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)45332);
                Debug.Assert(pack.target_component == (byte)(byte)22);
                Debug.Assert(pack.target_system == (byte)(byte)236);
            };
            MISSION_SET_CURRENT p41 = new MISSION_SET_CURRENT();
            PH.setPack(p41);
            p41.target_component = (byte)(byte)22;
            p41.target_system = (byte)(byte)236;
            p41.seq = (ushort)(ushort)45332;
            SMP_TEST_CH.send(p41);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CURRENTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)44826);
            };
            MISSION_CURRENT p42 = new MISSION_CURRENT();
            PH.setPack(p42);
            p42.seq = (ushort)(ushort)44826;
            SMP_TEST_CH.send(p42);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)17);
                Debug.Assert(pack.target_system == (byte)(byte)183);
            };
            MISSION_REQUEST_LIST p43 = new MISSION_REQUEST_LIST();
            PH.setPack(p43);
            p43.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p43.target_component = (byte)(byte)17;
            p43.target_system = (byte)(byte)183;
            SMP_TEST_CH.send(p43);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_COUNTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)147);
                Debug.Assert(pack.target_system == (byte)(byte)227);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.count == (ushort)(ushort)22270);
            };
            MISSION_COUNT p44 = new MISSION_COUNT();
            PH.setPack(p44);
            p44.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p44.target_component = (byte)(byte)147;
            p44.target_system = (byte)(byte)227;
            p44.count = (ushort)(ushort)22270;
            SMP_TEST_CH.send(p44);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_CLEAR_ALLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)161);
                Debug.Assert(pack.target_system == (byte)(byte)219);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            };
            MISSION_CLEAR_ALL p45 = new MISSION_CLEAR_ALL();
            PH.setPack(p45);
            p45.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p45.target_component = (byte)(byte)161;
            p45.target_system = (byte)(byte)219;
            SMP_TEST_CH.send(p45);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_REACHEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.seq == (ushort)(ushort)22437);
            };
            MISSION_ITEM_REACHED p46 = new MISSION_ITEM_REACHED();
            PH.setPack(p46);
            p46.seq = (ushort)(ushort)22437;
            SMP_TEST_CH.send(p46);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)166);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
                Debug.Assert(pack.target_component == (byte)(byte)150);
                Debug.Assert(pack.type == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM6_Y);
            };
            MISSION_ACK p47 = new MISSION_ACK();
            PH.setPack(p47);
            p47.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE;
            p47.type = MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM6_Y;
            p47.target_component = (byte)(byte)150;
            p47.target_system = (byte)(byte)166;
            SMP_TEST_CH.send(p47);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_GPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude == (int) -639352870);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)829224650360095947L);
                Debug.Assert(pack.target_system == (byte)(byte)87);
                Debug.Assert(pack.latitude == (int) -1872160769);
                Debug.Assert(pack.longitude == (int)786969730);
            };
            SET_GPS_GLOBAL_ORIGIN p48 = new SET_GPS_GLOBAL_ORIGIN();
            PH.setPack(p48);
            p48.target_system = (byte)(byte)87;
            p48.longitude = (int)786969730;
            p48.altitude = (int) -639352870;
            p48.time_usec_SET((ulong)829224650360095947L, PH) ;
            p48.latitude = (int) -1872160769;
            SMP_TEST_CH.send(p48);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_GLOBAL_ORIGINReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)8596919517800403181L);
                Debug.Assert(pack.latitude == (int) -525580049);
                Debug.Assert(pack.altitude == (int)2125337876);
                Debug.Assert(pack.longitude == (int)473046315);
            };
            GPS_GLOBAL_ORIGIN p49 = new GPS_GLOBAL_ORIGIN();
            PH.setPack(p49);
            p49.altitude = (int)2125337876;
            p49.longitude = (int)473046315;
            p49.time_usec_SET((ulong)8596919517800403181L, PH) ;
            p49.latitude = (int) -525580049;
            SMP_TEST_CH.send(p49);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPARAM_MAP_RCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_value_max == (float) -2.4871781E38F);
                Debug.Assert(pack.param_value_min == (float) -2.0104953E38F);
                Debug.Assert(pack.scale == (float) -4.799449E36F);
                Debug.Assert(pack.param_index == (short)(short)32380);
                Debug.Assert(pack.parameter_rc_channel_index == (byte)(byte)233);
                Debug.Assert(pack.param_id_LEN(ph) == 1);
                Debug.Assert(pack.param_id_TRY(ph).Equals("y"));
                Debug.Assert(pack.target_component == (byte)(byte)83);
                Debug.Assert(pack.target_system == (byte)(byte)240);
                Debug.Assert(pack.param_value0 == (float) -3.5021845E37F);
            };
            PARAM_MAP_RC p50 = new PARAM_MAP_RC();
            PH.setPack(p50);
            p50.param_value_min = (float) -2.0104953E38F;
            p50.target_system = (byte)(byte)240;
            p50.parameter_rc_channel_index = (byte)(byte)233;
            p50.param_id_SET("y", PH) ;
            p50.scale = (float) -4.799449E36F;
            p50.param_value0 = (float) -3.5021845E37F;
            p50.param_value_max = (float) -2.4871781E38F;
            p50.param_index = (short)(short)32380;
            p50.target_component = (byte)(byte)83;
            SMP_TEST_CH.send(p50);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_REQUEST_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)144);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
                Debug.Assert(pack.seq == (ushort)(ushort)51379);
                Debug.Assert(pack.target_component == (byte)(byte)55);
            };
            MISSION_REQUEST_INT p51 = new MISSION_REQUEST_INT();
            PH.setPack(p51);
            p51.seq = (ushort)(ushort)51379;
            p51.target_component = (byte)(byte)55;
            p51.target_system = (byte)(byte)144;
            p51.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
            SMP_TEST_CH.send(p51);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_SET_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p2y == (float) -8.718917E37F);
                Debug.Assert(pack.p1z == (float)3.345059E38F);
                Debug.Assert(pack.p1y == (float) -3.950325E37F);
                Debug.Assert(pack.target_component == (byte)(byte)7);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.p2z == (float)7.726752E37F);
                Debug.Assert(pack.p1x == (float)2.3271613E38F);
                Debug.Assert(pack.target_system == (byte)(byte)104);
                Debug.Assert(pack.p2x == (float) -3.1080876E38F);
            };
            SAFETY_SET_ALLOWED_AREA p54 = new SAFETY_SET_ALLOWED_AREA();
            PH.setPack(p54);
            p54.p1y = (float) -3.950325E37F;
            p54.target_component = (byte)(byte)7;
            p54.p2z = (float)7.726752E37F;
            p54.frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p54.p2x = (float) -3.1080876E38F;
            p54.target_system = (byte)(byte)104;
            p54.p1z = (float)3.345059E38F;
            p54.p1x = (float)2.3271613E38F;
            p54.p2y = (float) -8.718917E37F;
            SMP_TEST_CH.send(p54);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSAFETY_ALLOWED_AREAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.p1x == (float)6.406356E37F);
                Debug.Assert(pack.p2y == (float)1.1342989E38F);
                Debug.Assert(pack.p1y == (float) -7.662078E37F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
                Debug.Assert(pack.p2x == (float)1.9199294E38F);
                Debug.Assert(pack.p1z == (float)2.478381E38F);
                Debug.Assert(pack.p2z == (float)1.0153276E36F);
            };
            SAFETY_ALLOWED_AREA p55 = new SAFETY_ALLOWED_AREA();
            PH.setPack(p55);
            p55.p2x = (float)1.9199294E38F;
            p55.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT;
            p55.p2y = (float)1.1342989E38F;
            p55.p2z = (float)1.0153276E36F;
            p55.p1z = (float)2.478381E38F;
            p55.p1y = (float) -7.662078E37F;
            p55.p1x = (float)6.406356E37F;
            SMP_TEST_CH.send(p55);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_QUATERNION_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitchspeed == (float)1.5323894E38F);
                Debug.Assert(pack.yawspeed == (float) -1.0357305E38F);
                Debug.Assert(pack.rollspeed == (float) -1.6290312E38F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.5109373E38F, -1.7756578E38F, 1.0617699E38F, 1.0080663E38F, 1.0813966E38F, 2.521286E38F, 1.9064832E38F, -1.2040088E38F, 1.0412722E38F}));
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.6377268E38F, 1.4463958E38F, 2.5277103E38F, 1.9214699E38F}));
                Debug.Assert(pack.time_usec == (ulong)2254166777728626185L);
            };
            ATTITUDE_QUATERNION_COV p61 = new ATTITUDE_QUATERNION_COV();
            PH.setPack(p61);
            p61.pitchspeed = (float)1.5323894E38F;
            p61.yawspeed = (float) -1.0357305E38F;
            p61.rollspeed = (float) -1.6290312E38F;
            p61.q_SET(new float[] {-2.6377268E38F, 1.4463958E38F, 2.5277103E38F, 1.9214699E38F}, 0) ;
            p61.time_usec = (ulong)2254166777728626185L;
            p61.covariance_SET(new float[] {1.5109373E38F, -1.7756578E38F, 1.0617699E38F, 1.0080663E38F, 1.0813966E38F, 2.521286E38F, 1.9064832E38F, -1.2040088E38F, 1.0412722E38F}, 0) ;
            SMP_TEST_CH.send(p61);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnNAV_CONTROLLER_OUTPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_bearing == (short)(short)28993);
                Debug.Assert(pack.aspd_error == (float)2.1799759E38F);
                Debug.Assert(pack.nav_roll == (float)3.2323764E37F);
                Debug.Assert(pack.nav_pitch == (float)1.8351265E38F);
                Debug.Assert(pack.alt_error == (float)1.3909223E38F);
                Debug.Assert(pack.nav_bearing == (short)(short)13927);
                Debug.Assert(pack.xtrack_error == (float) -1.2028132E38F);
                Debug.Assert(pack.wp_dist == (ushort)(ushort)40998);
            };
            NAV_CONTROLLER_OUTPUT p62 = new NAV_CONTROLLER_OUTPUT();
            PH.setPack(p62);
            p62.aspd_error = (float)2.1799759E38F;
            p62.xtrack_error = (float) -1.2028132E38F;
            p62.nav_roll = (float)3.2323764E37F;
            p62.target_bearing = (short)(short)28993;
            p62.nav_bearing = (short)(short)13927;
            p62.alt_error = (float)1.3909223E38F;
            p62.wp_dist = (ushort)(ushort)40998;
            p62.nav_pitch = (float)1.8351265E38F;
            SMP_TEST_CH.send(p62);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_POSITION_INT_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)6663101638260978204L);
                Debug.Assert(pack.relative_alt == (int)1688787540);
                Debug.Assert(pack.lon == (int)721315173);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO);
                Debug.Assert(pack.alt == (int)1824545029);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {3.3552442E38F, -1.1976919E38F, 2.6413466E38F, -8.323015E37F, -2.4127527E38F, -3.2819141E38F, -2.1385424E38F, 1.8995986E38F, 1.5371232E38F, -1.445188E36F, -2.2489884E38F, 9.704077E37F, 3.0541508E37F, -2.9888035E38F, 4.5396397E36F, -3.3611792E38F, -1.630515E38F, 2.666096E38F, -9.842076E37F, -2.124882E38F, -2.2122343E38F, -6.0083196E37F, 9.499609E37F, -2.5391885E38F, -2.8994599E38F, -2.183362E38F, -1.4378398E38F, -2.0890386E37F, -2.6133956E38F, -1.6295705E38F, -3.1404048E38F, 2.5813398E38F, -2.6980708E37F, -5.6681344E37F, 4.629896E37F, -3.1411518E38F}));
                Debug.Assert(pack.vx == (float) -2.2753036E38F);
                Debug.Assert(pack.lat == (int) -1483139600);
                Debug.Assert(pack.vy == (float)2.7905245E38F);
                Debug.Assert(pack.vz == (float) -2.4549038E38F);
            };
            GLOBAL_POSITION_INT_COV p63 = new GLOBAL_POSITION_INT_COV();
            PH.setPack(p63);
            p63.relative_alt = (int)1688787540;
            p63.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VIO;
            p63.covariance_SET(new float[] {3.3552442E38F, -1.1976919E38F, 2.6413466E38F, -8.323015E37F, -2.4127527E38F, -3.2819141E38F, -2.1385424E38F, 1.8995986E38F, 1.5371232E38F, -1.445188E36F, -2.2489884E38F, 9.704077E37F, 3.0541508E37F, -2.9888035E38F, 4.5396397E36F, -3.3611792E38F, -1.630515E38F, 2.666096E38F, -9.842076E37F, -2.124882E38F, -2.2122343E38F, -6.0083196E37F, 9.499609E37F, -2.5391885E38F, -2.8994599E38F, -2.183362E38F, -1.4378398E38F, -2.0890386E37F, -2.6133956E38F, -1.6295705E38F, -3.1404048E38F, 2.5813398E38F, -2.6980708E37F, -5.6681344E37F, 4.629896E37F, -3.1411518E38F}, 0) ;
            p63.lat = (int) -1483139600;
            p63.lon = (int)721315173;
            p63.time_usec = (ulong)6663101638260978204L;
            p63.vy = (float)2.7905245E38F;
            p63.alt = (int)1824545029;
            p63.vx = (float) -2.2753036E38F;
            p63.vz = (float) -2.4549038E38F;
            SMP_TEST_CH.send(p63);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vz == (float) -7.4152657E37F);
                Debug.Assert(pack.x == (float) -3.3968992E37F);
                Debug.Assert(pack.az == (float) -2.976502E38F);
                Debug.Assert(pack.estimator_type == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
                Debug.Assert(pack.vx == (float) -1.0284598E38F);
                Debug.Assert(pack.ay == (float)2.4172069E38F);
                Debug.Assert(pack.ax == (float)2.654767E38F);
                Debug.Assert(pack.vy == (float) -9.416707E36F);
                Debug.Assert(pack.covariance.SequenceEqual(new float[] {1.3646195E38F, -7.783739E37F, 8.71587E37F, -1.2293441E38F, -3.0660156E38F, -5.8806783E37F, -2.8454381E38F, -1.6172736E38F, 3.132146E38F, 1.0667808E38F, -3.3079785E38F, 1.1957616E38F, -2.1577894E38F, 3.026641E37F, 3.0030473E37F, 2.0220435E38F, -7.4384186E37F, -1.9125584E38F, -2.9279587E38F, -3.0918266E38F, -5.329444E37F, -3.1663322E38F, 1.4383218E38F, -2.1307866E38F, -3.2356386E38F, -5.259081E37F, -8.642295E37F, 4.712288E35F, 2.3145732E38F, -1.6959647E38F, 9.793464E37F, 2.6265004E38F, -1.5390597E38F, 2.4941727E38F, 2.2363314E38F, -2.3734415E38F, 3.2867815E37F, 4.244333E37F, -5.181058E37F, 2.2138368E38F, -1.219825E37F, -1.5953229E38F, 6.810735E37F, -3.3355585E38F, -1.1172445E38F}));
                Debug.Assert(pack.z == (float) -1.5451509E38F);
                Debug.Assert(pack.time_usec == (ulong)7421816337895729426L);
                Debug.Assert(pack.y == (float) -2.6271734E38F);
            };
            LOCAL_POSITION_NED_COV p64 = new LOCAL_POSITION_NED_COV();
            PH.setPack(p64);
            p64.z = (float) -1.5451509E38F;
            p64.vz = (float) -7.4152657E37F;
            p64.vx = (float) -1.0284598E38F;
            p64.ax = (float)2.654767E38F;
            p64.ay = (float)2.4172069E38F;
            p64.time_usec = (ulong)7421816337895729426L;
            p64.az = (float) -2.976502E38F;
            p64.x = (float) -3.3968992E37F;
            p64.covariance_SET(new float[] {1.3646195E38F, -7.783739E37F, 8.71587E37F, -1.2293441E38F, -3.0660156E38F, -5.8806783E37F, -2.8454381E38F, -1.6172736E38F, 3.132146E38F, 1.0667808E38F, -3.3079785E38F, 1.1957616E38F, -2.1577894E38F, 3.026641E37F, 3.0030473E37F, 2.0220435E38F, -7.4384186E37F, -1.9125584E38F, -2.9279587E38F, -3.0918266E38F, -5.329444E37F, -3.1663322E38F, 1.4383218E38F, -2.1307866E38F, -3.2356386E38F, -5.259081E37F, -8.642295E37F, 4.712288E35F, 2.3145732E38F, -1.6959647E38F, 9.793464E37F, 2.6265004E38F, -1.5390597E38F, 2.4941727E38F, 2.2363314E38F, -2.3734415E38F, 3.2867815E37F, 4.244333E37F, -5.181058E37F, 2.2138368E38F, -1.219825E37F, -1.5953229E38F, 6.810735E37F, -3.3355585E38F, -1.1172445E38F}, 0) ;
            p64.estimator_type = MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION;
            p64.y = (float) -2.6271734E38F;
            p64.vy = (float) -9.416707E36F;
            SMP_TEST_CH.send(p64);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan14_raw == (ushort)(ushort)16903);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)60287);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)55017);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)34400);
                Debug.Assert(pack.chan13_raw == (ushort)(ushort)1965);
                Debug.Assert(pack.chan17_raw == (ushort)(ushort)35712);
                Debug.Assert(pack.chancount == (byte)(byte)50);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)39652);
                Debug.Assert(pack.chan18_raw == (ushort)(ushort)44471);
                Debug.Assert(pack.time_boot_ms == (uint)690520541U);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)32784);
                Debug.Assert(pack.rssi == (byte)(byte)36);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)44005);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)45471);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)47364);
                Debug.Assert(pack.chan15_raw == (ushort)(ushort)681);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)61573);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)9100);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)44391);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)10571);
                Debug.Assert(pack.chan16_raw == (ushort)(ushort)24190);
            };
            RC_CHANNELS p65 = new RC_CHANNELS();
            PH.setPack(p65);
            p65.chan5_raw = (ushort)(ushort)10571;
            p65.chan11_raw = (ushort)(ushort)60287;
            p65.chan13_raw = (ushort)(ushort)1965;
            p65.chan2_raw = (ushort)(ushort)44391;
            p65.chan12_raw = (ushort)(ushort)47364;
            p65.chan1_raw = (ushort)(ushort)61573;
            p65.chan10_raw = (ushort)(ushort)39652;
            p65.chan18_raw = (ushort)(ushort)44471;
            p65.chan8_raw = (ushort)(ushort)55017;
            p65.time_boot_ms = (uint)690520541U;
            p65.chan16_raw = (ushort)(ushort)24190;
            p65.chancount = (byte)(byte)50;
            p65.chan4_raw = (ushort)(ushort)34400;
            p65.chan3_raw = (ushort)(ushort)44005;
            p65.chan9_raw = (ushort)(ushort)32784;
            p65.chan15_raw = (ushort)(ushort)681;
            p65.rssi = (byte)(byte)36;
            p65.chan6_raw = (ushort)(ushort)9100;
            p65.chan17_raw = (ushort)(ushort)35712;
            p65.chan7_raw = (ushort)(ushort)45471;
            p65.chan14_raw = (ushort)(ushort)16903;
            SMP_TEST_CH.send(p65);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnREQUEST_DATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.start_stop == (byte)(byte)187);
                Debug.Assert(pack.target_system == (byte)(byte)10);
                Debug.Assert(pack.target_component == (byte)(byte)31);
                Debug.Assert(pack.req_stream_id == (byte)(byte)49);
                Debug.Assert(pack.req_message_rate == (ushort)(ushort)2519);
            };
            REQUEST_DATA_STREAM p66 = new REQUEST_DATA_STREAM();
            PH.setPack(p66);
            p66.target_system = (byte)(byte)10;
            p66.start_stop = (byte)(byte)187;
            p66.req_stream_id = (byte)(byte)49;
            p66.target_component = (byte)(byte)31;
            p66.req_message_rate = (ushort)(ushort)2519;
            SMP_TEST_CH.send(p66);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_STREAMReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.on_off == (byte)(byte)48);
                Debug.Assert(pack.message_rate == (ushort)(ushort)59301);
                Debug.Assert(pack.stream_id == (byte)(byte)169);
            };
            DATA_STREAM p67 = new DATA_STREAM();
            PH.setPack(p67);
            p67.stream_id = (byte)(byte)169;
            p67.on_off = (byte)(byte)48;
            p67.message_rate = (ushort)(ushort)59301;
            SMP_TEST_CH.send(p67);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.r == (short)(short)12789);
                Debug.Assert(pack.x == (short)(short) -25089);
                Debug.Assert(pack.y == (short)(short) -10646);
                Debug.Assert(pack.buttons == (ushort)(ushort)35502);
                Debug.Assert(pack.z == (short)(short) -32248);
                Debug.Assert(pack.target == (byte)(byte)163);
            };
            MANUAL_CONTROL p69 = new MANUAL_CONTROL();
            PH.setPack(p69);
            p69.x = (short)(short) -25089;
            p69.target = (byte)(byte)163;
            p69.y = (short)(short) -10646;
            p69.r = (short)(short)12789;
            p69.buttons = (ushort)(ushort)35502;
            p69.z = (short)(short) -32248;
            SMP_TEST_CH.send(p69);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRC_CHANNELS_OVERRIDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)46737);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)13360);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)32987);
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)10950);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)53941);
                Debug.Assert(pack.target_component == (byte)(byte)100);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)43810);
                Debug.Assert(pack.target_system == (byte)(byte)237);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)4190);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)4141);
            };
            RC_CHANNELS_OVERRIDE p70 = new RC_CHANNELS_OVERRIDE();
            PH.setPack(p70);
            p70.target_component = (byte)(byte)100;
            p70.target_system = (byte)(byte)237;
            p70.chan1_raw = (ushort)(ushort)32987;
            p70.chan7_raw = (ushort)(ushort)10950;
            p70.chan6_raw = (ushort)(ushort)53941;
            p70.chan5_raw = (ushort)(ushort)43810;
            p70.chan8_raw = (ushort)(ushort)13360;
            p70.chan4_raw = (ushort)(ushort)46737;
            p70.chan3_raw = (ushort)(ushort)4190;
            p70.chan2_raw = (ushort)(ushort)4141;
            SMP_TEST_CH.send(p70);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMISSION_ITEM_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param3 == (float)1.5133026E38F);
                Debug.Assert(pack.z == (float) -2.6426183E38F);
                Debug.Assert(pack.seq == (ushort)(ushort)61766);
                Debug.Assert(pack.target_component == (byte)(byte)22);
                Debug.Assert(pack.autocontinue == (byte)(byte)32);
                Debug.Assert(pack.param1 == (float) -8.639676E37F);
                Debug.Assert(pack.param4 == (float)2.451137E37F);
                Debug.Assert(pack.x == (int) -160523614);
                Debug.Assert(pack.target_system == (byte)(byte)61);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE);
                Debug.Assert(pack.param2 == (float) -6.3593816E35F);
                Debug.Assert(pack.mission_type == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
                Debug.Assert(pack.current == (byte)(byte)158);
                Debug.Assert(pack.y == (int)972430093);
            };
            MISSION_ITEM_INT p73 = new MISSION_ITEM_INT();
            PH.setPack(p73);
            p73.x = (int) -160523614;
            p73.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL;
            p73.z = (float) -2.6426183E38F;
            p73.param3 = (float)1.5133026E38F;
            p73.current = (byte)(byte)158;
            p73.param2 = (float) -6.3593816E35F;
            p73.param1 = (float) -8.639676E37F;
            p73.y = (int)972430093;
            p73.target_component = (byte)(byte)22;
            p73.target_system = (byte)(byte)61;
            p73.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT;
            p73.seq = (ushort)(ushort)61766;
            p73.autocontinue = (byte)(byte)32;
            p73.command = MAV_CMD.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE;
            p73.param4 = (float)2.451137E37F;
            SMP_TEST_CH.send(p73);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVFR_HUDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.groundspeed == (float)1.9101815E38F);
                Debug.Assert(pack.airspeed == (float)1.9558474E38F);
                Debug.Assert(pack.climb == (float)3.2693148E37F);
                Debug.Assert(pack.throttle == (ushort)(ushort)4072);
                Debug.Assert(pack.heading == (short)(short)16893);
                Debug.Assert(pack.alt == (float) -2.272364E38F);
            };
            VFR_HUD p74 = new VFR_HUD();
            PH.setPack(p74);
            p74.alt = (float) -2.272364E38F;
            p74.airspeed = (float)1.9558474E38F;
            p74.throttle = (ushort)(ushort)4072;
            p74.climb = (float)3.2693148E37F;
            p74.groundspeed = (float)1.9101815E38F;
            p74.heading = (short)(short)16893;
            SMP_TEST_CH.send(p74);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param3 == (float) -1.6429459E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
                Debug.Assert(pack.target_component == (byte)(byte)111);
                Debug.Assert(pack.autocontinue == (byte)(byte)186);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_WAYPOINT_USER_4);
                Debug.Assert(pack.x == (int)2034635852);
                Debug.Assert(pack.target_system == (byte)(byte)50);
                Debug.Assert(pack.y == (int) -962299313);
                Debug.Assert(pack.param4 == (float) -1.9879084E38F);
                Debug.Assert(pack.current == (byte)(byte)51);
                Debug.Assert(pack.z == (float) -6.529178E37F);
                Debug.Assert(pack.param1 == (float)2.1067343E38F);
                Debug.Assert(pack.param2 == (float) -1.1133242E38F);
            };
            COMMAND_INT p75 = new COMMAND_INT();
            PH.setPack(p75);
            p75.target_component = (byte)(byte)111;
            p75.autocontinue = (byte)(byte)186;
            p75.target_system = (byte)(byte)50;
            p75.z = (float) -6.529178E37F;
            p75.param2 = (float) -1.1133242E38F;
            p75.command = MAV_CMD.MAV_CMD_WAYPOINT_USER_4;
            p75.current = (byte)(byte)51;
            p75.frame = MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            p75.param3 = (float) -1.6429459E38F;
            p75.param4 = (float) -1.9879084E38F;
            p75.param1 = (float)2.1067343E38F;
            p75.x = (int)2034635852;
            p75.y = (int) -962299313;
            SMP_TEST_CH.send(p75);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(SMP_TEST_CH, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_LONGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param7 == (float) -1.1194003E38F);
                Debug.Assert(pack.param3 == (float)8.721439E37F);
                Debug.Assert(pack.target_system == (byte)(byte)234);
                Debug.Assert(pack.param4 == (float)1.8367456E38F);
                Debug.Assert(pack.param1 == (float)3.521226E37F);
                Debug.Assert(pack.param5 == (float)1.2324223E38F);
                Debug.Assert(pack.param2 == (float) -2.117158E38F);
                Debug.Assert(pack.target_component == (byte)(byte)194);
                Debug.Assert(pack.param6 == (float)2.9308696E38F);
                Debug.Assert(pack.confirmation == (byte)(byte)82);
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM);
            };
            GroundControl.COMMAND_LONG p76 = CommunicationChannel.new_COMMAND_LONG();
            PH.setPack(p76);
            p76.target_system = (byte)(byte)234;
            p76.command = MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM;
            p76.param3 = (float)8.721439E37F;
            p76.confirmation = (byte)(byte)82;
            p76.param5 = (float)1.2324223E38F;
            p76.param7 = (float) -1.1194003E38F;
            p76.param4 = (float)1.8367456E38F;
            p76.param2 = (float) -2.117158E38F;
            p76.param6 = (float)2.9308696E38F;
            p76.target_component = (byte)(byte)194;
            p76.param1 = (float)3.521226E37F;
            CommunicationChannel.instance.send(p76);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCOMMAND_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.command == MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE);
                Debug.Assert(pack.result_param2_TRY(ph) == (int) -1209504604);
                Debug.Assert(pack.target_component_TRY(ph) == (byte)(byte)197);
                Debug.Assert(pack.result == MAV_RESULT.MAV_RESULT_IN_PROGRESS);
                Debug.Assert(pack.target_system_TRY(ph) == (byte)(byte)203);
                Debug.Assert(pack.progress_TRY(ph) == (byte)(byte)204);
            };
            GroundControl.COMMAND_ACK p77 = CommunicationChannel.new_COMMAND_ACK();
            PH.setPack(p77);
            p77.target_component_SET((byte)(byte)197, PH) ;
            p77.target_system_SET((byte)(byte)203, PH) ;
            p77.progress_SET((byte)(byte)204, PH) ;
            p77.result_param2_SET((int) -1209504604, PH) ;
            p77.result = MAV_RESULT.MAV_RESULT_IN_PROGRESS;
            p77.command = MAV_CMD.MAV_CMD_NAV_GUIDED_ENABLE;
            CommunicationChannel.instance.send(p77);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnMANUAL_SETPOINTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pitch == (float)3.1177262E38F);
                Debug.Assert(pack.thrust == (float)7.2794004E37F);
                Debug.Assert(pack.time_boot_ms == (uint)2047537210U);
                Debug.Assert(pack.mode_switch == (byte)(byte)122);
                Debug.Assert(pack.manual_override_switch == (byte)(byte)163);
                Debug.Assert(pack.yaw == (float)1.3892551E38F);
                Debug.Assert(pack.roll == (float)2.2695797E38F);
            };
            GroundControl.MANUAL_SETPOINT p81 = CommunicationChannel.new_MANUAL_SETPOINT();
            PH.setPack(p81);
            p81.time_boot_ms = (uint)2047537210U;
            p81.yaw = (float)1.3892551E38F;
            p81.pitch = (float)3.1177262E38F;
            p81.roll = (float)2.2695797E38F;
            p81.manual_override_switch = (byte)(byte)163;
            p81.mode_switch = (byte)(byte)122;
            p81.thrust = (float)7.2794004E37F;
            CommunicationChannel.instance.send(p81);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)91);
                Debug.Assert(pack.time_boot_ms == (uint)199790770U);
                Debug.Assert(pack.body_roll_rate == (float) -1.7760716E38F);
                Debug.Assert(pack.body_pitch_rate == (float) -8.640124E37F);
                Debug.Assert(pack.type_mask == (byte)(byte)210);
                Debug.Assert(pack.body_yaw_rate == (float)9.806734E37F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.0004304E38F, 2.2326183E38F, 1.4753882E38F, -3.1290665E38F}));
                Debug.Assert(pack.thrust == (float)6.272067E37F);
                Debug.Assert(pack.target_component == (byte)(byte)205);
            };
            GroundControl.SET_ATTITUDE_TARGET p82 = CommunicationChannel.new_SET_ATTITUDE_TARGET();
            PH.setPack(p82);
            p82.body_roll_rate = (float) -1.7760716E38F;
            p82.time_boot_ms = (uint)199790770U;
            p82.type_mask = (byte)(byte)210;
            p82.target_component = (byte)(byte)205;
            p82.body_yaw_rate = (float)9.806734E37F;
            p82.q_SET(new float[] {3.0004304E38F, 2.2326183E38F, 1.4753882E38F, -3.1290665E38F}, 0) ;
            p82.body_pitch_rate = (float) -8.640124E37F;
            p82.thrust = (float)6.272067E37F;
            p82.target_system = (byte)(byte)91;
            CommunicationChannel.instance.send(p82);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATTITUDE_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.thrust == (float)2.7138998E38F);
                Debug.Assert(pack.body_roll_rate == (float)2.4410959E38F);
                Debug.Assert(pack.body_pitch_rate == (float) -3.0857455E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-9.995347E37F, 1.3700394E38F, 1.2184247E37F, 1.1479848E38F}));
                Debug.Assert(pack.time_boot_ms == (uint)3486399664U);
                Debug.Assert(pack.body_yaw_rate == (float) -1.2948641E38F);
                Debug.Assert(pack.type_mask == (byte)(byte)207);
            };
            GroundControl.ATTITUDE_TARGET p83 = CommunicationChannel.new_ATTITUDE_TARGET();
            PH.setPack(p83);
            p83.body_roll_rate = (float)2.4410959E38F;
            p83.time_boot_ms = (uint)3486399664U;
            p83.body_yaw_rate = (float) -1.2948641E38F;
            p83.body_pitch_rate = (float) -3.0857455E38F;
            p83.type_mask = (byte)(byte)207;
            p83.thrust = (float)2.7138998E38F;
            p83.q_SET(new float[] {-9.995347E37F, 1.3700394E38F, 1.2184247E37F, 1.1479848E38F}, 0) ;
            CommunicationChannel.instance.send(p83);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_LOCAL_NEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float) -2.5662406E38F);
                Debug.Assert(pack.vz == (float)1.509916E38F);
                Debug.Assert(pack.z == (float)1.7599373E38F);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_MISSION);
                Debug.Assert(pack.afx == (float) -3.1137878E38F);
                Debug.Assert(pack.afy == (float) -7.0959754E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1743301091U);
                Debug.Assert(pack.afz == (float)1.6921871E38F);
                Debug.Assert(pack.y == (float) -1.2691522E38F);
                Debug.Assert(pack.target_system == (byte)(byte)196);
                Debug.Assert(pack.vx == (float) -2.9585626E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)53083);
                Debug.Assert(pack.target_component == (byte)(byte)138);
                Debug.Assert(pack.yaw_rate == (float)2.8153356E38F);
                Debug.Assert(pack.vy == (float)3.3475805E38F);
                Debug.Assert(pack.yaw == (float) -1.177406E38F);
            };
            GroundControl.SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.new_SET_POSITION_TARGET_LOCAL_NED();
            PH.setPack(p84);
            p84.yaw = (float) -1.177406E38F;
            p84.vx = (float) -2.9585626E38F;
            p84.vy = (float)3.3475805E38F;
            p84.z = (float)1.7599373E38F;
            p84.type_mask = (ushort)(ushort)53083;
            p84.vz = (float)1.509916E38F;
            p84.yaw_rate = (float)2.8153356E38F;
            p84.x = (float) -2.5662406E38F;
            p84.target_system = (byte)(byte)196;
            p84.time_boot_ms = (uint)1743301091U;
            p84.afy = (float) -7.0959754E37F;
            p84.afz = (float)1.6921871E38F;
            p84.y = (float) -1.2691522E38F;
            p84.afx = (float) -3.1137878E38F;
            p84.coordinate_frame = MAV_FRAME.MAV_FRAME_MISSION;
            p84.target_component = (byte)(byte)138;
            CommunicationChannel.instance.send(p84);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_POSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vx == (float)3.2868363E38F);
                Debug.Assert(pack.yaw == (float)2.3063049E38F);
                Debug.Assert(pack.afy == (float)1.7239103E38F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)7330);
                Debug.Assert(pack.time_boot_ms == (uint)2296418552U);
                Debug.Assert(pack.vz == (float) -1.459675E38F);
                Debug.Assert(pack.alt == (float)2.3306892E38F);
                Debug.Assert(pack.lon_int == (int) -1588660657);
                Debug.Assert(pack.lat_int == (int) -1725513068);
                Debug.Assert(pack.afx == (float) -2.6765343E38F);
                Debug.Assert(pack.target_system == (byte)(byte)226);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
                Debug.Assert(pack.yaw_rate == (float) -2.4240041E38F);
                Debug.Assert(pack.vy == (float) -2.8132976E38F);
                Debug.Assert(pack.afz == (float) -1.0909789E37F);
                Debug.Assert(pack.target_component == (byte)(byte)162);
            };
            GroundControl.SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p86);
            p86.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED;
            p86.vx = (float)3.2868363E38F;
            p86.lon_int = (int) -1588660657;
            p86.target_component = (byte)(byte)162;
            p86.vz = (float) -1.459675E38F;
            p86.afy = (float)1.7239103E38F;
            p86.lat_int = (int) -1725513068;
            p86.vy = (float) -2.8132976E38F;
            p86.time_boot_ms = (uint)2296418552U;
            p86.yaw_rate = (float) -2.4240041E38F;
            p86.alt = (float)2.3306892E38F;
            p86.type_mask = (ushort)(ushort)7330;
            p86.yaw = (float)2.3063049E38F;
            p86.afx = (float) -2.6765343E38F;
            p86.afz = (float) -1.0909789E37F;
            p86.target_system = (byte)(byte)226;
            CommunicationChannel.instance.send(p86);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOSITION_TARGET_GLOBAL_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon_int == (int)1096331328);
                Debug.Assert(pack.afy == (float)1.3208829E38F);
                Debug.Assert(pack.yaw == (float)3.3468544E38F);
                Debug.Assert(pack.yaw_rate == (float) -6.5326326E36F);
                Debug.Assert(pack.type_mask == (ushort)(ushort)13681);
                Debug.Assert(pack.vy == (float)1.1961197E38F);
                Debug.Assert(pack.time_boot_ms == (uint)4088662279U);
                Debug.Assert(pack.lat_int == (int) -470106259);
                Debug.Assert(pack.coordinate_frame == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
                Debug.Assert(pack.vx == (float) -2.031851E38F);
                Debug.Assert(pack.alt == (float)1.7901735E38F);
                Debug.Assert(pack.afx == (float) -2.0875735E38F);
                Debug.Assert(pack.vz == (float) -1.676974E38F);
                Debug.Assert(pack.afz == (float) -5.840866E36F);
            };
            GroundControl.POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.new_POSITION_TARGET_GLOBAL_INT();
            PH.setPack(p87);
            p87.vz = (float) -1.676974E38F;
            p87.vx = (float) -2.031851E38F;
            p87.lon_int = (int)1096331328;
            p87.alt = (float)1.7901735E38F;
            p87.afy = (float)1.3208829E38F;
            p87.yaw = (float)3.3468544E38F;
            p87.lat_int = (int) -470106259;
            p87.type_mask = (ushort)(ushort)13681;
            p87.afx = (float) -2.0875735E38F;
            p87.coordinate_frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU;
            p87.vy = (float)1.1961197E38F;
            p87.time_boot_ms = (uint)4088662279U;
            p87.afz = (float) -5.840866E36F;
            p87.yaw_rate = (float) -6.5326326E36F;
            CommunicationChannel.instance.send(p87);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -3.2681475E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1696020268U);
                Debug.Assert(pack.yaw == (float) -3.1588354E38F);
                Debug.Assert(pack.x == (float)5.9688576E37F);
                Debug.Assert(pack.pitch == (float) -3.1251838E38F);
                Debug.Assert(pack.roll == (float)3.2205315E38F);
                Debug.Assert(pack.z == (float) -2.6136972E38F);
            };
            GroundControl.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
            PH.setPack(p89);
            p89.pitch = (float) -3.1251838E38F;
            p89.y = (float) -3.2681475E38F;
            p89.x = (float)5.9688576E37F;
            p89.z = (float) -2.6136972E38F;
            p89.roll = (float)3.2205315E38F;
            p89.yaw = (float) -3.1588354E38F;
            p89.time_boot_ms = (uint)1696020268U;
            CommunicationChannel.instance.send(p89);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int) -1270523662);
                Debug.Assert(pack.vy == (short)(short) -32105);
                Debug.Assert(pack.pitch == (float) -1.7760765E38F);
                Debug.Assert(pack.zacc == (short)(short) -4115);
                Debug.Assert(pack.pitchspeed == (float)1.5894458E38F);
                Debug.Assert(pack.yacc == (short)(short) -25541);
                Debug.Assert(pack.vx == (short)(short)635);
                Debug.Assert(pack.alt == (int) -1216481948);
                Debug.Assert(pack.time_usec == (ulong)5896468618451445740L);
                Debug.Assert(pack.rollspeed == (float) -1.8995377E38F);
                Debug.Assert(pack.lat == (int) -275282936);
                Debug.Assert(pack.roll == (float) -3.1523957E38F);
                Debug.Assert(pack.vz == (short)(short) -19061);
                Debug.Assert(pack.xacc == (short)(short)13553);
                Debug.Assert(pack.yawspeed == (float)2.7649369E37F);
                Debug.Assert(pack.yaw == (float)1.2298362E36F);
            };
            GroundControl.HIL_STATE p90 = CommunicationChannel.new_HIL_STATE();
            PH.setPack(p90);
            p90.vx = (short)(short)635;
            p90.zacc = (short)(short) -4115;
            p90.rollspeed = (float) -1.8995377E38F;
            p90.yacc = (short)(short) -25541;
            p90.lon = (int) -1270523662;
            p90.vz = (short)(short) -19061;
            p90.roll = (float) -3.1523957E38F;
            p90.pitch = (float) -1.7760765E38F;
            p90.yawspeed = (float)2.7649369E37F;
            p90.yaw = (float)1.2298362E36F;
            p90.alt = (int) -1216481948;
            p90.xacc = (short)(short)13553;
            p90.lat = (int) -275282936;
            p90.pitchspeed = (float)1.5894458E38F;
            p90.time_usec = (ulong)5896468618451445740L;
            p90.vy = (short)(short) -32105;
            CommunicationChannel.instance.send(p90);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.aux4 == (float)2.2224213E38F);
                Debug.Assert(pack.pitch_elevator == (float)2.8932425E38F);
                Debug.Assert(pack.yaw_rudder == (float) -2.0151053E38F);
                Debug.Assert(pack.time_usec == (ulong)5045964966568933726L);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_AUTO_ARMED);
                Debug.Assert(pack.aux3 == (float)9.931043E36F);
                Debug.Assert(pack.aux1 == (float) -3.0761272E38F);
                Debug.Assert(pack.nav_mode == (byte)(byte)21);
                Debug.Assert(pack.roll_ailerons == (float) -1.9322674E38F);
                Debug.Assert(pack.aux2 == (float) -2.5431308E38F);
                Debug.Assert(pack.throttle == (float)2.688022E38F);
            };
            GroundControl.HIL_CONTROLS p91 = CommunicationChannel.new_HIL_CONTROLS();
            PH.setPack(p91);
            p91.time_usec = (ulong)5045964966568933726L;
            p91.aux1 = (float) -3.0761272E38F;
            p91.yaw_rudder = (float) -2.0151053E38F;
            p91.aux2 = (float) -2.5431308E38F;
            p91.mode = MAV_MODE.MAV_MODE_AUTO_ARMED;
            p91.nav_mode = (byte)(byte)21;
            p91.pitch_elevator = (float)2.8932425E38F;
            p91.aux4 = (float)2.2224213E38F;
            p91.throttle = (float)2.688022E38F;
            p91.roll_ailerons = (float) -1.9322674E38F;
            p91.aux3 = (float)9.931043E36F;
            CommunicationChannel.instance.send(p91);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_RC_INPUTS_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.chan7_raw == (ushort)(ushort)33820);
                Debug.Assert(pack.chan6_raw == (ushort)(ushort)38097);
                Debug.Assert(pack.chan10_raw == (ushort)(ushort)31962);
                Debug.Assert(pack.chan8_raw == (ushort)(ushort)60652);
                Debug.Assert(pack.chan1_raw == (ushort)(ushort)17755);
                Debug.Assert(pack.chan2_raw == (ushort)(ushort)3983);
                Debug.Assert(pack.chan9_raw == (ushort)(ushort)28697);
                Debug.Assert(pack.chan11_raw == (ushort)(ushort)12836);
                Debug.Assert(pack.chan4_raw == (ushort)(ushort)32527);
                Debug.Assert(pack.chan3_raw == (ushort)(ushort)57754);
                Debug.Assert(pack.time_usec == (ulong)7943762863492384815L);
                Debug.Assert(pack.chan12_raw == (ushort)(ushort)55303);
                Debug.Assert(pack.chan5_raw == (ushort)(ushort)11331);
                Debug.Assert(pack.rssi == (byte)(byte)144);
            };
            GroundControl.HIL_RC_INPUTS_RAW p92 = CommunicationChannel.new_HIL_RC_INPUTS_RAW();
            PH.setPack(p92);
            p92.chan3_raw = (ushort)(ushort)57754;
            p92.chan5_raw = (ushort)(ushort)11331;
            p92.chan11_raw = (ushort)(ushort)12836;
            p92.rssi = (byte)(byte)144;
            p92.chan7_raw = (ushort)(ushort)33820;
            p92.chan6_raw = (ushort)(ushort)38097;
            p92.chan12_raw = (ushort)(ushort)55303;
            p92.chan4_raw = (ushort)(ushort)32527;
            p92.chan1_raw = (ushort)(ushort)17755;
            p92.chan8_raw = (ushort)(ushort)60652;
            p92.chan9_raw = (ushort)(ushort)28697;
            p92.chan10_raw = (ushort)(ushort)31962;
            p92.chan2_raw = (ushort)(ushort)3983;
            p92.time_usec = (ulong)7943762863492384815L;
            CommunicationChannel.instance.send(p92);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_ACTUATOR_CONTROLSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3152679232664754177L);
                Debug.Assert(pack.mode == MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {2.4645274E38F, -1.195405E38F, 2.5586503E38F, 1.5261593E38F, 1.2054197E38F, 2.5923733E38F, 2.2930292E38F, -1.6654151E36F, -2.7816763E38F, 6.952691E37F, 1.3195055E36F, -2.6062024E38F, 1.862763E38F, -3.1528392E37F, -1.9534196E38F, 6.550784E37F}));
                Debug.Assert(pack.flags == (ulong)8227907583482630511L);
            };
            GroundControl.HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.new_HIL_ACTUATOR_CONTROLS();
            PH.setPack(p93);
            p93.controls_SET(new float[] {2.4645274E38F, -1.195405E38F, 2.5586503E38F, 1.5261593E38F, 1.2054197E38F, 2.5923733E38F, 2.2930292E38F, -1.6654151E36F, -2.7816763E38F, 6.952691E37F, 1.3195055E36F, -2.6062024E38F, 1.862763E38F, -3.1528392E37F, -1.9534196E38F, 6.550784E37F}, 0) ;
            p93.time_usec = (ulong)3152679232664754177L;
            p93.flags = (ulong)8227907583482630511L;
            p93.mode = MAV_MODE.MAV_MODE_STABILIZE_DISARMED;
            CommunicationChannel.instance.send(p93);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flow_rate_y_TRY(ph) == (float)2.5388036E38F);
                Debug.Assert(pack.ground_distance == (float)3.701295E37F);
                Debug.Assert(pack.time_usec == (ulong)3853921050261478315L);
                Debug.Assert(pack.flow_y == (short)(short)6419);
                Debug.Assert(pack.flow_comp_m_x == (float)2.9880782E38F);
                Debug.Assert(pack.quality == (byte)(byte)75);
                Debug.Assert(pack.flow_comp_m_y == (float) -1.8817352E38F);
                Debug.Assert(pack.flow_rate_x_TRY(ph) == (float)2.9292241E38F);
                Debug.Assert(pack.sensor_id == (byte)(byte)140);
                Debug.Assert(pack.flow_x == (short)(short) -14069);
            };
            GroundControl.OPTICAL_FLOW p100 = CommunicationChannel.new_OPTICAL_FLOW();
            PH.setPack(p100);
            p100.quality = (byte)(byte)75;
            p100.flow_y = (short)(short)6419;
            p100.ground_distance = (float)3.701295E37F;
            p100.flow_rate_x_SET((float)2.9292241E38F, PH) ;
            p100.flow_x = (short)(short) -14069;
            p100.flow_rate_y_SET((float)2.5388036E38F, PH) ;
            p100.flow_comp_m_y = (float) -1.8817352E38F;
            p100.time_usec = (ulong)3853921050261478315L;
            p100.sensor_id = (byte)(byte)140;
            p100.flow_comp_m_x = (float)2.9880782E38F;
            CommunicationChannel.instance.send(p100);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGLOBAL_VISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -3.117371E38F);
                Debug.Assert(pack.usec == (ulong)4156659302753854943L);
                Debug.Assert(pack.pitch == (float) -5.237249E37F);
                Debug.Assert(pack.z == (float)6.777853E37F);
                Debug.Assert(pack.yaw == (float)2.163942E38F);
                Debug.Assert(pack.roll == (float) -2.4885085E38F);
                Debug.Assert(pack.x == (float) -2.4341157E38F);
            };
            GroundControl.GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
            PH.setPack(p101);
            p101.y = (float) -3.117371E38F;
            p101.x = (float) -2.4341157E38F;
            p101.yaw = (float)2.163942E38F;
            p101.z = (float)6.777853E37F;
            p101.usec = (ulong)4156659302753854943L;
            p101.roll = (float) -2.4885085E38F;
            p101.pitch = (float) -5.237249E37F;
            CommunicationChannel.instance.send(p101);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.yaw == (float)2.1984657E38F);
                Debug.Assert(pack.z == (float) -2.5587813E38F);
                Debug.Assert(pack.usec == (ulong)4708302745887494419L);
                Debug.Assert(pack.y == (float)1.1610512E38F);
                Debug.Assert(pack.roll == (float)2.0605375E38F);
                Debug.Assert(pack.x == (float) -5.1001368E36F);
                Debug.Assert(pack.pitch == (float)2.407825E38F);
            };
            GroundControl.VISION_POSITION_ESTIMATE p102 = CommunicationChannel.new_VISION_POSITION_ESTIMATE();
            PH.setPack(p102);
            p102.usec = (ulong)4708302745887494419L;
            p102.pitch = (float)2.407825E38F;
            p102.x = (float) -5.1001368E36F;
            p102.roll = (float)2.0605375E38F;
            p102.yaw = (float)2.1984657E38F;
            p102.y = (float)1.1610512E38F;
            p102.z = (float) -2.5587813E38F;
            CommunicationChannel.instance.send(p102);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVISION_SPEED_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.z == (float)3.290622E38F);
                Debug.Assert(pack.x == (float) -5.4744145E37F);
                Debug.Assert(pack.usec == (ulong)1812283119816457687L);
                Debug.Assert(pack.y == (float)1.1932641E38F);
            };
            GroundControl.VISION_SPEED_ESTIMATE p103 = CommunicationChannel.new_VISION_SPEED_ESTIMATE();
            PH.setPack(p103);
            p103.z = (float)3.290622E38F;
            p103.x = (float) -5.4744145E37F;
            p103.y = (float)1.1932641E38F;
            p103.usec = (ulong)1812283119816457687L;
            CommunicationChannel.instance.send(p103);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnVICON_POSITION_ESTIMATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)2.9302393E38F);
                Debug.Assert(pack.usec == (ulong)2949094179149995511L);
                Debug.Assert(pack.roll == (float)3.3014278E38F);
                Debug.Assert(pack.pitch == (float)1.8951123E38F);
                Debug.Assert(pack.y == (float) -9.671959E37F);
                Debug.Assert(pack.yaw == (float) -9.683315E37F);
                Debug.Assert(pack.z == (float) -3.0672451E38F);
            };
            GroundControl.VICON_POSITION_ESTIMATE p104 = CommunicationChannel.new_VICON_POSITION_ESTIMATE();
            PH.setPack(p104);
            p104.usec = (ulong)2949094179149995511L;
            p104.x = (float)2.9302393E38F;
            p104.yaw = (float) -9.683315E37F;
            p104.roll = (float)3.3014278E38F;
            p104.pitch = (float)1.8951123E38F;
            p104.z = (float) -3.0672451E38F;
            p104.y = (float) -9.671959E37F;
            CommunicationChannel.instance.send(p104);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIGHRES_IMUReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ygyro == (float)1.476206E38F);
                Debug.Assert(pack.time_usec == (ulong)4654411358869607348L);
                Debug.Assert(pack.temperature == (float)1.4615575E38F);
                Debug.Assert(pack.zacc == (float) -1.0251155E38F);
                Debug.Assert(pack.zmag == (float)2.6271434E38F);
                Debug.Assert(pack.fields_updated == (ushort)(ushort)13654);
                Debug.Assert(pack.abs_pressure == (float) -2.6072648E38F);
                Debug.Assert(pack.diff_pressure == (float) -1.8738861E38F);
                Debug.Assert(pack.xacc == (float) -2.4448796E38F);
                Debug.Assert(pack.xgyro == (float)1.6159877E38F);
                Debug.Assert(pack.yacc == (float) -1.9189904E38F);
                Debug.Assert(pack.xmag == (float)2.1915855E38F);
                Debug.Assert(pack.zgyro == (float)2.0121083E38F);
                Debug.Assert(pack.pressure_alt == (float) -4.0085275E37F);
                Debug.Assert(pack.ymag == (float)1.5186401E38F);
            };
            GroundControl.HIGHRES_IMU p105 = CommunicationChannel.new_HIGHRES_IMU();
            PH.setPack(p105);
            p105.xmag = (float)2.1915855E38F;
            p105.ygyro = (float)1.476206E38F;
            p105.fields_updated = (ushort)(ushort)13654;
            p105.diff_pressure = (float) -1.8738861E38F;
            p105.xacc = (float) -2.4448796E38F;
            p105.zgyro = (float)2.0121083E38F;
            p105.time_usec = (ulong)4654411358869607348L;
            p105.temperature = (float)1.4615575E38F;
            p105.xgyro = (float)1.6159877E38F;
            p105.zacc = (float) -1.0251155E38F;
            p105.pressure_alt = (float) -4.0085275E37F;
            p105.ymag = (float)1.5186401E38F;
            p105.abs_pressure = (float) -2.6072648E38F;
            p105.zmag = (float)2.6271434E38F;
            p105.yacc = (float) -1.9189904E38F;
            CommunicationChannel.instance.send(p105);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnOPTICAL_FLOW_RADReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)2139956818275443659L);
                Debug.Assert(pack.integrated_xgyro == (float) -2.3305014E38F);
                Debug.Assert(pack.integrated_y == (float)1.5151211E38F);
                Debug.Assert(pack.quality == (byte)(byte)178);
                Debug.Assert(pack.temperature == (short)(short) -28698);
                Debug.Assert(pack.integration_time_us == (uint)2175756926U);
                Debug.Assert(pack.distance == (float)3.2696845E38F);
                Debug.Assert(pack.integrated_x == (float) -1.074959E38F);
                Debug.Assert(pack.time_delta_distance_us == (uint)961059883U);
                Debug.Assert(pack.integrated_ygyro == (float) -4.420889E37F);
                Debug.Assert(pack.sensor_id == (byte)(byte)116);
                Debug.Assert(pack.integrated_zgyro == (float) -9.172324E37F);
            };
            GroundControl.OPTICAL_FLOW_RAD p106 = CommunicationChannel.new_OPTICAL_FLOW_RAD();
            PH.setPack(p106);
            p106.temperature = (short)(short) -28698;
            p106.integrated_x = (float) -1.074959E38F;
            p106.distance = (float)3.2696845E38F;
            p106.integrated_zgyro = (float) -9.172324E37F;
            p106.integrated_xgyro = (float) -2.3305014E38F;
            p106.sensor_id = (byte)(byte)116;
            p106.quality = (byte)(byte)178;
            p106.time_delta_distance_us = (uint)961059883U;
            p106.time_usec = (ulong)2139956818275443659L;
            p106.integrated_y = (float)1.5151211E38F;
            p106.integration_time_us = (uint)2175756926U;
            p106.integrated_ygyro = (float) -4.420889E37F;
            CommunicationChannel.instance.send(p106);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (float)1.9543797E38F);
                Debug.Assert(pack.zgyro == (float)4.221546E37F);
                Debug.Assert(pack.ygyro == (float) -2.9292028E38F);
                Debug.Assert(pack.xacc == (float) -3.0350857E38F);
                Debug.Assert(pack.temperature == (float)5.769239E37F);
                Debug.Assert(pack.diff_pressure == (float) -3.6911574E37F);
                Debug.Assert(pack.zacc == (float) -7.117029E37F);
                Debug.Assert(pack.zmag == (float)2.2874719E38F);
                Debug.Assert(pack.fields_updated == (uint)1998061319U);
                Debug.Assert(pack.time_usec == (ulong)2108109417828610011L);
                Debug.Assert(pack.xmag == (float)9.287787E37F);
                Debug.Assert(pack.pressure_alt == (float) -2.2731693E38F);
                Debug.Assert(pack.yacc == (float)2.7219779E38F);
                Debug.Assert(pack.abs_pressure == (float) -1.9640214E38F);
                Debug.Assert(pack.ymag == (float)8.430639E37F);
            };
            GroundControl.HIL_SENSOR p107 = CommunicationChannel.new_HIL_SENSOR();
            PH.setPack(p107);
            p107.abs_pressure = (float) -1.9640214E38F;
            p107.xmag = (float)9.287787E37F;
            p107.zgyro = (float)4.221546E37F;
            p107.xgyro = (float)1.9543797E38F;
            p107.pressure_alt = (float) -2.2731693E38F;
            p107.yacc = (float)2.7219779E38F;
            p107.fields_updated = (uint)1998061319U;
            p107.zmag = (float)2.2874719E38F;
            p107.zacc = (float) -7.117029E37F;
            p107.time_usec = (ulong)2108109417828610011L;
            p107.xacc = (float) -3.0350857E38F;
            p107.ymag = (float)8.430639E37F;
            p107.ygyro = (float) -2.9292028E38F;
            p107.temperature = (float)5.769239E37F;
            p107.diff_pressure = (float) -3.6911574E37F;
            CommunicationChannel.instance.send(p107);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSIM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.q4 == (float)1.3772219E38F);
                Debug.Assert(pack.q2 == (float)2.356906E38F);
                Debug.Assert(pack.alt == (float) -1.7758174E38F);
                Debug.Assert(pack.xacc == (float) -2.2772353E38F);
                Debug.Assert(pack.lat == (float) -1.346605E38F);
                Debug.Assert(pack.q3 == (float) -3.1087575E38F);
                Debug.Assert(pack.xgyro == (float) -1.1491748E38F);
                Debug.Assert(pack.zacc == (float)1.2923056E38F);
                Debug.Assert(pack.lon == (float) -1.715964E38F);
                Debug.Assert(pack.ve == (float) -1.7847443E38F);
                Debug.Assert(pack.vn == (float)8.985612E37F);
                Debug.Assert(pack.roll == (float) -2.7056483E38F);
                Debug.Assert(pack.yaw == (float) -2.9220837E38F);
                Debug.Assert(pack.std_dev_vert == (float) -1.0714391E38F);
                Debug.Assert(pack.vd == (float)5.328031E37F);
                Debug.Assert(pack.zgyro == (float)1.7165245E38F);
                Debug.Assert(pack.ygyro == (float) -1.9078572E38F);
                Debug.Assert(pack.pitch == (float)2.6224202E38F);
                Debug.Assert(pack.yacc == (float)3.373441E38F);
                Debug.Assert(pack.q1 == (float)3.0999252E38F);
                Debug.Assert(pack.std_dev_horz == (float) -9.372738E37F);
            };
            GroundControl.SIM_STATE p108 = CommunicationChannel.new_SIM_STATE();
            PH.setPack(p108);
            p108.zgyro = (float)1.7165245E38F;
            p108.vn = (float)8.985612E37F;
            p108.q1 = (float)3.0999252E38F;
            p108.roll = (float) -2.7056483E38F;
            p108.q4 = (float)1.3772219E38F;
            p108.std_dev_vert = (float) -1.0714391E38F;
            p108.yacc = (float)3.373441E38F;
            p108.xgyro = (float) -1.1491748E38F;
            p108.lat = (float) -1.346605E38F;
            p108.std_dev_horz = (float) -9.372738E37F;
            p108.xacc = (float) -2.2772353E38F;
            p108.alt = (float) -1.7758174E38F;
            p108.q2 = (float)2.356906E38F;
            p108.lon = (float) -1.715964E38F;
            p108.ygyro = (float) -1.9078572E38F;
            p108.zacc = (float)1.2923056E38F;
            p108.yaw = (float) -2.9220837E38F;
            p108.q3 = (float) -3.1087575E38F;
            p108.ve = (float) -1.7847443E38F;
            p108.vd = (float)5.328031E37F;
            p108.pitch = (float)2.6224202E38F;
            CommunicationChannel.instance.send(p108);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnRADIO_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.remnoise == (byte)(byte)195);
                Debug.Assert(pack.rxerrors == (ushort)(ushort)4238);
                Debug.Assert(pack.fixed_ == (ushort)(ushort)21498);
                Debug.Assert(pack.remrssi == (byte)(byte)187);
                Debug.Assert(pack.rssi == (byte)(byte)251);
                Debug.Assert(pack.noise == (byte)(byte)222);
                Debug.Assert(pack.txbuf == (byte)(byte)157);
            };
            GroundControl.RADIO_STATUS p109 = CommunicationChannel.new_RADIO_STATUS();
            PH.setPack(p109);
            p109.remnoise = (byte)(byte)195;
            p109.rssi = (byte)(byte)251;
            p109.fixed_ = (ushort)(ushort)21498;
            p109.remrssi = (byte)(byte)187;
            p109.txbuf = (byte)(byte)157;
            p109.noise = (byte)(byte)222;
            p109.rxerrors = (ushort)(ushort)4238;
            CommunicationChannel.instance.send(p109);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnFILE_TRANSFER_PROTOCOLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)116, (byte)227, (byte)22, (byte)81, (byte)8, (byte)62, (byte)155, (byte)187, (byte)92, (byte)195, (byte)250, (byte)92, (byte)49, (byte)57, (byte)99, (byte)155, (byte)10, (byte)58, (byte)174, (byte)137, (byte)111, (byte)19, (byte)243, (byte)227, (byte)177, (byte)53, (byte)223, (byte)110, (byte)79, (byte)4, (byte)89, (byte)86, (byte)4, (byte)131, (byte)58, (byte)198, (byte)191, (byte)51, (byte)145, (byte)121, (byte)153, (byte)140, (byte)5, (byte)132, (byte)169, (byte)104, (byte)10, (byte)17, (byte)91, (byte)152, (byte)207, (byte)249, (byte)23, (byte)116, (byte)65, (byte)254, (byte)86, (byte)170, (byte)230, (byte)105, (byte)102, (byte)105, (byte)4, (byte)105, (byte)234, (byte)48, (byte)208, (byte)113, (byte)192, (byte)155, (byte)141, (byte)195, (byte)16, (byte)40, (byte)30, (byte)186, (byte)140, (byte)141, (byte)75, (byte)81, (byte)162, (byte)96, (byte)171, (byte)157, (byte)88, (byte)27, (byte)17, (byte)197, (byte)4, (byte)18, (byte)175, (byte)215, (byte)107, (byte)114, (byte)160, (byte)10, (byte)225, (byte)245, (byte)38, (byte)67, (byte)144, (byte)246, (byte)204, (byte)76, (byte)37, (byte)122, (byte)252, (byte)191, (byte)215, (byte)19, (byte)36, (byte)7, (byte)216, (byte)176, (byte)83, (byte)235, (byte)207, (byte)170, (byte)225, (byte)198, (byte)227, (byte)121, (byte)86, (byte)110, (byte)229, (byte)2, (byte)203, (byte)57, (byte)216, (byte)80, (byte)231, (byte)54, (byte)8, (byte)216, (byte)223, (byte)225, (byte)30, (byte)156, (byte)57, (byte)4, (byte)6, (byte)77, (byte)71, (byte)189, (byte)129, (byte)192, (byte)119, (byte)4, (byte)235, (byte)134, (byte)205, (byte)29, (byte)230, (byte)245, (byte)17, (byte)127, (byte)48, (byte)93, (byte)207, (byte)227, (byte)118, (byte)246, (byte)80, (byte)82, (byte)118, (byte)73, (byte)126, (byte)125, (byte)50, (byte)22, (byte)82, (byte)200, (byte)255, (byte)246, (byte)104, (byte)194, (byte)146, (byte)9, (byte)225, (byte)190, (byte)72, (byte)14, (byte)187, (byte)48, (byte)236, (byte)202, (byte)216, (byte)233, (byte)105, (byte)179, (byte)81, (byte)191, (byte)37, (byte)17, (byte)114, (byte)78, (byte)108, (byte)100, (byte)48, (byte)115, (byte)36, (byte)97, (byte)75, (byte)146, (byte)48, (byte)205, (byte)212, (byte)59, (byte)83, (byte)148, (byte)145, (byte)44, (byte)151, (byte)48, (byte)31, (byte)2, (byte)239, (byte)125, (byte)49, (byte)184, (byte)67, (byte)17, (byte)48, (byte)239, (byte)232, (byte)90, (byte)233, (byte)181, (byte)69, (byte)226, (byte)203, (byte)76, (byte)86, (byte)48, (byte)150, (byte)158, (byte)23, (byte)97, (byte)73, (byte)240, (byte)201, (byte)71, (byte)192, (byte)92, (byte)29, (byte)120, (byte)66, (byte)105, (byte)188, (byte)199, (byte)163}));
                Debug.Assert(pack.target_network == (byte)(byte)92);
                Debug.Assert(pack.target_component == (byte)(byte)142);
                Debug.Assert(pack.target_system == (byte)(byte)105);
            };
            GroundControl.FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.new_FILE_TRANSFER_PROTOCOL();
            PH.setPack(p110);
            p110.target_system = (byte)(byte)105;
            p110.target_network = (byte)(byte)92;
            p110.target_component = (byte)(byte)142;
            p110.payload_SET(new byte[] {(byte)116, (byte)227, (byte)22, (byte)81, (byte)8, (byte)62, (byte)155, (byte)187, (byte)92, (byte)195, (byte)250, (byte)92, (byte)49, (byte)57, (byte)99, (byte)155, (byte)10, (byte)58, (byte)174, (byte)137, (byte)111, (byte)19, (byte)243, (byte)227, (byte)177, (byte)53, (byte)223, (byte)110, (byte)79, (byte)4, (byte)89, (byte)86, (byte)4, (byte)131, (byte)58, (byte)198, (byte)191, (byte)51, (byte)145, (byte)121, (byte)153, (byte)140, (byte)5, (byte)132, (byte)169, (byte)104, (byte)10, (byte)17, (byte)91, (byte)152, (byte)207, (byte)249, (byte)23, (byte)116, (byte)65, (byte)254, (byte)86, (byte)170, (byte)230, (byte)105, (byte)102, (byte)105, (byte)4, (byte)105, (byte)234, (byte)48, (byte)208, (byte)113, (byte)192, (byte)155, (byte)141, (byte)195, (byte)16, (byte)40, (byte)30, (byte)186, (byte)140, (byte)141, (byte)75, (byte)81, (byte)162, (byte)96, (byte)171, (byte)157, (byte)88, (byte)27, (byte)17, (byte)197, (byte)4, (byte)18, (byte)175, (byte)215, (byte)107, (byte)114, (byte)160, (byte)10, (byte)225, (byte)245, (byte)38, (byte)67, (byte)144, (byte)246, (byte)204, (byte)76, (byte)37, (byte)122, (byte)252, (byte)191, (byte)215, (byte)19, (byte)36, (byte)7, (byte)216, (byte)176, (byte)83, (byte)235, (byte)207, (byte)170, (byte)225, (byte)198, (byte)227, (byte)121, (byte)86, (byte)110, (byte)229, (byte)2, (byte)203, (byte)57, (byte)216, (byte)80, (byte)231, (byte)54, (byte)8, (byte)216, (byte)223, (byte)225, (byte)30, (byte)156, (byte)57, (byte)4, (byte)6, (byte)77, (byte)71, (byte)189, (byte)129, (byte)192, (byte)119, (byte)4, (byte)235, (byte)134, (byte)205, (byte)29, (byte)230, (byte)245, (byte)17, (byte)127, (byte)48, (byte)93, (byte)207, (byte)227, (byte)118, (byte)246, (byte)80, (byte)82, (byte)118, (byte)73, (byte)126, (byte)125, (byte)50, (byte)22, (byte)82, (byte)200, (byte)255, (byte)246, (byte)104, (byte)194, (byte)146, (byte)9, (byte)225, (byte)190, (byte)72, (byte)14, (byte)187, (byte)48, (byte)236, (byte)202, (byte)216, (byte)233, (byte)105, (byte)179, (byte)81, (byte)191, (byte)37, (byte)17, (byte)114, (byte)78, (byte)108, (byte)100, (byte)48, (byte)115, (byte)36, (byte)97, (byte)75, (byte)146, (byte)48, (byte)205, (byte)212, (byte)59, (byte)83, (byte)148, (byte)145, (byte)44, (byte)151, (byte)48, (byte)31, (byte)2, (byte)239, (byte)125, (byte)49, (byte)184, (byte)67, (byte)17, (byte)48, (byte)239, (byte)232, (byte)90, (byte)233, (byte)181, (byte)69, (byte)226, (byte)203, (byte)76, (byte)86, (byte)48, (byte)150, (byte)158, (byte)23, (byte)97, (byte)73, (byte)240, (byte)201, (byte)71, (byte)192, (byte)92, (byte)29, (byte)120, (byte)66, (byte)105, (byte)188, (byte)199, (byte)163}, 0) ;
            CommunicationChannel.instance.send(p110);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTIMESYNCReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ts1 == (long) -8533057822013992175L);
                Debug.Assert(pack.tc1 == (long)889854997937689029L);
            };
            GroundControl.TIMESYNC p111 = CommunicationChannel.new_TIMESYNC();
            PH.setPack(p111);
            p111.tc1 = (long)889854997937689029L;
            p111.ts1 = (long) -8533057822013992175L;
            CommunicationChannel.instance.send(p111);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnCAMERA_TRIGGERReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)1624301276389849353L);
                Debug.Assert(pack.seq == (uint)3708994050U);
            };
            GroundControl.CAMERA_TRIGGER p112 = CommunicationChannel.new_CAMERA_TRIGGER();
            PH.setPack(p112);
            p112.seq = (uint)3708994050U;
            p112.time_usec = (ulong)1624301276389849353L;
            CommunicationChannel.instance.send(p112);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_GPSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.fix_type == (byte)(byte)89);
                Debug.Assert(pack.alt == (int)75206479);
                Debug.Assert(pack.ve == (short)(short) -13142);
                Debug.Assert(pack.cog == (ushort)(ushort)40476);
                Debug.Assert(pack.epv == (ushort)(ushort)63850);
                Debug.Assert(pack.lat == (int) -1249180924);
                Debug.Assert(pack.vel == (ushort)(ushort)42563);
                Debug.Assert(pack.lon == (int)1853499965);
                Debug.Assert(pack.eph == (ushort)(ushort)21208);
                Debug.Assert(pack.time_usec == (ulong)861553895972874250L);
                Debug.Assert(pack.satellites_visible == (byte)(byte)79);
                Debug.Assert(pack.vd == (short)(short)9896);
                Debug.Assert(pack.vn == (short)(short)37);
            };
            GroundControl.HIL_GPS p113 = CommunicationChannel.new_HIL_GPS();
            PH.setPack(p113);
            p113.cog = (ushort)(ushort)40476;
            p113.lon = (int)1853499965;
            p113.satellites_visible = (byte)(byte)79;
            p113.vn = (short)(short)37;
            p113.epv = (ushort)(ushort)63850;
            p113.time_usec = (ulong)861553895972874250L;
            p113.alt = (int)75206479;
            p113.fix_type = (byte)(byte)89;
            p113.eph = (ushort)(ushort)21208;
            p113.lat = (int) -1249180924;
            p113.ve = (short)(short) -13142;
            p113.vel = (ushort)(ushort)42563;
            p113.vd = (short)(short)9896;
            CommunicationChannel.instance.send(p113);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_OPTICAL_FLOWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.integrated_y == (float)1.796806E38F);
                Debug.Assert(pack.distance == (float)3.3035954E38F);
                Debug.Assert(pack.integrated_xgyro == (float)1.1813062E38F);
                Debug.Assert(pack.time_usec == (ulong)1273580385789348922L);
                Debug.Assert(pack.integrated_ygyro == (float)7.4779115E37F);
                Debug.Assert(pack.sensor_id == (byte)(byte)111);
                Debug.Assert(pack.temperature == (short)(short) -7123);
                Debug.Assert(pack.integrated_zgyro == (float) -1.9875273E38F);
                Debug.Assert(pack.integrated_x == (float) -4.698652E37F);
                Debug.Assert(pack.integration_time_us == (uint)3449520339U);
                Debug.Assert(pack.time_delta_distance_us == (uint)2060322130U);
                Debug.Assert(pack.quality == (byte)(byte)123);
            };
            GroundControl.HIL_OPTICAL_FLOW p114 = CommunicationChannel.new_HIL_OPTICAL_FLOW();
            PH.setPack(p114);
            p114.sensor_id = (byte)(byte)111;
            p114.integrated_ygyro = (float)7.4779115E37F;
            p114.time_usec = (ulong)1273580385789348922L;
            p114.distance = (float)3.3035954E38F;
            p114.time_delta_distance_us = (uint)2060322130U;
            p114.integrated_zgyro = (float) -1.9875273E38F;
            p114.integrated_x = (float) -4.698652E37F;
            p114.temperature = (short)(short) -7123;
            p114.quality = (byte)(byte)123;
            p114.integrated_xgyro = (float)1.1813062E38F;
            p114.integration_time_us = (uint)3449520339U;
            p114.integrated_y = (float)1.796806E38F;
            CommunicationChannel.instance.send(p114);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnHIL_STATE_QUATERNIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1129630918);
                Debug.Assert(pack.lon == (int)120404869);
                Debug.Assert(pack.vz == (short)(short) -26441);
                Debug.Assert(pack.ind_airspeed == (ushort)(ushort)40612);
                Debug.Assert(pack.xacc == (short)(short)6699);
                Debug.Assert(pack.vx == (short)(short)13412);
                Debug.Assert(pack.yacc == (short)(short) -9493);
                Debug.Assert(pack.vy == (short)(short)25589);
                Debug.Assert(pack.rollspeed == (float)6.350081E37F);
                Debug.Assert(pack.yawspeed == (float) -2.401846E38F);
                Debug.Assert(pack.zacc == (short)(short)21717);
                Debug.Assert(pack.time_usec == (ulong)1040147486476918679L);
                Debug.Assert(pack.pitchspeed == (float)8.722244E37F);
                Debug.Assert(pack.alt == (int)147901381);
                Debug.Assert(pack.true_airspeed == (ushort)(ushort)43082);
                Debug.Assert(pack.attitude_quaternion.SequenceEqual(new float[] {-3.0291801E38F, -1.779488E38F, -2.1074294E38F, 2.0824475E38F}));
            };
            GroundControl.HIL_STATE_QUATERNION p115 = CommunicationChannel.new_HIL_STATE_QUATERNION();
            PH.setPack(p115);
            p115.zacc = (short)(short)21717;
            p115.ind_airspeed = (ushort)(ushort)40612;
            p115.vy = (short)(short)25589;
            p115.alt = (int)147901381;
            p115.xacc = (short)(short)6699;
            p115.vz = (short)(short) -26441;
            p115.attitude_quaternion_SET(new float[] {-3.0291801E38F, -1.779488E38F, -2.1074294E38F, 2.0824475E38F}, 0) ;
            p115.true_airspeed = (ushort)(ushort)43082;
            p115.vx = (short)(short)13412;
            p115.pitchspeed = (float)8.722244E37F;
            p115.rollspeed = (float)6.350081E37F;
            p115.time_usec = (ulong)1040147486476918679L;
            p115.lat = (int)1129630918;
            p115.yacc = (short)(short) -9493;
            p115.yawspeed = (float) -2.401846E38F;
            p115.lon = (int)120404869;
            CommunicationChannel.instance.send(p115);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.zacc == (short)(short)11530);
                Debug.Assert(pack.xgyro == (short)(short)31611);
                Debug.Assert(pack.ygyro == (short)(short) -32058);
                Debug.Assert(pack.xmag == (short)(short) -1741);
                Debug.Assert(pack.time_boot_ms == (uint)3605346237U);
                Debug.Assert(pack.xacc == (short)(short) -5604);
                Debug.Assert(pack.zgyro == (short)(short) -1805);
                Debug.Assert(pack.ymag == (short)(short)21628);
                Debug.Assert(pack.zmag == (short)(short)16866);
                Debug.Assert(pack.yacc == (short)(short)29779);
            };
            GroundControl.SCALED_IMU2 p116 = CommunicationChannel.new_SCALED_IMU2();
            PH.setPack(p116);
            p116.yacc = (short)(short)29779;
            p116.time_boot_ms = (uint)3605346237U;
            p116.zacc = (short)(short)11530;
            p116.xgyro = (short)(short)31611;
            p116.ygyro = (short)(short) -32058;
            p116.xmag = (short)(short) -1741;
            p116.ymag = (short)(short)21628;
            p116.xacc = (short)(short) -5604;
            p116.zgyro = (short)(short) -1805;
            p116.zmag = (short)(short)16866;
            CommunicationChannel.instance.send(p116);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)209);
                Debug.Assert(pack.end == (ushort)(ushort)5761);
                Debug.Assert(pack.start == (ushort)(ushort)49874);
                Debug.Assert(pack.target_system == (byte)(byte)84);
            };
            GroundControl.LOG_REQUEST_LIST p117 = CommunicationChannel.new_LOG_REQUEST_LIST();
            PH.setPack(p117);
            p117.end = (ushort)(ushort)5761;
            p117.start = (ushort)(ushort)49874;
            p117.target_component = (byte)(byte)209;
            p117.target_system = (byte)(byte)84;
            CommunicationChannel.instance.send(p117);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ENTRYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.num_logs == (ushort)(ushort)64873);
                Debug.Assert(pack.id == (ushort)(ushort)42981);
                Debug.Assert(pack.time_utc == (uint)499531127U);
                Debug.Assert(pack.last_log_num == (ushort)(ushort)55458);
                Debug.Assert(pack.size == (uint)3698872890U);
            };
            GroundControl.LOG_ENTRY p118 = CommunicationChannel.new_LOG_ENTRY();
            PH.setPack(p118);
            p118.last_log_num = (ushort)(ushort)55458;
            p118.num_logs = (ushort)(ushort)64873;
            p118.size = (uint)3698872890U;
            p118.time_utc = (uint)499531127U;
            p118.id = (ushort)(ushort)42981;
            CommunicationChannel.instance.send(p118);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)2853665930U);
                Debug.Assert(pack.target_system == (byte)(byte)152);
                Debug.Assert(pack.count == (uint)64927731U);
                Debug.Assert(pack.target_component == (byte)(byte)68);
                Debug.Assert(pack.id == (ushort)(ushort)9496);
            };
            GroundControl.LOG_REQUEST_DATA p119 = CommunicationChannel.new_LOG_REQUEST_DATA();
            PH.setPack(p119);
            p119.id = (ushort)(ushort)9496;
            p119.count = (uint)64927731U;
            p119.target_system = (byte)(byte)152;
            p119.ofs = (uint)2853665930U;
            p119.target_component = (byte)(byte)68;
            CommunicationChannel.instance.send(p119);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ofs == (uint)3072195656U);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)153, (byte)228, (byte)50, (byte)225, (byte)0, (byte)49, (byte)159, (byte)118, (byte)149, (byte)192, (byte)70, (byte)122, (byte)40, (byte)165, (byte)242, (byte)158, (byte)103, (byte)193, (byte)214, (byte)255, (byte)16, (byte)237, (byte)140, (byte)179, (byte)189, (byte)117, (byte)113, (byte)223, (byte)63, (byte)111, (byte)88, (byte)180, (byte)55, (byte)98, (byte)3, (byte)40, (byte)118, (byte)90, (byte)245, (byte)73, (byte)45, (byte)179, (byte)124, (byte)199, (byte)67, (byte)225, (byte)200, (byte)161, (byte)177, (byte)51, (byte)200, (byte)32, (byte)6, (byte)67, (byte)195, (byte)253, (byte)190, (byte)222, (byte)50, (byte)255, (byte)173, (byte)195, (byte)175, (byte)108, (byte)113, (byte)237, (byte)119, (byte)239, (byte)174, (byte)103, (byte)188, (byte)208, (byte)89, (byte)201, (byte)186, (byte)159, (byte)209, (byte)117, (byte)139, (byte)0, (byte)240, (byte)138, (byte)240, (byte)185, (byte)24, (byte)28, (byte)212, (byte)175, (byte)150, (byte)238}));
                Debug.Assert(pack.id == (ushort)(ushort)3972);
                Debug.Assert(pack.count == (byte)(byte)142);
            };
            GroundControl.LOG_DATA p120 = CommunicationChannel.new_LOG_DATA();
            PH.setPack(p120);
            p120.id = (ushort)(ushort)3972;
            p120.ofs = (uint)3072195656U;
            p120.data__SET(new byte[] {(byte)153, (byte)228, (byte)50, (byte)225, (byte)0, (byte)49, (byte)159, (byte)118, (byte)149, (byte)192, (byte)70, (byte)122, (byte)40, (byte)165, (byte)242, (byte)158, (byte)103, (byte)193, (byte)214, (byte)255, (byte)16, (byte)237, (byte)140, (byte)179, (byte)189, (byte)117, (byte)113, (byte)223, (byte)63, (byte)111, (byte)88, (byte)180, (byte)55, (byte)98, (byte)3, (byte)40, (byte)118, (byte)90, (byte)245, (byte)73, (byte)45, (byte)179, (byte)124, (byte)199, (byte)67, (byte)225, (byte)200, (byte)161, (byte)177, (byte)51, (byte)200, (byte)32, (byte)6, (byte)67, (byte)195, (byte)253, (byte)190, (byte)222, (byte)50, (byte)255, (byte)173, (byte)195, (byte)175, (byte)108, (byte)113, (byte)237, (byte)119, (byte)239, (byte)174, (byte)103, (byte)188, (byte)208, (byte)89, (byte)201, (byte)186, (byte)159, (byte)209, (byte)117, (byte)139, (byte)0, (byte)240, (byte)138, (byte)240, (byte)185, (byte)24, (byte)28, (byte)212, (byte)175, (byte)150, (byte)238}, 0) ;
            p120.count = (byte)(byte)142;
            CommunicationChannel.instance.send(p120);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_ERASEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)213);
                Debug.Assert(pack.target_system == (byte)(byte)108);
            };
            GroundControl.LOG_ERASE p121 = CommunicationChannel.new_LOG_ERASE();
            PH.setPack(p121);
            p121.target_component = (byte)(byte)213;
            p121.target_system = (byte)(byte)108;
            CommunicationChannel.instance.send(p121);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnLOG_REQUEST_ENDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)180);
                Debug.Assert(pack.target_system == (byte)(byte)197);
            };
            GroundControl.LOG_REQUEST_END p122 = CommunicationChannel.new_LOG_REQUEST_END();
            PH.setPack(p122);
            p122.target_component = (byte)(byte)180;
            p122.target_system = (byte)(byte)197;
            CommunicationChannel.instance.send(p122);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_INJECT_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)9);
                Debug.Assert(pack.target_component == (byte)(byte)199);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)106, (byte)162, (byte)200, (byte)254, (byte)33, (byte)33, (byte)31, (byte)180, (byte)133, (byte)126, (byte)1, (byte)81, (byte)143, (byte)51, (byte)15, (byte)200, (byte)104, (byte)239, (byte)134, (byte)246, (byte)218, (byte)244, (byte)179, (byte)212, (byte)157, (byte)78, (byte)131, (byte)33, (byte)19, (byte)167, (byte)78, (byte)28, (byte)105, (byte)38, (byte)194, (byte)6, (byte)54, (byte)57, (byte)50, (byte)155, (byte)249, (byte)90, (byte)245, (byte)45, (byte)205, (byte)140, (byte)52, (byte)232, (byte)193, (byte)181, (byte)113, (byte)176, (byte)63, (byte)245, (byte)238, (byte)38, (byte)190, (byte)84, (byte)183, (byte)6, (byte)147, (byte)28, (byte)145, (byte)24, (byte)205, (byte)48, (byte)127, (byte)91, (byte)99, (byte)88, (byte)204, (byte)77, (byte)24, (byte)198, (byte)30, (byte)58, (byte)106, (byte)108, (byte)166, (byte)81, (byte)99, (byte)159, (byte)25, (byte)10, (byte)25, (byte)108, (byte)158, (byte)206, (byte)160, (byte)10, (byte)177, (byte)124, (byte)29, (byte)0, (byte)247, (byte)185, (byte)191, (byte)191, (byte)220, (byte)105, (byte)149, (byte)64, (byte)109, (byte)167, (byte)232, (byte)140, (byte)180, (byte)223, (byte)199, (byte)104}));
                Debug.Assert(pack.len == (byte)(byte)224);
            };
            GroundControl.GPS_INJECT_DATA p123 = CommunicationChannel.new_GPS_INJECT_DATA();
            PH.setPack(p123);
            p123.data__SET(new byte[] {(byte)106, (byte)162, (byte)200, (byte)254, (byte)33, (byte)33, (byte)31, (byte)180, (byte)133, (byte)126, (byte)1, (byte)81, (byte)143, (byte)51, (byte)15, (byte)200, (byte)104, (byte)239, (byte)134, (byte)246, (byte)218, (byte)244, (byte)179, (byte)212, (byte)157, (byte)78, (byte)131, (byte)33, (byte)19, (byte)167, (byte)78, (byte)28, (byte)105, (byte)38, (byte)194, (byte)6, (byte)54, (byte)57, (byte)50, (byte)155, (byte)249, (byte)90, (byte)245, (byte)45, (byte)205, (byte)140, (byte)52, (byte)232, (byte)193, (byte)181, (byte)113, (byte)176, (byte)63, (byte)245, (byte)238, (byte)38, (byte)190, (byte)84, (byte)183, (byte)6, (byte)147, (byte)28, (byte)145, (byte)24, (byte)205, (byte)48, (byte)127, (byte)91, (byte)99, (byte)88, (byte)204, (byte)77, (byte)24, (byte)198, (byte)30, (byte)58, (byte)106, (byte)108, (byte)166, (byte)81, (byte)99, (byte)159, (byte)25, (byte)10, (byte)25, (byte)108, (byte)158, (byte)206, (byte)160, (byte)10, (byte)177, (byte)124, (byte)29, (byte)0, (byte)247, (byte)185, (byte)191, (byte)191, (byte)220, (byte)105, (byte)149, (byte)64, (byte)109, (byte)167, (byte)232, (byte)140, (byte)180, (byte)223, (byte)199, (byte)104}, 0) ;
            p123.target_component = (byte)(byte)199;
            p123.target_system = (byte)(byte)9;
            p123.len = (byte)(byte)224;
            CommunicationChannel.instance.send(p123);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RAWReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vel == (ushort)(ushort)23108);
                Debug.Assert(pack.eph == (ushort)(ushort)22732);
                Debug.Assert(pack.epv == (ushort)(ushort)61286);
                Debug.Assert(pack.satellites_visible == (byte)(byte)164);
                Debug.Assert(pack.alt == (int) -545761329);
                Debug.Assert(pack.dgps_numch == (byte)(byte)175);
                Debug.Assert(pack.cog == (ushort)(ushort)43899);
                Debug.Assert(pack.dgps_age == (uint)1739539267U);
                Debug.Assert(pack.lon == (int) -376236121);
                Debug.Assert(pack.time_usec == (ulong)7470871973056555158L);
                Debug.Assert(pack.fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
                Debug.Assert(pack.lat == (int)391543455);
            };
            GroundControl.GPS2_RAW p124 = CommunicationChannel.new_GPS2_RAW();
            PH.setPack(p124);
            p124.dgps_age = (uint)1739539267U;
            p124.time_usec = (ulong)7470871973056555158L;
            p124.satellites_visible = (byte)(byte)164;
            p124.eph = (ushort)(ushort)22732;
            p124.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX;
            p124.alt = (int) -545761329;
            p124.lon = (int) -376236121;
            p124.vel = (ushort)(ushort)23108;
            p124.cog = (ushort)(ushort)43899;
            p124.epv = (ushort)(ushort)61286;
            p124.lat = (int)391543455;
            p124.dgps_numch = (byte)(byte)175;
            CommunicationChannel.instance.send(p124);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnPOWER_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.Vservo == (ushort)(ushort)10694);
                Debug.Assert(pack.Vcc == (ushort)(ushort)32851);
                Debug.Assert(pack.flags == (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                                            MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID));
            };
            GroundControl.POWER_STATUS p125 = CommunicationChannel.new_POWER_STATUS();
            PH.setPack(p125);
            p125.Vservo = (ushort)(ushort)10694;
            p125.Vcc = (ushort)(ushort)32851;
            p125.flags = (MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID |
                          MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID);
            CommunicationChannel.instance.send(p125);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSERIAL_CONTROLReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.timeout == (ushort)(ushort)10728);
                Debug.Assert(pack.count == (byte)(byte)216);
                Debug.Assert(pack.device == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)199, (byte)40, (byte)118, (byte)232, (byte)96, (byte)121, (byte)100, (byte)225, (byte)145, (byte)78, (byte)182, (byte)102, (byte)57, (byte)44, (byte)60, (byte)179, (byte)227, (byte)45, (byte)187, (byte)1, (byte)249, (byte)134, (byte)207, (byte)172, (byte)114, (byte)118, (byte)250, (byte)251, (byte)162, (byte)127, (byte)76, (byte)171, (byte)170, (byte)138, (byte)74, (byte)101, (byte)54, (byte)33, (byte)110, (byte)31, (byte)33, (byte)224, (byte)99, (byte)70, (byte)181, (byte)153, (byte)83, (byte)31, (byte)254, (byte)118, (byte)153, (byte)169, (byte)163, (byte)134, (byte)201, (byte)77, (byte)199, (byte)53, (byte)189, (byte)136, (byte)34, (byte)10, (byte)170, (byte)204, (byte)253, (byte)172, (byte)152, (byte)172, (byte)181, (byte)169}));
                Debug.Assert(pack.baudrate == (uint)1950048424U);
                Debug.Assert(pack.flags == (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                                            SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE));
            };
            GroundControl.SERIAL_CONTROL p126 = CommunicationChannel.new_SERIAL_CONTROL();
            PH.setPack(p126);
            p126.device = SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2;
            p126.baudrate = (uint)1950048424U;
            p126.flags = (SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY |
                          SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
            p126.data__SET(new byte[] {(byte)199, (byte)40, (byte)118, (byte)232, (byte)96, (byte)121, (byte)100, (byte)225, (byte)145, (byte)78, (byte)182, (byte)102, (byte)57, (byte)44, (byte)60, (byte)179, (byte)227, (byte)45, (byte)187, (byte)1, (byte)249, (byte)134, (byte)207, (byte)172, (byte)114, (byte)118, (byte)250, (byte)251, (byte)162, (byte)127, (byte)76, (byte)171, (byte)170, (byte)138, (byte)74, (byte)101, (byte)54, (byte)33, (byte)110, (byte)31, (byte)33, (byte)224, (byte)99, (byte)70, (byte)181, (byte)153, (byte)83, (byte)31, (byte)254, (byte)118, (byte)153, (byte)169, (byte)163, (byte)134, (byte)201, (byte)77, (byte)199, (byte)53, (byte)189, (byte)136, (byte)34, (byte)10, (byte)170, (byte)204, (byte)253, (byte)172, (byte)152, (byte)172, (byte)181, (byte)169}, 0) ;
            p126.timeout = (ushort)(ushort)10728;
            p126.count = (byte)(byte)216;
            CommunicationChannel.instance.send(p126);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tow == (uint)346495721U);
                Debug.Assert(pack.rtk_health == (byte)(byte)211);
                Debug.Assert(pack.wn == (ushort)(ushort)13696);
                Debug.Assert(pack.nsats == (byte)(byte)3);
                Debug.Assert(pack.baseline_c_mm == (int) -794891800);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)88);
                Debug.Assert(pack.iar_num_hypotheses == (int) -948561575);
                Debug.Assert(pack.baseline_a_mm == (int) -1793703732);
                Debug.Assert(pack.accuracy == (uint)2548640461U);
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)153);
                Debug.Assert(pack.baseline_b_mm == (int) -510292504);
                Debug.Assert(pack.rtk_rate == (byte)(byte)45);
                Debug.Assert(pack.time_last_baseline_ms == (uint)774758182U);
            };
            GroundControl.GPS_RTK p127 = CommunicationChannel.new_GPS_RTK();
            PH.setPack(p127);
            p127.rtk_health = (byte)(byte)211;
            p127.baseline_coords_type = (byte)(byte)88;
            p127.nsats = (byte)(byte)3;
            p127.baseline_a_mm = (int) -1793703732;
            p127.accuracy = (uint)2548640461U;
            p127.tow = (uint)346495721U;
            p127.rtk_receiver_id = (byte)(byte)153;
            p127.iar_num_hypotheses = (int) -948561575;
            p127.baseline_c_mm = (int) -794891800;
            p127.baseline_b_mm = (int) -510292504;
            p127.rtk_rate = (byte)(byte)45;
            p127.time_last_baseline_ms = (uint)774758182U;
            p127.wn = (ushort)(ushort)13696;
            CommunicationChannel.instance.send(p127);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnGPS2_RTKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rtk_receiver_id == (byte)(byte)233);
                Debug.Assert(pack.wn == (ushort)(ushort)26614);
                Debug.Assert(pack.rtk_rate == (byte)(byte)203);
                Debug.Assert(pack.time_last_baseline_ms == (uint)3399739602U);
                Debug.Assert(pack.baseline_b_mm == (int) -492061104);
                Debug.Assert(pack.tow == (uint)4234425262U);
                Debug.Assert(pack.nsats == (byte)(byte)102);
                Debug.Assert(pack.iar_num_hypotheses == (int) -296698558);
                Debug.Assert(pack.accuracy == (uint)1325944113U);
                Debug.Assert(pack.baseline_coords_type == (byte)(byte)94);
                Debug.Assert(pack.baseline_c_mm == (int)1868981953);
                Debug.Assert(pack.rtk_health == (byte)(byte)135);
                Debug.Assert(pack.baseline_a_mm == (int) -1211128240);
            };
            GroundControl.GPS2_RTK p128 = CommunicationChannel.new_GPS2_RTK();
            PH.setPack(p128);
            p128.baseline_a_mm = (int) -1211128240;
            p128.baseline_b_mm = (int) -492061104;
            p128.iar_num_hypotheses = (int) -296698558;
            p128.nsats = (byte)(byte)102;
            p128.rtk_receiver_id = (byte)(byte)233;
            p128.tow = (uint)4234425262U;
            p128.baseline_c_mm = (int)1868981953;
            p128.wn = (ushort)(ushort)26614;
            p128.accuracy = (uint)1325944113U;
            p128.time_last_baseline_ms = (uint)3399739602U;
            p128.rtk_health = (byte)(byte)135;
            p128.rtk_rate = (byte)(byte)203;
            p128.baseline_coords_type = (byte)(byte)94;
            CommunicationChannel.instance.send(p128);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_IMU3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.xgyro == (short)(short) -12339);
                Debug.Assert(pack.ygyro == (short)(short) -22462);
                Debug.Assert(pack.zmag == (short)(short)9867);
                Debug.Assert(pack.xmag == (short)(short)28318);
                Debug.Assert(pack.zgyro == (short)(short)29350);
                Debug.Assert(pack.xacc == (short)(short)17779);
                Debug.Assert(pack.zacc == (short)(short) -10064);
                Debug.Assert(pack.yacc == (short)(short) -24829);
                Debug.Assert(pack.time_boot_ms == (uint)2905539710U);
                Debug.Assert(pack.ymag == (short)(short) -28456);
            };
            GroundControl.SCALED_IMU3 p129 = CommunicationChannel.new_SCALED_IMU3();
            PH.setPack(p129);
            p129.ymag = (short)(short) -28456;
            p129.ygyro = (short)(short) -22462;
            p129.zmag = (short)(short)9867;
            p129.xgyro = (short)(short) -12339;
            p129.zgyro = (short)(short)29350;
            p129.zacc = (short)(short) -10064;
            p129.xmag = (short)(short)28318;
            p129.yacc = (short)(short) -24829;
            p129.xacc = (short)(short)17779;
            p129.time_boot_ms = (uint)2905539710U;
            CommunicationChannel.instance.send(p129);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDATA_TRANSMISSION_HANDSHAKEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload == (byte)(byte)109);
                Debug.Assert(pack.height == (ushort)(ushort)64387);
                Debug.Assert(pack.jpg_quality == (byte)(byte)28);
                Debug.Assert(pack.type == (byte)(byte)26);
                Debug.Assert(pack.size == (uint)3733905738U);
                Debug.Assert(pack.packets == (ushort)(ushort)50704);
                Debug.Assert(pack.width == (ushort)(ushort)31366);
            };
            GroundControl.DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.new_DATA_TRANSMISSION_HANDSHAKE();
            PH.setPack(p130);
            p130.payload = (byte)(byte)109;
            p130.jpg_quality = (byte)(byte)28;
            p130.packets = (ushort)(ushort)50704;
            p130.size = (uint)3733905738U;
            p130.height = (ushort)(ushort)64387;
            p130.width = (ushort)(ushort)31366;
            p130.type = (byte)(byte)26;
            CommunicationChannel.instance.send(p130);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnENCAPSULATED_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)22, (byte)154, (byte)51, (byte)2, (byte)212, (byte)202, (byte)161, (byte)18, (byte)228, (byte)88, (byte)13, (byte)2, (byte)36, (byte)97, (byte)4, (byte)48, (byte)103, (byte)211, (byte)16, (byte)162, (byte)26, (byte)229, (byte)167, (byte)200, (byte)111, (byte)51, (byte)190, (byte)72, (byte)63, (byte)168, (byte)71, (byte)188, (byte)145, (byte)17, (byte)175, (byte)238, (byte)111, (byte)180, (byte)89, (byte)195, (byte)123, (byte)90, (byte)83, (byte)134, (byte)132, (byte)70, (byte)152, (byte)130, (byte)228, (byte)201, (byte)201, (byte)34, (byte)204, (byte)130, (byte)93, (byte)80, (byte)73, (byte)150, (byte)231, (byte)83, (byte)12, (byte)93, (byte)208, (byte)203, (byte)124, (byte)98, (byte)213, (byte)248, (byte)22, (byte)35, (byte)93, (byte)128, (byte)126, (byte)112, (byte)107, (byte)43, (byte)111, (byte)160, (byte)66, (byte)241, (byte)160, (byte)253, (byte)190, (byte)57, (byte)80, (byte)40, (byte)124, (byte)161, (byte)9, (byte)121, (byte)69, (byte)202, (byte)198, (byte)136, (byte)38, (byte)47, (byte)242, (byte)221, (byte)176, (byte)200, (byte)224, (byte)155, (byte)127, (byte)100, (byte)174, (byte)97, (byte)251, (byte)94, (byte)20, (byte)93, (byte)161, (byte)61, (byte)15, (byte)157, (byte)78, (byte)222, (byte)48, (byte)78, (byte)51, (byte)97, (byte)197, (byte)204, (byte)80, (byte)86, (byte)82, (byte)15, (byte)16, (byte)146, (byte)29, (byte)22, (byte)1, (byte)34, (byte)221, (byte)215, (byte)179, (byte)68, (byte)176, (byte)27, (byte)145, (byte)181, (byte)106, (byte)51, (byte)151, (byte)27, (byte)140, (byte)11, (byte)19, (byte)54, (byte)193, (byte)36, (byte)116, (byte)110, (byte)111, (byte)235, (byte)241, (byte)28, (byte)72, (byte)213, (byte)112, (byte)68, (byte)244, (byte)2, (byte)255, (byte)146, (byte)123, (byte)174, (byte)163, (byte)184, (byte)166, (byte)145, (byte)198, (byte)255, (byte)222, (byte)156, (byte)237, (byte)20, (byte)247, (byte)37, (byte)72, (byte)77, (byte)15, (byte)57, (byte)236, (byte)154, (byte)228, (byte)132, (byte)105, (byte)14, (byte)42, (byte)242, (byte)205, (byte)98, (byte)91, (byte)208, (byte)255, (byte)26, (byte)173, (byte)175, (byte)233, (byte)251, (byte)189, (byte)165, (byte)224, (byte)175, (byte)252, (byte)47, (byte)167, (byte)76, (byte)8, (byte)28, (byte)194, (byte)125, (byte)170, (byte)63, (byte)11, (byte)196, (byte)235, (byte)255, (byte)54, (byte)85, (byte)42, (byte)78, (byte)18, (byte)100, (byte)59, (byte)126, (byte)85, (byte)247, (byte)117, (byte)18, (byte)7, (byte)5, (byte)247, (byte)201, (byte)122, (byte)136, (byte)204, (byte)115, (byte)88, (byte)198, (byte)135, (byte)188, (byte)67, (byte)152, (byte)87, (byte)71, (byte)226, (byte)133, (byte)5, (byte)151, (byte)108, (byte)173, (byte)110}));
                Debug.Assert(pack.seqnr == (ushort)(ushort)63435);
            };
            GroundControl.ENCAPSULATED_DATA p131 = CommunicationChannel.new_ENCAPSULATED_DATA();
            PH.setPack(p131);
            p131.seqnr = (ushort)(ushort)63435;
            p131.data__SET(new byte[] {(byte)22, (byte)154, (byte)51, (byte)2, (byte)212, (byte)202, (byte)161, (byte)18, (byte)228, (byte)88, (byte)13, (byte)2, (byte)36, (byte)97, (byte)4, (byte)48, (byte)103, (byte)211, (byte)16, (byte)162, (byte)26, (byte)229, (byte)167, (byte)200, (byte)111, (byte)51, (byte)190, (byte)72, (byte)63, (byte)168, (byte)71, (byte)188, (byte)145, (byte)17, (byte)175, (byte)238, (byte)111, (byte)180, (byte)89, (byte)195, (byte)123, (byte)90, (byte)83, (byte)134, (byte)132, (byte)70, (byte)152, (byte)130, (byte)228, (byte)201, (byte)201, (byte)34, (byte)204, (byte)130, (byte)93, (byte)80, (byte)73, (byte)150, (byte)231, (byte)83, (byte)12, (byte)93, (byte)208, (byte)203, (byte)124, (byte)98, (byte)213, (byte)248, (byte)22, (byte)35, (byte)93, (byte)128, (byte)126, (byte)112, (byte)107, (byte)43, (byte)111, (byte)160, (byte)66, (byte)241, (byte)160, (byte)253, (byte)190, (byte)57, (byte)80, (byte)40, (byte)124, (byte)161, (byte)9, (byte)121, (byte)69, (byte)202, (byte)198, (byte)136, (byte)38, (byte)47, (byte)242, (byte)221, (byte)176, (byte)200, (byte)224, (byte)155, (byte)127, (byte)100, (byte)174, (byte)97, (byte)251, (byte)94, (byte)20, (byte)93, (byte)161, (byte)61, (byte)15, (byte)157, (byte)78, (byte)222, (byte)48, (byte)78, (byte)51, (byte)97, (byte)197, (byte)204, (byte)80, (byte)86, (byte)82, (byte)15, (byte)16, (byte)146, (byte)29, (byte)22, (byte)1, (byte)34, (byte)221, (byte)215, (byte)179, (byte)68, (byte)176, (byte)27, (byte)145, (byte)181, (byte)106, (byte)51, (byte)151, (byte)27, (byte)140, (byte)11, (byte)19, (byte)54, (byte)193, (byte)36, (byte)116, (byte)110, (byte)111, (byte)235, (byte)241, (byte)28, (byte)72, (byte)213, (byte)112, (byte)68, (byte)244, (byte)2, (byte)255, (byte)146, (byte)123, (byte)174, (byte)163, (byte)184, (byte)166, (byte)145, (byte)198, (byte)255, (byte)222, (byte)156, (byte)237, (byte)20, (byte)247, (byte)37, (byte)72, (byte)77, (byte)15, (byte)57, (byte)236, (byte)154, (byte)228, (byte)132, (byte)105, (byte)14, (byte)42, (byte)242, (byte)205, (byte)98, (byte)91, (byte)208, (byte)255, (byte)26, (byte)173, (byte)175, (byte)233, (byte)251, (byte)189, (byte)165, (byte)224, (byte)175, (byte)252, (byte)47, (byte)167, (byte)76, (byte)8, (byte)28, (byte)194, (byte)125, (byte)170, (byte)63, (byte)11, (byte)196, (byte)235, (byte)255, (byte)54, (byte)85, (byte)42, (byte)78, (byte)18, (byte)100, (byte)59, (byte)126, (byte)85, (byte)247, (byte)117, (byte)18, (byte)7, (byte)5, (byte)247, (byte)201, (byte)122, (byte)136, (byte)204, (byte)115, (byte)88, (byte)198, (byte)135, (byte)188, (byte)67, (byte)152, (byte)87, (byte)71, (byte)226, (byte)133, (byte)5, (byte)151, (byte)108, (byte)173, (byte)110}, 0) ;
            CommunicationChannel.instance.send(p131);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnDISTANCE_SENSORReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.min_distance == (ushort)(ushort)19518);
                Debug.Assert(pack.id == (byte)(byte)64);
                Debug.Assert(pack.max_distance == (ushort)(ushort)32083);
                Debug.Assert(pack.type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
                Debug.Assert(pack.current_distance == (ushort)(ushort)48351);
                Debug.Assert(pack.covariance == (byte)(byte)87);
                Debug.Assert(pack.orientation == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_PITCH_270);
                Debug.Assert(pack.time_boot_ms == (uint)2653805061U);
            };
            GroundControl.DISTANCE_SENSOR p132 = CommunicationChannel.new_DISTANCE_SENSOR();
            PH.setPack(p132);
            p132.current_distance = (ushort)(ushort)48351;
            p132.min_distance = (ushort)(ushort)19518;
            p132.orientation = MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_PITCH_270;
            p132.id = (byte)(byte)64;
            p132.time_boot_ms = (uint)2653805061U;
            p132.type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED;
            p132.covariance = (byte)(byte)87;
            p132.max_distance = (ushort)(ushort)32083;
            CommunicationChannel.instance.send(p132);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mask == (ulong)2389803566319645692L);
                Debug.Assert(pack.lat == (int)857492860);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)3584);
                Debug.Assert(pack.lon == (int)1063967916);
            };
            GroundControl.TERRAIN_REQUEST p133 = CommunicationChannel.new_TERRAIN_REQUEST();
            PH.setPack(p133);
            p133.grid_spacing = (ushort)(ushort)3584;
            p133.lat = (int)857492860;
            p133.mask = (ulong)2389803566319645692L;
            p133.lon = (int)1063967916;
            CommunicationChannel.instance.send(p133);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int)1035805772);
                Debug.Assert(pack.grid_spacing == (ushort)(ushort)47423);
                Debug.Assert(pack.gridbit == (byte)(byte)79);
                Debug.Assert(pack.data_.SequenceEqual(new short[] {(short) -5469, (short) -32724, (short) -1319, (short) -8527, (short)16801, (short)22322, (short)16830, (short) -24484, (short) -15740, (short)22902, (short) -30806, (short)10613, (short)12624, (short)8372, (short)21208, (short) -19472}));
                Debug.Assert(pack.lon == (int) -1967558925);
            };
            GroundControl.TERRAIN_DATA p134 = CommunicationChannel.new_TERRAIN_DATA();
            PH.setPack(p134);
            p134.data__SET(new short[] {(short) -5469, (short) -32724, (short) -1319, (short) -8527, (short)16801, (short)22322, (short)16830, (short) -24484, (short) -15740, (short)22902, (short) -30806, (short)10613, (short)12624, (short)8372, (short)21208, (short) -19472}, 0) ;
            p134.gridbit = (byte)(byte)79;
            p134.lat = (int)1035805772;
            p134.lon = (int) -1967558925;
            p134.grid_spacing = (ushort)(ushort)47423;
            CommunicationChannel.instance.send(p134);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_CHECKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lon == (int)876255372);
                Debug.Assert(pack.lat == (int)339115249);
            };
            GroundControl.TERRAIN_CHECK p135 = CommunicationChannel.new_TERRAIN_CHECK();
            PH.setPack(p135);
            p135.lon = (int)876255372;
            p135.lat = (int)339115249;
            CommunicationChannel.instance.send(p135);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnTERRAIN_REPORTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.current_height == (float) -6.2797083E37F);
                Debug.Assert(pack.terrain_height == (float) -2.5951148E38F);
                Debug.Assert(pack.lon == (int) -974121517);
                Debug.Assert(pack.pending == (ushort)(ushort)20493);
                Debug.Assert(pack.loaded == (ushort)(ushort)6681);
                Debug.Assert(pack.spacing == (ushort)(ushort)35530);
                Debug.Assert(pack.lat == (int) -1152146335);
            };
            GroundControl.TERRAIN_REPORT p136 = CommunicationChannel.new_TERRAIN_REPORT();
            PH.setPack(p136);
            p136.current_height = (float) -6.2797083E37F;
            p136.spacing = (ushort)(ushort)35530;
            p136.lon = (int) -974121517;
            p136.terrain_height = (float) -2.5951148E38F;
            p136.loaded = (ushort)(ushort)6681;
            p136.lat = (int) -1152146335;
            p136.pending = (ushort)(ushort)20493;
            CommunicationChannel.instance.send(p136);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSCALED_PRESSURE2Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.press_diff == (float)1.6659858E38F);
                Debug.Assert(pack.press_abs == (float)1.2416449E38F);
                Debug.Assert(pack.time_boot_ms == (uint)3120355925U);
                Debug.Assert(pack.temperature == (short)(short)2255);
            };
            GroundControl.SCALED_PRESSURE2 p137 = CommunicationChannel.new_SCALED_PRESSURE2();
            PH.setPack(p137);
            p137.press_diff = (float)1.6659858E38F;
            p137.press_abs = (float)1.2416449E38F;
            p137.temperature = (short)(short)2255;
            p137.time_boot_ms = (uint)3120355925U;
            CommunicationChannel.instance.send(p137);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnATT_POS_MOCAPReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.x == (float)2.9446317E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-2.1079979E38F, 2.457332E38F, -8.482797E37F, 3.3500235E37F}));
                Debug.Assert(pack.time_usec == (ulong)7089739996931048268L);
                Debug.Assert(pack.y == (float) -2.4393453E38F);
                Debug.Assert(pack.z == (float)1.6775411E38F);
            };
            GroundControl.ATT_POS_MOCAP p138 = CommunicationChannel.new_ATT_POS_MOCAP();
            PH.setPack(p138);
            p138.q_SET(new float[] {-2.1079979E38F, 2.457332E38F, -8.482797E37F, 3.3500235E37F}, 0) ;
            p138.y = (float) -2.4393453E38F;
            p138.z = (float)1.6775411E38F;
            p138.x = (float)2.9446317E38F;
            p138.time_usec = (ulong)7089739996931048268L;
            CommunicationChannel.instance.send(p138);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnSET_ACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)37);
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-1.9871366E38F, 1.3543581E38F, 1.456204E37F, 2.004416E38F, 2.5661793E38F, -1.1737595E38F, -1.8450762E38F, 2.0225528E38F}));
                Debug.Assert(pack.target_system == (byte)(byte)210);
                Debug.Assert(pack.group_mlx == (byte)(byte)253);
                Debug.Assert(pack.time_usec == (ulong)8829313847919204513L);
            };
            GroundControl.SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.new_SET_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p139);
            p139.target_system = (byte)(byte)210;
            p139.group_mlx = (byte)(byte)253;
            p139.target_component = (byte)(byte)37;
            p139.controls_SET(new float[] {-1.9871366E38F, 1.3543581E38F, 1.456204E37F, 2.004416E38F, 2.5661793E38F, -1.1737595E38F, -1.8450762E38F, 2.0225528E38F}, 0) ;
            p139.time_usec = (ulong)8829313847919204513L;
            CommunicationChannel.instance.send(p139);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            CommunicationChannel.instance.OnACTUATOR_CONTROL_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.controls.SequenceEqual(new float[] {-2.9740982E38F, -2.335385E38F, 1.1765016E38F, -1.9168778E38F, 2.6863784E38F, -2.612726E38F, -1.3777613E38F, -3.5640106E37F}));
                Debug.Assert(pack.time_usec == (ulong)7279164704251636027L);
                Debug.Assert(pack.group_mlx == (byte)(byte)26);
            };
            GroundControl.ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.new_ACTUATOR_CONTROL_TARGET();
            PH.setPack(p140);
            p140.group_mlx = (byte)(byte)26;
            p140.time_usec = (ulong)7279164704251636027L;
            p140.controls_SET(new float[] {-2.9740982E38F, -2.335385E38F, 1.1765016E38F, -1.9168778E38F, 2.6863784E38F, -2.612726E38F, -1.3777613E38F, -3.5640106E37F}, 0) ;
            CommunicationChannel.instance.send(p140);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, CommunicationChannel.instance);
            ADV_TEST_CH.OnALTITUDEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)424657960533722193L);
                Debug.Assert(pack.altitude_relative == (float) -2.5294716E38F);
                Debug.Assert(pack.altitude_terrain == (float) -1.9406037E38F);
                Debug.Assert(pack.altitude_local == (float)1.9724501E38F);
                Debug.Assert(pack.altitude_amsl == (float)9.699698E37F);
                Debug.Assert(pack.bottom_clearance == (float) -3.2862692E38F);
                Debug.Assert(pack.altitude_monotonic == (float) -5.1618357E37F);
            };
            GroundControl.ALTITUDE p141 = CommunicationChannel.new_ALTITUDE();
            PH.setPack(p141);
            p141.bottom_clearance = (float) -3.2862692E38F;
            p141.altitude_amsl = (float)9.699698E37F;
            p141.altitude_local = (float)1.9724501E38F;
            p141.altitude_terrain = (float) -1.9406037E38F;
            p141.time_usec = (ulong)424657960533722193L;
            p141.altitude_relative = (float) -2.5294716E38F;
            p141.altitude_monotonic = (float) -5.1618357E37F;
            CommunicationChannel.instance.send(p141);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRESOURCE_REQUESTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.uri_type == (byte)(byte)51);
                Debug.Assert(pack.request_id == (byte)(byte)65);
                Debug.Assert(pack.uri.SequenceEqual(new byte[] {(byte)22, (byte)83, (byte)133, (byte)92, (byte)45, (byte)114, (byte)7, (byte)104, (byte)127, (byte)66, (byte)89, (byte)20, (byte)44, (byte)40, (byte)209, (byte)69, (byte)129, (byte)204, (byte)214, (byte)216, (byte)206, (byte)9, (byte)214, (byte)51, (byte)204, (byte)73, (byte)255, (byte)113, (byte)136, (byte)37, (byte)185, (byte)125, (byte)80, (byte)47, (byte)57, (byte)84, (byte)189, (byte)188, (byte)88, (byte)127, (byte)142, (byte)158, (byte)97, (byte)101, (byte)243, (byte)102, (byte)53, (byte)33, (byte)160, (byte)11, (byte)93, (byte)141, (byte)34, (byte)162, (byte)42, (byte)145, (byte)189, (byte)253, (byte)2, (byte)202, (byte)172, (byte)225, (byte)15, (byte)171, (byte)214, (byte)67, (byte)81, (byte)97, (byte)53, (byte)155, (byte)59, (byte)227, (byte)227, (byte)24, (byte)148, (byte)50, (byte)252, (byte)229, (byte)171, (byte)150, (byte)166, (byte)44, (byte)186, (byte)254, (byte)71, (byte)255, (byte)114, (byte)32, (byte)214, (byte)142, (byte)142, (byte)62, (byte)68, (byte)116, (byte)253, (byte)37, (byte)200, (byte)109, (byte)229, (byte)151, (byte)61, (byte)180, (byte)161, (byte)181, (byte)58, (byte)53, (byte)39, (byte)85, (byte)247, (byte)75, (byte)64, (byte)148, (byte)130, (byte)170, (byte)79, (byte)137, (byte)217, (byte)223, (byte)129, (byte)198}));
                Debug.Assert(pack.storage.SequenceEqual(new byte[] {(byte)12, (byte)230, (byte)158, (byte)253, (byte)229, (byte)84, (byte)161, (byte)210, (byte)13, (byte)248, (byte)97, (byte)169, (byte)121, (byte)105, (byte)154, (byte)22, (byte)32, (byte)132, (byte)238, (byte)70, (byte)147, (byte)172, (byte)191, (byte)150, (byte)66, (byte)136, (byte)25, (byte)249, (byte)31, (byte)227, (byte)148, (byte)255, (byte)140, (byte)180, (byte)23, (byte)58, (byte)64, (byte)14, (byte)72, (byte)241, (byte)231, (byte)202, (byte)166, (byte)135, (byte)50, (byte)61, (byte)163, (byte)208, (byte)154, (byte)20, (byte)161, (byte)91, (byte)219, (byte)179, (byte)70, (byte)58, (byte)139, (byte)116, (byte)83, (byte)50, (byte)42, (byte)1, (byte)221, (byte)59, (byte)60, (byte)221, (byte)95, (byte)38, (byte)123, (byte)29, (byte)155, (byte)162, (byte)218, (byte)149, (byte)93, (byte)248, (byte)87, (byte)133, (byte)191, (byte)182, (byte)72, (byte)13, (byte)231, (byte)198, (byte)91, (byte)43, (byte)26, (byte)53, (byte)186, (byte)244, (byte)210, (byte)55, (byte)150, (byte)208, (byte)127, (byte)128, (byte)86, (byte)92, (byte)73, (byte)199, (byte)107, (byte)188, (byte)43, (byte)9, (byte)139, (byte)175, (byte)227, (byte)165, (byte)153, (byte)186, (byte)248, (byte)140, (byte)157, (byte)117, (byte)107, (byte)82, (byte)149, (byte)54, (byte)11, (byte)198}));
                Debug.Assert(pack.transfer_type == (byte)(byte)48);
            };
            GroundControl.RESOURCE_REQUEST p142 = CommunicationChannel.new_RESOURCE_REQUEST();
            PH.setPack(p142);
            p142.uri_type = (byte)(byte)51;
            p142.transfer_type = (byte)(byte)48;
            p142.request_id = (byte)(byte)65;
            p142.storage_SET(new byte[] {(byte)12, (byte)230, (byte)158, (byte)253, (byte)229, (byte)84, (byte)161, (byte)210, (byte)13, (byte)248, (byte)97, (byte)169, (byte)121, (byte)105, (byte)154, (byte)22, (byte)32, (byte)132, (byte)238, (byte)70, (byte)147, (byte)172, (byte)191, (byte)150, (byte)66, (byte)136, (byte)25, (byte)249, (byte)31, (byte)227, (byte)148, (byte)255, (byte)140, (byte)180, (byte)23, (byte)58, (byte)64, (byte)14, (byte)72, (byte)241, (byte)231, (byte)202, (byte)166, (byte)135, (byte)50, (byte)61, (byte)163, (byte)208, (byte)154, (byte)20, (byte)161, (byte)91, (byte)219, (byte)179, (byte)70, (byte)58, (byte)139, (byte)116, (byte)83, (byte)50, (byte)42, (byte)1, (byte)221, (byte)59, (byte)60, (byte)221, (byte)95, (byte)38, (byte)123, (byte)29, (byte)155, (byte)162, (byte)218, (byte)149, (byte)93, (byte)248, (byte)87, (byte)133, (byte)191, (byte)182, (byte)72, (byte)13, (byte)231, (byte)198, (byte)91, (byte)43, (byte)26, (byte)53, (byte)186, (byte)244, (byte)210, (byte)55, (byte)150, (byte)208, (byte)127, (byte)128, (byte)86, (byte)92, (byte)73, (byte)199, (byte)107, (byte)188, (byte)43, (byte)9, (byte)139, (byte)175, (byte)227, (byte)165, (byte)153, (byte)186, (byte)248, (byte)140, (byte)157, (byte)117, (byte)107, (byte)82, (byte)149, (byte)54, (byte)11, (byte)198}, 0) ;
            p142.uri_SET(new byte[] {(byte)22, (byte)83, (byte)133, (byte)92, (byte)45, (byte)114, (byte)7, (byte)104, (byte)127, (byte)66, (byte)89, (byte)20, (byte)44, (byte)40, (byte)209, (byte)69, (byte)129, (byte)204, (byte)214, (byte)216, (byte)206, (byte)9, (byte)214, (byte)51, (byte)204, (byte)73, (byte)255, (byte)113, (byte)136, (byte)37, (byte)185, (byte)125, (byte)80, (byte)47, (byte)57, (byte)84, (byte)189, (byte)188, (byte)88, (byte)127, (byte)142, (byte)158, (byte)97, (byte)101, (byte)243, (byte)102, (byte)53, (byte)33, (byte)160, (byte)11, (byte)93, (byte)141, (byte)34, (byte)162, (byte)42, (byte)145, (byte)189, (byte)253, (byte)2, (byte)202, (byte)172, (byte)225, (byte)15, (byte)171, (byte)214, (byte)67, (byte)81, (byte)97, (byte)53, (byte)155, (byte)59, (byte)227, (byte)227, (byte)24, (byte)148, (byte)50, (byte)252, (byte)229, (byte)171, (byte)150, (byte)166, (byte)44, (byte)186, (byte)254, (byte)71, (byte)255, (byte)114, (byte)32, (byte)214, (byte)142, (byte)142, (byte)62, (byte)68, (byte)116, (byte)253, (byte)37, (byte)200, (byte)109, (byte)229, (byte)151, (byte)61, (byte)180, (byte)161, (byte)181, (byte)58, (byte)53, (byte)39, (byte)85, (byte)247, (byte)75, (byte)64, (byte)148, (byte)130, (byte)170, (byte)79, (byte)137, (byte)217, (byte)223, (byte)129, (byte)198}, 0) ;
            CommunicationChannel.instance.send(p142);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSCALED_PRESSURE3Receive += (src, ph, pack) =>
            {
                Debug.Assert(pack.temperature == (short)(short) -29673);
                Debug.Assert(pack.press_diff == (float)2.250029E36F);
                Debug.Assert(pack.press_abs == (float) -3.8763782E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1977583400U);
            };
            GroundControl.SCALED_PRESSURE3 p143 = CommunicationChannel.new_SCALED_PRESSURE3();
            PH.setPack(p143);
            p143.time_boot_ms = (uint)1977583400U;
            p143.temperature = (short)(short) -29673;
            p143.press_abs = (float) -3.8763782E37F;
            p143.press_diff = (float)2.250029E36F;
            CommunicationChannel.instance.send(p143);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFOLLOW_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.lat == (int) -433567704);
                Debug.Assert(pack.position_cov.SequenceEqual(new float[] {-8.940767E37F, -2.613461E38F, 1.140816E38F}));
                Debug.Assert(pack.alt == (float) -7.204209E37F);
                Debug.Assert(pack.attitude_q.SequenceEqual(new float[] {-2.9899551E38F, 3.47319E36F, 1.258318E38F, -1.74186E38F}));
                Debug.Assert(pack.timestamp == (ulong)4737963854163689212L);
                Debug.Assert(pack.est_capabilities == (byte)(byte)115);
                Debug.Assert(pack.vel.SequenceEqual(new float[] {3.319367E38F, 3.0747537E38F, -2.0832708E37F}));
                Debug.Assert(pack.acc.SequenceEqual(new float[] {-2.88359E38F, 9.407708E37F, 1.5908398E38F}));
                Debug.Assert(pack.rates.SequenceEqual(new float[] {2.5525226E38F, -2.2764745E38F, 6.196509E37F}));
                Debug.Assert(pack.custom_state == (ulong)2288689473800449055L);
                Debug.Assert(pack.lon == (int) -651417174);
            };
            GroundControl.FOLLOW_TARGET p144 = CommunicationChannel.new_FOLLOW_TARGET();
            PH.setPack(p144);
            p144.position_cov_SET(new float[] {-8.940767E37F, -2.613461E38F, 1.140816E38F}, 0) ;
            p144.alt = (float) -7.204209E37F;
            p144.rates_SET(new float[] {2.5525226E38F, -2.2764745E38F, 6.196509E37F}, 0) ;
            p144.acc_SET(new float[] {-2.88359E38F, 9.407708E37F, 1.5908398E38F}, 0) ;
            p144.est_capabilities = (byte)(byte)115;
            p144.attitude_q_SET(new float[] {-2.9899551E38F, 3.47319E36F, 1.258318E38F, -1.74186E38F}, 0) ;
            p144.lat = (int) -433567704;
            p144.vel_SET(new float[] {3.319367E38F, 3.0747537E38F, -2.0832708E37F}, 0) ;
            p144.lon = (int) -651417174;
            p144.custom_state = (ulong)2288689473800449055L;
            p144.timestamp = (ulong)4737963854163689212L;
            CommunicationChannel.instance.send(p144);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCONTROL_SYSTEM_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)4028853955189485581L);
                Debug.Assert(pack.vel_variance.SequenceEqual(new float[] {-1.1234292E38F, -1.4589288E38F, 1.3835614E38F}));
                Debug.Assert(pack.x_acc == (float)9.642614E36F);
                Debug.Assert(pack.y_pos == (float) -1.2215498E37F);
                Debug.Assert(pack.yaw_rate == (float)2.2527236E38F);
                Debug.Assert(pack.z_vel == (float) -2.6433247E38F);
                Debug.Assert(pack.y_vel == (float)1.6050476E38F);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.4274842E38F, -1.3156558E38F, 7.794843E37F, -2.2571444E38F}));
                Debug.Assert(pack.airspeed == (float)2.5673397E38F);
                Debug.Assert(pack.pitch_rate == (float)9.084362E37F);
                Debug.Assert(pack.y_acc == (float)1.0185343E38F);
                Debug.Assert(pack.pos_variance.SequenceEqual(new float[] {1.6139826E38F, 8.953624E37F, 2.8226157E38F}));
                Debug.Assert(pack.roll_rate == (float) -6.275578E36F);
                Debug.Assert(pack.x_pos == (float)1.648342E38F);
                Debug.Assert(pack.z_acc == (float) -4.934358E37F);
                Debug.Assert(pack.x_vel == (float) -1.2586801E38F);
                Debug.Assert(pack.z_pos == (float) -5.4004E37F);
            };
            GroundControl.CONTROL_SYSTEM_STATE p146 = CommunicationChannel.new_CONTROL_SYSTEM_STATE();
            PH.setPack(p146);
            p146.y_acc = (float)1.0185343E38F;
            p146.x_acc = (float)9.642614E36F;
            p146.airspeed = (float)2.5673397E38F;
            p146.time_usec = (ulong)4028853955189485581L;
            p146.z_pos = (float) -5.4004E37F;
            p146.vel_variance_SET(new float[] {-1.1234292E38F, -1.4589288E38F, 1.3835614E38F}, 0) ;
            p146.roll_rate = (float) -6.275578E36F;
            p146.z_acc = (float) -4.934358E37F;
            p146.x_vel = (float) -1.2586801E38F;
            p146.pitch_rate = (float)9.084362E37F;
            p146.y_vel = (float)1.6050476E38F;
            p146.y_pos = (float) -1.2215498E37F;
            p146.z_vel = (float) -2.6433247E38F;
            p146.yaw_rate = (float)2.2527236E38F;
            p146.pos_variance_SET(new float[] {1.6139826E38F, 8.953624E37F, 2.8226157E38F}, 0) ;
            p146.q_SET(new float[] {-1.4274842E38F, -1.3156558E38F, 7.794843E37F, -2.2571444E38F}, 0) ;
            p146.x_pos = (float)1.648342E38F;
            CommunicationChannel.instance.send(p146);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBATTERY_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.battery_function == MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
                Debug.Assert(pack.id == (byte)(byte)221);
                Debug.Assert(pack.voltages.SequenceEqual(new ushort[] {(ushort)6651, (ushort)24613, (ushort)18413, (ushort)860, (ushort)10539, (ushort)19697, (ushort)5769, (ushort)64389, (ushort)52770, (ushort)28061}));
                Debug.Assert(pack.battery_remaining == (sbyte)(sbyte) - 35);
                Debug.Assert(pack.current_battery == (short)(short) -25682);
                Debug.Assert(pack.energy_consumed == (int) -494706428);
                Debug.Assert(pack.temperature == (short)(short)10032);
                Debug.Assert(pack.current_consumed == (int) -1142592624);
                Debug.Assert(pack.type == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
            };
            GroundControl.BATTERY_STATUS p147 = CommunicationChannel.new_BATTERY_STATUS();
            PH.setPack(p147);
            p147.type = MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE;
            p147.voltages_SET(new ushort[] {(ushort)6651, (ushort)24613, (ushort)18413, (ushort)860, (ushort)10539, (ushort)19697, (ushort)5769, (ushort)64389, (ushort)52770, (ushort)28061}, 0) ;
            p147.energy_consumed = (int) -494706428;
            p147.id = (byte)(byte)221;
            p147.temperature = (short)(short)10032;
            p147.battery_remaining = (sbyte)(sbyte) - 35;
            p147.battery_function = MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD;
            p147.current_consumed = (int) -1142592624;
            p147.current_battery = (short)(short) -25682;
            CommunicationChannel.instance.send(p147);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnAUTOPILOT_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flight_sw_version == (uint)1335468481U);
                Debug.Assert(pack.uid == (ulong)2581642865699990701L);
                Debug.Assert(pack.capabilities == (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                                   MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT));
                Debug.Assert(pack.middleware_sw_version == (uint)3893898485U);
                Debug.Assert(pack.vendor_id == (ushort)(ushort)50682);
                Debug.Assert(pack.product_id == (ushort)(ushort)11218);
                Debug.Assert(pack.board_version == (uint)3310532352U);
                Debug.Assert(pack.os_sw_version == (uint)2206202373U);
                Debug.Assert(pack.uid2_TRY(ph).SequenceEqual(new byte[] {(byte)138, (byte)135, (byte)10, (byte)88, (byte)70, (byte)148, (byte)124, (byte)137, (byte)98, (byte)39, (byte)205, (byte)220, (byte)16, (byte)139, (byte)57, (byte)250, (byte)117, (byte)27}));
                Debug.Assert(pack.middleware_custom_version.SequenceEqual(new byte[] {(byte)54, (byte)53, (byte)28, (byte)188, (byte)167, (byte)55, (byte)21, (byte)146}));
                Debug.Assert(pack.flight_custom_version.SequenceEqual(new byte[] {(byte)42, (byte)111, (byte)224, (byte)37, (byte)103, (byte)66, (byte)198, (byte)182}));
                Debug.Assert(pack.os_custom_version.SequenceEqual(new byte[] {(byte)11, (byte)210, (byte)51, (byte)241, (byte)161, (byte)142, (byte)222, (byte)103}));
            };
            GroundControl.AUTOPILOT_VERSION p148 = CommunicationChannel.new_AUTOPILOT_VERSION();
            PH.setPack(p148);
            p148.product_id = (ushort)(ushort)11218;
            p148.flight_sw_version = (uint)1335468481U;
            p148.os_custom_version_SET(new byte[] {(byte)11, (byte)210, (byte)51, (byte)241, (byte)161, (byte)142, (byte)222, (byte)103}, 0) ;
            p148.vendor_id = (ushort)(ushort)50682;
            p148.os_sw_version = (uint)2206202373U;
            p148.board_version = (uint)3310532352U;
            p148.middleware_custom_version_SET(new byte[] {(byte)54, (byte)53, (byte)28, (byte)188, (byte)167, (byte)55, (byte)21, (byte)146}, 0) ;
            p148.middleware_sw_version = (uint)3893898485U;
            p148.uid = (ulong)2581642865699990701L;
            p148.flight_custom_version_SET(new byte[] {(byte)42, (byte)111, (byte)224, (byte)37, (byte)103, (byte)66, (byte)198, (byte)182}, 0) ;
            p148.capabilities = (MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                 MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT);
            p148.uid2_SET(new byte[] {(byte)138, (byte)135, (byte)10, (byte)88, (byte)70, (byte)148, (byte)124, (byte)137, (byte)98, (byte)39, (byte)205, (byte)220, (byte)16, (byte)139, (byte)57, (byte)250, (byte)117, (byte)27}, 0, PH) ;
            CommunicationChannel.instance.send(p148);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLANDING_TARGETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)7017973033869815806L);
                Debug.Assert(pack.y_TRY(ph) == (float) -2.9095042E38F);
                Debug.Assert(pack.size_y == (float) -2.3083197E38F);
                Debug.Assert(pack.z_TRY(ph) == (float) -1.6173943E38F);
                Debug.Assert(pack.position_valid_TRY(ph) == (byte)(byte)98);
                Debug.Assert(pack.q_TRY(ph).SequenceEqual(new float[] {-1.9144169E38F, -5.182604E37F, 2.087135E38F, -3.3180647E38F}));
                Debug.Assert(pack.x_TRY(ph) == (float) -1.4469894E38F);
                Debug.Assert(pack.target_num == (byte)(byte)250);
                Debug.Assert(pack.size_x == (float)2.8651374E38F);
                Debug.Assert(pack.frame == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
                Debug.Assert(pack.distance == (float)2.222169E38F);
                Debug.Assert(pack.angle_x == (float) -2.6390934E38F);
                Debug.Assert(pack.angle_y == (float) -3.4283547E36F);
                Debug.Assert(pack.type == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
            };
            GroundControl.LANDING_TARGET p149 = CommunicationChannel.new_LANDING_TARGET();
            PH.setPack(p149);
            p149.frame = MAV_FRAME.MAV_FRAME_GLOBAL_INT;
            p149.x_SET((float) -1.4469894E38F, PH) ;
            p149.position_valid_SET((byte)(byte)98, PH) ;
            p149.distance = (float)2.222169E38F;
            p149.size_y = (float) -2.3083197E38F;
            p149.time_usec = (ulong)7017973033869815806L;
            p149.z_SET((float) -1.6173943E38F, PH) ;
            p149.y_SET((float) -2.9095042E38F, PH) ;
            p149.type = LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER;
            p149.angle_x = (float) -2.6390934E38F;
            p149.size_x = (float)2.8651374E38F;
            p149.q_SET(new float[] {-1.9144169E38F, -5.182604E37F, 2.087135E38F, -3.3180647E38F}, 0, PH) ;
            p149.angle_y = (float) -3.4283547E36F;
            p149.target_num = (byte)(byte)250;
            CommunicationChannel.instance.send(p149);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAV_FILTER_BIASReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gyro_0 == (float)7.7440207E37F);
                Debug.Assert(pack.gyro_1 == (float) -2.4814918E38F);
                Debug.Assert(pack.accel_2 == (float) -2.3459383E38F);
                Debug.Assert(pack.usec == (ulong)506176826206164828L);
                Debug.Assert(pack.accel_0 == (float)1.0635025E38F);
                Debug.Assert(pack.gyro_2 == (float) -3.2981764E38F);
                Debug.Assert(pack.accel_1 == (float)1.5705015E38F);
            };
            GroundControl.NAV_FILTER_BIAS p220 = CommunicationChannel.new_NAV_FILTER_BIAS();
            PH.setPack(p220);
            p220.usec = (ulong)506176826206164828L;
            p220.accel_2 = (float) -2.3459383E38F;
            p220.accel_0 = (float)1.0635025E38F;
            p220.gyro_1 = (float) -2.4814918E38F;
            p220.gyro_2 = (float) -3.2981764E38F;
            p220.accel_1 = (float)1.5705015E38F;
            p220.gyro_0 = (float)7.7440207E37F;
            CommunicationChannel.instance.send(p220);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnRADIO_CALIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.rudder.SequenceEqual(new ushort[] {(ushort)43326, (ushort)25399, (ushort)58510}));
                Debug.Assert(pack.throttle.SequenceEqual(new ushort[] {(ushort)1422, (ushort)24981, (ushort)18224, (ushort)43889, (ushort)40088}));
                Debug.Assert(pack.elevator.SequenceEqual(new ushort[] {(ushort)7746, (ushort)53981, (ushort)50296}));
                Debug.Assert(pack.aileron.SequenceEqual(new ushort[] {(ushort)21157, (ushort)23271, (ushort)21320}));
                Debug.Assert(pack.pitch.SequenceEqual(new ushort[] {(ushort)54814, (ushort)40113, (ushort)34662, (ushort)40732, (ushort)63674}));
                Debug.Assert(pack.gyro.SequenceEqual(new ushort[] {(ushort)46884, (ushort)28524}));
            };
            GroundControl.RADIO_CALIBRATION p221 = CommunicationChannel.new_RADIO_CALIBRATION();
            PH.setPack(p221);
            p221.rudder_SET(new ushort[] {(ushort)43326, (ushort)25399, (ushort)58510}, 0) ;
            p221.elevator_SET(new ushort[] {(ushort)7746, (ushort)53981, (ushort)50296}, 0) ;
            p221.aileron_SET(new ushort[] {(ushort)21157, (ushort)23271, (ushort)21320}, 0) ;
            p221.gyro_SET(new ushort[] {(ushort)46884, (ushort)28524}, 0) ;
            p221.pitch_SET(new ushort[] {(ushort)54814, (ushort)40113, (ushort)34662, (ushort)40732, (ushort)63674}, 0) ;
            p221.throttle_SET(new ushort[] {(ushort)1422, (ushort)24981, (ushort)18224, (ushort)43889, (ushort)40088}, 0) ;
            CommunicationChannel.instance.send(p221);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUALBERTA_SYS_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode == (byte)(byte)156);
                Debug.Assert(pack.nav_mode == (byte)(byte)210);
                Debug.Assert(pack.pilot == (byte)(byte)101);
            };
            GroundControl.UALBERTA_SYS_STATUS p222 = CommunicationChannel.new_UALBERTA_SYS_STATUS();
            PH.setPack(p222);
            p222.nav_mode = (byte)(byte)210;
            p222.mode = (byte)(byte)156;
            p222.pilot = (byte)(byte)101;
            CommunicationChannel.instance.send(p222);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnESTIMATOR_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.pos_vert_accuracy == (float)1.7986761E38F);
                Debug.Assert(pack.pos_horiz_accuracy == (float)1.0395177E38F);
                Debug.Assert(pack.time_usec == (ulong)2695956819047877236L);
                Debug.Assert(pack.mag_ratio == (float) -5.027359E37F);
                Debug.Assert(pack.pos_vert_ratio == (float)2.898069E37F);
                Debug.Assert(pack.flags == (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                                            ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS));
                Debug.Assert(pack.hagl_ratio == (float)1.8378149E38F);
                Debug.Assert(pack.tas_ratio == (float) -1.385502E38F);
                Debug.Assert(pack.pos_horiz_ratio == (float) -2.364987E38F);
                Debug.Assert(pack.vel_ratio == (float)1.9889915E38F);
            };
            GroundControl.ESTIMATOR_STATUS p230 = CommunicationChannel.new_ESTIMATOR_STATUS();
            PH.setPack(p230);
            p230.pos_vert_accuracy = (float)1.7986761E38F;
            p230.flags = (ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH |
                          ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS);
            p230.tas_ratio = (float) -1.385502E38F;
            p230.hagl_ratio = (float)1.8378149E38F;
            p230.mag_ratio = (float) -5.027359E37F;
            p230.pos_horiz_accuracy = (float)1.0395177E38F;
            p230.pos_vert_ratio = (float)2.898069E37F;
            p230.vel_ratio = (float)1.9889915E38F;
            p230.time_usec = (ulong)2695956819047877236L;
            p230.pos_horiz_ratio = (float) -2.364987E38F;
            CommunicationChannel.instance.send(p230);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIND_COVReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)3524879521110262373L);
                Debug.Assert(pack.wind_alt == (float)2.9263388E38F);
                Debug.Assert(pack.wind_z == (float)1.3983885E38F);
                Debug.Assert(pack.var_horiz == (float)2.1440588E38F);
                Debug.Assert(pack.wind_x == (float)1.0055741E38F);
                Debug.Assert(pack.horiz_accuracy == (float) -1.9604206E37F);
                Debug.Assert(pack.var_vert == (float) -1.2883356E38F);
                Debug.Assert(pack.vert_accuracy == (float)9.965216E37F);
                Debug.Assert(pack.wind_y == (float) -3.189962E38F);
            };
            GroundControl.WIND_COV p231 = CommunicationChannel.new_WIND_COV();
            PH.setPack(p231);
            p231.vert_accuracy = (float)9.965216E37F;
            p231.wind_y = (float) -3.189962E38F;
            p231.wind_z = (float)1.3983885E38F;
            p231.var_horiz = (float)2.1440588E38F;
            p231.var_vert = (float) -1.2883356E38F;
            p231.time_usec = (ulong)3524879521110262373L;
            p231.wind_x = (float)1.0055741E38F;
            p231.wind_alt = (float)2.9263388E38F;
            p231.horiz_accuracy = (float) -1.9604206E37F;
            CommunicationChannel.instance.send(p231);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_INPUTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gps_id == (byte)(byte)191);
                Debug.Assert(pack.time_week_ms == (uint)3311830030U);
                Debug.Assert(pack.ve == (float)2.8822677E38F);
                Debug.Assert(pack.satellites_visible == (byte)(byte)94);
                Debug.Assert(pack.time_usec == (ulong)7594637732319667825L);
                Debug.Assert(pack.speed_accuracy == (float)2.866364E38F);
                Debug.Assert(pack.fix_type == (byte)(byte)169);
                Debug.Assert(pack.lon == (int) -1173062816);
                Debug.Assert(pack.time_week == (ushort)(ushort)33702);
                Debug.Assert(pack.ignore_flags == (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                                   GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT));
                Debug.Assert(pack.horiz_accuracy == (float) -3.5913898E36F);
                Debug.Assert(pack.alt == (float) -2.882015E38F);
                Debug.Assert(pack.vdop == (float)7.601752E37F);
                Debug.Assert(pack.vn == (float) -3.1311893E38F);
                Debug.Assert(pack.lat == (int) -646143319);
                Debug.Assert(pack.hdop == (float)1.939799E38F);
                Debug.Assert(pack.vert_accuracy == (float)9.342933E37F);
                Debug.Assert(pack.vd == (float) -2.9908317E38F);
            };
            GroundControl.GPS_INPUT p232 = CommunicationChannel.new_GPS_INPUT();
            PH.setPack(p232);
            p232.gps_id = (byte)(byte)191;
            p232.lat = (int) -646143319;
            p232.vdop = (float)7.601752E37F;
            p232.ve = (float)2.8822677E38F;
            p232.vn = (float) -3.1311893E38F;
            p232.vd = (float) -2.9908317E38F;
            p232.satellites_visible = (byte)(byte)94;
            p232.time_usec = (ulong)7594637732319667825L;
            p232.hdop = (float)1.939799E38F;
            p232.vert_accuracy = (float)9.342933E37F;
            p232.time_week_ms = (uint)3311830030U;
            p232.ignore_flags = (GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                                 GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT);
            p232.horiz_accuracy = (float) -3.5913898E36F;
            p232.lon = (int) -1173062816;
            p232.alt = (float) -2.882015E38F;
            p232.fix_type = (byte)(byte)169;
            p232.speed_accuracy = (float)2.866364E38F;
            p232.time_week = (ushort)(ushort)33702;
            CommunicationChannel.instance.send(p232);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnGPS_RTCM_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.flags == (byte)(byte)207);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)177, (byte)12, (byte)163, (byte)24, (byte)142, (byte)145, (byte)27, (byte)81, (byte)104, (byte)145, (byte)51, (byte)62, (byte)155, (byte)240, (byte)222, (byte)4, (byte)184, (byte)15, (byte)203, (byte)59, (byte)15, (byte)82, (byte)204, (byte)88, (byte)251, (byte)189, (byte)28, (byte)215, (byte)115, (byte)128, (byte)100, (byte)241, (byte)16, (byte)51, (byte)37, (byte)179, (byte)153, (byte)37, (byte)175, (byte)107, (byte)196, (byte)170, (byte)75, (byte)18, (byte)140, (byte)35, (byte)63, (byte)141, (byte)58, (byte)179, (byte)251, (byte)127, (byte)169, (byte)70, (byte)151, (byte)172, (byte)219, (byte)171, (byte)107, (byte)173, (byte)81, (byte)1, (byte)252, (byte)211, (byte)175, (byte)3, (byte)88, (byte)135, (byte)120, (byte)171, (byte)176, (byte)223, (byte)173, (byte)92, (byte)240, (byte)161, (byte)91, (byte)161, (byte)101, (byte)196, (byte)169, (byte)53, (byte)157, (byte)149, (byte)91, (byte)85, (byte)54, (byte)161, (byte)187, (byte)146, (byte)145, (byte)108, (byte)154, (byte)151, (byte)243, (byte)16, (byte)159, (byte)127, (byte)50, (byte)198, (byte)7, (byte)43, (byte)88, (byte)86, (byte)195, (byte)213, (byte)208, (byte)206, (byte)4, (byte)32, (byte)75, (byte)234, (byte)61, (byte)253, (byte)69, (byte)53, (byte)253, (byte)175, (byte)113, (byte)83, (byte)133, (byte)97, (byte)111, (byte)143, (byte)63, (byte)58, (byte)16, (byte)97, (byte)246, (byte)229, (byte)128, (byte)238, (byte)31, (byte)77, (byte)158, (byte)125, (byte)121, (byte)134, (byte)92, (byte)188, (byte)224, (byte)128, (byte)176, (byte)188, (byte)233, (byte)56, (byte)238, (byte)36, (byte)81, (byte)243, (byte)182, (byte)38, (byte)103, (byte)111, (byte)178, (byte)34, (byte)52, (byte)35, (byte)115, (byte)178, (byte)71, (byte)232, (byte)186, (byte)70, (byte)43, (byte)220, (byte)84, (byte)178, (byte)60, (byte)18, (byte)187, (byte)141, (byte)81, (byte)73, (byte)202, (byte)145, (byte)144, (byte)247, (byte)33, (byte)189}));
                Debug.Assert(pack.len == (byte)(byte)205);
            };
            GroundControl.GPS_RTCM_DATA p233 = CommunicationChannel.new_GPS_RTCM_DATA();
            PH.setPack(p233);
            p233.len = (byte)(byte)205;
            p233.flags = (byte)(byte)207;
            p233.data__SET(new byte[] {(byte)177, (byte)12, (byte)163, (byte)24, (byte)142, (byte)145, (byte)27, (byte)81, (byte)104, (byte)145, (byte)51, (byte)62, (byte)155, (byte)240, (byte)222, (byte)4, (byte)184, (byte)15, (byte)203, (byte)59, (byte)15, (byte)82, (byte)204, (byte)88, (byte)251, (byte)189, (byte)28, (byte)215, (byte)115, (byte)128, (byte)100, (byte)241, (byte)16, (byte)51, (byte)37, (byte)179, (byte)153, (byte)37, (byte)175, (byte)107, (byte)196, (byte)170, (byte)75, (byte)18, (byte)140, (byte)35, (byte)63, (byte)141, (byte)58, (byte)179, (byte)251, (byte)127, (byte)169, (byte)70, (byte)151, (byte)172, (byte)219, (byte)171, (byte)107, (byte)173, (byte)81, (byte)1, (byte)252, (byte)211, (byte)175, (byte)3, (byte)88, (byte)135, (byte)120, (byte)171, (byte)176, (byte)223, (byte)173, (byte)92, (byte)240, (byte)161, (byte)91, (byte)161, (byte)101, (byte)196, (byte)169, (byte)53, (byte)157, (byte)149, (byte)91, (byte)85, (byte)54, (byte)161, (byte)187, (byte)146, (byte)145, (byte)108, (byte)154, (byte)151, (byte)243, (byte)16, (byte)159, (byte)127, (byte)50, (byte)198, (byte)7, (byte)43, (byte)88, (byte)86, (byte)195, (byte)213, (byte)208, (byte)206, (byte)4, (byte)32, (byte)75, (byte)234, (byte)61, (byte)253, (byte)69, (byte)53, (byte)253, (byte)175, (byte)113, (byte)83, (byte)133, (byte)97, (byte)111, (byte)143, (byte)63, (byte)58, (byte)16, (byte)97, (byte)246, (byte)229, (byte)128, (byte)238, (byte)31, (byte)77, (byte)158, (byte)125, (byte)121, (byte)134, (byte)92, (byte)188, (byte)224, (byte)128, (byte)176, (byte)188, (byte)233, (byte)56, (byte)238, (byte)36, (byte)81, (byte)243, (byte)182, (byte)38, (byte)103, (byte)111, (byte)178, (byte)34, (byte)52, (byte)35, (byte)115, (byte)178, (byte)71, (byte)232, (byte)186, (byte)70, (byte)43, (byte)220, (byte)84, (byte)178, (byte)60, (byte)18, (byte)187, (byte)141, (byte)81, (byte)73, (byte)202, (byte)145, (byte)144, (byte)247, (byte)33, (byte)189}, 0) ;
            CommunicationChannel.instance.send(p233);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHIGH_LATENCYReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.gps_nsat == (byte)(byte)90);
                Debug.Assert(pack.airspeed_sp == (byte)(byte)3);
                Debug.Assert(pack.custom_mode == (uint)1940356277U);
                Debug.Assert(pack.altitude_amsl == (short)(short) -32411);
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
                Debug.Assert(pack.throttle == (sbyte)(sbyte)1);
                Debug.Assert(pack.wp_num == (byte)(byte)234);
                Debug.Assert(pack.roll == (short)(short)29469);
                Debug.Assert(pack.temperature_air == (sbyte)(sbyte) - 82);
                Debug.Assert(pack.climb_rate == (sbyte)(sbyte) - 82);
                Debug.Assert(pack.battery_remaining == (byte)(byte)249);
                Debug.Assert(pack.heading == (ushort)(ushort)6739);
                Debug.Assert(pack.altitude_sp == (short)(short) -7750);
                Debug.Assert(pack.airspeed == (byte)(byte)68);
                Debug.Assert(pack.base_mode == (MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                                                MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
                Debug.Assert(pack.latitude == (int)1158514416);
                Debug.Assert(pack.gps_fix_type == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
                Debug.Assert(pack.wp_distance == (ushort)(ushort)47844);
                Debug.Assert(pack.failsafe == (byte)(byte)79);
                Debug.Assert(pack.pitch == (short)(short)17396);
                Debug.Assert(pack.heading_sp == (short)(short)9255);
                Debug.Assert(pack.longitude == (int)1324467370);
                Debug.Assert(pack.groundspeed == (byte)(byte)145);
                Debug.Assert(pack.temperature == (sbyte)(sbyte) - 40);
            };
            GroundControl.HIGH_LATENCY p234 = CommunicationChannel.new_HIGH_LATENCY();
            PH.setPack(p234);
            p234.temperature = (sbyte)(sbyte) - 40;
            p234.failsafe = (byte)(byte)79;
            p234.pitch = (short)(short)17396;
            p234.gps_nsat = (byte)(byte)90;
            p234.battery_remaining = (byte)(byte)249;
            p234.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF;
            p234.airspeed_sp = (byte)(byte)3;
            p234.groundspeed = (byte)(byte)145;
            p234.temperature_air = (sbyte)(sbyte) - 82;
            p234.wp_num = (byte)(byte)234;
            p234.altitude_sp = (short)(short) -7750;
            p234.latitude = (int)1158514416;
            p234.base_mode = (MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_HIL_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                              MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
            p234.heading = (ushort)(ushort)6739;
            p234.airspeed = (byte)(byte)68;
            p234.heading_sp = (short)(short)9255;
            p234.wp_distance = (ushort)(ushort)47844;
            p234.climb_rate = (sbyte)(sbyte) - 82;
            p234.gps_fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT;
            p234.roll = (short)(short)29469;
            p234.longitude = (int)1324467370;
            p234.altitude_amsl = (short)(short) -32411;
            p234.custom_mode = (uint)1940356277U;
            p234.throttle = (sbyte)(sbyte)1;
            CommunicationChannel.instance.send(p234);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIBRATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.vibration_z == (float)1.8898571E38F);
                Debug.Assert(pack.clipping_2 == (uint)3959072862U);
                Debug.Assert(pack.clipping_0 == (uint)3014844210U);
                Debug.Assert(pack.vibration_y == (float)1.576336E38F);
                Debug.Assert(pack.vibration_x == (float)1.9530281E38F);
                Debug.Assert(pack.clipping_1 == (uint)149873050U);
                Debug.Assert(pack.time_usec == (ulong)3621161173228943864L);
            };
            GroundControl.VIBRATION p241 = CommunicationChannel.new_VIBRATION();
            PH.setPack(p241);
            p241.clipping_2 = (uint)3959072862U;
            p241.clipping_0 = (uint)3014844210U;
            p241.clipping_1 = (uint)149873050U;
            p241.vibration_x = (float)1.9530281E38F;
            p241.vibration_y = (float)1.576336E38F;
            p241.time_usec = (ulong)3621161173228943864L;
            p241.vibration_z = (float)1.8898571E38F;
            CommunicationChannel.instance.send(p241);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnHOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)7150006351629333555L);
                Debug.Assert(pack.q.SequenceEqual(new float[] {3.5747907E37F, -1.8659805E38F, 3.3703087E38F, -1.6773852E37F}));
                Debug.Assert(pack.longitude == (int)1195129364);
                Debug.Assert(pack.approach_x == (float) -1.2554379E38F);
                Debug.Assert(pack.approach_y == (float) -3.1198662E38F);
                Debug.Assert(pack.latitude == (int) -512766360);
                Debug.Assert(pack.approach_z == (float) -8.400102E37F);
                Debug.Assert(pack.y == (float)2.4070685E37F);
                Debug.Assert(pack.altitude == (int) -611841104);
                Debug.Assert(pack.z == (float) -3.2003845E37F);
                Debug.Assert(pack.x == (float) -1.69913E38F);
            };
            GroundControl.HOME_POSITION p242 = CommunicationChannel.new_HOME_POSITION();
            PH.setPack(p242);
            p242.longitude = (int)1195129364;
            p242.approach_y = (float) -3.1198662E38F;
            p242.x = (float) -1.69913E38F;
            p242.q_SET(new float[] {3.5747907E37F, -1.8659805E38F, 3.3703087E38F, -1.6773852E37F}, 0) ;
            p242.y = (float)2.4070685E37F;
            p242.time_usec_SET((ulong)7150006351629333555L, PH) ;
            p242.z = (float) -3.2003845E37F;
            p242.latitude = (int) -512766360;
            p242.approach_x = (float) -1.2554379E38F;
            p242.altitude = (int) -611841104;
            p242.approach_z = (float) -8.400102E37F;
            CommunicationChannel.instance.send(p242);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_HOME_POSITIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.approach_z == (float) -2.7774711E38F);
                Debug.Assert(pack.time_usec_TRY(ph) == (ulong)9098915736888927716L);
                Debug.Assert(pack.z == (float)2.6573165E38F);
                Debug.Assert(pack.target_system == (byte)(byte)115);
                Debug.Assert(pack.approach_y == (float)9.126745E37F);
                Debug.Assert(pack.longitude == (int)1286325193);
                Debug.Assert(pack.latitude == (int)1935093229);
                Debug.Assert(pack.x == (float)2.3340285E38F);
                Debug.Assert(pack.altitude == (int)1637834103);
                Debug.Assert(pack.q.SequenceEqual(new float[] {2.932268E37F, -1.5819526E38F, -3.3264807E38F, 6.560071E37F}));
                Debug.Assert(pack.y == (float)2.4817313E38F);
                Debug.Assert(pack.approach_x == (float)2.9165545E37F);
            };
            GroundControl.SET_HOME_POSITION p243 = CommunicationChannel.new_SET_HOME_POSITION();
            PH.setPack(p243);
            p243.approach_x = (float)2.9165545E37F;
            p243.y = (float)2.4817313E38F;
            p243.latitude = (int)1935093229;
            p243.altitude = (int)1637834103;
            p243.z = (float)2.6573165E38F;
            p243.approach_y = (float)9.126745E37F;
            p243.x = (float)2.3340285E38F;
            p243.approach_z = (float) -2.7774711E38F;
            p243.time_usec_SET((ulong)9098915736888927716L, PH) ;
            p243.q_SET(new float[] {2.932268E37F, -1.5819526E38F, -3.3264807E38F, 6.560071E37F}, 0) ;
            p243.target_system = (byte)(byte)115;
            p243.longitude = (int)1286325193;
            CommunicationChannel.instance.send(p243);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMESSAGE_INTERVALReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.message_id == (ushort)(ushort)17411);
                Debug.Assert(pack.interval_us == (int) -740824918);
            };
            GroundControl.MESSAGE_INTERVAL p244 = CommunicationChannel.new_MESSAGE_INTERVAL();
            PH.setPack(p244);
            p244.interval_us = (int) -740824918;
            p244.message_id = (ushort)(ushort)17411;
            CommunicationChannel.instance.send(p244);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnEXTENDED_SYS_STATEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.landed_state == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
                Debug.Assert(pack.vtol_state == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW);
            };
            GroundControl.EXTENDED_SYS_STATE p245 = CommunicationChannel.new_EXTENDED_SYS_STATE();
            PH.setPack(p245);
            p245.landed_state = MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED;
            p245.vtol_state = MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW;
            CommunicationChannel.instance.send(p245);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnADSB_VEHICLEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.ICAO_address == (uint)3145031410U);
                Debug.Assert(pack.flags == (ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK));
                Debug.Assert(pack.altitude_type == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
                Debug.Assert(pack.callsign_LEN(ph) == 4);
                Debug.Assert(pack.callsign_TRY(ph).Equals("Vfcv"));
                Debug.Assert(pack.emitter_type == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SMALL);
                Debug.Assert(pack.lat == (int) -1497925137);
                Debug.Assert(pack.tslc == (byte)(byte)101);
                Debug.Assert(pack.heading == (ushort)(ushort)36594);
                Debug.Assert(pack.hor_velocity == (ushort)(ushort)4842);
                Debug.Assert(pack.squawk == (ushort)(ushort)22590);
                Debug.Assert(pack.altitude == (int)2027824630);
                Debug.Assert(pack.lon == (int) -2117570083);
                Debug.Assert(pack.ver_velocity == (short)(short) -25695);
            };
            GroundControl.ADSB_VEHICLE p246 = CommunicationChannel.new_ADSB_VEHICLE();
            PH.setPack(p246);
            p246.lon = (int) -2117570083;
            p246.squawk = (ushort)(ushort)22590;
            p246.lat = (int) -1497925137;
            p246.emitter_type = ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SMALL;
            p246.callsign_SET("Vfcv", PH) ;
            p246.ver_velocity = (short)(short) -25695;
            p246.ICAO_address = (uint)3145031410U;
            p246.hor_velocity = (ushort)(ushort)4842;
            p246.tslc = (byte)(byte)101;
            p246.flags = (ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK);
            p246.altitude_type = ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC;
            p246.heading = (ushort)(ushort)36594;
            p246.altitude = (int)2027824630;
            CommunicationChannel.instance.send(p246);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCOLLISIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.altitude_minimum_delta == (float)1.8948991E38F);
                Debug.Assert(pack.threat_level == (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW));
                Debug.Assert(pack.id == (uint)2204886757U);
                Debug.Assert(pack.src_ == MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
                Debug.Assert(pack.time_to_minimum_delta == (float) -3.1853362E38F);
                Debug.Assert(pack.horizontal_minimum_delta == (float)2.1485643E38F);
                Debug.Assert(pack.action == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND);
            };
            GroundControl.COLLISION p247 = CommunicationChannel.new_COLLISION();
            PH.setPack(p247);
            p247.action = MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_ASCEND_OR_DESCEND;
            p247.horizontal_minimum_delta = (float)2.1485643E38F;
            p247.threat_level = (MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
            p247.time_to_minimum_delta = (float) -3.1853362E38F;
            p247.altitude_minimum_delta = (float)1.8948991E38F;
            p247.src_ = MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT;
            p247.id = (uint)2204886757U;
            CommunicationChannel.instance.send(p247);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnV2_EXTENSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.payload.SequenceEqual(new byte[] {(byte)13, (byte)89, (byte)91, (byte)64, (byte)94, (byte)217, (byte)159, (byte)222, (byte)161, (byte)54, (byte)77, (byte)132, (byte)168, (byte)226, (byte)128, (byte)137, (byte)254, (byte)42, (byte)152, (byte)119, (byte)164, (byte)15, (byte)15, (byte)148, (byte)38, (byte)238, (byte)66, (byte)52, (byte)25, (byte)69, (byte)94, (byte)253, (byte)149, (byte)205, (byte)76, (byte)72, (byte)53, (byte)175, (byte)98, (byte)106, (byte)76, (byte)150, (byte)200, (byte)139, (byte)195, (byte)170, (byte)220, (byte)154, (byte)86, (byte)29, (byte)240, (byte)39, (byte)90, (byte)32, (byte)97, (byte)51, (byte)45, (byte)50, (byte)114, (byte)108, (byte)255, (byte)156, (byte)85, (byte)95, (byte)23, (byte)155, (byte)232, (byte)15, (byte)138, (byte)44, (byte)95, (byte)45, (byte)101, (byte)125, (byte)163, (byte)188, (byte)178, (byte)221, (byte)213, (byte)175, (byte)11, (byte)114, (byte)20, (byte)30, (byte)114, (byte)63, (byte)9, (byte)187, (byte)36, (byte)190, (byte)116, (byte)54, (byte)121, (byte)146, (byte)199, (byte)91, (byte)53, (byte)244, (byte)5, (byte)53, (byte)158, (byte)205, (byte)255, (byte)19, (byte)17, (byte)140, (byte)254, (byte)65, (byte)72, (byte)244, (byte)178, (byte)236, (byte)38, (byte)97, (byte)71, (byte)86, (byte)58, (byte)246, (byte)70, (byte)154, (byte)11, (byte)37, (byte)219, (byte)99, (byte)134, (byte)138, (byte)74, (byte)125, (byte)45, (byte)198, (byte)63, (byte)30, (byte)118, (byte)203, (byte)197, (byte)136, (byte)3, (byte)76, (byte)108, (byte)55, (byte)37, (byte)68, (byte)3, (byte)216, (byte)110, (byte)219, (byte)187, (byte)125, (byte)181, (byte)22, (byte)177, (byte)217, (byte)241, (byte)197, (byte)141, (byte)208, (byte)6, (byte)251, (byte)168, (byte)77, (byte)164, (byte)95, (byte)161, (byte)77, (byte)207, (byte)152, (byte)22, (byte)163, (byte)49, (byte)167, (byte)89, (byte)120, (byte)142, (byte)118, (byte)115, (byte)81, (byte)10, (byte)223, (byte)160, (byte)207, (byte)62, (byte)193, (byte)174, (byte)184, (byte)68, (byte)122, (byte)208, (byte)68, (byte)143, (byte)119, (byte)1, (byte)108, (byte)108, (byte)227, (byte)31, (byte)161, (byte)189, (byte)102, (byte)30, (byte)7, (byte)55, (byte)90, (byte)214, (byte)199, (byte)248, (byte)171, (byte)171, (byte)15, (byte)164, (byte)2, (byte)9, (byte)239, (byte)149, (byte)67, (byte)35, (byte)0, (byte)193, (byte)108, (byte)186, (byte)199, (byte)133, (byte)60, (byte)18, (byte)93, (byte)152, (byte)76, (byte)138, (byte)177, (byte)143, (byte)64, (byte)44, (byte)107, (byte)254, (byte)35, (byte)255, (byte)179, (byte)126, (byte)252, (byte)5, (byte)167, (byte)206, (byte)42, (byte)65, (byte)173, (byte)50, (byte)160, (byte)224, (byte)79, (byte)27}));
                Debug.Assert(pack.message_type == (ushort)(ushort)36273);
                Debug.Assert(pack.target_system == (byte)(byte)221);
                Debug.Assert(pack.target_component == (byte)(byte)16);
                Debug.Assert(pack.target_network == (byte)(byte)116);
            };
            GroundControl.V2_EXTENSION p248 = CommunicationChannel.new_V2_EXTENSION();
            PH.setPack(p248);
            p248.target_network = (byte)(byte)116;
            p248.target_system = (byte)(byte)221;
            p248.target_component = (byte)(byte)16;
            p248.payload_SET(new byte[] {(byte)13, (byte)89, (byte)91, (byte)64, (byte)94, (byte)217, (byte)159, (byte)222, (byte)161, (byte)54, (byte)77, (byte)132, (byte)168, (byte)226, (byte)128, (byte)137, (byte)254, (byte)42, (byte)152, (byte)119, (byte)164, (byte)15, (byte)15, (byte)148, (byte)38, (byte)238, (byte)66, (byte)52, (byte)25, (byte)69, (byte)94, (byte)253, (byte)149, (byte)205, (byte)76, (byte)72, (byte)53, (byte)175, (byte)98, (byte)106, (byte)76, (byte)150, (byte)200, (byte)139, (byte)195, (byte)170, (byte)220, (byte)154, (byte)86, (byte)29, (byte)240, (byte)39, (byte)90, (byte)32, (byte)97, (byte)51, (byte)45, (byte)50, (byte)114, (byte)108, (byte)255, (byte)156, (byte)85, (byte)95, (byte)23, (byte)155, (byte)232, (byte)15, (byte)138, (byte)44, (byte)95, (byte)45, (byte)101, (byte)125, (byte)163, (byte)188, (byte)178, (byte)221, (byte)213, (byte)175, (byte)11, (byte)114, (byte)20, (byte)30, (byte)114, (byte)63, (byte)9, (byte)187, (byte)36, (byte)190, (byte)116, (byte)54, (byte)121, (byte)146, (byte)199, (byte)91, (byte)53, (byte)244, (byte)5, (byte)53, (byte)158, (byte)205, (byte)255, (byte)19, (byte)17, (byte)140, (byte)254, (byte)65, (byte)72, (byte)244, (byte)178, (byte)236, (byte)38, (byte)97, (byte)71, (byte)86, (byte)58, (byte)246, (byte)70, (byte)154, (byte)11, (byte)37, (byte)219, (byte)99, (byte)134, (byte)138, (byte)74, (byte)125, (byte)45, (byte)198, (byte)63, (byte)30, (byte)118, (byte)203, (byte)197, (byte)136, (byte)3, (byte)76, (byte)108, (byte)55, (byte)37, (byte)68, (byte)3, (byte)216, (byte)110, (byte)219, (byte)187, (byte)125, (byte)181, (byte)22, (byte)177, (byte)217, (byte)241, (byte)197, (byte)141, (byte)208, (byte)6, (byte)251, (byte)168, (byte)77, (byte)164, (byte)95, (byte)161, (byte)77, (byte)207, (byte)152, (byte)22, (byte)163, (byte)49, (byte)167, (byte)89, (byte)120, (byte)142, (byte)118, (byte)115, (byte)81, (byte)10, (byte)223, (byte)160, (byte)207, (byte)62, (byte)193, (byte)174, (byte)184, (byte)68, (byte)122, (byte)208, (byte)68, (byte)143, (byte)119, (byte)1, (byte)108, (byte)108, (byte)227, (byte)31, (byte)161, (byte)189, (byte)102, (byte)30, (byte)7, (byte)55, (byte)90, (byte)214, (byte)199, (byte)248, (byte)171, (byte)171, (byte)15, (byte)164, (byte)2, (byte)9, (byte)239, (byte)149, (byte)67, (byte)35, (byte)0, (byte)193, (byte)108, (byte)186, (byte)199, (byte)133, (byte)60, (byte)18, (byte)93, (byte)152, (byte)76, (byte)138, (byte)177, (byte)143, (byte)64, (byte)44, (byte)107, (byte)254, (byte)35, (byte)255, (byte)179, (byte)126, (byte)252, (byte)5, (byte)167, (byte)206, (byte)42, (byte)65, (byte)173, (byte)50, (byte)160, (byte)224, (byte)79, (byte)27}, 0) ;
            p248.message_type = (ushort)(ushort)36273;
            CommunicationChannel.instance.send(p248);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMEMORY_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.address == (ushort)(ushort)18668);
                Debug.Assert(pack.ver == (byte)(byte)237);
                Debug.Assert(pack.value.SequenceEqual(new sbyte[] {(sbyte) - 54, (sbyte)45, (sbyte) - 17, (sbyte) - 126, (sbyte) - 126, (sbyte)32, (sbyte) - 118, (sbyte) - 45, (sbyte) - 12, (sbyte)121, (sbyte)4, (sbyte)24, (sbyte) - 15, (sbyte) - 104, (sbyte)50, (sbyte) - 117, (sbyte)111, (sbyte) - 121, (sbyte)85, (sbyte) - 56, (sbyte)37, (sbyte)94, (sbyte)85, (sbyte)97, (sbyte) - 97, (sbyte) - 75, (sbyte) - 91, (sbyte) - 54, (sbyte) - 66, (sbyte) - 44, (sbyte)102, (sbyte)0}));
                Debug.Assert(pack.type == (byte)(byte)72);
            };
            GroundControl.MEMORY_VECT p249 = CommunicationChannel.new_MEMORY_VECT();
            PH.setPack(p249);
            p249.value_SET(new sbyte[] {(sbyte) - 54, (sbyte)45, (sbyte) - 17, (sbyte) - 126, (sbyte) - 126, (sbyte)32, (sbyte) - 118, (sbyte) - 45, (sbyte) - 12, (sbyte)121, (sbyte)4, (sbyte)24, (sbyte) - 15, (sbyte) - 104, (sbyte)50, (sbyte) - 117, (sbyte)111, (sbyte) - 121, (sbyte)85, (sbyte) - 56, (sbyte)37, (sbyte)94, (sbyte)85, (sbyte)97, (sbyte) - 97, (sbyte) - 75, (sbyte) - 91, (sbyte) - 54, (sbyte) - 66, (sbyte) - 44, (sbyte)102, (sbyte)0}, 0) ;
            p249.ver = (byte)(byte)237;
            p249.address = (ushort)(ushort)18668;
            p249.type = (byte)(byte)72;
            CommunicationChannel.instance.send(p249);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUG_VECTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.y == (float) -1.8395695E38F);
                Debug.Assert(pack.name_LEN(ph) == 1);
                Debug.Assert(pack.name_TRY(ph).Equals("s"));
                Debug.Assert(pack.time_usec == (ulong)6077770422254144849L);
                Debug.Assert(pack.z == (float) -1.6589952E38F);
                Debug.Assert(pack.x == (float) -1.0630811E38F);
            };
            GroundControl.DEBUG_VECT p250 = CommunicationChannel.new_DEBUG_VECT();
            PH.setPack(p250);
            p250.y = (float) -1.8395695E38F;
            p250.name_SET("s", PH) ;
            p250.time_usec = (ulong)6077770422254144849L;
            p250.x = (float) -1.0630811E38F;
            p250.z = (float) -1.6589952E38F;
            CommunicationChannel.instance.send(p250);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_FLOATReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)748182842U);
                Debug.Assert(pack.value == (float)1.2001163E38F);
                Debug.Assert(pack.name_LEN(ph) == 8);
                Debug.Assert(pack.name_TRY(ph).Equals("jdzhbfmo"));
            };
            GroundControl.NAMED_VALUE_FLOAT p251 = CommunicationChannel.new_NAMED_VALUE_FLOAT();
            PH.setPack(p251);
            p251.name_SET("jdzhbfmo", PH) ;
            p251.value = (float)1.2001163E38F;
            p251.time_boot_ms = (uint)748182842U;
            CommunicationChannel.instance.send(p251);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnNAMED_VALUE_INTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (int)1132764121);
                Debug.Assert(pack.time_boot_ms == (uint)2693367359U);
                Debug.Assert(pack.name_LEN(ph) == 10);
                Debug.Assert(pack.name_TRY(ph).Equals("DqGkohvxhb"));
            };
            GroundControl.NAMED_VALUE_INT p252 = CommunicationChannel.new_NAMED_VALUE_INT();
            PH.setPack(p252);
            p252.name_SET("DqGkohvxhb", PH) ;
            p252.value = (int)1132764121;
            p252.time_boot_ms = (uint)2693367359U;
            CommunicationChannel.instance.send(p252);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTATUSTEXTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.text_LEN(ph) == 15);
                Debug.Assert(pack.text_TRY(ph).Equals("bzqxljnzvzogtxk"));
                Debug.Assert(pack.severity == MAV_SEVERITY.MAV_SEVERITY_EMERGENCY);
            };
            GroundControl.STATUSTEXT p253 = CommunicationChannel.new_STATUSTEXT();
            PH.setPack(p253);
            p253.severity = MAV_SEVERITY.MAV_SEVERITY_EMERGENCY;
            p253.text_SET("bzqxljnzvzogtxk", PH) ;
            CommunicationChannel.instance.send(p253);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnDEBUGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.value == (float)2.4127015E38F);
                Debug.Assert(pack.time_boot_ms == (uint)1224366210U);
                Debug.Assert(pack.ind == (byte)(byte)172);
            };
            GroundControl.DEBUG p254 = CommunicationChannel.new_DEBUG();
            PH.setPack(p254);
            p254.ind = (byte)(byte)172;
            p254.value = (float)2.4127015E38F;
            p254.time_boot_ms = (uint)1224366210U;
            CommunicationChannel.instance.send(p254);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSETUP_SIGNINGReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.initial_timestamp == (ulong)1859081297579685694L);
                Debug.Assert(pack.secret_key.SequenceEqual(new byte[] {(byte)68, (byte)37, (byte)77, (byte)217, (byte)151, (byte)20, (byte)57, (byte)37, (byte)35, (byte)105, (byte)155, (byte)112, (byte)44, (byte)138, (byte)227, (byte)91, (byte)75, (byte)128, (byte)66, (byte)72, (byte)148, (byte)181, (byte)45, (byte)160, (byte)248, (byte)52, (byte)194, (byte)193, (byte)211, (byte)6, (byte)149, (byte)129}));
                Debug.Assert(pack.target_component == (byte)(byte)27);
                Debug.Assert(pack.target_system == (byte)(byte)205);
            };
            GroundControl.SETUP_SIGNING p256 = CommunicationChannel.new_SETUP_SIGNING();
            PH.setPack(p256);
            p256.secret_key_SET(new byte[] {(byte)68, (byte)37, (byte)77, (byte)217, (byte)151, (byte)20, (byte)57, (byte)37, (byte)35, (byte)105, (byte)155, (byte)112, (byte)44, (byte)138, (byte)227, (byte)91, (byte)75, (byte)128, (byte)66, (byte)72, (byte)148, (byte)181, (byte)45, (byte)160, (byte)248, (byte)52, (byte)194, (byte)193, (byte)211, (byte)6, (byte)149, (byte)129}, 0) ;
            p256.target_component = (byte)(byte)27;
            p256.target_system = (byte)(byte)205;
            p256.initial_timestamp = (ulong)1859081297579685694L;
            CommunicationChannel.instance.send(p256);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnBUTTON_CHANGEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.last_change_ms == (uint)2399336361U);
                Debug.Assert(pack.time_boot_ms == (uint)1364225339U);
                Debug.Assert(pack.state == (byte)(byte)141);
            };
            GroundControl.BUTTON_CHANGE p257 = CommunicationChannel.new_BUTTON_CHANGE();
            PH.setPack(p257);
            p257.state = (byte)(byte)141;
            p257.last_change_ms = (uint)2399336361U;
            p257.time_boot_ms = (uint)1364225339U;
            CommunicationChannel.instance.send(p257);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPLAY_TUNEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.tune_LEN(ph) == 27);
                Debug.Assert(pack.tune_TRY(ph).Equals("fvAVuzzauiBfMmztxhyqhQzrihQ"));
                Debug.Assert(pack.target_component == (byte)(byte)47);
                Debug.Assert(pack.target_system == (byte)(byte)80);
            };
            GroundControl.PLAY_TUNE p258 = CommunicationChannel.new_PLAY_TUNE();
            PH.setPack(p258);
            p258.target_system = (byte)(byte)80;
            p258.target_component = (byte)(byte)47;
            p258.tune_SET("fvAVuzzauiBfMmztxhyqhQzrihQ", PH) ;
            CommunicationChannel.instance.send(p258);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sensor_size_h == (float) -2.7397343E38F);
                Debug.Assert(pack.focal_length == (float)1.953725E38F);
                Debug.Assert(pack.model_name.SequenceEqual(new byte[] {(byte)238, (byte)97, (byte)81, (byte)154, (byte)107, (byte)57, (byte)69, (byte)176, (byte)172, (byte)111, (byte)124, (byte)19, (byte)112, (byte)125, (byte)14, (byte)152, (byte)189, (byte)9, (byte)25, (byte)194, (byte)233, (byte)108, (byte)112, (byte)80, (byte)140, (byte)78, (byte)209, (byte)116, (byte)181, (byte)19, (byte)79, (byte)122}));
                Debug.Assert(pack.cam_definition_version == (ushort)(ushort)49801);
                Debug.Assert(pack.time_boot_ms == (uint)1537577094U);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)32486);
                Debug.Assert(pack.cam_definition_uri_LEN(ph) == 38);
                Debug.Assert(pack.cam_definition_uri_TRY(ph).Equals("mfwzMhqhizuvnpqYbipSacqErkOeyekaodlnpx"));
                Debug.Assert(pack.resolution_h == (ushort)(ushort)16411);
                Debug.Assert(pack.vendor_name.SequenceEqual(new byte[] {(byte)23, (byte)171, (byte)24, (byte)71, (byte)147, (byte)234, (byte)230, (byte)94, (byte)210, (byte)181, (byte)238, (byte)112, (byte)204, (byte)42, (byte)235, (byte)69, (byte)157, (byte)21, (byte)104, (byte)117, (byte)8, (byte)81, (byte)24, (byte)47, (byte)82, (byte)84, (byte)246, (byte)237, (byte)222, (byte)244, (byte)209, (byte)97}));
                Debug.Assert(pack.lens_id == (byte)(byte)156);
                Debug.Assert(pack.flags == (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                                            CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE));
                Debug.Assert(pack.firmware_version == (uint)287540870U);
                Debug.Assert(pack.sensor_size_v == (float) -2.239419E38F);
            };
            GroundControl.CAMERA_INFORMATION p259 = CommunicationChannel.new_CAMERA_INFORMATION();
            PH.setPack(p259);
            p259.firmware_version = (uint)287540870U;
            p259.sensor_size_v = (float) -2.239419E38F;
            p259.time_boot_ms = (uint)1537577094U;
            p259.cam_definition_uri_SET("mfwzMhqhizuvnpqYbipSacqErkOeyekaodlnpx", PH) ;
            p259.lens_id = (byte)(byte)156;
            p259.flags = (CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                          CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE);
            p259.model_name_SET(new byte[] {(byte)238, (byte)97, (byte)81, (byte)154, (byte)107, (byte)57, (byte)69, (byte)176, (byte)172, (byte)111, (byte)124, (byte)19, (byte)112, (byte)125, (byte)14, (byte)152, (byte)189, (byte)9, (byte)25, (byte)194, (byte)233, (byte)108, (byte)112, (byte)80, (byte)140, (byte)78, (byte)209, (byte)116, (byte)181, (byte)19, (byte)79, (byte)122}, 0) ;
            p259.sensor_size_h = (float) -2.7397343E38F;
            p259.focal_length = (float)1.953725E38F;
            p259.resolution_v = (ushort)(ushort)32486;
            p259.resolution_h = (ushort)(ushort)16411;
            p259.vendor_name_SET(new byte[] {(byte)23, (byte)171, (byte)24, (byte)71, (byte)147, (byte)234, (byte)230, (byte)94, (byte)210, (byte)181, (byte)238, (byte)112, (byte)204, (byte)42, (byte)235, (byte)69, (byte)157, (byte)21, (byte)104, (byte)117, (byte)8, (byte)81, (byte)24, (byte)47, (byte)82, (byte)84, (byte)246, (byte)237, (byte)222, (byte)244, (byte)209, (byte)97}, 0) ;
            p259.cam_definition_version = (ushort)(ushort)49801;
            CommunicationChannel.instance.send(p259);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)2216557811U);
                Debug.Assert(pack.mode_id == (CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY |
                                              CAMERA_MODE.CAMERA_MODE_IMAGE));
            };
            GroundControl.CAMERA_SETTINGS p260 = CommunicationChannel.new_CAMERA_SETTINGS();
            PH.setPack(p260);
            p260.time_boot_ms = (uint)2216557811U;
            p260.mode_id = (CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY |
                            CAMERA_MODE.CAMERA_MODE_IMAGE);
            CommunicationChannel.instance.send(p260);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSTORAGE_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.storage_id == (byte)(byte)234);
                Debug.Assert(pack.total_capacity == (float)1.6591445E38F);
                Debug.Assert(pack.write_speed == (float) -4.0536497E37F);
                Debug.Assert(pack.time_boot_ms == (uint)1045320796U);
                Debug.Assert(pack.used_capacity == (float)2.6570965E38F);
                Debug.Assert(pack.storage_count == (byte)(byte)75);
                Debug.Assert(pack.available_capacity == (float)2.253776E37F);
                Debug.Assert(pack.status == (byte)(byte)100);
                Debug.Assert(pack.read_speed == (float) -2.8054605E38F);
            };
            GroundControl.STORAGE_INFORMATION p261 = CommunicationChannel.new_STORAGE_INFORMATION();
            PH.setPack(p261);
            p261.read_speed = (float) -2.8054605E38F;
            p261.storage_count = (byte)(byte)75;
            p261.total_capacity = (float)1.6591445E38F;
            p261.time_boot_ms = (uint)1045320796U;
            p261.storage_id = (byte)(byte)234;
            p261.write_speed = (float) -4.0536497E37F;
            p261.used_capacity = (float)2.6570965E38F;
            p261.status = (byte)(byte)100;
            p261.available_capacity = (float)2.253776E37F;
            CommunicationChannel.instance.send(p261);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_CAPTURE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.image_interval == (float) -1.6937835E38F);
                Debug.Assert(pack.video_status == (byte)(byte)200);
                Debug.Assert(pack.available_capacity == (float)2.2020282E38F);
                Debug.Assert(pack.recording_time_ms == (uint)2325865554U);
                Debug.Assert(pack.image_status == (byte)(byte)152);
                Debug.Assert(pack.time_boot_ms == (uint)668786632U);
            };
            GroundControl.CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.new_CAMERA_CAPTURE_STATUS();
            PH.setPack(p262);
            p262.video_status = (byte)(byte)200;
            p262.image_interval = (float) -1.6937835E38F;
            p262.available_capacity = (float)2.2020282E38F;
            p262.image_status = (byte)(byte)152;
            p262.recording_time_ms = (uint)2325865554U;
            p262.time_boot_ms = (uint)668786632U;
            CommunicationChannel.instance.send(p262);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnCAMERA_IMAGE_CAPTUREDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.file_url_LEN(ph) == 14);
                Debug.Assert(pack.file_url_TRY(ph).Equals("niarkrmywoqzrh"));
                Debug.Assert(pack.alt == (int) -1472939662);
                Debug.Assert(pack.lat == (int)1835783344);
                Debug.Assert(pack.capture_result == (sbyte)(sbyte)2);
                Debug.Assert(pack.lon == (int)1503631417);
                Debug.Assert(pack.relative_alt == (int)221132111);
                Debug.Assert(pack.time_utc == (ulong)3120675432844490238L);
                Debug.Assert(pack.camera_id == (byte)(byte)91);
                Debug.Assert(pack.time_boot_ms == (uint)3525715146U);
                Debug.Assert(pack.image_index == (int) -495382322);
                Debug.Assert(pack.q.SequenceEqual(new float[] {-1.1132951E38F, -3.1942128E37F, -1.675913E38F, 1.0108975E38F}));
            };
            GroundControl.CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.new_CAMERA_IMAGE_CAPTURED();
            PH.setPack(p263);
            p263.lon = (int)1503631417;
            p263.time_boot_ms = (uint)3525715146U;
            p263.camera_id = (byte)(byte)91;
            p263.relative_alt = (int)221132111;
            p263.q_SET(new float[] {-1.1132951E38F, -3.1942128E37F, -1.675913E38F, 1.0108975E38F}, 0) ;
            p263.lat = (int)1835783344;
            p263.image_index = (int) -495382322;
            p263.time_utc = (ulong)3120675432844490238L;
            p263.capture_result = (sbyte)(sbyte)2;
            p263.alt = (int) -1472939662;
            p263.file_url_SET("niarkrmywoqzrh", PH) ;
            CommunicationChannel.instance.send(p263);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnFLIGHT_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.takeoff_time_utc == (ulong)8618313906358048422L);
                Debug.Assert(pack.time_boot_ms == (uint)802811946U);
                Debug.Assert(pack.flight_uuid == (ulong)3344622437594397289L);
                Debug.Assert(pack.arming_time_utc == (ulong)9159401738365843868L);
            };
            GroundControl.FLIGHT_INFORMATION p264 = CommunicationChannel.new_FLIGHT_INFORMATION();
            PH.setPack(p264);
            p264.time_boot_ms = (uint)802811946U;
            p264.takeoff_time_utc = (ulong)8618313906358048422L;
            p264.flight_uuid = (ulong)3344622437594397289L;
            p264.arming_time_utc = (ulong)9159401738365843868L;
            CommunicationChannel.instance.send(p264);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnMOUNT_ORIENTATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_boot_ms == (uint)3263108003U);
                Debug.Assert(pack.yaw == (float)7.121702E37F);
                Debug.Assert(pack.pitch == (float)2.8174741E37F);
                Debug.Assert(pack.roll == (float)3.7171673E36F);
            };
            GroundControl.MOUNT_ORIENTATION p265 = CommunicationChannel.new_MOUNT_ORIENTATION();
            PH.setPack(p265);
            p265.pitch = (float)2.8174741E37F;
            p265.roll = (float)3.7171673E36F;
            p265.time_boot_ms = (uint)3263108003U;
            p265.yaw = (float)7.121702E37F;
            CommunicationChannel.instance.send(p265);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATAReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)205);
                Debug.Assert(pack.first_message_offset == (byte)(byte)119);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)101, (byte)145, (byte)93, (byte)104, (byte)86, (byte)161, (byte)217, (byte)181, (byte)101, (byte)216, (byte)69, (byte)40, (byte)118, (byte)248, (byte)22, (byte)246, (byte)248, (byte)255, (byte)36, (byte)37, (byte)233, (byte)172, (byte)232, (byte)249, (byte)230, (byte)159, (byte)6, (byte)135, (byte)223, (byte)124, (byte)179, (byte)214, (byte)11, (byte)74, (byte)134, (byte)35, (byte)3, (byte)158, (byte)38, (byte)216, (byte)201, (byte)146, (byte)102, (byte)42, (byte)149, (byte)67, (byte)255, (byte)114, (byte)170, (byte)15, (byte)82, (byte)168, (byte)19, (byte)17, (byte)69, (byte)221, (byte)1, (byte)122, (byte)75, (byte)231, (byte)249, (byte)108, (byte)78, (byte)224, (byte)232, (byte)186, (byte)227, (byte)149, (byte)21, (byte)99, (byte)75, (byte)141, (byte)5, (byte)250, (byte)116, (byte)10, (byte)202, (byte)219, (byte)51, (byte)70, (byte)68, (byte)85, (byte)168, (byte)135, (byte)172, (byte)219, (byte)161, (byte)148, (byte)92, (byte)238, (byte)184, (byte)187, (byte)176, (byte)47, (byte)164, (byte)239, (byte)6, (byte)136, (byte)23, (byte)238, (byte)83, (byte)111, (byte)92, (byte)160, (byte)148, (byte)253, (byte)25, (byte)14, (byte)187, (byte)158, (byte)117, (byte)168, (byte)41, (byte)12, (byte)19, (byte)89, (byte)44, (byte)211, (byte)133, (byte)32, (byte)215, (byte)25, (byte)232, (byte)106, (byte)197, (byte)218, (byte)163, (byte)85, (byte)182, (byte)62, (byte)205, (byte)200, (byte)122, (byte)171, (byte)230, (byte)20, (byte)81, (byte)103, (byte)127, (byte)94, (byte)13, (byte)212, (byte)26, (byte)130, (byte)88, (byte)71, (byte)79, (byte)35, (byte)60, (byte)67, (byte)197, (byte)74, (byte)101, (byte)42, (byte)16, (byte)135, (byte)250, (byte)32, (byte)169, (byte)14, (byte)59, (byte)83, (byte)217, (byte)97, (byte)58, (byte)26, (byte)237, (byte)9, (byte)190, (byte)14, (byte)120, (byte)131, (byte)40, (byte)146, (byte)215, (byte)15, (byte)186, (byte)217, (byte)197, (byte)240, (byte)89, (byte)116, (byte)108, (byte)141, (byte)3, (byte)107, (byte)89, (byte)161, (byte)103, (byte)165, (byte)235, (byte)108, (byte)198, (byte)198, (byte)219, (byte)227, (byte)53, (byte)134, (byte)30, (byte)73, (byte)164, (byte)180, (byte)162, (byte)6, (byte)76, (byte)134, (byte)107, (byte)73, (byte)131, (byte)58, (byte)58, (byte)2, (byte)99, (byte)36, (byte)136, (byte)155, (byte)127, (byte)233, (byte)9, (byte)182, (byte)62, (byte)185, (byte)224, (byte)172, (byte)142, (byte)109, (byte)121, (byte)8, (byte)25, (byte)47, (byte)64, (byte)242, (byte)183, (byte)194, (byte)102, (byte)165, (byte)165, (byte)187, (byte)18, (byte)69, (byte)171, (byte)102, (byte)165, (byte)12, (byte)123, (byte)61, (byte)235, (byte)222, (byte)86}));
                Debug.Assert(pack.length == (byte)(byte)136);
                Debug.Assert(pack.sequence == (ushort)(ushort)372);
                Debug.Assert(pack.target_system == (byte)(byte)64);
            };
            GroundControl.LOGGING_DATA p266 = CommunicationChannel.new_LOGGING_DATA();
            PH.setPack(p266);
            p266.target_component = (byte)(byte)205;
            p266.target_system = (byte)(byte)64;
            p266.sequence = (ushort)(ushort)372;
            p266.data__SET(new byte[] {(byte)101, (byte)145, (byte)93, (byte)104, (byte)86, (byte)161, (byte)217, (byte)181, (byte)101, (byte)216, (byte)69, (byte)40, (byte)118, (byte)248, (byte)22, (byte)246, (byte)248, (byte)255, (byte)36, (byte)37, (byte)233, (byte)172, (byte)232, (byte)249, (byte)230, (byte)159, (byte)6, (byte)135, (byte)223, (byte)124, (byte)179, (byte)214, (byte)11, (byte)74, (byte)134, (byte)35, (byte)3, (byte)158, (byte)38, (byte)216, (byte)201, (byte)146, (byte)102, (byte)42, (byte)149, (byte)67, (byte)255, (byte)114, (byte)170, (byte)15, (byte)82, (byte)168, (byte)19, (byte)17, (byte)69, (byte)221, (byte)1, (byte)122, (byte)75, (byte)231, (byte)249, (byte)108, (byte)78, (byte)224, (byte)232, (byte)186, (byte)227, (byte)149, (byte)21, (byte)99, (byte)75, (byte)141, (byte)5, (byte)250, (byte)116, (byte)10, (byte)202, (byte)219, (byte)51, (byte)70, (byte)68, (byte)85, (byte)168, (byte)135, (byte)172, (byte)219, (byte)161, (byte)148, (byte)92, (byte)238, (byte)184, (byte)187, (byte)176, (byte)47, (byte)164, (byte)239, (byte)6, (byte)136, (byte)23, (byte)238, (byte)83, (byte)111, (byte)92, (byte)160, (byte)148, (byte)253, (byte)25, (byte)14, (byte)187, (byte)158, (byte)117, (byte)168, (byte)41, (byte)12, (byte)19, (byte)89, (byte)44, (byte)211, (byte)133, (byte)32, (byte)215, (byte)25, (byte)232, (byte)106, (byte)197, (byte)218, (byte)163, (byte)85, (byte)182, (byte)62, (byte)205, (byte)200, (byte)122, (byte)171, (byte)230, (byte)20, (byte)81, (byte)103, (byte)127, (byte)94, (byte)13, (byte)212, (byte)26, (byte)130, (byte)88, (byte)71, (byte)79, (byte)35, (byte)60, (byte)67, (byte)197, (byte)74, (byte)101, (byte)42, (byte)16, (byte)135, (byte)250, (byte)32, (byte)169, (byte)14, (byte)59, (byte)83, (byte)217, (byte)97, (byte)58, (byte)26, (byte)237, (byte)9, (byte)190, (byte)14, (byte)120, (byte)131, (byte)40, (byte)146, (byte)215, (byte)15, (byte)186, (byte)217, (byte)197, (byte)240, (byte)89, (byte)116, (byte)108, (byte)141, (byte)3, (byte)107, (byte)89, (byte)161, (byte)103, (byte)165, (byte)235, (byte)108, (byte)198, (byte)198, (byte)219, (byte)227, (byte)53, (byte)134, (byte)30, (byte)73, (byte)164, (byte)180, (byte)162, (byte)6, (byte)76, (byte)134, (byte)107, (byte)73, (byte)131, (byte)58, (byte)58, (byte)2, (byte)99, (byte)36, (byte)136, (byte)155, (byte)127, (byte)233, (byte)9, (byte)182, (byte)62, (byte)185, (byte)224, (byte)172, (byte)142, (byte)109, (byte)121, (byte)8, (byte)25, (byte)47, (byte)64, (byte)242, (byte)183, (byte)194, (byte)102, (byte)165, (byte)165, (byte)187, (byte)18, (byte)69, (byte)171, (byte)102, (byte)165, (byte)12, (byte)123, (byte)61, (byte)235, (byte)222, (byte)86}, 0) ;
            p266.first_message_offset = (byte)(byte)119;
            p266.length = (byte)(byte)136;
            CommunicationChannel.instance.send(p266);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_DATA_ACKEDReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)68);
                Debug.Assert(pack.first_message_offset == (byte)(byte)164);
                Debug.Assert(pack.target_component == (byte)(byte)183);
                Debug.Assert(pack.length == (byte)(byte)42);
                Debug.Assert(pack.sequence == (ushort)(ushort)29170);
                Debug.Assert(pack.data_.SequenceEqual(new byte[] {(byte)234, (byte)83, (byte)191, (byte)27, (byte)138, (byte)223, (byte)250, (byte)241, (byte)248, (byte)178, (byte)217, (byte)211, (byte)0, (byte)41, (byte)167, (byte)134, (byte)184, (byte)72, (byte)98, (byte)55, (byte)132, (byte)122, (byte)238, (byte)12, (byte)74, (byte)27, (byte)30, (byte)55, (byte)62, (byte)229, (byte)124, (byte)236, (byte)94, (byte)46, (byte)78, (byte)143, (byte)199, (byte)49, (byte)206, (byte)159, (byte)41, (byte)183, (byte)15, (byte)18, (byte)243, (byte)57, (byte)69, (byte)208, (byte)41, (byte)204, (byte)54, (byte)137, (byte)132, (byte)3, (byte)168, (byte)206, (byte)236, (byte)185, (byte)220, (byte)112, (byte)194, (byte)49, (byte)190, (byte)243, (byte)225, (byte)140, (byte)169, (byte)141, (byte)238, (byte)10, (byte)134, (byte)188, (byte)32, (byte)76, (byte)254, (byte)31, (byte)28, (byte)215, (byte)202, (byte)14, (byte)161, (byte)138, (byte)92, (byte)207, (byte)108, (byte)39, (byte)38, (byte)140, (byte)45, (byte)90, (byte)35, (byte)110, (byte)239, (byte)88, (byte)108, (byte)119, (byte)211, (byte)106, (byte)89, (byte)143, (byte)210, (byte)119, (byte)223, (byte)114, (byte)124, (byte)174, (byte)89, (byte)234, (byte)112, (byte)14, (byte)11, (byte)126, (byte)172, (byte)178, (byte)31, (byte)240, (byte)54, (byte)224, (byte)225, (byte)87, (byte)198, (byte)4, (byte)157, (byte)48, (byte)136, (byte)150, (byte)21, (byte)219, (byte)107, (byte)136, (byte)88, (byte)17, (byte)59, (byte)63, (byte)151, (byte)214, (byte)31, (byte)222, (byte)63, (byte)114, (byte)184, (byte)24, (byte)30, (byte)78, (byte)236, (byte)176, (byte)115, (byte)142, (byte)246, (byte)55, (byte)192, (byte)76, (byte)172, (byte)251, (byte)216, (byte)218, (byte)161, (byte)27, (byte)235, (byte)103, (byte)136, (byte)207, (byte)52, (byte)250, (byte)249, (byte)108, (byte)71, (byte)70, (byte)78, (byte)153, (byte)254, (byte)190, (byte)207, (byte)116, (byte)49, (byte)189, (byte)172, (byte)6, (byte)15, (byte)197, (byte)75, (byte)186, (byte)138, (byte)12, (byte)49, (byte)205, (byte)253, (byte)216, (byte)117, (byte)247, (byte)195, (byte)183, (byte)248, (byte)88, (byte)66, (byte)17, (byte)171, (byte)214, (byte)184, (byte)89, (byte)85, (byte)197, (byte)248, (byte)247, (byte)128, (byte)79, (byte)40, (byte)231, (byte)95, (byte)57, (byte)87, (byte)37, (byte)39, (byte)143, (byte)134, (byte)143, (byte)36, (byte)128, (byte)13, (byte)141, (byte)214, (byte)65, (byte)237, (byte)100, (byte)64, (byte)58, (byte)148, (byte)93, (byte)206, (byte)242, (byte)235, (byte)14, (byte)12, (byte)167, (byte)84, (byte)122, (byte)231, (byte)46, (byte)206, (byte)49, (byte)90, (byte)222, (byte)136, (byte)109, (byte)60, (byte)66, (byte)22, (byte)71, (byte)97}));
            };
            GroundControl.LOGGING_DATA_ACKED p267 = CommunicationChannel.new_LOGGING_DATA_ACKED();
            PH.setPack(p267);
            p267.length = (byte)(byte)42;
            p267.data__SET(new byte[] {(byte)234, (byte)83, (byte)191, (byte)27, (byte)138, (byte)223, (byte)250, (byte)241, (byte)248, (byte)178, (byte)217, (byte)211, (byte)0, (byte)41, (byte)167, (byte)134, (byte)184, (byte)72, (byte)98, (byte)55, (byte)132, (byte)122, (byte)238, (byte)12, (byte)74, (byte)27, (byte)30, (byte)55, (byte)62, (byte)229, (byte)124, (byte)236, (byte)94, (byte)46, (byte)78, (byte)143, (byte)199, (byte)49, (byte)206, (byte)159, (byte)41, (byte)183, (byte)15, (byte)18, (byte)243, (byte)57, (byte)69, (byte)208, (byte)41, (byte)204, (byte)54, (byte)137, (byte)132, (byte)3, (byte)168, (byte)206, (byte)236, (byte)185, (byte)220, (byte)112, (byte)194, (byte)49, (byte)190, (byte)243, (byte)225, (byte)140, (byte)169, (byte)141, (byte)238, (byte)10, (byte)134, (byte)188, (byte)32, (byte)76, (byte)254, (byte)31, (byte)28, (byte)215, (byte)202, (byte)14, (byte)161, (byte)138, (byte)92, (byte)207, (byte)108, (byte)39, (byte)38, (byte)140, (byte)45, (byte)90, (byte)35, (byte)110, (byte)239, (byte)88, (byte)108, (byte)119, (byte)211, (byte)106, (byte)89, (byte)143, (byte)210, (byte)119, (byte)223, (byte)114, (byte)124, (byte)174, (byte)89, (byte)234, (byte)112, (byte)14, (byte)11, (byte)126, (byte)172, (byte)178, (byte)31, (byte)240, (byte)54, (byte)224, (byte)225, (byte)87, (byte)198, (byte)4, (byte)157, (byte)48, (byte)136, (byte)150, (byte)21, (byte)219, (byte)107, (byte)136, (byte)88, (byte)17, (byte)59, (byte)63, (byte)151, (byte)214, (byte)31, (byte)222, (byte)63, (byte)114, (byte)184, (byte)24, (byte)30, (byte)78, (byte)236, (byte)176, (byte)115, (byte)142, (byte)246, (byte)55, (byte)192, (byte)76, (byte)172, (byte)251, (byte)216, (byte)218, (byte)161, (byte)27, (byte)235, (byte)103, (byte)136, (byte)207, (byte)52, (byte)250, (byte)249, (byte)108, (byte)71, (byte)70, (byte)78, (byte)153, (byte)254, (byte)190, (byte)207, (byte)116, (byte)49, (byte)189, (byte)172, (byte)6, (byte)15, (byte)197, (byte)75, (byte)186, (byte)138, (byte)12, (byte)49, (byte)205, (byte)253, (byte)216, (byte)117, (byte)247, (byte)195, (byte)183, (byte)248, (byte)88, (byte)66, (byte)17, (byte)171, (byte)214, (byte)184, (byte)89, (byte)85, (byte)197, (byte)248, (byte)247, (byte)128, (byte)79, (byte)40, (byte)231, (byte)95, (byte)57, (byte)87, (byte)37, (byte)39, (byte)143, (byte)134, (byte)143, (byte)36, (byte)128, (byte)13, (byte)141, (byte)214, (byte)65, (byte)237, (byte)100, (byte)64, (byte)58, (byte)148, (byte)93, (byte)206, (byte)242, (byte)235, (byte)14, (byte)12, (byte)167, (byte)84, (byte)122, (byte)231, (byte)46, (byte)206, (byte)49, (byte)90, (byte)222, (byte)136, (byte)109, (byte)60, (byte)66, (byte)22, (byte)71, (byte)97}, 0) ;
            p267.sequence = (ushort)(ushort)29170;
            p267.first_message_offset = (byte)(byte)164;
            p267.target_system = (byte)(byte)68;
            p267.target_component = (byte)(byte)183;
            CommunicationChannel.instance.send(p267);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnLOGGING_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_system == (byte)(byte)150);
                Debug.Assert(pack.sequence == (ushort)(ushort)56513);
                Debug.Assert(pack.target_component == (byte)(byte)86);
            };
            GroundControl.LOGGING_ACK p268 = CommunicationChannel.new_LOGGING_ACK();
            PH.setPack(p268);
            p268.target_component = (byte)(byte)86;
            p268.sequence = (ushort)(ushort)56513;
            p268.target_system = (byte)(byte)150;
            CommunicationChannel.instance.send(p268);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnVIDEO_STREAM_INFORMATIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.framerate == (float)9.983274E37F);
                Debug.Assert(pack.camera_id == (byte)(byte)230);
                Debug.Assert(pack.uri_LEN(ph) == 188);
                Debug.Assert(pack.uri_TRY(ph).Equals("osafwAmGewnogriyswuuypgqtgdbqWclunrsGimbsfDJqoprzohdndhBifwhcmyxNpxfcmsxehpegwzlzphbquReeszddQkFxucmimlzfcHqhzmlsudkfjmohebreguzykavsnmpVkjreLhkkNDgebzfrtpaozhikqhlsecrzkjgagxcfzhzuyoJnvtq"));
                Debug.Assert(pack.rotation == (ushort)(ushort)60296);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)11907);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)22395);
                Debug.Assert(pack.bitrate == (uint)571021563U);
                Debug.Assert(pack.status == (byte)(byte)26);
            };
            GroundControl.VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.new_VIDEO_STREAM_INFORMATION();
            PH.setPack(p269);
            p269.rotation = (ushort)(ushort)60296;
            p269.bitrate = (uint)571021563U;
            p269.status = (byte)(byte)26;
            p269.uri_SET("osafwAmGewnogriyswuuypgqtgdbqWclunrsGimbsfDJqoprzohdndhBifwhcmyxNpxfcmsxehpegwzlzphbquReeszddQkFxucmimlzfcHqhzmlsudkfjmohebreguzykavsnmpVkjreLhkkNDgebzfrtpaozhikqhlsecrzkjgagxcfzhzuyoJnvtq", PH) ;
            p269.camera_id = (byte)(byte)230;
            p269.framerate = (float)9.983274E37F;
            p269.resolution_v = (ushort)(ushort)11907;
            p269.resolution_h = (ushort)(ushort)22395;
            CommunicationChannel.instance.send(p269);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnSET_VIDEO_STREAM_SETTINGSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.bitrate == (uint)2708173289U);
                Debug.Assert(pack.rotation == (ushort)(ushort)14470);
                Debug.Assert(pack.framerate == (float)9.714765E37F);
                Debug.Assert(pack.uri_LEN(ph) == 75);
                Debug.Assert(pack.uri_TRY(ph).Equals("mmgjbbcyfiebugJdtkpHtsadTlcmnblstwnjcihbakhjafHdvigkrvediCmqVIytnpgsbSgjdIs"));
                Debug.Assert(pack.target_component == (byte)(byte)236);
                Debug.Assert(pack.target_system == (byte)(byte)85);
                Debug.Assert(pack.camera_id == (byte)(byte)9);
                Debug.Assert(pack.resolution_v == (ushort)(ushort)24101);
                Debug.Assert(pack.resolution_h == (ushort)(ushort)4657);
            };
            GroundControl.SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.new_SET_VIDEO_STREAM_SETTINGS();
            PH.setPack(p270);
            p270.rotation = (ushort)(ushort)14470;
            p270.uri_SET("mmgjbbcyfiebugJdtkpHtsadTlcmnblstwnjcihbakhjafHdvigkrvediCmqVIytnpgsbSgjdIs", PH) ;
            p270.resolution_v = (ushort)(ushort)24101;
            p270.framerate = (float)9.714765E37F;
            p270.target_component = (byte)(byte)236;
            p270.resolution_h = (ushort)(ushort)4657;
            p270.target_system = (byte)(byte)85;
            p270.camera_id = (byte)(byte)9;
            p270.bitrate = (uint)2708173289U;
            CommunicationChannel.instance.send(p270);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnWIFI_CONFIG_APReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.password_LEN(ph) == 63);
                Debug.Assert(pack.password_TRY(ph).Equals("fyvmonmxwxrhYzsgxtnvltgudrjJuczVmaLzgylonjzweahqaidhwiwlJxeigfw"));
                Debug.Assert(pack.ssid_LEN(ph) == 10);
                Debug.Assert(pack.ssid_TRY(ph).Equals("ydlqrqnrmb"));
            };
            GroundControl.WIFI_CONFIG_AP p299 = CommunicationChannel.new_WIFI_CONFIG_AP();
            PH.setPack(p299);
            p299.ssid_SET("ydlqrqnrmb", PH) ;
            p299.password_SET("fyvmonmxwxrhYzsgxtnvltgudrjJuczVmaLzgylonjzweahqaidhwiwlJxeigfw", PH) ;
            CommunicationChannel.instance.send(p299);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPROTOCOL_VERSIONReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.max_version == (ushort)(ushort)53121);
                Debug.Assert(pack.version == (ushort)(ushort)37508);
                Debug.Assert(pack.min_version == (ushort)(ushort)14042);
                Debug.Assert(pack.spec_version_hash.SequenceEqual(new byte[] {(byte)38, (byte)249, (byte)124, (byte)74, (byte)199, (byte)244, (byte)22, (byte)37}));
                Debug.Assert(pack.library_version_hash.SequenceEqual(new byte[] {(byte)243, (byte)207, (byte)32, (byte)189, (byte)24, (byte)192, (byte)241, (byte)109}));
            };
            GroundControl.PROTOCOL_VERSION p300 = CommunicationChannel.new_PROTOCOL_VERSION();
            PH.setPack(p300);
            p300.max_version = (ushort)(ushort)53121;
            p300.library_version_hash_SET(new byte[] {(byte)243, (byte)207, (byte)32, (byte)189, (byte)24, (byte)192, (byte)241, (byte)109}, 0) ;
            p300.spec_version_hash_SET(new byte[] {(byte)38, (byte)249, (byte)124, (byte)74, (byte)199, (byte)244, (byte)22, (byte)37}, 0) ;
            p300.version = (ushort)(ushort)37508;
            p300.min_version = (ushort)(ushort)14042;
            CommunicationChannel.instance.send(p300);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_STATUSReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.mode == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL);
                Debug.Assert(pack.sub_mode == (byte)(byte)90);
                Debug.Assert(pack.uptime_sec == (uint)1434497287U);
                Debug.Assert(pack.time_usec == (ulong)1146101080649468980L);
                Debug.Assert(pack.health == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
                Debug.Assert(pack.vendor_specific_status_code == (ushort)(ushort)10394);
            };
            GroundControl.UAVCAN_NODE_STATUS p310 = CommunicationChannel.new_UAVCAN_NODE_STATUS();
            PH.setPack(p310);
            p310.uptime_sec = (uint)1434497287U;
            p310.mode = UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL;
            p310.sub_mode = (byte)(byte)90;
            p310.time_usec = (ulong)1146101080649468980L;
            p310.health = UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR;
            p310.vendor_specific_status_code = (ushort)(ushort)10394;
            CommunicationChannel.instance.send(p310);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnUAVCAN_NODE_INFOReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.sw_version_major == (byte)(byte)75);
                Debug.Assert(pack.name_LEN(ph) == 22);
                Debug.Assert(pack.name_TRY(ph).Equals("xjcfoocifdbakcucaavGat"));
                Debug.Assert(pack.uptime_sec == (uint)3302449844U);
                Debug.Assert(pack.sw_version_minor == (byte)(byte)134);
                Debug.Assert(pack.time_usec == (ulong)9048177567040891852L);
                Debug.Assert(pack.sw_vcs_commit == (uint)3474779882U);
                Debug.Assert(pack.hw_version_minor == (byte)(byte)122);
                Debug.Assert(pack.hw_version_major == (byte)(byte)144);
                Debug.Assert(pack.hw_unique_id.SequenceEqual(new byte[] {(byte)156, (byte)199, (byte)27, (byte)245, (byte)202, (byte)176, (byte)57, (byte)47, (byte)181, (byte)153, (byte)134, (byte)63, (byte)193, (byte)28, (byte)178, (byte)118}));
            };
            GroundControl.UAVCAN_NODE_INFO p311 = CommunicationChannel.new_UAVCAN_NODE_INFO();
            PH.setPack(p311);
            p311.sw_version_minor = (byte)(byte)134;
            p311.sw_vcs_commit = (uint)3474779882U;
            p311.name_SET("xjcfoocifdbakcucaavGat", PH) ;
            p311.hw_version_major = (byte)(byte)144;
            p311.hw_version_minor = (byte)(byte)122;
            p311.uptime_sec = (uint)3302449844U;
            p311.sw_version_major = (byte)(byte)75;
            p311.time_usec = (ulong)9048177567040891852L;
            p311.hw_unique_id_SET(new byte[] {(byte)156, (byte)199, (byte)27, (byte)245, (byte)202, (byte)176, (byte)57, (byte)47, (byte)181, (byte)153, (byte)134, (byte)63, (byte)193, (byte)28, (byte)178, (byte)118}, 0) ;
            CommunicationChannel.instance.send(p311);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_READReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_index == (short)(short)28471);
                Debug.Assert(pack.param_id_LEN(ph) == 13);
                Debug.Assert(pack.param_id_TRY(ph).Equals("mommmjzdizqar"));
                Debug.Assert(pack.target_system == (byte)(byte)250);
                Debug.Assert(pack.target_component == (byte)(byte)100);
            };
            GroundControl.PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.new_PARAM_EXT_REQUEST_READ();
            PH.setPack(p320);
            p320.target_component = (byte)(byte)100;
            p320.param_id_SET("mommmjzdizqar", PH) ;
            p320.target_system = (byte)(byte)250;
            p320.param_index = (short)(short)28471;
            CommunicationChannel.instance.send(p320);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_REQUEST_LISTReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)29);
                Debug.Assert(pack.target_system == (byte)(byte)180);
            };
            GroundControl.PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.new_PARAM_EXT_REQUEST_LIST();
            PH.setPack(p321);
            p321.target_component = (byte)(byte)29;
            p321.target_system = (byte)(byte)180;
            CommunicationChannel.instance.send(p321);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_VALUEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
                Debug.Assert(pack.param_index == (ushort)(ushort)23511);
                Debug.Assert(pack.param_id_LEN(ph) == 4);
                Debug.Assert(pack.param_id_TRY(ph).Equals("nfht"));
                Debug.Assert(pack.param_value_LEN(ph) == 112);
                Debug.Assert(pack.param_value_TRY(ph).Equals("IyPFcwFbammxnrtghbmsfqiLoivxfjktvrttpzoghwsxcvrbnhgZflzfxjrqeqjuisnhpuhrdHtrejazhXxnzvdnstukxdcvuxoEqlrqxqbuwoXp"));
                Debug.Assert(pack.param_count == (ushort)(ushort)45370);
            };
            GroundControl.PARAM_EXT_VALUE p322 = CommunicationChannel.new_PARAM_EXT_VALUE();
            PH.setPack(p322);
            p322.param_value_SET("IyPFcwFbammxnrtghbmsfqiLoivxfjktvrttpzoghwsxcvrbnhgZflzfxjrqeqjuisnhpuhrdHtrejazhXxnzvdnstukxdcvuxoEqlrqxqbuwoXp", PH) ;
            p322.param_index = (ushort)(ushort)23511;
            p322.param_count = (ushort)(ushort)45370;
            p322.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM;
            p322.param_id_SET("nfht", PH) ;
            CommunicationChannel.instance.send(p322);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_SETReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.target_component == (byte)(byte)71);
                Debug.Assert(pack.param_id_LEN(ph) == 7);
                Debug.Assert(pack.param_id_TRY(ph).Equals("uRnhumh"));
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
                Debug.Assert(pack.target_system == (byte)(byte)144);
                Debug.Assert(pack.param_value_LEN(ph) == 24);
                Debug.Assert(pack.param_value_TRY(ph).Equals("cviqwthdsoiOdvwbnjbbvvqp"));
            };
            GroundControl.PARAM_EXT_SET p323 = CommunicationChannel.new_PARAM_EXT_SET();
            PH.setPack(p323);
            p323.param_value_SET("cviqwthdsoiOdvwbnjbbvvqp", PH) ;
            p323.param_id_SET("uRnhumh", PH) ;
            p323.target_system = (byte)(byte)144;
            p323.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32;
            p323.target_component = (byte)(byte)71;
            CommunicationChannel.instance.send(p323);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnPARAM_EXT_ACKReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.param_type == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
                Debug.Assert(pack.param_value_LEN(ph) == 117);
                Debug.Assert(pack.param_value_TRY(ph).Equals("zoyrtocgfzpecrhxckhdcmmwdiadgxsqepbbnkkgzKfUidkEgavfgkbbursudwdGidjswhmlcqifNvstccaubtKfzvonhdlfpbpkzvbosvflgaiblqvrv"));
                Debug.Assert(pack.param_id_LEN(ph) == 7);
                Debug.Assert(pack.param_id_TRY(ph).Equals("Zoekbiq"));
                Debug.Assert(pack.param_result == PARAM_ACK.PARAM_ACK_IN_PROGRESS);
            };
            GroundControl.PARAM_EXT_ACK p324 = CommunicationChannel.new_PARAM_EXT_ACK();
            PH.setPack(p324);
            p324.param_id_SET("Zoekbiq", PH) ;
            p324.param_value_SET("zoyrtocgfzpecrhxckhdcmmwdiadgxsqepbbnkkgzKfUidkEgavfgkbbursudwdGidjswhmlcqifNvstccaubtKfzvonhdlfpbpkzvbosvflgaiblqvrv", PH) ;
            p324.param_result = PARAM_ACK.PARAM_ACK_IN_PROGRESS;
            p324.param_type = MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32;
            CommunicationChannel.instance.send(p324);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
            ADV_TEST_CH.OnOBSTACLE_DISTANCEReceive += (src, ph, pack) =>
            {
                Debug.Assert(pack.time_usec == (ulong)4468446342304799868L);
                Debug.Assert(pack.increment == (byte)(byte)79);
                Debug.Assert(pack.sensor_type == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
                Debug.Assert(pack.min_distance == (ushort)(ushort)48908);
                Debug.Assert(pack.distances.SequenceEqual(new ushort[] {(ushort)10174, (ushort)39414, (ushort)42321, (ushort)26021, (ushort)35148, (ushort)4604, (ushort)59577, (ushort)59455, (ushort)1860, (ushort)48226, (ushort)56229, (ushort)12836, (ushort)58698, (ushort)9293, (ushort)30365, (ushort)40910, (ushort)12652, (ushort)26128, (ushort)31559, (ushort)39590, (ushort)59776, (ushort)48650, (ushort)51365, (ushort)19184, (ushort)64460, (ushort)43655, (ushort)54351, (ushort)32071, (ushort)52558, (ushort)2776, (ushort)3516, (ushort)63828, (ushort)62822, (ushort)20975, (ushort)50810, (ushort)48041, (ushort)42900, (ushort)39175, (ushort)43241, (ushort)45080, (ushort)52384, (ushort)18977, (ushort)36380, (ushort)38205, (ushort)5247, (ushort)23725, (ushort)30639, (ushort)57231, (ushort)59337, (ushort)30154, (ushort)4187, (ushort)20415, (ushort)64244, (ushort)62655, (ushort)7835, (ushort)28310, (ushort)29483, (ushort)24186, (ushort)8352, (ushort)60815, (ushort)62808, (ushort)32727, (ushort)1954, (ushort)24566, (ushort)17491, (ushort)21155, (ushort)2143, (ushort)10449, (ushort)63320, (ushort)33451, (ushort)62765, (ushort)29085}));
                Debug.Assert(pack.max_distance == (ushort)(ushort)20111);
            };
            GroundControl.OBSTACLE_DISTANCE p330 = CommunicationChannel.new_OBSTACLE_DISTANCE();
            PH.setPack(p330);
            p330.min_distance = (ushort)(ushort)48908;
            p330.sensor_type = MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER;
            p330.increment = (byte)(byte)79;
            p330.distances_SET(new ushort[] {(ushort)10174, (ushort)39414, (ushort)42321, (ushort)26021, (ushort)35148, (ushort)4604, (ushort)59577, (ushort)59455, (ushort)1860, (ushort)48226, (ushort)56229, (ushort)12836, (ushort)58698, (ushort)9293, (ushort)30365, (ushort)40910, (ushort)12652, (ushort)26128, (ushort)31559, (ushort)39590, (ushort)59776, (ushort)48650, (ushort)51365, (ushort)19184, (ushort)64460, (ushort)43655, (ushort)54351, (ushort)32071, (ushort)52558, (ushort)2776, (ushort)3516, (ushort)63828, (ushort)62822, (ushort)20975, (ushort)50810, (ushort)48041, (ushort)42900, (ushort)39175, (ushort)43241, (ushort)45080, (ushort)52384, (ushort)18977, (ushort)36380, (ushort)38205, (ushort)5247, (ushort)23725, (ushort)30639, (ushort)57231, (ushort)59337, (ushort)30154, (ushort)4187, (ushort)20415, (ushort)64244, (ushort)62655, (ushort)7835, (ushort)28310, (ushort)29483, (ushort)24186, (ushort)8352, (ushort)60815, (ushort)62808, (ushort)32727, (ushort)1954, (ushort)24566, (ushort)17491, (ushort)21155, (ushort)2143, (ushort)10449, (ushort)63320, (ushort)33451, (ushort)62765, (ushort)29085}, 0) ;
            p330.time_usec = (ulong)4468446342304799868L;
            p330.max_distance = (ushort)(ushort)20111;
            CommunicationChannel.instance.send(p330);//put test pack to the  channel send buffer
            TestChannelAdvanced.transmission(CommunicationChannel.instance, SMP_TEST_CH);
        }
    }
}