package org.noname;

import org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds;

public class Demo extends GroundControl {
	public static void main(String[] args) {
		final Bounds.Inside PH = new Bounds.Inside();
		CommunicationChannel.instance.on_HEARTBEAT.add((src, ph, pack) ->
		{
			@MAV_TYPE int      type            = pack.type_GET();
			@MAV_AUTOPILOT int autopilot       = pack.autopilot_GET();
			@MAV_MODE_FLAG int base_mode       = pack.base_mode_GET();
			long               custom_mode     = pack.custom_mode_GET();
			@MAV_STATE int     system_status   = pack.system_status_GET();
			char               mavlink_version = pack.mavlink_version_GET();
		});
		CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
		{
			@MAV_SYS_STATUS_SENSOR int onboard_control_sensors_present = pack.onboard_control_sensors_present_GET();
			@MAV_SYS_STATUS_SENSOR int onboard_control_sensors_enabled = pack.onboard_control_sensors_enabled_GET();
			@MAV_SYS_STATUS_SENSOR int onboard_control_sensors_health  = pack.onboard_control_sensors_health_GET();
			char                       load                            = pack.load_GET();
			char                       voltage_battery                 = pack.voltage_battery_GET();
			short                      current_battery                 = pack.current_battery_GET();
			byte                       battery_remaining               = pack.battery_remaining_GET();
			char                       drop_rate_comm                  = pack.drop_rate_comm_GET();
			char                       errors_comm                     = pack.errors_comm_GET();
			char                       errors_count1                   = pack.errors_count1_GET();
			char                       errors_count2                   = pack.errors_count2_GET();
			char                       errors_count3                   = pack.errors_count3_GET();
			char                       errors_count4                   = pack.errors_count4_GET();
		});
		CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
		{
			long time_unix_usec = pack.time_unix_usec_GET();
			long time_boot_ms   = pack.time_boot_ms_GET();
		});
		CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
		{
			long           time_boot_ms     = pack.time_boot_ms_GET();
			@MAV_FRAME int coordinate_frame = pack.coordinate_frame_GET();
			char           type_mask        = pack.type_mask_GET();
			float          x                = pack.x_GET();
			float          y                = pack.y_GET();
			float          z                = pack.z_GET();
			float          vx               = pack.vx_GET();
			float          vy               = pack.vy_GET();
			float          vz               = pack.vz_GET();
			float          afx              = pack.afx_GET();
			float          afy              = pack.afy_GET();
			float          afz              = pack.afz_GET();
			float          yaw              = pack.yaw_GET();
			float          yaw_rate         = pack.yaw_rate_GET();
		});
		CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
		{
			long time_usec        = pack.time_usec_GET();
			long seq              = pack.seq_GET();
			char target_system    = pack.target_system_GET();
			char target_component = pack.target_component_GET();
		});
		CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
		{
			char   target_system   = pack.target_system_GET();
			char   control_request = pack.control_request_GET();
			char   version         = pack.version_GET();
			String passkey         = pack.passkey_TRY(ph);
		});
		CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
		{
			char gcs_system_id   = pack.gcs_system_id_GET();
			char control_request = pack.control_request_GET();
			char ack             = pack.ack_GET();
		});
		CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
		{
			String key = pack.key_TRY(ph);
		});
		CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
		{
			char          target_system = pack.target_system_GET();
			@MAV_MODE int base_mode     = pack.base_mode_GET();
			long          custom_mode   = pack.custom_mode_GET();
		});
		CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
		{
			char   target_system    = pack.target_system_GET();
			char   target_component = pack.target_component_GET();
			String param_id         = pack.param_id_TRY(ph);
			short  param_index      = pack.param_index_GET();
		});
		CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
		{
			char target_system    = pack.target_system_GET();
			char target_component = pack.target_component_GET();
		});
		CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
		{
			String              param_id    = pack.param_id_TRY(ph);
			float               param_value = pack.param_value_GET();
			@MAV_PARAM_TYPE int param_type  = pack.param_type_GET();
			char                param_count = pack.param_count_GET();
			char                param_index = pack.param_index_GET();
		});
		CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
		{
			char                target_system    = pack.target_system_GET();
			char                target_component = pack.target_component_GET();
			String              param_id         = pack.param_id_TRY(ph);
			float               param_value      = pack.param_value_GET();
			@MAV_PARAM_TYPE int param_type       = pack.param_type_GET();
		});
		CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
		{
			long              time_usec          = pack.time_usec_GET();
			@GPS_FIX_TYPE int fix_type           = pack.fix_type_GET();
			int               lat                = pack.lat_GET();
			int               lon                = pack.lon_GET();
			int               alt                = pack.alt_GET();
			char              eph                = pack.eph_GET();
			char              epv                = pack.epv_GET();
			char              vel                = pack.vel_GET();
			char              cog                = pack.cog_GET();
			char              satellites_visible = pack.satellites_visible_GET();
			int               alt_ellipsoid      = pack.alt_ellipsoid_TRY(ph);
			long              h_acc              = pack.h_acc_TRY(ph);
			long              v_acc              = pack.v_acc_TRY(ph);
			long              vel_acc            = pack.vel_acc_TRY(ph);
			long              hdg_acc            = pack.hdg_acc_TRY(ph);
		});
		CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
		{
			char   satellites_visible  = pack.satellites_visible_GET();
			char[] satellite_prn       = pack.satellite_prn_GET();
			char[] satellite_used      = pack.satellite_used_GET();
			char[] satellite_elevation = pack.satellite_elevation_GET();
			char[] satellite_azimuth   = pack.satellite_azimuth_GET();
			char[] satellite_snr       = pack.satellite_snr_GET();
		});
		CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
		{
			long  time_boot_ms = pack.time_boot_ms_GET();
			short xacc         = pack.xacc_GET();
			short yacc         = pack.yacc_GET();
			short zacc         = pack.zacc_GET();
			short xgyro        = pack.xgyro_GET();
			short ygyro        = pack.ygyro_GET();
			short zgyro        = pack.zgyro_GET();
			short xmag         = pack.xmag_GET();
			short ymag         = pack.ymag_GET();
			short zmag         = pack.zmag_GET();
		});
		CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
		{
			long  time_usec = pack.time_usec_GET();
			short xacc      = pack.xacc_GET();
			short yacc      = pack.yacc_GET();
			short zacc      = pack.zacc_GET();
			short xgyro     = pack.xgyro_GET();
			short ygyro     = pack.ygyro_GET();
			short zgyro     = pack.zgyro_GET();
			short xmag      = pack.xmag_GET();
			short ymag      = pack.ymag_GET();
			short zmag      = pack.zmag_GET();
		});
		CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
		{
			long  time_usec   = pack.time_usec_GET();
			short press_abs   = pack.press_abs_GET();
			short press_diff1 = pack.press_diff1_GET();
			short press_diff2 = pack.press_diff2_GET();
			short temperature = pack.temperature_GET();
		});
		CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
		{
			long  time_boot_ms = pack.time_boot_ms_GET();
			float press_abs    = pack.press_abs_GET();
			float press_diff   = pack.press_diff_GET();
			short temperature  = pack.temperature_GET();
		});
		CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
		{
			long  time_boot_ms = pack.time_boot_ms_GET();
			float roll         = pack.roll_GET();
			float pitch        = pack.pitch_GET();
			float yaw          = pack.yaw_GET();
			float rollspeed    = pack.rollspeed_GET();
			float pitchspeed   = pack.pitchspeed_GET();
			float yawspeed     = pack.yawspeed_GET();
		});
		CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
		{
			long  time_boot_ms = pack.time_boot_ms_GET();
			float q1           = pack.q1_GET();
			float q2           = pack.q2_GET();
			float q3           = pack.q3_GET();
			float q4           = pack.q4_GET();
			float rollspeed    = pack.rollspeed_GET();
			float pitchspeed   = pack.pitchspeed_GET();
			float yawspeed     = pack.yawspeed_GET();
		});
		CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
		{
			long  time_boot_ms = pack.time_boot_ms_GET();
			float x            = pack.x_GET();
			float y            = pack.y_GET();
			float z            = pack.z_GET();
			float vx           = pack.vx_GET();
			float vy           = pack.vy_GET();
			float vz           = pack.vz_GET();
		});
		CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
		{
			long  time_boot_ms = pack.time_boot_ms_GET();
			int   lat          = pack.lat_GET();
			int   lon          = pack.lon_GET();
			int   alt          = pack.alt_GET();
			int   relative_alt = pack.relative_alt_GET();
			short vx           = pack.vx_GET();
			short vy           = pack.vy_GET();
			short vz           = pack.vz_GET();
			char  hdg          = pack.hdg_GET();
		});
		CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
		{
			long  time_boot_ms = pack.time_boot_ms_GET();
			char  port         = pack.port_GET();
			short chan1_scaled = pack.chan1_scaled_GET();
			short chan2_scaled = pack.chan2_scaled_GET();
			short chan3_scaled = pack.chan3_scaled_GET();
			short chan4_scaled = pack.chan4_scaled_GET();
			short chan5_scaled = pack.chan5_scaled_GET();
			short chan6_scaled = pack.chan6_scaled_GET();
			short chan7_scaled = pack.chan7_scaled_GET();
			short chan8_scaled = pack.chan8_scaled_GET();
			char  rssi         = pack.rssi_GET();
		});
		CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
		{
			long time_boot_ms = pack.time_boot_ms_GET();
			char port         = pack.port_GET();
			char chan1_raw    = pack.chan1_raw_GET();
			char chan2_raw    = pack.chan2_raw_GET();
			char chan3_raw    = pack.chan3_raw_GET();
			char chan4_raw    = pack.chan4_raw_GET();
			char chan5_raw    = pack.chan5_raw_GET();
			char chan6_raw    = pack.chan6_raw_GET();
			char chan7_raw    = pack.chan7_raw_GET();
			char chan8_raw    = pack.chan8_raw_GET();
			char rssi         = pack.rssi_GET();
		});
		CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
		{
			long time_usec   = pack.time_usec_GET();
			char port        = pack.port_GET();
			char servo1_raw  = pack.servo1_raw_GET();
			char servo2_raw  = pack.servo2_raw_GET();
			char servo3_raw  = pack.servo3_raw_GET();
			char servo4_raw  = pack.servo4_raw_GET();
			char servo5_raw  = pack.servo5_raw_GET();
			char servo6_raw  = pack.servo6_raw_GET();
			char servo7_raw  = pack.servo7_raw_GET();
			char servo8_raw  = pack.servo8_raw_GET();
			char servo9_raw  = pack.servo9_raw_TRY(ph);
			char servo10_raw = pack.servo10_raw_TRY(ph);
			char servo11_raw = pack.servo11_raw_TRY(ph);
			char servo12_raw = pack.servo12_raw_TRY(ph);
			char servo13_raw = pack.servo13_raw_TRY(ph);
			char servo14_raw = pack.servo14_raw_TRY(ph);
			char servo15_raw = pack.servo15_raw_TRY(ph);
			char servo16_raw = pack.servo16_raw_TRY(ph);
		});
		CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
		{
			char                  target_system    = pack.target_system_GET();
			char                  target_component = pack.target_component_GET();
			short                 start_index      = pack.start_index_GET();
			short                 end_index        = pack.end_index_GET();
			@MAV_MISSION_TYPE int mission_type     = pack.mission_type_GET();
		});
		CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
		{
			char                  target_system    = pack.target_system_GET();
			char                  target_component = pack.target_component_GET();
			short                 start_index      = pack.start_index_GET();
			short                 end_index        = pack.end_index_GET();
			@MAV_MISSION_TYPE int mission_type     = pack.mission_type_GET();
		});
		CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
		{
			char                  target_system    = pack.target_system_GET();
			char                  target_component = pack.target_component_GET();
			char                  seq              = pack.seq_GET();
			@MAV_FRAME int        frame            = pack.frame_GET();
			@MAV_CMD int          command          = pack.command_GET();
			char                  current          = pack.current_GET();
			char                  autocontinue     = pack.autocontinue_GET();
			float                 param1           = pack.param1_GET();
			float                 param2           = pack.param2_GET();
			float                 param3           = pack.param3_GET();
			float                 param4           = pack.param4_GET();
			float                 x                = pack.x_GET();
			float                 y                = pack.y_GET();
			float                 z                = pack.z_GET();
			@MAV_MISSION_TYPE int mission_type     = pack.mission_type_GET();
		});
		CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
		{
			char                  target_system    = pack.target_system_GET();
			char                  target_component = pack.target_component_GET();
			char                  seq              = pack.seq_GET();
			@MAV_MISSION_TYPE int mission_type     = pack.mission_type_GET();
		});
		CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
		{
			char target_system    = pack.target_system_GET();
			char target_component = pack.target_component_GET();
			char seq              = pack.seq_GET();
		});
		CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
		{
			char seq = pack.seq_GET();
		});
		CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
		{
			char                  target_system    = pack.target_system_GET();
			char                  target_component = pack.target_component_GET();
			@MAV_MISSION_TYPE int mission_type     = pack.mission_type_GET();
		});
		CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
		{
			char                  target_system    = pack.target_system_GET();
			char                  target_component = pack.target_component_GET();
			char                  count            = pack.count_GET();
			@MAV_MISSION_TYPE int mission_type     = pack.mission_type_GET();
		});
		CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
		{
			char                  target_system    = pack.target_system_GET();
			char                  target_component = pack.target_component_GET();
			@MAV_MISSION_TYPE int mission_type     = pack.mission_type_GET();
		});
		CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
		{
			char seq = pack.seq_GET();
		});
		CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
		{
			char                    target_system    = pack.target_system_GET();
			char                    target_component = pack.target_component_GET();
			@MAV_MISSION_RESULT int type             = pack.type_GET();
			@MAV_MISSION_TYPE int   mission_type     = pack.mission_type_GET();
		});
		CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
		{
			char target_system = pack.target_system_GET();
			int  latitude      = pack.latitude_GET();
			int  longitude     = pack.longitude_GET();
			int  altitude      = pack.altitude_GET();
			long time_usec     = pack.time_usec_TRY(ph);
		});
		CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
		{
			int  latitude  = pack.latitude_GET();
			int  longitude = pack.longitude_GET();
			int  altitude  = pack.altitude_GET();
			long time_usec = pack.time_usec_TRY(ph);
		});
		CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
		{
			char   target_system              = pack.target_system_GET();
			char   target_component           = pack.target_component_GET();
			String param_id                   = pack.param_id_TRY(ph);
			short  param_index                = pack.param_index_GET();
			char   parameter_rc_channel_index = pack.parameter_rc_channel_index_GET();
			float  param_value0               = pack.param_value0_GET();
			float  scale                      = pack.scale_GET();
			float  param_value_min            = pack.param_value_min_GET();
			float  param_value_max            = pack.param_value_max_GET();
		});
		CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
		{
			char                  target_system    = pack.target_system_GET();
			char                  target_component = pack.target_component_GET();
			char                  seq              = pack.seq_GET();
			@MAV_MISSION_TYPE int mission_type     = pack.mission_type_GET();
		});
		CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
		{
			char           target_system    = pack.target_system_GET();
			char           target_component = pack.target_component_GET();
			@MAV_FRAME int frame            = pack.frame_GET();
			float          p1x              = pack.p1x_GET();
			float          p1y              = pack.p1y_GET();
			float          p1z              = pack.p1z_GET();
			float          p2x              = pack.p2x_GET();
			float          p2y              = pack.p2y_GET();
			float          p2z              = pack.p2z_GET();
		});
		CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
		{
			@MAV_FRAME int frame = pack.frame_GET();
			float          p1x   = pack.p1x_GET();
			float          p1y   = pack.p1y_GET();
			float          p1z   = pack.p1z_GET();
			float          p2x   = pack.p2x_GET();
			float          p2y   = pack.p2y_GET();
			float          p2z   = pack.p2z_GET();
		});
		CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
		{
			long    time_usec  = pack.time_usec_GET();
			float[] q          = pack.q_GET();
			float   rollspeed  = pack.rollspeed_GET();
			float   pitchspeed = pack.pitchspeed_GET();
			float   yawspeed   = pack.yawspeed_GET();
			float[] covariance = pack.covariance_GET();
		});
		CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
		{
			float nav_roll       = pack.nav_roll_GET();
			float nav_pitch      = pack.nav_pitch_GET();
			short nav_bearing    = pack.nav_bearing_GET();
			short target_bearing = pack.target_bearing_GET();
			char  wp_dist        = pack.wp_dist_GET();
			float alt_error      = pack.alt_error_GET();
			float aspd_error     = pack.aspd_error_GET();
			float xtrack_error   = pack.xtrack_error_GET();
		});
		CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
		{
			long                    time_usec      = pack.time_usec_GET();
			@MAV_ESTIMATOR_TYPE int estimator_type = pack.estimator_type_GET();
			int                     lat            = pack.lat_GET();
			int                     lon            = pack.lon_GET();
			int                     alt            = pack.alt_GET();
			int                     relative_alt   = pack.relative_alt_GET();
			float                   vx             = pack.vx_GET();
			float                   vy             = pack.vy_GET();
			float                   vz             = pack.vz_GET();
			float[]                 covariance     = pack.covariance_GET();
		});
		CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
		{
			long                    time_usec      = pack.time_usec_GET();
			@MAV_ESTIMATOR_TYPE int estimator_type = pack.estimator_type_GET();
			float                   x              = pack.x_GET();
			float                   y              = pack.y_GET();
			float                   z              = pack.z_GET();
			float                   vx             = pack.vx_GET();
			float                   vy             = pack.vy_GET();
			float                   vz             = pack.vz_GET();
			float                   ax             = pack.ax_GET();
			float                   ay             = pack.ay_GET();
			float                   az             = pack.az_GET();
			float[]                 covariance     = pack.covariance_GET();
		});
		CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
		{
			long time_boot_ms = pack.time_boot_ms_GET();
			char chancount    = pack.chancount_GET();
			char chan1_raw    = pack.chan1_raw_GET();
			char chan2_raw    = pack.chan2_raw_GET();
			char chan3_raw    = pack.chan3_raw_GET();
			char chan4_raw    = pack.chan4_raw_GET();
			char chan5_raw    = pack.chan5_raw_GET();
			char chan6_raw    = pack.chan6_raw_GET();
			char chan7_raw    = pack.chan7_raw_GET();
			char chan8_raw    = pack.chan8_raw_GET();
			char chan9_raw    = pack.chan9_raw_GET();
			char chan10_raw   = pack.chan10_raw_GET();
			char chan11_raw   = pack.chan11_raw_GET();
			char chan12_raw   = pack.chan12_raw_GET();
			char chan13_raw   = pack.chan13_raw_GET();
			char chan14_raw   = pack.chan14_raw_GET();
			char chan15_raw   = pack.chan15_raw_GET();
			char chan16_raw   = pack.chan16_raw_GET();
			char chan17_raw   = pack.chan17_raw_GET();
			char chan18_raw   = pack.chan18_raw_GET();
			char rssi         = pack.rssi_GET();
		});
		CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
		{
			char target_system    = pack.target_system_GET();
			char target_component = pack.target_component_GET();
			char req_stream_id    = pack.req_stream_id_GET();
			char req_message_rate = pack.req_message_rate_GET();
			char start_stop       = pack.start_stop_GET();
		});
		CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
		{
			char stream_id    = pack.stream_id_GET();
			char message_rate = pack.message_rate_GET();
			char on_off       = pack.on_off_GET();
		});
		CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
		{
			char  target  = pack.target_GET();
			short x       = pack.x_GET();
			short y       = pack.y_GET();
			short z       = pack.z_GET();
			short r       = pack.r_GET();
			char  buttons = pack.buttons_GET();
		});
		CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
		{
			char target_system    = pack.target_system_GET();
			char target_component = pack.target_component_GET();
			char chan1_raw        = pack.chan1_raw_GET();
			char chan2_raw        = pack.chan2_raw_GET();
			char chan3_raw        = pack.chan3_raw_GET();
			char chan4_raw        = pack.chan4_raw_GET();
			char chan5_raw        = pack.chan5_raw_GET();
			char chan6_raw        = pack.chan6_raw_GET();
			char chan7_raw        = pack.chan7_raw_GET();
			char chan8_raw        = pack.chan8_raw_GET();
		});
		CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
		{
			char                  target_system    = pack.target_system_GET();
			char                  target_component = pack.target_component_GET();
			char                  seq              = pack.seq_GET();
			@MAV_FRAME int        frame            = pack.frame_GET();
			@MAV_CMD int          command          = pack.command_GET();
			char                  current          = pack.current_GET();
			char                  autocontinue     = pack.autocontinue_GET();
			float                 param1           = pack.param1_GET();
			float                 param2           = pack.param2_GET();
			float                 param3           = pack.param3_GET();
			float                 param4           = pack.param4_GET();
			int                   x                = pack.x_GET();
			int                   y                = pack.y_GET();
			float                 z                = pack.z_GET();
			@MAV_MISSION_TYPE int mission_type     = pack.mission_type_GET();
		});
		CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
		{
			float airspeed    = pack.airspeed_GET();
			float groundspeed = pack.groundspeed_GET();
			short heading     = pack.heading_GET();
			char  throttle    = pack.throttle_GET();
			float alt         = pack.alt_GET();
			float climb       = pack.climb_GET();
		});
		CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
		{
			char           target_system    = pack.target_system_GET();
			char           target_component = pack.target_component_GET();
			@MAV_FRAME int frame            = pack.frame_GET();
			@MAV_CMD int   command          = pack.command_GET();
			char           current          = pack.current_GET();
			char           autocontinue     = pack.autocontinue_GET();
			float          param1           = pack.param1_GET();
			float          param2           = pack.param2_GET();
			float          param3           = pack.param3_GET();
			float          param4           = pack.param4_GET();
			int            x                = pack.x_GET();
			int            y                = pack.y_GET();
			float          z                = pack.z_GET();
		});
		CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
		{
			char         target_system    = pack.target_system_GET();
			char         target_component = pack.target_component_GET();
			@MAV_CMD int command          = pack.command_GET();
			char         confirmation     = pack.confirmation_GET();
			float        param1           = pack.param1_GET();
			float        param2           = pack.param2_GET();
			float        param3           = pack.param3_GET();
			float        param4           = pack.param4_GET();
			float        param5           = pack.param5_GET();
			float        param6           = pack.param6_GET();
			float        param7           = pack.param7_GET();
		});
		CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
		{
			@MAV_CMD int    command          = pack.command_GET();
			@MAV_RESULT int result           = pack.result_GET();
			char            progress         = pack.progress_TRY(ph);
			int             result_param2    = pack.result_param2_TRY(ph);
			char            target_system    = pack.target_system_TRY(ph);
			char            target_component = pack.target_component_TRY(ph);
		});
		CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
		{
			long  time_boot_ms           = pack.time_boot_ms_GET();
			float roll                   = pack.roll_GET();
			float pitch                  = pack.pitch_GET();
			float yaw                    = pack.yaw_GET();
			float thrust                 = pack.thrust_GET();
			char  mode_switch            = pack.mode_switch_GET();
			char  manual_override_switch = pack.manual_override_switch_GET();
		});
		CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
		{
			long    time_boot_ms     = pack.time_boot_ms_GET();
			char    target_system    = pack.target_system_GET();
			char    target_component = pack.target_component_GET();
			char    type_mask        = pack.type_mask_GET();
			float[] q                = pack.q_GET();
			float   body_roll_rate   = pack.body_roll_rate_GET();
			float   body_pitch_rate  = pack.body_pitch_rate_GET();
			float   body_yaw_rate    = pack.body_yaw_rate_GET();
			float   thrust           = pack.thrust_GET();
		});
		CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
		{
			long    time_boot_ms    = pack.time_boot_ms_GET();
			char    type_mask       = pack.type_mask_GET();
			float[] q               = pack.q_GET();
			float   body_roll_rate  = pack.body_roll_rate_GET();
			float   body_pitch_rate = pack.body_pitch_rate_GET();
			float   body_yaw_rate   = pack.body_yaw_rate_GET();
			float   thrust          = pack.thrust_GET();
		});
		CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
		{
			long           time_boot_ms     = pack.time_boot_ms_GET();
			char           target_system    = pack.target_system_GET();
			char           target_component = pack.target_component_GET();
			@MAV_FRAME int coordinate_frame = pack.coordinate_frame_GET();
			char           type_mask        = pack.type_mask_GET();
			float          x                = pack.x_GET();
			float          y                = pack.y_GET();
			float          z                = pack.z_GET();
			float          vx               = pack.vx_GET();
			float          vy               = pack.vy_GET();
			float          vz               = pack.vz_GET();
			float          afx              = pack.afx_GET();
			float          afy              = pack.afy_GET();
			float          afz              = pack.afz_GET();
			float          yaw              = pack.yaw_GET();
			float          yaw_rate         = pack.yaw_rate_GET();
		});
		CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
		{
			long           time_boot_ms     = pack.time_boot_ms_GET();
			char           target_system    = pack.target_system_GET();
			char           target_component = pack.target_component_GET();
			@MAV_FRAME int coordinate_frame = pack.coordinate_frame_GET();
			char           type_mask        = pack.type_mask_GET();
			int            lat_int          = pack.lat_int_GET();
			int            lon_int          = pack.lon_int_GET();
			float          alt              = pack.alt_GET();
			float          vx               = pack.vx_GET();
			float          vy               = pack.vy_GET();
			float          vz               = pack.vz_GET();
			float          afx              = pack.afx_GET();
			float          afy              = pack.afy_GET();
			float          afz              = pack.afz_GET();
			float          yaw              = pack.yaw_GET();
			float          yaw_rate         = pack.yaw_rate_GET();
		});
		CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
		{
			long           time_boot_ms     = pack.time_boot_ms_GET();
			@MAV_FRAME int coordinate_frame = pack.coordinate_frame_GET();
			char           type_mask        = pack.type_mask_GET();
			int            lat_int          = pack.lat_int_GET();
			int            lon_int          = pack.lon_int_GET();
			float          alt              = pack.alt_GET();
			float          vx               = pack.vx_GET();
			float          vy               = pack.vy_GET();
			float          vz               = pack.vz_GET();
			float          afx              = pack.afx_GET();
			float          afy              = pack.afy_GET();
			float          afz              = pack.afz_GET();
			float          yaw              = pack.yaw_GET();
			float          yaw_rate         = pack.yaw_rate_GET();
		});
		CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
		{
			long  time_boot_ms = pack.time_boot_ms_GET();
			float x            = pack.x_GET();
			float y            = pack.y_GET();
			float z            = pack.z_GET();
			float roll         = pack.roll_GET();
			float pitch        = pack.pitch_GET();
			float yaw          = pack.yaw_GET();
		});
		CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
		{
			long  time_usec  = pack.time_usec_GET();
			float roll       = pack.roll_GET();
			float pitch      = pack.pitch_GET();
			float yaw        = pack.yaw_GET();
			float rollspeed  = pack.rollspeed_GET();
			float pitchspeed = pack.pitchspeed_GET();
			float yawspeed   = pack.yawspeed_GET();
			int   lat        = pack.lat_GET();
			int   lon        = pack.lon_GET();
			int   alt        = pack.alt_GET();
			short vx         = pack.vx_GET();
			short vy         = pack.vy_GET();
			short vz         = pack.vz_GET();
			short xacc       = pack.xacc_GET();
			short yacc       = pack.yacc_GET();
			short zacc       = pack.zacc_GET();
		});
		CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
		{
			long          time_usec      = pack.time_usec_GET();
			float         roll_ailerons  = pack.roll_ailerons_GET();
			float         pitch_elevator = pack.pitch_elevator_GET();
			float         yaw_rudder     = pack.yaw_rudder_GET();
			float         throttle       = pack.throttle_GET();
			float         aux1           = pack.aux1_GET();
			float         aux2           = pack.aux2_GET();
			float         aux3           = pack.aux3_GET();
			float         aux4           = pack.aux4_GET();
			@MAV_MODE int mode           = pack.mode_GET();
			char          nav_mode       = pack.nav_mode_GET();
		});
		CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
		{
			long time_usec  = pack.time_usec_GET();
			char chan1_raw  = pack.chan1_raw_GET();
			char chan2_raw  = pack.chan2_raw_GET();
			char chan3_raw  = pack.chan3_raw_GET();
			char chan4_raw  = pack.chan4_raw_GET();
			char chan5_raw  = pack.chan5_raw_GET();
			char chan6_raw  = pack.chan6_raw_GET();
			char chan7_raw  = pack.chan7_raw_GET();
			char chan8_raw  = pack.chan8_raw_GET();
			char chan9_raw  = pack.chan9_raw_GET();
			char chan10_raw = pack.chan10_raw_GET();
			char chan11_raw = pack.chan11_raw_GET();
			char chan12_raw = pack.chan12_raw_GET();
			char rssi       = pack.rssi_GET();
		});
		CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
		{
			long          time_usec = pack.time_usec_GET();
			float[]       controls  = pack.controls_GET();
			@MAV_MODE int mode      = pack.mode_GET();
			long          flags     = pack.flags_GET();
		});
		CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
		{
			long  time_usec       = pack.time_usec_GET();
			char  sensor_id       = pack.sensor_id_GET();
			short flow_x          = pack.flow_x_GET();
			short flow_y          = pack.flow_y_GET();
			float flow_comp_m_x   = pack.flow_comp_m_x_GET();
			float flow_comp_m_y   = pack.flow_comp_m_y_GET();
			char  quality         = pack.quality_GET();
			float ground_distance = pack.ground_distance_GET();
			float flow_rate_x     = pack.flow_rate_x_TRY(ph);
			float flow_rate_y     = pack.flow_rate_y_TRY(ph);
		});
		CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
		{
			long  usec  = pack.usec_GET();
			float x     = pack.x_GET();
			float y     = pack.y_GET();
			float z     = pack.z_GET();
			float roll  = pack.roll_GET();
			float pitch = pack.pitch_GET();
			float yaw   = pack.yaw_GET();
		});
		CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
		{
			long  usec  = pack.usec_GET();
			float x     = pack.x_GET();
			float y     = pack.y_GET();
			float z     = pack.z_GET();
			float roll  = pack.roll_GET();
			float pitch = pack.pitch_GET();
			float yaw   = pack.yaw_GET();
		});
		CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
		{
			long  usec = pack.usec_GET();
			float x    = pack.x_GET();
			float y    = pack.y_GET();
			float z    = pack.z_GET();
		});
		CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
		{
			long  usec  = pack.usec_GET();
			float x     = pack.x_GET();
			float y     = pack.y_GET();
			float z     = pack.z_GET();
			float roll  = pack.roll_GET();
			float pitch = pack.pitch_GET();
			float yaw   = pack.yaw_GET();
		});
		CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
		{
			long  time_usec      = pack.time_usec_GET();
			float xacc           = pack.xacc_GET();
			float yacc           = pack.yacc_GET();
			float zacc           = pack.zacc_GET();
			float xgyro          = pack.xgyro_GET();
			float ygyro          = pack.ygyro_GET();
			float zgyro          = pack.zgyro_GET();
			float xmag           = pack.xmag_GET();
			float ymag           = pack.ymag_GET();
			float zmag           = pack.zmag_GET();
			float abs_pressure   = pack.abs_pressure_GET();
			float diff_pressure  = pack.diff_pressure_GET();
			float pressure_alt   = pack.pressure_alt_GET();
			float temperature    = pack.temperature_GET();
			char  fields_updated = pack.fields_updated_GET();
		});
		CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
		{
			long  time_usec              = pack.time_usec_GET();
			char  sensor_id              = pack.sensor_id_GET();
			long  integration_time_us    = pack.integration_time_us_GET();
			float integrated_x           = pack.integrated_x_GET();
			float integrated_y           = pack.integrated_y_GET();
			float integrated_xgyro       = pack.integrated_xgyro_GET();
			float integrated_ygyro       = pack.integrated_ygyro_GET();
			float integrated_zgyro       = pack.integrated_zgyro_GET();
			short temperature            = pack.temperature_GET();
			char  quality                = pack.quality_GET();
			long  time_delta_distance_us = pack.time_delta_distance_us_GET();
			float distance               = pack.distance_GET();
		});
		CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
		{
			long  time_usec      = pack.time_usec_GET();
			float xacc           = pack.xacc_GET();
			float yacc           = pack.yacc_GET();
			float zacc           = pack.zacc_GET();
			float xgyro          = pack.xgyro_GET();
			float ygyro          = pack.ygyro_GET();
			float zgyro          = pack.zgyro_GET();
			float xmag           = pack.xmag_GET();
			float ymag           = pack.ymag_GET();
			float zmag           = pack.zmag_GET();
			float abs_pressure   = pack.abs_pressure_GET();
			float diff_pressure  = pack.diff_pressure_GET();
			float pressure_alt   = pack.pressure_alt_GET();
			float temperature    = pack.temperature_GET();
			long  fields_updated = pack.fields_updated_GET();
		});
		CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
		{
			float q1           = pack.q1_GET();
			float q2           = pack.q2_GET();
			float q3           = pack.q3_GET();
			float q4           = pack.q4_GET();
			float roll         = pack.roll_GET();
			float pitch        = pack.pitch_GET();
			float yaw          = pack.yaw_GET();
			float xacc         = pack.xacc_GET();
			float yacc         = pack.yacc_GET();
			float zacc         = pack.zacc_GET();
			float xgyro        = pack.xgyro_GET();
			float ygyro        = pack.ygyro_GET();
			float zgyro        = pack.zgyro_GET();
			float lat          = pack.lat_GET();
			float lon          = pack.lon_GET();
			float alt          = pack.alt_GET();
			float std_dev_horz = pack.std_dev_horz_GET();
			float std_dev_vert = pack.std_dev_vert_GET();
			float vn           = pack.vn_GET();
			float ve           = pack.ve_GET();
			float vd           = pack.vd_GET();
		});
		CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
		{
			char rssi     = pack.rssi_GET();
			char remrssi  = pack.remrssi_GET();
			char txbuf    = pack.txbuf_GET();
			char noise    = pack.noise_GET();
			char remnoise = pack.remnoise_GET();
			char rxerrors = pack.rxerrors_GET();
			char fixed_   = pack.fixed__GET();
		});
		CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
		{
			char   target_network   = pack.target_network_GET();
			char   target_system    = pack.target_system_GET();
			char   target_component = pack.target_component_GET();
			char[] payload          = pack.payload_GET();
		});
		CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
		{
			long tc1 = pack.tc1_GET();
			long ts1 = pack.ts1_GET();
		});
		CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
		{
			long time_usec = pack.time_usec_GET();
			long seq       = pack.seq_GET();
		});
		CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
		{
			long  time_usec          = pack.time_usec_GET();
			char  fix_type           = pack.fix_type_GET();
			int   lat                = pack.lat_GET();
			int   lon                = pack.lon_GET();
			int   alt                = pack.alt_GET();
			char  eph                = pack.eph_GET();
			char  epv                = pack.epv_GET();
			char  vel                = pack.vel_GET();
			short vn                 = pack.vn_GET();
			short ve                 = pack.ve_GET();
			short vd                 = pack.vd_GET();
			char  cog                = pack.cog_GET();
			char  satellites_visible = pack.satellites_visible_GET();
		});
		CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
		{
			long  time_usec              = pack.time_usec_GET();
			char  sensor_id              = pack.sensor_id_GET();
			long  integration_time_us    = pack.integration_time_us_GET();
			float integrated_x           = pack.integrated_x_GET();
			float integrated_y           = pack.integrated_y_GET();
			float integrated_xgyro       = pack.integrated_xgyro_GET();
			float integrated_ygyro       = pack.integrated_ygyro_GET();
			float integrated_zgyro       = pack.integrated_zgyro_GET();
			short temperature            = pack.temperature_GET();
			char  quality                = pack.quality_GET();
			long  time_delta_distance_us = pack.time_delta_distance_us_GET();
			float distance               = pack.distance_GET();
		});
		CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
		{
			long    time_usec           = pack.time_usec_GET();
			float[] attitude_quaternion = pack.attitude_quaternion_GET();
			float   rollspeed           = pack.rollspeed_GET();
			float   pitchspeed          = pack.pitchspeed_GET();
			float   yawspeed            = pack.yawspeed_GET();
			int     lat                 = pack.lat_GET();
			int     lon                 = pack.lon_GET();
			int     alt                 = pack.alt_GET();
			short   vx                  = pack.vx_GET();
			short   vy                  = pack.vy_GET();
			short   vz                  = pack.vz_GET();
			char    ind_airspeed        = pack.ind_airspeed_GET();
			char    true_airspeed       = pack.true_airspeed_GET();
			short   xacc                = pack.xacc_GET();
			short   yacc                = pack.yacc_GET();
			short   zacc                = pack.zacc_GET();
		});
		CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
		{
			long  time_boot_ms = pack.time_boot_ms_GET();
			short xacc         = pack.xacc_GET();
			short yacc         = pack.yacc_GET();
			short zacc         = pack.zacc_GET();
			short xgyro        = pack.xgyro_GET();
			short ygyro        = pack.ygyro_GET();
			short zgyro        = pack.zgyro_GET();
			short xmag         = pack.xmag_GET();
			short ymag         = pack.ymag_GET();
			short zmag         = pack.zmag_GET();
		});
		CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
		{
			char target_system    = pack.target_system_GET();
			char target_component = pack.target_component_GET();
			char start            = pack.start_GET();
			char end              = pack.end_GET();
		});
		CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
		{
			char id           = pack.id_GET();
			char num_logs     = pack.num_logs_GET();
			char last_log_num = pack.last_log_num_GET();
			long time_utc     = pack.time_utc_GET();
			long size         = pack.size_GET();
		});
		CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
		{
			char target_system    = pack.target_system_GET();
			char target_component = pack.target_component_GET();
			char id               = pack.id_GET();
			long ofs              = pack.ofs_GET();
			long count            = pack.count_GET();
		});
		CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
		{
			char   id    = pack.id_GET();
			long   ofs   = pack.ofs_GET();
			char   count = pack.count_GET();
			char[] data_ = pack.data__GET();
		});
		CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
		{
			char target_system    = pack.target_system_GET();
			char target_component = pack.target_component_GET();
		});
		CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
		{
			char target_system    = pack.target_system_GET();
			char target_component = pack.target_component_GET();
		});
		CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
		{
			char   target_system    = pack.target_system_GET();
			char   target_component = pack.target_component_GET();
			char   len              = pack.len_GET();
			char[] data_            = pack.data__GET();
		});
		CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
		{
			long              time_usec          = pack.time_usec_GET();
			@GPS_FIX_TYPE int fix_type           = pack.fix_type_GET();
			int               lat                = pack.lat_GET();
			int               lon                = pack.lon_GET();
			int               alt                = pack.alt_GET();
			char              eph                = pack.eph_GET();
			char              epv                = pack.epv_GET();
			char              vel                = pack.vel_GET();
			char              cog                = pack.cog_GET();
			char              satellites_visible = pack.satellites_visible_GET();
			char              dgps_numch         = pack.dgps_numch_GET();
			long              dgps_age           = pack.dgps_age_GET();
		});
		CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
		{
			char                  Vcc    = pack.Vcc_GET();
			char                  Vservo = pack.Vservo_GET();
			@MAV_POWER_STATUS int flags  = pack.flags_GET();
		});
		CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
		{
			@SERIAL_CONTROL_DEV int  device   = pack.device_GET();
			@SERIAL_CONTROL_FLAG int flags    = pack.flags_GET();
			char                     timeout  = pack.timeout_GET();
			long                     baudrate = pack.baudrate_GET();
			char                     count    = pack.count_GET();
			char[]                   data_    = pack.data__GET();
		});
		CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
		{
			long time_last_baseline_ms = pack.time_last_baseline_ms_GET();
			char rtk_receiver_id       = pack.rtk_receiver_id_GET();
			char wn                    = pack.wn_GET();
			long tow                   = pack.tow_GET();
			char rtk_health            = pack.rtk_health_GET();
			char rtk_rate              = pack.rtk_rate_GET();
			char nsats                 = pack.nsats_GET();
			char baseline_coords_type  = pack.baseline_coords_type_GET();
			int  baseline_a_mm         = pack.baseline_a_mm_GET();
			int  baseline_b_mm         = pack.baseline_b_mm_GET();
			int  baseline_c_mm         = pack.baseline_c_mm_GET();
			long accuracy              = pack.accuracy_GET();
			int  iar_num_hypotheses    = pack.iar_num_hypotheses_GET();
		});
		CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
		{
			long time_last_baseline_ms = pack.time_last_baseline_ms_GET();
			char rtk_receiver_id       = pack.rtk_receiver_id_GET();
			char wn                    = pack.wn_GET();
			long tow                   = pack.tow_GET();
			char rtk_health            = pack.rtk_health_GET();
			char rtk_rate              = pack.rtk_rate_GET();
			char nsats                 = pack.nsats_GET();
			char baseline_coords_type  = pack.baseline_coords_type_GET();
			int  baseline_a_mm         = pack.baseline_a_mm_GET();
			int  baseline_b_mm         = pack.baseline_b_mm_GET();
			int  baseline_c_mm         = pack.baseline_c_mm_GET();
			long accuracy              = pack.accuracy_GET();
			int  iar_num_hypotheses    = pack.iar_num_hypotheses_GET();
		});
		CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
		{
			long  time_boot_ms = pack.time_boot_ms_GET();
			short xacc         = pack.xacc_GET();
			short yacc         = pack.yacc_GET();
			short zacc         = pack.zacc_GET();
			short xgyro        = pack.xgyro_GET();
			short ygyro        = pack.ygyro_GET();
			short zgyro        = pack.zgyro_GET();
			short xmag         = pack.xmag_GET();
			short ymag         = pack.ymag_GET();
			short zmag         = pack.zmag_GET();
		});
		CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
		{
			char type        = pack.type_GET();
			long size        = pack.size_GET();
			char width       = pack.width_GET();
			char height      = pack.height_GET();
			char packets     = pack.packets_GET();
			char payload     = pack.payload_GET();
			char jpg_quality = pack.jpg_quality_GET();
		});
		CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
		{
			char   seqnr = pack.seqnr_GET();
			char[] data_ = pack.data__GET();
		});
		CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
		{
			long                        time_boot_ms     = pack.time_boot_ms_GET();
			char                        min_distance     = pack.min_distance_GET();
			char                        max_distance     = pack.max_distance_GET();
			char                        current_distance = pack.current_distance_GET();
			@MAV_DISTANCE_SENSOR int    type             = pack.type_GET();
			char                        id               = pack.id_GET();
			@MAV_SENSOR_ORIENTATION int orientation      = pack.orientation_GET();
			char                        covariance       = pack.covariance_GET();
		});
		CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
		{
			int  lat          = pack.lat_GET();
			int  lon          = pack.lon_GET();
			char grid_spacing = pack.grid_spacing_GET();
			long mask         = pack.mask_GET();
		});
		CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
		{
			int     lat          = pack.lat_GET();
			int     lon          = pack.lon_GET();
			char    grid_spacing = pack.grid_spacing_GET();
			char    gridbit      = pack.gridbit_GET();
			short[] data_        = pack.data__GET();
		});
		CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
		{
			int lat = pack.lat_GET();
			int lon = pack.lon_GET();
		});
		CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
		{
			int   lat            = pack.lat_GET();
			int   lon            = pack.lon_GET();
			char  spacing        = pack.spacing_GET();
			float terrain_height = pack.terrain_height_GET();
			float current_height = pack.current_height_GET();
			char  pending        = pack.pending_GET();
			char  loaded         = pack.loaded_GET();
		});
		CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
		{
			long  time_boot_ms = pack.time_boot_ms_GET();
			float press_abs    = pack.press_abs_GET();
			float press_diff   = pack.press_diff_GET();
			short temperature  = pack.temperature_GET();
		});
		CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
		{
			long    time_usec = pack.time_usec_GET();
			float[] q         = pack.q_GET();
			float   x         = pack.x_GET();
			float   y         = pack.y_GET();
			float   z         = pack.z_GET();
		});
		CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
		{
			long    time_usec        = pack.time_usec_GET();
			char    group_mlx        = pack.group_mlx_GET();
			char    target_system    = pack.target_system_GET();
			char    target_component = pack.target_component_GET();
			float[] controls         = pack.controls_GET();
		});
		CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
		{
			long    time_usec = pack.time_usec_GET();
			char    group_mlx = pack.group_mlx_GET();
			float[] controls  = pack.controls_GET();
		});
		CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
		{
			long  time_usec          = pack.time_usec_GET();
			float altitude_monotonic = pack.altitude_monotonic_GET();
			float altitude_amsl      = pack.altitude_amsl_GET();
			float altitude_local     = pack.altitude_local_GET();
			float altitude_relative  = pack.altitude_relative_GET();
			float altitude_terrain   = pack.altitude_terrain_GET();
			float bottom_clearance   = pack.bottom_clearance_GET();
		});
		CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
		{
			char   request_id    = pack.request_id_GET();
			char   uri_type      = pack.uri_type_GET();
			char[] uri           = pack.uri_GET();
			char   transfer_type = pack.transfer_type_GET();
			char[] storage       = pack.storage_GET();
		});
		CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
		{
			long  time_boot_ms = pack.time_boot_ms_GET();
			float press_abs    = pack.press_abs_GET();
			float press_diff   = pack.press_diff_GET();
			short temperature  = pack.temperature_GET();
		});
		CommunicationChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
		{
			long    timestamp        = pack.timestamp_GET();
			char    est_capabilities = pack.est_capabilities_GET();
			int     lat              = pack.lat_GET();
			int     lon              = pack.lon_GET();
			float   alt              = pack.alt_GET();
			float[] vel              = pack.vel_GET();
			float[] acc              = pack.acc_GET();
			float[] attitude_q       = pack.attitude_q_GET();
			float[] rates            = pack.rates_GET();
			float[] position_cov     = pack.position_cov_GET();
			long    custom_state     = pack.custom_state_GET();
		});
		CommunicationChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
		{
			long    time_usec    = pack.time_usec_GET();
			float   x_acc        = pack.x_acc_GET();
			float   y_acc        = pack.y_acc_GET();
			float   z_acc        = pack.z_acc_GET();
			float   x_vel        = pack.x_vel_GET();
			float   y_vel        = pack.y_vel_GET();
			float   z_vel        = pack.z_vel_GET();
			float   x_pos        = pack.x_pos_GET();
			float   y_pos        = pack.y_pos_GET();
			float   z_pos        = pack.z_pos_GET();
			float   airspeed     = pack.airspeed_GET();
			float[] vel_variance = pack.vel_variance_GET();
			float[] pos_variance = pack.pos_variance_GET();
			float[] q            = pack.q_GET();
			float   roll_rate    = pack.roll_rate_GET();
			float   pitch_rate   = pack.pitch_rate_GET();
			float   yaw_rate     = pack.yaw_rate_GET();
		});
		POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.instance.new_POSITION_TARGET_LOCAL_NED();
		PH.setPack(p3);
		p3.time_boot_ms_SET(3016971342L);
		p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED);
		p3.type_mask_SET((char) 31847);
		p3.x_SET(-2.1122552E38F);
		p3.y_SET(-3.3476152E38F);
		p3.z_SET(-1.8820756E38F);
		p3.vx_SET(3.2350226E38F);
		p3.vy_SET(-1.8048694E38F);
		p3.vz_SET(1.220814E38F);
		p3.afx_SET(8.272634E36F);
		p3.afy_SET(-1.7707562E38F);
		p3.afz_SET(-1.6377037E38F);
		p3.yaw_SET(-3.2732473E38F);
		p3.yaw_rate_SET(1.72409E38F);
		CommunicationChannel.instance.send(p3); //===============================
		MANUAL_SETPOINT p81 = CommunicationChannel.instance.new_MANUAL_SETPOINT();
		PH.setPack(p81);
		p81.time_boot_ms_SET(2614926170L);
		p81.roll_SET(-3.2880788E38F);
		p81.pitch_SET(8.950259E37F);
		p81.yaw_SET(2.6524739E38F);
		p81.thrust_SET(2.290063E38F);
		p81.mode_switch_SET((char) 116);
		p81.manual_override_switch_SET((char) 21);
		CommunicationChannel.instance.send(p81); //===============================
		SET_ATTITUDE_TARGET p82 = CommunicationChannel.instance.new_SET_ATTITUDE_TARGET();
		PH.setPack(p82);
		p82.time_boot_ms_SET(2030245982L);
		p82.target_system_SET((char) 55);
		p82.target_component_SET((char) 124);
		p82.type_mask_SET((char) 216);
		p82.q_SET(new float[4], 0);
		p82.body_roll_rate_SET(-9.884882E37F);
		p82.body_pitch_rate_SET(-3.1773784E38F);
		p82.body_yaw_rate_SET(-1.3205255E38F);
		p82.thrust_SET(9.876765E37F);
		CommunicationChannel.instance.send(p82); //===============================
		ATTITUDE_TARGET p83 = CommunicationChannel.instance.new_ATTITUDE_TARGET();
		PH.setPack(p83);
		p83.time_boot_ms_SET(4280669706L);
		p83.type_mask_SET((char) 74);
		p83.q_SET(new float[4], 0);
		p83.body_roll_rate_SET(-2.7736893E38F);
		p83.body_pitch_rate_SET(-2.3549173E37F);
		p83.body_yaw_rate_SET(2.2495553E38F);
		p83.thrust_SET(1.8266318E37F);
		CommunicationChannel.instance.send(p83); //===============================
		SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.instance.new_SET_POSITION_TARGET_LOCAL_NED();
		PH.setPack(p84);
		p84.time_boot_ms_SET(1554125218L);
		p84.target_system_SET((char) 113);
		p84.target_component_SET((char) 217);
		p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
		p84.type_mask_SET((char) 63852);
		p84.x_SET(-6.8516966E37F);
		p84.y_SET(-3.955357E37F);
		p84.z_SET(-2.8081858E38F);
		p84.vx_SET(-9.591278E37F);
		p84.vy_SET(-2.40625E38F);
		p84.vz_SET(3.1933078E38F);
		p84.afx_SET(3.4897705E36F);
		p84.afy_SET(-2.172177E38F);
		p84.afz_SET(1.8264896E38F);
		p84.yaw_SET(1.8655345E38F);
		p84.yaw_rate_SET(2.2362294E38F);
		CommunicationChannel.instance.send(p84); //===============================
		SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.instance.new_SET_POSITION_TARGET_GLOBAL_INT();
		PH.setPack(p86);
		p86.time_boot_ms_SET(648864951L);
		p86.target_system_SET((char) 52);
		p86.target_component_SET((char) 161);
		p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
		p86.type_mask_SET((char) 49212);
		p86.lat_int_SET(-823580273);
		p86.lon_int_SET(-212788257);
		p86.alt_SET(-1.6385534E37F);
		p86.vx_SET(1.6118357E38F);
		p86.vy_SET(2.2178507E38F);
		p86.vz_SET(-8.1308636E37F);
		p86.afx_SET(2.6830766E38F);
		p86.afy_SET(1.6025859E38F);
		p86.afz_SET(2.8113464E38F);
		p86.yaw_SET(6.6960905E37F);
		p86.yaw_rate_SET(-1.8255696E38F);
		CommunicationChannel.instance.send(p86); //===============================
		POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.instance.new_POSITION_TARGET_GLOBAL_INT();
		PH.setPack(p87);
		p87.time_boot_ms_SET(831228426L);
		p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
		p87.type_mask_SET((char) 38662);
		p87.lat_int_SET(1141249470);
		p87.lon_int_SET(319328821);
		p87.alt_SET(-1.1615277E38F);
		p87.vx_SET(1.1958561E38F);
		p87.vy_SET(-3.6146547E37F);
		p87.vz_SET(-6.2384443E37F);
		p87.afx_SET(1.3931894E38F);
		p87.afy_SET(3.2882983E37F);
		p87.afz_SET(1.5680603E38F);
		p87.yaw_SET(2.7635927E38F);
		p87.yaw_rate_SET(2.733118E38F);
		CommunicationChannel.instance.send(p87); //===============================
		LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.instance.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
		PH.setPack(p89);
		p89.time_boot_ms_SET(3114418266L);
		p89.x_SET(1.7951612E38F);
		p89.y_SET(-6.388703E37F);
		p89.z_SET(-2.6985509E38F);
		p89.roll_SET(2.0238104E36F);
		p89.pitch_SET(1.2284335E38F);
		p89.yaw_SET(-3.2144265E38F);
		CommunicationChannel.instance.send(p89); //===============================
		HIL_STATE p90 = CommunicationChannel.instance.new_HIL_STATE();
		PH.setPack(p90);
		p90.time_usec_SET(4334754068745307473L);
		p90.roll_SET(1.1305143E38F);
		p90.pitch_SET(-9.248842E37F);
		p90.yaw_SET(3.7504556E37F);
		p90.rollspeed_SET(-6.2105585E37F);
		p90.pitchspeed_SET(1.0835432E38F);
		p90.yawspeed_SET(-2.9434263E38F);
		p90.lat_SET(340977367);
		p90.lon_SET(-2035117531);
		p90.alt_SET(1021536986);
		p90.vx_SET((short) 24902);
		p90.vy_SET((short) -9095);
		p90.vz_SET((short) -31420);
		p90.xacc_SET((short) -30805);
		p90.yacc_SET((short) 2913);
		p90.zacc_SET((short) -30348);
		CommunicationChannel.instance.send(p90); //===============================
		HIL_CONTROLS p91 = CommunicationChannel.instance.new_HIL_CONTROLS();
		PH.setPack(p91);
		p91.time_usec_SET(3467317965503556507L);
		p91.roll_ailerons_SET(1.3590784E38F);
		p91.pitch_elevator_SET(5.8831015E37F);
		p91.yaw_rudder_SET(1.5812715E38F);
		p91.throttle_SET(2.633162E38F);
		p91.aux1_SET(-2.1130194E38F);
		p91.aux2_SET(1.1215977E38F);
		p91.aux3_SET(7.0789666E37F);
		p91.aux4_SET(-3.1021337E38F);
		p91.mode_SET(MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
		p91.nav_mode_SET((char) 58);
		CommunicationChannel.instance.send(p91); //===============================
		HIL_RC_INPUTS_RAW p92 = CommunicationChannel.instance.new_HIL_RC_INPUTS_RAW();
		PH.setPack(p92);
		p92.time_usec_SET(4396799933426116117L);
		p92.chan1_raw_SET((char) 1112);
		p92.chan2_raw_SET((char) 38706);
		p92.chan3_raw_SET((char) 12572);
		p92.chan4_raw_SET((char) 31123);
		p92.chan5_raw_SET((char) 56941);
		p92.chan6_raw_SET((char) 51186);
		p92.chan7_raw_SET((char) 24143);
		p92.chan8_raw_SET((char) 37444);
		p92.chan9_raw_SET((char) 29478);
		p92.chan10_raw_SET((char) 18630);
		p92.chan11_raw_SET((char) 44445);
		p92.chan12_raw_SET((char) 39896);
		p92.rssi_SET((char) 179);
		CommunicationChannel.instance.send(p92); //===============================
		HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.instance.new_HIL_ACTUATOR_CONTROLS();
		PH.setPack(p93);
		p93.time_usec_SET(2236427347808230015L);
		p93.controls_SET(new float[16], 0);
		p93.mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED);
		p93.flags_SET(1856105784059968036L);
		CommunicationChannel.instance.send(p93); //===============================
		OPTICAL_FLOW p100 = CommunicationChannel.instance.new_OPTICAL_FLOW();
		PH.setPack(p100);
		p100.time_usec_SET(5496193203043900757L);
		p100.sensor_id_SET((char) 94);
		p100.flow_x_SET((short) -18290);
		p100.flow_y_SET((short) 23382);
		p100.flow_comp_m_x_SET(1.6489884E37F);
		p100.flow_comp_m_y_SET(1.4293263E38F);
		p100.quality_SET((char) 177);
		p100.ground_distance_SET(1.2064574E37F);
		p100.flow_rate_x_SET(2.4065531E38F, PH);
		p100.flow_rate_y_SET(8.619613E37F, PH);
		CommunicationChannel.instance.send(p100); //===============================
		GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.instance.new_GLOBAL_VISION_POSITION_ESTIMATE();
		PH.setPack(p101);
		p101.usec_SET(7212280843199522076L);
		p101.x_SET(1.7875908E38F);
		p101.y_SET(8.793016E37F);
		p101.z_SET(1.2566842E38F);
		p101.roll_SET(2.9135683E38F);
		p101.pitch_SET(2.9109623E38F);
		p101.yaw_SET(-1.8457384E38F);
		CommunicationChannel.instance.send(p101); //===============================
		VISION_POSITION_ESTIMATE p102 = CommunicationChannel.instance.new_VISION_POSITION_ESTIMATE();
		PH.setPack(p102);
		p102.usec_SET(5929872063328121488L);
		p102.x_SET(-2.3158474E38F);
		p102.y_SET(8.5046167E37F);
		p102.z_SET(3.3108158E38F);
		p102.roll_SET(-3.0614715E38F);
		p102.pitch_SET(1.7848038E38F);
		p102.yaw_SET(-3.272097E38F);
		CommunicationChannel.instance.send(p102); //===============================
		VISION_SPEED_ESTIMATE p103 = CommunicationChannel.instance.new_VISION_SPEED_ESTIMATE();
		PH.setPack(p103);
		p103.usec_SET(7287188161735911900L);
		p103.x_SET(2.3289021E38F);
		p103.y_SET(-4.732081E37F);
		p103.z_SET(-3.1374795E38F);
		CommunicationChannel.instance.send(p103); //===============================
		VICON_POSITION_ESTIMATE p104 = CommunicationChannel.instance.new_VICON_POSITION_ESTIMATE();
		PH.setPack(p104);
		p104.usec_SET(3102081563259159335L);
		p104.x_SET(4.8895236E37F);
		p104.y_SET(-3.2548497E38F);
		p104.z_SET(-2.8959155E38F);
		p104.roll_SET(-2.880567E38F);
		p104.pitch_SET(2.349858E37F);
		p104.yaw_SET(-2.4842824E38F);
		CommunicationChannel.instance.send(p104); //===============================
		HIGHRES_IMU p105 = CommunicationChannel.instance.new_HIGHRES_IMU();
		PH.setPack(p105);
		p105.time_usec_SET(5526671312263600930L);
		p105.xacc_SET(5.5828E37F);
		p105.yacc_SET(-4.3176927E36F);
		p105.zacc_SET(-2.237771E38F);
		p105.xgyro_SET(5.631384E37F);
		p105.ygyro_SET(1.03591215E37F);
		p105.zgyro_SET(-2.2643193E38F);
		p105.xmag_SET(1.8390156E38F);
		p105.ymag_SET(1.3977289E38F);
		p105.zmag_SET(2.477411E38F);
		p105.abs_pressure_SET(1.2606794E38F);
		p105.diff_pressure_SET(-1.7390876E38F);
		p105.pressure_alt_SET(-4.240112E37F);
		p105.temperature_SET(-1.722223E38F);
		p105.fields_updated_SET((char) 42004);
		CommunicationChannel.instance.send(p105); //===============================
		OPTICAL_FLOW_RAD p106 = CommunicationChannel.instance.new_OPTICAL_FLOW_RAD();
		PH.setPack(p106);
		p106.time_usec_SET(9121515097999921180L);
		p106.sensor_id_SET((char) 123);
		p106.integration_time_us_SET(3316760750L);
		p106.integrated_x_SET(-4.0864477E37F);
		p106.integrated_y_SET(1.3801594E38F);
		p106.integrated_xgyro_SET(1.1006564E38F);
		p106.integrated_ygyro_SET(2.3208587E38F);
		p106.integrated_zgyro_SET(1.7260941E38F);
		p106.temperature_SET((short) -16831);
		p106.quality_SET((char) 27);
		p106.time_delta_distance_us_SET(1319354105L);
		p106.distance_SET(-2.5348694E37F);
		CommunicationChannel.instance.send(p106); //===============================
		HIL_SENSOR p107 = CommunicationChannel.instance.new_HIL_SENSOR();
		PH.setPack(p107);
		p107.time_usec_SET(2275500442062021836L);
		p107.xacc_SET(1.7805374E38F);
		p107.yacc_SET(2.5264855E38F);
		p107.zacc_SET(-1.6742759E37F);
		p107.xgyro_SET(3.247775E37F);
		p107.ygyro_SET(-9.370038E37F);
		p107.zgyro_SET(-2.8504002E38F);
		p107.xmag_SET(2.6958018E38F);
		p107.ymag_SET(-2.547594E38F);
		p107.zmag_SET(2.0857576E38F);
		p107.abs_pressure_SET(1.7352457E38F);
		p107.diff_pressure_SET(6.5738744E36F);
		p107.pressure_alt_SET(-1.3183223E38F);
		p107.temperature_SET(-1.4519551E38F);
		p107.fields_updated_SET(757736851L);
		CommunicationChannel.instance.send(p107); //===============================
		SIM_STATE p108 = CommunicationChannel.instance.new_SIM_STATE();
		PH.setPack(p108);
		p108.q1_SET(5.0616543E37F);
		p108.q2_SET(1.4899294E38F);
		p108.q3_SET(-1.235985E38F);
		p108.q4_SET(1.1799338E38F);
		p108.roll_SET(-2.2252803E38F);
		p108.pitch_SET(-6.7467646E37F);
		p108.yaw_SET(2.5032676E38F);
		p108.xacc_SET(3.2885138E38F);
		p108.yacc_SET(-2.0604969E38F);
		p108.zacc_SET(1.1949233E38F);
		p108.xgyro_SET(1.6590928E38F);
		p108.ygyro_SET(1.7301218E38F);
		p108.zgyro_SET(3.3231708E38F);
		p108.lat_SET(-3.0226814E38F);
		p108.lon_SET(2.3616748E38F);
		p108.alt_SET(1.6864786E38F);
		p108.std_dev_horz_SET(-3.2634608E38F);
		p108.std_dev_vert_SET(-2.4794696E38F);
		p108.vn_SET(3.9423918E37F);
		p108.ve_SET(-2.2885775E38F);
		p108.vd_SET(7.2589826E37F);
		CommunicationChannel.instance.send(p108); //===============================
		RADIO_STATUS p109 = CommunicationChannel.instance.new_RADIO_STATUS();
		PH.setPack(p109);
		p109.rssi_SET((char) 82);
		p109.remrssi_SET((char) 18);
		p109.txbuf_SET((char) 179);
		p109.noise_SET((char) 230);
		p109.remnoise_SET((char) 14);
		p109.rxerrors_SET((char) 2952);
		p109.fixed__SET((char) 7136);
		CommunicationChannel.instance.send(p109); //===============================
		FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.instance.new_FILE_TRANSFER_PROTOCOL();
		PH.setPack(p110);
		p110.target_network_SET((char) 94);
		p110.target_system_SET((char) 133);
		p110.target_component_SET((char) 95);
		p110.payload_SET(new char[251], 0);
		CommunicationChannel.instance.send(p110); //===============================
		TIMESYNC p111 = CommunicationChannel.instance.new_TIMESYNC();
		PH.setPack(p111);
		p111.tc1_SET(6106130996687229916L);
		p111.ts1_SET(-5189001566711699545L);
		CommunicationChannel.instance.send(p111); //===============================
		CAMERA_TRIGGER p112 = CommunicationChannel.instance.new_CAMERA_TRIGGER();
		PH.setPack(p112);
		p112.time_usec_SET(8627779293136994006L);
		p112.seq_SET(868233317L);
		CommunicationChannel.instance.send(p112); //===============================
		HIL_GPS p113 = CommunicationChannel.instance.new_HIL_GPS();
		PH.setPack(p113);
		p113.time_usec_SET(1859569642172842517L);
		p113.fix_type_SET((char) 72);
		p113.lat_SET(27971973);
		p113.lon_SET(-1850124086);
		p113.alt_SET(1465495234);
		p113.eph_SET((char) 16753);
		p113.epv_SET((char) 10414);
		p113.vel_SET((char) 45103);
		p113.vn_SET((short) -19146);
		p113.ve_SET((short) 8139);
		p113.vd_SET((short) -18825);
		p113.cog_SET((char) 8823);
		p113.satellites_visible_SET((char) 172);
		CommunicationChannel.instance.send(p113); //===============================
		HIL_OPTICAL_FLOW p114 = CommunicationChannel.instance.new_HIL_OPTICAL_FLOW();
		PH.setPack(p114);
		p114.time_usec_SET(7235924987449367195L);
		p114.sensor_id_SET((char) 118);
		p114.integration_time_us_SET(2093409277L);
		p114.integrated_x_SET(-1.3259682E38F);
		p114.integrated_y_SET(7.695292E36F);
		p114.integrated_xgyro_SET(-1.8278849E37F);
		p114.integrated_ygyro_SET(-8.861522E37F);
		p114.integrated_zgyro_SET(-7.75335E37F);
		p114.temperature_SET((short) 14032);
		p114.quality_SET((char) 212);
		p114.time_delta_distance_us_SET(2287856671L);
		p114.distance_SET(-1.2767479E38F);
		CommunicationChannel.instance.send(p114); //===============================
		HIL_STATE_QUATERNION p115 = CommunicationChannel.instance.new_HIL_STATE_QUATERNION();
		PH.setPack(p115);
		p115.time_usec_SET(4781820944787289023L);
		p115.attitude_quaternion_SET(new float[4], 0);
		p115.rollspeed_SET(-2.7675891E38F);
		p115.pitchspeed_SET(-8.3924265E37F);
		p115.yawspeed_SET(8.868494E34F);
		p115.lat_SET(-1831335548);
		p115.lon_SET(-169639911);
		p115.alt_SET(805372652);
		p115.vx_SET((short) -1194);
		p115.vy_SET((short) -26141);
		p115.vz_SET((short) 4628);
		p115.ind_airspeed_SET((char) 58781);
		p115.true_airspeed_SET((char) 62371);
		p115.xacc_SET((short) -8544);
		p115.yacc_SET((short) -32212);
		p115.zacc_SET((short) 2055);
		CommunicationChannel.instance.send(p115); //===============================
		SCALED_IMU2 p116 = CommunicationChannel.instance.new_SCALED_IMU2();
		PH.setPack(p116);
		p116.time_boot_ms_SET(3874871895L);
		p116.xacc_SET((short) -11249);
		p116.yacc_SET((short) -6628);
		p116.zacc_SET((short) 11964);
		p116.xgyro_SET((short) -2677);
		p116.ygyro_SET((short) -10016);
		p116.zgyro_SET((short) 28933);
		p116.xmag_SET((short) 7847);
		p116.ymag_SET((short) 15771);
		p116.zmag_SET((short) 8968);
		CommunicationChannel.instance.send(p116); //===============================
		LOG_REQUEST_LIST p117 = CommunicationChannel.instance.new_LOG_REQUEST_LIST();
		PH.setPack(p117);
		p117.target_system_SET((char) 27);
		p117.target_component_SET((char) 25);
		p117.start_SET((char) 28002);
		p117.end_SET((char) 47233);
		CommunicationChannel.instance.send(p117); //===============================
		LOG_ENTRY p118 = CommunicationChannel.instance.new_LOG_ENTRY();
		PH.setPack(p118);
		p118.id_SET((char) 36729);
		p118.num_logs_SET((char) 7513);
		p118.last_log_num_SET((char) 17085);
		p118.time_utc_SET(463130066L);
		p118.size_SET(1831076696L);
		CommunicationChannel.instance.send(p118); //===============================
		LOG_REQUEST_DATA p119 = CommunicationChannel.instance.new_LOG_REQUEST_DATA();
		PH.setPack(p119);
		p119.target_system_SET((char) 131);
		p119.target_component_SET((char) 38);
		p119.id_SET((char) 24934);
		p119.ofs_SET(1193815034L);
		p119.count_SET(2364743236L);
		CommunicationChannel.instance.send(p119); //===============================
		LOG_DATA p120 = CommunicationChannel.instance.new_LOG_DATA();
		PH.setPack(p120);
		p120.id_SET((char) 24543);
		p120.ofs_SET(2303734344L);
		p120.count_SET((char) 245);
		p120.data__SET(new char[90], 0);
		CommunicationChannel.instance.send(p120); //===============================
		LOG_ERASE p121 = CommunicationChannel.instance.new_LOG_ERASE();
		PH.setPack(p121);
		p121.target_system_SET((char) 95);
		p121.target_component_SET((char) 118);
		CommunicationChannel.instance.send(p121); //===============================
		LOG_REQUEST_END p122 = CommunicationChannel.instance.new_LOG_REQUEST_END();
		PH.setPack(p122);
		p122.target_system_SET((char) 101);
		p122.target_component_SET((char) 120);
		CommunicationChannel.instance.send(p122); //===============================
		GPS_INJECT_DATA p123 = CommunicationChannel.instance.new_GPS_INJECT_DATA();
		PH.setPack(p123);
		p123.target_system_SET((char) 38);
		p123.target_component_SET((char) 186);
		p123.len_SET((char) 28);
		p123.data__SET(new char[110], 0);
		CommunicationChannel.instance.send(p123); //===============================
		GPS2_RAW p124 = CommunicationChannel.instance.new_GPS2_RAW();
		PH.setPack(p124);
		p124.time_usec_SET(8604814200586039921L);
		p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
		p124.lat_SET(-1879015761);
		p124.lon_SET(929402216);
		p124.alt_SET(-2144101534);
		p124.eph_SET((char) 58257);
		p124.epv_SET((char) 35188);
		p124.vel_SET((char) 33964);
		p124.cog_SET((char) 29353);
		p124.satellites_visible_SET((char) 1);
		p124.dgps_numch_SET((char) 186);
		p124.dgps_age_SET(880423525L);
		CommunicationChannel.instance.send(p124); //===============================
		POWER_STATUS p125 = CommunicationChannel.instance.new_POWER_STATUS();
		PH.setPack(p125);
		p125.Vcc_SET((char) 19470);
		p125.Vservo_SET((char) 18548);
		p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT);
		CommunicationChannel.instance.send(p125); //===============================
		SERIAL_CONTROL p126 = CommunicationChannel.instance.new_SERIAL_CONTROL();
		PH.setPack(p126);
		p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL);
		p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI);
		p126.timeout_SET((char) 55707);
		p126.baudrate_SET(1818039898L);
		p126.count_SET((char) 82);
		p126.data__SET(new char[70], 0);
		CommunicationChannel.instance.send(p126); //===============================
		GPS_RTK p127 = CommunicationChannel.instance.new_GPS_RTK();
		PH.setPack(p127);
		p127.time_last_baseline_ms_SET(2690418146L);
		p127.rtk_receiver_id_SET((char) 159);
		p127.wn_SET((char) 28752);
		p127.tow_SET(3891458413L);
		p127.rtk_health_SET((char) 179);
		p127.rtk_rate_SET((char) 67);
		p127.nsats_SET((char) 38);
		p127.baseline_coords_type_SET((char) 144);
		p127.baseline_a_mm_SET(1921388380);
		p127.baseline_b_mm_SET(1318119708);
		p127.baseline_c_mm_SET(-1866416872);
		p127.accuracy_SET(3472183224L);
		p127.iar_num_hypotheses_SET(726334978);
		CommunicationChannel.instance.send(p127); //===============================
		GPS2_RTK p128 = CommunicationChannel.instance.new_GPS2_RTK();
		PH.setPack(p128);
		p128.time_last_baseline_ms_SET(202419349L);
		p128.rtk_receiver_id_SET((char) 198);
		p128.wn_SET((char) 1695);
		p128.tow_SET(595195410L);
		p128.rtk_health_SET((char) 144);
		p128.rtk_rate_SET((char) 10);
		p128.nsats_SET((char) 1);
		p128.baseline_coords_type_SET((char) 9);
		p128.baseline_a_mm_SET(593113591);
		p128.baseline_b_mm_SET(-1526602817);
		p128.baseline_c_mm_SET(1190073511);
		p128.accuracy_SET(515939700L);
		p128.iar_num_hypotheses_SET(335205934);
		CommunicationChannel.instance.send(p128); //===============================
		SCALED_IMU3 p129 = CommunicationChannel.instance.new_SCALED_IMU3();
		PH.setPack(p129);
		p129.time_boot_ms_SET(932427487L);
		p129.xacc_SET((short) 7598);
		p129.yacc_SET((short) 29830);
		p129.zacc_SET((short) 7419);
		p129.xgyro_SET((short) -16370);
		p129.ygyro_SET((short) -15751);
		p129.zgyro_SET((short) -25250);
		p129.xmag_SET((short) 23904);
		p129.ymag_SET((short) 15557);
		p129.zmag_SET((short) 20724);
		CommunicationChannel.instance.send(p129); //===============================
		DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.instance.new_DATA_TRANSMISSION_HANDSHAKE();
		PH.setPack(p130);
		p130.type_SET((char) 35);
		p130.size_SET(2229359046L);
		p130.width_SET((char) 15069);
		p130.height_SET((char) 33599);
		p130.packets_SET((char) 11137);
		p130.payload_SET((char) 80);
		p130.jpg_quality_SET((char) 161);
		CommunicationChannel.instance.send(p130); //===============================
		ENCAPSULATED_DATA p131 = CommunicationChannel.instance.new_ENCAPSULATED_DATA();
		PH.setPack(p131);
		p131.seqnr_SET((char) 57742);
		p131.data__SET(new char[253], 0);
		CommunicationChannel.instance.send(p131); //===============================
		DISTANCE_SENSOR p132 = CommunicationChannel.instance.new_DISTANCE_SENSOR();
		PH.setPack(p132);
		p132.time_boot_ms_SET(1485181018L);
		p132.min_distance_SET((char) 60859);
		p132.max_distance_SET((char) 8466);
		p132.current_distance_SET((char) 33284);
		p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
		p132.id_SET((char) 175);
		p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_PITCH_90);
		p132.covariance_SET((char) 69);
		CommunicationChannel.instance.send(p132); //===============================
		TERRAIN_REQUEST p133 = CommunicationChannel.instance.new_TERRAIN_REQUEST();
		PH.setPack(p133);
		p133.lat_SET(-2058577191);
		p133.lon_SET(15480435);
		p133.grid_spacing_SET((char) 36975);
		p133.mask_SET(8455371874574562502L);
		CommunicationChannel.instance.send(p133); //===============================
		TERRAIN_DATA p134 = CommunicationChannel.instance.new_TERRAIN_DATA();
		PH.setPack(p134);
		p134.lat_SET(-845376082);
		p134.lon_SET(367160211);
		p134.grid_spacing_SET((char) 52661);
		p134.gridbit_SET((char) 120);
		p134.data__SET(new short[16], 0);
		CommunicationChannel.instance.send(p134); //===============================
		TERRAIN_CHECK p135 = CommunicationChannel.instance.new_TERRAIN_CHECK();
		PH.setPack(p135);
		p135.lat_SET(328540255);
		p135.lon_SET(-875872343);
		CommunicationChannel.instance.send(p135); //===============================
		TERRAIN_REPORT p136 = CommunicationChannel.instance.new_TERRAIN_REPORT();
		PH.setPack(p136);
		p136.lat_SET(1227057381);
		p136.lon_SET(426512769);
		p136.spacing_SET((char) 30044);
		p136.terrain_height_SET(2.1049862E38F);
		p136.current_height_SET(3.5767446E37F);
		p136.pending_SET((char) 40790);
		p136.loaded_SET((char) 65474);
		CommunicationChannel.instance.send(p136); //===============================
		SCALED_PRESSURE2 p137 = CommunicationChannel.instance.new_SCALED_PRESSURE2();
		PH.setPack(p137);
		p137.time_boot_ms_SET(2394028150L);
		p137.press_abs_SET(-2.3351773E38F);
		p137.press_diff_SET(-1.2759454E38F);
		p137.temperature_SET((short) -27763);
		CommunicationChannel.instance.send(p137); //===============================
		ATT_POS_MOCAP p138 = CommunicationChannel.instance.new_ATT_POS_MOCAP();
		PH.setPack(p138);
		p138.time_usec_SET(995542362126043302L);
		p138.q_SET(new float[4], 0);
		p138.x_SET(2.8786808E38F);
		p138.y_SET(1.3906877E38F);
		p138.z_SET(3.2328976E38F);
		CommunicationChannel.instance.send(p138); //===============================
		SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.instance.new_SET_ACTUATOR_CONTROL_TARGET();
		PH.setPack(p139);
		p139.time_usec_SET(1240293072080434565L);
		p139.group_mlx_SET((char) 37);
		p139.target_system_SET((char) 88);
		p139.target_component_SET((char) 122);
		p139.controls_SET(new float[8], 0);
		CommunicationChannel.instance.send(p139); //===============================
		ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.instance.new_ACTUATOR_CONTROL_TARGET();
		PH.setPack(p140);
		p140.time_usec_SET(3152419958638897318L);
		p140.group_mlx_SET((char) 185);
		p140.controls_SET(new float[8], 0);
		CommunicationChannel.instance.send(p140); //===============================
		ALTITUDE p141 = CommunicationChannel.instance.new_ALTITUDE();
		PH.setPack(p141);
		p141.time_usec_SET(6333852272228150710L);
		p141.altitude_monotonic_SET(2.8451043E38F);
		p141.altitude_amsl_SET(3.3315105E38F);
		p141.altitude_local_SET(2.7908748E38F);
		p141.altitude_relative_SET(-1.5680518E38F);
		p141.altitude_terrain_SET(-2.6934066E38F);
		p141.bottom_clearance_SET(1.450403E38F);
		CommunicationChannel.instance.send(p141); //===============================
		RESOURCE_REQUEST p142 = CommunicationChannel.instance.new_RESOURCE_REQUEST();
		PH.setPack(p142);
		p142.request_id_SET((char) 228);
		p142.uri_type_SET((char) 1);
		p142.uri_SET(new char[120], 0);
		p142.transfer_type_SET((char) 59);
		p142.storage_SET(new char[120], 0);
		CommunicationChannel.instance.send(p142); //===============================
		SCALED_PRESSURE3 p143 = CommunicationChannel.instance.new_SCALED_PRESSURE3();
		PH.setPack(p143);
		p143.time_boot_ms_SET(3139413535L);
		p143.press_abs_SET(-2.5620764E38F);
		p143.press_diff_SET(2.818941E36F);
		p143.temperature_SET((short) -27680);
		CommunicationChannel.instance.send(p143); //===============================
		FOLLOW_TARGET p144 = CommunicationChannel.instance.new_FOLLOW_TARGET();
		PH.setPack(p144);
		p144.timestamp_SET(6805687266982356320L);
		p144.est_capabilities_SET((char) 208);
		p144.lat_SET(2135364745);
		p144.lon_SET(1254955555);
		p144.alt_SET(-2.616803E38F);
		p144.vel_SET(new float[3], 0);
		p144.acc_SET(new float[3], 0);
		p144.attitude_q_SET(new float[4], 0);
		p144.rates_SET(new float[3], 0);
		p144.position_cov_SET(new float[3], 0);
		p144.custom_state_SET(6401238288361454577L);
		CommunicationChannel.instance.send(p144); //===============================
		CONTROL_SYSTEM_STATE p146 = CommunicationChannel.instance.new_CONTROL_SYSTEM_STATE();
		PH.setPack(p146);
		p146.time_usec_SET(12867772820119237L);
		p146.x_acc_SET(7.776327E37F);
		p146.y_acc_SET(1.2217906E37F);
		p146.z_acc_SET(-3.2725747E38F);
		p146.x_vel_SET(-8.494599E37F);
		p146.y_vel_SET(-1.7553685E38F);
		p146.z_vel_SET(-1.6222571E38F);
		p146.x_pos_SET(-8.250878E37F);
		p146.y_pos_SET(2.1327773E38F);
		p146.z_pos_SET(-3.0363222E38F);
		p146.airspeed_SET(-4.476236E37F);
		p146.vel_variance_SET(new float[3], 0);
		p146.pos_variance_SET(new float[3], 0);
		p146.q_SET(new float[4], 0);
		p146.roll_rate_SET(3.0096078E38F);
		p146.pitch_rate_SET(-3.9865862E37F);
		p146.yaw_rate_SET(-4.675372E37F);
		CommunicationChannel.instance.send(p146); //===============================
		BATTERY_STATUS p147 = CommunicationChannel.instance.new_BATTERY_STATUS();
		PH.setPack(p147);
		p147.id_SET((char) 178);
		p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
		p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
		p147.temperature_SET((short) -19117);
		p147.voltages_SET(new char[10], 0);
		p147.current_battery_SET((short) -6417);
		p147.current_consumed_SET(-688220560);
		p147.energy_consumed_SET(-1658425633);
		p147.battery_remaining_SET((byte) 73);
		CommunicationChannel.instance.send(p147); //===============================
		AUTOPILOT_VERSION p148 = CommunicationChannel.instance.new_AUTOPILOT_VERSION();
		PH.setPack(p148);
		p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION);
		p148.flight_sw_version_SET(3930460891L);
		p148.middleware_sw_version_SET(1313375330L);
		p148.os_sw_version_SET(2996930125L);
		p148.board_version_SET(3873630069L);
		p148.flight_custom_version_SET(new char[8], 0);
		p148.middleware_custom_version_SET(new char[8], 0);
		p148.os_custom_version_SET(new char[8], 0);
		p148.vendor_id_SET((char) 59569);
		p148.product_id_SET((char) 53806);
		p148.uid_SET(2888236619628373454L);
		p148.uid2_SET(new char[18], 0, PH);
		CommunicationChannel.instance.send(p148); //===============================
		LANDING_TARGET p149 = CommunicationChannel.instance.new_LANDING_TARGET();
		PH.setPack(p149);
		p149.time_usec_SET(7117651984151128074L);
		p149.target_num_SET((char) 199);
		p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
		p149.angle_x_SET(-5.346174E37F);
		p149.angle_y_SET(-1.2357895E38F);
		p149.distance_SET(2.4169779E38F);
		p149.size_x_SET(-2.268819E37F);
		p149.size_y_SET(1.1161184E38F);
		p149.x_SET(3.2041404E38F, PH);
		p149.y_SET(1.5082492E38F, PH);
		p149.z_SET(-2.0717236E38F, PH);
		p149.q_SET(new float[4], 0, PH);
		p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
		p149.position_valid_SET((char) 58, PH);
		CommunicationChannel.instance.send(p149); //===============================
		SENS_POWER p201 = CommunicationChannel.instance.new_SENS_POWER();
		PH.setPack(p201);
		p201.adc121_vspb_volt_SET(1.2144045E38F);
		p201.adc121_cspb_amp_SET(-2.6900615E38F);
		p201.adc121_cs1_amp_SET(-2.0684923E37F);
		p201.adc121_cs2_amp_SET(-1.8008038E38F);
		CommunicationChannel.instance.send(p201); //===============================
		SENS_MPPT p202 = CommunicationChannel.instance.new_SENS_MPPT();
		PH.setPack(p202);
		p202.mppt_timestamp_SET(7453860409936602374L);
		p202.mppt1_volt_SET(-7.3171E37F);
		p202.mppt1_amp_SET(1.8529113E38F);
		p202.mppt1_pwm_SET((char) 57789);
		p202.mppt1_status_SET((char) 27);
		p202.mppt2_volt_SET(-5.679979E37F);
		p202.mppt2_amp_SET(-1.089817E38F);
		p202.mppt2_pwm_SET((char) 24832);
		p202.mppt2_status_SET((char) 192);
		p202.mppt3_volt_SET(2.5279665E38F);
		p202.mppt3_amp_SET(3.0586655E38F);
		p202.mppt3_pwm_SET((char) 17448);
		p202.mppt3_status_SET((char) 89);
		CommunicationChannel.instance.send(p202); //===============================
		ASLCTRL_DATA p203 = CommunicationChannel.instance.new_ASLCTRL_DATA();
		PH.setPack(p203);
		p203.timestamp_SET(1000055120018387754L);
		p203.aslctrl_mode_SET((char) 58);
		p203.h_SET(1.3031508E38F);
		p203.hRef_SET(-3.0759974E38F);
		p203.hRef_t_SET(1.2853455E38F);
		p203.PitchAngle_SET(2.6468263E38F);
		p203.PitchAngleRef_SET(-2.8399821E38F);
		p203.q_SET(3.280796E38F);
		p203.qRef_SET(-2.9709832E38F);
		p203.uElev_SET(3.1698388E38F);
		p203.uThrot_SET(-3.023577E38F);
		p203.uThrot2_SET(-4.409295E37F);
		p203.nZ_SET(1.0379998E38F);
		p203.AirspeedRef_SET(-8.1160255E37F);
		p203.SpoilersEngaged_SET((char) 30);
		p203.YawAngle_SET(7.365487E37F);
		p203.YawAngleRef_SET(-2.4360302E38F);
		p203.RollAngle_SET(1.1279072E38F);
		p203.RollAngleRef_SET(2.2729387E38F);
		p203.p_SET(-1.833041E38F);
		p203.pRef_SET(2.9542429E38F);
		p203.r_SET(-2.335431E37F);
		p203.rRef_SET(-3.3165853E38F);
		p203.uAil_SET(1.2517579E38F);
		p203.uRud_SET(1.3049677E38F);
		CommunicationChannel.instance.send(p203); //===============================
		ASLCTRL_DEBUG p204 = CommunicationChannel.instance.new_ASLCTRL_DEBUG();
		PH.setPack(p204);
		p204.i32_1_SET(865979644L);
		p204.i8_1_SET((char) 141);
		p204.i8_2_SET((char) 15);
		p204.f_1_SET(-3.835365E37F);
		p204.f_2_SET(3.1765637E38F);
		p204.f_3_SET(5.820734E37F);
		p204.f_4_SET(4.0297621E37F);
		p204.f_5_SET(-1.3249249E38F);
		p204.f_6_SET(-7.0844205E37F);
		p204.f_7_SET(-1.8469326E38F);
		p204.f_8_SET(1.9173692E38F);
		CommunicationChannel.instance.send(p204); //===============================
		ASLUAV_STATUS p205 = CommunicationChannel.instance.new_ASLUAV_STATUS();
		PH.setPack(p205);
		p205.LED_status_SET((char) 251);
		p205.SATCOM_status_SET((char) 188);
		p205.Servo_status_SET(new char[8], 0);
		p205.Motor_rpm_SET(-3.2665285E38F);
		CommunicationChannel.instance.send(p205); //===============================
		EKF_EXT p206 = CommunicationChannel.instance.new_EKF_EXT();
		PH.setPack(p206);
		p206.timestamp_SET(5853961157713121851L);
		p206.Windspeed_SET(3.1368203E38F);
		p206.WindDir_SET(6.052749E37F);
		p206.WindZ_SET(-1.9788993E38F);
		p206.Airspeed_SET(-1.371789E38F);
		p206.beta_SET(1.1793534E38F);
		p206.alpha_SET(-1.8375591E38F);
		CommunicationChannel.instance.send(p206); //===============================
		ASL_OBCTRL p207 = CommunicationChannel.instance.new_ASL_OBCTRL();
		PH.setPack(p207);
		p207.timestamp_SET(2911658659888352726L);
		p207.uElev_SET(-3.3008721E38F);
		p207.uThrot_SET(3.2431396E38F);
		p207.uThrot2_SET(-1.2810937E37F);
		p207.uAilL_SET(-1.3258878E38F);
		p207.uAilR_SET(6.9066214E37F);
		p207.uRud_SET(2.7605426E38F);
		p207.obctrl_status_SET((char) 158);
		CommunicationChannel.instance.send(p207); //===============================
		SENS_ATMOS p208 = CommunicationChannel.instance.new_SENS_ATMOS();
		PH.setPack(p208);
		p208.TempAmbient_SET(-2.5732818E38F);
		p208.Humidity_SET(-4.387983E37F);
		CommunicationChannel.instance.send(p208); //===============================
		SENS_BATMON p209 = CommunicationChannel.instance.new_SENS_BATMON();
		PH.setPack(p209);
		p209.temperature_SET(-2.831328E38F);
		p209.voltage_SET((char) 1089);
		p209.current_SET((short) -11726);
		p209.SoC_SET((char) 1);
		p209.batterystatus_SET((char) 46009);
		p209.serialnumber_SET((char) 46484);
		p209.hostfetcontrol_SET((char) 63093);
		p209.cellvoltage1_SET((char) 8665);
		p209.cellvoltage2_SET((char) 56045);
		p209.cellvoltage3_SET((char) 42552);
		p209.cellvoltage4_SET((char) 37871);
		p209.cellvoltage5_SET((char) 32149);
		p209.cellvoltage6_SET((char) 18913);
		CommunicationChannel.instance.send(p209); //===============================
		FW_SOARING_DATA p210 = CommunicationChannel.instance.new_FW_SOARING_DATA();
		PH.setPack(p210);
		p210.timestamp_SET(4117983712362289082L);
		p210.timestampModeChanged_SET(2058381298788218372L);
		p210.xW_SET(1.2365441E38F);
		p210.xR_SET(3.1480762E38F);
		p210.xLat_SET(-2.0964112E38F);
		p210.xLon_SET(-1.757031E38F);
		p210.VarW_SET(2.992598E38F);
		p210.VarR_SET(-5.489331E37F);
		p210.VarLat_SET(1.720152E38F);
		p210.VarLon_SET(1.9409122E38F);
		p210.LoiterRadius_SET(9.2383466E36F);
		p210.LoiterDirection_SET(2.444553E38F);
		p210.DistToSoarPoint_SET(-6.897659E37F);
		p210.vSinkExp_SET(2.7023295E38F);
		p210.z1_LocalUpdraftSpeed_SET(-2.033655E38F);
		p210.z2_DeltaRoll_SET(-1.8448188E38F);
		p210.z1_exp_SET(3.1181035E38F);
		p210.z2_exp_SET(-2.435245E37F);
		p210.ThermalGSNorth_SET(-9.979966E36F);
		p210.ThermalGSEast_SET(-1.756285E38F);
		p210.TSE_dot_SET(3.0515577E38F);
		p210.DebugVar1_SET(2.9913033E38F);
		p210.DebugVar2_SET(8.456302E37F);
		p210.ControlMode_SET((char) 76);
		p210.valid_SET((char) 173);
		CommunicationChannel.instance.send(p210); //===============================
		SENSORPOD_STATUS p211 = CommunicationChannel.instance.new_SENSORPOD_STATUS();
		PH.setPack(p211);
		p211.timestamp_SET(8781014483918587255L);
		p211.visensor_rate_1_SET((char) 61);
		p211.visensor_rate_2_SET((char) 185);
		p211.visensor_rate_3_SET((char) 182);
		p211.visensor_rate_4_SET((char) 237);
		p211.recording_nodes_count_SET((char) 183);
		p211.cpu_temp_SET((char) 5);
		p211.free_space_SET((char) 27604);
		CommunicationChannel.instance.send(p211); //===============================
		SENS_POWER_BOARD p212 = CommunicationChannel.instance.new_SENS_POWER_BOARD();
		PH.setPack(p212);
		p212.timestamp_SET(1264249200644222134L);
		p212.pwr_brd_status_SET((char) 46);
		p212.pwr_brd_led_status_SET((char) 179);
		p212.pwr_brd_system_volt_SET(3.1562253E38F);
		p212.pwr_brd_servo_volt_SET(1.8453232E38F);
		p212.pwr_brd_mot_l_amp_SET(-2.2667236E38F);
		p212.pwr_brd_mot_r_amp_SET(-2.748869E38F);
		p212.pwr_brd_servo_1_amp_SET(1.4103464E38F);
		p212.pwr_brd_servo_2_amp_SET(2.516615E38F);
		p212.pwr_brd_servo_3_amp_SET(-1.783444E38F);
		p212.pwr_brd_servo_4_amp_SET(-1.6559286E37F);
		p212.pwr_brd_aux_amp_SET(-7.979071E37F);
		CommunicationChannel.instance.send(p212); //===============================
		ESTIMATOR_STATUS p230 = CommunicationChannel.instance.new_ESTIMATOR_STATUS();
		PH.setPack(p230);
		p230.time_usec_SET(8071257142883296588L);
		p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_GPS_GLITCH);
		p230.vel_ratio_SET(-2.2407576E38F);
		p230.pos_horiz_ratio_SET(-1.7628645E38F);
		p230.pos_vert_ratio_SET(8.4457855E36F);
		p230.mag_ratio_SET(2.2573484E38F);
		p230.hagl_ratio_SET(9.462089E36F);
		p230.tas_ratio_SET(9.474999E37F);
		p230.pos_horiz_accuracy_SET(-2.7341475E38F);
		p230.pos_vert_accuracy_SET(-1.6140344E38F);
		CommunicationChannel.instance.send(p230); //===============================
		WIND_COV p231 = CommunicationChannel.instance.new_WIND_COV();
		PH.setPack(p231);
		p231.time_usec_SET(317299493316411300L);
		p231.wind_x_SET(-1.352463E37F);
		p231.wind_y_SET(2.3531475E38F);
		p231.wind_z_SET(2.6317708E38F);
		p231.var_horiz_SET(-2.1176462E37F);
		p231.var_vert_SET(-5.931727E37F);
		p231.wind_alt_SET(-1.2069514E38F);
		p231.horiz_accuracy_SET(-2.4853077E38F);
		p231.vert_accuracy_SET(-1.5444164E38F);
		CommunicationChannel.instance.send(p231); //===============================
		GPS_INPUT p232 = CommunicationChannel.instance.new_GPS_INPUT();
		PH.setPack(p232);
		p232.time_usec_SET(3517587150526300912L);
		p232.gps_id_SET((char) 118);
		p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP);
		p232.time_week_ms_SET(205612949L);
		p232.time_week_SET((char) 55390);
		p232.fix_type_SET((char) 26);
		p232.lat_SET(1057418013);
		p232.lon_SET(-1784821854);
		p232.alt_SET(-2.673085E37F);
		p232.hdop_SET(-1.9929461E38F);
		p232.vdop_SET(2.290162E37F);
		p232.vn_SET(2.2233782E38F);
		p232.ve_SET(-5.7162873E37F);
		p232.vd_SET(2.4311032E38F);
		p232.speed_accuracy_SET(-2.3496303E38F);
		p232.horiz_accuracy_SET(3.0239963E38F);
		p232.vert_accuracy_SET(-3.0940889E38F);
		p232.satellites_visible_SET((char) 49);
		CommunicationChannel.instance.send(p232); //===============================
		GPS_RTCM_DATA p233 = CommunicationChannel.instance.new_GPS_RTCM_DATA();
		PH.setPack(p233);
		p233.flags_SET((char) 6);
		p233.len_SET((char) 128);
		p233.data__SET(new char[180], 0);
		CommunicationChannel.instance.send(p233); //===============================
		HIGH_LATENCY p234 = CommunicationChannel.instance.new_HIGH_LATENCY();
		PH.setPack(p234);
		p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED);
		p234.custom_mode_SET(3821671948L);
		p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
		p234.roll_SET((short) 11147);
		p234.pitch_SET((short) -18369);
		p234.heading_SET((char) 47406);
		p234.throttle_SET((byte) -3);
		p234.heading_sp_SET((short) 5028);
		p234.latitude_SET(385494395);
		p234.longitude_SET(-2089575423);
		p234.altitude_amsl_SET((short) 18871);
		p234.altitude_sp_SET((short) -32231);
		p234.airspeed_SET((char) 128);
		p234.airspeed_sp_SET((char) 8);
		p234.groundspeed_SET((char) 78);
		p234.climb_rate_SET((byte) 69);
		p234.gps_nsat_SET((char) 164);
		p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
		p234.battery_remaining_SET((char) 189);
		p234.temperature_SET((byte) 110);
		p234.temperature_air_SET((byte) -16);
		p234.failsafe_SET((char) 187);
		p234.wp_num_SET((char) 46);
		p234.wp_distance_SET((char) 48114);
		CommunicationChannel.instance.send(p234); //===============================
		VIBRATION p241 = CommunicationChannel.instance.new_VIBRATION();
		PH.setPack(p241);
		p241.time_usec_SET(2598301318614917370L);
		p241.vibration_x_SET(-1.2238551E38F);
		p241.vibration_y_SET(-1.4424481E38F);
		p241.vibration_z_SET(-2.1444079E38F);
		p241.clipping_0_SET(2371243110L);
		p241.clipping_1_SET(1935620340L);
		p241.clipping_2_SET(272262556L);
		CommunicationChannel.instance.send(p241); //===============================
		HOME_POSITION p242 = CommunicationChannel.instance.new_HOME_POSITION();
		PH.setPack(p242);
		p242.latitude_SET(165050369);
		p242.longitude_SET(668024242);
		p242.altitude_SET(2079591936);
		p242.x_SET(2.7796036E37F);
		p242.y_SET(-2.5307827E38F);
		p242.z_SET(4.015355E37F);
		p242.q_SET(new float[4], 0);
		p242.approach_x_SET(-2.4204271E38F);
		p242.approach_y_SET(-2.3530897E38F);
		p242.approach_z_SET(3.3800646E38F);
		p242.time_usec_SET(8424950513650698535L, PH);
		CommunicationChannel.instance.send(p242); //===============================
		SET_HOME_POSITION p243 = CommunicationChannel.instance.new_SET_HOME_POSITION();
		PH.setPack(p243);
		p243.target_system_SET((char) 152);
		p243.latitude_SET(-149299039);
		p243.longitude_SET(-1782432370);
		p243.altitude_SET(2034122365);
		p243.x_SET(2.5035596E38F);
		p243.y_SET(-5.1565775E37F);
		p243.z_SET(5.550448E37F);
		p243.q_SET(new float[4], 0);
		p243.approach_x_SET(2.7554745E38F);
		p243.approach_y_SET(-2.9169979E38F);
		p243.approach_z_SET(-1.2226441E38F);
		p243.time_usec_SET(2666356079721568920L, PH);
		CommunicationChannel.instance.send(p243); //===============================
		MESSAGE_INTERVAL p244 = CommunicationChannel.instance.new_MESSAGE_INTERVAL();
		PH.setPack(p244);
		p244.message_id_SET((char) 9122);
		p244.interval_us_SET(385671164);
		CommunicationChannel.instance.send(p244); //===============================
		EXTENDED_SYS_STATE p245 = CommunicationChannel.instance.new_EXTENDED_SYS_STATE();
		PH.setPack(p245);
		p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
		p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
		CommunicationChannel.instance.send(p245); //===============================
		ADSB_VEHICLE p246 = CommunicationChannel.instance.new_ADSB_VEHICLE();
		PH.setPack(p246);
		p246.ICAO_address_SET(338838464L);
		p246.lat_SET(1084192218);
		p246.lon_SET(-1929715297);
		p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
		p246.altitude_SET(1628690176);
		p246.heading_SET((char) 36072);
		p246.hor_velocity_SET((char) 23694);
		p246.ver_velocity_SET((short) 21517);
		p246.callsign_SET("DEMO", PH);
		p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HEAVY);
		p246.tslc_SET((char) 22);
		p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN);
		p246.squawk_SET((char) 46972);
		CommunicationChannel.instance.send(p246); //===============================
		COLLISION p247 = CommunicationChannel.instance.new_COLLISION();
		PH.setPack(p247);
		p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
		p247.id_SET(2212624485L);
		p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR);
		p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
		p247.time_to_minimum_delta_SET(8.912987E36F);
		p247.altitude_minimum_delta_SET(1.3166841E37F);
		p247.horizontal_minimum_delta_SET(8.329552E37F);
		CommunicationChannel.instance.send(p247); //===============================
		V2_EXTENSION p248 = CommunicationChannel.instance.new_V2_EXTENSION();
		PH.setPack(p248);
		p248.target_network_SET((char) 253);
		p248.target_system_SET((char) 136);
		p248.target_component_SET((char) 43);
		p248.message_type_SET((char) 6119);
		p248.payload_SET(new char[249], 0);
		CommunicationChannel.instance.send(p248); //===============================
		MEMORY_VECT p249 = CommunicationChannel.instance.new_MEMORY_VECT();
		PH.setPack(p249);
		p249.address_SET((char) 3797);
		p249.ver_SET((char) 165);
		p249.type_SET((char) 104);
		p249.value_SET(new byte[32], 0);
		CommunicationChannel.instance.send(p249); //===============================
		DEBUG_VECT p250 = CommunicationChannel.instance.new_DEBUG_VECT();
		PH.setPack(p250);
		p250.name_SET("DEMO", PH);
		p250.time_usec_SET(6987463802200419130L);
		p250.x_SET(1.2134924E38F);
		p250.y_SET(1.3740019E38F);
		p250.z_SET(-2.4638256E38F);
		CommunicationChannel.instance.send(p250); //===============================
		NAMED_VALUE_FLOAT p251 = CommunicationChannel.instance.new_NAMED_VALUE_FLOAT();
		PH.setPack(p251);
		p251.time_boot_ms_SET(1580363106L);
		p251.name_SET("DEMO", PH);
		p251.value_SET(-2.3429225E37F);
		CommunicationChannel.instance.send(p251); //===============================
		NAMED_VALUE_INT p252 = CommunicationChannel.instance.new_NAMED_VALUE_INT();
		PH.setPack(p252);
		p252.time_boot_ms_SET(2099077818L);
		p252.name_SET("DEMO", PH);
		p252.value_SET(-90229351);
		CommunicationChannel.instance.send(p252); //===============================
		STATUSTEXT p253 = CommunicationChannel.instance.new_STATUSTEXT();
		PH.setPack(p253);
		p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_DEBUG);
		p253.text_SET("DEMO", PH);
		CommunicationChannel.instance.send(p253); //===============================
		DEBUG p254 = CommunicationChannel.instance.new_DEBUG();
		PH.setPack(p254);
		p254.time_boot_ms_SET(1571767808L);
		p254.ind_SET((char) 254);
		p254.value_SET(-1.4855365E38F);
		CommunicationChannel.instance.send(p254); //===============================
		SETUP_SIGNING p256 = CommunicationChannel.instance.new_SETUP_SIGNING();
		PH.setPack(p256);
		p256.target_system_SET((char) 146);
		p256.target_component_SET((char) 25);
		p256.secret_key_SET(new char[32], 0);
		p256.initial_timestamp_SET(431909313149307940L);
		CommunicationChannel.instance.send(p256); //===============================
		BUTTON_CHANGE p257 = CommunicationChannel.instance.new_BUTTON_CHANGE();
		PH.setPack(p257);
		p257.time_boot_ms_SET(2432599966L);
		p257.last_change_ms_SET(145800350L);
		p257.state_SET((char) 141);
		CommunicationChannel.instance.send(p257); //===============================
		PLAY_TUNE p258 = CommunicationChannel.instance.new_PLAY_TUNE();
		PH.setPack(p258);
		p258.target_system_SET((char) 185);
		p258.target_component_SET((char) 245);
		p258.tune_SET("DEMO", PH);
		CommunicationChannel.instance.send(p258); //===============================
		CAMERA_INFORMATION p259 = CommunicationChannel.instance.new_CAMERA_INFORMATION();
		PH.setPack(p259);
		p259.time_boot_ms_SET(219399957L);
		p259.vendor_name_SET(new char[32], 0);
		p259.model_name_SET(new char[32], 0);
		p259.firmware_version_SET(2178570185L);
		p259.focal_length_SET(2.5565772E38F);
		p259.sensor_size_h_SET(-1.6091902E38F);
		p259.sensor_size_v_SET(-4.2139208E37F);
		p259.resolution_h_SET((char) 30910);
		p259.resolution_v_SET((char) 20614);
		p259.lens_id_SET((char) 192);
		p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE);
		p259.cam_definition_version_SET((char) 6510);
		p259.cam_definition_uri_SET("DEMO", PH);
		CommunicationChannel.instance.send(p259); //===============================
		CAMERA_SETTINGS p260 = CommunicationChannel.instance.new_CAMERA_SETTINGS();
		PH.setPack(p260);
		p260.time_boot_ms_SET(128048029L);
		p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_VIDEO);
		CommunicationChannel.instance.send(p260); //===============================
		STORAGE_INFORMATION p261 = CommunicationChannel.instance.new_STORAGE_INFORMATION();
		PH.setPack(p261);
		p261.time_boot_ms_SET(3570807968L);
		p261.storage_id_SET((char) 206);
		p261.storage_count_SET((char) 167);
		p261.status_SET((char) 177);
		p261.total_capacity_SET(-2.336988E38F);
		p261.used_capacity_SET(2.1467835E37F);
		p261.available_capacity_SET(1.3726899E38F);
		p261.read_speed_SET(2.9099143E38F);
		p261.write_speed_SET(2.241846E38F);
		CommunicationChannel.instance.send(p261); //===============================
		CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.instance.new_CAMERA_CAPTURE_STATUS();
		PH.setPack(p262);
		p262.time_boot_ms_SET(3914638317L);
		p262.image_status_SET((char) 130);
		p262.video_status_SET((char) 245);
		p262.image_interval_SET(-2.1445012E38F);
		p262.recording_time_ms_SET(4010536702L);
		p262.available_capacity_SET(-4.4755434E37F);
		CommunicationChannel.instance.send(p262); //===============================
		CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.instance.new_CAMERA_IMAGE_CAPTURED();
		PH.setPack(p263);
		p263.time_boot_ms_SET(716514677L);
		p263.time_utc_SET(8348154039501141972L);
		p263.camera_id_SET((char) 138);
		p263.lat_SET(-887293109);
		p263.lon_SET(543966424);
		p263.alt_SET(1876396411);
		p263.relative_alt_SET(981807023);
		p263.q_SET(new float[4], 0);
		p263.image_index_SET(-1460743747);
		p263.capture_result_SET((byte) -37);
		p263.file_url_SET("DEMO", PH);
		CommunicationChannel.instance.send(p263); //===============================
		FLIGHT_INFORMATION p264 = CommunicationChannel.instance.new_FLIGHT_INFORMATION();
		PH.setPack(p264);
		p264.time_boot_ms_SET(1223848914L);
		p264.arming_time_utc_SET(5425289034697284948L);
		p264.takeoff_time_utc_SET(7547670757216189668L);
		p264.flight_uuid_SET(3932054317829513203L);
		CommunicationChannel.instance.send(p264); //===============================
		MOUNT_ORIENTATION p265 = CommunicationChannel.instance.new_MOUNT_ORIENTATION();
		PH.setPack(p265);
		p265.time_boot_ms_SET(1109942870L);
		p265.roll_SET(2.8708686E38F);
		p265.pitch_SET(-1.7984086E38F);
		p265.yaw_SET(-1.1644462E38F);
		CommunicationChannel.instance.send(p265); //===============================
		LOGGING_DATA p266 = CommunicationChannel.instance.new_LOGGING_DATA();
		PH.setPack(p266);
		p266.target_system_SET((char) 26);
		p266.target_component_SET((char) 238);
		p266.sequence_SET((char) 26658);
		p266.length_SET((char) 35);
		p266.first_message_offset_SET((char) 105);
		p266.data__SET(new char[249], 0);
		CommunicationChannel.instance.send(p266); //===============================
		LOGGING_DATA_ACKED p267 = CommunicationChannel.instance.new_LOGGING_DATA_ACKED();
		PH.setPack(p267);
		p267.target_system_SET((char) 109);
		p267.target_component_SET((char) 83);
		p267.sequence_SET((char) 59419);
		p267.length_SET((char) 55);
		p267.first_message_offset_SET((char) 131);
		p267.data__SET(new char[249], 0);
		CommunicationChannel.instance.send(p267); //===============================
		LOGGING_ACK p268 = CommunicationChannel.instance.new_LOGGING_ACK();
		PH.setPack(p268);
		p268.target_system_SET((char) 173);
		p268.target_component_SET((char) 208);
		p268.sequence_SET((char) 29587);
		CommunicationChannel.instance.send(p268); //===============================
		VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.instance.new_VIDEO_STREAM_INFORMATION();
		PH.setPack(p269);
		p269.camera_id_SET((char) 200);
		p269.status_SET((char) 105);
		p269.framerate_SET(2.5273475E38F);
		p269.resolution_h_SET((char) 64101);
		p269.resolution_v_SET((char) 43315);
		p269.bitrate_SET(3917555493L);
		p269.rotation_SET((char) 12232);
		p269.uri_SET("DEMO", PH);
		CommunicationChannel.instance.send(p269); //===============================
		SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.instance.new_SET_VIDEO_STREAM_SETTINGS();
		PH.setPack(p270);
		p270.target_system_SET((char) 107);
		p270.target_component_SET((char) 220);
		p270.camera_id_SET((char) 90);
		p270.framerate_SET(8.479903E37F);
		p270.resolution_h_SET((char) 38232);
		p270.resolution_v_SET((char) 19987);
		p270.bitrate_SET(1056802925L);
		p270.rotation_SET((char) 63070);
		p270.uri_SET("DEMO", PH);
		CommunicationChannel.instance.send(p270); //===============================
		WIFI_CONFIG_AP p299 = CommunicationChannel.instance.new_WIFI_CONFIG_AP();
		PH.setPack(p299);
		p299.ssid_SET("DEMO", PH);
		p299.password_SET("DEMO", PH);
		CommunicationChannel.instance.send(p299); //===============================
		PROTOCOL_VERSION p300 = CommunicationChannel.instance.new_PROTOCOL_VERSION();
		PH.setPack(p300);
		p300.version_SET((char) 10067);
		p300.min_version_SET((char) 10753);
		p300.max_version_SET((char) 12569);
		p300.spec_version_hash_SET(new char[8], 0);
		p300.library_version_hash_SET(new char[8], 0);
		CommunicationChannel.instance.send(p300); //===============================
		UAVCAN_NODE_STATUS p310 = CommunicationChannel.instance.new_UAVCAN_NODE_STATUS();
		PH.setPack(p310);
		p310.time_usec_SET(5543054456431125998L);
		p310.uptime_sec_SET(1884841577L);
		p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
		p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_MAINTENANCE);
		p310.sub_mode_SET((char) 155);
		p310.vendor_specific_status_code_SET((char) 30080);
		CommunicationChannel.instance.send(p310); //===============================
		UAVCAN_NODE_INFO p311 = CommunicationChannel.instance.new_UAVCAN_NODE_INFO();
		PH.setPack(p311);
		p311.time_usec_SET(1797780547564628503L);
		p311.uptime_sec_SET(1199136063L);
		p311.name_SET("DEMO", PH);
		p311.hw_version_major_SET((char) 90);
		p311.hw_version_minor_SET((char) 169);
		p311.hw_unique_id_SET(new char[16], 0);
		p311.sw_version_major_SET((char) 222);
		p311.sw_version_minor_SET((char) 63);
		p311.sw_vcs_commit_SET(276866849L);
		CommunicationChannel.instance.send(p311); //===============================
		PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_READ();
		PH.setPack(p320);
		p320.target_system_SET((char) 65);
		p320.target_component_SET((char) 234);
		p320.param_id_SET("DEMO", PH);
		p320.param_index_SET((short) 3915);
		CommunicationChannel.instance.send(p320); //===============================
		PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_LIST();
		PH.setPack(p321);
		p321.target_system_SET((char) 38);
		p321.target_component_SET((char) 130);
		CommunicationChannel.instance.send(p321); //===============================
		PARAM_EXT_VALUE p322 = CommunicationChannel.instance.new_PARAM_EXT_VALUE();
		PH.setPack(p322);
		p322.param_id_SET("DEMO", PH);
		p322.param_value_SET("DEMO", PH);
		p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64);
		p322.param_count_SET((char) 42526);
		p322.param_index_SET((char) 60012);
		CommunicationChannel.instance.send(p322); //===============================
		PARAM_EXT_SET p323 = CommunicationChannel.instance.new_PARAM_EXT_SET();
		PH.setPack(p323);
		p323.target_system_SET((char) 230);
		p323.target_component_SET((char) 84);
		p323.param_id_SET("DEMO", PH);
		p323.param_value_SET("DEMO", PH);
		p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
		CommunicationChannel.instance.send(p323); //===============================
		PARAM_EXT_ACK p324 = CommunicationChannel.instance.new_PARAM_EXT_ACK();
		PH.setPack(p324);
		p324.param_id_SET("DEMO", PH);
		p324.param_value_SET("DEMO", PH);
		p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
		p324.param_result_SET(PARAM_ACK.PARAM_ACK_IN_PROGRESS);
		CommunicationChannel.instance.send(p324); //===============================
		OBSTACLE_DISTANCE p330 = CommunicationChannel.instance.new_OBSTACLE_DISTANCE();
		PH.setPack(p330);
		p330.time_usec_SET(1347623605244823338L);
		p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
		p330.distances_SET(new char[72], 0);
		p330.increment_SET((char) 88);
		p330.min_distance_SET((char) 50135);
		p330.max_distance_SET((char) 52186);
		CommunicationChannel.instance.send(p330); //===============================
	}
}
