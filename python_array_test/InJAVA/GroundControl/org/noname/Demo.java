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
		POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.instance.new_POSITION_TARGET_LOCAL_NED();
		PH.setPack(p3);
		p3.time_boot_ms_SET(800576941L);
		p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
		p3.type_mask_SET((char) 32154);
		p3.x_SET(-1.6952565E38F);
		p3.y_SET(2.1485406E38F);
		p3.z_SET(-3.2610553E38F);
		p3.vx_SET(6.6520817E37F);
		p3.vy_SET(-2.8504704E38F);
		p3.vz_SET(-1.8707142E38F);
		p3.afx_SET(1.4289018E38F);
		p3.afy_SET(-1.6130757E38F);
		p3.afz_SET(-1.7636488E38F);
		p3.yaw_SET(-1.7800544E38F);
		p3.yaw_rate_SET(9.068648E37F);
		CommunicationChannel.instance.send(p3); //===============================
		COMMAND_LONG p76 = CommunicationChannel.instance.new_COMMAND_LONG();
		PH.setPack(p76);
		p76.target_system_SET((char) 107);
		p76.target_component_SET((char) 238);
		p76.command_SET(MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD);
		p76.confirmation_SET((char) 213);
		p76.param1_SET(2.6628772E38F);
		p76.param2_SET(-2.9532164E38F);
		p76.param3_SET(1.0634542E38F);
		p76.param4_SET(-1.7403393E38F);
		p76.param5_SET(1.0935576E38F);
		p76.param6_SET(-3.1506362E38F);
		p76.param7_SET(-7.0572563E37F);
		CommunicationChannel.instance.send(p76); //===============================
		COMMAND_ACK p77 = CommunicationChannel.instance.new_COMMAND_ACK();
		PH.setPack(p77);
		p77.command_SET(MAV_CMD.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE);
		p77.result_SET(MAV_RESULT.MAV_RESULT_DENIED);
		p77.progress_SET((char) 130, PH);
		p77.result_param2_SET(2101190789, PH);
		p77.target_system_SET((char) 242, PH);
		p77.target_component_SET((char) 224, PH);
		CommunicationChannel.instance.send(p77); //===============================
		MANUAL_SETPOINT p81 = CommunicationChannel.instance.new_MANUAL_SETPOINT();
		PH.setPack(p81);
		p81.time_boot_ms_SET(3139021841L);
		p81.roll_SET(2.3625348E38F);
		p81.pitch_SET(-2.6963555E38F);
		p81.yaw_SET(-2.2765847E38F);
		p81.thrust_SET(-2.179213E38F);
		p81.mode_switch_SET((char) 132);
		p81.manual_override_switch_SET((char) 111);
		CommunicationChannel.instance.send(p81); //===============================
		SET_ATTITUDE_TARGET p82 = CommunicationChannel.instance.new_SET_ATTITUDE_TARGET();
		PH.setPack(p82);
		p82.time_boot_ms_SET(2163298210L);
		p82.target_system_SET((char) 64);
		p82.target_component_SET((char) 42);
		p82.type_mask_SET((char) 230);
		p82.q_SET(new float[4], 0);
		p82.body_roll_rate_SET(1.933792E38F);
		p82.body_pitch_rate_SET(-6.2314884E37F);
		p82.body_yaw_rate_SET(1.4099636E38F);
		p82.thrust_SET(1.1537594E38F);
		CommunicationChannel.instance.send(p82); //===============================
		ATTITUDE_TARGET p83 = CommunicationChannel.instance.new_ATTITUDE_TARGET();
		PH.setPack(p83);
		p83.time_boot_ms_SET(3880751570L);
		p83.type_mask_SET((char) 70);
		p83.q_SET(new float[4], 0);
		p83.body_roll_rate_SET(-2.5970763E38F);
		p83.body_pitch_rate_SET(-1.9028876E37F);
		p83.body_yaw_rate_SET(2.6381458E38F);
		p83.thrust_SET(-2.9862725E38F);
		CommunicationChannel.instance.send(p83); //===============================
		SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.instance.new_SET_POSITION_TARGET_LOCAL_NED();
		PH.setPack(p84);
		p84.time_boot_ms_SET(3045355374L);
		p84.target_system_SET((char) 88);
		p84.target_component_SET((char) 7);
		p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
		p84.type_mask_SET((char) 9209);
		p84.x_SET(3.1923647E38F);
		p84.y_SET(1.7251463E38F);
		p84.z_SET(2.6558286E38F);
		p84.vx_SET(-5.850846E36F);
		p84.vy_SET(3.0779885E38F);
		p84.vz_SET(-8.0065334E37F);
		p84.afx_SET(2.9562592E38F);
		p84.afy_SET(2.712576E38F);
		p84.afz_SET(-2.4469291E38F);
		p84.yaw_SET(-1.0397016E38F);
		p84.yaw_rate_SET(2.2634267E38F);
		CommunicationChannel.instance.send(p84); //===============================
		SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.instance.new_SET_POSITION_TARGET_GLOBAL_INT();
		PH.setPack(p86);
		p86.time_boot_ms_SET(2069404642L);
		p86.target_system_SET((char) 243);
		p86.target_component_SET((char) 56);
		p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED);
		p86.type_mask_SET((char) 37504);
		p86.lat_int_SET(-736601941);
		p86.lon_int_SET(-1028323859);
		p86.alt_SET(-3.315935E38F);
		p86.vx_SET(6.315448E37F);
		p86.vy_SET(-9.688993E37F);
		p86.vz_SET(-1.2286693E38F);
		p86.afx_SET(2.280371E38F);
		p86.afy_SET(-3.3243129E38F);
		p86.afz_SET(-9.378646E37F);
		p86.yaw_SET(1.2382166E38F);
		p86.yaw_rate_SET(-3.2602971E38F);
		CommunicationChannel.instance.send(p86); //===============================
		POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.instance.new_POSITION_TARGET_GLOBAL_INT();
		PH.setPack(p87);
		p87.time_boot_ms_SET(882383205L);
		p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
		p87.type_mask_SET((char) 12858);
		p87.lat_int_SET(-1244247105);
		p87.lon_int_SET(-1054593978);
		p87.alt_SET(-1.011536E38F);
		p87.vx_SET(1.9342778E38F);
		p87.vy_SET(-2.7946676E38F);
		p87.vz_SET(-1.474199E38F);
		p87.afx_SET(2.0109632E38F);
		p87.afy_SET(3.330128E38F);
		p87.afz_SET(7.4243725E37F);
		p87.yaw_SET(-1.2490128E38F);
		p87.yaw_rate_SET(5.041942E37F);
		CommunicationChannel.instance.send(p87); //===============================
		LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.instance.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
		PH.setPack(p89);
		p89.time_boot_ms_SET(1282373999L);
		p89.x_SET(-2.3790875E38F);
		p89.y_SET(1.5158424E38F);
		p89.z_SET(-3.3891773E38F);
		p89.roll_SET(1.0126134E38F);
		p89.pitch_SET(-1.842393E38F);
		p89.yaw_SET(-1.8881826E38F);
		CommunicationChannel.instance.send(p89); //===============================
		HIL_STATE p90 = CommunicationChannel.instance.new_HIL_STATE();
		PH.setPack(p90);
		p90.time_usec_SET(1247053077644434552L);
		p90.roll_SET(-2.9170587E38F);
		p90.pitch_SET(-2.3650431E38F);
		p90.yaw_SET(2.6721486E38F);
		p90.rollspeed_SET(-1.5238418E38F);
		p90.pitchspeed_SET(-3.0778985E38F);
		p90.yawspeed_SET(-1.9876488E38F);
		p90.lat_SET(-1527554289);
		p90.lon_SET(-1039239031);
		p90.alt_SET(-1792485080);
		p90.vx_SET((short) -27949);
		p90.vy_SET((short) -31197);
		p90.vz_SET((short) -16953);
		p90.xacc_SET((short) -24801);
		p90.yacc_SET((short) 11855);
		p90.zacc_SET((short) -1965);
		CommunicationChannel.instance.send(p90); //===============================
		HIL_CONTROLS p91 = CommunicationChannel.instance.new_HIL_CONTROLS();
		PH.setPack(p91);
		p91.time_usec_SET(1169559914433421236L);
		p91.roll_ailerons_SET(-1.6574983E37F);
		p91.pitch_elevator_SET(-1.93651E38F);
		p91.yaw_rudder_SET(1.7149357E38F);
		p91.throttle_SET(-1.0189637E38F);
		p91.aux1_SET(1.6311982E38F);
		p91.aux2_SET(-1.3064794E38F);
		p91.aux3_SET(2.324155E37F);
		p91.aux4_SET(5.4713204E37F);
		p91.mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED);
		p91.nav_mode_SET((char) 15);
		CommunicationChannel.instance.send(p91); //===============================
		HIL_RC_INPUTS_RAW p92 = CommunicationChannel.instance.new_HIL_RC_INPUTS_RAW();
		PH.setPack(p92);
		p92.time_usec_SET(5553187711291911311L);
		p92.chan1_raw_SET((char) 10896);
		p92.chan2_raw_SET((char) 59133);
		p92.chan3_raw_SET((char) 5714);
		p92.chan4_raw_SET((char) 16926);
		p92.chan5_raw_SET((char) 61664);
		p92.chan6_raw_SET((char) 64266);
		p92.chan7_raw_SET((char) 56114);
		p92.chan8_raw_SET((char) 23500);
		p92.chan9_raw_SET((char) 26239);
		p92.chan10_raw_SET((char) 6420);
		p92.chan11_raw_SET((char) 42202);
		p92.chan12_raw_SET((char) 11986);
		p92.rssi_SET((char) 236);
		CommunicationChannel.instance.send(p92); //===============================
		HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.instance.new_HIL_ACTUATOR_CONTROLS();
		PH.setPack(p93);
		p93.time_usec_SET(1100669357532415077L);
		p93.controls_SET(new float[16], 0);
		p93.mode_SET(MAV_MODE.MAV_MODE_GUIDED_ARMED);
		p93.flags_SET(1118882736490554309L);
		CommunicationChannel.instance.send(p93); //===============================
		OPTICAL_FLOW p100 = CommunicationChannel.instance.new_OPTICAL_FLOW();
		PH.setPack(p100);
		p100.time_usec_SET(2065289821965221013L);
		p100.sensor_id_SET((char) 189);
		p100.flow_x_SET((short) -32123);
		p100.flow_y_SET((short) 13865);
		p100.flow_comp_m_x_SET(-1.1316059E38F);
		p100.flow_comp_m_y_SET(-1.5737737E38F);
		p100.quality_SET((char) 66);
		p100.ground_distance_SET(-7.8095096E37F);
		p100.flow_rate_x_SET(-2.0768117E38F, PH);
		p100.flow_rate_y_SET(1.3080962E38F, PH);
		CommunicationChannel.instance.send(p100); //===============================
		GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.instance.new_GLOBAL_VISION_POSITION_ESTIMATE();
		PH.setPack(p101);
		p101.usec_SET(3301439957713683915L);
		p101.x_SET(1.4255684E38F);
		p101.y_SET(-3.365796E38F);
		p101.z_SET(7.927457E37F);
		p101.roll_SET(-3.38577E38F);
		p101.pitch_SET(-7.0287296E37F);
		p101.yaw_SET(2.8826906E38F);
		CommunicationChannel.instance.send(p101); //===============================
		VISION_POSITION_ESTIMATE p102 = CommunicationChannel.instance.new_VISION_POSITION_ESTIMATE();
		PH.setPack(p102);
		p102.usec_SET(5649941456803392222L);
		p102.x_SET(-3.340961E38F);
		p102.y_SET(1.3977836E38F);
		p102.z_SET(-2.2473253E38F);
		p102.roll_SET(2.1266867E38F);
		p102.pitch_SET(2.0199055E38F);
		p102.yaw_SET(-3.3642047E38F);
		CommunicationChannel.instance.send(p102); //===============================
		VISION_SPEED_ESTIMATE p103 = CommunicationChannel.instance.new_VISION_SPEED_ESTIMATE();
		PH.setPack(p103);
		p103.usec_SET(305858448145668095L);
		p103.x_SET(1.7656034E38F);
		p103.y_SET(1.7525775E38F);
		p103.z_SET(-3.1510222E37F);
		CommunicationChannel.instance.send(p103); //===============================
		VICON_POSITION_ESTIMATE p104 = CommunicationChannel.instance.new_VICON_POSITION_ESTIMATE();
		PH.setPack(p104);
		p104.usec_SET(2905842285052121255L);
		p104.x_SET(1.9444946E37F);
		p104.y_SET(2.4978086E38F);
		p104.z_SET(-3.927224E36F);
		p104.roll_SET(6.5195144E37F);
		p104.pitch_SET(-1.8101116E38F);
		p104.yaw_SET(-1.1406478E38F);
		CommunicationChannel.instance.send(p104); //===============================
		HIGHRES_IMU p105 = CommunicationChannel.instance.new_HIGHRES_IMU();
		PH.setPack(p105);
		p105.time_usec_SET(7094357272374988070L);
		p105.xacc_SET(4.4372046E37F);
		p105.yacc_SET(2.3487487E38F);
		p105.zacc_SET(-2.0076714E38F);
		p105.xgyro_SET(5.8429626E37F);
		p105.ygyro_SET(-6.383881E37F);
		p105.zgyro_SET(2.3163865E38F);
		p105.xmag_SET(2.5954128E38F);
		p105.ymag_SET(1.3459012E38F);
		p105.zmag_SET(2.0035001E37F);
		p105.abs_pressure_SET(-8.919327E37F);
		p105.diff_pressure_SET(3.0103661E38F);
		p105.pressure_alt_SET(1.4602209E37F);
		p105.temperature_SET(-1.6197291E38F);
		p105.fields_updated_SET((char) 26200);
		CommunicationChannel.instance.send(p105); //===============================
		OPTICAL_FLOW_RAD p106 = CommunicationChannel.instance.new_OPTICAL_FLOW_RAD();
		PH.setPack(p106);
		p106.time_usec_SET(3358617017261813781L);
		p106.sensor_id_SET((char) 89);
		p106.integration_time_us_SET(2428454969L);
		p106.integrated_x_SET(2.8070794E38F);
		p106.integrated_y_SET(-2.4095574E38F);
		p106.integrated_xgyro_SET(3.9973275E37F);
		p106.integrated_ygyro_SET(2.0908092E38F);
		p106.integrated_zgyro_SET(2.3167542E38F);
		p106.temperature_SET((short) 22968);
		p106.quality_SET((char) 240);
		p106.time_delta_distance_us_SET(748537804L);
		p106.distance_SET(-9.752392E37F);
		CommunicationChannel.instance.send(p106); //===============================
		HIL_SENSOR p107 = CommunicationChannel.instance.new_HIL_SENSOR();
		PH.setPack(p107);
		p107.time_usec_SET(748371864232426129L);
		p107.xacc_SET(1.4915763E38F);
		p107.yacc_SET(-1.2426674E38F);
		p107.zacc_SET(2.5669585E37F);
		p107.xgyro_SET(1.7082302E38F);
		p107.ygyro_SET(3.370761E38F);
		p107.zgyro_SET(-5.956698E37F);
		p107.xmag_SET(2.648216E38F);
		p107.ymag_SET(1.9944551E38F);
		p107.zmag_SET(-1.6545671E38F);
		p107.abs_pressure_SET(-2.2855748E38F);
		p107.diff_pressure_SET(-1.3387507E37F);
		p107.pressure_alt_SET(2.9996672E38F);
		p107.temperature_SET(-2.8002714E38F);
		p107.fields_updated_SET(1840302147L);
		CommunicationChannel.instance.send(p107); //===============================
		SIM_STATE p108 = CommunicationChannel.instance.new_SIM_STATE();
		PH.setPack(p108);
		p108.q1_SET(-2.6837847E37F);
		p108.q2_SET(2.0236888E38F);
		p108.q3_SET(-3.3168761E38F);
		p108.q4_SET(7.870537E37F);
		p108.roll_SET(2.2548052E38F);
		p108.pitch_SET(-1.2953103E38F);
		p108.yaw_SET(-1.5217563E38F);
		p108.xacc_SET(3.0804287E38F);
		p108.yacc_SET(-2.847244E37F);
		p108.zacc_SET(-1.907006E38F);
		p108.xgyro_SET(1.9259347E38F);
		p108.ygyro_SET(9.487448E37F);
		p108.zgyro_SET(8.816156E36F);
		p108.lat_SET(2.361037E38F);
		p108.lon_SET(3.0126556E38F);
		p108.alt_SET(-1.4029328E38F);
		p108.std_dev_horz_SET(1.9679659E38F);
		p108.std_dev_vert_SET(1.3729898E38F);
		p108.vn_SET(-1.8799005E37F);
		p108.ve_SET(-1.3406941E38F);
		p108.vd_SET(-9.223707E37F);
		CommunicationChannel.instance.send(p108); //===============================
		RADIO_STATUS p109 = CommunicationChannel.instance.new_RADIO_STATUS();
		PH.setPack(p109);
		p109.rssi_SET((char) 237);
		p109.remrssi_SET((char) 87);
		p109.txbuf_SET((char) 149);
		p109.noise_SET((char) 209);
		p109.remnoise_SET((char) 207);
		p109.rxerrors_SET((char) 56849);
		p109.fixed__SET((char) 62226);
		CommunicationChannel.instance.send(p109); //===============================
		FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.instance.new_FILE_TRANSFER_PROTOCOL();
		PH.setPack(p110);
		p110.target_network_SET((char) 195);
		p110.target_system_SET((char) 225);
		p110.target_component_SET((char) 169);
		p110.payload_SET(new char[251], 0);
		CommunicationChannel.instance.send(p110); //===============================
		TIMESYNC p111 = CommunicationChannel.instance.new_TIMESYNC();
		PH.setPack(p111);
		p111.tc1_SET(-3219837205771181014L);
		p111.ts1_SET(7592209202129267042L);
		CommunicationChannel.instance.send(p111); //===============================
		CAMERA_TRIGGER p112 = CommunicationChannel.instance.new_CAMERA_TRIGGER();
		PH.setPack(p112);
		p112.time_usec_SET(5238655980597723512L);
		p112.seq_SET(3043206254L);
		CommunicationChannel.instance.send(p112); //===============================
		HIL_GPS p113 = CommunicationChannel.instance.new_HIL_GPS();
		PH.setPack(p113);
		p113.time_usec_SET(2807518861538914106L);
		p113.fix_type_SET((char) 95);
		p113.lat_SET(1453289552);
		p113.lon_SET(-403000990);
		p113.alt_SET(153064874);
		p113.eph_SET((char) 41423);
		p113.epv_SET((char) 51734);
		p113.vel_SET((char) 25832);
		p113.vn_SET((short) 3340);
		p113.ve_SET((short) -32014);
		p113.vd_SET((short) -20890);
		p113.cog_SET((char) 6760);
		p113.satellites_visible_SET((char) 178);
		CommunicationChannel.instance.send(p113); //===============================
		HIL_OPTICAL_FLOW p114 = CommunicationChannel.instance.new_HIL_OPTICAL_FLOW();
		PH.setPack(p114);
		p114.time_usec_SET(7065715341978384322L);
		p114.sensor_id_SET((char) 202);
		p114.integration_time_us_SET(279080357L);
		p114.integrated_x_SET(2.7473175E38F);
		p114.integrated_y_SET(2.3401422E38F);
		p114.integrated_xgyro_SET(1.8817871E37F);
		p114.integrated_ygyro_SET(1.8324238E38F);
		p114.integrated_zgyro_SET(3.357441E38F);
		p114.temperature_SET((short) -14688);
		p114.quality_SET((char) 161);
		p114.time_delta_distance_us_SET(1771802526L);
		p114.distance_SET(-7.307403E37F);
		CommunicationChannel.instance.send(p114); //===============================
		HIL_STATE_QUATERNION p115 = CommunicationChannel.instance.new_HIL_STATE_QUATERNION();
		PH.setPack(p115);
		p115.time_usec_SET(4101666926983416545L);
		p115.attitude_quaternion_SET(new float[4], 0);
		p115.rollspeed_SET(2.5758685E38F);
		p115.pitchspeed_SET(-9.022221E37F);
		p115.yawspeed_SET(-2.670506E38F);
		p115.lat_SET(502556795);
		p115.lon_SET(1595409107);
		p115.alt_SET(1879917266);
		p115.vx_SET((short) 15973);
		p115.vy_SET((short) 12226);
		p115.vz_SET((short) -13017);
		p115.ind_airspeed_SET((char) 16530);
		p115.true_airspeed_SET((char) 36837);
		p115.xacc_SET((short) 12427);
		p115.yacc_SET((short) -23151);
		p115.zacc_SET((short) -25520);
		CommunicationChannel.instance.send(p115); //===============================
		SCALED_IMU2 p116 = CommunicationChannel.instance.new_SCALED_IMU2();
		PH.setPack(p116);
		p116.time_boot_ms_SET(2749094533L);
		p116.xacc_SET((short) 24574);
		p116.yacc_SET((short) -20122);
		p116.zacc_SET((short) 16182);
		p116.xgyro_SET((short) 2511);
		p116.ygyro_SET((short) 23588);
		p116.zgyro_SET((short) 14613);
		p116.xmag_SET((short) 310);
		p116.ymag_SET((short) -11457);
		p116.zmag_SET((short) 23349);
		CommunicationChannel.instance.send(p116); //===============================
		LOG_REQUEST_LIST p117 = CommunicationChannel.instance.new_LOG_REQUEST_LIST();
		PH.setPack(p117);
		p117.target_system_SET((char) 215);
		p117.target_component_SET((char) 180);
		p117.start_SET((char) 2282);
		p117.end_SET((char) 17724);
		CommunicationChannel.instance.send(p117); //===============================
		LOG_ENTRY p118 = CommunicationChannel.instance.new_LOG_ENTRY();
		PH.setPack(p118);
		p118.id_SET((char) 55362);
		p118.num_logs_SET((char) 64396);
		p118.last_log_num_SET((char) 5128);
		p118.time_utc_SET(2581326402L);
		p118.size_SET(181681132L);
		CommunicationChannel.instance.send(p118); //===============================
		LOG_REQUEST_DATA p119 = CommunicationChannel.instance.new_LOG_REQUEST_DATA();
		PH.setPack(p119);
		p119.target_system_SET((char) 20);
		p119.target_component_SET((char) 200);
		p119.id_SET((char) 24478);
		p119.ofs_SET(3316374448L);
		p119.count_SET(509246352L);
		CommunicationChannel.instance.send(p119); //===============================
		LOG_DATA p120 = CommunicationChannel.instance.new_LOG_DATA();
		PH.setPack(p120);
		p120.id_SET((char) 31333);
		p120.ofs_SET(2608847192L);
		p120.count_SET((char) 128);
		p120.data__SET(new char[90], 0);
		CommunicationChannel.instance.send(p120); //===============================
		LOG_ERASE p121 = CommunicationChannel.instance.new_LOG_ERASE();
		PH.setPack(p121);
		p121.target_system_SET((char) 34);
		p121.target_component_SET((char) 26);
		CommunicationChannel.instance.send(p121); //===============================
		LOG_REQUEST_END p122 = CommunicationChannel.instance.new_LOG_REQUEST_END();
		PH.setPack(p122);
		p122.target_system_SET((char) 209);
		p122.target_component_SET((char) 103);
		CommunicationChannel.instance.send(p122); //===============================
		GPS_INJECT_DATA p123 = CommunicationChannel.instance.new_GPS_INJECT_DATA();
		PH.setPack(p123);
		p123.target_system_SET((char) 230);
		p123.target_component_SET((char) 181);
		p123.len_SET((char) 154);
		p123.data__SET(new char[110], 0);
		CommunicationChannel.instance.send(p123); //===============================
		GPS2_RAW p124 = CommunicationChannel.instance.new_GPS2_RAW();
		PH.setPack(p124);
		p124.time_usec_SET(390539494427235207L);
		p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
		p124.lat_SET(128593823);
		p124.lon_SET(-1632327983);
		p124.alt_SET(-1625454868);
		p124.eph_SET((char) 54933);
		p124.epv_SET((char) 21406);
		p124.vel_SET((char) 19237);
		p124.cog_SET((char) 54746);
		p124.satellites_visible_SET((char) 248);
		p124.dgps_numch_SET((char) 210);
		p124.dgps_age_SET(1337423073L);
		CommunicationChannel.instance.send(p124); //===============================
		POWER_STATUS p125 = CommunicationChannel.instance.new_POWER_STATUS();
		PH.setPack(p125);
		p125.Vcc_SET((char) 58373);
		p125.Vservo_SET((char) 16920);
		p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED);
		CommunicationChannel.instance.send(p125); //===============================
		SERIAL_CONTROL p126 = CommunicationChannel.instance.new_SERIAL_CONTROL();
		PH.setPack(p126);
		p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2);
		p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY);
		p126.timeout_SET((char) 8820);
		p126.baudrate_SET(3122292584L);
		p126.count_SET((char) 237);
		p126.data__SET(new char[70], 0);
		CommunicationChannel.instance.send(p126); //===============================
		GPS_RTK p127 = CommunicationChannel.instance.new_GPS_RTK();
		PH.setPack(p127);
		p127.time_last_baseline_ms_SET(423568633L);
		p127.rtk_receiver_id_SET((char) 49);
		p127.wn_SET((char) 57959);
		p127.tow_SET(4006320828L);
		p127.rtk_health_SET((char) 81);
		p127.rtk_rate_SET((char) 36);
		p127.nsats_SET((char) 36);
		p127.baseline_coords_type_SET((char) 74);
		p127.baseline_a_mm_SET(1759089739);
		p127.baseline_b_mm_SET(708773684);
		p127.baseline_c_mm_SET(-1107401195);
		p127.accuracy_SET(1402823971L);
		p127.iar_num_hypotheses_SET(1595852513);
		CommunicationChannel.instance.send(p127); //===============================
		GPS2_RTK p128 = CommunicationChannel.instance.new_GPS2_RTK();
		PH.setPack(p128);
		p128.time_last_baseline_ms_SET(610751494L);
		p128.rtk_receiver_id_SET((char) 233);
		p128.wn_SET((char) 46156);
		p128.tow_SET(1439590563L);
		p128.rtk_health_SET((char) 51);
		p128.rtk_rate_SET((char) 120);
		p128.nsats_SET((char) 208);
		p128.baseline_coords_type_SET((char) 120);
		p128.baseline_a_mm_SET(1185768947);
		p128.baseline_b_mm_SET(1044171457);
		p128.baseline_c_mm_SET(-186817484);
		p128.accuracy_SET(1474683518L);
		p128.iar_num_hypotheses_SET(-1664495683);
		CommunicationChannel.instance.send(p128); //===============================
		SCALED_IMU3 p129 = CommunicationChannel.instance.new_SCALED_IMU3();
		PH.setPack(p129);
		p129.time_boot_ms_SET(3268164958L);
		p129.xacc_SET((short) 12279);
		p129.yacc_SET((short) 12747);
		p129.zacc_SET((short) 22912);
		p129.xgyro_SET((short) 1486);
		p129.ygyro_SET((short) 17522);
		p129.zgyro_SET((short) 586);
		p129.xmag_SET((short) -2905);
		p129.ymag_SET((short) -32588);
		p129.zmag_SET((short) -1171);
		CommunicationChannel.instance.send(p129); //===============================
		DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.instance.new_DATA_TRANSMISSION_HANDSHAKE();
		PH.setPack(p130);
		p130.type_SET((char) 226);
		p130.size_SET(3933210878L);
		p130.width_SET((char) 42330);
		p130.height_SET((char) 55246);
		p130.packets_SET((char) 31790);
		p130.payload_SET((char) 9);
		p130.jpg_quality_SET((char) 80);
		CommunicationChannel.instance.send(p130); //===============================
		ENCAPSULATED_DATA p131 = CommunicationChannel.instance.new_ENCAPSULATED_DATA();
		PH.setPack(p131);
		p131.seqnr_SET((char) 37035);
		p131.data__SET(new char[253], 0);
		CommunicationChannel.instance.send(p131); //===============================
		DISTANCE_SENSOR p132 = CommunicationChannel.instance.new_DISTANCE_SENSOR();
		PH.setPack(p132);
		p132.time_boot_ms_SET(350516363L);
		p132.min_distance_SET((char) 56930);
		p132.max_distance_SET((char) 11112);
		p132.current_distance_SET((char) 52986);
		p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
		p132.id_SET((char) 37);
		p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_NONE);
		p132.covariance_SET((char) 183);
		CommunicationChannel.instance.send(p132); //===============================
		TERRAIN_REQUEST p133 = CommunicationChannel.instance.new_TERRAIN_REQUEST();
		PH.setPack(p133);
		p133.lat_SET(-1373240375);
		p133.lon_SET(162393751);
		p133.grid_spacing_SET((char) 46812);
		p133.mask_SET(4251324778389778351L);
		CommunicationChannel.instance.send(p133); //===============================
		TERRAIN_DATA p134 = CommunicationChannel.instance.new_TERRAIN_DATA();
		PH.setPack(p134);
		p134.lat_SET(980642552);
		p134.lon_SET(328560350);
		p134.grid_spacing_SET((char) 34048);
		p134.gridbit_SET((char) 230);
		p134.data__SET(new short[16], 0);
		CommunicationChannel.instance.send(p134); //===============================
		TERRAIN_CHECK p135 = CommunicationChannel.instance.new_TERRAIN_CHECK();
		PH.setPack(p135);
		p135.lat_SET(448528626);
		p135.lon_SET(291683655);
		CommunicationChannel.instance.send(p135); //===============================
		TERRAIN_REPORT p136 = CommunicationChannel.instance.new_TERRAIN_REPORT();
		PH.setPack(p136);
		p136.lat_SET(232927899);
		p136.lon_SET(-1880662728);
		p136.spacing_SET((char) 14120);
		p136.terrain_height_SET(1.352577E38F);
		p136.current_height_SET(-2.0462306E38F);
		p136.pending_SET((char) 24007);
		p136.loaded_SET((char) 22045);
		CommunicationChannel.instance.send(p136); //===============================
		SCALED_PRESSURE2 p137 = CommunicationChannel.instance.new_SCALED_PRESSURE2();
		PH.setPack(p137);
		p137.time_boot_ms_SET(2075825732L);
		p137.press_abs_SET(-1.7187103E38F);
		p137.press_diff_SET(2.0656705E37F);
		p137.temperature_SET((short) -23412);
		CommunicationChannel.instance.send(p137); //===============================
		ATT_POS_MOCAP p138 = CommunicationChannel.instance.new_ATT_POS_MOCAP();
		PH.setPack(p138);
		p138.time_usec_SET(8907939702228231653L);
		p138.q_SET(new float[4], 0);
		p138.x_SET(2.0344557E38F);
		p138.y_SET(-2.1342109E38F);
		p138.z_SET(-3.6671185E37F);
		CommunicationChannel.instance.send(p138); //===============================
		SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.instance.new_SET_ACTUATOR_CONTROL_TARGET();
		PH.setPack(p139);
		p139.time_usec_SET(5568224164248792011L);
		p139.group_mlx_SET((char) 16);
		p139.target_system_SET((char) 133);
		p139.target_component_SET((char) 180);
		p139.controls_SET(new float[8], 0);
		CommunicationChannel.instance.send(p139); //===============================
		ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.instance.new_ACTUATOR_CONTROL_TARGET();
		PH.setPack(p140);
		p140.time_usec_SET(5459100198440520793L);
		p140.group_mlx_SET((char) 183);
		p140.controls_SET(new float[8], 0);
		CommunicationChannel.instance.send(p140); //===============================
		ALTITUDE p141 = CommunicationChannel.instance.new_ALTITUDE();
		PH.setPack(p141);
		p141.time_usec_SET(2751534134120144539L);
		p141.altitude_monotonic_SET(-1.8369117E38F);
		p141.altitude_amsl_SET(-8.642844E36F);
		p141.altitude_local_SET(-2.2764735E38F);
		p141.altitude_relative_SET(-1.838493E38F);
		p141.altitude_terrain_SET(-2.4042253E38F);
		p141.bottom_clearance_SET(-1.1079491E38F);
		CommunicationChannel.instance.send(p141); //===============================
		RESOURCE_REQUEST p142 = CommunicationChannel.instance.new_RESOURCE_REQUEST();
		PH.setPack(p142);
		p142.request_id_SET((char) 243);
		p142.uri_type_SET((char) 172);
		p142.uri_SET(new char[120], 0);
		p142.transfer_type_SET((char) 19);
		p142.storage_SET(new char[120], 0);
		CommunicationChannel.instance.send(p142); //===============================
		SCALED_PRESSURE3 p143 = CommunicationChannel.instance.new_SCALED_PRESSURE3();
		PH.setPack(p143);
		p143.time_boot_ms_SET(3230795646L);
		p143.press_abs_SET(9.504569E37F);
		p143.press_diff_SET(2.2218925E38F);
		p143.temperature_SET((short) -12462);
		CommunicationChannel.instance.send(p143); //===============================
		FOLLOW_TARGET p144 = CommunicationChannel.instance.new_FOLLOW_TARGET();
		PH.setPack(p144);
		p144.timestamp_SET(7372083226962960079L);
		p144.est_capabilities_SET((char) 82);
		p144.lat_SET(-1043356283);
		p144.lon_SET(-178182152);
		p144.alt_SET(1.333497E38F);
		p144.vel_SET(new float[3], 0);
		p144.acc_SET(new float[3], 0);
		p144.attitude_q_SET(new float[4], 0);
		p144.rates_SET(new float[3], 0);
		p144.position_cov_SET(new float[3], 0);
		p144.custom_state_SET(529610511247527128L);
		CommunicationChannel.instance.send(p144); //===============================
		CONTROL_SYSTEM_STATE p146 = CommunicationChannel.instance.new_CONTROL_SYSTEM_STATE();
		PH.setPack(p146);
		p146.time_usec_SET(8055449052908840562L);
		p146.x_acc_SET(4.9898526E37F);
		p146.y_acc_SET(-1.551754E38F);
		p146.z_acc_SET(-3.2399168E38F);
		p146.x_vel_SET(2.9833171E38F);
		p146.y_vel_SET(-2.370378E38F);
		p146.z_vel_SET(-1.5398709E38F);
		p146.x_pos_SET(-3.0935094E38F);
		p146.y_pos_SET(4.5952613E37F);
		p146.z_pos_SET(9.597858E36F);
		p146.airspeed_SET(2.135024E38F);
		p146.vel_variance_SET(new float[3], 0);
		p146.pos_variance_SET(new float[3], 0);
		p146.q_SET(new float[4], 0);
		p146.roll_rate_SET(9.662384E37F);
		p146.pitch_rate_SET(-1.5015501E38F);
		p146.yaw_rate_SET(2.4069127E38F);
		CommunicationChannel.instance.send(p146); //===============================
		BATTERY_STATUS p147 = CommunicationChannel.instance.new_BATTERY_STATUS();
		PH.setPack(p147);
		p147.id_SET((char) 119);
		p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN);
		p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
		p147.temperature_SET((short) 17110);
		p147.voltages_SET(new char[10], 0);
		p147.current_battery_SET((short) 30283);
		p147.current_consumed_SET(1411308568);
		p147.energy_consumed_SET(1264458713);
		p147.battery_remaining_SET((byte) -99);
		CommunicationChannel.instance.send(p147); //===============================
		AUTOPILOT_VERSION p148 = CommunicationChannel.instance.new_AUTOPILOT_VERSION();
		PH.setPack(p148);
		p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT);
		p148.flight_sw_version_SET(4041052651L);
		p148.middleware_sw_version_SET(3321811821L);
		p148.os_sw_version_SET(1493510827L);
		p148.board_version_SET(2217113737L);
		p148.flight_custom_version_SET(new char[8], 0);
		p148.middleware_custom_version_SET(new char[8], 0);
		p148.os_custom_version_SET(new char[8], 0);
		p148.vendor_id_SET((char) 49162);
		p148.product_id_SET((char) 44927);
		p148.uid_SET(3809971955163475459L);
		p148.uid2_SET(new char[18], 0, PH);
		CommunicationChannel.instance.send(p148); //===============================
		LANDING_TARGET p149 = CommunicationChannel.instance.new_LANDING_TARGET();
		PH.setPack(p149);
		p149.time_usec_SET(8357286739695819741L);
		p149.target_num_SET((char) 162);
		p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
		p149.angle_x_SET(-1.9774715E38F);
		p149.angle_y_SET(-2.763357E38F);
		p149.distance_SET(-2.6631426E38F);
		p149.size_x_SET(1.1573538E38F);
		p149.size_y_SET(-2.0288853E38F);
		p149.x_SET(-2.0223856E38F, PH);
		p149.y_SET(1.1965152E38F, PH);
		p149.z_SET(-1.0357671E38F, PH);
		p149.q_SET(new float[4], 0, PH);
		p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
		p149.position_valid_SET((char) 79, PH);
		CommunicationChannel.instance.send(p149); //===============================
		ARRAY_TEST_0 p150 = CommunicationChannel.instance.new_ARRAY_TEST_0();
		PH.setPack(p150);
		p150.v1_SET((char) 72);
		p150.ar_i8_SET(new byte[4], 0);
		p150.ar_u8_SET(new char[4], 0);
		p150.ar_u16_SET(new char[4], 0);
		p150.ar_u32_SET(new long[4], 0);
		CommunicationChannel.instance.send(p150); //===============================
		ARRAY_TEST_1 p151 = CommunicationChannel.instance.new_ARRAY_TEST_1();
		PH.setPack(p151);
		p151.ar_u32_SET(new long[4], 0);
		CommunicationChannel.instance.send(p151); //===============================
		ARRAY_TEST_3 p153 = CommunicationChannel.instance.new_ARRAY_TEST_3();
		PH.setPack(p153);
		p153.v_SET((char) 205);
		p153.ar_u32_SET(new long[4], 0);
		CommunicationChannel.instance.send(p153); //===============================
		ARRAY_TEST_4 p154 = CommunicationChannel.instance.new_ARRAY_TEST_4();
		PH.setPack(p154);
		p154.ar_u32_SET(new long[4], 0);
		p154.v_SET((char) 88);
		CommunicationChannel.instance.send(p154); //===============================
		ARRAY_TEST_5 p155 = CommunicationChannel.instance.new_ARRAY_TEST_5();
		PH.setPack(p155);
		p155.c1_SET("DEMO", PH);
		p155.c2_SET("DEMO", PH);
		CommunicationChannel.instance.send(p155); //===============================
		ARRAY_TEST_6 p156 = CommunicationChannel.instance.new_ARRAY_TEST_6();
		PH.setPack(p156);
		p156.v1_SET((char) 190);
		p156.v2_SET((char) 20124);
		p156.v3_SET(1604290875L);
		p156.ar_u32_SET(new long[2], 0);
		p156.ar_i32_SET(new int[2], 0);
		p156.ar_u16_SET(new char[2], 0);
		p156.ar_i16_SET(new short[2], 0);
		p156.ar_u8_SET(new char[2], 0);
		p156.ar_i8_SET(new byte[2], 0);
		p156.ar_c_SET("DEMO", PH);
		p156.ar_d_SET(new double[2], 0);
		p156.ar_f_SET(new float[2], 0);
		CommunicationChannel.instance.send(p156); //===============================
		ARRAY_TEST_7 p157 = CommunicationChannel.instance.new_ARRAY_TEST_7();
		PH.setPack(p157);
		p157.ar_d_SET(new double[2], 0);
		p157.ar_f_SET(new float[2], 0);
		p157.ar_u32_SET(new long[2], 0);
		p157.ar_i32_SET(new int[2], 0);
		p157.ar_u16_SET(new char[2], 0);
		p157.ar_i16_SET(new short[2], 0);
		p157.ar_u8_SET(new char[2], 0);
		p157.ar_i8_SET(new byte[2], 0);
		p157.ar_c_SET("DEMO", PH);
		CommunicationChannel.instance.send(p157); //===============================
		ARRAY_TEST_8 p158 = CommunicationChannel.instance.new_ARRAY_TEST_8();
		PH.setPack(p158);
		p158.v3_SET(2152686698L);
		p158.ar_d_SET(new double[2], 0);
		p158.ar_u16_SET(new char[2], 0);
		CommunicationChannel.instance.send(p158); //===============================
		ESTIMATOR_STATUS p230 = CommunicationChannel.instance.new_ESTIMATOR_STATUS();
		PH.setPack(p230);
		p230.time_usec_SET(2460898232587100454L);
		p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE);
		p230.vel_ratio_SET(-1.502351E38F);
		p230.pos_horiz_ratio_SET(-1.6540519E37F);
		p230.pos_vert_ratio_SET(2.2083855E38F);
		p230.mag_ratio_SET(-7.126604E37F);
		p230.hagl_ratio_SET(-1.2621311E38F);
		p230.tas_ratio_SET(1.1152327E38F);
		p230.pos_horiz_accuracy_SET(-2.4192049E38F);
		p230.pos_vert_accuracy_SET(2.1116325E38F);
		CommunicationChannel.instance.send(p230); //===============================
		WIND_COV p231 = CommunicationChannel.instance.new_WIND_COV();
		PH.setPack(p231);
		p231.time_usec_SET(8448675164376659392L);
		p231.wind_x_SET(-2.2365527E38F);
		p231.wind_y_SET(2.5577747E38F);
		p231.wind_z_SET(-1.7361885E37F);
		p231.var_horiz_SET(-2.4360077E38F);
		p231.var_vert_SET(-2.1545369E38F);
		p231.wind_alt_SET(3.0050118E38F);
		p231.horiz_accuracy_SET(-5.260791E37F);
		p231.vert_accuracy_SET(2.3465535E38F);
		CommunicationChannel.instance.send(p231); //===============================
		GPS_INPUT p232 = CommunicationChannel.instance.new_GPS_INPUT();
		PH.setPack(p232);
		p232.time_usec_SET(580836068306215296L);
		p232.gps_id_SET((char) 175);
		p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY);
		p232.time_week_ms_SET(1111599870L);
		p232.time_week_SET((char) 59308);
		p232.fix_type_SET((char) 220);
		p232.lat_SET(-814044645);
		p232.lon_SET(-405193063);
		p232.alt_SET(-1.0763074E38F);
		p232.hdop_SET(-3.14628E38F);
		p232.vdop_SET(-6.608357E37F);
		p232.vn_SET(5.8649305E37F);
		p232.ve_SET(-3.4020817E38F);
		p232.vd_SET(-1.810637E38F);
		p232.speed_accuracy_SET(2.000957E38F);
		p232.horiz_accuracy_SET(1.2053405E38F);
		p232.vert_accuracy_SET(2.7899247E38F);
		p232.satellites_visible_SET((char) 54);
		CommunicationChannel.instance.send(p232); //===============================
		GPS_RTCM_DATA p233 = CommunicationChannel.instance.new_GPS_RTCM_DATA();
		PH.setPack(p233);
		p233.flags_SET((char) 9);
		p233.len_SET((char) 57);
		p233.data__SET(new char[180], 0);
		CommunicationChannel.instance.send(p233); //===============================
		HIGH_LATENCY p234 = CommunicationChannel.instance.new_HIGH_LATENCY();
		PH.setPack(p234);
		p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED);
		p234.custom_mode_SET(422176892L);
		p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
		p234.roll_SET((short) -2990);
		p234.pitch_SET((short) 3208);
		p234.heading_SET((char) 21407);
		p234.throttle_SET((byte) 21);
		p234.heading_sp_SET((short) 31177);
		p234.latitude_SET(-1136725329);
		p234.longitude_SET(-1545157366);
		p234.altitude_amsl_SET((short) -15371);
		p234.altitude_sp_SET((short) 31678);
		p234.airspeed_SET((char) 76);
		p234.airspeed_sp_SET((char) 188);
		p234.groundspeed_SET((char) 234);
		p234.climb_rate_SET((byte) 116);
		p234.gps_nsat_SET((char) 116);
		p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
		p234.battery_remaining_SET((char) 8);
		p234.temperature_SET((byte) 125);
		p234.temperature_air_SET((byte) 0);
		p234.failsafe_SET((char) 14);
		p234.wp_num_SET((char) 46);
		p234.wp_distance_SET((char) 24086);
		CommunicationChannel.instance.send(p234); //===============================
		VIBRATION p241 = CommunicationChannel.instance.new_VIBRATION();
		PH.setPack(p241);
		p241.time_usec_SET(6624441753605979831L);
		p241.vibration_x_SET(-3.0643916E38F);
		p241.vibration_y_SET(-3.2427048E38F);
		p241.vibration_z_SET(5.13034E37F);
		p241.clipping_0_SET(923614045L);
		p241.clipping_1_SET(1069944408L);
		p241.clipping_2_SET(1621574227L);
		CommunicationChannel.instance.send(p241); //===============================
		HOME_POSITION p242 = CommunicationChannel.instance.new_HOME_POSITION();
		PH.setPack(p242);
		p242.latitude_SET(1766141528);
		p242.longitude_SET(-527856412);
		p242.altitude_SET(358712945);
		p242.x_SET(-1.2674235E38F);
		p242.y_SET(-2.9653773E38F);
		p242.z_SET(1.9017619E38F);
		p242.q_SET(new float[4], 0);
		p242.approach_x_SET(7.987166E37F);
		p242.approach_y_SET(1.8105543E38F);
		p242.approach_z_SET(-1.4465151E37F);
		p242.time_usec_SET(1664698904977607740L, PH);
		CommunicationChannel.instance.send(p242); //===============================
		SET_HOME_POSITION p243 = CommunicationChannel.instance.new_SET_HOME_POSITION();
		PH.setPack(p243);
		p243.target_system_SET((char) 62);
		p243.latitude_SET(1800484434);
		p243.longitude_SET(-1314995945);
		p243.altitude_SET(-423985252);
		p243.x_SET(-1.6883788E38F);
		p243.y_SET(-1.111432E38F);
		p243.z_SET(2.586582E38F);
		p243.q_SET(new float[4], 0);
		p243.approach_x_SET(-4.0392457E37F);
		p243.approach_y_SET(2.794994E37F);
		p243.approach_z_SET(-1.6157189E38F);
		p243.time_usec_SET(4069479777182156300L, PH);
		CommunicationChannel.instance.send(p243); //===============================
		MESSAGE_INTERVAL p244 = CommunicationChannel.instance.new_MESSAGE_INTERVAL();
		PH.setPack(p244);
		p244.message_id_SET((char) 12431);
		p244.interval_us_SET(-1452758498);
		CommunicationChannel.instance.send(p244); //===============================
		EXTENDED_SYS_STATE p245 = CommunicationChannel.instance.new_EXTENDED_SYS_STATE();
		PH.setPack(p245);
		p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
		p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
		CommunicationChannel.instance.send(p245); //===============================
		ADSB_VEHICLE p246 = CommunicationChannel.instance.new_ADSB_VEHICLE();
		PH.setPack(p246);
		p246.ICAO_address_SET(911027655L);
		p246.lat_SET(2012150732);
		p246.lon_SET(-617133789);
		p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
		p246.altitude_SET(-1863510151);
		p246.heading_SET((char) 39381);
		p246.hor_velocity_SET((char) 24446);
		p246.ver_velocity_SET((short) -18583);
		p246.callsign_SET("DEMO", PH);
		p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_LIGHT);
		p246.tslc_SET((char) 186);
		p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN);
		p246.squawk_SET((char) 10745);
		CommunicationChannel.instance.send(p246); //===============================
		COLLISION p247 = CommunicationChannel.instance.new_COLLISION();
		PH.setPack(p247);
		p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
		p247.id_SET(864730015L);
		p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER);
		p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
		p247.time_to_minimum_delta_SET(4.98564E37F);
		p247.altitude_minimum_delta_SET(2.7545839E38F);
		p247.horizontal_minimum_delta_SET(-2.4210368E38F);
		CommunicationChannel.instance.send(p247); //===============================
		V2_EXTENSION p248 = CommunicationChannel.instance.new_V2_EXTENSION();
		PH.setPack(p248);
		p248.target_network_SET((char) 179);
		p248.target_system_SET((char) 131);
		p248.target_component_SET((char) 122);
		p248.message_type_SET((char) 53583);
		p248.payload_SET(new char[249], 0);
		CommunicationChannel.instance.send(p248); //===============================
		MEMORY_VECT p249 = CommunicationChannel.instance.new_MEMORY_VECT();
		PH.setPack(p249);
		p249.address_SET((char) 41736);
		p249.ver_SET((char) 41);
		p249.type_SET((char) 206);
		p249.value_SET(new byte[32], 0);
		CommunicationChannel.instance.send(p249); //===============================
		DEBUG_VECT p250 = CommunicationChannel.instance.new_DEBUG_VECT();
		PH.setPack(p250);
		p250.name_SET("DEMO", PH);
		p250.time_usec_SET(3416837546522124219L);
		p250.x_SET(3.159788E38F);
		p250.y_SET(5.513483E37F);
		p250.z_SET(-2.3326485E38F);
		CommunicationChannel.instance.send(p250); //===============================
		NAMED_VALUE_FLOAT p251 = CommunicationChannel.instance.new_NAMED_VALUE_FLOAT();
		PH.setPack(p251);
		p251.time_boot_ms_SET(3434416763L);
		p251.name_SET("DEMO", PH);
		p251.value_SET(-2.6896092E38F);
		CommunicationChannel.instance.send(p251); //===============================
		NAMED_VALUE_INT p252 = CommunicationChannel.instance.new_NAMED_VALUE_INT();
		PH.setPack(p252);
		p252.time_boot_ms_SET(3085747703L);
		p252.name_SET("DEMO", PH);
		p252.value_SET(-502207934);
		CommunicationChannel.instance.send(p252); //===============================
		STATUSTEXT p253 = CommunicationChannel.instance.new_STATUSTEXT();
		PH.setPack(p253);
		p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
		p253.text_SET("DEMO", PH);
		CommunicationChannel.instance.send(p253); //===============================
		DEBUG p254 = CommunicationChannel.instance.new_DEBUG();
		PH.setPack(p254);
		p254.time_boot_ms_SET(262425625L);
		p254.ind_SET((char) 171);
		p254.value_SET(-3.564339E37F);
		CommunicationChannel.instance.send(p254); //===============================
		SETUP_SIGNING p256 = CommunicationChannel.instance.new_SETUP_SIGNING();
		PH.setPack(p256);
		p256.target_system_SET((char) 162);
		p256.target_component_SET((char) 96);
		p256.secret_key_SET(new char[32], 0);
		p256.initial_timestamp_SET(1628719077251514001L);
		CommunicationChannel.instance.send(p256); //===============================
		BUTTON_CHANGE p257 = CommunicationChannel.instance.new_BUTTON_CHANGE();
		PH.setPack(p257);
		p257.time_boot_ms_SET(1281947296L);
		p257.last_change_ms_SET(3589382224L);
		p257.state_SET((char) 97);
		CommunicationChannel.instance.send(p257); //===============================
		PLAY_TUNE p258 = CommunicationChannel.instance.new_PLAY_TUNE();
		PH.setPack(p258);
		p258.target_system_SET((char) 182);
		p258.target_component_SET((char) 124);
		p258.tune_SET("DEMO", PH);
		CommunicationChannel.instance.send(p258); //===============================
		CAMERA_INFORMATION p259 = CommunicationChannel.instance.new_CAMERA_INFORMATION();
		PH.setPack(p259);
		p259.time_boot_ms_SET(1858677168L);
		p259.vendor_name_SET(new char[32], 0);
		p259.model_name_SET(new char[32], 0);
		p259.firmware_version_SET(2584545268L);
		p259.focal_length_SET(-1.5050469E38F);
		p259.sensor_size_h_SET(-7.7614824E37F);
		p259.sensor_size_v_SET(2.4275314E38F);
		p259.resolution_h_SET((char) 13286);
		p259.resolution_v_SET((char) 4522);
		p259.lens_id_SET((char) 155);
		p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE);
		p259.cam_definition_version_SET((char) 25672);
		p259.cam_definition_uri_SET("DEMO", PH);
		CommunicationChannel.instance.send(p259); //===============================
		CAMERA_SETTINGS p260 = CommunicationChannel.instance.new_CAMERA_SETTINGS();
		PH.setPack(p260);
		p260.time_boot_ms_SET(1284065601L);
		p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_VIDEO);
		CommunicationChannel.instance.send(p260); //===============================
		STORAGE_INFORMATION p261 = CommunicationChannel.instance.new_STORAGE_INFORMATION();
		PH.setPack(p261);
		p261.time_boot_ms_SET(639162751L);
		p261.storage_id_SET((char) 50);
		p261.storage_count_SET((char) 185);
		p261.status_SET((char) 181);
		p261.total_capacity_SET(-1.4267924E38F);
		p261.used_capacity_SET(2.130453E38F);
		p261.available_capacity_SET(2.9429085E38F);
		p261.read_speed_SET(-2.617847E38F);
		p261.write_speed_SET(1.9931822E38F);
		CommunicationChannel.instance.send(p261); //===============================
		CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.instance.new_CAMERA_CAPTURE_STATUS();
		PH.setPack(p262);
		p262.time_boot_ms_SET(2761755859L);
		p262.image_status_SET((char) 122);
		p262.video_status_SET((char) 221);
		p262.image_interval_SET(-1.9029703E38F);
		p262.recording_time_ms_SET(2481659147L);
		p262.available_capacity_SET(1.7870283E38F);
		CommunicationChannel.instance.send(p262); //===============================
		CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.instance.new_CAMERA_IMAGE_CAPTURED();
		PH.setPack(p263);
		p263.time_boot_ms_SET(593455280L);
		p263.time_utc_SET(1488290547214392336L);
		p263.camera_id_SET((char) 92);
		p263.lat_SET(-561052518);
		p263.lon_SET(546234642);
		p263.alt_SET(-1976188854);
		p263.relative_alt_SET(826508548);
		p263.q_SET(new float[4], 0);
		p263.image_index_SET(837612);
		p263.capture_result_SET((byte) -54);
		p263.file_url_SET("DEMO", PH);
		CommunicationChannel.instance.send(p263); //===============================
		FLIGHT_INFORMATION p264 = CommunicationChannel.instance.new_FLIGHT_INFORMATION();
		PH.setPack(p264);
		p264.time_boot_ms_SET(2541240905L);
		p264.arming_time_utc_SET(3009464957829956958L);
		p264.takeoff_time_utc_SET(971740504216934332L);
		p264.flight_uuid_SET(6145283000743195842L);
		CommunicationChannel.instance.send(p264); //===============================
		MOUNT_ORIENTATION p265 = CommunicationChannel.instance.new_MOUNT_ORIENTATION();
		PH.setPack(p265);
		p265.time_boot_ms_SET(3379960111L);
		p265.roll_SET(2.1639826E38F);
		p265.pitch_SET(-6.400208E37F);
		p265.yaw_SET(-3.531142E36F);
		CommunicationChannel.instance.send(p265); //===============================
		LOGGING_DATA p266 = CommunicationChannel.instance.new_LOGGING_DATA();
		PH.setPack(p266);
		p266.target_system_SET((char) 99);
		p266.target_component_SET((char) 43);
		p266.sequence_SET((char) 61481);
		p266.length_SET((char) 160);
		p266.first_message_offset_SET((char) 13);
		p266.data__SET(new char[249], 0);
		CommunicationChannel.instance.send(p266); //===============================
		LOGGING_DATA_ACKED p267 = CommunicationChannel.instance.new_LOGGING_DATA_ACKED();
		PH.setPack(p267);
		p267.target_system_SET((char) 16);
		p267.target_component_SET((char) 130);
		p267.sequence_SET((char) 496);
		p267.length_SET((char) 136);
		p267.first_message_offset_SET((char) 187);
		p267.data__SET(new char[249], 0);
		CommunicationChannel.instance.send(p267); //===============================
		LOGGING_ACK p268 = CommunicationChannel.instance.new_LOGGING_ACK();
		PH.setPack(p268);
		p268.target_system_SET((char) 236);
		p268.target_component_SET((char) 225);
		p268.sequence_SET((char) 38357);
		CommunicationChannel.instance.send(p268); //===============================
		VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.instance.new_VIDEO_STREAM_INFORMATION();
		PH.setPack(p269);
		p269.camera_id_SET((char) 224);
		p269.status_SET((char) 146);
		p269.framerate_SET(1.9770615E38F);
		p269.resolution_h_SET((char) 21273);
		p269.resolution_v_SET((char) 7019);
		p269.bitrate_SET(1581500894L);
		p269.rotation_SET((char) 55553);
		p269.uri_SET("DEMO", PH);
		CommunicationChannel.instance.send(p269); //===============================
		SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.instance.new_SET_VIDEO_STREAM_SETTINGS();
		PH.setPack(p270);
		p270.target_system_SET((char) 167);
		p270.target_component_SET((char) 19);
		p270.camera_id_SET((char) 116);
		p270.framerate_SET(-3.3988583E38F);
		p270.resolution_h_SET((char) 39956);
		p270.resolution_v_SET((char) 14239);
		p270.bitrate_SET(3534309144L);
		p270.rotation_SET((char) 14171);
		p270.uri_SET("DEMO", PH);
		CommunicationChannel.instance.send(p270); //===============================
		WIFI_CONFIG_AP p299 = CommunicationChannel.instance.new_WIFI_CONFIG_AP();
		PH.setPack(p299);
		p299.ssid_SET("DEMO", PH);
		p299.password_SET("DEMO", PH);
		CommunicationChannel.instance.send(p299); //===============================
		PROTOCOL_VERSION p300 = CommunicationChannel.instance.new_PROTOCOL_VERSION();
		PH.setPack(p300);
		p300.version_SET((char) 43590);
		p300.min_version_SET((char) 7741);
		p300.max_version_SET((char) 20376);
		p300.spec_version_hash_SET(new char[8], 0);
		p300.library_version_hash_SET(new char[8], 0);
		CommunicationChannel.instance.send(p300); //===============================
		UAVCAN_NODE_STATUS p310 = CommunicationChannel.instance.new_UAVCAN_NODE_STATUS();
		PH.setPack(p310);
		p310.time_usec_SET(1165311981223438448L);
		p310.uptime_sec_SET(1700375161L);
		p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
		p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
		p310.sub_mode_SET((char) 44);
		p310.vendor_specific_status_code_SET((char) 18593);
		CommunicationChannel.instance.send(p310); //===============================
		UAVCAN_NODE_INFO p311 = CommunicationChannel.instance.new_UAVCAN_NODE_INFO();
		PH.setPack(p311);
		p311.time_usec_SET(4504180273415553291L);
		p311.uptime_sec_SET(2267411602L);
		p311.name_SET("DEMO", PH);
		p311.hw_version_major_SET((char) 232);
		p311.hw_version_minor_SET((char) 47);
		p311.hw_unique_id_SET(new char[16], 0);
		p311.sw_version_major_SET((char) 38);
		p311.sw_version_minor_SET((char) 30);
		p311.sw_vcs_commit_SET(3527001419L);
		CommunicationChannel.instance.send(p311); //===============================
		PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_READ();
		PH.setPack(p320);
		p320.target_system_SET((char) 50);
		p320.target_component_SET((char) 72);
		p320.param_id_SET("DEMO", PH);
		p320.param_index_SET((short) -1879);
		CommunicationChannel.instance.send(p320); //===============================
		PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_LIST();
		PH.setPack(p321);
		p321.target_system_SET((char) 192);
		p321.target_component_SET((char) 32);
		CommunicationChannel.instance.send(p321); //===============================
		PARAM_EXT_VALUE p322 = CommunicationChannel.instance.new_PARAM_EXT_VALUE();
		PH.setPack(p322);
		p322.param_id_SET("DEMO", PH);
		p322.param_value_SET("DEMO", PH);
		p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
		p322.param_count_SET((char) 30893);
		p322.param_index_SET((char) 37092);
		CommunicationChannel.instance.send(p322); //===============================
		PARAM_EXT_SET p323 = CommunicationChannel.instance.new_PARAM_EXT_SET();
		PH.setPack(p323);
		p323.target_system_SET((char) 173);
		p323.target_component_SET((char) 139);
		p323.param_id_SET("DEMO", PH);
		p323.param_value_SET("DEMO", PH);
		p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
		CommunicationChannel.instance.send(p323); //===============================
		PARAM_EXT_ACK p324 = CommunicationChannel.instance.new_PARAM_EXT_ACK();
		PH.setPack(p324);
		p324.param_id_SET("DEMO", PH);
		p324.param_value_SET("DEMO", PH);
		p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
		p324.param_result_SET(PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
		CommunicationChannel.instance.send(p324); //===============================
		OBSTACLE_DISTANCE p330 = CommunicationChannel.instance.new_OBSTACLE_DISTANCE();
		PH.setPack(p330);
		p330.time_usec_SET(6349307624244044231L);
		p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
		p330.distances_SET(new char[72], 0);
		p330.increment_SET((char) 216);
		p330.min_distance_SET((char) 9419);
		p330.max_distance_SET((char) 7974);
		CommunicationChannel.instance.send(p330); //===============================
	}
}
