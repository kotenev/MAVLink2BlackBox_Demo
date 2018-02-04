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
		POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.instance.new_POSITION_TARGET_LOCAL_NED();
		PH.setPack(p3);
		p3.time_boot_ms_SET(1773621817L);
		p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED);
		p3.type_mask_SET((char) 5244);
		p3.x_SET(-2.4410038E38F);
		p3.y_SET(-2.2886332E38F);
		p3.z_SET(2.7860399E38F);
		p3.vx_SET(1.8875145E38F);
		p3.vy_SET(1.037812E38F);
		p3.vz_SET(-2.885554E38F);
		p3.afx_SET(2.3154125E38F);
		p3.afy_SET(1.9391164E38F);
		p3.afz_SET(4.381359E36F);
		p3.yaw_SET(-2.7023838E38F);
		p3.yaw_rate_SET(1.0628898E38F);
		CommunicationChannel.instance.send(p3); //===============================
		VFR_HUD p74 = CommunicationChannel.instance.new_VFR_HUD();
		PH.setPack(p74);
		p74.airspeed_SET(-3.0886376E38F);
		p74.groundspeed_SET(1.8790663E38F);
		p74.heading_SET((short) -31907);
		p74.throttle_SET((char) 58704);
		p74.alt_SET(2.8914733E38F);
		p74.climb_SET(6.653755E37F);
		CommunicationChannel.instance.send(p74); //===============================
		COMMAND_INT p75 = CommunicationChannel.instance.new_COMMAND_INT();
		PH.setPack(p75);
		p75.target_system_SET((char) 225);
		p75.target_component_SET((char) 118);
		p75.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
		p75.command_SET(MAV_CMD.MAV_CMD_USER_2);
		p75.current_SET((char) 204);
		p75.autocontinue_SET((char) 211);
		p75.param1_SET(-9.445333E37F);
		p75.param2_SET(-7.7883444E37F);
		p75.param3_SET(-3.2130322E38F);
		p75.param4_SET(-1.5983367E38F);
		p75.x_SET(1177867259);
		p75.y_SET(-2000404133);
		p75.z_SET(-8.602738E37F);
		CommunicationChannel.instance.send(p75); //===============================
		COMMAND_LONG p76 = CommunicationChannel.instance.new_COMMAND_LONG();
		PH.setPack(p76);
		p76.target_system_SET((char) 87);
		p76.target_component_SET((char) 212);
		p76.command_SET(MAV_CMD.MAV_CMD_GET_HOME_POSITION);
		p76.confirmation_SET((char) 160);
		p76.param1_SET(-1.5182086E38F);
		p76.param2_SET(-9.128329E37F);
		p76.param3_SET(-1.0051099E38F);
		p76.param4_SET(-2.7949836E38F);
		p76.param5_SET(2.6815593E38F);
		p76.param6_SET(2.7500927E38F);
		p76.param7_SET(2.6474015E38F);
		CommunicationChannel.instance.send(p76); //===============================
		COMMAND_ACK p77 = CommunicationChannel.instance.new_COMMAND_ACK();
		PH.setPack(p77);
		p77.command_SET(MAV_CMD.MAV_CMD_NAV_DELAY);
		p77.result_SET(MAV_RESULT.MAV_RESULT_DENIED);
		p77.progress_SET((char) 214, PH);
		p77.result_param2_SET(-267156427, PH);
		p77.target_system_SET((char) 39, PH);
		p77.target_component_SET((char) 9, PH);
		CommunicationChannel.instance.send(p77); //===============================
		MANUAL_SETPOINT p81 = CommunicationChannel.instance.new_MANUAL_SETPOINT();
		PH.setPack(p81);
		p81.time_boot_ms_SET(939692813L);
		p81.roll_SET(3.1075347E38F);
		p81.pitch_SET(-2.6024753E38F);
		p81.yaw_SET(-6.591998E37F);
		p81.thrust_SET(1.9676744E38F);
		p81.mode_switch_SET((char) 203);
		p81.manual_override_switch_SET((char) 77);
		CommunicationChannel.instance.send(p81); //===============================
		SET_ATTITUDE_TARGET p82 = CommunicationChannel.instance.new_SET_ATTITUDE_TARGET();
		PH.setPack(p82);
		p82.time_boot_ms_SET(3328877988L);
		p82.target_system_SET((char) 7);
		p82.target_component_SET((char) 171);
		p82.type_mask_SET((char) 124);
		p82.q_SET(new float[4], 0);
		p82.body_roll_rate_SET(1.5409765E38F);
		p82.body_pitch_rate_SET(6.0819406E37F);
		p82.body_yaw_rate_SET(1.9505193E37F);
		p82.thrust_SET(4.891225E37F);
		CommunicationChannel.instance.send(p82); //===============================
		ATTITUDE_TARGET p83 = CommunicationChannel.instance.new_ATTITUDE_TARGET();
		PH.setPack(p83);
		p83.time_boot_ms_SET(2111313953L);
		p83.type_mask_SET((char) 251);
		p83.q_SET(new float[4], 0);
		p83.body_roll_rate_SET(3.363215E38F);
		p83.body_pitch_rate_SET(5.022514E37F);
		p83.body_yaw_rate_SET(-2.2758204E38F);
		p83.thrust_SET(2.6276158E38F);
		CommunicationChannel.instance.send(p83); //===============================
		SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.instance.new_SET_POSITION_TARGET_LOCAL_NED();
		PH.setPack(p84);
		p84.time_boot_ms_SET(1636236334L);
		p84.target_system_SET((char) 79);
		p84.target_component_SET((char) 31);
		p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
		p84.type_mask_SET((char) 50465);
		p84.x_SET(1.51551E38F);
		p84.y_SET(2.914747E38F);
		p84.z_SET(2.9609939E38F);
		p84.vx_SET(1.9995739E37F);
		p84.vy_SET(-5.5286794E36F);
		p84.vz_SET(-2.4085716E38F);
		p84.afx_SET(-1.9830455E38F);
		p84.afy_SET(8.526604E37F);
		p84.afz_SET(2.5164966E38F);
		p84.yaw_SET(1.9314308E38F);
		p84.yaw_rate_SET(2.1446707E38F);
		CommunicationChannel.instance.send(p84); //===============================
		SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.instance.new_SET_POSITION_TARGET_GLOBAL_INT();
		PH.setPack(p86);
		p86.time_boot_ms_SET(2298607339L);
		p86.target_system_SET((char) 121);
		p86.target_component_SET((char) 34);
		p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL);
		p86.type_mask_SET((char) 62017);
		p86.lat_int_SET(-861926626);
		p86.lon_int_SET(520223216);
		p86.alt_SET(-2.426924E38F);
		p86.vx_SET(-9.769538E37F);
		p86.vy_SET(1.315932E38F);
		p86.vz_SET(-1.3334627E38F);
		p86.afx_SET(-2.63723E38F);
		p86.afy_SET(-1.0199023E38F);
		p86.afz_SET(3.2107121E38F);
		p86.yaw_SET(2.1832168E38F);
		p86.yaw_rate_SET(1.1697003E37F);
		CommunicationChannel.instance.send(p86); //===============================
		POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.instance.new_POSITION_TARGET_GLOBAL_INT();
		PH.setPack(p87);
		p87.time_boot_ms_SET(3781385785L);
		p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
		p87.type_mask_SET((char) 22874);
		p87.lat_int_SET(1271922333);
		p87.lon_int_SET(-1110267617);
		p87.alt_SET(-1.7401462E38F);
		p87.vx_SET(6.763033E36F);
		p87.vy_SET(-2.1095174E38F);
		p87.vz_SET(-1.7729317E38F);
		p87.afx_SET(-5.080733E37F);
		p87.afy_SET(-4.4197247E37F);
		p87.afz_SET(-5.4898464E37F);
		p87.yaw_SET(-2.9870055E38F);
		p87.yaw_rate_SET(2.9405912E38F);
		CommunicationChannel.instance.send(p87); //===============================
		LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.instance.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
		PH.setPack(p89);
		p89.time_boot_ms_SET(14485499L);
		p89.x_SET(-2.14188E38F);
		p89.y_SET(-1.0757636E38F);
		p89.z_SET(2.9766008E38F);
		p89.roll_SET(1.72301E38F);
		p89.pitch_SET(-2.4631786E38F);
		p89.yaw_SET(-2.2870439E38F);
		CommunicationChannel.instance.send(p89); //===============================
		HIL_STATE p90 = CommunicationChannel.instance.new_HIL_STATE();
		PH.setPack(p90);
		p90.time_usec_SET(2328718886022333461L);
		p90.roll_SET(-1.3694742E38F);
		p90.pitch_SET(1.4440708E38F);
		p90.yaw_SET(-1.9027618E38F);
		p90.rollspeed_SET(-1.909593E38F);
		p90.pitchspeed_SET(-2.195738E38F);
		p90.yawspeed_SET(1.6730388E38F);
		p90.lat_SET(1526175110);
		p90.lon_SET(869423329);
		p90.alt_SET(-1733013148);
		p90.vx_SET((short) 22163);
		p90.vy_SET((short) -2676);
		p90.vz_SET((short) 31213);
		p90.xacc_SET((short) 19303);
		p90.yacc_SET((short) 14252);
		p90.zacc_SET((short) -6758);
		CommunicationChannel.instance.send(p90); //===============================
		HIL_CONTROLS p91 = CommunicationChannel.instance.new_HIL_CONTROLS();
		PH.setPack(p91);
		p91.time_usec_SET(7643644050925112189L);
		p91.roll_ailerons_SET(4.9479643E37F);
		p91.pitch_elevator_SET(2.7700577E38F);
		p91.yaw_rudder_SET(-2.9731015E38F);
		p91.throttle_SET(-2.0429279E38F);
		p91.aux1_SET(-2.967904E38F);
		p91.aux2_SET(-2.4040894E38F);
		p91.aux3_SET(2.193782E38F);
		p91.aux4_SET(-2.0429942E38F);
		p91.mode_SET(MAV_MODE.MAV_MODE_TEST_ARMED);
		p91.nav_mode_SET((char) 16);
		CommunicationChannel.instance.send(p91); //===============================
		HIL_RC_INPUTS_RAW p92 = CommunicationChannel.instance.new_HIL_RC_INPUTS_RAW();
		PH.setPack(p92);
		p92.time_usec_SET(1255293666330208731L);
		p92.chan1_raw_SET((char) 28575);
		p92.chan2_raw_SET((char) 3384);
		p92.chan3_raw_SET((char) 53512);
		p92.chan4_raw_SET((char) 44703);
		p92.chan5_raw_SET((char) 52185);
		p92.chan6_raw_SET((char) 8341);
		p92.chan7_raw_SET((char) 59304);
		p92.chan8_raw_SET((char) 20421);
		p92.chan9_raw_SET((char) 63639);
		p92.chan10_raw_SET((char) 32728);
		p92.chan11_raw_SET((char) 60922);
		p92.chan12_raw_SET((char) 61810);
		p92.rssi_SET((char) 71);
		CommunicationChannel.instance.send(p92); //===============================
		HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.instance.new_HIL_ACTUATOR_CONTROLS();
		PH.setPack(p93);
		p93.time_usec_SET(2442571987924489438L);
		p93.controls_SET(new float[16], 0);
		p93.mode_SET(MAV_MODE.MAV_MODE_TEST_DISARMED);
		p93.flags_SET(3577386747859004718L);
		CommunicationChannel.instance.send(p93); //===============================
		OPTICAL_FLOW p100 = CommunicationChannel.instance.new_OPTICAL_FLOW();
		PH.setPack(p100);
		p100.time_usec_SET(6354015317456343781L);
		p100.sensor_id_SET((char) 183);
		p100.flow_x_SET((short) 27796);
		p100.flow_y_SET((short) -8180);
		p100.flow_comp_m_x_SET(-3.4400374E37F);
		p100.flow_comp_m_y_SET(2.8089953E38F);
		p100.quality_SET((char) 191);
		p100.ground_distance_SET(1.9198986E38F);
		p100.flow_rate_x_SET(-1.9500204E38F, PH);
		p100.flow_rate_y_SET(-2.3368275E38F, PH);
		CommunicationChannel.instance.send(p100); //===============================
		GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.instance.new_GLOBAL_VISION_POSITION_ESTIMATE();
		PH.setPack(p101);
		p101.usec_SET(7877195442371012237L);
		p101.x_SET(-1.6218678E38F);
		p101.y_SET(5.6134687E37F);
		p101.z_SET(2.1600188E38F);
		p101.roll_SET(2.2249207E38F);
		p101.pitch_SET(1.0954416E38F);
		p101.yaw_SET(-1.2021385E38F);
		CommunicationChannel.instance.send(p101); //===============================
		VISION_POSITION_ESTIMATE p102 = CommunicationChannel.instance.new_VISION_POSITION_ESTIMATE();
		PH.setPack(p102);
		p102.usec_SET(7826677223842699522L);
		p102.x_SET(2.6207727E38F);
		p102.y_SET(9.83526E37F);
		p102.z_SET(1.8258584E38F);
		p102.roll_SET(2.7010022E38F);
		p102.pitch_SET(1.6404401E38F);
		p102.yaw_SET(3.3809248E38F);
		CommunicationChannel.instance.send(p102); //===============================
		VISION_SPEED_ESTIMATE p103 = CommunicationChannel.instance.new_VISION_SPEED_ESTIMATE();
		PH.setPack(p103);
		p103.usec_SET(1782387334734859765L);
		p103.x_SET(1.928845E38F);
		p103.y_SET(-1.6565091E38F);
		p103.z_SET(2.5141422E38F);
		CommunicationChannel.instance.send(p103); //===============================
		VICON_POSITION_ESTIMATE p104 = CommunicationChannel.instance.new_VICON_POSITION_ESTIMATE();
		PH.setPack(p104);
		p104.usec_SET(7445007231484253046L);
		p104.x_SET(-2.6577496E38F);
		p104.y_SET(-1.2941568E38F);
		p104.z_SET(-5.0619565E37F);
		p104.roll_SET(-2.4571186E38F);
		p104.pitch_SET(-3.2976233E38F);
		p104.yaw_SET(-8.791599E37F);
		CommunicationChannel.instance.send(p104); //===============================
		HIGHRES_IMU p105 = CommunicationChannel.instance.new_HIGHRES_IMU();
		PH.setPack(p105);
		p105.time_usec_SET(6044171387775559669L);
		p105.xacc_SET(2.5836818E38F);
		p105.yacc_SET(-1.8408775E38F);
		p105.zacc_SET(-3.2677313E38F);
		p105.xgyro_SET(-2.0492552E38F);
		p105.ygyro_SET(-2.4174052E38F);
		p105.zgyro_SET(2.3260768E38F);
		p105.xmag_SET(5.347217E37F);
		p105.ymag_SET(-1.276621E38F);
		p105.zmag_SET(1.8749617E38F);
		p105.abs_pressure_SET(-1.8998628E38F);
		p105.diff_pressure_SET(2.4450848E38F);
		p105.pressure_alt_SET(-5.301067E37F);
		p105.temperature_SET(-5.5248883E37F);
		p105.fields_updated_SET((char) 30838);
		CommunicationChannel.instance.send(p105); //===============================
		OPTICAL_FLOW_RAD p106 = CommunicationChannel.instance.new_OPTICAL_FLOW_RAD();
		PH.setPack(p106);
		p106.time_usec_SET(6109847380799401732L);
		p106.sensor_id_SET((char) 19);
		p106.integration_time_us_SET(2451238526L);
		p106.integrated_x_SET(-1.7316624E37F);
		p106.integrated_y_SET(9.13707E37F);
		p106.integrated_xgyro_SET(2.9556371E38F);
		p106.integrated_ygyro_SET(2.9308745E38F);
		p106.integrated_zgyro_SET(-2.8076163E38F);
		p106.temperature_SET((short) -11887);
		p106.quality_SET((char) 67);
		p106.time_delta_distance_us_SET(4236744123L);
		p106.distance_SET(-2.4331876E38F);
		CommunicationChannel.instance.send(p106); //===============================
		HIL_SENSOR p107 = CommunicationChannel.instance.new_HIL_SENSOR();
		PH.setPack(p107);
		p107.time_usec_SET(7430160690773001018L);
		p107.xacc_SET(-2.4201657E38F);
		p107.yacc_SET(-1.6954224E38F);
		p107.zacc_SET(-1.768947E38F);
		p107.xgyro_SET(-1.6192784E38F);
		p107.ygyro_SET(9.150584E37F);
		p107.zgyro_SET(1.6463795E38F);
		p107.xmag_SET(9.634691E37F);
		p107.ymag_SET(1.7588678E38F);
		p107.zmag_SET(-3.0185692E38F);
		p107.abs_pressure_SET(2.7442798E38F);
		p107.diff_pressure_SET(9.564202E37F);
		p107.pressure_alt_SET(-1.6055451E38F);
		p107.temperature_SET(-2.0320849E38F);
		p107.fields_updated_SET(3995060323L);
		CommunicationChannel.instance.send(p107); //===============================
		SIM_STATE p108 = CommunicationChannel.instance.new_SIM_STATE();
		PH.setPack(p108);
		p108.q1_SET(-9.558458E37F);
		p108.q2_SET(1.2622471E38F);
		p108.q3_SET(-8.547165E37F);
		p108.q4_SET(1.1453335E38F);
		p108.roll_SET(1.705972E38F);
		p108.pitch_SET(-1.1432513E38F);
		p108.yaw_SET(1.0331216E38F);
		p108.xacc_SET(-2.4683737E38F);
		p108.yacc_SET(-6.5155897E37F);
		p108.zacc_SET(1.1279143E38F);
		p108.xgyro_SET(-2.3302552E38F);
		p108.ygyro_SET(3.2164036E38F);
		p108.zgyro_SET(1.9319218E38F);
		p108.lat_SET(-2.4335028E38F);
		p108.lon_SET(-2.9373766E38F);
		p108.alt_SET(-1.7186924E38F);
		p108.std_dev_horz_SET(2.4867778E38F);
		p108.std_dev_vert_SET(3.3968787E38F);
		p108.vn_SET(1.1120591E38F);
		p108.ve_SET(1.6116791E38F);
		p108.vd_SET(-1.3414055E38F);
		CommunicationChannel.instance.send(p108); //===============================
		RADIO_STATUS p109 = CommunicationChannel.instance.new_RADIO_STATUS();
		PH.setPack(p109);
		p109.rssi_SET((char) 77);
		p109.remrssi_SET((char) 62);
		p109.txbuf_SET((char) 202);
		p109.noise_SET((char) 137);
		p109.remnoise_SET((char) 194);
		p109.rxerrors_SET((char) 53807);
		p109.fixed__SET((char) 20532);
		CommunicationChannel.instance.send(p109); //===============================
		FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.instance.new_FILE_TRANSFER_PROTOCOL();
		PH.setPack(p110);
		p110.target_network_SET((char) 44);
		p110.target_system_SET((char) 108);
		p110.target_component_SET((char) 135);
		p110.payload_SET(new char[251], 0);
		CommunicationChannel.instance.send(p110); //===============================
		TIMESYNC p111 = CommunicationChannel.instance.new_TIMESYNC();
		PH.setPack(p111);
		p111.tc1_SET(-6705011991138585617L);
		p111.ts1_SET(7183296850545924807L);
		CommunicationChannel.instance.send(p111); //===============================
		CAMERA_TRIGGER p112 = CommunicationChannel.instance.new_CAMERA_TRIGGER();
		PH.setPack(p112);
		p112.time_usec_SET(3729477797053851375L);
		p112.seq_SET(2818858996L);
		CommunicationChannel.instance.send(p112); //===============================
		HIL_GPS p113 = CommunicationChannel.instance.new_HIL_GPS();
		PH.setPack(p113);
		p113.time_usec_SET(8166212834997379625L);
		p113.fix_type_SET((char) 116);
		p113.lat_SET(-1306058234);
		p113.lon_SET(2132667094);
		p113.alt_SET(-894404722);
		p113.eph_SET((char) 51195);
		p113.epv_SET((char) 61823);
		p113.vel_SET((char) 11804);
		p113.vn_SET((short) -23836);
		p113.ve_SET((short) -12909);
		p113.vd_SET((short) -21629);
		p113.cog_SET((char) 63494);
		p113.satellites_visible_SET((char) 89);
		CommunicationChannel.instance.send(p113); //===============================
		HIL_OPTICAL_FLOW p114 = CommunicationChannel.instance.new_HIL_OPTICAL_FLOW();
		PH.setPack(p114);
		p114.time_usec_SET(3544230643120119461L);
		p114.sensor_id_SET((char) 55);
		p114.integration_time_us_SET(786046783L);
		p114.integrated_x_SET(2.9972278E38F);
		p114.integrated_y_SET(3.128108E38F);
		p114.integrated_xgyro_SET(-2.3589836E38F);
		p114.integrated_ygyro_SET(2.1224444E38F);
		p114.integrated_zgyro_SET(2.219733E37F);
		p114.temperature_SET((short) -24282);
		p114.quality_SET((char) 70);
		p114.time_delta_distance_us_SET(4184006584L);
		p114.distance_SET(2.5938725E38F);
		CommunicationChannel.instance.send(p114); //===============================
		HIL_STATE_QUATERNION p115 = CommunicationChannel.instance.new_HIL_STATE_QUATERNION();
		PH.setPack(p115);
		p115.time_usec_SET(9191058849536618214L);
		p115.attitude_quaternion_SET(new float[4], 0);
		p115.rollspeed_SET(-1.0115143E38F);
		p115.pitchspeed_SET(7.608008E37F);
		p115.yawspeed_SET(2.4354162E38F);
		p115.lat_SET(803076004);
		p115.lon_SET(169602786);
		p115.alt_SET(-1753856808);
		p115.vx_SET((short) 5889);
		p115.vy_SET((short) 3621);
		p115.vz_SET((short) -19558);
		p115.ind_airspeed_SET((char) 31179);
		p115.true_airspeed_SET((char) 42716);
		p115.xacc_SET((short) 22316);
		p115.yacc_SET((short) 14469);
		p115.zacc_SET((short) -8164);
		CommunicationChannel.instance.send(p115); //===============================
		SCALED_IMU2 p116 = CommunicationChannel.instance.new_SCALED_IMU2();
		PH.setPack(p116);
		p116.time_boot_ms_SET(1502439030L);
		p116.xacc_SET((short) 30287);
		p116.yacc_SET((short) 7299);
		p116.zacc_SET((short) -6447);
		p116.xgyro_SET((short) -4380);
		p116.ygyro_SET((short) 29695);
		p116.zgyro_SET((short) 14399);
		p116.xmag_SET((short) -17888);
		p116.ymag_SET((short) 16752);
		p116.zmag_SET((short) -28591);
		CommunicationChannel.instance.send(p116); //===============================
		LOG_REQUEST_LIST p117 = CommunicationChannel.instance.new_LOG_REQUEST_LIST();
		PH.setPack(p117);
		p117.target_system_SET((char) 63);
		p117.target_component_SET((char) 13);
		p117.start_SET((char) 40079);
		p117.end_SET((char) 25013);
		CommunicationChannel.instance.send(p117); //===============================
		LOG_ENTRY p118 = CommunicationChannel.instance.new_LOG_ENTRY();
		PH.setPack(p118);
		p118.id_SET((char) 53561);
		p118.num_logs_SET((char) 56949);
		p118.last_log_num_SET((char) 19232);
		p118.time_utc_SET(2401598522L);
		p118.size_SET(2490820462L);
		CommunicationChannel.instance.send(p118); //===============================
		LOG_REQUEST_DATA p119 = CommunicationChannel.instance.new_LOG_REQUEST_DATA();
		PH.setPack(p119);
		p119.target_system_SET((char) 186);
		p119.target_component_SET((char) 82);
		p119.id_SET((char) 32634);
		p119.ofs_SET(517009420L);
		p119.count_SET(3798115737L);
		CommunicationChannel.instance.send(p119); //===============================
		LOG_DATA p120 = CommunicationChannel.instance.new_LOG_DATA();
		PH.setPack(p120);
		p120.id_SET((char) 22608);
		p120.ofs_SET(3158076811L);
		p120.count_SET((char) 49);
		p120.data__SET(new char[90], 0);
		CommunicationChannel.instance.send(p120); //===============================
		LOG_ERASE p121 = CommunicationChannel.instance.new_LOG_ERASE();
		PH.setPack(p121);
		p121.target_system_SET((char) 192);
		p121.target_component_SET((char) 58);
		CommunicationChannel.instance.send(p121); //===============================
		LOG_REQUEST_END p122 = CommunicationChannel.instance.new_LOG_REQUEST_END();
		PH.setPack(p122);
		p122.target_system_SET((char) 123);
		p122.target_component_SET((char) 252);
		CommunicationChannel.instance.send(p122); //===============================
		GPS_INJECT_DATA p123 = CommunicationChannel.instance.new_GPS_INJECT_DATA();
		PH.setPack(p123);
		p123.target_system_SET((char) 207);
		p123.target_component_SET((char) 159);
		p123.len_SET((char) 118);
		p123.data__SET(new char[110], 0);
		CommunicationChannel.instance.send(p123); //===============================
		GPS2_RAW p124 = CommunicationChannel.instance.new_GPS2_RAW();
		PH.setPack(p124);
		p124.time_usec_SET(6453118806902844136L);
		p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX);
		p124.lat_SET(-1178643106);
		p124.lon_SET(1329936092);
		p124.alt_SET(2028838887);
		p124.eph_SET((char) 37070);
		p124.epv_SET((char) 44805);
		p124.vel_SET((char) 23637);
		p124.cog_SET((char) 56202);
		p124.satellites_visible_SET((char) 23);
		p124.dgps_numch_SET((char) 61);
		p124.dgps_age_SET(3405286512L);
		CommunicationChannel.instance.send(p124); //===============================
		POWER_STATUS p125 = CommunicationChannel.instance.new_POWER_STATUS();
		PH.setPack(p125);
		p125.Vcc_SET((char) 34483);
		p125.Vservo_SET((char) 45871);
		p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT);
		CommunicationChannel.instance.send(p125); //===============================
		SERIAL_CONTROL p126 = CommunicationChannel.instance.new_SERIAL_CONTROL();
		PH.setPack(p126);
		p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
		p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
		p126.timeout_SET((char) 17508);
		p126.baudrate_SET(2445168667L);
		p126.count_SET((char) 206);
		p126.data__SET(new char[70], 0);
		CommunicationChannel.instance.send(p126); //===============================
		GPS_RTK p127 = CommunicationChannel.instance.new_GPS_RTK();
		PH.setPack(p127);
		p127.time_last_baseline_ms_SET(2774861457L);
		p127.rtk_receiver_id_SET((char) 242);
		p127.wn_SET((char) 32296);
		p127.tow_SET(1985362752L);
		p127.rtk_health_SET((char) 59);
		p127.rtk_rate_SET((char) 237);
		p127.nsats_SET((char) 207);
		p127.baseline_coords_type_SET((char) 208);
		p127.baseline_a_mm_SET(-1089893848);
		p127.baseline_b_mm_SET(1214629336);
		p127.baseline_c_mm_SET(584425078);
		p127.accuracy_SET(3459282619L);
		p127.iar_num_hypotheses_SET(735385614);
		CommunicationChannel.instance.send(p127); //===============================
		GPS2_RTK p128 = CommunicationChannel.instance.new_GPS2_RTK();
		PH.setPack(p128);
		p128.time_last_baseline_ms_SET(4043744730L);
		p128.rtk_receiver_id_SET((char) 71);
		p128.wn_SET((char) 64);
		p128.tow_SET(201058497L);
		p128.rtk_health_SET((char) 24);
		p128.rtk_rate_SET((char) 97);
		p128.nsats_SET((char) 169);
		p128.baseline_coords_type_SET((char) 66);
		p128.baseline_a_mm_SET(-1826900354);
		p128.baseline_b_mm_SET(1311714256);
		p128.baseline_c_mm_SET(1178001142);
		p128.accuracy_SET(1363412813L);
		p128.iar_num_hypotheses_SET(1066067124);
		CommunicationChannel.instance.send(p128); //===============================
		SCALED_IMU3 p129 = CommunicationChannel.instance.new_SCALED_IMU3();
		PH.setPack(p129);
		p129.time_boot_ms_SET(88387657L);
		p129.xacc_SET((short) 22153);
		p129.yacc_SET((short) -31878);
		p129.zacc_SET((short) 3423);
		p129.xgyro_SET((short) 20683);
		p129.ygyro_SET((short) 11850);
		p129.zgyro_SET((short) -25126);
		p129.xmag_SET((short) 7170);
		p129.ymag_SET((short) 29139);
		p129.zmag_SET((short) -8822);
		CommunicationChannel.instance.send(p129); //===============================
		DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.instance.new_DATA_TRANSMISSION_HANDSHAKE();
		PH.setPack(p130);
		p130.type_SET((char) 180);
		p130.size_SET(3992920421L);
		p130.width_SET((char) 1062);
		p130.height_SET((char) 36329);
		p130.packets_SET((char) 55691);
		p130.payload_SET((char) 20);
		p130.jpg_quality_SET((char) 234);
		CommunicationChannel.instance.send(p130); //===============================
		ENCAPSULATED_DATA p131 = CommunicationChannel.instance.new_ENCAPSULATED_DATA();
		PH.setPack(p131);
		p131.seqnr_SET((char) 4150);
		p131.data__SET(new char[253], 0);
		CommunicationChannel.instance.send(p131); //===============================
		DISTANCE_SENSOR p132 = CommunicationChannel.instance.new_DISTANCE_SENSOR();
		PH.setPack(p132);
		p132.time_boot_ms_SET(1237680638L);
		p132.min_distance_SET((char) 58680);
		p132.max_distance_SET((char) 33985);
		p132.current_distance_SET((char) 18696);
		p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
		p132.id_SET((char) 20);
		p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315);
		p132.covariance_SET((char) 147);
		CommunicationChannel.instance.send(p132); //===============================
		TERRAIN_REQUEST p133 = CommunicationChannel.instance.new_TERRAIN_REQUEST();
		PH.setPack(p133);
		p133.lat_SET(-99013538);
		p133.lon_SET(-461022693);
		p133.grid_spacing_SET((char) 5498);
		p133.mask_SET(1428895368786571086L);
		CommunicationChannel.instance.send(p133); //===============================
		TERRAIN_DATA p134 = CommunicationChannel.instance.new_TERRAIN_DATA();
		PH.setPack(p134);
		p134.lat_SET(401647035);
		p134.lon_SET(432047903);
		p134.grid_spacing_SET((char) 37588);
		p134.gridbit_SET((char) 187);
		p134.data__SET(new short[16], 0);
		CommunicationChannel.instance.send(p134); //===============================
		TERRAIN_CHECK p135 = CommunicationChannel.instance.new_TERRAIN_CHECK();
		PH.setPack(p135);
		p135.lat_SET(-1405813389);
		p135.lon_SET(310984293);
		CommunicationChannel.instance.send(p135); //===============================
		TERRAIN_REPORT p136 = CommunicationChannel.instance.new_TERRAIN_REPORT();
		PH.setPack(p136);
		p136.lat_SET(-1404121847);
		p136.lon_SET(1949356119);
		p136.spacing_SET((char) 42303);
		p136.terrain_height_SET(-7.8199165E37F);
		p136.current_height_SET(-1.6726852E38F);
		p136.pending_SET((char) 24586);
		p136.loaded_SET((char) 12871);
		CommunicationChannel.instance.send(p136); //===============================
		SCALED_PRESSURE2 p137 = CommunicationChannel.instance.new_SCALED_PRESSURE2();
		PH.setPack(p137);
		p137.time_boot_ms_SET(3761513233L);
		p137.press_abs_SET(-3.5889757E37F);
		p137.press_diff_SET(3.3407638E38F);
		p137.temperature_SET((short) 27282);
		CommunicationChannel.instance.send(p137); //===============================
		ATT_POS_MOCAP p138 = CommunicationChannel.instance.new_ATT_POS_MOCAP();
		PH.setPack(p138);
		p138.time_usec_SET(4160356159778022584L);
		p138.q_SET(new float[4], 0);
		p138.x_SET(3.1437834E38F);
		p138.y_SET(3.138935E37F);
		p138.z_SET(-2.8682557E36F);
		CommunicationChannel.instance.send(p138); //===============================
		SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.instance.new_SET_ACTUATOR_CONTROL_TARGET();
		PH.setPack(p139);
		p139.time_usec_SET(491946113199247728L);
		p139.group_mlx_SET((char) 204);
		p139.target_system_SET((char) 87);
		p139.target_component_SET((char) 216);
		p139.controls_SET(new float[8], 0);
		CommunicationChannel.instance.send(p139); //===============================
		ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.instance.new_ACTUATOR_CONTROL_TARGET();
		PH.setPack(p140);
		p140.time_usec_SET(1494188233389401430L);
		p140.group_mlx_SET((char) 219);
		p140.controls_SET(new float[8], 0);
		CommunicationChannel.instance.send(p140); //===============================
		ALTITUDE p141 = CommunicationChannel.instance.new_ALTITUDE();
		PH.setPack(p141);
		p141.time_usec_SET(1044502037813058060L);
		p141.altitude_monotonic_SET(-9.403039E37F);
		p141.altitude_amsl_SET(-5.864258E37F);
		p141.altitude_local_SET(-1.2772628E38F);
		p141.altitude_relative_SET(-2.9044482E38F);
		p141.altitude_terrain_SET(5.1521306E37F);
		p141.bottom_clearance_SET(2.0055127E38F);
		CommunicationChannel.instance.send(p141); //===============================
		RESOURCE_REQUEST p142 = CommunicationChannel.instance.new_RESOURCE_REQUEST();
		PH.setPack(p142);
		p142.request_id_SET((char) 84);
		p142.uri_type_SET((char) 147);
		p142.uri_SET(new char[120], 0);
		p142.transfer_type_SET((char) 237);
		p142.storage_SET(new char[120], 0);
		CommunicationChannel.instance.send(p142); //===============================
		SCALED_PRESSURE3 p143 = CommunicationChannel.instance.new_SCALED_PRESSURE3();
		PH.setPack(p143);
		p143.time_boot_ms_SET(2491584099L);
		p143.press_abs_SET(2.3379424E38F);
		p143.press_diff_SET(2.9492341E38F);
		p143.temperature_SET((short) -5352);
		CommunicationChannel.instance.send(p143); //===============================
		FOLLOW_TARGET p144 = CommunicationChannel.instance.new_FOLLOW_TARGET();
		PH.setPack(p144);
		p144.timestamp_SET(4991163200180504548L);
		p144.est_capabilities_SET((char) 60);
		p144.lat_SET(973253982);
		p144.lon_SET(156307236);
		p144.alt_SET(1.5168113E38F);
		p144.vel_SET(new float[3], 0);
		p144.acc_SET(new float[3], 0);
		p144.attitude_q_SET(new float[4], 0);
		p144.rates_SET(new float[3], 0);
		p144.position_cov_SET(new float[3], 0);
		p144.custom_state_SET(3159553271268527353L);
		CommunicationChannel.instance.send(p144); //===============================
		CONTROL_SYSTEM_STATE p146 = CommunicationChannel.instance.new_CONTROL_SYSTEM_STATE();
		PH.setPack(p146);
		p146.time_usec_SET(2351603611013133830L);
		p146.x_acc_SET(-5.607365E37F);
		p146.y_acc_SET(1.6654694E38F);
		p146.z_acc_SET(-1.0366048E38F);
		p146.x_vel_SET(3.1284236E38F);
		p146.y_vel_SET(-1.2660987E38F);
		p146.z_vel_SET(-3.3947574E38F);
		p146.x_pos_SET(6.3833246E37F);
		p146.y_pos_SET(-1.4369381E38F);
		p146.z_pos_SET(5.7489805E36F);
		p146.airspeed_SET(-1.7262334E38F);
		p146.vel_variance_SET(new float[3], 0);
		p146.pos_variance_SET(new float[3], 0);
		p146.q_SET(new float[4], 0);
		p146.roll_rate_SET(-5.4958277E37F);
		p146.pitch_rate_SET(9.543304E37F);
		p146.yaw_rate_SET(2.3298112E38F);
		CommunicationChannel.instance.send(p146); //===============================
		BATTERY_STATUS p147 = CommunicationChannel.instance.new_BATTERY_STATUS();
		PH.setPack(p147);
		p147.id_SET((char) 206);
		p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_UNKNOWN);
		p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO);
		p147.temperature_SET((short) 4763);
		p147.voltages_SET(new char[10], 0);
		p147.current_battery_SET((short) -3169);
		p147.current_consumed_SET(-1232302785);
		p147.energy_consumed_SET(1694422537);
		p147.battery_remaining_SET((byte) -109);
		CommunicationChannel.instance.send(p147); //===============================
		AUTOPILOT_VERSION p148 = CommunicationChannel.instance.new_AUTOPILOT_VERSION();
		PH.setPack(p148);
		p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT);
		p148.flight_sw_version_SET(2964037234L);
		p148.middleware_sw_version_SET(154351857L);
		p148.os_sw_version_SET(3535271063L);
		p148.board_version_SET(1523096493L);
		p148.flight_custom_version_SET(new char[8], 0);
		p148.middleware_custom_version_SET(new char[8], 0);
		p148.os_custom_version_SET(new char[8], 0);
		p148.vendor_id_SET((char) 22572);
		p148.product_id_SET((char) 39546);
		p148.uid_SET(4753829919058019585L);
		p148.uid2_SET(new char[18], 0, PH);
		CommunicationChannel.instance.send(p148); //===============================
		LANDING_TARGET p149 = CommunicationChannel.instance.new_LANDING_TARGET();
		PH.setPack(p149);
		p149.time_usec_SET(4014932984786607854L);
		p149.target_num_SET((char) 96);
		p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT);
		p149.angle_x_SET(5.5859247E37F);
		p149.angle_y_SET(1.7091372E38F);
		p149.distance_SET(3.3560825E38F);
		p149.size_x_SET(-1.9621803E38F);
		p149.size_y_SET(9.580905E37F);
		p149.x_SET(6.210751E37F, PH);
		p149.y_SET(-4.7638213E37F, PH);
		p149.z_SET(-1.1289712E38F, PH);
		p149.q_SET(new float[4], 0, PH);
		p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL);
		p149.position_valid_SET((char) 215, PH);
		CommunicationChannel.instance.send(p149); //===============================
		ESTIMATOR_STATUS p230 = CommunicationChannel.instance.new_ESTIMATOR_STATUS();
		PH.setPack(p230);
		p230.time_usec_SET(5535507230743409219L);
		p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL);
		p230.vel_ratio_SET(-1.3823625E38F);
		p230.pos_horiz_ratio_SET(3.0659454E36F);
		p230.pos_vert_ratio_SET(5.5601097E37F);
		p230.mag_ratio_SET(-3.86401E37F);
		p230.hagl_ratio_SET(2.4817195E38F);
		p230.tas_ratio_SET(-1.9502829E38F);
		p230.pos_horiz_accuracy_SET(2.7057937E38F);
		p230.pos_vert_accuracy_SET(-6.697701E37F);
		CommunicationChannel.instance.send(p230); //===============================
		WIND_COV p231 = CommunicationChannel.instance.new_WIND_COV();
		PH.setPack(p231);
		p231.time_usec_SET(2488068708688426762L);
		p231.wind_x_SET(-1.4296979E38F);
		p231.wind_y_SET(1.5990502E38F);
		p231.wind_z_SET(2.6465516E38F);
		p231.var_horiz_SET(4.5972525E37F);
		p231.var_vert_SET(-1.5645152E36F);
		p231.wind_alt_SET(1.5273317E38F);
		p231.horiz_accuracy_SET(4.0732403E37F);
		p231.vert_accuracy_SET(3.8955476E37F);
		CommunicationChannel.instance.send(p231); //===============================
		GPS_INPUT p232 = CommunicationChannel.instance.new_GPS_INPUT();
		PH.setPack(p232);
		p232.time_usec_SET(736106650045528560L);
		p232.gps_id_SET((char) 236);
		p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT);
		p232.time_week_ms_SET(1363934020L);
		p232.time_week_SET((char) 32962);
		p232.fix_type_SET((char) 158);
		p232.lat_SET(-494890604);
		p232.lon_SET(-1107713271);
		p232.alt_SET(-1.0461711E38F);
		p232.hdop_SET(-1.8139843E38F);
		p232.vdop_SET(-3.3649726E38F);
		p232.vn_SET(-1.2527619E38F);
		p232.ve_SET(-2.5402102E38F);
		p232.vd_SET(2.5833644E38F);
		p232.speed_accuracy_SET(-2.6347951E38F);
		p232.horiz_accuracy_SET(-2.8201358E38F);
		p232.vert_accuracy_SET(-1.673418E38F);
		p232.satellites_visible_SET((char) 186);
		CommunicationChannel.instance.send(p232); //===============================
		GPS_RTCM_DATA p233 = CommunicationChannel.instance.new_GPS_RTCM_DATA();
		PH.setPack(p233);
		p233.flags_SET((char) 242);
		p233.len_SET((char) 111);
		p233.data__SET(new char[180], 0);
		CommunicationChannel.instance.send(p233); //===============================
		HIGH_LATENCY p234 = CommunicationChannel.instance.new_HIGH_LATENCY();
		PH.setPack(p234);
		p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED);
		p234.custom_mode_SET(2114269821L);
		p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
		p234.roll_SET((short) 25254);
		p234.pitch_SET((short) 31702);
		p234.heading_SET((char) 4111);
		p234.throttle_SET((byte) -32);
		p234.heading_sp_SET((short) -20703);
		p234.latitude_SET(1558393488);
		p234.longitude_SET(1823773854);
		p234.altitude_amsl_SET((short) 3825);
		p234.altitude_sp_SET((short) 12772);
		p234.airspeed_SET((char) 188);
		p234.airspeed_sp_SET((char) 230);
		p234.groundspeed_SET((char) 191);
		p234.climb_rate_SET((byte) -108);
		p234.gps_nsat_SET((char) 154);
		p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
		p234.battery_remaining_SET((char) 75);
		p234.temperature_SET((byte) -57);
		p234.temperature_air_SET((byte) 120);
		p234.failsafe_SET((char) 84);
		p234.wp_num_SET((char) 139);
		p234.wp_distance_SET((char) 33691);
		CommunicationChannel.instance.send(p234); //===============================
		VIBRATION p241 = CommunicationChannel.instance.new_VIBRATION();
		PH.setPack(p241);
		p241.time_usec_SET(5377674885912053145L);
		p241.vibration_x_SET(2.6781052E38F);
		p241.vibration_y_SET(-3.1054432E38F);
		p241.vibration_z_SET(2.5571088E38F);
		p241.clipping_0_SET(3309157517L);
		p241.clipping_1_SET(2468967591L);
		p241.clipping_2_SET(2613107600L);
		CommunicationChannel.instance.send(p241); //===============================
		HOME_POSITION p242 = CommunicationChannel.instance.new_HOME_POSITION();
		PH.setPack(p242);
		p242.latitude_SET(829488086);
		p242.longitude_SET(-601193844);
		p242.altitude_SET(-1827470816);
		p242.x_SET(-1.8426839E37F);
		p242.y_SET(2.2805199E38F);
		p242.z_SET(-2.2255087E38F);
		p242.q_SET(new float[4], 0);
		p242.approach_x_SET(-1.2137434E38F);
		p242.approach_y_SET(1.7042126E38F);
		p242.approach_z_SET(3.2632006E38F);
		p242.time_usec_SET(4671976574926981952L, PH);
		CommunicationChannel.instance.send(p242); //===============================
		SET_HOME_POSITION p243 = CommunicationChannel.instance.new_SET_HOME_POSITION();
		PH.setPack(p243);
		p243.target_system_SET((char) 41);
		p243.latitude_SET(714145074);
		p243.longitude_SET(587378644);
		p243.altitude_SET(-348518803);
		p243.x_SET(1.8286332E38F);
		p243.y_SET(-2.418535E38F);
		p243.z_SET(2.6590417E38F);
		p243.q_SET(new float[4], 0);
		p243.approach_x_SET(-1.428606E38F);
		p243.approach_y_SET(3.2417706E38F);
		p243.approach_z_SET(-7.0048684E37F);
		p243.time_usec_SET(1375551010088342675L, PH);
		CommunicationChannel.instance.send(p243); //===============================
		MESSAGE_INTERVAL p244 = CommunicationChannel.instance.new_MESSAGE_INTERVAL();
		PH.setPack(p244);
		p244.message_id_SET((char) 35103);
		p244.interval_us_SET(642587431);
		CommunicationChannel.instance.send(p244); //===============================
		EXTENDED_SYS_STATE p245 = CommunicationChannel.instance.new_EXTENDED_SYS_STATE();
		PH.setPack(p245);
		p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED);
		p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
		CommunicationChannel.instance.send(p245); //===============================
		ADSB_VEHICLE p246 = CommunicationChannel.instance.new_ADSB_VEHICLE();
		PH.setPack(p246);
		p246.ICAO_address_SET(3161974853L);
		p246.lat_SET(520493485);
		p246.lon_SET(1577329851);
		p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
		p246.altitude_SET(-1960371655);
		p246.heading_SET((char) 14215);
		p246.hor_velocity_SET((char) 53249);
		p246.ver_velocity_SET((short) -24724);
		p246.callsign_SET("DEMO", PH);
		p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED2);
		p246.tslc_SET((char) 107);
		p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE);
		p246.squawk_SET((char) 3529);
		CommunicationChannel.instance.send(p246); //===============================
		COLLISION p247 = CommunicationChannel.instance.new_COLLISION();
		PH.setPack(p247);
		p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
		p247.id_SET(1576864888L);
		p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_MOVE_PERPENDICULAR);
		p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
		p247.time_to_minimum_delta_SET(2.1088236E38F);
		p247.altitude_minimum_delta_SET(2.2829692E38F);
		p247.horizontal_minimum_delta_SET(-2.6312697E38F);
		CommunicationChannel.instance.send(p247); //===============================
		V2_EXTENSION p248 = CommunicationChannel.instance.new_V2_EXTENSION();
		PH.setPack(p248);
		p248.target_network_SET((char) 42);
		p248.target_system_SET((char) 143);
		p248.target_component_SET((char) 112);
		p248.message_type_SET((char) 25492);
		p248.payload_SET(new char[249], 0);
		CommunicationChannel.instance.send(p248); //===============================
		MEMORY_VECT p249 = CommunicationChannel.instance.new_MEMORY_VECT();
		PH.setPack(p249);
		p249.address_SET((char) 24338);
		p249.ver_SET((char) 232);
		p249.type_SET((char) 158);
		p249.value_SET(new byte[32], 0);
		CommunicationChannel.instance.send(p249); //===============================
		DEBUG_VECT p250 = CommunicationChannel.instance.new_DEBUG_VECT();
		PH.setPack(p250);
		p250.name_SET("DEMO", PH);
		p250.time_usec_SET(4024264627970120833L);
		p250.x_SET(-3.2367608E38F);
		p250.y_SET(1.7443284E38F);
		p250.z_SET(1.2875059E38F);
		CommunicationChannel.instance.send(p250); //===============================
		NAMED_VALUE_FLOAT p251 = CommunicationChannel.instance.new_NAMED_VALUE_FLOAT();
		PH.setPack(p251);
		p251.time_boot_ms_SET(4194511652L);
		p251.name_SET("DEMO", PH);
		p251.value_SET(-1.3731514E38F);
		CommunicationChannel.instance.send(p251); //===============================
		NAMED_VALUE_INT p252 = CommunicationChannel.instance.new_NAMED_VALUE_INT();
		PH.setPack(p252);
		p252.time_boot_ms_SET(2646431123L);
		p252.name_SET("DEMO", PH);
		p252.value_SET(-1080398238);
		CommunicationChannel.instance.send(p252); //===============================
		STATUSTEXT p253 = CommunicationChannel.instance.new_STATUSTEXT();
		PH.setPack(p253);
		p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_INFO);
		p253.text_SET("DEMO", PH);
		CommunicationChannel.instance.send(p253); //===============================
		DEBUG p254 = CommunicationChannel.instance.new_DEBUG();
		PH.setPack(p254);
		p254.time_boot_ms_SET(3012414303L);
		p254.ind_SET((char) 200);
		p254.value_SET(1.8510417E38F);
		CommunicationChannel.instance.send(p254); //===============================
		SETUP_SIGNING p256 = CommunicationChannel.instance.new_SETUP_SIGNING();
		PH.setPack(p256);
		p256.target_system_SET((char) 47);
		p256.target_component_SET((char) 42);
		p256.secret_key_SET(new char[32], 0);
		p256.initial_timestamp_SET(6385002811459305738L);
		CommunicationChannel.instance.send(p256); //===============================
		BUTTON_CHANGE p257 = CommunicationChannel.instance.new_BUTTON_CHANGE();
		PH.setPack(p257);
		p257.time_boot_ms_SET(311120860L);
		p257.last_change_ms_SET(403561462L);
		p257.state_SET((char) 128);
		CommunicationChannel.instance.send(p257); //===============================
		PLAY_TUNE p258 = CommunicationChannel.instance.new_PLAY_TUNE();
		PH.setPack(p258);
		p258.target_system_SET((char) 58);
		p258.target_component_SET((char) 214);
		p258.tune_SET("DEMO", PH);
		CommunicationChannel.instance.send(p258); //===============================
		CAMERA_INFORMATION p259 = CommunicationChannel.instance.new_CAMERA_INFORMATION();
		PH.setPack(p259);
		p259.time_boot_ms_SET(3669414901L);
		p259.vendor_name_SET(new char[32], 0);
		p259.model_name_SET(new char[32], 0);
		p259.firmware_version_SET(698237701L);
		p259.focal_length_SET(3.1840021E37F);
		p259.sensor_size_h_SET(2.5158816E38F);
		p259.sensor_size_v_SET(1.8536755E37F);
		p259.resolution_h_SET((char) 63965);
		p259.resolution_v_SET((char) 2934);
		p259.lens_id_SET((char) 250);
		p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES);
		p259.cam_definition_version_SET((char) 65525);
		p259.cam_definition_uri_SET("DEMO", PH);
		CommunicationChannel.instance.send(p259); //===============================
		CAMERA_SETTINGS p260 = CommunicationChannel.instance.new_CAMERA_SETTINGS();
		PH.setPack(p260);
		p260.time_boot_ms_SET(2519577413L);
		p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
		CommunicationChannel.instance.send(p260); //===============================
		STORAGE_INFORMATION p261 = CommunicationChannel.instance.new_STORAGE_INFORMATION();
		PH.setPack(p261);
		p261.time_boot_ms_SET(3261358968L);
		p261.storage_id_SET((char) 55);
		p261.storage_count_SET((char) 151);
		p261.status_SET((char) 74);
		p261.total_capacity_SET(-1.5714688E38F);
		p261.used_capacity_SET(1.1732564E38F);
		p261.available_capacity_SET(-9.567423E37F);
		p261.read_speed_SET(-2.9487776E38F);
		p261.write_speed_SET(2.4812914E38F);
		CommunicationChannel.instance.send(p261); //===============================
		CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.instance.new_CAMERA_CAPTURE_STATUS();
		PH.setPack(p262);
		p262.time_boot_ms_SET(611083151L);
		p262.image_status_SET((char) 88);
		p262.video_status_SET((char) 239);
		p262.image_interval_SET(-4.656542E37F);
		p262.recording_time_ms_SET(3538414281L);
		p262.available_capacity_SET(-2.3254332E38F);
		CommunicationChannel.instance.send(p262); //===============================
		CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.instance.new_CAMERA_IMAGE_CAPTURED();
		PH.setPack(p263);
		p263.time_boot_ms_SET(3865875401L);
		p263.time_utc_SET(1323879993128684775L);
		p263.camera_id_SET((char) 212);
		p263.lat_SET(-446541690);
		p263.lon_SET(-2142511805);
		p263.alt_SET(1448488135);
		p263.relative_alt_SET(-2051102341);
		p263.q_SET(new float[4], 0);
		p263.image_index_SET(1001277149);
		p263.capture_result_SET((byte) 70);
		p263.file_url_SET("DEMO", PH);
		CommunicationChannel.instance.send(p263); //===============================
		FLIGHT_INFORMATION p264 = CommunicationChannel.instance.new_FLIGHT_INFORMATION();
		PH.setPack(p264);
		p264.time_boot_ms_SET(4242119149L);
		p264.arming_time_utc_SET(3435252697116633460L);
		p264.takeoff_time_utc_SET(5393443364894430057L);
		p264.flight_uuid_SET(508712180790980113L);
		CommunicationChannel.instance.send(p264); //===============================
		MOUNT_ORIENTATION p265 = CommunicationChannel.instance.new_MOUNT_ORIENTATION();
		PH.setPack(p265);
		p265.time_boot_ms_SET(1093579592L);
		p265.roll_SET(1.8990586E38F);
		p265.pitch_SET(2.2948622E38F);
		p265.yaw_SET(3.3254245E37F);
		CommunicationChannel.instance.send(p265); //===============================
		LOGGING_DATA p266 = CommunicationChannel.instance.new_LOGGING_DATA();
		PH.setPack(p266);
		p266.target_system_SET((char) 65);
		p266.target_component_SET((char) 232);
		p266.sequence_SET((char) 50933);
		p266.length_SET((char) 41);
		p266.first_message_offset_SET((char) 77);
		p266.data__SET(new char[249], 0);
		CommunicationChannel.instance.send(p266); //===============================
		LOGGING_DATA_ACKED p267 = CommunicationChannel.instance.new_LOGGING_DATA_ACKED();
		PH.setPack(p267);
		p267.target_system_SET((char) 92);
		p267.target_component_SET((char) 169);
		p267.sequence_SET((char) 46126);
		p267.length_SET((char) 162);
		p267.first_message_offset_SET((char) 252);
		p267.data__SET(new char[249], 0);
		CommunicationChannel.instance.send(p267); //===============================
		LOGGING_ACK p268 = CommunicationChannel.instance.new_LOGGING_ACK();
		PH.setPack(p268);
		p268.target_system_SET((char) 81);
		p268.target_component_SET((char) 229);
		p268.sequence_SET((char) 7347);
		CommunicationChannel.instance.send(p268); //===============================
		VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.instance.new_VIDEO_STREAM_INFORMATION();
		PH.setPack(p269);
		p269.camera_id_SET((char) 98);
		p269.status_SET((char) 11);
		p269.framerate_SET(-2.3167407E37F);
		p269.resolution_h_SET((char) 15109);
		p269.resolution_v_SET((char) 36105);
		p269.bitrate_SET(1008317586L);
		p269.rotation_SET((char) 3144);
		p269.uri_SET("DEMO", PH);
		CommunicationChannel.instance.send(p269); //===============================
		SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.instance.new_SET_VIDEO_STREAM_SETTINGS();
		PH.setPack(p270);
		p270.target_system_SET((char) 31);
		p270.target_component_SET((char) 85);
		p270.camera_id_SET((char) 172);
		p270.framerate_SET(-7.3640303E37F);
		p270.resolution_h_SET((char) 16991);
		p270.resolution_v_SET((char) 23749);
		p270.bitrate_SET(3446533617L);
		p270.rotation_SET((char) 540);
		p270.uri_SET("DEMO", PH);
		CommunicationChannel.instance.send(p270); //===============================
		WIFI_CONFIG_AP p299 = CommunicationChannel.instance.new_WIFI_CONFIG_AP();
		PH.setPack(p299);
		p299.ssid_SET("DEMO", PH);
		p299.password_SET("DEMO", PH);
		CommunicationChannel.instance.send(p299); //===============================
		PROTOCOL_VERSION p300 = CommunicationChannel.instance.new_PROTOCOL_VERSION();
		PH.setPack(p300);
		p300.version_SET((char) 18176);
		p300.min_version_SET((char) 28908);
		p300.max_version_SET((char) 30541);
		p300.spec_version_hash_SET(new char[8], 0);
		p300.library_version_hash_SET(new char[8], 0);
		CommunicationChannel.instance.send(p300); //===============================
		UAVCAN_NODE_STATUS p310 = CommunicationChannel.instance.new_UAVCAN_NODE_STATUS();
		PH.setPack(p310);
		p310.time_usec_SET(3231949903452259295L);
		p310.uptime_sec_SET(3248520748L);
		p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
		p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
		p310.sub_mode_SET((char) 229);
		p310.vendor_specific_status_code_SET((char) 26487);
		CommunicationChannel.instance.send(p310); //===============================
		UAVCAN_NODE_INFO p311 = CommunicationChannel.instance.new_UAVCAN_NODE_INFO();
		PH.setPack(p311);
		p311.time_usec_SET(7218308609652932464L);
		p311.uptime_sec_SET(4022358619L);
		p311.name_SET("DEMO", PH);
		p311.hw_version_major_SET((char) 86);
		p311.hw_version_minor_SET((char) 113);
		p311.hw_unique_id_SET(new char[16], 0);
		p311.sw_version_major_SET((char) 13);
		p311.sw_version_minor_SET((char) 81);
		p311.sw_vcs_commit_SET(3158120518L);
		CommunicationChannel.instance.send(p311); //===============================
		PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_READ();
		PH.setPack(p320);
		p320.target_system_SET((char) 32);
		p320.target_component_SET((char) 176);
		p320.param_id_SET("DEMO", PH);
		p320.param_index_SET((short) 7887);
		CommunicationChannel.instance.send(p320); //===============================
		PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_LIST();
		PH.setPack(p321);
		p321.target_system_SET((char) 133);
		p321.target_component_SET((char) 179);
		CommunicationChannel.instance.send(p321); //===============================
		PARAM_EXT_VALUE p322 = CommunicationChannel.instance.new_PARAM_EXT_VALUE();
		PH.setPack(p322);
		p322.param_id_SET("DEMO", PH);
		p322.param_value_SET("DEMO", PH);
		p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
		p322.param_count_SET((char) 51976);
		p322.param_index_SET((char) 43095);
		CommunicationChannel.instance.send(p322); //===============================
		PARAM_EXT_SET p323 = CommunicationChannel.instance.new_PARAM_EXT_SET();
		PH.setPack(p323);
		p323.target_system_SET((char) 18);
		p323.target_component_SET((char) 66);
		p323.param_id_SET("DEMO", PH);
		p323.param_value_SET("DEMO", PH);
		p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
		CommunicationChannel.instance.send(p323); //===============================
		PARAM_EXT_ACK p324 = CommunicationChannel.instance.new_PARAM_EXT_ACK();
		PH.setPack(p324);
		p324.param_id_SET("DEMO", PH);
		p324.param_value_SET("DEMO", PH);
		p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
		p324.param_result_SET(PARAM_ACK.PARAM_ACK_FAILED);
		CommunicationChannel.instance.send(p324); //===============================
		OBSTACLE_DISTANCE p330 = CommunicationChannel.instance.new_OBSTACLE_DISTANCE();
		PH.setPack(p330);
		p330.time_usec_SET(1983904474692237074L);
		p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
		p330.distances_SET(new char[72], 0);
		p330.increment_SET((char) 221);
		p330.min_distance_SET((char) 24422);
		p330.max_distance_SET((char) 12430);
		CommunicationChannel.instance.send(p330); //===============================
	}
}
