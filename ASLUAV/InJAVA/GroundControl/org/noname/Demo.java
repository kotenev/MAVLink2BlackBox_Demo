package org.noname;
import org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds;

public class Demo extends  GroundControl
{
    public static void main(String[] args)
    {
        final Bounds.Inside PH = new Bounds.Inside();
        CommunicationChannel.instance.on_HEARTBEAT.add((src, ph, pack) ->
        {
            @MAV_TYPE int  type = pack.type_GET();
            @MAV_AUTOPILOT int  autopilot = pack.autopilot_GET();
            @MAV_MODE_FLAG int  base_mode = pack.base_mode_GET();
            long  custom_mode = pack.custom_mode_GET();
            @MAV_STATE int  system_status = pack.system_status_GET();
            char  mavlink_version = pack.mavlink_version_GET();
        });
        CommunicationChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            @MAV_SYS_STATUS_SENSOR int  onboard_control_sensors_present = pack.onboard_control_sensors_present_GET();
            @MAV_SYS_STATUS_SENSOR int  onboard_control_sensors_enabled = pack.onboard_control_sensors_enabled_GET();
            @MAV_SYS_STATUS_SENSOR int  onboard_control_sensors_health = pack.onboard_control_sensors_health_GET();
            char  load = pack.load_GET();
            char  voltage_battery = pack.voltage_battery_GET();
            short  current_battery = pack.current_battery_GET();
            byte  battery_remaining = pack.battery_remaining_GET();
            char  drop_rate_comm = pack.drop_rate_comm_GET();
            char  errors_comm = pack.errors_comm_GET();
            char  errors_count1 = pack.errors_count1_GET();
            char  errors_count2 = pack.errors_count2_GET();
            char  errors_count3 = pack.errors_count3_GET();
            char  errors_count4 = pack.errors_count4_GET();
        });
        CommunicationChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            long  time_unix_usec = pack.time_unix_usec_GET();
            long  time_boot_ms = pack.time_boot_ms_GET();
        });
        CommunicationChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            @MAV_FRAME int  coordinate_frame = pack.coordinate_frame_GET();
            char  type_mask = pack.type_mask_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float  vx = pack.vx_GET();
            float  vy = pack.vy_GET();
            float  vz = pack.vz_GET();
            float  afx = pack.afx_GET();
            float  afy = pack.afy_GET();
            float  afz = pack.afz_GET();
            float  yaw = pack.yaw_GET();
            float  yaw_rate = pack.yaw_rate_GET();
        });
        CommunicationChannel.instance.on_PING.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            long  seq = pack.seq_GET();
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
        });
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  control_request = pack.control_request_GET();
            char  version = pack.version_GET();
            String passkey = pack.passkey_TRY(ph);
        });
        CommunicationChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            char  gcs_system_id = pack.gcs_system_id_GET();
            char  control_request = pack.control_request_GET();
            char  ack = pack.ack_GET();
        });
        CommunicationChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            String key = pack.key_TRY(ph);
        });
        CommunicationChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            @MAV_MODE int  base_mode = pack.base_mode_GET();
            long  custom_mode = pack.custom_mode_GET();
        });
        CommunicationChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            String param_id = pack.param_id_TRY(ph);
            short  param_index = pack.param_index_GET();
        });
        CommunicationChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
        });
        CommunicationChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            String param_id = pack.param_id_TRY(ph);
            float  param_value = pack.param_value_GET();
            @MAV_PARAM_TYPE int  param_type = pack.param_type_GET();
            char  param_count = pack.param_count_GET();
            char  param_index = pack.param_index_GET();
        });
        CommunicationChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            String param_id = pack.param_id_TRY(ph);
            float  param_value = pack.param_value_GET();
            @MAV_PARAM_TYPE int  param_type = pack.param_type_GET();
        });
        CommunicationChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            @GPS_FIX_TYPE int  fix_type = pack.fix_type_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            int  alt = pack.alt_GET();
            char  eph = pack.eph_GET();
            char  epv = pack.epv_GET();
            char  vel = pack.vel_GET();
            char  cog = pack.cog_GET();
            char  satellites_visible = pack.satellites_visible_GET();
            int  alt_ellipsoid = pack.alt_ellipsoid_TRY(ph);
            long  h_acc = pack.h_acc_TRY(ph);
            long  v_acc = pack.v_acc_TRY(ph);
            long  vel_acc = pack.vel_acc_TRY(ph);
            long  hdg_acc = pack.hdg_acc_TRY(ph);
        });
        CommunicationChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            char  satellites_visible = pack.satellites_visible_GET();
            char[]  satellite_prn = pack.satellite_prn_GET();
            char[]  satellite_used = pack.satellite_used_GET();
            char[]  satellite_elevation = pack.satellite_elevation_GET();
            char[]  satellite_azimuth = pack.satellite_azimuth_GET();
            char[]  satellite_snr = pack.satellite_snr_GET();
        });
        CommunicationChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            short  xacc = pack.xacc_GET();
            short  yacc = pack.yacc_GET();
            short  zacc = pack.zacc_GET();
            short  xgyro = pack.xgyro_GET();
            short  ygyro = pack.ygyro_GET();
            short  zgyro = pack.zgyro_GET();
            short  xmag = pack.xmag_GET();
            short  ymag = pack.ymag_GET();
            short  zmag = pack.zmag_GET();
        });
        CommunicationChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            short  xacc = pack.xacc_GET();
            short  yacc = pack.yacc_GET();
            short  zacc = pack.zacc_GET();
            short  xgyro = pack.xgyro_GET();
            short  ygyro = pack.ygyro_GET();
            short  zgyro = pack.zgyro_GET();
            short  xmag = pack.xmag_GET();
            short  ymag = pack.ymag_GET();
            short  zmag = pack.zmag_GET();
        });
        CommunicationChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            short  press_abs = pack.press_abs_GET();
            short  press_diff1 = pack.press_diff1_GET();
            short  press_diff2 = pack.press_diff2_GET();
            short  temperature = pack.temperature_GET();
        });
        CommunicationChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  press_abs = pack.press_abs_GET();
            float  press_diff = pack.press_diff_GET();
            short  temperature = pack.temperature_GET();
        });
        CommunicationChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
            float  rollspeed = pack.rollspeed_GET();
            float  pitchspeed = pack.pitchspeed_GET();
            float  yawspeed = pack.yawspeed_GET();
        });
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  q1 = pack.q1_GET();
            float  q2 = pack.q2_GET();
            float  q3 = pack.q3_GET();
            float  q4 = pack.q4_GET();
            float  rollspeed = pack.rollspeed_GET();
            float  pitchspeed = pack.pitchspeed_GET();
            float  yawspeed = pack.yawspeed_GET();
        });
        CommunicationChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float  vx = pack.vx_GET();
            float  vy = pack.vy_GET();
            float  vz = pack.vz_GET();
        });
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            int  alt = pack.alt_GET();
            int  relative_alt = pack.relative_alt_GET();
            short  vx = pack.vx_GET();
            short  vy = pack.vy_GET();
            short  vz = pack.vz_GET();
            char  hdg = pack.hdg_GET();
        });
        CommunicationChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  port = pack.port_GET();
            short  chan1_scaled = pack.chan1_scaled_GET();
            short  chan2_scaled = pack.chan2_scaled_GET();
            short  chan3_scaled = pack.chan3_scaled_GET();
            short  chan4_scaled = pack.chan4_scaled_GET();
            short  chan5_scaled = pack.chan5_scaled_GET();
            short  chan6_scaled = pack.chan6_scaled_GET();
            short  chan7_scaled = pack.chan7_scaled_GET();
            short  chan8_scaled = pack.chan8_scaled_GET();
            char  rssi = pack.rssi_GET();
        });
        CommunicationChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  port = pack.port_GET();
            char  chan1_raw = pack.chan1_raw_GET();
            char  chan2_raw = pack.chan2_raw_GET();
            char  chan3_raw = pack.chan3_raw_GET();
            char  chan4_raw = pack.chan4_raw_GET();
            char  chan5_raw = pack.chan5_raw_GET();
            char  chan6_raw = pack.chan6_raw_GET();
            char  chan7_raw = pack.chan7_raw_GET();
            char  chan8_raw = pack.chan8_raw_GET();
            char  rssi = pack.rssi_GET();
        });
        CommunicationChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  port = pack.port_GET();
            char  servo1_raw = pack.servo1_raw_GET();
            char  servo2_raw = pack.servo2_raw_GET();
            char  servo3_raw = pack.servo3_raw_GET();
            char  servo4_raw = pack.servo4_raw_GET();
            char  servo5_raw = pack.servo5_raw_GET();
            char  servo6_raw = pack.servo6_raw_GET();
            char  servo7_raw = pack.servo7_raw_GET();
            char  servo8_raw = pack.servo8_raw_GET();
            char  servo9_raw = pack.servo9_raw_TRY(ph);
            char  servo10_raw = pack.servo10_raw_TRY(ph);
            char  servo11_raw = pack.servo11_raw_TRY(ph);
            char  servo12_raw = pack.servo12_raw_TRY(ph);
            char  servo13_raw = pack.servo13_raw_TRY(ph);
            char  servo14_raw = pack.servo14_raw_TRY(ph);
            char  servo15_raw = pack.servo15_raw_TRY(ph);
            char  servo16_raw = pack.servo16_raw_TRY(ph);
        });
        CommunicationChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            short  start_index = pack.start_index_GET();
            short  end_index = pack.end_index_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        CommunicationChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            short  start_index = pack.start_index_GET();
            short  end_index = pack.end_index_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        CommunicationChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  seq = pack.seq_GET();
            @MAV_FRAME int  frame = pack.frame_GET();
            @MAV_CMD int  command = pack.command_GET();
            char  current = pack.current_GET();
            char  autocontinue = pack.autocontinue_GET();
            float  param1 = pack.param1_GET();
            float  param2 = pack.param2_GET();
            float  param3 = pack.param3_GET();
            float  param4 = pack.param4_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        CommunicationChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  seq = pack.seq_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        CommunicationChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  seq = pack.seq_GET();
        });
        CommunicationChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            char  seq = pack.seq_GET();
        });
        CommunicationChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        CommunicationChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  count = pack.count_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        CommunicationChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        CommunicationChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            char  seq = pack.seq_GET();
        });
        CommunicationChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @MAV_MISSION_RESULT int  type = pack.type_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        CommunicationChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            int  latitude = pack.latitude_GET();
            int  longitude = pack.longitude_GET();
            int  altitude = pack.altitude_GET();
            long  time_usec = pack.time_usec_TRY(ph);
        });
        CommunicationChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            int  latitude = pack.latitude_GET();
            int  longitude = pack.longitude_GET();
            int  altitude = pack.altitude_GET();
            long  time_usec = pack.time_usec_TRY(ph);
        });
        CommunicationChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            String param_id = pack.param_id_TRY(ph);
            short  param_index = pack.param_index_GET();
            char  parameter_rc_channel_index = pack.parameter_rc_channel_index_GET();
            float  param_value0 = pack.param_value0_GET();
            float  scale = pack.scale_GET();
            float  param_value_min = pack.param_value_min_GET();
            float  param_value_max = pack.param_value_max_GET();
        });
        CommunicationChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  seq = pack.seq_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        CommunicationChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @MAV_FRAME int  frame = pack.frame_GET();
            float  p1x = pack.p1x_GET();
            float  p1y = pack.p1y_GET();
            float  p1z = pack.p1z_GET();
            float  p2x = pack.p2x_GET();
            float  p2y = pack.p2y_GET();
            float  p2z = pack.p2z_GET();
        });
        CommunicationChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            @MAV_FRAME int  frame = pack.frame_GET();
            float  p1x = pack.p1x_GET();
            float  p1y = pack.p1y_GET();
            float  p1z = pack.p1z_GET();
            float  p2x = pack.p2x_GET();
            float  p2y = pack.p2y_GET();
            float  p2z = pack.p2z_GET();
        });
        CommunicationChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float[]  q = pack.q_GET();
            float  rollspeed = pack.rollspeed_GET();
            float  pitchspeed = pack.pitchspeed_GET();
            float  yawspeed = pack.yawspeed_GET();
            float[]  covariance = pack.covariance_GET();
        });
        CommunicationChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            float  nav_roll = pack.nav_roll_GET();
            float  nav_pitch = pack.nav_pitch_GET();
            short  nav_bearing = pack.nav_bearing_GET();
            short  target_bearing = pack.target_bearing_GET();
            char  wp_dist = pack.wp_dist_GET();
            float  alt_error = pack.alt_error_GET();
            float  aspd_error = pack.aspd_error_GET();
            float  xtrack_error = pack.xtrack_error_GET();
        });
        CommunicationChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            @MAV_ESTIMATOR_TYPE int  estimator_type = pack.estimator_type_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            int  alt = pack.alt_GET();
            int  relative_alt = pack.relative_alt_GET();
            float  vx = pack.vx_GET();
            float  vy = pack.vy_GET();
            float  vz = pack.vz_GET();
            float[]  covariance = pack.covariance_GET();
        });
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            @MAV_ESTIMATOR_TYPE int  estimator_type = pack.estimator_type_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float  vx = pack.vx_GET();
            float  vy = pack.vy_GET();
            float  vz = pack.vz_GET();
            float  ax = pack.ax_GET();
            float  ay = pack.ay_GET();
            float  az = pack.az_GET();
            float[]  covariance = pack.covariance_GET();
        });
        CommunicationChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  chancount = pack.chancount_GET();
            char  chan1_raw = pack.chan1_raw_GET();
            char  chan2_raw = pack.chan2_raw_GET();
            char  chan3_raw = pack.chan3_raw_GET();
            char  chan4_raw = pack.chan4_raw_GET();
            char  chan5_raw = pack.chan5_raw_GET();
            char  chan6_raw = pack.chan6_raw_GET();
            char  chan7_raw = pack.chan7_raw_GET();
            char  chan8_raw = pack.chan8_raw_GET();
            char  chan9_raw = pack.chan9_raw_GET();
            char  chan10_raw = pack.chan10_raw_GET();
            char  chan11_raw = pack.chan11_raw_GET();
            char  chan12_raw = pack.chan12_raw_GET();
            char  chan13_raw = pack.chan13_raw_GET();
            char  chan14_raw = pack.chan14_raw_GET();
            char  chan15_raw = pack.chan15_raw_GET();
            char  chan16_raw = pack.chan16_raw_GET();
            char  chan17_raw = pack.chan17_raw_GET();
            char  chan18_raw = pack.chan18_raw_GET();
            char  rssi = pack.rssi_GET();
        });
        CommunicationChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  req_stream_id = pack.req_stream_id_GET();
            char  req_message_rate = pack.req_message_rate_GET();
            char  start_stop = pack.start_stop_GET();
        });
        CommunicationChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            char  stream_id = pack.stream_id_GET();
            char  message_rate = pack.message_rate_GET();
            char  on_off = pack.on_off_GET();
        });
        CommunicationChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            char  target = pack.target_GET();
            short  x = pack.x_GET();
            short  y = pack.y_GET();
            short  z = pack.z_GET();
            short  r = pack.r_GET();
            char  buttons = pack.buttons_GET();
        });
        CommunicationChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  chan1_raw = pack.chan1_raw_GET();
            char  chan2_raw = pack.chan2_raw_GET();
            char  chan3_raw = pack.chan3_raw_GET();
            char  chan4_raw = pack.chan4_raw_GET();
            char  chan5_raw = pack.chan5_raw_GET();
            char  chan6_raw = pack.chan6_raw_GET();
            char  chan7_raw = pack.chan7_raw_GET();
            char  chan8_raw = pack.chan8_raw_GET();
        });
        CommunicationChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  seq = pack.seq_GET();
            @MAV_FRAME int  frame = pack.frame_GET();
            @MAV_CMD int  command = pack.command_GET();
            char  current = pack.current_GET();
            char  autocontinue = pack.autocontinue_GET();
            float  param1 = pack.param1_GET();
            float  param2 = pack.param2_GET();
            float  param3 = pack.param3_GET();
            float  param4 = pack.param4_GET();
            int  x = pack.x_GET();
            int  y = pack.y_GET();
            float  z = pack.z_GET();
            @MAV_MISSION_TYPE int  mission_type = pack.mission_type_GET();
        });
        CommunicationChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            float  airspeed = pack.airspeed_GET();
            float  groundspeed = pack.groundspeed_GET();
            short  heading = pack.heading_GET();
            char  throttle = pack.throttle_GET();
            float  alt = pack.alt_GET();
            float  climb = pack.climb_GET();
        });
        CommunicationChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @MAV_FRAME int  frame = pack.frame_GET();
            @MAV_CMD int  command = pack.command_GET();
            char  current = pack.current_GET();
            char  autocontinue = pack.autocontinue_GET();
            float  param1 = pack.param1_GET();
            float  param2 = pack.param2_GET();
            float  param3 = pack.param3_GET();
            float  param4 = pack.param4_GET();
            int  x = pack.x_GET();
            int  y = pack.y_GET();
            float  z = pack.z_GET();
        });
        CommunicationChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @MAV_CMD int  command = pack.command_GET();
            char  confirmation = pack.confirmation_GET();
            float  param1 = pack.param1_GET();
            float  param2 = pack.param2_GET();
            float  param3 = pack.param3_GET();
            float  param4 = pack.param4_GET();
            float  param5 = pack.param5_GET();
            float  param6 = pack.param6_GET();
            float  param7 = pack.param7_GET();
        });
        CommunicationChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            @MAV_CMD int  command = pack.command_GET();
            @MAV_RESULT int  result = pack.result_GET();
            char  progress = pack.progress_TRY(ph);
            int  result_param2 = pack.result_param2_TRY(ph);
            char  target_system = pack.target_system_TRY(ph);
            char  target_component = pack.target_component_TRY(ph);
        });
        CommunicationChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
            float  thrust = pack.thrust_GET();
            char  mode_switch = pack.mode_switch_GET();
            char  manual_override_switch = pack.manual_override_switch_GET();
        });
        CommunicationChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  type_mask = pack.type_mask_GET();
            float[]  q = pack.q_GET();
            float  body_roll_rate = pack.body_roll_rate_GET();
            float  body_pitch_rate = pack.body_pitch_rate_GET();
            float  body_yaw_rate = pack.body_yaw_rate_GET();
            float  thrust = pack.thrust_GET();
        });
        CommunicationChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  type_mask = pack.type_mask_GET();
            float[]  q = pack.q_GET();
            float  body_roll_rate = pack.body_roll_rate_GET();
            float  body_pitch_rate = pack.body_pitch_rate_GET();
            float  body_yaw_rate = pack.body_yaw_rate_GET();
            float  thrust = pack.thrust_GET();
        });
        CommunicationChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @MAV_FRAME int  coordinate_frame = pack.coordinate_frame_GET();
            char  type_mask = pack.type_mask_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float  vx = pack.vx_GET();
            float  vy = pack.vy_GET();
            float  vz = pack.vz_GET();
            float  afx = pack.afx_GET();
            float  afy = pack.afy_GET();
            float  afz = pack.afz_GET();
            float  yaw = pack.yaw_GET();
            float  yaw_rate = pack.yaw_rate_GET();
        });
        CommunicationChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            @MAV_FRAME int  coordinate_frame = pack.coordinate_frame_GET();
            char  type_mask = pack.type_mask_GET();
            int  lat_int = pack.lat_int_GET();
            int  lon_int = pack.lon_int_GET();
            float  alt = pack.alt_GET();
            float  vx = pack.vx_GET();
            float  vy = pack.vy_GET();
            float  vz = pack.vz_GET();
            float  afx = pack.afx_GET();
            float  afy = pack.afy_GET();
            float  afz = pack.afz_GET();
            float  yaw = pack.yaw_GET();
            float  yaw_rate = pack.yaw_rate_GET();
        });
        CommunicationChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            @MAV_FRAME int  coordinate_frame = pack.coordinate_frame_GET();
            char  type_mask = pack.type_mask_GET();
            int  lat_int = pack.lat_int_GET();
            int  lon_int = pack.lon_int_GET();
            float  alt = pack.alt_GET();
            float  vx = pack.vx_GET();
            float  vy = pack.vy_GET();
            float  vz = pack.vz_GET();
            float  afx = pack.afx_GET();
            float  afy = pack.afy_GET();
            float  afz = pack.afz_GET();
            float  yaw = pack.yaw_GET();
            float  yaw_rate = pack.yaw_rate_GET();
        });
        CommunicationChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
        });
        CommunicationChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
            float  rollspeed = pack.rollspeed_GET();
            float  pitchspeed = pack.pitchspeed_GET();
            float  yawspeed = pack.yawspeed_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            int  alt = pack.alt_GET();
            short  vx = pack.vx_GET();
            short  vy = pack.vy_GET();
            short  vz = pack.vz_GET();
            short  xacc = pack.xacc_GET();
            short  yacc = pack.yacc_GET();
            short  zacc = pack.zacc_GET();
        });
        CommunicationChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float  roll_ailerons = pack.roll_ailerons_GET();
            float  pitch_elevator = pack.pitch_elevator_GET();
            float  yaw_rudder = pack.yaw_rudder_GET();
            float  throttle = pack.throttle_GET();
            float  aux1 = pack.aux1_GET();
            float  aux2 = pack.aux2_GET();
            float  aux3 = pack.aux3_GET();
            float  aux4 = pack.aux4_GET();
            @MAV_MODE int  mode = pack.mode_GET();
            char  nav_mode = pack.nav_mode_GET();
        });
        CommunicationChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  chan1_raw = pack.chan1_raw_GET();
            char  chan2_raw = pack.chan2_raw_GET();
            char  chan3_raw = pack.chan3_raw_GET();
            char  chan4_raw = pack.chan4_raw_GET();
            char  chan5_raw = pack.chan5_raw_GET();
            char  chan6_raw = pack.chan6_raw_GET();
            char  chan7_raw = pack.chan7_raw_GET();
            char  chan8_raw = pack.chan8_raw_GET();
            char  chan9_raw = pack.chan9_raw_GET();
            char  chan10_raw = pack.chan10_raw_GET();
            char  chan11_raw = pack.chan11_raw_GET();
            char  chan12_raw = pack.chan12_raw_GET();
            char  rssi = pack.rssi_GET();
        });
        CommunicationChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float[]  controls = pack.controls_GET();
            @MAV_MODE int  mode = pack.mode_GET();
            long  flags = pack.flags_GET();
        });
        CommunicationChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  sensor_id = pack.sensor_id_GET();
            short  flow_x = pack.flow_x_GET();
            short  flow_y = pack.flow_y_GET();
            float  flow_comp_m_x = pack.flow_comp_m_x_GET();
            float  flow_comp_m_y = pack.flow_comp_m_y_GET();
            char  quality = pack.quality_GET();
            float  ground_distance = pack.ground_distance_GET();
            float  flow_rate_x = pack.flow_rate_x_TRY(ph);
            float  flow_rate_y = pack.flow_rate_y_TRY(ph);
        });
        CommunicationChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            long  usec = pack.usec_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
        });
        CommunicationChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            long  usec = pack.usec_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
        });
        CommunicationChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            long  usec = pack.usec_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
        });
        CommunicationChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            long  usec = pack.usec_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
        });
        CommunicationChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float  xacc = pack.xacc_GET();
            float  yacc = pack.yacc_GET();
            float  zacc = pack.zacc_GET();
            float  xgyro = pack.xgyro_GET();
            float  ygyro = pack.ygyro_GET();
            float  zgyro = pack.zgyro_GET();
            float  xmag = pack.xmag_GET();
            float  ymag = pack.ymag_GET();
            float  zmag = pack.zmag_GET();
            float  abs_pressure = pack.abs_pressure_GET();
            float  diff_pressure = pack.diff_pressure_GET();
            float  pressure_alt = pack.pressure_alt_GET();
            float  temperature = pack.temperature_GET();
            char  fields_updated = pack.fields_updated_GET();
        });
        CommunicationChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  sensor_id = pack.sensor_id_GET();
            long  integration_time_us = pack.integration_time_us_GET();
            float  integrated_x = pack.integrated_x_GET();
            float  integrated_y = pack.integrated_y_GET();
            float  integrated_xgyro = pack.integrated_xgyro_GET();
            float  integrated_ygyro = pack.integrated_ygyro_GET();
            float  integrated_zgyro = pack.integrated_zgyro_GET();
            short  temperature = pack.temperature_GET();
            char  quality = pack.quality_GET();
            long  time_delta_distance_us = pack.time_delta_distance_us_GET();
            float  distance = pack.distance_GET();
        });
        CommunicationChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float  xacc = pack.xacc_GET();
            float  yacc = pack.yacc_GET();
            float  zacc = pack.zacc_GET();
            float  xgyro = pack.xgyro_GET();
            float  ygyro = pack.ygyro_GET();
            float  zgyro = pack.zgyro_GET();
            float  xmag = pack.xmag_GET();
            float  ymag = pack.ymag_GET();
            float  zmag = pack.zmag_GET();
            float  abs_pressure = pack.abs_pressure_GET();
            float  diff_pressure = pack.diff_pressure_GET();
            float  pressure_alt = pack.pressure_alt_GET();
            float  temperature = pack.temperature_GET();
            long  fields_updated = pack.fields_updated_GET();
        });
        CommunicationChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            float  q1 = pack.q1_GET();
            float  q2 = pack.q2_GET();
            float  q3 = pack.q3_GET();
            float  q4 = pack.q4_GET();
            float  roll = pack.roll_GET();
            float  pitch = pack.pitch_GET();
            float  yaw = pack.yaw_GET();
            float  xacc = pack.xacc_GET();
            float  yacc = pack.yacc_GET();
            float  zacc = pack.zacc_GET();
            float  xgyro = pack.xgyro_GET();
            float  ygyro = pack.ygyro_GET();
            float  zgyro = pack.zgyro_GET();
            float  lat = pack.lat_GET();
            float  lon = pack.lon_GET();
            float  alt = pack.alt_GET();
            float  std_dev_horz = pack.std_dev_horz_GET();
            float  std_dev_vert = pack.std_dev_vert_GET();
            float  vn = pack.vn_GET();
            float  ve = pack.ve_GET();
            float  vd = pack.vd_GET();
        });
        CommunicationChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            char  rssi = pack.rssi_GET();
            char  remrssi = pack.remrssi_GET();
            char  txbuf = pack.txbuf_GET();
            char  noise = pack.noise_GET();
            char  remnoise = pack.remnoise_GET();
            char  rxerrors = pack.rxerrors_GET();
            char  fixed_ = pack.fixed__GET();
        });
        CommunicationChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            char  target_network = pack.target_network_GET();
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char[]  payload = pack.payload_GET();
        });
        CommunicationChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            long  tc1 = pack.tc1_GET();
            long  ts1 = pack.ts1_GET();
        });
        CommunicationChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            long  seq = pack.seq_GET();
        });
        CommunicationChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  fix_type = pack.fix_type_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            int  alt = pack.alt_GET();
            char  eph = pack.eph_GET();
            char  epv = pack.epv_GET();
            char  vel = pack.vel_GET();
            short  vn = pack.vn_GET();
            short  ve = pack.ve_GET();
            short  vd = pack.vd_GET();
            char  cog = pack.cog_GET();
            char  satellites_visible = pack.satellites_visible_GET();
        });
        CommunicationChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  sensor_id = pack.sensor_id_GET();
            long  integration_time_us = pack.integration_time_us_GET();
            float  integrated_x = pack.integrated_x_GET();
            float  integrated_y = pack.integrated_y_GET();
            float  integrated_xgyro = pack.integrated_xgyro_GET();
            float  integrated_ygyro = pack.integrated_ygyro_GET();
            float  integrated_zgyro = pack.integrated_zgyro_GET();
            short  temperature = pack.temperature_GET();
            char  quality = pack.quality_GET();
            long  time_delta_distance_us = pack.time_delta_distance_us_GET();
            float  distance = pack.distance_GET();
        });
        CommunicationChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float[]  attitude_quaternion = pack.attitude_quaternion_GET();
            float  rollspeed = pack.rollspeed_GET();
            float  pitchspeed = pack.pitchspeed_GET();
            float  yawspeed = pack.yawspeed_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            int  alt = pack.alt_GET();
            short  vx = pack.vx_GET();
            short  vy = pack.vy_GET();
            short  vz = pack.vz_GET();
            char  ind_airspeed = pack.ind_airspeed_GET();
            char  true_airspeed = pack.true_airspeed_GET();
            short  xacc = pack.xacc_GET();
            short  yacc = pack.yacc_GET();
            short  zacc = pack.zacc_GET();
        });
        CommunicationChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            short  xacc = pack.xacc_GET();
            short  yacc = pack.yacc_GET();
            short  zacc = pack.zacc_GET();
            short  xgyro = pack.xgyro_GET();
            short  ygyro = pack.ygyro_GET();
            short  zgyro = pack.zgyro_GET();
            short  xmag = pack.xmag_GET();
            short  ymag = pack.ymag_GET();
            short  zmag = pack.zmag_GET();
        });
        CommunicationChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  start = pack.start_GET();
            char  end = pack.end_GET();
        });
        CommunicationChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            char  id = pack.id_GET();
            char  num_logs = pack.num_logs_GET();
            char  last_log_num = pack.last_log_num_GET();
            long  time_utc = pack.time_utc_GET();
            long  size = pack.size_GET();
        });
        CommunicationChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  id = pack.id_GET();
            long  ofs = pack.ofs_GET();
            long  count = pack.count_GET();
        });
        CommunicationChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            char  id = pack.id_GET();
            long  ofs = pack.ofs_GET();
            char  count = pack.count_GET();
            char[]  data_ = pack.data__GET();
        });
        CommunicationChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
        });
        CommunicationChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
        });
        CommunicationChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  len = pack.len_GET();
            char[]  data_ = pack.data__GET();
        });
        CommunicationChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            @GPS_FIX_TYPE int  fix_type = pack.fix_type_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            int  alt = pack.alt_GET();
            char  eph = pack.eph_GET();
            char  epv = pack.epv_GET();
            char  vel = pack.vel_GET();
            char  cog = pack.cog_GET();
            char  satellites_visible = pack.satellites_visible_GET();
            char  dgps_numch = pack.dgps_numch_GET();
            long  dgps_age = pack.dgps_age_GET();
        });
        CommunicationChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            char  Vcc = pack.Vcc_GET();
            char  Vservo = pack.Vservo_GET();
            @MAV_POWER_STATUS int  flags = pack.flags_GET();
        });
        CommunicationChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            @SERIAL_CONTROL_DEV int  device = pack.device_GET();
            @SERIAL_CONTROL_FLAG int  flags = pack.flags_GET();
            char  timeout = pack.timeout_GET();
            long  baudrate = pack.baudrate_GET();
            char  count = pack.count_GET();
            char[]  data_ = pack.data__GET();
        });
        CommunicationChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            long  time_last_baseline_ms = pack.time_last_baseline_ms_GET();
            char  rtk_receiver_id = pack.rtk_receiver_id_GET();
            char  wn = pack.wn_GET();
            long  tow = pack.tow_GET();
            char  rtk_health = pack.rtk_health_GET();
            char  rtk_rate = pack.rtk_rate_GET();
            char  nsats = pack.nsats_GET();
            char  baseline_coords_type = pack.baseline_coords_type_GET();
            int  baseline_a_mm = pack.baseline_a_mm_GET();
            int  baseline_b_mm = pack.baseline_b_mm_GET();
            int  baseline_c_mm = pack.baseline_c_mm_GET();
            long  accuracy = pack.accuracy_GET();
            int  iar_num_hypotheses = pack.iar_num_hypotheses_GET();
        });
        CommunicationChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            long  time_last_baseline_ms = pack.time_last_baseline_ms_GET();
            char  rtk_receiver_id = pack.rtk_receiver_id_GET();
            char  wn = pack.wn_GET();
            long  tow = pack.tow_GET();
            char  rtk_health = pack.rtk_health_GET();
            char  rtk_rate = pack.rtk_rate_GET();
            char  nsats = pack.nsats_GET();
            char  baseline_coords_type = pack.baseline_coords_type_GET();
            int  baseline_a_mm = pack.baseline_a_mm_GET();
            int  baseline_b_mm = pack.baseline_b_mm_GET();
            int  baseline_c_mm = pack.baseline_c_mm_GET();
            long  accuracy = pack.accuracy_GET();
            int  iar_num_hypotheses = pack.iar_num_hypotheses_GET();
        });
        CommunicationChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            short  xacc = pack.xacc_GET();
            short  yacc = pack.yacc_GET();
            short  zacc = pack.zacc_GET();
            short  xgyro = pack.xgyro_GET();
            short  ygyro = pack.ygyro_GET();
            short  zgyro = pack.zgyro_GET();
            short  xmag = pack.xmag_GET();
            short  ymag = pack.ymag_GET();
            short  zmag = pack.zmag_GET();
        });
        CommunicationChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            char  type = pack.type_GET();
            long  size = pack.size_GET();
            char  width = pack.width_GET();
            char  height = pack.height_GET();
            char  packets = pack.packets_GET();
            char  payload = pack.payload_GET();
            char  jpg_quality = pack.jpg_quality_GET();
        });
        CommunicationChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            char  seqnr = pack.seqnr_GET();
            char[]  data_ = pack.data__GET();
        });
        CommunicationChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  min_distance = pack.min_distance_GET();
            char  max_distance = pack.max_distance_GET();
            char  current_distance = pack.current_distance_GET();
            @MAV_DISTANCE_SENSOR int  type = pack.type_GET();
            char  id = pack.id_GET();
            @MAV_SENSOR_ORIENTATION int  orientation = pack.orientation_GET();
            char  covariance = pack.covariance_GET();
        });
        CommunicationChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            char  grid_spacing = pack.grid_spacing_GET();
            long  mask = pack.mask_GET();
        });
        CommunicationChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            char  grid_spacing = pack.grid_spacing_GET();
            char  gridbit = pack.gridbit_GET();
            short[]  data_ = pack.data__GET();
        });
        CommunicationChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
        });
        CommunicationChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            char  spacing = pack.spacing_GET();
            float  terrain_height = pack.terrain_height_GET();
            float  current_height = pack.current_height_GET();
            char  pending = pack.pending_GET();
            char  loaded = pack.loaded_GET();
        });
        CommunicationChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  press_abs = pack.press_abs_GET();
            float  press_diff = pack.press_diff_GET();
            short  temperature = pack.temperature_GET();
        });
        CommunicationChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float[]  q = pack.q_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
        });
        CommunicationChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  group_mlx = pack.group_mlx_GET();
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            float[]  controls = pack.controls_GET();
        });
        CommunicationChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  group_mlx = pack.group_mlx_GET();
            float[]  controls = pack.controls_GET();
        });
        CommunicationChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float  altitude_monotonic = pack.altitude_monotonic_GET();
            float  altitude_amsl = pack.altitude_amsl_GET();
            float  altitude_local = pack.altitude_local_GET();
            float  altitude_relative = pack.altitude_relative_GET();
            float  altitude_terrain = pack.altitude_terrain_GET();
            float  bottom_clearance = pack.bottom_clearance_GET();
        });
        CommunicationChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            char  request_id = pack.request_id_GET();
            char  uri_type = pack.uri_type_GET();
            char[]  uri = pack.uri_GET();
            char  transfer_type = pack.transfer_type_GET();
            char[]  storage = pack.storage_GET();
        });
        CommunicationChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            float  press_abs = pack.press_abs_GET();
            float  press_diff = pack.press_diff_GET();
            short  temperature = pack.temperature_GET();
        });
        CommunicationChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            long  timestamp = pack.timestamp_GET();
            char  est_capabilities = pack.est_capabilities_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            float  alt = pack.alt_GET();
            float[]  vel = pack.vel_GET();
            float[]  acc = pack.acc_GET();
            float[]  attitude_q = pack.attitude_q_GET();
            float[]  rates = pack.rates_GET();
            float[]  position_cov = pack.position_cov_GET();
            long  custom_state = pack.custom_state_GET();
        });
        CommunicationChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float  x_acc = pack.x_acc_GET();
            float  y_acc = pack.y_acc_GET();
            float  z_acc = pack.z_acc_GET();
            float  x_vel = pack.x_vel_GET();
            float  y_vel = pack.y_vel_GET();
            float  z_vel = pack.z_vel_GET();
            float  x_pos = pack.x_pos_GET();
            float  y_pos = pack.y_pos_GET();
            float  z_pos = pack.z_pos_GET();
            float  airspeed = pack.airspeed_GET();
            float[]  vel_variance = pack.vel_variance_GET();
            float[]  pos_variance = pack.pos_variance_GET();
            float[]  q = pack.q_GET();
            float  roll_rate = pack.roll_rate_GET();
            float  pitch_rate = pack.pitch_rate_GET();
            float  yaw_rate = pack.yaw_rate_GET();
        });
        CommunicationChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            char  id = pack.id_GET();
            @MAV_BATTERY_FUNCTION int  battery_function = pack.battery_function_GET();
            @MAV_BATTERY_TYPE int  type = pack.type_GET();
            short  temperature = pack.temperature_GET();
            char[]  voltages = pack.voltages_GET();
            short  current_battery = pack.current_battery_GET();
            int  current_consumed = pack.current_consumed_GET();
            int  energy_consumed = pack.energy_consumed_GET();
            byte  battery_remaining = pack.battery_remaining_GET();
        });
        POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.instance.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.time_boot_ms_SET(3035337118L);
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
        p3.type_mask_SET((char)28467);
        p3.x_SET(3.781089E37F);
        p3.y_SET(-6.284593E37F);
        p3.z_SET(-3.051044E38F);
        p3.vx_SET(-1.8907928E37F);
        p3.vy_SET(6.1795766E37F);
        p3.vz_SET(5.540286E37F);
        p3.afx_SET(2.7750632E38F);
        p3.afy_SET(2.4212132E37F);
        p3.afz_SET(7.79828E37F);
        p3.yaw_SET(5.7809193E37F);
        p3.yaw_rate_SET(-7.3061113E37F);
        CommunicationChannel.instance.send(p3); //===============================
        SET_ATTITUDE_TARGET p82 = CommunicationChannel.instance.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.time_boot_ms_SET(3303545253L);
        p82.target_system_SET((char)136);
        p82.target_component_SET((char)123);
        p82.type_mask_SET((char)102);
        p82.q_SET(new float[4], 0);
        p82.body_roll_rate_SET(-2.596874E38F);
        p82.body_pitch_rate_SET(-2.8531578E38F);
        p82.body_yaw_rate_SET(1.463031E38F);
        p82.thrust_SET(1.6114843E38F);
        CommunicationChannel.instance.send(p82); //===============================
        ATTITUDE_TARGET p83 = CommunicationChannel.instance.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.time_boot_ms_SET(1472834228L);
        p83.type_mask_SET((char)130);
        p83.q_SET(new float[4], 0);
        p83.body_roll_rate_SET(2.651529E37F);
        p83.body_pitch_rate_SET(1.0544804E38F);
        p83.body_yaw_rate_SET(8.471773E37F);
        p83.thrust_SET(-9.290501E37F);
        CommunicationChannel.instance.send(p83); //===============================
        SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.instance.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.time_boot_ms_SET(3692673709L);
        p84.target_system_SET((char)88);
        p84.target_component_SET((char)30);
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL);
        p84.type_mask_SET((char)11576);
        p84.x_SET(-3.156419E38F);
        p84.y_SET(-2.4116318E38F);
        p84.z_SET(-2.4640538E38F);
        p84.vx_SET(-3.3185837E38F);
        p84.vy_SET(3.4715552E37F);
        p84.vz_SET(2.4925252E38F);
        p84.afx_SET(-2.7446942E38F);
        p84.afy_SET(1.274306E37F);
        p84.afz_SET(-1.9725874E38F);
        p84.yaw_SET(1.910709E38F);
        p84.yaw_rate_SET(-2.688917E38F);
        CommunicationChannel.instance.send(p84); //===============================
        SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.instance.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.time_boot_ms_SET(55277819L);
        p86.target_system_SET((char)139);
        p86.target_component_SET((char)228);
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED);
        p86.type_mask_SET((char)1421);
        p86.lat_int_SET(-19670728);
        p86.lon_int_SET(-619987836);
        p86.alt_SET(3.1355995E38F);
        p86.vx_SET(-2.855191E38F);
        p86.vy_SET(-4.4325355E37F);
        p86.vz_SET(2.3886693E38F);
        p86.afx_SET(-1.8051247E38F);
        p86.afy_SET(8.436531E37F);
        p86.afz_SET(3.1895724E38F);
        p86.yaw_SET(1.845547E38F);
        p86.yaw_rate_SET(2.7534085E38F);
        CommunicationChannel.instance.send(p86); //===============================
        POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.instance.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.time_boot_ms_SET(3980096851L);
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED);
        p87.type_mask_SET((char)6400);
        p87.lat_int_SET(216891560);
        p87.lon_int_SET(-36627254);
        p87.alt_SET(-8.0925724E37F);
        p87.vx_SET(-2.062258E38F);
        p87.vy_SET(-6.3321414E37F);
        p87.vz_SET(1.6594647E38F);
        p87.afx_SET(4.5475287E37F);
        p87.afy_SET(-3.2431954E38F);
        p87.afz_SET(-1.0940906E38F);
        p87.yaw_SET(-2.6705666E38F);
        p87.yaw_rate_SET(1.7367132E38F);
        CommunicationChannel.instance.send(p87); //===============================
        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.instance.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.time_boot_ms_SET(2601582600L);
        p89.x_SET(-1.6155844E38F);
        p89.y_SET(-2.6131155E38F);
        p89.z_SET(-1.4395243E38F);
        p89.roll_SET(3.235487E38F);
        p89.pitch_SET(-1.2738797E38F);
        p89.yaw_SET(7.9327445E37F);
        CommunicationChannel.instance.send(p89); //===============================
        HIL_STATE p90 = CommunicationChannel.instance.new_HIL_STATE();
        PH.setPack(p90);
        p90.time_usec_SET(4575911700741833299L);
        p90.roll_SET(3.277932E38F);
        p90.pitch_SET(2.346501E37F);
        p90.yaw_SET(5.9259344E37F);
        p90.rollspeed_SET(3.1289347E38F);
        p90.pitchspeed_SET(3.1470329E38F);
        p90.yawspeed_SET(3.0103769E38F);
        p90.lat_SET(-595952758);
        p90.lon_SET(-2010679520);
        p90.alt_SET(-965696687);
        p90.vx_SET((short) -2278);
        p90.vy_SET((short)18164);
        p90.vz_SET((short) -7152);
        p90.xacc_SET((short) -3901);
        p90.yacc_SET((short) -27529);
        p90.zacc_SET((short)22794);
        CommunicationChannel.instance.send(p90); //===============================
        HIL_CONTROLS p91 = CommunicationChannel.instance.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.time_usec_SET(7715533697708728294L);
        p91.roll_ailerons_SET(2.2661825E37F);
        p91.pitch_elevator_SET(3.2163622E38F);
        p91.yaw_rudder_SET(4.1557357E37F);
        p91.throttle_SET(-6.9139905E37F);
        p91.aux1_SET(-3.250165E38F);
        p91.aux2_SET(-2.4029068E36F);
        p91.aux3_SET(-1.9013015E38F);
        p91.aux4_SET(-1.6940669E38F);
        p91.mode_SET(MAV_MODE.MAV_MODE_AUTO_DISARMED);
        p91.nav_mode_SET((char)135);
        CommunicationChannel.instance.send(p91); //===============================
        HIL_RC_INPUTS_RAW p92 = CommunicationChannel.instance.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.time_usec_SET(181524067995020535L);
        p92.chan1_raw_SET((char)8472);
        p92.chan2_raw_SET((char)3422);
        p92.chan3_raw_SET((char)19472);
        p92.chan4_raw_SET((char)10939);
        p92.chan5_raw_SET((char)3233);
        p92.chan6_raw_SET((char)41811);
        p92.chan7_raw_SET((char)42302);
        p92.chan8_raw_SET((char)21840);
        p92.chan9_raw_SET((char)61883);
        p92.chan10_raw_SET((char)4579);
        p92.chan11_raw_SET((char)60424);
        p92.chan12_raw_SET((char)60012);
        p92.rssi_SET((char)131);
        CommunicationChannel.instance.send(p92); //===============================
        HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.instance.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(8807827492406855928L);
        p93.controls_SET(new float[16], 0);
        p93.mode_SET(MAV_MODE.MAV_MODE_TEST_DISARMED);
        p93.flags_SET(4797736456279735164L);
        CommunicationChannel.instance.send(p93); //===============================
        OPTICAL_FLOW p100 = CommunicationChannel.instance.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.time_usec_SET(8073890771139014600L);
        p100.sensor_id_SET((char)237);
        p100.flow_x_SET((short) -23055);
        p100.flow_y_SET((short)5089);
        p100.flow_comp_m_x_SET(1.6935653E38F);
        p100.flow_comp_m_y_SET(2.2018E38F);
        p100.quality_SET((char)55);
        p100.ground_distance_SET(3.1230116E38F);
        p100.flow_rate_x_SET(7.514929E37F, PH);
        p100.flow_rate_y_SET(8.3574855E37F, PH);
        CommunicationChannel.instance.send(p100); //===============================
        GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.instance.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.usec_SET(6459908288791418576L);
        p101.x_SET(3.3821658E38F);
        p101.y_SET(2.563058E38F);
        p101.z_SET(2.9435279E38F);
        p101.roll_SET(-1.1804856E37F);
        p101.pitch_SET(-3.3357858E38F);
        p101.yaw_SET(-1.5120437E38F);
        CommunicationChannel.instance.send(p101); //===============================
        VISION_POSITION_ESTIMATE p102 = CommunicationChannel.instance.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.usec_SET(6908866066663676892L);
        p102.x_SET(-2.1963708E38F);
        p102.y_SET(2.3513984E38F);
        p102.z_SET(2.347197E38F);
        p102.roll_SET(-1.2975607E38F);
        p102.pitch_SET(5.864403E37F);
        p102.yaw_SET(-2.501069E37F);
        CommunicationChannel.instance.send(p102); //===============================
        VISION_SPEED_ESTIMATE p103 = CommunicationChannel.instance.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(3707715142062181843L);
        p103.x_SET(-8.164072E37F);
        p103.y_SET(-2.5360835E38F);
        p103.z_SET(-1.9559441E38F);
        CommunicationChannel.instance.send(p103); //===============================
        VICON_POSITION_ESTIMATE p104 = CommunicationChannel.instance.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(7070969635968205770L);
        p104.x_SET(2.6705392E37F);
        p104.y_SET(6.8753956E37F);
        p104.z_SET(4.841913E37F);
        p104.roll_SET(2.2489902E38F);
        p104.pitch_SET(-5.0812063E36F);
        p104.yaw_SET(-4.5267863E36F);
        CommunicationChannel.instance.send(p104); //===============================
        HIGHRES_IMU p105 = CommunicationChannel.instance.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.time_usec_SET(4341513612265242126L);
        p105.xacc_SET(-3.2168887E38F);
        p105.yacc_SET(-1.6973959E38F);
        p105.zacc_SET(-1.2011666E38F);
        p105.xgyro_SET(3.012189E38F);
        p105.ygyro_SET(3.819373E37F);
        p105.zgyro_SET(1.3493213E38F);
        p105.xmag_SET(7.475383E37F);
        p105.ymag_SET(-2.1188167E38F);
        p105.zmag_SET(2.423071E38F);
        p105.abs_pressure_SET(1.1938673E37F);
        p105.diff_pressure_SET(-3.6817403E37F);
        p105.pressure_alt_SET(2.2803077E38F);
        p105.temperature_SET(1.8786036E38F);
        p105.fields_updated_SET((char)18341);
        CommunicationChannel.instance.send(p105); //===============================
        OPTICAL_FLOW_RAD p106 = CommunicationChannel.instance.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_usec_SET(3071333899348767203L);
        p106.sensor_id_SET((char)66);
        p106.integration_time_us_SET(3237851542L);
        p106.integrated_x_SET(-8.536062E37F);
        p106.integrated_y_SET(-3.3595412E38F);
        p106.integrated_xgyro_SET(2.6116079E38F);
        p106.integrated_ygyro_SET(-2.0805493E38F);
        p106.integrated_zgyro_SET(-5.113982E36F);
        p106.temperature_SET((short)5681);
        p106.quality_SET((char)143);
        p106.time_delta_distance_us_SET(1777757171L);
        p106.distance_SET(-6.5392943E37F);
        CommunicationChannel.instance.send(p106); //===============================
        HIL_SENSOR p107 = CommunicationChannel.instance.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.time_usec_SET(2865436631460420334L);
        p107.xacc_SET(-3.08098E38F);
        p107.yacc_SET(-2.0262425E38F);
        p107.zacc_SET(-6.879658E37F);
        p107.xgyro_SET(-2.9576173E38F);
        p107.ygyro_SET(-3.5378006E37F);
        p107.zgyro_SET(1.7789164E38F);
        p107.xmag_SET(2.8875497E38F);
        p107.ymag_SET(-2.3535252E38F);
        p107.zmag_SET(2.3572034E38F);
        p107.abs_pressure_SET(-1.7946884E38F);
        p107.diff_pressure_SET(-8.897394E37F);
        p107.pressure_alt_SET(8.728777E37F);
        p107.temperature_SET(1.1926185E38F);
        p107.fields_updated_SET(3149013218L);
        CommunicationChannel.instance.send(p107); //===============================
        SIM_STATE p108 = CommunicationChannel.instance.new_SIM_STATE();
        PH.setPack(p108);
        p108.q1_SET(8.517023E37F);
        p108.q2_SET(6.5751145E37F);
        p108.q3_SET(-2.919102E38F);
        p108.q4_SET(1.3999546E38F);
        p108.roll_SET(-9.763997E37F);
        p108.pitch_SET(9.69926E37F);
        p108.yaw_SET(2.2658212E38F);
        p108.xacc_SET(2.15209E38F);
        p108.yacc_SET(-1.999373E38F);
        p108.zacc_SET(-2.9547307E38F);
        p108.xgyro_SET(4.237865E37F);
        p108.ygyro_SET(3.2578981E38F);
        p108.zgyro_SET(2.3868822E38F);
        p108.lat_SET(-2.6699612E37F);
        p108.lon_SET(-9.025704E37F);
        p108.alt_SET(-1.1445916E38F);
        p108.std_dev_horz_SET(-1.9796865E38F);
        p108.std_dev_vert_SET(5.974473E37F);
        p108.vn_SET(-3.494245E37F);
        p108.ve_SET(-2.5611947E38F);
        p108.vd_SET(2.8963003E38F);
        CommunicationChannel.instance.send(p108); //===============================
        RADIO_STATUS p109 = CommunicationChannel.instance.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)92);
        p109.remrssi_SET((char)247);
        p109.txbuf_SET((char)105);
        p109.noise_SET((char)157);
        p109.remnoise_SET((char)198);
        p109.rxerrors_SET((char)57293);
        p109.fixed__SET((char)6288);
        CommunicationChannel.instance.send(p109); //===============================
        FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.instance.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)239);
        p110.target_system_SET((char)124);
        p110.target_component_SET((char)49);
        p110.payload_SET(new char[251], 0);
        CommunicationChannel.instance.send(p110); //===============================
        TIMESYNC p111 = CommunicationChannel.instance.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(-2209676549662922572L);
        p111.ts1_SET(-4242498426602928048L);
        CommunicationChannel.instance.send(p111); //===============================
        CAMERA_TRIGGER p112 = CommunicationChannel.instance.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(3560726865022800072L);
        p112.seq_SET(2699179533L);
        CommunicationChannel.instance.send(p112); //===============================
        HIL_GPS p113 = CommunicationChannel.instance.new_HIL_GPS();
        PH.setPack(p113);
        p113.time_usec_SET(1992148526392145848L);
        p113.fix_type_SET((char)82);
        p113.lat_SET(1695761592);
        p113.lon_SET(949061394);
        p113.alt_SET(2134431167);
        p113.eph_SET((char)33444);
        p113.epv_SET((char)61645);
        p113.vel_SET((char)59008);
        p113.vn_SET((short) -29127);
        p113.ve_SET((short)4700);
        p113.vd_SET((short)32521);
        p113.cog_SET((char)60500);
        p113.satellites_visible_SET((char)171);
        CommunicationChannel.instance.send(p113); //===============================
        HIL_OPTICAL_FLOW p114 = CommunicationChannel.instance.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.time_usec_SET(1792641692873268818L);
        p114.sensor_id_SET((char)101);
        p114.integration_time_us_SET(2909020348L);
        p114.integrated_x_SET(-1.3311055E38F);
        p114.integrated_y_SET(2.6522962E37F);
        p114.integrated_xgyro_SET(1.3761274E37F);
        p114.integrated_ygyro_SET(-3.4461634E37F);
        p114.integrated_zgyro_SET(1.5047891E38F);
        p114.temperature_SET((short) -13594);
        p114.quality_SET((char)15);
        p114.time_delta_distance_us_SET(2122253140L);
        p114.distance_SET(1.4516553E38F);
        CommunicationChannel.instance.send(p114); //===============================
        HIL_STATE_QUATERNION p115 = CommunicationChannel.instance.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.time_usec_SET(7486230413363527285L);
        p115.attitude_quaternion_SET(new float[4], 0);
        p115.rollspeed_SET(2.640825E38F);
        p115.pitchspeed_SET(-2.0484396E38F);
        p115.yawspeed_SET(2.9489486E38F);
        p115.lat_SET(1595474683);
        p115.lon_SET(1937558226);
        p115.alt_SET(2048810606);
        p115.vx_SET((short) -3983);
        p115.vy_SET((short)3459);
        p115.vz_SET((short)7255);
        p115.ind_airspeed_SET((char)34055);
        p115.true_airspeed_SET((char)13352);
        p115.xacc_SET((short)22095);
        p115.yacc_SET((short) -31286);
        p115.zacc_SET((short) -19942);
        CommunicationChannel.instance.send(p115); //===============================
        SCALED_IMU2 p116 = CommunicationChannel.instance.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.time_boot_ms_SET(2640585840L);
        p116.xacc_SET((short) -21382);
        p116.yacc_SET((short) -7719);
        p116.zacc_SET((short) -30713);
        p116.xgyro_SET((short)15489);
        p116.ygyro_SET((short) -24766);
        p116.zgyro_SET((short) -30708);
        p116.xmag_SET((short)13186);
        p116.ymag_SET((short)7316);
        p116.zmag_SET((short) -30807);
        CommunicationChannel.instance.send(p116); //===============================
        LOG_REQUEST_LIST p117 = CommunicationChannel.instance.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)195);
        p117.target_component_SET((char)244);
        p117.start_SET((char)41289);
        p117.end_SET((char)28717);
        CommunicationChannel.instance.send(p117); //===============================
        LOG_ENTRY p118 = CommunicationChannel.instance.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)22466);
        p118.num_logs_SET((char)41083);
        p118.last_log_num_SET((char)64846);
        p118.time_utc_SET(4203420577L);
        p118.size_SET(740587547L);
        CommunicationChannel.instance.send(p118); //===============================
        LOG_REQUEST_DATA p119 = CommunicationChannel.instance.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)44);
        p119.target_component_SET((char)165);
        p119.id_SET((char)30028);
        p119.ofs_SET(2817747413L);
        p119.count_SET(4095025797L);
        CommunicationChannel.instance.send(p119); //===============================
        LOG_DATA p120 = CommunicationChannel.instance.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)60922);
        p120.ofs_SET(70744980L);
        p120.count_SET((char)20);
        p120.data__SET(new char[90], 0);
        CommunicationChannel.instance.send(p120); //===============================
        LOG_ERASE p121 = CommunicationChannel.instance.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)238);
        p121.target_component_SET((char)205);
        CommunicationChannel.instance.send(p121); //===============================
        LOG_REQUEST_END p122 = CommunicationChannel.instance.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)223);
        p122.target_component_SET((char)106);
        CommunicationChannel.instance.send(p122); //===============================
        GPS_INJECT_DATA p123 = CommunicationChannel.instance.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)123);
        p123.target_component_SET((char)230);
        p123.len_SET((char)184);
        p123.data__SET(new char[110], 0);
        CommunicationChannel.instance.send(p123); //===============================
        GPS2_RAW p124 = CommunicationChannel.instance.new_GPS2_RAW();
        PH.setPack(p124);
        p124.time_usec_SET(6155411100663218424L);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
        p124.lat_SET(2102818197);
        p124.lon_SET(1943581860);
        p124.alt_SET(1456133772);
        p124.eph_SET((char)37796);
        p124.epv_SET((char)63734);
        p124.vel_SET((char)29410);
        p124.cog_SET((char)16041);
        p124.satellites_visible_SET((char)7);
        p124.dgps_numch_SET((char)165);
        p124.dgps_age_SET(546338883L);
        CommunicationChannel.instance.send(p124); //===============================
        POWER_STATUS p125 = CommunicationChannel.instance.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)8837);
        p125.Vservo_SET((char)21359);
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_USB_CONNECTED |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED));
        CommunicationChannel.instance.send(p125); //===============================
        SERIAL_CONTROL p126 = CommunicationChannel.instance.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS2);
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_BLOCKING));
        p126.timeout_SET((char)37144);
        p126.baudrate_SET(673578104L);
        p126.count_SET((char)150);
        p126.data__SET(new char[70], 0);
        CommunicationChannel.instance.send(p126); //===============================
        GPS_RTK p127 = CommunicationChannel.instance.new_GPS_RTK();
        PH.setPack(p127);
        p127.time_last_baseline_ms_SET(67486244L);
        p127.rtk_receiver_id_SET((char)169);
        p127.wn_SET((char)6642);
        p127.tow_SET(2292444824L);
        p127.rtk_health_SET((char)185);
        p127.rtk_rate_SET((char)22);
        p127.nsats_SET((char)170);
        p127.baseline_coords_type_SET((char)91);
        p127.baseline_a_mm_SET(2098532926);
        p127.baseline_b_mm_SET(1343970934);
        p127.baseline_c_mm_SET(-772458781);
        p127.accuracy_SET(387870706L);
        p127.iar_num_hypotheses_SET(889717936);
        CommunicationChannel.instance.send(p127); //===============================
        GPS2_RTK p128 = CommunicationChannel.instance.new_GPS2_RTK();
        PH.setPack(p128);
        p128.time_last_baseline_ms_SET(914768894L);
        p128.rtk_receiver_id_SET((char)11);
        p128.wn_SET((char)3478);
        p128.tow_SET(4063359284L);
        p128.rtk_health_SET((char)127);
        p128.rtk_rate_SET((char)189);
        p128.nsats_SET((char)32);
        p128.baseline_coords_type_SET((char)225);
        p128.baseline_a_mm_SET(209465795);
        p128.baseline_b_mm_SET(218064164);
        p128.baseline_c_mm_SET(-364577035);
        p128.accuracy_SET(1421077496L);
        p128.iar_num_hypotheses_SET(-225737036);
        CommunicationChannel.instance.send(p128); //===============================
        SCALED_IMU3 p129 = CommunicationChannel.instance.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.time_boot_ms_SET(2794651162L);
        p129.xacc_SET((short)26773);
        p129.yacc_SET((short) -5318);
        p129.zacc_SET((short)29328);
        p129.xgyro_SET((short)17665);
        p129.ygyro_SET((short)13146);
        p129.zgyro_SET((short)4443);
        p129.xmag_SET((short)15492);
        p129.ymag_SET((short) -31822);
        p129.zmag_SET((short)27086);
        CommunicationChannel.instance.send(p129); //===============================
        DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.instance.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.type_SET((char)10);
        p130.size_SET(4155729445L);
        p130.width_SET((char)56509);
        p130.height_SET((char)48601);
        p130.packets_SET((char)20717);
        p130.payload_SET((char)147);
        p130.jpg_quality_SET((char)238);
        CommunicationChannel.instance.send(p130); //===============================
        ENCAPSULATED_DATA p131 = CommunicationChannel.instance.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)31748);
        p131.data__SET(new char[253], 0);
        CommunicationChannel.instance.send(p131); //===============================
        DISTANCE_SENSOR p132 = CommunicationChannel.instance.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.time_boot_ms_SET(3937284825L);
        p132.min_distance_SET((char)43234);
        p132.max_distance_SET((char)47265);
        p132.current_distance_SET((char)59354);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
        p132.id_SET((char)126);
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_PITCH_180);
        p132.covariance_SET((char)161);
        CommunicationChannel.instance.send(p132); //===============================
        TERRAIN_REQUEST p133 = CommunicationChannel.instance.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(-1522283191);
        p133.lon_SET(733191380);
        p133.grid_spacing_SET((char)48961);
        p133.mask_SET(6879292338776583537L);
        CommunicationChannel.instance.send(p133); //===============================
        TERRAIN_DATA p134 = CommunicationChannel.instance.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lat_SET(-937381318);
        p134.lon_SET(-752124169);
        p134.grid_spacing_SET((char)8180);
        p134.gridbit_SET((char)134);
        p134.data__SET(new short[16], 0);
        CommunicationChannel.instance.send(p134); //===============================
        TERRAIN_CHECK p135 = CommunicationChannel.instance.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(1562692305);
        p135.lon_SET(694120066);
        CommunicationChannel.instance.send(p135); //===============================
        TERRAIN_REPORT p136 = CommunicationChannel.instance.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lat_SET(950159558);
        p136.lon_SET(1851693803);
        p136.spacing_SET((char)8926);
        p136.terrain_height_SET(1.7400973E37F);
        p136.current_height_SET(3.3766324E38F);
        p136.pending_SET((char)27104);
        p136.loaded_SET((char)10105);
        CommunicationChannel.instance.send(p136); //===============================
        SCALED_PRESSURE2 p137 = CommunicationChannel.instance.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(1997086432L);
        p137.press_abs_SET(5.5420574E37F);
        p137.press_diff_SET(1.6486368E37F);
        p137.temperature_SET((short) -6218);
        CommunicationChannel.instance.send(p137); //===============================
        ATT_POS_MOCAP p138 = CommunicationChannel.instance.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(9197816028347008781L);
        p138.q_SET(new float[4], 0);
        p138.x_SET(-2.6540809E38F);
        p138.y_SET(-2.3109075E37F);
        p138.z_SET(-3.2737856E38F);
        CommunicationChannel.instance.send(p138); //===============================
        SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.instance.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(7207843220563814194L);
        p139.group_mlx_SET((char)224);
        p139.target_system_SET((char)215);
        p139.target_component_SET((char)233);
        p139.controls_SET(new float[8], 0);
        CommunicationChannel.instance.send(p139); //===============================
        ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.instance.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(8986298423909015109L);
        p140.group_mlx_SET((char)203);
        p140.controls_SET(new float[8], 0);
        CommunicationChannel.instance.send(p140); //===============================
        ALTITUDE p141 = CommunicationChannel.instance.new_ALTITUDE();
        PH.setPack(p141);
        p141.time_usec_SET(5926948281985661183L);
        p141.altitude_monotonic_SET(-2.6690454E38F);
        p141.altitude_amsl_SET(-7.888329E37F);
        p141.altitude_local_SET(1.5574126E37F);
        p141.altitude_relative_SET(1.5368412E38F);
        p141.altitude_terrain_SET(-1.0678394E38F);
        p141.bottom_clearance_SET(2.525755E38F);
        CommunicationChannel.instance.send(p141); //===============================
        RESOURCE_REQUEST p142 = CommunicationChannel.instance.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.request_id_SET((char)129);
        p142.uri_type_SET((char)14);
        p142.uri_SET(new char[120], 0);
        p142.transfer_type_SET((char)68);
        p142.storage_SET(new char[120], 0);
        CommunicationChannel.instance.send(p142); //===============================
        SCALED_PRESSURE3 p143 = CommunicationChannel.instance.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(3544760648L);
        p143.press_abs_SET(-5.5995783E37F);
        p143.press_diff_SET(6.0843426E37F);
        p143.temperature_SET((short) -29852);
        CommunicationChannel.instance.send(p143); //===============================
        FOLLOW_TARGET p144 = CommunicationChannel.instance.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.timestamp_SET(5287127582448330178L);
        p144.est_capabilities_SET((char)121);
        p144.lat_SET(38586459);
        p144.lon_SET(1681468257);
        p144.alt_SET(-9.938908E37F);
        p144.vel_SET(new float[3], 0);
        p144.acc_SET(new float[3], 0);
        p144.attitude_q_SET(new float[4], 0);
        p144.rates_SET(new float[3], 0);
        p144.position_cov_SET(new float[3], 0);
        p144.custom_state_SET(1939886508960154695L);
        CommunicationChannel.instance.send(p144); //===============================
        CONTROL_SYSTEM_STATE p146 = CommunicationChannel.instance.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.time_usec_SET(6466411330757674562L);
        p146.x_acc_SET(4.4208615E37F);
        p146.y_acc_SET(8.647602E37F);
        p146.z_acc_SET(-2.4326742E38F);
        p146.x_vel_SET(-2.1825621E38F);
        p146.y_vel_SET(2.3944433E38F);
        p146.z_vel_SET(-3.0476712E38F);
        p146.x_pos_SET(1.3557936E38F);
        p146.y_pos_SET(-2.051335E38F);
        p146.z_pos_SET(1.523755E38F);
        p146.airspeed_SET(2.9885887E38F);
        p146.vel_variance_SET(new float[3], 0);
        p146.pos_variance_SET(new float[3], 0);
        p146.q_SET(new float[4], 0);
        p146.roll_rate_SET(2.648638E38F);
        p146.pitch_rate_SET(4.2071146E37F);
        p146.yaw_rate_SET(8.905204E37F);
        CommunicationChannel.instance.send(p146); //===============================
        BATTERY_STATUS p147 = CommunicationChannel.instance.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.id_SET((char)191);
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN);
        p147.temperature_SET((short) -26549);
        p147.voltages_SET(new char[10], 0);
        p147.current_battery_SET((short) -13264);
        p147.current_consumed_SET(-1680580379);
        p147.energy_consumed_SET(511936929);
        p147.battery_remaining_SET((byte)18);
        CommunicationChannel.instance.send(p147); //===============================
        AUTOPILOT_VERSION p148 = CommunicationChannel.instance.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT));
        p148.flight_sw_version_SET(4135925270L);
        p148.middleware_sw_version_SET(420158204L);
        p148.os_sw_version_SET(1313863341L);
        p148.board_version_SET(3957685174L);
        p148.flight_custom_version_SET(new char[8], 0);
        p148.middleware_custom_version_SET(new char[8], 0);
        p148.os_custom_version_SET(new char[8], 0);
        p148.vendor_id_SET((char)30352);
        p148.product_id_SET((char)9253);
        p148.uid_SET(5341348265520581241L);
        p148.uid2_SET(new char[18], 0, PH);
        CommunicationChannel.instance.send(p148); //===============================
        LANDING_TARGET p149 = CommunicationChannel.instance.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.time_usec_SET(4657277402460356825L);
        p149.target_num_SET((char)187);
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL);
        p149.angle_x_SET(2.5492748E38F);
        p149.angle_y_SET(2.484372E38F);
        p149.distance_SET(-2.2040629E38F);
        p149.size_x_SET(-2.876992E38F);
        p149.size_y_SET(-2.6257388E38F);
        p149.x_SET(-3.014783E38F, PH);
        p149.y_SET(5.6182254E37F, PH);
        p149.z_SET(2.6709185E38F, PH);
        p149.q_SET(new float[4], 0, PH);
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
        p149.position_valid_SET((char)234, PH);
        CommunicationChannel.instance.send(p149); //===============================
        SENS_POWER p201 = CommunicationChannel.instance.new_SENS_POWER();
        PH.setPack(p201);
        p201.adc121_vspb_volt_SET(1.4103971E38F);
        p201.adc121_cspb_amp_SET(6.407243E37F);
        p201.adc121_cs1_amp_SET(-2.0572503E38F);
        p201.adc121_cs2_amp_SET(2.4240666E38F);
        CommunicationChannel.instance.send(p201); //===============================
        SENS_MPPT p202 = CommunicationChannel.instance.new_SENS_MPPT();
        PH.setPack(p202);
        p202.mppt_timestamp_SET(5246901608483416581L);
        p202.mppt1_volt_SET(-2.8744756E38F);
        p202.mppt1_amp_SET(2.0960221E38F);
        p202.mppt1_pwm_SET((char)33926);
        p202.mppt1_status_SET((char)199);
        p202.mppt2_volt_SET(-1.2117467E38F);
        p202.mppt2_amp_SET(-9.821654E37F);
        p202.mppt2_pwm_SET((char)12637);
        p202.mppt2_status_SET((char)111);
        p202.mppt3_volt_SET(2.2154393E38F);
        p202.mppt3_amp_SET(-1.7152839E37F);
        p202.mppt3_pwm_SET((char)30434);
        p202.mppt3_status_SET((char)215);
        CommunicationChannel.instance.send(p202); //===============================
        ASLCTRL_DATA p203 = CommunicationChannel.instance.new_ASLCTRL_DATA();
        PH.setPack(p203);
        p203.timestamp_SET(4242219466853425276L);
        p203.aslctrl_mode_SET((char)32);
        p203.h_SET(2.772711E38F);
        p203.hRef_SET(2.1540576E38F);
        p203.hRef_t_SET(-1.2905837E38F);
        p203.PitchAngle_SET(-2.4154206E38F);
        p203.PitchAngleRef_SET(-9.038432E37F);
        p203.q_SET(4.4327796E36F);
        p203.qRef_SET(-1.0291414E36F);
        p203.uElev_SET(1.2477813E38F);
        p203.uThrot_SET(-2.0013284E38F);
        p203.uThrot2_SET(1.6305452E38F);
        p203.nZ_SET(1.6355243E37F);
        p203.AirspeedRef_SET(3.40252E37F);
        p203.SpoilersEngaged_SET((char)254);
        p203.YawAngle_SET(2.270113E38F);
        p203.YawAngleRef_SET(-3.0085626E38F);
        p203.RollAngle_SET(1.779945E38F);
        p203.RollAngleRef_SET(-1.195728E38F);
        p203.p_SET(3.1960287E38F);
        p203.pRef_SET(2.4893206E38F);
        p203.r_SET(-7.3706003E37F);
        p203.rRef_SET(3.049571E38F);
        p203.uAil_SET(-2.1443263E38F);
        p203.uRud_SET(-1.7866782E37F);
        CommunicationChannel.instance.send(p203); //===============================
        ASLCTRL_DEBUG p204 = CommunicationChannel.instance.new_ASLCTRL_DEBUG();
        PH.setPack(p204);
        p204.i32_1_SET(1604188583L);
        p204.i8_1_SET((char)20);
        p204.i8_2_SET((char)118);
        p204.f_1_SET(2.8883607E37F);
        p204.f_2_SET(-2.8628447E38F);
        p204.f_3_SET(2.240384E38F);
        p204.f_4_SET(-5.442156E36F);
        p204.f_5_SET(9.893658E37F);
        p204.f_6_SET(-3.0943203E38F);
        p204.f_7_SET(-3.1162125E38F);
        p204.f_8_SET(-3.4681085E37F);
        CommunicationChannel.instance.send(p204); //===============================
        ASLUAV_STATUS p205 = CommunicationChannel.instance.new_ASLUAV_STATUS();
        PH.setPack(p205);
        p205.LED_status_SET((char)108);
        p205.SATCOM_status_SET((char)188);
        p205.Servo_status_SET(new char[8], 0);
        p205.Motor_rpm_SET(-3.3984325E38F);
        CommunicationChannel.instance.send(p205); //===============================
        EKF_EXT p206 = CommunicationChannel.instance.new_EKF_EXT();
        PH.setPack(p206);
        p206.timestamp_SET(310732556837932594L);
        p206.Windspeed_SET(-3.2641352E38F);
        p206.WindDir_SET(6.30502E37F);
        p206.WindZ_SET(-8.5485216E36F);
        p206.Airspeed_SET(1.2294881E38F);
        p206.beta_SET(1.6864675E37F);
        p206.alpha_SET(-2.0265387E38F);
        CommunicationChannel.instance.send(p206); //===============================
        ASL_OBCTRL p207 = CommunicationChannel.instance.new_ASL_OBCTRL();
        PH.setPack(p207);
        p207.timestamp_SET(1389052079010213758L);
        p207.uElev_SET(-4.3221577E37F);
        p207.uThrot_SET(-1.5488853E38F);
        p207.uThrot2_SET(2.0831102E38F);
        p207.uAilL_SET(-3.0391169E38F);
        p207.uAilR_SET(-2.6667324E38F);
        p207.uRud_SET(3.5195784E37F);
        p207.obctrl_status_SET((char)53);
        CommunicationChannel.instance.send(p207); //===============================
        SENS_ATMOS p208 = CommunicationChannel.instance.new_SENS_ATMOS();
        PH.setPack(p208);
        p208.TempAmbient_SET(-3.2123205E38F);
        p208.Humidity_SET(-6.687417E36F);
        CommunicationChannel.instance.send(p208); //===============================
        SENS_BATMON p209 = CommunicationChannel.instance.new_SENS_BATMON();
        PH.setPack(p209);
        p209.temperature_SET(2.6140726E38F);
        p209.voltage_SET((char)38317);
        p209.current_SET((short)23471);
        p209.SoC_SET((char)252);
        p209.batterystatus_SET((char)49527);
        p209.serialnumber_SET((char)60270);
        p209.hostfetcontrol_SET((char)12353);
        p209.cellvoltage1_SET((char)25860);
        p209.cellvoltage2_SET((char)46534);
        p209.cellvoltage3_SET((char)24617);
        p209.cellvoltage4_SET((char)47097);
        p209.cellvoltage5_SET((char)45327);
        p209.cellvoltage6_SET((char)10329);
        CommunicationChannel.instance.send(p209); //===============================
        FW_SOARING_DATA p210 = CommunicationChannel.instance.new_FW_SOARING_DATA();
        PH.setPack(p210);
        p210.timestamp_SET(3716604060848370762L);
        p210.timestampModeChanged_SET(7487900533233245190L);
        p210.xW_SET(2.1809338E38F);
        p210.xR_SET(3.2937877E38F);
        p210.xLat_SET(-4.673911E36F);
        p210.xLon_SET(3.1092591E38F);
        p210.VarW_SET(1.803493E38F);
        p210.VarR_SET(-2.1335113E38F);
        p210.VarLat_SET(-1.1315347E38F);
        p210.VarLon_SET(1.7604274E38F);
        p210.LoiterRadius_SET(1.3974411E38F);
        p210.LoiterDirection_SET(3.3359326E37F);
        p210.DistToSoarPoint_SET(8.840957E37F);
        p210.vSinkExp_SET(1.2972927E38F);
        p210.z1_LocalUpdraftSpeed_SET(-2.614429E38F);
        p210.z2_DeltaRoll_SET(-3.1691265E38F);
        p210.z1_exp_SET(2.06502E38F);
        p210.z2_exp_SET(8.187666E37F);
        p210.ThermalGSNorth_SET(2.5912867E38F);
        p210.ThermalGSEast_SET(1.6001114E38F);
        p210.TSE_dot_SET(3.3474797E38F);
        p210.DebugVar1_SET(1.7980744E38F);
        p210.DebugVar2_SET(2.0435905E38F);
        p210.ControlMode_SET((char)172);
        p210.valid_SET((char)22);
        CommunicationChannel.instance.send(p210); //===============================
        SENSORPOD_STATUS p211 = CommunicationChannel.instance.new_SENSORPOD_STATUS();
        PH.setPack(p211);
        p211.timestamp_SET(1740946523134962116L);
        p211.visensor_rate_1_SET((char)223);
        p211.visensor_rate_2_SET((char)16);
        p211.visensor_rate_3_SET((char)33);
        p211.visensor_rate_4_SET((char)115);
        p211.recording_nodes_count_SET((char)123);
        p211.cpu_temp_SET((char)48);
        p211.free_space_SET((char)58637);
        CommunicationChannel.instance.send(p211); //===============================
        SENS_POWER_BOARD p212 = CommunicationChannel.instance.new_SENS_POWER_BOARD();
        PH.setPack(p212);
        p212.timestamp_SET(1502795535723410447L);
        p212.pwr_brd_status_SET((char)197);
        p212.pwr_brd_led_status_SET((char)124);
        p212.pwr_brd_system_volt_SET(5.9027167E37F);
        p212.pwr_brd_servo_volt_SET(2.4460645E38F);
        p212.pwr_brd_mot_l_amp_SET(-1.3304153E38F);
        p212.pwr_brd_mot_r_amp_SET(-3.2190707E38F);
        p212.pwr_brd_servo_1_amp_SET(3.2942836E38F);
        p212.pwr_brd_servo_2_amp_SET(-1.8172443E38F);
        p212.pwr_brd_servo_3_amp_SET(3.0478178E38F);
        p212.pwr_brd_servo_4_amp_SET(1.7088666E38F);
        p212.pwr_brd_aux_amp_SET(3.4844447E37F);
        CommunicationChannel.instance.send(p212); //===============================
        ESTIMATOR_STATUS p230 = CommunicationChannel.instance.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.time_usec_SET(8256706034982193996L);
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL));
        p230.vel_ratio_SET(-6.2902483E36F);
        p230.pos_horiz_ratio_SET(3.36946E38F);
        p230.pos_vert_ratio_SET(-2.541895E38F);
        p230.mag_ratio_SET(-3.008879E38F);
        p230.hagl_ratio_SET(2.8683566E38F);
        p230.tas_ratio_SET(-1.2617829E38F);
        p230.pos_horiz_accuracy_SET(5.333649E37F);
        p230.pos_vert_accuracy_SET(-2.8034207E38F);
        CommunicationChannel.instance.send(p230); //===============================
        WIND_COV p231 = CommunicationChannel.instance.new_WIND_COV();
        PH.setPack(p231);
        p231.time_usec_SET(12907555235777148L);
        p231.wind_x_SET(-2.2796846E38F);
        p231.wind_y_SET(7.989844E37F);
        p231.wind_z_SET(-2.6336376E38F);
        p231.var_horiz_SET(5.4703956E37F);
        p231.var_vert_SET(1.8181115E38F);
        p231.wind_alt_SET(-2.6241327E38F);
        p231.horiz_accuracy_SET(3.0009466E38F);
        p231.vert_accuracy_SET(3.3296827E38F);
        CommunicationChannel.instance.send(p231); //===============================
        GPS_INPUT p232 = CommunicationChannel.instance.new_GPS_INPUT();
        PH.setPack(p232);
        p232.time_usec_SET(2091160440789233944L);
        p232.gps_id_SET((char)49);
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT));
        p232.time_week_ms_SET(100276533L);
        p232.time_week_SET((char)58278);
        p232.fix_type_SET((char)61);
        p232.lat_SET(-295265656);
        p232.lon_SET(1726610340);
        p232.alt_SET(2.5576031E38F);
        p232.hdop_SET(-2.6594845E38F);
        p232.vdop_SET(-2.7294168E38F);
        p232.vn_SET(-2.1906593E38F);
        p232.ve_SET(2.9499185E38F);
        p232.vd_SET(1.6802428E38F);
        p232.speed_accuracy_SET(-1.9981352E38F);
        p232.horiz_accuracy_SET(-3.0339874E38F);
        p232.vert_accuracy_SET(2.8076566E38F);
        p232.satellites_visible_SET((char)158);
        CommunicationChannel.instance.send(p232); //===============================
        GPS_RTCM_DATA p233 = CommunicationChannel.instance.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)194);
        p233.len_SET((char)177);
        p233.data__SET(new char[180], 0);
        CommunicationChannel.instance.send(p233); //===============================
        HIGH_LATENCY p234 = CommunicationChannel.instance.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED));
        p234.custom_mode_SET(1534113465L);
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_TAKEOFF);
        p234.roll_SET((short)22242);
        p234.pitch_SET((short) -23496);
        p234.heading_SET((char)33450);
        p234.throttle_SET((byte) - 15);
        p234.heading_sp_SET((short) -1856);
        p234.latitude_SET(1841565422);
        p234.longitude_SET(907344176);
        p234.altitude_amsl_SET((short) -18467);
        p234.altitude_sp_SET((short) -32217);
        p234.airspeed_SET((char)65);
        p234.airspeed_sp_SET((char)5);
        p234.groundspeed_SET((char)232);
        p234.climb_rate_SET((byte)1);
        p234.gps_nsat_SET((char)124);
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
        p234.battery_remaining_SET((char)112);
        p234.temperature_SET((byte)91);
        p234.temperature_air_SET((byte)34);
        p234.failsafe_SET((char)98);
        p234.wp_num_SET((char)250);
        p234.wp_distance_SET((char)48709);
        CommunicationChannel.instance.send(p234); //===============================
        VIBRATION p241 = CommunicationChannel.instance.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(1354607790402766541L);
        p241.vibration_x_SET(1.1228826E38F);
        p241.vibration_y_SET(-1.786591E38F);
        p241.vibration_z_SET(-2.4348315E38F);
        p241.clipping_0_SET(2497379058L);
        p241.clipping_1_SET(3888469741L);
        p241.clipping_2_SET(557930478L);
        CommunicationChannel.instance.send(p241); //===============================
        HOME_POSITION p242 = CommunicationChannel.instance.new_HOME_POSITION();
        PH.setPack(p242);
        p242.latitude_SET(758258235);
        p242.longitude_SET(1840346481);
        p242.altitude_SET(991465094);
        p242.x_SET(1.977214E38F);
        p242.y_SET(1.5267592E38F);
        p242.z_SET(3.2059464E37F);
        p242.q_SET(new float[4], 0);
        p242.approach_x_SET(-1.5688674E38F);
        p242.approach_y_SET(4.9750246E37F);
        p242.approach_z_SET(-2.6336461E38F);
        p242.time_usec_SET(7019555368585398521L, PH);
        CommunicationChannel.instance.send(p242); //===============================
        SET_HOME_POSITION p243 = CommunicationChannel.instance.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.target_system_SET((char)44);
        p243.latitude_SET(-1088093630);
        p243.longitude_SET(436791963);
        p243.altitude_SET(-1067178706);
        p243.x_SET(2.21099E38F);
        p243.y_SET(2.609025E37F);
        p243.z_SET(2.6155248E38F);
        p243.q_SET(new float[4], 0);
        p243.approach_x_SET(8.4928645E37F);
        p243.approach_y_SET(2.2052255E38F);
        p243.approach_z_SET(-2.3192911E38F);
        p243.time_usec_SET(8399005078470948334L, PH);
        CommunicationChannel.instance.send(p243); //===============================
        MESSAGE_INTERVAL p244 = CommunicationChannel.instance.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)49396);
        p244.interval_us_SET(-165741891);
        CommunicationChannel.instance.send(p244); //===============================
        EXTENDED_SYS_STATE p245 = CommunicationChannel.instance.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
        CommunicationChannel.instance.send(p245); //===============================
        ADSB_VEHICLE p246 = CommunicationChannel.instance.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.ICAO_address_SET(2337716273L);
        p246.lat_SET(-1874163909);
        p246.lon_SET(-1703938811);
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
        p246.altitude_SET(-930155583);
        p246.heading_SET((char)13867);
        p246.hor_velocity_SET((char)14010);
        p246.ver_velocity_SET((short)21556);
        p246.callsign_SET("DEMO", PH);
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED2);
        p246.tslc_SET((char)52);
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK |
                        ADSB_FLAGS.ADSB_FLAGS_VALID_HEADING |
                        ADSB_FLAGS.ADSB_FLAGS_SIMULATED));
        p246.squawk_SET((char)5296);
        CommunicationChannel.instance.send(p246); //===============================
        COLLISION p247 = CommunicationChannel.instance.new_COLLISION();
        PH.setPack(p247);
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
        p247.id_SET(1963072160L);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL);
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
        p247.time_to_minimum_delta_SET(7.252245E37F);
        p247.altitude_minimum_delta_SET(-2.8073404E38F);
        p247.horizontal_minimum_delta_SET(1.6508033E38F);
        CommunicationChannel.instance.send(p247); //===============================
        V2_EXTENSION p248 = CommunicationChannel.instance.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)182);
        p248.target_system_SET((char)228);
        p248.target_component_SET((char)177);
        p248.message_type_SET((char)40205);
        p248.payload_SET(new char[249], 0);
        CommunicationChannel.instance.send(p248); //===============================
        MEMORY_VECT p249 = CommunicationChannel.instance.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)39767);
        p249.ver_SET((char)103);
        p249.type_SET((char)191);
        p249.value_SET(new byte[32], 0);
        CommunicationChannel.instance.send(p249); //===============================
        DEBUG_VECT p250 = CommunicationChannel.instance.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("DEMO", PH);
        p250.time_usec_SET(2198675521325991908L);
        p250.x_SET(1.3093073E38F);
        p250.y_SET(-2.9326717E38F);
        p250.z_SET(-3.0116676E37F);
        CommunicationChannel.instance.send(p250); //===============================
        NAMED_VALUE_FLOAT p251 = CommunicationChannel.instance.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(4019066844L);
        p251.name_SET("DEMO", PH);
        p251.value_SET(-2.77375E38F);
        CommunicationChannel.instance.send(p251); //===============================
        NAMED_VALUE_INT p252 = CommunicationChannel.instance.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(14903616L);
        p252.name_SET("DEMO", PH);
        p252.value_SET(-1403821636);
        CommunicationChannel.instance.send(p252); //===============================
        STATUSTEXT p253 = CommunicationChannel.instance.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_ALERT);
        p253.text_SET("DEMO", PH);
        CommunicationChannel.instance.send(p253); //===============================
        DEBUG p254 = CommunicationChannel.instance.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(3814873331L);
        p254.ind_SET((char)14);
        p254.value_SET(-2.7896398E38F);
        CommunicationChannel.instance.send(p254); //===============================
        SETUP_SIGNING p256 = CommunicationChannel.instance.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)116);
        p256.target_component_SET((char)91);
        p256.secret_key_SET(new char[32], 0);
        p256.initial_timestamp_SET(7957730526197928748L);
        CommunicationChannel.instance.send(p256); //===============================
        BUTTON_CHANGE p257 = CommunicationChannel.instance.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(2648970988L);
        p257.last_change_ms_SET(2394654779L);
        p257.state_SET((char)107);
        CommunicationChannel.instance.send(p257); //===============================
        PLAY_TUNE p258 = CommunicationChannel.instance.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)107);
        p258.target_component_SET((char)155);
        p258.tune_SET("DEMO", PH);
        CommunicationChannel.instance.send(p258); //===============================
        CAMERA_INFORMATION p259 = CommunicationChannel.instance.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.time_boot_ms_SET(1757839169L);
        p259.vendor_name_SET(new char[32], 0);
        p259.model_name_SET(new char[32], 0);
        p259.firmware_version_SET(1740955976L);
        p259.focal_length_SET(-2.3406925E38F);
        p259.sensor_size_h_SET(-1.1145048E37F);
        p259.sensor_size_v_SET(-2.9658148E38F);
        p259.resolution_h_SET((char)36341);
        p259.resolution_v_SET((char)63059);
        p259.lens_id_SET((char)102);
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE));
        p259.cam_definition_version_SET((char)2155);
        p259.cam_definition_uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p259); //===============================
        CAMERA_SETTINGS p260 = CommunicationChannel.instance.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(3241169991L);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE);
        CommunicationChannel.instance.send(p260); //===============================
        STORAGE_INFORMATION p261 = CommunicationChannel.instance.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.time_boot_ms_SET(3104559058L);
        p261.storage_id_SET((char)194);
        p261.storage_count_SET((char)87);
        p261.status_SET((char)112);
        p261.total_capacity_SET(2.4042123E38F);
        p261.used_capacity_SET(1.1003408E38F);
        p261.available_capacity_SET(6.1077627E37F);
        p261.read_speed_SET(-7.0793565E37F);
        p261.write_speed_SET(1.6505902E38F);
        CommunicationChannel.instance.send(p261); //===============================
        CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.instance.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.time_boot_ms_SET(4023696238L);
        p262.image_status_SET((char)20);
        p262.video_status_SET((char)217);
        p262.image_interval_SET(-1.8676503E38F);
        p262.recording_time_ms_SET(1474697648L);
        p262.available_capacity_SET(-1.4552373E38F);
        CommunicationChannel.instance.send(p262); //===============================
        CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.instance.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.time_boot_ms_SET(2706438586L);
        p263.time_utc_SET(5229887447239135929L);
        p263.camera_id_SET((char)54);
        p263.lat_SET(459269488);
        p263.lon_SET(-1609715913);
        p263.alt_SET(-1940141501);
        p263.relative_alt_SET(425728048);
        p263.q_SET(new float[4], 0);
        p263.image_index_SET(1218804369);
        p263.capture_result_SET((byte) - 76);
        p263.file_url_SET("DEMO", PH);
        CommunicationChannel.instance.send(p263); //===============================
        FLIGHT_INFORMATION p264 = CommunicationChannel.instance.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.time_boot_ms_SET(1798230791L);
        p264.arming_time_utc_SET(2984388492611470421L);
        p264.takeoff_time_utc_SET(2431138811420408276L);
        p264.flight_uuid_SET(4258074047648291011L);
        CommunicationChannel.instance.send(p264); //===============================
        MOUNT_ORIENTATION p265 = CommunicationChannel.instance.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(2754856767L);
        p265.roll_SET(3.1839106E38F);
        p265.pitch_SET(-1.3255794E38F);
        p265.yaw_SET(-2.6827365E38F);
        CommunicationChannel.instance.send(p265); //===============================
        LOGGING_DATA p266 = CommunicationChannel.instance.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_system_SET((char)100);
        p266.target_component_SET((char)90);
        p266.sequence_SET((char)57336);
        p266.length_SET((char)197);
        p266.first_message_offset_SET((char)41);
        p266.data__SET(new char[249], 0);
        CommunicationChannel.instance.send(p266); //===============================
        LOGGING_DATA_ACKED p267 = CommunicationChannel.instance.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_system_SET((char)185);
        p267.target_component_SET((char)78);
        p267.sequence_SET((char)42297);
        p267.length_SET((char)47);
        p267.first_message_offset_SET((char)195);
        p267.data__SET(new char[249], 0);
        CommunicationChannel.instance.send(p267); //===============================
        LOGGING_ACK p268 = CommunicationChannel.instance.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)43);
        p268.target_component_SET((char)81);
        p268.sequence_SET((char)6984);
        CommunicationChannel.instance.send(p268); //===============================
        VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.instance.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.camera_id_SET((char)24);
        p269.status_SET((char)199);
        p269.framerate_SET(1.0749176E37F);
        p269.resolution_h_SET((char)47313);
        p269.resolution_v_SET((char)55890);
        p269.bitrate_SET(471059478L);
        p269.rotation_SET((char)31200);
        p269.uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p269); //===============================
        SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.instance.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_system_SET((char)227);
        p270.target_component_SET((char)29);
        p270.camera_id_SET((char)242);
        p270.framerate_SET(6.196954E37F);
        p270.resolution_h_SET((char)32874);
        p270.resolution_v_SET((char)5461);
        p270.bitrate_SET(564490020L);
        p270.rotation_SET((char)20792);
        p270.uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p270); //===============================
        WIFI_CONFIG_AP p299 = CommunicationChannel.instance.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("DEMO", PH);
        p299.password_SET("DEMO", PH);
        CommunicationChannel.instance.send(p299); //===============================
        PROTOCOL_VERSION p300 = CommunicationChannel.instance.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)10045);
        p300.min_version_SET((char)37820);
        p300.max_version_SET((char)60079);
        p300.spec_version_hash_SET(new char[8], 0);
        p300.library_version_hash_SET(new char[8], 0);
        CommunicationChannel.instance.send(p300); //===============================
        UAVCAN_NODE_STATUS p310 = CommunicationChannel.instance.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.time_usec_SET(5629463745390523095L);
        p310.uptime_sec_SET(2185701079L);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
        p310.sub_mode_SET((char)108);
        p310.vendor_specific_status_code_SET((char)37125);
        CommunicationChannel.instance.send(p310); //===============================
        UAVCAN_NODE_INFO p311 = CommunicationChannel.instance.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.time_usec_SET(2541280166882898026L);
        p311.uptime_sec_SET(1003675359L);
        p311.name_SET("DEMO", PH);
        p311.hw_version_major_SET((char)52);
        p311.hw_version_minor_SET((char)126);
        p311.hw_unique_id_SET(new char[16], 0);
        p311.sw_version_major_SET((char)237);
        p311.sw_version_minor_SET((char)102);
        p311.sw_vcs_commit_SET(2975762795L);
        CommunicationChannel.instance.send(p311); //===============================
        PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)145);
        p320.target_component_SET((char)166);
        p320.param_id_SET("DEMO", PH);
        p320.param_index_SET((short) -14912);
        CommunicationChannel.instance.send(p320); //===============================
        PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)43);
        p321.target_component_SET((char)39);
        CommunicationChannel.instance.send(p321); //===============================
        PARAM_EXT_VALUE p322 = CommunicationChannel.instance.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("DEMO", PH);
        p322.param_value_SET("DEMO", PH);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
        p322.param_count_SET((char)32059);
        p322.param_index_SET((char)43885);
        CommunicationChannel.instance.send(p322); //===============================
        PARAM_EXT_SET p323 = CommunicationChannel.instance.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)91);
        p323.target_component_SET((char)207);
        p323.param_id_SET("DEMO", PH);
        p323.param_value_SET("DEMO", PH);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
        CommunicationChannel.instance.send(p323); //===============================
        PARAM_EXT_ACK p324 = CommunicationChannel.instance.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("DEMO", PH);
        p324.param_value_SET("DEMO", PH);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_FAILED);
        CommunicationChannel.instance.send(p324); //===============================
        OBSTACLE_DISTANCE p330 = CommunicationChannel.instance.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.time_usec_SET(628748174304619831L);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
        p330.distances_SET(new char[72], 0);
        p330.increment_SET((char)252);
        p330.min_distance_SET((char)39185);
        p330.max_distance_SET((char)55757);
        CommunicationChannel.instance.send(p330); //===============================
    }
}
