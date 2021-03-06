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
        p3.time_boot_ms_SET(3678480432L);
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
        p3.type_mask_SET((char)42546);
        p3.x_SET(3.1467658E38F);
        p3.y_SET(1.718189E38F);
        p3.z_SET(-1.229348E38F);
        p3.vx_SET(-2.0187817E38F);
        p3.vy_SET(-1.938392E38F);
        p3.vz_SET(3.1871237E38F);
        p3.afx_SET(1.6597503E38F);
        p3.afy_SET(-5.6611897E37F);
        p3.afz_SET(-2.0470123E38F);
        p3.yaw_SET(-5.083693E36F);
        p3.yaw_rate_SET(2.7248736E38F);
        CommunicationChannel.instance.send(p3); //===============================
        SET_ATTITUDE_TARGET p82 = CommunicationChannel.instance.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.time_boot_ms_SET(587560992L);
        p82.target_system_SET((char)230);
        p82.target_component_SET((char)250);
        p82.type_mask_SET((char)211);
        p82.q_SET(new float[4], 0);
        p82.body_roll_rate_SET(2.885139E38F);
        p82.body_pitch_rate_SET(1.3806252E38F);
        p82.body_yaw_rate_SET(4.839142E37F);
        p82.thrust_SET(4.489358E37F);
        CommunicationChannel.instance.send(p82); //===============================
        ATTITUDE_TARGET p83 = CommunicationChannel.instance.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.time_boot_ms_SET(2015075556L);
        p83.type_mask_SET((char)4);
        p83.q_SET(new float[4], 0);
        p83.body_roll_rate_SET(2.133216E38F);
        p83.body_pitch_rate_SET(-2.3897575E38F);
        p83.body_yaw_rate_SET(3.108875E38F);
        p83.thrust_SET(2.2002705E37F);
        CommunicationChannel.instance.send(p83); //===============================
        SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.instance.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.time_boot_ms_SET(3300401996L);
        p84.target_system_SET((char)180);
        p84.target_component_SET((char)161);
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED);
        p84.type_mask_SET((char)4744);
        p84.x_SET(-2.8038249E38F);
        p84.y_SET(1.1960617E38F);
        p84.z_SET(2.5800055E38F);
        p84.vx_SET(3.2274814E38F);
        p84.vy_SET(9.536827E37F);
        p84.vz_SET(-9.28116E37F);
        p84.afx_SET(-3.09568E37F);
        p84.afy_SET(3.1930419E38F);
        p84.afz_SET(-2.7986486E38F);
        p84.yaw_SET(-4.546499E37F);
        p84.yaw_rate_SET(1.6433902E37F);
        CommunicationChannel.instance.send(p84); //===============================
        SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.instance.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.time_boot_ms_SET(123045668L);
        p86.target_system_SET((char)131);
        p86.target_component_SET((char)137);
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
        p86.type_mask_SET((char)15128);
        p86.lat_int_SET(-2032628955);
        p86.lon_int_SET(415478600);
        p86.alt_SET(-1.7948339E37F);
        p86.vx_SET(-1.552591E38F);
        p86.vy_SET(-2.1329822E38F);
        p86.vz_SET(1.1476756E34F);
        p86.afx_SET(-1.6102751E38F);
        p86.afy_SET(7.8810674E36F);
        p86.afz_SET(-2.9875634E38F);
        p86.yaw_SET(3.0952835E38F);
        p86.yaw_rate_SET(1.132895E38F);
        CommunicationChannel.instance.send(p86); //===============================
        POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.instance.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.time_boot_ms_SET(4199470303L);
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
        p87.type_mask_SET((char)35651);
        p87.lat_int_SET(-538822306);
        p87.lon_int_SET(-2013208131);
        p87.alt_SET(3.3034518E38F);
        p87.vx_SET(-2.0762689E38F);
        p87.vy_SET(-1.2538066E38F);
        p87.vz_SET(-2.7918258E38F);
        p87.afx_SET(-2.0429019E38F);
        p87.afy_SET(-1.970816E38F);
        p87.afz_SET(-2.6078727E37F);
        p87.yaw_SET(1.1735978E38F);
        p87.yaw_rate_SET(9.172257E37F);
        CommunicationChannel.instance.send(p87); //===============================
        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.instance.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.time_boot_ms_SET(2782623549L);
        p89.x_SET(-2.907293E38F);
        p89.y_SET(-1.2424863E38F);
        p89.z_SET(3.3648428E38F);
        p89.roll_SET(-7.2666154E37F);
        p89.pitch_SET(-7.26409E37F);
        p89.yaw_SET(-3.2147532E38F);
        CommunicationChannel.instance.send(p89); //===============================
        HIL_STATE p90 = CommunicationChannel.instance.new_HIL_STATE();
        PH.setPack(p90);
        p90.time_usec_SET(7298853979676238222L);
        p90.roll_SET(2.96098E38F);
        p90.pitch_SET(-2.3939105E38F);
        p90.yaw_SET(-3.0158373E38F);
        p90.rollspeed_SET(-1.0425129E38F);
        p90.pitchspeed_SET(-2.0545272E38F);
        p90.yawspeed_SET(2.4755259E38F);
        p90.lat_SET(1287404168);
        p90.lon_SET(-111690811);
        p90.alt_SET(796780789);
        p90.vx_SET((short) -1844);
        p90.vy_SET((short)101);
        p90.vz_SET((short) -19433);
        p90.xacc_SET((short)29718);
        p90.yacc_SET((short) -15);
        p90.zacc_SET((short)10140);
        CommunicationChannel.instance.send(p90); //===============================
        HIL_CONTROLS p91 = CommunicationChannel.instance.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.time_usec_SET(5945115225065725477L);
        p91.roll_ailerons_SET(1.5797716E38F);
        p91.pitch_elevator_SET(-1.7061868E37F);
        p91.yaw_rudder_SET(5.0732133E37F);
        p91.throttle_SET(-2.0352708E38F);
        p91.aux1_SET(-1.4945005E38F);
        p91.aux2_SET(6.0761343E37F);
        p91.aux3_SET(-2.3914717E38F);
        p91.aux4_SET(-1.0413412E38F);
        p91.mode_SET(MAV_MODE.MAV_MODE_GUIDED_DISARMED);
        p91.nav_mode_SET((char)242);
        CommunicationChannel.instance.send(p91); //===============================
        HIL_RC_INPUTS_RAW p92 = CommunicationChannel.instance.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.time_usec_SET(8232218302891203423L);
        p92.chan1_raw_SET((char)48704);
        p92.chan2_raw_SET((char)24058);
        p92.chan3_raw_SET((char)46071);
        p92.chan4_raw_SET((char)585);
        p92.chan5_raw_SET((char)18197);
        p92.chan6_raw_SET((char)3615);
        p92.chan7_raw_SET((char)26459);
        p92.chan8_raw_SET((char)1666);
        p92.chan9_raw_SET((char)22786);
        p92.chan10_raw_SET((char)13098);
        p92.chan11_raw_SET((char)48323);
        p92.chan12_raw_SET((char)614);
        p92.rssi_SET((char)105);
        CommunicationChannel.instance.send(p92); //===============================
        HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.instance.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(1635488707626889357L);
        p93.controls_SET(new float[16], 0);
        p93.mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED);
        p93.flags_SET(3018869270837769657L);
        CommunicationChannel.instance.send(p93); //===============================
        OPTICAL_FLOW p100 = CommunicationChannel.instance.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.time_usec_SET(8011413177692268146L);
        p100.sensor_id_SET((char)157);
        p100.flow_x_SET((short)26223);
        p100.flow_y_SET((short) -23973);
        p100.flow_comp_m_x_SET(-8.804099E36F);
        p100.flow_comp_m_y_SET(4.229269E37F);
        p100.quality_SET((char)204);
        p100.ground_distance_SET(2.757177E38F);
        p100.flow_rate_x_SET(2.0223404E38F, PH);
        p100.flow_rate_y_SET(-4.830015E37F, PH);
        CommunicationChannel.instance.send(p100); //===============================
        GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.instance.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.usec_SET(2013297711108830188L);
        p101.x_SET(1.9061031E38F);
        p101.y_SET(2.034768E38F);
        p101.z_SET(-2.8561194E38F);
        p101.roll_SET(2.918678E38F);
        p101.pitch_SET(-8.904187E37F);
        p101.yaw_SET(-1.5698466E38F);
        CommunicationChannel.instance.send(p101); //===============================
        VISION_POSITION_ESTIMATE p102 = CommunicationChannel.instance.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.usec_SET(9055118568361899938L);
        p102.x_SET(-2.0930003E38F);
        p102.y_SET(-1.2193095E38F);
        p102.z_SET(2.036615E38F);
        p102.roll_SET(1.7628144E38F);
        p102.pitch_SET(1.2947097E38F);
        p102.yaw_SET(-2.3064195E38F);
        CommunicationChannel.instance.send(p102); //===============================
        VISION_SPEED_ESTIMATE p103 = CommunicationChannel.instance.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(8175100797541516986L);
        p103.x_SET(1.7406786E38F);
        p103.y_SET(3.2358554E38F);
        p103.z_SET(9.736504E37F);
        CommunicationChannel.instance.send(p103); //===============================
        VICON_POSITION_ESTIMATE p104 = CommunicationChannel.instance.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(7805138300007178676L);
        p104.x_SET(3.1076296E36F);
        p104.y_SET(-2.4260254E37F);
        p104.z_SET(1.7100785E38F);
        p104.roll_SET(1.8498878E38F);
        p104.pitch_SET(1.492942E38F);
        p104.yaw_SET(3.1871E36F);
        CommunicationChannel.instance.send(p104); //===============================
        HIGHRES_IMU p105 = CommunicationChannel.instance.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.time_usec_SET(7334923245733036967L);
        p105.xacc_SET(6.501047E37F);
        p105.yacc_SET(3.3446734E38F);
        p105.zacc_SET(-3.808937E37F);
        p105.xgyro_SET(-2.5283693E38F);
        p105.ygyro_SET(2.371918E38F);
        p105.zgyro_SET(5.1589074E37F);
        p105.xmag_SET(1.6272138E38F);
        p105.ymag_SET(3.35861E38F);
        p105.zmag_SET(-3.8325585E37F);
        p105.abs_pressure_SET(2.0338079E38F);
        p105.diff_pressure_SET(1.6040133E38F);
        p105.pressure_alt_SET(-1.0938669E38F);
        p105.temperature_SET(1.2156132E38F);
        p105.fields_updated_SET((char)5649);
        CommunicationChannel.instance.send(p105); //===============================
        OPTICAL_FLOW_RAD p106 = CommunicationChannel.instance.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_usec_SET(8289996674078455876L);
        p106.sensor_id_SET((char)80);
        p106.integration_time_us_SET(2243061843L);
        p106.integrated_x_SET(2.1371918E38F);
        p106.integrated_y_SET(-4.36045E37F);
        p106.integrated_xgyro_SET(-2.1688567E38F);
        p106.integrated_ygyro_SET(-1.4468676E38F);
        p106.integrated_zgyro_SET(-3.2378725E38F);
        p106.temperature_SET((short)17483);
        p106.quality_SET((char)167);
        p106.time_delta_distance_us_SET(2461188289L);
        p106.distance_SET(-2.4444936E38F);
        CommunicationChannel.instance.send(p106); //===============================
        HIL_SENSOR p107 = CommunicationChannel.instance.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.time_usec_SET(109471469173323191L);
        p107.xacc_SET(2.130035E38F);
        p107.yacc_SET(-3.5838546E37F);
        p107.zacc_SET(1.4439065E37F);
        p107.xgyro_SET(-1.5144807E38F);
        p107.ygyro_SET(2.2981363E38F);
        p107.zgyro_SET(-1.038013E38F);
        p107.xmag_SET(-9.321285E36F);
        p107.ymag_SET(-3.2545118E38F);
        p107.zmag_SET(6.9254774E37F);
        p107.abs_pressure_SET(-2.2298215E38F);
        p107.diff_pressure_SET(-5.8632344E37F);
        p107.pressure_alt_SET(-8.216916E37F);
        p107.temperature_SET(2.4898641E38F);
        p107.fields_updated_SET(1019641581L);
        CommunicationChannel.instance.send(p107); //===============================
        SIM_STATE p108 = CommunicationChannel.instance.new_SIM_STATE();
        PH.setPack(p108);
        p108.q1_SET(2.6262812E38F);
        p108.q2_SET(2.0507642E38F);
        p108.q3_SET(1.1154398E38F);
        p108.q4_SET(2.392897E37F);
        p108.roll_SET(1.4797263E38F);
        p108.pitch_SET(-1.8005975E38F);
        p108.yaw_SET(-2.4045162E37F);
        p108.xacc_SET(3.8831687E36F);
        p108.yacc_SET(-3.3480303E38F);
        p108.zacc_SET(5.812574E36F);
        p108.xgyro_SET(-2.634642E38F);
        p108.ygyro_SET(4.890473E37F);
        p108.zgyro_SET(-1.621573E38F);
        p108.lat_SET(3.40267E38F);
        p108.lon_SET(1.82664E38F);
        p108.alt_SET(-2.3649789E38F);
        p108.std_dev_horz_SET(-6.1431976E37F);
        p108.std_dev_vert_SET(-1.0017856E37F);
        p108.vn_SET(-4.5274354E37F);
        p108.ve_SET(3.296102E38F);
        p108.vd_SET(2.458992E38F);
        CommunicationChannel.instance.send(p108); //===============================
        RADIO_STATUS p109 = CommunicationChannel.instance.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)68);
        p109.remrssi_SET((char)56);
        p109.txbuf_SET((char)9);
        p109.noise_SET((char)29);
        p109.remnoise_SET((char)243);
        p109.rxerrors_SET((char)47691);
        p109.fixed__SET((char)21610);
        CommunicationChannel.instance.send(p109); //===============================
        FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.instance.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)240);
        p110.target_system_SET((char)141);
        p110.target_component_SET((char)211);
        p110.payload_SET(new char[251], 0);
        CommunicationChannel.instance.send(p110); //===============================
        TIMESYNC p111 = CommunicationChannel.instance.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(-8603494360707937299L);
        p111.ts1_SET(4275490596166455132L);
        CommunicationChannel.instance.send(p111); //===============================
        CAMERA_TRIGGER p112 = CommunicationChannel.instance.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(7949647610269179451L);
        p112.seq_SET(1267495663L);
        CommunicationChannel.instance.send(p112); //===============================
        HIL_GPS p113 = CommunicationChannel.instance.new_HIL_GPS();
        PH.setPack(p113);
        p113.time_usec_SET(165669667302715610L);
        p113.fix_type_SET((char)9);
        p113.lat_SET(-421073049);
        p113.lon_SET(-49710144);
        p113.alt_SET(375317088);
        p113.eph_SET((char)6548);
        p113.epv_SET((char)32528);
        p113.vel_SET((char)28291);
        p113.vn_SET((short) -28667);
        p113.ve_SET((short)16381);
        p113.vd_SET((short)17741);
        p113.cog_SET((char)2074);
        p113.satellites_visible_SET((char)185);
        CommunicationChannel.instance.send(p113); //===============================
        HIL_OPTICAL_FLOW p114 = CommunicationChannel.instance.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.time_usec_SET(5816025366028181202L);
        p114.sensor_id_SET((char)181);
        p114.integration_time_us_SET(1226782397L);
        p114.integrated_x_SET(-3.3178704E38F);
        p114.integrated_y_SET(-4.1943298E37F);
        p114.integrated_xgyro_SET(-2.1607595E38F);
        p114.integrated_ygyro_SET(2.595262E38F);
        p114.integrated_zgyro_SET(6.2336815E37F);
        p114.temperature_SET((short) -22862);
        p114.quality_SET((char)97);
        p114.time_delta_distance_us_SET(292058764L);
        p114.distance_SET(2.1988452E38F);
        CommunicationChannel.instance.send(p114); //===============================
        HIL_STATE_QUATERNION p115 = CommunicationChannel.instance.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.time_usec_SET(565773686898612291L);
        p115.attitude_quaternion_SET(new float[4], 0);
        p115.rollspeed_SET(-4.486875E37F);
        p115.pitchspeed_SET(2.4320558E38F);
        p115.yawspeed_SET(-1.5068043E37F);
        p115.lat_SET(-1442066501);
        p115.lon_SET(-562309439);
        p115.alt_SET(-1530045712);
        p115.vx_SET((short)16282);
        p115.vy_SET((short) -21659);
        p115.vz_SET((short)30330);
        p115.ind_airspeed_SET((char)39241);
        p115.true_airspeed_SET((char)55673);
        p115.xacc_SET((short)27185);
        p115.yacc_SET((short)5406);
        p115.zacc_SET((short)1975);
        CommunicationChannel.instance.send(p115); //===============================
        SCALED_IMU2 p116 = CommunicationChannel.instance.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.time_boot_ms_SET(1165841662L);
        p116.xacc_SET((short)2206);
        p116.yacc_SET((short) -13096);
        p116.zacc_SET((short) -24234);
        p116.xgyro_SET((short) -18043);
        p116.ygyro_SET((short)22305);
        p116.zgyro_SET((short) -13008);
        p116.xmag_SET((short)29845);
        p116.ymag_SET((short) -4932);
        p116.zmag_SET((short)17069);
        CommunicationChannel.instance.send(p116); //===============================
        LOG_REQUEST_LIST p117 = CommunicationChannel.instance.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)58);
        p117.target_component_SET((char)157);
        p117.start_SET((char)46145);
        p117.end_SET((char)5349);
        CommunicationChannel.instance.send(p117); //===============================
        LOG_ENTRY p118 = CommunicationChannel.instance.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)61480);
        p118.num_logs_SET((char)19158);
        p118.last_log_num_SET((char)17266);
        p118.time_utc_SET(508627456L);
        p118.size_SET(978953062L);
        CommunicationChannel.instance.send(p118); //===============================
        LOG_REQUEST_DATA p119 = CommunicationChannel.instance.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)255);
        p119.target_component_SET((char)73);
        p119.id_SET((char)48922);
        p119.ofs_SET(3076544377L);
        p119.count_SET(1530447059L);
        CommunicationChannel.instance.send(p119); //===============================
        LOG_DATA p120 = CommunicationChannel.instance.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)22054);
        p120.ofs_SET(118494243L);
        p120.count_SET((char)0);
        p120.data__SET(new char[90], 0);
        CommunicationChannel.instance.send(p120); //===============================
        LOG_ERASE p121 = CommunicationChannel.instance.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)152);
        p121.target_component_SET((char)94);
        CommunicationChannel.instance.send(p121); //===============================
        LOG_REQUEST_END p122 = CommunicationChannel.instance.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)28);
        p122.target_component_SET((char)34);
        CommunicationChannel.instance.send(p122); //===============================
        GPS_INJECT_DATA p123 = CommunicationChannel.instance.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)115);
        p123.target_component_SET((char)18);
        p123.len_SET((char)172);
        p123.data__SET(new char[110], 0);
        CommunicationChannel.instance.send(p123); //===============================
        GPS2_RAW p124 = CommunicationChannel.instance.new_GPS2_RAW();
        PH.setPack(p124);
        p124.time_usec_SET(3329881863286117794L);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
        p124.lat_SET(-859414387);
        p124.lon_SET(858398919);
        p124.alt_SET(295332655);
        p124.eph_SET((char)41871);
        p124.epv_SET((char)55057);
        p124.vel_SET((char)11070);
        p124.cog_SET((char)57574);
        p124.satellites_visible_SET((char)91);
        p124.dgps_numch_SET((char)26);
        p124.dgps_age_SET(2950620694L);
        CommunicationChannel.instance.send(p124); //===============================
        POWER_STATUS p125 = CommunicationChannel.instance.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)39475);
        p125.Vservo_SET((char)5160);
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                        MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID));
        CommunicationChannel.instance.send(p125); //===============================
        SERIAL_CONTROL p126 = CommunicationChannel.instance.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL);
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_RESPOND |
                        SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY));
        p126.timeout_SET((char)37804);
        p126.baudrate_SET(2738216314L);
        p126.count_SET((char)148);
        p126.data__SET(new char[70], 0);
        CommunicationChannel.instance.send(p126); //===============================
        GPS_RTK p127 = CommunicationChannel.instance.new_GPS_RTK();
        PH.setPack(p127);
        p127.time_last_baseline_ms_SET(2738339095L);
        p127.rtk_receiver_id_SET((char)124);
        p127.wn_SET((char)17660);
        p127.tow_SET(3480461123L);
        p127.rtk_health_SET((char)27);
        p127.rtk_rate_SET((char)105);
        p127.nsats_SET((char)214);
        p127.baseline_coords_type_SET((char)162);
        p127.baseline_a_mm_SET(906047416);
        p127.baseline_b_mm_SET(853449358);
        p127.baseline_c_mm_SET(-1106589909);
        p127.accuracy_SET(818682483L);
        p127.iar_num_hypotheses_SET(-1690896807);
        CommunicationChannel.instance.send(p127); //===============================
        GPS2_RTK p128 = CommunicationChannel.instance.new_GPS2_RTK();
        PH.setPack(p128);
        p128.time_last_baseline_ms_SET(2338597903L);
        p128.rtk_receiver_id_SET((char)26);
        p128.wn_SET((char)65456);
        p128.tow_SET(5803802L);
        p128.rtk_health_SET((char)56);
        p128.rtk_rate_SET((char)4);
        p128.nsats_SET((char)169);
        p128.baseline_coords_type_SET((char)60);
        p128.baseline_a_mm_SET(759399274);
        p128.baseline_b_mm_SET(1248795867);
        p128.baseline_c_mm_SET(693060760);
        p128.accuracy_SET(607546267L);
        p128.iar_num_hypotheses_SET(759387284);
        CommunicationChannel.instance.send(p128); //===============================
        SCALED_IMU3 p129 = CommunicationChannel.instance.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.time_boot_ms_SET(2016848129L);
        p129.xacc_SET((short) -24320);
        p129.yacc_SET((short) -17564);
        p129.zacc_SET((short)23057);
        p129.xgyro_SET((short) -19506);
        p129.ygyro_SET((short)10924);
        p129.zgyro_SET((short) -19330);
        p129.xmag_SET((short)22003);
        p129.ymag_SET((short) -25331);
        p129.zmag_SET((short) -24529);
        CommunicationChannel.instance.send(p129); //===============================
        DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.instance.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.type_SET((char)200);
        p130.size_SET(435902190L);
        p130.width_SET((char)12752);
        p130.height_SET((char)17808);
        p130.packets_SET((char)56107);
        p130.payload_SET((char)53);
        p130.jpg_quality_SET((char)145);
        CommunicationChannel.instance.send(p130); //===============================
        ENCAPSULATED_DATA p131 = CommunicationChannel.instance.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)28278);
        p131.data__SET(new char[253], 0);
        CommunicationChannel.instance.send(p131); //===============================
        DISTANCE_SENSOR p132 = CommunicationChannel.instance.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.time_boot_ms_SET(847229595L);
        p132.min_distance_SET((char)284);
        p132.max_distance_SET((char)16721);
        p132.current_distance_SET((char)15011);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
        p132.id_SET((char)239);
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_135);
        p132.covariance_SET((char)123);
        CommunicationChannel.instance.send(p132); //===============================
        TERRAIN_REQUEST p133 = CommunicationChannel.instance.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(41111379);
        p133.lon_SET(-1007331962);
        p133.grid_spacing_SET((char)47892);
        p133.mask_SET(2576776353210044503L);
        CommunicationChannel.instance.send(p133); //===============================
        TERRAIN_DATA p134 = CommunicationChannel.instance.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lat_SET(-562261697);
        p134.lon_SET(-1859888905);
        p134.grid_spacing_SET((char)21675);
        p134.gridbit_SET((char)148);
        p134.data__SET(new short[16], 0);
        CommunicationChannel.instance.send(p134); //===============================
        TERRAIN_CHECK p135 = CommunicationChannel.instance.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(-1620999432);
        p135.lon_SET(-2135732367);
        CommunicationChannel.instance.send(p135); //===============================
        TERRAIN_REPORT p136 = CommunicationChannel.instance.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lat_SET(-320336492);
        p136.lon_SET(-666904045);
        p136.spacing_SET((char)15488);
        p136.terrain_height_SET(-1.57651E37F);
        p136.current_height_SET(-9.11206E37F);
        p136.pending_SET((char)47935);
        p136.loaded_SET((char)62365);
        CommunicationChannel.instance.send(p136); //===============================
        SCALED_PRESSURE2 p137 = CommunicationChannel.instance.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(3637857123L);
        p137.press_abs_SET(-1.2153614E37F);
        p137.press_diff_SET(1.4507469E38F);
        p137.temperature_SET((short)7973);
        CommunicationChannel.instance.send(p137); //===============================
        ATT_POS_MOCAP p138 = CommunicationChannel.instance.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(7149463300832739324L);
        p138.q_SET(new float[4], 0);
        p138.x_SET(-7.906098E36F);
        p138.y_SET(-9.71097E37F);
        p138.z_SET(1.1541682E38F);
        CommunicationChannel.instance.send(p138); //===============================
        SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.instance.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(5228054798446114586L);
        p139.group_mlx_SET((char)200);
        p139.target_system_SET((char)109);
        p139.target_component_SET((char)165);
        p139.controls_SET(new float[8], 0);
        CommunicationChannel.instance.send(p139); //===============================
        ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.instance.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(8373405002363093727L);
        p140.group_mlx_SET((char)36);
        p140.controls_SET(new float[8], 0);
        CommunicationChannel.instance.send(p140); //===============================
        ALTITUDE p141 = CommunicationChannel.instance.new_ALTITUDE();
        PH.setPack(p141);
        p141.time_usec_SET(6064301243698324558L);
        p141.altitude_monotonic_SET(-3.3350222E38F);
        p141.altitude_amsl_SET(1.1966174E38F);
        p141.altitude_local_SET(-4.316675E37F);
        p141.altitude_relative_SET(-2.2157798E38F);
        p141.altitude_terrain_SET(-1.8644045E38F);
        p141.bottom_clearance_SET(3.273657E38F);
        CommunicationChannel.instance.send(p141); //===============================
        RESOURCE_REQUEST p142 = CommunicationChannel.instance.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.request_id_SET((char)26);
        p142.uri_type_SET((char)244);
        p142.uri_SET(new char[120], 0);
        p142.transfer_type_SET((char)138);
        p142.storage_SET(new char[120], 0);
        CommunicationChannel.instance.send(p142); //===============================
        SCALED_PRESSURE3 p143 = CommunicationChannel.instance.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(877171423L);
        p143.press_abs_SET(2.8208262E38F);
        p143.press_diff_SET(-2.4796406E38F);
        p143.temperature_SET((short)16009);
        CommunicationChannel.instance.send(p143); //===============================
        FOLLOW_TARGET p144 = CommunicationChannel.instance.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.timestamp_SET(849825195593839238L);
        p144.est_capabilities_SET((char)116);
        p144.lat_SET(494408346);
        p144.lon_SET(-1302661777);
        p144.alt_SET(-3.0136188E38F);
        p144.vel_SET(new float[3], 0);
        p144.acc_SET(new float[3], 0);
        p144.attitude_q_SET(new float[4], 0);
        p144.rates_SET(new float[3], 0);
        p144.position_cov_SET(new float[3], 0);
        p144.custom_state_SET(5186755072624027250L);
        CommunicationChannel.instance.send(p144); //===============================
        CONTROL_SYSTEM_STATE p146 = CommunicationChannel.instance.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.time_usec_SET(4543237251879923203L);
        p146.x_acc_SET(1.1128808E38F);
        p146.y_acc_SET(-2.1790782E38F);
        p146.z_acc_SET(-1.3453898E38F);
        p146.x_vel_SET(-2.7634381E38F);
        p146.y_vel_SET(-2.7735319E38F);
        p146.z_vel_SET(5.1721615E37F);
        p146.x_pos_SET(-3.3542755E37F);
        p146.y_pos_SET(2.1443036E38F);
        p146.z_pos_SET(2.7547248E38F);
        p146.airspeed_SET(-5.5243605E37F);
        p146.vel_variance_SET(new float[3], 0);
        p146.pos_variance_SET(new float[3], 0);
        p146.q_SET(new float[4], 0);
        p146.roll_rate_SET(1.861622E38F);
        p146.pitch_rate_SET(-1.905516E38F);
        p146.yaw_rate_SET(-1.865375E38F);
        CommunicationChannel.instance.send(p146); //===============================
        BATTERY_STATUS p147 = CommunicationChannel.instance.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.id_SET((char)19);
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
        p147.temperature_SET((short) -19268);
        p147.voltages_SET(new char[10], 0);
        p147.current_battery_SET((short)21547);
        p147.current_consumed_SET(-477705634);
        p147.energy_consumed_SET(-515763980);
        p147.battery_remaining_SET((byte)29);
        CommunicationChannel.instance.send(p147); //===============================
        AUTOPILOT_VERSION p148 = CommunicationChannel.instance.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT));
        p148.flight_sw_version_SET(1906124265L);
        p148.middleware_sw_version_SET(763201815L);
        p148.os_sw_version_SET(1662194975L);
        p148.board_version_SET(2874277328L);
        p148.flight_custom_version_SET(new char[8], 0);
        p148.middleware_custom_version_SET(new char[8], 0);
        p148.os_custom_version_SET(new char[8], 0);
        p148.vendor_id_SET((char)55954);
        p148.product_id_SET((char)17834);
        p148.uid_SET(5683567732228358265L);
        p148.uid2_SET(new char[18], 0, PH);
        CommunicationChannel.instance.send(p148); //===============================
        LANDING_TARGET p149 = CommunicationChannel.instance.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.time_usec_SET(6838860093327387204L);
        p149.target_num_SET((char)117);
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
        p149.angle_x_SET(-1.005959E38F);
        p149.angle_y_SET(-1.9818178E38F);
        p149.distance_SET(-2.4073364E38F);
        p149.size_x_SET(1.4556797E38F);
        p149.size_y_SET(1.3372339E38F);
        p149.x_SET(1.7029111E38F, PH);
        p149.y_SET(-4.5906455E37F, PH);
        p149.z_SET(-2.3765946E38F, PH);
        p149.q_SET(new float[4], 0, PH);
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
        p149.position_valid_SET((char)177, PH);
        CommunicationChannel.instance.send(p149); //===============================
        SENS_POWER p201 = CommunicationChannel.instance.new_SENS_POWER();
        PH.setPack(p201);
        p201.adc121_vspb_volt_SET(-9.392719E37F);
        p201.adc121_cspb_amp_SET(-3.1583152E38F);
        p201.adc121_cs1_amp_SET(-2.2942172E38F);
        p201.adc121_cs2_amp_SET(-3.1594638E38F);
        CommunicationChannel.instance.send(p201); //===============================
        SENS_MPPT p202 = CommunicationChannel.instance.new_SENS_MPPT();
        PH.setPack(p202);
        p202.mppt_timestamp_SET(285966515252055149L);
        p202.mppt1_volt_SET(-7.2659167E37F);
        p202.mppt1_amp_SET(-1.3261746E38F);
        p202.mppt1_pwm_SET((char)40268);
        p202.mppt1_status_SET((char)108);
        p202.mppt2_volt_SET(-9.036085E37F);
        p202.mppt2_amp_SET(3.067764E38F);
        p202.mppt2_pwm_SET((char)4969);
        p202.mppt2_status_SET((char)20);
        p202.mppt3_volt_SET(-1.1597582E38F);
        p202.mppt3_amp_SET(1.8450492E38F);
        p202.mppt3_pwm_SET((char)55493);
        p202.mppt3_status_SET((char)216);
        CommunicationChannel.instance.send(p202); //===============================
        ASLCTRL_DATA p203 = CommunicationChannel.instance.new_ASLCTRL_DATA();
        PH.setPack(p203);
        p203.timestamp_SET(721740766278431795L);
        p203.aslctrl_mode_SET((char)81);
        p203.h_SET(-2.5636203E38F);
        p203.hRef_SET(-3.0302336E38F);
        p203.hRef_t_SET(1.6873743E38F);
        p203.PitchAngle_SET(3.0986721E38F);
        p203.PitchAngleRef_SET(2.3024508E38F);
        p203.q_SET(1.411053E37F);
        p203.qRef_SET(-1.0346933E38F);
        p203.uElev_SET(-1.9487445E38F);
        p203.uThrot_SET(-1.6329929E38F);
        p203.uThrot2_SET(-3.6633574E37F);
        p203.nZ_SET(-3.0203678E38F);
        p203.AirspeedRef_SET(-1.5410964E38F);
        p203.SpoilersEngaged_SET((char)149);
        p203.YawAngle_SET(1.2329856E38F);
        p203.YawAngleRef_SET(-1.25608876E36F);
        p203.RollAngle_SET(3.0108655E38F);
        p203.RollAngleRef_SET(-1.9067615E38F);
        p203.p_SET(-3.3435664E38F);
        p203.pRef_SET(-2.33573E38F);
        p203.r_SET(2.8504497E38F);
        p203.rRef_SET(2.4610086E38F);
        p203.uAil_SET(6.5281395E37F);
        p203.uRud_SET(1.4405046E38F);
        CommunicationChannel.instance.send(p203); //===============================
        ASLCTRL_DEBUG p204 = CommunicationChannel.instance.new_ASLCTRL_DEBUG();
        PH.setPack(p204);
        p204.i32_1_SET(2613962605L);
        p204.i8_1_SET((char)211);
        p204.i8_2_SET((char)102);
        p204.f_1_SET(-1.7472444E38F);
        p204.f_2_SET(2.0649743E37F);
        p204.f_3_SET(-2.0684368E38F);
        p204.f_4_SET(-1.1479392E38F);
        p204.f_5_SET(5.0368986E37F);
        p204.f_6_SET(-1.2779227E38F);
        p204.f_7_SET(8.2263887E37F);
        p204.f_8_SET(2.4539831E38F);
        CommunicationChannel.instance.send(p204); //===============================
        ASLUAV_STATUS p205 = CommunicationChannel.instance.new_ASLUAV_STATUS();
        PH.setPack(p205);
        p205.LED_status_SET((char)47);
        p205.SATCOM_status_SET((char)57);
        p205.Servo_status_SET(new char[8], 0);
        p205.Motor_rpm_SET(2.6637122E38F);
        CommunicationChannel.instance.send(p205); //===============================
        EKF_EXT p206 = CommunicationChannel.instance.new_EKF_EXT();
        PH.setPack(p206);
        p206.timestamp_SET(3630311323414869364L);
        p206.Windspeed_SET(-8.0561457E37F);
        p206.WindDir_SET(-2.9914956E38F);
        p206.WindZ_SET(2.7013628E38F);
        p206.Airspeed_SET(2.566965E38F);
        p206.beta_SET(-3.9925106E36F);
        p206.alpha_SET(4.927029E37F);
        CommunicationChannel.instance.send(p206); //===============================
        ASL_OBCTRL p207 = CommunicationChannel.instance.new_ASL_OBCTRL();
        PH.setPack(p207);
        p207.timestamp_SET(5614095297556502928L);
        p207.uElev_SET(2.3941665E38F);
        p207.uThrot_SET(-3.3935968E38F);
        p207.uThrot2_SET(2.23761E38F);
        p207.uAilL_SET(1.6100936E38F);
        p207.uAilR_SET(-4.2589754E37F);
        p207.uRud_SET(-3.1551884E38F);
        p207.obctrl_status_SET((char)98);
        CommunicationChannel.instance.send(p207); //===============================
        SENS_ATMOS p208 = CommunicationChannel.instance.new_SENS_ATMOS();
        PH.setPack(p208);
        p208.TempAmbient_SET(-2.75226E38F);
        p208.Humidity_SET(-2.3306947E38F);
        CommunicationChannel.instance.send(p208); //===============================
        SENS_BATMON p209 = CommunicationChannel.instance.new_SENS_BATMON();
        PH.setPack(p209);
        p209.temperature_SET(5.137105E37F);
        p209.voltage_SET((char)24463);
        p209.current_SET((short) -31623);
        p209.SoC_SET((char)167);
        p209.batterystatus_SET((char)53603);
        p209.serialnumber_SET((char)25615);
        p209.hostfetcontrol_SET((char)47837);
        p209.cellvoltage1_SET((char)62237);
        p209.cellvoltage2_SET((char)64850);
        p209.cellvoltage3_SET((char)17910);
        p209.cellvoltage4_SET((char)50437);
        p209.cellvoltage5_SET((char)8434);
        p209.cellvoltage6_SET((char)12213);
        CommunicationChannel.instance.send(p209); //===============================
        FW_SOARING_DATA p210 = CommunicationChannel.instance.new_FW_SOARING_DATA();
        PH.setPack(p210);
        p210.timestamp_SET(5057521756023728354L);
        p210.timestampModeChanged_SET(9197520290352658533L);
        p210.xW_SET(1.5231806E38F);
        p210.xR_SET(-2.9726176E37F);
        p210.xLat_SET(-6.563429E37F);
        p210.xLon_SET(5.760957E37F);
        p210.VarW_SET(1.9018856E38F);
        p210.VarR_SET(1.9819096E38F);
        p210.VarLat_SET(-3.982494E37F);
        p210.VarLon_SET(-7.9793763E37F);
        p210.LoiterRadius_SET(-2.7081838E38F);
        p210.LoiterDirection_SET(8.097126E35F);
        p210.DistToSoarPoint_SET(1.4720412E36F);
        p210.vSinkExp_SET(-7.102223E37F);
        p210.z1_LocalUpdraftSpeed_SET(-3.363717E38F);
        p210.z2_DeltaRoll_SET(-2.8644969E38F);
        p210.z1_exp_SET(-8.770127E37F);
        p210.z2_exp_SET(3.7261887E37F);
        p210.ThermalGSNorth_SET(3.2706171E37F);
        p210.ThermalGSEast_SET(2.0141617E38F);
        p210.TSE_dot_SET(1.003095E38F);
        p210.DebugVar1_SET(-1.4586223E38F);
        p210.DebugVar2_SET(-2.9814396E38F);
        p210.ControlMode_SET((char)20);
        p210.valid_SET((char)252);
        CommunicationChannel.instance.send(p210); //===============================
        SENSORPOD_STATUS p211 = CommunicationChannel.instance.new_SENSORPOD_STATUS();
        PH.setPack(p211);
        p211.timestamp_SET(2111929808917609010L);
        p211.visensor_rate_1_SET((char)2);
        p211.visensor_rate_2_SET((char)15);
        p211.visensor_rate_3_SET((char)245);
        p211.visensor_rate_4_SET((char)73);
        p211.recording_nodes_count_SET((char)151);
        p211.cpu_temp_SET((char)146);
        p211.free_space_SET((char)11829);
        CommunicationChannel.instance.send(p211); //===============================
        SENS_POWER_BOARD p212 = CommunicationChannel.instance.new_SENS_POWER_BOARD();
        PH.setPack(p212);
        p212.timestamp_SET(9158462801807364080L);
        p212.pwr_brd_status_SET((char)3);
        p212.pwr_brd_led_status_SET((char)7);
        p212.pwr_brd_system_volt_SET(-7.6787215E37F);
        p212.pwr_brd_servo_volt_SET(-2.4515873E37F);
        p212.pwr_brd_mot_l_amp_SET(-1.04085327E37F);
        p212.pwr_brd_mot_r_amp_SET(3.1275306E37F);
        p212.pwr_brd_servo_1_amp_SET(2.4562984E38F);
        p212.pwr_brd_servo_2_amp_SET(2.4575924E38F);
        p212.pwr_brd_servo_3_amp_SET(-6.245017E37F);
        p212.pwr_brd_servo_4_amp_SET(-2.2305515E38F);
        p212.pwr_brd_aux_amp_SET(-5.074417E37F);
        CommunicationChannel.instance.send(p212); //===============================
        ESTIMATOR_STATUS p230 = CommunicationChannel.instance.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.time_usec_SET(4471970500393313088L);
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_ABS));
        p230.vel_ratio_SET(2.816689E37F);
        p230.pos_horiz_ratio_SET(1.3131387E37F);
        p230.pos_vert_ratio_SET(-2.0391245E38F);
        p230.mag_ratio_SET(-3.5807906E36F);
        p230.hagl_ratio_SET(2.8466504E38F);
        p230.tas_ratio_SET(-3.3437446E37F);
        p230.pos_horiz_accuracy_SET(1.1671746E38F);
        p230.pos_vert_accuracy_SET(-2.9400411E38F);
        CommunicationChannel.instance.send(p230); //===============================
        WIND_COV p231 = CommunicationChannel.instance.new_WIND_COV();
        PH.setPack(p231);
        p231.time_usec_SET(569429177960630374L);
        p231.wind_x_SET(2.1683853E38F);
        p231.wind_y_SET(-2.6531465E38F);
        p231.wind_z_SET(1.0108287E36F);
        p231.var_horiz_SET(-1.2253974E38F);
        p231.var_vert_SET(1.756158E38F);
        p231.wind_alt_SET(3.602468E37F);
        p231.horiz_accuracy_SET(-6.0495183E36F);
        p231.vert_accuracy_SET(2.5397282E38F);
        CommunicationChannel.instance.send(p231); //===============================
        GPS_INPUT p232 = CommunicationChannel.instance.new_GPS_INPUT();
        PH.setPack(p232);
        p232.time_usec_SET(7500573554202046544L);
        p232.gps_id_SET((char)166);
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ));
        p232.time_week_ms_SET(129806698L);
        p232.time_week_SET((char)20597);
        p232.fix_type_SET((char)207);
        p232.lat_SET(2142051402);
        p232.lon_SET(-2108792641);
        p232.alt_SET(-1.2020233E38F);
        p232.hdop_SET(-2.8655931E38F);
        p232.vdop_SET(2.4498384E38F);
        p232.vn_SET(-3.2784314E38F);
        p232.ve_SET(-7.447038E37F);
        p232.vd_SET(-4.226657E37F);
        p232.speed_accuracy_SET(1.2006797E38F);
        p232.horiz_accuracy_SET(3.3188411E38F);
        p232.vert_accuracy_SET(2.1946206E38F);
        p232.satellites_visible_SET((char)96);
        CommunicationChannel.instance.send(p232); //===============================
        GPS_RTCM_DATA p233 = CommunicationChannel.instance.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)34);
        p233.len_SET((char)62);
        p233.data__SET(new char[180], 0);
        CommunicationChannel.instance.send(p233); //===============================
        HIGH_LATENCY p234 = CommunicationChannel.instance.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                            MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
        p234.custom_mode_SET(3255339054L);
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
        p234.roll_SET((short)8148);
        p234.pitch_SET((short) -2723);
        p234.heading_SET((char)34563);
        p234.throttle_SET((byte)82);
        p234.heading_sp_SET((short)18129);
        p234.latitude_SET(-1360475864);
        p234.longitude_SET(-1505530755);
        p234.altitude_amsl_SET((short)10876);
        p234.altitude_sp_SET((short) -31133);
        p234.airspeed_SET((char)4);
        p234.airspeed_sp_SET((char)195);
        p234.groundspeed_SET((char)121);
        p234.climb_rate_SET((byte)54);
        p234.gps_nsat_SET((char)114);
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX);
        p234.battery_remaining_SET((char)142);
        p234.temperature_SET((byte) - 73);
        p234.temperature_air_SET((byte) - 78);
        p234.failsafe_SET((char)182);
        p234.wp_num_SET((char)106);
        p234.wp_distance_SET((char)55832);
        CommunicationChannel.instance.send(p234); //===============================
        VIBRATION p241 = CommunicationChannel.instance.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(1263057684956593178L);
        p241.vibration_x_SET(2.4359075E38F);
        p241.vibration_y_SET(2.0589627E38F);
        p241.vibration_z_SET(2.0667826E38F);
        p241.clipping_0_SET(1479912219L);
        p241.clipping_1_SET(4164245249L);
        p241.clipping_2_SET(3450311287L);
        CommunicationChannel.instance.send(p241); //===============================
        HOME_POSITION p242 = CommunicationChannel.instance.new_HOME_POSITION();
        PH.setPack(p242);
        p242.latitude_SET(1831015651);
        p242.longitude_SET(-169310877);
        p242.altitude_SET(-2106633053);
        p242.x_SET(-3.1645557E36F);
        p242.y_SET(2.0717222E38F);
        p242.z_SET(-2.2030976E38F);
        p242.q_SET(new float[4], 0);
        p242.approach_x_SET(3.2209016E38F);
        p242.approach_y_SET(-1.4848597E38F);
        p242.approach_z_SET(-8.777803E37F);
        p242.time_usec_SET(3907273803725014843L, PH);
        CommunicationChannel.instance.send(p242); //===============================
        SET_HOME_POSITION p243 = CommunicationChannel.instance.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.target_system_SET((char)232);
        p243.latitude_SET(-43491072);
        p243.longitude_SET(867699738);
        p243.altitude_SET(1310303933);
        p243.x_SET(3.3128552E38F);
        p243.y_SET(2.2042815E38F);
        p243.z_SET(9.832437E37F);
        p243.q_SET(new float[4], 0);
        p243.approach_x_SET(-5.752101E36F);
        p243.approach_y_SET(2.3531688E38F);
        p243.approach_z_SET(2.3702192E38F);
        p243.time_usec_SET(566497544937150737L, PH);
        CommunicationChannel.instance.send(p243); //===============================
        MESSAGE_INTERVAL p244 = CommunicationChannel.instance.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)23759);
        p244.interval_us_SET(675879233);
        CommunicationChannel.instance.send(p244); //===============================
        EXTENDED_SYS_STATE p245 = CommunicationChannel.instance.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_FW);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
        CommunicationChannel.instance.send(p245); //===============================
        ADSB_VEHICLE p246 = CommunicationChannel.instance.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.ICAO_address_SET(2099062680L);
        p246.lat_SET(-1027922297);
        p246.lon_SET(-1233172137);
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
        p246.altitude_SET(-146566088);
        p246.heading_SET((char)58441);
        p246.hor_velocity_SET((char)8181);
        p246.ver_velocity_SET((short) -5556);
        p246.callsign_SET("DEMO", PH);
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSGINED3);
        p246.tslc_SET((char)179);
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN));
        p246.squawk_SET((char)35763);
        CommunicationChannel.instance.send(p246); //===============================
        COLLISION p247 = CommunicationChannel.instance.new_COLLISION();
        PH.setPack(p247);
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
        p247.id_SET(2591444663L);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL);
        p247.threat_level_SET((MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH |
                               MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE));
        p247.time_to_minimum_delta_SET(-3.0144192E38F);
        p247.altitude_minimum_delta_SET(3.241708E38F);
        p247.horizontal_minimum_delta_SET(7.288083E37F);
        CommunicationChannel.instance.send(p247); //===============================
        V2_EXTENSION p248 = CommunicationChannel.instance.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)150);
        p248.target_system_SET((char)88);
        p248.target_component_SET((char)133);
        p248.message_type_SET((char)35542);
        p248.payload_SET(new char[249], 0);
        CommunicationChannel.instance.send(p248); //===============================
        MEMORY_VECT p249 = CommunicationChannel.instance.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)56617);
        p249.ver_SET((char)132);
        p249.type_SET((char)217);
        p249.value_SET(new byte[32], 0);
        CommunicationChannel.instance.send(p249); //===============================
        DEBUG_VECT p250 = CommunicationChannel.instance.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("DEMO", PH);
        p250.time_usec_SET(6811562872487576221L);
        p250.x_SET(-2.365849E38F);
        p250.y_SET(1.2918593E37F);
        p250.z_SET(-1.0781713E38F);
        CommunicationChannel.instance.send(p250); //===============================
        NAMED_VALUE_FLOAT p251 = CommunicationChannel.instance.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(2424397202L);
        p251.name_SET("DEMO", PH);
        p251.value_SET(8.876756E37F);
        CommunicationChannel.instance.send(p251); //===============================
        NAMED_VALUE_INT p252 = CommunicationChannel.instance.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(3471919779L);
        p252.name_SET("DEMO", PH);
        p252.value_SET(709050123);
        CommunicationChannel.instance.send(p252); //===============================
        STATUSTEXT p253 = CommunicationChannel.instance.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_ERROR);
        p253.text_SET("DEMO", PH);
        CommunicationChannel.instance.send(p253); //===============================
        DEBUG p254 = CommunicationChannel.instance.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(2370905328L);
        p254.ind_SET((char)86);
        p254.value_SET(-2.3189165E38F);
        CommunicationChannel.instance.send(p254); //===============================
        SETUP_SIGNING p256 = CommunicationChannel.instance.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)231);
        p256.target_component_SET((char)102);
        p256.secret_key_SET(new char[32], 0);
        p256.initial_timestamp_SET(1947728120845190666L);
        CommunicationChannel.instance.send(p256); //===============================
        BUTTON_CHANGE p257 = CommunicationChannel.instance.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(1895281741L);
        p257.last_change_ms_SET(660122751L);
        p257.state_SET((char)108);
        CommunicationChannel.instance.send(p257); //===============================
        PLAY_TUNE p258 = CommunicationChannel.instance.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)195);
        p258.target_component_SET((char)146);
        p258.tune_SET("DEMO", PH);
        CommunicationChannel.instance.send(p258); //===============================
        CAMERA_INFORMATION p259 = CommunicationChannel.instance.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.time_boot_ms_SET(389281371L);
        p259.vendor_name_SET(new char[32], 0);
        p259.model_name_SET(new char[32], 0);
        p259.firmware_version_SET(3364085633L);
        p259.focal_length_SET(2.593824E38F);
        p259.sensor_size_h_SET(-8.0281453E37F);
        p259.sensor_size_v_SET(-1.0965914E38F);
        p259.resolution_h_SET((char)58286);
        p259.resolution_v_SET((char)64819);
        p259.lens_id_SET((char)195);
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_MODES |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE));
        p259.cam_definition_version_SET((char)48010);
        p259.cam_definition_uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p259); //===============================
        CAMERA_SETTINGS p260 = CommunicationChannel.instance.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(3388966806L);
        p260.mode_id_SET((CAMERA_MODE.CAMERA_MODE_VIDEO));
        CommunicationChannel.instance.send(p260); //===============================
        STORAGE_INFORMATION p261 = CommunicationChannel.instance.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.time_boot_ms_SET(20786425L);
        p261.storage_id_SET((char)151);
        p261.storage_count_SET((char)98);
        p261.status_SET((char)156);
        p261.total_capacity_SET(-2.6593685E38F);
        p261.used_capacity_SET(5.660316E37F);
        p261.available_capacity_SET(1.0759928E38F);
        p261.read_speed_SET(-6.4510374E37F);
        p261.write_speed_SET(-9.880876E37F);
        CommunicationChannel.instance.send(p261); //===============================
        CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.instance.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.time_boot_ms_SET(1461496391L);
        p262.image_status_SET((char)198);
        p262.video_status_SET((char)181);
        p262.image_interval_SET(5.3001207E37F);
        p262.recording_time_ms_SET(3802610546L);
        p262.available_capacity_SET(-2.894071E38F);
        CommunicationChannel.instance.send(p262); //===============================
        CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.instance.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.time_boot_ms_SET(2296446992L);
        p263.time_utc_SET(8486995137643174140L);
        p263.camera_id_SET((char)166);
        p263.lat_SET(-1782884817);
        p263.lon_SET(1508084612);
        p263.alt_SET(-790471820);
        p263.relative_alt_SET(272516953);
        p263.q_SET(new float[4], 0);
        p263.image_index_SET(-269886057);
        p263.capture_result_SET((byte) - 115);
        p263.file_url_SET("DEMO", PH);
        CommunicationChannel.instance.send(p263); //===============================
        FLIGHT_INFORMATION p264 = CommunicationChannel.instance.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.time_boot_ms_SET(980366518L);
        p264.arming_time_utc_SET(2176579890421139009L);
        p264.takeoff_time_utc_SET(8180480394363927613L);
        p264.flight_uuid_SET(3089640916494592437L);
        CommunicationChannel.instance.send(p264); //===============================
        MOUNT_ORIENTATION p265 = CommunicationChannel.instance.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(1773145L);
        p265.roll_SET(7.9757625E37F);
        p265.pitch_SET(-1.1220753E38F);
        p265.yaw_SET(9.774324E37F);
        CommunicationChannel.instance.send(p265); //===============================
        LOGGING_DATA p266 = CommunicationChannel.instance.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_system_SET((char)194);
        p266.target_component_SET((char)254);
        p266.sequence_SET((char)19568);
        p266.length_SET((char)180);
        p266.first_message_offset_SET((char)184);
        p266.data__SET(new char[249], 0);
        CommunicationChannel.instance.send(p266); //===============================
        LOGGING_DATA_ACKED p267 = CommunicationChannel.instance.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_system_SET((char)129);
        p267.target_component_SET((char)138);
        p267.sequence_SET((char)28891);
        p267.length_SET((char)145);
        p267.first_message_offset_SET((char)220);
        p267.data__SET(new char[249], 0);
        CommunicationChannel.instance.send(p267); //===============================
        LOGGING_ACK p268 = CommunicationChannel.instance.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)251);
        p268.target_component_SET((char)0);
        p268.sequence_SET((char)25218);
        CommunicationChannel.instance.send(p268); //===============================
        VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.instance.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.camera_id_SET((char)172);
        p269.status_SET((char)16);
        p269.framerate_SET(2.3148967E38F);
        p269.resolution_h_SET((char)13060);
        p269.resolution_v_SET((char)52479);
        p269.bitrate_SET(3576444744L);
        p269.rotation_SET((char)53400);
        p269.uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p269); //===============================
        SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.instance.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_system_SET((char)168);
        p270.target_component_SET((char)119);
        p270.camera_id_SET((char)233);
        p270.framerate_SET(-3.366749E38F);
        p270.resolution_h_SET((char)33132);
        p270.resolution_v_SET((char)1270);
        p270.bitrate_SET(4235427992L);
        p270.rotation_SET((char)43585);
        p270.uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p270); //===============================
        WIFI_CONFIG_AP p299 = CommunicationChannel.instance.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("DEMO", PH);
        p299.password_SET("DEMO", PH);
        CommunicationChannel.instance.send(p299); //===============================
        PROTOCOL_VERSION p300 = CommunicationChannel.instance.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)59806);
        p300.min_version_SET((char)57905);
        p300.max_version_SET((char)10081);
        p300.spec_version_hash_SET(new char[8], 0);
        p300.library_version_hash_SET(new char[8], 0);
        CommunicationChannel.instance.send(p300); //===============================
        UAVCAN_NODE_STATUS p310 = CommunicationChannel.instance.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.time_usec_SET(1726539440430944777L);
        p310.uptime_sec_SET(1152262224L);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL);
        p310.sub_mode_SET((char)89);
        p310.vendor_specific_status_code_SET((char)57424);
        CommunicationChannel.instance.send(p310); //===============================
        UAVCAN_NODE_INFO p311 = CommunicationChannel.instance.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.time_usec_SET(6377771458545398447L);
        p311.uptime_sec_SET(2306724884L);
        p311.name_SET("DEMO", PH);
        p311.hw_version_major_SET((char)92);
        p311.hw_version_minor_SET((char)169);
        p311.hw_unique_id_SET(new char[16], 0);
        p311.sw_version_major_SET((char)150);
        p311.sw_version_minor_SET((char)241);
        p311.sw_vcs_commit_SET(1697158245L);
        CommunicationChannel.instance.send(p311); //===============================
        PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)98);
        p320.target_component_SET((char)246);
        p320.param_id_SET("DEMO", PH);
        p320.param_index_SET((short) -10604);
        CommunicationChannel.instance.send(p320); //===============================
        PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)159);
        p321.target_component_SET((char)125);
        CommunicationChannel.instance.send(p321); //===============================
        PARAM_EXT_VALUE p322 = CommunicationChannel.instance.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("DEMO", PH);
        p322.param_value_SET("DEMO", PH);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
        p322.param_count_SET((char)64330);
        p322.param_index_SET((char)43617);
        CommunicationChannel.instance.send(p322); //===============================
        PARAM_EXT_SET p323 = CommunicationChannel.instance.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)35);
        p323.target_component_SET((char)49);
        p323.param_id_SET("DEMO", PH);
        p323.param_value_SET("DEMO", PH);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
        CommunicationChannel.instance.send(p323); //===============================
        PARAM_EXT_ACK p324 = CommunicationChannel.instance.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("DEMO", PH);
        p324.param_value_SET("DEMO", PH);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT32);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_IN_PROGRESS);
        CommunicationChannel.instance.send(p324); //===============================
        OBSTACLE_DISTANCE p330 = CommunicationChannel.instance.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.time_usec_SET(3306152457235773940L);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
        p330.distances_SET(new char[72], 0);
        p330.increment_SET((char)228);
        p330.min_distance_SET((char)60505);
        p330.max_distance_SET((char)22786);
        CommunicationChannel.instance.send(p330); //===============================
    }
}
