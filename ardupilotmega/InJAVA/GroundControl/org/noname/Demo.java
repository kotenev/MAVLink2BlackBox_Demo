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
        CommunicationChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            @MAV_PROTOCOL_CAPABILITY int  capabilities = pack.capabilities_GET();
            long  flight_sw_version = pack.flight_sw_version_GET();
            long  middleware_sw_version = pack.middleware_sw_version_GET();
            long  os_sw_version = pack.os_sw_version_GET();
            long  board_version = pack.board_version_GET();
            char[]  flight_custom_version = pack.flight_custom_version_GET();
            char[]  middleware_custom_version = pack.middleware_custom_version_GET();
            char[]  os_custom_version = pack.os_custom_version_GET();
            char  vendor_id = pack.vendor_id_GET();
            char  product_id = pack.product_id_GET();
            long  uid = pack.uid_GET();
            char[]  uid2 = pack.uid2_TRY(ph);
        });
        CommunicationChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  target_num = pack.target_num_GET();
            @MAV_FRAME int  frame = pack.frame_GET();
            float  angle_x = pack.angle_x_GET();
            float  angle_y = pack.angle_y_GET();
            float  distance = pack.distance_GET();
            float  size_x = pack.size_x_GET();
            float  size_y = pack.size_y_GET();
            float  x = pack.x_TRY(ph);
            float  y = pack.y_TRY(ph);
            float  z = pack.z_TRY(ph);
            float[]  q = pack.q_TRY(ph);
            @LANDING_TARGET_TYPE int  type = pack.type_GET();
            char  position_valid = pack.position_valid_TRY(ph);
        });
        CommunicationChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            @ESTIMATOR_STATUS_FLAGS int  flags = pack.flags_GET();
            float  vel_ratio = pack.vel_ratio_GET();
            float  pos_horiz_ratio = pack.pos_horiz_ratio_GET();
            float  pos_vert_ratio = pack.pos_vert_ratio_GET();
            float  mag_ratio = pack.mag_ratio_GET();
            float  hagl_ratio = pack.hagl_ratio_GET();
            float  tas_ratio = pack.tas_ratio_GET();
            float  pos_horiz_accuracy = pack.pos_horiz_accuracy_GET();
            float  pos_vert_accuracy = pack.pos_vert_accuracy_GET();
        });
        CommunicationChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float  wind_x = pack.wind_x_GET();
            float  wind_y = pack.wind_y_GET();
            float  wind_z = pack.wind_z_GET();
            float  var_horiz = pack.var_horiz_GET();
            float  var_vert = pack.var_vert_GET();
            float  wind_alt = pack.wind_alt_GET();
            float  horiz_accuracy = pack.horiz_accuracy_GET();
            float  vert_accuracy = pack.vert_accuracy_GET();
        });
        CommunicationChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            char  gps_id = pack.gps_id_GET();
            @GPS_INPUT_IGNORE_FLAGS int  ignore_flags = pack.ignore_flags_GET();
            long  time_week_ms = pack.time_week_ms_GET();
            char  time_week = pack.time_week_GET();
            char  fix_type = pack.fix_type_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            float  alt = pack.alt_GET();
            float  hdop = pack.hdop_GET();
            float  vdop = pack.vdop_GET();
            float  vn = pack.vn_GET();
            float  ve = pack.ve_GET();
            float  vd = pack.vd_GET();
            float  speed_accuracy = pack.speed_accuracy_GET();
            float  horiz_accuracy = pack.horiz_accuracy_GET();
            float  vert_accuracy = pack.vert_accuracy_GET();
            char  satellites_visible = pack.satellites_visible_GET();
        });
        CommunicationChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            char  flags = pack.flags_GET();
            char  len = pack.len_GET();
            char[]  data_ = pack.data__GET();
        });
        CommunicationChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            @MAV_MODE_FLAG int  base_mode = pack.base_mode_GET();
            long  custom_mode = pack.custom_mode_GET();
            @MAV_LANDED_STATE int  landed_state = pack.landed_state_GET();
            short  roll = pack.roll_GET();
            short  pitch = pack.pitch_GET();
            char  heading = pack.heading_GET();
            byte  throttle = pack.throttle_GET();
            short  heading_sp = pack.heading_sp_GET();
            int  latitude = pack.latitude_GET();
            int  longitude = pack.longitude_GET();
            short  altitude_amsl = pack.altitude_amsl_GET();
            short  altitude_sp = pack.altitude_sp_GET();
            char  airspeed = pack.airspeed_GET();
            char  airspeed_sp = pack.airspeed_sp_GET();
            char  groundspeed = pack.groundspeed_GET();
            byte  climb_rate = pack.climb_rate_GET();
            char  gps_nsat = pack.gps_nsat_GET();
            @GPS_FIX_TYPE int  gps_fix_type = pack.gps_fix_type_GET();
            char  battery_remaining = pack.battery_remaining_GET();
            byte  temperature = pack.temperature_GET();
            byte  temperature_air = pack.temperature_air_GET();
            char  failsafe = pack.failsafe_GET();
            char  wp_num = pack.wp_num_GET();
            char  wp_distance = pack.wp_distance_GET();
        });
        CommunicationChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            long  time_usec = pack.time_usec_GET();
            float  vibration_x = pack.vibration_x_GET();
            float  vibration_y = pack.vibration_y_GET();
            float  vibration_z = pack.vibration_z_GET();
            long  clipping_0 = pack.clipping_0_GET();
            long  clipping_1 = pack.clipping_1_GET();
            long  clipping_2 = pack.clipping_2_GET();
        });
        CommunicationChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            int  latitude = pack.latitude_GET();
            int  longitude = pack.longitude_GET();
            int  altitude = pack.altitude_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float[]  q = pack.q_GET();
            float  approach_x = pack.approach_x_GET();
            float  approach_y = pack.approach_y_GET();
            float  approach_z = pack.approach_z_GET();
            long  time_usec = pack.time_usec_TRY(ph);
        });
        CommunicationChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            int  latitude = pack.latitude_GET();
            int  longitude = pack.longitude_GET();
            int  altitude = pack.altitude_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
            float[]  q = pack.q_GET();
            float  approach_x = pack.approach_x_GET();
            float  approach_y = pack.approach_y_GET();
            float  approach_z = pack.approach_z_GET();
            long  time_usec = pack.time_usec_TRY(ph);
        });
        CommunicationChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            char  message_id = pack.message_id_GET();
            int  interval_us = pack.interval_us_GET();
        });
        CommunicationChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            @MAV_VTOL_STATE int  vtol_state = pack.vtol_state_GET();
            @MAV_LANDED_STATE int  landed_state = pack.landed_state_GET();
        });
        CommunicationChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            long  ICAO_address = pack.ICAO_address_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            @ADSB_ALTITUDE_TYPE int  altitude_type = pack.altitude_type_GET();
            int  altitude = pack.altitude_GET();
            char  heading = pack.heading_GET();
            char  hor_velocity = pack.hor_velocity_GET();
            short  ver_velocity = pack.ver_velocity_GET();
            String callsign = pack.callsign_TRY(ph);
            @ADSB_EMITTER_TYPE int  emitter_type = pack.emitter_type_GET();
            char  tslc = pack.tslc_GET();
            @ADSB_FLAGS int  flags = pack.flags_GET();
            char  squawk = pack.squawk_GET();
        });
        CommunicationChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            @MAV_COLLISION_SRC int  src_ = pack.src__GET();
            long  id = pack.id_GET();
            @MAV_COLLISION_ACTION int  action = pack.action_GET();
            @MAV_COLLISION_THREAT_LEVEL int  threat_level = pack.threat_level_GET();
            float  time_to_minimum_delta = pack.time_to_minimum_delta_GET();
            float  altitude_minimum_delta = pack.altitude_minimum_delta_GET();
            float  horizontal_minimum_delta = pack.horizontal_minimum_delta_GET();
        });
        CommunicationChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            char  target_network = pack.target_network_GET();
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char  message_type = pack.message_type_GET();
            char[]  payload = pack.payload_GET();
        });
        CommunicationChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            char  address = pack.address_GET();
            char  ver = pack.ver_GET();
            char  type = pack.type_GET();
            byte[]  value = pack.value_GET();
        });
        CommunicationChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            String name = pack.name_TRY(ph);
            long  time_usec = pack.time_usec_GET();
            float  x = pack.x_GET();
            float  y = pack.y_GET();
            float  z = pack.z_GET();
        });
        CommunicationChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            String name = pack.name_TRY(ph);
            float  value = pack.value_GET();
        });
        CommunicationChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            String name = pack.name_TRY(ph);
            int  value = pack.value_GET();
        });
        CommunicationChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            @MAV_SEVERITY int  severity = pack.severity_GET();
            String text = pack.text_TRY(ph);
        });
        CommunicationChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  ind = pack.ind_GET();
            float  value = pack.value_GET();
        });
        CommunicationChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            char[]  secret_key = pack.secret_key_GET();
            long  initial_timestamp = pack.initial_timestamp_GET();
        });
        CommunicationChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            long  last_change_ms = pack.last_change_ms_GET();
            char  state = pack.state_GET();
        });
        CommunicationChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            char  target_system = pack.target_system_GET();
            char  target_component = pack.target_component_GET();
            String tune = pack.tune_TRY(ph);
        });
        CommunicationChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char[]  vendor_name = pack.vendor_name_GET();
            char[]  model_name = pack.model_name_GET();
            long  firmware_version = pack.firmware_version_GET();
            float  focal_length = pack.focal_length_GET();
            float  sensor_size_h = pack.sensor_size_h_GET();
            float  sensor_size_v = pack.sensor_size_v_GET();
            char  resolution_h = pack.resolution_h_GET();
            char  resolution_v = pack.resolution_v_GET();
            char  lens_id = pack.lens_id_GET();
            @CAMERA_CAP_FLAGS int  flags = pack.flags_GET();
            char  cam_definition_version = pack.cam_definition_version_GET();
            String cam_definition_uri = pack.cam_definition_uri_TRY(ph);
        });
        CommunicationChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            @CAMERA_MODE int  mode_id = pack.mode_id_GET();
        });
        CommunicationChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  storage_id = pack.storage_id_GET();
            char  storage_count = pack.storage_count_GET();
            char  status = pack.status_GET();
            float  total_capacity = pack.total_capacity_GET();
            float  used_capacity = pack.used_capacity_GET();
            float  available_capacity = pack.available_capacity_GET();
            float  read_speed = pack.read_speed_GET();
            float  write_speed = pack.write_speed_GET();
        });
        CommunicationChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            char  image_status = pack.image_status_GET();
            char  video_status = pack.video_status_GET();
            float  image_interval = pack.image_interval_GET();
            long  recording_time_ms = pack.recording_time_ms_GET();
            float  available_capacity = pack.available_capacity_GET();
        });
        CommunicationChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            long  time_boot_ms = pack.time_boot_ms_GET();
            long  time_utc = pack.time_utc_GET();
            char  camera_id = pack.camera_id_GET();
            int  lat = pack.lat_GET();
            int  lon = pack.lon_GET();
            int  alt = pack.alt_GET();
            int  relative_alt = pack.relative_alt_GET();
            float[]  q = pack.q_GET();
            int  image_index = pack.image_index_GET();
            byte  capture_result = pack.capture_result_GET();
            String file_url = pack.file_url_TRY(ph);
        });
        VISION_POSITION_ESTIMATE p102 = CommunicationChannel.instance.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.usec_SET(9002335461222318826L);
        p102.x_SET(1.4937973E38F);
        p102.y_SET(2.5646407E38F);
        p102.z_SET(-1.1022985E38F);
        p102.roll_SET(1.8950606E38F);
        p102.pitch_SET(-3.101487E38F);
        p102.yaw_SET(-4.3439425E37F);
        CommunicationChannel.instance.send(p102); //===============================
        VISION_SPEED_ESTIMATE p103 = CommunicationChannel.instance.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(6245613673395942243L);
        p103.x_SET(1.7916381E38F);
        p103.y_SET(-1.9336405E37F);
        p103.z_SET(1.9112614E38F);
        CommunicationChannel.instance.send(p103); //===============================
        VICON_POSITION_ESTIMATE p104 = CommunicationChannel.instance.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(4672688534597501346L);
        p104.x_SET(-1.9965955E38F);
        p104.y_SET(9.21156E37F);
        p104.z_SET(-1.0265314E38F);
        p104.roll_SET(-7.23392E37F);
        p104.pitch_SET(-2.3352745E38F);
        p104.yaw_SET(-5.0641506E37F);
        CommunicationChannel.instance.send(p104); //===============================
        HIGHRES_IMU p105 = CommunicationChannel.instance.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.time_usec_SET(7011294213470954762L);
        p105.xacc_SET(3.0996065E38F);
        p105.yacc_SET(2.3646627E38F);
        p105.zacc_SET(-1.0128157E38F);
        p105.xgyro_SET(1.8098126E38F);
        p105.ygyro_SET(-8.925481E37F);
        p105.zgyro_SET(1.8959587E38F);
        p105.xmag_SET(-2.782485E38F);
        p105.ymag_SET(-1.7181241E38F);
        p105.zmag_SET(-2.2690063E38F);
        p105.abs_pressure_SET(-1.6904581E37F);
        p105.diff_pressure_SET(-7.566236E37F);
        p105.pressure_alt_SET(2.7479473E37F);
        p105.temperature_SET(-1.0173047E38F);
        p105.fields_updated_SET((char)53419);
        CommunicationChannel.instance.send(p105); //===============================
        OPTICAL_FLOW_RAD p106 = CommunicationChannel.instance.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_usec_SET(3069158023119397318L);
        p106.sensor_id_SET((char)63);
        p106.integration_time_us_SET(1156356235L);
        p106.integrated_x_SET(1.909574E38F);
        p106.integrated_y_SET(9.9213245E36F);
        p106.integrated_xgyro_SET(1.4576827E38F);
        p106.integrated_ygyro_SET(1.5127598E38F);
        p106.integrated_zgyro_SET(1.6380643E38F);
        p106.temperature_SET((short) -22064);
        p106.quality_SET((char)31);
        p106.time_delta_distance_us_SET(1100053122L);
        p106.distance_SET(-2.3712824E38F);
        CommunicationChannel.instance.send(p106); //===============================
        HIL_SENSOR p107 = CommunicationChannel.instance.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.time_usec_SET(7381500755085992282L);
        p107.xacc_SET(-3.0991812E38F);
        p107.yacc_SET(6.4656443E37F);
        p107.zacc_SET(-3.132735E38F);
        p107.xgyro_SET(1.0904427E38F);
        p107.ygyro_SET(3.3146923E38F);
        p107.zgyro_SET(-2.956358E37F);
        p107.xmag_SET(-2.7920516E38F);
        p107.ymag_SET(1.1787026E38F);
        p107.zmag_SET(-8.973156E37F);
        p107.abs_pressure_SET(2.9655043E38F);
        p107.diff_pressure_SET(1.158888E38F);
        p107.pressure_alt_SET(-3.3721475E38F);
        p107.temperature_SET(3.3447303E37F);
        p107.fields_updated_SET(439631681L);
        CommunicationChannel.instance.send(p107); //===============================
        SIM_STATE p108 = CommunicationChannel.instance.new_SIM_STATE();
        PH.setPack(p108);
        p108.q1_SET(-1.542215E38F);
        p108.q2_SET(1.8916517E38F);
        p108.q3_SET(-3.317334E38F);
        p108.q4_SET(1.845717E38F);
        p108.roll_SET(2.8115924E38F);
        p108.pitch_SET(2.2252754E38F);
        p108.yaw_SET(-1.5886908E38F);
        p108.xacc_SET(2.8882129E38F);
        p108.yacc_SET(1.5121799E38F);
        p108.zacc_SET(2.6527483E38F);
        p108.xgyro_SET(-2.1364922E38F);
        p108.ygyro_SET(-2.1133944E38F);
        p108.zgyro_SET(1.5944061E38F);
        p108.lat_SET(8.574178E37F);
        p108.lon_SET(-3.2513123E37F);
        p108.alt_SET(3.3951754E38F);
        p108.std_dev_horz_SET(-1.6977653E38F);
        p108.std_dev_vert_SET(-9.267328E37F);
        p108.vn_SET(2.0199951E38F);
        p108.ve_SET(1.0450131E38F);
        p108.vd_SET(-2.4696909E38F);
        CommunicationChannel.instance.send(p108); //===============================
        RADIO_STATUS p109 = CommunicationChannel.instance.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)210);
        p109.remrssi_SET((char)180);
        p109.txbuf_SET((char)74);
        p109.noise_SET((char)224);
        p109.remnoise_SET((char)218);
        p109.rxerrors_SET((char)32067);
        p109.fixed__SET((char)26741);
        CommunicationChannel.instance.send(p109); //===============================
        FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.instance.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)53);
        p110.target_system_SET((char)137);
        p110.target_component_SET((char)164);
        p110.payload_SET(new char[251], 0);
        CommunicationChannel.instance.send(p110); //===============================
        TIMESYNC p111 = CommunicationChannel.instance.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(-9066314238249721728L);
        p111.ts1_SET(6032526577505007466L);
        CommunicationChannel.instance.send(p111); //===============================
        CAMERA_TRIGGER p112 = CommunicationChannel.instance.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(4992040808645174441L);
        p112.seq_SET(1832811051L);
        CommunicationChannel.instance.send(p112); //===============================
        HIL_GPS p113 = CommunicationChannel.instance.new_HIL_GPS();
        PH.setPack(p113);
        p113.time_usec_SET(9048772951471620096L);
        p113.fix_type_SET((char)90);
        p113.lat_SET(978618146);
        p113.lon_SET(1600638485);
        p113.alt_SET(199309256);
        p113.eph_SET((char)62669);
        p113.epv_SET((char)47863);
        p113.vel_SET((char)5340);
        p113.vn_SET((short) -25016);
        p113.ve_SET((short) -5557);
        p113.vd_SET((short) -186);
        p113.cog_SET((char)47487);
        p113.satellites_visible_SET((char)243);
        CommunicationChannel.instance.send(p113); //===============================
        HIL_OPTICAL_FLOW p114 = CommunicationChannel.instance.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.time_usec_SET(6589139060104662312L);
        p114.sensor_id_SET((char)245);
        p114.integration_time_us_SET(2886393924L);
        p114.integrated_x_SET(3.176488E38F);
        p114.integrated_y_SET(-3.4572432E37F);
        p114.integrated_xgyro_SET(-3.0474655E38F);
        p114.integrated_ygyro_SET(-9.663983E37F);
        p114.integrated_zgyro_SET(5.467597E37F);
        p114.temperature_SET((short)23909);
        p114.quality_SET((char)70);
        p114.time_delta_distance_us_SET(541832758L);
        p114.distance_SET(-5.513986E37F);
        CommunicationChannel.instance.send(p114); //===============================
        HIL_STATE_QUATERNION p115 = CommunicationChannel.instance.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.time_usec_SET(4728525441021717561L);
        p115.attitude_quaternion_SET(new float[4], 0);
        p115.rollspeed_SET(-1.3004156E38F);
        p115.pitchspeed_SET(-1.3409017E38F);
        p115.yawspeed_SET(3.0282358E38F);
        p115.lat_SET(1395464402);
        p115.lon_SET(-9628001);
        p115.alt_SET(1617471267);
        p115.vx_SET((short)5508);
        p115.vy_SET((short) -14861);
        p115.vz_SET((short)26164);
        p115.ind_airspeed_SET((char)28606);
        p115.true_airspeed_SET((char)15134);
        p115.xacc_SET((short) -20182);
        p115.yacc_SET((short)9991);
        p115.zacc_SET((short)30015);
        CommunicationChannel.instance.send(p115); //===============================
        SCALED_IMU2 p116 = CommunicationChannel.instance.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.time_boot_ms_SET(4078919293L);
        p116.xacc_SET((short)2343);
        p116.yacc_SET((short)18013);
        p116.zacc_SET((short)23559);
        p116.xgyro_SET((short)9862);
        p116.ygyro_SET((short) -9898);
        p116.zgyro_SET((short)25660);
        p116.xmag_SET((short)11095);
        p116.ymag_SET((short) -23084);
        p116.zmag_SET((short)1526);
        CommunicationChannel.instance.send(p116); //===============================
        LOG_REQUEST_LIST p117 = CommunicationChannel.instance.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)205);
        p117.target_component_SET((char)156);
        p117.start_SET((char)47142);
        p117.end_SET((char)39738);
        CommunicationChannel.instance.send(p117); //===============================
        LOG_ENTRY p118 = CommunicationChannel.instance.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)35119);
        p118.num_logs_SET((char)3827);
        p118.last_log_num_SET((char)13600);
        p118.time_utc_SET(1223509398L);
        p118.size_SET(869174313L);
        CommunicationChannel.instance.send(p118); //===============================
        LOG_REQUEST_DATA p119 = CommunicationChannel.instance.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)3);
        p119.target_component_SET((char)226);
        p119.id_SET((char)64914);
        p119.ofs_SET(2915274814L);
        p119.count_SET(2040237877L);
        CommunicationChannel.instance.send(p119); //===============================
        LOG_DATA p120 = CommunicationChannel.instance.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)61943);
        p120.ofs_SET(717383414L);
        p120.count_SET((char)124);
        p120.data__SET(new char[90], 0);
        CommunicationChannel.instance.send(p120); //===============================
        LOG_ERASE p121 = CommunicationChannel.instance.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)36);
        p121.target_component_SET((char)169);
        CommunicationChannel.instance.send(p121); //===============================
        LOG_REQUEST_END p122 = CommunicationChannel.instance.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)17);
        p122.target_component_SET((char)144);
        CommunicationChannel.instance.send(p122); //===============================
        GPS_INJECT_DATA p123 = CommunicationChannel.instance.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)98);
        p123.target_component_SET((char)91);
        p123.len_SET((char)114);
        p123.data__SET(new char[110], 0);
        CommunicationChannel.instance.send(p123); //===============================
        GPS2_RAW p124 = CommunicationChannel.instance.new_GPS2_RAW();
        PH.setPack(p124);
        p124.time_usec_SET(8797825304965010017L);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
        p124.lat_SET(-36418916);
        p124.lon_SET(1217876065);
        p124.alt_SET(1591041182);
        p124.eph_SET((char)12458);
        p124.epv_SET((char)53810);
        p124.vel_SET((char)8007);
        p124.cog_SET((char)3228);
        p124.satellites_visible_SET((char)51);
        p124.dgps_numch_SET((char)47);
        p124.dgps_age_SET(4070013189L);
        CommunicationChannel.instance.send(p124); //===============================
        POWER_STATUS p125 = CommunicationChannel.instance.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)56568);
        p125.Vservo_SET((char)33093);
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_PERIPH_OVERCURRENT);
        CommunicationChannel.instance.send(p125); //===============================
        SERIAL_CONTROL p126 = CommunicationChannel.instance.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY);
        p126.timeout_SET((char)47924);
        p126.baudrate_SET(588432212L);
        p126.count_SET((char)243);
        p126.data__SET(new char[70], 0);
        CommunicationChannel.instance.send(p126); //===============================
        GPS_RTK p127 = CommunicationChannel.instance.new_GPS_RTK();
        PH.setPack(p127);
        p127.time_last_baseline_ms_SET(2551751176L);
        p127.rtk_receiver_id_SET((char)117);
        p127.wn_SET((char)41790);
        p127.tow_SET(395613234L);
        p127.rtk_health_SET((char)80);
        p127.rtk_rate_SET((char)9);
        p127.nsats_SET((char)241);
        p127.baseline_coords_type_SET((char)140);
        p127.baseline_a_mm_SET(2054760514);
        p127.baseline_b_mm_SET(-1891967840);
        p127.baseline_c_mm_SET(-1891463412);
        p127.accuracy_SET(4156580377L);
        p127.iar_num_hypotheses_SET(678416251);
        CommunicationChannel.instance.send(p127); //===============================
        GPS2_RTK p128 = CommunicationChannel.instance.new_GPS2_RTK();
        PH.setPack(p128);
        p128.time_last_baseline_ms_SET(3538748487L);
        p128.rtk_receiver_id_SET((char)47);
        p128.wn_SET((char)59282);
        p128.tow_SET(234183074L);
        p128.rtk_health_SET((char)167);
        p128.rtk_rate_SET((char)204);
        p128.nsats_SET((char)27);
        p128.baseline_coords_type_SET((char)157);
        p128.baseline_a_mm_SET(-379863866);
        p128.baseline_b_mm_SET(47384511);
        p128.baseline_c_mm_SET(1502873839);
        p128.accuracy_SET(1072692566L);
        p128.iar_num_hypotheses_SET(-1326394165);
        CommunicationChannel.instance.send(p128); //===============================
        SCALED_IMU3 p129 = CommunicationChannel.instance.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.time_boot_ms_SET(1079754569L);
        p129.xacc_SET((short)5631);
        p129.yacc_SET((short)7228);
        p129.zacc_SET((short) -30868);
        p129.xgyro_SET((short)29205);
        p129.ygyro_SET((short) -14483);
        p129.zgyro_SET((short)659);
        p129.xmag_SET((short)3228);
        p129.ymag_SET((short) -28437);
        p129.zmag_SET((short)24422);
        CommunicationChannel.instance.send(p129); //===============================
        DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.instance.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.type_SET((char)102);
        p130.size_SET(1289034052L);
        p130.width_SET((char)31318);
        p130.height_SET((char)35642);
        p130.packets_SET((char)34333);
        p130.payload_SET((char)186);
        p130.jpg_quality_SET((char)174);
        CommunicationChannel.instance.send(p130); //===============================
        ENCAPSULATED_DATA p131 = CommunicationChannel.instance.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)27781);
        p131.data__SET(new char[253], 0);
        CommunicationChannel.instance.send(p131); //===============================
        DISTANCE_SENSOR p132 = CommunicationChannel.instance.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.time_boot_ms_SET(2344912450L);
        p132.min_distance_SET((char)55414);
        p132.max_distance_SET((char)55868);
        p132.current_distance_SET((char)12381);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
        p132.id_SET((char)206);
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_PITCH_270);
        p132.covariance_SET((char)130);
        CommunicationChannel.instance.send(p132); //===============================
        TERRAIN_REQUEST p133 = CommunicationChannel.instance.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(-401667627);
        p133.lon_SET(-337607895);
        p133.grid_spacing_SET((char)42382);
        p133.mask_SET(6698525864614671108L);
        CommunicationChannel.instance.send(p133); //===============================
        TERRAIN_DATA p134 = CommunicationChannel.instance.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lat_SET(1279086323);
        p134.lon_SET(637458618);
        p134.grid_spacing_SET((char)1829);
        p134.gridbit_SET((char)105);
        p134.data__SET(new short[16], 0);
        CommunicationChannel.instance.send(p134); //===============================
        TERRAIN_CHECK p135 = CommunicationChannel.instance.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(1025686666);
        p135.lon_SET(242721885);
        CommunicationChannel.instance.send(p135); //===============================
        TERRAIN_REPORT p136 = CommunicationChannel.instance.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lat_SET(217966983);
        p136.lon_SET(1893271212);
        p136.spacing_SET((char)22257);
        p136.terrain_height_SET(1.5545087E38F);
        p136.current_height_SET(-6.078508E37F);
        p136.pending_SET((char)44152);
        p136.loaded_SET((char)35305);
        CommunicationChannel.instance.send(p136); //===============================
        SCALED_PRESSURE2 p137 = CommunicationChannel.instance.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(2587799853L);
        p137.press_abs_SET(2.5713745E38F);
        p137.press_diff_SET(1.9887282E38F);
        p137.temperature_SET((short) -16619);
        CommunicationChannel.instance.send(p137); //===============================
        ATT_POS_MOCAP p138 = CommunicationChannel.instance.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(838326167799722656L);
        p138.q_SET(new float[4], 0);
        p138.x_SET(1.7035695E38F);
        p138.y_SET(-3.2591143E38F);
        p138.z_SET(1.1991967E38F);
        CommunicationChannel.instance.send(p138); //===============================
        SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.instance.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(8262012714569704568L);
        p139.group_mlx_SET((char)175);
        p139.target_system_SET((char)248);
        p139.target_component_SET((char)127);
        p139.controls_SET(new float[8], 0);
        CommunicationChannel.instance.send(p139); //===============================
        ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.instance.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(7750537434962809361L);
        p140.group_mlx_SET((char)74);
        p140.controls_SET(new float[8], 0);
        CommunicationChannel.instance.send(p140); //===============================
        ALTITUDE p141 = CommunicationChannel.instance.new_ALTITUDE();
        PH.setPack(p141);
        p141.time_usec_SET(5275274545719413895L);
        p141.altitude_monotonic_SET(3.1407263E38F);
        p141.altitude_amsl_SET(2.5930203E38F);
        p141.altitude_local_SET(-1.0499535E38F);
        p141.altitude_relative_SET(2.0561887E38F);
        p141.altitude_terrain_SET(-1.863858E38F);
        p141.bottom_clearance_SET(-1.1271652E38F);
        CommunicationChannel.instance.send(p141); //===============================
        RESOURCE_REQUEST p142 = CommunicationChannel.instance.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.request_id_SET((char)80);
        p142.uri_type_SET((char)3);
        p142.uri_SET(new char[120], 0);
        p142.transfer_type_SET((char)167);
        p142.storage_SET(new char[120], 0);
        CommunicationChannel.instance.send(p142); //===============================
        SCALED_PRESSURE3 p143 = CommunicationChannel.instance.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(1297848202L);
        p143.press_abs_SET(-1.994093E38F);
        p143.press_diff_SET(3.0464092E38F);
        p143.temperature_SET((short)23363);
        CommunicationChannel.instance.send(p143); //===============================
        FOLLOW_TARGET p144 = CommunicationChannel.instance.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.timestamp_SET(7444325519818913274L);
        p144.est_capabilities_SET((char)181);
        p144.lat_SET(-991794910);
        p144.lon_SET(-564817220);
        p144.alt_SET(-6.6902466E37F);
        p144.vel_SET(new float[3], 0);
        p144.acc_SET(new float[3], 0);
        p144.attitude_q_SET(new float[4], 0);
        p144.rates_SET(new float[3], 0);
        p144.position_cov_SET(new float[3], 0);
        p144.custom_state_SET(2214806273159227169L);
        CommunicationChannel.instance.send(p144); //===============================
        CONTROL_SYSTEM_STATE p146 = CommunicationChannel.instance.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.time_usec_SET(8808025338224684407L);
        p146.x_acc_SET(2.9321933E38F);
        p146.y_acc_SET(1.1370517E38F);
        p146.z_acc_SET(-7.673765E37F);
        p146.x_vel_SET(-2.6148011E38F);
        p146.y_vel_SET(3.1168508E38F);
        p146.z_vel_SET(8.4284593E37F);
        p146.x_pos_SET(1.8349563E38F);
        p146.y_pos_SET(2.7582807E38F);
        p146.z_pos_SET(-2.9150082E38F);
        p146.airspeed_SET(1.8485013E38F);
        p146.vel_variance_SET(new float[3], 0);
        p146.pos_variance_SET(new float[3], 0);
        p146.q_SET(new float[4], 0);
        p146.roll_rate_SET(2.4300057E38F);
        p146.pitch_rate_SET(-1.5206221E38F);
        p146.yaw_rate_SET(2.1771792E38F);
        CommunicationChannel.instance.send(p146); //===============================
        BATTERY_STATUS p147 = CommunicationChannel.instance.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.id_SET((char)108);
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIPO);
        p147.temperature_SET((short)17794);
        p147.voltages_SET(new char[10], 0);
        p147.current_battery_SET((short) -23681);
        p147.current_consumed_SET(1296509916);
        p147.energy_consumed_SET(-88443298);
        p147.battery_remaining_SET((byte)58);
        CommunicationChannel.instance.send(p147); //===============================
        AUTOPILOT_VERSION p148 = CommunicationChannel.instance.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT);
        p148.flight_sw_version_SET(3460770353L);
        p148.middleware_sw_version_SET(1100884989L);
        p148.os_sw_version_SET(2894167151L);
        p148.board_version_SET(2854894656L);
        p148.flight_custom_version_SET(new char[8], 0);
        p148.middleware_custom_version_SET(new char[8], 0);
        p148.os_custom_version_SET(new char[8], 0);
        p148.vendor_id_SET((char)60084);
        p148.product_id_SET((char)19943);
        p148.uid_SET(2692636660720297916L);
        p148.uid2_SET(new char[18], 0, PH);
        CommunicationChannel.instance.send(p148); //===============================
        LANDING_TARGET p149 = CommunicationChannel.instance.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.time_usec_SET(4691116915086054239L);
        p149.target_num_SET((char)91);
        p149.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED);
        p149.angle_x_SET(1.832622E38F);
        p149.angle_y_SET(3.059009E38F);
        p149.distance_SET(1.8682746E38F);
        p149.size_x_SET(6.635498E37F);
        p149.size_y_SET(-7.0355217E37F);
        p149.x_SET(-2.1727732E38F, PH);
        p149.y_SET(1.2458444E38F, PH);
        p149.z_SET(-2.8189507E38F, PH);
        p149.q_SET(new float[4], 0, PH);
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_OTHER);
        p149.position_valid_SET((char)177, PH);
        CommunicationChannel.instance.send(p149); //===============================
        SENSOR_OFFSETS p150 = CommunicationChannel.instance.new_SENSOR_OFFSETS();
        PH.setPack(p150);
        p150.mag_ofs_x_SET((short) -28774);
        p150.mag_ofs_y_SET((short) -26732);
        p150.mag_ofs_z_SET((short)104);
        p150.mag_declination_SET(-2.0382566E38F);
        p150.raw_press_SET(-1297607940);
        p150.raw_temp_SET(211639443);
        p150.gyro_cal_x_SET(-1.0978434E38F);
        p150.gyro_cal_y_SET(-2.8375525E38F);
        p150.gyro_cal_z_SET(-1.2319487E38F);
        p150.accel_cal_x_SET(-8.4453743E37F);
        p150.accel_cal_y_SET(1.0819309E38F);
        p150.accel_cal_z_SET(9.276705E37F);
        CommunicationChannel.instance.send(p150); //===============================
        SET_MAG_OFFSETS p151 = CommunicationChannel.instance.new_SET_MAG_OFFSETS();
        PH.setPack(p151);
        p151.target_system_SET((char)236);
        p151.target_component_SET((char)8);
        p151.mag_ofs_x_SET((short)22638);
        p151.mag_ofs_y_SET((short) -7956);
        p151.mag_ofs_z_SET((short) -1452);
        CommunicationChannel.instance.send(p151); //===============================
        MEMINFO p152 = CommunicationChannel.instance.new_MEMINFO();
        PH.setPack(p152);
        p152.brkval_SET((char)15124);
        p152.freemem_SET((char)11151);
        p152.freemem32_SET(3371243019L, PH);
        CommunicationChannel.instance.send(p152); //===============================
        AP_ADC p153 = CommunicationChannel.instance.new_AP_ADC();
        PH.setPack(p153);
        p153.adc1_SET((char)42679);
        p153.adc2_SET((char)21127);
        p153.adc3_SET((char)50691);
        p153.adc4_SET((char)21202);
        p153.adc5_SET((char)45820);
        p153.adc6_SET((char)9989);
        CommunicationChannel.instance.send(p153); //===============================
        DIGICAM_CONFIGURE p154 = CommunicationChannel.instance.new_DIGICAM_CONFIGURE();
        PH.setPack(p154);
        p154.target_system_SET((char)93);
        p154.target_component_SET((char)186);
        p154.mode_SET((char)6);
        p154.shutter_speed_SET((char)21558);
        p154.aperture_SET((char)194);
        p154.iso_SET((char)26);
        p154.exposure_type_SET((char)190);
        p154.command_id_SET((char)215);
        p154.engine_cut_off_SET((char)111);
        p154.extra_param_SET((char)56);
        p154.extra_value_SET(-1.3683332E38F);
        CommunicationChannel.instance.send(p154); //===============================
        DIGICAM_CONTROL p155 = CommunicationChannel.instance.new_DIGICAM_CONTROL();
        PH.setPack(p155);
        p155.target_system_SET((char)100);
        p155.target_component_SET((char)84);
        p155.session_SET((char)55);
        p155.zoom_pos_SET((char)53);
        p155.zoom_step_SET((byte) - 102);
        p155.focus_lock_SET((char)239);
        p155.shot_SET((char)237);
        p155.command_id_SET((char)111);
        p155.extra_param_SET((char)11);
        p155.extra_value_SET(-3.1307033E38F);
        CommunicationChannel.instance.send(p155); //===============================
        MOUNT_CONFIGURE p156 = CommunicationChannel.instance.new_MOUNT_CONFIGURE();
        PH.setPack(p156);
        p156.target_system_SET((char)149);
        p156.target_component_SET((char)127);
        p156.mount_mode_SET(MAV_MOUNT_MODE.MAV_MOUNT_MODE_NEUTRAL);
        p156.stab_roll_SET((char)241);
        p156.stab_pitch_SET((char)28);
        p156.stab_yaw_SET((char)153);
        CommunicationChannel.instance.send(p156); //===============================
        MOUNT_CONTROL p157 = CommunicationChannel.instance.new_MOUNT_CONTROL();
        PH.setPack(p157);
        p157.target_system_SET((char)167);
        p157.target_component_SET((char)113);
        p157.input_a_SET(-896986593);
        p157.input_b_SET(2009588205);
        p157.input_c_SET(-1803203184);
        p157.save_position_SET((char)110);
        CommunicationChannel.instance.send(p157); //===============================
        MOUNT_STATUS p158 = CommunicationChannel.instance.new_MOUNT_STATUS();
        PH.setPack(p158);
        p158.target_system_SET((char)124);
        p158.target_component_SET((char)218);
        p158.pointing_a_SET(1190895885);
        p158.pointing_b_SET(2051860378);
        p158.pointing_c_SET(-1074797495);
        CommunicationChannel.instance.send(p158); //===============================
        FENCE_POINT p160 = CommunicationChannel.instance.new_FENCE_POINT();
        PH.setPack(p160);
        p160.target_system_SET((char)196);
        p160.target_component_SET((char)66);
        p160.idx_SET((char)169);
        p160.count_SET((char)157);
        p160.lat_SET(-2.5120271E38F);
        p160.lng_SET(-2.0510723E38F);
        CommunicationChannel.instance.send(p160); //===============================
        FENCE_FETCH_POINT p161 = CommunicationChannel.instance.new_FENCE_FETCH_POINT();
        PH.setPack(p161);
        p161.target_system_SET((char)108);
        p161.target_component_SET((char)15);
        p161.idx_SET((char)150);
        CommunicationChannel.instance.send(p161); //===============================
        FENCE_STATUS p162 = CommunicationChannel.instance.new_FENCE_STATUS();
        PH.setPack(p162);
        p162.breach_status_SET((char)84);
        p162.breach_count_SET((char)19905);
        p162.breach_type_SET(FENCE_BREACH.FENCE_BREACH_NONE);
        p162.breach_time_SET(1163742482L);
        CommunicationChannel.instance.send(p162); //===============================
        AHRS p163 = CommunicationChannel.instance.new_AHRS();
        PH.setPack(p163);
        p163.omegaIx_SET(2.5471017E38F);
        p163.omegaIy_SET(-1.3817917E38F);
        p163.omegaIz_SET(4.818214E37F);
        p163.accel_weight_SET(-1.7882439E38F);
        p163.renorm_val_SET(-6.008343E37F);
        p163.error_rp_SET(1.9912116E38F);
        p163.error_yaw_SET(-8.514826E37F);
        CommunicationChannel.instance.send(p163); //===============================
        SIMSTATE p164 = CommunicationChannel.instance.new_SIMSTATE();
        PH.setPack(p164);
        p164.roll_SET(2.803217E38F);
        p164.pitch_SET(3.2709401E38F);
        p164.yaw_SET(1.4677944E37F);
        p164.xacc_SET(-3.2073004E38F);
        p164.yacc_SET(1.2777563E38F);
        p164.zacc_SET(-2.323375E38F);
        p164.xgyro_SET(2.726649E37F);
        p164.ygyro_SET(1.0314184E38F);
        p164.zgyro_SET(3.1395984E38F);
        p164.lat_SET(1003818431);
        p164.lng_SET(-1674776772);
        CommunicationChannel.instance.send(p164); //===============================
        HWSTATUS p165 = CommunicationChannel.instance.new_HWSTATUS();
        PH.setPack(p165);
        p165.Vcc_SET((char)8421);
        p165.I2Cerr_SET((char)196);
        CommunicationChannel.instance.send(p165); //===============================
        RADIO p166 = CommunicationChannel.instance.new_RADIO();
        PH.setPack(p166);
        p166.rssi_SET((char)43);
        p166.remrssi_SET((char)254);
        p166.txbuf_SET((char)201);
        p166.noise_SET((char)236);
        p166.remnoise_SET((char)114);
        p166.rxerrors_SET((char)45625);
        p166.fixed__SET((char)40259);
        CommunicationChannel.instance.send(p166); //===============================
        LIMITS_STATUS p167 = CommunicationChannel.instance.new_LIMITS_STATUS();
        PH.setPack(p167);
        p167.limits_state_SET(LIMITS_STATE.LIMITS_ENABLED);
        p167.last_trigger_SET(1874207402L);
        p167.last_action_SET(536820714L);
        p167.last_recovery_SET(931351533L);
        p167.last_clear_SET(2462303393L);
        p167.breach_count_SET((char)6632);
        p167.mods_enabled_SET(LIMIT_MODULE.LIMIT_ALTITUDE);
        p167.mods_required_SET(LIMIT_MODULE.LIMIT_GPSLOCK);
        p167.mods_triggered_SET(LIMIT_MODULE.LIMIT_ALTITUDE);
        CommunicationChannel.instance.send(p167); //===============================
        WIND p168 = CommunicationChannel.instance.new_WIND();
        PH.setPack(p168);
        p168.direction_SET(-1.760195E38F);
        p168.speed_SET(3.2565843E38F);
        p168.speed_z_SET(-1.5055529E38F);
        CommunicationChannel.instance.send(p168); //===============================
        DATA16 p169 = CommunicationChannel.instance.new_DATA16();
        PH.setPack(p169);
        p169.type_SET((char)170);
        p169.len_SET((char)83);
        p169.data__SET(new char[16], 0);
        CommunicationChannel.instance.send(p169); //===============================
        DATA32 p170 = CommunicationChannel.instance.new_DATA32();
        PH.setPack(p170);
        p170.type_SET((char)119);
        p170.len_SET((char)30);
        p170.data__SET(new char[32], 0);
        CommunicationChannel.instance.send(p170); //===============================
        DATA64 p171 = CommunicationChannel.instance.new_DATA64();
        PH.setPack(p171);
        p171.type_SET((char)80);
        p171.len_SET((char)211);
        p171.data__SET(new char[64], 0);
        CommunicationChannel.instance.send(p171); //===============================
        DATA96 p172 = CommunicationChannel.instance.new_DATA96();
        PH.setPack(p172);
        p172.type_SET((char)163);
        p172.len_SET((char)184);
        p172.data__SET(new char[96], 0);
        CommunicationChannel.instance.send(p172); //===============================
        RANGEFINDER p173 = CommunicationChannel.instance.new_RANGEFINDER();
        PH.setPack(p173);
        p173.distance_SET(3.3481532E38F);
        p173.voltage_SET(2.5671194E38F);
        CommunicationChannel.instance.send(p173); //===============================
        AIRSPEED_AUTOCAL p174 = CommunicationChannel.instance.new_AIRSPEED_AUTOCAL();
        PH.setPack(p174);
        p174.vx_SET(-1.836593E38F);
        p174.vy_SET(-2.8924091E38F);
        p174.vz_SET(-3.1630468E38F);
        p174.diff_pressure_SET(-2.2493576E38F);
        p174.EAS2TAS_SET(8.903927E37F);
        p174.ratio_SET(-6.3244954E37F);
        p174.state_x_SET(-2.3641587E37F);
        p174.state_y_SET(-6.351535E37F);
        p174.state_z_SET(2.4711847E38F);
        p174.Pax_SET(-5.3856085E37F);
        p174.Pby_SET(-3.239495E37F);
        p174.Pcz_SET(6.199225E36F);
        CommunicationChannel.instance.send(p174); //===============================
        RALLY_POINT p175 = CommunicationChannel.instance.new_RALLY_POINT();
        PH.setPack(p175);
        p175.target_system_SET((char)172);
        p175.target_component_SET((char)88);
        p175.idx_SET((char)155);
        p175.count_SET((char)220);
        p175.lat_SET(-1153692612);
        p175.lng_SET(2136623129);
        p175.alt_SET((short)25334);
        p175.break_alt_SET((short) -30944);
        p175.land_dir_SET((char)63308);
        p175.flags_SET(RALLY_FLAGS.FAVORABLE_WIND);
        CommunicationChannel.instance.send(p175); //===============================
        RALLY_FETCH_POINT p176 = CommunicationChannel.instance.new_RALLY_FETCH_POINT();
        PH.setPack(p176);
        p176.target_system_SET((char)34);
        p176.target_component_SET((char)153);
        p176.idx_SET((char)115);
        CommunicationChannel.instance.send(p176); //===============================
        COMPASSMOT_STATUS p177 = CommunicationChannel.instance.new_COMPASSMOT_STATUS();
        PH.setPack(p177);
        p177.throttle_SET((char)40479);
        p177.current_SET(1.1642048E38F);
        p177.interference_SET((char)25559);
        p177.CompensationX_SET(-2.5459639E38F);
        p177.CompensationY_SET(1.1956136E38F);
        p177.CompensationZ_SET(-8.972589E37F);
        CommunicationChannel.instance.send(p177); //===============================
        AHRS2 p178 = CommunicationChannel.instance.new_AHRS2();
        PH.setPack(p178);
        p178.roll_SET(3.232485E38F);
        p178.pitch_SET(-1.5387118E37F);
        p178.yaw_SET(-1.5791228E38F);
        p178.altitude_SET(-7.0966133E37F);
        p178.lat_SET(1792145483);
        p178.lng_SET(-67637969);
        CommunicationChannel.instance.send(p178); //===============================
        CAMERA_STATUS p179 = CommunicationChannel.instance.new_CAMERA_STATUS();
        PH.setPack(p179);
        p179.time_usec_SET(8377200364574672055L);
        p179.target_system_SET((char)118);
        p179.cam_idx_SET((char)6);
        p179.img_idx_SET((char)9710);
        p179.event_id_SET(CAMERA_STATUS_TYPES.CAMERA_STATUS_TYPE_ERROR);
        p179.p1_SET(-3.1974166E38F);
        p179.p2_SET(3.0766008E38F);
        p179.p3_SET(2.7140927E38F);
        p179.p4_SET(-1.6864846E38F);
        CommunicationChannel.instance.send(p179); //===============================
        CAMERA_FEEDBACK p180 = CommunicationChannel.instance.new_CAMERA_FEEDBACK();
        PH.setPack(p180);
        p180.time_usec_SET(7903050306879865139L);
        p180.target_system_SET((char)154);
        p180.cam_idx_SET((char)83);
        p180.img_idx_SET((char)32497);
        p180.lat_SET(973562114);
        p180.lng_SET(332238221);
        p180.alt_msl_SET(-2.3898031E38F);
        p180.alt_rel_SET(-2.0418308E38F);
        p180.roll_SET(1.7295705E38F);
        p180.pitch_SET(-8.3992926E37F);
        p180.yaw_SET(-2.4687126E38F);
        p180.foc_len_SET(-1.922986E37F);
        p180.flags_SET(CAMERA_FEEDBACK_FLAGS.CAMERA_FEEDBACK_BADEXPOSURE);
        CommunicationChannel.instance.send(p180); //===============================
        BATTERY2 p181 = CommunicationChannel.instance.new_BATTERY2();
        PH.setPack(p181);
        p181.voltage_SET((char)35398);
        p181.current_battery_SET((short) -30294);
        CommunicationChannel.instance.send(p181); //===============================
        AHRS3 p182 = CommunicationChannel.instance.new_AHRS3();
        PH.setPack(p182);
        p182.roll_SET(2.2489647E38F);
        p182.pitch_SET(-1.356318E37F);
        p182.yaw_SET(-2.6701731E38F);
        p182.altitude_SET(2.7271764E38F);
        p182.lat_SET(-135389596);
        p182.lng_SET(1961162626);
        p182.v1_SET(-1.749757E38F);
        p182.v2_SET(1.3273674E38F);
        p182.v3_SET(-1.1574368E38F);
        p182.v4_SET(-1.8571868E38F);
        CommunicationChannel.instance.send(p182); //===============================
        AUTOPILOT_VERSION_REQUEST p183 = CommunicationChannel.instance.new_AUTOPILOT_VERSION_REQUEST();
        PH.setPack(p183);
        p183.target_system_SET((char)130);
        p183.target_component_SET((char)80);
        CommunicationChannel.instance.send(p183); //===============================
        REMOTE_LOG_DATA_BLOCK p184 = CommunicationChannel.instance.new_REMOTE_LOG_DATA_BLOCK();
        PH.setPack(p184);
        p184.target_system_SET((char)205);
        p184.target_component_SET((char)18);
        p184.seqno_SET(MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS.MAV_REMOTE_LOG_DATA_BLOCK_STOP);
        p184.data__SET(new char[200], 0);
        CommunicationChannel.instance.send(p184); //===============================
        REMOTE_LOG_BLOCK_STATUS p185 = CommunicationChannel.instance.new_REMOTE_LOG_BLOCK_STATUS();
        PH.setPack(p185);
        p185.target_system_SET((char)20);
        p185.target_component_SET((char)210);
        p185.seqno_SET(1588075387L);
        p185.status_SET(MAV_REMOTE_LOG_DATA_BLOCK_STATUSES.MAV_REMOTE_LOG_DATA_BLOCK_NACK);
        CommunicationChannel.instance.send(p185); //===============================
        LED_CONTROL p186 = CommunicationChannel.instance.new_LED_CONTROL();
        PH.setPack(p186);
        p186.target_system_SET((char)158);
        p186.target_component_SET((char)175);
        p186.instance_SET((char)97);
        p186.pattern_SET((char)55);
        p186.custom_len_SET((char)52);
        p186.custom_bytes_SET(new char[24], 0);
        CommunicationChannel.instance.send(p186); //===============================
        MAG_CAL_PROGRESS p191 = CommunicationChannel.instance.new_MAG_CAL_PROGRESS();
        PH.setPack(p191);
        p191.compass_id_SET((char)90);
        p191.cal_mask_SET((char)74);
        p191.cal_status_SET(MAG_CAL_STATUS.MAG_CAL_NOT_STARTED);
        p191.attempt_SET((char)30);
        p191.completion_pct_SET((char)50);
        p191.completion_mask_SET(new char[10], 0);
        p191.direction_x_SET(-2.4274953E38F);
        p191.direction_y_SET(-3.1877825E38F);
        p191.direction_z_SET(2.560103E38F);
        CommunicationChannel.instance.send(p191); //===============================
        MAG_CAL_REPORT p192 = CommunicationChannel.instance.new_MAG_CAL_REPORT();
        PH.setPack(p192);
        p192.compass_id_SET((char)50);
        p192.cal_mask_SET((char)235);
        p192.cal_status_SET(MAG_CAL_STATUS.MAG_CAL_WAITING_TO_START);
        p192.autosaved_SET((char)209);
        p192.fitness_SET(-3.484884E37F);
        p192.ofs_x_SET(-3.2660137E38F);
        p192.ofs_y_SET(-3.3008273E38F);
        p192.ofs_z_SET(1.1433599E38F);
        p192.diag_x_SET(-1.4388914E37F);
        p192.diag_y_SET(3.180071E38F);
        p192.diag_z_SET(1.6610224E38F);
        p192.offdiag_x_SET(2.89908E38F);
        p192.offdiag_y_SET(1.7729572E38F);
        p192.offdiag_z_SET(-1.9992286E38F);
        CommunicationChannel.instance.send(p192); //===============================
        EKF_STATUS_REPORT p193 = CommunicationChannel.instance.new_EKF_STATUS_REPORT();
        PH.setPack(p193);
        p193.flags_SET(EKF_STATUS_FLAGS.EKF_POS_HORIZ_ABS);
        p193.velocity_variance_SET(-3.8733964E37F);
        p193.pos_horiz_variance_SET(-2.2820124E38F);
        p193.pos_vert_variance_SET(6.159467E37F);
        p193.compass_variance_SET(-9.851307E37F);
        p193.terrain_alt_variance_SET(2.0508496E38F);
        CommunicationChannel.instance.send(p193); //===============================
        PID_TUNING p194 = CommunicationChannel.instance.new_PID_TUNING();
        PH.setPack(p194);
        p194.axis_SET(PID_TUNING_AXIS.PID_TUNING_ROLL);
        p194.desired_SET(1.6242179E38F);
        p194.achieved_SET(2.5969948E38F);
        p194.FF_SET(-2.2001802E38F);
        p194.P_SET(-3.349815E38F);
        p194.I_SET(-2.6371617E38F);
        p194.D_SET(4.824366E37F);
        CommunicationChannel.instance.send(p194); //===============================
        GIMBAL_REPORT p200 = CommunicationChannel.instance.new_GIMBAL_REPORT();
        PH.setPack(p200);
        p200.target_system_SET((char)216);
        p200.target_component_SET((char)87);
        p200.delta_time_SET(1.8055506E37F);
        p200.delta_angle_x_SET(-3.3209957E38F);
        p200.delta_angle_y_SET(-3.3943842E37F);
        p200.delta_angle_z_SET(2.6954767E38F);
        p200.delta_velocity_x_SET(-1.9960621E38F);
        p200.delta_velocity_y_SET(3.286128E38F);
        p200.delta_velocity_z_SET(1.3875381E38F);
        p200.joint_roll_SET(-1.3173841E38F);
        p200.joint_el_SET(-6.0435643E37F);
        p200.joint_az_SET(1.438657E38F);
        CommunicationChannel.instance.send(p200); //===============================
        GIMBAL_CONTROL p201 = CommunicationChannel.instance.new_GIMBAL_CONTROL();
        PH.setPack(p201);
        p201.target_system_SET((char)222);
        p201.target_component_SET((char)217);
        p201.demanded_rate_x_SET(1.150252E38F);
        p201.demanded_rate_y_SET(-2.6684645E38F);
        p201.demanded_rate_z_SET(-2.3050318E38F);
        CommunicationChannel.instance.send(p201); //===============================
        GIMBAL_TORQUE_CMD_REPORT p214 = CommunicationChannel.instance.new_GIMBAL_TORQUE_CMD_REPORT();
        PH.setPack(p214);
        p214.target_system_SET((char)125);
        p214.target_component_SET((char)60);
        p214.rl_torque_cmd_SET((short) -8160);
        p214.el_torque_cmd_SET((short)18080);
        p214.az_torque_cmd_SET((short)9031);
        CommunicationChannel.instance.send(p214); //===============================
        GOPRO_HEARTBEAT p215 = CommunicationChannel.instance.new_GOPRO_HEARTBEAT();
        PH.setPack(p215);
        p215.status_SET(GOPRO_HEARTBEAT_STATUS.GOPRO_HEARTBEAT_STATUS_DISCONNECTED);
        p215.capture_mode_SET(GOPRO_CAPTURE_MODE.GOPRO_CAPTURE_MODE_UNKNOWN);
        p215.flags_SET(GOPRO_HEARTBEAT_FLAGS.GOPRO_FLAG_RECORDING);
        CommunicationChannel.instance.send(p215); //===============================
        GOPRO_GET_REQUEST p216 = CommunicationChannel.instance.new_GOPRO_GET_REQUEST();
        PH.setPack(p216);
        p216.target_system_SET((char)35);
        p216.target_component_SET((char)7);
        p216.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_PHOTO_BURST_RATE);
        CommunicationChannel.instance.send(p216); //===============================
        GOPRO_GET_RESPONSE p217 = CommunicationChannel.instance.new_GOPRO_GET_RESPONSE();
        PH.setPack(p217);
        p217.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_MODEL);
        p217.status_SET(GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED);
        p217.value_SET(new char[4], 0);
        CommunicationChannel.instance.send(p217); //===============================
        GOPRO_SET_REQUEST p218 = CommunicationChannel.instance.new_GOPRO_SET_REQUEST();
        PH.setPack(p218);
        p218.target_system_SET((char)41);
        p218.target_component_SET((char)111);
        p218.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_CAPTURE_MODE);
        p218.value_SET(new char[4], 0);
        CommunicationChannel.instance.send(p218); //===============================
        GOPRO_SET_RESPONSE p219 = CommunicationChannel.instance.new_GOPRO_SET_RESPONSE();
        PH.setPack(p219);
        p219.cmd_id_SET(GOPRO_COMMAND.GOPRO_COMMAND_PROTUNE);
        p219.status_SET(GOPRO_REQUEST_STATUS.GOPRO_REQUEST_FAILED);
        CommunicationChannel.instance.send(p219); //===============================
        RPM p226 = CommunicationChannel.instance.new_RPM();
        PH.setPack(p226);
        p226.rpm1_SET(-9.639251E37F);
        p226.rpm2_SET(2.1989055E38F);
        CommunicationChannel.instance.send(p226); //===============================
        ESTIMATOR_STATUS p230 = CommunicationChannel.instance.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.time_usec_SET(3104640896960263236L);
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL);
        p230.vel_ratio_SET(-2.02764E38F);
        p230.pos_horiz_ratio_SET(-1.7088954E38F);
        p230.pos_vert_ratio_SET(-3.9087484E37F);
        p230.mag_ratio_SET(-3.2798287E38F);
        p230.hagl_ratio_SET(-1.337295E38F);
        p230.tas_ratio_SET(-2.4682737E38F);
        p230.pos_horiz_accuracy_SET(2.6841631E38F);
        p230.pos_vert_accuracy_SET(-3.3086845E38F);
        CommunicationChannel.instance.send(p230); //===============================
        WIND_COV p231 = CommunicationChannel.instance.new_WIND_COV();
        PH.setPack(p231);
        p231.time_usec_SET(5484158268449816810L);
        p231.wind_x_SET(-1.5245236E38F);
        p231.wind_y_SET(1.2152331E38F);
        p231.wind_z_SET(-4.4568405E37F);
        p231.var_horiz_SET(1.9866458E38F);
        p231.var_vert_SET(2.3923646E38F);
        p231.wind_alt_SET(2.2167238E38F);
        p231.horiz_accuracy_SET(-1.4465121E38F);
        p231.vert_accuracy_SET(-2.4433479E38F);
        CommunicationChannel.instance.send(p231); //===============================
        GPS_INPUT p232 = CommunicationChannel.instance.new_GPS_INPUT();
        PH.setPack(p232);
        p232.time_usec_SET(277675346281490879L);
        p232.gps_id_SET((char)205);
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ);
        p232.time_week_ms_SET(3111978115L);
        p232.time_week_SET((char)5018);
        p232.fix_type_SET((char)166);
        p232.lat_SET(1008584402);
        p232.lon_SET(261687284);
        p232.alt_SET(-2.5809737E38F);
        p232.hdop_SET(-7.906796E36F);
        p232.vdop_SET(1.0679058E38F);
        p232.vn_SET(9.432203E37F);
        p232.ve_SET(1.0242306E38F);
        p232.vd_SET(7.2356487E37F);
        p232.speed_accuracy_SET(2.0584356E38F);
        p232.horiz_accuracy_SET(-2.734558E38F);
        p232.vert_accuracy_SET(2.7434387E38F);
        p232.satellites_visible_SET((char)192);
        CommunicationChannel.instance.send(p232); //===============================
        GPS_RTCM_DATA p233 = CommunicationChannel.instance.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)234);
        p233.len_SET((char)6);
        p233.data__SET(new char[180], 0);
        CommunicationChannel.instance.send(p233); //===============================
        HIGH_LATENCY p234 = CommunicationChannel.instance.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
        p234.custom_mode_SET(1890184648L);
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
        p234.roll_SET((short)6364);
        p234.pitch_SET((short)18854);
        p234.heading_SET((char)6899);
        p234.throttle_SET((byte) - 27);
        p234.heading_sp_SET((short) -2003);
        p234.latitude_SET(1960389852);
        p234.longitude_SET(315072079);
        p234.altitude_amsl_SET((short) -13333);
        p234.altitude_sp_SET((short) -566);
        p234.airspeed_SET((char)9);
        p234.airspeed_sp_SET((char)9);
        p234.groundspeed_SET((char)207);
        p234.climb_rate_SET((byte) - 101);
        p234.gps_nsat_SET((char)188);
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
        p234.battery_remaining_SET((char)151);
        p234.temperature_SET((byte)98);
        p234.temperature_air_SET((byte)78);
        p234.failsafe_SET((char)229);
        p234.wp_num_SET((char)253);
        p234.wp_distance_SET((char)17730);
        CommunicationChannel.instance.send(p234); //===============================
        VIBRATION p241 = CommunicationChannel.instance.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(6703082629775499171L);
        p241.vibration_x_SET(2.2935789E38F);
        p241.vibration_y_SET(3.2604675E38F);
        p241.vibration_z_SET(-1.5524646E38F);
        p241.clipping_0_SET(4057576092L);
        p241.clipping_1_SET(3058907422L);
        p241.clipping_2_SET(2609384993L);
        CommunicationChannel.instance.send(p241); //===============================
        HOME_POSITION p242 = CommunicationChannel.instance.new_HOME_POSITION();
        PH.setPack(p242);
        p242.latitude_SET(-754134666);
        p242.longitude_SET(681918577);
        p242.altitude_SET(732890776);
        p242.x_SET(-2.5297779E38F);
        p242.y_SET(2.721576E38F);
        p242.z_SET(2.6789019E38F);
        p242.q_SET(new float[4], 0);
        p242.approach_x_SET(-2.475982E38F);
        p242.approach_y_SET(3.165217E38F);
        p242.approach_z_SET(2.0210448E38F);
        p242.time_usec_SET(7000753117056218797L, PH);
        CommunicationChannel.instance.send(p242); //===============================
        SET_HOME_POSITION p243 = CommunicationChannel.instance.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.target_system_SET((char)29);
        p243.latitude_SET(929033212);
        p243.longitude_SET(544976567);
        p243.altitude_SET(-1729144511);
        p243.x_SET(-1.045489E38F);
        p243.y_SET(1.823613E38F);
        p243.z_SET(-1.8783347E38F);
        p243.q_SET(new float[4], 0);
        p243.approach_x_SET(1.9799098E38F);
        p243.approach_y_SET(-1.0794218E38F);
        p243.approach_z_SET(1.3080442E38F);
        p243.time_usec_SET(1205125104683311726L, PH);
        CommunicationChannel.instance.send(p243); //===============================
        MESSAGE_INTERVAL p244 = CommunicationChannel.instance.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)48823);
        p244.interval_us_SET(-1498906802);
        CommunicationChannel.instance.send(p244); //===============================
        EXTENDED_SYS_STATE p245 = CommunicationChannel.instance.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
        CommunicationChannel.instance.send(p245); //===============================
        ADSB_VEHICLE p246 = CommunicationChannel.instance.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.ICAO_address_SET(726098603L);
        p246.lat_SET(1051775838);
        p246.lon_SET(-1687503021);
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
        p246.altitude_SET(-781073692);
        p246.heading_SET((char)41853);
        p246.hor_velocity_SET((char)53190);
        p246.ver_velocity_SET((short)13595);
        p246.callsign_SET("DEMO", PH);
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_NO_INFO);
        p246.tslc_SET((char)40);
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN);
        p246.squawk_SET((char)61730);
        CommunicationChannel.instance.send(p246); //===============================
        COLLISION p247 = CommunicationChannel.instance.new_COLLISION();
        PH.setPack(p247);
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
        p247.id_SET(1290516417L);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL);
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_LOW);
        p247.time_to_minimum_delta_SET(3.3414566E38F);
        p247.altitude_minimum_delta_SET(-2.3405586E38F);
        p247.horizontal_minimum_delta_SET(5.703365E37F);
        CommunicationChannel.instance.send(p247); //===============================
        V2_EXTENSION p248 = CommunicationChannel.instance.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)167);
        p248.target_system_SET((char)101);
        p248.target_component_SET((char)175);
        p248.message_type_SET((char)16145);
        p248.payload_SET(new char[249], 0);
        CommunicationChannel.instance.send(p248); //===============================
        MEMORY_VECT p249 = CommunicationChannel.instance.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)60884);
        p249.ver_SET((char)188);
        p249.type_SET((char)249);
        p249.value_SET(new byte[32], 0);
        CommunicationChannel.instance.send(p249); //===============================
        DEBUG_VECT p250 = CommunicationChannel.instance.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("DEMO", PH);
        p250.time_usec_SET(5786193641306027539L);
        p250.x_SET(6.2099734E37F);
        p250.y_SET(8.801703E37F);
        p250.z_SET(-3.306518E38F);
        CommunicationChannel.instance.send(p250); //===============================
        NAMED_VALUE_FLOAT p251 = CommunicationChannel.instance.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(3545681951L);
        p251.name_SET("DEMO", PH);
        p251.value_SET(1.0048184E38F);
        CommunicationChannel.instance.send(p251); //===============================
        NAMED_VALUE_INT p252 = CommunicationChannel.instance.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(1511324935L);
        p252.name_SET("DEMO", PH);
        p252.value_SET(-533680638);
        CommunicationChannel.instance.send(p252); //===============================
        STATUSTEXT p253 = CommunicationChannel.instance.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_DEBUG);
        p253.text_SET("DEMO", PH);
        CommunicationChannel.instance.send(p253); //===============================
        DEBUG p254 = CommunicationChannel.instance.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(870289323L);
        p254.ind_SET((char)51);
        p254.value_SET(-3.1401742E38F);
        CommunicationChannel.instance.send(p254); //===============================
        SETUP_SIGNING p256 = CommunicationChannel.instance.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)73);
        p256.target_component_SET((char)169);
        p256.secret_key_SET(new char[32], 0);
        p256.initial_timestamp_SET(6729255957512180141L);
        CommunicationChannel.instance.send(p256); //===============================
        BUTTON_CHANGE p257 = CommunicationChannel.instance.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(726405590L);
        p257.last_change_ms_SET(1867403775L);
        p257.state_SET((char)20);
        CommunicationChannel.instance.send(p257); //===============================
        PLAY_TUNE p258 = CommunicationChannel.instance.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)223);
        p258.target_component_SET((char)205);
        p258.tune_SET("DEMO", PH);
        CommunicationChannel.instance.send(p258); //===============================
        CAMERA_INFORMATION p259 = CommunicationChannel.instance.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.time_boot_ms_SET(3631221056L);
        p259.vendor_name_SET(new char[32], 0);
        p259.model_name_SET(new char[32], 0);
        p259.firmware_version_SET(2411804769L);
        p259.focal_length_SET(3.2699023E38F);
        p259.sensor_size_h_SET(2.7372246E38F);
        p259.sensor_size_v_SET(-2.1200613E38F);
        p259.resolution_h_SET((char)31472);
        p259.resolution_v_SET((char)30523);
        p259.lens_id_SET((char)69);
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE);
        p259.cam_definition_version_SET((char)58956);
        p259.cam_definition_uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p259); //===============================
        CAMERA_SETTINGS p260 = CommunicationChannel.instance.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(1675088451L);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE);
        CommunicationChannel.instance.send(p260); //===============================
        STORAGE_INFORMATION p261 = CommunicationChannel.instance.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.time_boot_ms_SET(1054745245L);
        p261.storage_id_SET((char)190);
        p261.storage_count_SET((char)166);
        p261.status_SET((char)42);
        p261.total_capacity_SET(1.8032545E38F);
        p261.used_capacity_SET(-2.5413594E38F);
        p261.available_capacity_SET(2.3046764E38F);
        p261.read_speed_SET(-3.3434658E38F);
        p261.write_speed_SET(2.612583E38F);
        CommunicationChannel.instance.send(p261); //===============================
        CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.instance.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.time_boot_ms_SET(1151832516L);
        p262.image_status_SET((char)171);
        p262.video_status_SET((char)131);
        p262.image_interval_SET(3.0101873E38F);
        p262.recording_time_ms_SET(3371394431L);
        p262.available_capacity_SET(-3.3296269E38F);
        CommunicationChannel.instance.send(p262); //===============================
        CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.instance.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.time_boot_ms_SET(1874471850L);
        p263.time_utc_SET(6646226395697556654L);
        p263.camera_id_SET((char)138);
        p263.lat_SET(-57404273);
        p263.lon_SET(-631336495);
        p263.alt_SET(800383937);
        p263.relative_alt_SET(1214606242);
        p263.q_SET(new float[4], 0);
        p263.image_index_SET(-1425414957);
        p263.capture_result_SET((byte)126);
        p263.file_url_SET("DEMO", PH);
        CommunicationChannel.instance.send(p263); //===============================
        FLIGHT_INFORMATION p264 = CommunicationChannel.instance.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.time_boot_ms_SET(3237028611L);
        p264.arming_time_utc_SET(479668313861969467L);
        p264.takeoff_time_utc_SET(4561518217550724392L);
        p264.flight_uuid_SET(940114990144959956L);
        CommunicationChannel.instance.send(p264); //===============================
        MOUNT_ORIENTATION p265 = CommunicationChannel.instance.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(3531158383L);
        p265.roll_SET(-2.3920538E38F);
        p265.pitch_SET(-1.8502955E38F);
        p265.yaw_SET(-2.2434187E38F);
        CommunicationChannel.instance.send(p265); //===============================
        LOGGING_DATA p266 = CommunicationChannel.instance.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_system_SET((char)234);
        p266.target_component_SET((char)100);
        p266.sequence_SET((char)21063);
        p266.length_SET((char)237);
        p266.first_message_offset_SET((char)210);
        p266.data__SET(new char[249], 0);
        CommunicationChannel.instance.send(p266); //===============================
        LOGGING_DATA_ACKED p267 = CommunicationChannel.instance.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_system_SET((char)216);
        p267.target_component_SET((char)57);
        p267.sequence_SET((char)9438);
        p267.length_SET((char)107);
        p267.first_message_offset_SET((char)192);
        p267.data__SET(new char[249], 0);
        CommunicationChannel.instance.send(p267); //===============================
        LOGGING_ACK p268 = CommunicationChannel.instance.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)116);
        p268.target_component_SET((char)226);
        p268.sequence_SET((char)42281);
        CommunicationChannel.instance.send(p268); //===============================
        VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.instance.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.camera_id_SET((char)107);
        p269.status_SET((char)25);
        p269.framerate_SET(1.1266683E37F);
        p269.resolution_h_SET((char)58132);
        p269.resolution_v_SET((char)27212);
        p269.bitrate_SET(2262799918L);
        p269.rotation_SET((char)36611);
        p269.uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p269); //===============================
        SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.instance.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_system_SET((char)134);
        p270.target_component_SET((char)30);
        p270.camera_id_SET((char)25);
        p270.framerate_SET(1.8072538E38F);
        p270.resolution_h_SET((char)57557);
        p270.resolution_v_SET((char)64974);
        p270.bitrate_SET(2338901583L);
        p270.rotation_SET((char)22833);
        p270.uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p270); //===============================
        WIFI_CONFIG_AP p299 = CommunicationChannel.instance.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("DEMO", PH);
        p299.password_SET("DEMO", PH);
        CommunicationChannel.instance.send(p299); //===============================
        PROTOCOL_VERSION p300 = CommunicationChannel.instance.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)54551);
        p300.min_version_SET((char)19993);
        p300.max_version_SET((char)16107);
        p300.spec_version_hash_SET(new char[8], 0);
        p300.library_version_hash_SET(new char[8], 0);
        CommunicationChannel.instance.send(p300); //===============================
        UAVCAN_NODE_STATUS p310 = CommunicationChannel.instance.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.time_usec_SET(8808659280684869497L);
        p310.uptime_sec_SET(1635876450L);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
        p310.sub_mode_SET((char)68);
        p310.vendor_specific_status_code_SET((char)18895);
        CommunicationChannel.instance.send(p310); //===============================
        UAVCAN_NODE_INFO p311 = CommunicationChannel.instance.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.time_usec_SET(5506852192033536201L);
        p311.uptime_sec_SET(2615511240L);
        p311.name_SET("DEMO", PH);
        p311.hw_version_major_SET((char)135);
        p311.hw_version_minor_SET((char)158);
        p311.hw_unique_id_SET(new char[16], 0);
        p311.sw_version_major_SET((char)84);
        p311.sw_version_minor_SET((char)21);
        p311.sw_vcs_commit_SET(2978169437L);
        CommunicationChannel.instance.send(p311); //===============================
        PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)9);
        p320.target_component_SET((char)153);
        p320.param_id_SET("DEMO", PH);
        p320.param_index_SET((short)19664);
        CommunicationChannel.instance.send(p320); //===============================
        PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)54);
        p321.target_component_SET((char)255);
        CommunicationChannel.instance.send(p321); //===============================
        PARAM_EXT_VALUE p322 = CommunicationChannel.instance.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("DEMO", PH);
        p322.param_value_SET("DEMO", PH);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
        p322.param_count_SET((char)47712);
        p322.param_index_SET((char)5808);
        CommunicationChannel.instance.send(p322); //===============================
        PARAM_EXT_SET p323 = CommunicationChannel.instance.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)116);
        p323.target_component_SET((char)253);
        p323.param_id_SET("DEMO", PH);
        p323.param_value_SET("DEMO", PH);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
        CommunicationChannel.instance.send(p323); //===============================
        PARAM_EXT_ACK p324 = CommunicationChannel.instance.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("DEMO", PH);
        p324.param_value_SET("DEMO", PH);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_VALUE_UNSUPPORTED);
        CommunicationChannel.instance.send(p324); //===============================
        OBSTACLE_DISTANCE p330 = CommunicationChannel.instance.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.time_usec_SET(6261835219626853978L);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED);
        p330.distances_SET(new char[72], 0);
        p330.increment_SET((char)85);
        p330.min_distance_SET((char)11791);
        p330.max_distance_SET((char)35294);
        CommunicationChannel.instance.send(p330); //===============================
        UAVIONIX_ADSB_OUT_CFG p10001 = CommunicationChannel.instance.new_UAVIONIX_ADSB_OUT_CFG();
        PH.setPack(p10001);
        p10001.ICAO_SET(3988288656L);
        p10001.callsign_SET("DEMO", PH);
        p10001.emitterType_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SERVICE_SURFACE);
        p10001.aircraftSize_SET(UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE.UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_67M);
        p10001.gpsOffsetLat_SET(UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_4M);
        p10001.gpsOffsetLon_SET(UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON.UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA);
        p10001.stallSpeed_SET((char)4916);
        p10001.rfSelect_SET(UAVIONIX_ADSB_OUT_RF_SELECT.UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED);
        CommunicationChannel.instance.send(p10001); //===============================
        UAVIONIX_ADSB_OUT_DYNAMIC p10002 = CommunicationChannel.instance.new_UAVIONIX_ADSB_OUT_DYNAMIC();
        PH.setPack(p10002);
        p10002.utcTime_SET(3458934851L);
        p10002.gpsLat_SET(-674844017);
        p10002.gpsLon_SET(416615144);
        p10002.gpsAlt_SET(1107888329);
        p10002.gpsFix_SET(UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX.UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_2D);
        p10002.numSats_SET((char)120);
        p10002.baroAltMSL_SET(-704804133);
        p10002.accuracyHor_SET(149187673L);
        p10002.accuracyVert_SET((char)3221);
        p10002.accuracyVel_SET((char)9404);
        p10002.velVert_SET((short) -32053);
        p10002.velNS_SET((short) -7873);
        p10002.VelEW_SET((short) -31104);
        p10002.emergencyStatus_SET(UAVIONIX_ADSB_EMERGENCY_STATUS.UAVIONIX_ADSB_OUT_DOWNED_AIRCRAFT_EMERGENCY);
        p10002.state_SET(UAVIONIX_ADSB_OUT_DYNAMIC_STATE.UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE);
        p10002.squawk_SET((char)31740);
        CommunicationChannel.instance.send(p10002); //===============================
        UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT p10003 = CommunicationChannel.instance.new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT();
        PH.setPack(p10003);
        p10003.rfHealth_SET(UAVIONIX_ADSB_RF_HEALTH.UAVIONIX_ADSB_RF_HEALTH_OK);
        CommunicationChannel.instance.send(p10003); //===============================
        DEVICE_OP_READ p11000 = CommunicationChannel.instance.new_DEVICE_OP_READ();
        PH.setPack(p11000);
        p11000.target_system_SET((char)23);
        p11000.target_component_SET((char)107);
        p11000.request_id_SET(1591495567L);
        p11000.bustype_SET(DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C);
        p11000.bus_SET((char)174);
        p11000.address_SET((char)122);
        p11000.busname_SET("DEMO", PH);
        p11000.regstart_SET((char)238);
        p11000.count_SET((char)79);
        CommunicationChannel.instance.send(p11000); //===============================
        DEVICE_OP_READ_REPLY p11001 = CommunicationChannel.instance.new_DEVICE_OP_READ_REPLY();
        PH.setPack(p11001);
        p11001.request_id_SET(1969229814L);
        p11001.result_SET((char)66);
        p11001.regstart_SET((char)128);
        p11001.count_SET((char)178);
        p11001.data__SET(new char[128], 0);
        CommunicationChannel.instance.send(p11001); //===============================
        DEVICE_OP_WRITE p11002 = CommunicationChannel.instance.new_DEVICE_OP_WRITE();
        PH.setPack(p11002);
        p11002.target_system_SET((char)12);
        p11002.target_component_SET((char)191);
        p11002.request_id_SET(2363464254L);
        p11002.bustype_SET(DEVICE_OP_BUSTYPE.DEVICE_OP_BUSTYPE_I2C);
        p11002.bus_SET((char)13);
        p11002.address_SET((char)51);
        p11002.busname_SET("DEMO", PH);
        p11002.regstart_SET((char)137);
        p11002.count_SET((char)108);
        p11002.data__SET(new char[128], 0);
        CommunicationChannel.instance.send(p11002); //===============================
        DEVICE_OP_WRITE_REPLY p11003 = CommunicationChannel.instance.new_DEVICE_OP_WRITE_REPLY();
        PH.setPack(p11003);
        p11003.request_id_SET(1962474101L);
        p11003.result_SET((char)171);
        CommunicationChannel.instance.send(p11003); //===============================
        ADAP_TUNING p11010 = CommunicationChannel.instance.new_ADAP_TUNING();
        PH.setPack(p11010);
        p11010.axis_SET(PID_TUNING_AXIS.PID_TUNING_PITCH);
        p11010.desired_SET(2.24757E38F);
        p11010.achieved_SET(-1.2650838E38F);
        p11010.error_SET(2.9813895E38F);
        p11010.theta_SET(-3.1977107E38F);
        p11010.omega_SET(3.1121274E37F);
        p11010.sigma_SET(2.9520205E38F);
        p11010.theta_dot_SET(1.90809E38F);
        p11010.omega_dot_SET(-2.774381E38F);
        p11010.sigma_dot_SET(1.5504602E38F);
        p11010.f_SET(2.6152429E38F);
        p11010.f_dot_SET(-1.2288116E38F);
        p11010.u_SET(2.444445E36F);
        CommunicationChannel.instance.send(p11010); //===============================
        VISION_POSITION_DELTA p11011 = CommunicationChannel.instance.new_VISION_POSITION_DELTA();
        PH.setPack(p11011);
        p11011.time_usec_SET(4689885540971063141L);
        p11011.time_delta_usec_SET(6039441604297246944L);
        p11011.angle_delta_SET(new float[3], 0);
        p11011.position_delta_SET(new float[3], 0);
        p11011.confidence_SET(-1.1003723E37F);
        CommunicationChannel.instance.send(p11011); //===============================
    }
}
