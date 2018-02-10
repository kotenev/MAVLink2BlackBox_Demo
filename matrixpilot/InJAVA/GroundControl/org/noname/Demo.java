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
        POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.instance.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.time_boot_ms_SET(577804694L);
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
        p87.type_mask_SET((char)64168);
        p87.lat_int_SET(-2070828529);
        p87.lon_int_SET(-1535105630);
        p87.alt_SET(3.2301745E37F);
        p87.vx_SET(-2.5561189E38F);
        p87.vy_SET(8.816459E37F);
        p87.vz_SET(-2.4557688E38F);
        p87.afx_SET(-3.0699368E38F);
        p87.afy_SET(2.6162325E38F);
        p87.afz_SET(1.954449E38F);
        p87.yaw_SET(-1.4992145E38F);
        p87.yaw_rate_SET(-5.200203E37F);
        CommunicationChannel.instance.send(p87); //===============================
        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.instance.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.time_boot_ms_SET(3724390613L);
        p89.x_SET(-3.7320237E37F);
        p89.y_SET(2.0548541E38F);
        p89.z_SET(-1.8464483E38F);
        p89.roll_SET(2.6933937E38F);
        p89.pitch_SET(-7.9271237E37F);
        p89.yaw_SET(1.8133123E38F);
        CommunicationChannel.instance.send(p89); //===============================
        HIL_STATE p90 = CommunicationChannel.instance.new_HIL_STATE();
        PH.setPack(p90);
        p90.time_usec_SET(8448301363041289282L);
        p90.roll_SET(4.98356E37F);
        p90.pitch_SET(-2.7838015E37F);
        p90.yaw_SET(-8.337182E36F);
        p90.rollspeed_SET(-2.6435478E38F);
        p90.pitchspeed_SET(-2.4214493E38F);
        p90.yawspeed_SET(4.0509252E36F);
        p90.lat_SET(-2005784599);
        p90.lon_SET(-268544327);
        p90.alt_SET(1585379181);
        p90.vx_SET((short) -7867);
        p90.vy_SET((short)17810);
        p90.vz_SET((short)393);
        p90.xacc_SET((short) -10601);
        p90.yacc_SET((short)32439);
        p90.zacc_SET((short)5535);
        CommunicationChannel.instance.send(p90); //===============================
        HIL_CONTROLS p91 = CommunicationChannel.instance.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.time_usec_SET(3755660158501600710L);
        p91.roll_ailerons_SET(-1.8277366E38F);
        p91.pitch_elevator_SET(1.2633341E38F);
        p91.yaw_rudder_SET(3.1267638E38F);
        p91.throttle_SET(1.9214032E38F);
        p91.aux1_SET(3.3646147E38F);
        p91.aux2_SET(1.4756275E38F);
        p91.aux3_SET(-1.4885165E38F);
        p91.aux4_SET(-3.1336584E38F);
        p91.mode_SET(MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
        p91.nav_mode_SET((char)51);
        CommunicationChannel.instance.send(p91); //===============================
        HIL_RC_INPUTS_RAW p92 = CommunicationChannel.instance.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.time_usec_SET(567975648707837392L);
        p92.chan1_raw_SET((char)9329);
        p92.chan2_raw_SET((char)47633);
        p92.chan3_raw_SET((char)51134);
        p92.chan4_raw_SET((char)59032);
        p92.chan5_raw_SET((char)43678);
        p92.chan6_raw_SET((char)1152);
        p92.chan7_raw_SET((char)61737);
        p92.chan8_raw_SET((char)29087);
        p92.chan9_raw_SET((char)21030);
        p92.chan10_raw_SET((char)33049);
        p92.chan11_raw_SET((char)50026);
        p92.chan12_raw_SET((char)1009);
        p92.rssi_SET((char)218);
        CommunicationChannel.instance.send(p92); //===============================
        HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.instance.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(8827903697502620713L);
        p93.controls_SET(new float[16], 0);
        p93.mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED);
        p93.flags_SET(7561395266897570823L);
        CommunicationChannel.instance.send(p93); //===============================
        OPTICAL_FLOW p100 = CommunicationChannel.instance.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.time_usec_SET(489517948934861328L);
        p100.sensor_id_SET((char)115);
        p100.flow_x_SET((short) -4221);
        p100.flow_y_SET((short)9203);
        p100.flow_comp_m_x_SET(1.284355E38F);
        p100.flow_comp_m_y_SET(-1.809876E38F);
        p100.quality_SET((char)117);
        p100.ground_distance_SET(-3.3763949E38F);
        p100.flow_rate_x_SET(-1.6004246E37F, PH);
        p100.flow_rate_y_SET(-1.0756467E37F, PH);
        CommunicationChannel.instance.send(p100); //===============================
        GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.instance.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.usec_SET(5111936798500086008L);
        p101.x_SET(5.0188615E37F);
        p101.y_SET(1.9443775E38F);
        p101.z_SET(3.6072324E37F);
        p101.roll_SET(-2.4685195E38F);
        p101.pitch_SET(7.2697005E36F);
        p101.yaw_SET(-2.8885514E38F);
        CommunicationChannel.instance.send(p101); //===============================
        VISION_POSITION_ESTIMATE p102 = CommunicationChannel.instance.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.usec_SET(6628262181521301500L);
        p102.x_SET(-1.0292021E38F);
        p102.y_SET(2.37423E38F);
        p102.z_SET(-6.5001954E37F);
        p102.roll_SET(-1.1608732E38F);
        p102.pitch_SET(-2.3542213E38F);
        p102.yaw_SET(1.0588543E38F);
        CommunicationChannel.instance.send(p102); //===============================
        VISION_SPEED_ESTIMATE p103 = CommunicationChannel.instance.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(4689754044506930403L);
        p103.x_SET(-3.34189E38F);
        p103.y_SET(-2.3209979E38F);
        p103.z_SET(-3.7535768E37F);
        CommunicationChannel.instance.send(p103); //===============================
        VICON_POSITION_ESTIMATE p104 = CommunicationChannel.instance.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(5178041875926979924L);
        p104.x_SET(3.3329545E36F);
        p104.y_SET(3.0780617E38F);
        p104.z_SET(-2.9892157E37F);
        p104.roll_SET(1.0490415E38F);
        p104.pitch_SET(-2.5780488E38F);
        p104.yaw_SET(2.7552013E37F);
        CommunicationChannel.instance.send(p104); //===============================
        HIGHRES_IMU p105 = CommunicationChannel.instance.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.time_usec_SET(1404931668856174565L);
        p105.xacc_SET(-9.430647E37F);
        p105.yacc_SET(1.9724067E38F);
        p105.zacc_SET(-7.092522E36F);
        p105.xgyro_SET(-2.636212E38F);
        p105.ygyro_SET(8.067618E37F);
        p105.zgyro_SET(2.7879578E38F);
        p105.xmag_SET(-4.6958763E37F);
        p105.ymag_SET(-3.3401224E38F);
        p105.zmag_SET(2.7364433E38F);
        p105.abs_pressure_SET(2.1193974E38F);
        p105.diff_pressure_SET(2.371094E38F);
        p105.pressure_alt_SET(-5.6588836E37F);
        p105.temperature_SET(-5.991777E37F);
        p105.fields_updated_SET((char)39319);
        CommunicationChannel.instance.send(p105); //===============================
        OPTICAL_FLOW_RAD p106 = CommunicationChannel.instance.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_usec_SET(8961072396053607229L);
        p106.sensor_id_SET((char)188);
        p106.integration_time_us_SET(2362219259L);
        p106.integrated_x_SET(7.2712165E37F);
        p106.integrated_y_SET(1.3661934E38F);
        p106.integrated_xgyro_SET(-3.365381E36F);
        p106.integrated_ygyro_SET(1.9778023E38F);
        p106.integrated_zgyro_SET(1.6216061E38F);
        p106.temperature_SET((short) -30815);
        p106.quality_SET((char)100);
        p106.time_delta_distance_us_SET(1184103813L);
        p106.distance_SET(-8.590483E37F);
        CommunicationChannel.instance.send(p106); //===============================
        HIL_SENSOR p107 = CommunicationChannel.instance.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.time_usec_SET(712250309518207101L);
        p107.xacc_SET(-1.4634478E38F);
        p107.yacc_SET(1.830797E38F);
        p107.zacc_SET(2.6455564E38F);
        p107.xgyro_SET(2.8835494E38F);
        p107.ygyro_SET(-3.4677084E37F);
        p107.zgyro_SET(3.1419996E38F);
        p107.xmag_SET(3.0001395E38F);
        p107.ymag_SET(2.7614215E38F);
        p107.zmag_SET(2.2192439E38F);
        p107.abs_pressure_SET(-2.334494E38F);
        p107.diff_pressure_SET(-1.8492179E38F);
        p107.pressure_alt_SET(2.2891886E38F);
        p107.temperature_SET(-2.2867786E38F);
        p107.fields_updated_SET(2928410133L);
        CommunicationChannel.instance.send(p107); //===============================
        SIM_STATE p108 = CommunicationChannel.instance.new_SIM_STATE();
        PH.setPack(p108);
        p108.q1_SET(2.3748805E38F);
        p108.q2_SET(1.8472744E38F);
        p108.q3_SET(-1.4286178E38F);
        p108.q4_SET(2.645144E38F);
        p108.roll_SET(3.0894178E38F);
        p108.pitch_SET(-2.3967208E38F);
        p108.yaw_SET(3.3802139E38F);
        p108.xacc_SET(-3.2929697E38F);
        p108.yacc_SET(-2.900862E38F);
        p108.zacc_SET(-1.4758833E38F);
        p108.xgyro_SET(-2.6017583E38F);
        p108.ygyro_SET(1.6356218E38F);
        p108.zgyro_SET(-2.973865E38F);
        p108.lat_SET(2.6754086E38F);
        p108.lon_SET(-3.204879E38F);
        p108.alt_SET(-3.015861E38F);
        p108.std_dev_horz_SET(-3.2135217E38F);
        p108.std_dev_vert_SET(3.9249637E37F);
        p108.vn_SET(2.0571858E38F);
        p108.ve_SET(-1.5234453E38F);
        p108.vd_SET(1.4933568E38F);
        CommunicationChannel.instance.send(p108); //===============================
        RADIO_STATUS p109 = CommunicationChannel.instance.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)238);
        p109.remrssi_SET((char)157);
        p109.txbuf_SET((char)211);
        p109.noise_SET((char)198);
        p109.remnoise_SET((char)54);
        p109.rxerrors_SET((char)64151);
        p109.fixed__SET((char)15228);
        CommunicationChannel.instance.send(p109); //===============================
        FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.instance.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)206);
        p110.target_system_SET((char)145);
        p110.target_component_SET((char)47);
        p110.payload_SET(new char[251], 0);
        CommunicationChannel.instance.send(p110); //===============================
        TIMESYNC p111 = CommunicationChannel.instance.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(9024756351301395589L);
        p111.ts1_SET(-7248544862341934211L);
        CommunicationChannel.instance.send(p111); //===============================
        CAMERA_TRIGGER p112 = CommunicationChannel.instance.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(1997056519269270819L);
        p112.seq_SET(4046847219L);
        CommunicationChannel.instance.send(p112); //===============================
        HIL_GPS p113 = CommunicationChannel.instance.new_HIL_GPS();
        PH.setPack(p113);
        p113.time_usec_SET(6018815068292978558L);
        p113.fix_type_SET((char)91);
        p113.lat_SET(-1309576576);
        p113.lon_SET(-1357140397);
        p113.alt_SET(1602440671);
        p113.eph_SET((char)11137);
        p113.epv_SET((char)42630);
        p113.vel_SET((char)3503);
        p113.vn_SET((short)7045);
        p113.ve_SET((short)3919);
        p113.vd_SET((short) -17504);
        p113.cog_SET((char)25229);
        p113.satellites_visible_SET((char)19);
        CommunicationChannel.instance.send(p113); //===============================
        HIL_OPTICAL_FLOW p114 = CommunicationChannel.instance.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.time_usec_SET(5521485158777078193L);
        p114.sensor_id_SET((char)124);
        p114.integration_time_us_SET(3122920544L);
        p114.integrated_x_SET(3.1805384E37F);
        p114.integrated_y_SET(2.8755283E38F);
        p114.integrated_xgyro_SET(5.2653085E37F);
        p114.integrated_ygyro_SET(-2.5219842E38F);
        p114.integrated_zgyro_SET(8.369234E37F);
        p114.temperature_SET((short)26540);
        p114.quality_SET((char)26);
        p114.time_delta_distance_us_SET(4186662523L);
        p114.distance_SET(9.332338E37F);
        CommunicationChannel.instance.send(p114); //===============================
        HIL_STATE_QUATERNION p115 = CommunicationChannel.instance.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.time_usec_SET(2342140633251245445L);
        p115.attitude_quaternion_SET(new float[4], 0);
        p115.rollspeed_SET(-2.0389069E38F);
        p115.pitchspeed_SET(7.1434647E37F);
        p115.yawspeed_SET(2.742615E38F);
        p115.lat_SET(685049952);
        p115.lon_SET(-1069778157);
        p115.alt_SET(1696896704);
        p115.vx_SET((short)11055);
        p115.vy_SET((short)17304);
        p115.vz_SET((short) -15330);
        p115.ind_airspeed_SET((char)26955);
        p115.true_airspeed_SET((char)45366);
        p115.xacc_SET((short) -25820);
        p115.yacc_SET((short) -23918);
        p115.zacc_SET((short)23858);
        CommunicationChannel.instance.send(p115); //===============================
        SCALED_IMU2 p116 = CommunicationChannel.instance.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.time_boot_ms_SET(852989810L);
        p116.xacc_SET((short) -7311);
        p116.yacc_SET((short) -28153);
        p116.zacc_SET((short) -20461);
        p116.xgyro_SET((short)13453);
        p116.ygyro_SET((short) -18238);
        p116.zgyro_SET((short) -26092);
        p116.xmag_SET((short) -31164);
        p116.ymag_SET((short)28707);
        p116.zmag_SET((short)4343);
        CommunicationChannel.instance.send(p116); //===============================
        LOG_REQUEST_LIST p117 = CommunicationChannel.instance.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)68);
        p117.target_component_SET((char)157);
        p117.start_SET((char)48156);
        p117.end_SET((char)39726);
        CommunicationChannel.instance.send(p117); //===============================
        LOG_ENTRY p118 = CommunicationChannel.instance.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)20736);
        p118.num_logs_SET((char)38896);
        p118.last_log_num_SET((char)39793);
        p118.time_utc_SET(3825907267L);
        p118.size_SET(3365550746L);
        CommunicationChannel.instance.send(p118); //===============================
        LOG_REQUEST_DATA p119 = CommunicationChannel.instance.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)26);
        p119.target_component_SET((char)25);
        p119.id_SET((char)18846);
        p119.ofs_SET(4084317872L);
        p119.count_SET(1086932887L);
        CommunicationChannel.instance.send(p119); //===============================
        LOG_DATA p120 = CommunicationChannel.instance.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)8185);
        p120.ofs_SET(2700545926L);
        p120.count_SET((char)21);
        p120.data__SET(new char[90], 0);
        CommunicationChannel.instance.send(p120); //===============================
        LOG_ERASE p121 = CommunicationChannel.instance.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)159);
        p121.target_component_SET((char)246);
        CommunicationChannel.instance.send(p121); //===============================
        LOG_REQUEST_END p122 = CommunicationChannel.instance.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)22);
        p122.target_component_SET((char)224);
        CommunicationChannel.instance.send(p122); //===============================
        GPS_INJECT_DATA p123 = CommunicationChannel.instance.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)50);
        p123.target_component_SET((char)24);
        p123.len_SET((char)187);
        p123.data__SET(new char[110], 0);
        CommunicationChannel.instance.send(p123); //===============================
        GPS2_RAW p124 = CommunicationChannel.instance.new_GPS2_RAW();
        PH.setPack(p124);
        p124.time_usec_SET(1908182484441896421L);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
        p124.lat_SET(199166346);
        p124.lon_SET(1570708065);
        p124.alt_SET(-616978741);
        p124.eph_SET((char)60389);
        p124.epv_SET((char)13740);
        p124.vel_SET((char)26664);
        p124.cog_SET((char)18653);
        p124.satellites_visible_SET((char)157);
        p124.dgps_numch_SET((char)167);
        p124.dgps_age_SET(1678328714L);
        CommunicationChannel.instance.send(p124); //===============================
        POWER_STATUS p125 = CommunicationChannel.instance.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)11959);
        p125.Vservo_SET((char)24897);
        p125.flags_SET((MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID));
        CommunicationChannel.instance.send(p125); //===============================
        SERIAL_CONTROL p126 = CommunicationChannel.instance.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_GPS1);
        p126.flags_SET((SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY));
        p126.timeout_SET((char)55859);
        p126.baudrate_SET(763838988L);
        p126.count_SET((char)89);
        p126.data__SET(new char[70], 0);
        CommunicationChannel.instance.send(p126); //===============================
        GPS_RTK p127 = CommunicationChannel.instance.new_GPS_RTK();
        PH.setPack(p127);
        p127.time_last_baseline_ms_SET(597454430L);
        p127.rtk_receiver_id_SET((char)183);
        p127.wn_SET((char)65385);
        p127.tow_SET(813313594L);
        p127.rtk_health_SET((char)216);
        p127.rtk_rate_SET((char)129);
        p127.nsats_SET((char)21);
        p127.baseline_coords_type_SET((char)197);
        p127.baseline_a_mm_SET(1367387249);
        p127.baseline_b_mm_SET(1544902444);
        p127.baseline_c_mm_SET(370526538);
        p127.accuracy_SET(2536610028L);
        p127.iar_num_hypotheses_SET(-1585663719);
        CommunicationChannel.instance.send(p127); //===============================
        GPS2_RTK p128 = CommunicationChannel.instance.new_GPS2_RTK();
        PH.setPack(p128);
        p128.time_last_baseline_ms_SET(3748039503L);
        p128.rtk_receiver_id_SET((char)143);
        p128.wn_SET((char)17734);
        p128.tow_SET(2817849057L);
        p128.rtk_health_SET((char)39);
        p128.rtk_rate_SET((char)100);
        p128.nsats_SET((char)237);
        p128.baseline_coords_type_SET((char)193);
        p128.baseline_a_mm_SET(-762238040);
        p128.baseline_b_mm_SET(255658218);
        p128.baseline_c_mm_SET(-1408274075);
        p128.accuracy_SET(2064206826L);
        p128.iar_num_hypotheses_SET(-1168930522);
        CommunicationChannel.instance.send(p128); //===============================
        SCALED_IMU3 p129 = CommunicationChannel.instance.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.time_boot_ms_SET(662288172L);
        p129.xacc_SET((short)20202);
        p129.yacc_SET((short) -8210);
        p129.zacc_SET((short)18821);
        p129.xgyro_SET((short)16011);
        p129.ygyro_SET((short)16905);
        p129.zgyro_SET((short) -18228);
        p129.xmag_SET((short) -15621);
        p129.ymag_SET((short) -1111);
        p129.zmag_SET((short)28391);
        CommunicationChannel.instance.send(p129); //===============================
        DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.instance.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.type_SET((char)200);
        p130.size_SET(1103031984L);
        p130.width_SET((char)8798);
        p130.height_SET((char)19095);
        p130.packets_SET((char)41820);
        p130.payload_SET((char)191);
        p130.jpg_quality_SET((char)13);
        CommunicationChannel.instance.send(p130); //===============================
        ENCAPSULATED_DATA p131 = CommunicationChannel.instance.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)42212);
        p131.data__SET(new char[253], 0);
        CommunicationChannel.instance.send(p131); //===============================
        DISTANCE_SENSOR p132 = CommunicationChannel.instance.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.time_boot_ms_SET(151898238L);
        p132.min_distance_SET((char)12480);
        p132.max_distance_SET((char)36223);
        p132.current_distance_SET((char)7509);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
        p132.id_SET((char)152);
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_PITCH_180_YAW_270);
        p132.covariance_SET((char)84);
        CommunicationChannel.instance.send(p132); //===============================
        TERRAIN_REQUEST p133 = CommunicationChannel.instance.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(1491901004);
        p133.lon_SET(-1425069248);
        p133.grid_spacing_SET((char)7018);
        p133.mask_SET(4791340499908812331L);
        CommunicationChannel.instance.send(p133); //===============================
        TERRAIN_DATA p134 = CommunicationChannel.instance.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lat_SET(550760871);
        p134.lon_SET(8153216);
        p134.grid_spacing_SET((char)42956);
        p134.gridbit_SET((char)75);
        p134.data__SET(new short[16], 0);
        CommunicationChannel.instance.send(p134); //===============================
        TERRAIN_CHECK p135 = CommunicationChannel.instance.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(-618171499);
        p135.lon_SET(1211690206);
        CommunicationChannel.instance.send(p135); //===============================
        TERRAIN_REPORT p136 = CommunicationChannel.instance.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lat_SET(907316122);
        p136.lon_SET(-1337692650);
        p136.spacing_SET((char)24364);
        p136.terrain_height_SET(3.5154438E37F);
        p136.current_height_SET(-2.1044438E38F);
        p136.pending_SET((char)24987);
        p136.loaded_SET((char)21785);
        CommunicationChannel.instance.send(p136); //===============================
        SCALED_PRESSURE2 p137 = CommunicationChannel.instance.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(1790328511L);
        p137.press_abs_SET(-3.345375E38F);
        p137.press_diff_SET(-2.941463E38F);
        p137.temperature_SET((short) -7439);
        CommunicationChannel.instance.send(p137); //===============================
        ATT_POS_MOCAP p138 = CommunicationChannel.instance.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(9125326858812918863L);
        p138.q_SET(new float[4], 0);
        p138.x_SET(-2.4023954E38F);
        p138.y_SET(-1.1419259E37F);
        p138.z_SET(7.3460536E36F);
        CommunicationChannel.instance.send(p138); //===============================
        SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.instance.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(4116829417421197375L);
        p139.group_mlx_SET((char)95);
        p139.target_system_SET((char)191);
        p139.target_component_SET((char)63);
        p139.controls_SET(new float[8], 0);
        CommunicationChannel.instance.send(p139); //===============================
        ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.instance.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(8703636354834405385L);
        p140.group_mlx_SET((char)114);
        p140.controls_SET(new float[8], 0);
        CommunicationChannel.instance.send(p140); //===============================
        ALTITUDE p141 = CommunicationChannel.instance.new_ALTITUDE();
        PH.setPack(p141);
        p141.time_usec_SET(5315801875939010717L);
        p141.altitude_monotonic_SET(-1.1597144E38F);
        p141.altitude_amsl_SET(3.2302952E38F);
        p141.altitude_local_SET(-8.0476094E37F);
        p141.altitude_relative_SET(3.1971183E38F);
        p141.altitude_terrain_SET(1.6950678E38F);
        p141.bottom_clearance_SET(-3.3048671E38F);
        CommunicationChannel.instance.send(p141); //===============================
        RESOURCE_REQUEST p142 = CommunicationChannel.instance.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.request_id_SET((char)13);
        p142.uri_type_SET((char)148);
        p142.uri_SET(new char[120], 0);
        p142.transfer_type_SET((char)209);
        p142.storage_SET(new char[120], 0);
        CommunicationChannel.instance.send(p142); //===============================
        SCALED_PRESSURE3 p143 = CommunicationChannel.instance.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(989435670L);
        p143.press_abs_SET(-1.8206714E38F);
        p143.press_diff_SET(-2.5199204E38F);
        p143.temperature_SET((short) -13032);
        CommunicationChannel.instance.send(p143); //===============================
        FOLLOW_TARGET p144 = CommunicationChannel.instance.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.timestamp_SET(989650139785226614L);
        p144.est_capabilities_SET((char)209);
        p144.lat_SET(-1916864000);
        p144.lon_SET(-616318781);
        p144.alt_SET(2.62394E38F);
        p144.vel_SET(new float[3], 0);
        p144.acc_SET(new float[3], 0);
        p144.attitude_q_SET(new float[4], 0);
        p144.rates_SET(new float[3], 0);
        p144.position_cov_SET(new float[3], 0);
        p144.custom_state_SET(93138867864364066L);
        CommunicationChannel.instance.send(p144); //===============================
        CONTROL_SYSTEM_STATE p146 = CommunicationChannel.instance.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.time_usec_SET(4268555814926780817L);
        p146.x_acc_SET(-2.4914066E38F);
        p146.y_acc_SET(1.2183706E38F);
        p146.z_acc_SET(-1.928925E38F);
        p146.x_vel_SET(-1.6733396E38F);
        p146.y_vel_SET(2.0173043E38F);
        p146.z_vel_SET(-3.249435E38F);
        p146.x_pos_SET(-2.5520963E38F);
        p146.y_pos_SET(-2.9670948E38F);
        p146.z_pos_SET(1.1201335E38F);
        p146.airspeed_SET(-1.1006575E38F);
        p146.vel_variance_SET(new float[3], 0);
        p146.pos_variance_SET(new float[3], 0);
        p146.q_SET(new float[4], 0);
        p146.roll_rate_SET(-1.1751178E38F);
        p146.pitch_rate_SET(-1.3758886E38F);
        p146.yaw_rate_SET(-3.6222982E37F);
        CommunicationChannel.instance.send(p146); //===============================
        BATTERY_STATUS p147 = CommunicationChannel.instance.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.id_SET((char)159);
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_ALL);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_NIMH);
        p147.temperature_SET((short) -10552);
        p147.voltages_SET(new char[10], 0);
        p147.current_battery_SET((short)28851);
        p147.current_consumed_SET(-2100872042);
        p147.energy_consumed_SET(-181689431);
        p147.battery_remaining_SET((byte) - 9);
        CommunicationChannel.instance.send(p147); //===============================
        AUTOPILOT_VERSION p148 = CommunicationChannel.instance.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.capabilities_SET((MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_UNION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                               MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE));
        p148.flight_sw_version_SET(486981256L);
        p148.middleware_sw_version_SET(320615135L);
        p148.os_sw_version_SET(4022030895L);
        p148.board_version_SET(4135625600L);
        p148.flight_custom_version_SET(new char[8], 0);
        p148.middleware_custom_version_SET(new char[8], 0);
        p148.os_custom_version_SET(new char[8], 0);
        p148.vendor_id_SET((char)4224);
        p148.product_id_SET((char)36016);
        p148.uid_SET(6718556733330423814L);
        p148.uid2_SET(new char[18], 0, PH);
        CommunicationChannel.instance.send(p148); //===============================
        LANDING_TARGET p149 = CommunicationChannel.instance.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.time_usec_SET(3262522667718709829L);
        p149.target_num_SET((char)4);
        p149.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
        p149.angle_x_SET(1.2004586E38F);
        p149.angle_y_SET(-2.9170565E38F);
        p149.distance_SET(2.161003E38F);
        p149.size_x_SET(-3.3135324E38F);
        p149.size_y_SET(-1.0316057E38F);
        p149.x_SET(1.4016392E38F, PH);
        p149.y_SET(-3.3575262E38F, PH);
        p149.z_SET(2.8769235E38F, PH);
        p149.q_SET(new float[4], 0, PH);
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL);
        p149.position_valid_SET((char)72, PH);
        CommunicationChannel.instance.send(p149); //===============================
        FLEXIFUNCTION_SET p150 = CommunicationChannel.instance.new_FLEXIFUNCTION_SET();
        PH.setPack(p150);
        p150.target_system_SET((char)145);
        p150.target_component_SET((char)20);
        CommunicationChannel.instance.send(p150); //===============================
        FLEXIFUNCTION_READ_REQ p151 = CommunicationChannel.instance.new_FLEXIFUNCTION_READ_REQ();
        PH.setPack(p151);
        p151.target_system_SET((char)254);
        p151.target_component_SET((char)81);
        p151.read_req_type_SET((short) -6236);
        p151.data_index_SET((short) -4139);
        CommunicationChannel.instance.send(p151); //===============================
        FLEXIFUNCTION_BUFFER_FUNCTION p152 = CommunicationChannel.instance.new_FLEXIFUNCTION_BUFFER_FUNCTION();
        PH.setPack(p152);
        p152.target_system_SET((char)231);
        p152.target_component_SET((char)251);
        p152.func_index_SET((char)56574);
        p152.func_count_SET((char)10243);
        p152.data_address_SET((char)43348);
        p152.data_size_SET((char)56357);
        p152.data__SET(new byte[48], 0);
        CommunicationChannel.instance.send(p152); //===============================
        FLEXIFUNCTION_BUFFER_FUNCTION_ACK p153 = CommunicationChannel.instance.new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK();
        PH.setPack(p153);
        p153.target_system_SET((char)144);
        p153.target_component_SET((char)165);
        p153.func_index_SET((char)34478);
        p153.result_SET((char)40273);
        CommunicationChannel.instance.send(p153); //===============================
        FLEXIFUNCTION_DIRECTORY p155 = CommunicationChannel.instance.new_FLEXIFUNCTION_DIRECTORY();
        PH.setPack(p155);
        p155.target_system_SET((char)119);
        p155.target_component_SET((char)122);
        p155.directory_type_SET((char)89);
        p155.start_index_SET((char)107);
        p155.count_SET((char)25);
        p155.directory_data_SET(new byte[48], 0);
        CommunicationChannel.instance.send(p155); //===============================
        FLEXIFUNCTION_DIRECTORY_ACK p156 = CommunicationChannel.instance.new_FLEXIFUNCTION_DIRECTORY_ACK();
        PH.setPack(p156);
        p156.target_system_SET((char)21);
        p156.target_component_SET((char)173);
        p156.directory_type_SET((char)97);
        p156.start_index_SET((char)71);
        p156.count_SET((char)162);
        p156.result_SET((char)54192);
        CommunicationChannel.instance.send(p156); //===============================
        FLEXIFUNCTION_COMMAND p157 = CommunicationChannel.instance.new_FLEXIFUNCTION_COMMAND();
        PH.setPack(p157);
        p157.target_system_SET((char)110);
        p157.target_component_SET((char)203);
        p157.command_type_SET((char)223);
        CommunicationChannel.instance.send(p157); //===============================
        FLEXIFUNCTION_COMMAND_ACK p158 = CommunicationChannel.instance.new_FLEXIFUNCTION_COMMAND_ACK();
        PH.setPack(p158);
        p158.command_type_SET((char)5038);
        p158.result_SET((char)34987);
        CommunicationChannel.instance.send(p158); //===============================
        SERIAL_UDB_EXTRA_F2_A p170 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F2_A();
        PH.setPack(p170);
        p170.sue_time_SET(1961497739L);
        p170.sue_status_SET((char)19);
        p170.sue_latitude_SET(-1492389818);
        p170.sue_longitude_SET(-560511644);
        p170.sue_altitude_SET(-1542282176);
        p170.sue_waypoint_index_SET((char)58785);
        p170.sue_rmat0_SET((short)12290);
        p170.sue_rmat1_SET((short) -18519);
        p170.sue_rmat2_SET((short) -31884);
        p170.sue_rmat3_SET((short)16756);
        p170.sue_rmat4_SET((short)22014);
        p170.sue_rmat5_SET((short)30449);
        p170.sue_rmat6_SET((short) -14201);
        p170.sue_rmat7_SET((short)19146);
        p170.sue_rmat8_SET((short) -32755);
        p170.sue_cog_SET((char)37410);
        p170.sue_sog_SET((short) -11987);
        p170.sue_cpu_load_SET((char)31582);
        p170.sue_air_speed_3DIMU_SET((char)53914);
        p170.sue_estimated_wind_0_SET((short) -22181);
        p170.sue_estimated_wind_1_SET((short) -28236);
        p170.sue_estimated_wind_2_SET((short)31931);
        p170.sue_magFieldEarth0_SET((short)2488);
        p170.sue_magFieldEarth1_SET((short)5058);
        p170.sue_magFieldEarth2_SET((short) -3494);
        p170.sue_svs_SET((short)10828);
        p170.sue_hdop_SET((short) -27082);
        CommunicationChannel.instance.send(p170); //===============================
        SERIAL_UDB_EXTRA_F2_B p171 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F2_B();
        PH.setPack(p171);
        p171.sue_time_SET(3845614337L);
        p171.sue_pwm_input_1_SET((short) -8944);
        p171.sue_pwm_input_2_SET((short) -29733);
        p171.sue_pwm_input_3_SET((short)14351);
        p171.sue_pwm_input_4_SET((short) -9576);
        p171.sue_pwm_input_5_SET((short) -15892);
        p171.sue_pwm_input_6_SET((short)18943);
        p171.sue_pwm_input_7_SET((short)11221);
        p171.sue_pwm_input_8_SET((short) -19550);
        p171.sue_pwm_input_9_SET((short) -21743);
        p171.sue_pwm_input_10_SET((short)6168);
        p171.sue_pwm_input_11_SET((short) -5080);
        p171.sue_pwm_input_12_SET((short)13540);
        p171.sue_pwm_output_1_SET((short) -29493);
        p171.sue_pwm_output_2_SET((short) -10836);
        p171.sue_pwm_output_3_SET((short)2568);
        p171.sue_pwm_output_4_SET((short) -28091);
        p171.sue_pwm_output_5_SET((short)16648);
        p171.sue_pwm_output_6_SET((short) -579);
        p171.sue_pwm_output_7_SET((short) -14027);
        p171.sue_pwm_output_8_SET((short)15104);
        p171.sue_pwm_output_9_SET((short) -31731);
        p171.sue_pwm_output_10_SET((short) -19413);
        p171.sue_pwm_output_11_SET((short)12863);
        p171.sue_pwm_output_12_SET((short) -22948);
        p171.sue_imu_location_x_SET((short)28986);
        p171.sue_imu_location_y_SET((short)32390);
        p171.sue_imu_location_z_SET((short)17245);
        p171.sue_location_error_earth_x_SET((short)9793);
        p171.sue_location_error_earth_y_SET((short)25334);
        p171.sue_location_error_earth_z_SET((short)22905);
        p171.sue_flags_SET(918975329L);
        p171.sue_osc_fails_SET((short)24665);
        p171.sue_imu_velocity_x_SET((short)7646);
        p171.sue_imu_velocity_y_SET((short)6157);
        p171.sue_imu_velocity_z_SET((short)21582);
        p171.sue_waypoint_goal_x_SET((short)28593);
        p171.sue_waypoint_goal_y_SET((short)12754);
        p171.sue_waypoint_goal_z_SET((short)23300);
        p171.sue_aero_x_SET((short)16886);
        p171.sue_aero_y_SET((short)15378);
        p171.sue_aero_z_SET((short)2407);
        p171.sue_barom_temp_SET((short)7662);
        p171.sue_barom_press_SET(1038508767);
        p171.sue_barom_alt_SET(-464139116);
        p171.sue_bat_volt_SET((short) -5369);
        p171.sue_bat_amp_SET((short)17937);
        p171.sue_bat_amp_hours_SET((short) -28523);
        p171.sue_desired_height_SET((short)9204);
        p171.sue_memory_stack_free_SET((short)3760);
        CommunicationChannel.instance.send(p171); //===============================
        SERIAL_UDB_EXTRA_F4 p172 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F4();
        PH.setPack(p172);
        p172.sue_ROLL_STABILIZATION_AILERONS_SET((char)245);
        p172.sue_ROLL_STABILIZATION_RUDDER_SET((char)88);
        p172.sue_PITCH_STABILIZATION_SET((char)78);
        p172.sue_YAW_STABILIZATION_RUDDER_SET((char)32);
        p172.sue_YAW_STABILIZATION_AILERON_SET((char)112);
        p172.sue_AILERON_NAVIGATION_SET((char)12);
        p172.sue_RUDDER_NAVIGATION_SET((char)190);
        p172.sue_ALTITUDEHOLD_STABILIZED_SET((char)176);
        p172.sue_ALTITUDEHOLD_WAYPOINT_SET((char)125);
        p172.sue_RACING_MODE_SET((char)4);
        CommunicationChannel.instance.send(p172); //===============================
        SERIAL_UDB_EXTRA_F5 p173 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F5();
        PH.setPack(p173);
        p173.sue_YAWKP_AILERON_SET(2.8001664E37F);
        p173.sue_YAWKD_AILERON_SET(-1.602184E38F);
        p173.sue_ROLLKP_SET(1.4332204E38F);
        p173.sue_ROLLKD_SET(8.587762E37F);
        CommunicationChannel.instance.send(p173); //===============================
        SERIAL_UDB_EXTRA_F6 p174 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F6();
        PH.setPack(p174);
        p174.sue_PITCHGAIN_SET(-1.425006E38F);
        p174.sue_PITCHKD_SET(1.910446E38F);
        p174.sue_RUDDER_ELEV_MIX_SET(-1.7744516E38F);
        p174.sue_ROLL_ELEV_MIX_SET(-1.3244043E38F);
        p174.sue_ELEVATOR_BOOST_SET(-2.262058E37F);
        CommunicationChannel.instance.send(p174); //===============================
        SERIAL_UDB_EXTRA_F7 p175 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F7();
        PH.setPack(p175);
        p175.sue_YAWKP_RUDDER_SET(3.3442504E37F);
        p175.sue_YAWKD_RUDDER_SET(-1.9406917E38F);
        p175.sue_ROLLKP_RUDDER_SET(1.7884092E38F);
        p175.sue_ROLLKD_RUDDER_SET(4.2132153E37F);
        p175.sue_RUDDER_BOOST_SET(-1.368733E38F);
        p175.sue_RTL_PITCH_DOWN_SET(2.9055639E38F);
        CommunicationChannel.instance.send(p175); //===============================
        SERIAL_UDB_EXTRA_F8 p176 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F8();
        PH.setPack(p176);
        p176.sue_HEIGHT_TARGET_MAX_SET(2.0188328E38F);
        p176.sue_HEIGHT_TARGET_MIN_SET(-2.9776084E38F);
        p176.sue_ALT_HOLD_THROTTLE_MIN_SET(7.2444675E37F);
        p176.sue_ALT_HOLD_THROTTLE_MAX_SET(1.8019832E38F);
        p176.sue_ALT_HOLD_PITCH_MIN_SET(-7.120413E37F);
        p176.sue_ALT_HOLD_PITCH_MAX_SET(-1.7471197E38F);
        p176.sue_ALT_HOLD_PITCH_HIGH_SET(6.368308E37F);
        CommunicationChannel.instance.send(p176); //===============================
        SERIAL_UDB_EXTRA_F13 p177 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F13();
        PH.setPack(p177);
        p177.sue_week_no_SET((short)182);
        p177.sue_lat_origin_SET(1210320875);
        p177.sue_lon_origin_SET(1909954112);
        p177.sue_alt_origin_SET(-535892527);
        CommunicationChannel.instance.send(p177); //===============================
        SERIAL_UDB_EXTRA_F14 p178 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F14();
        PH.setPack(p178);
        p178.sue_WIND_ESTIMATION_SET((char)7);
        p178.sue_GPS_TYPE_SET((char)8);
        p178.sue_DR_SET((char)79);
        p178.sue_BOARD_TYPE_SET((char)83);
        p178.sue_AIRFRAME_SET((char)214);
        p178.sue_RCON_SET((short) -21857);
        p178.sue_TRAP_FLAGS_SET((short) -6526);
        p178.sue_TRAP_SOURCE_SET(2414920917L);
        p178.sue_osc_fail_count_SET((short)19137);
        p178.sue_CLOCK_CONFIG_SET((char)54);
        p178.sue_FLIGHT_PLAN_TYPE_SET((char)176);
        CommunicationChannel.instance.send(p178); //===============================
        SERIAL_UDB_EXTRA_F15 p179 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F15();
        PH.setPack(p179);
        p179.sue_ID_VEHICLE_MODEL_NAME_SET(new char[40], 0);
        p179.sue_ID_VEHICLE_REGISTRATION_SET(new char[20], 0);
        CommunicationChannel.instance.send(p179); //===============================
        SERIAL_UDB_EXTRA_F16 p180 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F16();
        PH.setPack(p180);
        p180.sue_ID_LEAD_PILOT_SET(new char[40], 0);
        p180.sue_ID_DIY_DRONES_URL_SET(new char[70], 0);
        CommunicationChannel.instance.send(p180); //===============================
        ALTITUDES p181 = CommunicationChannel.instance.new_ALTITUDES();
        PH.setPack(p181);
        p181.time_boot_ms_SET(1597787339L);
        p181.alt_gps_SET(377102197);
        p181.alt_imu_SET(-1822540740);
        p181.alt_barometric_SET(-743623424);
        p181.alt_optical_flow_SET(1354205468);
        p181.alt_range_finder_SET(1032446003);
        p181.alt_extra_SET(650202009);
        CommunicationChannel.instance.send(p181); //===============================
        AIRSPEEDS p182 = CommunicationChannel.instance.new_AIRSPEEDS();
        PH.setPack(p182);
        p182.time_boot_ms_SET(515153796L);
        p182.airspeed_imu_SET((short)4786);
        p182.airspeed_pitot_SET((short)31850);
        p182.airspeed_hot_wire_SET((short) -14981);
        p182.airspeed_ultrasonic_SET((short) -3374);
        p182.aoa_SET((short) -16556);
        p182.aoy_SET((short) -6817);
        CommunicationChannel.instance.send(p182); //===============================
        SERIAL_UDB_EXTRA_F17 p183 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F17();
        PH.setPack(p183);
        p183.sue_feed_forward_SET(-1.9459084E38F);
        p183.sue_turn_rate_nav_SET(2.9320174E38F);
        p183.sue_turn_rate_fbw_SET(-1.59548E38F);
        CommunicationChannel.instance.send(p183); //===============================
        SERIAL_UDB_EXTRA_F18 p184 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F18();
        PH.setPack(p184);
        p184.angle_of_attack_normal_SET(-5.7498E37F);
        p184.angle_of_attack_inverted_SET(-1.7214657E38F);
        p184.elevator_trim_normal_SET(2.0470912E38F);
        p184.elevator_trim_inverted_SET(4.956201E37F);
        p184.reference_speed_SET(-2.0470125E38F);
        CommunicationChannel.instance.send(p184); //===============================
        SERIAL_UDB_EXTRA_F19 p185 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F19();
        PH.setPack(p185);
        p185.sue_aileron_output_channel_SET((char)189);
        p185.sue_aileron_reversed_SET((char)112);
        p185.sue_elevator_output_channel_SET((char)213);
        p185.sue_elevator_reversed_SET((char)3);
        p185.sue_throttle_output_channel_SET((char)54);
        p185.sue_throttle_reversed_SET((char)52);
        p185.sue_rudder_output_channel_SET((char)60);
        p185.sue_rudder_reversed_SET((char)152);
        CommunicationChannel.instance.send(p185); //===============================
        SERIAL_UDB_EXTRA_F20 p186 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F20();
        PH.setPack(p186);
        p186.sue_number_of_inputs_SET((char)224);
        p186.sue_trim_value_input_1_SET((short)4609);
        p186.sue_trim_value_input_2_SET((short) -25940);
        p186.sue_trim_value_input_3_SET((short)3338);
        p186.sue_trim_value_input_4_SET((short)1127);
        p186.sue_trim_value_input_5_SET((short)18024);
        p186.sue_trim_value_input_6_SET((short)21260);
        p186.sue_trim_value_input_7_SET((short) -7399);
        p186.sue_trim_value_input_8_SET((short)30423);
        p186.sue_trim_value_input_9_SET((short) -2540);
        p186.sue_trim_value_input_10_SET((short)3120);
        p186.sue_trim_value_input_11_SET((short)31316);
        p186.sue_trim_value_input_12_SET((short) -1200);
        CommunicationChannel.instance.send(p186); //===============================
        SERIAL_UDB_EXTRA_F21 p187 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F21();
        PH.setPack(p187);
        p187.sue_accel_x_offset_SET((short) -23336);
        p187.sue_accel_y_offset_SET((short) -26579);
        p187.sue_accel_z_offset_SET((short)26538);
        p187.sue_gyro_x_offset_SET((short)29969);
        p187.sue_gyro_y_offset_SET((short) -23207);
        p187.sue_gyro_z_offset_SET((short)7467);
        CommunicationChannel.instance.send(p187); //===============================
        SERIAL_UDB_EXTRA_F22 p188 = CommunicationChannel.instance.new_SERIAL_UDB_EXTRA_F22();
        PH.setPack(p188);
        p188.sue_accel_x_at_calibration_SET((short)19999);
        p188.sue_accel_y_at_calibration_SET((short)9269);
        p188.sue_accel_z_at_calibration_SET((short) -15586);
        p188.sue_gyro_x_at_calibration_SET((short)3556);
        p188.sue_gyro_y_at_calibration_SET((short) -16419);
        p188.sue_gyro_z_at_calibration_SET((short)14455);
        CommunicationChannel.instance.send(p188); //===============================
        ESTIMATOR_STATUS p230 = CommunicationChannel.instance.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.time_usec_SET(8286933100749105719L);
        p230.flags_SET((ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_VERT |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_REL |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_HORIZ_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_PRED_POS_HORIZ_ABS |
                        ESTIMATOR_STATUS_FLAGS.ESTIMATOR_ATTITUDE));
        p230.vel_ratio_SET(-2.5406253E38F);
        p230.pos_horiz_ratio_SET(-8.999118E37F);
        p230.pos_vert_ratio_SET(3.3113105E38F);
        p230.mag_ratio_SET(2.4804046E38F);
        p230.hagl_ratio_SET(-2.3292741E38F);
        p230.tas_ratio_SET(-4.625697E37F);
        p230.pos_horiz_accuracy_SET(3.1102102E38F);
        p230.pos_vert_accuracy_SET(-1.9239888E38F);
        CommunicationChannel.instance.send(p230); //===============================
        WIND_COV p231 = CommunicationChannel.instance.new_WIND_COV();
        PH.setPack(p231);
        p231.time_usec_SET(6929469276355952290L);
        p231.wind_x_SET(1.0120249E38F);
        p231.wind_y_SET(-4.545382E37F);
        p231.wind_z_SET(-4.549671E37F);
        p231.var_horiz_SET(-1.7259552E38F);
        p231.var_vert_SET(-4.887033E37F);
        p231.wind_alt_SET(2.8695454E38F);
        p231.horiz_accuracy_SET(2.7449664E38F);
        p231.vert_accuracy_SET(3.6208685E37F);
        CommunicationChannel.instance.send(p231); //===============================
        GPS_INPUT p232 = CommunicationChannel.instance.new_GPS_INPUT();
        PH.setPack(p232);
        p232.time_usec_SET(2672610673708147462L);
        p232.gps_id_SET((char)46);
        p232.ignore_flags_SET((GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_ALT |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                               GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY));
        p232.time_week_ms_SET(1518146679L);
        p232.time_week_SET((char)41137);
        p232.fix_type_SET((char)33);
        p232.lat_SET(-874700150);
        p232.lon_SET(665420518);
        p232.alt_SET(3.3480723E38F);
        p232.hdop_SET(-1.1984724E38F);
        p232.vdop_SET(9.53826E36F);
        p232.vn_SET(2.060181E38F);
        p232.ve_SET(-2.5787595E38F);
        p232.vd_SET(2.7177591E38F);
        p232.speed_accuracy_SET(-2.5839467E38F);
        p232.horiz_accuracy_SET(2.6972654E37F);
        p232.vert_accuracy_SET(2.8968061E38F);
        p232.satellites_visible_SET((char)28);
        CommunicationChannel.instance.send(p232); //===============================
        GPS_RTCM_DATA p233 = CommunicationChannel.instance.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)43);
        p233.len_SET((char)133);
        p233.data__SET(new char[180], 0);
        CommunicationChannel.instance.send(p233); //===============================
        HIGH_LATENCY p234 = CommunicationChannel.instance.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.base_mode_SET((MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED));
        p234.custom_mode_SET(1566661084L);
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
        p234.roll_SET((short) -11003);
        p234.pitch_SET((short)14945);
        p234.heading_SET((char)10185);
        p234.throttle_SET((byte)100);
        p234.heading_sp_SET((short) -27891);
        p234.latitude_SET(-706512153);
        p234.longitude_SET(-278328244);
        p234.altitude_amsl_SET((short) -26414);
        p234.altitude_sp_SET((short)4500);
        p234.airspeed_SET((char)177);
        p234.airspeed_sp_SET((char)105);
        p234.groundspeed_SET((char)95);
        p234.climb_rate_SET((byte)111);
        p234.gps_nsat_SET((char)122);
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
        p234.battery_remaining_SET((char)36);
        p234.temperature_SET((byte)18);
        p234.temperature_air_SET((byte)78);
        p234.failsafe_SET((char)55);
        p234.wp_num_SET((char)203);
        p234.wp_distance_SET((char)19251);
        CommunicationChannel.instance.send(p234); //===============================
        VIBRATION p241 = CommunicationChannel.instance.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(7628837586281625346L);
        p241.vibration_x_SET(1.3515736E37F);
        p241.vibration_y_SET(-3.2555582E38F);
        p241.vibration_z_SET(1.8971065E38F);
        p241.clipping_0_SET(168467972L);
        p241.clipping_1_SET(2763053139L);
        p241.clipping_2_SET(2957368890L);
        CommunicationChannel.instance.send(p241); //===============================
        HOME_POSITION p242 = CommunicationChannel.instance.new_HOME_POSITION();
        PH.setPack(p242);
        p242.latitude_SET(1622381362);
        p242.longitude_SET(-2104408978);
        p242.altitude_SET(-1340023288);
        p242.x_SET(1.7635667E38F);
        p242.y_SET(-2.3319782E38F);
        p242.z_SET(-1.950184E38F);
        p242.q_SET(new float[4], 0);
        p242.approach_x_SET(4.0670777E37F);
        p242.approach_y_SET(-4.0113353E37F);
        p242.approach_z_SET(-1.8967383E38F);
        p242.time_usec_SET(2639081355282965543L, PH);
        CommunicationChannel.instance.send(p242); //===============================
        SET_HOME_POSITION p243 = CommunicationChannel.instance.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.target_system_SET((char)154);
        p243.latitude_SET(1175285077);
        p243.longitude_SET(-1692312251);
        p243.altitude_SET(1014843309);
        p243.x_SET(2.4221556E38F);
        p243.y_SET(1.4875886E38F);
        p243.z_SET(-1.0552775E38F);
        p243.q_SET(new float[4], 0);
        p243.approach_x_SET(-1.88749E38F);
        p243.approach_y_SET(-6.0835683E37F);
        p243.approach_z_SET(-2.0404098E38F);
        p243.time_usec_SET(1208348629814445755L, PH);
        CommunicationChannel.instance.send(p243); //===============================
        MESSAGE_INTERVAL p244 = CommunicationChannel.instance.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)20022);
        p244.interval_us_SET(-565104021);
        CommunicationChannel.instance.send(p244); //===============================
        EXTENDED_SYS_STATE p245 = CommunicationChannel.instance.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
        CommunicationChannel.instance.send(p245); //===============================
        ADSB_VEHICLE p246 = CommunicationChannel.instance.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.ICAO_address_SET(3918333960L);
        p246.lat_SET(-1626748001);
        p246.lon_SET(-996473881);
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
        p246.altitude_SET(-1132902700);
        p246.heading_SET((char)52469);
        p246.hor_velocity_SET((char)51273);
        p246.ver_velocity_SET((short)14837);
        p246.callsign_SET("DEMO", PH);
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_UNASSIGNED);
        p246.tslc_SET((char)92);
        p246.flags_SET((ADSB_FLAGS.ADSB_FLAGS_VALID_CALLSIGN));
        p246.squawk_SET((char)40824);
        CommunicationChannel.instance.send(p246); //===============================
        COLLISION p247 = CommunicationChannel.instance.new_COLLISION();
        PH.setPack(p247);
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
        p247.id_SET(887501279L);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT);
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
        p247.time_to_minimum_delta_SET(1.6935243E38F);
        p247.altitude_minimum_delta_SET(2.183697E38F);
        p247.horizontal_minimum_delta_SET(-4.6182514E37F);
        CommunicationChannel.instance.send(p247); //===============================
        V2_EXTENSION p248 = CommunicationChannel.instance.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)165);
        p248.target_system_SET((char)55);
        p248.target_component_SET((char)133);
        p248.message_type_SET((char)31795);
        p248.payload_SET(new char[249], 0);
        CommunicationChannel.instance.send(p248); //===============================
        MEMORY_VECT p249 = CommunicationChannel.instance.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)10970);
        p249.ver_SET((char)145);
        p249.type_SET((char)191);
        p249.value_SET(new byte[32], 0);
        CommunicationChannel.instance.send(p249); //===============================
        DEBUG_VECT p250 = CommunicationChannel.instance.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("DEMO", PH);
        p250.time_usec_SET(4259388065835490570L);
        p250.x_SET(-1.9853534E38F);
        p250.y_SET(-1.7825153E37F);
        p250.z_SET(3.1451584E38F);
        CommunicationChannel.instance.send(p250); //===============================
        NAMED_VALUE_FLOAT p251 = CommunicationChannel.instance.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(2291290713L);
        p251.name_SET("DEMO", PH);
        p251.value_SET(3.2384869E38F);
        CommunicationChannel.instance.send(p251); //===============================
        NAMED_VALUE_INT p252 = CommunicationChannel.instance.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(934410436L);
        p252.name_SET("DEMO", PH);
        p252.value_SET(-1160859751);
        CommunicationChannel.instance.send(p252); //===============================
        STATUSTEXT p253 = CommunicationChannel.instance.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_DEBUG);
        p253.text_SET("DEMO", PH);
        CommunicationChannel.instance.send(p253); //===============================
        DEBUG p254 = CommunicationChannel.instance.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(3161087676L);
        p254.ind_SET((char)222);
        p254.value_SET(2.9871326E38F);
        CommunicationChannel.instance.send(p254); //===============================
        SETUP_SIGNING p256 = CommunicationChannel.instance.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)216);
        p256.target_component_SET((char)132);
        p256.secret_key_SET(new char[32], 0);
        p256.initial_timestamp_SET(7290830969393907490L);
        CommunicationChannel.instance.send(p256); //===============================
        BUTTON_CHANGE p257 = CommunicationChannel.instance.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(3684194377L);
        p257.last_change_ms_SET(3423822742L);
        p257.state_SET((char)143);
        CommunicationChannel.instance.send(p257); //===============================
        PLAY_TUNE p258 = CommunicationChannel.instance.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)1);
        p258.target_component_SET((char)181);
        p258.tune_SET("DEMO", PH);
        CommunicationChannel.instance.send(p258); //===============================
        CAMERA_INFORMATION p259 = CommunicationChannel.instance.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.time_boot_ms_SET(456276017L);
        p259.vendor_name_SET(new char[32], 0);
        p259.model_name_SET(new char[32], 0);
        p259.firmware_version_SET(1464635683L);
        p259.focal_length_SET(-2.777643E38F);
        p259.sensor_size_h_SET(2.6533132E38F);
        p259.sensor_size_v_SET(-3.0418668E38F);
        p259.resolution_h_SET((char)29937);
        p259.resolution_v_SET((char)56249);
        p259.lens_id_SET((char)196);
        p259.flags_SET((CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                        CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE));
        p259.cam_definition_version_SET((char)3086);
        p259.cam_definition_uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p259); //===============================
        CAMERA_SETTINGS p260 = CommunicationChannel.instance.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(1067554981L);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE_SURVEY);
        CommunicationChannel.instance.send(p260); //===============================
        STORAGE_INFORMATION p261 = CommunicationChannel.instance.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.time_boot_ms_SET(3003153701L);
        p261.storage_id_SET((char)29);
        p261.storage_count_SET((char)124);
        p261.status_SET((char)177);
        p261.total_capacity_SET(6.3914624E37F);
        p261.used_capacity_SET(-6.542238E37F);
        p261.available_capacity_SET(3.1110478E38F);
        p261.read_speed_SET(-3.3931713E38F);
        p261.write_speed_SET(-2.1461118E38F);
        CommunicationChannel.instance.send(p261); //===============================
        CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.instance.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.time_boot_ms_SET(1346449593L);
        p262.image_status_SET((char)191);
        p262.video_status_SET((char)126);
        p262.image_interval_SET(-1.7423612E38F);
        p262.recording_time_ms_SET(149104380L);
        p262.available_capacity_SET(-3.1337158E38F);
        CommunicationChannel.instance.send(p262); //===============================
        CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.instance.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.time_boot_ms_SET(3016730577L);
        p263.time_utc_SET(8422358278910310794L);
        p263.camera_id_SET((char)241);
        p263.lat_SET(1160122562);
        p263.lon_SET(1249673770);
        p263.alt_SET(1176575004);
        p263.relative_alt_SET(89508958);
        p263.q_SET(new float[4], 0);
        p263.image_index_SET(-2059690268);
        p263.capture_result_SET((byte)51);
        p263.file_url_SET("DEMO", PH);
        CommunicationChannel.instance.send(p263); //===============================
        FLIGHT_INFORMATION p264 = CommunicationChannel.instance.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.time_boot_ms_SET(1364443947L);
        p264.arming_time_utc_SET(6331398249208429278L);
        p264.takeoff_time_utc_SET(313393642617056958L);
        p264.flight_uuid_SET(7884174932400450142L);
        CommunicationChannel.instance.send(p264); //===============================
        MOUNT_ORIENTATION p265 = CommunicationChannel.instance.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(3898511567L);
        p265.roll_SET(1.7838308E38F);
        p265.pitch_SET(-3.2925748E38F);
        p265.yaw_SET(2.0697593E38F);
        CommunicationChannel.instance.send(p265); //===============================
        LOGGING_DATA p266 = CommunicationChannel.instance.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_system_SET((char)102);
        p266.target_component_SET((char)202);
        p266.sequence_SET((char)53217);
        p266.length_SET((char)229);
        p266.first_message_offset_SET((char)117);
        p266.data__SET(new char[249], 0);
        CommunicationChannel.instance.send(p266); //===============================
        LOGGING_DATA_ACKED p267 = CommunicationChannel.instance.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_system_SET((char)148);
        p267.target_component_SET((char)31);
        p267.sequence_SET((char)28432);
        p267.length_SET((char)5);
        p267.first_message_offset_SET((char)135);
        p267.data__SET(new char[249], 0);
        CommunicationChannel.instance.send(p267); //===============================
        LOGGING_ACK p268 = CommunicationChannel.instance.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)43);
        p268.target_component_SET((char)19);
        p268.sequence_SET((char)22411);
        CommunicationChannel.instance.send(p268); //===============================
        VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.instance.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.camera_id_SET((char)217);
        p269.status_SET((char)77);
        p269.framerate_SET(5.794724E37F);
        p269.resolution_h_SET((char)48309);
        p269.resolution_v_SET((char)29053);
        p269.bitrate_SET(940281392L);
        p269.rotation_SET((char)13667);
        p269.uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p269); //===============================
        SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.instance.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_system_SET((char)230);
        p270.target_component_SET((char)90);
        p270.camera_id_SET((char)173);
        p270.framerate_SET(-1.3480775E38F);
        p270.resolution_h_SET((char)34707);
        p270.resolution_v_SET((char)17936);
        p270.bitrate_SET(1771148947L);
        p270.rotation_SET((char)9374);
        p270.uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p270); //===============================
        WIFI_CONFIG_AP p299 = CommunicationChannel.instance.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("DEMO", PH);
        p299.password_SET("DEMO", PH);
        CommunicationChannel.instance.send(p299); //===============================
        PROTOCOL_VERSION p300 = CommunicationChannel.instance.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)24122);
        p300.min_version_SET((char)24852);
        p300.max_version_SET((char)50736);
        p300.spec_version_hash_SET(new char[8], 0);
        p300.library_version_hash_SET(new char[8], 0);
        CommunicationChannel.instance.send(p300); //===============================
        UAVCAN_NODE_STATUS p310 = CommunicationChannel.instance.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.time_usec_SET(4977095292360382261L);
        p310.uptime_sec_SET(1582126242L);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
        p310.sub_mode_SET((char)60);
        p310.vendor_specific_status_code_SET((char)11471);
        CommunicationChannel.instance.send(p310); //===============================
        UAVCAN_NODE_INFO p311 = CommunicationChannel.instance.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.time_usec_SET(1076167131761422708L);
        p311.uptime_sec_SET(143753266L);
        p311.name_SET("DEMO", PH);
        p311.hw_version_major_SET((char)75);
        p311.hw_version_minor_SET((char)101);
        p311.hw_unique_id_SET(new char[16], 0);
        p311.sw_version_major_SET((char)3);
        p311.sw_version_minor_SET((char)9);
        p311.sw_vcs_commit_SET(264147615L);
        CommunicationChannel.instance.send(p311); //===============================
        PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)136);
        p320.target_component_SET((char)215);
        p320.param_id_SET("DEMO", PH);
        p320.param_index_SET((short) -28475);
        CommunicationChannel.instance.send(p320); //===============================
        PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)101);
        p321.target_component_SET((char)103);
        CommunicationChannel.instance.send(p321); //===============================
        PARAM_EXT_VALUE p322 = CommunicationChannel.instance.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("DEMO", PH);
        p322.param_value_SET("DEMO", PH);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
        p322.param_count_SET((char)57318);
        p322.param_index_SET((char)13492);
        CommunicationChannel.instance.send(p322); //===============================
        PARAM_EXT_SET p323 = CommunicationChannel.instance.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)83);
        p323.target_component_SET((char)0);
        p323.param_id_SET("DEMO", PH);
        p323.param_value_SET("DEMO", PH);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL32);
        CommunicationChannel.instance.send(p323); //===============================
        PARAM_EXT_ACK p324 = CommunicationChannel.instance.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("DEMO", PH);
        p324.param_value_SET("DEMO", PH);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_ACCEPTED);
        CommunicationChannel.instance.send(p324); //===============================
        OBSTACLE_DISTANCE p330 = CommunicationChannel.instance.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.time_usec_SET(4690675060830589743L);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
        p330.distances_SET(new char[72], 0);
        p330.increment_SET((char)152);
        p330.min_distance_SET((char)10754);
        p330.max_distance_SET((char)4913);
        CommunicationChannel.instance.send(p330); //===============================
    }
}
