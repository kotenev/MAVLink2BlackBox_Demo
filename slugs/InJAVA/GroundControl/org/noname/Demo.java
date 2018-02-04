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
        POSITION_TARGET_LOCAL_NED p3 = CommunicationChannel.instance.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.time_boot_ms_SET(828468480L);
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
        p3.type_mask_SET((char)27925);
        p3.x_SET(3.1994732E38F);
        p3.y_SET(1.9860937E38F);
        p3.z_SET(-2.426185E38F);
        p3.vx_SET(2.1499717E38F);
        p3.vy_SET(2.496084E37F);
        p3.vz_SET(1.4662248E38F);
        p3.afx_SET(-2.0656346E38F);
        p3.afy_SET(3.233846E38F);
        p3.afz_SET(2.3393137E38F);
        p3.yaw_SET(3.207813E38F);
        p3.yaw_rate_SET(-3.3949302E38F);
        CommunicationChannel.instance.send(p3); //===============================
        ATTITUDE_TARGET p83 = CommunicationChannel.instance.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.time_boot_ms_SET(1788797203L);
        p83.type_mask_SET((char)153);
        p83.q_SET(new float[4], 0);
        p83.body_roll_rate_SET(-2.1979015E38F);
        p83.body_pitch_rate_SET(-2.991644E37F);
        p83.body_yaw_rate_SET(-2.6192237E38F);
        p83.thrust_SET(-2.6957085E38F);
        CommunicationChannel.instance.send(p83); //===============================
        SET_POSITION_TARGET_LOCAL_NED p84 = CommunicationChannel.instance.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.time_boot_ms_SET(2298853685L);
        p84.target_system_SET((char)144);
        p84.target_component_SET((char)28);
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED);
        p84.type_mask_SET((char)12480);
        p84.x_SET(-3.311631E38F);
        p84.y_SET(-8.339113E36F);
        p84.z_SET(-2.7951414E38F);
        p84.vx_SET(1.8365316E38F);
        p84.vy_SET(-2.153969E38F);
        p84.vz_SET(-2.2117252E38F);
        p84.afx_SET(2.0931544E38F);
        p84.afy_SET(2.64944E38F);
        p84.afz_SET(2.5850998E38F);
        p84.yaw_SET(3.1880997E38F);
        p84.yaw_rate_SET(-1.1669787E38F);
        CommunicationChannel.instance.send(p84); //===============================
        SET_POSITION_TARGET_GLOBAL_INT p86 = CommunicationChannel.instance.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.time_boot_ms_SET(4263912115L);
        p86.target_system_SET((char)110);
        p86.target_component_SET((char)220);
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
        p86.type_mask_SET((char)30835);
        p86.lat_int_SET(-439667635);
        p86.lon_int_SET(544016022);
        p86.alt_SET(1.8714052E38F);
        p86.vx_SET(3.0363398E38F);
        p86.vy_SET(3.004141E38F);
        p86.vz_SET(1.7544587E38F);
        p86.afx_SET(2.0527072E38F);
        p86.afy_SET(-2.951421E38F);
        p86.afz_SET(-1.4119305E37F);
        p86.yaw_SET(-3.0870124E38F);
        p86.yaw_rate_SET(4.900182E37F);
        CommunicationChannel.instance.send(p86); //===============================
        POSITION_TARGET_GLOBAL_INT p87 = CommunicationChannel.instance.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.time_boot_ms_SET(2945887675L);
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
        p87.type_mask_SET((char)53030);
        p87.lat_int_SET(-1319134213);
        p87.lon_int_SET(-851749689);
        p87.alt_SET(2.4323564E38F);
        p87.vx_SET(-1.0900277E38F);
        p87.vy_SET(2.2021987E38F);
        p87.vz_SET(-2.6746227E38F);
        p87.afx_SET(-1.0945104E38F);
        p87.afy_SET(1.2826886E38F);
        p87.afz_SET(-9.546475E37F);
        p87.yaw_SET(-8.585899E37F);
        p87.yaw_rate_SET(9.164611E37F);
        CommunicationChannel.instance.send(p87); //===============================
        LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = CommunicationChannel.instance.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.time_boot_ms_SET(734013221L);
        p89.x_SET(2.1401954E38F);
        p89.y_SET(1.8892018E38F);
        p89.z_SET(-4.627979E37F);
        p89.roll_SET(3.3812937E38F);
        p89.pitch_SET(-1.4566345E38F);
        p89.yaw_SET(-2.0426772E38F);
        CommunicationChannel.instance.send(p89); //===============================
        HIL_STATE p90 = CommunicationChannel.instance.new_HIL_STATE();
        PH.setPack(p90);
        p90.time_usec_SET(7161391955776345680L);
        p90.roll_SET(-1.0387026E38F);
        p90.pitch_SET(-4.373546E37F);
        p90.yaw_SET(3.1027067E38F);
        p90.rollspeed_SET(-1.8704767E38F);
        p90.pitchspeed_SET(6.716747E37F);
        p90.yawspeed_SET(2.0793557E38F);
        p90.lat_SET(860649835);
        p90.lon_SET(383028754);
        p90.alt_SET(-383089699);
        p90.vx_SET((short) -2373);
        p90.vy_SET((short)30508);
        p90.vz_SET((short) -20113);
        p90.xacc_SET((short) -8641);
        p90.yacc_SET((short)21841);
        p90.zacc_SET((short) -20774);
        CommunicationChannel.instance.send(p90); //===============================
        HIL_CONTROLS p91 = CommunicationChannel.instance.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.time_usec_SET(5092545318792236240L);
        p91.roll_ailerons_SET(-4.56865E36F);
        p91.pitch_elevator_SET(-2.8081937E38F);
        p91.yaw_rudder_SET(-1.3592838E38F);
        p91.throttle_SET(-1.7612015E38F);
        p91.aux1_SET(-2.3059883E38F);
        p91.aux2_SET(5.1700587E37F);
        p91.aux3_SET(2.327436E38F);
        p91.aux4_SET(-1.8895935E38F);
        p91.mode_SET(MAV_MODE.MAV_MODE_TEST_ARMED);
        p91.nav_mode_SET((char)21);
        CommunicationChannel.instance.send(p91); //===============================
        HIL_RC_INPUTS_RAW p92 = CommunicationChannel.instance.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.time_usec_SET(9182093232895045139L);
        p92.chan1_raw_SET((char)17993);
        p92.chan2_raw_SET((char)37488);
        p92.chan3_raw_SET((char)64006);
        p92.chan4_raw_SET((char)65350);
        p92.chan5_raw_SET((char)47565);
        p92.chan6_raw_SET((char)24225);
        p92.chan7_raw_SET((char)29207);
        p92.chan8_raw_SET((char)4915);
        p92.chan9_raw_SET((char)64566);
        p92.chan10_raw_SET((char)52757);
        p92.chan11_raw_SET((char)32200);
        p92.chan12_raw_SET((char)4470);
        p92.rssi_SET((char)43);
        CommunicationChannel.instance.send(p92); //===============================
        HIL_ACTUATOR_CONTROLS p93 = CommunicationChannel.instance.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(4291141730423552491L);
        p93.controls_SET(new float[16], 0);
        p93.mode_SET(MAV_MODE.MAV_MODE_AUTO_DISARMED);
        p93.flags_SET(945413500859971811L);
        CommunicationChannel.instance.send(p93); //===============================
        OPTICAL_FLOW p100 = CommunicationChannel.instance.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.time_usec_SET(2156792960167617753L);
        p100.sensor_id_SET((char)176);
        p100.flow_x_SET((short) -30235);
        p100.flow_y_SET((short)22077);
        p100.flow_comp_m_x_SET(-2.9490569E38F);
        p100.flow_comp_m_y_SET(-1.9041191E38F);
        p100.quality_SET((char)63);
        p100.ground_distance_SET(-1.1642589E38F);
        p100.flow_rate_x_SET(1.7385913E38F, PH);
        p100.flow_rate_y_SET(1.5279897E38F, PH);
        CommunicationChannel.instance.send(p100); //===============================
        GLOBAL_VISION_POSITION_ESTIMATE p101 = CommunicationChannel.instance.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.usec_SET(3419455678482195580L);
        p101.x_SET(-1.1248262E38F);
        p101.y_SET(-3.2524574E38F);
        p101.z_SET(-1.7842797E38F);
        p101.roll_SET(-8.4936444E37F);
        p101.pitch_SET(-4.500634E37F);
        p101.yaw_SET(-3.2592818E38F);
        CommunicationChannel.instance.send(p101); //===============================
        VISION_POSITION_ESTIMATE p102 = CommunicationChannel.instance.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.usec_SET(7606130600668643347L);
        p102.x_SET(7.439834E37F);
        p102.y_SET(6.547142E37F);
        p102.z_SET(-2.4848264E38F);
        p102.roll_SET(-2.6336751E38F);
        p102.pitch_SET(-1.5296059E38F);
        p102.yaw_SET(6.0280183E37F);
        CommunicationChannel.instance.send(p102); //===============================
        VISION_SPEED_ESTIMATE p103 = CommunicationChannel.instance.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(4976858992826844334L);
        p103.x_SET(-1.5345299E38F);
        p103.y_SET(-1.2501316E38F);
        p103.z_SET(3.2981311E38F);
        CommunicationChannel.instance.send(p103); //===============================
        VICON_POSITION_ESTIMATE p104 = CommunicationChannel.instance.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.usec_SET(1060717229029599455L);
        p104.x_SET(-9.856605E37F);
        p104.y_SET(2.073608E38F);
        p104.z_SET(1.1624302E38F);
        p104.roll_SET(1.6148448E38F);
        p104.pitch_SET(-2.9457188E37F);
        p104.yaw_SET(2.7496218E38F);
        CommunicationChannel.instance.send(p104); //===============================
        HIGHRES_IMU p105 = CommunicationChannel.instance.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.time_usec_SET(2381648525974932644L);
        p105.xacc_SET(1.0433334E38F);
        p105.yacc_SET(-3.2283029E38F);
        p105.zacc_SET(-2.973804E38F);
        p105.xgyro_SET(8.771764E37F);
        p105.ygyro_SET(1.0936186E38F);
        p105.zgyro_SET(-5.9235416E37F);
        p105.xmag_SET(1.0337656E37F);
        p105.ymag_SET(-2.40893E38F);
        p105.zmag_SET(-2.497725E38F);
        p105.abs_pressure_SET(1.5351268E36F);
        p105.diff_pressure_SET(-2.2196347E38F);
        p105.pressure_alt_SET(2.8229965E37F);
        p105.temperature_SET(1.2628388E38F);
        p105.fields_updated_SET((char)56595);
        CommunicationChannel.instance.send(p105); //===============================
        OPTICAL_FLOW_RAD p106 = CommunicationChannel.instance.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.time_usec_SET(6140286548438912060L);
        p106.sensor_id_SET((char)21);
        p106.integration_time_us_SET(956046548L);
        p106.integrated_x_SET(1.7481958E38F);
        p106.integrated_y_SET(-1.5354509E37F);
        p106.integrated_xgyro_SET(-2.1332909E38F);
        p106.integrated_ygyro_SET(-3.3364868E38F);
        p106.integrated_zgyro_SET(-1.1184221E38F);
        p106.temperature_SET((short) -22998);
        p106.quality_SET((char)128);
        p106.time_delta_distance_us_SET(790680402L);
        p106.distance_SET(-1.6080346E38F);
        CommunicationChannel.instance.send(p106); //===============================
        HIL_SENSOR p107 = CommunicationChannel.instance.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.time_usec_SET(1817512183475424216L);
        p107.xacc_SET(-2.9875673E38F);
        p107.yacc_SET(-8.582295E37F);
        p107.zacc_SET(-3.3868342E38F);
        p107.xgyro_SET(2.8623358E38F);
        p107.ygyro_SET(3.9894336E37F);
        p107.zgyro_SET(8.614404E37F);
        p107.xmag_SET(-1.2488042E38F);
        p107.ymag_SET(1.5925827E38F);
        p107.zmag_SET(2.5572001E38F);
        p107.abs_pressure_SET(2.2703894E38F);
        p107.diff_pressure_SET(-3.003522E38F);
        p107.pressure_alt_SET(2.5675486E38F);
        p107.temperature_SET(2.0357678E38F);
        p107.fields_updated_SET(952296943L);
        CommunicationChannel.instance.send(p107); //===============================
        SIM_STATE p108 = CommunicationChannel.instance.new_SIM_STATE();
        PH.setPack(p108);
        p108.q1_SET(9.878349E36F);
        p108.q2_SET(2.1368658E38F);
        p108.q3_SET(2.7123311E38F);
        p108.q4_SET(-2.5638775E38F);
        p108.roll_SET(1.4279773E38F);
        p108.pitch_SET(3.7667824E37F);
        p108.yaw_SET(-3.2471189E38F);
        p108.xacc_SET(-2.476672E38F);
        p108.yacc_SET(-2.3139297E38F);
        p108.zacc_SET(2.6793319E38F);
        p108.xgyro_SET(-2.7706285E38F);
        p108.ygyro_SET(1.6366166E38F);
        p108.zgyro_SET(-8.2711976E37F);
        p108.lat_SET(1.1992936E38F);
        p108.lon_SET(3.1178694E38F);
        p108.alt_SET(-1.8836929E38F);
        p108.std_dev_horz_SET(-2.0117053E38F);
        p108.std_dev_vert_SET(-3.0264454E38F);
        p108.vn_SET(-3.117023E38F);
        p108.ve_SET(-3.1420235E38F);
        p108.vd_SET(2.4583378E38F);
        CommunicationChannel.instance.send(p108); //===============================
        RADIO_STATUS p109 = CommunicationChannel.instance.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.rssi_SET((char)59);
        p109.remrssi_SET((char)59);
        p109.txbuf_SET((char)164);
        p109.noise_SET((char)183);
        p109.remnoise_SET((char)221);
        p109.rxerrors_SET((char)20951);
        p109.fixed__SET((char)25128);
        CommunicationChannel.instance.send(p109); //===============================
        FILE_TRANSFER_PROTOCOL p110 = CommunicationChannel.instance.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_network_SET((char)29);
        p110.target_system_SET((char)216);
        p110.target_component_SET((char)215);
        p110.payload_SET(new char[251], 0);
        CommunicationChannel.instance.send(p110); //===============================
        TIMESYNC p111 = CommunicationChannel.instance.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(3693997296890131264L);
        p111.ts1_SET(-4271394345795071481L);
        CommunicationChannel.instance.send(p111); //===============================
        CAMERA_TRIGGER p112 = CommunicationChannel.instance.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(2467780518027519523L);
        p112.seq_SET(2390629203L);
        CommunicationChannel.instance.send(p112); //===============================
        HIL_GPS p113 = CommunicationChannel.instance.new_HIL_GPS();
        PH.setPack(p113);
        p113.time_usec_SET(5688773228021596331L);
        p113.fix_type_SET((char)132);
        p113.lat_SET(-1981213953);
        p113.lon_SET(-2025012103);
        p113.alt_SET(-288108003);
        p113.eph_SET((char)30472);
        p113.epv_SET((char)64960);
        p113.vel_SET((char)30164);
        p113.vn_SET((short) -30732);
        p113.ve_SET((short) -12404);
        p113.vd_SET((short)24116);
        p113.cog_SET((char)46967);
        p113.satellites_visible_SET((char)88);
        CommunicationChannel.instance.send(p113); //===============================
        HIL_OPTICAL_FLOW p114 = CommunicationChannel.instance.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.time_usec_SET(2898011163224080789L);
        p114.sensor_id_SET((char)61);
        p114.integration_time_us_SET(1197880287L);
        p114.integrated_x_SET(1.5445239E38F);
        p114.integrated_y_SET(9.960687E37F);
        p114.integrated_xgyro_SET(3.763005E37F);
        p114.integrated_ygyro_SET(9.336533E37F);
        p114.integrated_zgyro_SET(-2.496855E38F);
        p114.temperature_SET((short) -471);
        p114.quality_SET((char)153);
        p114.time_delta_distance_us_SET(3970493201L);
        p114.distance_SET(-1.6001771E38F);
        CommunicationChannel.instance.send(p114); //===============================
        HIL_STATE_QUATERNION p115 = CommunicationChannel.instance.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.time_usec_SET(5325957687661219920L);
        p115.attitude_quaternion_SET(new float[4], 0);
        p115.rollspeed_SET(-2.155776E38F);
        p115.pitchspeed_SET(2.8410238E38F);
        p115.yawspeed_SET(3.3570822E38F);
        p115.lat_SET(-1461057738);
        p115.lon_SET(302108922);
        p115.alt_SET(-1370439403);
        p115.vx_SET((short)31618);
        p115.vy_SET((short)12632);
        p115.vz_SET((short) -10753);
        p115.ind_airspeed_SET((char)5499);
        p115.true_airspeed_SET((char)49276);
        p115.xacc_SET((short) -20792);
        p115.yacc_SET((short)29609);
        p115.zacc_SET((short) -27187);
        CommunicationChannel.instance.send(p115); //===============================
        SCALED_IMU2 p116 = CommunicationChannel.instance.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.time_boot_ms_SET(3692547276L);
        p116.xacc_SET((short)10733);
        p116.yacc_SET((short)30134);
        p116.zacc_SET((short) -25017);
        p116.xgyro_SET((short)10121);
        p116.ygyro_SET((short) -30881);
        p116.zgyro_SET((short) -29141);
        p116.xmag_SET((short) -19207);
        p116.ymag_SET((short)32762);
        p116.zmag_SET((short)6210);
        CommunicationChannel.instance.send(p116); //===============================
        LOG_REQUEST_LIST p117 = CommunicationChannel.instance.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_system_SET((char)28);
        p117.target_component_SET((char)246);
        p117.start_SET((char)123);
        p117.end_SET((char)4228);
        CommunicationChannel.instance.send(p117); //===============================
        LOG_ENTRY p118 = CommunicationChannel.instance.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)60161);
        p118.num_logs_SET((char)15917);
        p118.last_log_num_SET((char)8514);
        p118.time_utc_SET(1528420900L);
        p118.size_SET(773797808L);
        CommunicationChannel.instance.send(p118); //===============================
        LOG_REQUEST_DATA p119 = CommunicationChannel.instance.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)178);
        p119.target_component_SET((char)188);
        p119.id_SET((char)58554);
        p119.ofs_SET(4126266705L);
        p119.count_SET(2781384340L);
        CommunicationChannel.instance.send(p119); //===============================
        LOG_DATA p120 = CommunicationChannel.instance.new_LOG_DATA();
        PH.setPack(p120);
        p120.id_SET((char)26571);
        p120.ofs_SET(107102017L);
        p120.count_SET((char)151);
        p120.data__SET(new char[90], 0);
        CommunicationChannel.instance.send(p120); //===============================
        LOG_ERASE p121 = CommunicationChannel.instance.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)3);
        p121.target_component_SET((char)72);
        CommunicationChannel.instance.send(p121); //===============================
        LOG_REQUEST_END p122 = CommunicationChannel.instance.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_system_SET((char)22);
        p122.target_component_SET((char)84);
        CommunicationChannel.instance.send(p122); //===============================
        GPS_INJECT_DATA p123 = CommunicationChannel.instance.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.target_system_SET((char)7);
        p123.target_component_SET((char)117);
        p123.len_SET((char)30);
        p123.data__SET(new char[110], 0);
        CommunicationChannel.instance.send(p123); //===============================
        GPS2_RAW p124 = CommunicationChannel.instance.new_GPS2_RAW();
        PH.setPack(p124);
        p124.time_usec_SET(1639656153120749696L);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_PPP);
        p124.lat_SET(-548414935);
        p124.lon_SET(-320413267);
        p124.alt_SET(1926945318);
        p124.eph_SET((char)36537);
        p124.epv_SET((char)25198);
        p124.vel_SET((char)27434);
        p124.cog_SET((char)41795);
        p124.satellites_visible_SET((char)57);
        p124.dgps_numch_SET((char)155);
        p124.dgps_age_SET(1471940445L);
        CommunicationChannel.instance.send(p124); //===============================
        POWER_STATUS p125 = CommunicationChannel.instance.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vcc_SET((char)53362);
        p125.Vservo_SET((char)352);
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_SERVO_VALID);
        CommunicationChannel.instance.send(p125); //===============================
        SERIAL_CONTROL p126 = CommunicationChannel.instance.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
        p126.timeout_SET((char)59657);
        p126.baudrate_SET(1411590481L);
        p126.count_SET((char)113);
        p126.data__SET(new char[70], 0);
        CommunicationChannel.instance.send(p126); //===============================
        GPS_RTK p127 = CommunicationChannel.instance.new_GPS_RTK();
        PH.setPack(p127);
        p127.time_last_baseline_ms_SET(191284644L);
        p127.rtk_receiver_id_SET((char)49);
        p127.wn_SET((char)50723);
        p127.tow_SET(2847625728L);
        p127.rtk_health_SET((char)73);
        p127.rtk_rate_SET((char)53);
        p127.nsats_SET((char)37);
        p127.baseline_coords_type_SET((char)30);
        p127.baseline_a_mm_SET(-505069676);
        p127.baseline_b_mm_SET(1304360903);
        p127.baseline_c_mm_SET(-635506895);
        p127.accuracy_SET(897466173L);
        p127.iar_num_hypotheses_SET(1437000493);
        CommunicationChannel.instance.send(p127); //===============================
        GPS2_RTK p128 = CommunicationChannel.instance.new_GPS2_RTK();
        PH.setPack(p128);
        p128.time_last_baseline_ms_SET(210191847L);
        p128.rtk_receiver_id_SET((char)93);
        p128.wn_SET((char)64357);
        p128.tow_SET(1950967641L);
        p128.rtk_health_SET((char)25);
        p128.rtk_rate_SET((char)224);
        p128.nsats_SET((char)129);
        p128.baseline_coords_type_SET((char)192);
        p128.baseline_a_mm_SET(1579524885);
        p128.baseline_b_mm_SET(989019487);
        p128.baseline_c_mm_SET(1117732385);
        p128.accuracy_SET(2656801538L);
        p128.iar_num_hypotheses_SET(-2041355688);
        CommunicationChannel.instance.send(p128); //===============================
        SCALED_IMU3 p129 = CommunicationChannel.instance.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.time_boot_ms_SET(1835582158L);
        p129.xacc_SET((short)14370);
        p129.yacc_SET((short)9438);
        p129.zacc_SET((short) -3275);
        p129.xgyro_SET((short)4496);
        p129.ygyro_SET((short)14685);
        p129.zgyro_SET((short) -14480);
        p129.xmag_SET((short)15358);
        p129.ymag_SET((short) -12286);
        p129.zmag_SET((short) -263);
        CommunicationChannel.instance.send(p129); //===============================
        DATA_TRANSMISSION_HANDSHAKE p130 = CommunicationChannel.instance.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.type_SET((char)187);
        p130.size_SET(1116501842L);
        p130.width_SET((char)33865);
        p130.height_SET((char)47643);
        p130.packets_SET((char)54205);
        p130.payload_SET((char)160);
        p130.jpg_quality_SET((char)57);
        CommunicationChannel.instance.send(p130); //===============================
        ENCAPSULATED_DATA p131 = CommunicationChannel.instance.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)42981);
        p131.data__SET(new char[253], 0);
        CommunicationChannel.instance.send(p131); //===============================
        DISTANCE_SENSOR p132 = CommunicationChannel.instance.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.time_boot_ms_SET(2498148037L);
        p132.min_distance_SET((char)65295);
        p132.max_distance_SET((char)39106);
        p132.current_distance_SET((char)5970);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
        p132.id_SET((char)68);
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_270_PITCH_180);
        p132.covariance_SET((char)163);
        CommunicationChannel.instance.send(p132); //===============================
        TERRAIN_REQUEST p133 = CommunicationChannel.instance.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lat_SET(242800901);
        p133.lon_SET(1714819963);
        p133.grid_spacing_SET((char)40984);
        p133.mask_SET(7987722390593808569L);
        CommunicationChannel.instance.send(p133); //===============================
        TERRAIN_DATA p134 = CommunicationChannel.instance.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lat_SET(1041547625);
        p134.lon_SET(-1372382063);
        p134.grid_spacing_SET((char)57593);
        p134.gridbit_SET((char)215);
        p134.data__SET(new short[16], 0);
        CommunicationChannel.instance.send(p134); //===============================
        TERRAIN_CHECK p135 = CommunicationChannel.instance.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(-434025629);
        p135.lon_SET(469324590);
        CommunicationChannel.instance.send(p135); //===============================
        TERRAIN_REPORT p136 = CommunicationChannel.instance.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.lat_SET(1757221060);
        p136.lon_SET(-980298000);
        p136.spacing_SET((char)34195);
        p136.terrain_height_SET(1.7638698E37F);
        p136.current_height_SET(-8.264649E37F);
        p136.pending_SET((char)32576);
        p136.loaded_SET((char)55569);
        CommunicationChannel.instance.send(p136); //===============================
        SCALED_PRESSURE2 p137 = CommunicationChannel.instance.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(1347115911L);
        p137.press_abs_SET(-3.0291055E38F);
        p137.press_diff_SET(-2.053642E38F);
        p137.temperature_SET((short)23621);
        CommunicationChannel.instance.send(p137); //===============================
        ATT_POS_MOCAP p138 = CommunicationChannel.instance.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.time_usec_SET(3802243935665528313L);
        p138.q_SET(new float[4], 0);
        p138.x_SET(-1.3628395E38F);
        p138.y_SET(2.5517746E38F);
        p138.z_SET(2.9473318E37F);
        CommunicationChannel.instance.send(p138); //===============================
        SET_ACTUATOR_CONTROL_TARGET p139 = CommunicationChannel.instance.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(6755459112968779630L);
        p139.group_mlx_SET((char)127);
        p139.target_system_SET((char)160);
        p139.target_component_SET((char)25);
        p139.controls_SET(new float[8], 0);
        CommunicationChannel.instance.send(p139); //===============================
        ACTUATOR_CONTROL_TARGET p140 = CommunicationChannel.instance.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.time_usec_SET(5131180020989771490L);
        p140.group_mlx_SET((char)8);
        p140.controls_SET(new float[8], 0);
        CommunicationChannel.instance.send(p140); //===============================
        ALTITUDE p141 = CommunicationChannel.instance.new_ALTITUDE();
        PH.setPack(p141);
        p141.time_usec_SET(5986922528571301580L);
        p141.altitude_monotonic_SET(-1.718392E38F);
        p141.altitude_amsl_SET(-8.543753E37F);
        p141.altitude_local_SET(-2.2136295E38F);
        p141.altitude_relative_SET(-1.9403356E38F);
        p141.altitude_terrain_SET(2.1570332E38F);
        p141.bottom_clearance_SET(3.3040564E38F);
        CommunicationChannel.instance.send(p141); //===============================
        RESOURCE_REQUEST p142 = CommunicationChannel.instance.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.request_id_SET((char)104);
        p142.uri_type_SET((char)215);
        p142.uri_SET(new char[120], 0);
        p142.transfer_type_SET((char)192);
        p142.storage_SET(new char[120], 0);
        CommunicationChannel.instance.send(p142); //===============================
        SCALED_PRESSURE3 p143 = CommunicationChannel.instance.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(2694732049L);
        p143.press_abs_SET(2.0605143E38F);
        p143.press_diff_SET(2.7763033E38F);
        p143.temperature_SET((short)29283);
        CommunicationChannel.instance.send(p143); //===============================
        FOLLOW_TARGET p144 = CommunicationChannel.instance.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.timestamp_SET(3501205337944410351L);
        p144.est_capabilities_SET((char)133);
        p144.lat_SET(-246610341);
        p144.lon_SET(1082656641);
        p144.alt_SET(1.314954E38F);
        p144.vel_SET(new float[3], 0);
        p144.acc_SET(new float[3], 0);
        p144.attitude_q_SET(new float[4], 0);
        p144.rates_SET(new float[3], 0);
        p144.position_cov_SET(new float[3], 0);
        p144.custom_state_SET(8040605166655808222L);
        CommunicationChannel.instance.send(p144); //===============================
        CONTROL_SYSTEM_STATE p146 = CommunicationChannel.instance.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.time_usec_SET(1748359265387410557L);
        p146.x_acc_SET(1.4160934E38F);
        p146.y_acc_SET(3.3193678E36F);
        p146.z_acc_SET(3.3388576E38F);
        p146.x_vel_SET(1.928448E37F);
        p146.y_vel_SET(2.1679604E38F);
        p146.z_vel_SET(-2.3964466E38F);
        p146.x_pos_SET(2.219644E37F);
        p146.y_pos_SET(2.673008E38F);
        p146.z_pos_SET(-1.962938E38F);
        p146.airspeed_SET(2.4927757E38F);
        p146.vel_variance_SET(new float[3], 0);
        p146.pos_variance_SET(new float[3], 0);
        p146.q_SET(new float[4], 0);
        p146.roll_rate_SET(1.6869067E38F);
        p146.pitch_rate_SET(-2.0586463E38F);
        p146.yaw_rate_SET(-3.589221E37F);
        CommunicationChannel.instance.send(p146); //===============================
        BATTERY_STATUS p147 = CommunicationChannel.instance.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.id_SET((char)58);
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
        p147.temperature_SET((short) -14821);
        p147.voltages_SET(new char[10], 0);
        p147.current_battery_SET((short) -17046);
        p147.current_consumed_SET(-811642745);
        p147.energy_consumed_SET(-1911335526);
        p147.battery_remaining_SET((byte) - 100);
        CommunicationChannel.instance.send(p147); //===============================
        AUTOPILOT_VERSION p148 = CommunicationChannel.instance.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT);
        p148.flight_sw_version_SET(679696732L);
        p148.middleware_sw_version_SET(2323792888L);
        p148.os_sw_version_SET(913564342L);
        p148.board_version_SET(4260852452L);
        p148.flight_custom_version_SET(new char[8], 0);
        p148.middleware_custom_version_SET(new char[8], 0);
        p148.os_custom_version_SET(new char[8], 0);
        p148.vendor_id_SET((char)57638);
        p148.product_id_SET((char)10257);
        p148.uid_SET(4418905348372370545L);
        p148.uid2_SET(new char[18], 0, PH);
        CommunicationChannel.instance.send(p148); //===============================
        LANDING_TARGET p149 = CommunicationChannel.instance.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.time_usec_SET(5502966963788926707L);
        p149.target_num_SET((char)230);
        p149.frame_SET(MAV_FRAME.MAV_FRAME_MISSION);
        p149.angle_x_SET(-1.064834E38F);
        p149.angle_y_SET(-1.894844E38F);
        p149.distance_SET(-1.6276685E38F);
        p149.size_x_SET(-8.758249E37F);
        p149.size_y_SET(2.402217E36F);
        p149.x_SET(-7.7865433E37F, PH);
        p149.y_SET(-6.9569177E37F, PH);
        p149.z_SET(2.3222162E38F, PH);
        p149.q_SET(new float[4], 0, PH);
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
        p149.position_valid_SET((char)21, PH);
        CommunicationChannel.instance.send(p149); //===============================
        CPU_LOAD p170 = CommunicationChannel.instance.new_CPU_LOAD();
        PH.setPack(p170);
        p170.sensLoad_SET((char)75);
        p170.ctrlLoad_SET((char)67);
        p170.batVolt_SET((char)56160);
        CommunicationChannel.instance.send(p170); //===============================
        SENSOR_BIAS p172 = CommunicationChannel.instance.new_SENSOR_BIAS();
        PH.setPack(p172);
        p172.axBias_SET(2.545262E38F);
        p172.ayBias_SET(-1.6483997E38F);
        p172.azBias_SET(1.3591684E38F);
        p172.gxBias_SET(2.4575496E38F);
        p172.gyBias_SET(2.7320363E38F);
        p172.gzBias_SET(-3.0779853E38F);
        CommunicationChannel.instance.send(p172); //===============================
        DIAGNOSTIC p173 = CommunicationChannel.instance.new_DIAGNOSTIC();
        PH.setPack(p173);
        p173.diagFl1_SET(2.779955E38F);
        p173.diagFl2_SET(-6.7956274E37F);
        p173.diagFl3_SET(7.39685E37F);
        p173.diagSh1_SET((short) -20779);
        p173.diagSh2_SET((short)19470);
        p173.diagSh3_SET((short)26447);
        CommunicationChannel.instance.send(p173); //===============================
        SLUGS_NAVIGATION p176 = CommunicationChannel.instance.new_SLUGS_NAVIGATION();
        PH.setPack(p176);
        p176.u_m_SET(1.8152039E38F);
        p176.phi_c_SET(-2.4482252E38F);
        p176.theta_c_SET(1.1814363E37F);
        p176.psiDot_c_SET(-1.3356435E38F);
        p176.ay_body_SET(-1.8953863E38F);
        p176.totalDist_SET(-3.511373E37F);
        p176.dist2Go_SET(2.1191222E38F);
        p176.fromWP_SET((char)96);
        p176.toWP_SET((char)15);
        p176.h_c_SET((char)11332);
        CommunicationChannel.instance.send(p176); //===============================
        DATA_LOG p177 = CommunicationChannel.instance.new_DATA_LOG();
        PH.setPack(p177);
        p177.fl_1_SET(1.881955E38F);
        p177.fl_2_SET(1.8958727E38F);
        p177.fl_3_SET(2.3449792E38F);
        p177.fl_4_SET(2.6426034E36F);
        p177.fl_5_SET(-3.150142E38F);
        p177.fl_6_SET(3.3468663E38F);
        CommunicationChannel.instance.send(p177); //===============================
        GPS_DATE_TIME p179 = CommunicationChannel.instance.new_GPS_DATE_TIME();
        PH.setPack(p179);
        p179.year_SET((char)185);
        p179.month_SET((char)131);
        p179.day_SET((char)159);
        p179.hour_SET((char)164);
        p179.min_SET((char)29);
        p179.sec_SET((char)130);
        p179.clockStat_SET((char)152);
        p179.visSat_SET((char)170);
        p179.useSat_SET((char)125);
        p179.GppGl_SET((char)171);
        p179.sigUsedMask_SET((char)96);
        p179.percentUsed_SET((char)53);
        CommunicationChannel.instance.send(p179); //===============================
        MID_LVL_CMDS p180 = CommunicationChannel.instance.new_MID_LVL_CMDS();
        PH.setPack(p180);
        p180.target_SET((char)56);
        p180.hCommand_SET(1.1328539E38F);
        p180.uCommand_SET(-1.8837858E38F);
        p180.rCommand_SET(-2.3143274E38F);
        CommunicationChannel.instance.send(p180); //===============================
        CTRL_SRFC_PT p181 = CommunicationChannel.instance.new_CTRL_SRFC_PT();
        PH.setPack(p181);
        p181.target_SET((char)61);
        p181.bitfieldPt_SET((char)18049);
        CommunicationChannel.instance.send(p181); //===============================
        SLUGS_CAMERA_ORDER p184 = CommunicationChannel.instance.new_SLUGS_CAMERA_ORDER();
        PH.setPack(p184);
        p184.target_SET((char)253);
        p184.pan_SET((byte) - 40);
        p184.tilt_SET((byte) - 41);
        p184.zoom_SET((byte)94);
        p184.moveHome_SET((byte)50);
        CommunicationChannel.instance.send(p184); //===============================
        CONTROL_SURFACE p185 = CommunicationChannel.instance.new_CONTROL_SURFACE();
        PH.setPack(p185);
        p185.target_SET((char)186);
        p185.idSurface_SET((char)178);
        p185.mControl_SET(7.6058757E37F);
        p185.bControl_SET(-5.2694466E37F);
        CommunicationChannel.instance.send(p185); //===============================
        SLUGS_MOBILE_LOCATION p186 = CommunicationChannel.instance.new_SLUGS_MOBILE_LOCATION();
        PH.setPack(p186);
        p186.target_SET((char)39);
        p186.latitude_SET(1.8129521E38F);
        p186.longitude_SET(-1.8864647E38F);
        CommunicationChannel.instance.send(p186); //===============================
        SLUGS_CONFIGURATION_CAMERA p188 = CommunicationChannel.instance.new_SLUGS_CONFIGURATION_CAMERA();
        PH.setPack(p188);
        p188.target_SET((char)244);
        p188.idOrder_SET((char)81);
        p188.order_SET((char)70);
        CommunicationChannel.instance.send(p188); //===============================
        ISR_LOCATION p189 = CommunicationChannel.instance.new_ISR_LOCATION();
        PH.setPack(p189);
        p189.target_SET((char)53);
        p189.latitude_SET(-2.7045214E38F);
        p189.longitude_SET(-1.9436586E38F);
        p189.height_SET(-2.5108913E38F);
        p189.option1_SET((char)28);
        p189.option2_SET((char)226);
        p189.option3_SET((char)193);
        CommunicationChannel.instance.send(p189); //===============================
        VOLT_SENSOR p191 = CommunicationChannel.instance.new_VOLT_SENSOR();
        PH.setPack(p191);
        p191.r2Type_SET((char)71);
        p191.voltage_SET((char)3817);
        p191.reading2_SET((char)43860);
        CommunicationChannel.instance.send(p191); //===============================
        PTZ_STATUS p192 = CommunicationChannel.instance.new_PTZ_STATUS();
        PH.setPack(p192);
        p192.zoom_SET((char)11);
        p192.pan_SET((short)19826);
        p192.tilt_SET((short) -31313);
        CommunicationChannel.instance.send(p192); //===============================
        UAV_STATUS p193 = CommunicationChannel.instance.new_UAV_STATUS();
        PH.setPack(p193);
        p193.target_SET((char)224);
        p193.latitude_SET(1.7780954E38F);
        p193.longitude_SET(-2.903387E37F);
        p193.altitude_SET(-2.589808E38F);
        p193.speed_SET(3.3614753E38F);
        p193.course_SET(-4.0785238E37F);
        CommunicationChannel.instance.send(p193); //===============================
        STATUS_GPS p194 = CommunicationChannel.instance.new_STATUS_GPS();
        PH.setPack(p194);
        p194.csFails_SET((char)12802);
        p194.gpsQuality_SET((char)218);
        p194.msgsType_SET((char)5);
        p194.posStatus_SET((char)254);
        p194.magVar_SET(2.146707E38F);
        p194.magDir_SET((byte)18);
        p194.modeInd_SET((char)16);
        CommunicationChannel.instance.send(p194); //===============================
        NOVATEL_DIAG p195 = CommunicationChannel.instance.new_NOVATEL_DIAG();
        PH.setPack(p195);
        p195.timeStatus_SET((char)73);
        p195.receiverStatus_SET(2783704258L);
        p195.solStatus_SET((char)240);
        p195.posType_SET((char)88);
        p195.velType_SET((char)226);
        p195.posSolAge_SET(2.105531E38F);
        p195.csFails_SET((char)52544);
        CommunicationChannel.instance.send(p195); //===============================
        SENSOR_DIAG p196 = CommunicationChannel.instance.new_SENSOR_DIAG();
        PH.setPack(p196);
        p196.float1_SET(-2.845622E38F);
        p196.float2_SET(-1.1868031E38F);
        p196.int1_SET((short) -7684);
        p196.char1_SET((byte) - 24);
        CommunicationChannel.instance.send(p196); //===============================
        BOOT p197 = CommunicationChannel.instance.new_BOOT();
        PH.setPack(p197);
        p197.version_SET(4024386787L);
        CommunicationChannel.instance.send(p197); //===============================
        ESTIMATOR_STATUS p230 = CommunicationChannel.instance.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.time_usec_SET(3625808879925475904L);
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL);
        p230.vel_ratio_SET(-9.412184E37F);
        p230.pos_horiz_ratio_SET(1.7777395E37F);
        p230.pos_vert_ratio_SET(2.6345305E38F);
        p230.mag_ratio_SET(-2.7571788E38F);
        p230.hagl_ratio_SET(-1.8370586E38F);
        p230.tas_ratio_SET(1.9898956E38F);
        p230.pos_horiz_accuracy_SET(-5.6278667E37F);
        p230.pos_vert_accuracy_SET(-4.6801985E37F);
        CommunicationChannel.instance.send(p230); //===============================
        WIND_COV p231 = CommunicationChannel.instance.new_WIND_COV();
        PH.setPack(p231);
        p231.time_usec_SET(1608010671546332917L);
        p231.wind_x_SET(-1.5458476E38F);
        p231.wind_y_SET(-7.500376E37F);
        p231.wind_z_SET(1.3466058E38F);
        p231.var_horiz_SET(-4.9562137E37F);
        p231.var_vert_SET(7.2004243E37F);
        p231.wind_alt_SET(1.1991235E38F);
        p231.horiz_accuracy_SET(2.1121836E38F);
        p231.vert_accuracy_SET(6.006051E37F);
        CommunicationChannel.instance.send(p231); //===============================
        GPS_INPUT p232 = CommunicationChannel.instance.new_GPS_INPUT();
        PH.setPack(p232);
        p232.time_usec_SET(2847721793131449541L);
        p232.gps_id_SET((char)66);
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY);
        p232.time_week_ms_SET(905082820L);
        p232.time_week_SET((char)35328);
        p232.fix_type_SET((char)239);
        p232.lat_SET(-1854174071);
        p232.lon_SET(-999338239);
        p232.alt_SET(7.771708E36F);
        p232.hdop_SET(-1.7726972E38F);
        p232.vdop_SET(3.2817683E38F);
        p232.vn_SET(7.8612784E37F);
        p232.ve_SET(-1.305932E38F);
        p232.vd_SET(3.290659E36F);
        p232.speed_accuracy_SET(3.3464712E38F);
        p232.horiz_accuracy_SET(-2.796747E38F);
        p232.vert_accuracy_SET(2.9361193E38F);
        p232.satellites_visible_SET((char)95);
        CommunicationChannel.instance.send(p232); //===============================
        GPS_RTCM_DATA p233 = CommunicationChannel.instance.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.flags_SET((char)49);
        p233.len_SET((char)197);
        p233.data__SET(new char[180], 0);
        CommunicationChannel.instance.send(p233); //===============================
        HIGH_LATENCY p234 = CommunicationChannel.instance.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
        p234.custom_mode_SET(633913634L);
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
        p234.roll_SET((short) -2706);
        p234.pitch_SET((short) -25451);
        p234.heading_SET((char)46583);
        p234.throttle_SET((byte) - 89);
        p234.heading_sp_SET((short)5300);
        p234.latitude_SET(45215178);
        p234.longitude_SET(-197913303);
        p234.altitude_amsl_SET((short) -9860);
        p234.altitude_sp_SET((short)12499);
        p234.airspeed_SET((char)110);
        p234.airspeed_sp_SET((char)65);
        p234.groundspeed_SET((char)205);
        p234.climb_rate_SET((byte)122);
        p234.gps_nsat_SET((char)253);
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_GPS);
        p234.battery_remaining_SET((char)208);
        p234.temperature_SET((byte)26);
        p234.temperature_air_SET((byte) - 56);
        p234.failsafe_SET((char)38);
        p234.wp_num_SET((char)33);
        p234.wp_distance_SET((char)24100);
        CommunicationChannel.instance.send(p234); //===============================
        VIBRATION p241 = CommunicationChannel.instance.new_VIBRATION();
        PH.setPack(p241);
        p241.time_usec_SET(572107689941236555L);
        p241.vibration_x_SET(9.751561E37F);
        p241.vibration_y_SET(-2.4006595E38F);
        p241.vibration_z_SET(-4.9204603E37F);
        p241.clipping_0_SET(2093636484L);
        p241.clipping_1_SET(2833212748L);
        p241.clipping_2_SET(246767615L);
        CommunicationChannel.instance.send(p241); //===============================
        HOME_POSITION p242 = CommunicationChannel.instance.new_HOME_POSITION();
        PH.setPack(p242);
        p242.latitude_SET(1622243913);
        p242.longitude_SET(1351631534);
        p242.altitude_SET(-747581934);
        p242.x_SET(-2.9562563E38F);
        p242.y_SET(-2.5058193E38F);
        p242.z_SET(6.147558E37F);
        p242.q_SET(new float[4], 0);
        p242.approach_x_SET(-2.6486874E38F);
        p242.approach_y_SET(1.9493923E38F);
        p242.approach_z_SET(-3.2474466E38F);
        p242.time_usec_SET(8785650505389593581L, PH);
        CommunicationChannel.instance.send(p242); //===============================
        SET_HOME_POSITION p243 = CommunicationChannel.instance.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.target_system_SET((char)215);
        p243.latitude_SET(-1513932689);
        p243.longitude_SET(-1567270843);
        p243.altitude_SET(644486605);
        p243.x_SET(-3.4019416E38F);
        p243.y_SET(9.339938E37F);
        p243.z_SET(2.357198E38F);
        p243.q_SET(new float[4], 0);
        p243.approach_x_SET(-3.4389642E37F);
        p243.approach_y_SET(1.5854052E38F);
        p243.approach_z_SET(-7.2393036E37F);
        p243.time_usec_SET(1864270404583406407L, PH);
        CommunicationChannel.instance.send(p243); //===============================
        MESSAGE_INTERVAL p244 = CommunicationChannel.instance.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)22451);
        p244.interval_us_SET(-692510060);
        CommunicationChannel.instance.send(p244); //===============================
        EXTENDED_SYS_STATE p245 = CommunicationChannel.instance.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
        CommunicationChannel.instance.send(p245); //===============================
        ADSB_VEHICLE p246 = CommunicationChannel.instance.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.ICAO_address_SET(2141175065L);
        p246.lat_SET(-1430782870);
        p246.lon_SET(113183111);
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
        p246.altitude_SET(-552764786);
        p246.heading_SET((char)28161);
        p246.hor_velocity_SET((char)52071);
        p246.ver_velocity_SET((short)7766);
        p246.callsign_SET("DEMO", PH);
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_HEAVY);
        p246.tslc_SET((char)69);
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS);
        p246.squawk_SET((char)16546);
        CommunicationChannel.instance.send(p246); //===============================
        COLLISION p247 = CommunicationChannel.instance.new_COLLISION();
        PH.setPack(p247);
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
        p247.id_SET(1901361958L);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_RTL);
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_HIGH);
        p247.time_to_minimum_delta_SET(2.1551646E38F);
        p247.altitude_minimum_delta_SET(-1.6372575E38F);
        p247.horizontal_minimum_delta_SET(-2.5412914E37F);
        CommunicationChannel.instance.send(p247); //===============================
        V2_EXTENSION p248 = CommunicationChannel.instance.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)161);
        p248.target_system_SET((char)194);
        p248.target_component_SET((char)210);
        p248.message_type_SET((char)11315);
        p248.payload_SET(new char[249], 0);
        CommunicationChannel.instance.send(p248); //===============================
        MEMORY_VECT p249 = CommunicationChannel.instance.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)933);
        p249.ver_SET((char)52);
        p249.type_SET((char)102);
        p249.value_SET(new byte[32], 0);
        CommunicationChannel.instance.send(p249); //===============================
        DEBUG_VECT p250 = CommunicationChannel.instance.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("DEMO", PH);
        p250.time_usec_SET(841729527149358074L);
        p250.x_SET(-1.9283253E38F);
        p250.y_SET(-2.3655372E38F);
        p250.z_SET(-1.8281556E38F);
        CommunicationChannel.instance.send(p250); //===============================
        NAMED_VALUE_FLOAT p251 = CommunicationChannel.instance.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(3573748272L);
        p251.name_SET("DEMO", PH);
        p251.value_SET(-2.1292816E38F);
        CommunicationChannel.instance.send(p251); //===============================
        NAMED_VALUE_INT p252 = CommunicationChannel.instance.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.time_boot_ms_SET(3053880140L);
        p252.name_SET("DEMO", PH);
        p252.value_SET(-815514424);
        CommunicationChannel.instance.send(p252); //===============================
        STATUSTEXT p253 = CommunicationChannel.instance.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
        p253.text_SET("DEMO", PH);
        CommunicationChannel.instance.send(p253); //===============================
        DEBUG p254 = CommunicationChannel.instance.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(3379610192L);
        p254.ind_SET((char)75);
        p254.value_SET(1.618983E38F);
        CommunicationChannel.instance.send(p254); //===============================
        SETUP_SIGNING p256 = CommunicationChannel.instance.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_system_SET((char)159);
        p256.target_component_SET((char)0);
        p256.secret_key_SET(new char[32], 0);
        p256.initial_timestamp_SET(7985218798400614102L);
        CommunicationChannel.instance.send(p256); //===============================
        BUTTON_CHANGE p257 = CommunicationChannel.instance.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(3134167921L);
        p257.last_change_ms_SET(3066746904L);
        p257.state_SET((char)12);
        CommunicationChannel.instance.send(p257); //===============================
        PLAY_TUNE p258 = CommunicationChannel.instance.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)120);
        p258.target_component_SET((char)225);
        p258.tune_SET("DEMO", PH);
        CommunicationChannel.instance.send(p258); //===============================
        CAMERA_INFORMATION p259 = CommunicationChannel.instance.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.time_boot_ms_SET(1089382700L);
        p259.vendor_name_SET(new char[32], 0);
        p259.model_name_SET(new char[32], 0);
        p259.firmware_version_SET(2103503355L);
        p259.focal_length_SET(2.3165004E38F);
        p259.sensor_size_h_SET(-4.9547077E37F);
        p259.sensor_size_v_SET(-1.9031993E38F);
        p259.resolution_h_SET((char)28238);
        p259.resolution_v_SET((char)42243);
        p259.lens_id_SET((char)62);
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
        p259.cam_definition_version_SET((char)40486);
        p259.cam_definition_uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p259); //===============================
        CAMERA_SETTINGS p260 = CommunicationChannel.instance.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(1245819857L);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_VIDEO);
        CommunicationChannel.instance.send(p260); //===============================
        STORAGE_INFORMATION p261 = CommunicationChannel.instance.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.time_boot_ms_SET(2641556939L);
        p261.storage_id_SET((char)118);
        p261.storage_count_SET((char)186);
        p261.status_SET((char)24);
        p261.total_capacity_SET(2.3326615E38F);
        p261.used_capacity_SET(-9.952794E37F);
        p261.available_capacity_SET(6.807014E37F);
        p261.read_speed_SET(6.504146E37F);
        p261.write_speed_SET(-2.2292881E38F);
        CommunicationChannel.instance.send(p261); //===============================
        CAMERA_CAPTURE_STATUS p262 = CommunicationChannel.instance.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.time_boot_ms_SET(2882925563L);
        p262.image_status_SET((char)230);
        p262.video_status_SET((char)165);
        p262.image_interval_SET(-2.2186774E38F);
        p262.recording_time_ms_SET(30123780L);
        p262.available_capacity_SET(2.8952489E38F);
        CommunicationChannel.instance.send(p262); //===============================
        CAMERA_IMAGE_CAPTURED p263 = CommunicationChannel.instance.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.time_boot_ms_SET(4003097909L);
        p263.time_utc_SET(8945373999265567221L);
        p263.camera_id_SET((char)108);
        p263.lat_SET(-1488885266);
        p263.lon_SET(-648578603);
        p263.alt_SET(-64331853);
        p263.relative_alt_SET(-730230158);
        p263.q_SET(new float[4], 0);
        p263.image_index_SET(1109509791);
        p263.capture_result_SET((byte)6);
        p263.file_url_SET("DEMO", PH);
        CommunicationChannel.instance.send(p263); //===============================
        FLIGHT_INFORMATION p264 = CommunicationChannel.instance.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.time_boot_ms_SET(3054555583L);
        p264.arming_time_utc_SET(3500209524621347949L);
        p264.takeoff_time_utc_SET(4615809199537904949L);
        p264.flight_uuid_SET(2214817178543985298L);
        CommunicationChannel.instance.send(p264); //===============================
        MOUNT_ORIENTATION p265 = CommunicationChannel.instance.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.time_boot_ms_SET(1648002808L);
        p265.roll_SET(-1.6408636E38F);
        p265.pitch_SET(3.3476247E38F);
        p265.yaw_SET(-1.2000453E38F);
        CommunicationChannel.instance.send(p265); //===============================
        LOGGING_DATA p266 = CommunicationChannel.instance.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.target_system_SET((char)127);
        p266.target_component_SET((char)107);
        p266.sequence_SET((char)19607);
        p266.length_SET((char)46);
        p266.first_message_offset_SET((char)115);
        p266.data__SET(new char[249], 0);
        CommunicationChannel.instance.send(p266); //===============================
        LOGGING_DATA_ACKED p267 = CommunicationChannel.instance.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_system_SET((char)175);
        p267.target_component_SET((char)176);
        p267.sequence_SET((char)26774);
        p267.length_SET((char)192);
        p267.first_message_offset_SET((char)142);
        p267.data__SET(new char[249], 0);
        CommunicationChannel.instance.send(p267); //===============================
        LOGGING_ACK p268 = CommunicationChannel.instance.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)156);
        p268.target_component_SET((char)123);
        p268.sequence_SET((char)36691);
        CommunicationChannel.instance.send(p268); //===============================
        VIDEO_STREAM_INFORMATION p269 = CommunicationChannel.instance.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.camera_id_SET((char)235);
        p269.status_SET((char)27);
        p269.framerate_SET(-1.0701912E38F);
        p269.resolution_h_SET((char)4190);
        p269.resolution_v_SET((char)19070);
        p269.bitrate_SET(131819429L);
        p269.rotation_SET((char)39846);
        p269.uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p269); //===============================
        SET_VIDEO_STREAM_SETTINGS p270 = CommunicationChannel.instance.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.target_system_SET((char)231);
        p270.target_component_SET((char)204);
        p270.camera_id_SET((char)235);
        p270.framerate_SET(4.5483527E37F);
        p270.resolution_h_SET((char)24767);
        p270.resolution_v_SET((char)27637);
        p270.bitrate_SET(1419076338L);
        p270.rotation_SET((char)18362);
        p270.uri_SET("DEMO", PH);
        CommunicationChannel.instance.send(p270); //===============================
        WIFI_CONFIG_AP p299 = CommunicationChannel.instance.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("DEMO", PH);
        p299.password_SET("DEMO", PH);
        CommunicationChannel.instance.send(p299); //===============================
        PROTOCOL_VERSION p300 = CommunicationChannel.instance.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.version_SET((char)57030);
        p300.min_version_SET((char)53743);
        p300.max_version_SET((char)56219);
        p300.spec_version_hash_SET(new char[8], 0);
        p300.library_version_hash_SET(new char[8], 0);
        CommunicationChannel.instance.send(p300); //===============================
        UAVCAN_NODE_STATUS p310 = CommunicationChannel.instance.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.time_usec_SET(4727522561228915209L);
        p310.uptime_sec_SET(3594480487L);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_CRITICAL);
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
        p310.sub_mode_SET((char)225);
        p310.vendor_specific_status_code_SET((char)58393);
        CommunicationChannel.instance.send(p310); //===============================
        UAVCAN_NODE_INFO p311 = CommunicationChannel.instance.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.time_usec_SET(8654784361025716220L);
        p311.uptime_sec_SET(3106331844L);
        p311.name_SET("DEMO", PH);
        p311.hw_version_major_SET((char)176);
        p311.hw_version_minor_SET((char)243);
        p311.hw_unique_id_SET(new char[16], 0);
        p311.sw_version_major_SET((char)210);
        p311.sw_version_minor_SET((char)77);
        p311.sw_vcs_commit_SET(2006993635L);
        CommunicationChannel.instance.send(p311); //===============================
        PARAM_EXT_REQUEST_READ p320 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.target_system_SET((char)27);
        p320.target_component_SET((char)73);
        p320.param_id_SET("DEMO", PH);
        p320.param_index_SET((short)14074);
        CommunicationChannel.instance.send(p320); //===============================
        PARAM_EXT_REQUEST_LIST p321 = CommunicationChannel.instance.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)177);
        p321.target_component_SET((char)140);
        CommunicationChannel.instance.send(p321); //===============================
        PARAM_EXT_VALUE p322 = CommunicationChannel.instance.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_id_SET("DEMO", PH);
        p322.param_value_SET("DEMO", PH);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
        p322.param_count_SET((char)5892);
        p322.param_index_SET((char)51248);
        CommunicationChannel.instance.send(p322); //===============================
        PARAM_EXT_SET p323 = CommunicationChannel.instance.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)33);
        p323.target_component_SET((char)70);
        p323.param_id_SET("DEMO", PH);
        p323.param_value_SET("DEMO", PH);
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT32);
        CommunicationChannel.instance.send(p323); //===============================
        PARAM_EXT_ACK p324 = CommunicationChannel.instance.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_id_SET("DEMO", PH);
        p324.param_value_SET("DEMO", PH);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_CUSTOM);
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_IN_PROGRESS);
        CommunicationChannel.instance.send(p324); //===============================
        OBSTACLE_DISTANCE p330 = CommunicationChannel.instance.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.time_usec_SET(1847838347288849813L);
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
        p330.distances_SET(new char[72], 0);
        p330.increment_SET((char)118);
        p330.min_distance_SET((char)46817);
        p330.max_distance_SET((char)38411);
        CommunicationChannel.instance.send(p330); //===============================
    }
}
