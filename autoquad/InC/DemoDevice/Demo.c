
#include "DemoDevice.h"
void failure(Channel * ch, int32_t id, int32_t arg) {}
void c_LoopBackDemoChannel_on_HEARTBEAT_0(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_TYPE  type = p0_type_GET(pack);
    e_MAV_AUTOPILOT  autopilot = p0_autopilot_GET(pack);
    e_MAV_MODE_FLAG  base_mode = p0_base_mode_GET(pack);
    uint32_t  custom_mode = p0_custom_mode_GET(pack);
    e_MAV_STATE  system_status = p0_system_status_GET(pack);
    uint8_t  mavlink_version = p0_mavlink_version_GET(pack);
}
void c_LoopBackDemoChannel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_SYS_STATUS_SENSOR  onboard_control_sensors_present = p1_onboard_control_sensors_present_GET(pack);
    e_MAV_SYS_STATUS_SENSOR  onboard_control_sensors_enabled = p1_onboard_control_sensors_enabled_GET(pack);
    e_MAV_SYS_STATUS_SENSOR  onboard_control_sensors_health = p1_onboard_control_sensors_health_GET(pack);
    uint16_t  load = p1_load_GET(pack);
    uint16_t  voltage_battery = p1_voltage_battery_GET(pack);
    int16_t  current_battery = p1_current_battery_GET(pack);
    int8_t  battery_remaining = p1_battery_remaining_GET(pack);
    uint16_t  drop_rate_comm = p1_drop_rate_comm_GET(pack);
    uint16_t  errors_comm = p1_errors_comm_GET(pack);
    uint16_t  errors_count1 = p1_errors_count1_GET(pack);
    uint16_t  errors_count2 = p1_errors_count2_GET(pack);
    uint16_t  errors_count3 = p1_errors_count3_GET(pack);
    uint16_t  errors_count4 = p1_errors_count4_GET(pack);
}
void c_LoopBackDemoChannel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_unix_usec = p2_time_unix_usec_GET(pack);
    uint32_t  time_boot_ms = p2_time_boot_ms_GET(pack);
}
void c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p3_time_boot_ms_GET(pack);
    e_MAV_FRAME  coordinate_frame = p3_coordinate_frame_GET(pack);
    uint16_t  type_mask = p3_type_mask_GET(pack);
    float  x = p3_x_GET(pack);
    float  y = p3_y_GET(pack);
    float  z = p3_z_GET(pack);
    float  vx = p3_vx_GET(pack);
    float  vy = p3_vy_GET(pack);
    float  vz = p3_vz_GET(pack);
    float  afx = p3_afx_GET(pack);
    float  afy = p3_afy_GET(pack);
    float  afz = p3_afz_GET(pack);
    float  yaw = p3_yaw_GET(pack);
    float  yaw_rate = p3_yaw_rate_GET(pack);
}
void c_LoopBackDemoChannel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p4_time_usec_GET(pack);
    uint32_t  seq = p4_seq_GET(pack);
    uint8_t  target_system = p4_target_system_GET(pack);
    uint8_t  target_component = p4_target_component_GET(pack);
}
void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p5_target_system_GET(pack);
    uint8_t  control_request = p5_control_request_GET(pack);
    uint8_t  version = p5_version_GET(pack);
    char16_t *  passkey = p5_passkey_TRY_(ph);
}
void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  gcs_system_id = p6_gcs_system_id_GET(pack);
    uint8_t  control_request = p6_control_request_GET(pack);
    uint8_t  ack = p6_ack_GET(pack);
}
void c_LoopBackDemoChannel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  key = p7_key_TRY_(ph);
}
void c_LoopBackDemoChannel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p11_target_system_GET(pack);
    e_MAV_MODE  base_mode = p11_base_mode_GET(pack);
    uint32_t  custom_mode = p11_custom_mode_GET(pack);
}
void c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p20_target_system_GET(pack);
    uint8_t  target_component = p20_target_component_GET(pack);
    char16_t *  param_id = p20_param_id_TRY_(ph);
    int16_t  param_index = p20_param_index_GET(pack);
}
void c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p21_target_system_GET(pack);
    uint8_t  target_component = p21_target_component_GET(pack);
}
void c_LoopBackDemoChannel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  param_id = p22_param_id_TRY_(ph);
    float  param_value = p22_param_value_GET(pack);
    e_MAV_PARAM_TYPE  param_type = p22_param_type_GET(pack);
    uint16_t  param_count = p22_param_count_GET(pack);
    uint16_t  param_index = p22_param_index_GET(pack);
}
void c_LoopBackDemoChannel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p23_target_system_GET(pack);
    uint8_t  target_component = p23_target_component_GET(pack);
    char16_t *  param_id = p23_param_id_TRY_(ph);
    float  param_value = p23_param_value_GET(pack);
    e_MAV_PARAM_TYPE  param_type = p23_param_type_GET(pack);
}
void c_LoopBackDemoChannel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p24_time_usec_GET(pack);
    e_GPS_FIX_TYPE  fix_type = p24_fix_type_GET(pack);
    int32_t  lat = p24_lat_GET(pack);
    int32_t  lon = p24_lon_GET(pack);
    int32_t  alt = p24_alt_GET(pack);
    uint16_t  eph = p24_eph_GET(pack);
    uint16_t  epv = p24_epv_GET(pack);
    uint16_t  vel = p24_vel_GET(pack);
    uint16_t  cog = p24_cog_GET(pack);
    uint8_t  satellites_visible = p24_satellites_visible_GET(pack);
    int32_t  alt_ellipsoid = p24_alt_ellipsoid_TRY(ph);
    uint32_t  h_acc = p24_h_acc_TRY(ph);
    uint32_t  v_acc = p24_v_acc_TRY(ph);
    uint32_t  vel_acc = p24_vel_acc_TRY(ph);
    uint32_t  hdg_acc = p24_hdg_acc_TRY(ph);
}
void c_LoopBackDemoChannel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  satellites_visible = p25_satellites_visible_GET(pack);
    uint8_t*  satellite_prn = p25_satellite_prn_GET_(pack);
//process data in satellite_prn
    free(satellite_prn);//never forget to dispose
    uint8_t*  satellite_used = p25_satellite_used_GET_(pack);
//process data in satellite_used
    free(satellite_used);//never forget to dispose
    uint8_t*  satellite_elevation = p25_satellite_elevation_GET_(pack);
//process data in satellite_elevation
    free(satellite_elevation);//never forget to dispose
    uint8_t*  satellite_azimuth = p25_satellite_azimuth_GET_(pack);
//process data in satellite_azimuth
    free(satellite_azimuth);//never forget to dispose
    uint8_t*  satellite_snr = p25_satellite_snr_GET_(pack);
//process data in satellite_snr
    free(satellite_snr);//never forget to dispose
}
void c_LoopBackDemoChannel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p26_time_boot_ms_GET(pack);
    int16_t  xacc = p26_xacc_GET(pack);
    int16_t  yacc = p26_yacc_GET(pack);
    int16_t  zacc = p26_zacc_GET(pack);
    int16_t  xgyro = p26_xgyro_GET(pack);
    int16_t  ygyro = p26_ygyro_GET(pack);
    int16_t  zgyro = p26_zgyro_GET(pack);
    int16_t  xmag = p26_xmag_GET(pack);
    int16_t  ymag = p26_ymag_GET(pack);
    int16_t  zmag = p26_zmag_GET(pack);
}
void c_LoopBackDemoChannel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p27_time_usec_GET(pack);
    int16_t  xacc = p27_xacc_GET(pack);
    int16_t  yacc = p27_yacc_GET(pack);
    int16_t  zacc = p27_zacc_GET(pack);
    int16_t  xgyro = p27_xgyro_GET(pack);
    int16_t  ygyro = p27_ygyro_GET(pack);
    int16_t  zgyro = p27_zgyro_GET(pack);
    int16_t  xmag = p27_xmag_GET(pack);
    int16_t  ymag = p27_ymag_GET(pack);
    int16_t  zmag = p27_zmag_GET(pack);
}
void c_LoopBackDemoChannel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p28_time_usec_GET(pack);
    int16_t  press_abs = p28_press_abs_GET(pack);
    int16_t  press_diff1 = p28_press_diff1_GET(pack);
    int16_t  press_diff2 = p28_press_diff2_GET(pack);
    int16_t  temperature = p28_temperature_GET(pack);
}
void c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p29_time_boot_ms_GET(pack);
    float  press_abs = p29_press_abs_GET(pack);
    float  press_diff = p29_press_diff_GET(pack);
    int16_t  temperature = p29_temperature_GET(pack);
}
void c_LoopBackDemoChannel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p30_time_boot_ms_GET(pack);
    float  roll = p30_roll_GET(pack);
    float  pitch = p30_pitch_GET(pack);
    float  yaw = p30_yaw_GET(pack);
    float  rollspeed = p30_rollspeed_GET(pack);
    float  pitchspeed = p30_pitchspeed_GET(pack);
    float  yawspeed = p30_yawspeed_GET(pack);
}
void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p31_time_boot_ms_GET(pack);
    float  q1 = p31_q1_GET(pack);
    float  q2 = p31_q2_GET(pack);
    float  q3 = p31_q3_GET(pack);
    float  q4 = p31_q4_GET(pack);
    float  rollspeed = p31_rollspeed_GET(pack);
    float  pitchspeed = p31_pitchspeed_GET(pack);
    float  yawspeed = p31_yawspeed_GET(pack);
}
void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p32_time_boot_ms_GET(pack);
    float  x = p32_x_GET(pack);
    float  y = p32_y_GET(pack);
    float  z = p32_z_GET(pack);
    float  vx = p32_vx_GET(pack);
    float  vy = p32_vy_GET(pack);
    float  vz = p32_vz_GET(pack);
}
void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p33_time_boot_ms_GET(pack);
    int32_t  lat = p33_lat_GET(pack);
    int32_t  lon = p33_lon_GET(pack);
    int32_t  alt = p33_alt_GET(pack);
    int32_t  relative_alt = p33_relative_alt_GET(pack);
    int16_t  vx = p33_vx_GET(pack);
    int16_t  vy = p33_vy_GET(pack);
    int16_t  vz = p33_vz_GET(pack);
    uint16_t  hdg = p33_hdg_GET(pack);
}
void c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p34_time_boot_ms_GET(pack);
    uint8_t  port = p34_port_GET(pack);
    int16_t  chan1_scaled = p34_chan1_scaled_GET(pack);
    int16_t  chan2_scaled = p34_chan2_scaled_GET(pack);
    int16_t  chan3_scaled = p34_chan3_scaled_GET(pack);
    int16_t  chan4_scaled = p34_chan4_scaled_GET(pack);
    int16_t  chan5_scaled = p34_chan5_scaled_GET(pack);
    int16_t  chan6_scaled = p34_chan6_scaled_GET(pack);
    int16_t  chan7_scaled = p34_chan7_scaled_GET(pack);
    int16_t  chan8_scaled = p34_chan8_scaled_GET(pack);
    uint8_t  rssi = p34_rssi_GET(pack);
}
void c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p35_time_boot_ms_GET(pack);
    uint8_t  port = p35_port_GET(pack);
    uint16_t  chan1_raw = p35_chan1_raw_GET(pack);
    uint16_t  chan2_raw = p35_chan2_raw_GET(pack);
    uint16_t  chan3_raw = p35_chan3_raw_GET(pack);
    uint16_t  chan4_raw = p35_chan4_raw_GET(pack);
    uint16_t  chan5_raw = p35_chan5_raw_GET(pack);
    uint16_t  chan6_raw = p35_chan6_raw_GET(pack);
    uint16_t  chan7_raw = p35_chan7_raw_GET(pack);
    uint16_t  chan8_raw = p35_chan8_raw_GET(pack);
    uint8_t  rssi = p35_rssi_GET(pack);
}
void c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_usec = p36_time_usec_GET(pack);
    uint8_t  port = p36_port_GET(pack);
    uint16_t  servo1_raw = p36_servo1_raw_GET(pack);
    uint16_t  servo2_raw = p36_servo2_raw_GET(pack);
    uint16_t  servo3_raw = p36_servo3_raw_GET(pack);
    uint16_t  servo4_raw = p36_servo4_raw_GET(pack);
    uint16_t  servo5_raw = p36_servo5_raw_GET(pack);
    uint16_t  servo6_raw = p36_servo6_raw_GET(pack);
    uint16_t  servo7_raw = p36_servo7_raw_GET(pack);
    uint16_t  servo8_raw = p36_servo8_raw_GET(pack);
    uint16_t  servo9_raw = p36_servo9_raw_TRY(ph);
    uint16_t  servo10_raw = p36_servo10_raw_TRY(ph);
    uint16_t  servo11_raw = p36_servo11_raw_TRY(ph);
    uint16_t  servo12_raw = p36_servo12_raw_TRY(ph);
    uint16_t  servo13_raw = p36_servo13_raw_TRY(ph);
    uint16_t  servo14_raw = p36_servo14_raw_TRY(ph);
    uint16_t  servo15_raw = p36_servo15_raw_TRY(ph);
    uint16_t  servo16_raw = p36_servo16_raw_TRY(ph);
}
void c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p37_target_system_GET(pack);
    uint8_t  target_component = p37_target_component_GET(pack);
    int16_t  start_index = p37_start_index_GET(pack);
    int16_t  end_index = p37_end_index_GET(pack);
    e_MAV_MISSION_TYPE  mission_type = p37_mission_type_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p38_target_system_GET(pack);
    uint8_t  target_component = p38_target_component_GET(pack);
    int16_t  start_index = p38_start_index_GET(pack);
    int16_t  end_index = p38_end_index_GET(pack);
    e_MAV_MISSION_TYPE  mission_type = p38_mission_type_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p39_target_system_GET(pack);
    uint8_t  target_component = p39_target_component_GET(pack);
    uint16_t  seq = p39_seq_GET(pack);
    e_MAV_FRAME  frame = p39_frame_GET(pack);
    e_MAV_CMD  command = p39_command_GET(pack);
    uint8_t  current = p39_current_GET(pack);
    uint8_t  autocontinue = p39_autocontinue_GET(pack);
    float  param1 = p39_param1_GET(pack);
    float  param2 = p39_param2_GET(pack);
    float  param3 = p39_param3_GET(pack);
    float  param4 = p39_param4_GET(pack);
    float  x = p39_x_GET(pack);
    float  y = p39_y_GET(pack);
    float  z = p39_z_GET(pack);
    e_MAV_MISSION_TYPE  mission_type = p39_mission_type_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p40_target_system_GET(pack);
    uint8_t  target_component = p40_target_component_GET(pack);
    uint16_t  seq = p40_seq_GET(pack);
    e_MAV_MISSION_TYPE  mission_type = p40_mission_type_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p41_target_system_GET(pack);
    uint8_t  target_component = p41_target_component_GET(pack);
    uint16_t  seq = p41_seq_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  seq = p42_seq_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p43_target_system_GET(pack);
    uint8_t  target_component = p43_target_component_GET(pack);
    e_MAV_MISSION_TYPE  mission_type = p43_mission_type_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p44_target_system_GET(pack);
    uint8_t  target_component = p44_target_component_GET(pack);
    uint16_t  count = p44_count_GET(pack);
    e_MAV_MISSION_TYPE  mission_type = p44_mission_type_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p45_target_system_GET(pack);
    uint8_t  target_component = p45_target_component_GET(pack);
    e_MAV_MISSION_TYPE  mission_type = p45_mission_type_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  seq = p46_seq_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p47_target_system_GET(pack);
    uint8_t  target_component = p47_target_component_GET(pack);
    e_MAV_MISSION_RESULT  type = p47_type_GET(pack);
    e_MAV_MISSION_TYPE  mission_type = p47_mission_type_GET(pack);
}
void c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p48_target_system_GET(pack);
    int32_t  latitude = p48_latitude_GET(pack);
    int32_t  longitude = p48_longitude_GET(pack);
    int32_t  altitude = p48_altitude_GET(pack);
    uint64_t  time_usec = p48_time_usec_TRY(ph);
}
void c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    int32_t  latitude = p49_latitude_GET(pack);
    int32_t  longitude = p49_longitude_GET(pack);
    int32_t  altitude = p49_altitude_GET(pack);
    uint64_t  time_usec = p49_time_usec_TRY(ph);
}
void c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p50_target_system_GET(pack);
    uint8_t  target_component = p50_target_component_GET(pack);
    char16_t *  param_id = p50_param_id_TRY_(ph);
    int16_t  param_index = p50_param_index_GET(pack);
    uint8_t  parameter_rc_channel_index = p50_parameter_rc_channel_index_GET(pack);
    float  param_value0 = p50_param_value0_GET(pack);
    float  scale = p50_scale_GET(pack);
    float  param_value_min = p50_param_value_min_GET(pack);
    float  param_value_max = p50_param_value_max_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p51_target_system_GET(pack);
    uint8_t  target_component = p51_target_component_GET(pack);
    uint16_t  seq = p51_seq_GET(pack);
    e_MAV_MISSION_TYPE  mission_type = p51_mission_type_GET(pack);
}
void c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p54_target_system_GET(pack);
    uint8_t  target_component = p54_target_component_GET(pack);
    e_MAV_FRAME  frame = p54_frame_GET(pack);
    float  p1x = p54_p1x_GET(pack);
    float  p1y = p54_p1y_GET(pack);
    float  p1z = p54_p1z_GET(pack);
    float  p2x = p54_p2x_GET(pack);
    float  p2y = p54_p2y_GET(pack);
    float  p2z = p54_p2z_GET(pack);
}
void c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_FRAME  frame = p55_frame_GET(pack);
    float  p1x = p55_p1x_GET(pack);
    float  p1y = p55_p1y_GET(pack);
    float  p1z = p55_p1z_GET(pack);
    float  p2x = p55_p2x_GET(pack);
    float  p2y = p55_p2y_GET(pack);
    float  p2z = p55_p2z_GET(pack);
}
void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p61_time_usec_GET(pack);
    float*  q = p61_q_GET_(pack);
//process data in q
    free(q);//never forget to dispose
    float  rollspeed = p61_rollspeed_GET(pack);
    float  pitchspeed = p61_pitchspeed_GET(pack);
    float  yawspeed = p61_yawspeed_GET(pack);
    float*  covariance = p61_covariance_GET_(pack);
//process data in covariance
    free(covariance);//never forget to dispose
}
void c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    float  nav_roll = p62_nav_roll_GET(pack);
    float  nav_pitch = p62_nav_pitch_GET(pack);
    int16_t  nav_bearing = p62_nav_bearing_GET(pack);
    int16_t  target_bearing = p62_target_bearing_GET(pack);
    uint16_t  wp_dist = p62_wp_dist_GET(pack);
    float  alt_error = p62_alt_error_GET(pack);
    float  aspd_error = p62_aspd_error_GET(pack);
    float  xtrack_error = p62_xtrack_error_GET(pack);
}
void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p63_time_usec_GET(pack);
    e_MAV_ESTIMATOR_TYPE  estimator_type = p63_estimator_type_GET(pack);
    int32_t  lat = p63_lat_GET(pack);
    int32_t  lon = p63_lon_GET(pack);
    int32_t  alt = p63_alt_GET(pack);
    int32_t  relative_alt = p63_relative_alt_GET(pack);
    float  vx = p63_vx_GET(pack);
    float  vy = p63_vy_GET(pack);
    float  vz = p63_vz_GET(pack);
    float*  covariance = p63_covariance_GET_(pack);
//process data in covariance
    free(covariance);//never forget to dispose
}
void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p64_time_usec_GET(pack);
    e_MAV_ESTIMATOR_TYPE  estimator_type = p64_estimator_type_GET(pack);
    float  x = p64_x_GET(pack);
    float  y = p64_y_GET(pack);
    float  z = p64_z_GET(pack);
    float  vx = p64_vx_GET(pack);
    float  vy = p64_vy_GET(pack);
    float  vz = p64_vz_GET(pack);
    float  ax = p64_ax_GET(pack);
    float  ay = p64_ay_GET(pack);
    float  az = p64_az_GET(pack);
    float*  covariance = p64_covariance_GET_(pack);
//process data in covariance
    free(covariance);//never forget to dispose
}
void c_LoopBackDemoChannel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p65_time_boot_ms_GET(pack);
    uint8_t  chancount = p65_chancount_GET(pack);
    uint16_t  chan1_raw = p65_chan1_raw_GET(pack);
    uint16_t  chan2_raw = p65_chan2_raw_GET(pack);
    uint16_t  chan3_raw = p65_chan3_raw_GET(pack);
    uint16_t  chan4_raw = p65_chan4_raw_GET(pack);
    uint16_t  chan5_raw = p65_chan5_raw_GET(pack);
    uint16_t  chan6_raw = p65_chan6_raw_GET(pack);
    uint16_t  chan7_raw = p65_chan7_raw_GET(pack);
    uint16_t  chan8_raw = p65_chan8_raw_GET(pack);
    uint16_t  chan9_raw = p65_chan9_raw_GET(pack);
    uint16_t  chan10_raw = p65_chan10_raw_GET(pack);
    uint16_t  chan11_raw = p65_chan11_raw_GET(pack);
    uint16_t  chan12_raw = p65_chan12_raw_GET(pack);
    uint16_t  chan13_raw = p65_chan13_raw_GET(pack);
    uint16_t  chan14_raw = p65_chan14_raw_GET(pack);
    uint16_t  chan15_raw = p65_chan15_raw_GET(pack);
    uint16_t  chan16_raw = p65_chan16_raw_GET(pack);
    uint16_t  chan17_raw = p65_chan17_raw_GET(pack);
    uint16_t  chan18_raw = p65_chan18_raw_GET(pack);
    uint8_t  rssi = p65_rssi_GET(pack);
}
void c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p66_target_system_GET(pack);
    uint8_t  target_component = p66_target_component_GET(pack);
    uint8_t  req_stream_id = p66_req_stream_id_GET(pack);
    uint16_t  req_message_rate = p66_req_message_rate_GET(pack);
    uint8_t  start_stop = p66_start_stop_GET(pack);
}
void c_LoopBackDemoChannel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  stream_id = p67_stream_id_GET(pack);
    uint16_t  message_rate = p67_message_rate_GET(pack);
    uint8_t  on_off = p67_on_off_GET(pack);
}
void c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target = p69_target_GET(pack);
    int16_t  x = p69_x_GET(pack);
    int16_t  y = p69_y_GET(pack);
    int16_t  z = p69_z_GET(pack);
    int16_t  r = p69_r_GET(pack);
    uint16_t  buttons = p69_buttons_GET(pack);
}
void c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p70_target_system_GET(pack);
    uint8_t  target_component = p70_target_component_GET(pack);
    uint16_t  chan1_raw = p70_chan1_raw_GET(pack);
    uint16_t  chan2_raw = p70_chan2_raw_GET(pack);
    uint16_t  chan3_raw = p70_chan3_raw_GET(pack);
    uint16_t  chan4_raw = p70_chan4_raw_GET(pack);
    uint16_t  chan5_raw = p70_chan5_raw_GET(pack);
    uint16_t  chan6_raw = p70_chan6_raw_GET(pack);
    uint16_t  chan7_raw = p70_chan7_raw_GET(pack);
    uint16_t  chan8_raw = p70_chan8_raw_GET(pack);
}
void c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p73_target_system_GET(pack);
    uint8_t  target_component = p73_target_component_GET(pack);
    uint16_t  seq = p73_seq_GET(pack);
    e_MAV_FRAME  frame = p73_frame_GET(pack);
    e_MAV_CMD  command = p73_command_GET(pack);
    uint8_t  current = p73_current_GET(pack);
    uint8_t  autocontinue = p73_autocontinue_GET(pack);
    float  param1 = p73_param1_GET(pack);
    float  param2 = p73_param2_GET(pack);
    float  param3 = p73_param3_GET(pack);
    float  param4 = p73_param4_GET(pack);
    int32_t  x = p73_x_GET(pack);
    int32_t  y = p73_y_GET(pack);
    float  z = p73_z_GET(pack);
    e_MAV_MISSION_TYPE  mission_type = p73_mission_type_GET(pack);
}
void c_LoopBackDemoChannel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    float  airspeed = p74_airspeed_GET(pack);
    float  groundspeed = p74_groundspeed_GET(pack);
    int16_t  heading = p74_heading_GET(pack);
    uint16_t  throttle = p74_throttle_GET(pack);
    float  alt = p74_alt_GET(pack);
    float  climb = p74_climb_GET(pack);
}
void c_LoopBackDemoChannel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p75_target_system_GET(pack);
    uint8_t  target_component = p75_target_component_GET(pack);
    e_MAV_FRAME  frame = p75_frame_GET(pack);
    e_MAV_CMD  command = p75_command_GET(pack);
    uint8_t  current = p75_current_GET(pack);
    uint8_t  autocontinue = p75_autocontinue_GET(pack);
    float  param1 = p75_param1_GET(pack);
    float  param2 = p75_param2_GET(pack);
    float  param3 = p75_param3_GET(pack);
    float  param4 = p75_param4_GET(pack);
    int32_t  x = p75_x_GET(pack);
    int32_t  y = p75_y_GET(pack);
    float  z = p75_z_GET(pack);
}
void c_LoopBackDemoChannel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p76_target_system_GET(pack);
    uint8_t  target_component = p76_target_component_GET(pack);
    e_MAV_CMD  command = p76_command_GET(pack);
    uint8_t  confirmation = p76_confirmation_GET(pack);
    float  param1 = p76_param1_GET(pack);
    float  param2 = p76_param2_GET(pack);
    float  param3 = p76_param3_GET(pack);
    float  param4 = p76_param4_GET(pack);
    float  param5 = p76_param5_GET(pack);
    float  param6 = p76_param6_GET(pack);
    float  param7 = p76_param7_GET(pack);
}
void c_LoopBackDemoChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_CMD  command = p77_command_GET(pack);
    e_MAV_RESULT  result = p77_result_GET(pack);
    uint8_t  progress = p77_progress_TRY(ph);
    int32_t  result_param2 = p77_result_param2_TRY(ph);
    uint8_t  target_system = p77_target_system_TRY(ph);
    uint8_t  target_component = p77_target_component_TRY(ph);
}
void c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p81_time_boot_ms_GET(pack);
    float  roll = p81_roll_GET(pack);
    float  pitch = p81_pitch_GET(pack);
    float  yaw = p81_yaw_GET(pack);
    float  thrust = p81_thrust_GET(pack);
    uint8_t  mode_switch = p81_mode_switch_GET(pack);
    uint8_t  manual_override_switch = p81_manual_override_switch_GET(pack);
}
void c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p82_time_boot_ms_GET(pack);
    uint8_t  target_system = p82_target_system_GET(pack);
    uint8_t  target_component = p82_target_component_GET(pack);
    uint8_t  type_mask = p82_type_mask_GET(pack);
    float*  q = p82_q_GET_(pack);
//process data in q
    free(q);//never forget to dispose
    float  body_roll_rate = p82_body_roll_rate_GET(pack);
    float  body_pitch_rate = p82_body_pitch_rate_GET(pack);
    float  body_yaw_rate = p82_body_yaw_rate_GET(pack);
    float  thrust = p82_thrust_GET(pack);
}
void c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p83_time_boot_ms_GET(pack);
    uint8_t  type_mask = p83_type_mask_GET(pack);
    float*  q = p83_q_GET_(pack);
//process data in q
    free(q);//never forget to dispose
    float  body_roll_rate = p83_body_roll_rate_GET(pack);
    float  body_pitch_rate = p83_body_pitch_rate_GET(pack);
    float  body_yaw_rate = p83_body_yaw_rate_GET(pack);
    float  thrust = p83_thrust_GET(pack);
}
void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p84_time_boot_ms_GET(pack);
    uint8_t  target_system = p84_target_system_GET(pack);
    uint8_t  target_component = p84_target_component_GET(pack);
    e_MAV_FRAME  coordinate_frame = p84_coordinate_frame_GET(pack);
    uint16_t  type_mask = p84_type_mask_GET(pack);
    float  x = p84_x_GET(pack);
    float  y = p84_y_GET(pack);
    float  z = p84_z_GET(pack);
    float  vx = p84_vx_GET(pack);
    float  vy = p84_vy_GET(pack);
    float  vz = p84_vz_GET(pack);
    float  afx = p84_afx_GET(pack);
    float  afy = p84_afy_GET(pack);
    float  afz = p84_afz_GET(pack);
    float  yaw = p84_yaw_GET(pack);
    float  yaw_rate = p84_yaw_rate_GET(pack);
}
void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p86_time_boot_ms_GET(pack);
    uint8_t  target_system = p86_target_system_GET(pack);
    uint8_t  target_component = p86_target_component_GET(pack);
    e_MAV_FRAME  coordinate_frame = p86_coordinate_frame_GET(pack);
    uint16_t  type_mask = p86_type_mask_GET(pack);
    int32_t  lat_int = p86_lat_int_GET(pack);
    int32_t  lon_int = p86_lon_int_GET(pack);
    float  alt = p86_alt_GET(pack);
    float  vx = p86_vx_GET(pack);
    float  vy = p86_vy_GET(pack);
    float  vz = p86_vz_GET(pack);
    float  afx = p86_afx_GET(pack);
    float  afy = p86_afy_GET(pack);
    float  afz = p86_afz_GET(pack);
    float  yaw = p86_yaw_GET(pack);
    float  yaw_rate = p86_yaw_rate_GET(pack);
}
void c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p87_time_boot_ms_GET(pack);
    e_MAV_FRAME  coordinate_frame = p87_coordinate_frame_GET(pack);
    uint16_t  type_mask = p87_type_mask_GET(pack);
    int32_t  lat_int = p87_lat_int_GET(pack);
    int32_t  lon_int = p87_lon_int_GET(pack);
    float  alt = p87_alt_GET(pack);
    float  vx = p87_vx_GET(pack);
    float  vy = p87_vy_GET(pack);
    float  vz = p87_vz_GET(pack);
    float  afx = p87_afx_GET(pack);
    float  afy = p87_afy_GET(pack);
    float  afz = p87_afz_GET(pack);
    float  yaw = p87_yaw_GET(pack);
    float  yaw_rate = p87_yaw_rate_GET(pack);
}
void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p89_time_boot_ms_GET(pack);
    float  x = p89_x_GET(pack);
    float  y = p89_y_GET(pack);
    float  z = p89_z_GET(pack);
    float  roll = p89_roll_GET(pack);
    float  pitch = p89_pitch_GET(pack);
    float  yaw = p89_yaw_GET(pack);
}
void c_LoopBackDemoChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p90_time_usec_GET(pack);
    float  roll = p90_roll_GET(pack);
    float  pitch = p90_pitch_GET(pack);
    float  yaw = p90_yaw_GET(pack);
    float  rollspeed = p90_rollspeed_GET(pack);
    float  pitchspeed = p90_pitchspeed_GET(pack);
    float  yawspeed = p90_yawspeed_GET(pack);
    int32_t  lat = p90_lat_GET(pack);
    int32_t  lon = p90_lon_GET(pack);
    int32_t  alt = p90_alt_GET(pack);
    int16_t  vx = p90_vx_GET(pack);
    int16_t  vy = p90_vy_GET(pack);
    int16_t  vz = p90_vz_GET(pack);
    int16_t  xacc = p90_xacc_GET(pack);
    int16_t  yacc = p90_yacc_GET(pack);
    int16_t  zacc = p90_zacc_GET(pack);
}
void c_LoopBackDemoChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p91_time_usec_GET(pack);
    float  roll_ailerons = p91_roll_ailerons_GET(pack);
    float  pitch_elevator = p91_pitch_elevator_GET(pack);
    float  yaw_rudder = p91_yaw_rudder_GET(pack);
    float  throttle = p91_throttle_GET(pack);
    float  aux1 = p91_aux1_GET(pack);
    float  aux2 = p91_aux2_GET(pack);
    float  aux3 = p91_aux3_GET(pack);
    float  aux4 = p91_aux4_GET(pack);
    e_MAV_MODE  mode = p91_mode_GET(pack);
    uint8_t  nav_mode = p91_nav_mode_GET(pack);
}
void c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p92_time_usec_GET(pack);
    uint16_t  chan1_raw = p92_chan1_raw_GET(pack);
    uint16_t  chan2_raw = p92_chan2_raw_GET(pack);
    uint16_t  chan3_raw = p92_chan3_raw_GET(pack);
    uint16_t  chan4_raw = p92_chan4_raw_GET(pack);
    uint16_t  chan5_raw = p92_chan5_raw_GET(pack);
    uint16_t  chan6_raw = p92_chan6_raw_GET(pack);
    uint16_t  chan7_raw = p92_chan7_raw_GET(pack);
    uint16_t  chan8_raw = p92_chan8_raw_GET(pack);
    uint16_t  chan9_raw = p92_chan9_raw_GET(pack);
    uint16_t  chan10_raw = p92_chan10_raw_GET(pack);
    uint16_t  chan11_raw = p92_chan11_raw_GET(pack);
    uint16_t  chan12_raw = p92_chan12_raw_GET(pack);
    uint8_t  rssi = p92_rssi_GET(pack);
}
void c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p93_time_usec_GET(pack);
    float*  controls = p93_controls_GET_(pack);
//process data in controls
    free(controls);//never forget to dispose
    e_MAV_MODE  mode = p93_mode_GET(pack);
    uint64_t  flags = p93_flags_GET(pack);
}
void c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p100_time_usec_GET(pack);
    uint8_t  sensor_id = p100_sensor_id_GET(pack);
    int16_t  flow_x = p100_flow_x_GET(pack);
    int16_t  flow_y = p100_flow_y_GET(pack);
    float  flow_comp_m_x = p100_flow_comp_m_x_GET(pack);
    float  flow_comp_m_y = p100_flow_comp_m_y_GET(pack);
    uint8_t  quality = p100_quality_GET(pack);
    float  ground_distance = p100_ground_distance_GET(pack);
    float  flow_rate_x = p100_flow_rate_x_TRY(ph);
    float  flow_rate_y = p100_flow_rate_y_TRY(ph);
}
void c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  usec = p101_usec_GET(pack);
    float  x = p101_x_GET(pack);
    float  y = p101_y_GET(pack);
    float  z = p101_z_GET(pack);
    float  roll = p101_roll_GET(pack);
    float  pitch = p101_pitch_GET(pack);
    float  yaw = p101_yaw_GET(pack);
}
void c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  usec = p102_usec_GET(pack);
    float  x = p102_x_GET(pack);
    float  y = p102_y_GET(pack);
    float  z = p102_z_GET(pack);
    float  roll = p102_roll_GET(pack);
    float  pitch = p102_pitch_GET(pack);
    float  yaw = p102_yaw_GET(pack);
}
void c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  usec = p103_usec_GET(pack);
    float  x = p103_x_GET(pack);
    float  y = p103_y_GET(pack);
    float  z = p103_z_GET(pack);
}
void c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  usec = p104_usec_GET(pack);
    float  x = p104_x_GET(pack);
    float  y = p104_y_GET(pack);
    float  z = p104_z_GET(pack);
    float  roll = p104_roll_GET(pack);
    float  pitch = p104_pitch_GET(pack);
    float  yaw = p104_yaw_GET(pack);
}
void c_LoopBackDemoChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p105_time_usec_GET(pack);
    float  xacc = p105_xacc_GET(pack);
    float  yacc = p105_yacc_GET(pack);
    float  zacc = p105_zacc_GET(pack);
    float  xgyro = p105_xgyro_GET(pack);
    float  ygyro = p105_ygyro_GET(pack);
    float  zgyro = p105_zgyro_GET(pack);
    float  xmag = p105_xmag_GET(pack);
    float  ymag = p105_ymag_GET(pack);
    float  zmag = p105_zmag_GET(pack);
    float  abs_pressure = p105_abs_pressure_GET(pack);
    float  diff_pressure = p105_diff_pressure_GET(pack);
    float  pressure_alt = p105_pressure_alt_GET(pack);
    float  temperature = p105_temperature_GET(pack);
    uint16_t  fields_updated = p105_fields_updated_GET(pack);
}
void c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p106_time_usec_GET(pack);
    uint8_t  sensor_id = p106_sensor_id_GET(pack);
    uint32_t  integration_time_us = p106_integration_time_us_GET(pack);
    float  integrated_x = p106_integrated_x_GET(pack);
    float  integrated_y = p106_integrated_y_GET(pack);
    float  integrated_xgyro = p106_integrated_xgyro_GET(pack);
    float  integrated_ygyro = p106_integrated_ygyro_GET(pack);
    float  integrated_zgyro = p106_integrated_zgyro_GET(pack);
    int16_t  temperature = p106_temperature_GET(pack);
    uint8_t  quality = p106_quality_GET(pack);
    uint32_t  time_delta_distance_us = p106_time_delta_distance_us_GET(pack);
    float  distance = p106_distance_GET(pack);
}
void c_LoopBackDemoChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p107_time_usec_GET(pack);
    float  xacc = p107_xacc_GET(pack);
    float  yacc = p107_yacc_GET(pack);
    float  zacc = p107_zacc_GET(pack);
    float  xgyro = p107_xgyro_GET(pack);
    float  ygyro = p107_ygyro_GET(pack);
    float  zgyro = p107_zgyro_GET(pack);
    float  xmag = p107_xmag_GET(pack);
    float  ymag = p107_ymag_GET(pack);
    float  zmag = p107_zmag_GET(pack);
    float  abs_pressure = p107_abs_pressure_GET(pack);
    float  diff_pressure = p107_diff_pressure_GET(pack);
    float  pressure_alt = p107_pressure_alt_GET(pack);
    float  temperature = p107_temperature_GET(pack);
    uint32_t  fields_updated = p107_fields_updated_GET(pack);
}
void c_LoopBackDemoChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    float  q1 = p108_q1_GET(pack);
    float  q2 = p108_q2_GET(pack);
    float  q3 = p108_q3_GET(pack);
    float  q4 = p108_q4_GET(pack);
    float  roll = p108_roll_GET(pack);
    float  pitch = p108_pitch_GET(pack);
    float  yaw = p108_yaw_GET(pack);
    float  xacc = p108_xacc_GET(pack);
    float  yacc = p108_yacc_GET(pack);
    float  zacc = p108_zacc_GET(pack);
    float  xgyro = p108_xgyro_GET(pack);
    float  ygyro = p108_ygyro_GET(pack);
    float  zgyro = p108_zgyro_GET(pack);
    float  lat = p108_lat_GET(pack);
    float  lon = p108_lon_GET(pack);
    float  alt = p108_alt_GET(pack);
    float  std_dev_horz = p108_std_dev_horz_GET(pack);
    float  std_dev_vert = p108_std_dev_vert_GET(pack);
    float  vn = p108_vn_GET(pack);
    float  ve = p108_ve_GET(pack);
    float  vd = p108_vd_GET(pack);
}
void c_LoopBackDemoChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  rssi = p109_rssi_GET(pack);
    uint8_t  remrssi = p109_remrssi_GET(pack);
    uint8_t  txbuf = p109_txbuf_GET(pack);
    uint8_t  noise = p109_noise_GET(pack);
    uint8_t  remnoise = p109_remnoise_GET(pack);
    uint16_t  rxerrors = p109_rxerrors_GET(pack);
    uint16_t  fixed_ = p109_fixed__GET(pack);
}
void c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_network = p110_target_network_GET(pack);
    uint8_t  target_system = p110_target_system_GET(pack);
    uint8_t  target_component = p110_target_component_GET(pack);
    uint8_t*  payload = p110_payload_GET_(pack);
//process data in payload
    free(payload);//never forget to dispose
}
void c_LoopBackDemoChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    int64_t  tc1 = p111_tc1_GET(pack);
    int64_t  ts1 = p111_ts1_GET(pack);
}
void c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p112_time_usec_GET(pack);
    uint32_t  seq = p112_seq_GET(pack);
}
void c_LoopBackDemoChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p113_time_usec_GET(pack);
    uint8_t  fix_type = p113_fix_type_GET(pack);
    int32_t  lat = p113_lat_GET(pack);
    int32_t  lon = p113_lon_GET(pack);
    int32_t  alt = p113_alt_GET(pack);
    uint16_t  eph = p113_eph_GET(pack);
    uint16_t  epv = p113_epv_GET(pack);
    uint16_t  vel = p113_vel_GET(pack);
    int16_t  vn = p113_vn_GET(pack);
    int16_t  ve = p113_ve_GET(pack);
    int16_t  vd = p113_vd_GET(pack);
    uint16_t  cog = p113_cog_GET(pack);
    uint8_t  satellites_visible = p113_satellites_visible_GET(pack);
}
void c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p114_time_usec_GET(pack);
    uint8_t  sensor_id = p114_sensor_id_GET(pack);
    uint32_t  integration_time_us = p114_integration_time_us_GET(pack);
    float  integrated_x = p114_integrated_x_GET(pack);
    float  integrated_y = p114_integrated_y_GET(pack);
    float  integrated_xgyro = p114_integrated_xgyro_GET(pack);
    float  integrated_ygyro = p114_integrated_ygyro_GET(pack);
    float  integrated_zgyro = p114_integrated_zgyro_GET(pack);
    int16_t  temperature = p114_temperature_GET(pack);
    uint8_t  quality = p114_quality_GET(pack);
    uint32_t  time_delta_distance_us = p114_time_delta_distance_us_GET(pack);
    float  distance = p114_distance_GET(pack);
}
void c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p115_time_usec_GET(pack);
    float*  attitude_quaternion = p115_attitude_quaternion_GET_(pack);
//process data in attitude_quaternion
    free(attitude_quaternion);//never forget to dispose
    float  rollspeed = p115_rollspeed_GET(pack);
    float  pitchspeed = p115_pitchspeed_GET(pack);
    float  yawspeed = p115_yawspeed_GET(pack);
    int32_t  lat = p115_lat_GET(pack);
    int32_t  lon = p115_lon_GET(pack);
    int32_t  alt = p115_alt_GET(pack);
    int16_t  vx = p115_vx_GET(pack);
    int16_t  vy = p115_vy_GET(pack);
    int16_t  vz = p115_vz_GET(pack);
    uint16_t  ind_airspeed = p115_ind_airspeed_GET(pack);
    uint16_t  true_airspeed = p115_true_airspeed_GET(pack);
    int16_t  xacc = p115_xacc_GET(pack);
    int16_t  yacc = p115_yacc_GET(pack);
    int16_t  zacc = p115_zacc_GET(pack);
}
void c_LoopBackDemoChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p116_time_boot_ms_GET(pack);
    int16_t  xacc = p116_xacc_GET(pack);
    int16_t  yacc = p116_yacc_GET(pack);
    int16_t  zacc = p116_zacc_GET(pack);
    int16_t  xgyro = p116_xgyro_GET(pack);
    int16_t  ygyro = p116_ygyro_GET(pack);
    int16_t  zgyro = p116_zgyro_GET(pack);
    int16_t  xmag = p116_xmag_GET(pack);
    int16_t  ymag = p116_ymag_GET(pack);
    int16_t  zmag = p116_zmag_GET(pack);
}
void c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p117_target_system_GET(pack);
    uint8_t  target_component = p117_target_component_GET(pack);
    uint16_t  start = p117_start_GET(pack);
    uint16_t  end = p117_end_GET(pack);
}
void c_LoopBackDemoChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  id = p118_id_GET(pack);
    uint16_t  num_logs = p118_num_logs_GET(pack);
    uint16_t  last_log_num = p118_last_log_num_GET(pack);
    uint32_t  time_utc = p118_time_utc_GET(pack);
    uint32_t  size = p118_size_GET(pack);
}
void c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p119_target_system_GET(pack);
    uint8_t  target_component = p119_target_component_GET(pack);
    uint16_t  id = p119_id_GET(pack);
    uint32_t  ofs = p119_ofs_GET(pack);
    uint32_t  count = p119_count_GET(pack);
}
void c_LoopBackDemoChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  id = p120_id_GET(pack);
    uint32_t  ofs = p120_ofs_GET(pack);
    uint8_t  count = p120_count_GET(pack);
    uint8_t*  data_ = p120_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p121_target_system_GET(pack);
    uint8_t  target_component = p121_target_component_GET(pack);
}
void c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p122_target_system_GET(pack);
    uint8_t  target_component = p122_target_component_GET(pack);
}
void c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p123_target_system_GET(pack);
    uint8_t  target_component = p123_target_component_GET(pack);
    uint8_t  len = p123_len_GET(pack);
    uint8_t*  data_ = p123_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p124_time_usec_GET(pack);
    e_GPS_FIX_TYPE  fix_type = p124_fix_type_GET(pack);
    int32_t  lat = p124_lat_GET(pack);
    int32_t  lon = p124_lon_GET(pack);
    int32_t  alt = p124_alt_GET(pack);
    uint16_t  eph = p124_eph_GET(pack);
    uint16_t  epv = p124_epv_GET(pack);
    uint16_t  vel = p124_vel_GET(pack);
    uint16_t  cog = p124_cog_GET(pack);
    uint8_t  satellites_visible = p124_satellites_visible_GET(pack);
    uint8_t  dgps_numch = p124_dgps_numch_GET(pack);
    uint32_t  dgps_age = p124_dgps_age_GET(pack);
}
void c_LoopBackDemoChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  Vcc = p125_Vcc_GET(pack);
    uint16_t  Vservo = p125_Vservo_GET(pack);
    e_MAV_POWER_STATUS  flags = p125_flags_GET(pack);
}
void c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    e_SERIAL_CONTROL_DEV  device = p126_device_GET(pack);
    e_SERIAL_CONTROL_FLAG  flags = p126_flags_GET(pack);
    uint16_t  timeout = p126_timeout_GET(pack);
    uint32_t  baudrate = p126_baudrate_GET(pack);
    uint8_t  count = p126_count_GET(pack);
    uint8_t*  data_ = p126_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_last_baseline_ms = p127_time_last_baseline_ms_GET(pack);
    uint8_t  rtk_receiver_id = p127_rtk_receiver_id_GET(pack);
    uint16_t  wn = p127_wn_GET(pack);
    uint32_t  tow = p127_tow_GET(pack);
    uint8_t  rtk_health = p127_rtk_health_GET(pack);
    uint8_t  rtk_rate = p127_rtk_rate_GET(pack);
    uint8_t  nsats = p127_nsats_GET(pack);
    uint8_t  baseline_coords_type = p127_baseline_coords_type_GET(pack);
    int32_t  baseline_a_mm = p127_baseline_a_mm_GET(pack);
    int32_t  baseline_b_mm = p127_baseline_b_mm_GET(pack);
    int32_t  baseline_c_mm = p127_baseline_c_mm_GET(pack);
    uint32_t  accuracy = p127_accuracy_GET(pack);
    int32_t  iar_num_hypotheses = p127_iar_num_hypotheses_GET(pack);
}
void c_LoopBackDemoChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_last_baseline_ms = p128_time_last_baseline_ms_GET(pack);
    uint8_t  rtk_receiver_id = p128_rtk_receiver_id_GET(pack);
    uint16_t  wn = p128_wn_GET(pack);
    uint32_t  tow = p128_tow_GET(pack);
    uint8_t  rtk_health = p128_rtk_health_GET(pack);
    uint8_t  rtk_rate = p128_rtk_rate_GET(pack);
    uint8_t  nsats = p128_nsats_GET(pack);
    uint8_t  baseline_coords_type = p128_baseline_coords_type_GET(pack);
    int32_t  baseline_a_mm = p128_baseline_a_mm_GET(pack);
    int32_t  baseline_b_mm = p128_baseline_b_mm_GET(pack);
    int32_t  baseline_c_mm = p128_baseline_c_mm_GET(pack);
    uint32_t  accuracy = p128_accuracy_GET(pack);
    int32_t  iar_num_hypotheses = p128_iar_num_hypotheses_GET(pack);
}
void c_LoopBackDemoChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p129_time_boot_ms_GET(pack);
    int16_t  xacc = p129_xacc_GET(pack);
    int16_t  yacc = p129_yacc_GET(pack);
    int16_t  zacc = p129_zacc_GET(pack);
    int16_t  xgyro = p129_xgyro_GET(pack);
    int16_t  ygyro = p129_ygyro_GET(pack);
    int16_t  zgyro = p129_zgyro_GET(pack);
    int16_t  xmag = p129_xmag_GET(pack);
    int16_t  ymag = p129_ymag_GET(pack);
    int16_t  zmag = p129_zmag_GET(pack);
}
void c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  type = p130_type_GET(pack);
    uint32_t  size = p130_size_GET(pack);
    uint16_t  width = p130_width_GET(pack);
    uint16_t  height = p130_height_GET(pack);
    uint16_t  packets = p130_packets_GET(pack);
    uint8_t  payload = p130_payload_GET(pack);
    uint8_t  jpg_quality = p130_jpg_quality_GET(pack);
}
void c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  seqnr = p131_seqnr_GET(pack);
    uint8_t*  data_ = p131_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p132_time_boot_ms_GET(pack);
    uint16_t  min_distance = p132_min_distance_GET(pack);
    uint16_t  max_distance = p132_max_distance_GET(pack);
    uint16_t  current_distance = p132_current_distance_GET(pack);
    e_MAV_DISTANCE_SENSOR  type = p132_type_GET(pack);
    uint8_t  id = p132_id_GET(pack);
    e_MAV_SENSOR_ORIENTATION  orientation = p132_orientation_GET(pack);
    uint8_t  covariance = p132_covariance_GET(pack);
}
void c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    int32_t  lat = p133_lat_GET(pack);
    int32_t  lon = p133_lon_GET(pack);
    uint16_t  grid_spacing = p133_grid_spacing_GET(pack);
    uint64_t  mask = p133_mask_GET(pack);
}
void c_LoopBackDemoChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    int32_t  lat = p134_lat_GET(pack);
    int32_t  lon = p134_lon_GET(pack);
    uint16_t  grid_spacing = p134_grid_spacing_GET(pack);
    uint8_t  gridbit = p134_gridbit_GET(pack);
    int16_t*  data_ = p134_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    int32_t  lat = p135_lat_GET(pack);
    int32_t  lon = p135_lon_GET(pack);
}
void c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    int32_t  lat = p136_lat_GET(pack);
    int32_t  lon = p136_lon_GET(pack);
    uint16_t  spacing = p136_spacing_GET(pack);
    float  terrain_height = p136_terrain_height_GET(pack);
    float  current_height = p136_current_height_GET(pack);
    uint16_t  pending = p136_pending_GET(pack);
    uint16_t  loaded = p136_loaded_GET(pack);
}
void c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p137_time_boot_ms_GET(pack);
    float  press_abs = p137_press_abs_GET(pack);
    float  press_diff = p137_press_diff_GET(pack);
    int16_t  temperature = p137_temperature_GET(pack);
}
void c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p138_time_usec_GET(pack);
    float*  q = p138_q_GET_(pack);
//process data in q
    free(q);//never forget to dispose
    float  x = p138_x_GET(pack);
    float  y = p138_y_GET(pack);
    float  z = p138_z_GET(pack);
}
void c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p139_time_usec_GET(pack);
    uint8_t  group_mlx = p139_group_mlx_GET(pack);
    uint8_t  target_system = p139_target_system_GET(pack);
    uint8_t  target_component = p139_target_component_GET(pack);
    float*  controls = p139_controls_GET_(pack);
//process data in controls
    free(controls);//never forget to dispose
}
void c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p140_time_usec_GET(pack);
    uint8_t  group_mlx = p140_group_mlx_GET(pack);
    float*  controls = p140_controls_GET_(pack);
//process data in controls
    free(controls);//never forget to dispose
}
void c_LoopBackDemoChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p141_time_usec_GET(pack);
    float  altitude_monotonic = p141_altitude_monotonic_GET(pack);
    float  altitude_amsl = p141_altitude_amsl_GET(pack);
    float  altitude_local = p141_altitude_local_GET(pack);
    float  altitude_relative = p141_altitude_relative_GET(pack);
    float  altitude_terrain = p141_altitude_terrain_GET(pack);
    float  bottom_clearance = p141_bottom_clearance_GET(pack);
}
void c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  request_id = p142_request_id_GET(pack);
    uint8_t  uri_type = p142_uri_type_GET(pack);
    uint8_t*  uri = p142_uri_GET_(pack);
//process data in uri
    free(uri);//never forget to dispose
    uint8_t  transfer_type = p142_transfer_type_GET(pack);
    uint8_t*  storage = p142_storage_GET_(pack);
//process data in storage
    free(storage);//never forget to dispose
}
void c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p143_time_boot_ms_GET(pack);
    float  press_abs = p143_press_abs_GET(pack);
    float  press_diff = p143_press_diff_GET(pack);
    int16_t  temperature = p143_temperature_GET(pack);
}
void c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  timestamp = p144_timestamp_GET(pack);
    uint8_t  est_capabilities = p144_est_capabilities_GET(pack);
    int32_t  lat = p144_lat_GET(pack);
    int32_t  lon = p144_lon_GET(pack);
    float  alt = p144_alt_GET(pack);
    float*  vel = p144_vel_GET_(pack);
//process data in vel
    free(vel);//never forget to dispose
    float*  acc = p144_acc_GET_(pack);
//process data in acc
    free(acc);//never forget to dispose
    float*  attitude_q = p144_attitude_q_GET_(pack);
//process data in attitude_q
    free(attitude_q);//never forget to dispose
    float*  rates = p144_rates_GET_(pack);
//process data in rates
    free(rates);//never forget to dispose
    float*  position_cov = p144_position_cov_GET_(pack);
//process data in position_cov
    free(position_cov);//never forget to dispose
    uint64_t  custom_state = p144_custom_state_GET(pack);
}
void c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p146_time_usec_GET(pack);
    float  x_acc = p146_x_acc_GET(pack);
    float  y_acc = p146_y_acc_GET(pack);
    float  z_acc = p146_z_acc_GET(pack);
    float  x_vel = p146_x_vel_GET(pack);
    float  y_vel = p146_y_vel_GET(pack);
    float  z_vel = p146_z_vel_GET(pack);
    float  x_pos = p146_x_pos_GET(pack);
    float  y_pos = p146_y_pos_GET(pack);
    float  z_pos = p146_z_pos_GET(pack);
    float  airspeed = p146_airspeed_GET(pack);
    float*  vel_variance = p146_vel_variance_GET_(pack);
//process data in vel_variance
    free(vel_variance);//never forget to dispose
    float*  pos_variance = p146_pos_variance_GET_(pack);
//process data in pos_variance
    free(pos_variance);//never forget to dispose
    float*  q = p146_q_GET_(pack);
//process data in q
    free(q);//never forget to dispose
    float  roll_rate = p146_roll_rate_GET(pack);
    float  pitch_rate = p146_pitch_rate_GET(pack);
    float  yaw_rate = p146_yaw_rate_GET(pack);
}
void c_LoopBackDemoChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  id = p147_id_GET(pack);
    e_MAV_BATTERY_FUNCTION  battery_function = p147_battery_function_GET(pack);
    e_MAV_BATTERY_TYPE  type = p147_type_GET(pack);
    int16_t  temperature = p147_temperature_GET(pack);
    uint16_t*  voltages = p147_voltages_GET_(pack);
//process data in voltages
    free(voltages);//never forget to dispose
    int16_t  current_battery = p147_current_battery_GET(pack);
    int32_t  current_consumed = p147_current_consumed_GET(pack);
    int32_t  energy_consumed = p147_energy_consumed_GET(pack);
    int8_t  battery_remaining = p147_battery_remaining_GET(pack);
}
void c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_PROTOCOL_CAPABILITY  capabilities = p148_capabilities_GET(pack);
    uint32_t  flight_sw_version = p148_flight_sw_version_GET(pack);
    uint32_t  middleware_sw_version = p148_middleware_sw_version_GET(pack);
    uint32_t  os_sw_version = p148_os_sw_version_GET(pack);
    uint32_t  board_version = p148_board_version_GET(pack);
    uint8_t*  flight_custom_version = p148_flight_custom_version_GET_(pack);
//process data in flight_custom_version
    free(flight_custom_version);//never forget to dispose
    uint8_t*  middleware_custom_version = p148_middleware_custom_version_GET_(pack);
//process data in middleware_custom_version
    free(middleware_custom_version);//never forget to dispose
    uint8_t*  os_custom_version = p148_os_custom_version_GET_(pack);
//process data in os_custom_version
    free(os_custom_version);//never forget to dispose
    uint16_t  vendor_id = p148_vendor_id_GET(pack);
    uint16_t  product_id = p148_product_id_GET(pack);
    uint64_t  uid = p148_uid_GET(pack);
    uint8_t*  uid2 = p148_uid2_TRY(ph);
}
void c_LoopBackDemoChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p149_time_usec_GET(pack);
    uint8_t  target_num = p149_target_num_GET(pack);
    e_MAV_FRAME  frame = p149_frame_GET(pack);
    float  angle_x = p149_angle_x_GET(pack);
    float  angle_y = p149_angle_y_GET(pack);
    float  distance = p149_distance_GET(pack);
    float  size_x = p149_size_x_GET(pack);
    float  size_y = p149_size_y_GET(pack);
    float  x = p149_x_TRY(ph);
    float  y = p149_y_TRY(ph);
    float  z = p149_z_TRY(ph);
    float*  q = p149_q_TRY(ph);
    e_LANDING_TARGET_TYPE  type = p149_type_GET(pack);
    uint8_t  position_valid = p149_position_valid_TRY(ph);
}
void c_LoopBackDemoChannel_on_AQ_TELEMETRY_F_150(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  Index = p150_Index_GET(pack);
    float  value1 = p150_value1_GET(pack);
    float  value2 = p150_value2_GET(pack);
    float  value3 = p150_value3_GET(pack);
    float  value4 = p150_value4_GET(pack);
    float  value5 = p150_value5_GET(pack);
    float  value6 = p150_value6_GET(pack);
    float  value7 = p150_value7_GET(pack);
    float  value8 = p150_value8_GET(pack);
    float  value9 = p150_value9_GET(pack);
    float  value10 = p150_value10_GET(pack);
    float  value11 = p150_value11_GET(pack);
    float  value12 = p150_value12_GET(pack);
    float  value13 = p150_value13_GET(pack);
    float  value14 = p150_value14_GET(pack);
    float  value15 = p150_value15_GET(pack);
    float  value16 = p150_value16_GET(pack);
    float  value17 = p150_value17_GET(pack);
    float  value18 = p150_value18_GET(pack);
    float  value19 = p150_value19_GET(pack);
    float  value20 = p150_value20_GET(pack);
}
void c_LoopBackDemoChannel_on_AQ_ESC_TELEMETRY_152(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p152_time_boot_ms_GET(pack);
    uint8_t  seq = p152_seq_GET(pack);
    uint8_t  num_motors = p152_num_motors_GET(pack);
    uint8_t  num_in_seq = p152_num_in_seq_GET(pack);
    uint8_t*  escid = p152_escid_GET_(pack);
//process data in escid
    free(escid);//never forget to dispose
    uint16_t*  status_age = p152_status_age_GET_(pack);
//process data in status_age
    free(status_age);//never forget to dispose
    uint8_t*  data_version = p152_data_version_GET_(pack);
//process data in data_version
    free(data_version);//never forget to dispose
    uint32_t*  data0 = p152_data0_GET_(pack);
//process data in data0
    free(data0);//never forget to dispose
    uint32_t*  data1 = p152_data1_GET_(pack);
//process data in data1
    free(data1);//never forget to dispose
}
void c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p230_time_usec_GET(pack);
    e_ESTIMATOR_STATUS_FLAGS  flags = p230_flags_GET(pack);
    float  vel_ratio = p230_vel_ratio_GET(pack);
    float  pos_horiz_ratio = p230_pos_horiz_ratio_GET(pack);
    float  pos_vert_ratio = p230_pos_vert_ratio_GET(pack);
    float  mag_ratio = p230_mag_ratio_GET(pack);
    float  hagl_ratio = p230_hagl_ratio_GET(pack);
    float  tas_ratio = p230_tas_ratio_GET(pack);
    float  pos_horiz_accuracy = p230_pos_horiz_accuracy_GET(pack);
    float  pos_vert_accuracy = p230_pos_vert_accuracy_GET(pack);
}
void c_LoopBackDemoChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p231_time_usec_GET(pack);
    float  wind_x = p231_wind_x_GET(pack);
    float  wind_y = p231_wind_y_GET(pack);
    float  wind_z = p231_wind_z_GET(pack);
    float  var_horiz = p231_var_horiz_GET(pack);
    float  var_vert = p231_var_vert_GET(pack);
    float  wind_alt = p231_wind_alt_GET(pack);
    float  horiz_accuracy = p231_horiz_accuracy_GET(pack);
    float  vert_accuracy = p231_vert_accuracy_GET(pack);
}
void c_LoopBackDemoChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p232_time_usec_GET(pack);
    uint8_t  gps_id = p232_gps_id_GET(pack);
    e_GPS_INPUT_IGNORE_FLAGS  ignore_flags = p232_ignore_flags_GET(pack);
    uint32_t  time_week_ms = p232_time_week_ms_GET(pack);
    uint16_t  time_week = p232_time_week_GET(pack);
    uint8_t  fix_type = p232_fix_type_GET(pack);
    int32_t  lat = p232_lat_GET(pack);
    int32_t  lon = p232_lon_GET(pack);
    float  alt = p232_alt_GET(pack);
    float  hdop = p232_hdop_GET(pack);
    float  vdop = p232_vdop_GET(pack);
    float  vn = p232_vn_GET(pack);
    float  ve = p232_ve_GET(pack);
    float  vd = p232_vd_GET(pack);
    float  speed_accuracy = p232_speed_accuracy_GET(pack);
    float  horiz_accuracy = p232_horiz_accuracy_GET(pack);
    float  vert_accuracy = p232_vert_accuracy_GET(pack);
    uint8_t  satellites_visible = p232_satellites_visible_GET(pack);
}
void c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  flags = p233_flags_GET(pack);
    uint8_t  len = p233_len_GET(pack);
    uint8_t*  data_ = p233_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_MODE_FLAG  base_mode = p234_base_mode_GET(pack);
    uint32_t  custom_mode = p234_custom_mode_GET(pack);
    e_MAV_LANDED_STATE  landed_state = p234_landed_state_GET(pack);
    int16_t  roll = p234_roll_GET(pack);
    int16_t  pitch = p234_pitch_GET(pack);
    uint16_t  heading = p234_heading_GET(pack);
    int8_t  throttle = p234_throttle_GET(pack);
    int16_t  heading_sp = p234_heading_sp_GET(pack);
    int32_t  latitude = p234_latitude_GET(pack);
    int32_t  longitude = p234_longitude_GET(pack);
    int16_t  altitude_amsl = p234_altitude_amsl_GET(pack);
    int16_t  altitude_sp = p234_altitude_sp_GET(pack);
    uint8_t  airspeed = p234_airspeed_GET(pack);
    uint8_t  airspeed_sp = p234_airspeed_sp_GET(pack);
    uint8_t  groundspeed = p234_groundspeed_GET(pack);
    int8_t  climb_rate = p234_climb_rate_GET(pack);
    uint8_t  gps_nsat = p234_gps_nsat_GET(pack);
    e_GPS_FIX_TYPE  gps_fix_type = p234_gps_fix_type_GET(pack);
    uint8_t  battery_remaining = p234_battery_remaining_GET(pack);
    int8_t  temperature = p234_temperature_GET(pack);
    int8_t  temperature_air = p234_temperature_air_GET(pack);
    uint8_t  failsafe = p234_failsafe_GET(pack);
    uint8_t  wp_num = p234_wp_num_GET(pack);
    uint16_t  wp_distance = p234_wp_distance_GET(pack);
}
void c_LoopBackDemoChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p241_time_usec_GET(pack);
    float  vibration_x = p241_vibration_x_GET(pack);
    float  vibration_y = p241_vibration_y_GET(pack);
    float  vibration_z = p241_vibration_z_GET(pack);
    uint32_t  clipping_0 = p241_clipping_0_GET(pack);
    uint32_t  clipping_1 = p241_clipping_1_GET(pack);
    uint32_t  clipping_2 = p241_clipping_2_GET(pack);
}
void c_LoopBackDemoChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    int32_t  latitude = p242_latitude_GET(pack);
    int32_t  longitude = p242_longitude_GET(pack);
    int32_t  altitude = p242_altitude_GET(pack);
    float  x = p242_x_GET(pack);
    float  y = p242_y_GET(pack);
    float  z = p242_z_GET(pack);
    float*  q = p242_q_GET_(pack);
//process data in q
    free(q);//never forget to dispose
    float  approach_x = p242_approach_x_GET(pack);
    float  approach_y = p242_approach_y_GET(pack);
    float  approach_z = p242_approach_z_GET(pack);
    uint64_t  time_usec = p242_time_usec_TRY(ph);
}
void c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p243_target_system_GET(pack);
    int32_t  latitude = p243_latitude_GET(pack);
    int32_t  longitude = p243_longitude_GET(pack);
    int32_t  altitude = p243_altitude_GET(pack);
    float  x = p243_x_GET(pack);
    float  y = p243_y_GET(pack);
    float  z = p243_z_GET(pack);
    float*  q = p243_q_GET_(pack);
//process data in q
    free(q);//never forget to dispose
    float  approach_x = p243_approach_x_GET(pack);
    float  approach_y = p243_approach_y_GET(pack);
    float  approach_z = p243_approach_z_GET(pack);
    uint64_t  time_usec = p243_time_usec_TRY(ph);
}
void c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  message_id = p244_message_id_GET(pack);
    int32_t  interval_us = p244_interval_us_GET(pack);
}
void c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_VTOL_STATE  vtol_state = p245_vtol_state_GET(pack);
    e_MAV_LANDED_STATE  landed_state = p245_landed_state_GET(pack);
}
void c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  ICAO_address = p246_ICAO_address_GET(pack);
    int32_t  lat = p246_lat_GET(pack);
    int32_t  lon = p246_lon_GET(pack);
    e_ADSB_ALTITUDE_TYPE  altitude_type = p246_altitude_type_GET(pack);
    int32_t  altitude = p246_altitude_GET(pack);
    uint16_t  heading = p246_heading_GET(pack);
    uint16_t  hor_velocity = p246_hor_velocity_GET(pack);
    int16_t  ver_velocity = p246_ver_velocity_GET(pack);
    char16_t *  callsign = p246_callsign_TRY_(ph);
    e_ADSB_EMITTER_TYPE  emitter_type = p246_emitter_type_GET(pack);
    uint8_t  tslc = p246_tslc_GET(pack);
    e_ADSB_FLAGS  flags = p246_flags_GET(pack);
    uint16_t  squawk = p246_squawk_GET(pack);
}
void c_LoopBackDemoChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_COLLISION_SRC  src_ = p247_src__GET(pack);
    uint32_t  id = p247_id_GET(pack);
    e_MAV_COLLISION_ACTION  action = p247_action_GET(pack);
    e_MAV_COLLISION_THREAT_LEVEL  threat_level = p247_threat_level_GET(pack);
    float  time_to_minimum_delta = p247_time_to_minimum_delta_GET(pack);
    float  altitude_minimum_delta = p247_altitude_minimum_delta_GET(pack);
    float  horizontal_minimum_delta = p247_horizontal_minimum_delta_GET(pack);
}
void c_LoopBackDemoChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_network = p248_target_network_GET(pack);
    uint8_t  target_system = p248_target_system_GET(pack);
    uint8_t  target_component = p248_target_component_GET(pack);
    uint16_t  message_type = p248_message_type_GET(pack);
    uint8_t*  payload = p248_payload_GET_(pack);
//process data in payload
    free(payload);//never forget to dispose
}
void c_LoopBackDemoChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  address = p249_address_GET(pack);
    uint8_t  ver = p249_ver_GET(pack);
    uint8_t  type = p249_type_GET(pack);
    int8_t*  value = p249_value_GET_(pack);
//process data in value
    free(value);//never forget to dispose
}
void c_LoopBackDemoChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  name = p250_name_TRY_(ph);
    uint64_t  time_usec = p250_time_usec_GET(pack);
    float  x = p250_x_GET(pack);
    float  y = p250_y_GET(pack);
    float  z = p250_z_GET(pack);
}
void c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p251_time_boot_ms_GET(pack);
    char16_t *  name = p251_name_TRY_(ph);
    float  value = p251_value_GET(pack);
}
void c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p252_time_boot_ms_GET(pack);
    char16_t *  name = p252_name_TRY_(ph);
    int32_t  value = p252_value_GET(pack);
}
void c_LoopBackDemoChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_SEVERITY  severity = p253_severity_GET(pack);
    char16_t *  text = p253_text_TRY_(ph);
}
void c_LoopBackDemoChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p254_time_boot_ms_GET(pack);
    uint8_t  ind = p254_ind_GET(pack);
    float  value = p254_value_GET(pack);
}
void c_LoopBackDemoChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p256_target_system_GET(pack);
    uint8_t  target_component = p256_target_component_GET(pack);
    uint8_t*  secret_key = p256_secret_key_GET_(pack);
//process data in secret_key
    free(secret_key);//never forget to dispose
    uint64_t  initial_timestamp = p256_initial_timestamp_GET(pack);
}
void c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p257_time_boot_ms_GET(pack);
    uint32_t  last_change_ms = p257_last_change_ms_GET(pack);
    uint8_t  state = p257_state_GET(pack);
}
void c_LoopBackDemoChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p258_target_system_GET(pack);
    uint8_t  target_component = p258_target_component_GET(pack);
    char16_t *  tune = p258_tune_TRY_(ph);
}
void c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p259_time_boot_ms_GET(pack);
    uint8_t*  vendor_name = p259_vendor_name_GET_(pack);
//process data in vendor_name
    free(vendor_name);//never forget to dispose
    uint8_t*  model_name = p259_model_name_GET_(pack);
//process data in model_name
    free(model_name);//never forget to dispose
    uint32_t  firmware_version = p259_firmware_version_GET(pack);
    float  focal_length = p259_focal_length_GET(pack);
    float  sensor_size_h = p259_sensor_size_h_GET(pack);
    float  sensor_size_v = p259_sensor_size_v_GET(pack);
    uint16_t  resolution_h = p259_resolution_h_GET(pack);
    uint16_t  resolution_v = p259_resolution_v_GET(pack);
    uint8_t  lens_id = p259_lens_id_GET(pack);
    e_CAMERA_CAP_FLAGS  flags = p259_flags_GET(pack);
    uint16_t  cam_definition_version = p259_cam_definition_version_GET(pack);
    char16_t *  cam_definition_uri = p259_cam_definition_uri_TRY_(ph);
}
void c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p260_time_boot_ms_GET(pack);
    e_CAMERA_MODE  mode_id = p260_mode_id_GET(pack);
}
void c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p261_time_boot_ms_GET(pack);
    uint8_t  storage_id = p261_storage_id_GET(pack);
    uint8_t  storage_count = p261_storage_count_GET(pack);
    uint8_t  status = p261_status_GET(pack);
    float  total_capacity = p261_total_capacity_GET(pack);
    float  used_capacity = p261_used_capacity_GET(pack);
    float  available_capacity = p261_available_capacity_GET(pack);
    float  read_speed = p261_read_speed_GET(pack);
    float  write_speed = p261_write_speed_GET(pack);
}
void c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p262_time_boot_ms_GET(pack);
    uint8_t  image_status = p262_image_status_GET(pack);
    uint8_t  video_status = p262_video_status_GET(pack);
    float  image_interval = p262_image_interval_GET(pack);
    uint32_t  recording_time_ms = p262_recording_time_ms_GET(pack);
    float  available_capacity = p262_available_capacity_GET(pack);
}
void c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p263_time_boot_ms_GET(pack);
    uint64_t  time_utc = p263_time_utc_GET(pack);
    uint8_t  camera_id = p263_camera_id_GET(pack);
    int32_t  lat = p263_lat_GET(pack);
    int32_t  lon = p263_lon_GET(pack);
    int32_t  alt = p263_alt_GET(pack);
    int32_t  relative_alt = p263_relative_alt_GET(pack);
    float*  q = p263_q_GET_(pack);
//process data in q
    free(q);//never forget to dispose
    int32_t  image_index = p263_image_index_GET(pack);
    int8_t  capture_result = p263_capture_result_GET(pack);
    char16_t *  file_url = p263_file_url_TRY_(ph);
}
void c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p264_time_boot_ms_GET(pack);
    uint64_t  arming_time_utc = p264_arming_time_utc_GET(pack);
    uint64_t  takeoff_time_utc = p264_takeoff_time_utc_GET(pack);
    uint64_t  flight_uuid = p264_flight_uuid_GET(pack);
}
void c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p265_time_boot_ms_GET(pack);
    float  roll = p265_roll_GET(pack);
    float  pitch = p265_pitch_GET(pack);
    float  yaw = p265_yaw_GET(pack);
}
void c_LoopBackDemoChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p266_target_system_GET(pack);
    uint8_t  target_component = p266_target_component_GET(pack);
    uint16_t  sequence = p266_sequence_GET(pack);
    uint8_t  length = p266_length_GET(pack);
    uint8_t  first_message_offset = p266_first_message_offset_GET(pack);
    uint8_t*  data_ = p266_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p267_target_system_GET(pack);
    uint8_t  target_component = p267_target_component_GET(pack);
    uint16_t  sequence = p267_sequence_GET(pack);
    uint8_t  length = p267_length_GET(pack);
    uint8_t  first_message_offset = p267_first_message_offset_GET(pack);
    uint8_t*  data_ = p267_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_LoopBackDemoChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p268_target_system_GET(pack);
    uint8_t  target_component = p268_target_component_GET(pack);
    uint16_t  sequence = p268_sequence_GET(pack);
}
void c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  camera_id = p269_camera_id_GET(pack);
    uint8_t  status = p269_status_GET(pack);
    float  framerate = p269_framerate_GET(pack);
    uint16_t  resolution_h = p269_resolution_h_GET(pack);
    uint16_t  resolution_v = p269_resolution_v_GET(pack);
    uint32_t  bitrate = p269_bitrate_GET(pack);
    uint16_t  rotation = p269_rotation_GET(pack);
    char16_t *  uri = p269_uri_TRY_(ph);
}
void c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p270_target_system_GET(pack);
    uint8_t  target_component = p270_target_component_GET(pack);
    uint8_t  camera_id = p270_camera_id_GET(pack);
    float  framerate = p270_framerate_GET(pack);
    uint16_t  resolution_h = p270_resolution_h_GET(pack);
    uint16_t  resolution_v = p270_resolution_v_GET(pack);
    uint32_t  bitrate = p270_bitrate_GET(pack);
    uint16_t  rotation = p270_rotation_GET(pack);
    char16_t *  uri = p270_uri_TRY_(ph);
}
void c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  ssid = p299_ssid_TRY_(ph);
    char16_t *  password = p299_password_TRY_(ph);
}
void c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  version = p300_version_GET(pack);
    uint16_t  min_version = p300_min_version_GET(pack);
    uint16_t  max_version = p300_max_version_GET(pack);
    uint8_t*  spec_version_hash = p300_spec_version_hash_GET_(pack);
//process data in spec_version_hash
    free(spec_version_hash);//never forget to dispose
    uint8_t*  library_version_hash = p300_library_version_hash_GET_(pack);
//process data in library_version_hash
    free(library_version_hash);//never forget to dispose
}
void c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p310_time_usec_GET(pack);
    uint32_t  uptime_sec = p310_uptime_sec_GET(pack);
    e_UAVCAN_NODE_HEALTH  health = p310_health_GET(pack);
    e_UAVCAN_NODE_MODE  mode = p310_mode_GET(pack);
    uint8_t  sub_mode = p310_sub_mode_GET(pack);
    uint16_t  vendor_specific_status_code = p310_vendor_specific_status_code_GET(pack);
}
void c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p311_time_usec_GET(pack);
    uint32_t  uptime_sec = p311_uptime_sec_GET(pack);
    char16_t *  name = p311_name_TRY_(ph);
    uint8_t  hw_version_major = p311_hw_version_major_GET(pack);
    uint8_t  hw_version_minor = p311_hw_version_minor_GET(pack);
    uint8_t*  hw_unique_id = p311_hw_unique_id_GET_(pack);
//process data in hw_unique_id
    free(hw_unique_id);//never forget to dispose
    uint8_t  sw_version_major = p311_sw_version_major_GET(pack);
    uint8_t  sw_version_minor = p311_sw_version_minor_GET(pack);
    uint32_t  sw_vcs_commit = p311_sw_vcs_commit_GET(pack);
}
void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p320_target_system_GET(pack);
    uint8_t  target_component = p320_target_component_GET(pack);
    char16_t *  param_id = p320_param_id_TRY_(ph);
    int16_t  param_index = p320_param_index_GET(pack);
}
void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p321_target_system_GET(pack);
    uint8_t  target_component = p321_target_component_GET(pack);
}
void c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  param_id = p322_param_id_TRY_(ph);
    char16_t *  param_value = p322_param_value_TRY_(ph);
    e_MAV_PARAM_EXT_TYPE  param_type = p322_param_type_GET(pack);
    uint16_t  param_count = p322_param_count_GET(pack);
    uint16_t  param_index = p322_param_index_GET(pack);
}
void c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p323_target_system_GET(pack);
    uint8_t  target_component = p323_target_component_GET(pack);
    char16_t *  param_id = p323_param_id_TRY_(ph);
    char16_t *  param_value = p323_param_value_TRY_(ph);
    e_MAV_PARAM_EXT_TYPE  param_type = p323_param_type_GET(pack);
}
void c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  param_id = p324_param_id_TRY_(ph);
    char16_t *  param_value = p324_param_value_TRY_(ph);
    e_MAV_PARAM_EXT_TYPE  param_type = p324_param_type_GET(pack);
    e_PARAM_ACK  param_result = p324_param_result_GET(pack);
}
void c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p330_time_usec_GET(pack);
    e_MAV_DISTANCE_SENSOR  sensor_type = p330_sensor_type_GET(pack);
    uint16_t*  distances = p330_distances_GET_(pack);
//process data in distances
    free(distances);//never forget to dispose
    uint8_t  increment = p330_increment_GET(pack);
    uint16_t  min_distance = p330_min_distance_GET(pack);
    uint16_t  max_distance = p330_max_distance_GET(pack);
}

void main()
{
    static Bounds_Inside PH;
    setPack(c_LoopBackDemoChannel_new_HEARTBEAT_0(), &PH);
    p0_type_SET(e_MAV_TYPE_MAV_TYPE_GCS, PH.base.pack) ;
    p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_OPENPILOT, PH.base.pack) ;
    p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PH.base.pack) ;
    p0_custom_mode_SET((uint32_t)1737227624L, PH.base.pack) ;
    p0_system_status_SET(e_MAV_STATE_MAV_STATE_POWEROFF, PH.base.pack) ;
    p0_mavlink_version_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SYS_STATUS_1(), &PH);
    p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL, PH.base.pack) ;
    p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE, PH.base.pack) ;
    p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION, PH.base.pack) ;
    p1_load_SET((uint16_t)(uint16_t)26180, PH.base.pack) ;
    p1_voltage_battery_SET((uint16_t)(uint16_t)12456, PH.base.pack) ;
    p1_current_battery_SET((int16_t)(int16_t) -3380, PH.base.pack) ;
    p1_battery_remaining_SET((int8_t)(int8_t) -33, PH.base.pack) ;
    p1_drop_rate_comm_SET((uint16_t)(uint16_t)63072, PH.base.pack) ;
    p1_errors_comm_SET((uint16_t)(uint16_t)35443, PH.base.pack) ;
    p1_errors_count1_SET((uint16_t)(uint16_t)29862, PH.base.pack) ;
    p1_errors_count2_SET((uint16_t)(uint16_t)13183, PH.base.pack) ;
    p1_errors_count3_SET((uint16_t)(uint16_t)571, PH.base.pack) ;
    p1_errors_count4_SET((uint16_t)(uint16_t)35141, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SYSTEM_TIME_2(), &PH);
    p2_time_unix_usec_SET((uint64_t)4518015577041855232L, PH.base.pack) ;
    p2_time_boot_ms_SET((uint32_t)3417660083L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
    p3_time_boot_ms_SET((uint32_t)1762380064L, PH.base.pack) ;
    p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
    p3_type_mask_SET((uint16_t)(uint16_t)53072, PH.base.pack) ;
    p3_x_SET((float)2.7264363E37F, PH.base.pack) ;
    p3_y_SET((float) -1.4854896E38F, PH.base.pack) ;
    p3_z_SET((float) -1.6343134E38F, PH.base.pack) ;
    p3_vx_SET((float) -9.123569E37F, PH.base.pack) ;
    p3_vy_SET((float)8.59165E37F, PH.base.pack) ;
    p3_vz_SET((float) -1.2018864E38F, PH.base.pack) ;
    p3_afx_SET((float)3.074158E37F, PH.base.pack) ;
    p3_afy_SET((float)2.3013627E38F, PH.base.pack) ;
    p3_afz_SET((float) -1.8050807E38F, PH.base.pack) ;
    p3_yaw_SET((float)8.2258877E37F, PH.base.pack) ;
    p3_yaw_rate_SET((float) -3.042688E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PING_4(), &PH);
    p4_time_usec_SET((uint64_t)1766308526042857290L, PH.base.pack) ;
    p4_seq_SET((uint32_t)84689248L, PH.base.pack) ;
    p4_target_system_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
    p4_target_component_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
    p5_target_system_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
    p5_control_request_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    p5_version_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
    {
        char16_t   passkey = "dlmpp";
        p5_passkey_SET(&passkey, 0,  sizeof(passkey), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
    p6_gcs_system_id_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
    p6_control_request_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
    p6_ack_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AUTH_KEY_7(), &PH);
    {
        char16_t   key = "jrccdKgm";
        p7_key_SET(&key, 0,  sizeof(key), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_MODE_11(), &PH);
    p11_target_system_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
    p11_base_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_DISARMED, PH.base.pack) ;
    p11_custom_mode_SET((uint32_t)489825474L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_READ_20(), &PH);
    p20_target_system_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
    p20_target_component_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
    {
        char16_t   param_id = "IexmarcbSn";
        p20_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p20_param_index_SET((int16_t)(int16_t)26282, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_LIST_21(), &PH);
    p21_target_system_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
    p21_target_component_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_VALUE_22(), &PH);
    {
        char16_t   param_id = "fktdYgea";
        p22_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p22_param_value_SET((float) -1.4461194E38F, PH.base.pack) ;
    p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16, PH.base.pack) ;
    p22_param_count_SET((uint16_t)(uint16_t)52137, PH.base.pack) ;
    p22_param_index_SET((uint16_t)(uint16_t)39115, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_SET_23(), &PH);
    p23_target_system_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
    p23_target_component_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
    {
        char16_t   param_id = "uvstdzlkriqs";
        p23_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p23_param_value_SET((float) -1.2896232E38F, PH.base.pack) ;
    p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT8, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RAW_INT_24(), &PH);
    p24_time_usec_SET((uint64_t)5763796025690970668L, PH.base.pack) ;
    p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, PH.base.pack) ;
    p24_lat_SET((int32_t)1206985681, PH.base.pack) ;
    p24_lon_SET((int32_t) -1900024392, PH.base.pack) ;
    p24_alt_SET((int32_t)203543098, PH.base.pack) ;
    p24_eph_SET((uint16_t)(uint16_t)1868, PH.base.pack) ;
    p24_epv_SET((uint16_t)(uint16_t)50974, PH.base.pack) ;
    p24_vel_SET((uint16_t)(uint16_t)5947, PH.base.pack) ;
    p24_cog_SET((uint16_t)(uint16_t)5959, PH.base.pack) ;
    p24_satellites_visible_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
    p24_alt_ellipsoid_SET((int32_t) -295756047, &PH) ;
    p24_h_acc_SET((uint32_t)491721479L, &PH) ;
    p24_v_acc_SET((uint32_t)2315952582L, &PH) ;
    p24_vel_acc_SET((uint32_t)2595228346L, &PH) ;
    p24_hdg_acc_SET((uint32_t)2664148580L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_STATUS_25(), &PH);
    p25_satellites_visible_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
    {
        uint8_t  satellite_prn [] =  {(uint8_t)57, (uint8_t)32, (uint8_t)86, (uint8_t)113, (uint8_t)199, (uint8_t)226, (uint8_t)168, (uint8_t)220, (uint8_t)152, (uint8_t)25, (uint8_t)159, (uint8_t)58, (uint8_t)184, (uint8_t)79, (uint8_t)145, (uint8_t)234, (uint8_t)147, (uint8_t)11, (uint8_t)63, (uint8_t)2};
        p25_satellite_prn_SET(&satellite_prn, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_used [] =  {(uint8_t)237, (uint8_t)127, (uint8_t)94, (uint8_t)143, (uint8_t)97, (uint8_t)133, (uint8_t)77, (uint8_t)243, (uint8_t)185, (uint8_t)210, (uint8_t)53, (uint8_t)87, (uint8_t)132, (uint8_t)190, (uint8_t)217, (uint8_t)132, (uint8_t)123, (uint8_t)91, (uint8_t)233, (uint8_t)215};
        p25_satellite_used_SET(&satellite_used, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_elevation [] =  {(uint8_t)75, (uint8_t)106, (uint8_t)149, (uint8_t)78, (uint8_t)172, (uint8_t)122, (uint8_t)226, (uint8_t)96, (uint8_t)68, (uint8_t)47, (uint8_t)194, (uint8_t)183, (uint8_t)75, (uint8_t)221, (uint8_t)62, (uint8_t)75, (uint8_t)170, (uint8_t)106, (uint8_t)230, (uint8_t)7};
        p25_satellite_elevation_SET(&satellite_elevation, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_azimuth [] =  {(uint8_t)187, (uint8_t)119, (uint8_t)99, (uint8_t)14, (uint8_t)132, (uint8_t)177, (uint8_t)250, (uint8_t)46, (uint8_t)44, (uint8_t)37, (uint8_t)123, (uint8_t)194, (uint8_t)154, (uint8_t)139, (uint8_t)114, (uint8_t)194, (uint8_t)134, (uint8_t)158, (uint8_t)43, (uint8_t)68};
        p25_satellite_azimuth_SET(&satellite_azimuth, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_snr [] =  {(uint8_t)237, (uint8_t)158, (uint8_t)86, (uint8_t)99, (uint8_t)154, (uint8_t)206, (uint8_t)233, (uint8_t)241, (uint8_t)81, (uint8_t)55, (uint8_t)244, (uint8_t)170, (uint8_t)153, (uint8_t)1, (uint8_t)178, (uint8_t)151, (uint8_t)99, (uint8_t)114, (uint8_t)87, (uint8_t)91};
        p25_satellite_snr_SET(&satellite_snr, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU_26(), &PH);
    p26_time_boot_ms_SET((uint32_t)1130551649L, PH.base.pack) ;
    p26_xacc_SET((int16_t)(int16_t) -15791, PH.base.pack) ;
    p26_yacc_SET((int16_t)(int16_t)15380, PH.base.pack) ;
    p26_zacc_SET((int16_t)(int16_t) -22033, PH.base.pack) ;
    p26_xgyro_SET((int16_t)(int16_t) -210, PH.base.pack) ;
    p26_ygyro_SET((int16_t)(int16_t) -3716, PH.base.pack) ;
    p26_zgyro_SET((int16_t)(int16_t) -29186, PH.base.pack) ;
    p26_xmag_SET((int16_t)(int16_t) -2196, PH.base.pack) ;
    p26_ymag_SET((int16_t)(int16_t) -31757, PH.base.pack) ;
    p26_zmag_SET((int16_t)(int16_t) -29255, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RAW_IMU_27(), &PH);
    p27_time_usec_SET((uint64_t)3057431895679495115L, PH.base.pack) ;
    p27_xacc_SET((int16_t)(int16_t)6406, PH.base.pack) ;
    p27_yacc_SET((int16_t)(int16_t) -5460, PH.base.pack) ;
    p27_zacc_SET((int16_t)(int16_t) -4881, PH.base.pack) ;
    p27_xgyro_SET((int16_t)(int16_t)16829, PH.base.pack) ;
    p27_ygyro_SET((int16_t)(int16_t) -7316, PH.base.pack) ;
    p27_zgyro_SET((int16_t)(int16_t) -2696, PH.base.pack) ;
    p27_xmag_SET((int16_t)(int16_t) -10397, PH.base.pack) ;
    p27_ymag_SET((int16_t)(int16_t)26599, PH.base.pack) ;
    p27_zmag_SET((int16_t)(int16_t)12853, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RAW_PRESSURE_28(), &PH);
    p28_time_usec_SET((uint64_t)2073026626476100362L, PH.base.pack) ;
    p28_press_abs_SET((int16_t)(int16_t) -28029, PH.base.pack) ;
    p28_press_diff1_SET((int16_t)(int16_t)23839, PH.base.pack) ;
    p28_press_diff2_SET((int16_t)(int16_t) -19107, PH.base.pack) ;
    p28_temperature_SET((int16_t)(int16_t)1686, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE_29(), &PH);
    p29_time_boot_ms_SET((uint32_t)1451254287L, PH.base.pack) ;
    p29_press_abs_SET((float)3.9573309E37F, PH.base.pack) ;
    p29_press_diff_SET((float)7.9822665E37F, PH.base.pack) ;
    p29_temperature_SET((int16_t)(int16_t)31989, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_30(), &PH);
    p30_time_boot_ms_SET((uint32_t)3455984217L, PH.base.pack) ;
    p30_roll_SET((float) -1.689224E38F, PH.base.pack) ;
    p30_pitch_SET((float) -1.0396993E38F, PH.base.pack) ;
    p30_yaw_SET((float)2.8690456E38F, PH.base.pack) ;
    p30_rollspeed_SET((float)4.6431785E37F, PH.base.pack) ;
    p30_pitchspeed_SET((float)2.876438E38F, PH.base.pack) ;
    p30_yawspeed_SET((float)1.4041132E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_31(), &PH);
    p31_time_boot_ms_SET((uint32_t)4224333196L, PH.base.pack) ;
    p31_q1_SET((float)1.0161362E38F, PH.base.pack) ;
    p31_q2_SET((float)3.6038275E37F, PH.base.pack) ;
    p31_q3_SET((float) -1.2948791E38F, PH.base.pack) ;
    p31_q4_SET((float) -2.2180922E38F, PH.base.pack) ;
    p31_rollspeed_SET((float)2.4933063E38F, PH.base.pack) ;
    p31_pitchspeed_SET((float) -1.007861E38F, PH.base.pack) ;
    p31_yawspeed_SET((float)1.6509663E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_32(), &PH);
    p32_time_boot_ms_SET((uint32_t)948377576L, PH.base.pack) ;
    p32_x_SET((float) -9.234789E37F, PH.base.pack) ;
    p32_y_SET((float) -1.3707017E38F, PH.base.pack) ;
    p32_z_SET((float) -5.938711E37F, PH.base.pack) ;
    p32_vx_SET((float)3.8334723E37F, PH.base.pack) ;
    p32_vy_SET((float) -1.708851E38F, PH.base.pack) ;
    p32_vz_SET((float) -2.8423943E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_33(), &PH);
    p33_time_boot_ms_SET((uint32_t)4209668129L, PH.base.pack) ;
    p33_lat_SET((int32_t) -831263820, PH.base.pack) ;
    p33_lon_SET((int32_t) -505485584, PH.base.pack) ;
    p33_alt_SET((int32_t)781779872, PH.base.pack) ;
    p33_relative_alt_SET((int32_t)1564776414, PH.base.pack) ;
    p33_vx_SET((int16_t)(int16_t) -7489, PH.base.pack) ;
    p33_vy_SET((int16_t)(int16_t)560, PH.base.pack) ;
    p33_vz_SET((int16_t)(int16_t)1221, PH.base.pack) ;
    p33_hdg_SET((uint16_t)(uint16_t)8458, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_SCALED_34(), &PH);
    p34_time_boot_ms_SET((uint32_t)468599416L, PH.base.pack) ;
    p34_port_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
    p34_chan1_scaled_SET((int16_t)(int16_t) -13604, PH.base.pack) ;
    p34_chan2_scaled_SET((int16_t)(int16_t) -16145, PH.base.pack) ;
    p34_chan3_scaled_SET((int16_t)(int16_t)20024, PH.base.pack) ;
    p34_chan4_scaled_SET((int16_t)(int16_t)4199, PH.base.pack) ;
    p34_chan5_scaled_SET((int16_t)(int16_t)19036, PH.base.pack) ;
    p34_chan6_scaled_SET((int16_t)(int16_t) -6515, PH.base.pack) ;
    p34_chan7_scaled_SET((int16_t)(int16_t) -9270, PH.base.pack) ;
    p34_chan8_scaled_SET((int16_t)(int16_t)23916, PH.base.pack) ;
    p34_rssi_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_RAW_35(), &PH);
    p35_time_boot_ms_SET((uint32_t)3151488184L, PH.base.pack) ;
    p35_port_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
    p35_chan1_raw_SET((uint16_t)(uint16_t)52708, PH.base.pack) ;
    p35_chan2_raw_SET((uint16_t)(uint16_t)55298, PH.base.pack) ;
    p35_chan3_raw_SET((uint16_t)(uint16_t)22173, PH.base.pack) ;
    p35_chan4_raw_SET((uint16_t)(uint16_t)3460, PH.base.pack) ;
    p35_chan5_raw_SET((uint16_t)(uint16_t)22597, PH.base.pack) ;
    p35_chan6_raw_SET((uint16_t)(uint16_t)9072, PH.base.pack) ;
    p35_chan7_raw_SET((uint16_t)(uint16_t)29172, PH.base.pack) ;
    p35_chan8_raw_SET((uint16_t)(uint16_t)15003, PH.base.pack) ;
    p35_rssi_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
    p36_time_usec_SET((uint32_t)1394862748L, PH.base.pack) ;
    p36_port_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
    p36_servo1_raw_SET((uint16_t)(uint16_t)21731, PH.base.pack) ;
    p36_servo2_raw_SET((uint16_t)(uint16_t)50862, PH.base.pack) ;
    p36_servo3_raw_SET((uint16_t)(uint16_t)16882, PH.base.pack) ;
    p36_servo4_raw_SET((uint16_t)(uint16_t)48383, PH.base.pack) ;
    p36_servo5_raw_SET((uint16_t)(uint16_t)11610, PH.base.pack) ;
    p36_servo6_raw_SET((uint16_t)(uint16_t)3256, PH.base.pack) ;
    p36_servo7_raw_SET((uint16_t)(uint16_t)21628, PH.base.pack) ;
    p36_servo8_raw_SET((uint16_t)(uint16_t)35466, PH.base.pack) ;
    p36_servo9_raw_SET((uint16_t)(uint16_t)18549, &PH) ;
    p36_servo10_raw_SET((uint16_t)(uint16_t)56516, &PH) ;
    p36_servo11_raw_SET((uint16_t)(uint16_t)48487, &PH) ;
    p36_servo12_raw_SET((uint16_t)(uint16_t)16925, &PH) ;
    p36_servo13_raw_SET((uint16_t)(uint16_t)44687, &PH) ;
    p36_servo14_raw_SET((uint16_t)(uint16_t)51518, &PH) ;
    p36_servo15_raw_SET((uint16_t)(uint16_t)63600, &PH) ;
    p36_servo16_raw_SET((uint16_t)(uint16_t)51794, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
    p37_target_system_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
    p37_target_component_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
    p37_start_index_SET((int16_t)(int16_t)24558, PH.base.pack) ;
    p37_end_index_SET((int16_t)(int16_t)11227, PH.base.pack) ;
    p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
    p38_target_system_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
    p38_target_component_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
    p38_start_index_SET((int16_t)(int16_t)30416, PH.base.pack) ;
    p38_end_index_SET((int16_t)(int16_t)10099, PH.base.pack) ;
    p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_39(), &PH);
    p39_target_system_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
    p39_target_component_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
    p39_seq_SET((uint16_t)(uint16_t)59747, PH.base.pack) ;
    p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
    p39_command_SET(e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE, PH.base.pack) ;
    p39_current_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
    p39_autocontinue_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    p39_param1_SET((float) -1.0213316E38F, PH.base.pack) ;
    p39_param2_SET((float) -2.2004845E38F, PH.base.pack) ;
    p39_param3_SET((float) -8.714419E37F, PH.base.pack) ;
    p39_param4_SET((float)1.4273069E38F, PH.base.pack) ;
    p39_x_SET((float)2.6960784E38F, PH.base.pack) ;
    p39_y_SET((float) -2.7431014E38F, PH.base.pack) ;
    p39_z_SET((float)2.7780173E38F, PH.base.pack) ;
    p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_40(), &PH);
    p40_target_system_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
    p40_target_component_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
    p40_seq_SET((uint16_t)(uint16_t)40614, PH.base.pack) ;
    p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_SET_CURRENT_41(), &PH);
    p41_target_system_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
    p41_target_component_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
    p41_seq_SET((uint16_t)(uint16_t)52355, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_CURRENT_42(), &PH);
    p42_seq_SET((uint16_t)(uint16_t)60551, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_LIST_43(), &PH);
    p43_target_system_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
    p43_target_component_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
    p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_COUNT_44(), &PH);
    p44_target_system_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    p44_target_component_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
    p44_count_SET((uint16_t)(uint16_t)4959, PH.base.pack) ;
    p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_CLEAR_ALL_45(), &PH);
    p45_target_system_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
    p45_target_component_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
    p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_REACHED_46(), &PH);
    p46_seq_SET((uint16_t)(uint16_t)33999, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ACK_47(), &PH);
    p47_target_system_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
    p47_target_component_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
    p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_NO_SPACE, PH.base.pack) ;
    p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
    p48_target_system_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
    p48_latitude_SET((int32_t)1785434130, PH.base.pack) ;
    p48_longitude_SET((int32_t) -723872251, PH.base.pack) ;
    p48_altitude_SET((int32_t) -185434036, PH.base.pack) ;
    p48_time_usec_SET((uint64_t)5760740357723282368L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
    p49_latitude_SET((int32_t)728476399, PH.base.pack) ;
    p49_longitude_SET((int32_t)2045879348, PH.base.pack) ;
    p49_altitude_SET((int32_t)1262438727, PH.base.pack) ;
    p49_time_usec_SET((uint64_t)1569237769319003778L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_MAP_RC_50(), &PH);
    p50_target_system_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
    p50_target_component_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
    {
        char16_t   param_id = "uhfqqhttfuoax";
        p50_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p50_param_index_SET((int16_t)(int16_t)19930, PH.base.pack) ;
    p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
    p50_param_value0_SET((float) -1.0747382E38F, PH.base.pack) ;
    p50_scale_SET((float) -3.8873748E37F, PH.base.pack) ;
    p50_param_value_min_SET((float)2.8730408E38F, PH.base.pack) ;
    p50_param_value_max_SET((float)3.0786688E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_INT_51(), &PH);
    p51_target_system_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
    p51_target_component_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    p51_seq_SET((uint16_t)(uint16_t)62101, PH.base.pack) ;
    p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
    p54_target_system_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
    p54_target_component_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    p54_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
    p54_p1x_SET((float)1.4708324E38F, PH.base.pack) ;
    p54_p1y_SET((float) -1.26059E38F, PH.base.pack) ;
    p54_p1z_SET((float)3.6650915E37F, PH.base.pack) ;
    p54_p2x_SET((float) -2.6499485E38F, PH.base.pack) ;
    p54_p2y_SET((float)9.8466656E36F, PH.base.pack) ;
    p54_p2z_SET((float) -4.161477E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
    p55_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
    p55_p1x_SET((float)1.1361696E38F, PH.base.pack) ;
    p55_p1y_SET((float) -4.9624206E37F, PH.base.pack) ;
    p55_p1z_SET((float)1.7112389E38F, PH.base.pack) ;
    p55_p2x_SET((float) -6.7803086E37F, PH.base.pack) ;
    p55_p2y_SET((float) -8.816947E37F, PH.base.pack) ;
    p55_p2z_SET((float) -2.4501157E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
    p61_time_usec_SET((uint64_t)7430122223724103140L, PH.base.pack) ;
    {
        float  q [] =  {-1.91642E38F, -8.376368E37F, 2.4569062E38F, -8.049464E37F};
        p61_q_SET(&q, 0, &PH.base.pack) ;
    }
    p61_rollspeed_SET((float)2.1240074E38F, PH.base.pack) ;
    p61_pitchspeed_SET((float)3.3049594E38F, PH.base.pack) ;
    p61_yawspeed_SET((float) -3.130889E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-2.2810474E38F, 1.3358773E38F, -1.4446664E38F, 1.4297602E38F, -1.6268406E38F, -1.6728455E38F, -3.155553E38F, -1.3236538E38F, -1.8618124E38F};
        p61_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
    p62_nav_roll_SET((float)2.5514929E38F, PH.base.pack) ;
    p62_nav_pitch_SET((float)1.8792248E37F, PH.base.pack) ;
    p62_nav_bearing_SET((int16_t)(int16_t)22351, PH.base.pack) ;
    p62_target_bearing_SET((int16_t)(int16_t)30849, PH.base.pack) ;
    p62_wp_dist_SET((uint16_t)(uint16_t)60118, PH.base.pack) ;
    p62_alt_error_SET((float)1.2391192E38F, PH.base.pack) ;
    p62_aspd_error_SET((float) -1.6625793E38F, PH.base.pack) ;
    p62_xtrack_error_SET((float)1.7863436E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
    p63_time_usec_SET((uint64_t)286664375120158336L, PH.base.pack) ;
    p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, PH.base.pack) ;
    p63_lat_SET((int32_t) -593107861, PH.base.pack) ;
    p63_lon_SET((int32_t)1171036818, PH.base.pack) ;
    p63_alt_SET((int32_t) -2010410586, PH.base.pack) ;
    p63_relative_alt_SET((int32_t)577680746, PH.base.pack) ;
    p63_vx_SET((float)1.4313333E36F, PH.base.pack) ;
    p63_vy_SET((float)7.949816E37F, PH.base.pack) ;
    p63_vz_SET((float)3.0609797E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-1.3614847E38F, -2.0788659E37F, -7.145911E37F, 1.6442955E37F, 4.0416758E36F, 1.7272699E37F, -3.1065048E38F, 2.275822E38F, -1.4094322E38F, -2.47717E38F, -3.3553847E38F, -1.2847003E38F, 1.4036349E38F, -6.9893067E37F, 6.7387647E37F, -3.3115733E38F, 6.831641E37F, -7.764375E37F, 1.4040648E38F, 2.8349925E38F, 5.5536686E37F, 2.1002032E38F, 7.167918E37F, 3.1036076E38F, -2.308695E38F, -8.934805E37F, -3.0965252E38F, 5.526329E37F, 1.2159905E38F, -2.2926897E38F, -1.7566449E38F, 8.591834E37F, 2.5229249E38F, -2.3185056E38F, -1.2243008E38F, -9.308392E37F};
        p63_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
    p64_time_usec_SET((uint64_t)7534573964856576651L, PH.base.pack) ;
    p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
    p64_x_SET((float) -2.5990306E38F, PH.base.pack) ;
    p64_y_SET((float) -2.9939623E38F, PH.base.pack) ;
    p64_z_SET((float) -1.828329E38F, PH.base.pack) ;
    p64_vx_SET((float) -1.423245E38F, PH.base.pack) ;
    p64_vy_SET((float)6.8516895E37F, PH.base.pack) ;
    p64_vz_SET((float)2.6105337E38F, PH.base.pack) ;
    p64_ax_SET((float)6.102676E36F, PH.base.pack) ;
    p64_ay_SET((float)1.812416E38F, PH.base.pack) ;
    p64_az_SET((float)2.4444167E38F, PH.base.pack) ;
    {
        float  covariance [] =  {1.9752663E38F, 2.6628388E38F, -2.3923025E38F, 7.47036E37F, -1.4083202E38F, -1.2962721E38F, 2.383034E38F, -3.0277291E38F, 3.0542794E38F, -1.3220032E38F, 1.814889E38F, 6.4271523E37F, 2.8117433E38F, -1.5516023E38F, -3.379386E38F, 5.4057484E37F, -2.3606185E38F, 1.8597657E38F, -2.7457683E38F, -3.2857605E38F, -3.3389298E38F, 2.5328813E38F, 2.5747639E38F, 7.6008894E36F, -6.0321397E37F, -1.4937731E38F, -1.0538886E38F, -3.1826818E38F, -1.9892898E38F, -1.6491188E38F, -6.208276E37F, -2.0046178E38F, 2.7961596E38F, 3.0181982E38F, 3.0190164E37F, 3.632914E37F, -2.3104575E38F, -2.8045826E38F, -2.227145E38F, -1.8186896E38F, 5.6257167E37F, -2.0258267E38F, -3.6440515E37F, -3.3848356E38F, 3.2963783E38F};
        p64_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_65(), &PH);
    p65_time_boot_ms_SET((uint32_t)3097922852L, PH.base.pack) ;
    p65_chancount_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
    p65_chan1_raw_SET((uint16_t)(uint16_t)54473, PH.base.pack) ;
    p65_chan2_raw_SET((uint16_t)(uint16_t)15813, PH.base.pack) ;
    p65_chan3_raw_SET((uint16_t)(uint16_t)16290, PH.base.pack) ;
    p65_chan4_raw_SET((uint16_t)(uint16_t)24459, PH.base.pack) ;
    p65_chan5_raw_SET((uint16_t)(uint16_t)20695, PH.base.pack) ;
    p65_chan6_raw_SET((uint16_t)(uint16_t)23250, PH.base.pack) ;
    p65_chan7_raw_SET((uint16_t)(uint16_t)27868, PH.base.pack) ;
    p65_chan8_raw_SET((uint16_t)(uint16_t)12356, PH.base.pack) ;
    p65_chan9_raw_SET((uint16_t)(uint16_t)1627, PH.base.pack) ;
    p65_chan10_raw_SET((uint16_t)(uint16_t)41631, PH.base.pack) ;
    p65_chan11_raw_SET((uint16_t)(uint16_t)31081, PH.base.pack) ;
    p65_chan12_raw_SET((uint16_t)(uint16_t)23103, PH.base.pack) ;
    p65_chan13_raw_SET((uint16_t)(uint16_t)12411, PH.base.pack) ;
    p65_chan14_raw_SET((uint16_t)(uint16_t)12460, PH.base.pack) ;
    p65_chan15_raw_SET((uint16_t)(uint16_t)26022, PH.base.pack) ;
    p65_chan16_raw_SET((uint16_t)(uint16_t)25835, PH.base.pack) ;
    p65_chan17_raw_SET((uint16_t)(uint16_t)21204, PH.base.pack) ;
    p65_chan18_raw_SET((uint16_t)(uint16_t)41271, PH.base.pack) ;
    p65_rssi_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_REQUEST_DATA_STREAM_66(), &PH);
    p66_target_system_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
    p66_target_component_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
    p66_req_stream_id_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
    p66_req_message_rate_SET((uint16_t)(uint16_t)22753, PH.base.pack) ;
    p66_start_stop_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA_STREAM_67(), &PH);
    p67_stream_id_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
    p67_message_rate_SET((uint16_t)(uint16_t)26728, PH.base.pack) ;
    p67_on_off_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MANUAL_CONTROL_69(), &PH);
    p69_target_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
    p69_x_SET((int16_t)(int16_t)28324, PH.base.pack) ;
    p69_y_SET((int16_t)(int16_t)18005, PH.base.pack) ;
    p69_z_SET((int16_t)(int16_t) -11719, PH.base.pack) ;
    p69_r_SET((int16_t)(int16_t) -25337, PH.base.pack) ;
    p69_buttons_SET((uint16_t)(uint16_t)13821, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
    p70_target_system_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
    p70_target_component_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
    p70_chan1_raw_SET((uint16_t)(uint16_t)14672, PH.base.pack) ;
    p70_chan2_raw_SET((uint16_t)(uint16_t)34993, PH.base.pack) ;
    p70_chan3_raw_SET((uint16_t)(uint16_t)36509, PH.base.pack) ;
    p70_chan4_raw_SET((uint16_t)(uint16_t)23189, PH.base.pack) ;
    p70_chan5_raw_SET((uint16_t)(uint16_t)34854, PH.base.pack) ;
    p70_chan6_raw_SET((uint16_t)(uint16_t)48282, PH.base.pack) ;
    p70_chan7_raw_SET((uint16_t)(uint16_t)61321, PH.base.pack) ;
    p70_chan8_raw_SET((uint16_t)(uint16_t)57476, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_INT_73(), &PH);
    p73_target_system_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
    p73_target_component_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    p73_seq_SET((uint16_t)(uint16_t)50093, PH.base.pack) ;
    p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
    p73_command_SET(e_MAV_CMD_MAV_CMD_NAV_PAYLOAD_PLACE, PH.base.pack) ;
    p73_current_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
    p73_autocontinue_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
    p73_param1_SET((float) -2.9406492E38F, PH.base.pack) ;
    p73_param2_SET((float)2.2830298E38F, PH.base.pack) ;
    p73_param3_SET((float)3.0158661E38F, PH.base.pack) ;
    p73_param4_SET((float)1.6949403E38F, PH.base.pack) ;
    p73_x_SET((int32_t)1951974804, PH.base.pack) ;
    p73_y_SET((int32_t) -989517583, PH.base.pack) ;
    p73_z_SET((float)2.4740398E38F, PH.base.pack) ;
    p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VFR_HUD_74(), &PH);
    p74_airspeed_SET((float) -1.6344288E38F, PH.base.pack) ;
    p74_groundspeed_SET((float)3.0937542E38F, PH.base.pack) ;
    p74_heading_SET((int16_t)(int16_t)1013, PH.base.pack) ;
    p74_throttle_SET((uint16_t)(uint16_t)3451, PH.base.pack) ;
    p74_alt_SET((float)1.1434677E38F, PH.base.pack) ;
    p74_climb_SET((float)4.3134033E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_INT_75(), &PH);
    p75_target_system_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    p75_target_component_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
    p75_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
    p75_command_SET(e_MAV_CMD_MAV_CMD_CONDITION_DELAY, PH.base.pack) ;
    p75_current_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
    p75_autocontinue_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
    p75_param1_SET((float)2.0247925E38F, PH.base.pack) ;
    p75_param2_SET((float)8.013044E37F, PH.base.pack) ;
    p75_param3_SET((float)1.3915553E38F, PH.base.pack) ;
    p75_param4_SET((float)1.3849942E38F, PH.base.pack) ;
    p75_x_SET((int32_t)429001674, PH.base.pack) ;
    p75_y_SET((int32_t) -564786605, PH.base.pack) ;
    p75_z_SET((float)2.924876E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_LONG_76(), &PH);
    p76_target_system_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
    p76_target_component_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
    p76_command_SET(e_MAV_CMD_MAV_CMD_DO_PAUSE_CONTINUE, PH.base.pack) ;
    p76_confirmation_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
    p76_param1_SET((float) -8.90831E37F, PH.base.pack) ;
    p76_param2_SET((float) -1.0122978E38F, PH.base.pack) ;
    p76_param3_SET((float) -2.1567815E38F, PH.base.pack) ;
    p76_param4_SET((float)2.3251053E38F, PH.base.pack) ;
    p76_param5_SET((float)2.9556562E38F, PH.base.pack) ;
    p76_param6_SET((float)1.7575244E37F, PH.base.pack) ;
    p76_param7_SET((float) -7.294407E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_ACK_77(), &PH);
    p77_command_SET(e_MAV_CMD_MAV_CMD_START_RX_PAIR, PH.base.pack) ;
    p77_result_SET(e_MAV_RESULT_MAV_RESULT_FAILED, PH.base.pack) ;
    p77_progress_SET((uint8_t)(uint8_t)51, &PH) ;
    p77_result_param2_SET((int32_t) -182077456, &PH) ;
    p77_target_system_SET((uint8_t)(uint8_t)145, &PH) ;
    p77_target_component_SET((uint8_t)(uint8_t)84, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MANUAL_SETPOINT_81(), &PH);
    p81_time_boot_ms_SET((uint32_t)857541862L, PH.base.pack) ;
    p81_roll_SET((float) -5.776991E37F, PH.base.pack) ;
    p81_pitch_SET((float) -7.768134E37F, PH.base.pack) ;
    p81_yaw_SET((float) -3.2653012E38F, PH.base.pack) ;
    p81_thrust_SET((float) -1.9034891E38F, PH.base.pack) ;
    p81_mode_switch_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
    p81_manual_override_switch_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
    p82_time_boot_ms_SET((uint32_t)2762108080L, PH.base.pack) ;
    p82_target_system_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
    p82_target_component_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
    p82_type_mask_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
    {
        float  q [] =  {-1.3657229E38F, 6.7257743E37F, -9.411723E37F, -8.49047E37F};
        p82_q_SET(&q, 0, &PH.base.pack) ;
    }
    p82_body_roll_rate_SET((float) -2.96848E38F, PH.base.pack) ;
    p82_body_pitch_rate_SET((float) -2.0109707E38F, PH.base.pack) ;
    p82_body_yaw_rate_SET((float) -3.258326E38F, PH.base.pack) ;
    p82_thrust_SET((float) -2.42896E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_TARGET_83(), &PH);
    p83_time_boot_ms_SET((uint32_t)1403642941L, PH.base.pack) ;
    p83_type_mask_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
    {
        float  q [] =  {1.9973277E38F, 1.6887165E38F, 1.979094E38F, -3.397397E38F};
        p83_q_SET(&q, 0, &PH.base.pack) ;
    }
    p83_body_roll_rate_SET((float)2.3371687E37F, PH.base.pack) ;
    p83_body_pitch_rate_SET((float) -1.0180238E38F, PH.base.pack) ;
    p83_body_yaw_rate_SET((float)2.3640917E38F, PH.base.pack) ;
    p83_thrust_SET((float)2.2216088E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
    p84_time_boot_ms_SET((uint32_t)1237853250L, PH.base.pack) ;
    p84_target_system_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
    p84_target_component_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
    p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
    p84_type_mask_SET((uint16_t)(uint16_t)61490, PH.base.pack) ;
    p84_x_SET((float) -1.4004756E38F, PH.base.pack) ;
    p84_y_SET((float) -2.4982617E37F, PH.base.pack) ;
    p84_z_SET((float) -9.078579E37F, PH.base.pack) ;
    p84_vx_SET((float)1.0750499E38F, PH.base.pack) ;
    p84_vy_SET((float)2.2281486E38F, PH.base.pack) ;
    p84_vz_SET((float) -9.986976E37F, PH.base.pack) ;
    p84_afx_SET((float) -2.2323351E38F, PH.base.pack) ;
    p84_afy_SET((float) -1.814184E38F, PH.base.pack) ;
    p84_afz_SET((float) -2.0643134E38F, PH.base.pack) ;
    p84_yaw_SET((float) -2.7864753E38F, PH.base.pack) ;
    p84_yaw_rate_SET((float)2.4938046E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
    p86_time_boot_ms_SET((uint32_t)1962595448L, PH.base.pack) ;
    p86_target_system_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
    p86_target_component_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
    p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
    p86_type_mask_SET((uint16_t)(uint16_t)10434, PH.base.pack) ;
    p86_lat_int_SET((int32_t)1767008906, PH.base.pack) ;
    p86_lon_int_SET((int32_t)1763006747, PH.base.pack) ;
    p86_alt_SET((float)9.985667E37F, PH.base.pack) ;
    p86_vx_SET((float) -3.22704E38F, PH.base.pack) ;
    p86_vy_SET((float) -1.2028137E37F, PH.base.pack) ;
    p86_vz_SET((float) -1.1412184E38F, PH.base.pack) ;
    p86_afx_SET((float) -1.807808E38F, PH.base.pack) ;
    p86_afy_SET((float) -2.8687446E38F, PH.base.pack) ;
    p86_afz_SET((float) -1.0814511E38F, PH.base.pack) ;
    p86_yaw_SET((float) -4.8175753E37F, PH.base.pack) ;
    p86_yaw_rate_SET((float) -3.2938585E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
    p87_time_boot_ms_SET((uint32_t)3622893946L, PH.base.pack) ;
    p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
    p87_type_mask_SET((uint16_t)(uint16_t)50070, PH.base.pack) ;
    p87_lat_int_SET((int32_t)2045403397, PH.base.pack) ;
    p87_lon_int_SET((int32_t) -1016846542, PH.base.pack) ;
    p87_alt_SET((float)9.378401E37F, PH.base.pack) ;
    p87_vx_SET((float) -3.1754765E38F, PH.base.pack) ;
    p87_vy_SET((float)3.0913612E37F, PH.base.pack) ;
    p87_vz_SET((float)3.1550765E38F, PH.base.pack) ;
    p87_afx_SET((float)6.5029796E37F, PH.base.pack) ;
    p87_afy_SET((float)1.94468E38F, PH.base.pack) ;
    p87_afz_SET((float)4.2403795E37F, PH.base.pack) ;
    p87_yaw_SET((float) -9.440383E37F, PH.base.pack) ;
    p87_yaw_rate_SET((float)1.4652233E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
    p89_time_boot_ms_SET((uint32_t)46983381L, PH.base.pack) ;
    p89_x_SET((float) -6.4810777E37F, PH.base.pack) ;
    p89_y_SET((float) -5.215934E37F, PH.base.pack) ;
    p89_z_SET((float)8.318739E37F, PH.base.pack) ;
    p89_roll_SET((float) -4.011661E37F, PH.base.pack) ;
    p89_pitch_SET((float)1.0876762E38F, PH.base.pack) ;
    p89_yaw_SET((float) -2.1618302E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_STATE_90(), &PH);
    p90_time_usec_SET((uint64_t)1397316411111058757L, PH.base.pack) ;
    p90_roll_SET((float)1.1902043E38F, PH.base.pack) ;
    p90_pitch_SET((float)3.3670958E38F, PH.base.pack) ;
    p90_yaw_SET((float)1.7098286E38F, PH.base.pack) ;
    p90_rollspeed_SET((float) -3.0882108E38F, PH.base.pack) ;
    p90_pitchspeed_SET((float) -1.6849361E38F, PH.base.pack) ;
    p90_yawspeed_SET((float)1.3117669E38F, PH.base.pack) ;
    p90_lat_SET((int32_t)1741387705, PH.base.pack) ;
    p90_lon_SET((int32_t)1519659903, PH.base.pack) ;
    p90_alt_SET((int32_t) -2039762663, PH.base.pack) ;
    p90_vx_SET((int16_t)(int16_t) -18855, PH.base.pack) ;
    p90_vy_SET((int16_t)(int16_t)32066, PH.base.pack) ;
    p90_vz_SET((int16_t)(int16_t) -7054, PH.base.pack) ;
    p90_xacc_SET((int16_t)(int16_t) -23789, PH.base.pack) ;
    p90_yacc_SET((int16_t)(int16_t)4822, PH.base.pack) ;
    p90_zacc_SET((int16_t)(int16_t)10785, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_CONTROLS_91(), &PH);
    p91_time_usec_SET((uint64_t)6777255903333111928L, PH.base.pack) ;
    p91_roll_ailerons_SET((float) -2.5908284E38F, PH.base.pack) ;
    p91_pitch_elevator_SET((float)1.2434146E38F, PH.base.pack) ;
    p91_yaw_rudder_SET((float) -1.3718196E38F, PH.base.pack) ;
    p91_throttle_SET((float) -1.022629E38F, PH.base.pack) ;
    p91_aux1_SET((float)4.640562E37F, PH.base.pack) ;
    p91_aux2_SET((float) -2.4350562E38F, PH.base.pack) ;
    p91_aux3_SET((float) -3.6608697E37F, PH.base.pack) ;
    p91_aux4_SET((float) -3.737027E37F, PH.base.pack) ;
    p91_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_DISARMED, PH.base.pack) ;
    p91_nav_mode_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
    p92_time_usec_SET((uint64_t)5132026826558698509L, PH.base.pack) ;
    p92_chan1_raw_SET((uint16_t)(uint16_t)24645, PH.base.pack) ;
    p92_chan2_raw_SET((uint16_t)(uint16_t)27154, PH.base.pack) ;
    p92_chan3_raw_SET((uint16_t)(uint16_t)19228, PH.base.pack) ;
    p92_chan4_raw_SET((uint16_t)(uint16_t)62904, PH.base.pack) ;
    p92_chan5_raw_SET((uint16_t)(uint16_t)46409, PH.base.pack) ;
    p92_chan6_raw_SET((uint16_t)(uint16_t)60772, PH.base.pack) ;
    p92_chan7_raw_SET((uint16_t)(uint16_t)65436, PH.base.pack) ;
    p92_chan8_raw_SET((uint16_t)(uint16_t)10158, PH.base.pack) ;
    p92_chan9_raw_SET((uint16_t)(uint16_t)59129, PH.base.pack) ;
    p92_chan10_raw_SET((uint16_t)(uint16_t)60789, PH.base.pack) ;
    p92_chan11_raw_SET((uint16_t)(uint16_t)24232, PH.base.pack) ;
    p92_chan12_raw_SET((uint16_t)(uint16_t)1543, PH.base.pack) ;
    p92_rssi_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
    p93_time_usec_SET((uint64_t)3544650297114319224L, PH.base.pack) ;
    {
        float  controls [] =  {3.3533808E38F, -1.3639443E38F, -7.0415456E36F, -2.4738384E38F, -1.354842E38F, -2.1814082E38F, -5.8565204E35F, 3.1922777E38F, 1.3861305E38F, 7.8934286E37F, -1.464575E38F, -3.2739338E38F, 1.7874078E38F, -2.7127023E38F, -3.3476764E38F, -1.8926422E37F};
        p93_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    p93_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED, PH.base.pack) ;
    p93_flags_SET((uint64_t)1301875728449153526L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_100(), &PH);
    p100_time_usec_SET((uint64_t)934781250744509605L, PH.base.pack) ;
    p100_sensor_id_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
    p100_flow_x_SET((int16_t)(int16_t)13337, PH.base.pack) ;
    p100_flow_y_SET((int16_t)(int16_t)870, PH.base.pack) ;
    p100_flow_comp_m_x_SET((float)2.5156187E38F, PH.base.pack) ;
    p100_flow_comp_m_y_SET((float) -3.9150405E37F, PH.base.pack) ;
    p100_quality_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
    p100_ground_distance_SET((float)2.3855383E38F, PH.base.pack) ;
    p100_flow_rate_x_SET((float)3.204598E38F, &PH) ;
    p100_flow_rate_y_SET((float) -1.0699184E37F, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
    p101_usec_SET((uint64_t)6391416897348968564L, PH.base.pack) ;
    p101_x_SET((float)8.1017036E37F, PH.base.pack) ;
    p101_y_SET((float)2.6248192E38F, PH.base.pack) ;
    p101_z_SET((float) -2.9489934E38F, PH.base.pack) ;
    p101_roll_SET((float)1.0682604E38F, PH.base.pack) ;
    p101_pitch_SET((float) -3.3687151E38F, PH.base.pack) ;
    p101_yaw_SET((float) -6.883188E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
    p102_usec_SET((uint64_t)7827031774657631729L, PH.base.pack) ;
    p102_x_SET((float) -8.3052015E37F, PH.base.pack) ;
    p102_y_SET((float)2.3869555E38F, PH.base.pack) ;
    p102_z_SET((float)8.484955E37F, PH.base.pack) ;
    p102_roll_SET((float)2.105495E38F, PH.base.pack) ;
    p102_pitch_SET((float)1.812043E38F, PH.base.pack) ;
    p102_yaw_SET((float)1.7660413E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
    p103_usec_SET((uint64_t)2767866094970963780L, PH.base.pack) ;
    p103_x_SET((float)1.2367738E38F, PH.base.pack) ;
    p103_y_SET((float)1.309627E38F, PH.base.pack) ;
    p103_z_SET((float) -1.952911E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
    p104_usec_SET((uint64_t)5959013388348210600L, PH.base.pack) ;
    p104_x_SET((float)2.7469289E38F, PH.base.pack) ;
    p104_y_SET((float)2.3311835E38F, PH.base.pack) ;
    p104_z_SET((float)3.6934487E36F, PH.base.pack) ;
    p104_roll_SET((float)1.9600779E38F, PH.base.pack) ;
    p104_pitch_SET((float)2.4958939E38F, PH.base.pack) ;
    p104_yaw_SET((float) -1.8342677E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIGHRES_IMU_105(), &PH);
    p105_time_usec_SET((uint64_t)2417473733705087826L, PH.base.pack) ;
    p105_xacc_SET((float) -8.312196E37F, PH.base.pack) ;
    p105_yacc_SET((float) -2.6904961E38F, PH.base.pack) ;
    p105_zacc_SET((float) -2.6677147E38F, PH.base.pack) ;
    p105_xgyro_SET((float) -2.627182E38F, PH.base.pack) ;
    p105_ygyro_SET((float) -2.9770446E38F, PH.base.pack) ;
    p105_zgyro_SET((float)3.262825E34F, PH.base.pack) ;
    p105_xmag_SET((float)3.9028832E37F, PH.base.pack) ;
    p105_ymag_SET((float) -2.3642832E38F, PH.base.pack) ;
    p105_zmag_SET((float)2.940419E38F, PH.base.pack) ;
    p105_abs_pressure_SET((float)6.8443377E37F, PH.base.pack) ;
    p105_diff_pressure_SET((float) -2.5482903E38F, PH.base.pack) ;
    p105_pressure_alt_SET((float)9.525604E37F, PH.base.pack) ;
    p105_temperature_SET((float) -1.2546376E38F, PH.base.pack) ;
    p105_fields_updated_SET((uint16_t)(uint16_t)45230, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
    p106_time_usec_SET((uint64_t)9207426841133223007L, PH.base.pack) ;
    p106_sensor_id_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
    p106_integration_time_us_SET((uint32_t)3265044279L, PH.base.pack) ;
    p106_integrated_x_SET((float) -3.2598477E38F, PH.base.pack) ;
    p106_integrated_y_SET((float)8.713967E37F, PH.base.pack) ;
    p106_integrated_xgyro_SET((float) -2.3569306E38F, PH.base.pack) ;
    p106_integrated_ygyro_SET((float)3.6132994E37F, PH.base.pack) ;
    p106_integrated_zgyro_SET((float)3.27324E38F, PH.base.pack) ;
    p106_temperature_SET((int16_t)(int16_t)10858, PH.base.pack) ;
    p106_quality_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
    p106_time_delta_distance_us_SET((uint32_t)4256175987L, PH.base.pack) ;
    p106_distance_SET((float)1.4944132E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_SENSOR_107(), &PH);
    p107_time_usec_SET((uint64_t)2829241031198109299L, PH.base.pack) ;
    p107_xacc_SET((float)2.906973E38F, PH.base.pack) ;
    p107_yacc_SET((float)3.0183388E38F, PH.base.pack) ;
    p107_zacc_SET((float) -2.9898245E38F, PH.base.pack) ;
    p107_xgyro_SET((float)3.3424638E38F, PH.base.pack) ;
    p107_ygyro_SET((float) -2.2429583E38F, PH.base.pack) ;
    p107_zgyro_SET((float)7.246257E37F, PH.base.pack) ;
    p107_xmag_SET((float) -1.04804E38F, PH.base.pack) ;
    p107_ymag_SET((float)2.2939776E38F, PH.base.pack) ;
    p107_zmag_SET((float)9.588375E36F, PH.base.pack) ;
    p107_abs_pressure_SET((float)1.6698863E38F, PH.base.pack) ;
    p107_diff_pressure_SET((float)1.905168E38F, PH.base.pack) ;
    p107_pressure_alt_SET((float)2.3757638E38F, PH.base.pack) ;
    p107_temperature_SET((float) -2.8054302E37F, PH.base.pack) ;
    p107_fields_updated_SET((uint32_t)1314585490L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SIM_STATE_108(), &PH);
    p108_q1_SET((float)6.703787E37F, PH.base.pack) ;
    p108_q2_SET((float) -1.05040975E36F, PH.base.pack) ;
    p108_q3_SET((float) -1.5497931E38F, PH.base.pack) ;
    p108_q4_SET((float)2.8859847E38F, PH.base.pack) ;
    p108_roll_SET((float) -1.6940201E38F, PH.base.pack) ;
    p108_pitch_SET((float) -1.5574477E38F, PH.base.pack) ;
    p108_yaw_SET((float)2.9870406E38F, PH.base.pack) ;
    p108_xacc_SET((float) -1.111463E38F, PH.base.pack) ;
    p108_yacc_SET((float)2.1655078E38F, PH.base.pack) ;
    p108_zacc_SET((float)8.1442317E37F, PH.base.pack) ;
    p108_xgyro_SET((float)1.2668951E37F, PH.base.pack) ;
    p108_ygyro_SET((float) -1.7365947E38F, PH.base.pack) ;
    p108_zgyro_SET((float) -8.2743216E37F, PH.base.pack) ;
    p108_lat_SET((float) -3.3632298E38F, PH.base.pack) ;
    p108_lon_SET((float) -4.8811044E37F, PH.base.pack) ;
    p108_alt_SET((float) -1.8116342E38F, PH.base.pack) ;
    p108_std_dev_horz_SET((float)2.5647261E38F, PH.base.pack) ;
    p108_std_dev_vert_SET((float) -7.9093827E37F, PH.base.pack) ;
    p108_vn_SET((float)3.1002107E38F, PH.base.pack) ;
    p108_ve_SET((float) -7.344353E37F, PH.base.pack) ;
    p108_vd_SET((float)4.0559525E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RADIO_STATUS_109(), &PH);
    p109_rssi_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
    p109_remrssi_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
    p109_txbuf_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    p109_noise_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
    p109_remnoise_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
    p109_rxerrors_SET((uint16_t)(uint16_t)34391, PH.base.pack) ;
    p109_fixed__SET((uint16_t)(uint16_t)10032, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
    p110_target_network_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
    p110_target_system_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
    p110_target_component_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)99, (uint8_t)218, (uint8_t)219, (uint8_t)100, (uint8_t)170, (uint8_t)149, (uint8_t)243, (uint8_t)49, (uint8_t)82, (uint8_t)108, (uint8_t)51, (uint8_t)27, (uint8_t)120, (uint8_t)70, (uint8_t)63, (uint8_t)39, (uint8_t)230, (uint8_t)220, (uint8_t)34, (uint8_t)249, (uint8_t)133, (uint8_t)192, (uint8_t)73, (uint8_t)149, (uint8_t)31, (uint8_t)55, (uint8_t)196, (uint8_t)227, (uint8_t)2, (uint8_t)198, (uint8_t)95, (uint8_t)143, (uint8_t)80, (uint8_t)132, (uint8_t)188, (uint8_t)199, (uint8_t)183, (uint8_t)187, (uint8_t)17, (uint8_t)104, (uint8_t)198, (uint8_t)193, (uint8_t)62, (uint8_t)57, (uint8_t)83, (uint8_t)186, (uint8_t)231, (uint8_t)27, (uint8_t)67, (uint8_t)102, (uint8_t)12, (uint8_t)120, (uint8_t)248, (uint8_t)150, (uint8_t)16, (uint8_t)125, (uint8_t)126, (uint8_t)83, (uint8_t)244, (uint8_t)185, (uint8_t)145, (uint8_t)3, (uint8_t)75, (uint8_t)29, (uint8_t)121, (uint8_t)194, (uint8_t)186, (uint8_t)85, (uint8_t)56, (uint8_t)211, (uint8_t)153, (uint8_t)64, (uint8_t)242, (uint8_t)104, (uint8_t)194, (uint8_t)52, (uint8_t)57, (uint8_t)222, (uint8_t)250, (uint8_t)160, (uint8_t)215, (uint8_t)57, (uint8_t)151, (uint8_t)167, (uint8_t)11, (uint8_t)25, (uint8_t)78, (uint8_t)20, (uint8_t)90, (uint8_t)233, (uint8_t)106, (uint8_t)253, (uint8_t)26, (uint8_t)7, (uint8_t)42, (uint8_t)113, (uint8_t)229, (uint8_t)204, (uint8_t)72, (uint8_t)184, (uint8_t)19, (uint8_t)181, (uint8_t)241, (uint8_t)220, (uint8_t)152, (uint8_t)156, (uint8_t)191, (uint8_t)235, (uint8_t)175, (uint8_t)9, (uint8_t)198, (uint8_t)246, (uint8_t)46, (uint8_t)69, (uint8_t)153, (uint8_t)46, (uint8_t)125, (uint8_t)225, (uint8_t)65, (uint8_t)141, (uint8_t)186, (uint8_t)117, (uint8_t)0, (uint8_t)57, (uint8_t)89, (uint8_t)104, (uint8_t)146, (uint8_t)51, (uint8_t)136, (uint8_t)234, (uint8_t)39, (uint8_t)79, (uint8_t)139, (uint8_t)155, (uint8_t)254, (uint8_t)126, (uint8_t)15, (uint8_t)202, (uint8_t)160, (uint8_t)193, (uint8_t)241, (uint8_t)212, (uint8_t)145, (uint8_t)100, (uint8_t)112, (uint8_t)163, (uint8_t)127, (uint8_t)100, (uint8_t)156, (uint8_t)177, (uint8_t)78, (uint8_t)45, (uint8_t)243, (uint8_t)7, (uint8_t)112, (uint8_t)47, (uint8_t)206, (uint8_t)246, (uint8_t)8, (uint8_t)144, (uint8_t)245, (uint8_t)175, (uint8_t)85, (uint8_t)217, (uint8_t)80, (uint8_t)238, (uint8_t)230, (uint8_t)59, (uint8_t)144, (uint8_t)4, (uint8_t)246, (uint8_t)101, (uint8_t)207, (uint8_t)140, (uint8_t)89, (uint8_t)199, (uint8_t)54, (uint8_t)165, (uint8_t)23, (uint8_t)18, (uint8_t)5, (uint8_t)216, (uint8_t)237, (uint8_t)116, (uint8_t)238, (uint8_t)15, (uint8_t)32, (uint8_t)131, (uint8_t)244, (uint8_t)255, (uint8_t)52, (uint8_t)237, (uint8_t)203, (uint8_t)153, (uint8_t)10, (uint8_t)195, (uint8_t)166, (uint8_t)140, (uint8_t)170, (uint8_t)188, (uint8_t)42, (uint8_t)228, (uint8_t)110, (uint8_t)73, (uint8_t)192, (uint8_t)41, (uint8_t)30, (uint8_t)157, (uint8_t)194, (uint8_t)32, (uint8_t)109, (uint8_t)153, (uint8_t)169, (uint8_t)152, (uint8_t)247, (uint8_t)106, (uint8_t)2, (uint8_t)102, (uint8_t)52, (uint8_t)183, (uint8_t)141, (uint8_t)20, (uint8_t)41, (uint8_t)52, (uint8_t)184, (uint8_t)167, (uint8_t)145, (uint8_t)140, (uint8_t)74, (uint8_t)114, (uint8_t)61, (uint8_t)159, (uint8_t)172, (uint8_t)246, (uint8_t)5, (uint8_t)62, (uint8_t)106, (uint8_t)232, (uint8_t)199, (uint8_t)60, (uint8_t)160, (uint8_t)37, (uint8_t)180, (uint8_t)242, (uint8_t)208, (uint8_t)182, (uint8_t)174, (uint8_t)98, (uint8_t)0, (uint8_t)178, (uint8_t)233};
        p110_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TIMESYNC_111(), &PH);
    p111_tc1_SET((int64_t) -256174467303935043L, PH.base.pack) ;
    p111_ts1_SET((int64_t)4917205953139654590L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_TRIGGER_112(), &PH);
    p112_time_usec_SET((uint64_t)3264311955426296796L, PH.base.pack) ;
    p112_seq_SET((uint32_t)473561553L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_GPS_113(), &PH);
    p113_time_usec_SET((uint64_t)5180733595687596988L, PH.base.pack) ;
    p113_fix_type_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
    p113_lat_SET((int32_t) -1652454353, PH.base.pack) ;
    p113_lon_SET((int32_t) -1869453582, PH.base.pack) ;
    p113_alt_SET((int32_t) -317233827, PH.base.pack) ;
    p113_eph_SET((uint16_t)(uint16_t)58313, PH.base.pack) ;
    p113_epv_SET((uint16_t)(uint16_t)50241, PH.base.pack) ;
    p113_vel_SET((uint16_t)(uint16_t)8600, PH.base.pack) ;
    p113_vn_SET((int16_t)(int16_t)14476, PH.base.pack) ;
    p113_ve_SET((int16_t)(int16_t) -23631, PH.base.pack) ;
    p113_vd_SET((int16_t)(int16_t)20444, PH.base.pack) ;
    p113_cog_SET((uint16_t)(uint16_t)32887, PH.base.pack) ;
    p113_satellites_visible_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
    p114_time_usec_SET((uint64_t)2094864144498631091L, PH.base.pack) ;
    p114_sensor_id_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
    p114_integration_time_us_SET((uint32_t)3727196708L, PH.base.pack) ;
    p114_integrated_x_SET((float)1.8022572E38F, PH.base.pack) ;
    p114_integrated_y_SET((float) -1.2630504E38F, PH.base.pack) ;
    p114_integrated_xgyro_SET((float)1.3774321E38F, PH.base.pack) ;
    p114_integrated_ygyro_SET((float) -4.240771E37F, PH.base.pack) ;
    p114_integrated_zgyro_SET((float)3.0701849E38F, PH.base.pack) ;
    p114_temperature_SET((int16_t)(int16_t) -13297, PH.base.pack) ;
    p114_quality_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
    p114_time_delta_distance_us_SET((uint32_t)4100474324L, PH.base.pack) ;
    p114_distance_SET((float)5.871347E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_STATE_QUATERNION_115(), &PH);
    p115_time_usec_SET((uint64_t)331740915368991327L, PH.base.pack) ;
    {
        float  attitude_quaternion [] =  {-9.744728E37F, 3.0502683E38F, 2.4685354E38F, 2.6784522E38F};
        p115_attitude_quaternion_SET(&attitude_quaternion, 0, &PH.base.pack) ;
    }
    p115_rollspeed_SET((float)2.895328E38F, PH.base.pack) ;
    p115_pitchspeed_SET((float) -2.542096E38F, PH.base.pack) ;
    p115_yawspeed_SET((float) -1.4780292E38F, PH.base.pack) ;
    p115_lat_SET((int32_t) -1618312904, PH.base.pack) ;
    p115_lon_SET((int32_t) -403226961, PH.base.pack) ;
    p115_alt_SET((int32_t)604880881, PH.base.pack) ;
    p115_vx_SET((int16_t)(int16_t)17908, PH.base.pack) ;
    p115_vy_SET((int16_t)(int16_t)8014, PH.base.pack) ;
    p115_vz_SET((int16_t)(int16_t)17509, PH.base.pack) ;
    p115_ind_airspeed_SET((uint16_t)(uint16_t)59846, PH.base.pack) ;
    p115_true_airspeed_SET((uint16_t)(uint16_t)1285, PH.base.pack) ;
    p115_xacc_SET((int16_t)(int16_t) -624, PH.base.pack) ;
    p115_yacc_SET((int16_t)(int16_t)26939, PH.base.pack) ;
    p115_zacc_SET((int16_t)(int16_t)25749, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU2_116(), &PH);
    p116_time_boot_ms_SET((uint32_t)3095074767L, PH.base.pack) ;
    p116_xacc_SET((int16_t)(int16_t)21035, PH.base.pack) ;
    p116_yacc_SET((int16_t)(int16_t) -12938, PH.base.pack) ;
    p116_zacc_SET((int16_t)(int16_t) -8302, PH.base.pack) ;
    p116_xgyro_SET((int16_t)(int16_t) -22784, PH.base.pack) ;
    p116_ygyro_SET((int16_t)(int16_t)1166, PH.base.pack) ;
    p116_zgyro_SET((int16_t)(int16_t) -24165, PH.base.pack) ;
    p116_xmag_SET((int16_t)(int16_t)13103, PH.base.pack) ;
    p116_ymag_SET((int16_t)(int16_t)12229, PH.base.pack) ;
    p116_zmag_SET((int16_t)(int16_t)5079, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_LIST_117(), &PH);
    p117_target_system_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
    p117_target_component_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
    p117_start_SET((uint16_t)(uint16_t)23729, PH.base.pack) ;
    p117_end_SET((uint16_t)(uint16_t)16805, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_ENTRY_118(), &PH);
    p118_id_SET((uint16_t)(uint16_t)65443, PH.base.pack) ;
    p118_num_logs_SET((uint16_t)(uint16_t)33285, PH.base.pack) ;
    p118_last_log_num_SET((uint16_t)(uint16_t)35570, PH.base.pack) ;
    p118_time_utc_SET((uint32_t)1892704718L, PH.base.pack) ;
    p118_size_SET((uint32_t)1054299352L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_DATA_119(), &PH);
    p119_target_system_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    p119_target_component_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
    p119_id_SET((uint16_t)(uint16_t)54040, PH.base.pack) ;
    p119_ofs_SET((uint32_t)2249247898L, PH.base.pack) ;
    p119_count_SET((uint32_t)4210108939L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_DATA_120(), &PH);
    p120_id_SET((uint16_t)(uint16_t)50607, PH.base.pack) ;
    p120_ofs_SET((uint32_t)636006543L, PH.base.pack) ;
    p120_count_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)164, (uint8_t)249, (uint8_t)4, (uint8_t)152, (uint8_t)250, (uint8_t)35, (uint8_t)154, (uint8_t)229, (uint8_t)59, (uint8_t)153, (uint8_t)108, (uint8_t)39, (uint8_t)142, (uint8_t)187, (uint8_t)178, (uint8_t)69, (uint8_t)177, (uint8_t)222, (uint8_t)252, (uint8_t)102, (uint8_t)148, (uint8_t)48, (uint8_t)49, (uint8_t)40, (uint8_t)234, (uint8_t)128, (uint8_t)123, (uint8_t)143, (uint8_t)70, (uint8_t)49, (uint8_t)185, (uint8_t)2, (uint8_t)197, (uint8_t)18, (uint8_t)121, (uint8_t)81, (uint8_t)116, (uint8_t)195, (uint8_t)167, (uint8_t)251, (uint8_t)193, (uint8_t)6, (uint8_t)68, (uint8_t)66, (uint8_t)170, (uint8_t)237, (uint8_t)204, (uint8_t)230, (uint8_t)57, (uint8_t)118, (uint8_t)191, (uint8_t)251, (uint8_t)152, (uint8_t)50, (uint8_t)135, (uint8_t)203, (uint8_t)176, (uint8_t)204, (uint8_t)113, (uint8_t)179, (uint8_t)58, (uint8_t)23, (uint8_t)221, (uint8_t)98, (uint8_t)213, (uint8_t)6, (uint8_t)32, (uint8_t)34, (uint8_t)172, (uint8_t)32, (uint8_t)223, (uint8_t)49, (uint8_t)97, (uint8_t)1, (uint8_t)220, (uint8_t)120, (uint8_t)71, (uint8_t)187, (uint8_t)76, (uint8_t)118, (uint8_t)132, (uint8_t)224, (uint8_t)34, (uint8_t)160, (uint8_t)190, (uint8_t)221, (uint8_t)124, (uint8_t)186, (uint8_t)97, (uint8_t)115};
        p120_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_ERASE_121(), &PH);
    p121_target_system_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
    p121_target_component_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_END_122(), &PH);
    p122_target_system_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
    p122_target_component_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_INJECT_DATA_123(), &PH);
    p123_target_system_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
    p123_target_component_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
    p123_len_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)38, (uint8_t)255, (uint8_t)168, (uint8_t)133, (uint8_t)129, (uint8_t)78, (uint8_t)41, (uint8_t)185, (uint8_t)182, (uint8_t)223, (uint8_t)159, (uint8_t)236, (uint8_t)83, (uint8_t)4, (uint8_t)244, (uint8_t)252, (uint8_t)136, (uint8_t)154, (uint8_t)223, (uint8_t)189, (uint8_t)0, (uint8_t)207, (uint8_t)84, (uint8_t)225, (uint8_t)217, (uint8_t)221, (uint8_t)233, (uint8_t)54, (uint8_t)144, (uint8_t)249, (uint8_t)13, (uint8_t)30, (uint8_t)135, (uint8_t)228, (uint8_t)100, (uint8_t)100, (uint8_t)40, (uint8_t)27, (uint8_t)117, (uint8_t)201, (uint8_t)186, (uint8_t)122, (uint8_t)117, (uint8_t)242, (uint8_t)206, (uint8_t)108, (uint8_t)123, (uint8_t)32, (uint8_t)251, (uint8_t)80, (uint8_t)171, (uint8_t)241, (uint8_t)16, (uint8_t)83, (uint8_t)41, (uint8_t)207, (uint8_t)138, (uint8_t)85, (uint8_t)44, (uint8_t)221, (uint8_t)115, (uint8_t)228, (uint8_t)122, (uint8_t)6, (uint8_t)13, (uint8_t)83, (uint8_t)242, (uint8_t)193, (uint8_t)104, (uint8_t)189, (uint8_t)135, (uint8_t)157, (uint8_t)14, (uint8_t)176, (uint8_t)119, (uint8_t)94, (uint8_t)21, (uint8_t)187, (uint8_t)173, (uint8_t)104, (uint8_t)234, (uint8_t)78, (uint8_t)13, (uint8_t)238, (uint8_t)129, (uint8_t)207, (uint8_t)46, (uint8_t)189, (uint8_t)27, (uint8_t)109, (uint8_t)46, (uint8_t)60, (uint8_t)62, (uint8_t)5, (uint8_t)164, (uint8_t)42, (uint8_t)51, (uint8_t)138, (uint8_t)48, (uint8_t)3, (uint8_t)239, (uint8_t)168, (uint8_t)83, (uint8_t)116, (uint8_t)102, (uint8_t)60, (uint8_t)26, (uint8_t)43, (uint8_t)35, (uint8_t)195};
        p123_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS2_RAW_124(), &PH);
    p124_time_usec_SET((uint64_t)5910866517257731816L, PH.base.pack) ;
    p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS, PH.base.pack) ;
    p124_lat_SET((int32_t)628147452, PH.base.pack) ;
    p124_lon_SET((int32_t)839889265, PH.base.pack) ;
    p124_alt_SET((int32_t)1301751407, PH.base.pack) ;
    p124_eph_SET((uint16_t)(uint16_t)42749, PH.base.pack) ;
    p124_epv_SET((uint16_t)(uint16_t)51823, PH.base.pack) ;
    p124_vel_SET((uint16_t)(uint16_t)3245, PH.base.pack) ;
    p124_cog_SET((uint16_t)(uint16_t)8632, PH.base.pack) ;
    p124_satellites_visible_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
    p124_dgps_numch_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
    p124_dgps_age_SET((uint32_t)4103076271L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POWER_STATUS_125(), &PH);
    p125_Vcc_SET((uint16_t)(uint16_t)62518, PH.base.pack) ;
    p125_Vservo_SET((uint16_t)(uint16_t)11987, PH.base.pack) ;
    p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_CONTROL_126(), &PH);
    p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1, PH.base.pack) ;
    p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND, PH.base.pack) ;
    p126_timeout_SET((uint16_t)(uint16_t)30402, PH.base.pack) ;
    p126_baudrate_SET((uint32_t)1420402435L, PH.base.pack) ;
    p126_count_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)159, (uint8_t)195, (uint8_t)14, (uint8_t)62, (uint8_t)199, (uint8_t)12, (uint8_t)28, (uint8_t)49, (uint8_t)95, (uint8_t)65, (uint8_t)201, (uint8_t)204, (uint8_t)93, (uint8_t)60, (uint8_t)41, (uint8_t)56, (uint8_t)177, (uint8_t)23, (uint8_t)42, (uint8_t)179, (uint8_t)97, (uint8_t)17, (uint8_t)116, (uint8_t)117, (uint8_t)215, (uint8_t)223, (uint8_t)159, (uint8_t)243, (uint8_t)10, (uint8_t)30, (uint8_t)191, (uint8_t)27, (uint8_t)78, (uint8_t)38, (uint8_t)94, (uint8_t)183, (uint8_t)234, (uint8_t)178, (uint8_t)112, (uint8_t)82, (uint8_t)240, (uint8_t)82, (uint8_t)109, (uint8_t)77, (uint8_t)250, (uint8_t)65, (uint8_t)204, (uint8_t)173, (uint8_t)173, (uint8_t)28, (uint8_t)98, (uint8_t)132, (uint8_t)13, (uint8_t)39, (uint8_t)73, (uint8_t)167, (uint8_t)186, (uint8_t)95, (uint8_t)226, (uint8_t)73, (uint8_t)254, (uint8_t)118, (uint8_t)49, (uint8_t)121, (uint8_t)82, (uint8_t)87, (uint8_t)87, (uint8_t)96, (uint8_t)112, (uint8_t)229};
        p126_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RTK_127(), &PH);
    p127_time_last_baseline_ms_SET((uint32_t)3859526723L, PH.base.pack) ;
    p127_rtk_receiver_id_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
    p127_wn_SET((uint16_t)(uint16_t)31710, PH.base.pack) ;
    p127_tow_SET((uint32_t)3025047921L, PH.base.pack) ;
    p127_rtk_health_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
    p127_rtk_rate_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
    p127_nsats_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
    p127_baseline_coords_type_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
    p127_baseline_a_mm_SET((int32_t)349485155, PH.base.pack) ;
    p127_baseline_b_mm_SET((int32_t)1593970562, PH.base.pack) ;
    p127_baseline_c_mm_SET((int32_t) -452680255, PH.base.pack) ;
    p127_accuracy_SET((uint32_t)2612661282L, PH.base.pack) ;
    p127_iar_num_hypotheses_SET((int32_t) -2143959482, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS2_RTK_128(), &PH);
    p128_time_last_baseline_ms_SET((uint32_t)673318266L, PH.base.pack) ;
    p128_rtk_receiver_id_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
    p128_wn_SET((uint16_t)(uint16_t)42707, PH.base.pack) ;
    p128_tow_SET((uint32_t)2844368004L, PH.base.pack) ;
    p128_rtk_health_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
    p128_rtk_rate_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
    p128_nsats_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
    p128_baseline_coords_type_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
    p128_baseline_a_mm_SET((int32_t) -1022413920, PH.base.pack) ;
    p128_baseline_b_mm_SET((int32_t)303477750, PH.base.pack) ;
    p128_baseline_c_mm_SET((int32_t) -835228254, PH.base.pack) ;
    p128_accuracy_SET((uint32_t)2603970711L, PH.base.pack) ;
    p128_iar_num_hypotheses_SET((int32_t) -59858084, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU3_129(), &PH);
    p129_time_boot_ms_SET((uint32_t)3481094474L, PH.base.pack) ;
    p129_xacc_SET((int16_t)(int16_t)9478, PH.base.pack) ;
    p129_yacc_SET((int16_t)(int16_t)29896, PH.base.pack) ;
    p129_zacc_SET((int16_t)(int16_t)28051, PH.base.pack) ;
    p129_xgyro_SET((int16_t)(int16_t)19348, PH.base.pack) ;
    p129_ygyro_SET((int16_t)(int16_t)23466, PH.base.pack) ;
    p129_zgyro_SET((int16_t)(int16_t)9308, PH.base.pack) ;
    p129_xmag_SET((int16_t)(int16_t)13245, PH.base.pack) ;
    p129_ymag_SET((int16_t)(int16_t)11616, PH.base.pack) ;
    p129_zmag_SET((int16_t)(int16_t) -17442, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
    p130_type_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
    p130_size_SET((uint32_t)4263745322L, PH.base.pack) ;
    p130_width_SET((uint16_t)(uint16_t)60010, PH.base.pack) ;
    p130_height_SET((uint16_t)(uint16_t)1847, PH.base.pack) ;
    p130_packets_SET((uint16_t)(uint16_t)59692, PH.base.pack) ;
    p130_payload_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
    p130_jpg_quality_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ENCAPSULATED_DATA_131(), &PH);
    p131_seqnr_SET((uint16_t)(uint16_t)57103, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)202, (uint8_t)178, (uint8_t)181, (uint8_t)62, (uint8_t)31, (uint8_t)162, (uint8_t)20, (uint8_t)206, (uint8_t)159, (uint8_t)107, (uint8_t)193, (uint8_t)207, (uint8_t)118, (uint8_t)185, (uint8_t)200, (uint8_t)15, (uint8_t)45, (uint8_t)97, (uint8_t)221, (uint8_t)152, (uint8_t)229, (uint8_t)230, (uint8_t)141, (uint8_t)90, (uint8_t)151, (uint8_t)93, (uint8_t)146, (uint8_t)14, (uint8_t)236, (uint8_t)159, (uint8_t)214, (uint8_t)134, (uint8_t)65, (uint8_t)117, (uint8_t)207, (uint8_t)89, (uint8_t)11, (uint8_t)31, (uint8_t)164, (uint8_t)86, (uint8_t)153, (uint8_t)30, (uint8_t)96, (uint8_t)158, (uint8_t)234, (uint8_t)156, (uint8_t)193, (uint8_t)115, (uint8_t)126, (uint8_t)205, (uint8_t)0, (uint8_t)101, (uint8_t)209, (uint8_t)16, (uint8_t)25, (uint8_t)88, (uint8_t)224, (uint8_t)245, (uint8_t)56, (uint8_t)196, (uint8_t)191, (uint8_t)116, (uint8_t)231, (uint8_t)119, (uint8_t)134, (uint8_t)146, (uint8_t)230, (uint8_t)123, (uint8_t)20, (uint8_t)209, (uint8_t)144, (uint8_t)15, (uint8_t)189, (uint8_t)113, (uint8_t)51, (uint8_t)12, (uint8_t)97, (uint8_t)202, (uint8_t)228, (uint8_t)238, (uint8_t)160, (uint8_t)104, (uint8_t)97, (uint8_t)251, (uint8_t)184, (uint8_t)78, (uint8_t)57, (uint8_t)153, (uint8_t)184, (uint8_t)249, (uint8_t)56, (uint8_t)110, (uint8_t)218, (uint8_t)131, (uint8_t)122, (uint8_t)129, (uint8_t)222, (uint8_t)245, (uint8_t)139, (uint8_t)144, (uint8_t)33, (uint8_t)162, (uint8_t)6, (uint8_t)205, (uint8_t)51, (uint8_t)10, (uint8_t)94, (uint8_t)30, (uint8_t)218, (uint8_t)176, (uint8_t)121, (uint8_t)23, (uint8_t)77, (uint8_t)124, (uint8_t)99, (uint8_t)195, (uint8_t)134, (uint8_t)97, (uint8_t)66, (uint8_t)48, (uint8_t)0, (uint8_t)255, (uint8_t)98, (uint8_t)21, (uint8_t)255, (uint8_t)154, (uint8_t)119, (uint8_t)156, (uint8_t)197, (uint8_t)67, (uint8_t)131, (uint8_t)208, (uint8_t)87, (uint8_t)212, (uint8_t)40, (uint8_t)251, (uint8_t)41, (uint8_t)15, (uint8_t)29, (uint8_t)200, (uint8_t)196, (uint8_t)33, (uint8_t)63, (uint8_t)187, (uint8_t)202, (uint8_t)181, (uint8_t)31, (uint8_t)221, (uint8_t)110, (uint8_t)201, (uint8_t)37, (uint8_t)61, (uint8_t)251, (uint8_t)187, (uint8_t)108, (uint8_t)9, (uint8_t)175, (uint8_t)71, (uint8_t)214, (uint8_t)195, (uint8_t)194, (uint8_t)206, (uint8_t)107, (uint8_t)81, (uint8_t)245, (uint8_t)162, (uint8_t)216, (uint8_t)72, (uint8_t)238, (uint8_t)172, (uint8_t)106, (uint8_t)179, (uint8_t)160, (uint8_t)116, (uint8_t)155, (uint8_t)67, (uint8_t)17, (uint8_t)4, (uint8_t)112, (uint8_t)147, (uint8_t)221, (uint8_t)199, (uint8_t)199, (uint8_t)42, (uint8_t)210, (uint8_t)127, (uint8_t)40, (uint8_t)7, (uint8_t)232, (uint8_t)34, (uint8_t)45, (uint8_t)74, (uint8_t)59, (uint8_t)245, (uint8_t)201, (uint8_t)46, (uint8_t)57, (uint8_t)34, (uint8_t)123, (uint8_t)216, (uint8_t)152, (uint8_t)131, (uint8_t)80, (uint8_t)24, (uint8_t)61, (uint8_t)81, (uint8_t)127, (uint8_t)149, (uint8_t)59, (uint8_t)134, (uint8_t)167, (uint8_t)199, (uint8_t)49, (uint8_t)199, (uint8_t)152, (uint8_t)252, (uint8_t)93, (uint8_t)211, (uint8_t)132, (uint8_t)78, (uint8_t)236, (uint8_t)71, (uint8_t)170, (uint8_t)49, (uint8_t)218, (uint8_t)120, (uint8_t)209, (uint8_t)248, (uint8_t)51, (uint8_t)23, (uint8_t)250, (uint8_t)59, (uint8_t)5, (uint8_t)249, (uint8_t)131, (uint8_t)177, (uint8_t)246, (uint8_t)13, (uint8_t)63, (uint8_t)72, (uint8_t)57, (uint8_t)34, (uint8_t)106, (uint8_t)34, (uint8_t)73, (uint8_t)150, (uint8_t)18, (uint8_t)4, (uint8_t)68, (uint8_t)58, (uint8_t)130, (uint8_t)205, (uint8_t)70};
        p131_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DISTANCE_SENSOR_132(), &PH);
    p132_time_boot_ms_SET((uint32_t)2745564438L, PH.base.pack) ;
    p132_min_distance_SET((uint16_t)(uint16_t)47708, PH.base.pack) ;
    p132_max_distance_SET((uint16_t)(uint16_t)22600, PH.base.pack) ;
    p132_current_distance_SET((uint16_t)(uint16_t)10078, PH.base.pack) ;
    p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND, PH.base.pack) ;
    p132_id_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
    p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_YAW_225, PH.base.pack) ;
    p132_covariance_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_REQUEST_133(), &PH);
    p133_lat_SET((int32_t) -592169448, PH.base.pack) ;
    p133_lon_SET((int32_t)1671562562, PH.base.pack) ;
    p133_grid_spacing_SET((uint16_t)(uint16_t)46949, PH.base.pack) ;
    p133_mask_SET((uint64_t)7214525644638467396L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_DATA_134(), &PH);
    p134_lat_SET((int32_t)2067799188, PH.base.pack) ;
    p134_lon_SET((int32_t)1645896828, PH.base.pack) ;
    p134_grid_spacing_SET((uint16_t)(uint16_t)29415, PH.base.pack) ;
    p134_gridbit_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
    {
        int16_t  data_ [] =  {(int16_t)11628, (int16_t) -27857, (int16_t)24132, (int16_t)5681, (int16_t) -17081, (int16_t)18014, (int16_t)19351, (int16_t)30769, (int16_t) -4567, (int16_t)2148, (int16_t) -12721, (int16_t) -7963, (int16_t) -32316, (int16_t)5739, (int16_t)31891, (int16_t)23946};
        p134_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_CHECK_135(), &PH);
    p135_lat_SET((int32_t) -1448504659, PH.base.pack) ;
    p135_lon_SET((int32_t)1133991993, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_REPORT_136(), &PH);
    p136_lat_SET((int32_t)481071336, PH.base.pack) ;
    p136_lon_SET((int32_t) -1659474490, PH.base.pack) ;
    p136_spacing_SET((uint16_t)(uint16_t)20654, PH.base.pack) ;
    p136_terrain_height_SET((float)5.2287667E37F, PH.base.pack) ;
    p136_current_height_SET((float)3.0370831E38F, PH.base.pack) ;
    p136_pending_SET((uint16_t)(uint16_t)6365, PH.base.pack) ;
    p136_loaded_SET((uint16_t)(uint16_t)9914, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE2_137(), &PH);
    p137_time_boot_ms_SET((uint32_t)3929854300L, PH.base.pack) ;
    p137_press_abs_SET((float) -1.2690781E38F, PH.base.pack) ;
    p137_press_diff_SET((float)7.277434E37F, PH.base.pack) ;
    p137_temperature_SET((int16_t)(int16_t)22483, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATT_POS_MOCAP_138(), &PH);
    p138_time_usec_SET((uint64_t)5202060891212368668L, PH.base.pack) ;
    {
        float  q [] =  {6.296165E37F, -4.7048497E37F, -1.0032557E38F, 1.0851432E38F};
        p138_q_SET(&q, 0, &PH.base.pack) ;
    }
    p138_x_SET((float) -2.5059933E38F, PH.base.pack) ;
    p138_y_SET((float)1.6987E36F, PH.base.pack) ;
    p138_z_SET((float) -8.410495E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
    p139_time_usec_SET((uint64_t)2679176531723335087L, PH.base.pack) ;
    p139_group_mlx_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
    p139_target_system_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
    p139_target_component_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
    {
        float  controls [] =  {-2.4324947E38F, -2.4792295E38F, -1.5507885E37F, -1.7670573E38F, 6.5589114E37F, 1.7854962E38F, 2.9961549E38F, 1.3042933E38F};
        p139_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
    p140_time_usec_SET((uint64_t)5397834563995637057L, PH.base.pack) ;
    p140_group_mlx_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    {
        float  controls [] =  {-8.377306E37F, 2.3101905E37F, -1.9849033E38F, 2.2351822E38F, 3.1154035E38F, 6.3369316E37F, 1.3000757E37F, -2.0742824E38F};
        p140_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ALTITUDE_141(), &PH);
    p141_time_usec_SET((uint64_t)3300152448818003934L, PH.base.pack) ;
    p141_altitude_monotonic_SET((float) -1.0238025E38F, PH.base.pack) ;
    p141_altitude_amsl_SET((float) -5.606967E37F, PH.base.pack) ;
    p141_altitude_local_SET((float)2.5984395E38F, PH.base.pack) ;
    p141_altitude_relative_SET((float) -2.5506523E38F, PH.base.pack) ;
    p141_altitude_terrain_SET((float) -2.578044E38F, PH.base.pack) ;
    p141_bottom_clearance_SET((float)4.7384653E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RESOURCE_REQUEST_142(), &PH);
    p142_request_id_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
    p142_uri_type_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
    {
        uint8_t  uri [] =  {(uint8_t)251, (uint8_t)143, (uint8_t)9, (uint8_t)81, (uint8_t)22, (uint8_t)1, (uint8_t)109, (uint8_t)237, (uint8_t)163, (uint8_t)101, (uint8_t)127, (uint8_t)125, (uint8_t)207, (uint8_t)123, (uint8_t)255, (uint8_t)78, (uint8_t)236, (uint8_t)228, (uint8_t)240, (uint8_t)181, (uint8_t)139, (uint8_t)173, (uint8_t)65, (uint8_t)54, (uint8_t)59, (uint8_t)15, (uint8_t)153, (uint8_t)29, (uint8_t)151, (uint8_t)104, (uint8_t)159, (uint8_t)0, (uint8_t)36, (uint8_t)136, (uint8_t)113, (uint8_t)45, (uint8_t)161, (uint8_t)182, (uint8_t)58, (uint8_t)17, (uint8_t)164, (uint8_t)82, (uint8_t)214, (uint8_t)164, (uint8_t)213, (uint8_t)189, (uint8_t)169, (uint8_t)36, (uint8_t)227, (uint8_t)163, (uint8_t)61, (uint8_t)51, (uint8_t)233, (uint8_t)141, (uint8_t)101, (uint8_t)165, (uint8_t)29, (uint8_t)59, (uint8_t)109, (uint8_t)61, (uint8_t)254, (uint8_t)255, (uint8_t)111, (uint8_t)145, (uint8_t)229, (uint8_t)100, (uint8_t)248, (uint8_t)121, (uint8_t)64, (uint8_t)2, (uint8_t)242, (uint8_t)230, (uint8_t)235, (uint8_t)34, (uint8_t)47, (uint8_t)120, (uint8_t)233, (uint8_t)38, (uint8_t)12, (uint8_t)154, (uint8_t)86, (uint8_t)84, (uint8_t)1, (uint8_t)173, (uint8_t)73, (uint8_t)75, (uint8_t)25, (uint8_t)209, (uint8_t)230, (uint8_t)215, (uint8_t)131, (uint8_t)69, (uint8_t)194, (uint8_t)65, (uint8_t)170, (uint8_t)88, (uint8_t)136, (uint8_t)115, (uint8_t)44, (uint8_t)45, (uint8_t)26, (uint8_t)233, (uint8_t)197, (uint8_t)163, (uint8_t)169, (uint8_t)165, (uint8_t)88, (uint8_t)178, (uint8_t)250, (uint8_t)58, (uint8_t)11, (uint8_t)147, (uint8_t)197, (uint8_t)234, (uint8_t)75, (uint8_t)40, (uint8_t)14, (uint8_t)120, (uint8_t)85, (uint8_t)0};
        p142_uri_SET(&uri, 0, &PH.base.pack) ;
    }
    p142_transfer_type_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
    {
        uint8_t  storage [] =  {(uint8_t)39, (uint8_t)235, (uint8_t)222, (uint8_t)99, (uint8_t)93, (uint8_t)132, (uint8_t)222, (uint8_t)249, (uint8_t)247, (uint8_t)65, (uint8_t)104, (uint8_t)70, (uint8_t)205, (uint8_t)140, (uint8_t)160, (uint8_t)61, (uint8_t)224, (uint8_t)53, (uint8_t)79, (uint8_t)211, (uint8_t)255, (uint8_t)29, (uint8_t)199, (uint8_t)206, (uint8_t)53, (uint8_t)216, (uint8_t)252, (uint8_t)106, (uint8_t)162, (uint8_t)132, (uint8_t)186, (uint8_t)4, (uint8_t)50, (uint8_t)154, (uint8_t)138, (uint8_t)145, (uint8_t)181, (uint8_t)74, (uint8_t)47, (uint8_t)194, (uint8_t)132, (uint8_t)173, (uint8_t)111, (uint8_t)22, (uint8_t)113, (uint8_t)39, (uint8_t)19, (uint8_t)75, (uint8_t)144, (uint8_t)131, (uint8_t)116, (uint8_t)47, (uint8_t)187, (uint8_t)150, (uint8_t)116, (uint8_t)211, (uint8_t)173, (uint8_t)195, (uint8_t)135, (uint8_t)112, (uint8_t)84, (uint8_t)251, (uint8_t)202, (uint8_t)26, (uint8_t)110, (uint8_t)131, (uint8_t)76, (uint8_t)15, (uint8_t)42, (uint8_t)65, (uint8_t)160, (uint8_t)118, (uint8_t)75, (uint8_t)129, (uint8_t)38, (uint8_t)155, (uint8_t)44, (uint8_t)66, (uint8_t)174, (uint8_t)130, (uint8_t)251, (uint8_t)249, (uint8_t)189, (uint8_t)236, (uint8_t)103, (uint8_t)41, (uint8_t)90, (uint8_t)126, (uint8_t)213, (uint8_t)89, (uint8_t)94, (uint8_t)38, (uint8_t)33, (uint8_t)238, (uint8_t)25, (uint8_t)77, (uint8_t)210, (uint8_t)116, (uint8_t)78, (uint8_t)67, (uint8_t)73, (uint8_t)43, (uint8_t)52, (uint8_t)8, (uint8_t)184, (uint8_t)165, (uint8_t)253, (uint8_t)192, (uint8_t)133, (uint8_t)169, (uint8_t)243, (uint8_t)78, (uint8_t)185, (uint8_t)109, (uint8_t)98, (uint8_t)64, (uint8_t)10, (uint8_t)16, (uint8_t)208, (uint8_t)171};
        p142_storage_SET(&storage, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE3_143(), &PH);
    p143_time_boot_ms_SET((uint32_t)537465188L, PH.base.pack) ;
    p143_press_abs_SET((float) -1.2559362E38F, PH.base.pack) ;
    p143_press_diff_SET((float) -2.362353E38F, PH.base.pack) ;
    p143_temperature_SET((int16_t)(int16_t)20189, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FOLLOW_TARGET_144(), &PH);
    p144_timestamp_SET((uint64_t)1980826667193809501L, PH.base.pack) ;
    p144_est_capabilities_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
    p144_lat_SET((int32_t) -1530550536, PH.base.pack) ;
    p144_lon_SET((int32_t)686165865, PH.base.pack) ;
    p144_alt_SET((float)1.6963673E38F, PH.base.pack) ;
    {
        float  vel [] =  {1.217102E38F, 2.342185E38F, -2.5886784E37F};
        p144_vel_SET(&vel, 0, &PH.base.pack) ;
    }
    {
        float  acc [] =  {-2.5087087E38F, 1.0159578E38F, 2.2831982E38F};
        p144_acc_SET(&acc, 0, &PH.base.pack) ;
    }
    {
        float  attitude_q [] =  {-2.4308752E38F, 3.018251E38F, -2.302798E38F, 2.9228236E38F};
        p144_attitude_q_SET(&attitude_q, 0, &PH.base.pack) ;
    }
    {
        float  rates [] =  {1.7166445E38F, 7.845173E37F, -3.1144913E37F};
        p144_rates_SET(&rates, 0, &PH.base.pack) ;
    }
    {
        float  position_cov [] =  {-1.0068776E38F, 2.0317249E38F, -6.8205053E37F};
        p144_position_cov_SET(&position_cov, 0, &PH.base.pack) ;
    }
    p144_custom_state_SET((uint64_t)8239001736069494150L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
    p146_time_usec_SET((uint64_t)8821712913345694618L, PH.base.pack) ;
    p146_x_acc_SET((float) -2.5887547E38F, PH.base.pack) ;
    p146_y_acc_SET((float)2.4585371E38F, PH.base.pack) ;
    p146_z_acc_SET((float) -3.0719798E37F, PH.base.pack) ;
    p146_x_vel_SET((float)2.3197045E38F, PH.base.pack) ;
    p146_y_vel_SET((float)9.424602E37F, PH.base.pack) ;
    p146_z_vel_SET((float) -3.2034409E38F, PH.base.pack) ;
    p146_x_pos_SET((float)2.6292323E38F, PH.base.pack) ;
    p146_y_pos_SET((float) -2.8988743E38F, PH.base.pack) ;
    p146_z_pos_SET((float) -2.20766E38F, PH.base.pack) ;
    p146_airspeed_SET((float)1.6459703E38F, PH.base.pack) ;
    {
        float  vel_variance [] =  {-1.8427802E38F, -2.6043304E38F, 2.2090092E37F};
        p146_vel_variance_SET(&vel_variance, 0, &PH.base.pack) ;
    }
    {
        float  pos_variance [] =  {3.8678897E37F, -2.3620478E38F, 9.89111E37F};
        p146_pos_variance_SET(&pos_variance, 0, &PH.base.pack) ;
    }
    {
        float  q [] =  {2.0180042E38F, -2.0974545E38F, -1.873893E38F, 1.6610065E38F};
        p146_q_SET(&q, 0, &PH.base.pack) ;
    }
    p146_roll_rate_SET((float) -2.0072801E38F, PH.base.pack) ;
    p146_pitch_rate_SET((float) -3.15599E38F, PH.base.pack) ;
    p146_yaw_rate_SET((float)1.8639587E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_BATTERY_STATUS_147(), &PH);
    p147_id_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
    p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL, PH.base.pack) ;
    p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN, PH.base.pack) ;
    p147_temperature_SET((int16_t)(int16_t) -23128, PH.base.pack) ;
    {
        uint16_t  voltages [] =  {(uint16_t)42449, (uint16_t)15215, (uint16_t)52542, (uint16_t)26912, (uint16_t)56100, (uint16_t)56842, (uint16_t)48696, (uint16_t)50926, (uint16_t)2083, (uint16_t)54154};
        p147_voltages_SET(&voltages, 0, &PH.base.pack) ;
    }
    p147_current_battery_SET((int16_t)(int16_t)4132, PH.base.pack) ;
    p147_current_consumed_SET((int32_t)501078885, PH.base.pack) ;
    p147_energy_consumed_SET((int32_t)41460346, PH.base.pack) ;
    p147_battery_remaining_SET((int8_t)(int8_t)35, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_148(), &PH);
    p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION, PH.base.pack) ;
    p148_flight_sw_version_SET((uint32_t)2920604554L, PH.base.pack) ;
    p148_middleware_sw_version_SET((uint32_t)174241854L, PH.base.pack) ;
    p148_os_sw_version_SET((uint32_t)1186282655L, PH.base.pack) ;
    p148_board_version_SET((uint32_t)1353406112L, PH.base.pack) ;
    {
        uint8_t  flight_custom_version [] =  {(uint8_t)116, (uint8_t)253, (uint8_t)109, (uint8_t)36, (uint8_t)178, (uint8_t)232, (uint8_t)170, (uint8_t)118};
        p148_flight_custom_version_SET(&flight_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  middleware_custom_version [] =  {(uint8_t)216, (uint8_t)19, (uint8_t)150, (uint8_t)241, (uint8_t)46, (uint8_t)198, (uint8_t)56, (uint8_t)138};
        p148_middleware_custom_version_SET(&middleware_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  os_custom_version [] =  {(uint8_t)162, (uint8_t)207, (uint8_t)66, (uint8_t)196, (uint8_t)194, (uint8_t)106, (uint8_t)222, (uint8_t)113};
        p148_os_custom_version_SET(&os_custom_version, 0, &PH.base.pack) ;
    }
    p148_vendor_id_SET((uint16_t)(uint16_t)39085, PH.base.pack) ;
    p148_product_id_SET((uint16_t)(uint16_t)37012, PH.base.pack) ;
    p148_uid_SET((uint64_t)4957283315778497190L, PH.base.pack) ;
    {
        uint8_t  uid2 [] =  {(uint8_t)34, (uint8_t)114, (uint8_t)135, (uint8_t)183, (uint8_t)17, (uint8_t)254, (uint8_t)221, (uint8_t)73, (uint8_t)65, (uint8_t)131, (uint8_t)53, (uint8_t)119, (uint8_t)11, (uint8_t)77, (uint8_t)56, (uint8_t)12, (uint8_t)48, (uint8_t)215};
        p148_uid2_SET(&uid2, 0,  &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LANDING_TARGET_149(), &PH);
    p149_time_usec_SET((uint64_t)4555758637209521420L, PH.base.pack) ;
    p149_target_num_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
    p149_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
    p149_angle_x_SET((float)2.0166761E38F, PH.base.pack) ;
    p149_angle_y_SET((float)5.802958E37F, PH.base.pack) ;
    p149_distance_SET((float) -1.9702209E38F, PH.base.pack) ;
    p149_size_x_SET((float)4.2865696E37F, PH.base.pack) ;
    p149_size_y_SET((float)2.2368683E38F, PH.base.pack) ;
    p149_x_SET((float) -3.2083405E37F, &PH) ;
    p149_y_SET((float) -1.421172E37F, &PH) ;
    p149_z_SET((float)1.2673731E38F, &PH) ;
    {
        float  q [] =  {-1.1102174E38F, 1.1133783E38F, -1.3499175E38F, -2.7329847E38F};
        p149_q_SET(&q, 0,  &PH) ;
    }
    p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL, PH.base.pack) ;
    p149_position_valid_SET((uint8_t)(uint8_t)2, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AQ_TELEMETRY_F_150(), &PH);
    p150_Index_SET((uint16_t)(uint16_t)8215, PH.base.pack) ;
    p150_value1_SET((float)3.2425565E38F, PH.base.pack) ;
    p150_value2_SET((float)9.732832E37F, PH.base.pack) ;
    p150_value3_SET((float) -1.6857911E37F, PH.base.pack) ;
    p150_value4_SET((float) -1.4117375E38F, PH.base.pack) ;
    p150_value5_SET((float)2.0747526E38F, PH.base.pack) ;
    p150_value6_SET((float) -3.1870947E37F, PH.base.pack) ;
    p150_value7_SET((float)3.1487745E38F, PH.base.pack) ;
    p150_value8_SET((float) -1.5523871E38F, PH.base.pack) ;
    p150_value9_SET((float)2.7646259E38F, PH.base.pack) ;
    p150_value10_SET((float)2.8985792E38F, PH.base.pack) ;
    p150_value11_SET((float) -7.7230857E37F, PH.base.pack) ;
    p150_value12_SET((float) -1.4645382E37F, PH.base.pack) ;
    p150_value13_SET((float) -1.3467451E38F, PH.base.pack) ;
    p150_value14_SET((float)2.0067446E38F, PH.base.pack) ;
    p150_value15_SET((float) -2.8991206E38F, PH.base.pack) ;
    p150_value16_SET((float) -1.547037E38F, PH.base.pack) ;
    p150_value17_SET((float)4.1874056E37F, PH.base.pack) ;
    p150_value18_SET((float) -9.774463E37F, PH.base.pack) ;
    p150_value19_SET((float) -5.708449E37F, PH.base.pack) ;
    p150_value20_SET((float) -2.8756786E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AQ_ESC_TELEMETRY_152(), &PH);
    p152_time_boot_ms_SET((uint32_t)3441716330L, PH.base.pack) ;
    p152_seq_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
    p152_num_motors_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
    p152_num_in_seq_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
    {
        uint8_t  escid [] =  {(uint8_t)193, (uint8_t)116, (uint8_t)73, (uint8_t)10};
        p152_escid_SET(&escid, 0, &PH.base.pack) ;
    }
    {
        uint16_t  status_age [] =  {(uint16_t)8924, (uint16_t)60097, (uint16_t)56688, (uint16_t)56208};
        p152_status_age_SET(&status_age, 0, &PH.base.pack) ;
    }
    {
        uint8_t  data_version [] =  {(uint8_t)66, (uint8_t)65, (uint8_t)207, (uint8_t)136};
        p152_data_version_SET(&data_version, 0, &PH.base.pack) ;
    }
    {
        uint32_t  data0 [] =  {4156862397L, 4243691641L, 3396688498L, 1562849532L};
        p152_data0_SET(&data0, 0, &PH.base.pack) ;
    }
    {
        uint32_t  data1 [] =  {3615265512L, 2504953082L, 2323936685L, 2486157522L};
        p152_data1_SET(&data1, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ESTIMATOR_STATUS_230(), &PH);
    p230_time_usec_SET((uint64_t)7652877906342958648L, PH.base.pack) ;
    p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_HORIZ, PH.base.pack) ;
    p230_vel_ratio_SET((float) -2.6059235E38F, PH.base.pack) ;
    p230_pos_horiz_ratio_SET((float) -1.7590587E38F, PH.base.pack) ;
    p230_pos_vert_ratio_SET((float) -3.29675E38F, PH.base.pack) ;
    p230_mag_ratio_SET((float) -2.2622217E38F, PH.base.pack) ;
    p230_hagl_ratio_SET((float)2.3594553E38F, PH.base.pack) ;
    p230_tas_ratio_SET((float)1.0018556E38F, PH.base.pack) ;
    p230_pos_horiz_accuracy_SET((float) -9.555731E36F, PH.base.pack) ;
    p230_pos_vert_accuracy_SET((float) -2.54017E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_WIND_COV_231(), &PH);
    p231_time_usec_SET((uint64_t)1449445597784834774L, PH.base.pack) ;
    p231_wind_x_SET((float)1.2756421E38F, PH.base.pack) ;
    p231_wind_y_SET((float) -1.6399617E37F, PH.base.pack) ;
    p231_wind_z_SET((float) -7.598185E37F, PH.base.pack) ;
    p231_var_horiz_SET((float) -7.8043254E37F, PH.base.pack) ;
    p231_var_vert_SET((float)1.283285E38F, PH.base.pack) ;
    p231_wind_alt_SET((float)6.739121E37F, PH.base.pack) ;
    p231_horiz_accuracy_SET((float)1.4406639E37F, PH.base.pack) ;
    p231_vert_accuracy_SET((float) -2.2794755E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_INPUT_232(), &PH);
    p232_time_usec_SET((uint64_t)2100201480252669762L, PH.base.pack) ;
    p232_gps_id_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
    p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_HORIZ, PH.base.pack) ;
    p232_time_week_ms_SET((uint32_t)490375932L, PH.base.pack) ;
    p232_time_week_SET((uint16_t)(uint16_t)59092, PH.base.pack) ;
    p232_fix_type_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
    p232_lat_SET((int32_t)2028721542, PH.base.pack) ;
    p232_lon_SET((int32_t)1997020008, PH.base.pack) ;
    p232_alt_SET((float)3.4775213E37F, PH.base.pack) ;
    p232_hdop_SET((float) -2.6335204E38F, PH.base.pack) ;
    p232_vdop_SET((float)2.2795664E38F, PH.base.pack) ;
    p232_vn_SET((float)1.721709E38F, PH.base.pack) ;
    p232_ve_SET((float) -3.8897757E37F, PH.base.pack) ;
    p232_vd_SET((float)1.283201E38F, PH.base.pack) ;
    p232_speed_accuracy_SET((float) -1.887837E37F, PH.base.pack) ;
    p232_horiz_accuracy_SET((float) -1.8859453E38F, PH.base.pack) ;
    p232_vert_accuracy_SET((float)3.1735497E38F, PH.base.pack) ;
    p232_satellites_visible_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RTCM_DATA_233(), &PH);
    p233_flags_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    p233_len_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)206, (uint8_t)155, (uint8_t)49, (uint8_t)12, (uint8_t)1, (uint8_t)215, (uint8_t)73, (uint8_t)145, (uint8_t)9, (uint8_t)201, (uint8_t)23, (uint8_t)177, (uint8_t)58, (uint8_t)93, (uint8_t)21, (uint8_t)97, (uint8_t)38, (uint8_t)160, (uint8_t)73, (uint8_t)37, (uint8_t)238, (uint8_t)54, (uint8_t)32, (uint8_t)157, (uint8_t)14, (uint8_t)6, (uint8_t)228, (uint8_t)250, (uint8_t)1, (uint8_t)165, (uint8_t)72, (uint8_t)3, (uint8_t)5, (uint8_t)60, (uint8_t)78, (uint8_t)15, (uint8_t)2, (uint8_t)13, (uint8_t)253, (uint8_t)42, (uint8_t)89, (uint8_t)204, (uint8_t)154, (uint8_t)73, (uint8_t)40, (uint8_t)121, (uint8_t)218, (uint8_t)88, (uint8_t)80, (uint8_t)99, (uint8_t)194, (uint8_t)240, (uint8_t)73, (uint8_t)90, (uint8_t)18, (uint8_t)211, (uint8_t)79, (uint8_t)157, (uint8_t)174, (uint8_t)167, (uint8_t)43, (uint8_t)47, (uint8_t)242, (uint8_t)217, (uint8_t)115, (uint8_t)34, (uint8_t)215, (uint8_t)184, (uint8_t)13, (uint8_t)17, (uint8_t)191, (uint8_t)110, (uint8_t)231, (uint8_t)229, (uint8_t)74, (uint8_t)64, (uint8_t)247, (uint8_t)223, (uint8_t)93, (uint8_t)252, (uint8_t)114, (uint8_t)108, (uint8_t)238, (uint8_t)98, (uint8_t)171, (uint8_t)203, (uint8_t)131, (uint8_t)165, (uint8_t)122, (uint8_t)195, (uint8_t)132, (uint8_t)47, (uint8_t)179, (uint8_t)63, (uint8_t)75, (uint8_t)252, (uint8_t)242, (uint8_t)87, (uint8_t)16, (uint8_t)199, (uint8_t)111, (uint8_t)235, (uint8_t)84, (uint8_t)221, (uint8_t)116, (uint8_t)244, (uint8_t)83, (uint8_t)211, (uint8_t)233, (uint8_t)169, (uint8_t)210, (uint8_t)125, (uint8_t)142, (uint8_t)130, (uint8_t)16, (uint8_t)89, (uint8_t)246, (uint8_t)104, (uint8_t)76, (uint8_t)114, (uint8_t)230, (uint8_t)251, (uint8_t)152, (uint8_t)103, (uint8_t)249, (uint8_t)206, (uint8_t)47, (uint8_t)167, (uint8_t)94, (uint8_t)151, (uint8_t)239, (uint8_t)197, (uint8_t)106, (uint8_t)136, (uint8_t)7, (uint8_t)59, (uint8_t)77, (uint8_t)185, (uint8_t)68, (uint8_t)190, (uint8_t)72, (uint8_t)74, (uint8_t)243, (uint8_t)57, (uint8_t)157, (uint8_t)8, (uint8_t)135, (uint8_t)196, (uint8_t)57, (uint8_t)149, (uint8_t)31, (uint8_t)122, (uint8_t)204, (uint8_t)91, (uint8_t)200, (uint8_t)145, (uint8_t)5, (uint8_t)66, (uint8_t)250, (uint8_t)221, (uint8_t)210, (uint8_t)192, (uint8_t)171, (uint8_t)174, (uint8_t)32, (uint8_t)88, (uint8_t)171, (uint8_t)55, (uint8_t)6, (uint8_t)167, (uint8_t)193, (uint8_t)234, (uint8_t)23, (uint8_t)25, (uint8_t)165, (uint8_t)131, (uint8_t)36, (uint8_t)128, (uint8_t)108, (uint8_t)197};
        p233_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIGH_LATENCY_234(), &PH);
    p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED, PH.base.pack) ;
    p234_custom_mode_SET((uint32_t)3676338054L, PH.base.pack) ;
    p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND, PH.base.pack) ;
    p234_roll_SET((int16_t)(int16_t) -13069, PH.base.pack) ;
    p234_pitch_SET((int16_t)(int16_t) -25567, PH.base.pack) ;
    p234_heading_SET((uint16_t)(uint16_t)59778, PH.base.pack) ;
    p234_throttle_SET((int8_t)(int8_t)22, PH.base.pack) ;
    p234_heading_sp_SET((int16_t)(int16_t) -8957, PH.base.pack) ;
    p234_latitude_SET((int32_t) -1761481171, PH.base.pack) ;
    p234_longitude_SET((int32_t)173395454, PH.base.pack) ;
    p234_altitude_amsl_SET((int16_t)(int16_t) -6793, PH.base.pack) ;
    p234_altitude_sp_SET((int16_t)(int16_t) -7685, PH.base.pack) ;
    p234_airspeed_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
    p234_airspeed_sp_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
    p234_groundspeed_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
    p234_climb_rate_SET((int8_t)(int8_t)25, PH.base.pack) ;
    p234_gps_nsat_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
    p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, PH.base.pack) ;
    p234_battery_remaining_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
    p234_temperature_SET((int8_t)(int8_t) -47, PH.base.pack) ;
    p234_temperature_air_SET((int8_t)(int8_t)53, PH.base.pack) ;
    p234_failsafe_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
    p234_wp_num_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
    p234_wp_distance_SET((uint16_t)(uint16_t)64875, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VIBRATION_241(), &PH);
    p241_time_usec_SET((uint64_t)5068928243012022426L, PH.base.pack) ;
    p241_vibration_x_SET((float) -9.704041E37F, PH.base.pack) ;
    p241_vibration_y_SET((float)6.5454713E37F, PH.base.pack) ;
    p241_vibration_z_SET((float) -2.3902278E38F, PH.base.pack) ;
    p241_clipping_0_SET((uint32_t)621335521L, PH.base.pack) ;
    p241_clipping_1_SET((uint32_t)1298291290L, PH.base.pack) ;
    p241_clipping_2_SET((uint32_t)3733918262L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HOME_POSITION_242(), &PH);
    p242_latitude_SET((int32_t) -1760860767, PH.base.pack) ;
    p242_longitude_SET((int32_t) -1993199256, PH.base.pack) ;
    p242_altitude_SET((int32_t)1692622732, PH.base.pack) ;
    p242_x_SET((float)1.7036153E38F, PH.base.pack) ;
    p242_y_SET((float)1.0205843E38F, PH.base.pack) ;
    p242_z_SET((float) -1.4255104E38F, PH.base.pack) ;
    {
        float  q [] =  {1.6234098E38F, -3.0179888E37F, 2.234242E38F, -3.2238965E38F};
        p242_q_SET(&q, 0, &PH.base.pack) ;
    }
    p242_approach_x_SET((float)2.231906E38F, PH.base.pack) ;
    p242_approach_y_SET((float)1.878204E38F, PH.base.pack) ;
    p242_approach_z_SET((float) -1.8964803E38F, PH.base.pack) ;
    p242_time_usec_SET((uint64_t)725662011587815512L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_HOME_POSITION_243(), &PH);
    p243_target_system_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
    p243_latitude_SET((int32_t) -804071778, PH.base.pack) ;
    p243_longitude_SET((int32_t) -305361053, PH.base.pack) ;
    p243_altitude_SET((int32_t)757429272, PH.base.pack) ;
    p243_x_SET((float)2.7895692E38F, PH.base.pack) ;
    p243_y_SET((float) -1.7151357E38F, PH.base.pack) ;
    p243_z_SET((float)9.230728E37F, PH.base.pack) ;
    {
        float  q [] =  {-8.429615E37F, -1.7385611E36F, 8.521392E37F, -1.5408644E38F};
        p243_q_SET(&q, 0, &PH.base.pack) ;
    }
    p243_approach_x_SET((float)4.1478352E36F, PH.base.pack) ;
    p243_approach_y_SET((float) -1.4909387E38F, PH.base.pack) ;
    p243_approach_z_SET((float) -1.0824574E38F, PH.base.pack) ;
    p243_time_usec_SET((uint64_t)8394373033326383825L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MESSAGE_INTERVAL_244(), &PH);
    p244_message_id_SET((uint16_t)(uint16_t)13420, PH.base.pack) ;
    p244_interval_us_SET((int32_t) -1301573513, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_EXTENDED_SYS_STATE_245(), &PH);
    p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED, PH.base.pack) ;
    p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ADSB_VEHICLE_246(), &PH);
    p246_ICAO_address_SET((uint32_t)814909341L, PH.base.pack) ;
    p246_lat_SET((int32_t) -764407051, PH.base.pack) ;
    p246_lon_SET((int32_t)1392649829, PH.base.pack) ;
    p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
    p246_altitude_SET((int32_t)243407679, PH.base.pack) ;
    p246_heading_SET((uint16_t)(uint16_t)53370, PH.base.pack) ;
    p246_hor_velocity_SET((uint16_t)(uint16_t)29485, PH.base.pack) ;
    p246_ver_velocity_SET((int16_t)(int16_t) -876, PH.base.pack) ;
    {
        char16_t   callsign = "vx";
        p246_callsign_SET(&callsign, 0,  sizeof(callsign), &PH) ;
    }
    p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_NO_INFO, PH.base.pack) ;
    p246_tslc_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
    p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK, PH.base.pack) ;
    p246_squawk_SET((uint16_t)(uint16_t)26092, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COLLISION_247(), &PH);
    p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, PH.base.pack) ;
    p247_id_SET((uint32_t)1585234632L, PH.base.pack) ;
    p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_NONE, PH.base.pack) ;
    p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW, PH.base.pack) ;
    p247_time_to_minimum_delta_SET((float) -5.5649167E37F, PH.base.pack) ;
    p247_altitude_minimum_delta_SET((float) -2.4749517E37F, PH.base.pack) ;
    p247_horizontal_minimum_delta_SET((float)5.8197773E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_V2_EXTENSION_248(), &PH);
    p248_target_network_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
    p248_target_system_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
    p248_target_component_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
    p248_message_type_SET((uint16_t)(uint16_t)31015, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)247, (uint8_t)243, (uint8_t)77, (uint8_t)95, (uint8_t)162, (uint8_t)108, (uint8_t)199, (uint8_t)135, (uint8_t)108, (uint8_t)244, (uint8_t)104, (uint8_t)153, (uint8_t)235, (uint8_t)164, (uint8_t)133, (uint8_t)171, (uint8_t)33, (uint8_t)14, (uint8_t)136, (uint8_t)109, (uint8_t)55, (uint8_t)57, (uint8_t)114, (uint8_t)137, (uint8_t)138, (uint8_t)141, (uint8_t)144, (uint8_t)174, (uint8_t)134, (uint8_t)97, (uint8_t)241, (uint8_t)130, (uint8_t)175, (uint8_t)14, (uint8_t)216, (uint8_t)112, (uint8_t)200, (uint8_t)166, (uint8_t)116, (uint8_t)9, (uint8_t)59, (uint8_t)96, (uint8_t)194, (uint8_t)8, (uint8_t)11, (uint8_t)30, (uint8_t)146, (uint8_t)133, (uint8_t)176, (uint8_t)174, (uint8_t)129, (uint8_t)88, (uint8_t)48, (uint8_t)109, (uint8_t)101, (uint8_t)241, (uint8_t)57, (uint8_t)209, (uint8_t)171, (uint8_t)210, (uint8_t)209, (uint8_t)55, (uint8_t)10, (uint8_t)45, (uint8_t)52, (uint8_t)63, (uint8_t)32, (uint8_t)128, (uint8_t)251, (uint8_t)252, (uint8_t)223, (uint8_t)145, (uint8_t)185, (uint8_t)236, (uint8_t)69, (uint8_t)124, (uint8_t)255, (uint8_t)131, (uint8_t)87, (uint8_t)32, (uint8_t)136, (uint8_t)198, (uint8_t)144, (uint8_t)154, (uint8_t)23, (uint8_t)143, (uint8_t)171, (uint8_t)160, (uint8_t)28, (uint8_t)22, (uint8_t)253, (uint8_t)194, (uint8_t)191, (uint8_t)60, (uint8_t)164, (uint8_t)63, (uint8_t)152, (uint8_t)27, (uint8_t)151, (uint8_t)71, (uint8_t)228, (uint8_t)34, (uint8_t)104, (uint8_t)192, (uint8_t)31, (uint8_t)54, (uint8_t)247, (uint8_t)2, (uint8_t)33, (uint8_t)100, (uint8_t)65, (uint8_t)194, (uint8_t)171, (uint8_t)76, (uint8_t)114, (uint8_t)155, (uint8_t)57, (uint8_t)179, (uint8_t)193, (uint8_t)200, (uint8_t)193, (uint8_t)39, (uint8_t)97, (uint8_t)197, (uint8_t)158, (uint8_t)68, (uint8_t)181, (uint8_t)198, (uint8_t)60, (uint8_t)16, (uint8_t)242, (uint8_t)109, (uint8_t)151, (uint8_t)197, (uint8_t)43, (uint8_t)106, (uint8_t)195, (uint8_t)220, (uint8_t)137, (uint8_t)3, (uint8_t)113, (uint8_t)189, (uint8_t)78, (uint8_t)3, (uint8_t)207, (uint8_t)91, (uint8_t)194, (uint8_t)198, (uint8_t)20, (uint8_t)3, (uint8_t)129, (uint8_t)144, (uint8_t)230, (uint8_t)114, (uint8_t)194, (uint8_t)178, (uint8_t)5, (uint8_t)139, (uint8_t)228, (uint8_t)219, (uint8_t)208, (uint8_t)233, (uint8_t)33, (uint8_t)22, (uint8_t)236, (uint8_t)251, (uint8_t)213, (uint8_t)0, (uint8_t)232, (uint8_t)207, (uint8_t)68, (uint8_t)111, (uint8_t)232, (uint8_t)46, (uint8_t)223, (uint8_t)242, (uint8_t)63, (uint8_t)132, (uint8_t)157, (uint8_t)59, (uint8_t)191, (uint8_t)232, (uint8_t)86, (uint8_t)211, (uint8_t)46, (uint8_t)178, (uint8_t)31, (uint8_t)53, (uint8_t)6, (uint8_t)116, (uint8_t)2, (uint8_t)111, (uint8_t)69, (uint8_t)250, (uint8_t)128, (uint8_t)103, (uint8_t)161, (uint8_t)138, (uint8_t)15, (uint8_t)84, (uint8_t)20, (uint8_t)83, (uint8_t)7, (uint8_t)188, (uint8_t)0, (uint8_t)91, (uint8_t)128, (uint8_t)206, (uint8_t)33, (uint8_t)130, (uint8_t)232, (uint8_t)128, (uint8_t)169, (uint8_t)71, (uint8_t)104, (uint8_t)170, (uint8_t)82, (uint8_t)72, (uint8_t)76, (uint8_t)167, (uint8_t)156, (uint8_t)139, (uint8_t)155, (uint8_t)244, (uint8_t)172, (uint8_t)42, (uint8_t)145, (uint8_t)224, (uint8_t)192, (uint8_t)97, (uint8_t)226, (uint8_t)74, (uint8_t)188, (uint8_t)162, (uint8_t)254, (uint8_t)107, (uint8_t)232, (uint8_t)203, (uint8_t)9, (uint8_t)46, (uint8_t)241, (uint8_t)104, (uint8_t)6, (uint8_t)64, (uint8_t)53, (uint8_t)3, (uint8_t)112, (uint8_t)137, (uint8_t)40};
        p248_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MEMORY_VECT_249(), &PH);
    p249_address_SET((uint16_t)(uint16_t)52714, PH.base.pack) ;
    p249_ver_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
    p249_type_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
    {
        int8_t  value [] =  {(int8_t) -34, (int8_t) -6, (int8_t) -23, (int8_t)99, (int8_t) -91, (int8_t) -36, (int8_t)82, (int8_t)125, (int8_t)19, (int8_t)89, (int8_t) -38, (int8_t)50, (int8_t) -40, (int8_t)66, (int8_t) -80, (int8_t)81, (int8_t)101, (int8_t) -18, (int8_t)43, (int8_t)4, (int8_t) -72, (int8_t) -116, (int8_t)72, (int8_t)82, (int8_t)126, (int8_t)39, (int8_t)4, (int8_t) -50, (int8_t) -125, (int8_t)12, (int8_t)87, (int8_t)83};
        p249_value_SET(&value, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEBUG_VECT_250(), &PH);
    {
        char16_t   name = "elGbxeba";
        p250_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p250_time_usec_SET((uint64_t)7613052097526609908L, PH.base.pack) ;
    p250_x_SET((float) -1.7411599E38F, PH.base.pack) ;
    p250_y_SET((float)2.3546346E38F, PH.base.pack) ;
    p250_z_SET((float)1.1731336E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
    p251_time_boot_ms_SET((uint32_t)3310513289L, PH.base.pack) ;
    {
        char16_t   name = "H";
        p251_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p251_value_SET((float)2.0854694E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_INT_252(), &PH);
    p252_time_boot_ms_SET((uint32_t)1898609590L, PH.base.pack) ;
    {
        char16_t   name = "tcxjm";
        p252_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p252_value_SET((int32_t) -1026281735, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_STATUSTEXT_253(), &PH);
    p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_NOTICE, PH.base.pack) ;
    {
        char16_t   text = "eBtbfjwylj";
        p253_text_SET(&text, 0,  sizeof(text), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEBUG_254(), &PH);
    p254_time_boot_ms_SET((uint32_t)1787902610L, PH.base.pack) ;
    p254_ind_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
    p254_value_SET((float)2.9708343E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SETUP_SIGNING_256(), &PH);
    p256_target_system_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
    p256_target_component_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
    {
        uint8_t  secret_key [] =  {(uint8_t)63, (uint8_t)212, (uint8_t)37, (uint8_t)76, (uint8_t)203, (uint8_t)54, (uint8_t)39, (uint8_t)96, (uint8_t)126, (uint8_t)127, (uint8_t)85, (uint8_t)206, (uint8_t)79, (uint8_t)53, (uint8_t)124, (uint8_t)158, (uint8_t)146, (uint8_t)69, (uint8_t)74, (uint8_t)91, (uint8_t)92, (uint8_t)84, (uint8_t)107, (uint8_t)237, (uint8_t)151, (uint8_t)237, (uint8_t)27, (uint8_t)229, (uint8_t)186, (uint8_t)222, (uint8_t)153, (uint8_t)165};
        p256_secret_key_SET(&secret_key, 0, &PH.base.pack) ;
    }
    p256_initial_timestamp_SET((uint64_t)8301344560485216775L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_BUTTON_CHANGE_257(), &PH);
    p257_time_boot_ms_SET((uint32_t)1345458872L, PH.base.pack) ;
    p257_last_change_ms_SET((uint32_t)3542142868L, PH.base.pack) ;
    p257_state_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PLAY_TUNE_258(), &PH);
    p258_target_system_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
    p258_target_component_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
    {
        char16_t   tune = "xyNaOprqhqwqjgjZ";
        p258_tune_SET(&tune, 0,  sizeof(tune), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_INFORMATION_259(), &PH);
    p259_time_boot_ms_SET((uint32_t)3234969089L, PH.base.pack) ;
    {
        uint8_t  vendor_name [] =  {(uint8_t)160, (uint8_t)98, (uint8_t)223, (uint8_t)117, (uint8_t)178, (uint8_t)43, (uint8_t)236, (uint8_t)83, (uint8_t)159, (uint8_t)152, (uint8_t)255, (uint8_t)142, (uint8_t)180, (uint8_t)228, (uint8_t)1, (uint8_t)75, (uint8_t)25, (uint8_t)119, (uint8_t)230, (uint8_t)252, (uint8_t)27, (uint8_t)82, (uint8_t)226, (uint8_t)146, (uint8_t)235, (uint8_t)116, (uint8_t)102, (uint8_t)102, (uint8_t)187, (uint8_t)20, (uint8_t)166, (uint8_t)9};
        p259_vendor_name_SET(&vendor_name, 0, &PH.base.pack) ;
    }
    {
        uint8_t  model_name [] =  {(uint8_t)21, (uint8_t)180, (uint8_t)30, (uint8_t)188, (uint8_t)96, (uint8_t)255, (uint8_t)174, (uint8_t)175, (uint8_t)253, (uint8_t)52, (uint8_t)27, (uint8_t)131, (uint8_t)244, (uint8_t)144, (uint8_t)177, (uint8_t)83, (uint8_t)146, (uint8_t)77, (uint8_t)118, (uint8_t)173, (uint8_t)176, (uint8_t)43, (uint8_t)74, (uint8_t)210, (uint8_t)26, (uint8_t)246, (uint8_t)120, (uint8_t)173, (uint8_t)232, (uint8_t)246, (uint8_t)210, (uint8_t)233};
        p259_model_name_SET(&model_name, 0, &PH.base.pack) ;
    }
    p259_firmware_version_SET((uint32_t)1933920185L, PH.base.pack) ;
    p259_focal_length_SET((float) -2.7896485E38F, PH.base.pack) ;
    p259_sensor_size_h_SET((float)1.3110258E38F, PH.base.pack) ;
    p259_sensor_size_v_SET((float) -1.7025564E38F, PH.base.pack) ;
    p259_resolution_h_SET((uint16_t)(uint16_t)17087, PH.base.pack) ;
    p259_resolution_v_SET((uint16_t)(uint16_t)59235, PH.base.pack) ;
    p259_lens_id_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
    p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES, PH.base.pack) ;
    p259_cam_definition_version_SET((uint16_t)(uint16_t)60366, PH.base.pack) ;
    {
        char16_t   cam_definition_uri = "mqsfiFegjHyunxjikpqplqdTzdxsdHvzzJqiadwzpNtsSswjbeitapvdhuuukamkexncoztmazfmdn";
        p259_cam_definition_uri_SET(&cam_definition_uri, 0,  sizeof(cam_definition_uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_SETTINGS_260(), &PH);
    p260_time_boot_ms_SET((uint32_t)611968324L, PH.base.pack) ;
    p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_VIDEO, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_STORAGE_INFORMATION_261(), &PH);
    p261_time_boot_ms_SET((uint32_t)3114893870L, PH.base.pack) ;
    p261_storage_id_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
    p261_storage_count_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
    p261_status_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
    p261_total_capacity_SET((float)8.691132E37F, PH.base.pack) ;
    p261_used_capacity_SET((float)9.823962E37F, PH.base.pack) ;
    p261_available_capacity_SET((float)3.6427664E37F, PH.base.pack) ;
    p261_read_speed_SET((float) -3.0113117E38F, PH.base.pack) ;
    p261_write_speed_SET((float) -2.3249252E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
    p262_time_boot_ms_SET((uint32_t)3612591682L, PH.base.pack) ;
    p262_image_status_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
    p262_video_status_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
    p262_image_interval_SET((float)1.6313E38F, PH.base.pack) ;
    p262_recording_time_ms_SET((uint32_t)1378752798L, PH.base.pack) ;
    p262_available_capacity_SET((float)1.6827012E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
    p263_time_boot_ms_SET((uint32_t)2226187124L, PH.base.pack) ;
    p263_time_utc_SET((uint64_t)5939372248170287011L, PH.base.pack) ;
    p263_camera_id_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
    p263_lat_SET((int32_t) -807719204, PH.base.pack) ;
    p263_lon_SET((int32_t)86641811, PH.base.pack) ;
    p263_alt_SET((int32_t) -209358645, PH.base.pack) ;
    p263_relative_alt_SET((int32_t)1078031775, PH.base.pack) ;
    {
        float  q [] =  {2.3388991E38F, 2.4349483E38F, -2.584012E37F, -1.6634287E38F};
        p263_q_SET(&q, 0, &PH.base.pack) ;
    }
    p263_image_index_SET((int32_t) -161705436, PH.base.pack) ;
    p263_capture_result_SET((int8_t)(int8_t) -37, PH.base.pack) ;
    {
        char16_t   file_url = "hwpuijXirssxquqcrhqjyyfciqzzwewPggpukcdcyefdpvzWqtmRZjiovbvpzvaxxyankphygTQsiyoeagynhrnylnufGnxwbteatgyjgccfinwwxoyxjrtsGrbaozlulZjttqjfomrtZdtlzwObcqjFiaxinZtcfxihrinekkuvvIhaivrokmhtpxfrkjg";
        p263_file_url_SET(&file_url, 0,  sizeof(file_url), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FLIGHT_INFORMATION_264(), &PH);
    p264_time_boot_ms_SET((uint32_t)1387299715L, PH.base.pack) ;
    p264_arming_time_utc_SET((uint64_t)3752838912712854777L, PH.base.pack) ;
    p264_takeoff_time_utc_SET((uint64_t)1361049307422263164L, PH.base.pack) ;
    p264_flight_uuid_SET((uint64_t)647015749760814498L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MOUNT_ORIENTATION_265(), &PH);
    p265_time_boot_ms_SET((uint32_t)2739959273L, PH.base.pack) ;
    p265_roll_SET((float) -1.6318429E38F, PH.base.pack) ;
    p265_pitch_SET((float) -3.4180315E37F, PH.base.pack) ;
    p265_yaw_SET((float) -1.8626783E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_266(), &PH);
    p266_target_system_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
    p266_target_component_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
    p266_sequence_SET((uint16_t)(uint16_t)5, PH.base.pack) ;
    p266_length_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
    p266_first_message_offset_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)249, (uint8_t)216, (uint8_t)73, (uint8_t)44, (uint8_t)180, (uint8_t)167, (uint8_t)138, (uint8_t)6, (uint8_t)23, (uint8_t)253, (uint8_t)0, (uint8_t)159, (uint8_t)124, (uint8_t)220, (uint8_t)153, (uint8_t)78, (uint8_t)131, (uint8_t)59, (uint8_t)66, (uint8_t)83, (uint8_t)148, (uint8_t)76, (uint8_t)17, (uint8_t)72, (uint8_t)223, (uint8_t)59, (uint8_t)121, (uint8_t)248, (uint8_t)93, (uint8_t)187, (uint8_t)7, (uint8_t)22, (uint8_t)82, (uint8_t)119, (uint8_t)234, (uint8_t)32, (uint8_t)154, (uint8_t)102, (uint8_t)116, (uint8_t)245, (uint8_t)31, (uint8_t)159, (uint8_t)227, (uint8_t)237, (uint8_t)235, (uint8_t)87, (uint8_t)90, (uint8_t)190, (uint8_t)69, (uint8_t)29, (uint8_t)13, (uint8_t)144, (uint8_t)84, (uint8_t)48, (uint8_t)12, (uint8_t)209, (uint8_t)142, (uint8_t)145, (uint8_t)130, (uint8_t)73, (uint8_t)63, (uint8_t)81, (uint8_t)8, (uint8_t)31, (uint8_t)167, (uint8_t)194, (uint8_t)144, (uint8_t)239, (uint8_t)13, (uint8_t)154, (uint8_t)218, (uint8_t)162, (uint8_t)224, (uint8_t)177, (uint8_t)208, (uint8_t)91, (uint8_t)228, (uint8_t)126, (uint8_t)215, (uint8_t)71, (uint8_t)3, (uint8_t)174, (uint8_t)122, (uint8_t)246, (uint8_t)1, (uint8_t)33, (uint8_t)86, (uint8_t)202, (uint8_t)177, (uint8_t)75, (uint8_t)26, (uint8_t)81, (uint8_t)161, (uint8_t)155, (uint8_t)253, (uint8_t)159, (uint8_t)218, (uint8_t)224, (uint8_t)50, (uint8_t)133, (uint8_t)168, (uint8_t)104, (uint8_t)25, (uint8_t)77, (uint8_t)49, (uint8_t)121, (uint8_t)173, (uint8_t)229, (uint8_t)251, (uint8_t)24, (uint8_t)33, (uint8_t)198, (uint8_t)167, (uint8_t)41, (uint8_t)54, (uint8_t)60, (uint8_t)22, (uint8_t)3, (uint8_t)118, (uint8_t)138, (uint8_t)86, (uint8_t)127, (uint8_t)70, (uint8_t)202, (uint8_t)31, (uint8_t)182, (uint8_t)95, (uint8_t)148, (uint8_t)25, (uint8_t)83, (uint8_t)206, (uint8_t)96, (uint8_t)234, (uint8_t)226, (uint8_t)194, (uint8_t)76, (uint8_t)180, (uint8_t)24, (uint8_t)49, (uint8_t)112, (uint8_t)194, (uint8_t)132, (uint8_t)89, (uint8_t)83, (uint8_t)60, (uint8_t)14, (uint8_t)74, (uint8_t)67, (uint8_t)254, (uint8_t)29, (uint8_t)178, (uint8_t)75, (uint8_t)203, (uint8_t)167, (uint8_t)53, (uint8_t)244, (uint8_t)247, (uint8_t)218, (uint8_t)194, (uint8_t)87, (uint8_t)215, (uint8_t)78, (uint8_t)240, (uint8_t)144, (uint8_t)214, (uint8_t)26, (uint8_t)229, (uint8_t)8, (uint8_t)166, (uint8_t)59, (uint8_t)32, (uint8_t)128, (uint8_t)86, (uint8_t)57, (uint8_t)93, (uint8_t)226, (uint8_t)156, (uint8_t)171, (uint8_t)76, (uint8_t)138, (uint8_t)86, (uint8_t)61, (uint8_t)241, (uint8_t)64, (uint8_t)210, (uint8_t)155, (uint8_t)64, (uint8_t)248, (uint8_t)195, (uint8_t)176, (uint8_t)226, (uint8_t)151, (uint8_t)130, (uint8_t)146, (uint8_t)191, (uint8_t)31, (uint8_t)140, (uint8_t)71, (uint8_t)28, (uint8_t)35, (uint8_t)64, (uint8_t)99, (uint8_t)103, (uint8_t)229, (uint8_t)119, (uint8_t)146, (uint8_t)127, (uint8_t)115, (uint8_t)19, (uint8_t)14, (uint8_t)17, (uint8_t)2, (uint8_t)65, (uint8_t)212, (uint8_t)170, (uint8_t)90, (uint8_t)119, (uint8_t)43, (uint8_t)230, (uint8_t)90, (uint8_t)20, (uint8_t)215, (uint8_t)170, (uint8_t)45, (uint8_t)147, (uint8_t)58, (uint8_t)132, (uint8_t)124, (uint8_t)107, (uint8_t)176, (uint8_t)213, (uint8_t)13, (uint8_t)76, (uint8_t)183, (uint8_t)173, (uint8_t)242, (uint8_t)16, (uint8_t)177, (uint8_t)130, (uint8_t)187, (uint8_t)225, (uint8_t)210, (uint8_t)65, (uint8_t)190, (uint8_t)208, (uint8_t)120, (uint8_t)255, (uint8_t)200, (uint8_t)30};
        p266_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_ACKED_267(), &PH);
    p267_target_system_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
    p267_target_component_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
    p267_sequence_SET((uint16_t)(uint16_t)31625, PH.base.pack) ;
    p267_length_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
    p267_first_message_offset_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)122, (uint8_t)85, (uint8_t)183, (uint8_t)142, (uint8_t)76, (uint8_t)205, (uint8_t)200, (uint8_t)56, (uint8_t)72, (uint8_t)117, (uint8_t)33, (uint8_t)92, (uint8_t)148, (uint8_t)198, (uint8_t)22, (uint8_t)242, (uint8_t)171, (uint8_t)129, (uint8_t)182, (uint8_t)68, (uint8_t)158, (uint8_t)99, (uint8_t)76, (uint8_t)68, (uint8_t)150, (uint8_t)70, (uint8_t)116, (uint8_t)119, (uint8_t)82, (uint8_t)115, (uint8_t)18, (uint8_t)10, (uint8_t)50, (uint8_t)237, (uint8_t)225, (uint8_t)137, (uint8_t)209, (uint8_t)236, (uint8_t)58, (uint8_t)36, (uint8_t)29, (uint8_t)253, (uint8_t)85, (uint8_t)68, (uint8_t)10, (uint8_t)92, (uint8_t)58, (uint8_t)7, (uint8_t)214, (uint8_t)197, (uint8_t)18, (uint8_t)244, (uint8_t)76, (uint8_t)153, (uint8_t)177, (uint8_t)152, (uint8_t)141, (uint8_t)197, (uint8_t)141, (uint8_t)232, (uint8_t)255, (uint8_t)121, (uint8_t)2, (uint8_t)101, (uint8_t)27, (uint8_t)157, (uint8_t)222, (uint8_t)214, (uint8_t)57, (uint8_t)14, (uint8_t)17, (uint8_t)115, (uint8_t)9, (uint8_t)224, (uint8_t)73, (uint8_t)203, (uint8_t)78, (uint8_t)20, (uint8_t)108, (uint8_t)2, (uint8_t)44, (uint8_t)8, (uint8_t)213, (uint8_t)95, (uint8_t)228, (uint8_t)92, (uint8_t)77, (uint8_t)251, (uint8_t)139, (uint8_t)177, (uint8_t)6, (uint8_t)186, (uint8_t)60, (uint8_t)39, (uint8_t)87, (uint8_t)125, (uint8_t)57, (uint8_t)156, (uint8_t)206, (uint8_t)54, (uint8_t)54, (uint8_t)129, (uint8_t)236, (uint8_t)27, (uint8_t)70, (uint8_t)36, (uint8_t)82, (uint8_t)22, (uint8_t)7, (uint8_t)84, (uint8_t)254, (uint8_t)88, (uint8_t)0, (uint8_t)30, (uint8_t)10, (uint8_t)115, (uint8_t)204, (uint8_t)189, (uint8_t)108, (uint8_t)43, (uint8_t)116, (uint8_t)92, (uint8_t)94, (uint8_t)102, (uint8_t)101, (uint8_t)127, (uint8_t)150, (uint8_t)97, (uint8_t)151, (uint8_t)156, (uint8_t)45, (uint8_t)112, (uint8_t)42, (uint8_t)1, (uint8_t)67, (uint8_t)196, (uint8_t)251, (uint8_t)14, (uint8_t)162, (uint8_t)168, (uint8_t)249, (uint8_t)34, (uint8_t)205, (uint8_t)12, (uint8_t)149, (uint8_t)226, (uint8_t)110, (uint8_t)20, (uint8_t)113, (uint8_t)176, (uint8_t)48, (uint8_t)126, (uint8_t)194, (uint8_t)209, (uint8_t)155, (uint8_t)64, (uint8_t)53, (uint8_t)238, (uint8_t)20, (uint8_t)40, (uint8_t)228, (uint8_t)25, (uint8_t)242, (uint8_t)115, (uint8_t)106, (uint8_t)174, (uint8_t)7, (uint8_t)190, (uint8_t)22, (uint8_t)236, (uint8_t)234, (uint8_t)240, (uint8_t)126, (uint8_t)189, (uint8_t)194, (uint8_t)19, (uint8_t)115, (uint8_t)135, (uint8_t)127, (uint8_t)218, (uint8_t)133, (uint8_t)102, (uint8_t)239, (uint8_t)102, (uint8_t)155, (uint8_t)114, (uint8_t)97, (uint8_t)64, (uint8_t)113, (uint8_t)136, (uint8_t)62, (uint8_t)61, (uint8_t)28, (uint8_t)120, (uint8_t)219, (uint8_t)131, (uint8_t)16, (uint8_t)156, (uint8_t)48, (uint8_t)180, (uint8_t)233, (uint8_t)183, (uint8_t)95, (uint8_t)118, (uint8_t)251, (uint8_t)211, (uint8_t)241, (uint8_t)10, (uint8_t)54, (uint8_t)152, (uint8_t)6, (uint8_t)207, (uint8_t)71, (uint8_t)73, (uint8_t)59, (uint8_t)134, (uint8_t)54, (uint8_t)166, (uint8_t)188, (uint8_t)108, (uint8_t)88, (uint8_t)4, (uint8_t)99, (uint8_t)85, (uint8_t)157, (uint8_t)221, (uint8_t)236, (uint8_t)188, (uint8_t)207, (uint8_t)65, (uint8_t)243, (uint8_t)134, (uint8_t)163, (uint8_t)0, (uint8_t)56, (uint8_t)160, (uint8_t)69, (uint8_t)207, (uint8_t)176, (uint8_t)2, (uint8_t)59, (uint8_t)2, (uint8_t)61, (uint8_t)161, (uint8_t)71, (uint8_t)60, (uint8_t)110, (uint8_t)229, (uint8_t)89};
        p267_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_ACK_268(), &PH);
    p268_target_system_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
    p268_target_component_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
    p268_sequence_SET((uint16_t)(uint16_t)58811, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
    p269_camera_id_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
    p269_status_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
    p269_framerate_SET((float)1.5330611E38F, PH.base.pack) ;
    p269_resolution_h_SET((uint16_t)(uint16_t)46095, PH.base.pack) ;
    p269_resolution_v_SET((uint16_t)(uint16_t)35324, PH.base.pack) ;
    p269_bitrate_SET((uint32_t)38995710L, PH.base.pack) ;
    p269_rotation_SET((uint16_t)(uint16_t)64469, PH.base.pack) ;
    {
        char16_t   uri = "yobuxubpbhitkDhvbfqMOagaglmzvabhcZmthdxKlaqXqokpikkmlfuVqZedalhcdmtpwqclwjjirftTalgkpoevbZstfatgtIyzdjekfjygmorabtqdusjmrSwJsnuwcrqpLxuGizu";
        p269_uri_SET(&uri, 0,  sizeof(uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
    p270_target_system_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
    p270_target_component_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
    p270_camera_id_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
    p270_framerate_SET((float) -2.507923E38F, PH.base.pack) ;
    p270_resolution_h_SET((uint16_t)(uint16_t)28330, PH.base.pack) ;
    p270_resolution_v_SET((uint16_t)(uint16_t)35143, PH.base.pack) ;
    p270_bitrate_SET((uint32_t)564617657L, PH.base.pack) ;
    p270_rotation_SET((uint16_t)(uint16_t)36199, PH.base.pack) ;
    {
        char16_t   uri = "cKtJdelkbcdntgjcuoloklztsIkoFmSttlfltycuuzimxhwfiutlNxzatblyLacaejxrspyMquplsjzadadcctoCqvwzYrvyexwyhxbddaWVpFbqtjsxnuyohmbLgcgysfbxtnOwtbOdomwrkzubrYsvdyPaukavhpzKrhtwvUhpqvhiaZaxwqjwhGaaykovywvzPxqzhpnjsyxjhsgqgu";
        p270_uri_SET(&uri, 0,  sizeof(uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_WIFI_CONFIG_AP_299(), &PH);
    {
        char16_t   ssid = "hiBqjoicbxklosts";
        p299_ssid_SET(&ssid, 0,  sizeof(ssid), &PH) ;
    }
    {
        char16_t   password = "cfjanbtcEilcQdpaumwgxwfvgha";
        p299_password_SET(&password, 0,  sizeof(password), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PROTOCOL_VERSION_300(), &PH);
    p300_version_SET((uint16_t)(uint16_t)64348, PH.base.pack) ;
    p300_min_version_SET((uint16_t)(uint16_t)31584, PH.base.pack) ;
    p300_max_version_SET((uint16_t)(uint16_t)50043, PH.base.pack) ;
    {
        uint8_t  spec_version_hash [] =  {(uint8_t)135, (uint8_t)105, (uint8_t)173, (uint8_t)130, (uint8_t)239, (uint8_t)187, (uint8_t)104, (uint8_t)22};
        p300_spec_version_hash_SET(&spec_version_hash, 0, &PH.base.pack) ;
    }
    {
        uint8_t  library_version_hash [] =  {(uint8_t)34, (uint8_t)88, (uint8_t)168, (uint8_t)186, (uint8_t)1, (uint8_t)219, (uint8_t)205, (uint8_t)236};
        p300_library_version_hash_SET(&library_version_hash, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_STATUS_310(), &PH);
    p310_time_usec_SET((uint64_t)5404244242087147541L, PH.base.pack) ;
    p310_uptime_sec_SET((uint32_t)138596452L, PH.base.pack) ;
    p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_WARNING, PH.base.pack) ;
    p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_MAINTENANCE, PH.base.pack) ;
    p310_sub_mode_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
    p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)43939, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_INFO_311(), &PH);
    p311_time_usec_SET((uint64_t)5739589054982104222L, PH.base.pack) ;
    p311_uptime_sec_SET((uint32_t)700758550L, PH.base.pack) ;
    {
        char16_t   name = "yevhpuqti";
        p311_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p311_hw_version_major_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
    p311_hw_version_minor_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
    {
        uint8_t  hw_unique_id [] =  {(uint8_t)231, (uint8_t)80, (uint8_t)111, (uint8_t)22, (uint8_t)157, (uint8_t)246, (uint8_t)3, (uint8_t)238, (uint8_t)129, (uint8_t)196, (uint8_t)231, (uint8_t)70, (uint8_t)46, (uint8_t)237, (uint8_t)19, (uint8_t)151};
        p311_hw_unique_id_SET(&hw_unique_id, 0, &PH.base.pack) ;
    }
    p311_sw_version_major_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
    p311_sw_version_minor_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
    p311_sw_vcs_commit_SET((uint32_t)2187428167L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
    p320_target_system_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
    p320_target_component_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
    {
        char16_t   param_id = "ntemvzdemMriU";
        p320_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p320_param_index_SET((int16_t)(int16_t)29546, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
    p321_target_system_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
    p321_target_component_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_VALUE_322(), &PH);
    {
        char16_t   param_id = "bxm";
        p322_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "ifocubpitmtxqceam";
        p322_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64, PH.base.pack) ;
    p322_param_count_SET((uint16_t)(uint16_t)49499, PH.base.pack) ;
    p322_param_index_SET((uint16_t)(uint16_t)61392, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_SET_323(), &PH);
    p323_target_system_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
    p323_target_component_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
    {
        char16_t   param_id = "yvoieeQlnss";
        p323_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "vvgxzb";
        p323_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_ACK_324(), &PH);
    {
        char16_t   param_id = "eyigdqnFo";
        p324_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "lclbJvzgrvfxnwhigpWcjxffrbbyoaydqEmwcxvgmccpoodnjhdlzvuIuysjdnydhjkqFezwtxcwjdyjamuddmrBqpxrfmeculqhnMwomlmgzesrx";
        p324_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16, PH.base.pack) ;
    p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_IN_PROGRESS, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OBSTACLE_DISTANCE_330(), &PH);
    p330_time_usec_SET((uint64_t)357441087477099672L, PH.base.pack) ;
    p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, PH.base.pack) ;
    {
        uint16_t  distances [] =  {(uint16_t)2563, (uint16_t)7011, (uint16_t)27671, (uint16_t)24063, (uint16_t)42632, (uint16_t)65464, (uint16_t)38714, (uint16_t)2939, (uint16_t)59863, (uint16_t)24137, (uint16_t)41537, (uint16_t)19810, (uint16_t)1041, (uint16_t)16621, (uint16_t)64826, (uint16_t)160, (uint16_t)30447, (uint16_t)50510, (uint16_t)28445, (uint16_t)12334, (uint16_t)34688, (uint16_t)64659, (uint16_t)8924, (uint16_t)18939, (uint16_t)24225, (uint16_t)26072, (uint16_t)64389, (uint16_t)63367, (uint16_t)53506, (uint16_t)20200, (uint16_t)63732, (uint16_t)5005, (uint16_t)28670, (uint16_t)42535, (uint16_t)10911, (uint16_t)37325, (uint16_t)13124, (uint16_t)49846, (uint16_t)18733, (uint16_t)13389, (uint16_t)8438, (uint16_t)29787, (uint16_t)62571, (uint16_t)6889, (uint16_t)51655, (uint16_t)12634, (uint16_t)34232, (uint16_t)21490, (uint16_t)42046, (uint16_t)18475, (uint16_t)5334, (uint16_t)9498, (uint16_t)63462, (uint16_t)24462, (uint16_t)13900, (uint16_t)4144, (uint16_t)35867, (uint16_t)30378, (uint16_t)51285, (uint16_t)42439, (uint16_t)39958, (uint16_t)26648, (uint16_t)27852, (uint16_t)9045, (uint16_t)22030, (uint16_t)8675, (uint16_t)4235, (uint16_t)24802, (uint16_t)11693, (uint16_t)65224, (uint16_t)53354, (uint16_t)56100};
        p330_distances_SET(&distances, 0, &PH.base.pack) ;
    }
    p330_increment_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
    p330_min_distance_SET((uint16_t)(uint16_t)63984, PH.base.pack) ;
    p330_max_distance_SET((uint16_t)(uint16_t)34535, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}
