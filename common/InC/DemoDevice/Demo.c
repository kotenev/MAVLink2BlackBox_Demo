
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
    p0_type_SET(e_MAV_TYPE_MAV_TYPE_HEXAROTOR, PH.base.pack) ;
    p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_AEROB, PH.base.pack) ;
    p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED, PH.base.pack) ;
    p0_custom_mode_SET((uint32_t)2708377792L, PH.base.pack) ;
    p0_system_status_SET(e_MAV_STATE_MAV_STATE_CRITICAL, PH.base.pack) ;
    p0_mavlink_version_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SYS_STATUS_1(), &PH);
    p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS, PH.base.pack) ;
    p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING, PH.base.pack) ;
    p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO, PH.base.pack) ;
    p1_load_SET((uint16_t)(uint16_t)57954, PH.base.pack) ;
    p1_voltage_battery_SET((uint16_t)(uint16_t)15623, PH.base.pack) ;
    p1_current_battery_SET((int16_t)(int16_t)13813, PH.base.pack) ;
    p1_battery_remaining_SET((int8_t)(int8_t) -108, PH.base.pack) ;
    p1_drop_rate_comm_SET((uint16_t)(uint16_t)64110, PH.base.pack) ;
    p1_errors_comm_SET((uint16_t)(uint16_t)63580, PH.base.pack) ;
    p1_errors_count1_SET((uint16_t)(uint16_t)44512, PH.base.pack) ;
    p1_errors_count2_SET((uint16_t)(uint16_t)2666, PH.base.pack) ;
    p1_errors_count3_SET((uint16_t)(uint16_t)26523, PH.base.pack) ;
    p1_errors_count4_SET((uint16_t)(uint16_t)64816, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SYSTEM_TIME_2(), &PH);
    p2_time_unix_usec_SET((uint64_t)2986197993783025711L, PH.base.pack) ;
    p2_time_boot_ms_SET((uint32_t)560638846L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
    p3_time_boot_ms_SET((uint32_t)688792525L, PH.base.pack) ;
    p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
    p3_type_mask_SET((uint16_t)(uint16_t)47224, PH.base.pack) ;
    p3_x_SET((float)1.7375257E38F, PH.base.pack) ;
    p3_y_SET((float)3.1451868E38F, PH.base.pack) ;
    p3_z_SET((float) -2.5251995E38F, PH.base.pack) ;
    p3_vx_SET((float) -2.0379473E38F, PH.base.pack) ;
    p3_vy_SET((float)2.5062505E38F, PH.base.pack) ;
    p3_vz_SET((float) -1.4240298E38F, PH.base.pack) ;
    p3_afx_SET((float) -1.215842E38F, PH.base.pack) ;
    p3_afy_SET((float)4.958427E37F, PH.base.pack) ;
    p3_afz_SET((float)2.3336407E38F, PH.base.pack) ;
    p3_yaw_SET((float) -1.6116489E38F, PH.base.pack) ;
    p3_yaw_rate_SET((float) -1.8844109E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PING_4(), &PH);
    p4_time_usec_SET((uint64_t)6287906808642754506L, PH.base.pack) ;
    p4_seq_SET((uint32_t)2454130560L, PH.base.pack) ;
    p4_target_system_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
    p4_target_component_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
    p5_target_system_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
    p5_control_request_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
    p5_version_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    {
        char16_t   passkey = "qz";
        p5_passkey_SET(&passkey, 0,  sizeof(passkey), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
    p6_gcs_system_id_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    p6_control_request_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
    p6_ack_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AUTH_KEY_7(), &PH);
    {
        char16_t   key = "ohuqqrzounrbzdebfbpoIuzjcmiicsR";
        p7_key_SET(&key, 0,  sizeof(key), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_MODE_11(), &PH);
    p11_target_system_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
    p11_base_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_DISARMED, PH.base.pack) ;
    p11_custom_mode_SET((uint32_t)4241302500L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_READ_20(), &PH);
    p20_target_system_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
    p20_target_component_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    {
        char16_t   param_id = "iMhxgxx";
        p20_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p20_param_index_SET((int16_t)(int16_t) -17783, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_LIST_21(), &PH);
    p21_target_system_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
    p21_target_component_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_VALUE_22(), &PH);
    {
        char16_t   param_id = "ekzcorr";
        p22_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p22_param_value_SET((float) -2.4082512E38F, PH.base.pack) ;
    p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT8, PH.base.pack) ;
    p22_param_count_SET((uint16_t)(uint16_t)5514, PH.base.pack) ;
    p22_param_index_SET((uint16_t)(uint16_t)10981, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_SET_23(), &PH);
    p23_target_system_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
    p23_target_component_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
    {
        char16_t   param_id = "cwkwem";
        p23_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p23_param_value_SET((float)2.6534479E38F, PH.base.pack) ;
    p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RAW_INT_24(), &PH);
    p24_time_usec_SET((uint64_t)3669618934475876969L, PH.base.pack) ;
    p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX, PH.base.pack) ;
    p24_lat_SET((int32_t) -962688067, PH.base.pack) ;
    p24_lon_SET((int32_t) -800098816, PH.base.pack) ;
    p24_alt_SET((int32_t) -516805736, PH.base.pack) ;
    p24_eph_SET((uint16_t)(uint16_t)59429, PH.base.pack) ;
    p24_epv_SET((uint16_t)(uint16_t)8490, PH.base.pack) ;
    p24_vel_SET((uint16_t)(uint16_t)64, PH.base.pack) ;
    p24_cog_SET((uint16_t)(uint16_t)42134, PH.base.pack) ;
    p24_satellites_visible_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
    p24_alt_ellipsoid_SET((int32_t)1249491804, &PH) ;
    p24_h_acc_SET((uint32_t)1528221406L, &PH) ;
    p24_v_acc_SET((uint32_t)3074312283L, &PH) ;
    p24_vel_acc_SET((uint32_t)885539647L, &PH) ;
    p24_hdg_acc_SET((uint32_t)582523175L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_STATUS_25(), &PH);
    p25_satellites_visible_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
    {
        uint8_t  satellite_prn [] =  {(uint8_t)65, (uint8_t)176, (uint8_t)175, (uint8_t)96, (uint8_t)10, (uint8_t)66, (uint8_t)194, (uint8_t)118, (uint8_t)117, (uint8_t)88, (uint8_t)1, (uint8_t)134, (uint8_t)121, (uint8_t)72, (uint8_t)145, (uint8_t)77, (uint8_t)69, (uint8_t)52, (uint8_t)139, (uint8_t)234};
        p25_satellite_prn_SET(&satellite_prn, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_used [] =  {(uint8_t)6, (uint8_t)200, (uint8_t)136, (uint8_t)3, (uint8_t)42, (uint8_t)194, (uint8_t)195, (uint8_t)95, (uint8_t)158, (uint8_t)0, (uint8_t)68, (uint8_t)111, (uint8_t)123, (uint8_t)247, (uint8_t)1, (uint8_t)29, (uint8_t)19, (uint8_t)159, (uint8_t)145, (uint8_t)115};
        p25_satellite_used_SET(&satellite_used, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_elevation [] =  {(uint8_t)220, (uint8_t)47, (uint8_t)63, (uint8_t)230, (uint8_t)57, (uint8_t)124, (uint8_t)116, (uint8_t)141, (uint8_t)54, (uint8_t)198, (uint8_t)167, (uint8_t)198, (uint8_t)49, (uint8_t)10, (uint8_t)81, (uint8_t)102, (uint8_t)138, (uint8_t)241, (uint8_t)76, (uint8_t)164};
        p25_satellite_elevation_SET(&satellite_elevation, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_azimuth [] =  {(uint8_t)184, (uint8_t)124, (uint8_t)238, (uint8_t)87, (uint8_t)127, (uint8_t)2, (uint8_t)107, (uint8_t)5, (uint8_t)142, (uint8_t)178, (uint8_t)42, (uint8_t)104, (uint8_t)209, (uint8_t)14, (uint8_t)125, (uint8_t)14, (uint8_t)230, (uint8_t)179, (uint8_t)72, (uint8_t)213};
        p25_satellite_azimuth_SET(&satellite_azimuth, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_snr [] =  {(uint8_t)239, (uint8_t)118, (uint8_t)144, (uint8_t)229, (uint8_t)159, (uint8_t)214, (uint8_t)176, (uint8_t)250, (uint8_t)250, (uint8_t)129, (uint8_t)77, (uint8_t)235, (uint8_t)67, (uint8_t)49, (uint8_t)222, (uint8_t)92, (uint8_t)56, (uint8_t)153, (uint8_t)226, (uint8_t)175};
        p25_satellite_snr_SET(&satellite_snr, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU_26(), &PH);
    p26_time_boot_ms_SET((uint32_t)224191044L, PH.base.pack) ;
    p26_xacc_SET((int16_t)(int16_t) -29103, PH.base.pack) ;
    p26_yacc_SET((int16_t)(int16_t)2849, PH.base.pack) ;
    p26_zacc_SET((int16_t)(int16_t)17000, PH.base.pack) ;
    p26_xgyro_SET((int16_t)(int16_t)27894, PH.base.pack) ;
    p26_ygyro_SET((int16_t)(int16_t) -17220, PH.base.pack) ;
    p26_zgyro_SET((int16_t)(int16_t) -6170, PH.base.pack) ;
    p26_xmag_SET((int16_t)(int16_t)11873, PH.base.pack) ;
    p26_ymag_SET((int16_t)(int16_t) -32142, PH.base.pack) ;
    p26_zmag_SET((int16_t)(int16_t) -7793, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RAW_IMU_27(), &PH);
    p27_time_usec_SET((uint64_t)4413492389280680503L, PH.base.pack) ;
    p27_xacc_SET((int16_t)(int16_t)21906, PH.base.pack) ;
    p27_yacc_SET((int16_t)(int16_t) -23162, PH.base.pack) ;
    p27_zacc_SET((int16_t)(int16_t)2341, PH.base.pack) ;
    p27_xgyro_SET((int16_t)(int16_t)6213, PH.base.pack) ;
    p27_ygyro_SET((int16_t)(int16_t) -14628, PH.base.pack) ;
    p27_zgyro_SET((int16_t)(int16_t) -23931, PH.base.pack) ;
    p27_xmag_SET((int16_t)(int16_t)13245, PH.base.pack) ;
    p27_ymag_SET((int16_t)(int16_t) -22244, PH.base.pack) ;
    p27_zmag_SET((int16_t)(int16_t)8036, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RAW_PRESSURE_28(), &PH);
    p28_time_usec_SET((uint64_t)2608542070795276278L, PH.base.pack) ;
    p28_press_abs_SET((int16_t)(int16_t) -2491, PH.base.pack) ;
    p28_press_diff1_SET((int16_t)(int16_t) -17169, PH.base.pack) ;
    p28_press_diff2_SET((int16_t)(int16_t)18959, PH.base.pack) ;
    p28_temperature_SET((int16_t)(int16_t) -5475, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE_29(), &PH);
    p29_time_boot_ms_SET((uint32_t)161793265L, PH.base.pack) ;
    p29_press_abs_SET((float)3.000979E38F, PH.base.pack) ;
    p29_press_diff_SET((float) -1.781383E38F, PH.base.pack) ;
    p29_temperature_SET((int16_t)(int16_t)32654, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_30(), &PH);
    p30_time_boot_ms_SET((uint32_t)2723323454L, PH.base.pack) ;
    p30_roll_SET((float) -1.8592546E38F, PH.base.pack) ;
    p30_pitch_SET((float) -4.0848928E37F, PH.base.pack) ;
    p30_yaw_SET((float) -3.4007307E38F, PH.base.pack) ;
    p30_rollspeed_SET((float) -4.795022E37F, PH.base.pack) ;
    p30_pitchspeed_SET((float)1.4679422E38F, PH.base.pack) ;
    p30_yawspeed_SET((float)1.2704157E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_31(), &PH);
    p31_time_boot_ms_SET((uint32_t)3976674062L, PH.base.pack) ;
    p31_q1_SET((float)1.7977117E38F, PH.base.pack) ;
    p31_q2_SET((float) -1.878372E38F, PH.base.pack) ;
    p31_q3_SET((float)3.157534E38F, PH.base.pack) ;
    p31_q4_SET((float) -2.816562E38F, PH.base.pack) ;
    p31_rollspeed_SET((float) -5.828677E36F, PH.base.pack) ;
    p31_pitchspeed_SET((float) -4.369364E37F, PH.base.pack) ;
    p31_yawspeed_SET((float)1.1075929E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_32(), &PH);
    p32_time_boot_ms_SET((uint32_t)3753666744L, PH.base.pack) ;
    p32_x_SET((float)2.9457344E38F, PH.base.pack) ;
    p32_y_SET((float) -1.6124218E38F, PH.base.pack) ;
    p32_z_SET((float)1.9016274E38F, PH.base.pack) ;
    p32_vx_SET((float)2.0818622E38F, PH.base.pack) ;
    p32_vy_SET((float)2.3945829E38F, PH.base.pack) ;
    p32_vz_SET((float)1.9494896E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_33(), &PH);
    p33_time_boot_ms_SET((uint32_t)3775918930L, PH.base.pack) ;
    p33_lat_SET((int32_t)952798822, PH.base.pack) ;
    p33_lon_SET((int32_t) -1549594334, PH.base.pack) ;
    p33_alt_SET((int32_t)54442823, PH.base.pack) ;
    p33_relative_alt_SET((int32_t)1974609517, PH.base.pack) ;
    p33_vx_SET((int16_t)(int16_t) -31046, PH.base.pack) ;
    p33_vy_SET((int16_t)(int16_t) -26228, PH.base.pack) ;
    p33_vz_SET((int16_t)(int16_t)21378, PH.base.pack) ;
    p33_hdg_SET((uint16_t)(uint16_t)22224, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_SCALED_34(), &PH);
    p34_time_boot_ms_SET((uint32_t)656215525L, PH.base.pack) ;
    p34_port_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
    p34_chan1_scaled_SET((int16_t)(int16_t) -26909, PH.base.pack) ;
    p34_chan2_scaled_SET((int16_t)(int16_t) -31650, PH.base.pack) ;
    p34_chan3_scaled_SET((int16_t)(int16_t)350, PH.base.pack) ;
    p34_chan4_scaled_SET((int16_t)(int16_t) -32186, PH.base.pack) ;
    p34_chan5_scaled_SET((int16_t)(int16_t) -4903, PH.base.pack) ;
    p34_chan6_scaled_SET((int16_t)(int16_t)11841, PH.base.pack) ;
    p34_chan7_scaled_SET((int16_t)(int16_t) -592, PH.base.pack) ;
    p34_chan8_scaled_SET((int16_t)(int16_t)928, PH.base.pack) ;
    p34_rssi_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_RAW_35(), &PH);
    p35_time_boot_ms_SET((uint32_t)3233372473L, PH.base.pack) ;
    p35_port_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
    p35_chan1_raw_SET((uint16_t)(uint16_t)10923, PH.base.pack) ;
    p35_chan2_raw_SET((uint16_t)(uint16_t)24754, PH.base.pack) ;
    p35_chan3_raw_SET((uint16_t)(uint16_t)56287, PH.base.pack) ;
    p35_chan4_raw_SET((uint16_t)(uint16_t)9330, PH.base.pack) ;
    p35_chan5_raw_SET((uint16_t)(uint16_t)45286, PH.base.pack) ;
    p35_chan6_raw_SET((uint16_t)(uint16_t)31049, PH.base.pack) ;
    p35_chan7_raw_SET((uint16_t)(uint16_t)35436, PH.base.pack) ;
    p35_chan8_raw_SET((uint16_t)(uint16_t)35837, PH.base.pack) ;
    p35_rssi_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
    p36_time_usec_SET((uint32_t)3091741830L, PH.base.pack) ;
    p36_port_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
    p36_servo1_raw_SET((uint16_t)(uint16_t)11631, PH.base.pack) ;
    p36_servo2_raw_SET((uint16_t)(uint16_t)39862, PH.base.pack) ;
    p36_servo3_raw_SET((uint16_t)(uint16_t)29644, PH.base.pack) ;
    p36_servo4_raw_SET((uint16_t)(uint16_t)35251, PH.base.pack) ;
    p36_servo5_raw_SET((uint16_t)(uint16_t)63589, PH.base.pack) ;
    p36_servo6_raw_SET((uint16_t)(uint16_t)11422, PH.base.pack) ;
    p36_servo7_raw_SET((uint16_t)(uint16_t)28374, PH.base.pack) ;
    p36_servo8_raw_SET((uint16_t)(uint16_t)54317, PH.base.pack) ;
    p36_servo9_raw_SET((uint16_t)(uint16_t)30777, &PH) ;
    p36_servo10_raw_SET((uint16_t)(uint16_t)29188, &PH) ;
    p36_servo11_raw_SET((uint16_t)(uint16_t)25373, &PH) ;
    p36_servo12_raw_SET((uint16_t)(uint16_t)48839, &PH) ;
    p36_servo13_raw_SET((uint16_t)(uint16_t)14892, &PH) ;
    p36_servo14_raw_SET((uint16_t)(uint16_t)14732, &PH) ;
    p36_servo15_raw_SET((uint16_t)(uint16_t)43157, &PH) ;
    p36_servo16_raw_SET((uint16_t)(uint16_t)30161, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
    p37_target_system_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
    p37_target_component_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
    p37_start_index_SET((int16_t)(int16_t) -29617, PH.base.pack) ;
    p37_end_index_SET((int16_t)(int16_t)3308, PH.base.pack) ;
    p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
    p38_target_system_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
    p38_target_component_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
    p38_start_index_SET((int16_t)(int16_t)6682, PH.base.pack) ;
    p38_end_index_SET((int16_t)(int16_t) -28094, PH.base.pack) ;
    p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_39(), &PH);
    p39_target_system_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
    p39_target_component_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
    p39_seq_SET((uint16_t)(uint16_t)50757, PH.base.pack) ;
    p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
    p39_command_SET(e_MAV_CMD_MAV_CMD_DO_PARACHUTE, PH.base.pack) ;
    p39_current_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
    p39_autocontinue_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
    p39_param1_SET((float)2.9407587E38F, PH.base.pack) ;
    p39_param2_SET((float) -3.1979413E38F, PH.base.pack) ;
    p39_param3_SET((float) -4.950071E37F, PH.base.pack) ;
    p39_param4_SET((float)1.8841667E38F, PH.base.pack) ;
    p39_x_SET((float) -1.766403E37F, PH.base.pack) ;
    p39_y_SET((float) -2.1704253E38F, PH.base.pack) ;
    p39_z_SET((float) -9.784313E37F, PH.base.pack) ;
    p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_40(), &PH);
    p40_target_system_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
    p40_target_component_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
    p40_seq_SET((uint16_t)(uint16_t)58582, PH.base.pack) ;
    p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_SET_CURRENT_41(), &PH);
    p41_target_system_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
    p41_target_component_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
    p41_seq_SET((uint16_t)(uint16_t)57454, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_CURRENT_42(), &PH);
    p42_seq_SET((uint16_t)(uint16_t)63037, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_LIST_43(), &PH);
    p43_target_system_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
    p43_target_component_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
    p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_COUNT_44(), &PH);
    p44_target_system_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
    p44_target_component_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
    p44_count_SET((uint16_t)(uint16_t)35328, PH.base.pack) ;
    p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_CLEAR_ALL_45(), &PH);
    p45_target_system_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
    p45_target_component_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
    p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_REACHED_46(), &PH);
    p46_seq_SET((uint16_t)(uint16_t)6556, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ACK_47(), &PH);
    p47_target_system_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    p47_target_component_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_ERROR, PH.base.pack) ;
    p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
    p48_target_system_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
    p48_latitude_SET((int32_t) -1619063024, PH.base.pack) ;
    p48_longitude_SET((int32_t)1334001227, PH.base.pack) ;
    p48_altitude_SET((int32_t)1178432466, PH.base.pack) ;
    p48_time_usec_SET((uint64_t)8939030000092306906L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
    p49_latitude_SET((int32_t)504212209, PH.base.pack) ;
    p49_longitude_SET((int32_t) -1727238861, PH.base.pack) ;
    p49_altitude_SET((int32_t) -1678863488, PH.base.pack) ;
    p49_time_usec_SET((uint64_t)4795564419836156732L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_MAP_RC_50(), &PH);
    p50_target_system_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
    p50_target_component_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
    {
        char16_t   param_id = "roviiyii";
        p50_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p50_param_index_SET((int16_t)(int16_t) -14339, PH.base.pack) ;
    p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
    p50_param_value0_SET((float) -3.3083253E38F, PH.base.pack) ;
    p50_scale_SET((float) -1.0091819E38F, PH.base.pack) ;
    p50_param_value_min_SET((float) -3.1888522E38F, PH.base.pack) ;
    p50_param_value_max_SET((float) -3.554647E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_INT_51(), &PH);
    p51_target_system_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
    p51_target_component_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    p51_seq_SET((uint16_t)(uint16_t)22066, PH.base.pack) ;
    p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
    p54_target_system_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
    p54_target_component_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
    p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
    p54_p1x_SET((float) -3.2052316E38F, PH.base.pack) ;
    p54_p1y_SET((float)2.2900887E38F, PH.base.pack) ;
    p54_p1z_SET((float) -1.6788292E38F, PH.base.pack) ;
    p54_p2x_SET((float)1.2143122E37F, PH.base.pack) ;
    p54_p2y_SET((float) -3.2035894E38F, PH.base.pack) ;
    p54_p2z_SET((float)1.8457265E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
    p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
    p55_p1x_SET((float) -3.8994647E36F, PH.base.pack) ;
    p55_p1y_SET((float)2.8033876E38F, PH.base.pack) ;
    p55_p1z_SET((float)1.5142516E38F, PH.base.pack) ;
    p55_p2x_SET((float) -8.083445E37F, PH.base.pack) ;
    p55_p2y_SET((float) -3.1452089E38F, PH.base.pack) ;
    p55_p2z_SET((float)8.61808E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
    p61_time_usec_SET((uint64_t)1231042011117387285L, PH.base.pack) ;
    {
        float  q [] =  {-1.1530489E38F, 1.4097726E38F, 1.0293577E38F, -2.8233033E38F};
        p61_q_SET(&q, 0, &PH.base.pack) ;
    }
    p61_rollspeed_SET((float) -4.5813734E37F, PH.base.pack) ;
    p61_pitchspeed_SET((float)1.3227773E38F, PH.base.pack) ;
    p61_yawspeed_SET((float) -2.7503252E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-1.0610147E38F, -1.5128245E38F, 2.6228267E38F, 1.1884203E38F, -2.5004875E38F, 4.283149E37F, 3.1420873E37F, 1.3810599E38F, 2.1486672E38F};
        p61_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
    p62_nav_roll_SET((float) -3.201516E38F, PH.base.pack) ;
    p62_nav_pitch_SET((float)1.7676933E38F, PH.base.pack) ;
    p62_nav_bearing_SET((int16_t)(int16_t)16420, PH.base.pack) ;
    p62_target_bearing_SET((int16_t)(int16_t)24049, PH.base.pack) ;
    p62_wp_dist_SET((uint16_t)(uint16_t)42405, PH.base.pack) ;
    p62_alt_error_SET((float)2.596962E38F, PH.base.pack) ;
    p62_aspd_error_SET((float)1.824591E38F, PH.base.pack) ;
    p62_xtrack_error_SET((float)1.3032027E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
    p63_time_usec_SET((uint64_t)1417023411436858501L, PH.base.pack) ;
    p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE, PH.base.pack) ;
    p63_lat_SET((int32_t)133859816, PH.base.pack) ;
    p63_lon_SET((int32_t) -661715545, PH.base.pack) ;
    p63_alt_SET((int32_t)490934398, PH.base.pack) ;
    p63_relative_alt_SET((int32_t)1424124569, PH.base.pack) ;
    p63_vx_SET((float) -2.2001723E38F, PH.base.pack) ;
    p63_vy_SET((float)1.0723031E38F, PH.base.pack) ;
    p63_vz_SET((float) -2.3555915E38F, PH.base.pack) ;
    {
        float  covariance [] =  {1.1802306E38F, 3.098889E38F, 4.144383E37F, -6.2322404E37F, 3.3915896E38F, -1.3909427E38F, 3.249623E38F, -2.4193657E38F, -6.593044E37F, -9.403297E37F, 1.1264255E38F, 1.0986961E38F, -2.8621068E37F, -1.579839E38F, -1.0517232E38F, -3.3010435E38F, -1.1866508E38F, -2.278325E38F, -3.3744877E38F, -2.9320637E38F, 3.1430529E38F, -1.718996E38F, 3.0650642E38F, -4.3784763E37F, 1.6718139E38F, -1.9754452E38F, 5.690008E37F, -5.9825447E37F, 2.554337E38F, 3.0170573E38F, -2.1725856E38F, -3.102354E38F, 3.2196015E36F, 1.2263852E38F, 2.1783488E38F, 1.3322595E37F};
        p63_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
    p64_time_usec_SET((uint64_t)8647295822565057024L, PH.base.pack) ;
    p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, PH.base.pack) ;
    p64_x_SET((float)1.9788415E38F, PH.base.pack) ;
    p64_y_SET((float)1.0396012E38F, PH.base.pack) ;
    p64_z_SET((float) -2.6140045E38F, PH.base.pack) ;
    p64_vx_SET((float) -2.8150096E38F, PH.base.pack) ;
    p64_vy_SET((float) -2.9694107E38F, PH.base.pack) ;
    p64_vz_SET((float)1.6941994E38F, PH.base.pack) ;
    p64_ax_SET((float) -1.69539E38F, PH.base.pack) ;
    p64_ay_SET((float) -5.842234E37F, PH.base.pack) ;
    p64_az_SET((float) -2.5386468E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-2.6517052E38F, -4.554355E37F, 3.164732E38F, -1.7515672E38F, -2.1472346E38F, -3.0907043E37F, 1.5955604E38F, 2.6918033E38F, 1.3155407E37F, 2.1703551E38F, 8.92403E37F, 1.4566889E38F, -1.3798724E38F, 2.5210957E37F, -1.6347997E38F, 2.9992206E38F, 2.020377E38F, -6.1275137E37F, -1.6272286E38F, 3.0498793E38F, -3.3013068E38F, 2.2708754E38F, -6.217923E37F, 1.3892638E38F, -2.1885913E38F, -1.4650042E38F, -9.225426E37F, 3.0711667E38F, 3.4338251E37F, -8.1977453E37F, -1.6843318E37F, 1.5801081E37F, 2.1575352E38F, 1.6967376E38F, -1.011008E38F, 1.6115343E38F, 1.0455575E38F, -1.4230901E38F, 1.0132272E38F, 8.5900575E36F, 3.3117615E38F, -5.5774735E37F, -1.0944068E38F, -1.8297648E38F, 1.3614768E38F};
        p64_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_65(), &PH);
    p65_time_boot_ms_SET((uint32_t)1215265497L, PH.base.pack) ;
    p65_chancount_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
    p65_chan1_raw_SET((uint16_t)(uint16_t)50002, PH.base.pack) ;
    p65_chan2_raw_SET((uint16_t)(uint16_t)38023, PH.base.pack) ;
    p65_chan3_raw_SET((uint16_t)(uint16_t)16267, PH.base.pack) ;
    p65_chan4_raw_SET((uint16_t)(uint16_t)27087, PH.base.pack) ;
    p65_chan5_raw_SET((uint16_t)(uint16_t)34012, PH.base.pack) ;
    p65_chan6_raw_SET((uint16_t)(uint16_t)775, PH.base.pack) ;
    p65_chan7_raw_SET((uint16_t)(uint16_t)34164, PH.base.pack) ;
    p65_chan8_raw_SET((uint16_t)(uint16_t)40132, PH.base.pack) ;
    p65_chan9_raw_SET((uint16_t)(uint16_t)3581, PH.base.pack) ;
    p65_chan10_raw_SET((uint16_t)(uint16_t)9800, PH.base.pack) ;
    p65_chan11_raw_SET((uint16_t)(uint16_t)48313, PH.base.pack) ;
    p65_chan12_raw_SET((uint16_t)(uint16_t)14487, PH.base.pack) ;
    p65_chan13_raw_SET((uint16_t)(uint16_t)31070, PH.base.pack) ;
    p65_chan14_raw_SET((uint16_t)(uint16_t)30027, PH.base.pack) ;
    p65_chan15_raw_SET((uint16_t)(uint16_t)59594, PH.base.pack) ;
    p65_chan16_raw_SET((uint16_t)(uint16_t)30409, PH.base.pack) ;
    p65_chan17_raw_SET((uint16_t)(uint16_t)13059, PH.base.pack) ;
    p65_chan18_raw_SET((uint16_t)(uint16_t)61513, PH.base.pack) ;
    p65_rssi_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_REQUEST_DATA_STREAM_66(), &PH);
    p66_target_system_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
    p66_target_component_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
    p66_req_stream_id_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
    p66_req_message_rate_SET((uint16_t)(uint16_t)14817, PH.base.pack) ;
    p66_start_stop_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA_STREAM_67(), &PH);
    p67_stream_id_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
    p67_message_rate_SET((uint16_t)(uint16_t)2140, PH.base.pack) ;
    p67_on_off_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MANUAL_CONTROL_69(), &PH);
    p69_target_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
    p69_x_SET((int16_t)(int16_t) -14094, PH.base.pack) ;
    p69_y_SET((int16_t)(int16_t) -8908, PH.base.pack) ;
    p69_z_SET((int16_t)(int16_t)14313, PH.base.pack) ;
    p69_r_SET((int16_t)(int16_t) -13727, PH.base.pack) ;
    p69_buttons_SET((uint16_t)(uint16_t)41325, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
    p70_target_system_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
    p70_target_component_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
    p70_chan1_raw_SET((uint16_t)(uint16_t)3092, PH.base.pack) ;
    p70_chan2_raw_SET((uint16_t)(uint16_t)44775, PH.base.pack) ;
    p70_chan3_raw_SET((uint16_t)(uint16_t)28407, PH.base.pack) ;
    p70_chan4_raw_SET((uint16_t)(uint16_t)37569, PH.base.pack) ;
    p70_chan5_raw_SET((uint16_t)(uint16_t)36243, PH.base.pack) ;
    p70_chan6_raw_SET((uint16_t)(uint16_t)48269, PH.base.pack) ;
    p70_chan7_raw_SET((uint16_t)(uint16_t)58406, PH.base.pack) ;
    p70_chan8_raw_SET((uint16_t)(uint16_t)57075, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_INT_73(), &PH);
    p73_target_system_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    p73_target_component_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
    p73_seq_SET((uint16_t)(uint16_t)4722, PH.base.pack) ;
    p73_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
    p73_command_SET(e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION, PH.base.pack) ;
    p73_current_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
    p73_autocontinue_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
    p73_param1_SET((float)1.052856E38F, PH.base.pack) ;
    p73_param2_SET((float) -5.842199E37F, PH.base.pack) ;
    p73_param3_SET((float)2.2375057E38F, PH.base.pack) ;
    p73_param4_SET((float)1.3449649E38F, PH.base.pack) ;
    p73_x_SET((int32_t) -86567537, PH.base.pack) ;
    p73_y_SET((int32_t)337297938, PH.base.pack) ;
    p73_z_SET((float) -1.7038784E38F, PH.base.pack) ;
    p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VFR_HUD_74(), &PH);
    p74_airspeed_SET((float)2.6285465E38F, PH.base.pack) ;
    p74_groundspeed_SET((float)1.8125696E38F, PH.base.pack) ;
    p74_heading_SET((int16_t)(int16_t)4854, PH.base.pack) ;
    p74_throttle_SET((uint16_t)(uint16_t)36187, PH.base.pack) ;
    p74_alt_SET((float)2.5365117E38F, PH.base.pack) ;
    p74_climb_SET((float)2.866526E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_INT_75(), &PH);
    p75_target_system_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
    p75_target_component_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
    p75_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
    p75_command_SET(e_MAV_CMD_MAV_CMD_CONDITION_GATE, PH.base.pack) ;
    p75_current_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
    p75_autocontinue_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    p75_param1_SET((float)2.9030582E38F, PH.base.pack) ;
    p75_param2_SET((float)1.2642274E38F, PH.base.pack) ;
    p75_param3_SET((float) -2.8954056E38F, PH.base.pack) ;
    p75_param4_SET((float)3.3300157E38F, PH.base.pack) ;
    p75_x_SET((int32_t)324304853, PH.base.pack) ;
    p75_y_SET((int32_t)273532939, PH.base.pack) ;
    p75_z_SET((float)7.0541556E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_LONG_76(), &PH);
    p76_target_system_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
    p76_target_component_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
    p76_command_SET(e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE, PH.base.pack) ;
    p76_confirmation_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
    p76_param1_SET((float) -2.618954E38F, PH.base.pack) ;
    p76_param2_SET((float)1.5280231E38F, PH.base.pack) ;
    p76_param3_SET((float) -9.100563E37F, PH.base.pack) ;
    p76_param4_SET((float)2.9071327E38F, PH.base.pack) ;
    p76_param5_SET((float)3.1069536E38F, PH.base.pack) ;
    p76_param6_SET((float)3.4847728E37F, PH.base.pack) ;
    p76_param7_SET((float) -1.8607103E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_ACK_77(), &PH);
    p77_command_SET(e_MAV_CMD_MAV_CMD_DO_REPOSITION, PH.base.pack) ;
    p77_result_SET(e_MAV_RESULT_MAV_RESULT_TEMPORARILY_REJECTED, PH.base.pack) ;
    p77_progress_SET((uint8_t)(uint8_t)187, &PH) ;
    p77_result_param2_SET((int32_t) -337269061, &PH) ;
    p77_target_system_SET((uint8_t)(uint8_t)144, &PH) ;
    p77_target_component_SET((uint8_t)(uint8_t)232, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MANUAL_SETPOINT_81(), &PH);
    p81_time_boot_ms_SET((uint32_t)3675820482L, PH.base.pack) ;
    p81_roll_SET((float)2.7484401E38F, PH.base.pack) ;
    p81_pitch_SET((float)9.56145E37F, PH.base.pack) ;
    p81_yaw_SET((float)1.0800793E37F, PH.base.pack) ;
    p81_thrust_SET((float)2.1689901E38F, PH.base.pack) ;
    p81_mode_switch_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
    p81_manual_override_switch_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
    p82_time_boot_ms_SET((uint32_t)392865729L, PH.base.pack) ;
    p82_target_system_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    p82_target_component_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
    p82_type_mask_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
    {
        float  q [] =  {2.900335E38F, 8.482697E37F, 1.6682118E38F, -2.5632463E38F};
        p82_q_SET(&q, 0, &PH.base.pack) ;
    }
    p82_body_roll_rate_SET((float) -1.9965318E38F, PH.base.pack) ;
    p82_body_pitch_rate_SET((float)2.9852452E38F, PH.base.pack) ;
    p82_body_yaw_rate_SET((float) -1.8623183E38F, PH.base.pack) ;
    p82_thrust_SET((float)1.3723217E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_TARGET_83(), &PH);
    p83_time_boot_ms_SET((uint32_t)4022759282L, PH.base.pack) ;
    p83_type_mask_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
    {
        float  q [] =  {1.8181464E38F, -2.1109094E38F, 9.10195E37F, 2.1407254E37F};
        p83_q_SET(&q, 0, &PH.base.pack) ;
    }
    p83_body_roll_rate_SET((float) -2.299402E38F, PH.base.pack) ;
    p83_body_pitch_rate_SET((float)1.4362962E38F, PH.base.pack) ;
    p83_body_yaw_rate_SET((float) -3.217569E37F, PH.base.pack) ;
    p83_thrust_SET((float)1.9105636E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
    p84_time_boot_ms_SET((uint32_t)2631068196L, PH.base.pack) ;
    p84_target_system_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
    p84_target_component_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
    p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
    p84_type_mask_SET((uint16_t)(uint16_t)46168, PH.base.pack) ;
    p84_x_SET((float)2.47037E38F, PH.base.pack) ;
    p84_y_SET((float) -6.929038E37F, PH.base.pack) ;
    p84_z_SET((float)2.896314E38F, PH.base.pack) ;
    p84_vx_SET((float) -7.6707936E37F, PH.base.pack) ;
    p84_vy_SET((float) -9.296436E37F, PH.base.pack) ;
    p84_vz_SET((float)1.4956256E38F, PH.base.pack) ;
    p84_afx_SET((float)1.3270345E38F, PH.base.pack) ;
    p84_afy_SET((float)2.117331E38F, PH.base.pack) ;
    p84_afz_SET((float) -2.214145E38F, PH.base.pack) ;
    p84_yaw_SET((float) -6.4820543E37F, PH.base.pack) ;
    p84_yaw_rate_SET((float)3.2064512E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
    p86_time_boot_ms_SET((uint32_t)2045878186L, PH.base.pack) ;
    p86_target_system_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
    p86_target_component_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
    p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
    p86_type_mask_SET((uint16_t)(uint16_t)36169, PH.base.pack) ;
    p86_lat_int_SET((int32_t) -767869496, PH.base.pack) ;
    p86_lon_int_SET((int32_t) -545726356, PH.base.pack) ;
    p86_alt_SET((float)1.5442441E38F, PH.base.pack) ;
    p86_vx_SET((float) -2.1125397E38F, PH.base.pack) ;
    p86_vy_SET((float) -3.3424287E38F, PH.base.pack) ;
    p86_vz_SET((float) -1.1365366E38F, PH.base.pack) ;
    p86_afx_SET((float)1.840992E38F, PH.base.pack) ;
    p86_afy_SET((float) -1.4504441E38F, PH.base.pack) ;
    p86_afz_SET((float) -7.8964035E37F, PH.base.pack) ;
    p86_yaw_SET((float)2.5784973E38F, PH.base.pack) ;
    p86_yaw_rate_SET((float)2.9196642E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
    p87_time_boot_ms_SET((uint32_t)2460145408L, PH.base.pack) ;
    p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
    p87_type_mask_SET((uint16_t)(uint16_t)58087, PH.base.pack) ;
    p87_lat_int_SET((int32_t)308905932, PH.base.pack) ;
    p87_lon_int_SET((int32_t) -654954025, PH.base.pack) ;
    p87_alt_SET((float)3.193636E38F, PH.base.pack) ;
    p87_vx_SET((float)1.0747833E38F, PH.base.pack) ;
    p87_vy_SET((float) -3.176783E37F, PH.base.pack) ;
    p87_vz_SET((float) -7.033174E37F, PH.base.pack) ;
    p87_afx_SET((float)2.1560534E38F, PH.base.pack) ;
    p87_afy_SET((float) -3.3662967E38F, PH.base.pack) ;
    p87_afz_SET((float)3.055877E37F, PH.base.pack) ;
    p87_yaw_SET((float) -2.707645E38F, PH.base.pack) ;
    p87_yaw_rate_SET((float) -4.2092242E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
    p89_time_boot_ms_SET((uint32_t)3224722042L, PH.base.pack) ;
    p89_x_SET((float)1.5697716E38F, PH.base.pack) ;
    p89_y_SET((float)1.6907915E38F, PH.base.pack) ;
    p89_z_SET((float)1.089339E38F, PH.base.pack) ;
    p89_roll_SET((float)6.21082E36F, PH.base.pack) ;
    p89_pitch_SET((float) -1.4095638E38F, PH.base.pack) ;
    p89_yaw_SET((float)2.5857436E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_STATE_90(), &PH);
    p90_time_usec_SET((uint64_t)7317103516021063635L, PH.base.pack) ;
    p90_roll_SET((float)2.7954014E38F, PH.base.pack) ;
    p90_pitch_SET((float) -3.898519E37F, PH.base.pack) ;
    p90_yaw_SET((float) -9.99652E37F, PH.base.pack) ;
    p90_rollspeed_SET((float) -2.872123E38F, PH.base.pack) ;
    p90_pitchspeed_SET((float) -1.856355E38F, PH.base.pack) ;
    p90_yawspeed_SET((float)2.6723799E37F, PH.base.pack) ;
    p90_lat_SET((int32_t)1657619569, PH.base.pack) ;
    p90_lon_SET((int32_t) -128529994, PH.base.pack) ;
    p90_alt_SET((int32_t) -1127153017, PH.base.pack) ;
    p90_vx_SET((int16_t)(int16_t) -313, PH.base.pack) ;
    p90_vy_SET((int16_t)(int16_t) -2946, PH.base.pack) ;
    p90_vz_SET((int16_t)(int16_t) -26010, PH.base.pack) ;
    p90_xacc_SET((int16_t)(int16_t) -23434, PH.base.pack) ;
    p90_yacc_SET((int16_t)(int16_t)27294, PH.base.pack) ;
    p90_zacc_SET((int16_t)(int16_t)28296, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_CONTROLS_91(), &PH);
    p91_time_usec_SET((uint64_t)5825887119169200483L, PH.base.pack) ;
    p91_roll_ailerons_SET((float) -1.1098815E38F, PH.base.pack) ;
    p91_pitch_elevator_SET((float)1.4709866E38F, PH.base.pack) ;
    p91_yaw_rudder_SET((float) -2.8330622E38F, PH.base.pack) ;
    p91_throttle_SET((float)2.2426782E38F, PH.base.pack) ;
    p91_aux1_SET((float) -1.037011E37F, PH.base.pack) ;
    p91_aux2_SET((float) -1.034214E38F, PH.base.pack) ;
    p91_aux3_SET((float) -4.0049456E37F, PH.base.pack) ;
    p91_aux4_SET((float)2.2685865E38F, PH.base.pack) ;
    p91_mode_SET(e_MAV_MODE_MAV_MODE_TEST_DISARMED, PH.base.pack) ;
    p91_nav_mode_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
    p92_time_usec_SET((uint64_t)8299207767053624516L, PH.base.pack) ;
    p92_chan1_raw_SET((uint16_t)(uint16_t)41295, PH.base.pack) ;
    p92_chan2_raw_SET((uint16_t)(uint16_t)31359, PH.base.pack) ;
    p92_chan3_raw_SET((uint16_t)(uint16_t)26888, PH.base.pack) ;
    p92_chan4_raw_SET((uint16_t)(uint16_t)29670, PH.base.pack) ;
    p92_chan5_raw_SET((uint16_t)(uint16_t)63159, PH.base.pack) ;
    p92_chan6_raw_SET((uint16_t)(uint16_t)60158, PH.base.pack) ;
    p92_chan7_raw_SET((uint16_t)(uint16_t)22797, PH.base.pack) ;
    p92_chan8_raw_SET((uint16_t)(uint16_t)2674, PH.base.pack) ;
    p92_chan9_raw_SET((uint16_t)(uint16_t)25238, PH.base.pack) ;
    p92_chan10_raw_SET((uint16_t)(uint16_t)61037, PH.base.pack) ;
    p92_chan11_raw_SET((uint16_t)(uint16_t)6408, PH.base.pack) ;
    p92_chan12_raw_SET((uint16_t)(uint16_t)24873, PH.base.pack) ;
    p92_rssi_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
    p93_time_usec_SET((uint64_t)5829299534223357222L, PH.base.pack) ;
    {
        float  controls [] =  {-1.4654629E38F, -1.3515148E37F, -2.3468908E38F, 2.6516421E38F, 1.452659E38F, -2.0402232E38F, 5.494981E37F, -8.2643213E37F, 2.1383179E38F, -1.4314573E38F, -2.0639364E38F, -2.3700827E37F, 2.1634946E38F, 7.2131474E36F, -2.9993201E38F, 2.7182599E38F};
        p93_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    p93_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_ARMED, PH.base.pack) ;
    p93_flags_SET((uint64_t)3091966654087073860L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_100(), &PH);
    p100_time_usec_SET((uint64_t)1827115461861877270L, PH.base.pack) ;
    p100_sensor_id_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
    p100_flow_x_SET((int16_t)(int16_t) -19376, PH.base.pack) ;
    p100_flow_y_SET((int16_t)(int16_t)26503, PH.base.pack) ;
    p100_flow_comp_m_x_SET((float) -1.3304612E38F, PH.base.pack) ;
    p100_flow_comp_m_y_SET((float) -2.416492E38F, PH.base.pack) ;
    p100_quality_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
    p100_ground_distance_SET((float)4.611099E37F, PH.base.pack) ;
    p100_flow_rate_x_SET((float) -2.9700534E38F, &PH) ;
    p100_flow_rate_y_SET((float)2.7983052E37F, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
    p101_usec_SET((uint64_t)786935146692682065L, PH.base.pack) ;
    p101_x_SET((float) -1.1217073E38F, PH.base.pack) ;
    p101_y_SET((float) -1.3692651E38F, PH.base.pack) ;
    p101_z_SET((float) -2.9269115E38F, PH.base.pack) ;
    p101_roll_SET((float) -2.3345366E38F, PH.base.pack) ;
    p101_pitch_SET((float) -3.0212424E38F, PH.base.pack) ;
    p101_yaw_SET((float)2.767292E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
    p102_usec_SET((uint64_t)6583823871771814030L, PH.base.pack) ;
    p102_x_SET((float)7.1954135E37F, PH.base.pack) ;
    p102_y_SET((float)1.0416945E38F, PH.base.pack) ;
    p102_z_SET((float) -2.1342161E38F, PH.base.pack) ;
    p102_roll_SET((float) -9.964412E37F, PH.base.pack) ;
    p102_pitch_SET((float) -1.6921134E38F, PH.base.pack) ;
    p102_yaw_SET((float)8.879244E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
    p103_usec_SET((uint64_t)2489829453211487760L, PH.base.pack) ;
    p103_x_SET((float) -2.928226E38F, PH.base.pack) ;
    p103_y_SET((float) -9.639669E37F, PH.base.pack) ;
    p103_z_SET((float)1.1834877E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
    p104_usec_SET((uint64_t)4493805930769103201L, PH.base.pack) ;
    p104_x_SET((float) -1.352907E38F, PH.base.pack) ;
    p104_y_SET((float)2.9824018E38F, PH.base.pack) ;
    p104_z_SET((float)6.2417807E37F, PH.base.pack) ;
    p104_roll_SET((float) -1.1665097E38F, PH.base.pack) ;
    p104_pitch_SET((float)2.1075261E38F, PH.base.pack) ;
    p104_yaw_SET((float)1.4133725E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIGHRES_IMU_105(), &PH);
    p105_time_usec_SET((uint64_t)765791739703541980L, PH.base.pack) ;
    p105_xacc_SET((float) -1.8821528E38F, PH.base.pack) ;
    p105_yacc_SET((float)1.9500956E37F, PH.base.pack) ;
    p105_zacc_SET((float) -2.625976E38F, PH.base.pack) ;
    p105_xgyro_SET((float) -9.546294E37F, PH.base.pack) ;
    p105_ygyro_SET((float)4.9229845E37F, PH.base.pack) ;
    p105_zgyro_SET((float) -1.3499522E38F, PH.base.pack) ;
    p105_xmag_SET((float)8.569525E37F, PH.base.pack) ;
    p105_ymag_SET((float) -2.583609E38F, PH.base.pack) ;
    p105_zmag_SET((float)1.5451471E38F, PH.base.pack) ;
    p105_abs_pressure_SET((float) -7.2800545E37F, PH.base.pack) ;
    p105_diff_pressure_SET((float)1.0423744E38F, PH.base.pack) ;
    p105_pressure_alt_SET((float)6.8988786E37F, PH.base.pack) ;
    p105_temperature_SET((float)2.8015238E38F, PH.base.pack) ;
    p105_fields_updated_SET((uint16_t)(uint16_t)57346, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
    p106_time_usec_SET((uint64_t)1987588944172669729L, PH.base.pack) ;
    p106_sensor_id_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
    p106_integration_time_us_SET((uint32_t)2966117127L, PH.base.pack) ;
    p106_integrated_x_SET((float) -2.2659447E38F, PH.base.pack) ;
    p106_integrated_y_SET((float)3.242596E37F, PH.base.pack) ;
    p106_integrated_xgyro_SET((float)1.3752145E38F, PH.base.pack) ;
    p106_integrated_ygyro_SET((float)1.7352226E38F, PH.base.pack) ;
    p106_integrated_zgyro_SET((float) -2.5783072E38F, PH.base.pack) ;
    p106_temperature_SET((int16_t)(int16_t) -31300, PH.base.pack) ;
    p106_quality_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
    p106_time_delta_distance_us_SET((uint32_t)389959430L, PH.base.pack) ;
    p106_distance_SET((float)1.0399749E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_SENSOR_107(), &PH);
    p107_time_usec_SET((uint64_t)4346228506454762301L, PH.base.pack) ;
    p107_xacc_SET((float) -1.0807565E38F, PH.base.pack) ;
    p107_yacc_SET((float)1.84775E38F, PH.base.pack) ;
    p107_zacc_SET((float)1.4371039E38F, PH.base.pack) ;
    p107_xgyro_SET((float) -2.0507552E38F, PH.base.pack) ;
    p107_ygyro_SET((float) -4.1992526E37F, PH.base.pack) ;
    p107_zgyro_SET((float) -5.222096E36F, PH.base.pack) ;
    p107_xmag_SET((float) -6.103015E36F, PH.base.pack) ;
    p107_ymag_SET((float)2.861619E38F, PH.base.pack) ;
    p107_zmag_SET((float) -1.7544572E38F, PH.base.pack) ;
    p107_abs_pressure_SET((float)2.033127E38F, PH.base.pack) ;
    p107_diff_pressure_SET((float) -9.463537E37F, PH.base.pack) ;
    p107_pressure_alt_SET((float) -9.138511E37F, PH.base.pack) ;
    p107_temperature_SET((float)4.2903533E37F, PH.base.pack) ;
    p107_fields_updated_SET((uint32_t)1440896171L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SIM_STATE_108(), &PH);
    p108_q1_SET((float) -1.4052019E38F, PH.base.pack) ;
    p108_q2_SET((float)9.666706E37F, PH.base.pack) ;
    p108_q3_SET((float) -9.166351E37F, PH.base.pack) ;
    p108_q4_SET((float)2.7266325E38F, PH.base.pack) ;
    p108_roll_SET((float) -9.029051E35F, PH.base.pack) ;
    p108_pitch_SET((float) -2.055378E38F, PH.base.pack) ;
    p108_yaw_SET((float)4.882438E37F, PH.base.pack) ;
    p108_xacc_SET((float)9.938323E37F, PH.base.pack) ;
    p108_yacc_SET((float) -1.6936984E38F, PH.base.pack) ;
    p108_zacc_SET((float) -2.3780523E38F, PH.base.pack) ;
    p108_xgyro_SET((float) -2.7340429E38F, PH.base.pack) ;
    p108_ygyro_SET((float) -8.578401E37F, PH.base.pack) ;
    p108_zgyro_SET((float) -2.1983335E38F, PH.base.pack) ;
    p108_lat_SET((float)1.531097E37F, PH.base.pack) ;
    p108_lon_SET((float)1.5449095E38F, PH.base.pack) ;
    p108_alt_SET((float) -6.740671E37F, PH.base.pack) ;
    p108_std_dev_horz_SET((float) -1.7970574E38F, PH.base.pack) ;
    p108_std_dev_vert_SET((float)3.2533093E38F, PH.base.pack) ;
    p108_vn_SET((float) -1.071071E38F, PH.base.pack) ;
    p108_ve_SET((float)1.3840984E38F, PH.base.pack) ;
    p108_vd_SET((float)1.7007407E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RADIO_STATUS_109(), &PH);
    p109_rssi_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
    p109_remrssi_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
    p109_txbuf_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    p109_noise_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
    p109_remnoise_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
    p109_rxerrors_SET((uint16_t)(uint16_t)47123, PH.base.pack) ;
    p109_fixed__SET((uint16_t)(uint16_t)44631, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
    p110_target_network_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
    p110_target_system_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
    p110_target_component_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)22, (uint8_t)230, (uint8_t)5, (uint8_t)148, (uint8_t)2, (uint8_t)132, (uint8_t)225, (uint8_t)142, (uint8_t)161, (uint8_t)240, (uint8_t)63, (uint8_t)208, (uint8_t)50, (uint8_t)116, (uint8_t)146, (uint8_t)144, (uint8_t)174, (uint8_t)69, (uint8_t)133, (uint8_t)52, (uint8_t)117, (uint8_t)17, (uint8_t)251, (uint8_t)254, (uint8_t)37, (uint8_t)182, (uint8_t)5, (uint8_t)64, (uint8_t)165, (uint8_t)253, (uint8_t)21, (uint8_t)68, (uint8_t)222, (uint8_t)230, (uint8_t)209, (uint8_t)72, (uint8_t)116, (uint8_t)180, (uint8_t)119, (uint8_t)109, (uint8_t)203, (uint8_t)23, (uint8_t)27, (uint8_t)114, (uint8_t)150, (uint8_t)61, (uint8_t)44, (uint8_t)224, (uint8_t)53, (uint8_t)180, (uint8_t)63, (uint8_t)134, (uint8_t)138, (uint8_t)220, (uint8_t)29, (uint8_t)255, (uint8_t)165, (uint8_t)50, (uint8_t)216, (uint8_t)3, (uint8_t)208, (uint8_t)190, (uint8_t)95, (uint8_t)211, (uint8_t)237, (uint8_t)209, (uint8_t)82, (uint8_t)26, (uint8_t)238, (uint8_t)246, (uint8_t)244, (uint8_t)116, (uint8_t)206, (uint8_t)180, (uint8_t)153, (uint8_t)254, (uint8_t)77, (uint8_t)239, (uint8_t)11, (uint8_t)193, (uint8_t)241, (uint8_t)106, (uint8_t)111, (uint8_t)176, (uint8_t)111, (uint8_t)225, (uint8_t)51, (uint8_t)204, (uint8_t)105, (uint8_t)4, (uint8_t)167, (uint8_t)118, (uint8_t)89, (uint8_t)55, (uint8_t)146, (uint8_t)8, (uint8_t)229, (uint8_t)158, (uint8_t)99, (uint8_t)164, (uint8_t)65, (uint8_t)121, (uint8_t)86, (uint8_t)253, (uint8_t)104, (uint8_t)61, (uint8_t)167, (uint8_t)109, (uint8_t)108, (uint8_t)81, (uint8_t)21, (uint8_t)90, (uint8_t)67, (uint8_t)225, (uint8_t)96, (uint8_t)112, (uint8_t)238, (uint8_t)255, (uint8_t)237, (uint8_t)98, (uint8_t)46, (uint8_t)152, (uint8_t)240, (uint8_t)72, (uint8_t)99, (uint8_t)150, (uint8_t)106, (uint8_t)143, (uint8_t)128, (uint8_t)170, (uint8_t)186, (uint8_t)223, (uint8_t)251, (uint8_t)61, (uint8_t)195, (uint8_t)60, (uint8_t)183, (uint8_t)173, (uint8_t)24, (uint8_t)86, (uint8_t)121, (uint8_t)38, (uint8_t)94, (uint8_t)157, (uint8_t)86, (uint8_t)128, (uint8_t)29, (uint8_t)119, (uint8_t)189, (uint8_t)106, (uint8_t)120, (uint8_t)98, (uint8_t)61, (uint8_t)183, (uint8_t)168, (uint8_t)47, (uint8_t)224, (uint8_t)154, (uint8_t)175, (uint8_t)221, (uint8_t)59, (uint8_t)131, (uint8_t)191, (uint8_t)204, (uint8_t)199, (uint8_t)210, (uint8_t)54, (uint8_t)220, (uint8_t)142, (uint8_t)168, (uint8_t)51, (uint8_t)253, (uint8_t)12, (uint8_t)253, (uint8_t)7, (uint8_t)6, (uint8_t)40, (uint8_t)97, (uint8_t)152, (uint8_t)225, (uint8_t)152, (uint8_t)79, (uint8_t)40, (uint8_t)51, (uint8_t)115, (uint8_t)116, (uint8_t)34, (uint8_t)99, (uint8_t)70, (uint8_t)136, (uint8_t)141, (uint8_t)242, (uint8_t)43, (uint8_t)153, (uint8_t)160, (uint8_t)197, (uint8_t)4, (uint8_t)151, (uint8_t)188, (uint8_t)164, (uint8_t)41, (uint8_t)2, (uint8_t)173, (uint8_t)173, (uint8_t)252, (uint8_t)50, (uint8_t)145, (uint8_t)0, (uint8_t)153, (uint8_t)249, (uint8_t)75, (uint8_t)244, (uint8_t)29, (uint8_t)214, (uint8_t)223, (uint8_t)120, (uint8_t)231, (uint8_t)50, (uint8_t)135, (uint8_t)63, (uint8_t)124, (uint8_t)107, (uint8_t)170, (uint8_t)22, (uint8_t)48, (uint8_t)216, (uint8_t)102, (uint8_t)233, (uint8_t)12, (uint8_t)28, (uint8_t)62, (uint8_t)60, (uint8_t)209, (uint8_t)24, (uint8_t)231, (uint8_t)58, (uint8_t)139, (uint8_t)252, (uint8_t)249, (uint8_t)164, (uint8_t)157, (uint8_t)216, (uint8_t)118, (uint8_t)180, (uint8_t)98, (uint8_t)150, (uint8_t)15, (uint8_t)115, (uint8_t)82, (uint8_t)46, (uint8_t)46};
        p110_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TIMESYNC_111(), &PH);
    p111_tc1_SET((int64_t)3605787947025281187L, PH.base.pack) ;
    p111_ts1_SET((int64_t) -1908117402423873997L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_TRIGGER_112(), &PH);
    p112_time_usec_SET((uint64_t)5975988671582103167L, PH.base.pack) ;
    p112_seq_SET((uint32_t)4092788976L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_GPS_113(), &PH);
    p113_time_usec_SET((uint64_t)1485080164440798288L, PH.base.pack) ;
    p113_fix_type_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
    p113_lat_SET((int32_t) -2082668241, PH.base.pack) ;
    p113_lon_SET((int32_t)1440290398, PH.base.pack) ;
    p113_alt_SET((int32_t)1321197344, PH.base.pack) ;
    p113_eph_SET((uint16_t)(uint16_t)22297, PH.base.pack) ;
    p113_epv_SET((uint16_t)(uint16_t)60121, PH.base.pack) ;
    p113_vel_SET((uint16_t)(uint16_t)61121, PH.base.pack) ;
    p113_vn_SET((int16_t)(int16_t)15360, PH.base.pack) ;
    p113_ve_SET((int16_t)(int16_t) -20812, PH.base.pack) ;
    p113_vd_SET((int16_t)(int16_t) -18815, PH.base.pack) ;
    p113_cog_SET((uint16_t)(uint16_t)11528, PH.base.pack) ;
    p113_satellites_visible_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
    p114_time_usec_SET((uint64_t)3940941705204712184L, PH.base.pack) ;
    p114_sensor_id_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
    p114_integration_time_us_SET((uint32_t)977536981L, PH.base.pack) ;
    p114_integrated_x_SET((float)3.2771954E38F, PH.base.pack) ;
    p114_integrated_y_SET((float)2.995704E38F, PH.base.pack) ;
    p114_integrated_xgyro_SET((float)3.2136504E37F, PH.base.pack) ;
    p114_integrated_ygyro_SET((float)1.1677499E38F, PH.base.pack) ;
    p114_integrated_zgyro_SET((float) -2.561246E38F, PH.base.pack) ;
    p114_temperature_SET((int16_t)(int16_t) -22699, PH.base.pack) ;
    p114_quality_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
    p114_time_delta_distance_us_SET((uint32_t)1201946033L, PH.base.pack) ;
    p114_distance_SET((float)1.6147181E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_STATE_QUATERNION_115(), &PH);
    p115_time_usec_SET((uint64_t)6853378591275884394L, PH.base.pack) ;
    {
        float  attitude_quaternion [] =  {5.3997763E37F, -1.5741268E38F, 7.658012E36F, -2.505573E38F};
        p115_attitude_quaternion_SET(&attitude_quaternion, 0, &PH.base.pack) ;
    }
    p115_rollspeed_SET((float)1.666988E38F, PH.base.pack) ;
    p115_pitchspeed_SET((float)4.3314825E37F, PH.base.pack) ;
    p115_yawspeed_SET((float) -1.683891E38F, PH.base.pack) ;
    p115_lat_SET((int32_t)1060562216, PH.base.pack) ;
    p115_lon_SET((int32_t)522586351, PH.base.pack) ;
    p115_alt_SET((int32_t)1961642471, PH.base.pack) ;
    p115_vx_SET((int16_t)(int16_t)10287, PH.base.pack) ;
    p115_vy_SET((int16_t)(int16_t) -79, PH.base.pack) ;
    p115_vz_SET((int16_t)(int16_t) -9075, PH.base.pack) ;
    p115_ind_airspeed_SET((uint16_t)(uint16_t)60109, PH.base.pack) ;
    p115_true_airspeed_SET((uint16_t)(uint16_t)46655, PH.base.pack) ;
    p115_xacc_SET((int16_t)(int16_t) -16155, PH.base.pack) ;
    p115_yacc_SET((int16_t)(int16_t)9426, PH.base.pack) ;
    p115_zacc_SET((int16_t)(int16_t)3250, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU2_116(), &PH);
    p116_time_boot_ms_SET((uint32_t)13131073L, PH.base.pack) ;
    p116_xacc_SET((int16_t)(int16_t) -9219, PH.base.pack) ;
    p116_yacc_SET((int16_t)(int16_t) -31989, PH.base.pack) ;
    p116_zacc_SET((int16_t)(int16_t) -3905, PH.base.pack) ;
    p116_xgyro_SET((int16_t)(int16_t)7848, PH.base.pack) ;
    p116_ygyro_SET((int16_t)(int16_t)4023, PH.base.pack) ;
    p116_zgyro_SET((int16_t)(int16_t)3909, PH.base.pack) ;
    p116_xmag_SET((int16_t)(int16_t) -5452, PH.base.pack) ;
    p116_ymag_SET((int16_t)(int16_t)21337, PH.base.pack) ;
    p116_zmag_SET((int16_t)(int16_t)12033, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_LIST_117(), &PH);
    p117_target_system_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
    p117_target_component_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
    p117_start_SET((uint16_t)(uint16_t)48868, PH.base.pack) ;
    p117_end_SET((uint16_t)(uint16_t)21236, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_ENTRY_118(), &PH);
    p118_id_SET((uint16_t)(uint16_t)41165, PH.base.pack) ;
    p118_num_logs_SET((uint16_t)(uint16_t)52257, PH.base.pack) ;
    p118_last_log_num_SET((uint16_t)(uint16_t)24731, PH.base.pack) ;
    p118_time_utc_SET((uint32_t)3258645717L, PH.base.pack) ;
    p118_size_SET((uint32_t)84103429L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_DATA_119(), &PH);
    p119_target_system_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
    p119_target_component_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
    p119_id_SET((uint16_t)(uint16_t)33178, PH.base.pack) ;
    p119_ofs_SET((uint32_t)3164707932L, PH.base.pack) ;
    p119_count_SET((uint32_t)249845921L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_DATA_120(), &PH);
    p120_id_SET((uint16_t)(uint16_t)4045, PH.base.pack) ;
    p120_ofs_SET((uint32_t)1302917140L, PH.base.pack) ;
    p120_count_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)28, (uint8_t)217, (uint8_t)103, (uint8_t)21, (uint8_t)115, (uint8_t)75, (uint8_t)163, (uint8_t)131, (uint8_t)36, (uint8_t)145, (uint8_t)181, (uint8_t)243, (uint8_t)254, (uint8_t)5, (uint8_t)211, (uint8_t)178, (uint8_t)120, (uint8_t)174, (uint8_t)0, (uint8_t)21, (uint8_t)142, (uint8_t)176, (uint8_t)221, (uint8_t)205, (uint8_t)63, (uint8_t)119, (uint8_t)237, (uint8_t)243, (uint8_t)132, (uint8_t)109, (uint8_t)83, (uint8_t)133, (uint8_t)126, (uint8_t)197, (uint8_t)166, (uint8_t)152, (uint8_t)199, (uint8_t)160, (uint8_t)61, (uint8_t)254, (uint8_t)220, (uint8_t)15, (uint8_t)12, (uint8_t)14, (uint8_t)142, (uint8_t)190, (uint8_t)254, (uint8_t)234, (uint8_t)186, (uint8_t)14, (uint8_t)59, (uint8_t)112, (uint8_t)23, (uint8_t)64, (uint8_t)78, (uint8_t)35, (uint8_t)162, (uint8_t)235, (uint8_t)231, (uint8_t)54, (uint8_t)51, (uint8_t)184, (uint8_t)200, (uint8_t)232, (uint8_t)136, (uint8_t)146, (uint8_t)169, (uint8_t)190, (uint8_t)41, (uint8_t)52, (uint8_t)46, (uint8_t)61, (uint8_t)49, (uint8_t)102, (uint8_t)142, (uint8_t)196, (uint8_t)100, (uint8_t)183, (uint8_t)233, (uint8_t)64, (uint8_t)233, (uint8_t)197, (uint8_t)64, (uint8_t)51, (uint8_t)230, (uint8_t)197, (uint8_t)195, (uint8_t)125, (uint8_t)138, (uint8_t)231};
        p120_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_ERASE_121(), &PH);
    p121_target_system_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
    p121_target_component_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_END_122(), &PH);
    p122_target_system_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
    p122_target_component_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_INJECT_DATA_123(), &PH);
    p123_target_system_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    p123_target_component_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
    p123_len_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)194, (uint8_t)153, (uint8_t)60, (uint8_t)231, (uint8_t)238, (uint8_t)143, (uint8_t)174, (uint8_t)174, (uint8_t)253, (uint8_t)242, (uint8_t)205, (uint8_t)121, (uint8_t)155, (uint8_t)12, (uint8_t)204, (uint8_t)33, (uint8_t)167, (uint8_t)204, (uint8_t)125, (uint8_t)178, (uint8_t)28, (uint8_t)220, (uint8_t)37, (uint8_t)197, (uint8_t)4, (uint8_t)247, (uint8_t)146, (uint8_t)199, (uint8_t)47, (uint8_t)228, (uint8_t)171, (uint8_t)150, (uint8_t)206, (uint8_t)154, (uint8_t)53, (uint8_t)6, (uint8_t)151, (uint8_t)142, (uint8_t)47, (uint8_t)131, (uint8_t)49, (uint8_t)214, (uint8_t)222, (uint8_t)234, (uint8_t)245, (uint8_t)72, (uint8_t)121, (uint8_t)243, (uint8_t)95, (uint8_t)110, (uint8_t)68, (uint8_t)198, (uint8_t)210, (uint8_t)5, (uint8_t)171, (uint8_t)220, (uint8_t)4, (uint8_t)175, (uint8_t)115, (uint8_t)37, (uint8_t)223, (uint8_t)140, (uint8_t)52, (uint8_t)222, (uint8_t)124, (uint8_t)29, (uint8_t)168, (uint8_t)205, (uint8_t)133, (uint8_t)201, (uint8_t)137, (uint8_t)224, (uint8_t)240, (uint8_t)41, (uint8_t)180, (uint8_t)43, (uint8_t)4, (uint8_t)118, (uint8_t)141, (uint8_t)38, (uint8_t)67, (uint8_t)141, (uint8_t)98, (uint8_t)107, (uint8_t)133, (uint8_t)89, (uint8_t)28, (uint8_t)169, (uint8_t)131, (uint8_t)23, (uint8_t)57, (uint8_t)252, (uint8_t)131, (uint8_t)120, (uint8_t)14, (uint8_t)72, (uint8_t)213, (uint8_t)172, (uint8_t)61, (uint8_t)189, (uint8_t)183, (uint8_t)101, (uint8_t)147, (uint8_t)110, (uint8_t)96, (uint8_t)250, (uint8_t)2, (uint8_t)73, (uint8_t)61, (uint8_t)10};
        p123_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS2_RAW_124(), &PH);
    p124_time_usec_SET((uint64_t)798161817942857831L, PH.base.pack) ;
    p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, PH.base.pack) ;
    p124_lat_SET((int32_t) -1317609366, PH.base.pack) ;
    p124_lon_SET((int32_t)1215874589, PH.base.pack) ;
    p124_alt_SET((int32_t)855408159, PH.base.pack) ;
    p124_eph_SET((uint16_t)(uint16_t)40575, PH.base.pack) ;
    p124_epv_SET((uint16_t)(uint16_t)6439, PH.base.pack) ;
    p124_vel_SET((uint16_t)(uint16_t)4019, PH.base.pack) ;
    p124_cog_SET((uint16_t)(uint16_t)57159, PH.base.pack) ;
    p124_satellites_visible_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
    p124_dgps_numch_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
    p124_dgps_age_SET((uint32_t)2745608532L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POWER_STATUS_125(), &PH);
    p125_Vcc_SET((uint16_t)(uint16_t)3891, PH.base.pack) ;
    p125_Vservo_SET((uint16_t)(uint16_t)8715, PH.base.pack) ;
    p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_CONTROL_126(), &PH);
    p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2, PH.base.pack) ;
    p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND, PH.base.pack) ;
    p126_timeout_SET((uint16_t)(uint16_t)54451, PH.base.pack) ;
    p126_baudrate_SET((uint32_t)1255063112L, PH.base.pack) ;
    p126_count_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)13, (uint8_t)140, (uint8_t)205, (uint8_t)184, (uint8_t)44, (uint8_t)3, (uint8_t)229, (uint8_t)236, (uint8_t)79, (uint8_t)159, (uint8_t)143, (uint8_t)49, (uint8_t)186, (uint8_t)106, (uint8_t)124, (uint8_t)7, (uint8_t)138, (uint8_t)217, (uint8_t)67, (uint8_t)163, (uint8_t)144, (uint8_t)201, (uint8_t)249, (uint8_t)43, (uint8_t)155, (uint8_t)130, (uint8_t)69, (uint8_t)55, (uint8_t)38, (uint8_t)82, (uint8_t)8, (uint8_t)11, (uint8_t)69, (uint8_t)177, (uint8_t)97, (uint8_t)227, (uint8_t)15, (uint8_t)254, (uint8_t)194, (uint8_t)142, (uint8_t)35, (uint8_t)130, (uint8_t)77, (uint8_t)158, (uint8_t)183, (uint8_t)15, (uint8_t)44, (uint8_t)247, (uint8_t)211, (uint8_t)128, (uint8_t)34, (uint8_t)219, (uint8_t)164, (uint8_t)199, (uint8_t)86, (uint8_t)153, (uint8_t)144, (uint8_t)100, (uint8_t)0, (uint8_t)156, (uint8_t)151, (uint8_t)195, (uint8_t)151, (uint8_t)128, (uint8_t)133, (uint8_t)79, (uint8_t)207, (uint8_t)239, (uint8_t)15, (uint8_t)108};
        p126_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RTK_127(), &PH);
    p127_time_last_baseline_ms_SET((uint32_t)3041892532L, PH.base.pack) ;
    p127_rtk_receiver_id_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
    p127_wn_SET((uint16_t)(uint16_t)47119, PH.base.pack) ;
    p127_tow_SET((uint32_t)3235105030L, PH.base.pack) ;
    p127_rtk_health_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
    p127_rtk_rate_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
    p127_nsats_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
    p127_baseline_coords_type_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
    p127_baseline_a_mm_SET((int32_t) -1403609462, PH.base.pack) ;
    p127_baseline_b_mm_SET((int32_t)1985134021, PH.base.pack) ;
    p127_baseline_c_mm_SET((int32_t)1887240621, PH.base.pack) ;
    p127_accuracy_SET((uint32_t)1317268351L, PH.base.pack) ;
    p127_iar_num_hypotheses_SET((int32_t)1250100525, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS2_RTK_128(), &PH);
    p128_time_last_baseline_ms_SET((uint32_t)2256503178L, PH.base.pack) ;
    p128_rtk_receiver_id_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    p128_wn_SET((uint16_t)(uint16_t)43680, PH.base.pack) ;
    p128_tow_SET((uint32_t)2957872540L, PH.base.pack) ;
    p128_rtk_health_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
    p128_rtk_rate_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    p128_nsats_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
    p128_baseline_coords_type_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
    p128_baseline_a_mm_SET((int32_t)568844896, PH.base.pack) ;
    p128_baseline_b_mm_SET((int32_t)214000792, PH.base.pack) ;
    p128_baseline_c_mm_SET((int32_t) -619930418, PH.base.pack) ;
    p128_accuracy_SET((uint32_t)1862515126L, PH.base.pack) ;
    p128_iar_num_hypotheses_SET((int32_t) -1351659656, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU3_129(), &PH);
    p129_time_boot_ms_SET((uint32_t)2119512159L, PH.base.pack) ;
    p129_xacc_SET((int16_t)(int16_t) -30485, PH.base.pack) ;
    p129_yacc_SET((int16_t)(int16_t) -11196, PH.base.pack) ;
    p129_zacc_SET((int16_t)(int16_t) -19949, PH.base.pack) ;
    p129_xgyro_SET((int16_t)(int16_t) -27950, PH.base.pack) ;
    p129_ygyro_SET((int16_t)(int16_t) -26833, PH.base.pack) ;
    p129_zgyro_SET((int16_t)(int16_t) -1861, PH.base.pack) ;
    p129_xmag_SET((int16_t)(int16_t) -32256, PH.base.pack) ;
    p129_ymag_SET((int16_t)(int16_t) -7714, PH.base.pack) ;
    p129_zmag_SET((int16_t)(int16_t)12504, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
    p130_type_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
    p130_size_SET((uint32_t)4289317811L, PH.base.pack) ;
    p130_width_SET((uint16_t)(uint16_t)63522, PH.base.pack) ;
    p130_height_SET((uint16_t)(uint16_t)22693, PH.base.pack) ;
    p130_packets_SET((uint16_t)(uint16_t)48448, PH.base.pack) ;
    p130_payload_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
    p130_jpg_quality_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ENCAPSULATED_DATA_131(), &PH);
    p131_seqnr_SET((uint16_t)(uint16_t)37735, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)205, (uint8_t)236, (uint8_t)148, (uint8_t)170, (uint8_t)105, (uint8_t)149, (uint8_t)8, (uint8_t)18, (uint8_t)204, (uint8_t)148, (uint8_t)109, (uint8_t)92, (uint8_t)140, (uint8_t)109, (uint8_t)116, (uint8_t)40, (uint8_t)2, (uint8_t)204, (uint8_t)155, (uint8_t)14, (uint8_t)201, (uint8_t)171, (uint8_t)210, (uint8_t)10, (uint8_t)206, (uint8_t)142, (uint8_t)191, (uint8_t)188, (uint8_t)2, (uint8_t)129, (uint8_t)240, (uint8_t)97, (uint8_t)18, (uint8_t)184, (uint8_t)72, (uint8_t)46, (uint8_t)6, (uint8_t)7, (uint8_t)61, (uint8_t)12, (uint8_t)237, (uint8_t)119, (uint8_t)18, (uint8_t)48, (uint8_t)25, (uint8_t)118, (uint8_t)178, (uint8_t)210, (uint8_t)10, (uint8_t)15, (uint8_t)51, (uint8_t)229, (uint8_t)91, (uint8_t)127, (uint8_t)194, (uint8_t)249, (uint8_t)154, (uint8_t)214, (uint8_t)189, (uint8_t)183, (uint8_t)14, (uint8_t)191, (uint8_t)166, (uint8_t)243, (uint8_t)93, (uint8_t)231, (uint8_t)35, (uint8_t)140, (uint8_t)202, (uint8_t)7, (uint8_t)254, (uint8_t)213, (uint8_t)38, (uint8_t)244, (uint8_t)99, (uint8_t)8, (uint8_t)162, (uint8_t)178, (uint8_t)237, (uint8_t)41, (uint8_t)96, (uint8_t)205, (uint8_t)86, (uint8_t)244, (uint8_t)45, (uint8_t)70, (uint8_t)96, (uint8_t)28, (uint8_t)173, (uint8_t)141, (uint8_t)226, (uint8_t)222, (uint8_t)242, (uint8_t)52, (uint8_t)124, (uint8_t)2, (uint8_t)95, (uint8_t)92, (uint8_t)219, (uint8_t)19, (uint8_t)53, (uint8_t)181, (uint8_t)252, (uint8_t)34, (uint8_t)165, (uint8_t)203, (uint8_t)95, (uint8_t)64, (uint8_t)248, (uint8_t)165, (uint8_t)95, (uint8_t)90, (uint8_t)218, (uint8_t)58, (uint8_t)71, (uint8_t)227, (uint8_t)197, (uint8_t)218, (uint8_t)36, (uint8_t)15, (uint8_t)217, (uint8_t)127, (uint8_t)49, (uint8_t)191, (uint8_t)130, (uint8_t)158, (uint8_t)57, (uint8_t)150, (uint8_t)83, (uint8_t)229, (uint8_t)84, (uint8_t)82, (uint8_t)12, (uint8_t)87, (uint8_t)206, (uint8_t)158, (uint8_t)34, (uint8_t)138, (uint8_t)143, (uint8_t)243, (uint8_t)184, (uint8_t)110, (uint8_t)19, (uint8_t)145, (uint8_t)151, (uint8_t)94, (uint8_t)226, (uint8_t)251, (uint8_t)163, (uint8_t)68, (uint8_t)41, (uint8_t)41, (uint8_t)250, (uint8_t)68, (uint8_t)39, (uint8_t)178, (uint8_t)113, (uint8_t)230, (uint8_t)126, (uint8_t)56, (uint8_t)180, (uint8_t)48, (uint8_t)221, (uint8_t)192, (uint8_t)131, (uint8_t)76, (uint8_t)235, (uint8_t)100, (uint8_t)231, (uint8_t)108, (uint8_t)129, (uint8_t)92, (uint8_t)94, (uint8_t)199, (uint8_t)220, (uint8_t)193, (uint8_t)46, (uint8_t)170, (uint8_t)131, (uint8_t)248, (uint8_t)235, (uint8_t)76, (uint8_t)57, (uint8_t)92, (uint8_t)247, (uint8_t)177, (uint8_t)150, (uint8_t)179, (uint8_t)188, (uint8_t)171, (uint8_t)86, (uint8_t)176, (uint8_t)120, (uint8_t)15, (uint8_t)208, (uint8_t)179, (uint8_t)71, (uint8_t)226, (uint8_t)79, (uint8_t)2, (uint8_t)162, (uint8_t)251, (uint8_t)31, (uint8_t)235, (uint8_t)75, (uint8_t)221, (uint8_t)44, (uint8_t)66, (uint8_t)215, (uint8_t)69, (uint8_t)150, (uint8_t)114, (uint8_t)33, (uint8_t)129, (uint8_t)20, (uint8_t)131, (uint8_t)157, (uint8_t)112, (uint8_t)152, (uint8_t)18, (uint8_t)151, (uint8_t)72, (uint8_t)183, (uint8_t)228, (uint8_t)22, (uint8_t)126, (uint8_t)207, (uint8_t)77, (uint8_t)63, (uint8_t)106, (uint8_t)1, (uint8_t)5, (uint8_t)13, (uint8_t)77, (uint8_t)156, (uint8_t)234, (uint8_t)111, (uint8_t)27, (uint8_t)166, (uint8_t)180, (uint8_t)162, (uint8_t)24, (uint8_t)64, (uint8_t)169, (uint8_t)137, (uint8_t)89, (uint8_t)17, (uint8_t)227, (uint8_t)156, (uint8_t)57, (uint8_t)134, (uint8_t)47, (uint8_t)220};
        p131_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DISTANCE_SENSOR_132(), &PH);
    p132_time_boot_ms_SET((uint32_t)1438196807L, PH.base.pack) ;
    p132_min_distance_SET((uint16_t)(uint16_t)17958, PH.base.pack) ;
    p132_max_distance_SET((uint16_t)(uint16_t)26214, PH.base.pack) ;
    p132_current_distance_SET((uint16_t)(uint16_t)41750, PH.base.pack) ;
    p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND, PH.base.pack) ;
    p132_id_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
    p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_180, PH.base.pack) ;
    p132_covariance_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_REQUEST_133(), &PH);
    p133_lat_SET((int32_t)1090295499, PH.base.pack) ;
    p133_lon_SET((int32_t)689474914, PH.base.pack) ;
    p133_grid_spacing_SET((uint16_t)(uint16_t)12613, PH.base.pack) ;
    p133_mask_SET((uint64_t)4119276118391108380L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_DATA_134(), &PH);
    p134_lat_SET((int32_t) -521604917, PH.base.pack) ;
    p134_lon_SET((int32_t) -1303269874, PH.base.pack) ;
    p134_grid_spacing_SET((uint16_t)(uint16_t)31947, PH.base.pack) ;
    p134_gridbit_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
    {
        int16_t  data_ [] =  {(int16_t) -18098, (int16_t)22000, (int16_t)6039, (int16_t) -7345, (int16_t) -10358, (int16_t)22303, (int16_t) -5726, (int16_t) -3461, (int16_t)10713, (int16_t)15408, (int16_t) -20115, (int16_t)25566, (int16_t) -11793, (int16_t) -25360, (int16_t)22285, (int16_t) -4755};
        p134_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_CHECK_135(), &PH);
    p135_lat_SET((int32_t) -492825457, PH.base.pack) ;
    p135_lon_SET((int32_t) -445280860, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_REPORT_136(), &PH);
    p136_lat_SET((int32_t)1853035105, PH.base.pack) ;
    p136_lon_SET((int32_t)1769919562, PH.base.pack) ;
    p136_spacing_SET((uint16_t)(uint16_t)61421, PH.base.pack) ;
    p136_terrain_height_SET((float)2.9347113E37F, PH.base.pack) ;
    p136_current_height_SET((float) -1.9805856E38F, PH.base.pack) ;
    p136_pending_SET((uint16_t)(uint16_t)63212, PH.base.pack) ;
    p136_loaded_SET((uint16_t)(uint16_t)58477, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE2_137(), &PH);
    p137_time_boot_ms_SET((uint32_t)90060809L, PH.base.pack) ;
    p137_press_abs_SET((float)2.066236E38F, PH.base.pack) ;
    p137_press_diff_SET((float)9.921204E37F, PH.base.pack) ;
    p137_temperature_SET((int16_t)(int16_t) -14234, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATT_POS_MOCAP_138(), &PH);
    p138_time_usec_SET((uint64_t)5822781674610048034L, PH.base.pack) ;
    {
        float  q [] =  {6.4019393E37F, 3.2110711E38F, -2.1513495E38F, -3.0396623E38F};
        p138_q_SET(&q, 0, &PH.base.pack) ;
    }
    p138_x_SET((float)6.251562E37F, PH.base.pack) ;
    p138_y_SET((float)1.8143159E37F, PH.base.pack) ;
    p138_z_SET((float)1.838289E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
    p139_time_usec_SET((uint64_t)7963600608660363471L, PH.base.pack) ;
    p139_group_mlx_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
    p139_target_system_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
    p139_target_component_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
    {
        float  controls [] =  {-8.360181E37F, 1.7470584E38F, -1.833321E38F, -1.0055463E38F, 1.8651177E38F, -1.2362866E38F, 1.5646656E38F, 1.0913988E38F};
        p139_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
    p140_time_usec_SET((uint64_t)1924707582964396065L, PH.base.pack) ;
    p140_group_mlx_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
    {
        float  controls [] =  {7.477699E37F, 2.427153E38F, -1.2653946E38F, 2.7849639E38F, -8.2131E37F, -8.754354E37F, 2.1302814E38F, 2.0050114E37F};
        p140_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ALTITUDE_141(), &PH);
    p141_time_usec_SET((uint64_t)608269731265091107L, PH.base.pack) ;
    p141_altitude_monotonic_SET((float)1.0684426E38F, PH.base.pack) ;
    p141_altitude_amsl_SET((float) -3.27354E38F, PH.base.pack) ;
    p141_altitude_local_SET((float) -2.274561E36F, PH.base.pack) ;
    p141_altitude_relative_SET((float) -7.72032E37F, PH.base.pack) ;
    p141_altitude_terrain_SET((float) -3.3207779E38F, PH.base.pack) ;
    p141_bottom_clearance_SET((float)2.8963218E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RESOURCE_REQUEST_142(), &PH);
    p142_request_id_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
    p142_uri_type_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
    {
        uint8_t  uri [] =  {(uint8_t)206, (uint8_t)12, (uint8_t)81, (uint8_t)29, (uint8_t)16, (uint8_t)125, (uint8_t)114, (uint8_t)108, (uint8_t)32, (uint8_t)205, (uint8_t)76, (uint8_t)184, (uint8_t)121, (uint8_t)207, (uint8_t)243, (uint8_t)46, (uint8_t)209, (uint8_t)81, (uint8_t)0, (uint8_t)135, (uint8_t)183, (uint8_t)179, (uint8_t)122, (uint8_t)23, (uint8_t)119, (uint8_t)38, (uint8_t)122, (uint8_t)44, (uint8_t)124, (uint8_t)54, (uint8_t)203, (uint8_t)26, (uint8_t)85, (uint8_t)219, (uint8_t)22, (uint8_t)47, (uint8_t)213, (uint8_t)50, (uint8_t)250, (uint8_t)55, (uint8_t)167, (uint8_t)44, (uint8_t)78, (uint8_t)97, (uint8_t)203, (uint8_t)203, (uint8_t)1, (uint8_t)31, (uint8_t)164, (uint8_t)22, (uint8_t)2, (uint8_t)106, (uint8_t)238, (uint8_t)81, (uint8_t)107, (uint8_t)135, (uint8_t)112, (uint8_t)140, (uint8_t)153, (uint8_t)48, (uint8_t)38, (uint8_t)105, (uint8_t)181, (uint8_t)55, (uint8_t)111, (uint8_t)47, (uint8_t)27, (uint8_t)79, (uint8_t)166, (uint8_t)55, (uint8_t)148, (uint8_t)194, (uint8_t)243, (uint8_t)248, (uint8_t)37, (uint8_t)79, (uint8_t)254, (uint8_t)163, (uint8_t)127, (uint8_t)75, (uint8_t)211, (uint8_t)158, (uint8_t)149, (uint8_t)98, (uint8_t)204, (uint8_t)219, (uint8_t)99, (uint8_t)78, (uint8_t)147, (uint8_t)221, (uint8_t)55, (uint8_t)249, (uint8_t)85, (uint8_t)173, (uint8_t)195, (uint8_t)62, (uint8_t)93, (uint8_t)220, (uint8_t)27, (uint8_t)227, (uint8_t)28, (uint8_t)182, (uint8_t)104, (uint8_t)210, (uint8_t)207, (uint8_t)177, (uint8_t)186, (uint8_t)33, (uint8_t)76, (uint8_t)158, (uint8_t)143, (uint8_t)174, (uint8_t)20, (uint8_t)176, (uint8_t)9, (uint8_t)212, (uint8_t)23, (uint8_t)180, (uint8_t)137, (uint8_t)109};
        p142_uri_SET(&uri, 0, &PH.base.pack) ;
    }
    p142_transfer_type_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
    {
        uint8_t  storage [] =  {(uint8_t)107, (uint8_t)203, (uint8_t)241, (uint8_t)145, (uint8_t)103, (uint8_t)5, (uint8_t)31, (uint8_t)172, (uint8_t)238, (uint8_t)91, (uint8_t)102, (uint8_t)93, (uint8_t)40, (uint8_t)200, (uint8_t)41, (uint8_t)189, (uint8_t)129, (uint8_t)220, (uint8_t)100, (uint8_t)194, (uint8_t)233, (uint8_t)63, (uint8_t)93, (uint8_t)127, (uint8_t)228, (uint8_t)94, (uint8_t)33, (uint8_t)189, (uint8_t)59, (uint8_t)7, (uint8_t)90, (uint8_t)142, (uint8_t)251, (uint8_t)40, (uint8_t)53, (uint8_t)196, (uint8_t)158, (uint8_t)69, (uint8_t)236, (uint8_t)87, (uint8_t)221, (uint8_t)192, (uint8_t)112, (uint8_t)66, (uint8_t)201, (uint8_t)31, (uint8_t)22, (uint8_t)48, (uint8_t)118, (uint8_t)13, (uint8_t)116, (uint8_t)50, (uint8_t)240, (uint8_t)151, (uint8_t)133, (uint8_t)81, (uint8_t)236, (uint8_t)127, (uint8_t)198, (uint8_t)46, (uint8_t)178, (uint8_t)241, (uint8_t)38, (uint8_t)248, (uint8_t)161, (uint8_t)91, (uint8_t)36, (uint8_t)187, (uint8_t)149, (uint8_t)136, (uint8_t)104, (uint8_t)148, (uint8_t)60, (uint8_t)87, (uint8_t)38, (uint8_t)226, (uint8_t)43, (uint8_t)44, (uint8_t)100, (uint8_t)254, (uint8_t)144, (uint8_t)242, (uint8_t)211, (uint8_t)13, (uint8_t)131, (uint8_t)139, (uint8_t)43, (uint8_t)187, (uint8_t)73, (uint8_t)50, (uint8_t)213, (uint8_t)151, (uint8_t)89, (uint8_t)226, (uint8_t)61, (uint8_t)117, (uint8_t)150, (uint8_t)71, (uint8_t)9, (uint8_t)21, (uint8_t)93, (uint8_t)49, (uint8_t)218, (uint8_t)36, (uint8_t)245, (uint8_t)17, (uint8_t)241, (uint8_t)0, (uint8_t)17, (uint8_t)181, (uint8_t)163, (uint8_t)118, (uint8_t)152, (uint8_t)169, (uint8_t)189, (uint8_t)132, (uint8_t)78, (uint8_t)43, (uint8_t)70, (uint8_t)47};
        p142_storage_SET(&storage, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE3_143(), &PH);
    p143_time_boot_ms_SET((uint32_t)2176551007L, PH.base.pack) ;
    p143_press_abs_SET((float) -2.7294947E38F, PH.base.pack) ;
    p143_press_diff_SET((float)2.2026967E38F, PH.base.pack) ;
    p143_temperature_SET((int16_t)(int16_t) -11744, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FOLLOW_TARGET_144(), &PH);
    p144_timestamp_SET((uint64_t)1211177192134181430L, PH.base.pack) ;
    p144_est_capabilities_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
    p144_lat_SET((int32_t)878427851, PH.base.pack) ;
    p144_lon_SET((int32_t) -1955515206, PH.base.pack) ;
    p144_alt_SET((float)4.0616392E37F, PH.base.pack) ;
    {
        float  vel [] =  {-3.3049477E38F, -2.6262834E38F, 3.4340472E37F};
        p144_vel_SET(&vel, 0, &PH.base.pack) ;
    }
    {
        float  acc [] =  {-1.7521016E38F, -9.110791E37F, -2.6279548E37F};
        p144_acc_SET(&acc, 0, &PH.base.pack) ;
    }
    {
        float  attitude_q [] =  {-3.2785917E38F, 8.1159764E36F, 1.0112571E37F, 1.6444557E37F};
        p144_attitude_q_SET(&attitude_q, 0, &PH.base.pack) ;
    }
    {
        float  rates [] =  {3.5624265E37F, -1.8044077E38F, -2.951598E38F};
        p144_rates_SET(&rates, 0, &PH.base.pack) ;
    }
    {
        float  position_cov [] =  {-1.2852815E38F, -8.077757E37F, 5.6833467E37F};
        p144_position_cov_SET(&position_cov, 0, &PH.base.pack) ;
    }
    p144_custom_state_SET((uint64_t)3551562862497773913L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
    p146_time_usec_SET((uint64_t)3583350137710395622L, PH.base.pack) ;
    p146_x_acc_SET((float) -8.608284E37F, PH.base.pack) ;
    p146_y_acc_SET((float) -1.4245284E37F, PH.base.pack) ;
    p146_z_acc_SET((float) -2.243811E38F, PH.base.pack) ;
    p146_x_vel_SET((float) -2.9991088E38F, PH.base.pack) ;
    p146_y_vel_SET((float) -2.160632E38F, PH.base.pack) ;
    p146_z_vel_SET((float) -2.5602248E38F, PH.base.pack) ;
    p146_x_pos_SET((float)2.4057333E38F, PH.base.pack) ;
    p146_y_pos_SET((float) -7.2209475E37F, PH.base.pack) ;
    p146_z_pos_SET((float) -1.0718445E37F, PH.base.pack) ;
    p146_airspeed_SET((float)2.6380002E38F, PH.base.pack) ;
    {
        float  vel_variance [] =  {3.0978598E38F, 1.841874E38F, 2.6733656E37F};
        p146_vel_variance_SET(&vel_variance, 0, &PH.base.pack) ;
    }
    {
        float  pos_variance [] =  {-4.2294295E37F, -1.582476E38F, -1.1789059E38F};
        p146_pos_variance_SET(&pos_variance, 0, &PH.base.pack) ;
    }
    {
        float  q [] =  {8.973543E37F, -2.8444895E38F, 1.2986635E38F, -9.369148E37F};
        p146_q_SET(&q, 0, &PH.base.pack) ;
    }
    p146_roll_rate_SET((float)1.3473483E38F, PH.base.pack) ;
    p146_pitch_rate_SET((float)2.6406426E38F, PH.base.pack) ;
    p146_yaw_rate_SET((float)2.9946078E36F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_BATTERY_STATUS_147(), &PH);
    p147_id_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
    p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_UNKNOWN, PH.base.pack) ;
    p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIFE, PH.base.pack) ;
    p147_temperature_SET((int16_t)(int16_t) -25690, PH.base.pack) ;
    {
        uint16_t  voltages [] =  {(uint16_t)60055, (uint16_t)50816, (uint16_t)24712, (uint16_t)21723, (uint16_t)57397, (uint16_t)22054, (uint16_t)46381, (uint16_t)50603, (uint16_t)34770, (uint16_t)6707};
        p147_voltages_SET(&voltages, 0, &PH.base.pack) ;
    }
    p147_current_battery_SET((int16_t)(int16_t)20167, PH.base.pack) ;
    p147_current_consumed_SET((int32_t)714961344, PH.base.pack) ;
    p147_energy_consumed_SET((int32_t)41728405, PH.base.pack) ;
    p147_battery_remaining_SET((int8_t)(int8_t)5, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_148(), &PH);
    p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FENCE, PH.base.pack) ;
    p148_flight_sw_version_SET((uint32_t)1161062205L, PH.base.pack) ;
    p148_middleware_sw_version_SET((uint32_t)2308838219L, PH.base.pack) ;
    p148_os_sw_version_SET((uint32_t)3666702466L, PH.base.pack) ;
    p148_board_version_SET((uint32_t)1591591852L, PH.base.pack) ;
    {
        uint8_t  flight_custom_version [] =  {(uint8_t)101, (uint8_t)120, (uint8_t)254, (uint8_t)252, (uint8_t)72, (uint8_t)190, (uint8_t)48, (uint8_t)127};
        p148_flight_custom_version_SET(&flight_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  middleware_custom_version [] =  {(uint8_t)84, (uint8_t)75, (uint8_t)192, (uint8_t)116, (uint8_t)188, (uint8_t)143, (uint8_t)254, (uint8_t)207};
        p148_middleware_custom_version_SET(&middleware_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  os_custom_version [] =  {(uint8_t)96, (uint8_t)66, (uint8_t)173, (uint8_t)189, (uint8_t)44, (uint8_t)31, (uint8_t)248, (uint8_t)201};
        p148_os_custom_version_SET(&os_custom_version, 0, &PH.base.pack) ;
    }
    p148_vendor_id_SET((uint16_t)(uint16_t)50954, PH.base.pack) ;
    p148_product_id_SET((uint16_t)(uint16_t)19591, PH.base.pack) ;
    p148_uid_SET((uint64_t)7577113619181002614L, PH.base.pack) ;
    {
        uint8_t  uid2 [] =  {(uint8_t)187, (uint8_t)155, (uint8_t)170, (uint8_t)20, (uint8_t)78, (uint8_t)90, (uint8_t)81, (uint8_t)56, (uint8_t)199, (uint8_t)77, (uint8_t)124, (uint8_t)233, (uint8_t)153, (uint8_t)63, (uint8_t)67, (uint8_t)218, (uint8_t)24, (uint8_t)156};
        p148_uid2_SET(&uid2, 0,  &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LANDING_TARGET_149(), &PH);
    p149_time_usec_SET((uint64_t)8129365543103503323L, PH.base.pack) ;
    p149_target_num_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
    p149_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
    p149_angle_x_SET((float)2.084454E38F, PH.base.pack) ;
    p149_angle_y_SET((float) -2.2607492E38F, PH.base.pack) ;
    p149_distance_SET((float) -3.5080068E37F, PH.base.pack) ;
    p149_size_x_SET((float)1.1117044E38F, PH.base.pack) ;
    p149_size_y_SET((float)1.2426231E38F, PH.base.pack) ;
    p149_x_SET((float) -2.018894E38F, &PH) ;
    p149_y_SET((float) -1.4828165E38F, &PH) ;
    p149_z_SET((float)1.5472961E38F, &PH) ;
    {
        float  q [] =  {-2.8612274E38F, -2.4108784E38F, -2.8450986E38F, -1.2720057E38F};
        p149_q_SET(&q, 0,  &PH) ;
    }
    p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, PH.base.pack) ;
    p149_position_valid_SET((uint8_t)(uint8_t)14, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ESTIMATOR_STATUS_230(), &PH);
    p230_time_usec_SET((uint64_t)6075124223066566808L, PH.base.pack) ;
    p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE, PH.base.pack) ;
    p230_vel_ratio_SET((float)3.9755764E37F, PH.base.pack) ;
    p230_pos_horiz_ratio_SET((float)2.297946E38F, PH.base.pack) ;
    p230_pos_vert_ratio_SET((float) -1.7743206E38F, PH.base.pack) ;
    p230_mag_ratio_SET((float) -1.5807328E38F, PH.base.pack) ;
    p230_hagl_ratio_SET((float)1.6581983E38F, PH.base.pack) ;
    p230_tas_ratio_SET((float)2.8067498E38F, PH.base.pack) ;
    p230_pos_horiz_accuracy_SET((float)2.2609794E38F, PH.base.pack) ;
    p230_pos_vert_accuracy_SET((float)2.4646533E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_WIND_COV_231(), &PH);
    p231_time_usec_SET((uint64_t)1279976347629248289L, PH.base.pack) ;
    p231_wind_x_SET((float)2.5121229E38F, PH.base.pack) ;
    p231_wind_y_SET((float) -4.959825E37F, PH.base.pack) ;
    p231_wind_z_SET((float) -2.1903554E38F, PH.base.pack) ;
    p231_var_horiz_SET((float) -1.7275796E38F, PH.base.pack) ;
    p231_var_vert_SET((float)3.5229E37F, PH.base.pack) ;
    p231_wind_alt_SET((float) -1.1317128E38F, PH.base.pack) ;
    p231_horiz_accuracy_SET((float)2.1202521E38F, PH.base.pack) ;
    p231_vert_accuracy_SET((float)3.0087648E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_INPUT_232(), &PH);
    p232_time_usec_SET((uint64_t)6306513146728592309L, PH.base.pack) ;
    p232_gps_id_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
    p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP, PH.base.pack) ;
    p232_time_week_ms_SET((uint32_t)2479364512L, PH.base.pack) ;
    p232_time_week_SET((uint16_t)(uint16_t)12268, PH.base.pack) ;
    p232_fix_type_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
    p232_lat_SET((int32_t)1986291853, PH.base.pack) ;
    p232_lon_SET((int32_t) -1247107394, PH.base.pack) ;
    p232_alt_SET((float) -2.7808212E38F, PH.base.pack) ;
    p232_hdop_SET((float) -1.6041172E38F, PH.base.pack) ;
    p232_vdop_SET((float) -3.8435181E37F, PH.base.pack) ;
    p232_vn_SET((float)1.5895784E38F, PH.base.pack) ;
    p232_ve_SET((float) -2.9729986E38F, PH.base.pack) ;
    p232_vd_SET((float)3.382673E38F, PH.base.pack) ;
    p232_speed_accuracy_SET((float) -1.1525197E38F, PH.base.pack) ;
    p232_horiz_accuracy_SET((float) -2.017128E38F, PH.base.pack) ;
    p232_vert_accuracy_SET((float)2.2350432E38F, PH.base.pack) ;
    p232_satellites_visible_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RTCM_DATA_233(), &PH);
    p233_flags_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
    p233_len_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)65, (uint8_t)197, (uint8_t)177, (uint8_t)186, (uint8_t)106, (uint8_t)77, (uint8_t)187, (uint8_t)132, (uint8_t)135, (uint8_t)37, (uint8_t)106, (uint8_t)19, (uint8_t)225, (uint8_t)208, (uint8_t)253, (uint8_t)122, (uint8_t)133, (uint8_t)164, (uint8_t)129, (uint8_t)110, (uint8_t)216, (uint8_t)99, (uint8_t)253, (uint8_t)227, (uint8_t)248, (uint8_t)216, (uint8_t)214, (uint8_t)19, (uint8_t)8, (uint8_t)48, (uint8_t)134, (uint8_t)195, (uint8_t)110, (uint8_t)119, (uint8_t)35, (uint8_t)158, (uint8_t)158, (uint8_t)85, (uint8_t)25, (uint8_t)213, (uint8_t)54, (uint8_t)247, (uint8_t)153, (uint8_t)242, (uint8_t)40, (uint8_t)106, (uint8_t)90, (uint8_t)96, (uint8_t)177, (uint8_t)196, (uint8_t)120, (uint8_t)170, (uint8_t)53, (uint8_t)175, (uint8_t)33, (uint8_t)142, (uint8_t)56, (uint8_t)135, (uint8_t)206, (uint8_t)40, (uint8_t)5, (uint8_t)60, (uint8_t)105, (uint8_t)216, (uint8_t)194, (uint8_t)174, (uint8_t)199, (uint8_t)136, (uint8_t)161, (uint8_t)210, (uint8_t)59, (uint8_t)97, (uint8_t)232, (uint8_t)162, (uint8_t)12, (uint8_t)83, (uint8_t)246, (uint8_t)96, (uint8_t)20, (uint8_t)73, (uint8_t)54, (uint8_t)106, (uint8_t)156, (uint8_t)146, (uint8_t)237, (uint8_t)133, (uint8_t)175, (uint8_t)233, (uint8_t)67, (uint8_t)54, (uint8_t)134, (uint8_t)191, (uint8_t)62, (uint8_t)39, (uint8_t)7, (uint8_t)57, (uint8_t)182, (uint8_t)116, (uint8_t)97, (uint8_t)240, (uint8_t)118, (uint8_t)85, (uint8_t)119, (uint8_t)4, (uint8_t)118, (uint8_t)91, (uint8_t)207, (uint8_t)58, (uint8_t)113, (uint8_t)123, (uint8_t)120, (uint8_t)29, (uint8_t)96, (uint8_t)253, (uint8_t)135, (uint8_t)188, (uint8_t)59, (uint8_t)154, (uint8_t)158, (uint8_t)176, (uint8_t)254, (uint8_t)132, (uint8_t)140, (uint8_t)246, (uint8_t)172, (uint8_t)23, (uint8_t)67, (uint8_t)113, (uint8_t)203, (uint8_t)136, (uint8_t)14, (uint8_t)24, (uint8_t)35, (uint8_t)116, (uint8_t)111, (uint8_t)162, (uint8_t)128, (uint8_t)234, (uint8_t)10, (uint8_t)118, (uint8_t)226, (uint8_t)213, (uint8_t)21, (uint8_t)180, (uint8_t)153, (uint8_t)1, (uint8_t)118, (uint8_t)178, (uint8_t)109, (uint8_t)189, (uint8_t)253, (uint8_t)50, (uint8_t)107, (uint8_t)31, (uint8_t)8, (uint8_t)49, (uint8_t)0, (uint8_t)207, (uint8_t)34, (uint8_t)25, (uint8_t)212, (uint8_t)218, (uint8_t)67, (uint8_t)44, (uint8_t)112, (uint8_t)148, (uint8_t)151, (uint8_t)217, (uint8_t)44, (uint8_t)127, (uint8_t)83, (uint8_t)128, (uint8_t)172, (uint8_t)213, (uint8_t)177, (uint8_t)115, (uint8_t)202, (uint8_t)83, (uint8_t)204, (uint8_t)242};
        p233_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIGH_LATENCY_234(), &PH);
    p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED, PH.base.pack) ;
    p234_custom_mode_SET((uint32_t)1548649566L, PH.base.pack) ;
    p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, PH.base.pack) ;
    p234_roll_SET((int16_t)(int16_t)27598, PH.base.pack) ;
    p234_pitch_SET((int16_t)(int16_t)442, PH.base.pack) ;
    p234_heading_SET((uint16_t)(uint16_t)44411, PH.base.pack) ;
    p234_throttle_SET((int8_t)(int8_t) -2, PH.base.pack) ;
    p234_heading_sp_SET((int16_t)(int16_t) -5512, PH.base.pack) ;
    p234_latitude_SET((int32_t)1696273668, PH.base.pack) ;
    p234_longitude_SET((int32_t) -1695180171, PH.base.pack) ;
    p234_altitude_amsl_SET((int16_t)(int16_t)29225, PH.base.pack) ;
    p234_altitude_sp_SET((int16_t)(int16_t) -2430, PH.base.pack) ;
    p234_airspeed_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
    p234_airspeed_sp_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
    p234_groundspeed_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
    p234_climb_rate_SET((int8_t)(int8_t) -110, PH.base.pack) ;
    p234_gps_nsat_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
    p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX, PH.base.pack) ;
    p234_battery_remaining_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
    p234_temperature_SET((int8_t)(int8_t) -123, PH.base.pack) ;
    p234_temperature_air_SET((int8_t)(int8_t) -13, PH.base.pack) ;
    p234_failsafe_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
    p234_wp_num_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
    p234_wp_distance_SET((uint16_t)(uint16_t)784, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VIBRATION_241(), &PH);
    p241_time_usec_SET((uint64_t)4489821708161053084L, PH.base.pack) ;
    p241_vibration_x_SET((float) -3.3762006E38F, PH.base.pack) ;
    p241_vibration_y_SET((float)2.1744358E38F, PH.base.pack) ;
    p241_vibration_z_SET((float)3.1551296E38F, PH.base.pack) ;
    p241_clipping_0_SET((uint32_t)3919677065L, PH.base.pack) ;
    p241_clipping_1_SET((uint32_t)2229292699L, PH.base.pack) ;
    p241_clipping_2_SET((uint32_t)3391236171L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HOME_POSITION_242(), &PH);
    p242_latitude_SET((int32_t) -569080200, PH.base.pack) ;
    p242_longitude_SET((int32_t) -1623078779, PH.base.pack) ;
    p242_altitude_SET((int32_t) -668892684, PH.base.pack) ;
    p242_x_SET((float) -2.6413717E38F, PH.base.pack) ;
    p242_y_SET((float)6.7106994E37F, PH.base.pack) ;
    p242_z_SET((float)2.0284837E38F, PH.base.pack) ;
    {
        float  q [] =  {3.3328002E37F, -2.7218375E38F, -2.7264935E38F, -2.9419536E38F};
        p242_q_SET(&q, 0, &PH.base.pack) ;
    }
    p242_approach_x_SET((float)1.8337059E38F, PH.base.pack) ;
    p242_approach_y_SET((float)1.2662743E38F, PH.base.pack) ;
    p242_approach_z_SET((float)1.985588E38F, PH.base.pack) ;
    p242_time_usec_SET((uint64_t)4988332789114925411L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_HOME_POSITION_243(), &PH);
    p243_target_system_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
    p243_latitude_SET((int32_t) -492986649, PH.base.pack) ;
    p243_longitude_SET((int32_t)984181354, PH.base.pack) ;
    p243_altitude_SET((int32_t) -1696402584, PH.base.pack) ;
    p243_x_SET((float) -4.0993823E37F, PH.base.pack) ;
    p243_y_SET((float) -2.3263027E38F, PH.base.pack) ;
    p243_z_SET((float) -1.079366E38F, PH.base.pack) ;
    {
        float  q [] =  {2.5943183E38F, 8.783671E37F, 2.6254366E38F, 1.4344571E38F};
        p243_q_SET(&q, 0, &PH.base.pack) ;
    }
    p243_approach_x_SET((float)2.4007073E38F, PH.base.pack) ;
    p243_approach_y_SET((float)8.808641E36F, PH.base.pack) ;
    p243_approach_z_SET((float) -1.8735476E38F, PH.base.pack) ;
    p243_time_usec_SET((uint64_t)3380594850452376002L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MESSAGE_INTERVAL_244(), &PH);
    p244_message_id_SET((uint16_t)(uint16_t)61249, PH.base.pack) ;
    p244_interval_us_SET((int32_t)1821395833, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_EXTENDED_SYS_STATE_245(), &PH);
    p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW, PH.base.pack) ;
    p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ADSB_VEHICLE_246(), &PH);
    p246_ICAO_address_SET((uint32_t)3400275762L, PH.base.pack) ;
    p246_lat_SET((int32_t) -1370819526, PH.base.pack) ;
    p246_lon_SET((int32_t) -1296119818, PH.base.pack) ;
    p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, PH.base.pack) ;
    p246_altitude_SET((int32_t) -102549543, PH.base.pack) ;
    p246_heading_SET((uint16_t)(uint16_t)57390, PH.base.pack) ;
    p246_hor_velocity_SET((uint16_t)(uint16_t)20317, PH.base.pack) ;
    p246_ver_velocity_SET((int16_t)(int16_t)24578, PH.base.pack) ;
    {
        char16_t   callsign = "yiUul";
        p246_callsign_SET(&callsign, 0,  sizeof(callsign), &PH) ;
    }
    p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UNASSGINED3, PH.base.pack) ;
    p246_tslc_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED, PH.base.pack) ;
    p246_squawk_SET((uint16_t)(uint16_t)31063, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COLLISION_247(), &PH);
    p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, PH.base.pack) ;
    p247_id_SET((uint32_t)4028228392L, PH.base.pack) ;
    p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_PERPENDICULAR, PH.base.pack) ;
    p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW, PH.base.pack) ;
    p247_time_to_minimum_delta_SET((float) -3.494439E37F, PH.base.pack) ;
    p247_altitude_minimum_delta_SET((float) -1.1428649E38F, PH.base.pack) ;
    p247_horizontal_minimum_delta_SET((float)1.1687071E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_V2_EXTENSION_248(), &PH);
    p248_target_network_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
    p248_target_system_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
    p248_target_component_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
    p248_message_type_SET((uint16_t)(uint16_t)21853, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)127, (uint8_t)169, (uint8_t)19, (uint8_t)134, (uint8_t)3, (uint8_t)230, (uint8_t)39, (uint8_t)209, (uint8_t)87, (uint8_t)215, (uint8_t)17, (uint8_t)183, (uint8_t)151, (uint8_t)167, (uint8_t)180, (uint8_t)106, (uint8_t)217, (uint8_t)155, (uint8_t)21, (uint8_t)243, (uint8_t)68, (uint8_t)111, (uint8_t)118, (uint8_t)131, (uint8_t)74, (uint8_t)2, (uint8_t)55, (uint8_t)102, (uint8_t)194, (uint8_t)185, (uint8_t)255, (uint8_t)33, (uint8_t)161, (uint8_t)9, (uint8_t)18, (uint8_t)82, (uint8_t)145, (uint8_t)110, (uint8_t)77, (uint8_t)147, (uint8_t)57, (uint8_t)17, (uint8_t)152, (uint8_t)183, (uint8_t)169, (uint8_t)186, (uint8_t)84, (uint8_t)119, (uint8_t)75, (uint8_t)154, (uint8_t)112, (uint8_t)219, (uint8_t)98, (uint8_t)10, (uint8_t)47, (uint8_t)205, (uint8_t)255, (uint8_t)102, (uint8_t)45, (uint8_t)195, (uint8_t)68, (uint8_t)98, (uint8_t)232, (uint8_t)170, (uint8_t)253, (uint8_t)224, (uint8_t)78, (uint8_t)31, (uint8_t)17, (uint8_t)139, (uint8_t)147, (uint8_t)71, (uint8_t)205, (uint8_t)164, (uint8_t)102, (uint8_t)181, (uint8_t)194, (uint8_t)19, (uint8_t)112, (uint8_t)125, (uint8_t)11, (uint8_t)25, (uint8_t)253, (uint8_t)141, (uint8_t)128, (uint8_t)159, (uint8_t)219, (uint8_t)245, (uint8_t)178, (uint8_t)81, (uint8_t)248, (uint8_t)93, (uint8_t)133, (uint8_t)249, (uint8_t)25, (uint8_t)103, (uint8_t)27, (uint8_t)30, (uint8_t)59, (uint8_t)179, (uint8_t)248, (uint8_t)219, (uint8_t)128, (uint8_t)182, (uint8_t)128, (uint8_t)137, (uint8_t)220, (uint8_t)244, (uint8_t)171, (uint8_t)173, (uint8_t)160, (uint8_t)27, (uint8_t)170, (uint8_t)178, (uint8_t)217, (uint8_t)59, (uint8_t)199, (uint8_t)124, (uint8_t)34, (uint8_t)132, (uint8_t)60, (uint8_t)122, (uint8_t)212, (uint8_t)7, (uint8_t)192, (uint8_t)61, (uint8_t)125, (uint8_t)239, (uint8_t)134, (uint8_t)168, (uint8_t)33, (uint8_t)159, (uint8_t)156, (uint8_t)130, (uint8_t)80, (uint8_t)10, (uint8_t)195, (uint8_t)211, (uint8_t)246, (uint8_t)172, (uint8_t)236, (uint8_t)24, (uint8_t)45, (uint8_t)228, (uint8_t)46, (uint8_t)142, (uint8_t)95, (uint8_t)106, (uint8_t)94, (uint8_t)102, (uint8_t)99, (uint8_t)214, (uint8_t)76, (uint8_t)35, (uint8_t)112, (uint8_t)45, (uint8_t)64, (uint8_t)184, (uint8_t)103, (uint8_t)81, (uint8_t)139, (uint8_t)47, (uint8_t)236, (uint8_t)200, (uint8_t)213, (uint8_t)207, (uint8_t)130, (uint8_t)87, (uint8_t)91, (uint8_t)247, (uint8_t)150, (uint8_t)142, (uint8_t)241, (uint8_t)18, (uint8_t)161, (uint8_t)140, (uint8_t)103, (uint8_t)43, (uint8_t)203, (uint8_t)89, (uint8_t)150, (uint8_t)150, (uint8_t)165, (uint8_t)207, (uint8_t)168, (uint8_t)246, (uint8_t)82, (uint8_t)237, (uint8_t)193, (uint8_t)57, (uint8_t)100, (uint8_t)8, (uint8_t)21, (uint8_t)84, (uint8_t)9, (uint8_t)160, (uint8_t)86, (uint8_t)199, (uint8_t)129, (uint8_t)79, (uint8_t)244, (uint8_t)184, (uint8_t)217, (uint8_t)235, (uint8_t)9, (uint8_t)51, (uint8_t)7, (uint8_t)25, (uint8_t)23, (uint8_t)71, (uint8_t)211, (uint8_t)196, (uint8_t)127, (uint8_t)196, (uint8_t)63, (uint8_t)88, (uint8_t)106, (uint8_t)101, (uint8_t)151, (uint8_t)97, (uint8_t)214, (uint8_t)194, (uint8_t)22, (uint8_t)253, (uint8_t)31, (uint8_t)63, (uint8_t)46, (uint8_t)192, (uint8_t)236, (uint8_t)57, (uint8_t)66, (uint8_t)10, (uint8_t)17, (uint8_t)246, (uint8_t)223, (uint8_t)27, (uint8_t)30, (uint8_t)236, (uint8_t)12, (uint8_t)173, (uint8_t)130, (uint8_t)23, (uint8_t)244, (uint8_t)217, (uint8_t)110, (uint8_t)246, (uint8_t)162, (uint8_t)54, (uint8_t)11};
        p248_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MEMORY_VECT_249(), &PH);
    p249_address_SET((uint16_t)(uint16_t)10379, PH.base.pack) ;
    p249_ver_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
    p249_type_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
    {
        int8_t  value [] =  {(int8_t)94, (int8_t)19, (int8_t) -4, (int8_t)99, (int8_t)77, (int8_t) -83, (int8_t)58, (int8_t) -109, (int8_t) -73, (int8_t)50, (int8_t) -55, (int8_t)88, (int8_t)84, (int8_t) -66, (int8_t) -98, (int8_t) -6, (int8_t)57, (int8_t) -93, (int8_t) -66, (int8_t)42, (int8_t)98, (int8_t) -120, (int8_t)69, (int8_t)43, (int8_t) -103, (int8_t) -106, (int8_t) -125, (int8_t)86, (int8_t)24, (int8_t)86, (int8_t)36, (int8_t) -73};
        p249_value_SET(&value, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEBUG_VECT_250(), &PH);
    {
        char16_t   name = "fAos";
        p250_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p250_time_usec_SET((uint64_t)4050651370012005865L, PH.base.pack) ;
    p250_x_SET((float)1.4587519E38F, PH.base.pack) ;
    p250_y_SET((float)2.275545E38F, PH.base.pack) ;
    p250_z_SET((float) -3.2367298E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
    p251_time_boot_ms_SET((uint32_t)448483884L, PH.base.pack) ;
    {
        char16_t   name = "epyPnrb";
        p251_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p251_value_SET((float)2.9553256E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_INT_252(), &PH);
    p252_time_boot_ms_SET((uint32_t)2694909121L, PH.base.pack) ;
    {
        char16_t   name = "kh";
        p252_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p252_value_SET((int32_t)542487852, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_STATUSTEXT_253(), &PH);
    p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_ALERT, PH.base.pack) ;
    {
        char16_t   text = "ndihdhtQrHevldlacagipqtdhynblnlbftfsggqFauxnd";
        p253_text_SET(&text, 0,  sizeof(text), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEBUG_254(), &PH);
    p254_time_boot_ms_SET((uint32_t)3852867679L, PH.base.pack) ;
    p254_ind_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
    p254_value_SET((float) -1.3796155E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SETUP_SIGNING_256(), &PH);
    p256_target_system_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
    p256_target_component_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
    {
        uint8_t  secret_key [] =  {(uint8_t)157, (uint8_t)136, (uint8_t)135, (uint8_t)39, (uint8_t)6, (uint8_t)145, (uint8_t)177, (uint8_t)206, (uint8_t)78, (uint8_t)10, (uint8_t)46, (uint8_t)237, (uint8_t)164, (uint8_t)241, (uint8_t)50, (uint8_t)222, (uint8_t)183, (uint8_t)207, (uint8_t)167, (uint8_t)183, (uint8_t)141, (uint8_t)36, (uint8_t)203, (uint8_t)72, (uint8_t)190, (uint8_t)15, (uint8_t)78, (uint8_t)117, (uint8_t)232, (uint8_t)52, (uint8_t)226, (uint8_t)239};
        p256_secret_key_SET(&secret_key, 0, &PH.base.pack) ;
    }
    p256_initial_timestamp_SET((uint64_t)8974793790281174340L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_BUTTON_CHANGE_257(), &PH);
    p257_time_boot_ms_SET((uint32_t)625158912L, PH.base.pack) ;
    p257_last_change_ms_SET((uint32_t)3573677844L, PH.base.pack) ;
    p257_state_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PLAY_TUNE_258(), &PH);
    p258_target_system_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
    p258_target_component_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
    {
        char16_t   tune = "qjpntjlfr";
        p258_tune_SET(&tune, 0,  sizeof(tune), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_INFORMATION_259(), &PH);
    p259_time_boot_ms_SET((uint32_t)3069726193L, PH.base.pack) ;
    {
        uint8_t  vendor_name [] =  {(uint8_t)152, (uint8_t)62, (uint8_t)75, (uint8_t)133, (uint8_t)122, (uint8_t)26, (uint8_t)250, (uint8_t)39, (uint8_t)246, (uint8_t)207, (uint8_t)113, (uint8_t)142, (uint8_t)30, (uint8_t)22, (uint8_t)180, (uint8_t)40, (uint8_t)113, (uint8_t)76, (uint8_t)49, (uint8_t)30, (uint8_t)95, (uint8_t)162, (uint8_t)144, (uint8_t)130, (uint8_t)222, (uint8_t)38, (uint8_t)9, (uint8_t)222, (uint8_t)221, (uint8_t)114, (uint8_t)214, (uint8_t)194};
        p259_vendor_name_SET(&vendor_name, 0, &PH.base.pack) ;
    }
    {
        uint8_t  model_name [] =  {(uint8_t)134, (uint8_t)210, (uint8_t)24, (uint8_t)192, (uint8_t)85, (uint8_t)20, (uint8_t)70, (uint8_t)85, (uint8_t)242, (uint8_t)207, (uint8_t)223, (uint8_t)37, (uint8_t)195, (uint8_t)247, (uint8_t)141, (uint8_t)116, (uint8_t)68, (uint8_t)216, (uint8_t)75, (uint8_t)79, (uint8_t)151, (uint8_t)40, (uint8_t)106, (uint8_t)222, (uint8_t)158, (uint8_t)249, (uint8_t)150, (uint8_t)89, (uint8_t)212, (uint8_t)217, (uint8_t)96, (uint8_t)164};
        p259_model_name_SET(&model_name, 0, &PH.base.pack) ;
    }
    p259_firmware_version_SET((uint32_t)785228079L, PH.base.pack) ;
    p259_focal_length_SET((float)2.7248133E38F, PH.base.pack) ;
    p259_sensor_size_h_SET((float)7.2224697E37F, PH.base.pack) ;
    p259_sensor_size_v_SET((float)9.237446E37F, PH.base.pack) ;
    p259_resolution_h_SET((uint16_t)(uint16_t)15157, PH.base.pack) ;
    p259_resolution_v_SET((uint16_t)(uint16_t)61916, PH.base.pack) ;
    p259_lens_id_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
    p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO, PH.base.pack) ;
    p259_cam_definition_version_SET((uint16_t)(uint16_t)16571, PH.base.pack) ;
    {
        char16_t   cam_definition_uri = "swCpbgupibzayhYonyvbzdfppbay";
        p259_cam_definition_uri_SET(&cam_definition_uri, 0,  sizeof(cam_definition_uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_SETTINGS_260(), &PH);
    p260_time_boot_ms_SET((uint32_t)718205736L, PH.base.pack) ;
    p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_STORAGE_INFORMATION_261(), &PH);
    p261_time_boot_ms_SET((uint32_t)4017929410L, PH.base.pack) ;
    p261_storage_id_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    p261_storage_count_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
    p261_status_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
    p261_total_capacity_SET((float)2.2158017E38F, PH.base.pack) ;
    p261_used_capacity_SET((float)1.2235626E38F, PH.base.pack) ;
    p261_available_capacity_SET((float) -2.383429E38F, PH.base.pack) ;
    p261_read_speed_SET((float)3.3263146E38F, PH.base.pack) ;
    p261_write_speed_SET((float) -2.8343235E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
    p262_time_boot_ms_SET((uint32_t)1143126264L, PH.base.pack) ;
    p262_image_status_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
    p262_video_status_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
    p262_image_interval_SET((float) -2.1055626E38F, PH.base.pack) ;
    p262_recording_time_ms_SET((uint32_t)717266392L, PH.base.pack) ;
    p262_available_capacity_SET((float) -5.7807494E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
    p263_time_boot_ms_SET((uint32_t)954711509L, PH.base.pack) ;
    p263_time_utc_SET((uint64_t)7382735882685224215L, PH.base.pack) ;
    p263_camera_id_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
    p263_lat_SET((int32_t) -267611027, PH.base.pack) ;
    p263_lon_SET((int32_t) -1345010032, PH.base.pack) ;
    p263_alt_SET((int32_t)2107882059, PH.base.pack) ;
    p263_relative_alt_SET((int32_t)1805960813, PH.base.pack) ;
    {
        float  q [] =  {-2.0716246E38F, 1.6713103E38F, 8.983599E37F, -1.8528987E38F};
        p263_q_SET(&q, 0, &PH.base.pack) ;
    }
    p263_image_index_SET((int32_t) -1395544141, PH.base.pack) ;
    p263_capture_result_SET((int8_t)(int8_t)104, PH.base.pack) ;
    {
        char16_t   file_url = "rkjgGqjqeinhkqVsYpgrfkycykyWjjxQbueueovsoejanhvcgasYtdiwrhssjwbmfcsIp";
        p263_file_url_SET(&file_url, 0,  sizeof(file_url), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FLIGHT_INFORMATION_264(), &PH);
    p264_time_boot_ms_SET((uint32_t)1732971225L, PH.base.pack) ;
    p264_arming_time_utc_SET((uint64_t)6345052211884329476L, PH.base.pack) ;
    p264_takeoff_time_utc_SET((uint64_t)3910001477371680702L, PH.base.pack) ;
    p264_flight_uuid_SET((uint64_t)5038693174887389362L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MOUNT_ORIENTATION_265(), &PH);
    p265_time_boot_ms_SET((uint32_t)535492549L, PH.base.pack) ;
    p265_roll_SET((float)1.0595802E38F, PH.base.pack) ;
    p265_pitch_SET((float) -2.5747736E38F, PH.base.pack) ;
    p265_yaw_SET((float) -3.0888106E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_266(), &PH);
    p266_target_system_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
    p266_target_component_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
    p266_sequence_SET((uint16_t)(uint16_t)16472, PH.base.pack) ;
    p266_length_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
    p266_first_message_offset_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)123, (uint8_t)124, (uint8_t)138, (uint8_t)94, (uint8_t)81, (uint8_t)53, (uint8_t)77, (uint8_t)6, (uint8_t)100, (uint8_t)105, (uint8_t)94, (uint8_t)2, (uint8_t)196, (uint8_t)218, (uint8_t)92, (uint8_t)242, (uint8_t)42, (uint8_t)202, (uint8_t)170, (uint8_t)213, (uint8_t)148, (uint8_t)52, (uint8_t)218, (uint8_t)109, (uint8_t)209, (uint8_t)181, (uint8_t)205, (uint8_t)139, (uint8_t)139, (uint8_t)227, (uint8_t)172, (uint8_t)225, (uint8_t)140, (uint8_t)112, (uint8_t)146, (uint8_t)253, (uint8_t)245, (uint8_t)251, (uint8_t)132, (uint8_t)128, (uint8_t)213, (uint8_t)136, (uint8_t)185, (uint8_t)34, (uint8_t)87, (uint8_t)188, (uint8_t)231, (uint8_t)99, (uint8_t)91, (uint8_t)146, (uint8_t)179, (uint8_t)205, (uint8_t)38, (uint8_t)34, (uint8_t)67, (uint8_t)177, (uint8_t)128, (uint8_t)127, (uint8_t)228, (uint8_t)83, (uint8_t)72, (uint8_t)54, (uint8_t)17, (uint8_t)220, (uint8_t)74, (uint8_t)151, (uint8_t)157, (uint8_t)50, (uint8_t)54, (uint8_t)27, (uint8_t)253, (uint8_t)1, (uint8_t)205, (uint8_t)28, (uint8_t)43, (uint8_t)217, (uint8_t)83, (uint8_t)225, (uint8_t)145, (uint8_t)185, (uint8_t)138, (uint8_t)154, (uint8_t)115, (uint8_t)67, (uint8_t)154, (uint8_t)41, (uint8_t)252, (uint8_t)130, (uint8_t)207, (uint8_t)157, (uint8_t)152, (uint8_t)213, (uint8_t)92, (uint8_t)190, (uint8_t)16, (uint8_t)192, (uint8_t)80, (uint8_t)27, (uint8_t)56, (uint8_t)110, (uint8_t)7, (uint8_t)100, (uint8_t)160, (uint8_t)105, (uint8_t)37, (uint8_t)12, (uint8_t)38, (uint8_t)53, (uint8_t)209, (uint8_t)20, (uint8_t)53, (uint8_t)183, (uint8_t)49, (uint8_t)230, (uint8_t)212, (uint8_t)192, (uint8_t)2, (uint8_t)35, (uint8_t)58, (uint8_t)24, (uint8_t)117, (uint8_t)203, (uint8_t)72, (uint8_t)30, (uint8_t)51, (uint8_t)87, (uint8_t)237, (uint8_t)186, (uint8_t)47, (uint8_t)1, (uint8_t)7, (uint8_t)208, (uint8_t)150, (uint8_t)69, (uint8_t)129, (uint8_t)136, (uint8_t)94, (uint8_t)64, (uint8_t)241, (uint8_t)59, (uint8_t)63, (uint8_t)42, (uint8_t)61, (uint8_t)10, (uint8_t)201, (uint8_t)130, (uint8_t)62, (uint8_t)89, (uint8_t)10, (uint8_t)148, (uint8_t)146, (uint8_t)28, (uint8_t)231, (uint8_t)71, (uint8_t)158, (uint8_t)127, (uint8_t)88, (uint8_t)249, (uint8_t)252, (uint8_t)24, (uint8_t)232, (uint8_t)9, (uint8_t)66, (uint8_t)39, (uint8_t)20, (uint8_t)233, (uint8_t)239, (uint8_t)64, (uint8_t)108, (uint8_t)54, (uint8_t)98, (uint8_t)177, (uint8_t)216, (uint8_t)165, (uint8_t)214, (uint8_t)148, (uint8_t)51, (uint8_t)150, (uint8_t)142, (uint8_t)37, (uint8_t)110, (uint8_t)242, (uint8_t)207, (uint8_t)169, (uint8_t)121, (uint8_t)36, (uint8_t)227, (uint8_t)80, (uint8_t)218, (uint8_t)250, (uint8_t)59, (uint8_t)0, (uint8_t)7, (uint8_t)213, (uint8_t)81, (uint8_t)1, (uint8_t)35, (uint8_t)29, (uint8_t)202, (uint8_t)161, (uint8_t)29, (uint8_t)110, (uint8_t)46, (uint8_t)165, (uint8_t)29, (uint8_t)206, (uint8_t)195, (uint8_t)139, (uint8_t)244, (uint8_t)235, (uint8_t)9, (uint8_t)147, (uint8_t)107, (uint8_t)163, (uint8_t)112, (uint8_t)86, (uint8_t)88, (uint8_t)205, (uint8_t)175, (uint8_t)25, (uint8_t)45, (uint8_t)35, (uint8_t)92, (uint8_t)84, (uint8_t)64, (uint8_t)90, (uint8_t)186, (uint8_t)69, (uint8_t)225, (uint8_t)46, (uint8_t)60, (uint8_t)199, (uint8_t)34, (uint8_t)94, (uint8_t)34, (uint8_t)127, (uint8_t)10, (uint8_t)142, (uint8_t)195, (uint8_t)194, (uint8_t)253, (uint8_t)219, (uint8_t)165, (uint8_t)33, (uint8_t)82, (uint8_t)132, (uint8_t)14, (uint8_t)52, (uint8_t)175};
        p266_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_ACKED_267(), &PH);
    p267_target_system_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
    p267_target_component_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
    p267_sequence_SET((uint16_t)(uint16_t)3969, PH.base.pack) ;
    p267_length_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
    p267_first_message_offset_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)232, (uint8_t)89, (uint8_t)60, (uint8_t)94, (uint8_t)213, (uint8_t)86, (uint8_t)176, (uint8_t)42, (uint8_t)35, (uint8_t)153, (uint8_t)247, (uint8_t)57, (uint8_t)16, (uint8_t)99, (uint8_t)25, (uint8_t)45, (uint8_t)31, (uint8_t)4, (uint8_t)214, (uint8_t)225, (uint8_t)107, (uint8_t)100, (uint8_t)109, (uint8_t)124, (uint8_t)29, (uint8_t)44, (uint8_t)132, (uint8_t)88, (uint8_t)78, (uint8_t)29, (uint8_t)111, (uint8_t)60, (uint8_t)239, (uint8_t)241, (uint8_t)158, (uint8_t)210, (uint8_t)5, (uint8_t)83, (uint8_t)75, (uint8_t)166, (uint8_t)85, (uint8_t)194, (uint8_t)90, (uint8_t)82, (uint8_t)153, (uint8_t)144, (uint8_t)165, (uint8_t)18, (uint8_t)252, (uint8_t)8, (uint8_t)226, (uint8_t)72, (uint8_t)214, (uint8_t)98, (uint8_t)99, (uint8_t)41, (uint8_t)148, (uint8_t)51, (uint8_t)93, (uint8_t)75, (uint8_t)243, (uint8_t)52, (uint8_t)212, (uint8_t)204, (uint8_t)45, (uint8_t)238, (uint8_t)236, (uint8_t)32, (uint8_t)10, (uint8_t)63, (uint8_t)87, (uint8_t)117, (uint8_t)36, (uint8_t)158, (uint8_t)85, (uint8_t)151, (uint8_t)235, (uint8_t)96, (uint8_t)222, (uint8_t)250, (uint8_t)151, (uint8_t)229, (uint8_t)188, (uint8_t)37, (uint8_t)21, (uint8_t)125, (uint8_t)221, (uint8_t)86, (uint8_t)184, (uint8_t)235, (uint8_t)198, (uint8_t)133, (uint8_t)202, (uint8_t)183, (uint8_t)111, (uint8_t)122, (uint8_t)113, (uint8_t)158, (uint8_t)87, (uint8_t)108, (uint8_t)17, (uint8_t)67, (uint8_t)254, (uint8_t)24, (uint8_t)235, (uint8_t)58, (uint8_t)169, (uint8_t)4, (uint8_t)34, (uint8_t)101, (uint8_t)189, (uint8_t)22, (uint8_t)247, (uint8_t)77, (uint8_t)189, (uint8_t)194, (uint8_t)76, (uint8_t)163, (uint8_t)125, (uint8_t)90, (uint8_t)254, (uint8_t)18, (uint8_t)195, (uint8_t)146, (uint8_t)179, (uint8_t)50, (uint8_t)6, (uint8_t)60, (uint8_t)27, (uint8_t)134, (uint8_t)148, (uint8_t)0, (uint8_t)82, (uint8_t)209, (uint8_t)25, (uint8_t)135, (uint8_t)186, (uint8_t)195, (uint8_t)59, (uint8_t)100, (uint8_t)20, (uint8_t)214, (uint8_t)223, (uint8_t)96, (uint8_t)165, (uint8_t)250, (uint8_t)142, (uint8_t)82, (uint8_t)48, (uint8_t)142, (uint8_t)169, (uint8_t)213, (uint8_t)28, (uint8_t)167, (uint8_t)172, (uint8_t)167, (uint8_t)172, (uint8_t)109, (uint8_t)214, (uint8_t)44, (uint8_t)1, (uint8_t)80, (uint8_t)225, (uint8_t)86, (uint8_t)212, (uint8_t)100, (uint8_t)193, (uint8_t)179, (uint8_t)42, (uint8_t)106, (uint8_t)235, (uint8_t)57, (uint8_t)121, (uint8_t)142, (uint8_t)192, (uint8_t)210, (uint8_t)69, (uint8_t)172, (uint8_t)183, (uint8_t)1, (uint8_t)144, (uint8_t)161, (uint8_t)213, (uint8_t)51, (uint8_t)52, (uint8_t)92, (uint8_t)173, (uint8_t)89, (uint8_t)142, (uint8_t)145, (uint8_t)122, (uint8_t)24, (uint8_t)86, (uint8_t)55, (uint8_t)107, (uint8_t)153, (uint8_t)134, (uint8_t)133, (uint8_t)57, (uint8_t)202, (uint8_t)59, (uint8_t)136, (uint8_t)24, (uint8_t)22, (uint8_t)154, (uint8_t)218, (uint8_t)171, (uint8_t)150, (uint8_t)12, (uint8_t)131, (uint8_t)40, (uint8_t)107, (uint8_t)159, (uint8_t)103, (uint8_t)119, (uint8_t)166, (uint8_t)96, (uint8_t)243, (uint8_t)55, (uint8_t)115, (uint8_t)196, (uint8_t)137, (uint8_t)159, (uint8_t)231, (uint8_t)34, (uint8_t)206, (uint8_t)26, (uint8_t)150, (uint8_t)131, (uint8_t)104, (uint8_t)73, (uint8_t)230, (uint8_t)242, (uint8_t)165, (uint8_t)114, (uint8_t)114, (uint8_t)175, (uint8_t)185, (uint8_t)241, (uint8_t)130, (uint8_t)86, (uint8_t)3, (uint8_t)150, (uint8_t)132, (uint8_t)90, (uint8_t)165, (uint8_t)102, (uint8_t)179, (uint8_t)89};
        p267_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_ACK_268(), &PH);
    p268_target_system_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
    p268_target_component_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
    p268_sequence_SET((uint16_t)(uint16_t)3471, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
    p269_camera_id_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
    p269_status_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
    p269_framerate_SET((float)2.1459716E38F, PH.base.pack) ;
    p269_resolution_h_SET((uint16_t)(uint16_t)25042, PH.base.pack) ;
    p269_resolution_v_SET((uint16_t)(uint16_t)36680, PH.base.pack) ;
    p269_bitrate_SET((uint32_t)973706674L, PH.base.pack) ;
    p269_rotation_SET((uint16_t)(uint16_t)48884, PH.base.pack) ;
    {
        char16_t   uri = "wjgexvwpxBtccwujrriejhfcxmJzjCrjtkbxGuwokkvitsqnaNaeEoeawifdlKVfstlfpoKddAnpPqvqaolaDTrcdvwnhnismqflnghpthnsjwgcsStfacbnqKvngTmbldaidprjnzigzgbxypammqQvfDYgfvyarEvaNphGaJfznmmgzdxnotvbzxPtzoeydnfsndqVzqpnukvGeGRsezqsbqCbg";
        p269_uri_SET(&uri, 0,  sizeof(uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
    p270_target_system_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
    p270_target_component_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
    p270_camera_id_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
    p270_framerate_SET((float)2.5221026E38F, PH.base.pack) ;
    p270_resolution_h_SET((uint16_t)(uint16_t)4823, PH.base.pack) ;
    p270_resolution_v_SET((uint16_t)(uint16_t)15398, PH.base.pack) ;
    p270_bitrate_SET((uint32_t)3052803815L, PH.base.pack) ;
    p270_rotation_SET((uint16_t)(uint16_t)32337, PH.base.pack) ;
    {
        char16_t   uri = "qtonJlnztluohSxzkuuqrehftjqxarfdlmfjpkaQntxqslPlCwrkarKjdzOpujyugmZnuioktbretmejlAvsBrkohxYkdpnKvlbvMOibidTjeiykfpjhTvgLQrgazyfekkdNrpqviygaeszlcjxxmhfxewnxulgcqjlgwncknvumhkdqhubvwzzelVjyydczDhljujCavnjpjltfZzqfrytmrDykhvrG";
        p270_uri_SET(&uri, 0,  sizeof(uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_WIFI_CONFIG_AP_299(), &PH);
    {
        char16_t   ssid = "umomflbzesldcgjcsqwzjdqbux";
        p299_ssid_SET(&ssid, 0,  sizeof(ssid), &PH) ;
    }
    {
        char16_t   password = "lqISymarpygqvMhd";
        p299_password_SET(&password, 0,  sizeof(password), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PROTOCOL_VERSION_300(), &PH);
    p300_version_SET((uint16_t)(uint16_t)63978, PH.base.pack) ;
    p300_min_version_SET((uint16_t)(uint16_t)4970, PH.base.pack) ;
    p300_max_version_SET((uint16_t)(uint16_t)37947, PH.base.pack) ;
    {
        uint8_t  spec_version_hash [] =  {(uint8_t)234, (uint8_t)2, (uint8_t)13, (uint8_t)139, (uint8_t)174, (uint8_t)98, (uint8_t)253, (uint8_t)145};
        p300_spec_version_hash_SET(&spec_version_hash, 0, &PH.base.pack) ;
    }
    {
        uint8_t  library_version_hash [] =  {(uint8_t)38, (uint8_t)226, (uint8_t)94, (uint8_t)108, (uint8_t)48, (uint8_t)94, (uint8_t)173, (uint8_t)45};
        p300_library_version_hash_SET(&library_version_hash, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_STATUS_310(), &PH);
    p310_time_usec_SET((uint64_t)6174615944857833449L, PH.base.pack) ;
    p310_uptime_sec_SET((uint32_t)977282552L, PH.base.pack) ;
    p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_WARNING, PH.base.pack) ;
    p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL, PH.base.pack) ;
    p310_sub_mode_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
    p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)7872, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_INFO_311(), &PH);
    p311_time_usec_SET((uint64_t)8866394533102405191L, PH.base.pack) ;
    p311_uptime_sec_SET((uint32_t)2723714140L, PH.base.pack) ;
    {
        char16_t   name = "izidezEclEIjltqzdtVawkocEnlotjjteqrIlQfyxCzlxjzvpeczJupPjSwtyr";
        p311_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p311_hw_version_major_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
    p311_hw_version_minor_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
    {
        uint8_t  hw_unique_id [] =  {(uint8_t)72, (uint8_t)6, (uint8_t)99, (uint8_t)111, (uint8_t)124, (uint8_t)166, (uint8_t)166, (uint8_t)85, (uint8_t)198, (uint8_t)101, (uint8_t)38, (uint8_t)88, (uint8_t)214, (uint8_t)224, (uint8_t)161, (uint8_t)153};
        p311_hw_unique_id_SET(&hw_unique_id, 0, &PH.base.pack) ;
    }
    p311_sw_version_major_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
    p311_sw_version_minor_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
    p311_sw_vcs_commit_SET((uint32_t)462442049L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
    p320_target_system_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
    p320_target_component_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
    {
        char16_t   param_id = "udxcxaniQS";
        p320_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p320_param_index_SET((int16_t)(int16_t)19543, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
    p321_target_system_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
    p321_target_component_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_VALUE_322(), &PH);
    {
        char16_t   param_id = "zwpzglo";
        p322_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "mQnhgqmohbVFzgingthtghdyicyixwih";
        p322_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT64, PH.base.pack) ;
    p322_param_count_SET((uint16_t)(uint16_t)46022, PH.base.pack) ;
    p322_param_index_SET((uint16_t)(uint16_t)17966, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_SET_323(), &PH);
    p323_target_system_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
    p323_target_component_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
    {
        char16_t   param_id = "ocqagfEjiLiphw";
        p323_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "stbhjqwiyOddiHloTtskazZuoohqnhrxvrnmnhogvssKUibakkvFojpUkgmnichffdIrkphyofhwyejQbuXtxmTiHtzX";
        p323_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT64, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_ACK_324(), &PH);
    {
        char16_t   param_id = "Uvhootyyug";
        p324_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "azjtpemrnmbfeollhyioUhtozqgllsH";
        p324_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32, PH.base.pack) ;
    p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_VALUE_UNSUPPORTED, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OBSTACLE_DISTANCE_330(), &PH);
    p330_time_usec_SET((uint64_t)3392600730210012512L, PH.base.pack) ;
    p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
    {
        uint16_t  distances [] =  {(uint16_t)9167, (uint16_t)17029, (uint16_t)24123, (uint16_t)57060, (uint16_t)7666, (uint16_t)55553, (uint16_t)56766, (uint16_t)41138, (uint16_t)22179, (uint16_t)56949, (uint16_t)25842, (uint16_t)41976, (uint16_t)20057, (uint16_t)54291, (uint16_t)53502, (uint16_t)9935, (uint16_t)24506, (uint16_t)12897, (uint16_t)27346, (uint16_t)42113, (uint16_t)51558, (uint16_t)39709, (uint16_t)40063, (uint16_t)9929, (uint16_t)40625, (uint16_t)47859, (uint16_t)6299, (uint16_t)27428, (uint16_t)31607, (uint16_t)50245, (uint16_t)8147, (uint16_t)31199, (uint16_t)40718, (uint16_t)16228, (uint16_t)63675, (uint16_t)52933, (uint16_t)6909, (uint16_t)31405, (uint16_t)38196, (uint16_t)48909, (uint16_t)60266, (uint16_t)40224, (uint16_t)29164, (uint16_t)14802, (uint16_t)52929, (uint16_t)46997, (uint16_t)27120, (uint16_t)47111, (uint16_t)63369, (uint16_t)5050, (uint16_t)36474, (uint16_t)15075, (uint16_t)56952, (uint16_t)32933, (uint16_t)52420, (uint16_t)64430, (uint16_t)23972, (uint16_t)60321, (uint16_t)48862, (uint16_t)26710, (uint16_t)16856, (uint16_t)2159, (uint16_t)7099, (uint16_t)1397, (uint16_t)23285, (uint16_t)31931, (uint16_t)14946, (uint16_t)50223, (uint16_t)14244, (uint16_t)53551, (uint16_t)13594, (uint16_t)63726};
        p330_distances_SET(&distances, 0, &PH.base.pack) ;
    }
    p330_increment_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
    p330_min_distance_SET((uint16_t)(uint16_t)59321, PH.base.pack) ;
    p330_max_distance_SET((uint16_t)(uint16_t)4775, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}
