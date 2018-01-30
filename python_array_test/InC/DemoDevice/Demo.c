
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
void c_LoopBackDemoChannel_on_ARRAY_TEST_0_150(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  v1 = p150_v1_GET(pack);
    int8_t*  ar_i8 = p150_ar_i8_GET_(pack);
//process data in ar_i8
    free(ar_i8);//never forget to dispose
    uint8_t*  ar_u8 = p150_ar_u8_GET_(pack);
//process data in ar_u8
    free(ar_u8);//never forget to dispose
    uint16_t*  ar_u16 = p150_ar_u16_GET_(pack);
//process data in ar_u16
    free(ar_u16);//never forget to dispose
    uint32_t*  ar_u32 = p150_ar_u32_GET_(pack);
//process data in ar_u32
    free(ar_u32);//never forget to dispose
}
void c_LoopBackDemoChannel_on_ARRAY_TEST_1_151(Bounds_Inside * ph, Pack * pack)
{
    uint32_t*  ar_u32 = p151_ar_u32_GET_(pack);
//process data in ar_u32
    free(ar_u32);//never forget to dispose
}
void c_LoopBackDemoChannel_on_ARRAY_TEST_3_153(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  v = p153_v_GET(pack);
    uint32_t*  ar_u32 = p153_ar_u32_GET_(pack);
//process data in ar_u32
    free(ar_u32);//never forget to dispose
}
void c_LoopBackDemoChannel_on_ARRAY_TEST_4_154(Bounds_Inside * ph, Pack * pack)
{
    uint32_t*  ar_u32 = p154_ar_u32_GET_(pack);
//process data in ar_u32
    free(ar_u32);//never forget to dispose
    uint8_t  v = p154_v_GET(pack);
}
void c_LoopBackDemoChannel_on_ARRAY_TEST_5_155(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  c1 = p155_c1_TRY_(ph);
    char16_t *  c2 = p155_c2_TRY_(ph);
}
void c_LoopBackDemoChannel_on_ARRAY_TEST_6_156(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  v1 = p156_v1_GET(pack);
    uint16_t  v2 = p156_v2_GET(pack);
    uint32_t  v3 = p156_v3_GET(pack);
    uint32_t*  ar_u32 = p156_ar_u32_GET_(pack);
//process data in ar_u32
    free(ar_u32);//never forget to dispose
    int32_t*  ar_i32 = p156_ar_i32_GET_(pack);
//process data in ar_i32
    free(ar_i32);//never forget to dispose
    uint16_t*  ar_u16 = p156_ar_u16_GET_(pack);
//process data in ar_u16
    free(ar_u16);//never forget to dispose
    int16_t*  ar_i16 = p156_ar_i16_GET_(pack);
//process data in ar_i16
    free(ar_i16);//never forget to dispose
    uint8_t*  ar_u8 = p156_ar_u8_GET_(pack);
//process data in ar_u8
    free(ar_u8);//never forget to dispose
    int8_t*  ar_i8 = p156_ar_i8_GET_(pack);
//process data in ar_i8
    free(ar_i8);//never forget to dispose
    char16_t *  ar_c = p156_ar_c_TRY_(ph);
    double*  ar_d = p156_ar_d_GET_(pack);
//process data in ar_d
    free(ar_d);//never forget to dispose
    float*  ar_f = p156_ar_f_GET_(pack);
//process data in ar_f
    free(ar_f);//never forget to dispose
}
void c_LoopBackDemoChannel_on_ARRAY_TEST_7_157(Bounds_Inside * ph, Pack * pack)
{
    double*  ar_d = p157_ar_d_GET_(pack);
//process data in ar_d
    free(ar_d);//never forget to dispose
    float*  ar_f = p157_ar_f_GET_(pack);
//process data in ar_f
    free(ar_f);//never forget to dispose
    uint32_t*  ar_u32 = p157_ar_u32_GET_(pack);
//process data in ar_u32
    free(ar_u32);//never forget to dispose
    int32_t*  ar_i32 = p157_ar_i32_GET_(pack);
//process data in ar_i32
    free(ar_i32);//never forget to dispose
    uint16_t*  ar_u16 = p157_ar_u16_GET_(pack);
//process data in ar_u16
    free(ar_u16);//never forget to dispose
    int16_t*  ar_i16 = p157_ar_i16_GET_(pack);
//process data in ar_i16
    free(ar_i16);//never forget to dispose
    uint8_t*  ar_u8 = p157_ar_u8_GET_(pack);
//process data in ar_u8
    free(ar_u8);//never forget to dispose
    int8_t*  ar_i8 = p157_ar_i8_GET_(pack);
//process data in ar_i8
    free(ar_i8);//never forget to dispose
    char16_t *  ar_c = p157_ar_c_TRY_(ph);
}
void c_LoopBackDemoChannel_on_ARRAY_TEST_8_158(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  v3 = p158_v3_GET(pack);
    double*  ar_d = p158_ar_d_GET_(pack);
//process data in ar_d
    free(ar_d);//never forget to dispose
    uint16_t*  ar_u16 = p158_ar_u16_GET_(pack);
//process data in ar_u16
    free(ar_u16);//never forget to dispose
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
    p0_type_SET(e_MAV_TYPE_MAV_TYPE_ADSB, PH.base.pack) ;
    p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_FP, PH.base.pack) ;
    p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED, PH.base.pack) ;
    p0_custom_mode_SET((uint32_t)3505445617L, PH.base.pack) ;
    p0_system_status_SET(e_MAV_STATE_MAV_STATE_POWEROFF, PH.base.pack) ;
    p0_mavlink_version_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SYS_STATUS_1(), &PH);
    p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR, PH.base.pack) ;
    p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2, PH.base.pack) ;
    p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO, PH.base.pack) ;
    p1_load_SET((uint16_t)(uint16_t)38857, PH.base.pack) ;
    p1_voltage_battery_SET((uint16_t)(uint16_t)3903, PH.base.pack) ;
    p1_current_battery_SET((int16_t)(int16_t)14565, PH.base.pack) ;
    p1_battery_remaining_SET((int8_t)(int8_t) -101, PH.base.pack) ;
    p1_drop_rate_comm_SET((uint16_t)(uint16_t)45889, PH.base.pack) ;
    p1_errors_comm_SET((uint16_t)(uint16_t)21912, PH.base.pack) ;
    p1_errors_count1_SET((uint16_t)(uint16_t)45461, PH.base.pack) ;
    p1_errors_count2_SET((uint16_t)(uint16_t)11083, PH.base.pack) ;
    p1_errors_count3_SET((uint16_t)(uint16_t)55355, PH.base.pack) ;
    p1_errors_count4_SET((uint16_t)(uint16_t)64338, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SYSTEM_TIME_2(), &PH);
    p2_time_unix_usec_SET((uint64_t)9084380584995275623L, PH.base.pack) ;
    p2_time_boot_ms_SET((uint32_t)1172773556L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
    p3_time_boot_ms_SET((uint32_t)2659751667L, PH.base.pack) ;
    p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
    p3_type_mask_SET((uint16_t)(uint16_t)14509, PH.base.pack) ;
    p3_x_SET((float) -1.9632464E38F, PH.base.pack) ;
    p3_y_SET((float) -2.5639176E36F, PH.base.pack) ;
    p3_z_SET((float) -3.9638567E37F, PH.base.pack) ;
    p3_vx_SET((float)1.6852324E38F, PH.base.pack) ;
    p3_vy_SET((float)1.2862106E38F, PH.base.pack) ;
    p3_vz_SET((float)2.0582397E38F, PH.base.pack) ;
    p3_afx_SET((float) -2.4773943E38F, PH.base.pack) ;
    p3_afy_SET((float) -2.3671363E38F, PH.base.pack) ;
    p3_afz_SET((float) -2.8249383E38F, PH.base.pack) ;
    p3_yaw_SET((float) -4.8201167E37F, PH.base.pack) ;
    p3_yaw_rate_SET((float)3.9619983E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PING_4(), &PH);
    p4_time_usec_SET((uint64_t)8493738140108384798L, PH.base.pack) ;
    p4_seq_SET((uint32_t)2861039248L, PH.base.pack) ;
    p4_target_system_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
    p4_target_component_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
    p5_target_system_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
    p5_control_request_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    p5_version_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
    {
        char16_t   passkey = "qhcVleazxlvuabCxmfyxhoe";
        p5_passkey_SET(&passkey, 0,  sizeof(passkey), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
    p6_gcs_system_id_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
    p6_control_request_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
    p6_ack_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AUTH_KEY_7(), &PH);
    {
        char16_t   key = "qQauoobyeekbldcjxmdqojlpcrxeoksb";
        p7_key_SET(&key, 0,  sizeof(key), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_MODE_11(), &PH);
    p11_target_system_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
    p11_base_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_ARMED, PH.base.pack) ;
    p11_custom_mode_SET((uint32_t)3699165188L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_READ_20(), &PH);
    p20_target_system_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
    p20_target_component_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
    {
        char16_t   param_id = "jnmgrq";
        p20_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p20_param_index_SET((int16_t)(int16_t) -17339, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_LIST_21(), &PH);
    p21_target_system_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
    p21_target_component_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_VALUE_22(), &PH);
    {
        char16_t   param_id = "iclz";
        p22_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p22_param_value_SET((float)2.6164757E38F, PH.base.pack) ;
    p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64, PH.base.pack) ;
    p22_param_count_SET((uint16_t)(uint16_t)35136, PH.base.pack) ;
    p22_param_index_SET((uint16_t)(uint16_t)52256, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_SET_23(), &PH);
    p23_target_system_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
    p23_target_component_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
    {
        char16_t   param_id = "vUakhzzu";
        p23_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p23_param_value_SET((float) -1.1445042E38F, PH.base.pack) ;
    p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RAW_INT_24(), &PH);
    p24_time_usec_SET((uint64_t)5416198932869707942L, PH.base.pack) ;
    p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, PH.base.pack) ;
    p24_lat_SET((int32_t)82144791, PH.base.pack) ;
    p24_lon_SET((int32_t)1228125092, PH.base.pack) ;
    p24_alt_SET((int32_t) -561340285, PH.base.pack) ;
    p24_eph_SET((uint16_t)(uint16_t)36132, PH.base.pack) ;
    p24_epv_SET((uint16_t)(uint16_t)56935, PH.base.pack) ;
    p24_vel_SET((uint16_t)(uint16_t)43825, PH.base.pack) ;
    p24_cog_SET((uint16_t)(uint16_t)14677, PH.base.pack) ;
    p24_satellites_visible_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
    p24_alt_ellipsoid_SET((int32_t) -408898085, &PH) ;
    p24_h_acc_SET((uint32_t)1469197476L, &PH) ;
    p24_v_acc_SET((uint32_t)4039537060L, &PH) ;
    p24_vel_acc_SET((uint32_t)700615764L, &PH) ;
    p24_hdg_acc_SET((uint32_t)3767089339L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_STATUS_25(), &PH);
    p25_satellites_visible_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
    {
        uint8_t  satellite_prn [] =  {(uint8_t)234, (uint8_t)160, (uint8_t)117, (uint8_t)25, (uint8_t)231, (uint8_t)185, (uint8_t)50, (uint8_t)91, (uint8_t)36, (uint8_t)209, (uint8_t)35, (uint8_t)173, (uint8_t)78, (uint8_t)26, (uint8_t)136, (uint8_t)236, (uint8_t)103, (uint8_t)50, (uint8_t)72, (uint8_t)0};
        p25_satellite_prn_SET(&satellite_prn, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_used [] =  {(uint8_t)29, (uint8_t)247, (uint8_t)154, (uint8_t)112, (uint8_t)94, (uint8_t)61, (uint8_t)248, (uint8_t)161, (uint8_t)93, (uint8_t)252, (uint8_t)61, (uint8_t)10, (uint8_t)223, (uint8_t)59, (uint8_t)176, (uint8_t)120, (uint8_t)176, (uint8_t)72, (uint8_t)221, (uint8_t)247};
        p25_satellite_used_SET(&satellite_used, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_elevation [] =  {(uint8_t)213, (uint8_t)222, (uint8_t)179, (uint8_t)140, (uint8_t)174, (uint8_t)105, (uint8_t)158, (uint8_t)10, (uint8_t)148, (uint8_t)169, (uint8_t)243, (uint8_t)10, (uint8_t)47, (uint8_t)70, (uint8_t)136, (uint8_t)236, (uint8_t)127, (uint8_t)94, (uint8_t)90, (uint8_t)158};
        p25_satellite_elevation_SET(&satellite_elevation, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_azimuth [] =  {(uint8_t)62, (uint8_t)90, (uint8_t)240, (uint8_t)85, (uint8_t)254, (uint8_t)235, (uint8_t)134, (uint8_t)64, (uint8_t)58, (uint8_t)204, (uint8_t)13, (uint8_t)227, (uint8_t)12, (uint8_t)38, (uint8_t)48, (uint8_t)83, (uint8_t)30, (uint8_t)56, (uint8_t)36, (uint8_t)70};
        p25_satellite_azimuth_SET(&satellite_azimuth, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_snr [] =  {(uint8_t)11, (uint8_t)254, (uint8_t)28, (uint8_t)181, (uint8_t)222, (uint8_t)86, (uint8_t)3, (uint8_t)158, (uint8_t)84, (uint8_t)141, (uint8_t)164, (uint8_t)217, (uint8_t)76, (uint8_t)154, (uint8_t)164, (uint8_t)64, (uint8_t)154, (uint8_t)15, (uint8_t)114, (uint8_t)75};
        p25_satellite_snr_SET(&satellite_snr, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU_26(), &PH);
    p26_time_boot_ms_SET((uint32_t)3906168667L, PH.base.pack) ;
    p26_xacc_SET((int16_t)(int16_t)2677, PH.base.pack) ;
    p26_yacc_SET((int16_t)(int16_t)26142, PH.base.pack) ;
    p26_zacc_SET((int16_t)(int16_t)24721, PH.base.pack) ;
    p26_xgyro_SET((int16_t)(int16_t) -32457, PH.base.pack) ;
    p26_ygyro_SET((int16_t)(int16_t)7139, PH.base.pack) ;
    p26_zgyro_SET((int16_t)(int16_t)9062, PH.base.pack) ;
    p26_xmag_SET((int16_t)(int16_t)8338, PH.base.pack) ;
    p26_ymag_SET((int16_t)(int16_t)4564, PH.base.pack) ;
    p26_zmag_SET((int16_t)(int16_t) -17503, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RAW_IMU_27(), &PH);
    p27_time_usec_SET((uint64_t)3465460130645628538L, PH.base.pack) ;
    p27_xacc_SET((int16_t)(int16_t)26531, PH.base.pack) ;
    p27_yacc_SET((int16_t)(int16_t) -11291, PH.base.pack) ;
    p27_zacc_SET((int16_t)(int16_t) -10346, PH.base.pack) ;
    p27_xgyro_SET((int16_t)(int16_t) -18566, PH.base.pack) ;
    p27_ygyro_SET((int16_t)(int16_t)31671, PH.base.pack) ;
    p27_zgyro_SET((int16_t)(int16_t)25488, PH.base.pack) ;
    p27_xmag_SET((int16_t)(int16_t) -17540, PH.base.pack) ;
    p27_ymag_SET((int16_t)(int16_t) -24570, PH.base.pack) ;
    p27_zmag_SET((int16_t)(int16_t)11489, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RAW_PRESSURE_28(), &PH);
    p28_time_usec_SET((uint64_t)1372567754648819522L, PH.base.pack) ;
    p28_press_abs_SET((int16_t)(int16_t) -11093, PH.base.pack) ;
    p28_press_diff1_SET((int16_t)(int16_t)17628, PH.base.pack) ;
    p28_press_diff2_SET((int16_t)(int16_t)3231, PH.base.pack) ;
    p28_temperature_SET((int16_t)(int16_t)17651, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE_29(), &PH);
    p29_time_boot_ms_SET((uint32_t)879241743L, PH.base.pack) ;
    p29_press_abs_SET((float) -2.1384746E38F, PH.base.pack) ;
    p29_press_diff_SET((float) -2.608027E38F, PH.base.pack) ;
    p29_temperature_SET((int16_t)(int16_t) -32709, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_30(), &PH);
    p30_time_boot_ms_SET((uint32_t)944494128L, PH.base.pack) ;
    p30_roll_SET((float)5.8885986E37F, PH.base.pack) ;
    p30_pitch_SET((float) -2.7545944E38F, PH.base.pack) ;
    p30_yaw_SET((float)9.341567E37F, PH.base.pack) ;
    p30_rollspeed_SET((float) -1.2472932E38F, PH.base.pack) ;
    p30_pitchspeed_SET((float)1.9822729E37F, PH.base.pack) ;
    p30_yawspeed_SET((float)2.3351661E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_31(), &PH);
    p31_time_boot_ms_SET((uint32_t)372893844L, PH.base.pack) ;
    p31_q1_SET((float) -2.3373563E37F, PH.base.pack) ;
    p31_q2_SET((float) -2.373033E38F, PH.base.pack) ;
    p31_q3_SET((float)1.0762108E38F, PH.base.pack) ;
    p31_q4_SET((float) -8.727873E37F, PH.base.pack) ;
    p31_rollspeed_SET((float) -2.3432712E38F, PH.base.pack) ;
    p31_pitchspeed_SET((float) -3.1498385E38F, PH.base.pack) ;
    p31_yawspeed_SET((float)1.7728702E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_32(), &PH);
    p32_time_boot_ms_SET((uint32_t)2186949077L, PH.base.pack) ;
    p32_x_SET((float) -2.9974059E38F, PH.base.pack) ;
    p32_y_SET((float) -1.4963127E37F, PH.base.pack) ;
    p32_z_SET((float) -3.304115E38F, PH.base.pack) ;
    p32_vx_SET((float) -1.0319568E38F, PH.base.pack) ;
    p32_vy_SET((float) -3.1855658E38F, PH.base.pack) ;
    p32_vz_SET((float) -1.1755414E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_33(), &PH);
    p33_time_boot_ms_SET((uint32_t)1944855222L, PH.base.pack) ;
    p33_lat_SET((int32_t)1962801952, PH.base.pack) ;
    p33_lon_SET((int32_t) -1023348636, PH.base.pack) ;
    p33_alt_SET((int32_t) -1492227391, PH.base.pack) ;
    p33_relative_alt_SET((int32_t) -1384965501, PH.base.pack) ;
    p33_vx_SET((int16_t)(int16_t)6677, PH.base.pack) ;
    p33_vy_SET((int16_t)(int16_t) -29963, PH.base.pack) ;
    p33_vz_SET((int16_t)(int16_t)6492, PH.base.pack) ;
    p33_hdg_SET((uint16_t)(uint16_t)7141, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_SCALED_34(), &PH);
    p34_time_boot_ms_SET((uint32_t)608554795L, PH.base.pack) ;
    p34_port_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
    p34_chan1_scaled_SET((int16_t)(int16_t)13328, PH.base.pack) ;
    p34_chan2_scaled_SET((int16_t)(int16_t)10761, PH.base.pack) ;
    p34_chan3_scaled_SET((int16_t)(int16_t)14753, PH.base.pack) ;
    p34_chan4_scaled_SET((int16_t)(int16_t)17621, PH.base.pack) ;
    p34_chan5_scaled_SET((int16_t)(int16_t) -20790, PH.base.pack) ;
    p34_chan6_scaled_SET((int16_t)(int16_t)1765, PH.base.pack) ;
    p34_chan7_scaled_SET((int16_t)(int16_t) -12003, PH.base.pack) ;
    p34_chan8_scaled_SET((int16_t)(int16_t) -18996, PH.base.pack) ;
    p34_rssi_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_RAW_35(), &PH);
    p35_time_boot_ms_SET((uint32_t)3231960149L, PH.base.pack) ;
    p35_port_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
    p35_chan1_raw_SET((uint16_t)(uint16_t)22529, PH.base.pack) ;
    p35_chan2_raw_SET((uint16_t)(uint16_t)56815, PH.base.pack) ;
    p35_chan3_raw_SET((uint16_t)(uint16_t)15441, PH.base.pack) ;
    p35_chan4_raw_SET((uint16_t)(uint16_t)13066, PH.base.pack) ;
    p35_chan5_raw_SET((uint16_t)(uint16_t)16306, PH.base.pack) ;
    p35_chan6_raw_SET((uint16_t)(uint16_t)1010, PH.base.pack) ;
    p35_chan7_raw_SET((uint16_t)(uint16_t)42876, PH.base.pack) ;
    p35_chan8_raw_SET((uint16_t)(uint16_t)3460, PH.base.pack) ;
    p35_rssi_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
    p36_time_usec_SET((uint32_t)1699716410L, PH.base.pack) ;
    p36_port_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
    p36_servo1_raw_SET((uint16_t)(uint16_t)16736, PH.base.pack) ;
    p36_servo2_raw_SET((uint16_t)(uint16_t)20203, PH.base.pack) ;
    p36_servo3_raw_SET((uint16_t)(uint16_t)10852, PH.base.pack) ;
    p36_servo4_raw_SET((uint16_t)(uint16_t)24169, PH.base.pack) ;
    p36_servo5_raw_SET((uint16_t)(uint16_t)63022, PH.base.pack) ;
    p36_servo6_raw_SET((uint16_t)(uint16_t)27233, PH.base.pack) ;
    p36_servo7_raw_SET((uint16_t)(uint16_t)24643, PH.base.pack) ;
    p36_servo8_raw_SET((uint16_t)(uint16_t)61979, PH.base.pack) ;
    p36_servo9_raw_SET((uint16_t)(uint16_t)44355, &PH) ;
    p36_servo10_raw_SET((uint16_t)(uint16_t)16672, &PH) ;
    p36_servo11_raw_SET((uint16_t)(uint16_t)11534, &PH) ;
    p36_servo12_raw_SET((uint16_t)(uint16_t)56429, &PH) ;
    p36_servo13_raw_SET((uint16_t)(uint16_t)57746, &PH) ;
    p36_servo14_raw_SET((uint16_t)(uint16_t)21416, &PH) ;
    p36_servo15_raw_SET((uint16_t)(uint16_t)3344, &PH) ;
    p36_servo16_raw_SET((uint16_t)(uint16_t)1107, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
    p37_target_system_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
    p37_target_component_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
    p37_start_index_SET((int16_t)(int16_t) -6204, PH.base.pack) ;
    p37_end_index_SET((int16_t)(int16_t) -26302, PH.base.pack) ;
    p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
    p38_target_system_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
    p38_target_component_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
    p38_start_index_SET((int16_t)(int16_t)30665, PH.base.pack) ;
    p38_end_index_SET((int16_t)(int16_t)30817, PH.base.pack) ;
    p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_39(), &PH);
    p39_target_system_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
    p39_target_component_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
    p39_seq_SET((uint16_t)(uint16_t)64160, PH.base.pack) ;
    p39_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
    p39_command_SET(e_MAV_CMD_MAV_CMD_DO_SET_REVERSE, PH.base.pack) ;
    p39_current_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
    p39_autocontinue_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
    p39_param1_SET((float)1.0952539E38F, PH.base.pack) ;
    p39_param2_SET((float) -3.000232E38F, PH.base.pack) ;
    p39_param3_SET((float)2.8189262E38F, PH.base.pack) ;
    p39_param4_SET((float) -9.428973E37F, PH.base.pack) ;
    p39_x_SET((float)1.0461332E38F, PH.base.pack) ;
    p39_y_SET((float)1.9075436E38F, PH.base.pack) ;
    p39_z_SET((float) -1.3991871E37F, PH.base.pack) ;
    p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_40(), &PH);
    p40_target_system_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
    p40_target_component_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
    p40_seq_SET((uint16_t)(uint16_t)17930, PH.base.pack) ;
    p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_SET_CURRENT_41(), &PH);
    p41_target_system_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
    p41_target_component_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
    p41_seq_SET((uint16_t)(uint16_t)236, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_CURRENT_42(), &PH);
    p42_seq_SET((uint16_t)(uint16_t)45348, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_LIST_43(), &PH);
    p43_target_system_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
    p43_target_component_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
    p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_COUNT_44(), &PH);
    p44_target_system_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
    p44_target_component_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
    p44_count_SET((uint16_t)(uint16_t)43027, PH.base.pack) ;
    p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_CLEAR_ALL_45(), &PH);
    p45_target_system_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
    p45_target_component_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
    p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_REACHED_46(), &PH);
    p46_seq_SET((uint16_t)(uint16_t)31834, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ACK_47(), &PH);
    p47_target_system_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
    p47_target_component_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
    p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM2, PH.base.pack) ;
    p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
    p48_target_system_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
    p48_latitude_SET((int32_t)1355603933, PH.base.pack) ;
    p48_longitude_SET((int32_t) -1974816801, PH.base.pack) ;
    p48_altitude_SET((int32_t) -1914157570, PH.base.pack) ;
    p48_time_usec_SET((uint64_t)5192668455935243602L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
    p49_latitude_SET((int32_t) -2140477988, PH.base.pack) ;
    p49_longitude_SET((int32_t)1688702304, PH.base.pack) ;
    p49_altitude_SET((int32_t)1909696353, PH.base.pack) ;
    p49_time_usec_SET((uint64_t)3659705927942417559L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_MAP_RC_50(), &PH);
    p50_target_system_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
    p50_target_component_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
    {
        char16_t   param_id = "kblfBznbvlq";
        p50_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p50_param_index_SET((int16_t)(int16_t) -14772, PH.base.pack) ;
    p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
    p50_param_value0_SET((float) -2.759545E38F, PH.base.pack) ;
    p50_scale_SET((float) -3.1091046E38F, PH.base.pack) ;
    p50_param_value_min_SET((float)1.3493785E38F, PH.base.pack) ;
    p50_param_value_max_SET((float)2.9335925E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_INT_51(), &PH);
    p51_target_system_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    p51_target_component_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
    p51_seq_SET((uint16_t)(uint16_t)7503, PH.base.pack) ;
    p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
    p54_target_system_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
    p54_target_component_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
    p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
    p54_p1x_SET((float) -1.6723052E38F, PH.base.pack) ;
    p54_p1y_SET((float) -3.3742979E38F, PH.base.pack) ;
    p54_p1z_SET((float) -1.5876824E38F, PH.base.pack) ;
    p54_p2x_SET((float) -1.1653755E38F, PH.base.pack) ;
    p54_p2y_SET((float) -1.814398E38F, PH.base.pack) ;
    p54_p2z_SET((float) -1.9341131E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
    p55_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
    p55_p1x_SET((float) -1.4218359E38F, PH.base.pack) ;
    p55_p1y_SET((float) -2.526801E38F, PH.base.pack) ;
    p55_p1z_SET((float) -3.0879336E38F, PH.base.pack) ;
    p55_p2x_SET((float)2.450228E37F, PH.base.pack) ;
    p55_p2y_SET((float) -5.067096E37F, PH.base.pack) ;
    p55_p2z_SET((float) -1.9318922E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
    p61_time_usec_SET((uint64_t)726793707566010817L, PH.base.pack) ;
    {
        float  q [] =  {-9.009549E37F, 1.9483031E38F, -2.1413448E37F, 2.7379062E38F};
        p61_q_SET(&q, 0, &PH.base.pack) ;
    }
    p61_rollspeed_SET((float) -1.1932315E38F, PH.base.pack) ;
    p61_pitchspeed_SET((float) -1.8515228E38F, PH.base.pack) ;
    p61_yawspeed_SET((float)3.2995903E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-1.6204029E38F, 1.7819533E38F, -2.3332519E38F, -2.0033185E38F, 7.9432437E37F, -2.2137416E38F, -3.2871393E38F, -2.4877627E38F, -2.9367162E37F};
        p61_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
    p62_nav_roll_SET((float) -2.3184662E38F, PH.base.pack) ;
    p62_nav_pitch_SET((float) -1.6618832E38F, PH.base.pack) ;
    p62_nav_bearing_SET((int16_t)(int16_t)29618, PH.base.pack) ;
    p62_target_bearing_SET((int16_t)(int16_t)18549, PH.base.pack) ;
    p62_wp_dist_SET((uint16_t)(uint16_t)17664, PH.base.pack) ;
    p62_alt_error_SET((float) -2.8954659E38F, PH.base.pack) ;
    p62_aspd_error_SET((float)9.25541E37F, PH.base.pack) ;
    p62_xtrack_error_SET((float)3.2304737E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
    p63_time_usec_SET((uint64_t)4335969117596424175L, PH.base.pack) ;
    p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, PH.base.pack) ;
    p63_lat_SET((int32_t) -1351958907, PH.base.pack) ;
    p63_lon_SET((int32_t)882398543, PH.base.pack) ;
    p63_alt_SET((int32_t) -1717601392, PH.base.pack) ;
    p63_relative_alt_SET((int32_t) -1670965260, PH.base.pack) ;
    p63_vx_SET((float) -2.6377698E38F, PH.base.pack) ;
    p63_vy_SET((float)3.0867986E38F, PH.base.pack) ;
    p63_vz_SET((float) -3.144352E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-2.9356839E38F, -3.0716815E38F, -9.648417E37F, 3.121494E38F, -2.041483E38F, 2.5479643E38F, 5.213027E37F, 7.762337E37F, 1.86598E38F, 3.3962522E38F, 3.087952E38F, -2.4634136E38F, -1.485978E38F, 1.9834982E38F, -8.822449E37F, 2.543458E38F, 3.3307035E38F, -8.634061E37F, -3.253559E38F, 1.9240385E38F, -3.367906E38F, -2.4106465E38F, 1.3006059E38F, -2.5919964E37F, -2.0021957E38F, -2.4605964E38F, -1.1317658E37F, 3.0958786E38F, -2.7805812E38F, -6.7763247E37F, 9.443681E37F, -3.6862024E37F, 3.3253297E38F, -9.96046E37F, -1.4323481E38F, -3.2208945E38F};
        p63_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
    p64_time_usec_SET((uint64_t)2807689251735800272L, PH.base.pack) ;
    p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, PH.base.pack) ;
    p64_x_SET((float)3.3118603E38F, PH.base.pack) ;
    p64_y_SET((float)6.461398E37F, PH.base.pack) ;
    p64_z_SET((float)2.3654026E38F, PH.base.pack) ;
    p64_vx_SET((float)1.0977553E38F, PH.base.pack) ;
    p64_vy_SET((float) -2.4279959E38F, PH.base.pack) ;
    p64_vz_SET((float) -1.5343271E38F, PH.base.pack) ;
    p64_ax_SET((float)3.3626755E38F, PH.base.pack) ;
    p64_ay_SET((float)2.8436206E38F, PH.base.pack) ;
    p64_az_SET((float)4.7666025E37F, PH.base.pack) ;
    {
        float  covariance [] =  {-2.8940887E37F, 2.6354462E38F, -2.476372E38F, 1.7683535E38F, 2.73508E38F, -2.291189E38F, 2.3273931E38F, 4.279828E36F, -2.5197787E38F, -1.0695069E38F, 1.8995194E36F, -1.7168555E38F, 2.6631792E38F, 1.9575452E38F, 2.7133704E37F, -1.6204896E38F, -2.098455E38F, 3.0512234E38F, 2.9144529E38F, 2.4796298E38F, 2.0568235E38F, 1.505509E38F, -2.3227237E38F, -2.6946694E38F, -1.1399436E38F, -3.7631E37F, 3.055262E37F, -1.4756897E38F, 2.8322965E38F, -1.0051575E38F, 3.215895E38F, 6.978396E37F, -2.0078738E38F, 3.1616488E38F, -1.8403153E38F, -2.9889887E38F, -7.550873E37F, 3.3423247E38F, 7.944369E37F, 2.7540348E38F, 2.0552602E38F, -2.9106858E37F, -2.8353057E37F, 4.563368E37F, -2.4243463E38F};
        p64_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_65(), &PH);
    p65_time_boot_ms_SET((uint32_t)2457354110L, PH.base.pack) ;
    p65_chancount_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
    p65_chan1_raw_SET((uint16_t)(uint16_t)6555, PH.base.pack) ;
    p65_chan2_raw_SET((uint16_t)(uint16_t)52917, PH.base.pack) ;
    p65_chan3_raw_SET((uint16_t)(uint16_t)9185, PH.base.pack) ;
    p65_chan4_raw_SET((uint16_t)(uint16_t)17414, PH.base.pack) ;
    p65_chan5_raw_SET((uint16_t)(uint16_t)17754, PH.base.pack) ;
    p65_chan6_raw_SET((uint16_t)(uint16_t)24438, PH.base.pack) ;
    p65_chan7_raw_SET((uint16_t)(uint16_t)21644, PH.base.pack) ;
    p65_chan8_raw_SET((uint16_t)(uint16_t)64407, PH.base.pack) ;
    p65_chan9_raw_SET((uint16_t)(uint16_t)42668, PH.base.pack) ;
    p65_chan10_raw_SET((uint16_t)(uint16_t)33671, PH.base.pack) ;
    p65_chan11_raw_SET((uint16_t)(uint16_t)43565, PH.base.pack) ;
    p65_chan12_raw_SET((uint16_t)(uint16_t)16534, PH.base.pack) ;
    p65_chan13_raw_SET((uint16_t)(uint16_t)60072, PH.base.pack) ;
    p65_chan14_raw_SET((uint16_t)(uint16_t)901, PH.base.pack) ;
    p65_chan15_raw_SET((uint16_t)(uint16_t)49840, PH.base.pack) ;
    p65_chan16_raw_SET((uint16_t)(uint16_t)15677, PH.base.pack) ;
    p65_chan17_raw_SET((uint16_t)(uint16_t)61874, PH.base.pack) ;
    p65_chan18_raw_SET((uint16_t)(uint16_t)62002, PH.base.pack) ;
    p65_rssi_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_REQUEST_DATA_STREAM_66(), &PH);
    p66_target_system_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
    p66_target_component_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
    p66_req_stream_id_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
    p66_req_message_rate_SET((uint16_t)(uint16_t)11732, PH.base.pack) ;
    p66_start_stop_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA_STREAM_67(), &PH);
    p67_stream_id_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
    p67_message_rate_SET((uint16_t)(uint16_t)11219, PH.base.pack) ;
    p67_on_off_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MANUAL_CONTROL_69(), &PH);
    p69_target_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
    p69_x_SET((int16_t)(int16_t) -10247, PH.base.pack) ;
    p69_y_SET((int16_t)(int16_t) -20845, PH.base.pack) ;
    p69_z_SET((int16_t)(int16_t) -23933, PH.base.pack) ;
    p69_r_SET((int16_t)(int16_t)13639, PH.base.pack) ;
    p69_buttons_SET((uint16_t)(uint16_t)52126, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
    p70_target_system_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
    p70_target_component_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
    p70_chan1_raw_SET((uint16_t)(uint16_t)9208, PH.base.pack) ;
    p70_chan2_raw_SET((uint16_t)(uint16_t)5700, PH.base.pack) ;
    p70_chan3_raw_SET((uint16_t)(uint16_t)65328, PH.base.pack) ;
    p70_chan4_raw_SET((uint16_t)(uint16_t)52511, PH.base.pack) ;
    p70_chan5_raw_SET((uint16_t)(uint16_t)21241, PH.base.pack) ;
    p70_chan6_raw_SET((uint16_t)(uint16_t)62097, PH.base.pack) ;
    p70_chan7_raw_SET((uint16_t)(uint16_t)60585, PH.base.pack) ;
    p70_chan8_raw_SET((uint16_t)(uint16_t)49417, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_INT_73(), &PH);
    p73_target_system_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
    p73_target_component_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
    p73_seq_SET((uint16_t)(uint16_t)9016, PH.base.pack) ;
    p73_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
    p73_command_SET(e_MAV_CMD_MAV_CMD_DO_LAST, PH.base.pack) ;
    p73_current_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
    p73_autocontinue_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
    p73_param1_SET((float) -9.701137E37F, PH.base.pack) ;
    p73_param2_SET((float)1.7596662E38F, PH.base.pack) ;
    p73_param3_SET((float) -2.74613E38F, PH.base.pack) ;
    p73_param4_SET((float)2.4374806E38F, PH.base.pack) ;
    p73_x_SET((int32_t) -1450142249, PH.base.pack) ;
    p73_y_SET((int32_t) -1999791260, PH.base.pack) ;
    p73_z_SET((float)2.7743706E38F, PH.base.pack) ;
    p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VFR_HUD_74(), &PH);
    p74_airspeed_SET((float) -1.3198663E38F, PH.base.pack) ;
    p74_groundspeed_SET((float) -6.719252E37F, PH.base.pack) ;
    p74_heading_SET((int16_t)(int16_t) -22228, PH.base.pack) ;
    p74_throttle_SET((uint16_t)(uint16_t)48571, PH.base.pack) ;
    p74_alt_SET((float)1.3519902E38F, PH.base.pack) ;
    p74_climb_SET((float) -6.7070415E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_INT_75(), &PH);
    p75_target_system_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
    p75_target_component_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
    p75_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
    p75_command_SET(e_MAV_CMD_MAV_CMD_USER_3, PH.base.pack) ;
    p75_current_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
    p75_autocontinue_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
    p75_param1_SET((float)3.3074639E38F, PH.base.pack) ;
    p75_param2_SET((float)2.758582E37F, PH.base.pack) ;
    p75_param3_SET((float) -2.332283E38F, PH.base.pack) ;
    p75_param4_SET((float)1.343761E38F, PH.base.pack) ;
    p75_x_SET((int32_t) -1707871892, PH.base.pack) ;
    p75_y_SET((int32_t) -1179206753, PH.base.pack) ;
    p75_z_SET((float) -1.3574292E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_LONG_76(), &PH);
    p76_target_system_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
    p76_target_component_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
    p76_command_SET(e_MAV_CMD_MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION, PH.base.pack) ;
    p76_confirmation_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
    p76_param1_SET((float) -3.0861698E38F, PH.base.pack) ;
    p76_param2_SET((float) -5.7784813E37F, PH.base.pack) ;
    p76_param3_SET((float) -1.127989E38F, PH.base.pack) ;
    p76_param4_SET((float) -1.6385099E38F, PH.base.pack) ;
    p76_param5_SET((float) -4.972488E37F, PH.base.pack) ;
    p76_param6_SET((float) -5.798511E37F, PH.base.pack) ;
    p76_param7_SET((float) -6.4570983E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_ACK_77(), &PH);
    p77_command_SET(e_MAV_CMD_MAV_CMD_NAV_LOITER_UNLIM, PH.base.pack) ;
    p77_result_SET(e_MAV_RESULT_MAV_RESULT_TEMPORARILY_REJECTED, PH.base.pack) ;
    p77_progress_SET((uint8_t)(uint8_t)55, &PH) ;
    p77_result_param2_SET((int32_t) -404766067, &PH) ;
    p77_target_system_SET((uint8_t)(uint8_t)61, &PH) ;
    p77_target_component_SET((uint8_t)(uint8_t)169, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MANUAL_SETPOINT_81(), &PH);
    p81_time_boot_ms_SET((uint32_t)2086509084L, PH.base.pack) ;
    p81_roll_SET((float) -2.5877643E38F, PH.base.pack) ;
    p81_pitch_SET((float)3.0507296E38F, PH.base.pack) ;
    p81_yaw_SET((float)2.3646235E38F, PH.base.pack) ;
    p81_thrust_SET((float)5.61795E37F, PH.base.pack) ;
    p81_mode_switch_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
    p81_manual_override_switch_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
    p82_time_boot_ms_SET((uint32_t)559719964L, PH.base.pack) ;
    p82_target_system_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
    p82_target_component_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
    p82_type_mask_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
    {
        float  q [] =  {2.8618452E38F, 6.912224E36F, 2.0037897E38F, 1.1645633E38F};
        p82_q_SET(&q, 0, &PH.base.pack) ;
    }
    p82_body_roll_rate_SET((float) -2.633361E38F, PH.base.pack) ;
    p82_body_pitch_rate_SET((float) -2.6522252E38F, PH.base.pack) ;
    p82_body_yaw_rate_SET((float) -2.5756977E38F, PH.base.pack) ;
    p82_thrust_SET((float) -9.264652E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_TARGET_83(), &PH);
    p83_time_boot_ms_SET((uint32_t)930911190L, PH.base.pack) ;
    p83_type_mask_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
    {
        float  q [] =  {-3.0921773E37F, 1.609771E38F, 2.772059E38F, -2.8587127E38F};
        p83_q_SET(&q, 0, &PH.base.pack) ;
    }
    p83_body_roll_rate_SET((float) -1.8358791E38F, PH.base.pack) ;
    p83_body_pitch_rate_SET((float)3.124438E38F, PH.base.pack) ;
    p83_body_yaw_rate_SET((float)1.2128352E38F, PH.base.pack) ;
    p83_thrust_SET((float)4.2243933E35F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
    p84_time_boot_ms_SET((uint32_t)3152915688L, PH.base.pack) ;
    p84_target_system_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
    p84_target_component_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
    p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
    p84_type_mask_SET((uint16_t)(uint16_t)56573, PH.base.pack) ;
    p84_x_SET((float)3.4018063E38F, PH.base.pack) ;
    p84_y_SET((float) -2.078967E37F, PH.base.pack) ;
    p84_z_SET((float)6.092865E37F, PH.base.pack) ;
    p84_vx_SET((float)1.2280138E38F, PH.base.pack) ;
    p84_vy_SET((float) -1.3332232E38F, PH.base.pack) ;
    p84_vz_SET((float) -1.8413856E36F, PH.base.pack) ;
    p84_afx_SET((float)2.5574423E38F, PH.base.pack) ;
    p84_afy_SET((float) -9.357953E37F, PH.base.pack) ;
    p84_afz_SET((float)4.451221E37F, PH.base.pack) ;
    p84_yaw_SET((float)8.79952E37F, PH.base.pack) ;
    p84_yaw_rate_SET((float)2.9618285E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
    p86_time_boot_ms_SET((uint32_t)3042995894L, PH.base.pack) ;
    p86_target_system_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
    p86_target_component_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
    p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
    p86_type_mask_SET((uint16_t)(uint16_t)35106, PH.base.pack) ;
    p86_lat_int_SET((int32_t)37141146, PH.base.pack) ;
    p86_lon_int_SET((int32_t) -423173847, PH.base.pack) ;
    p86_alt_SET((float)1.0755752E38F, PH.base.pack) ;
    p86_vx_SET((float)2.1630188E38F, PH.base.pack) ;
    p86_vy_SET((float) -2.9776727E38F, PH.base.pack) ;
    p86_vz_SET((float) -2.9320604E38F, PH.base.pack) ;
    p86_afx_SET((float)2.5523218E38F, PH.base.pack) ;
    p86_afy_SET((float)2.7677908E38F, PH.base.pack) ;
    p86_afz_SET((float) -3.1322352E38F, PH.base.pack) ;
    p86_yaw_SET((float)1.2610847E38F, PH.base.pack) ;
    p86_yaw_rate_SET((float)7.331565E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
    p87_time_boot_ms_SET((uint32_t)1287628563L, PH.base.pack) ;
    p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
    p87_type_mask_SET((uint16_t)(uint16_t)50687, PH.base.pack) ;
    p87_lat_int_SET((int32_t) -1668137670, PH.base.pack) ;
    p87_lon_int_SET((int32_t) -2013373986, PH.base.pack) ;
    p87_alt_SET((float)1.5533546E38F, PH.base.pack) ;
    p87_vx_SET((float) -1.7065427E38F, PH.base.pack) ;
    p87_vy_SET((float)3.8346841E37F, PH.base.pack) ;
    p87_vz_SET((float)2.4664773E38F, PH.base.pack) ;
    p87_afx_SET((float) -3.209904E38F, PH.base.pack) ;
    p87_afy_SET((float)2.8619153E38F, PH.base.pack) ;
    p87_afz_SET((float)8.907023E37F, PH.base.pack) ;
    p87_yaw_SET((float) -2.394787E38F, PH.base.pack) ;
    p87_yaw_rate_SET((float) -1.9130772E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
    p89_time_boot_ms_SET((uint32_t)1596502620L, PH.base.pack) ;
    p89_x_SET((float)6.2321344E37F, PH.base.pack) ;
    p89_y_SET((float)2.5479203E38F, PH.base.pack) ;
    p89_z_SET((float) -2.2462345E38F, PH.base.pack) ;
    p89_roll_SET((float) -3.042525E37F, PH.base.pack) ;
    p89_pitch_SET((float)1.6701148E38F, PH.base.pack) ;
    p89_yaw_SET((float)7.2944986E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_STATE_90(), &PH);
    p90_time_usec_SET((uint64_t)2786535884008425350L, PH.base.pack) ;
    p90_roll_SET((float)2.9907366E37F, PH.base.pack) ;
    p90_pitch_SET((float) -1.6338236E38F, PH.base.pack) ;
    p90_yaw_SET((float)1.6351354E38F, PH.base.pack) ;
    p90_rollspeed_SET((float)3.0817638E37F, PH.base.pack) ;
    p90_pitchspeed_SET((float)8.820287E37F, PH.base.pack) ;
    p90_yawspeed_SET((float) -2.1609656E38F, PH.base.pack) ;
    p90_lat_SET((int32_t) -488461029, PH.base.pack) ;
    p90_lon_SET((int32_t)501299900, PH.base.pack) ;
    p90_alt_SET((int32_t)822930886, PH.base.pack) ;
    p90_vx_SET((int16_t)(int16_t) -31997, PH.base.pack) ;
    p90_vy_SET((int16_t)(int16_t) -15941, PH.base.pack) ;
    p90_vz_SET((int16_t)(int16_t)22551, PH.base.pack) ;
    p90_xacc_SET((int16_t)(int16_t)22476, PH.base.pack) ;
    p90_yacc_SET((int16_t)(int16_t)6800, PH.base.pack) ;
    p90_zacc_SET((int16_t)(int16_t) -25037, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_CONTROLS_91(), &PH);
    p91_time_usec_SET((uint64_t)7732271529539508498L, PH.base.pack) ;
    p91_roll_ailerons_SET((float) -2.8625692E38F, PH.base.pack) ;
    p91_pitch_elevator_SET((float) -1.4268202E38F, PH.base.pack) ;
    p91_yaw_rudder_SET((float)6.565701E37F, PH.base.pack) ;
    p91_throttle_SET((float) -2.3522882E37F, PH.base.pack) ;
    p91_aux1_SET((float)1.4193492E38F, PH.base.pack) ;
    p91_aux2_SET((float)9.262746E37F, PH.base.pack) ;
    p91_aux3_SET((float)2.4129484E38F, PH.base.pack) ;
    p91_aux4_SET((float) -4.1735789E37F, PH.base.pack) ;
    p91_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_ARMED, PH.base.pack) ;
    p91_nav_mode_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
    p92_time_usec_SET((uint64_t)6971618074789051445L, PH.base.pack) ;
    p92_chan1_raw_SET((uint16_t)(uint16_t)53262, PH.base.pack) ;
    p92_chan2_raw_SET((uint16_t)(uint16_t)58623, PH.base.pack) ;
    p92_chan3_raw_SET((uint16_t)(uint16_t)1030, PH.base.pack) ;
    p92_chan4_raw_SET((uint16_t)(uint16_t)8585, PH.base.pack) ;
    p92_chan5_raw_SET((uint16_t)(uint16_t)41836, PH.base.pack) ;
    p92_chan6_raw_SET((uint16_t)(uint16_t)20541, PH.base.pack) ;
    p92_chan7_raw_SET((uint16_t)(uint16_t)59281, PH.base.pack) ;
    p92_chan8_raw_SET((uint16_t)(uint16_t)30855, PH.base.pack) ;
    p92_chan9_raw_SET((uint16_t)(uint16_t)838, PH.base.pack) ;
    p92_chan10_raw_SET((uint16_t)(uint16_t)18782, PH.base.pack) ;
    p92_chan11_raw_SET((uint16_t)(uint16_t)42039, PH.base.pack) ;
    p92_chan12_raw_SET((uint16_t)(uint16_t)712, PH.base.pack) ;
    p92_rssi_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
    p93_time_usec_SET((uint64_t)2931403255011891248L, PH.base.pack) ;
    {
        float  controls [] =  {-1.5152991E38F, 1.9899883E38F, -8.982685E37F, -3.0773403E38F, -1.2108794E38F, 1.9455514E38F, -2.6770874E37F, -2.2198298E38F, 2.445431E38F, -2.9620451E38F, 1.8363919E38F, -3.2530214E38F, -1.4961726E38F, 9.39317E37F, 2.3742771E38F, -2.5653017E38F};
        p93_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    p93_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED, PH.base.pack) ;
    p93_flags_SET((uint64_t)156194792073919048L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_100(), &PH);
    p100_time_usec_SET((uint64_t)315968538885252363L, PH.base.pack) ;
    p100_sensor_id_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
    p100_flow_x_SET((int16_t)(int16_t) -12336, PH.base.pack) ;
    p100_flow_y_SET((int16_t)(int16_t) -10234, PH.base.pack) ;
    p100_flow_comp_m_x_SET((float) -7.5267855E37F, PH.base.pack) ;
    p100_flow_comp_m_y_SET((float)1.8950458E38F, PH.base.pack) ;
    p100_quality_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
    p100_ground_distance_SET((float)3.3832192E37F, PH.base.pack) ;
    p100_flow_rate_x_SET((float) -2.9338623E38F, &PH) ;
    p100_flow_rate_y_SET((float)2.3533747E38F, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
    p101_usec_SET((uint64_t)7542661154192999010L, PH.base.pack) ;
    p101_x_SET((float) -2.4538838E38F, PH.base.pack) ;
    p101_y_SET((float) -2.4448204E38F, PH.base.pack) ;
    p101_z_SET((float) -1.6563415E38F, PH.base.pack) ;
    p101_roll_SET((float)3.0551684E38F, PH.base.pack) ;
    p101_pitch_SET((float) -1.7277605E38F, PH.base.pack) ;
    p101_yaw_SET((float)2.724749E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
    p102_usec_SET((uint64_t)1362356042743680057L, PH.base.pack) ;
    p102_x_SET((float) -7.589476E37F, PH.base.pack) ;
    p102_y_SET((float) -2.2023646E38F, PH.base.pack) ;
    p102_z_SET((float)1.6397639E38F, PH.base.pack) ;
    p102_roll_SET((float) -1.6832791E38F, PH.base.pack) ;
    p102_pitch_SET((float) -1.3568958E38F, PH.base.pack) ;
    p102_yaw_SET((float)1.9663094E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
    p103_usec_SET((uint64_t)4609143984909615544L, PH.base.pack) ;
    p103_x_SET((float) -1.9290364E38F, PH.base.pack) ;
    p103_y_SET((float)1.2977819E37F, PH.base.pack) ;
    p103_z_SET((float)2.9788712E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
    p104_usec_SET((uint64_t)415039175303270084L, PH.base.pack) ;
    p104_x_SET((float)6.4834E37F, PH.base.pack) ;
    p104_y_SET((float) -7.87705E37F, PH.base.pack) ;
    p104_z_SET((float) -1.5696972E36F, PH.base.pack) ;
    p104_roll_SET((float)2.1166662E38F, PH.base.pack) ;
    p104_pitch_SET((float) -5.363197E37F, PH.base.pack) ;
    p104_yaw_SET((float)7.4668094E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIGHRES_IMU_105(), &PH);
    p105_time_usec_SET((uint64_t)4824634268360031592L, PH.base.pack) ;
    p105_xacc_SET((float) -3.1011654E38F, PH.base.pack) ;
    p105_yacc_SET((float)1.321798E37F, PH.base.pack) ;
    p105_zacc_SET((float)1.7176353E38F, PH.base.pack) ;
    p105_xgyro_SET((float) -8.2172986E37F, PH.base.pack) ;
    p105_ygyro_SET((float)1.6590217E38F, PH.base.pack) ;
    p105_zgyro_SET((float)2.095109E38F, PH.base.pack) ;
    p105_xmag_SET((float)5.2982567E37F, PH.base.pack) ;
    p105_ymag_SET((float) -3.3931843E38F, PH.base.pack) ;
    p105_zmag_SET((float) -4.2498493E37F, PH.base.pack) ;
    p105_abs_pressure_SET((float) -1.8884116E38F, PH.base.pack) ;
    p105_diff_pressure_SET((float)1.4149659E38F, PH.base.pack) ;
    p105_pressure_alt_SET((float) -2.3538485E38F, PH.base.pack) ;
    p105_temperature_SET((float) -2.7567476E38F, PH.base.pack) ;
    p105_fields_updated_SET((uint16_t)(uint16_t)44684, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
    p106_time_usec_SET((uint64_t)3000620590626488241L, PH.base.pack) ;
    p106_sensor_id_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
    p106_integration_time_us_SET((uint32_t)1904420886L, PH.base.pack) ;
    p106_integrated_x_SET((float)1.3861822E38F, PH.base.pack) ;
    p106_integrated_y_SET((float)1.2024848E38F, PH.base.pack) ;
    p106_integrated_xgyro_SET((float) -4.0151502E37F, PH.base.pack) ;
    p106_integrated_ygyro_SET((float)1.9309047E38F, PH.base.pack) ;
    p106_integrated_zgyro_SET((float)1.1077466E36F, PH.base.pack) ;
    p106_temperature_SET((int16_t)(int16_t) -822, PH.base.pack) ;
    p106_quality_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
    p106_time_delta_distance_us_SET((uint32_t)3889033458L, PH.base.pack) ;
    p106_distance_SET((float) -3.9546985E36F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_SENSOR_107(), &PH);
    p107_time_usec_SET((uint64_t)6506649155243504115L, PH.base.pack) ;
    p107_xacc_SET((float)1.5991379E38F, PH.base.pack) ;
    p107_yacc_SET((float) -1.3243685E38F, PH.base.pack) ;
    p107_zacc_SET((float)2.825528E38F, PH.base.pack) ;
    p107_xgyro_SET((float) -1.6446864E38F, PH.base.pack) ;
    p107_ygyro_SET((float)1.6815882E38F, PH.base.pack) ;
    p107_zgyro_SET((float)3.1017362E38F, PH.base.pack) ;
    p107_xmag_SET((float) -2.6977315E38F, PH.base.pack) ;
    p107_ymag_SET((float)2.9377087E38F, PH.base.pack) ;
    p107_zmag_SET((float) -6.0943925E37F, PH.base.pack) ;
    p107_abs_pressure_SET((float)6.1384246E37F, PH.base.pack) ;
    p107_diff_pressure_SET((float) -8.168175E37F, PH.base.pack) ;
    p107_pressure_alt_SET((float) -1.0856461E38F, PH.base.pack) ;
    p107_temperature_SET((float)1.0102293E38F, PH.base.pack) ;
    p107_fields_updated_SET((uint32_t)246030894L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SIM_STATE_108(), &PH);
    p108_q1_SET((float) -2.7161278E38F, PH.base.pack) ;
    p108_q2_SET((float)2.939176E38F, PH.base.pack) ;
    p108_q3_SET((float)2.9763769E38F, PH.base.pack) ;
    p108_q4_SET((float) -6.236526E37F, PH.base.pack) ;
    p108_roll_SET((float) -1.2070721E38F, PH.base.pack) ;
    p108_pitch_SET((float) -2.2893443E38F, PH.base.pack) ;
    p108_yaw_SET((float)2.8413265E38F, PH.base.pack) ;
    p108_xacc_SET((float)7.2899777E37F, PH.base.pack) ;
    p108_yacc_SET((float)1.3829465E38F, PH.base.pack) ;
    p108_zacc_SET((float)1.8888298E38F, PH.base.pack) ;
    p108_xgyro_SET((float)2.2385779E38F, PH.base.pack) ;
    p108_ygyro_SET((float) -2.7610146E38F, PH.base.pack) ;
    p108_zgyro_SET((float) -2.0060729E38F, PH.base.pack) ;
    p108_lat_SET((float) -1.3582641E38F, PH.base.pack) ;
    p108_lon_SET((float) -1.0436942E38F, PH.base.pack) ;
    p108_alt_SET((float)1.1502327E38F, PH.base.pack) ;
    p108_std_dev_horz_SET((float)3.1096893E38F, PH.base.pack) ;
    p108_std_dev_vert_SET((float)3.1402318E38F, PH.base.pack) ;
    p108_vn_SET((float)4.655769E37F, PH.base.pack) ;
    p108_ve_SET((float) -2.7716136E38F, PH.base.pack) ;
    p108_vd_SET((float)3.1804047E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RADIO_STATUS_109(), &PH);
    p109_rssi_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
    p109_remrssi_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
    p109_txbuf_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
    p109_noise_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
    p109_remnoise_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
    p109_rxerrors_SET((uint16_t)(uint16_t)61474, PH.base.pack) ;
    p109_fixed__SET((uint16_t)(uint16_t)50216, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
    p110_target_network_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
    p110_target_system_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
    p110_target_component_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)173, (uint8_t)70, (uint8_t)231, (uint8_t)14, (uint8_t)180, (uint8_t)65, (uint8_t)133, (uint8_t)95, (uint8_t)247, (uint8_t)80, (uint8_t)49, (uint8_t)191, (uint8_t)94, (uint8_t)68, (uint8_t)145, (uint8_t)64, (uint8_t)206, (uint8_t)215, (uint8_t)102, (uint8_t)169, (uint8_t)10, (uint8_t)147, (uint8_t)212, (uint8_t)75, (uint8_t)226, (uint8_t)161, (uint8_t)23, (uint8_t)99, (uint8_t)214, (uint8_t)123, (uint8_t)66, (uint8_t)163, (uint8_t)52, (uint8_t)184, (uint8_t)141, (uint8_t)40, (uint8_t)209, (uint8_t)201, (uint8_t)89, (uint8_t)175, (uint8_t)4, (uint8_t)29, (uint8_t)31, (uint8_t)9, (uint8_t)130, (uint8_t)216, (uint8_t)4, (uint8_t)21, (uint8_t)129, (uint8_t)165, (uint8_t)13, (uint8_t)54, (uint8_t)88, (uint8_t)248, (uint8_t)228, (uint8_t)251, (uint8_t)173, (uint8_t)202, (uint8_t)251, (uint8_t)254, (uint8_t)155, (uint8_t)171, (uint8_t)36, (uint8_t)38, (uint8_t)80, (uint8_t)126, (uint8_t)175, (uint8_t)79, (uint8_t)102, (uint8_t)173, (uint8_t)175, (uint8_t)36, (uint8_t)56, (uint8_t)29, (uint8_t)7, (uint8_t)170, (uint8_t)28, (uint8_t)207, (uint8_t)126, (uint8_t)114, (uint8_t)234, (uint8_t)88, (uint8_t)229, (uint8_t)116, (uint8_t)127, (uint8_t)67, (uint8_t)233, (uint8_t)205, (uint8_t)124, (uint8_t)78, (uint8_t)72, (uint8_t)195, (uint8_t)188, (uint8_t)15, (uint8_t)242, (uint8_t)134, (uint8_t)244, (uint8_t)80, (uint8_t)146, (uint8_t)1, (uint8_t)17, (uint8_t)165, (uint8_t)215, (uint8_t)52, (uint8_t)192, (uint8_t)81, (uint8_t)134, (uint8_t)154, (uint8_t)222, (uint8_t)195, (uint8_t)240, (uint8_t)183, (uint8_t)219, (uint8_t)159, (uint8_t)96, (uint8_t)22, (uint8_t)121, (uint8_t)122, (uint8_t)100, (uint8_t)102, (uint8_t)72, (uint8_t)243, (uint8_t)245, (uint8_t)191, (uint8_t)182, (uint8_t)121, (uint8_t)230, (uint8_t)214, (uint8_t)191, (uint8_t)158, (uint8_t)9, (uint8_t)149, (uint8_t)174, (uint8_t)1, (uint8_t)12, (uint8_t)238, (uint8_t)126, (uint8_t)147, (uint8_t)214, (uint8_t)23, (uint8_t)202, (uint8_t)177, (uint8_t)25, (uint8_t)170, (uint8_t)242, (uint8_t)124, (uint8_t)185, (uint8_t)221, (uint8_t)20, (uint8_t)83, (uint8_t)119, (uint8_t)170, (uint8_t)135, (uint8_t)177, (uint8_t)146, (uint8_t)79, (uint8_t)137, (uint8_t)132, (uint8_t)22, (uint8_t)227, (uint8_t)97, (uint8_t)100, (uint8_t)110, (uint8_t)30, (uint8_t)65, (uint8_t)62, (uint8_t)69, (uint8_t)109, (uint8_t)231, (uint8_t)21, (uint8_t)242, (uint8_t)201, (uint8_t)9, (uint8_t)17, (uint8_t)245, (uint8_t)241, (uint8_t)233, (uint8_t)12, (uint8_t)70, (uint8_t)45, (uint8_t)47, (uint8_t)69, (uint8_t)165, (uint8_t)2, (uint8_t)113, (uint8_t)206, (uint8_t)247, (uint8_t)58, (uint8_t)60, (uint8_t)76, (uint8_t)153, (uint8_t)135, (uint8_t)125, (uint8_t)180, (uint8_t)187, (uint8_t)208, (uint8_t)225, (uint8_t)61, (uint8_t)73, (uint8_t)38, (uint8_t)181, (uint8_t)229, (uint8_t)227, (uint8_t)94, (uint8_t)180, (uint8_t)161, (uint8_t)103, (uint8_t)254, (uint8_t)90, (uint8_t)64, (uint8_t)218, (uint8_t)28, (uint8_t)37, (uint8_t)20, (uint8_t)2, (uint8_t)7, (uint8_t)45, (uint8_t)134, (uint8_t)109, (uint8_t)196, (uint8_t)83, (uint8_t)160, (uint8_t)143, (uint8_t)84, (uint8_t)96, (uint8_t)206, (uint8_t)244, (uint8_t)200, (uint8_t)198, (uint8_t)133, (uint8_t)41, (uint8_t)115, (uint8_t)163, (uint8_t)86, (uint8_t)141, (uint8_t)137, (uint8_t)27, (uint8_t)190, (uint8_t)138, (uint8_t)40, (uint8_t)233, (uint8_t)178, (uint8_t)162, (uint8_t)219, (uint8_t)108, (uint8_t)132, (uint8_t)127, (uint8_t)67, (uint8_t)207, (uint8_t)172, (uint8_t)149};
        p110_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TIMESYNC_111(), &PH);
    p111_tc1_SET((int64_t) -3980886313449807109L, PH.base.pack) ;
    p111_ts1_SET((int64_t)7610126423292089652L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_TRIGGER_112(), &PH);
    p112_time_usec_SET((uint64_t)205729919713430169L, PH.base.pack) ;
    p112_seq_SET((uint32_t)3197568761L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_GPS_113(), &PH);
    p113_time_usec_SET((uint64_t)5964359842029786282L, PH.base.pack) ;
    p113_fix_type_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
    p113_lat_SET((int32_t) -622154592, PH.base.pack) ;
    p113_lon_SET((int32_t) -1786175150, PH.base.pack) ;
    p113_alt_SET((int32_t) -1196262596, PH.base.pack) ;
    p113_eph_SET((uint16_t)(uint16_t)17858, PH.base.pack) ;
    p113_epv_SET((uint16_t)(uint16_t)11519, PH.base.pack) ;
    p113_vel_SET((uint16_t)(uint16_t)12733, PH.base.pack) ;
    p113_vn_SET((int16_t)(int16_t)14523, PH.base.pack) ;
    p113_ve_SET((int16_t)(int16_t)15079, PH.base.pack) ;
    p113_vd_SET((int16_t)(int16_t) -23546, PH.base.pack) ;
    p113_cog_SET((uint16_t)(uint16_t)64410, PH.base.pack) ;
    p113_satellites_visible_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
    p114_time_usec_SET((uint64_t)4034605928141062525L, PH.base.pack) ;
    p114_sensor_id_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
    p114_integration_time_us_SET((uint32_t)3282200633L, PH.base.pack) ;
    p114_integrated_x_SET((float)1.798024E38F, PH.base.pack) ;
    p114_integrated_y_SET((float) -2.2767784E38F, PH.base.pack) ;
    p114_integrated_xgyro_SET((float) -3.1418817E38F, PH.base.pack) ;
    p114_integrated_ygyro_SET((float)2.8264476E37F, PH.base.pack) ;
    p114_integrated_zgyro_SET((float)2.06754E38F, PH.base.pack) ;
    p114_temperature_SET((int16_t)(int16_t) -12969, PH.base.pack) ;
    p114_quality_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
    p114_time_delta_distance_us_SET((uint32_t)1702111188L, PH.base.pack) ;
    p114_distance_SET((float) -1.4571369E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_STATE_QUATERNION_115(), &PH);
    p115_time_usec_SET((uint64_t)3905308898922940306L, PH.base.pack) ;
    {
        float  attitude_quaternion [] =  {8.4595644E37F, 3.287188E38F, -1.0923559E38F, -1.4499375E38F};
        p115_attitude_quaternion_SET(&attitude_quaternion, 0, &PH.base.pack) ;
    }
    p115_rollspeed_SET((float) -1.2127862E38F, PH.base.pack) ;
    p115_pitchspeed_SET((float)9.913432E37F, PH.base.pack) ;
    p115_yawspeed_SET((float) -1.5144088E38F, PH.base.pack) ;
    p115_lat_SET((int32_t) -438518487, PH.base.pack) ;
    p115_lon_SET((int32_t)1850108602, PH.base.pack) ;
    p115_alt_SET((int32_t) -245902254, PH.base.pack) ;
    p115_vx_SET((int16_t)(int16_t) -21969, PH.base.pack) ;
    p115_vy_SET((int16_t)(int16_t) -19538, PH.base.pack) ;
    p115_vz_SET((int16_t)(int16_t)21228, PH.base.pack) ;
    p115_ind_airspeed_SET((uint16_t)(uint16_t)56075, PH.base.pack) ;
    p115_true_airspeed_SET((uint16_t)(uint16_t)27002, PH.base.pack) ;
    p115_xacc_SET((int16_t)(int16_t)29407, PH.base.pack) ;
    p115_yacc_SET((int16_t)(int16_t) -17761, PH.base.pack) ;
    p115_zacc_SET((int16_t)(int16_t) -26272, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU2_116(), &PH);
    p116_time_boot_ms_SET((uint32_t)849653898L, PH.base.pack) ;
    p116_xacc_SET((int16_t)(int16_t)15781, PH.base.pack) ;
    p116_yacc_SET((int16_t)(int16_t)32587, PH.base.pack) ;
    p116_zacc_SET((int16_t)(int16_t) -28479, PH.base.pack) ;
    p116_xgyro_SET((int16_t)(int16_t)26559, PH.base.pack) ;
    p116_ygyro_SET((int16_t)(int16_t)25755, PH.base.pack) ;
    p116_zgyro_SET((int16_t)(int16_t) -26906, PH.base.pack) ;
    p116_xmag_SET((int16_t)(int16_t)12540, PH.base.pack) ;
    p116_ymag_SET((int16_t)(int16_t)2822, PH.base.pack) ;
    p116_zmag_SET((int16_t)(int16_t) -5361, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_LIST_117(), &PH);
    p117_target_system_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
    p117_target_component_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
    p117_start_SET((uint16_t)(uint16_t)11290, PH.base.pack) ;
    p117_end_SET((uint16_t)(uint16_t)3720, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_ENTRY_118(), &PH);
    p118_id_SET((uint16_t)(uint16_t)13888, PH.base.pack) ;
    p118_num_logs_SET((uint16_t)(uint16_t)11030, PH.base.pack) ;
    p118_last_log_num_SET((uint16_t)(uint16_t)6973, PH.base.pack) ;
    p118_time_utc_SET((uint32_t)3135651758L, PH.base.pack) ;
    p118_size_SET((uint32_t)824325759L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_DATA_119(), &PH);
    p119_target_system_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
    p119_target_component_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
    p119_id_SET((uint16_t)(uint16_t)27010, PH.base.pack) ;
    p119_ofs_SET((uint32_t)3040604740L, PH.base.pack) ;
    p119_count_SET((uint32_t)3829680111L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_DATA_120(), &PH);
    p120_id_SET((uint16_t)(uint16_t)28129, PH.base.pack) ;
    p120_ofs_SET((uint32_t)3110261773L, PH.base.pack) ;
    p120_count_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)53, (uint8_t)37, (uint8_t)237, (uint8_t)175, (uint8_t)191, (uint8_t)22, (uint8_t)175, (uint8_t)239, (uint8_t)51, (uint8_t)116, (uint8_t)36, (uint8_t)221, (uint8_t)177, (uint8_t)63, (uint8_t)234, (uint8_t)159, (uint8_t)149, (uint8_t)144, (uint8_t)197, (uint8_t)180, (uint8_t)222, (uint8_t)101, (uint8_t)179, (uint8_t)185, (uint8_t)16, (uint8_t)138, (uint8_t)66, (uint8_t)104, (uint8_t)167, (uint8_t)79, (uint8_t)114, (uint8_t)135, (uint8_t)220, (uint8_t)160, (uint8_t)79, (uint8_t)246, (uint8_t)146, (uint8_t)167, (uint8_t)239, (uint8_t)78, (uint8_t)57, (uint8_t)108, (uint8_t)29, (uint8_t)247, (uint8_t)131, (uint8_t)177, (uint8_t)242, (uint8_t)128, (uint8_t)153, (uint8_t)31, (uint8_t)61, (uint8_t)22, (uint8_t)220, (uint8_t)114, (uint8_t)168, (uint8_t)1, (uint8_t)178, (uint8_t)185, (uint8_t)69, (uint8_t)135, (uint8_t)130, (uint8_t)9, (uint8_t)8, (uint8_t)244, (uint8_t)173, (uint8_t)9, (uint8_t)55, (uint8_t)108, (uint8_t)35, (uint8_t)234, (uint8_t)53, (uint8_t)129, (uint8_t)88, (uint8_t)109, (uint8_t)148, (uint8_t)125, (uint8_t)54, (uint8_t)118, (uint8_t)140, (uint8_t)153, (uint8_t)242, (uint8_t)105, (uint8_t)14, (uint8_t)70, (uint8_t)127, (uint8_t)194, (uint8_t)210, (uint8_t)221, (uint8_t)227, (uint8_t)162};
        p120_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_ERASE_121(), &PH);
    p121_target_system_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
    p121_target_component_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_END_122(), &PH);
    p122_target_system_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
    p122_target_component_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_INJECT_DATA_123(), &PH);
    p123_target_system_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
    p123_target_component_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
    p123_len_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)231, (uint8_t)72, (uint8_t)44, (uint8_t)189, (uint8_t)252, (uint8_t)139, (uint8_t)76, (uint8_t)104, (uint8_t)4, (uint8_t)195, (uint8_t)240, (uint8_t)160, (uint8_t)3, (uint8_t)147, (uint8_t)218, (uint8_t)175, (uint8_t)135, (uint8_t)158, (uint8_t)102, (uint8_t)56, (uint8_t)151, (uint8_t)61, (uint8_t)196, (uint8_t)13, (uint8_t)58, (uint8_t)126, (uint8_t)95, (uint8_t)42, (uint8_t)3, (uint8_t)213, (uint8_t)110, (uint8_t)150, (uint8_t)61, (uint8_t)207, (uint8_t)115, (uint8_t)144, (uint8_t)166, (uint8_t)165, (uint8_t)163, (uint8_t)89, (uint8_t)71, (uint8_t)12, (uint8_t)250, (uint8_t)79, (uint8_t)2, (uint8_t)203, (uint8_t)26, (uint8_t)25, (uint8_t)185, (uint8_t)80, (uint8_t)40, (uint8_t)169, (uint8_t)233, (uint8_t)128, (uint8_t)110, (uint8_t)211, (uint8_t)128, (uint8_t)235, (uint8_t)186, (uint8_t)121, (uint8_t)102, (uint8_t)196, (uint8_t)176, (uint8_t)35, (uint8_t)207, (uint8_t)215, (uint8_t)201, (uint8_t)53, (uint8_t)221, (uint8_t)0, (uint8_t)68, (uint8_t)185, (uint8_t)169, (uint8_t)195, (uint8_t)241, (uint8_t)0, (uint8_t)61, (uint8_t)10, (uint8_t)127, (uint8_t)91, (uint8_t)97, (uint8_t)180, (uint8_t)193, (uint8_t)127, (uint8_t)167, (uint8_t)44, (uint8_t)126, (uint8_t)80, (uint8_t)146, (uint8_t)126, (uint8_t)70, (uint8_t)168, (uint8_t)133, (uint8_t)124, (uint8_t)146, (uint8_t)209, (uint8_t)198, (uint8_t)27, (uint8_t)186, (uint8_t)125, (uint8_t)1, (uint8_t)113, (uint8_t)226, (uint8_t)97, (uint8_t)37, (uint8_t)156, (uint8_t)52, (uint8_t)219, (uint8_t)106, (uint8_t)171};
        p123_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS2_RAW_124(), &PH);
    p124_time_usec_SET((uint64_t)8254552387727231495L, PH.base.pack) ;
    p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX, PH.base.pack) ;
    p124_lat_SET((int32_t)1328205017, PH.base.pack) ;
    p124_lon_SET((int32_t)1756102306, PH.base.pack) ;
    p124_alt_SET((int32_t)899858390, PH.base.pack) ;
    p124_eph_SET((uint16_t)(uint16_t)19730, PH.base.pack) ;
    p124_epv_SET((uint16_t)(uint16_t)42118, PH.base.pack) ;
    p124_vel_SET((uint16_t)(uint16_t)44144, PH.base.pack) ;
    p124_cog_SET((uint16_t)(uint16_t)49652, PH.base.pack) ;
    p124_satellites_visible_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
    p124_dgps_numch_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
    p124_dgps_age_SET((uint32_t)1053082108L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POWER_STATUS_125(), &PH);
    p125_Vcc_SET((uint16_t)(uint16_t)35759, PH.base.pack) ;
    p125_Vservo_SET((uint16_t)(uint16_t)31079, PH.base.pack) ;
    p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_CONTROL_126(), &PH);
    p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2, PH.base.pack) ;
    p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING, PH.base.pack) ;
    p126_timeout_SET((uint16_t)(uint16_t)30559, PH.base.pack) ;
    p126_baudrate_SET((uint32_t)1067857851L, PH.base.pack) ;
    p126_count_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)135, (uint8_t)161, (uint8_t)118, (uint8_t)113, (uint8_t)84, (uint8_t)243, (uint8_t)118, (uint8_t)226, (uint8_t)135, (uint8_t)158, (uint8_t)85, (uint8_t)205, (uint8_t)175, (uint8_t)28, (uint8_t)27, (uint8_t)92, (uint8_t)129, (uint8_t)248, (uint8_t)151, (uint8_t)124, (uint8_t)161, (uint8_t)166, (uint8_t)227, (uint8_t)231, (uint8_t)76, (uint8_t)195, (uint8_t)56, (uint8_t)74, (uint8_t)186, (uint8_t)52, (uint8_t)177, (uint8_t)40, (uint8_t)81, (uint8_t)193, (uint8_t)158, (uint8_t)45, (uint8_t)118, (uint8_t)45, (uint8_t)42, (uint8_t)51, (uint8_t)233, (uint8_t)158, (uint8_t)80, (uint8_t)252, (uint8_t)133, (uint8_t)82, (uint8_t)163, (uint8_t)48, (uint8_t)252, (uint8_t)250, (uint8_t)250, (uint8_t)156, (uint8_t)13, (uint8_t)240, (uint8_t)97, (uint8_t)51, (uint8_t)144, (uint8_t)135, (uint8_t)244, (uint8_t)125, (uint8_t)1, (uint8_t)170, (uint8_t)158, (uint8_t)100, (uint8_t)51, (uint8_t)20, (uint8_t)204, (uint8_t)165, (uint8_t)252, (uint8_t)168};
        p126_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RTK_127(), &PH);
    p127_time_last_baseline_ms_SET((uint32_t)3180326306L, PH.base.pack) ;
    p127_rtk_receiver_id_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
    p127_wn_SET((uint16_t)(uint16_t)50065, PH.base.pack) ;
    p127_tow_SET((uint32_t)2951148881L, PH.base.pack) ;
    p127_rtk_health_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
    p127_rtk_rate_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
    p127_nsats_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
    p127_baseline_coords_type_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
    p127_baseline_a_mm_SET((int32_t) -1401910624, PH.base.pack) ;
    p127_baseline_b_mm_SET((int32_t) -310833770, PH.base.pack) ;
    p127_baseline_c_mm_SET((int32_t) -1156554295, PH.base.pack) ;
    p127_accuracy_SET((uint32_t)2930546254L, PH.base.pack) ;
    p127_iar_num_hypotheses_SET((int32_t) -422543602, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS2_RTK_128(), &PH);
    p128_time_last_baseline_ms_SET((uint32_t)786860486L, PH.base.pack) ;
    p128_rtk_receiver_id_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
    p128_wn_SET((uint16_t)(uint16_t)31929, PH.base.pack) ;
    p128_tow_SET((uint32_t)2645983890L, PH.base.pack) ;
    p128_rtk_health_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
    p128_rtk_rate_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
    p128_nsats_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
    p128_baseline_coords_type_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
    p128_baseline_a_mm_SET((int32_t) -1074851129, PH.base.pack) ;
    p128_baseline_b_mm_SET((int32_t) -1086306682, PH.base.pack) ;
    p128_baseline_c_mm_SET((int32_t) -2146409573, PH.base.pack) ;
    p128_accuracy_SET((uint32_t)991049533L, PH.base.pack) ;
    p128_iar_num_hypotheses_SET((int32_t) -654997869, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU3_129(), &PH);
    p129_time_boot_ms_SET((uint32_t)2276234255L, PH.base.pack) ;
    p129_xacc_SET((int16_t)(int16_t) -6334, PH.base.pack) ;
    p129_yacc_SET((int16_t)(int16_t) -11692, PH.base.pack) ;
    p129_zacc_SET((int16_t)(int16_t)30922, PH.base.pack) ;
    p129_xgyro_SET((int16_t)(int16_t)12504, PH.base.pack) ;
    p129_ygyro_SET((int16_t)(int16_t)30315, PH.base.pack) ;
    p129_zgyro_SET((int16_t)(int16_t) -13412, PH.base.pack) ;
    p129_xmag_SET((int16_t)(int16_t) -32545, PH.base.pack) ;
    p129_ymag_SET((int16_t)(int16_t) -14082, PH.base.pack) ;
    p129_zmag_SET((int16_t)(int16_t) -23584, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
    p130_type_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
    p130_size_SET((uint32_t)1522968564L, PH.base.pack) ;
    p130_width_SET((uint16_t)(uint16_t)13234, PH.base.pack) ;
    p130_height_SET((uint16_t)(uint16_t)64538, PH.base.pack) ;
    p130_packets_SET((uint16_t)(uint16_t)57930, PH.base.pack) ;
    p130_payload_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    p130_jpg_quality_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ENCAPSULATED_DATA_131(), &PH);
    p131_seqnr_SET((uint16_t)(uint16_t)11154, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)129, (uint8_t)103, (uint8_t)42, (uint8_t)34, (uint8_t)39, (uint8_t)175, (uint8_t)99, (uint8_t)135, (uint8_t)175, (uint8_t)175, (uint8_t)225, (uint8_t)227, (uint8_t)23, (uint8_t)32, (uint8_t)70, (uint8_t)36, (uint8_t)243, (uint8_t)213, (uint8_t)216, (uint8_t)182, (uint8_t)142, (uint8_t)235, (uint8_t)176, (uint8_t)12, (uint8_t)129, (uint8_t)22, (uint8_t)213, (uint8_t)13, (uint8_t)180, (uint8_t)206, (uint8_t)129, (uint8_t)207, (uint8_t)13, (uint8_t)111, (uint8_t)199, (uint8_t)52, (uint8_t)37, (uint8_t)194, (uint8_t)232, (uint8_t)23, (uint8_t)105, (uint8_t)124, (uint8_t)232, (uint8_t)135, (uint8_t)179, (uint8_t)148, (uint8_t)23, (uint8_t)35, (uint8_t)163, (uint8_t)41, (uint8_t)202, (uint8_t)107, (uint8_t)156, (uint8_t)17, (uint8_t)67, (uint8_t)165, (uint8_t)248, (uint8_t)174, (uint8_t)88, (uint8_t)0, (uint8_t)105, (uint8_t)163, (uint8_t)23, (uint8_t)51, (uint8_t)138, (uint8_t)97, (uint8_t)204, (uint8_t)202, (uint8_t)126, (uint8_t)64, (uint8_t)188, (uint8_t)58, (uint8_t)183, (uint8_t)123, (uint8_t)194, (uint8_t)140, (uint8_t)1, (uint8_t)61, (uint8_t)0, (uint8_t)152, (uint8_t)46, (uint8_t)143, (uint8_t)69, (uint8_t)135, (uint8_t)104, (uint8_t)96, (uint8_t)50, (uint8_t)100, (uint8_t)247, (uint8_t)217, (uint8_t)69, (uint8_t)101, (uint8_t)40, (uint8_t)183, (uint8_t)219, (uint8_t)19, (uint8_t)61, (uint8_t)18, (uint8_t)104, (uint8_t)115, (uint8_t)10, (uint8_t)186, (uint8_t)7, (uint8_t)46, (uint8_t)8, (uint8_t)21, (uint8_t)241, (uint8_t)249, (uint8_t)100, (uint8_t)172, (uint8_t)250, (uint8_t)149, (uint8_t)127, (uint8_t)21, (uint8_t)195, (uint8_t)6, (uint8_t)169, (uint8_t)203, (uint8_t)69, (uint8_t)132, (uint8_t)65, (uint8_t)24, (uint8_t)180, (uint8_t)158, (uint8_t)199, (uint8_t)31, (uint8_t)80, (uint8_t)157, (uint8_t)150, (uint8_t)200, (uint8_t)26, (uint8_t)64, (uint8_t)30, (uint8_t)69, (uint8_t)119, (uint8_t)146, (uint8_t)238, (uint8_t)8, (uint8_t)188, (uint8_t)248, (uint8_t)159, (uint8_t)25, (uint8_t)75, (uint8_t)186, (uint8_t)109, (uint8_t)217, (uint8_t)29, (uint8_t)19, (uint8_t)6, (uint8_t)236, (uint8_t)20, (uint8_t)21, (uint8_t)149, (uint8_t)204, (uint8_t)170, (uint8_t)10, (uint8_t)65, (uint8_t)148, (uint8_t)118, (uint8_t)204, (uint8_t)251, (uint8_t)247, (uint8_t)244, (uint8_t)45, (uint8_t)181, (uint8_t)67, (uint8_t)112, (uint8_t)42, (uint8_t)107, (uint8_t)65, (uint8_t)108, (uint8_t)148, (uint8_t)128, (uint8_t)163, (uint8_t)56, (uint8_t)45, (uint8_t)168, (uint8_t)48, (uint8_t)18, (uint8_t)194, (uint8_t)169, (uint8_t)71, (uint8_t)37, (uint8_t)139, (uint8_t)29, (uint8_t)136, (uint8_t)17, (uint8_t)135, (uint8_t)227, (uint8_t)33, (uint8_t)119, (uint8_t)92, (uint8_t)39, (uint8_t)253, (uint8_t)137, (uint8_t)178, (uint8_t)154, (uint8_t)233, (uint8_t)58, (uint8_t)87, (uint8_t)251, (uint8_t)225, (uint8_t)245, (uint8_t)235, (uint8_t)62, (uint8_t)66, (uint8_t)252, (uint8_t)243, (uint8_t)196, (uint8_t)57, (uint8_t)25, (uint8_t)17, (uint8_t)38, (uint8_t)67, (uint8_t)154, (uint8_t)30, (uint8_t)51, (uint8_t)31, (uint8_t)210, (uint8_t)56, (uint8_t)34, (uint8_t)116, (uint8_t)235, (uint8_t)133, (uint8_t)93, (uint8_t)25, (uint8_t)121, (uint8_t)164, (uint8_t)155, (uint8_t)249, (uint8_t)188, (uint8_t)183, (uint8_t)184, (uint8_t)148, (uint8_t)4, (uint8_t)117, (uint8_t)33, (uint8_t)78, (uint8_t)76, (uint8_t)232, (uint8_t)210, (uint8_t)74, (uint8_t)85, (uint8_t)141, (uint8_t)235, (uint8_t)71, (uint8_t)161, (uint8_t)156, (uint8_t)194, (uint8_t)150, (uint8_t)223, (uint8_t)94, (uint8_t)71};
        p131_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DISTANCE_SENSOR_132(), &PH);
    p132_time_boot_ms_SET((uint32_t)1509420228L, PH.base.pack) ;
    p132_min_distance_SET((uint16_t)(uint16_t)11241, PH.base.pack) ;
    p132_max_distance_SET((uint16_t)(uint16_t)50487, PH.base.pack) ;
    p132_current_distance_SET((uint16_t)(uint16_t)26004, PH.base.pack) ;
    p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, PH.base.pack) ;
    p132_id_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_PITCH_180, PH.base.pack) ;
    p132_covariance_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_REQUEST_133(), &PH);
    p133_lat_SET((int32_t)1970073817, PH.base.pack) ;
    p133_lon_SET((int32_t) -2116855127, PH.base.pack) ;
    p133_grid_spacing_SET((uint16_t)(uint16_t)40854, PH.base.pack) ;
    p133_mask_SET((uint64_t)7613183817383092242L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_DATA_134(), &PH);
    p134_lat_SET((int32_t)1449544788, PH.base.pack) ;
    p134_lon_SET((int32_t) -1439988899, PH.base.pack) ;
    p134_grid_spacing_SET((uint16_t)(uint16_t)49150, PH.base.pack) ;
    p134_gridbit_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
    {
        int16_t  data_ [] =  {(int16_t)13483, (int16_t) -4232, (int16_t)24164, (int16_t) -14669, (int16_t)31453, (int16_t)7932, (int16_t)24036, (int16_t)4684, (int16_t) -27873, (int16_t)2682, (int16_t)17004, (int16_t)15783, (int16_t) -5542, (int16_t)18735, (int16_t) -1674, (int16_t)28685};
        p134_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_CHECK_135(), &PH);
    p135_lat_SET((int32_t) -1484222998, PH.base.pack) ;
    p135_lon_SET((int32_t) -1394620142, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_REPORT_136(), &PH);
    p136_lat_SET((int32_t) -1383828728, PH.base.pack) ;
    p136_lon_SET((int32_t) -991524830, PH.base.pack) ;
    p136_spacing_SET((uint16_t)(uint16_t)57470, PH.base.pack) ;
    p136_terrain_height_SET((float)2.869799E38F, PH.base.pack) ;
    p136_current_height_SET((float) -1.3228487E38F, PH.base.pack) ;
    p136_pending_SET((uint16_t)(uint16_t)61997, PH.base.pack) ;
    p136_loaded_SET((uint16_t)(uint16_t)63404, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE2_137(), &PH);
    p137_time_boot_ms_SET((uint32_t)3746582913L, PH.base.pack) ;
    p137_press_abs_SET((float)2.3217029E38F, PH.base.pack) ;
    p137_press_diff_SET((float)3.2529922E38F, PH.base.pack) ;
    p137_temperature_SET((int16_t)(int16_t)10432, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATT_POS_MOCAP_138(), &PH);
    p138_time_usec_SET((uint64_t)4501849337318127528L, PH.base.pack) ;
    {
        float  q [] =  {-1.7148913E38F, -2.9687661E38F, -2.014643E38F, 1.5526012E38F};
        p138_q_SET(&q, 0, &PH.base.pack) ;
    }
    p138_x_SET((float)1.9479336E37F, PH.base.pack) ;
    p138_y_SET((float)2.1057926E38F, PH.base.pack) ;
    p138_z_SET((float) -3.0120833E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
    p139_time_usec_SET((uint64_t)5364173618849725497L, PH.base.pack) ;
    p139_group_mlx_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
    p139_target_system_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
    p139_target_component_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
    {
        float  controls [] =  {2.8736566E38F, 2.8038089E37F, 9.67009E37F, -9.793181E37F, -1.6132902E37F, -1.1822442E38F, -1.0494103E38F, -1.9413357E38F};
        p139_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
    p140_time_usec_SET((uint64_t)3687989299770976527L, PH.base.pack) ;
    p140_group_mlx_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
    {
        float  controls [] =  {9.928273E37F, -7.969536E37F, -7.999068E37F, 3.9286183E37F, -1.517251E38F, -2.7778518E38F, 3.2404441E38F, -3.3936374E38F};
        p140_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ALTITUDE_141(), &PH);
    p141_time_usec_SET((uint64_t)6423965067881677834L, PH.base.pack) ;
    p141_altitude_monotonic_SET((float)1.1473465E38F, PH.base.pack) ;
    p141_altitude_amsl_SET((float) -9.666269E37F, PH.base.pack) ;
    p141_altitude_local_SET((float) -3.353236E38F, PH.base.pack) ;
    p141_altitude_relative_SET((float)1.4035228E38F, PH.base.pack) ;
    p141_altitude_terrain_SET((float) -1.1905084E38F, PH.base.pack) ;
    p141_bottom_clearance_SET((float)6.108888E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RESOURCE_REQUEST_142(), &PH);
    p142_request_id_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
    p142_uri_type_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
    {
        uint8_t  uri [] =  {(uint8_t)5, (uint8_t)117, (uint8_t)112, (uint8_t)244, (uint8_t)14, (uint8_t)62, (uint8_t)171, (uint8_t)179, (uint8_t)171, (uint8_t)169, (uint8_t)218, (uint8_t)149, (uint8_t)254, (uint8_t)179, (uint8_t)54, (uint8_t)3, (uint8_t)155, (uint8_t)247, (uint8_t)152, (uint8_t)101, (uint8_t)54, (uint8_t)242, (uint8_t)247, (uint8_t)109, (uint8_t)193, (uint8_t)73, (uint8_t)161, (uint8_t)65, (uint8_t)91, (uint8_t)92, (uint8_t)194, (uint8_t)210, (uint8_t)154, (uint8_t)168, (uint8_t)177, (uint8_t)110, (uint8_t)166, (uint8_t)168, (uint8_t)99, (uint8_t)15, (uint8_t)246, (uint8_t)223, (uint8_t)98, (uint8_t)94, (uint8_t)72, (uint8_t)117, (uint8_t)58, (uint8_t)97, (uint8_t)85, (uint8_t)228, (uint8_t)69, (uint8_t)40, (uint8_t)118, (uint8_t)245, (uint8_t)152, (uint8_t)254, (uint8_t)178, (uint8_t)122, (uint8_t)178, (uint8_t)238, (uint8_t)137, (uint8_t)207, (uint8_t)72, (uint8_t)108, (uint8_t)180, (uint8_t)82, (uint8_t)193, (uint8_t)191, (uint8_t)83, (uint8_t)120, (uint8_t)82, (uint8_t)222, (uint8_t)136, (uint8_t)187, (uint8_t)190, (uint8_t)106, (uint8_t)235, (uint8_t)163, (uint8_t)34, (uint8_t)186, (uint8_t)198, (uint8_t)192, (uint8_t)203, (uint8_t)167, (uint8_t)232, (uint8_t)172, (uint8_t)27, (uint8_t)237, (uint8_t)193, (uint8_t)71, (uint8_t)163, (uint8_t)120, (uint8_t)114, (uint8_t)126, (uint8_t)168, (uint8_t)220, (uint8_t)144, (uint8_t)229, (uint8_t)125, (uint8_t)54, (uint8_t)232, (uint8_t)159, (uint8_t)185, (uint8_t)63, (uint8_t)169, (uint8_t)124, (uint8_t)238, (uint8_t)150, (uint8_t)203, (uint8_t)115, (uint8_t)183, (uint8_t)165, (uint8_t)119, (uint8_t)184, (uint8_t)179, (uint8_t)49, (uint8_t)73, (uint8_t)40, (uint8_t)240, (uint8_t)249};
        p142_uri_SET(&uri, 0, &PH.base.pack) ;
    }
    p142_transfer_type_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
    {
        uint8_t  storage [] =  {(uint8_t)102, (uint8_t)99, (uint8_t)224, (uint8_t)222, (uint8_t)29, (uint8_t)180, (uint8_t)124, (uint8_t)40, (uint8_t)157, (uint8_t)38, (uint8_t)160, (uint8_t)118, (uint8_t)42, (uint8_t)163, (uint8_t)253, (uint8_t)46, (uint8_t)206, (uint8_t)228, (uint8_t)21, (uint8_t)252, (uint8_t)111, (uint8_t)58, (uint8_t)114, (uint8_t)7, (uint8_t)114, (uint8_t)77, (uint8_t)116, (uint8_t)28, (uint8_t)83, (uint8_t)175, (uint8_t)182, (uint8_t)143, (uint8_t)106, (uint8_t)237, (uint8_t)12, (uint8_t)173, (uint8_t)93, (uint8_t)208, (uint8_t)61, (uint8_t)184, (uint8_t)164, (uint8_t)163, (uint8_t)232, (uint8_t)144, (uint8_t)70, (uint8_t)205, (uint8_t)93, (uint8_t)155, (uint8_t)73, (uint8_t)232, (uint8_t)219, (uint8_t)34, (uint8_t)159, (uint8_t)20, (uint8_t)83, (uint8_t)188, (uint8_t)232, (uint8_t)146, (uint8_t)139, (uint8_t)120, (uint8_t)54, (uint8_t)96, (uint8_t)52, (uint8_t)215, (uint8_t)235, (uint8_t)115, (uint8_t)134, (uint8_t)104, (uint8_t)168, (uint8_t)213, (uint8_t)216, (uint8_t)227, (uint8_t)253, (uint8_t)253, (uint8_t)248, (uint8_t)157, (uint8_t)179, (uint8_t)243, (uint8_t)217, (uint8_t)100, (uint8_t)179, (uint8_t)55, (uint8_t)200, (uint8_t)184, (uint8_t)67, (uint8_t)74, (uint8_t)164, (uint8_t)99, (uint8_t)103, (uint8_t)173, (uint8_t)89, (uint8_t)186, (uint8_t)183, (uint8_t)243, (uint8_t)249, (uint8_t)163, (uint8_t)81, (uint8_t)157, (uint8_t)170, (uint8_t)148, (uint8_t)232, (uint8_t)215, (uint8_t)42, (uint8_t)232, (uint8_t)101, (uint8_t)199, (uint8_t)49, (uint8_t)216, (uint8_t)249, (uint8_t)194, (uint8_t)137, (uint8_t)94, (uint8_t)104, (uint8_t)31, (uint8_t)108, (uint8_t)243, (uint8_t)111, (uint8_t)173, (uint8_t)10, (uint8_t)132};
        p142_storage_SET(&storage, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE3_143(), &PH);
    p143_time_boot_ms_SET((uint32_t)377737739L, PH.base.pack) ;
    p143_press_abs_SET((float)1.7633417E38F, PH.base.pack) ;
    p143_press_diff_SET((float)2.5050035E38F, PH.base.pack) ;
    p143_temperature_SET((int16_t)(int16_t) -22350, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FOLLOW_TARGET_144(), &PH);
    p144_timestamp_SET((uint64_t)4537212384804477906L, PH.base.pack) ;
    p144_est_capabilities_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
    p144_lat_SET((int32_t)658731865, PH.base.pack) ;
    p144_lon_SET((int32_t) -742735328, PH.base.pack) ;
    p144_alt_SET((float)2.7883924E38F, PH.base.pack) ;
    {
        float  vel [] =  {2.6351752E38F, -7.208423E36F, 1.2273383E38F};
        p144_vel_SET(&vel, 0, &PH.base.pack) ;
    }
    {
        float  acc [] =  {-4.785494E37F, 3.2916629E38F, -2.770703E38F};
        p144_acc_SET(&acc, 0, &PH.base.pack) ;
    }
    {
        float  attitude_q [] =  {2.8888246E38F, -6.993062E37F, 3.3563828E38F, 2.0208452E38F};
        p144_attitude_q_SET(&attitude_q, 0, &PH.base.pack) ;
    }
    {
        float  rates [] =  {-5.114614E37F, -3.384919E38F, 1.4898923E38F};
        p144_rates_SET(&rates, 0, &PH.base.pack) ;
    }
    {
        float  position_cov [] =  {-2.2217727E38F, -3.1215584E38F, -2.4131386E38F};
        p144_position_cov_SET(&position_cov, 0, &PH.base.pack) ;
    }
    p144_custom_state_SET((uint64_t)811507403200394647L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
    p146_time_usec_SET((uint64_t)3785781016630243354L, PH.base.pack) ;
    p146_x_acc_SET((float) -2.5439953E38F, PH.base.pack) ;
    p146_y_acc_SET((float)1.9022627E38F, PH.base.pack) ;
    p146_z_acc_SET((float)1.3545043E38F, PH.base.pack) ;
    p146_x_vel_SET((float) -8.235197E37F, PH.base.pack) ;
    p146_y_vel_SET((float)1.0014878E38F, PH.base.pack) ;
    p146_z_vel_SET((float)1.1996183E38F, PH.base.pack) ;
    p146_x_pos_SET((float) -8.527213E37F, PH.base.pack) ;
    p146_y_pos_SET((float) -2.2226452E38F, PH.base.pack) ;
    p146_z_pos_SET((float) -2.148288E38F, PH.base.pack) ;
    p146_airspeed_SET((float)1.2302272E38F, PH.base.pack) ;
    {
        float  vel_variance [] =  {1.2406784E37F, 1.8381445E37F, 2.9045313E37F};
        p146_vel_variance_SET(&vel_variance, 0, &PH.base.pack) ;
    }
    {
        float  pos_variance [] =  {-4.151832E37F, -2.9220107E37F, -3.212136E38F};
        p146_pos_variance_SET(&pos_variance, 0, &PH.base.pack) ;
    }
    {
        float  q [] =  {-6.31757E37F, -4.6367978E36F, -1.2173845E38F, -2.937482E38F};
        p146_q_SET(&q, 0, &PH.base.pack) ;
    }
    p146_roll_rate_SET((float)3.5987495E37F, PH.base.pack) ;
    p146_pitch_rate_SET((float) -2.7760901E38F, PH.base.pack) ;
    p146_yaw_rate_SET((float) -1.7520381E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_BATTERY_STATUS_147(), &PH);
    p147_id_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
    p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_AVIONICS, PH.base.pack) ;
    p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_NIMH, PH.base.pack) ;
    p147_temperature_SET((int16_t)(int16_t) -26649, PH.base.pack) ;
    {
        uint16_t  voltages [] =  {(uint16_t)46054, (uint16_t)11433, (uint16_t)4657, (uint16_t)54376, (uint16_t)43622, (uint16_t)34538, (uint16_t)235, (uint16_t)60055, (uint16_t)55435, (uint16_t)5569};
        p147_voltages_SET(&voltages, 0, &PH.base.pack) ;
    }
    p147_current_battery_SET((int16_t)(int16_t) -19160, PH.base.pack) ;
    p147_current_consumed_SET((int32_t)600798142, PH.base.pack) ;
    p147_energy_consumed_SET((int32_t)1419261104, PH.base.pack) ;
    p147_battery_remaining_SET((int8_t)(int8_t) -24, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_148(), &PH);
    p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT, PH.base.pack) ;
    p148_flight_sw_version_SET((uint32_t)1338915238L, PH.base.pack) ;
    p148_middleware_sw_version_SET((uint32_t)943697850L, PH.base.pack) ;
    p148_os_sw_version_SET((uint32_t)44405789L, PH.base.pack) ;
    p148_board_version_SET((uint32_t)757427731L, PH.base.pack) ;
    {
        uint8_t  flight_custom_version [] =  {(uint8_t)91, (uint8_t)207, (uint8_t)87, (uint8_t)89, (uint8_t)44, (uint8_t)166, (uint8_t)134, (uint8_t)214};
        p148_flight_custom_version_SET(&flight_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  middleware_custom_version [] =  {(uint8_t)27, (uint8_t)168, (uint8_t)77, (uint8_t)198, (uint8_t)150, (uint8_t)125, (uint8_t)196, (uint8_t)102};
        p148_middleware_custom_version_SET(&middleware_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  os_custom_version [] =  {(uint8_t)211, (uint8_t)147, (uint8_t)183, (uint8_t)62, (uint8_t)243, (uint8_t)97, (uint8_t)84, (uint8_t)190};
        p148_os_custom_version_SET(&os_custom_version, 0, &PH.base.pack) ;
    }
    p148_vendor_id_SET((uint16_t)(uint16_t)44775, PH.base.pack) ;
    p148_product_id_SET((uint16_t)(uint16_t)58376, PH.base.pack) ;
    p148_uid_SET((uint64_t)5289393542628390383L, PH.base.pack) ;
    {
        uint8_t  uid2 [] =  {(uint8_t)199, (uint8_t)172, (uint8_t)132, (uint8_t)131, (uint8_t)87, (uint8_t)24, (uint8_t)154, (uint8_t)43, (uint8_t)24, (uint8_t)68, (uint8_t)244, (uint8_t)209, (uint8_t)82, (uint8_t)88, (uint8_t)181, (uint8_t)0, (uint8_t)147, (uint8_t)71};
        p148_uid2_SET(&uid2, 0,  &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LANDING_TARGET_149(), &PH);
    p149_time_usec_SET((uint64_t)3813840326933737929L, PH.base.pack) ;
    p149_target_num_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
    p149_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
    p149_angle_x_SET((float) -2.8059665E38F, PH.base.pack) ;
    p149_angle_y_SET((float)2.1258397E38F, PH.base.pack) ;
    p149_distance_SET((float) -2.782728E37F, PH.base.pack) ;
    p149_size_x_SET((float)1.7039204E37F, PH.base.pack) ;
    p149_size_y_SET((float) -2.7427951E38F, PH.base.pack) ;
    p149_x_SET((float)1.4259117E38F, &PH) ;
    p149_y_SET((float) -1.8524891E37F, &PH) ;
    p149_z_SET((float) -9.698043E37F, &PH) ;
    {
        float  q [] =  {-1.5179417E38F, -2.3982006E38F, 3.0906113E37F, -1.214373E38F};
        p149_q_SET(&q, 0,  &PH) ;
    }
    p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_RADIO_BEACON, PH.base.pack) ;
    p149_position_valid_SET((uint8_t)(uint8_t)246, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ARRAY_TEST_0_150(), &PH);
    p150_v1_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
    {
        int8_t  ar_i8 [] =  {(int8_t) -93, (int8_t) -95, (int8_t)14, (int8_t)39};
        p150_ar_i8_SET(&ar_i8, 0, &PH.base.pack) ;
    }
    {
        uint8_t  ar_u8 [] =  {(uint8_t)96, (uint8_t)17, (uint8_t)104, (uint8_t)251};
        p150_ar_u8_SET(&ar_u8, 0, &PH.base.pack) ;
    }
    {
        uint16_t  ar_u16 [] =  {(uint16_t)2081, (uint16_t)32322, (uint16_t)12802, (uint16_t)7737};
        p150_ar_u16_SET(&ar_u16, 0, &PH.base.pack) ;
    }
    {
        uint32_t  ar_u32 [] =  {3392259534L, 109750112L, 3647685037L, 2141614743L};
        p150_ar_u32_SET(&ar_u32, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ARRAY_TEST_1_151(), &PH);
    {
        uint32_t  ar_u32 [] =  {3644095272L, 3459378478L, 2829693583L, 3009485134L};
        p151_ar_u32_SET(&ar_u32, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ARRAY_TEST_3_153(), &PH);
    p153_v_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
    {
        uint32_t  ar_u32 [] =  {3819112638L, 2445406262L, 3612853548L, 1563662428L};
        p153_ar_u32_SET(&ar_u32, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ARRAY_TEST_4_154(), &PH);
    {
        uint32_t  ar_u32 [] =  {3639469009L, 4093252822L, 1363556654L, 3264273378L};
        p154_ar_u32_SET(&ar_u32, 0, &PH.base.pack) ;
    }
    p154_v_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ARRAY_TEST_5_155(), &PH);
    {
        char16_t   c1 = "vhj";
        p155_c1_SET(&c1, 0,  sizeof(c1), &PH) ;
    }
    {
        char16_t   c2 = "snv";
        p155_c2_SET(&c2, 0,  sizeof(c2), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ARRAY_TEST_6_156(), &PH);
    p156_v1_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
    p156_v2_SET((uint16_t)(uint16_t)30940, PH.base.pack) ;
    p156_v3_SET((uint32_t)1705895054L, PH.base.pack) ;
    {
        uint32_t  ar_u32 [] =  {686306295L, 622389201L};
        p156_ar_u32_SET(&ar_u32, 0, &PH.base.pack) ;
    }
    {
        int32_t  ar_i32 [] =  {-426545172, 813175261};
        p156_ar_i32_SET(&ar_i32, 0, &PH.base.pack) ;
    }
    {
        uint16_t  ar_u16 [] =  {(uint16_t)44636, (uint16_t)12895};
        p156_ar_u16_SET(&ar_u16, 0, &PH.base.pack) ;
    }
    {
        int16_t  ar_i16 [] =  {(int16_t) -22379, (int16_t) -17356};
        p156_ar_i16_SET(&ar_i16, 0, &PH.base.pack) ;
    }
    {
        uint8_t  ar_u8 [] =  {(uint8_t)229, (uint8_t)244};
        p156_ar_u8_SET(&ar_u8, 0, &PH.base.pack) ;
    }
    {
        int8_t  ar_i8 [] =  {(int8_t) -25, (int8_t) -120};
        p156_ar_i8_SET(&ar_i8, 0, &PH.base.pack) ;
    }
    {
        char16_t   ar_c = "TjcaCrcemfwWxrswkrnl";
        p156_ar_c_SET(&ar_c, 0,  sizeof(ar_c), &PH) ;
    }
    {
        double  ar_d [] =  {-8.079727330277224E307, 1.3821797538163953E308};
        p156_ar_d_SET(&ar_d, 0, &PH.base.pack) ;
    }
    {
        float  ar_f [] =  {2.162456E38F, 5.0127286E37F};
        p156_ar_f_SET(&ar_f, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ARRAY_TEST_7_157(), &PH);
    {
        double  ar_d [] =  {7.269029569623625E307, -9.801297883150842E307};
        p157_ar_d_SET(&ar_d, 0, &PH.base.pack) ;
    }
    {
        float  ar_f [] =  {-3.358535E38F, 1.0519932E38F};
        p157_ar_f_SET(&ar_f, 0, &PH.base.pack) ;
    }
    {
        uint32_t  ar_u32 [] =  {2780950687L, 699205751L};
        p157_ar_u32_SET(&ar_u32, 0, &PH.base.pack) ;
    }
    {
        int32_t  ar_i32 [] =  {534537787, -721672226};
        p157_ar_i32_SET(&ar_i32, 0, &PH.base.pack) ;
    }
    {
        uint16_t  ar_u16 [] =  {(uint16_t)35147, (uint16_t)35108};
        p157_ar_u16_SET(&ar_u16, 0, &PH.base.pack) ;
    }
    {
        int16_t  ar_i16 [] =  {(int16_t)17855, (int16_t)7206};
        p157_ar_i16_SET(&ar_i16, 0, &PH.base.pack) ;
    }
    {
        uint8_t  ar_u8 [] =  {(uint8_t)2, (uint8_t)153};
        p157_ar_u8_SET(&ar_u8, 0, &PH.base.pack) ;
    }
    {
        int8_t  ar_i8 [] =  {(int8_t)0, (int8_t)51};
        p157_ar_i8_SET(&ar_i8, 0, &PH.base.pack) ;
    }
    {
        char16_t   ar_c = "gpbaPmywgmxa";
        p157_ar_c_SET(&ar_c, 0,  sizeof(ar_c), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ARRAY_TEST_8_158(), &PH);
    p158_v3_SET((uint32_t)1578924135L, PH.base.pack) ;
    {
        double  ar_d [] =  {-6.712638822365219E307, -1.0067095873752114E308};
        p158_ar_d_SET(&ar_d, 0, &PH.base.pack) ;
    }
    {
        uint16_t  ar_u16 [] =  {(uint16_t)16911, (uint16_t)29633};
        p158_ar_u16_SET(&ar_u16, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ESTIMATOR_STATUS_230(), &PH);
    p230_time_usec_SET((uint64_t)1184481221881453683L, PH.base.pack) ;
    p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_GPS_GLITCH, PH.base.pack) ;
    p230_vel_ratio_SET((float) -3.4647913E37F, PH.base.pack) ;
    p230_pos_horiz_ratio_SET((float)2.8419877E38F, PH.base.pack) ;
    p230_pos_vert_ratio_SET((float)8.3472576E37F, PH.base.pack) ;
    p230_mag_ratio_SET((float) -8.3938E37F, PH.base.pack) ;
    p230_hagl_ratio_SET((float) -3.0586125E38F, PH.base.pack) ;
    p230_tas_ratio_SET((float)1.0522731E38F, PH.base.pack) ;
    p230_pos_horiz_accuracy_SET((float)4.2865215E37F, PH.base.pack) ;
    p230_pos_vert_accuracy_SET((float) -2.728335E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_WIND_COV_231(), &PH);
    p231_time_usec_SET((uint64_t)237627180124713961L, PH.base.pack) ;
    p231_wind_x_SET((float)1.9127925E38F, PH.base.pack) ;
    p231_wind_y_SET((float)8.815322E37F, PH.base.pack) ;
    p231_wind_z_SET((float)2.817637E38F, PH.base.pack) ;
    p231_var_horiz_SET((float)4.7338774E37F, PH.base.pack) ;
    p231_var_vert_SET((float) -1.0935267E37F, PH.base.pack) ;
    p231_wind_alt_SET((float) -2.4591186E38F, PH.base.pack) ;
    p231_horiz_accuracy_SET((float)2.9708426E38F, PH.base.pack) ;
    p231_vert_accuracy_SET((float)2.0708486E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_INPUT_232(), &PH);
    p232_time_usec_SET((uint64_t)7107643048017843464L, PH.base.pack) ;
    p232_gps_id_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
    p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY, PH.base.pack) ;
    p232_time_week_ms_SET((uint32_t)2833597173L, PH.base.pack) ;
    p232_time_week_SET((uint16_t)(uint16_t)57626, PH.base.pack) ;
    p232_fix_type_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
    p232_lat_SET((int32_t) -1352656034, PH.base.pack) ;
    p232_lon_SET((int32_t) -1062008476, PH.base.pack) ;
    p232_alt_SET((float)2.5995088E38F, PH.base.pack) ;
    p232_hdop_SET((float) -1.2651396E38F, PH.base.pack) ;
    p232_vdop_SET((float)2.7569044E38F, PH.base.pack) ;
    p232_vn_SET((float) -8.4421605E37F, PH.base.pack) ;
    p232_ve_SET((float) -4.52888E37F, PH.base.pack) ;
    p232_vd_SET((float)3.610641E37F, PH.base.pack) ;
    p232_speed_accuracy_SET((float) -4.858261E37F, PH.base.pack) ;
    p232_horiz_accuracy_SET((float)1.5316041E38F, PH.base.pack) ;
    p232_vert_accuracy_SET((float) -3.3805404E38F, PH.base.pack) ;
    p232_satellites_visible_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RTCM_DATA_233(), &PH);
    p233_flags_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
    p233_len_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)97, (uint8_t)153, (uint8_t)12, (uint8_t)52, (uint8_t)64, (uint8_t)162, (uint8_t)224, (uint8_t)31, (uint8_t)133, (uint8_t)60, (uint8_t)138, (uint8_t)194, (uint8_t)229, (uint8_t)122, (uint8_t)93, (uint8_t)134, (uint8_t)166, (uint8_t)26, (uint8_t)85, (uint8_t)210, (uint8_t)23, (uint8_t)147, (uint8_t)142, (uint8_t)93, (uint8_t)56, (uint8_t)120, (uint8_t)237, (uint8_t)130, (uint8_t)119, (uint8_t)174, (uint8_t)76, (uint8_t)145, (uint8_t)216, (uint8_t)182, (uint8_t)71, (uint8_t)23, (uint8_t)213, (uint8_t)205, (uint8_t)90, (uint8_t)13, (uint8_t)248, (uint8_t)178, (uint8_t)115, (uint8_t)145, (uint8_t)210, (uint8_t)68, (uint8_t)29, (uint8_t)150, (uint8_t)23, (uint8_t)221, (uint8_t)169, (uint8_t)241, (uint8_t)84, (uint8_t)7, (uint8_t)93, (uint8_t)193, (uint8_t)57, (uint8_t)211, (uint8_t)168, (uint8_t)4, (uint8_t)219, (uint8_t)182, (uint8_t)126, (uint8_t)70, (uint8_t)187, (uint8_t)4, (uint8_t)58, (uint8_t)95, (uint8_t)163, (uint8_t)58, (uint8_t)25, (uint8_t)182, (uint8_t)84, (uint8_t)201, (uint8_t)102, (uint8_t)208, (uint8_t)154, (uint8_t)29, (uint8_t)186, (uint8_t)62, (uint8_t)181, (uint8_t)1, (uint8_t)157, (uint8_t)88, (uint8_t)10, (uint8_t)40, (uint8_t)53, (uint8_t)44, (uint8_t)72, (uint8_t)196, (uint8_t)14, (uint8_t)231, (uint8_t)99, (uint8_t)82, (uint8_t)8, (uint8_t)14, (uint8_t)14, (uint8_t)226, (uint8_t)25, (uint8_t)110, (uint8_t)46, (uint8_t)91, (uint8_t)248, (uint8_t)29, (uint8_t)145, (uint8_t)249, (uint8_t)150, (uint8_t)255, (uint8_t)95, (uint8_t)91, (uint8_t)236, (uint8_t)2, (uint8_t)222, (uint8_t)91, (uint8_t)152, (uint8_t)55, (uint8_t)195, (uint8_t)248, (uint8_t)165, (uint8_t)158, (uint8_t)6, (uint8_t)110, (uint8_t)135, (uint8_t)40, (uint8_t)197, (uint8_t)78, (uint8_t)90, (uint8_t)11, (uint8_t)15, (uint8_t)177, (uint8_t)206, (uint8_t)222, (uint8_t)58, (uint8_t)100, (uint8_t)252, (uint8_t)219, (uint8_t)240, (uint8_t)160, (uint8_t)243, (uint8_t)157, (uint8_t)24, (uint8_t)97, (uint8_t)133, (uint8_t)152, (uint8_t)78, (uint8_t)20, (uint8_t)69, (uint8_t)249, (uint8_t)47, (uint8_t)37, (uint8_t)146, (uint8_t)173, (uint8_t)26, (uint8_t)146, (uint8_t)51, (uint8_t)148, (uint8_t)142, (uint8_t)66, (uint8_t)208, (uint8_t)51, (uint8_t)101, (uint8_t)118, (uint8_t)83, (uint8_t)57, (uint8_t)197, (uint8_t)71, (uint8_t)62, (uint8_t)45, (uint8_t)141, (uint8_t)221, (uint8_t)54, (uint8_t)201, (uint8_t)59, (uint8_t)202, (uint8_t)238, (uint8_t)143, (uint8_t)42, (uint8_t)43, (uint8_t)79, (uint8_t)243};
        p233_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIGH_LATENCY_234(), &PH);
    p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED, PH.base.pack) ;
    p234_custom_mode_SET((uint32_t)3852598935L, PH.base.pack) ;
    p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, PH.base.pack) ;
    p234_roll_SET((int16_t)(int16_t)23736, PH.base.pack) ;
    p234_pitch_SET((int16_t)(int16_t)19860, PH.base.pack) ;
    p234_heading_SET((uint16_t)(uint16_t)3070, PH.base.pack) ;
    p234_throttle_SET((int8_t)(int8_t) -93, PH.base.pack) ;
    p234_heading_sp_SET((int16_t)(int16_t) -15977, PH.base.pack) ;
    p234_latitude_SET((int32_t)344434564, PH.base.pack) ;
    p234_longitude_SET((int32_t) -53874850, PH.base.pack) ;
    p234_altitude_amsl_SET((int16_t)(int16_t)83, PH.base.pack) ;
    p234_altitude_sp_SET((int16_t)(int16_t) -7056, PH.base.pack) ;
    p234_airspeed_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
    p234_airspeed_sp_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
    p234_groundspeed_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
    p234_climb_rate_SET((int8_t)(int8_t)52, PH.base.pack) ;
    p234_gps_nsat_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
    p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, PH.base.pack) ;
    p234_battery_remaining_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
    p234_temperature_SET((int8_t)(int8_t)39, PH.base.pack) ;
    p234_temperature_air_SET((int8_t)(int8_t)126, PH.base.pack) ;
    p234_failsafe_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
    p234_wp_num_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
    p234_wp_distance_SET((uint16_t)(uint16_t)13944, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VIBRATION_241(), &PH);
    p241_time_usec_SET((uint64_t)5834234529272122548L, PH.base.pack) ;
    p241_vibration_x_SET((float) -1.413422E37F, PH.base.pack) ;
    p241_vibration_y_SET((float)1.1366548E38F, PH.base.pack) ;
    p241_vibration_z_SET((float) -1.7821949E37F, PH.base.pack) ;
    p241_clipping_0_SET((uint32_t)1310666738L, PH.base.pack) ;
    p241_clipping_1_SET((uint32_t)108500479L, PH.base.pack) ;
    p241_clipping_2_SET((uint32_t)341406795L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HOME_POSITION_242(), &PH);
    p242_latitude_SET((int32_t) -1637694920, PH.base.pack) ;
    p242_longitude_SET((int32_t)1708589720, PH.base.pack) ;
    p242_altitude_SET((int32_t)1985636678, PH.base.pack) ;
    p242_x_SET((float)2.3629034E38F, PH.base.pack) ;
    p242_y_SET((float) -3.2035502E38F, PH.base.pack) ;
    p242_z_SET((float) -3.2604436E38F, PH.base.pack) ;
    {
        float  q [] =  {-2.9734098E38F, 1.6215007E38F, -8.3688594E37F, 2.5234632E38F};
        p242_q_SET(&q, 0, &PH.base.pack) ;
    }
    p242_approach_x_SET((float)3.440815E37F, PH.base.pack) ;
    p242_approach_y_SET((float) -2.3566606E37F, PH.base.pack) ;
    p242_approach_z_SET((float)3.0596098E38F, PH.base.pack) ;
    p242_time_usec_SET((uint64_t)7790518771225965510L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_HOME_POSITION_243(), &PH);
    p243_target_system_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
    p243_latitude_SET((int32_t)143609524, PH.base.pack) ;
    p243_longitude_SET((int32_t)1642400354, PH.base.pack) ;
    p243_altitude_SET((int32_t) -1993343925, PH.base.pack) ;
    p243_x_SET((float)2.3780957E38F, PH.base.pack) ;
    p243_y_SET((float)2.3762265E38F, PH.base.pack) ;
    p243_z_SET((float) -1.3878898E37F, PH.base.pack) ;
    {
        float  q [] =  {2.219258E38F, -2.2659536E38F, 9.990825E37F, -3.156974E38F};
        p243_q_SET(&q, 0, &PH.base.pack) ;
    }
    p243_approach_x_SET((float) -1.8851757E38F, PH.base.pack) ;
    p243_approach_y_SET((float) -1.3600812E38F, PH.base.pack) ;
    p243_approach_z_SET((float)7.158145E37F, PH.base.pack) ;
    p243_time_usec_SET((uint64_t)7400060130955744519L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MESSAGE_INTERVAL_244(), &PH);
    p244_message_id_SET((uint16_t)(uint16_t)56980, PH.base.pack) ;
    p244_interval_us_SET((int32_t) -1554820642, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_EXTENDED_SYS_STATE_245(), &PH);
    p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW, PH.base.pack) ;
    p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ADSB_VEHICLE_246(), &PH);
    p246_ICAO_address_SET((uint32_t)2570021296L, PH.base.pack) ;
    p246_lat_SET((int32_t)103348676, PH.base.pack) ;
    p246_lon_SET((int32_t)1193531001, PH.base.pack) ;
    p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
    p246_altitude_SET((int32_t)42946194, PH.base.pack) ;
    p246_heading_SET((uint16_t)(uint16_t)45690, PH.base.pack) ;
    p246_hor_velocity_SET((uint16_t)(uint16_t)14165, PH.base.pack) ;
    p246_ver_velocity_SET((int16_t)(int16_t)17986, PH.base.pack) ;
    {
        char16_t   callsign = "czrb";
        p246_callsign_SET(&callsign, 0,  sizeof(callsign), &PH) ;
    }
    p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_LIGHT, PH.base.pack) ;
    p246_tslc_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
    p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_SIMULATED, PH.base.pack) ;
    p246_squawk_SET((uint16_t)(uint16_t)62419, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COLLISION_247(), &PH);
    p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, PH.base.pack) ;
    p247_id_SET((uint32_t)4146480051L, PH.base.pack) ;
    p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_ASCEND_OR_DESCEND, PH.base.pack) ;
    p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW, PH.base.pack) ;
    p247_time_to_minimum_delta_SET((float)7.0220344E37F, PH.base.pack) ;
    p247_altitude_minimum_delta_SET((float)3.1815231E38F, PH.base.pack) ;
    p247_horizontal_minimum_delta_SET((float) -1.8119185E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_V2_EXTENSION_248(), &PH);
    p248_target_network_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
    p248_target_system_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    p248_target_component_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
    p248_message_type_SET((uint16_t)(uint16_t)38301, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)43, (uint8_t)26, (uint8_t)7, (uint8_t)96, (uint8_t)186, (uint8_t)217, (uint8_t)127, (uint8_t)196, (uint8_t)118, (uint8_t)65, (uint8_t)45, (uint8_t)135, (uint8_t)71, (uint8_t)60, (uint8_t)101, (uint8_t)128, (uint8_t)157, (uint8_t)128, (uint8_t)236, (uint8_t)236, (uint8_t)35, (uint8_t)88, (uint8_t)204, (uint8_t)83, (uint8_t)60, (uint8_t)255, (uint8_t)82, (uint8_t)176, (uint8_t)71, (uint8_t)6, (uint8_t)92, (uint8_t)243, (uint8_t)122, (uint8_t)145, (uint8_t)85, (uint8_t)107, (uint8_t)177, (uint8_t)110, (uint8_t)32, (uint8_t)6, (uint8_t)209, (uint8_t)140, (uint8_t)120, (uint8_t)140, (uint8_t)192, (uint8_t)94, (uint8_t)75, (uint8_t)248, (uint8_t)152, (uint8_t)1, (uint8_t)251, (uint8_t)29, (uint8_t)36, (uint8_t)140, (uint8_t)248, (uint8_t)242, (uint8_t)131, (uint8_t)68, (uint8_t)85, (uint8_t)95, (uint8_t)190, (uint8_t)31, (uint8_t)234, (uint8_t)53, (uint8_t)177, (uint8_t)28, (uint8_t)111, (uint8_t)194, (uint8_t)178, (uint8_t)212, (uint8_t)28, (uint8_t)121, (uint8_t)166, (uint8_t)20, (uint8_t)146, (uint8_t)150, (uint8_t)165, (uint8_t)24, (uint8_t)22, (uint8_t)239, (uint8_t)64, (uint8_t)105, (uint8_t)47, (uint8_t)121, (uint8_t)167, (uint8_t)229, (uint8_t)144, (uint8_t)148, (uint8_t)21, (uint8_t)95, (uint8_t)232, (uint8_t)39, (uint8_t)91, (uint8_t)177, (uint8_t)150, (uint8_t)119, (uint8_t)21, (uint8_t)240, (uint8_t)230, (uint8_t)241, (uint8_t)245, (uint8_t)168, (uint8_t)209, (uint8_t)115, (uint8_t)61, (uint8_t)210, (uint8_t)178, (uint8_t)230, (uint8_t)23, (uint8_t)111, (uint8_t)25, (uint8_t)52, (uint8_t)152, (uint8_t)32, (uint8_t)226, (uint8_t)195, (uint8_t)53, (uint8_t)152, (uint8_t)178, (uint8_t)232, (uint8_t)137, (uint8_t)110, (uint8_t)215, (uint8_t)247, (uint8_t)181, (uint8_t)190, (uint8_t)65, (uint8_t)90, (uint8_t)31, (uint8_t)162, (uint8_t)65, (uint8_t)95, (uint8_t)185, (uint8_t)122, (uint8_t)171, (uint8_t)242, (uint8_t)229, (uint8_t)196, (uint8_t)159, (uint8_t)208, (uint8_t)132, (uint8_t)150, (uint8_t)206, (uint8_t)222, (uint8_t)244, (uint8_t)126, (uint8_t)126, (uint8_t)167, (uint8_t)217, (uint8_t)203, (uint8_t)20, (uint8_t)99, (uint8_t)192, (uint8_t)121, (uint8_t)163, (uint8_t)45, (uint8_t)89, (uint8_t)174, (uint8_t)154, (uint8_t)171, (uint8_t)68, (uint8_t)143, (uint8_t)243, (uint8_t)163, (uint8_t)254, (uint8_t)34, (uint8_t)140, (uint8_t)57, (uint8_t)42, (uint8_t)144, (uint8_t)173, (uint8_t)91, (uint8_t)126, (uint8_t)168, (uint8_t)31, (uint8_t)115, (uint8_t)158, (uint8_t)133, (uint8_t)242, (uint8_t)42, (uint8_t)10, (uint8_t)189, (uint8_t)156, (uint8_t)147, (uint8_t)108, (uint8_t)97, (uint8_t)206, (uint8_t)55, (uint8_t)40, (uint8_t)229, (uint8_t)50, (uint8_t)31, (uint8_t)141, (uint8_t)3, (uint8_t)238, (uint8_t)181, (uint8_t)13, (uint8_t)29, (uint8_t)160, (uint8_t)54, (uint8_t)152, (uint8_t)211, (uint8_t)224, (uint8_t)249, (uint8_t)38, (uint8_t)69, (uint8_t)225, (uint8_t)59, (uint8_t)26, (uint8_t)181, (uint8_t)88, (uint8_t)197, (uint8_t)41, (uint8_t)250, (uint8_t)1, (uint8_t)214, (uint8_t)175, (uint8_t)67, (uint8_t)182, (uint8_t)251, (uint8_t)253, (uint8_t)34, (uint8_t)41, (uint8_t)10, (uint8_t)17, (uint8_t)114, (uint8_t)101, (uint8_t)21, (uint8_t)242, (uint8_t)203, (uint8_t)246, (uint8_t)43, (uint8_t)187, (uint8_t)203, (uint8_t)8, (uint8_t)237, (uint8_t)194, (uint8_t)244, (uint8_t)123, (uint8_t)237, (uint8_t)250, (uint8_t)21, (uint8_t)132, (uint8_t)144, (uint8_t)92, (uint8_t)195, (uint8_t)145, (uint8_t)131, (uint8_t)168};
        p248_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MEMORY_VECT_249(), &PH);
    p249_address_SET((uint16_t)(uint16_t)47708, PH.base.pack) ;
    p249_ver_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
    p249_type_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
    {
        int8_t  value [] =  {(int8_t) -54, (int8_t) -47, (int8_t) -23, (int8_t)59, (int8_t)96, (int8_t) -67, (int8_t)127, (int8_t)62, (int8_t) -49, (int8_t)87, (int8_t)48, (int8_t)88, (int8_t)96, (int8_t)81, (int8_t)110, (int8_t) -126, (int8_t)24, (int8_t) -58, (int8_t) -56, (int8_t) -46, (int8_t) -114, (int8_t)66, (int8_t)76, (int8_t)28, (int8_t)126, (int8_t) -91, (int8_t)6, (int8_t) -48, (int8_t) -67, (int8_t)34, (int8_t)66, (int8_t) -74};
        p249_value_SET(&value, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEBUG_VECT_250(), &PH);
    {
        char16_t   name = "jptjOckbx";
        p250_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p250_time_usec_SET((uint64_t)8142224576643181151L, PH.base.pack) ;
    p250_x_SET((float)1.3218775E38F, PH.base.pack) ;
    p250_y_SET((float)3.3263523E38F, PH.base.pack) ;
    p250_z_SET((float) -1.8633486E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
    p251_time_boot_ms_SET((uint32_t)416098286L, PH.base.pack) ;
    {
        char16_t   name = "xbczkimmj";
        p251_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p251_value_SET((float)2.8427251E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_INT_252(), &PH);
    p252_time_boot_ms_SET((uint32_t)4164865172L, PH.base.pack) ;
    {
        char16_t   name = "al";
        p252_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p252_value_SET((int32_t) -257214593, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_STATUSTEXT_253(), &PH);
    p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_ALERT, PH.base.pack) ;
    {
        char16_t   text = "wk";
        p253_text_SET(&text, 0,  sizeof(text), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEBUG_254(), &PH);
    p254_time_boot_ms_SET((uint32_t)1248052662L, PH.base.pack) ;
    p254_ind_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
    p254_value_SET((float)3.5239455E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SETUP_SIGNING_256(), &PH);
    p256_target_system_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
    p256_target_component_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
    {
        uint8_t  secret_key [] =  {(uint8_t)102, (uint8_t)184, (uint8_t)98, (uint8_t)73, (uint8_t)104, (uint8_t)216, (uint8_t)186, (uint8_t)60, (uint8_t)16, (uint8_t)7, (uint8_t)84, (uint8_t)70, (uint8_t)230, (uint8_t)132, (uint8_t)248, (uint8_t)201, (uint8_t)98, (uint8_t)90, (uint8_t)120, (uint8_t)37, (uint8_t)86, (uint8_t)142, (uint8_t)60, (uint8_t)119, (uint8_t)240, (uint8_t)178, (uint8_t)47, (uint8_t)217, (uint8_t)121, (uint8_t)197, (uint8_t)23, (uint8_t)239};
        p256_secret_key_SET(&secret_key, 0, &PH.base.pack) ;
    }
    p256_initial_timestamp_SET((uint64_t)5199769445767842474L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_BUTTON_CHANGE_257(), &PH);
    p257_time_boot_ms_SET((uint32_t)2858306861L, PH.base.pack) ;
    p257_last_change_ms_SET((uint32_t)1964593800L, PH.base.pack) ;
    p257_state_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PLAY_TUNE_258(), &PH);
    p258_target_system_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
    p258_target_component_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
    {
        char16_t   tune = "yeyqbdjctqcv";
        p258_tune_SET(&tune, 0,  sizeof(tune), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_INFORMATION_259(), &PH);
    p259_time_boot_ms_SET((uint32_t)1572240093L, PH.base.pack) ;
    {
        uint8_t  vendor_name [] =  {(uint8_t)129, (uint8_t)3, (uint8_t)112, (uint8_t)45, (uint8_t)201, (uint8_t)124, (uint8_t)110, (uint8_t)202, (uint8_t)255, (uint8_t)105, (uint8_t)26, (uint8_t)237, (uint8_t)61, (uint8_t)196, (uint8_t)96, (uint8_t)51, (uint8_t)107, (uint8_t)25, (uint8_t)227, (uint8_t)97, (uint8_t)87, (uint8_t)198, (uint8_t)139, (uint8_t)23, (uint8_t)83, (uint8_t)95, (uint8_t)123, (uint8_t)212, (uint8_t)46, (uint8_t)252, (uint8_t)187, (uint8_t)150};
        p259_vendor_name_SET(&vendor_name, 0, &PH.base.pack) ;
    }
    {
        uint8_t  model_name [] =  {(uint8_t)245, (uint8_t)97, (uint8_t)90, (uint8_t)189, (uint8_t)248, (uint8_t)39, (uint8_t)80, (uint8_t)7, (uint8_t)218, (uint8_t)241, (uint8_t)152, (uint8_t)29, (uint8_t)56, (uint8_t)246, (uint8_t)159, (uint8_t)170, (uint8_t)61, (uint8_t)66, (uint8_t)243, (uint8_t)17, (uint8_t)7, (uint8_t)4, (uint8_t)61, (uint8_t)132, (uint8_t)230, (uint8_t)141, (uint8_t)134, (uint8_t)230, (uint8_t)188, (uint8_t)72, (uint8_t)57, (uint8_t)77};
        p259_model_name_SET(&model_name, 0, &PH.base.pack) ;
    }
    p259_firmware_version_SET((uint32_t)3065250216L, PH.base.pack) ;
    p259_focal_length_SET((float) -2.5595014E38F, PH.base.pack) ;
    p259_sensor_size_h_SET((float) -2.4141185E38F, PH.base.pack) ;
    p259_sensor_size_v_SET((float)1.5837928E38F, PH.base.pack) ;
    p259_resolution_h_SET((uint16_t)(uint16_t)12213, PH.base.pack) ;
    p259_resolution_v_SET((uint16_t)(uint16_t)31944, PH.base.pack) ;
    p259_lens_id_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
    p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE, PH.base.pack) ;
    p259_cam_definition_version_SET((uint16_t)(uint16_t)30292, PH.base.pack) ;
    {
        char16_t   cam_definition_uri = "hfjhhmldifFfjpxgrJxbmbntwqammoewcobdeefucomjesvbwstcixjxfbjsXlgpiktftvmujQbca";
        p259_cam_definition_uri_SET(&cam_definition_uri, 0,  sizeof(cam_definition_uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_SETTINGS_260(), &PH);
    p260_time_boot_ms_SET((uint32_t)2946551342L, PH.base.pack) ;
    p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_IMAGE_SURVEY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_STORAGE_INFORMATION_261(), &PH);
    p261_time_boot_ms_SET((uint32_t)1480738194L, PH.base.pack) ;
    p261_storage_id_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
    p261_storage_count_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
    p261_status_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
    p261_total_capacity_SET((float) -1.7325466E38F, PH.base.pack) ;
    p261_used_capacity_SET((float) -1.3202586E38F, PH.base.pack) ;
    p261_available_capacity_SET((float) -5.5610453E37F, PH.base.pack) ;
    p261_read_speed_SET((float) -1.7615828E38F, PH.base.pack) ;
    p261_write_speed_SET((float) -6.952125E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
    p262_time_boot_ms_SET((uint32_t)399388556L, PH.base.pack) ;
    p262_image_status_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
    p262_video_status_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
    p262_image_interval_SET((float)1.1824741E37F, PH.base.pack) ;
    p262_recording_time_ms_SET((uint32_t)1988433053L, PH.base.pack) ;
    p262_available_capacity_SET((float)2.071448E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
    p263_time_boot_ms_SET((uint32_t)609373087L, PH.base.pack) ;
    p263_time_utc_SET((uint64_t)4113460733927720584L, PH.base.pack) ;
    p263_camera_id_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
    p263_lat_SET((int32_t)1803930613, PH.base.pack) ;
    p263_lon_SET((int32_t) -943548550, PH.base.pack) ;
    p263_alt_SET((int32_t)988634043, PH.base.pack) ;
    p263_relative_alt_SET((int32_t) -272746822, PH.base.pack) ;
    {
        float  q [] =  {8.513423E37F, 3.3871721E38F, -2.9394702E38F, 4.306883E37F};
        p263_q_SET(&q, 0, &PH.base.pack) ;
    }
    p263_image_index_SET((int32_t)1406365908, PH.base.pack) ;
    p263_capture_result_SET((int8_t)(int8_t) -115, PH.base.pack) ;
    {
        char16_t   file_url = "qrpcyalRkoRzlrbhtdiaZdgeBdPaatmHmdfdgnJlmqyvtncszolBpgNvfCawslzrrvtrktlrxrqnavbkkyplufdsegxCswcdkEvwnnwoektndbFegvcutkxBkLeslZIwfanoxftznbmcyuQdedovbnlhjHsbGqsfzich";
        p263_file_url_SET(&file_url, 0,  sizeof(file_url), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FLIGHT_INFORMATION_264(), &PH);
    p264_time_boot_ms_SET((uint32_t)3979719226L, PH.base.pack) ;
    p264_arming_time_utc_SET((uint64_t)7019573565446908279L, PH.base.pack) ;
    p264_takeoff_time_utc_SET((uint64_t)3426897956990591750L, PH.base.pack) ;
    p264_flight_uuid_SET((uint64_t)6516643134320855782L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MOUNT_ORIENTATION_265(), &PH);
    p265_time_boot_ms_SET((uint32_t)3447411947L, PH.base.pack) ;
    p265_roll_SET((float)7.639659E37F, PH.base.pack) ;
    p265_pitch_SET((float)4.7942257E37F, PH.base.pack) ;
    p265_yaw_SET((float)1.6018939E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_266(), &PH);
    p266_target_system_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
    p266_target_component_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
    p266_sequence_SET((uint16_t)(uint16_t)45489, PH.base.pack) ;
    p266_length_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    p266_first_message_offset_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)76, (uint8_t)52, (uint8_t)33, (uint8_t)115, (uint8_t)167, (uint8_t)50, (uint8_t)171, (uint8_t)186, (uint8_t)60, (uint8_t)89, (uint8_t)135, (uint8_t)71, (uint8_t)6, (uint8_t)231, (uint8_t)156, (uint8_t)142, (uint8_t)170, (uint8_t)84, (uint8_t)88, (uint8_t)158, (uint8_t)137, (uint8_t)161, (uint8_t)221, (uint8_t)182, (uint8_t)201, (uint8_t)48, (uint8_t)144, (uint8_t)6, (uint8_t)46, (uint8_t)189, (uint8_t)240, (uint8_t)41, (uint8_t)149, (uint8_t)10, (uint8_t)87, (uint8_t)174, (uint8_t)178, (uint8_t)88, (uint8_t)253, (uint8_t)75, (uint8_t)36, (uint8_t)47, (uint8_t)0, (uint8_t)1, (uint8_t)147, (uint8_t)182, (uint8_t)55, (uint8_t)204, (uint8_t)11, (uint8_t)218, (uint8_t)169, (uint8_t)127, (uint8_t)249, (uint8_t)147, (uint8_t)146, (uint8_t)228, (uint8_t)110, (uint8_t)160, (uint8_t)72, (uint8_t)142, (uint8_t)117, (uint8_t)50, (uint8_t)194, (uint8_t)24, (uint8_t)123, (uint8_t)153, (uint8_t)89, (uint8_t)175, (uint8_t)32, (uint8_t)78, (uint8_t)14, (uint8_t)77, (uint8_t)28, (uint8_t)141, (uint8_t)132, (uint8_t)182, (uint8_t)126, (uint8_t)125, (uint8_t)161, (uint8_t)223, (uint8_t)81, (uint8_t)5, (uint8_t)141, (uint8_t)237, (uint8_t)201, (uint8_t)250, (uint8_t)1, (uint8_t)224, (uint8_t)17, (uint8_t)68, (uint8_t)236, (uint8_t)99, (uint8_t)124, (uint8_t)62, (uint8_t)184, (uint8_t)163, (uint8_t)233, (uint8_t)194, (uint8_t)210, (uint8_t)59, (uint8_t)123, (uint8_t)120, (uint8_t)208, (uint8_t)165, (uint8_t)199, (uint8_t)159, (uint8_t)118, (uint8_t)232, (uint8_t)161, (uint8_t)74, (uint8_t)218, (uint8_t)23, (uint8_t)32, (uint8_t)174, (uint8_t)160, (uint8_t)54, (uint8_t)54, (uint8_t)213, (uint8_t)127, (uint8_t)37, (uint8_t)43, (uint8_t)190, (uint8_t)53, (uint8_t)17, (uint8_t)248, (uint8_t)163, (uint8_t)191, (uint8_t)8, (uint8_t)56, (uint8_t)226, (uint8_t)179, (uint8_t)161, (uint8_t)198, (uint8_t)149, (uint8_t)52, (uint8_t)123, (uint8_t)236, (uint8_t)198, (uint8_t)53, (uint8_t)182, (uint8_t)178, (uint8_t)61, (uint8_t)108, (uint8_t)7, (uint8_t)196, (uint8_t)12, (uint8_t)88, (uint8_t)75, (uint8_t)230, (uint8_t)125, (uint8_t)186, (uint8_t)191, (uint8_t)206, (uint8_t)163, (uint8_t)73, (uint8_t)43, (uint8_t)94, (uint8_t)200, (uint8_t)255, (uint8_t)96, (uint8_t)232, (uint8_t)18, (uint8_t)104, (uint8_t)215, (uint8_t)120, (uint8_t)155, (uint8_t)152, (uint8_t)152, (uint8_t)146, (uint8_t)136, (uint8_t)117, (uint8_t)114, (uint8_t)242, (uint8_t)20, (uint8_t)144, (uint8_t)27, (uint8_t)174, (uint8_t)36, (uint8_t)187, (uint8_t)108, (uint8_t)126, (uint8_t)50, (uint8_t)208, (uint8_t)216, (uint8_t)232, (uint8_t)112, (uint8_t)159, (uint8_t)237, (uint8_t)94, (uint8_t)179, (uint8_t)79, (uint8_t)252, (uint8_t)28, (uint8_t)169, (uint8_t)217, (uint8_t)208, (uint8_t)54, (uint8_t)238, (uint8_t)179, (uint8_t)251, (uint8_t)112, (uint8_t)23, (uint8_t)110, (uint8_t)24, (uint8_t)74, (uint8_t)208, (uint8_t)193, (uint8_t)106, (uint8_t)9, (uint8_t)88, (uint8_t)170, (uint8_t)46, (uint8_t)80, (uint8_t)44, (uint8_t)63, (uint8_t)94, (uint8_t)35, (uint8_t)61, (uint8_t)157, (uint8_t)179, (uint8_t)134, (uint8_t)87, (uint8_t)68, (uint8_t)156, (uint8_t)223, (uint8_t)212, (uint8_t)192, (uint8_t)49, (uint8_t)205, (uint8_t)158, (uint8_t)30, (uint8_t)174, (uint8_t)24, (uint8_t)89, (uint8_t)54, (uint8_t)249, (uint8_t)192, (uint8_t)130, (uint8_t)85, (uint8_t)4, (uint8_t)223, (uint8_t)206, (uint8_t)53, (uint8_t)40, (uint8_t)192, (uint8_t)151, (uint8_t)48, (uint8_t)27, (uint8_t)110};
        p266_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_ACKED_267(), &PH);
    p267_target_system_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
    p267_target_component_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
    p267_sequence_SET((uint16_t)(uint16_t)27321, PH.base.pack) ;
    p267_length_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
    p267_first_message_offset_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)123, (uint8_t)42, (uint8_t)198, (uint8_t)4, (uint8_t)124, (uint8_t)248, (uint8_t)31, (uint8_t)120, (uint8_t)36, (uint8_t)127, (uint8_t)84, (uint8_t)163, (uint8_t)99, (uint8_t)9, (uint8_t)55, (uint8_t)51, (uint8_t)0, (uint8_t)40, (uint8_t)84, (uint8_t)223, (uint8_t)101, (uint8_t)237, (uint8_t)187, (uint8_t)65, (uint8_t)52, (uint8_t)148, (uint8_t)97, (uint8_t)52, (uint8_t)197, (uint8_t)244, (uint8_t)78, (uint8_t)16, (uint8_t)224, (uint8_t)187, (uint8_t)227, (uint8_t)190, (uint8_t)18, (uint8_t)96, (uint8_t)175, (uint8_t)80, (uint8_t)11, (uint8_t)60, (uint8_t)87, (uint8_t)46, (uint8_t)1, (uint8_t)83, (uint8_t)148, (uint8_t)106, (uint8_t)29, (uint8_t)24, (uint8_t)192, (uint8_t)33, (uint8_t)163, (uint8_t)108, (uint8_t)228, (uint8_t)239, (uint8_t)116, (uint8_t)21, (uint8_t)144, (uint8_t)118, (uint8_t)149, (uint8_t)56, (uint8_t)123, (uint8_t)70, (uint8_t)111, (uint8_t)194, (uint8_t)68, (uint8_t)144, (uint8_t)198, (uint8_t)94, (uint8_t)248, (uint8_t)195, (uint8_t)233, (uint8_t)237, (uint8_t)94, (uint8_t)73, (uint8_t)121, (uint8_t)36, (uint8_t)30, (uint8_t)9, (uint8_t)17, (uint8_t)102, (uint8_t)37, (uint8_t)208, (uint8_t)169, (uint8_t)13, (uint8_t)104, (uint8_t)103, (uint8_t)180, (uint8_t)206, (uint8_t)124, (uint8_t)183, (uint8_t)12, (uint8_t)123, (uint8_t)239, (uint8_t)175, (uint8_t)220, (uint8_t)37, (uint8_t)102, (uint8_t)73, (uint8_t)248, (uint8_t)31, (uint8_t)43, (uint8_t)192, (uint8_t)210, (uint8_t)83, (uint8_t)22, (uint8_t)213, (uint8_t)213, (uint8_t)227, (uint8_t)227, (uint8_t)88, (uint8_t)228, (uint8_t)76, (uint8_t)112, (uint8_t)130, (uint8_t)223, (uint8_t)195, (uint8_t)26, (uint8_t)254, (uint8_t)168, (uint8_t)40, (uint8_t)5, (uint8_t)251, (uint8_t)95, (uint8_t)34, (uint8_t)79, (uint8_t)233, (uint8_t)244, (uint8_t)135, (uint8_t)219, (uint8_t)69, (uint8_t)220, (uint8_t)193, (uint8_t)206, (uint8_t)210, (uint8_t)63, (uint8_t)8, (uint8_t)208, (uint8_t)79, (uint8_t)244, (uint8_t)157, (uint8_t)106, (uint8_t)216, (uint8_t)50, (uint8_t)70, (uint8_t)127, (uint8_t)128, (uint8_t)57, (uint8_t)85, (uint8_t)174, (uint8_t)156, (uint8_t)33, (uint8_t)223, (uint8_t)63, (uint8_t)143, (uint8_t)9, (uint8_t)84, (uint8_t)84, (uint8_t)73, (uint8_t)66, (uint8_t)205, (uint8_t)35, (uint8_t)113, (uint8_t)43, (uint8_t)150, (uint8_t)253, (uint8_t)80, (uint8_t)152, (uint8_t)124, (uint8_t)24, (uint8_t)241, (uint8_t)250, (uint8_t)89, (uint8_t)127, (uint8_t)7, (uint8_t)9, (uint8_t)182, (uint8_t)144, (uint8_t)213, (uint8_t)210, (uint8_t)4, (uint8_t)27, (uint8_t)234, (uint8_t)138, (uint8_t)11, (uint8_t)92, (uint8_t)91, (uint8_t)92, (uint8_t)146, (uint8_t)124, (uint8_t)217, (uint8_t)53, (uint8_t)96, (uint8_t)30, (uint8_t)113, (uint8_t)194, (uint8_t)205, (uint8_t)142, (uint8_t)51, (uint8_t)118, (uint8_t)145, (uint8_t)240, (uint8_t)243, (uint8_t)17, (uint8_t)167, (uint8_t)59, (uint8_t)204, (uint8_t)252, (uint8_t)196, (uint8_t)189, (uint8_t)133, (uint8_t)144, (uint8_t)179, (uint8_t)69, (uint8_t)166, (uint8_t)67, (uint8_t)172, (uint8_t)149, (uint8_t)134, (uint8_t)173, (uint8_t)193, (uint8_t)170, (uint8_t)66, (uint8_t)236, (uint8_t)12, (uint8_t)250, (uint8_t)96, (uint8_t)123, (uint8_t)211, (uint8_t)122, (uint8_t)117, (uint8_t)151, (uint8_t)170, (uint8_t)101, (uint8_t)16, (uint8_t)130, (uint8_t)96, (uint8_t)172, (uint8_t)133, (uint8_t)196, (uint8_t)74, (uint8_t)70, (uint8_t)46, (uint8_t)11, (uint8_t)26, (uint8_t)35, (uint8_t)110, (uint8_t)106};
        p267_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_ACK_268(), &PH);
    p268_target_system_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
    p268_target_component_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
    p268_sequence_SET((uint16_t)(uint16_t)51672, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
    p269_camera_id_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
    p269_status_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    p269_framerate_SET((float) -8.944181E37F, PH.base.pack) ;
    p269_resolution_h_SET((uint16_t)(uint16_t)13339, PH.base.pack) ;
    p269_resolution_v_SET((uint16_t)(uint16_t)5212, PH.base.pack) ;
    p269_bitrate_SET((uint32_t)2611934233L, PH.base.pack) ;
    p269_rotation_SET((uint16_t)(uint16_t)15528, PH.base.pack) ;
    {
        char16_t   uri = "pToaamjkPzvmcgptNnmendzvleugapepJlrrsjinKzkzhUrvnxEVyilwlyJhjdmtoxhUhntkImebymrglhftyMAwpsiaorSqldFeedwAgvwOpaisioNikfmohIvvsquTkdqXlkyuasYftdjNbrhtCdxgMqqotrcreqdiZhsokxSyqufderRpfgfqzoeitqgmrf";
        p269_uri_SET(&uri, 0,  sizeof(uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
    p270_target_system_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
    p270_target_component_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
    p270_camera_id_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
    p270_framerate_SET((float) -2.8879758E38F, PH.base.pack) ;
    p270_resolution_h_SET((uint16_t)(uint16_t)19899, PH.base.pack) ;
    p270_resolution_v_SET((uint16_t)(uint16_t)10835, PH.base.pack) ;
    p270_bitrate_SET((uint32_t)494913063L, PH.base.pack) ;
    p270_rotation_SET((uint16_t)(uint16_t)12613, PH.base.pack) ;
    {
        char16_t   uri = "UigayfhfqgfRbifbazvbIvtyjeyxcdpaQwdMbwxzhmbyneoniemfsovsqfznmbzwkGixkYuakldktDjpqxeheojWmuvoqa";
        p270_uri_SET(&uri, 0,  sizeof(uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_WIFI_CONFIG_AP_299(), &PH);
    {
        char16_t   ssid = "vstndyragupv";
        p299_ssid_SET(&ssid, 0,  sizeof(ssid), &PH) ;
    }
    {
        char16_t   password = "uqvospyxjlzoztquzfhaazwaeysanAiepvjtngikqrwvpupcmhzrhdLQWn";
        p299_password_SET(&password, 0,  sizeof(password), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PROTOCOL_VERSION_300(), &PH);
    p300_version_SET((uint16_t)(uint16_t)62630, PH.base.pack) ;
    p300_min_version_SET((uint16_t)(uint16_t)9686, PH.base.pack) ;
    p300_max_version_SET((uint16_t)(uint16_t)19480, PH.base.pack) ;
    {
        uint8_t  spec_version_hash [] =  {(uint8_t)224, (uint8_t)255, (uint8_t)83, (uint8_t)177, (uint8_t)38, (uint8_t)143, (uint8_t)161, (uint8_t)56};
        p300_spec_version_hash_SET(&spec_version_hash, 0, &PH.base.pack) ;
    }
    {
        uint8_t  library_version_hash [] =  {(uint8_t)140, (uint8_t)251, (uint8_t)243, (uint8_t)82, (uint8_t)3, (uint8_t)83, (uint8_t)17, (uint8_t)25};
        p300_library_version_hash_SET(&library_version_hash, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_STATUS_310(), &PH);
    p310_time_usec_SET((uint64_t)1215423144249647264L, PH.base.pack) ;
    p310_uptime_sec_SET((uint32_t)739523004L, PH.base.pack) ;
    p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_WARNING, PH.base.pack) ;
    p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE, PH.base.pack) ;
    p310_sub_mode_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
    p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)22897, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_INFO_311(), &PH);
    p311_time_usec_SET((uint64_t)3697228442774539954L, PH.base.pack) ;
    p311_uptime_sec_SET((uint32_t)250707757L, PH.base.pack) ;
    {
        char16_t   name = "aPbuxbvqkfaklimylcrcjhpdiekypwbvclaqbxofjifByRhxhaqhapYtqxl";
        p311_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p311_hw_version_major_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
    p311_hw_version_minor_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
    {
        uint8_t  hw_unique_id [] =  {(uint8_t)51, (uint8_t)7, (uint8_t)140, (uint8_t)251, (uint8_t)229, (uint8_t)175, (uint8_t)210, (uint8_t)79, (uint8_t)178, (uint8_t)233, (uint8_t)11, (uint8_t)208, (uint8_t)53, (uint8_t)248, (uint8_t)89, (uint8_t)15};
        p311_hw_unique_id_SET(&hw_unique_id, 0, &PH.base.pack) ;
    }
    p311_sw_version_major_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
    p311_sw_version_minor_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
    p311_sw_vcs_commit_SET((uint32_t)2231356481L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
    p320_target_system_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
    p320_target_component_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
    {
        char16_t   param_id = "lsDscrskuzuh";
        p320_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p320_param_index_SET((int16_t)(int16_t) -7890, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
    p321_target_system_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
    p321_target_component_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_VALUE_322(), &PH);
    {
        char16_t   param_id = "perirUn";
        p322_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "jNqywnbtnuAtmgfwrvyzpnCnzzjkk";
        p322_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT16, PH.base.pack) ;
    p322_param_count_SET((uint16_t)(uint16_t)32752, PH.base.pack) ;
    p322_param_index_SET((uint16_t)(uint16_t)46402, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_SET_323(), &PH);
    p323_target_system_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
    p323_target_component_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
    {
        char16_t   param_id = "mtGmeniucdmD";
        p323_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "ellmnpkfulxwopcBdmmodxvSIy";
        p323_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_ACK_324(), &PH);
    {
        char16_t   param_id = "sfmwWops";
        p324_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "mncjrittrvwkzsivpplUvbloyPmMzlsrzghrdrgqaxHBjdg";
        p324_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32, PH.base.pack) ;
    p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_ACCEPTED, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OBSTACLE_DISTANCE_330(), &PH);
    p330_time_usec_SET((uint64_t)707861525152538896L, PH.base.pack) ;
    p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, PH.base.pack) ;
    {
        uint16_t  distances [] =  {(uint16_t)18955, (uint16_t)1499, (uint16_t)28070, (uint16_t)1765, (uint16_t)49393, (uint16_t)28612, (uint16_t)48533, (uint16_t)58444, (uint16_t)27560, (uint16_t)16588, (uint16_t)26926, (uint16_t)11092, (uint16_t)20347, (uint16_t)24829, (uint16_t)31163, (uint16_t)24787, (uint16_t)13484, (uint16_t)9269, (uint16_t)10841, (uint16_t)64950, (uint16_t)54696, (uint16_t)22273, (uint16_t)57475, (uint16_t)6129, (uint16_t)26468, (uint16_t)11143, (uint16_t)50820, (uint16_t)28281, (uint16_t)31333, (uint16_t)18023, (uint16_t)61468, (uint16_t)43478, (uint16_t)25440, (uint16_t)47151, (uint16_t)54799, (uint16_t)64790, (uint16_t)4166, (uint16_t)55818, (uint16_t)28710, (uint16_t)21773, (uint16_t)57219, (uint16_t)881, (uint16_t)15159, (uint16_t)45143, (uint16_t)3088, (uint16_t)50095, (uint16_t)24406, (uint16_t)32882, (uint16_t)33237, (uint16_t)254, (uint16_t)33499, (uint16_t)27943, (uint16_t)43355, (uint16_t)4240, (uint16_t)30531, (uint16_t)22918, (uint16_t)48068, (uint16_t)15047, (uint16_t)32464, (uint16_t)24353, (uint16_t)52122, (uint16_t)51915, (uint16_t)58921, (uint16_t)29332, (uint16_t)20858, (uint16_t)5523, (uint16_t)18965, (uint16_t)39900, (uint16_t)32574, (uint16_t)17612, (uint16_t)12189, (uint16_t)4975};
        p330_distances_SET(&distances, 0, &PH.base.pack) ;
    }
    p330_increment_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
    p330_min_distance_SET((uint16_t)(uint16_t)47737, PH.base.pack) ;
    p330_max_distance_SET((uint16_t)(uint16_t)63495, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}
