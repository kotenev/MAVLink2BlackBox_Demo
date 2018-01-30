
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
    p0_type_SET(e_MAV_TYPE_MAV_TYPE_FIXED_WING, PH.base.pack) ;
    p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_SLUGS, PH.base.pack) ;
    p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PH.base.pack) ;
    p0_custom_mode_SET((uint32_t)2076363837L, PH.base.pack) ;
    p0_system_status_SET(e_MAV_STATE_MAV_STATE_CRITICAL, PH.base.pack) ;
    p0_mavlink_version_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SYS_STATUS_1(), &PH);
    p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL, PH.base.pack) ;
    p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER, PH.base.pack) ;
    p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO, PH.base.pack) ;
    p1_load_SET((uint16_t)(uint16_t)10368, PH.base.pack) ;
    p1_voltage_battery_SET((uint16_t)(uint16_t)27951, PH.base.pack) ;
    p1_current_battery_SET((int16_t)(int16_t)1729, PH.base.pack) ;
    p1_battery_remaining_SET((int8_t)(int8_t)105, PH.base.pack) ;
    p1_drop_rate_comm_SET((uint16_t)(uint16_t)46322, PH.base.pack) ;
    p1_errors_comm_SET((uint16_t)(uint16_t)53057, PH.base.pack) ;
    p1_errors_count1_SET((uint16_t)(uint16_t)52200, PH.base.pack) ;
    p1_errors_count2_SET((uint16_t)(uint16_t)41890, PH.base.pack) ;
    p1_errors_count3_SET((uint16_t)(uint16_t)61087, PH.base.pack) ;
    p1_errors_count4_SET((uint16_t)(uint16_t)9478, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SYSTEM_TIME_2(), &PH);
    p2_time_unix_usec_SET((uint64_t)7640884377301030992L, PH.base.pack) ;
    p2_time_boot_ms_SET((uint32_t)1593238076L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
    p3_time_boot_ms_SET((uint32_t)632949951L, PH.base.pack) ;
    p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
    p3_type_mask_SET((uint16_t)(uint16_t)30280, PH.base.pack) ;
    p3_x_SET((float)1.8628174E38F, PH.base.pack) ;
    p3_y_SET((float)3.3437138E38F, PH.base.pack) ;
    p3_z_SET((float) -2.8259143E38F, PH.base.pack) ;
    p3_vx_SET((float) -1.2789352E37F, PH.base.pack) ;
    p3_vy_SET((float)3.3484378E38F, PH.base.pack) ;
    p3_vz_SET((float) -3.353768E38F, PH.base.pack) ;
    p3_afx_SET((float)7.764897E37F, PH.base.pack) ;
    p3_afy_SET((float) -3.2079612E38F, PH.base.pack) ;
    p3_afz_SET((float)1.900854E38F, PH.base.pack) ;
    p3_yaw_SET((float)3.0879456E37F, PH.base.pack) ;
    p3_yaw_rate_SET((float)2.415365E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PING_4(), &PH);
    p4_time_usec_SET((uint64_t)7886306146663641629L, PH.base.pack) ;
    p4_seq_SET((uint32_t)460368484L, PH.base.pack) ;
    p4_target_system_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
    p4_target_component_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
    p5_target_system_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
    p5_control_request_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
    p5_version_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
    {
        char16_t   passkey = "egodhkswgExuUoynfblo";
        p5_passkey_SET(&passkey, 0,  sizeof(passkey), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
    p6_gcs_system_id_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
    p6_control_request_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
    p6_ack_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AUTH_KEY_7(), &PH);
    {
        char16_t   key = "fnhuqtfkedfevd";
        p7_key_SET(&key, 0,  sizeof(key), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_MODE_11(), &PH);
    p11_target_system_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
    p11_base_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_ARMED, PH.base.pack) ;
    p11_custom_mode_SET((uint32_t)3255053756L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_READ_20(), &PH);
    p20_target_system_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
    p20_target_component_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
    {
        char16_t   param_id = "rbFpmhhms";
        p20_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p20_param_index_SET((int16_t)(int16_t)21641, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_LIST_21(), &PH);
    p21_target_system_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
    p21_target_component_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_VALUE_22(), &PH);
    {
        char16_t   param_id = "yDqwweovj";
        p22_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p22_param_value_SET((float) -6.5838633E37F, PH.base.pack) ;
    p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64, PH.base.pack) ;
    p22_param_count_SET((uint16_t)(uint16_t)16935, PH.base.pack) ;
    p22_param_index_SET((uint16_t)(uint16_t)5066, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_SET_23(), &PH);
    p23_target_system_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
    p23_target_component_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
    {
        char16_t   param_id = "yorsvddjkoyV";
        p23_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p23_param_value_SET((float) -1.243719E38F, PH.base.pack) ;
    p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RAW_INT_24(), &PH);
    p24_time_usec_SET((uint64_t)2771358627930544884L, PH.base.pack) ;
    p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS, PH.base.pack) ;
    p24_lat_SET((int32_t) -1507985541, PH.base.pack) ;
    p24_lon_SET((int32_t)1773135659, PH.base.pack) ;
    p24_alt_SET((int32_t)399015946, PH.base.pack) ;
    p24_eph_SET((uint16_t)(uint16_t)23343, PH.base.pack) ;
    p24_epv_SET((uint16_t)(uint16_t)21481, PH.base.pack) ;
    p24_vel_SET((uint16_t)(uint16_t)15431, PH.base.pack) ;
    p24_cog_SET((uint16_t)(uint16_t)33783, PH.base.pack) ;
    p24_satellites_visible_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
    p24_alt_ellipsoid_SET((int32_t)1529462782, &PH) ;
    p24_h_acc_SET((uint32_t)829287255L, &PH) ;
    p24_v_acc_SET((uint32_t)2150391707L, &PH) ;
    p24_vel_acc_SET((uint32_t)264949658L, &PH) ;
    p24_hdg_acc_SET((uint32_t)1192100490L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_STATUS_25(), &PH);
    p25_satellites_visible_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
    {
        uint8_t  satellite_prn [] =  {(uint8_t)73, (uint8_t)87, (uint8_t)74, (uint8_t)183, (uint8_t)234, (uint8_t)12, (uint8_t)23, (uint8_t)117, (uint8_t)216, (uint8_t)67, (uint8_t)180, (uint8_t)120, (uint8_t)40, (uint8_t)166, (uint8_t)26, (uint8_t)192, (uint8_t)200, (uint8_t)166, (uint8_t)101, (uint8_t)74};
        p25_satellite_prn_SET(&satellite_prn, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_used [] =  {(uint8_t)65, (uint8_t)214, (uint8_t)104, (uint8_t)39, (uint8_t)144, (uint8_t)242, (uint8_t)255, (uint8_t)205, (uint8_t)213, (uint8_t)1, (uint8_t)44, (uint8_t)42, (uint8_t)177, (uint8_t)30, (uint8_t)106, (uint8_t)54, (uint8_t)45, (uint8_t)247, (uint8_t)155, (uint8_t)176};
        p25_satellite_used_SET(&satellite_used, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_elevation [] =  {(uint8_t)82, (uint8_t)3, (uint8_t)250, (uint8_t)1, (uint8_t)109, (uint8_t)245, (uint8_t)142, (uint8_t)123, (uint8_t)8, (uint8_t)96, (uint8_t)125, (uint8_t)4, (uint8_t)4, (uint8_t)144, (uint8_t)152, (uint8_t)102, (uint8_t)33, (uint8_t)179, (uint8_t)70, (uint8_t)198};
        p25_satellite_elevation_SET(&satellite_elevation, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_azimuth [] =  {(uint8_t)173, (uint8_t)127, (uint8_t)244, (uint8_t)138, (uint8_t)209, (uint8_t)38, (uint8_t)111, (uint8_t)157, (uint8_t)160, (uint8_t)91, (uint8_t)168, (uint8_t)242, (uint8_t)85, (uint8_t)35, (uint8_t)61, (uint8_t)24, (uint8_t)163, (uint8_t)110, (uint8_t)182, (uint8_t)79};
        p25_satellite_azimuth_SET(&satellite_azimuth, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_snr [] =  {(uint8_t)42, (uint8_t)101, (uint8_t)37, (uint8_t)230, (uint8_t)10, (uint8_t)181, (uint8_t)210, (uint8_t)249, (uint8_t)128, (uint8_t)221, (uint8_t)87, (uint8_t)161, (uint8_t)153, (uint8_t)6, (uint8_t)44, (uint8_t)97, (uint8_t)67, (uint8_t)111, (uint8_t)37, (uint8_t)80};
        p25_satellite_snr_SET(&satellite_snr, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU_26(), &PH);
    p26_time_boot_ms_SET((uint32_t)2040539040L, PH.base.pack) ;
    p26_xacc_SET((int16_t)(int16_t) -15206, PH.base.pack) ;
    p26_yacc_SET((int16_t)(int16_t)15211, PH.base.pack) ;
    p26_zacc_SET((int16_t)(int16_t) -14554, PH.base.pack) ;
    p26_xgyro_SET((int16_t)(int16_t)13686, PH.base.pack) ;
    p26_ygyro_SET((int16_t)(int16_t)16922, PH.base.pack) ;
    p26_zgyro_SET((int16_t)(int16_t)9402, PH.base.pack) ;
    p26_xmag_SET((int16_t)(int16_t) -8462, PH.base.pack) ;
    p26_ymag_SET((int16_t)(int16_t)9194, PH.base.pack) ;
    p26_zmag_SET((int16_t)(int16_t) -28854, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RAW_IMU_27(), &PH);
    p27_time_usec_SET((uint64_t)3845036908625915432L, PH.base.pack) ;
    p27_xacc_SET((int16_t)(int16_t) -23250, PH.base.pack) ;
    p27_yacc_SET((int16_t)(int16_t)3499, PH.base.pack) ;
    p27_zacc_SET((int16_t)(int16_t)28193, PH.base.pack) ;
    p27_xgyro_SET((int16_t)(int16_t) -5910, PH.base.pack) ;
    p27_ygyro_SET((int16_t)(int16_t)28790, PH.base.pack) ;
    p27_zgyro_SET((int16_t)(int16_t)25622, PH.base.pack) ;
    p27_xmag_SET((int16_t)(int16_t)2448, PH.base.pack) ;
    p27_ymag_SET((int16_t)(int16_t)3609, PH.base.pack) ;
    p27_zmag_SET((int16_t)(int16_t)18050, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RAW_PRESSURE_28(), &PH);
    p28_time_usec_SET((uint64_t)4856584786471321726L, PH.base.pack) ;
    p28_press_abs_SET((int16_t)(int16_t)31544, PH.base.pack) ;
    p28_press_diff1_SET((int16_t)(int16_t) -19788, PH.base.pack) ;
    p28_press_diff2_SET((int16_t)(int16_t)18953, PH.base.pack) ;
    p28_temperature_SET((int16_t)(int16_t) -9094, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE_29(), &PH);
    p29_time_boot_ms_SET((uint32_t)2310052736L, PH.base.pack) ;
    p29_press_abs_SET((float)3.226355E38F, PH.base.pack) ;
    p29_press_diff_SET((float)1.8360323E38F, PH.base.pack) ;
    p29_temperature_SET((int16_t)(int16_t)24014, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_30(), &PH);
    p30_time_boot_ms_SET((uint32_t)2924143737L, PH.base.pack) ;
    p30_roll_SET((float) -2.8214856E38F, PH.base.pack) ;
    p30_pitch_SET((float)1.2147038E38F, PH.base.pack) ;
    p30_yaw_SET((float)1.1478807E38F, PH.base.pack) ;
    p30_rollspeed_SET((float) -1.5547941E38F, PH.base.pack) ;
    p30_pitchspeed_SET((float)1.5887226E38F, PH.base.pack) ;
    p30_yawspeed_SET((float) -8.7064E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_31(), &PH);
    p31_time_boot_ms_SET((uint32_t)1969842337L, PH.base.pack) ;
    p31_q1_SET((float) -2.9947394E38F, PH.base.pack) ;
    p31_q2_SET((float)1.2430685E38F, PH.base.pack) ;
    p31_q3_SET((float) -1.1639694E38F, PH.base.pack) ;
    p31_q4_SET((float)2.460121E38F, PH.base.pack) ;
    p31_rollspeed_SET((float) -1.9269076E38F, PH.base.pack) ;
    p31_pitchspeed_SET((float) -1.0714847E37F, PH.base.pack) ;
    p31_yawspeed_SET((float) -2.2688575E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_32(), &PH);
    p32_time_boot_ms_SET((uint32_t)1049080857L, PH.base.pack) ;
    p32_x_SET((float)1.8443564E38F, PH.base.pack) ;
    p32_y_SET((float)3.373636E38F, PH.base.pack) ;
    p32_z_SET((float) -6.011131E37F, PH.base.pack) ;
    p32_vx_SET((float) -3.1433833E38F, PH.base.pack) ;
    p32_vy_SET((float)1.8292535E38F, PH.base.pack) ;
    p32_vz_SET((float) -2.2575656E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_33(), &PH);
    p33_time_boot_ms_SET((uint32_t)1128390943L, PH.base.pack) ;
    p33_lat_SET((int32_t) -1795846143, PH.base.pack) ;
    p33_lon_SET((int32_t)2125486360, PH.base.pack) ;
    p33_alt_SET((int32_t)715057713, PH.base.pack) ;
    p33_relative_alt_SET((int32_t) -386806999, PH.base.pack) ;
    p33_vx_SET((int16_t)(int16_t)22285, PH.base.pack) ;
    p33_vy_SET((int16_t)(int16_t) -18659, PH.base.pack) ;
    p33_vz_SET((int16_t)(int16_t)3251, PH.base.pack) ;
    p33_hdg_SET((uint16_t)(uint16_t)34947, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_SCALED_34(), &PH);
    p34_time_boot_ms_SET((uint32_t)1604819289L, PH.base.pack) ;
    p34_port_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
    p34_chan1_scaled_SET((int16_t)(int16_t) -17012, PH.base.pack) ;
    p34_chan2_scaled_SET((int16_t)(int16_t) -7802, PH.base.pack) ;
    p34_chan3_scaled_SET((int16_t)(int16_t) -31304, PH.base.pack) ;
    p34_chan4_scaled_SET((int16_t)(int16_t) -30978, PH.base.pack) ;
    p34_chan5_scaled_SET((int16_t)(int16_t) -31596, PH.base.pack) ;
    p34_chan6_scaled_SET((int16_t)(int16_t) -14611, PH.base.pack) ;
    p34_chan7_scaled_SET((int16_t)(int16_t) -16178, PH.base.pack) ;
    p34_chan8_scaled_SET((int16_t)(int16_t)10398, PH.base.pack) ;
    p34_rssi_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_RAW_35(), &PH);
    p35_time_boot_ms_SET((uint32_t)2418290357L, PH.base.pack) ;
    p35_port_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
    p35_chan1_raw_SET((uint16_t)(uint16_t)64231, PH.base.pack) ;
    p35_chan2_raw_SET((uint16_t)(uint16_t)35828, PH.base.pack) ;
    p35_chan3_raw_SET((uint16_t)(uint16_t)29149, PH.base.pack) ;
    p35_chan4_raw_SET((uint16_t)(uint16_t)36924, PH.base.pack) ;
    p35_chan5_raw_SET((uint16_t)(uint16_t)39764, PH.base.pack) ;
    p35_chan6_raw_SET((uint16_t)(uint16_t)24508, PH.base.pack) ;
    p35_chan7_raw_SET((uint16_t)(uint16_t)24972, PH.base.pack) ;
    p35_chan8_raw_SET((uint16_t)(uint16_t)33256, PH.base.pack) ;
    p35_rssi_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
    p36_time_usec_SET((uint32_t)2979375478L, PH.base.pack) ;
    p36_port_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
    p36_servo1_raw_SET((uint16_t)(uint16_t)46591, PH.base.pack) ;
    p36_servo2_raw_SET((uint16_t)(uint16_t)21011, PH.base.pack) ;
    p36_servo3_raw_SET((uint16_t)(uint16_t)60479, PH.base.pack) ;
    p36_servo4_raw_SET((uint16_t)(uint16_t)35042, PH.base.pack) ;
    p36_servo5_raw_SET((uint16_t)(uint16_t)53846, PH.base.pack) ;
    p36_servo6_raw_SET((uint16_t)(uint16_t)54619, PH.base.pack) ;
    p36_servo7_raw_SET((uint16_t)(uint16_t)61419, PH.base.pack) ;
    p36_servo8_raw_SET((uint16_t)(uint16_t)37474, PH.base.pack) ;
    p36_servo9_raw_SET((uint16_t)(uint16_t)22087, &PH) ;
    p36_servo10_raw_SET((uint16_t)(uint16_t)52567, &PH) ;
    p36_servo11_raw_SET((uint16_t)(uint16_t)60736, &PH) ;
    p36_servo12_raw_SET((uint16_t)(uint16_t)61562, &PH) ;
    p36_servo13_raw_SET((uint16_t)(uint16_t)7949, &PH) ;
    p36_servo14_raw_SET((uint16_t)(uint16_t)18722, &PH) ;
    p36_servo15_raw_SET((uint16_t)(uint16_t)19381, &PH) ;
    p36_servo16_raw_SET((uint16_t)(uint16_t)15698, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
    p37_target_system_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
    p37_target_component_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
    p37_start_index_SET((int16_t)(int16_t) -31435, PH.base.pack) ;
    p37_end_index_SET((int16_t)(int16_t) -25473, PH.base.pack) ;
    p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
    p38_target_system_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
    p38_target_component_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
    p38_start_index_SET((int16_t)(int16_t) -9692, PH.base.pack) ;
    p38_end_index_SET((int16_t)(int16_t)13984, PH.base.pack) ;
    p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_39(), &PH);
    p39_target_system_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
    p39_target_component_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
    p39_seq_SET((uint16_t)(uint16_t)37884, PH.base.pack) ;
    p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
    p39_command_SET(e_MAV_CMD_MAV_CMD_USER_1, PH.base.pack) ;
    p39_current_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
    p39_autocontinue_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
    p39_param1_SET((float) -9.903107E37F, PH.base.pack) ;
    p39_param2_SET((float) -1.4954728E37F, PH.base.pack) ;
    p39_param3_SET((float) -1.9294034E38F, PH.base.pack) ;
    p39_param4_SET((float)2.019859E38F, PH.base.pack) ;
    p39_x_SET((float)2.9745234E37F, PH.base.pack) ;
    p39_y_SET((float) -8.692874E37F, PH.base.pack) ;
    p39_z_SET((float) -8.657573E37F, PH.base.pack) ;
    p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_40(), &PH);
    p40_target_system_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
    p40_target_component_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
    p40_seq_SET((uint16_t)(uint16_t)16765, PH.base.pack) ;
    p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_SET_CURRENT_41(), &PH);
    p41_target_system_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
    p41_target_component_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
    p41_seq_SET((uint16_t)(uint16_t)24751, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_CURRENT_42(), &PH);
    p42_seq_SET((uint16_t)(uint16_t)42404, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_LIST_43(), &PH);
    p43_target_system_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
    p43_target_component_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
    p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_COUNT_44(), &PH);
    p44_target_system_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
    p44_target_component_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
    p44_count_SET((uint16_t)(uint16_t)16556, PH.base.pack) ;
    p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_CLEAR_ALL_45(), &PH);
    p45_target_system_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
    p45_target_component_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
    p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_REACHED_46(), &PH);
    p46_seq_SET((uint16_t)(uint16_t)21460, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ACK_47(), &PH);
    p47_target_system_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
    p47_target_component_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
    p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM4, PH.base.pack) ;
    p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
    p48_target_system_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
    p48_latitude_SET((int32_t) -347430284, PH.base.pack) ;
    p48_longitude_SET((int32_t)1127037479, PH.base.pack) ;
    p48_altitude_SET((int32_t) -1686024620, PH.base.pack) ;
    p48_time_usec_SET((uint64_t)8956068826333053854L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
    p49_latitude_SET((int32_t)1110222716, PH.base.pack) ;
    p49_longitude_SET((int32_t) -632190735, PH.base.pack) ;
    p49_altitude_SET((int32_t)1305744202, PH.base.pack) ;
    p49_time_usec_SET((uint64_t)1227730310867952797L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_MAP_RC_50(), &PH);
    p50_target_system_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
    p50_target_component_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
    {
        char16_t   param_id = "hn";
        p50_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p50_param_index_SET((int16_t)(int16_t) -14363, PH.base.pack) ;
    p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
    p50_param_value0_SET((float)1.5892339E38F, PH.base.pack) ;
    p50_scale_SET((float)2.8857265E38F, PH.base.pack) ;
    p50_param_value_min_SET((float) -3.1522205E38F, PH.base.pack) ;
    p50_param_value_max_SET((float) -3.312524E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_INT_51(), &PH);
    p51_target_system_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
    p51_target_component_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
    p51_seq_SET((uint16_t)(uint16_t)53885, PH.base.pack) ;
    p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
    p54_target_system_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
    p54_target_component_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
    p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
    p54_p1x_SET((float)8.422524E37F, PH.base.pack) ;
    p54_p1y_SET((float) -3.0977756E38F, PH.base.pack) ;
    p54_p1z_SET((float)3.2874372E38F, PH.base.pack) ;
    p54_p2x_SET((float) -1.7024527E38F, PH.base.pack) ;
    p54_p2y_SET((float)1.5271992E38F, PH.base.pack) ;
    p54_p2z_SET((float)2.8511237E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
    p55_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
    p55_p1x_SET((float)3.3533182E38F, PH.base.pack) ;
    p55_p1y_SET((float)1.0958457E38F, PH.base.pack) ;
    p55_p1z_SET((float)3.0799022E38F, PH.base.pack) ;
    p55_p2x_SET((float)2.2806152E38F, PH.base.pack) ;
    p55_p2y_SET((float)3.3293014E38F, PH.base.pack) ;
    p55_p2z_SET((float)2.0045353E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
    p61_time_usec_SET((uint64_t)5601032132651292905L, PH.base.pack) ;
    {
        float  q [] =  {1.9054663E38F, -2.4243907E38F, -1.0236245E38F, 3.3669524E38F};
        p61_q_SET(&q, 0, &PH.base.pack) ;
    }
    p61_rollspeed_SET((float) -1.56912E37F, PH.base.pack) ;
    p61_pitchspeed_SET((float) -3.1780577E38F, PH.base.pack) ;
    p61_yawspeed_SET((float)1.0742077E38F, PH.base.pack) ;
    {
        float  covariance [] =  {1.1043494E38F, 2.2167355E37F, 2.309583E38F, 5.7120197E36F, -3.0534502E38F, 2.441516E38F, 5.26381E37F, -5.032862E37F, 4.812824E37F};
        p61_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
    p62_nav_roll_SET((float)1.5649944E38F, PH.base.pack) ;
    p62_nav_pitch_SET((float) -2.2190834E38F, PH.base.pack) ;
    p62_nav_bearing_SET((int16_t)(int16_t) -18789, PH.base.pack) ;
    p62_target_bearing_SET((int16_t)(int16_t) -32498, PH.base.pack) ;
    p62_wp_dist_SET((uint16_t)(uint16_t)35284, PH.base.pack) ;
    p62_alt_error_SET((float) -3.2435481E38F, PH.base.pack) ;
    p62_aspd_error_SET((float)2.913223E38F, PH.base.pack) ;
    p62_xtrack_error_SET((float)1.4060982E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
    p63_time_usec_SET((uint64_t)5512869399875635489L, PH.base.pack) ;
    p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, PH.base.pack) ;
    p63_lat_SET((int32_t)1887106180, PH.base.pack) ;
    p63_lon_SET((int32_t)2066353130, PH.base.pack) ;
    p63_alt_SET((int32_t)576675410, PH.base.pack) ;
    p63_relative_alt_SET((int32_t)480544947, PH.base.pack) ;
    p63_vx_SET((float) -3.1341127E37F, PH.base.pack) ;
    p63_vy_SET((float)1.6976069E38F, PH.base.pack) ;
    p63_vz_SET((float)3.142136E38F, PH.base.pack) ;
    {
        float  covariance [] =  {3.1360747E38F, -1.2889093E38F, 7.4783556E37F, 1.9161421E38F, 8.454302E37F, 2.1479887E38F, -3.3181345E38F, 9.485974E37F, 3.3347965E38F, -1.878173E38F, 5.645007E37F, 1.9146617E38F, -1.3452861E38F, -1.4938843E38F, -1.5716391E38F, 3.3208042E38F, -2.827886E38F, 2.8166906E38F, -1.2570551E38F, 3.1326658E38F, -2.1663617E38F, -2.9640423E38F, -2.8777924E38F, -9.542535E37F, 1.8949352E38F, 1.2821082E37F, -3.167467E38F, -1.7184714E38F, 2.4455357E38F, 9.303021E37F, 2.436708E38F, -5.8337965E37F, -2.5100932E38F, -2.5256462E38F, -2.0278953E38F, -2.8170446E38F};
        p63_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
    p64_time_usec_SET((uint64_t)271868031217872473L, PH.base.pack) ;
    p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, PH.base.pack) ;
    p64_x_SET((float) -7.8068E37F, PH.base.pack) ;
    p64_y_SET((float) -2.601786E38F, PH.base.pack) ;
    p64_z_SET((float)1.3449994E38F, PH.base.pack) ;
    p64_vx_SET((float)9.431189E37F, PH.base.pack) ;
    p64_vy_SET((float)8.715671E37F, PH.base.pack) ;
    p64_vz_SET((float)1.2721633E38F, PH.base.pack) ;
    p64_ax_SET((float)1.991631E38F, PH.base.pack) ;
    p64_ay_SET((float)2.2919529E38F, PH.base.pack) ;
    p64_az_SET((float) -3.2099323E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-2.081239E38F, -2.0483122E37F, -1.3581703E38F, 2.9220194E38F, 8.056299E37F, 1.3512779E37F, -1.7441292E38F, -2.2953907E38F, -1.4965819E38F, -2.2111143E37F, 1.4875712E38F, -2.0823023E38F, 1.8946864E38F, -5.7547357E37F, -1.2564841E38F, -2.137952E38F, 5.5513417E37F, 2.6896234E38F, 2.3258916E38F, -2.1422794E38F, -3.0401198E38F, -8.063976E37F, -1.118386E38F, -3.5408458E37F, -3.3138399E38F, 3.0136113E38F, 2.0570203E38F, 1.8182729E37F, 1.6772499E38F, -2.9183732E38F, -1.9315346E38F, 3.0790219E38F, -2.550142E38F, 7.5160556E37F, 1.2446346E38F, -3.1681596E38F, 7.151176E37F, 2.926394E37F, 9.73982E37F, -2.6423907E38F, 7.9514763E37F, -2.9094539E38F, 9.081766E37F, 9.365193E37F, -1.0229759E38F};
        p64_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_65(), &PH);
    p65_time_boot_ms_SET((uint32_t)2795935166L, PH.base.pack) ;
    p65_chancount_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
    p65_chan1_raw_SET((uint16_t)(uint16_t)22868, PH.base.pack) ;
    p65_chan2_raw_SET((uint16_t)(uint16_t)52787, PH.base.pack) ;
    p65_chan3_raw_SET((uint16_t)(uint16_t)65235, PH.base.pack) ;
    p65_chan4_raw_SET((uint16_t)(uint16_t)37475, PH.base.pack) ;
    p65_chan5_raw_SET((uint16_t)(uint16_t)44711, PH.base.pack) ;
    p65_chan6_raw_SET((uint16_t)(uint16_t)2512, PH.base.pack) ;
    p65_chan7_raw_SET((uint16_t)(uint16_t)8813, PH.base.pack) ;
    p65_chan8_raw_SET((uint16_t)(uint16_t)38994, PH.base.pack) ;
    p65_chan9_raw_SET((uint16_t)(uint16_t)51522, PH.base.pack) ;
    p65_chan10_raw_SET((uint16_t)(uint16_t)231, PH.base.pack) ;
    p65_chan11_raw_SET((uint16_t)(uint16_t)49700, PH.base.pack) ;
    p65_chan12_raw_SET((uint16_t)(uint16_t)1558, PH.base.pack) ;
    p65_chan13_raw_SET((uint16_t)(uint16_t)59074, PH.base.pack) ;
    p65_chan14_raw_SET((uint16_t)(uint16_t)64219, PH.base.pack) ;
    p65_chan15_raw_SET((uint16_t)(uint16_t)62454, PH.base.pack) ;
    p65_chan16_raw_SET((uint16_t)(uint16_t)11785, PH.base.pack) ;
    p65_chan17_raw_SET((uint16_t)(uint16_t)8729, PH.base.pack) ;
    p65_chan18_raw_SET((uint16_t)(uint16_t)14823, PH.base.pack) ;
    p65_rssi_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_REQUEST_DATA_STREAM_66(), &PH);
    p66_target_system_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
    p66_target_component_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
    p66_req_stream_id_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
    p66_req_message_rate_SET((uint16_t)(uint16_t)62434, PH.base.pack) ;
    p66_start_stop_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA_STREAM_67(), &PH);
    p67_stream_id_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
    p67_message_rate_SET((uint16_t)(uint16_t)5912, PH.base.pack) ;
    p67_on_off_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MANUAL_CONTROL_69(), &PH);
    p69_target_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
    p69_x_SET((int16_t)(int16_t) -17295, PH.base.pack) ;
    p69_y_SET((int16_t)(int16_t)10650, PH.base.pack) ;
    p69_z_SET((int16_t)(int16_t) -26026, PH.base.pack) ;
    p69_r_SET((int16_t)(int16_t)12948, PH.base.pack) ;
    p69_buttons_SET((uint16_t)(uint16_t)22903, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
    p70_target_system_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
    p70_target_component_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
    p70_chan1_raw_SET((uint16_t)(uint16_t)12773, PH.base.pack) ;
    p70_chan2_raw_SET((uint16_t)(uint16_t)43924, PH.base.pack) ;
    p70_chan3_raw_SET((uint16_t)(uint16_t)2920, PH.base.pack) ;
    p70_chan4_raw_SET((uint16_t)(uint16_t)17788, PH.base.pack) ;
    p70_chan5_raw_SET((uint16_t)(uint16_t)10283, PH.base.pack) ;
    p70_chan6_raw_SET((uint16_t)(uint16_t)44011, PH.base.pack) ;
    p70_chan7_raw_SET((uint16_t)(uint16_t)59062, PH.base.pack) ;
    p70_chan8_raw_SET((uint16_t)(uint16_t)7304, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_INT_73(), &PH);
    p73_target_system_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
    p73_target_component_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
    p73_seq_SET((uint16_t)(uint16_t)63701, PH.base.pack) ;
    p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
    p73_command_SET(e_MAV_CMD_MAV_CMD_NAV_PATHPLANNING, PH.base.pack) ;
    p73_current_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
    p73_autocontinue_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
    p73_param1_SET((float)6.6036377E37F, PH.base.pack) ;
    p73_param2_SET((float)1.5919663E38F, PH.base.pack) ;
    p73_param3_SET((float) -1.0714061E38F, PH.base.pack) ;
    p73_param4_SET((float) -1.3029392E38F, PH.base.pack) ;
    p73_x_SET((int32_t)74363670, PH.base.pack) ;
    p73_y_SET((int32_t)2044700385, PH.base.pack) ;
    p73_z_SET((float) -1.6613777E38F, PH.base.pack) ;
    p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VFR_HUD_74(), &PH);
    p74_airspeed_SET((float)3.9671612E37F, PH.base.pack) ;
    p74_groundspeed_SET((float)1.6009282E38F, PH.base.pack) ;
    p74_heading_SET((int16_t)(int16_t)30962, PH.base.pack) ;
    p74_throttle_SET((uint16_t)(uint16_t)57989, PH.base.pack) ;
    p74_alt_SET((float) -2.4436975E38F, PH.base.pack) ;
    p74_climb_SET((float) -3.2940146E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_INT_75(), &PH);
    p75_target_system_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
    p75_target_component_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
    p75_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
    p75_command_SET(e_MAV_CMD_MAV_CMD_DO_JUMP, PH.base.pack) ;
    p75_current_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
    p75_autocontinue_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
    p75_param1_SET((float) -6.887754E37F, PH.base.pack) ;
    p75_param2_SET((float) -1.5513682E38F, PH.base.pack) ;
    p75_param3_SET((float)2.0128805E38F, PH.base.pack) ;
    p75_param4_SET((float) -1.4738301E38F, PH.base.pack) ;
    p75_x_SET((int32_t)1299856439, PH.base.pack) ;
    p75_y_SET((int32_t) -349659215, PH.base.pack) ;
    p75_z_SET((float)1.5468072E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_LONG_76(), &PH);
    p76_target_system_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
    p76_target_component_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
    p76_command_SET(e_MAV_CMD_MAV_CMD_DO_FOLLOW, PH.base.pack) ;
    p76_confirmation_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
    p76_param1_SET((float)1.466042E38F, PH.base.pack) ;
    p76_param2_SET((float) -2.454243E38F, PH.base.pack) ;
    p76_param3_SET((float) -2.1534222E38F, PH.base.pack) ;
    p76_param4_SET((float)2.6255567E38F, PH.base.pack) ;
    p76_param5_SET((float)2.5303442E38F, PH.base.pack) ;
    p76_param6_SET((float) -2.332018E38F, PH.base.pack) ;
    p76_param7_SET((float)2.318116E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COMMAND_ACK_77(), &PH);
    p77_command_SET(e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS, PH.base.pack) ;
    p77_result_SET(e_MAV_RESULT_MAV_RESULT_DENIED, PH.base.pack) ;
    p77_progress_SET((uint8_t)(uint8_t)145, &PH) ;
    p77_result_param2_SET((int32_t)1894508724, &PH) ;
    p77_target_system_SET((uint8_t)(uint8_t)157, &PH) ;
    p77_target_component_SET((uint8_t)(uint8_t)233, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MANUAL_SETPOINT_81(), &PH);
    p81_time_boot_ms_SET((uint32_t)970491360L, PH.base.pack) ;
    p81_roll_SET((float) -2.6258007E38F, PH.base.pack) ;
    p81_pitch_SET((float) -6.5598546E37F, PH.base.pack) ;
    p81_yaw_SET((float)2.469466E38F, PH.base.pack) ;
    p81_thrust_SET((float)1.538181E38F, PH.base.pack) ;
    p81_mode_switch_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
    p81_manual_override_switch_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
    p82_time_boot_ms_SET((uint32_t)1848417279L, PH.base.pack) ;
    p82_target_system_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
    p82_target_component_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
    p82_type_mask_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
    {
        float  q [] =  {8.255527E37F, 1.5583975E38F, -2.8445836E38F, 8.11647E37F};
        p82_q_SET(&q, 0, &PH.base.pack) ;
    }
    p82_body_roll_rate_SET((float)2.0968243E38F, PH.base.pack) ;
    p82_body_pitch_rate_SET((float)9.023092E37F, PH.base.pack) ;
    p82_body_yaw_rate_SET((float)2.560165E38F, PH.base.pack) ;
    p82_thrust_SET((float)2.5173002E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATTITUDE_TARGET_83(), &PH);
    p83_time_boot_ms_SET((uint32_t)1359670054L, PH.base.pack) ;
    p83_type_mask_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
    {
        float  q [] =  {-4.034605E37F, 1.6457261E38F, -1.2264144E38F, 2.7818199E38F};
        p83_q_SET(&q, 0, &PH.base.pack) ;
    }
    p83_body_roll_rate_SET((float)1.9764208E38F, PH.base.pack) ;
    p83_body_pitch_rate_SET((float) -2.6272077E38F, PH.base.pack) ;
    p83_body_yaw_rate_SET((float) -1.4467036E37F, PH.base.pack) ;
    p83_thrust_SET((float)5.5561065E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
    p84_time_boot_ms_SET((uint32_t)4156844145L, PH.base.pack) ;
    p84_target_system_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
    p84_target_component_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
    p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
    p84_type_mask_SET((uint16_t)(uint16_t)60018, PH.base.pack) ;
    p84_x_SET((float)2.582585E38F, PH.base.pack) ;
    p84_y_SET((float)1.4710871E38F, PH.base.pack) ;
    p84_z_SET((float)1.2760721E38F, PH.base.pack) ;
    p84_vx_SET((float) -1.5377027E38F, PH.base.pack) ;
    p84_vy_SET((float) -3.2727057E37F, PH.base.pack) ;
    p84_vz_SET((float)2.4881136E38F, PH.base.pack) ;
    p84_afx_SET((float)1.80667E38F, PH.base.pack) ;
    p84_afy_SET((float)2.6121314E38F, PH.base.pack) ;
    p84_afz_SET((float) -3.071976E38F, PH.base.pack) ;
    p84_yaw_SET((float) -1.6136875E38F, PH.base.pack) ;
    p84_yaw_rate_SET((float)5.277498E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
    p86_time_boot_ms_SET((uint32_t)720740831L, PH.base.pack) ;
    p86_target_system_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
    p86_target_component_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
    p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
    p86_type_mask_SET((uint16_t)(uint16_t)43075, PH.base.pack) ;
    p86_lat_int_SET((int32_t) -1703311701, PH.base.pack) ;
    p86_lon_int_SET((int32_t) -2039708312, PH.base.pack) ;
    p86_alt_SET((float) -1.3292824E38F, PH.base.pack) ;
    p86_vx_SET((float) -2.4680086E38F, PH.base.pack) ;
    p86_vy_SET((float)8.119289E37F, PH.base.pack) ;
    p86_vz_SET((float)6.238474E37F, PH.base.pack) ;
    p86_afx_SET((float) -2.4820009E38F, PH.base.pack) ;
    p86_afy_SET((float) -2.0979936E38F, PH.base.pack) ;
    p86_afz_SET((float)7.474185E37F, PH.base.pack) ;
    p86_yaw_SET((float) -2.6493794E38F, PH.base.pack) ;
    p86_yaw_rate_SET((float)1.113412E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
    p87_time_boot_ms_SET((uint32_t)2342391149L, PH.base.pack) ;
    p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
    p87_type_mask_SET((uint16_t)(uint16_t)4295, PH.base.pack) ;
    p87_lat_int_SET((int32_t)1936789477, PH.base.pack) ;
    p87_lon_int_SET((int32_t)545052119, PH.base.pack) ;
    p87_alt_SET((float)2.8035113E38F, PH.base.pack) ;
    p87_vx_SET((float) -1.0946104E38F, PH.base.pack) ;
    p87_vy_SET((float)4.4003387E37F, PH.base.pack) ;
    p87_vz_SET((float) -2.8543805E37F, PH.base.pack) ;
    p87_afx_SET((float)3.2497957E38F, PH.base.pack) ;
    p87_afy_SET((float)3.333904E38F, PH.base.pack) ;
    p87_afz_SET((float)1.8051004E38F, PH.base.pack) ;
    p87_yaw_SET((float) -1.2959069E38F, PH.base.pack) ;
    p87_yaw_rate_SET((float) -2.8724295E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
    p89_time_boot_ms_SET((uint32_t)1712496087L, PH.base.pack) ;
    p89_x_SET((float)2.8731506E38F, PH.base.pack) ;
    p89_y_SET((float)1.0968078E38F, PH.base.pack) ;
    p89_z_SET((float) -1.7448894E38F, PH.base.pack) ;
    p89_roll_SET((float) -3.076759E38F, PH.base.pack) ;
    p89_pitch_SET((float)2.3761175E38F, PH.base.pack) ;
    p89_yaw_SET((float) -1.1999036E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_STATE_90(), &PH);
    p90_time_usec_SET((uint64_t)2136343238144955045L, PH.base.pack) ;
    p90_roll_SET((float)2.7725295E38F, PH.base.pack) ;
    p90_pitch_SET((float) -5.908866E37F, PH.base.pack) ;
    p90_yaw_SET((float) -3.2390516E37F, PH.base.pack) ;
    p90_rollspeed_SET((float)3.4013877E37F, PH.base.pack) ;
    p90_pitchspeed_SET((float)2.7379176E38F, PH.base.pack) ;
    p90_yawspeed_SET((float) -1.9382554E38F, PH.base.pack) ;
    p90_lat_SET((int32_t) -1698935775, PH.base.pack) ;
    p90_lon_SET((int32_t) -1453649646, PH.base.pack) ;
    p90_alt_SET((int32_t) -71336429, PH.base.pack) ;
    p90_vx_SET((int16_t)(int16_t)17599, PH.base.pack) ;
    p90_vy_SET((int16_t)(int16_t) -12964, PH.base.pack) ;
    p90_vz_SET((int16_t)(int16_t)12374, PH.base.pack) ;
    p90_xacc_SET((int16_t)(int16_t) -27453, PH.base.pack) ;
    p90_yacc_SET((int16_t)(int16_t)19153, PH.base.pack) ;
    p90_zacc_SET((int16_t)(int16_t)20433, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_CONTROLS_91(), &PH);
    p91_time_usec_SET((uint64_t)441081852059556686L, PH.base.pack) ;
    p91_roll_ailerons_SET((float) -9.179999E37F, PH.base.pack) ;
    p91_pitch_elevator_SET((float)1.1755291E38F, PH.base.pack) ;
    p91_yaw_rudder_SET((float) -1.2763315E38F, PH.base.pack) ;
    p91_throttle_SET((float) -1.0566007E38F, PH.base.pack) ;
    p91_aux1_SET((float) -1.7770155E38F, PH.base.pack) ;
    p91_aux2_SET((float)2.3383022E37F, PH.base.pack) ;
    p91_aux3_SET((float) -6.6484557E37F, PH.base.pack) ;
    p91_aux4_SET((float)2.2866054E37F, PH.base.pack) ;
    p91_mode_SET(e_MAV_MODE_MAV_MODE_TEST_ARMED, PH.base.pack) ;
    p91_nav_mode_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
    p92_time_usec_SET((uint64_t)34525463518650274L, PH.base.pack) ;
    p92_chan1_raw_SET((uint16_t)(uint16_t)56003, PH.base.pack) ;
    p92_chan2_raw_SET((uint16_t)(uint16_t)19038, PH.base.pack) ;
    p92_chan3_raw_SET((uint16_t)(uint16_t)37761, PH.base.pack) ;
    p92_chan4_raw_SET((uint16_t)(uint16_t)37504, PH.base.pack) ;
    p92_chan5_raw_SET((uint16_t)(uint16_t)46054, PH.base.pack) ;
    p92_chan6_raw_SET((uint16_t)(uint16_t)52451, PH.base.pack) ;
    p92_chan7_raw_SET((uint16_t)(uint16_t)31439, PH.base.pack) ;
    p92_chan8_raw_SET((uint16_t)(uint16_t)27228, PH.base.pack) ;
    p92_chan9_raw_SET((uint16_t)(uint16_t)40024, PH.base.pack) ;
    p92_chan10_raw_SET((uint16_t)(uint16_t)4665, PH.base.pack) ;
    p92_chan11_raw_SET((uint16_t)(uint16_t)26819, PH.base.pack) ;
    p92_chan12_raw_SET((uint16_t)(uint16_t)41326, PH.base.pack) ;
    p92_rssi_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
    p93_time_usec_SET((uint64_t)1755082204673065417L, PH.base.pack) ;
    {
        float  controls [] =  {-2.3135506E38F, 2.913928E38F, 3.1575418E38F, 2.1774674E38F, -2.807979E38F, -2.8969723E38F, 2.6075672E38F, 9.049323E37F, -1.11864E38F, 7.4468905E37F, -3.1257252E38F, -3.3381751E38F, -3.1136945E38F, 1.7303686E38F, -2.787944E38F, -3.3663253E38F};
        p93_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    p93_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_DISARMED, PH.base.pack) ;
    p93_flags_SET((uint64_t)308878796452728522L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_100(), &PH);
    p100_time_usec_SET((uint64_t)4535808162787984361L, PH.base.pack) ;
    p100_sensor_id_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
    p100_flow_x_SET((int16_t)(int16_t)31632, PH.base.pack) ;
    p100_flow_y_SET((int16_t)(int16_t)25356, PH.base.pack) ;
    p100_flow_comp_m_x_SET((float)7.3372124E37F, PH.base.pack) ;
    p100_flow_comp_m_y_SET((float)2.580678E37F, PH.base.pack) ;
    p100_quality_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
    p100_ground_distance_SET((float) -7.7613176E37F, PH.base.pack) ;
    p100_flow_rate_x_SET((float)2.6698908E38F, &PH) ;
    p100_flow_rate_y_SET((float) -1.5769859E38F, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
    p101_usec_SET((uint64_t)2664043275124770062L, PH.base.pack) ;
    p101_x_SET((float) -1.1654721E38F, PH.base.pack) ;
    p101_y_SET((float)3.0316767E38F, PH.base.pack) ;
    p101_z_SET((float) -8.645872E36F, PH.base.pack) ;
    p101_roll_SET((float) -2.2855225E38F, PH.base.pack) ;
    p101_pitch_SET((float) -2.5438711E38F, PH.base.pack) ;
    p101_yaw_SET((float) -2.2626972E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
    p102_usec_SET((uint64_t)726158014712105214L, PH.base.pack) ;
    p102_x_SET((float) -3.3215543E38F, PH.base.pack) ;
    p102_y_SET((float) -2.5507694E38F, PH.base.pack) ;
    p102_z_SET((float) -2.5490427E38F, PH.base.pack) ;
    p102_roll_SET((float)2.76815E38F, PH.base.pack) ;
    p102_pitch_SET((float) -1.8379175E38F, PH.base.pack) ;
    p102_yaw_SET((float)1.1930237E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
    p103_usec_SET((uint64_t)8972468165054814030L, PH.base.pack) ;
    p103_x_SET((float)4.670373E36F, PH.base.pack) ;
    p103_y_SET((float) -1.324145E38F, PH.base.pack) ;
    p103_z_SET((float)2.4458813E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
    p104_usec_SET((uint64_t)4608035087221994981L, PH.base.pack) ;
    p104_x_SET((float) -1.0678378E38F, PH.base.pack) ;
    p104_y_SET((float) -2.9723786E38F, PH.base.pack) ;
    p104_z_SET((float) -2.9050854E38F, PH.base.pack) ;
    p104_roll_SET((float)4.8150694E37F, PH.base.pack) ;
    p104_pitch_SET((float)2.4551392E38F, PH.base.pack) ;
    p104_yaw_SET((float)2.3063006E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIGHRES_IMU_105(), &PH);
    p105_time_usec_SET((uint64_t)309777257055862476L, PH.base.pack) ;
    p105_xacc_SET((float) -2.751723E38F, PH.base.pack) ;
    p105_yacc_SET((float)3.1133813E38F, PH.base.pack) ;
    p105_zacc_SET((float)1.3193575E38F, PH.base.pack) ;
    p105_xgyro_SET((float) -7.7691887E37F, PH.base.pack) ;
    p105_ygyro_SET((float)2.1692964E38F, PH.base.pack) ;
    p105_zgyro_SET((float)1.1173287E36F, PH.base.pack) ;
    p105_xmag_SET((float) -2.3054583E38F, PH.base.pack) ;
    p105_ymag_SET((float) -3.1068118E38F, PH.base.pack) ;
    p105_zmag_SET((float)2.4273152E38F, PH.base.pack) ;
    p105_abs_pressure_SET((float)1.9677107E38F, PH.base.pack) ;
    p105_diff_pressure_SET((float)2.583067E38F, PH.base.pack) ;
    p105_pressure_alt_SET((float)6.7902064E37F, PH.base.pack) ;
    p105_temperature_SET((float)6.2374804E37F, PH.base.pack) ;
    p105_fields_updated_SET((uint16_t)(uint16_t)2203, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
    p106_time_usec_SET((uint64_t)2510971808504042378L, PH.base.pack) ;
    p106_sensor_id_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
    p106_integration_time_us_SET((uint32_t)2239592534L, PH.base.pack) ;
    p106_integrated_x_SET((float)1.0718792E38F, PH.base.pack) ;
    p106_integrated_y_SET((float)3.4418298E37F, PH.base.pack) ;
    p106_integrated_xgyro_SET((float) -1.6894108E38F, PH.base.pack) ;
    p106_integrated_ygyro_SET((float) -1.397188E38F, PH.base.pack) ;
    p106_integrated_zgyro_SET((float)1.0384619E38F, PH.base.pack) ;
    p106_temperature_SET((int16_t)(int16_t)6193, PH.base.pack) ;
    p106_quality_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
    p106_time_delta_distance_us_SET((uint32_t)1913763000L, PH.base.pack) ;
    p106_distance_SET((float)4.4136095E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_SENSOR_107(), &PH);
    p107_time_usec_SET((uint64_t)570878653220243509L, PH.base.pack) ;
    p107_xacc_SET((float) -7.196374E37F, PH.base.pack) ;
    p107_yacc_SET((float) -2.0086557E38F, PH.base.pack) ;
    p107_zacc_SET((float)3.1251202E38F, PH.base.pack) ;
    p107_xgyro_SET((float) -2.1238145E38F, PH.base.pack) ;
    p107_ygyro_SET((float)2.9910849E38F, PH.base.pack) ;
    p107_zgyro_SET((float)4.529732E37F, PH.base.pack) ;
    p107_xmag_SET((float) -1.1397214E38F, PH.base.pack) ;
    p107_ymag_SET((float) -2.4485823E38F, PH.base.pack) ;
    p107_zmag_SET((float) -3.9798758E37F, PH.base.pack) ;
    p107_abs_pressure_SET((float) -5.19969E37F, PH.base.pack) ;
    p107_diff_pressure_SET((float)3.3298877E38F, PH.base.pack) ;
    p107_pressure_alt_SET((float)1.2908117E38F, PH.base.pack) ;
    p107_temperature_SET((float) -5.2595394E36F, PH.base.pack) ;
    p107_fields_updated_SET((uint32_t)461618231L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SIM_STATE_108(), &PH);
    p108_q1_SET((float)1.5674351E38F, PH.base.pack) ;
    p108_q2_SET((float)7.059568E37F, PH.base.pack) ;
    p108_q3_SET((float)2.6121348E38F, PH.base.pack) ;
    p108_q4_SET((float)1.9640234E38F, PH.base.pack) ;
    p108_roll_SET((float) -1.9969643E37F, PH.base.pack) ;
    p108_pitch_SET((float)8.291188E37F, PH.base.pack) ;
    p108_yaw_SET((float) -2.4585116E38F, PH.base.pack) ;
    p108_xacc_SET((float) -3.053259E38F, PH.base.pack) ;
    p108_yacc_SET((float)3.3486844E38F, PH.base.pack) ;
    p108_zacc_SET((float)2.0355154E38F, PH.base.pack) ;
    p108_xgyro_SET((float)1.9637585E38F, PH.base.pack) ;
    p108_ygyro_SET((float) -2.8180751E38F, PH.base.pack) ;
    p108_zgyro_SET((float) -1.997473E38F, PH.base.pack) ;
    p108_lat_SET((float) -3.0873953E38F, PH.base.pack) ;
    p108_lon_SET((float)6.3744476E36F, PH.base.pack) ;
    p108_alt_SET((float)2.2488142E38F, PH.base.pack) ;
    p108_std_dev_horz_SET((float)1.9674947E38F, PH.base.pack) ;
    p108_std_dev_vert_SET((float) -2.8551806E38F, PH.base.pack) ;
    p108_vn_SET((float)3.150497E38F, PH.base.pack) ;
    p108_ve_SET((float) -1.0978711E38F, PH.base.pack) ;
    p108_vd_SET((float)2.0032204E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RADIO_STATUS_109(), &PH);
    p109_rssi_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
    p109_remrssi_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
    p109_txbuf_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
    p109_noise_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
    p109_remnoise_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
    p109_rxerrors_SET((uint16_t)(uint16_t)9022, PH.base.pack) ;
    p109_fixed__SET((uint16_t)(uint16_t)19516, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
    p110_target_network_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
    p110_target_system_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
    p110_target_component_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)163, (uint8_t)247, (uint8_t)192, (uint8_t)46, (uint8_t)197, (uint8_t)78, (uint8_t)163, (uint8_t)180, (uint8_t)100, (uint8_t)36, (uint8_t)178, (uint8_t)32, (uint8_t)204, (uint8_t)123, (uint8_t)213, (uint8_t)250, (uint8_t)49, (uint8_t)23, (uint8_t)245, (uint8_t)2, (uint8_t)225, (uint8_t)135, (uint8_t)16, (uint8_t)248, (uint8_t)20, (uint8_t)202, (uint8_t)107, (uint8_t)132, (uint8_t)188, (uint8_t)100, (uint8_t)125, (uint8_t)72, (uint8_t)210, (uint8_t)232, (uint8_t)208, (uint8_t)156, (uint8_t)149, (uint8_t)92, (uint8_t)107, (uint8_t)171, (uint8_t)6, (uint8_t)239, (uint8_t)158, (uint8_t)17, (uint8_t)51, (uint8_t)199, (uint8_t)224, (uint8_t)73, (uint8_t)28, (uint8_t)25, (uint8_t)158, (uint8_t)211, (uint8_t)89, (uint8_t)128, (uint8_t)11, (uint8_t)193, (uint8_t)70, (uint8_t)136, (uint8_t)46, (uint8_t)208, (uint8_t)51, (uint8_t)249, (uint8_t)106, (uint8_t)121, (uint8_t)170, (uint8_t)62, (uint8_t)9, (uint8_t)64, (uint8_t)184, (uint8_t)147, (uint8_t)214, (uint8_t)169, (uint8_t)50, (uint8_t)27, (uint8_t)104, (uint8_t)75, (uint8_t)128, (uint8_t)119, (uint8_t)126, (uint8_t)229, (uint8_t)222, (uint8_t)59, (uint8_t)158, (uint8_t)38, (uint8_t)137, (uint8_t)41, (uint8_t)41, (uint8_t)23, (uint8_t)203, (uint8_t)107, (uint8_t)160, (uint8_t)172, (uint8_t)60, (uint8_t)143, (uint8_t)59, (uint8_t)226, (uint8_t)249, (uint8_t)237, (uint8_t)167, (uint8_t)183, (uint8_t)40, (uint8_t)194, (uint8_t)124, (uint8_t)106, (uint8_t)143, (uint8_t)178, (uint8_t)215, (uint8_t)13, (uint8_t)18, (uint8_t)216, (uint8_t)151, (uint8_t)144, (uint8_t)240, (uint8_t)15, (uint8_t)81, (uint8_t)232, (uint8_t)15, (uint8_t)141, (uint8_t)114, (uint8_t)59, (uint8_t)231, (uint8_t)120, (uint8_t)15, (uint8_t)108, (uint8_t)59, (uint8_t)21, (uint8_t)127, (uint8_t)216, (uint8_t)117, (uint8_t)105, (uint8_t)176, (uint8_t)81, (uint8_t)255, (uint8_t)243, (uint8_t)44, (uint8_t)190, (uint8_t)85, (uint8_t)153, (uint8_t)20, (uint8_t)94, (uint8_t)126, (uint8_t)225, (uint8_t)101, (uint8_t)245, (uint8_t)43, (uint8_t)162, (uint8_t)84, (uint8_t)249, (uint8_t)57, (uint8_t)176, (uint8_t)8, (uint8_t)161, (uint8_t)81, (uint8_t)183, (uint8_t)85, (uint8_t)162, (uint8_t)83, (uint8_t)133, (uint8_t)82, (uint8_t)240, (uint8_t)185, (uint8_t)123, (uint8_t)89, (uint8_t)127, (uint8_t)68, (uint8_t)59, (uint8_t)8, (uint8_t)218, (uint8_t)45, (uint8_t)9, (uint8_t)134, (uint8_t)220, (uint8_t)6, (uint8_t)108, (uint8_t)178, (uint8_t)93, (uint8_t)103, (uint8_t)61, (uint8_t)124, (uint8_t)164, (uint8_t)24, (uint8_t)195, (uint8_t)185, (uint8_t)254, (uint8_t)66, (uint8_t)200, (uint8_t)165, (uint8_t)22, (uint8_t)215, (uint8_t)179, (uint8_t)163, (uint8_t)163, (uint8_t)82, (uint8_t)254, (uint8_t)137, (uint8_t)109, (uint8_t)196, (uint8_t)96, (uint8_t)226, (uint8_t)248, (uint8_t)136, (uint8_t)160, (uint8_t)189, (uint8_t)191, (uint8_t)202, (uint8_t)89, (uint8_t)53, (uint8_t)115, (uint8_t)157, (uint8_t)143, (uint8_t)21, (uint8_t)23, (uint8_t)35, (uint8_t)200, (uint8_t)201, (uint8_t)224, (uint8_t)40, (uint8_t)177, (uint8_t)140, (uint8_t)88, (uint8_t)23, (uint8_t)42, (uint8_t)203, (uint8_t)210, (uint8_t)100, (uint8_t)22, (uint8_t)184, (uint8_t)81, (uint8_t)227, (uint8_t)252, (uint8_t)195, (uint8_t)73, (uint8_t)167, (uint8_t)144, (uint8_t)182, (uint8_t)248, (uint8_t)83, (uint8_t)143, (uint8_t)233, (uint8_t)136, (uint8_t)19, (uint8_t)173, (uint8_t)174, (uint8_t)30, (uint8_t)54, (uint8_t)21, (uint8_t)90, (uint8_t)233, (uint8_t)155, (uint8_t)56, (uint8_t)197};
        p110_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TIMESYNC_111(), &PH);
    p111_tc1_SET((int64_t) -5452038257749512807L, PH.base.pack) ;
    p111_ts1_SET((int64_t)2011279633537756386L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_TRIGGER_112(), &PH);
    p112_time_usec_SET((uint64_t)907207691952924066L, PH.base.pack) ;
    p112_seq_SET((uint32_t)1866668509L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_GPS_113(), &PH);
    p113_time_usec_SET((uint64_t)2014743544093002006L, PH.base.pack) ;
    p113_fix_type_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
    p113_lat_SET((int32_t)1488839239, PH.base.pack) ;
    p113_lon_SET((int32_t) -1838906527, PH.base.pack) ;
    p113_alt_SET((int32_t) -1388072710, PH.base.pack) ;
    p113_eph_SET((uint16_t)(uint16_t)14377, PH.base.pack) ;
    p113_epv_SET((uint16_t)(uint16_t)14061, PH.base.pack) ;
    p113_vel_SET((uint16_t)(uint16_t)35265, PH.base.pack) ;
    p113_vn_SET((int16_t)(int16_t)19264, PH.base.pack) ;
    p113_ve_SET((int16_t)(int16_t)5342, PH.base.pack) ;
    p113_vd_SET((int16_t)(int16_t)25211, PH.base.pack) ;
    p113_cog_SET((uint16_t)(uint16_t)6607, PH.base.pack) ;
    p113_satellites_visible_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
    p114_time_usec_SET((uint64_t)3436932824065492165L, PH.base.pack) ;
    p114_sensor_id_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
    p114_integration_time_us_SET((uint32_t)3613634747L, PH.base.pack) ;
    p114_integrated_x_SET((float) -2.1760229E38F, PH.base.pack) ;
    p114_integrated_y_SET((float) -1.1828678E38F, PH.base.pack) ;
    p114_integrated_xgyro_SET((float)1.9493801E38F, PH.base.pack) ;
    p114_integrated_ygyro_SET((float)2.8211296E38F, PH.base.pack) ;
    p114_integrated_zgyro_SET((float)2.7019881E38F, PH.base.pack) ;
    p114_temperature_SET((int16_t)(int16_t)15870, PH.base.pack) ;
    p114_quality_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
    p114_time_delta_distance_us_SET((uint32_t)1277453367L, PH.base.pack) ;
    p114_distance_SET((float) -2.0465576E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIL_STATE_QUATERNION_115(), &PH);
    p115_time_usec_SET((uint64_t)3544883005733159711L, PH.base.pack) ;
    {
        float  attitude_quaternion [] =  {-7.0750516E37F, -2.6530976E38F, -7.704404E37F, 3.2034263E37F};
        p115_attitude_quaternion_SET(&attitude_quaternion, 0, &PH.base.pack) ;
    }
    p115_rollspeed_SET((float)2.5637309E38F, PH.base.pack) ;
    p115_pitchspeed_SET((float) -3.3194453E38F, PH.base.pack) ;
    p115_yawspeed_SET((float)2.7804095E38F, PH.base.pack) ;
    p115_lat_SET((int32_t)110205200, PH.base.pack) ;
    p115_lon_SET((int32_t)1012405367, PH.base.pack) ;
    p115_alt_SET((int32_t)1892234692, PH.base.pack) ;
    p115_vx_SET((int16_t)(int16_t)23951, PH.base.pack) ;
    p115_vy_SET((int16_t)(int16_t) -12766, PH.base.pack) ;
    p115_vz_SET((int16_t)(int16_t)2432, PH.base.pack) ;
    p115_ind_airspeed_SET((uint16_t)(uint16_t)41509, PH.base.pack) ;
    p115_true_airspeed_SET((uint16_t)(uint16_t)36329, PH.base.pack) ;
    p115_xacc_SET((int16_t)(int16_t) -18391, PH.base.pack) ;
    p115_yacc_SET((int16_t)(int16_t) -1011, PH.base.pack) ;
    p115_zacc_SET((int16_t)(int16_t)6412, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU2_116(), &PH);
    p116_time_boot_ms_SET((uint32_t)1612254245L, PH.base.pack) ;
    p116_xacc_SET((int16_t)(int16_t) -119, PH.base.pack) ;
    p116_yacc_SET((int16_t)(int16_t)10737, PH.base.pack) ;
    p116_zacc_SET((int16_t)(int16_t)22082, PH.base.pack) ;
    p116_xgyro_SET((int16_t)(int16_t)30165, PH.base.pack) ;
    p116_ygyro_SET((int16_t)(int16_t)11828, PH.base.pack) ;
    p116_zgyro_SET((int16_t)(int16_t) -25733, PH.base.pack) ;
    p116_xmag_SET((int16_t)(int16_t) -7932, PH.base.pack) ;
    p116_ymag_SET((int16_t)(int16_t)55, PH.base.pack) ;
    p116_zmag_SET((int16_t)(int16_t) -8625, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_LIST_117(), &PH);
    p117_target_system_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
    p117_target_component_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
    p117_start_SET((uint16_t)(uint16_t)42903, PH.base.pack) ;
    p117_end_SET((uint16_t)(uint16_t)14120, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_ENTRY_118(), &PH);
    p118_id_SET((uint16_t)(uint16_t)21067, PH.base.pack) ;
    p118_num_logs_SET((uint16_t)(uint16_t)8690, PH.base.pack) ;
    p118_last_log_num_SET((uint16_t)(uint16_t)53503, PH.base.pack) ;
    p118_time_utc_SET((uint32_t)2690727321L, PH.base.pack) ;
    p118_size_SET((uint32_t)2205484252L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_DATA_119(), &PH);
    p119_target_system_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
    p119_target_component_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
    p119_id_SET((uint16_t)(uint16_t)49302, PH.base.pack) ;
    p119_ofs_SET((uint32_t)2182745244L, PH.base.pack) ;
    p119_count_SET((uint32_t)2728209359L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_DATA_120(), &PH);
    p120_id_SET((uint16_t)(uint16_t)54874, PH.base.pack) ;
    p120_ofs_SET((uint32_t)2412920259L, PH.base.pack) ;
    p120_count_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)237, (uint8_t)77, (uint8_t)184, (uint8_t)250, (uint8_t)1, (uint8_t)135, (uint8_t)173, (uint8_t)117, (uint8_t)234, (uint8_t)42, (uint8_t)227, (uint8_t)251, (uint8_t)92, (uint8_t)136, (uint8_t)172, (uint8_t)229, (uint8_t)7, (uint8_t)89, (uint8_t)225, (uint8_t)116, (uint8_t)40, (uint8_t)247, (uint8_t)213, (uint8_t)238, (uint8_t)215, (uint8_t)191, (uint8_t)15, (uint8_t)85, (uint8_t)108, (uint8_t)213, (uint8_t)68, (uint8_t)228, (uint8_t)68, (uint8_t)247, (uint8_t)220, (uint8_t)187, (uint8_t)46, (uint8_t)94, (uint8_t)161, (uint8_t)52, (uint8_t)76, (uint8_t)207, (uint8_t)196, (uint8_t)203, (uint8_t)7, (uint8_t)96, (uint8_t)84, (uint8_t)81, (uint8_t)109, (uint8_t)235, (uint8_t)217, (uint8_t)114, (uint8_t)44, (uint8_t)200, (uint8_t)136, (uint8_t)105, (uint8_t)83, (uint8_t)187, (uint8_t)174, (uint8_t)37, (uint8_t)66, (uint8_t)99, (uint8_t)195, (uint8_t)122, (uint8_t)255, (uint8_t)220, (uint8_t)238, (uint8_t)198, (uint8_t)10, (uint8_t)94, (uint8_t)145, (uint8_t)111, (uint8_t)117, (uint8_t)255, (uint8_t)133, (uint8_t)119, (uint8_t)100, (uint8_t)128, (uint8_t)143, (uint8_t)243, (uint8_t)246, (uint8_t)76, (uint8_t)59, (uint8_t)26, (uint8_t)135, (uint8_t)166, (uint8_t)200, (uint8_t)129, (uint8_t)163, (uint8_t)2};
        p120_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_ERASE_121(), &PH);
    p121_target_system_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
    p121_target_component_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_END_122(), &PH);
    p122_target_system_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
    p122_target_component_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_INJECT_DATA_123(), &PH);
    p123_target_system_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
    p123_target_component_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
    p123_len_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)86, (uint8_t)193, (uint8_t)128, (uint8_t)48, (uint8_t)113, (uint8_t)242, (uint8_t)193, (uint8_t)202, (uint8_t)210, (uint8_t)127, (uint8_t)72, (uint8_t)178, (uint8_t)23, (uint8_t)79, (uint8_t)20, (uint8_t)92, (uint8_t)241, (uint8_t)57, (uint8_t)160, (uint8_t)6, (uint8_t)155, (uint8_t)58, (uint8_t)49, (uint8_t)49, (uint8_t)136, (uint8_t)106, (uint8_t)40, (uint8_t)193, (uint8_t)101, (uint8_t)179, (uint8_t)113, (uint8_t)221, (uint8_t)123, (uint8_t)116, (uint8_t)209, (uint8_t)38, (uint8_t)51, (uint8_t)112, (uint8_t)43, (uint8_t)163, (uint8_t)118, (uint8_t)240, (uint8_t)248, (uint8_t)90, (uint8_t)47, (uint8_t)12, (uint8_t)34, (uint8_t)41, (uint8_t)70, (uint8_t)185, (uint8_t)155, (uint8_t)125, (uint8_t)207, (uint8_t)207, (uint8_t)238, (uint8_t)166, (uint8_t)66, (uint8_t)19, (uint8_t)249, (uint8_t)38, (uint8_t)142, (uint8_t)163, (uint8_t)108, (uint8_t)15, (uint8_t)168, (uint8_t)155, (uint8_t)28, (uint8_t)136, (uint8_t)224, (uint8_t)236, (uint8_t)68, (uint8_t)253, (uint8_t)152, (uint8_t)191, (uint8_t)119, (uint8_t)178, (uint8_t)204, (uint8_t)74, (uint8_t)97, (uint8_t)55, (uint8_t)144, (uint8_t)95, (uint8_t)70, (uint8_t)124, (uint8_t)44, (uint8_t)22, (uint8_t)110, (uint8_t)175, (uint8_t)229, (uint8_t)173, (uint8_t)206, (uint8_t)57, (uint8_t)53, (uint8_t)161, (uint8_t)186, (uint8_t)26, (uint8_t)201, (uint8_t)99, (uint8_t)51, (uint8_t)101, (uint8_t)13, (uint8_t)175, (uint8_t)71, (uint8_t)60, (uint8_t)192, (uint8_t)243, (uint8_t)6, (uint8_t)253, (uint8_t)32, (uint8_t)19};
        p123_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS2_RAW_124(), &PH);
    p124_time_usec_SET((uint64_t)4009127123087126405L, PH.base.pack) ;
    p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, PH.base.pack) ;
    p124_lat_SET((int32_t) -2036450809, PH.base.pack) ;
    p124_lon_SET((int32_t) -842412801, PH.base.pack) ;
    p124_alt_SET((int32_t)1608006264, PH.base.pack) ;
    p124_eph_SET((uint16_t)(uint16_t)48493, PH.base.pack) ;
    p124_epv_SET((uint16_t)(uint16_t)18908, PH.base.pack) ;
    p124_vel_SET((uint16_t)(uint16_t)28801, PH.base.pack) ;
    p124_cog_SET((uint16_t)(uint16_t)32874, PH.base.pack) ;
    p124_satellites_visible_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
    p124_dgps_numch_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
    p124_dgps_age_SET((uint32_t)265975584L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_POWER_STATUS_125(), &PH);
    p125_Vcc_SET((uint16_t)(uint16_t)59348, PH.base.pack) ;
    p125_Vservo_SET((uint16_t)(uint16_t)24289, PH.base.pack) ;
    p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SERIAL_CONTROL_126(), &PH);
    p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1, PH.base.pack) ;
    p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE, PH.base.pack) ;
    p126_timeout_SET((uint16_t)(uint16_t)13661, PH.base.pack) ;
    p126_baudrate_SET((uint32_t)1532782763L, PH.base.pack) ;
    p126_count_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)153, (uint8_t)113, (uint8_t)58, (uint8_t)120, (uint8_t)68, (uint8_t)8, (uint8_t)42, (uint8_t)179, (uint8_t)180, (uint8_t)61, (uint8_t)146, (uint8_t)220, (uint8_t)120, (uint8_t)18, (uint8_t)90, (uint8_t)132, (uint8_t)132, (uint8_t)5, (uint8_t)243, (uint8_t)224, (uint8_t)19, (uint8_t)249, (uint8_t)50, (uint8_t)91, (uint8_t)57, (uint8_t)168, (uint8_t)229, (uint8_t)147, (uint8_t)179, (uint8_t)125, (uint8_t)210, (uint8_t)243, (uint8_t)139, (uint8_t)97, (uint8_t)119, (uint8_t)154, (uint8_t)56, (uint8_t)115, (uint8_t)128, (uint8_t)55, (uint8_t)211, (uint8_t)37, (uint8_t)15, (uint8_t)3, (uint8_t)7, (uint8_t)26, (uint8_t)103, (uint8_t)103, (uint8_t)126, (uint8_t)7, (uint8_t)113, (uint8_t)130, (uint8_t)141, (uint8_t)140, (uint8_t)229, (uint8_t)206, (uint8_t)94, (uint8_t)54, (uint8_t)56, (uint8_t)7, (uint8_t)168, (uint8_t)5, (uint8_t)116, (uint8_t)88, (uint8_t)238, (uint8_t)29, (uint8_t)23, (uint8_t)16, (uint8_t)43, (uint8_t)12};
        p126_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RTK_127(), &PH);
    p127_time_last_baseline_ms_SET((uint32_t)1532634604L, PH.base.pack) ;
    p127_rtk_receiver_id_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
    p127_wn_SET((uint16_t)(uint16_t)21578, PH.base.pack) ;
    p127_tow_SET((uint32_t)976297281L, PH.base.pack) ;
    p127_rtk_health_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    p127_rtk_rate_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
    p127_nsats_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
    p127_baseline_coords_type_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
    p127_baseline_a_mm_SET((int32_t)1826266307, PH.base.pack) ;
    p127_baseline_b_mm_SET((int32_t) -233198197, PH.base.pack) ;
    p127_baseline_c_mm_SET((int32_t)229280422, PH.base.pack) ;
    p127_accuracy_SET((uint32_t)329782799L, PH.base.pack) ;
    p127_iar_num_hypotheses_SET((int32_t) -1419488049, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS2_RTK_128(), &PH);
    p128_time_last_baseline_ms_SET((uint32_t)749301077L, PH.base.pack) ;
    p128_rtk_receiver_id_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
    p128_wn_SET((uint16_t)(uint16_t)31481, PH.base.pack) ;
    p128_tow_SET((uint32_t)3979412524L, PH.base.pack) ;
    p128_rtk_health_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
    p128_rtk_rate_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
    p128_nsats_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
    p128_baseline_coords_type_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
    p128_baseline_a_mm_SET((int32_t)1926327859, PH.base.pack) ;
    p128_baseline_b_mm_SET((int32_t) -331215195, PH.base.pack) ;
    p128_baseline_c_mm_SET((int32_t)1948021500, PH.base.pack) ;
    p128_accuracy_SET((uint32_t)1750915098L, PH.base.pack) ;
    p128_iar_num_hypotheses_SET((int32_t)495648608, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_IMU3_129(), &PH);
    p129_time_boot_ms_SET((uint32_t)3229036313L, PH.base.pack) ;
    p129_xacc_SET((int16_t)(int16_t)29311, PH.base.pack) ;
    p129_yacc_SET((int16_t)(int16_t)11324, PH.base.pack) ;
    p129_zacc_SET((int16_t)(int16_t) -32189, PH.base.pack) ;
    p129_xgyro_SET((int16_t)(int16_t)19732, PH.base.pack) ;
    p129_ygyro_SET((int16_t)(int16_t) -5522, PH.base.pack) ;
    p129_zgyro_SET((int16_t)(int16_t)15692, PH.base.pack) ;
    p129_xmag_SET((int16_t)(int16_t) -6611, PH.base.pack) ;
    p129_ymag_SET((int16_t)(int16_t) -19512, PH.base.pack) ;
    p129_zmag_SET((int16_t)(int16_t)16395, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
    p130_type_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    p130_size_SET((uint32_t)1425973067L, PH.base.pack) ;
    p130_width_SET((uint16_t)(uint16_t)27644, PH.base.pack) ;
    p130_height_SET((uint16_t)(uint16_t)46506, PH.base.pack) ;
    p130_packets_SET((uint16_t)(uint16_t)32980, PH.base.pack) ;
    p130_payload_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
    p130_jpg_quality_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ENCAPSULATED_DATA_131(), &PH);
    p131_seqnr_SET((uint16_t)(uint16_t)17249, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)132, (uint8_t)145, (uint8_t)76, (uint8_t)234, (uint8_t)23, (uint8_t)104, (uint8_t)48, (uint8_t)43, (uint8_t)30, (uint8_t)34, (uint8_t)213, (uint8_t)141, (uint8_t)233, (uint8_t)46, (uint8_t)55, (uint8_t)182, (uint8_t)192, (uint8_t)135, (uint8_t)37, (uint8_t)33, (uint8_t)200, (uint8_t)110, (uint8_t)154, (uint8_t)139, (uint8_t)89, (uint8_t)198, (uint8_t)163, (uint8_t)222, (uint8_t)92, (uint8_t)55, (uint8_t)52, (uint8_t)38, (uint8_t)108, (uint8_t)210, (uint8_t)32, (uint8_t)228, (uint8_t)80, (uint8_t)193, (uint8_t)225, (uint8_t)160, (uint8_t)84, (uint8_t)28, (uint8_t)207, (uint8_t)119, (uint8_t)115, (uint8_t)113, (uint8_t)108, (uint8_t)145, (uint8_t)202, (uint8_t)67, (uint8_t)99, (uint8_t)180, (uint8_t)156, (uint8_t)119, (uint8_t)66, (uint8_t)129, (uint8_t)51, (uint8_t)112, (uint8_t)52, (uint8_t)153, (uint8_t)179, (uint8_t)162, (uint8_t)15, (uint8_t)74, (uint8_t)211, (uint8_t)151, (uint8_t)136, (uint8_t)211, (uint8_t)139, (uint8_t)250, (uint8_t)14, (uint8_t)99, (uint8_t)98, (uint8_t)24, (uint8_t)12, (uint8_t)102, (uint8_t)143, (uint8_t)12, (uint8_t)253, (uint8_t)9, (uint8_t)166, (uint8_t)72, (uint8_t)81, (uint8_t)190, (uint8_t)60, (uint8_t)25, (uint8_t)78, (uint8_t)170, (uint8_t)18, (uint8_t)77, (uint8_t)245, (uint8_t)173, (uint8_t)21, (uint8_t)65, (uint8_t)194, (uint8_t)220, (uint8_t)191, (uint8_t)39, (uint8_t)30, (uint8_t)61, (uint8_t)52, (uint8_t)225, (uint8_t)192, (uint8_t)32, (uint8_t)68, (uint8_t)110, (uint8_t)182, (uint8_t)250, (uint8_t)109, (uint8_t)65, (uint8_t)27, (uint8_t)237, (uint8_t)173, (uint8_t)61, (uint8_t)99, (uint8_t)173, (uint8_t)51, (uint8_t)94, (uint8_t)107, (uint8_t)169, (uint8_t)227, (uint8_t)70, (uint8_t)24, (uint8_t)125, (uint8_t)122, (uint8_t)45, (uint8_t)36, (uint8_t)136, (uint8_t)120, (uint8_t)57, (uint8_t)24, (uint8_t)130, (uint8_t)174, (uint8_t)187, (uint8_t)131, (uint8_t)195, (uint8_t)55, (uint8_t)140, (uint8_t)120, (uint8_t)240, (uint8_t)46, (uint8_t)113, (uint8_t)209, (uint8_t)244, (uint8_t)169, (uint8_t)178, (uint8_t)93, (uint8_t)86, (uint8_t)142, (uint8_t)120, (uint8_t)154, (uint8_t)166, (uint8_t)202, (uint8_t)83, (uint8_t)209, (uint8_t)209, (uint8_t)4, (uint8_t)245, (uint8_t)150, (uint8_t)188, (uint8_t)155, (uint8_t)108, (uint8_t)43, (uint8_t)72, (uint8_t)101, (uint8_t)19, (uint8_t)120, (uint8_t)208, (uint8_t)109, (uint8_t)242, (uint8_t)96, (uint8_t)37, (uint8_t)122, (uint8_t)244, (uint8_t)82, (uint8_t)157, (uint8_t)54, (uint8_t)124, (uint8_t)81, (uint8_t)151, (uint8_t)29, (uint8_t)53, (uint8_t)213, (uint8_t)245, (uint8_t)129, (uint8_t)236, (uint8_t)183, (uint8_t)85, (uint8_t)162, (uint8_t)226, (uint8_t)193, (uint8_t)243, (uint8_t)16, (uint8_t)128, (uint8_t)54, (uint8_t)218, (uint8_t)100, (uint8_t)195, (uint8_t)103, (uint8_t)165, (uint8_t)231, (uint8_t)167, (uint8_t)90, (uint8_t)254, (uint8_t)149, (uint8_t)221, (uint8_t)204, (uint8_t)237, (uint8_t)12, (uint8_t)10, (uint8_t)252, (uint8_t)167, (uint8_t)69, (uint8_t)11, (uint8_t)237, (uint8_t)177, (uint8_t)53, (uint8_t)28, (uint8_t)23, (uint8_t)29, (uint8_t)199, (uint8_t)130, (uint8_t)241, (uint8_t)197, (uint8_t)230, (uint8_t)168, (uint8_t)166, (uint8_t)103, (uint8_t)202, (uint8_t)79, (uint8_t)244, (uint8_t)144, (uint8_t)98, (uint8_t)62, (uint8_t)72, (uint8_t)107, (uint8_t)161, (uint8_t)138, (uint8_t)239, (uint8_t)22, (uint8_t)125, (uint8_t)42, (uint8_t)221, (uint8_t)78, (uint8_t)189, (uint8_t)153, (uint8_t)153, (uint8_t)163, (uint8_t)31, (uint8_t)27, (uint8_t)45, (uint8_t)187, (uint8_t)34};
        p131_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DISTANCE_SENSOR_132(), &PH);
    p132_time_boot_ms_SET((uint32_t)847127236L, PH.base.pack) ;
    p132_min_distance_SET((uint16_t)(uint16_t)55612, PH.base.pack) ;
    p132_max_distance_SET((uint16_t)(uint16_t)26351, PH.base.pack) ;
    p132_current_distance_SET((uint16_t)(uint16_t)39650, PH.base.pack) ;
    p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
    p132_id_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
    p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_180_YAW_90, PH.base.pack) ;
    p132_covariance_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_REQUEST_133(), &PH);
    p133_lat_SET((int32_t)668235038, PH.base.pack) ;
    p133_lon_SET((int32_t) -1921810376, PH.base.pack) ;
    p133_grid_spacing_SET((uint16_t)(uint16_t)33614, PH.base.pack) ;
    p133_mask_SET((uint64_t)4032335248297603785L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_DATA_134(), &PH);
    p134_lat_SET((int32_t) -776977467, PH.base.pack) ;
    p134_lon_SET((int32_t) -1486953423, PH.base.pack) ;
    p134_grid_spacing_SET((uint16_t)(uint16_t)31004, PH.base.pack) ;
    p134_gridbit_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
    {
        int16_t  data_ [] =  {(int16_t)23504, (int16_t)21617, (int16_t)13411, (int16_t) -20764, (int16_t) -32516, (int16_t)25817, (int16_t) -2747, (int16_t)16042, (int16_t) -6527, (int16_t) -12695, (int16_t)29427, (int16_t) -1999, (int16_t) -4221, (int16_t) -15032, (int16_t) -11251, (int16_t)6702};
        p134_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_CHECK_135(), &PH);
    p135_lat_SET((int32_t) -1545504129, PH.base.pack) ;
    p135_lon_SET((int32_t) -1839888248, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_TERRAIN_REPORT_136(), &PH);
    p136_lat_SET((int32_t)733886233, PH.base.pack) ;
    p136_lon_SET((int32_t)1646113062, PH.base.pack) ;
    p136_spacing_SET((uint16_t)(uint16_t)30402, PH.base.pack) ;
    p136_terrain_height_SET((float) -3.2721715E37F, PH.base.pack) ;
    p136_current_height_SET((float) -1.1947135E38F, PH.base.pack) ;
    p136_pending_SET((uint16_t)(uint16_t)16759, PH.base.pack) ;
    p136_loaded_SET((uint16_t)(uint16_t)49270, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE2_137(), &PH);
    p137_time_boot_ms_SET((uint32_t)3436000026L, PH.base.pack) ;
    p137_press_abs_SET((float) -7.137068E37F, PH.base.pack) ;
    p137_press_diff_SET((float) -3.0676978E38F, PH.base.pack) ;
    p137_temperature_SET((int16_t)(int16_t) -22953, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ATT_POS_MOCAP_138(), &PH);
    p138_time_usec_SET((uint64_t)1866491575833419255L, PH.base.pack) ;
    {
        float  q [] =  {-1.6001076E38F, 1.3726941E37F, 2.2977368E38F, -1.4650597E38F};
        p138_q_SET(&q, 0, &PH.base.pack) ;
    }
    p138_x_SET((float)3.169569E38F, PH.base.pack) ;
    p138_y_SET((float) -3.5428938E37F, PH.base.pack) ;
    p138_z_SET((float) -1.6634839E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
    p139_time_usec_SET((uint64_t)4496778388671120610L, PH.base.pack) ;
    p139_group_mlx_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
    p139_target_system_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
    p139_target_component_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
    {
        float  controls [] =  {2.017082E38F, -1.474979E38F, 7.3977584E37F, 1.4729402E38F, -1.2361875E38F, 3.0873123E37F, -6.699231E37F, -2.4344124E38F};
        p139_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
    p140_time_usec_SET((uint64_t)8109023925999662949L, PH.base.pack) ;
    p140_group_mlx_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
    {
        float  controls [] =  {-3.2683854E38F, 2.5959955E38F, -4.793536E37F, 2.0280325E38F, 2.7821586E38F, -2.7136683E37F, -1.9277228E38F, 2.783423E38F};
        p140_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ALTITUDE_141(), &PH);
    p141_time_usec_SET((uint64_t)4575325391312921176L, PH.base.pack) ;
    p141_altitude_monotonic_SET((float) -1.3064877E38F, PH.base.pack) ;
    p141_altitude_amsl_SET((float) -3.3645459E38F, PH.base.pack) ;
    p141_altitude_local_SET((float) -1.2452549E38F, PH.base.pack) ;
    p141_altitude_relative_SET((float)1.2618145E38F, PH.base.pack) ;
    p141_altitude_terrain_SET((float)2.914733E38F, PH.base.pack) ;
    p141_bottom_clearance_SET((float)2.335763E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_RESOURCE_REQUEST_142(), &PH);
    p142_request_id_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
    p142_uri_type_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
    {
        uint8_t  uri [] =  {(uint8_t)248, (uint8_t)122, (uint8_t)34, (uint8_t)228, (uint8_t)129, (uint8_t)108, (uint8_t)104, (uint8_t)32, (uint8_t)22, (uint8_t)148, (uint8_t)187, (uint8_t)190, (uint8_t)85, (uint8_t)235, (uint8_t)138, (uint8_t)90, (uint8_t)139, (uint8_t)26, (uint8_t)171, (uint8_t)36, (uint8_t)176, (uint8_t)193, (uint8_t)129, (uint8_t)152, (uint8_t)96, (uint8_t)163, (uint8_t)33, (uint8_t)115, (uint8_t)150, (uint8_t)196, (uint8_t)144, (uint8_t)222, (uint8_t)86, (uint8_t)58, (uint8_t)102, (uint8_t)103, (uint8_t)99, (uint8_t)101, (uint8_t)221, (uint8_t)180, (uint8_t)234, (uint8_t)112, (uint8_t)41, (uint8_t)5, (uint8_t)4, (uint8_t)41, (uint8_t)156, (uint8_t)146, (uint8_t)119, (uint8_t)236, (uint8_t)75, (uint8_t)105, (uint8_t)249, (uint8_t)152, (uint8_t)231, (uint8_t)91, (uint8_t)146, (uint8_t)182, (uint8_t)117, (uint8_t)189, (uint8_t)66, (uint8_t)22, (uint8_t)168, (uint8_t)55, (uint8_t)158, (uint8_t)144, (uint8_t)187, (uint8_t)248, (uint8_t)92, (uint8_t)90, (uint8_t)47, (uint8_t)121, (uint8_t)27, (uint8_t)248, (uint8_t)190, (uint8_t)248, (uint8_t)103, (uint8_t)253, (uint8_t)19, (uint8_t)189, (uint8_t)240, (uint8_t)40, (uint8_t)34, (uint8_t)253, (uint8_t)59, (uint8_t)253, (uint8_t)70, (uint8_t)239, (uint8_t)75, (uint8_t)3, (uint8_t)196, (uint8_t)76, (uint8_t)69, (uint8_t)1, (uint8_t)3, (uint8_t)69, (uint8_t)66, (uint8_t)170, (uint8_t)114, (uint8_t)229, (uint8_t)233, (uint8_t)237, (uint8_t)158, (uint8_t)90, (uint8_t)77, (uint8_t)4, (uint8_t)7, (uint8_t)75, (uint8_t)110, (uint8_t)59, (uint8_t)43, (uint8_t)122, (uint8_t)198, (uint8_t)174, (uint8_t)145, (uint8_t)79, (uint8_t)130, (uint8_t)170, (uint8_t)253, (uint8_t)182};
        p142_uri_SET(&uri, 0, &PH.base.pack) ;
    }
    p142_transfer_type_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
    {
        uint8_t  storage [] =  {(uint8_t)45, (uint8_t)123, (uint8_t)191, (uint8_t)94, (uint8_t)97, (uint8_t)11, (uint8_t)164, (uint8_t)127, (uint8_t)109, (uint8_t)26, (uint8_t)32, (uint8_t)28, (uint8_t)121, (uint8_t)188, (uint8_t)105, (uint8_t)195, (uint8_t)104, (uint8_t)39, (uint8_t)95, (uint8_t)105, (uint8_t)78, (uint8_t)140, (uint8_t)231, (uint8_t)35, (uint8_t)34, (uint8_t)5, (uint8_t)153, (uint8_t)166, (uint8_t)176, (uint8_t)155, (uint8_t)236, (uint8_t)85, (uint8_t)62, (uint8_t)191, (uint8_t)202, (uint8_t)109, (uint8_t)11, (uint8_t)198, (uint8_t)48, (uint8_t)65, (uint8_t)107, (uint8_t)151, (uint8_t)25, (uint8_t)191, (uint8_t)196, (uint8_t)54, (uint8_t)6, (uint8_t)131, (uint8_t)17, (uint8_t)216, (uint8_t)20, (uint8_t)56, (uint8_t)77, (uint8_t)243, (uint8_t)44, (uint8_t)161, (uint8_t)98, (uint8_t)248, (uint8_t)62, (uint8_t)20, (uint8_t)123, (uint8_t)56, (uint8_t)130, (uint8_t)129, (uint8_t)235, (uint8_t)37, (uint8_t)184, (uint8_t)43, (uint8_t)244, (uint8_t)211, (uint8_t)172, (uint8_t)203, (uint8_t)223, (uint8_t)177, (uint8_t)162, (uint8_t)75, (uint8_t)91, (uint8_t)174, (uint8_t)89, (uint8_t)144, (uint8_t)160, (uint8_t)227, (uint8_t)45, (uint8_t)148, (uint8_t)169, (uint8_t)16, (uint8_t)152, (uint8_t)191, (uint8_t)94, (uint8_t)242, (uint8_t)10, (uint8_t)242, (uint8_t)192, (uint8_t)248, (uint8_t)40, (uint8_t)41, (uint8_t)191, (uint8_t)245, (uint8_t)92, (uint8_t)17, (uint8_t)121, (uint8_t)20, (uint8_t)98, (uint8_t)105, (uint8_t)67, (uint8_t)91, (uint8_t)97, (uint8_t)70, (uint8_t)45, (uint8_t)252, (uint8_t)245, (uint8_t)100, (uint8_t)227, (uint8_t)121, (uint8_t)52, (uint8_t)29, (uint8_t)192, (uint8_t)226, (uint8_t)54, (uint8_t)242};
        p142_storage_SET(&storage, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE3_143(), &PH);
    p143_time_boot_ms_SET((uint32_t)261829869L, PH.base.pack) ;
    p143_press_abs_SET((float) -1.2506271E38F, PH.base.pack) ;
    p143_press_diff_SET((float) -1.982665E38F, PH.base.pack) ;
    p143_temperature_SET((int16_t)(int16_t) -12811, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FOLLOW_TARGET_144(), &PH);
    p144_timestamp_SET((uint64_t)6045803290026852477L, PH.base.pack) ;
    p144_est_capabilities_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    p144_lat_SET((int32_t)1373909448, PH.base.pack) ;
    p144_lon_SET((int32_t) -590161083, PH.base.pack) ;
    p144_alt_SET((float) -3.0640444E38F, PH.base.pack) ;
    {
        float  vel [] =  {-1.4883996E38F, -1.1739039E38F, 1.2443105E37F};
        p144_vel_SET(&vel, 0, &PH.base.pack) ;
    }
    {
        float  acc [] =  {-3.2802792E38F, -1.5897382E38F, 1.4887215E38F};
        p144_acc_SET(&acc, 0, &PH.base.pack) ;
    }
    {
        float  attitude_q [] =  {1.874772E38F, -2.4481087E38F, -2.8991275E38F, -2.7960545E38F};
        p144_attitude_q_SET(&attitude_q, 0, &PH.base.pack) ;
    }
    {
        float  rates [] =  {-1.3956016E38F, 9.747554E37F, 8.49404E37F};
        p144_rates_SET(&rates, 0, &PH.base.pack) ;
    }
    {
        float  position_cov [] =  {-1.228028E38F, 2.8052126E38F, 2.6634832E38F};
        p144_position_cov_SET(&position_cov, 0, &PH.base.pack) ;
    }
    p144_custom_state_SET((uint64_t)1976436519289076784L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
    p146_time_usec_SET((uint64_t)1728323603532146237L, PH.base.pack) ;
    p146_x_acc_SET((float) -6.817792E37F, PH.base.pack) ;
    p146_y_acc_SET((float) -2.7002918E37F, PH.base.pack) ;
    p146_z_acc_SET((float)2.287702E38F, PH.base.pack) ;
    p146_x_vel_SET((float)1.4648989E38F, PH.base.pack) ;
    p146_y_vel_SET((float) -2.6816248E38F, PH.base.pack) ;
    p146_z_vel_SET((float) -3.7985404E37F, PH.base.pack) ;
    p146_x_pos_SET((float)6.7723214E37F, PH.base.pack) ;
    p146_y_pos_SET((float)2.0473306E38F, PH.base.pack) ;
    p146_z_pos_SET((float)4.886804E37F, PH.base.pack) ;
    p146_airspeed_SET((float) -1.1392275E38F, PH.base.pack) ;
    {
        float  vel_variance [] =  {2.1476985E38F, -1.5984362E38F, 2.5650482E38F};
        p146_vel_variance_SET(&vel_variance, 0, &PH.base.pack) ;
    }
    {
        float  pos_variance [] =  {-1.9138887E38F, 3.2065078E38F, 1.4733604E38F};
        p146_pos_variance_SET(&pos_variance, 0, &PH.base.pack) ;
    }
    {
        float  q [] =  {-2.6240647E38F, -4.668087E37F, 4.9649544E37F, -2.491982E38F};
        p146_q_SET(&q, 0, &PH.base.pack) ;
    }
    p146_roll_rate_SET((float) -1.402613E38F, PH.base.pack) ;
    p146_pitch_rate_SET((float) -8.847158E37F, PH.base.pack) ;
    p146_yaw_rate_SET((float) -2.8475416E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_BATTERY_STATUS_147(), &PH);
    p147_id_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
    p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_UNKNOWN, PH.base.pack) ;
    p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_UNKNOWN, PH.base.pack) ;
    p147_temperature_SET((int16_t)(int16_t)15497, PH.base.pack) ;
    {
        uint16_t  voltages [] =  {(uint16_t)12040, (uint16_t)47061, (uint16_t)29088, (uint16_t)48494, (uint16_t)45621, (uint16_t)61389, (uint16_t)41726, (uint16_t)35137, (uint16_t)4769, (uint16_t)5234};
        p147_voltages_SET(&voltages, 0, &PH.base.pack) ;
    }
    p147_current_battery_SET((int16_t)(int16_t)28460, PH.base.pack) ;
    p147_current_consumed_SET((int32_t)764836157, PH.base.pack) ;
    p147_energy_consumed_SET((int32_t) -466233321, PH.base.pack) ;
    p147_battery_remaining_SET((int8_t)(int8_t)1, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_148(), &PH);
    p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION, PH.base.pack) ;
    p148_flight_sw_version_SET((uint32_t)524015875L, PH.base.pack) ;
    p148_middleware_sw_version_SET((uint32_t)2647086037L, PH.base.pack) ;
    p148_os_sw_version_SET((uint32_t)1469392758L, PH.base.pack) ;
    p148_board_version_SET((uint32_t)4275606187L, PH.base.pack) ;
    {
        uint8_t  flight_custom_version [] =  {(uint8_t)176, (uint8_t)254, (uint8_t)11, (uint8_t)40, (uint8_t)92, (uint8_t)56, (uint8_t)243, (uint8_t)14};
        p148_flight_custom_version_SET(&flight_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  middleware_custom_version [] =  {(uint8_t)62, (uint8_t)165, (uint8_t)161, (uint8_t)162, (uint8_t)133, (uint8_t)30, (uint8_t)232, (uint8_t)92};
        p148_middleware_custom_version_SET(&middleware_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  os_custom_version [] =  {(uint8_t)187, (uint8_t)172, (uint8_t)3, (uint8_t)92, (uint8_t)238, (uint8_t)4, (uint8_t)172, (uint8_t)29};
        p148_os_custom_version_SET(&os_custom_version, 0, &PH.base.pack) ;
    }
    p148_vendor_id_SET((uint16_t)(uint16_t)41199, PH.base.pack) ;
    p148_product_id_SET((uint16_t)(uint16_t)2574, PH.base.pack) ;
    p148_uid_SET((uint64_t)4342755093189230267L, PH.base.pack) ;
    {
        uint8_t  uid2 [] =  {(uint8_t)176, (uint8_t)212, (uint8_t)202, (uint8_t)212, (uint8_t)163, (uint8_t)172, (uint8_t)83, (uint8_t)96, (uint8_t)212, (uint8_t)29, (uint8_t)110, (uint8_t)65, (uint8_t)118, (uint8_t)145, (uint8_t)35, (uint8_t)23, (uint8_t)184, (uint8_t)145};
        p148_uid2_SET(&uid2, 0,  &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LANDING_TARGET_149(), &PH);
    p149_time_usec_SET((uint64_t)7246088299113754930L, PH.base.pack) ;
    p149_target_num_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
    p149_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
    p149_angle_x_SET((float)3.154197E38F, PH.base.pack) ;
    p149_angle_y_SET((float)2.215269E38F, PH.base.pack) ;
    p149_distance_SET((float) -1.2429037E38F, PH.base.pack) ;
    p149_size_x_SET((float)8.024529E37F, PH.base.pack) ;
    p149_size_y_SET((float)1.6906002E38F, PH.base.pack) ;
    p149_x_SET((float)1.9568777E38F, &PH) ;
    p149_y_SET((float)2.3378012E37F, &PH) ;
    p149_z_SET((float)8.057441E37F, &PH) ;
    {
        float  q [] =  {9.269981E37F, -1.1792666E38F, -5.387745E37F, -2.307458E38F};
        p149_q_SET(&q, 0,  &PH) ;
    }
    p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, PH.base.pack) ;
    p149_position_valid_SET((uint8_t)(uint8_t)109, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ESTIMATOR_STATUS_230(), &PH);
    p230_time_usec_SET((uint64_t)1371434895886686228L, PH.base.pack) ;
    p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_CONST_POS_MODE, PH.base.pack) ;
    p230_vel_ratio_SET((float) -4.5910715E37F, PH.base.pack) ;
    p230_pos_horiz_ratio_SET((float)1.5514264E38F, PH.base.pack) ;
    p230_pos_vert_ratio_SET((float)1.8754081E38F, PH.base.pack) ;
    p230_mag_ratio_SET((float) -2.0870782E38F, PH.base.pack) ;
    p230_hagl_ratio_SET((float)1.8798617E38F, PH.base.pack) ;
    p230_tas_ratio_SET((float) -1.3197082E38F, PH.base.pack) ;
    p230_pos_horiz_accuracy_SET((float)1.3692411E38F, PH.base.pack) ;
    p230_pos_vert_accuracy_SET((float)2.9526047E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_WIND_COV_231(), &PH);
    p231_time_usec_SET((uint64_t)6228869940680440252L, PH.base.pack) ;
    p231_wind_x_SET((float) -2.5730546E37F, PH.base.pack) ;
    p231_wind_y_SET((float)1.2918715E37F, PH.base.pack) ;
    p231_wind_z_SET((float) -1.026476E37F, PH.base.pack) ;
    p231_var_horiz_SET((float) -2.5111203E37F, PH.base.pack) ;
    p231_var_vert_SET((float)1.0797022E38F, PH.base.pack) ;
    p231_wind_alt_SET((float)1.2952337E38F, PH.base.pack) ;
    p231_horiz_accuracy_SET((float)2.269672E38F, PH.base.pack) ;
    p231_vert_accuracy_SET((float) -1.1839499E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_INPUT_232(), &PH);
    p232_time_usec_SET((uint64_t)7035567511162908663L, PH.base.pack) ;
    p232_gps_id_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
    p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY, PH.base.pack) ;
    p232_time_week_ms_SET((uint32_t)865837995L, PH.base.pack) ;
    p232_time_week_SET((uint16_t)(uint16_t)26984, PH.base.pack) ;
    p232_fix_type_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
    p232_lat_SET((int32_t) -1952406796, PH.base.pack) ;
    p232_lon_SET((int32_t)915024779, PH.base.pack) ;
    p232_alt_SET((float)1.474923E37F, PH.base.pack) ;
    p232_hdop_SET((float) -3.023848E38F, PH.base.pack) ;
    p232_vdop_SET((float)2.7206733E38F, PH.base.pack) ;
    p232_vn_SET((float) -2.836222E38F, PH.base.pack) ;
    p232_ve_SET((float)1.2103829E38F, PH.base.pack) ;
    p232_vd_SET((float)3.105435E38F, PH.base.pack) ;
    p232_speed_accuracy_SET((float)2.8453263E38F, PH.base.pack) ;
    p232_horiz_accuracy_SET((float) -6.6159603E37F, PH.base.pack) ;
    p232_vert_accuracy_SET((float) -2.1331556E38F, PH.base.pack) ;
    p232_satellites_visible_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_GPS_RTCM_DATA_233(), &PH);
    p233_flags_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
    p233_len_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)130, (uint8_t)148, (uint8_t)166, (uint8_t)217, (uint8_t)38, (uint8_t)2, (uint8_t)33, (uint8_t)223, (uint8_t)29, (uint8_t)105, (uint8_t)155, (uint8_t)33, (uint8_t)182, (uint8_t)110, (uint8_t)73, (uint8_t)109, (uint8_t)151, (uint8_t)231, (uint8_t)125, (uint8_t)69, (uint8_t)164, (uint8_t)60, (uint8_t)38, (uint8_t)75, (uint8_t)154, (uint8_t)68, (uint8_t)171, (uint8_t)158, (uint8_t)47, (uint8_t)248, (uint8_t)118, (uint8_t)53, (uint8_t)54, (uint8_t)117, (uint8_t)207, (uint8_t)72, (uint8_t)44, (uint8_t)78, (uint8_t)240, (uint8_t)176, (uint8_t)169, (uint8_t)184, (uint8_t)208, (uint8_t)41, (uint8_t)101, (uint8_t)30, (uint8_t)112, (uint8_t)210, (uint8_t)76, (uint8_t)176, (uint8_t)169, (uint8_t)103, (uint8_t)221, (uint8_t)66, (uint8_t)16, (uint8_t)124, (uint8_t)26, (uint8_t)245, (uint8_t)226, (uint8_t)116, (uint8_t)97, (uint8_t)105, (uint8_t)8, (uint8_t)46, (uint8_t)111, (uint8_t)206, (uint8_t)171, (uint8_t)187, (uint8_t)211, (uint8_t)55, (uint8_t)94, (uint8_t)226, (uint8_t)238, (uint8_t)117, (uint8_t)95, (uint8_t)7, (uint8_t)136, (uint8_t)35, (uint8_t)47, (uint8_t)13, (uint8_t)234, (uint8_t)81, (uint8_t)109, (uint8_t)88, (uint8_t)132, (uint8_t)242, (uint8_t)85, (uint8_t)102, (uint8_t)249, (uint8_t)45, (uint8_t)84, (uint8_t)252, (uint8_t)245, (uint8_t)245, (uint8_t)255, (uint8_t)22, (uint8_t)138, (uint8_t)76, (uint8_t)152, (uint8_t)101, (uint8_t)22, (uint8_t)198, (uint8_t)31, (uint8_t)173, (uint8_t)68, (uint8_t)217, (uint8_t)79, (uint8_t)161, (uint8_t)74, (uint8_t)150, (uint8_t)28, (uint8_t)28, (uint8_t)1, (uint8_t)58, (uint8_t)18, (uint8_t)8, (uint8_t)125, (uint8_t)54, (uint8_t)234, (uint8_t)115, (uint8_t)153, (uint8_t)206, (uint8_t)201, (uint8_t)78, (uint8_t)123, (uint8_t)212, (uint8_t)48, (uint8_t)131, (uint8_t)90, (uint8_t)9, (uint8_t)222, (uint8_t)6, (uint8_t)139, (uint8_t)127, (uint8_t)38, (uint8_t)123, (uint8_t)45, (uint8_t)65, (uint8_t)135, (uint8_t)252, (uint8_t)52, (uint8_t)231, (uint8_t)32, (uint8_t)206, (uint8_t)168, (uint8_t)74, (uint8_t)50, (uint8_t)40, (uint8_t)231, (uint8_t)9, (uint8_t)58, (uint8_t)5, (uint8_t)158, (uint8_t)8, (uint8_t)230, (uint8_t)66, (uint8_t)226, (uint8_t)142, (uint8_t)100, (uint8_t)40, (uint8_t)106, (uint8_t)45, (uint8_t)155, (uint8_t)141, (uint8_t)26, (uint8_t)23, (uint8_t)53, (uint8_t)29, (uint8_t)170, (uint8_t)49, (uint8_t)49, (uint8_t)192, (uint8_t)22, (uint8_t)68, (uint8_t)110, (uint8_t)136, (uint8_t)129, (uint8_t)115, (uint8_t)66, (uint8_t)190};
        p233_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HIGH_LATENCY_234(), &PH);
    p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PH.base.pack) ;
    p234_custom_mode_SET((uint32_t)2827873708L, PH.base.pack) ;
    p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, PH.base.pack) ;
    p234_roll_SET((int16_t)(int16_t) -28712, PH.base.pack) ;
    p234_pitch_SET((int16_t)(int16_t) -13135, PH.base.pack) ;
    p234_heading_SET((uint16_t)(uint16_t)17638, PH.base.pack) ;
    p234_throttle_SET((int8_t)(int8_t) -55, PH.base.pack) ;
    p234_heading_sp_SET((int16_t)(int16_t) -9664, PH.base.pack) ;
    p234_latitude_SET((int32_t) -990078822, PH.base.pack) ;
    p234_longitude_SET((int32_t) -1604088041, PH.base.pack) ;
    p234_altitude_amsl_SET((int16_t)(int16_t) -19394, PH.base.pack) ;
    p234_altitude_sp_SET((int16_t)(int16_t)2529, PH.base.pack) ;
    p234_airspeed_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
    p234_airspeed_sp_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
    p234_groundspeed_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
    p234_climb_rate_SET((int8_t)(int8_t)40, PH.base.pack) ;
    p234_gps_nsat_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
    p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, PH.base.pack) ;
    p234_battery_remaining_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
    p234_temperature_SET((int8_t)(int8_t)50, PH.base.pack) ;
    p234_temperature_air_SET((int8_t)(int8_t)85, PH.base.pack) ;
    p234_failsafe_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
    p234_wp_num_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
    p234_wp_distance_SET((uint16_t)(uint16_t)34376, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VIBRATION_241(), &PH);
    p241_time_usec_SET((uint64_t)6910677199700170897L, PH.base.pack) ;
    p241_vibration_x_SET((float)2.4472137E38F, PH.base.pack) ;
    p241_vibration_y_SET((float) -1.4458727E38F, PH.base.pack) ;
    p241_vibration_z_SET((float)4.3368903E37F, PH.base.pack) ;
    p241_clipping_0_SET((uint32_t)2161493271L, PH.base.pack) ;
    p241_clipping_1_SET((uint32_t)2740319927L, PH.base.pack) ;
    p241_clipping_2_SET((uint32_t)3063683587L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_HOME_POSITION_242(), &PH);
    p242_latitude_SET((int32_t) -445033275, PH.base.pack) ;
    p242_longitude_SET((int32_t)52798795, PH.base.pack) ;
    p242_altitude_SET((int32_t) -1472120634, PH.base.pack) ;
    p242_x_SET((float)3.9324045E37F, PH.base.pack) ;
    p242_y_SET((float) -2.916387E38F, PH.base.pack) ;
    p242_z_SET((float)1.0404469E38F, PH.base.pack) ;
    {
        float  q [] =  {-2.628586E38F, 2.9255465E38F, 1.5358514E38F, 2.9086073E38F};
        p242_q_SET(&q, 0, &PH.base.pack) ;
    }
    p242_approach_x_SET((float) -2.8862903E38F, PH.base.pack) ;
    p242_approach_y_SET((float) -1.3909747E38F, PH.base.pack) ;
    p242_approach_z_SET((float) -2.0561603E38F, PH.base.pack) ;
    p242_time_usec_SET((uint64_t)5819918013444366616L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_HOME_POSITION_243(), &PH);
    p243_target_system_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
    p243_latitude_SET((int32_t)1080720017, PH.base.pack) ;
    p243_longitude_SET((int32_t) -1461595523, PH.base.pack) ;
    p243_altitude_SET((int32_t) -1756176114, PH.base.pack) ;
    p243_x_SET((float) -2.3332717E37F, PH.base.pack) ;
    p243_y_SET((float) -1.6083233E38F, PH.base.pack) ;
    p243_z_SET((float)5.917876E37F, PH.base.pack) ;
    {
        float  q [] =  {1.4868828E38F, -3.2962433E38F, 1.2691164E38F, -1.3252333E38F};
        p243_q_SET(&q, 0, &PH.base.pack) ;
    }
    p243_approach_x_SET((float)1.2902509E38F, PH.base.pack) ;
    p243_approach_y_SET((float) -2.9411155E38F, PH.base.pack) ;
    p243_approach_z_SET((float) -2.3150703E38F, PH.base.pack) ;
    p243_time_usec_SET((uint64_t)5308301402664132851L, &PH) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MESSAGE_INTERVAL_244(), &PH);
    p244_message_id_SET((uint16_t)(uint16_t)38393, PH.base.pack) ;
    p244_interval_us_SET((int32_t)2011612291, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_EXTENDED_SYS_STATE_245(), &PH);
    p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_FW, PH.base.pack) ;
    p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_ADSB_VEHICLE_246(), &PH);
    p246_ICAO_address_SET((uint32_t)229871748L, PH.base.pack) ;
    p246_lat_SET((int32_t) -1119184935, PH.base.pack) ;
    p246_lon_SET((int32_t) -1270483466, PH.base.pack) ;
    p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, PH.base.pack) ;
    p246_altitude_SET((int32_t) -1022223999, PH.base.pack) ;
    p246_heading_SET((uint16_t)(uint16_t)46964, PH.base.pack) ;
    p246_hor_velocity_SET((uint16_t)(uint16_t)44359, PH.base.pack) ;
    p246_ver_velocity_SET((int16_t)(int16_t)27928, PH.base.pack) ;
    {
        char16_t   callsign = "udJruccsb";
        p246_callsign_SET(&callsign, 0,  sizeof(callsign), &PH) ;
    }
    p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UNASSIGNED, PH.base.pack) ;
    p246_tslc_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
    p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_ALTITUDE, PH.base.pack) ;
    p246_squawk_SET((uint16_t)(uint16_t)50435, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_COLLISION_247(), &PH);
    p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, PH.base.pack) ;
    p247_id_SET((uint32_t)3149864843L, PH.base.pack) ;
    p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_PERPENDICULAR, PH.base.pack) ;
    p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW, PH.base.pack) ;
    p247_time_to_minimum_delta_SET((float)2.0290823E38F, PH.base.pack) ;
    p247_altitude_minimum_delta_SET((float)2.1496634E38F, PH.base.pack) ;
    p247_horizontal_minimum_delta_SET((float)6.483975E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_V2_EXTENSION_248(), &PH);
    p248_target_network_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
    p248_target_system_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
    p248_target_component_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
    p248_message_type_SET((uint16_t)(uint16_t)1545, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)14, (uint8_t)144, (uint8_t)66, (uint8_t)183, (uint8_t)241, (uint8_t)247, (uint8_t)39, (uint8_t)50, (uint8_t)157, (uint8_t)185, (uint8_t)1, (uint8_t)17, (uint8_t)186, (uint8_t)180, (uint8_t)223, (uint8_t)19, (uint8_t)234, (uint8_t)56, (uint8_t)195, (uint8_t)120, (uint8_t)84, (uint8_t)149, (uint8_t)122, (uint8_t)103, (uint8_t)187, (uint8_t)194, (uint8_t)78, (uint8_t)218, (uint8_t)34, (uint8_t)28, (uint8_t)23, (uint8_t)2, (uint8_t)201, (uint8_t)176, (uint8_t)196, (uint8_t)39, (uint8_t)244, (uint8_t)1, (uint8_t)55, (uint8_t)242, (uint8_t)143, (uint8_t)60, (uint8_t)79, (uint8_t)172, (uint8_t)232, (uint8_t)212, (uint8_t)68, (uint8_t)48, (uint8_t)72, (uint8_t)215, (uint8_t)33, (uint8_t)96, (uint8_t)105, (uint8_t)174, (uint8_t)130, (uint8_t)121, (uint8_t)75, (uint8_t)168, (uint8_t)36, (uint8_t)15, (uint8_t)124, (uint8_t)5, (uint8_t)195, (uint8_t)213, (uint8_t)95, (uint8_t)101, (uint8_t)130, (uint8_t)131, (uint8_t)244, (uint8_t)241, (uint8_t)174, (uint8_t)219, (uint8_t)92, (uint8_t)233, (uint8_t)73, (uint8_t)39, (uint8_t)182, (uint8_t)2, (uint8_t)112, (uint8_t)185, (uint8_t)41, (uint8_t)210, (uint8_t)124, (uint8_t)254, (uint8_t)114, (uint8_t)166, (uint8_t)61, (uint8_t)176, (uint8_t)161, (uint8_t)218, (uint8_t)241, (uint8_t)139, (uint8_t)201, (uint8_t)20, (uint8_t)25, (uint8_t)199, (uint8_t)14, (uint8_t)32, (uint8_t)211, (uint8_t)215, (uint8_t)151, (uint8_t)55, (uint8_t)43, (uint8_t)66, (uint8_t)236, (uint8_t)51, (uint8_t)180, (uint8_t)169, (uint8_t)117, (uint8_t)223, (uint8_t)27, (uint8_t)25, (uint8_t)198, (uint8_t)26, (uint8_t)244, (uint8_t)238, (uint8_t)24, (uint8_t)8, (uint8_t)73, (uint8_t)155, (uint8_t)110, (uint8_t)218, (uint8_t)37, (uint8_t)13, (uint8_t)57, (uint8_t)7, (uint8_t)74, (uint8_t)109, (uint8_t)192, (uint8_t)211, (uint8_t)96, (uint8_t)50, (uint8_t)105, (uint8_t)67, (uint8_t)124, (uint8_t)240, (uint8_t)129, (uint8_t)32, (uint8_t)28, (uint8_t)34, (uint8_t)226, (uint8_t)52, (uint8_t)119, (uint8_t)68, (uint8_t)63, (uint8_t)197, (uint8_t)91, (uint8_t)151, (uint8_t)191, (uint8_t)163, (uint8_t)66, (uint8_t)218, (uint8_t)20, (uint8_t)141, (uint8_t)128, (uint8_t)92, (uint8_t)160, (uint8_t)5, (uint8_t)60, (uint8_t)38, (uint8_t)114, (uint8_t)112, (uint8_t)115, (uint8_t)112, (uint8_t)207, (uint8_t)78, (uint8_t)246, (uint8_t)62, (uint8_t)100, (uint8_t)64, (uint8_t)194, (uint8_t)138, (uint8_t)127, (uint8_t)242, (uint8_t)71, (uint8_t)78, (uint8_t)246, (uint8_t)41, (uint8_t)114, (uint8_t)131, (uint8_t)115, (uint8_t)66, (uint8_t)128, (uint8_t)105, (uint8_t)144, (uint8_t)149, (uint8_t)76, (uint8_t)169, (uint8_t)30, (uint8_t)116, (uint8_t)78, (uint8_t)151, (uint8_t)151, (uint8_t)106, (uint8_t)72, (uint8_t)21, (uint8_t)122, (uint8_t)39, (uint8_t)188, (uint8_t)192, (uint8_t)204, (uint8_t)117, (uint8_t)3, (uint8_t)159, (uint8_t)133, (uint8_t)210, (uint8_t)246, (uint8_t)174, (uint8_t)225, (uint8_t)181, (uint8_t)31, (uint8_t)165, (uint8_t)189, (uint8_t)207, (uint8_t)28, (uint8_t)219, (uint8_t)42, (uint8_t)15, (uint8_t)73, (uint8_t)211, (uint8_t)44, (uint8_t)106, (uint8_t)134, (uint8_t)246, (uint8_t)209, (uint8_t)17, (uint8_t)27, (uint8_t)72, (uint8_t)187, (uint8_t)115, (uint8_t)249, (uint8_t)105, (uint8_t)34, (uint8_t)101, (uint8_t)30, (uint8_t)97, (uint8_t)134, (uint8_t)62, (uint8_t)171, (uint8_t)42, (uint8_t)82, (uint8_t)184, (uint8_t)243, (uint8_t)47, (uint8_t)215, (uint8_t)30, (uint8_t)13, (uint8_t)248, (uint8_t)212};
        p248_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MEMORY_VECT_249(), &PH);
    p249_address_SET((uint16_t)(uint16_t)2799, PH.base.pack) ;
    p249_ver_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    p249_type_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
    {
        int8_t  value [] =  {(int8_t)59, (int8_t) -27, (int8_t) -58, (int8_t) -71, (int8_t) -103, (int8_t) -99, (int8_t) -57, (int8_t)80, (int8_t) -74, (int8_t) -106, (int8_t) -25, (int8_t) -123, (int8_t) -63, (int8_t) -97, (int8_t)110, (int8_t)10, (int8_t)96, (int8_t)117, (int8_t)5, (int8_t)55, (int8_t)109, (int8_t)123, (int8_t)89, (int8_t) -68, (int8_t) -106, (int8_t) -38, (int8_t) -86, (int8_t)85, (int8_t)29, (int8_t)104, (int8_t)4, (int8_t) -98};
        p249_value_SET(&value, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEBUG_VECT_250(), &PH);
    {
        char16_t   name = "fiike";
        p250_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p250_time_usec_SET((uint64_t)4466033623783818194L, PH.base.pack) ;
    p250_x_SET((float) -2.9131532E38F, PH.base.pack) ;
    p250_y_SET((float)2.8536945E38F, PH.base.pack) ;
    p250_z_SET((float)7.816021E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
    p251_time_boot_ms_SET((uint32_t)3274757993L, PH.base.pack) ;
    {
        char16_t   name = "unruqoxqmg";
        p251_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p251_value_SET((float) -1.5329668E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_INT_252(), &PH);
    p252_time_boot_ms_SET((uint32_t)2914016413L, PH.base.pack) ;
    {
        char16_t   name = "tzmegkeh";
        p252_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p252_value_SET((int32_t) -2110221698, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_STATUSTEXT_253(), &PH);
    p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_EMERGENCY, PH.base.pack) ;
    {
        char16_t   text = "gcqyMUulhrfZzkegrjmrtnyotsosbgQnNaf";
        p253_text_SET(&text, 0,  sizeof(text), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_DEBUG_254(), &PH);
    p254_time_boot_ms_SET((uint32_t)3171985543L, PH.base.pack) ;
    p254_ind_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
    p254_value_SET((float)2.0943032E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SETUP_SIGNING_256(), &PH);
    p256_target_system_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
    p256_target_component_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
    {
        uint8_t  secret_key [] =  {(uint8_t)100, (uint8_t)218, (uint8_t)44, (uint8_t)66, (uint8_t)1, (uint8_t)17, (uint8_t)227, (uint8_t)249, (uint8_t)70, (uint8_t)192, (uint8_t)63, (uint8_t)125, (uint8_t)206, (uint8_t)214, (uint8_t)79, (uint8_t)25, (uint8_t)61, (uint8_t)90, (uint8_t)77, (uint8_t)191, (uint8_t)36, (uint8_t)218, (uint8_t)178, (uint8_t)162, (uint8_t)28, (uint8_t)135, (uint8_t)189, (uint8_t)35, (uint8_t)109, (uint8_t)141, (uint8_t)14, (uint8_t)227};
        p256_secret_key_SET(&secret_key, 0, &PH.base.pack) ;
    }
    p256_initial_timestamp_SET((uint64_t)6957795645511579264L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_BUTTON_CHANGE_257(), &PH);
    p257_time_boot_ms_SET((uint32_t)2892694849L, PH.base.pack) ;
    p257_last_change_ms_SET((uint32_t)4277331235L, PH.base.pack) ;
    p257_state_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PLAY_TUNE_258(), &PH);
    p258_target_system_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
    p258_target_component_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
    {
        char16_t   tune = "fwvyaqr";
        p258_tune_SET(&tune, 0,  sizeof(tune), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_INFORMATION_259(), &PH);
    p259_time_boot_ms_SET((uint32_t)2538759997L, PH.base.pack) ;
    {
        uint8_t  vendor_name [] =  {(uint8_t)63, (uint8_t)187, (uint8_t)18, (uint8_t)29, (uint8_t)91, (uint8_t)180, (uint8_t)131, (uint8_t)133, (uint8_t)121, (uint8_t)146, (uint8_t)138, (uint8_t)127, (uint8_t)182, (uint8_t)84, (uint8_t)225, (uint8_t)159, (uint8_t)247, (uint8_t)81, (uint8_t)93, (uint8_t)95, (uint8_t)2, (uint8_t)172, (uint8_t)250, (uint8_t)228, (uint8_t)66, (uint8_t)140, (uint8_t)16, (uint8_t)219, (uint8_t)176, (uint8_t)89, (uint8_t)142, (uint8_t)50};
        p259_vendor_name_SET(&vendor_name, 0, &PH.base.pack) ;
    }
    {
        uint8_t  model_name [] =  {(uint8_t)195, (uint8_t)93, (uint8_t)119, (uint8_t)41, (uint8_t)142, (uint8_t)36, (uint8_t)220, (uint8_t)174, (uint8_t)103, (uint8_t)54, (uint8_t)183, (uint8_t)68, (uint8_t)18, (uint8_t)106, (uint8_t)22, (uint8_t)255, (uint8_t)135, (uint8_t)21, (uint8_t)13, (uint8_t)217, (uint8_t)6, (uint8_t)34, (uint8_t)246, (uint8_t)26, (uint8_t)96, (uint8_t)3, (uint8_t)159, (uint8_t)69, (uint8_t)213, (uint8_t)148, (uint8_t)93, (uint8_t)67};
        p259_model_name_SET(&model_name, 0, &PH.base.pack) ;
    }
    p259_firmware_version_SET((uint32_t)1983841748L, PH.base.pack) ;
    p259_focal_length_SET((float) -2.3790005E38F, PH.base.pack) ;
    p259_sensor_size_h_SET((float) -8.509505E37F, PH.base.pack) ;
    p259_sensor_size_v_SET((float) -3.9679568E37F, PH.base.pack) ;
    p259_resolution_h_SET((uint16_t)(uint16_t)53216, PH.base.pack) ;
    p259_resolution_v_SET((uint16_t)(uint16_t)60278, PH.base.pack) ;
    p259_lens_id_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
    p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_MODES, PH.base.pack) ;
    p259_cam_definition_version_SET((uint16_t)(uint16_t)680, PH.base.pack) ;
    {
        char16_t   cam_definition_uri = "flNuwmqiZkn";
        p259_cam_definition_uri_SET(&cam_definition_uri, 0,  sizeof(cam_definition_uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_SETTINGS_260(), &PH);
    p260_time_boot_ms_SET((uint32_t)4104722354L, PH.base.pack) ;
    p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_IMAGE, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_STORAGE_INFORMATION_261(), &PH);
    p261_time_boot_ms_SET((uint32_t)2804020655L, PH.base.pack) ;
    p261_storage_id_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
    p261_storage_count_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
    p261_status_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
    p261_total_capacity_SET((float)2.3919644E38F, PH.base.pack) ;
    p261_used_capacity_SET((float)2.5099028E38F, PH.base.pack) ;
    p261_available_capacity_SET((float) -2.8907873E38F, PH.base.pack) ;
    p261_read_speed_SET((float) -1.4107405E38F, PH.base.pack) ;
    p261_write_speed_SET((float) -2.7735631E38F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
    p262_time_boot_ms_SET((uint32_t)3364756851L, PH.base.pack) ;
    p262_image_status_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
    p262_video_status_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
    p262_image_interval_SET((float)2.6620715E38F, PH.base.pack) ;
    p262_recording_time_ms_SET((uint32_t)1182560673L, PH.base.pack) ;
    p262_available_capacity_SET((float)7.3578883E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
    p263_time_boot_ms_SET((uint32_t)3094929052L, PH.base.pack) ;
    p263_time_utc_SET((uint64_t)8812983647704553792L, PH.base.pack) ;
    p263_camera_id_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
    p263_lat_SET((int32_t) -1810728773, PH.base.pack) ;
    p263_lon_SET((int32_t) -1715918504, PH.base.pack) ;
    p263_alt_SET((int32_t)319933654, PH.base.pack) ;
    p263_relative_alt_SET((int32_t)1487431259, PH.base.pack) ;
    {
        float  q [] =  {2.470945E38F, 3.8953192E37F, 2.095054E38F, -2.962796E38F};
        p263_q_SET(&q, 0, &PH.base.pack) ;
    }
    p263_image_index_SET((int32_t) -103598369, PH.base.pack) ;
    p263_capture_result_SET((int8_t)(int8_t)4, PH.base.pack) ;
    {
        char16_t   file_url = "bGtuTogiekFbhljiKxpcbwzsebxoqesaRnfiqfiSapFpVkkmgqqlhiSakvqcyEeupohflikulkjtxmnnebdVlonjfllqcyyelmjuyvwjfhbdffbpxIRuljuavmkzejgkxvlagiimAlzpvchtgycaieqzokdvv";
        p263_file_url_SET(&file_url, 0,  sizeof(file_url), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_FLIGHT_INFORMATION_264(), &PH);
    p264_time_boot_ms_SET((uint32_t)3584816826L, PH.base.pack) ;
    p264_arming_time_utc_SET((uint64_t)1750848821463060260L, PH.base.pack) ;
    p264_takeoff_time_utc_SET((uint64_t)8508700567798385175L, PH.base.pack) ;
    p264_flight_uuid_SET((uint64_t)9195698223599307698L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_MOUNT_ORIENTATION_265(), &PH);
    p265_time_boot_ms_SET((uint32_t)268928820L, PH.base.pack) ;
    p265_roll_SET((float) -3.323863E38F, PH.base.pack) ;
    p265_pitch_SET((float) -1.2932701E38F, PH.base.pack) ;
    p265_yaw_SET((float) -8.040165E37F, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_266(), &PH);
    p266_target_system_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    p266_target_component_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
    p266_sequence_SET((uint16_t)(uint16_t)26958, PH.base.pack) ;
    p266_length_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
    p266_first_message_offset_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)218, (uint8_t)131, (uint8_t)246, (uint8_t)189, (uint8_t)196, (uint8_t)31, (uint8_t)18, (uint8_t)74, (uint8_t)180, (uint8_t)95, (uint8_t)218, (uint8_t)197, (uint8_t)215, (uint8_t)219, (uint8_t)33, (uint8_t)237, (uint8_t)69, (uint8_t)147, (uint8_t)38, (uint8_t)77, (uint8_t)216, (uint8_t)126, (uint8_t)137, (uint8_t)207, (uint8_t)77, (uint8_t)206, (uint8_t)252, (uint8_t)20, (uint8_t)29, (uint8_t)143, (uint8_t)194, (uint8_t)67, (uint8_t)146, (uint8_t)148, (uint8_t)209, (uint8_t)51, (uint8_t)57, (uint8_t)102, (uint8_t)255, (uint8_t)85, (uint8_t)157, (uint8_t)148, (uint8_t)147, (uint8_t)36, (uint8_t)250, (uint8_t)11, (uint8_t)173, (uint8_t)163, (uint8_t)103, (uint8_t)98, (uint8_t)184, (uint8_t)16, (uint8_t)21, (uint8_t)124, (uint8_t)2, (uint8_t)112, (uint8_t)145, (uint8_t)128, (uint8_t)118, (uint8_t)15, (uint8_t)51, (uint8_t)225, (uint8_t)11, (uint8_t)176, (uint8_t)23, (uint8_t)39, (uint8_t)72, (uint8_t)118, (uint8_t)140, (uint8_t)251, (uint8_t)227, (uint8_t)195, (uint8_t)10, (uint8_t)132, (uint8_t)250, (uint8_t)156, (uint8_t)254, (uint8_t)168, (uint8_t)20, (uint8_t)60, (uint8_t)130, (uint8_t)202, (uint8_t)235, (uint8_t)79, (uint8_t)19, (uint8_t)116, (uint8_t)115, (uint8_t)183, (uint8_t)79, (uint8_t)234, (uint8_t)153, (uint8_t)160, (uint8_t)253, (uint8_t)92, (uint8_t)166, (uint8_t)63, (uint8_t)220, (uint8_t)227, (uint8_t)182, (uint8_t)2, (uint8_t)249, (uint8_t)132, (uint8_t)224, (uint8_t)209, (uint8_t)246, (uint8_t)191, (uint8_t)238, (uint8_t)171, (uint8_t)212, (uint8_t)7, (uint8_t)28, (uint8_t)47, (uint8_t)34, (uint8_t)194, (uint8_t)109, (uint8_t)105, (uint8_t)128, (uint8_t)121, (uint8_t)187, (uint8_t)135, (uint8_t)13, (uint8_t)69, (uint8_t)166, (uint8_t)53, (uint8_t)172, (uint8_t)35, (uint8_t)135, (uint8_t)161, (uint8_t)173, (uint8_t)21, (uint8_t)242, (uint8_t)232, (uint8_t)254, (uint8_t)185, (uint8_t)128, (uint8_t)118, (uint8_t)84, (uint8_t)235, (uint8_t)124, (uint8_t)164, (uint8_t)48, (uint8_t)79, (uint8_t)190, (uint8_t)67, (uint8_t)212, (uint8_t)127, (uint8_t)52, (uint8_t)41, (uint8_t)60, (uint8_t)0, (uint8_t)170, (uint8_t)55, (uint8_t)112, (uint8_t)89, (uint8_t)151, (uint8_t)52, (uint8_t)34, (uint8_t)38, (uint8_t)233, (uint8_t)174, (uint8_t)227, (uint8_t)96, (uint8_t)190, (uint8_t)202, (uint8_t)72, (uint8_t)207, (uint8_t)182, (uint8_t)123, (uint8_t)5, (uint8_t)150, (uint8_t)149, (uint8_t)186, (uint8_t)114, (uint8_t)129, (uint8_t)215, (uint8_t)26, (uint8_t)196, (uint8_t)101, (uint8_t)74, (uint8_t)18, (uint8_t)192, (uint8_t)165, (uint8_t)51, (uint8_t)172, (uint8_t)243, (uint8_t)73, (uint8_t)13, (uint8_t)20, (uint8_t)154, (uint8_t)230, (uint8_t)249, (uint8_t)151, (uint8_t)80, (uint8_t)244, (uint8_t)232, (uint8_t)128, (uint8_t)172, (uint8_t)56, (uint8_t)206, (uint8_t)151, (uint8_t)8, (uint8_t)57, (uint8_t)163, (uint8_t)185, (uint8_t)205, (uint8_t)141, (uint8_t)140, (uint8_t)70, (uint8_t)47, (uint8_t)181, (uint8_t)151, (uint8_t)226, (uint8_t)197, (uint8_t)112, (uint8_t)182, (uint8_t)91, (uint8_t)22, (uint8_t)30, (uint8_t)92, (uint8_t)124, (uint8_t)99, (uint8_t)96, (uint8_t)58, (uint8_t)84, (uint8_t)250, (uint8_t)11, (uint8_t)42, (uint8_t)232, (uint8_t)41, (uint8_t)172, (uint8_t)42, (uint8_t)85, (uint8_t)58, (uint8_t)156, (uint8_t)179, (uint8_t)161, (uint8_t)29, (uint8_t)90, (uint8_t)71, (uint8_t)162, (uint8_t)227, (uint8_t)26, (uint8_t)222, (uint8_t)221, (uint8_t)147, (uint8_t)142, (uint8_t)219, (uint8_t)79, (uint8_t)250};
        p266_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_ACKED_267(), &PH);
    p267_target_system_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
    p267_target_component_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
    p267_sequence_SET((uint16_t)(uint16_t)58911, PH.base.pack) ;
    p267_length_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
    p267_first_message_offset_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)243, (uint8_t)168, (uint8_t)74, (uint8_t)100, (uint8_t)157, (uint8_t)219, (uint8_t)23, (uint8_t)177, (uint8_t)177, (uint8_t)56, (uint8_t)230, (uint8_t)202, (uint8_t)216, (uint8_t)155, (uint8_t)90, (uint8_t)130, (uint8_t)137, (uint8_t)166, (uint8_t)154, (uint8_t)170, (uint8_t)251, (uint8_t)217, (uint8_t)121, (uint8_t)112, (uint8_t)45, (uint8_t)180, (uint8_t)73, (uint8_t)204, (uint8_t)16, (uint8_t)31, (uint8_t)255, (uint8_t)143, (uint8_t)105, (uint8_t)144, (uint8_t)148, (uint8_t)91, (uint8_t)195, (uint8_t)87, (uint8_t)40, (uint8_t)12, (uint8_t)158, (uint8_t)48, (uint8_t)190, (uint8_t)140, (uint8_t)144, (uint8_t)201, (uint8_t)67, (uint8_t)11, (uint8_t)27, (uint8_t)253, (uint8_t)227, (uint8_t)25, (uint8_t)75, (uint8_t)112, (uint8_t)147, (uint8_t)222, (uint8_t)248, (uint8_t)150, (uint8_t)70, (uint8_t)109, (uint8_t)147, (uint8_t)17, (uint8_t)201, (uint8_t)29, (uint8_t)58, (uint8_t)235, (uint8_t)176, (uint8_t)7, (uint8_t)149, (uint8_t)92, (uint8_t)74, (uint8_t)14, (uint8_t)170, (uint8_t)223, (uint8_t)32, (uint8_t)106, (uint8_t)197, (uint8_t)174, (uint8_t)7, (uint8_t)112, (uint8_t)253, (uint8_t)179, (uint8_t)99, (uint8_t)105, (uint8_t)211, (uint8_t)88, (uint8_t)149, (uint8_t)47, (uint8_t)4, (uint8_t)34, (uint8_t)71, (uint8_t)146, (uint8_t)243, (uint8_t)2, (uint8_t)188, (uint8_t)31, (uint8_t)7, (uint8_t)215, (uint8_t)139, (uint8_t)1, (uint8_t)68, (uint8_t)183, (uint8_t)143, (uint8_t)4, (uint8_t)4, (uint8_t)81, (uint8_t)222, (uint8_t)201, (uint8_t)235, (uint8_t)216, (uint8_t)223, (uint8_t)42, (uint8_t)66, (uint8_t)78, (uint8_t)36, (uint8_t)14, (uint8_t)79, (uint8_t)105, (uint8_t)11, (uint8_t)33, (uint8_t)163, (uint8_t)115, (uint8_t)107, (uint8_t)141, (uint8_t)112, (uint8_t)200, (uint8_t)229, (uint8_t)246, (uint8_t)251, (uint8_t)140, (uint8_t)98, (uint8_t)27, (uint8_t)0, (uint8_t)114, (uint8_t)246, (uint8_t)24, (uint8_t)60, (uint8_t)35, (uint8_t)63, (uint8_t)29, (uint8_t)38, (uint8_t)140, (uint8_t)73, (uint8_t)224, (uint8_t)136, (uint8_t)8, (uint8_t)84, (uint8_t)237, (uint8_t)148, (uint8_t)145, (uint8_t)135, (uint8_t)192, (uint8_t)14, (uint8_t)142, (uint8_t)139, (uint8_t)123, (uint8_t)187, (uint8_t)84, (uint8_t)51, (uint8_t)247, (uint8_t)241, (uint8_t)188, (uint8_t)204, (uint8_t)228, (uint8_t)90, (uint8_t)4, (uint8_t)156, (uint8_t)25, (uint8_t)199, (uint8_t)66, (uint8_t)27, (uint8_t)131, (uint8_t)110, (uint8_t)47, (uint8_t)145, (uint8_t)52, (uint8_t)58, (uint8_t)246, (uint8_t)176, (uint8_t)35, (uint8_t)83, (uint8_t)83, (uint8_t)236, (uint8_t)27, (uint8_t)231, (uint8_t)217, (uint8_t)51, (uint8_t)133, (uint8_t)40, (uint8_t)182, (uint8_t)108, (uint8_t)187, (uint8_t)57, (uint8_t)202, (uint8_t)145, (uint8_t)5, (uint8_t)30, (uint8_t)233, (uint8_t)35, (uint8_t)5, (uint8_t)220, (uint8_t)186, (uint8_t)11, (uint8_t)184, (uint8_t)57, (uint8_t)208, (uint8_t)253, (uint8_t)201, (uint8_t)78, (uint8_t)35, (uint8_t)126, (uint8_t)67, (uint8_t)180, (uint8_t)235, (uint8_t)244, (uint8_t)250, (uint8_t)161, (uint8_t)65, (uint8_t)22, (uint8_t)188, (uint8_t)101, (uint8_t)145, (uint8_t)62, (uint8_t)115, (uint8_t)128, (uint8_t)141, (uint8_t)123, (uint8_t)109, (uint8_t)226, (uint8_t)245, (uint8_t)66, (uint8_t)18, (uint8_t)189, (uint8_t)88, (uint8_t)121, (uint8_t)168, (uint8_t)67, (uint8_t)129, (uint8_t)191, (uint8_t)124, (uint8_t)223, (uint8_t)61, (uint8_t)110, (uint8_t)222, (uint8_t)189, (uint8_t)13, (uint8_t)138, (uint8_t)142, (uint8_t)188};
        p267_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_LOGGING_ACK_268(), &PH);
    p268_target_system_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
    p268_target_component_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
    p268_sequence_SET((uint16_t)(uint16_t)63878, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
    p269_camera_id_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
    p269_status_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
    p269_framerate_SET((float)3.8361085E37F, PH.base.pack) ;
    p269_resolution_h_SET((uint16_t)(uint16_t)53065, PH.base.pack) ;
    p269_resolution_v_SET((uint16_t)(uint16_t)2171, PH.base.pack) ;
    p269_bitrate_SET((uint32_t)1229084679L, PH.base.pack) ;
    p269_rotation_SET((uint16_t)(uint16_t)52357, PH.base.pack) ;
    {
        char16_t   uri = "doasanEkPjmzvVlnitjhxeHztgsgphilEisxPsApsegburriduWrxqkogrppafzjnoflhitvkfzTvsscBsk";
        p269_uri_SET(&uri, 0,  sizeof(uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
    p270_target_system_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
    p270_target_component_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
    p270_camera_id_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
    p270_framerate_SET((float)3.1020931E38F, PH.base.pack) ;
    p270_resolution_h_SET((uint16_t)(uint16_t)17693, PH.base.pack) ;
    p270_resolution_v_SET((uint16_t)(uint16_t)32176, PH.base.pack) ;
    p270_bitrate_SET((uint32_t)1124354096L, PH.base.pack) ;
    p270_rotation_SET((uint16_t)(uint16_t)19293, PH.base.pack) ;
    {
        char16_t   uri = "kdxfpdinQtuhqtKuefLswtabfbpqejmzbtoblKizatQkbvzGpsxobxqjwpbxPmDiazkzursewgimanzhfcbCwxyzcknKcEAuytvxvqocbxzqWHYpdgmrsitbssgUuqzjsnzorjbxCdfebtenaegswujblwhieumczpqlhjbdqyydyesnwafbOygcxhpvwgavyvFmfdrlgkcddnteycdox";
        p270_uri_SET(&uri, 0,  sizeof(uri), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_WIFI_CONFIG_AP_299(), &PH);
    {
        char16_t   ssid = "rP";
        p299_ssid_SET(&ssid, 0,  sizeof(ssid), &PH) ;
    }
    {
        char16_t   password = "fpuagVoswzcyjfzlyve";
        p299_password_SET(&password, 0,  sizeof(password), &PH) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PROTOCOL_VERSION_300(), &PH);
    p300_version_SET((uint16_t)(uint16_t)24820, PH.base.pack) ;
    p300_min_version_SET((uint16_t)(uint16_t)2987, PH.base.pack) ;
    p300_max_version_SET((uint16_t)(uint16_t)18895, PH.base.pack) ;
    {
        uint8_t  spec_version_hash [] =  {(uint8_t)182, (uint8_t)3, (uint8_t)175, (uint8_t)14, (uint8_t)195, (uint8_t)243, (uint8_t)46, (uint8_t)166};
        p300_spec_version_hash_SET(&spec_version_hash, 0, &PH.base.pack) ;
    }
    {
        uint8_t  library_version_hash [] =  {(uint8_t)55, (uint8_t)222, (uint8_t)49, (uint8_t)167, (uint8_t)210, (uint8_t)33, (uint8_t)100, (uint8_t)85};
        p300_library_version_hash_SET(&library_version_hash, 0, &PH.base.pack) ;
    }
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_STATUS_310(), &PH);
    p310_time_usec_SET((uint64_t)7937613305272134571L, PH.base.pack) ;
    p310_uptime_sec_SET((uint32_t)923200694L, PH.base.pack) ;
    p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR, PH.base.pack) ;
    p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OFFLINE, PH.base.pack) ;
    p310_sub_mode_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
    p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)65330, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_INFO_311(), &PH);
    p311_time_usec_SET((uint64_t)9024922718606888401L, PH.base.pack) ;
    p311_uptime_sec_SET((uint32_t)3139924807L, PH.base.pack) ;
    {
        char16_t   name = "obzntvtcxuxpnhhoyhueukhbKavrpassybiakjBfojZrlzvijsdosdmkozgttZyqpggjgwymvyxiy";
        p311_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p311_hw_version_major_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
    p311_hw_version_minor_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
    {
        uint8_t  hw_unique_id [] =  {(uint8_t)209, (uint8_t)13, (uint8_t)116, (uint8_t)230, (uint8_t)123, (uint8_t)105, (uint8_t)42, (uint8_t)196, (uint8_t)120, (uint8_t)73, (uint8_t)250, (uint8_t)74, (uint8_t)242, (uint8_t)194, (uint8_t)124, (uint8_t)161};
        p311_hw_unique_id_SET(&hw_unique_id, 0, &PH.base.pack) ;
    }
    p311_sw_version_major_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
    p311_sw_version_minor_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
    p311_sw_vcs_commit_SET((uint32_t)1942516826L, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
    p320_target_system_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
    p320_target_component_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
    {
        char16_t   param_id = "dHvkahbeioopvlT";
        p320_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p320_param_index_SET((int16_t)(int16_t) -5733, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
    p321_target_system_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
    p321_target_component_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_VALUE_322(), &PH);
    {
        char16_t   param_id = "hAdkNc";
        p322_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "bsTxcsidpyozgdKfvbmhsxxLarccesnpnxqrpckpyjyxOmqucDzadiesysmzmspusslxkqpwlyxhdJfcqfyyb";
        p322_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32, PH.base.pack) ;
    p322_param_count_SET((uint16_t)(uint16_t)22918, PH.base.pack) ;
    p322_param_index_SET((uint16_t)(uint16_t)48975, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_SET_323(), &PH);
    p323_target_system_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
    p323_target_component_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
    {
        char16_t   param_id = "ogYf";
        p323_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "aMlywjzzyqazypirelshhsghppzezxsunrsthkiehmpwckivvldtyzhvrmkyaeolusemncmpaogzhejirgOjkionjRcPyakzyxlieybnzxmffUfzvxpfkwshprmqvefi";
        p323_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT8, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_PARAM_EXT_ACK_324(), &PH);
    {
        char16_t   param_id = "uqptCcilYbrwhM";
        p324_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    {
        char16_t   param_value = "wosebqsqjixenodgtzfxaAFNGunjhcjrotwpfxue";
        p324_param_value_SET(&param_value, 0,  sizeof(param_value), &PH) ;
    }
    p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8, PH.base.pack) ;
    p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_VALUE_UNSUPPORTED, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    setPack(c_LoopBackDemoChannel_new_OBSTACLE_DISTANCE_330(), &PH);
    p330_time_usec_SET((uint64_t)6886078031263776739L, PH.base.pack) ;
    p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND, PH.base.pack) ;
    {
        uint16_t  distances [] =  {(uint16_t)64521, (uint16_t)23321, (uint16_t)56780, (uint16_t)6927, (uint16_t)17659, (uint16_t)53721, (uint16_t)260, (uint16_t)43836, (uint16_t)56008, (uint16_t)28564, (uint16_t)31151, (uint16_t)34043, (uint16_t)16457, (uint16_t)25964, (uint16_t)23241, (uint16_t)52641, (uint16_t)55388, (uint16_t)12783, (uint16_t)57321, (uint16_t)21648, (uint16_t)27042, (uint16_t)63816, (uint16_t)37733, (uint16_t)9180, (uint16_t)52070, (uint16_t)40995, (uint16_t)57587, (uint16_t)5143, (uint16_t)5175, (uint16_t)26268, (uint16_t)25895, (uint16_t)37908, (uint16_t)21097, (uint16_t)12224, (uint16_t)26199, (uint16_t)53919, (uint16_t)62329, (uint16_t)54863, (uint16_t)27966, (uint16_t)5767, (uint16_t)64380, (uint16_t)4151, (uint16_t)3099, (uint16_t)58208, (uint16_t)58127, (uint16_t)14302, (uint16_t)30714, (uint16_t)56942, (uint16_t)50101, (uint16_t)65522, (uint16_t)51788, (uint16_t)64672, (uint16_t)14510, (uint16_t)50379, (uint16_t)1582, (uint16_t)46872, (uint16_t)51607, (uint16_t)65455, (uint16_t)25063, (uint16_t)29948, (uint16_t)18989, (uint16_t)57276, (uint16_t)46088, (uint16_t)1430, (uint16_t)39004, (uint16_t)25571, (uint16_t)27392, (uint16_t)1237, (uint16_t)33283, (uint16_t)41779, (uint16_t)9995, (uint16_t)2491};
        p330_distances_SET(&distances, 0, &PH.base.pack) ;
    }
    p330_increment_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    p330_min_distance_SET((uint16_t)(uint16_t)4004, PH.base.pack) ;
    p330_max_distance_SET((uint16_t)(uint16_t)32346, PH.base.pack) ;
    c_LoopBackDemoChannel_send(PH.base.pack);//put pack to the c_LoopBackDemoChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}
