
#include "MicroAirVehicle.h"
void failure(Channel * ch, int32_t id, int32_t arg) {}
void c_CommunicationChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p89_time_boot_ms_GET(pack);
    float  x = p89_x_GET(pack);
    float  y = p89_y_GET(pack);
    float  z = p89_z_GET(pack);
    float  roll = p89_roll_GET(pack);
    float  pitch = p89_pitch_GET(pack);
    float  yaw = p89_yaw_GET(pack);
}
void c_CommunicationChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p93_time_usec_GET(pack);
    float*  controls = p93_controls_GET_(pack);
//process data in controls
    free(controls);//never forget to dispose
    e_MAV_MODE  mode = p93_mode_GET(pack);
    uint64_t  flags = p93_flags_GET(pack);
}
void c_CommunicationChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  usec = p101_usec_GET(pack);
    float  x = p101_x_GET(pack);
    float  y = p101_y_GET(pack);
    float  z = p101_z_GET(pack);
    float  roll = p101_roll_GET(pack);
    float  pitch = p101_pitch_GET(pack);
    float  yaw = p101_yaw_GET(pack);
}
void c_CommunicationChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  usec = p102_usec_GET(pack);
    float  x = p102_x_GET(pack);
    float  y = p102_y_GET(pack);
    float  z = p102_z_GET(pack);
    float  roll = p102_roll_GET(pack);
    float  pitch = p102_pitch_GET(pack);
    float  yaw = p102_yaw_GET(pack);
}
void c_CommunicationChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  usec = p103_usec_GET(pack);
    float  x = p103_x_GET(pack);
    float  y = p103_y_GET(pack);
    float  z = p103_z_GET(pack);
}
void c_CommunicationChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  usec = p104_usec_GET(pack);
    float  x = p104_x_GET(pack);
    float  y = p104_y_GET(pack);
    float  z = p104_z_GET(pack);
    float  roll = p104_roll_GET(pack);
    float  pitch = p104_pitch_GET(pack);
    float  yaw = p104_yaw_GET(pack);
}
void c_CommunicationChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  rssi = p109_rssi_GET(pack);
    uint8_t  remrssi = p109_remrssi_GET(pack);
    uint8_t  txbuf = p109_txbuf_GET(pack);
    uint8_t  noise = p109_noise_GET(pack);
    uint8_t  remnoise = p109_remnoise_GET(pack);
    uint16_t  rxerrors = p109_rxerrors_GET(pack);
    uint16_t  fixed_ = p109_fixed__GET(pack);
}
void c_CommunicationChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_network = p110_target_network_GET(pack);
    uint8_t  target_system = p110_target_system_GET(pack);
    uint8_t  target_component = p110_target_component_GET(pack);
    uint8_t*  payload = p110_payload_GET_(pack);
//process data in payload
    free(payload);//never forget to dispose
}
void c_CommunicationChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    int64_t  tc1 = p111_tc1_GET(pack);
    int64_t  ts1 = p111_ts1_GET(pack);
}
void c_CommunicationChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p112_time_usec_GET(pack);
    uint32_t  seq = p112_seq_GET(pack);
}
void c_CommunicationChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p117_target_system_GET(pack);
    uint8_t  target_component = p117_target_component_GET(pack);
    uint16_t  start = p117_start_GET(pack);
    uint16_t  end = p117_end_GET(pack);
}
void c_CommunicationChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  id = p118_id_GET(pack);
    uint16_t  num_logs = p118_num_logs_GET(pack);
    uint16_t  last_log_num = p118_last_log_num_GET(pack);
    uint32_t  time_utc = p118_time_utc_GET(pack);
    uint32_t  size = p118_size_GET(pack);
}
void c_CommunicationChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p119_target_system_GET(pack);
    uint8_t  target_component = p119_target_component_GET(pack);
    uint16_t  id = p119_id_GET(pack);
    uint32_t  ofs = p119_ofs_GET(pack);
    uint32_t  count = p119_count_GET(pack);
}
void c_CommunicationChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  id = p120_id_GET(pack);
    uint32_t  ofs = p120_ofs_GET(pack);
    uint8_t  count = p120_count_GET(pack);
    uint8_t*  data_ = p120_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p121_target_system_GET(pack);
    uint8_t  target_component = p121_target_component_GET(pack);
}
void c_CommunicationChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p122_target_system_GET(pack);
    uint8_t  target_component = p122_target_component_GET(pack);
}
void c_CommunicationChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p123_target_system_GET(pack);
    uint8_t  target_component = p123_target_component_GET(pack);
    uint8_t  len = p123_len_GET(pack);
    uint8_t*  data_ = p123_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  Vcc = p125_Vcc_GET(pack);
    uint16_t  Vservo = p125_Vservo_GET(pack);
    e_MAV_POWER_STATUS  flags = p125_flags_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  type = p130_type_GET(pack);
    uint32_t  size = p130_size_GET(pack);
    uint16_t  width = p130_width_GET(pack);
    uint16_t  height = p130_height_GET(pack);
    uint16_t  packets = p130_packets_GET(pack);
    uint8_t  payload = p130_payload_GET(pack);
    uint8_t  jpg_quality = p130_jpg_quality_GET(pack);
}
void c_CommunicationChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  seqnr = p131_seqnr_GET(pack);
    uint8_t*  data_ = p131_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    int32_t  lat = p133_lat_GET(pack);
    int32_t  lon = p133_lon_GET(pack);
    uint16_t  grid_spacing = p133_grid_spacing_GET(pack);
    uint64_t  mask = p133_mask_GET(pack);
}
void c_CommunicationChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    int32_t  lat = p134_lat_GET(pack);
    int32_t  lon = p134_lon_GET(pack);
    uint16_t  grid_spacing = p134_grid_spacing_GET(pack);
    uint8_t  gridbit = p134_gridbit_GET(pack);
    int16_t*  data_ = p134_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    int32_t  lat = p135_lat_GET(pack);
    int32_t  lon = p135_lon_GET(pack);
}
void c_CommunicationChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    int32_t  lat = p136_lat_GET(pack);
    int32_t  lon = p136_lon_GET(pack);
    uint16_t  spacing = p136_spacing_GET(pack);
    float  terrain_height = p136_terrain_height_GET(pack);
    float  current_height = p136_current_height_GET(pack);
    uint16_t  pending = p136_pending_GET(pack);
    uint16_t  loaded = p136_loaded_GET(pack);
}
void c_CommunicationChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p137_time_boot_ms_GET(pack);
    float  press_abs = p137_press_abs_GET(pack);
    float  press_diff = p137_press_diff_GET(pack);
    int16_t  temperature = p137_temperature_GET(pack);
}
void c_CommunicationChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p138_time_usec_GET(pack);
    float*  q = p138_q_GET_(pack);
//process data in q
    free(q);//never forget to dispose
    float  x = p138_x_GET(pack);
    float  y = p138_y_GET(pack);
    float  z = p138_z_GET(pack);
}
void c_CommunicationChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p139_time_usec_GET(pack);
    uint8_t  group_mlx = p139_group_mlx_GET(pack);
    uint8_t  target_system = p139_target_system_GET(pack);
    uint8_t  target_component = p139_target_component_GET(pack);
    float*  controls = p139_controls_GET_(pack);
//process data in controls
    free(controls);//never forget to dispose
}
void c_CommunicationChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p140_time_usec_GET(pack);
    uint8_t  group_mlx = p140_group_mlx_GET(pack);
    float*  controls = p140_controls_GET_(pack);
//process data in controls
    free(controls);//never forget to dispose
}
void c_CommunicationChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p141_time_usec_GET(pack);
    float  altitude_monotonic = p141_altitude_monotonic_GET(pack);
    float  altitude_amsl = p141_altitude_amsl_GET(pack);
    float  altitude_local = p141_altitude_local_GET(pack);
    float  altitude_relative = p141_altitude_relative_GET(pack);
    float  altitude_terrain = p141_altitude_terrain_GET(pack);
    float  bottom_clearance = p141_bottom_clearance_GET(pack);
}
void c_CommunicationChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p143_time_boot_ms_GET(pack);
    float  press_abs = p143_press_abs_GET(pack);
    float  press_diff = p143_press_diff_GET(pack);
    int16_t  temperature = p143_temperature_GET(pack);
}
void c_CommunicationChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_CPU_LOAD_170(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  sensLoad = p170_sensLoad_GET(pack);
    uint8_t  ctrlLoad = p170_ctrlLoad_GET(pack);
    uint16_t  batVolt = p170_batVolt_GET(pack);
}
void c_CommunicationChannel_on_SENSOR_BIAS_172(Bounds_Inside * ph, Pack * pack)
{
    float  axBias = p172_axBias_GET(pack);
    float  ayBias = p172_ayBias_GET(pack);
    float  azBias = p172_azBias_GET(pack);
    float  gxBias = p172_gxBias_GET(pack);
    float  gyBias = p172_gyBias_GET(pack);
    float  gzBias = p172_gzBias_GET(pack);
}
void c_CommunicationChannel_on_DIAGNOSTIC_173(Bounds_Inside * ph, Pack * pack)
{
    float  diagFl1 = p173_diagFl1_GET(pack);
    float  diagFl2 = p173_diagFl2_GET(pack);
    float  diagFl3 = p173_diagFl3_GET(pack);
    int16_t  diagSh1 = p173_diagSh1_GET(pack);
    int16_t  diagSh2 = p173_diagSh2_GET(pack);
    int16_t  diagSh3 = p173_diagSh3_GET(pack);
}
void c_CommunicationChannel_on_SLUGS_NAVIGATION_176(Bounds_Inside * ph, Pack * pack)
{
    float  u_m = p176_u_m_GET(pack);
    float  phi_c = p176_phi_c_GET(pack);
    float  theta_c = p176_theta_c_GET(pack);
    float  psiDot_c = p176_psiDot_c_GET(pack);
    float  ay_body = p176_ay_body_GET(pack);
    float  totalDist = p176_totalDist_GET(pack);
    float  dist2Go = p176_dist2Go_GET(pack);
    uint8_t  fromWP = p176_fromWP_GET(pack);
    uint8_t  toWP = p176_toWP_GET(pack);
    uint16_t  h_c = p176_h_c_GET(pack);
}
void c_CommunicationChannel_on_DATA_LOG_177(Bounds_Inside * ph, Pack * pack)
{
    float  fl_1 = p177_fl_1_GET(pack);
    float  fl_2 = p177_fl_2_GET(pack);
    float  fl_3 = p177_fl_3_GET(pack);
    float  fl_4 = p177_fl_4_GET(pack);
    float  fl_5 = p177_fl_5_GET(pack);
    float  fl_6 = p177_fl_6_GET(pack);
}
void c_CommunicationChannel_on_GPS_DATE_TIME_179(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  year = p179_year_GET(pack);
    uint8_t  month = p179_month_GET(pack);
    uint8_t  day = p179_day_GET(pack);
    uint8_t  hour = p179_hour_GET(pack);
    uint8_t  min = p179_min_GET(pack);
    uint8_t  sec = p179_sec_GET(pack);
    uint8_t  clockStat = p179_clockStat_GET(pack);
    uint8_t  visSat = p179_visSat_GET(pack);
    uint8_t  useSat = p179_useSat_GET(pack);
    uint8_t  GppGl = p179_GppGl_GET(pack);
    uint8_t  sigUsedMask = p179_sigUsedMask_GET(pack);
    uint8_t  percentUsed = p179_percentUsed_GET(pack);
}
void c_CommunicationChannel_on_MID_LVL_CMDS_180(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target = p180_target_GET(pack);
    float  hCommand = p180_hCommand_GET(pack);
    float  uCommand = p180_uCommand_GET(pack);
    float  rCommand = p180_rCommand_GET(pack);
}
void c_CommunicationChannel_on_CTRL_SRFC_PT_181(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target = p181_target_GET(pack);
    uint16_t  bitfieldPt = p181_bitfieldPt_GET(pack);
}
void c_CommunicationChannel_on_SLUGS_CAMERA_ORDER_184(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target = p184_target_GET(pack);
    int8_t  pan = p184_pan_GET(pack);
    int8_t  tilt = p184_tilt_GET(pack);
    int8_t  zoom = p184_zoom_GET(pack);
    int8_t  moveHome = p184_moveHome_GET(pack);
}
void c_CommunicationChannel_on_CONTROL_SURFACE_185(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target = p185_target_GET(pack);
    uint8_t  idSurface = p185_idSurface_GET(pack);
    float  mControl = p185_mControl_GET(pack);
    float  bControl = p185_bControl_GET(pack);
}
void c_CommunicationChannel_on_SLUGS_MOBILE_LOCATION_186(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target = p186_target_GET(pack);
    float  latitude = p186_latitude_GET(pack);
    float  longitude = p186_longitude_GET(pack);
}
void c_CommunicationChannel_on_SLUGS_CONFIGURATION_CAMERA_188(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target = p188_target_GET(pack);
    uint8_t  idOrder = p188_idOrder_GET(pack);
    uint8_t  order = p188_order_GET(pack);
}
void c_CommunicationChannel_on_ISR_LOCATION_189(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target = p189_target_GET(pack);
    float  latitude = p189_latitude_GET(pack);
    float  longitude = p189_longitude_GET(pack);
    float  height = p189_height_GET(pack);
    uint8_t  option1 = p189_option1_GET(pack);
    uint8_t  option2 = p189_option2_GET(pack);
    uint8_t  option3 = p189_option3_GET(pack);
}
void c_CommunicationChannel_on_VOLT_SENSOR_191(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  r2Type = p191_r2Type_GET(pack);
    uint16_t  voltage = p191_voltage_GET(pack);
    uint16_t  reading2 = p191_reading2_GET(pack);
}
void c_CommunicationChannel_on_PTZ_STATUS_192(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  zoom = p192_zoom_GET(pack);
    int16_t  pan = p192_pan_GET(pack);
    int16_t  tilt = p192_tilt_GET(pack);
}
void c_CommunicationChannel_on_UAV_STATUS_193(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target = p193_target_GET(pack);
    float  latitude = p193_latitude_GET(pack);
    float  longitude = p193_longitude_GET(pack);
    float  altitude = p193_altitude_GET(pack);
    float  speed = p193_speed_GET(pack);
    float  course = p193_course_GET(pack);
}
void c_CommunicationChannel_on_STATUS_GPS_194(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  csFails = p194_csFails_GET(pack);
    uint8_t  gpsQuality = p194_gpsQuality_GET(pack);
    uint8_t  msgsType = p194_msgsType_GET(pack);
    uint8_t  posStatus = p194_posStatus_GET(pack);
    float  magVar = p194_magVar_GET(pack);
    int8_t  magDir = p194_magDir_GET(pack);
    uint8_t  modeInd = p194_modeInd_GET(pack);
}
void c_CommunicationChannel_on_NOVATEL_DIAG_195(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  timeStatus = p195_timeStatus_GET(pack);
    uint32_t  receiverStatus = p195_receiverStatus_GET(pack);
    uint8_t  solStatus = p195_solStatus_GET(pack);
    uint8_t  posType = p195_posType_GET(pack);
    uint8_t  velType = p195_velType_GET(pack);
    float  posSolAge = p195_posSolAge_GET(pack);
    uint16_t  csFails = p195_csFails_GET(pack);
}
void c_CommunicationChannel_on_SENSOR_DIAG_196(Bounds_Inside * ph, Pack * pack)
{
    float  float1 = p196_float1_GET(pack);
    float  float2 = p196_float2_GET(pack);
    int16_t  int1 = p196_int1_GET(pack);
    int8_t  char1 = p196_char1_GET(pack);
}
void c_CommunicationChannel_on_BOOT_197(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  version = p197_version_GET(pack);
}
void c_CommunicationChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  flags = p233_flags_GET(pack);
    uint8_t  len = p233_len_GET(pack);
    uint8_t*  data_ = p233_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p241_time_usec_GET(pack);
    float  vibration_x = p241_vibration_x_GET(pack);
    float  vibration_y = p241_vibration_y_GET(pack);
    float  vibration_z = p241_vibration_z_GET(pack);
    uint32_t  clipping_0 = p241_clipping_0_GET(pack);
    uint32_t  clipping_1 = p241_clipping_1_GET(pack);
    uint32_t  clipping_2 = p241_clipping_2_GET(pack);
}
void c_CommunicationChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  message_id = p244_message_id_GET(pack);
    int32_t  interval_us = p244_interval_us_GET(pack);
}
void c_CommunicationChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_VTOL_STATE  vtol_state = p245_vtol_state_GET(pack);
    e_MAV_LANDED_STATE  landed_state = p245_landed_state_GET(pack);
}
void c_CommunicationChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_COLLISION_SRC  src_ = p247_src__GET(pack);
    uint32_t  id = p247_id_GET(pack);
    e_MAV_COLLISION_ACTION  action = p247_action_GET(pack);
    e_MAV_COLLISION_THREAT_LEVEL  threat_level = p247_threat_level_GET(pack);
    float  time_to_minimum_delta = p247_time_to_minimum_delta_GET(pack);
    float  altitude_minimum_delta = p247_altitude_minimum_delta_GET(pack);
    float  horizontal_minimum_delta = p247_horizontal_minimum_delta_GET(pack);
}
void c_CommunicationChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_network = p248_target_network_GET(pack);
    uint8_t  target_system = p248_target_system_GET(pack);
    uint8_t  target_component = p248_target_component_GET(pack);
    uint16_t  message_type = p248_message_type_GET(pack);
    uint8_t*  payload = p248_payload_GET_(pack);
//process data in payload
    free(payload);//never forget to dispose
}
void c_CommunicationChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  address = p249_address_GET(pack);
    uint8_t  ver = p249_ver_GET(pack);
    uint8_t  type = p249_type_GET(pack);
    int8_t*  value = p249_value_GET_(pack);
//process data in value
    free(value);//never forget to dispose
}
void c_CommunicationChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  name = p250_name_TRY_(ph);
    uint64_t  time_usec = p250_time_usec_GET(pack);
    float  x = p250_x_GET(pack);
    float  y = p250_y_GET(pack);
    float  z = p250_z_GET(pack);
}
void c_CommunicationChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p251_time_boot_ms_GET(pack);
    char16_t *  name = p251_name_TRY_(ph);
    float  value = p251_value_GET(pack);
}
void c_CommunicationChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p252_time_boot_ms_GET(pack);
    char16_t *  name = p252_name_TRY_(ph);
    int32_t  value = p252_value_GET(pack);
}
void c_CommunicationChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_SEVERITY  severity = p253_severity_GET(pack);
    char16_t *  text = p253_text_TRY_(ph);
}
void c_CommunicationChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p254_time_boot_ms_GET(pack);
    uint8_t  ind = p254_ind_GET(pack);
    float  value = p254_value_GET(pack);
}
void c_CommunicationChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p256_target_system_GET(pack);
    uint8_t  target_component = p256_target_component_GET(pack);
    uint8_t*  secret_key = p256_secret_key_GET_(pack);
//process data in secret_key
    free(secret_key);//never forget to dispose
    uint64_t  initial_timestamp = p256_initial_timestamp_GET(pack);
}
void c_CommunicationChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p257_time_boot_ms_GET(pack);
    uint32_t  last_change_ms = p257_last_change_ms_GET(pack);
    uint8_t  state = p257_state_GET(pack);
}
void c_CommunicationChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p258_target_system_GET(pack);
    uint8_t  target_component = p258_target_component_GET(pack);
    char16_t *  tune = p258_tune_TRY_(ph);
}
void c_CommunicationChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p260_time_boot_ms_GET(pack);
    e_CAMERA_MODE  mode_id = p260_mode_id_GET(pack);
}
void c_CommunicationChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p262_time_boot_ms_GET(pack);
    uint8_t  image_status = p262_image_status_GET(pack);
    uint8_t  video_status = p262_video_status_GET(pack);
    float  image_interval = p262_image_interval_GET(pack);
    uint32_t  recording_time_ms = p262_recording_time_ms_GET(pack);
    float  available_capacity = p262_available_capacity_GET(pack);
}
void c_CommunicationChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p264_time_boot_ms_GET(pack);
    uint64_t  arming_time_utc = p264_arming_time_utc_GET(pack);
    uint64_t  takeoff_time_utc = p264_takeoff_time_utc_GET(pack);
    uint64_t  flight_uuid = p264_flight_uuid_GET(pack);
}
void c_CommunicationChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p265_time_boot_ms_GET(pack);
    float  roll = p265_roll_GET(pack);
    float  pitch = p265_pitch_GET(pack);
    float  yaw = p265_yaw_GET(pack);
}
void c_CommunicationChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p268_target_system_GET(pack);
    uint8_t  target_component = p268_target_component_GET(pack);
    uint16_t  sequence = p268_sequence_GET(pack);
}
void c_CommunicationChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  ssid = p299_ssid_TRY_(ph);
    char16_t *  password = p299_password_TRY_(ph);
}
void c_CommunicationChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p310_time_usec_GET(pack);
    uint32_t  uptime_sec = p310_uptime_sec_GET(pack);
    e_UAVCAN_NODE_HEALTH  health = p310_health_GET(pack);
    e_UAVCAN_NODE_MODE  mode = p310_mode_GET(pack);
    uint8_t  sub_mode = p310_sub_mode_GET(pack);
    uint16_t  vendor_specific_status_code = p310_vendor_specific_status_code_GET(pack);
}
void c_CommunicationChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p320_target_system_GET(pack);
    uint8_t  target_component = p320_target_component_GET(pack);
    char16_t *  param_id = p320_param_id_TRY_(ph);
    int16_t  param_index = p320_param_index_GET(pack);
}
void c_CommunicationChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p321_target_system_GET(pack);
    uint8_t  target_component = p321_target_component_GET(pack);
}
void c_CommunicationChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  param_id = p322_param_id_TRY_(ph);
    char16_t *  param_value = p322_param_value_TRY_(ph);
    e_MAV_PARAM_EXT_TYPE  param_type = p322_param_type_GET(pack);
    uint16_t  param_count = p322_param_count_GET(pack);
    uint16_t  param_index = p322_param_index_GET(pack);
}
void c_CommunicationChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p323_target_system_GET(pack);
    uint8_t  target_component = p323_target_component_GET(pack);
    char16_t *  param_id = p323_param_id_TRY_(ph);
    char16_t *  param_value = p323_param_value_TRY_(ph);
    e_MAV_PARAM_EXT_TYPE  param_type = p323_param_type_GET(pack);
}
void c_CommunicationChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    char16_t *  param_id = p324_param_id_TRY_(ph);
    char16_t *  param_value = p324_param_value_TRY_(ph);
    e_MAV_PARAM_EXT_TYPE  param_type = p324_param_type_GET(pack);
    e_PARAM_ACK  param_result = p324_param_result_GET(pack);
}
void c_CommunicationChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
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
    setPack(c_CommunicationChannel_new_HEARTBEAT_0(), &PH);
    p0_type_SET(e_MAV_TYPE_MAV_TYPE_FIXED_WING, PH.base.pack) ;
    p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_AEROB, PH.base.pack) ;
    p0_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED |
                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED |
                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED), PH.base.pack) ;
    p0_custom_mode_SET((uint32_t)2449308252L, PH.base.pack) ;
    p0_system_status_SET(e_MAV_STATE_MAV_STATE_BOOT, PH.base.pack) ;
    p0_mavlink_version_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
    p1_onboard_control_sensors_present_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW), PH.base.pack) ;
    p1_onboard_control_sensors_enabled_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW), PH.base.pack) ;
    p1_onboard_control_sensors_health_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
    p1_load_SET((uint16_t)(uint16_t)34011, PH.base.pack) ;
    p1_voltage_battery_SET((uint16_t)(uint16_t)18290, PH.base.pack) ;
    p1_current_battery_SET((int16_t)(int16_t)21011, PH.base.pack) ;
    p1_battery_remaining_SET((int8_t)(int8_t)95, PH.base.pack) ;
    p1_drop_rate_comm_SET((uint16_t)(uint16_t)34734, PH.base.pack) ;
    p1_errors_comm_SET((uint16_t)(uint16_t)51110, PH.base.pack) ;
    p1_errors_count1_SET((uint16_t)(uint16_t)24039, PH.base.pack) ;
    p1_errors_count2_SET((uint16_t)(uint16_t)60100, PH.base.pack) ;
    p1_errors_count3_SET((uint16_t)(uint16_t)48424, PH.base.pack) ;
    p1_errors_count4_SET((uint16_t)(uint16_t)47320, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
    p2_time_unix_usec_SET((uint64_t)9129120950520418963L, PH.base.pack) ;
    p2_time_boot_ms_SET((uint32_t)3001726722L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
    p3_time_boot_ms_SET((uint32_t)2824307842L, PH.base.pack) ;
    p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
    p3_type_mask_SET((uint16_t)(uint16_t)38289, PH.base.pack) ;
    p3_x_SET((float)1.8022535E38F, PH.base.pack) ;
    p3_y_SET((float)2.71195E38F, PH.base.pack) ;
    p3_z_SET((float)2.0195898E37F, PH.base.pack) ;
    p3_vx_SET((float)3.3736937E38F, PH.base.pack) ;
    p3_vy_SET((float)2.3626683E38F, PH.base.pack) ;
    p3_vz_SET((float) -3.3036587E38F, PH.base.pack) ;
    p3_afx_SET((float)1.9073002E38F, PH.base.pack) ;
    p3_afy_SET((float)2.011187E38F, PH.base.pack) ;
    p3_afz_SET((float)2.5397694E37F, PH.base.pack) ;
    p3_yaw_SET((float) -3.032646E38F, PH.base.pack) ;
    p3_yaw_rate_SET((float)3.7913224E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PING_4(), &PH);
    p4_time_usec_SET((uint64_t)673155150286755847L, PH.base.pack) ;
    p4_seq_SET((uint32_t)3218909385L, PH.base.pack) ;
    p4_target_system_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
    p4_target_component_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
    p5_target_system_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
    p5_control_request_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
    p5_version_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
    {
        char16_t   passkey = "ibnrfnbPj";
        p5_passkey_SET(&passkey, 0,  sizeof(passkey), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
    p6_gcs_system_id_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
    p6_control_request_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
    p6_ack_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
    {
        char16_t   key = "txejoe";
        p7_key_SET(&key, 0,  sizeof(key), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
    p11_target_system_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
    p11_base_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_DISARMED, PH.base.pack) ;
    p11_custom_mode_SET((uint32_t)3891238338L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
    p20_target_system_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
    p20_target_component_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
    {
        char16_t   param_id = "e";
        p20_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p20_param_index_SET((int16_t)(int16_t) -24729, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
    p21_target_system_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
    p21_target_component_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
    {
        char16_t   param_id = "kykuezc";
        p22_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p22_param_value_SET((float) -5.999788E37F, PH.base.pack) ;
    p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT8, PH.base.pack) ;
    p22_param_count_SET((uint16_t)(uint16_t)53990, PH.base.pack) ;
    p22_param_index_SET((uint16_t)(uint16_t)30254, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
    p23_target_system_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
    p23_target_component_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
    {
        char16_t   param_id = "ddsjdaubpkfa";
        p23_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p23_param_value_SET((float) -5.457586E37F, PH.base.pack) ;
    p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
    p24_time_usec_SET((uint64_t)5438549161666269089L, PH.base.pack) ;
    p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX, PH.base.pack) ;
    p24_lat_SET((int32_t) -1621416726, PH.base.pack) ;
    p24_lon_SET((int32_t) -1709897973, PH.base.pack) ;
    p24_alt_SET((int32_t)1868937517, PH.base.pack) ;
    p24_eph_SET((uint16_t)(uint16_t)40003, PH.base.pack) ;
    p24_epv_SET((uint16_t)(uint16_t)61024, PH.base.pack) ;
    p24_vel_SET((uint16_t)(uint16_t)21097, PH.base.pack) ;
    p24_cog_SET((uint16_t)(uint16_t)55237, PH.base.pack) ;
    p24_satellites_visible_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
    p24_alt_ellipsoid_SET((int32_t)351495488, &PH) ;
    p24_h_acc_SET((uint32_t)1901262041L, &PH) ;
    p24_v_acc_SET((uint32_t)3978076474L, &PH) ;
    p24_vel_acc_SET((uint32_t)530327306L, &PH) ;
    p24_hdg_acc_SET((uint32_t)3871197547L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
    p25_satellites_visible_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
    {
        uint8_t  satellite_prn [] =  {(uint8_t)50, (uint8_t)10, (uint8_t)240, (uint8_t)0, (uint8_t)170, (uint8_t)249, (uint8_t)40, (uint8_t)238, (uint8_t)162, (uint8_t)96, (uint8_t)126, (uint8_t)189, (uint8_t)141, (uint8_t)101, (uint8_t)66, (uint8_t)62, (uint8_t)200, (uint8_t)252, (uint8_t)242, (uint8_t)207};
        p25_satellite_prn_SET(&satellite_prn, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_used [] =  {(uint8_t)246, (uint8_t)27, (uint8_t)129, (uint8_t)167, (uint8_t)165, (uint8_t)200, (uint8_t)242, (uint8_t)123, (uint8_t)24, (uint8_t)94, (uint8_t)170, (uint8_t)30, (uint8_t)78, (uint8_t)198, (uint8_t)70, (uint8_t)123, (uint8_t)192, (uint8_t)147, (uint8_t)64, (uint8_t)94};
        p25_satellite_used_SET(&satellite_used, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_elevation [] =  {(uint8_t)235, (uint8_t)64, (uint8_t)169, (uint8_t)61, (uint8_t)34, (uint8_t)245, (uint8_t)75, (uint8_t)67, (uint8_t)174, (uint8_t)160, (uint8_t)248, (uint8_t)215, (uint8_t)90, (uint8_t)185, (uint8_t)232, (uint8_t)239, (uint8_t)1, (uint8_t)225, (uint8_t)125, (uint8_t)184};
        p25_satellite_elevation_SET(&satellite_elevation, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_azimuth [] =  {(uint8_t)237, (uint8_t)53, (uint8_t)121, (uint8_t)254, (uint8_t)63, (uint8_t)48, (uint8_t)153, (uint8_t)184, (uint8_t)174, (uint8_t)122, (uint8_t)49, (uint8_t)218, (uint8_t)61, (uint8_t)49, (uint8_t)234, (uint8_t)7, (uint8_t)94, (uint8_t)117, (uint8_t)49, (uint8_t)126};
        p25_satellite_azimuth_SET(&satellite_azimuth, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_snr [] =  {(uint8_t)101, (uint8_t)238, (uint8_t)234, (uint8_t)221, (uint8_t)210, (uint8_t)221, (uint8_t)115, (uint8_t)91, (uint8_t)46, (uint8_t)135, (uint8_t)201, (uint8_t)56, (uint8_t)221, (uint8_t)37, (uint8_t)151, (uint8_t)73, (uint8_t)129, (uint8_t)77, (uint8_t)119, (uint8_t)51};
        p25_satellite_snr_SET(&satellite_snr, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
    p26_time_boot_ms_SET((uint32_t)3809748822L, PH.base.pack) ;
    p26_xacc_SET((int16_t)(int16_t) -10683, PH.base.pack) ;
    p26_yacc_SET((int16_t)(int16_t)23323, PH.base.pack) ;
    p26_zacc_SET((int16_t)(int16_t) -27406, PH.base.pack) ;
    p26_xgyro_SET((int16_t)(int16_t) -22632, PH.base.pack) ;
    p26_ygyro_SET((int16_t)(int16_t)20609, PH.base.pack) ;
    p26_zgyro_SET((int16_t)(int16_t)3018, PH.base.pack) ;
    p26_xmag_SET((int16_t)(int16_t) -5740, PH.base.pack) ;
    p26_ymag_SET((int16_t)(int16_t) -22345, PH.base.pack) ;
    p26_zmag_SET((int16_t)(int16_t) -3688, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
    p27_time_usec_SET((uint64_t)6711813011547934101L, PH.base.pack) ;
    p27_xacc_SET((int16_t)(int16_t) -6809, PH.base.pack) ;
    p27_yacc_SET((int16_t)(int16_t) -24332, PH.base.pack) ;
    p27_zacc_SET((int16_t)(int16_t)28750, PH.base.pack) ;
    p27_xgyro_SET((int16_t)(int16_t)8980, PH.base.pack) ;
    p27_ygyro_SET((int16_t)(int16_t)10392, PH.base.pack) ;
    p27_zgyro_SET((int16_t)(int16_t)23599, PH.base.pack) ;
    p27_xmag_SET((int16_t)(int16_t) -20853, PH.base.pack) ;
    p27_ymag_SET((int16_t)(int16_t)21458, PH.base.pack) ;
    p27_zmag_SET((int16_t)(int16_t)28914, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
    p28_time_usec_SET((uint64_t)5964514890389255692L, PH.base.pack) ;
    p28_press_abs_SET((int16_t)(int16_t)11881, PH.base.pack) ;
    p28_press_diff1_SET((int16_t)(int16_t) -23172, PH.base.pack) ;
    p28_press_diff2_SET((int16_t)(int16_t) -18999, PH.base.pack) ;
    p28_temperature_SET((int16_t)(int16_t) -1751, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
    p29_time_boot_ms_SET((uint32_t)3929745268L, PH.base.pack) ;
    p29_press_abs_SET((float) -1.3714157E38F, PH.base.pack) ;
    p29_press_diff_SET((float) -3.0464138E35F, PH.base.pack) ;
    p29_temperature_SET((int16_t)(int16_t)21158, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
    p30_time_boot_ms_SET((uint32_t)590665218L, PH.base.pack) ;
    p30_roll_SET((float)1.7541606E37F, PH.base.pack) ;
    p30_pitch_SET((float)3.18356E37F, PH.base.pack) ;
    p30_yaw_SET((float) -3.6599692E37F, PH.base.pack) ;
    p30_rollspeed_SET((float) -5.657639E37F, PH.base.pack) ;
    p30_pitchspeed_SET((float)2.5779788E38F, PH.base.pack) ;
    p30_yawspeed_SET((float)2.680326E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
    p31_time_boot_ms_SET((uint32_t)891019921L, PH.base.pack) ;
    p31_q1_SET((float)6.659233E37F, PH.base.pack) ;
    p31_q2_SET((float) -2.7509807E38F, PH.base.pack) ;
    p31_q3_SET((float)1.4319285E37F, PH.base.pack) ;
    p31_q4_SET((float) -1.6074223E38F, PH.base.pack) ;
    p31_rollspeed_SET((float) -3.0985855E38F, PH.base.pack) ;
    p31_pitchspeed_SET((float)1.4155143E38F, PH.base.pack) ;
    p31_yawspeed_SET((float)1.6861079E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
    p32_time_boot_ms_SET((uint32_t)1593341101L, PH.base.pack) ;
    p32_x_SET((float)2.688693E38F, PH.base.pack) ;
    p32_y_SET((float) -9.119855E37F, PH.base.pack) ;
    p32_z_SET((float)1.5861718E38F, PH.base.pack) ;
    p32_vx_SET((float)2.6009748E38F, PH.base.pack) ;
    p32_vy_SET((float)1.688825E38F, PH.base.pack) ;
    p32_vz_SET((float) -4.1982073E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
    p33_time_boot_ms_SET((uint32_t)268502384L, PH.base.pack) ;
    p33_lat_SET((int32_t) -703103273, PH.base.pack) ;
    p33_lon_SET((int32_t) -1590573189, PH.base.pack) ;
    p33_alt_SET((int32_t)1022411655, PH.base.pack) ;
    p33_relative_alt_SET((int32_t)2117787315, PH.base.pack) ;
    p33_vx_SET((int16_t)(int16_t)32709, PH.base.pack) ;
    p33_vy_SET((int16_t)(int16_t) -11959, PH.base.pack) ;
    p33_vz_SET((int16_t)(int16_t) -21353, PH.base.pack) ;
    p33_hdg_SET((uint16_t)(uint16_t)7648, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
    p34_time_boot_ms_SET((uint32_t)3417655603L, PH.base.pack) ;
    p34_port_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    p34_chan1_scaled_SET((int16_t)(int16_t)29338, PH.base.pack) ;
    p34_chan2_scaled_SET((int16_t)(int16_t) -4848, PH.base.pack) ;
    p34_chan3_scaled_SET((int16_t)(int16_t) -32743, PH.base.pack) ;
    p34_chan4_scaled_SET((int16_t)(int16_t) -18176, PH.base.pack) ;
    p34_chan5_scaled_SET((int16_t)(int16_t) -9807, PH.base.pack) ;
    p34_chan6_scaled_SET((int16_t)(int16_t) -27770, PH.base.pack) ;
    p34_chan7_scaled_SET((int16_t)(int16_t) -24895, PH.base.pack) ;
    p34_chan8_scaled_SET((int16_t)(int16_t)23064, PH.base.pack) ;
    p34_rssi_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
    p35_time_boot_ms_SET((uint32_t)3863565875L, PH.base.pack) ;
    p35_port_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    p35_chan1_raw_SET((uint16_t)(uint16_t)64979, PH.base.pack) ;
    p35_chan2_raw_SET((uint16_t)(uint16_t)46929, PH.base.pack) ;
    p35_chan3_raw_SET((uint16_t)(uint16_t)32069, PH.base.pack) ;
    p35_chan4_raw_SET((uint16_t)(uint16_t)52470, PH.base.pack) ;
    p35_chan5_raw_SET((uint16_t)(uint16_t)12704, PH.base.pack) ;
    p35_chan6_raw_SET((uint16_t)(uint16_t)57327, PH.base.pack) ;
    p35_chan7_raw_SET((uint16_t)(uint16_t)40738, PH.base.pack) ;
    p35_chan8_raw_SET((uint16_t)(uint16_t)58496, PH.base.pack) ;
    p35_rssi_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
    p36_time_usec_SET((uint32_t)2294571328L, PH.base.pack) ;
    p36_port_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
    p36_servo1_raw_SET((uint16_t)(uint16_t)63592, PH.base.pack) ;
    p36_servo2_raw_SET((uint16_t)(uint16_t)15038, PH.base.pack) ;
    p36_servo3_raw_SET((uint16_t)(uint16_t)16942, PH.base.pack) ;
    p36_servo4_raw_SET((uint16_t)(uint16_t)56486, PH.base.pack) ;
    p36_servo5_raw_SET((uint16_t)(uint16_t)15177, PH.base.pack) ;
    p36_servo6_raw_SET((uint16_t)(uint16_t)9893, PH.base.pack) ;
    p36_servo7_raw_SET((uint16_t)(uint16_t)62018, PH.base.pack) ;
    p36_servo8_raw_SET((uint16_t)(uint16_t)9652, PH.base.pack) ;
    p36_servo9_raw_SET((uint16_t)(uint16_t)11891, &PH) ;
    p36_servo10_raw_SET((uint16_t)(uint16_t)29996, &PH) ;
    p36_servo11_raw_SET((uint16_t)(uint16_t)44695, &PH) ;
    p36_servo12_raw_SET((uint16_t)(uint16_t)10638, &PH) ;
    p36_servo13_raw_SET((uint16_t)(uint16_t)45869, &PH) ;
    p36_servo14_raw_SET((uint16_t)(uint16_t)17309, &PH) ;
    p36_servo15_raw_SET((uint16_t)(uint16_t)13168, &PH) ;
    p36_servo16_raw_SET((uint16_t)(uint16_t)59951, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
    p37_target_system_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
    p37_target_component_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
    p37_start_index_SET((int16_t)(int16_t)2860, PH.base.pack) ;
    p37_end_index_SET((int16_t)(int16_t)28508, PH.base.pack) ;
    p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
    p38_target_system_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
    p38_target_component_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
    p38_start_index_SET((int16_t)(int16_t) -26633, PH.base.pack) ;
    p38_end_index_SET((int16_t)(int16_t) -13980, PH.base.pack) ;
    p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
    p39_target_system_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
    p39_target_component_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
    p39_seq_SET((uint16_t)(uint16_t)10103, PH.base.pack) ;
    p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
    p39_command_SET(e_MAV_CMD_MAV_CMD_DO_ENGINE_CONTROL, PH.base.pack) ;
    p39_current_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
    p39_autocontinue_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
    p39_param1_SET((float) -1.5770371E38F, PH.base.pack) ;
    p39_param2_SET((float)1.5457778E38F, PH.base.pack) ;
    p39_param3_SET((float) -1.9919166E38F, PH.base.pack) ;
    p39_param4_SET((float)2.7603599E38F, PH.base.pack) ;
    p39_x_SET((float)1.6641845E38F, PH.base.pack) ;
    p39_y_SET((float)2.6656883E38F, PH.base.pack) ;
    p39_z_SET((float)2.0164313E38F, PH.base.pack) ;
    p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
    p40_target_system_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
    p40_target_component_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
    p40_seq_SET((uint16_t)(uint16_t)62417, PH.base.pack) ;
    p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
    p41_target_system_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
    p41_target_component_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
    p41_seq_SET((uint16_t)(uint16_t)15632, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
    p42_seq_SET((uint16_t)(uint16_t)53760, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
    p43_target_system_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
    p43_target_component_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
    p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
    p44_target_system_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
    p44_target_component_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
    p44_count_SET((uint16_t)(uint16_t)55450, PH.base.pack) ;
    p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
    p45_target_system_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
    p45_target_component_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
    p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
    p46_seq_SET((uint16_t)(uint16_t)17628, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
    p47_target_system_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
    p47_target_component_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
    p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM7, PH.base.pack) ;
    p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
    p48_target_system_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
    p48_latitude_SET((int32_t)1453625282, PH.base.pack) ;
    p48_longitude_SET((int32_t) -1442114313, PH.base.pack) ;
    p48_altitude_SET((int32_t) -84505620, PH.base.pack) ;
    p48_time_usec_SET((uint64_t)2757914955522034011L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
    p49_latitude_SET((int32_t)800997571, PH.base.pack) ;
    p49_longitude_SET((int32_t) -2115590744, PH.base.pack) ;
    p49_altitude_SET((int32_t)1750907823, PH.base.pack) ;
    p49_time_usec_SET((uint64_t)892360354152417483L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
    p50_target_system_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
    p50_target_component_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
    {
        char16_t   param_id = "kYaasg";
        p50_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p50_param_index_SET((int16_t)(int16_t)9236, PH.base.pack) ;
    p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
    p50_param_value0_SET((float) -3.0419114E38F, PH.base.pack) ;
    p50_scale_SET((float)2.366529E37F, PH.base.pack) ;
    p50_param_value_min_SET((float) -1.5346541E38F, PH.base.pack) ;
    p50_param_value_max_SET((float)1.6159185E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
    p51_target_system_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
    p51_target_component_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
    p51_seq_SET((uint16_t)(uint16_t)36305, PH.base.pack) ;
    p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
    p54_target_system_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
    p54_target_component_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
    p54_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
    p54_p1x_SET((float)2.5275584E38F, PH.base.pack) ;
    p54_p1y_SET((float) -3.374551E38F, PH.base.pack) ;
    p54_p1z_SET((float) -2.771131E38F, PH.base.pack) ;
    p54_p2x_SET((float)2.8739554E38F, PH.base.pack) ;
    p54_p2y_SET((float)3.3895845E38F, PH.base.pack) ;
    p54_p2z_SET((float) -3.2989333E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
    p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
    p55_p1x_SET((float) -2.1580153E38F, PH.base.pack) ;
    p55_p1y_SET((float) -2.1891638E38F, PH.base.pack) ;
    p55_p1z_SET((float) -1.4008857E37F, PH.base.pack) ;
    p55_p2x_SET((float) -9.362351E37F, PH.base.pack) ;
    p55_p2y_SET((float)2.910098E38F, PH.base.pack) ;
    p55_p2z_SET((float)8.0024293E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
    p61_time_usec_SET((uint64_t)5188271734225377151L, PH.base.pack) ;
    {
        float  q [] =  {2.9685215E38F, -1.5304942E38F, 1.6572899E37F, 1.506658E38F};
        p61_q_SET(&q, 0, &PH.base.pack) ;
    }
    p61_rollspeed_SET((float) -5.8917515E37F, PH.base.pack) ;
    p61_pitchspeed_SET((float)3.2640918E38F, PH.base.pack) ;
    p61_yawspeed_SET((float) -1.6879748E38F, PH.base.pack) ;
    {
        float  covariance [] =  {6.172111E36F, -5.80965E37F, -2.0710208E38F, -1.1710511E38F, -2.808119E38F, 1.8965298E38F, 3.2259258E38F, -2.6555445E36F, -2.3965141E38F};
        p61_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
    p62_nav_roll_SET((float) -1.1198453E38F, PH.base.pack) ;
    p62_nav_pitch_SET((float)2.7555672E38F, PH.base.pack) ;
    p62_nav_bearing_SET((int16_t)(int16_t)15521, PH.base.pack) ;
    p62_target_bearing_SET((int16_t)(int16_t) -9760, PH.base.pack) ;
    p62_wp_dist_SET((uint16_t)(uint16_t)18827, PH.base.pack) ;
    p62_alt_error_SET((float)9.840972E37F, PH.base.pack) ;
    p62_aspd_error_SET((float)3.330724E36F, PH.base.pack) ;
    p62_xtrack_error_SET((float)3.3780739E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
    p63_time_usec_SET((uint64_t)6919657676230582051L, PH.base.pack) ;
    p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE, PH.base.pack) ;
    p63_lat_SET((int32_t)912057006, PH.base.pack) ;
    p63_lon_SET((int32_t)321362216, PH.base.pack) ;
    p63_alt_SET((int32_t) -1678494353, PH.base.pack) ;
    p63_relative_alt_SET((int32_t) -1586422653, PH.base.pack) ;
    p63_vx_SET((float) -2.8990094E38F, PH.base.pack) ;
    p63_vy_SET((float) -3.1972523E38F, PH.base.pack) ;
    p63_vz_SET((float)2.2314307E38F, PH.base.pack) ;
    {
        float  covariance [] =  {6.0080275E37F, 1.0475283E38F, 1.4200545E38F, 3.6900665E37F, -1.3038204E38F, 2.4788668E38F, 9.771844E37F, -1.7025955E38F, 6.446973E37F, 7.1565747E37F, 2.176159E38F, 1.3600311E38F, -3.1020658E38F, -3.5763098E37F, 1.4921436E38F, -4.3563933E37F, 3.9916634E37F, -9.749929E37F, 2.3891174E38F, -2.162012E38F, -1.4833569E38F, -2.2824063E38F, 8.984298E37F, 2.034594E38F, 8.866105E37F, 1.6975455E38F, 1.440002E38F, 3.2100532E38F, -3.281858E38F, -8.731673E37F, -2.9600433E38F, -7.2006925E37F, 2.296917E38F, 1.6309547E38F, 6.7518123E37F, -2.6274251E38F};
        p63_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
    p64_time_usec_SET((uint64_t)7181512568074637205L, PH.base.pack) ;
    p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE, PH.base.pack) ;
    p64_x_SET((float)2.5671533E38F, PH.base.pack) ;
    p64_y_SET((float)3.161956E38F, PH.base.pack) ;
    p64_z_SET((float)2.8835907E37F, PH.base.pack) ;
    p64_vx_SET((float) -1.8383408E38F, PH.base.pack) ;
    p64_vy_SET((float) -5.8979437E37F, PH.base.pack) ;
    p64_vz_SET((float) -2.3236374E38F, PH.base.pack) ;
    p64_ax_SET((float)2.255646E38F, PH.base.pack) ;
    p64_ay_SET((float) -1.672633E37F, PH.base.pack) ;
    p64_az_SET((float)2.5068553E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-2.7991885E38F, 3.2436144E37F, 1.9744488E38F, 7.4579135E37F, 5.111103E37F, -8.229575E37F, 2.7531935E38F, -3.0238166E38F, -1.5218076E38F, -2.2063223E38F, 1.9466168E38F, -1.3121028E37F, -2.9260402E38F, -3.0407032E38F, 1.7950547E38F, -1.392788E38F, -3.2709661E38F, 2.8028203E38F, -1.713464E38F, -4.995606E37F, -2.459424E38F, -2.0261872E38F, -2.0501835E38F, 7.2424367E37F, 2.8874908E38F, -3.138754E38F, 2.958616E38F, -3.908422E37F, -1.8536662E38F, -1.9442107E38F, -6.999957E37F, 2.824918E38F, 6.5948153E37F, 3.2127012E38F, -2.0165378E38F, -8.644314E36F, 2.3222147E37F, -1.5389284E38F, 2.0108782E38F, -2.3443693E38F, -1.9515775E38F, 1.068727E38F, 3.8317533E37F, 3.348967E38F, 1.6895389E38F};
        p64_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
    p65_time_boot_ms_SET((uint32_t)3197395651L, PH.base.pack) ;
    p65_chancount_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
    p65_chan1_raw_SET((uint16_t)(uint16_t)15072, PH.base.pack) ;
    p65_chan2_raw_SET((uint16_t)(uint16_t)622, PH.base.pack) ;
    p65_chan3_raw_SET((uint16_t)(uint16_t)14886, PH.base.pack) ;
    p65_chan4_raw_SET((uint16_t)(uint16_t)23945, PH.base.pack) ;
    p65_chan5_raw_SET((uint16_t)(uint16_t)20372, PH.base.pack) ;
    p65_chan6_raw_SET((uint16_t)(uint16_t)13538, PH.base.pack) ;
    p65_chan7_raw_SET((uint16_t)(uint16_t)12599, PH.base.pack) ;
    p65_chan8_raw_SET((uint16_t)(uint16_t)32420, PH.base.pack) ;
    p65_chan9_raw_SET((uint16_t)(uint16_t)40423, PH.base.pack) ;
    p65_chan10_raw_SET((uint16_t)(uint16_t)25766, PH.base.pack) ;
    p65_chan11_raw_SET((uint16_t)(uint16_t)56751, PH.base.pack) ;
    p65_chan12_raw_SET((uint16_t)(uint16_t)34078, PH.base.pack) ;
    p65_chan13_raw_SET((uint16_t)(uint16_t)27522, PH.base.pack) ;
    p65_chan14_raw_SET((uint16_t)(uint16_t)19343, PH.base.pack) ;
    p65_chan15_raw_SET((uint16_t)(uint16_t)8458, PH.base.pack) ;
    p65_chan16_raw_SET((uint16_t)(uint16_t)7635, PH.base.pack) ;
    p65_chan17_raw_SET((uint16_t)(uint16_t)10311, PH.base.pack) ;
    p65_chan18_raw_SET((uint16_t)(uint16_t)12959, PH.base.pack) ;
    p65_rssi_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
    p66_target_system_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
    p66_target_component_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
    p66_req_stream_id_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
    p66_req_message_rate_SET((uint16_t)(uint16_t)42072, PH.base.pack) ;
    p66_start_stop_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
    p67_stream_id_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
    p67_message_rate_SET((uint16_t)(uint16_t)4291, PH.base.pack) ;
    p67_on_off_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
    p69_target_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
    p69_x_SET((int16_t)(int16_t) -23555, PH.base.pack) ;
    p69_y_SET((int16_t)(int16_t)27973, PH.base.pack) ;
    p69_z_SET((int16_t)(int16_t) -21836, PH.base.pack) ;
    p69_r_SET((int16_t)(int16_t)6896, PH.base.pack) ;
    p69_buttons_SET((uint16_t)(uint16_t)31903, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
    p70_target_system_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
    p70_target_component_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
    p70_chan1_raw_SET((uint16_t)(uint16_t)54284, PH.base.pack) ;
    p70_chan2_raw_SET((uint16_t)(uint16_t)24391, PH.base.pack) ;
    p70_chan3_raw_SET((uint16_t)(uint16_t)55934, PH.base.pack) ;
    p70_chan4_raw_SET((uint16_t)(uint16_t)40741, PH.base.pack) ;
    p70_chan5_raw_SET((uint16_t)(uint16_t)45012, PH.base.pack) ;
    p70_chan6_raw_SET((uint16_t)(uint16_t)57989, PH.base.pack) ;
    p70_chan7_raw_SET((uint16_t)(uint16_t)57799, PH.base.pack) ;
    p70_chan8_raw_SET((uint16_t)(uint16_t)42671, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
    p73_target_system_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
    p73_target_component_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
    p73_seq_SET((uint16_t)(uint16_t)17645, PH.base.pack) ;
    p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
    p73_command_SET(e_MAV_CMD_MAV_CMD_NAV_LAND_LOCAL, PH.base.pack) ;
    p73_current_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
    p73_autocontinue_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
    p73_param1_SET((float)2.6315203E38F, PH.base.pack) ;
    p73_param2_SET((float) -1.0935899E38F, PH.base.pack) ;
    p73_param3_SET((float)8.982411E37F, PH.base.pack) ;
    p73_param4_SET((float) -2.614626E38F, PH.base.pack) ;
    p73_x_SET((int32_t) -302754228, PH.base.pack) ;
    p73_y_SET((int32_t) -1628080246, PH.base.pack) ;
    p73_z_SET((float) -1.00294056E37F, PH.base.pack) ;
    p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
    p74_airspeed_SET((float) -2.0692509E36F, PH.base.pack) ;
    p74_groundspeed_SET((float)1.5504547E38F, PH.base.pack) ;
    p74_heading_SET((int16_t)(int16_t) -2921, PH.base.pack) ;
    p74_throttle_SET((uint16_t)(uint16_t)63171, PH.base.pack) ;
    p74_alt_SET((float) -2.4179705E38F, PH.base.pack) ;
    p74_climb_SET((float) -1.8141467E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
    p75_target_system_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
    p75_target_component_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
    p75_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
    p75_command_SET(e_MAV_CMD_MAV_CMD_PAYLOAD_PREPARE_DEPLOY, PH.base.pack) ;
    p75_current_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
    p75_autocontinue_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
    p75_param1_SET((float) -2.0777623E38F, PH.base.pack) ;
    p75_param2_SET((float) -2.0718126E38F, PH.base.pack) ;
    p75_param3_SET((float)2.669973E38F, PH.base.pack) ;
    p75_param4_SET((float)1.1963802E38F, PH.base.pack) ;
    p75_x_SET((int32_t) -933106834, PH.base.pack) ;
    p75_y_SET((int32_t) -1591347506, PH.base.pack) ;
    p75_z_SET((float)2.82447E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
    p76_target_system_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
    p76_target_component_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
    p76_command_SET(e_MAV_CMD_MAV_CMD_VIDEO_STOP_STREAMING, PH.base.pack) ;
    p76_confirmation_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
    p76_param1_SET((float)2.7563245E38F, PH.base.pack) ;
    p76_param2_SET((float)7.98883E36F, PH.base.pack) ;
    p76_param3_SET((float)2.232927E37F, PH.base.pack) ;
    p76_param4_SET((float) -5.154905E37F, PH.base.pack) ;
    p76_param5_SET((float)1.759963E38F, PH.base.pack) ;
    p76_param6_SET((float)1.5856067E38F, PH.base.pack) ;
    p76_param7_SET((float) -2.188497E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
    p77_command_SET(e_MAV_CMD_MAV_CMD_NAV_SPLINE_WAYPOINT, PH.base.pack) ;
    p77_result_SET(e_MAV_RESULT_MAV_RESULT_UNSUPPORTED, PH.base.pack) ;
    p77_progress_SET((uint8_t)(uint8_t)105, &PH) ;
    p77_result_param2_SET((int32_t)1457435366, &PH) ;
    p77_target_system_SET((uint8_t)(uint8_t)45, &PH) ;
    p77_target_component_SET((uint8_t)(uint8_t)131, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
    p81_time_boot_ms_SET((uint32_t)3920963015L, PH.base.pack) ;
    p81_roll_SET((float) -6.9943793E37F, PH.base.pack) ;
    p81_pitch_SET((float) -1.3466088E38F, PH.base.pack) ;
    p81_yaw_SET((float) -2.0640836E38F, PH.base.pack) ;
    p81_thrust_SET((float) -2.235762E38F, PH.base.pack) ;
    p81_mode_switch_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
    p81_manual_override_switch_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
    p82_time_boot_ms_SET((uint32_t)3442725412L, PH.base.pack) ;
    p82_target_system_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
    p82_target_component_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
    p82_type_mask_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
    {
        float  q [] =  {-8.822691E37F, -1.7368397E38F, 1.6583507E38F, -2.3336922E38F};
        p82_q_SET(&q, 0, &PH.base.pack) ;
    }
    p82_body_roll_rate_SET((float)5.5432616E37F, PH.base.pack) ;
    p82_body_pitch_rate_SET((float)2.3415109E38F, PH.base.pack) ;
    p82_body_yaw_rate_SET((float)2.7905815E38F, PH.base.pack) ;
    p82_thrust_SET((float) -2.1560771E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
    p83_time_boot_ms_SET((uint32_t)3034193941L, PH.base.pack) ;
    p83_type_mask_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
    {
        float  q [] =  {-7.4673247E36F, -1.5058105E38F, 1.01866E38F, 2.8912222E38F};
        p83_q_SET(&q, 0, &PH.base.pack) ;
    }
    p83_body_roll_rate_SET((float)1.9407063E38F, PH.base.pack) ;
    p83_body_pitch_rate_SET((float)1.3311501E38F, PH.base.pack) ;
    p83_body_yaw_rate_SET((float) -4.433938E37F, PH.base.pack) ;
    p83_thrust_SET((float) -1.3951346E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
    p84_time_boot_ms_SET((uint32_t)1130758341L, PH.base.pack) ;
    p84_target_system_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
    p84_target_component_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
    p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
    p84_type_mask_SET((uint16_t)(uint16_t)50638, PH.base.pack) ;
    p84_x_SET((float) -2.3877597E38F, PH.base.pack) ;
    p84_y_SET((float)7.7726346E37F, PH.base.pack) ;
    p84_z_SET((float) -3.53505E37F, PH.base.pack) ;
    p84_vx_SET((float) -4.525448E37F, PH.base.pack) ;
    p84_vy_SET((float)4.912087E37F, PH.base.pack) ;
    p84_vz_SET((float)1.1779169E38F, PH.base.pack) ;
    p84_afx_SET((float) -3.9487722E37F, PH.base.pack) ;
    p84_afy_SET((float) -2.8101719E38F, PH.base.pack) ;
    p84_afz_SET((float) -3.2060642E38F, PH.base.pack) ;
    p84_yaw_SET((float)1.6070506E38F, PH.base.pack) ;
    p84_yaw_rate_SET((float) -1.4734883E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
    p86_time_boot_ms_SET((uint32_t)1068102232L, PH.base.pack) ;
    p86_target_system_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
    p86_target_component_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
    p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
    p86_type_mask_SET((uint16_t)(uint16_t)14054, PH.base.pack) ;
    p86_lat_int_SET((int32_t)1953791254, PH.base.pack) ;
    p86_lon_int_SET((int32_t) -1770941345, PH.base.pack) ;
    p86_alt_SET((float) -5.317274E37F, PH.base.pack) ;
    p86_vx_SET((float)1.2317968E38F, PH.base.pack) ;
    p86_vy_SET((float) -2.0937448E38F, PH.base.pack) ;
    p86_vz_SET((float) -3.014403E38F, PH.base.pack) ;
    p86_afx_SET((float) -1.1510624E38F, PH.base.pack) ;
    p86_afy_SET((float)8.1524796E37F, PH.base.pack) ;
    p86_afz_SET((float)8.679599E36F, PH.base.pack) ;
    p86_yaw_SET((float) -1.1788206E38F, PH.base.pack) ;
    p86_yaw_rate_SET((float)1.8138542E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
    p87_time_boot_ms_SET((uint32_t)453551063L, PH.base.pack) ;
    p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
    p87_type_mask_SET((uint16_t)(uint16_t)47541, PH.base.pack) ;
    p87_lat_int_SET((int32_t)2134800374, PH.base.pack) ;
    p87_lon_int_SET((int32_t) -315149106, PH.base.pack) ;
    p87_alt_SET((float)2.3307983E38F, PH.base.pack) ;
    p87_vx_SET((float)2.9647403E38F, PH.base.pack) ;
    p87_vy_SET((float)1.2456156E38F, PH.base.pack) ;
    p87_vz_SET((float) -1.9502612E38F, PH.base.pack) ;
    p87_afx_SET((float)8.693464E37F, PH.base.pack) ;
    p87_afy_SET((float) -6.353699E37F, PH.base.pack) ;
    p87_afz_SET((float) -1.8841866E36F, PH.base.pack) ;
    p87_yaw_SET((float)3.323751E38F, PH.base.pack) ;
    p87_yaw_rate_SET((float) -2.4146541E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
    p89_time_boot_ms_SET((uint32_t)3202728705L, PH.base.pack) ;
    p89_x_SET((float)2.532127E38F, PH.base.pack) ;
    p89_y_SET((float) -8.3407754E37F, PH.base.pack) ;
    p89_z_SET((float) -3.096978E38F, PH.base.pack) ;
    p89_roll_SET((float)1.0331373E38F, PH.base.pack) ;
    p89_pitch_SET((float)1.8167246E38F, PH.base.pack) ;
    p89_yaw_SET((float) -3.299574E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
    p90_time_usec_SET((uint64_t)2623279210004356774L, PH.base.pack) ;
    p90_roll_SET((float)1.5172265E38F, PH.base.pack) ;
    p90_pitch_SET((float) -2.9762605E38F, PH.base.pack) ;
    p90_yaw_SET((float)6.205101E37F, PH.base.pack) ;
    p90_rollspeed_SET((float) -6.8472827E37F, PH.base.pack) ;
    p90_pitchspeed_SET((float) -2.0406364E38F, PH.base.pack) ;
    p90_yawspeed_SET((float)2.8963839E38F, PH.base.pack) ;
    p90_lat_SET((int32_t)1964060285, PH.base.pack) ;
    p90_lon_SET((int32_t) -227459188, PH.base.pack) ;
    p90_alt_SET((int32_t) -911663800, PH.base.pack) ;
    p90_vx_SET((int16_t)(int16_t)32178, PH.base.pack) ;
    p90_vy_SET((int16_t)(int16_t) -9374, PH.base.pack) ;
    p90_vz_SET((int16_t)(int16_t)21648, PH.base.pack) ;
    p90_xacc_SET((int16_t)(int16_t)1711, PH.base.pack) ;
    p90_yacc_SET((int16_t)(int16_t)27925, PH.base.pack) ;
    p90_zacc_SET((int16_t)(int16_t)29678, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
    p91_time_usec_SET((uint64_t)1813557255719657592L, PH.base.pack) ;
    p91_roll_ailerons_SET((float) -2.9402133E38F, PH.base.pack) ;
    p91_pitch_elevator_SET((float) -2.6338404E38F, PH.base.pack) ;
    p91_yaw_rudder_SET((float)9.73482E37F, PH.base.pack) ;
    p91_throttle_SET((float)2.3180837E38F, PH.base.pack) ;
    p91_aux1_SET((float) -2.0539716E38F, PH.base.pack) ;
    p91_aux2_SET((float) -2.5378832E37F, PH.base.pack) ;
    p91_aux3_SET((float) -1.4072659E38F, PH.base.pack) ;
    p91_aux4_SET((float) -8.3469666E37F, PH.base.pack) ;
    p91_mode_SET(e_MAV_MODE_MAV_MODE_TEST_DISARMED, PH.base.pack) ;
    p91_nav_mode_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
    p92_time_usec_SET((uint64_t)8851004949681717443L, PH.base.pack) ;
    p92_chan1_raw_SET((uint16_t)(uint16_t)4340, PH.base.pack) ;
    p92_chan2_raw_SET((uint16_t)(uint16_t)17483, PH.base.pack) ;
    p92_chan3_raw_SET((uint16_t)(uint16_t)55783, PH.base.pack) ;
    p92_chan4_raw_SET((uint16_t)(uint16_t)3890, PH.base.pack) ;
    p92_chan5_raw_SET((uint16_t)(uint16_t)57280, PH.base.pack) ;
    p92_chan6_raw_SET((uint16_t)(uint16_t)42894, PH.base.pack) ;
    p92_chan7_raw_SET((uint16_t)(uint16_t)64849, PH.base.pack) ;
    p92_chan8_raw_SET((uint16_t)(uint16_t)449, PH.base.pack) ;
    p92_chan9_raw_SET((uint16_t)(uint16_t)24252, PH.base.pack) ;
    p92_chan10_raw_SET((uint16_t)(uint16_t)24624, PH.base.pack) ;
    p92_chan11_raw_SET((uint16_t)(uint16_t)65022, PH.base.pack) ;
    p92_chan12_raw_SET((uint16_t)(uint16_t)57818, PH.base.pack) ;
    p92_rssi_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
    p93_time_usec_SET((uint64_t)8223293580512759930L, PH.base.pack) ;
    {
        float  controls [] =  {2.6987612E38F, 1.8851524E38F, -3.3723341E38F, -2.82559E38F, -1.4273499E38F, 4.6271336E37F, -4.868922E37F, 7.7398826E37F, 2.9060202E38F, -1.0715245E37F, 2.8747299E37F, -1.4755045E38F, 1.022696E38F, 3.212552E38F, -2.9892753E37F, -9.975208E37F};
        p93_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    p93_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_ARMED, PH.base.pack) ;
    p93_flags_SET((uint64_t)5130991619616741608L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
    p100_time_usec_SET((uint64_t)42550278598171286L, PH.base.pack) ;
    p100_sensor_id_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
    p100_flow_x_SET((int16_t)(int16_t) -18851, PH.base.pack) ;
    p100_flow_y_SET((int16_t)(int16_t) -31654, PH.base.pack) ;
    p100_flow_comp_m_x_SET((float) -2.0488512E37F, PH.base.pack) ;
    p100_flow_comp_m_y_SET((float)1.3899211E38F, PH.base.pack) ;
    p100_quality_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
    p100_ground_distance_SET((float)1.3484765E38F, PH.base.pack) ;
    p100_flow_rate_x_SET((float)9.078774E37F, &PH) ;
    p100_flow_rate_y_SET((float)7.3332365E37F, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
    p101_usec_SET((uint64_t)1130168857076733589L, PH.base.pack) ;
    p101_x_SET((float) -2.9566658E38F, PH.base.pack) ;
    p101_y_SET((float) -3.3665127E38F, PH.base.pack) ;
    p101_z_SET((float) -3.6575685E37F, PH.base.pack) ;
    p101_roll_SET((float)2.1783908E38F, PH.base.pack) ;
    p101_pitch_SET((float)4.241108E37F, PH.base.pack) ;
    p101_yaw_SET((float) -2.3587357E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
    p102_usec_SET((uint64_t)1694921601228442136L, PH.base.pack) ;
    p102_x_SET((float) -2.6337977E38F, PH.base.pack) ;
    p102_y_SET((float) -1.1149963E38F, PH.base.pack) ;
    p102_z_SET((float) -2.2626553E38F, PH.base.pack) ;
    p102_roll_SET((float)9.787536E37F, PH.base.pack) ;
    p102_pitch_SET((float) -1.7983017E38F, PH.base.pack) ;
    p102_yaw_SET((float) -4.9437425E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
    p103_usec_SET((uint64_t)4232197828260574706L, PH.base.pack) ;
    p103_x_SET((float) -2.13981E38F, PH.base.pack) ;
    p103_y_SET((float) -9.756284E37F, PH.base.pack) ;
    p103_z_SET((float)2.4995685E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
    p104_usec_SET((uint64_t)650407725228129217L, PH.base.pack) ;
    p104_x_SET((float)2.9584937E38F, PH.base.pack) ;
    p104_y_SET((float)1.0823731E38F, PH.base.pack) ;
    p104_z_SET((float)3.3224104E38F, PH.base.pack) ;
    p104_roll_SET((float)3.120775E36F, PH.base.pack) ;
    p104_pitch_SET((float)5.6840094E37F, PH.base.pack) ;
    p104_yaw_SET((float) -2.0609317E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
    p105_time_usec_SET((uint64_t)8570665567323669104L, PH.base.pack) ;
    p105_xacc_SET((float) -2.7141456E38F, PH.base.pack) ;
    p105_yacc_SET((float)6.996656E37F, PH.base.pack) ;
    p105_zacc_SET((float) -1.9641429E38F, PH.base.pack) ;
    p105_xgyro_SET((float)2.6835885E38F, PH.base.pack) ;
    p105_ygyro_SET((float)7.7656316E37F, PH.base.pack) ;
    p105_zgyro_SET((float) -2.4059846E38F, PH.base.pack) ;
    p105_xmag_SET((float) -1.4751742E38F, PH.base.pack) ;
    p105_ymag_SET((float)3.0512679E38F, PH.base.pack) ;
    p105_zmag_SET((float)3.0419515E38F, PH.base.pack) ;
    p105_abs_pressure_SET((float)2.7760502E38F, PH.base.pack) ;
    p105_diff_pressure_SET((float) -7.6121435E37F, PH.base.pack) ;
    p105_pressure_alt_SET((float)1.9863075E38F, PH.base.pack) ;
    p105_temperature_SET((float)1.4182631E38F, PH.base.pack) ;
    p105_fields_updated_SET((uint16_t)(uint16_t)52062, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
    p106_time_usec_SET((uint64_t)9102525726611498639L, PH.base.pack) ;
    p106_sensor_id_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
    p106_integration_time_us_SET((uint32_t)1574508482L, PH.base.pack) ;
    p106_integrated_x_SET((float)3.18547E38F, PH.base.pack) ;
    p106_integrated_y_SET((float) -2.6881924E38F, PH.base.pack) ;
    p106_integrated_xgyro_SET((float)2.4251779E38F, PH.base.pack) ;
    p106_integrated_ygyro_SET((float) -2.668375E38F, PH.base.pack) ;
    p106_integrated_zgyro_SET((float)2.2190631E38F, PH.base.pack) ;
    p106_temperature_SET((int16_t)(int16_t)15688, PH.base.pack) ;
    p106_quality_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
    p106_time_delta_distance_us_SET((uint32_t)451088096L, PH.base.pack) ;
    p106_distance_SET((float)2.7163124E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
    p107_time_usec_SET((uint64_t)8581409631843585513L, PH.base.pack) ;
    p107_xacc_SET((float) -8.957111E37F, PH.base.pack) ;
    p107_yacc_SET((float)3.1952227E38F, PH.base.pack) ;
    p107_zacc_SET((float)3.0941781E38F, PH.base.pack) ;
    p107_xgyro_SET((float) -3.1337887E37F, PH.base.pack) ;
    p107_ygyro_SET((float) -2.3479356E37F, PH.base.pack) ;
    p107_zgyro_SET((float)1.2905182E38F, PH.base.pack) ;
    p107_xmag_SET((float)3.1112214E38F, PH.base.pack) ;
    p107_ymag_SET((float) -1.4439035E36F, PH.base.pack) ;
    p107_zmag_SET((float)1.3738774E38F, PH.base.pack) ;
    p107_abs_pressure_SET((float)1.3519519E38F, PH.base.pack) ;
    p107_diff_pressure_SET((float)3.2453462E38F, PH.base.pack) ;
    p107_pressure_alt_SET((float) -3.3571323E38F, PH.base.pack) ;
    p107_temperature_SET((float)1.4370549E38F, PH.base.pack) ;
    p107_fields_updated_SET((uint32_t)2416668555L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
    p108_q1_SET((float) -8.48146E37F, PH.base.pack) ;
    p108_q2_SET((float)2.5419956E38F, PH.base.pack) ;
    p108_q3_SET((float)1.3823911E38F, PH.base.pack) ;
    p108_q4_SET((float) -3.9570175E37F, PH.base.pack) ;
    p108_roll_SET((float) -1.1521087E38F, PH.base.pack) ;
    p108_pitch_SET((float) -2.5718495E38F, PH.base.pack) ;
    p108_yaw_SET((float)2.569415E38F, PH.base.pack) ;
    p108_xacc_SET((float) -1.2073694E37F, PH.base.pack) ;
    p108_yacc_SET((float) -3.3387937E37F, PH.base.pack) ;
    p108_zacc_SET((float)2.3187556E38F, PH.base.pack) ;
    p108_xgyro_SET((float) -1.5276878E38F, PH.base.pack) ;
    p108_ygyro_SET((float) -2.4479294E38F, PH.base.pack) ;
    p108_zgyro_SET((float)2.3379125E37F, PH.base.pack) ;
    p108_lat_SET((float)1.5238996E38F, PH.base.pack) ;
    p108_lon_SET((float) -4.477919E37F, PH.base.pack) ;
    p108_alt_SET((float)1.3025911E38F, PH.base.pack) ;
    p108_std_dev_horz_SET((float) -5.7076303E37F, PH.base.pack) ;
    p108_std_dev_vert_SET((float) -7.2598776E37F, PH.base.pack) ;
    p108_vn_SET((float)1.9180566E38F, PH.base.pack) ;
    p108_ve_SET((float)3.1716963E38F, PH.base.pack) ;
    p108_vd_SET((float) -3.238435E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
    p109_rssi_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
    p109_remrssi_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
    p109_txbuf_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
    p109_noise_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
    p109_remnoise_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
    p109_rxerrors_SET((uint16_t)(uint16_t)15484, PH.base.pack) ;
    p109_fixed__SET((uint16_t)(uint16_t)11887, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
    p110_target_network_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
    p110_target_system_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
    p110_target_component_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)22, (uint8_t)224, (uint8_t)246, (uint8_t)49, (uint8_t)183, (uint8_t)139, (uint8_t)178, (uint8_t)164, (uint8_t)255, (uint8_t)161, (uint8_t)168, (uint8_t)43, (uint8_t)250, (uint8_t)33, (uint8_t)143, (uint8_t)212, (uint8_t)76, (uint8_t)14, (uint8_t)26, (uint8_t)197, (uint8_t)26, (uint8_t)164, (uint8_t)192, (uint8_t)149, (uint8_t)99, (uint8_t)164, (uint8_t)181, (uint8_t)171, (uint8_t)125, (uint8_t)13, (uint8_t)14, (uint8_t)231, (uint8_t)50, (uint8_t)238, (uint8_t)32, (uint8_t)68, (uint8_t)247, (uint8_t)188, (uint8_t)64, (uint8_t)24, (uint8_t)97, (uint8_t)187, (uint8_t)76, (uint8_t)150, (uint8_t)182, (uint8_t)65, (uint8_t)205, (uint8_t)56, (uint8_t)102, (uint8_t)141, (uint8_t)165, (uint8_t)90, (uint8_t)88, (uint8_t)163, (uint8_t)179, (uint8_t)225, (uint8_t)114, (uint8_t)158, (uint8_t)81, (uint8_t)9, (uint8_t)159, (uint8_t)188, (uint8_t)218, (uint8_t)75, (uint8_t)224, (uint8_t)180, (uint8_t)255, (uint8_t)222, (uint8_t)214, (uint8_t)197, (uint8_t)199, (uint8_t)118, (uint8_t)67, (uint8_t)9, (uint8_t)138, (uint8_t)244, (uint8_t)215, (uint8_t)12, (uint8_t)176, (uint8_t)21, (uint8_t)8, (uint8_t)131, (uint8_t)237, (uint8_t)252, (uint8_t)26, (uint8_t)51, (uint8_t)177, (uint8_t)27, (uint8_t)176, (uint8_t)11, (uint8_t)3, (uint8_t)122, (uint8_t)240, (uint8_t)52, (uint8_t)168, (uint8_t)26, (uint8_t)186, (uint8_t)254, (uint8_t)186, (uint8_t)136, (uint8_t)127, (uint8_t)68, (uint8_t)31, (uint8_t)75, (uint8_t)62, (uint8_t)242, (uint8_t)94, (uint8_t)13, (uint8_t)236, (uint8_t)198, (uint8_t)59, (uint8_t)57, (uint8_t)224, (uint8_t)73, (uint8_t)217, (uint8_t)29, (uint8_t)84, (uint8_t)158, (uint8_t)216, (uint8_t)173, (uint8_t)26, (uint8_t)44, (uint8_t)162, (uint8_t)157, (uint8_t)9, (uint8_t)22, (uint8_t)45, (uint8_t)142, (uint8_t)160, (uint8_t)20, (uint8_t)241, (uint8_t)146, (uint8_t)43, (uint8_t)87, (uint8_t)205, (uint8_t)105, (uint8_t)222, (uint8_t)238, (uint8_t)226, (uint8_t)194, (uint8_t)245, (uint8_t)149, (uint8_t)70, (uint8_t)30, (uint8_t)238, (uint8_t)161, (uint8_t)112, (uint8_t)206, (uint8_t)92, (uint8_t)170, (uint8_t)133, (uint8_t)226, (uint8_t)88, (uint8_t)43, (uint8_t)210, (uint8_t)81, (uint8_t)238, (uint8_t)145, (uint8_t)38, (uint8_t)197, (uint8_t)160, (uint8_t)97, (uint8_t)192, (uint8_t)84, (uint8_t)230, (uint8_t)13, (uint8_t)115, (uint8_t)182, (uint8_t)78, (uint8_t)70, (uint8_t)191, (uint8_t)185, (uint8_t)54, (uint8_t)123, (uint8_t)215, (uint8_t)36, (uint8_t)197, (uint8_t)176, (uint8_t)112, (uint8_t)254, (uint8_t)15, (uint8_t)148, (uint8_t)202, (uint8_t)240, (uint8_t)170, (uint8_t)246, (uint8_t)170, (uint8_t)227, (uint8_t)249, (uint8_t)219, (uint8_t)88, (uint8_t)236, (uint8_t)39, (uint8_t)183, (uint8_t)29, (uint8_t)110, (uint8_t)138, (uint8_t)173, (uint8_t)11, (uint8_t)175, (uint8_t)75, (uint8_t)135, (uint8_t)6, (uint8_t)169, (uint8_t)231, (uint8_t)177, (uint8_t)140, (uint8_t)254, (uint8_t)144, (uint8_t)254, (uint8_t)236, (uint8_t)206, (uint8_t)129, (uint8_t)54, (uint8_t)105, (uint8_t)2, (uint8_t)64, (uint8_t)36, (uint8_t)251, (uint8_t)114, (uint8_t)53, (uint8_t)90, (uint8_t)254, (uint8_t)173, (uint8_t)112, (uint8_t)34, (uint8_t)56, (uint8_t)212, (uint8_t)146, (uint8_t)100, (uint8_t)252, (uint8_t)221, (uint8_t)55, (uint8_t)175, (uint8_t)174, (uint8_t)92, (uint8_t)191, (uint8_t)159, (uint8_t)32, (uint8_t)116, (uint8_t)10, (uint8_t)227, (uint8_t)170, (uint8_t)185, (uint8_t)15, (uint8_t)215, (uint8_t)96, (uint8_t)74, (uint8_t)13, (uint8_t)224, (uint8_t)211};
        p110_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
    p111_tc1_SET((int64_t)3406980552144625561L, PH.base.pack) ;
    p111_ts1_SET((int64_t)7848333924933577602L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
    p112_time_usec_SET((uint64_t)700085357346779116L, PH.base.pack) ;
    p112_seq_SET((uint32_t)1720998777L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
    p113_time_usec_SET((uint64_t)568979459583643150L, PH.base.pack) ;
    p113_fix_type_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
    p113_lat_SET((int32_t)1523159333, PH.base.pack) ;
    p113_lon_SET((int32_t) -410414474, PH.base.pack) ;
    p113_alt_SET((int32_t)1222354246, PH.base.pack) ;
    p113_eph_SET((uint16_t)(uint16_t)30887, PH.base.pack) ;
    p113_epv_SET((uint16_t)(uint16_t)54490, PH.base.pack) ;
    p113_vel_SET((uint16_t)(uint16_t)47226, PH.base.pack) ;
    p113_vn_SET((int16_t)(int16_t) -32302, PH.base.pack) ;
    p113_ve_SET((int16_t)(int16_t)24268, PH.base.pack) ;
    p113_vd_SET((int16_t)(int16_t) -10785, PH.base.pack) ;
    p113_cog_SET((uint16_t)(uint16_t)20955, PH.base.pack) ;
    p113_satellites_visible_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
    p114_time_usec_SET((uint64_t)4801543769942977155L, PH.base.pack) ;
    p114_sensor_id_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
    p114_integration_time_us_SET((uint32_t)3976994889L, PH.base.pack) ;
    p114_integrated_x_SET((float)1.3700044E38F, PH.base.pack) ;
    p114_integrated_y_SET((float) -3.344541E38F, PH.base.pack) ;
    p114_integrated_xgyro_SET((float) -1.9395685E38F, PH.base.pack) ;
    p114_integrated_ygyro_SET((float) -2.8699587E38F, PH.base.pack) ;
    p114_integrated_zgyro_SET((float) -1.6081801E38F, PH.base.pack) ;
    p114_temperature_SET((int16_t)(int16_t) -4649, PH.base.pack) ;
    p114_quality_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
    p114_time_delta_distance_us_SET((uint32_t)2822034908L, PH.base.pack) ;
    p114_distance_SET((float) -8.04168E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
    p115_time_usec_SET((uint64_t)7291950311689954763L, PH.base.pack) ;
    {
        float  attitude_quaternion [] =  {-1.7526513E38F, 1.045998E38F, -1.4985936E38F, -1.3045319E38F};
        p115_attitude_quaternion_SET(&attitude_quaternion, 0, &PH.base.pack) ;
    }
    p115_rollspeed_SET((float) -2.0376043E37F, PH.base.pack) ;
    p115_pitchspeed_SET((float) -8.811967E37F, PH.base.pack) ;
    p115_yawspeed_SET((float)1.3516292E38F, PH.base.pack) ;
    p115_lat_SET((int32_t) -1044380917, PH.base.pack) ;
    p115_lon_SET((int32_t) -160904718, PH.base.pack) ;
    p115_alt_SET((int32_t) -2122019151, PH.base.pack) ;
    p115_vx_SET((int16_t)(int16_t)2725, PH.base.pack) ;
    p115_vy_SET((int16_t)(int16_t)9, PH.base.pack) ;
    p115_vz_SET((int16_t)(int16_t)25122, PH.base.pack) ;
    p115_ind_airspeed_SET((uint16_t)(uint16_t)8885, PH.base.pack) ;
    p115_true_airspeed_SET((uint16_t)(uint16_t)30018, PH.base.pack) ;
    p115_xacc_SET((int16_t)(int16_t) -31940, PH.base.pack) ;
    p115_yacc_SET((int16_t)(int16_t) -6979, PH.base.pack) ;
    p115_zacc_SET((int16_t)(int16_t) -22895, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
    p116_time_boot_ms_SET((uint32_t)82101079L, PH.base.pack) ;
    p116_xacc_SET((int16_t)(int16_t) -7856, PH.base.pack) ;
    p116_yacc_SET((int16_t)(int16_t)19884, PH.base.pack) ;
    p116_zacc_SET((int16_t)(int16_t)14424, PH.base.pack) ;
    p116_xgyro_SET((int16_t)(int16_t)31194, PH.base.pack) ;
    p116_ygyro_SET((int16_t)(int16_t) -28173, PH.base.pack) ;
    p116_zgyro_SET((int16_t)(int16_t) -3151, PH.base.pack) ;
    p116_xmag_SET((int16_t)(int16_t) -31933, PH.base.pack) ;
    p116_ymag_SET((int16_t)(int16_t) -18253, PH.base.pack) ;
    p116_zmag_SET((int16_t)(int16_t)14654, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
    p117_target_system_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
    p117_target_component_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    p117_start_SET((uint16_t)(uint16_t)9074, PH.base.pack) ;
    p117_end_SET((uint16_t)(uint16_t)37170, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
    p118_id_SET((uint16_t)(uint16_t)18678, PH.base.pack) ;
    p118_num_logs_SET((uint16_t)(uint16_t)41743, PH.base.pack) ;
    p118_last_log_num_SET((uint16_t)(uint16_t)36272, PH.base.pack) ;
    p118_time_utc_SET((uint32_t)1016750272L, PH.base.pack) ;
    p118_size_SET((uint32_t)3978956184L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
    p119_target_system_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
    p119_target_component_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
    p119_id_SET((uint16_t)(uint16_t)7556, PH.base.pack) ;
    p119_ofs_SET((uint32_t)3843926765L, PH.base.pack) ;
    p119_count_SET((uint32_t)3194150411L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
    p120_id_SET((uint16_t)(uint16_t)15428, PH.base.pack) ;
    p120_ofs_SET((uint32_t)3254875042L, PH.base.pack) ;
    p120_count_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)23, (uint8_t)229, (uint8_t)120, (uint8_t)137, (uint8_t)170, (uint8_t)126, (uint8_t)219, (uint8_t)85, (uint8_t)194, (uint8_t)42, (uint8_t)190, (uint8_t)27, (uint8_t)42, (uint8_t)78, (uint8_t)148, (uint8_t)207, (uint8_t)171, (uint8_t)100, (uint8_t)36, (uint8_t)37, (uint8_t)74, (uint8_t)7, (uint8_t)243, (uint8_t)28, (uint8_t)206, (uint8_t)26, (uint8_t)84, (uint8_t)107, (uint8_t)195, (uint8_t)175, (uint8_t)76, (uint8_t)122, (uint8_t)88, (uint8_t)216, (uint8_t)243, (uint8_t)169, (uint8_t)188, (uint8_t)227, (uint8_t)65, (uint8_t)20, (uint8_t)123, (uint8_t)230, (uint8_t)84, (uint8_t)40, (uint8_t)15, (uint8_t)83, (uint8_t)25, (uint8_t)43, (uint8_t)112, (uint8_t)121, (uint8_t)93, (uint8_t)231, (uint8_t)193, (uint8_t)112, (uint8_t)138, (uint8_t)232, (uint8_t)18, (uint8_t)101, (uint8_t)172, (uint8_t)158, (uint8_t)226, (uint8_t)171, (uint8_t)151, (uint8_t)215, (uint8_t)221, (uint8_t)34, (uint8_t)95, (uint8_t)110, (uint8_t)70, (uint8_t)155, (uint8_t)43, (uint8_t)210, (uint8_t)17, (uint8_t)64, (uint8_t)28, (uint8_t)82, (uint8_t)120, (uint8_t)181, (uint8_t)74, (uint8_t)124, (uint8_t)15, (uint8_t)21, (uint8_t)124, (uint8_t)59, (uint8_t)204, (uint8_t)114, (uint8_t)227, (uint8_t)213, (uint8_t)66, (uint8_t)243};
        p120_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
    p121_target_system_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
    p121_target_component_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
    p122_target_system_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
    p122_target_component_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
    p123_target_system_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
    p123_target_component_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
    p123_len_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)88, (uint8_t)117, (uint8_t)223, (uint8_t)222, (uint8_t)212, (uint8_t)221, (uint8_t)38, (uint8_t)243, (uint8_t)222, (uint8_t)12, (uint8_t)120, (uint8_t)42, (uint8_t)46, (uint8_t)194, (uint8_t)83, (uint8_t)86, (uint8_t)195, (uint8_t)119, (uint8_t)209, (uint8_t)170, (uint8_t)92, (uint8_t)52, (uint8_t)146, (uint8_t)229, (uint8_t)41, (uint8_t)232, (uint8_t)47, (uint8_t)35, (uint8_t)228, (uint8_t)80, (uint8_t)119, (uint8_t)22, (uint8_t)42, (uint8_t)236, (uint8_t)12, (uint8_t)225, (uint8_t)23, (uint8_t)214, (uint8_t)191, (uint8_t)18, (uint8_t)136, (uint8_t)30, (uint8_t)148, (uint8_t)166, (uint8_t)15, (uint8_t)33, (uint8_t)140, (uint8_t)90, (uint8_t)214, (uint8_t)76, (uint8_t)174, (uint8_t)24, (uint8_t)138, (uint8_t)218, (uint8_t)207, (uint8_t)211, (uint8_t)199, (uint8_t)211, (uint8_t)128, (uint8_t)126, (uint8_t)75, (uint8_t)106, (uint8_t)117, (uint8_t)212, (uint8_t)129, (uint8_t)58, (uint8_t)22, (uint8_t)194, (uint8_t)155, (uint8_t)45, (uint8_t)69, (uint8_t)35, (uint8_t)179, (uint8_t)67, (uint8_t)168, (uint8_t)47, (uint8_t)32, (uint8_t)49, (uint8_t)159, (uint8_t)56, (uint8_t)171, (uint8_t)182, (uint8_t)192, (uint8_t)130, (uint8_t)253, (uint8_t)214, (uint8_t)11, (uint8_t)92, (uint8_t)201, (uint8_t)119, (uint8_t)183, (uint8_t)187, (uint8_t)160, (uint8_t)159, (uint8_t)88, (uint8_t)77, (uint8_t)162, (uint8_t)198, (uint8_t)250, (uint8_t)49, (uint8_t)121, (uint8_t)37, (uint8_t)66, (uint8_t)7, (uint8_t)98, (uint8_t)203, (uint8_t)89, (uint8_t)75, (uint8_t)167, (uint8_t)92};
        p123_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
    p124_time_usec_SET((uint64_t)5597161962191665228L, PH.base.pack) ;
    p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, PH.base.pack) ;
    p124_lat_SET((int32_t) -1822867429, PH.base.pack) ;
    p124_lon_SET((int32_t) -1104227962, PH.base.pack) ;
    p124_alt_SET((int32_t)1877719130, PH.base.pack) ;
    p124_eph_SET((uint16_t)(uint16_t)60453, PH.base.pack) ;
    p124_epv_SET((uint16_t)(uint16_t)36400, PH.base.pack) ;
    p124_vel_SET((uint16_t)(uint16_t)29975, PH.base.pack) ;
    p124_cog_SET((uint16_t)(uint16_t)54363, PH.base.pack) ;
    p124_satellites_visible_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
    p124_dgps_numch_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
    p124_dgps_age_SET((uint32_t)2391744303L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
    p125_Vcc_SET((uint16_t)(uint16_t)1847, PH.base.pack) ;
    p125_Vservo_SET((uint16_t)(uint16_t)14789, PH.base.pack) ;
    p125_flags_SET((e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT |
                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED), PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
    p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2, PH.base.pack) ;
    p126_flags_SET((e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING |
                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI |
                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND |
                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY), PH.base.pack) ;
    p126_timeout_SET((uint16_t)(uint16_t)6531, PH.base.pack) ;
    p126_baudrate_SET((uint32_t)1263880434L, PH.base.pack) ;
    p126_count_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)229, (uint8_t)63, (uint8_t)182, (uint8_t)47, (uint8_t)32, (uint8_t)232, (uint8_t)26, (uint8_t)225, (uint8_t)109, (uint8_t)180, (uint8_t)24, (uint8_t)155, (uint8_t)112, (uint8_t)159, (uint8_t)51, (uint8_t)55, (uint8_t)19, (uint8_t)223, (uint8_t)88, (uint8_t)169, (uint8_t)19, (uint8_t)62, (uint8_t)181, (uint8_t)244, (uint8_t)36, (uint8_t)206, (uint8_t)111, (uint8_t)184, (uint8_t)51, (uint8_t)67, (uint8_t)64, (uint8_t)52, (uint8_t)135, (uint8_t)65, (uint8_t)188, (uint8_t)136, (uint8_t)113, (uint8_t)240, (uint8_t)119, (uint8_t)197, (uint8_t)11, (uint8_t)107, (uint8_t)185, (uint8_t)139, (uint8_t)207, (uint8_t)250, (uint8_t)105, (uint8_t)163, (uint8_t)25, (uint8_t)213, (uint8_t)81, (uint8_t)91, (uint8_t)18, (uint8_t)2, (uint8_t)3, (uint8_t)195, (uint8_t)22, (uint8_t)224, (uint8_t)229, (uint8_t)13, (uint8_t)53, (uint8_t)248, (uint8_t)205, (uint8_t)188, (uint8_t)33, (uint8_t)3, (uint8_t)122, (uint8_t)233, (uint8_t)39, (uint8_t)155};
        p126_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
    p127_time_last_baseline_ms_SET((uint32_t)2319202448L, PH.base.pack) ;
    p127_rtk_receiver_id_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
    p127_wn_SET((uint16_t)(uint16_t)43695, PH.base.pack) ;
    p127_tow_SET((uint32_t)3089037463L, PH.base.pack) ;
    p127_rtk_health_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
    p127_rtk_rate_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
    p127_nsats_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
    p127_baseline_coords_type_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
    p127_baseline_a_mm_SET((int32_t)491096770, PH.base.pack) ;
    p127_baseline_b_mm_SET((int32_t) -1311861000, PH.base.pack) ;
    p127_baseline_c_mm_SET((int32_t)780786419, PH.base.pack) ;
    p127_accuracy_SET((uint32_t)3617569924L, PH.base.pack) ;
    p127_iar_num_hypotheses_SET((int32_t)446607269, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
    p128_time_last_baseline_ms_SET((uint32_t)2554954173L, PH.base.pack) ;
    p128_rtk_receiver_id_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
    p128_wn_SET((uint16_t)(uint16_t)40067, PH.base.pack) ;
    p128_tow_SET((uint32_t)4285387447L, PH.base.pack) ;
    p128_rtk_health_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
    p128_rtk_rate_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
    p128_nsats_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
    p128_baseline_coords_type_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
    p128_baseline_a_mm_SET((int32_t) -777007341, PH.base.pack) ;
    p128_baseline_b_mm_SET((int32_t)1128369286, PH.base.pack) ;
    p128_baseline_c_mm_SET((int32_t) -308644309, PH.base.pack) ;
    p128_accuracy_SET((uint32_t)1092021424L, PH.base.pack) ;
    p128_iar_num_hypotheses_SET((int32_t)1969939802, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
    p129_time_boot_ms_SET((uint32_t)4179798437L, PH.base.pack) ;
    p129_xacc_SET((int16_t)(int16_t)5512, PH.base.pack) ;
    p129_yacc_SET((int16_t)(int16_t)2743, PH.base.pack) ;
    p129_zacc_SET((int16_t)(int16_t) -24412, PH.base.pack) ;
    p129_xgyro_SET((int16_t)(int16_t) -25457, PH.base.pack) ;
    p129_ygyro_SET((int16_t)(int16_t)16156, PH.base.pack) ;
    p129_zgyro_SET((int16_t)(int16_t) -14172, PH.base.pack) ;
    p129_xmag_SET((int16_t)(int16_t) -972, PH.base.pack) ;
    p129_ymag_SET((int16_t)(int16_t)13245, PH.base.pack) ;
    p129_zmag_SET((int16_t)(int16_t)3185, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
    p130_type_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
    p130_size_SET((uint32_t)3965457199L, PH.base.pack) ;
    p130_width_SET((uint16_t)(uint16_t)56872, PH.base.pack) ;
    p130_height_SET((uint16_t)(uint16_t)30580, PH.base.pack) ;
    p130_packets_SET((uint16_t)(uint16_t)58517, PH.base.pack) ;
    p130_payload_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
    p130_jpg_quality_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
    p131_seqnr_SET((uint16_t)(uint16_t)64282, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)49, (uint8_t)151, (uint8_t)49, (uint8_t)66, (uint8_t)84, (uint8_t)152, (uint8_t)52, (uint8_t)246, (uint8_t)165, (uint8_t)46, (uint8_t)116, (uint8_t)123, (uint8_t)119, (uint8_t)136, (uint8_t)36, (uint8_t)147, (uint8_t)234, (uint8_t)22, (uint8_t)164, (uint8_t)207, (uint8_t)249, (uint8_t)21, (uint8_t)212, (uint8_t)255, (uint8_t)53, (uint8_t)63, (uint8_t)197, (uint8_t)172, (uint8_t)229, (uint8_t)92, (uint8_t)167, (uint8_t)221, (uint8_t)10, (uint8_t)71, (uint8_t)91, (uint8_t)125, (uint8_t)12, (uint8_t)43, (uint8_t)139, (uint8_t)89, (uint8_t)109, (uint8_t)167, (uint8_t)99, (uint8_t)106, (uint8_t)79, (uint8_t)62, (uint8_t)48, (uint8_t)116, (uint8_t)127, (uint8_t)146, (uint8_t)179, (uint8_t)220, (uint8_t)177, (uint8_t)143, (uint8_t)191, (uint8_t)174, (uint8_t)25, (uint8_t)105, (uint8_t)142, (uint8_t)57, (uint8_t)232, (uint8_t)19, (uint8_t)179, (uint8_t)35, (uint8_t)29, (uint8_t)10, (uint8_t)189, (uint8_t)28, (uint8_t)182, (uint8_t)169, (uint8_t)49, (uint8_t)100, (uint8_t)248, (uint8_t)25, (uint8_t)63, (uint8_t)76, (uint8_t)29, (uint8_t)195, (uint8_t)60, (uint8_t)3, (uint8_t)153, (uint8_t)255, (uint8_t)225, (uint8_t)65, (uint8_t)114, (uint8_t)226, (uint8_t)253, (uint8_t)10, (uint8_t)31, (uint8_t)104, (uint8_t)230, (uint8_t)170, (uint8_t)213, (uint8_t)18, (uint8_t)26, (uint8_t)121, (uint8_t)213, (uint8_t)90, (uint8_t)96, (uint8_t)70, (uint8_t)210, (uint8_t)167, (uint8_t)235, (uint8_t)122, (uint8_t)223, (uint8_t)48, (uint8_t)22, (uint8_t)24, (uint8_t)200, (uint8_t)7, (uint8_t)21, (uint8_t)252, (uint8_t)103, (uint8_t)245, (uint8_t)145, (uint8_t)32, (uint8_t)99, (uint8_t)43, (uint8_t)209, (uint8_t)42, (uint8_t)25, (uint8_t)54, (uint8_t)64, (uint8_t)129, (uint8_t)33, (uint8_t)16, (uint8_t)159, (uint8_t)170, (uint8_t)213, (uint8_t)147, (uint8_t)144, (uint8_t)79, (uint8_t)178, (uint8_t)160, (uint8_t)1, (uint8_t)3, (uint8_t)125, (uint8_t)203, (uint8_t)148, (uint8_t)115, (uint8_t)145, (uint8_t)250, (uint8_t)207, (uint8_t)154, (uint8_t)62, (uint8_t)43, (uint8_t)62, (uint8_t)163, (uint8_t)21, (uint8_t)191, (uint8_t)86, (uint8_t)42, (uint8_t)129, (uint8_t)137, (uint8_t)2, (uint8_t)221, (uint8_t)253, (uint8_t)28, (uint8_t)170, (uint8_t)79, (uint8_t)255, (uint8_t)3, (uint8_t)143, (uint8_t)144, (uint8_t)204, (uint8_t)183, (uint8_t)85, (uint8_t)178, (uint8_t)227, (uint8_t)45, (uint8_t)197, (uint8_t)34, (uint8_t)31, (uint8_t)174, (uint8_t)141, (uint8_t)96, (uint8_t)175, (uint8_t)31, (uint8_t)95, (uint8_t)204, (uint8_t)252, (uint8_t)20, (uint8_t)115, (uint8_t)101, (uint8_t)37, (uint8_t)99, (uint8_t)177, (uint8_t)181, (uint8_t)32, (uint8_t)89, (uint8_t)201, (uint8_t)192, (uint8_t)147, (uint8_t)138, (uint8_t)48, (uint8_t)22, (uint8_t)178, (uint8_t)76, (uint8_t)24, (uint8_t)72, (uint8_t)206, (uint8_t)28, (uint8_t)134, (uint8_t)147, (uint8_t)140, (uint8_t)203, (uint8_t)34, (uint8_t)13, (uint8_t)49, (uint8_t)230, (uint8_t)195, (uint8_t)206, (uint8_t)62, (uint8_t)41, (uint8_t)176, (uint8_t)160, (uint8_t)105, (uint8_t)251, (uint8_t)46, (uint8_t)45, (uint8_t)38, (uint8_t)159, (uint8_t)209, (uint8_t)151, (uint8_t)196, (uint8_t)82, (uint8_t)147, (uint8_t)193, (uint8_t)106, (uint8_t)238, (uint8_t)74, (uint8_t)50, (uint8_t)184, (uint8_t)158, (uint8_t)241, (uint8_t)5, (uint8_t)255, (uint8_t)3, (uint8_t)114, (uint8_t)163, (uint8_t)204, (uint8_t)99, (uint8_t)28, (uint8_t)135, (uint8_t)49, (uint8_t)117, (uint8_t)70, (uint8_t)125, (uint8_t)229, (uint8_t)85, (uint8_t)156, (uint8_t)234, (uint8_t)93};
        p131_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
    p132_time_boot_ms_SET((uint32_t)701383327L, PH.base.pack) ;
    p132_min_distance_SET((uint16_t)(uint16_t)45179, PH.base.pack) ;
    p132_max_distance_SET((uint16_t)(uint16_t)18648, PH.base.pack) ;
    p132_current_distance_SET((uint16_t)(uint16_t)16077, PH.base.pack) ;
    p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, PH.base.pack) ;
    p132_id_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
    p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_PITCH_90, PH.base.pack) ;
    p132_covariance_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
    p133_lat_SET((int32_t) -216430068, PH.base.pack) ;
    p133_lon_SET((int32_t) -2101876389, PH.base.pack) ;
    p133_grid_spacing_SET((uint16_t)(uint16_t)7931, PH.base.pack) ;
    p133_mask_SET((uint64_t)9209935700882852531L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
    p134_lat_SET((int32_t)866101655, PH.base.pack) ;
    p134_lon_SET((int32_t) -1153111029, PH.base.pack) ;
    p134_grid_spacing_SET((uint16_t)(uint16_t)49074, PH.base.pack) ;
    p134_gridbit_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    {
        int16_t  data_ [] =  {(int16_t) -5839, (int16_t)1670, (int16_t) -20598, (int16_t) -16430, (int16_t)16019, (int16_t) -11568, (int16_t)32723, (int16_t)16214, (int16_t) -23418, (int16_t) -18265, (int16_t) -8748, (int16_t)27810, (int16_t)15177, (int16_t) -5779, (int16_t) -27901, (int16_t) -6946};
        p134_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
    p135_lat_SET((int32_t)1608754866, PH.base.pack) ;
    p135_lon_SET((int32_t) -611225574, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
    p136_lat_SET((int32_t)727269216, PH.base.pack) ;
    p136_lon_SET((int32_t) -1367470104, PH.base.pack) ;
    p136_spacing_SET((uint16_t)(uint16_t)63062, PH.base.pack) ;
    p136_terrain_height_SET((float)3.14061E38F, PH.base.pack) ;
    p136_current_height_SET((float)1.2361206E38F, PH.base.pack) ;
    p136_pending_SET((uint16_t)(uint16_t)43224, PH.base.pack) ;
    p136_loaded_SET((uint16_t)(uint16_t)46129, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
    p137_time_boot_ms_SET((uint32_t)2013532653L, PH.base.pack) ;
    p137_press_abs_SET((float)3.4771022E37F, PH.base.pack) ;
    p137_press_diff_SET((float) -1.9422064E38F, PH.base.pack) ;
    p137_temperature_SET((int16_t)(int16_t)32284, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
    p138_time_usec_SET((uint64_t)7919872639335755428L, PH.base.pack) ;
    {
        float  q [] =  {-1.3148211E38F, -1.4204649E38F, -1.6370736E38F, 1.0672911E38F};
        p138_q_SET(&q, 0, &PH.base.pack) ;
    }
    p138_x_SET((float) -2.7849833E38F, PH.base.pack) ;
    p138_y_SET((float)2.8755995E37F, PH.base.pack) ;
    p138_z_SET((float) -2.3960115E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
    p139_time_usec_SET((uint64_t)6329583534722735841L, PH.base.pack) ;
    p139_group_mlx_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
    p139_target_system_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    p139_target_component_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
    {
        float  controls [] =  {-1.9799944E38F, -2.9104954E38F, 5.544335E36F, -3.3589838E38F, -2.2594164E38F, 1.4713998E38F, -3.1122895E38F, 8.519906E37F};
        p139_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
    p140_time_usec_SET((uint64_t)3607777630869343378L, PH.base.pack) ;
    p140_group_mlx_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
    {
        float  controls [] =  {5.5660084E37F, 2.389723E38F, 3.1703828E38F, -3.252407E38F, 3.1656698E38F, -8.669258E37F, 1.6568046E38F, 2.2049358E38F};
        p140_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ALTITUDE_141(), &PH);
    p141_time_usec_SET((uint64_t)8110277072275527029L, PH.base.pack) ;
    p141_altitude_monotonic_SET((float) -9.776503E36F, PH.base.pack) ;
    p141_altitude_amsl_SET((float) -1.0378495E38F, PH.base.pack) ;
    p141_altitude_local_SET((float) -2.9327705E38F, PH.base.pack) ;
    p141_altitude_relative_SET((float)1.619941E37F, PH.base.pack) ;
    p141_altitude_terrain_SET((float) -2.8749476E38F, PH.base.pack) ;
    p141_bottom_clearance_SET((float)2.8420637E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RESOURCE_REQUEST_142(), &PH);
    p142_request_id_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
    p142_uri_type_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
    {
        uint8_t  uri [] =  {(uint8_t)8, (uint8_t)159, (uint8_t)97, (uint8_t)167, (uint8_t)46, (uint8_t)138, (uint8_t)143, (uint8_t)200, (uint8_t)177, (uint8_t)127, (uint8_t)234, (uint8_t)59, (uint8_t)96, (uint8_t)82, (uint8_t)8, (uint8_t)200, (uint8_t)132, (uint8_t)101, (uint8_t)173, (uint8_t)21, (uint8_t)222, (uint8_t)240, (uint8_t)187, (uint8_t)164, (uint8_t)66, (uint8_t)245, (uint8_t)158, (uint8_t)195, (uint8_t)9, (uint8_t)84, (uint8_t)85, (uint8_t)151, (uint8_t)101, (uint8_t)164, (uint8_t)235, (uint8_t)14, (uint8_t)146, (uint8_t)167, (uint8_t)242, (uint8_t)94, (uint8_t)59, (uint8_t)89, (uint8_t)254, (uint8_t)193, (uint8_t)201, (uint8_t)54, (uint8_t)60, (uint8_t)185, (uint8_t)248, (uint8_t)15, (uint8_t)168, (uint8_t)51, (uint8_t)82, (uint8_t)197, (uint8_t)189, (uint8_t)27, (uint8_t)118, (uint8_t)92, (uint8_t)243, (uint8_t)182, (uint8_t)229, (uint8_t)227, (uint8_t)120, (uint8_t)224, (uint8_t)191, (uint8_t)183, (uint8_t)161, (uint8_t)108, (uint8_t)241, (uint8_t)253, (uint8_t)29, (uint8_t)80, (uint8_t)190, (uint8_t)23, (uint8_t)167, (uint8_t)229, (uint8_t)173, (uint8_t)255, (uint8_t)112, (uint8_t)193, (uint8_t)249, (uint8_t)127, (uint8_t)178, (uint8_t)171, (uint8_t)118, (uint8_t)18, (uint8_t)4, (uint8_t)249, (uint8_t)224, (uint8_t)157, (uint8_t)0, (uint8_t)55, (uint8_t)130, (uint8_t)56, (uint8_t)25, (uint8_t)47, (uint8_t)143, (uint8_t)131, (uint8_t)55, (uint8_t)176, (uint8_t)105, (uint8_t)177, (uint8_t)69, (uint8_t)238, (uint8_t)158, (uint8_t)55, (uint8_t)9, (uint8_t)71, (uint8_t)225, (uint8_t)214, (uint8_t)5, (uint8_t)165, (uint8_t)109, (uint8_t)255, (uint8_t)204, (uint8_t)168, (uint8_t)91, (uint8_t)206, (uint8_t)62, (uint8_t)36};
        p142_uri_SET(&uri, 0, &PH.base.pack) ;
    }
    p142_transfer_type_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
    {
        uint8_t  storage [] =  {(uint8_t)113, (uint8_t)37, (uint8_t)59, (uint8_t)235, (uint8_t)225, (uint8_t)39, (uint8_t)40, (uint8_t)228, (uint8_t)65, (uint8_t)219, (uint8_t)238, (uint8_t)91, (uint8_t)219, (uint8_t)41, (uint8_t)82, (uint8_t)64, (uint8_t)239, (uint8_t)58, (uint8_t)251, (uint8_t)105, (uint8_t)211, (uint8_t)157, (uint8_t)180, (uint8_t)0, (uint8_t)218, (uint8_t)218, (uint8_t)120, (uint8_t)6, (uint8_t)29, (uint8_t)100, (uint8_t)248, (uint8_t)240, (uint8_t)188, (uint8_t)253, (uint8_t)30, (uint8_t)219, (uint8_t)87, (uint8_t)90, (uint8_t)137, (uint8_t)228, (uint8_t)80, (uint8_t)1, (uint8_t)95, (uint8_t)0, (uint8_t)146, (uint8_t)202, (uint8_t)39, (uint8_t)44, (uint8_t)221, (uint8_t)162, (uint8_t)201, (uint8_t)47, (uint8_t)120, (uint8_t)239, (uint8_t)39, (uint8_t)253, (uint8_t)244, (uint8_t)149, (uint8_t)118, (uint8_t)199, (uint8_t)48, (uint8_t)142, (uint8_t)46, (uint8_t)29, (uint8_t)6, (uint8_t)78, (uint8_t)13, (uint8_t)43, (uint8_t)68, (uint8_t)82, (uint8_t)124, (uint8_t)191, (uint8_t)157, (uint8_t)67, (uint8_t)20, (uint8_t)9, (uint8_t)117, (uint8_t)203, (uint8_t)232, (uint8_t)108, (uint8_t)1, (uint8_t)171, (uint8_t)79, (uint8_t)173, (uint8_t)168, (uint8_t)114, (uint8_t)15, (uint8_t)144, (uint8_t)21, (uint8_t)211, (uint8_t)51, (uint8_t)101, (uint8_t)102, (uint8_t)44, (uint8_t)151, (uint8_t)196, (uint8_t)143, (uint8_t)210, (uint8_t)176, (uint8_t)17, (uint8_t)57, (uint8_t)204, (uint8_t)11, (uint8_t)65, (uint8_t)60, (uint8_t)82, (uint8_t)164, (uint8_t)200, (uint8_t)210, (uint8_t)136, (uint8_t)52, (uint8_t)35, (uint8_t)159, (uint8_t)40, (uint8_t)149, (uint8_t)91, (uint8_t)178, (uint8_t)200, (uint8_t)2, (uint8_t)21};
        p142_storage_SET(&storage, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE3_143(), &PH);
    p143_time_boot_ms_SET((uint32_t)998804943L, PH.base.pack) ;
    p143_press_abs_SET((float)2.7525952E38F, PH.base.pack) ;
    p143_press_diff_SET((float) -1.8847974E37F, PH.base.pack) ;
    p143_temperature_SET((int16_t)(int16_t)1724, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_FOLLOW_TARGET_144(), &PH);
    p144_timestamp_SET((uint64_t)2733348737661843024L, PH.base.pack) ;
    p144_est_capabilities_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
    p144_lat_SET((int32_t)2146025716, PH.base.pack) ;
    p144_lon_SET((int32_t)1099052062, PH.base.pack) ;
    p144_alt_SET((float)1.5056402E38F, PH.base.pack) ;
    {
        float  vel [] =  {1.784827E38F, -1.9759852E38F, -2.6606175E38F};
        p144_vel_SET(&vel, 0, &PH.base.pack) ;
    }
    {
        float  acc [] =  {-1.0624228E38F, 1.583264E38F, -1.959818E38F};
        p144_acc_SET(&acc, 0, &PH.base.pack) ;
    }
    {
        float  attitude_q [] =  {-4.2410298E37F, 8.850066E37F, 3.1605817E38F, 3.2311994E38F};
        p144_attitude_q_SET(&attitude_q, 0, &PH.base.pack) ;
    }
    {
        float  rates [] =  {3.2695287E37F, -2.8716649E38F, -3.0352368E38F};
        p144_rates_SET(&rates, 0, &PH.base.pack) ;
    }
    {
        float  position_cov [] =  {3.1095396E38F, 4.4517714E37F, 7.3331823E37F};
        p144_position_cov_SET(&position_cov, 0, &PH.base.pack) ;
    }
    p144_custom_state_SET((uint64_t)34038198585572781L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
    p146_time_usec_SET((uint64_t)3575086916140010366L, PH.base.pack) ;
    p146_x_acc_SET((float) -1.7289294E38F, PH.base.pack) ;
    p146_y_acc_SET((float) -1.4231078E38F, PH.base.pack) ;
    p146_z_acc_SET((float)3.5582374E37F, PH.base.pack) ;
    p146_x_vel_SET((float) -2.4816459E37F, PH.base.pack) ;
    p146_y_vel_SET((float)1.0871301E38F, PH.base.pack) ;
    p146_z_vel_SET((float) -1.4834202E38F, PH.base.pack) ;
    p146_x_pos_SET((float) -2.8880271E38F, PH.base.pack) ;
    p146_y_pos_SET((float)9.356439E37F, PH.base.pack) ;
    p146_z_pos_SET((float) -1.4007183E38F, PH.base.pack) ;
    p146_airspeed_SET((float)1.0217885E38F, PH.base.pack) ;
    {
        float  vel_variance [] =  {3.0395081E38F, 2.0536222E38F, 2.1399999E38F};
        p146_vel_variance_SET(&vel_variance, 0, &PH.base.pack) ;
    }
    {
        float  pos_variance [] =  {4.1525847E36F, -2.0835813E38F, -3.2719638E38F};
        p146_pos_variance_SET(&pos_variance, 0, &PH.base.pack) ;
    }
    {
        float  q [] =  {-4.4354486E37F, -1.2796116E38F, 9.47666E37F, -1.3192401E38F};
        p146_q_SET(&q, 0, &PH.base.pack) ;
    }
    p146_roll_rate_SET((float)1.4638666E38F, PH.base.pack) ;
    p146_pitch_rate_SET((float)9.917451E37F, PH.base.pack) ;
    p146_yaw_rate_SET((float) -2.493363E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_BATTERY_STATUS_147(), &PH);
    p147_id_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
    p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION, PH.base.pack) ;
    p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIFE, PH.base.pack) ;
    p147_temperature_SET((int16_t)(int16_t) -24464, PH.base.pack) ;
    {
        uint16_t  voltages [] =  {(uint16_t)480, (uint16_t)35788, (uint16_t)32999, (uint16_t)331, (uint16_t)39527, (uint16_t)57118, (uint16_t)30780, (uint16_t)15384, (uint16_t)1661, (uint16_t)51272};
        p147_voltages_SET(&voltages, 0, &PH.base.pack) ;
    }
    p147_current_battery_SET((int16_t)(int16_t)22742, PH.base.pack) ;
    p147_current_consumed_SET((int32_t)1608819754, PH.base.pack) ;
    p147_energy_consumed_SET((int32_t) -1314307783, PH.base.pack) ;
    p147_battery_remaining_SET((int8_t)(int8_t)109, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_AUTOPILOT_VERSION_148(), &PH);
    p148_capabilities_SET((e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT), PH.base.pack) ;
    p148_flight_sw_version_SET((uint32_t)3038752402L, PH.base.pack) ;
    p148_middleware_sw_version_SET((uint32_t)1559407936L, PH.base.pack) ;
    p148_os_sw_version_SET((uint32_t)1562507562L, PH.base.pack) ;
    p148_board_version_SET((uint32_t)333199365L, PH.base.pack) ;
    {
        uint8_t  flight_custom_version [] =  {(uint8_t)100, (uint8_t)121, (uint8_t)202, (uint8_t)64, (uint8_t)118, (uint8_t)25, (uint8_t)230, (uint8_t)107};
        p148_flight_custom_version_SET(&flight_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  middleware_custom_version [] =  {(uint8_t)181, (uint8_t)244, (uint8_t)32, (uint8_t)248, (uint8_t)237, (uint8_t)235, (uint8_t)42, (uint8_t)176};
        p148_middleware_custom_version_SET(&middleware_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  os_custom_version [] =  {(uint8_t)197, (uint8_t)132, (uint8_t)251, (uint8_t)80, (uint8_t)165, (uint8_t)71, (uint8_t)121, (uint8_t)59};
        p148_os_custom_version_SET(&os_custom_version, 0, &PH.base.pack) ;
    }
    p148_vendor_id_SET((uint16_t)(uint16_t)48732, PH.base.pack) ;
    p148_product_id_SET((uint16_t)(uint16_t)60656, PH.base.pack) ;
    p148_uid_SET((uint64_t)4863708006280241260L, PH.base.pack) ;
    {
        uint8_t  uid2 [] =  {(uint8_t)219, (uint8_t)25, (uint8_t)17, (uint8_t)17, (uint8_t)119, (uint8_t)255, (uint8_t)249, (uint8_t)114, (uint8_t)238, (uint8_t)26, (uint8_t)34, (uint8_t)115, (uint8_t)88, (uint8_t)186, (uint8_t)141, (uint8_t)4, (uint8_t)207, (uint8_t)46};
        p148_uid2_SET(&uid2, 0,  &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LANDING_TARGET_149(), &PH);
    p149_time_usec_SET((uint64_t)8887401548408321917L, PH.base.pack) ;
    p149_target_num_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
    p149_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
    p149_angle_x_SET((float) -2.1663548E38F, PH.base.pack) ;
    p149_angle_y_SET((float) -5.532887E37F, PH.base.pack) ;
    p149_distance_SET((float) -8.496534E36F, PH.base.pack) ;
    p149_size_x_SET((float)1.5468967E38F, PH.base.pack) ;
    p149_size_y_SET((float) -3.015083E38F, PH.base.pack) ;
    p149_x_SET((float) -7.611091E37F, &PH) ;
    p149_y_SET((float) -2.2104376E38F, &PH) ;
    p149_z_SET((float) -1.4761529E38F, &PH) ;
    {
        float  q [] =  {-1.958877E38F, -1.9239979E38F, -3.080201E38F, -3.947018E37F};
        p149_q_SET(&q, 0,  &PH) ;
    }
    p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON, PH.base.pack) ;
    p149_position_valid_SET((uint8_t)(uint8_t)4, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ESTIMATOR_STATUS_230(), &PH);
    p230_time_usec_SET((uint64_t)9159429352946754881L, PH.base.pack) ;
    p230_flags_SET((e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE |
                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_REL |
                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS), PH.base.pack) ;
    p230_vel_ratio_SET((float) -1.9261545E38F, PH.base.pack) ;
    p230_pos_horiz_ratio_SET((float) -2.8816167E38F, PH.base.pack) ;
    p230_pos_vert_ratio_SET((float)1.3135184E38F, PH.base.pack) ;
    p230_mag_ratio_SET((float)2.9661377E38F, PH.base.pack) ;
    p230_hagl_ratio_SET((float)2.5164988E38F, PH.base.pack) ;
    p230_tas_ratio_SET((float) -2.0429765E38F, PH.base.pack) ;
    p230_pos_horiz_accuracy_SET((float) -3.0897446E38F, PH.base.pack) ;
    p230_pos_vert_accuracy_SET((float)1.7701735E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_WIND_COV_231(), &PH);
    p231_time_usec_SET((uint64_t)8935338664551539947L, PH.base.pack) ;
    p231_wind_x_SET((float) -1.2663966E38F, PH.base.pack) ;
    p231_wind_y_SET((float) -2.135001E38F, PH.base.pack) ;
    p231_wind_z_SET((float) -2.0215642E38F, PH.base.pack) ;
    p231_var_horiz_SET((float)4.5955544E37F, PH.base.pack) ;
    p231_var_vert_SET((float) -1.831241E38F, PH.base.pack) ;
    p231_wind_alt_SET((float)1.8610616E38F, PH.base.pack) ;
    p231_horiz_accuracy_SET((float)1.826334E38F, PH.base.pack) ;
    p231_vert_accuracy_SET((float)3.032302E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_INPUT_232(), &PH);
    p232_time_usec_SET((uint64_t)765718174782711338L, PH.base.pack) ;
    p232_gps_id_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
    p232_ignore_flags_SET((e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY), PH.base.pack) ;
    p232_time_week_ms_SET((uint32_t)1191902293L, PH.base.pack) ;
    p232_time_week_SET((uint16_t)(uint16_t)19944, PH.base.pack) ;
    p232_fix_type_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
    p232_lat_SET((int32_t) -1608975769, PH.base.pack) ;
    p232_lon_SET((int32_t) -971905598, PH.base.pack) ;
    p232_alt_SET((float) -1.7722463E38F, PH.base.pack) ;
    p232_hdop_SET((float) -9.139829E37F, PH.base.pack) ;
    p232_vdop_SET((float) -3.3709833E38F, PH.base.pack) ;
    p232_vn_SET((float) -1.8965227E38F, PH.base.pack) ;
    p232_ve_SET((float)2.1818557E38F, PH.base.pack) ;
    p232_vd_SET((float)1.9044552E38F, PH.base.pack) ;
    p232_speed_accuracy_SET((float)3.8327337E37F, PH.base.pack) ;
    p232_horiz_accuracy_SET((float) -2.769369E38F, PH.base.pack) ;
    p232_vert_accuracy_SET((float) -2.892285E38F, PH.base.pack) ;
    p232_satellites_visible_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}
