
#include "MicroAirVehicle.h"
void failure(Channel * ch, int32_t id, int32_t arg) {}
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
void c_CommunicationChannel_on_FLEXIFUNCTION_SET_150(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p150_target_system_GET(pack);
    uint8_t  target_component = p150_target_component_GET(pack);
}
void c_CommunicationChannel_on_FLEXIFUNCTION_READ_REQ_151(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p151_target_system_GET(pack);
    uint8_t  target_component = p151_target_component_GET(pack);
    int16_t  read_req_type = p151_read_req_type_GET(pack);
    int16_t  data_index = p151_data_index_GET(pack);
}
void c_CommunicationChannel_on_FLEXIFUNCTION_BUFFER_FUNCTION_152(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p152_target_system_GET(pack);
    uint8_t  target_component = p152_target_component_GET(pack);
    uint16_t  func_index = p152_func_index_GET(pack);
    uint16_t  func_count = p152_func_count_GET(pack);
    uint16_t  data_address = p152_data_address_GET(pack);
    uint16_t  data_size = p152_data_size_GET(pack);
    int8_t*  data_ = p152_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_153(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p153_target_system_GET(pack);
    uint8_t  target_component = p153_target_component_GET(pack);
    uint16_t  func_index = p153_func_index_GET(pack);
    uint16_t  result = p153_result_GET(pack);
}
void c_CommunicationChannel_on_FLEXIFUNCTION_DIRECTORY_155(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p155_target_system_GET(pack);
    uint8_t  target_component = p155_target_component_GET(pack);
    uint8_t  directory_type = p155_directory_type_GET(pack);
    uint8_t  start_index = p155_start_index_GET(pack);
    uint8_t  count = p155_count_GET(pack);
    int8_t*  directory_data = p155_directory_data_GET_(pack);
//process data in directory_data
    free(directory_data);//never forget to dispose
}
void c_CommunicationChannel_on_FLEXIFUNCTION_DIRECTORY_ACK_156(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p156_target_system_GET(pack);
    uint8_t  target_component = p156_target_component_GET(pack);
    uint8_t  directory_type = p156_directory_type_GET(pack);
    uint8_t  start_index = p156_start_index_GET(pack);
    uint8_t  count = p156_count_GET(pack);
    uint16_t  result = p156_result_GET(pack);
}
void c_CommunicationChannel_on_FLEXIFUNCTION_COMMAND_157(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p157_target_system_GET(pack);
    uint8_t  target_component = p157_target_component_GET(pack);
    uint8_t  command_type = p157_command_type_GET(pack);
}
void c_CommunicationChannel_on_FLEXIFUNCTION_COMMAND_ACK_158(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  command_type = p158_command_type_GET(pack);
    uint16_t  result = p158_result_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F2_A_170(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  sue_time = p170_sue_time_GET(pack);
    uint8_t  sue_status = p170_sue_status_GET(pack);
    int32_t  sue_latitude = p170_sue_latitude_GET(pack);
    int32_t  sue_longitude = p170_sue_longitude_GET(pack);
    int32_t  sue_altitude = p170_sue_altitude_GET(pack);
    uint16_t  sue_waypoint_index = p170_sue_waypoint_index_GET(pack);
    int16_t  sue_rmat0 = p170_sue_rmat0_GET(pack);
    int16_t  sue_rmat1 = p170_sue_rmat1_GET(pack);
    int16_t  sue_rmat2 = p170_sue_rmat2_GET(pack);
    int16_t  sue_rmat3 = p170_sue_rmat3_GET(pack);
    int16_t  sue_rmat4 = p170_sue_rmat4_GET(pack);
    int16_t  sue_rmat5 = p170_sue_rmat5_GET(pack);
    int16_t  sue_rmat6 = p170_sue_rmat6_GET(pack);
    int16_t  sue_rmat7 = p170_sue_rmat7_GET(pack);
    int16_t  sue_rmat8 = p170_sue_rmat8_GET(pack);
    uint16_t  sue_cog = p170_sue_cog_GET(pack);
    int16_t  sue_sog = p170_sue_sog_GET(pack);
    uint16_t  sue_cpu_load = p170_sue_cpu_load_GET(pack);
    uint16_t  sue_air_speed_3DIMU = p170_sue_air_speed_3DIMU_GET(pack);
    int16_t  sue_estimated_wind_0 = p170_sue_estimated_wind_0_GET(pack);
    int16_t  sue_estimated_wind_1 = p170_sue_estimated_wind_1_GET(pack);
    int16_t  sue_estimated_wind_2 = p170_sue_estimated_wind_2_GET(pack);
    int16_t  sue_magFieldEarth0 = p170_sue_magFieldEarth0_GET(pack);
    int16_t  sue_magFieldEarth1 = p170_sue_magFieldEarth1_GET(pack);
    int16_t  sue_magFieldEarth2 = p170_sue_magFieldEarth2_GET(pack);
    int16_t  sue_svs = p170_sue_svs_GET(pack);
    int16_t  sue_hdop = p170_sue_hdop_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F2_B_171(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  sue_time = p171_sue_time_GET(pack);
    int16_t  sue_pwm_input_1 = p171_sue_pwm_input_1_GET(pack);
    int16_t  sue_pwm_input_2 = p171_sue_pwm_input_2_GET(pack);
    int16_t  sue_pwm_input_3 = p171_sue_pwm_input_3_GET(pack);
    int16_t  sue_pwm_input_4 = p171_sue_pwm_input_4_GET(pack);
    int16_t  sue_pwm_input_5 = p171_sue_pwm_input_5_GET(pack);
    int16_t  sue_pwm_input_6 = p171_sue_pwm_input_6_GET(pack);
    int16_t  sue_pwm_input_7 = p171_sue_pwm_input_7_GET(pack);
    int16_t  sue_pwm_input_8 = p171_sue_pwm_input_8_GET(pack);
    int16_t  sue_pwm_input_9 = p171_sue_pwm_input_9_GET(pack);
    int16_t  sue_pwm_input_10 = p171_sue_pwm_input_10_GET(pack);
    int16_t  sue_pwm_input_11 = p171_sue_pwm_input_11_GET(pack);
    int16_t  sue_pwm_input_12 = p171_sue_pwm_input_12_GET(pack);
    int16_t  sue_pwm_output_1 = p171_sue_pwm_output_1_GET(pack);
    int16_t  sue_pwm_output_2 = p171_sue_pwm_output_2_GET(pack);
    int16_t  sue_pwm_output_3 = p171_sue_pwm_output_3_GET(pack);
    int16_t  sue_pwm_output_4 = p171_sue_pwm_output_4_GET(pack);
    int16_t  sue_pwm_output_5 = p171_sue_pwm_output_5_GET(pack);
    int16_t  sue_pwm_output_6 = p171_sue_pwm_output_6_GET(pack);
    int16_t  sue_pwm_output_7 = p171_sue_pwm_output_7_GET(pack);
    int16_t  sue_pwm_output_8 = p171_sue_pwm_output_8_GET(pack);
    int16_t  sue_pwm_output_9 = p171_sue_pwm_output_9_GET(pack);
    int16_t  sue_pwm_output_10 = p171_sue_pwm_output_10_GET(pack);
    int16_t  sue_pwm_output_11 = p171_sue_pwm_output_11_GET(pack);
    int16_t  sue_pwm_output_12 = p171_sue_pwm_output_12_GET(pack);
    int16_t  sue_imu_location_x = p171_sue_imu_location_x_GET(pack);
    int16_t  sue_imu_location_y = p171_sue_imu_location_y_GET(pack);
    int16_t  sue_imu_location_z = p171_sue_imu_location_z_GET(pack);
    int16_t  sue_location_error_earth_x = p171_sue_location_error_earth_x_GET(pack);
    int16_t  sue_location_error_earth_y = p171_sue_location_error_earth_y_GET(pack);
    int16_t  sue_location_error_earth_z = p171_sue_location_error_earth_z_GET(pack);
    uint32_t  sue_flags = p171_sue_flags_GET(pack);
    int16_t  sue_osc_fails = p171_sue_osc_fails_GET(pack);
    int16_t  sue_imu_velocity_x = p171_sue_imu_velocity_x_GET(pack);
    int16_t  sue_imu_velocity_y = p171_sue_imu_velocity_y_GET(pack);
    int16_t  sue_imu_velocity_z = p171_sue_imu_velocity_z_GET(pack);
    int16_t  sue_waypoint_goal_x = p171_sue_waypoint_goal_x_GET(pack);
    int16_t  sue_waypoint_goal_y = p171_sue_waypoint_goal_y_GET(pack);
    int16_t  sue_waypoint_goal_z = p171_sue_waypoint_goal_z_GET(pack);
    int16_t  sue_aero_x = p171_sue_aero_x_GET(pack);
    int16_t  sue_aero_y = p171_sue_aero_y_GET(pack);
    int16_t  sue_aero_z = p171_sue_aero_z_GET(pack);
    int16_t  sue_barom_temp = p171_sue_barom_temp_GET(pack);
    int32_t  sue_barom_press = p171_sue_barom_press_GET(pack);
    int32_t  sue_barom_alt = p171_sue_barom_alt_GET(pack);
    int16_t  sue_bat_volt = p171_sue_bat_volt_GET(pack);
    int16_t  sue_bat_amp = p171_sue_bat_amp_GET(pack);
    int16_t  sue_bat_amp_hours = p171_sue_bat_amp_hours_GET(pack);
    int16_t  sue_desired_height = p171_sue_desired_height_GET(pack);
    int16_t  sue_memory_stack_free = p171_sue_memory_stack_free_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F4_172(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  sue_ROLL_STABILIZATION_AILERONS = p172_sue_ROLL_STABILIZATION_AILERONS_GET(pack);
    uint8_t  sue_ROLL_STABILIZATION_RUDDER = p172_sue_ROLL_STABILIZATION_RUDDER_GET(pack);
    uint8_t  sue_PITCH_STABILIZATION = p172_sue_PITCH_STABILIZATION_GET(pack);
    uint8_t  sue_YAW_STABILIZATION_RUDDER = p172_sue_YAW_STABILIZATION_RUDDER_GET(pack);
    uint8_t  sue_YAW_STABILIZATION_AILERON = p172_sue_YAW_STABILIZATION_AILERON_GET(pack);
    uint8_t  sue_AILERON_NAVIGATION = p172_sue_AILERON_NAVIGATION_GET(pack);
    uint8_t  sue_RUDDER_NAVIGATION = p172_sue_RUDDER_NAVIGATION_GET(pack);
    uint8_t  sue_ALTITUDEHOLD_STABILIZED = p172_sue_ALTITUDEHOLD_STABILIZED_GET(pack);
    uint8_t  sue_ALTITUDEHOLD_WAYPOINT = p172_sue_ALTITUDEHOLD_WAYPOINT_GET(pack);
    uint8_t  sue_RACING_MODE = p172_sue_RACING_MODE_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F5_173(Bounds_Inside * ph, Pack * pack)
{
    float  sue_YAWKP_AILERON = p173_sue_YAWKP_AILERON_GET(pack);
    float  sue_YAWKD_AILERON = p173_sue_YAWKD_AILERON_GET(pack);
    float  sue_ROLLKP = p173_sue_ROLLKP_GET(pack);
    float  sue_ROLLKD = p173_sue_ROLLKD_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F6_174(Bounds_Inside * ph, Pack * pack)
{
    float  sue_PITCHGAIN = p174_sue_PITCHGAIN_GET(pack);
    float  sue_PITCHKD = p174_sue_PITCHKD_GET(pack);
    float  sue_RUDDER_ELEV_MIX = p174_sue_RUDDER_ELEV_MIX_GET(pack);
    float  sue_ROLL_ELEV_MIX = p174_sue_ROLL_ELEV_MIX_GET(pack);
    float  sue_ELEVATOR_BOOST = p174_sue_ELEVATOR_BOOST_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F7_175(Bounds_Inside * ph, Pack * pack)
{
    float  sue_YAWKP_RUDDER = p175_sue_YAWKP_RUDDER_GET(pack);
    float  sue_YAWKD_RUDDER = p175_sue_YAWKD_RUDDER_GET(pack);
    float  sue_ROLLKP_RUDDER = p175_sue_ROLLKP_RUDDER_GET(pack);
    float  sue_ROLLKD_RUDDER = p175_sue_ROLLKD_RUDDER_GET(pack);
    float  sue_RUDDER_BOOST = p175_sue_RUDDER_BOOST_GET(pack);
    float  sue_RTL_PITCH_DOWN = p175_sue_RTL_PITCH_DOWN_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F8_176(Bounds_Inside * ph, Pack * pack)
{
    float  sue_HEIGHT_TARGET_MAX = p176_sue_HEIGHT_TARGET_MAX_GET(pack);
    float  sue_HEIGHT_TARGET_MIN = p176_sue_HEIGHT_TARGET_MIN_GET(pack);
    float  sue_ALT_HOLD_THROTTLE_MIN = p176_sue_ALT_HOLD_THROTTLE_MIN_GET(pack);
    float  sue_ALT_HOLD_THROTTLE_MAX = p176_sue_ALT_HOLD_THROTTLE_MAX_GET(pack);
    float  sue_ALT_HOLD_PITCH_MIN = p176_sue_ALT_HOLD_PITCH_MIN_GET(pack);
    float  sue_ALT_HOLD_PITCH_MAX = p176_sue_ALT_HOLD_PITCH_MAX_GET(pack);
    float  sue_ALT_HOLD_PITCH_HIGH = p176_sue_ALT_HOLD_PITCH_HIGH_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F13_177(Bounds_Inside * ph, Pack * pack)
{
    int16_t  sue_week_no = p177_sue_week_no_GET(pack);
    int32_t  sue_lat_origin = p177_sue_lat_origin_GET(pack);
    int32_t  sue_lon_origin = p177_sue_lon_origin_GET(pack);
    int32_t  sue_alt_origin = p177_sue_alt_origin_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F14_178(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  sue_WIND_ESTIMATION = p178_sue_WIND_ESTIMATION_GET(pack);
    uint8_t  sue_GPS_TYPE = p178_sue_GPS_TYPE_GET(pack);
    uint8_t  sue_DR = p178_sue_DR_GET(pack);
    uint8_t  sue_BOARD_TYPE = p178_sue_BOARD_TYPE_GET(pack);
    uint8_t  sue_AIRFRAME = p178_sue_AIRFRAME_GET(pack);
    int16_t  sue_RCON = p178_sue_RCON_GET(pack);
    int16_t  sue_TRAP_FLAGS = p178_sue_TRAP_FLAGS_GET(pack);
    uint32_t  sue_TRAP_SOURCE = p178_sue_TRAP_SOURCE_GET(pack);
    int16_t  sue_osc_fail_count = p178_sue_osc_fail_count_GET(pack);
    uint8_t  sue_CLOCK_CONFIG = p178_sue_CLOCK_CONFIG_GET(pack);
    uint8_t  sue_FLIGHT_PLAN_TYPE = p178_sue_FLIGHT_PLAN_TYPE_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F15_179(Bounds_Inside * ph, Pack * pack)
{
    uint8_t*  sue_ID_VEHICLE_MODEL_NAME = p179_sue_ID_VEHICLE_MODEL_NAME_GET_(pack);
//process data in sue_ID_VEHICLE_MODEL_NAME
    free(sue_ID_VEHICLE_MODEL_NAME);//never forget to dispose
    uint8_t*  sue_ID_VEHICLE_REGISTRATION = p179_sue_ID_VEHICLE_REGISTRATION_GET_(pack);
//process data in sue_ID_VEHICLE_REGISTRATION
    free(sue_ID_VEHICLE_REGISTRATION);//never forget to dispose
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F16_180(Bounds_Inside * ph, Pack * pack)
{
    uint8_t*  sue_ID_LEAD_PILOT = p180_sue_ID_LEAD_PILOT_GET_(pack);
//process data in sue_ID_LEAD_PILOT
    free(sue_ID_LEAD_PILOT);//never forget to dispose
    uint8_t*  sue_ID_DIY_DRONES_URL = p180_sue_ID_DIY_DRONES_URL_GET_(pack);
//process data in sue_ID_DIY_DRONES_URL
    free(sue_ID_DIY_DRONES_URL);//never forget to dispose
}
void c_CommunicationChannel_on_ALTITUDES_181(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p181_time_boot_ms_GET(pack);
    int32_t  alt_gps = p181_alt_gps_GET(pack);
    int32_t  alt_imu = p181_alt_imu_GET(pack);
    int32_t  alt_barometric = p181_alt_barometric_GET(pack);
    int32_t  alt_optical_flow = p181_alt_optical_flow_GET(pack);
    int32_t  alt_range_finder = p181_alt_range_finder_GET(pack);
    int32_t  alt_extra = p181_alt_extra_GET(pack);
}
void c_CommunicationChannel_on_AIRSPEEDS_182(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p182_time_boot_ms_GET(pack);
    int16_t  airspeed_imu = p182_airspeed_imu_GET(pack);
    int16_t  airspeed_pitot = p182_airspeed_pitot_GET(pack);
    int16_t  airspeed_hot_wire = p182_airspeed_hot_wire_GET(pack);
    int16_t  airspeed_ultrasonic = p182_airspeed_ultrasonic_GET(pack);
    int16_t  aoa = p182_aoa_GET(pack);
    int16_t  aoy = p182_aoy_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F17_183(Bounds_Inside * ph, Pack * pack)
{
    float  sue_feed_forward = p183_sue_feed_forward_GET(pack);
    float  sue_turn_rate_nav = p183_sue_turn_rate_nav_GET(pack);
    float  sue_turn_rate_fbw = p183_sue_turn_rate_fbw_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F18_184(Bounds_Inside * ph, Pack * pack)
{
    float  angle_of_attack_normal = p184_angle_of_attack_normal_GET(pack);
    float  angle_of_attack_inverted = p184_angle_of_attack_inverted_GET(pack);
    float  elevator_trim_normal = p184_elevator_trim_normal_GET(pack);
    float  elevator_trim_inverted = p184_elevator_trim_inverted_GET(pack);
    float  reference_speed = p184_reference_speed_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F19_185(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  sue_aileron_output_channel = p185_sue_aileron_output_channel_GET(pack);
    uint8_t  sue_aileron_reversed = p185_sue_aileron_reversed_GET(pack);
    uint8_t  sue_elevator_output_channel = p185_sue_elevator_output_channel_GET(pack);
    uint8_t  sue_elevator_reversed = p185_sue_elevator_reversed_GET(pack);
    uint8_t  sue_throttle_output_channel = p185_sue_throttle_output_channel_GET(pack);
    uint8_t  sue_throttle_reversed = p185_sue_throttle_reversed_GET(pack);
    uint8_t  sue_rudder_output_channel = p185_sue_rudder_output_channel_GET(pack);
    uint8_t  sue_rudder_reversed = p185_sue_rudder_reversed_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F20_186(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  sue_number_of_inputs = p186_sue_number_of_inputs_GET(pack);
    int16_t  sue_trim_value_input_1 = p186_sue_trim_value_input_1_GET(pack);
    int16_t  sue_trim_value_input_2 = p186_sue_trim_value_input_2_GET(pack);
    int16_t  sue_trim_value_input_3 = p186_sue_trim_value_input_3_GET(pack);
    int16_t  sue_trim_value_input_4 = p186_sue_trim_value_input_4_GET(pack);
    int16_t  sue_trim_value_input_5 = p186_sue_trim_value_input_5_GET(pack);
    int16_t  sue_trim_value_input_6 = p186_sue_trim_value_input_6_GET(pack);
    int16_t  sue_trim_value_input_7 = p186_sue_trim_value_input_7_GET(pack);
    int16_t  sue_trim_value_input_8 = p186_sue_trim_value_input_8_GET(pack);
    int16_t  sue_trim_value_input_9 = p186_sue_trim_value_input_9_GET(pack);
    int16_t  sue_trim_value_input_10 = p186_sue_trim_value_input_10_GET(pack);
    int16_t  sue_trim_value_input_11 = p186_sue_trim_value_input_11_GET(pack);
    int16_t  sue_trim_value_input_12 = p186_sue_trim_value_input_12_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F21_187(Bounds_Inside * ph, Pack * pack)
{
    int16_t  sue_accel_x_offset = p187_sue_accel_x_offset_GET(pack);
    int16_t  sue_accel_y_offset = p187_sue_accel_y_offset_GET(pack);
    int16_t  sue_accel_z_offset = p187_sue_accel_z_offset_GET(pack);
    int16_t  sue_gyro_x_offset = p187_sue_gyro_x_offset_GET(pack);
    int16_t  sue_gyro_y_offset = p187_sue_gyro_y_offset_GET(pack);
    int16_t  sue_gyro_z_offset = p187_sue_gyro_z_offset_GET(pack);
}
void c_CommunicationChannel_on_SERIAL_UDB_EXTRA_F22_188(Bounds_Inside * ph, Pack * pack)
{
    int16_t  sue_accel_x_at_calibration = p188_sue_accel_x_at_calibration_GET(pack);
    int16_t  sue_accel_y_at_calibration = p188_sue_accel_y_at_calibration_GET(pack);
    int16_t  sue_accel_z_at_calibration = p188_sue_accel_z_at_calibration_GET(pack);
    int16_t  sue_gyro_x_at_calibration = p188_sue_gyro_x_at_calibration_GET(pack);
    int16_t  sue_gyro_y_at_calibration = p188_sue_gyro_y_at_calibration_GET(pack);
    int16_t  sue_gyro_z_at_calibration = p188_sue_gyro_z_at_calibration_GET(pack);
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
    p0_type_SET(e_MAV_TYPE_MAV_TYPE_GROUND_ROVER, PH.base.pack) ;
    p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_FP, PH.base.pack) ;
    p0_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED), PH.base.pack) ;
    p0_custom_mode_SET((uint32_t)90622741L, PH.base.pack) ;
    p0_system_status_SET(e_MAV_STATE_MAV_STATE_CALIBRATING, PH.base.pack) ;
    p0_mavlink_version_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
    p1_onboard_control_sensors_present_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING), PH.base.pack) ;
    p1_onboard_control_sensors_enabled_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE), PH.base.pack) ;
    p1_onboard_control_sensors_health_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2), PH.base.pack) ;
    p1_load_SET((uint16_t)(uint16_t)44577, PH.base.pack) ;
    p1_voltage_battery_SET((uint16_t)(uint16_t)50312, PH.base.pack) ;
    p1_current_battery_SET((int16_t)(int16_t)24715, PH.base.pack) ;
    p1_battery_remaining_SET((int8_t)(int8_t) -32, PH.base.pack) ;
    p1_drop_rate_comm_SET((uint16_t)(uint16_t)40722, PH.base.pack) ;
    p1_errors_comm_SET((uint16_t)(uint16_t)26234, PH.base.pack) ;
    p1_errors_count1_SET((uint16_t)(uint16_t)18285, PH.base.pack) ;
    p1_errors_count2_SET((uint16_t)(uint16_t)3286, PH.base.pack) ;
    p1_errors_count3_SET((uint16_t)(uint16_t)54898, PH.base.pack) ;
    p1_errors_count4_SET((uint16_t)(uint16_t)9191, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
    p2_time_unix_usec_SET((uint64_t)932194005525831578L, PH.base.pack) ;
    p2_time_boot_ms_SET((uint32_t)2608757548L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
    p3_time_boot_ms_SET((uint32_t)4275094138L, PH.base.pack) ;
    p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
    p3_type_mask_SET((uint16_t)(uint16_t)8033, PH.base.pack) ;
    p3_x_SET((float) -3.0094474E37F, PH.base.pack) ;
    p3_y_SET((float)3.993842E37F, PH.base.pack) ;
    p3_z_SET((float)1.3564545E38F, PH.base.pack) ;
    p3_vx_SET((float)3.0220713E38F, PH.base.pack) ;
    p3_vy_SET((float)2.5665463E38F, PH.base.pack) ;
    p3_vz_SET((float) -3.0166527E38F, PH.base.pack) ;
    p3_afx_SET((float)2.1923423E38F, PH.base.pack) ;
    p3_afy_SET((float) -2.824174E38F, PH.base.pack) ;
    p3_afz_SET((float) -5.163811E37F, PH.base.pack) ;
    p3_yaw_SET((float)2.2231154E38F, PH.base.pack) ;
    p3_yaw_rate_SET((float)1.9466619E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PING_4(), &PH);
    p4_time_usec_SET((uint64_t)1753753100787085291L, PH.base.pack) ;
    p4_seq_SET((uint32_t)3693816888L, PH.base.pack) ;
    p4_target_system_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
    p4_target_component_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
    p5_target_system_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
    p5_control_request_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
    p5_version_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
    {
        char16_t   passkey = "uKhYggsbum";
        p5_passkey_SET(&passkey, 0,  sizeof(passkey), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
    p6_gcs_system_id_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
    p6_control_request_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
    p6_ack_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
    {
        char16_t   key = "iifnctddDrgZzShwv";
        p7_key_SET(&key, 0,  sizeof(key), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
    p11_target_system_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
    p11_base_mode_SET(e_MAV_MODE_MAV_MODE_TEST_DISARMED, PH.base.pack) ;
    p11_custom_mode_SET((uint32_t)2336654065L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
    p20_target_system_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
    p20_target_component_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
    {
        char16_t   param_id = "txotgrrivuem";
        p20_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p20_param_index_SET((int16_t)(int16_t)14968, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
    p21_target_system_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
    p21_target_component_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
    {
        char16_t   param_id = "qdsmazpsebhqz";
        p22_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p22_param_value_SET((float)1.9988175E38F, PH.base.pack) ;
    p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64, PH.base.pack) ;
    p22_param_count_SET((uint16_t)(uint16_t)65318, PH.base.pack) ;
    p22_param_index_SET((uint16_t)(uint16_t)55876, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
    p23_target_system_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
    p23_target_component_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
    {
        char16_t   param_id = "izahaeocjjlqoma";
        p23_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p23_param_value_SET((float)5.699156E37F, PH.base.pack) ;
    p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT32, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
    p24_time_usec_SET((uint64_t)4175890495120203258L, PH.base.pack) ;
    p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, PH.base.pack) ;
    p24_lat_SET((int32_t)1346266633, PH.base.pack) ;
    p24_lon_SET((int32_t)458223225, PH.base.pack) ;
    p24_alt_SET((int32_t)565553210, PH.base.pack) ;
    p24_eph_SET((uint16_t)(uint16_t)26125, PH.base.pack) ;
    p24_epv_SET((uint16_t)(uint16_t)11475, PH.base.pack) ;
    p24_vel_SET((uint16_t)(uint16_t)52279, PH.base.pack) ;
    p24_cog_SET((uint16_t)(uint16_t)64906, PH.base.pack) ;
    p24_satellites_visible_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
    p24_alt_ellipsoid_SET((int32_t) -1100394687, &PH) ;
    p24_h_acc_SET((uint32_t)682530398L, &PH) ;
    p24_v_acc_SET((uint32_t)3060137763L, &PH) ;
    p24_vel_acc_SET((uint32_t)3601469862L, &PH) ;
    p24_hdg_acc_SET((uint32_t)1870411227L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
    p25_satellites_visible_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
    {
        uint8_t  satellite_prn [] =  {(uint8_t)214, (uint8_t)186, (uint8_t)56, (uint8_t)72, (uint8_t)8, (uint8_t)6, (uint8_t)166, (uint8_t)49, (uint8_t)134, (uint8_t)202, (uint8_t)92, (uint8_t)90, (uint8_t)207, (uint8_t)72, (uint8_t)50, (uint8_t)29, (uint8_t)158, (uint8_t)111, (uint8_t)78, (uint8_t)32};
        p25_satellite_prn_SET(&satellite_prn, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_used [] =  {(uint8_t)113, (uint8_t)137, (uint8_t)45, (uint8_t)106, (uint8_t)175, (uint8_t)224, (uint8_t)151, (uint8_t)180, (uint8_t)202, (uint8_t)131, (uint8_t)94, (uint8_t)32, (uint8_t)104, (uint8_t)241, (uint8_t)82, (uint8_t)79, (uint8_t)120, (uint8_t)157, (uint8_t)145, (uint8_t)44};
        p25_satellite_used_SET(&satellite_used, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_elevation [] =  {(uint8_t)183, (uint8_t)191, (uint8_t)98, (uint8_t)173, (uint8_t)63, (uint8_t)174, (uint8_t)245, (uint8_t)54, (uint8_t)100, (uint8_t)55, (uint8_t)100, (uint8_t)102, (uint8_t)147, (uint8_t)157, (uint8_t)229, (uint8_t)208, (uint8_t)13, (uint8_t)29, (uint8_t)14, (uint8_t)117};
        p25_satellite_elevation_SET(&satellite_elevation, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_azimuth [] =  {(uint8_t)233, (uint8_t)229, (uint8_t)55, (uint8_t)98, (uint8_t)177, (uint8_t)104, (uint8_t)116, (uint8_t)232, (uint8_t)48, (uint8_t)147, (uint8_t)185, (uint8_t)193, (uint8_t)107, (uint8_t)56, (uint8_t)119, (uint8_t)231, (uint8_t)39, (uint8_t)142, (uint8_t)133, (uint8_t)187};
        p25_satellite_azimuth_SET(&satellite_azimuth, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_snr [] =  {(uint8_t)148, (uint8_t)173, (uint8_t)36, (uint8_t)157, (uint8_t)85, (uint8_t)110, (uint8_t)204, (uint8_t)67, (uint8_t)133, (uint8_t)34, (uint8_t)122, (uint8_t)4, (uint8_t)43, (uint8_t)228, (uint8_t)238, (uint8_t)223, (uint8_t)108, (uint8_t)179, (uint8_t)50, (uint8_t)157};
        p25_satellite_snr_SET(&satellite_snr, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
    p26_time_boot_ms_SET((uint32_t)1928472078L, PH.base.pack) ;
    p26_xacc_SET((int16_t)(int16_t) -30078, PH.base.pack) ;
    p26_yacc_SET((int16_t)(int16_t)14284, PH.base.pack) ;
    p26_zacc_SET((int16_t)(int16_t)23773, PH.base.pack) ;
    p26_xgyro_SET((int16_t)(int16_t) -7934, PH.base.pack) ;
    p26_ygyro_SET((int16_t)(int16_t)2993, PH.base.pack) ;
    p26_zgyro_SET((int16_t)(int16_t) -32690, PH.base.pack) ;
    p26_xmag_SET((int16_t)(int16_t)24478, PH.base.pack) ;
    p26_ymag_SET((int16_t)(int16_t) -10121, PH.base.pack) ;
    p26_zmag_SET((int16_t)(int16_t) -16497, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
    p27_time_usec_SET((uint64_t)1075046587618278154L, PH.base.pack) ;
    p27_xacc_SET((int16_t)(int16_t) -2146, PH.base.pack) ;
    p27_yacc_SET((int16_t)(int16_t) -5005, PH.base.pack) ;
    p27_zacc_SET((int16_t)(int16_t)13512, PH.base.pack) ;
    p27_xgyro_SET((int16_t)(int16_t) -4367, PH.base.pack) ;
    p27_ygyro_SET((int16_t)(int16_t) -22665, PH.base.pack) ;
    p27_zgyro_SET((int16_t)(int16_t) -11947, PH.base.pack) ;
    p27_xmag_SET((int16_t)(int16_t)17008, PH.base.pack) ;
    p27_ymag_SET((int16_t)(int16_t) -10988, PH.base.pack) ;
    p27_zmag_SET((int16_t)(int16_t) -31163, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
    p28_time_usec_SET((uint64_t)6187945938420674303L, PH.base.pack) ;
    p28_press_abs_SET((int16_t)(int16_t)24501, PH.base.pack) ;
    p28_press_diff1_SET((int16_t)(int16_t) -12621, PH.base.pack) ;
    p28_press_diff2_SET((int16_t)(int16_t)27127, PH.base.pack) ;
    p28_temperature_SET((int16_t)(int16_t)6569, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
    p29_time_boot_ms_SET((uint32_t)808341937L, PH.base.pack) ;
    p29_press_abs_SET((float)2.8164598E38F, PH.base.pack) ;
    p29_press_diff_SET((float)1.3438708E38F, PH.base.pack) ;
    p29_temperature_SET((int16_t)(int16_t) -28053, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
    p30_time_boot_ms_SET((uint32_t)2815736377L, PH.base.pack) ;
    p30_roll_SET((float)1.7924433E38F, PH.base.pack) ;
    p30_pitch_SET((float)2.547549E38F, PH.base.pack) ;
    p30_yaw_SET((float)1.0580472E38F, PH.base.pack) ;
    p30_rollspeed_SET((float) -3.0935206E38F, PH.base.pack) ;
    p30_pitchspeed_SET((float) -8.523917E37F, PH.base.pack) ;
    p30_yawspeed_SET((float) -1.1408744E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
    p31_time_boot_ms_SET((uint32_t)1088952411L, PH.base.pack) ;
    p31_q1_SET((float)1.4293426E38F, PH.base.pack) ;
    p31_q2_SET((float) -7.400092E37F, PH.base.pack) ;
    p31_q3_SET((float) -6.5398667E37F, PH.base.pack) ;
    p31_q4_SET((float) -2.7218105E38F, PH.base.pack) ;
    p31_rollspeed_SET((float) -9.291097E37F, PH.base.pack) ;
    p31_pitchspeed_SET((float) -1.7492998E38F, PH.base.pack) ;
    p31_yawspeed_SET((float)2.7674977E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
    p32_time_boot_ms_SET((uint32_t)2122673408L, PH.base.pack) ;
    p32_x_SET((float)5.0580346E36F, PH.base.pack) ;
    p32_y_SET((float)1.7012838E38F, PH.base.pack) ;
    p32_z_SET((float) -1.6209903E37F, PH.base.pack) ;
    p32_vx_SET((float) -7.551036E37F, PH.base.pack) ;
    p32_vy_SET((float) -3.2934554E38F, PH.base.pack) ;
    p32_vz_SET((float) -4.8966706E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
    p33_time_boot_ms_SET((uint32_t)3038607218L, PH.base.pack) ;
    p33_lat_SET((int32_t) -271628643, PH.base.pack) ;
    p33_lon_SET((int32_t)869140424, PH.base.pack) ;
    p33_alt_SET((int32_t)274183095, PH.base.pack) ;
    p33_relative_alt_SET((int32_t)668033477, PH.base.pack) ;
    p33_vx_SET((int16_t)(int16_t) -23550, PH.base.pack) ;
    p33_vy_SET((int16_t)(int16_t)28496, PH.base.pack) ;
    p33_vz_SET((int16_t)(int16_t)30240, PH.base.pack) ;
    p33_hdg_SET((uint16_t)(uint16_t)3178, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
    p34_time_boot_ms_SET((uint32_t)2362390763L, PH.base.pack) ;
    p34_port_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
    p34_chan1_scaled_SET((int16_t)(int16_t)6779, PH.base.pack) ;
    p34_chan2_scaled_SET((int16_t)(int16_t)12375, PH.base.pack) ;
    p34_chan3_scaled_SET((int16_t)(int16_t)11018, PH.base.pack) ;
    p34_chan4_scaled_SET((int16_t)(int16_t) -22873, PH.base.pack) ;
    p34_chan5_scaled_SET((int16_t)(int16_t)6508, PH.base.pack) ;
    p34_chan6_scaled_SET((int16_t)(int16_t) -30755, PH.base.pack) ;
    p34_chan7_scaled_SET((int16_t)(int16_t)2551, PH.base.pack) ;
    p34_chan8_scaled_SET((int16_t)(int16_t)8794, PH.base.pack) ;
    p34_rssi_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
    p35_time_boot_ms_SET((uint32_t)1088778628L, PH.base.pack) ;
    p35_port_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
    p35_chan1_raw_SET((uint16_t)(uint16_t)45711, PH.base.pack) ;
    p35_chan2_raw_SET((uint16_t)(uint16_t)7531, PH.base.pack) ;
    p35_chan3_raw_SET((uint16_t)(uint16_t)6848, PH.base.pack) ;
    p35_chan4_raw_SET((uint16_t)(uint16_t)49111, PH.base.pack) ;
    p35_chan5_raw_SET((uint16_t)(uint16_t)29730, PH.base.pack) ;
    p35_chan6_raw_SET((uint16_t)(uint16_t)5116, PH.base.pack) ;
    p35_chan7_raw_SET((uint16_t)(uint16_t)45711, PH.base.pack) ;
    p35_chan8_raw_SET((uint16_t)(uint16_t)61824, PH.base.pack) ;
    p35_rssi_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
    p36_time_usec_SET((uint32_t)800366892L, PH.base.pack) ;
    p36_port_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
    p36_servo1_raw_SET((uint16_t)(uint16_t)44984, PH.base.pack) ;
    p36_servo2_raw_SET((uint16_t)(uint16_t)9281, PH.base.pack) ;
    p36_servo3_raw_SET((uint16_t)(uint16_t)14738, PH.base.pack) ;
    p36_servo4_raw_SET((uint16_t)(uint16_t)25963, PH.base.pack) ;
    p36_servo5_raw_SET((uint16_t)(uint16_t)60909, PH.base.pack) ;
    p36_servo6_raw_SET((uint16_t)(uint16_t)25415, PH.base.pack) ;
    p36_servo7_raw_SET((uint16_t)(uint16_t)53514, PH.base.pack) ;
    p36_servo8_raw_SET((uint16_t)(uint16_t)64055, PH.base.pack) ;
    p36_servo9_raw_SET((uint16_t)(uint16_t)7003, &PH) ;
    p36_servo10_raw_SET((uint16_t)(uint16_t)34082, &PH) ;
    p36_servo11_raw_SET((uint16_t)(uint16_t)12881, &PH) ;
    p36_servo12_raw_SET((uint16_t)(uint16_t)41382, &PH) ;
    p36_servo13_raw_SET((uint16_t)(uint16_t)51436, &PH) ;
    p36_servo14_raw_SET((uint16_t)(uint16_t)8606, &PH) ;
    p36_servo15_raw_SET((uint16_t)(uint16_t)49266, &PH) ;
    p36_servo16_raw_SET((uint16_t)(uint16_t)16328, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
    p37_target_system_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
    p37_target_component_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
    p37_start_index_SET((int16_t)(int16_t)15296, PH.base.pack) ;
    p37_end_index_SET((int16_t)(int16_t)30000, PH.base.pack) ;
    p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
    p38_target_system_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
    p38_target_component_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
    p38_start_index_SET((int16_t)(int16_t) -1836, PH.base.pack) ;
    p38_end_index_SET((int16_t)(int16_t) -16401, PH.base.pack) ;
    p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
    p39_target_system_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
    p39_target_component_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
    p39_seq_SET((uint16_t)(uint16_t)60195, PH.base.pack) ;
    p39_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
    p39_command_SET(e_MAV_CMD_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS, PH.base.pack) ;
    p39_current_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
    p39_autocontinue_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    p39_param1_SET((float) -2.7347744E38F, PH.base.pack) ;
    p39_param2_SET((float) -2.4275466E38F, PH.base.pack) ;
    p39_param3_SET((float) -1.855246E38F, PH.base.pack) ;
    p39_param4_SET((float)1.8142853E38F, PH.base.pack) ;
    p39_x_SET((float) -1.5399856E37F, PH.base.pack) ;
    p39_y_SET((float) -2.3260721E38F, PH.base.pack) ;
    p39_z_SET((float) -2.3912754E38F, PH.base.pack) ;
    p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
    p40_target_system_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
    p40_target_component_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
    p40_seq_SET((uint16_t)(uint16_t)5622, PH.base.pack) ;
    p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
    p41_target_system_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
    p41_target_component_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
    p41_seq_SET((uint16_t)(uint16_t)59497, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
    p42_seq_SET((uint16_t)(uint16_t)24853, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
    p43_target_system_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
    p43_target_component_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
    p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
    p44_target_system_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
    p44_target_component_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
    p44_count_SET((uint16_t)(uint16_t)61304, PH.base.pack) ;
    p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
    p45_target_system_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
    p45_target_component_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
    p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
    p46_seq_SET((uint16_t)(uint16_t)46565, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
    p47_target_system_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
    p47_target_component_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
    p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_NO_SPACE, PH.base.pack) ;
    p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
    p48_target_system_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
    p48_latitude_SET((int32_t)1700919131, PH.base.pack) ;
    p48_longitude_SET((int32_t) -741852359, PH.base.pack) ;
    p48_altitude_SET((int32_t) -955195554, PH.base.pack) ;
    p48_time_usec_SET((uint64_t)4928946272648962390L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
    p49_latitude_SET((int32_t)1386375462, PH.base.pack) ;
    p49_longitude_SET((int32_t)1237368579, PH.base.pack) ;
    p49_altitude_SET((int32_t) -1637187669, PH.base.pack) ;
    p49_time_usec_SET((uint64_t)3910961694744480022L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
    p50_target_system_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
    p50_target_component_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
    {
        char16_t   param_id = "ndtqnqdpwladlnig";
        p50_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p50_param_index_SET((int16_t)(int16_t) -9734, PH.base.pack) ;
    p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
    p50_param_value0_SET((float)2.5690225E38F, PH.base.pack) ;
    p50_scale_SET((float) -4.879682E37F, PH.base.pack) ;
    p50_param_value_min_SET((float)3.8670422E37F, PH.base.pack) ;
    p50_param_value_max_SET((float) -8.324063E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
    p51_target_system_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
    p51_target_component_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
    p51_seq_SET((uint16_t)(uint16_t)47744, PH.base.pack) ;
    p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
    p54_target_system_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
    p54_target_component_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
    p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
    p54_p1x_SET((float) -2.1778516E37F, PH.base.pack) ;
    p54_p1y_SET((float)1.6863711E38F, PH.base.pack) ;
    p54_p1z_SET((float) -2.5084374E38F, PH.base.pack) ;
    p54_p2x_SET((float)2.251429E38F, PH.base.pack) ;
    p54_p2y_SET((float) -2.8953846E38F, PH.base.pack) ;
    p54_p2z_SET((float) -3.9684672E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
    p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
    p55_p1x_SET((float) -1.1772356E38F, PH.base.pack) ;
    p55_p1y_SET((float) -7.075527E37F, PH.base.pack) ;
    p55_p1z_SET((float)9.721082E36F, PH.base.pack) ;
    p55_p2x_SET((float)1.1956995E38F, PH.base.pack) ;
    p55_p2y_SET((float)3.4025667E38F, PH.base.pack) ;
    p55_p2z_SET((float) -2.5396053E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
    p61_time_usec_SET((uint64_t)8477699336163269578L, PH.base.pack) ;
    {
        float  q [] =  {-5.554903E37F, -9.237861E36F, 2.7980823E38F, 9.267566E37F};
        p61_q_SET(&q, 0, &PH.base.pack) ;
    }
    p61_rollspeed_SET((float) -6.655249E37F, PH.base.pack) ;
    p61_pitchspeed_SET((float) -1.6086925E38F, PH.base.pack) ;
    p61_yawspeed_SET((float)7.2332204E37F, PH.base.pack) ;
    {
        float  covariance [] =  {2.3990695E38F, -3.2444284E38F, 1.0920996E38F, 3.2294778E38F, 2.1673714E38F, 2.1190928E38F, -1.7452446E38F, 4.0636267E37F, -3.3162E38F};
        p61_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
    p62_nav_roll_SET((float)2.2320656E38F, PH.base.pack) ;
    p62_nav_pitch_SET((float) -1.3545818E38F, PH.base.pack) ;
    p62_nav_bearing_SET((int16_t)(int16_t)509, PH.base.pack) ;
    p62_target_bearing_SET((int16_t)(int16_t)20354, PH.base.pack) ;
    p62_wp_dist_SET((uint16_t)(uint16_t)39488, PH.base.pack) ;
    p62_alt_error_SET((float) -2.4697779E38F, PH.base.pack) ;
    p62_aspd_error_SET((float) -2.63178E36F, PH.base.pack) ;
    p62_xtrack_error_SET((float)7.693932E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
    p63_time_usec_SET((uint64_t)6955783068410987852L, PH.base.pack) ;
    p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, PH.base.pack) ;
    p63_lat_SET((int32_t)1131832198, PH.base.pack) ;
    p63_lon_SET((int32_t)1730794799, PH.base.pack) ;
    p63_alt_SET((int32_t)2004258430, PH.base.pack) ;
    p63_relative_alt_SET((int32_t) -1820362835, PH.base.pack) ;
    p63_vx_SET((float) -2.9635568E38F, PH.base.pack) ;
    p63_vy_SET((float)1.1650152E38F, PH.base.pack) ;
    p63_vz_SET((float)2.1332689E37F, PH.base.pack) ;
    {
        float  covariance [] =  {-2.9505075E38F, -2.0482496E38F, 2.6215824E38F, -1.7401529E38F, 2.48196E38F, 3.3608928E38F, 1.4478262E37F, 1.2653661E38F, -3.2654034E38F, -9.846968E37F, 1.2231161E38F, 3.3067603E38F, -1.6730275E37F, -1.1418033E37F, -1.3010882E38F, 6.8143927E37F, 2.2543418E38F, 1.3164144E38F, -1.9371232E38F, -3.3476E38F, -2.8281636E38F, 1.1517274E38F, 2.2085149E38F, 1.8798816E38F, -3.20413E38F, 1.7632328E38F, 3.0792034E38F, 2.1031891E38F, -7.954787E37F, 7.0860026E37F, -3.1455064E38F, -9.364822E37F, -2.4507448E38F, -1.4507269E38F, 9.920864E37F, 1.9427461E38F};
        p63_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
    p64_time_usec_SET((uint64_t)132637083608808593L, PH.base.pack) ;
    p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, PH.base.pack) ;
    p64_x_SET((float) -8.1273583E37F, PH.base.pack) ;
    p64_y_SET((float) -1.0991529E38F, PH.base.pack) ;
    p64_z_SET((float)4.786781E36F, PH.base.pack) ;
    p64_vx_SET((float) -1.6207931E37F, PH.base.pack) ;
    p64_vy_SET((float) -3.3807534E38F, PH.base.pack) ;
    p64_vz_SET((float) -2.5945911E38F, PH.base.pack) ;
    p64_ax_SET((float)1.3586672E38F, PH.base.pack) ;
    p64_ay_SET((float) -3.1195697E38F, PH.base.pack) ;
    p64_az_SET((float)2.111199E37F, PH.base.pack) ;
    {
        float  covariance [] =  {-1.7087539E38F, -3.2701625E38F, -1.4175319E37F, 1.8490234E38F, 3.003634E38F, -6.467857E37F, 2.7380981E38F, 3.3208748E38F, 2.7351302E38F, -9.969804E36F, -1.0559317E38F, 3.3525598E38F, 2.315659E37F, 2.5089752E38F, 2.720222E38F, 3.3497732E38F, 2.5472938E38F, 2.6601133E38F, -1.0602151E38F, 4.2978725E37F, -2.8137919E37F, -1.3388151E38F, -8.4821285E37F, 1.0199237E38F, 1.990993E38F, 6.9055464E37F, 7.0893116E37F, 5.464506E36F, 3.1141555E38F, 6.143661E37F, 1.1505404E38F, -2.4402284E38F, 1.4814772E38F, -3.0251441E38F, 3.3305065E38F, -8.96407E37F, -2.0265593E38F, -9.341102E36F, 2.932505E38F, -2.5327626E37F, 5.072836E37F, 3.4266847E37F, 2.4971046E38F, -2.747304E38F, -1.3212447E38F};
        p64_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
    p65_time_boot_ms_SET((uint32_t)4225220073L, PH.base.pack) ;
    p65_chancount_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
    p65_chan1_raw_SET((uint16_t)(uint16_t)64344, PH.base.pack) ;
    p65_chan2_raw_SET((uint16_t)(uint16_t)30316, PH.base.pack) ;
    p65_chan3_raw_SET((uint16_t)(uint16_t)62414, PH.base.pack) ;
    p65_chan4_raw_SET((uint16_t)(uint16_t)25950, PH.base.pack) ;
    p65_chan5_raw_SET((uint16_t)(uint16_t)2907, PH.base.pack) ;
    p65_chan6_raw_SET((uint16_t)(uint16_t)42746, PH.base.pack) ;
    p65_chan7_raw_SET((uint16_t)(uint16_t)59756, PH.base.pack) ;
    p65_chan8_raw_SET((uint16_t)(uint16_t)49269, PH.base.pack) ;
    p65_chan9_raw_SET((uint16_t)(uint16_t)11465, PH.base.pack) ;
    p65_chan10_raw_SET((uint16_t)(uint16_t)35536, PH.base.pack) ;
    p65_chan11_raw_SET((uint16_t)(uint16_t)30514, PH.base.pack) ;
    p65_chan12_raw_SET((uint16_t)(uint16_t)29772, PH.base.pack) ;
    p65_chan13_raw_SET((uint16_t)(uint16_t)49608, PH.base.pack) ;
    p65_chan14_raw_SET((uint16_t)(uint16_t)61265, PH.base.pack) ;
    p65_chan15_raw_SET((uint16_t)(uint16_t)10709, PH.base.pack) ;
    p65_chan16_raw_SET((uint16_t)(uint16_t)20582, PH.base.pack) ;
    p65_chan17_raw_SET((uint16_t)(uint16_t)15989, PH.base.pack) ;
    p65_chan18_raw_SET((uint16_t)(uint16_t)11384, PH.base.pack) ;
    p65_rssi_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
    p66_target_system_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
    p66_target_component_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
    p66_req_stream_id_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
    p66_req_message_rate_SET((uint16_t)(uint16_t)6594, PH.base.pack) ;
    p66_start_stop_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
    p67_stream_id_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
    p67_message_rate_SET((uint16_t)(uint16_t)5392, PH.base.pack) ;
    p67_on_off_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
    p69_target_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
    p69_x_SET((int16_t)(int16_t)14411, PH.base.pack) ;
    p69_y_SET((int16_t)(int16_t) -8607, PH.base.pack) ;
    p69_z_SET((int16_t)(int16_t) -30231, PH.base.pack) ;
    p69_r_SET((int16_t)(int16_t)4162, PH.base.pack) ;
    p69_buttons_SET((uint16_t)(uint16_t)38486, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
    p70_target_system_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
    p70_target_component_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
    p70_chan1_raw_SET((uint16_t)(uint16_t)49202, PH.base.pack) ;
    p70_chan2_raw_SET((uint16_t)(uint16_t)25660, PH.base.pack) ;
    p70_chan3_raw_SET((uint16_t)(uint16_t)25867, PH.base.pack) ;
    p70_chan4_raw_SET((uint16_t)(uint16_t)24323, PH.base.pack) ;
    p70_chan5_raw_SET((uint16_t)(uint16_t)36270, PH.base.pack) ;
    p70_chan6_raw_SET((uint16_t)(uint16_t)36067, PH.base.pack) ;
    p70_chan7_raw_SET((uint16_t)(uint16_t)48809, PH.base.pack) ;
    p70_chan8_raw_SET((uint16_t)(uint16_t)22603, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
    p73_target_system_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
    p73_target_component_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
    p73_seq_SET((uint16_t)(uint16_t)30154, PH.base.pack) ;
    p73_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
    p73_command_SET(e_MAV_CMD_MAV_CMD_CONDITION_DELAY, PH.base.pack) ;
    p73_current_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
    p73_autocontinue_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
    p73_param1_SET((float) -3.7351977E37F, PH.base.pack) ;
    p73_param2_SET((float)3.3608756E38F, PH.base.pack) ;
    p73_param3_SET((float)3.3768492E38F, PH.base.pack) ;
    p73_param4_SET((float)5.6953016E37F, PH.base.pack) ;
    p73_x_SET((int32_t) -924205303, PH.base.pack) ;
    p73_y_SET((int32_t)1728588222, PH.base.pack) ;
    p73_z_SET((float)1.9622373E38F, PH.base.pack) ;
    p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
    p74_airspeed_SET((float)2.5076396E38F, PH.base.pack) ;
    p74_groundspeed_SET((float) -1.2654419E38F, PH.base.pack) ;
    p74_heading_SET((int16_t)(int16_t)32297, PH.base.pack) ;
    p74_throttle_SET((uint16_t)(uint16_t)50308, PH.base.pack) ;
    p74_alt_SET((float) -6.863841E37F, PH.base.pack) ;
    p74_climb_SET((float)4.2524457E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
    p75_target_system_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
    p75_target_component_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
    p75_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
    p75_command_SET(e_MAV_CMD_MAV_CMD_USER_5, PH.base.pack) ;
    p75_current_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
    p75_autocontinue_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
    p75_param1_SET((float) -3.2249293E38F, PH.base.pack) ;
    p75_param2_SET((float)2.6578153E38F, PH.base.pack) ;
    p75_param3_SET((float) -2.610821E38F, PH.base.pack) ;
    p75_param4_SET((float) -9.857873E37F, PH.base.pack) ;
    p75_x_SET((int32_t)1777564096, PH.base.pack) ;
    p75_y_SET((int32_t)471658299, PH.base.pack) ;
    p75_z_SET((float) -3.3640186E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
    p76_target_system_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
    p76_target_component_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
    p76_command_SET(e_MAV_CMD_MAV_CMD_LOGGING_STOP, PH.base.pack) ;
    p76_confirmation_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
    p76_param1_SET((float) -5.2421586E37F, PH.base.pack) ;
    p76_param2_SET((float) -5.2933834E37F, PH.base.pack) ;
    p76_param3_SET((float) -2.7753703E38F, PH.base.pack) ;
    p76_param4_SET((float) -1.0104854E38F, PH.base.pack) ;
    p76_param5_SET((float) -2.3005676E38F, PH.base.pack) ;
    p76_param6_SET((float)2.2186514E38F, PH.base.pack) ;
    p76_param7_SET((float)6.592085E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
    p77_command_SET(e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS, PH.base.pack) ;
    p77_result_SET(e_MAV_RESULT_MAV_RESULT_DENIED, PH.base.pack) ;
    p77_progress_SET((uint8_t)(uint8_t)186, &PH) ;
    p77_result_param2_SET((int32_t) -480153796, &PH) ;
    p77_target_system_SET((uint8_t)(uint8_t)43, &PH) ;
    p77_target_component_SET((uint8_t)(uint8_t)249, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
    p81_time_boot_ms_SET((uint32_t)4170880325L, PH.base.pack) ;
    p81_roll_SET((float) -2.1749568E38F, PH.base.pack) ;
    p81_pitch_SET((float) -7.4324687E37F, PH.base.pack) ;
    p81_yaw_SET((float)2.2153482E38F, PH.base.pack) ;
    p81_thrust_SET((float)5.150542E36F, PH.base.pack) ;
    p81_mode_switch_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    p81_manual_override_switch_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
    p82_time_boot_ms_SET((uint32_t)2089778393L, PH.base.pack) ;
    p82_target_system_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
    p82_target_component_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
    p82_type_mask_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
    {
        float  q [] =  {-1.0673538E37F, -2.0313971E38F, 7.409919E37F, -1.2320533E38F};
        p82_q_SET(&q, 0, &PH.base.pack) ;
    }
    p82_body_roll_rate_SET((float)1.9564809E37F, PH.base.pack) ;
    p82_body_pitch_rate_SET((float)1.790217E38F, PH.base.pack) ;
    p82_body_yaw_rate_SET((float) -2.812571E38F, PH.base.pack) ;
    p82_thrust_SET((float)2.0887736E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
    p83_time_boot_ms_SET((uint32_t)592281183L, PH.base.pack) ;
    p83_type_mask_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    {
        float  q [] =  {-7.0017373E37F, -1.8358386E38F, 2.2562589E38F, -2.7151186E38F};
        p83_q_SET(&q, 0, &PH.base.pack) ;
    }
    p83_body_roll_rate_SET((float) -1.7224234E38F, PH.base.pack) ;
    p83_body_pitch_rate_SET((float) -9.249427E37F, PH.base.pack) ;
    p83_body_yaw_rate_SET((float) -3.6471867E37F, PH.base.pack) ;
    p83_thrust_SET((float)2.9618956E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
    p84_time_boot_ms_SET((uint32_t)1044054888L, PH.base.pack) ;
    p84_target_system_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
    p84_target_component_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
    p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
    p84_type_mask_SET((uint16_t)(uint16_t)51161, PH.base.pack) ;
    p84_x_SET((float) -5.323154E37F, PH.base.pack) ;
    p84_y_SET((float)4.0221692E37F, PH.base.pack) ;
    p84_z_SET((float)3.1833351E38F, PH.base.pack) ;
    p84_vx_SET((float)2.5245564E38F, PH.base.pack) ;
    p84_vy_SET((float) -3.0697845E37F, PH.base.pack) ;
    p84_vz_SET((float)5.7795046E37F, PH.base.pack) ;
    p84_afx_SET((float) -7.277042E36F, PH.base.pack) ;
    p84_afy_SET((float)1.7967426E38F, PH.base.pack) ;
    p84_afz_SET((float) -2.9401604E38F, PH.base.pack) ;
    p84_yaw_SET((float)2.7973941E38F, PH.base.pack) ;
    p84_yaw_rate_SET((float)6.7141664E34F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
    p86_time_boot_ms_SET((uint32_t)334754349L, PH.base.pack) ;
    p86_target_system_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
    p86_target_component_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
    p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
    p86_type_mask_SET((uint16_t)(uint16_t)35908, PH.base.pack) ;
    p86_lat_int_SET((int32_t)73744947, PH.base.pack) ;
    p86_lon_int_SET((int32_t) -769702917, PH.base.pack) ;
    p86_alt_SET((float) -1.5138962E38F, PH.base.pack) ;
    p86_vx_SET((float)2.2809198E38F, PH.base.pack) ;
    p86_vy_SET((float)2.2866348E38F, PH.base.pack) ;
    p86_vz_SET((float) -1.6773715E38F, PH.base.pack) ;
    p86_afx_SET((float) -7.1823676E36F, PH.base.pack) ;
    p86_afy_SET((float)1.6699774E38F, PH.base.pack) ;
    p86_afz_SET((float)1.596341E38F, PH.base.pack) ;
    p86_yaw_SET((float)3.1734951E38F, PH.base.pack) ;
    p86_yaw_rate_SET((float)6.976341E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
    p87_time_boot_ms_SET((uint32_t)665895130L, PH.base.pack) ;
    p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
    p87_type_mask_SET((uint16_t)(uint16_t)11186, PH.base.pack) ;
    p87_lat_int_SET((int32_t)1554772347, PH.base.pack) ;
    p87_lon_int_SET((int32_t) -514172670, PH.base.pack) ;
    p87_alt_SET((float) -4.639195E37F, PH.base.pack) ;
    p87_vx_SET((float) -2.1311685E38F, PH.base.pack) ;
    p87_vy_SET((float)9.088623E37F, PH.base.pack) ;
    p87_vz_SET((float) -1.4517926E38F, PH.base.pack) ;
    p87_afx_SET((float) -1.4543545E38F, PH.base.pack) ;
    p87_afy_SET((float)2.2808088E36F, PH.base.pack) ;
    p87_afz_SET((float)3.136523E38F, PH.base.pack) ;
    p87_yaw_SET((float)2.5863241E37F, PH.base.pack) ;
    p87_yaw_rate_SET((float)6.376009E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
    p89_time_boot_ms_SET((uint32_t)558436595L, PH.base.pack) ;
    p89_x_SET((float) -4.0306975E35F, PH.base.pack) ;
    p89_y_SET((float)2.0421472E38F, PH.base.pack) ;
    p89_z_SET((float)8.959782E37F, PH.base.pack) ;
    p89_roll_SET((float) -2.7195458E38F, PH.base.pack) ;
    p89_pitch_SET((float)8.4298973E37F, PH.base.pack) ;
    p89_yaw_SET((float) -3.136907E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
    p90_time_usec_SET((uint64_t)8063945284631360842L, PH.base.pack) ;
    p90_roll_SET((float)2.444573E38F, PH.base.pack) ;
    p90_pitch_SET((float) -1.8017643E38F, PH.base.pack) ;
    p90_yaw_SET((float) -2.2886229E38F, PH.base.pack) ;
    p90_rollspeed_SET((float)1.4970577E38F, PH.base.pack) ;
    p90_pitchspeed_SET((float) -1.7642007E38F, PH.base.pack) ;
    p90_yawspeed_SET((float) -2.989514E37F, PH.base.pack) ;
    p90_lat_SET((int32_t) -1660322951, PH.base.pack) ;
    p90_lon_SET((int32_t)1267999072, PH.base.pack) ;
    p90_alt_SET((int32_t)602015228, PH.base.pack) ;
    p90_vx_SET((int16_t)(int16_t) -22188, PH.base.pack) ;
    p90_vy_SET((int16_t)(int16_t) -31697, PH.base.pack) ;
    p90_vz_SET((int16_t)(int16_t) -2202, PH.base.pack) ;
    p90_xacc_SET((int16_t)(int16_t) -1548, PH.base.pack) ;
    p90_yacc_SET((int16_t)(int16_t) -23986, PH.base.pack) ;
    p90_zacc_SET((int16_t)(int16_t) -15016, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
    p91_time_usec_SET((uint64_t)8175275076540496198L, PH.base.pack) ;
    p91_roll_ailerons_SET((float) -2.0763344E38F, PH.base.pack) ;
    p91_pitch_elevator_SET((float)1.1470739E38F, PH.base.pack) ;
    p91_yaw_rudder_SET((float) -1.1720378E38F, PH.base.pack) ;
    p91_throttle_SET((float)2.5714453E38F, PH.base.pack) ;
    p91_aux1_SET((float)2.6816954E38F, PH.base.pack) ;
    p91_aux2_SET((float)6.7433024E37F, PH.base.pack) ;
    p91_aux3_SET((float) -3.0891524E38F, PH.base.pack) ;
    p91_aux4_SET((float)4.713858E37F, PH.base.pack) ;
    p91_mode_SET(e_MAV_MODE_MAV_MODE_TEST_ARMED, PH.base.pack) ;
    p91_nav_mode_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
    p92_time_usec_SET((uint64_t)6734527571030624803L, PH.base.pack) ;
    p92_chan1_raw_SET((uint16_t)(uint16_t)54510, PH.base.pack) ;
    p92_chan2_raw_SET((uint16_t)(uint16_t)13182, PH.base.pack) ;
    p92_chan3_raw_SET((uint16_t)(uint16_t)26465, PH.base.pack) ;
    p92_chan4_raw_SET((uint16_t)(uint16_t)26732, PH.base.pack) ;
    p92_chan5_raw_SET((uint16_t)(uint16_t)63223, PH.base.pack) ;
    p92_chan6_raw_SET((uint16_t)(uint16_t)24643, PH.base.pack) ;
    p92_chan7_raw_SET((uint16_t)(uint16_t)50000, PH.base.pack) ;
    p92_chan8_raw_SET((uint16_t)(uint16_t)15111, PH.base.pack) ;
    p92_chan9_raw_SET((uint16_t)(uint16_t)33679, PH.base.pack) ;
    p92_chan10_raw_SET((uint16_t)(uint16_t)22984, PH.base.pack) ;
    p92_chan11_raw_SET((uint16_t)(uint16_t)53205, PH.base.pack) ;
    p92_chan12_raw_SET((uint16_t)(uint16_t)55461, PH.base.pack) ;
    p92_rssi_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
    p93_time_usec_SET((uint64_t)2726178637260607179L, PH.base.pack) ;
    {
        float  controls [] =  {-4.4919625E37F, 6.910042E36F, -1.8576768E38F, -8.98319E37F, 2.2270514E38F, -4.793556E37F, 1.54738E38F, -1.3133122E38F, -2.9745373E38F, 2.1085506E38F, -1.5606844E38F, 2.57357E38F, -1.4359534E38F, 2.3316275E38F, -9.619955E37F, -2.8538121E38F};
        p93_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    p93_mode_SET(e_MAV_MODE_MAV_MODE_STABILIZE_ARMED, PH.base.pack) ;
    p93_flags_SET((uint64_t)2004258051289243705L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
    p100_time_usec_SET((uint64_t)8794203508303720623L, PH.base.pack) ;
    p100_sensor_id_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
    p100_flow_x_SET((int16_t)(int16_t) -26813, PH.base.pack) ;
    p100_flow_y_SET((int16_t)(int16_t)8390, PH.base.pack) ;
    p100_flow_comp_m_x_SET((float) -2.8330717E38F, PH.base.pack) ;
    p100_flow_comp_m_y_SET((float)1.5205219E38F, PH.base.pack) ;
    p100_quality_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
    p100_ground_distance_SET((float)8.51984E37F, PH.base.pack) ;
    p100_flow_rate_x_SET((float) -8.585769E37F, &PH) ;
    p100_flow_rate_y_SET((float)2.7047751E38F, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
    p101_usec_SET((uint64_t)39036506165630457L, PH.base.pack) ;
    p101_x_SET((float)2.3393881E38F, PH.base.pack) ;
    p101_y_SET((float) -1.3419146E38F, PH.base.pack) ;
    p101_z_SET((float) -5.620598E37F, PH.base.pack) ;
    p101_roll_SET((float)8.863658E37F, PH.base.pack) ;
    p101_pitch_SET((float) -2.2300558E38F, PH.base.pack) ;
    p101_yaw_SET((float) -1.271486E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
    p102_usec_SET((uint64_t)758011276318687682L, PH.base.pack) ;
    p102_x_SET((float)9.666026E37F, PH.base.pack) ;
    p102_y_SET((float) -3.2571246E38F, PH.base.pack) ;
    p102_z_SET((float)6.9856533E37F, PH.base.pack) ;
    p102_roll_SET((float) -2.1188794E38F, PH.base.pack) ;
    p102_pitch_SET((float) -1.2006324E38F, PH.base.pack) ;
    p102_yaw_SET((float) -3.3255603E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
    p103_usec_SET((uint64_t)1477374267272482943L, PH.base.pack) ;
    p103_x_SET((float) -1.5692472E37F, PH.base.pack) ;
    p103_y_SET((float)2.0316413E38F, PH.base.pack) ;
    p103_z_SET((float) -1.5726408E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
    p104_usec_SET((uint64_t)9172129376859788689L, PH.base.pack) ;
    p104_x_SET((float)2.4192252E38F, PH.base.pack) ;
    p104_y_SET((float) -3.2048552E38F, PH.base.pack) ;
    p104_z_SET((float) -1.3183307E38F, PH.base.pack) ;
    p104_roll_SET((float) -1.1576192E37F, PH.base.pack) ;
    p104_pitch_SET((float)3.8131574E37F, PH.base.pack) ;
    p104_yaw_SET((float) -1.1818667E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
    p105_time_usec_SET((uint64_t)4687423389652527738L, PH.base.pack) ;
    p105_xacc_SET((float)3.134972E38F, PH.base.pack) ;
    p105_yacc_SET((float)1.0699847E38F, PH.base.pack) ;
    p105_zacc_SET((float)1.8188953E38F, PH.base.pack) ;
    p105_xgyro_SET((float) -1.7762908E38F, PH.base.pack) ;
    p105_ygyro_SET((float)3.3721536E38F, PH.base.pack) ;
    p105_zgyro_SET((float) -8.461262E37F, PH.base.pack) ;
    p105_xmag_SET((float) -9.297854E37F, PH.base.pack) ;
    p105_ymag_SET((float)2.7614178E38F, PH.base.pack) ;
    p105_zmag_SET((float)2.5794144E38F, PH.base.pack) ;
    p105_abs_pressure_SET((float) -6.9772057E37F, PH.base.pack) ;
    p105_diff_pressure_SET((float)1.7258584E38F, PH.base.pack) ;
    p105_pressure_alt_SET((float)3.2019501E38F, PH.base.pack) ;
    p105_temperature_SET((float) -8.0057145E37F, PH.base.pack) ;
    p105_fields_updated_SET((uint16_t)(uint16_t)22303, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
    p106_time_usec_SET((uint64_t)7738516914062548071L, PH.base.pack) ;
    p106_sensor_id_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
    p106_integration_time_us_SET((uint32_t)3662815482L, PH.base.pack) ;
    p106_integrated_x_SET((float)1.7621512E38F, PH.base.pack) ;
    p106_integrated_y_SET((float) -3.3942187E38F, PH.base.pack) ;
    p106_integrated_xgyro_SET((float)9.24108E37F, PH.base.pack) ;
    p106_integrated_ygyro_SET((float)1.5782856E37F, PH.base.pack) ;
    p106_integrated_zgyro_SET((float) -1.5254649E38F, PH.base.pack) ;
    p106_temperature_SET((int16_t)(int16_t)14917, PH.base.pack) ;
    p106_quality_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
    p106_time_delta_distance_us_SET((uint32_t)2330464323L, PH.base.pack) ;
    p106_distance_SET((float)1.3040229E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
    p107_time_usec_SET((uint64_t)7833804347690351924L, PH.base.pack) ;
    p107_xacc_SET((float) -1.0073537E38F, PH.base.pack) ;
    p107_yacc_SET((float)3.2189154E38F, PH.base.pack) ;
    p107_zacc_SET((float)2.3865601E38F, PH.base.pack) ;
    p107_xgyro_SET((float)1.0325277E38F, PH.base.pack) ;
    p107_ygyro_SET((float)2.8168338E38F, PH.base.pack) ;
    p107_zgyro_SET((float) -2.279013E38F, PH.base.pack) ;
    p107_xmag_SET((float) -3.166686E38F, PH.base.pack) ;
    p107_ymag_SET((float)3.3625582E38F, PH.base.pack) ;
    p107_zmag_SET((float)3.270283E37F, PH.base.pack) ;
    p107_abs_pressure_SET((float)1.0857627E38F, PH.base.pack) ;
    p107_diff_pressure_SET((float) -2.1548987E38F, PH.base.pack) ;
    p107_pressure_alt_SET((float)8.3112406E37F, PH.base.pack) ;
    p107_temperature_SET((float) -8.635682E37F, PH.base.pack) ;
    p107_fields_updated_SET((uint32_t)1821669674L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
    p108_q1_SET((float)2.085926E38F, PH.base.pack) ;
    p108_q2_SET((float)1.1926611E38F, PH.base.pack) ;
    p108_q3_SET((float)3.1006014E38F, PH.base.pack) ;
    p108_q4_SET((float) -1.6903655E38F, PH.base.pack) ;
    p108_roll_SET((float)1.7113772E38F, PH.base.pack) ;
    p108_pitch_SET((float)3.2768097E38F, PH.base.pack) ;
    p108_yaw_SET((float)9.24446E37F, PH.base.pack) ;
    p108_xacc_SET((float)1.4717488E37F, PH.base.pack) ;
    p108_yacc_SET((float)2.4952363E37F, PH.base.pack) ;
    p108_zacc_SET((float) -2.8646484E38F, PH.base.pack) ;
    p108_xgyro_SET((float) -9.642756E36F, PH.base.pack) ;
    p108_ygyro_SET((float) -4.8400796E37F, PH.base.pack) ;
    p108_zgyro_SET((float)3.2464104E38F, PH.base.pack) ;
    p108_lat_SET((float)1.9481569E38F, PH.base.pack) ;
    p108_lon_SET((float)5.482393E37F, PH.base.pack) ;
    p108_alt_SET((float) -2.4570599E36F, PH.base.pack) ;
    p108_std_dev_horz_SET((float) -9.527103E37F, PH.base.pack) ;
    p108_std_dev_vert_SET((float) -3.287499E38F, PH.base.pack) ;
    p108_vn_SET((float) -2.614415E38F, PH.base.pack) ;
    p108_ve_SET((float) -4.267082E37F, PH.base.pack) ;
    p108_vd_SET((float) -5.2082667E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
    p109_rssi_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
    p109_remrssi_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
    p109_txbuf_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
    p109_noise_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
    p109_remnoise_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
    p109_rxerrors_SET((uint16_t)(uint16_t)47351, PH.base.pack) ;
    p109_fixed__SET((uint16_t)(uint16_t)20148, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
    p110_target_network_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
    p110_target_system_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
    p110_target_component_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)27, (uint8_t)248, (uint8_t)85, (uint8_t)230, (uint8_t)185, (uint8_t)133, (uint8_t)41, (uint8_t)240, (uint8_t)19, (uint8_t)91, (uint8_t)211, (uint8_t)232, (uint8_t)251, (uint8_t)107, (uint8_t)87, (uint8_t)57, (uint8_t)242, (uint8_t)20, (uint8_t)22, (uint8_t)12, (uint8_t)192, (uint8_t)14, (uint8_t)31, (uint8_t)128, (uint8_t)109, (uint8_t)203, (uint8_t)2, (uint8_t)186, (uint8_t)162, (uint8_t)147, (uint8_t)91, (uint8_t)29, (uint8_t)186, (uint8_t)73, (uint8_t)230, (uint8_t)162, (uint8_t)249, (uint8_t)64, (uint8_t)97, (uint8_t)43, (uint8_t)14, (uint8_t)211, (uint8_t)197, (uint8_t)51, (uint8_t)32, (uint8_t)173, (uint8_t)30, (uint8_t)70, (uint8_t)23, (uint8_t)7, (uint8_t)242, (uint8_t)151, (uint8_t)242, (uint8_t)34, (uint8_t)62, (uint8_t)39, (uint8_t)75, (uint8_t)156, (uint8_t)105, (uint8_t)153, (uint8_t)122, (uint8_t)118, (uint8_t)221, (uint8_t)10, (uint8_t)124, (uint8_t)156, (uint8_t)174, (uint8_t)11, (uint8_t)31, (uint8_t)255, (uint8_t)157, (uint8_t)220, (uint8_t)167, (uint8_t)61, (uint8_t)125, (uint8_t)82, (uint8_t)160, (uint8_t)90, (uint8_t)158, (uint8_t)168, (uint8_t)128, (uint8_t)90, (uint8_t)161, (uint8_t)85, (uint8_t)91, (uint8_t)189, (uint8_t)239, (uint8_t)214, (uint8_t)81, (uint8_t)93, (uint8_t)212, (uint8_t)205, (uint8_t)10, (uint8_t)125, (uint8_t)215, (uint8_t)54, (uint8_t)78, (uint8_t)135, (uint8_t)12, (uint8_t)224, (uint8_t)58, (uint8_t)230, (uint8_t)228, (uint8_t)216, (uint8_t)8, (uint8_t)157, (uint8_t)50, (uint8_t)208, (uint8_t)89, (uint8_t)160, (uint8_t)178, (uint8_t)6, (uint8_t)161, (uint8_t)51, (uint8_t)41, (uint8_t)57, (uint8_t)240, (uint8_t)157, (uint8_t)53, (uint8_t)32, (uint8_t)205, (uint8_t)28, (uint8_t)18, (uint8_t)85, (uint8_t)127, (uint8_t)152, (uint8_t)76, (uint8_t)214, (uint8_t)203, (uint8_t)170, (uint8_t)174, (uint8_t)124, (uint8_t)28, (uint8_t)37, (uint8_t)72, (uint8_t)211, (uint8_t)35, (uint8_t)237, (uint8_t)186, (uint8_t)227, (uint8_t)182, (uint8_t)204, (uint8_t)95, (uint8_t)194, (uint8_t)71, (uint8_t)68, (uint8_t)239, (uint8_t)237, (uint8_t)230, (uint8_t)164, (uint8_t)250, (uint8_t)41, (uint8_t)45, (uint8_t)254, (uint8_t)42, (uint8_t)182, (uint8_t)122, (uint8_t)85, (uint8_t)59, (uint8_t)157, (uint8_t)9, (uint8_t)163, (uint8_t)111, (uint8_t)220, (uint8_t)227, (uint8_t)82, (uint8_t)195, (uint8_t)226, (uint8_t)139, (uint8_t)235, (uint8_t)80, (uint8_t)28, (uint8_t)151, (uint8_t)253, (uint8_t)112, (uint8_t)77, (uint8_t)85, (uint8_t)90, (uint8_t)34, (uint8_t)143, (uint8_t)67, (uint8_t)47, (uint8_t)211, (uint8_t)190, (uint8_t)84, (uint8_t)45, (uint8_t)182, (uint8_t)78, (uint8_t)43, (uint8_t)4, (uint8_t)203, (uint8_t)106, (uint8_t)229, (uint8_t)245, (uint8_t)65, (uint8_t)241, (uint8_t)182, (uint8_t)104, (uint8_t)227, (uint8_t)34, (uint8_t)136, (uint8_t)128, (uint8_t)122, (uint8_t)42, (uint8_t)6, (uint8_t)248, (uint8_t)253, (uint8_t)147, (uint8_t)183, (uint8_t)163, (uint8_t)23, (uint8_t)228, (uint8_t)182, (uint8_t)151, (uint8_t)191, (uint8_t)164, (uint8_t)186, (uint8_t)145, (uint8_t)211, (uint8_t)43, (uint8_t)222, (uint8_t)239, (uint8_t)18, (uint8_t)218, (uint8_t)122, (uint8_t)242, (uint8_t)174, (uint8_t)3, (uint8_t)244, (uint8_t)57, (uint8_t)185, (uint8_t)180, (uint8_t)138, (uint8_t)13, (uint8_t)245, (uint8_t)39, (uint8_t)21, (uint8_t)76, (uint8_t)19, (uint8_t)123, (uint8_t)238, (uint8_t)149, (uint8_t)206, (uint8_t)101, (uint8_t)162, (uint8_t)186, (uint8_t)130, (uint8_t)164, (uint8_t)204, (uint8_t)117, (uint8_t)55};
        p110_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
    p111_tc1_SET((int64_t) -7874107046264322886L, PH.base.pack) ;
    p111_ts1_SET((int64_t) -8376860871772575566L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
    p112_time_usec_SET((uint64_t)9210911378659716293L, PH.base.pack) ;
    p112_seq_SET((uint32_t)3788981883L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
    p113_time_usec_SET((uint64_t)8957657935917503734L, PH.base.pack) ;
    p113_fix_type_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
    p113_lat_SET((int32_t) -1665058800, PH.base.pack) ;
    p113_lon_SET((int32_t)1150537956, PH.base.pack) ;
    p113_alt_SET((int32_t)346278090, PH.base.pack) ;
    p113_eph_SET((uint16_t)(uint16_t)62242, PH.base.pack) ;
    p113_epv_SET((uint16_t)(uint16_t)36898, PH.base.pack) ;
    p113_vel_SET((uint16_t)(uint16_t)54581, PH.base.pack) ;
    p113_vn_SET((int16_t)(int16_t)22496, PH.base.pack) ;
    p113_ve_SET((int16_t)(int16_t)13387, PH.base.pack) ;
    p113_vd_SET((int16_t)(int16_t) -4817, PH.base.pack) ;
    p113_cog_SET((uint16_t)(uint16_t)3078, PH.base.pack) ;
    p113_satellites_visible_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
    p114_time_usec_SET((uint64_t)7558846145237443452L, PH.base.pack) ;
    p114_sensor_id_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
    p114_integration_time_us_SET((uint32_t)1487654077L, PH.base.pack) ;
    p114_integrated_x_SET((float)7.262159E37F, PH.base.pack) ;
    p114_integrated_y_SET((float)2.9702486E38F, PH.base.pack) ;
    p114_integrated_xgyro_SET((float) -2.7302535E38F, PH.base.pack) ;
    p114_integrated_ygyro_SET((float)9.834438E37F, PH.base.pack) ;
    p114_integrated_zgyro_SET((float)1.753603E38F, PH.base.pack) ;
    p114_temperature_SET((int16_t)(int16_t) -6600, PH.base.pack) ;
    p114_quality_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
    p114_time_delta_distance_us_SET((uint32_t)3813128744L, PH.base.pack) ;
    p114_distance_SET((float)2.204598E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
    p115_time_usec_SET((uint64_t)3871083405387793357L, PH.base.pack) ;
    {
        float  attitude_quaternion [] =  {-7.211338E37F, -2.2752103E38F, 1.0439552E37F, -2.568307E38F};
        p115_attitude_quaternion_SET(&attitude_quaternion, 0, &PH.base.pack) ;
    }
    p115_rollspeed_SET((float) -1.9971075E38F, PH.base.pack) ;
    p115_pitchspeed_SET((float)3.913848E37F, PH.base.pack) ;
    p115_yawspeed_SET((float)2.513846E38F, PH.base.pack) ;
    p115_lat_SET((int32_t)416811420, PH.base.pack) ;
    p115_lon_SET((int32_t)417892738, PH.base.pack) ;
    p115_alt_SET((int32_t)218109135, PH.base.pack) ;
    p115_vx_SET((int16_t)(int16_t) -21423, PH.base.pack) ;
    p115_vy_SET((int16_t)(int16_t) -28014, PH.base.pack) ;
    p115_vz_SET((int16_t)(int16_t)19382, PH.base.pack) ;
    p115_ind_airspeed_SET((uint16_t)(uint16_t)65441, PH.base.pack) ;
    p115_true_airspeed_SET((uint16_t)(uint16_t)9245, PH.base.pack) ;
    p115_xacc_SET((int16_t)(int16_t) -922, PH.base.pack) ;
    p115_yacc_SET((int16_t)(int16_t)15203, PH.base.pack) ;
    p115_zacc_SET((int16_t)(int16_t)7115, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
    p116_time_boot_ms_SET((uint32_t)4057578442L, PH.base.pack) ;
    p116_xacc_SET((int16_t)(int16_t)32127, PH.base.pack) ;
    p116_yacc_SET((int16_t)(int16_t) -7523, PH.base.pack) ;
    p116_zacc_SET((int16_t)(int16_t) -17166, PH.base.pack) ;
    p116_xgyro_SET((int16_t)(int16_t) -31910, PH.base.pack) ;
    p116_ygyro_SET((int16_t)(int16_t)27085, PH.base.pack) ;
    p116_zgyro_SET((int16_t)(int16_t) -29808, PH.base.pack) ;
    p116_xmag_SET((int16_t)(int16_t)7480, PH.base.pack) ;
    p116_ymag_SET((int16_t)(int16_t) -21187, PH.base.pack) ;
    p116_zmag_SET((int16_t)(int16_t) -8740, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
    p117_target_system_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
    p117_target_component_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
    p117_start_SET((uint16_t)(uint16_t)58544, PH.base.pack) ;
    p117_end_SET((uint16_t)(uint16_t)17467, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
    p118_id_SET((uint16_t)(uint16_t)47857, PH.base.pack) ;
    p118_num_logs_SET((uint16_t)(uint16_t)56847, PH.base.pack) ;
    p118_last_log_num_SET((uint16_t)(uint16_t)31415, PH.base.pack) ;
    p118_time_utc_SET((uint32_t)2781239400L, PH.base.pack) ;
    p118_size_SET((uint32_t)2135085327L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
    p119_target_system_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
    p119_target_component_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
    p119_id_SET((uint16_t)(uint16_t)27027, PH.base.pack) ;
    p119_ofs_SET((uint32_t)3487118813L, PH.base.pack) ;
    p119_count_SET((uint32_t)2377608491L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
    p120_id_SET((uint16_t)(uint16_t)27565, PH.base.pack) ;
    p120_ofs_SET((uint32_t)2741886818L, PH.base.pack) ;
    p120_count_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)114, (uint8_t)35, (uint8_t)122, (uint8_t)170, (uint8_t)251, (uint8_t)106, (uint8_t)74, (uint8_t)22, (uint8_t)142, (uint8_t)17, (uint8_t)155, (uint8_t)78, (uint8_t)204, (uint8_t)136, (uint8_t)3, (uint8_t)131, (uint8_t)151, (uint8_t)228, (uint8_t)55, (uint8_t)166, (uint8_t)131, (uint8_t)200, (uint8_t)164, (uint8_t)72, (uint8_t)193, (uint8_t)18, (uint8_t)181, (uint8_t)118, (uint8_t)189, (uint8_t)158, (uint8_t)153, (uint8_t)89, (uint8_t)180, (uint8_t)80, (uint8_t)20, (uint8_t)99, (uint8_t)5, (uint8_t)23, (uint8_t)18, (uint8_t)127, (uint8_t)246, (uint8_t)249, (uint8_t)246, (uint8_t)188, (uint8_t)207, (uint8_t)92, (uint8_t)77, (uint8_t)69, (uint8_t)216, (uint8_t)245, (uint8_t)77, (uint8_t)38, (uint8_t)252, (uint8_t)43, (uint8_t)29, (uint8_t)10, (uint8_t)8, (uint8_t)197, (uint8_t)144, (uint8_t)158, (uint8_t)13, (uint8_t)68, (uint8_t)28, (uint8_t)56, (uint8_t)89, (uint8_t)71, (uint8_t)233, (uint8_t)202, (uint8_t)79, (uint8_t)134, (uint8_t)206, (uint8_t)246, (uint8_t)174, (uint8_t)243, (uint8_t)168, (uint8_t)26, (uint8_t)150, (uint8_t)77, (uint8_t)94, (uint8_t)206, (uint8_t)177, (uint8_t)197, (uint8_t)41, (uint8_t)39, (uint8_t)116, (uint8_t)63, (uint8_t)220, (uint8_t)100, (uint8_t)142, (uint8_t)208};
        p120_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
    p121_target_system_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
    p121_target_component_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
    p122_target_system_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
    p122_target_component_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
    p123_target_system_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
    p123_target_component_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
    p123_len_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)126, (uint8_t)121, (uint8_t)189, (uint8_t)122, (uint8_t)4, (uint8_t)15, (uint8_t)238, (uint8_t)126, (uint8_t)236, (uint8_t)31, (uint8_t)12, (uint8_t)158, (uint8_t)192, (uint8_t)176, (uint8_t)152, (uint8_t)175, (uint8_t)193, (uint8_t)225, (uint8_t)2, (uint8_t)46, (uint8_t)182, (uint8_t)232, (uint8_t)148, (uint8_t)229, (uint8_t)116, (uint8_t)86, (uint8_t)125, (uint8_t)56, (uint8_t)123, (uint8_t)115, (uint8_t)117, (uint8_t)244, (uint8_t)45, (uint8_t)207, (uint8_t)252, (uint8_t)19, (uint8_t)40, (uint8_t)0, (uint8_t)5, (uint8_t)2, (uint8_t)5, (uint8_t)203, (uint8_t)22, (uint8_t)7, (uint8_t)28, (uint8_t)1, (uint8_t)58, (uint8_t)57, (uint8_t)8, (uint8_t)179, (uint8_t)115, (uint8_t)205, (uint8_t)22, (uint8_t)139, (uint8_t)117, (uint8_t)49, (uint8_t)144, (uint8_t)200, (uint8_t)12, (uint8_t)18, (uint8_t)84, (uint8_t)48, (uint8_t)130, (uint8_t)71, (uint8_t)110, (uint8_t)25, (uint8_t)111, (uint8_t)176, (uint8_t)198, (uint8_t)76, (uint8_t)14, (uint8_t)38, (uint8_t)230, (uint8_t)61, (uint8_t)204, (uint8_t)16, (uint8_t)76, (uint8_t)152, (uint8_t)9, (uint8_t)40, (uint8_t)177, (uint8_t)98, (uint8_t)92, (uint8_t)28, (uint8_t)178, (uint8_t)6, (uint8_t)171, (uint8_t)228, (uint8_t)111, (uint8_t)66, (uint8_t)147, (uint8_t)131, (uint8_t)30, (uint8_t)42, (uint8_t)161, (uint8_t)205, (uint8_t)178, (uint8_t)157, (uint8_t)205, (uint8_t)88, (uint8_t)171, (uint8_t)14, (uint8_t)229, (uint8_t)78, (uint8_t)81, (uint8_t)187, (uint8_t)157, (uint8_t)126, (uint8_t)30, (uint8_t)44};
        p123_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
    p124_time_usec_SET((uint64_t)5244988860544249721L, PH.base.pack) ;
    p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, PH.base.pack) ;
    p124_lat_SET((int32_t) -318016350, PH.base.pack) ;
    p124_lon_SET((int32_t)2032387544, PH.base.pack) ;
    p124_alt_SET((int32_t) -2080246388, PH.base.pack) ;
    p124_eph_SET((uint16_t)(uint16_t)5385, PH.base.pack) ;
    p124_epv_SET((uint16_t)(uint16_t)20006, PH.base.pack) ;
    p124_vel_SET((uint16_t)(uint16_t)31247, PH.base.pack) ;
    p124_cog_SET((uint16_t)(uint16_t)27264, PH.base.pack) ;
    p124_satellites_visible_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
    p124_dgps_numch_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
    p124_dgps_age_SET((uint32_t)1531631417L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
    p125_Vcc_SET((uint16_t)(uint16_t)5976, PH.base.pack) ;
    p125_Vservo_SET((uint16_t)(uint16_t)35217, PH.base.pack) ;
    p125_flags_SET((e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED |
                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT |
                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED |
                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID |
                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT), PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
    p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM2, PH.base.pack) ;
    p126_flags_SET((e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE |
                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND), PH.base.pack) ;
    p126_timeout_SET((uint16_t)(uint16_t)6661, PH.base.pack) ;
    p126_baudrate_SET((uint32_t)2102353576L, PH.base.pack) ;
    p126_count_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)170, (uint8_t)136, (uint8_t)40, (uint8_t)16, (uint8_t)206, (uint8_t)249, (uint8_t)201, (uint8_t)203, (uint8_t)148, (uint8_t)75, (uint8_t)59, (uint8_t)185, (uint8_t)55, (uint8_t)85, (uint8_t)21, (uint8_t)224, (uint8_t)19, (uint8_t)41, (uint8_t)43, (uint8_t)104, (uint8_t)224, (uint8_t)4, (uint8_t)45, (uint8_t)53, (uint8_t)200, (uint8_t)126, (uint8_t)95, (uint8_t)84, (uint8_t)48, (uint8_t)172, (uint8_t)88, (uint8_t)186, (uint8_t)193, (uint8_t)136, (uint8_t)78, (uint8_t)166, (uint8_t)222, (uint8_t)58, (uint8_t)196, (uint8_t)93, (uint8_t)231, (uint8_t)255, (uint8_t)181, (uint8_t)241, (uint8_t)220, (uint8_t)86, (uint8_t)221, (uint8_t)197, (uint8_t)47, (uint8_t)55, (uint8_t)159, (uint8_t)226, (uint8_t)244, (uint8_t)80, (uint8_t)91, (uint8_t)220, (uint8_t)72, (uint8_t)227, (uint8_t)38, (uint8_t)87, (uint8_t)241, (uint8_t)108, (uint8_t)108, (uint8_t)127, (uint8_t)132, (uint8_t)20, (uint8_t)51, (uint8_t)99, (uint8_t)140, (uint8_t)101};
        p126_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
    p127_time_last_baseline_ms_SET((uint32_t)2709138015L, PH.base.pack) ;
    p127_rtk_receiver_id_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
    p127_wn_SET((uint16_t)(uint16_t)35551, PH.base.pack) ;
    p127_tow_SET((uint32_t)3506341149L, PH.base.pack) ;
    p127_rtk_health_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
    p127_rtk_rate_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
    p127_nsats_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
    p127_baseline_coords_type_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
    p127_baseline_a_mm_SET((int32_t)1147565713, PH.base.pack) ;
    p127_baseline_b_mm_SET((int32_t)146048738, PH.base.pack) ;
    p127_baseline_c_mm_SET((int32_t) -336072945, PH.base.pack) ;
    p127_accuracy_SET((uint32_t)2980184128L, PH.base.pack) ;
    p127_iar_num_hypotheses_SET((int32_t) -1948725900, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
    p128_time_last_baseline_ms_SET((uint32_t)3998127840L, PH.base.pack) ;
    p128_rtk_receiver_id_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
    p128_wn_SET((uint16_t)(uint16_t)64242, PH.base.pack) ;
    p128_tow_SET((uint32_t)38767383L, PH.base.pack) ;
    p128_rtk_health_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
    p128_rtk_rate_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
    p128_nsats_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
    p128_baseline_coords_type_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
    p128_baseline_a_mm_SET((int32_t) -1597296220, PH.base.pack) ;
    p128_baseline_b_mm_SET((int32_t) -11383768, PH.base.pack) ;
    p128_baseline_c_mm_SET((int32_t) -1898309723, PH.base.pack) ;
    p128_accuracy_SET((uint32_t)2666564521L, PH.base.pack) ;
    p128_iar_num_hypotheses_SET((int32_t) -1113964005, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
    p129_time_boot_ms_SET((uint32_t)1238758888L, PH.base.pack) ;
    p129_xacc_SET((int16_t)(int16_t) -21770, PH.base.pack) ;
    p129_yacc_SET((int16_t)(int16_t)31109, PH.base.pack) ;
    p129_zacc_SET((int16_t)(int16_t) -10905, PH.base.pack) ;
    p129_xgyro_SET((int16_t)(int16_t)19231, PH.base.pack) ;
    p129_ygyro_SET((int16_t)(int16_t)4763, PH.base.pack) ;
    p129_zgyro_SET((int16_t)(int16_t) -21347, PH.base.pack) ;
    p129_xmag_SET((int16_t)(int16_t) -6722, PH.base.pack) ;
    p129_ymag_SET((int16_t)(int16_t) -31335, PH.base.pack) ;
    p129_zmag_SET((int16_t)(int16_t)7781, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
    p130_type_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    p130_size_SET((uint32_t)3205265637L, PH.base.pack) ;
    p130_width_SET((uint16_t)(uint16_t)15929, PH.base.pack) ;
    p130_height_SET((uint16_t)(uint16_t)27016, PH.base.pack) ;
    p130_packets_SET((uint16_t)(uint16_t)36623, PH.base.pack) ;
    p130_payload_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
    p130_jpg_quality_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
    p131_seqnr_SET((uint16_t)(uint16_t)50281, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)129, (uint8_t)240, (uint8_t)133, (uint8_t)143, (uint8_t)52, (uint8_t)252, (uint8_t)40, (uint8_t)131, (uint8_t)241, (uint8_t)60, (uint8_t)240, (uint8_t)135, (uint8_t)142, (uint8_t)17, (uint8_t)25, (uint8_t)231, (uint8_t)136, (uint8_t)219, (uint8_t)113, (uint8_t)99, (uint8_t)27, (uint8_t)107, (uint8_t)10, (uint8_t)142, (uint8_t)136, (uint8_t)30, (uint8_t)165, (uint8_t)248, (uint8_t)220, (uint8_t)149, (uint8_t)64, (uint8_t)74, (uint8_t)172, (uint8_t)117, (uint8_t)36, (uint8_t)39, (uint8_t)193, (uint8_t)144, (uint8_t)155, (uint8_t)96, (uint8_t)116, (uint8_t)221, (uint8_t)251, (uint8_t)30, (uint8_t)167, (uint8_t)27, (uint8_t)135, (uint8_t)53, (uint8_t)246, (uint8_t)254, (uint8_t)18, (uint8_t)83, (uint8_t)77, (uint8_t)133, (uint8_t)130, (uint8_t)101, (uint8_t)34, (uint8_t)12, (uint8_t)1, (uint8_t)158, (uint8_t)18, (uint8_t)34, (uint8_t)115, (uint8_t)226, (uint8_t)250, (uint8_t)170, (uint8_t)233, (uint8_t)111, (uint8_t)205, (uint8_t)158, (uint8_t)25, (uint8_t)51, (uint8_t)191, (uint8_t)171, (uint8_t)84, (uint8_t)42, (uint8_t)85, (uint8_t)44, (uint8_t)178, (uint8_t)75, (uint8_t)253, (uint8_t)53, (uint8_t)107, (uint8_t)5, (uint8_t)182, (uint8_t)9, (uint8_t)40, (uint8_t)254, (uint8_t)146, (uint8_t)221, (uint8_t)235, (uint8_t)142, (uint8_t)109, (uint8_t)60, (uint8_t)98, (uint8_t)52, (uint8_t)36, (uint8_t)235, (uint8_t)34, (uint8_t)173, (uint8_t)246, (uint8_t)1, (uint8_t)56, (uint8_t)130, (uint8_t)173, (uint8_t)92, (uint8_t)230, (uint8_t)226, (uint8_t)213, (uint8_t)8, (uint8_t)53, (uint8_t)46, (uint8_t)249, (uint8_t)75, (uint8_t)100, (uint8_t)39, (uint8_t)74, (uint8_t)106, (uint8_t)127, (uint8_t)2, (uint8_t)227, (uint8_t)67, (uint8_t)202, (uint8_t)33, (uint8_t)133, (uint8_t)213, (uint8_t)69, (uint8_t)194, (uint8_t)243, (uint8_t)136, (uint8_t)29, (uint8_t)146, (uint8_t)253, (uint8_t)4, (uint8_t)153, (uint8_t)229, (uint8_t)161, (uint8_t)96, (uint8_t)253, (uint8_t)119, (uint8_t)44, (uint8_t)230, (uint8_t)165, (uint8_t)167, (uint8_t)167, (uint8_t)19, (uint8_t)82, (uint8_t)226, (uint8_t)204, (uint8_t)26, (uint8_t)15, (uint8_t)247, (uint8_t)178, (uint8_t)160, (uint8_t)115, (uint8_t)11, (uint8_t)44, (uint8_t)100, (uint8_t)164, (uint8_t)87, (uint8_t)49, (uint8_t)217, (uint8_t)96, (uint8_t)106, (uint8_t)84, (uint8_t)144, (uint8_t)176, (uint8_t)53, (uint8_t)241, (uint8_t)192, (uint8_t)54, (uint8_t)188, (uint8_t)31, (uint8_t)64, (uint8_t)223, (uint8_t)95, (uint8_t)238, (uint8_t)219, (uint8_t)192, (uint8_t)248, (uint8_t)45, (uint8_t)244, (uint8_t)66, (uint8_t)58, (uint8_t)215, (uint8_t)194, (uint8_t)177, (uint8_t)165, (uint8_t)169, (uint8_t)157, (uint8_t)104, (uint8_t)20, (uint8_t)48, (uint8_t)45, (uint8_t)209, (uint8_t)255, (uint8_t)31, (uint8_t)42, (uint8_t)115, (uint8_t)217, (uint8_t)156, (uint8_t)23, (uint8_t)101, (uint8_t)146, (uint8_t)76, (uint8_t)31, (uint8_t)44, (uint8_t)223, (uint8_t)88, (uint8_t)136, (uint8_t)232, (uint8_t)193, (uint8_t)199, (uint8_t)175, (uint8_t)100, (uint8_t)231, (uint8_t)105, (uint8_t)129, (uint8_t)7, (uint8_t)15, (uint8_t)178, (uint8_t)102, (uint8_t)6, (uint8_t)189, (uint8_t)177, (uint8_t)238, (uint8_t)118, (uint8_t)85, (uint8_t)44, (uint8_t)192, (uint8_t)248, (uint8_t)94, (uint8_t)68, (uint8_t)60, (uint8_t)115, (uint8_t)212, (uint8_t)41, (uint8_t)171, (uint8_t)245, (uint8_t)218, (uint8_t)87, (uint8_t)126, (uint8_t)6, (uint8_t)17, (uint8_t)97, (uint8_t)216, (uint8_t)50, (uint8_t)253, (uint8_t)112, (uint8_t)136, (uint8_t)138, (uint8_t)238, (uint8_t)41};
        p131_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
    p132_time_boot_ms_SET((uint32_t)1654631098L, PH.base.pack) ;
    p132_min_distance_SET((uint16_t)(uint16_t)28294, PH.base.pack) ;
    p132_max_distance_SET((uint16_t)(uint16_t)54457, PH.base.pack) ;
    p132_current_distance_SET((uint16_t)(uint16_t)42399, PH.base.pack) ;
    p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
    p132_id_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
    p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_YAW_315, PH.base.pack) ;
    p132_covariance_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
    p133_lat_SET((int32_t) -1873199016, PH.base.pack) ;
    p133_lon_SET((int32_t)597139839, PH.base.pack) ;
    p133_grid_spacing_SET((uint16_t)(uint16_t)37039, PH.base.pack) ;
    p133_mask_SET((uint64_t)5893907370289240913L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
    p134_lat_SET((int32_t)900692200, PH.base.pack) ;
    p134_lon_SET((int32_t)924958172, PH.base.pack) ;
    p134_grid_spacing_SET((uint16_t)(uint16_t)17343, PH.base.pack) ;
    p134_gridbit_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
    {
        int16_t  data_ [] =  {(int16_t)14110, (int16_t) -1820, (int16_t) -25233, (int16_t)18097, (int16_t) -31812, (int16_t)17359, (int16_t) -21260, (int16_t) -3739, (int16_t) -31644, (int16_t)13061, (int16_t)12469, (int16_t) -13771, (int16_t)14129, (int16_t) -10571, (int16_t) -27057, (int16_t) -21872};
        p134_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
    p135_lat_SET((int32_t)1337496017, PH.base.pack) ;
    p135_lon_SET((int32_t)1201779145, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
    p136_lat_SET((int32_t)370931207, PH.base.pack) ;
    p136_lon_SET((int32_t)1430838989, PH.base.pack) ;
    p136_spacing_SET((uint16_t)(uint16_t)26176, PH.base.pack) ;
    p136_terrain_height_SET((float) -2.516343E38F, PH.base.pack) ;
    p136_current_height_SET((float)1.02779E38F, PH.base.pack) ;
    p136_pending_SET((uint16_t)(uint16_t)7095, PH.base.pack) ;
    p136_loaded_SET((uint16_t)(uint16_t)21760, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
    p137_time_boot_ms_SET((uint32_t)3430616688L, PH.base.pack) ;
    p137_press_abs_SET((float) -3.4553486E37F, PH.base.pack) ;
    p137_press_diff_SET((float)3.0114516E37F, PH.base.pack) ;
    p137_temperature_SET((int16_t)(int16_t)9124, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
    p138_time_usec_SET((uint64_t)4021941943000076442L, PH.base.pack) ;
    {
        float  q [] =  {2.7605864E38F, -7.4939827E37F, 1.5746763E37F, -2.7445321E38F};
        p138_q_SET(&q, 0, &PH.base.pack) ;
    }
    p138_x_SET((float)2.1470234E37F, PH.base.pack) ;
    p138_y_SET((float)1.1207535E38F, PH.base.pack) ;
    p138_z_SET((float)2.8973065E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
    p139_time_usec_SET((uint64_t)3510855270352019834L, PH.base.pack) ;
    p139_group_mlx_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
    p139_target_system_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
    p139_target_component_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
    {
        float  controls [] =  {-5.7768983E37F, 1.5335958E38F, -3.237539E37F, 8.931324E37F, 3.2386424E38F, 4.2076396E37F, 7.209477E37F, 3.0225417E38F};
        p139_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
    p140_time_usec_SET((uint64_t)6893350764812749782L, PH.base.pack) ;
    p140_group_mlx_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
    {
        float  controls [] =  {-1.2088719E38F, -1.4662578E38F, -1.0272625E38F, 2.662195E38F, -4.0247197E36F, 3.095317E38F, 2.8568603E38F, 9.277505E37F};
        p140_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ALTITUDE_141(), &PH);
    p141_time_usec_SET((uint64_t)4297878497038594848L, PH.base.pack) ;
    p141_altitude_monotonic_SET((float) -2.7006288E38F, PH.base.pack) ;
    p141_altitude_amsl_SET((float)1.8429115E38F, PH.base.pack) ;
    p141_altitude_local_SET((float)2.1544349E38F, PH.base.pack) ;
    p141_altitude_relative_SET((float) -4.2626455E37F, PH.base.pack) ;
    p141_altitude_terrain_SET((float)1.4997115E38F, PH.base.pack) ;
    p141_bottom_clearance_SET((float)2.4460175E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RESOURCE_REQUEST_142(), &PH);
    p142_request_id_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
    p142_uri_type_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
    {
        uint8_t  uri [] =  {(uint8_t)171, (uint8_t)176, (uint8_t)2, (uint8_t)184, (uint8_t)202, (uint8_t)128, (uint8_t)87, (uint8_t)241, (uint8_t)191, (uint8_t)240, (uint8_t)232, (uint8_t)178, (uint8_t)122, (uint8_t)69, (uint8_t)139, (uint8_t)116, (uint8_t)77, (uint8_t)64, (uint8_t)119, (uint8_t)221, (uint8_t)201, (uint8_t)192, (uint8_t)211, (uint8_t)127, (uint8_t)78, (uint8_t)147, (uint8_t)29, (uint8_t)124, (uint8_t)136, (uint8_t)173, (uint8_t)232, (uint8_t)242, (uint8_t)186, (uint8_t)95, (uint8_t)156, (uint8_t)9, (uint8_t)182, (uint8_t)18, (uint8_t)223, (uint8_t)94, (uint8_t)74, (uint8_t)179, (uint8_t)0, (uint8_t)106, (uint8_t)237, (uint8_t)40, (uint8_t)181, (uint8_t)163, (uint8_t)253, (uint8_t)175, (uint8_t)30, (uint8_t)244, (uint8_t)77, (uint8_t)209, (uint8_t)228, (uint8_t)125, (uint8_t)22, (uint8_t)178, (uint8_t)234, (uint8_t)139, (uint8_t)223, (uint8_t)138, (uint8_t)132, (uint8_t)201, (uint8_t)55, (uint8_t)150, (uint8_t)154, (uint8_t)61, (uint8_t)246, (uint8_t)32, (uint8_t)138, (uint8_t)241, (uint8_t)43, (uint8_t)209, (uint8_t)141, (uint8_t)123, (uint8_t)112, (uint8_t)52, (uint8_t)143, (uint8_t)35, (uint8_t)151, (uint8_t)0, (uint8_t)166, (uint8_t)65, (uint8_t)134, (uint8_t)54, (uint8_t)190, (uint8_t)193, (uint8_t)203, (uint8_t)84, (uint8_t)209, (uint8_t)214, (uint8_t)118, (uint8_t)171, (uint8_t)192, (uint8_t)158, (uint8_t)56, (uint8_t)3, (uint8_t)219, (uint8_t)87, (uint8_t)235, (uint8_t)202, (uint8_t)31, (uint8_t)65, (uint8_t)250, (uint8_t)182, (uint8_t)218, (uint8_t)107, (uint8_t)235, (uint8_t)9, (uint8_t)110, (uint8_t)66, (uint8_t)46, (uint8_t)168, (uint8_t)58, (uint8_t)103, (uint8_t)30, (uint8_t)32, (uint8_t)83, (uint8_t)86};
        p142_uri_SET(&uri, 0, &PH.base.pack) ;
    }
    p142_transfer_type_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
    {
        uint8_t  storage [] =  {(uint8_t)64, (uint8_t)18, (uint8_t)132, (uint8_t)62, (uint8_t)66, (uint8_t)62, (uint8_t)171, (uint8_t)73, (uint8_t)97, (uint8_t)83, (uint8_t)193, (uint8_t)250, (uint8_t)198, (uint8_t)16, (uint8_t)113, (uint8_t)202, (uint8_t)226, (uint8_t)5, (uint8_t)91, (uint8_t)44, (uint8_t)45, (uint8_t)41, (uint8_t)186, (uint8_t)115, (uint8_t)134, (uint8_t)115, (uint8_t)216, (uint8_t)68, (uint8_t)91, (uint8_t)113, (uint8_t)25, (uint8_t)185, (uint8_t)243, (uint8_t)183, (uint8_t)153, (uint8_t)20, (uint8_t)62, (uint8_t)152, (uint8_t)97, (uint8_t)230, (uint8_t)136, (uint8_t)56, (uint8_t)82, (uint8_t)188, (uint8_t)172, (uint8_t)144, (uint8_t)11, (uint8_t)100, (uint8_t)133, (uint8_t)117, (uint8_t)85, (uint8_t)76, (uint8_t)39, (uint8_t)158, (uint8_t)206, (uint8_t)34, (uint8_t)189, (uint8_t)106, (uint8_t)240, (uint8_t)19, (uint8_t)233, (uint8_t)130, (uint8_t)213, (uint8_t)169, (uint8_t)107, (uint8_t)225, (uint8_t)233, (uint8_t)137, (uint8_t)185, (uint8_t)218, (uint8_t)61, (uint8_t)146, (uint8_t)110, (uint8_t)109, (uint8_t)162, (uint8_t)14, (uint8_t)92, (uint8_t)152, (uint8_t)193, (uint8_t)179, (uint8_t)170, (uint8_t)67, (uint8_t)145, (uint8_t)12, (uint8_t)107, (uint8_t)7, (uint8_t)69, (uint8_t)12, (uint8_t)61, (uint8_t)224, (uint8_t)27, (uint8_t)103, (uint8_t)117, (uint8_t)79, (uint8_t)13, (uint8_t)6, (uint8_t)22, (uint8_t)150, (uint8_t)113, (uint8_t)188, (uint8_t)130, (uint8_t)24, (uint8_t)208, (uint8_t)117, (uint8_t)98, (uint8_t)186, (uint8_t)3, (uint8_t)255, (uint8_t)173, (uint8_t)164, (uint8_t)68, (uint8_t)46, (uint8_t)208, (uint8_t)11, (uint8_t)113, (uint8_t)108, (uint8_t)227, (uint8_t)150, (uint8_t)80, (uint8_t)214};
        p142_storage_SET(&storage, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE3_143(), &PH);
    p143_time_boot_ms_SET((uint32_t)2058309346L, PH.base.pack) ;
    p143_press_abs_SET((float) -2.0775483E38F, PH.base.pack) ;
    p143_press_diff_SET((float) -1.3202304E38F, PH.base.pack) ;
    p143_temperature_SET((int16_t)(int16_t) -32210, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_FOLLOW_TARGET_144(), &PH);
    p144_timestamp_SET((uint64_t)8656693646975544704L, PH.base.pack) ;
    p144_est_capabilities_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
    p144_lat_SET((int32_t)824402240, PH.base.pack) ;
    p144_lon_SET((int32_t)1086997533, PH.base.pack) ;
    p144_alt_SET((float)3.0625045E38F, PH.base.pack) ;
    {
        float  vel [] =  {3.0815856E37F, -1.0806546E38F, -7.407583E37F};
        p144_vel_SET(&vel, 0, &PH.base.pack) ;
    }
    {
        float  acc [] =  {8.392826E37F, 2.357233E38F, 2.7499378E38F};
        p144_acc_SET(&acc, 0, &PH.base.pack) ;
    }
    {
        float  attitude_q [] =  {2.6588558E38F, 5.424589E37F, 1.0646484E37F, 3.2972156E38F};
        p144_attitude_q_SET(&attitude_q, 0, &PH.base.pack) ;
    }
    {
        float  rates [] =  {4.4771685E37F, 3.0845136E38F, 2.0066008E38F};
        p144_rates_SET(&rates, 0, &PH.base.pack) ;
    }
    {
        float  position_cov [] =  {-3.3249033E38F, -1.8791675E38F, -3.1471986E38F};
        p144_position_cov_SET(&position_cov, 0, &PH.base.pack) ;
    }
    p144_custom_state_SET((uint64_t)7825199006837823665L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
    p146_time_usec_SET((uint64_t)1723693782970388418L, PH.base.pack) ;
    p146_x_acc_SET((float) -2.0206309E37F, PH.base.pack) ;
    p146_y_acc_SET((float) -3.046741E38F, PH.base.pack) ;
    p146_z_acc_SET((float)1.1019279E38F, PH.base.pack) ;
    p146_x_vel_SET((float)2.5534574E38F, PH.base.pack) ;
    p146_y_vel_SET((float) -2.3876913E38F, PH.base.pack) ;
    p146_z_vel_SET((float)2.4725742E38F, PH.base.pack) ;
    p146_x_pos_SET((float) -1.887067E38F, PH.base.pack) ;
    p146_y_pos_SET((float) -1.2347893E38F, PH.base.pack) ;
    p146_z_pos_SET((float)3.0034127E38F, PH.base.pack) ;
    p146_airspeed_SET((float) -1.3602898E38F, PH.base.pack) ;
    {
        float  vel_variance [] =  {2.5312567E38F, 2.0177935E38F, 3.3052953E38F};
        p146_vel_variance_SET(&vel_variance, 0, &PH.base.pack) ;
    }
    {
        float  pos_variance [] =  {-2.4689368E38F, 3.5417752E37F, -1.1940836E38F};
        p146_pos_variance_SET(&pos_variance, 0, &PH.base.pack) ;
    }
    {
        float  q [] =  {-2.030424E38F, 1.7485802E38F, -8.7740855E36F, -1.1056221E38F};
        p146_q_SET(&q, 0, &PH.base.pack) ;
    }
    p146_roll_rate_SET((float) -2.3604516E38F, PH.base.pack) ;
    p146_pitch_rate_SET((float) -3.2341882E38F, PH.base.pack) ;
    p146_yaw_rate_SET((float)1.4470825E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_BATTERY_STATUS_147(), &PH);
    p147_id_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
    p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION, PH.base.pack) ;
    p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIFE, PH.base.pack) ;
    p147_temperature_SET((int16_t)(int16_t) -21271, PH.base.pack) ;
    {
        uint16_t  voltages [] =  {(uint16_t)33738, (uint16_t)64150, (uint16_t)65008, (uint16_t)41426, (uint16_t)11009, (uint16_t)59881, (uint16_t)43747, (uint16_t)26523, (uint16_t)22938, (uint16_t)30714};
        p147_voltages_SET(&voltages, 0, &PH.base.pack) ;
    }
    p147_current_battery_SET((int16_t)(int16_t)30841, PH.base.pack) ;
    p147_current_consumed_SET((int32_t) -983301019, PH.base.pack) ;
    p147_energy_consumed_SET((int32_t) -1331553849, PH.base.pack) ;
    p147_battery_remaining_SET((int8_t)(int8_t)38, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_AUTOPILOT_VERSION_148(), &PH);
    p148_capabilities_SET((e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET |
                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_UNION), PH.base.pack) ;
    p148_flight_sw_version_SET((uint32_t)2072225973L, PH.base.pack) ;
    p148_middleware_sw_version_SET((uint32_t)1956511248L, PH.base.pack) ;
    p148_os_sw_version_SET((uint32_t)1587598353L, PH.base.pack) ;
    p148_board_version_SET((uint32_t)1330809899L, PH.base.pack) ;
    {
        uint8_t  flight_custom_version [] =  {(uint8_t)128, (uint8_t)207, (uint8_t)236, (uint8_t)183, (uint8_t)113, (uint8_t)160, (uint8_t)211, (uint8_t)250};
        p148_flight_custom_version_SET(&flight_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  middleware_custom_version [] =  {(uint8_t)86, (uint8_t)141, (uint8_t)77, (uint8_t)112, (uint8_t)241, (uint8_t)120, (uint8_t)207, (uint8_t)84};
        p148_middleware_custom_version_SET(&middleware_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  os_custom_version [] =  {(uint8_t)17, (uint8_t)124, (uint8_t)60, (uint8_t)215, (uint8_t)51, (uint8_t)151, (uint8_t)247, (uint8_t)189};
        p148_os_custom_version_SET(&os_custom_version, 0, &PH.base.pack) ;
    }
    p148_vendor_id_SET((uint16_t)(uint16_t)17469, PH.base.pack) ;
    p148_product_id_SET((uint16_t)(uint16_t)3457, PH.base.pack) ;
    p148_uid_SET((uint64_t)2563853964197841298L, PH.base.pack) ;
    {
        uint8_t  uid2 [] =  {(uint8_t)254, (uint8_t)203, (uint8_t)154, (uint8_t)184, (uint8_t)29, (uint8_t)175, (uint8_t)1, (uint8_t)31, (uint8_t)170, (uint8_t)222, (uint8_t)237, (uint8_t)205, (uint8_t)33, (uint8_t)226, (uint8_t)76, (uint8_t)96, (uint8_t)165, (uint8_t)20};
        p148_uid2_SET(&uid2, 0,  &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LANDING_TARGET_149(), &PH);
    p149_time_usec_SET((uint64_t)5463966567332858898L, PH.base.pack) ;
    p149_target_num_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
    p149_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
    p149_angle_x_SET((float)1.295886E38F, PH.base.pack) ;
    p149_angle_y_SET((float)3.0533819E38F, PH.base.pack) ;
    p149_distance_SET((float)3.7854605E37F, PH.base.pack) ;
    p149_size_x_SET((float) -2.1315598E37F, PH.base.pack) ;
    p149_size_y_SET((float) -9.7068405E36F, PH.base.pack) ;
    p149_x_SET((float)7.623321E37F, &PH) ;
    p149_y_SET((float)5.479995E37F, &PH) ;
    p149_z_SET((float)1.3390418E38F, &PH) ;
    {
        float  q [] =  {2.0601971E38F, 1.5308117E38F, 4.9844655E37F, 2.1406108E38F};
        p149_q_SET(&q, 0,  &PH) ;
    }
    p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL, PH.base.pack) ;
    p149_position_valid_SET((uint8_t)(uint8_t)209, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ESTIMATOR_STATUS_230(), &PH);
    p230_time_usec_SET((uint64_t)359554688752130816L, PH.base.pack) ;
    p230_flags_SET((e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS |
                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_HORIZ |
                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_ABS |
                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE |
                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_GPS_GLITCH |
                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_VERT |
                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_PRED_POS_HORIZ_ABS), PH.base.pack) ;
    p230_vel_ratio_SET((float)8.259242E37F, PH.base.pack) ;
    p230_pos_horiz_ratio_SET((float) -1.3608058E38F, PH.base.pack) ;
    p230_pos_vert_ratio_SET((float)2.8085535E38F, PH.base.pack) ;
    p230_mag_ratio_SET((float)1.6272312E38F, PH.base.pack) ;
    p230_hagl_ratio_SET((float) -4.187386E37F, PH.base.pack) ;
    p230_tas_ratio_SET((float)1.950999E38F, PH.base.pack) ;
    p230_pos_horiz_accuracy_SET((float) -4.975529E37F, PH.base.pack) ;
    p230_pos_vert_accuracy_SET((float) -2.8225385E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_WIND_COV_231(), &PH);
    p231_time_usec_SET((uint64_t)1525205051434164378L, PH.base.pack) ;
    p231_wind_x_SET((float)1.5537542E38F, PH.base.pack) ;
    p231_wind_y_SET((float) -3.039994E38F, PH.base.pack) ;
    p231_wind_z_SET((float)6.176915E36F, PH.base.pack) ;
    p231_var_horiz_SET((float) -1.1245793E38F, PH.base.pack) ;
    p231_var_vert_SET((float)2.228534E38F, PH.base.pack) ;
    p231_wind_alt_SET((float) -2.1678079E38F, PH.base.pack) ;
    p231_horiz_accuracy_SET((float) -2.7275121E38F, PH.base.pack) ;
    p231_vert_accuracy_SET((float) -1.975306E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_INPUT_232(), &PH);
    p232_time_usec_SET((uint64_t)2553593307779991322L, PH.base.pack) ;
    p232_gps_id_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
    p232_ignore_flags_SET((e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT |
                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY), PH.base.pack) ;
    p232_time_week_ms_SET((uint32_t)3830755777L, PH.base.pack) ;
    p232_time_week_SET((uint16_t)(uint16_t)12868, PH.base.pack) ;
    p232_fix_type_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
    p232_lat_SET((int32_t) -871384454, PH.base.pack) ;
    p232_lon_SET((int32_t) -1000370624, PH.base.pack) ;
    p232_alt_SET((float)2.8007981E38F, PH.base.pack) ;
    p232_hdop_SET((float)9.307033E37F, PH.base.pack) ;
    p232_vdop_SET((float) -1.4438727E38F, PH.base.pack) ;
    p232_vn_SET((float)2.2996504E38F, PH.base.pack) ;
    p232_ve_SET((float) -2.8664328E37F, PH.base.pack) ;
    p232_vd_SET((float) -2.0489586E38F, PH.base.pack) ;
    p232_speed_accuracy_SET((float)1.1686495E38F, PH.base.pack) ;
    p232_horiz_accuracy_SET((float) -1.6608525E38F, PH.base.pack) ;
    p232_vert_accuracy_SET((float)3.2313525E38F, PH.base.pack) ;
    p232_satellites_visible_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RTCM_DATA_233(), &PH);
    p233_flags_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
    p233_len_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)67, (uint8_t)233, (uint8_t)21, (uint8_t)186, (uint8_t)98, (uint8_t)233, (uint8_t)55, (uint8_t)169, (uint8_t)78, (uint8_t)194, (uint8_t)163, (uint8_t)91, (uint8_t)109, (uint8_t)217, (uint8_t)159, (uint8_t)60, (uint8_t)116, (uint8_t)1, (uint8_t)13, (uint8_t)210, (uint8_t)147, (uint8_t)117, (uint8_t)164, (uint8_t)142, (uint8_t)54, (uint8_t)47, (uint8_t)24, (uint8_t)23, (uint8_t)116, (uint8_t)193, (uint8_t)64, (uint8_t)89, (uint8_t)120, (uint8_t)224, (uint8_t)237, (uint8_t)162, (uint8_t)13, (uint8_t)107, (uint8_t)155, (uint8_t)231, (uint8_t)33, (uint8_t)166, (uint8_t)61, (uint8_t)117, (uint8_t)1, (uint8_t)20, (uint8_t)113, (uint8_t)134, (uint8_t)249, (uint8_t)228, (uint8_t)113, (uint8_t)100, (uint8_t)35, (uint8_t)188, (uint8_t)188, (uint8_t)100, (uint8_t)81, (uint8_t)90, (uint8_t)143, (uint8_t)38, (uint8_t)58, (uint8_t)45, (uint8_t)248, (uint8_t)77, (uint8_t)104, (uint8_t)155, (uint8_t)248, (uint8_t)77, (uint8_t)70, (uint8_t)159, (uint8_t)244, (uint8_t)178, (uint8_t)6, (uint8_t)78, (uint8_t)38, (uint8_t)210, (uint8_t)181, (uint8_t)230, (uint8_t)127, (uint8_t)22, (uint8_t)49, (uint8_t)211, (uint8_t)110, (uint8_t)154, (uint8_t)198, (uint8_t)76, (uint8_t)223, (uint8_t)111, (uint8_t)255, (uint8_t)8, (uint8_t)122, (uint8_t)103, (uint8_t)131, (uint8_t)114, (uint8_t)149, (uint8_t)177, (uint8_t)188, (uint8_t)27, (uint8_t)56, (uint8_t)141, (uint8_t)139, (uint8_t)120, (uint8_t)165, (uint8_t)218, (uint8_t)139, (uint8_t)110, (uint8_t)147, (uint8_t)60, (uint8_t)241, (uint8_t)199, (uint8_t)147, (uint8_t)121, (uint8_t)131, (uint8_t)77, (uint8_t)56, (uint8_t)27, (uint8_t)221, (uint8_t)138, (uint8_t)128, (uint8_t)173, (uint8_t)142, (uint8_t)252, (uint8_t)113, (uint8_t)48, (uint8_t)100, (uint8_t)13, (uint8_t)153, (uint8_t)202, (uint8_t)205, (uint8_t)120, (uint8_t)131, (uint8_t)219, (uint8_t)36, (uint8_t)26, (uint8_t)166, (uint8_t)122, (uint8_t)179, (uint8_t)240, (uint8_t)252, (uint8_t)78, (uint8_t)211, (uint8_t)156, (uint8_t)103, (uint8_t)249, (uint8_t)84, (uint8_t)180, (uint8_t)193, (uint8_t)161, (uint8_t)232, (uint8_t)61, (uint8_t)11, (uint8_t)108, (uint8_t)103, (uint8_t)56, (uint8_t)129, (uint8_t)163, (uint8_t)179, (uint8_t)114, (uint8_t)233, (uint8_t)181, (uint8_t)64, (uint8_t)58, (uint8_t)249, (uint8_t)139, (uint8_t)221, (uint8_t)159, (uint8_t)7, (uint8_t)13, (uint8_t)45, (uint8_t)178, (uint8_t)204, (uint8_t)95, (uint8_t)49, (uint8_t)29, (uint8_t)59, (uint8_t)52, (uint8_t)70, (uint8_t)237, (uint8_t)149, (uint8_t)223};
        p233_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIGH_LATENCY_234(), &PH);
    p234_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED |
                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED |
                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED), PH.base.pack) ;
    p234_custom_mode_SET((uint32_t)1706685515L, PH.base.pack) ;
    p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, PH.base.pack) ;
    p234_roll_SET((int16_t)(int16_t) -2598, PH.base.pack) ;
    p234_pitch_SET((int16_t)(int16_t) -16333, PH.base.pack) ;
    p234_heading_SET((uint16_t)(uint16_t)26792, PH.base.pack) ;
    p234_throttle_SET((int8_t)(int8_t)28, PH.base.pack) ;
    p234_heading_sp_SET((int16_t)(int16_t)11208, PH.base.pack) ;
    p234_latitude_SET((int32_t) -1107398569, PH.base.pack) ;
    p234_longitude_SET((int32_t) -1777937078, PH.base.pack) ;
    p234_altitude_amsl_SET((int16_t)(int16_t)14863, PH.base.pack) ;
    p234_altitude_sp_SET((int16_t)(int16_t) -23960, PH.base.pack) ;
    p234_airspeed_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
    p234_airspeed_sp_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
    p234_groundspeed_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
    p234_climb_rate_SET((int8_t)(int8_t) -90, PH.base.pack) ;
    p234_gps_nsat_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
    p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, PH.base.pack) ;
    p234_battery_remaining_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
    p234_temperature_SET((int8_t)(int8_t)12, PH.base.pack) ;
    p234_temperature_air_SET((int8_t)(int8_t) -110, PH.base.pack) ;
    p234_failsafe_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
    p234_wp_num_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
    p234_wp_distance_SET((uint16_t)(uint16_t)56776, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VIBRATION_241(), &PH);
    p241_time_usec_SET((uint64_t)7804559335775064311L, PH.base.pack) ;
    p241_vibration_x_SET((float) -2.3420124E37F, PH.base.pack) ;
    p241_vibration_y_SET((float)1.0769282E38F, PH.base.pack) ;
    p241_vibration_z_SET((float)1.9674552E38F, PH.base.pack) ;
    p241_clipping_0_SET((uint32_t)3067673935L, PH.base.pack) ;
    p241_clipping_1_SET((uint32_t)1947474645L, PH.base.pack) ;
    p241_clipping_2_SET((uint32_t)2593942272L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HOME_POSITION_242(), &PH);
    p242_latitude_SET((int32_t) -948431649, PH.base.pack) ;
    p242_longitude_SET((int32_t)28962305, PH.base.pack) ;
    p242_altitude_SET((int32_t) -1678106853, PH.base.pack) ;
    p242_x_SET((float) -2.4089753E38F, PH.base.pack) ;
    p242_y_SET((float)2.9875878E38F, PH.base.pack) ;
    p242_z_SET((float)7.3251636E37F, PH.base.pack) ;
    {
        float  q [] =  {-1.0107038E38F, 2.5532848E38F, 9.380276E36F, 2.7057742E38F};
        p242_q_SET(&q, 0, &PH.base.pack) ;
    }
    p242_approach_x_SET((float)6.631973E37F, PH.base.pack) ;
    p242_approach_y_SET((float)3.0050197E38F, PH.base.pack) ;
    p242_approach_z_SET((float) -2.4209818E38F, PH.base.pack) ;
    p242_time_usec_SET((uint64_t)2166487100628153129L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_HOME_POSITION_243(), &PH);
    p243_target_system_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
    p243_latitude_SET((int32_t)582053597, PH.base.pack) ;
    p243_longitude_SET((int32_t)1518954246, PH.base.pack) ;
    p243_altitude_SET((int32_t) -114822217, PH.base.pack) ;
    p243_x_SET((float) -2.836739E38F, PH.base.pack) ;
    p243_y_SET((float)2.5572617E37F, PH.base.pack) ;
    p243_z_SET((float)2.7004272E38F, PH.base.pack) ;
    {
        float  q [] =  {2.0513682E38F, -3.083031E38F, 2.5844481E38F, -1.794787E38F};
        p243_q_SET(&q, 0, &PH.base.pack) ;
    }
    p243_approach_x_SET((float) -2.4856004E38F, PH.base.pack) ;
    p243_approach_y_SET((float) -3.2389592E38F, PH.base.pack) ;
    p243_approach_z_SET((float)3.3309013E37F, PH.base.pack) ;
    p243_time_usec_SET((uint64_t)7479942355995098298L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}
