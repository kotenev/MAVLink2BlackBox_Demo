
#include "MicroAirVehicle.h"
void failure(Channel * ch, int32_t id, int32_t arg) {}
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
void c_CommunicationChannel_on_SENSOR_OFFSETS_150(Bounds_Inside * ph, Pack * pack)
{
    int16_t  mag_ofs_x = p150_mag_ofs_x_GET(pack);
    int16_t  mag_ofs_y = p150_mag_ofs_y_GET(pack);
    int16_t  mag_ofs_z = p150_mag_ofs_z_GET(pack);
    float  mag_declination = p150_mag_declination_GET(pack);
    int32_t  raw_press = p150_raw_press_GET(pack);
    int32_t  raw_temp = p150_raw_temp_GET(pack);
    float  gyro_cal_x = p150_gyro_cal_x_GET(pack);
    float  gyro_cal_y = p150_gyro_cal_y_GET(pack);
    float  gyro_cal_z = p150_gyro_cal_z_GET(pack);
    float  accel_cal_x = p150_accel_cal_x_GET(pack);
    float  accel_cal_y = p150_accel_cal_y_GET(pack);
    float  accel_cal_z = p150_accel_cal_z_GET(pack);
}
void c_CommunicationChannel_on_SET_MAG_OFFSETS_151(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p151_target_system_GET(pack);
    uint8_t  target_component = p151_target_component_GET(pack);
    int16_t  mag_ofs_x = p151_mag_ofs_x_GET(pack);
    int16_t  mag_ofs_y = p151_mag_ofs_y_GET(pack);
    int16_t  mag_ofs_z = p151_mag_ofs_z_GET(pack);
}
void c_CommunicationChannel_on_MEMINFO_152(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  brkval = p152_brkval_GET(pack);
    uint16_t  freemem = p152_freemem_GET(pack);
    uint32_t  freemem32 = p152_freemem32_TRY(ph);
}
void c_CommunicationChannel_on_AP_ADC_153(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  adc1 = p153_adc1_GET(pack);
    uint16_t  adc2 = p153_adc2_GET(pack);
    uint16_t  adc3 = p153_adc3_GET(pack);
    uint16_t  adc4 = p153_adc4_GET(pack);
    uint16_t  adc5 = p153_adc5_GET(pack);
    uint16_t  adc6 = p153_adc6_GET(pack);
}
void c_CommunicationChannel_on_DIGICAM_CONFIGURE_154(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p154_target_system_GET(pack);
    uint8_t  target_component = p154_target_component_GET(pack);
    uint8_t  mode = p154_mode_GET(pack);
    uint16_t  shutter_speed = p154_shutter_speed_GET(pack);
    uint8_t  aperture = p154_aperture_GET(pack);
    uint8_t  iso = p154_iso_GET(pack);
    uint8_t  exposure_type = p154_exposure_type_GET(pack);
    uint8_t  command_id = p154_command_id_GET(pack);
    uint8_t  engine_cut_off = p154_engine_cut_off_GET(pack);
    uint8_t  extra_param = p154_extra_param_GET(pack);
    float  extra_value = p154_extra_value_GET(pack);
}
void c_CommunicationChannel_on_DIGICAM_CONTROL_155(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p155_target_system_GET(pack);
    uint8_t  target_component = p155_target_component_GET(pack);
    uint8_t  session = p155_session_GET(pack);
    uint8_t  zoom_pos = p155_zoom_pos_GET(pack);
    int8_t  zoom_step = p155_zoom_step_GET(pack);
    uint8_t  focus_lock = p155_focus_lock_GET(pack);
    uint8_t  shot = p155_shot_GET(pack);
    uint8_t  command_id = p155_command_id_GET(pack);
    uint8_t  extra_param = p155_extra_param_GET(pack);
    float  extra_value = p155_extra_value_GET(pack);
}
void c_CommunicationChannel_on_MOUNT_CONFIGURE_156(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p156_target_system_GET(pack);
    uint8_t  target_component = p156_target_component_GET(pack);
    e_MAV_MOUNT_MODE  mount_mode = p156_mount_mode_GET(pack);
    uint8_t  stab_roll = p156_stab_roll_GET(pack);
    uint8_t  stab_pitch = p156_stab_pitch_GET(pack);
    uint8_t  stab_yaw = p156_stab_yaw_GET(pack);
}
void c_CommunicationChannel_on_MOUNT_CONTROL_157(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p157_target_system_GET(pack);
    uint8_t  target_component = p157_target_component_GET(pack);
    int32_t  input_a = p157_input_a_GET(pack);
    int32_t  input_b = p157_input_b_GET(pack);
    int32_t  input_c = p157_input_c_GET(pack);
    uint8_t  save_position = p157_save_position_GET(pack);
}
void c_CommunicationChannel_on_MOUNT_STATUS_158(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p158_target_system_GET(pack);
    uint8_t  target_component = p158_target_component_GET(pack);
    int32_t  pointing_a = p158_pointing_a_GET(pack);
    int32_t  pointing_b = p158_pointing_b_GET(pack);
    int32_t  pointing_c = p158_pointing_c_GET(pack);
}
void c_CommunicationChannel_on_FENCE_POINT_160(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p160_target_system_GET(pack);
    uint8_t  target_component = p160_target_component_GET(pack);
    uint8_t  idx = p160_idx_GET(pack);
    uint8_t  count = p160_count_GET(pack);
    float  lat = p160_lat_GET(pack);
    float  lng = p160_lng_GET(pack);
}
void c_CommunicationChannel_on_FENCE_FETCH_POINT_161(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p161_target_system_GET(pack);
    uint8_t  target_component = p161_target_component_GET(pack);
    uint8_t  idx = p161_idx_GET(pack);
}
void c_CommunicationChannel_on_FENCE_STATUS_162(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  breach_status = p162_breach_status_GET(pack);
    uint16_t  breach_count = p162_breach_count_GET(pack);
    e_FENCE_BREACH  breach_type = p162_breach_type_GET(pack);
    uint32_t  breach_time = p162_breach_time_GET(pack);
}
void c_CommunicationChannel_on_AHRS_163(Bounds_Inside * ph, Pack * pack)
{
    float  omegaIx = p163_omegaIx_GET(pack);
    float  omegaIy = p163_omegaIy_GET(pack);
    float  omegaIz = p163_omegaIz_GET(pack);
    float  accel_weight = p163_accel_weight_GET(pack);
    float  renorm_val = p163_renorm_val_GET(pack);
    float  error_rp = p163_error_rp_GET(pack);
    float  error_yaw = p163_error_yaw_GET(pack);
}
void c_CommunicationChannel_on_SIMSTATE_164(Bounds_Inside * ph, Pack * pack)
{
    float  roll = p164_roll_GET(pack);
    float  pitch = p164_pitch_GET(pack);
    float  yaw = p164_yaw_GET(pack);
    float  xacc = p164_xacc_GET(pack);
    float  yacc = p164_yacc_GET(pack);
    float  zacc = p164_zacc_GET(pack);
    float  xgyro = p164_xgyro_GET(pack);
    float  ygyro = p164_ygyro_GET(pack);
    float  zgyro = p164_zgyro_GET(pack);
    int32_t  lat = p164_lat_GET(pack);
    int32_t  lng = p164_lng_GET(pack);
}
void c_CommunicationChannel_on_HWSTATUS_165(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  Vcc = p165_Vcc_GET(pack);
    uint8_t  I2Cerr = p165_I2Cerr_GET(pack);
}
void c_CommunicationChannel_on_RADIO_166(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  rssi = p166_rssi_GET(pack);
    uint8_t  remrssi = p166_remrssi_GET(pack);
    uint8_t  txbuf = p166_txbuf_GET(pack);
    uint8_t  noise = p166_noise_GET(pack);
    uint8_t  remnoise = p166_remnoise_GET(pack);
    uint16_t  rxerrors = p166_rxerrors_GET(pack);
    uint16_t  fixed_ = p166_fixed__GET(pack);
}
void c_CommunicationChannel_on_LIMITS_STATUS_167(Bounds_Inside * ph, Pack * pack)
{
    e_LIMITS_STATE  limits_state = p167_limits_state_GET(pack);
    uint32_t  last_trigger = p167_last_trigger_GET(pack);
    uint32_t  last_action = p167_last_action_GET(pack);
    uint32_t  last_recovery = p167_last_recovery_GET(pack);
    uint32_t  last_clear = p167_last_clear_GET(pack);
    uint16_t  breach_count = p167_breach_count_GET(pack);
    e_LIMIT_MODULE  mods_enabled = p167_mods_enabled_GET(pack);
    e_LIMIT_MODULE  mods_required = p167_mods_required_GET(pack);
    e_LIMIT_MODULE  mods_triggered = p167_mods_triggered_GET(pack);
}
void c_CommunicationChannel_on_WIND_168(Bounds_Inside * ph, Pack * pack)
{
    float  direction = p168_direction_GET(pack);
    float  speed = p168_speed_GET(pack);
    float  speed_z = p168_speed_z_GET(pack);
}
void c_CommunicationChannel_on_DATA16_169(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  type = p169_type_GET(pack);
    uint8_t  len = p169_len_GET(pack);
    uint8_t*  data_ = p169_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_DATA32_170(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  type = p170_type_GET(pack);
    uint8_t  len = p170_len_GET(pack);
    uint8_t*  data_ = p170_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_DATA64_171(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  type = p171_type_GET(pack);
    uint8_t  len = p171_len_GET(pack);
    uint8_t*  data_ = p171_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_DATA96_172(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  type = p172_type_GET(pack);
    uint8_t  len = p172_len_GET(pack);
    uint8_t*  data_ = p172_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_RANGEFINDER_173(Bounds_Inside * ph, Pack * pack)
{
    float  distance = p173_distance_GET(pack);
    float  voltage = p173_voltage_GET(pack);
}
void c_CommunicationChannel_on_AIRSPEED_AUTOCAL_174(Bounds_Inside * ph, Pack * pack)
{
    float  vx = p174_vx_GET(pack);
    float  vy = p174_vy_GET(pack);
    float  vz = p174_vz_GET(pack);
    float  diff_pressure = p174_diff_pressure_GET(pack);
    float  EAS2TAS = p174_EAS2TAS_GET(pack);
    float  ratio = p174_ratio_GET(pack);
    float  state_x = p174_state_x_GET(pack);
    float  state_y = p174_state_y_GET(pack);
    float  state_z = p174_state_z_GET(pack);
    float  Pax = p174_Pax_GET(pack);
    float  Pby = p174_Pby_GET(pack);
    float  Pcz = p174_Pcz_GET(pack);
}
void c_CommunicationChannel_on_RALLY_POINT_175(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p175_target_system_GET(pack);
    uint8_t  target_component = p175_target_component_GET(pack);
    uint8_t  idx = p175_idx_GET(pack);
    uint8_t  count = p175_count_GET(pack);
    int32_t  lat = p175_lat_GET(pack);
    int32_t  lng = p175_lng_GET(pack);
    int16_t  alt = p175_alt_GET(pack);
    int16_t  break_alt = p175_break_alt_GET(pack);
    uint16_t  land_dir = p175_land_dir_GET(pack);
    e_RALLY_FLAGS  flags = p175_flags_GET(pack);
}
void c_CommunicationChannel_on_RALLY_FETCH_POINT_176(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p176_target_system_GET(pack);
    uint8_t  target_component = p176_target_component_GET(pack);
    uint8_t  idx = p176_idx_GET(pack);
}
void c_CommunicationChannel_on_COMPASSMOT_STATUS_177(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  throttle = p177_throttle_GET(pack);
    float  current = p177_current_GET(pack);
    uint16_t  interference = p177_interference_GET(pack);
    float  CompensationX = p177_CompensationX_GET(pack);
    float  CompensationY = p177_CompensationY_GET(pack);
    float  CompensationZ = p177_CompensationZ_GET(pack);
}
void c_CommunicationChannel_on_AHRS2_178(Bounds_Inside * ph, Pack * pack)
{
    float  roll = p178_roll_GET(pack);
    float  pitch = p178_pitch_GET(pack);
    float  yaw = p178_yaw_GET(pack);
    float  altitude = p178_altitude_GET(pack);
    int32_t  lat = p178_lat_GET(pack);
    int32_t  lng = p178_lng_GET(pack);
}
void c_CommunicationChannel_on_CAMERA_STATUS_179(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p179_time_usec_GET(pack);
    uint8_t  target_system = p179_target_system_GET(pack);
    uint8_t  cam_idx = p179_cam_idx_GET(pack);
    uint16_t  img_idx = p179_img_idx_GET(pack);
    e_CAMERA_STATUS_TYPES  event_id = p179_event_id_GET(pack);
    float  p1 = p179_p1_GET(pack);
    float  p2 = p179_p2_GET(pack);
    float  p3 = p179_p3_GET(pack);
    float  p4 = p179_p4_GET(pack);
}
void c_CommunicationChannel_on_CAMERA_FEEDBACK_180(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p180_time_usec_GET(pack);
    uint8_t  target_system = p180_target_system_GET(pack);
    uint8_t  cam_idx = p180_cam_idx_GET(pack);
    uint16_t  img_idx = p180_img_idx_GET(pack);
    int32_t  lat = p180_lat_GET(pack);
    int32_t  lng = p180_lng_GET(pack);
    float  alt_msl = p180_alt_msl_GET(pack);
    float  alt_rel = p180_alt_rel_GET(pack);
    float  roll = p180_roll_GET(pack);
    float  pitch = p180_pitch_GET(pack);
    float  yaw = p180_yaw_GET(pack);
    float  foc_len = p180_foc_len_GET(pack);
    e_CAMERA_FEEDBACK_FLAGS  flags = p180_flags_GET(pack);
}
void c_CommunicationChannel_on_BATTERY2_181(Bounds_Inside * ph, Pack * pack)
{
    uint16_t  voltage = p181_voltage_GET(pack);
    int16_t  current_battery = p181_current_battery_GET(pack);
}
void c_CommunicationChannel_on_AHRS3_182(Bounds_Inside * ph, Pack * pack)
{
    float  roll = p182_roll_GET(pack);
    float  pitch = p182_pitch_GET(pack);
    float  yaw = p182_yaw_GET(pack);
    float  altitude = p182_altitude_GET(pack);
    int32_t  lat = p182_lat_GET(pack);
    int32_t  lng = p182_lng_GET(pack);
    float  v1 = p182_v1_GET(pack);
    float  v2 = p182_v2_GET(pack);
    float  v3 = p182_v3_GET(pack);
    float  v4 = p182_v4_GET(pack);
}
void c_CommunicationChannel_on_AUTOPILOT_VERSION_REQUEST_183(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p183_target_system_GET(pack);
    uint8_t  target_component = p183_target_component_GET(pack);
}
void c_CommunicationChannel_on_REMOTE_LOG_DATA_BLOCK_184(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p184_target_system_GET(pack);
    uint8_t  target_component = p184_target_component_GET(pack);
    e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS  seqno = p184_seqno_GET(pack);
    uint8_t*  data_ = p184_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_REMOTE_LOG_BLOCK_STATUS_185(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p185_target_system_GET(pack);
    uint8_t  target_component = p185_target_component_GET(pack);
    uint32_t  seqno = p185_seqno_GET(pack);
    e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES  status = p185_status_GET(pack);
}
void c_CommunicationChannel_on_LED_CONTROL_186(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p186_target_system_GET(pack);
    uint8_t  target_component = p186_target_component_GET(pack);
    uint8_t  instance = p186_instance_GET(pack);
    uint8_t  pattern = p186_pattern_GET(pack);
    uint8_t  custom_len = p186_custom_len_GET(pack);
    uint8_t*  custom_bytes = p186_custom_bytes_GET_(pack);
//process data in custom_bytes
    free(custom_bytes);//never forget to dispose
}
void c_CommunicationChannel_on_MAG_CAL_PROGRESS_191(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  compass_id = p191_compass_id_GET(pack);
    uint8_t  cal_mask = p191_cal_mask_GET(pack);
    e_MAG_CAL_STATUS  cal_status = p191_cal_status_GET(pack);
    uint8_t  attempt = p191_attempt_GET(pack);
    uint8_t  completion_pct = p191_completion_pct_GET(pack);
    uint8_t*  completion_mask = p191_completion_mask_GET_(pack);
//process data in completion_mask
    free(completion_mask);//never forget to dispose
    float  direction_x = p191_direction_x_GET(pack);
    float  direction_y = p191_direction_y_GET(pack);
    float  direction_z = p191_direction_z_GET(pack);
}
void c_CommunicationChannel_on_MAG_CAL_REPORT_192(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  compass_id = p192_compass_id_GET(pack);
    uint8_t  cal_mask = p192_cal_mask_GET(pack);
    e_MAG_CAL_STATUS  cal_status = p192_cal_status_GET(pack);
    uint8_t  autosaved = p192_autosaved_GET(pack);
    float  fitness = p192_fitness_GET(pack);
    float  ofs_x = p192_ofs_x_GET(pack);
    float  ofs_y = p192_ofs_y_GET(pack);
    float  ofs_z = p192_ofs_z_GET(pack);
    float  diag_x = p192_diag_x_GET(pack);
    float  diag_y = p192_diag_y_GET(pack);
    float  diag_z = p192_diag_z_GET(pack);
    float  offdiag_x = p192_offdiag_x_GET(pack);
    float  offdiag_y = p192_offdiag_y_GET(pack);
    float  offdiag_z = p192_offdiag_z_GET(pack);
}
void c_CommunicationChannel_on_EKF_STATUS_REPORT_193(Bounds_Inside * ph, Pack * pack)
{
    e_EKF_STATUS_FLAGS  flags = p193_flags_GET(pack);
    float  velocity_variance = p193_velocity_variance_GET(pack);
    float  pos_horiz_variance = p193_pos_horiz_variance_GET(pack);
    float  pos_vert_variance = p193_pos_vert_variance_GET(pack);
    float  compass_variance = p193_compass_variance_GET(pack);
    float  terrain_alt_variance = p193_terrain_alt_variance_GET(pack);
}
void c_CommunicationChannel_on_PID_TUNING_194(Bounds_Inside * ph, Pack * pack)
{
    e_PID_TUNING_AXIS  axis = p194_axis_GET(pack);
    float  desired = p194_desired_GET(pack);
    float  achieved = p194_achieved_GET(pack);
    float  FF = p194_FF_GET(pack);
    float  P = p194_P_GET(pack);
    float  I = p194_I_GET(pack);
    float  D = p194_D_GET(pack);
}
void c_CommunicationChannel_on_GIMBAL_REPORT_200(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p200_target_system_GET(pack);
    uint8_t  target_component = p200_target_component_GET(pack);
    float  delta_time = p200_delta_time_GET(pack);
    float  delta_angle_x = p200_delta_angle_x_GET(pack);
    float  delta_angle_y = p200_delta_angle_y_GET(pack);
    float  delta_angle_z = p200_delta_angle_z_GET(pack);
    float  delta_velocity_x = p200_delta_velocity_x_GET(pack);
    float  delta_velocity_y = p200_delta_velocity_y_GET(pack);
    float  delta_velocity_z = p200_delta_velocity_z_GET(pack);
    float  joint_roll = p200_joint_roll_GET(pack);
    float  joint_el = p200_joint_el_GET(pack);
    float  joint_az = p200_joint_az_GET(pack);
}
void c_CommunicationChannel_on_GIMBAL_CONTROL_201(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p201_target_system_GET(pack);
    uint8_t  target_component = p201_target_component_GET(pack);
    float  demanded_rate_x = p201_demanded_rate_x_GET(pack);
    float  demanded_rate_y = p201_demanded_rate_y_GET(pack);
    float  demanded_rate_z = p201_demanded_rate_z_GET(pack);
}
void c_CommunicationChannel_on_GIMBAL_TORQUE_CMD_REPORT_214(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p214_target_system_GET(pack);
    uint8_t  target_component = p214_target_component_GET(pack);
    int16_t  rl_torque_cmd = p214_rl_torque_cmd_GET(pack);
    int16_t  el_torque_cmd = p214_el_torque_cmd_GET(pack);
    int16_t  az_torque_cmd = p214_az_torque_cmd_GET(pack);
}
void c_CommunicationChannel_on_GOPRO_HEARTBEAT_215(Bounds_Inside * ph, Pack * pack)
{
    e_GOPRO_HEARTBEAT_STATUS  status = p215_status_GET(pack);
    e_GOPRO_CAPTURE_MODE  capture_mode = p215_capture_mode_GET(pack);
    e_GOPRO_HEARTBEAT_FLAGS  flags = p215_flags_GET(pack);
}
void c_CommunicationChannel_on_GOPRO_GET_REQUEST_216(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p216_target_system_GET(pack);
    uint8_t  target_component = p216_target_component_GET(pack);
    e_GOPRO_COMMAND  cmd_id = p216_cmd_id_GET(pack);
}
void c_CommunicationChannel_on_GOPRO_GET_RESPONSE_217(Bounds_Inside * ph, Pack * pack)
{
    e_GOPRO_COMMAND  cmd_id = p217_cmd_id_GET(pack);
    e_GOPRO_REQUEST_STATUS  status = p217_status_GET(pack);
    uint8_t*  value = p217_value_GET_(pack);
//process data in value
    free(value);//never forget to dispose
}
void c_CommunicationChannel_on_GOPRO_SET_REQUEST_218(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p218_target_system_GET(pack);
    uint8_t  target_component = p218_target_component_GET(pack);
    e_GOPRO_COMMAND  cmd_id = p218_cmd_id_GET(pack);
    uint8_t*  value = p218_value_GET_(pack);
//process data in value
    free(value);//never forget to dispose
}
void c_CommunicationChannel_on_GOPRO_SET_RESPONSE_219(Bounds_Inside * ph, Pack * pack)
{
    e_GOPRO_COMMAND  cmd_id = p219_cmd_id_GET(pack);
    e_GOPRO_REQUEST_STATUS  status = p219_status_GET(pack);
}
void c_CommunicationChannel_on_RPM_226(Bounds_Inside * ph, Pack * pack)
{
    float  rpm1 = p226_rpm1_GET(pack);
    float  rpm2 = p226_rpm2_GET(pack);
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
void c_CommunicationChannel_on_UAVIONIX_ADSB_OUT_CFG_10001(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  ICAO = p10001_ICAO_GET(pack);
    char16_t *  callsign = p10001_callsign_TRY_(ph);
    e_ADSB_EMITTER_TYPE  emitterType = p10001_emitterType_GET(pack);
    e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE  aircraftSize = p10001_aircraftSize_GET(pack);
    e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT  gpsOffsetLat = p10001_gpsOffsetLat_GET(pack);
    e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON  gpsOffsetLon = p10001_gpsOffsetLon_GET(pack);
    uint16_t  stallSpeed = p10001_stallSpeed_GET(pack);
    e_UAVIONIX_ADSB_OUT_RF_SELECT  rfSelect = p10001_rfSelect_GET(pack);
}
void c_CommunicationChannel_on_UAVIONIX_ADSB_OUT_DYNAMIC_10002(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  utcTime = p10002_utcTime_GET(pack);
    int32_t  gpsLat = p10002_gpsLat_GET(pack);
    int32_t  gpsLon = p10002_gpsLon_GET(pack);
    int32_t  gpsAlt = p10002_gpsAlt_GET(pack);
    e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX  gpsFix = p10002_gpsFix_GET(pack);
    uint8_t  numSats = p10002_numSats_GET(pack);
    int32_t  baroAltMSL = p10002_baroAltMSL_GET(pack);
    uint32_t  accuracyHor = p10002_accuracyHor_GET(pack);
    uint16_t  accuracyVert = p10002_accuracyVert_GET(pack);
    uint16_t  accuracyVel = p10002_accuracyVel_GET(pack);
    int16_t  velVert = p10002_velVert_GET(pack);
    int16_t  velNS = p10002_velNS_GET(pack);
    int16_t  VelEW = p10002_VelEW_GET(pack);
    e_UAVIONIX_ADSB_EMERGENCY_STATUS  emergencyStatus = p10002_emergencyStatus_GET(pack);
    e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE  state = p10002_state_GET(pack);
    uint16_t  squawk = p10002_squawk_GET(pack);
}
void c_CommunicationChannel_on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_10003(Bounds_Inside * ph, Pack * pack)
{
    e_UAVIONIX_ADSB_RF_HEALTH  rfHealth = p10003_rfHealth_GET(pack);
}
void c_CommunicationChannel_on_DEVICE_OP_READ_11000(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p11000_target_system_GET(pack);
    uint8_t  target_component = p11000_target_component_GET(pack);
    uint32_t  request_id = p11000_request_id_GET(pack);
    e_DEVICE_OP_BUSTYPE  bustype = p11000_bustype_GET(pack);
    uint8_t  bus = p11000_bus_GET(pack);
    uint8_t  address = p11000_address_GET(pack);
    char16_t *  busname = p11000_busname_TRY_(ph);
    uint8_t  regstart = p11000_regstart_GET(pack);
    uint8_t  count = p11000_count_GET(pack);
}
void c_CommunicationChannel_on_DEVICE_OP_READ_REPLY_11001(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  request_id = p11001_request_id_GET(pack);
    uint8_t  result = p11001_result_GET(pack);
    uint8_t  regstart = p11001_regstart_GET(pack);
    uint8_t  count = p11001_count_GET(pack);
    uint8_t*  data_ = p11001_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_DEVICE_OP_WRITE_11002(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  target_system = p11002_target_system_GET(pack);
    uint8_t  target_component = p11002_target_component_GET(pack);
    uint32_t  request_id = p11002_request_id_GET(pack);
    e_DEVICE_OP_BUSTYPE  bustype = p11002_bustype_GET(pack);
    uint8_t  bus = p11002_bus_GET(pack);
    uint8_t  address = p11002_address_GET(pack);
    char16_t *  busname = p11002_busname_TRY_(ph);
    uint8_t  regstart = p11002_regstart_GET(pack);
    uint8_t  count = p11002_count_GET(pack);
    uint8_t*  data_ = p11002_data__GET_(pack);
//process data in data_
    free(data_);//never forget to dispose
}
void c_CommunicationChannel_on_DEVICE_OP_WRITE_REPLY_11003(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  request_id = p11003_request_id_GET(pack);
    uint8_t  result = p11003_result_GET(pack);
}
void c_CommunicationChannel_on_ADAP_TUNING_11010(Bounds_Inside * ph, Pack * pack)
{
    e_PID_TUNING_AXIS  axis = p11010_axis_GET(pack);
    float  desired = p11010_desired_GET(pack);
    float  achieved = p11010_achieved_GET(pack);
    float  error = p11010_error_GET(pack);
    float  theta = p11010_theta_GET(pack);
    float  omega = p11010_omega_GET(pack);
    float  sigma = p11010_sigma_GET(pack);
    float  theta_dot = p11010_theta_dot_GET(pack);
    float  omega_dot = p11010_omega_dot_GET(pack);
    float  sigma_dot = p11010_sigma_dot_GET(pack);
    float  f = p11010_f_GET(pack);
    float  f_dot = p11010_f_dot_GET(pack);
    float  u = p11010_u_GET(pack);
}
void c_CommunicationChannel_on_VISION_POSITION_DELTA_11011(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  time_usec = p11011_time_usec_GET(pack);
    uint64_t  time_delta_usec = p11011_time_delta_usec_GET(pack);
    float*  angle_delta = p11011_angle_delta_GET_(pack);
//process data in angle_delta
    free(angle_delta);//never forget to dispose
    float*  position_delta = p11011_position_delta_GET_(pack);
//process data in position_delta
    free(position_delta);//never forget to dispose
    float  confidence = p11011_confidence_GET(pack);
}

void main()
{
    static Bounds_Inside PH;
    setPack(c_CommunicationChannel_new_HEARTBEAT_0(), &PH);
    p0_type_SET(e_MAV_TYPE_MAV_TYPE_COAXIAL, PH.base.pack) ;
    p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY, PH.base.pack) ;
    p0_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED |
                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED |
                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED), PH.base.pack) ;
    p0_custom_mode_SET((uint32_t)1794138626L, PH.base.pack) ;
    p0_system_status_SET(e_MAV_STATE_MAV_STATE_ACTIVE, PH.base.pack) ;
    p0_mavlink_version_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
    p1_onboard_control_sensors_present_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE), PH.base.pack) ;
    p1_onboard_control_sensors_enabled_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW), PH.base.pack) ;
    p1_onboard_control_sensors_health_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
    p1_load_SET((uint16_t)(uint16_t)59031, PH.base.pack) ;
    p1_voltage_battery_SET((uint16_t)(uint16_t)22861, PH.base.pack) ;
    p1_current_battery_SET((int16_t)(int16_t) -29496, PH.base.pack) ;
    p1_battery_remaining_SET((int8_t)(int8_t)16, PH.base.pack) ;
    p1_drop_rate_comm_SET((uint16_t)(uint16_t)38654, PH.base.pack) ;
    p1_errors_comm_SET((uint16_t)(uint16_t)39623, PH.base.pack) ;
    p1_errors_count1_SET((uint16_t)(uint16_t)62273, PH.base.pack) ;
    p1_errors_count2_SET((uint16_t)(uint16_t)42126, PH.base.pack) ;
    p1_errors_count3_SET((uint16_t)(uint16_t)63110, PH.base.pack) ;
    p1_errors_count4_SET((uint16_t)(uint16_t)25058, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
    p2_time_unix_usec_SET((uint64_t)8847076042522342758L, PH.base.pack) ;
    p2_time_boot_ms_SET((uint32_t)288488498L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
    p3_time_boot_ms_SET((uint32_t)776071453L, PH.base.pack) ;
    p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
    p3_type_mask_SET((uint16_t)(uint16_t)44212, PH.base.pack) ;
    p3_x_SET((float)3.4342538E37F, PH.base.pack) ;
    p3_y_SET((float)2.7719852E38F, PH.base.pack) ;
    p3_z_SET((float)9.773622E37F, PH.base.pack) ;
    p3_vx_SET((float) -1.9108983E38F, PH.base.pack) ;
    p3_vy_SET((float)5.815187E37F, PH.base.pack) ;
    p3_vz_SET((float)3.0484906E38F, PH.base.pack) ;
    p3_afx_SET((float) -1.9984613E38F, PH.base.pack) ;
    p3_afy_SET((float) -2.7095458E38F, PH.base.pack) ;
    p3_afz_SET((float)6.863453E37F, PH.base.pack) ;
    p3_yaw_SET((float)2.6464174E38F, PH.base.pack) ;
    p3_yaw_rate_SET((float) -8.817172E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PING_4(), &PH);
    p4_time_usec_SET((uint64_t)3724659471987162165L, PH.base.pack) ;
    p4_seq_SET((uint32_t)2051370164L, PH.base.pack) ;
    p4_target_system_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
    p4_target_component_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
    p5_target_system_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
    p5_control_request_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
    p5_version_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
    {
        char16_t   passkey = "adaLkjcnmuogj";
        p5_passkey_SET(&passkey, 0,  sizeof(passkey), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
    p6_gcs_system_id_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
    p6_control_request_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
    p6_ack_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
    {
        char16_t   key = "mre";
        p7_key_SET(&key, 0,  sizeof(key), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
    p11_target_system_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
    p11_base_mode_SET(e_MAV_MODE_MAV_MODE_PREFLIGHT, PH.base.pack) ;
    p11_custom_mode_SET((uint32_t)2473253527L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
    p20_target_system_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
    p20_target_component_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
    {
        char16_t   param_id = "obuquvuQxkbx";
        p20_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p20_param_index_SET((int16_t)(int16_t) -5331, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
    p21_target_system_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
    p21_target_component_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
    {
        char16_t   param_id = "f";
        p22_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p22_param_value_SET((float) -2.729466E38F, PH.base.pack) ;
    p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16, PH.base.pack) ;
    p22_param_count_SET((uint16_t)(uint16_t)48126, PH.base.pack) ;
    p22_param_index_SET((uint16_t)(uint16_t)5591, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
    p23_target_system_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
    p23_target_component_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
    {
        char16_t   param_id = "blurmQuey";
        p23_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p23_param_value_SET((float)2.8223545E38F, PH.base.pack) ;
    p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
    p24_time_usec_SET((uint64_t)5245106818006308505L, PH.base.pack) ;
    p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, PH.base.pack) ;
    p24_lat_SET((int32_t) -315325754, PH.base.pack) ;
    p24_lon_SET((int32_t)2041912937, PH.base.pack) ;
    p24_alt_SET((int32_t) -1723378949, PH.base.pack) ;
    p24_eph_SET((uint16_t)(uint16_t)40021, PH.base.pack) ;
    p24_epv_SET((uint16_t)(uint16_t)53400, PH.base.pack) ;
    p24_vel_SET((uint16_t)(uint16_t)61590, PH.base.pack) ;
    p24_cog_SET((uint16_t)(uint16_t)15228, PH.base.pack) ;
    p24_satellites_visible_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
    p24_alt_ellipsoid_SET((int32_t) -82596147, &PH) ;
    p24_h_acc_SET((uint32_t)3547689919L, &PH) ;
    p24_v_acc_SET((uint32_t)3641903022L, &PH) ;
    p24_vel_acc_SET((uint32_t)273702697L, &PH) ;
    p24_hdg_acc_SET((uint32_t)2500702066L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
    p25_satellites_visible_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
    {
        uint8_t  satellite_prn [] =  {(uint8_t)4, (uint8_t)199, (uint8_t)104, (uint8_t)34, (uint8_t)122, (uint8_t)219, (uint8_t)129, (uint8_t)223, (uint8_t)46, (uint8_t)168, (uint8_t)184, (uint8_t)188, (uint8_t)114, (uint8_t)131, (uint8_t)183, (uint8_t)2, (uint8_t)103, (uint8_t)243, (uint8_t)38, (uint8_t)85};
        p25_satellite_prn_SET(&satellite_prn, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_used [] =  {(uint8_t)217, (uint8_t)104, (uint8_t)134, (uint8_t)167, (uint8_t)147, (uint8_t)111, (uint8_t)68, (uint8_t)71, (uint8_t)43, (uint8_t)61, (uint8_t)170, (uint8_t)35, (uint8_t)195, (uint8_t)87, (uint8_t)139, (uint8_t)88, (uint8_t)119, (uint8_t)58, (uint8_t)154, (uint8_t)69};
        p25_satellite_used_SET(&satellite_used, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_elevation [] =  {(uint8_t)12, (uint8_t)2, (uint8_t)215, (uint8_t)37, (uint8_t)197, (uint8_t)145, (uint8_t)115, (uint8_t)207, (uint8_t)221, (uint8_t)113, (uint8_t)145, (uint8_t)118, (uint8_t)50, (uint8_t)93, (uint8_t)81, (uint8_t)159, (uint8_t)173, (uint8_t)36, (uint8_t)79, (uint8_t)76};
        p25_satellite_elevation_SET(&satellite_elevation, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_azimuth [] =  {(uint8_t)221, (uint8_t)211, (uint8_t)197, (uint8_t)220, (uint8_t)154, (uint8_t)54, (uint8_t)181, (uint8_t)179, (uint8_t)100, (uint8_t)217, (uint8_t)231, (uint8_t)218, (uint8_t)18, (uint8_t)116, (uint8_t)233, (uint8_t)30, (uint8_t)145, (uint8_t)96, (uint8_t)231, (uint8_t)231};
        p25_satellite_azimuth_SET(&satellite_azimuth, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_snr [] =  {(uint8_t)255, (uint8_t)9, (uint8_t)114, (uint8_t)25, (uint8_t)118, (uint8_t)87, (uint8_t)21, (uint8_t)1, (uint8_t)248, (uint8_t)3, (uint8_t)199, (uint8_t)0, (uint8_t)76, (uint8_t)36, (uint8_t)198, (uint8_t)107, (uint8_t)236, (uint8_t)142, (uint8_t)121, (uint8_t)142};
        p25_satellite_snr_SET(&satellite_snr, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
    p26_time_boot_ms_SET((uint32_t)3887127073L, PH.base.pack) ;
    p26_xacc_SET((int16_t)(int16_t) -24343, PH.base.pack) ;
    p26_yacc_SET((int16_t)(int16_t)28160, PH.base.pack) ;
    p26_zacc_SET((int16_t)(int16_t)10151, PH.base.pack) ;
    p26_xgyro_SET((int16_t)(int16_t) -15072, PH.base.pack) ;
    p26_ygyro_SET((int16_t)(int16_t) -23598, PH.base.pack) ;
    p26_zgyro_SET((int16_t)(int16_t)27673, PH.base.pack) ;
    p26_xmag_SET((int16_t)(int16_t)344, PH.base.pack) ;
    p26_ymag_SET((int16_t)(int16_t)20816, PH.base.pack) ;
    p26_zmag_SET((int16_t)(int16_t) -30000, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
    p27_time_usec_SET((uint64_t)5598807067171673541L, PH.base.pack) ;
    p27_xacc_SET((int16_t)(int16_t) -2748, PH.base.pack) ;
    p27_yacc_SET((int16_t)(int16_t) -15957, PH.base.pack) ;
    p27_zacc_SET((int16_t)(int16_t) -20163, PH.base.pack) ;
    p27_xgyro_SET((int16_t)(int16_t) -25760, PH.base.pack) ;
    p27_ygyro_SET((int16_t)(int16_t)5592, PH.base.pack) ;
    p27_zgyro_SET((int16_t)(int16_t)19598, PH.base.pack) ;
    p27_xmag_SET((int16_t)(int16_t) -4659, PH.base.pack) ;
    p27_ymag_SET((int16_t)(int16_t)21840, PH.base.pack) ;
    p27_zmag_SET((int16_t)(int16_t) -11167, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
    p28_time_usec_SET((uint64_t)6515023430153036158L, PH.base.pack) ;
    p28_press_abs_SET((int16_t)(int16_t)22347, PH.base.pack) ;
    p28_press_diff1_SET((int16_t)(int16_t) -29475, PH.base.pack) ;
    p28_press_diff2_SET((int16_t)(int16_t)31418, PH.base.pack) ;
    p28_temperature_SET((int16_t)(int16_t) -18970, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
    p29_time_boot_ms_SET((uint32_t)1868346673L, PH.base.pack) ;
    p29_press_abs_SET((float) -3.8275034E37F, PH.base.pack) ;
    p29_press_diff_SET((float) -2.7150806E38F, PH.base.pack) ;
    p29_temperature_SET((int16_t)(int16_t)11497, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
    p30_time_boot_ms_SET((uint32_t)3861966626L, PH.base.pack) ;
    p30_roll_SET((float)2.7514732E38F, PH.base.pack) ;
    p30_pitch_SET((float)3.0036697E38F, PH.base.pack) ;
    p30_yaw_SET((float) -2.756812E36F, PH.base.pack) ;
    p30_rollspeed_SET((float) -2.5145219E38F, PH.base.pack) ;
    p30_pitchspeed_SET((float)8.107138E37F, PH.base.pack) ;
    p30_yawspeed_SET((float) -2.963921E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
    p31_time_boot_ms_SET((uint32_t)1840231337L, PH.base.pack) ;
    p31_q1_SET((float)4.4946377E37F, PH.base.pack) ;
    p31_q2_SET((float)1.0324499E38F, PH.base.pack) ;
    p31_q3_SET((float)1.0594882E38F, PH.base.pack) ;
    p31_q4_SET((float)8.4607453E37F, PH.base.pack) ;
    p31_rollspeed_SET((float) -7.1825783E37F, PH.base.pack) ;
    p31_pitchspeed_SET((float) -1.108721E38F, PH.base.pack) ;
    p31_yawspeed_SET((float)1.9618802E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
    p32_time_boot_ms_SET((uint32_t)3322525078L, PH.base.pack) ;
    p32_x_SET((float)1.4449457E38F, PH.base.pack) ;
    p32_y_SET((float)3.0913798E38F, PH.base.pack) ;
    p32_z_SET((float) -2.762298E38F, PH.base.pack) ;
    p32_vx_SET((float) -4.9368656E36F, PH.base.pack) ;
    p32_vy_SET((float) -1.6416049E37F, PH.base.pack) ;
    p32_vz_SET((float)2.8009886E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
    p33_time_boot_ms_SET((uint32_t)3574551000L, PH.base.pack) ;
    p33_lat_SET((int32_t) -1045513756, PH.base.pack) ;
    p33_lon_SET((int32_t) -158795525, PH.base.pack) ;
    p33_alt_SET((int32_t) -1026553670, PH.base.pack) ;
    p33_relative_alt_SET((int32_t) -1895248501, PH.base.pack) ;
    p33_vx_SET((int16_t)(int16_t) -31271, PH.base.pack) ;
    p33_vy_SET((int16_t)(int16_t) -13873, PH.base.pack) ;
    p33_vz_SET((int16_t)(int16_t) -26891, PH.base.pack) ;
    p33_hdg_SET((uint16_t)(uint16_t)48411, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
    p34_time_boot_ms_SET((uint32_t)3520109737L, PH.base.pack) ;
    p34_port_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
    p34_chan1_scaled_SET((int16_t)(int16_t)4591, PH.base.pack) ;
    p34_chan2_scaled_SET((int16_t)(int16_t) -25649, PH.base.pack) ;
    p34_chan3_scaled_SET((int16_t)(int16_t)20884, PH.base.pack) ;
    p34_chan4_scaled_SET((int16_t)(int16_t) -4533, PH.base.pack) ;
    p34_chan5_scaled_SET((int16_t)(int16_t)23071, PH.base.pack) ;
    p34_chan6_scaled_SET((int16_t)(int16_t) -5713, PH.base.pack) ;
    p34_chan7_scaled_SET((int16_t)(int16_t)31977, PH.base.pack) ;
    p34_chan8_scaled_SET((int16_t)(int16_t)7740, PH.base.pack) ;
    p34_rssi_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
    p35_time_boot_ms_SET((uint32_t)2852656874L, PH.base.pack) ;
    p35_port_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
    p35_chan1_raw_SET((uint16_t)(uint16_t)6735, PH.base.pack) ;
    p35_chan2_raw_SET((uint16_t)(uint16_t)21762, PH.base.pack) ;
    p35_chan3_raw_SET((uint16_t)(uint16_t)14801, PH.base.pack) ;
    p35_chan4_raw_SET((uint16_t)(uint16_t)51097, PH.base.pack) ;
    p35_chan5_raw_SET((uint16_t)(uint16_t)22694, PH.base.pack) ;
    p35_chan6_raw_SET((uint16_t)(uint16_t)14385, PH.base.pack) ;
    p35_chan7_raw_SET((uint16_t)(uint16_t)5842, PH.base.pack) ;
    p35_chan8_raw_SET((uint16_t)(uint16_t)6825, PH.base.pack) ;
    p35_rssi_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
    p36_time_usec_SET((uint32_t)1376159625L, PH.base.pack) ;
    p36_port_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
    p36_servo1_raw_SET((uint16_t)(uint16_t)17057, PH.base.pack) ;
    p36_servo2_raw_SET((uint16_t)(uint16_t)64446, PH.base.pack) ;
    p36_servo3_raw_SET((uint16_t)(uint16_t)19870, PH.base.pack) ;
    p36_servo4_raw_SET((uint16_t)(uint16_t)13690, PH.base.pack) ;
    p36_servo5_raw_SET((uint16_t)(uint16_t)13597, PH.base.pack) ;
    p36_servo6_raw_SET((uint16_t)(uint16_t)24250, PH.base.pack) ;
    p36_servo7_raw_SET((uint16_t)(uint16_t)9663, PH.base.pack) ;
    p36_servo8_raw_SET((uint16_t)(uint16_t)60244, PH.base.pack) ;
    p36_servo9_raw_SET((uint16_t)(uint16_t)54533, &PH) ;
    p36_servo10_raw_SET((uint16_t)(uint16_t)53647, &PH) ;
    p36_servo11_raw_SET((uint16_t)(uint16_t)35780, &PH) ;
    p36_servo12_raw_SET((uint16_t)(uint16_t)18159, &PH) ;
    p36_servo13_raw_SET((uint16_t)(uint16_t)26380, &PH) ;
    p36_servo14_raw_SET((uint16_t)(uint16_t)27036, &PH) ;
    p36_servo15_raw_SET((uint16_t)(uint16_t)57884, &PH) ;
    p36_servo16_raw_SET((uint16_t)(uint16_t)30465, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
    p37_target_system_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
    p37_target_component_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
    p37_start_index_SET((int16_t)(int16_t) -20168, PH.base.pack) ;
    p37_end_index_SET((int16_t)(int16_t) -10590, PH.base.pack) ;
    p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
    p38_target_system_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
    p38_target_component_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
    p38_start_index_SET((int16_t)(int16_t) -32113, PH.base.pack) ;
    p38_end_index_SET((int16_t)(int16_t) -7036, PH.base.pack) ;
    p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
    p39_target_system_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
    p39_target_component_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
    p39_seq_SET((uint16_t)(uint16_t)2362, PH.base.pack) ;
    p39_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
    p39_command_SET(e_MAV_CMD_MAV_CMD_CONDITION_LAST, PH.base.pack) ;
    p39_current_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
    p39_autocontinue_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
    p39_param1_SET((float)2.22558E38F, PH.base.pack) ;
    p39_param2_SET((float) -1.0450476E38F, PH.base.pack) ;
    p39_param3_SET((float)6.092712E37F, PH.base.pack) ;
    p39_param4_SET((float) -1.369415E38F, PH.base.pack) ;
    p39_x_SET((float) -2.1084901E38F, PH.base.pack) ;
    p39_y_SET((float) -2.024752E38F, PH.base.pack) ;
    p39_z_SET((float) -2.7644953E38F, PH.base.pack) ;
    p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
    p40_target_system_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
    p40_target_component_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
    p40_seq_SET((uint16_t)(uint16_t)28085, PH.base.pack) ;
    p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
    p41_target_system_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
    p41_target_component_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
    p41_seq_SET((uint16_t)(uint16_t)25922, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
    p42_seq_SET((uint16_t)(uint16_t)55047, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
    p43_target_system_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
    p43_target_component_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
    p44_target_system_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
    p44_target_component_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
    p44_count_SET((uint16_t)(uint16_t)44015, PH.base.pack) ;
    p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
    p45_target_system_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
    p45_target_component_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
    p46_seq_SET((uint16_t)(uint16_t)44126, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
    p47_target_system_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
    p47_target_component_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
    p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_UNSUPPORTED, PH.base.pack) ;
    p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
    p48_target_system_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    p48_latitude_SET((int32_t) -1183007116, PH.base.pack) ;
    p48_longitude_SET((int32_t) -51632521, PH.base.pack) ;
    p48_altitude_SET((int32_t) -449410717, PH.base.pack) ;
    p48_time_usec_SET((uint64_t)7709315784454351722L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
    p49_latitude_SET((int32_t)1050040058, PH.base.pack) ;
    p49_longitude_SET((int32_t)186496614, PH.base.pack) ;
    p49_altitude_SET((int32_t)605228830, PH.base.pack) ;
    p49_time_usec_SET((uint64_t)5700338116940403896L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
    p50_target_system_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
    p50_target_component_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
    {
        char16_t   param_id = "jawdnxcm";
        p50_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p50_param_index_SET((int16_t)(int16_t)23328, PH.base.pack) ;
    p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
    p50_param_value0_SET((float) -1.8524452E38F, PH.base.pack) ;
    p50_scale_SET((float)2.073261E38F, PH.base.pack) ;
    p50_param_value_min_SET((float) -1.6046378E38F, PH.base.pack) ;
    p50_param_value_max_SET((float) -9.212174E36F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
    p51_target_system_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
    p51_target_component_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
    p51_seq_SET((uint16_t)(uint16_t)54606, PH.base.pack) ;
    p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
    p54_target_system_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    p54_target_component_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
    p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
    p54_p1x_SET((float) -1.1123954E38F, PH.base.pack) ;
    p54_p1y_SET((float) -1.6631131E38F, PH.base.pack) ;
    p54_p1z_SET((float)2.962272E38F, PH.base.pack) ;
    p54_p2x_SET((float)8.687196E37F, PH.base.pack) ;
    p54_p2y_SET((float) -2.9389915E38F, PH.base.pack) ;
    p54_p2z_SET((float) -2.4148523E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
    p55_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
    p55_p1x_SET((float)3.191105E38F, PH.base.pack) ;
    p55_p1y_SET((float) -1.6744839E38F, PH.base.pack) ;
    p55_p1z_SET((float)8.287071E37F, PH.base.pack) ;
    p55_p2x_SET((float)4.6480143E37F, PH.base.pack) ;
    p55_p2y_SET((float) -3.5481807E37F, PH.base.pack) ;
    p55_p2z_SET((float)2.2746238E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
    p61_time_usec_SET((uint64_t)3836543023249443351L, PH.base.pack) ;
    {
        float  q [] =  {2.8554436E38F, 1.9626293E37F, -2.598871E38F, 1.8409191E38F};
        p61_q_SET(&q, 0, &PH.base.pack) ;
    }
    p61_rollspeed_SET((float) -2.783615E38F, PH.base.pack) ;
    p61_pitchspeed_SET((float)9.16851E37F, PH.base.pack) ;
    p61_yawspeed_SET((float)1.2327944E38F, PH.base.pack) ;
    {
        float  covariance [] =  {2.0394362E38F, 2.083172E38F, 3.3029302E38F, 1.9099617E38F, -2.7628771E38F, 2.7735026E37F, 3.9887623E37F, 2.05177E37F, -1.428461E38F};
        p61_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
    p62_nav_roll_SET((float) -1.8149696E38F, PH.base.pack) ;
    p62_nav_pitch_SET((float) -5.8526303E37F, PH.base.pack) ;
    p62_nav_bearing_SET((int16_t)(int16_t) -2793, PH.base.pack) ;
    p62_target_bearing_SET((int16_t)(int16_t) -29634, PH.base.pack) ;
    p62_wp_dist_SET((uint16_t)(uint16_t)60059, PH.base.pack) ;
    p62_alt_error_SET((float) -2.1365446E38F, PH.base.pack) ;
    p62_aspd_error_SET((float) -1.3096406E38F, PH.base.pack) ;
    p62_xtrack_error_SET((float) -7.184584E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
    p63_time_usec_SET((uint64_t)8980745358086888168L, PH.base.pack) ;
    p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE, PH.base.pack) ;
    p63_lat_SET((int32_t)979658897, PH.base.pack) ;
    p63_lon_SET((int32_t)1469993051, PH.base.pack) ;
    p63_alt_SET((int32_t)1583696582, PH.base.pack) ;
    p63_relative_alt_SET((int32_t) -466043284, PH.base.pack) ;
    p63_vx_SET((float) -8.674508E37F, PH.base.pack) ;
    p63_vy_SET((float)9.808179E37F, PH.base.pack) ;
    p63_vz_SET((float) -6.295627E37F, PH.base.pack) ;
    {
        float  covariance [] =  {-1.492338E37F, -9.578294E37F, -1.3857746E38F, 1.5342269E38F, 9.773934E36F, 1.3411047E38F, 2.6211381E37F, 3.2812298E38F, 3.0377216E38F, 2.4310356E38F, 3.1349486E38F, 2.9480653E38F, 3.3356228E38F, -2.7780608E38F, -8.094973E36F, -3.3118027E38F, -1.8688137E38F, 6.5762473E37F, 9.98091E37F, -1.830246E38F, -1.8838105E38F, -3.264768E38F, -1.0882063E38F, -7.6492755E37F, 1.0119421E38F, 1.0890555E38F, 2.3926737E38F, 3.2987289E38F, 1.5886628E38F, 2.819655E38F, -3.1200709E38F, 3.5433943E37F, -1.0589944E38F, -1.5446096E38F, -2.8238619E38F, -2.67178E38F};
        p63_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
    p64_time_usec_SET((uint64_t)5753134687399660105L, PH.base.pack) ;
    p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
    p64_x_SET((float) -1.901284E38F, PH.base.pack) ;
    p64_y_SET((float)2.9160156E38F, PH.base.pack) ;
    p64_z_SET((float)3.1731546E38F, PH.base.pack) ;
    p64_vx_SET((float) -2.5981176E38F, PH.base.pack) ;
    p64_vy_SET((float)2.312403E38F, PH.base.pack) ;
    p64_vz_SET((float) -3.04813E38F, PH.base.pack) ;
    p64_ax_SET((float)9.552652E37F, PH.base.pack) ;
    p64_ay_SET((float) -2.1991953E38F, PH.base.pack) ;
    p64_az_SET((float)2.256777E37F, PH.base.pack) ;
    {
        float  covariance [] =  {-7.097883E37F, -1.9425792E37F, -4.7785565E37F, 1.2634698E38F, 2.3860732E38F, 1.4851076E38F, -1.2895471E38F, -1.1322745E38F, 2.8198666E36F, -5.5376637E37F, 1.0397984E38F, -2.2598476E38F, 3.2714502E38F, -2.7028518E38F, 3.399118E38F, 3.2738499E38F, -7.7773797E37F, -1.352764E38F, -2.980781E38F, 5.583368E37F, -2.9645993E38F, -1.1126759E38F, 2.9627687E37F, 1.1181961E37F, -2.7923493E38F, -2.0452885E38F, -3.1125743E38F, 6.776342E37F, 3.3896374E37F, 1.9511938E38F, -1.5830327E38F, 1.5340033E38F, 9.431621E37F, 1.968191E38F, -2.1883323E38F, -3.0730197E38F, 7.358649E37F, -1.0588617E38F, -1.1015954E38F, 5.536831E37F, -2.8593114E37F, -9.260119E37F, -2.3460777E38F, -2.290293E38F, -4.2248934E37F};
        p64_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
    p65_time_boot_ms_SET((uint32_t)36474232L, PH.base.pack) ;
    p65_chancount_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
    p65_chan1_raw_SET((uint16_t)(uint16_t)24610, PH.base.pack) ;
    p65_chan2_raw_SET((uint16_t)(uint16_t)35914, PH.base.pack) ;
    p65_chan3_raw_SET((uint16_t)(uint16_t)54055, PH.base.pack) ;
    p65_chan4_raw_SET((uint16_t)(uint16_t)51229, PH.base.pack) ;
    p65_chan5_raw_SET((uint16_t)(uint16_t)9502, PH.base.pack) ;
    p65_chan6_raw_SET((uint16_t)(uint16_t)17086, PH.base.pack) ;
    p65_chan7_raw_SET((uint16_t)(uint16_t)55896, PH.base.pack) ;
    p65_chan8_raw_SET((uint16_t)(uint16_t)20144, PH.base.pack) ;
    p65_chan9_raw_SET((uint16_t)(uint16_t)10091, PH.base.pack) ;
    p65_chan10_raw_SET((uint16_t)(uint16_t)61456, PH.base.pack) ;
    p65_chan11_raw_SET((uint16_t)(uint16_t)8947, PH.base.pack) ;
    p65_chan12_raw_SET((uint16_t)(uint16_t)3029, PH.base.pack) ;
    p65_chan13_raw_SET((uint16_t)(uint16_t)47665, PH.base.pack) ;
    p65_chan14_raw_SET((uint16_t)(uint16_t)57821, PH.base.pack) ;
    p65_chan15_raw_SET((uint16_t)(uint16_t)47608, PH.base.pack) ;
    p65_chan16_raw_SET((uint16_t)(uint16_t)43674, PH.base.pack) ;
    p65_chan17_raw_SET((uint16_t)(uint16_t)23795, PH.base.pack) ;
    p65_chan18_raw_SET((uint16_t)(uint16_t)8186, PH.base.pack) ;
    p65_rssi_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
    p66_target_system_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
    p66_target_component_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
    p66_req_stream_id_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
    p66_req_message_rate_SET((uint16_t)(uint16_t)54061, PH.base.pack) ;
    p66_start_stop_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
    p67_stream_id_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
    p67_message_rate_SET((uint16_t)(uint16_t)32159, PH.base.pack) ;
    p67_on_off_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
    p69_target_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    p69_x_SET((int16_t)(int16_t)5798, PH.base.pack) ;
    p69_y_SET((int16_t)(int16_t)8846, PH.base.pack) ;
    p69_z_SET((int16_t)(int16_t)32262, PH.base.pack) ;
    p69_r_SET((int16_t)(int16_t)5481, PH.base.pack) ;
    p69_buttons_SET((uint16_t)(uint16_t)5668, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
    p70_target_system_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
    p70_target_component_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
    p70_chan1_raw_SET((uint16_t)(uint16_t)25417, PH.base.pack) ;
    p70_chan2_raw_SET((uint16_t)(uint16_t)26203, PH.base.pack) ;
    p70_chan3_raw_SET((uint16_t)(uint16_t)8546, PH.base.pack) ;
    p70_chan4_raw_SET((uint16_t)(uint16_t)26343, PH.base.pack) ;
    p70_chan5_raw_SET((uint16_t)(uint16_t)6425, PH.base.pack) ;
    p70_chan6_raw_SET((uint16_t)(uint16_t)44741, PH.base.pack) ;
    p70_chan7_raw_SET((uint16_t)(uint16_t)23366, PH.base.pack) ;
    p70_chan8_raw_SET((uint16_t)(uint16_t)33957, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
    p73_target_system_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
    p73_target_component_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
    p73_seq_SET((uint16_t)(uint16_t)17036, PH.base.pack) ;
    p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
    p73_command_SET(e_MAV_CMD_MAV_CMD_DO_SEND_BANNER, PH.base.pack) ;
    p73_current_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
    p73_autocontinue_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
    p73_param1_SET((float) -1.2063085E38F, PH.base.pack) ;
    p73_param2_SET((float) -1.8883082E38F, PH.base.pack) ;
    p73_param3_SET((float)2.3187047E38F, PH.base.pack) ;
    p73_param4_SET((float)2.3115588E38F, PH.base.pack) ;
    p73_x_SET((int32_t)929012062, PH.base.pack) ;
    p73_y_SET((int32_t)645270781, PH.base.pack) ;
    p73_z_SET((float)2.9494411E37F, PH.base.pack) ;
    p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
    p74_airspeed_SET((float) -2.3765956E38F, PH.base.pack) ;
    p74_groundspeed_SET((float) -1.1947844E38F, PH.base.pack) ;
    p74_heading_SET((int16_t)(int16_t) -5795, PH.base.pack) ;
    p74_throttle_SET((uint16_t)(uint16_t)13543, PH.base.pack) ;
    p74_alt_SET((float) -1.8584273E38F, PH.base.pack) ;
    p74_climb_SET((float) -2.7143338E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
    p75_target_system_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
    p75_target_component_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
    p75_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
    p75_command_SET(e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT, PH.base.pack) ;
    p75_current_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
    p75_autocontinue_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
    p75_param1_SET((float) -5.708975E37F, PH.base.pack) ;
    p75_param2_SET((float)1.712717E38F, PH.base.pack) ;
    p75_param3_SET((float)4.657074E37F, PH.base.pack) ;
    p75_param4_SET((float) -1.6418198E38F, PH.base.pack) ;
    p75_x_SET((int32_t) -57596391, PH.base.pack) ;
    p75_y_SET((int32_t)1966543052, PH.base.pack) ;
    p75_z_SET((float) -1.4717572E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
    p76_target_system_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
    p76_target_component_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
    p76_command_SET(e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION, PH.base.pack) ;
    p76_confirmation_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
    p76_param1_SET((float) -2.7541452E38F, PH.base.pack) ;
    p76_param2_SET((float) -5.0639797E37F, PH.base.pack) ;
    p76_param3_SET((float) -1.5231783E37F, PH.base.pack) ;
    p76_param4_SET((float)2.2840936E38F, PH.base.pack) ;
    p76_param5_SET((float)8.7214716E36F, PH.base.pack) ;
    p76_param6_SET((float) -3.1100824E37F, PH.base.pack) ;
    p76_param7_SET((float)9.280989E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
    p77_command_SET(e_MAV_CMD_MAV_CMD_CONDITION_GATE, PH.base.pack) ;
    p77_result_SET(e_MAV_RESULT_MAV_RESULT_TEMPORARILY_REJECTED, PH.base.pack) ;
    p77_progress_SET((uint8_t)(uint8_t)5, &PH) ;
    p77_result_param2_SET((int32_t)2106093670, &PH) ;
    p77_target_system_SET((uint8_t)(uint8_t)4, &PH) ;
    p77_target_component_SET((uint8_t)(uint8_t)134, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
    p81_time_boot_ms_SET((uint32_t)2280608423L, PH.base.pack) ;
    p81_roll_SET((float)3.3688594E38F, PH.base.pack) ;
    p81_pitch_SET((float) -2.831012E38F, PH.base.pack) ;
    p81_yaw_SET((float) -1.6482548E37F, PH.base.pack) ;
    p81_thrust_SET((float) -1.8350386E38F, PH.base.pack) ;
    p81_mode_switch_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
    p81_manual_override_switch_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
    p82_time_boot_ms_SET((uint32_t)2100610741L, PH.base.pack) ;
    p82_target_system_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
    p82_target_component_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
    p82_type_mask_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
    {
        float  q [] =  {-2.206433E38F, -9.14711E37F, 2.1355763E38F, -4.5235726E37F};
        p82_q_SET(&q, 0, &PH.base.pack) ;
    }
    p82_body_roll_rate_SET((float)2.9071672E38F, PH.base.pack) ;
    p82_body_pitch_rate_SET((float)3.034331E38F, PH.base.pack) ;
    p82_body_yaw_rate_SET((float) -1.7933966E38F, PH.base.pack) ;
    p82_thrust_SET((float) -3.0765913E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
    p83_time_boot_ms_SET((uint32_t)2270149589L, PH.base.pack) ;
    p83_type_mask_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
    {
        float  q [] =  {-1.2409717E38F, -5.129719E35F, 1.2898662E38F, 2.7059953E38F};
        p83_q_SET(&q, 0, &PH.base.pack) ;
    }
    p83_body_roll_rate_SET((float)1.5008479E38F, PH.base.pack) ;
    p83_body_pitch_rate_SET((float) -2.8980985E38F, PH.base.pack) ;
    p83_body_yaw_rate_SET((float) -2.3381883E36F, PH.base.pack) ;
    p83_thrust_SET((float) -2.796097E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
    p84_time_boot_ms_SET((uint32_t)4264097479L, PH.base.pack) ;
    p84_target_system_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
    p84_target_component_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
    p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
    p84_type_mask_SET((uint16_t)(uint16_t)19593, PH.base.pack) ;
    p84_x_SET((float)2.0196925E38F, PH.base.pack) ;
    p84_y_SET((float)1.5843063E38F, PH.base.pack) ;
    p84_z_SET((float)3.9043192E37F, PH.base.pack) ;
    p84_vx_SET((float) -1.9370444E38F, PH.base.pack) ;
    p84_vy_SET((float) -3.3265476E38F, PH.base.pack) ;
    p84_vz_SET((float) -1.822911E38F, PH.base.pack) ;
    p84_afx_SET((float) -5.9389633E37F, PH.base.pack) ;
    p84_afy_SET((float) -2.2179726E38F, PH.base.pack) ;
    p84_afz_SET((float)1.968063E38F, PH.base.pack) ;
    p84_yaw_SET((float) -2.3148897E36F, PH.base.pack) ;
    p84_yaw_rate_SET((float)3.1729727E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
    p86_time_boot_ms_SET((uint32_t)1762493528L, PH.base.pack) ;
    p86_target_system_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
    p86_target_component_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
    p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
    p86_type_mask_SET((uint16_t)(uint16_t)10475, PH.base.pack) ;
    p86_lat_int_SET((int32_t)178947355, PH.base.pack) ;
    p86_lon_int_SET((int32_t) -131629010, PH.base.pack) ;
    p86_alt_SET((float) -1.5265478E38F, PH.base.pack) ;
    p86_vx_SET((float) -9.364079E37F, PH.base.pack) ;
    p86_vy_SET((float)1.768468E38F, PH.base.pack) ;
    p86_vz_SET((float) -7.0925543E37F, PH.base.pack) ;
    p86_afx_SET((float)6.9198516E37F, PH.base.pack) ;
    p86_afy_SET((float)1.7726106E38F, PH.base.pack) ;
    p86_afz_SET((float) -5.4184396E37F, PH.base.pack) ;
    p86_yaw_SET((float)2.6687777E38F, PH.base.pack) ;
    p86_yaw_rate_SET((float)1.2254463E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
    p87_time_boot_ms_SET((uint32_t)3608610932L, PH.base.pack) ;
    p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
    p87_type_mask_SET((uint16_t)(uint16_t)35329, PH.base.pack) ;
    p87_lat_int_SET((int32_t) -219563813, PH.base.pack) ;
    p87_lon_int_SET((int32_t)147091694, PH.base.pack) ;
    p87_alt_SET((float) -6.182761E37F, PH.base.pack) ;
    p87_vx_SET((float)7.9302614E37F, PH.base.pack) ;
    p87_vy_SET((float)9.001249E37F, PH.base.pack) ;
    p87_vz_SET((float)2.2233322E38F, PH.base.pack) ;
    p87_afx_SET((float) -2.2819907E38F, PH.base.pack) ;
    p87_afy_SET((float) -2.3682865E38F, PH.base.pack) ;
    p87_afz_SET((float) -2.5046946E38F, PH.base.pack) ;
    p87_yaw_SET((float) -1.218221E38F, PH.base.pack) ;
    p87_yaw_rate_SET((float)4.6033276E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
    p89_time_boot_ms_SET((uint32_t)2551853937L, PH.base.pack) ;
    p89_x_SET((float)2.5596922E38F, PH.base.pack) ;
    p89_y_SET((float) -2.3761985E38F, PH.base.pack) ;
    p89_z_SET((float)4.741769E37F, PH.base.pack) ;
    p89_roll_SET((float)1.6708633E38F, PH.base.pack) ;
    p89_pitch_SET((float) -2.8340185E38F, PH.base.pack) ;
    p89_yaw_SET((float)2.4007641E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
    p90_time_usec_SET((uint64_t)8312434524061418167L, PH.base.pack) ;
    p90_roll_SET((float)1.2693974E38F, PH.base.pack) ;
    p90_pitch_SET((float) -1.6517995E38F, PH.base.pack) ;
    p90_yaw_SET((float) -1.0144464E38F, PH.base.pack) ;
    p90_rollspeed_SET((float) -1.0668886E38F, PH.base.pack) ;
    p90_pitchspeed_SET((float) -2.8366098E38F, PH.base.pack) ;
    p90_yawspeed_SET((float) -7.308968E37F, PH.base.pack) ;
    p90_lat_SET((int32_t)1577505239, PH.base.pack) ;
    p90_lon_SET((int32_t)1438004165, PH.base.pack) ;
    p90_alt_SET((int32_t)1147408499, PH.base.pack) ;
    p90_vx_SET((int16_t)(int16_t)31614, PH.base.pack) ;
    p90_vy_SET((int16_t)(int16_t)13033, PH.base.pack) ;
    p90_vz_SET((int16_t)(int16_t)22269, PH.base.pack) ;
    p90_xacc_SET((int16_t)(int16_t) -15907, PH.base.pack) ;
    p90_yacc_SET((int16_t)(int16_t) -5272, PH.base.pack) ;
    p90_zacc_SET((int16_t)(int16_t)16859, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
    p91_time_usec_SET((uint64_t)7824065097881371023L, PH.base.pack) ;
    p91_roll_ailerons_SET((float) -1.9637853E38F, PH.base.pack) ;
    p91_pitch_elevator_SET((float) -1.1235114E38F, PH.base.pack) ;
    p91_yaw_rudder_SET((float)3.7165356E37F, PH.base.pack) ;
    p91_throttle_SET((float) -2.9887617E38F, PH.base.pack) ;
    p91_aux1_SET((float)2.6169294E38F, PH.base.pack) ;
    p91_aux2_SET((float)7.493669E37F, PH.base.pack) ;
    p91_aux3_SET((float) -1.0732825E38F, PH.base.pack) ;
    p91_aux4_SET((float)1.2021228E38F, PH.base.pack) ;
    p91_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_DISARMED, PH.base.pack) ;
    p91_nav_mode_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
    p92_time_usec_SET((uint64_t)1744773223897207602L, PH.base.pack) ;
    p92_chan1_raw_SET((uint16_t)(uint16_t)55505, PH.base.pack) ;
    p92_chan2_raw_SET((uint16_t)(uint16_t)47633, PH.base.pack) ;
    p92_chan3_raw_SET((uint16_t)(uint16_t)9556, PH.base.pack) ;
    p92_chan4_raw_SET((uint16_t)(uint16_t)18580, PH.base.pack) ;
    p92_chan5_raw_SET((uint16_t)(uint16_t)53739, PH.base.pack) ;
    p92_chan6_raw_SET((uint16_t)(uint16_t)32865, PH.base.pack) ;
    p92_chan7_raw_SET((uint16_t)(uint16_t)19046, PH.base.pack) ;
    p92_chan8_raw_SET((uint16_t)(uint16_t)7254, PH.base.pack) ;
    p92_chan9_raw_SET((uint16_t)(uint16_t)34381, PH.base.pack) ;
    p92_chan10_raw_SET((uint16_t)(uint16_t)64583, PH.base.pack) ;
    p92_chan11_raw_SET((uint16_t)(uint16_t)39399, PH.base.pack) ;
    p92_chan12_raw_SET((uint16_t)(uint16_t)30680, PH.base.pack) ;
    p92_rssi_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
    p93_time_usec_SET((uint64_t)5914389144798745929L, PH.base.pack) ;
    {
        float  controls [] =  {1.5075924E38F, -4.298959E37F, 1.5495163E38F, 9.796703E37F, 1.3970774E38F, -3.260935E38F, 2.970473E38F, -1.7624329E38F, 6.2602595E37F, 8.947302E37F, -8.616701E36F, 2.4861157E38F, 1.7422257E38F, 3.1726315E38F, -2.6224298E38F, 2.5871116E38F};
        p93_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    p93_mode_SET(e_MAV_MODE_MAV_MODE_TEST_ARMED, PH.base.pack) ;
    p93_flags_SET((uint64_t)1319672796685716507L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
    p100_time_usec_SET((uint64_t)3930086568366578817L, PH.base.pack) ;
    p100_sensor_id_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
    p100_flow_x_SET((int16_t)(int16_t)24113, PH.base.pack) ;
    p100_flow_y_SET((int16_t)(int16_t) -15586, PH.base.pack) ;
    p100_flow_comp_m_x_SET((float)3.3131424E38F, PH.base.pack) ;
    p100_flow_comp_m_y_SET((float) -1.7950263E38F, PH.base.pack) ;
    p100_quality_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
    p100_ground_distance_SET((float) -2.3959708E38F, PH.base.pack) ;
    p100_flow_rate_x_SET((float)3.0940165E38F, &PH) ;
    p100_flow_rate_y_SET((float)2.9519958E38F, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
    p101_usec_SET((uint64_t)6165612741461081670L, PH.base.pack) ;
    p101_x_SET((float)3.1770164E38F, PH.base.pack) ;
    p101_y_SET((float) -2.64665E38F, PH.base.pack) ;
    p101_z_SET((float)2.7192114E37F, PH.base.pack) ;
    p101_roll_SET((float)3.008824E37F, PH.base.pack) ;
    p101_pitch_SET((float)3.2694844E37F, PH.base.pack) ;
    p101_yaw_SET((float) -4.1917075E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
    p102_usec_SET((uint64_t)3917619219381770554L, PH.base.pack) ;
    p102_x_SET((float)2.4979201E38F, PH.base.pack) ;
    p102_y_SET((float) -3.1823705E38F, PH.base.pack) ;
    p102_z_SET((float)2.5659714E38F, PH.base.pack) ;
    p102_roll_SET((float) -2.6633931E38F, PH.base.pack) ;
    p102_pitch_SET((float)2.3039876E38F, PH.base.pack) ;
    p102_yaw_SET((float) -2.1710814E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
    p103_usec_SET((uint64_t)5882985575872365917L, PH.base.pack) ;
    p103_x_SET((float) -2.2038313E38F, PH.base.pack) ;
    p103_y_SET((float)1.2950806E38F, PH.base.pack) ;
    p103_z_SET((float)7.1794573E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
    p104_usec_SET((uint64_t)2581166373331438874L, PH.base.pack) ;
    p104_x_SET((float)9.253112E37F, PH.base.pack) ;
    p104_y_SET((float)1.3174324E38F, PH.base.pack) ;
    p104_z_SET((float) -1.8150879E37F, PH.base.pack) ;
    p104_roll_SET((float)2.2785066E38F, PH.base.pack) ;
    p104_pitch_SET((float)5.204356E37F, PH.base.pack) ;
    p104_yaw_SET((float)2.549939E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
    p105_time_usec_SET((uint64_t)3134918521038205193L, PH.base.pack) ;
    p105_xacc_SET((float) -7.4427676E37F, PH.base.pack) ;
    p105_yacc_SET((float) -1.2283214E37F, PH.base.pack) ;
    p105_zacc_SET((float) -2.6769821E38F, PH.base.pack) ;
    p105_xgyro_SET((float) -2.0149503E38F, PH.base.pack) ;
    p105_ygyro_SET((float) -3.2804E38F, PH.base.pack) ;
    p105_zgyro_SET((float) -9.676253E37F, PH.base.pack) ;
    p105_xmag_SET((float) -3.2863039E38F, PH.base.pack) ;
    p105_ymag_SET((float)3.2472564E38F, PH.base.pack) ;
    p105_zmag_SET((float) -2.4082177E38F, PH.base.pack) ;
    p105_abs_pressure_SET((float) -2.824019E38F, PH.base.pack) ;
    p105_diff_pressure_SET((float)3.138774E38F, PH.base.pack) ;
    p105_pressure_alt_SET((float)3.0192949E38F, PH.base.pack) ;
    p105_temperature_SET((float) -8.583163E37F, PH.base.pack) ;
    p105_fields_updated_SET((uint16_t)(uint16_t)1385, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
    p106_time_usec_SET((uint64_t)4305486796294486257L, PH.base.pack) ;
    p106_sensor_id_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
    p106_integration_time_us_SET((uint32_t)4061342504L, PH.base.pack) ;
    p106_integrated_x_SET((float) -2.7397477E38F, PH.base.pack) ;
    p106_integrated_y_SET((float) -7.260246E37F, PH.base.pack) ;
    p106_integrated_xgyro_SET((float) -2.6045E38F, PH.base.pack) ;
    p106_integrated_ygyro_SET((float)9.718579E37F, PH.base.pack) ;
    p106_integrated_zgyro_SET((float)3.2438816E38F, PH.base.pack) ;
    p106_temperature_SET((int16_t)(int16_t) -23225, PH.base.pack) ;
    p106_quality_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
    p106_time_delta_distance_us_SET((uint32_t)4033133855L, PH.base.pack) ;
    p106_distance_SET((float)1.0968919E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
    p107_time_usec_SET((uint64_t)5333369974839408915L, PH.base.pack) ;
    p107_xacc_SET((float)9.3695916E36F, PH.base.pack) ;
    p107_yacc_SET((float) -1.245651E38F, PH.base.pack) ;
    p107_zacc_SET((float) -1.548367E38F, PH.base.pack) ;
    p107_xgyro_SET((float) -2.8994717E38F, PH.base.pack) ;
    p107_ygyro_SET((float) -1.976965E38F, PH.base.pack) ;
    p107_zgyro_SET((float) -2.1710431E38F, PH.base.pack) ;
    p107_xmag_SET((float)4.9210014E37F, PH.base.pack) ;
    p107_ymag_SET((float) -1.3117023E38F, PH.base.pack) ;
    p107_zmag_SET((float)2.2047044E38F, PH.base.pack) ;
    p107_abs_pressure_SET((float) -3.2200512E38F, PH.base.pack) ;
    p107_diff_pressure_SET((float) -8.901475E37F, PH.base.pack) ;
    p107_pressure_alt_SET((float)8.669209E37F, PH.base.pack) ;
    p107_temperature_SET((float) -2.9602438E38F, PH.base.pack) ;
    p107_fields_updated_SET((uint32_t)4005308110L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
    p108_q1_SET((float) -1.2989525E36F, PH.base.pack) ;
    p108_q2_SET((float)1.417247E38F, PH.base.pack) ;
    p108_q3_SET((float)3.2427186E38F, PH.base.pack) ;
    p108_q4_SET((float) -1.9384136E38F, PH.base.pack) ;
    p108_roll_SET((float) -2.0430487E38F, PH.base.pack) ;
    p108_pitch_SET((float)2.7020094E38F, PH.base.pack) ;
    p108_yaw_SET((float)3.3760483E38F, PH.base.pack) ;
    p108_xacc_SET((float) -3.2642216E38F, PH.base.pack) ;
    p108_yacc_SET((float) -1.2466872E38F, PH.base.pack) ;
    p108_zacc_SET((float)6.236136E37F, PH.base.pack) ;
    p108_xgyro_SET((float) -1.125959E38F, PH.base.pack) ;
    p108_ygyro_SET((float) -1.048436E38F, PH.base.pack) ;
    p108_zgyro_SET((float) -1.5938445E38F, PH.base.pack) ;
    p108_lat_SET((float)5.313473E37F, PH.base.pack) ;
    p108_lon_SET((float) -2.828134E37F, PH.base.pack) ;
    p108_alt_SET((float) -1.3576451E38F, PH.base.pack) ;
    p108_std_dev_horz_SET((float)1.3829161E38F, PH.base.pack) ;
    p108_std_dev_vert_SET((float)2.8356364E38F, PH.base.pack) ;
    p108_vn_SET((float) -2.6874012E38F, PH.base.pack) ;
    p108_ve_SET((float)2.13743E38F, PH.base.pack) ;
    p108_vd_SET((float)3.0321707E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
    p109_rssi_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
    p109_remrssi_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
    p109_txbuf_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
    p109_noise_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
    p109_remnoise_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
    p109_rxerrors_SET((uint16_t)(uint16_t)45250, PH.base.pack) ;
    p109_fixed__SET((uint16_t)(uint16_t)54559, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
    p110_target_network_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
    p110_target_system_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
    p110_target_component_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)236, (uint8_t)244, (uint8_t)180, (uint8_t)114, (uint8_t)159, (uint8_t)148, (uint8_t)13, (uint8_t)93, (uint8_t)73, (uint8_t)115, (uint8_t)148, (uint8_t)40, (uint8_t)115, (uint8_t)222, (uint8_t)232, (uint8_t)59, (uint8_t)63, (uint8_t)93, (uint8_t)73, (uint8_t)237, (uint8_t)2, (uint8_t)57, (uint8_t)52, (uint8_t)110, (uint8_t)175, (uint8_t)171, (uint8_t)241, (uint8_t)88, (uint8_t)214, (uint8_t)217, (uint8_t)88, (uint8_t)229, (uint8_t)22, (uint8_t)114, (uint8_t)163, (uint8_t)83, (uint8_t)125, (uint8_t)28, (uint8_t)234, (uint8_t)183, (uint8_t)30, (uint8_t)122, (uint8_t)14, (uint8_t)204, (uint8_t)152, (uint8_t)206, (uint8_t)66, (uint8_t)213, (uint8_t)107, (uint8_t)9, (uint8_t)209, (uint8_t)142, (uint8_t)234, (uint8_t)230, (uint8_t)131, (uint8_t)238, (uint8_t)173, (uint8_t)217, (uint8_t)255, (uint8_t)17, (uint8_t)227, (uint8_t)88, (uint8_t)95, (uint8_t)7, (uint8_t)209, (uint8_t)123, (uint8_t)27, (uint8_t)192, (uint8_t)46, (uint8_t)51, (uint8_t)152, (uint8_t)124, (uint8_t)11, (uint8_t)73, (uint8_t)145, (uint8_t)29, (uint8_t)192, (uint8_t)22, (uint8_t)90, (uint8_t)225, (uint8_t)73, (uint8_t)96, (uint8_t)18, (uint8_t)47, (uint8_t)75, (uint8_t)43, (uint8_t)239, (uint8_t)5, (uint8_t)247, (uint8_t)198, (uint8_t)40, (uint8_t)227, (uint8_t)80, (uint8_t)46, (uint8_t)30, (uint8_t)252, (uint8_t)108, (uint8_t)78, (uint8_t)101, (uint8_t)232, (uint8_t)98, (uint8_t)76, (uint8_t)16, (uint8_t)107, (uint8_t)205, (uint8_t)94, (uint8_t)31, (uint8_t)13, (uint8_t)36, (uint8_t)59, (uint8_t)176, (uint8_t)15, (uint8_t)32, (uint8_t)55, (uint8_t)228, (uint8_t)202, (uint8_t)241, (uint8_t)8, (uint8_t)128, (uint8_t)83, (uint8_t)6, (uint8_t)124, (uint8_t)213, (uint8_t)184, (uint8_t)71, (uint8_t)185, (uint8_t)149, (uint8_t)3, (uint8_t)95, (uint8_t)193, (uint8_t)123, (uint8_t)72, (uint8_t)109, (uint8_t)159, (uint8_t)203, (uint8_t)68, (uint8_t)233, (uint8_t)77, (uint8_t)0, (uint8_t)143, (uint8_t)20, (uint8_t)157, (uint8_t)126, (uint8_t)14, (uint8_t)245, (uint8_t)54, (uint8_t)73, (uint8_t)234, (uint8_t)59, (uint8_t)229, (uint8_t)126, (uint8_t)214, (uint8_t)69, (uint8_t)96, (uint8_t)181, (uint8_t)200, (uint8_t)193, (uint8_t)148, (uint8_t)119, (uint8_t)60, (uint8_t)114, (uint8_t)145, (uint8_t)79, (uint8_t)90, (uint8_t)155, (uint8_t)50, (uint8_t)237, (uint8_t)49, (uint8_t)183, (uint8_t)106, (uint8_t)21, (uint8_t)179, (uint8_t)35, (uint8_t)94, (uint8_t)167, (uint8_t)222, (uint8_t)19, (uint8_t)147, (uint8_t)167, (uint8_t)87, (uint8_t)205, (uint8_t)92, (uint8_t)200, (uint8_t)185, (uint8_t)53, (uint8_t)119, (uint8_t)192, (uint8_t)114, (uint8_t)101, (uint8_t)221, (uint8_t)143, (uint8_t)119, (uint8_t)237, (uint8_t)25, (uint8_t)248, (uint8_t)83, (uint8_t)135, (uint8_t)38, (uint8_t)168, (uint8_t)187, (uint8_t)201, (uint8_t)23, (uint8_t)244, (uint8_t)109, (uint8_t)253, (uint8_t)215, (uint8_t)242, (uint8_t)191, (uint8_t)106, (uint8_t)116, (uint8_t)43, (uint8_t)74, (uint8_t)247, (uint8_t)144, (uint8_t)235, (uint8_t)79, (uint8_t)40, (uint8_t)218, (uint8_t)152, (uint8_t)204, (uint8_t)169, (uint8_t)90, (uint8_t)3, (uint8_t)73, (uint8_t)1, (uint8_t)133, (uint8_t)95, (uint8_t)2, (uint8_t)50, (uint8_t)123, (uint8_t)133, (uint8_t)175, (uint8_t)22, (uint8_t)88, (uint8_t)131, (uint8_t)170, (uint8_t)93, (uint8_t)1, (uint8_t)221, (uint8_t)60, (uint8_t)109, (uint8_t)212, (uint8_t)148, (uint8_t)94, (uint8_t)123, (uint8_t)145, (uint8_t)28, (uint8_t)249, (uint8_t)99, (uint8_t)111, (uint8_t)25};
        p110_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
    p111_tc1_SET((int64_t) -5423754372094889200L, PH.base.pack) ;
    p111_ts1_SET((int64_t)3982516629444983542L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
    p112_time_usec_SET((uint64_t)2270779481285246034L, PH.base.pack) ;
    p112_seq_SET((uint32_t)2915250215L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
    p113_time_usec_SET((uint64_t)4273611755634580205L, PH.base.pack) ;
    p113_fix_type_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
    p113_lat_SET((int32_t) -67668763, PH.base.pack) ;
    p113_lon_SET((int32_t) -2033879827, PH.base.pack) ;
    p113_alt_SET((int32_t)116296180, PH.base.pack) ;
    p113_eph_SET((uint16_t)(uint16_t)30322, PH.base.pack) ;
    p113_epv_SET((uint16_t)(uint16_t)40451, PH.base.pack) ;
    p113_vel_SET((uint16_t)(uint16_t)29927, PH.base.pack) ;
    p113_vn_SET((int16_t)(int16_t) -30753, PH.base.pack) ;
    p113_ve_SET((int16_t)(int16_t)17813, PH.base.pack) ;
    p113_vd_SET((int16_t)(int16_t)30580, PH.base.pack) ;
    p113_cog_SET((uint16_t)(uint16_t)59213, PH.base.pack) ;
    p113_satellites_visible_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
    p114_time_usec_SET((uint64_t)6607690694469320422L, PH.base.pack) ;
    p114_sensor_id_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    p114_integration_time_us_SET((uint32_t)710791365L, PH.base.pack) ;
    p114_integrated_x_SET((float) -3.40553E37F, PH.base.pack) ;
    p114_integrated_y_SET((float)2.6983048E37F, PH.base.pack) ;
    p114_integrated_xgyro_SET((float) -2.341682E38F, PH.base.pack) ;
    p114_integrated_ygyro_SET((float) -1.4647303E38F, PH.base.pack) ;
    p114_integrated_zgyro_SET((float)1.7639798E38F, PH.base.pack) ;
    p114_temperature_SET((int16_t)(int16_t)22345, PH.base.pack) ;
    p114_quality_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
    p114_time_delta_distance_us_SET((uint32_t)2721515267L, PH.base.pack) ;
    p114_distance_SET((float) -1.0086552E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
    p115_time_usec_SET((uint64_t)1175564938579215801L, PH.base.pack) ;
    {
        float  attitude_quaternion [] =  {2.5262796E38F, 2.1700197E38F, 2.5680248E38F, 1.4317303E38F};
        p115_attitude_quaternion_SET(&attitude_quaternion, 0, &PH.base.pack) ;
    }
    p115_rollspeed_SET((float)3.8518646E37F, PH.base.pack) ;
    p115_pitchspeed_SET((float)3.3965264E38F, PH.base.pack) ;
    p115_yawspeed_SET((float) -3.2123982E38F, PH.base.pack) ;
    p115_lat_SET((int32_t)1147276526, PH.base.pack) ;
    p115_lon_SET((int32_t) -818294620, PH.base.pack) ;
    p115_alt_SET((int32_t)1601332960, PH.base.pack) ;
    p115_vx_SET((int16_t)(int16_t)28921, PH.base.pack) ;
    p115_vy_SET((int16_t)(int16_t)2108, PH.base.pack) ;
    p115_vz_SET((int16_t)(int16_t)1769, PH.base.pack) ;
    p115_ind_airspeed_SET((uint16_t)(uint16_t)42246, PH.base.pack) ;
    p115_true_airspeed_SET((uint16_t)(uint16_t)48306, PH.base.pack) ;
    p115_xacc_SET((int16_t)(int16_t) -3447, PH.base.pack) ;
    p115_yacc_SET((int16_t)(int16_t)30074, PH.base.pack) ;
    p115_zacc_SET((int16_t)(int16_t) -20291, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
    p116_time_boot_ms_SET((uint32_t)1920904872L, PH.base.pack) ;
    p116_xacc_SET((int16_t)(int16_t) -5201, PH.base.pack) ;
    p116_yacc_SET((int16_t)(int16_t) -5649, PH.base.pack) ;
    p116_zacc_SET((int16_t)(int16_t)22502, PH.base.pack) ;
    p116_xgyro_SET((int16_t)(int16_t)5573, PH.base.pack) ;
    p116_ygyro_SET((int16_t)(int16_t) -32549, PH.base.pack) ;
    p116_zgyro_SET((int16_t)(int16_t)9733, PH.base.pack) ;
    p116_xmag_SET((int16_t)(int16_t)16039, PH.base.pack) ;
    p116_ymag_SET((int16_t)(int16_t) -854, PH.base.pack) ;
    p116_zmag_SET((int16_t)(int16_t)445, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
    p117_target_system_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
    p117_target_component_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
    p117_start_SET((uint16_t)(uint16_t)1199, PH.base.pack) ;
    p117_end_SET((uint16_t)(uint16_t)45894, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
    p118_id_SET((uint16_t)(uint16_t)4841, PH.base.pack) ;
    p118_num_logs_SET((uint16_t)(uint16_t)19101, PH.base.pack) ;
    p118_last_log_num_SET((uint16_t)(uint16_t)37650, PH.base.pack) ;
    p118_time_utc_SET((uint32_t)164704149L, PH.base.pack) ;
    p118_size_SET((uint32_t)2707946473L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
    p119_target_system_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
    p119_target_component_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
    p119_id_SET((uint16_t)(uint16_t)60923, PH.base.pack) ;
    p119_ofs_SET((uint32_t)1961052055L, PH.base.pack) ;
    p119_count_SET((uint32_t)3335577732L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
    p120_id_SET((uint16_t)(uint16_t)17904, PH.base.pack) ;
    p120_ofs_SET((uint32_t)871951407L, PH.base.pack) ;
    p120_count_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)113, (uint8_t)168, (uint8_t)157, (uint8_t)232, (uint8_t)142, (uint8_t)191, (uint8_t)141, (uint8_t)117, (uint8_t)117, (uint8_t)104, (uint8_t)161, (uint8_t)152, (uint8_t)2, (uint8_t)86, (uint8_t)15, (uint8_t)103, (uint8_t)208, (uint8_t)22, (uint8_t)2, (uint8_t)84, (uint8_t)66, (uint8_t)145, (uint8_t)202, (uint8_t)221, (uint8_t)123, (uint8_t)114, (uint8_t)4, (uint8_t)51, (uint8_t)55, (uint8_t)114, (uint8_t)69, (uint8_t)144, (uint8_t)60, (uint8_t)203, (uint8_t)7, (uint8_t)68, (uint8_t)58, (uint8_t)139, (uint8_t)113, (uint8_t)179, (uint8_t)46, (uint8_t)17, (uint8_t)195, (uint8_t)199, (uint8_t)255, (uint8_t)73, (uint8_t)60, (uint8_t)83, (uint8_t)47, (uint8_t)175, (uint8_t)20, (uint8_t)194, (uint8_t)135, (uint8_t)205, (uint8_t)254, (uint8_t)40, (uint8_t)57, (uint8_t)16, (uint8_t)114, (uint8_t)158, (uint8_t)58, (uint8_t)43, (uint8_t)79, (uint8_t)206, (uint8_t)239, (uint8_t)205, (uint8_t)105, (uint8_t)167, (uint8_t)124, (uint8_t)39, (uint8_t)87, (uint8_t)58, (uint8_t)213, (uint8_t)66, (uint8_t)67, (uint8_t)95, (uint8_t)65, (uint8_t)134, (uint8_t)108, (uint8_t)191, (uint8_t)144, (uint8_t)134, (uint8_t)161, (uint8_t)15, (uint8_t)5, (uint8_t)218, (uint8_t)152, (uint8_t)208, (uint8_t)174, (uint8_t)166};
        p120_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
    p121_target_system_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
    p121_target_component_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
    p122_target_system_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
    p122_target_component_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
    p123_target_system_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
    p123_target_component_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
    p123_len_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)116, (uint8_t)15, (uint8_t)121, (uint8_t)69, (uint8_t)180, (uint8_t)114, (uint8_t)149, (uint8_t)65, (uint8_t)228, (uint8_t)59, (uint8_t)90, (uint8_t)85, (uint8_t)225, (uint8_t)16, (uint8_t)35, (uint8_t)144, (uint8_t)253, (uint8_t)29, (uint8_t)63, (uint8_t)154, (uint8_t)171, (uint8_t)115, (uint8_t)75, (uint8_t)181, (uint8_t)12, (uint8_t)2, (uint8_t)39, (uint8_t)226, (uint8_t)14, (uint8_t)10, (uint8_t)54, (uint8_t)19, (uint8_t)57, (uint8_t)78, (uint8_t)79, (uint8_t)156, (uint8_t)219, (uint8_t)203, (uint8_t)215, (uint8_t)193, (uint8_t)210, (uint8_t)143, (uint8_t)180, (uint8_t)222, (uint8_t)254, (uint8_t)250, (uint8_t)231, (uint8_t)193, (uint8_t)101, (uint8_t)182, (uint8_t)105, (uint8_t)186, (uint8_t)42, (uint8_t)212, (uint8_t)119, (uint8_t)31, (uint8_t)58, (uint8_t)36, (uint8_t)202, (uint8_t)240, (uint8_t)188, (uint8_t)88, (uint8_t)227, (uint8_t)104, (uint8_t)251, (uint8_t)117, (uint8_t)97, (uint8_t)130, (uint8_t)20, (uint8_t)231, (uint8_t)105, (uint8_t)103, (uint8_t)51, (uint8_t)108, (uint8_t)106, (uint8_t)95, (uint8_t)79, (uint8_t)23, (uint8_t)143, (uint8_t)225, (uint8_t)30, (uint8_t)194, (uint8_t)0, (uint8_t)73, (uint8_t)183, (uint8_t)116, (uint8_t)10, (uint8_t)250, (uint8_t)130, (uint8_t)247, (uint8_t)89, (uint8_t)15, (uint8_t)144, (uint8_t)209, (uint8_t)34, (uint8_t)149, (uint8_t)2, (uint8_t)247, (uint8_t)92, (uint8_t)69, (uint8_t)209, (uint8_t)79, (uint8_t)208, (uint8_t)67, (uint8_t)241, (uint8_t)60, (uint8_t)250, (uint8_t)150, (uint8_t)213, (uint8_t)27};
        p123_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
    p124_time_usec_SET((uint64_t)1442750071716925880L, PH.base.pack) ;
    p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, PH.base.pack) ;
    p124_lat_SET((int32_t)97780579, PH.base.pack) ;
    p124_lon_SET((int32_t) -1343999963, PH.base.pack) ;
    p124_alt_SET((int32_t)1609506570, PH.base.pack) ;
    p124_eph_SET((uint16_t)(uint16_t)18532, PH.base.pack) ;
    p124_epv_SET((uint16_t)(uint16_t)17080, PH.base.pack) ;
    p124_vel_SET((uint16_t)(uint16_t)24174, PH.base.pack) ;
    p124_cog_SET((uint16_t)(uint16_t)25982, PH.base.pack) ;
    p124_satellites_visible_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
    p124_dgps_numch_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
    p124_dgps_age_SET((uint32_t)2088157406L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
    p125_Vcc_SET((uint16_t)(uint16_t)13035, PH.base.pack) ;
    p125_Vservo_SET((uint16_t)(uint16_t)39972, PH.base.pack) ;
    p125_flags_SET((e_MAV_POWER_STATUS_MAV_POWER_STATUS_BRICK_VALID |
                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT), PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
    p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL, PH.base.pack) ;
    p126_flags_SET((e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY |
                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING |
                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI |
                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE), PH.base.pack) ;
    p126_timeout_SET((uint16_t)(uint16_t)37554, PH.base.pack) ;
    p126_baudrate_SET((uint32_t)196963548L, PH.base.pack) ;
    p126_count_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)239, (uint8_t)202, (uint8_t)74, (uint8_t)250, (uint8_t)170, (uint8_t)233, (uint8_t)198, (uint8_t)174, (uint8_t)221, (uint8_t)151, (uint8_t)199, (uint8_t)241, (uint8_t)7, (uint8_t)131, (uint8_t)146, (uint8_t)157, (uint8_t)106, (uint8_t)193, (uint8_t)121, (uint8_t)165, (uint8_t)140, (uint8_t)197, (uint8_t)16, (uint8_t)102, (uint8_t)135, (uint8_t)103, (uint8_t)204, (uint8_t)88, (uint8_t)254, (uint8_t)253, (uint8_t)127, (uint8_t)236, (uint8_t)72, (uint8_t)160, (uint8_t)90, (uint8_t)116, (uint8_t)171, (uint8_t)214, (uint8_t)139, (uint8_t)82, (uint8_t)96, (uint8_t)194, (uint8_t)216, (uint8_t)55, (uint8_t)90, (uint8_t)183, (uint8_t)92, (uint8_t)49, (uint8_t)113, (uint8_t)136, (uint8_t)209, (uint8_t)79, (uint8_t)124, (uint8_t)55, (uint8_t)21, (uint8_t)244, (uint8_t)222, (uint8_t)238, (uint8_t)42, (uint8_t)167, (uint8_t)84, (uint8_t)242, (uint8_t)195, (uint8_t)118, (uint8_t)102, (uint8_t)193, (uint8_t)3, (uint8_t)180, (uint8_t)148, (uint8_t)28};
        p126_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
    p127_time_last_baseline_ms_SET((uint32_t)405155441L, PH.base.pack) ;
    p127_rtk_receiver_id_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
    p127_wn_SET((uint16_t)(uint16_t)56541, PH.base.pack) ;
    p127_tow_SET((uint32_t)2317679937L, PH.base.pack) ;
    p127_rtk_health_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
    p127_rtk_rate_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
    p127_nsats_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
    p127_baseline_coords_type_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
    p127_baseline_a_mm_SET((int32_t) -1458750008, PH.base.pack) ;
    p127_baseline_b_mm_SET((int32_t) -123953256, PH.base.pack) ;
    p127_baseline_c_mm_SET((int32_t)3137812, PH.base.pack) ;
    p127_accuracy_SET((uint32_t)3351746025L, PH.base.pack) ;
    p127_iar_num_hypotheses_SET((int32_t) -762325797, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
    p128_time_last_baseline_ms_SET((uint32_t)3417576374L, PH.base.pack) ;
    p128_rtk_receiver_id_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
    p128_wn_SET((uint16_t)(uint16_t)20307, PH.base.pack) ;
    p128_tow_SET((uint32_t)565457068L, PH.base.pack) ;
    p128_rtk_health_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
    p128_rtk_rate_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
    p128_nsats_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
    p128_baseline_coords_type_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
    p128_baseline_a_mm_SET((int32_t)201424783, PH.base.pack) ;
    p128_baseline_b_mm_SET((int32_t)153946380, PH.base.pack) ;
    p128_baseline_c_mm_SET((int32_t) -898146372, PH.base.pack) ;
    p128_accuracy_SET((uint32_t)418011990L, PH.base.pack) ;
    p128_iar_num_hypotheses_SET((int32_t)747483615, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
    p129_time_boot_ms_SET((uint32_t)4274998056L, PH.base.pack) ;
    p129_xacc_SET((int16_t)(int16_t)16255, PH.base.pack) ;
    p129_yacc_SET((int16_t)(int16_t) -11476, PH.base.pack) ;
    p129_zacc_SET((int16_t)(int16_t) -20195, PH.base.pack) ;
    p129_xgyro_SET((int16_t)(int16_t) -27874, PH.base.pack) ;
    p129_ygyro_SET((int16_t)(int16_t)13107, PH.base.pack) ;
    p129_zgyro_SET((int16_t)(int16_t) -24375, PH.base.pack) ;
    p129_xmag_SET((int16_t)(int16_t)11236, PH.base.pack) ;
    p129_ymag_SET((int16_t)(int16_t)29749, PH.base.pack) ;
    p129_zmag_SET((int16_t)(int16_t)121, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
    p130_type_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
    p130_size_SET((uint32_t)1547899119L, PH.base.pack) ;
    p130_width_SET((uint16_t)(uint16_t)21229, PH.base.pack) ;
    p130_height_SET((uint16_t)(uint16_t)56179, PH.base.pack) ;
    p130_packets_SET((uint16_t)(uint16_t)63683, PH.base.pack) ;
    p130_payload_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
    p130_jpg_quality_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
    p131_seqnr_SET((uint16_t)(uint16_t)5888, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)71, (uint8_t)24, (uint8_t)195, (uint8_t)196, (uint8_t)17, (uint8_t)135, (uint8_t)136, (uint8_t)179, (uint8_t)117, (uint8_t)12, (uint8_t)23, (uint8_t)13, (uint8_t)189, (uint8_t)199, (uint8_t)135, (uint8_t)229, (uint8_t)34, (uint8_t)173, (uint8_t)108, (uint8_t)83, (uint8_t)104, (uint8_t)63, (uint8_t)244, (uint8_t)176, (uint8_t)172, (uint8_t)4, (uint8_t)36, (uint8_t)138, (uint8_t)150, (uint8_t)43, (uint8_t)135, (uint8_t)242, (uint8_t)187, (uint8_t)26, (uint8_t)237, (uint8_t)164, (uint8_t)152, (uint8_t)205, (uint8_t)207, (uint8_t)117, (uint8_t)103, (uint8_t)158, (uint8_t)12, (uint8_t)232, (uint8_t)18, (uint8_t)231, (uint8_t)244, (uint8_t)246, (uint8_t)113, (uint8_t)46, (uint8_t)25, (uint8_t)35, (uint8_t)198, (uint8_t)232, (uint8_t)107, (uint8_t)217, (uint8_t)68, (uint8_t)9, (uint8_t)63, (uint8_t)205, (uint8_t)55, (uint8_t)160, (uint8_t)91, (uint8_t)49, (uint8_t)132, (uint8_t)142, (uint8_t)193, (uint8_t)35, (uint8_t)191, (uint8_t)125, (uint8_t)101, (uint8_t)8, (uint8_t)183, (uint8_t)37, (uint8_t)246, (uint8_t)62, (uint8_t)242, (uint8_t)59, (uint8_t)152, (uint8_t)231, (uint8_t)172, (uint8_t)5, (uint8_t)80, (uint8_t)20, (uint8_t)125, (uint8_t)227, (uint8_t)190, (uint8_t)27, (uint8_t)207, (uint8_t)35, (uint8_t)206, (uint8_t)166, (uint8_t)246, (uint8_t)64, (uint8_t)68, (uint8_t)248, (uint8_t)2, (uint8_t)8, (uint8_t)199, (uint8_t)66, (uint8_t)102, (uint8_t)47, (uint8_t)13, (uint8_t)9, (uint8_t)218, (uint8_t)212, (uint8_t)121, (uint8_t)49, (uint8_t)198, (uint8_t)39, (uint8_t)118, (uint8_t)251, (uint8_t)255, (uint8_t)238, (uint8_t)79, (uint8_t)40, (uint8_t)253, (uint8_t)46, (uint8_t)241, (uint8_t)73, (uint8_t)190, (uint8_t)168, (uint8_t)166, (uint8_t)107, (uint8_t)174, (uint8_t)110, (uint8_t)225, (uint8_t)130, (uint8_t)216, (uint8_t)76, (uint8_t)237, (uint8_t)32, (uint8_t)251, (uint8_t)59, (uint8_t)97, (uint8_t)76, (uint8_t)198, (uint8_t)100, (uint8_t)2, (uint8_t)80, (uint8_t)76, (uint8_t)150, (uint8_t)127, (uint8_t)128, (uint8_t)224, (uint8_t)10, (uint8_t)57, (uint8_t)204, (uint8_t)194, (uint8_t)185, (uint8_t)129, (uint8_t)222, (uint8_t)13, (uint8_t)69, (uint8_t)66, (uint8_t)167, (uint8_t)161, (uint8_t)18, (uint8_t)30, (uint8_t)51, (uint8_t)44, (uint8_t)208, (uint8_t)43, (uint8_t)4, (uint8_t)146, (uint8_t)215, (uint8_t)122, (uint8_t)206, (uint8_t)13, (uint8_t)61, (uint8_t)80, (uint8_t)242, (uint8_t)27, (uint8_t)224, (uint8_t)71, (uint8_t)171, (uint8_t)181, (uint8_t)82, (uint8_t)23, (uint8_t)163, (uint8_t)61, (uint8_t)103, (uint8_t)227, (uint8_t)151, (uint8_t)100, (uint8_t)235, (uint8_t)14, (uint8_t)135, (uint8_t)174, (uint8_t)255, (uint8_t)4, (uint8_t)243, (uint8_t)65, (uint8_t)91, (uint8_t)63, (uint8_t)60, (uint8_t)84, (uint8_t)204, (uint8_t)216, (uint8_t)194, (uint8_t)123, (uint8_t)251, (uint8_t)2, (uint8_t)58, (uint8_t)247, (uint8_t)164, (uint8_t)131, (uint8_t)243, (uint8_t)136, (uint8_t)204, (uint8_t)85, (uint8_t)4, (uint8_t)208, (uint8_t)134, (uint8_t)21, (uint8_t)255, (uint8_t)18, (uint8_t)110, (uint8_t)90, (uint8_t)110, (uint8_t)242, (uint8_t)114, (uint8_t)218, (uint8_t)126, (uint8_t)241, (uint8_t)127, (uint8_t)3, (uint8_t)136, (uint8_t)66, (uint8_t)240, (uint8_t)9, (uint8_t)234, (uint8_t)183, (uint8_t)78, (uint8_t)149, (uint8_t)109, (uint8_t)186, (uint8_t)93, (uint8_t)95, (uint8_t)160, (uint8_t)72, (uint8_t)111, (uint8_t)109, (uint8_t)27, (uint8_t)109, (uint8_t)47, (uint8_t)224, (uint8_t)233, (uint8_t)41, (uint8_t)53, (uint8_t)189, (uint8_t)26, (uint8_t)7};
        p131_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
    p132_time_boot_ms_SET((uint32_t)2414803378L, PH.base.pack) ;
    p132_min_distance_SET((uint16_t)(uint16_t)58899, PH.base.pack) ;
    p132_max_distance_SET((uint16_t)(uint16_t)42358, PH.base.pack) ;
    p132_current_distance_SET((uint16_t)(uint16_t)21251, PH.base.pack) ;
    p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, PH.base.pack) ;
    p132_id_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
    p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_90, PH.base.pack) ;
    p132_covariance_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
    p133_lat_SET((int32_t) -1025277805, PH.base.pack) ;
    p133_lon_SET((int32_t)1216652787, PH.base.pack) ;
    p133_grid_spacing_SET((uint16_t)(uint16_t)21602, PH.base.pack) ;
    p133_mask_SET((uint64_t)9178034741354313592L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
    p134_lat_SET((int32_t)676169873, PH.base.pack) ;
    p134_lon_SET((int32_t) -1936333270, PH.base.pack) ;
    p134_grid_spacing_SET((uint16_t)(uint16_t)42450, PH.base.pack) ;
    p134_gridbit_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
    {
        int16_t  data_ [] =  {(int16_t) -23481, (int16_t)29540, (int16_t)2300, (int16_t) -8808, (int16_t)22863, (int16_t) -20901, (int16_t)28138, (int16_t)11445, (int16_t)8739, (int16_t)20475, (int16_t) -6885, (int16_t)14573, (int16_t) -12958, (int16_t) -28596, (int16_t)21803, (int16_t) -4477};
        p134_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
    p135_lat_SET((int32_t) -1249332159, PH.base.pack) ;
    p135_lon_SET((int32_t) -378683865, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
    p136_lat_SET((int32_t) -1674812232, PH.base.pack) ;
    p136_lon_SET((int32_t)1579442386, PH.base.pack) ;
    p136_spacing_SET((uint16_t)(uint16_t)4409, PH.base.pack) ;
    p136_terrain_height_SET((float)2.4356322E38F, PH.base.pack) ;
    p136_current_height_SET((float) -2.1684774E38F, PH.base.pack) ;
    p136_pending_SET((uint16_t)(uint16_t)41921, PH.base.pack) ;
    p136_loaded_SET((uint16_t)(uint16_t)21761, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
    p137_time_boot_ms_SET((uint32_t)1712481471L, PH.base.pack) ;
    p137_press_abs_SET((float) -2.9117358E38F, PH.base.pack) ;
    p137_press_diff_SET((float) -3.2679302E37F, PH.base.pack) ;
    p137_temperature_SET((int16_t)(int16_t)2074, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
    p138_time_usec_SET((uint64_t)5366265203698864383L, PH.base.pack) ;
    {
        float  q [] =  {4.355217E37F, 2.833944E38F, -3.2053614E38F, 1.7888006E38F};
        p138_q_SET(&q, 0, &PH.base.pack) ;
    }
    p138_x_SET((float)7.3660053E37F, PH.base.pack) ;
    p138_y_SET((float) -4.1579814E37F, PH.base.pack) ;
    p138_z_SET((float) -1.4274166E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
    p139_time_usec_SET((uint64_t)4691424108379411565L, PH.base.pack) ;
    p139_group_mlx_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
    p139_target_system_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
    p139_target_component_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
    {
        float  controls [] =  {-3.326934E38F, -2.8822653E37F, 3.131917E38F, -2.9662209E38F, -1.776087E38F, 2.401045E38F, -3.0129304E37F, -2.6602776E38F};
        p139_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
    p140_time_usec_SET((uint64_t)1675602005250934308L, PH.base.pack) ;
    p140_group_mlx_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
    {
        float  controls [] =  {-3.323674E38F, 1.0828264E38F, -2.2766603E38F, -2.4064418E38F, 3.6470906E37F, 6.6181077E37F, 1.7989116E38F, -1.4852803E37F};
        p140_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ALTITUDE_141(), &PH);
    p141_time_usec_SET((uint64_t)2303585232686531680L, PH.base.pack) ;
    p141_altitude_monotonic_SET((float)1.0660114E38F, PH.base.pack) ;
    p141_altitude_amsl_SET((float) -6.9432093E37F, PH.base.pack) ;
    p141_altitude_local_SET((float)2.1763973E38F, PH.base.pack) ;
    p141_altitude_relative_SET((float) -8.775851E37F, PH.base.pack) ;
    p141_altitude_terrain_SET((float) -2.5493498E38F, PH.base.pack) ;
    p141_bottom_clearance_SET((float)2.3039162E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RESOURCE_REQUEST_142(), &PH);
    p142_request_id_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
    p142_uri_type_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    {
        uint8_t  uri [] =  {(uint8_t)90, (uint8_t)190, (uint8_t)230, (uint8_t)35, (uint8_t)192, (uint8_t)134, (uint8_t)32, (uint8_t)203, (uint8_t)181, (uint8_t)10, (uint8_t)18, (uint8_t)162, (uint8_t)155, (uint8_t)41, (uint8_t)237, (uint8_t)252, (uint8_t)231, (uint8_t)69, (uint8_t)101, (uint8_t)186, (uint8_t)154, (uint8_t)47, (uint8_t)0, (uint8_t)196, (uint8_t)228, (uint8_t)56, (uint8_t)6, (uint8_t)231, (uint8_t)51, (uint8_t)59, (uint8_t)114, (uint8_t)217, (uint8_t)147, (uint8_t)166, (uint8_t)199, (uint8_t)59, (uint8_t)178, (uint8_t)50, (uint8_t)32, (uint8_t)124, (uint8_t)114, (uint8_t)83, (uint8_t)17, (uint8_t)57, (uint8_t)171, (uint8_t)212, (uint8_t)243, (uint8_t)98, (uint8_t)132, (uint8_t)168, (uint8_t)231, (uint8_t)191, (uint8_t)62, (uint8_t)211, (uint8_t)151, (uint8_t)135, (uint8_t)65, (uint8_t)238, (uint8_t)245, (uint8_t)204, (uint8_t)10, (uint8_t)90, (uint8_t)89, (uint8_t)47, (uint8_t)166, (uint8_t)168, (uint8_t)249, (uint8_t)21, (uint8_t)224, (uint8_t)189, (uint8_t)26, (uint8_t)29, (uint8_t)234, (uint8_t)106, (uint8_t)0, (uint8_t)39, (uint8_t)177, (uint8_t)71, (uint8_t)242, (uint8_t)99, (uint8_t)127, (uint8_t)200, (uint8_t)240, (uint8_t)120, (uint8_t)217, (uint8_t)122, (uint8_t)213, (uint8_t)166, (uint8_t)131, (uint8_t)216, (uint8_t)230, (uint8_t)71, (uint8_t)182, (uint8_t)206, (uint8_t)199, (uint8_t)87, (uint8_t)40, (uint8_t)2, (uint8_t)240, (uint8_t)168, (uint8_t)150, (uint8_t)246, (uint8_t)110, (uint8_t)174, (uint8_t)3, (uint8_t)57, (uint8_t)21, (uint8_t)131, (uint8_t)173, (uint8_t)222, (uint8_t)119, (uint8_t)171, (uint8_t)163, (uint8_t)74, (uint8_t)51, (uint8_t)143, (uint8_t)158, (uint8_t)156, (uint8_t)226, (uint8_t)29};
        p142_uri_SET(&uri, 0, &PH.base.pack) ;
    }
    p142_transfer_type_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
    {
        uint8_t  storage [] =  {(uint8_t)153, (uint8_t)110, (uint8_t)208, (uint8_t)134, (uint8_t)151, (uint8_t)176, (uint8_t)243, (uint8_t)95, (uint8_t)212, (uint8_t)107, (uint8_t)67, (uint8_t)241, (uint8_t)7, (uint8_t)241, (uint8_t)98, (uint8_t)37, (uint8_t)173, (uint8_t)250, (uint8_t)221, (uint8_t)158, (uint8_t)250, (uint8_t)223, (uint8_t)255, (uint8_t)151, (uint8_t)29, (uint8_t)171, (uint8_t)241, (uint8_t)170, (uint8_t)244, (uint8_t)115, (uint8_t)217, (uint8_t)172, (uint8_t)39, (uint8_t)51, (uint8_t)224, (uint8_t)7, (uint8_t)16, (uint8_t)228, (uint8_t)252, (uint8_t)59, (uint8_t)221, (uint8_t)90, (uint8_t)254, (uint8_t)72, (uint8_t)206, (uint8_t)99, (uint8_t)172, (uint8_t)147, (uint8_t)50, (uint8_t)230, (uint8_t)9, (uint8_t)223, (uint8_t)204, (uint8_t)24, (uint8_t)78, (uint8_t)29, (uint8_t)109, (uint8_t)8, (uint8_t)56, (uint8_t)117, (uint8_t)1, (uint8_t)226, (uint8_t)28, (uint8_t)117, (uint8_t)48, (uint8_t)34, (uint8_t)103, (uint8_t)193, (uint8_t)126, (uint8_t)236, (uint8_t)68, (uint8_t)186, (uint8_t)196, (uint8_t)128, (uint8_t)155, (uint8_t)42, (uint8_t)44, (uint8_t)239, (uint8_t)151, (uint8_t)143, (uint8_t)243, (uint8_t)200, (uint8_t)190, (uint8_t)236, (uint8_t)36, (uint8_t)167, (uint8_t)235, (uint8_t)69, (uint8_t)199, (uint8_t)49, (uint8_t)103, (uint8_t)236, (uint8_t)248, (uint8_t)184, (uint8_t)140, (uint8_t)109, (uint8_t)243, (uint8_t)205, (uint8_t)31, (uint8_t)90, (uint8_t)5, (uint8_t)120, (uint8_t)67, (uint8_t)89, (uint8_t)241, (uint8_t)21, (uint8_t)229, (uint8_t)175, (uint8_t)34, (uint8_t)208, (uint8_t)68, (uint8_t)139, (uint8_t)165, (uint8_t)75, (uint8_t)111, (uint8_t)111, (uint8_t)41, (uint8_t)201, (uint8_t)201, (uint8_t)167};
        p142_storage_SET(&storage, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE3_143(), &PH);
    p143_time_boot_ms_SET((uint32_t)2419363546L, PH.base.pack) ;
    p143_press_abs_SET((float) -2.1847936E38F, PH.base.pack) ;
    p143_press_diff_SET((float)1.6494393E38F, PH.base.pack) ;
    p143_temperature_SET((int16_t)(int16_t)23973, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_FOLLOW_TARGET_144(), &PH);
    p144_timestamp_SET((uint64_t)6970285557533599979L, PH.base.pack) ;
    p144_est_capabilities_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
    p144_lat_SET((int32_t) -1736850145, PH.base.pack) ;
    p144_lon_SET((int32_t)90672288, PH.base.pack) ;
    p144_alt_SET((float)5.989022E36F, PH.base.pack) ;
    {
        float  vel [] =  {2.8890684E38F, 2.9196934E38F, 4.8325047E37F};
        p144_vel_SET(&vel, 0, &PH.base.pack) ;
    }
    {
        float  acc [] =  {-1.7973547E38F, 1.6992219E38F, 2.1728368E37F};
        p144_acc_SET(&acc, 0, &PH.base.pack) ;
    }
    {
        float  attitude_q [] =  {-5.570558E37F, 5.956585E37F, 2.3762435E38F, 1.4483644E38F};
        p144_attitude_q_SET(&attitude_q, 0, &PH.base.pack) ;
    }
    {
        float  rates [] =  {-1.1269603E38F, 1.3362695E38F, -2.6362058E38F};
        p144_rates_SET(&rates, 0, &PH.base.pack) ;
    }
    {
        float  position_cov [] =  {1.6720585E38F, 1.1755325E38F, -2.9341155E37F};
        p144_position_cov_SET(&position_cov, 0, &PH.base.pack) ;
    }
    p144_custom_state_SET((uint64_t)5499357501751455640L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
    p146_time_usec_SET((uint64_t)5592152252462846662L, PH.base.pack) ;
    p146_x_acc_SET((float)2.2189587E38F, PH.base.pack) ;
    p146_y_acc_SET((float) -1.3062085E37F, PH.base.pack) ;
    p146_z_acc_SET((float)1.1997662E38F, PH.base.pack) ;
    p146_x_vel_SET((float)2.826548E38F, PH.base.pack) ;
    p146_y_vel_SET((float) -2.6357809E38F, PH.base.pack) ;
    p146_z_vel_SET((float) -1.8960696E38F, PH.base.pack) ;
    p146_x_pos_SET((float)6.241673E37F, PH.base.pack) ;
    p146_y_pos_SET((float)8.993433E37F, PH.base.pack) ;
    p146_z_pos_SET((float)4.1766243E37F, PH.base.pack) ;
    p146_airspeed_SET((float) -1.0558314E38F, PH.base.pack) ;
    {
        float  vel_variance [] =  {-6.9337755E37F, 2.3281391E38F, -2.9036446E38F};
        p146_vel_variance_SET(&vel_variance, 0, &PH.base.pack) ;
    }
    {
        float  pos_variance [] =  {-2.2665946E38F, -6.9825886E37F, -7.696953E37F};
        p146_pos_variance_SET(&pos_variance, 0, &PH.base.pack) ;
    }
    {
        float  q [] =  {1.8536555E38F, -1.0950398E38F, 4.723596E37F, 1.1910996E38F};
        p146_q_SET(&q, 0, &PH.base.pack) ;
    }
    p146_roll_rate_SET((float) -1.2065784E38F, PH.base.pack) ;
    p146_pitch_rate_SET((float)1.708068E38F, PH.base.pack) ;
    p146_yaw_rate_SET((float)4.823937E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_BATTERY_STATUS_147(), &PH);
    p147_id_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
    p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_UNKNOWN, PH.base.pack) ;
    p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIFE, PH.base.pack) ;
    p147_temperature_SET((int16_t)(int16_t) -27750, PH.base.pack) ;
    {
        uint16_t  voltages [] =  {(uint16_t)58185, (uint16_t)645, (uint16_t)33642, (uint16_t)1469, (uint16_t)9272, (uint16_t)40884, (uint16_t)51377, (uint16_t)37867, (uint16_t)41749, (uint16_t)3498};
        p147_voltages_SET(&voltages, 0, &PH.base.pack) ;
    }
    p147_current_battery_SET((int16_t)(int16_t)10427, PH.base.pack) ;
    p147_current_consumed_SET((int32_t)412912343, PH.base.pack) ;
    p147_energy_consumed_SET((int32_t)543529564, PH.base.pack) ;
    p147_battery_remaining_SET((int8_t)(int8_t) -44, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_AUTOPILOT_VERSION_148(), &PH);
    p148_capabilities_SET((e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION |
                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_FTP |
                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_TERRAIN |
                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_RALLY |
                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
                           e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION), PH.base.pack) ;
    p148_flight_sw_version_SET((uint32_t)952103054L, PH.base.pack) ;
    p148_middleware_sw_version_SET((uint32_t)3095085606L, PH.base.pack) ;
    p148_os_sw_version_SET((uint32_t)1978113574L, PH.base.pack) ;
    p148_board_version_SET((uint32_t)1528142462L, PH.base.pack) ;
    {
        uint8_t  flight_custom_version [] =  {(uint8_t)196, (uint8_t)186, (uint8_t)60, (uint8_t)236, (uint8_t)195, (uint8_t)245, (uint8_t)145, (uint8_t)38};
        p148_flight_custom_version_SET(&flight_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  middleware_custom_version [] =  {(uint8_t)193, (uint8_t)78, (uint8_t)249, (uint8_t)13, (uint8_t)194, (uint8_t)218, (uint8_t)20, (uint8_t)123};
        p148_middleware_custom_version_SET(&middleware_custom_version, 0, &PH.base.pack) ;
    }
    {
        uint8_t  os_custom_version [] =  {(uint8_t)230, (uint8_t)77, (uint8_t)208, (uint8_t)5, (uint8_t)0, (uint8_t)154, (uint8_t)223, (uint8_t)123};
        p148_os_custom_version_SET(&os_custom_version, 0, &PH.base.pack) ;
    }
    p148_vendor_id_SET((uint16_t)(uint16_t)41281, PH.base.pack) ;
    p148_product_id_SET((uint16_t)(uint16_t)18714, PH.base.pack) ;
    p148_uid_SET((uint64_t)6237136068716021264L, PH.base.pack) ;
    {
        uint8_t  uid2 [] =  {(uint8_t)78, (uint8_t)195, (uint8_t)141, (uint8_t)212, (uint8_t)2, (uint8_t)197, (uint8_t)146, (uint8_t)115, (uint8_t)204, (uint8_t)201, (uint8_t)42, (uint8_t)200, (uint8_t)171, (uint8_t)48, (uint8_t)48, (uint8_t)247, (uint8_t)225, (uint8_t)121};
        p148_uid2_SET(&uid2, 0,  &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LANDING_TARGET_149(), &PH);
    p149_time_usec_SET((uint64_t)8854137567692628785L, PH.base.pack) ;
    p149_target_num_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
    p149_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
    p149_angle_x_SET((float)4.820701E37F, PH.base.pack) ;
    p149_angle_y_SET((float) -5.7993636E37F, PH.base.pack) ;
    p149_distance_SET((float)5.792803E37F, PH.base.pack) ;
    p149_size_x_SET((float) -4.94256E37F, PH.base.pack) ;
    p149_size_y_SET((float) -2.98158E38F, PH.base.pack) ;
    p149_x_SET((float) -2.442799E38F, &PH) ;
    p149_y_SET((float)3.1793407E38F, &PH) ;
    p149_z_SET((float)8.545735E37F, &PH) ;
    {
        float  q [] =  {1.7065778E37F, -1.4958167E38F, -6.455168E37F, -2.6390352E38F};
        p149_q_SET(&q, 0,  &PH) ;
    }
    p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON, PH.base.pack) ;
    p149_position_valid_SET((uint8_t)(uint8_t)187, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ESTIMATOR_STATUS_230(), &PH);
    p230_time_usec_SET((uint64_t)9004588379304744223L, PH.base.pack) ;
    p230_flags_SET((e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE |
                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL |
                    e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS), PH.base.pack) ;
    p230_vel_ratio_SET((float)1.6077008E38F, PH.base.pack) ;
    p230_pos_horiz_ratio_SET((float)9.819922E37F, PH.base.pack) ;
    p230_pos_vert_ratio_SET((float) -1.0779392E38F, PH.base.pack) ;
    p230_mag_ratio_SET((float) -2.3423672E38F, PH.base.pack) ;
    p230_hagl_ratio_SET((float)7.9717283E37F, PH.base.pack) ;
    p230_tas_ratio_SET((float) -8.454725E37F, PH.base.pack) ;
    p230_pos_horiz_accuracy_SET((float)1.0685553E37F, PH.base.pack) ;
    p230_pos_vert_accuracy_SET((float) -1.3563689E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_WIND_COV_231(), &PH);
    p231_time_usec_SET((uint64_t)683967046655633025L, PH.base.pack) ;
    p231_wind_x_SET((float) -6.905059E36F, PH.base.pack) ;
    p231_wind_y_SET((float) -1.8261683E38F, PH.base.pack) ;
    p231_wind_z_SET((float) -1.8894742E38F, PH.base.pack) ;
    p231_var_horiz_SET((float) -3.3378518E38F, PH.base.pack) ;
    p231_var_vert_SET((float)2.8214231E38F, PH.base.pack) ;
    p231_wind_alt_SET((float) -1.524965E38F, PH.base.pack) ;
    p231_horiz_accuracy_SET((float) -3.7603952E37F, PH.base.pack) ;
    p231_vert_accuracy_SET((float) -2.000714E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_INPUT_232(), &PH);
    p232_time_usec_SET((uint64_t)6998904390872050501L, PH.base.pack) ;
    p232_gps_id_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
    p232_ignore_flags_SET((e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY |
                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VDOP |
                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                           e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP), PH.base.pack) ;
    p232_time_week_ms_SET((uint32_t)528766996L, PH.base.pack) ;
    p232_time_week_SET((uint16_t)(uint16_t)48322, PH.base.pack) ;
    p232_fix_type_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
    p232_lat_SET((int32_t) -575350080, PH.base.pack) ;
    p232_lon_SET((int32_t) -1553062892, PH.base.pack) ;
    p232_alt_SET((float) -7.0870446E37F, PH.base.pack) ;
    p232_hdop_SET((float) -2.1284224E37F, PH.base.pack) ;
    p232_vdop_SET((float) -7.439378E37F, PH.base.pack) ;
    p232_vn_SET((float)2.1524226E38F, PH.base.pack) ;
    p232_ve_SET((float) -2.2475985E38F, PH.base.pack) ;
    p232_vd_SET((float)2.9733363E38F, PH.base.pack) ;
    p232_speed_accuracy_SET((float) -5.5192807E37F, PH.base.pack) ;
    p232_horiz_accuracy_SET((float) -2.8788826E38F, PH.base.pack) ;
    p232_vert_accuracy_SET((float)1.6462188E38F, PH.base.pack) ;
    p232_satellites_visible_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RTCM_DATA_233(), &PH);
    p233_flags_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
    p233_len_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)81, (uint8_t)42, (uint8_t)116, (uint8_t)5, (uint8_t)28, (uint8_t)47, (uint8_t)34, (uint8_t)57, (uint8_t)109, (uint8_t)152, (uint8_t)46, (uint8_t)169, (uint8_t)118, (uint8_t)241, (uint8_t)180, (uint8_t)10, (uint8_t)144, (uint8_t)142, (uint8_t)228, (uint8_t)9, (uint8_t)123, (uint8_t)4, (uint8_t)248, (uint8_t)83, (uint8_t)157, (uint8_t)105, (uint8_t)45, (uint8_t)226, (uint8_t)127, (uint8_t)158, (uint8_t)130, (uint8_t)19, (uint8_t)107, (uint8_t)46, (uint8_t)127, (uint8_t)44, (uint8_t)35, (uint8_t)193, (uint8_t)59, (uint8_t)17, (uint8_t)132, (uint8_t)243, (uint8_t)15, (uint8_t)224, (uint8_t)152, (uint8_t)249, (uint8_t)80, (uint8_t)8, (uint8_t)147, (uint8_t)36, (uint8_t)42, (uint8_t)5, (uint8_t)139, (uint8_t)139, (uint8_t)149, (uint8_t)137, (uint8_t)132, (uint8_t)159, (uint8_t)43, (uint8_t)92, (uint8_t)214, (uint8_t)23, (uint8_t)105, (uint8_t)20, (uint8_t)71, (uint8_t)109, (uint8_t)214, (uint8_t)15, (uint8_t)102, (uint8_t)183, (uint8_t)233, (uint8_t)56, (uint8_t)244, (uint8_t)17, (uint8_t)61, (uint8_t)126, (uint8_t)193, (uint8_t)137, (uint8_t)216, (uint8_t)41, (uint8_t)8, (uint8_t)177, (uint8_t)37, (uint8_t)53, (uint8_t)49, (uint8_t)154, (uint8_t)171, (uint8_t)129, (uint8_t)166, (uint8_t)23, (uint8_t)62, (uint8_t)100, (uint8_t)191, (uint8_t)240, (uint8_t)237, (uint8_t)9, (uint8_t)209, (uint8_t)18, (uint8_t)12, (uint8_t)60, (uint8_t)47, (uint8_t)247, (uint8_t)112, (uint8_t)103, (uint8_t)53, (uint8_t)160, (uint8_t)51, (uint8_t)198, (uint8_t)130, (uint8_t)76, (uint8_t)85, (uint8_t)204, (uint8_t)184, (uint8_t)106, (uint8_t)206, (uint8_t)227, (uint8_t)191, (uint8_t)234, (uint8_t)182, (uint8_t)50, (uint8_t)108, (uint8_t)208, (uint8_t)125, (uint8_t)250, (uint8_t)103, (uint8_t)30, (uint8_t)32, (uint8_t)74, (uint8_t)149, (uint8_t)207, (uint8_t)222, (uint8_t)49, (uint8_t)175, (uint8_t)53, (uint8_t)105, (uint8_t)141, (uint8_t)218, (uint8_t)230, (uint8_t)145, (uint8_t)144, (uint8_t)181, (uint8_t)36, (uint8_t)243, (uint8_t)109, (uint8_t)114, (uint8_t)112, (uint8_t)134, (uint8_t)43, (uint8_t)200, (uint8_t)118, (uint8_t)106, (uint8_t)132, (uint8_t)214, (uint8_t)60, (uint8_t)148, (uint8_t)44, (uint8_t)193, (uint8_t)61, (uint8_t)16, (uint8_t)168, (uint8_t)25, (uint8_t)195, (uint8_t)209, (uint8_t)123, (uint8_t)254, (uint8_t)162, (uint8_t)205, (uint8_t)145, (uint8_t)71, (uint8_t)140, (uint8_t)101, (uint8_t)185, (uint8_t)77, (uint8_t)78, (uint8_t)113, (uint8_t)87, (uint8_t)110, (uint8_t)56, (uint8_t)62, (uint8_t)197};
        p233_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIGH_LATENCY_234(), &PH);
    p234_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED |
                        e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED), PH.base.pack) ;
    p234_custom_mode_SET((uint32_t)3911615143L, PH.base.pack) ;
    p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, PH.base.pack) ;
    p234_roll_SET((int16_t)(int16_t)12678, PH.base.pack) ;
    p234_pitch_SET((int16_t)(int16_t) -26220, PH.base.pack) ;
    p234_heading_SET((uint16_t)(uint16_t)33579, PH.base.pack) ;
    p234_throttle_SET((int8_t)(int8_t) -62, PH.base.pack) ;
    p234_heading_sp_SET((int16_t)(int16_t)29246, PH.base.pack) ;
    p234_latitude_SET((int32_t) -626951179, PH.base.pack) ;
    p234_longitude_SET((int32_t) -1246431903, PH.base.pack) ;
    p234_altitude_amsl_SET((int16_t)(int16_t) -7577, PH.base.pack) ;
    p234_altitude_sp_SET((int16_t)(int16_t) -8882, PH.base.pack) ;
    p234_airspeed_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
    p234_airspeed_sp_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
    p234_groundspeed_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
    p234_climb_rate_SET((int8_t)(int8_t) -123, PH.base.pack) ;
    p234_gps_nsat_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
    p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_3D_FIX, PH.base.pack) ;
    p234_battery_remaining_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
    p234_temperature_SET((int8_t)(int8_t)35, PH.base.pack) ;
    p234_temperature_air_SET((int8_t)(int8_t) -18, PH.base.pack) ;
    p234_failsafe_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
    p234_wp_num_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    p234_wp_distance_SET((uint16_t)(uint16_t)44473, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VIBRATION_241(), &PH);
    p241_time_usec_SET((uint64_t)262592919393784822L, PH.base.pack) ;
    p241_vibration_x_SET((float)3.2718494E38F, PH.base.pack) ;
    p241_vibration_y_SET((float) -2.8089665E38F, PH.base.pack) ;
    p241_vibration_z_SET((float) -2.6326706E38F, PH.base.pack) ;
    p241_clipping_0_SET((uint32_t)505854146L, PH.base.pack) ;
    p241_clipping_1_SET((uint32_t)1917595893L, PH.base.pack) ;
    p241_clipping_2_SET((uint32_t)1103568017L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HOME_POSITION_242(), &PH);
    p242_latitude_SET((int32_t)100655370, PH.base.pack) ;
    p242_longitude_SET((int32_t) -1879891856, PH.base.pack) ;
    p242_altitude_SET((int32_t)685299298, PH.base.pack) ;
    p242_x_SET((float)1.4123497E38F, PH.base.pack) ;
    p242_y_SET((float)1.3630583E38F, PH.base.pack) ;
    p242_z_SET((float) -1.4112179E38F, PH.base.pack) ;
    {
        float  q [] =  {-1.0056269E38F, 1.0968317E38F, -2.0005717E38F, -1.6999068E36F};
        p242_q_SET(&q, 0, &PH.base.pack) ;
    }
    p242_approach_x_SET((float) -6.6809517E37F, PH.base.pack) ;
    p242_approach_y_SET((float)7.884983E37F, PH.base.pack) ;
    p242_approach_z_SET((float)3.3606006E38F, PH.base.pack) ;
    p242_time_usec_SET((uint64_t)3633811278315167919L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_HOME_POSITION_243(), &PH);
    p243_target_system_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    p243_latitude_SET((int32_t)2022397048, PH.base.pack) ;
    p243_longitude_SET((int32_t)1295308315, PH.base.pack) ;
    p243_altitude_SET((int32_t) -161689242, PH.base.pack) ;
    p243_x_SET((float)2.3775069E38F, PH.base.pack) ;
    p243_y_SET((float)1.9550513E38F, PH.base.pack) ;
    p243_z_SET((float) -1.5058844E38F, PH.base.pack) ;
    {
        float  q [] =  {-6.266916E37F, 1.0956074E38F, 1.5893664E38F, -1.9241243E38F};
        p243_q_SET(&q, 0, &PH.base.pack) ;
    }
    p243_approach_x_SET((float) -1.3066433E38F, PH.base.pack) ;
    p243_approach_y_SET((float)7.588266E37F, PH.base.pack) ;
    p243_approach_z_SET((float)1.0512071E37F, PH.base.pack) ;
    p243_time_usec_SET((uint64_t)4275511453430885093L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MESSAGE_INTERVAL_244(), &PH);
    p244_message_id_SET((uint16_t)(uint16_t)34383, PH.base.pack) ;
    p244_interval_us_SET((int32_t) -1028995976, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_EXTENDED_SYS_STATE_245(), &PH);
    p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_UNDEFINED, PH.base.pack) ;
    p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ADSB_VEHICLE_246(), &PH);
    p246_ICAO_address_SET((uint32_t)276124833L, PH.base.pack) ;
    p246_lat_SET((int32_t)1273954264, PH.base.pack) ;
    p246_lon_SET((int32_t) -586871147, PH.base.pack) ;
    p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
    p246_altitude_SET((int32_t)293920344, PH.base.pack) ;
    p246_heading_SET((uint16_t)(uint16_t)60245, PH.base.pack) ;
    p246_hor_velocity_SET((uint16_t)(uint16_t)8913, PH.base.pack) ;
    p246_ver_velocity_SET((int16_t)(int16_t) -31049, PH.base.pack) ;
    {
        char16_t   callsign = "pumsfc";
        p246_callsign_SET(&callsign, 0,  sizeof(callsign), &PH) ;
    }
    p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UNASSIGNED2, PH.base.pack) ;
    p246_tslc_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
    p246_flags_SET((e_ADSB_FLAGS_ADSB_FLAGS_VALID_HEADING), PH.base.pack) ;
    p246_squawk_SET((uint16_t)(uint16_t)8914, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COLLISION_247(), &PH);
    p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, PH.base.pack) ;
    p247_id_SET((uint32_t)4072933469L, PH.base.pack) ;
    p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_HOVER, PH.base.pack) ;
    p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW, PH.base.pack) ;
    p247_time_to_minimum_delta_SET((float)1.6098681E38F, PH.base.pack) ;
    p247_altitude_minimum_delta_SET((float)2.355356E38F, PH.base.pack) ;
    p247_horizontal_minimum_delta_SET((float)1.5054929E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_V2_EXTENSION_248(), &PH);
    p248_target_network_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
    p248_target_system_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
    p248_target_component_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
    p248_message_type_SET((uint16_t)(uint16_t)43994, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)27, (uint8_t)22, (uint8_t)251, (uint8_t)154, (uint8_t)47, (uint8_t)123, (uint8_t)166, (uint8_t)124, (uint8_t)83, (uint8_t)39, (uint8_t)100, (uint8_t)109, (uint8_t)36, (uint8_t)255, (uint8_t)238, (uint8_t)85, (uint8_t)175, (uint8_t)177, (uint8_t)155, (uint8_t)208, (uint8_t)91, (uint8_t)173, (uint8_t)77, (uint8_t)207, (uint8_t)140, (uint8_t)97, (uint8_t)108, (uint8_t)197, (uint8_t)168, (uint8_t)217, (uint8_t)118, (uint8_t)97, (uint8_t)154, (uint8_t)149, (uint8_t)145, (uint8_t)170, (uint8_t)147, (uint8_t)54, (uint8_t)247, (uint8_t)214, (uint8_t)90, (uint8_t)15, (uint8_t)194, (uint8_t)70, (uint8_t)229, (uint8_t)45, (uint8_t)251, (uint8_t)103, (uint8_t)226, (uint8_t)204, (uint8_t)165, (uint8_t)114, (uint8_t)8, (uint8_t)14, (uint8_t)216, (uint8_t)245, (uint8_t)140, (uint8_t)192, (uint8_t)128, (uint8_t)154, (uint8_t)235, (uint8_t)237, (uint8_t)154, (uint8_t)147, (uint8_t)125, (uint8_t)0, (uint8_t)118, (uint8_t)28, (uint8_t)183, (uint8_t)191, (uint8_t)125, (uint8_t)133, (uint8_t)158, (uint8_t)177, (uint8_t)209, (uint8_t)109, (uint8_t)224, (uint8_t)221, (uint8_t)166, (uint8_t)123, (uint8_t)131, (uint8_t)247, (uint8_t)29, (uint8_t)44, (uint8_t)97, (uint8_t)85, (uint8_t)63, (uint8_t)50, (uint8_t)150, (uint8_t)113, (uint8_t)1, (uint8_t)235, (uint8_t)117, (uint8_t)223, (uint8_t)81, (uint8_t)183, (uint8_t)3, (uint8_t)211, (uint8_t)168, (uint8_t)87, (uint8_t)91, (uint8_t)216, (uint8_t)253, (uint8_t)139, (uint8_t)229, (uint8_t)110, (uint8_t)112, (uint8_t)96, (uint8_t)31, (uint8_t)209, (uint8_t)216, (uint8_t)153, (uint8_t)118, (uint8_t)54, (uint8_t)181, (uint8_t)43, (uint8_t)77, (uint8_t)180, (uint8_t)220, (uint8_t)243, (uint8_t)127, (uint8_t)97, (uint8_t)57, (uint8_t)29, (uint8_t)65, (uint8_t)245, (uint8_t)91, (uint8_t)94, (uint8_t)131, (uint8_t)231, (uint8_t)151, (uint8_t)214, (uint8_t)27, (uint8_t)50, (uint8_t)174, (uint8_t)9, (uint8_t)168, (uint8_t)226, (uint8_t)30, (uint8_t)87, (uint8_t)143, (uint8_t)165, (uint8_t)29, (uint8_t)197, (uint8_t)56, (uint8_t)7, (uint8_t)198, (uint8_t)43, (uint8_t)129, (uint8_t)238, (uint8_t)118, (uint8_t)42, (uint8_t)177, (uint8_t)83, (uint8_t)167, (uint8_t)98, (uint8_t)222, (uint8_t)58, (uint8_t)166, (uint8_t)133, (uint8_t)158, (uint8_t)75, (uint8_t)200, (uint8_t)94, (uint8_t)98, (uint8_t)151, (uint8_t)11, (uint8_t)24, (uint8_t)164, (uint8_t)60, (uint8_t)119, (uint8_t)62, (uint8_t)208, (uint8_t)246, (uint8_t)248, (uint8_t)11, (uint8_t)134, (uint8_t)125, (uint8_t)1, (uint8_t)225, (uint8_t)60, (uint8_t)254, (uint8_t)151, (uint8_t)178, (uint8_t)217, (uint8_t)130, (uint8_t)90, (uint8_t)238, (uint8_t)232, (uint8_t)132, (uint8_t)226, (uint8_t)210, (uint8_t)207, (uint8_t)69, (uint8_t)71, (uint8_t)48, (uint8_t)80, (uint8_t)190, (uint8_t)247, (uint8_t)162, (uint8_t)2, (uint8_t)23, (uint8_t)128, (uint8_t)207, (uint8_t)60, (uint8_t)254, (uint8_t)189, (uint8_t)166, (uint8_t)116, (uint8_t)175, (uint8_t)251, (uint8_t)216, (uint8_t)237, (uint8_t)11, (uint8_t)63, (uint8_t)237, (uint8_t)26, (uint8_t)237, (uint8_t)113, (uint8_t)198, (uint8_t)37, (uint8_t)102, (uint8_t)6, (uint8_t)235, (uint8_t)208, (uint8_t)110, (uint8_t)120, (uint8_t)37, (uint8_t)158, (uint8_t)46, (uint8_t)145, (uint8_t)119, (uint8_t)218, (uint8_t)165, (uint8_t)234, (uint8_t)242, (uint8_t)176, (uint8_t)65, (uint8_t)70, (uint8_t)69, (uint8_t)118, (uint8_t)93, (uint8_t)32, (uint8_t)86, (uint8_t)177, (uint8_t)92, (uint8_t)125, (uint8_t)129, (uint8_t)139};
        p248_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MEMORY_VECT_249(), &PH);
    p249_address_SET((uint16_t)(uint16_t)49677, PH.base.pack) ;
    p249_ver_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
    p249_type_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
    {
        int8_t  value [] =  {(int8_t) -87, (int8_t)127, (int8_t)56, (int8_t) -29, (int8_t) -122, (int8_t) -49, (int8_t)76, (int8_t) -73, (int8_t) -81, (int8_t) -50, (int8_t)6, (int8_t) -8, (int8_t) -7, (int8_t)23, (int8_t) -33, (int8_t) -127, (int8_t) -126, (int8_t)7, (int8_t) -88, (int8_t)63, (int8_t)117, (int8_t) -96, (int8_t)127, (int8_t)25, (int8_t) -77, (int8_t)30, (int8_t)38, (int8_t)4, (int8_t)10, (int8_t) -7, (int8_t) -109, (int8_t)83};
        p249_value_SET(&value, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DEBUG_VECT_250(), &PH);
    {
        char16_t   name = "tfjQwzr";
        p250_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p250_time_usec_SET((uint64_t)6004727741565533909L, PH.base.pack) ;
    p250_x_SET((float) -1.8778146E38F, PH.base.pack) ;
    p250_y_SET((float)2.7740134E38F, PH.base.pack) ;
    p250_z_SET((float) -3.1927205E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
    p251_time_boot_ms_SET((uint32_t)3079952730L, PH.base.pack) ;
    {
        char16_t   name = "b";
        p251_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p251_value_SET((float) -2.725571E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_NAMED_VALUE_INT_252(), &PH);
    p252_time_boot_ms_SET((uint32_t)2487640513L, PH.base.pack) ;
    {
        char16_t   name = "fbzhs";
        p252_name_SET(&name, 0,  sizeof(name), &PH) ;
    }
    p252_value_SET((int32_t)1429357956, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_STATUSTEXT_253(), &PH);
    p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_ERROR, PH.base.pack) ;
    {
        char16_t   text = "ilnnnoXfvq";
        p253_text_SET(&text, 0,  sizeof(text), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DEBUG_254(), &PH);
    p254_time_boot_ms_SET((uint32_t)262097726L, PH.base.pack) ;
    p254_ind_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
    p254_value_SET((float) -3.0179538E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SETUP_SIGNING_256(), &PH);
    p256_target_system_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
    p256_target_component_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
    {
        uint8_t  secret_key [] =  {(uint8_t)230, (uint8_t)207, (uint8_t)248, (uint8_t)243, (uint8_t)31, (uint8_t)186, (uint8_t)49, (uint8_t)171, (uint8_t)212, (uint8_t)102, (uint8_t)160, (uint8_t)110, (uint8_t)143, (uint8_t)233, (uint8_t)231, (uint8_t)252, (uint8_t)207, (uint8_t)230, (uint8_t)28, (uint8_t)241, (uint8_t)237, (uint8_t)71, (uint8_t)4, (uint8_t)199, (uint8_t)167, (uint8_t)244, (uint8_t)157, (uint8_t)210, (uint8_t)116, (uint8_t)120, (uint8_t)38, (uint8_t)207};
        p256_secret_key_SET(&secret_key, 0, &PH.base.pack) ;
    }
    p256_initial_timestamp_SET((uint64_t)5939427613693815825L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_BUTTON_CHANGE_257(), &PH);
    p257_time_boot_ms_SET((uint32_t)2482371148L, PH.base.pack) ;
    p257_last_change_ms_SET((uint32_t)4185515061L, PH.base.pack) ;
    p257_state_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PLAY_TUNE_258(), &PH);
    p258_target_system_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
    p258_target_component_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
    {
        char16_t   tune = "gnzdjbpjlezpRaimlGhwftAuvjtcrt";
        p258_tune_SET(&tune, 0,  sizeof(tune), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_INFORMATION_259(), &PH);
    p259_time_boot_ms_SET((uint32_t)1881177120L, PH.base.pack) ;
    {
        uint8_t  vendor_name [] =  {(uint8_t)74, (uint8_t)248, (uint8_t)53, (uint8_t)252, (uint8_t)133, (uint8_t)58, (uint8_t)66, (uint8_t)16, (uint8_t)251, (uint8_t)239, (uint8_t)254, (uint8_t)170, (uint8_t)218, (uint8_t)24, (uint8_t)238, (uint8_t)38, (uint8_t)207, (uint8_t)205, (uint8_t)112, (uint8_t)137, (uint8_t)90, (uint8_t)228, (uint8_t)209, (uint8_t)31, (uint8_t)246, (uint8_t)1, (uint8_t)3, (uint8_t)180, (uint8_t)92, (uint8_t)25, (uint8_t)255, (uint8_t)33};
        p259_vendor_name_SET(&vendor_name, 0, &PH.base.pack) ;
    }
    {
        uint8_t  model_name [] =  {(uint8_t)154, (uint8_t)201, (uint8_t)32, (uint8_t)237, (uint8_t)44, (uint8_t)175, (uint8_t)206, (uint8_t)136, (uint8_t)63, (uint8_t)112, (uint8_t)43, (uint8_t)84, (uint8_t)114, (uint8_t)56, (uint8_t)184, (uint8_t)64, (uint8_t)192, (uint8_t)89, (uint8_t)23, (uint8_t)61, (uint8_t)163, (uint8_t)144, (uint8_t)3, (uint8_t)210, (uint8_t)170, (uint8_t)190, (uint8_t)125, (uint8_t)35, (uint8_t)75, (uint8_t)120, (uint8_t)117, (uint8_t)155};
        p259_model_name_SET(&model_name, 0, &PH.base.pack) ;
    }
    p259_firmware_version_SET((uint32_t)1258402261L, PH.base.pack) ;
    p259_focal_length_SET((float) -1.3117845E38F, PH.base.pack) ;
    p259_sensor_size_h_SET((float) -6.693684E37F, PH.base.pack) ;
    p259_sensor_size_v_SET((float) -7.6419013E37F, PH.base.pack) ;
    p259_resolution_h_SET((uint16_t)(uint16_t)52959, PH.base.pack) ;
    p259_resolution_v_SET((uint16_t)(uint16_t)51734, PH.base.pack) ;
    p259_lens_id_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
    p259_flags_SET((e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                    e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE), PH.base.pack) ;
    p259_cam_definition_version_SET((uint16_t)(uint16_t)23321, PH.base.pack) ;
    {
        char16_t   cam_definition_uri = "znwreaahprdsxaexBbuazbczgcvZgxwpoewmynjiwmdxchvPzzYvgfwnigGujytUBqcQkirxjkpvckptkBnowiuRrtvrkiqhjtfLdgvzarjsrhexBdgbldrjwnaqkmHHevmCdar";
        p259_cam_definition_uri_SET(&cam_definition_uri, 0,  sizeof(cam_definition_uri), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_SETTINGS_260(), &PH);
    p260_time_boot_ms_SET((uint32_t)3670208156L, PH.base.pack) ;
    p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_VIDEO, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_STORAGE_INFORMATION_261(), &PH);
    p261_time_boot_ms_SET((uint32_t)2723765199L, PH.base.pack) ;
    p261_storage_id_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
    p261_storage_count_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
    p261_status_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
    p261_total_capacity_SET((float) -9.591786E37F, PH.base.pack) ;
    p261_used_capacity_SET((float) -2.9385553E37F, PH.base.pack) ;
    p261_available_capacity_SET((float) -1.2285598E37F, PH.base.pack) ;
    p261_read_speed_SET((float)1.159426E38F, PH.base.pack) ;
    p261_write_speed_SET((float) -6.7052896E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
    p262_time_boot_ms_SET((uint32_t)3257299541L, PH.base.pack) ;
    p262_image_status_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    p262_video_status_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
    p262_image_interval_SET((float)1.9200643E38F, PH.base.pack) ;
    p262_recording_time_ms_SET((uint32_t)3694621673L, PH.base.pack) ;
    p262_available_capacity_SET((float)2.6054548E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
    p263_time_boot_ms_SET((uint32_t)216524076L, PH.base.pack) ;
    p263_time_utc_SET((uint64_t)7063805209508973274L, PH.base.pack) ;
    p263_camera_id_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
    p263_lat_SET((int32_t) -529993874, PH.base.pack) ;
    p263_lon_SET((int32_t) -1081882568, PH.base.pack) ;
    p263_alt_SET((int32_t)70073026, PH.base.pack) ;
    p263_relative_alt_SET((int32_t) -1302938421, PH.base.pack) ;
    {
        float  q [] =  {-1.5377201E38F, -2.1615485E38F, -3.2729053E38F, 2.8705942E38F};
        p263_q_SET(&q, 0, &PH.base.pack) ;
    }
    p263_image_index_SET((int32_t)27121121, PH.base.pack) ;
    p263_capture_result_SET((int8_t)(int8_t) -4, PH.base.pack) ;
    {
        char16_t   file_url = "kojkshyvxnhbitfuauZonfzxxfttoIxvyqvifnjisgzNxmjwKtvxuitzhqtfgoztXnjegeijhqaekbkqetlmftsrwxdTmXrGXaxfppa";
        p263_file_url_SET(&file_url, 0,  sizeof(file_url), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_FLIGHT_INFORMATION_264(), &PH);
    p264_time_boot_ms_SET((uint32_t)1734358918L, PH.base.pack) ;
    p264_arming_time_utc_SET((uint64_t)6072246649656908820L, PH.base.pack) ;
    p264_takeoff_time_utc_SET((uint64_t)9186353611596876048L, PH.base.pack) ;
    p264_flight_uuid_SET((uint64_t)8533419607464417428L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}
