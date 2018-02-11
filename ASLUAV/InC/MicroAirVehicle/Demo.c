
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
void c_CommunicationChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_SENS_POWER_201(Bounds_Inside * ph, Pack * pack)
{
    float  adc121_vspb_volt = p201_adc121_vspb_volt_GET(pack);
    float  adc121_cspb_amp = p201_adc121_cspb_amp_GET(pack);
    float  adc121_cs1_amp = p201_adc121_cs1_amp_GET(pack);
    float  adc121_cs2_amp = p201_adc121_cs2_amp_GET(pack);
}
void c_CommunicationChannel_on_SENS_MPPT_202(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  mppt_timestamp = p202_mppt_timestamp_GET(pack);
    float  mppt1_volt = p202_mppt1_volt_GET(pack);
    float  mppt1_amp = p202_mppt1_amp_GET(pack);
    uint16_t  mppt1_pwm = p202_mppt1_pwm_GET(pack);
    uint8_t  mppt1_status = p202_mppt1_status_GET(pack);
    float  mppt2_volt = p202_mppt2_volt_GET(pack);
    float  mppt2_amp = p202_mppt2_amp_GET(pack);
    uint16_t  mppt2_pwm = p202_mppt2_pwm_GET(pack);
    uint8_t  mppt2_status = p202_mppt2_status_GET(pack);
    float  mppt3_volt = p202_mppt3_volt_GET(pack);
    float  mppt3_amp = p202_mppt3_amp_GET(pack);
    uint16_t  mppt3_pwm = p202_mppt3_pwm_GET(pack);
    uint8_t  mppt3_status = p202_mppt3_status_GET(pack);
}
void c_CommunicationChannel_on_ASLCTRL_DATA_203(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  timestamp = p203_timestamp_GET(pack);
    uint8_t  aslctrl_mode = p203_aslctrl_mode_GET(pack);
    float  h = p203_h_GET(pack);
    float  hRef = p203_hRef_GET(pack);
    float  hRef_t = p203_hRef_t_GET(pack);
    float  PitchAngle = p203_PitchAngle_GET(pack);
    float  PitchAngleRef = p203_PitchAngleRef_GET(pack);
    float  q = p203_q_GET(pack);
    float  qRef = p203_qRef_GET(pack);
    float  uElev = p203_uElev_GET(pack);
    float  uThrot = p203_uThrot_GET(pack);
    float  uThrot2 = p203_uThrot2_GET(pack);
    float  nZ = p203_nZ_GET(pack);
    float  AirspeedRef = p203_AirspeedRef_GET(pack);
    uint8_t  SpoilersEngaged = p203_SpoilersEngaged_GET(pack);
    float  YawAngle = p203_YawAngle_GET(pack);
    float  YawAngleRef = p203_YawAngleRef_GET(pack);
    float  RollAngle = p203_RollAngle_GET(pack);
    float  RollAngleRef = p203_RollAngleRef_GET(pack);
    float  p = p203_p_GET(pack);
    float  pRef = p203_pRef_GET(pack);
    float  r = p203_r_GET(pack);
    float  rRef = p203_rRef_GET(pack);
    float  uAil = p203_uAil_GET(pack);
    float  uRud = p203_uRud_GET(pack);
}
void c_CommunicationChannel_on_ASLCTRL_DEBUG_204(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  i32_1 = p204_i32_1_GET(pack);
    uint8_t  i8_1 = p204_i8_1_GET(pack);
    uint8_t  i8_2 = p204_i8_2_GET(pack);
    float  f_1 = p204_f_1_GET(pack);
    float  f_2 = p204_f_2_GET(pack);
    float  f_3 = p204_f_3_GET(pack);
    float  f_4 = p204_f_4_GET(pack);
    float  f_5 = p204_f_5_GET(pack);
    float  f_6 = p204_f_6_GET(pack);
    float  f_7 = p204_f_7_GET(pack);
    float  f_8 = p204_f_8_GET(pack);
}
void c_CommunicationChannel_on_ASLUAV_STATUS_205(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  LED_status = p205_LED_status_GET(pack);
    uint8_t  SATCOM_status = p205_SATCOM_status_GET(pack);
    uint8_t*  Servo_status = p205_Servo_status_GET_(pack);
//process data in Servo_status
    free(Servo_status);//never forget to dispose
    float  Motor_rpm = p205_Motor_rpm_GET(pack);
}
void c_CommunicationChannel_on_EKF_EXT_206(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  timestamp = p206_timestamp_GET(pack);
    float  Windspeed = p206_Windspeed_GET(pack);
    float  WindDir = p206_WindDir_GET(pack);
    float  WindZ = p206_WindZ_GET(pack);
    float  Airspeed = p206_Airspeed_GET(pack);
    float  beta = p206_beta_GET(pack);
    float  alpha = p206_alpha_GET(pack);
}
void c_CommunicationChannel_on_ASL_OBCTRL_207(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  timestamp = p207_timestamp_GET(pack);
    float  uElev = p207_uElev_GET(pack);
    float  uThrot = p207_uThrot_GET(pack);
    float  uThrot2 = p207_uThrot2_GET(pack);
    float  uAilL = p207_uAilL_GET(pack);
    float  uAilR = p207_uAilR_GET(pack);
    float  uRud = p207_uRud_GET(pack);
    uint8_t  obctrl_status = p207_obctrl_status_GET(pack);
}
void c_CommunicationChannel_on_SENS_ATMOS_208(Bounds_Inside * ph, Pack * pack)
{
    float  TempAmbient = p208_TempAmbient_GET(pack);
    float  Humidity = p208_Humidity_GET(pack);
}
void c_CommunicationChannel_on_SENS_BATMON_209(Bounds_Inside * ph, Pack * pack)
{
    float  temperature = p209_temperature_GET(pack);
    uint16_t  voltage = p209_voltage_GET(pack);
    int16_t  current = p209_current_GET(pack);
    uint8_t  SoC = p209_SoC_GET(pack);
    uint16_t  batterystatus = p209_batterystatus_GET(pack);
    uint16_t  serialnumber = p209_serialnumber_GET(pack);
    uint16_t  hostfetcontrol = p209_hostfetcontrol_GET(pack);
    uint16_t  cellvoltage1 = p209_cellvoltage1_GET(pack);
    uint16_t  cellvoltage2 = p209_cellvoltage2_GET(pack);
    uint16_t  cellvoltage3 = p209_cellvoltage3_GET(pack);
    uint16_t  cellvoltage4 = p209_cellvoltage4_GET(pack);
    uint16_t  cellvoltage5 = p209_cellvoltage5_GET(pack);
    uint16_t  cellvoltage6 = p209_cellvoltage6_GET(pack);
}
void c_CommunicationChannel_on_FW_SOARING_DATA_210(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  timestamp = p210_timestamp_GET(pack);
    uint64_t  timestampModeChanged = p210_timestampModeChanged_GET(pack);
    float  xW = p210_xW_GET(pack);
    float  xR = p210_xR_GET(pack);
    float  xLat = p210_xLat_GET(pack);
    float  xLon = p210_xLon_GET(pack);
    float  VarW = p210_VarW_GET(pack);
    float  VarR = p210_VarR_GET(pack);
    float  VarLat = p210_VarLat_GET(pack);
    float  VarLon = p210_VarLon_GET(pack);
    float  LoiterRadius = p210_LoiterRadius_GET(pack);
    float  LoiterDirection = p210_LoiterDirection_GET(pack);
    float  DistToSoarPoint = p210_DistToSoarPoint_GET(pack);
    float  vSinkExp = p210_vSinkExp_GET(pack);
    float  z1_LocalUpdraftSpeed = p210_z1_LocalUpdraftSpeed_GET(pack);
    float  z2_DeltaRoll = p210_z2_DeltaRoll_GET(pack);
    float  z1_exp = p210_z1_exp_GET(pack);
    float  z2_exp = p210_z2_exp_GET(pack);
    float  ThermalGSNorth = p210_ThermalGSNorth_GET(pack);
    float  ThermalGSEast = p210_ThermalGSEast_GET(pack);
    float  TSE_dot = p210_TSE_dot_GET(pack);
    float  DebugVar1 = p210_DebugVar1_GET(pack);
    float  DebugVar2 = p210_DebugVar2_GET(pack);
    uint8_t  ControlMode = p210_ControlMode_GET(pack);
    uint8_t  valid = p210_valid_GET(pack);
}
void c_CommunicationChannel_on_SENSORPOD_STATUS_211(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  timestamp = p211_timestamp_GET(pack);
    uint8_t  visensor_rate_1 = p211_visensor_rate_1_GET(pack);
    uint8_t  visensor_rate_2 = p211_visensor_rate_2_GET(pack);
    uint8_t  visensor_rate_3 = p211_visensor_rate_3_GET(pack);
    uint8_t  visensor_rate_4 = p211_visensor_rate_4_GET(pack);
    uint8_t  recording_nodes_count = p211_recording_nodes_count_GET(pack);
    uint8_t  cpu_temp = p211_cpu_temp_GET(pack);
    uint16_t  free_space = p211_free_space_GET(pack);
}
void c_CommunicationChannel_on_SENS_POWER_BOARD_212(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  timestamp = p212_timestamp_GET(pack);
    uint8_t  pwr_brd_status = p212_pwr_brd_status_GET(pack);
    uint8_t  pwr_brd_led_status = p212_pwr_brd_led_status_GET(pack);
    float  pwr_brd_system_volt = p212_pwr_brd_system_volt_GET(pack);
    float  pwr_brd_servo_volt = p212_pwr_brd_servo_volt_GET(pack);
    float  pwr_brd_mot_l_amp = p212_pwr_brd_mot_l_amp_GET(pack);
    float  pwr_brd_mot_r_amp = p212_pwr_brd_mot_r_amp_GET(pack);
    float  pwr_brd_servo_1_amp = p212_pwr_brd_servo_1_amp_GET(pack);
    float  pwr_brd_servo_2_amp = p212_pwr_brd_servo_2_amp_GET(pack);
    float  pwr_brd_servo_3_amp = p212_pwr_brd_servo_3_amp_GET(pack);
    float  pwr_brd_servo_4_amp = p212_pwr_brd_servo_4_amp_GET(pack);
    float  pwr_brd_aux_amp = p212_pwr_brd_aux_amp_GET(pack);
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
    p0_type_SET(e_MAV_TYPE_MAV_TYPE_VTOL_RESERVED5, PH.base.pack) ;
    p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_RESERVED, PH.base.pack) ;
    p0_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED |
                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED |
                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED), PH.base.pack) ;
    p0_custom_mode_SET((uint32_t)2201187185L, PH.base.pack) ;
    p0_system_status_SET(e_MAV_STATE_MAV_STATE_STANDBY, PH.base.pack) ;
    p0_mavlink_version_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
    p1_onboard_control_sensors_present_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
    p1_onboard_control_sensors_enabled_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
    p1_onboard_control_sensors_health_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_AHRS |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
    p1_load_SET((uint16_t)(uint16_t)56890, PH.base.pack) ;
    p1_voltage_battery_SET((uint16_t)(uint16_t)59096, PH.base.pack) ;
    p1_current_battery_SET((int16_t)(int16_t) -8853, PH.base.pack) ;
    p1_battery_remaining_SET((int8_t)(int8_t) -9, PH.base.pack) ;
    p1_drop_rate_comm_SET((uint16_t)(uint16_t)41415, PH.base.pack) ;
    p1_errors_comm_SET((uint16_t)(uint16_t)28407, PH.base.pack) ;
    p1_errors_count1_SET((uint16_t)(uint16_t)50002, PH.base.pack) ;
    p1_errors_count2_SET((uint16_t)(uint16_t)11954, PH.base.pack) ;
    p1_errors_count3_SET((uint16_t)(uint16_t)26808, PH.base.pack) ;
    p1_errors_count4_SET((uint16_t)(uint16_t)52045, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
    p2_time_unix_usec_SET((uint64_t)6791166292361810332L, PH.base.pack) ;
    p2_time_boot_ms_SET((uint32_t)1736165163L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
    p3_time_boot_ms_SET((uint32_t)1783983640L, PH.base.pack) ;
    p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
    p3_type_mask_SET((uint16_t)(uint16_t)26887, PH.base.pack) ;
    p3_x_SET((float)2.5482329E38F, PH.base.pack) ;
    p3_y_SET((float) -1.831874E38F, PH.base.pack) ;
    p3_z_SET((float) -1.9558904E38F, PH.base.pack) ;
    p3_vx_SET((float)8.4276186E37F, PH.base.pack) ;
    p3_vy_SET((float) -1.7726201E38F, PH.base.pack) ;
    p3_vz_SET((float) -1.1813718E38F, PH.base.pack) ;
    p3_afx_SET((float) -3.1582778E38F, PH.base.pack) ;
    p3_afy_SET((float)1.9376043E38F, PH.base.pack) ;
    p3_afz_SET((float)2.3010487E38F, PH.base.pack) ;
    p3_yaw_SET((float)1.4243126E38F, PH.base.pack) ;
    p3_yaw_rate_SET((float)1.3977048E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PING_4(), &PH);
    p4_time_usec_SET((uint64_t)5677122388790058393L, PH.base.pack) ;
    p4_seq_SET((uint32_t)781101867L, PH.base.pack) ;
    p4_target_system_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
    p4_target_component_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
    p5_target_system_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
    p5_control_request_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
    p5_version_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
    {
        char16_t   passkey = "dhlmy";
        p5_passkey_SET(&passkey, 0,  sizeof(passkey), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
    p6_gcs_system_id_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
    p6_control_request_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
    p6_ack_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
    {
        char16_t   key = "oRx";
        p7_key_SET(&key, 0,  sizeof(key), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
    p11_target_system_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
    p11_base_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_ARMED, PH.base.pack) ;
    p11_custom_mode_SET((uint32_t)391548891L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
    p20_target_system_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
    p20_target_component_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
    {
        char16_t   param_id = "kwwgIrcwqvseji";
        p20_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p20_param_index_SET((int16_t)(int16_t) -29640, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
    p21_target_system_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
    p21_target_component_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
    {
        char16_t   param_id = "az";
        p22_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p22_param_value_SET((float)1.1734394E38F, PH.base.pack) ;
    p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT8, PH.base.pack) ;
    p22_param_count_SET((uint16_t)(uint16_t)40941, PH.base.pack) ;
    p22_param_index_SET((uint16_t)(uint16_t)34857, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
    p23_target_system_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
    p23_target_component_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
    {
        char16_t   param_id = "mnbdytxd";
        p23_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p23_param_value_SET((float) -2.8138186E38F, PH.base.pack) ;
    p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
    p24_time_usec_SET((uint64_t)8923494868176310717L, PH.base.pack) ;
    p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, PH.base.pack) ;
    p24_lat_SET((int32_t) -66652848, PH.base.pack) ;
    p24_lon_SET((int32_t) -1911554945, PH.base.pack) ;
    p24_alt_SET((int32_t) -1232346263, PH.base.pack) ;
    p24_eph_SET((uint16_t)(uint16_t)59644, PH.base.pack) ;
    p24_epv_SET((uint16_t)(uint16_t)47802, PH.base.pack) ;
    p24_vel_SET((uint16_t)(uint16_t)15934, PH.base.pack) ;
    p24_cog_SET((uint16_t)(uint16_t)28740, PH.base.pack) ;
    p24_satellites_visible_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
    p24_alt_ellipsoid_SET((int32_t)1161611241, &PH) ;
    p24_h_acc_SET((uint32_t)3655212505L, &PH) ;
    p24_v_acc_SET((uint32_t)1660270880L, &PH) ;
    p24_vel_acc_SET((uint32_t)3437264579L, &PH) ;
    p24_hdg_acc_SET((uint32_t)2786472676L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
    p25_satellites_visible_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
    {
        uint8_t  satellite_prn [] =  {(uint8_t)218, (uint8_t)10, (uint8_t)174, (uint8_t)18, (uint8_t)10, (uint8_t)232, (uint8_t)118, (uint8_t)93, (uint8_t)19, (uint8_t)164, (uint8_t)84, (uint8_t)209, (uint8_t)162, (uint8_t)74, (uint8_t)41, (uint8_t)205, (uint8_t)48, (uint8_t)162, (uint8_t)63, (uint8_t)178};
        p25_satellite_prn_SET(&satellite_prn, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_used [] =  {(uint8_t)223, (uint8_t)55, (uint8_t)2, (uint8_t)33, (uint8_t)49, (uint8_t)41, (uint8_t)143, (uint8_t)63, (uint8_t)61, (uint8_t)134, (uint8_t)76, (uint8_t)205, (uint8_t)107, (uint8_t)178, (uint8_t)200, (uint8_t)107, (uint8_t)231, (uint8_t)13, (uint8_t)249, (uint8_t)161};
        p25_satellite_used_SET(&satellite_used, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_elevation [] =  {(uint8_t)220, (uint8_t)116, (uint8_t)109, (uint8_t)18, (uint8_t)121, (uint8_t)119, (uint8_t)119, (uint8_t)192, (uint8_t)23, (uint8_t)42, (uint8_t)86, (uint8_t)198, (uint8_t)51, (uint8_t)215, (uint8_t)44, (uint8_t)219, (uint8_t)253, (uint8_t)248, (uint8_t)150, (uint8_t)9};
        p25_satellite_elevation_SET(&satellite_elevation, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_azimuth [] =  {(uint8_t)188, (uint8_t)254, (uint8_t)206, (uint8_t)22, (uint8_t)145, (uint8_t)247, (uint8_t)106, (uint8_t)60, (uint8_t)101, (uint8_t)249, (uint8_t)77, (uint8_t)217, (uint8_t)240, (uint8_t)12, (uint8_t)43, (uint8_t)127, (uint8_t)1, (uint8_t)213, (uint8_t)216, (uint8_t)71};
        p25_satellite_azimuth_SET(&satellite_azimuth, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_snr [] =  {(uint8_t)146, (uint8_t)94, (uint8_t)160, (uint8_t)70, (uint8_t)83, (uint8_t)91, (uint8_t)240, (uint8_t)143, (uint8_t)198, (uint8_t)180, (uint8_t)186, (uint8_t)231, (uint8_t)46, (uint8_t)12, (uint8_t)202, (uint8_t)55, (uint8_t)34, (uint8_t)58, (uint8_t)78, (uint8_t)154};
        p25_satellite_snr_SET(&satellite_snr, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
    p26_time_boot_ms_SET((uint32_t)579838249L, PH.base.pack) ;
    p26_xacc_SET((int16_t)(int16_t)11322, PH.base.pack) ;
    p26_yacc_SET((int16_t)(int16_t) -1444, PH.base.pack) ;
    p26_zacc_SET((int16_t)(int16_t) -30275, PH.base.pack) ;
    p26_xgyro_SET((int16_t)(int16_t) -1929, PH.base.pack) ;
    p26_ygyro_SET((int16_t)(int16_t)11275, PH.base.pack) ;
    p26_zgyro_SET((int16_t)(int16_t)12621, PH.base.pack) ;
    p26_xmag_SET((int16_t)(int16_t) -29122, PH.base.pack) ;
    p26_ymag_SET((int16_t)(int16_t) -6571, PH.base.pack) ;
    p26_zmag_SET((int16_t)(int16_t)29833, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
    p27_time_usec_SET((uint64_t)8799981321075467182L, PH.base.pack) ;
    p27_xacc_SET((int16_t)(int16_t) -32355, PH.base.pack) ;
    p27_yacc_SET((int16_t)(int16_t) -13857, PH.base.pack) ;
    p27_zacc_SET((int16_t)(int16_t) -21193, PH.base.pack) ;
    p27_xgyro_SET((int16_t)(int16_t) -32529, PH.base.pack) ;
    p27_ygyro_SET((int16_t)(int16_t)22797, PH.base.pack) ;
    p27_zgyro_SET((int16_t)(int16_t) -18702, PH.base.pack) ;
    p27_xmag_SET((int16_t)(int16_t) -4234, PH.base.pack) ;
    p27_ymag_SET((int16_t)(int16_t)19311, PH.base.pack) ;
    p27_zmag_SET((int16_t)(int16_t)24140, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
    p28_time_usec_SET((uint64_t)6355311060475779369L, PH.base.pack) ;
    p28_press_abs_SET((int16_t)(int16_t) -11026, PH.base.pack) ;
    p28_press_diff1_SET((int16_t)(int16_t)18601, PH.base.pack) ;
    p28_press_diff2_SET((int16_t)(int16_t)31435, PH.base.pack) ;
    p28_temperature_SET((int16_t)(int16_t) -8809, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
    p29_time_boot_ms_SET((uint32_t)2762034891L, PH.base.pack) ;
    p29_press_abs_SET((float)9.768094E37F, PH.base.pack) ;
    p29_press_diff_SET((float) -1.1230685E38F, PH.base.pack) ;
    p29_temperature_SET((int16_t)(int16_t) -9087, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
    p30_time_boot_ms_SET((uint32_t)1509936101L, PH.base.pack) ;
    p30_roll_SET((float) -2.9769225E38F, PH.base.pack) ;
    p30_pitch_SET((float)2.884003E37F, PH.base.pack) ;
    p30_yaw_SET((float)9.058421E37F, PH.base.pack) ;
    p30_rollspeed_SET((float) -2.645663E38F, PH.base.pack) ;
    p30_pitchspeed_SET((float) -9.241656E37F, PH.base.pack) ;
    p30_yawspeed_SET((float)3.1572193E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
    p31_time_boot_ms_SET((uint32_t)2532498997L, PH.base.pack) ;
    p31_q1_SET((float)4.098188E35F, PH.base.pack) ;
    p31_q2_SET((float) -3.0477967E38F, PH.base.pack) ;
    p31_q3_SET((float) -1.8799603E38F, PH.base.pack) ;
    p31_q4_SET((float)3.0402265E38F, PH.base.pack) ;
    p31_rollspeed_SET((float)1.1953652E38F, PH.base.pack) ;
    p31_pitchspeed_SET((float) -2.9207915E38F, PH.base.pack) ;
    p31_yawspeed_SET((float) -2.5268396E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
    p32_time_boot_ms_SET((uint32_t)3410394913L, PH.base.pack) ;
    p32_x_SET((float) -1.7131385E37F, PH.base.pack) ;
    p32_y_SET((float) -3.2576203E38F, PH.base.pack) ;
    p32_z_SET((float)1.2513575E38F, PH.base.pack) ;
    p32_vx_SET((float) -1.7733707E37F, PH.base.pack) ;
    p32_vy_SET((float)1.8511423E38F, PH.base.pack) ;
    p32_vz_SET((float) -4.3392983E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
    p33_time_boot_ms_SET((uint32_t)3058495100L, PH.base.pack) ;
    p33_lat_SET((int32_t) -1655826634, PH.base.pack) ;
    p33_lon_SET((int32_t) -203692284, PH.base.pack) ;
    p33_alt_SET((int32_t)288467512, PH.base.pack) ;
    p33_relative_alt_SET((int32_t)1293345827, PH.base.pack) ;
    p33_vx_SET((int16_t)(int16_t)54, PH.base.pack) ;
    p33_vy_SET((int16_t)(int16_t) -28243, PH.base.pack) ;
    p33_vz_SET((int16_t)(int16_t) -8514, PH.base.pack) ;
    p33_hdg_SET((uint16_t)(uint16_t)40665, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
    p34_time_boot_ms_SET((uint32_t)3666850938L, PH.base.pack) ;
    p34_port_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
    p34_chan1_scaled_SET((int16_t)(int16_t)15255, PH.base.pack) ;
    p34_chan2_scaled_SET((int16_t)(int16_t)5320, PH.base.pack) ;
    p34_chan3_scaled_SET((int16_t)(int16_t) -22604, PH.base.pack) ;
    p34_chan4_scaled_SET((int16_t)(int16_t) -14846, PH.base.pack) ;
    p34_chan5_scaled_SET((int16_t)(int16_t) -20096, PH.base.pack) ;
    p34_chan6_scaled_SET((int16_t)(int16_t) -25461, PH.base.pack) ;
    p34_chan7_scaled_SET((int16_t)(int16_t)19829, PH.base.pack) ;
    p34_chan8_scaled_SET((int16_t)(int16_t)27665, PH.base.pack) ;
    p34_rssi_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
    p35_time_boot_ms_SET((uint32_t)54286098L, PH.base.pack) ;
    p35_port_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
    p35_chan1_raw_SET((uint16_t)(uint16_t)1835, PH.base.pack) ;
    p35_chan2_raw_SET((uint16_t)(uint16_t)62889, PH.base.pack) ;
    p35_chan3_raw_SET((uint16_t)(uint16_t)65103, PH.base.pack) ;
    p35_chan4_raw_SET((uint16_t)(uint16_t)58623, PH.base.pack) ;
    p35_chan5_raw_SET((uint16_t)(uint16_t)4994, PH.base.pack) ;
    p35_chan6_raw_SET((uint16_t)(uint16_t)28813, PH.base.pack) ;
    p35_chan7_raw_SET((uint16_t)(uint16_t)44871, PH.base.pack) ;
    p35_chan8_raw_SET((uint16_t)(uint16_t)51138, PH.base.pack) ;
    p35_rssi_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
    p36_time_usec_SET((uint32_t)226747420L, PH.base.pack) ;
    p36_port_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
    p36_servo1_raw_SET((uint16_t)(uint16_t)27836, PH.base.pack) ;
    p36_servo2_raw_SET((uint16_t)(uint16_t)61191, PH.base.pack) ;
    p36_servo3_raw_SET((uint16_t)(uint16_t)806, PH.base.pack) ;
    p36_servo4_raw_SET((uint16_t)(uint16_t)53936, PH.base.pack) ;
    p36_servo5_raw_SET((uint16_t)(uint16_t)5569, PH.base.pack) ;
    p36_servo6_raw_SET((uint16_t)(uint16_t)37015, PH.base.pack) ;
    p36_servo7_raw_SET((uint16_t)(uint16_t)630, PH.base.pack) ;
    p36_servo8_raw_SET((uint16_t)(uint16_t)17448, PH.base.pack) ;
    p36_servo9_raw_SET((uint16_t)(uint16_t)55642, &PH) ;
    p36_servo10_raw_SET((uint16_t)(uint16_t)61540, &PH) ;
    p36_servo11_raw_SET((uint16_t)(uint16_t)29103, &PH) ;
    p36_servo12_raw_SET((uint16_t)(uint16_t)59887, &PH) ;
    p36_servo13_raw_SET((uint16_t)(uint16_t)55188, &PH) ;
    p36_servo14_raw_SET((uint16_t)(uint16_t)33779, &PH) ;
    p36_servo15_raw_SET((uint16_t)(uint16_t)18448, &PH) ;
    p36_servo16_raw_SET((uint16_t)(uint16_t)17962, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
    p37_target_system_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
    p37_target_component_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
    p37_start_index_SET((int16_t)(int16_t) -18005, PH.base.pack) ;
    p37_end_index_SET((int16_t)(int16_t) -7120, PH.base.pack) ;
    p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
    p38_target_system_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
    p38_target_component_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
    p38_start_index_SET((int16_t)(int16_t)9788, PH.base.pack) ;
    p38_end_index_SET((int16_t)(int16_t)22658, PH.base.pack) ;
    p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
    p39_target_system_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
    p39_target_component_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
    p39_seq_SET((uint16_t)(uint16_t)25134, PH.base.pack) ;
    p39_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
    p39_command_SET(e_MAV_CMD_MAV_CMD_NAV_SET_YAW_SPEED, PH.base.pack) ;
    p39_current_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
    p39_autocontinue_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
    p39_param1_SET((float) -1.8067749E38F, PH.base.pack) ;
    p39_param2_SET((float)1.9612613E38F, PH.base.pack) ;
    p39_param3_SET((float)3.0333228E38F, PH.base.pack) ;
    p39_param4_SET((float) -2.9429478E38F, PH.base.pack) ;
    p39_x_SET((float) -1.0592561E38F, PH.base.pack) ;
    p39_y_SET((float) -2.6837713E38F, PH.base.pack) ;
    p39_z_SET((float)6.262717E37F, PH.base.pack) ;
    p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
    p40_target_system_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
    p40_target_component_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
    p40_seq_SET((uint16_t)(uint16_t)4214, PH.base.pack) ;
    p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
    p41_target_system_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
    p41_target_component_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
    p41_seq_SET((uint16_t)(uint16_t)32937, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
    p42_seq_SET((uint16_t)(uint16_t)23942, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
    p43_target_system_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
    p43_target_component_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
    p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
    p44_target_system_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
    p44_target_component_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
    p44_count_SET((uint16_t)(uint16_t)30419, PH.base.pack) ;
    p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
    p45_target_system_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
    p45_target_component_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
    p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
    p46_seq_SET((uint16_t)(uint16_t)3235, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
    p47_target_system_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
    p47_target_component_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
    p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM5_X, PH.base.pack) ;
    p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
    p48_target_system_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
    p48_latitude_SET((int32_t) -906141108, PH.base.pack) ;
    p48_longitude_SET((int32_t) -181732577, PH.base.pack) ;
    p48_altitude_SET((int32_t)380662200, PH.base.pack) ;
    p48_time_usec_SET((uint64_t)2911223747541999525L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
    p49_latitude_SET((int32_t)1095820945, PH.base.pack) ;
    p49_longitude_SET((int32_t)2033832905, PH.base.pack) ;
    p49_altitude_SET((int32_t) -701144648, PH.base.pack) ;
    p49_time_usec_SET((uint64_t)6869394159430768138L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
    p50_target_system_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
    p50_target_component_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
    {
        char16_t   param_id = "wciqokbeonkhthzr";
        p50_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p50_param_index_SET((int16_t)(int16_t) -4606, PH.base.pack) ;
    p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
    p50_param_value0_SET((float)3.8435263E37F, PH.base.pack) ;
    p50_scale_SET((float) -1.1787138E37F, PH.base.pack) ;
    p50_param_value_min_SET((float)2.8826478E38F, PH.base.pack) ;
    p50_param_value_max_SET((float)2.379297E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
    p51_target_system_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
    p51_target_component_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
    p51_seq_SET((uint16_t)(uint16_t)50278, PH.base.pack) ;
    p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
    p54_target_system_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
    p54_target_component_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
    p54_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
    p54_p1x_SET((float) -3.2455238E38F, PH.base.pack) ;
    p54_p1y_SET((float)2.3939048E38F, PH.base.pack) ;
    p54_p1z_SET((float) -2.920961E38F, PH.base.pack) ;
    p54_p2x_SET((float) -8.464592E37F, PH.base.pack) ;
    p54_p2y_SET((float)3.063485E38F, PH.base.pack) ;
    p54_p2z_SET((float)8.0488E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
    p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
    p55_p1x_SET((float) -1.5041004E38F, PH.base.pack) ;
    p55_p1y_SET((float)2.1413854E37F, PH.base.pack) ;
    p55_p1z_SET((float)2.720262E38F, PH.base.pack) ;
    p55_p2x_SET((float) -3.3031689E38F, PH.base.pack) ;
    p55_p2y_SET((float) -5.6525585E37F, PH.base.pack) ;
    p55_p2z_SET((float) -2.4401589E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
    p61_time_usec_SET((uint64_t)8570164182894649592L, PH.base.pack) ;
    {
        float  q [] =  {-8.733526E37F, -2.4493253E38F, 2.9985366E38F, -1.1280966E38F};
        p61_q_SET(&q, 0, &PH.base.pack) ;
    }
    p61_rollspeed_SET((float) -2.0065992E38F, PH.base.pack) ;
    p61_pitchspeed_SET((float) -2.6136694E38F, PH.base.pack) ;
    p61_yawspeed_SET((float) -5.0240046E37F, PH.base.pack) ;
    {
        float  covariance [] =  {3.3564027E38F, 2.212174E38F, -1.2564594E38F, -2.4984671E38F, 2.2161502E38F, -2.863882E38F, -5.769167E37F, 1.6144079E38F, -2.8877667E38F};
        p61_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
    p62_nav_roll_SET((float) -7.5009085E36F, PH.base.pack) ;
    p62_nav_pitch_SET((float)3.3070937E38F, PH.base.pack) ;
    p62_nav_bearing_SET((int16_t)(int16_t)22281, PH.base.pack) ;
    p62_target_bearing_SET((int16_t)(int16_t)85, PH.base.pack) ;
    p62_wp_dist_SET((uint16_t)(uint16_t)7334, PH.base.pack) ;
    p62_alt_error_SET((float) -3.0803192E38F, PH.base.pack) ;
    p62_aspd_error_SET((float)4.6273045E37F, PH.base.pack) ;
    p62_xtrack_error_SET((float)1.3999432E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
    p63_time_usec_SET((uint64_t)2217346795427138671L, PH.base.pack) ;
    p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
    p63_lat_SET((int32_t) -1266514086, PH.base.pack) ;
    p63_lon_SET((int32_t)1318805148, PH.base.pack) ;
    p63_alt_SET((int32_t)1292184404, PH.base.pack) ;
    p63_relative_alt_SET((int32_t) -1430791908, PH.base.pack) ;
    p63_vx_SET((float) -3.2151846E38F, PH.base.pack) ;
    p63_vy_SET((float) -3.5389235E37F, PH.base.pack) ;
    p63_vz_SET((float) -3.2463264E38F, PH.base.pack) ;
    {
        float  covariance [] =  {-1.0472362E38F, 3.2706775E38F, -1.006326E38F, -6.936558E37F, 2.1453743E38F, -2.2003098E38F, -1.5240448E38F, 3.0645715E38F, -1.0567337E38F, 3.0215718E38F, -1.9544035E38F, 4.605902E37F, -1.9996897E37F, -5.3432994E37F, 3.2309207E38F, 1.4320647E38F, -1.0414777E38F, 1.2860565E38F, 2.6395826E38F, -1.2758482E38F, -1.4493233E38F, 3.0698254E38F, 7.5208245E37F, -4.3321427E37F, -2.2042576E38F, -2.4603014E37F, -1.942864E38F, -1.0622544E38F, -3.314324E38F, 3.2993091E37F, 1.7901082E38F, -2.105836E38F, 1.2231975E38F, -3.0457399E38F, -3.1510695E38F, 9.147967E37F};
        p63_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
    p64_time_usec_SET((uint64_t)8585431058930280885L, PH.base.pack) ;
    p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, PH.base.pack) ;
    p64_x_SET((float) -5.0941523E37F, PH.base.pack) ;
    p64_y_SET((float) -9.820992E37F, PH.base.pack) ;
    p64_z_SET((float)1.1643745E38F, PH.base.pack) ;
    p64_vx_SET((float)2.567097E38F, PH.base.pack) ;
    p64_vy_SET((float) -1.2377562E38F, PH.base.pack) ;
    p64_vz_SET((float)2.6958927E38F, PH.base.pack) ;
    p64_ax_SET((float) -3.395844E38F, PH.base.pack) ;
    p64_ay_SET((float) -1.4414878E38F, PH.base.pack) ;
    p64_az_SET((float)2.5326489E38F, PH.base.pack) ;
    {
        float  covariance [] =  {1.9668155E38F, 2.5475114E38F, 1.0746221E38F, -3.1477032E36F, 1.4596127E38F, 3.1976077E38F, 4.504997E37F, -5.0619616E37F, -1.301386E38F, 2.1928088E38F, 1.0108835E38F, -1.3743051E38F, -2.5116627E38F, 1.5212221E38F, 9.99049E37F, 1.8866795E38F, -2.8843489E38F, -3.1544546E38F, 2.623261E38F, 2.1064351E38F, 2.8256196E38F, -3.103483E38F, 3.064332E38F, 1.9653618E38F, -2.3321358E38F, 2.9953172E38F, -1.8013976E38F, 9.554226E36F, 2.597982E38F, 2.8750522E38F, 7.699007E37F, -1.874088E38F, -1.0944985E38F, 5.6682195E37F, -9.213513E37F, 2.1611761E38F, 2.1397423E38F, 2.2880812E37F, 2.6156913E37F, 3.3451151E38F, 3.2372866E38F, 1.2728449E38F, 1.1082454E38F, -5.20969E37F, -2.5557643E38F};
        p64_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
    p65_time_boot_ms_SET((uint32_t)4064696567L, PH.base.pack) ;
    p65_chancount_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
    p65_chan1_raw_SET((uint16_t)(uint16_t)35299, PH.base.pack) ;
    p65_chan2_raw_SET((uint16_t)(uint16_t)18650, PH.base.pack) ;
    p65_chan3_raw_SET((uint16_t)(uint16_t)1898, PH.base.pack) ;
    p65_chan4_raw_SET((uint16_t)(uint16_t)60562, PH.base.pack) ;
    p65_chan5_raw_SET((uint16_t)(uint16_t)17948, PH.base.pack) ;
    p65_chan6_raw_SET((uint16_t)(uint16_t)16295, PH.base.pack) ;
    p65_chan7_raw_SET((uint16_t)(uint16_t)12381, PH.base.pack) ;
    p65_chan8_raw_SET((uint16_t)(uint16_t)63896, PH.base.pack) ;
    p65_chan9_raw_SET((uint16_t)(uint16_t)49549, PH.base.pack) ;
    p65_chan10_raw_SET((uint16_t)(uint16_t)54594, PH.base.pack) ;
    p65_chan11_raw_SET((uint16_t)(uint16_t)6238, PH.base.pack) ;
    p65_chan12_raw_SET((uint16_t)(uint16_t)12090, PH.base.pack) ;
    p65_chan13_raw_SET((uint16_t)(uint16_t)60855, PH.base.pack) ;
    p65_chan14_raw_SET((uint16_t)(uint16_t)14581, PH.base.pack) ;
    p65_chan15_raw_SET((uint16_t)(uint16_t)23813, PH.base.pack) ;
    p65_chan16_raw_SET((uint16_t)(uint16_t)8672, PH.base.pack) ;
    p65_chan17_raw_SET((uint16_t)(uint16_t)48007, PH.base.pack) ;
    p65_chan18_raw_SET((uint16_t)(uint16_t)47747, PH.base.pack) ;
    p65_rssi_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
    p66_target_system_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
    p66_target_component_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
    p66_req_stream_id_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
    p66_req_message_rate_SET((uint16_t)(uint16_t)826, PH.base.pack) ;
    p66_start_stop_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
    p67_stream_id_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
    p67_message_rate_SET((uint16_t)(uint16_t)22211, PH.base.pack) ;
    p67_on_off_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
    p69_target_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
    p69_x_SET((int16_t)(int16_t)31106, PH.base.pack) ;
    p69_y_SET((int16_t)(int16_t) -9268, PH.base.pack) ;
    p69_z_SET((int16_t)(int16_t) -27421, PH.base.pack) ;
    p69_r_SET((int16_t)(int16_t)8161, PH.base.pack) ;
    p69_buttons_SET((uint16_t)(uint16_t)21592, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
    p70_target_system_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
    p70_target_component_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
    p70_chan1_raw_SET((uint16_t)(uint16_t)26366, PH.base.pack) ;
    p70_chan2_raw_SET((uint16_t)(uint16_t)15861, PH.base.pack) ;
    p70_chan3_raw_SET((uint16_t)(uint16_t)41870, PH.base.pack) ;
    p70_chan4_raw_SET((uint16_t)(uint16_t)45530, PH.base.pack) ;
    p70_chan5_raw_SET((uint16_t)(uint16_t)11541, PH.base.pack) ;
    p70_chan6_raw_SET((uint16_t)(uint16_t)61600, PH.base.pack) ;
    p70_chan7_raw_SET((uint16_t)(uint16_t)9982, PH.base.pack) ;
    p70_chan8_raw_SET((uint16_t)(uint16_t)57052, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
    p73_target_system_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
    p73_target_component_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
    p73_seq_SET((uint16_t)(uint16_t)10220, PH.base.pack) ;
    p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
    p73_command_SET(e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE, PH.base.pack) ;
    p73_current_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
    p73_autocontinue_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
    p73_param1_SET((float)3.2129475E38F, PH.base.pack) ;
    p73_param2_SET((float)1.368144E38F, PH.base.pack) ;
    p73_param3_SET((float)1.2201582E38F, PH.base.pack) ;
    p73_param4_SET((float)2.2664903E38F, PH.base.pack) ;
    p73_x_SET((int32_t) -499469543, PH.base.pack) ;
    p73_y_SET((int32_t)1186111988, PH.base.pack) ;
    p73_z_SET((float)2.34911E37F, PH.base.pack) ;
    p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
    p74_airspeed_SET((float)2.633807E38F, PH.base.pack) ;
    p74_groundspeed_SET((float) -1.6525017E38F, PH.base.pack) ;
    p74_heading_SET((int16_t)(int16_t) -7175, PH.base.pack) ;
    p74_throttle_SET((uint16_t)(uint16_t)58069, PH.base.pack) ;
    p74_alt_SET((float)1.856145E38F, PH.base.pack) ;
    p74_climb_SET((float) -2.9824807E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
    p75_target_system_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
    p75_target_component_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
    p75_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
    p75_command_SET(e_MAV_CMD_MAV_CMD_DO_REPOSITION, PH.base.pack) ;
    p75_current_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
    p75_autocontinue_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
    p75_param1_SET((float)1.8924353E37F, PH.base.pack) ;
    p75_param2_SET((float)1.832551E38F, PH.base.pack) ;
    p75_param3_SET((float)1.063586E37F, PH.base.pack) ;
    p75_param4_SET((float)2.3507917E38F, PH.base.pack) ;
    p75_x_SET((int32_t) -1870397582, PH.base.pack) ;
    p75_y_SET((int32_t) -346197997, PH.base.pack) ;
    p75_z_SET((float) -2.470725E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
    p76_target_system_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    p76_target_component_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
    p76_command_SET(e_MAV_CMD_MAV_CMD_VIDEO_START_CAPTURE, PH.base.pack) ;
    p76_confirmation_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
    p76_param1_SET((float)2.5421644E38F, PH.base.pack) ;
    p76_param2_SET((float) -2.8856206E38F, PH.base.pack) ;
    p76_param3_SET((float)3.1576294E38F, PH.base.pack) ;
    p76_param4_SET((float)4.584168E37F, PH.base.pack) ;
    p76_param5_SET((float)7.9984717E37F, PH.base.pack) ;
    p76_param6_SET((float) -8.3503127E37F, PH.base.pack) ;
    p76_param7_SET((float) -3.2431918E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
    p77_command_SET(e_MAV_CMD_MAV_CMD_USER_1, PH.base.pack) ;
    p77_result_SET(e_MAV_RESULT_MAV_RESULT_TEMPORARILY_REJECTED, PH.base.pack) ;
    p77_progress_SET((uint8_t)(uint8_t)137, &PH) ;
    p77_result_param2_SET((int32_t)1689337330, &PH) ;
    p77_target_system_SET((uint8_t)(uint8_t)35, &PH) ;
    p77_target_component_SET((uint8_t)(uint8_t)134, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
    p81_time_boot_ms_SET((uint32_t)4049312931L, PH.base.pack) ;
    p81_roll_SET((float) -1.2680011E38F, PH.base.pack) ;
    p81_pitch_SET((float) -3.3968174E38F, PH.base.pack) ;
    p81_yaw_SET((float)2.9335098E38F, PH.base.pack) ;
    p81_thrust_SET((float)2.4098393E38F, PH.base.pack) ;
    p81_mode_switch_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
    p81_manual_override_switch_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
    p82_time_boot_ms_SET((uint32_t)1613162156L, PH.base.pack) ;
    p82_target_system_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
    p82_target_component_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
    p82_type_mask_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    {
        float  q [] =  {-1.5434881E38F, -8.480451E37F, -2.2385454E38F, -2.341493E38F};
        p82_q_SET(&q, 0, &PH.base.pack) ;
    }
    p82_body_roll_rate_SET((float) -1.0401008E38F, PH.base.pack) ;
    p82_body_pitch_rate_SET((float)1.0553878E38F, PH.base.pack) ;
    p82_body_yaw_rate_SET((float)1.03215896E37F, PH.base.pack) ;
    p82_thrust_SET((float) -2.8704504E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
    p83_time_boot_ms_SET((uint32_t)949205208L, PH.base.pack) ;
    p83_type_mask_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
    {
        float  q [] =  {1.9360422E38F, 9.431156E37F, -3.1538409E38F, 3.0747932E38F};
        p83_q_SET(&q, 0, &PH.base.pack) ;
    }
    p83_body_roll_rate_SET((float) -2.3531063E38F, PH.base.pack) ;
    p83_body_pitch_rate_SET((float)2.9713955E38F, PH.base.pack) ;
    p83_body_yaw_rate_SET((float) -1.4552778E37F, PH.base.pack) ;
    p83_thrust_SET((float) -9.397901E36F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
    p84_time_boot_ms_SET((uint32_t)2538741845L, PH.base.pack) ;
    p84_target_system_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
    p84_target_component_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
    p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
    p84_type_mask_SET((uint16_t)(uint16_t)36981, PH.base.pack) ;
    p84_x_SET((float) -2.1206208E38F, PH.base.pack) ;
    p84_y_SET((float)6.8793885E35F, PH.base.pack) ;
    p84_z_SET((float)2.5968997E38F, PH.base.pack) ;
    p84_vx_SET((float) -3.5776013E37F, PH.base.pack) ;
    p84_vy_SET((float) -1.2033491E38F, PH.base.pack) ;
    p84_vz_SET((float) -2.2173629E38F, PH.base.pack) ;
    p84_afx_SET((float)3.34507E38F, PH.base.pack) ;
    p84_afy_SET((float)1.8991582E38F, PH.base.pack) ;
    p84_afz_SET((float) -2.3071164E38F, PH.base.pack) ;
    p84_yaw_SET((float) -4.8976796E37F, PH.base.pack) ;
    p84_yaw_rate_SET((float)3.3197995E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
    p86_time_boot_ms_SET((uint32_t)105135173L, PH.base.pack) ;
    p86_target_system_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
    p86_target_component_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
    p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
    p86_type_mask_SET((uint16_t)(uint16_t)63512, PH.base.pack) ;
    p86_lat_int_SET((int32_t)1420120511, PH.base.pack) ;
    p86_lon_int_SET((int32_t) -909926977, PH.base.pack) ;
    p86_alt_SET((float) -1.7432382E38F, PH.base.pack) ;
    p86_vx_SET((float)1.1159322E38F, PH.base.pack) ;
    p86_vy_SET((float) -2.7315897E38F, PH.base.pack) ;
    p86_vz_SET((float)2.7123052E38F, PH.base.pack) ;
    p86_afx_SET((float)1.5689539E36F, PH.base.pack) ;
    p86_afy_SET((float) -1.2718594E38F, PH.base.pack) ;
    p86_afz_SET((float)1.8905348E38F, PH.base.pack) ;
    p86_yaw_SET((float)1.010589E38F, PH.base.pack) ;
    p86_yaw_rate_SET((float) -1.6461203E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
    p87_time_boot_ms_SET((uint32_t)2550682948L, PH.base.pack) ;
    p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
    p87_type_mask_SET((uint16_t)(uint16_t)58336, PH.base.pack) ;
    p87_lat_int_SET((int32_t) -437959290, PH.base.pack) ;
    p87_lon_int_SET((int32_t)902912682, PH.base.pack) ;
    p87_alt_SET((float) -7.072096E36F, PH.base.pack) ;
    p87_vx_SET((float) -3.025129E36F, PH.base.pack) ;
    p87_vy_SET((float) -3.1180371E38F, PH.base.pack) ;
    p87_vz_SET((float) -3.3527657E38F, PH.base.pack) ;
    p87_afx_SET((float) -3.2007644E38F, PH.base.pack) ;
    p87_afy_SET((float) -3.1401007E38F, PH.base.pack) ;
    p87_afz_SET((float) -6.201761E36F, PH.base.pack) ;
    p87_yaw_SET((float) -3.3885382E38F, PH.base.pack) ;
    p87_yaw_rate_SET((float) -2.0741275E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
    p89_time_boot_ms_SET((uint32_t)3613991523L, PH.base.pack) ;
    p89_x_SET((float)3.060826E38F, PH.base.pack) ;
    p89_y_SET((float)1.4347075E38F, PH.base.pack) ;
    p89_z_SET((float)2.8838017E38F, PH.base.pack) ;
    p89_roll_SET((float) -2.0184756E38F, PH.base.pack) ;
    p89_pitch_SET((float)4.0860245E37F, PH.base.pack) ;
    p89_yaw_SET((float)2.2745436E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
    p90_time_usec_SET((uint64_t)3096456882384616701L, PH.base.pack) ;
    p90_roll_SET((float)1.9972971E38F, PH.base.pack) ;
    p90_pitch_SET((float)1.3590445E38F, PH.base.pack) ;
    p90_yaw_SET((float)8.574109E37F, PH.base.pack) ;
    p90_rollspeed_SET((float) -1.6587071E38F, PH.base.pack) ;
    p90_pitchspeed_SET((float) -1.2700726E37F, PH.base.pack) ;
    p90_yawspeed_SET((float) -9.201678E37F, PH.base.pack) ;
    p90_lat_SET((int32_t)2024321636, PH.base.pack) ;
    p90_lon_SET((int32_t) -2028659991, PH.base.pack) ;
    p90_alt_SET((int32_t)1728247936, PH.base.pack) ;
    p90_vx_SET((int16_t)(int16_t)27894, PH.base.pack) ;
    p90_vy_SET((int16_t)(int16_t) -23051, PH.base.pack) ;
    p90_vz_SET((int16_t)(int16_t) -21995, PH.base.pack) ;
    p90_xacc_SET((int16_t)(int16_t) -28946, PH.base.pack) ;
    p90_yacc_SET((int16_t)(int16_t)13453, PH.base.pack) ;
    p90_zacc_SET((int16_t)(int16_t)31863, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
    p91_time_usec_SET((uint64_t)1651681917591088613L, PH.base.pack) ;
    p91_roll_ailerons_SET((float)3.07792E38F, PH.base.pack) ;
    p91_pitch_elevator_SET((float) -2.9726577E38F, PH.base.pack) ;
    p91_yaw_rudder_SET((float)1.4961061E38F, PH.base.pack) ;
    p91_throttle_SET((float)2.0218575E38F, PH.base.pack) ;
    p91_aux1_SET((float)2.6715426E38F, PH.base.pack) ;
    p91_aux2_SET((float) -2.1297546E38F, PH.base.pack) ;
    p91_aux3_SET((float)2.6049173E38F, PH.base.pack) ;
    p91_aux4_SET((float) -3.314329E38F, PH.base.pack) ;
    p91_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_DISARMED, PH.base.pack) ;
    p91_nav_mode_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
    p92_time_usec_SET((uint64_t)7652801895974468473L, PH.base.pack) ;
    p92_chan1_raw_SET((uint16_t)(uint16_t)50757, PH.base.pack) ;
    p92_chan2_raw_SET((uint16_t)(uint16_t)44489, PH.base.pack) ;
    p92_chan3_raw_SET((uint16_t)(uint16_t)49063, PH.base.pack) ;
    p92_chan4_raw_SET((uint16_t)(uint16_t)15312, PH.base.pack) ;
    p92_chan5_raw_SET((uint16_t)(uint16_t)16531, PH.base.pack) ;
    p92_chan6_raw_SET((uint16_t)(uint16_t)14978, PH.base.pack) ;
    p92_chan7_raw_SET((uint16_t)(uint16_t)52513, PH.base.pack) ;
    p92_chan8_raw_SET((uint16_t)(uint16_t)21935, PH.base.pack) ;
    p92_chan9_raw_SET((uint16_t)(uint16_t)1782, PH.base.pack) ;
    p92_chan10_raw_SET((uint16_t)(uint16_t)21121, PH.base.pack) ;
    p92_chan11_raw_SET((uint16_t)(uint16_t)44775, PH.base.pack) ;
    p92_chan12_raw_SET((uint16_t)(uint16_t)65438, PH.base.pack) ;
    p92_rssi_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
    p93_time_usec_SET((uint64_t)6326383582190867092L, PH.base.pack) ;
    {
        float  controls [] =  {-2.3660865E38F, -2.7644044E38F, 3.2107703E38F, 2.4840508E38F, -2.1045028E38F, 2.1287938E38F, -2.6325126E38F, -1.4062388E38F, 4.7054588E36F, -3.0761665E38F, 8.603017E37F, -2.616005E38F, -5.3019613E37F, 2.3223227E37F, 2.6920443E38F, 2.524919E38F};
        p93_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    p93_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_ARMED, PH.base.pack) ;
    p93_flags_SET((uint64_t)6051291327207663260L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
    p100_time_usec_SET((uint64_t)5720241244320102645L, PH.base.pack) ;
    p100_sensor_id_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
    p100_flow_x_SET((int16_t)(int16_t) -12206, PH.base.pack) ;
    p100_flow_y_SET((int16_t)(int16_t)19076, PH.base.pack) ;
    p100_flow_comp_m_x_SET((float)5.896041E37F, PH.base.pack) ;
    p100_flow_comp_m_y_SET((float) -3.0456543E38F, PH.base.pack) ;
    p100_quality_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
    p100_ground_distance_SET((float)2.4433353E38F, PH.base.pack) ;
    p100_flow_rate_x_SET((float)3.9567906E37F, &PH) ;
    p100_flow_rate_y_SET((float) -2.9452953E38F, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
    p101_usec_SET((uint64_t)6368577351963912544L, PH.base.pack) ;
    p101_x_SET((float)2.8915005E38F, PH.base.pack) ;
    p101_y_SET((float)6.608536E37F, PH.base.pack) ;
    p101_z_SET((float)1.2774369E38F, PH.base.pack) ;
    p101_roll_SET((float)7.374682E37F, PH.base.pack) ;
    p101_pitch_SET((float)1.2506385E37F, PH.base.pack) ;
    p101_yaw_SET((float) -2.4336932E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
    p102_usec_SET((uint64_t)3080955536155636058L, PH.base.pack) ;
    p102_x_SET((float) -1.2784195E38F, PH.base.pack) ;
    p102_y_SET((float) -2.097128E38F, PH.base.pack) ;
    p102_z_SET((float) -1.9903074E38F, PH.base.pack) ;
    p102_roll_SET((float) -2.3133068E38F, PH.base.pack) ;
    p102_pitch_SET((float)7.223705E37F, PH.base.pack) ;
    p102_yaw_SET((float)1.791305E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
    p103_usec_SET((uint64_t)6613718977582519345L, PH.base.pack) ;
    p103_x_SET((float) -2.5498252E38F, PH.base.pack) ;
    p103_y_SET((float) -9.710518E37F, PH.base.pack) ;
    p103_z_SET((float) -2.6110803E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
    p104_usec_SET((uint64_t)7082367013406058609L, PH.base.pack) ;
    p104_x_SET((float)3.1093768E37F, PH.base.pack) ;
    p104_y_SET((float) -2.9166066E38F, PH.base.pack) ;
    p104_z_SET((float)1.2203056E38F, PH.base.pack) ;
    p104_roll_SET((float)1.9663119E38F, PH.base.pack) ;
    p104_pitch_SET((float)1.4570026E37F, PH.base.pack) ;
    p104_yaw_SET((float)2.0139043E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
    p105_time_usec_SET((uint64_t)4482188031076442041L, PH.base.pack) ;
    p105_xacc_SET((float) -2.615638E38F, PH.base.pack) ;
    p105_yacc_SET((float)2.0148704E38F, PH.base.pack) ;
    p105_zacc_SET((float)1.5641526E38F, PH.base.pack) ;
    p105_xgyro_SET((float)1.9119128E37F, PH.base.pack) ;
    p105_ygyro_SET((float)2.4024279E38F, PH.base.pack) ;
    p105_zgyro_SET((float)1.8053545E38F, PH.base.pack) ;
    p105_xmag_SET((float) -2.053897E38F, PH.base.pack) ;
    p105_ymag_SET((float) -1.5946575E38F, PH.base.pack) ;
    p105_zmag_SET((float) -2.4290834E38F, PH.base.pack) ;
    p105_abs_pressure_SET((float) -3.0870714E38F, PH.base.pack) ;
    p105_diff_pressure_SET((float)1.7850597E38F, PH.base.pack) ;
    p105_pressure_alt_SET((float)6.556253E37F, PH.base.pack) ;
    p105_temperature_SET((float)2.9932291E38F, PH.base.pack) ;
    p105_fields_updated_SET((uint16_t)(uint16_t)57223, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
    p106_time_usec_SET((uint64_t)5235106387598777991L, PH.base.pack) ;
    p106_sensor_id_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
    p106_integration_time_us_SET((uint32_t)678641429L, PH.base.pack) ;
    p106_integrated_x_SET((float) -7.8277805E37F, PH.base.pack) ;
    p106_integrated_y_SET((float)4.9894525E37F, PH.base.pack) ;
    p106_integrated_xgyro_SET((float) -1.3245922E38F, PH.base.pack) ;
    p106_integrated_ygyro_SET((float)1.83872E38F, PH.base.pack) ;
    p106_integrated_zgyro_SET((float)1.2875326E38F, PH.base.pack) ;
    p106_temperature_SET((int16_t)(int16_t) -9331, PH.base.pack) ;
    p106_quality_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
    p106_time_delta_distance_us_SET((uint32_t)684245845L, PH.base.pack) ;
    p106_distance_SET((float)1.0262325E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
    p107_time_usec_SET((uint64_t)6367928032798698465L, PH.base.pack) ;
    p107_xacc_SET((float)1.3634665E38F, PH.base.pack) ;
    p107_yacc_SET((float) -1.1198279E38F, PH.base.pack) ;
    p107_zacc_SET((float) -2.7481495E38F, PH.base.pack) ;
    p107_xgyro_SET((float) -2.37004E37F, PH.base.pack) ;
    p107_ygyro_SET((float) -1.3631034E38F, PH.base.pack) ;
    p107_zgyro_SET((float)9.772317E37F, PH.base.pack) ;
    p107_xmag_SET((float)7.4302366E37F, PH.base.pack) ;
    p107_ymag_SET((float) -2.277526E38F, PH.base.pack) ;
    p107_zmag_SET((float) -5.168149E37F, PH.base.pack) ;
    p107_abs_pressure_SET((float)1.9422302E38F, PH.base.pack) ;
    p107_diff_pressure_SET((float)1.740575E38F, PH.base.pack) ;
    p107_pressure_alt_SET((float) -2.6524982E38F, PH.base.pack) ;
    p107_temperature_SET((float)1.7817619E37F, PH.base.pack) ;
    p107_fields_updated_SET((uint32_t)2834926424L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
    p108_q1_SET((float) -2.3569308E38F, PH.base.pack) ;
    p108_q2_SET((float) -5.098605E37F, PH.base.pack) ;
    p108_q3_SET((float) -9.501731E37F, PH.base.pack) ;
    p108_q4_SET((float)6.011529E37F, PH.base.pack) ;
    p108_roll_SET((float) -2.0061254E38F, PH.base.pack) ;
    p108_pitch_SET((float) -2.5103526E38F, PH.base.pack) ;
    p108_yaw_SET((float) -2.102208E38F, PH.base.pack) ;
    p108_xacc_SET((float) -1.7204328E37F, PH.base.pack) ;
    p108_yacc_SET((float) -3.5910833E37F, PH.base.pack) ;
    p108_zacc_SET((float) -4.100083E37F, PH.base.pack) ;
    p108_xgyro_SET((float)1.5320786E37F, PH.base.pack) ;
    p108_ygyro_SET((float)2.4513902E38F, PH.base.pack) ;
    p108_zgyro_SET((float)3.9097625E37F, PH.base.pack) ;
    p108_lat_SET((float)2.9165914E38F, PH.base.pack) ;
    p108_lon_SET((float)8.562636E37F, PH.base.pack) ;
    p108_alt_SET((float)2.4920626E38F, PH.base.pack) ;
    p108_std_dev_horz_SET((float)2.534454E38F, PH.base.pack) ;
    p108_std_dev_vert_SET((float)1.1581955E38F, PH.base.pack) ;
    p108_vn_SET((float) -2.7912794E38F, PH.base.pack) ;
    p108_ve_SET((float) -2.4375E38F, PH.base.pack) ;
    p108_vd_SET((float) -1.5300715E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
    p109_rssi_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
    p109_remrssi_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
    p109_txbuf_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
    p109_noise_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
    p109_remnoise_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
    p109_rxerrors_SET((uint16_t)(uint16_t)35006, PH.base.pack) ;
    p109_fixed__SET((uint16_t)(uint16_t)19436, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
    p110_target_network_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
    p110_target_system_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
    p110_target_component_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)241, (uint8_t)203, (uint8_t)231, (uint8_t)164, (uint8_t)80, (uint8_t)38, (uint8_t)91, (uint8_t)166, (uint8_t)89, (uint8_t)32, (uint8_t)23, (uint8_t)175, (uint8_t)48, (uint8_t)114, (uint8_t)16, (uint8_t)182, (uint8_t)143, (uint8_t)46, (uint8_t)77, (uint8_t)194, (uint8_t)253, (uint8_t)64, (uint8_t)68, (uint8_t)51, (uint8_t)248, (uint8_t)134, (uint8_t)93, (uint8_t)165, (uint8_t)210, (uint8_t)183, (uint8_t)101, (uint8_t)102, (uint8_t)8, (uint8_t)10, (uint8_t)157, (uint8_t)30, (uint8_t)127, (uint8_t)208, (uint8_t)119, (uint8_t)124, (uint8_t)98, (uint8_t)205, (uint8_t)21, (uint8_t)38, (uint8_t)27, (uint8_t)110, (uint8_t)165, (uint8_t)131, (uint8_t)92, (uint8_t)235, (uint8_t)150, (uint8_t)152, (uint8_t)113, (uint8_t)169, (uint8_t)122, (uint8_t)118, (uint8_t)201, (uint8_t)91, (uint8_t)194, (uint8_t)212, (uint8_t)112, (uint8_t)139, (uint8_t)103, (uint8_t)11, (uint8_t)114, (uint8_t)235, (uint8_t)128, (uint8_t)20, (uint8_t)206, (uint8_t)212, (uint8_t)48, (uint8_t)190, (uint8_t)14, (uint8_t)114, (uint8_t)119, (uint8_t)206, (uint8_t)177, (uint8_t)226, (uint8_t)218, (uint8_t)93, (uint8_t)247, (uint8_t)235, (uint8_t)201, (uint8_t)86, (uint8_t)149, (uint8_t)205, (uint8_t)227, (uint8_t)132, (uint8_t)240, (uint8_t)24, (uint8_t)173, (uint8_t)127, (uint8_t)105, (uint8_t)246, (uint8_t)205, (uint8_t)51, (uint8_t)43, (uint8_t)91, (uint8_t)28, (uint8_t)247, (uint8_t)162, (uint8_t)168, (uint8_t)95, (uint8_t)92, (uint8_t)205, (uint8_t)134, (uint8_t)42, (uint8_t)194, (uint8_t)96, (uint8_t)118, (uint8_t)177, (uint8_t)3, (uint8_t)67, (uint8_t)213, (uint8_t)108, (uint8_t)79, (uint8_t)13, (uint8_t)235, (uint8_t)38, (uint8_t)105, (uint8_t)164, (uint8_t)116, (uint8_t)140, (uint8_t)91, (uint8_t)111, (uint8_t)71, (uint8_t)225, (uint8_t)68, (uint8_t)250, (uint8_t)176, (uint8_t)54, (uint8_t)173, (uint8_t)98, (uint8_t)209, (uint8_t)107, (uint8_t)57, (uint8_t)167, (uint8_t)188, (uint8_t)225, (uint8_t)111, (uint8_t)71, (uint8_t)132, (uint8_t)149, (uint8_t)108, (uint8_t)59, (uint8_t)204, (uint8_t)68, (uint8_t)216, (uint8_t)216, (uint8_t)200, (uint8_t)5, (uint8_t)205, (uint8_t)19, (uint8_t)117, (uint8_t)87, (uint8_t)116, (uint8_t)126, (uint8_t)154, (uint8_t)41, (uint8_t)7, (uint8_t)241, (uint8_t)123, (uint8_t)100, (uint8_t)149, (uint8_t)150, (uint8_t)121, (uint8_t)165, (uint8_t)235, (uint8_t)78, (uint8_t)33, (uint8_t)149, (uint8_t)159, (uint8_t)96, (uint8_t)203, (uint8_t)207, (uint8_t)161, (uint8_t)227, (uint8_t)183, (uint8_t)77, (uint8_t)221, (uint8_t)71, (uint8_t)12, (uint8_t)226, (uint8_t)214, (uint8_t)239, (uint8_t)241, (uint8_t)92, (uint8_t)30, (uint8_t)21, (uint8_t)150, (uint8_t)54, (uint8_t)109, (uint8_t)118, (uint8_t)225, (uint8_t)112, (uint8_t)70, (uint8_t)237, (uint8_t)72, (uint8_t)21, (uint8_t)43, (uint8_t)123, (uint8_t)111, (uint8_t)135, (uint8_t)156, (uint8_t)50, (uint8_t)181, (uint8_t)135, (uint8_t)226, (uint8_t)123, (uint8_t)106, (uint8_t)130, (uint8_t)140, (uint8_t)161, (uint8_t)186, (uint8_t)133, (uint8_t)206, (uint8_t)130, (uint8_t)236, (uint8_t)183, (uint8_t)68, (uint8_t)42, (uint8_t)36, (uint8_t)205, (uint8_t)111, (uint8_t)44, (uint8_t)124, (uint8_t)151, (uint8_t)94, (uint8_t)96, (uint8_t)27, (uint8_t)65, (uint8_t)241, (uint8_t)159, (uint8_t)105, (uint8_t)13, (uint8_t)82, (uint8_t)180, (uint8_t)70, (uint8_t)1, (uint8_t)110, (uint8_t)1, (uint8_t)46, (uint8_t)4, (uint8_t)181, (uint8_t)164, (uint8_t)116, (uint8_t)118, (uint8_t)201, (uint8_t)29, (uint8_t)230, (uint8_t)162};
        p110_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
    p111_tc1_SET((int64_t) -2358467190905321438L, PH.base.pack) ;
    p111_ts1_SET((int64_t)2918544891280580487L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
    p112_time_usec_SET((uint64_t)5114163568098114566L, PH.base.pack) ;
    p112_seq_SET((uint32_t)4229179371L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
    p113_time_usec_SET((uint64_t)8682810361526319110L, PH.base.pack) ;
    p113_fix_type_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
    p113_lat_SET((int32_t) -113418158, PH.base.pack) ;
    p113_lon_SET((int32_t) -2499995, PH.base.pack) ;
    p113_alt_SET((int32_t)1761981240, PH.base.pack) ;
    p113_eph_SET((uint16_t)(uint16_t)64556, PH.base.pack) ;
    p113_epv_SET((uint16_t)(uint16_t)46837, PH.base.pack) ;
    p113_vel_SET((uint16_t)(uint16_t)18132, PH.base.pack) ;
    p113_vn_SET((int16_t)(int16_t) -15630, PH.base.pack) ;
    p113_ve_SET((int16_t)(int16_t) -22778, PH.base.pack) ;
    p113_vd_SET((int16_t)(int16_t)17037, PH.base.pack) ;
    p113_cog_SET((uint16_t)(uint16_t)38011, PH.base.pack) ;
    p113_satellites_visible_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
    p114_time_usec_SET((uint64_t)5403999418410045678L, PH.base.pack) ;
    p114_sensor_id_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
    p114_integration_time_us_SET((uint32_t)1389696064L, PH.base.pack) ;
    p114_integrated_x_SET((float) -1.6308623E37F, PH.base.pack) ;
    p114_integrated_y_SET((float) -1.5979531E38F, PH.base.pack) ;
    p114_integrated_xgyro_SET((float)2.669849E37F, PH.base.pack) ;
    p114_integrated_ygyro_SET((float) -1.4624736E38F, PH.base.pack) ;
    p114_integrated_zgyro_SET((float)2.602053E38F, PH.base.pack) ;
    p114_temperature_SET((int16_t)(int16_t) -31377, PH.base.pack) ;
    p114_quality_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
    p114_time_delta_distance_us_SET((uint32_t)40326332L, PH.base.pack) ;
    p114_distance_SET((float)1.1049828E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
    p115_time_usec_SET((uint64_t)2565468961887164933L, PH.base.pack) ;
    {
        float  attitude_quaternion [] =  {-5.923337E37F, -2.3201024E38F, -2.5304359E38F, 8.3105703E37F};
        p115_attitude_quaternion_SET(&attitude_quaternion, 0, &PH.base.pack) ;
    }
    p115_rollspeed_SET((float) -1.3675286E38F, PH.base.pack) ;
    p115_pitchspeed_SET((float) -1.0469271E38F, PH.base.pack) ;
    p115_yawspeed_SET((float)2.5599973E38F, PH.base.pack) ;
    p115_lat_SET((int32_t) -460026959, PH.base.pack) ;
    p115_lon_SET((int32_t) -540088594, PH.base.pack) ;
    p115_alt_SET((int32_t)778219492, PH.base.pack) ;
    p115_vx_SET((int16_t)(int16_t)20257, PH.base.pack) ;
    p115_vy_SET((int16_t)(int16_t) -12570, PH.base.pack) ;
    p115_vz_SET((int16_t)(int16_t)22238, PH.base.pack) ;
    p115_ind_airspeed_SET((uint16_t)(uint16_t)47443, PH.base.pack) ;
    p115_true_airspeed_SET((uint16_t)(uint16_t)7399, PH.base.pack) ;
    p115_xacc_SET((int16_t)(int16_t) -18268, PH.base.pack) ;
    p115_yacc_SET((int16_t)(int16_t)5628, PH.base.pack) ;
    p115_zacc_SET((int16_t)(int16_t)5701, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
    p116_time_boot_ms_SET((uint32_t)699029333L, PH.base.pack) ;
    p116_xacc_SET((int16_t)(int16_t) -30205, PH.base.pack) ;
    p116_yacc_SET((int16_t)(int16_t)23869, PH.base.pack) ;
    p116_zacc_SET((int16_t)(int16_t)25088, PH.base.pack) ;
    p116_xgyro_SET((int16_t)(int16_t) -7451, PH.base.pack) ;
    p116_ygyro_SET((int16_t)(int16_t)9387, PH.base.pack) ;
    p116_zgyro_SET((int16_t)(int16_t) -10795, PH.base.pack) ;
    p116_xmag_SET((int16_t)(int16_t)12562, PH.base.pack) ;
    p116_ymag_SET((int16_t)(int16_t) -12434, PH.base.pack) ;
    p116_zmag_SET((int16_t)(int16_t) -10920, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
    p117_target_system_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
    p117_target_component_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
    p117_start_SET((uint16_t)(uint16_t)50161, PH.base.pack) ;
    p117_end_SET((uint16_t)(uint16_t)32961, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
    p118_id_SET((uint16_t)(uint16_t)21126, PH.base.pack) ;
    p118_num_logs_SET((uint16_t)(uint16_t)35887, PH.base.pack) ;
    p118_last_log_num_SET((uint16_t)(uint16_t)12891, PH.base.pack) ;
    p118_time_utc_SET((uint32_t)2171835181L, PH.base.pack) ;
    p118_size_SET((uint32_t)1578037091L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
    p119_target_system_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
    p119_target_component_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
    p119_id_SET((uint16_t)(uint16_t)29377, PH.base.pack) ;
    p119_ofs_SET((uint32_t)702352396L, PH.base.pack) ;
    p119_count_SET((uint32_t)762968965L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
    p120_id_SET((uint16_t)(uint16_t)38652, PH.base.pack) ;
    p120_ofs_SET((uint32_t)1184408166L, PH.base.pack) ;
    p120_count_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)211, (uint8_t)120, (uint8_t)20, (uint8_t)143, (uint8_t)220, (uint8_t)81, (uint8_t)130, (uint8_t)215, (uint8_t)12, (uint8_t)164, (uint8_t)241, (uint8_t)243, (uint8_t)15, (uint8_t)2, (uint8_t)210, (uint8_t)203, (uint8_t)99, (uint8_t)68, (uint8_t)197, (uint8_t)87, (uint8_t)205, (uint8_t)229, (uint8_t)16, (uint8_t)145, (uint8_t)135, (uint8_t)223, (uint8_t)215, (uint8_t)205, (uint8_t)117, (uint8_t)252, (uint8_t)12, (uint8_t)161, (uint8_t)83, (uint8_t)62, (uint8_t)73, (uint8_t)148, (uint8_t)236, (uint8_t)69, (uint8_t)12, (uint8_t)122, (uint8_t)70, (uint8_t)198, (uint8_t)98, (uint8_t)85, (uint8_t)187, (uint8_t)0, (uint8_t)224, (uint8_t)225, (uint8_t)90, (uint8_t)82, (uint8_t)209, (uint8_t)154, (uint8_t)218, (uint8_t)53, (uint8_t)21, (uint8_t)187, (uint8_t)133, (uint8_t)65, (uint8_t)239, (uint8_t)237, (uint8_t)115, (uint8_t)74, (uint8_t)85, (uint8_t)37, (uint8_t)234, (uint8_t)122, (uint8_t)90, (uint8_t)66, (uint8_t)34, (uint8_t)108, (uint8_t)2, (uint8_t)120, (uint8_t)112, (uint8_t)67, (uint8_t)78, (uint8_t)0, (uint8_t)94, (uint8_t)88, (uint8_t)51, (uint8_t)69, (uint8_t)88, (uint8_t)16, (uint8_t)127, (uint8_t)147, (uint8_t)199, (uint8_t)55, (uint8_t)169, (uint8_t)211, (uint8_t)152, (uint8_t)37};
        p120_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
    p121_target_system_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    p121_target_component_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
    p122_target_system_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
    p122_target_component_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
    p123_target_system_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
    p123_target_component_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
    p123_len_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)196, (uint8_t)35, (uint8_t)103, (uint8_t)158, (uint8_t)14, (uint8_t)249, (uint8_t)33, (uint8_t)130, (uint8_t)53, (uint8_t)126, (uint8_t)85, (uint8_t)97, (uint8_t)119, (uint8_t)75, (uint8_t)183, (uint8_t)23, (uint8_t)219, (uint8_t)45, (uint8_t)170, (uint8_t)76, (uint8_t)20, (uint8_t)91, (uint8_t)116, (uint8_t)63, (uint8_t)144, (uint8_t)80, (uint8_t)180, (uint8_t)255, (uint8_t)193, (uint8_t)55, (uint8_t)238, (uint8_t)85, (uint8_t)128, (uint8_t)185, (uint8_t)255, (uint8_t)5, (uint8_t)234, (uint8_t)201, (uint8_t)7, (uint8_t)129, (uint8_t)16, (uint8_t)156, (uint8_t)108, (uint8_t)241, (uint8_t)93, (uint8_t)84, (uint8_t)226, (uint8_t)48, (uint8_t)136, (uint8_t)160, (uint8_t)19, (uint8_t)220, (uint8_t)79, (uint8_t)155, (uint8_t)108, (uint8_t)77, (uint8_t)187, (uint8_t)109, (uint8_t)180, (uint8_t)87, (uint8_t)38, (uint8_t)0, (uint8_t)17, (uint8_t)243, (uint8_t)179, (uint8_t)145, (uint8_t)24, (uint8_t)15, (uint8_t)115, (uint8_t)99, (uint8_t)167, (uint8_t)116, (uint8_t)237, (uint8_t)205, (uint8_t)202, (uint8_t)194, (uint8_t)196, (uint8_t)93, (uint8_t)52, (uint8_t)65, (uint8_t)105, (uint8_t)236, (uint8_t)164, (uint8_t)203, (uint8_t)30, (uint8_t)163, (uint8_t)254, (uint8_t)28, (uint8_t)36, (uint8_t)45, (uint8_t)92, (uint8_t)186, (uint8_t)6, (uint8_t)0, (uint8_t)211, (uint8_t)240, (uint8_t)121, (uint8_t)25, (uint8_t)38, (uint8_t)223, (uint8_t)221, (uint8_t)81, (uint8_t)95, (uint8_t)57, (uint8_t)2, (uint8_t)177, (uint8_t)127, (uint8_t)126, (uint8_t)45, (uint8_t)155};
        p123_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
    p124_time_usec_SET((uint64_t)8842523890409068891L, PH.base.pack) ;
    p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, PH.base.pack) ;
    p124_lat_SET((int32_t)1059688182, PH.base.pack) ;
    p124_lon_SET((int32_t) -337296495, PH.base.pack) ;
    p124_alt_SET((int32_t)1169545304, PH.base.pack) ;
    p124_eph_SET((uint16_t)(uint16_t)20250, PH.base.pack) ;
    p124_epv_SET((uint16_t)(uint16_t)4850, PH.base.pack) ;
    p124_vel_SET((uint16_t)(uint16_t)5637, PH.base.pack) ;
    p124_cog_SET((uint16_t)(uint16_t)25639, PH.base.pack) ;
    p124_satellites_visible_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
    p124_dgps_numch_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
    p124_dgps_age_SET((uint32_t)1563984170L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
    p125_Vcc_SET((uint16_t)(uint16_t)33465, PH.base.pack) ;
    p125_Vservo_SET((uint16_t)(uint16_t)53753, PH.base.pack) ;
    p125_flags_SET((e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID |
                    e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED), PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
    p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2, PH.base.pack) ;
    p126_flags_SET((e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_BLOCKING |
                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE |
                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND), PH.base.pack) ;
    p126_timeout_SET((uint16_t)(uint16_t)189, PH.base.pack) ;
    p126_baudrate_SET((uint32_t)3260259215L, PH.base.pack) ;
    p126_count_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)125, (uint8_t)113, (uint8_t)59, (uint8_t)222, (uint8_t)245, (uint8_t)180, (uint8_t)90, (uint8_t)225, (uint8_t)22, (uint8_t)16, (uint8_t)0, (uint8_t)131, (uint8_t)14, (uint8_t)166, (uint8_t)34, (uint8_t)183, (uint8_t)161, (uint8_t)230, (uint8_t)194, (uint8_t)118, (uint8_t)214, (uint8_t)94, (uint8_t)214, (uint8_t)115, (uint8_t)26, (uint8_t)254, (uint8_t)126, (uint8_t)40, (uint8_t)191, (uint8_t)115, (uint8_t)98, (uint8_t)67, (uint8_t)178, (uint8_t)170, (uint8_t)26, (uint8_t)169, (uint8_t)89, (uint8_t)93, (uint8_t)53, (uint8_t)22, (uint8_t)3, (uint8_t)115, (uint8_t)56, (uint8_t)35, (uint8_t)149, (uint8_t)227, (uint8_t)160, (uint8_t)207, (uint8_t)237, (uint8_t)55, (uint8_t)47, (uint8_t)36, (uint8_t)74, (uint8_t)148, (uint8_t)134, (uint8_t)170, (uint8_t)153, (uint8_t)144, (uint8_t)65, (uint8_t)183, (uint8_t)75, (uint8_t)189, (uint8_t)64, (uint8_t)138, (uint8_t)21, (uint8_t)190, (uint8_t)6, (uint8_t)129, (uint8_t)56, (uint8_t)166};
        p126_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
    p127_time_last_baseline_ms_SET((uint32_t)3934172627L, PH.base.pack) ;
    p127_rtk_receiver_id_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
    p127_wn_SET((uint16_t)(uint16_t)10556, PH.base.pack) ;
    p127_tow_SET((uint32_t)3879439764L, PH.base.pack) ;
    p127_rtk_health_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
    p127_rtk_rate_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
    p127_nsats_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    p127_baseline_coords_type_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
    p127_baseline_a_mm_SET((int32_t) -1986589874, PH.base.pack) ;
    p127_baseline_b_mm_SET((int32_t) -310505040, PH.base.pack) ;
    p127_baseline_c_mm_SET((int32_t)1971648983, PH.base.pack) ;
    p127_accuracy_SET((uint32_t)3929214396L, PH.base.pack) ;
    p127_iar_num_hypotheses_SET((int32_t)1763979879, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
    p128_time_last_baseline_ms_SET((uint32_t)2076307731L, PH.base.pack) ;
    p128_rtk_receiver_id_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
    p128_wn_SET((uint16_t)(uint16_t)1755, PH.base.pack) ;
    p128_tow_SET((uint32_t)2948953473L, PH.base.pack) ;
    p128_rtk_health_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
    p128_rtk_rate_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
    p128_nsats_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
    p128_baseline_coords_type_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
    p128_baseline_a_mm_SET((int32_t) -1865607041, PH.base.pack) ;
    p128_baseline_b_mm_SET((int32_t) -407695700, PH.base.pack) ;
    p128_baseline_c_mm_SET((int32_t)935906508, PH.base.pack) ;
    p128_accuracy_SET((uint32_t)2995331743L, PH.base.pack) ;
    p128_iar_num_hypotheses_SET((int32_t)1466535805, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
    p129_time_boot_ms_SET((uint32_t)2693793090L, PH.base.pack) ;
    p129_xacc_SET((int16_t)(int16_t)29913, PH.base.pack) ;
    p129_yacc_SET((int16_t)(int16_t) -7295, PH.base.pack) ;
    p129_zacc_SET((int16_t)(int16_t) -32514, PH.base.pack) ;
    p129_xgyro_SET((int16_t)(int16_t)29993, PH.base.pack) ;
    p129_ygyro_SET((int16_t)(int16_t) -23098, PH.base.pack) ;
    p129_zgyro_SET((int16_t)(int16_t) -10408, PH.base.pack) ;
    p129_xmag_SET((int16_t)(int16_t) -22669, PH.base.pack) ;
    p129_ymag_SET((int16_t)(int16_t) -17629, PH.base.pack) ;
    p129_zmag_SET((int16_t)(int16_t) -16790, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
    p130_type_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
    p130_size_SET((uint32_t)3457762029L, PH.base.pack) ;
    p130_width_SET((uint16_t)(uint16_t)49672, PH.base.pack) ;
    p130_height_SET((uint16_t)(uint16_t)11810, PH.base.pack) ;
    p130_packets_SET((uint16_t)(uint16_t)19945, PH.base.pack) ;
    p130_payload_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
    p130_jpg_quality_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
    p131_seqnr_SET((uint16_t)(uint16_t)10798, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)97, (uint8_t)247, (uint8_t)195, (uint8_t)141, (uint8_t)150, (uint8_t)63, (uint8_t)107, (uint8_t)169, (uint8_t)2, (uint8_t)134, (uint8_t)228, (uint8_t)120, (uint8_t)132, (uint8_t)25, (uint8_t)54, (uint8_t)238, (uint8_t)219, (uint8_t)66, (uint8_t)146, (uint8_t)229, (uint8_t)82, (uint8_t)224, (uint8_t)140, (uint8_t)120, (uint8_t)4, (uint8_t)78, (uint8_t)245, (uint8_t)91, (uint8_t)86, (uint8_t)31, (uint8_t)190, (uint8_t)73, (uint8_t)234, (uint8_t)77, (uint8_t)37, (uint8_t)88, (uint8_t)51, (uint8_t)82, (uint8_t)111, (uint8_t)169, (uint8_t)5, (uint8_t)212, (uint8_t)51, (uint8_t)131, (uint8_t)192, (uint8_t)161, (uint8_t)37, (uint8_t)206, (uint8_t)30, (uint8_t)18, (uint8_t)33, (uint8_t)146, (uint8_t)147, (uint8_t)33, (uint8_t)134, (uint8_t)176, (uint8_t)182, (uint8_t)153, (uint8_t)50, (uint8_t)246, (uint8_t)21, (uint8_t)117, (uint8_t)114, (uint8_t)15, (uint8_t)207, (uint8_t)106, (uint8_t)99, (uint8_t)57, (uint8_t)166, (uint8_t)145, (uint8_t)161, (uint8_t)136, (uint8_t)17, (uint8_t)85, (uint8_t)54, (uint8_t)184, (uint8_t)49, (uint8_t)79, (uint8_t)16, (uint8_t)48, (uint8_t)243, (uint8_t)72, (uint8_t)166, (uint8_t)217, (uint8_t)111, (uint8_t)220, (uint8_t)252, (uint8_t)199, (uint8_t)238, (uint8_t)195, (uint8_t)196, (uint8_t)41, (uint8_t)207, (uint8_t)42, (uint8_t)142, (uint8_t)115, (uint8_t)226, (uint8_t)205, (uint8_t)44, (uint8_t)44, (uint8_t)70, (uint8_t)229, (uint8_t)46, (uint8_t)193, (uint8_t)237, (uint8_t)181, (uint8_t)101, (uint8_t)33, (uint8_t)222, (uint8_t)210, (uint8_t)8, (uint8_t)157, (uint8_t)185, (uint8_t)105, (uint8_t)24, (uint8_t)160, (uint8_t)235, (uint8_t)194, (uint8_t)165, (uint8_t)53, (uint8_t)187, (uint8_t)183, (uint8_t)124, (uint8_t)179, (uint8_t)145, (uint8_t)7, (uint8_t)20, (uint8_t)104, (uint8_t)222, (uint8_t)133, (uint8_t)94, (uint8_t)76, (uint8_t)228, (uint8_t)141, (uint8_t)82, (uint8_t)126, (uint8_t)42, (uint8_t)220, (uint8_t)49, (uint8_t)38, (uint8_t)43, (uint8_t)92, (uint8_t)240, (uint8_t)161, (uint8_t)226, (uint8_t)188, (uint8_t)232, (uint8_t)90, (uint8_t)156, (uint8_t)209, (uint8_t)242, (uint8_t)62, (uint8_t)66, (uint8_t)164, (uint8_t)59, (uint8_t)64, (uint8_t)81, (uint8_t)193, (uint8_t)143, (uint8_t)133, (uint8_t)71, (uint8_t)28, (uint8_t)107, (uint8_t)216, (uint8_t)235, (uint8_t)18, (uint8_t)125, (uint8_t)135, (uint8_t)145, (uint8_t)173, (uint8_t)133, (uint8_t)148, (uint8_t)234, (uint8_t)203, (uint8_t)67, (uint8_t)174, (uint8_t)59, (uint8_t)157, (uint8_t)118, (uint8_t)122, (uint8_t)223, (uint8_t)249, (uint8_t)102, (uint8_t)53, (uint8_t)186, (uint8_t)179, (uint8_t)80, (uint8_t)3, (uint8_t)151, (uint8_t)218, (uint8_t)152, (uint8_t)48, (uint8_t)169, (uint8_t)52, (uint8_t)67, (uint8_t)125, (uint8_t)115, (uint8_t)185, (uint8_t)255, (uint8_t)153, (uint8_t)252, (uint8_t)142, (uint8_t)125, (uint8_t)62, (uint8_t)230, (uint8_t)105, (uint8_t)25, (uint8_t)97, (uint8_t)163, (uint8_t)202, (uint8_t)115, (uint8_t)144, (uint8_t)50, (uint8_t)212, (uint8_t)88, (uint8_t)253, (uint8_t)63, (uint8_t)20, (uint8_t)72, (uint8_t)85, (uint8_t)95, (uint8_t)144, (uint8_t)176, (uint8_t)62, (uint8_t)179, (uint8_t)78, (uint8_t)207, (uint8_t)237, (uint8_t)205, (uint8_t)17, (uint8_t)4, (uint8_t)37, (uint8_t)38, (uint8_t)184, (uint8_t)220, (uint8_t)214, (uint8_t)134, (uint8_t)238, (uint8_t)28, (uint8_t)200, (uint8_t)12, (uint8_t)210, (uint8_t)95, (uint8_t)2, (uint8_t)21, (uint8_t)217, (uint8_t)165, (uint8_t)24, (uint8_t)124, (uint8_t)187, (uint8_t)6, (uint8_t)20, (uint8_t)63};
        p131_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
    p132_time_boot_ms_SET((uint32_t)1136523733L, PH.base.pack) ;
    p132_min_distance_SET((uint16_t)(uint16_t)18706, PH.base.pack) ;
    p132_max_distance_SET((uint16_t)(uint16_t)41179, PH.base.pack) ;
    p132_current_distance_SET((uint16_t)(uint16_t)65176, PH.base.pack) ;
    p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, PH.base.pack) ;
    p132_id_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
    p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90_YAW_90, PH.base.pack) ;
    p132_covariance_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
    p133_lat_SET((int32_t)2056406751, PH.base.pack) ;
    p133_lon_SET((int32_t)1088007706, PH.base.pack) ;
    p133_grid_spacing_SET((uint16_t)(uint16_t)45122, PH.base.pack) ;
    p133_mask_SET((uint64_t)2336420198701319342L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
    p134_lat_SET((int32_t)369085252, PH.base.pack) ;
    p134_lon_SET((int32_t)834681579, PH.base.pack) ;
    p134_grid_spacing_SET((uint16_t)(uint16_t)35570, PH.base.pack) ;
    p134_gridbit_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
    {
        int16_t  data_ [] =  {(int16_t) -20576, (int16_t) -3623, (int16_t)25821, (int16_t) -5047, (int16_t) -2358, (int16_t) -30351, (int16_t) -22260, (int16_t) -21273, (int16_t) -9877, (int16_t) -8821, (int16_t) -17797, (int16_t)14372, (int16_t) -13778, (int16_t) -10465, (int16_t)10798, (int16_t) -1496};
        p134_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
    p135_lat_SET((int32_t)1503647307, PH.base.pack) ;
    p135_lon_SET((int32_t)1214874247, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
    p136_lat_SET((int32_t) -1469884971, PH.base.pack) ;
    p136_lon_SET((int32_t)578431301, PH.base.pack) ;
    p136_spacing_SET((uint16_t)(uint16_t)39358, PH.base.pack) ;
    p136_terrain_height_SET((float)1.7853676E38F, PH.base.pack) ;
    p136_current_height_SET((float)3.116989E38F, PH.base.pack) ;
    p136_pending_SET((uint16_t)(uint16_t)7888, PH.base.pack) ;
    p136_loaded_SET((uint16_t)(uint16_t)23326, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
    p137_time_boot_ms_SET((uint32_t)2639052002L, PH.base.pack) ;
    p137_press_abs_SET((float) -3.376464E38F, PH.base.pack) ;
    p137_press_diff_SET((float) -1.271619E38F, PH.base.pack) ;
    p137_temperature_SET((int16_t)(int16_t) -103, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
    p138_time_usec_SET((uint64_t)154337630603578698L, PH.base.pack) ;
    {
        float  q [] =  {3.1540857E38F, 2.8674516E38F, -4.737075E37F, 2.7483742E38F};
        p138_q_SET(&q, 0, &PH.base.pack) ;
    }
    p138_x_SET((float)2.23161E38F, PH.base.pack) ;
    p138_y_SET((float)2.2114737E38F, PH.base.pack) ;
    p138_z_SET((float) -2.6960774E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
    p139_time_usec_SET((uint64_t)2889290145356492576L, PH.base.pack) ;
    p139_group_mlx_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
    p139_target_system_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
    p139_target_component_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
    {
        float  controls [] =  {-3.0216726E38F, 8.98125E37F, 3.093206E38F, -3.2822003E38F, 1.0725293E38F, 2.6930365E38F, -2.9301938E38F, 3.419911E37F};
        p139_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
    p140_time_usec_SET((uint64_t)7985952851786692168L, PH.base.pack) ;
    p140_group_mlx_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
    {
        float  controls [] =  {-2.132638E38F, -2.3990168E38F, -1.5976583E38F, 1.9893963E38F, -2.7304865E38F, 1.891946E38F, 1.0871689E38F, -1.3262533E37F};
        p140_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ALTITUDE_141(), &PH);
    p141_time_usec_SET((uint64_t)768519672184759496L, PH.base.pack) ;
    p141_altitude_monotonic_SET((float) -2.976669E38F, PH.base.pack) ;
    p141_altitude_amsl_SET((float) -7.5546373E37F, PH.base.pack) ;
    p141_altitude_local_SET((float)1.8772127E38F, PH.base.pack) ;
    p141_altitude_relative_SET((float)1.8194096E38F, PH.base.pack) ;
    p141_altitude_terrain_SET((float) -2.4156433E38F, PH.base.pack) ;
    p141_bottom_clearance_SET((float)5.610847E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RESOURCE_REQUEST_142(), &PH);
    p142_request_id_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
    p142_uri_type_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
    {
        uint8_t  uri [] =  {(uint8_t)207, (uint8_t)169, (uint8_t)241, (uint8_t)243, (uint8_t)43, (uint8_t)152, (uint8_t)173, (uint8_t)167, (uint8_t)96, (uint8_t)28, (uint8_t)2, (uint8_t)59, (uint8_t)24, (uint8_t)71, (uint8_t)57, (uint8_t)243, (uint8_t)197, (uint8_t)55, (uint8_t)228, (uint8_t)253, (uint8_t)59, (uint8_t)202, (uint8_t)50, (uint8_t)66, (uint8_t)129, (uint8_t)28, (uint8_t)137, (uint8_t)177, (uint8_t)167, (uint8_t)52, (uint8_t)98, (uint8_t)108, (uint8_t)0, (uint8_t)194, (uint8_t)127, (uint8_t)226, (uint8_t)45, (uint8_t)32, (uint8_t)90, (uint8_t)21, (uint8_t)200, (uint8_t)78, (uint8_t)178, (uint8_t)164, (uint8_t)197, (uint8_t)142, (uint8_t)47, (uint8_t)31, (uint8_t)242, (uint8_t)27, (uint8_t)15, (uint8_t)149, (uint8_t)186, (uint8_t)237, (uint8_t)156, (uint8_t)206, (uint8_t)169, (uint8_t)216, (uint8_t)43, (uint8_t)172, (uint8_t)251, (uint8_t)4, (uint8_t)57, (uint8_t)168, (uint8_t)225, (uint8_t)124, (uint8_t)88, (uint8_t)186, (uint8_t)48, (uint8_t)123, (uint8_t)110, (uint8_t)171, (uint8_t)31, (uint8_t)51, (uint8_t)74, (uint8_t)106, (uint8_t)190, (uint8_t)220, (uint8_t)124, (uint8_t)73, (uint8_t)48, (uint8_t)86, (uint8_t)49, (uint8_t)24, (uint8_t)189, (uint8_t)81, (uint8_t)197, (uint8_t)251, (uint8_t)20, (uint8_t)27, (uint8_t)169, (uint8_t)53, (uint8_t)252, (uint8_t)150, (uint8_t)86, (uint8_t)228, (uint8_t)236, (uint8_t)135, (uint8_t)126, (uint8_t)104, (uint8_t)217, (uint8_t)141, (uint8_t)89, (uint8_t)2, (uint8_t)129, (uint8_t)211, (uint8_t)215, (uint8_t)233, (uint8_t)245, (uint8_t)105, (uint8_t)79, (uint8_t)86, (uint8_t)131, (uint8_t)231, (uint8_t)145, (uint8_t)200, (uint8_t)124, (uint8_t)148, (uint8_t)160, (uint8_t)206};
        p142_uri_SET(&uri, 0, &PH.base.pack) ;
    }
    p142_transfer_type_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    {
        uint8_t  storage [] =  {(uint8_t)171, (uint8_t)148, (uint8_t)85, (uint8_t)234, (uint8_t)69, (uint8_t)99, (uint8_t)68, (uint8_t)161, (uint8_t)65, (uint8_t)201, (uint8_t)206, (uint8_t)203, (uint8_t)198, (uint8_t)20, (uint8_t)44, (uint8_t)1, (uint8_t)39, (uint8_t)216, (uint8_t)146, (uint8_t)236, (uint8_t)62, (uint8_t)15, (uint8_t)118, (uint8_t)11, (uint8_t)23, (uint8_t)193, (uint8_t)26, (uint8_t)0, (uint8_t)159, (uint8_t)139, (uint8_t)14, (uint8_t)34, (uint8_t)44, (uint8_t)221, (uint8_t)249, (uint8_t)227, (uint8_t)89, (uint8_t)135, (uint8_t)83, (uint8_t)23, (uint8_t)228, (uint8_t)30, (uint8_t)25, (uint8_t)147, (uint8_t)19, (uint8_t)222, (uint8_t)203, (uint8_t)78, (uint8_t)201, (uint8_t)82, (uint8_t)208, (uint8_t)53, (uint8_t)120, (uint8_t)44, (uint8_t)143, (uint8_t)74, (uint8_t)7, (uint8_t)81, (uint8_t)63, (uint8_t)34, (uint8_t)55, (uint8_t)95, (uint8_t)65, (uint8_t)27, (uint8_t)216, (uint8_t)99, (uint8_t)146, (uint8_t)235, (uint8_t)72, (uint8_t)240, (uint8_t)12, (uint8_t)240, (uint8_t)230, (uint8_t)216, (uint8_t)197, (uint8_t)9, (uint8_t)241, (uint8_t)39, (uint8_t)236, (uint8_t)43, (uint8_t)0, (uint8_t)61, (uint8_t)142, (uint8_t)107, (uint8_t)82, (uint8_t)165, (uint8_t)81, (uint8_t)149, (uint8_t)96, (uint8_t)24, (uint8_t)108, (uint8_t)170, (uint8_t)161, (uint8_t)132, (uint8_t)230, (uint8_t)159, (uint8_t)46, (uint8_t)124, (uint8_t)65, (uint8_t)132, (uint8_t)51, (uint8_t)169, (uint8_t)178, (uint8_t)79, (uint8_t)212, (uint8_t)251, (uint8_t)37, (uint8_t)177, (uint8_t)234, (uint8_t)163, (uint8_t)227, (uint8_t)137, (uint8_t)240, (uint8_t)204, (uint8_t)52, (uint8_t)156, (uint8_t)86, (uint8_t)186, (uint8_t)157, (uint8_t)6};
        p142_storage_SET(&storage, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE3_143(), &PH);
    p143_time_boot_ms_SET((uint32_t)858870190L, PH.base.pack) ;
    p143_press_abs_SET((float) -2.2679764E38F, PH.base.pack) ;
    p143_press_diff_SET((float) -1.2795468E38F, PH.base.pack) ;
    p143_temperature_SET((int16_t)(int16_t)29925, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_FOLLOW_TARGET_144(), &PH);
    p144_timestamp_SET((uint64_t)7591292485515706513L, PH.base.pack) ;
    p144_est_capabilities_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
    p144_lat_SET((int32_t)499161478, PH.base.pack) ;
    p144_lon_SET((int32_t) -1849376186, PH.base.pack) ;
    p144_alt_SET((float)1.5292151E38F, PH.base.pack) ;
    {
        float  vel [] =  {-2.0873583E38F, -5.296516E37F, 8.681471E37F};
        p144_vel_SET(&vel, 0, &PH.base.pack) ;
    }
    {
        float  acc [] =  {2.7606826E38F, 1.017563E38F, -1.4740517E38F};
        p144_acc_SET(&acc, 0, &PH.base.pack) ;
    }
    {
        float  attitude_q [] =  {3.0735272E38F, 2.1929643E38F, 2.0806554E38F, 3.345139E38F};
        p144_attitude_q_SET(&attitude_q, 0, &PH.base.pack) ;
    }
    {
        float  rates [] =  {3.1352143E38F, 2.2137715E38F, -4.672339E37F};
        p144_rates_SET(&rates, 0, &PH.base.pack) ;
    }
    {
        float  position_cov [] =  {1.4452777E38F, 1.1381707E38F, 4.4087255E37F};
        p144_position_cov_SET(&position_cov, 0, &PH.base.pack) ;
    }
    p144_custom_state_SET((uint64_t)708683633355509442L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
    p146_time_usec_SET((uint64_t)7406191253518216630L, PH.base.pack) ;
    p146_x_acc_SET((float)2.0461822E38F, PH.base.pack) ;
    p146_y_acc_SET((float) -1.74489E38F, PH.base.pack) ;
    p146_z_acc_SET((float) -5.809625E37F, PH.base.pack) ;
    p146_x_vel_SET((float) -3.6619515E37F, PH.base.pack) ;
    p146_y_vel_SET((float)7.387581E36F, PH.base.pack) ;
    p146_z_vel_SET((float)2.5560692E38F, PH.base.pack) ;
    p146_x_pos_SET((float) -1.2891674E38F, PH.base.pack) ;
    p146_y_pos_SET((float)1.2680144E38F, PH.base.pack) ;
    p146_z_pos_SET((float)5.2833497E37F, PH.base.pack) ;
    p146_airspeed_SET((float) -2.7186883E38F, PH.base.pack) ;
    {
        float  vel_variance [] =  {1.6082574E38F, -4.6456885E37F, 1.1561391E38F};
        p146_vel_variance_SET(&vel_variance, 0, &PH.base.pack) ;
    }
    {
        float  pos_variance [] =  {-4.7310515E37F, -1.5672089E38F, -2.902237E38F};
        p146_pos_variance_SET(&pos_variance, 0, &PH.base.pack) ;
    }
    {
        float  q [] =  {2.0518955E38F, -1.971955E38F, 1.5007849E38F, 2.650412E38F};
        p146_q_SET(&q, 0, &PH.base.pack) ;
    }
    p146_roll_rate_SET((float)1.4857371E38F, PH.base.pack) ;
    p146_pitch_rate_SET((float)1.2541189E38F, PH.base.pack) ;
    p146_yaw_rate_SET((float) -3.266415E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_BATTERY_STATUS_147(), &PH);
    p147_id_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
    p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_AVIONICS, PH.base.pack) ;
    p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_NIMH, PH.base.pack) ;
    p147_temperature_SET((int16_t)(int16_t) -15138, PH.base.pack) ;
    {
        uint16_t  voltages [] =  {(uint16_t)54497, (uint16_t)1630, (uint16_t)40515, (uint16_t)48381, (uint16_t)61314, (uint16_t)34832, (uint16_t)56086, (uint16_t)39496, (uint16_t)44900, (uint16_t)22540};
        p147_voltages_SET(&voltages, 0, &PH.base.pack) ;
    }
    p147_current_battery_SET((int16_t)(int16_t) -10592, PH.base.pack) ;
    p147_current_consumed_SET((int32_t)266924364, PH.base.pack) ;
    p147_energy_consumed_SET((int32_t)2084483404, PH.base.pack) ;
    p147_battery_remaining_SET((int8_t)(int8_t) -72, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}
