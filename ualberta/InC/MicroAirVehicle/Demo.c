
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
void c_CommunicationChannel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
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
void c_CommunicationChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    e_MAV_CMD  command = p77_command_GET(pack);
    e_MAV_RESULT  result = p77_result_GET(pack);
    uint8_t  progress = p77_progress_TRY(ph);
    int32_t  result_param2 = p77_result_param2_TRY(ph);
    uint8_t  target_system = p77_target_system_TRY(ph);
    uint8_t  target_component = p77_target_component_TRY(ph);
}
void c_CommunicationChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    uint32_t  time_boot_ms = p81_time_boot_ms_GET(pack);
    float  roll = p81_roll_GET(pack);
    float  pitch = p81_pitch_GET(pack);
    float  yaw = p81_yaw_GET(pack);
    float  thrust = p81_thrust_GET(pack);
    uint8_t  mode_switch = p81_mode_switch_GET(pack);
    uint8_t  manual_override_switch = p81_manual_override_switch_GET(pack);
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
void c_CommunicationChannel_on_NAV_FILTER_BIAS_220(Bounds_Inside * ph, Pack * pack)
{
    uint64_t  usec = p220_usec_GET(pack);
    float  accel_0 = p220_accel_0_GET(pack);
    float  accel_1 = p220_accel_1_GET(pack);
    float  accel_2 = p220_accel_2_GET(pack);
    float  gyro_0 = p220_gyro_0_GET(pack);
    float  gyro_1 = p220_gyro_1_GET(pack);
    float  gyro_2 = p220_gyro_2_GET(pack);
}
void c_CommunicationChannel_on_RADIO_CALIBRATION_221(Bounds_Inside * ph, Pack * pack)
{
    uint16_t*  aileron = p221_aileron_GET_(pack);
//process data in aileron
    free(aileron);//never forget to dispose
    uint16_t*  elevator = p221_elevator_GET_(pack);
//process data in elevator
    free(elevator);//never forget to dispose
    uint16_t*  rudder = p221_rudder_GET_(pack);
//process data in rudder
    free(rudder);//never forget to dispose
    uint16_t*  gyro = p221_gyro_GET_(pack);
//process data in gyro
    free(gyro);//never forget to dispose
    uint16_t*  pitch = p221_pitch_GET_(pack);
//process data in pitch
    free(pitch);//never forget to dispose
    uint16_t*  throttle = p221_throttle_GET_(pack);
//process data in throttle
    free(throttle);//never forget to dispose
}
void c_CommunicationChannel_on_UALBERTA_SYS_STATUS_222(Bounds_Inside * ph, Pack * pack)
{
    uint8_t  mode = p222_mode_GET(pack);
    uint8_t  nav_mode = p222_nav_mode_GET(pack);
    uint8_t  pilot = p222_pilot_GET(pack);
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
    p0_type_SET(e_MAV_TYPE_MAV_TYPE_GCS, PH.base.pack) ;
    p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY, PH.base.pack) ;
    p0_base_mode_SET((e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED |
                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED |
                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED |
                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED |
                      e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED), PH.base.pack) ;
    p0_custom_mode_SET((uint32_t)3216381115L, PH.base.pack) ;
    p0_system_status_SET(e_MAV_STATE_MAV_STATE_UNINIT, PH.base.pack) ;
    p0_mavlink_version_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
    p1_onboard_control_sensors_present_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_BATTERY |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_REVERSE_MOTOR |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_LASER_POSITION |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL), PH.base.pack) ;
    p1_onboard_control_sensors_enabled_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_GPS |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL |
                                            e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE), PH.base.pack) ;
    p1_onboard_control_sensors_health_SET((e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2 |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2 |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_TERRAIN |
                                           e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE), PH.base.pack) ;
    p1_load_SET((uint16_t)(uint16_t)3814, PH.base.pack) ;
    p1_voltage_battery_SET((uint16_t)(uint16_t)22970, PH.base.pack) ;
    p1_current_battery_SET((int16_t)(int16_t) -20295, PH.base.pack) ;
    p1_battery_remaining_SET((int8_t)(int8_t) -43, PH.base.pack) ;
    p1_drop_rate_comm_SET((uint16_t)(uint16_t)13790, PH.base.pack) ;
    p1_errors_comm_SET((uint16_t)(uint16_t)3478, PH.base.pack) ;
    p1_errors_count1_SET((uint16_t)(uint16_t)2532, PH.base.pack) ;
    p1_errors_count2_SET((uint16_t)(uint16_t)56382, PH.base.pack) ;
    p1_errors_count3_SET((uint16_t)(uint16_t)44287, PH.base.pack) ;
    p1_errors_count4_SET((uint16_t)(uint16_t)53993, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
    p2_time_unix_usec_SET((uint64_t)5217947963841989353L, PH.base.pack) ;
    p2_time_boot_ms_SET((uint32_t)332894182L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
    p3_time_boot_ms_SET((uint32_t)633054683L, PH.base.pack) ;
    p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
    p3_type_mask_SET((uint16_t)(uint16_t)13888, PH.base.pack) ;
    p3_x_SET((float)2.2480023E38F, PH.base.pack) ;
    p3_y_SET((float)2.039061E38F, PH.base.pack) ;
    p3_z_SET((float)2.781697E38F, PH.base.pack) ;
    p3_vx_SET((float)1.5803396E38F, PH.base.pack) ;
    p3_vy_SET((float)1.1575759E37F, PH.base.pack) ;
    p3_vz_SET((float) -2.5129046E37F, PH.base.pack) ;
    p3_afx_SET((float)2.434888E38F, PH.base.pack) ;
    p3_afy_SET((float) -3.3322549E38F, PH.base.pack) ;
    p3_afz_SET((float)5.1684397E36F, PH.base.pack) ;
    p3_yaw_SET((float) -2.714866E37F, PH.base.pack) ;
    p3_yaw_rate_SET((float)3.132937E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PING_4(), &PH);
    p4_time_usec_SET((uint64_t)1479768462825499687L, PH.base.pack) ;
    p4_seq_SET((uint32_t)2093171681L, PH.base.pack) ;
    p4_target_system_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
    p4_target_component_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
    p5_target_system_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
    p5_control_request_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
    p5_version_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
    {
        char16_t   passkey = "Pmiumbhdwu";
        p5_passkey_SET(&passkey, 0,  sizeof(passkey), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
    p6_gcs_system_id_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
    p6_control_request_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
    p6_ack_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
    {
        char16_t   key = "qoupyyubfbUIjiqshxpm";
        p7_key_SET(&key, 0,  sizeof(key), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
    p11_target_system_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
    p11_base_mode_SET(e_MAV_MODE_MAV_MODE_TEST_ARMED, PH.base.pack) ;
    p11_custom_mode_SET((uint32_t)3136973905L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
    p20_target_system_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
    p20_target_component_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
    {
        char16_t   param_id = "ohoiaQgrbijbmrbO";
        p20_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p20_param_index_SET((int16_t)(int16_t) -13098, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
    p21_target_system_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
    p21_target_component_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
    {
        char16_t   param_id = "nchzFxjYdwswuc";
        p22_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p22_param_value_SET((float)4.995292E37F, PH.base.pack) ;
    p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64, PH.base.pack) ;
    p22_param_count_SET((uint16_t)(uint16_t)10157, PH.base.pack) ;
    p22_param_index_SET((uint16_t)(uint16_t)54580, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
    p23_target_system_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
    p23_target_component_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
    {
        char16_t   param_id = "lbpgsotpO";
        p23_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p23_param_value_SET((float)2.72348E36F, PH.base.pack) ;
    p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
    p24_time_usec_SET((uint64_t)4734729470283101963L, PH.base.pack) ;
    p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FIXED, PH.base.pack) ;
    p24_lat_SET((int32_t)1916795022, PH.base.pack) ;
    p24_lon_SET((int32_t) -1176059589, PH.base.pack) ;
    p24_alt_SET((int32_t) -1500981994, PH.base.pack) ;
    p24_eph_SET((uint16_t)(uint16_t)45346, PH.base.pack) ;
    p24_epv_SET((uint16_t)(uint16_t)33744, PH.base.pack) ;
    p24_vel_SET((uint16_t)(uint16_t)7422, PH.base.pack) ;
    p24_cog_SET((uint16_t)(uint16_t)16924, PH.base.pack) ;
    p24_satellites_visible_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
    p24_alt_ellipsoid_SET((int32_t)1010902256, &PH) ;
    p24_h_acc_SET((uint32_t)606705040L, &PH) ;
    p24_v_acc_SET((uint32_t)3048630058L, &PH) ;
    p24_vel_acc_SET((uint32_t)3670549772L, &PH) ;
    p24_hdg_acc_SET((uint32_t)3807355196L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
    p25_satellites_visible_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
    {
        uint8_t  satellite_prn [] =  {(uint8_t)229, (uint8_t)188, (uint8_t)136, (uint8_t)62, (uint8_t)26, (uint8_t)222, (uint8_t)247, (uint8_t)29, (uint8_t)24, (uint8_t)171, (uint8_t)128, (uint8_t)190, (uint8_t)230, (uint8_t)79, (uint8_t)114, (uint8_t)114, (uint8_t)227, (uint8_t)188, (uint8_t)140, (uint8_t)132};
        p25_satellite_prn_SET(&satellite_prn, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_used [] =  {(uint8_t)79, (uint8_t)177, (uint8_t)35, (uint8_t)97, (uint8_t)68, (uint8_t)22, (uint8_t)102, (uint8_t)199, (uint8_t)96, (uint8_t)155, (uint8_t)179, (uint8_t)175, (uint8_t)199, (uint8_t)239, (uint8_t)83, (uint8_t)239, (uint8_t)215, (uint8_t)129, (uint8_t)166, (uint8_t)145};
        p25_satellite_used_SET(&satellite_used, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_elevation [] =  {(uint8_t)132, (uint8_t)32, (uint8_t)157, (uint8_t)206, (uint8_t)92, (uint8_t)219, (uint8_t)86, (uint8_t)199, (uint8_t)207, (uint8_t)163, (uint8_t)58, (uint8_t)96, (uint8_t)35, (uint8_t)36, (uint8_t)118, (uint8_t)204, (uint8_t)130, (uint8_t)233, (uint8_t)215, (uint8_t)123};
        p25_satellite_elevation_SET(&satellite_elevation, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_azimuth [] =  {(uint8_t)224, (uint8_t)159, (uint8_t)37, (uint8_t)14, (uint8_t)146, (uint8_t)178, (uint8_t)167, (uint8_t)213, (uint8_t)88, (uint8_t)95, (uint8_t)147, (uint8_t)242, (uint8_t)9, (uint8_t)243, (uint8_t)144, (uint8_t)88, (uint8_t)225, (uint8_t)211, (uint8_t)12, (uint8_t)249};
        p25_satellite_azimuth_SET(&satellite_azimuth, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_snr [] =  {(uint8_t)230, (uint8_t)30, (uint8_t)46, (uint8_t)133, (uint8_t)244, (uint8_t)149, (uint8_t)32, (uint8_t)166, (uint8_t)142, (uint8_t)71, (uint8_t)185, (uint8_t)26, (uint8_t)123, (uint8_t)153, (uint8_t)115, (uint8_t)25, (uint8_t)136, (uint8_t)227, (uint8_t)130, (uint8_t)53};
        p25_satellite_snr_SET(&satellite_snr, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
    p26_time_boot_ms_SET((uint32_t)958084455L, PH.base.pack) ;
    p26_xacc_SET((int16_t)(int16_t)15522, PH.base.pack) ;
    p26_yacc_SET((int16_t)(int16_t)17204, PH.base.pack) ;
    p26_zacc_SET((int16_t)(int16_t) -29938, PH.base.pack) ;
    p26_xgyro_SET((int16_t)(int16_t)22139, PH.base.pack) ;
    p26_ygyro_SET((int16_t)(int16_t) -31307, PH.base.pack) ;
    p26_zgyro_SET((int16_t)(int16_t) -24893, PH.base.pack) ;
    p26_xmag_SET((int16_t)(int16_t) -6308, PH.base.pack) ;
    p26_ymag_SET((int16_t)(int16_t) -25005, PH.base.pack) ;
    p26_zmag_SET((int16_t)(int16_t)15956, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
    p27_time_usec_SET((uint64_t)8104992084835561568L, PH.base.pack) ;
    p27_xacc_SET((int16_t)(int16_t) -27627, PH.base.pack) ;
    p27_yacc_SET((int16_t)(int16_t) -8594, PH.base.pack) ;
    p27_zacc_SET((int16_t)(int16_t)13146, PH.base.pack) ;
    p27_xgyro_SET((int16_t)(int16_t)12093, PH.base.pack) ;
    p27_ygyro_SET((int16_t)(int16_t) -9888, PH.base.pack) ;
    p27_zgyro_SET((int16_t)(int16_t)10580, PH.base.pack) ;
    p27_xmag_SET((int16_t)(int16_t) -10248, PH.base.pack) ;
    p27_ymag_SET((int16_t)(int16_t)32234, PH.base.pack) ;
    p27_zmag_SET((int16_t)(int16_t) -245, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
    p28_time_usec_SET((uint64_t)6687426256422861681L, PH.base.pack) ;
    p28_press_abs_SET((int16_t)(int16_t)21713, PH.base.pack) ;
    p28_press_diff1_SET((int16_t)(int16_t)2881, PH.base.pack) ;
    p28_press_diff2_SET((int16_t)(int16_t)1822, PH.base.pack) ;
    p28_temperature_SET((int16_t)(int16_t) -25885, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
    p29_time_boot_ms_SET((uint32_t)1503792710L, PH.base.pack) ;
    p29_press_abs_SET((float) -1.6522436E38F, PH.base.pack) ;
    p29_press_diff_SET((float)2.0080706E37F, PH.base.pack) ;
    p29_temperature_SET((int16_t)(int16_t)1713, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
    p30_time_boot_ms_SET((uint32_t)2202153070L, PH.base.pack) ;
    p30_roll_SET((float) -1.5084811E38F, PH.base.pack) ;
    p30_pitch_SET((float)8.205509E37F, PH.base.pack) ;
    p30_yaw_SET((float)1.0590595E38F, PH.base.pack) ;
    p30_rollspeed_SET((float) -1.0230562E38F, PH.base.pack) ;
    p30_pitchspeed_SET((float)4.9611455E35F, PH.base.pack) ;
    p30_yawspeed_SET((float) -1.8104704E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
    p31_time_boot_ms_SET((uint32_t)1805137315L, PH.base.pack) ;
    p31_q1_SET((float)3.1967236E38F, PH.base.pack) ;
    p31_q2_SET((float) -2.8890877E38F, PH.base.pack) ;
    p31_q3_SET((float) -2.1268402E38F, PH.base.pack) ;
    p31_q4_SET((float) -2.0705113E38F, PH.base.pack) ;
    p31_rollspeed_SET((float) -3.190445E38F, PH.base.pack) ;
    p31_pitchspeed_SET((float)2.272454E38F, PH.base.pack) ;
    p31_yawspeed_SET((float)3.614422E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
    p32_time_boot_ms_SET((uint32_t)831555217L, PH.base.pack) ;
    p32_x_SET((float) -1.8047493E38F, PH.base.pack) ;
    p32_y_SET((float)8.423156E37F, PH.base.pack) ;
    p32_z_SET((float)2.1254363E38F, PH.base.pack) ;
    p32_vx_SET((float) -2.315799E38F, PH.base.pack) ;
    p32_vy_SET((float)1.5703729E38F, PH.base.pack) ;
    p32_vz_SET((float) -3.2781083E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
    p33_time_boot_ms_SET((uint32_t)4127245095L, PH.base.pack) ;
    p33_lat_SET((int32_t)855603969, PH.base.pack) ;
    p33_lon_SET((int32_t)1006860083, PH.base.pack) ;
    p33_alt_SET((int32_t) -1822818286, PH.base.pack) ;
    p33_relative_alt_SET((int32_t) -1407424499, PH.base.pack) ;
    p33_vx_SET((int16_t)(int16_t)18788, PH.base.pack) ;
    p33_vy_SET((int16_t)(int16_t)25210, PH.base.pack) ;
    p33_vz_SET((int16_t)(int16_t) -883, PH.base.pack) ;
    p33_hdg_SET((uint16_t)(uint16_t)16903, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
    p34_time_boot_ms_SET((uint32_t)2600894251L, PH.base.pack) ;
    p34_port_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
    p34_chan1_scaled_SET((int16_t)(int16_t) -5762, PH.base.pack) ;
    p34_chan2_scaled_SET((int16_t)(int16_t) -17956, PH.base.pack) ;
    p34_chan3_scaled_SET((int16_t)(int16_t) -17377, PH.base.pack) ;
    p34_chan4_scaled_SET((int16_t)(int16_t) -19504, PH.base.pack) ;
    p34_chan5_scaled_SET((int16_t)(int16_t) -8228, PH.base.pack) ;
    p34_chan6_scaled_SET((int16_t)(int16_t)25538, PH.base.pack) ;
    p34_chan7_scaled_SET((int16_t)(int16_t) -9152, PH.base.pack) ;
    p34_chan8_scaled_SET((int16_t)(int16_t)25753, PH.base.pack) ;
    p34_rssi_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
    p35_time_boot_ms_SET((uint32_t)4280902985L, PH.base.pack) ;
    p35_port_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    p35_chan1_raw_SET((uint16_t)(uint16_t)25887, PH.base.pack) ;
    p35_chan2_raw_SET((uint16_t)(uint16_t)22450, PH.base.pack) ;
    p35_chan3_raw_SET((uint16_t)(uint16_t)50604, PH.base.pack) ;
    p35_chan4_raw_SET((uint16_t)(uint16_t)52532, PH.base.pack) ;
    p35_chan5_raw_SET((uint16_t)(uint16_t)32171, PH.base.pack) ;
    p35_chan6_raw_SET((uint16_t)(uint16_t)61, PH.base.pack) ;
    p35_chan7_raw_SET((uint16_t)(uint16_t)17391, PH.base.pack) ;
    p35_chan8_raw_SET((uint16_t)(uint16_t)18756, PH.base.pack) ;
    p35_rssi_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
    p36_time_usec_SET((uint32_t)3051120845L, PH.base.pack) ;
    p36_port_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
    p36_servo1_raw_SET((uint16_t)(uint16_t)44471, PH.base.pack) ;
    p36_servo2_raw_SET((uint16_t)(uint16_t)22425, PH.base.pack) ;
    p36_servo3_raw_SET((uint16_t)(uint16_t)23676, PH.base.pack) ;
    p36_servo4_raw_SET((uint16_t)(uint16_t)4847, PH.base.pack) ;
    p36_servo5_raw_SET((uint16_t)(uint16_t)20315, PH.base.pack) ;
    p36_servo6_raw_SET((uint16_t)(uint16_t)9269, PH.base.pack) ;
    p36_servo7_raw_SET((uint16_t)(uint16_t)27637, PH.base.pack) ;
    p36_servo8_raw_SET((uint16_t)(uint16_t)7892, PH.base.pack) ;
    p36_servo9_raw_SET((uint16_t)(uint16_t)36698, &PH) ;
    p36_servo10_raw_SET((uint16_t)(uint16_t)64328, &PH) ;
    p36_servo11_raw_SET((uint16_t)(uint16_t)51990, &PH) ;
    p36_servo12_raw_SET((uint16_t)(uint16_t)60174, &PH) ;
    p36_servo13_raw_SET((uint16_t)(uint16_t)51071, &PH) ;
    p36_servo14_raw_SET((uint16_t)(uint16_t)52303, &PH) ;
    p36_servo15_raw_SET((uint16_t)(uint16_t)51760, &PH) ;
    p36_servo16_raw_SET((uint16_t)(uint16_t)53744, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
    p37_target_system_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
    p37_target_component_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
    p37_start_index_SET((int16_t)(int16_t) -30801, PH.base.pack) ;
    p37_end_index_SET((int16_t)(int16_t) -8218, PH.base.pack) ;
    p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
    p38_target_system_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
    p38_target_component_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
    p38_start_index_SET((int16_t)(int16_t) -5520, PH.base.pack) ;
    p38_end_index_SET((int16_t)(int16_t) -17880, PH.base.pack) ;
    p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
    p39_target_system_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
    p39_target_component_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
    p39_seq_SET((uint16_t)(uint16_t)15595, PH.base.pack) ;
    p39_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
    p39_command_SET(e_MAV_CMD_MAV_CMD_START_RX_PAIR, PH.base.pack) ;
    p39_current_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
    p39_autocontinue_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
    p39_param1_SET((float)3.1231197E38F, PH.base.pack) ;
    p39_param2_SET((float)1.8711971E38F, PH.base.pack) ;
    p39_param3_SET((float) -9.10352E37F, PH.base.pack) ;
    p39_param4_SET((float) -2.4549001E38F, PH.base.pack) ;
    p39_x_SET((float)1.1556237E38F, PH.base.pack) ;
    p39_y_SET((float) -2.9596928E38F, PH.base.pack) ;
    p39_z_SET((float) -2.8235009E38F, PH.base.pack) ;
    p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
    p40_target_system_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
    p40_target_component_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
    p40_seq_SET((uint16_t)(uint16_t)59212, PH.base.pack) ;
    p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
    p41_target_system_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
    p41_target_component_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
    p41_seq_SET((uint16_t)(uint16_t)49510, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
    p42_seq_SET((uint16_t)(uint16_t)25401, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
    p43_target_system_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
    p43_target_component_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
    p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
    p44_target_system_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
    p44_target_component_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
    p44_count_SET((uint16_t)(uint16_t)8005, PH.base.pack) ;
    p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
    p45_target_system_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
    p45_target_component_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
    p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
    p46_seq_SET((uint16_t)(uint16_t)63004, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
    p47_target_system_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
    p47_target_component_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
    p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_UNSUPPORTED_FRAME, PH.base.pack) ;
    p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
    p48_target_system_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
    p48_latitude_SET((int32_t) -895899808, PH.base.pack) ;
    p48_longitude_SET((int32_t) -3401728, PH.base.pack) ;
    p48_altitude_SET((int32_t) -216757664, PH.base.pack) ;
    p48_time_usec_SET((uint64_t)2245073568319566666L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
    p49_latitude_SET((int32_t)290740174, PH.base.pack) ;
    p49_longitude_SET((int32_t)312263707, PH.base.pack) ;
    p49_altitude_SET((int32_t)221205005, PH.base.pack) ;
    p49_time_usec_SET((uint64_t)1114103420939242125L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
    p50_target_system_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
    p50_target_component_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
    {
        char16_t   param_id = "fvhsjfegwn";
        p50_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p50_param_index_SET((int16_t)(int16_t)27427, PH.base.pack) ;
    p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
    p50_param_value0_SET((float) -1.6815183E38F, PH.base.pack) ;
    p50_scale_SET((float) -1.3525053E38F, PH.base.pack) ;
    p50_param_value_min_SET((float)2.480518E37F, PH.base.pack) ;
    p50_param_value_max_SET((float)4.254301E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
    p51_target_system_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
    p51_target_component_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
    p51_seq_SET((uint16_t)(uint16_t)24230, PH.base.pack) ;
    p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
    p54_target_system_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
    p54_target_component_SET((uint8_t)(uint8_t)3, PH.base.pack) ;
    p54_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
    p54_p1x_SET((float)1.9813042E38F, PH.base.pack) ;
    p54_p1y_SET((float) -2.3980814E38F, PH.base.pack) ;
    p54_p1z_SET((float) -1.2313909E38F, PH.base.pack) ;
    p54_p2x_SET((float)2.3882231E38F, PH.base.pack) ;
    p54_p2y_SET((float) -8.585758E37F, PH.base.pack) ;
    p54_p2z_SET((float)3.0941422E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
    p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
    p55_p1x_SET((float)1.0420477E38F, PH.base.pack) ;
    p55_p1y_SET((float) -2.1551265E38F, PH.base.pack) ;
    p55_p1z_SET((float)2.0438408E38F, PH.base.pack) ;
    p55_p2x_SET((float) -8.535802E37F, PH.base.pack) ;
    p55_p2y_SET((float)2.9940155E38F, PH.base.pack) ;
    p55_p2z_SET((float) -7.5858595E36F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
    p61_time_usec_SET((uint64_t)340148096160705904L, PH.base.pack) ;
    {
        float  q [] =  {6.6152483E37F, -2.8170914E38F, -5.4256394E37F, -9.151324E37F};
        p61_q_SET(&q, 0, &PH.base.pack) ;
    }
    p61_rollspeed_SET((float)4.0600161E37F, PH.base.pack) ;
    p61_pitchspeed_SET((float)7.7069156E37F, PH.base.pack) ;
    p61_yawspeed_SET((float) -9.773593E37F, PH.base.pack) ;
    {
        float  covariance [] =  {1.075257E38F, -2.6367766E37F, 4.1020543E36F, -2.7014523E38F, 1.5750498E38F, 1.0898306E38F, -3.166457E38F, -2.6156605E38F, -1.452675E36F};
        p61_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
    p62_nav_roll_SET((float)2.7445877E38F, PH.base.pack) ;
    p62_nav_pitch_SET((float) -3.2075196E37F, PH.base.pack) ;
    p62_nav_bearing_SET((int16_t)(int16_t)292, PH.base.pack) ;
    p62_target_bearing_SET((int16_t)(int16_t)27391, PH.base.pack) ;
    p62_wp_dist_SET((uint16_t)(uint16_t)4281, PH.base.pack) ;
    p62_alt_error_SET((float) -2.7844422E38F, PH.base.pack) ;
    p62_aspd_error_SET((float)2.2664433E38F, PH.base.pack) ;
    p62_xtrack_error_SET((float) -2.7343692E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
    p63_time_usec_SET((uint64_t)141165716654026047L, PH.base.pack) ;
    p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, PH.base.pack) ;
    p63_lat_SET((int32_t)1797859049, PH.base.pack) ;
    p63_lon_SET((int32_t)2000245979, PH.base.pack) ;
    p63_alt_SET((int32_t) -1495981330, PH.base.pack) ;
    p63_relative_alt_SET((int32_t) -1967297372, PH.base.pack) ;
    p63_vx_SET((float)2.8055124E38F, PH.base.pack) ;
    p63_vy_SET((float)1.4886969E38F, PH.base.pack) ;
    p63_vz_SET((float) -2.8779591E38F, PH.base.pack) ;
    {
        float  covariance [] =  {8.104749E37F, 1.2524885E38F, 2.5794276E38F, -1.2751398E38F, -2.5387308E38F, -1.1266743E37F, -3.3851877E38F, -2.4833813E38F, -2.7645046E38F, -1.5466555E38F, -2.0593521E38F, 1.0964467E38F, -1.0061144E38F, 2.3524926E38F, -2.7112489E38F, -1.6626627E38F, 2.479949E38F, -2.988941E38F, 3.3347336E38F, -1.6765892E38F, 8.983619E37F, 2.60574E38F, -5.193236E37F, -2.562448E38F, -2.5618748E38F, 1.0920993E38F, 1.3613574E38F, -2.458234E37F, 7.8385844E37F, 1.6424837E38F, 9.99633E37F, -2.2166684E38F, -1.7437274E38F, 2.7397094E38F, 1.5799132E38F, 2.7262211E38F};
        p63_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
    p64_time_usec_SET((uint64_t)3065429413666768856L, PH.base.pack) ;
    p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, PH.base.pack) ;
    p64_x_SET((float)2.9435182E38F, PH.base.pack) ;
    p64_y_SET((float) -4.4349334E37F, PH.base.pack) ;
    p64_z_SET((float)1.2191157E38F, PH.base.pack) ;
    p64_vx_SET((float) -1.0713045E38F, PH.base.pack) ;
    p64_vy_SET((float) -7.943966E37F, PH.base.pack) ;
    p64_vz_SET((float) -9.823025E37F, PH.base.pack) ;
    p64_ax_SET((float)1.733242E38F, PH.base.pack) ;
    p64_ay_SET((float)1.6847664E38F, PH.base.pack) ;
    p64_az_SET((float) -1.1488105E38F, PH.base.pack) ;
    {
        float  covariance [] =  {2.2475806E38F, -3.0432165E36F, -2.1158804E38F, -2.3397412E38F, 3.2337071E38F, 3.5847024E37F, -2.6009255E38F, -2.5628703E38F, 5.6623067E37F, -2.9340033E38F, 2.586149E38F, -1.9945766E38F, 1.4016933E38F, 3.039884E37F, -2.7900216E37F, 1.6468573E38F, 3.0419112E37F, 2.4688666E38F, 1.2172406E38F, -2.338965E38F, 2.7916865E38F, -1.5195881E38F, 1.9228199E38F, -2.8519962E38F, -2.876049E38F, 2.3072914E38F, 2.0929321E38F, -2.7724585E38F, 3.4001178E38F, 3.0739183E37F, -2.3263007E38F, -7.341743E36F, -2.7535764E38F, 1.4658312E37F, 6.2522105E37F, -2.2183695E38F, 2.6859474E38F, 2.200448E38F, -2.3343226E38F, -6.6241336E37F, 2.5168866E38F, 2.691112E38F, 1.900162E38F, -1.3428966E38F, 2.91366E38F};
        p64_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
    p65_time_boot_ms_SET((uint32_t)1466744401L, PH.base.pack) ;
    p65_chancount_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
    p65_chan1_raw_SET((uint16_t)(uint16_t)54958, PH.base.pack) ;
    p65_chan2_raw_SET((uint16_t)(uint16_t)31774, PH.base.pack) ;
    p65_chan3_raw_SET((uint16_t)(uint16_t)36583, PH.base.pack) ;
    p65_chan4_raw_SET((uint16_t)(uint16_t)26297, PH.base.pack) ;
    p65_chan5_raw_SET((uint16_t)(uint16_t)53850, PH.base.pack) ;
    p65_chan6_raw_SET((uint16_t)(uint16_t)50865, PH.base.pack) ;
    p65_chan7_raw_SET((uint16_t)(uint16_t)10409, PH.base.pack) ;
    p65_chan8_raw_SET((uint16_t)(uint16_t)18420, PH.base.pack) ;
    p65_chan9_raw_SET((uint16_t)(uint16_t)50341, PH.base.pack) ;
    p65_chan10_raw_SET((uint16_t)(uint16_t)55544, PH.base.pack) ;
    p65_chan11_raw_SET((uint16_t)(uint16_t)40092, PH.base.pack) ;
    p65_chan12_raw_SET((uint16_t)(uint16_t)28683, PH.base.pack) ;
    p65_chan13_raw_SET((uint16_t)(uint16_t)22381, PH.base.pack) ;
    p65_chan14_raw_SET((uint16_t)(uint16_t)55922, PH.base.pack) ;
    p65_chan15_raw_SET((uint16_t)(uint16_t)50560, PH.base.pack) ;
    p65_chan16_raw_SET((uint16_t)(uint16_t)55007, PH.base.pack) ;
    p65_chan17_raw_SET((uint16_t)(uint16_t)51764, PH.base.pack) ;
    p65_chan18_raw_SET((uint16_t)(uint16_t)43760, PH.base.pack) ;
    p65_rssi_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
    p66_target_system_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
    p66_target_component_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
    p66_req_stream_id_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
    p66_req_message_rate_SET((uint16_t)(uint16_t)49757, PH.base.pack) ;
    p66_start_stop_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
    p67_stream_id_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    p67_message_rate_SET((uint16_t)(uint16_t)56464, PH.base.pack) ;
    p67_on_off_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
    p69_target_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
    p69_x_SET((int16_t)(int16_t)32611, PH.base.pack) ;
    p69_y_SET((int16_t)(int16_t) -15064, PH.base.pack) ;
    p69_z_SET((int16_t)(int16_t)7548, PH.base.pack) ;
    p69_r_SET((int16_t)(int16_t)595, PH.base.pack) ;
    p69_buttons_SET((uint16_t)(uint16_t)19106, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
    p70_target_system_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
    p70_target_component_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
    p70_chan1_raw_SET((uint16_t)(uint16_t)41200, PH.base.pack) ;
    p70_chan2_raw_SET((uint16_t)(uint16_t)22346, PH.base.pack) ;
    p70_chan3_raw_SET((uint16_t)(uint16_t)19737, PH.base.pack) ;
    p70_chan4_raw_SET((uint16_t)(uint16_t)36287, PH.base.pack) ;
    p70_chan5_raw_SET((uint16_t)(uint16_t)3774, PH.base.pack) ;
    p70_chan6_raw_SET((uint16_t)(uint16_t)55936, PH.base.pack) ;
    p70_chan7_raw_SET((uint16_t)(uint16_t)24878, PH.base.pack) ;
    p70_chan8_raw_SET((uint16_t)(uint16_t)12156, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
    p73_target_system_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
    p73_target_component_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
    p73_seq_SET((uint16_t)(uint16_t)43411, PH.base.pack) ;
    p73_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
    p73_command_SET(e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_SETTINGS, PH.base.pack) ;
    p73_current_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
    p73_autocontinue_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
    p73_param1_SET((float)2.5957156E38F, PH.base.pack) ;
    p73_param2_SET((float)2.8508353E38F, PH.base.pack) ;
    p73_param3_SET((float)1.7302498E38F, PH.base.pack) ;
    p73_param4_SET((float)2.3700333E37F, PH.base.pack) ;
    p73_x_SET((int32_t)1365890479, PH.base.pack) ;
    p73_y_SET((int32_t) -364546134, PH.base.pack) ;
    p73_z_SET((float)6.85644E37F, PH.base.pack) ;
    p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
    p74_airspeed_SET((float) -1.7465826E38F, PH.base.pack) ;
    p74_groundspeed_SET((float)1.7353429E38F, PH.base.pack) ;
    p74_heading_SET((int16_t)(int16_t) -14175, PH.base.pack) ;
    p74_throttle_SET((uint16_t)(uint16_t)36628, PH.base.pack) ;
    p74_alt_SET((float) -2.9345168E38F, PH.base.pack) ;
    p74_climb_SET((float)1.1600296E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
    p75_target_system_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    p75_target_component_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
    p75_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
    p75_command_SET(e_MAV_CMD_MAV_CMD_OVERRIDE_GOTO, PH.base.pack) ;
    p75_current_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
    p75_autocontinue_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
    p75_param1_SET((float)1.1647496E38F, PH.base.pack) ;
    p75_param2_SET((float)9.795089E37F, PH.base.pack) ;
    p75_param3_SET((float)3.219588E38F, PH.base.pack) ;
    p75_param4_SET((float) -1.6889033E38F, PH.base.pack) ;
    p75_x_SET((int32_t)539983529, PH.base.pack) ;
    p75_y_SET((int32_t) -1748995489, PH.base.pack) ;
    p75_z_SET((float)2.6087862E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
    p76_target_system_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
    p76_target_component_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
    p76_command_SET(e_MAV_CMD_MAV_CMD_NAV_DELAY, PH.base.pack) ;
    p76_confirmation_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
    p76_param1_SET((float)9.035446E37F, PH.base.pack) ;
    p76_param2_SET((float) -1.8673283E37F, PH.base.pack) ;
    p76_param3_SET((float) -2.756719E38F, PH.base.pack) ;
    p76_param4_SET((float)1.9967055E38F, PH.base.pack) ;
    p76_param5_SET((float) -2.9721512E38F, PH.base.pack) ;
    p76_param6_SET((float)9.556492E37F, PH.base.pack) ;
    p76_param7_SET((float) -2.5120888E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
    p77_command_SET(e_MAV_CMD_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, PH.base.pack) ;
    p77_result_SET(e_MAV_RESULT_MAV_RESULT_TEMPORARILY_REJECTED, PH.base.pack) ;
    p77_progress_SET((uint8_t)(uint8_t)33, &PH) ;
    p77_result_param2_SET((int32_t) -1792162075, &PH) ;
    p77_target_system_SET((uint8_t)(uint8_t)33, &PH) ;
    p77_target_component_SET((uint8_t)(uint8_t)8, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
    p81_time_boot_ms_SET((uint32_t)2810712198L, PH.base.pack) ;
    p81_roll_SET((float)3.0227238E38F, PH.base.pack) ;
    p81_pitch_SET((float) -2.533393E38F, PH.base.pack) ;
    p81_yaw_SET((float) -1.1728671E38F, PH.base.pack) ;
    p81_thrust_SET((float) -2.6500353E38F, PH.base.pack) ;
    p81_mode_switch_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
    p81_manual_override_switch_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
    p82_time_boot_ms_SET((uint32_t)2014981986L, PH.base.pack) ;
    p82_target_system_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
    p82_target_component_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
    p82_type_mask_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
    {
        float  q [] =  {1.9382428E36F, 1.4207334E37F, 6.091885E37F, -1.6527438E38F};
        p82_q_SET(&q, 0, &PH.base.pack) ;
    }
    p82_body_roll_rate_SET((float) -1.6174364E38F, PH.base.pack) ;
    p82_body_pitch_rate_SET((float) -1.8332412E38F, PH.base.pack) ;
    p82_body_yaw_rate_SET((float) -2.8054532E38F, PH.base.pack) ;
    p82_thrust_SET((float) -1.661251E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
    p83_time_boot_ms_SET((uint32_t)950691124L, PH.base.pack) ;
    p83_type_mask_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
    {
        float  q [] =  {2.3652896E38F, 6.752904E37F, 2.3024173E37F, -2.3784508E38F};
        p83_q_SET(&q, 0, &PH.base.pack) ;
    }
    p83_body_roll_rate_SET((float)1.2130248E38F, PH.base.pack) ;
    p83_body_pitch_rate_SET((float)7.202863E37F, PH.base.pack) ;
    p83_body_yaw_rate_SET((float)4.890133E37F, PH.base.pack) ;
    p83_thrust_SET((float)1.3809921E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
    p84_time_boot_ms_SET((uint32_t)762835315L, PH.base.pack) ;
    p84_target_system_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
    p84_target_component_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
    p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
    p84_type_mask_SET((uint16_t)(uint16_t)11602, PH.base.pack) ;
    p84_x_SET((float)3.1949994E38F, PH.base.pack) ;
    p84_y_SET((float)7.7441607E37F, PH.base.pack) ;
    p84_z_SET((float) -4.786081E37F, PH.base.pack) ;
    p84_vx_SET((float) -1.0218424E38F, PH.base.pack) ;
    p84_vy_SET((float) -3.9348215E35F, PH.base.pack) ;
    p84_vz_SET((float) -3.3499708E38F, PH.base.pack) ;
    p84_afx_SET((float) -8.267119E37F, PH.base.pack) ;
    p84_afy_SET((float) -1.4943619E37F, PH.base.pack) ;
    p84_afz_SET((float) -1.7771528E38F, PH.base.pack) ;
    p84_yaw_SET((float)1.5737425E38F, PH.base.pack) ;
    p84_yaw_rate_SET((float)1.6573711E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
    p86_time_boot_ms_SET((uint32_t)1079857436L, PH.base.pack) ;
    p86_target_system_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
    p86_target_component_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
    p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
    p86_type_mask_SET((uint16_t)(uint16_t)32207, PH.base.pack) ;
    p86_lat_int_SET((int32_t)434227737, PH.base.pack) ;
    p86_lon_int_SET((int32_t)1821904091, PH.base.pack) ;
    p86_alt_SET((float) -2.4781796E38F, PH.base.pack) ;
    p86_vx_SET((float) -2.2667524E38F, PH.base.pack) ;
    p86_vy_SET((float)1.345068E38F, PH.base.pack) ;
    p86_vz_SET((float)6.46784E37F, PH.base.pack) ;
    p86_afx_SET((float) -2.7087395E38F, PH.base.pack) ;
    p86_afy_SET((float) -2.198512E38F, PH.base.pack) ;
    p86_afz_SET((float)7.931045E37F, PH.base.pack) ;
    p86_yaw_SET((float)8.537458E36F, PH.base.pack) ;
    p86_yaw_rate_SET((float) -1.2041514E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
    p87_time_boot_ms_SET((uint32_t)2798838376L, PH.base.pack) ;
    p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
    p87_type_mask_SET((uint16_t)(uint16_t)42027, PH.base.pack) ;
    p87_lat_int_SET((int32_t)1970758427, PH.base.pack) ;
    p87_lon_int_SET((int32_t)1626383694, PH.base.pack) ;
    p87_alt_SET((float)2.2067203E38F, PH.base.pack) ;
    p87_vx_SET((float) -2.0211393E37F, PH.base.pack) ;
    p87_vy_SET((float) -1.9172578E38F, PH.base.pack) ;
    p87_vz_SET((float)1.7867237E38F, PH.base.pack) ;
    p87_afx_SET((float)3.2845988E37F, PH.base.pack) ;
    p87_afy_SET((float)2.4577364E38F, PH.base.pack) ;
    p87_afz_SET((float)1.8813659E38F, PH.base.pack) ;
    p87_yaw_SET((float) -1.71369E38F, PH.base.pack) ;
    p87_yaw_rate_SET((float)3.2436617E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
    p89_time_boot_ms_SET((uint32_t)1890111511L, PH.base.pack) ;
    p89_x_SET((float) -3.008898E38F, PH.base.pack) ;
    p89_y_SET((float) -5.076574E37F, PH.base.pack) ;
    p89_z_SET((float) -1.5871897E38F, PH.base.pack) ;
    p89_roll_SET((float)1.80907E38F, PH.base.pack) ;
    p89_pitch_SET((float)4.418717E37F, PH.base.pack) ;
    p89_yaw_SET((float)3.3400346E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
    p90_time_usec_SET((uint64_t)3132721783879604660L, PH.base.pack) ;
    p90_roll_SET((float)1.276703E38F, PH.base.pack) ;
    p90_pitch_SET((float)2.4498035E38F, PH.base.pack) ;
    p90_yaw_SET((float)2.9720486E38F, PH.base.pack) ;
    p90_rollspeed_SET((float)3.0436244E38F, PH.base.pack) ;
    p90_pitchspeed_SET((float)6.417813E37F, PH.base.pack) ;
    p90_yawspeed_SET((float) -2.1086546E38F, PH.base.pack) ;
    p90_lat_SET((int32_t) -1867965568, PH.base.pack) ;
    p90_lon_SET((int32_t) -1745532368, PH.base.pack) ;
    p90_alt_SET((int32_t)759659626, PH.base.pack) ;
    p90_vx_SET((int16_t)(int16_t)14443, PH.base.pack) ;
    p90_vy_SET((int16_t)(int16_t) -24598, PH.base.pack) ;
    p90_vz_SET((int16_t)(int16_t)8654, PH.base.pack) ;
    p90_xacc_SET((int16_t)(int16_t) -7713, PH.base.pack) ;
    p90_yacc_SET((int16_t)(int16_t) -24416, PH.base.pack) ;
    p90_zacc_SET((int16_t)(int16_t)947, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
    p91_time_usec_SET((uint64_t)7331737469010093701L, PH.base.pack) ;
    p91_roll_ailerons_SET((float) -2.7560215E38F, PH.base.pack) ;
    p91_pitch_elevator_SET((float)1.2106883E38F, PH.base.pack) ;
    p91_yaw_rudder_SET((float)2.6609915E38F, PH.base.pack) ;
    p91_throttle_SET((float)1.3233306E38F, PH.base.pack) ;
    p91_aux1_SET((float)3.2294853E38F, PH.base.pack) ;
    p91_aux2_SET((float)1.2227424E38F, PH.base.pack) ;
    p91_aux3_SET((float) -3.029037E38F, PH.base.pack) ;
    p91_aux4_SET((float)6.821744E37F, PH.base.pack) ;
    p91_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_ARMED, PH.base.pack) ;
    p91_nav_mode_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
    p92_time_usec_SET((uint64_t)8478081646621887537L, PH.base.pack) ;
    p92_chan1_raw_SET((uint16_t)(uint16_t)58698, PH.base.pack) ;
    p92_chan2_raw_SET((uint16_t)(uint16_t)14881, PH.base.pack) ;
    p92_chan3_raw_SET((uint16_t)(uint16_t)9561, PH.base.pack) ;
    p92_chan4_raw_SET((uint16_t)(uint16_t)34052, PH.base.pack) ;
    p92_chan5_raw_SET((uint16_t)(uint16_t)31222, PH.base.pack) ;
    p92_chan6_raw_SET((uint16_t)(uint16_t)623, PH.base.pack) ;
    p92_chan7_raw_SET((uint16_t)(uint16_t)1509, PH.base.pack) ;
    p92_chan8_raw_SET((uint16_t)(uint16_t)59248, PH.base.pack) ;
    p92_chan9_raw_SET((uint16_t)(uint16_t)53628, PH.base.pack) ;
    p92_chan10_raw_SET((uint16_t)(uint16_t)2238, PH.base.pack) ;
    p92_chan11_raw_SET((uint16_t)(uint16_t)40353, PH.base.pack) ;
    p92_chan12_raw_SET((uint16_t)(uint16_t)16631, PH.base.pack) ;
    p92_rssi_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
    p93_time_usec_SET((uint64_t)3377579572228142588L, PH.base.pack) ;
    {
        float  controls [] =  {3.115052E37F, 2.335828E38F, 1.5603375E38F, -1.2763023E38F, -1.517666E38F, -1.122629E38F, 2.8062557E37F, -8.1499214E37F, 3.592239E37F, -1.0675769E38F, -8.4258154E37F, -2.7055596E35F, 2.9767815E38F, 3.3245845E38F, -1.6129236E38F, -4.641517E37F};
        p93_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    p93_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_ARMED, PH.base.pack) ;
    p93_flags_SET((uint64_t)4254304206573870158L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
    p100_time_usec_SET((uint64_t)1257793804726681302L, PH.base.pack) ;
    p100_sensor_id_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
    p100_flow_x_SET((int16_t)(int16_t)11249, PH.base.pack) ;
    p100_flow_y_SET((int16_t)(int16_t) -23177, PH.base.pack) ;
    p100_flow_comp_m_x_SET((float)4.085885E37F, PH.base.pack) ;
    p100_flow_comp_m_y_SET((float)7.584379E37F, PH.base.pack) ;
    p100_quality_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
    p100_ground_distance_SET((float) -1.8350653E37F, PH.base.pack) ;
    p100_flow_rate_x_SET((float) -1.5427681E38F, &PH) ;
    p100_flow_rate_y_SET((float) -1.7753964E38F, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
    p101_usec_SET((uint64_t)7427270529337260356L, PH.base.pack) ;
    p101_x_SET((float)8.0014704E37F, PH.base.pack) ;
    p101_y_SET((float)1.007623E38F, PH.base.pack) ;
    p101_z_SET((float) -2.6500473E38F, PH.base.pack) ;
    p101_roll_SET((float)2.0862067E38F, PH.base.pack) ;
    p101_pitch_SET((float)8.740831E37F, PH.base.pack) ;
    p101_yaw_SET((float)4.303384E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
    p102_usec_SET((uint64_t)4009797482575636330L, PH.base.pack) ;
    p102_x_SET((float) -1.7130677E38F, PH.base.pack) ;
    p102_y_SET((float) -2.150565E38F, PH.base.pack) ;
    p102_z_SET((float) -2.3434785E38F, PH.base.pack) ;
    p102_roll_SET((float) -1.793768E38F, PH.base.pack) ;
    p102_pitch_SET((float) -2.0080466E38F, PH.base.pack) ;
    p102_yaw_SET((float) -2.9892388E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
    p103_usec_SET((uint64_t)5754209549155967576L, PH.base.pack) ;
    p103_x_SET((float) -8.296917E37F, PH.base.pack) ;
    p103_y_SET((float)2.3208202E38F, PH.base.pack) ;
    p103_z_SET((float) -1.9253723E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
    p104_usec_SET((uint64_t)6656532359053443819L, PH.base.pack) ;
    p104_x_SET((float) -2.5923704E38F, PH.base.pack) ;
    p104_y_SET((float)3.2302826E38F, PH.base.pack) ;
    p104_z_SET((float)4.3064834E37F, PH.base.pack) ;
    p104_roll_SET((float)2.4227168E38F, PH.base.pack) ;
    p104_pitch_SET((float) -2.4889733E38F, PH.base.pack) ;
    p104_yaw_SET((float) -2.6585263E36F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
    p105_time_usec_SET((uint64_t)4260396827207647978L, PH.base.pack) ;
    p105_xacc_SET((float) -1.550892E38F, PH.base.pack) ;
    p105_yacc_SET((float)3.157959E36F, PH.base.pack) ;
    p105_zacc_SET((float) -2.9819251E38F, PH.base.pack) ;
    p105_xgyro_SET((float)1.6125706E38F, PH.base.pack) ;
    p105_ygyro_SET((float) -2.9635012E38F, PH.base.pack) ;
    p105_zgyro_SET((float) -7.583318E37F, PH.base.pack) ;
    p105_xmag_SET((float) -2.5953757E38F, PH.base.pack) ;
    p105_ymag_SET((float) -2.9237844E38F, PH.base.pack) ;
    p105_zmag_SET((float)2.448817E38F, PH.base.pack) ;
    p105_abs_pressure_SET((float)6.5710114E37F, PH.base.pack) ;
    p105_diff_pressure_SET((float)2.517384E38F, PH.base.pack) ;
    p105_pressure_alt_SET((float) -1.6964837E38F, PH.base.pack) ;
    p105_temperature_SET((float)3.3744291E38F, PH.base.pack) ;
    p105_fields_updated_SET((uint16_t)(uint16_t)49612, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
    p106_time_usec_SET((uint64_t)5606737186856698470L, PH.base.pack) ;
    p106_sensor_id_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
    p106_integration_time_us_SET((uint32_t)1451140565L, PH.base.pack) ;
    p106_integrated_x_SET((float) -1.3865997E38F, PH.base.pack) ;
    p106_integrated_y_SET((float)2.6995003E38F, PH.base.pack) ;
    p106_integrated_xgyro_SET((float) -2.461377E38F, PH.base.pack) ;
    p106_integrated_ygyro_SET((float) -3.5122116E37F, PH.base.pack) ;
    p106_integrated_zgyro_SET((float) -1.8938105E37F, PH.base.pack) ;
    p106_temperature_SET((int16_t)(int16_t) -2601, PH.base.pack) ;
    p106_quality_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
    p106_time_delta_distance_us_SET((uint32_t)2182968209L, PH.base.pack) ;
    p106_distance_SET((float) -1.03194384E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
    p107_time_usec_SET((uint64_t)3842142701966527322L, PH.base.pack) ;
    p107_xacc_SET((float) -1.13365E38F, PH.base.pack) ;
    p107_yacc_SET((float)1.8865345E38F, PH.base.pack) ;
    p107_zacc_SET((float)3.6531586E37F, PH.base.pack) ;
    p107_xgyro_SET((float) -2.020837E38F, PH.base.pack) ;
    p107_ygyro_SET((float) -1.907875E38F, PH.base.pack) ;
    p107_zgyro_SET((float)2.9764951E38F, PH.base.pack) ;
    p107_xmag_SET((float) -3.645167E37F, PH.base.pack) ;
    p107_ymag_SET((float)1.2988601E38F, PH.base.pack) ;
    p107_zmag_SET((float)3.3410958E38F, PH.base.pack) ;
    p107_abs_pressure_SET((float) -2.5009365E38F, PH.base.pack) ;
    p107_diff_pressure_SET((float)5.2678235E37F, PH.base.pack) ;
    p107_pressure_alt_SET((float) -7.446476E37F, PH.base.pack) ;
    p107_temperature_SET((float) -5.025158E37F, PH.base.pack) ;
    p107_fields_updated_SET((uint32_t)3701440866L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
    p108_q1_SET((float)1.405499E38F, PH.base.pack) ;
    p108_q2_SET((float) -1.1261036E38F, PH.base.pack) ;
    p108_q3_SET((float)5.956751E37F, PH.base.pack) ;
    p108_q4_SET((float)4.990476E37F, PH.base.pack) ;
    p108_roll_SET((float)2.1279398E38F, PH.base.pack) ;
    p108_pitch_SET((float) -8.937433E37F, PH.base.pack) ;
    p108_yaw_SET((float)2.884331E38F, PH.base.pack) ;
    p108_xacc_SET((float) -1.6500921E38F, PH.base.pack) ;
    p108_yacc_SET((float)2.9869945E38F, PH.base.pack) ;
    p108_zacc_SET((float)2.008377E38F, PH.base.pack) ;
    p108_xgyro_SET((float) -1.9170471E38F, PH.base.pack) ;
    p108_ygyro_SET((float) -9.13312E37F, PH.base.pack) ;
    p108_zgyro_SET((float) -8.889733E37F, PH.base.pack) ;
    p108_lat_SET((float)3.1486703E38F, PH.base.pack) ;
    p108_lon_SET((float) -1.3898173E38F, PH.base.pack) ;
    p108_alt_SET((float) -1.4649233E38F, PH.base.pack) ;
    p108_std_dev_horz_SET((float)2.7011277E38F, PH.base.pack) ;
    p108_std_dev_vert_SET((float)1.8948107E38F, PH.base.pack) ;
    p108_vn_SET((float) -2.7354358E38F, PH.base.pack) ;
    p108_ve_SET((float) -1.4413416E38F, PH.base.pack) ;
    p108_vd_SET((float)3.0828273E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
    p109_rssi_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
    p109_remrssi_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
    p109_txbuf_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
    p109_noise_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
    p109_remnoise_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
    p109_rxerrors_SET((uint16_t)(uint16_t)63539, PH.base.pack) ;
    p109_fixed__SET((uint16_t)(uint16_t)58445, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
    p110_target_network_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
    p110_target_system_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
    p110_target_component_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)231, (uint8_t)10, (uint8_t)75, (uint8_t)187, (uint8_t)215, (uint8_t)233, (uint8_t)59, (uint8_t)9, (uint8_t)21, (uint8_t)153, (uint8_t)225, (uint8_t)160, (uint8_t)132, (uint8_t)103, (uint8_t)68, (uint8_t)74, (uint8_t)124, (uint8_t)221, (uint8_t)195, (uint8_t)81, (uint8_t)6, (uint8_t)246, (uint8_t)8, (uint8_t)21, (uint8_t)165, (uint8_t)25, (uint8_t)160, (uint8_t)165, (uint8_t)52, (uint8_t)48, (uint8_t)154, (uint8_t)24, (uint8_t)231, (uint8_t)20, (uint8_t)245, (uint8_t)95, (uint8_t)156, (uint8_t)213, (uint8_t)125, (uint8_t)123, (uint8_t)80, (uint8_t)111, (uint8_t)118, (uint8_t)93, (uint8_t)54, (uint8_t)160, (uint8_t)216, (uint8_t)96, (uint8_t)93, (uint8_t)116, (uint8_t)114, (uint8_t)254, (uint8_t)75, (uint8_t)52, (uint8_t)202, (uint8_t)241, (uint8_t)56, (uint8_t)74, (uint8_t)145, (uint8_t)111, (uint8_t)139, (uint8_t)214, (uint8_t)130, (uint8_t)204, (uint8_t)10, (uint8_t)140, (uint8_t)161, (uint8_t)178, (uint8_t)192, (uint8_t)25, (uint8_t)242, (uint8_t)83, (uint8_t)77, (uint8_t)52, (uint8_t)32, (uint8_t)61, (uint8_t)188, (uint8_t)32, (uint8_t)236, (uint8_t)19, (uint8_t)48, (uint8_t)166, (uint8_t)128, (uint8_t)200, (uint8_t)14, (uint8_t)23, (uint8_t)250, (uint8_t)215, (uint8_t)126, (uint8_t)237, (uint8_t)56, (uint8_t)56, (uint8_t)56, (uint8_t)236, (uint8_t)226, (uint8_t)64, (uint8_t)161, (uint8_t)178, (uint8_t)174, (uint8_t)159, (uint8_t)175, (uint8_t)11, (uint8_t)243, (uint8_t)226, (uint8_t)42, (uint8_t)108, (uint8_t)204, (uint8_t)208, (uint8_t)123, (uint8_t)45, (uint8_t)127, (uint8_t)106, (uint8_t)220, (uint8_t)10, (uint8_t)213, (uint8_t)214, (uint8_t)193, (uint8_t)4, (uint8_t)169, (uint8_t)184, (uint8_t)179, (uint8_t)99, (uint8_t)183, (uint8_t)73, (uint8_t)132, (uint8_t)2, (uint8_t)119, (uint8_t)165, (uint8_t)174, (uint8_t)108, (uint8_t)121, (uint8_t)55, (uint8_t)2, (uint8_t)128, (uint8_t)111, (uint8_t)17, (uint8_t)59, (uint8_t)160, (uint8_t)213, (uint8_t)199, (uint8_t)128, (uint8_t)251, (uint8_t)63, (uint8_t)172, (uint8_t)176, (uint8_t)113, (uint8_t)39, (uint8_t)120, (uint8_t)110, (uint8_t)146, (uint8_t)216, (uint8_t)55, (uint8_t)70, (uint8_t)162, (uint8_t)15, (uint8_t)103, (uint8_t)8, (uint8_t)44, (uint8_t)188, (uint8_t)50, (uint8_t)78, (uint8_t)17, (uint8_t)156, (uint8_t)196, (uint8_t)100, (uint8_t)155, (uint8_t)246, (uint8_t)145, (uint8_t)233, (uint8_t)20, (uint8_t)10, (uint8_t)75, (uint8_t)125, (uint8_t)125, (uint8_t)54, (uint8_t)86, (uint8_t)136, (uint8_t)248, (uint8_t)104, (uint8_t)159, (uint8_t)102, (uint8_t)119, (uint8_t)30, (uint8_t)199, (uint8_t)220, (uint8_t)25, (uint8_t)18, (uint8_t)172, (uint8_t)210, (uint8_t)127, (uint8_t)202, (uint8_t)81, (uint8_t)229, (uint8_t)213, (uint8_t)77, (uint8_t)175, (uint8_t)238, (uint8_t)107, (uint8_t)85, (uint8_t)234, (uint8_t)94, (uint8_t)157, (uint8_t)100, (uint8_t)88, (uint8_t)107, (uint8_t)15, (uint8_t)252, (uint8_t)154, (uint8_t)124, (uint8_t)96, (uint8_t)166, (uint8_t)214, (uint8_t)228, (uint8_t)21, (uint8_t)126, (uint8_t)165, (uint8_t)181, (uint8_t)160, (uint8_t)156, (uint8_t)217, (uint8_t)33, (uint8_t)141, (uint8_t)232, (uint8_t)31, (uint8_t)32, (uint8_t)159, (uint8_t)91, (uint8_t)7, (uint8_t)176, (uint8_t)1, (uint8_t)75, (uint8_t)45, (uint8_t)235, (uint8_t)94, (uint8_t)10, (uint8_t)244, (uint8_t)44, (uint8_t)62, (uint8_t)39, (uint8_t)27, (uint8_t)209, (uint8_t)16, (uint8_t)193, (uint8_t)28, (uint8_t)58, (uint8_t)240, (uint8_t)63, (uint8_t)56, (uint8_t)137, (uint8_t)87, (uint8_t)146};
        p110_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
    p111_tc1_SET((int64_t) -2724559450418795906L, PH.base.pack) ;
    p111_ts1_SET((int64_t) -7564454805535380958L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
    p112_time_usec_SET((uint64_t)6310032171945329843L, PH.base.pack) ;
    p112_seq_SET((uint32_t)1889409738L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
    p113_time_usec_SET((uint64_t)860407595570962057L, PH.base.pack) ;
    p113_fix_type_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
    p113_lat_SET((int32_t)1267170040, PH.base.pack) ;
    p113_lon_SET((int32_t)1692199325, PH.base.pack) ;
    p113_alt_SET((int32_t) -397069956, PH.base.pack) ;
    p113_eph_SET((uint16_t)(uint16_t)16837, PH.base.pack) ;
    p113_epv_SET((uint16_t)(uint16_t)27538, PH.base.pack) ;
    p113_vel_SET((uint16_t)(uint16_t)58053, PH.base.pack) ;
    p113_vn_SET((int16_t)(int16_t)19179, PH.base.pack) ;
    p113_ve_SET((int16_t)(int16_t)14462, PH.base.pack) ;
    p113_vd_SET((int16_t)(int16_t)11814, PH.base.pack) ;
    p113_cog_SET((uint16_t)(uint16_t)11844, PH.base.pack) ;
    p113_satellites_visible_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
    p114_time_usec_SET((uint64_t)8474280136979819521L, PH.base.pack) ;
    p114_sensor_id_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
    p114_integration_time_us_SET((uint32_t)3227545689L, PH.base.pack) ;
    p114_integrated_x_SET((float)7.832773E37F, PH.base.pack) ;
    p114_integrated_y_SET((float) -1.5434842E38F, PH.base.pack) ;
    p114_integrated_xgyro_SET((float)5.1585074E37F, PH.base.pack) ;
    p114_integrated_ygyro_SET((float)2.3623975E38F, PH.base.pack) ;
    p114_integrated_zgyro_SET((float) -2.8021654E38F, PH.base.pack) ;
    p114_temperature_SET((int16_t)(int16_t)24021, PH.base.pack) ;
    p114_quality_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
    p114_time_delta_distance_us_SET((uint32_t)2995687161L, PH.base.pack) ;
    p114_distance_SET((float)3.2088875E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
    p115_time_usec_SET((uint64_t)7233218095038879306L, PH.base.pack) ;
    {
        float  attitude_quaternion [] =  {-2.5463985E38F, 1.137509E38F, -3.3197157E38F, 5.487058E36F};
        p115_attitude_quaternion_SET(&attitude_quaternion, 0, &PH.base.pack) ;
    }
    p115_rollspeed_SET((float) -2.8697539E38F, PH.base.pack) ;
    p115_pitchspeed_SET((float) -2.2180145E38F, PH.base.pack) ;
    p115_yawspeed_SET((float)1.1513669E38F, PH.base.pack) ;
    p115_lat_SET((int32_t) -2120060807, PH.base.pack) ;
    p115_lon_SET((int32_t) -1671304675, PH.base.pack) ;
    p115_alt_SET((int32_t) -1369178707, PH.base.pack) ;
    p115_vx_SET((int16_t)(int16_t)19241, PH.base.pack) ;
    p115_vy_SET((int16_t)(int16_t)27759, PH.base.pack) ;
    p115_vz_SET((int16_t)(int16_t)18146, PH.base.pack) ;
    p115_ind_airspeed_SET((uint16_t)(uint16_t)670, PH.base.pack) ;
    p115_true_airspeed_SET((uint16_t)(uint16_t)15435, PH.base.pack) ;
    p115_xacc_SET((int16_t)(int16_t) -13516, PH.base.pack) ;
    p115_yacc_SET((int16_t)(int16_t) -13066, PH.base.pack) ;
    p115_zacc_SET((int16_t)(int16_t)1310, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
    p116_time_boot_ms_SET((uint32_t)118427866L, PH.base.pack) ;
    p116_xacc_SET((int16_t)(int16_t) -20009, PH.base.pack) ;
    p116_yacc_SET((int16_t)(int16_t) -29551, PH.base.pack) ;
    p116_zacc_SET((int16_t)(int16_t) -9571, PH.base.pack) ;
    p116_xgyro_SET((int16_t)(int16_t)1478, PH.base.pack) ;
    p116_ygyro_SET((int16_t)(int16_t)30246, PH.base.pack) ;
    p116_zgyro_SET((int16_t)(int16_t)4330, PH.base.pack) ;
    p116_xmag_SET((int16_t)(int16_t)16329, PH.base.pack) ;
    p116_ymag_SET((int16_t)(int16_t) -7173, PH.base.pack) ;
    p116_zmag_SET((int16_t)(int16_t) -23670, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
    p117_target_system_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
    p117_target_component_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
    p117_start_SET((uint16_t)(uint16_t)21648, PH.base.pack) ;
    p117_end_SET((uint16_t)(uint16_t)60392, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
    p118_id_SET((uint16_t)(uint16_t)52438, PH.base.pack) ;
    p118_num_logs_SET((uint16_t)(uint16_t)9907, PH.base.pack) ;
    p118_last_log_num_SET((uint16_t)(uint16_t)29149, PH.base.pack) ;
    p118_time_utc_SET((uint32_t)3881430225L, PH.base.pack) ;
    p118_size_SET((uint32_t)3630241301L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
    p119_target_system_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
    p119_target_component_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
    p119_id_SET((uint16_t)(uint16_t)43280, PH.base.pack) ;
    p119_ofs_SET((uint32_t)3818179941L, PH.base.pack) ;
    p119_count_SET((uint32_t)3381501892L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
    p120_id_SET((uint16_t)(uint16_t)31424, PH.base.pack) ;
    p120_ofs_SET((uint32_t)546347596L, PH.base.pack) ;
    p120_count_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)214, (uint8_t)119, (uint8_t)169, (uint8_t)113, (uint8_t)205, (uint8_t)0, (uint8_t)210, (uint8_t)145, (uint8_t)254, (uint8_t)227, (uint8_t)51, (uint8_t)113, (uint8_t)241, (uint8_t)111, (uint8_t)86, (uint8_t)48, (uint8_t)92, (uint8_t)19, (uint8_t)90, (uint8_t)253, (uint8_t)174, (uint8_t)104, (uint8_t)179, (uint8_t)144, (uint8_t)47, (uint8_t)92, (uint8_t)212, (uint8_t)147, (uint8_t)50, (uint8_t)179, (uint8_t)185, (uint8_t)217, (uint8_t)167, (uint8_t)104, (uint8_t)170, (uint8_t)28, (uint8_t)112, (uint8_t)90, (uint8_t)239, (uint8_t)92, (uint8_t)5, (uint8_t)102, (uint8_t)129, (uint8_t)192, (uint8_t)140, (uint8_t)117, (uint8_t)24, (uint8_t)236, (uint8_t)67, (uint8_t)32, (uint8_t)61, (uint8_t)93, (uint8_t)151, (uint8_t)141, (uint8_t)123, (uint8_t)141, (uint8_t)165, (uint8_t)133, (uint8_t)48, (uint8_t)229, (uint8_t)175, (uint8_t)205, (uint8_t)227, (uint8_t)213, (uint8_t)204, (uint8_t)165, (uint8_t)212, (uint8_t)244, (uint8_t)249, (uint8_t)43, (uint8_t)188, (uint8_t)195, (uint8_t)251, (uint8_t)239, (uint8_t)183, (uint8_t)170, (uint8_t)174, (uint8_t)114, (uint8_t)137, (uint8_t)20, (uint8_t)149, (uint8_t)90, (uint8_t)84, (uint8_t)176, (uint8_t)58, (uint8_t)103, (uint8_t)13, (uint8_t)50, (uint8_t)132, (uint8_t)223};
        p120_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
    p121_target_system_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
    p121_target_component_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
    p122_target_system_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
    p122_target_component_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
    p123_target_system_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
    p123_target_component_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
    p123_len_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)15, (uint8_t)127, (uint8_t)225, (uint8_t)237, (uint8_t)188, (uint8_t)10, (uint8_t)100, (uint8_t)215, (uint8_t)151, (uint8_t)217, (uint8_t)150, (uint8_t)31, (uint8_t)103, (uint8_t)137, (uint8_t)18, (uint8_t)136, (uint8_t)224, (uint8_t)12, (uint8_t)115, (uint8_t)201, (uint8_t)28, (uint8_t)27, (uint8_t)220, (uint8_t)96, (uint8_t)230, (uint8_t)204, (uint8_t)181, (uint8_t)38, (uint8_t)53, (uint8_t)174, (uint8_t)229, (uint8_t)134, (uint8_t)203, (uint8_t)216, (uint8_t)248, (uint8_t)250, (uint8_t)76, (uint8_t)252, (uint8_t)210, (uint8_t)237, (uint8_t)204, (uint8_t)65, (uint8_t)186, (uint8_t)167, (uint8_t)122, (uint8_t)212, (uint8_t)129, (uint8_t)175, (uint8_t)216, (uint8_t)153, (uint8_t)18, (uint8_t)133, (uint8_t)50, (uint8_t)51, (uint8_t)69, (uint8_t)106, (uint8_t)11, (uint8_t)146, (uint8_t)168, (uint8_t)91, (uint8_t)254, (uint8_t)92, (uint8_t)195, (uint8_t)56, (uint8_t)40, (uint8_t)108, (uint8_t)160, (uint8_t)17, (uint8_t)153, (uint8_t)123, (uint8_t)214, (uint8_t)103, (uint8_t)128, (uint8_t)10, (uint8_t)159, (uint8_t)227, (uint8_t)99, (uint8_t)188, (uint8_t)237, (uint8_t)88, (uint8_t)215, (uint8_t)62, (uint8_t)108, (uint8_t)232, (uint8_t)192, (uint8_t)20, (uint8_t)158, (uint8_t)13, (uint8_t)73, (uint8_t)79, (uint8_t)56, (uint8_t)110, (uint8_t)81, (uint8_t)255, (uint8_t)92, (uint8_t)126, (uint8_t)53, (uint8_t)81, (uint8_t)212, (uint8_t)29, (uint8_t)145, (uint8_t)196, (uint8_t)13, (uint8_t)254, (uint8_t)246, (uint8_t)165, (uint8_t)225, (uint8_t)191, (uint8_t)194, (uint8_t)242};
        p123_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
    p124_time_usec_SET((uint64_t)4501472111885704257L, PH.base.pack) ;
    p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX, PH.base.pack) ;
    p124_lat_SET((int32_t) -739631989, PH.base.pack) ;
    p124_lon_SET((int32_t)997123940, PH.base.pack) ;
    p124_alt_SET((int32_t)817758563, PH.base.pack) ;
    p124_eph_SET((uint16_t)(uint16_t)10210, PH.base.pack) ;
    p124_epv_SET((uint16_t)(uint16_t)3046, PH.base.pack) ;
    p124_vel_SET((uint16_t)(uint16_t)10583, PH.base.pack) ;
    p124_cog_SET((uint16_t)(uint16_t)6849, PH.base.pack) ;
    p124_satellites_visible_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
    p124_dgps_numch_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
    p124_dgps_age_SET((uint32_t)4057606497L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
    p125_Vcc_SET((uint16_t)(uint16_t)46864, PH.base.pack) ;
    p125_Vservo_SET((uint16_t)(uint16_t)24716, PH.base.pack) ;
    p125_flags_SET((e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT), PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
    p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS1, PH.base.pack) ;
    p126_flags_SET((e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND |
                    e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_MULTI), PH.base.pack) ;
    p126_timeout_SET((uint16_t)(uint16_t)30986, PH.base.pack) ;
    p126_baudrate_SET((uint32_t)3309786956L, PH.base.pack) ;
    p126_count_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)33, (uint8_t)3, (uint8_t)81, (uint8_t)58, (uint8_t)50, (uint8_t)223, (uint8_t)20, (uint8_t)43, (uint8_t)148, (uint8_t)68, (uint8_t)148, (uint8_t)84, (uint8_t)18, (uint8_t)23, (uint8_t)219, (uint8_t)101, (uint8_t)53, (uint8_t)89, (uint8_t)14, (uint8_t)149, (uint8_t)126, (uint8_t)144, (uint8_t)157, (uint8_t)58, (uint8_t)78, (uint8_t)159, (uint8_t)112, (uint8_t)187, (uint8_t)142, (uint8_t)51, (uint8_t)27, (uint8_t)103, (uint8_t)115, (uint8_t)117, (uint8_t)167, (uint8_t)17, (uint8_t)164, (uint8_t)67, (uint8_t)211, (uint8_t)140, (uint8_t)245, (uint8_t)82, (uint8_t)9, (uint8_t)217, (uint8_t)81, (uint8_t)9, (uint8_t)209, (uint8_t)134, (uint8_t)39, (uint8_t)70, (uint8_t)67, (uint8_t)118, (uint8_t)186, (uint8_t)188, (uint8_t)234, (uint8_t)119, (uint8_t)50, (uint8_t)55, (uint8_t)209, (uint8_t)24, (uint8_t)51, (uint8_t)127, (uint8_t)120, (uint8_t)12, (uint8_t)226, (uint8_t)162, (uint8_t)243, (uint8_t)158, (uint8_t)50, (uint8_t)82};
        p126_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
    p127_time_last_baseline_ms_SET((uint32_t)77575278L, PH.base.pack) ;
    p127_rtk_receiver_id_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
    p127_wn_SET((uint16_t)(uint16_t)39332, PH.base.pack) ;
    p127_tow_SET((uint32_t)1704741509L, PH.base.pack) ;
    p127_rtk_health_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
    p127_rtk_rate_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
    p127_nsats_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
    p127_baseline_coords_type_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
    p127_baseline_a_mm_SET((int32_t)381787636, PH.base.pack) ;
    p127_baseline_b_mm_SET((int32_t)1046017811, PH.base.pack) ;
    p127_baseline_c_mm_SET((int32_t) -1135382246, PH.base.pack) ;
    p127_accuracy_SET((uint32_t)3343171255L, PH.base.pack) ;
    p127_iar_num_hypotheses_SET((int32_t)2036896606, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
    p128_time_last_baseline_ms_SET((uint32_t)3436669208L, PH.base.pack) ;
    p128_rtk_receiver_id_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
    p128_wn_SET((uint16_t)(uint16_t)5240, PH.base.pack) ;
    p128_tow_SET((uint32_t)3898219583L, PH.base.pack) ;
    p128_rtk_health_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
    p128_rtk_rate_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
    p128_nsats_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
    p128_baseline_coords_type_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
    p128_baseline_a_mm_SET((int32_t) -824759318, PH.base.pack) ;
    p128_baseline_b_mm_SET((int32_t) -1395714276, PH.base.pack) ;
    p128_baseline_c_mm_SET((int32_t)2136989364, PH.base.pack) ;
    p128_accuracy_SET((uint32_t)2734537110L, PH.base.pack) ;
    p128_iar_num_hypotheses_SET((int32_t)1755693092, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
    p129_time_boot_ms_SET((uint32_t)1920848599L, PH.base.pack) ;
    p129_xacc_SET((int16_t)(int16_t)30256, PH.base.pack) ;
    p129_yacc_SET((int16_t)(int16_t) -16270, PH.base.pack) ;
    p129_zacc_SET((int16_t)(int16_t)17875, PH.base.pack) ;
    p129_xgyro_SET((int16_t)(int16_t)28835, PH.base.pack) ;
    p129_ygyro_SET((int16_t)(int16_t)6353, PH.base.pack) ;
    p129_zgyro_SET((int16_t)(int16_t) -6867, PH.base.pack) ;
    p129_xmag_SET((int16_t)(int16_t)14527, PH.base.pack) ;
    p129_ymag_SET((int16_t)(int16_t) -14242, PH.base.pack) ;
    p129_zmag_SET((int16_t)(int16_t) -26748, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
    p130_type_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
    p130_size_SET((uint32_t)200328043L, PH.base.pack) ;
    p130_width_SET((uint16_t)(uint16_t)31570, PH.base.pack) ;
    p130_height_SET((uint16_t)(uint16_t)986, PH.base.pack) ;
    p130_packets_SET((uint16_t)(uint16_t)30958, PH.base.pack) ;
    p130_payload_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
    p130_jpg_quality_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
    p131_seqnr_SET((uint16_t)(uint16_t)21809, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)200, (uint8_t)139, (uint8_t)57, (uint8_t)6, (uint8_t)118, (uint8_t)108, (uint8_t)42, (uint8_t)181, (uint8_t)94, (uint8_t)138, (uint8_t)79, (uint8_t)58, (uint8_t)53, (uint8_t)21, (uint8_t)40, (uint8_t)12, (uint8_t)226, (uint8_t)43, (uint8_t)22, (uint8_t)215, (uint8_t)199, (uint8_t)185, (uint8_t)235, (uint8_t)30, (uint8_t)111, (uint8_t)169, (uint8_t)99, (uint8_t)21, (uint8_t)234, (uint8_t)17, (uint8_t)207, (uint8_t)111, (uint8_t)94, (uint8_t)237, (uint8_t)176, (uint8_t)143, (uint8_t)86, (uint8_t)17, (uint8_t)211, (uint8_t)164, (uint8_t)131, (uint8_t)91, (uint8_t)234, (uint8_t)248, (uint8_t)237, (uint8_t)232, (uint8_t)148, (uint8_t)116, (uint8_t)205, (uint8_t)34, (uint8_t)82, (uint8_t)155, (uint8_t)212, (uint8_t)172, (uint8_t)26, (uint8_t)189, (uint8_t)10, (uint8_t)144, (uint8_t)7, (uint8_t)190, (uint8_t)120, (uint8_t)17, (uint8_t)230, (uint8_t)218, (uint8_t)210, (uint8_t)21, (uint8_t)183, (uint8_t)109, (uint8_t)230, (uint8_t)11, (uint8_t)33, (uint8_t)55, (uint8_t)3, (uint8_t)189, (uint8_t)155, (uint8_t)179, (uint8_t)238, (uint8_t)98, (uint8_t)208, (uint8_t)132, (uint8_t)31, (uint8_t)3, (uint8_t)71, (uint8_t)246, (uint8_t)163, (uint8_t)134, (uint8_t)39, (uint8_t)203, (uint8_t)180, (uint8_t)146, (uint8_t)182, (uint8_t)233, (uint8_t)73, (uint8_t)187, (uint8_t)184, (uint8_t)52, (uint8_t)209, (uint8_t)81, (uint8_t)72, (uint8_t)77, (uint8_t)2, (uint8_t)54, (uint8_t)150, (uint8_t)194, (uint8_t)33, (uint8_t)108, (uint8_t)137, (uint8_t)253, (uint8_t)104, (uint8_t)98, (uint8_t)205, (uint8_t)2, (uint8_t)253, (uint8_t)46, (uint8_t)24, (uint8_t)178, (uint8_t)171, (uint8_t)160, (uint8_t)170, (uint8_t)170, (uint8_t)188, (uint8_t)84, (uint8_t)147, (uint8_t)93, (uint8_t)165, (uint8_t)165, (uint8_t)211, (uint8_t)196, (uint8_t)219, (uint8_t)180, (uint8_t)72, (uint8_t)86, (uint8_t)76, (uint8_t)51, (uint8_t)101, (uint8_t)151, (uint8_t)112, (uint8_t)72, (uint8_t)185, (uint8_t)108, (uint8_t)17, (uint8_t)233, (uint8_t)44, (uint8_t)72, (uint8_t)75, (uint8_t)215, (uint8_t)84, (uint8_t)118, (uint8_t)7, (uint8_t)254, (uint8_t)199, (uint8_t)130, (uint8_t)216, (uint8_t)95, (uint8_t)192, (uint8_t)141, (uint8_t)109, (uint8_t)19, (uint8_t)129, (uint8_t)217, (uint8_t)96, (uint8_t)5, (uint8_t)132, (uint8_t)239, (uint8_t)130, (uint8_t)168, (uint8_t)116, (uint8_t)215, (uint8_t)132, (uint8_t)213, (uint8_t)133, (uint8_t)156, (uint8_t)0, (uint8_t)165, (uint8_t)191, (uint8_t)123, (uint8_t)227, (uint8_t)24, (uint8_t)42, (uint8_t)127, (uint8_t)10, (uint8_t)227, (uint8_t)54, (uint8_t)221, (uint8_t)85, (uint8_t)20, (uint8_t)217, (uint8_t)216, (uint8_t)0, (uint8_t)234, (uint8_t)123, (uint8_t)38, (uint8_t)20, (uint8_t)46, (uint8_t)204, (uint8_t)136, (uint8_t)53, (uint8_t)235, (uint8_t)1, (uint8_t)150, (uint8_t)32, (uint8_t)41, (uint8_t)181, (uint8_t)97, (uint8_t)84, (uint8_t)102, (uint8_t)65, (uint8_t)253, (uint8_t)151, (uint8_t)186, (uint8_t)200, (uint8_t)184, (uint8_t)52, (uint8_t)117, (uint8_t)81, (uint8_t)236, (uint8_t)215, (uint8_t)233, (uint8_t)27, (uint8_t)92, (uint8_t)7, (uint8_t)238, (uint8_t)47, (uint8_t)32, (uint8_t)91, (uint8_t)23, (uint8_t)57, (uint8_t)245, (uint8_t)184, (uint8_t)183, (uint8_t)78, (uint8_t)136, (uint8_t)169, (uint8_t)13, (uint8_t)85, (uint8_t)62, (uint8_t)89, (uint8_t)128, (uint8_t)93, (uint8_t)203, (uint8_t)66, (uint8_t)210, (uint8_t)179, (uint8_t)163, (uint8_t)168, (uint8_t)49, (uint8_t)253, (uint8_t)173, (uint8_t)39, (uint8_t)235, (uint8_t)127, (uint8_t)90, (uint8_t)31};
        p131_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
    p132_time_boot_ms_SET((uint32_t)1207490027L, PH.base.pack) ;
    p132_min_distance_SET((uint16_t)(uint16_t)48836, PH.base.pack) ;
    p132_max_distance_SET((uint16_t)(uint16_t)7740, PH.base.pack) ;
    p132_current_distance_SET((uint16_t)(uint16_t)60430, PH.base.pack) ;
    p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, PH.base.pack) ;
    p132_id_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
    p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_PITCH_90, PH.base.pack) ;
    p132_covariance_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
    p133_lat_SET((int32_t) -1341432789, PH.base.pack) ;
    p133_lon_SET((int32_t)1043581398, PH.base.pack) ;
    p133_grid_spacing_SET((uint16_t)(uint16_t)10032, PH.base.pack) ;
    p133_mask_SET((uint64_t)1044329737121762911L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
    p134_lat_SET((int32_t) -722816149, PH.base.pack) ;
    p134_lon_SET((int32_t)833919821, PH.base.pack) ;
    p134_grid_spacing_SET((uint16_t)(uint16_t)57225, PH.base.pack) ;
    p134_gridbit_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
    {
        int16_t  data_ [] =  {(int16_t) -32164, (int16_t)18884, (int16_t) -8638, (int16_t)9657, (int16_t) -25272, (int16_t) -16774, (int16_t)6602, (int16_t)1687, (int16_t)8027, (int16_t)26086, (int16_t) -27931, (int16_t)18408, (int16_t)24573, (int16_t)19506, (int16_t)13736, (int16_t) -17525};
        p134_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
    p135_lat_SET((int32_t)753958221, PH.base.pack) ;
    p135_lon_SET((int32_t) -485047959, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
    p136_lat_SET((int32_t) -1923045729, PH.base.pack) ;
    p136_lon_SET((int32_t) -481773217, PH.base.pack) ;
    p136_spacing_SET((uint16_t)(uint16_t)13151, PH.base.pack) ;
    p136_terrain_height_SET((float) -1.0978655E38F, PH.base.pack) ;
    p136_current_height_SET((float)2.8606582E38F, PH.base.pack) ;
    p136_pending_SET((uint16_t)(uint16_t)59394, PH.base.pack) ;
    p136_loaded_SET((uint16_t)(uint16_t)7638, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
    p137_time_boot_ms_SET((uint32_t)1030849397L, PH.base.pack) ;
    p137_press_abs_SET((float) -4.743992E37F, PH.base.pack) ;
    p137_press_diff_SET((float)2.5500834E38F, PH.base.pack) ;
    p137_temperature_SET((int16_t)(int16_t)5490, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATT_POS_MOCAP_138(), &PH);
    p138_time_usec_SET((uint64_t)3157242327314456989L, PH.base.pack) ;
    {
        float  q [] =  {4.73818E37F, 1.2039917E38F, 2.2861949E38F, 2.0737245E38F};
        p138_q_SET(&q, 0, &PH.base.pack) ;
    }
    p138_x_SET((float) -2.5223894E38F, PH.base.pack) ;
    p138_y_SET((float)6.8018683E37F, PH.base.pack) ;
    p138_z_SET((float) -1.735763E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
    p139_time_usec_SET((uint64_t)3509604243254527329L, PH.base.pack) ;
    p139_group_mlx_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
    p139_target_system_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
    p139_target_component_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
    {
        float  controls [] =  {-2.4540738E38F, 4.5332007E37F, -1.1041999E38F, -2.0228456E38F, -4.7921954E37F, -2.8183443E38F, 7.1638145E37F, 1.0725675E38F};
        p139_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
    p140_time_usec_SET((uint64_t)977198036313859407L, PH.base.pack) ;
    p140_group_mlx_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
    {
        float  controls [] =  {-2.4318865E38F, 1.6954593E38F, 5.601244E37F, 3.3113666E38F, -2.2597807E38F, 2.7606104E38F, -2.8394923E38F, -3.0706972E38F};
        p140_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}
