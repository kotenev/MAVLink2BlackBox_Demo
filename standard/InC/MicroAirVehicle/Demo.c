
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
void c_CommunicationChannel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    float  airspeed = p74_airspeed_GET(pack);
    float  groundspeed = p74_groundspeed_GET(pack);
    int16_t  heading = p74_heading_GET(pack);
    uint16_t  throttle = p74_throttle_GET(pack);
    float  alt = p74_alt_GET(pack);
    float  climb = p74_climb_GET(pack);
}
void c_CommunicationChannel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
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
    p0_type_SET(e_MAV_TYPE_MAV_TYPE_OCTOROTOR, PH.base.pack) ;
    p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_AEROB, PH.base.pack) ;
    p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PH.base.pack) ;
    p0_custom_mode_SET((uint32_t)1648605355L, PH.base.pack) ;
    p0_system_status_SET(e_MAV_STATE_MAV_STATE_CALIBRATING, PH.base.pack) ;
    p0_mavlink_version_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYS_STATUS_1(), &PH);
    p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE, PH.base.pack) ;
    p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL, PH.base.pack) ;
    p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_RC_RECEIVER, PH.base.pack) ;
    p1_load_SET((uint16_t)(uint16_t)1563, PH.base.pack) ;
    p1_voltage_battery_SET((uint16_t)(uint16_t)37272, PH.base.pack) ;
    p1_current_battery_SET((int16_t)(int16_t) -2802, PH.base.pack) ;
    p1_battery_remaining_SET((int8_t)(int8_t)126, PH.base.pack) ;
    p1_drop_rate_comm_SET((uint16_t)(uint16_t)22182, PH.base.pack) ;
    p1_errors_comm_SET((uint16_t)(uint16_t)37442, PH.base.pack) ;
    p1_errors_count1_SET((uint16_t)(uint16_t)42029, PH.base.pack) ;
    p1_errors_count2_SET((uint16_t)(uint16_t)52972, PH.base.pack) ;
    p1_errors_count3_SET((uint16_t)(uint16_t)41352, PH.base.pack) ;
    p1_errors_count4_SET((uint16_t)(uint16_t)41334, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SYSTEM_TIME_2(), &PH);
    p2_time_unix_usec_SET((uint64_t)8631735063674127869L, PH.base.pack) ;
    p2_time_boot_ms_SET((uint32_t)2427043488L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
    p3_time_boot_ms_SET((uint32_t)4145631702L, PH.base.pack) ;
    p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
    p3_type_mask_SET((uint16_t)(uint16_t)14404, PH.base.pack) ;
    p3_x_SET((float) -3.0129777E38F, PH.base.pack) ;
    p3_y_SET((float) -3.36035E37F, PH.base.pack) ;
    p3_z_SET((float)3.2645331E38F, PH.base.pack) ;
    p3_vx_SET((float) -1.8049207E38F, PH.base.pack) ;
    p3_vy_SET((float)1.7431946E38F, PH.base.pack) ;
    p3_vz_SET((float)1.2946782E38F, PH.base.pack) ;
    p3_afx_SET((float) -1.5063795E38F, PH.base.pack) ;
    p3_afy_SET((float) -1.433387E38F, PH.base.pack) ;
    p3_afz_SET((float)6.540181E37F, PH.base.pack) ;
    p3_yaw_SET((float)2.0522916E38F, PH.base.pack) ;
    p3_yaw_rate_SET((float)6.6050744E36F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PING_4(), &PH);
    p4_time_usec_SET((uint64_t)1570948352612493307L, PH.base.pack) ;
    p4_seq_SET((uint32_t)2368233634L, PH.base.pack) ;
    p4_target_system_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
    p4_target_component_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
    p5_target_system_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
    p5_control_request_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
    p5_version_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
    {
        char16_t   passkey = "vcxSkfkfigkiiuow";
        p5_passkey_SET(&passkey, 0,  sizeof(passkey), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
    p6_gcs_system_id_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
    p6_control_request_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
    p6_ack_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_AUTH_KEY_7(), &PH);
    {
        char16_t   key = "icgcxhbkeoxrwhiupggmis";
        p7_key_SET(&key, 0,  sizeof(key), &PH) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_MODE_11(), &PH);
    p11_target_system_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
    p11_base_mode_SET(e_MAV_MODE_MAV_MODE_PREFLIGHT, PH.base.pack) ;
    p11_custom_mode_SET((uint32_t)1648505448L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_READ_20(), &PH);
    p20_target_system_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
    p20_target_component_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
    {
        char16_t   param_id = "auxfi";
        p20_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p20_param_index_SET((int16_t)(int16_t) -12548, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_REQUEST_LIST_21(), &PH);
    p21_target_system_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    p21_target_component_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_VALUE_22(), &PH);
    {
        char16_t   param_id = "umtrbqa";
        p22_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p22_param_value_SET((float) -5.51632E37F, PH.base.pack) ;
    p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32, PH.base.pack) ;
    p22_param_count_SET((uint16_t)(uint16_t)63387, PH.base.pack) ;
    p22_param_index_SET((uint16_t)(uint16_t)35637, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_SET_23(), &PH);
    p23_target_system_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
    p23_target_component_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
    {
        char16_t   param_id = "uqg";
        p23_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p23_param_value_SET((float)2.8589263E38F, PH.base.pack) ;
    p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT16, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RAW_INT_24(), &PH);
    p24_time_usec_SET((uint64_t)5632731960461193555L, PH.base.pack) ;
    p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, PH.base.pack) ;
    p24_lat_SET((int32_t)318929803, PH.base.pack) ;
    p24_lon_SET((int32_t)1624341885, PH.base.pack) ;
    p24_alt_SET((int32_t) -2060574657, PH.base.pack) ;
    p24_eph_SET((uint16_t)(uint16_t)18586, PH.base.pack) ;
    p24_epv_SET((uint16_t)(uint16_t)48161, PH.base.pack) ;
    p24_vel_SET((uint16_t)(uint16_t)35287, PH.base.pack) ;
    p24_cog_SET((uint16_t)(uint16_t)17002, PH.base.pack) ;
    p24_satellites_visible_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
    p24_alt_ellipsoid_SET((int32_t)1532038998, &PH) ;
    p24_h_acc_SET((uint32_t)2872625574L, &PH) ;
    p24_v_acc_SET((uint32_t)2562855933L, &PH) ;
    p24_vel_acc_SET((uint32_t)4072740714L, &PH) ;
    p24_hdg_acc_SET((uint32_t)3338633148L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_STATUS_25(), &PH);
    p25_satellites_visible_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
    {
        uint8_t  satellite_prn [] =  {(uint8_t)135, (uint8_t)201, (uint8_t)150, (uint8_t)147, (uint8_t)139, (uint8_t)166, (uint8_t)140, (uint8_t)225, (uint8_t)236, (uint8_t)237, (uint8_t)140, (uint8_t)123, (uint8_t)49, (uint8_t)51, (uint8_t)26, (uint8_t)237, (uint8_t)71, (uint8_t)12, (uint8_t)180, (uint8_t)10};
        p25_satellite_prn_SET(&satellite_prn, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_used [] =  {(uint8_t)71, (uint8_t)90, (uint8_t)86, (uint8_t)15, (uint8_t)162, (uint8_t)46, (uint8_t)158, (uint8_t)107, (uint8_t)117, (uint8_t)24, (uint8_t)96, (uint8_t)186, (uint8_t)222, (uint8_t)18, (uint8_t)143, (uint8_t)93, (uint8_t)133, (uint8_t)167, (uint8_t)36, (uint8_t)218};
        p25_satellite_used_SET(&satellite_used, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_elevation [] =  {(uint8_t)220, (uint8_t)184, (uint8_t)131, (uint8_t)34, (uint8_t)199, (uint8_t)196, (uint8_t)192, (uint8_t)233, (uint8_t)141, (uint8_t)244, (uint8_t)200, (uint8_t)6, (uint8_t)149, (uint8_t)219, (uint8_t)166, (uint8_t)134, (uint8_t)185, (uint8_t)53, (uint8_t)138, (uint8_t)239};
        p25_satellite_elevation_SET(&satellite_elevation, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_azimuth [] =  {(uint8_t)172, (uint8_t)126, (uint8_t)128, (uint8_t)254, (uint8_t)78, (uint8_t)63, (uint8_t)212, (uint8_t)101, (uint8_t)52, (uint8_t)125, (uint8_t)253, (uint8_t)22, (uint8_t)254, (uint8_t)168, (uint8_t)209, (uint8_t)98, (uint8_t)14, (uint8_t)107, (uint8_t)94, (uint8_t)209};
        p25_satellite_azimuth_SET(&satellite_azimuth, 0, &PH.base.pack) ;
    }
    {
        uint8_t  satellite_snr [] =  {(uint8_t)251, (uint8_t)196, (uint8_t)187, (uint8_t)238, (uint8_t)242, (uint8_t)15, (uint8_t)181, (uint8_t)7, (uint8_t)227, (uint8_t)116, (uint8_t)13, (uint8_t)186, (uint8_t)226, (uint8_t)183, (uint8_t)143, (uint8_t)153, (uint8_t)35, (uint8_t)215, (uint8_t)30, (uint8_t)102};
        p25_satellite_snr_SET(&satellite_snr, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU_26(), &PH);
    p26_time_boot_ms_SET((uint32_t)364347162L, PH.base.pack) ;
    p26_xacc_SET((int16_t)(int16_t)26179, PH.base.pack) ;
    p26_yacc_SET((int16_t)(int16_t)12777, PH.base.pack) ;
    p26_zacc_SET((int16_t)(int16_t) -15549, PH.base.pack) ;
    p26_xgyro_SET((int16_t)(int16_t)8440, PH.base.pack) ;
    p26_ygyro_SET((int16_t)(int16_t) -21724, PH.base.pack) ;
    p26_zgyro_SET((int16_t)(int16_t) -22795, PH.base.pack) ;
    p26_xmag_SET((int16_t)(int16_t)2534, PH.base.pack) ;
    p26_ymag_SET((int16_t)(int16_t) -949, PH.base.pack) ;
    p26_zmag_SET((int16_t)(int16_t) -31194, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_IMU_27(), &PH);
    p27_time_usec_SET((uint64_t)9221859188511667333L, PH.base.pack) ;
    p27_xacc_SET((int16_t)(int16_t)6315, PH.base.pack) ;
    p27_yacc_SET((int16_t)(int16_t)24041, PH.base.pack) ;
    p27_zacc_SET((int16_t)(int16_t) -31673, PH.base.pack) ;
    p27_xgyro_SET((int16_t)(int16_t)2483, PH.base.pack) ;
    p27_ygyro_SET((int16_t)(int16_t)12285, PH.base.pack) ;
    p27_zgyro_SET((int16_t)(int16_t)27123, PH.base.pack) ;
    p27_xmag_SET((int16_t)(int16_t) -26443, PH.base.pack) ;
    p27_ymag_SET((int16_t)(int16_t) -10128, PH.base.pack) ;
    p27_zmag_SET((int16_t)(int16_t) -6056, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RAW_PRESSURE_28(), &PH);
    p28_time_usec_SET((uint64_t)29555134007837601L, PH.base.pack) ;
    p28_press_abs_SET((int16_t)(int16_t)4311, PH.base.pack) ;
    p28_press_diff1_SET((int16_t)(int16_t) -6460, PH.base.pack) ;
    p28_press_diff2_SET((int16_t)(int16_t) -6849, PH.base.pack) ;
    p28_temperature_SET((int16_t)(int16_t)8455, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE_29(), &PH);
    p29_time_boot_ms_SET((uint32_t)540506576L, PH.base.pack) ;
    p29_press_abs_SET((float)2.6626453E38F, PH.base.pack) ;
    p29_press_diff_SET((float)1.956484E38F, PH.base.pack) ;
    p29_temperature_SET((int16_t)(int16_t) -17486, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_30(), &PH);
    p30_time_boot_ms_SET((uint32_t)767461965L, PH.base.pack) ;
    p30_roll_SET((float) -1.0994344E38F, PH.base.pack) ;
    p30_pitch_SET((float)2.0175122E38F, PH.base.pack) ;
    p30_yaw_SET((float)3.2862327E38F, PH.base.pack) ;
    p30_rollspeed_SET((float)1.5508008E38F, PH.base.pack) ;
    p30_pitchspeed_SET((float)1.2901151E37F, PH.base.pack) ;
    p30_yawspeed_SET((float)2.4595198E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_31(), &PH);
    p31_time_boot_ms_SET((uint32_t)2188500732L, PH.base.pack) ;
    p31_q1_SET((float)2.1422864E37F, PH.base.pack) ;
    p31_q2_SET((float) -2.99534E38F, PH.base.pack) ;
    p31_q3_SET((float)7.984548E37F, PH.base.pack) ;
    p31_q4_SET((float)1.965361E38F, PH.base.pack) ;
    p31_rollspeed_SET((float) -1.2293671E38F, PH.base.pack) ;
    p31_pitchspeed_SET((float)2.3404446E38F, PH.base.pack) ;
    p31_yawspeed_SET((float)1.7839383E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_32(), &PH);
    p32_time_boot_ms_SET((uint32_t)3133076158L, PH.base.pack) ;
    p32_x_SET((float) -9.617235E37F, PH.base.pack) ;
    p32_y_SET((float) -3.273192E38F, PH.base.pack) ;
    p32_z_SET((float) -2.8252883E38F, PH.base.pack) ;
    p32_vx_SET((float) -2.6340321E38F, PH.base.pack) ;
    p32_vy_SET((float)2.5256754E38F, PH.base.pack) ;
    p32_vz_SET((float)8.850355E36F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_33(), &PH);
    p33_time_boot_ms_SET((uint32_t)311242606L, PH.base.pack) ;
    p33_lat_SET((int32_t) -1930034033, PH.base.pack) ;
    p33_lon_SET((int32_t)2031840727, PH.base.pack) ;
    p33_alt_SET((int32_t) -440007170, PH.base.pack) ;
    p33_relative_alt_SET((int32_t) -161652473, PH.base.pack) ;
    p33_vx_SET((int16_t)(int16_t)17746, PH.base.pack) ;
    p33_vy_SET((int16_t)(int16_t)27163, PH.base.pack) ;
    p33_vz_SET((int16_t)(int16_t) -14585, PH.base.pack) ;
    p33_hdg_SET((uint16_t)(uint16_t)35961, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_SCALED_34(), &PH);
    p34_time_boot_ms_SET((uint32_t)3981700707L, PH.base.pack) ;
    p34_port_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
    p34_chan1_scaled_SET((int16_t)(int16_t) -29093, PH.base.pack) ;
    p34_chan2_scaled_SET((int16_t)(int16_t)26727, PH.base.pack) ;
    p34_chan3_scaled_SET((int16_t)(int16_t) -32421, PH.base.pack) ;
    p34_chan4_scaled_SET((int16_t)(int16_t)27887, PH.base.pack) ;
    p34_chan5_scaled_SET((int16_t)(int16_t)1240, PH.base.pack) ;
    p34_chan6_scaled_SET((int16_t)(int16_t)20819, PH.base.pack) ;
    p34_chan7_scaled_SET((int16_t)(int16_t)32362, PH.base.pack) ;
    p34_chan8_scaled_SET((int16_t)(int16_t) -21339, PH.base.pack) ;
    p34_rssi_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_RAW_35(), &PH);
    p35_time_boot_ms_SET((uint32_t)315956255L, PH.base.pack) ;
    p35_port_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
    p35_chan1_raw_SET((uint16_t)(uint16_t)53570, PH.base.pack) ;
    p35_chan2_raw_SET((uint16_t)(uint16_t)1199, PH.base.pack) ;
    p35_chan3_raw_SET((uint16_t)(uint16_t)25530, PH.base.pack) ;
    p35_chan4_raw_SET((uint16_t)(uint16_t)44001, PH.base.pack) ;
    p35_chan5_raw_SET((uint16_t)(uint16_t)48737, PH.base.pack) ;
    p35_chan6_raw_SET((uint16_t)(uint16_t)15059, PH.base.pack) ;
    p35_chan7_raw_SET((uint16_t)(uint16_t)34013, PH.base.pack) ;
    p35_chan8_raw_SET((uint16_t)(uint16_t)36264, PH.base.pack) ;
    p35_rssi_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
    p36_time_usec_SET((uint32_t)2961862733L, PH.base.pack) ;
    p36_port_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
    p36_servo1_raw_SET((uint16_t)(uint16_t)46128, PH.base.pack) ;
    p36_servo2_raw_SET((uint16_t)(uint16_t)3231, PH.base.pack) ;
    p36_servo3_raw_SET((uint16_t)(uint16_t)38864, PH.base.pack) ;
    p36_servo4_raw_SET((uint16_t)(uint16_t)34175, PH.base.pack) ;
    p36_servo5_raw_SET((uint16_t)(uint16_t)11327, PH.base.pack) ;
    p36_servo6_raw_SET((uint16_t)(uint16_t)8878, PH.base.pack) ;
    p36_servo7_raw_SET((uint16_t)(uint16_t)3059, PH.base.pack) ;
    p36_servo8_raw_SET((uint16_t)(uint16_t)55594, PH.base.pack) ;
    p36_servo9_raw_SET((uint16_t)(uint16_t)55813, &PH) ;
    p36_servo10_raw_SET((uint16_t)(uint16_t)4615, &PH) ;
    p36_servo11_raw_SET((uint16_t)(uint16_t)22810, &PH) ;
    p36_servo12_raw_SET((uint16_t)(uint16_t)44551, &PH) ;
    p36_servo13_raw_SET((uint16_t)(uint16_t)62700, &PH) ;
    p36_servo14_raw_SET((uint16_t)(uint16_t)6594, &PH) ;
    p36_servo15_raw_SET((uint16_t)(uint16_t)61757, &PH) ;
    p36_servo16_raw_SET((uint16_t)(uint16_t)56737, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
    p37_target_system_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
    p37_target_component_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
    p37_start_index_SET((int16_t)(int16_t) -16302, PH.base.pack) ;
    p37_end_index_SET((int16_t)(int16_t)21360, PH.base.pack) ;
    p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
    p38_target_system_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
    p38_target_component_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
    p38_start_index_SET((int16_t)(int16_t) -6066, PH.base.pack) ;
    p38_end_index_SET((int16_t)(int16_t) -6631, PH.base.pack) ;
    p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_39(), &PH);
    p39_target_system_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
    p39_target_component_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
    p39_seq_SET((uint16_t)(uint16_t)14371, PH.base.pack) ;
    p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
    p39_command_SET(e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL, PH.base.pack) ;
    p39_current_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
    p39_autocontinue_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
    p39_param1_SET((float) -1.1854013E38F, PH.base.pack) ;
    p39_param2_SET((float) -8.202984E37F, PH.base.pack) ;
    p39_param3_SET((float) -1.5177287E38F, PH.base.pack) ;
    p39_param4_SET((float)1.0634846E38F, PH.base.pack) ;
    p39_x_SET((float)3.360024E38F, PH.base.pack) ;
    p39_y_SET((float) -2.2531538E38F, PH.base.pack) ;
    p39_z_SET((float) -2.0617743E38F, PH.base.pack) ;
    p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_40(), &PH);
    p40_target_system_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
    p40_target_component_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
    p40_seq_SET((uint16_t)(uint16_t)36985, PH.base.pack) ;
    p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_SET_CURRENT_41(), &PH);
    p41_target_system_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
    p41_target_component_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
    p41_seq_SET((uint16_t)(uint16_t)11999, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CURRENT_42(), &PH);
    p42_seq_SET((uint16_t)(uint16_t)42612, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_LIST_43(), &PH);
    p43_target_system_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
    p43_target_component_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
    p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_COUNT_44(), &PH);
    p44_target_system_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
    p44_target_component_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
    p44_count_SET((uint16_t)(uint16_t)21759, PH.base.pack) ;
    p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_CLEAR_ALL_45(), &PH);
    p45_target_system_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
    p45_target_component_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
    p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_REACHED_46(), &PH);
    p46_seq_SET((uint16_t)(uint16_t)35587, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ACK_47(), &PH);
    p47_target_system_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
    p47_target_component_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
    p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM5_X, PH.base.pack) ;
    p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
    p48_target_system_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
    p48_latitude_SET((int32_t) -340870859, PH.base.pack) ;
    p48_longitude_SET((int32_t) -2057795484, PH.base.pack) ;
    p48_altitude_SET((int32_t) -337278210, PH.base.pack) ;
    p48_time_usec_SET((uint64_t)7689333315978180704L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
    p49_latitude_SET((int32_t)446063487, PH.base.pack) ;
    p49_longitude_SET((int32_t)1933899436, PH.base.pack) ;
    p49_altitude_SET((int32_t) -1476073096, PH.base.pack) ;
    p49_time_usec_SET((uint64_t)4097431930220796922L, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_PARAM_MAP_RC_50(), &PH);
    p50_target_system_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
    p50_target_component_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
    {
        char16_t   param_id = "ijmxezf";
        p50_param_id_SET(&param_id, 0,  sizeof(param_id), &PH) ;
    }
    p50_param_index_SET((int16_t)(int16_t) -16546, PH.base.pack) ;
    p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    p50_param_value0_SET((float)2.0644457E37F, PH.base.pack) ;
    p50_scale_SET((float)3.0832893E38F, PH.base.pack) ;
    p50_param_value_min_SET((float) -2.261846E38F, PH.base.pack) ;
    p50_param_value_max_SET((float) -4.169297E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_REQUEST_INT_51(), &PH);
    p51_target_system_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
    p51_target_component_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
    p51_seq_SET((uint16_t)(uint16_t)23073, PH.base.pack) ;
    p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
    p54_target_system_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
    p54_target_component_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
    p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
    p54_p1x_SET((float) -1.9324707E38F, PH.base.pack) ;
    p54_p1y_SET((float) -6.081303E37F, PH.base.pack) ;
    p54_p1z_SET((float) -8.041365E37F, PH.base.pack) ;
    p54_p2x_SET((float) -2.6007422E38F, PH.base.pack) ;
    p54_p2y_SET((float) -1.3365783E38F, PH.base.pack) ;
    p54_p2z_SET((float)2.1967492E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
    p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
    p55_p1x_SET((float) -2.8429334E37F, PH.base.pack) ;
    p55_p1y_SET((float)1.5617584E38F, PH.base.pack) ;
    p55_p1z_SET((float)2.923644E38F, PH.base.pack) ;
    p55_p2x_SET((float)3.389787E37F, PH.base.pack) ;
    p55_p2y_SET((float) -1.7247052E38F, PH.base.pack) ;
    p55_p2z_SET((float)3.0768006E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
    p61_time_usec_SET((uint64_t)164508566247766755L, PH.base.pack) ;
    {
        float  q [] =  {-2.4458217E38F, 2.9883668E38F, -1.574662E38F, 2.363958E35F};
        p61_q_SET(&q, 0, &PH.base.pack) ;
    }
    p61_rollspeed_SET((float)8.142295E37F, PH.base.pack) ;
    p61_pitchspeed_SET((float) -2.8959704E37F, PH.base.pack) ;
    p61_yawspeed_SET((float) -3.1640646E38F, PH.base.pack) ;
    {
        float  covariance [] =  {9.083695E37F, 1.2804607E38F, -2.9342225E38F, 1.491931E38F, 1.5512138E38F, 5.2094203E37F, -1.1362006E38F, -2.9225386E38F, -3.1974736E38F};
        p61_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
    p62_nav_roll_SET((float)2.9100396E38F, PH.base.pack) ;
    p62_nav_pitch_SET((float)2.3379732E38F, PH.base.pack) ;
    p62_nav_bearing_SET((int16_t)(int16_t)4751, PH.base.pack) ;
    p62_target_bearing_SET((int16_t)(int16_t)18172, PH.base.pack) ;
    p62_wp_dist_SET((uint16_t)(uint16_t)61498, PH.base.pack) ;
    p62_alt_error_SET((float)8.685378E37F, PH.base.pack) ;
    p62_aspd_error_SET((float)2.9873645E38F, PH.base.pack) ;
    p62_xtrack_error_SET((float)3.6188932E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
    p63_time_usec_SET((uint64_t)7916425715246787694L, PH.base.pack) ;
    p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
    p63_lat_SET((int32_t)1536682943, PH.base.pack) ;
    p63_lon_SET((int32_t) -2030160394, PH.base.pack) ;
    p63_alt_SET((int32_t)1556692604, PH.base.pack) ;
    p63_relative_alt_SET((int32_t) -2081767165, PH.base.pack) ;
    p63_vx_SET((float)6.7159105E37F, PH.base.pack) ;
    p63_vy_SET((float)1.3183805E37F, PH.base.pack) ;
    p63_vz_SET((float)6.959964E37F, PH.base.pack) ;
    {
        float  covariance [] =  {3.1341069E38F, -3.3021793E38F, 1.6072292E38F, -3.3037293E38F, 2.6866378E38F, 9.599349E37F, -1.932797E38F, -2.2417696E36F, -2.5329768E38F, 4.104705E37F, 2.341959E37F, -2.011084E38F, 2.0797733E38F, -7.7226573E37F, 1.3584926E38F, -9.786703E37F, -2.894624E38F, 2.6834137E38F, 2.7749861E38F, -5.7810895E36F, -2.6498353E36F, 1.5892308E38F, 9.498251E37F, 5.5900577E37F, -1.580293E38F, -2.7193202E37F, 3.2945547E37F, -1.8387273E37F, 2.9937751E38F, -1.2441769E38F, 2.9427472E38F, -1.7422602E38F, -1.5469509E38F, 1.3188236E38F, -2.6331066E38F, -1.4285471E38F};
        p63_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
    p64_time_usec_SET((uint64_t)8230009524276465730L, PH.base.pack) ;
    p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, PH.base.pack) ;
    p64_x_SET((float)4.739741E37F, PH.base.pack) ;
    p64_y_SET((float) -3.5541804E37F, PH.base.pack) ;
    p64_z_SET((float) -3.115324E37F, PH.base.pack) ;
    p64_vx_SET((float) -9.447551E37F, PH.base.pack) ;
    p64_vy_SET((float)6.449582E37F, PH.base.pack) ;
    p64_vz_SET((float) -4.0481945E37F, PH.base.pack) ;
    p64_ax_SET((float) -1.6908395E38F, PH.base.pack) ;
    p64_ay_SET((float)1.7511686E38F, PH.base.pack) ;
    p64_az_SET((float) -3.0771229E38F, PH.base.pack) ;
    {
        float  covariance [] =  {2.0875717E38F, 3.3182393E38F, 1.1752535E38F, -7.2145104E37F, -1.6790315E38F, -1.5884556E38F, 6.8431603E37F, -2.4621E38F, 5.5201397E37F, -1.9731633E38F, 1.2152023E38F, 1.0000542E38F, -3.1219336E38F, -1.1865205E38F, 8.649665E37F, 6.9535823E37F, -2.1331801E38F, -5.739351E37F, 1.1439912E38F, -1.7734606E38F, -3.2459887E37F, -6.6160258E35F, 1.2867995E38F, 1.2554725E38F, 2.049936E38F, -2.6142535E38F, -5.31937E37F, 2.5124837E38F, 1.276492E38F, 3.376064E38F, -7.3726655E37F, -2.1334237E38F, 6.845172E37F, -1.3340987E38F, -5.9124324E37F, 2.8917313E37F, -1.5733478E38F, -9.50674E37F, 3.0602915E38F, 2.6285398E38F, 1.4135399E38F, 3.2334625E38F, -2.083671E36F, 2.7147608E38F, 3.3389983E37F};
        p64_covariance_SET(&covariance, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_65(), &PH);
    p65_time_boot_ms_SET((uint32_t)745675030L, PH.base.pack) ;
    p65_chancount_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
    p65_chan1_raw_SET((uint16_t)(uint16_t)20675, PH.base.pack) ;
    p65_chan2_raw_SET((uint16_t)(uint16_t)15187, PH.base.pack) ;
    p65_chan3_raw_SET((uint16_t)(uint16_t)53930, PH.base.pack) ;
    p65_chan4_raw_SET((uint16_t)(uint16_t)58519, PH.base.pack) ;
    p65_chan5_raw_SET((uint16_t)(uint16_t)34289, PH.base.pack) ;
    p65_chan6_raw_SET((uint16_t)(uint16_t)21065, PH.base.pack) ;
    p65_chan7_raw_SET((uint16_t)(uint16_t)18924, PH.base.pack) ;
    p65_chan8_raw_SET((uint16_t)(uint16_t)24492, PH.base.pack) ;
    p65_chan9_raw_SET((uint16_t)(uint16_t)34980, PH.base.pack) ;
    p65_chan10_raw_SET((uint16_t)(uint16_t)39664, PH.base.pack) ;
    p65_chan11_raw_SET((uint16_t)(uint16_t)56358, PH.base.pack) ;
    p65_chan12_raw_SET((uint16_t)(uint16_t)15703, PH.base.pack) ;
    p65_chan13_raw_SET((uint16_t)(uint16_t)35238, PH.base.pack) ;
    p65_chan14_raw_SET((uint16_t)(uint16_t)31166, PH.base.pack) ;
    p65_chan15_raw_SET((uint16_t)(uint16_t)26533, PH.base.pack) ;
    p65_chan16_raw_SET((uint16_t)(uint16_t)8269, PH.base.pack) ;
    p65_chan17_raw_SET((uint16_t)(uint16_t)19484, PH.base.pack) ;
    p65_chan18_raw_SET((uint16_t)(uint16_t)2430, PH.base.pack) ;
    p65_rssi_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_REQUEST_DATA_STREAM_66(), &PH);
    p66_target_system_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
    p66_target_component_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
    p66_req_stream_id_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
    p66_req_message_rate_SET((uint16_t)(uint16_t)11874, PH.base.pack) ;
    p66_start_stop_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_STREAM_67(), &PH);
    p67_stream_id_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
    p67_message_rate_SET((uint16_t)(uint16_t)35980, PH.base.pack) ;
    p67_on_off_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_CONTROL_69(), &PH);
    p69_target_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
    p69_x_SET((int16_t)(int16_t)1067, PH.base.pack) ;
    p69_y_SET((int16_t)(int16_t) -25306, PH.base.pack) ;
    p69_z_SET((int16_t)(int16_t)7300, PH.base.pack) ;
    p69_r_SET((int16_t)(int16_t) -17586, PH.base.pack) ;
    p69_buttons_SET((uint16_t)(uint16_t)13527, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
    p70_target_system_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
    p70_target_component_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
    p70_chan1_raw_SET((uint16_t)(uint16_t)39600, PH.base.pack) ;
    p70_chan2_raw_SET((uint16_t)(uint16_t)52746, PH.base.pack) ;
    p70_chan3_raw_SET((uint16_t)(uint16_t)30282, PH.base.pack) ;
    p70_chan4_raw_SET((uint16_t)(uint16_t)7059, PH.base.pack) ;
    p70_chan5_raw_SET((uint16_t)(uint16_t)64215, PH.base.pack) ;
    p70_chan6_raw_SET((uint16_t)(uint16_t)35303, PH.base.pack) ;
    p70_chan7_raw_SET((uint16_t)(uint16_t)31467, PH.base.pack) ;
    p70_chan8_raw_SET((uint16_t)(uint16_t)54144, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MISSION_ITEM_INT_73(), &PH);
    p73_target_system_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
    p73_target_component_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
    p73_seq_SET((uint16_t)(uint16_t)49870, PH.base.pack) ;
    p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
    p73_command_SET(e_MAV_CMD_MAV_CMD_PREFLIGHT_CALIBRATION, PH.base.pack) ;
    p73_current_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
    p73_autocontinue_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
    p73_param1_SET((float) -1.3567182E38F, PH.base.pack) ;
    p73_param2_SET((float) -2.4378432E38F, PH.base.pack) ;
    p73_param3_SET((float)1.0209167E38F, PH.base.pack) ;
    p73_param4_SET((float)2.6797288E38F, PH.base.pack) ;
    p73_x_SET((int32_t)1478288735, PH.base.pack) ;
    p73_y_SET((int32_t) -510055727, PH.base.pack) ;
    p73_z_SET((float)1.8363217E37F, PH.base.pack) ;
    p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VFR_HUD_74(), &PH);
    p74_airspeed_SET((float)2.9709398E38F, PH.base.pack) ;
    p74_groundspeed_SET((float)1.2671126E38F, PH.base.pack) ;
    p74_heading_SET((int16_t)(int16_t)5406, PH.base.pack) ;
    p74_throttle_SET((uint16_t)(uint16_t)13145, PH.base.pack) ;
    p74_alt_SET((float)1.2245234E37F, PH.base.pack) ;
    p74_climb_SET((float)2.2428122E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_INT_75(), &PH);
    p75_target_system_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
    p75_target_component_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
    p75_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
    p75_command_SET(e_MAV_CMD_MAV_CMD_WAYPOINT_USER_4, PH.base.pack) ;
    p75_current_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
    p75_autocontinue_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
    p75_param1_SET((float)2.234576E37F, PH.base.pack) ;
    p75_param2_SET((float)2.8721121E38F, PH.base.pack) ;
    p75_param3_SET((float) -1.8486781E37F, PH.base.pack) ;
    p75_param4_SET((float)2.4371627E37F, PH.base.pack) ;
    p75_x_SET((int32_t)1775601700, PH.base.pack) ;
    p75_y_SET((int32_t) -172520437, PH.base.pack) ;
    p75_z_SET((float)2.6614223E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_LONG_76(), &PH);
    p76_target_system_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
    p76_target_component_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
    p76_command_SET(e_MAV_CMD_MAV_CMD_DO_DIGICAM_CONTROL, PH.base.pack) ;
    p76_confirmation_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
    p76_param1_SET((float)1.5974629E38F, PH.base.pack) ;
    p76_param2_SET((float) -1.8773307E38F, PH.base.pack) ;
    p76_param3_SET((float) -7.485199E37F, PH.base.pack) ;
    p76_param4_SET((float)1.325319E38F, PH.base.pack) ;
    p76_param5_SET((float) -2.740286E38F, PH.base.pack) ;
    p76_param6_SET((float) -3.204037E38F, PH.base.pack) ;
    p76_param7_SET((float)2.8329774E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_COMMAND_ACK_77(), &PH);
    p77_command_SET(e_MAV_CMD_MAV_CMD_NAV_RETURN_TO_LAUNCH, PH.base.pack) ;
    p77_result_SET(e_MAV_RESULT_MAV_RESULT_FAILED, PH.base.pack) ;
    p77_progress_SET((uint8_t)(uint8_t)100, &PH) ;
    p77_result_param2_SET((int32_t) -855940102, &PH) ;
    p77_target_system_SET((uint8_t)(uint8_t)147, &PH) ;
    p77_target_component_SET((uint8_t)(uint8_t)195, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_MANUAL_SETPOINT_81(), &PH);
    p81_time_boot_ms_SET((uint32_t)1786416554L, PH.base.pack) ;
    p81_roll_SET((float)1.3612918E38F, PH.base.pack) ;
    p81_pitch_SET((float)1.5976459E38F, PH.base.pack) ;
    p81_yaw_SET((float) -8.666187E37F, PH.base.pack) ;
    p81_thrust_SET((float) -2.2068278E37F, PH.base.pack) ;
    p81_mode_switch_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
    p81_manual_override_switch_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
    p82_time_boot_ms_SET((uint32_t)332993614L, PH.base.pack) ;
    p82_target_system_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
    p82_target_component_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
    p82_type_mask_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
    {
        float  q [] =  {1.1196449E38F, 2.3629269E38F, 2.5788323E38F, 1.6943712E38F};
        p82_q_SET(&q, 0, &PH.base.pack) ;
    }
    p82_body_roll_rate_SET((float) -1.2248404E38F, PH.base.pack) ;
    p82_body_pitch_rate_SET((float)3.3523144E38F, PH.base.pack) ;
    p82_body_yaw_rate_SET((float) -3.0136497E38F, PH.base.pack) ;
    p82_thrust_SET((float) -8.115512E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ATTITUDE_TARGET_83(), &PH);
    p83_time_boot_ms_SET((uint32_t)2054787773L, PH.base.pack) ;
    p83_type_mask_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
    {
        float  q [] =  {2.034153E38F, -2.5728172E38F, 1.4842383E38F, -5.8135476E37F};
        p83_q_SET(&q, 0, &PH.base.pack) ;
    }
    p83_body_roll_rate_SET((float)5.695059E37F, PH.base.pack) ;
    p83_body_pitch_rate_SET((float)6.2283807E37F, PH.base.pack) ;
    p83_body_yaw_rate_SET((float)1.9480597E38F, PH.base.pack) ;
    p83_thrust_SET((float) -9.920217E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
    p84_time_boot_ms_SET((uint32_t)2083458204L, PH.base.pack) ;
    p84_target_system_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
    p84_target_component_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
    p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
    p84_type_mask_SET((uint16_t)(uint16_t)40414, PH.base.pack) ;
    p84_x_SET((float) -1.6281714E37F, PH.base.pack) ;
    p84_y_SET((float)1.9230937E38F, PH.base.pack) ;
    p84_z_SET((float)2.0517681E38F, PH.base.pack) ;
    p84_vx_SET((float)1.8598848E38F, PH.base.pack) ;
    p84_vy_SET((float)1.5900531E38F, PH.base.pack) ;
    p84_vz_SET((float)1.1988462E38F, PH.base.pack) ;
    p84_afx_SET((float)1.9475445E37F, PH.base.pack) ;
    p84_afy_SET((float)7.669155E37F, PH.base.pack) ;
    p84_afz_SET((float) -1.0241513E38F, PH.base.pack) ;
    p84_yaw_SET((float) -3.0909108E38F, PH.base.pack) ;
    p84_yaw_rate_SET((float) -2.8830453E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
    p86_time_boot_ms_SET((uint32_t)1214311026L, PH.base.pack) ;
    p86_target_system_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
    p86_target_component_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
    p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
    p86_type_mask_SET((uint16_t)(uint16_t)6117, PH.base.pack) ;
    p86_lat_int_SET((int32_t) -950869180, PH.base.pack) ;
    p86_lon_int_SET((int32_t)913545056, PH.base.pack) ;
    p86_alt_SET((float)2.1460594E38F, PH.base.pack) ;
    p86_vx_SET((float)5.269532E37F, PH.base.pack) ;
    p86_vy_SET((float) -1.6711351E38F, PH.base.pack) ;
    p86_vz_SET((float) -7.318834E37F, PH.base.pack) ;
    p86_afx_SET((float) -3.2516564E38F, PH.base.pack) ;
    p86_afy_SET((float) -2.6336334E38F, PH.base.pack) ;
    p86_afz_SET((float) -1.1674212E38F, PH.base.pack) ;
    p86_yaw_SET((float)1.3757631E38F, PH.base.pack) ;
    p86_yaw_rate_SET((float)1.098879E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
    p87_time_boot_ms_SET((uint32_t)1637329645L, PH.base.pack) ;
    p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
    p87_type_mask_SET((uint16_t)(uint16_t)62259, PH.base.pack) ;
    p87_lat_int_SET((int32_t)809616855, PH.base.pack) ;
    p87_lon_int_SET((int32_t)876161403, PH.base.pack) ;
    p87_alt_SET((float) -4.3088514E37F, PH.base.pack) ;
    p87_vx_SET((float)2.8118395E38F, PH.base.pack) ;
    p87_vy_SET((float)4.0955532E37F, PH.base.pack) ;
    p87_vz_SET((float) -3.1850628E38F, PH.base.pack) ;
    p87_afx_SET((float) -6.4894056E37F, PH.base.pack) ;
    p87_afy_SET((float) -3.094502E38F, PH.base.pack) ;
    p87_afz_SET((float) -3.3243969E38F, PH.base.pack) ;
    p87_yaw_SET((float)1.3832224E38F, PH.base.pack) ;
    p87_yaw_rate_SET((float)2.3053222E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
    p89_time_boot_ms_SET((uint32_t)3515221554L, PH.base.pack) ;
    p89_x_SET((float) -3.035379E38F, PH.base.pack) ;
    p89_y_SET((float)1.9774047E38F, PH.base.pack) ;
    p89_z_SET((float) -7.707809E37F, PH.base.pack) ;
    p89_roll_SET((float)9.627283E36F, PH.base.pack) ;
    p89_pitch_SET((float)3.5259372E37F, PH.base.pack) ;
    p89_yaw_SET((float)4.5718143E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_90(), &PH);
    p90_time_usec_SET((uint64_t)670294528767533017L, PH.base.pack) ;
    p90_roll_SET((float) -1.1033617E38F, PH.base.pack) ;
    p90_pitch_SET((float)6.472874E37F, PH.base.pack) ;
    p90_yaw_SET((float)7.6685894E37F, PH.base.pack) ;
    p90_rollspeed_SET((float)2.6987608E38F, PH.base.pack) ;
    p90_pitchspeed_SET((float)3.6718818E37F, PH.base.pack) ;
    p90_yawspeed_SET((float) -3.363881E38F, PH.base.pack) ;
    p90_lat_SET((int32_t) -9370685, PH.base.pack) ;
    p90_lon_SET((int32_t) -1559838694, PH.base.pack) ;
    p90_alt_SET((int32_t) -308419700, PH.base.pack) ;
    p90_vx_SET((int16_t)(int16_t) -896, PH.base.pack) ;
    p90_vy_SET((int16_t)(int16_t) -11967, PH.base.pack) ;
    p90_vz_SET((int16_t)(int16_t) -25713, PH.base.pack) ;
    p90_xacc_SET((int16_t)(int16_t)27407, PH.base.pack) ;
    p90_yacc_SET((int16_t)(int16_t) -25956, PH.base.pack) ;
    p90_zacc_SET((int16_t)(int16_t)10852, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_CONTROLS_91(), &PH);
    p91_time_usec_SET((uint64_t)8935944143780507012L, PH.base.pack) ;
    p91_roll_ailerons_SET((float)1.9807607E38F, PH.base.pack) ;
    p91_pitch_elevator_SET((float) -4.2891754E37F, PH.base.pack) ;
    p91_yaw_rudder_SET((float)2.115697E38F, PH.base.pack) ;
    p91_throttle_SET((float) -8.087037E37F, PH.base.pack) ;
    p91_aux1_SET((float) -2.662818E38F, PH.base.pack) ;
    p91_aux2_SET((float) -6.573369E37F, PH.base.pack) ;
    p91_aux3_SET((float)2.6974009E38F, PH.base.pack) ;
    p91_aux4_SET((float) -8.246559E37F, PH.base.pack) ;
    p91_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_DISARMED, PH.base.pack) ;
    p91_nav_mode_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
    p92_time_usec_SET((uint64_t)2266227918372376305L, PH.base.pack) ;
    p92_chan1_raw_SET((uint16_t)(uint16_t)42685, PH.base.pack) ;
    p92_chan2_raw_SET((uint16_t)(uint16_t)36000, PH.base.pack) ;
    p92_chan3_raw_SET((uint16_t)(uint16_t)53183, PH.base.pack) ;
    p92_chan4_raw_SET((uint16_t)(uint16_t)5058, PH.base.pack) ;
    p92_chan5_raw_SET((uint16_t)(uint16_t)7993, PH.base.pack) ;
    p92_chan6_raw_SET((uint16_t)(uint16_t)59824, PH.base.pack) ;
    p92_chan7_raw_SET((uint16_t)(uint16_t)2632, PH.base.pack) ;
    p92_chan8_raw_SET((uint16_t)(uint16_t)5681, PH.base.pack) ;
    p92_chan9_raw_SET((uint16_t)(uint16_t)55090, PH.base.pack) ;
    p92_chan10_raw_SET((uint16_t)(uint16_t)44593, PH.base.pack) ;
    p92_chan11_raw_SET((uint16_t)(uint16_t)13161, PH.base.pack) ;
    p92_chan12_raw_SET((uint16_t)(uint16_t)63635, PH.base.pack) ;
    p92_rssi_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
    p93_time_usec_SET((uint64_t)5959296493678505580L, PH.base.pack) ;
    {
        float  controls [] =  {-6.379836E37F, -9.777998E37F, 1.3481454E38F, 2.9304585E38F, -2.0135352E38F, -1.5056932E38F, 8.710311E37F, -1.0009039E38F, -8.389078E37F, 2.1225558E38F, -1.0663007E38F, -2.094838E38F, 1.8381214E38F, -1.5957821E38F, 2.885731E38F, 3.1450825E37F};
        p93_controls_SET(&controls, 0, &PH.base.pack) ;
    }
    p93_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_ARMED, PH.base.pack) ;
    p93_flags_SET((uint64_t)627044018457658975L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_100(), &PH);
    p100_time_usec_SET((uint64_t)6401885295049494185L, PH.base.pack) ;
    p100_sensor_id_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
    p100_flow_x_SET((int16_t)(int16_t) -9879, PH.base.pack) ;
    p100_flow_y_SET((int16_t)(int16_t)10337, PH.base.pack) ;
    p100_flow_comp_m_x_SET((float) -6.6889156E37F, PH.base.pack) ;
    p100_flow_comp_m_y_SET((float) -4.7429507E37F, PH.base.pack) ;
    p100_quality_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
    p100_ground_distance_SET((float)4.409183E37F, PH.base.pack) ;
    p100_flow_rate_x_SET((float) -1.659576E38F, &PH) ;
    p100_flow_rate_y_SET((float) -1.8568935E38F, &PH) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
    p101_usec_SET((uint64_t)2414220955890488947L, PH.base.pack) ;
    p101_x_SET((float)2.0018241E38F, PH.base.pack) ;
    p101_y_SET((float)3.4597585E36F, PH.base.pack) ;
    p101_z_SET((float)1.853217E38F, PH.base.pack) ;
    p101_roll_SET((float)2.4267485E38F, PH.base.pack) ;
    p101_pitch_SET((float) -3.2383532E38F, PH.base.pack) ;
    p101_yaw_SET((float)2.7683477E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
    p102_usec_SET((uint64_t)6327843723361346336L, PH.base.pack) ;
    p102_x_SET((float)4.7226364E37F, PH.base.pack) ;
    p102_y_SET((float) -2.4526603E38F, PH.base.pack) ;
    p102_z_SET((float)6.734253E37F, PH.base.pack) ;
    p102_roll_SET((float)7.984549E37F, PH.base.pack) ;
    p102_pitch_SET((float)3.1817953E38F, PH.base.pack) ;
    p102_yaw_SET((float)3.324642E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
    p103_usec_SET((uint64_t)3587003765393422852L, PH.base.pack) ;
    p103_x_SET((float) -1.0414447E38F, PH.base.pack) ;
    p103_y_SET((float) -2.9572729E38F, PH.base.pack) ;
    p103_z_SET((float)1.3762161E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
    p104_usec_SET((uint64_t)8966221652826111675L, PH.base.pack) ;
    p104_x_SET((float) -4.3902416E37F, PH.base.pack) ;
    p104_y_SET((float)9.995634E37F, PH.base.pack) ;
    p104_z_SET((float) -3.2127564E38F, PH.base.pack) ;
    p104_roll_SET((float) -1.7106186E38F, PH.base.pack) ;
    p104_pitch_SET((float)9.648687E37F, PH.base.pack) ;
    p104_yaw_SET((float) -2.900095E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIGHRES_IMU_105(), &PH);
    p105_time_usec_SET((uint64_t)4209679166364440042L, PH.base.pack) ;
    p105_xacc_SET((float) -2.1164481E38F, PH.base.pack) ;
    p105_yacc_SET((float) -1.3641874E38F, PH.base.pack) ;
    p105_zacc_SET((float)5.473575E37F, PH.base.pack) ;
    p105_xgyro_SET((float) -1.3833266E38F, PH.base.pack) ;
    p105_ygyro_SET((float)1.7192137E38F, PH.base.pack) ;
    p105_zgyro_SET((float) -3.1889145E37F, PH.base.pack) ;
    p105_xmag_SET((float) -1.7382907E38F, PH.base.pack) ;
    p105_ymag_SET((float) -1.6247434E38F, PH.base.pack) ;
    p105_zmag_SET((float) -4.717971E37F, PH.base.pack) ;
    p105_abs_pressure_SET((float) -1.2320977E38F, PH.base.pack) ;
    p105_diff_pressure_SET((float)3.346056E38F, PH.base.pack) ;
    p105_pressure_alt_SET((float) -8.398252E36F, PH.base.pack) ;
    p105_temperature_SET((float)1.847712E38F, PH.base.pack) ;
    p105_fields_updated_SET((uint16_t)(uint16_t)2943, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
    p106_time_usec_SET((uint64_t)3901395775437466322L, PH.base.pack) ;
    p106_sensor_id_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
    p106_integration_time_us_SET((uint32_t)2280657557L, PH.base.pack) ;
    p106_integrated_x_SET((float) -1.679522E38F, PH.base.pack) ;
    p106_integrated_y_SET((float) -8.338788E37F, PH.base.pack) ;
    p106_integrated_xgyro_SET((float) -1.4587062E38F, PH.base.pack) ;
    p106_integrated_ygyro_SET((float) -1.1371183E38F, PH.base.pack) ;
    p106_integrated_zgyro_SET((float)3.0304569E38F, PH.base.pack) ;
    p106_temperature_SET((int16_t)(int16_t) -20737, PH.base.pack) ;
    p106_quality_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
    p106_time_delta_distance_us_SET((uint32_t)2011156105L, PH.base.pack) ;
    p106_distance_SET((float)8.833478E37F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_SENSOR_107(), &PH);
    p107_time_usec_SET((uint64_t)2385926006176080087L, PH.base.pack) ;
    p107_xacc_SET((float) -2.3898E38F, PH.base.pack) ;
    p107_yacc_SET((float)3.1709584E38F, PH.base.pack) ;
    p107_zacc_SET((float) -4.233658E37F, PH.base.pack) ;
    p107_xgyro_SET((float) -7.9620125E37F, PH.base.pack) ;
    p107_ygyro_SET((float) -2.1178215E38F, PH.base.pack) ;
    p107_zgyro_SET((float)1.7063512E38F, PH.base.pack) ;
    p107_xmag_SET((float)9.276419E37F, PH.base.pack) ;
    p107_ymag_SET((float) -2.8902768E38F, PH.base.pack) ;
    p107_zmag_SET((float)2.6599668E38F, PH.base.pack) ;
    p107_abs_pressure_SET((float)2.5489857E38F, PH.base.pack) ;
    p107_diff_pressure_SET((float) -2.2255902E38F, PH.base.pack) ;
    p107_pressure_alt_SET((float) -5.621408E37F, PH.base.pack) ;
    p107_temperature_SET((float) -7.22004E35F, PH.base.pack) ;
    p107_fields_updated_SET((uint32_t)1102954998L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SIM_STATE_108(), &PH);
    p108_q1_SET((float) -4.5544845E37F, PH.base.pack) ;
    p108_q2_SET((float) -1.2488855E38F, PH.base.pack) ;
    p108_q3_SET((float)1.4891466E38F, PH.base.pack) ;
    p108_q4_SET((float) -2.0196459E38F, PH.base.pack) ;
    p108_roll_SET((float)3.2452478E38F, PH.base.pack) ;
    p108_pitch_SET((float) -1.7319616E38F, PH.base.pack) ;
    p108_yaw_SET((float) -1.9423066E38F, PH.base.pack) ;
    p108_xacc_SET((float) -1.3267286E38F, PH.base.pack) ;
    p108_yacc_SET((float)1.8012498E38F, PH.base.pack) ;
    p108_zacc_SET((float) -2.7687722E38F, PH.base.pack) ;
    p108_xgyro_SET((float) -1.2955917E38F, PH.base.pack) ;
    p108_ygyro_SET((float)1.6126705E38F, PH.base.pack) ;
    p108_zgyro_SET((float) -1.9168372E38F, PH.base.pack) ;
    p108_lat_SET((float)2.9221632E38F, PH.base.pack) ;
    p108_lon_SET((float) -1.8428302E37F, PH.base.pack) ;
    p108_alt_SET((float)5.288635E37F, PH.base.pack) ;
    p108_std_dev_horz_SET((float) -1.9828524E38F, PH.base.pack) ;
    p108_std_dev_vert_SET((float) -3.9568857E37F, PH.base.pack) ;
    p108_vn_SET((float) -1.9386278E38F, PH.base.pack) ;
    p108_ve_SET((float) -1.4456593E37F, PH.base.pack) ;
    p108_vd_SET((float)2.039224E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_RADIO_STATUS_109(), &PH);
    p109_rssi_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
    p109_remrssi_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
    p109_txbuf_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
    p109_noise_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
    p109_remnoise_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
    p109_rxerrors_SET((uint16_t)(uint16_t)45548, PH.base.pack) ;
    p109_fixed__SET((uint16_t)(uint16_t)50614, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
    p110_target_network_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
    p110_target_system_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
    p110_target_component_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
    {
        uint8_t  payload [] =  {(uint8_t)180, (uint8_t)23, (uint8_t)242, (uint8_t)74, (uint8_t)156, (uint8_t)188, (uint8_t)50, (uint8_t)185, (uint8_t)233, (uint8_t)84, (uint8_t)8, (uint8_t)65, (uint8_t)59, (uint8_t)26, (uint8_t)62, (uint8_t)155, (uint8_t)122, (uint8_t)228, (uint8_t)240, (uint8_t)195, (uint8_t)162, (uint8_t)193, (uint8_t)12, (uint8_t)78, (uint8_t)222, (uint8_t)116, (uint8_t)49, (uint8_t)202, (uint8_t)72, (uint8_t)156, (uint8_t)245, (uint8_t)175, (uint8_t)200, (uint8_t)141, (uint8_t)141, (uint8_t)26, (uint8_t)193, (uint8_t)60, (uint8_t)25, (uint8_t)54, (uint8_t)228, (uint8_t)156, (uint8_t)91, (uint8_t)69, (uint8_t)248, (uint8_t)209, (uint8_t)157, (uint8_t)13, (uint8_t)50, (uint8_t)251, (uint8_t)223, (uint8_t)10, (uint8_t)211, (uint8_t)90, (uint8_t)93, (uint8_t)15, (uint8_t)107, (uint8_t)198, (uint8_t)212, (uint8_t)198, (uint8_t)15, (uint8_t)21, (uint8_t)237, (uint8_t)126, (uint8_t)181, (uint8_t)136, (uint8_t)251, (uint8_t)24, (uint8_t)41, (uint8_t)220, (uint8_t)127, (uint8_t)162, (uint8_t)200, (uint8_t)241, (uint8_t)169, (uint8_t)23, (uint8_t)45, (uint8_t)224, (uint8_t)53, (uint8_t)235, (uint8_t)93, (uint8_t)113, (uint8_t)148, (uint8_t)236, (uint8_t)89, (uint8_t)246, (uint8_t)47, (uint8_t)199, (uint8_t)156, (uint8_t)82, (uint8_t)180, (uint8_t)166, (uint8_t)14, (uint8_t)146, (uint8_t)141, (uint8_t)34, (uint8_t)248, (uint8_t)13, (uint8_t)88, (uint8_t)66, (uint8_t)7, (uint8_t)73, (uint8_t)70, (uint8_t)243, (uint8_t)112, (uint8_t)229, (uint8_t)187, (uint8_t)120, (uint8_t)82, (uint8_t)54, (uint8_t)142, (uint8_t)47, (uint8_t)207, (uint8_t)4, (uint8_t)191, (uint8_t)133, (uint8_t)134, (uint8_t)124, (uint8_t)174, (uint8_t)117, (uint8_t)20, (uint8_t)186, (uint8_t)10, (uint8_t)43, (uint8_t)195, (uint8_t)72, (uint8_t)19, (uint8_t)124, (uint8_t)145, (uint8_t)186, (uint8_t)181, (uint8_t)134, (uint8_t)178, (uint8_t)242, (uint8_t)171, (uint8_t)18, (uint8_t)186, (uint8_t)201, (uint8_t)8, (uint8_t)43, (uint8_t)1, (uint8_t)56, (uint8_t)213, (uint8_t)76, (uint8_t)194, (uint8_t)158, (uint8_t)192, (uint8_t)165, (uint8_t)90, (uint8_t)131, (uint8_t)109, (uint8_t)189, (uint8_t)168, (uint8_t)73, (uint8_t)245, (uint8_t)247, (uint8_t)166, (uint8_t)162, (uint8_t)110, (uint8_t)163, (uint8_t)66, (uint8_t)58, (uint8_t)203, (uint8_t)64, (uint8_t)178, (uint8_t)204, (uint8_t)114, (uint8_t)113, (uint8_t)236, (uint8_t)147, (uint8_t)34, (uint8_t)141, (uint8_t)200, (uint8_t)202, (uint8_t)208, (uint8_t)72, (uint8_t)50, (uint8_t)29, (uint8_t)1, (uint8_t)69, (uint8_t)210, (uint8_t)42, (uint8_t)84, (uint8_t)205, (uint8_t)200, (uint8_t)58, (uint8_t)217, (uint8_t)40, (uint8_t)89, (uint8_t)242, (uint8_t)255, (uint8_t)107, (uint8_t)7, (uint8_t)131, (uint8_t)21, (uint8_t)202, (uint8_t)235, (uint8_t)84, (uint8_t)251, (uint8_t)118, (uint8_t)107, (uint8_t)172, (uint8_t)46, (uint8_t)33, (uint8_t)19, (uint8_t)206, (uint8_t)115, (uint8_t)14, (uint8_t)111, (uint8_t)176, (uint8_t)130, (uint8_t)74, (uint8_t)144, (uint8_t)196, (uint8_t)132, (uint8_t)63, (uint8_t)182, (uint8_t)18, (uint8_t)244, (uint8_t)110, (uint8_t)186, (uint8_t)17, (uint8_t)129, (uint8_t)37, (uint8_t)140, (uint8_t)21, (uint8_t)151, (uint8_t)108, (uint8_t)249, (uint8_t)42, (uint8_t)156, (uint8_t)98, (uint8_t)5, (uint8_t)31, (uint8_t)231, (uint8_t)100, (uint8_t)68, (uint8_t)168, (uint8_t)189, (uint8_t)156, (uint8_t)45, (uint8_t)7, (uint8_t)144, (uint8_t)206, (uint8_t)211, (uint8_t)179, (uint8_t)194, (uint8_t)95, (uint8_t)160, (uint8_t)112, (uint8_t)183};
        p110_payload_SET(&payload, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TIMESYNC_111(), &PH);
    p111_tc1_SET((int64_t)7535869707110966252L, PH.base.pack) ;
    p111_ts1_SET((int64_t)3300448876729949580L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_CAMERA_TRIGGER_112(), &PH);
    p112_time_usec_SET((uint64_t)7980117001169952345L, PH.base.pack) ;
    p112_seq_SET((uint32_t)923619548L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_GPS_113(), &PH);
    p113_time_usec_SET((uint64_t)3841261748565391889L, PH.base.pack) ;
    p113_fix_type_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
    p113_lat_SET((int32_t)876431862, PH.base.pack) ;
    p113_lon_SET((int32_t) -1981206341, PH.base.pack) ;
    p113_alt_SET((int32_t)1684712759, PH.base.pack) ;
    p113_eph_SET((uint16_t)(uint16_t)13562, PH.base.pack) ;
    p113_epv_SET((uint16_t)(uint16_t)44397, PH.base.pack) ;
    p113_vel_SET((uint16_t)(uint16_t)14632, PH.base.pack) ;
    p113_vn_SET((int16_t)(int16_t) -25345, PH.base.pack) ;
    p113_ve_SET((int16_t)(int16_t) -10628, PH.base.pack) ;
    p113_vd_SET((int16_t)(int16_t)20280, PH.base.pack) ;
    p113_cog_SET((uint16_t)(uint16_t)36454, PH.base.pack) ;
    p113_satellites_visible_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
    p114_time_usec_SET((uint64_t)4474291204102134513L, PH.base.pack) ;
    p114_sensor_id_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
    p114_integration_time_us_SET((uint32_t)3773541239L, PH.base.pack) ;
    p114_integrated_x_SET((float)1.2987116E37F, PH.base.pack) ;
    p114_integrated_y_SET((float)2.0089777E38F, PH.base.pack) ;
    p114_integrated_xgyro_SET((float) -1.818949E38F, PH.base.pack) ;
    p114_integrated_ygyro_SET((float)2.6187637E38F, PH.base.pack) ;
    p114_integrated_zgyro_SET((float)2.1076813E38F, PH.base.pack) ;
    p114_temperature_SET((int16_t)(int16_t) -6105, PH.base.pack) ;
    p114_quality_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
    p114_time_delta_distance_us_SET((uint32_t)3119304227L, PH.base.pack) ;
    p114_distance_SET((float)2.8505645E38F, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_HIL_STATE_QUATERNION_115(), &PH);
    p115_time_usec_SET((uint64_t)1381084001478272785L, PH.base.pack) ;
    {
        float  attitude_quaternion [] =  {2.6016302E38F, -2.808542E38F, 3.0330804E38F, -1.0014126E38F};
        p115_attitude_quaternion_SET(&attitude_quaternion, 0, &PH.base.pack) ;
    }
    p115_rollspeed_SET((float) -1.4728493E38F, PH.base.pack) ;
    p115_pitchspeed_SET((float) -3.2824784E38F, PH.base.pack) ;
    p115_yawspeed_SET((float)2.5579147E38F, PH.base.pack) ;
    p115_lat_SET((int32_t)1523608469, PH.base.pack) ;
    p115_lon_SET((int32_t) -1439902795, PH.base.pack) ;
    p115_alt_SET((int32_t)507608252, PH.base.pack) ;
    p115_vx_SET((int16_t)(int16_t)6976, PH.base.pack) ;
    p115_vy_SET((int16_t)(int16_t)5183, PH.base.pack) ;
    p115_vz_SET((int16_t)(int16_t) -14703, PH.base.pack) ;
    p115_ind_airspeed_SET((uint16_t)(uint16_t)50248, PH.base.pack) ;
    p115_true_airspeed_SET((uint16_t)(uint16_t)10427, PH.base.pack) ;
    p115_xacc_SET((int16_t)(int16_t)10671, PH.base.pack) ;
    p115_yacc_SET((int16_t)(int16_t)4708, PH.base.pack) ;
    p115_zacc_SET((int16_t)(int16_t) -12412, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU2_116(), &PH);
    p116_time_boot_ms_SET((uint32_t)251939289L, PH.base.pack) ;
    p116_xacc_SET((int16_t)(int16_t) -11805, PH.base.pack) ;
    p116_yacc_SET((int16_t)(int16_t) -6752, PH.base.pack) ;
    p116_zacc_SET((int16_t)(int16_t) -31888, PH.base.pack) ;
    p116_xgyro_SET((int16_t)(int16_t) -5860, PH.base.pack) ;
    p116_ygyro_SET((int16_t)(int16_t)3593, PH.base.pack) ;
    p116_zgyro_SET((int16_t)(int16_t)17554, PH.base.pack) ;
    p116_xmag_SET((int16_t)(int16_t)26010, PH.base.pack) ;
    p116_ymag_SET((int16_t)(int16_t) -6175, PH.base.pack) ;
    p116_zmag_SET((int16_t)(int16_t) -18185, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_LIST_117(), &PH);
    p117_target_system_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
    p117_target_component_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
    p117_start_SET((uint16_t)(uint16_t)35979, PH.base.pack) ;
    p117_end_SET((uint16_t)(uint16_t)300, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ENTRY_118(), &PH);
    p118_id_SET((uint16_t)(uint16_t)65076, PH.base.pack) ;
    p118_num_logs_SET((uint16_t)(uint16_t)12494, PH.base.pack) ;
    p118_last_log_num_SET((uint16_t)(uint16_t)21720, PH.base.pack) ;
    p118_time_utc_SET((uint32_t)2935087007L, PH.base.pack) ;
    p118_size_SET((uint32_t)3652706919L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_DATA_119(), &PH);
    p119_target_system_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
    p119_target_component_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
    p119_id_SET((uint16_t)(uint16_t)7727, PH.base.pack) ;
    p119_ofs_SET((uint32_t)504356969L, PH.base.pack) ;
    p119_count_SET((uint32_t)939109849L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_DATA_120(), &PH);
    p120_id_SET((uint16_t)(uint16_t)2522, PH.base.pack) ;
    p120_ofs_SET((uint32_t)985988306L, PH.base.pack) ;
    p120_count_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)201, (uint8_t)192, (uint8_t)132, (uint8_t)22, (uint8_t)25, (uint8_t)37, (uint8_t)59, (uint8_t)185, (uint8_t)212, (uint8_t)201, (uint8_t)216, (uint8_t)245, (uint8_t)25, (uint8_t)3, (uint8_t)67, (uint8_t)52, (uint8_t)22, (uint8_t)68, (uint8_t)6, (uint8_t)176, (uint8_t)73, (uint8_t)231, (uint8_t)39, (uint8_t)49, (uint8_t)66, (uint8_t)110, (uint8_t)253, (uint8_t)225, (uint8_t)54, (uint8_t)197, (uint8_t)36, (uint8_t)156, (uint8_t)181, (uint8_t)49, (uint8_t)39, (uint8_t)191, (uint8_t)252, (uint8_t)204, (uint8_t)166, (uint8_t)23, (uint8_t)119, (uint8_t)145, (uint8_t)0, (uint8_t)182, (uint8_t)48, (uint8_t)121, (uint8_t)123, (uint8_t)53, (uint8_t)40, (uint8_t)15, (uint8_t)16, (uint8_t)243, (uint8_t)129, (uint8_t)19, (uint8_t)203, (uint8_t)116, (uint8_t)128, (uint8_t)228, (uint8_t)135, (uint8_t)40, (uint8_t)191, (uint8_t)67, (uint8_t)197, (uint8_t)63, (uint8_t)195, (uint8_t)229, (uint8_t)50, (uint8_t)58, (uint8_t)250, (uint8_t)115, (uint8_t)158, (uint8_t)55, (uint8_t)10, (uint8_t)194, (uint8_t)233, (uint8_t)88, (uint8_t)139, (uint8_t)138, (uint8_t)156, (uint8_t)197, (uint8_t)64, (uint8_t)235, (uint8_t)222, (uint8_t)37, (uint8_t)16, (uint8_t)45, (uint8_t)250, (uint8_t)112, (uint8_t)101, (uint8_t)29};
        p120_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_ERASE_121(), &PH);
    p121_target_system_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
    p121_target_component_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_LOG_REQUEST_END_122(), &PH);
    p122_target_system_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
    p122_target_component_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_INJECT_DATA_123(), &PH);
    p123_target_system_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
    p123_target_component_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
    p123_len_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)48, (uint8_t)84, (uint8_t)174, (uint8_t)186, (uint8_t)99, (uint8_t)159, (uint8_t)225, (uint8_t)84, (uint8_t)106, (uint8_t)5, (uint8_t)238, (uint8_t)79, (uint8_t)51, (uint8_t)76, (uint8_t)21, (uint8_t)135, (uint8_t)43, (uint8_t)0, (uint8_t)180, (uint8_t)63, (uint8_t)162, (uint8_t)194, (uint8_t)146, (uint8_t)48, (uint8_t)218, (uint8_t)23, (uint8_t)189, (uint8_t)154, (uint8_t)32, (uint8_t)188, (uint8_t)182, (uint8_t)166, (uint8_t)147, (uint8_t)40, (uint8_t)217, (uint8_t)170, (uint8_t)211, (uint8_t)114, (uint8_t)4, (uint8_t)146, (uint8_t)87, (uint8_t)194, (uint8_t)251, (uint8_t)236, (uint8_t)65, (uint8_t)131, (uint8_t)100, (uint8_t)60, (uint8_t)153, (uint8_t)24, (uint8_t)119, (uint8_t)163, (uint8_t)159, (uint8_t)222, (uint8_t)66, (uint8_t)10, (uint8_t)143, (uint8_t)5, (uint8_t)193, (uint8_t)27, (uint8_t)113, (uint8_t)65, (uint8_t)33, (uint8_t)167, (uint8_t)254, (uint8_t)47, (uint8_t)113, (uint8_t)108, (uint8_t)80, (uint8_t)30, (uint8_t)187, (uint8_t)232, (uint8_t)38, (uint8_t)143, (uint8_t)19, (uint8_t)241, (uint8_t)165, (uint8_t)148, (uint8_t)138, (uint8_t)106, (uint8_t)227, (uint8_t)142, (uint8_t)119, (uint8_t)108, (uint8_t)68, (uint8_t)99, (uint8_t)138, (uint8_t)170, (uint8_t)28, (uint8_t)47, (uint8_t)69, (uint8_t)68, (uint8_t)128, (uint8_t)225, (uint8_t)87, (uint8_t)191, (uint8_t)11, (uint8_t)160, (uint8_t)150, (uint8_t)97, (uint8_t)107, (uint8_t)186, (uint8_t)0, (uint8_t)142, (uint8_t)253, (uint8_t)171, (uint8_t)20, (uint8_t)236, (uint8_t)120, (uint8_t)32};
        p123_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RAW_124(), &PH);
    p124_time_usec_SET((uint64_t)462723372170626788L, PH.base.pack) ;
    p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, PH.base.pack) ;
    p124_lat_SET((int32_t)2071844205, PH.base.pack) ;
    p124_lon_SET((int32_t) -733726296, PH.base.pack) ;
    p124_alt_SET((int32_t) -1533311483, PH.base.pack) ;
    p124_eph_SET((uint16_t)(uint16_t)19523, PH.base.pack) ;
    p124_epv_SET((uint16_t)(uint16_t)63810, PH.base.pack) ;
    p124_vel_SET((uint16_t)(uint16_t)46902, PH.base.pack) ;
    p124_cog_SET((uint16_t)(uint16_t)56737, PH.base.pack) ;
    p124_satellites_visible_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
    p124_dgps_numch_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
    p124_dgps_age_SET((uint32_t)2340773896L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_POWER_STATUS_125(), &PH);
    p125_Vcc_SET((uint16_t)(uint16_t)36326, PH.base.pack) ;
    p125_Vservo_SET((uint16_t)(uint16_t)9845, PH.base.pack) ;
    p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_CHANGED, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SERIAL_CONTROL_126(), &PH);
    p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL, PH.base.pack) ;
    p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND, PH.base.pack) ;
    p126_timeout_SET((uint16_t)(uint16_t)51371, PH.base.pack) ;
    p126_baudrate_SET((uint32_t)1618647378L, PH.base.pack) ;
    p126_count_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)54, (uint8_t)200, (uint8_t)223, (uint8_t)57, (uint8_t)140, (uint8_t)242, (uint8_t)32, (uint8_t)45, (uint8_t)45, (uint8_t)0, (uint8_t)109, (uint8_t)201, (uint8_t)37, (uint8_t)136, (uint8_t)82, (uint8_t)183, (uint8_t)201, (uint8_t)9, (uint8_t)149, (uint8_t)141, (uint8_t)233, (uint8_t)233, (uint8_t)170, (uint8_t)200, (uint8_t)34, (uint8_t)99, (uint8_t)212, (uint8_t)168, (uint8_t)161, (uint8_t)57, (uint8_t)141, (uint8_t)129, (uint8_t)252, (uint8_t)87, (uint8_t)248, (uint8_t)251, (uint8_t)199, (uint8_t)237, (uint8_t)35, (uint8_t)222, (uint8_t)197, (uint8_t)169, (uint8_t)59, (uint8_t)151, (uint8_t)243, (uint8_t)149, (uint8_t)89, (uint8_t)6, (uint8_t)238, (uint8_t)222, (uint8_t)191, (uint8_t)167, (uint8_t)245, (uint8_t)158, (uint8_t)81, (uint8_t)138, (uint8_t)29, (uint8_t)66, (uint8_t)23, (uint8_t)85, (uint8_t)176, (uint8_t)7, (uint8_t)127, (uint8_t)28, (uint8_t)81, (uint8_t)242, (uint8_t)145, (uint8_t)199, (uint8_t)131, (uint8_t)46};
        p126_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS_RTK_127(), &PH);
    p127_time_last_baseline_ms_SET((uint32_t)2996849311L, PH.base.pack) ;
    p127_rtk_receiver_id_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
    p127_wn_SET((uint16_t)(uint16_t)28380, PH.base.pack) ;
    p127_tow_SET((uint32_t)1992081300L, PH.base.pack) ;
    p127_rtk_health_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
    p127_rtk_rate_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
    p127_nsats_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
    p127_baseline_coords_type_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
    p127_baseline_a_mm_SET((int32_t)624590611, PH.base.pack) ;
    p127_baseline_b_mm_SET((int32_t)130663848, PH.base.pack) ;
    p127_baseline_c_mm_SET((int32_t) -206424854, PH.base.pack) ;
    p127_accuracy_SET((uint32_t)1303207994L, PH.base.pack) ;
    p127_iar_num_hypotheses_SET((int32_t)1380269448, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_GPS2_RTK_128(), &PH);
    p128_time_last_baseline_ms_SET((uint32_t)2809733335L, PH.base.pack) ;
    p128_rtk_receiver_id_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
    p128_wn_SET((uint16_t)(uint16_t)24658, PH.base.pack) ;
    p128_tow_SET((uint32_t)1136253209L, PH.base.pack) ;
    p128_rtk_health_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
    p128_rtk_rate_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
    p128_nsats_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
    p128_baseline_coords_type_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
    p128_baseline_a_mm_SET((int32_t) -1078183315, PH.base.pack) ;
    p128_baseline_b_mm_SET((int32_t)761641827, PH.base.pack) ;
    p128_baseline_c_mm_SET((int32_t) -2143987923, PH.base.pack) ;
    p128_accuracy_SET((uint32_t)2136630929L, PH.base.pack) ;
    p128_iar_num_hypotheses_SET((int32_t)1083011047, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_IMU3_129(), &PH);
    p129_time_boot_ms_SET((uint32_t)2256415698L, PH.base.pack) ;
    p129_xacc_SET((int16_t)(int16_t) -25556, PH.base.pack) ;
    p129_yacc_SET((int16_t)(int16_t) -17234, PH.base.pack) ;
    p129_zacc_SET((int16_t)(int16_t)29057, PH.base.pack) ;
    p129_xgyro_SET((int16_t)(int16_t)14014, PH.base.pack) ;
    p129_ygyro_SET((int16_t)(int16_t) -24675, PH.base.pack) ;
    p129_zgyro_SET((int16_t)(int16_t) -25113, PH.base.pack) ;
    p129_xmag_SET((int16_t)(int16_t)24390, PH.base.pack) ;
    p129_ymag_SET((int16_t)(int16_t) -12660, PH.base.pack) ;
    p129_zmag_SET((int16_t)(int16_t) -18090, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
    p130_type_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
    p130_size_SET((uint32_t)1317546812L, PH.base.pack) ;
    p130_width_SET((uint16_t)(uint16_t)28169, PH.base.pack) ;
    p130_height_SET((uint16_t)(uint16_t)55217, PH.base.pack) ;
    p130_packets_SET((uint16_t)(uint16_t)1605, PH.base.pack) ;
    p130_payload_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
    p130_jpg_quality_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_ENCAPSULATED_DATA_131(), &PH);
    p131_seqnr_SET((uint16_t)(uint16_t)3148, PH.base.pack) ;
    {
        uint8_t  data_ [] =  {(uint8_t)4, (uint8_t)13, (uint8_t)212, (uint8_t)156, (uint8_t)252, (uint8_t)234, (uint8_t)81, (uint8_t)153, (uint8_t)28, (uint8_t)131, (uint8_t)178, (uint8_t)229, (uint8_t)121, (uint8_t)202, (uint8_t)16, (uint8_t)148, (uint8_t)57, (uint8_t)193, (uint8_t)45, (uint8_t)177, (uint8_t)101, (uint8_t)248, (uint8_t)86, (uint8_t)155, (uint8_t)146, (uint8_t)118, (uint8_t)197, (uint8_t)63, (uint8_t)14, (uint8_t)91, (uint8_t)29, (uint8_t)8, (uint8_t)97, (uint8_t)66, (uint8_t)20, (uint8_t)188, (uint8_t)35, (uint8_t)154, (uint8_t)138, (uint8_t)139, (uint8_t)19, (uint8_t)199, (uint8_t)80, (uint8_t)66, (uint8_t)227, (uint8_t)8, (uint8_t)132, (uint8_t)158, (uint8_t)32, (uint8_t)115, (uint8_t)12, (uint8_t)140, (uint8_t)65, (uint8_t)141, (uint8_t)229, (uint8_t)144, (uint8_t)72, (uint8_t)27, (uint8_t)188, (uint8_t)46, (uint8_t)228, (uint8_t)166, (uint8_t)194, (uint8_t)148, (uint8_t)238, (uint8_t)35, (uint8_t)146, (uint8_t)216, (uint8_t)148, (uint8_t)254, (uint8_t)98, (uint8_t)207, (uint8_t)26, (uint8_t)120, (uint8_t)41, (uint8_t)226, (uint8_t)17, (uint8_t)198, (uint8_t)114, (uint8_t)198, (uint8_t)213, (uint8_t)86, (uint8_t)142, (uint8_t)249, (uint8_t)22, (uint8_t)121, (uint8_t)4, (uint8_t)223, (uint8_t)111, (uint8_t)234, (uint8_t)11, (uint8_t)102, (uint8_t)247, (uint8_t)20, (uint8_t)54, (uint8_t)223, (uint8_t)176, (uint8_t)71, (uint8_t)218, (uint8_t)155, (uint8_t)106, (uint8_t)251, (uint8_t)9, (uint8_t)228, (uint8_t)119, (uint8_t)78, (uint8_t)184, (uint8_t)178, (uint8_t)66, (uint8_t)221, (uint8_t)19, (uint8_t)242, (uint8_t)228, (uint8_t)162, (uint8_t)68, (uint8_t)16, (uint8_t)42, (uint8_t)100, (uint8_t)61, (uint8_t)110, (uint8_t)171, (uint8_t)131, (uint8_t)110, (uint8_t)132, (uint8_t)151, (uint8_t)213, (uint8_t)11, (uint8_t)178, (uint8_t)190, (uint8_t)225, (uint8_t)159, (uint8_t)95, (uint8_t)205, (uint8_t)101, (uint8_t)1, (uint8_t)71, (uint8_t)231, (uint8_t)15, (uint8_t)15, (uint8_t)252, (uint8_t)116, (uint8_t)148, (uint8_t)150, (uint8_t)74, (uint8_t)159, (uint8_t)221, (uint8_t)50, (uint8_t)104, (uint8_t)236, (uint8_t)156, (uint8_t)196, (uint8_t)112, (uint8_t)50, (uint8_t)250, (uint8_t)192, (uint8_t)152, (uint8_t)208, (uint8_t)30, (uint8_t)118, (uint8_t)70, (uint8_t)30, (uint8_t)136, (uint8_t)163, (uint8_t)119, (uint8_t)150, (uint8_t)110, (uint8_t)146, (uint8_t)214, (uint8_t)167, (uint8_t)241, (uint8_t)199, (uint8_t)223, (uint8_t)177, (uint8_t)15, (uint8_t)131, (uint8_t)34, (uint8_t)196, (uint8_t)207, (uint8_t)152, (uint8_t)16, (uint8_t)155, (uint8_t)206, (uint8_t)108, (uint8_t)79, (uint8_t)134, (uint8_t)3, (uint8_t)100, (uint8_t)152, (uint8_t)247, (uint8_t)118, (uint8_t)47, (uint8_t)57, (uint8_t)14, (uint8_t)242, (uint8_t)84, (uint8_t)14, (uint8_t)111, (uint8_t)121, (uint8_t)246, (uint8_t)45, (uint8_t)146, (uint8_t)128, (uint8_t)220, (uint8_t)44, (uint8_t)187, (uint8_t)42, (uint8_t)82, (uint8_t)96, (uint8_t)140, (uint8_t)58, (uint8_t)41, (uint8_t)183, (uint8_t)218, (uint8_t)118, (uint8_t)225, (uint8_t)46, (uint8_t)99, (uint8_t)159, (uint8_t)27, (uint8_t)249, (uint8_t)49, (uint8_t)212, (uint8_t)86, (uint8_t)211, (uint8_t)219, (uint8_t)51, (uint8_t)219, (uint8_t)76, (uint8_t)125, (uint8_t)152, (uint8_t)242, (uint8_t)20, (uint8_t)111, (uint8_t)181, (uint8_t)192, (uint8_t)158, (uint8_t)23, (uint8_t)141, (uint8_t)170, (uint8_t)21, (uint8_t)9, (uint8_t)51, (uint8_t)219, (uint8_t)100, (uint8_t)151, (uint8_t)122, (uint8_t)115, (uint8_t)212, (uint8_t)145, (uint8_t)102, (uint8_t)155, (uint8_t)144, (uint8_t)15};
        p131_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_DISTANCE_SENSOR_132(), &PH);
    p132_time_boot_ms_SET((uint32_t)1633299204L, PH.base.pack) ;
    p132_min_distance_SET((uint16_t)(uint16_t)50777, PH.base.pack) ;
    p132_max_distance_SET((uint16_t)(uint16_t)36995, PH.base.pack) ;
    p132_current_distance_SET((uint16_t)(uint16_t)42652, PH.base.pack) ;
    p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND, PH.base.pack) ;
    p132_id_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
    p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_NONE, PH.base.pack) ;
    p132_covariance_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REQUEST_133(), &PH);
    p133_lat_SET((int32_t) -1919602746, PH.base.pack) ;
    p133_lon_SET((int32_t)1315763769, PH.base.pack) ;
    p133_grid_spacing_SET((uint16_t)(uint16_t)57182, PH.base.pack) ;
    p133_mask_SET((uint64_t)4817772823485641370L, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_DATA_134(), &PH);
    p134_lat_SET((int32_t)635954529, PH.base.pack) ;
    p134_lon_SET((int32_t) -215781403, PH.base.pack) ;
    p134_grid_spacing_SET((uint16_t)(uint16_t)17894, PH.base.pack) ;
    p134_gridbit_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
    {
        int16_t  data_ [] =  {(int16_t)30863, (int16_t) -3799, (int16_t)28587, (int16_t) -25328, (int16_t)5877, (int16_t)7423, (int16_t) -14429, (int16_t) -30033, (int16_t) -13648, (int16_t) -26305, (int16_t)22716, (int16_t)1100, (int16_t)6481, (int16_t)11787, (int16_t)346, (int16_t)10395};
        p134_data__SET(&data_, 0, &PH.base.pack) ;
    }
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_CHECK_135(), &PH);
    p135_lat_SET((int32_t) -1476316884, PH.base.pack) ;
    p135_lon_SET((int32_t)1095792280, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_TERRAIN_REPORT_136(), &PH);
    p136_lat_SET((int32_t)1632480494, PH.base.pack) ;
    p136_lon_SET((int32_t)513586654, PH.base.pack) ;
    p136_spacing_SET((uint16_t)(uint16_t)14339, PH.base.pack) ;
    p136_terrain_height_SET((float)2.4732457E38F, PH.base.pack) ;
    p136_current_height_SET((float)2.5015052E38F, PH.base.pack) ;
    p136_pending_SET((uint16_t)(uint16_t)55551, PH.base.pack) ;
    p136_loaded_SET((uint16_t)(uint16_t)13865, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    setPack(c_CommunicationChannel_new_SCALED_PRESSURE2_137(), &PH);
    p137_time_boot_ms_SET((uint32_t)118689760L, PH.base.pack) ;
    p137_press_abs_SET((float) -3.12474E38F, PH.base.pack) ;
    p137_press_diff_SET((float)2.735448E38F, PH.base.pack) ;
    p137_temperature_SET((int16_t)(int16_t) -2980, PH.base.pack) ;
    c_CommunicationChannel_send(PH.base.pack);//put pack to the c_CommunicationChannel send buffer
    // yours initialization logic code here
    uint8_t buff[512];
    while(true)  //main working LOOP(very typical for microcontrollers without any OS)
    {
        // yours main loop code here
    }
}
