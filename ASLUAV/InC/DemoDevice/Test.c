
#pragma once
#include "DemoDevice.h"
#include <string.h>
#include <stdio.h>
#include <assert.h>

INLINER int32_t Arrays_equals(uint8_t * L, uint8_t * R, int32_t bytes)
{
    for(int32_t i = 0; i < bytes; i++) if(L[i] != R[i]) return i;
    return -1;
}
static Pack * zero2empty(Pack * pack)
{
    pack = realloc(pack, sizeof(Pack) + pack->meta->packMinBytes);
    for(int32_t i = pack->meta->packMinBytes; -1 < --i;)pack->data[i] = 0; //clean bytes
    return pack;
}
void failure(Channel * ch, int32_t id, int32_t arg) {assert(false);}




void c_LoopBackDemoChannel_on_HEARTBEAT_0(Bounds_Inside * ph, Pack * pack)
{
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_FREE_BALLOON);
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_CRITICAL);
    assert(p0_custom_mode_GET(pack) == (uint32_t)382891060L);
    assert(p0_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_UDB);
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)105);
};


void c_LoopBackDemoChannel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_onboard_control_sensors_health_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)24952);
    assert(p1_onboard_control_sensors_present_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)38257);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)47266);
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)61465);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)32967);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)26772);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)478);
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t)120);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t) -5895);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)49041);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION);
};


void c_LoopBackDemoChannel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)2001467072119379248L);
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)2000782192L);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_vz_GET(pack) == (float) -2.6772925E38F);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)4219490201L);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    assert(p3_z_GET(pack) == (float)2.4108416E38F);
    assert(p3_y_GET(pack) == (float) -3.3853051E38F);
    assert(p3_afx_GET(pack) == (float)6.7673806E37F);
    assert(p3_afz_GET(pack) == (float) -2.201726E38F);
    assert(p3_afy_GET(pack) == (float)1.1247662E38F);
    assert(p3_yaw_rate_GET(pack) == (float) -4.646478E37F);
    assert(p3_x_GET(pack) == (float)3.5905156E37F);
    assert(p3_yaw_GET(pack) == (float) -1.4891242E38F);
    assert(p3_vx_GET(pack) == (float)1.6977839E38F);
    assert(p3_vy_GET(pack) == (float)2.4366265E38F);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)11547);
};


void c_LoopBackDemoChannel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p4_time_usec_GET(pack) == (uint64_t)4394367281331872007L);
    assert(p4_seq_GET(pack) == (uint32_t)391320775L);
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)20);
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)5);
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)174);
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p5_passkey_LEN(ph) == 10);
    {
        char16_t * exemplary = u"jFbmdfjdep";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)138);
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)111);
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)79);
};


void c_LoopBackDemoChannel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 3);
    {
        char16_t * exemplary = u"vla";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)220);
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_PREFLIGHT);
    assert(p11_custom_mode_GET(pack) == (uint32_t)4287840112L);
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p20_param_id_LEN(ph) == 12);
    {
        char16_t * exemplary = u"dWhtgpbugzyq";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)64);
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t) -26478);
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)205);
};


void c_LoopBackDemoChannel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64);
    assert(p22_param_value_GET(pack) == (float)1.8036741E38F);
    assert(p22_param_id_LEN(ph) == 3);
    {
        char16_t * exemplary = u"Uac";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)47818);
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)43310);
};


void c_LoopBackDemoChannel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p23_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"eneMdtauuJqvc";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)106);
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL32);
    assert(p23_param_value_GET(pack) == (float) -1.0370037E38F);
};


void c_LoopBackDemoChannel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)1035320536L);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)3383337681L);
    assert(p24_v_acc_TRY(ph) == (uint32_t)2848072965L);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)21355);
    assert(p24_alt_GET(pack) == (int32_t) -1918439573);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX);
    assert(p24_lat_GET(pack) == (int32_t) -1432525086);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)8459);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)50669);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t) -1457148080);
    assert(p24_h_acc_TRY(ph) == (uint32_t)3854070657L);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p24_time_usec_GET(pack) == (uint64_t)8189189502200789322L);
    assert(p24_lon_GET(pack) == (int32_t)629163508);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)4912);
};


void c_LoopBackDemoChannel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)97, (uint8_t)12, (uint8_t)71, (uint8_t)165, (uint8_t)238, (uint8_t)133, (uint8_t)84, (uint8_t)140, (uint8_t)183, (uint8_t)176, (uint8_t)182, (uint8_t)79, (uint8_t)25, (uint8_t)34, (uint8_t)137, (uint8_t)238, (uint8_t)143, (uint8_t)249, (uint8_t)49, (uint8_t)140} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)17, (uint8_t)114, (uint8_t)102, (uint8_t)216, (uint8_t)216, (uint8_t)83, (uint8_t)147, (uint8_t)49, (uint8_t)159, (uint8_t)106, (uint8_t)175, (uint8_t)77, (uint8_t)161, (uint8_t)106, (uint8_t)219, (uint8_t)75, (uint8_t)114, (uint8_t)208, (uint8_t)235, (uint8_t)253} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)168, (uint8_t)243, (uint8_t)250, (uint8_t)186, (uint8_t)93, (uint8_t)190, (uint8_t)21, (uint8_t)96, (uint8_t)61, (uint8_t)29, (uint8_t)112, (uint8_t)64, (uint8_t)100, (uint8_t)222, (uint8_t)199, (uint8_t)190, (uint8_t)226, (uint8_t)26, (uint8_t)66, (uint8_t)24} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)58, (uint8_t)212, (uint8_t)174, (uint8_t)44, (uint8_t)215, (uint8_t)198, (uint8_t)254, (uint8_t)177, (uint8_t)179, (uint8_t)72, (uint8_t)20, (uint8_t)162, (uint8_t)9, (uint8_t)238, (uint8_t)77, (uint8_t)167, (uint8_t)50, (uint8_t)103, (uint8_t)134, (uint8_t)232} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)219, (uint8_t)148, (uint8_t)119, (uint8_t)32, (uint8_t)100, (uint8_t)160, (uint8_t)133, (uint8_t)181, (uint8_t)12, (uint8_t)17, (uint8_t)219, (uint8_t)110, (uint8_t)2, (uint8_t)232, (uint8_t)179, (uint8_t)179, (uint8_t)104, (uint8_t)16, (uint8_t)78, (uint8_t)52} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)189);
};


void c_LoopBackDemoChannel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t) -8672);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)4036912829L);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t) -7041);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t)2172);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t) -11545);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t)9691);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t) -32235);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t)615);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t)20327);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t)32131);
};


void c_LoopBackDemoChannel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t) -24088);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t)16022);
    assert(p27_time_usec_GET(pack) == (uint64_t)7216798221575728225L);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t)6515);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t)31197);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t)3928);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t)25495);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t)25559);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t)30789);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t) -25817);
};


void c_LoopBackDemoChannel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t) -32212);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t) -8857);
    assert(p28_time_usec_GET(pack) == (uint64_t)243619827054177068L);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t)32601);
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t) -26180);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)3033124254L);
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t)2046);
    assert(p29_press_diff_GET(pack) == (float)2.9726248E38F);
    assert(p29_press_abs_GET(pack) == (float)1.3179264E37F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_yaw_GET(pack) == (float) -1.1631537E37F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)3139756574L);
    assert(p30_rollspeed_GET(pack) == (float) -1.4567409E38F);
    assert(p30_roll_GET(pack) == (float) -6.6726704E37F);
    assert(p30_yawspeed_GET(pack) == (float)1.0771738E38F);
    assert(p30_pitch_GET(pack) == (float)1.1084469E38F);
    assert(p30_pitchspeed_GET(pack) == (float) -9.2073165E36F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_rollspeed_GET(pack) == (float)8.948354E37F);
    assert(p31_q4_GET(pack) == (float) -5.658694E37F);
    assert(p31_q2_GET(pack) == (float) -1.8149124E38F);
    assert(p31_q1_GET(pack) == (float)1.2481478E38F);
    assert(p31_pitchspeed_GET(pack) == (float)2.6883168E38F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)4170885047L);
    assert(p31_q3_GET(pack) == (float) -4.365963E37F);
    assert(p31_yawspeed_GET(pack) == (float) -1.5073941E37F);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_vz_GET(pack) == (float)1.7343537E38F);
    assert(p32_z_GET(pack) == (float) -8.862546E37F);
    assert(p32_vx_GET(pack) == (float) -2.1408738E38F);
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)1446325996L);
    assert(p32_y_GET(pack) == (float) -2.4837753E37F);
    assert(p32_x_GET(pack) == (float) -1.9297512E38F);
    assert(p32_vy_GET(pack) == (float) -8.3731015E37F);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_vy_GET(pack) == (int16_t)(int16_t)8782);
    assert(p33_lon_GET(pack) == (int32_t) -125045043);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t)29727);
    assert(p33_lat_GET(pack) == (int32_t) -616424532);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)1213190351L);
    assert(p33_alt_GET(pack) == (int32_t)2060600999);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t) -25042);
    assert(p33_relative_alt_GET(pack) == (int32_t) -810805129);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)481);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)94824788L);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t)28711);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t)15877);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t) -3430);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t)29843);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t)14041);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t)26924);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t)6516);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)90);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t) -17343);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)56467);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)38835);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)22532);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)39008);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)35628);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)32682);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)4079824296L);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)59059);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)715);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)55);
};


void c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)51195);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)28257);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)63659);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)29953);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)63443);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)8367);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)30050);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)49701);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)46912);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)19340);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)36544);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)23622);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)63239);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)3734);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)21468);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)53147);
    assert(p36_time_usec_GET(pack) == (uint32_t)1046268007L);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t)28078);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)7);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t)28941);
};


void c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t)25357);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)204);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)246);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t) -14403);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_y_GET(pack) == (float) -1.5484021E38F);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_USER_3);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p39_param1_GET(pack) == (float)3.809215E37F);
    assert(p39_z_GET(pack) == (float) -1.9231929E38F);
    assert(p39_param2_GET(pack) == (float) -1.6415855E38F);
    assert(p39_x_GET(pack) == (float) -2.4108134E38F);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p39_param3_GET(pack) == (float)2.624958E38F);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)26695);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p39_param4_GET(pack) == (float) -1.2472369E38F);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)14774);
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)150);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)157);
};


void c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)25023);
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)101);
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)109);
};


void c_LoopBackDemoChannel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)51729);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)38);
};


void c_LoopBackDemoChannel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)49);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)64);
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)62855);
};


void c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)143);
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)178);
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)3348);
};


void c_LoopBackDemoChannel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM6_Y);
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)224);
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
};


void c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_time_usec_TRY(ph) == (uint64_t)7111868083256045978L);
    assert(p48_latitude_GET(pack) == (int32_t) -1757679436);
    assert(p48_longitude_GET(pack) == (int32_t)1065410464);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p48_altitude_GET(pack) == (int32_t) -1661531009);
};


void c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_longitude_GET(pack) == (int32_t)2125369841);
    assert(p49_time_usec_TRY(ph) == (uint64_t)1496616033139386054L);
    assert(p49_latitude_GET(pack) == (int32_t) -897162437);
    assert(p49_altitude_GET(pack) == (int32_t) -439213338);
};


void c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)189);
    assert(p50_param_value_min_GET(pack) == (float) -3.9886535E37F);
    assert(p50_param_value0_GET(pack) == (float)9.546097E37F);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t)28572);
    assert(p50_scale_GET(pack) == (float) -6.224224E37F);
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)13);
    assert(p50_param_id_LEN(ph) == 15);
    {
        char16_t * exemplary = u"pldhosxbXfjWheq";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_param_value_max_GET(pack) == (float)2.9222707E38F);
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)221);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)116);
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)23153);
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)254);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
};


void c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_p1z_GET(pack) == (float)1.0655978E38F);
    assert(p54_p1x_GET(pack) == (float) -9.447388E37F);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)193);
    assert(p54_p2x_GET(pack) == (float)2.4337247E38F);
    assert(p54_p1y_GET(pack) == (float)2.3931024E38F);
    assert(p54_p2y_GET(pack) == (float) -1.979168E38F);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)133);
    assert(p54_p2z_GET(pack) == (float)1.3778072E38F);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
};


void c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p2z_GET(pack) == (float)2.7835837E38F);
    assert(p55_p1y_GET(pack) == (float)2.323834E38F);
    assert(p55_p2x_GET(pack) == (float) -1.961659E38F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p55_p1x_GET(pack) == (float) -2.100223E38F);
    assert(p55_p2y_GET(pack) == (float) -4.100305E37F);
    assert(p55_p1z_GET(pack) == (float)2.1455272E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    assert(p61_rollspeed_GET(pack) == (float)1.7116438E37F);
    assert(p61_yawspeed_GET(pack) == (float)1.7629446E38F);
    assert(p61_time_usec_GET(pack) == (uint64_t)1395623904185529744L);
    {
        float exemplary[] =  {-1.4502536E38F, -1.570404E38F, -1.5137144E38F, 2.3692E38F, 1.1128518E38F, -2.5696424E38F, -3.14519E38F, -4.7458192E36F, 3.3965801E38F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_pitchspeed_GET(pack) == (float) -2.4582138E38F);
    {
        float exemplary[] =  {1.7980267E38F, 2.7932182E38F, 9.812939E36F, 1.8663496E38F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_nav_pitch_GET(pack) == (float) -7.0518815E37F);
    assert(p62_xtrack_error_GET(pack) == (float)2.915725E38F);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t) -1784);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t) -6768);
    assert(p62_alt_error_GET(pack) == (float)2.5611362E37F);
    assert(p62_nav_roll_GET(pack) == (float) -6.366582E37F);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)37118);
    assert(p62_aspd_error_GET(pack) == (float)2.8150358E38F);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_vz_GET(pack) == (float) -1.7804214E38F);
    assert(p63_lon_GET(pack) == (int32_t)2096167018);
    assert(p63_vx_GET(pack) == (float)2.5237445E38F);
    assert(p63_alt_GET(pack) == (int32_t)842713801);
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS);
    assert(p63_time_usec_GET(pack) == (uint64_t)4508598773115017152L);
    assert(p63_relative_alt_GET(pack) == (int32_t) -638264417);
    assert(p63_vy_GET(pack) == (float)6.8101293E37F);
    assert(p63_lat_GET(pack) == (int32_t) -525078506);
    {
        float exemplary[] =  {-5.4249437E37F, 2.9416879E38F, -4.4168415E37F, 1.3104747E38F, 9.382291E37F, 1.5998572E38F, 3.3507632E38F, -2.292925E38F, -1.6868603E38F, 2.7172515E38F, 8.4289075E37F, -2.9789647E38F, -2.2398934E38F, 3.39837E38F, -2.2064065E38F, -1.289841E38F, 2.874709E38F, 3.0594506E37F, -5.495724E36F, -2.1821603E38F, -1.2167034E38F, -2.8934813E38F, 3.0227982E38F, 2.3812163E38F, -5.312995E36F, 2.7701707E38F, 7.2185917E37F, -5.686211E37F, 9.375075E37F, 3.0961934E38F, 1.2841155E38F, -1.2661921E38F, -3.1126083E38F, -4.835894E36F, 5.3859016E37F, -2.2911245E38F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE);
    {
        float exemplary[] =  {1.5765722E38F, -3.2556983E38F, -2.11587E38F, 1.3596571E38F, -3.3975196E38F, 1.4609596E38F, 1.9316523E38F, 4.0557114E37F, -1.672904E38F, 3.2760247E38F, 5.397551E37F, 2.2322449E38F, 3.1968988E38F, 8.638095E37F, -2.5987567E38F, -1.8142236E38F, -1.7727702E38F, -8.3979226E37F, 8.53531E37F, 2.8176366E38F, 1.982721E38F, -3.8904625E37F, -1.7831976E38F, -1.2906652E36F, -5.656256E37F, 2.0214066E38F, 2.9967734E37F, -1.3097519E38F, 2.8692083E38F, 3.3332102E37F, 1.4555263E38F, 3.0342183E38F, 2.9484949E38F, 2.171893E38F, 2.6226425E38F, 2.4110104E38F, -2.1600574E38F, -1.5622081E38F, -1.2029982E38F, -1.4067521E38F, 4.1617862E37F, -1.457584E38F, 2.5611339E38F, -1.8860497E38F, -2.1995616E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_vx_GET(pack) == (float) -1.7773027E38F);
    assert(p64_ay_GET(pack) == (float) -4.350729E37F);
    assert(p64_ax_GET(pack) == (float) -2.2648166E38F);
    assert(p64_az_GET(pack) == (float)1.0936082E38F);
    assert(p64_time_usec_GET(pack) == (uint64_t)729078912293458530L);
    assert(p64_x_GET(pack) == (float)3.2786862E38F);
    assert(p64_vy_GET(pack) == (float) -1.3042387E37F);
    assert(p64_y_GET(pack) == (float)1.5247751E37F);
    assert(p64_vz_GET(pack) == (float)2.968266E38F);
    assert(p64_z_GET(pack) == (float) -1.4282688E38F);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)57560);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)47563);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)27043);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)47307);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)18367);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)28653);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)53473);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)12462);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)54888);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)26822);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)49352);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)62487);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)44647);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)1212031166L);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)7121);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)6245);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)30897);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)45933);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)29832);
};


void c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)12839);
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)188);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)239);
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)77);
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)133);
};


void c_LoopBackDemoChannel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)26);
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)49572);
};


void c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_r_GET(pack) == (int16_t)(int16_t)8985);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)40131);
    assert(p69_x_GET(pack) == (int16_t)(int16_t) -337);
    assert(p69_y_GET(pack) == (int16_t)(int16_t) -16442);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)199);
    assert(p69_z_GET(pack) == (int16_t)(int16_t)10460);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)28780);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)23214);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)57302);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)57586);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)107);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)20207);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)45353);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)47592);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)33757);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2);
    assert(p73_param4_GET(pack) == (float) -1.9195625E38F);
    assert(p73_z_GET(pack) == (float)2.7857813E38F);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)16);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)50);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)2310);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p73_x_GET(pack) == (int32_t) -1530355114);
    assert(p73_param2_GET(pack) == (float) -2.6976566E38F);
    assert(p73_param3_GET(pack) == (float) -5.729489E37F);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)41);
    assert(p73_y_GET(pack) == (int32_t) -988671962);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_NED);
    assert(p73_param1_GET(pack) == (float) -1.7446746E38F);
};


void c_LoopBackDemoChannel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)47373);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t) -6582);
    assert(p74_alt_GET(pack) == (float) -2.7094224E38F);
    assert(p74_groundspeed_GET(pack) == (float)1.3487866E38F);
    assert(p74_airspeed_GET(pack) == (float) -1.6297735E38F);
    assert(p74_climb_GET(pack) == (float) -7.4897493E37F);
};


void c_LoopBackDemoChannel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_z_GET(pack) == (float) -1.020097E38F);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE);
    assert(p75_x_GET(pack) == (int32_t)1481735238);
    assert(p75_param3_GET(pack) == (float)3.8557576E37F);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p75_y_GET(pack) == (int32_t) -172980655);
    assert(p75_param4_GET(pack) == (float) -2.6409746E38F);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)88);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)153);
    assert(p75_param2_GET(pack) == (float) -8.548695E37F);
    assert(p75_param1_GET(pack) == (float) -7.370007E37F);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)88);
};


void c_LoopBackDemoChannel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_param7_GET(pack) == (float)4.347604E37F);
    assert(p76_param2_GET(pack) == (float)3.3006835E38F);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)113);
    assert(p76_param4_GET(pack) == (float)2.356487E38F);
    assert(p76_param3_GET(pack) == (float)1.598852E38F);
    assert(p76_param1_GET(pack) == (float) -1.5383248E38F);
    assert(p76_param6_GET(pack) == (float) -5.4729065E37F);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)176);
    assert(p76_param5_GET(pack) == (float)3.0634476E38F);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)237);
};


void c_LoopBackDemoChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_UNSUPPORTED);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)32);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)146);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)59);
    assert(p77_result_param2_TRY(ph) == (int32_t)159139809);
};


void c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)2762761990L);
    assert(p81_pitch_GET(pack) == (float)2.812498E38F);
    assert(p81_thrust_GET(pack) == (float)1.6839498E38F);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)39);
    assert(p81_yaw_GET(pack) == (float) -1.0463766E38F);
    assert(p81_roll_GET(pack) == (float) -2.16264E38F);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)109);
};


void c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)165);
    assert(p82_body_yaw_rate_GET(pack) == (float) -3.388621E37F);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)243);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)182);
    {
        float exemplary[] =  {2.7532846E38F, 2.7016567E38F, -8.953648E37F, 5.068591E37F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_body_pitch_rate_GET(pack) == (float) -3.0050773E37F);
    assert(p82_body_roll_rate_GET(pack) == (float) -2.5303894E38F);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)2625049059L);
    assert(p82_thrust_GET(pack) == (float) -5.3324103E37F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_body_pitch_rate_GET(pack) == (float)6.9882855E37F);
    {
        float exemplary[] =  {-1.1101187E38F, 2.3889826E36F, -6.286642E37F, 1.0802368E38F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p83_body_yaw_rate_GET(pack) == (float)2.1996612E38F);
    assert(p83_body_roll_rate_GET(pack) == (float) -2.29858E38F);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)924478959L);
    assert(p83_thrust_GET(pack) == (float)3.1946614E38F);
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)150);
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_y_GET(pack) == (float)9.661816E37F);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_INT);
    assert(p84_yaw_rate_GET(pack) == (float) -1.9125884E38F);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)73);
    assert(p84_z_GET(pack) == (float)1.643459E38F);
    assert(p84_vz_GET(pack) == (float)2.3674448E38F);
    assert(p84_afx_GET(pack) == (float) -1.9017676E38F);
    assert(p84_x_GET(pack) == (float)1.6778579E38F);
    assert(p84_afz_GET(pack) == (float)1.1613883E38F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)4224975782L);
    assert(p84_yaw_GET(pack) == (float)2.7142034E38F);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)138);
    assert(p84_afy_GET(pack) == (float) -8.5038E37F);
    assert(p84_vy_GET(pack) == (float) -2.9362713E38F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)15538);
    assert(p84_vx_GET(pack) == (float) -3.07721E38F);
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)2916312152L);
    assert(p86_afx_GET(pack) == (float)2.321499E38F);
    assert(p86_afy_GET(pack) == (float) -3.0702422E38F);
    assert(p86_vx_GET(pack) == (float) -2.803274E38F);
    assert(p86_afz_GET(pack) == (float) -2.8523834E38F);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)171);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)8621);
    assert(p86_vz_GET(pack) == (float)2.7713643E38F);
    assert(p86_yaw_rate_GET(pack) == (float) -1.0091898E38F);
    assert(p86_lat_int_GET(pack) == (int32_t)754665512);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
    assert(p86_alt_GET(pack) == (float)2.607765E37F);
    assert(p86_lon_int_GET(pack) == (int32_t) -1000026484);
    assert(p86_yaw_GET(pack) == (float)2.9290568E38F);
    assert(p86_vy_GET(pack) == (float)1.7814503E38F);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)61);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_yaw_rate_GET(pack) == (float)1.094423E38F);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p87_afz_GET(pack) == (float)1.3092844E38F);
    assert(p87_lat_int_GET(pack) == (int32_t) -1197576590);
    assert(p87_vx_GET(pack) == (float) -7.57192E37F);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)117344157L);
    assert(p87_lon_int_GET(pack) == (int32_t)1822498290);
    assert(p87_afy_GET(pack) == (float) -2.0958761E38F);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)23057);
    assert(p87_vy_GET(pack) == (float) -4.259472E37F);
    assert(p87_vz_GET(pack) == (float)2.8224585E38F);
    assert(p87_yaw_GET(pack) == (float) -1.9163721E38F);
    assert(p87_alt_GET(pack) == (float)5.5447985E37F);
    assert(p87_afx_GET(pack) == (float) -2.1303402E38F);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_x_GET(pack) == (float)5.3473493E37F);
    assert(p89_yaw_GET(pack) == (float)3.575522E37F);
    assert(p89_y_GET(pack) == (float) -1.9642388E38F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)1502930190L);
    assert(p89_pitch_GET(pack) == (float) -2.483864E37F);
    assert(p89_roll_GET(pack) == (float)2.0342634E38F);
    assert(p89_z_GET(pack) == (float) -2.7399651E38F);
};


void c_LoopBackDemoChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_vz_GET(pack) == (int16_t)(int16_t) -6756);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t) -4455);
    assert(p90_yawspeed_GET(pack) == (float)5.69241E36F);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t) -20497);
    assert(p90_rollspeed_GET(pack) == (float)1.4077079E38F);
    assert(p90_time_usec_GET(pack) == (uint64_t)6351120631471618254L);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t) -1071);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t) -12838);
    assert(p90_lat_GET(pack) == (int32_t) -1179385857);
    assert(p90_lon_GET(pack) == (int32_t) -419254813);
    assert(p90_yaw_GET(pack) == (float) -3.2758941E38F);
    assert(p90_pitch_GET(pack) == (float)3.3234233E38F);
    assert(p90_roll_GET(pack) == (float) -1.3157888E38F);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t)602);
    assert(p90_alt_GET(pack) == (int32_t)136007623);
    assert(p90_pitchspeed_GET(pack) == (float)1.9569645E38F);
};


void c_LoopBackDemoChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_aux1_GET(pack) == (float)1.2093928E38F);
    assert(p91_yaw_rudder_GET(pack) == (float) -1.1297777E38F);
    assert(p91_throttle_GET(pack) == (float) -2.8360528E38F);
    assert(p91_roll_ailerons_GET(pack) == (float) -2.7024368E38F);
    assert(p91_aux2_GET(pack) == (float)3.1192938E38F);
    assert(p91_aux4_GET(pack) == (float) -3.1073276E38F);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)232);
    assert(p91_aux3_GET(pack) == (float) -2.9371225E38F);
    assert(p91_time_usec_GET(pack) == (uint64_t)6186905204282790330L);
    assert(p91_pitch_elevator_GET(pack) == (float)2.7158877E38F);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_GUIDED_ARMED);
};


void c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)3697);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)11615);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)38908);
    assert(p92_time_usec_GET(pack) == (uint64_t)2909482447472706510L);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)169);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)54979);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)63516);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)7004);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)36155);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)38341);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)5008);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)22238);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)59752);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)53119);
};


void c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    assert(p93_time_usec_GET(pack) == (uint64_t)1474428559822065775L);
    assert(p93_flags_GET(pack) == (uint64_t)2447111430720415967L);
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_AUTO_ARMED);
    {
        float exemplary[] =  {-4.559752E37F, -4.2790804E37F, 5.0237475E37F, 2.792074E38F, 2.039452E38F, 2.5347511E38F, 8.3849307E37F, -1.1949787E38F, -2.6480702E38F, -7.4492316E37F, -2.2160818E38F, -1.3950712E36F, 1.0835192E38F, 2.307989E38F, 5.011607E37F, 1.6929832E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t)24614);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t) -14870);
    assert(p100_time_usec_GET(pack) == (uint64_t)1113412688679105861L);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p100_flow_comp_m_y_GET(pack) == (float) -5.320796E36F);
    assert(p100_flow_comp_m_x_GET(pack) == (float) -1.6088992E38F);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)40);
    assert(p100_flow_rate_x_TRY(ph) == (float)1.275639E38F);
    assert(p100_flow_rate_y_TRY(ph) == (float)9.293658E37F);
    assert(p100_ground_distance_GET(pack) == (float)2.6935705E38F);
};


void c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_y_GET(pack) == (float) -2.7758275E38F);
    assert(p101_z_GET(pack) == (float) -2.532482E38F);
    assert(p101_x_GET(pack) == (float)1.8389213E38F);
    assert(p101_pitch_GET(pack) == (float) -5.9725627E37F);
    assert(p101_roll_GET(pack) == (float) -2.6079942E38F);
    assert(p101_usec_GET(pack) == (uint64_t)7413588249836671176L);
    assert(p101_yaw_GET(pack) == (float)2.7849545E38F);
};


void c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_z_GET(pack) == (float)1.8050556E38F);
    assert(p102_usec_GET(pack) == (uint64_t)2187933040608949220L);
    assert(p102_y_GET(pack) == (float)1.489833E38F);
    assert(p102_yaw_GET(pack) == (float)2.8376892E38F);
    assert(p102_x_GET(pack) == (float) -2.8131179E38F);
    assert(p102_roll_GET(pack) == (float)1.2498342E38F);
    assert(p102_pitch_GET(pack) == (float) -3.1738886E38F);
};


void c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_z_GET(pack) == (float)1.6438094E37F);
    assert(p103_usec_GET(pack) == (uint64_t)6237931433859141154L);
    assert(p103_y_GET(pack) == (float) -2.4580869E38F);
    assert(p103_x_GET(pack) == (float) -8.97572E37F);
};


void c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_x_GET(pack) == (float)2.1471652E38F);
    assert(p104_z_GET(pack) == (float) -1.8825437E38F);
    assert(p104_roll_GET(pack) == (float)9.129925E37F);
    assert(p104_pitch_GET(pack) == (float) -2.981448E38F);
    assert(p104_usec_GET(pack) == (uint64_t)9157310829916126523L);
    assert(p104_y_GET(pack) == (float) -2.51371E38F);
    assert(p104_yaw_GET(pack) == (float)2.8584724E38F);
};


void c_LoopBackDemoChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_zmag_GET(pack) == (float) -8.770419E37F);
    assert(p105_zacc_GET(pack) == (float)1.2941294E38F);
    assert(p105_ymag_GET(pack) == (float)3.9218417E37F);
    assert(p105_xmag_GET(pack) == (float)3.30223E38F);
    assert(p105_yacc_GET(pack) == (float)2.568159E38F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)65219);
    assert(p105_temperature_GET(pack) == (float) -8.266431E37F);
    assert(p105_xacc_GET(pack) == (float)1.9197362E38F);
    assert(p105_time_usec_GET(pack) == (uint64_t)5847931941142543067L);
    assert(p105_pressure_alt_GET(pack) == (float) -2.57348E38F);
    assert(p105_ygyro_GET(pack) == (float)2.3003891E38F);
    assert(p105_xgyro_GET(pack) == (float)2.494728E38F);
    assert(p105_abs_pressure_GET(pack) == (float) -4.440598E37F);
    assert(p105_zgyro_GET(pack) == (float) -2.8902166E38F);
    assert(p105_diff_pressure_GET(pack) == (float)2.8557158E38F);
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_time_usec_GET(pack) == (uint64_t)6130859101733734007L);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t) -16717);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)3886026346L);
    assert(p106_integrated_y_GET(pack) == (float) -5.182737E37F);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)3932332572L);
    assert(p106_integrated_x_GET(pack) == (float) -2.5634656E38F);
    assert(p106_integrated_zgyro_GET(pack) == (float) -1.0291345E38F);
    assert(p106_distance_GET(pack) == (float) -2.892295E37F);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)157);
    assert(p106_integrated_ygyro_GET(pack) == (float) -1.9078628E38F);
    assert(p106_integrated_xgyro_GET(pack) == (float)2.1110392E37F);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)7);
};


void c_LoopBackDemoChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_diff_pressure_GET(pack) == (float) -3.3856493E38F);
    assert(p107_pressure_alt_GET(pack) == (float)2.425569E38F);
    assert(p107_xacc_GET(pack) == (float)1.9213829E38F);
    assert(p107_xgyro_GET(pack) == (float)2.731438E38F);
    assert(p107_temperature_GET(pack) == (float)4.2096836E37F);
    assert(p107_zacc_GET(pack) == (float) -5.4318336E37F);
    assert(p107_abs_pressure_GET(pack) == (float)3.0610595E37F);
    assert(p107_xmag_GET(pack) == (float) -8.0888724E37F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)1620466569L);
    assert(p107_ymag_GET(pack) == (float)1.779252E37F);
    assert(p107_zmag_GET(pack) == (float)1.536344E38F);
    assert(p107_ygyro_GET(pack) == (float)2.3495407E38F);
    assert(p107_zgyro_GET(pack) == (float) -1.5603505E38F);
    assert(p107_time_usec_GET(pack) == (uint64_t)3601497273475327337L);
    assert(p107_yacc_GET(pack) == (float) -1.1382674E38F);
};


void c_LoopBackDemoChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_q4_GET(pack) == (float)2.1514824E38F);
    assert(p108_q1_GET(pack) == (float) -1.3125554E38F);
    assert(p108_pitch_GET(pack) == (float)2.8383871E38F);
    assert(p108_zacc_GET(pack) == (float)1.774281E38F);
    assert(p108_roll_GET(pack) == (float)1.6393231E38F);
    assert(p108_xacc_GET(pack) == (float) -2.4084877E38F);
    assert(p108_yacc_GET(pack) == (float)7.763163E37F);
    assert(p108_q2_GET(pack) == (float)2.6786946E38F);
    assert(p108_ygyro_GET(pack) == (float)8.4674943E37F);
    assert(p108_ve_GET(pack) == (float)2.9338777E38F);
    assert(p108_alt_GET(pack) == (float)1.5932562E38F);
    assert(p108_lat_GET(pack) == (float) -2.1080216E37F);
    assert(p108_vn_GET(pack) == (float) -1.9720254E38F);
    assert(p108_lon_GET(pack) == (float) -2.4611098E38F);
    assert(p108_q3_GET(pack) == (float)2.9039204E38F);
    assert(p108_std_dev_vert_GET(pack) == (float)1.0083831E38F);
    assert(p108_zgyro_GET(pack) == (float)3.1192496E38F);
    assert(p108_xgyro_GET(pack) == (float)2.8250987E38F);
    assert(p108_std_dev_horz_GET(pack) == (float)1.1476512E38F);
    assert(p108_yaw_GET(pack) == (float) -8.3425323E37F);
    assert(p108_vd_GET(pack) == (float)2.0174749E38F);
};


void c_LoopBackDemoChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)157);
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)70);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)46103);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)147);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)20);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)7072);
};


void c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)237);
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)182);
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)92);
    {
        uint8_t exemplary[] =  {(uint8_t)95, (uint8_t)52, (uint8_t)55, (uint8_t)0, (uint8_t)96, (uint8_t)132, (uint8_t)51, (uint8_t)145, (uint8_t)169, (uint8_t)150, (uint8_t)109, (uint8_t)212, (uint8_t)157, (uint8_t)85, (uint8_t)171, (uint8_t)98, (uint8_t)94, (uint8_t)98, (uint8_t)21, (uint8_t)16, (uint8_t)114, (uint8_t)135, (uint8_t)11, (uint8_t)34, (uint8_t)193, (uint8_t)210, (uint8_t)107, (uint8_t)198, (uint8_t)40, (uint8_t)100, (uint8_t)89, (uint8_t)198, (uint8_t)172, (uint8_t)60, (uint8_t)147, (uint8_t)96, (uint8_t)159, (uint8_t)178, (uint8_t)164, (uint8_t)211, (uint8_t)189, (uint8_t)234, (uint8_t)72, (uint8_t)6, (uint8_t)223, (uint8_t)162, (uint8_t)57, (uint8_t)129, (uint8_t)53, (uint8_t)120, (uint8_t)216, (uint8_t)103, (uint8_t)109, (uint8_t)69, (uint8_t)185, (uint8_t)85, (uint8_t)178, (uint8_t)5, (uint8_t)161, (uint8_t)67, (uint8_t)160, (uint8_t)103, (uint8_t)227, (uint8_t)189, (uint8_t)86, (uint8_t)69, (uint8_t)6, (uint8_t)187, (uint8_t)95, (uint8_t)74, (uint8_t)157, (uint8_t)172, (uint8_t)181, (uint8_t)98, (uint8_t)209, (uint8_t)131, (uint8_t)172, (uint8_t)168, (uint8_t)65, (uint8_t)216, (uint8_t)236, (uint8_t)142, (uint8_t)170, (uint8_t)133, (uint8_t)97, (uint8_t)193, (uint8_t)132, (uint8_t)49, (uint8_t)198, (uint8_t)239, (uint8_t)121, (uint8_t)67, (uint8_t)142, (uint8_t)122, (uint8_t)11, (uint8_t)74, (uint8_t)107, (uint8_t)229, (uint8_t)150, (uint8_t)129, (uint8_t)176, (uint8_t)99, (uint8_t)115, (uint8_t)60, (uint8_t)170, (uint8_t)122, (uint8_t)7, (uint8_t)149, (uint8_t)49, (uint8_t)30, (uint8_t)123, (uint8_t)139, (uint8_t)188, (uint8_t)67, (uint8_t)51, (uint8_t)236, (uint8_t)114, (uint8_t)230, (uint8_t)235, (uint8_t)9, (uint8_t)79, (uint8_t)116, (uint8_t)119, (uint8_t)90, (uint8_t)53, (uint8_t)87, (uint8_t)153, (uint8_t)102, (uint8_t)30, (uint8_t)104, (uint8_t)50, (uint8_t)145, (uint8_t)221, (uint8_t)129, (uint8_t)65, (uint8_t)219, (uint8_t)218, (uint8_t)152, (uint8_t)158, (uint8_t)228, (uint8_t)240, (uint8_t)135, (uint8_t)28, (uint8_t)150, (uint8_t)96, (uint8_t)184, (uint8_t)203, (uint8_t)184, (uint8_t)242, (uint8_t)161, (uint8_t)9, (uint8_t)255, (uint8_t)98, (uint8_t)193, (uint8_t)50, (uint8_t)164, (uint8_t)61, (uint8_t)222, (uint8_t)245, (uint8_t)156, (uint8_t)185, (uint8_t)21, (uint8_t)184, (uint8_t)13, (uint8_t)193, (uint8_t)63, (uint8_t)116, (uint8_t)19, (uint8_t)240, (uint8_t)171, (uint8_t)76, (uint8_t)137, (uint8_t)227, (uint8_t)97, (uint8_t)23, (uint8_t)27, (uint8_t)213, (uint8_t)71, (uint8_t)0, (uint8_t)162, (uint8_t)146, (uint8_t)193, (uint8_t)238, (uint8_t)160, (uint8_t)55, (uint8_t)157, (uint8_t)250, (uint8_t)57, (uint8_t)167, (uint8_t)10, (uint8_t)201, (uint8_t)182, (uint8_t)109, (uint8_t)41, (uint8_t)118, (uint8_t)24, (uint8_t)174, (uint8_t)160, (uint8_t)116, (uint8_t)110, (uint8_t)152, (uint8_t)174, (uint8_t)229, (uint8_t)205, (uint8_t)117, (uint8_t)140, (uint8_t)143, (uint8_t)130, (uint8_t)171, (uint8_t)88, (uint8_t)112, (uint8_t)56, (uint8_t)2, (uint8_t)79, (uint8_t)131, (uint8_t)112, (uint8_t)83, (uint8_t)105, (uint8_t)86, (uint8_t)245, (uint8_t)91, (uint8_t)12, (uint8_t)46, (uint8_t)80, (uint8_t)110, (uint8_t)139, (uint8_t)25, (uint8_t)33, (uint8_t)12, (uint8_t)215, (uint8_t)168, (uint8_t)89, (uint8_t)126, (uint8_t)30, (uint8_t)148, (uint8_t)29, (uint8_t)16, (uint8_t)122, (uint8_t)61, (uint8_t)131, (uint8_t)46, (uint8_t)141, (uint8_t)116, (uint8_t)169, (uint8_t)13, (uint8_t)211, (uint8_t)233, (uint8_t)126, (uint8_t)175, (uint8_t)29, (uint8_t)187} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_ts1_GET(pack) == (int64_t) -1256720058239829039L);
    assert(p111_tc1_GET(pack) == (int64_t)7001358374000970968L);
};


void c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_time_usec_GET(pack) == (uint64_t)6406682260498060426L);
    assert(p112_seq_GET(pack) == (uint32_t)2003097899L);
};


void c_LoopBackDemoChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_lat_GET(pack) == (int32_t)1505859352);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)159);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)48358);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)9959);
    assert(p113_time_usec_GET(pack) == (uint64_t)584389270026088170L);
    assert(p113_alt_GET(pack) == (int32_t)1541183800);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t) -31151);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t)19323);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)223);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t) -20472);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)41276);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)28210);
    assert(p113_lon_GET(pack) == (int32_t) -1277557796);
};


void c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_integrated_x_GET(pack) == (float)2.5084702E38F);
    assert(p114_integrated_y_GET(pack) == (float)2.7481069E38F);
    assert(p114_distance_GET(pack) == (float) -2.9111874E38F);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)709654250L);
    assert(p114_integrated_ygyro_GET(pack) == (float) -2.4196894E38F);
    assert(p114_time_usec_GET(pack) == (uint64_t)4906432917925905023L);
    assert(p114_integrated_xgyro_GET(pack) == (float) -2.0259229E38F);
    assert(p114_integrated_zgyro_GET(pack) == (float)1.359779E38F);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t)10378);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)76);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)2150439752L);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)89);
};


void c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t) -25511);
    assert(p115_pitchspeed_GET(pack) == (float) -3.1169291E38F);
    assert(p115_time_usec_GET(pack) == (uint64_t)7747292613500140737L);
    {
        float exemplary[] =  {-3.3104975E38F, -1.7653187E38F, -1.0558286E38F, 2.9584223E38F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t) -27479);
    assert(p115_alt_GET(pack) == (int32_t) -1794345036);
    assert(p115_lon_GET(pack) == (int32_t) -59761761);
    assert(p115_lat_GET(pack) == (int32_t) -1534223920);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t) -9988);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t)16534);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)54508);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t) -3225);
    assert(p115_yawspeed_GET(pack) == (float) -2.8317325E38F);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)38711);
    assert(p115_rollspeed_GET(pack) == (float)1.5065548E38F);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t) -26566);
};


void c_LoopBackDemoChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t)15253);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t) -344);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t)7890);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t) -11034);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t)17994);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t)29247);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t)14913);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)2376143842L);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t) -29470);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t)6605);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)23373);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)245);
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)13430);
};


void c_LoopBackDemoChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)41625);
    assert(p118_size_GET(pack) == (uint32_t)3575712693L);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)22779);
    assert(p118_time_utc_GET(pack) == (uint32_t)2131655307L);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)51263);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)32334);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)129);
    assert(p119_ofs_GET(pack) == (uint32_t)1273570673L);
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p119_count_GET(pack) == (uint32_t)3004841004L);
};


void c_LoopBackDemoChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)169, (uint8_t)238, (uint8_t)99, (uint8_t)135, (uint8_t)35, (uint8_t)5, (uint8_t)49, (uint8_t)180, (uint8_t)173, (uint8_t)248, (uint8_t)108, (uint8_t)190, (uint8_t)62, (uint8_t)141, (uint8_t)172, (uint8_t)36, (uint8_t)12, (uint8_t)107, (uint8_t)51, (uint8_t)76, (uint8_t)17, (uint8_t)68, (uint8_t)144, (uint8_t)157, (uint8_t)104, (uint8_t)226, (uint8_t)104, (uint8_t)187, (uint8_t)225, (uint8_t)98, (uint8_t)9, (uint8_t)203, (uint8_t)90, (uint8_t)163, (uint8_t)9, (uint8_t)163, (uint8_t)178, (uint8_t)153, (uint8_t)219, (uint8_t)81, (uint8_t)249, (uint8_t)223, (uint8_t)68, (uint8_t)231, (uint8_t)71, (uint8_t)222, (uint8_t)15, (uint8_t)236, (uint8_t)56, (uint8_t)130, (uint8_t)226, (uint8_t)235, (uint8_t)136, (uint8_t)224, (uint8_t)204, (uint8_t)118, (uint8_t)134, (uint8_t)88, (uint8_t)233, (uint8_t)254, (uint8_t)112, (uint8_t)156, (uint8_t)66, (uint8_t)255, (uint8_t)38, (uint8_t)77, (uint8_t)100, (uint8_t)152, (uint8_t)22, (uint8_t)107, (uint8_t)40, (uint8_t)93, (uint8_t)115, (uint8_t)85, (uint8_t)14, (uint8_t)100, (uint8_t)10, (uint8_t)180, (uint8_t)39, (uint8_t)211, (uint8_t)93, (uint8_t)202, (uint8_t)244, (uint8_t)74, (uint8_t)14, (uint8_t)147, (uint8_t)105, (uint8_t)93, (uint8_t)17, (uint8_t)97} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)5464);
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)211);
    assert(p120_ofs_GET(pack) == (uint32_t)30661205L);
};


void c_LoopBackDemoChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)213);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)5);
};


void c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)146);
    {
        uint8_t exemplary[] =  {(uint8_t)94, (uint8_t)148, (uint8_t)166, (uint8_t)170, (uint8_t)33, (uint8_t)84, (uint8_t)104, (uint8_t)171, (uint8_t)185, (uint8_t)191, (uint8_t)93, (uint8_t)1, (uint8_t)172, (uint8_t)197, (uint8_t)186, (uint8_t)95, (uint8_t)227, (uint8_t)88, (uint8_t)76, (uint8_t)95, (uint8_t)86, (uint8_t)85, (uint8_t)237, (uint8_t)193, (uint8_t)191, (uint8_t)67, (uint8_t)208, (uint8_t)225, (uint8_t)132, (uint8_t)27, (uint8_t)161, (uint8_t)107, (uint8_t)210, (uint8_t)120, (uint8_t)239, (uint8_t)247, (uint8_t)50, (uint8_t)255, (uint8_t)201, (uint8_t)111, (uint8_t)134, (uint8_t)187, (uint8_t)11, (uint8_t)212, (uint8_t)158, (uint8_t)27, (uint8_t)22, (uint8_t)4, (uint8_t)105, (uint8_t)154, (uint8_t)56, (uint8_t)12, (uint8_t)241, (uint8_t)255, (uint8_t)183, (uint8_t)21, (uint8_t)166, (uint8_t)102, (uint8_t)247, (uint8_t)10, (uint8_t)240, (uint8_t)28, (uint8_t)45, (uint8_t)224, (uint8_t)36, (uint8_t)121, (uint8_t)72, (uint8_t)89, (uint8_t)183, (uint8_t)165, (uint8_t)116, (uint8_t)189, (uint8_t)34, (uint8_t)238, (uint8_t)118, (uint8_t)149, (uint8_t)182, (uint8_t)6, (uint8_t)22, (uint8_t)169, (uint8_t)235, (uint8_t)66, (uint8_t)216, (uint8_t)182, (uint8_t)75, (uint8_t)59, (uint8_t)195, (uint8_t)75, (uint8_t)162, (uint8_t)6, (uint8_t)139, (uint8_t)54, (uint8_t)7, (uint8_t)95, (uint8_t)64, (uint8_t)137, (uint8_t)137, (uint8_t)113, (uint8_t)87, (uint8_t)147, (uint8_t)159, (uint8_t)51, (uint8_t)91, (uint8_t)59, (uint8_t)202, (uint8_t)154, (uint8_t)190, (uint8_t)252, (uint8_t)144, (uint8_t)42} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)8);
};


void c_LoopBackDemoChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)42727);
    assert(p124_dgps_age_GET(pack) == (uint32_t)3817662718L);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)90);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p124_time_usec_GET(pack) == (uint64_t)2113104133381955284L);
    assert(p124_alt_GET(pack) == (int32_t)738404057);
    assert(p124_lat_GET(pack) == (int32_t) -12403877);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS);
    assert(p124_lon_GET(pack) == (int32_t) -1607122032);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)32637);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)5505);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)10302);
};


void c_LoopBackDemoChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)773);
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)28608);
    assert(p125_flags_GET(pack) == e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT);
};


void c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    assert(p126_flags_GET(pack) == e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY);
    assert(p126_baudrate_GET(pack) == (uint32_t)2790515056L);
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL);
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)40730);
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)215);
    {
        uint8_t exemplary[] =  {(uint8_t)3, (uint8_t)251, (uint8_t)184, (uint8_t)48, (uint8_t)43, (uint8_t)245, (uint8_t)162, (uint8_t)137, (uint8_t)206, (uint8_t)81, (uint8_t)216, (uint8_t)234, (uint8_t)0, (uint8_t)213, (uint8_t)240, (uint8_t)131, (uint8_t)12, (uint8_t)91, (uint8_t)199, (uint8_t)108, (uint8_t)130, (uint8_t)81, (uint8_t)178, (uint8_t)115, (uint8_t)94, (uint8_t)211, (uint8_t)84, (uint8_t)224, (uint8_t)183, (uint8_t)195, (uint8_t)126, (uint8_t)115, (uint8_t)239, (uint8_t)138, (uint8_t)103, (uint8_t)106, (uint8_t)152, (uint8_t)57, (uint8_t)69, (uint8_t)157, (uint8_t)66, (uint8_t)171, (uint8_t)94, (uint8_t)240, (uint8_t)180, (uint8_t)17, (uint8_t)143, (uint8_t)78, (uint8_t)28, (uint8_t)113, (uint8_t)187, (uint8_t)206, (uint8_t)247, (uint8_t)78, (uint8_t)122, (uint8_t)97, (uint8_t)161, (uint8_t)169, (uint8_t)189, (uint8_t)196, (uint8_t)196, (uint8_t)13, (uint8_t)119, (uint8_t)141, (uint8_t)126, (uint8_t)124, (uint8_t)131, (uint8_t)81, (uint8_t)245, (uint8_t)81} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)59);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t) -1773376485);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)15974);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p127_tow_GET(pack) == (uint32_t)4141091657L);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t) -288526269);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)106);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)248);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t)1089269669);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)165);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t) -795670701);
    assert(p127_accuracy_GET(pack) == (uint32_t)2438175303L);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)1357272484L);
};


void c_LoopBackDemoChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_tow_GET(pack) == (uint32_t)363261800L);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t) -686020420);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)4119232381L);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t)987371513);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)58);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)184);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)16014);
    assert(p128_accuracy_GET(pack) == (uint32_t)2349941569L);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t) -1214642229);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t)48509868);
};


void c_LoopBackDemoChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t)22583);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t) -16546);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t)779);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t) -1965);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)4248968409L);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t) -5751);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t)20491);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t) -28098);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t)29671);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t)5143);
};


void c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)34376);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p130_size_GET(pack) == (uint32_t)1087905603L);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)201);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)49679);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)21395);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)188);
};


void c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)151, (uint8_t)21, (uint8_t)212, (uint8_t)59, (uint8_t)72, (uint8_t)34, (uint8_t)160, (uint8_t)29, (uint8_t)117, (uint8_t)213, (uint8_t)6, (uint8_t)247, (uint8_t)97, (uint8_t)222, (uint8_t)40, (uint8_t)176, (uint8_t)194, (uint8_t)187, (uint8_t)149, (uint8_t)230, (uint8_t)215, (uint8_t)246, (uint8_t)9, (uint8_t)41, (uint8_t)253, (uint8_t)142, (uint8_t)158, (uint8_t)117, (uint8_t)155, (uint8_t)158, (uint8_t)211, (uint8_t)51, (uint8_t)27, (uint8_t)112, (uint8_t)62, (uint8_t)117, (uint8_t)208, (uint8_t)228, (uint8_t)72, (uint8_t)37, (uint8_t)233, (uint8_t)159, (uint8_t)36, (uint8_t)6, (uint8_t)192, (uint8_t)117, (uint8_t)154, (uint8_t)57, (uint8_t)214, (uint8_t)28, (uint8_t)24, (uint8_t)28, (uint8_t)107, (uint8_t)60, (uint8_t)10, (uint8_t)170, (uint8_t)131, (uint8_t)99, (uint8_t)14, (uint8_t)168, (uint8_t)179, (uint8_t)117, (uint8_t)23, (uint8_t)140, (uint8_t)28, (uint8_t)25, (uint8_t)255, (uint8_t)6, (uint8_t)7, (uint8_t)211, (uint8_t)103, (uint8_t)38, (uint8_t)189, (uint8_t)250, (uint8_t)36, (uint8_t)150, (uint8_t)211, (uint8_t)91, (uint8_t)0, (uint8_t)116, (uint8_t)109, (uint8_t)204, (uint8_t)97, (uint8_t)100, (uint8_t)151, (uint8_t)107, (uint8_t)18, (uint8_t)166, (uint8_t)18, (uint8_t)68, (uint8_t)119, (uint8_t)235, (uint8_t)7, (uint8_t)139, (uint8_t)235, (uint8_t)77, (uint8_t)34, (uint8_t)17, (uint8_t)210, (uint8_t)190, (uint8_t)43, (uint8_t)55, (uint8_t)123, (uint8_t)213, (uint8_t)96, (uint8_t)213, (uint8_t)14, (uint8_t)136, (uint8_t)255, (uint8_t)198, (uint8_t)61, (uint8_t)6, (uint8_t)117, (uint8_t)20, (uint8_t)27, (uint8_t)128, (uint8_t)229, (uint8_t)148, (uint8_t)122, (uint8_t)113, (uint8_t)228, (uint8_t)110, (uint8_t)16, (uint8_t)86, (uint8_t)147, (uint8_t)164, (uint8_t)60, (uint8_t)58, (uint8_t)215, (uint8_t)76, (uint8_t)117, (uint8_t)78, (uint8_t)138, (uint8_t)175, (uint8_t)50, (uint8_t)244, (uint8_t)223, (uint8_t)123, (uint8_t)36, (uint8_t)61, (uint8_t)45, (uint8_t)240, (uint8_t)4, (uint8_t)22, (uint8_t)232, (uint8_t)199, (uint8_t)137, (uint8_t)78, (uint8_t)104, (uint8_t)136, (uint8_t)215, (uint8_t)35, (uint8_t)101, (uint8_t)96, (uint8_t)114, (uint8_t)82, (uint8_t)35, (uint8_t)223, (uint8_t)15, (uint8_t)66, (uint8_t)117, (uint8_t)29, (uint8_t)177, (uint8_t)181, (uint8_t)76, (uint8_t)187, (uint8_t)75, (uint8_t)195, (uint8_t)34, (uint8_t)232, (uint8_t)185, (uint8_t)162, (uint8_t)173, (uint8_t)150, (uint8_t)244, (uint8_t)207, (uint8_t)136, (uint8_t)233, (uint8_t)211, (uint8_t)66, (uint8_t)133, (uint8_t)153, (uint8_t)95, (uint8_t)167, (uint8_t)167, (uint8_t)92, (uint8_t)89, (uint8_t)187, (uint8_t)213, (uint8_t)137, (uint8_t)0, (uint8_t)131, (uint8_t)108, (uint8_t)161, (uint8_t)112, (uint8_t)113, (uint8_t)236, (uint8_t)148, (uint8_t)9, (uint8_t)81, (uint8_t)56, (uint8_t)175, (uint8_t)249, (uint8_t)203, (uint8_t)157, (uint8_t)76, (uint8_t)47, (uint8_t)34, (uint8_t)153, (uint8_t)46, (uint8_t)100, (uint8_t)7, (uint8_t)155, (uint8_t)45, (uint8_t)146, (uint8_t)41, (uint8_t)66, (uint8_t)213, (uint8_t)144, (uint8_t)207, (uint8_t)91, (uint8_t)20, (uint8_t)188, (uint8_t)79, (uint8_t)224, (uint8_t)87, (uint8_t)220, (uint8_t)174, (uint8_t)50, (uint8_t)91, (uint8_t)117, (uint8_t)141, (uint8_t)53, (uint8_t)152, (uint8_t)164, (uint8_t)22, (uint8_t)201, (uint8_t)246, (uint8_t)140, (uint8_t)121, (uint8_t)143, (uint8_t)140, (uint8_t)157, (uint8_t)54, (uint8_t)156, (uint8_t)197, (uint8_t)106, (uint8_t)3, (uint8_t)67, (uint8_t)79, (uint8_t)249, (uint8_t)130, (uint8_t)248} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)30171);
};


void c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)12190);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)63084);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)124);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)1719722399L);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)225);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)18773);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_PITCH_270);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER);
};


void c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_mask_GET(pack) == (uint64_t)5045198130015881974L);
    assert(p133_lon_GET(pack) == (int32_t) -546150550);
    assert(p133_lat_GET(pack) == (int32_t)1909875114);
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)62793);
};


void c_LoopBackDemoChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)14);
    assert(p134_lon_GET(pack) == (int32_t)81012863);
    assert(p134_lat_GET(pack) == (int32_t)881949668);
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)29357);
    {
        int16_t exemplary[] =  {(int16_t)30506, (int16_t) -196, (int16_t) -7299, (int16_t)19898, (int16_t) -19640, (int16_t) -27318, (int16_t)22540, (int16_t) -32200, (int16_t) -2762, (int16_t) -14698, (int16_t) -5005, (int16_t)11620, (int16_t) -20376, (int16_t) -20742, (int16_t) -7570, (int16_t) -7619} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lat_GET(pack) == (int32_t)1649657393);
    assert(p135_lon_GET(pack) == (int32_t) -907548386);
};


void c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_terrain_height_GET(pack) == (float)1.1474786E38F);
    assert(p136_lat_GET(pack) == (int32_t) -2106605408);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)57924);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)62712);
    assert(p136_lon_GET(pack) == (int32_t) -419599966);
    assert(p136_current_height_GET(pack) == (float)3.0763292E38F);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)45755);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_press_abs_GET(pack) == (float)1.802346E38F);
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)1943872072L);
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t) -25967);
    assert(p137_press_diff_GET(pack) == (float) -1.394602E38F);
};


void c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    assert(p138_x_GET(pack) == (float) -1.3502241E38F);
    assert(p138_z_GET(pack) == (float) -4.6914944E36F);
    assert(p138_y_GET(pack) == (float) -7.4446544E37F);
    assert(p138_time_usec_GET(pack) == (uint64_t)3084115421394166686L);
    {
        float exemplary[] =  {-1.2674449E38F, 3.1475624E38F, 4.842804E37F, -3.5136836E37F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)186);
    assert(p139_time_usec_GET(pack) == (uint64_t)4355607148381091790L);
    {
        float exemplary[] =  {-2.923095E38F, -1.2090126E37F, -6.3982905E37F, 1.4410708E37F, 1.4688799E38F, -2.4167106E38F, -4.400476E37F, 2.8209485E38F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)86);
};


void c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    assert(p140_time_usec_GET(pack) == (uint64_t)1282913368201206871L);
    {
        float exemplary[] =  {-1.4799952E38F, 5.670321E37F, 2.810417E38F, 3.1342777E38F, 1.5096031E38F, 1.7014222E38F, 2.2655076E38F, -2.7770111E38F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)121);
};


void c_LoopBackDemoChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_bottom_clearance_GET(pack) == (float) -1.8705608E38F);
    assert(p141_altitude_amsl_GET(pack) == (float)1.3755245E38F);
    assert(p141_altitude_monotonic_GET(pack) == (float)3.081234E38F);
    assert(p141_altitude_terrain_GET(pack) == (float) -1.1050284E38F);
    assert(p141_altitude_local_GET(pack) == (float)2.209842E38F);
    assert(p141_altitude_relative_GET(pack) == (float) -2.506123E37F);
    assert(p141_time_usec_GET(pack) == (uint64_t)7151248871248795174L);
};


void c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)109, (uint8_t)78, (uint8_t)193, (uint8_t)93, (uint8_t)158, (uint8_t)200, (uint8_t)135, (uint8_t)84, (uint8_t)187, (uint8_t)166, (uint8_t)136, (uint8_t)163, (uint8_t)114, (uint8_t)77, (uint8_t)36, (uint8_t)184, (uint8_t)114, (uint8_t)52, (uint8_t)149, (uint8_t)153, (uint8_t)143, (uint8_t)27, (uint8_t)227, (uint8_t)0, (uint8_t)237, (uint8_t)116, (uint8_t)29, (uint8_t)108, (uint8_t)99, (uint8_t)236, (uint8_t)193, (uint8_t)50, (uint8_t)92, (uint8_t)200, (uint8_t)159, (uint8_t)240, (uint8_t)135, (uint8_t)218, (uint8_t)224, (uint8_t)154, (uint8_t)193, (uint8_t)134, (uint8_t)73, (uint8_t)84, (uint8_t)173, (uint8_t)63, (uint8_t)83, (uint8_t)254, (uint8_t)226, (uint8_t)197, (uint8_t)225, (uint8_t)235, (uint8_t)226, (uint8_t)210, (uint8_t)53, (uint8_t)249, (uint8_t)60, (uint8_t)231, (uint8_t)132, (uint8_t)181, (uint8_t)129, (uint8_t)128, (uint8_t)139, (uint8_t)36, (uint8_t)91, (uint8_t)218, (uint8_t)208, (uint8_t)63, (uint8_t)125, (uint8_t)177, (uint8_t)201, (uint8_t)169, (uint8_t)170, (uint8_t)236, (uint8_t)62, (uint8_t)124, (uint8_t)188, (uint8_t)176, (uint8_t)50, (uint8_t)250, (uint8_t)205, (uint8_t)223, (uint8_t)0, (uint8_t)178, (uint8_t)145, (uint8_t)32, (uint8_t)91, (uint8_t)230, (uint8_t)56, (uint8_t)67, (uint8_t)92, (uint8_t)245, (uint8_t)186, (uint8_t)144, (uint8_t)218, (uint8_t)55, (uint8_t)27, (uint8_t)200, (uint8_t)162, (uint8_t)206, (uint8_t)219, (uint8_t)133, (uint8_t)218, (uint8_t)194, (uint8_t)104, (uint8_t)66, (uint8_t)233, (uint8_t)176, (uint8_t)15, (uint8_t)28, (uint8_t)90, (uint8_t)77, (uint8_t)217, (uint8_t)96, (uint8_t)242, (uint8_t)40, (uint8_t)185, (uint8_t)55, (uint8_t)114, (uint8_t)96} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)18);
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)194);
    {
        uint8_t exemplary[] =  {(uint8_t)252, (uint8_t)151, (uint8_t)170, (uint8_t)190, (uint8_t)143, (uint8_t)24, (uint8_t)133, (uint8_t)221, (uint8_t)131, (uint8_t)38, (uint8_t)218, (uint8_t)208, (uint8_t)98, (uint8_t)123, (uint8_t)193, (uint8_t)201, (uint8_t)153, (uint8_t)3, (uint8_t)220, (uint8_t)82, (uint8_t)47, (uint8_t)115, (uint8_t)59, (uint8_t)31, (uint8_t)185, (uint8_t)97, (uint8_t)68, (uint8_t)25, (uint8_t)81, (uint8_t)124, (uint8_t)3, (uint8_t)195, (uint8_t)11, (uint8_t)148, (uint8_t)60, (uint8_t)228, (uint8_t)133, (uint8_t)5, (uint8_t)119, (uint8_t)165, (uint8_t)64, (uint8_t)124, (uint8_t)163, (uint8_t)47, (uint8_t)50, (uint8_t)243, (uint8_t)125, (uint8_t)0, (uint8_t)174, (uint8_t)213, (uint8_t)119, (uint8_t)99, (uint8_t)181, (uint8_t)37, (uint8_t)30, (uint8_t)111, (uint8_t)29, (uint8_t)190, (uint8_t)8, (uint8_t)186, (uint8_t)217, (uint8_t)207, (uint8_t)49, (uint8_t)76, (uint8_t)66, (uint8_t)249, (uint8_t)127, (uint8_t)105, (uint8_t)94, (uint8_t)139, (uint8_t)85, (uint8_t)146, (uint8_t)41, (uint8_t)67, (uint8_t)46, (uint8_t)74, (uint8_t)199, (uint8_t)41, (uint8_t)44, (uint8_t)217, (uint8_t)15, (uint8_t)98, (uint8_t)144, (uint8_t)188, (uint8_t)207, (uint8_t)178, (uint8_t)144, (uint8_t)242, (uint8_t)44, (uint8_t)96, (uint8_t)223, (uint8_t)46, (uint8_t)87, (uint8_t)118, (uint8_t)210, (uint8_t)216, (uint8_t)131, (uint8_t)198, (uint8_t)251, (uint8_t)109, (uint8_t)146, (uint8_t)242, (uint8_t)208, (uint8_t)173, (uint8_t)85, (uint8_t)61, (uint8_t)135, (uint8_t)120, (uint8_t)243, (uint8_t)155, (uint8_t)31, (uint8_t)111, (uint8_t)158, (uint8_t)52, (uint8_t)200, (uint8_t)198, (uint8_t)226, (uint8_t)135, (uint8_t)95, (uint8_t)6} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)196);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)301252434L);
    assert(p143_press_diff_GET(pack) == (float) -6.446512E37F);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t) -1936);
    assert(p143_press_abs_GET(pack) == (float)3.3359274E38F);
};


void c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {3.0747788E38F, -3.282472E37F, 3.2775796E38F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_timestamp_GET(pack) == (uint64_t)4508543658451459500L);
    {
        float exemplary[] =  {2.0837202E38F, -8.564641E37F, -2.751647E38F, 3.2136598E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_alt_GET(pack) == (float)1.2751748E38F);
    {
        float exemplary[] =  {-7.676915E37F, -1.933088E38F, 3.3357357E38F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lon_GET(pack) == (int32_t) -1431688308);
    {
        float exemplary[] =  {-2.0564846E38F, -2.0070848E38F, 9.417945E37F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p144_custom_state_GET(pack) == (uint64_t)6281143862288843862L);
    {
        float exemplary[] =  {-2.0805243E38F, 3.8223933E37F, 1.0781675E38F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lat_GET(pack) == (int32_t) -2119254407);
};


void c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_x_vel_GET(pack) == (float)1.3500513E38F);
    {
        float exemplary[] =  {1.9622343E38F, 2.919766E38F, 3.758412E37F, 1.6440576E38F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-7.851598E37F, 2.3192197E38F, 7.864304E37F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_y_acc_GET(pack) == (float) -3.2815207E38F);
    assert(p146_z_pos_GET(pack) == (float)1.9250503E38F);
    assert(p146_y_pos_GET(pack) == (float) -1.1970083E38F);
    assert(p146_yaw_rate_GET(pack) == (float)5.716821E37F);
    assert(p146_y_vel_GET(pack) == (float) -2.3755977E38F);
    assert(p146_roll_rate_GET(pack) == (float)1.5119659E38F);
    assert(p146_z_acc_GET(pack) == (float) -1.8951675E38F);
    assert(p146_airspeed_GET(pack) == (float) -1.6229394E38F);
    assert(p146_x_pos_GET(pack) == (float)2.7127883E38F);
    assert(p146_pitch_rate_GET(pack) == (float)3.8926122E37F);
    assert(p146_time_usec_GET(pack) == (uint64_t)5210835352239418770L);
    {
        float exemplary[] =  {2.7453927E38F, -7.922378E37F, -8.719615E37F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_x_acc_GET(pack) == (float)2.6040015E37F);
    assert(p146_z_vel_GET(pack) == (float)8.2071746E37F);
};


void c_LoopBackDemoChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_AVIONICS);
    assert(p147_energy_consumed_GET(pack) == (int32_t) -2123353129);
    {
        uint16_t exemplary[] =  {(uint16_t)43808, (uint16_t)40100, (uint16_t)7192, (uint16_t)1603, (uint16_t)50056, (uint16_t)60856, (uint16_t)10405, (uint16_t)1002, (uint16_t)34679, (uint16_t)9404} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t)14291);
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t) -4501);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIPO);
    assert(p147_current_consumed_GET(pack) == (int32_t)167114971);
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)181);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t) -106);
};


void c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_board_version_GET(pack) == (uint32_t)173704400L);
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)4058879657L);
    assert(p148_os_sw_version_GET(pack) == (uint32_t)1644113992L);
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)39551);
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)23748);
    {
        uint8_t exemplary[] =  {(uint8_t)206, (uint8_t)49, (uint8_t)130, (uint8_t)148, (uint8_t)190, (uint8_t)216, (uint8_t)235, (uint8_t)236} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)215, (uint8_t)241, (uint8_t)147, (uint8_t)166, (uint8_t)240, (uint8_t)191, (uint8_t)84, (uint8_t)48} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)1747828716L);
    assert(p148_capabilities_GET(pack) == e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET);
    assert(p148_uid_GET(pack) == (uint64_t)2384860938964567975L);
    {
        uint8_t exemplary[] =  {(uint8_t)195, (uint8_t)251, (uint8_t)251, (uint8_t)147, (uint8_t)100, (uint8_t)86, (uint8_t)228, (uint8_t)159, (uint8_t)218, (uint8_t)227, (uint8_t)221, (uint8_t)136, (uint8_t)154, (uint8_t)127, (uint8_t)190, (uint8_t)196, (uint8_t)243, (uint8_t)10} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)61, (uint8_t)228, (uint8_t)115, (uint8_t)242, (uint8_t)63, (uint8_t)203, (uint8_t)161, (uint8_t)213} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)200);
    assert(p149_time_usec_GET(pack) == (uint64_t)2646807189233344234L);
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)153);
    assert(p149_z_TRY(ph) == (float) -2.0506976E38F);
    assert(p149_size_y_GET(pack) == (float) -3.3474183E37F);
    assert(p149_angle_x_GET(pack) == (float) -4.836181E37F);
    {
        float exemplary[] =  {1.7037305E38F, 3.8433452E37F, -2.6192434E38F, -1.3906081E38F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_distance_GET(pack) == (float)1.325009E38F);
    assert(p149_size_x_GET(pack) == (float) -2.977634E38F);
    assert(p149_y_TRY(ph) == (float)1.3351318E38F);
    assert(p149_angle_y_GET(pack) == (float) -1.860374E38F);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL);
    assert(p149_x_TRY(ph) == (float) -5.111856E37F);
};


void c_LoopBackDemoChannel_on_SENS_POWER_201(Bounds_Inside * ph, Pack * pack)
{
    assert(p201_adc121_cs1_amp_GET(pack) == (float)1.1484542E38F);
    assert(p201_adc121_cspb_amp_GET(pack) == (float) -2.472064E38F);
    assert(p201_adc121_cs2_amp_GET(pack) == (float)1.2709903E38F);
    assert(p201_adc121_vspb_volt_GET(pack) == (float) -6.6312517E37F);
};


void c_LoopBackDemoChannel_on_SENS_MPPT_202(Bounds_Inside * ph, Pack * pack)
{
    assert(p202_mppt_timestamp_GET(pack) == (uint64_t)7709173357184220256L);
    assert(p202_mppt2_pwm_GET(pack) == (uint16_t)(uint16_t)31714);
    assert(p202_mppt3_volt_GET(pack) == (float) -2.8348923E38F);
    assert(p202_mppt2_volt_GET(pack) == (float)2.6294315E38F);
    assert(p202_mppt2_status_GET(pack) == (uint8_t)(uint8_t)220);
    assert(p202_mppt3_pwm_GET(pack) == (uint16_t)(uint16_t)25571);
    assert(p202_mppt1_amp_GET(pack) == (float)6.2788347E37F);
    assert(p202_mppt1_status_GET(pack) == (uint8_t)(uint8_t)45);
    assert(p202_mppt3_status_GET(pack) == (uint8_t)(uint8_t)177);
    assert(p202_mppt1_volt_GET(pack) == (float) -1.4196332E38F);
    assert(p202_mppt2_amp_GET(pack) == (float)7.443564E36F);
    assert(p202_mppt3_amp_GET(pack) == (float)1.6447508E37F);
    assert(p202_mppt1_pwm_GET(pack) == (uint16_t)(uint16_t)40393);
};


void c_LoopBackDemoChannel_on_ASLCTRL_DATA_203(Bounds_Inside * ph, Pack * pack)
{
    assert(p203_SpoilersEngaged_GET(pack) == (uint8_t)(uint8_t)110);
    assert(p203_h_GET(pack) == (float) -8.204243E37F);
    assert(p203_qRef_GET(pack) == (float)5.0166623E37F);
    assert(p203_PitchAngle_GET(pack) == (float)8.536788E37F);
    assert(p203_hRef_t_GET(pack) == (float)2.5684188E37F);
    assert(p203_PitchAngleRef_GET(pack) == (float)1.2853069E38F);
    assert(p203_YawAngleRef_GET(pack) == (float)1.0213407E38F);
    assert(p203_AirspeedRef_GET(pack) == (float)2.2330905E38F);
    assert(p203_hRef_GET(pack) == (float)2.8496761E38F);
    assert(p203_uThrot2_GET(pack) == (float) -2.7862985E38F);
    assert(p203_RollAngleRef_GET(pack) == (float) -3.3198287E38F);
    assert(p203_uRud_GET(pack) == (float)6.354283E37F);
    assert(p203_timestamp_GET(pack) == (uint64_t)2839711964677103812L);
    assert(p203_uThrot_GET(pack) == (float)1.03157875E37F);
    assert(p203_q_GET(pack) == (float)1.6267226E38F);
    assert(p203_p_GET(pack) == (float) -1.1820807E38F);
    assert(p203_rRef_GET(pack) == (float)1.7257706E38F);
    assert(p203_pRef_GET(pack) == (float)1.4270859E38F);
    assert(p203_r_GET(pack) == (float)2.7407695E38F);
    assert(p203_uAil_GET(pack) == (float)2.509371E38F);
    assert(p203_YawAngle_GET(pack) == (float)1.4467668E38F);
    assert(p203_RollAngle_GET(pack) == (float) -2.0206758E38F);
    assert(p203_aslctrl_mode_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p203_nZ_GET(pack) == (float) -1.8932135E38F);
    assert(p203_uElev_GET(pack) == (float)6.2933285E37F);
};


void c_LoopBackDemoChannel_on_ASLCTRL_DEBUG_204(Bounds_Inside * ph, Pack * pack)
{
    assert(p204_f_4_GET(pack) == (float)1.141648E38F);
    assert(p204_f_2_GET(pack) == (float)1.0981849E38F);
    assert(p204_i8_1_GET(pack) == (uint8_t)(uint8_t)49);
    assert(p204_f_1_GET(pack) == (float)1.4823851E38F);
    assert(p204_f_6_GET(pack) == (float)1.6534829E38F);
    assert(p204_f_5_GET(pack) == (float) -3.473277E37F);
    assert(p204_i32_1_GET(pack) == (uint32_t)3477459138L);
    assert(p204_f_8_GET(pack) == (float) -3.1197891E38F);
    assert(p204_i8_2_GET(pack) == (uint8_t)(uint8_t)65);
    assert(p204_f_7_GET(pack) == (float) -2.0398502E38F);
    assert(p204_f_3_GET(pack) == (float) -1.5716811E38F);
};


void c_LoopBackDemoChannel_on_ASLUAV_STATUS_205(Bounds_Inside * ph, Pack * pack)
{
    assert(p205_SATCOM_status_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p205_Motor_rpm_GET(pack) == (float)1.7545795E38F);
    assert(p205_LED_status_GET(pack) == (uint8_t)(uint8_t)186);
    {
        uint8_t exemplary[] =  {(uint8_t)186, (uint8_t)193, (uint8_t)68, (uint8_t)163, (uint8_t)127, (uint8_t)175, (uint8_t)234, (uint8_t)120} ;
        uint8_t*  sample = p205_Servo_status_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_EKF_EXT_206(Bounds_Inside * ph, Pack * pack)
{
    assert(p206_WindZ_GET(pack) == (float) -1.1933851E38F);
    assert(p206_Airspeed_GET(pack) == (float) -2.9590606E38F);
    assert(p206_beta_GET(pack) == (float) -2.042851E38F);
    assert(p206_WindDir_GET(pack) == (float) -3.1065693E38F);
    assert(p206_alpha_GET(pack) == (float) -1.8749775E38F);
    assert(p206_timestamp_GET(pack) == (uint64_t)3959655981415736750L);
    assert(p206_Windspeed_GET(pack) == (float)2.4845583E38F);
};


void c_LoopBackDemoChannel_on_ASL_OBCTRL_207(Bounds_Inside * ph, Pack * pack)
{
    assert(p207_uRud_GET(pack) == (float)2.4662937E38F);
    assert(p207_obctrl_status_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p207_timestamp_GET(pack) == (uint64_t)1581780945369460773L);
    assert(p207_uElev_GET(pack) == (float) -1.8069589E38F);
    assert(p207_uThrot_GET(pack) == (float) -3.3603827E38F);
    assert(p207_uAilL_GET(pack) == (float) -1.2966283E38F);
    assert(p207_uThrot2_GET(pack) == (float)9.397812E37F);
    assert(p207_uAilR_GET(pack) == (float)3.176943E38F);
};


void c_LoopBackDemoChannel_on_SENS_ATMOS_208(Bounds_Inside * ph, Pack * pack)
{
    assert(p208_TempAmbient_GET(pack) == (float) -1.6661712E38F);
    assert(p208_Humidity_GET(pack) == (float) -4.8499693E37F);
};


void c_LoopBackDemoChannel_on_SENS_BATMON_209(Bounds_Inside * ph, Pack * pack)
{
    assert(p209_cellvoltage5_GET(pack) == (uint16_t)(uint16_t)43417);
    assert(p209_temperature_GET(pack) == (float) -1.6098552E38F);
    assert(p209_cellvoltage1_GET(pack) == (uint16_t)(uint16_t)7204);
    assert(p209_cellvoltage6_GET(pack) == (uint16_t)(uint16_t)11043);
    assert(p209_SoC_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p209_current_GET(pack) == (int16_t)(int16_t)23965);
    assert(p209_voltage_GET(pack) == (uint16_t)(uint16_t)64240);
    assert(p209_cellvoltage3_GET(pack) == (uint16_t)(uint16_t)58613);
    assert(p209_cellvoltage4_GET(pack) == (uint16_t)(uint16_t)26533);
    assert(p209_serialnumber_GET(pack) == (uint16_t)(uint16_t)13491);
    assert(p209_batterystatus_GET(pack) == (uint16_t)(uint16_t)52349);
    assert(p209_cellvoltage2_GET(pack) == (uint16_t)(uint16_t)34114);
    assert(p209_hostfetcontrol_GET(pack) == (uint16_t)(uint16_t)4540);
};


void c_LoopBackDemoChannel_on_FW_SOARING_DATA_210(Bounds_Inside * ph, Pack * pack)
{
    assert(p210_DebugVar1_GET(pack) == (float)5.863707E37F);
    assert(p210_VarW_GET(pack) == (float)1.5323193E38F);
    assert(p210_VarLon_GET(pack) == (float) -1.5190939E38F);
    assert(p210_timestamp_GET(pack) == (uint64_t)8254730636230267653L);
    assert(p210_ThermalGSEast_GET(pack) == (float)1.5255992E38F);
    assert(p210_xLon_GET(pack) == (float)8.168664E37F);
    assert(p210_xLat_GET(pack) == (float) -2.7183572E38F);
    assert(p210_DebugVar2_GET(pack) == (float) -6.694976E37F);
    assert(p210_xR_GET(pack) == (float) -8.115779E37F);
    assert(p210_DistToSoarPoint_GET(pack) == (float)2.6811417E38F);
    assert(p210_VarLat_GET(pack) == (float)1.8490593E38F);
    assert(p210_valid_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p210_timestampModeChanged_GET(pack) == (uint64_t)3964925808991726004L);
    assert(p210_vSinkExp_GET(pack) == (float)1.6848517E38F);
    assert(p210_z2_exp_GET(pack) == (float)3.3330948E38F);
    assert(p210_z1_exp_GET(pack) == (float)2.1347374E38F);
    assert(p210_LoiterRadius_GET(pack) == (float) -2.3070618E38F);
    assert(p210_VarR_GET(pack) == (float) -1.7816513E38F);
    assert(p210_xW_GET(pack) == (float)2.9684097E38F);
    assert(p210_TSE_dot_GET(pack) == (float) -2.5806823E37F);
    assert(p210_z1_LocalUpdraftSpeed_GET(pack) == (float)7.254837E37F);
    assert(p210_z2_DeltaRoll_GET(pack) == (float)1.0596796E38F);
    assert(p210_ThermalGSNorth_GET(pack) == (float) -1.5927078E38F);
    assert(p210_LoiterDirection_GET(pack) == (float) -2.1322698E38F);
    assert(p210_ControlMode_GET(pack) == (uint8_t)(uint8_t)108);
};


void c_LoopBackDemoChannel_on_SENSORPOD_STATUS_211(Bounds_Inside * ph, Pack * pack)
{
    assert(p211_cpu_temp_GET(pack) == (uint8_t)(uint8_t)44);
    assert(p211_visensor_rate_4_GET(pack) == (uint8_t)(uint8_t)185);
    assert(p211_visensor_rate_2_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p211_visensor_rate_3_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p211_visensor_rate_1_GET(pack) == (uint8_t)(uint8_t)109);
    assert(p211_timestamp_GET(pack) == (uint64_t)4828835503268257265L);
    assert(p211_recording_nodes_count_GET(pack) == (uint8_t)(uint8_t)72);
    assert(p211_free_space_GET(pack) == (uint16_t)(uint16_t)35977);
};


void c_LoopBackDemoChannel_on_SENS_POWER_BOARD_212(Bounds_Inside * ph, Pack * pack)
{
    assert(p212_pwr_brd_mot_l_amp_GET(pack) == (float)8.649035E37F);
    assert(p212_pwr_brd_servo_2_amp_GET(pack) == (float) -2.300927E38F);
    assert(p212_pwr_brd_system_volt_GET(pack) == (float)1.7004213E38F);
    assert(p212_pwr_brd_status_GET(pack) == (uint8_t)(uint8_t)156);
    assert(p212_pwr_brd_servo_4_amp_GET(pack) == (float)3.2339803E38F);
    assert(p212_pwr_brd_aux_amp_GET(pack) == (float) -2.094471E38F);
    assert(p212_pwr_brd_servo_3_amp_GET(pack) == (float) -3.2302747E38F);
    assert(p212_pwr_brd_servo_1_amp_GET(pack) == (float) -1.8028186E38F);
    assert(p212_timestamp_GET(pack) == (uint64_t)7977805117771849541L);
    assert(p212_pwr_brd_led_status_GET(pack) == (uint8_t)(uint8_t)164);
    assert(p212_pwr_brd_servo_volt_GET(pack) == (float)2.5396808E38F);
    assert(p212_pwr_brd_mot_r_amp_GET(pack) == (float) -2.4720256E38F);
};


void c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_vel_ratio_GET(pack) == (float) -1.924471E38F);
    assert(p230_pos_vert_ratio_GET(pack) == (float) -3.3297192E38F);
    assert(p230_mag_ratio_GET(pack) == (float)2.5343443E38F);
    assert(p230_time_usec_GET(pack) == (uint64_t)2222425225711890723L);
    assert(p230_tas_ratio_GET(pack) == (float) -1.6221537E38F);
    assert(p230_flags_GET(pack) == e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS);
    assert(p230_hagl_ratio_GET(pack) == (float) -1.1146074E38F);
    assert(p230_pos_horiz_ratio_GET(pack) == (float) -1.0157043E38F);
    assert(p230_pos_vert_accuracy_GET(pack) == (float)1.3621639E38F);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float) -1.3561237E38F);
};


void c_LoopBackDemoChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_wind_alt_GET(pack) == (float)5.032081E37F);
    assert(p231_wind_z_GET(pack) == (float)6.33732E37F);
    assert(p231_var_vert_GET(pack) == (float) -3.2011423E38F);
    assert(p231_vert_accuracy_GET(pack) == (float) -7.8305206E37F);
    assert(p231_horiz_accuracy_GET(pack) == (float) -1.8065092E38F);
    assert(p231_var_horiz_GET(pack) == (float) -3.0530284E38F);
    assert(p231_wind_y_GET(pack) == (float)1.3150941E38F);
    assert(p231_time_usec_GET(pack) == (uint64_t)4339778027335178994L);
    assert(p231_wind_x_GET(pack) == (float) -1.5283752E38F);
};


void c_LoopBackDemoChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_vdop_GET(pack) == (float) -2.484766E38F);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)11);
    assert(p232_ve_GET(pack) == (float) -2.5622385E37F);
    assert(p232_time_usec_GET(pack) == (uint64_t)4555437622948578456L);
    assert(p232_lon_GET(pack) == (int32_t) -1885416528);
    assert(p232_horiz_accuracy_GET(pack) == (float) -7.165747E37F);
    assert(p232_ignore_flags_GET(pack) == e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_HORIZ);
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)63002);
    assert(p232_vd_GET(pack) == (float)3.2100513E38F);
    assert(p232_speed_accuracy_GET(pack) == (float)6.4142664E37F);
    assert(p232_alt_GET(pack) == (float)1.2941889E37F);
    assert(p232_hdop_GET(pack) == (float)8.850664E37F);
    assert(p232_vert_accuracy_GET(pack) == (float) -3.0716042E38F);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)4109536047L);
    assert(p232_lat_GET(pack) == (int32_t) -861731653);
    assert(p232_vn_GET(pack) == (float) -1.4626416E37F);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)112);
};


void c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)38, (uint8_t)23, (uint8_t)142, (uint8_t)244, (uint8_t)9, (uint8_t)233, (uint8_t)143, (uint8_t)224, (uint8_t)106, (uint8_t)255, (uint8_t)102, (uint8_t)106, (uint8_t)128, (uint8_t)216, (uint8_t)163, (uint8_t)178, (uint8_t)126, (uint8_t)24, (uint8_t)148, (uint8_t)39, (uint8_t)94, (uint8_t)199, (uint8_t)16, (uint8_t)45, (uint8_t)57, (uint8_t)44, (uint8_t)116, (uint8_t)203, (uint8_t)13, (uint8_t)81, (uint8_t)220, (uint8_t)55, (uint8_t)47, (uint8_t)255, (uint8_t)3, (uint8_t)185, (uint8_t)200, (uint8_t)189, (uint8_t)131, (uint8_t)91, (uint8_t)72, (uint8_t)0, (uint8_t)155, (uint8_t)66, (uint8_t)173, (uint8_t)145, (uint8_t)65, (uint8_t)81, (uint8_t)16, (uint8_t)73, (uint8_t)148, (uint8_t)182, (uint8_t)93, (uint8_t)68, (uint8_t)218, (uint8_t)173, (uint8_t)192, (uint8_t)214, (uint8_t)215, (uint8_t)172, (uint8_t)207, (uint8_t)89, (uint8_t)87, (uint8_t)192, (uint8_t)61, (uint8_t)119, (uint8_t)45, (uint8_t)183, (uint8_t)171, (uint8_t)9, (uint8_t)39, (uint8_t)69, (uint8_t)45, (uint8_t)221, (uint8_t)53, (uint8_t)29, (uint8_t)122, (uint8_t)63, (uint8_t)68, (uint8_t)168, (uint8_t)253, (uint8_t)235, (uint8_t)222, (uint8_t)150, (uint8_t)144, (uint8_t)143, (uint8_t)201, (uint8_t)50, (uint8_t)55, (uint8_t)34, (uint8_t)57, (uint8_t)216, (uint8_t)151, (uint8_t)146, (uint8_t)221, (uint8_t)137, (uint8_t)187, (uint8_t)167, (uint8_t)87, (uint8_t)134, (uint8_t)105, (uint8_t)182, (uint8_t)129, (uint8_t)122, (uint8_t)244, (uint8_t)16, (uint8_t)115, (uint8_t)223, (uint8_t)234, (uint8_t)54, (uint8_t)22, (uint8_t)188, (uint8_t)192, (uint8_t)41, (uint8_t)98, (uint8_t)157, (uint8_t)113, (uint8_t)4, (uint8_t)147, (uint8_t)199, (uint8_t)1, (uint8_t)37, (uint8_t)194, (uint8_t)181, (uint8_t)219, (uint8_t)23, (uint8_t)143, (uint8_t)232, (uint8_t)62, (uint8_t)155, (uint8_t)83, (uint8_t)79, (uint8_t)200, (uint8_t)236, (uint8_t)3, (uint8_t)247, (uint8_t)66, (uint8_t)199, (uint8_t)35, (uint8_t)149, (uint8_t)99, (uint8_t)156, (uint8_t)199, (uint8_t)231, (uint8_t)43, (uint8_t)23, (uint8_t)116, (uint8_t)65, (uint8_t)169, (uint8_t)194, (uint8_t)148, (uint8_t)0, (uint8_t)71, (uint8_t)88, (uint8_t)245, (uint8_t)135, (uint8_t)32, (uint8_t)145, (uint8_t)170, (uint8_t)160, (uint8_t)199, (uint8_t)66, (uint8_t)50, (uint8_t)70, (uint8_t)175, (uint8_t)112, (uint8_t)150, (uint8_t)38, (uint8_t)240, (uint8_t)39, (uint8_t)49, (uint8_t)80, (uint8_t)85, (uint8_t)155, (uint8_t)0, (uint8_t)2, (uint8_t)212, (uint8_t)195, (uint8_t)204, (uint8_t)17} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)188);
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)237);
};


void c_LoopBackDemoChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_longitude_GET(pack) == (int32_t) -487214218);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t) -23334);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)31);
    assert(p234_latitude_GET(pack) == (int32_t)444984026);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t)10450);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t) -122);
    assert(p234_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)4735);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t)39);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t) -5);
    assert(p234_custom_mode_GET(pack) == (uint32_t)1441592018L);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)47362);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t)27742);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t) -5039);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)88);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t)52);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t)25928);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)177);
};


void c_LoopBackDemoChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_clipping_1_GET(pack) == (uint32_t)203456575L);
    assert(p241_clipping_0_GET(pack) == (uint32_t)4045557072L);
    assert(p241_clipping_2_GET(pack) == (uint32_t)2023792727L);
    assert(p241_vibration_z_GET(pack) == (float)1.7776869E38F);
    assert(p241_time_usec_GET(pack) == (uint64_t)4812390448161481924L);
    assert(p241_vibration_x_GET(pack) == (float) -1.0071702E38F);
    assert(p241_vibration_y_GET(pack) == (float)2.524764E38F);
};


void c_LoopBackDemoChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_latitude_GET(pack) == (int32_t) -1652085522);
    assert(p242_approach_x_GET(pack) == (float)9.843417E37F);
    assert(p242_altitude_GET(pack) == (int32_t)1253789822);
    assert(p242_x_GET(pack) == (float) -6.500271E37F);
    assert(p242_longitude_GET(pack) == (int32_t) -1321795391);
    assert(p242_approach_y_GET(pack) == (float)2.987103E38F);
    assert(p242_time_usec_TRY(ph) == (uint64_t)6048627198369473930L);
    assert(p242_y_GET(pack) == (float)2.0991635E38F);
    assert(p242_approach_z_GET(pack) == (float) -2.7606063E38F);
    {
        float exemplary[] =  {2.5609722E38F, -3.2643139E38F, 4.5071637E37F, -1.2297266E38F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_z_GET(pack) == (float)1.674138E38F);
};


void c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_altitude_GET(pack) == (int32_t)1381970857);
    assert(p243_time_usec_TRY(ph) == (uint64_t)4689729217731423973L);
    assert(p243_approach_y_GET(pack) == (float)2.8431512E37F);
    assert(p243_y_GET(pack) == (float) -3.3332408E38F);
    assert(p243_latitude_GET(pack) == (int32_t) -1184188248);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)250);
    assert(p243_approach_z_GET(pack) == (float) -1.1804108E38F);
    assert(p243_x_GET(pack) == (float)3.2857491E38F);
    assert(p243_approach_x_GET(pack) == (float) -2.6963608E38F);
    assert(p243_longitude_GET(pack) == (int32_t) -746852402);
    assert(p243_z_GET(pack) == (float) -1.5725698E38F);
    {
        float exemplary[] =  {2.3337851E38F, -2.706834E38F, 2.3572868E38F, 1.1489484E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)23162);
    assert(p244_interval_us_GET(pack) == (int32_t)97104549);
};


void c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED);
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_MC);
};


void c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)46649);
    assert(p246_altitude_GET(pack) == (int32_t) -2079155041);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)17816);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)9866);
    assert(p246_callsign_LEN(ph) == 5);
    {
        char16_t * exemplary = u"imiuG";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t) -935);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)3178382147L);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_POINT_OBSTACLE);
    assert(p246_flags_GET(pack) == e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
    assert(p246_lon_GET(pack) == (int32_t)69433851);
    assert(p246_lat_GET(pack) == (int32_t)1442366609);
};


void c_LoopBackDemoChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_REPORT);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float) -2.6483261E38F);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH);
    assert(p247_altitude_minimum_delta_GET(pack) == (float) -1.3951047E38F);
    assert(p247_time_to_minimum_delta_GET(pack) == (float)1.0671214E38F);
    assert(p247_id_GET(pack) == (uint32_t)2225551542L);
};


void c_LoopBackDemoChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)26226);
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)199);
    {
        uint8_t exemplary[] =  {(uint8_t)124, (uint8_t)62, (uint8_t)85, (uint8_t)10, (uint8_t)189, (uint8_t)227, (uint8_t)9, (uint8_t)69, (uint8_t)235, (uint8_t)197, (uint8_t)166, (uint8_t)221, (uint8_t)117, (uint8_t)194, (uint8_t)5, (uint8_t)169, (uint8_t)224, (uint8_t)222, (uint8_t)145, (uint8_t)5, (uint8_t)206, (uint8_t)18, (uint8_t)205, (uint8_t)168, (uint8_t)227, (uint8_t)186, (uint8_t)93, (uint8_t)115, (uint8_t)56, (uint8_t)14, (uint8_t)105, (uint8_t)202, (uint8_t)5, (uint8_t)21, (uint8_t)28, (uint8_t)90, (uint8_t)239, (uint8_t)158, (uint8_t)6, (uint8_t)252, (uint8_t)90, (uint8_t)153, (uint8_t)16, (uint8_t)115, (uint8_t)211, (uint8_t)97, (uint8_t)53, (uint8_t)16, (uint8_t)59, (uint8_t)9, (uint8_t)99, (uint8_t)245, (uint8_t)208, (uint8_t)238, (uint8_t)197, (uint8_t)248, (uint8_t)130, (uint8_t)17, (uint8_t)182, (uint8_t)255, (uint8_t)63, (uint8_t)245, (uint8_t)125, (uint8_t)113, (uint8_t)198, (uint8_t)168, (uint8_t)154, (uint8_t)252, (uint8_t)95, (uint8_t)225, (uint8_t)42, (uint8_t)124, (uint8_t)51, (uint8_t)28, (uint8_t)239, (uint8_t)209, (uint8_t)2, (uint8_t)95, (uint8_t)198, (uint8_t)60, (uint8_t)166, (uint8_t)125, (uint8_t)162, (uint8_t)82, (uint8_t)114, (uint8_t)104, (uint8_t)254, (uint8_t)208, (uint8_t)88, (uint8_t)106, (uint8_t)225, (uint8_t)238, (uint8_t)128, (uint8_t)37, (uint8_t)194, (uint8_t)60, (uint8_t)117, (uint8_t)38, (uint8_t)20, (uint8_t)193, (uint8_t)27, (uint8_t)85, (uint8_t)28, (uint8_t)217, (uint8_t)174, (uint8_t)27, (uint8_t)244, (uint8_t)18, (uint8_t)195, (uint8_t)74, (uint8_t)181, (uint8_t)27, (uint8_t)109, (uint8_t)181, (uint8_t)227, (uint8_t)57, (uint8_t)105, (uint8_t)51, (uint8_t)123, (uint8_t)1, (uint8_t)202, (uint8_t)107, (uint8_t)23, (uint8_t)83, (uint8_t)197, (uint8_t)84, (uint8_t)178, (uint8_t)228, (uint8_t)24, (uint8_t)9, (uint8_t)239, (uint8_t)160, (uint8_t)213, (uint8_t)91, (uint8_t)47, (uint8_t)194, (uint8_t)38, (uint8_t)109, (uint8_t)28, (uint8_t)42, (uint8_t)116, (uint8_t)85, (uint8_t)40, (uint8_t)102, (uint8_t)139, (uint8_t)36, (uint8_t)87, (uint8_t)211, (uint8_t)111, (uint8_t)87, (uint8_t)209, (uint8_t)240, (uint8_t)37, (uint8_t)234, (uint8_t)130, (uint8_t)6, (uint8_t)21, (uint8_t)84, (uint8_t)130, (uint8_t)243, (uint8_t)12, (uint8_t)253, (uint8_t)132, (uint8_t)247, (uint8_t)97, (uint8_t)33, (uint8_t)22, (uint8_t)120, (uint8_t)61, (uint8_t)76, (uint8_t)113, (uint8_t)103, (uint8_t)66, (uint8_t)147, (uint8_t)156, (uint8_t)99, (uint8_t)243, (uint8_t)143, (uint8_t)89, (uint8_t)123, (uint8_t)165, (uint8_t)226, (uint8_t)85, (uint8_t)206, (uint8_t)88, (uint8_t)150, (uint8_t)40, (uint8_t)74, (uint8_t)31, (uint8_t)14, (uint8_t)0, (uint8_t)116, (uint8_t)129, (uint8_t)175, (uint8_t)110, (uint8_t)240, (uint8_t)218, (uint8_t)220, (uint8_t)170, (uint8_t)201, (uint8_t)114, (uint8_t)121, (uint8_t)108, (uint8_t)78, (uint8_t)7, (uint8_t)233, (uint8_t)218, (uint8_t)186, (uint8_t)98, (uint8_t)58, (uint8_t)8, (uint8_t)50, (uint8_t)60, (uint8_t)37, (uint8_t)123, (uint8_t)127, (uint8_t)171, (uint8_t)130, (uint8_t)230, (uint8_t)235, (uint8_t)236, (uint8_t)1, (uint8_t)13, (uint8_t)53, (uint8_t)76, (uint8_t)133, (uint8_t)231, (uint8_t)43, (uint8_t)166, (uint8_t)114, (uint8_t)91, (uint8_t)220, (uint8_t)191, (uint8_t)146, (uint8_t)56, (uint8_t)212, (uint8_t)161, (uint8_t)137, (uint8_t)123, (uint8_t)162, (uint8_t)105, (uint8_t)220, (uint8_t)155, (uint8_t)11, (uint8_t)58, (uint8_t)175, (uint8_t)41, (uint8_t)180, (uint8_t)148} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)152);
};


void c_LoopBackDemoChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)13755);
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)7);
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)43);
    {
        int8_t exemplary[] =  {(int8_t) -104, (int8_t) -44, (int8_t)105, (int8_t)64, (int8_t)69, (int8_t)48, (int8_t) -25, (int8_t)79, (int8_t) -45, (int8_t) -74, (int8_t)76, (int8_t) -39, (int8_t) -63, (int8_t) -31, (int8_t)31, (int8_t)55, (int8_t) -18, (int8_t)94, (int8_t)97, (int8_t) -89, (int8_t) -60, (int8_t) -23, (int8_t)90, (int8_t) -61, (int8_t) -57, (int8_t)77, (int8_t) -36, (int8_t)69, (int8_t) -119, (int8_t)10, (int8_t)45, (int8_t)90} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_name_LEN(ph) == 7);
    {
        char16_t * exemplary = u"rgotjwa";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p250_z_GET(pack) == (float)1.6140032E37F);
    assert(p250_x_GET(pack) == (float)3.2966663E38F);
    assert(p250_y_GET(pack) == (float) -1.9186628E38F);
    assert(p250_time_usec_GET(pack) == (uint64_t)8444417496427771690L);
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)943151648L);
    assert(p251_name_LEN(ph) == 4);
    {
        char16_t * exemplary = u"Ookj";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p251_value_GET(pack) == (float) -1.61227E37F);
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_value_GET(pack) == (int32_t) -67626460);
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)1278932014L);
    assert(p252_name_LEN(ph) == 8);
    {
        char16_t * exemplary = u"cpujHodw";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_NOTICE);
    assert(p253_text_LEN(ph) == 19);
    {
        char16_t * exemplary = u"xepPfityymscfAqcpwM";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 38);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p254_value_GET(pack) == (float) -1.2460224E38F);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)3429019146L);
};


void c_LoopBackDemoChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)139);
    {
        uint8_t exemplary[] =  {(uint8_t)213, (uint8_t)213, (uint8_t)196, (uint8_t)218, (uint8_t)15, (uint8_t)224, (uint8_t)248, (uint8_t)74, (uint8_t)166, (uint8_t)57, (uint8_t)80, (uint8_t)18, (uint8_t)230, (uint8_t)140, (uint8_t)116, (uint8_t)50, (uint8_t)216, (uint8_t)252, (uint8_t)234, (uint8_t)37, (uint8_t)237, (uint8_t)215, (uint8_t)99, (uint8_t)120, (uint8_t)17, (uint8_t)248, (uint8_t)190, (uint8_t)190, (uint8_t)249, (uint8_t)243, (uint8_t)149, (uint8_t)63} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)4830817049512321994L);
};


void c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_last_change_ms_GET(pack) == (uint32_t)3180213569L);
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)581012751L);
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)88);
};


void c_LoopBackDemoChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_tune_LEN(ph) == 22);
    {
        char16_t * exemplary = u"kcodxpbjgowiIzedTqkzoe";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 44);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)152);
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)17);
};


void c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)7148);
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)185);
    assert(p259_firmware_version_GET(pack) == (uint32_t)2008976050L);
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)61822);
    assert(p259_sensor_size_h_GET(pack) == (float)4.5183016E36F);
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)3041948288L);
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)64762);
    assert(p259_flags_GET(pack) == e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
    {
        uint8_t exemplary[] =  {(uint8_t)88, (uint8_t)109, (uint8_t)203, (uint8_t)204, (uint8_t)4, (uint8_t)63, (uint8_t)124, (uint8_t)108, (uint8_t)90, (uint8_t)252, (uint8_t)171, (uint8_t)143, (uint8_t)218, (uint8_t)157, (uint8_t)187, (uint8_t)46, (uint8_t)87, (uint8_t)13, (uint8_t)103, (uint8_t)241, (uint8_t)39, (uint8_t)245, (uint8_t)241, (uint8_t)128, (uint8_t)175, (uint8_t)202, (uint8_t)129, (uint8_t)194, (uint8_t)195, (uint8_t)74, (uint8_t)206, (uint8_t)173} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)95, (uint8_t)159, (uint8_t)182, (uint8_t)18, (uint8_t)20, (uint8_t)216, (uint8_t)158, (uint8_t)218, (uint8_t)210, (uint8_t)42, (uint8_t)246, (uint8_t)58, (uint8_t)72, (uint8_t)51, (uint8_t)130, (uint8_t)225, (uint8_t)147, (uint8_t)177, (uint8_t)51, (uint8_t)252, (uint8_t)225, (uint8_t)193, (uint8_t)209, (uint8_t)65, (uint8_t)11, (uint8_t)215, (uint8_t)230, (uint8_t)228, (uint8_t)150, (uint8_t)54, (uint8_t)203, (uint8_t)175} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_sensor_size_v_GET(pack) == (float) -2.7029937E38F);
    assert(p259_cam_definition_uri_LEN(ph) == 51);
    {
        char16_t * exemplary = u"bqaLtysquffApftVWkfopzmhiarutwxUkiarvbcEvgVejtbjecB";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 102);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_focal_length_GET(pack) == (float) -7.86967E36F);
};


void c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)3582678275L);
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_VIDEO);
};


void c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)19);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p261_read_speed_GET(pack) == (float)1.1372106E38F);
    assert(p261_write_speed_GET(pack) == (float)2.7982332E38F);
    assert(p261_used_capacity_GET(pack) == (float) -1.738584E38F);
    assert(p261_total_capacity_GET(pack) == (float) -2.0407491E38F);
    assert(p261_available_capacity_GET(pack) == (float)2.163062E38F);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)3886475785L);
};


void c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)115010151L);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)243);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)61);
    assert(p262_available_capacity_GET(pack) == (float) -1.5141892E38F);
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)3335658806L);
    assert(p262_image_interval_GET(pack) == (float) -2.8617732E38F);
};


void c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_alt_GET(pack) == (int32_t) -1726351602);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p263_file_url_LEN(ph) == 102);
    {
        char16_t * exemplary = u"xoztysfrmzjtsjovmkceeUsrclqyjsjhgSlqwhzzoyknruasnDlFzaorqpLflazdckruPoUklxdwurlxsoyjrjzcdpnzgqhgjbweuv";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 204);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-1.4205667E38F, 2.7128394E38F, 3.390555E38F, 1.292218E38F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_image_index_GET(pack) == (int32_t) -1529274944);
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)1667410008L);
    assert(p263_time_utc_GET(pack) == (uint64_t)2630662737987732657L);
    assert(p263_lat_GET(pack) == (int32_t)2098175055);
    assert(p263_relative_alt_GET(pack) == (int32_t) -1912880356);
    assert(p263_lon_GET(pack) == (int32_t)908280405);
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t)68);
};


void c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_flight_uuid_GET(pack) == (uint64_t)3609340950990879452L);
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)3554170764L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)1658814051001568727L);
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)2025762840362005073L);
};


void c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_roll_GET(pack) == (float) -2.9459328E38F);
    assert(p265_yaw_GET(pack) == (float)1.4478411E38F);
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)1239268293L);
    assert(p265_pitch_GET(pack) == (float) -2.2581035E38F);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)180);
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)251);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)41867);
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)168);
    {
        uint8_t exemplary[] =  {(uint8_t)215, (uint8_t)215, (uint8_t)7, (uint8_t)16, (uint8_t)179, (uint8_t)237, (uint8_t)31, (uint8_t)208, (uint8_t)58, (uint8_t)170, (uint8_t)189, (uint8_t)114, (uint8_t)149, (uint8_t)34, (uint8_t)14, (uint8_t)21, (uint8_t)90, (uint8_t)74, (uint8_t)152, (uint8_t)196, (uint8_t)59, (uint8_t)226, (uint8_t)246, (uint8_t)0, (uint8_t)230, (uint8_t)47, (uint8_t)80, (uint8_t)11, (uint8_t)7, (uint8_t)106, (uint8_t)138, (uint8_t)109, (uint8_t)67, (uint8_t)186, (uint8_t)90, (uint8_t)73, (uint8_t)134, (uint8_t)34, (uint8_t)62, (uint8_t)75, (uint8_t)250, (uint8_t)49, (uint8_t)55, (uint8_t)90, (uint8_t)27, (uint8_t)207, (uint8_t)179, (uint8_t)246, (uint8_t)100, (uint8_t)34, (uint8_t)177, (uint8_t)47, (uint8_t)146, (uint8_t)12, (uint8_t)155, (uint8_t)91, (uint8_t)177, (uint8_t)22, (uint8_t)96, (uint8_t)120, (uint8_t)209, (uint8_t)106, (uint8_t)12, (uint8_t)209, (uint8_t)239, (uint8_t)8, (uint8_t)58, (uint8_t)244, (uint8_t)60, (uint8_t)72, (uint8_t)36, (uint8_t)27, (uint8_t)232, (uint8_t)148, (uint8_t)115, (uint8_t)171, (uint8_t)218, (uint8_t)143, (uint8_t)89, (uint8_t)8, (uint8_t)27, (uint8_t)22, (uint8_t)235, (uint8_t)86, (uint8_t)27, (uint8_t)210, (uint8_t)94, (uint8_t)82, (uint8_t)207, (uint8_t)42, (uint8_t)181, (uint8_t)34, (uint8_t)170, (uint8_t)12, (uint8_t)116, (uint8_t)9, (uint8_t)170, (uint8_t)169, (uint8_t)203, (uint8_t)179, (uint8_t)73, (uint8_t)94, (uint8_t)226, (uint8_t)54, (uint8_t)147, (uint8_t)8, (uint8_t)201, (uint8_t)252, (uint8_t)192, (uint8_t)207, (uint8_t)155, (uint8_t)73, (uint8_t)184, (uint8_t)41, (uint8_t)215, (uint8_t)110, (uint8_t)33, (uint8_t)254, (uint8_t)235, (uint8_t)13, (uint8_t)157, (uint8_t)129, (uint8_t)115, (uint8_t)63, (uint8_t)126, (uint8_t)106, (uint8_t)243, (uint8_t)178, (uint8_t)179, (uint8_t)161, (uint8_t)6, (uint8_t)74, (uint8_t)68, (uint8_t)113, (uint8_t)91, (uint8_t)115, (uint8_t)50, (uint8_t)110, (uint8_t)81, (uint8_t)101, (uint8_t)166, (uint8_t)230, (uint8_t)30, (uint8_t)250, (uint8_t)18, (uint8_t)112, (uint8_t)192, (uint8_t)237, (uint8_t)55, (uint8_t)162, (uint8_t)185, (uint8_t)37, (uint8_t)249, (uint8_t)109, (uint8_t)3, (uint8_t)235, (uint8_t)157, (uint8_t)234, (uint8_t)4, (uint8_t)204, (uint8_t)45, (uint8_t)17, (uint8_t)229, (uint8_t)117, (uint8_t)134, (uint8_t)98, (uint8_t)46, (uint8_t)175, (uint8_t)116, (uint8_t)247, (uint8_t)78, (uint8_t)40, (uint8_t)155, (uint8_t)105, (uint8_t)57, (uint8_t)191, (uint8_t)206, (uint8_t)252, (uint8_t)146, (uint8_t)60, (uint8_t)238, (uint8_t)186, (uint8_t)225, (uint8_t)20, (uint8_t)114, (uint8_t)24, (uint8_t)33, (uint8_t)169, (uint8_t)166, (uint8_t)162, (uint8_t)63, (uint8_t)168, (uint8_t)46, (uint8_t)182, (uint8_t)129, (uint8_t)198, (uint8_t)30, (uint8_t)173, (uint8_t)66, (uint8_t)109, (uint8_t)245, (uint8_t)182, (uint8_t)97, (uint8_t)11, (uint8_t)120, (uint8_t)215, (uint8_t)20, (uint8_t)164, (uint8_t)49, (uint8_t)40, (uint8_t)36, (uint8_t)249, (uint8_t)89, (uint8_t)98, (uint8_t)198, (uint8_t)216, (uint8_t)161, (uint8_t)35, (uint8_t)66, (uint8_t)122, (uint8_t)165, (uint8_t)140, (uint8_t)28, (uint8_t)240, (uint8_t)70, (uint8_t)216, (uint8_t)78, (uint8_t)71, (uint8_t)25, (uint8_t)174, (uint8_t)25, (uint8_t)239, (uint8_t)25, (uint8_t)195, (uint8_t)202, (uint8_t)200, (uint8_t)132, (uint8_t)222, (uint8_t)42, (uint8_t)108, (uint8_t)27, (uint8_t)107, (uint8_t)234, (uint8_t)31, (uint8_t)149, (uint8_t)52, (uint8_t)245, (uint8_t)236, (uint8_t)196} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)248);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)208);
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)160);
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)58062);
    {
        uint8_t exemplary[] =  {(uint8_t)135, (uint8_t)1, (uint8_t)0, (uint8_t)205, (uint8_t)154, (uint8_t)128, (uint8_t)136, (uint8_t)20, (uint8_t)251, (uint8_t)160, (uint8_t)19, (uint8_t)20, (uint8_t)164, (uint8_t)138, (uint8_t)64, (uint8_t)152, (uint8_t)133, (uint8_t)9, (uint8_t)198, (uint8_t)93, (uint8_t)138, (uint8_t)180, (uint8_t)69, (uint8_t)83, (uint8_t)22, (uint8_t)174, (uint8_t)136, (uint8_t)250, (uint8_t)167, (uint8_t)255, (uint8_t)241, (uint8_t)237, (uint8_t)23, (uint8_t)137, (uint8_t)201, (uint8_t)150, (uint8_t)140, (uint8_t)28, (uint8_t)231, (uint8_t)223, (uint8_t)241, (uint8_t)238, (uint8_t)230, (uint8_t)20, (uint8_t)17, (uint8_t)249, (uint8_t)245, (uint8_t)135, (uint8_t)107, (uint8_t)90, (uint8_t)42, (uint8_t)47, (uint8_t)5, (uint8_t)148, (uint8_t)204, (uint8_t)179, (uint8_t)18, (uint8_t)160, (uint8_t)232, (uint8_t)207, (uint8_t)52, (uint8_t)39, (uint8_t)94, (uint8_t)59, (uint8_t)175, (uint8_t)216, (uint8_t)239, (uint8_t)79, (uint8_t)226, (uint8_t)33, (uint8_t)67, (uint8_t)139, (uint8_t)62, (uint8_t)92, (uint8_t)202, (uint8_t)155, (uint8_t)150, (uint8_t)50, (uint8_t)196, (uint8_t)71, (uint8_t)220, (uint8_t)210, (uint8_t)163, (uint8_t)157, (uint8_t)244, (uint8_t)42, (uint8_t)87, (uint8_t)170, (uint8_t)113, (uint8_t)0, (uint8_t)241, (uint8_t)215, (uint8_t)139, (uint8_t)91, (uint8_t)143, (uint8_t)1, (uint8_t)103, (uint8_t)116, (uint8_t)62, (uint8_t)162, (uint8_t)182, (uint8_t)44, (uint8_t)255, (uint8_t)70, (uint8_t)220, (uint8_t)138, (uint8_t)88, (uint8_t)180, (uint8_t)77, (uint8_t)240, (uint8_t)165, (uint8_t)216, (uint8_t)118, (uint8_t)122, (uint8_t)113, (uint8_t)224, (uint8_t)42, (uint8_t)147, (uint8_t)97, (uint8_t)48, (uint8_t)145, (uint8_t)11, (uint8_t)69, (uint8_t)154, (uint8_t)251, (uint8_t)126, (uint8_t)226, (uint8_t)190, (uint8_t)203, (uint8_t)172, (uint8_t)107, (uint8_t)221, (uint8_t)182, (uint8_t)226, (uint8_t)241, (uint8_t)52, (uint8_t)38, (uint8_t)19, (uint8_t)85, (uint8_t)66, (uint8_t)11, (uint8_t)162, (uint8_t)60, (uint8_t)21, (uint8_t)150, (uint8_t)147, (uint8_t)72, (uint8_t)98, (uint8_t)95, (uint8_t)142, (uint8_t)22, (uint8_t)65, (uint8_t)229, (uint8_t)131, (uint8_t)78, (uint8_t)126, (uint8_t)187, (uint8_t)140, (uint8_t)250, (uint8_t)100, (uint8_t)66, (uint8_t)108, (uint8_t)102, (uint8_t)173, (uint8_t)17, (uint8_t)62, (uint8_t)230, (uint8_t)230, (uint8_t)55, (uint8_t)162, (uint8_t)166, (uint8_t)242, (uint8_t)26, (uint8_t)229, (uint8_t)203, (uint8_t)128, (uint8_t)120, (uint8_t)16, (uint8_t)237, (uint8_t)149, (uint8_t)188, (uint8_t)236, (uint8_t)225, (uint8_t)157, (uint8_t)78, (uint8_t)232, (uint8_t)196, (uint8_t)85, (uint8_t)90, (uint8_t)75, (uint8_t)117, (uint8_t)100, (uint8_t)242, (uint8_t)58, (uint8_t)224, (uint8_t)185, (uint8_t)39, (uint8_t)45, (uint8_t)255, (uint8_t)93, (uint8_t)87, (uint8_t)103, (uint8_t)238, (uint8_t)54, (uint8_t)237, (uint8_t)195, (uint8_t)32, (uint8_t)1, (uint8_t)36, (uint8_t)147, (uint8_t)217, (uint8_t)189, (uint8_t)140, (uint8_t)73, (uint8_t)28, (uint8_t)207, (uint8_t)162, (uint8_t)226, (uint8_t)111, (uint8_t)183, (uint8_t)37, (uint8_t)41, (uint8_t)97, (uint8_t)43, (uint8_t)160, (uint8_t)171, (uint8_t)90, (uint8_t)128, (uint8_t)216, (uint8_t)235, (uint8_t)222, (uint8_t)150, (uint8_t)74, (uint8_t)90, (uint8_t)79, (uint8_t)99, (uint8_t)29, (uint8_t)100, (uint8_t)113, (uint8_t)35, (uint8_t)171, (uint8_t)131, (uint8_t)73, (uint8_t)148, (uint8_t)67, (uint8_t)224, (uint8_t)39, (uint8_t)132, (uint8_t)114} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)76);
};


void c_LoopBackDemoChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)48209);
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)159);
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)238);
};


void c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)15223);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)23910);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)42767);
    assert(p269_framerate_GET(pack) == (float) -1.892122E38F);
    assert(p269_uri_LEN(ph) == 70);
    {
        char16_t * exemplary = u"qnetjvpiurvrcaqtFemrEkifhctvkClvgucqwikxpspsizvrlmckfUdtksxhydlBjmbmgj";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 140);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_bitrate_GET(pack) == (uint32_t)3518055541L);
};


void c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)55641);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)32635);
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)45);
    assert(p270_bitrate_GET(pack) == (uint32_t)1787464869L);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)58576);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)246);
    assert(p270_framerate_GET(pack) == (float)3.0801269E38F);
    assert(p270_uri_LEN(ph) == 208);
    {
        char16_t * exemplary = u"zqtjgrmspZdctrqdkduhjglehsgozrlnPdgnecgpceaAmxudmwxmpdreCbybzqtubpzolomsrUeekTExkmywvaurYmnhhyfkeTyqXwxEriejuAlznxbggtyKtjubNzsratjwextheeydWwywOtchcCrduxwwqqcZedhcciosLRJghdiCygmwlegipkiJskuplqmyRnotmksaqfeb";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 416);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_password_LEN(ph) == 19);
    {
        char16_t * exemplary = u"ztunbtuaMhcznpTjgdb";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 38);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_ssid_LEN(ph) == 26);
    {
        char16_t * exemplary = u"kPmunnjFxnqvpdqzueEljkqmrn";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 52);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)223, (uint8_t)55, (uint8_t)175, (uint8_t)131, (uint8_t)112, (uint8_t)29, (uint8_t)165, (uint8_t)21} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)27937);
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)58834);
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)19682);
    {
        uint8_t exemplary[] =  {(uint8_t)82, (uint8_t)17, (uint8_t)215, (uint8_t)216, (uint8_t)174, (uint8_t)46, (uint8_t)186, (uint8_t)187} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p310_time_usec_GET(pack) == (uint64_t)1229122492311986474L);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL);
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)42975);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR);
    assert(p310_uptime_sec_GET(pack) == (uint32_t)1066611158L);
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)3892334352L);
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)163);
    assert(p311_time_usec_GET(pack) == (uint64_t)5806495681950907237L);
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p311_name_LEN(ph) == 10);
    {
        char16_t * exemplary = u"rqhitxxtRf";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_uptime_sec_GET(pack) == (uint32_t)2322642673L);
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)68);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)135);
    {
        uint8_t exemplary[] =  {(uint8_t)214, (uint8_t)99, (uint8_t)24, (uint8_t)144, (uint8_t)217, (uint8_t)8, (uint8_t)254, (uint8_t)123, (uint8_t)94, (uint8_t)228, (uint8_t)254, (uint8_t)97, (uint8_t)14, (uint8_t)95, (uint8_t)49, (uint8_t)197} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t) -26255);
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)243);
    assert(p320_param_id_LEN(ph) == 11);
    {
        char16_t * exemplary = u"kivpdmXCkom";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 22);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)99);
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)64);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)62059);
    assert(p322_param_id_LEN(ph) == 9);
    {
        char16_t * exemplary = u"mdqtfubpv";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_value_LEN(ph) == 40);
    {
        char16_t * exemplary = u"jxfjhwdOwjhdohDxjreglaXxAcjobmZkallpybcj";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 80);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)40063);
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT64);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_param_id_LEN(ph) == 3);
    {
        char16_t * exemplary = u"wsc";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)120);
    assert(p323_param_value_LEN(ph) == 4);
    {
        char16_t * exemplary = u"zdoh";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8);
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)46);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_id_LEN(ph) == 11);
    {
        char16_t * exemplary = u"TuDQohrVlmc";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 22);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_value_LEN(ph) == 9);
    {
        char16_t * exemplary = u"dditnzLpd";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_FAILED);
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32);
};


void c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)30479);
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)81);
    {
        uint16_t exemplary[] =  {(uint16_t)19084, (uint16_t)20831, (uint16_t)13321, (uint16_t)23501, (uint16_t)27876, (uint16_t)63557, (uint16_t)5902, (uint16_t)64843, (uint16_t)3163, (uint16_t)46570, (uint16_t)21066, (uint16_t)35535, (uint16_t)7921, (uint16_t)28506, (uint16_t)1925, (uint16_t)65092, (uint16_t)1542, (uint16_t)4204, (uint16_t)37576, (uint16_t)60103, (uint16_t)36452, (uint16_t)10448, (uint16_t)15869, (uint16_t)11869, (uint16_t)43007, (uint16_t)12628, (uint16_t)20022, (uint16_t)46690, (uint16_t)39755, (uint16_t)21009, (uint16_t)40977, (uint16_t)29542, (uint16_t)338, (uint16_t)34630, (uint16_t)17236, (uint16_t)8526, (uint16_t)53293, (uint16_t)65226, (uint16_t)1315, (uint16_t)62574, (uint16_t)51596, (uint16_t)63158, (uint16_t)60628, (uint16_t)61508, (uint16_t)10092, (uint16_t)40377, (uint16_t)932, (uint16_t)23084, (uint16_t)4273, (uint16_t)35990, (uint16_t)18731, (uint16_t)48522, (uint16_t)3191, (uint16_t)27772, (uint16_t)27119, (uint16_t)16034, (uint16_t)47129, (uint16_t)46378, (uint16_t)50501, (uint16_t)30548, (uint16_t)11491, (uint16_t)55455, (uint16_t)4135, (uint16_t)4563, (uint16_t)34911, (uint16_t)9885, (uint16_t)45780, (uint16_t)65101, (uint16_t)9117, (uint16_t)50065, (uint16_t)9987, (uint16_t)53710} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_time_usec_GET(pack) == (uint64_t)7745215932703127157L);
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)53560);
};



Pack * c_TEST_Channel_process(Pack * pack, int32_t id)
{
#define rb_size0 (5)
    static RBUF_INIT(Pack*, rb_size0) sendout_packs;
    static RBUF_INIT(Pack*, rb_size0) received_packs;
    static Bounds_Inside ph;
    for(bool LOOP = false;;)
    {
        switch(id) /*220*/
        {
            default:
                assert(0);
                return NULL;
            case PROCESS_CHANNEL_REQEST:
                if(pack == NULL) return (RBUF_ISEMPTY(sendout_packs)) ? NULL : RBUF_GET(sendout_packs);
                if(RBUF_ISFULL(received_packs)) return pack;
                RBUF_PUT(received_packs, pack)
                return NULL;
            case PROCESS_HOST_REQEST:
                if(pack == NULL) return (RBUF_ISEMPTY(received_packs)) ? NULL : RBUF_GET(received_packs);
                if(RBUF_ISFULL(sendout_packs)) return pack;
                RBUF_PUT(sendout_packs, pack)
                return NULL;
            case PROCESS_RECEIVED_PACKS:
                LOOP = true;
                goto next_pack;
        }
        free(pack); // dispose pack
        if(!LOOP) return NULL;
next_pack:
        if(RBUF_ISEMPTY(received_packs)) return NULL;
        pack = RBUF_GET(received_packs);
        id = pack->meta->id;
        setPack(pack, &ph);
    }
}
Channel c_TEST_Channel = {.process = c_TEST_Channel_process};


void c_TEST_Channel_process_received() { c_TEST_Channel_process(NULL, PROCESS_RECEIVED_PACKS); }
void c_TEST_Channel_send(Pack* pack) { c_TEST_Channel_process(pack, PROCESS_HOST_REQEST); }
int main()
{
    static uint8_t buff[512];
    static Bounds_Inside PH;
    {
        setPack(c_LoopBackDemoChannel_new_HEARTBEAT_0(), &PH);
        p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_FREE_BALLOON, PH.base.pack) ;
        p0_custom_mode_SET((uint32_t)382891060L, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_UDB, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_CRITICAL, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYS_STATUS_1(), &PH);
        p1_current_battery_SET((int16_t)(int16_t) -5895, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)478, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)61465, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)38257, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)32967, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)26772, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t)120, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)24952, PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)49041, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_YAW_POSITION, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)47266, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_boot_ms_SET((uint32_t)2000782192L, PH.base.pack) ;
        p2_time_unix_usec_SET((uint64_t)2001467072119379248L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_yaw_rate_SET((float) -4.646478E37F, PH.base.pack) ;
        p3_z_SET((float)2.4108416E38F, PH.base.pack) ;
        p3_y_SET((float) -3.3853051E38F, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        p3_yaw_SET((float) -1.4891242E38F, PH.base.pack) ;
        p3_afz_SET((float) -2.201726E38F, PH.base.pack) ;
        p3_afy_SET((float)1.1247662E38F, PH.base.pack) ;
        p3_vy_SET((float)2.4366265E38F, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)11547, PH.base.pack) ;
        p3_afx_SET((float)6.7673806E37F, PH.base.pack) ;
        p3_vx_SET((float)1.6977839E38F, PH.base.pack) ;
        p3_x_SET((float)3.5905156E37F, PH.base.pack) ;
        p3_vz_SET((float) -2.6772925E38F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)4219490201L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PING_4(), &PH);
        p4_time_usec_SET((uint64_t)4394367281331872007L, PH.base.pack) ;
        p4_target_system_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p4_seq_SET((uint32_t)391320775L, PH.base.pack) ;
        p4_target_component_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        p5_control_request_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
        {
            char16_t* passkey = u"jFbmdfjdep";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_version_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        p5_target_system_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_control_request_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        p6_ack_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"vla";
            p7_key_SET_(key, &PH) ;
        }
        c_LoopBackDemoChannel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_MODE_11(), &PH);
        p11_target_system_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
        p11_custom_mode_SET((uint32_t)4287840112L, PH.base.pack) ;
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_PREFLIGHT, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_READ_20(), &PH);
        p20_target_system_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p20_target_component_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        p20_param_index_SET((int16_t)(int16_t) -26478, PH.base.pack) ;
        {
            char16_t* param_id = u"dWhtgpbugzyq";
            p20_param_id_SET_(param_id, &PH) ;
        }
        c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_system_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        p21_target_component_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_VALUE_22(), &PH);
        {
            char16_t* param_id = u"Uac";
            p22_param_id_SET_(param_id, &PH) ;
        }
        p22_param_value_SET((float)1.8036741E38F, PH.base.pack) ;
        p22_param_count_SET((uint16_t)(uint16_t)47818, PH.base.pack) ;
        p22_param_index_SET((uint16_t)(uint16_t)43310, PH.base.pack) ;
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_SET_23(), &PH);
        p23_param_value_SET((float) -1.0370037E38F, PH.base.pack) ;
        p23_target_component_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL32, PH.base.pack) ;
        p23_target_system_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        {
            char16_t* param_id = u"eneMdtauuJqvc";
            p23_param_id_SET_(param_id, &PH) ;
        }
        c_LoopBackDemoChannel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RAW_INT_24(), &PH);
        p24_h_acc_SET((uint32_t)3854070657L, &PH) ;
        p24_vel_acc_SET((uint32_t)3383337681L, &PH) ;
        p24_epv_SET((uint16_t)(uint16_t)4912, PH.base.pack) ;
        p24_lon_SET((int32_t)629163508, PH.base.pack) ;
        p24_cog_SET((uint16_t)(uint16_t)50669, PH.base.pack) ;
        p24_lat_SET((int32_t) -1432525086, PH.base.pack) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, PH.base.pack) ;
        p24_alt_ellipsoid_SET((int32_t) -1457148080, &PH) ;
        p24_hdg_acc_SET((uint32_t)1035320536L, &PH) ;
        p24_vel_SET((uint16_t)(uint16_t)21355, PH.base.pack) ;
        p24_v_acc_SET((uint32_t)2848072965L, &PH) ;
        p24_eph_SET((uint16_t)(uint16_t)8459, PH.base.pack) ;
        p24_time_usec_SET((uint64_t)8189189502200789322L, PH.base.pack) ;
        p24_alt_SET((int32_t) -1918439573, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_STATUS_25(), &PH);
        p25_satellites_visible_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        {
            uint8_t satellite_prn[] =  {(uint8_t)168, (uint8_t)243, (uint8_t)250, (uint8_t)186, (uint8_t)93, (uint8_t)190, (uint8_t)21, (uint8_t)96, (uint8_t)61, (uint8_t)29, (uint8_t)112, (uint8_t)64, (uint8_t)100, (uint8_t)222, (uint8_t)199, (uint8_t)190, (uint8_t)226, (uint8_t)26, (uint8_t)66, (uint8_t)24};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)58, (uint8_t)212, (uint8_t)174, (uint8_t)44, (uint8_t)215, (uint8_t)198, (uint8_t)254, (uint8_t)177, (uint8_t)179, (uint8_t)72, (uint8_t)20, (uint8_t)162, (uint8_t)9, (uint8_t)238, (uint8_t)77, (uint8_t)167, (uint8_t)50, (uint8_t)103, (uint8_t)134, (uint8_t)232};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_used[] =  {(uint8_t)219, (uint8_t)148, (uint8_t)119, (uint8_t)32, (uint8_t)100, (uint8_t)160, (uint8_t)133, (uint8_t)181, (uint8_t)12, (uint8_t)17, (uint8_t)219, (uint8_t)110, (uint8_t)2, (uint8_t)232, (uint8_t)179, (uint8_t)179, (uint8_t)104, (uint8_t)16, (uint8_t)78, (uint8_t)52};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)97, (uint8_t)12, (uint8_t)71, (uint8_t)165, (uint8_t)238, (uint8_t)133, (uint8_t)84, (uint8_t)140, (uint8_t)183, (uint8_t)176, (uint8_t)182, (uint8_t)79, (uint8_t)25, (uint8_t)34, (uint8_t)137, (uint8_t)238, (uint8_t)143, (uint8_t)249, (uint8_t)49, (uint8_t)140};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_snr[] =  {(uint8_t)17, (uint8_t)114, (uint8_t)102, (uint8_t)216, (uint8_t)216, (uint8_t)83, (uint8_t)147, (uint8_t)49, (uint8_t)159, (uint8_t)106, (uint8_t)175, (uint8_t)77, (uint8_t)161, (uint8_t)106, (uint8_t)219, (uint8_t)75, (uint8_t)114, (uint8_t)208, (uint8_t)235, (uint8_t)253};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU_26(), &PH);
        p26_time_boot_ms_SET((uint32_t)4036912829L, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t)615, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t)9691, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t)2172, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t)32131, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t) -11545, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t) -32235, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t) -8672, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t) -7041, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t)20327, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_IMU_27(), &PH);
        p27_time_usec_SET((uint64_t)7216798221575728225L, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t)31197, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t)25559, PH.base.pack) ;
        p27_zacc_SET((int16_t)(int16_t) -25817, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t)30789, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t) -24088, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t)6515, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t)3928, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t)16022, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t)25495, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_PRESSURE_28(), &PH);
        p28_press_diff1_SET((int16_t)(int16_t) -8857, PH.base.pack) ;
        p28_press_diff2_SET((int16_t)(int16_t) -32212, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t)32601, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t) -26180, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)243619827054177068L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_press_abs_SET((float)1.3179264E37F, PH.base.pack) ;
        p29_temperature_SET((int16_t)(int16_t)2046, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)3033124254L, PH.base.pack) ;
        p29_press_diff_SET((float)2.9726248E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_30(), &PH);
        p30_yaw_SET((float) -1.1631537E37F, PH.base.pack) ;
        p30_yawspeed_SET((float)1.0771738E38F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)3139756574L, PH.base.pack) ;
        p30_pitch_SET((float)1.1084469E38F, PH.base.pack) ;
        p30_roll_SET((float) -6.6726704E37F, PH.base.pack) ;
        p30_rollspeed_SET((float) -1.4567409E38F, PH.base.pack) ;
        p30_pitchspeed_SET((float) -9.2073165E36F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_rollspeed_SET((float)8.948354E37F, PH.base.pack) ;
        p31_pitchspeed_SET((float)2.6883168E38F, PH.base.pack) ;
        p31_q4_SET((float) -5.658694E37F, PH.base.pack) ;
        p31_q3_SET((float) -4.365963E37F, PH.base.pack) ;
        p31_q1_SET((float)1.2481478E38F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)4170885047L, PH.base.pack) ;
        p31_yawspeed_SET((float) -1.5073941E37F, PH.base.pack) ;
        p31_q2_SET((float) -1.8149124E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_vy_SET((float) -8.3731015E37F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)1446325996L, PH.base.pack) ;
        p32_x_SET((float) -1.9297512E38F, PH.base.pack) ;
        p32_vx_SET((float) -2.1408738E38F, PH.base.pack) ;
        p32_y_SET((float) -2.4837753E37F, PH.base.pack) ;
        p32_vz_SET((float)1.7343537E38F, PH.base.pack) ;
        p32_z_SET((float) -8.862546E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_time_boot_ms_SET((uint32_t)1213190351L, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)481, PH.base.pack) ;
        p33_lat_SET((int32_t) -616424532, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t)29727, PH.base.pack) ;
        p33_lon_SET((int32_t) -125045043, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t)8782, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t) -25042, PH.base.pack) ;
        p33_alt_SET((int32_t)2060600999, PH.base.pack) ;
        p33_relative_alt_SET((int32_t) -810805129, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan3_scaled_SET((int16_t)(int16_t)6516, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)94824788L, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t)29843, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t) -17343, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t)28711, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t)15877, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t)26924, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t)14041, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t) -3430, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_chan5_raw_SET((uint16_t)(uint16_t)22532, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)39008, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)4079824296L, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)32682, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)56467, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)59059, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)715, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)35628, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)38835, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo3_raw_SET((uint16_t)(uint16_t)63239, PH.base.pack) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)29953, PH.base.pack) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)46912, &PH) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)19340, &PH) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)23622, &PH) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)53147, PH.base.pack) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)36544, PH.base.pack) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)21468, PH.base.pack) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)51195, &PH) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)30050, &PH) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)49701, &PH) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)3734, PH.base.pack) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)8367, PH.base.pack) ;
        p36_time_usec_SET((uint32_t)1046268007L, PH.base.pack) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)28257, PH.base.pack) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)63443, &PH) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)63659, &PH) ;
        p36_port_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p37_start_index_SET((int16_t)(int16_t)28078, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t)28941, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_target_system_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p38_end_index_SET((int16_t)(int16_t) -14403, PH.base.pack) ;
        p38_target_component_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t)25357, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_39(), &PH);
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p39_z_SET((float) -1.9231929E38F, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p39_param1_SET((float)3.809215E37F, PH.base.pack) ;
        p39_y_SET((float) -1.5484021E38F, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)26695, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        p39_x_SET((float) -2.4108134E38F, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_USER_3, PH.base.pack) ;
        p39_param2_SET((float) -1.6415855E38F, PH.base.pack) ;
        p39_param4_SET((float) -1.2472369E38F, PH.base.pack) ;
        p39_param3_SET((float)2.624958E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_40(), &PH);
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        p40_seq_SET((uint16_t)(uint16_t)14774, PH.base.pack) ;
        p40_target_component_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_target_component_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        p41_target_system_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        p41_seq_SET((uint16_t)(uint16_t)25023, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)51729, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        p43_target_system_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_COUNT_44(), &PH);
        p44_target_component_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p44_count_SET((uint16_t)(uint16_t)62855, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_target_system_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        p45_target_component_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)3348, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ACK_47(), &PH);
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_PARAM6_Y, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_longitude_SET((int32_t)1065410464, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)7111868083256045978L, &PH) ;
        p48_target_system_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p48_altitude_SET((int32_t) -1661531009, PH.base.pack) ;
        p48_latitude_SET((int32_t) -1757679436, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_time_usec_SET((uint64_t)1496616033139386054L, &PH) ;
        p49_altitude_SET((int32_t) -439213338, PH.base.pack) ;
        p49_longitude_SET((int32_t)2125369841, PH.base.pack) ;
        p49_latitude_SET((int32_t) -897162437, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_param_value0_SET((float)9.546097E37F, PH.base.pack) ;
        p50_param_value_max_SET((float)2.9222707E38F, PH.base.pack) ;
        p50_target_component_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        {
            char16_t* param_id = u"pldhosxbXfjWheq";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_param_value_min_SET((float) -3.9886535E37F, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t)28572, PH.base.pack) ;
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        p50_scale_SET((float) -6.224224E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
        p51_target_component_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)23153, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_target_system_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
        p54_p1x_SET((float) -9.447388E37F, PH.base.pack) ;
        p54_p1z_SET((float)1.0655978E38F, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        p54_p2y_SET((float) -1.979168E38F, PH.base.pack) ;
        p54_p2z_SET((float)1.3778072E38F, PH.base.pack) ;
        p54_p1y_SET((float)2.3931024E38F, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        p54_p2x_SET((float)2.4337247E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p2z_SET((float)2.7835837E38F, PH.base.pack) ;
        p55_p1y_SET((float)2.323834E38F, PH.base.pack) ;
        p55_p2x_SET((float) -1.961659E38F, PH.base.pack) ;
        p55_p2y_SET((float) -4.100305E37F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p55_p1x_SET((float) -2.100223E38F, PH.base.pack) ;
        p55_p1z_SET((float)2.1455272E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_time_usec_SET((uint64_t)1395623904185529744L, PH.base.pack) ;
        p61_pitchspeed_SET((float) -2.4582138E38F, PH.base.pack) ;
        p61_yawspeed_SET((float)1.7629446E38F, PH.base.pack) ;
        {
            float q[] =  {1.7980267E38F, 2.7932182E38F, 9.812939E36F, 1.8663496E38F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        p61_rollspeed_SET((float)1.7116438E37F, PH.base.pack) ;
        {
            float covariance[] =  {-1.4502536E38F, -1.570404E38F, -1.5137144E38F, 2.3692E38F, 1.1128518E38F, -2.5696424E38F, -3.14519E38F, -4.7458192E36F, 3.3965801E38F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_alt_error_SET((float)2.5611362E37F, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t) -6768, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t) -1784, PH.base.pack) ;
        p62_nav_roll_SET((float) -6.366582E37F, PH.base.pack) ;
        p62_aspd_error_SET((float)2.8150358E38F, PH.base.pack) ;
        p62_nav_pitch_SET((float) -7.0518815E37F, PH.base.pack) ;
        p62_xtrack_error_SET((float)2.915725E38F, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)37118, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
        p63_vz_SET((float) -1.7804214E38F, PH.base.pack) ;
        p63_vx_SET((float)2.5237445E38F, PH.base.pack) ;
        p63_alt_SET((int32_t)842713801, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)4508598773115017152L, PH.base.pack) ;
        p63_lon_SET((int32_t)2096167018, PH.base.pack) ;
        p63_lat_SET((int32_t) -525078506, PH.base.pack) ;
        p63_relative_alt_SET((int32_t) -638264417, PH.base.pack) ;
        {
            float covariance[] =  {-5.4249437E37F, 2.9416879E38F, -4.4168415E37F, 1.3104747E38F, 9.382291E37F, 1.5998572E38F, 3.3507632E38F, -2.292925E38F, -1.6868603E38F, 2.7172515E38F, 8.4289075E37F, -2.9789647E38F, -2.2398934E38F, 3.39837E38F, -2.2064065E38F, -1.289841E38F, 2.874709E38F, 3.0594506E37F, -5.495724E36F, -2.1821603E38F, -1.2167034E38F, -2.8934813E38F, 3.0227982E38F, 2.3812163E38F, -5.312995E36F, 2.7701707E38F, 7.2185917E37F, -5.686211E37F, 9.375075E37F, 3.0961934E38F, 1.2841155E38F, -1.2661921E38F, -3.1126083E38F, -4.835894E36F, 5.3859016E37F, -2.2911245E38F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_vy_SET((float)6.8101293E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_vy_SET((float) -1.3042387E37F, PH.base.pack) ;
        {
            float covariance[] =  {1.5765722E38F, -3.2556983E38F, -2.11587E38F, 1.3596571E38F, -3.3975196E38F, 1.4609596E38F, 1.9316523E38F, 4.0557114E37F, -1.672904E38F, 3.2760247E38F, 5.397551E37F, 2.2322449E38F, 3.1968988E38F, 8.638095E37F, -2.5987567E38F, -1.8142236E38F, -1.7727702E38F, -8.3979226E37F, 8.53531E37F, 2.8176366E38F, 1.982721E38F, -3.8904625E37F, -1.7831976E38F, -1.2906652E36F, -5.656256E37F, 2.0214066E38F, 2.9967734E37F, -1.3097519E38F, 2.8692083E38F, 3.3332102E37F, 1.4555263E38F, 3.0342183E38F, 2.9484949E38F, 2.171893E38F, 2.6226425E38F, 2.4110104E38F, -2.1600574E38F, -1.5622081E38F, -1.2029982E38F, -1.4067521E38F, 4.1617862E37F, -1.457584E38F, 2.5611339E38F, -1.8860497E38F, -2.1995616E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_ay_SET((float) -4.350729E37F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_NAIVE, PH.base.pack) ;
        p64_x_SET((float)3.2786862E38F, PH.base.pack) ;
        p64_z_SET((float) -1.4282688E38F, PH.base.pack) ;
        p64_ax_SET((float) -2.2648166E38F, PH.base.pack) ;
        p64_y_SET((float)1.5247751E37F, PH.base.pack) ;
        p64_vz_SET((float)2.968266E38F, PH.base.pack) ;
        p64_time_usec_SET((uint64_t)729078912293458530L, PH.base.pack) ;
        p64_vx_SET((float) -1.7773027E38F, PH.base.pack) ;
        p64_az_SET((float)1.0936082E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan1_raw_SET((uint16_t)(uint16_t)6245, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)57560, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)54888, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)47307, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)62487, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)27043, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)26822, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)18367, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)28653, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)7121, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)49352, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)29832, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)12462, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)45933, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)30897, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)1212031166L, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)47563, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)53473, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)44647, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_req_message_rate_SET((uint16_t)(uint16_t)12839, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_STREAM_67(), &PH);
        p67_stream_id_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p67_message_rate_SET((uint16_t)(uint16_t)49572, PH.base.pack) ;
        p67_on_off_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_x_SET((int16_t)(int16_t) -337, PH.base.pack) ;
        p69_z_SET((int16_t)(int16_t)10460, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t) -16442, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t)8985, PH.base.pack) ;
        p69_buttons_SET((uint16_t)(uint16_t)40131, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan1_raw_SET((uint16_t)(uint16_t)20207, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)57586, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)33757, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)47592, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)28780, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)45353, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)57302, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)23214, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_param4_SET((float) -1.9195625E38F, PH.base.pack) ;
        p73_param3_SET((float) -5.729489E37F, PH.base.pack) ;
        p73_z_SET((float)2.7857813E38F, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_WAYPOINT_USER_2, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)2310, PH.base.pack) ;
        p73_param1_SET((float) -1.7446746E38F, PH.base.pack) ;
        p73_x_SET((int32_t) -1530355114, PH.base.pack) ;
        p73_y_SET((int32_t) -988671962, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        p73_param2_SET((float) -2.6976566E38F, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VFR_HUD_74(), &PH);
        p74_alt_SET((float) -2.7094224E38F, PH.base.pack) ;
        p74_groundspeed_SET((float)1.3487866E38F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t) -6582, PH.base.pack) ;
        p74_airspeed_SET((float) -1.6297735E38F, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)47373, PH.base.pack) ;
        p74_climb_SET((float) -7.4897493E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_INT_75(), &PH);
        p75_target_component_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        p75_param3_SET((float)3.8557576E37F, PH.base.pack) ;
        p75_z_SET((float) -1.020097E38F, PH.base.pack) ;
        p75_param1_SET((float) -7.370007E37F, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_DO_MOUNT_CONFIGURE, PH.base.pack) ;
        p75_param4_SET((float) -2.6409746E38F, PH.base.pack) ;
        p75_x_SET((int32_t)1481735238, PH.base.pack) ;
        p75_y_SET((int32_t) -172980655, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        p75_param2_SET((float) -8.548695E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_LONG_76(), &PH);
        p76_param4_SET((float)2.356487E38F, PH.base.pack) ;
        p76_param7_SET((float)4.347604E37F, PH.base.pack) ;
        p76_param1_SET((float) -1.5383248E38F, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        p76_param5_SET((float)3.0634476E38F, PH.base.pack) ;
        p76_param3_SET((float)1.598852E38F, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_NAV_FENCE_RETURN_POINT, PH.base.pack) ;
        p76_param6_SET((float) -5.4729065E37F, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        p76_param2_SET((float)3.3006835E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_ACK_77(), &PH);
        p77_command_SET(e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER, PH.base.pack) ;
        p77_target_system_SET((uint8_t)(uint8_t)32, &PH) ;
        p77_progress_SET((uint8_t)(uint8_t)59, &PH) ;
        p77_target_component_SET((uint8_t)(uint8_t)146, &PH) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_UNSUPPORTED, PH.base.pack) ;
        p77_result_param2_SET((int32_t)159139809, &PH) ;
        c_LoopBackDemoChannel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_roll_SET((float) -2.16264E38F, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        p81_thrust_SET((float)1.6839498E38F, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)2762761990L, PH.base.pack) ;
        p81_pitch_SET((float)2.812498E38F, PH.base.pack) ;
        p81_manual_override_switch_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        p81_yaw_SET((float) -1.0463766E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_body_roll_rate_SET((float) -2.5303894E38F, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
        p82_body_pitch_rate_SET((float) -3.0050773E37F, PH.base.pack) ;
        {
            float q[] =  {2.7532846E38F, 2.7016567E38F, -8.953648E37F, 5.068591E37F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_time_boot_ms_SET((uint32_t)2625049059L, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        p82_thrust_SET((float) -5.3324103E37F, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        p82_body_yaw_rate_SET((float) -3.388621E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_TARGET_83(), &PH);
        {
            float q[] =  {-1.1101187E38F, 2.3889826E36F, -6.286642E37F, 1.0802368E38F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_time_boot_ms_SET((uint32_t)924478959L, PH.base.pack) ;
        p83_body_roll_rate_SET((float) -2.29858E38F, PH.base.pack) ;
        p83_type_mask_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p83_body_pitch_rate_SET((float)6.9882855E37F, PH.base.pack) ;
        p83_body_yaw_rate_SET((float)2.1996612E38F, PH.base.pack) ;
        p83_thrust_SET((float)3.1946614E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_y_SET((float)9.661816E37F, PH.base.pack) ;
        p84_yaw_SET((float)2.7142034E38F, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)15538, PH.base.pack) ;
        p84_vx_SET((float) -3.07721E38F, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
        p84_vz_SET((float)2.3674448E38F, PH.base.pack) ;
        p84_afz_SET((float)1.1613883E38F, PH.base.pack) ;
        p84_vy_SET((float) -2.9362713E38F, PH.base.pack) ;
        p84_z_SET((float)1.643459E38F, PH.base.pack) ;
        p84_afy_SET((float) -8.5038E37F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)4224975782L, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
        p84_yaw_rate_SET((float) -1.9125884E38F, PH.base.pack) ;
        p84_x_SET((float)1.6778579E38F, PH.base.pack) ;
        p84_afx_SET((float) -1.9017676E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_lon_int_SET((int32_t) -1000026484, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)2916312152L, PH.base.pack) ;
        p86_vx_SET((float) -2.803274E38F, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
        p86_vy_SET((float)1.7814503E38F, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        p86_lat_int_SET((int32_t)754665512, PH.base.pack) ;
        p86_vz_SET((float)2.7713643E38F, PH.base.pack) ;
        p86_afx_SET((float)2.321499E38F, PH.base.pack) ;
        p86_yaw_rate_SET((float) -1.0091898E38F, PH.base.pack) ;
        p86_alt_SET((float)2.607765E37F, PH.base.pack) ;
        p86_afy_SET((float) -3.0702422E38F, PH.base.pack) ;
        p86_afz_SET((float) -2.8523834E38F, PH.base.pack) ;
        p86_yaw_SET((float)2.9290568E38F, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)8621, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_vy_SET((float) -4.259472E37F, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)117344157L, PH.base.pack) ;
        p87_lat_int_SET((int32_t) -1197576590, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)23057, PH.base.pack) ;
        p87_lon_int_SET((int32_t)1822498290, PH.base.pack) ;
        p87_alt_SET((float)5.5447985E37F, PH.base.pack) ;
        p87_afy_SET((float) -2.0958761E38F, PH.base.pack) ;
        p87_vx_SET((float) -7.57192E37F, PH.base.pack) ;
        p87_yaw_rate_SET((float)1.094423E38F, PH.base.pack) ;
        p87_afx_SET((float) -2.1303402E38F, PH.base.pack) ;
        p87_vz_SET((float)2.8224585E38F, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p87_afz_SET((float)1.3092844E38F, PH.base.pack) ;
        p87_yaw_SET((float) -1.9163721E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_yaw_SET((float)3.575522E37F, PH.base.pack) ;
        p89_y_SET((float) -1.9642388E38F, PH.base.pack) ;
        p89_z_SET((float) -2.7399651E38F, PH.base.pack) ;
        p89_roll_SET((float)2.0342634E38F, PH.base.pack) ;
        p89_pitch_SET((float) -2.483864E37F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)1502930190L, PH.base.pack) ;
        p89_x_SET((float)5.3473493E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_90(), &PH);
        p90_lon_SET((int32_t) -419254813, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t) -20497, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t) -6756, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t) -12838, PH.base.pack) ;
        p90_rollspeed_SET((float)1.4077079E38F, PH.base.pack) ;
        p90_lat_SET((int32_t) -1179385857, PH.base.pack) ;
        p90_roll_SET((float) -1.3157888E38F, PH.base.pack) ;
        p90_yawspeed_SET((float)5.69241E36F, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t) -4455, PH.base.pack) ;
        p90_pitch_SET((float)3.3234233E38F, PH.base.pack) ;
        p90_yaw_SET((float) -3.2758941E38F, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t)602, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t) -1071, PH.base.pack) ;
        p90_alt_SET((int32_t)136007623, PH.base.pack) ;
        p90_pitchspeed_SET((float)1.9569645E38F, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)6351120631471618254L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_CONTROLS_91(), &PH);
        p91_yaw_rudder_SET((float) -1.1297777E38F, PH.base.pack) ;
        p91_aux3_SET((float) -2.9371225E38F, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)6186905204282790330L, PH.base.pack) ;
        p91_aux1_SET((float)1.2093928E38F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        p91_roll_ailerons_SET((float) -2.7024368E38F, PH.base.pack) ;
        p91_aux4_SET((float) -3.1073276E38F, PH.base.pack) ;
        p91_pitch_elevator_SET((float)2.7158877E38F, PH.base.pack) ;
        p91_throttle_SET((float) -2.8360528E38F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_ARMED, PH.base.pack) ;
        p91_aux2_SET((float)3.1192938E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan6_raw_SET((uint16_t)(uint16_t)63516, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)38908, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)11615, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)59752, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)5008, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)53119, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)3697, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)7004, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)22238, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)2909482447472706510L, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)38341, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)54979, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)36155, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_flags_SET((uint64_t)2447111430720415967L, PH.base.pack) ;
        p93_time_usec_SET((uint64_t)1474428559822065775L, PH.base.pack) ;
        p93_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_ARMED, PH.base.pack) ;
        {
            float controls[] =  {-4.559752E37F, -4.2790804E37F, 5.0237475E37F, 2.792074E38F, 2.039452E38F, 2.5347511E38F, 8.3849307E37F, -1.1949787E38F, -2.6480702E38F, -7.4492316E37F, -2.2160818E38F, -1.3950712E36F, 1.0835192E38F, 2.307989E38F, 5.011607E37F, 1.6929832E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_ground_distance_SET((float)2.6935705E38F, PH.base.pack) ;
        p100_quality_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        p100_flow_rate_y_SET((float)9.293658E37F, &PH) ;
        p100_flow_comp_m_y_SET((float) -5.320796E36F, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float) -1.6088992E38F, PH.base.pack) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)1113412688679105861L, PH.base.pack) ;
        p100_flow_rate_x_SET((float)1.275639E38F, &PH) ;
        p100_flow_y_SET((int16_t)(int16_t)24614, PH.base.pack) ;
        p100_flow_x_SET((int16_t)(int16_t) -14870, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_y_SET((float) -2.7758275E38F, PH.base.pack) ;
        p101_usec_SET((uint64_t)7413588249836671176L, PH.base.pack) ;
        p101_z_SET((float) -2.532482E38F, PH.base.pack) ;
        p101_x_SET((float)1.8389213E38F, PH.base.pack) ;
        p101_yaw_SET((float)2.7849545E38F, PH.base.pack) ;
        p101_roll_SET((float) -2.6079942E38F, PH.base.pack) ;
        p101_pitch_SET((float) -5.9725627E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_z_SET((float)1.8050556E38F, PH.base.pack) ;
        p102_yaw_SET((float)2.8376892E38F, PH.base.pack) ;
        p102_x_SET((float) -2.8131179E38F, PH.base.pack) ;
        p102_roll_SET((float)1.2498342E38F, PH.base.pack) ;
        p102_pitch_SET((float) -3.1738886E38F, PH.base.pack) ;
        p102_y_SET((float)1.489833E38F, PH.base.pack) ;
        p102_usec_SET((uint64_t)2187933040608949220L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_usec_SET((uint64_t)6237931433859141154L, PH.base.pack) ;
        p103_x_SET((float) -8.97572E37F, PH.base.pack) ;
        p103_z_SET((float)1.6438094E37F, PH.base.pack) ;
        p103_y_SET((float) -2.4580869E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_usec_SET((uint64_t)9157310829916126523L, PH.base.pack) ;
        p104_pitch_SET((float) -2.981448E38F, PH.base.pack) ;
        p104_x_SET((float)2.1471652E38F, PH.base.pack) ;
        p104_z_SET((float) -1.8825437E38F, PH.base.pack) ;
        p104_y_SET((float) -2.51371E38F, PH.base.pack) ;
        p104_yaw_SET((float)2.8584724E38F, PH.base.pack) ;
        p104_roll_SET((float)9.129925E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGHRES_IMU_105(), &PH);
        p105_ymag_SET((float)3.9218417E37F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)5847931941142543067L, PH.base.pack) ;
        p105_pressure_alt_SET((float) -2.57348E38F, PH.base.pack) ;
        p105_zgyro_SET((float) -2.8902166E38F, PH.base.pack) ;
        p105_yacc_SET((float)2.568159E38F, PH.base.pack) ;
        p105_diff_pressure_SET((float)2.8557158E38F, PH.base.pack) ;
        p105_xmag_SET((float)3.30223E38F, PH.base.pack) ;
        p105_zmag_SET((float) -8.770419E37F, PH.base.pack) ;
        p105_abs_pressure_SET((float) -4.440598E37F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)65219, PH.base.pack) ;
        p105_temperature_SET((float) -8.266431E37F, PH.base.pack) ;
        p105_xgyro_SET((float)2.494728E38F, PH.base.pack) ;
        p105_ygyro_SET((float)2.3003891E38F, PH.base.pack) ;
        p105_xacc_SET((float)1.9197362E38F, PH.base.pack) ;
        p105_zacc_SET((float)1.2941294E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_integrated_zgyro_SET((float) -1.0291345E38F, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)6130859101733734007L, PH.base.pack) ;
        p106_integrated_x_SET((float) -2.5634656E38F, PH.base.pack) ;
        p106_integrated_ygyro_SET((float) -1.9078628E38F, PH.base.pack) ;
        p106_integrated_xgyro_SET((float)2.1110392E37F, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)3886026346L, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
        p106_distance_SET((float) -2.892295E37F, PH.base.pack) ;
        p106_integrated_y_SET((float) -5.182737E37F, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t) -16717, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)3932332572L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_SENSOR_107(), &PH);
        p107_fields_updated_SET((uint32_t)1620466569L, PH.base.pack) ;
        p107_xgyro_SET((float)2.731438E38F, PH.base.pack) ;
        p107_ygyro_SET((float)2.3495407E38F, PH.base.pack) ;
        p107_xmag_SET((float) -8.0888724E37F, PH.base.pack) ;
        p107_xacc_SET((float)1.9213829E38F, PH.base.pack) ;
        p107_zacc_SET((float) -5.4318336E37F, PH.base.pack) ;
        p107_abs_pressure_SET((float)3.0610595E37F, PH.base.pack) ;
        p107_diff_pressure_SET((float) -3.3856493E38F, PH.base.pack) ;
        p107_yacc_SET((float) -1.1382674E38F, PH.base.pack) ;
        p107_temperature_SET((float)4.2096836E37F, PH.base.pack) ;
        p107_ymag_SET((float)1.779252E37F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)3601497273475327337L, PH.base.pack) ;
        p107_pressure_alt_SET((float)2.425569E38F, PH.base.pack) ;
        p107_zgyro_SET((float) -1.5603505E38F, PH.base.pack) ;
        p107_zmag_SET((float)1.536344E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SIM_STATE_108(), &PH);
        p108_q4_SET((float)2.1514824E38F, PH.base.pack) ;
        p108_xgyro_SET((float)2.8250987E38F, PH.base.pack) ;
        p108_xacc_SET((float) -2.4084877E38F, PH.base.pack) ;
        p108_q2_SET((float)2.6786946E38F, PH.base.pack) ;
        p108_lat_SET((float) -2.1080216E37F, PH.base.pack) ;
        p108_std_dev_horz_SET((float)1.1476512E38F, PH.base.pack) ;
        p108_yaw_SET((float) -8.3425323E37F, PH.base.pack) ;
        p108_zacc_SET((float)1.774281E38F, PH.base.pack) ;
        p108_ve_SET((float)2.9338777E38F, PH.base.pack) ;
        p108_q3_SET((float)2.9039204E38F, PH.base.pack) ;
        p108_roll_SET((float)1.6393231E38F, PH.base.pack) ;
        p108_zgyro_SET((float)3.1192496E38F, PH.base.pack) ;
        p108_yacc_SET((float)7.763163E37F, PH.base.pack) ;
        p108_alt_SET((float)1.5932562E38F, PH.base.pack) ;
        p108_std_dev_vert_SET((float)1.0083831E38F, PH.base.pack) ;
        p108_pitch_SET((float)2.8383871E38F, PH.base.pack) ;
        p108_ygyro_SET((float)8.4674943E37F, PH.base.pack) ;
        p108_lon_SET((float) -2.4611098E38F, PH.base.pack) ;
        p108_q1_SET((float) -1.3125554E38F, PH.base.pack) ;
        p108_vd_SET((float)2.0174749E38F, PH.base.pack) ;
        p108_vn_SET((float) -1.9720254E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RADIO_STATUS_109(), &PH);
        p109_noise_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)7072, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)46103, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        p110_target_network_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)95, (uint8_t)52, (uint8_t)55, (uint8_t)0, (uint8_t)96, (uint8_t)132, (uint8_t)51, (uint8_t)145, (uint8_t)169, (uint8_t)150, (uint8_t)109, (uint8_t)212, (uint8_t)157, (uint8_t)85, (uint8_t)171, (uint8_t)98, (uint8_t)94, (uint8_t)98, (uint8_t)21, (uint8_t)16, (uint8_t)114, (uint8_t)135, (uint8_t)11, (uint8_t)34, (uint8_t)193, (uint8_t)210, (uint8_t)107, (uint8_t)198, (uint8_t)40, (uint8_t)100, (uint8_t)89, (uint8_t)198, (uint8_t)172, (uint8_t)60, (uint8_t)147, (uint8_t)96, (uint8_t)159, (uint8_t)178, (uint8_t)164, (uint8_t)211, (uint8_t)189, (uint8_t)234, (uint8_t)72, (uint8_t)6, (uint8_t)223, (uint8_t)162, (uint8_t)57, (uint8_t)129, (uint8_t)53, (uint8_t)120, (uint8_t)216, (uint8_t)103, (uint8_t)109, (uint8_t)69, (uint8_t)185, (uint8_t)85, (uint8_t)178, (uint8_t)5, (uint8_t)161, (uint8_t)67, (uint8_t)160, (uint8_t)103, (uint8_t)227, (uint8_t)189, (uint8_t)86, (uint8_t)69, (uint8_t)6, (uint8_t)187, (uint8_t)95, (uint8_t)74, (uint8_t)157, (uint8_t)172, (uint8_t)181, (uint8_t)98, (uint8_t)209, (uint8_t)131, (uint8_t)172, (uint8_t)168, (uint8_t)65, (uint8_t)216, (uint8_t)236, (uint8_t)142, (uint8_t)170, (uint8_t)133, (uint8_t)97, (uint8_t)193, (uint8_t)132, (uint8_t)49, (uint8_t)198, (uint8_t)239, (uint8_t)121, (uint8_t)67, (uint8_t)142, (uint8_t)122, (uint8_t)11, (uint8_t)74, (uint8_t)107, (uint8_t)229, (uint8_t)150, (uint8_t)129, (uint8_t)176, (uint8_t)99, (uint8_t)115, (uint8_t)60, (uint8_t)170, (uint8_t)122, (uint8_t)7, (uint8_t)149, (uint8_t)49, (uint8_t)30, (uint8_t)123, (uint8_t)139, (uint8_t)188, (uint8_t)67, (uint8_t)51, (uint8_t)236, (uint8_t)114, (uint8_t)230, (uint8_t)235, (uint8_t)9, (uint8_t)79, (uint8_t)116, (uint8_t)119, (uint8_t)90, (uint8_t)53, (uint8_t)87, (uint8_t)153, (uint8_t)102, (uint8_t)30, (uint8_t)104, (uint8_t)50, (uint8_t)145, (uint8_t)221, (uint8_t)129, (uint8_t)65, (uint8_t)219, (uint8_t)218, (uint8_t)152, (uint8_t)158, (uint8_t)228, (uint8_t)240, (uint8_t)135, (uint8_t)28, (uint8_t)150, (uint8_t)96, (uint8_t)184, (uint8_t)203, (uint8_t)184, (uint8_t)242, (uint8_t)161, (uint8_t)9, (uint8_t)255, (uint8_t)98, (uint8_t)193, (uint8_t)50, (uint8_t)164, (uint8_t)61, (uint8_t)222, (uint8_t)245, (uint8_t)156, (uint8_t)185, (uint8_t)21, (uint8_t)184, (uint8_t)13, (uint8_t)193, (uint8_t)63, (uint8_t)116, (uint8_t)19, (uint8_t)240, (uint8_t)171, (uint8_t)76, (uint8_t)137, (uint8_t)227, (uint8_t)97, (uint8_t)23, (uint8_t)27, (uint8_t)213, (uint8_t)71, (uint8_t)0, (uint8_t)162, (uint8_t)146, (uint8_t)193, (uint8_t)238, (uint8_t)160, (uint8_t)55, (uint8_t)157, (uint8_t)250, (uint8_t)57, (uint8_t)167, (uint8_t)10, (uint8_t)201, (uint8_t)182, (uint8_t)109, (uint8_t)41, (uint8_t)118, (uint8_t)24, (uint8_t)174, (uint8_t)160, (uint8_t)116, (uint8_t)110, (uint8_t)152, (uint8_t)174, (uint8_t)229, (uint8_t)205, (uint8_t)117, (uint8_t)140, (uint8_t)143, (uint8_t)130, (uint8_t)171, (uint8_t)88, (uint8_t)112, (uint8_t)56, (uint8_t)2, (uint8_t)79, (uint8_t)131, (uint8_t)112, (uint8_t)83, (uint8_t)105, (uint8_t)86, (uint8_t)245, (uint8_t)91, (uint8_t)12, (uint8_t)46, (uint8_t)80, (uint8_t)110, (uint8_t)139, (uint8_t)25, (uint8_t)33, (uint8_t)12, (uint8_t)215, (uint8_t)168, (uint8_t)89, (uint8_t)126, (uint8_t)30, (uint8_t)148, (uint8_t)29, (uint8_t)16, (uint8_t)122, (uint8_t)61, (uint8_t)131, (uint8_t)46, (uint8_t)141, (uint8_t)116, (uint8_t)169, (uint8_t)13, (uint8_t)211, (uint8_t)233, (uint8_t)126, (uint8_t)175, (uint8_t)29, (uint8_t)187};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_component_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        p110_target_system_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TIMESYNC_111(), &PH);
        p111_ts1_SET((int64_t) -1256720058239829039L, PH.base.pack) ;
        p111_tc1_SET((int64_t)7001358374000970968L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_seq_SET((uint32_t)2003097899L, PH.base.pack) ;
        p112_time_usec_SET((uint64_t)6406682260498060426L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_GPS_113(), &PH);
        p113_eph_SET((uint16_t)(uint16_t)9959, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)48358, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)41276, PH.base.pack) ;
        p113_lat_SET((int32_t)1505859352, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)584389270026088170L, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t) -31151, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t) -20472, PH.base.pack) ;
        p113_lon_SET((int32_t) -1277557796, PH.base.pack) ;
        p113_alt_SET((int32_t)1541183800, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)28210, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t)19323, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_sensor_id_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        p114_integrated_ygyro_SET((float) -2.4196894E38F, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t)10378, PH.base.pack) ;
        p114_integrated_x_SET((float)2.5084702E38F, PH.base.pack) ;
        p114_distance_SET((float) -2.9111874E38F, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        p114_integrated_zgyro_SET((float)1.359779E38F, PH.base.pack) ;
        p114_integrated_y_SET((float)2.7481069E38F, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)4906432917925905023L, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)709654250L, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)2150439752L, PH.base.pack) ;
        p114_integrated_xgyro_SET((float) -2.0259229E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_alt_SET((int32_t) -1794345036, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t) -27479, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t) -26566, PH.base.pack) ;
        p115_rollspeed_SET((float)1.5065548E38F, PH.base.pack) ;
        p115_lat_SET((int32_t) -1534223920, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)38711, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t) -9988, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t)16534, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t) -3225, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)54508, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t) -25511, PH.base.pack) ;
        p115_pitchspeed_SET((float) -3.1169291E38F, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {-3.3104975E38F, -1.7653187E38F, -1.0558286E38F, 2.9584223E38F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_lon_SET((int32_t) -59761761, PH.base.pack) ;
        p115_yawspeed_SET((float) -2.8317325E38F, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)7747292613500140737L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU2_116(), &PH);
        p116_zacc_SET((int16_t)(int16_t)14913, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t)17994, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t)29247, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t)15253, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t) -344, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t)7890, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t) -11034, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)2376143842L, PH.base.pack) ;
        p116_ymag_SET((int16_t)(int16_t) -29470, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t)6605, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_end_SET((uint16_t)(uint16_t)23373, PH.base.pack) ;
        p117_target_component_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
        p117_start_SET((uint16_t)(uint16_t)13430, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ENTRY_118(), &PH);
        p118_last_log_num_SET((uint16_t)(uint16_t)22779, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)41625, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)2131655307L, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)51263, PH.base.pack) ;
        p118_size_SET((uint32_t)3575712693L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_count_SET((uint32_t)3004841004L, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p119_ofs_SET((uint32_t)1273570673L, PH.base.pack) ;
        p119_id_SET((uint16_t)(uint16_t)32334, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_DATA_120(), &PH);
        p120_count_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)169, (uint8_t)238, (uint8_t)99, (uint8_t)135, (uint8_t)35, (uint8_t)5, (uint8_t)49, (uint8_t)180, (uint8_t)173, (uint8_t)248, (uint8_t)108, (uint8_t)190, (uint8_t)62, (uint8_t)141, (uint8_t)172, (uint8_t)36, (uint8_t)12, (uint8_t)107, (uint8_t)51, (uint8_t)76, (uint8_t)17, (uint8_t)68, (uint8_t)144, (uint8_t)157, (uint8_t)104, (uint8_t)226, (uint8_t)104, (uint8_t)187, (uint8_t)225, (uint8_t)98, (uint8_t)9, (uint8_t)203, (uint8_t)90, (uint8_t)163, (uint8_t)9, (uint8_t)163, (uint8_t)178, (uint8_t)153, (uint8_t)219, (uint8_t)81, (uint8_t)249, (uint8_t)223, (uint8_t)68, (uint8_t)231, (uint8_t)71, (uint8_t)222, (uint8_t)15, (uint8_t)236, (uint8_t)56, (uint8_t)130, (uint8_t)226, (uint8_t)235, (uint8_t)136, (uint8_t)224, (uint8_t)204, (uint8_t)118, (uint8_t)134, (uint8_t)88, (uint8_t)233, (uint8_t)254, (uint8_t)112, (uint8_t)156, (uint8_t)66, (uint8_t)255, (uint8_t)38, (uint8_t)77, (uint8_t)100, (uint8_t)152, (uint8_t)22, (uint8_t)107, (uint8_t)40, (uint8_t)93, (uint8_t)115, (uint8_t)85, (uint8_t)14, (uint8_t)100, (uint8_t)10, (uint8_t)180, (uint8_t)39, (uint8_t)211, (uint8_t)93, (uint8_t)202, (uint8_t)244, (uint8_t)74, (uint8_t)14, (uint8_t)147, (uint8_t)105, (uint8_t)93, (uint8_t)17, (uint8_t)97};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        p120_ofs_SET((uint32_t)30661205L, PH.base.pack) ;
        p120_id_SET((uint16_t)(uint16_t)5464, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ERASE_121(), &PH);
        p121_target_system_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p121_target_component_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_system_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p122_target_component_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_target_system_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p123_target_component_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)94, (uint8_t)148, (uint8_t)166, (uint8_t)170, (uint8_t)33, (uint8_t)84, (uint8_t)104, (uint8_t)171, (uint8_t)185, (uint8_t)191, (uint8_t)93, (uint8_t)1, (uint8_t)172, (uint8_t)197, (uint8_t)186, (uint8_t)95, (uint8_t)227, (uint8_t)88, (uint8_t)76, (uint8_t)95, (uint8_t)86, (uint8_t)85, (uint8_t)237, (uint8_t)193, (uint8_t)191, (uint8_t)67, (uint8_t)208, (uint8_t)225, (uint8_t)132, (uint8_t)27, (uint8_t)161, (uint8_t)107, (uint8_t)210, (uint8_t)120, (uint8_t)239, (uint8_t)247, (uint8_t)50, (uint8_t)255, (uint8_t)201, (uint8_t)111, (uint8_t)134, (uint8_t)187, (uint8_t)11, (uint8_t)212, (uint8_t)158, (uint8_t)27, (uint8_t)22, (uint8_t)4, (uint8_t)105, (uint8_t)154, (uint8_t)56, (uint8_t)12, (uint8_t)241, (uint8_t)255, (uint8_t)183, (uint8_t)21, (uint8_t)166, (uint8_t)102, (uint8_t)247, (uint8_t)10, (uint8_t)240, (uint8_t)28, (uint8_t)45, (uint8_t)224, (uint8_t)36, (uint8_t)121, (uint8_t)72, (uint8_t)89, (uint8_t)183, (uint8_t)165, (uint8_t)116, (uint8_t)189, (uint8_t)34, (uint8_t)238, (uint8_t)118, (uint8_t)149, (uint8_t)182, (uint8_t)6, (uint8_t)22, (uint8_t)169, (uint8_t)235, (uint8_t)66, (uint8_t)216, (uint8_t)182, (uint8_t)75, (uint8_t)59, (uint8_t)195, (uint8_t)75, (uint8_t)162, (uint8_t)6, (uint8_t)139, (uint8_t)54, (uint8_t)7, (uint8_t)95, (uint8_t)64, (uint8_t)137, (uint8_t)137, (uint8_t)113, (uint8_t)87, (uint8_t)147, (uint8_t)159, (uint8_t)51, (uint8_t)91, (uint8_t)59, (uint8_t)202, (uint8_t)154, (uint8_t)190, (uint8_t)252, (uint8_t)144, (uint8_t)42};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_len_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RAW_124(), &PH);
        p124_eph_SET((uint16_t)(uint16_t)10302, PH.base.pack) ;
        p124_lat_SET((int32_t) -12403877, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)42727, PH.base.pack) ;
        p124_alt_SET((int32_t)738404057, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)2113104133381955284L, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)32637, PH.base.pack) ;
        p124_lon_SET((int32_t) -1607122032, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)5505, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)3817662718L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POWER_STATUS_125(), &PH);
        p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_OVERCURRENT, PH.base.pack) ;
        p125_Vcc_SET((uint16_t)(uint16_t)28608, PH.base.pack) ;
        p125_Vservo_SET((uint16_t)(uint16_t)773, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL, PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        p126_timeout_SET((uint16_t)(uint16_t)40730, PH.base.pack) ;
        p126_baudrate_SET((uint32_t)2790515056L, PH.base.pack) ;
        p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)3, (uint8_t)251, (uint8_t)184, (uint8_t)48, (uint8_t)43, (uint8_t)245, (uint8_t)162, (uint8_t)137, (uint8_t)206, (uint8_t)81, (uint8_t)216, (uint8_t)234, (uint8_t)0, (uint8_t)213, (uint8_t)240, (uint8_t)131, (uint8_t)12, (uint8_t)91, (uint8_t)199, (uint8_t)108, (uint8_t)130, (uint8_t)81, (uint8_t)178, (uint8_t)115, (uint8_t)94, (uint8_t)211, (uint8_t)84, (uint8_t)224, (uint8_t)183, (uint8_t)195, (uint8_t)126, (uint8_t)115, (uint8_t)239, (uint8_t)138, (uint8_t)103, (uint8_t)106, (uint8_t)152, (uint8_t)57, (uint8_t)69, (uint8_t)157, (uint8_t)66, (uint8_t)171, (uint8_t)94, (uint8_t)240, (uint8_t)180, (uint8_t)17, (uint8_t)143, (uint8_t)78, (uint8_t)28, (uint8_t)113, (uint8_t)187, (uint8_t)206, (uint8_t)247, (uint8_t)78, (uint8_t)122, (uint8_t)97, (uint8_t)161, (uint8_t)169, (uint8_t)189, (uint8_t)196, (uint8_t)196, (uint8_t)13, (uint8_t)119, (uint8_t)141, (uint8_t)126, (uint8_t)124, (uint8_t)131, (uint8_t)81, (uint8_t)245, (uint8_t)81};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTK_127(), &PH);
        p127_tow_SET((uint32_t)4141091657L, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)2438175303L, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)1357272484L, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t)1089269669, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)15974, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t) -795670701, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t) -1773376485, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t) -288526269, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RTK_128(), &PH);
        p128_baseline_c_mm_SET((int32_t)987371513, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)4119232381L, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)16014, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)2349941569L, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t)48509868, PH.base.pack) ;
        p128_tow_SET((uint32_t)363261800L, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t) -686020420, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t) -1214642229, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU3_129(), &PH);
        p129_yacc_SET((int16_t)(int16_t)5143, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t) -1965, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t)20491, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t)779, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)4248968409L, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t) -16546, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t)22583, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t) -28098, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t)29671, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t) -5751, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_jpg_quality_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)49679, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)34376, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)21395, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        p130_size_SET((uint32_t)1087905603L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ENCAPSULATED_DATA_131(), &PH);
        p131_seqnr_SET((uint16_t)(uint16_t)30171, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)151, (uint8_t)21, (uint8_t)212, (uint8_t)59, (uint8_t)72, (uint8_t)34, (uint8_t)160, (uint8_t)29, (uint8_t)117, (uint8_t)213, (uint8_t)6, (uint8_t)247, (uint8_t)97, (uint8_t)222, (uint8_t)40, (uint8_t)176, (uint8_t)194, (uint8_t)187, (uint8_t)149, (uint8_t)230, (uint8_t)215, (uint8_t)246, (uint8_t)9, (uint8_t)41, (uint8_t)253, (uint8_t)142, (uint8_t)158, (uint8_t)117, (uint8_t)155, (uint8_t)158, (uint8_t)211, (uint8_t)51, (uint8_t)27, (uint8_t)112, (uint8_t)62, (uint8_t)117, (uint8_t)208, (uint8_t)228, (uint8_t)72, (uint8_t)37, (uint8_t)233, (uint8_t)159, (uint8_t)36, (uint8_t)6, (uint8_t)192, (uint8_t)117, (uint8_t)154, (uint8_t)57, (uint8_t)214, (uint8_t)28, (uint8_t)24, (uint8_t)28, (uint8_t)107, (uint8_t)60, (uint8_t)10, (uint8_t)170, (uint8_t)131, (uint8_t)99, (uint8_t)14, (uint8_t)168, (uint8_t)179, (uint8_t)117, (uint8_t)23, (uint8_t)140, (uint8_t)28, (uint8_t)25, (uint8_t)255, (uint8_t)6, (uint8_t)7, (uint8_t)211, (uint8_t)103, (uint8_t)38, (uint8_t)189, (uint8_t)250, (uint8_t)36, (uint8_t)150, (uint8_t)211, (uint8_t)91, (uint8_t)0, (uint8_t)116, (uint8_t)109, (uint8_t)204, (uint8_t)97, (uint8_t)100, (uint8_t)151, (uint8_t)107, (uint8_t)18, (uint8_t)166, (uint8_t)18, (uint8_t)68, (uint8_t)119, (uint8_t)235, (uint8_t)7, (uint8_t)139, (uint8_t)235, (uint8_t)77, (uint8_t)34, (uint8_t)17, (uint8_t)210, (uint8_t)190, (uint8_t)43, (uint8_t)55, (uint8_t)123, (uint8_t)213, (uint8_t)96, (uint8_t)213, (uint8_t)14, (uint8_t)136, (uint8_t)255, (uint8_t)198, (uint8_t)61, (uint8_t)6, (uint8_t)117, (uint8_t)20, (uint8_t)27, (uint8_t)128, (uint8_t)229, (uint8_t)148, (uint8_t)122, (uint8_t)113, (uint8_t)228, (uint8_t)110, (uint8_t)16, (uint8_t)86, (uint8_t)147, (uint8_t)164, (uint8_t)60, (uint8_t)58, (uint8_t)215, (uint8_t)76, (uint8_t)117, (uint8_t)78, (uint8_t)138, (uint8_t)175, (uint8_t)50, (uint8_t)244, (uint8_t)223, (uint8_t)123, (uint8_t)36, (uint8_t)61, (uint8_t)45, (uint8_t)240, (uint8_t)4, (uint8_t)22, (uint8_t)232, (uint8_t)199, (uint8_t)137, (uint8_t)78, (uint8_t)104, (uint8_t)136, (uint8_t)215, (uint8_t)35, (uint8_t)101, (uint8_t)96, (uint8_t)114, (uint8_t)82, (uint8_t)35, (uint8_t)223, (uint8_t)15, (uint8_t)66, (uint8_t)117, (uint8_t)29, (uint8_t)177, (uint8_t)181, (uint8_t)76, (uint8_t)187, (uint8_t)75, (uint8_t)195, (uint8_t)34, (uint8_t)232, (uint8_t)185, (uint8_t)162, (uint8_t)173, (uint8_t)150, (uint8_t)244, (uint8_t)207, (uint8_t)136, (uint8_t)233, (uint8_t)211, (uint8_t)66, (uint8_t)133, (uint8_t)153, (uint8_t)95, (uint8_t)167, (uint8_t)167, (uint8_t)92, (uint8_t)89, (uint8_t)187, (uint8_t)213, (uint8_t)137, (uint8_t)0, (uint8_t)131, (uint8_t)108, (uint8_t)161, (uint8_t)112, (uint8_t)113, (uint8_t)236, (uint8_t)148, (uint8_t)9, (uint8_t)81, (uint8_t)56, (uint8_t)175, (uint8_t)249, (uint8_t)203, (uint8_t)157, (uint8_t)76, (uint8_t)47, (uint8_t)34, (uint8_t)153, (uint8_t)46, (uint8_t)100, (uint8_t)7, (uint8_t)155, (uint8_t)45, (uint8_t)146, (uint8_t)41, (uint8_t)66, (uint8_t)213, (uint8_t)144, (uint8_t)207, (uint8_t)91, (uint8_t)20, (uint8_t)188, (uint8_t)79, (uint8_t)224, (uint8_t)87, (uint8_t)220, (uint8_t)174, (uint8_t)50, (uint8_t)91, (uint8_t)117, (uint8_t)141, (uint8_t)53, (uint8_t)152, (uint8_t)164, (uint8_t)22, (uint8_t)201, (uint8_t)246, (uint8_t)140, (uint8_t)121, (uint8_t)143, (uint8_t)140, (uint8_t)157, (uint8_t)54, (uint8_t)156, (uint8_t)197, (uint8_t)106, (uint8_t)3, (uint8_t)67, (uint8_t)79, (uint8_t)249, (uint8_t)130, (uint8_t)248};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_max_distance_SET((uint16_t)(uint16_t)12190, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_270_PITCH_270, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)1719722399L, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)63084, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)18773, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_mask_SET((uint64_t)5045198130015881974L, PH.base.pack) ;
        p133_lon_SET((int32_t) -546150550, PH.base.pack) ;
        p133_lat_SET((int32_t)1909875114, PH.base.pack) ;
        p133_grid_spacing_SET((uint16_t)(uint16_t)62793, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_DATA_134(), &PH);
        p134_gridbit_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        p134_lat_SET((int32_t)881949668, PH.base.pack) ;
        p134_grid_spacing_SET((uint16_t)(uint16_t)29357, PH.base.pack) ;
        p134_lon_SET((int32_t)81012863, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t)30506, (int16_t) -196, (int16_t) -7299, (int16_t)19898, (int16_t) -19640, (int16_t) -27318, (int16_t)22540, (int16_t) -32200, (int16_t) -2762, (int16_t) -14698, (int16_t) -5005, (int16_t)11620, (int16_t) -20376, (int16_t) -20742, (int16_t) -7570, (int16_t) -7619};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lon_SET((int32_t) -907548386, PH.base.pack) ;
        p135_lat_SET((int32_t)1649657393, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_loaded_SET((uint16_t)(uint16_t)62712, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)57924, PH.base.pack) ;
        p136_lat_SET((int32_t) -2106605408, PH.base.pack) ;
        p136_terrain_height_SET((float)1.1474786E38F, PH.base.pack) ;
        p136_lon_SET((int32_t) -419599966, PH.base.pack) ;
        p136_current_height_SET((float)3.0763292E38F, PH.base.pack) ;
        p136_spacing_SET((uint16_t)(uint16_t)45755, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_temperature_SET((int16_t)(int16_t) -25967, PH.base.pack) ;
        p137_press_diff_SET((float) -1.394602E38F, PH.base.pack) ;
        p137_time_boot_ms_SET((uint32_t)1943872072L, PH.base.pack) ;
        p137_press_abs_SET((float)1.802346E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATT_POS_MOCAP_138(), &PH);
        p138_z_SET((float) -4.6914944E36F, PH.base.pack) ;
        {
            float q[] =  {-1.2674449E38F, 3.1475624E38F, 4.842804E37F, -3.5136836E37F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_x_SET((float) -1.3502241E38F, PH.base.pack) ;
        p138_time_usec_SET((uint64_t)3084115421394166686L, PH.base.pack) ;
        p138_y_SET((float) -7.4446544E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        {
            float controls[] =  {-2.923095E38F, -1.2090126E37F, -6.3982905E37F, 1.4410708E37F, 1.4688799E38F, -2.4167106E38F, -4.400476E37F, 2.8209485E38F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_group_mlx_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        p139_target_system_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        p139_time_usec_SET((uint64_t)4355607148381091790L, PH.base.pack) ;
        p139_target_component_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        {
            float controls[] =  {-1.4799952E38F, 5.670321E37F, 2.810417E38F, 3.1342777E38F, 1.5096031E38F, 1.7014222E38F, 2.2655076E38F, -2.7770111E38F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p140_time_usec_SET((uint64_t)1282913368201206871L, PH.base.pack) ;
        p140_group_mlx_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ALTITUDE_141(), &PH);
        p141_altitude_local_SET((float)2.209842E38F, PH.base.pack) ;
        p141_altitude_monotonic_SET((float)3.081234E38F, PH.base.pack) ;
        p141_altitude_terrain_SET((float) -1.1050284E38F, PH.base.pack) ;
        p141_altitude_amsl_SET((float)1.3755245E38F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)7151248871248795174L, PH.base.pack) ;
        p141_bottom_clearance_SET((float) -1.8705608E38F, PH.base.pack) ;
        p141_altitude_relative_SET((float) -2.506123E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RESOURCE_REQUEST_142(), &PH);
        {
            uint8_t uri[] =  {(uint8_t)109, (uint8_t)78, (uint8_t)193, (uint8_t)93, (uint8_t)158, (uint8_t)200, (uint8_t)135, (uint8_t)84, (uint8_t)187, (uint8_t)166, (uint8_t)136, (uint8_t)163, (uint8_t)114, (uint8_t)77, (uint8_t)36, (uint8_t)184, (uint8_t)114, (uint8_t)52, (uint8_t)149, (uint8_t)153, (uint8_t)143, (uint8_t)27, (uint8_t)227, (uint8_t)0, (uint8_t)237, (uint8_t)116, (uint8_t)29, (uint8_t)108, (uint8_t)99, (uint8_t)236, (uint8_t)193, (uint8_t)50, (uint8_t)92, (uint8_t)200, (uint8_t)159, (uint8_t)240, (uint8_t)135, (uint8_t)218, (uint8_t)224, (uint8_t)154, (uint8_t)193, (uint8_t)134, (uint8_t)73, (uint8_t)84, (uint8_t)173, (uint8_t)63, (uint8_t)83, (uint8_t)254, (uint8_t)226, (uint8_t)197, (uint8_t)225, (uint8_t)235, (uint8_t)226, (uint8_t)210, (uint8_t)53, (uint8_t)249, (uint8_t)60, (uint8_t)231, (uint8_t)132, (uint8_t)181, (uint8_t)129, (uint8_t)128, (uint8_t)139, (uint8_t)36, (uint8_t)91, (uint8_t)218, (uint8_t)208, (uint8_t)63, (uint8_t)125, (uint8_t)177, (uint8_t)201, (uint8_t)169, (uint8_t)170, (uint8_t)236, (uint8_t)62, (uint8_t)124, (uint8_t)188, (uint8_t)176, (uint8_t)50, (uint8_t)250, (uint8_t)205, (uint8_t)223, (uint8_t)0, (uint8_t)178, (uint8_t)145, (uint8_t)32, (uint8_t)91, (uint8_t)230, (uint8_t)56, (uint8_t)67, (uint8_t)92, (uint8_t)245, (uint8_t)186, (uint8_t)144, (uint8_t)218, (uint8_t)55, (uint8_t)27, (uint8_t)200, (uint8_t)162, (uint8_t)206, (uint8_t)219, (uint8_t)133, (uint8_t)218, (uint8_t)194, (uint8_t)104, (uint8_t)66, (uint8_t)233, (uint8_t)176, (uint8_t)15, (uint8_t)28, (uint8_t)90, (uint8_t)77, (uint8_t)217, (uint8_t)96, (uint8_t)242, (uint8_t)40, (uint8_t)185, (uint8_t)55, (uint8_t)114, (uint8_t)96};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        p142_transfer_type_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
        p142_uri_type_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        {
            uint8_t storage[] =  {(uint8_t)252, (uint8_t)151, (uint8_t)170, (uint8_t)190, (uint8_t)143, (uint8_t)24, (uint8_t)133, (uint8_t)221, (uint8_t)131, (uint8_t)38, (uint8_t)218, (uint8_t)208, (uint8_t)98, (uint8_t)123, (uint8_t)193, (uint8_t)201, (uint8_t)153, (uint8_t)3, (uint8_t)220, (uint8_t)82, (uint8_t)47, (uint8_t)115, (uint8_t)59, (uint8_t)31, (uint8_t)185, (uint8_t)97, (uint8_t)68, (uint8_t)25, (uint8_t)81, (uint8_t)124, (uint8_t)3, (uint8_t)195, (uint8_t)11, (uint8_t)148, (uint8_t)60, (uint8_t)228, (uint8_t)133, (uint8_t)5, (uint8_t)119, (uint8_t)165, (uint8_t)64, (uint8_t)124, (uint8_t)163, (uint8_t)47, (uint8_t)50, (uint8_t)243, (uint8_t)125, (uint8_t)0, (uint8_t)174, (uint8_t)213, (uint8_t)119, (uint8_t)99, (uint8_t)181, (uint8_t)37, (uint8_t)30, (uint8_t)111, (uint8_t)29, (uint8_t)190, (uint8_t)8, (uint8_t)186, (uint8_t)217, (uint8_t)207, (uint8_t)49, (uint8_t)76, (uint8_t)66, (uint8_t)249, (uint8_t)127, (uint8_t)105, (uint8_t)94, (uint8_t)139, (uint8_t)85, (uint8_t)146, (uint8_t)41, (uint8_t)67, (uint8_t)46, (uint8_t)74, (uint8_t)199, (uint8_t)41, (uint8_t)44, (uint8_t)217, (uint8_t)15, (uint8_t)98, (uint8_t)144, (uint8_t)188, (uint8_t)207, (uint8_t)178, (uint8_t)144, (uint8_t)242, (uint8_t)44, (uint8_t)96, (uint8_t)223, (uint8_t)46, (uint8_t)87, (uint8_t)118, (uint8_t)210, (uint8_t)216, (uint8_t)131, (uint8_t)198, (uint8_t)251, (uint8_t)109, (uint8_t)146, (uint8_t)242, (uint8_t)208, (uint8_t)173, (uint8_t)85, (uint8_t)61, (uint8_t)135, (uint8_t)120, (uint8_t)243, (uint8_t)155, (uint8_t)31, (uint8_t)111, (uint8_t)158, (uint8_t)52, (uint8_t)200, (uint8_t)198, (uint8_t)226, (uint8_t)135, (uint8_t)95, (uint8_t)6};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        p142_request_id_SET((uint8_t)(uint8_t)196, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_press_diff_SET((float) -6.446512E37F, PH.base.pack) ;
        p143_temperature_SET((int16_t)(int16_t) -1936, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)301252434L, PH.base.pack) ;
        p143_press_abs_SET((float)3.3359274E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FOLLOW_TARGET_144(), &PH);
        {
            float attitude_q[] =  {2.0837202E38F, -8.564641E37F, -2.751647E38F, 3.2136598E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        p144_est_capabilities_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p144_lon_SET((int32_t) -1431688308, PH.base.pack) ;
        {
            float rates[] =  {-7.676915E37F, -1.933088E38F, 3.3357357E38F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        p144_timestamp_SET((uint64_t)4508543658451459500L, PH.base.pack) ;
        {
            float acc[] =  {3.0747788E38F, -3.282472E37F, 3.2775796E38F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        p144_lat_SET((int32_t) -2119254407, PH.base.pack) ;
        p144_alt_SET((float)1.2751748E38F, PH.base.pack) ;
        {
            float vel[] =  {-2.0564846E38F, -2.0070848E38F, 9.417945E37F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        p144_custom_state_SET((uint64_t)6281143862288843862L, PH.base.pack) ;
        {
            float position_cov[] =  {-2.0805243E38F, 3.8223933E37F, 1.0781675E38F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_y_pos_SET((float) -1.1970083E38F, PH.base.pack) ;
        {
            float q[] =  {1.9622343E38F, 2.919766E38F, 3.758412E37F, 1.6440576E38F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        {
            float vel_variance[] =  {2.7453927E38F, -7.922378E37F, -8.719615E37F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        {
            float pos_variance[] =  {-7.851598E37F, 2.3192197E38F, 7.864304E37F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_z_vel_SET((float)8.2071746E37F, PH.base.pack) ;
        p146_x_pos_SET((float)2.7127883E38F, PH.base.pack) ;
        p146_x_acc_SET((float)2.6040015E37F, PH.base.pack) ;
        p146_y_vel_SET((float) -2.3755977E38F, PH.base.pack) ;
        p146_z_pos_SET((float)1.9250503E38F, PH.base.pack) ;
        p146_y_acc_SET((float) -3.2815207E38F, PH.base.pack) ;
        p146_yaw_rate_SET((float)5.716821E37F, PH.base.pack) ;
        p146_x_vel_SET((float)1.3500513E38F, PH.base.pack) ;
        p146_airspeed_SET((float) -1.6229394E38F, PH.base.pack) ;
        p146_time_usec_SET((uint64_t)5210835352239418770L, PH.base.pack) ;
        p146_roll_rate_SET((float)1.5119659E38F, PH.base.pack) ;
        p146_z_acc_SET((float) -1.8951675E38F, PH.base.pack) ;
        p146_pitch_rate_SET((float)3.8926122E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BATTERY_STATUS_147(), &PH);
        p147_current_consumed_SET((int32_t)167114971, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t)14291, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t) -4501, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_AVIONICS, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)43808, (uint16_t)40100, (uint16_t)7192, (uint16_t)1603, (uint16_t)50056, (uint16_t)60856, (uint16_t)10405, (uint16_t)1002, (uint16_t)34679, (uint16_t)9404};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_energy_consumed_SET((int32_t) -2123353129, PH.base.pack) ;
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIPO, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t) -106, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_148(), &PH);
        {
            uint8_t uid2[] =  {(uint8_t)195, (uint8_t)251, (uint8_t)251, (uint8_t)147, (uint8_t)100, (uint8_t)86, (uint8_t)228, (uint8_t)159, (uint8_t)218, (uint8_t)227, (uint8_t)221, (uint8_t)136, (uint8_t)154, (uint8_t)127, (uint8_t)190, (uint8_t)196, (uint8_t)243, (uint8_t)10};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET, PH.base.pack) ;
        p148_middleware_sw_version_SET((uint32_t)1747828716L, PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)215, (uint8_t)241, (uint8_t)147, (uint8_t)166, (uint8_t)240, (uint8_t)191, (uint8_t)84, (uint8_t)48};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        p148_vendor_id_SET((uint16_t)(uint16_t)39551, PH.base.pack) ;
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)206, (uint8_t)49, (uint8_t)130, (uint8_t)148, (uint8_t)190, (uint8_t)216, (uint8_t)235, (uint8_t)236};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        p148_uid_SET((uint64_t)2384860938964567975L, PH.base.pack) ;
        {
            uint8_t flight_custom_version[] =  {(uint8_t)61, (uint8_t)228, (uint8_t)115, (uint8_t)242, (uint8_t)63, (uint8_t)203, (uint8_t)161, (uint8_t)213};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        p148_flight_sw_version_SET((uint32_t)4058879657L, PH.base.pack) ;
        p148_board_version_SET((uint32_t)173704400L, PH.base.pack) ;
        p148_os_sw_version_SET((uint32_t)1644113992L, PH.base.pack) ;
        p148_product_id_SET((uint16_t)(uint16_t)23748, PH.base.pack) ;
        c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LANDING_TARGET_149(), &PH);
        p149_target_num_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
        p149_angle_y_SET((float) -1.860374E38F, PH.base.pack) ;
        p149_distance_SET((float)1.325009E38F, PH.base.pack) ;
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_FIDUCIAL, PH.base.pack) ;
        p149_size_y_SET((float) -3.3474183E37F, PH.base.pack) ;
        p149_y_SET((float)1.3351318E38F, &PH) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p149_x_SET((float) -5.111856E37F, &PH) ;
        {
            float q[] =  {1.7037305E38F, 3.8433452E37F, -2.6192434E38F, -1.3906081E38F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_size_x_SET((float) -2.977634E38F, PH.base.pack) ;
        p149_angle_x_SET((float) -4.836181E37F, PH.base.pack) ;
        p149_z_SET((float) -2.0506976E38F, &PH) ;
        p149_time_usec_SET((uint64_t)2646807189233344234L, PH.base.pack) ;
        p149_position_valid_SET((uint8_t)(uint8_t)200, &PH) ;
        c_LoopBackDemoChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SENS_POWER_201(), &PH);
        p201_adc121_cspb_amp_SET((float) -2.472064E38F, PH.base.pack) ;
        p201_adc121_cs1_amp_SET((float)1.1484542E38F, PH.base.pack) ;
        p201_adc121_cs2_amp_SET((float)1.2709903E38F, PH.base.pack) ;
        p201_adc121_vspb_volt_SET((float) -6.6312517E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SENS_POWER_201(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SENS_MPPT_202(), &PH);
        p202_mppt2_status_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
        p202_mppt1_status_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        p202_mppt3_pwm_SET((uint16_t)(uint16_t)25571, PH.base.pack) ;
        p202_mppt1_amp_SET((float)6.2788347E37F, PH.base.pack) ;
        p202_mppt3_volt_SET((float) -2.8348923E38F, PH.base.pack) ;
        p202_mppt2_volt_SET((float)2.6294315E38F, PH.base.pack) ;
        p202_mppt1_pwm_SET((uint16_t)(uint16_t)40393, PH.base.pack) ;
        p202_mppt_timestamp_SET((uint64_t)7709173357184220256L, PH.base.pack) ;
        p202_mppt2_amp_SET((float)7.443564E36F, PH.base.pack) ;
        p202_mppt2_pwm_SET((uint16_t)(uint16_t)31714, PH.base.pack) ;
        p202_mppt3_status_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        p202_mppt1_volt_SET((float) -1.4196332E38F, PH.base.pack) ;
        p202_mppt3_amp_SET((float)1.6447508E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SENS_MPPT_202(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ASLCTRL_DATA_203(), &PH);
        p203_r_SET((float)2.7407695E38F, PH.base.pack) ;
        p203_aslctrl_mode_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        p203_RollAngleRef_SET((float) -3.3198287E38F, PH.base.pack) ;
        p203_pRef_SET((float)1.4270859E38F, PH.base.pack) ;
        p203_h_SET((float) -8.204243E37F, PH.base.pack) ;
        p203_qRef_SET((float)5.0166623E37F, PH.base.pack) ;
        p203_uRud_SET((float)6.354283E37F, PH.base.pack) ;
        p203_PitchAngleRef_SET((float)1.2853069E38F, PH.base.pack) ;
        p203_RollAngle_SET((float) -2.0206758E38F, PH.base.pack) ;
        p203_rRef_SET((float)1.7257706E38F, PH.base.pack) ;
        p203_hRef_t_SET((float)2.5684188E37F, PH.base.pack) ;
        p203_hRef_SET((float)2.8496761E38F, PH.base.pack) ;
        p203_SpoilersEngaged_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
        p203_YawAngleRef_SET((float)1.0213407E38F, PH.base.pack) ;
        p203_timestamp_SET((uint64_t)2839711964677103812L, PH.base.pack) ;
        p203_PitchAngle_SET((float)8.536788E37F, PH.base.pack) ;
        p203_uThrot_SET((float)1.03157875E37F, PH.base.pack) ;
        p203_q_SET((float)1.6267226E38F, PH.base.pack) ;
        p203_uAil_SET((float)2.509371E38F, PH.base.pack) ;
        p203_nZ_SET((float) -1.8932135E38F, PH.base.pack) ;
        p203_YawAngle_SET((float)1.4467668E38F, PH.base.pack) ;
        p203_p_SET((float) -1.1820807E38F, PH.base.pack) ;
        p203_uElev_SET((float)6.2933285E37F, PH.base.pack) ;
        p203_AirspeedRef_SET((float)2.2330905E38F, PH.base.pack) ;
        p203_uThrot2_SET((float) -2.7862985E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ASLCTRL_DATA_203(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ASLCTRL_DEBUG_204(), &PH);
        p204_f_1_SET((float)1.4823851E38F, PH.base.pack) ;
        p204_f_3_SET((float) -1.5716811E38F, PH.base.pack) ;
        p204_f_6_SET((float)1.6534829E38F, PH.base.pack) ;
        p204_f_7_SET((float) -2.0398502E38F, PH.base.pack) ;
        p204_f_5_SET((float) -3.473277E37F, PH.base.pack) ;
        p204_f_4_SET((float)1.141648E38F, PH.base.pack) ;
        p204_f_2_SET((float)1.0981849E38F, PH.base.pack) ;
        p204_f_8_SET((float) -3.1197891E38F, PH.base.pack) ;
        p204_i8_2_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p204_i8_1_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        p204_i32_1_SET((uint32_t)3477459138L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ASLCTRL_DEBUG_204(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ASLUAV_STATUS_205(), &PH);
        p205_LED_status_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        {
            uint8_t Servo_status[] =  {(uint8_t)186, (uint8_t)193, (uint8_t)68, (uint8_t)163, (uint8_t)127, (uint8_t)175, (uint8_t)234, (uint8_t)120};
            p205_Servo_status_SET(&Servo_status, 0, PH.base.pack) ;
        }
        p205_Motor_rpm_SET((float)1.7545795E38F, PH.base.pack) ;
        p205_SATCOM_status_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ASLUAV_STATUS_205(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_EKF_EXT_206(), &PH);
        p206_alpha_SET((float) -1.8749775E38F, PH.base.pack) ;
        p206_timestamp_SET((uint64_t)3959655981415736750L, PH.base.pack) ;
        p206_Windspeed_SET((float)2.4845583E38F, PH.base.pack) ;
        p206_beta_SET((float) -2.042851E38F, PH.base.pack) ;
        p206_WindZ_SET((float) -1.1933851E38F, PH.base.pack) ;
        p206_Airspeed_SET((float) -2.9590606E38F, PH.base.pack) ;
        p206_WindDir_SET((float) -3.1065693E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_EKF_EXT_206(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ASL_OBCTRL_207(), &PH);
        p207_obctrl_status_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p207_uThrot_SET((float) -3.3603827E38F, PH.base.pack) ;
        p207_timestamp_SET((uint64_t)1581780945369460773L, PH.base.pack) ;
        p207_uThrot2_SET((float)9.397812E37F, PH.base.pack) ;
        p207_uElev_SET((float) -1.8069589E38F, PH.base.pack) ;
        p207_uAilL_SET((float) -1.2966283E38F, PH.base.pack) ;
        p207_uRud_SET((float)2.4662937E38F, PH.base.pack) ;
        p207_uAilR_SET((float)3.176943E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ASL_OBCTRL_207(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SENS_ATMOS_208(), &PH);
        p208_Humidity_SET((float) -4.8499693E37F, PH.base.pack) ;
        p208_TempAmbient_SET((float) -1.6661712E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SENS_ATMOS_208(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SENS_BATMON_209(), &PH);
        p209_cellvoltage3_SET((uint16_t)(uint16_t)58613, PH.base.pack) ;
        p209_hostfetcontrol_SET((uint16_t)(uint16_t)4540, PH.base.pack) ;
        p209_cellvoltage2_SET((uint16_t)(uint16_t)34114, PH.base.pack) ;
        p209_serialnumber_SET((uint16_t)(uint16_t)13491, PH.base.pack) ;
        p209_cellvoltage4_SET((uint16_t)(uint16_t)26533, PH.base.pack) ;
        p209_cellvoltage5_SET((uint16_t)(uint16_t)43417, PH.base.pack) ;
        p209_cellvoltage1_SET((uint16_t)(uint16_t)7204, PH.base.pack) ;
        p209_voltage_SET((uint16_t)(uint16_t)64240, PH.base.pack) ;
        p209_batterystatus_SET((uint16_t)(uint16_t)52349, PH.base.pack) ;
        p209_current_SET((int16_t)(int16_t)23965, PH.base.pack) ;
        p209_SoC_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p209_temperature_SET((float) -1.6098552E38F, PH.base.pack) ;
        p209_cellvoltage6_SET((uint16_t)(uint16_t)11043, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SENS_BATMON_209(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FW_SOARING_DATA_210(), &PH);
        p210_xW_SET((float)2.9684097E38F, PH.base.pack) ;
        p210_xLat_SET((float) -2.7183572E38F, PH.base.pack) ;
        p210_VarR_SET((float) -1.7816513E38F, PH.base.pack) ;
        p210_z1_LocalUpdraftSpeed_SET((float)7.254837E37F, PH.base.pack) ;
        p210_vSinkExp_SET((float)1.6848517E38F, PH.base.pack) ;
        p210_DebugVar1_SET((float)5.863707E37F, PH.base.pack) ;
        p210_LoiterRadius_SET((float) -2.3070618E38F, PH.base.pack) ;
        p210_z2_exp_SET((float)3.3330948E38F, PH.base.pack) ;
        p210_VarW_SET((float)1.5323193E38F, PH.base.pack) ;
        p210_TSE_dot_SET((float) -2.5806823E37F, PH.base.pack) ;
        p210_valid_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p210_VarLon_SET((float) -1.5190939E38F, PH.base.pack) ;
        p210_ThermalGSNorth_SET((float) -1.5927078E38F, PH.base.pack) ;
        p210_ControlMode_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        p210_LoiterDirection_SET((float) -2.1322698E38F, PH.base.pack) ;
        p210_z1_exp_SET((float)2.1347374E38F, PH.base.pack) ;
        p210_xR_SET((float) -8.115779E37F, PH.base.pack) ;
        p210_VarLat_SET((float)1.8490593E38F, PH.base.pack) ;
        p210_ThermalGSEast_SET((float)1.5255992E38F, PH.base.pack) ;
        p210_timestamp_SET((uint64_t)8254730636230267653L, PH.base.pack) ;
        p210_xLon_SET((float)8.168664E37F, PH.base.pack) ;
        p210_z2_DeltaRoll_SET((float)1.0596796E38F, PH.base.pack) ;
        p210_DistToSoarPoint_SET((float)2.6811417E38F, PH.base.pack) ;
        p210_timestampModeChanged_SET((uint64_t)3964925808991726004L, PH.base.pack) ;
        p210_DebugVar2_SET((float) -6.694976E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FW_SOARING_DATA_210(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SENSORPOD_STATUS_211(), &PH);
        p211_visensor_rate_3_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p211_cpu_temp_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
        p211_visensor_rate_2_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p211_free_space_SET((uint16_t)(uint16_t)35977, PH.base.pack) ;
        p211_recording_nodes_count_SET((uint8_t)(uint8_t)72, PH.base.pack) ;
        p211_visensor_rate_4_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        p211_timestamp_SET((uint64_t)4828835503268257265L, PH.base.pack) ;
        p211_visensor_rate_1_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SENSORPOD_STATUS_211(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SENS_POWER_BOARD_212(), &PH);
        p212_pwr_brd_servo_4_amp_SET((float)3.2339803E38F, PH.base.pack) ;
        p212_pwr_brd_system_volt_SET((float)1.7004213E38F, PH.base.pack) ;
        p212_pwr_brd_status_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        p212_timestamp_SET((uint64_t)7977805117771849541L, PH.base.pack) ;
        p212_pwr_brd_led_status_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        p212_pwr_brd_servo_volt_SET((float)2.5396808E38F, PH.base.pack) ;
        p212_pwr_brd_servo_1_amp_SET((float) -1.8028186E38F, PH.base.pack) ;
        p212_pwr_brd_mot_l_amp_SET((float)8.649035E37F, PH.base.pack) ;
        p212_pwr_brd_aux_amp_SET((float) -2.094471E38F, PH.base.pack) ;
        p212_pwr_brd_mot_r_amp_SET((float) -2.4720256E38F, PH.base.pack) ;
        p212_pwr_brd_servo_3_amp_SET((float) -3.2302747E38F, PH.base.pack) ;
        p212_pwr_brd_servo_2_amp_SET((float) -2.300927E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SENS_POWER_BOARD_212(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_hagl_ratio_SET((float) -1.1146074E38F, PH.base.pack) ;
        p230_mag_ratio_SET((float)2.5343443E38F, PH.base.pack) ;
        p230_tas_ratio_SET((float) -1.6221537E38F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float) -3.3297192E38F, PH.base.pack) ;
        p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_ABS, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float)1.3621639E38F, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float) -1.3561237E38F, PH.base.pack) ;
        p230_vel_ratio_SET((float) -1.924471E38F, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float) -1.0157043E38F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)2222425225711890723L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIND_COV_231(), &PH);
        p231_wind_alt_SET((float)5.032081E37F, PH.base.pack) ;
        p231_wind_x_SET((float) -1.5283752E38F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)4339778027335178994L, PH.base.pack) ;
        p231_var_horiz_SET((float) -3.0530284E38F, PH.base.pack) ;
        p231_wind_y_SET((float)1.3150941E38F, PH.base.pack) ;
        p231_wind_z_SET((float)6.33732E37F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float) -1.8065092E38F, PH.base.pack) ;
        p231_vert_accuracy_SET((float) -7.8305206E37F, PH.base.pack) ;
        p231_var_vert_SET((float) -3.2011423E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INPUT_232(), &PH);
        p232_time_week_SET((uint16_t)(uint16_t)63002, PH.base.pack) ;
        p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_HORIZ, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p232_vdop_SET((float) -2.484766E38F, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)4555437622948578456L, PH.base.pack) ;
        p232_lat_SET((int32_t) -861731653, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        p232_vn_SET((float) -1.4626416E37F, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)4109536047L, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
        p232_speed_accuracy_SET((float)6.4142664E37F, PH.base.pack) ;
        p232_vd_SET((float)3.2100513E38F, PH.base.pack) ;
        p232_lon_SET((int32_t) -1885416528, PH.base.pack) ;
        p232_alt_SET((float)1.2941889E37F, PH.base.pack) ;
        p232_hdop_SET((float)8.850664E37F, PH.base.pack) ;
        p232_vert_accuracy_SET((float) -3.0716042E38F, PH.base.pack) ;
        p232_horiz_accuracy_SET((float) -7.165747E37F, PH.base.pack) ;
        p232_ve_SET((float) -2.5622385E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTCM_DATA_233(), &PH);
        p233_flags_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)38, (uint8_t)23, (uint8_t)142, (uint8_t)244, (uint8_t)9, (uint8_t)233, (uint8_t)143, (uint8_t)224, (uint8_t)106, (uint8_t)255, (uint8_t)102, (uint8_t)106, (uint8_t)128, (uint8_t)216, (uint8_t)163, (uint8_t)178, (uint8_t)126, (uint8_t)24, (uint8_t)148, (uint8_t)39, (uint8_t)94, (uint8_t)199, (uint8_t)16, (uint8_t)45, (uint8_t)57, (uint8_t)44, (uint8_t)116, (uint8_t)203, (uint8_t)13, (uint8_t)81, (uint8_t)220, (uint8_t)55, (uint8_t)47, (uint8_t)255, (uint8_t)3, (uint8_t)185, (uint8_t)200, (uint8_t)189, (uint8_t)131, (uint8_t)91, (uint8_t)72, (uint8_t)0, (uint8_t)155, (uint8_t)66, (uint8_t)173, (uint8_t)145, (uint8_t)65, (uint8_t)81, (uint8_t)16, (uint8_t)73, (uint8_t)148, (uint8_t)182, (uint8_t)93, (uint8_t)68, (uint8_t)218, (uint8_t)173, (uint8_t)192, (uint8_t)214, (uint8_t)215, (uint8_t)172, (uint8_t)207, (uint8_t)89, (uint8_t)87, (uint8_t)192, (uint8_t)61, (uint8_t)119, (uint8_t)45, (uint8_t)183, (uint8_t)171, (uint8_t)9, (uint8_t)39, (uint8_t)69, (uint8_t)45, (uint8_t)221, (uint8_t)53, (uint8_t)29, (uint8_t)122, (uint8_t)63, (uint8_t)68, (uint8_t)168, (uint8_t)253, (uint8_t)235, (uint8_t)222, (uint8_t)150, (uint8_t)144, (uint8_t)143, (uint8_t)201, (uint8_t)50, (uint8_t)55, (uint8_t)34, (uint8_t)57, (uint8_t)216, (uint8_t)151, (uint8_t)146, (uint8_t)221, (uint8_t)137, (uint8_t)187, (uint8_t)167, (uint8_t)87, (uint8_t)134, (uint8_t)105, (uint8_t)182, (uint8_t)129, (uint8_t)122, (uint8_t)244, (uint8_t)16, (uint8_t)115, (uint8_t)223, (uint8_t)234, (uint8_t)54, (uint8_t)22, (uint8_t)188, (uint8_t)192, (uint8_t)41, (uint8_t)98, (uint8_t)157, (uint8_t)113, (uint8_t)4, (uint8_t)147, (uint8_t)199, (uint8_t)1, (uint8_t)37, (uint8_t)194, (uint8_t)181, (uint8_t)219, (uint8_t)23, (uint8_t)143, (uint8_t)232, (uint8_t)62, (uint8_t)155, (uint8_t)83, (uint8_t)79, (uint8_t)200, (uint8_t)236, (uint8_t)3, (uint8_t)247, (uint8_t)66, (uint8_t)199, (uint8_t)35, (uint8_t)149, (uint8_t)99, (uint8_t)156, (uint8_t)199, (uint8_t)231, (uint8_t)43, (uint8_t)23, (uint8_t)116, (uint8_t)65, (uint8_t)169, (uint8_t)194, (uint8_t)148, (uint8_t)0, (uint8_t)71, (uint8_t)88, (uint8_t)245, (uint8_t)135, (uint8_t)32, (uint8_t)145, (uint8_t)170, (uint8_t)160, (uint8_t)199, (uint8_t)66, (uint8_t)50, (uint8_t)70, (uint8_t)175, (uint8_t)112, (uint8_t)150, (uint8_t)38, (uint8_t)240, (uint8_t)39, (uint8_t)49, (uint8_t)80, (uint8_t)85, (uint8_t)155, (uint8_t)0, (uint8_t)2, (uint8_t)212, (uint8_t)195, (uint8_t)204, (uint8_t)17};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        p233_len_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGH_LATENCY_234(), &PH);
        p234_longitude_SET((int32_t) -487214218, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t)39, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t)10450, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t) -5039, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)4735, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t)27742, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)1441592018L, PH.base.pack) ;
        p234_latitude_SET((int32_t)444984026, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t) -23334, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t)25928, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)47362, PH.base.pack) ;
        p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_GUIDED_ENABLED, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t) -5, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t) -122, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t)52, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIBRATION_241(), &PH);
        p241_vibration_y_SET((float)2.524764E38F, PH.base.pack) ;
        p241_time_usec_SET((uint64_t)4812390448161481924L, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)203456575L, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)2023792727L, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)4045557072L, PH.base.pack) ;
        p241_vibration_x_SET((float) -1.0071702E38F, PH.base.pack) ;
        p241_vibration_z_SET((float)1.7776869E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HOME_POSITION_242(), &PH);
        p242_x_SET((float) -6.500271E37F, PH.base.pack) ;
        p242_approach_x_SET((float)9.843417E37F, PH.base.pack) ;
        p242_y_SET((float)2.0991635E38F, PH.base.pack) ;
        p242_longitude_SET((int32_t) -1321795391, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)6048627198369473930L, &PH) ;
        {
            float q[] =  {2.5609722E38F, -3.2643139E38F, 4.5071637E37F, -1.2297266E38F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_approach_y_SET((float)2.987103E38F, PH.base.pack) ;
        p242_approach_z_SET((float) -2.7606063E38F, PH.base.pack) ;
        p242_z_SET((float)1.674138E38F, PH.base.pack) ;
        p242_altitude_SET((int32_t)1253789822, PH.base.pack) ;
        p242_latitude_SET((int32_t) -1652085522, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_HOME_POSITION_243(), &PH);
        p243_approach_y_SET((float)2.8431512E37F, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)4689729217731423973L, &PH) ;
        p243_approach_z_SET((float) -1.1804108E38F, PH.base.pack) ;
        p243_altitude_SET((int32_t)1381970857, PH.base.pack) ;
        {
            float q[] =  {2.3337851E38F, -2.706834E38F, 2.3572868E38F, 1.1489484E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_z_SET((float) -1.5725698E38F, PH.base.pack) ;
        p243_y_SET((float) -3.3332408E38F, PH.base.pack) ;
        p243_x_SET((float)3.2857491E38F, PH.base.pack) ;
        p243_approach_x_SET((float) -2.6963608E38F, PH.base.pack) ;
        p243_longitude_SET((int32_t) -746852402, PH.base.pack) ;
        p243_latitude_SET((int32_t) -1184188248, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_message_id_SET((uint16_t)(uint16_t)23162, PH.base.pack) ;
        p244_interval_us_SET((int32_t)97104549, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_MC, PH.base.pack) ;
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, PH.base.pack) ;
        c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ADSB_VEHICLE_246(), &PH);
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, PH.base.pack) ;
        p246_lon_SET((int32_t)69433851, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)9866, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)46649, PH.base.pack) ;
        p246_altitude_SET((int32_t) -2079155041, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t) -935, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)3178382147L, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p246_lat_SET((int32_t)1442366609, PH.base.pack) ;
        p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)17816, PH.base.pack) ;
        {
            char16_t* callsign = u"imiuG";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_POINT_OBSTACLE, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COLLISION_247(), &PH);
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float)1.0671214E38F, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float) -2.6483261E38F, PH.base.pack) ;
        p247_id_SET((uint32_t)2225551542L, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_REPORT, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float) -1.3951047E38F, PH.base.pack) ;
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_V2_EXTENSION_248(), &PH);
        p248_target_component_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)124, (uint8_t)62, (uint8_t)85, (uint8_t)10, (uint8_t)189, (uint8_t)227, (uint8_t)9, (uint8_t)69, (uint8_t)235, (uint8_t)197, (uint8_t)166, (uint8_t)221, (uint8_t)117, (uint8_t)194, (uint8_t)5, (uint8_t)169, (uint8_t)224, (uint8_t)222, (uint8_t)145, (uint8_t)5, (uint8_t)206, (uint8_t)18, (uint8_t)205, (uint8_t)168, (uint8_t)227, (uint8_t)186, (uint8_t)93, (uint8_t)115, (uint8_t)56, (uint8_t)14, (uint8_t)105, (uint8_t)202, (uint8_t)5, (uint8_t)21, (uint8_t)28, (uint8_t)90, (uint8_t)239, (uint8_t)158, (uint8_t)6, (uint8_t)252, (uint8_t)90, (uint8_t)153, (uint8_t)16, (uint8_t)115, (uint8_t)211, (uint8_t)97, (uint8_t)53, (uint8_t)16, (uint8_t)59, (uint8_t)9, (uint8_t)99, (uint8_t)245, (uint8_t)208, (uint8_t)238, (uint8_t)197, (uint8_t)248, (uint8_t)130, (uint8_t)17, (uint8_t)182, (uint8_t)255, (uint8_t)63, (uint8_t)245, (uint8_t)125, (uint8_t)113, (uint8_t)198, (uint8_t)168, (uint8_t)154, (uint8_t)252, (uint8_t)95, (uint8_t)225, (uint8_t)42, (uint8_t)124, (uint8_t)51, (uint8_t)28, (uint8_t)239, (uint8_t)209, (uint8_t)2, (uint8_t)95, (uint8_t)198, (uint8_t)60, (uint8_t)166, (uint8_t)125, (uint8_t)162, (uint8_t)82, (uint8_t)114, (uint8_t)104, (uint8_t)254, (uint8_t)208, (uint8_t)88, (uint8_t)106, (uint8_t)225, (uint8_t)238, (uint8_t)128, (uint8_t)37, (uint8_t)194, (uint8_t)60, (uint8_t)117, (uint8_t)38, (uint8_t)20, (uint8_t)193, (uint8_t)27, (uint8_t)85, (uint8_t)28, (uint8_t)217, (uint8_t)174, (uint8_t)27, (uint8_t)244, (uint8_t)18, (uint8_t)195, (uint8_t)74, (uint8_t)181, (uint8_t)27, (uint8_t)109, (uint8_t)181, (uint8_t)227, (uint8_t)57, (uint8_t)105, (uint8_t)51, (uint8_t)123, (uint8_t)1, (uint8_t)202, (uint8_t)107, (uint8_t)23, (uint8_t)83, (uint8_t)197, (uint8_t)84, (uint8_t)178, (uint8_t)228, (uint8_t)24, (uint8_t)9, (uint8_t)239, (uint8_t)160, (uint8_t)213, (uint8_t)91, (uint8_t)47, (uint8_t)194, (uint8_t)38, (uint8_t)109, (uint8_t)28, (uint8_t)42, (uint8_t)116, (uint8_t)85, (uint8_t)40, (uint8_t)102, (uint8_t)139, (uint8_t)36, (uint8_t)87, (uint8_t)211, (uint8_t)111, (uint8_t)87, (uint8_t)209, (uint8_t)240, (uint8_t)37, (uint8_t)234, (uint8_t)130, (uint8_t)6, (uint8_t)21, (uint8_t)84, (uint8_t)130, (uint8_t)243, (uint8_t)12, (uint8_t)253, (uint8_t)132, (uint8_t)247, (uint8_t)97, (uint8_t)33, (uint8_t)22, (uint8_t)120, (uint8_t)61, (uint8_t)76, (uint8_t)113, (uint8_t)103, (uint8_t)66, (uint8_t)147, (uint8_t)156, (uint8_t)99, (uint8_t)243, (uint8_t)143, (uint8_t)89, (uint8_t)123, (uint8_t)165, (uint8_t)226, (uint8_t)85, (uint8_t)206, (uint8_t)88, (uint8_t)150, (uint8_t)40, (uint8_t)74, (uint8_t)31, (uint8_t)14, (uint8_t)0, (uint8_t)116, (uint8_t)129, (uint8_t)175, (uint8_t)110, (uint8_t)240, (uint8_t)218, (uint8_t)220, (uint8_t)170, (uint8_t)201, (uint8_t)114, (uint8_t)121, (uint8_t)108, (uint8_t)78, (uint8_t)7, (uint8_t)233, (uint8_t)218, (uint8_t)186, (uint8_t)98, (uint8_t)58, (uint8_t)8, (uint8_t)50, (uint8_t)60, (uint8_t)37, (uint8_t)123, (uint8_t)127, (uint8_t)171, (uint8_t)130, (uint8_t)230, (uint8_t)235, (uint8_t)236, (uint8_t)1, (uint8_t)13, (uint8_t)53, (uint8_t)76, (uint8_t)133, (uint8_t)231, (uint8_t)43, (uint8_t)166, (uint8_t)114, (uint8_t)91, (uint8_t)220, (uint8_t)191, (uint8_t)146, (uint8_t)56, (uint8_t)212, (uint8_t)161, (uint8_t)137, (uint8_t)123, (uint8_t)162, (uint8_t)105, (uint8_t)220, (uint8_t)155, (uint8_t)11, (uint8_t)58, (uint8_t)175, (uint8_t)41, (uint8_t)180, (uint8_t)148};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p248_target_network_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
        p248_target_system_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        p248_message_type_SET((uint16_t)(uint16_t)26226, PH.base.pack) ;
        c_LoopBackDemoChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MEMORY_VECT_249(), &PH);
        {
            int8_t value[] =  {(int8_t) -104, (int8_t) -44, (int8_t)105, (int8_t)64, (int8_t)69, (int8_t)48, (int8_t) -25, (int8_t)79, (int8_t) -45, (int8_t) -74, (int8_t)76, (int8_t) -39, (int8_t) -63, (int8_t) -31, (int8_t)31, (int8_t)55, (int8_t) -18, (int8_t)94, (int8_t)97, (int8_t) -89, (int8_t) -60, (int8_t) -23, (int8_t)90, (int8_t) -61, (int8_t) -57, (int8_t)77, (int8_t) -36, (int8_t)69, (int8_t) -119, (int8_t)10, (int8_t)45, (int8_t)90};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_type_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        p249_ver_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
        p249_address_SET((uint16_t)(uint16_t)13755, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEBUG_VECT_250(), &PH);
        p250_time_usec_SET((uint64_t)8444417496427771690L, PH.base.pack) ;
        p250_y_SET((float) -1.9186628E38F, PH.base.pack) ;
        {
            char16_t* name = u"rgotjwa";
            p250_name_SET_(name, &PH) ;
        }
        p250_x_SET((float)3.2966663E38F, PH.base.pack) ;
        p250_z_SET((float)1.6140032E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
        p251_value_SET((float) -1.61227E37F, PH.base.pack) ;
        p251_time_boot_ms_SET((uint32_t)943151648L, PH.base.pack) ;
        {
            char16_t* name = u"Ookj";
            p251_name_SET_(name, &PH) ;
        }
        c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_INT_252(), &PH);
        p252_value_SET((int32_t) -67626460, PH.base.pack) ;
        p252_time_boot_ms_SET((uint32_t)1278932014L, PH.base.pack) ;
        {
            char16_t* name = u"cpujHodw";
            p252_name_SET_(name, &PH) ;
        }
        c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STATUSTEXT_253(), &PH);
        {
            char16_t* text = u"xepPfityymscfAqcpwM";
            p253_text_SET_(text, &PH) ;
        }
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_NOTICE, PH.base.pack) ;
        c_LoopBackDemoChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEBUG_254(), &PH);
        p254_ind_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p254_time_boot_ms_SET((uint32_t)3429019146L, PH.base.pack) ;
        p254_value_SET((float) -1.2460224E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SETUP_SIGNING_256(), &PH);
        p256_initial_timestamp_SET((uint64_t)4830817049512321994L, PH.base.pack) ;
        p256_target_system_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)213, (uint8_t)213, (uint8_t)196, (uint8_t)218, (uint8_t)15, (uint8_t)224, (uint8_t)248, (uint8_t)74, (uint8_t)166, (uint8_t)57, (uint8_t)80, (uint8_t)18, (uint8_t)230, (uint8_t)140, (uint8_t)116, (uint8_t)50, (uint8_t)216, (uint8_t)252, (uint8_t)234, (uint8_t)37, (uint8_t)237, (uint8_t)215, (uint8_t)99, (uint8_t)120, (uint8_t)17, (uint8_t)248, (uint8_t)190, (uint8_t)190, (uint8_t)249, (uint8_t)243, (uint8_t)149, (uint8_t)63};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        p256_target_component_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BUTTON_CHANGE_257(), &PH);
        p257_state_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        p257_time_boot_ms_SET((uint32_t)581012751L, PH.base.pack) ;
        p257_last_change_ms_SET((uint32_t)3180213569L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PLAY_TUNE_258(), &PH);
        p258_target_component_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        {
            char16_t* tune = u"kcodxpbjgowiIzedTqkzoe";
            p258_tune_SET_(tune, &PH) ;
        }
        p258_target_system_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_INFORMATION_259(), &PH);
        p259_focal_length_SET((float) -7.86967E36F, PH.base.pack) ;
        p259_firmware_version_SET((uint32_t)2008976050L, PH.base.pack) ;
        p259_cam_definition_version_SET((uint16_t)(uint16_t)61822, PH.base.pack) ;
        p259_sensor_size_h_SET((float)4.5183016E36F, PH.base.pack) ;
        p259_resolution_h_SET((uint16_t)(uint16_t)7148, PH.base.pack) ;
        p259_resolution_v_SET((uint16_t)(uint16_t)64762, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)88, (uint8_t)109, (uint8_t)203, (uint8_t)204, (uint8_t)4, (uint8_t)63, (uint8_t)124, (uint8_t)108, (uint8_t)90, (uint8_t)252, (uint8_t)171, (uint8_t)143, (uint8_t)218, (uint8_t)157, (uint8_t)187, (uint8_t)46, (uint8_t)87, (uint8_t)13, (uint8_t)103, (uint8_t)241, (uint8_t)39, (uint8_t)245, (uint8_t)241, (uint8_t)128, (uint8_t)175, (uint8_t)202, (uint8_t)129, (uint8_t)194, (uint8_t)195, (uint8_t)74, (uint8_t)206, (uint8_t)173};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE, PH.base.pack) ;
        p259_lens_id_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        p259_sensor_size_v_SET((float) -2.7029937E38F, PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)95, (uint8_t)159, (uint8_t)182, (uint8_t)18, (uint8_t)20, (uint8_t)216, (uint8_t)158, (uint8_t)218, (uint8_t)210, (uint8_t)42, (uint8_t)246, (uint8_t)58, (uint8_t)72, (uint8_t)51, (uint8_t)130, (uint8_t)225, (uint8_t)147, (uint8_t)177, (uint8_t)51, (uint8_t)252, (uint8_t)225, (uint8_t)193, (uint8_t)209, (uint8_t)65, (uint8_t)11, (uint8_t)215, (uint8_t)230, (uint8_t)228, (uint8_t)150, (uint8_t)54, (uint8_t)203, (uint8_t)175};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        {
            char16_t* cam_definition_uri = u"bqaLtysquffApftVWkfopzmhiarutwxUkiarvbcEvgVejtbjecB";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_time_boot_ms_SET((uint32_t)3041948288L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_SETTINGS_260(), &PH);
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_VIDEO, PH.base.pack) ;
        p260_time_boot_ms_SET((uint32_t)3582678275L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STORAGE_INFORMATION_261(), &PH);
        p261_write_speed_SET((float)2.7982332E38F, PH.base.pack) ;
        p261_read_speed_SET((float)1.1372106E38F, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        p261_used_capacity_SET((float) -1.738584E38F, PH.base.pack) ;
        p261_total_capacity_SET((float) -2.0407491E38F, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)3886475785L, PH.base.pack) ;
        p261_available_capacity_SET((float)2.163062E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_image_status_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        p262_image_interval_SET((float) -2.8617732E38F, PH.base.pack) ;
        p262_recording_time_ms_SET((uint32_t)115010151L, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)3335658806L, PH.base.pack) ;
        p262_available_capacity_SET((float) -1.5141892E38F, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_lon_SET((int32_t)908280405, PH.base.pack) ;
        p263_camera_id_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        p263_time_utc_SET((uint64_t)2630662737987732657L, PH.base.pack) ;
        p263_capture_result_SET((int8_t)(int8_t)68, PH.base.pack) ;
        p263_lat_SET((int32_t)2098175055, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)1667410008L, PH.base.pack) ;
        p263_alt_SET((int32_t) -1726351602, PH.base.pack) ;
        {
            float q[] =  {-1.4205667E38F, 2.7128394E38F, 3.390555E38F, 1.292218E38F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        {
            char16_t* file_url = u"xoztysfrmzjtsjovmkceeUsrclqyjsjhgSlqwhzzoyknruasnDlFzaorqpLflazdckruPoUklxdwurlxsoyjrjzcdpnzgqhgjbweuv";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_relative_alt_SET((int32_t) -1912880356, PH.base.pack) ;
        p263_image_index_SET((int32_t) -1529274944, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_arming_time_utc_SET((uint64_t)2025762840362005073L, PH.base.pack) ;
        p264_time_boot_ms_SET((uint32_t)3554170764L, PH.base.pack) ;
        p264_takeoff_time_utc_SET((uint64_t)1658814051001568727L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)3609340950990879452L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_pitch_SET((float) -2.2581035E38F, PH.base.pack) ;
        p265_roll_SET((float) -2.9459328E38F, PH.base.pack) ;
        p265_time_boot_ms_SET((uint32_t)1239268293L, PH.base.pack) ;
        p265_yaw_SET((float)1.4478411E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_266(), &PH);
        p266_length_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        p266_target_system_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)215, (uint8_t)215, (uint8_t)7, (uint8_t)16, (uint8_t)179, (uint8_t)237, (uint8_t)31, (uint8_t)208, (uint8_t)58, (uint8_t)170, (uint8_t)189, (uint8_t)114, (uint8_t)149, (uint8_t)34, (uint8_t)14, (uint8_t)21, (uint8_t)90, (uint8_t)74, (uint8_t)152, (uint8_t)196, (uint8_t)59, (uint8_t)226, (uint8_t)246, (uint8_t)0, (uint8_t)230, (uint8_t)47, (uint8_t)80, (uint8_t)11, (uint8_t)7, (uint8_t)106, (uint8_t)138, (uint8_t)109, (uint8_t)67, (uint8_t)186, (uint8_t)90, (uint8_t)73, (uint8_t)134, (uint8_t)34, (uint8_t)62, (uint8_t)75, (uint8_t)250, (uint8_t)49, (uint8_t)55, (uint8_t)90, (uint8_t)27, (uint8_t)207, (uint8_t)179, (uint8_t)246, (uint8_t)100, (uint8_t)34, (uint8_t)177, (uint8_t)47, (uint8_t)146, (uint8_t)12, (uint8_t)155, (uint8_t)91, (uint8_t)177, (uint8_t)22, (uint8_t)96, (uint8_t)120, (uint8_t)209, (uint8_t)106, (uint8_t)12, (uint8_t)209, (uint8_t)239, (uint8_t)8, (uint8_t)58, (uint8_t)244, (uint8_t)60, (uint8_t)72, (uint8_t)36, (uint8_t)27, (uint8_t)232, (uint8_t)148, (uint8_t)115, (uint8_t)171, (uint8_t)218, (uint8_t)143, (uint8_t)89, (uint8_t)8, (uint8_t)27, (uint8_t)22, (uint8_t)235, (uint8_t)86, (uint8_t)27, (uint8_t)210, (uint8_t)94, (uint8_t)82, (uint8_t)207, (uint8_t)42, (uint8_t)181, (uint8_t)34, (uint8_t)170, (uint8_t)12, (uint8_t)116, (uint8_t)9, (uint8_t)170, (uint8_t)169, (uint8_t)203, (uint8_t)179, (uint8_t)73, (uint8_t)94, (uint8_t)226, (uint8_t)54, (uint8_t)147, (uint8_t)8, (uint8_t)201, (uint8_t)252, (uint8_t)192, (uint8_t)207, (uint8_t)155, (uint8_t)73, (uint8_t)184, (uint8_t)41, (uint8_t)215, (uint8_t)110, (uint8_t)33, (uint8_t)254, (uint8_t)235, (uint8_t)13, (uint8_t)157, (uint8_t)129, (uint8_t)115, (uint8_t)63, (uint8_t)126, (uint8_t)106, (uint8_t)243, (uint8_t)178, (uint8_t)179, (uint8_t)161, (uint8_t)6, (uint8_t)74, (uint8_t)68, (uint8_t)113, (uint8_t)91, (uint8_t)115, (uint8_t)50, (uint8_t)110, (uint8_t)81, (uint8_t)101, (uint8_t)166, (uint8_t)230, (uint8_t)30, (uint8_t)250, (uint8_t)18, (uint8_t)112, (uint8_t)192, (uint8_t)237, (uint8_t)55, (uint8_t)162, (uint8_t)185, (uint8_t)37, (uint8_t)249, (uint8_t)109, (uint8_t)3, (uint8_t)235, (uint8_t)157, (uint8_t)234, (uint8_t)4, (uint8_t)204, (uint8_t)45, (uint8_t)17, (uint8_t)229, (uint8_t)117, (uint8_t)134, (uint8_t)98, (uint8_t)46, (uint8_t)175, (uint8_t)116, (uint8_t)247, (uint8_t)78, (uint8_t)40, (uint8_t)155, (uint8_t)105, (uint8_t)57, (uint8_t)191, (uint8_t)206, (uint8_t)252, (uint8_t)146, (uint8_t)60, (uint8_t)238, (uint8_t)186, (uint8_t)225, (uint8_t)20, (uint8_t)114, (uint8_t)24, (uint8_t)33, (uint8_t)169, (uint8_t)166, (uint8_t)162, (uint8_t)63, (uint8_t)168, (uint8_t)46, (uint8_t)182, (uint8_t)129, (uint8_t)198, (uint8_t)30, (uint8_t)173, (uint8_t)66, (uint8_t)109, (uint8_t)245, (uint8_t)182, (uint8_t)97, (uint8_t)11, (uint8_t)120, (uint8_t)215, (uint8_t)20, (uint8_t)164, (uint8_t)49, (uint8_t)40, (uint8_t)36, (uint8_t)249, (uint8_t)89, (uint8_t)98, (uint8_t)198, (uint8_t)216, (uint8_t)161, (uint8_t)35, (uint8_t)66, (uint8_t)122, (uint8_t)165, (uint8_t)140, (uint8_t)28, (uint8_t)240, (uint8_t)70, (uint8_t)216, (uint8_t)78, (uint8_t)71, (uint8_t)25, (uint8_t)174, (uint8_t)25, (uint8_t)239, (uint8_t)25, (uint8_t)195, (uint8_t)202, (uint8_t)200, (uint8_t)132, (uint8_t)222, (uint8_t)42, (uint8_t)108, (uint8_t)27, (uint8_t)107, (uint8_t)234, (uint8_t)31, (uint8_t)149, (uint8_t)52, (uint8_t)245, (uint8_t)236, (uint8_t)196};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_sequence_SET((uint16_t)(uint16_t)41867, PH.base.pack) ;
        p266_first_message_offset_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_ACKED_267(), &PH);
        p267_length_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        p267_target_component_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        p267_target_system_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)135, (uint8_t)1, (uint8_t)0, (uint8_t)205, (uint8_t)154, (uint8_t)128, (uint8_t)136, (uint8_t)20, (uint8_t)251, (uint8_t)160, (uint8_t)19, (uint8_t)20, (uint8_t)164, (uint8_t)138, (uint8_t)64, (uint8_t)152, (uint8_t)133, (uint8_t)9, (uint8_t)198, (uint8_t)93, (uint8_t)138, (uint8_t)180, (uint8_t)69, (uint8_t)83, (uint8_t)22, (uint8_t)174, (uint8_t)136, (uint8_t)250, (uint8_t)167, (uint8_t)255, (uint8_t)241, (uint8_t)237, (uint8_t)23, (uint8_t)137, (uint8_t)201, (uint8_t)150, (uint8_t)140, (uint8_t)28, (uint8_t)231, (uint8_t)223, (uint8_t)241, (uint8_t)238, (uint8_t)230, (uint8_t)20, (uint8_t)17, (uint8_t)249, (uint8_t)245, (uint8_t)135, (uint8_t)107, (uint8_t)90, (uint8_t)42, (uint8_t)47, (uint8_t)5, (uint8_t)148, (uint8_t)204, (uint8_t)179, (uint8_t)18, (uint8_t)160, (uint8_t)232, (uint8_t)207, (uint8_t)52, (uint8_t)39, (uint8_t)94, (uint8_t)59, (uint8_t)175, (uint8_t)216, (uint8_t)239, (uint8_t)79, (uint8_t)226, (uint8_t)33, (uint8_t)67, (uint8_t)139, (uint8_t)62, (uint8_t)92, (uint8_t)202, (uint8_t)155, (uint8_t)150, (uint8_t)50, (uint8_t)196, (uint8_t)71, (uint8_t)220, (uint8_t)210, (uint8_t)163, (uint8_t)157, (uint8_t)244, (uint8_t)42, (uint8_t)87, (uint8_t)170, (uint8_t)113, (uint8_t)0, (uint8_t)241, (uint8_t)215, (uint8_t)139, (uint8_t)91, (uint8_t)143, (uint8_t)1, (uint8_t)103, (uint8_t)116, (uint8_t)62, (uint8_t)162, (uint8_t)182, (uint8_t)44, (uint8_t)255, (uint8_t)70, (uint8_t)220, (uint8_t)138, (uint8_t)88, (uint8_t)180, (uint8_t)77, (uint8_t)240, (uint8_t)165, (uint8_t)216, (uint8_t)118, (uint8_t)122, (uint8_t)113, (uint8_t)224, (uint8_t)42, (uint8_t)147, (uint8_t)97, (uint8_t)48, (uint8_t)145, (uint8_t)11, (uint8_t)69, (uint8_t)154, (uint8_t)251, (uint8_t)126, (uint8_t)226, (uint8_t)190, (uint8_t)203, (uint8_t)172, (uint8_t)107, (uint8_t)221, (uint8_t)182, (uint8_t)226, (uint8_t)241, (uint8_t)52, (uint8_t)38, (uint8_t)19, (uint8_t)85, (uint8_t)66, (uint8_t)11, (uint8_t)162, (uint8_t)60, (uint8_t)21, (uint8_t)150, (uint8_t)147, (uint8_t)72, (uint8_t)98, (uint8_t)95, (uint8_t)142, (uint8_t)22, (uint8_t)65, (uint8_t)229, (uint8_t)131, (uint8_t)78, (uint8_t)126, (uint8_t)187, (uint8_t)140, (uint8_t)250, (uint8_t)100, (uint8_t)66, (uint8_t)108, (uint8_t)102, (uint8_t)173, (uint8_t)17, (uint8_t)62, (uint8_t)230, (uint8_t)230, (uint8_t)55, (uint8_t)162, (uint8_t)166, (uint8_t)242, (uint8_t)26, (uint8_t)229, (uint8_t)203, (uint8_t)128, (uint8_t)120, (uint8_t)16, (uint8_t)237, (uint8_t)149, (uint8_t)188, (uint8_t)236, (uint8_t)225, (uint8_t)157, (uint8_t)78, (uint8_t)232, (uint8_t)196, (uint8_t)85, (uint8_t)90, (uint8_t)75, (uint8_t)117, (uint8_t)100, (uint8_t)242, (uint8_t)58, (uint8_t)224, (uint8_t)185, (uint8_t)39, (uint8_t)45, (uint8_t)255, (uint8_t)93, (uint8_t)87, (uint8_t)103, (uint8_t)238, (uint8_t)54, (uint8_t)237, (uint8_t)195, (uint8_t)32, (uint8_t)1, (uint8_t)36, (uint8_t)147, (uint8_t)217, (uint8_t)189, (uint8_t)140, (uint8_t)73, (uint8_t)28, (uint8_t)207, (uint8_t)162, (uint8_t)226, (uint8_t)111, (uint8_t)183, (uint8_t)37, (uint8_t)41, (uint8_t)97, (uint8_t)43, (uint8_t)160, (uint8_t)171, (uint8_t)90, (uint8_t)128, (uint8_t)216, (uint8_t)235, (uint8_t)222, (uint8_t)150, (uint8_t)74, (uint8_t)90, (uint8_t)79, (uint8_t)99, (uint8_t)29, (uint8_t)100, (uint8_t)113, (uint8_t)35, (uint8_t)171, (uint8_t)131, (uint8_t)73, (uint8_t)148, (uint8_t)67, (uint8_t)224, (uint8_t)39, (uint8_t)132, (uint8_t)114};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_first_message_offset_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        p267_sequence_SET((uint16_t)(uint16_t)58062, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_ACK_268(), &PH);
        p268_target_system_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
        p268_target_component_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p268_sequence_SET((uint16_t)(uint16_t)48209, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_rotation_SET((uint16_t)(uint16_t)23910, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)42767, PH.base.pack) ;
        p269_bitrate_SET((uint32_t)3518055541L, PH.base.pack) ;
        {
            char16_t* uri = u"qnetjvpiurvrcaqtFemrEkifhctvkClvgucqwikxpspsizvrlmckfUdtksxhydlBjmbmgj";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_camera_id_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p269_framerate_SET((float) -1.892122E38F, PH.base.pack) ;
        p269_status_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)15223, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_target_component_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        p270_framerate_SET((float)3.0801269E38F, PH.base.pack) ;
        p270_rotation_SET((uint16_t)(uint16_t)32635, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)55641, PH.base.pack) ;
        p270_resolution_h_SET((uint16_t)(uint16_t)58576, PH.base.pack) ;
        {
            char16_t* uri = u"zqtjgrmspZdctrqdkduhjglehsgozrlnPdgnecgpceaAmxudmwxmpdreCbybzqtubpzolomsrUeekTExkmywvaurYmnhhyfkeTyqXwxEriejuAlznxbggtyKtjubNzsratjwextheeydWwywOtchcCrduxwwqqcZedhcciosLRJghdiCygmwlegipkiJskuplqmyRnotmksaqfeb";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_target_system_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)1787464869L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* password = u"ztunbtuaMhcznpTjgdb";
            p299_password_SET_(password, &PH) ;
        }
        {
            char16_t* ssid = u"kPmunnjFxnqvpdqzueEljkqmrn";
            p299_ssid_SET_(ssid, &PH) ;
        }
        c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PROTOCOL_VERSION_300(), &PH);
        {
            uint8_t library_version_hash[] =  {(uint8_t)223, (uint8_t)55, (uint8_t)175, (uint8_t)131, (uint8_t)112, (uint8_t)29, (uint8_t)165, (uint8_t)21};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        p300_min_version_SET((uint16_t)(uint16_t)27937, PH.base.pack) ;
        p300_version_SET((uint16_t)(uint16_t)19682, PH.base.pack) ;
        {
            uint8_t spec_version_hash[] =  {(uint8_t)82, (uint8_t)17, (uint8_t)215, (uint8_t)216, (uint8_t)174, (uint8_t)46, (uint8_t)186, (uint8_t)187};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        p300_max_version_SET((uint16_t)(uint16_t)58834, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_time_usec_SET((uint64_t)1229122492311986474L, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_OPERATIONAL, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)1066611158L, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)42975, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_sw_version_minor_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        p311_hw_version_major_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)3892334352L, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)214, (uint8_t)99, (uint8_t)24, (uint8_t)144, (uint8_t)217, (uint8_t)8, (uint8_t)254, (uint8_t)123, (uint8_t)94, (uint8_t)228, (uint8_t)254, (uint8_t)97, (uint8_t)14, (uint8_t)95, (uint8_t)49, (uint8_t)197};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        p311_sw_version_major_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        {
            char16_t* name = u"rqhitxxtRf";
            p311_name_SET_(name, &PH) ;
        }
        p311_uptime_sec_SET((uint32_t)2322642673L, PH.base.pack) ;
        p311_hw_version_minor_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
        p311_time_usec_SET((uint64_t)5806495681950907237L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        p320_param_index_SET((int16_t)(int16_t) -26255, PH.base.pack) ;
        p320_target_component_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        {
            char16_t* param_id = u"kivpdmXCkom";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_target_system_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_component_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        p321_target_system_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_VALUE_322(), &PH);
        p322_param_index_SET((uint16_t)(uint16_t)40063, PH.base.pack) ;
        {
            char16_t* param_id = u"mdqtfubpv";
            p322_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"jxfjhwdOwjhdohDxjreglaXxAcjobmZkallpybcj";
            p322_param_value_SET_(param_value, &PH) ;
        }
        p322_param_count_SET((uint16_t)(uint16_t)62059, PH.base.pack) ;
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT64, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_SET_323(), &PH);
        {
            char16_t* param_value = u"zdoh";
            p323_param_value_SET_(param_value, &PH) ;
        }
        {
            char16_t* param_id = u"wsc";
            p323_param_id_SET_(param_id, &PH) ;
        }
        p323_target_system_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p323_target_component_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_ACK_324(), &PH);
        {
            char16_t* param_value = u"dditnzLpd";
            p324_param_value_SET_(param_value, &PH) ;
        }
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_FAILED, PH.base.pack) ;
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32, PH.base.pack) ;
        {
            char16_t* param_id = u"TuDQohrVlmc";
            p324_param_id_SET_(param_id, &PH) ;
        }
        c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OBSTACLE_DISTANCE_330(), &PH);
        p330_time_usec_SET((uint64_t)7745215932703127157L, PH.base.pack) ;
        p330_max_distance_SET((uint16_t)(uint16_t)53560, PH.base.pack) ;
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_LASER, PH.base.pack) ;
        p330_min_distance_SET((uint16_t)(uint16_t)30479, PH.base.pack) ;
        {
            uint16_t distances[] =  {(uint16_t)19084, (uint16_t)20831, (uint16_t)13321, (uint16_t)23501, (uint16_t)27876, (uint16_t)63557, (uint16_t)5902, (uint16_t)64843, (uint16_t)3163, (uint16_t)46570, (uint16_t)21066, (uint16_t)35535, (uint16_t)7921, (uint16_t)28506, (uint16_t)1925, (uint16_t)65092, (uint16_t)1542, (uint16_t)4204, (uint16_t)37576, (uint16_t)60103, (uint16_t)36452, (uint16_t)10448, (uint16_t)15869, (uint16_t)11869, (uint16_t)43007, (uint16_t)12628, (uint16_t)20022, (uint16_t)46690, (uint16_t)39755, (uint16_t)21009, (uint16_t)40977, (uint16_t)29542, (uint16_t)338, (uint16_t)34630, (uint16_t)17236, (uint16_t)8526, (uint16_t)53293, (uint16_t)65226, (uint16_t)1315, (uint16_t)62574, (uint16_t)51596, (uint16_t)63158, (uint16_t)60628, (uint16_t)61508, (uint16_t)10092, (uint16_t)40377, (uint16_t)932, (uint16_t)23084, (uint16_t)4273, (uint16_t)35990, (uint16_t)18731, (uint16_t)48522, (uint16_t)3191, (uint16_t)27772, (uint16_t)27119, (uint16_t)16034, (uint16_t)47129, (uint16_t)46378, (uint16_t)50501, (uint16_t)30548, (uint16_t)11491, (uint16_t)55455, (uint16_t)4135, (uint16_t)4563, (uint16_t)34911, (uint16_t)9885, (uint16_t)45780, (uint16_t)65101, (uint16_t)9117, (uint16_t)50065, (uint16_t)9987, (uint16_t)53710};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_increment_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
}

