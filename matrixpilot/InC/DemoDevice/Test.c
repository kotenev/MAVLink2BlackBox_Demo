
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
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_PARAFOIL);
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p0_custom_mode_GET(pack) == (uint32_t)608911116L);
    assert(p0_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED);
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_ACTIVE);
};


void c_LoopBackDemoChannel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)56848);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)22837);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)20237);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)61593);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)35114);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)50544);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)34420);
    assert(p1_onboard_control_sensors_present_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t) -8982);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)32020);
    assert(p1_onboard_control_sensors_health_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2);
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t) -72);
};


void c_LoopBackDemoChannel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)2091648250L);
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)2043441244541234898L);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)53046);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)1174442147L);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p3_afz_GET(pack) == (float) -1.6588216E38F);
    assert(p3_x_GET(pack) == (float) -2.934203E38F);
    assert(p3_yaw_rate_GET(pack) == (float)1.7318629E37F);
    assert(p3_z_GET(pack) == (float)1.987928E38F);
    assert(p3_y_GET(pack) == (float) -2.0777373E38F);
    assert(p3_yaw_GET(pack) == (float)1.2048585E38F);
    assert(p3_afx_GET(pack) == (float) -2.1753657E38F);
    assert(p3_vy_GET(pack) == (float)3.5381251E37F);
    assert(p3_vx_GET(pack) == (float)7.186914E36F);
    assert(p3_vz_GET(pack) == (float)2.566778E37F);
    assert(p3_afy_GET(pack) == (float)1.5738301E38F);
};


void c_LoopBackDemoChannel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_time_usec_GET(pack) == (uint64_t)2596471207447125292L);
    assert(p4_seq_GET(pack) == (uint32_t)36322742L);
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)254);
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)185);
    assert(p5_passkey_LEN(ph) == 20);
    {
        char16_t * exemplary = u"zetmppgvwxnbKucpaqtn";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 40);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)246);
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)209);
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)93);
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)32);
};


void c_LoopBackDemoChannel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 14);
    {
        char16_t * exemplary = u"EvwNuzapksrqhb";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 28);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_TEST_DISARMED);
    assert(p11_custom_mode_GET(pack) == (uint32_t)2864409613L);
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)14);
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)214);
    assert(p20_param_id_LEN(ph) == 14);
    {
        char16_t * exemplary = u"bxoecsjIuqfdug";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 28);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t) -17605);
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)208);
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)77);
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)223);
};


void c_LoopBackDemoChannel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)63383);
    assert(p22_param_value_GET(pack) == (float)2.1476496E38F);
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)1499);
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64);
    assert(p22_param_id_LEN(ph) == 14);
    {
        char16_t * exemplary = u"geieifjvdpkhaw";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 28);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64);
    assert(p23_param_value_GET(pack) == (float)2.917324E38F);
    assert(p23_param_id_LEN(ph) == 5);
    {
        char16_t * exemplary = u"oorsn";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)229);
};


void c_LoopBackDemoChannel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)18580);
    assert(p24_h_acc_TRY(ph) == (uint32_t)2473481522L);
    assert(p24_time_usec_GET(pack) == (uint64_t)2223950332722576602L);
    assert(p24_lat_GET(pack) == (int32_t)188124256);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)10);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t)1295129396);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)48465);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX);
    assert(p24_v_acc_TRY(ph) == (uint32_t)1414022041L);
    assert(p24_lon_GET(pack) == (int32_t) -1619661240);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)19086);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)50420);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)2764756066L);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)4062890655L);
    assert(p24_alt_GET(pack) == (int32_t)88586637);
};


void c_LoopBackDemoChannel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)12);
    {
        uint8_t exemplary[] =  {(uint8_t)14, (uint8_t)114, (uint8_t)17, (uint8_t)191, (uint8_t)101, (uint8_t)197, (uint8_t)206, (uint8_t)232, (uint8_t)4, (uint8_t)117, (uint8_t)235, (uint8_t)78, (uint8_t)132, (uint8_t)61, (uint8_t)155, (uint8_t)70, (uint8_t)70, (uint8_t)174, (uint8_t)31, (uint8_t)49} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)4, (uint8_t)225, (uint8_t)175, (uint8_t)133, (uint8_t)211, (uint8_t)213, (uint8_t)178, (uint8_t)140, (uint8_t)0, (uint8_t)8, (uint8_t)9, (uint8_t)51, (uint8_t)162, (uint8_t)208, (uint8_t)219, (uint8_t)99, (uint8_t)210, (uint8_t)218, (uint8_t)177, (uint8_t)50} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)231, (uint8_t)76, (uint8_t)245, (uint8_t)121, (uint8_t)79, (uint8_t)242, (uint8_t)125, (uint8_t)65, (uint8_t)17, (uint8_t)187, (uint8_t)231, (uint8_t)160, (uint8_t)133, (uint8_t)62, (uint8_t)135, (uint8_t)188, (uint8_t)128, (uint8_t)197, (uint8_t)207, (uint8_t)80} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)107, (uint8_t)64, (uint8_t)41, (uint8_t)13, (uint8_t)102, (uint8_t)131, (uint8_t)185, (uint8_t)225, (uint8_t)79, (uint8_t)131, (uint8_t)155, (uint8_t)60, (uint8_t)165, (uint8_t)60, (uint8_t)157, (uint8_t)166, (uint8_t)44, (uint8_t)179, (uint8_t)214, (uint8_t)226} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)47, (uint8_t)177, (uint8_t)146, (uint8_t)69, (uint8_t)33, (uint8_t)166, (uint8_t)72, (uint8_t)17, (uint8_t)156, (uint8_t)20, (uint8_t)125, (uint8_t)40, (uint8_t)17, (uint8_t)65, (uint8_t)1, (uint8_t)58, (uint8_t)117, (uint8_t)147, (uint8_t)15, (uint8_t)96} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t)15907);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t) -1956);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t)5009);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t)4263);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t) -6728);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t)22726);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t) -27648);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t) -3020);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)3775875017L);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t) -516);
};


void c_LoopBackDemoChannel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t)30790);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t) -21084);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t) -9462);
    assert(p27_time_usec_GET(pack) == (uint64_t)1880333694408336652L);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t) -9683);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t)9415);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t) -3678);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t) -27748);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t)19236);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t)22253);
};


void c_LoopBackDemoChannel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t)28547);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t) -475);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t) -11047);
    assert(p28_time_usec_GET(pack) == (uint64_t)5265318711483516121L);
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t)17752);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_press_abs_GET(pack) == (float)2.751141E38F);
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)1231281807L);
    assert(p29_press_diff_GET(pack) == (float) -1.6788992E38F);
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t)20771);
};


void c_LoopBackDemoChannel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_rollspeed_GET(pack) == (float)3.1747585E38F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)3208008904L);
    assert(p30_yaw_GET(pack) == (float)5.7502487E37F);
    assert(p30_pitch_GET(pack) == (float) -2.351709E38F);
    assert(p30_pitchspeed_GET(pack) == (float)9.057269E37F);
    assert(p30_yawspeed_GET(pack) == (float)2.832218E38F);
    assert(p30_roll_GET(pack) == (float) -1.1826288E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_pitchspeed_GET(pack) == (float) -5.5469267E37F);
    assert(p31_rollspeed_GET(pack) == (float) -1.1518879E38F);
    assert(p31_yawspeed_GET(pack) == (float) -1.097817E38F);
    assert(p31_q3_GET(pack) == (float) -1.9085723E38F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)2078463354L);
    assert(p31_q4_GET(pack) == (float)1.3455929E38F);
    assert(p31_q1_GET(pack) == (float) -1.728796E37F);
    assert(p31_q2_GET(pack) == (float) -9.304058E37F);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)389510108L);
    assert(p32_z_GET(pack) == (float) -2.2783263E38F);
    assert(p32_vz_GET(pack) == (float)1.3089776E38F);
    assert(p32_y_GET(pack) == (float)1.8874818E38F);
    assert(p32_x_GET(pack) == (float)4.563876E37F);
    assert(p32_vx_GET(pack) == (float) -1.5412577E38F);
    assert(p32_vy_GET(pack) == (float)2.0742216E37F);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_vz_GET(pack) == (int16_t)(int16_t) -12199);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t) -15935);
    assert(p33_alt_GET(pack) == (int32_t)378103104);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)1498510014L);
    assert(p33_relative_alt_GET(pack) == (int32_t)908964739);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)13548);
    assert(p33_lon_GET(pack) == (int32_t) -1751877285);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t)18767);
    assert(p33_lat_GET(pack) == (int32_t)1460670613);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t)22507);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t)4235);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)110);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t)12550);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t)31209);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t) -14409);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t) -22562);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)1761612416L);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t) -28088);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t) -15233);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)50879);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)231);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)22687);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)4086467756L);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)53947);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)54570);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)51874);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)31109);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)4300);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)16542);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)222);
};


void c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)210);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)11025);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)12755);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)30895);
    assert(p36_time_usec_GET(pack) == (uint32_t)4193802934L);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)44761);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)32752);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)27026);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)23067);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)30269);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)14378);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)53778);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)39725);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)13241);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)11420);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)56480);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)23639);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)60249);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t)2845);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)133);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t) -1262);
};


void c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)111);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t) -20222);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t) -25399);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_x_GET(pack) == (float)1.4821053E38F);
    assert(p39_param2_GET(pack) == (float)7.164766E37F);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)193);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p39_z_GET(pack) == (float)1.4609133E38F);
    assert(p39_y_GET(pack) == (float) -2.677653E38F);
    assert(p39_param1_GET(pack) == (float)3.1403156E37F);
    assert(p39_param3_GET(pack) == (float) -1.466763E38F);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)209);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p39_param4_GET(pack) == (float)1.2415398E38F);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)175);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)16572);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)166);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)248);
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)57812);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
};


void c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)64);
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)4118);
};


void c_LoopBackDemoChannel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)37617);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)5);
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)112);
};


void c_LoopBackDemoChannel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)22609);
};


void c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)50);
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)39);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)4262);
};


void c_LoopBackDemoChannel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)62);
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_SEQUENCE);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
};


void c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_altitude_GET(pack) == (int32_t)1447101445);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)49);
    assert(p48_longitude_GET(pack) == (int32_t) -96419761);
    assert(p48_latitude_GET(pack) == (int32_t)246494998);
    assert(p48_time_usec_TRY(ph) == (uint64_t)1689186273279456461L);
};


void c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_time_usec_TRY(ph) == (uint64_t)7772308075405123271L);
    assert(p49_altitude_GET(pack) == (int32_t)1388924462);
    assert(p49_longitude_GET(pack) == (int32_t)1677571725);
    assert(p49_latitude_GET(pack) == (int32_t) -894338627);
};


void c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_param_value_max_GET(pack) == (float)1.2889844E38F);
    assert(p50_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"x";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)53);
    assert(p50_scale_GET(pack) == (float) -3.1657284E38F);
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t)2759);
    assert(p50_param_value0_GET(pack) == (float)2.8810404E38F);
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)57);
    assert(p50_param_value_min_GET(pack) == (float) -3.1935595E38F);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)240);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)16437);
};


void c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p54_p2x_GET(pack) == (float) -1.7703116E38F);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)20);
    assert(p54_p1y_GET(pack) == (float) -6.8910587E37F);
    assert(p54_p2z_GET(pack) == (float)1.592504E38F);
    assert(p54_p1z_GET(pack) == (float) -6.921723E37F);
    assert(p54_p2y_GET(pack) == (float)1.1362559E38F);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)150);
    assert(p54_p1x_GET(pack) == (float) -1.476E38F);
};


void c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p1x_GET(pack) == (float) -5.2829653E37F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_NED);
    assert(p55_p2z_GET(pack) == (float)3.0054541E38F);
    assert(p55_p2y_GET(pack) == (float)2.7170334E38F);
    assert(p55_p1y_GET(pack) == (float)1.02418816E37F);
    assert(p55_p1z_GET(pack) == (float)2.2613587E38F);
    assert(p55_p2x_GET(pack) == (float) -2.2251803E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-1.8411104E38F, 1.2576557E38F, 7.358117E35F, 1.7797652E38F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_rollspeed_GET(pack) == (float) -2.5533773E38F);
    assert(p61_yawspeed_GET(pack) == (float) -2.3712817E37F);
    {
        float exemplary[] =  {-2.1757858E38F, -2.2325838E38F, -2.0609202E38F, -1.3545536E38F, -2.6835516E38F, 1.8713276E37F, -3.275008E37F, -3.2909019E38F, 1.0279843E38F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_time_usec_GET(pack) == (uint64_t)2027238448247776551L);
    assert(p61_pitchspeed_GET(pack) == (float)2.2948181E38F);
};


void c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_alt_error_GET(pack) == (float)1.8675964E38F);
    assert(p62_xtrack_error_GET(pack) == (float) -5.558497E37F);
    assert(p62_aspd_error_GET(pack) == (float)2.975827E38F);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)60841);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t) -15433);
    assert(p62_nav_roll_GET(pack) == (float) -5.1804996E37F);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t)6327);
    assert(p62_nav_pitch_GET(pack) == (float) -2.5081376E38F);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_time_usec_GET(pack) == (uint64_t)439871212851646840L);
    assert(p63_vy_GET(pack) == (float)3.2711872E38F);
    assert(p63_vx_GET(pack) == (float) -1.4760363E37F);
    assert(p63_alt_GET(pack) == (int32_t)520924949);
    {
        float exemplary[] =  {-1.3137368E38F, -5.8552185E36F, 2.5839116E38F, -2.8346126E38F, -1.9321891E38F, -2.4013531E38F, -2.9720058E37F, 1.5450868E37F, -1.5599742E38F, 1.2197836E38F, 1.0432707E38F, -4.9605906E37F, -1.1265894E38F, 2.825135E38F, 1.910001E35F, 1.9217301E38F, 2.3110595E38F, 3.001742E38F, -6.157229E36F, 2.086521E38F, 1.3769842E38F, -1.0125496E38F, 5.1744336E37F, 2.196297E38F, 9.703999E37F, -1.0977377E38F, 2.256481E38F, 1.5228168E38F, -1.6072909E38F, 2.2262285E38F, -3.128197E38F, 1.0950184E38F, -2.9132974E38F, -1.5105492E37F, -3.6947274E37F, -3.1630765E38F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_lat_GET(pack) == (int32_t)507971145);
    assert(p63_relative_alt_GET(pack) == (int32_t)1725956373);
    assert(p63_lon_GET(pack) == (int32_t)1121004261);
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS);
    assert(p63_vz_GET(pack) == (float) -2.3574127E38F);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_time_usec_GET(pack) == (uint64_t)6616303826973665248L);
    assert(p64_vz_GET(pack) == (float)2.3255182E38F);
    assert(p64_vx_GET(pack) == (float)1.4142584E38F);
    assert(p64_ax_GET(pack) == (float) -1.9848853E38F);
    assert(p64_ay_GET(pack) == (float) -8.029553E37F);
    assert(p64_az_GET(pack) == (float)2.1205643E38F);
    assert(p64_x_GET(pack) == (float)1.8095692E38F);
    {
        float exemplary[] =  {1.4543427E38F, 9.513874E36F, -2.912348E38F, 1.7087697E38F, 3.0348417E37F, -1.4724606E38F, 1.3101637E38F, 1.2357915E37F, 1.2265813E38F, -2.8239177E38F, 2.252982E38F, -1.5035562E38F, 2.9943759E38F, 2.5908338E38F, 2.0578707E38F, -1.5149244E38F, -6.21007E37F, -3.2837536E38F, -1.1982878E38F, 1.5744983E38F, 1.196918E38F, 1.7094037E38F, -1.6328402E37F, -1.0454476E38F, 2.3091032E38F, 2.6607824E38F, 1.2197965E38F, -1.3083888E38F, 6.847789E37F, -2.927342E37F, 2.9716073E38F, 2.203803E38F, -3.1905731E38F, -5.8663503E37F, -3.664734E37F, -2.252716E38F, -5.255211E37F, 1.3913266E38F, -1.02506804E37F, -3.377416E38F, -5.371793E37F, 2.9412893E38F, -1.5260005E38F, 2.7233305E38F, -1.0831086E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO);
    assert(p64_y_GET(pack) == (float) -7.343388E37F);
    assert(p64_vy_GET(pack) == (float)2.264946E38F);
    assert(p64_z_GET(pack) == (float) -2.922676E38F);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)7432);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)9243);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)40438);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)15277);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)46422);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)2063716958L);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)6151);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)45453);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)13367);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)56544);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)1744);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)7403);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)64989);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)57169);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)39785);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)54720);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)42034);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)35221);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)222);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)47515);
};


void c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)185);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)43065);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)66);
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)93);
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)239);
};


void c_LoopBackDemoChannel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)103);
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)21655);
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)181);
};


void c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_y_GET(pack) == (int16_t)(int16_t) -21986);
    assert(p69_r_GET(pack) == (int16_t)(int16_t) -3917);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)87);
    assert(p69_x_GET(pack) == (int16_t)(int16_t) -23034);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)16078);
    assert(p69_z_GET(pack) == (int16_t)(int16_t) -4271);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)217);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)55675);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)18703);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)45958);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)60928);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)11627);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)2333);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)53449);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)1557);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)190);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p73_param4_GET(pack) == (float)1.2432373E38F);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)35);
    assert(p73_param3_GET(pack) == (float) -1.9477867E38F);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)20);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)164);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION);
    assert(p73_x_GET(pack) == (int32_t)647782352);
    assert(p73_z_GET(pack) == (float) -2.5425002E38F);
    assert(p73_param1_GET(pack) == (float) -1.4558292E38F);
    assert(p73_y_GET(pack) == (int32_t) -934327576);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)11383);
    assert(p73_param2_GET(pack) == (float)3.2115687E36F);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)172);
};


void c_LoopBackDemoChannel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)57359);
    assert(p74_climb_GET(pack) == (float) -2.53799E38F);
    assert(p74_airspeed_GET(pack) == (float)1.7795034E38F);
    assert(p74_groundspeed_GET(pack) == (float)3.171498E38F);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t)18252);
    assert(p74_alt_GET(pack) == (float) -6.79031E37F);
};


void c_LoopBackDemoChannel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_param1_GET(pack) == (float)1.5619148E38F);
    assert(p75_x_GET(pack) == (int32_t)1263879986);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_LAND);
    assert(p75_param2_GET(pack) == (float) -2.3643594E38F);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)186);
    assert(p75_z_GET(pack) == (float) -3.1066983E38F);
    assert(p75_param3_GET(pack) == (float) -2.2364502E38F);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p75_param4_GET(pack) == (float)2.7672206E38F);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p75_y_GET(pack) == (int32_t) -1735071120);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)232);
};


void c_LoopBackDemoChannel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_param3_GET(pack) == (float) -8.127245E37F);
    assert(p76_param7_GET(pack) == (float)9.2381704E36F);
    assert(p76_param4_GET(pack) == (float)8.0648626E37F);
    assert(p76_param6_GET(pack) == (float) -1.6709151E38F);
    assert(p76_param1_GET(pack) == (float) -1.4408041E38F);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST);
    assert(p76_param2_GET(pack) == (float) -2.598825E38F);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)43);
    assert(p76_param5_GET(pack) == (float) -5.443922E37F);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)216);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)211);
};


void c_LoopBackDemoChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_result_param2_TRY(ph) == (int32_t) -1413645411);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_TEMPORARILY_REJECTED);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)63);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)119);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)211);
};


void c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_thrust_GET(pack) == (float)1.5985351E35F);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p81_roll_GET(pack) == (float)1.3492964E37F);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)3077001124L);
    assert(p81_pitch_GET(pack) == (float)1.4344153E38F);
    assert(p81_yaw_GET(pack) == (float) -1.8082857E37F);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)136);
};


void c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)99);
    assert(p82_thrust_GET(pack) == (float)1.3915958E38F);
    assert(p82_body_roll_rate_GET(pack) == (float)1.60722E38F);
    assert(p82_body_yaw_rate_GET(pack) == (float) -1.1748586E38F);
    {
        float exemplary[] =  {9.827737E36F, 7.6253793E37F, 3.3648002E38F, -2.3640467E38F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_body_pitch_rate_GET(pack) == (float) -1.22335E38F);
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)3362812495L);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)187);
};


void c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_thrust_GET(pack) == (float)1.4852369E37F);
    assert(p83_body_roll_rate_GET(pack) == (float)6.7055365E37F);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)4212279201L);
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p83_body_pitch_rate_GET(pack) == (float) -2.3422727E38F);
    {
        float exemplary[] =  {3.2115115E38F, -1.6909373E38F, -4.17908E37F, 1.8488136E38F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p83_body_yaw_rate_GET(pack) == (float) -1.6572855E38F);
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_vy_GET(pack) == (float) -5.589193E37F);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)19167);
    assert(p84_yaw_GET(pack) == (float)1.2643319E37F);
    assert(p84_y_GET(pack) == (float)1.788249E38F);
    assert(p84_z_GET(pack) == (float) -8.728754E37F);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)59);
    assert(p84_yaw_rate_GET(pack) == (float)1.6212486E37F);
    assert(p84_vx_GET(pack) == (float)3.944657E37F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)1372683262L);
    assert(p84_afz_GET(pack) == (float) -2.0773946E38F);
    assert(p84_x_GET(pack) == (float) -3.6156181E37F);
    assert(p84_vz_GET(pack) == (float)1.3215223E38F);
    assert(p84_afx_GET(pack) == (float)5.8983965E37F);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)93);
    assert(p84_afy_GET(pack) == (float)3.0531197E37F);
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_vy_GET(pack) == (float) -3.31067E38F);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)1485767989L);
    assert(p86_yaw_GET(pack) == (float)2.2641116E38F);
    assert(p86_afz_GET(pack) == (float) -5.698596E36F);
    assert(p86_lat_int_GET(pack) == (int32_t) -594609588);
    assert(p86_alt_GET(pack) == (float)8.654877E37F);
    assert(p86_vz_GET(pack) == (float) -2.4826807E38F);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p86_lon_int_GET(pack) == (int32_t)217882610);
    assert(p86_vx_GET(pack) == (float) -4.3946313E37F);
    assert(p86_yaw_rate_GET(pack) == (float)2.2039E37F);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)39473);
    assert(p86_afx_GET(pack) == (float)1.5389717E38F);
    assert(p86_afy_GET(pack) == (float)2.1982662E38F);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)254);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)149);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_yaw_GET(pack) == (float)1.4593399E38F);
    assert(p87_lat_int_GET(pack) == (int32_t) -1329558456);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)62854);
    assert(p87_vz_GET(pack) == (float)2.0316486E38F);
    assert(p87_alt_GET(pack) == (float) -1.1865505E38F);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)2696483531L);
    assert(p87_afy_GET(pack) == (float) -2.4690698E38F);
    assert(p87_yaw_rate_GET(pack) == (float) -1.2319185E38F);
    assert(p87_vx_GET(pack) == (float) -2.4330117E38F);
    assert(p87_afz_GET(pack) == (float)1.7977113E38F);
    assert(p87_vy_GET(pack) == (float)9.675336E37F);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p87_lon_int_GET(pack) == (int32_t)151048346);
    assert(p87_afx_GET(pack) == (float)6.702347E37F);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_yaw_GET(pack) == (float) -2.649655E37F);
    assert(p89_x_GET(pack) == (float) -1.9832343E38F);
    assert(p89_pitch_GET(pack) == (float)7.46976E37F);
    assert(p89_roll_GET(pack) == (float) -2.6716357E38F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)2033540922L);
    assert(p89_y_GET(pack) == (float)2.6991956E38F);
    assert(p89_z_GET(pack) == (float) -4.360321E37F);
};


void c_LoopBackDemoChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_yawspeed_GET(pack) == (float)1.8385872E38F);
    assert(p90_roll_GET(pack) == (float) -1.9059512E38F);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t) -18676);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t)30412);
    assert(p90_pitchspeed_GET(pack) == (float)1.2938217E38F);
    assert(p90_lon_GET(pack) == (int32_t) -1267138609);
    assert(p90_alt_GET(pack) == (int32_t)1918810612);
    assert(p90_time_usec_GET(pack) == (uint64_t)6787992741562173485L);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t)17812);
    assert(p90_lat_GET(pack) == (int32_t)980739012);
    assert(p90_yaw_GET(pack) == (float) -9.842137E37F);
    assert(p90_rollspeed_GET(pack) == (float) -3.2884453E38F);
    assert(p90_pitch_GET(pack) == (float)2.2606328E38F);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t) -24621);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t)8507);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t)31120);
};


void c_LoopBackDemoChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_aux4_GET(pack) == (float)5.8914964E37F);
    assert(p91_throttle_GET(pack) == (float)1.6939847E38F);
    assert(p91_aux1_GET(pack) == (float) -3.0324807E38F);
    assert(p91_yaw_rudder_GET(pack) == (float)1.7071875E38F);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p91_roll_ailerons_GET(pack) == (float) -6.7911776E36F);
    assert(p91_time_usec_GET(pack) == (uint64_t)4675934756276081356L);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_GUIDED_DISARMED);
    assert(p91_pitch_elevator_GET(pack) == (float) -2.9617127E38F);
    assert(p91_aux3_GET(pack) == (float)1.7066859E38F);
    assert(p91_aux2_GET(pack) == (float) -1.5771295E37F);
};


void c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)41793);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)38237);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)18538);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)53720);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)48115);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)57026);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)14927);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)55021);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)43475);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)37528);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)154);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)14260);
    assert(p92_time_usec_GET(pack) == (uint64_t)2135519452123108868L);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)55363);
};


void c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-9.195961E37F, 2.186847E38F, 5.5575065E37F, 1.2605075E38F, 1.6354382E38F, -9.536934E36F, 2.5829497E38F, 1.3364095E38F, -1.2035559E38F, 2.5763771E38F, 3.495093E36F, -2.536353E38F, -9.197629E37F, -1.1461317E36F, -2.678362E38F, -3.2190433E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_flags_GET(pack) == (uint64_t)7810884624022963362L);
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_GUIDED_DISARMED);
    assert(p93_time_usec_GET(pack) == (uint64_t)7659780335194894607L);
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t) -11275);
    assert(p100_flow_rate_y_TRY(ph) == (float)3.9594623E37F);
    assert(p100_flow_comp_m_y_GET(pack) == (float) -2.2816147E38F);
    assert(p100_flow_rate_x_TRY(ph) == (float)2.6984604E38F);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)154);
    assert(p100_ground_distance_GET(pack) == (float) -2.2753813E38F);
    assert(p100_flow_comp_m_x_GET(pack) == (float) -1.7370926E38F);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t) -792);
    assert(p100_time_usec_GET(pack) == (uint64_t)3745428389356944857L);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)174);
};


void c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_x_GET(pack) == (float)2.7741357E37F);
    assert(p101_roll_GET(pack) == (float)3.9770106E37F);
    assert(p101_usec_GET(pack) == (uint64_t)4538980757761818582L);
    assert(p101_pitch_GET(pack) == (float)9.0129825E36F);
    assert(p101_y_GET(pack) == (float) -3.05946E38F);
    assert(p101_yaw_GET(pack) == (float)3.3048965E38F);
    assert(p101_z_GET(pack) == (float)3.2837953E38F);
};


void c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_roll_GET(pack) == (float) -1.5199651E38F);
    assert(p102_y_GET(pack) == (float) -2.8158469E38F);
    assert(p102_x_GET(pack) == (float) -2.159477E38F);
    assert(p102_pitch_GET(pack) == (float) -1.2080066E38F);
    assert(p102_yaw_GET(pack) == (float) -2.274152E38F);
    assert(p102_usec_GET(pack) == (uint64_t)4224273548866285134L);
    assert(p102_z_GET(pack) == (float) -1.7959903E38F);
};


void c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_z_GET(pack) == (float)6.2916324E37F);
    assert(p103_x_GET(pack) == (float)1.714876E38F);
    assert(p103_usec_GET(pack) == (uint64_t)6028443427704655638L);
    assert(p103_y_GET(pack) == (float) -3.386113E38F);
};


void c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_x_GET(pack) == (float)1.6925496E37F);
    assert(p104_roll_GET(pack) == (float) -3.0593208E38F);
    assert(p104_yaw_GET(pack) == (float) -1.309778E37F);
    assert(p104_usec_GET(pack) == (uint64_t)2172812222773511369L);
    assert(p104_y_GET(pack) == (float) -1.0063673E38F);
    assert(p104_pitch_GET(pack) == (float)3.3037514E38F);
    assert(p104_z_GET(pack) == (float)2.55075E38F);
};


void c_LoopBackDemoChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_ygyro_GET(pack) == (float)2.1649223E37F);
    assert(p105_time_usec_GET(pack) == (uint64_t)4367100325929275467L);
    assert(p105_zgyro_GET(pack) == (float)2.4886486E37F);
    assert(p105_temperature_GET(pack) == (float) -8.2285726E37F);
    assert(p105_zacc_GET(pack) == (float)1.0971283E37F);
    assert(p105_ymag_GET(pack) == (float)2.8535243E36F);
    assert(p105_yacc_GET(pack) == (float) -1.2796279E38F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)33507);
    assert(p105_xgyro_GET(pack) == (float)3.0065125E38F);
    assert(p105_xmag_GET(pack) == (float) -1.1435678E38F);
    assert(p105_abs_pressure_GET(pack) == (float) -8.679207E36F);
    assert(p105_pressure_alt_GET(pack) == (float) -1.4378184E38F);
    assert(p105_xacc_GET(pack) == (float)2.6947958E38F);
    assert(p105_diff_pressure_GET(pack) == (float) -3.0090083E36F);
    assert(p105_zmag_GET(pack) == (float) -1.4859829E38F);
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p106_integrated_y_GET(pack) == (float) -7.349907E37F);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t) -26463);
    assert(p106_distance_GET(pack) == (float)1.3560463E38F);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)2857941788L);
    assert(p106_integrated_ygyro_GET(pack) == (float)1.0070465E38F);
    assert(p106_integrated_xgyro_GET(pack) == (float)1.0353261E38F);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)1472098126L);
    assert(p106_time_usec_GET(pack) == (uint64_t)6641210133720438358L);
    assert(p106_integrated_zgyro_GET(pack) == (float)2.7413068E38F);
    assert(p106_integrated_x_GET(pack) == (float)1.535877E38F);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)130);
};


void c_LoopBackDemoChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_temperature_GET(pack) == (float) -1.4208809E38F);
    assert(p107_ygyro_GET(pack) == (float) -2.2810939E38F);
    assert(p107_xgyro_GET(pack) == (float) -1.7534839E38F);
    assert(p107_ymag_GET(pack) == (float) -2.974762E37F);
    assert(p107_xmag_GET(pack) == (float) -2.991842E38F);
    assert(p107_zmag_GET(pack) == (float)4.7743215E37F);
    assert(p107_yacc_GET(pack) == (float)2.3348007E38F);
    assert(p107_time_usec_GET(pack) == (uint64_t)761034797834480357L);
    assert(p107_diff_pressure_GET(pack) == (float) -1.3920149E38F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)2282386627L);
    assert(p107_abs_pressure_GET(pack) == (float)2.950383E38F);
    assert(p107_pressure_alt_GET(pack) == (float)3.2916793E38F);
    assert(p107_zgyro_GET(pack) == (float)6.5607485E37F);
    assert(p107_zacc_GET(pack) == (float)3.1340568E38F);
    assert(p107_xacc_GET(pack) == (float) -2.9589553E38F);
};


void c_LoopBackDemoChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_q2_GET(pack) == (float)6.094539E36F);
    assert(p108_ygyro_GET(pack) == (float) -1.6960331E38F);
    assert(p108_zacc_GET(pack) == (float)3.2259203E37F);
    assert(p108_vd_GET(pack) == (float) -2.7822499E38F);
    assert(p108_std_dev_vert_GET(pack) == (float) -1.4212291E38F);
    assert(p108_zgyro_GET(pack) == (float)1.318147E38F);
    assert(p108_roll_GET(pack) == (float) -1.9656557E38F);
    assert(p108_q4_GET(pack) == (float) -7.410368E37F);
    assert(p108_lon_GET(pack) == (float)1.4557926E38F);
    assert(p108_yacc_GET(pack) == (float) -3.235714E37F);
    assert(p108_ve_GET(pack) == (float)9.930724E37F);
    assert(p108_yaw_GET(pack) == (float)1.6935886E38F);
    assert(p108_pitch_GET(pack) == (float)3.0300995E38F);
    assert(p108_vn_GET(pack) == (float) -6.7731393E37F);
    assert(p108_q1_GET(pack) == (float)1.5124588E38F);
    assert(p108_std_dev_horz_GET(pack) == (float) -1.1074171E38F);
    assert(p108_xgyro_GET(pack) == (float)2.7502661E38F);
    assert(p108_alt_GET(pack) == (float) -4.674864E37F);
    assert(p108_q3_GET(pack) == (float) -1.3268813E38F);
    assert(p108_lat_GET(pack) == (float)9.753474E37F);
    assert(p108_xacc_GET(pack) == (float)1.74182E38F);
};


void c_LoopBackDemoChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)43349);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)61079);
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)192);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)37);
};


void c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)120);
    {
        uint8_t exemplary[] =  {(uint8_t)37, (uint8_t)79, (uint8_t)179, (uint8_t)146, (uint8_t)65, (uint8_t)40, (uint8_t)105, (uint8_t)241, (uint8_t)110, (uint8_t)52, (uint8_t)199, (uint8_t)196, (uint8_t)162, (uint8_t)248, (uint8_t)5, (uint8_t)105, (uint8_t)208, (uint8_t)7, (uint8_t)97, (uint8_t)215, (uint8_t)49, (uint8_t)175, (uint8_t)17, (uint8_t)92, (uint8_t)222, (uint8_t)51, (uint8_t)224, (uint8_t)8, (uint8_t)156, (uint8_t)239, (uint8_t)182, (uint8_t)4, (uint8_t)204, (uint8_t)148, (uint8_t)95, (uint8_t)24, (uint8_t)113, (uint8_t)148, (uint8_t)3, (uint8_t)22, (uint8_t)8, (uint8_t)205, (uint8_t)78, (uint8_t)84, (uint8_t)206, (uint8_t)50, (uint8_t)144, (uint8_t)94, (uint8_t)110, (uint8_t)124, (uint8_t)71, (uint8_t)105, (uint8_t)69, (uint8_t)218, (uint8_t)68, (uint8_t)118, (uint8_t)232, (uint8_t)230, (uint8_t)202, (uint8_t)85, (uint8_t)0, (uint8_t)63, (uint8_t)141, (uint8_t)117, (uint8_t)241, (uint8_t)129, (uint8_t)239, (uint8_t)200, (uint8_t)52, (uint8_t)224, (uint8_t)28, (uint8_t)16, (uint8_t)47, (uint8_t)78, (uint8_t)26, (uint8_t)72, (uint8_t)5, (uint8_t)181, (uint8_t)165, (uint8_t)15, (uint8_t)100, (uint8_t)46, (uint8_t)32, (uint8_t)120, (uint8_t)51, (uint8_t)222, (uint8_t)183, (uint8_t)188, (uint8_t)40, (uint8_t)74, (uint8_t)85, (uint8_t)132, (uint8_t)194, (uint8_t)209, (uint8_t)17, (uint8_t)124, (uint8_t)210, (uint8_t)232, (uint8_t)67, (uint8_t)220, (uint8_t)129, (uint8_t)116, (uint8_t)35, (uint8_t)81, (uint8_t)61, (uint8_t)38, (uint8_t)222, (uint8_t)53, (uint8_t)159, (uint8_t)223, (uint8_t)134, (uint8_t)94, (uint8_t)181, (uint8_t)138, (uint8_t)165, (uint8_t)230, (uint8_t)97, (uint8_t)181, (uint8_t)208, (uint8_t)24, (uint8_t)127, (uint8_t)170, (uint8_t)236, (uint8_t)59, (uint8_t)65, (uint8_t)181, (uint8_t)162, (uint8_t)6, (uint8_t)1, (uint8_t)167, (uint8_t)66, (uint8_t)190, (uint8_t)250, (uint8_t)4, (uint8_t)137, (uint8_t)209, (uint8_t)164, (uint8_t)127, (uint8_t)182, (uint8_t)197, (uint8_t)136, (uint8_t)211, (uint8_t)21, (uint8_t)186, (uint8_t)158, (uint8_t)241, (uint8_t)154, (uint8_t)251, (uint8_t)104, (uint8_t)1, (uint8_t)103, (uint8_t)87, (uint8_t)238, (uint8_t)187, (uint8_t)199, (uint8_t)69, (uint8_t)176, (uint8_t)45, (uint8_t)140, (uint8_t)186, (uint8_t)116, (uint8_t)162, (uint8_t)50, (uint8_t)236, (uint8_t)138, (uint8_t)24, (uint8_t)94, (uint8_t)4, (uint8_t)115, (uint8_t)134, (uint8_t)98, (uint8_t)204, (uint8_t)136, (uint8_t)114, (uint8_t)69, (uint8_t)128, (uint8_t)212, (uint8_t)130, (uint8_t)195, (uint8_t)111, (uint8_t)100, (uint8_t)1, (uint8_t)243, (uint8_t)26, (uint8_t)26, (uint8_t)85, (uint8_t)1, (uint8_t)83, (uint8_t)170, (uint8_t)47, (uint8_t)232, (uint8_t)111, (uint8_t)135, (uint8_t)102, (uint8_t)166, (uint8_t)242, (uint8_t)134, (uint8_t)80, (uint8_t)82, (uint8_t)145, (uint8_t)73, (uint8_t)85, (uint8_t)176, (uint8_t)8, (uint8_t)182, (uint8_t)176, (uint8_t)7, (uint8_t)44, (uint8_t)180, (uint8_t)247, (uint8_t)163, (uint8_t)210, (uint8_t)251, (uint8_t)238, (uint8_t)195, (uint8_t)86, (uint8_t)127, (uint8_t)60, (uint8_t)38, (uint8_t)44, (uint8_t)224, (uint8_t)77, (uint8_t)75, (uint8_t)41, (uint8_t)2, (uint8_t)168, (uint8_t)83, (uint8_t)189, (uint8_t)157, (uint8_t)11, (uint8_t)255, (uint8_t)158, (uint8_t)136, (uint8_t)250, (uint8_t)3, (uint8_t)242, (uint8_t)90, (uint8_t)108, (uint8_t)65, (uint8_t)159, (uint8_t)243, (uint8_t)169, (uint8_t)252, (uint8_t)173, (uint8_t)197, (uint8_t)20, (uint8_t)175, (uint8_t)30, (uint8_t)96, (uint8_t)113, (uint8_t)40} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)74);
};


void c_LoopBackDemoChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_tc1_GET(pack) == (int64_t)6216357383372432676L);
    assert(p111_ts1_GET(pack) == (int64_t)2390687213564004357L);
};


void c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_time_usec_GET(pack) == (uint64_t)4899409209772939272L);
    assert(p112_seq_GET(pack) == (uint32_t)3422818750L);
};


void c_LoopBackDemoChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_lat_GET(pack) == (int32_t) -237324281);
    assert(p113_alt_GET(pack) == (int32_t)879407622);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t)26397);
    assert(p113_time_usec_GET(pack) == (uint64_t)5893784415727799151L);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)17442);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)50228);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t) -3495);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)53230);
    assert(p113_lon_GET(pack) == (int32_t) -521968509);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)247);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t) -31939);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)44446);
};


void c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)39);
    assert(p114_integrated_y_GET(pack) == (float)1.2220862E38F);
    assert(p114_integrated_zgyro_GET(pack) == (float)2.3900682E38F);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)1409113352L);
    assert(p114_time_usec_GET(pack) == (uint64_t)8520707118502435317L);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p114_integrated_ygyro_GET(pack) == (float) -9.949381E37F);
    assert(p114_integrated_x_GET(pack) == (float) -3.928495E37F);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)1600145426L);
    assert(p114_integrated_xgyro_GET(pack) == (float) -2.8678171E38F);
    assert(p114_distance_GET(pack) == (float)2.095769E38F);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t)1672);
};


void c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_vz_GET(pack) == (int16_t)(int16_t) -8304);
    assert(p115_lon_GET(pack) == (int32_t) -6974396);
    assert(p115_time_usec_GET(pack) == (uint64_t)1351689720868747456L);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t) -20664);
    assert(p115_lat_GET(pack) == (int32_t)1212037323);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)27669);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t)7930);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t) -12181);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t)27648);
    assert(p115_rollspeed_GET(pack) == (float) -1.570657E38F);
    {
        float exemplary[] =  {2.518166E38F, 1.2960316E38F, -2.8939532E38F, 2.103113E38F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_pitchspeed_GET(pack) == (float)2.6214165E38F);
    assert(p115_alt_GET(pack) == (int32_t) -1194922708);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t) -5613);
    assert(p115_yawspeed_GET(pack) == (float) -2.6819956E38F);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)62367);
};


void c_LoopBackDemoChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t)9577);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t) -17202);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t) -15757);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t) -31001);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t)21198);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)168968689L);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t) -11694);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t)11437);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t)20034);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t) -26090);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)91);
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)20974);
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)47881);
};


void c_LoopBackDemoChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_size_GET(pack) == (uint32_t)1272123671L);
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)1529);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)63504);
    assert(p118_time_utc_GET(pack) == (uint32_t)1913120668L);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)30598);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)16351);
    assert(p119_ofs_GET(pack) == (uint32_t)3962080764L);
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)63);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)246);
    assert(p119_count_GET(pack) == (uint32_t)1043121257L);
};


void c_LoopBackDemoChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)149);
    {
        uint8_t exemplary[] =  {(uint8_t)252, (uint8_t)89, (uint8_t)191, (uint8_t)253, (uint8_t)169, (uint8_t)242, (uint8_t)17, (uint8_t)243, (uint8_t)99, (uint8_t)40, (uint8_t)42, (uint8_t)102, (uint8_t)248, (uint8_t)14, (uint8_t)228, (uint8_t)18, (uint8_t)34, (uint8_t)253, (uint8_t)21, (uint8_t)93, (uint8_t)182, (uint8_t)125, (uint8_t)255, (uint8_t)28, (uint8_t)32, (uint8_t)126, (uint8_t)125, (uint8_t)117, (uint8_t)110, (uint8_t)81, (uint8_t)250, (uint8_t)63, (uint8_t)66, (uint8_t)77, (uint8_t)213, (uint8_t)186, (uint8_t)179, (uint8_t)11, (uint8_t)142, (uint8_t)185, (uint8_t)144, (uint8_t)145, (uint8_t)184, (uint8_t)92, (uint8_t)199, (uint8_t)221, (uint8_t)254, (uint8_t)117, (uint8_t)53, (uint8_t)148, (uint8_t)160, (uint8_t)213, (uint8_t)32, (uint8_t)224, (uint8_t)116, (uint8_t)77, (uint8_t)102, (uint8_t)114, (uint8_t)111, (uint8_t)201, (uint8_t)124, (uint8_t)205, (uint8_t)159, (uint8_t)102, (uint8_t)246, (uint8_t)34, (uint8_t)18, (uint8_t)40, (uint8_t)236, (uint8_t)241, (uint8_t)157, (uint8_t)109, (uint8_t)252, (uint8_t)134, (uint8_t)254, (uint8_t)98, (uint8_t)44, (uint8_t)38, (uint8_t)156, (uint8_t)169, (uint8_t)134, (uint8_t)242, (uint8_t)34, (uint8_t)62, (uint8_t)69, (uint8_t)76, (uint8_t)58, (uint8_t)236, (uint8_t)96, (uint8_t)62} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p120_ofs_GET(pack) == (uint32_t)2099474651L);
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)17925);
};


void c_LoopBackDemoChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)40);
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)30);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)244);
};


void c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)194);
    {
        uint8_t exemplary[] =  {(uint8_t)48, (uint8_t)71, (uint8_t)222, (uint8_t)245, (uint8_t)195, (uint8_t)28, (uint8_t)53, (uint8_t)124, (uint8_t)193, (uint8_t)126, (uint8_t)114, (uint8_t)174, (uint8_t)73, (uint8_t)123, (uint8_t)96, (uint8_t)188, (uint8_t)236, (uint8_t)104, (uint8_t)198, (uint8_t)30, (uint8_t)210, (uint8_t)68, (uint8_t)126, (uint8_t)48, (uint8_t)71, (uint8_t)33, (uint8_t)72, (uint8_t)20, (uint8_t)170, (uint8_t)199, (uint8_t)38, (uint8_t)200, (uint8_t)54, (uint8_t)95, (uint8_t)195, (uint8_t)18, (uint8_t)85, (uint8_t)60, (uint8_t)23, (uint8_t)166, (uint8_t)107, (uint8_t)66, (uint8_t)109, (uint8_t)67, (uint8_t)39, (uint8_t)76, (uint8_t)182, (uint8_t)4, (uint8_t)50, (uint8_t)48, (uint8_t)222, (uint8_t)104, (uint8_t)110, (uint8_t)84, (uint8_t)167, (uint8_t)230, (uint8_t)133, (uint8_t)207, (uint8_t)29, (uint8_t)215, (uint8_t)174, (uint8_t)250, (uint8_t)205, (uint8_t)116, (uint8_t)181, (uint8_t)25, (uint8_t)229, (uint8_t)15, (uint8_t)140, (uint8_t)155, (uint8_t)54, (uint8_t)25, (uint8_t)135, (uint8_t)8, (uint8_t)201, (uint8_t)32, (uint8_t)226, (uint8_t)55, (uint8_t)215, (uint8_t)112, (uint8_t)38, (uint8_t)173, (uint8_t)87, (uint8_t)81, (uint8_t)201, (uint8_t)13, (uint8_t)103, (uint8_t)99, (uint8_t)226, (uint8_t)36, (uint8_t)117, (uint8_t)162, (uint8_t)69, (uint8_t)163, (uint8_t)240, (uint8_t)90, (uint8_t)253, (uint8_t)196, (uint8_t)254, (uint8_t)250, (uint8_t)25, (uint8_t)46, (uint8_t)24, (uint8_t)10, (uint8_t)46, (uint8_t)28, (uint8_t)205, (uint8_t)246, (uint8_t)15, (uint8_t)222} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)135);
};


void c_LoopBackDemoChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)197);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)59066);
    assert(p124_lat_GET(pack) == (int32_t) -1546327807);
    assert(p124_time_usec_GET(pack) == (uint64_t)6403242311285254109L);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p124_dgps_age_GET(pack) == (uint32_t)4008752176L);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)17676);
    assert(p124_alt_GET(pack) == (int32_t) -1004414700);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)28464);
    assert(p124_lon_GET(pack) == (int32_t) -722751253);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)121);
};


void c_LoopBackDemoChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_flags_GET(pack) == e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID);
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)7007);
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)44625);
};


void c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)172);
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)24870);
    assert(p126_baudrate_GET(pack) == (uint32_t)4108406790L);
    {
        uint8_t exemplary[] =  {(uint8_t)31, (uint8_t)219, (uint8_t)59, (uint8_t)11, (uint8_t)231, (uint8_t)129, (uint8_t)248, (uint8_t)210, (uint8_t)229, (uint8_t)214, (uint8_t)213, (uint8_t)179, (uint8_t)146, (uint8_t)210, (uint8_t)200, (uint8_t)156, (uint8_t)165, (uint8_t)50, (uint8_t)125, (uint8_t)88, (uint8_t)195, (uint8_t)14, (uint8_t)9, (uint8_t)69, (uint8_t)214, (uint8_t)56, (uint8_t)101, (uint8_t)64, (uint8_t)154, (uint8_t)43, (uint8_t)129, (uint8_t)23, (uint8_t)38, (uint8_t)229, (uint8_t)28, (uint8_t)58, (uint8_t)126, (uint8_t)46, (uint8_t)190, (uint8_t)59, (uint8_t)142, (uint8_t)72, (uint8_t)22, (uint8_t)81, (uint8_t)15, (uint8_t)115, (uint8_t)218, (uint8_t)14, (uint8_t)160, (uint8_t)1, (uint8_t)87, (uint8_t)129, (uint8_t)80, (uint8_t)9, (uint8_t)239, (uint8_t)86, (uint8_t)71, (uint8_t)151, (uint8_t)255, (uint8_t)190, (uint8_t)202, (uint8_t)245, (uint8_t)118, (uint8_t)68, (uint8_t)63, (uint8_t)69, (uint8_t)142, (uint8_t)150, (uint8_t)65, (uint8_t)171} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2);
    assert(p126_flags_GET(pack) == e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND);
};


void c_LoopBackDemoChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)92);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)86);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t)1401138708);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t)454422181);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t) -211152695);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p127_accuracy_GET(pack) == (uint32_t)301579157L);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)250);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t) -1996770874);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)36124);
    assert(p127_tow_GET(pack) == (uint32_t)1583842290L);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)18);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)2421812449L);
};


void c_LoopBackDemoChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_tow_GET(pack) == (uint32_t)2356240765L);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)184);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t) -520773142);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)35125137L);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)13164);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)86);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t) -2105526490);
    assert(p128_accuracy_GET(pack) == (uint32_t)332185840L);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t)2146873563);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)144);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t) -1103639651);
};


void c_LoopBackDemoChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t) -15534);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t) -8183);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t)19408);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t)11561);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t)28381);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t)21156);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)1163283638L);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t)25546);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t)1050);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t)14330);
};


void c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)32390);
    assert(p130_size_GET(pack) == (uint32_t)2396788274L);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)64772);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)35095);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)160);
};


void c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)31869);
    {
        uint8_t exemplary[] =  {(uint8_t)34, (uint8_t)151, (uint8_t)143, (uint8_t)221, (uint8_t)148, (uint8_t)60, (uint8_t)172, (uint8_t)60, (uint8_t)76, (uint8_t)165, (uint8_t)130, (uint8_t)6, (uint8_t)41, (uint8_t)254, (uint8_t)5, (uint8_t)130, (uint8_t)233, (uint8_t)194, (uint8_t)126, (uint8_t)133, (uint8_t)44, (uint8_t)157, (uint8_t)185, (uint8_t)90, (uint8_t)45, (uint8_t)43, (uint8_t)49, (uint8_t)237, (uint8_t)139, (uint8_t)89, (uint8_t)146, (uint8_t)40, (uint8_t)48, (uint8_t)18, (uint8_t)252, (uint8_t)111, (uint8_t)144, (uint8_t)116, (uint8_t)194, (uint8_t)3, (uint8_t)206, (uint8_t)210, (uint8_t)37, (uint8_t)53, (uint8_t)8, (uint8_t)229, (uint8_t)247, (uint8_t)243, (uint8_t)134, (uint8_t)79, (uint8_t)25, (uint8_t)193, (uint8_t)53, (uint8_t)221, (uint8_t)190, (uint8_t)245, (uint8_t)122, (uint8_t)80, (uint8_t)190, (uint8_t)63, (uint8_t)62, (uint8_t)55, (uint8_t)177, (uint8_t)67, (uint8_t)72, (uint8_t)172, (uint8_t)5, (uint8_t)186, (uint8_t)87, (uint8_t)219, (uint8_t)190, (uint8_t)64, (uint8_t)136, (uint8_t)218, (uint8_t)182, (uint8_t)139, (uint8_t)229, (uint8_t)79, (uint8_t)130, (uint8_t)75, (uint8_t)135, (uint8_t)114, (uint8_t)158, (uint8_t)79, (uint8_t)23, (uint8_t)2, (uint8_t)89, (uint8_t)97, (uint8_t)34, (uint8_t)244, (uint8_t)226, (uint8_t)235, (uint8_t)35, (uint8_t)206, (uint8_t)83, (uint8_t)122, (uint8_t)248, (uint8_t)130, (uint8_t)251, (uint8_t)169, (uint8_t)113, (uint8_t)1, (uint8_t)212, (uint8_t)128, (uint8_t)245, (uint8_t)109, (uint8_t)223, (uint8_t)61, (uint8_t)123, (uint8_t)169, (uint8_t)1, (uint8_t)14, (uint8_t)52, (uint8_t)218, (uint8_t)148, (uint8_t)159, (uint8_t)186, (uint8_t)38, (uint8_t)12, (uint8_t)240, (uint8_t)193, (uint8_t)55, (uint8_t)161, (uint8_t)6, (uint8_t)84, (uint8_t)138, (uint8_t)226, (uint8_t)75, (uint8_t)230, (uint8_t)189, (uint8_t)159, (uint8_t)174, (uint8_t)184, (uint8_t)219, (uint8_t)182, (uint8_t)54, (uint8_t)252, (uint8_t)220, (uint8_t)232, (uint8_t)31, (uint8_t)148, (uint8_t)230, (uint8_t)210, (uint8_t)97, (uint8_t)10, (uint8_t)192, (uint8_t)65, (uint8_t)33, (uint8_t)27, (uint8_t)254, (uint8_t)3, (uint8_t)16, (uint8_t)33, (uint8_t)156, (uint8_t)244, (uint8_t)147, (uint8_t)247, (uint8_t)198, (uint8_t)42, (uint8_t)76, (uint8_t)17, (uint8_t)29, (uint8_t)223, (uint8_t)139, (uint8_t)102, (uint8_t)110, (uint8_t)125, (uint8_t)219, (uint8_t)217, (uint8_t)184, (uint8_t)119, (uint8_t)7, (uint8_t)177, (uint8_t)139, (uint8_t)138, (uint8_t)0, (uint8_t)83, (uint8_t)137, (uint8_t)133, (uint8_t)39, (uint8_t)197, (uint8_t)16, (uint8_t)105, (uint8_t)65, (uint8_t)114, (uint8_t)105, (uint8_t)223, (uint8_t)25, (uint8_t)163, (uint8_t)99, (uint8_t)91, (uint8_t)49, (uint8_t)70, (uint8_t)19, (uint8_t)148, (uint8_t)172, (uint8_t)128, (uint8_t)111, (uint8_t)111, (uint8_t)52, (uint8_t)200, (uint8_t)140, (uint8_t)121, (uint8_t)45, (uint8_t)149, (uint8_t)216, (uint8_t)138, (uint8_t)175, (uint8_t)223, (uint8_t)217, (uint8_t)110, (uint8_t)176, (uint8_t)165, (uint8_t)44, (uint8_t)235, (uint8_t)199, (uint8_t)97, (uint8_t)133, (uint8_t)244, (uint8_t)222, (uint8_t)52, (uint8_t)163, (uint8_t)152, (uint8_t)74, (uint8_t)58, (uint8_t)86, (uint8_t)15, (uint8_t)69, (uint8_t)23, (uint8_t)157, (uint8_t)7, (uint8_t)22, (uint8_t)119, (uint8_t)49, (uint8_t)65, (uint8_t)106, (uint8_t)69, (uint8_t)148, (uint8_t)31, (uint8_t)198, (uint8_t)1, (uint8_t)252, (uint8_t)103, (uint8_t)131, (uint8_t)43, (uint8_t)48, (uint8_t)109, (uint8_t)195, (uint8_t)199, (uint8_t)28, (uint8_t)28, (uint8_t)207, (uint8_t)56} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)16498);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)18072);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)3165828177L);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)146);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)22920);
};


void c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_lat_GET(pack) == (int32_t)872981282);
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)52108);
    assert(p133_mask_GET(pack) == (uint64_t)5750839363117718524L);
    assert(p133_lon_GET(pack) == (int32_t)1332624919);
};


void c_LoopBackDemoChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)220);
    assert(p134_lon_GET(pack) == (int32_t) -1445773787);
    assert(p134_lat_GET(pack) == (int32_t)727738984);
    {
        int16_t exemplary[] =  {(int16_t) -8965, (int16_t)23500, (int16_t) -10554, (int16_t)7371, (int16_t)21014, (int16_t) -2462, (int16_t)1745, (int16_t)12415, (int16_t)4500, (int16_t)5139, (int16_t) -834, (int16_t)13234, (int16_t) -20726, (int16_t) -12068, (int16_t)11417, (int16_t) -21289} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)38626);
};


void c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lon_GET(pack) == (int32_t)687372593);
    assert(p135_lat_GET(pack) == (int32_t)973896620);
};


void c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_terrain_height_GET(pack) == (float)5.7114317E37F);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)54835);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)1367);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)6214);
    assert(p136_lat_GET(pack) == (int32_t) -835632668);
    assert(p136_current_height_GET(pack) == (float) -3.0924985E38F);
    assert(p136_lon_GET(pack) == (int32_t)977416983);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t)18773);
    assert(p137_press_abs_GET(pack) == (float) -2.1143743E38F);
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)147823374L);
    assert(p137_press_diff_GET(pack) == (float)8.890517E37F);
};


void c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    assert(p138_time_usec_GET(pack) == (uint64_t)2128277214926114194L);
    {
        float exemplary[] =  {-2.0509934E38F, 6.3353886E37F, 1.7112174E38F, -1.566212E38F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_z_GET(pack) == (float) -9.1791436E36F);
    assert(p138_x_GET(pack) == (float)1.0035574E38F);
    assert(p138_y_GET(pack) == (float)1.678517E38F);
};


void c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)183);
    {
        float exemplary[] =  {-3.0423326E38F, 9.392168E37F, 3.4375003E37F, -9.828785E36F, 3.0990173E37F, -5.2925797E37F, -2.7914668E38F, 1.9264969E38F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)115);
    assert(p139_time_usec_GET(pack) == (uint64_t)5031269004653802540L);
};


void c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)218);
    assert(p140_time_usec_GET(pack) == (uint64_t)942289404302727694L);
    {
        float exemplary[] =  {-1.5950132E38F, 1.477658E38F, -3.1520013E38F, 1.0405636E38F, 1.6015853E37F, -2.6285352E38F, -4.018335E37F, 6.183711E37F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_time_usec_GET(pack) == (uint64_t)6532285242415557817L);
    assert(p141_altitude_monotonic_GET(pack) == (float)2.4898207E38F);
    assert(p141_altitude_relative_GET(pack) == (float) -7.6352664E37F);
    assert(p141_bottom_clearance_GET(pack) == (float) -1.2795308E38F);
    assert(p141_altitude_amsl_GET(pack) == (float) -5.7240134E37F);
    assert(p141_altitude_local_GET(pack) == (float)3.2494465E38F);
    assert(p141_altitude_terrain_GET(pack) == (float) -1.0545204E38F);
};


void c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)79);
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)153);
    {
        uint8_t exemplary[] =  {(uint8_t)149, (uint8_t)50, (uint8_t)176, (uint8_t)63, (uint8_t)204, (uint8_t)132, (uint8_t)226, (uint8_t)128, (uint8_t)28, (uint8_t)188, (uint8_t)56, (uint8_t)195, (uint8_t)251, (uint8_t)126, (uint8_t)143, (uint8_t)144, (uint8_t)55, (uint8_t)28, (uint8_t)86, (uint8_t)152, (uint8_t)63, (uint8_t)195, (uint8_t)138, (uint8_t)231, (uint8_t)99, (uint8_t)20, (uint8_t)63, (uint8_t)255, (uint8_t)129, (uint8_t)234, (uint8_t)186, (uint8_t)175, (uint8_t)46, (uint8_t)96, (uint8_t)108, (uint8_t)94, (uint8_t)190, (uint8_t)214, (uint8_t)128, (uint8_t)243, (uint8_t)95, (uint8_t)239, (uint8_t)17, (uint8_t)101, (uint8_t)77, (uint8_t)59, (uint8_t)247, (uint8_t)170, (uint8_t)234, (uint8_t)141, (uint8_t)160, (uint8_t)80, (uint8_t)31, (uint8_t)148, (uint8_t)208, (uint8_t)228, (uint8_t)97, (uint8_t)91, (uint8_t)242, (uint8_t)118, (uint8_t)209, (uint8_t)92, (uint8_t)44, (uint8_t)251, (uint8_t)211, (uint8_t)207, (uint8_t)206, (uint8_t)47, (uint8_t)62, (uint8_t)63, (uint8_t)82, (uint8_t)81, (uint8_t)150, (uint8_t)195, (uint8_t)148, (uint8_t)73, (uint8_t)33, (uint8_t)228, (uint8_t)166, (uint8_t)114, (uint8_t)124, (uint8_t)4, (uint8_t)108, (uint8_t)137, (uint8_t)207, (uint8_t)204, (uint8_t)86, (uint8_t)188, (uint8_t)37, (uint8_t)68, (uint8_t)52, (uint8_t)135, (uint8_t)213, (uint8_t)16, (uint8_t)249, (uint8_t)165, (uint8_t)254, (uint8_t)243, (uint8_t)51, (uint8_t)59, (uint8_t)10, (uint8_t)113, (uint8_t)90, (uint8_t)127, (uint8_t)64, (uint8_t)2, (uint8_t)180, (uint8_t)173, (uint8_t)219, (uint8_t)248, (uint8_t)170, (uint8_t)186, (uint8_t)195, (uint8_t)7, (uint8_t)201, (uint8_t)156, (uint8_t)140, (uint8_t)51, (uint8_t)190, (uint8_t)204} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)46);
    {
        uint8_t exemplary[] =  {(uint8_t)139, (uint8_t)44, (uint8_t)59, (uint8_t)189, (uint8_t)66, (uint8_t)196, (uint8_t)59, (uint8_t)53, (uint8_t)59, (uint8_t)158, (uint8_t)248, (uint8_t)58, (uint8_t)85, (uint8_t)156, (uint8_t)126, (uint8_t)94, (uint8_t)47, (uint8_t)177, (uint8_t)183, (uint8_t)52, (uint8_t)140, (uint8_t)122, (uint8_t)68, (uint8_t)87, (uint8_t)64, (uint8_t)129, (uint8_t)149, (uint8_t)67, (uint8_t)130, (uint8_t)35, (uint8_t)149, (uint8_t)189, (uint8_t)205, (uint8_t)156, (uint8_t)120, (uint8_t)7, (uint8_t)91, (uint8_t)114, (uint8_t)174, (uint8_t)62, (uint8_t)33, (uint8_t)0, (uint8_t)254, (uint8_t)57, (uint8_t)140, (uint8_t)254, (uint8_t)39, (uint8_t)143, (uint8_t)76, (uint8_t)142, (uint8_t)86, (uint8_t)192, (uint8_t)223, (uint8_t)197, (uint8_t)31, (uint8_t)240, (uint8_t)8, (uint8_t)85, (uint8_t)55, (uint8_t)126, (uint8_t)232, (uint8_t)224, (uint8_t)123, (uint8_t)156, (uint8_t)206, (uint8_t)129, (uint8_t)133, (uint8_t)50, (uint8_t)255, (uint8_t)53, (uint8_t)137, (uint8_t)173, (uint8_t)113, (uint8_t)183, (uint8_t)213, (uint8_t)46, (uint8_t)164, (uint8_t)113, (uint8_t)1, (uint8_t)112, (uint8_t)94, (uint8_t)69, (uint8_t)151, (uint8_t)185, (uint8_t)111, (uint8_t)16, (uint8_t)26, (uint8_t)243, (uint8_t)82, (uint8_t)117, (uint8_t)126, (uint8_t)224, (uint8_t)239, (uint8_t)188, (uint8_t)137, (uint8_t)74, (uint8_t)42, (uint8_t)213, (uint8_t)133, (uint8_t)91, (uint8_t)85, (uint8_t)6, (uint8_t)206, (uint8_t)91, (uint8_t)240, (uint8_t)68, (uint8_t)181, (uint8_t)61, (uint8_t)225, (uint8_t)104, (uint8_t)107, (uint8_t)146, (uint8_t)43, (uint8_t)163, (uint8_t)121, (uint8_t)137, (uint8_t)217, (uint8_t)184, (uint8_t)18, (uint8_t)22} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_press_abs_GET(pack) == (float)1.665604E38F);
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)734939791L);
    assert(p143_press_diff_GET(pack) == (float) -9.77934E37F);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t) -20481);
};


void c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-3.3584447E38F, -1.8081815E38F, 1.7826905E38F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {2.163902E38F, 2.3993E38F, 3.0728895E38F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_alt_GET(pack) == (float) -3.0320637E38F);
    assert(p144_timestamp_GET(pack) == (uint64_t)8269045999858669938L);
    {
        float exemplary[] =  {5.6824005E37F, -1.8867868E38F, 6.502442E37F, 6.1573633E37F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lat_GET(pack) == (int32_t) -391197420);
    {
        float exemplary[] =  {2.4964814E37F, 1.5506228E38F, -1.8536333E38F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_custom_state_GET(pack) == (uint64_t)4993819247708968352L);
    {
        float exemplary[] =  {-2.7329446E38F, -2.827166E38F, 4.0466333E37F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lon_GET(pack) == (int32_t)1642453623);
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)8);
};


void c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_roll_rate_GET(pack) == (float)2.16629E38F);
    assert(p146_time_usec_GET(pack) == (uint64_t)8973000993090494951L);
    assert(p146_y_pos_GET(pack) == (float)2.3718232E38F);
    assert(p146_z_acc_GET(pack) == (float) -9.220448E37F);
    assert(p146_z_pos_GET(pack) == (float) -3.1056279E37F);
    {
        float exemplary[] =  {2.557897E38F, 1.5348905E38F, 3.354148E38F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-2.4050161E38F, 3.2881427E38F, 2.6955282E38F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-1.8872664E38F, -3.174721E37F, -6.6659797E37F, 1.893513E38F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_pitch_rate_GET(pack) == (float)2.9684976E38F);
    assert(p146_z_vel_GET(pack) == (float) -1.4210391E38F);
    assert(p146_yaw_rate_GET(pack) == (float)7.6844305E37F);
    assert(p146_y_acc_GET(pack) == (float) -3.3161135E38F);
    assert(p146_x_vel_GET(pack) == (float) -2.7937464E38F);
    assert(p146_y_vel_GET(pack) == (float) -5.7973886E37F);
    assert(p146_x_pos_GET(pack) == (float) -1.1236507E37F);
    assert(p146_airspeed_GET(pack) == (float) -9.363715E37F);
    assert(p146_x_acc_GET(pack) == (float)1.94784E38F);
};


void c_LoopBackDemoChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_current_consumed_GET(pack) == (int32_t)61266026);
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)99);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_UNKNOWN);
    {
        uint16_t exemplary[] =  {(uint16_t)62298, (uint16_t)4960, (uint16_t)12902, (uint16_t)15033, (uint16_t)35394, (uint16_t)28127, (uint16_t)55679, (uint16_t)11449, (uint16_t)25408, (uint16_t)56559} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t) -15423);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIPO);
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t) -17799);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t) -82);
    assert(p147_energy_consumed_GET(pack) == (int32_t)732109563);
};


void c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_capabilities_GET(pack) == e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_TERRAIN);
    {
        uint8_t exemplary[] =  {(uint8_t)207, (uint8_t)138, (uint8_t)215, (uint8_t)170, (uint8_t)235, (uint8_t)59, (uint8_t)61, (uint8_t)140, (uint8_t)187, (uint8_t)45, (uint8_t)166, (uint8_t)56, (uint8_t)198, (uint8_t)56, (uint8_t)218, (uint8_t)36, (uint8_t)17, (uint8_t)18} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)74, (uint8_t)250, (uint8_t)201, (uint8_t)149, (uint8_t)154, (uint8_t)46, (uint8_t)94, (uint8_t)210} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)46012);
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)2803531321L);
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)3961482113L);
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)65267);
    {
        uint8_t exemplary[] =  {(uint8_t)40, (uint8_t)93, (uint8_t)43, (uint8_t)169, (uint8_t)49, (uint8_t)102, (uint8_t)162, (uint8_t)43} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_board_version_GET(pack) == (uint32_t)1434113677L);
    {
        uint8_t exemplary[] =  {(uint8_t)228, (uint8_t)78, (uint8_t)254, (uint8_t)90, (uint8_t)63, (uint8_t)174, (uint8_t)226, (uint8_t)79} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_uid_GET(pack) == (uint64_t)1161299515349844093L);
    assert(p148_os_sw_version_GET(pack) == (uint32_t)3761179558L);
};


void c_LoopBackDemoChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)134);
    assert(p149_x_TRY(ph) == (float) -6.980733E37F);
    assert(p149_time_usec_GET(pack) == (uint64_t)2138286559829983005L);
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p149_angle_y_GET(pack) == (float) -3.1792681E38F);
    assert(p149_size_x_GET(pack) == (float) -1.8563865E38F);
    assert(p149_z_TRY(ph) == (float)2.6987243E38F);
    assert(p149_y_TRY(ph) == (float) -1.9031743E38F);
    assert(p149_size_y_GET(pack) == (float) -2.2719933E38F);
    {
        float exemplary[] =  {2.2045105E38F, -2.2482891E37F, 3.9971734E37F, -8.769988E37F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_angle_x_GET(pack) == (float) -2.948762E38F);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)86);
    assert(p149_distance_GET(pack) == (float)1.6958546E38F);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER);
};


void c_LoopBackDemoChannel_on_FLEXIFUNCTION_SET_150(Bounds_Inside * ph, Pack * pack)
{
    assert(p150_target_system_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p150_target_component_GET(pack) == (uint8_t)(uint8_t)186);
};


void c_LoopBackDemoChannel_on_FLEXIFUNCTION_READ_REQ_151(Bounds_Inside * ph, Pack * pack)
{
    assert(p151_target_component_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p151_target_system_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p151_read_req_type_GET(pack) == (int16_t)(int16_t) -5975);
    assert(p151_data_index_GET(pack) == (int16_t)(int16_t)26186);
};


void c_LoopBackDemoChannel_on_FLEXIFUNCTION_BUFFER_FUNCTION_152(Bounds_Inside * ph, Pack * pack)
{
    assert(p152_data_size_GET(pack) == (uint16_t)(uint16_t)53874);
    assert(p152_func_count_GET(pack) == (uint16_t)(uint16_t)1681);
    {
        int8_t exemplary[] =  {(int8_t) -124, (int8_t)51, (int8_t) -53, (int8_t) -105, (int8_t) -103, (int8_t) -25, (int8_t) -31, (int8_t) -65, (int8_t) -121, (int8_t) -38, (int8_t)93, (int8_t) -50, (int8_t)109, (int8_t) -85, (int8_t)66, (int8_t) -50, (int8_t) -52, (int8_t) -48, (int8_t) -118, (int8_t)6, (int8_t) -27, (int8_t) -49, (int8_t)44, (int8_t) -80, (int8_t) -79, (int8_t) -90, (int8_t)66, (int8_t) -40, (int8_t) -78, (int8_t) -48, (int8_t) -85, (int8_t)73, (int8_t)118, (int8_t)56, (int8_t)15, (int8_t)2, (int8_t)25, (int8_t) -55, (int8_t)92, (int8_t)104, (int8_t)78, (int8_t)116, (int8_t) -67, (int8_t)110, (int8_t)86, (int8_t) -40, (int8_t) -31, (int8_t)103} ;
        int8_t*  sample = p152_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 48);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p152_data_address_GET(pack) == (uint16_t)(uint16_t)21369);
    assert(p152_target_component_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p152_func_index_GET(pack) == (uint16_t)(uint16_t)38100);
    assert(p152_target_system_GET(pack) == (uint8_t)(uint8_t)209);
};


void c_LoopBackDemoChannel_on_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_153(Bounds_Inside * ph, Pack * pack)
{
    assert(p153_func_index_GET(pack) == (uint16_t)(uint16_t)966);
    assert(p153_result_GET(pack) == (uint16_t)(uint16_t)47749);
    assert(p153_target_system_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p153_target_component_GET(pack) == (uint8_t)(uint8_t)224);
};


void c_LoopBackDemoChannel_on_FLEXIFUNCTION_DIRECTORY_155(Bounds_Inside * ph, Pack * pack)
{
    assert(p155_start_index_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p155_target_component_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p155_count_GET(pack) == (uint8_t)(uint8_t)39);
    assert(p155_target_system_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p155_directory_type_GET(pack) == (uint8_t)(uint8_t)178);
    {
        int8_t exemplary[] =  {(int8_t)125, (int8_t) -90, (int8_t)40, (int8_t) -2, (int8_t) -55, (int8_t)76, (int8_t)113, (int8_t) -93, (int8_t) -68, (int8_t) -32, (int8_t) -124, (int8_t) -4, (int8_t)111, (int8_t) -101, (int8_t)127, (int8_t) -78, (int8_t) -108, (int8_t) -61, (int8_t) -112, (int8_t) -2, (int8_t) -37, (int8_t)33, (int8_t) -76, (int8_t) -18, (int8_t) -6, (int8_t) -29, (int8_t)23, (int8_t)33, (int8_t) -7, (int8_t) -7, (int8_t)21, (int8_t)100, (int8_t) -36, (int8_t)79, (int8_t)47, (int8_t) -76, (int8_t) -110, (int8_t) -46, (int8_t)41, (int8_t) -117, (int8_t)73, (int8_t)72, (int8_t)27, (int8_t)37, (int8_t)23, (int8_t)5, (int8_t)109, (int8_t) -33} ;
        int8_t*  sample = p155_directory_data_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 48);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_FLEXIFUNCTION_DIRECTORY_ACK_156(Bounds_Inside * ph, Pack * pack)
{
    assert(p156_directory_type_GET(pack) == (uint8_t)(uint8_t)206);
    assert(p156_count_GET(pack) == (uint8_t)(uint8_t)57);
    assert(p156_target_system_GET(pack) == (uint8_t)(uint8_t)66);
    assert(p156_target_component_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p156_result_GET(pack) == (uint16_t)(uint16_t)19194);
    assert(p156_start_index_GET(pack) == (uint8_t)(uint8_t)144);
};


void c_LoopBackDemoChannel_on_FLEXIFUNCTION_COMMAND_157(Bounds_Inside * ph, Pack * pack)
{
    assert(p157_target_component_GET(pack) == (uint8_t)(uint8_t)250);
    assert(p157_target_system_GET(pack) == (uint8_t)(uint8_t)124);
    assert(p157_command_type_GET(pack) == (uint8_t)(uint8_t)211);
};


void c_LoopBackDemoChannel_on_FLEXIFUNCTION_COMMAND_ACK_158(Bounds_Inside * ph, Pack * pack)
{
    assert(p158_result_GET(pack) == (uint16_t)(uint16_t)63717);
    assert(p158_command_type_GET(pack) == (uint16_t)(uint16_t)42545);
};


void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F2_A_170(Bounds_Inside * ph, Pack * pack)
{
    assert(p170_sue_air_speed_3DIMU_GET(pack) == (uint16_t)(uint16_t)11457);
    assert(p170_sue_time_GET(pack) == (uint32_t)1728506598L);
    assert(p170_sue_rmat5_GET(pack) == (int16_t)(int16_t) -1379);
    assert(p170_sue_rmat1_GET(pack) == (int16_t)(int16_t) -26362);
    assert(p170_sue_longitude_GET(pack) == (int32_t) -1783621007);
    assert(p170_sue_rmat8_GET(pack) == (int16_t)(int16_t)10354);
    assert(p170_sue_sog_GET(pack) == (int16_t)(int16_t)6789);
    assert(p170_sue_hdop_GET(pack) == (int16_t)(int16_t)28146);
    assert(p170_sue_magFieldEarth0_GET(pack) == (int16_t)(int16_t) -16512);
    assert(p170_sue_magFieldEarth1_GET(pack) == (int16_t)(int16_t)9307);
    assert(p170_sue_rmat7_GET(pack) == (int16_t)(int16_t)23119);
    assert(p170_sue_estimated_wind_0_GET(pack) == (int16_t)(int16_t)26897);
    assert(p170_sue_estimated_wind_2_GET(pack) == (int16_t)(int16_t) -15002);
    assert(p170_sue_rmat6_GET(pack) == (int16_t)(int16_t) -29186);
    assert(p170_sue_estimated_wind_1_GET(pack) == (int16_t)(int16_t) -828);
    assert(p170_sue_altitude_GET(pack) == (int32_t) -1117530091);
    assert(p170_sue_status_GET(pack) == (uint8_t)(uint8_t)114);
    assert(p170_sue_rmat4_GET(pack) == (int16_t)(int16_t)24282);
    assert(p170_sue_svs_GET(pack) == (int16_t)(int16_t)8882);
    assert(p170_sue_cpu_load_GET(pack) == (uint16_t)(uint16_t)9708);
    assert(p170_sue_magFieldEarth2_GET(pack) == (int16_t)(int16_t) -20330);
    assert(p170_sue_cog_GET(pack) == (uint16_t)(uint16_t)48597);
    assert(p170_sue_rmat3_GET(pack) == (int16_t)(int16_t)28244);
    assert(p170_sue_latitude_GET(pack) == (int32_t) -1007516440);
    assert(p170_sue_waypoint_index_GET(pack) == (uint16_t)(uint16_t)6676);
    assert(p170_sue_rmat0_GET(pack) == (int16_t)(int16_t)29220);
    assert(p170_sue_rmat2_GET(pack) == (int16_t)(int16_t)13603);
};


void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F2_B_171(Bounds_Inside * ph, Pack * pack)
{
    assert(p171_sue_bat_volt_GET(pack) == (int16_t)(int16_t) -7955);
    assert(p171_sue_pwm_input_6_GET(pack) == (int16_t)(int16_t)25241);
    assert(p171_sue_imu_location_y_GET(pack) == (int16_t)(int16_t)8176);
    assert(p171_sue_imu_location_z_GET(pack) == (int16_t)(int16_t) -4780);
    assert(p171_sue_pwm_input_9_GET(pack) == (int16_t)(int16_t)28547);
    assert(p171_sue_desired_height_GET(pack) == (int16_t)(int16_t) -13330);
    assert(p171_sue_pwm_input_1_GET(pack) == (int16_t)(int16_t)24751);
    assert(p171_sue_pwm_output_9_GET(pack) == (int16_t)(int16_t) -17909);
    assert(p171_sue_pwm_output_4_GET(pack) == (int16_t)(int16_t)15693);
    assert(p171_sue_barom_temp_GET(pack) == (int16_t)(int16_t)11530);
    assert(p171_sue_pwm_output_2_GET(pack) == (int16_t)(int16_t)29303);
    assert(p171_sue_bat_amp_hours_GET(pack) == (int16_t)(int16_t) -5323);
    assert(p171_sue_pwm_input_8_GET(pack) == (int16_t)(int16_t) -30685);
    assert(p171_sue_location_error_earth_x_GET(pack) == (int16_t)(int16_t)28411);
    assert(p171_sue_aero_x_GET(pack) == (int16_t)(int16_t) -6512);
    assert(p171_sue_aero_y_GET(pack) == (int16_t)(int16_t) -9088);
    assert(p171_sue_waypoint_goal_y_GET(pack) == (int16_t)(int16_t)7273);
    assert(p171_sue_pwm_input_5_GET(pack) == (int16_t)(int16_t)1568);
    assert(p171_sue_pwm_output_6_GET(pack) == (int16_t)(int16_t) -2795);
    assert(p171_sue_pwm_output_10_GET(pack) == (int16_t)(int16_t) -6341);
    assert(p171_sue_waypoint_goal_x_GET(pack) == (int16_t)(int16_t) -5987);
    assert(p171_sue_aero_z_GET(pack) == (int16_t)(int16_t)382);
    assert(p171_sue_pwm_output_7_GET(pack) == (int16_t)(int16_t) -30336);
    assert(p171_sue_bat_amp_GET(pack) == (int16_t)(int16_t) -18786);
    assert(p171_sue_pwm_input_12_GET(pack) == (int16_t)(int16_t)27942);
    assert(p171_sue_imu_velocity_x_GET(pack) == (int16_t)(int16_t) -13942);
    assert(p171_sue_osc_fails_GET(pack) == (int16_t)(int16_t) -31414);
    assert(p171_sue_location_error_earth_y_GET(pack) == (int16_t)(int16_t)21565);
    assert(p171_sue_pwm_input_3_GET(pack) == (int16_t)(int16_t) -13672);
    assert(p171_sue_flags_GET(pack) == (uint32_t)3214985689L);
    assert(p171_sue_memory_stack_free_GET(pack) == (int16_t)(int16_t) -29867);
    assert(p171_sue_pwm_input_10_GET(pack) == (int16_t)(int16_t)3660);
    assert(p171_sue_pwm_input_11_GET(pack) == (int16_t)(int16_t)25752);
    assert(p171_sue_imu_velocity_z_GET(pack) == (int16_t)(int16_t)17561);
    assert(p171_sue_barom_alt_GET(pack) == (int32_t) -1445104344);
    assert(p171_sue_pwm_output_1_GET(pack) == (int16_t)(int16_t) -13236);
    assert(p171_sue_time_GET(pack) == (uint32_t)2855536104L);
    assert(p171_sue_pwm_output_5_GET(pack) == (int16_t)(int16_t) -19925);
    assert(p171_sue_barom_press_GET(pack) == (int32_t) -514352545);
    assert(p171_sue_waypoint_goal_z_GET(pack) == (int16_t)(int16_t) -22498);
    assert(p171_sue_pwm_output_8_GET(pack) == (int16_t)(int16_t)26375);
    assert(p171_sue_pwm_input_2_GET(pack) == (int16_t)(int16_t) -217);
    assert(p171_sue_pwm_output_12_GET(pack) == (int16_t)(int16_t) -27486);
    assert(p171_sue_pwm_input_7_GET(pack) == (int16_t)(int16_t)19846);
    assert(p171_sue_location_error_earth_z_GET(pack) == (int16_t)(int16_t)8882);
    assert(p171_sue_pwm_output_11_GET(pack) == (int16_t)(int16_t) -10020);
    assert(p171_sue_imu_location_x_GET(pack) == (int16_t)(int16_t)1112);
    assert(p171_sue_pwm_output_3_GET(pack) == (int16_t)(int16_t) -23267);
    assert(p171_sue_pwm_input_4_GET(pack) == (int16_t)(int16_t)29425);
    assert(p171_sue_imu_velocity_y_GET(pack) == (int16_t)(int16_t)11093);
};


void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F4_172(Bounds_Inside * ph, Pack * pack)
{
    assert(p172_sue_ROLL_STABILIZATION_RUDDER_GET(pack) == (uint8_t)(uint8_t)37);
    assert(p172_sue_RUDDER_NAVIGATION_GET(pack) == (uint8_t)(uint8_t)134);
    assert(p172_sue_ROLL_STABILIZATION_AILERONS_GET(pack) == (uint8_t)(uint8_t)237);
    assert(p172_sue_YAW_STABILIZATION_RUDDER_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p172_sue_AILERON_NAVIGATION_GET(pack) == (uint8_t)(uint8_t)87);
    assert(p172_sue_ALTITUDEHOLD_STABILIZED_GET(pack) == (uint8_t)(uint8_t)248);
    assert(p172_sue_YAW_STABILIZATION_AILERON_GET(pack) == (uint8_t)(uint8_t)246);
    assert(p172_sue_PITCH_STABILIZATION_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p172_sue_RACING_MODE_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p172_sue_ALTITUDEHOLD_WAYPOINT_GET(pack) == (uint8_t)(uint8_t)119);
};


void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F5_173(Bounds_Inside * ph, Pack * pack)
{
    assert(p173_sue_YAWKD_AILERON_GET(pack) == (float) -1.0503641E38F);
    assert(p173_sue_ROLLKP_GET(pack) == (float) -1.2620685E38F);
    assert(p173_sue_YAWKP_AILERON_GET(pack) == (float)1.9366971E38F);
    assert(p173_sue_ROLLKD_GET(pack) == (float) -3.2196832E37F);
};


void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F6_174(Bounds_Inside * ph, Pack * pack)
{
    assert(p174_sue_RUDDER_ELEV_MIX_GET(pack) == (float)3.1195027E38F);
    assert(p174_sue_ROLL_ELEV_MIX_GET(pack) == (float)2.0757967E38F);
    assert(p174_sue_ELEVATOR_BOOST_GET(pack) == (float) -1.6657113E38F);
    assert(p174_sue_PITCHKD_GET(pack) == (float) -1.1216847E37F);
    assert(p174_sue_PITCHGAIN_GET(pack) == (float)2.7023494E38F);
};


void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F7_175(Bounds_Inside * ph, Pack * pack)
{
    assert(p175_sue_ROLLKP_RUDDER_GET(pack) == (float) -1.5813407E38F);
    assert(p175_sue_YAWKD_RUDDER_GET(pack) == (float) -3.1053255E38F);
    assert(p175_sue_ROLLKD_RUDDER_GET(pack) == (float)3.323893E38F);
    assert(p175_sue_RTL_PITCH_DOWN_GET(pack) == (float) -2.70709E38F);
    assert(p175_sue_RUDDER_BOOST_GET(pack) == (float) -1.1036311E38F);
    assert(p175_sue_YAWKP_RUDDER_GET(pack) == (float) -8.559254E37F);
};


void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F8_176(Bounds_Inside * ph, Pack * pack)
{
    assert(p176_sue_ALT_HOLD_PITCH_MAX_GET(pack) == (float)3.5566065E37F);
    assert(p176_sue_ALT_HOLD_PITCH_MIN_GET(pack) == (float)3.2096414E38F);
    assert(p176_sue_ALT_HOLD_THROTTLE_MAX_GET(pack) == (float) -1.4351243E38F);
    assert(p176_sue_ALT_HOLD_PITCH_HIGH_GET(pack) == (float)5.4667163E37F);
    assert(p176_sue_HEIGHT_TARGET_MIN_GET(pack) == (float) -2.1224836E38F);
    assert(p176_sue_ALT_HOLD_THROTTLE_MIN_GET(pack) == (float) -7.519257E35F);
    assert(p176_sue_HEIGHT_TARGET_MAX_GET(pack) == (float)2.7034168E38F);
};


void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F13_177(Bounds_Inside * ph, Pack * pack)
{
    assert(p177_sue_week_no_GET(pack) == (int16_t)(int16_t)31424);
    assert(p177_sue_lon_origin_GET(pack) == (int32_t)1452747412);
    assert(p177_sue_lat_origin_GET(pack) == (int32_t)541527982);
    assert(p177_sue_alt_origin_GET(pack) == (int32_t) -1026972575);
};


void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F14_178(Bounds_Inside * ph, Pack * pack)
{
    assert(p178_sue_DR_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p178_sue_RCON_GET(pack) == (int16_t)(int16_t) -7174);
    assert(p178_sue_osc_fail_count_GET(pack) == (int16_t)(int16_t) -10414);
    assert(p178_sue_GPS_TYPE_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p178_sue_BOARD_TYPE_GET(pack) == (uint8_t)(uint8_t)14);
    assert(p178_sue_AIRFRAME_GET(pack) == (uint8_t)(uint8_t)87);
    assert(p178_sue_TRAP_FLAGS_GET(pack) == (int16_t)(int16_t) -25821);
    assert(p178_sue_WIND_ESTIMATION_GET(pack) == (uint8_t)(uint8_t)153);
    assert(p178_sue_FLIGHT_PLAN_TYPE_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p178_sue_TRAP_SOURCE_GET(pack) == (uint32_t)3293308493L);
    assert(p178_sue_CLOCK_CONFIG_GET(pack) == (uint8_t)(uint8_t)130);
};


void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F15_179(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)3, (uint8_t)28, (uint8_t)251, (uint8_t)11, (uint8_t)152, (uint8_t)152, (uint8_t)229, (uint8_t)114, (uint8_t)77, (uint8_t)227, (uint8_t)109, (uint8_t)81, (uint8_t)34, (uint8_t)93, (uint8_t)195, (uint8_t)98, (uint8_t)116, (uint8_t)92, (uint8_t)227, (uint8_t)58} ;
        uint8_t*  sample = p179_sue_ID_VEHICLE_REGISTRATION_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)84, (uint8_t)158, (uint8_t)157, (uint8_t)230, (uint8_t)235, (uint8_t)89, (uint8_t)78, (uint8_t)138, (uint8_t)56, (uint8_t)238, (uint8_t)58, (uint8_t)197, (uint8_t)18, (uint8_t)134, (uint8_t)112, (uint8_t)88, (uint8_t)23, (uint8_t)16, (uint8_t)27, (uint8_t)149, (uint8_t)62, (uint8_t)113, (uint8_t)112, (uint8_t)246, (uint8_t)230, (uint8_t)198, (uint8_t)99, (uint8_t)130, (uint8_t)161, (uint8_t)8, (uint8_t)246, (uint8_t)148, (uint8_t)100, (uint8_t)141, (uint8_t)179, (uint8_t)29, (uint8_t)8, (uint8_t)185, (uint8_t)209, (uint8_t)82} ;
        uint8_t*  sample = p179_sue_ID_VEHICLE_MODEL_NAME_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 40);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F16_180(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)120, (uint8_t)194, (uint8_t)197, (uint8_t)86, (uint8_t)25, (uint8_t)29, (uint8_t)148, (uint8_t)84, (uint8_t)38, (uint8_t)174, (uint8_t)162, (uint8_t)121, (uint8_t)0, (uint8_t)110, (uint8_t)150, (uint8_t)148, (uint8_t)106, (uint8_t)215, (uint8_t)50, (uint8_t)90, (uint8_t)17, (uint8_t)177, (uint8_t)157, (uint8_t)229, (uint8_t)108, (uint8_t)61, (uint8_t)109, (uint8_t)180, (uint8_t)129, (uint8_t)189, (uint8_t)16, (uint8_t)199, (uint8_t)159, (uint8_t)47, (uint8_t)34, (uint8_t)18, (uint8_t)99, (uint8_t)52, (uint8_t)73, (uint8_t)201} ;
        uint8_t*  sample = p180_sue_ID_LEAD_PILOT_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 40);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)161, (uint8_t)109, (uint8_t)134, (uint8_t)57, (uint8_t)143, (uint8_t)152, (uint8_t)52, (uint8_t)128, (uint8_t)142, (uint8_t)201, (uint8_t)190, (uint8_t)26, (uint8_t)128, (uint8_t)64, (uint8_t)239, (uint8_t)171, (uint8_t)95, (uint8_t)26, (uint8_t)168, (uint8_t)59, (uint8_t)90, (uint8_t)111, (uint8_t)106, (uint8_t)234, (uint8_t)14, (uint8_t)97, (uint8_t)93, (uint8_t)133, (uint8_t)6, (uint8_t)8, (uint8_t)254, (uint8_t)201, (uint8_t)40, (uint8_t)211, (uint8_t)100, (uint8_t)31, (uint8_t)245, (uint8_t)109, (uint8_t)62, (uint8_t)92, (uint8_t)240, (uint8_t)140, (uint8_t)142, (uint8_t)164, (uint8_t)17, (uint8_t)130, (uint8_t)203, (uint8_t)26, (uint8_t)0, (uint8_t)194, (uint8_t)254, (uint8_t)49, (uint8_t)97, (uint8_t)154, (uint8_t)35, (uint8_t)107, (uint8_t)49, (uint8_t)97, (uint8_t)49, (uint8_t)224, (uint8_t)238, (uint8_t)98, (uint8_t)233, (uint8_t)225, (uint8_t)141, (uint8_t)250, (uint8_t)40, (uint8_t)111, (uint8_t)188, (uint8_t)61} ;
        uint8_t*  sample = p180_sue_ID_DIY_DRONES_URL_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_ALTITUDES_181(Bounds_Inside * ph, Pack * pack)
{
    assert(p181_alt_range_finder_GET(pack) == (int32_t)1388909485);
    assert(p181_alt_gps_GET(pack) == (int32_t)2111149540);
    assert(p181_alt_optical_flow_GET(pack) == (int32_t)1829594703);
    assert(p181_alt_barometric_GET(pack) == (int32_t) -753107528);
    assert(p181_time_boot_ms_GET(pack) == (uint32_t)2502614283L);
    assert(p181_alt_extra_GET(pack) == (int32_t) -713274293);
    assert(p181_alt_imu_GET(pack) == (int32_t) -1486294764);
};


void c_LoopBackDemoChannel_on_AIRSPEEDS_182(Bounds_Inside * ph, Pack * pack)
{
    assert(p182_airspeed_pitot_GET(pack) == (int16_t)(int16_t) -32452);
    assert(p182_time_boot_ms_GET(pack) == (uint32_t)340954237L);
    assert(p182_aoy_GET(pack) == (int16_t)(int16_t)16380);
    assert(p182_aoa_GET(pack) == (int16_t)(int16_t)20121);
    assert(p182_airspeed_ultrasonic_GET(pack) == (int16_t)(int16_t)24035);
    assert(p182_airspeed_imu_GET(pack) == (int16_t)(int16_t)23835);
    assert(p182_airspeed_hot_wire_GET(pack) == (int16_t)(int16_t)25501);
};


void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F17_183(Bounds_Inside * ph, Pack * pack)
{
    assert(p183_sue_turn_rate_nav_GET(pack) == (float)2.1354444E38F);
    assert(p183_sue_turn_rate_fbw_GET(pack) == (float) -1.4698529E38F);
    assert(p183_sue_feed_forward_GET(pack) == (float)2.8075637E38F);
};


void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F18_184(Bounds_Inside * ph, Pack * pack)
{
    assert(p184_reference_speed_GET(pack) == (float)2.2881497E38F);
    assert(p184_angle_of_attack_normal_GET(pack) == (float)1.895346E38F);
    assert(p184_angle_of_attack_inverted_GET(pack) == (float) -9.633828E37F);
    assert(p184_elevator_trim_inverted_GET(pack) == (float) -4.2361884E37F);
    assert(p184_elevator_trim_normal_GET(pack) == (float) -3.2713592E37F);
};


void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F19_185(Bounds_Inside * ph, Pack * pack)
{
    assert(p185_sue_throttle_reversed_GET(pack) == (uint8_t)(uint8_t)171);
    assert(p185_sue_aileron_output_channel_GET(pack) == (uint8_t)(uint8_t)68);
    assert(p185_sue_aileron_reversed_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p185_sue_elevator_output_channel_GET(pack) == (uint8_t)(uint8_t)226);
    assert(p185_sue_rudder_reversed_GET(pack) == (uint8_t)(uint8_t)192);
    assert(p185_sue_throttle_output_channel_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p185_sue_elevator_reversed_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p185_sue_rudder_output_channel_GET(pack) == (uint8_t)(uint8_t)59);
};


void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F20_186(Bounds_Inside * ph, Pack * pack)
{
    assert(p186_sue_number_of_inputs_GET(pack) == (uint8_t)(uint8_t)238);
    assert(p186_sue_trim_value_input_6_GET(pack) == (int16_t)(int16_t)6825);
    assert(p186_sue_trim_value_input_12_GET(pack) == (int16_t)(int16_t) -4155);
    assert(p186_sue_trim_value_input_3_GET(pack) == (int16_t)(int16_t) -16118);
    assert(p186_sue_trim_value_input_9_GET(pack) == (int16_t)(int16_t)29377);
    assert(p186_sue_trim_value_input_1_GET(pack) == (int16_t)(int16_t)14963);
    assert(p186_sue_trim_value_input_10_GET(pack) == (int16_t)(int16_t)2635);
    assert(p186_sue_trim_value_input_11_GET(pack) == (int16_t)(int16_t) -32438);
    assert(p186_sue_trim_value_input_8_GET(pack) == (int16_t)(int16_t) -10835);
    assert(p186_sue_trim_value_input_5_GET(pack) == (int16_t)(int16_t)21356);
    assert(p186_sue_trim_value_input_4_GET(pack) == (int16_t)(int16_t)23495);
    assert(p186_sue_trim_value_input_7_GET(pack) == (int16_t)(int16_t) -29366);
    assert(p186_sue_trim_value_input_2_GET(pack) == (int16_t)(int16_t)27719);
};


void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F21_187(Bounds_Inside * ph, Pack * pack)
{
    assert(p187_sue_accel_z_offset_GET(pack) == (int16_t)(int16_t) -23288);
    assert(p187_sue_gyro_z_offset_GET(pack) == (int16_t)(int16_t)15653);
    assert(p187_sue_gyro_y_offset_GET(pack) == (int16_t)(int16_t)32089);
    assert(p187_sue_gyro_x_offset_GET(pack) == (int16_t)(int16_t) -21848);
    assert(p187_sue_accel_x_offset_GET(pack) == (int16_t)(int16_t) -20152);
    assert(p187_sue_accel_y_offset_GET(pack) == (int16_t)(int16_t) -13079);
};


void c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F22_188(Bounds_Inside * ph, Pack * pack)
{
    assert(p188_sue_accel_x_at_calibration_GET(pack) == (int16_t)(int16_t)21209);
    assert(p188_sue_gyro_y_at_calibration_GET(pack) == (int16_t)(int16_t)28244);
    assert(p188_sue_gyro_x_at_calibration_GET(pack) == (int16_t)(int16_t)25302);
    assert(p188_sue_accel_z_at_calibration_GET(pack) == (int16_t)(int16_t) -30732);
    assert(p188_sue_accel_y_at_calibration_GET(pack) == (int16_t)(int16_t) -7854);
    assert(p188_sue_gyro_z_at_calibration_GET(pack) == (int16_t)(int16_t)6097);
};


void c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_pos_vert_accuracy_GET(pack) == (float)2.540702E38F);
    assert(p230_hagl_ratio_GET(pack) == (float)1.5586507E38F);
    assert(p230_vel_ratio_GET(pack) == (float) -2.7176372E38F);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float) -1.4204905E38F);
    assert(p230_pos_horiz_ratio_GET(pack) == (float) -2.0821147E38F);
    assert(p230_tas_ratio_GET(pack) == (float) -2.266486E38F);
    assert(p230_pos_vert_ratio_GET(pack) == (float) -1.1959103E37F);
    assert(p230_flags_GET(pack) == e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL);
    assert(p230_mag_ratio_GET(pack) == (float)5.140957E37F);
    assert(p230_time_usec_GET(pack) == (uint64_t)4919896461901697827L);
};


void c_LoopBackDemoChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_wind_x_GET(pack) == (float) -2.535672E38F);
    assert(p231_wind_z_GET(pack) == (float)6.3631745E37F);
    assert(p231_var_vert_GET(pack) == (float)1.8776325E38F);
    assert(p231_vert_accuracy_GET(pack) == (float) -1.1415515E38F);
    assert(p231_var_horiz_GET(pack) == (float)4.759938E37F);
    assert(p231_time_usec_GET(pack) == (uint64_t)3172658640750231149L);
    assert(p231_wind_alt_GET(pack) == (float) -1.0932512E38F);
    assert(p231_horiz_accuracy_GET(pack) == (float)1.9992158E38F);
    assert(p231_wind_y_GET(pack) == (float)2.3589286E38F);
};


void c_LoopBackDemoChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_horiz_accuracy_GET(pack) == (float) -1.477235E38F);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)2315051899L);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)59);
    assert(p232_lon_GET(pack) == (int32_t) -1363430442);
    assert(p232_speed_accuracy_GET(pack) == (float) -2.6937752E38F);
    assert(p232_vert_accuracy_GET(pack) == (float)2.7551856E38F);
    assert(p232_vn_GET(pack) == (float) -1.835027E38F);
    assert(p232_alt_GET(pack) == (float) -2.4603725E38F);
    assert(p232_ve_GET(pack) == (float) -2.3348002E37F);
    assert(p232_hdop_GET(pack) == (float) -4.1281863E37F);
    assert(p232_lat_GET(pack) == (int32_t) -990445892);
    assert(p232_time_usec_GET(pack) == (uint64_t)6681706430048135615L);
    assert(p232_vd_GET(pack) == (float) -3.233995E38F);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)82);
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)48189);
    assert(p232_vdop_GET(pack) == (float)2.7464287E38F);
    assert(p232_ignore_flags_GET(pack) == e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT);
};


void c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)236, (uint8_t)129, (uint8_t)95, (uint8_t)60, (uint8_t)162, (uint8_t)173, (uint8_t)122, (uint8_t)94, (uint8_t)49, (uint8_t)100, (uint8_t)82, (uint8_t)206, (uint8_t)136, (uint8_t)95, (uint8_t)229, (uint8_t)99, (uint8_t)65, (uint8_t)30, (uint8_t)121, (uint8_t)35, (uint8_t)246, (uint8_t)166, (uint8_t)37, (uint8_t)249, (uint8_t)208, (uint8_t)87, (uint8_t)74, (uint8_t)7, (uint8_t)58, (uint8_t)49, (uint8_t)207, (uint8_t)148, (uint8_t)113, (uint8_t)148, (uint8_t)179, (uint8_t)98, (uint8_t)86, (uint8_t)66, (uint8_t)214, (uint8_t)3, (uint8_t)151, (uint8_t)241, (uint8_t)233, (uint8_t)228, (uint8_t)122, (uint8_t)231, (uint8_t)84, (uint8_t)234, (uint8_t)224, (uint8_t)176, (uint8_t)87, (uint8_t)190, (uint8_t)210, (uint8_t)222, (uint8_t)93, (uint8_t)71, (uint8_t)34, (uint8_t)130, (uint8_t)29, (uint8_t)65, (uint8_t)200, (uint8_t)56, (uint8_t)44, (uint8_t)50, (uint8_t)90, (uint8_t)235, (uint8_t)96, (uint8_t)148, (uint8_t)235, (uint8_t)226, (uint8_t)25, (uint8_t)198, (uint8_t)210, (uint8_t)72, (uint8_t)113, (uint8_t)240, (uint8_t)69, (uint8_t)187, (uint8_t)228, (uint8_t)80, (uint8_t)0, (uint8_t)201, (uint8_t)167, (uint8_t)100, (uint8_t)247, (uint8_t)202, (uint8_t)65, (uint8_t)42, (uint8_t)146, (uint8_t)137, (uint8_t)130, (uint8_t)163, (uint8_t)248, (uint8_t)234, (uint8_t)165, (uint8_t)22, (uint8_t)86, (uint8_t)211, (uint8_t)73, (uint8_t)170, (uint8_t)9, (uint8_t)98, (uint8_t)211, (uint8_t)95, (uint8_t)83, (uint8_t)204, (uint8_t)114, (uint8_t)176, (uint8_t)51, (uint8_t)32, (uint8_t)183, (uint8_t)104, (uint8_t)201, (uint8_t)19, (uint8_t)241, (uint8_t)248, (uint8_t)116, (uint8_t)184, (uint8_t)147, (uint8_t)222, (uint8_t)171, (uint8_t)254, (uint8_t)34, (uint8_t)148, (uint8_t)251, (uint8_t)30, (uint8_t)50, (uint8_t)21, (uint8_t)85, (uint8_t)131, (uint8_t)22, (uint8_t)224, (uint8_t)76, (uint8_t)90, (uint8_t)77, (uint8_t)90, (uint8_t)98, (uint8_t)6, (uint8_t)244, (uint8_t)225, (uint8_t)152, (uint8_t)58, (uint8_t)62, (uint8_t)211, (uint8_t)86, (uint8_t)253, (uint8_t)173, (uint8_t)252, (uint8_t)201, (uint8_t)157, (uint8_t)105, (uint8_t)22, (uint8_t)131, (uint8_t)56, (uint8_t)195, (uint8_t)19, (uint8_t)190, (uint8_t)73, (uint8_t)202, (uint8_t)72, (uint8_t)58, (uint8_t)41, (uint8_t)132, (uint8_t)113, (uint8_t)186, (uint8_t)91, (uint8_t)202, (uint8_t)18, (uint8_t)216, (uint8_t)115, (uint8_t)225, (uint8_t)102, (uint8_t)31, (uint8_t)129, (uint8_t)218, (uint8_t)180, (uint8_t)40, (uint8_t)39, (uint8_t)71, (uint8_t)81} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)32);
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)206);
};


void c_LoopBackDemoChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t) -2548);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t)15);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)242);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)237);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)6);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)21);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t) -19);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t) -127);
    assert(p234_longitude_GET(pack) == (int32_t)310876586);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t) -17593);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t) -116);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t) -14432);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)38892);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)54822);
    assert(p234_custom_mode_GET(pack) == (uint32_t)1028590280L);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)166);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t) -19855);
    assert(p234_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)213);
    assert(p234_latitude_GET(pack) == (int32_t)332871058);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t) -28571);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)17);
};


void c_LoopBackDemoChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_clipping_0_GET(pack) == (uint32_t)1331707777L);
    assert(p241_vibration_z_GET(pack) == (float)1.952359E38F);
    assert(p241_clipping_1_GET(pack) == (uint32_t)274320862L);
    assert(p241_time_usec_GET(pack) == (uint64_t)2649767703318378168L);
    assert(p241_vibration_x_GET(pack) == (float) -1.8421609E37F);
    assert(p241_vibration_y_GET(pack) == (float) -3.3649986E38F);
    assert(p241_clipping_2_GET(pack) == (uint32_t)1990133049L);
};


void c_LoopBackDemoChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_approach_y_GET(pack) == (float)2.9953799E38F);
    assert(p242_longitude_GET(pack) == (int32_t) -174092834);
    assert(p242_altitude_GET(pack) == (int32_t) -199153132);
    {
        float exemplary[] =  {-2.4927116E38F, -9.354272E37F, -4.800433E37F, -8.0089835E37F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_approach_x_GET(pack) == (float)9.832744E35F);
    assert(p242_time_usec_TRY(ph) == (uint64_t)7403138715367473169L);
    assert(p242_approach_z_GET(pack) == (float)3.945386E37F);
    assert(p242_latitude_GET(pack) == (int32_t)357603074);
    assert(p242_y_GET(pack) == (float) -2.8488717E38F);
    assert(p242_x_GET(pack) == (float) -3.1197646E38F);
    assert(p242_z_GET(pack) == (float)1.704071E38F);
};


void c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_altitude_GET(pack) == (int32_t) -2020291669);
    assert(p243_latitude_GET(pack) == (int32_t)546257757);
    assert(p243_time_usec_TRY(ph) == (uint64_t)3624049427776006904L);
    assert(p243_z_GET(pack) == (float) -3.238082E38F);
    assert(p243_approach_y_GET(pack) == (float)3.3198613E38F);
    assert(p243_longitude_GET(pack) == (int32_t)1654014446);
    {
        float exemplary[] =  {2.097284E38F, -4.733057E35F, 3.2660475E37F, 6.231203E37F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_y_GET(pack) == (float)8.794241E36F);
    assert(p243_x_GET(pack) == (float)3.0121123E38F);
    assert(p243_approach_z_GET(pack) == (float) -1.4029907E38F);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p243_approach_x_GET(pack) == (float) -1.0615717E38F);
};


void c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_interval_us_GET(pack) == (int32_t)511832011);
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)19404);
};


void c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND);
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW);
};


void c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)25900);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t)23333);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)51501);
    assert(p246_lat_GET(pack) == (int32_t)668032009);
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)28861);
    assert(p246_flags_GET(pack) == e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HIGHLY_MANUV);
    assert(p246_callsign_LEN(ph) == 8);
    {
        char16_t * exemplary = u"muligytd";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_lon_GET(pack) == (int32_t)1883219111);
    assert(p246_altitude_GET(pack) == (int32_t) -1954982159);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)3586869764L);
};


void c_LoopBackDemoChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_altitude_minimum_delta_GET(pack) == (float)2.7698488E38F);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_HOVER);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float)9.646423E37F);
    assert(p247_time_to_minimum_delta_GET(pack) == (float)1.1264116E38F);
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE);
    assert(p247_id_GET(pack) == (uint32_t)2355477366L);
};


void c_LoopBackDemoChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)120);
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)22802);
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)201);
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)110);
    {
        uint8_t exemplary[] =  {(uint8_t)73, (uint8_t)211, (uint8_t)108, (uint8_t)82, (uint8_t)144, (uint8_t)109, (uint8_t)33, (uint8_t)182, (uint8_t)143, (uint8_t)89, (uint8_t)74, (uint8_t)103, (uint8_t)253, (uint8_t)236, (uint8_t)37, (uint8_t)65, (uint8_t)15, (uint8_t)63, (uint8_t)179, (uint8_t)139, (uint8_t)153, (uint8_t)162, (uint8_t)117, (uint8_t)76, (uint8_t)239, (uint8_t)27, (uint8_t)38, (uint8_t)128, (uint8_t)200, (uint8_t)224, (uint8_t)154, (uint8_t)142, (uint8_t)87, (uint8_t)94, (uint8_t)192, (uint8_t)183, (uint8_t)3, (uint8_t)66, (uint8_t)17, (uint8_t)212, (uint8_t)164, (uint8_t)182, (uint8_t)245, (uint8_t)16, (uint8_t)187, (uint8_t)159, (uint8_t)243, (uint8_t)184, (uint8_t)142, (uint8_t)110, (uint8_t)93, (uint8_t)5, (uint8_t)213, (uint8_t)15, (uint8_t)196, (uint8_t)247, (uint8_t)227, (uint8_t)73, (uint8_t)54, (uint8_t)79, (uint8_t)153, (uint8_t)112, (uint8_t)56, (uint8_t)42, (uint8_t)221, (uint8_t)169, (uint8_t)45, (uint8_t)172, (uint8_t)253, (uint8_t)188, (uint8_t)187, (uint8_t)254, (uint8_t)165, (uint8_t)28, (uint8_t)136, (uint8_t)203, (uint8_t)63, (uint8_t)102, (uint8_t)14, (uint8_t)128, (uint8_t)129, (uint8_t)102, (uint8_t)132, (uint8_t)201, (uint8_t)21, (uint8_t)55, (uint8_t)240, (uint8_t)49, (uint8_t)11, (uint8_t)126, (uint8_t)113, (uint8_t)1, (uint8_t)235, (uint8_t)178, (uint8_t)155, (uint8_t)189, (uint8_t)105, (uint8_t)72, (uint8_t)70, (uint8_t)96, (uint8_t)77, (uint8_t)34, (uint8_t)77, (uint8_t)223, (uint8_t)251, (uint8_t)43, (uint8_t)16, (uint8_t)208, (uint8_t)221, (uint8_t)83, (uint8_t)90, (uint8_t)12, (uint8_t)14, (uint8_t)26, (uint8_t)92, (uint8_t)136, (uint8_t)169, (uint8_t)105, (uint8_t)148, (uint8_t)227, (uint8_t)48, (uint8_t)78, (uint8_t)196, (uint8_t)223, (uint8_t)71, (uint8_t)168, (uint8_t)213, (uint8_t)202, (uint8_t)243, (uint8_t)249, (uint8_t)96, (uint8_t)229, (uint8_t)146, (uint8_t)159, (uint8_t)245, (uint8_t)34, (uint8_t)7, (uint8_t)42, (uint8_t)204, (uint8_t)40, (uint8_t)160, (uint8_t)171, (uint8_t)200, (uint8_t)15, (uint8_t)253, (uint8_t)213, (uint8_t)145, (uint8_t)166, (uint8_t)182, (uint8_t)95, (uint8_t)140, (uint8_t)78, (uint8_t)161, (uint8_t)233, (uint8_t)150, (uint8_t)21, (uint8_t)121, (uint8_t)29, (uint8_t)207, (uint8_t)41, (uint8_t)75, (uint8_t)75, (uint8_t)165, (uint8_t)30, (uint8_t)171, (uint8_t)87, (uint8_t)53, (uint8_t)20, (uint8_t)98, (uint8_t)167, (uint8_t)19, (uint8_t)210, (uint8_t)132, (uint8_t)150, (uint8_t)33, (uint8_t)43, (uint8_t)143, (uint8_t)209, (uint8_t)44, (uint8_t)201, (uint8_t)29, (uint8_t)16, (uint8_t)251, (uint8_t)3, (uint8_t)115, (uint8_t)68, (uint8_t)135, (uint8_t)83, (uint8_t)121, (uint8_t)27, (uint8_t)36, (uint8_t)182, (uint8_t)77, (uint8_t)143, (uint8_t)109, (uint8_t)214, (uint8_t)156, (uint8_t)35, (uint8_t)164, (uint8_t)214, (uint8_t)21, (uint8_t)154, (uint8_t)29, (uint8_t)215, (uint8_t)0, (uint8_t)173, (uint8_t)55, (uint8_t)96, (uint8_t)160, (uint8_t)74, (uint8_t)110, (uint8_t)43, (uint8_t)237, (uint8_t)160, (uint8_t)83, (uint8_t)248, (uint8_t)226, (uint8_t)87, (uint8_t)142, (uint8_t)136, (uint8_t)118, (uint8_t)141, (uint8_t)156, (uint8_t)79, (uint8_t)118, (uint8_t)149, (uint8_t)88, (uint8_t)119, (uint8_t)242, (uint8_t)0, (uint8_t)72, (uint8_t)170, (uint8_t)68, (uint8_t)226, (uint8_t)164, (uint8_t)117, (uint8_t)100, (uint8_t)167, (uint8_t)211, (uint8_t)101, (uint8_t)236, (uint8_t)21, (uint8_t)163, (uint8_t)103, (uint8_t)90, (uint8_t)251, (uint8_t)223, (uint8_t)113, (uint8_t)123} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    {
        int8_t exemplary[] =  {(int8_t) -13, (int8_t)34, (int8_t) -55, (int8_t) -38, (int8_t) -108, (int8_t) -60, (int8_t) -28, (int8_t) -111, (int8_t)81, (int8_t) -106, (int8_t)95, (int8_t)34, (int8_t)57, (int8_t) -89, (int8_t)122, (int8_t)13, (int8_t) -52, (int8_t) -126, (int8_t) -89, (int8_t)7, (int8_t) -126, (int8_t) -69, (int8_t) -82, (int8_t) -13, (int8_t)110, (int8_t)103, (int8_t) -22, (int8_t)2, (int8_t) -49, (int8_t)48, (int8_t) -116, (int8_t) -119} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)52);
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)30239);
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)77);
};


void c_LoopBackDemoChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_x_GET(pack) == (float) -1.6659714E38F);
    assert(p250_name_LEN(ph) == 6);
    {
        char16_t * exemplary = u"gwzpbe";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p250_y_GET(pack) == (float) -1.0727332E38F);
    assert(p250_time_usec_GET(pack) == (uint64_t)3027300410375224418L);
    assert(p250_z_GET(pack) == (float) -1.9761509E38F);
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_name_LEN(ph) == 5);
    {
        char16_t * exemplary = u"enkzi";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)2555712429L);
    assert(p251_value_GET(pack) == (float)9.119059E37F);
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)397974298L);
    assert(p252_value_GET(pack) == (int32_t)1319768634);
    assert(p252_name_LEN(ph) == 3);
    {
        char16_t * exemplary = u"rTz";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_text_LEN(ph) == 25);
    {
        char16_t * exemplary = u"xluvYxfbtbnlqpgdbguleoqce";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 50);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_INFO);
};


void c_LoopBackDemoChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_value_GET(pack) == (float)3.1525967E38F);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)303994040L);
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)58);
};


void c_LoopBackDemoChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)105);
    {
        uint8_t exemplary[] =  {(uint8_t)208, (uint8_t)123, (uint8_t)38, (uint8_t)35, (uint8_t)202, (uint8_t)18, (uint8_t)88, (uint8_t)244, (uint8_t)249, (uint8_t)204, (uint8_t)243, (uint8_t)151, (uint8_t)189, (uint8_t)130, (uint8_t)186, (uint8_t)153, (uint8_t)21, (uint8_t)19, (uint8_t)177, (uint8_t)24, (uint8_t)126, (uint8_t)58, (uint8_t)167, (uint8_t)18, (uint8_t)33, (uint8_t)176, (uint8_t)243, (uint8_t)172, (uint8_t)228, (uint8_t)184, (uint8_t)118, (uint8_t)200} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)117);
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)2820896605047503436L);
};


void c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)1758702292L);
    assert(p257_last_change_ms_GET(pack) == (uint32_t)574848545L);
};


void c_LoopBackDemoChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_tune_LEN(ph) == 21);
    {
        char16_t * exemplary = u"vhyultpuuwRIswvlNjozd";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 42);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)115);
};


void c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)36674);
    assert(p259_sensor_size_h_GET(pack) == (float)1.7616076E38F);
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)2417296277L);
    assert(p259_flags_GET(pack) == e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)2445);
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)204);
    assert(p259_firmware_version_GET(pack) == (uint32_t)497768214L);
    assert(p259_sensor_size_v_GET(pack) == (float) -5.6201594E37F);
    assert(p259_focal_length_GET(pack) == (float)1.1809058E38F);
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)62221);
    assert(p259_cam_definition_uri_LEN(ph) == 88);
    {
        char16_t * exemplary = u"kjVtxxhNygdypsNrxsuDacxlqtxwynzziqcicfuekxmqMslpuvveiqafeEqqCwkbZqudvQepwropbdgysbcnidgz";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 176);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)32, (uint8_t)176, (uint8_t)89, (uint8_t)184, (uint8_t)34, (uint8_t)144, (uint8_t)250, (uint8_t)168, (uint8_t)168, (uint8_t)206, (uint8_t)210, (uint8_t)64, (uint8_t)63, (uint8_t)11, (uint8_t)81, (uint8_t)2, (uint8_t)94, (uint8_t)195, (uint8_t)169, (uint8_t)104, (uint8_t)238, (uint8_t)17, (uint8_t)208, (uint8_t)160, (uint8_t)243, (uint8_t)130, (uint8_t)152, (uint8_t)46, (uint8_t)219, (uint8_t)50, (uint8_t)46, (uint8_t)93} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)168, (uint8_t)107, (uint8_t)62, (uint8_t)217, (uint8_t)82, (uint8_t)96, (uint8_t)70, (uint8_t)188, (uint8_t)95, (uint8_t)22, (uint8_t)15, (uint8_t)253, (uint8_t)57, (uint8_t)158, (uint8_t)124, (uint8_t)26, (uint8_t)82, (uint8_t)128, (uint8_t)37, (uint8_t)61, (uint8_t)227, (uint8_t)171, (uint8_t)93, (uint8_t)189, (uint8_t)122, (uint8_t)56, (uint8_t)130, (uint8_t)210, (uint8_t)77, (uint8_t)236, (uint8_t)19, (uint8_t)171} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)208363571L);
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_VIDEO);
};


void c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)247);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)106);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)96763514L);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p261_total_capacity_GET(pack) == (float)1.9682963E38F);
    assert(p261_used_capacity_GET(pack) == (float) -1.0681241E38F);
    assert(p261_available_capacity_GET(pack) == (float)8.396993E37F);
    assert(p261_read_speed_GET(pack) == (float)3.3001087E38F);
    assert(p261_write_speed_GET(pack) == (float)1.6166886E38F);
};


void c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_available_capacity_GET(pack) == (float) -1.3131566E37F);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)79);
    assert(p262_image_interval_GET(pack) == (float)7.1712074E37F);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)2808172676L);
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)3273184033L);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)243);
};


void c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t)61);
    assert(p263_image_index_GET(pack) == (int32_t)1682594756);
    assert(p263_lat_GET(pack) == (int32_t)328399714);
    assert(p263_alt_GET(pack) == (int32_t)2048365045);
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)4206286646L);
    assert(p263_time_utc_GET(pack) == (uint64_t)9197667099251689232L);
    assert(p263_lon_GET(pack) == (int32_t) -1124557312);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)15);
    {
        float exemplary[] =  {-3.0787955E38F, 2.9240276E38F, -6.805334E37F, -8.451733E37F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_relative_alt_GET(pack) == (int32_t)124379189);
    assert(p263_file_url_LEN(ph) == 72);
    {
        char16_t * exemplary = u"pvshfpYjuljnxCloguqvoiaXzmDZijFyxyqHahyrdscQqrrylrWahwddhtgYTYkcdgwtWhUf";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)5784841417719717045L);
    assert(p264_flight_uuid_GET(pack) == (uint64_t)6490037615656584532L);
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)2513094030L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)3758199398182636261L);
};


void c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_yaw_GET(pack) == (float) -2.64721E38F);
    assert(p265_roll_GET(pack) == (float) -1.6656885E38F);
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)1579153439L);
    assert(p265_pitch_GET(pack) == (float)1.3567627E38F);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)33);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)14121);
    {
        uint8_t exemplary[] =  {(uint8_t)123, (uint8_t)70, (uint8_t)35, (uint8_t)177, (uint8_t)182, (uint8_t)108, (uint8_t)209, (uint8_t)74, (uint8_t)188, (uint8_t)131, (uint8_t)232, (uint8_t)148, (uint8_t)188, (uint8_t)135, (uint8_t)4, (uint8_t)85, (uint8_t)23, (uint8_t)124, (uint8_t)168, (uint8_t)252, (uint8_t)2, (uint8_t)135, (uint8_t)244, (uint8_t)238, (uint8_t)106, (uint8_t)73, (uint8_t)170, (uint8_t)227, (uint8_t)239, (uint8_t)76, (uint8_t)228, (uint8_t)141, (uint8_t)62, (uint8_t)176, (uint8_t)47, (uint8_t)57, (uint8_t)157, (uint8_t)10, (uint8_t)73, (uint8_t)112, (uint8_t)167, (uint8_t)51, (uint8_t)72, (uint8_t)163, (uint8_t)217, (uint8_t)114, (uint8_t)130, (uint8_t)4, (uint8_t)89, (uint8_t)110, (uint8_t)240, (uint8_t)148, (uint8_t)187, (uint8_t)8, (uint8_t)169, (uint8_t)246, (uint8_t)121, (uint8_t)161, (uint8_t)39, (uint8_t)9, (uint8_t)92, (uint8_t)138, (uint8_t)217, (uint8_t)230, (uint8_t)161, (uint8_t)93, (uint8_t)87, (uint8_t)84, (uint8_t)47, (uint8_t)151, (uint8_t)177, (uint8_t)52, (uint8_t)251, (uint8_t)238, (uint8_t)77, (uint8_t)115, (uint8_t)67, (uint8_t)19, (uint8_t)85, (uint8_t)84, (uint8_t)226, (uint8_t)225, (uint8_t)52, (uint8_t)246, (uint8_t)127, (uint8_t)213, (uint8_t)38, (uint8_t)250, (uint8_t)158, (uint8_t)142, (uint8_t)45, (uint8_t)250, (uint8_t)245, (uint8_t)159, (uint8_t)191, (uint8_t)140, (uint8_t)227, (uint8_t)71, (uint8_t)40, (uint8_t)169, (uint8_t)176, (uint8_t)114, (uint8_t)139, (uint8_t)24, (uint8_t)11, (uint8_t)246, (uint8_t)83, (uint8_t)242, (uint8_t)47, (uint8_t)5, (uint8_t)94, (uint8_t)71, (uint8_t)229, (uint8_t)139, (uint8_t)86, (uint8_t)57, (uint8_t)42, (uint8_t)144, (uint8_t)8, (uint8_t)59, (uint8_t)170, (uint8_t)120, (uint8_t)201, (uint8_t)245, (uint8_t)112, (uint8_t)64, (uint8_t)210, (uint8_t)103, (uint8_t)63, (uint8_t)95, (uint8_t)107, (uint8_t)145, (uint8_t)229, (uint8_t)24, (uint8_t)35, (uint8_t)210, (uint8_t)184, (uint8_t)198, (uint8_t)84, (uint8_t)13, (uint8_t)57, (uint8_t)17, (uint8_t)137, (uint8_t)243, (uint8_t)95, (uint8_t)183, (uint8_t)102, (uint8_t)3, (uint8_t)59, (uint8_t)40, (uint8_t)223, (uint8_t)14, (uint8_t)215, (uint8_t)253, (uint8_t)4, (uint8_t)94, (uint8_t)254, (uint8_t)63, (uint8_t)250, (uint8_t)204, (uint8_t)53, (uint8_t)91, (uint8_t)124, (uint8_t)53, (uint8_t)147, (uint8_t)189, (uint8_t)169, (uint8_t)143, (uint8_t)212, (uint8_t)229, (uint8_t)227, (uint8_t)255, (uint8_t)205, (uint8_t)130, (uint8_t)70, (uint8_t)50, (uint8_t)16, (uint8_t)201, (uint8_t)207, (uint8_t)24, (uint8_t)203, (uint8_t)40, (uint8_t)255, (uint8_t)247, (uint8_t)129, (uint8_t)44, (uint8_t)126, (uint8_t)198, (uint8_t)45, (uint8_t)193, (uint8_t)81, (uint8_t)116, (uint8_t)114, (uint8_t)134, (uint8_t)118, (uint8_t)33, (uint8_t)106, (uint8_t)184, (uint8_t)145, (uint8_t)255, (uint8_t)103, (uint8_t)121, (uint8_t)117, (uint8_t)98, (uint8_t)183, (uint8_t)240, (uint8_t)52, (uint8_t)94, (uint8_t)189, (uint8_t)118, (uint8_t)96, (uint8_t)103, (uint8_t)24, (uint8_t)115, (uint8_t)44, (uint8_t)111, (uint8_t)40, (uint8_t)218, (uint8_t)112, (uint8_t)100, (uint8_t)8, (uint8_t)8, (uint8_t)216, (uint8_t)138, (uint8_t)66, (uint8_t)9, (uint8_t)30, (uint8_t)190, (uint8_t)254, (uint8_t)144, (uint8_t)143, (uint8_t)119, (uint8_t)255, (uint8_t)34, (uint8_t)26, (uint8_t)233, (uint8_t)31, (uint8_t)170, (uint8_t)71, (uint8_t)178, (uint8_t)117, (uint8_t)130, (uint8_t)250, (uint8_t)115, (uint8_t)7, (uint8_t)35, (uint8_t)202, (uint8_t)131, (uint8_t)131} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)111);
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)184);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)35);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)169);
    {
        uint8_t exemplary[] =  {(uint8_t)153, (uint8_t)203, (uint8_t)181, (uint8_t)25, (uint8_t)207, (uint8_t)152, (uint8_t)186, (uint8_t)68, (uint8_t)35, (uint8_t)228, (uint8_t)69, (uint8_t)205, (uint8_t)164, (uint8_t)235, (uint8_t)182, (uint8_t)77, (uint8_t)205, (uint8_t)140, (uint8_t)179, (uint8_t)237, (uint8_t)117, (uint8_t)119, (uint8_t)71, (uint8_t)254, (uint8_t)173, (uint8_t)71, (uint8_t)6, (uint8_t)155, (uint8_t)246, (uint8_t)233, (uint8_t)202, (uint8_t)176, (uint8_t)181, (uint8_t)15, (uint8_t)249, (uint8_t)173, (uint8_t)7, (uint8_t)150, (uint8_t)227, (uint8_t)112, (uint8_t)123, (uint8_t)186, (uint8_t)253, (uint8_t)177, (uint8_t)41, (uint8_t)28, (uint8_t)249, (uint8_t)220, (uint8_t)138, (uint8_t)28, (uint8_t)39, (uint8_t)58, (uint8_t)249, (uint8_t)158, (uint8_t)35, (uint8_t)14, (uint8_t)92, (uint8_t)162, (uint8_t)110, (uint8_t)209, (uint8_t)19, (uint8_t)165, (uint8_t)135, (uint8_t)224, (uint8_t)28, (uint8_t)144, (uint8_t)239, (uint8_t)42, (uint8_t)113, (uint8_t)99, (uint8_t)105, (uint8_t)233, (uint8_t)155, (uint8_t)181, (uint8_t)146, (uint8_t)189, (uint8_t)171, (uint8_t)75, (uint8_t)130, (uint8_t)35, (uint8_t)192, (uint8_t)196, (uint8_t)41, (uint8_t)111, (uint8_t)7, (uint8_t)149, (uint8_t)50, (uint8_t)119, (uint8_t)55, (uint8_t)33, (uint8_t)62, (uint8_t)111, (uint8_t)63, (uint8_t)14, (uint8_t)100, (uint8_t)167, (uint8_t)129, (uint8_t)210, (uint8_t)7, (uint8_t)174, (uint8_t)144, (uint8_t)192, (uint8_t)214, (uint8_t)123, (uint8_t)183, (uint8_t)5, (uint8_t)155, (uint8_t)227, (uint8_t)56, (uint8_t)30, (uint8_t)188, (uint8_t)78, (uint8_t)20, (uint8_t)224, (uint8_t)183, (uint8_t)146, (uint8_t)13, (uint8_t)84, (uint8_t)59, (uint8_t)49, (uint8_t)91, (uint8_t)52, (uint8_t)60, (uint8_t)129, (uint8_t)114, (uint8_t)6, (uint8_t)144, (uint8_t)172, (uint8_t)173, (uint8_t)156, (uint8_t)19, (uint8_t)92, (uint8_t)52, (uint8_t)124, (uint8_t)67, (uint8_t)47, (uint8_t)23, (uint8_t)87, (uint8_t)44, (uint8_t)9, (uint8_t)218, (uint8_t)145, (uint8_t)177, (uint8_t)106, (uint8_t)88, (uint8_t)154, (uint8_t)71, (uint8_t)86, (uint8_t)137, (uint8_t)60, (uint8_t)7, (uint8_t)212, (uint8_t)133, (uint8_t)35, (uint8_t)167, (uint8_t)134, (uint8_t)112, (uint8_t)22, (uint8_t)209, (uint8_t)101, (uint8_t)8, (uint8_t)179, (uint8_t)244, (uint8_t)33, (uint8_t)237, (uint8_t)129, (uint8_t)47, (uint8_t)209, (uint8_t)132, (uint8_t)158, (uint8_t)139, (uint8_t)173, (uint8_t)105, (uint8_t)66, (uint8_t)211, (uint8_t)10, (uint8_t)18, (uint8_t)78, (uint8_t)209, (uint8_t)124, (uint8_t)33, (uint8_t)164, (uint8_t)55, (uint8_t)223, (uint8_t)193, (uint8_t)84, (uint8_t)11, (uint8_t)248, (uint8_t)146, (uint8_t)64, (uint8_t)111, (uint8_t)145, (uint8_t)205, (uint8_t)203, (uint8_t)124, (uint8_t)248, (uint8_t)20, (uint8_t)180, (uint8_t)228, (uint8_t)9, (uint8_t)21, (uint8_t)30, (uint8_t)116, (uint8_t)47, (uint8_t)36, (uint8_t)33, (uint8_t)42, (uint8_t)89, (uint8_t)36, (uint8_t)118, (uint8_t)66, (uint8_t)44, (uint8_t)0, (uint8_t)17, (uint8_t)4, (uint8_t)236, (uint8_t)250, (uint8_t)131, (uint8_t)104, (uint8_t)6, (uint8_t)16, (uint8_t)46, (uint8_t)233, (uint8_t)147, (uint8_t)238, (uint8_t)244, (uint8_t)14, (uint8_t)0, (uint8_t)109, (uint8_t)163, (uint8_t)80, (uint8_t)197, (uint8_t)127, (uint8_t)174, (uint8_t)157, (uint8_t)142, (uint8_t)44, (uint8_t)42, (uint8_t)126, (uint8_t)112, (uint8_t)80, (uint8_t)174, (uint8_t)113, (uint8_t)217, (uint8_t)212, (uint8_t)79, (uint8_t)18, (uint8_t)80, (uint8_t)252} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)57975);
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)205);
};


void c_LoopBackDemoChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)5);
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)13);
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)64146);
};


void c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_framerate_GET(pack) == (float)1.7457027E38F);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)21256);
    assert(p269_uri_LEN(ph) == 167);
    {
        char16_t * exemplary = u"ZgqwzkpafAbqbwbkgQRbocaxvfyobfqocoundbbdbNGhtlanopokkkCksirgzdotakrilkpmPlhujiaBvcOrvnsstbznOnyyopqqlumcuyPlfnvaklrdhqynugtvMVMyBQfjyjRmtaimbupwbkwihEoxmcazvhsBtwdsbyg";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 334);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)17043);
    assert(p269_bitrate_GET(pack) == (uint32_t)1598502410L);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)110);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)40711);
};


void c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)12784);
    assert(p270_framerate_GET(pack) == (float) -2.186394E38F);
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)116);
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)16);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)4341);
    assert(p270_uri_LEN(ph) == 56);
    {
        char16_t * exemplary = u"gBkGzkqhuevdrcgdqjaDcvxqnvokWsuqvvjveTdavpkfmciKaxljbfNx";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 112);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)23944);
    assert(p270_bitrate_GET(pack) == (uint32_t)180048792L);
};


void c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_ssid_LEN(ph) == 15);
    {
        char16_t * exemplary = u"CqbTepxRyYfwadg";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_password_LEN(ph) == 54);
    {
        char16_t * exemplary = u"ikirlmzdcbkshrtmgzHMahfvvrrdzamvWzyfsemcatcbxuujMdaJIL";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 108);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)64, (uint8_t)211, (uint8_t)242, (uint8_t)211, (uint8_t)9, (uint8_t)254, (uint8_t)168, (uint8_t)235} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)58237);
    {
        uint8_t exemplary[] =  {(uint8_t)101, (uint8_t)71, (uint8_t)147, (uint8_t)16, (uint8_t)221, (uint8_t)23, (uint8_t)123, (uint8_t)170} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)39206);
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)53058);
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)84);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR);
    assert(p310_uptime_sec_GET(pack) == (uint32_t)1216213891L);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION);
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)51041);
    assert(p310_time_usec_GET(pack) == (uint64_t)6217231610304152909L);
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_name_LEN(ph) == 24);
    {
        char16_t * exemplary = u"YbsmymkkIpkibbltnipdjihu";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 48);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)134);
    {
        uint8_t exemplary[] =  {(uint8_t)11, (uint8_t)225, (uint8_t)141, (uint8_t)245, (uint8_t)6, (uint8_t)35, (uint8_t)221, (uint8_t)153, (uint8_t)37, (uint8_t)5, (uint8_t)62, (uint8_t)171, (uint8_t)207, (uint8_t)143, (uint8_t)229, (uint8_t)169} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_uptime_sec_GET(pack) == (uint32_t)1921693059L);
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)153);
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)1035911788L);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)186);
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)79);
    assert(p311_time_usec_GET(pack) == (uint64_t)7863210411443711174L);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_param_id_LEN(ph) == 16);
    {
        char16_t * exemplary = u"arYurbapumkopPpc";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)189);
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t)22898);
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)208);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)51);
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)86);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)50562);
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)28665);
    assert(p322_param_id_LEN(ph) == 14);
    {
        char16_t * exemplary = u"qwsyajbbxbpwik";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 28);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8);
    assert(p322_param_value_LEN(ph) == 69);
    {
        char16_t * exemplary = u"pirnrxqleuzyhkgxrjhtxtsdibjrobmrJhhntBbrqZyeqerqWsvshsrchkikhgPhhRork";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 138);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_param_id_LEN(ph) == 11);
    {
        char16_t * exemplary = u"dEaoifxgsfR";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 22);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32);
    assert(p323_param_value_LEN(ph) == 97);
    {
        char16_t * exemplary = u"xllyjpfbksqkwyjrpAukjlgomtrxxikarpdrsjywtvwvrcunfeiDlmoviowzuqukuuKahMmcyyeKyajpryduykagcmqqhYlih";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 194);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)156);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_id_LEN(ph) == 14);
    {
        char16_t * exemplary = u"OBmwpmUrDbictj";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 28);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_ACCEPTED);
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32);
    assert(p324_param_value_LEN(ph) == 14);
    {
        char16_t * exemplary = u"kvoalrkghepljs";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 28);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)16426);
    assert(p330_time_usec_GET(pack) == (uint64_t)6722039981066533068L);
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)52893);
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN);
    {
        uint16_t exemplary[] =  {(uint16_t)52167, (uint16_t)49703, (uint16_t)34223, (uint16_t)34217, (uint16_t)44194, (uint16_t)6242, (uint16_t)3768, (uint16_t)51858, (uint16_t)28335, (uint16_t)53773, (uint16_t)32276, (uint16_t)61293, (uint16_t)4927, (uint16_t)42861, (uint16_t)10461, (uint16_t)30623, (uint16_t)49322, (uint16_t)25991, (uint16_t)48461, (uint16_t)40976, (uint16_t)10495, (uint16_t)36480, (uint16_t)8005, (uint16_t)43062, (uint16_t)62196, (uint16_t)16768, (uint16_t)16273, (uint16_t)31916, (uint16_t)38353, (uint16_t)43956, (uint16_t)62278, (uint16_t)29595, (uint16_t)55264, (uint16_t)55795, (uint16_t)47947, (uint16_t)14882, (uint16_t)20367, (uint16_t)62009, (uint16_t)31601, (uint16_t)62541, (uint16_t)38803, (uint16_t)2276, (uint16_t)41689, (uint16_t)54276, (uint16_t)56617, (uint16_t)62689, (uint16_t)6760, (uint16_t)42473, (uint16_t)43218, (uint16_t)42858, (uint16_t)3409, (uint16_t)27087, (uint16_t)13323, (uint16_t)63926, (uint16_t)24153, (uint16_t)11993, (uint16_t)56408, (uint16_t)61331, (uint16_t)16106, (uint16_t)8310, (uint16_t)18158, (uint16_t)10199, (uint16_t)23348, (uint16_t)34581, (uint16_t)59870, (uint16_t)32542, (uint16_t)4948, (uint16_t)50136, (uint16_t)19799, (uint16_t)26650, (uint16_t)27749, (uint16_t)7226} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)9);
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
        p0_custom_mode_SET((uint32_t)608911116L, PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_PARAFOIL, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_ACTIVE, PH.base.pack) ;
        p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYS_STATUS_1(), &PH);
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)32020, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)22837, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG2, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)50544, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)35114, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)34420, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t) -8982, PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)61593, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)56848, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_VISION_POSITION, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t) -72, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)20237, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_unix_usec_SET((uint64_t)2043441244541234898L, PH.base.pack) ;
        p2_time_boot_ms_SET((uint32_t)2091648250L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_afy_SET((float)1.5738301E38F, PH.base.pack) ;
        p3_afz_SET((float) -1.6588216E38F, PH.base.pack) ;
        p3_z_SET((float)1.987928E38F, PH.base.pack) ;
        p3_vy_SET((float)3.5381251E37F, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)53046, PH.base.pack) ;
        p3_yaw_SET((float)1.2048585E38F, PH.base.pack) ;
        p3_yaw_rate_SET((float)1.7318629E37F, PH.base.pack) ;
        p3_vz_SET((float)2.566778E37F, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        p3_afx_SET((float) -2.1753657E38F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)1174442147L, PH.base.pack) ;
        p3_x_SET((float) -2.934203E38F, PH.base.pack) ;
        p3_vx_SET((float)7.186914E36F, PH.base.pack) ;
        p3_y_SET((float) -2.0777373E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PING_4(), &PH);
        p4_target_component_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        p4_seq_SET((uint32_t)36322742L, PH.base.pack) ;
        p4_target_system_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
        p4_time_usec_SET((uint64_t)2596471207447125292L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        p5_control_request_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        p5_target_system_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p5_version_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        {
            char16_t* passkey = u"zetmppgvwxnbKucpaqtn";
            p5_passkey_SET_(passkey, &PH) ;
        }
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_control_request_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
        p6_ack_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"EvwNuzapksrqhb";
            p7_key_SET_(key, &PH) ;
        }
        c_LoopBackDemoChannel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_MODE_11(), &PH);
        p11_custom_mode_SET((uint32_t)2864409613L, PH.base.pack) ;
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_TEST_DISARMED, PH.base.pack) ;
        p11_target_system_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_READ_20(), &PH);
        p20_param_index_SET((int16_t)(int16_t) -17605, PH.base.pack) ;
        p20_target_system_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
        p20_target_component_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        {
            char16_t* param_id = u"bxoecsjIuqfdug";
            p20_param_id_SET_(param_id, &PH) ;
        }
        c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_component_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        p21_target_system_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_VALUE_22(), &PH);
        p22_param_count_SET((uint16_t)(uint16_t)63383, PH.base.pack) ;
        p22_param_index_SET((uint16_t)(uint16_t)1499, PH.base.pack) ;
        {
            char16_t* param_id = u"geieifjvdpkhaw";
            p22_param_id_SET_(param_id, &PH) ;
        }
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64, PH.base.pack) ;
        p22_param_value_SET((float)2.1476496E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_SET_23(), &PH);
        p23_target_system_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64, PH.base.pack) ;
        {
            char16_t* param_id = u"oorsn";
            p23_param_id_SET_(param_id, &PH) ;
        }
        p23_param_value_SET((float)2.917324E38F, PH.base.pack) ;
        p23_target_component_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RAW_INT_24(), &PH);
        p24_h_acc_SET((uint32_t)2473481522L, &PH) ;
        p24_alt_ellipsoid_SET((int32_t)1295129396, &PH) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        p24_vel_SET((uint16_t)(uint16_t)50420, PH.base.pack) ;
        p24_lat_SET((int32_t)188124256, PH.base.pack) ;
        p24_epv_SET((uint16_t)(uint16_t)48465, PH.base.pack) ;
        p24_v_acc_SET((uint32_t)1414022041L, &PH) ;
        p24_eph_SET((uint16_t)(uint16_t)18580, PH.base.pack) ;
        p24_cog_SET((uint16_t)(uint16_t)19086, PH.base.pack) ;
        p24_hdg_acc_SET((uint32_t)2764756066L, &PH) ;
        p24_time_usec_SET((uint64_t)2223950332722576602L, PH.base.pack) ;
        p24_lon_SET((int32_t) -1619661240, PH.base.pack) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_2D_FIX, PH.base.pack) ;
        p24_alt_SET((int32_t)88586637, PH.base.pack) ;
        p24_vel_acc_SET((uint32_t)4062890655L, &PH) ;
        c_LoopBackDemoChannel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_used[] =  {(uint8_t)14, (uint8_t)114, (uint8_t)17, (uint8_t)191, (uint8_t)101, (uint8_t)197, (uint8_t)206, (uint8_t)232, (uint8_t)4, (uint8_t)117, (uint8_t)235, (uint8_t)78, (uint8_t)132, (uint8_t)61, (uint8_t)155, (uint8_t)70, (uint8_t)70, (uint8_t)174, (uint8_t)31, (uint8_t)49};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)4, (uint8_t)225, (uint8_t)175, (uint8_t)133, (uint8_t)211, (uint8_t)213, (uint8_t)178, (uint8_t)140, (uint8_t)0, (uint8_t)8, (uint8_t)9, (uint8_t)51, (uint8_t)162, (uint8_t)208, (uint8_t)219, (uint8_t)99, (uint8_t)210, (uint8_t)218, (uint8_t)177, (uint8_t)50};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_snr[] =  {(uint8_t)47, (uint8_t)177, (uint8_t)146, (uint8_t)69, (uint8_t)33, (uint8_t)166, (uint8_t)72, (uint8_t)17, (uint8_t)156, (uint8_t)20, (uint8_t)125, (uint8_t)40, (uint8_t)17, (uint8_t)65, (uint8_t)1, (uint8_t)58, (uint8_t)117, (uint8_t)147, (uint8_t)15, (uint8_t)96};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)107, (uint8_t)64, (uint8_t)41, (uint8_t)13, (uint8_t)102, (uint8_t)131, (uint8_t)185, (uint8_t)225, (uint8_t)79, (uint8_t)131, (uint8_t)155, (uint8_t)60, (uint8_t)165, (uint8_t)60, (uint8_t)157, (uint8_t)166, (uint8_t)44, (uint8_t)179, (uint8_t)214, (uint8_t)226};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        {
            uint8_t satellite_prn[] =  {(uint8_t)231, (uint8_t)76, (uint8_t)245, (uint8_t)121, (uint8_t)79, (uint8_t)242, (uint8_t)125, (uint8_t)65, (uint8_t)17, (uint8_t)187, (uint8_t)231, (uint8_t)160, (uint8_t)133, (uint8_t)62, (uint8_t)135, (uint8_t)188, (uint8_t)128, (uint8_t)197, (uint8_t)207, (uint8_t)80};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU_26(), &PH);
        p26_yacc_SET((int16_t)(int16_t) -27648, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t) -516, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t) -3020, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t) -6728, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t)5009, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t)4263, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t)15907, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t)22726, PH.base.pack) ;
        p26_time_boot_ms_SET((uint32_t)3775875017L, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t) -1956, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_IMU_27(), &PH);
        p27_ymag_SET((int16_t)(int16_t) -9462, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t) -9683, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)1880333694408336652L, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t)22253, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t)9415, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t) -3678, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t) -21084, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t)30790, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t)19236, PH.base.pack) ;
        p27_zacc_SET((int16_t)(int16_t) -27748, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_PRESSURE_28(), &PH);
        p28_press_abs_SET((int16_t)(int16_t) -11047, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)5265318711483516121L, PH.base.pack) ;
        p28_press_diff1_SET((int16_t)(int16_t)28547, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t)17752, PH.base.pack) ;
        p28_press_diff2_SET((int16_t)(int16_t) -475, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_temperature_SET((int16_t)(int16_t)20771, PH.base.pack) ;
        p29_press_diff_SET((float) -1.6788992E38F, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)1231281807L, PH.base.pack) ;
        p29_press_abs_SET((float)2.751141E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_30(), &PH);
        p30_pitchspeed_SET((float)9.057269E37F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)3208008904L, PH.base.pack) ;
        p30_yawspeed_SET((float)2.832218E38F, PH.base.pack) ;
        p30_rollspeed_SET((float)3.1747585E38F, PH.base.pack) ;
        p30_yaw_SET((float)5.7502487E37F, PH.base.pack) ;
        p30_pitch_SET((float) -2.351709E38F, PH.base.pack) ;
        p30_roll_SET((float) -1.1826288E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_yawspeed_SET((float) -1.097817E38F, PH.base.pack) ;
        p31_q1_SET((float) -1.728796E37F, PH.base.pack) ;
        p31_q2_SET((float) -9.304058E37F, PH.base.pack) ;
        p31_pitchspeed_SET((float) -5.5469267E37F, PH.base.pack) ;
        p31_rollspeed_SET((float) -1.1518879E38F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)2078463354L, PH.base.pack) ;
        p31_q3_SET((float) -1.9085723E38F, PH.base.pack) ;
        p31_q4_SET((float)1.3455929E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_z_SET((float) -2.2783263E38F, PH.base.pack) ;
        p32_vx_SET((float) -1.5412577E38F, PH.base.pack) ;
        p32_y_SET((float)1.8874818E38F, PH.base.pack) ;
        p32_vy_SET((float)2.0742216E37F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)389510108L, PH.base.pack) ;
        p32_x_SET((float)4.563876E37F, PH.base.pack) ;
        p32_vz_SET((float)1.3089776E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_lat_SET((int32_t)1460670613, PH.base.pack) ;
        p33_lon_SET((int32_t) -1751877285, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t) -15935, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t) -12199, PH.base.pack) ;
        p33_alt_SET((int32_t)378103104, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t)18767, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)13548, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)1498510014L, PH.base.pack) ;
        p33_relative_alt_SET((int32_t)908964739, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan1_scaled_SET((int16_t)(int16_t) -14409, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t) -15233, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t) -28088, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t)31209, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t)12550, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)1761612416L, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t) -22562, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t)4235, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t)22507, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_rssi_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)22687, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)51874, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)4300, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)4086467756L, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)16542, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)53947, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)31109, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)54570, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)50879, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo12_raw_SET((uint16_t)(uint16_t)13241, &PH) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)14378, PH.base.pack) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)53778, PH.base.pack) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)27026, &PH) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)32752, &PH) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)23639, PH.base.pack) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)12755, &PH) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)30895, &PH) ;
        p36_port_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)44761, &PH) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)39725, PH.base.pack) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)30269, PH.base.pack) ;
        p36_time_usec_SET((uint32_t)4193802934L, PH.base.pack) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)11420, PH.base.pack) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)11025, PH.base.pack) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)56480, &PH) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)23067, PH.base.pack) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)60249, &PH) ;
        c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_target_system_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t)2845, PH.base.pack) ;
        p37_start_index_SET((int16_t)(int16_t) -1262, PH.base.pack) ;
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_end_index_SET((int16_t)(int16_t) -25399, PH.base.pack) ;
        p38_target_component_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t) -20222, PH.base.pack) ;
        p38_target_system_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_39(), &PH);
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
        p39_z_SET((float)1.4609133E38F, PH.base.pack) ;
        p39_param4_SET((float)1.2415398E38F, PH.base.pack) ;
        p39_param1_SET((float)3.1403156E37F, PH.base.pack) ;
        p39_param2_SET((float)7.164766E37F, PH.base.pack) ;
        p39_param3_SET((float) -1.466763E38F, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)16572, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p39_x_SET((float)1.4821053E38F, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT, PH.base.pack) ;
        p39_y_SET((float) -2.677653E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_40(), &PH);
        p40_target_component_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
        p40_seq_SET((uint16_t)(uint16_t)57812, PH.base.pack) ;
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_target_system_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        p41_seq_SET((uint16_t)(uint16_t)4118, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)37617, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
        p43_target_system_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_COUNT_44(), &PH);
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p44_count_SET((uint16_t)(uint16_t)22609, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p44_target_component_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_target_component_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        p45_target_system_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)4262, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ACK_47(), &PH);
        p47_target_component_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_SEQUENCE, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_latitude_SET((int32_t)246494998, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)1689186273279456461L, &PH) ;
        p48_longitude_SET((int32_t) -96419761, PH.base.pack) ;
        p48_altitude_SET((int32_t)1447101445, PH.base.pack) ;
        p48_target_system_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_longitude_SET((int32_t)1677571725, PH.base.pack) ;
        p49_altitude_SET((int32_t)1388924462, PH.base.pack) ;
        p49_latitude_SET((int32_t) -894338627, PH.base.pack) ;
        p49_time_usec_SET((uint64_t)7772308075405123271L, &PH) ;
        c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_param_value_min_SET((float) -3.1935595E38F, PH.base.pack) ;
        p50_param_value_max_SET((float)1.2889844E38F, PH.base.pack) ;
        p50_param_value0_SET((float)2.8810404E38F, PH.base.pack) ;
        {
            char16_t* param_id = u"x";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_param_index_SET((int16_t)(int16_t)2759, PH.base.pack) ;
        p50_target_component_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p50_scale_SET((float) -3.1657284E38F, PH.base.pack) ;
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p51_target_component_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)16437, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_target_component_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p54_p1y_SET((float) -6.8910587E37F, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
        p54_p2x_SET((float) -1.7703116E38F, PH.base.pack) ;
        p54_p2z_SET((float)1.592504E38F, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        p54_p1z_SET((float) -6.921723E37F, PH.base.pack) ;
        p54_p1x_SET((float) -1.476E38F, PH.base.pack) ;
        p54_p2y_SET((float)1.1362559E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p1z_SET((float)2.2613587E38F, PH.base.pack) ;
        p55_p1y_SET((float)1.02418816E37F, PH.base.pack) ;
        p55_p2x_SET((float) -2.2251803E38F, PH.base.pack) ;
        p55_p2z_SET((float)3.0054541E38F, PH.base.pack) ;
        p55_p1x_SET((float) -5.2829653E37F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
        p55_p2y_SET((float)2.7170334E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_yawspeed_SET((float) -2.3712817E37F, PH.base.pack) ;
        {
            float q[] =  {-1.8411104E38F, 1.2576557E38F, 7.358117E35F, 1.7797652E38F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        {
            float covariance[] =  {-2.1757858E38F, -2.2325838E38F, -2.0609202E38F, -1.3545536E38F, -2.6835516E38F, 1.8713276E37F, -3.275008E37F, -3.2909019E38F, 1.0279843E38F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p61_pitchspeed_SET((float)2.2948181E38F, PH.base.pack) ;
        p61_rollspeed_SET((float) -2.5533773E38F, PH.base.pack) ;
        p61_time_usec_SET((uint64_t)2027238448247776551L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_xtrack_error_SET((float) -5.558497E37F, PH.base.pack) ;
        p62_nav_pitch_SET((float) -2.5081376E38F, PH.base.pack) ;
        p62_alt_error_SET((float)1.8675964E38F, PH.base.pack) ;
        p62_nav_roll_SET((float) -5.1804996E37F, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)60841, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t)6327, PH.base.pack) ;
        p62_aspd_error_SET((float)2.975827E38F, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t) -15433, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_lat_SET((int32_t)507971145, PH.base.pack) ;
        {
            float covariance[] =  {-1.3137368E38F, -5.8552185E36F, 2.5839116E38F, -2.8346126E38F, -1.9321891E38F, -2.4013531E38F, -2.9720058E37F, 1.5450868E37F, -1.5599742E38F, 1.2197836E38F, 1.0432707E38F, -4.9605906E37F, -1.1265894E38F, 2.825135E38F, 1.910001E35F, 1.9217301E38F, 2.3110595E38F, 3.001742E38F, -6.157229E36F, 2.086521E38F, 1.3769842E38F, -1.0125496E38F, 5.1744336E37F, 2.196297E38F, 9.703999E37F, -1.0977377E38F, 2.256481E38F, 1.5228168E38F, -1.6072909E38F, 2.2262285E38F, -3.128197E38F, 1.0950184E38F, -2.9132974E38F, -1.5105492E37F, -3.6947274E37F, -3.1630765E38F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_relative_alt_SET((int32_t)1725956373, PH.base.pack) ;
        p63_vy_SET((float)3.2711872E38F, PH.base.pack) ;
        p63_vx_SET((float) -1.4760363E37F, PH.base.pack) ;
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
        p63_alt_SET((int32_t)520924949, PH.base.pack) ;
        p63_vz_SET((float) -2.3574127E38F, PH.base.pack) ;
        p63_lon_SET((int32_t)1121004261, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)439871212851646840L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_time_usec_SET((uint64_t)6616303826973665248L, PH.base.pack) ;
        p64_vz_SET((float)2.3255182E38F, PH.base.pack) ;
        p64_z_SET((float) -2.922676E38F, PH.base.pack) ;
        p64_y_SET((float) -7.343388E37F, PH.base.pack) ;
        {
            float covariance[] =  {1.4543427E38F, 9.513874E36F, -2.912348E38F, 1.7087697E38F, 3.0348417E37F, -1.4724606E38F, 1.3101637E38F, 1.2357915E37F, 1.2265813E38F, -2.8239177E38F, 2.252982E38F, -1.5035562E38F, 2.9943759E38F, 2.5908338E38F, 2.0578707E38F, -1.5149244E38F, -6.21007E37F, -3.2837536E38F, -1.1982878E38F, 1.5744983E38F, 1.196918E38F, 1.7094037E38F, -1.6328402E37F, -1.0454476E38F, 2.3091032E38F, 2.6607824E38F, 1.2197965E38F, -1.3083888E38F, 6.847789E37F, -2.927342E37F, 2.9716073E38F, 2.203803E38F, -3.1905731E38F, -5.8663503E37F, -3.664734E37F, -2.252716E38F, -5.255211E37F, 1.3913266E38F, -1.02506804E37F, -3.377416E38F, -5.371793E37F, 2.9412893E38F, -1.5260005E38F, 2.7233305E38F, -1.0831086E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, PH.base.pack) ;
        p64_vx_SET((float)1.4142584E38F, PH.base.pack) ;
        p64_ax_SET((float) -1.9848853E38F, PH.base.pack) ;
        p64_az_SET((float)2.1205643E38F, PH.base.pack) ;
        p64_x_SET((float)1.8095692E38F, PH.base.pack) ;
        p64_ay_SET((float) -8.029553E37F, PH.base.pack) ;
        p64_vy_SET((float)2.264946E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan4_raw_SET((uint16_t)(uint16_t)40438, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)9243, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)64989, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)15277, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)7403, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)1744, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)6151, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)56544, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)45453, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)13367, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)35221, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)42034, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)47515, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)2063716958L, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)39785, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)57169, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)7432, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)46422, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)54720, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_req_message_rate_SET((uint16_t)(uint16_t)43065, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_STREAM_67(), &PH);
        p67_on_off_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        p67_message_rate_SET((uint16_t)(uint16_t)21655, PH.base.pack) ;
        p67_stream_id_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_z_SET((int16_t)(int16_t) -4271, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t) -3917, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t) -23034, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t) -21986, PH.base.pack) ;
        p69_buttons_SET((uint16_t)(uint16_t)16078, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_target_system_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)45958, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)18703, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)1557, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)53449, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)2333, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)55675, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)60928, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)11627, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_x_SET((int32_t)647782352, PH.base.pack) ;
        p73_y_SET((int32_t) -934327576, PH.base.pack) ;
        p73_z_SET((float) -2.5425002E38F, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        p73_param3_SET((float) -1.9477867E38F, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_DO_FOLLOW_REPOSITION, PH.base.pack) ;
        p73_param2_SET((float)3.2115687E36F, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p73_param4_SET((float)1.2432373E38F, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)11383, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
        p73_param1_SET((float) -1.4558292E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VFR_HUD_74(), &PH);
        p74_groundspeed_SET((float)3.171498E38F, PH.base.pack) ;
        p74_climb_SET((float) -2.53799E38F, PH.base.pack) ;
        p74_alt_SET((float) -6.79031E37F, PH.base.pack) ;
        p74_airspeed_SET((float)1.7795034E38F, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)57359, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t)18252, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_INT_75(), &PH);
        p75_param2_SET((float) -2.3643594E38F, PH.base.pack) ;
        p75_param4_SET((float)2.7672206E38F, PH.base.pack) ;
        p75_y_SET((int32_t) -1735071120, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_NAV_LAND, PH.base.pack) ;
        p75_x_SET((int32_t)1263879986, PH.base.pack) ;
        p75_param1_SET((float)1.5619148E38F, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        p75_param3_SET((float) -2.2364502E38F, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p75_z_SET((float) -3.1066983E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_LONG_76(), &PH);
        p76_param6_SET((float) -1.6709151E38F, PH.base.pack) ;
        p76_param2_SET((float) -2.598825E38F, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_DO_MOTOR_TEST, PH.base.pack) ;
        p76_param7_SET((float)9.2381704E36F, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        p76_param5_SET((float) -5.443922E37F, PH.base.pack) ;
        p76_param4_SET((float)8.0648626E37F, PH.base.pack) ;
        p76_param1_SET((float) -1.4408041E38F, PH.base.pack) ;
        p76_param3_SET((float) -8.127245E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_ACK_77(), &PH);
        p77_progress_SET((uint8_t)(uint8_t)211, &PH) ;
        p77_result_param2_SET((int32_t) -1413645411, &PH) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_DO_GUIDED_MASTER, PH.base.pack) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_TEMPORARILY_REJECTED, PH.base.pack) ;
        p77_target_system_SET((uint8_t)(uint8_t)63, &PH) ;
        p77_target_component_SET((uint8_t)(uint8_t)119, &PH) ;
        c_LoopBackDemoChannel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_mode_switch_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
        p81_manual_override_switch_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p81_thrust_SET((float)1.5985351E35F, PH.base.pack) ;
        p81_roll_SET((float)1.3492964E37F, PH.base.pack) ;
        p81_pitch_SET((float)1.4344153E38F, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)3077001124L, PH.base.pack) ;
        p81_yaw_SET((float) -1.8082857E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_type_mask_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p82_thrust_SET((float)1.3915958E38F, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        p82_body_yaw_rate_SET((float) -1.1748586E38F, PH.base.pack) ;
        p82_time_boot_ms_SET((uint32_t)3362812495L, PH.base.pack) ;
        p82_body_roll_rate_SET((float)1.60722E38F, PH.base.pack) ;
        {
            float q[] =  {9.827737E36F, 7.6253793E37F, 3.3648002E38F, -2.3640467E38F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_target_component_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p82_body_pitch_rate_SET((float) -1.22335E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_time_boot_ms_SET((uint32_t)4212279201L, PH.base.pack) ;
        p83_type_mask_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        p83_body_yaw_rate_SET((float) -1.6572855E38F, PH.base.pack) ;
        {
            float q[] =  {3.2115115E38F, -1.6909373E38F, -4.17908E37F, 1.8488136E38F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_body_roll_rate_SET((float)6.7055365E37F, PH.base.pack) ;
        p83_thrust_SET((float)1.4852369E37F, PH.base.pack) ;
        p83_body_pitch_rate_SET((float) -2.3422727E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_afz_SET((float) -2.0773946E38F, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)19167, PH.base.pack) ;
        p84_vy_SET((float) -5.589193E37F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)1372683262L, PH.base.pack) ;
        p84_vz_SET((float)1.3215223E38F, PH.base.pack) ;
        p84_z_SET((float) -8.728754E37F, PH.base.pack) ;
        p84_afy_SET((float)3.0531197E37F, PH.base.pack) ;
        p84_afx_SET((float)5.8983965E37F, PH.base.pack) ;
        p84_vx_SET((float)3.944657E37F, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p84_yaw_rate_SET((float)1.6212486E37F, PH.base.pack) ;
        p84_x_SET((float) -3.6156181E37F, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
        p84_y_SET((float)1.788249E38F, PH.base.pack) ;
        p84_yaw_SET((float)1.2643319E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_time_boot_ms_SET((uint32_t)1485767989L, PH.base.pack) ;
        p86_afy_SET((float)2.1982662E38F, PH.base.pack) ;
        p86_lon_int_SET((int32_t)217882610, PH.base.pack) ;
        p86_alt_SET((float)8.654877E37F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)39473, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p86_vx_SET((float) -4.3946313E37F, PH.base.pack) ;
        p86_vy_SET((float) -3.31067E38F, PH.base.pack) ;
        p86_yaw_rate_SET((float)2.2039E37F, PH.base.pack) ;
        p86_lat_int_SET((int32_t) -594609588, PH.base.pack) ;
        p86_afx_SET((float)1.5389717E38F, PH.base.pack) ;
        p86_yaw_SET((float)2.2641116E38F, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
        p86_vz_SET((float) -2.4826807E38F, PH.base.pack) ;
        p86_afz_SET((float) -5.698596E36F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_time_boot_ms_SET((uint32_t)2696483531L, PH.base.pack) ;
        p87_afy_SET((float) -2.4690698E38F, PH.base.pack) ;
        p87_vx_SET((float) -2.4330117E38F, PH.base.pack) ;
        p87_vy_SET((float)9.675336E37F, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p87_afz_SET((float)1.7977113E38F, PH.base.pack) ;
        p87_vz_SET((float)2.0316486E38F, PH.base.pack) ;
        p87_yaw_rate_SET((float) -1.2319185E38F, PH.base.pack) ;
        p87_alt_SET((float) -1.1865505E38F, PH.base.pack) ;
        p87_lon_int_SET((int32_t)151048346, PH.base.pack) ;
        p87_yaw_SET((float)1.4593399E38F, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)62854, PH.base.pack) ;
        p87_afx_SET((float)6.702347E37F, PH.base.pack) ;
        p87_lat_int_SET((int32_t) -1329558456, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_x_SET((float) -1.9832343E38F, PH.base.pack) ;
        p89_z_SET((float) -4.360321E37F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)2033540922L, PH.base.pack) ;
        p89_yaw_SET((float) -2.649655E37F, PH.base.pack) ;
        p89_roll_SET((float) -2.6716357E38F, PH.base.pack) ;
        p89_y_SET((float)2.6991956E38F, PH.base.pack) ;
        p89_pitch_SET((float)7.46976E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_90(), &PH);
        p90_yacc_SET((int16_t)(int16_t)31120, PH.base.pack) ;
        p90_rollspeed_SET((float) -3.2884453E38F, PH.base.pack) ;
        p90_yawspeed_SET((float)1.8385872E38F, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t)8507, PH.base.pack) ;
        p90_pitch_SET((float)2.2606328E38F, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)6787992741562173485L, PH.base.pack) ;
        p90_lon_SET((int32_t) -1267138609, PH.base.pack) ;
        p90_roll_SET((float) -1.9059512E38F, PH.base.pack) ;
        p90_yaw_SET((float) -9.842137E37F, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t) -24621, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t)17812, PH.base.pack) ;
        p90_alt_SET((int32_t)1918810612, PH.base.pack) ;
        p90_lat_SET((int32_t)980739012, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t)30412, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t) -18676, PH.base.pack) ;
        p90_pitchspeed_SET((float)1.2938217E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_CONTROLS_91(), &PH);
        p91_throttle_SET((float)1.6939847E38F, PH.base.pack) ;
        p91_yaw_rudder_SET((float)1.7071875E38F, PH.base.pack) ;
        p91_aux1_SET((float) -3.0324807E38F, PH.base.pack) ;
        p91_aux3_SET((float)1.7066859E38F, PH.base.pack) ;
        p91_aux4_SET((float)5.8914964E37F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        p91_roll_ailerons_SET((float) -6.7911776E36F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_DISARMED, PH.base.pack) ;
        p91_pitch_elevator_SET((float) -2.9617127E38F, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)4675934756276081356L, PH.base.pack) ;
        p91_aux2_SET((float) -1.5771295E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_rssi_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)2135519452123108868L, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)55363, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)37528, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)18538, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)41793, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)48115, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)53720, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)55021, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)57026, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)43475, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)14927, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)14260, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)38237, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        {
            float controls[] =  {-9.195961E37F, 2.186847E38F, 5.5575065E37F, 1.2605075E38F, 1.6354382E38F, -9.536934E36F, 2.5829497E38F, 1.3364095E38F, -1.2035559E38F, 2.5763771E38F, 3.495093E36F, -2.536353E38F, -9.197629E37F, -1.1461317E36F, -2.678362E38F, -3.2190433E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p93_mode_SET(e_MAV_MODE_MAV_MODE_GUIDED_DISARMED, PH.base.pack) ;
        p93_flags_SET((uint64_t)7810884624022963362L, PH.base.pack) ;
        p93_time_usec_SET((uint64_t)7659780335194894607L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_flow_rate_y_SET((float)3.9594623E37F, &PH) ;
        p100_ground_distance_SET((float) -2.2753813E38F, PH.base.pack) ;
        p100_flow_rate_x_SET((float)2.6984604E38F, &PH) ;
        p100_time_usec_SET((uint64_t)3745428389356944857L, PH.base.pack) ;
        p100_quality_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        p100_flow_x_SET((int16_t)(int16_t) -792, PH.base.pack) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        p100_flow_y_SET((int16_t)(int16_t) -11275, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float) -1.7370926E38F, PH.base.pack) ;
        p100_flow_comp_m_y_SET((float) -2.2816147E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_pitch_SET((float)9.0129825E36F, PH.base.pack) ;
        p101_y_SET((float) -3.05946E38F, PH.base.pack) ;
        p101_x_SET((float)2.7741357E37F, PH.base.pack) ;
        p101_usec_SET((uint64_t)4538980757761818582L, PH.base.pack) ;
        p101_yaw_SET((float)3.3048965E38F, PH.base.pack) ;
        p101_roll_SET((float)3.9770106E37F, PH.base.pack) ;
        p101_z_SET((float)3.2837953E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_z_SET((float) -1.7959903E38F, PH.base.pack) ;
        p102_x_SET((float) -2.159477E38F, PH.base.pack) ;
        p102_roll_SET((float) -1.5199651E38F, PH.base.pack) ;
        p102_usec_SET((uint64_t)4224273548866285134L, PH.base.pack) ;
        p102_y_SET((float) -2.8158469E38F, PH.base.pack) ;
        p102_pitch_SET((float) -1.2080066E38F, PH.base.pack) ;
        p102_yaw_SET((float) -2.274152E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_x_SET((float)1.714876E38F, PH.base.pack) ;
        p103_z_SET((float)6.2916324E37F, PH.base.pack) ;
        p103_usec_SET((uint64_t)6028443427704655638L, PH.base.pack) ;
        p103_y_SET((float) -3.386113E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_y_SET((float) -1.0063673E38F, PH.base.pack) ;
        p104_usec_SET((uint64_t)2172812222773511369L, PH.base.pack) ;
        p104_x_SET((float)1.6925496E37F, PH.base.pack) ;
        p104_roll_SET((float) -3.0593208E38F, PH.base.pack) ;
        p104_yaw_SET((float) -1.309778E37F, PH.base.pack) ;
        p104_z_SET((float)2.55075E38F, PH.base.pack) ;
        p104_pitch_SET((float)3.3037514E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGHRES_IMU_105(), &PH);
        p105_ygyro_SET((float)2.1649223E37F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)33507, PH.base.pack) ;
        p105_xacc_SET((float)2.6947958E38F, PH.base.pack) ;
        p105_xmag_SET((float) -1.1435678E38F, PH.base.pack) ;
        p105_pressure_alt_SET((float) -1.4378184E38F, PH.base.pack) ;
        p105_diff_pressure_SET((float) -3.0090083E36F, PH.base.pack) ;
        p105_abs_pressure_SET((float) -8.679207E36F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)4367100325929275467L, PH.base.pack) ;
        p105_xgyro_SET((float)3.0065125E38F, PH.base.pack) ;
        p105_ymag_SET((float)2.8535243E36F, PH.base.pack) ;
        p105_zgyro_SET((float)2.4886486E37F, PH.base.pack) ;
        p105_yacc_SET((float) -1.2796279E38F, PH.base.pack) ;
        p105_zmag_SET((float) -1.4859829E38F, PH.base.pack) ;
        p105_temperature_SET((float) -8.2285726E37F, PH.base.pack) ;
        p105_zacc_SET((float)1.0971283E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_time_delta_distance_us_SET((uint32_t)1472098126L, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)2857941788L, PH.base.pack) ;
        p106_distance_SET((float)1.3560463E38F, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)6641210133720438358L, PH.base.pack) ;
        p106_integrated_xgyro_SET((float)1.0353261E38F, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t) -26463, PH.base.pack) ;
        p106_integrated_y_SET((float) -7.349907E37F, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p106_integrated_x_SET((float)1.535877E38F, PH.base.pack) ;
        p106_integrated_ygyro_SET((float)1.0070465E38F, PH.base.pack) ;
        p106_integrated_zgyro_SET((float)2.7413068E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_SENSOR_107(), &PH);
        p107_xgyro_SET((float) -1.7534839E38F, PH.base.pack) ;
        p107_ymag_SET((float) -2.974762E37F, PH.base.pack) ;
        p107_zgyro_SET((float)6.5607485E37F, PH.base.pack) ;
        p107_diff_pressure_SET((float) -1.3920149E38F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)761034797834480357L, PH.base.pack) ;
        p107_ygyro_SET((float) -2.2810939E38F, PH.base.pack) ;
        p107_abs_pressure_SET((float)2.950383E38F, PH.base.pack) ;
        p107_xmag_SET((float) -2.991842E38F, PH.base.pack) ;
        p107_zmag_SET((float)4.7743215E37F, PH.base.pack) ;
        p107_pressure_alt_SET((float)3.2916793E38F, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)2282386627L, PH.base.pack) ;
        p107_xacc_SET((float) -2.9589553E38F, PH.base.pack) ;
        p107_yacc_SET((float)2.3348007E38F, PH.base.pack) ;
        p107_temperature_SET((float) -1.4208809E38F, PH.base.pack) ;
        p107_zacc_SET((float)3.1340568E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SIM_STATE_108(), &PH);
        p108_ve_SET((float)9.930724E37F, PH.base.pack) ;
        p108_roll_SET((float) -1.9656557E38F, PH.base.pack) ;
        p108_q2_SET((float)6.094539E36F, PH.base.pack) ;
        p108_ygyro_SET((float) -1.6960331E38F, PH.base.pack) ;
        p108_zgyro_SET((float)1.318147E38F, PH.base.pack) ;
        p108_yacc_SET((float) -3.235714E37F, PH.base.pack) ;
        p108_q3_SET((float) -1.3268813E38F, PH.base.pack) ;
        p108_vn_SET((float) -6.7731393E37F, PH.base.pack) ;
        p108_std_dev_horz_SET((float) -1.1074171E38F, PH.base.pack) ;
        p108_lat_SET((float)9.753474E37F, PH.base.pack) ;
        p108_xgyro_SET((float)2.7502661E38F, PH.base.pack) ;
        p108_q4_SET((float) -7.410368E37F, PH.base.pack) ;
        p108_yaw_SET((float)1.6935886E38F, PH.base.pack) ;
        p108_pitch_SET((float)3.0300995E38F, PH.base.pack) ;
        p108_zacc_SET((float)3.2259203E37F, PH.base.pack) ;
        p108_alt_SET((float) -4.674864E37F, PH.base.pack) ;
        p108_xacc_SET((float)1.74182E38F, PH.base.pack) ;
        p108_q1_SET((float)1.5124588E38F, PH.base.pack) ;
        p108_lon_SET((float)1.4557926E38F, PH.base.pack) ;
        p108_vd_SET((float) -2.7822499E38F, PH.base.pack) ;
        p108_std_dev_vert_SET((float) -1.4212291E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RADIO_STATUS_109(), &PH);
        p109_rxerrors_SET((uint16_t)(uint16_t)61079, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)43349, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        p110_target_system_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p110_target_network_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)37, (uint8_t)79, (uint8_t)179, (uint8_t)146, (uint8_t)65, (uint8_t)40, (uint8_t)105, (uint8_t)241, (uint8_t)110, (uint8_t)52, (uint8_t)199, (uint8_t)196, (uint8_t)162, (uint8_t)248, (uint8_t)5, (uint8_t)105, (uint8_t)208, (uint8_t)7, (uint8_t)97, (uint8_t)215, (uint8_t)49, (uint8_t)175, (uint8_t)17, (uint8_t)92, (uint8_t)222, (uint8_t)51, (uint8_t)224, (uint8_t)8, (uint8_t)156, (uint8_t)239, (uint8_t)182, (uint8_t)4, (uint8_t)204, (uint8_t)148, (uint8_t)95, (uint8_t)24, (uint8_t)113, (uint8_t)148, (uint8_t)3, (uint8_t)22, (uint8_t)8, (uint8_t)205, (uint8_t)78, (uint8_t)84, (uint8_t)206, (uint8_t)50, (uint8_t)144, (uint8_t)94, (uint8_t)110, (uint8_t)124, (uint8_t)71, (uint8_t)105, (uint8_t)69, (uint8_t)218, (uint8_t)68, (uint8_t)118, (uint8_t)232, (uint8_t)230, (uint8_t)202, (uint8_t)85, (uint8_t)0, (uint8_t)63, (uint8_t)141, (uint8_t)117, (uint8_t)241, (uint8_t)129, (uint8_t)239, (uint8_t)200, (uint8_t)52, (uint8_t)224, (uint8_t)28, (uint8_t)16, (uint8_t)47, (uint8_t)78, (uint8_t)26, (uint8_t)72, (uint8_t)5, (uint8_t)181, (uint8_t)165, (uint8_t)15, (uint8_t)100, (uint8_t)46, (uint8_t)32, (uint8_t)120, (uint8_t)51, (uint8_t)222, (uint8_t)183, (uint8_t)188, (uint8_t)40, (uint8_t)74, (uint8_t)85, (uint8_t)132, (uint8_t)194, (uint8_t)209, (uint8_t)17, (uint8_t)124, (uint8_t)210, (uint8_t)232, (uint8_t)67, (uint8_t)220, (uint8_t)129, (uint8_t)116, (uint8_t)35, (uint8_t)81, (uint8_t)61, (uint8_t)38, (uint8_t)222, (uint8_t)53, (uint8_t)159, (uint8_t)223, (uint8_t)134, (uint8_t)94, (uint8_t)181, (uint8_t)138, (uint8_t)165, (uint8_t)230, (uint8_t)97, (uint8_t)181, (uint8_t)208, (uint8_t)24, (uint8_t)127, (uint8_t)170, (uint8_t)236, (uint8_t)59, (uint8_t)65, (uint8_t)181, (uint8_t)162, (uint8_t)6, (uint8_t)1, (uint8_t)167, (uint8_t)66, (uint8_t)190, (uint8_t)250, (uint8_t)4, (uint8_t)137, (uint8_t)209, (uint8_t)164, (uint8_t)127, (uint8_t)182, (uint8_t)197, (uint8_t)136, (uint8_t)211, (uint8_t)21, (uint8_t)186, (uint8_t)158, (uint8_t)241, (uint8_t)154, (uint8_t)251, (uint8_t)104, (uint8_t)1, (uint8_t)103, (uint8_t)87, (uint8_t)238, (uint8_t)187, (uint8_t)199, (uint8_t)69, (uint8_t)176, (uint8_t)45, (uint8_t)140, (uint8_t)186, (uint8_t)116, (uint8_t)162, (uint8_t)50, (uint8_t)236, (uint8_t)138, (uint8_t)24, (uint8_t)94, (uint8_t)4, (uint8_t)115, (uint8_t)134, (uint8_t)98, (uint8_t)204, (uint8_t)136, (uint8_t)114, (uint8_t)69, (uint8_t)128, (uint8_t)212, (uint8_t)130, (uint8_t)195, (uint8_t)111, (uint8_t)100, (uint8_t)1, (uint8_t)243, (uint8_t)26, (uint8_t)26, (uint8_t)85, (uint8_t)1, (uint8_t)83, (uint8_t)170, (uint8_t)47, (uint8_t)232, (uint8_t)111, (uint8_t)135, (uint8_t)102, (uint8_t)166, (uint8_t)242, (uint8_t)134, (uint8_t)80, (uint8_t)82, (uint8_t)145, (uint8_t)73, (uint8_t)85, (uint8_t)176, (uint8_t)8, (uint8_t)182, (uint8_t)176, (uint8_t)7, (uint8_t)44, (uint8_t)180, (uint8_t)247, (uint8_t)163, (uint8_t)210, (uint8_t)251, (uint8_t)238, (uint8_t)195, (uint8_t)86, (uint8_t)127, (uint8_t)60, (uint8_t)38, (uint8_t)44, (uint8_t)224, (uint8_t)77, (uint8_t)75, (uint8_t)41, (uint8_t)2, (uint8_t)168, (uint8_t)83, (uint8_t)189, (uint8_t)157, (uint8_t)11, (uint8_t)255, (uint8_t)158, (uint8_t)136, (uint8_t)250, (uint8_t)3, (uint8_t)242, (uint8_t)90, (uint8_t)108, (uint8_t)65, (uint8_t)159, (uint8_t)243, (uint8_t)169, (uint8_t)252, (uint8_t)173, (uint8_t)197, (uint8_t)20, (uint8_t)175, (uint8_t)30, (uint8_t)96, (uint8_t)113, (uint8_t)40};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_component_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TIMESYNC_111(), &PH);
        p111_ts1_SET((int64_t)2390687213564004357L, PH.base.pack) ;
        p111_tc1_SET((int64_t)6216357383372432676L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_time_usec_SET((uint64_t)4899409209772939272L, PH.base.pack) ;
        p112_seq_SET((uint32_t)3422818750L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_GPS_113(), &PH);
        p113_cog_SET((uint16_t)(uint16_t)44446, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)17442, PH.base.pack) ;
        p113_lon_SET((int32_t) -521968509, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t)26397, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t) -31939, PH.base.pack) ;
        p113_lat_SET((int32_t) -237324281, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)50228, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t) -3495, PH.base.pack) ;
        p113_alt_SET((int32_t)879407622, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)53230, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)5893784415727799151L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_integrated_ygyro_SET((float) -9.949381E37F, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t)1672, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p114_integrated_x_SET((float) -3.928495E37F, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)1409113352L, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)8520707118502435317L, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        p114_integrated_zgyro_SET((float)2.3900682E38F, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)1600145426L, PH.base.pack) ;
        p114_integrated_xgyro_SET((float) -2.8678171E38F, PH.base.pack) ;
        p114_distance_SET((float)2.095769E38F, PH.base.pack) ;
        p114_integrated_y_SET((float)1.2220862E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_time_usec_SET((uint64_t)1351689720868747456L, PH.base.pack) ;
        p115_alt_SET((int32_t) -1194922708, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t) -8304, PH.base.pack) ;
        p115_rollspeed_SET((float) -1.570657E38F, PH.base.pack) ;
        p115_lat_SET((int32_t)1212037323, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)27669, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)62367, PH.base.pack) ;
        p115_pitchspeed_SET((float)2.6214165E38F, PH.base.pack) ;
        p115_yawspeed_SET((float) -2.6819956E38F, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {2.518166E38F, 1.2960316E38F, -2.8939532E38F, 2.103113E38F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_yacc_SET((int16_t)(int16_t) -5613, PH.base.pack) ;
        p115_lon_SET((int32_t) -6974396, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t) -20664, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t)7930, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t) -12181, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t)27648, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU2_116(), &PH);
        p116_xmag_SET((int16_t)(int16_t) -15757, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t)9577, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t) -11694, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t)21198, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t)20034, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t)11437, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t) -31001, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t) -17202, PH.base.pack) ;
        p116_ymag_SET((int16_t)(int16_t) -26090, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)168968689L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_end_SET((uint16_t)(uint16_t)47881, PH.base.pack) ;
        p117_start_SET((uint16_t)(uint16_t)20974, PH.base.pack) ;
        p117_target_component_SET((uint8_t)(uint8_t)91, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ENTRY_118(), &PH);
        p118_size_SET((uint32_t)1272123671L, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)1913120668L, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)63504, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)30598, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)1529, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_ofs_SET((uint32_t)3962080764L, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p119_id_SET((uint16_t)(uint16_t)16351, PH.base.pack) ;
        p119_count_SET((uint32_t)1043121257L, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_DATA_120(), &PH);
        p120_count_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p120_id_SET((uint16_t)(uint16_t)17925, PH.base.pack) ;
        p120_ofs_SET((uint32_t)2099474651L, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)252, (uint8_t)89, (uint8_t)191, (uint8_t)253, (uint8_t)169, (uint8_t)242, (uint8_t)17, (uint8_t)243, (uint8_t)99, (uint8_t)40, (uint8_t)42, (uint8_t)102, (uint8_t)248, (uint8_t)14, (uint8_t)228, (uint8_t)18, (uint8_t)34, (uint8_t)253, (uint8_t)21, (uint8_t)93, (uint8_t)182, (uint8_t)125, (uint8_t)255, (uint8_t)28, (uint8_t)32, (uint8_t)126, (uint8_t)125, (uint8_t)117, (uint8_t)110, (uint8_t)81, (uint8_t)250, (uint8_t)63, (uint8_t)66, (uint8_t)77, (uint8_t)213, (uint8_t)186, (uint8_t)179, (uint8_t)11, (uint8_t)142, (uint8_t)185, (uint8_t)144, (uint8_t)145, (uint8_t)184, (uint8_t)92, (uint8_t)199, (uint8_t)221, (uint8_t)254, (uint8_t)117, (uint8_t)53, (uint8_t)148, (uint8_t)160, (uint8_t)213, (uint8_t)32, (uint8_t)224, (uint8_t)116, (uint8_t)77, (uint8_t)102, (uint8_t)114, (uint8_t)111, (uint8_t)201, (uint8_t)124, (uint8_t)205, (uint8_t)159, (uint8_t)102, (uint8_t)246, (uint8_t)34, (uint8_t)18, (uint8_t)40, (uint8_t)236, (uint8_t)241, (uint8_t)157, (uint8_t)109, (uint8_t)252, (uint8_t)134, (uint8_t)254, (uint8_t)98, (uint8_t)44, (uint8_t)38, (uint8_t)156, (uint8_t)169, (uint8_t)134, (uint8_t)242, (uint8_t)34, (uint8_t)62, (uint8_t)69, (uint8_t)76, (uint8_t)58, (uint8_t)236, (uint8_t)96, (uint8_t)62};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ERASE_121(), &PH);
        p121_target_system_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        p121_target_component_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_system_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p122_target_component_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_target_component_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p123_target_system_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p123_len_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)48, (uint8_t)71, (uint8_t)222, (uint8_t)245, (uint8_t)195, (uint8_t)28, (uint8_t)53, (uint8_t)124, (uint8_t)193, (uint8_t)126, (uint8_t)114, (uint8_t)174, (uint8_t)73, (uint8_t)123, (uint8_t)96, (uint8_t)188, (uint8_t)236, (uint8_t)104, (uint8_t)198, (uint8_t)30, (uint8_t)210, (uint8_t)68, (uint8_t)126, (uint8_t)48, (uint8_t)71, (uint8_t)33, (uint8_t)72, (uint8_t)20, (uint8_t)170, (uint8_t)199, (uint8_t)38, (uint8_t)200, (uint8_t)54, (uint8_t)95, (uint8_t)195, (uint8_t)18, (uint8_t)85, (uint8_t)60, (uint8_t)23, (uint8_t)166, (uint8_t)107, (uint8_t)66, (uint8_t)109, (uint8_t)67, (uint8_t)39, (uint8_t)76, (uint8_t)182, (uint8_t)4, (uint8_t)50, (uint8_t)48, (uint8_t)222, (uint8_t)104, (uint8_t)110, (uint8_t)84, (uint8_t)167, (uint8_t)230, (uint8_t)133, (uint8_t)207, (uint8_t)29, (uint8_t)215, (uint8_t)174, (uint8_t)250, (uint8_t)205, (uint8_t)116, (uint8_t)181, (uint8_t)25, (uint8_t)229, (uint8_t)15, (uint8_t)140, (uint8_t)155, (uint8_t)54, (uint8_t)25, (uint8_t)135, (uint8_t)8, (uint8_t)201, (uint8_t)32, (uint8_t)226, (uint8_t)55, (uint8_t)215, (uint8_t)112, (uint8_t)38, (uint8_t)173, (uint8_t)87, (uint8_t)81, (uint8_t)201, (uint8_t)13, (uint8_t)103, (uint8_t)99, (uint8_t)226, (uint8_t)36, (uint8_t)117, (uint8_t)162, (uint8_t)69, (uint8_t)163, (uint8_t)240, (uint8_t)90, (uint8_t)253, (uint8_t)196, (uint8_t)254, (uint8_t)250, (uint8_t)25, (uint8_t)46, (uint8_t)24, (uint8_t)10, (uint8_t)46, (uint8_t)28, (uint8_t)205, (uint8_t)246, (uint8_t)15, (uint8_t)222};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RAW_124(), &PH);
        p124_lat_SET((int32_t) -1546327807, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)59066, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)4008752176L, PH.base.pack) ;
        p124_alt_SET((int32_t) -1004414700, PH.base.pack) ;
        p124_lon_SET((int32_t) -722751253, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)17676, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)6403242311285254109L, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)121, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)28464, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POWER_STATUS_125(), &PH);
        p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_SERVO_VALID, PH.base.pack) ;
        p125_Vcc_SET((uint16_t)(uint16_t)44625, PH.base.pack) ;
        p125_Vservo_SET((uint16_t)(uint16_t)7007, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_timeout_SET((uint16_t)(uint16_t)24870, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)31, (uint8_t)219, (uint8_t)59, (uint8_t)11, (uint8_t)231, (uint8_t)129, (uint8_t)248, (uint8_t)210, (uint8_t)229, (uint8_t)214, (uint8_t)213, (uint8_t)179, (uint8_t)146, (uint8_t)210, (uint8_t)200, (uint8_t)156, (uint8_t)165, (uint8_t)50, (uint8_t)125, (uint8_t)88, (uint8_t)195, (uint8_t)14, (uint8_t)9, (uint8_t)69, (uint8_t)214, (uint8_t)56, (uint8_t)101, (uint8_t)64, (uint8_t)154, (uint8_t)43, (uint8_t)129, (uint8_t)23, (uint8_t)38, (uint8_t)229, (uint8_t)28, (uint8_t)58, (uint8_t)126, (uint8_t)46, (uint8_t)190, (uint8_t)59, (uint8_t)142, (uint8_t)72, (uint8_t)22, (uint8_t)81, (uint8_t)15, (uint8_t)115, (uint8_t)218, (uint8_t)14, (uint8_t)160, (uint8_t)1, (uint8_t)87, (uint8_t)129, (uint8_t)80, (uint8_t)9, (uint8_t)239, (uint8_t)86, (uint8_t)71, (uint8_t)151, (uint8_t)255, (uint8_t)190, (uint8_t)202, (uint8_t)245, (uint8_t)118, (uint8_t)68, (uint8_t)63, (uint8_t)69, (uint8_t)142, (uint8_t)150, (uint8_t)65, (uint8_t)171};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_count_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_RESPOND, PH.base.pack) ;
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2, PH.base.pack) ;
        p126_baudrate_SET((uint32_t)4108406790L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTK_127(), &PH);
        p127_baseline_c_mm_SET((int32_t)1401138708, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t) -211152695, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)92, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t)454422181, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t) -1996770874, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)2421812449L, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
        p127_tow_SET((uint32_t)1583842290L, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)301579157L, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)36124, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RTK_128(), &PH);
        p128_accuracy_SET((uint32_t)332185840L, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t) -1103639651, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t) -520773142, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t)2146873563, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)35125137L, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t) -2105526490, PH.base.pack) ;
        p128_tow_SET((uint32_t)2356240765L, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)13164, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU3_129(), &PH);
        p129_ymag_SET((int16_t)(int16_t)11561, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t)28381, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t)19408, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t) -8183, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t)21156, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t)14330, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)1163283638L, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t)25546, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t)1050, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t) -15534, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_payload_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)32390, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)64772, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)35095, PH.base.pack) ;
        p130_size_SET((uint32_t)2396788274L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ENCAPSULATED_DATA_131(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)34, (uint8_t)151, (uint8_t)143, (uint8_t)221, (uint8_t)148, (uint8_t)60, (uint8_t)172, (uint8_t)60, (uint8_t)76, (uint8_t)165, (uint8_t)130, (uint8_t)6, (uint8_t)41, (uint8_t)254, (uint8_t)5, (uint8_t)130, (uint8_t)233, (uint8_t)194, (uint8_t)126, (uint8_t)133, (uint8_t)44, (uint8_t)157, (uint8_t)185, (uint8_t)90, (uint8_t)45, (uint8_t)43, (uint8_t)49, (uint8_t)237, (uint8_t)139, (uint8_t)89, (uint8_t)146, (uint8_t)40, (uint8_t)48, (uint8_t)18, (uint8_t)252, (uint8_t)111, (uint8_t)144, (uint8_t)116, (uint8_t)194, (uint8_t)3, (uint8_t)206, (uint8_t)210, (uint8_t)37, (uint8_t)53, (uint8_t)8, (uint8_t)229, (uint8_t)247, (uint8_t)243, (uint8_t)134, (uint8_t)79, (uint8_t)25, (uint8_t)193, (uint8_t)53, (uint8_t)221, (uint8_t)190, (uint8_t)245, (uint8_t)122, (uint8_t)80, (uint8_t)190, (uint8_t)63, (uint8_t)62, (uint8_t)55, (uint8_t)177, (uint8_t)67, (uint8_t)72, (uint8_t)172, (uint8_t)5, (uint8_t)186, (uint8_t)87, (uint8_t)219, (uint8_t)190, (uint8_t)64, (uint8_t)136, (uint8_t)218, (uint8_t)182, (uint8_t)139, (uint8_t)229, (uint8_t)79, (uint8_t)130, (uint8_t)75, (uint8_t)135, (uint8_t)114, (uint8_t)158, (uint8_t)79, (uint8_t)23, (uint8_t)2, (uint8_t)89, (uint8_t)97, (uint8_t)34, (uint8_t)244, (uint8_t)226, (uint8_t)235, (uint8_t)35, (uint8_t)206, (uint8_t)83, (uint8_t)122, (uint8_t)248, (uint8_t)130, (uint8_t)251, (uint8_t)169, (uint8_t)113, (uint8_t)1, (uint8_t)212, (uint8_t)128, (uint8_t)245, (uint8_t)109, (uint8_t)223, (uint8_t)61, (uint8_t)123, (uint8_t)169, (uint8_t)1, (uint8_t)14, (uint8_t)52, (uint8_t)218, (uint8_t)148, (uint8_t)159, (uint8_t)186, (uint8_t)38, (uint8_t)12, (uint8_t)240, (uint8_t)193, (uint8_t)55, (uint8_t)161, (uint8_t)6, (uint8_t)84, (uint8_t)138, (uint8_t)226, (uint8_t)75, (uint8_t)230, (uint8_t)189, (uint8_t)159, (uint8_t)174, (uint8_t)184, (uint8_t)219, (uint8_t)182, (uint8_t)54, (uint8_t)252, (uint8_t)220, (uint8_t)232, (uint8_t)31, (uint8_t)148, (uint8_t)230, (uint8_t)210, (uint8_t)97, (uint8_t)10, (uint8_t)192, (uint8_t)65, (uint8_t)33, (uint8_t)27, (uint8_t)254, (uint8_t)3, (uint8_t)16, (uint8_t)33, (uint8_t)156, (uint8_t)244, (uint8_t)147, (uint8_t)247, (uint8_t)198, (uint8_t)42, (uint8_t)76, (uint8_t)17, (uint8_t)29, (uint8_t)223, (uint8_t)139, (uint8_t)102, (uint8_t)110, (uint8_t)125, (uint8_t)219, (uint8_t)217, (uint8_t)184, (uint8_t)119, (uint8_t)7, (uint8_t)177, (uint8_t)139, (uint8_t)138, (uint8_t)0, (uint8_t)83, (uint8_t)137, (uint8_t)133, (uint8_t)39, (uint8_t)197, (uint8_t)16, (uint8_t)105, (uint8_t)65, (uint8_t)114, (uint8_t)105, (uint8_t)223, (uint8_t)25, (uint8_t)163, (uint8_t)99, (uint8_t)91, (uint8_t)49, (uint8_t)70, (uint8_t)19, (uint8_t)148, (uint8_t)172, (uint8_t)128, (uint8_t)111, (uint8_t)111, (uint8_t)52, (uint8_t)200, (uint8_t)140, (uint8_t)121, (uint8_t)45, (uint8_t)149, (uint8_t)216, (uint8_t)138, (uint8_t)175, (uint8_t)223, (uint8_t)217, (uint8_t)110, (uint8_t)176, (uint8_t)165, (uint8_t)44, (uint8_t)235, (uint8_t)199, (uint8_t)97, (uint8_t)133, (uint8_t)244, (uint8_t)222, (uint8_t)52, (uint8_t)163, (uint8_t)152, (uint8_t)74, (uint8_t)58, (uint8_t)86, (uint8_t)15, (uint8_t)69, (uint8_t)23, (uint8_t)157, (uint8_t)7, (uint8_t)22, (uint8_t)119, (uint8_t)49, (uint8_t)65, (uint8_t)106, (uint8_t)69, (uint8_t)148, (uint8_t)31, (uint8_t)198, (uint8_t)1, (uint8_t)252, (uint8_t)103, (uint8_t)131, (uint8_t)43, (uint8_t)48, (uint8_t)109, (uint8_t)195, (uint8_t)199, (uint8_t)28, (uint8_t)28, (uint8_t)207, (uint8_t)56};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        p131_seqnr_SET((uint16_t)(uint16_t)31869, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_min_distance_SET((uint16_t)(uint16_t)16498, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_90, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)18072, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)3165828177L, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)22920, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_lat_SET((int32_t)872981282, PH.base.pack) ;
        p133_grid_spacing_SET((uint16_t)(uint16_t)52108, PH.base.pack) ;
        p133_mask_SET((uint64_t)5750839363117718524L, PH.base.pack) ;
        p133_lon_SET((int32_t)1332624919, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_DATA_134(), &PH);
        p134_grid_spacing_SET((uint16_t)(uint16_t)38626, PH.base.pack) ;
        p134_lat_SET((int32_t)727738984, PH.base.pack) ;
        p134_gridbit_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
        p134_lon_SET((int32_t) -1445773787, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t) -8965, (int16_t)23500, (int16_t) -10554, (int16_t)7371, (int16_t)21014, (int16_t) -2462, (int16_t)1745, (int16_t)12415, (int16_t)4500, (int16_t)5139, (int16_t) -834, (int16_t)13234, (int16_t) -20726, (int16_t) -12068, (int16_t)11417, (int16_t) -21289};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lat_SET((int32_t)973896620, PH.base.pack) ;
        p135_lon_SET((int32_t)687372593, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_current_height_SET((float) -3.0924985E38F, PH.base.pack) ;
        p136_terrain_height_SET((float)5.7114317E37F, PH.base.pack) ;
        p136_spacing_SET((uint16_t)(uint16_t)1367, PH.base.pack) ;
        p136_lon_SET((int32_t)977416983, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)6214, PH.base.pack) ;
        p136_lat_SET((int32_t) -835632668, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)54835, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_time_boot_ms_SET((uint32_t)147823374L, PH.base.pack) ;
        p137_press_diff_SET((float)8.890517E37F, PH.base.pack) ;
        p137_press_abs_SET((float) -2.1143743E38F, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t)18773, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATT_POS_MOCAP_138(), &PH);
        p138_time_usec_SET((uint64_t)2128277214926114194L, PH.base.pack) ;
        p138_x_SET((float)1.0035574E38F, PH.base.pack) ;
        p138_z_SET((float) -9.1791436E36F, PH.base.pack) ;
        {
            float q[] =  {-2.0509934E38F, 6.3353886E37F, 1.7112174E38F, -1.566212E38F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_y_SET((float)1.678517E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        p139_target_component_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        {
            float controls[] =  {-3.0423326E38F, 9.392168E37F, 3.4375003E37F, -9.828785E36F, 3.0990173E37F, -5.2925797E37F, -2.7914668E38F, 1.9264969E38F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_group_mlx_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
        p139_target_system_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p139_time_usec_SET((uint64_t)5031269004653802540L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        {
            float controls[] =  {-1.5950132E38F, 1.477658E38F, -3.1520013E38F, 1.0405636E38F, 1.6015853E37F, -2.6285352E38F, -4.018335E37F, 6.183711E37F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p140_group_mlx_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
        p140_time_usec_SET((uint64_t)942289404302727694L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ALTITUDE_141(), &PH);
        p141_altitude_relative_SET((float) -7.6352664E37F, PH.base.pack) ;
        p141_altitude_amsl_SET((float) -5.7240134E37F, PH.base.pack) ;
        p141_altitude_local_SET((float)3.2494465E38F, PH.base.pack) ;
        p141_altitude_terrain_SET((float) -1.0545204E38F, PH.base.pack) ;
        p141_bottom_clearance_SET((float) -1.2795308E38F, PH.base.pack) ;
        p141_altitude_monotonic_SET((float)2.4898207E38F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)6532285242415557817L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RESOURCE_REQUEST_142(), &PH);
        {
            uint8_t storage[] =  {(uint8_t)139, (uint8_t)44, (uint8_t)59, (uint8_t)189, (uint8_t)66, (uint8_t)196, (uint8_t)59, (uint8_t)53, (uint8_t)59, (uint8_t)158, (uint8_t)248, (uint8_t)58, (uint8_t)85, (uint8_t)156, (uint8_t)126, (uint8_t)94, (uint8_t)47, (uint8_t)177, (uint8_t)183, (uint8_t)52, (uint8_t)140, (uint8_t)122, (uint8_t)68, (uint8_t)87, (uint8_t)64, (uint8_t)129, (uint8_t)149, (uint8_t)67, (uint8_t)130, (uint8_t)35, (uint8_t)149, (uint8_t)189, (uint8_t)205, (uint8_t)156, (uint8_t)120, (uint8_t)7, (uint8_t)91, (uint8_t)114, (uint8_t)174, (uint8_t)62, (uint8_t)33, (uint8_t)0, (uint8_t)254, (uint8_t)57, (uint8_t)140, (uint8_t)254, (uint8_t)39, (uint8_t)143, (uint8_t)76, (uint8_t)142, (uint8_t)86, (uint8_t)192, (uint8_t)223, (uint8_t)197, (uint8_t)31, (uint8_t)240, (uint8_t)8, (uint8_t)85, (uint8_t)55, (uint8_t)126, (uint8_t)232, (uint8_t)224, (uint8_t)123, (uint8_t)156, (uint8_t)206, (uint8_t)129, (uint8_t)133, (uint8_t)50, (uint8_t)255, (uint8_t)53, (uint8_t)137, (uint8_t)173, (uint8_t)113, (uint8_t)183, (uint8_t)213, (uint8_t)46, (uint8_t)164, (uint8_t)113, (uint8_t)1, (uint8_t)112, (uint8_t)94, (uint8_t)69, (uint8_t)151, (uint8_t)185, (uint8_t)111, (uint8_t)16, (uint8_t)26, (uint8_t)243, (uint8_t)82, (uint8_t)117, (uint8_t)126, (uint8_t)224, (uint8_t)239, (uint8_t)188, (uint8_t)137, (uint8_t)74, (uint8_t)42, (uint8_t)213, (uint8_t)133, (uint8_t)91, (uint8_t)85, (uint8_t)6, (uint8_t)206, (uint8_t)91, (uint8_t)240, (uint8_t)68, (uint8_t)181, (uint8_t)61, (uint8_t)225, (uint8_t)104, (uint8_t)107, (uint8_t)146, (uint8_t)43, (uint8_t)163, (uint8_t)121, (uint8_t)137, (uint8_t)217, (uint8_t)184, (uint8_t)18, (uint8_t)22};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        p142_transfer_type_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        p142_uri_type_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
        p142_request_id_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        {
            uint8_t uri[] =  {(uint8_t)149, (uint8_t)50, (uint8_t)176, (uint8_t)63, (uint8_t)204, (uint8_t)132, (uint8_t)226, (uint8_t)128, (uint8_t)28, (uint8_t)188, (uint8_t)56, (uint8_t)195, (uint8_t)251, (uint8_t)126, (uint8_t)143, (uint8_t)144, (uint8_t)55, (uint8_t)28, (uint8_t)86, (uint8_t)152, (uint8_t)63, (uint8_t)195, (uint8_t)138, (uint8_t)231, (uint8_t)99, (uint8_t)20, (uint8_t)63, (uint8_t)255, (uint8_t)129, (uint8_t)234, (uint8_t)186, (uint8_t)175, (uint8_t)46, (uint8_t)96, (uint8_t)108, (uint8_t)94, (uint8_t)190, (uint8_t)214, (uint8_t)128, (uint8_t)243, (uint8_t)95, (uint8_t)239, (uint8_t)17, (uint8_t)101, (uint8_t)77, (uint8_t)59, (uint8_t)247, (uint8_t)170, (uint8_t)234, (uint8_t)141, (uint8_t)160, (uint8_t)80, (uint8_t)31, (uint8_t)148, (uint8_t)208, (uint8_t)228, (uint8_t)97, (uint8_t)91, (uint8_t)242, (uint8_t)118, (uint8_t)209, (uint8_t)92, (uint8_t)44, (uint8_t)251, (uint8_t)211, (uint8_t)207, (uint8_t)206, (uint8_t)47, (uint8_t)62, (uint8_t)63, (uint8_t)82, (uint8_t)81, (uint8_t)150, (uint8_t)195, (uint8_t)148, (uint8_t)73, (uint8_t)33, (uint8_t)228, (uint8_t)166, (uint8_t)114, (uint8_t)124, (uint8_t)4, (uint8_t)108, (uint8_t)137, (uint8_t)207, (uint8_t)204, (uint8_t)86, (uint8_t)188, (uint8_t)37, (uint8_t)68, (uint8_t)52, (uint8_t)135, (uint8_t)213, (uint8_t)16, (uint8_t)249, (uint8_t)165, (uint8_t)254, (uint8_t)243, (uint8_t)51, (uint8_t)59, (uint8_t)10, (uint8_t)113, (uint8_t)90, (uint8_t)127, (uint8_t)64, (uint8_t)2, (uint8_t)180, (uint8_t)173, (uint8_t)219, (uint8_t)248, (uint8_t)170, (uint8_t)186, (uint8_t)195, (uint8_t)7, (uint8_t)201, (uint8_t)156, (uint8_t)140, (uint8_t)51, (uint8_t)190, (uint8_t)204};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_press_diff_SET((float) -9.77934E37F, PH.base.pack) ;
        p143_temperature_SET((int16_t)(int16_t) -20481, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)734939791L, PH.base.pack) ;
        p143_press_abs_SET((float)1.665604E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FOLLOW_TARGET_144(), &PH);
        {
            float rates[] =  {-2.7329446E38F, -2.827166E38F, 4.0466333E37F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        {
            float position_cov[] =  {2.4964814E37F, 1.5506228E38F, -1.8536333E38F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        p144_alt_SET((float) -3.0320637E38F, PH.base.pack) ;
        p144_lat_SET((int32_t) -391197420, PH.base.pack) ;
        {
            float acc[] =  {-3.3584447E38F, -1.8081815E38F, 1.7826905E38F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        {
            float vel[] =  {2.163902E38F, 2.3993E38F, 3.0728895E38F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        {
            float attitude_q[] =  {5.6824005E37F, -1.8867868E38F, 6.502442E37F, 6.1573633E37F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        p144_lon_SET((int32_t)1642453623, PH.base.pack) ;
        p144_timestamp_SET((uint64_t)8269045999858669938L, PH.base.pack) ;
        p144_custom_state_SET((uint64_t)4993819247708968352L, PH.base.pack) ;
        p144_est_capabilities_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_z_vel_SET((float) -1.4210391E38F, PH.base.pack) ;
        p146_x_pos_SET((float) -1.1236507E37F, PH.base.pack) ;
        p146_pitch_rate_SET((float)2.9684976E38F, PH.base.pack) ;
        p146_y_vel_SET((float) -5.7973886E37F, PH.base.pack) ;
        p146_yaw_rate_SET((float)7.6844305E37F, PH.base.pack) ;
        p146_y_acc_SET((float) -3.3161135E38F, PH.base.pack) ;
        {
            float vel_variance[] =  {-2.4050161E38F, 3.2881427E38F, 2.6955282E38F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_roll_rate_SET((float)2.16629E38F, PH.base.pack) ;
        p146_z_acc_SET((float) -9.220448E37F, PH.base.pack) ;
        p146_z_pos_SET((float) -3.1056279E37F, PH.base.pack) ;
        {
            float pos_variance[] =  {2.557897E38F, 1.5348905E38F, 3.354148E38F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_x_vel_SET((float) -2.7937464E38F, PH.base.pack) ;
        p146_x_acc_SET((float)1.94784E38F, PH.base.pack) ;
        {
            float q[] =  {-1.8872664E38F, -3.174721E37F, -6.6659797E37F, 1.893513E38F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        p146_airspeed_SET((float) -9.363715E37F, PH.base.pack) ;
        p146_y_pos_SET((float)2.3718232E38F, PH.base.pack) ;
        p146_time_usec_SET((uint64_t)8973000993090494951L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BATTERY_STATUS_147(), &PH);
        p147_battery_remaining_SET((int8_t)(int8_t) -82, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_UNKNOWN, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t) -15423, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t)732109563, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)62298, (uint16_t)4960, (uint16_t)12902, (uint16_t)15033, (uint16_t)35394, (uint16_t)28127, (uint16_t)55679, (uint16_t)11449, (uint16_t)25408, (uint16_t)56559};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_current_battery_SET((int16_t)(int16_t) -17799, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        p147_current_consumed_SET((int32_t)61266026, PH.base.pack) ;
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIPO, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_148(), &PH);
        p148_board_version_SET((uint32_t)1434113677L, PH.base.pack) ;
        p148_product_id_SET((uint16_t)(uint16_t)65267, PH.base.pack) ;
        p148_uid_SET((uint64_t)1161299515349844093L, PH.base.pack) ;
        p148_flight_sw_version_SET((uint32_t)2803531321L, PH.base.pack) ;
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)228, (uint8_t)78, (uint8_t)254, (uint8_t)90, (uint8_t)63, (uint8_t)174, (uint8_t)226, (uint8_t)79};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t uid2[] =  {(uint8_t)207, (uint8_t)138, (uint8_t)215, (uint8_t)170, (uint8_t)235, (uint8_t)59, (uint8_t)61, (uint8_t)140, (uint8_t)187, (uint8_t)45, (uint8_t)166, (uint8_t)56, (uint8_t)198, (uint8_t)56, (uint8_t)218, (uint8_t)36, (uint8_t)17, (uint8_t)18};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_os_sw_version_SET((uint32_t)3761179558L, PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)74, (uint8_t)250, (uint8_t)201, (uint8_t)149, (uint8_t)154, (uint8_t)46, (uint8_t)94, (uint8_t)210};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_TERRAIN, PH.base.pack) ;
        {
            uint8_t flight_custom_version[] =  {(uint8_t)40, (uint8_t)93, (uint8_t)43, (uint8_t)169, (uint8_t)49, (uint8_t)102, (uint8_t)162, (uint8_t)43};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        p148_middleware_sw_version_SET((uint32_t)3961482113L, PH.base.pack) ;
        p148_vendor_id_SET((uint16_t)(uint16_t)46012, PH.base.pack) ;
        c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LANDING_TARGET_149(), &PH);
        p149_angle_y_SET((float) -3.1792681E38F, PH.base.pack) ;
        p149_x_SET((float) -6.980733E37F, &PH) ;
        p149_angle_x_SET((float) -2.948762E38F, PH.base.pack) ;
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, PH.base.pack) ;
        p149_distance_SET((float)1.6958546E38F, PH.base.pack) ;
        p149_time_usec_SET((uint64_t)2138286559829983005L, PH.base.pack) ;
        p149_z_SET((float)2.6987243E38F, &PH) ;
        p149_position_valid_SET((uint8_t)(uint8_t)134, &PH) ;
        p149_size_x_SET((float) -1.8563865E38F, PH.base.pack) ;
        {
            float q[] =  {2.2045105E38F, -2.2482891E37F, 3.9971734E37F, -8.769988E37F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_target_num_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        p149_y_SET((float) -1.9031743E38F, &PH) ;
        p149_size_y_SET((float) -2.2719933E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FLEXIFUNCTION_SET_150(), &PH);
        p150_target_system_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p150_target_component_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FLEXIFUNCTION_SET_150(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FLEXIFUNCTION_READ_REQ_151(), &PH);
        p151_target_component_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        p151_data_index_SET((int16_t)(int16_t)26186, PH.base.pack) ;
        p151_read_req_type_SET((int16_t)(int16_t) -5975, PH.base.pack) ;
        p151_target_system_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FLEXIFUNCTION_READ_REQ_151(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FLEXIFUNCTION_BUFFER_FUNCTION_152(), &PH);
        {
            int8_t data_[] =  {(int8_t) -124, (int8_t)51, (int8_t) -53, (int8_t) -105, (int8_t) -103, (int8_t) -25, (int8_t) -31, (int8_t) -65, (int8_t) -121, (int8_t) -38, (int8_t)93, (int8_t) -50, (int8_t)109, (int8_t) -85, (int8_t)66, (int8_t) -50, (int8_t) -52, (int8_t) -48, (int8_t) -118, (int8_t)6, (int8_t) -27, (int8_t) -49, (int8_t)44, (int8_t) -80, (int8_t) -79, (int8_t) -90, (int8_t)66, (int8_t) -40, (int8_t) -78, (int8_t) -48, (int8_t) -85, (int8_t)73, (int8_t)118, (int8_t)56, (int8_t)15, (int8_t)2, (int8_t)25, (int8_t) -55, (int8_t)92, (int8_t)104, (int8_t)78, (int8_t)116, (int8_t) -67, (int8_t)110, (int8_t)86, (int8_t) -40, (int8_t) -31, (int8_t)103};
            p152_data__SET(&data_, 0, PH.base.pack) ;
        }
        p152_func_count_SET((uint16_t)(uint16_t)1681, PH.base.pack) ;
        p152_data_address_SET((uint16_t)(uint16_t)21369, PH.base.pack) ;
        p152_target_system_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        p152_func_index_SET((uint16_t)(uint16_t)38100, PH.base.pack) ;
        p152_data_size_SET((uint16_t)(uint16_t)53874, PH.base.pack) ;
        p152_target_component_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FLEXIFUNCTION_BUFFER_FUNCTION_152(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_153(), &PH);
        p153_result_SET((uint16_t)(uint16_t)47749, PH.base.pack) ;
        p153_target_system_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        p153_target_component_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        p153_func_index_SET((uint16_t)(uint16_t)966, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FLEXIFUNCTION_BUFFER_FUNCTION_ACK_153(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FLEXIFUNCTION_DIRECTORY_155(), &PH);
        p155_directory_type_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        p155_target_component_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        p155_start_index_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p155_count_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        {
            int8_t directory_data[] =  {(int8_t)125, (int8_t) -90, (int8_t)40, (int8_t) -2, (int8_t) -55, (int8_t)76, (int8_t)113, (int8_t) -93, (int8_t) -68, (int8_t) -32, (int8_t) -124, (int8_t) -4, (int8_t)111, (int8_t) -101, (int8_t)127, (int8_t) -78, (int8_t) -108, (int8_t) -61, (int8_t) -112, (int8_t) -2, (int8_t) -37, (int8_t)33, (int8_t) -76, (int8_t) -18, (int8_t) -6, (int8_t) -29, (int8_t)23, (int8_t)33, (int8_t) -7, (int8_t) -7, (int8_t)21, (int8_t)100, (int8_t) -36, (int8_t)79, (int8_t)47, (int8_t) -76, (int8_t) -110, (int8_t) -46, (int8_t)41, (int8_t) -117, (int8_t)73, (int8_t)72, (int8_t)27, (int8_t)37, (int8_t)23, (int8_t)5, (int8_t)109, (int8_t) -33};
            p155_directory_data_SET(&directory_data, 0, PH.base.pack) ;
        }
        p155_target_system_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FLEXIFUNCTION_DIRECTORY_155(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FLEXIFUNCTION_DIRECTORY_ACK_156(), &PH);
        p156_directory_type_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p156_start_index_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        p156_result_SET((uint16_t)(uint16_t)19194, PH.base.pack) ;
        p156_target_system_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        p156_count_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p156_target_component_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FLEXIFUNCTION_DIRECTORY_ACK_156(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FLEXIFUNCTION_COMMAND_157(), &PH);
        p157_target_system_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        p157_command_type_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        p157_target_component_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FLEXIFUNCTION_COMMAND_157(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FLEXIFUNCTION_COMMAND_ACK_158(), &PH);
        p158_result_SET((uint16_t)(uint16_t)63717, PH.base.pack) ;
        p158_command_type_SET((uint16_t)(uint16_t)42545, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FLEXIFUNCTION_COMMAND_ACK_158(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F2_A_170(), &PH);
        p170_sue_altitude_SET((int32_t) -1117530091, PH.base.pack) ;
        p170_sue_cog_SET((uint16_t)(uint16_t)48597, PH.base.pack) ;
        p170_sue_longitude_SET((int32_t) -1783621007, PH.base.pack) ;
        p170_sue_estimated_wind_2_SET((int16_t)(int16_t) -15002, PH.base.pack) ;
        p170_sue_svs_SET((int16_t)(int16_t)8882, PH.base.pack) ;
        p170_sue_rmat7_SET((int16_t)(int16_t)23119, PH.base.pack) ;
        p170_sue_sog_SET((int16_t)(int16_t)6789, PH.base.pack) ;
        p170_sue_magFieldEarth0_SET((int16_t)(int16_t) -16512, PH.base.pack) ;
        p170_sue_status_SET((uint8_t)(uint8_t)114, PH.base.pack) ;
        p170_sue_estimated_wind_1_SET((int16_t)(int16_t) -828, PH.base.pack) ;
        p170_sue_hdop_SET((int16_t)(int16_t)28146, PH.base.pack) ;
        p170_sue_rmat1_SET((int16_t)(int16_t) -26362, PH.base.pack) ;
        p170_sue_rmat3_SET((int16_t)(int16_t)28244, PH.base.pack) ;
        p170_sue_air_speed_3DIMU_SET((uint16_t)(uint16_t)11457, PH.base.pack) ;
        p170_sue_rmat4_SET((int16_t)(int16_t)24282, PH.base.pack) ;
        p170_sue_rmat6_SET((int16_t)(int16_t) -29186, PH.base.pack) ;
        p170_sue_estimated_wind_0_SET((int16_t)(int16_t)26897, PH.base.pack) ;
        p170_sue_cpu_load_SET((uint16_t)(uint16_t)9708, PH.base.pack) ;
        p170_sue_rmat8_SET((int16_t)(int16_t)10354, PH.base.pack) ;
        p170_sue_latitude_SET((int32_t) -1007516440, PH.base.pack) ;
        p170_sue_rmat2_SET((int16_t)(int16_t)13603, PH.base.pack) ;
        p170_sue_magFieldEarth2_SET((int16_t)(int16_t) -20330, PH.base.pack) ;
        p170_sue_time_SET((uint32_t)1728506598L, PH.base.pack) ;
        p170_sue_rmat0_SET((int16_t)(int16_t)29220, PH.base.pack) ;
        p170_sue_magFieldEarth1_SET((int16_t)(int16_t)9307, PH.base.pack) ;
        p170_sue_waypoint_index_SET((uint16_t)(uint16_t)6676, PH.base.pack) ;
        p170_sue_rmat5_SET((int16_t)(int16_t) -1379, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F2_A_170(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F2_B_171(), &PH);
        p171_sue_pwm_output_6_SET((int16_t)(int16_t) -2795, PH.base.pack) ;
        p171_sue_waypoint_goal_z_SET((int16_t)(int16_t) -22498, PH.base.pack) ;
        p171_sue_imu_velocity_y_SET((int16_t)(int16_t)11093, PH.base.pack) ;
        p171_sue_barom_press_SET((int32_t) -514352545, PH.base.pack) ;
        p171_sue_waypoint_goal_x_SET((int16_t)(int16_t) -5987, PH.base.pack) ;
        p171_sue_pwm_input_9_SET((int16_t)(int16_t)28547, PH.base.pack) ;
        p171_sue_imu_location_y_SET((int16_t)(int16_t)8176, PH.base.pack) ;
        p171_sue_pwm_output_7_SET((int16_t)(int16_t) -30336, PH.base.pack) ;
        p171_sue_pwm_input_1_SET((int16_t)(int16_t)24751, PH.base.pack) ;
        p171_sue_pwm_input_12_SET((int16_t)(int16_t)27942, PH.base.pack) ;
        p171_sue_pwm_output_9_SET((int16_t)(int16_t) -17909, PH.base.pack) ;
        p171_sue_pwm_output_12_SET((int16_t)(int16_t) -27486, PH.base.pack) ;
        p171_sue_pwm_output_2_SET((int16_t)(int16_t)29303, PH.base.pack) ;
        p171_sue_pwm_output_8_SET((int16_t)(int16_t)26375, PH.base.pack) ;
        p171_sue_time_SET((uint32_t)2855536104L, PH.base.pack) ;
        p171_sue_aero_z_SET((int16_t)(int16_t)382, PH.base.pack) ;
        p171_sue_desired_height_SET((int16_t)(int16_t) -13330, PH.base.pack) ;
        p171_sue_pwm_input_11_SET((int16_t)(int16_t)25752, PH.base.pack) ;
        p171_sue_bat_amp_SET((int16_t)(int16_t) -18786, PH.base.pack) ;
        p171_sue_pwm_input_7_SET((int16_t)(int16_t)19846, PH.base.pack) ;
        p171_sue_bat_volt_SET((int16_t)(int16_t) -7955, PH.base.pack) ;
        p171_sue_location_error_earth_x_SET((int16_t)(int16_t)28411, PH.base.pack) ;
        p171_sue_pwm_input_8_SET((int16_t)(int16_t) -30685, PH.base.pack) ;
        p171_sue_pwm_input_6_SET((int16_t)(int16_t)25241, PH.base.pack) ;
        p171_sue_pwm_output_11_SET((int16_t)(int16_t) -10020, PH.base.pack) ;
        p171_sue_pwm_output_4_SET((int16_t)(int16_t)15693, PH.base.pack) ;
        p171_sue_osc_fails_SET((int16_t)(int16_t) -31414, PH.base.pack) ;
        p171_sue_flags_SET((uint32_t)3214985689L, PH.base.pack) ;
        p171_sue_pwm_input_3_SET((int16_t)(int16_t) -13672, PH.base.pack) ;
        p171_sue_location_error_earth_y_SET((int16_t)(int16_t)21565, PH.base.pack) ;
        p171_sue_pwm_input_5_SET((int16_t)(int16_t)1568, PH.base.pack) ;
        p171_sue_aero_x_SET((int16_t)(int16_t) -6512, PH.base.pack) ;
        p171_sue_barom_alt_SET((int32_t) -1445104344, PH.base.pack) ;
        p171_sue_pwm_output_1_SET((int16_t)(int16_t) -13236, PH.base.pack) ;
        p171_sue_bat_amp_hours_SET((int16_t)(int16_t) -5323, PH.base.pack) ;
        p171_sue_pwm_output_10_SET((int16_t)(int16_t) -6341, PH.base.pack) ;
        p171_sue_imu_velocity_x_SET((int16_t)(int16_t) -13942, PH.base.pack) ;
        p171_sue_pwm_output_3_SET((int16_t)(int16_t) -23267, PH.base.pack) ;
        p171_sue_pwm_input_4_SET((int16_t)(int16_t)29425, PH.base.pack) ;
        p171_sue_imu_location_x_SET((int16_t)(int16_t)1112, PH.base.pack) ;
        p171_sue_pwm_input_2_SET((int16_t)(int16_t) -217, PH.base.pack) ;
        p171_sue_imu_location_z_SET((int16_t)(int16_t) -4780, PH.base.pack) ;
        p171_sue_pwm_output_5_SET((int16_t)(int16_t) -19925, PH.base.pack) ;
        p171_sue_location_error_earth_z_SET((int16_t)(int16_t)8882, PH.base.pack) ;
        p171_sue_pwm_input_10_SET((int16_t)(int16_t)3660, PH.base.pack) ;
        p171_sue_barom_temp_SET((int16_t)(int16_t)11530, PH.base.pack) ;
        p171_sue_waypoint_goal_y_SET((int16_t)(int16_t)7273, PH.base.pack) ;
        p171_sue_imu_velocity_z_SET((int16_t)(int16_t)17561, PH.base.pack) ;
        p171_sue_memory_stack_free_SET((int16_t)(int16_t) -29867, PH.base.pack) ;
        p171_sue_aero_y_SET((int16_t)(int16_t) -9088, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F2_B_171(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F4_172(), &PH);
        p172_sue_AILERON_NAVIGATION_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p172_sue_ROLL_STABILIZATION_AILERONS_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        p172_sue_YAW_STABILIZATION_RUDDER_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p172_sue_PITCH_STABILIZATION_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        p172_sue_YAW_STABILIZATION_AILERON_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p172_sue_ALTITUDEHOLD_WAYPOINT_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p172_sue_RUDDER_NAVIGATION_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
        p172_sue_ROLL_STABILIZATION_RUDDER_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p172_sue_RACING_MODE_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p172_sue_ALTITUDEHOLD_STABILIZED_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F4_172(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F5_173(), &PH);
        p173_sue_ROLLKP_SET((float) -1.2620685E38F, PH.base.pack) ;
        p173_sue_YAWKP_AILERON_SET((float)1.9366971E38F, PH.base.pack) ;
        p173_sue_YAWKD_AILERON_SET((float) -1.0503641E38F, PH.base.pack) ;
        p173_sue_ROLLKD_SET((float) -3.2196832E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F5_173(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F6_174(), &PH);
        p174_sue_ELEVATOR_BOOST_SET((float) -1.6657113E38F, PH.base.pack) ;
        p174_sue_RUDDER_ELEV_MIX_SET((float)3.1195027E38F, PH.base.pack) ;
        p174_sue_ROLL_ELEV_MIX_SET((float)2.0757967E38F, PH.base.pack) ;
        p174_sue_PITCHGAIN_SET((float)2.7023494E38F, PH.base.pack) ;
        p174_sue_PITCHKD_SET((float) -1.1216847E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F6_174(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F7_175(), &PH);
        p175_sue_ROLLKP_RUDDER_SET((float) -1.5813407E38F, PH.base.pack) ;
        p175_sue_ROLLKD_RUDDER_SET((float)3.323893E38F, PH.base.pack) ;
        p175_sue_YAWKD_RUDDER_SET((float) -3.1053255E38F, PH.base.pack) ;
        p175_sue_RTL_PITCH_DOWN_SET((float) -2.70709E38F, PH.base.pack) ;
        p175_sue_YAWKP_RUDDER_SET((float) -8.559254E37F, PH.base.pack) ;
        p175_sue_RUDDER_BOOST_SET((float) -1.1036311E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F7_175(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F8_176(), &PH);
        p176_sue_ALT_HOLD_PITCH_MIN_SET((float)3.2096414E38F, PH.base.pack) ;
        p176_sue_ALT_HOLD_THROTTLE_MAX_SET((float) -1.4351243E38F, PH.base.pack) ;
        p176_sue_HEIGHT_TARGET_MAX_SET((float)2.7034168E38F, PH.base.pack) ;
        p176_sue_ALT_HOLD_PITCH_HIGH_SET((float)5.4667163E37F, PH.base.pack) ;
        p176_sue_ALT_HOLD_THROTTLE_MIN_SET((float) -7.519257E35F, PH.base.pack) ;
        p176_sue_HEIGHT_TARGET_MIN_SET((float) -2.1224836E38F, PH.base.pack) ;
        p176_sue_ALT_HOLD_PITCH_MAX_SET((float)3.5566065E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F8_176(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F13_177(), &PH);
        p177_sue_week_no_SET((int16_t)(int16_t)31424, PH.base.pack) ;
        p177_sue_lat_origin_SET((int32_t)541527982, PH.base.pack) ;
        p177_sue_alt_origin_SET((int32_t) -1026972575, PH.base.pack) ;
        p177_sue_lon_origin_SET((int32_t)1452747412, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F13_177(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F14_178(), &PH);
        p178_sue_WIND_ESTIMATION_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
        p178_sue_AIRFRAME_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p178_sue_CLOCK_CONFIG_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        p178_sue_TRAP_FLAGS_SET((int16_t)(int16_t) -25821, PH.base.pack) ;
        p178_sue_BOARD_TYPE_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        p178_sue_osc_fail_count_SET((int16_t)(int16_t) -10414, PH.base.pack) ;
        p178_sue_GPS_TYPE_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p178_sue_DR_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p178_sue_FLIGHT_PLAN_TYPE_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p178_sue_RCON_SET((int16_t)(int16_t) -7174, PH.base.pack) ;
        p178_sue_TRAP_SOURCE_SET((uint32_t)3293308493L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F14_178(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F15_179(), &PH);
        {
            uint8_t sue_ID_VEHICLE_MODEL_NAME[] =  {(uint8_t)84, (uint8_t)158, (uint8_t)157, (uint8_t)230, (uint8_t)235, (uint8_t)89, (uint8_t)78, (uint8_t)138, (uint8_t)56, (uint8_t)238, (uint8_t)58, (uint8_t)197, (uint8_t)18, (uint8_t)134, (uint8_t)112, (uint8_t)88, (uint8_t)23, (uint8_t)16, (uint8_t)27, (uint8_t)149, (uint8_t)62, (uint8_t)113, (uint8_t)112, (uint8_t)246, (uint8_t)230, (uint8_t)198, (uint8_t)99, (uint8_t)130, (uint8_t)161, (uint8_t)8, (uint8_t)246, (uint8_t)148, (uint8_t)100, (uint8_t)141, (uint8_t)179, (uint8_t)29, (uint8_t)8, (uint8_t)185, (uint8_t)209, (uint8_t)82};
            p179_sue_ID_VEHICLE_MODEL_NAME_SET(&sue_ID_VEHICLE_MODEL_NAME, 0, PH.base.pack) ;
        }
        {
            uint8_t sue_ID_VEHICLE_REGISTRATION[] =  {(uint8_t)3, (uint8_t)28, (uint8_t)251, (uint8_t)11, (uint8_t)152, (uint8_t)152, (uint8_t)229, (uint8_t)114, (uint8_t)77, (uint8_t)227, (uint8_t)109, (uint8_t)81, (uint8_t)34, (uint8_t)93, (uint8_t)195, (uint8_t)98, (uint8_t)116, (uint8_t)92, (uint8_t)227, (uint8_t)58};
            p179_sue_ID_VEHICLE_REGISTRATION_SET(&sue_ID_VEHICLE_REGISTRATION, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F15_179(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F16_180(), &PH);
        {
            uint8_t sue_ID_DIY_DRONES_URL[] =  {(uint8_t)161, (uint8_t)109, (uint8_t)134, (uint8_t)57, (uint8_t)143, (uint8_t)152, (uint8_t)52, (uint8_t)128, (uint8_t)142, (uint8_t)201, (uint8_t)190, (uint8_t)26, (uint8_t)128, (uint8_t)64, (uint8_t)239, (uint8_t)171, (uint8_t)95, (uint8_t)26, (uint8_t)168, (uint8_t)59, (uint8_t)90, (uint8_t)111, (uint8_t)106, (uint8_t)234, (uint8_t)14, (uint8_t)97, (uint8_t)93, (uint8_t)133, (uint8_t)6, (uint8_t)8, (uint8_t)254, (uint8_t)201, (uint8_t)40, (uint8_t)211, (uint8_t)100, (uint8_t)31, (uint8_t)245, (uint8_t)109, (uint8_t)62, (uint8_t)92, (uint8_t)240, (uint8_t)140, (uint8_t)142, (uint8_t)164, (uint8_t)17, (uint8_t)130, (uint8_t)203, (uint8_t)26, (uint8_t)0, (uint8_t)194, (uint8_t)254, (uint8_t)49, (uint8_t)97, (uint8_t)154, (uint8_t)35, (uint8_t)107, (uint8_t)49, (uint8_t)97, (uint8_t)49, (uint8_t)224, (uint8_t)238, (uint8_t)98, (uint8_t)233, (uint8_t)225, (uint8_t)141, (uint8_t)250, (uint8_t)40, (uint8_t)111, (uint8_t)188, (uint8_t)61};
            p180_sue_ID_DIY_DRONES_URL_SET(&sue_ID_DIY_DRONES_URL, 0, PH.base.pack) ;
        }
        {
            uint8_t sue_ID_LEAD_PILOT[] =  {(uint8_t)120, (uint8_t)194, (uint8_t)197, (uint8_t)86, (uint8_t)25, (uint8_t)29, (uint8_t)148, (uint8_t)84, (uint8_t)38, (uint8_t)174, (uint8_t)162, (uint8_t)121, (uint8_t)0, (uint8_t)110, (uint8_t)150, (uint8_t)148, (uint8_t)106, (uint8_t)215, (uint8_t)50, (uint8_t)90, (uint8_t)17, (uint8_t)177, (uint8_t)157, (uint8_t)229, (uint8_t)108, (uint8_t)61, (uint8_t)109, (uint8_t)180, (uint8_t)129, (uint8_t)189, (uint8_t)16, (uint8_t)199, (uint8_t)159, (uint8_t)47, (uint8_t)34, (uint8_t)18, (uint8_t)99, (uint8_t)52, (uint8_t)73, (uint8_t)201};
            p180_sue_ID_LEAD_PILOT_SET(&sue_ID_LEAD_PILOT, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F16_180(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ALTITUDES_181(), &PH);
        p181_alt_imu_SET((int32_t) -1486294764, PH.base.pack) ;
        p181_alt_gps_SET((int32_t)2111149540, PH.base.pack) ;
        p181_time_boot_ms_SET((uint32_t)2502614283L, PH.base.pack) ;
        p181_alt_barometric_SET((int32_t) -753107528, PH.base.pack) ;
        p181_alt_optical_flow_SET((int32_t)1829594703, PH.base.pack) ;
        p181_alt_extra_SET((int32_t) -713274293, PH.base.pack) ;
        p181_alt_range_finder_SET((int32_t)1388909485, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ALTITUDES_181(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AIRSPEEDS_182(), &PH);
        p182_aoa_SET((int16_t)(int16_t)20121, PH.base.pack) ;
        p182_airspeed_pitot_SET((int16_t)(int16_t) -32452, PH.base.pack) ;
        p182_airspeed_imu_SET((int16_t)(int16_t)23835, PH.base.pack) ;
        p182_time_boot_ms_SET((uint32_t)340954237L, PH.base.pack) ;
        p182_airspeed_hot_wire_SET((int16_t)(int16_t)25501, PH.base.pack) ;
        p182_aoy_SET((int16_t)(int16_t)16380, PH.base.pack) ;
        p182_airspeed_ultrasonic_SET((int16_t)(int16_t)24035, PH.base.pack) ;
        c_LoopBackDemoChannel_on_AIRSPEEDS_182(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F17_183(), &PH);
        p183_sue_feed_forward_SET((float)2.8075637E38F, PH.base.pack) ;
        p183_sue_turn_rate_fbw_SET((float) -1.4698529E38F, PH.base.pack) ;
        p183_sue_turn_rate_nav_SET((float)2.1354444E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F17_183(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F18_184(), &PH);
        p184_reference_speed_SET((float)2.2881497E38F, PH.base.pack) ;
        p184_angle_of_attack_normal_SET((float)1.895346E38F, PH.base.pack) ;
        p184_angle_of_attack_inverted_SET((float) -9.633828E37F, PH.base.pack) ;
        p184_elevator_trim_normal_SET((float) -3.2713592E37F, PH.base.pack) ;
        p184_elevator_trim_inverted_SET((float) -4.2361884E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F18_184(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F19_185(), &PH);
        p185_sue_aileron_output_channel_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
        p185_sue_throttle_output_channel_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p185_sue_aileron_reversed_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p185_sue_rudder_output_channel_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        p185_sue_throttle_reversed_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
        p185_sue_elevator_reversed_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p185_sue_rudder_reversed_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p185_sue_elevator_output_channel_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F19_185(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F20_186(), &PH);
        p186_sue_number_of_inputs_SET((uint8_t)(uint8_t)238, PH.base.pack) ;
        p186_sue_trim_value_input_5_SET((int16_t)(int16_t)21356, PH.base.pack) ;
        p186_sue_trim_value_input_8_SET((int16_t)(int16_t) -10835, PH.base.pack) ;
        p186_sue_trim_value_input_11_SET((int16_t)(int16_t) -32438, PH.base.pack) ;
        p186_sue_trim_value_input_10_SET((int16_t)(int16_t)2635, PH.base.pack) ;
        p186_sue_trim_value_input_9_SET((int16_t)(int16_t)29377, PH.base.pack) ;
        p186_sue_trim_value_input_2_SET((int16_t)(int16_t)27719, PH.base.pack) ;
        p186_sue_trim_value_input_3_SET((int16_t)(int16_t) -16118, PH.base.pack) ;
        p186_sue_trim_value_input_1_SET((int16_t)(int16_t)14963, PH.base.pack) ;
        p186_sue_trim_value_input_12_SET((int16_t)(int16_t) -4155, PH.base.pack) ;
        p186_sue_trim_value_input_4_SET((int16_t)(int16_t)23495, PH.base.pack) ;
        p186_sue_trim_value_input_7_SET((int16_t)(int16_t) -29366, PH.base.pack) ;
        p186_sue_trim_value_input_6_SET((int16_t)(int16_t)6825, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F20_186(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F21_187(), &PH);
        p187_sue_gyro_z_offset_SET((int16_t)(int16_t)15653, PH.base.pack) ;
        p187_sue_gyro_y_offset_SET((int16_t)(int16_t)32089, PH.base.pack) ;
        p187_sue_accel_x_offset_SET((int16_t)(int16_t) -20152, PH.base.pack) ;
        p187_sue_accel_y_offset_SET((int16_t)(int16_t) -13079, PH.base.pack) ;
        p187_sue_gyro_x_offset_SET((int16_t)(int16_t) -21848, PH.base.pack) ;
        p187_sue_accel_z_offset_SET((int16_t)(int16_t) -23288, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F21_187(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_UDB_EXTRA_F22_188(), &PH);
        p188_sue_gyro_z_at_calibration_SET((int16_t)(int16_t)6097, PH.base.pack) ;
        p188_sue_gyro_y_at_calibration_SET((int16_t)(int16_t)28244, PH.base.pack) ;
        p188_sue_accel_x_at_calibration_SET((int16_t)(int16_t)21209, PH.base.pack) ;
        p188_sue_accel_z_at_calibration_SET((int16_t)(int16_t) -30732, PH.base.pack) ;
        p188_sue_accel_y_at_calibration_SET((int16_t)(int16_t) -7854, PH.base.pack) ;
        p188_sue_gyro_x_at_calibration_SET((int16_t)(int16_t)25302, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_UDB_EXTRA_F22_188(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_pos_horiz_ratio_SET((float) -2.0821147E38F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)4919896461901697827L, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float)2.540702E38F, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float) -1.4204905E38F, PH.base.pack) ;
        p230_hagl_ratio_SET((float)1.5586507E38F, PH.base.pack) ;
        p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_VERT_AGL, PH.base.pack) ;
        p230_tas_ratio_SET((float) -2.266486E38F, PH.base.pack) ;
        p230_mag_ratio_SET((float)5.140957E37F, PH.base.pack) ;
        p230_vel_ratio_SET((float) -2.7176372E38F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float) -1.1959103E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIND_COV_231(), &PH);
        p231_vert_accuracy_SET((float) -1.1415515E38F, PH.base.pack) ;
        p231_var_horiz_SET((float)4.759938E37F, PH.base.pack) ;
        p231_wind_alt_SET((float) -1.0932512E38F, PH.base.pack) ;
        p231_var_vert_SET((float)1.8776325E38F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float)1.9992158E38F, PH.base.pack) ;
        p231_wind_y_SET((float)2.3589286E38F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)3172658640750231149L, PH.base.pack) ;
        p231_wind_z_SET((float)6.3631745E37F, PH.base.pack) ;
        p231_wind_x_SET((float) -2.535672E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INPUT_232(), &PH);
        p232_satellites_visible_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        p232_lon_SET((int32_t) -1363430442, PH.base.pack) ;
        p232_vn_SET((float) -1.835027E38F, PH.base.pack) ;
        p232_hdop_SET((float) -4.1281863E37F, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)2315051899L, PH.base.pack) ;
        p232_speed_accuracy_SET((float) -2.6937752E38F, PH.base.pack) ;
        p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_ALT, PH.base.pack) ;
        p232_lat_SET((int32_t) -990445892, PH.base.pack) ;
        p232_vdop_SET((float)2.7464287E38F, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        p232_horiz_accuracy_SET((float) -1.477235E38F, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)6681706430048135615L, PH.base.pack) ;
        p232_ve_SET((float) -2.3348002E37F, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)48189, PH.base.pack) ;
        p232_vert_accuracy_SET((float)2.7551856E38F, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p232_vd_SET((float) -3.233995E38F, PH.base.pack) ;
        p232_alt_SET((float) -2.4603725E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTCM_DATA_233(), &PH);
        p233_len_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)236, (uint8_t)129, (uint8_t)95, (uint8_t)60, (uint8_t)162, (uint8_t)173, (uint8_t)122, (uint8_t)94, (uint8_t)49, (uint8_t)100, (uint8_t)82, (uint8_t)206, (uint8_t)136, (uint8_t)95, (uint8_t)229, (uint8_t)99, (uint8_t)65, (uint8_t)30, (uint8_t)121, (uint8_t)35, (uint8_t)246, (uint8_t)166, (uint8_t)37, (uint8_t)249, (uint8_t)208, (uint8_t)87, (uint8_t)74, (uint8_t)7, (uint8_t)58, (uint8_t)49, (uint8_t)207, (uint8_t)148, (uint8_t)113, (uint8_t)148, (uint8_t)179, (uint8_t)98, (uint8_t)86, (uint8_t)66, (uint8_t)214, (uint8_t)3, (uint8_t)151, (uint8_t)241, (uint8_t)233, (uint8_t)228, (uint8_t)122, (uint8_t)231, (uint8_t)84, (uint8_t)234, (uint8_t)224, (uint8_t)176, (uint8_t)87, (uint8_t)190, (uint8_t)210, (uint8_t)222, (uint8_t)93, (uint8_t)71, (uint8_t)34, (uint8_t)130, (uint8_t)29, (uint8_t)65, (uint8_t)200, (uint8_t)56, (uint8_t)44, (uint8_t)50, (uint8_t)90, (uint8_t)235, (uint8_t)96, (uint8_t)148, (uint8_t)235, (uint8_t)226, (uint8_t)25, (uint8_t)198, (uint8_t)210, (uint8_t)72, (uint8_t)113, (uint8_t)240, (uint8_t)69, (uint8_t)187, (uint8_t)228, (uint8_t)80, (uint8_t)0, (uint8_t)201, (uint8_t)167, (uint8_t)100, (uint8_t)247, (uint8_t)202, (uint8_t)65, (uint8_t)42, (uint8_t)146, (uint8_t)137, (uint8_t)130, (uint8_t)163, (uint8_t)248, (uint8_t)234, (uint8_t)165, (uint8_t)22, (uint8_t)86, (uint8_t)211, (uint8_t)73, (uint8_t)170, (uint8_t)9, (uint8_t)98, (uint8_t)211, (uint8_t)95, (uint8_t)83, (uint8_t)204, (uint8_t)114, (uint8_t)176, (uint8_t)51, (uint8_t)32, (uint8_t)183, (uint8_t)104, (uint8_t)201, (uint8_t)19, (uint8_t)241, (uint8_t)248, (uint8_t)116, (uint8_t)184, (uint8_t)147, (uint8_t)222, (uint8_t)171, (uint8_t)254, (uint8_t)34, (uint8_t)148, (uint8_t)251, (uint8_t)30, (uint8_t)50, (uint8_t)21, (uint8_t)85, (uint8_t)131, (uint8_t)22, (uint8_t)224, (uint8_t)76, (uint8_t)90, (uint8_t)77, (uint8_t)90, (uint8_t)98, (uint8_t)6, (uint8_t)244, (uint8_t)225, (uint8_t)152, (uint8_t)58, (uint8_t)62, (uint8_t)211, (uint8_t)86, (uint8_t)253, (uint8_t)173, (uint8_t)252, (uint8_t)201, (uint8_t)157, (uint8_t)105, (uint8_t)22, (uint8_t)131, (uint8_t)56, (uint8_t)195, (uint8_t)19, (uint8_t)190, (uint8_t)73, (uint8_t)202, (uint8_t)72, (uint8_t)58, (uint8_t)41, (uint8_t)132, (uint8_t)113, (uint8_t)186, (uint8_t)91, (uint8_t)202, (uint8_t)18, (uint8_t)216, (uint8_t)115, (uint8_t)225, (uint8_t)102, (uint8_t)31, (uint8_t)129, (uint8_t)218, (uint8_t)180, (uint8_t)40, (uint8_t)39, (uint8_t)71, (uint8_t)81};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        p233_flags_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGH_LATENCY_234(), &PH);
        p234_airspeed_sp_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t) -19, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)1028590280L, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
        p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t) -2548, PH.base.pack) ;
        p234_longitude_SET((int32_t)310876586, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)6, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t)15, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)54822, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t) -17593, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t) -116, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t) -28571, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_FIX, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t) -19855, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t) -14432, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        p234_latitude_SET((int32_t)332871058, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t) -127, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)38892, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIBRATION_241(), &PH);
        p241_vibration_y_SET((float) -3.3649986E38F, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)1990133049L, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)1331707777L, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)274320862L, PH.base.pack) ;
        p241_vibration_x_SET((float) -1.8421609E37F, PH.base.pack) ;
        p241_vibration_z_SET((float)1.952359E38F, PH.base.pack) ;
        p241_time_usec_SET((uint64_t)2649767703318378168L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HOME_POSITION_242(), &PH);
        p242_longitude_SET((int32_t) -174092834, PH.base.pack) ;
        p242_latitude_SET((int32_t)357603074, PH.base.pack) ;
        p242_approach_z_SET((float)3.945386E37F, PH.base.pack) ;
        {
            float q[] =  {-2.4927116E38F, -9.354272E37F, -4.800433E37F, -8.0089835E37F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_approach_y_SET((float)2.9953799E38F, PH.base.pack) ;
        p242_altitude_SET((int32_t) -199153132, PH.base.pack) ;
        p242_y_SET((float) -2.8488717E38F, PH.base.pack) ;
        p242_z_SET((float)1.704071E38F, PH.base.pack) ;
        p242_x_SET((float) -3.1197646E38F, PH.base.pack) ;
        p242_approach_x_SET((float)9.832744E35F, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)7403138715367473169L, &PH) ;
        c_LoopBackDemoChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_HOME_POSITION_243(), &PH);
        p243_latitude_SET((int32_t)546257757, PH.base.pack) ;
        p243_x_SET((float)3.0121123E38F, PH.base.pack) ;
        p243_y_SET((float)8.794241E36F, PH.base.pack) ;
        p243_approach_x_SET((float) -1.0615717E38F, PH.base.pack) ;
        p243_altitude_SET((int32_t) -2020291669, PH.base.pack) ;
        p243_approach_z_SET((float) -1.4029907E38F, PH.base.pack) ;
        p243_z_SET((float) -3.238082E38F, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)3624049427776006904L, &PH) ;
        p243_approach_y_SET((float)3.3198613E38F, PH.base.pack) ;
        {
            float q[] =  {2.097284E38F, -4.733057E35F, 3.2660475E37F, 6.231203E37F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_longitude_SET((int32_t)1654014446, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_message_id_SET((uint16_t)(uint16_t)19404, PH.base.pack) ;
        p244_interval_us_SET((int32_t)511832011, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND, PH.base.pack) ;
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW, PH.base.pack) ;
        c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ADSB_VEHICLE_246(), &PH);
        p246_altitude_SET((int32_t) -1954982159, PH.base.pack) ;
        p246_lon_SET((int32_t)1883219111, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_HIGHLY_MANUV, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)3586869764L, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)25900, PH.base.pack) ;
        {
            char16_t* callsign = u"muligytd";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_ver_velocity_SET((int16_t)(int16_t)23333, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)28861, PH.base.pack) ;
        p246_lat_SET((int32_t)668032009, PH.base.pack) ;
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
        p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)51501, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COLLISION_247(), &PH);
        p247_id_SET((uint32_t)2355477366L, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_HOVER, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float)9.646423E37F, PH.base.pack) ;
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE, PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float)1.1264116E38F, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float)2.7698488E38F, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_V2_EXTENSION_248(), &PH);
        {
            uint8_t payload[] =  {(uint8_t)73, (uint8_t)211, (uint8_t)108, (uint8_t)82, (uint8_t)144, (uint8_t)109, (uint8_t)33, (uint8_t)182, (uint8_t)143, (uint8_t)89, (uint8_t)74, (uint8_t)103, (uint8_t)253, (uint8_t)236, (uint8_t)37, (uint8_t)65, (uint8_t)15, (uint8_t)63, (uint8_t)179, (uint8_t)139, (uint8_t)153, (uint8_t)162, (uint8_t)117, (uint8_t)76, (uint8_t)239, (uint8_t)27, (uint8_t)38, (uint8_t)128, (uint8_t)200, (uint8_t)224, (uint8_t)154, (uint8_t)142, (uint8_t)87, (uint8_t)94, (uint8_t)192, (uint8_t)183, (uint8_t)3, (uint8_t)66, (uint8_t)17, (uint8_t)212, (uint8_t)164, (uint8_t)182, (uint8_t)245, (uint8_t)16, (uint8_t)187, (uint8_t)159, (uint8_t)243, (uint8_t)184, (uint8_t)142, (uint8_t)110, (uint8_t)93, (uint8_t)5, (uint8_t)213, (uint8_t)15, (uint8_t)196, (uint8_t)247, (uint8_t)227, (uint8_t)73, (uint8_t)54, (uint8_t)79, (uint8_t)153, (uint8_t)112, (uint8_t)56, (uint8_t)42, (uint8_t)221, (uint8_t)169, (uint8_t)45, (uint8_t)172, (uint8_t)253, (uint8_t)188, (uint8_t)187, (uint8_t)254, (uint8_t)165, (uint8_t)28, (uint8_t)136, (uint8_t)203, (uint8_t)63, (uint8_t)102, (uint8_t)14, (uint8_t)128, (uint8_t)129, (uint8_t)102, (uint8_t)132, (uint8_t)201, (uint8_t)21, (uint8_t)55, (uint8_t)240, (uint8_t)49, (uint8_t)11, (uint8_t)126, (uint8_t)113, (uint8_t)1, (uint8_t)235, (uint8_t)178, (uint8_t)155, (uint8_t)189, (uint8_t)105, (uint8_t)72, (uint8_t)70, (uint8_t)96, (uint8_t)77, (uint8_t)34, (uint8_t)77, (uint8_t)223, (uint8_t)251, (uint8_t)43, (uint8_t)16, (uint8_t)208, (uint8_t)221, (uint8_t)83, (uint8_t)90, (uint8_t)12, (uint8_t)14, (uint8_t)26, (uint8_t)92, (uint8_t)136, (uint8_t)169, (uint8_t)105, (uint8_t)148, (uint8_t)227, (uint8_t)48, (uint8_t)78, (uint8_t)196, (uint8_t)223, (uint8_t)71, (uint8_t)168, (uint8_t)213, (uint8_t)202, (uint8_t)243, (uint8_t)249, (uint8_t)96, (uint8_t)229, (uint8_t)146, (uint8_t)159, (uint8_t)245, (uint8_t)34, (uint8_t)7, (uint8_t)42, (uint8_t)204, (uint8_t)40, (uint8_t)160, (uint8_t)171, (uint8_t)200, (uint8_t)15, (uint8_t)253, (uint8_t)213, (uint8_t)145, (uint8_t)166, (uint8_t)182, (uint8_t)95, (uint8_t)140, (uint8_t)78, (uint8_t)161, (uint8_t)233, (uint8_t)150, (uint8_t)21, (uint8_t)121, (uint8_t)29, (uint8_t)207, (uint8_t)41, (uint8_t)75, (uint8_t)75, (uint8_t)165, (uint8_t)30, (uint8_t)171, (uint8_t)87, (uint8_t)53, (uint8_t)20, (uint8_t)98, (uint8_t)167, (uint8_t)19, (uint8_t)210, (uint8_t)132, (uint8_t)150, (uint8_t)33, (uint8_t)43, (uint8_t)143, (uint8_t)209, (uint8_t)44, (uint8_t)201, (uint8_t)29, (uint8_t)16, (uint8_t)251, (uint8_t)3, (uint8_t)115, (uint8_t)68, (uint8_t)135, (uint8_t)83, (uint8_t)121, (uint8_t)27, (uint8_t)36, (uint8_t)182, (uint8_t)77, (uint8_t)143, (uint8_t)109, (uint8_t)214, (uint8_t)156, (uint8_t)35, (uint8_t)164, (uint8_t)214, (uint8_t)21, (uint8_t)154, (uint8_t)29, (uint8_t)215, (uint8_t)0, (uint8_t)173, (uint8_t)55, (uint8_t)96, (uint8_t)160, (uint8_t)74, (uint8_t)110, (uint8_t)43, (uint8_t)237, (uint8_t)160, (uint8_t)83, (uint8_t)248, (uint8_t)226, (uint8_t)87, (uint8_t)142, (uint8_t)136, (uint8_t)118, (uint8_t)141, (uint8_t)156, (uint8_t)79, (uint8_t)118, (uint8_t)149, (uint8_t)88, (uint8_t)119, (uint8_t)242, (uint8_t)0, (uint8_t)72, (uint8_t)170, (uint8_t)68, (uint8_t)226, (uint8_t)164, (uint8_t)117, (uint8_t)100, (uint8_t)167, (uint8_t)211, (uint8_t)101, (uint8_t)236, (uint8_t)21, (uint8_t)163, (uint8_t)103, (uint8_t)90, (uint8_t)251, (uint8_t)223, (uint8_t)113, (uint8_t)123};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p248_target_network_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
        p248_target_system_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        p248_message_type_SET((uint16_t)(uint16_t)22802, PH.base.pack) ;
        p248_target_component_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        c_LoopBackDemoChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MEMORY_VECT_249(), &PH);
        p249_address_SET((uint16_t)(uint16_t)30239, PH.base.pack) ;
        {
            int8_t value[] =  {(int8_t) -13, (int8_t)34, (int8_t) -55, (int8_t) -38, (int8_t) -108, (int8_t) -60, (int8_t) -28, (int8_t) -111, (int8_t)81, (int8_t) -106, (int8_t)95, (int8_t)34, (int8_t)57, (int8_t) -89, (int8_t)122, (int8_t)13, (int8_t) -52, (int8_t) -126, (int8_t) -89, (int8_t)7, (int8_t) -126, (int8_t) -69, (int8_t) -82, (int8_t) -13, (int8_t)110, (int8_t)103, (int8_t) -22, (int8_t)2, (int8_t) -49, (int8_t)48, (int8_t) -116, (int8_t) -119};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_ver_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        p249_type_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEBUG_VECT_250(), &PH);
        p250_y_SET((float) -1.0727332E38F, PH.base.pack) ;
        p250_time_usec_SET((uint64_t)3027300410375224418L, PH.base.pack) ;
        p250_x_SET((float) -1.6659714E38F, PH.base.pack) ;
        {
            char16_t* name = u"gwzpbe";
            p250_name_SET_(name, &PH) ;
        }
        p250_z_SET((float) -1.9761509E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
        p251_value_SET((float)9.119059E37F, PH.base.pack) ;
        p251_time_boot_ms_SET((uint32_t)2555712429L, PH.base.pack) ;
        {
            char16_t* name = u"enkzi";
            p251_name_SET_(name, &PH) ;
        }
        c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_INT_252(), &PH);
        p252_time_boot_ms_SET((uint32_t)397974298L, PH.base.pack) ;
        {
            char16_t* name = u"rTz";
            p252_name_SET_(name, &PH) ;
        }
        p252_value_SET((int32_t)1319768634, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STATUSTEXT_253(), &PH);
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_INFO, PH.base.pack) ;
        {
            char16_t* text = u"xluvYxfbtbnlqpgdbguleoqce";
            p253_text_SET_(text, &PH) ;
        }
        c_LoopBackDemoChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEBUG_254(), &PH);
        p254_time_boot_ms_SET((uint32_t)303994040L, PH.base.pack) ;
        p254_value_SET((float)3.1525967E38F, PH.base.pack) ;
        p254_ind_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SETUP_SIGNING_256(), &PH);
        p256_target_component_SET((uint8_t)(uint8_t)117, PH.base.pack) ;
        p256_target_system_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)208, (uint8_t)123, (uint8_t)38, (uint8_t)35, (uint8_t)202, (uint8_t)18, (uint8_t)88, (uint8_t)244, (uint8_t)249, (uint8_t)204, (uint8_t)243, (uint8_t)151, (uint8_t)189, (uint8_t)130, (uint8_t)186, (uint8_t)153, (uint8_t)21, (uint8_t)19, (uint8_t)177, (uint8_t)24, (uint8_t)126, (uint8_t)58, (uint8_t)167, (uint8_t)18, (uint8_t)33, (uint8_t)176, (uint8_t)243, (uint8_t)172, (uint8_t)228, (uint8_t)184, (uint8_t)118, (uint8_t)200};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        p256_initial_timestamp_SET((uint64_t)2820896605047503436L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BUTTON_CHANGE_257(), &PH);
        p257_time_boot_ms_SET((uint32_t)1758702292L, PH.base.pack) ;
        p257_last_change_ms_SET((uint32_t)574848545L, PH.base.pack) ;
        p257_state_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PLAY_TUNE_258(), &PH);
        p258_target_component_SET((uint8_t)(uint8_t)115, PH.base.pack) ;
        {
            char16_t* tune = u"vhyultpuuwRIswvlNjozd";
            p258_tune_SET_(tune, &PH) ;
        }
        p258_target_system_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_INFORMATION_259(), &PH);
        p259_resolution_v_SET((uint16_t)(uint16_t)62221, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"kjVtxxhNygdypsNrxsuDacxlqtxwynzziqcicfuekxmqMslpuvveiqafeEqqCwkbZqudvQepwropbdgysbcnidgz";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_sensor_size_h_SET((float)1.7616076E38F, PH.base.pack) ;
        p259_focal_length_SET((float)1.1809058E38F, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)32, (uint8_t)176, (uint8_t)89, (uint8_t)184, (uint8_t)34, (uint8_t)144, (uint8_t)250, (uint8_t)168, (uint8_t)168, (uint8_t)206, (uint8_t)210, (uint8_t)64, (uint8_t)63, (uint8_t)11, (uint8_t)81, (uint8_t)2, (uint8_t)94, (uint8_t)195, (uint8_t)169, (uint8_t)104, (uint8_t)238, (uint8_t)17, (uint8_t)208, (uint8_t)160, (uint8_t)243, (uint8_t)130, (uint8_t)152, (uint8_t)46, (uint8_t)219, (uint8_t)50, (uint8_t)46, (uint8_t)93};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_resolution_h_SET((uint16_t)(uint16_t)2445, PH.base.pack) ;
        p259_cam_definition_version_SET((uint16_t)(uint16_t)36674, PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)168, (uint8_t)107, (uint8_t)62, (uint8_t)217, (uint8_t)82, (uint8_t)96, (uint8_t)70, (uint8_t)188, (uint8_t)95, (uint8_t)22, (uint8_t)15, (uint8_t)253, (uint8_t)57, (uint8_t)158, (uint8_t)124, (uint8_t)26, (uint8_t)82, (uint8_t)128, (uint8_t)37, (uint8_t)61, (uint8_t)227, (uint8_t)171, (uint8_t)93, (uint8_t)189, (uint8_t)122, (uint8_t)56, (uint8_t)130, (uint8_t)210, (uint8_t)77, (uint8_t)236, (uint8_t)19, (uint8_t)171};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_time_boot_ms_SET((uint32_t)2417296277L, PH.base.pack) ;
        p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE, PH.base.pack) ;
        p259_sensor_size_v_SET((float) -5.6201594E37F, PH.base.pack) ;
        p259_firmware_version_SET((uint32_t)497768214L, PH.base.pack) ;
        p259_lens_id_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_SETTINGS_260(), &PH);
        p260_time_boot_ms_SET((uint32_t)208363571L, PH.base.pack) ;
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_VIDEO, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STORAGE_INFORMATION_261(), &PH);
        p261_storage_count_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        p261_read_speed_SET((float)3.3001087E38F, PH.base.pack) ;
        p261_total_capacity_SET((float)1.9682963E38F, PH.base.pack) ;
        p261_write_speed_SET((float)1.6166886E38F, PH.base.pack) ;
        p261_available_capacity_SET((float)8.396993E37F, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p261_used_capacity_SET((float) -1.0681241E38F, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)96763514L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_image_interval_SET((float)7.1712074E37F, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)3273184033L, PH.base.pack) ;
        p262_available_capacity_SET((float) -1.3131566E37F, PH.base.pack) ;
        p262_recording_time_ms_SET((uint32_t)2808172676L, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_lon_SET((int32_t) -1124557312, PH.base.pack) ;
        {
            char16_t* file_url = u"pvshfpYjuljnxCloguqvoiaXzmDZijFyxyqHahyrdscQqrrylrWahwddhtgYTYkcdgwtWhUf";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_time_boot_ms_SET((uint32_t)4206286646L, PH.base.pack) ;
        p263_image_index_SET((int32_t)1682594756, PH.base.pack) ;
        p263_time_utc_SET((uint64_t)9197667099251689232L, PH.base.pack) ;
        p263_camera_id_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p263_relative_alt_SET((int32_t)124379189, PH.base.pack) ;
        p263_lat_SET((int32_t)328399714, PH.base.pack) ;
        p263_capture_result_SET((int8_t)(int8_t)61, PH.base.pack) ;
        {
            float q[] =  {-3.0787955E38F, 2.9240276E38F, -6.805334E37F, -8.451733E37F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_alt_SET((int32_t)2048365045, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_time_boot_ms_SET((uint32_t)2513094030L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)6490037615656584532L, PH.base.pack) ;
        p264_takeoff_time_utc_SET((uint64_t)3758199398182636261L, PH.base.pack) ;
        p264_arming_time_utc_SET((uint64_t)5784841417719717045L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_pitch_SET((float)1.3567627E38F, PH.base.pack) ;
        p265_yaw_SET((float) -2.64721E38F, PH.base.pack) ;
        p265_time_boot_ms_SET((uint32_t)1579153439L, PH.base.pack) ;
        p265_roll_SET((float) -1.6656885E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_266(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)123, (uint8_t)70, (uint8_t)35, (uint8_t)177, (uint8_t)182, (uint8_t)108, (uint8_t)209, (uint8_t)74, (uint8_t)188, (uint8_t)131, (uint8_t)232, (uint8_t)148, (uint8_t)188, (uint8_t)135, (uint8_t)4, (uint8_t)85, (uint8_t)23, (uint8_t)124, (uint8_t)168, (uint8_t)252, (uint8_t)2, (uint8_t)135, (uint8_t)244, (uint8_t)238, (uint8_t)106, (uint8_t)73, (uint8_t)170, (uint8_t)227, (uint8_t)239, (uint8_t)76, (uint8_t)228, (uint8_t)141, (uint8_t)62, (uint8_t)176, (uint8_t)47, (uint8_t)57, (uint8_t)157, (uint8_t)10, (uint8_t)73, (uint8_t)112, (uint8_t)167, (uint8_t)51, (uint8_t)72, (uint8_t)163, (uint8_t)217, (uint8_t)114, (uint8_t)130, (uint8_t)4, (uint8_t)89, (uint8_t)110, (uint8_t)240, (uint8_t)148, (uint8_t)187, (uint8_t)8, (uint8_t)169, (uint8_t)246, (uint8_t)121, (uint8_t)161, (uint8_t)39, (uint8_t)9, (uint8_t)92, (uint8_t)138, (uint8_t)217, (uint8_t)230, (uint8_t)161, (uint8_t)93, (uint8_t)87, (uint8_t)84, (uint8_t)47, (uint8_t)151, (uint8_t)177, (uint8_t)52, (uint8_t)251, (uint8_t)238, (uint8_t)77, (uint8_t)115, (uint8_t)67, (uint8_t)19, (uint8_t)85, (uint8_t)84, (uint8_t)226, (uint8_t)225, (uint8_t)52, (uint8_t)246, (uint8_t)127, (uint8_t)213, (uint8_t)38, (uint8_t)250, (uint8_t)158, (uint8_t)142, (uint8_t)45, (uint8_t)250, (uint8_t)245, (uint8_t)159, (uint8_t)191, (uint8_t)140, (uint8_t)227, (uint8_t)71, (uint8_t)40, (uint8_t)169, (uint8_t)176, (uint8_t)114, (uint8_t)139, (uint8_t)24, (uint8_t)11, (uint8_t)246, (uint8_t)83, (uint8_t)242, (uint8_t)47, (uint8_t)5, (uint8_t)94, (uint8_t)71, (uint8_t)229, (uint8_t)139, (uint8_t)86, (uint8_t)57, (uint8_t)42, (uint8_t)144, (uint8_t)8, (uint8_t)59, (uint8_t)170, (uint8_t)120, (uint8_t)201, (uint8_t)245, (uint8_t)112, (uint8_t)64, (uint8_t)210, (uint8_t)103, (uint8_t)63, (uint8_t)95, (uint8_t)107, (uint8_t)145, (uint8_t)229, (uint8_t)24, (uint8_t)35, (uint8_t)210, (uint8_t)184, (uint8_t)198, (uint8_t)84, (uint8_t)13, (uint8_t)57, (uint8_t)17, (uint8_t)137, (uint8_t)243, (uint8_t)95, (uint8_t)183, (uint8_t)102, (uint8_t)3, (uint8_t)59, (uint8_t)40, (uint8_t)223, (uint8_t)14, (uint8_t)215, (uint8_t)253, (uint8_t)4, (uint8_t)94, (uint8_t)254, (uint8_t)63, (uint8_t)250, (uint8_t)204, (uint8_t)53, (uint8_t)91, (uint8_t)124, (uint8_t)53, (uint8_t)147, (uint8_t)189, (uint8_t)169, (uint8_t)143, (uint8_t)212, (uint8_t)229, (uint8_t)227, (uint8_t)255, (uint8_t)205, (uint8_t)130, (uint8_t)70, (uint8_t)50, (uint8_t)16, (uint8_t)201, (uint8_t)207, (uint8_t)24, (uint8_t)203, (uint8_t)40, (uint8_t)255, (uint8_t)247, (uint8_t)129, (uint8_t)44, (uint8_t)126, (uint8_t)198, (uint8_t)45, (uint8_t)193, (uint8_t)81, (uint8_t)116, (uint8_t)114, (uint8_t)134, (uint8_t)118, (uint8_t)33, (uint8_t)106, (uint8_t)184, (uint8_t)145, (uint8_t)255, (uint8_t)103, (uint8_t)121, (uint8_t)117, (uint8_t)98, (uint8_t)183, (uint8_t)240, (uint8_t)52, (uint8_t)94, (uint8_t)189, (uint8_t)118, (uint8_t)96, (uint8_t)103, (uint8_t)24, (uint8_t)115, (uint8_t)44, (uint8_t)111, (uint8_t)40, (uint8_t)218, (uint8_t)112, (uint8_t)100, (uint8_t)8, (uint8_t)8, (uint8_t)216, (uint8_t)138, (uint8_t)66, (uint8_t)9, (uint8_t)30, (uint8_t)190, (uint8_t)254, (uint8_t)144, (uint8_t)143, (uint8_t)119, (uint8_t)255, (uint8_t)34, (uint8_t)26, (uint8_t)233, (uint8_t)31, (uint8_t)170, (uint8_t)71, (uint8_t)178, (uint8_t)117, (uint8_t)130, (uint8_t)250, (uint8_t)115, (uint8_t)7, (uint8_t)35, (uint8_t)202, (uint8_t)131, (uint8_t)131};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_length_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        p266_first_message_offset_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        p266_target_system_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p266_sequence_SET((uint16_t)(uint16_t)14121, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_ACKED_267(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)153, (uint8_t)203, (uint8_t)181, (uint8_t)25, (uint8_t)207, (uint8_t)152, (uint8_t)186, (uint8_t)68, (uint8_t)35, (uint8_t)228, (uint8_t)69, (uint8_t)205, (uint8_t)164, (uint8_t)235, (uint8_t)182, (uint8_t)77, (uint8_t)205, (uint8_t)140, (uint8_t)179, (uint8_t)237, (uint8_t)117, (uint8_t)119, (uint8_t)71, (uint8_t)254, (uint8_t)173, (uint8_t)71, (uint8_t)6, (uint8_t)155, (uint8_t)246, (uint8_t)233, (uint8_t)202, (uint8_t)176, (uint8_t)181, (uint8_t)15, (uint8_t)249, (uint8_t)173, (uint8_t)7, (uint8_t)150, (uint8_t)227, (uint8_t)112, (uint8_t)123, (uint8_t)186, (uint8_t)253, (uint8_t)177, (uint8_t)41, (uint8_t)28, (uint8_t)249, (uint8_t)220, (uint8_t)138, (uint8_t)28, (uint8_t)39, (uint8_t)58, (uint8_t)249, (uint8_t)158, (uint8_t)35, (uint8_t)14, (uint8_t)92, (uint8_t)162, (uint8_t)110, (uint8_t)209, (uint8_t)19, (uint8_t)165, (uint8_t)135, (uint8_t)224, (uint8_t)28, (uint8_t)144, (uint8_t)239, (uint8_t)42, (uint8_t)113, (uint8_t)99, (uint8_t)105, (uint8_t)233, (uint8_t)155, (uint8_t)181, (uint8_t)146, (uint8_t)189, (uint8_t)171, (uint8_t)75, (uint8_t)130, (uint8_t)35, (uint8_t)192, (uint8_t)196, (uint8_t)41, (uint8_t)111, (uint8_t)7, (uint8_t)149, (uint8_t)50, (uint8_t)119, (uint8_t)55, (uint8_t)33, (uint8_t)62, (uint8_t)111, (uint8_t)63, (uint8_t)14, (uint8_t)100, (uint8_t)167, (uint8_t)129, (uint8_t)210, (uint8_t)7, (uint8_t)174, (uint8_t)144, (uint8_t)192, (uint8_t)214, (uint8_t)123, (uint8_t)183, (uint8_t)5, (uint8_t)155, (uint8_t)227, (uint8_t)56, (uint8_t)30, (uint8_t)188, (uint8_t)78, (uint8_t)20, (uint8_t)224, (uint8_t)183, (uint8_t)146, (uint8_t)13, (uint8_t)84, (uint8_t)59, (uint8_t)49, (uint8_t)91, (uint8_t)52, (uint8_t)60, (uint8_t)129, (uint8_t)114, (uint8_t)6, (uint8_t)144, (uint8_t)172, (uint8_t)173, (uint8_t)156, (uint8_t)19, (uint8_t)92, (uint8_t)52, (uint8_t)124, (uint8_t)67, (uint8_t)47, (uint8_t)23, (uint8_t)87, (uint8_t)44, (uint8_t)9, (uint8_t)218, (uint8_t)145, (uint8_t)177, (uint8_t)106, (uint8_t)88, (uint8_t)154, (uint8_t)71, (uint8_t)86, (uint8_t)137, (uint8_t)60, (uint8_t)7, (uint8_t)212, (uint8_t)133, (uint8_t)35, (uint8_t)167, (uint8_t)134, (uint8_t)112, (uint8_t)22, (uint8_t)209, (uint8_t)101, (uint8_t)8, (uint8_t)179, (uint8_t)244, (uint8_t)33, (uint8_t)237, (uint8_t)129, (uint8_t)47, (uint8_t)209, (uint8_t)132, (uint8_t)158, (uint8_t)139, (uint8_t)173, (uint8_t)105, (uint8_t)66, (uint8_t)211, (uint8_t)10, (uint8_t)18, (uint8_t)78, (uint8_t)209, (uint8_t)124, (uint8_t)33, (uint8_t)164, (uint8_t)55, (uint8_t)223, (uint8_t)193, (uint8_t)84, (uint8_t)11, (uint8_t)248, (uint8_t)146, (uint8_t)64, (uint8_t)111, (uint8_t)145, (uint8_t)205, (uint8_t)203, (uint8_t)124, (uint8_t)248, (uint8_t)20, (uint8_t)180, (uint8_t)228, (uint8_t)9, (uint8_t)21, (uint8_t)30, (uint8_t)116, (uint8_t)47, (uint8_t)36, (uint8_t)33, (uint8_t)42, (uint8_t)89, (uint8_t)36, (uint8_t)118, (uint8_t)66, (uint8_t)44, (uint8_t)0, (uint8_t)17, (uint8_t)4, (uint8_t)236, (uint8_t)250, (uint8_t)131, (uint8_t)104, (uint8_t)6, (uint8_t)16, (uint8_t)46, (uint8_t)233, (uint8_t)147, (uint8_t)238, (uint8_t)244, (uint8_t)14, (uint8_t)0, (uint8_t)109, (uint8_t)163, (uint8_t)80, (uint8_t)197, (uint8_t)127, (uint8_t)174, (uint8_t)157, (uint8_t)142, (uint8_t)44, (uint8_t)42, (uint8_t)126, (uint8_t)112, (uint8_t)80, (uint8_t)174, (uint8_t)113, (uint8_t)217, (uint8_t)212, (uint8_t)79, (uint8_t)18, (uint8_t)80, (uint8_t)252};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        p267_target_system_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p267_sequence_SET((uint16_t)(uint16_t)57975, PH.base.pack) ;
        p267_first_message_offset_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p267_target_component_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p267_length_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_ACK_268(), &PH);
        p268_sequence_SET((uint16_t)(uint16_t)64146, PH.base.pack) ;
        p268_target_component_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
        p268_target_system_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_rotation_SET((uint16_t)(uint16_t)40711, PH.base.pack) ;
        p269_bitrate_SET((uint32_t)1598502410L, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)21256, PH.base.pack) ;
        p269_framerate_SET((float)1.7457027E38F, PH.base.pack) ;
        p269_status_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
        {
            char16_t* uri = u"ZgqwzkpafAbqbwbkgQRbocaxvfyobfqocoundbbdbNGhtlanopokkkCksirgzdotakrilkpmPlhujiaBvcOrvnsstbznOnyyopqqlumcuyPlfnvaklrdhqynugtvMVMyBQfjyjRmtaimbupwbkwihEoxmcazvhsBtwdsbyg";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_resolution_v_SET((uint16_t)(uint16_t)17043, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_resolution_h_SET((uint16_t)(uint16_t)4341, PH.base.pack) ;
        {
            char16_t* uri = u"gBkGzkqhuevdrcgdqjaDcvxqnvokWsuqvvjveTdavpkfmciKaxljbfNx";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_framerate_SET((float) -2.186394E38F, PH.base.pack) ;
        p270_target_component_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)180048792L, PH.base.pack) ;
        p270_rotation_SET((uint16_t)(uint16_t)23944, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)12784, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* password = u"ikirlmzdcbkshrtmgzHMahfvvrrdzamvWzyfsemcatcbxuujMdaJIL";
            p299_password_SET_(password, &PH) ;
        }
        {
            char16_t* ssid = u"CqbTepxRyYfwadg";
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
            uint8_t spec_version_hash[] =  {(uint8_t)101, (uint8_t)71, (uint8_t)147, (uint8_t)16, (uint8_t)221, (uint8_t)23, (uint8_t)123, (uint8_t)170};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        p300_min_version_SET((uint16_t)(uint16_t)53058, PH.base.pack) ;
        p300_version_SET((uint16_t)(uint16_t)58237, PH.base.pack) ;
        p300_max_version_SET((uint16_t)(uint16_t)39206, PH.base.pack) ;
        {
            uint8_t library_version_hash[] =  {(uint8_t)64, (uint8_t)211, (uint8_t)242, (uint8_t)211, (uint8_t)9, (uint8_t)254, (uint8_t)168, (uint8_t)235};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_time_usec_SET((uint64_t)6217231610304152909L, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)51041, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)1216213891L, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_sw_vcs_commit_SET((uint32_t)1035911788L, PH.base.pack) ;
        p311_hw_version_minor_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)11, (uint8_t)225, (uint8_t)141, (uint8_t)245, (uint8_t)6, (uint8_t)35, (uint8_t)221, (uint8_t)153, (uint8_t)37, (uint8_t)5, (uint8_t)62, (uint8_t)171, (uint8_t)207, (uint8_t)143, (uint8_t)229, (uint8_t)169};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        p311_sw_version_minor_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
        p311_uptime_sec_SET((uint32_t)1921693059L, PH.base.pack) ;
        p311_sw_version_major_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        {
            char16_t* name = u"YbsmymkkIpkibbltnipdjihu";
            p311_name_SET_(name, &PH) ;
        }
        p311_time_usec_SET((uint64_t)7863210411443711174L, PH.base.pack) ;
        p311_hw_version_major_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        p320_target_component_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        {
            char16_t* param_id = u"arYurbapumkopPpc";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_param_index_SET((int16_t)(int16_t)22898, PH.base.pack) ;
        p320_target_system_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_system_SET((uint8_t)(uint8_t)86, PH.base.pack) ;
        p321_target_component_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_VALUE_322(), &PH);
        {
            char16_t* param_value = u"pirnrxqleuzyhkgxrjhtxtsdibjrobmrJhhntBbrqZyeqerqWsvshsrchkikhgPhhRork";
            p322_param_value_SET_(param_value, &PH) ;
        }
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8, PH.base.pack) ;
        {
            char16_t* param_id = u"qwsyajbbxbpwik";
            p322_param_id_SET_(param_id, &PH) ;
        }
        p322_param_count_SET((uint16_t)(uint16_t)28665, PH.base.pack) ;
        p322_param_index_SET((uint16_t)(uint16_t)50562, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_SET_323(), &PH);
        p323_target_system_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        {
            char16_t* param_id = u"dEaoifxgsfR";
            p323_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"xllyjpfbksqkwyjrpAukjlgomtrxxikarpdrsjywtvwvrcunfeiDlmoviowzuqukuuKahMmcyyeKyajpryduykagcmqqhYlih";
            p323_param_value_SET_(param_value, &PH) ;
        }
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32, PH.base.pack) ;
        p323_target_component_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_ACK_324(), &PH);
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_ACCEPTED, PH.base.pack) ;
        {
            char16_t* param_id = u"OBmwpmUrDbictj";
            p324_param_id_SET_(param_id, &PH) ;
        }
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32, PH.base.pack) ;
        {
            char16_t* param_value = u"kvoalrkghepljs";
            p324_param_value_SET_(param_value, &PH) ;
        }
        c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OBSTACLE_DISTANCE_330(), &PH);
        {
            uint16_t distances[] =  {(uint16_t)52167, (uint16_t)49703, (uint16_t)34223, (uint16_t)34217, (uint16_t)44194, (uint16_t)6242, (uint16_t)3768, (uint16_t)51858, (uint16_t)28335, (uint16_t)53773, (uint16_t)32276, (uint16_t)61293, (uint16_t)4927, (uint16_t)42861, (uint16_t)10461, (uint16_t)30623, (uint16_t)49322, (uint16_t)25991, (uint16_t)48461, (uint16_t)40976, (uint16_t)10495, (uint16_t)36480, (uint16_t)8005, (uint16_t)43062, (uint16_t)62196, (uint16_t)16768, (uint16_t)16273, (uint16_t)31916, (uint16_t)38353, (uint16_t)43956, (uint16_t)62278, (uint16_t)29595, (uint16_t)55264, (uint16_t)55795, (uint16_t)47947, (uint16_t)14882, (uint16_t)20367, (uint16_t)62009, (uint16_t)31601, (uint16_t)62541, (uint16_t)38803, (uint16_t)2276, (uint16_t)41689, (uint16_t)54276, (uint16_t)56617, (uint16_t)62689, (uint16_t)6760, (uint16_t)42473, (uint16_t)43218, (uint16_t)42858, (uint16_t)3409, (uint16_t)27087, (uint16_t)13323, (uint16_t)63926, (uint16_t)24153, (uint16_t)11993, (uint16_t)56408, (uint16_t)61331, (uint16_t)16106, (uint16_t)8310, (uint16_t)18158, (uint16_t)10199, (uint16_t)23348, (uint16_t)34581, (uint16_t)59870, (uint16_t)32542, (uint16_t)4948, (uint16_t)50136, (uint16_t)19799, (uint16_t)26650, (uint16_t)27749, (uint16_t)7226};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_time_usec_SET((uint64_t)6722039981066533068L, PH.base.pack) ;
        p330_min_distance_SET((uint16_t)(uint16_t)16426, PH.base.pack) ;
        p330_increment_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        p330_max_distance_SET((uint16_t)(uint16_t)52893, PH.base.pack) ;
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
}

