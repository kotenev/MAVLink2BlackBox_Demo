
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
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_VTOL_TILTROTOR);
    assert(p0_custom_mode_GET(pack) == (uint32_t)2738671950L);
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)214);
    assert(p0_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED);
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_EMERGENCY);
};


void c_LoopBackDemoChannel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_onboard_control_sensors_enabled_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)42862);
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)44177);
    assert(p1_onboard_control_sensors_present_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW);
    assert(p1_onboard_control_sensors_health_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)16946);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t) -25779);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)6434);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)27906);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)62410);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)26031);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)44974);
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t)118);
};


void c_LoopBackDemoChannel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)694755993L);
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)5537790853574210705L);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_vy_GET(pack) == (float)5.9839147E37F);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)29723796L);
    assert(p3_y_GET(pack) == (float) -2.7512079E38F);
    assert(p3_afy_GET(pack) == (float)4.496233E37F);
    assert(p3_yaw_GET(pack) == (float)1.5791067E38F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p3_vz_GET(pack) == (float) -2.4121089E38F);
    assert(p3_afx_GET(pack) == (float) -4.544112E37F);
    assert(p3_yaw_rate_GET(pack) == (float)3.2824632E38F);
    assert(p3_z_GET(pack) == (float)7.410594E37F);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)3530);
    assert(p3_vx_GET(pack) == (float)2.1979907E38F);
    assert(p3_x_GET(pack) == (float)1.4921827E36F);
    assert(p3_afz_GET(pack) == (float) -7.6385735E36F);
};


void c_LoopBackDemoChannel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p4_seq_GET(pack) == (uint32_t)1079617963L);
    assert(p4_time_usec_GET(pack) == (uint64_t)295212837129797761L);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)33);
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p5_passkey_LEN(ph) == 15);
    {
        char16_t * exemplary = u"duflvxubDGzuxnf";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)235);
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)144);
};


void c_LoopBackDemoChannel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 17);
    {
        char16_t * exemplary = u"xmomOxxegpxaiglhk";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 34);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_TEST_ARMED);
    assert(p11_custom_mode_GET(pack) == (uint32_t)4002579925L);
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)210);
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t)11603);
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p20_param_id_LEN(ph) == 11);
    {
        char16_t * exemplary = u"npWzjbpdwrp";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 22);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)35);
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)156);
};


void c_LoopBackDemoChannel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_id_LEN(ph) == 7);
    {
        char16_t * exemplary = u"rtwbTRl";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)31089);
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)35104);
    assert(p22_param_value_GET(pack) == (float) -4.541054E37F);
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64);
};


void c_LoopBackDemoChannel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_param_id_LEN(ph) == 8);
    {
        char16_t * exemplary = u"xTNaPBjy";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL32);
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)50);
    assert(p23_param_value_GET(pack) == (float) -2.9286487E38F);
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)11);
};


void c_LoopBackDemoChannel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_h_acc_TRY(ph) == (uint32_t)1731412571L);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)33232);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)15587);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)2826814005L);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)1023747684L);
    assert(p24_lon_GET(pack) == (int32_t) -1176944854);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p24_lat_GET(pack) == (int32_t)139828968);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t) -503660480);
    assert(p24_alt_GET(pack) == (int32_t)1734640160);
    assert(p24_v_acc_TRY(ph) == (uint32_t)1834107273L);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)48540);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)5309);
    assert(p24_time_usec_GET(pack) == (uint64_t)8726738046740334972L);
};


void c_LoopBackDemoChannel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)244, (uint8_t)168, (uint8_t)154, (uint8_t)97, (uint8_t)111, (uint8_t)23, (uint8_t)229, (uint8_t)173, (uint8_t)12, (uint8_t)160, (uint8_t)170, (uint8_t)252, (uint8_t)224, (uint8_t)151, (uint8_t)254, (uint8_t)227, (uint8_t)235, (uint8_t)124, (uint8_t)220, (uint8_t)221} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)205, (uint8_t)119, (uint8_t)46, (uint8_t)111, (uint8_t)51, (uint8_t)169, (uint8_t)21, (uint8_t)40, (uint8_t)146, (uint8_t)113, (uint8_t)87, (uint8_t)18, (uint8_t)214, (uint8_t)159, (uint8_t)134, (uint8_t)230, (uint8_t)48, (uint8_t)6, (uint8_t)95, (uint8_t)156} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)25, (uint8_t)39, (uint8_t)66, (uint8_t)78, (uint8_t)205, (uint8_t)3, (uint8_t)153, (uint8_t)3, (uint8_t)209, (uint8_t)123, (uint8_t)62, (uint8_t)37, (uint8_t)253, (uint8_t)158, (uint8_t)241, (uint8_t)93, (uint8_t)96, (uint8_t)147, (uint8_t)154, (uint8_t)10} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)153);
    {
        uint8_t exemplary[] =  {(uint8_t)125, (uint8_t)46, (uint8_t)221, (uint8_t)228, (uint8_t)91, (uint8_t)110, (uint8_t)6, (uint8_t)161, (uint8_t)225, (uint8_t)55, (uint8_t)153, (uint8_t)76, (uint8_t)238, (uint8_t)185, (uint8_t)101, (uint8_t)92, (uint8_t)112, (uint8_t)49, (uint8_t)129, (uint8_t)72} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)197, (uint8_t)255, (uint8_t)55, (uint8_t)227, (uint8_t)12, (uint8_t)6, (uint8_t)181, (uint8_t)194, (uint8_t)123, (uint8_t)222, (uint8_t)234, (uint8_t)188, (uint8_t)1, (uint8_t)222, (uint8_t)185, (uint8_t)116, (uint8_t)226, (uint8_t)99, (uint8_t)1, (uint8_t)63} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t) -21568);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t) -27123);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t) -19076);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t)7351);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t) -9004);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t)17639);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t) -1756);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t)18231);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t) -11455);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)3178484726L);
};


void c_LoopBackDemoChannel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t)1039);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t)30972);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t)15369);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t) -8081);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t)21658);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t) -17290);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t) -12330);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t) -32221);
    assert(p27_time_usec_GET(pack) == (uint64_t)1660514289681552398L);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t)16203);
};


void c_LoopBackDemoChannel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t)5548);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t) -2807);
    assert(p28_time_usec_GET(pack) == (uint64_t)322954987939889448L);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t)31337);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t)2081);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)4272831302L);
    assert(p29_press_abs_GET(pack) == (float)3.3113036E38F);
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t)2956);
    assert(p29_press_diff_GET(pack) == (float) -2.0278008E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_rollspeed_GET(pack) == (float) -2.275626E38F);
    assert(p30_pitch_GET(pack) == (float)3.5172857E37F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)686142457L);
    assert(p30_yawspeed_GET(pack) == (float) -2.4112037E38F);
    assert(p30_yaw_GET(pack) == (float) -1.3763289E38F);
    assert(p30_roll_GET(pack) == (float) -2.9523175E38F);
    assert(p30_pitchspeed_GET(pack) == (float)1.1517027E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_yawspeed_GET(pack) == (float) -3.3210433E37F);
    assert(p31_pitchspeed_GET(pack) == (float)1.2690295E38F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)1114166402L);
    assert(p31_q3_GET(pack) == (float)2.7786903E38F);
    assert(p31_q2_GET(pack) == (float)1.6304298E38F);
    assert(p31_q1_GET(pack) == (float)3.1645586E37F);
    assert(p31_rollspeed_GET(pack) == (float) -3.0380153E38F);
    assert(p31_q4_GET(pack) == (float)6.218109E36F);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_z_GET(pack) == (float) -1.0667396E38F);
    assert(p32_vz_GET(pack) == (float) -2.2068387E38F);
    assert(p32_vx_GET(pack) == (float) -7.070674E37F);
    assert(p32_x_GET(pack) == (float)3.2993804E37F);
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)525889239L);
    assert(p32_vy_GET(pack) == (float) -1.1324511E38F);
    assert(p32_y_GET(pack) == (float) -1.1857329E38F);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)2471165527L);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t) -11924);
    assert(p33_relative_alt_GET(pack) == (int32_t)319145427);
    assert(p33_lat_GET(pack) == (int32_t) -19907120);
    assert(p33_alt_GET(pack) == (int32_t)226553483);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)23299);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t)16709);
    assert(p33_lon_GET(pack) == (int32_t)1627215834);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t) -22013);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)48);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t) -16611);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t) -1718);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t)30651);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t) -22726);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)3708360954L);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t) -8791);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t) -30033);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t) -17096);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t) -27631);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)13);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)254);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)28923);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)36007);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)35720);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)46015);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)20432);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)24758);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)52247);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)25318);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)1624542723L);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)60);
};


void c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)3225);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)69);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)19097);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)15104);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)36637);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)4498);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)46534);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)64371);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)32297);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)17085);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)26132);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)37827);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)55878);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)54789);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)6263);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)22987);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)47221);
    assert(p36_time_usec_GET(pack) == (uint32_t)3776491599L);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)96);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t) -15483);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t)3785);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
};


void c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t) -7132);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t)21117);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)109);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)167);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_SET_FACTORY_TEST_MODE);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)201);
    assert(p39_y_GET(pack) == (float)2.7043695E37F);
    assert(p39_param4_GET(pack) == (float) -1.4588676E38F);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p39_z_GET(pack) == (float) -2.8204723E38F);
    assert(p39_x_GET(pack) == (float) -2.176817E38F);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p39_param2_GET(pack) == (float) -2.717986E38F);
    assert(p39_param3_GET(pack) == (float) -3.1855453E38F);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)80);
    assert(p39_param1_GET(pack) == (float) -6.999244E37F);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)46645);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)24011);
};


void c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)56896);
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)166);
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)250);
};


void c_LoopBackDemoChannel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)27028);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)111);
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)68);
};


void c_LoopBackDemoChannel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)17202);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
};


void c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)2);
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)179);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)10861);
};


void c_LoopBackDemoChannel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)199);
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_SEQUENCE);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
};


void c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_latitude_GET(pack) == (int32_t) -205468366);
    assert(p48_altitude_GET(pack) == (int32_t)1216808069);
    assert(p48_time_usec_TRY(ph) == (uint64_t)3850265073922235744L);
    assert(p48_longitude_GET(pack) == (int32_t) -1060847626);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)237);
};


void c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_longitude_GET(pack) == (int32_t)1354325720);
    assert(p49_altitude_GET(pack) == (int32_t) -983333754);
    assert(p49_latitude_GET(pack) == (int32_t)1686598451);
    assert(p49_time_usec_TRY(ph) == (uint64_t)6713642608886351560L);
};


void c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)31);
    assert(p50_param_value0_GET(pack) == (float) -1.5687017E38F);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t) -2458);
    assert(p50_param_value_min_GET(pack) == (float) -7.10308E37F);
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)248);
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p50_scale_GET(pack) == (float)2.203298E38F);
    assert(p50_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"g";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_param_value_max_GET(pack) == (float)9.822185E37F);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)4339);
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)231);
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)167);
};


void c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_p1x_GET(pack) == (float)3.1360792E38F);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)61);
    assert(p54_p2x_GET(pack) == (float) -1.6307219E38F);
    assert(p54_p1z_GET(pack) == (float) -2.2999342E38F);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT);
    assert(p54_p2y_GET(pack) == (float) -1.0273129E38F);
    assert(p54_p2z_GET(pack) == (float)9.710677E37F);
    assert(p54_p1y_GET(pack) == (float) -1.8485999E38F);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)191);
};


void c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p1y_GET(pack) == (float) -2.1775747E38F);
    assert(p55_p2z_GET(pack) == (float) -3.0495473E38F);
    assert(p55_p2x_GET(pack) == (float) -3.7479743E37F);
    assert(p55_p1z_GET(pack) == (float) -3.0465901E38F);
    assert(p55_p2y_GET(pack) == (float)1.7371462E37F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT);
    assert(p55_p1x_GET(pack) == (float)9.332756E37F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    assert(p61_rollspeed_GET(pack) == (float)1.9005847E38F);
    {
        float exemplary[] =  {2.6722955E38F, -1.2552885E38F, 1.7940227E38F, -2.1209194E38F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_pitchspeed_GET(pack) == (float)1.7580716E38F);
    assert(p61_time_usec_GET(pack) == (uint64_t)8954629646871929114L);
    assert(p61_yawspeed_GET(pack) == (float)1.3639271E38F);
    {
        float exemplary[] =  {2.7418932E38F, -2.0236178E38F, 5.8848707E37F, 7.521846E37F, -1.5501494E38F, 2.4142602E38F, 2.4843589E38F, -6.225056E37F, -5.7039303E37F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_nav_roll_GET(pack) == (float) -9.387065E37F);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)10267);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t)8406);
    assert(p62_aspd_error_GET(pack) == (float)3.2367955E38F);
    assert(p62_alt_error_GET(pack) == (float)5.7469325E37F);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t) -19908);
    assert(p62_xtrack_error_GET(pack) == (float) -2.1998015E38F);
    assert(p62_nav_pitch_GET(pack) == (float)6.717252E36F);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_lon_GET(pack) == (int32_t) -504838712);
    assert(p63_vz_GET(pack) == (float) -2.3772614E37F);
    assert(p63_alt_GET(pack) == (int32_t) -1534862069);
    assert(p63_vx_GET(pack) == (float)6.183623E37F);
    assert(p63_vy_GET(pack) == (float)2.0715894E36F);
    assert(p63_lat_GET(pack) == (int32_t) -1623622009);
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO);
    {
        float exemplary[] =  {2.5606228E38F, -2.307961E38F, 5.4891345E37F, 2.1177996E38F, -4.8552407E37F, -1.0163799E38F, 1.3329797E38F, -5.208861E37F, -1.4886549E38F, 1.4519405E38F, -1.469174E37F, 6.0758676E37F, 7.3372403E37F, 2.8506357E38F, 3.2207477E38F, 1.4757735E38F, 1.4200969E38F, 1.3198769E38F, 1.4131736E38F, 8.857364E37F, 5.4403573E37F, -1.0235026E38F, -2.818973E38F, 8.996177E37F, 2.7526445E38F, 1.7888211E38F, 2.0106995E38F, -1.065261E37F, 2.5724417E38F, -1.992602E38F, -1.8715593E38F, -3.062535E38F, 3.3508153E37F, -3.0475572E38F, 2.5526165E38F, 2.8315765E38F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_relative_alt_GET(pack) == (int32_t)1080065771);
    assert(p63_time_usec_GET(pack) == (uint64_t)6729255855491189222L);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_ax_GET(pack) == (float) -3.349672E38F);
    assert(p64_z_GET(pack) == (float)1.6027555E38F);
    assert(p64_x_GET(pack) == (float) -1.8997894E38F);
    assert(p64_y_GET(pack) == (float) -2.0582683E37F);
    assert(p64_vx_GET(pack) == (float) -1.2809868E37F);
    {
        float exemplary[] =  {1.8606715E38F, -1.2011744E38F, 9.951099E37F, 3.0946533E38F, -2.4818682E38F, -2.6271178E38F, -1.5186705E38F, 2.8541664E38F, 3.5220503E37F, -8.99158E37F, -2.3841796E38F, 1.271449E38F, -1.5102609E38F, -2.1707734E38F, 2.5208891E38F, -2.480653E38F, 1.3881866E38F, 2.9458642E38F, -2.4458651E38F, -2.460553E38F, 5.0651586E37F, 3.3222751E38F, -1.635427E38F, 7.6055664E37F, 1.3094552E38F, 2.5690655E38F, -2.7075216E38F, -2.2721197E38F, -1.2924376E38F, 1.4971302E38F, -3.3300193E37F, -1.2358398E37F, 1.9181393E38F, 1.4504982E38F, -3.0959537E38F, 9.22801E37F, -2.9958997E38F, -2.6933494E38F, 9.316511E37F, 2.9019041E38F, 4.0905183E37F, 1.7252666E38F, 1.3998047E38F, -1.5735885E38F, 1.221211E37F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_vy_GET(pack) == (float)1.8128783E38F);
    assert(p64_time_usec_GET(pack) == (uint64_t)5283133235681849801L);
    assert(p64_vz_GET(pack) == (float)1.9953693E38F);
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS);
    assert(p64_ay_GET(pack) == (float)2.217106E38F);
    assert(p64_az_GET(pack) == (float)1.4947249E38F);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)6719);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)65129);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)1343);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)13134);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)231);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)58906);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)3764083324L);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)32635);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)12414);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)52838);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)10130);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)35353);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)32695);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)21304);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)27279);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)18988);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)46395);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)50663);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)3650);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)59);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)51730);
};


void c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)221);
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)40);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)63508);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)233);
};


void c_LoopBackDemoChannel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)140);
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)19747);
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)136);
};


void c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_x_GET(pack) == (int16_t)(int16_t)24478);
    assert(p69_y_GET(pack) == (int16_t)(int16_t)13683);
    assert(p69_r_GET(pack) == (int16_t)(int16_t) -8231);
    assert(p69_z_GET(pack) == (int16_t)(int16_t)2859);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)50);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)34956);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)8992);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)34422);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)57828);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)65027);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)24273);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)37);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)48914);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)56535);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)51127);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)13);
    assert(p73_param2_GET(pack) == (float)1.3783983E38F);
    assert(p73_z_GET(pack) == (float)3.3374273E38F);
    assert(p73_param1_GET(pack) == (float) -3.2703964E38F);
    assert(p73_y_GET(pack) == (int32_t)767398716);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)106);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)9251);
    assert(p73_x_GET(pack) == (int32_t)1588369050);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)227);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)165);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS);
    assert(p73_param4_GET(pack) == (float)2.8016328E38F);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p73_param3_GET(pack) == (float)2.1609066E38F);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
};


void c_LoopBackDemoChannel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_groundspeed_GET(pack) == (float)2.5356395E38F);
    assert(p74_climb_GET(pack) == (float) -1.4950729E38F);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)31910);
    assert(p74_airspeed_GET(pack) == (float) -1.5998549E38F);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t)1209);
    assert(p74_alt_GET(pack) == (float)3.2069477E38F);
};


void c_LoopBackDemoChannel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_param1_GET(pack) == (float) -2.9364562E38F);
    assert(p75_y_GET(pack) == (int32_t)87216988);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)31);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)167);
    assert(p75_z_GET(pack) == (float)2.7057347E38F);
    assert(p75_param2_GET(pack) == (float) -1.0598652E38F);
    assert(p75_x_GET(pack) == (int32_t)1216159528);
    assert(p75_param4_GET(pack) == (float)2.195852E38F);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_SPATIAL_USER_5);
    assert(p75_param3_GET(pack) == (float)3.3703351E38F);
};


void c_LoopBackDemoChannel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_param3_GET(pack) == (float)2.861883E38F);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE);
    assert(p76_param6_GET(pack) == (float) -2.8771945E38F);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p76_param4_GET(pack) == (float)2.9928464E38F);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)31);
    assert(p76_param2_GET(pack) == (float) -3.1319247E38F);
    assert(p76_param5_GET(pack) == (float) -1.9226787E38F);
    assert(p76_param7_GET(pack) == (float) -3.3321677E38F);
    assert(p76_param1_GET(pack) == (float)1.4418545E38F);
};


void c_LoopBackDemoChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)150);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_IN_PROGRESS);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)62);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)109);
    assert(p77_result_param2_TRY(ph) == (int32_t)1545741484);
};


void c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_roll_GET(pack) == (float) -2.7895122E38F);
    assert(p81_yaw_GET(pack) == (float)4.0076343E36F);
    assert(p81_pitch_GET(pack) == (float)3.1803447E38F);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)3750877057L);
    assert(p81_thrust_GET(pack) == (float) -1.830613E37F);
};


void c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_thrust_GET(pack) == (float) -2.9186544E38F);
    assert(p82_body_pitch_rate_GET(pack) == (float) -3.0549619E38F);
    {
        float exemplary[] =  {-1.4671242E38F, -8.58204E37F, 1.7484778E38F, -1.413136E37F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_body_yaw_rate_GET(pack) == (float)1.9718532E38F);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)199);
    assert(p82_body_roll_rate_GET(pack) == (float)3.3626189E38F);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)3883848164L);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)132);
};


void c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_body_yaw_rate_GET(pack) == (float) -1.9527042E38F);
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)70);
    assert(p83_thrust_GET(pack) == (float) -6.427242E37F);
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)641785841L);
    assert(p83_body_pitch_rate_GET(pack) == (float) -2.3883612E38F);
    assert(p83_body_roll_rate_GET(pack) == (float)2.5926627E38F);
    {
        float exemplary[] =  {-9.643587E37F, 3.1130878E38F, 8.923623E37F, 3.3298433E38F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_vz_GET(pack) == (float) -1.1845589E38F);
    assert(p84_yaw_GET(pack) == (float)2.109506E38F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)1416464694L);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)177);
    assert(p84_yaw_rate_GET(pack) == (float) -2.7849231E38F);
    assert(p84_afy_GET(pack) == (float)2.3537326E37F);
    assert(p84_vx_GET(pack) == (float) -1.9269764E38F);
    assert(p84_x_GET(pack) == (float) -1.1328118E38F);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)215);
    assert(p84_z_GET(pack) == (float) -2.0589157E38F);
    assert(p84_afx_GET(pack) == (float) -2.9063679E38F);
    assert(p84_vy_GET(pack) == (float) -1.5731545E38F);
    assert(p84_y_GET(pack) == (float) -1.086098E38F);
    assert(p84_afz_GET(pack) == (float)5.043134E37F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)53737);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_vz_GET(pack) == (float)1.973987E38F);
    assert(p86_afz_GET(pack) == (float)1.877922E38F);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)38540);
    assert(p86_afx_GET(pack) == (float) -5.218414E37F);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)1731633069L);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p86_lon_int_GET(pack) == (int32_t)1229895209);
    assert(p86_lat_int_GET(pack) == (int32_t)1829564625);
    assert(p86_afy_GET(pack) == (float)1.474745E38F);
    assert(p86_alt_GET(pack) == (float) -3.3001401E38F);
    assert(p86_vy_GET(pack) == (float) -1.9492552E38F);
    assert(p86_yaw_rate_GET(pack) == (float) -9.669672E37F);
    assert(p86_yaw_GET(pack) == (float)7.8878717E37F);
    assert(p86_vx_GET(pack) == (float) -1.8779907E38F);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)192);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_yaw_rate_GET(pack) == (float)2.1143937E37F);
    assert(p87_alt_GET(pack) == (float) -1.1121075E38F);
    assert(p87_vy_GET(pack) == (float) -7.36924E36F);
    assert(p87_afy_GET(pack) == (float)1.5987702E38F);
    assert(p87_vx_GET(pack) == (float) -1.3133995E38F);
    assert(p87_afx_GET(pack) == (float) -2.4330955E38F);
    assert(p87_afz_GET(pack) == (float) -3.5804155E37F);
    assert(p87_yaw_GET(pack) == (float) -2.3570772E38F);
    assert(p87_lon_int_GET(pack) == (int32_t)972280192);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)2764924049L);
    assert(p87_vz_GET(pack) == (float) -3.122269E38F);
    assert(p87_lat_int_GET(pack) == (int32_t) -2133855171);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)59220);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_yaw_GET(pack) == (float)8.186212E37F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)3449120020L);
    assert(p89_x_GET(pack) == (float) -8.914448E37F);
    assert(p89_z_GET(pack) == (float)2.635742E38F);
    assert(p89_y_GET(pack) == (float)2.4002461E38F);
    assert(p89_roll_GET(pack) == (float)2.9606661E38F);
    assert(p89_pitch_GET(pack) == (float)4.8518145E37F);
};


void c_LoopBackDemoChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_vx_GET(pack) == (int16_t)(int16_t) -15297);
    assert(p90_time_usec_GET(pack) == (uint64_t)2536275186215324673L);
    assert(p90_yawspeed_GET(pack) == (float) -7.8936157E37F);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t)11631);
    assert(p90_yaw_GET(pack) == (float) -3.2473172E38F);
    assert(p90_lat_GET(pack) == (int32_t) -613426522);
    assert(p90_lon_GET(pack) == (int32_t) -1980839820);
    assert(p90_pitchspeed_GET(pack) == (float)8.824346E37F);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t)19271);
    assert(p90_pitch_GET(pack) == (float)8.681347E37F);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t)18640);
    assert(p90_rollspeed_GET(pack) == (float) -3.2657432E38F);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t) -6912);
    assert(p90_alt_GET(pack) == (int32_t) -1532746219);
    assert(p90_roll_GET(pack) == (float)3.1443818E38F);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t) -23917);
};


void c_LoopBackDemoChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_aux1_GET(pack) == (float) -5.359019E37F);
    assert(p91_aux2_GET(pack) == (float)1.9419781E36F);
    assert(p91_throttle_GET(pack) == (float)3.5207347E37F);
    assert(p91_pitch_elevator_GET(pack) == (float) -2.4176093E38F);
    assert(p91_time_usec_GET(pack) == (uint64_t)7408040310355973957L);
    assert(p91_yaw_rudder_GET(pack) == (float)1.2177409E38F);
    assert(p91_aux4_GET(pack) == (float)3.2120467E38F);
    assert(p91_roll_ailerons_GET(pack) == (float)1.7811035E38F);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)79);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_MANUAL_DISARMED);
    assert(p91_aux3_GET(pack) == (float)1.3995551E38F);
};


void c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)28);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)4312);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)27352);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)17334);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)51880);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)54775);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)52166);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)244);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)13738);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)23339);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)4044);
    assert(p92_time_usec_GET(pack) == (uint64_t)3915794727397469184L);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)32853);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)47816);
};


void c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_TEST_ARMED);
    assert(p93_time_usec_GET(pack) == (uint64_t)3610378822110451169L);
    {
        float exemplary[] =  {-1.2365345E37F, 3.194105E38F, -2.491589E38F, 1.3174247E38F, 2.0227769E38F, -2.1508094E38F, 5.98522E37F, -2.627403E38F, -1.5292548E37F, 3.3163926E38F, -8.861006E37F, 2.6985736E38F, 1.1782844E38F, -2.6362545E38F, 2.9915023E38F, -1.1797941E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_flags_GET(pack) == (uint64_t)3345631702855313753L);
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_flow_rate_y_TRY(ph) == (float)9.212116E36F);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t)30938);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)243);
    assert(p100_ground_distance_GET(pack) == (float)6.510126E37F);
    assert(p100_time_usec_GET(pack) == (uint64_t)2492431362765764245L);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p100_flow_comp_m_y_GET(pack) == (float) -2.4726239E38F);
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t)2545);
    assert(p100_flow_rate_x_TRY(ph) == (float)2.8145007E37F);
    assert(p100_flow_comp_m_x_GET(pack) == (float)1.2318748E38F);
};


void c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_pitch_GET(pack) == (float) -9.180316E34F);
    assert(p101_y_GET(pack) == (float) -2.0894648E38F);
    assert(p101_roll_GET(pack) == (float)2.4883297E37F);
    assert(p101_x_GET(pack) == (float)3.2478097E38F);
    assert(p101_usec_GET(pack) == (uint64_t)346127162980979691L);
    assert(p101_yaw_GET(pack) == (float)7.589145E37F);
    assert(p101_z_GET(pack) == (float)2.3854702E38F);
};


void c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_y_GET(pack) == (float) -2.5966035E36F);
    assert(p102_roll_GET(pack) == (float) -5.49813E37F);
    assert(p102_usec_GET(pack) == (uint64_t)6982712676303463025L);
    assert(p102_x_GET(pack) == (float)1.1648626E38F);
    assert(p102_pitch_GET(pack) == (float)3.0887832E38F);
    assert(p102_yaw_GET(pack) == (float) -3.1029334E38F);
    assert(p102_z_GET(pack) == (float)1.2230557E38F);
};


void c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_x_GET(pack) == (float)1.1976817E37F);
    assert(p103_z_GET(pack) == (float)4.0948854E37F);
    assert(p103_y_GET(pack) == (float)1.1641661E38F);
    assert(p103_usec_GET(pack) == (uint64_t)642658115926641724L);
};


void c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_yaw_GET(pack) == (float)2.3019705E38F);
    assert(p104_pitch_GET(pack) == (float)2.4871863E38F);
    assert(p104_x_GET(pack) == (float)3.037076E38F);
    assert(p104_roll_GET(pack) == (float)1.955726E38F);
    assert(p104_z_GET(pack) == (float) -1.5407191E38F);
    assert(p104_usec_GET(pack) == (uint64_t)2848649985460158797L);
    assert(p104_y_GET(pack) == (float)1.6648662E38F);
};


void c_LoopBackDemoChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_yacc_GET(pack) == (float) -1.3589238E38F);
    assert(p105_ygyro_GET(pack) == (float)1.5737628E38F);
    assert(p105_ymag_GET(pack) == (float)2.3814648E38F);
    assert(p105_xgyro_GET(pack) == (float)2.1181823E38F);
    assert(p105_temperature_GET(pack) == (float)6.1761656E37F);
    assert(p105_xacc_GET(pack) == (float)2.7754194E38F);
    assert(p105_zacc_GET(pack) == (float)1.4714333E38F);
    assert(p105_diff_pressure_GET(pack) == (float)1.938417E38F);
    assert(p105_time_usec_GET(pack) == (uint64_t)8035732459994930555L);
    assert(p105_zgyro_GET(pack) == (float) -1.1962299E38F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)52405);
    assert(p105_xmag_GET(pack) == (float) -4.3688057E37F);
    assert(p105_abs_pressure_GET(pack) == (float)9.383402E37F);
    assert(p105_zmag_GET(pack) == (float) -1.0142596E38F);
    assert(p105_pressure_alt_GET(pack) == (float) -1.763626E38F);
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_integrated_x_GET(pack) == (float) -1.7889487E37F);
    assert(p106_integrated_xgyro_GET(pack) == (float)1.6676042E38F);
    assert(p106_distance_GET(pack) == (float) -1.5792013E38F);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)891935782L);
    assert(p106_integrated_ygyro_GET(pack) == (float)2.4168254E38F);
    assert(p106_integrated_y_GET(pack) == (float)2.8276845E38F);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t)16115);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)1080346057L);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)215);
    assert(p106_integrated_zgyro_GET(pack) == (float)5.222451E37F);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)248);
    assert(p106_time_usec_GET(pack) == (uint64_t)6274787953676475057L);
};


void c_LoopBackDemoChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_zmag_GET(pack) == (float)2.5152946E38F);
    assert(p107_zgyro_GET(pack) == (float) -2.4262745E38F);
    assert(p107_xgyro_GET(pack) == (float)1.3213113E38F);
    assert(p107_ygyro_GET(pack) == (float)4.0846103E37F);
    assert(p107_pressure_alt_GET(pack) == (float)5.9387255E37F);
    assert(p107_temperature_GET(pack) == (float) -1.2685911E38F);
    assert(p107_ymag_GET(pack) == (float)6.05489E37F);
    assert(p107_abs_pressure_GET(pack) == (float)1.830222E38F);
    assert(p107_time_usec_GET(pack) == (uint64_t)5766368874335532026L);
    assert(p107_diff_pressure_GET(pack) == (float)3.1826766E38F);
    assert(p107_xmag_GET(pack) == (float) -2.519297E38F);
    assert(p107_xacc_GET(pack) == (float)3.2411813E37F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)4017465888L);
    assert(p107_zacc_GET(pack) == (float) -3.079331E38F);
    assert(p107_yacc_GET(pack) == (float)3.2596494E37F);
};


void c_LoopBackDemoChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_vd_GET(pack) == (float)2.8343235E38F);
    assert(p108_xacc_GET(pack) == (float) -1.3477078E38F);
    assert(p108_roll_GET(pack) == (float) -3.1138229E38F);
    assert(p108_std_dev_vert_GET(pack) == (float)5.989723E36F);
    assert(p108_yacc_GET(pack) == (float)1.846219E38F);
    assert(p108_std_dev_horz_GET(pack) == (float) -3.1721109E38F);
    assert(p108_alt_GET(pack) == (float) -8.4081474E37F);
    assert(p108_zacc_GET(pack) == (float)7.2927503E37F);
    assert(p108_q1_GET(pack) == (float) -1.6243857E38F);
    assert(p108_zgyro_GET(pack) == (float) -6.008581E37F);
    assert(p108_q3_GET(pack) == (float) -3.7699934E37F);
    assert(p108_q4_GET(pack) == (float)2.7459766E37F);
    assert(p108_q2_GET(pack) == (float) -1.1490626E38F);
    assert(p108_ve_GET(pack) == (float)2.4364466E38F);
    assert(p108_lon_GET(pack) == (float) -3.2346636E38F);
    assert(p108_vn_GET(pack) == (float) -3.3265365E38F);
    assert(p108_pitch_GET(pack) == (float) -2.8306762E38F);
    assert(p108_yaw_GET(pack) == (float)2.4804916E38F);
    assert(p108_lat_GET(pack) == (float) -6.3704747E37F);
    assert(p108_ygyro_GET(pack) == (float)2.3939776E38F);
    assert(p108_xgyro_GET(pack) == (float)1.7057622E38F);
};


void c_LoopBackDemoChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)32022);
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)133);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)157);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)50);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)235);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)52);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)48456);
};


void c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)195, (uint8_t)132, (uint8_t)148, (uint8_t)28, (uint8_t)250, (uint8_t)89, (uint8_t)130, (uint8_t)101, (uint8_t)153, (uint8_t)184, (uint8_t)71, (uint8_t)138, (uint8_t)140, (uint8_t)130, (uint8_t)126, (uint8_t)118, (uint8_t)80, (uint8_t)41, (uint8_t)201, (uint8_t)1, (uint8_t)15, (uint8_t)213, (uint8_t)72, (uint8_t)147, (uint8_t)155, (uint8_t)226, (uint8_t)65, (uint8_t)30, (uint8_t)168, (uint8_t)205, (uint8_t)170, (uint8_t)58, (uint8_t)122, (uint8_t)102, (uint8_t)67, (uint8_t)64, (uint8_t)6, (uint8_t)61, (uint8_t)80, (uint8_t)139, (uint8_t)106, (uint8_t)98, (uint8_t)161, (uint8_t)152, (uint8_t)106, (uint8_t)99, (uint8_t)153, (uint8_t)68, (uint8_t)76, (uint8_t)78, (uint8_t)206, (uint8_t)2, (uint8_t)105, (uint8_t)3, (uint8_t)85, (uint8_t)62, (uint8_t)220, (uint8_t)251, (uint8_t)127, (uint8_t)205, (uint8_t)165, (uint8_t)99, (uint8_t)98, (uint8_t)19, (uint8_t)156, (uint8_t)161, (uint8_t)150, (uint8_t)34, (uint8_t)200, (uint8_t)146, (uint8_t)112, (uint8_t)104, (uint8_t)15, (uint8_t)77, (uint8_t)46, (uint8_t)81, (uint8_t)218, (uint8_t)125, (uint8_t)128, (uint8_t)20, (uint8_t)222, (uint8_t)237, (uint8_t)158, (uint8_t)12, (uint8_t)245, (uint8_t)158, (uint8_t)238, (uint8_t)51, (uint8_t)23, (uint8_t)193, (uint8_t)249, (uint8_t)186, (uint8_t)62, (uint8_t)74, (uint8_t)62, (uint8_t)128, (uint8_t)34, (uint8_t)225, (uint8_t)0, (uint8_t)79, (uint8_t)10, (uint8_t)179, (uint8_t)200, (uint8_t)131, (uint8_t)176, (uint8_t)125, (uint8_t)246, (uint8_t)80, (uint8_t)116, (uint8_t)48, (uint8_t)120, (uint8_t)93, (uint8_t)41, (uint8_t)15, (uint8_t)238, (uint8_t)201, (uint8_t)110, (uint8_t)43, (uint8_t)89, (uint8_t)232, (uint8_t)48, (uint8_t)153, (uint8_t)124, (uint8_t)155, (uint8_t)137, (uint8_t)78, (uint8_t)231, (uint8_t)44, (uint8_t)229, (uint8_t)163, (uint8_t)122, (uint8_t)243, (uint8_t)216, (uint8_t)219, (uint8_t)191, (uint8_t)99, (uint8_t)90, (uint8_t)90, (uint8_t)47, (uint8_t)145, (uint8_t)110, (uint8_t)118, (uint8_t)47, (uint8_t)19, (uint8_t)242, (uint8_t)57, (uint8_t)3, (uint8_t)5, (uint8_t)230, (uint8_t)119, (uint8_t)91, (uint8_t)45, (uint8_t)207, (uint8_t)47, (uint8_t)27, (uint8_t)200, (uint8_t)208, (uint8_t)43, (uint8_t)26, (uint8_t)229, (uint8_t)148, (uint8_t)173, (uint8_t)96, (uint8_t)113, (uint8_t)27, (uint8_t)83, (uint8_t)226, (uint8_t)10, (uint8_t)72, (uint8_t)230, (uint8_t)201, (uint8_t)72, (uint8_t)241, (uint8_t)202, (uint8_t)239, (uint8_t)57, (uint8_t)166, (uint8_t)76, (uint8_t)112, (uint8_t)186, (uint8_t)42, (uint8_t)197, (uint8_t)62, (uint8_t)12, (uint8_t)111, (uint8_t)3, (uint8_t)133, (uint8_t)69, (uint8_t)65, (uint8_t)164, (uint8_t)104, (uint8_t)20, (uint8_t)202, (uint8_t)188, (uint8_t)161, (uint8_t)206, (uint8_t)76, (uint8_t)171, (uint8_t)149, (uint8_t)2, (uint8_t)39, (uint8_t)214, (uint8_t)36, (uint8_t)173, (uint8_t)187, (uint8_t)232, (uint8_t)125, (uint8_t)19, (uint8_t)161, (uint8_t)180, (uint8_t)174, (uint8_t)30, (uint8_t)213, (uint8_t)167, (uint8_t)151, (uint8_t)148, (uint8_t)184, (uint8_t)188, (uint8_t)141, (uint8_t)160, (uint8_t)77, (uint8_t)149, (uint8_t)169, (uint8_t)20, (uint8_t)205, (uint8_t)82, (uint8_t)168, (uint8_t)207, (uint8_t)139, (uint8_t)249, (uint8_t)22, (uint8_t)21, (uint8_t)145, (uint8_t)78, (uint8_t)16, (uint8_t)87, (uint8_t)64, (uint8_t)80, (uint8_t)252, (uint8_t)111, (uint8_t)214, (uint8_t)194, (uint8_t)44, (uint8_t)161, (uint8_t)118, (uint8_t)7, (uint8_t)33, (uint8_t)46, (uint8_t)7, (uint8_t)84, (uint8_t)73} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)210);
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)49);
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)181);
};


void c_LoopBackDemoChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_tc1_GET(pack) == (int64_t)8489164696769321262L);
    assert(p111_ts1_GET(pack) == (int64_t)3110380596920743801L);
};


void c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_time_usec_GET(pack) == (uint64_t)4428343964171788555L);
    assert(p112_seq_GET(pack) == (uint32_t)2418821322L);
};


void c_LoopBackDemoChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_vn_GET(pack) == (int16_t)(int16_t) -29225);
    assert(p113_lat_GET(pack) == (int32_t) -804956579);
    assert(p113_time_usec_GET(pack) == (uint64_t)6268276013888803942L);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)39421);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t) -17259);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)25131);
    assert(p113_alt_GET(pack) == (int32_t) -1775594596);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)39585);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)150);
    assert(p113_lon_GET(pack) == (int32_t)888165502);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t)2173);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)52344);
};


void c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_integration_time_us_GET(pack) == (uint32_t)4185809387L);
    assert(p114_integrated_xgyro_GET(pack) == (float)2.2546302E38F);
    assert(p114_time_usec_GET(pack) == (uint64_t)600192806215029821L);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)18);
    assert(p114_integrated_ygyro_GET(pack) == (float)1.5032397E38F);
    assert(p114_integrated_zgyro_GET(pack) == (float) -9.685447E37F);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)3846312628L);
    assert(p114_integrated_y_GET(pack) == (float) -3.0095987E38F);
    assert(p114_distance_GET(pack) == (float) -1.8865199E38F);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t) -27837);
    assert(p114_integrated_x_GET(pack) == (float)1.933978E38F);
};


void c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_vy_GET(pack) == (int16_t)(int16_t)4665);
    assert(p115_time_usec_GET(pack) == (uint64_t)3018348233812227960L);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)41574);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t)22823);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t) -15495);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)55959);
    assert(p115_yawspeed_GET(pack) == (float) -3.9748234E37F);
    assert(p115_alt_GET(pack) == (int32_t) -1265732710);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t) -3060);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t) -699);
    assert(p115_pitchspeed_GET(pack) == (float)1.7434455E38F);
    {
        float exemplary[] =  {-8.116909E37F, -2.0489463E37F, 3.3970558E38F, 8.841517E36F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_lon_GET(pack) == (int32_t)875146244);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t) -1297);
    assert(p115_rollspeed_GET(pack) == (float)2.9922363E38F);
    assert(p115_lat_GET(pack) == (int32_t)502577731);
};


void c_LoopBackDemoChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t) -10307);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)2282362795L);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t)29886);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t) -9614);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t) -17275);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t) -8097);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t) -22006);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t) -11758);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t)27190);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t)5913);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)21467);
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)39470);
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)82);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)210);
};


void c_LoopBackDemoChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)40149);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)36401);
    assert(p118_time_utc_GET(pack) == (uint32_t)1561019446L);
    assert(p118_size_GET(pack) == (uint32_t)590600547L);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)33501);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)43);
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)10);
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)21497);
    assert(p119_ofs_GET(pack) == (uint32_t)880909687L);
    assert(p119_count_GET(pack) == (uint32_t)3806372522L);
};


void c_LoopBackDemoChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_ofs_GET(pack) == (uint32_t)2687092424L);
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)44);
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)63599);
    {
        uint8_t exemplary[] =  {(uint8_t)118, (uint8_t)210, (uint8_t)172, (uint8_t)248, (uint8_t)147, (uint8_t)137, (uint8_t)85, (uint8_t)29, (uint8_t)98, (uint8_t)65, (uint8_t)39, (uint8_t)51, (uint8_t)65, (uint8_t)71, (uint8_t)129, (uint8_t)20, (uint8_t)142, (uint8_t)98, (uint8_t)185, (uint8_t)52, (uint8_t)204, (uint8_t)131, (uint8_t)191, (uint8_t)93, (uint8_t)195, (uint8_t)115, (uint8_t)79, (uint8_t)24, (uint8_t)248, (uint8_t)112, (uint8_t)125, (uint8_t)136, (uint8_t)96, (uint8_t)49, (uint8_t)17, (uint8_t)172, (uint8_t)227, (uint8_t)64, (uint8_t)233, (uint8_t)96, (uint8_t)176, (uint8_t)244, (uint8_t)173, (uint8_t)88, (uint8_t)170, (uint8_t)72, (uint8_t)110, (uint8_t)29, (uint8_t)72, (uint8_t)251, (uint8_t)244, (uint8_t)211, (uint8_t)47, (uint8_t)228, (uint8_t)163, (uint8_t)252, (uint8_t)77, (uint8_t)68, (uint8_t)169, (uint8_t)253, (uint8_t)196, (uint8_t)57, (uint8_t)46, (uint8_t)82, (uint8_t)246, (uint8_t)136, (uint8_t)87, (uint8_t)210, (uint8_t)34, (uint8_t)214, (uint8_t)12, (uint8_t)163, (uint8_t)192, (uint8_t)148, (uint8_t)11, (uint8_t)158, (uint8_t)51, (uint8_t)137, (uint8_t)68, (uint8_t)237, (uint8_t)225, (uint8_t)183, (uint8_t)24, (uint8_t)37, (uint8_t)237, (uint8_t)58, (uint8_t)176, (uint8_t)106, (uint8_t)57, (uint8_t)88} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)90);
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)75);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)61);
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)14);
};


void c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)208, (uint8_t)166, (uint8_t)182, (uint8_t)200, (uint8_t)209, (uint8_t)209, (uint8_t)245, (uint8_t)164, (uint8_t)234, (uint8_t)60, (uint8_t)76, (uint8_t)131, (uint8_t)71, (uint8_t)226, (uint8_t)23, (uint8_t)117, (uint8_t)29, (uint8_t)4, (uint8_t)173, (uint8_t)152, (uint8_t)42, (uint8_t)246, (uint8_t)158, (uint8_t)248, (uint8_t)141, (uint8_t)225, (uint8_t)172, (uint8_t)159, (uint8_t)75, (uint8_t)39, (uint8_t)8, (uint8_t)203, (uint8_t)201, (uint8_t)105, (uint8_t)41, (uint8_t)86, (uint8_t)162, (uint8_t)157, (uint8_t)212, (uint8_t)123, (uint8_t)237, (uint8_t)192, (uint8_t)94, (uint8_t)147, (uint8_t)139, (uint8_t)53, (uint8_t)74, (uint8_t)204, (uint8_t)193, (uint8_t)228, (uint8_t)238, (uint8_t)207, (uint8_t)22, (uint8_t)153, (uint8_t)215, (uint8_t)77, (uint8_t)102, (uint8_t)191, (uint8_t)80, (uint8_t)51, (uint8_t)84, (uint8_t)140, (uint8_t)17, (uint8_t)250, (uint8_t)242, (uint8_t)185, (uint8_t)247, (uint8_t)188, (uint8_t)93, (uint8_t)33, (uint8_t)69, (uint8_t)96, (uint8_t)16, (uint8_t)169, (uint8_t)122, (uint8_t)98, (uint8_t)160, (uint8_t)42, (uint8_t)171, (uint8_t)84, (uint8_t)225, (uint8_t)37, (uint8_t)129, (uint8_t)218, (uint8_t)179, (uint8_t)50, (uint8_t)73, (uint8_t)129, (uint8_t)120, (uint8_t)125, (uint8_t)62, (uint8_t)243, (uint8_t)198, (uint8_t)79, (uint8_t)124, (uint8_t)198, (uint8_t)123, (uint8_t)143, (uint8_t)73, (uint8_t)184, (uint8_t)237, (uint8_t)88, (uint8_t)192, (uint8_t)245, (uint8_t)201, (uint8_t)146, (uint8_t)127, (uint8_t)112, (uint8_t)7, (uint8_t)87} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)248);
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)211);
};


void c_LoopBackDemoChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_alt_GET(pack) == (int32_t) -612669741);
    assert(p124_lat_GET(pack) == (int32_t) -1764239476);
    assert(p124_time_usec_GET(pack) == (uint64_t)6704756267007216384L);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)37576);
    assert(p124_dgps_age_GET(pack) == (uint32_t)1235000715L);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p124_lon_GET(pack) == (int32_t) -64322712);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)30532);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)132);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)3343);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)63598);
};


void c_LoopBackDemoChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)7725);
    assert(p125_flags_GET(pack) == e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED);
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)21552);
};


void c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)93, (uint8_t)136, (uint8_t)117, (uint8_t)131, (uint8_t)38, (uint8_t)30, (uint8_t)121, (uint8_t)30, (uint8_t)129, (uint8_t)143, (uint8_t)163, (uint8_t)157, (uint8_t)147, (uint8_t)54, (uint8_t)130, (uint8_t)209, (uint8_t)205, (uint8_t)26, (uint8_t)15, (uint8_t)161, (uint8_t)181, (uint8_t)251, (uint8_t)141, (uint8_t)147, (uint8_t)110, (uint8_t)232, (uint8_t)44, (uint8_t)188, (uint8_t)69, (uint8_t)238, (uint8_t)166, (uint8_t)220, (uint8_t)107, (uint8_t)220, (uint8_t)92, (uint8_t)204, (uint8_t)121, (uint8_t)79, (uint8_t)155, (uint8_t)162, (uint8_t)31, (uint8_t)113, (uint8_t)36, (uint8_t)179, (uint8_t)185, (uint8_t)245, (uint8_t)138, (uint8_t)159, (uint8_t)114, (uint8_t)20, (uint8_t)163, (uint8_t)60, (uint8_t)38, (uint8_t)124, (uint8_t)68, (uint8_t)95, (uint8_t)183, (uint8_t)145, (uint8_t)203, (uint8_t)195, (uint8_t)174, (uint8_t)47, (uint8_t)107, (uint8_t)52, (uint8_t)235, (uint8_t)204, (uint8_t)55, (uint8_t)2, (uint8_t)37, (uint8_t)88} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1);
    assert(p126_flags_GET(pack) == e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE);
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)182);
    assert(p126_baudrate_GET(pack) == (uint32_t)1141564324L);
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)724);
};


void c_LoopBackDemoChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_tow_GET(pack) == (uint32_t)4147896568L);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)164);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)15337);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t) -766641944);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)255);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t) -1640939192);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)1816118331L);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t)49578127);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t) -1233024591);
    assert(p127_accuracy_GET(pack) == (uint32_t)669787245L);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)177);
};


void c_LoopBackDemoChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)16);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t) -911973171);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t) -761973663);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)51296);
    assert(p128_accuracy_GET(pack) == (uint32_t)1179054331L);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t) -1470565959);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)171);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)39656706L);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t) -61766415);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)201);
    assert(p128_tow_GET(pack) == (uint32_t)4120914998L);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)96);
};


void c_LoopBackDemoChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t)11713);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t)25704);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t)12139);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t)19009);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t)18382);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t)16314);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t) -4747);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t)23639);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)1391269655L);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t) -14883);
};


void c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)12654);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p130_size_GET(pack) == (uint32_t)432989241L);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)44425);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)2022);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)126);
};


void c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)169, (uint8_t)200, (uint8_t)128, (uint8_t)87, (uint8_t)13, (uint8_t)127, (uint8_t)157, (uint8_t)7, (uint8_t)106, (uint8_t)66, (uint8_t)87, (uint8_t)90, (uint8_t)119, (uint8_t)32, (uint8_t)174, (uint8_t)37, (uint8_t)215, (uint8_t)69, (uint8_t)193, (uint8_t)21, (uint8_t)11, (uint8_t)63, (uint8_t)99, (uint8_t)41, (uint8_t)53, (uint8_t)81, (uint8_t)251, (uint8_t)253, (uint8_t)14, (uint8_t)253, (uint8_t)18, (uint8_t)195, (uint8_t)216, (uint8_t)165, (uint8_t)50, (uint8_t)141, (uint8_t)225, (uint8_t)64, (uint8_t)36, (uint8_t)64, (uint8_t)77, (uint8_t)130, (uint8_t)188, (uint8_t)134, (uint8_t)97, (uint8_t)174, (uint8_t)127, (uint8_t)90, (uint8_t)122, (uint8_t)76, (uint8_t)51, (uint8_t)90, (uint8_t)90, (uint8_t)242, (uint8_t)58, (uint8_t)107, (uint8_t)234, (uint8_t)74, (uint8_t)50, (uint8_t)225, (uint8_t)183, (uint8_t)77, (uint8_t)59, (uint8_t)160, (uint8_t)54, (uint8_t)148, (uint8_t)80, (uint8_t)169, (uint8_t)50, (uint8_t)153, (uint8_t)58, (uint8_t)78, (uint8_t)54, (uint8_t)125, (uint8_t)193, (uint8_t)146, (uint8_t)140, (uint8_t)19, (uint8_t)247, (uint8_t)249, (uint8_t)208, (uint8_t)123, (uint8_t)232, (uint8_t)29, (uint8_t)203, (uint8_t)188, (uint8_t)239, (uint8_t)59, (uint8_t)70, (uint8_t)59, (uint8_t)90, (uint8_t)203, (uint8_t)198, (uint8_t)233, (uint8_t)185, (uint8_t)169, (uint8_t)76, (uint8_t)209, (uint8_t)202, (uint8_t)39, (uint8_t)156, (uint8_t)222, (uint8_t)93, (uint8_t)208, (uint8_t)57, (uint8_t)78, (uint8_t)174, (uint8_t)36, (uint8_t)170, (uint8_t)214, (uint8_t)251, (uint8_t)155, (uint8_t)72, (uint8_t)62, (uint8_t)32, (uint8_t)248, (uint8_t)146, (uint8_t)133, (uint8_t)182, (uint8_t)37, (uint8_t)106, (uint8_t)173, (uint8_t)57, (uint8_t)221, (uint8_t)201, (uint8_t)204, (uint8_t)150, (uint8_t)6, (uint8_t)10, (uint8_t)137, (uint8_t)99, (uint8_t)250, (uint8_t)142, (uint8_t)156, (uint8_t)211, (uint8_t)210, (uint8_t)152, (uint8_t)143, (uint8_t)226, (uint8_t)56, (uint8_t)194, (uint8_t)2, (uint8_t)139, (uint8_t)235, (uint8_t)110, (uint8_t)215, (uint8_t)32, (uint8_t)211, (uint8_t)166, (uint8_t)161, (uint8_t)175, (uint8_t)10, (uint8_t)213, (uint8_t)129, (uint8_t)118, (uint8_t)231, (uint8_t)128, (uint8_t)170, (uint8_t)192, (uint8_t)244, (uint8_t)64, (uint8_t)148, (uint8_t)169, (uint8_t)31, (uint8_t)139, (uint8_t)146, (uint8_t)244, (uint8_t)50, (uint8_t)11, (uint8_t)222, (uint8_t)80, (uint8_t)32, (uint8_t)77, (uint8_t)69, (uint8_t)83, (uint8_t)247, (uint8_t)170, (uint8_t)166, (uint8_t)131, (uint8_t)22, (uint8_t)196, (uint8_t)17, (uint8_t)161, (uint8_t)191, (uint8_t)59, (uint8_t)241, (uint8_t)2, (uint8_t)199, (uint8_t)83, (uint8_t)44, (uint8_t)150, (uint8_t)31, (uint8_t)243, (uint8_t)71, (uint8_t)161, (uint8_t)10, (uint8_t)62, (uint8_t)181, (uint8_t)205, (uint8_t)176, (uint8_t)56, (uint8_t)218, (uint8_t)32, (uint8_t)17, (uint8_t)73, (uint8_t)126, (uint8_t)192, (uint8_t)196, (uint8_t)12, (uint8_t)200, (uint8_t)104, (uint8_t)194, (uint8_t)12, (uint8_t)84, (uint8_t)52, (uint8_t)40, (uint8_t)167, (uint8_t)26, (uint8_t)207, (uint8_t)200, (uint8_t)236, (uint8_t)128, (uint8_t)230, (uint8_t)12, (uint8_t)7, (uint8_t)250, (uint8_t)71, (uint8_t)2, (uint8_t)235, (uint8_t)203, (uint8_t)6, (uint8_t)69, (uint8_t)7, (uint8_t)75, (uint8_t)65, (uint8_t)189, (uint8_t)101, (uint8_t)164, (uint8_t)151, (uint8_t)164, (uint8_t)77, (uint8_t)168, (uint8_t)46, (uint8_t)246, (uint8_t)216, (uint8_t)10, (uint8_t)57, (uint8_t)121, (uint8_t)145, (uint8_t)118, (uint8_t)36, (uint8_t)28, (uint8_t)113} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)12511);
};


void c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)35806);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_NONE);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)189);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)3838);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)44083);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)753725895L);
};


void c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_mask_GET(pack) == (uint64_t)6261140212422089752L);
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)24495);
    assert(p133_lat_GET(pack) == (int32_t) -1764712385);
    assert(p133_lon_GET(pack) == (int32_t) -1825795948);
};


void c_LoopBackDemoChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    {
        int16_t exemplary[] =  {(int16_t)27833, (int16_t)22673, (int16_t)14677, (int16_t)19443, (int16_t)30669, (int16_t) -29870, (int16_t)27554, (int16_t)18946, (int16_t)11349, (int16_t)2864, (int16_t) -1610, (int16_t) -30786, (int16_t) -25067, (int16_t)8214, (int16_t) -15260, (int16_t) -6677} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)66);
    assert(p134_lat_GET(pack) == (int32_t)1590518892);
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)43813);
    assert(p134_lon_GET(pack) == (int32_t) -1760890676);
};


void c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lat_GET(pack) == (int32_t)140754526);
    assert(p135_lon_GET(pack) == (int32_t) -1113134223);
};


void c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)15156);
    assert(p136_lat_GET(pack) == (int32_t) -818639174);
    assert(p136_lon_GET(pack) == (int32_t) -640064180);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)41490);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)38374);
    assert(p136_terrain_height_GET(pack) == (float)3.2869115E38F);
    assert(p136_current_height_GET(pack) == (float) -1.5538309E38F);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_press_abs_GET(pack) == (float)2.5038456E38F);
    assert(p137_press_diff_GET(pack) == (float) -1.5110764E37F);
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)1533155839L);
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t) -30783);
};


void c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    assert(p138_x_GET(pack) == (float)3.3864838E38F);
    assert(p138_time_usec_GET(pack) == (uint64_t)6130085898167468531L);
    {
        float exemplary[] =  {3.090613E38F, -2.5215609E38F, 1.9434485E38F, 2.857603E38F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_y_GET(pack) == (float)2.8904693E37F);
    assert(p138_z_GET(pack) == (float) -2.8343605E38F);
};


void c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)93);
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)193);
    {
        float exemplary[] =  {-5.343814E37F, 2.4513756E38F, -3.862387E37F, -1.9150422E38F, -1.757943E38F, 6.4529404E37F, -6.4053153E37F, 1.954266E38F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p139_time_usec_GET(pack) == (uint64_t)7180790925433005027L);
};


void c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)109);
    assert(p140_time_usec_GET(pack) == (uint64_t)673306687967548736L);
    {
        float exemplary[] =  {-1.725789E38F, 1.362936E37F, 9.010594E37F, -1.5440354E38F, -3.2248465E38F, 1.4358472E37F, -4.661489E37F, -1.462023E38F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_altitude_amsl_GET(pack) == (float) -2.66418E38F);
    assert(p141_altitude_terrain_GET(pack) == (float)1.6658766E38F);
    assert(p141_altitude_local_GET(pack) == (float) -2.2252118E38F);
    assert(p141_time_usec_GET(pack) == (uint64_t)6223877247300762799L);
    assert(p141_bottom_clearance_GET(pack) == (float) -3.2258004E38F);
    assert(p141_altitude_monotonic_GET(pack) == (float)1.063608E38F);
    assert(p141_altitude_relative_GET(pack) == (float)4.719326E37F);
};


void c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)46);
    {
        uint8_t exemplary[] =  {(uint8_t)75, (uint8_t)221, (uint8_t)216, (uint8_t)38, (uint8_t)73, (uint8_t)59, (uint8_t)98, (uint8_t)134, (uint8_t)179, (uint8_t)24, (uint8_t)117, (uint8_t)110, (uint8_t)67, (uint8_t)160, (uint8_t)93, (uint8_t)122, (uint8_t)148, (uint8_t)138, (uint8_t)86, (uint8_t)76, (uint8_t)172, (uint8_t)229, (uint8_t)72, (uint8_t)36, (uint8_t)155, (uint8_t)153, (uint8_t)175, (uint8_t)242, (uint8_t)133, (uint8_t)24, (uint8_t)205, (uint8_t)22, (uint8_t)241, (uint8_t)177, (uint8_t)215, (uint8_t)149, (uint8_t)30, (uint8_t)168, (uint8_t)143, (uint8_t)80, (uint8_t)13, (uint8_t)142, (uint8_t)29, (uint8_t)70, (uint8_t)175, (uint8_t)210, (uint8_t)247, (uint8_t)227, (uint8_t)23, (uint8_t)138, (uint8_t)106, (uint8_t)222, (uint8_t)223, (uint8_t)39, (uint8_t)72, (uint8_t)29, (uint8_t)10, (uint8_t)241, (uint8_t)80, (uint8_t)239, (uint8_t)75, (uint8_t)14, (uint8_t)140, (uint8_t)86, (uint8_t)225, (uint8_t)47, (uint8_t)200, (uint8_t)8, (uint8_t)230, (uint8_t)37, (uint8_t)153, (uint8_t)246, (uint8_t)192, (uint8_t)125, (uint8_t)235, (uint8_t)18, (uint8_t)19, (uint8_t)87, (uint8_t)148, (uint8_t)204, (uint8_t)103, (uint8_t)114, (uint8_t)221, (uint8_t)189, (uint8_t)2, (uint8_t)71, (uint8_t)190, (uint8_t)222, (uint8_t)211, (uint8_t)241, (uint8_t)68, (uint8_t)29, (uint8_t)240, (uint8_t)189, (uint8_t)230, (uint8_t)174, (uint8_t)213, (uint8_t)55, (uint8_t)141, (uint8_t)102, (uint8_t)86, (uint8_t)13, (uint8_t)224, (uint8_t)241, (uint8_t)102, (uint8_t)211, (uint8_t)131, (uint8_t)60, (uint8_t)139, (uint8_t)145, (uint8_t)1, (uint8_t)134, (uint8_t)85, (uint8_t)27, (uint8_t)2, (uint8_t)233, (uint8_t)78, (uint8_t)186, (uint8_t)17, (uint8_t)143} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)182);
    {
        uint8_t exemplary[] =  {(uint8_t)131, (uint8_t)181, (uint8_t)55, (uint8_t)15, (uint8_t)199, (uint8_t)199, (uint8_t)199, (uint8_t)201, (uint8_t)183, (uint8_t)198, (uint8_t)158, (uint8_t)173, (uint8_t)88, (uint8_t)144, (uint8_t)28, (uint8_t)126, (uint8_t)83, (uint8_t)23, (uint8_t)32, (uint8_t)52, (uint8_t)100, (uint8_t)176, (uint8_t)104, (uint8_t)220, (uint8_t)44, (uint8_t)142, (uint8_t)158, (uint8_t)193, (uint8_t)57, (uint8_t)82, (uint8_t)125, (uint8_t)132, (uint8_t)30, (uint8_t)116, (uint8_t)206, (uint8_t)163, (uint8_t)221, (uint8_t)156, (uint8_t)228, (uint8_t)22, (uint8_t)180, (uint8_t)129, (uint8_t)122, (uint8_t)110, (uint8_t)145, (uint8_t)92, (uint8_t)91, (uint8_t)200, (uint8_t)239, (uint8_t)192, (uint8_t)23, (uint8_t)206, (uint8_t)223, (uint8_t)199, (uint8_t)139, (uint8_t)177, (uint8_t)240, (uint8_t)46, (uint8_t)209, (uint8_t)80, (uint8_t)31, (uint8_t)150, (uint8_t)101, (uint8_t)122, (uint8_t)140, (uint8_t)139, (uint8_t)110, (uint8_t)187, (uint8_t)55, (uint8_t)85, (uint8_t)30, (uint8_t)215, (uint8_t)210, (uint8_t)42, (uint8_t)169, (uint8_t)44, (uint8_t)96, (uint8_t)241, (uint8_t)237, (uint8_t)92, (uint8_t)109, (uint8_t)30, (uint8_t)172, (uint8_t)200, (uint8_t)151, (uint8_t)225, (uint8_t)31, (uint8_t)248, (uint8_t)6, (uint8_t)59, (uint8_t)183, (uint8_t)11, (uint8_t)167, (uint8_t)39, (uint8_t)102, (uint8_t)221, (uint8_t)141, (uint8_t)113, (uint8_t)229, (uint8_t)213, (uint8_t)200, (uint8_t)187, (uint8_t)237, (uint8_t)138, (uint8_t)155, (uint8_t)36, (uint8_t)104, (uint8_t)200, (uint8_t)71, (uint8_t)254, (uint8_t)14, (uint8_t)154, (uint8_t)63, (uint8_t)43, (uint8_t)143, (uint8_t)136, (uint8_t)239, (uint8_t)9, (uint8_t)112, (uint8_t)34} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)66829004L);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t) -51);
    assert(p143_press_diff_GET(pack) == (float) -1.6845655E38F);
    assert(p143_press_abs_GET(pack) == (float)2.52316E37F);
};


void c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {3.0799354E38F, 1.6351668E38F, 1.6545734E38F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lon_GET(pack) == (int32_t)1375766043);
    assert(p144_custom_state_GET(pack) == (uint64_t)2717888927284365146L);
    assert(p144_alt_GET(pack) == (float)1.5226047E37F);
    {
        float exemplary[] =  {-1.5395566E37F, -2.6846333E36F, -8.607719E37F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lat_GET(pack) == (int32_t)1556273008);
    {
        float exemplary[] =  {3.3329078E38F, -1.342158E38F, 1.4809308E37F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {2.2025005E38F, -6.8749173E36F, -3.3925957E38F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-3.1413248E38F, 2.6149843E38F, 1.0600463E38F, -1.0675729E37F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_timestamp_GET(pack) == (uint64_t)6792780101266041577L);
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)250);
};


void c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_x_acc_GET(pack) == (float) -9.888169E37F);
    assert(p146_z_pos_GET(pack) == (float) -1.6894234E38F);
    assert(p146_yaw_rate_GET(pack) == (float) -8.4477514E37F);
    assert(p146_pitch_rate_GET(pack) == (float)2.7584817E38F);
    assert(p146_roll_rate_GET(pack) == (float)9.435573E37F);
    {
        float exemplary[] =  {4.1987072E37F, 3.057614E38F, -2.8744436E38F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_x_vel_GET(pack) == (float) -3.0191914E38F);
    assert(p146_time_usec_GET(pack) == (uint64_t)8459229483011361321L);
    assert(p146_x_pos_GET(pack) == (float) -7.394305E37F);
    assert(p146_y_pos_GET(pack) == (float) -3.0764189E38F);
    assert(p146_y_acc_GET(pack) == (float) -5.0228343E37F);
    assert(p146_z_vel_GET(pack) == (float)3.2853638E38F);
    assert(p146_airspeed_GET(pack) == (float)5.704219E37F);
    assert(p146_y_vel_GET(pack) == (float)3.273235E38F);
    {
        float exemplary[] =  {2.2286768E38F, 2.6500471E38F, -5.46416E37F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-2.4306344E38F, -2.0323453E38F, -2.2395342E38F, 8.215653E37F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_z_acc_GET(pack) == (float) -6.153206E37F);
};


void c_LoopBackDemoChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_NIMH);
    assert(p147_current_consumed_GET(pack) == (int32_t) -91129369);
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)165);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL);
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t)15305);
    assert(p147_energy_consumed_GET(pack) == (int32_t) -1729670274);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t) -100);
    {
        uint16_t exemplary[] =  {(uint16_t)30069, (uint16_t)12518, (uint16_t)30842, (uint16_t)20782, (uint16_t)16283, (uint16_t)13262, (uint16_t)40865, (uint16_t)64318, (uint16_t)9993, (uint16_t)6006} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t)8174);
};


void c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)9999);
    {
        uint8_t exemplary[] =  {(uint8_t)230, (uint8_t)83, (uint8_t)16, (uint8_t)95, (uint8_t)12, (uint8_t)22, (uint8_t)86, (uint8_t)200} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)10, (uint8_t)107, (uint8_t)16, (uint8_t)108, (uint8_t)230, (uint8_t)78, (uint8_t)153, (uint8_t)5} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_board_version_GET(pack) == (uint32_t)2775529188L);
    assert(p148_capabilities_GET(pack) == e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MAVLINK2);
    assert(p148_uid_GET(pack) == (uint64_t)8599935115229991113L);
    {
        uint8_t exemplary[] =  {(uint8_t)175, (uint8_t)84, (uint8_t)201, (uint8_t)1, (uint8_t)78, (uint8_t)239, (uint8_t)49, (uint8_t)205, (uint8_t)81, (uint8_t)95, (uint8_t)189, (uint8_t)11, (uint8_t)155, (uint8_t)94, (uint8_t)233, (uint8_t)55, (uint8_t)114, (uint8_t)23} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)60905);
    assert(p148_os_sw_version_GET(pack) == (uint32_t)1271115204L);
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)745415640L);
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)2338041536L);
    {
        uint8_t exemplary[] =  {(uint8_t)90, (uint8_t)79, (uint8_t)30, (uint8_t)70, (uint8_t)158, (uint8_t)221, (uint8_t)191, (uint8_t)254} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_size_x_GET(pack) == (float)1.8853116E38F);
    assert(p149_angle_y_GET(pack) == (float)1.0874596E38F);
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)30);
    assert(p149_distance_GET(pack) == (float)2.936526E38F);
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON);
    assert(p149_size_y_GET(pack) == (float) -9.713978E37F);
    assert(p149_x_TRY(ph) == (float) -1.2569357E38F);
    assert(p149_z_TRY(ph) == (float)4.6078536E37F);
    assert(p149_y_TRY(ph) == (float) -2.1549141E38F);
    {
        float exemplary[] =  {-1.2245927E38F, 1.4668084E38F, 5.591384E37F, -1.171001E38F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_angle_x_GET(pack) == (float) -7.4662354E37F);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)214);
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p149_time_usec_GET(pack) == (uint64_t)4903025259323122880L);
};


void c_LoopBackDemoChannel_on_SENSOR_OFFSETS_150(Bounds_Inside * ph, Pack * pack)
{
    assert(p150_raw_press_GET(pack) == (int32_t) -1579267925);
    assert(p150_gyro_cal_z_GET(pack) == (float)7.012354E37F);
    assert(p150_accel_cal_x_GET(pack) == (float)2.860304E38F);
    assert(p150_gyro_cal_y_GET(pack) == (float)3.1044122E38F);
    assert(p150_accel_cal_z_GET(pack) == (float) -2.2330706E38F);
    assert(p150_gyro_cal_x_GET(pack) == (float)2.1309393E38F);
    assert(p150_mag_ofs_y_GET(pack) == (int16_t)(int16_t) -16837);
    assert(p150_accel_cal_y_GET(pack) == (float) -1.5397079E38F);
    assert(p150_mag_ofs_z_GET(pack) == (int16_t)(int16_t)20377);
    assert(p150_mag_declination_GET(pack) == (float)1.2611883E38F);
    assert(p150_raw_temp_GET(pack) == (int32_t) -594743805);
    assert(p150_mag_ofs_x_GET(pack) == (int16_t)(int16_t) -15522);
};


void c_LoopBackDemoChannel_on_SET_MAG_OFFSETS_151(Bounds_Inside * ph, Pack * pack)
{
    assert(p151_target_component_GET(pack) == (uint8_t)(uint8_t)64);
    assert(p151_mag_ofs_z_GET(pack) == (int16_t)(int16_t)16316);
    assert(p151_mag_ofs_y_GET(pack) == (int16_t)(int16_t) -14157);
    assert(p151_target_system_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p151_mag_ofs_x_GET(pack) == (int16_t)(int16_t) -4731);
};


void c_LoopBackDemoChannel_on_MEMINFO_152(Bounds_Inside * ph, Pack * pack)
{
    assert(p152_freemem_GET(pack) == (uint16_t)(uint16_t)36412);
    assert(p152_brkval_GET(pack) == (uint16_t)(uint16_t)46559);
    assert(p152_freemem32_TRY(ph) == (uint32_t)3485485193L);
};


void c_LoopBackDemoChannel_on_AP_ADC_153(Bounds_Inside * ph, Pack * pack)
{
    assert(p153_adc3_GET(pack) == (uint16_t)(uint16_t)32538);
    assert(p153_adc5_GET(pack) == (uint16_t)(uint16_t)42873);
    assert(p153_adc1_GET(pack) == (uint16_t)(uint16_t)45797);
    assert(p153_adc6_GET(pack) == (uint16_t)(uint16_t)21353);
    assert(p153_adc4_GET(pack) == (uint16_t)(uint16_t)44077);
    assert(p153_adc2_GET(pack) == (uint16_t)(uint16_t)18073);
};


void c_LoopBackDemoChannel_on_DIGICAM_CONFIGURE_154(Bounds_Inside * ph, Pack * pack)
{
    assert(p154_mode_GET(pack) == (uint8_t)(uint8_t)154);
    assert(p154_iso_GET(pack) == (uint8_t)(uint8_t)235);
    assert(p154_command_id_GET(pack) == (uint8_t)(uint8_t)237);
    assert(p154_aperture_GET(pack) == (uint8_t)(uint8_t)9);
    assert(p154_target_component_GET(pack) == (uint8_t)(uint8_t)152);
    assert(p154_shutter_speed_GET(pack) == (uint16_t)(uint16_t)30584);
    assert(p154_extra_param_GET(pack) == (uint8_t)(uint8_t)172);
    assert(p154_engine_cut_off_GET(pack) == (uint8_t)(uint8_t)13);
    assert(p154_exposure_type_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p154_target_system_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p154_extra_value_GET(pack) == (float)1.6760428E38F);
};


void c_LoopBackDemoChannel_on_DIGICAM_CONTROL_155(Bounds_Inside * ph, Pack * pack)
{
    assert(p155_shot_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p155_extra_value_GET(pack) == (float) -2.9372049E38F);
    assert(p155_extra_param_GET(pack) == (uint8_t)(uint8_t)41);
    assert(p155_target_system_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p155_zoom_pos_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p155_zoom_step_GET(pack) == (int8_t)(int8_t)102);
    assert(p155_target_component_GET(pack) == (uint8_t)(uint8_t)134);
    assert(p155_session_GET(pack) == (uint8_t)(uint8_t)56);
    assert(p155_command_id_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p155_focus_lock_GET(pack) == (uint8_t)(uint8_t)4);
};


void c_LoopBackDemoChannel_on_MOUNT_CONFIGURE_156(Bounds_Inside * ph, Pack * pack)
{
    assert(p156_target_system_GET(pack) == (uint8_t)(uint8_t)124);
    assert(p156_stab_pitch_GET(pack) == (uint8_t)(uint8_t)185);
    assert(p156_target_component_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p156_stab_roll_GET(pack) == (uint8_t)(uint8_t)70);
    assert(p156_stab_yaw_GET(pack) == (uint8_t)(uint8_t)63);
    assert(p156_mount_mode_GET(pack) == e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_NEUTRAL);
};


void c_LoopBackDemoChannel_on_MOUNT_CONTROL_157(Bounds_Inside * ph, Pack * pack)
{
    assert(p157_input_a_GET(pack) == (int32_t) -2035463648);
    assert(p157_save_position_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p157_target_system_GET(pack) == (uint8_t)(uint8_t)216);
    assert(p157_target_component_GET(pack) == (uint8_t)(uint8_t)235);
    assert(p157_input_b_GET(pack) == (int32_t) -2018012814);
    assert(p157_input_c_GET(pack) == (int32_t) -312038060);
};


void c_LoopBackDemoChannel_on_MOUNT_STATUS_158(Bounds_Inside * ph, Pack * pack)
{
    assert(p158_pointing_a_GET(pack) == (int32_t) -1590969205);
    assert(p158_pointing_b_GET(pack) == (int32_t)718597256);
    assert(p158_pointing_c_GET(pack) == (int32_t) -815885501);
    assert(p158_target_component_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p158_target_system_GET(pack) == (uint8_t)(uint8_t)84);
};


void c_LoopBackDemoChannel_on_FENCE_POINT_160(Bounds_Inside * ph, Pack * pack)
{
    assert(p160_lng_GET(pack) == (float) -1.5820584E38F);
    assert(p160_idx_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p160_count_GET(pack) == (uint8_t)(uint8_t)251);
    assert(p160_lat_GET(pack) == (float)1.9412977E37F);
    assert(p160_target_system_GET(pack) == (uint8_t)(uint8_t)239);
    assert(p160_target_component_GET(pack) == (uint8_t)(uint8_t)37);
};


void c_LoopBackDemoChannel_on_FENCE_FETCH_POINT_161(Bounds_Inside * ph, Pack * pack)
{
    assert(p161_target_system_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p161_target_component_GET(pack) == (uint8_t)(uint8_t)252);
    assert(p161_idx_GET(pack) == (uint8_t)(uint8_t)149);
};


void c_LoopBackDemoChannel_on_FENCE_STATUS_162(Bounds_Inside * ph, Pack * pack)
{
    assert(p162_breach_count_GET(pack) == (uint16_t)(uint16_t)9760);
    assert(p162_breach_time_GET(pack) == (uint32_t)479161813L);
    assert(p162_breach_status_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p162_breach_type_GET(pack) == e_FENCE_BREACH_FENCE_BREACH_MAXALT);
};


void c_LoopBackDemoChannel_on_AHRS_163(Bounds_Inside * ph, Pack * pack)
{
    assert(p163_error_yaw_GET(pack) == (float) -9.424832E37F);
    assert(p163_renorm_val_GET(pack) == (float) -9.525016E37F);
    assert(p163_omegaIx_GET(pack) == (float)6.618985E37F);
    assert(p163_omegaIy_GET(pack) == (float) -1.5669243E38F);
    assert(p163_omegaIz_GET(pack) == (float) -2.3994788E38F);
    assert(p163_error_rp_GET(pack) == (float)2.7917982E38F);
    assert(p163_accel_weight_GET(pack) == (float) -1.1451615E38F);
};


void c_LoopBackDemoChannel_on_SIMSTATE_164(Bounds_Inside * ph, Pack * pack)
{
    assert(p164_ygyro_GET(pack) == (float)3.1385443E38F);
    assert(p164_xgyro_GET(pack) == (float) -1.7176615E38F);
    assert(p164_zgyro_GET(pack) == (float) -2.4914583E38F);
    assert(p164_lng_GET(pack) == (int32_t) -1179316840);
    assert(p164_xacc_GET(pack) == (float) -1.8808295E37F);
    assert(p164_lat_GET(pack) == (int32_t)117311869);
    assert(p164_roll_GET(pack) == (float)1.5977427E38F);
    assert(p164_yacc_GET(pack) == (float)1.6983207E38F);
    assert(p164_zacc_GET(pack) == (float) -3.0120283E38F);
    assert(p164_yaw_GET(pack) == (float)9.282846E37F);
    assert(p164_pitch_GET(pack) == (float) -2.1396447E38F);
};


void c_LoopBackDemoChannel_on_HWSTATUS_165(Bounds_Inside * ph, Pack * pack)
{
    assert(p165_Vcc_GET(pack) == (uint16_t)(uint16_t)58304);
    assert(p165_I2Cerr_GET(pack) == (uint8_t)(uint8_t)127);
};


void c_LoopBackDemoChannel_on_RADIO_166(Bounds_Inside * ph, Pack * pack)
{
    assert(p166_rxerrors_GET(pack) == (uint16_t)(uint16_t)634);
    assert(p166_noise_GET(pack) == (uint8_t)(uint8_t)132);
    assert(p166_remrssi_GET(pack) == (uint8_t)(uint8_t)12);
    assert(p166_rssi_GET(pack) == (uint8_t)(uint8_t)51);
    assert(p166_fixed__GET(pack) == (uint16_t)(uint16_t)4775);
    assert(p166_txbuf_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p166_remnoise_GET(pack) == (uint8_t)(uint8_t)176);
};


void c_LoopBackDemoChannel_on_LIMITS_STATUS_167(Bounds_Inside * ph, Pack * pack)
{
    assert(p167_mods_triggered_GET(pack) == e_LIMIT_MODULE_LIMIT_ALTITUDE);
    assert(p167_last_recovery_GET(pack) == (uint32_t)2342811393L);
    assert(p167_limits_state_GET(pack) == e_LIMITS_STATE_LIMITS_TRIGGERED);
    assert(p167_last_action_GET(pack) == (uint32_t)1278445368L);
    assert(p167_mods_required_GET(pack) == e_LIMIT_MODULE_LIMIT_GPSLOCK);
    assert(p167_last_clear_GET(pack) == (uint32_t)2064376321L);
    assert(p167_mods_enabled_GET(pack) == e_LIMIT_MODULE_LIMIT_GPSLOCK);
    assert(p167_last_trigger_GET(pack) == (uint32_t)1054747665L);
    assert(p167_breach_count_GET(pack) == (uint16_t)(uint16_t)36529);
};


void c_LoopBackDemoChannel_on_WIND_168(Bounds_Inside * ph, Pack * pack)
{
    assert(p168_speed_z_GET(pack) == (float) -7.2069735E37F);
    assert(p168_speed_GET(pack) == (float) -2.2988727E37F);
    assert(p168_direction_GET(pack) == (float) -2.9902588E38F);
};


void c_LoopBackDemoChannel_on_DATA16_169(Bounds_Inside * ph, Pack * pack)
{
    assert(p169_len_GET(pack) == (uint8_t)(uint8_t)53);
    {
        uint8_t exemplary[] =  {(uint8_t)214, (uint8_t)104, (uint8_t)151, (uint8_t)241, (uint8_t)120, (uint8_t)97, (uint8_t)129, (uint8_t)96, (uint8_t)185, (uint8_t)245, (uint8_t)115, (uint8_t)235, (uint8_t)35, (uint8_t)49, (uint8_t)162, (uint8_t)213} ;
        uint8_t*  sample = p169_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p169_type_GET(pack) == (uint8_t)(uint8_t)168);
};


void c_LoopBackDemoChannel_on_DATA32_170(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)229, (uint8_t)206, (uint8_t)197, (uint8_t)206, (uint8_t)99, (uint8_t)225, (uint8_t)48, (uint8_t)30, (uint8_t)53, (uint8_t)32, (uint8_t)123, (uint8_t)16, (uint8_t)95, (uint8_t)83, (uint8_t)93, (uint8_t)230, (uint8_t)40, (uint8_t)246, (uint8_t)40, (uint8_t)67, (uint8_t)150, (uint8_t)253, (uint8_t)188, (uint8_t)248, (uint8_t)202, (uint8_t)38, (uint8_t)71, (uint8_t)170, (uint8_t)218, (uint8_t)125, (uint8_t)228, (uint8_t)170} ;
        uint8_t*  sample = p170_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p170_len_GET(pack) == (uint8_t)(uint8_t)65);
    assert(p170_type_GET(pack) == (uint8_t)(uint8_t)230);
};


void c_LoopBackDemoChannel_on_DATA64_171(Bounds_Inside * ph, Pack * pack)
{
    assert(p171_len_GET(pack) == (uint8_t)(uint8_t)175);
    {
        uint8_t exemplary[] =  {(uint8_t)230, (uint8_t)91, (uint8_t)101, (uint8_t)36, (uint8_t)244, (uint8_t)16, (uint8_t)225, (uint8_t)68, (uint8_t)107, (uint8_t)114, (uint8_t)43, (uint8_t)48, (uint8_t)52, (uint8_t)104, (uint8_t)12, (uint8_t)190, (uint8_t)7, (uint8_t)117, (uint8_t)208, (uint8_t)0, (uint8_t)97, (uint8_t)105, (uint8_t)179, (uint8_t)142, (uint8_t)120, (uint8_t)205, (uint8_t)238, (uint8_t)168, (uint8_t)209, (uint8_t)81, (uint8_t)43, (uint8_t)63, (uint8_t)140, (uint8_t)67, (uint8_t)82, (uint8_t)177, (uint8_t)156, (uint8_t)116, (uint8_t)63, (uint8_t)217, (uint8_t)205, (uint8_t)174, (uint8_t)175, (uint8_t)144, (uint8_t)186, (uint8_t)6, (uint8_t)72, (uint8_t)126, (uint8_t)75, (uint8_t)207, (uint8_t)161, (uint8_t)190, (uint8_t)128, (uint8_t)182, (uint8_t)6, (uint8_t)154, (uint8_t)52, (uint8_t)138, (uint8_t)66, (uint8_t)97, (uint8_t)208, (uint8_t)139, (uint8_t)128, (uint8_t)13} ;
        uint8_t*  sample = p171_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p171_type_GET(pack) == (uint8_t)(uint8_t)165);
};


void c_LoopBackDemoChannel_on_DATA96_172(Bounds_Inside * ph, Pack * pack)
{
    assert(p172_type_GET(pack) == (uint8_t)(uint8_t)229);
    {
        uint8_t exemplary[] =  {(uint8_t)233, (uint8_t)89, (uint8_t)249, (uint8_t)137, (uint8_t)147, (uint8_t)81, (uint8_t)158, (uint8_t)173, (uint8_t)125, (uint8_t)230, (uint8_t)100, (uint8_t)40, (uint8_t)108, (uint8_t)238, (uint8_t)96, (uint8_t)5, (uint8_t)111, (uint8_t)53, (uint8_t)254, (uint8_t)167, (uint8_t)42, (uint8_t)221, (uint8_t)126, (uint8_t)5, (uint8_t)175, (uint8_t)104, (uint8_t)30, (uint8_t)215, (uint8_t)83, (uint8_t)79, (uint8_t)181, (uint8_t)123, (uint8_t)154, (uint8_t)8, (uint8_t)196, (uint8_t)8, (uint8_t)129, (uint8_t)84, (uint8_t)12, (uint8_t)75, (uint8_t)193, (uint8_t)151, (uint8_t)250, (uint8_t)249, (uint8_t)129, (uint8_t)203, (uint8_t)15, (uint8_t)131, (uint8_t)26, (uint8_t)34, (uint8_t)86, (uint8_t)64, (uint8_t)130, (uint8_t)175, (uint8_t)164, (uint8_t)251, (uint8_t)235, (uint8_t)47, (uint8_t)26, (uint8_t)10, (uint8_t)128, (uint8_t)212, (uint8_t)77, (uint8_t)248, (uint8_t)249, (uint8_t)64, (uint8_t)189, (uint8_t)66, (uint8_t)71, (uint8_t)221, (uint8_t)38, (uint8_t)60, (uint8_t)51, (uint8_t)245, (uint8_t)90, (uint8_t)60, (uint8_t)194, (uint8_t)163, (uint8_t)105, (uint8_t)138, (uint8_t)128, (uint8_t)189, (uint8_t)231, (uint8_t)103, (uint8_t)21, (uint8_t)206, (uint8_t)242, (uint8_t)44, (uint8_t)159, (uint8_t)53, (uint8_t)231, (uint8_t)135, (uint8_t)39, (uint8_t)29, (uint8_t)244, (uint8_t)77} ;
        uint8_t*  sample = p172_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 96);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p172_len_GET(pack) == (uint8_t)(uint8_t)204);
};


void c_LoopBackDemoChannel_on_RANGEFINDER_173(Bounds_Inside * ph, Pack * pack)
{
    assert(p173_distance_GET(pack) == (float)8.775278E36F);
    assert(p173_voltage_GET(pack) == (float)1.1511416E38F);
};


void c_LoopBackDemoChannel_on_AIRSPEED_AUTOCAL_174(Bounds_Inside * ph, Pack * pack)
{
    assert(p174_diff_pressure_GET(pack) == (float)2.627316E38F);
    assert(p174_state_x_GET(pack) == (float)2.3604212E38F);
    assert(p174_state_z_GET(pack) == (float)1.7863696E36F);
    assert(p174_vy_GET(pack) == (float)1.5481385E38F);
    assert(p174_Pby_GET(pack) == (float) -2.5670368E37F);
    assert(p174_EAS2TAS_GET(pack) == (float) -2.6361409E38F);
    assert(p174_Pax_GET(pack) == (float) -2.7244335E38F);
    assert(p174_state_y_GET(pack) == (float) -7.902479E37F);
    assert(p174_vx_GET(pack) == (float) -2.699091E37F);
    assert(p174_Pcz_GET(pack) == (float) -3.319359E37F);
    assert(p174_ratio_GET(pack) == (float)2.2589408E38F);
    assert(p174_vz_GET(pack) == (float)1.5437915E36F);
};


void c_LoopBackDemoChannel_on_RALLY_POINT_175(Bounds_Inside * ph, Pack * pack)
{
    assert(p175_count_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p175_land_dir_GET(pack) == (uint16_t)(uint16_t)49355);
    assert(p175_target_component_GET(pack) == (uint8_t)(uint8_t)167);
    assert(p175_idx_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p175_lng_GET(pack) == (int32_t) -1840716910);
    assert(p175_lat_GET(pack) == (int32_t)1520808089);
    assert(p175_target_system_GET(pack) == (uint8_t)(uint8_t)232);
    assert(p175_alt_GET(pack) == (int16_t)(int16_t) -5455);
    assert(p175_flags_GET(pack) == e_RALLY_FLAGS_LAND_IMMEDIATELY);
    assert(p175_break_alt_GET(pack) == (int16_t)(int16_t)32264);
};


void c_LoopBackDemoChannel_on_RALLY_FETCH_POINT_176(Bounds_Inside * ph, Pack * pack)
{
    assert(p176_target_component_GET(pack) == (uint8_t)(uint8_t)37);
    assert(p176_idx_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p176_target_system_GET(pack) == (uint8_t)(uint8_t)156);
};


void c_LoopBackDemoChannel_on_COMPASSMOT_STATUS_177(Bounds_Inside * ph, Pack * pack)
{
    assert(p177_CompensationZ_GET(pack) == (float)2.6876929E38F);
    assert(p177_CompensationY_GET(pack) == (float) -2.132742E37F);
    assert(p177_throttle_GET(pack) == (uint16_t)(uint16_t)21490);
    assert(p177_interference_GET(pack) == (uint16_t)(uint16_t)49008);
    assert(p177_current_GET(pack) == (float) -2.1091678E38F);
    assert(p177_CompensationX_GET(pack) == (float) -4.8398406E36F);
};


void c_LoopBackDemoChannel_on_AHRS2_178(Bounds_Inside * ph, Pack * pack)
{
    assert(p178_roll_GET(pack) == (float) -2.18424E38F);
    assert(p178_pitch_GET(pack) == (float)6.0537035E37F);
    assert(p178_lat_GET(pack) == (int32_t)1953768174);
    assert(p178_altitude_GET(pack) == (float) -1.8755982E38F);
    assert(p178_lng_GET(pack) == (int32_t) -71042179);
    assert(p178_yaw_GET(pack) == (float) -3.2196327E38F);
};


void c_LoopBackDemoChannel_on_CAMERA_STATUS_179(Bounds_Inside * ph, Pack * pack)
{
    assert(p179_img_idx_GET(pack) == (uint16_t)(uint16_t)32291);
    assert(p179_p3_GET(pack) == (float) -3.0889179E38F);
    assert(p179_event_id_GET(pack) == e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_TRIGGER);
    assert(p179_p1_GET(pack) == (float) -8.335899E37F);
    assert(p179_p2_GET(pack) == (float)1.7823453E38F);
    assert(p179_cam_idx_GET(pack) == (uint8_t)(uint8_t)52);
    assert(p179_target_system_GET(pack) == (uint8_t)(uint8_t)167);
    assert(p179_p4_GET(pack) == (float)2.8667513E38F);
    assert(p179_time_usec_GET(pack) == (uint64_t)4706417044326555564L);
};


void c_LoopBackDemoChannel_on_CAMERA_FEEDBACK_180(Bounds_Inside * ph, Pack * pack)
{
    assert(p180_lat_GET(pack) == (int32_t) -2045931032);
    assert(p180_foc_len_GET(pack) == (float)3.3419738E38F);
    assert(p180_alt_msl_GET(pack) == (float)8.822345E37F);
    assert(p180_yaw_GET(pack) == (float) -1.1044116E38F);
    assert(p180_time_usec_GET(pack) == (uint64_t)2117520856037814619L);
    assert(p180_pitch_GET(pack) == (float) -2.7764868E38F);
    assert(p180_target_system_GET(pack) == (uint8_t)(uint8_t)178);
    assert(p180_cam_idx_GET(pack) == (uint8_t)(uint8_t)2);
    assert(p180_img_idx_GET(pack) == (uint16_t)(uint16_t)24557);
    assert(p180_roll_GET(pack) == (float) -2.9591117E38F);
    assert(p180_flags_GET(pack) == e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_CLOSEDLOOP);
    assert(p180_alt_rel_GET(pack) == (float)6.511096E37F);
    assert(p180_lng_GET(pack) == (int32_t)214634616);
};


void c_LoopBackDemoChannel_on_BATTERY2_181(Bounds_Inside * ph, Pack * pack)
{
    assert(p181_current_battery_GET(pack) == (int16_t)(int16_t) -3636);
    assert(p181_voltage_GET(pack) == (uint16_t)(uint16_t)54847);
};


void c_LoopBackDemoChannel_on_AHRS3_182(Bounds_Inside * ph, Pack * pack)
{
    assert(p182_v2_GET(pack) == (float)1.0581919E38F);
    assert(p182_v4_GET(pack) == (float) -1.6173555E38F);
    assert(p182_v1_GET(pack) == (float) -7.7223277E37F);
    assert(p182_yaw_GET(pack) == (float) -9.890003E37F);
    assert(p182_pitch_GET(pack) == (float)3.8468594E37F);
    assert(p182_altitude_GET(pack) == (float)2.8746115E38F);
    assert(p182_lat_GET(pack) == (int32_t)1103231578);
    assert(p182_v3_GET(pack) == (float) -3.113939E38F);
    assert(p182_lng_GET(pack) == (int32_t) -878298418);
    assert(p182_roll_GET(pack) == (float)3.1158531E38F);
};


void c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_REQUEST_183(Bounds_Inside * ph, Pack * pack)
{
    assert(p183_target_component_GET(pack) == (uint8_t)(uint8_t)80);
    assert(p183_target_system_GET(pack) == (uint8_t)(uint8_t)5);
};


void c_LoopBackDemoChannel_on_REMOTE_LOG_DATA_BLOCK_184(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)57, (uint8_t)250, (uint8_t)12, (uint8_t)149, (uint8_t)9, (uint8_t)252, (uint8_t)179, (uint8_t)99, (uint8_t)190, (uint8_t)192, (uint8_t)232, (uint8_t)154, (uint8_t)212, (uint8_t)156, (uint8_t)140, (uint8_t)242, (uint8_t)94, (uint8_t)128, (uint8_t)124, (uint8_t)28, (uint8_t)63, (uint8_t)30, (uint8_t)86, (uint8_t)115, (uint8_t)250, (uint8_t)77, (uint8_t)165, (uint8_t)127, (uint8_t)67, (uint8_t)97, (uint8_t)178, (uint8_t)53, (uint8_t)215, (uint8_t)209, (uint8_t)168, (uint8_t)217, (uint8_t)56, (uint8_t)183, (uint8_t)229, (uint8_t)12, (uint8_t)197, (uint8_t)130, (uint8_t)137, (uint8_t)208, (uint8_t)85, (uint8_t)77, (uint8_t)248, (uint8_t)43, (uint8_t)110, (uint8_t)67, (uint8_t)249, (uint8_t)135, (uint8_t)93, (uint8_t)72, (uint8_t)100, (uint8_t)9, (uint8_t)230, (uint8_t)234, (uint8_t)102, (uint8_t)117, (uint8_t)52, (uint8_t)147, (uint8_t)206, (uint8_t)73, (uint8_t)51, (uint8_t)90, (uint8_t)61, (uint8_t)50, (uint8_t)114, (uint8_t)147, (uint8_t)213, (uint8_t)46, (uint8_t)230, (uint8_t)237, (uint8_t)27, (uint8_t)151, (uint8_t)8, (uint8_t)101, (uint8_t)148, (uint8_t)42, (uint8_t)221, (uint8_t)134, (uint8_t)135, (uint8_t)175, (uint8_t)12, (uint8_t)248, (uint8_t)140, (uint8_t)177, (uint8_t)66, (uint8_t)15, (uint8_t)80, (uint8_t)108, (uint8_t)33, (uint8_t)168, (uint8_t)84, (uint8_t)220, (uint8_t)228, (uint8_t)36, (uint8_t)23, (uint8_t)206, (uint8_t)204, (uint8_t)255, (uint8_t)94, (uint8_t)104, (uint8_t)59, (uint8_t)78, (uint8_t)245, (uint8_t)117, (uint8_t)121, (uint8_t)168, (uint8_t)138, (uint8_t)14, (uint8_t)186, (uint8_t)137, (uint8_t)76, (uint8_t)52, (uint8_t)157, (uint8_t)117, (uint8_t)144, (uint8_t)41, (uint8_t)111, (uint8_t)162, (uint8_t)25, (uint8_t)94, (uint8_t)246, (uint8_t)159, (uint8_t)153, (uint8_t)4, (uint8_t)239, (uint8_t)234, (uint8_t)205, (uint8_t)73, (uint8_t)237, (uint8_t)253, (uint8_t)105, (uint8_t)210, (uint8_t)229, (uint8_t)243, (uint8_t)129, (uint8_t)233, (uint8_t)114, (uint8_t)42, (uint8_t)26, (uint8_t)193, (uint8_t)247, (uint8_t)144, (uint8_t)225, (uint8_t)203, (uint8_t)162, (uint8_t)122, (uint8_t)59, (uint8_t)195, (uint8_t)215, (uint8_t)65, (uint8_t)58, (uint8_t)242, (uint8_t)122, (uint8_t)100, (uint8_t)117, (uint8_t)247, (uint8_t)127, (uint8_t)151, (uint8_t)124, (uint8_t)129, (uint8_t)131, (uint8_t)191, (uint8_t)166, (uint8_t)42, (uint8_t)150, (uint8_t)167, (uint8_t)247, (uint8_t)135, (uint8_t)107, (uint8_t)179, (uint8_t)119, (uint8_t)247, (uint8_t)161, (uint8_t)57, (uint8_t)61, (uint8_t)102, (uint8_t)115, (uint8_t)3, (uint8_t)236, (uint8_t)63, (uint8_t)110, (uint8_t)2, (uint8_t)19, (uint8_t)39, (uint8_t)255, (uint8_t)22, (uint8_t)77, (uint8_t)20, (uint8_t)159, (uint8_t)222, (uint8_t)93, (uint8_t)13, (uint8_t)183, (uint8_t)110, (uint8_t)119, (uint8_t)151} ;
        uint8_t*  sample = p184_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 200);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p184_target_system_GET(pack) == (uint8_t)(uint8_t)146);
    assert(p184_target_component_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p184_seqno_GET(pack) == e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_STOP);
};


void c_LoopBackDemoChannel_on_REMOTE_LOG_BLOCK_STATUS_185(Bounds_Inside * ph, Pack * pack)
{
    assert(p185_target_component_GET(pack) == (uint8_t)(uint8_t)44);
    assert(p185_target_system_GET(pack) == (uint8_t)(uint8_t)219);
    assert(p185_status_GET(pack) == e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_ACK);
    assert(p185_seqno_GET(pack) == (uint32_t)4173146683L);
};


void c_LoopBackDemoChannel_on_LED_CONTROL_186(Bounds_Inside * ph, Pack * pack)
{
    assert(p186_custom_len_GET(pack) == (uint8_t)(uint8_t)88);
    assert(p186_instance_GET(pack) == (uint8_t)(uint8_t)132);
    assert(p186_pattern_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p186_target_system_GET(pack) == (uint8_t)(uint8_t)101);
    {
        uint8_t exemplary[] =  {(uint8_t)68, (uint8_t)130, (uint8_t)59, (uint8_t)92, (uint8_t)140, (uint8_t)36, (uint8_t)58, (uint8_t)111, (uint8_t)53, (uint8_t)74, (uint8_t)66, (uint8_t)215, (uint8_t)160, (uint8_t)29, (uint8_t)153, (uint8_t)127, (uint8_t)210, (uint8_t)236, (uint8_t)168, (uint8_t)23, (uint8_t)50, (uint8_t)60, (uint8_t)15, (uint8_t)99} ;
        uint8_t*  sample = p186_custom_bytes_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p186_target_component_GET(pack) == (uint8_t)(uint8_t)229);
};


void c_LoopBackDemoChannel_on_MAG_CAL_PROGRESS_191(Bounds_Inside * ph, Pack * pack)
{
    assert(p191_cal_mask_GET(pack) == (uint8_t)(uint8_t)69);
    assert(p191_direction_z_GET(pack) == (float) -2.3552013E37F);
    assert(p191_direction_x_GET(pack) == (float) -2.282132E38F);
    assert(p191_cal_status_GET(pack) == e_MAG_CAL_STATUS_MAG_CAL_RUNNING_STEP_TWO);
    assert(p191_compass_id_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p191_direction_y_GET(pack) == (float) -2.7257313E38F);
    assert(p191_attempt_GET(pack) == (uint8_t)(uint8_t)24);
    {
        uint8_t exemplary[] =  {(uint8_t)174, (uint8_t)89, (uint8_t)154, (uint8_t)35, (uint8_t)158, (uint8_t)203, (uint8_t)177, (uint8_t)155, (uint8_t)114, (uint8_t)57} ;
        uint8_t*  sample = p191_completion_mask_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p191_completion_pct_GET(pack) == (uint8_t)(uint8_t)14);
};


void c_LoopBackDemoChannel_on_MAG_CAL_REPORT_192(Bounds_Inside * ph, Pack * pack)
{
    assert(p192_cal_status_GET(pack) == e_MAG_CAL_STATUS_MAG_CAL_RUNNING_STEP_ONE);
    assert(p192_compass_id_GET(pack) == (uint8_t)(uint8_t)239);
    assert(p192_ofs_z_GET(pack) == (float) -6.4722746E37F);
    assert(p192_offdiag_y_GET(pack) == (float) -1.710795E38F);
    assert(p192_offdiag_z_GET(pack) == (float)2.8926476E38F);
    assert(p192_ofs_x_GET(pack) == (float)1.405528E37F);
    assert(p192_diag_x_GET(pack) == (float) -5.221766E37F);
    assert(p192_diag_z_GET(pack) == (float)8.785531E37F);
    assert(p192_fitness_GET(pack) == (float) -2.9418743E38F);
    assert(p192_autosaved_GET(pack) == (uint8_t)(uint8_t)103);
    assert(p192_ofs_y_GET(pack) == (float)5.6115454E37F);
    assert(p192_diag_y_GET(pack) == (float)1.1972406E38F);
    assert(p192_cal_mask_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p192_offdiag_x_GET(pack) == (float) -3.1873028E38F);
};


void c_LoopBackDemoChannel_on_EKF_STATUS_REPORT_193(Bounds_Inside * ph, Pack * pack)
{
    assert(p193_terrain_alt_variance_GET(pack) == (float) -1.5273746E38F);
    assert(p193_compass_variance_GET(pack) == (float) -2.8166561E38F);
    assert(p193_flags_GET(pack) == e_EKF_STATUS_FLAGS_EKF_PRED_POS_HORIZ_REL);
    assert(p193_velocity_variance_GET(pack) == (float)1.7448637E36F);
    assert(p193_pos_vert_variance_GET(pack) == (float) -3.2420274E38F);
    assert(p193_pos_horiz_variance_GET(pack) == (float) -2.6690665E38F);
};


void c_LoopBackDemoChannel_on_PID_TUNING_194(Bounds_Inside * ph, Pack * pack)
{
    assert(p194_P_GET(pack) == (float) -5.021555E37F);
    assert(p194_desired_GET(pack) == (float)2.656921E38F);
    assert(p194_axis_GET(pack) == e_PID_TUNING_AXIS_PID_TUNING_ACCZ);
    assert(p194_D_GET(pack) == (float) -2.5154638E38F);
    assert(p194_achieved_GET(pack) == (float) -2.2209305E38F);
    assert(p194_FF_GET(pack) == (float)2.3683226E38F);
    assert(p194_I_GET(pack) == (float)4.897631E37F);
};


void c_LoopBackDemoChannel_on_GIMBAL_REPORT_200(Bounds_Inside * ph, Pack * pack)
{
    assert(p200_delta_angle_x_GET(pack) == (float)1.7592719E38F);
    assert(p200_target_component_GET(pack) == (uint8_t)(uint8_t)83);
    assert(p200_delta_velocity_x_GET(pack) == (float) -2.328791E38F);
    assert(p200_joint_el_GET(pack) == (float) -3.0479184E38F);
    assert(p200_delta_angle_z_GET(pack) == (float) -2.4934578E38F);
    assert(p200_delta_velocity_z_GET(pack) == (float)2.0462215E38F);
    assert(p200_delta_velocity_y_GET(pack) == (float) -6.456526E37F);
    assert(p200_delta_time_GET(pack) == (float)2.1533662E38F);
    assert(p200_delta_angle_y_GET(pack) == (float) -1.2333405E38F);
    assert(p200_joint_az_GET(pack) == (float) -1.767394E38F);
    assert(p200_target_system_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p200_joint_roll_GET(pack) == (float) -3.3034052E38F);
};


void c_LoopBackDemoChannel_on_GIMBAL_CONTROL_201(Bounds_Inside * ph, Pack * pack)
{
    assert(p201_demanded_rate_x_GET(pack) == (float)2.6927653E38F);
    assert(p201_demanded_rate_y_GET(pack) == (float)3.197891E38F);
    assert(p201_demanded_rate_z_GET(pack) == (float) -5.561861E37F);
    assert(p201_target_system_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p201_target_component_GET(pack) == (uint8_t)(uint8_t)231);
};


void c_LoopBackDemoChannel_on_GIMBAL_TORQUE_CMD_REPORT_214(Bounds_Inside * ph, Pack * pack)
{
    assert(p214_target_component_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p214_az_torque_cmd_GET(pack) == (int16_t)(int16_t) -8600);
    assert(p214_target_system_GET(pack) == (uint8_t)(uint8_t)113);
    assert(p214_rl_torque_cmd_GET(pack) == (int16_t)(int16_t)16847);
    assert(p214_el_torque_cmd_GET(pack) == (int16_t)(int16_t) -291);
};


void c_LoopBackDemoChannel_on_GOPRO_HEARTBEAT_215(Bounds_Inside * ph, Pack * pack)
{
    assert(p215_flags_GET(pack) == e_GOPRO_HEARTBEAT_FLAGS_GOPRO_FLAG_RECORDING);
    assert(p215_status_GET(pack) == e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_CONNECTED);
    assert(p215_capture_mode_GET(pack) == e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_VIDEO);
};


void c_LoopBackDemoChannel_on_GOPRO_GET_REQUEST_216(Bounds_Inside * ph, Pack * pack)
{
    assert(p216_target_component_GET(pack) == (uint8_t)(uint8_t)180);
    assert(p216_target_system_GET(pack) == (uint8_t)(uint8_t)111);
    assert(p216_cmd_id_GET(pack) == e_GOPRO_COMMAND_GOPRO_COMMAND_MODEL);
};


void c_LoopBackDemoChannel_on_GOPRO_GET_RESPONSE_217(Bounds_Inside * ph, Pack * pack)
{
    assert(p217_cmd_id_GET(pack) == e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE_EXPOSURE);
    assert(p217_status_GET(pack) == e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS);
    {
        uint8_t exemplary[] =  {(uint8_t)213, (uint8_t)95, (uint8_t)244, (uint8_t)204} ;
        uint8_t*  sample = p217_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_GOPRO_SET_REQUEST_218(Bounds_Inside * ph, Pack * pack)
{
    assert(p218_cmd_id_GET(pack) == e_GOPRO_COMMAND_GOPRO_COMMAND_LOW_LIGHT);
    assert(p218_target_system_GET(pack) == (uint8_t)(uint8_t)17);
    {
        uint8_t exemplary[] =  {(uint8_t)138, (uint8_t)128, (uint8_t)8, (uint8_t)34} ;
        uint8_t*  sample = p218_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p218_target_component_GET(pack) == (uint8_t)(uint8_t)90);
};


void c_LoopBackDemoChannel_on_GOPRO_SET_RESPONSE_219(Bounds_Inside * ph, Pack * pack)
{
    assert(p219_cmd_id_GET(pack) == e_GOPRO_COMMAND_GOPRO_COMMAND_CHARGING);
    assert(p219_status_GET(pack) == e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS);
};


void c_LoopBackDemoChannel_on_RPM_226(Bounds_Inside * ph, Pack * pack)
{
    assert(p226_rpm1_GET(pack) == (float)2.4958253E37F);
    assert(p226_rpm2_GET(pack) == (float)5.787931E37F);
};


void c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_pos_horiz_accuracy_GET(pack) == (float) -2.6168369E38F);
    assert(p230_tas_ratio_GET(pack) == (float) -3.5839938E37F);
    assert(p230_time_usec_GET(pack) == (uint64_t)6566160794537745771L);
    assert(p230_pos_horiz_ratio_GET(pack) == (float) -2.4435578E38F);
    assert(p230_mag_ratio_GET(pack) == (float)3.078284E38F);
    assert(p230_hagl_ratio_GET(pack) == (float) -8.286331E37F);
    assert(p230_pos_vert_accuracy_GET(pack) == (float)3.8424503E37F);
    assert(p230_pos_vert_ratio_GET(pack) == (float) -3.225713E38F);
    assert(p230_vel_ratio_GET(pack) == (float)1.1329453E38F);
    assert(p230_flags_GET(pack) == e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE);
};


void c_LoopBackDemoChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_horiz_accuracy_GET(pack) == (float)2.1439427E37F);
    assert(p231_wind_z_GET(pack) == (float) -4.8913343E37F);
    assert(p231_wind_alt_GET(pack) == (float) -1.5742209E38F);
    assert(p231_wind_x_GET(pack) == (float) -1.1693875E38F);
    assert(p231_time_usec_GET(pack) == (uint64_t)1632632645745052918L);
    assert(p231_vert_accuracy_GET(pack) == (float) -1.3564092E38F);
    assert(p231_var_vert_GET(pack) == (float)6.7309737E37F);
    assert(p231_wind_y_GET(pack) == (float)1.9179276E38F);
    assert(p231_var_horiz_GET(pack) == (float) -1.2457691E38F);
};


void c_LoopBackDemoChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_hdop_GET(pack) == (float) -1.879019E38F);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)199);
    assert(p232_speed_accuracy_GET(pack) == (float)2.7299513E38F);
    assert(p232_horiz_accuracy_GET(pack) == (float)1.6460375E38F);
    assert(p232_vd_GET(pack) == (float) -2.2507297E38F);
    assert(p232_vert_accuracy_GET(pack) == (float)6.7156067E37F);
    assert(p232_alt_GET(pack) == (float) -3.3129977E38F);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)659730492L);
    assert(p232_vn_GET(pack) == (float)4.7901033E37F);
    assert(p232_ignore_flags_GET(pack) == e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP);
    assert(p232_ve_GET(pack) == (float) -1.5670992E38F);
    assert(p232_vdop_GET(pack) == (float)1.2685448E38F);
    assert(p232_time_usec_GET(pack) == (uint64_t)234275877406942109L);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p232_lon_GET(pack) == (int32_t)573024533);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)189);
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)59543);
    assert(p232_lat_GET(pack) == (int32_t)317830732);
};


void c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)216);
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)204);
    {
        uint8_t exemplary[] =  {(uint8_t)188, (uint8_t)144, (uint8_t)145, (uint8_t)79, (uint8_t)223, (uint8_t)216, (uint8_t)248, (uint8_t)16, (uint8_t)209, (uint8_t)228, (uint8_t)133, (uint8_t)125, (uint8_t)63, (uint8_t)231, (uint8_t)47, (uint8_t)244, (uint8_t)21, (uint8_t)242, (uint8_t)155, (uint8_t)147, (uint8_t)225, (uint8_t)209, (uint8_t)205, (uint8_t)2, (uint8_t)27, (uint8_t)50, (uint8_t)160, (uint8_t)94, (uint8_t)114, (uint8_t)197, (uint8_t)89, (uint8_t)164, (uint8_t)240, (uint8_t)51, (uint8_t)30, (uint8_t)50, (uint8_t)133, (uint8_t)82, (uint8_t)144, (uint8_t)115, (uint8_t)191, (uint8_t)198, (uint8_t)189, (uint8_t)219, (uint8_t)30, (uint8_t)230, (uint8_t)13, (uint8_t)54, (uint8_t)189, (uint8_t)227, (uint8_t)231, (uint8_t)197, (uint8_t)146, (uint8_t)167, (uint8_t)182, (uint8_t)132, (uint8_t)23, (uint8_t)114, (uint8_t)125, (uint8_t)41, (uint8_t)38, (uint8_t)132, (uint8_t)180, (uint8_t)187, (uint8_t)129, (uint8_t)37, (uint8_t)134, (uint8_t)198, (uint8_t)118, (uint8_t)134, (uint8_t)16, (uint8_t)247, (uint8_t)77, (uint8_t)121, (uint8_t)72, (uint8_t)174, (uint8_t)107, (uint8_t)69, (uint8_t)114, (uint8_t)200, (uint8_t)193, (uint8_t)123, (uint8_t)215, (uint8_t)43, (uint8_t)19, (uint8_t)227, (uint8_t)21, (uint8_t)147, (uint8_t)70, (uint8_t)6, (uint8_t)211, (uint8_t)196, (uint8_t)34, (uint8_t)114, (uint8_t)47, (uint8_t)159, (uint8_t)255, (uint8_t)104, (uint8_t)197, (uint8_t)254, (uint8_t)106, (uint8_t)190, (uint8_t)216, (uint8_t)232, (uint8_t)139, (uint8_t)19, (uint8_t)202, (uint8_t)94, (uint8_t)201, (uint8_t)125, (uint8_t)246, (uint8_t)177, (uint8_t)59, (uint8_t)87, (uint8_t)187, (uint8_t)77, (uint8_t)135, (uint8_t)70, (uint8_t)7, (uint8_t)41, (uint8_t)228, (uint8_t)250, (uint8_t)154, (uint8_t)43, (uint8_t)40, (uint8_t)223, (uint8_t)249, (uint8_t)77, (uint8_t)21, (uint8_t)241, (uint8_t)222, (uint8_t)189, (uint8_t)99, (uint8_t)174, (uint8_t)197, (uint8_t)194, (uint8_t)14, (uint8_t)249, (uint8_t)226, (uint8_t)20, (uint8_t)31, (uint8_t)250, (uint8_t)183, (uint8_t)43, (uint8_t)31, (uint8_t)96, (uint8_t)0, (uint8_t)132, (uint8_t)125, (uint8_t)26, (uint8_t)61, (uint8_t)231, (uint8_t)223, (uint8_t)169, (uint8_t)58, (uint8_t)216, (uint8_t)117, (uint8_t)71, (uint8_t)202, (uint8_t)206, (uint8_t)182, (uint8_t)17, (uint8_t)99, (uint8_t)7, (uint8_t)27, (uint8_t)126, (uint8_t)4, (uint8_t)4, (uint8_t)3, (uint8_t)205, (uint8_t)45, (uint8_t)153, (uint8_t)195, (uint8_t)227, (uint8_t)97, (uint8_t)175, (uint8_t)211, (uint8_t)116, (uint8_t)182, (uint8_t)137} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)64);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)76);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t) -61);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t)12708);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t) -31266);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t)22934);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t)25829);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)23393);
    assert(p234_latitude_GET(pack) == (int32_t) -1070663787);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)105);
    assert(p234_custom_mode_GET(pack) == (uint32_t)844053704L);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t) -59);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t) -119);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS);
    assert(p234_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t) -14348);
    assert(p234_longitude_GET(pack) == (int32_t) -1678887264);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)58);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)21577);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t) -58);
};


void c_LoopBackDemoChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_clipping_2_GET(pack) == (uint32_t)3790782853L);
    assert(p241_time_usec_GET(pack) == (uint64_t)6337791997861692261L);
    assert(p241_vibration_z_GET(pack) == (float)3.3642585E38F);
    assert(p241_vibration_x_GET(pack) == (float) -7.302798E37F);
    assert(p241_vibration_y_GET(pack) == (float) -2.4237972E38F);
    assert(p241_clipping_1_GET(pack) == (uint32_t)3070810146L);
    assert(p241_clipping_0_GET(pack) == (uint32_t)1016481609L);
};


void c_LoopBackDemoChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_x_GET(pack) == (float)2.740982E38F);
    assert(p242_approach_y_GET(pack) == (float) -2.8006787E38F);
    assert(p242_approach_x_GET(pack) == (float) -1.02680706E37F);
    assert(p242_longitude_GET(pack) == (int32_t)992323133);
    assert(p242_latitude_GET(pack) == (int32_t) -410021199);
    assert(p242_y_GET(pack) == (float)2.457572E37F);
    assert(p242_z_GET(pack) == (float)1.9928486E38F);
    assert(p242_altitude_GET(pack) == (int32_t)1464979421);
    assert(p242_time_usec_TRY(ph) == (uint64_t)8564977927090414023L);
    {
        float exemplary[] =  {1.1752303E38F, -1.8741703E38F, 4.426819E37F, 8.521899E37F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_approach_z_GET(pack) == (float)7.4888574E37F);
};


void c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_longitude_GET(pack) == (int32_t) -1381449306);
    assert(p243_time_usec_TRY(ph) == (uint64_t)8503561173903668787L);
    assert(p243_altitude_GET(pack) == (int32_t) -711633390);
    assert(p243_y_GET(pack) == (float) -2.3047253E38F);
    {
        float exemplary[] =  {2.0803254E38F, -5.998711E36F, 1.0976029E38F, 1.1206643E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_x_GET(pack) == (float) -3.1241395E38F);
    assert(p243_approach_z_GET(pack) == (float) -7.8850413E37F);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)230);
    assert(p243_approach_y_GET(pack) == (float)2.9522305E38F);
    assert(p243_z_GET(pack) == (float)1.62176E38F);
    assert(p243_approach_x_GET(pack) == (float)7.9038583E37F);
    assert(p243_latitude_GET(pack) == (int32_t)1684008788);
};


void c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)4984);
    assert(p244_interval_us_GET(pack) == (int32_t) -1204877428);
};


void c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_MC);
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND);
};


void c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_altitude_GET(pack) == (int32_t)1643306639);
    assert(p246_lat_GET(pack) == (int32_t) -330783888);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)30793);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_SERVICE_SURFACE);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)27852);
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)14);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC);
    assert(p246_lon_GET(pack) == (int32_t)1553951816);
    assert(p246_flags_GET(pack) == e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK);
    assert(p246_callsign_LEN(ph) == 8);
    {
        char16_t * exemplary = u"ooistezy";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_ICAO_address_GET(pack) == (uint32_t)1721914629L);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)62652);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t)12898);
};


void c_LoopBackDemoChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_id_GET(pack) == (uint32_t)4079285685L);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_NONE);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB);
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float)1.3347511E38F);
    assert(p247_altitude_minimum_delta_GET(pack) == (float) -2.9769393E37F);
    assert(p247_time_to_minimum_delta_GET(pack) == (float) -1.8626663E38F);
};


void c_LoopBackDemoChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)57228);
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)139);
    {
        uint8_t exemplary[] =  {(uint8_t)160, (uint8_t)175, (uint8_t)168, (uint8_t)0, (uint8_t)7, (uint8_t)57, (uint8_t)148, (uint8_t)11, (uint8_t)134, (uint8_t)18, (uint8_t)6, (uint8_t)226, (uint8_t)96, (uint8_t)72, (uint8_t)125, (uint8_t)156, (uint8_t)39, (uint8_t)73, (uint8_t)8, (uint8_t)142, (uint8_t)144, (uint8_t)245, (uint8_t)31, (uint8_t)34, (uint8_t)71, (uint8_t)222, (uint8_t)180, (uint8_t)118, (uint8_t)2, (uint8_t)233, (uint8_t)113, (uint8_t)69, (uint8_t)37, (uint8_t)99, (uint8_t)91, (uint8_t)86, (uint8_t)232, (uint8_t)232, (uint8_t)165, (uint8_t)23, (uint8_t)14, (uint8_t)247, (uint8_t)241, (uint8_t)81, (uint8_t)196, (uint8_t)87, (uint8_t)188, (uint8_t)9, (uint8_t)150, (uint8_t)112, (uint8_t)230, (uint8_t)106, (uint8_t)225, (uint8_t)154, (uint8_t)50, (uint8_t)252, (uint8_t)30, (uint8_t)217, (uint8_t)31, (uint8_t)174, (uint8_t)156, (uint8_t)51, (uint8_t)179, (uint8_t)126, (uint8_t)6, (uint8_t)34, (uint8_t)201, (uint8_t)169, (uint8_t)91, (uint8_t)143, (uint8_t)253, (uint8_t)97, (uint8_t)81, (uint8_t)154, (uint8_t)75, (uint8_t)250, (uint8_t)6, (uint8_t)198, (uint8_t)70, (uint8_t)125, (uint8_t)253, (uint8_t)180, (uint8_t)176, (uint8_t)2, (uint8_t)196, (uint8_t)86, (uint8_t)22, (uint8_t)250, (uint8_t)211, (uint8_t)152, (uint8_t)227, (uint8_t)169, (uint8_t)163, (uint8_t)139, (uint8_t)47, (uint8_t)165, (uint8_t)171, (uint8_t)113, (uint8_t)100, (uint8_t)117, (uint8_t)151, (uint8_t)231, (uint8_t)35, (uint8_t)10, (uint8_t)128, (uint8_t)19, (uint8_t)38, (uint8_t)49, (uint8_t)182, (uint8_t)210, (uint8_t)174, (uint8_t)72, (uint8_t)178, (uint8_t)91, (uint8_t)212, (uint8_t)38, (uint8_t)202, (uint8_t)144, (uint8_t)150, (uint8_t)204, (uint8_t)88, (uint8_t)206, (uint8_t)141, (uint8_t)205, (uint8_t)24, (uint8_t)37, (uint8_t)214, (uint8_t)194, (uint8_t)28, (uint8_t)177, (uint8_t)180, (uint8_t)14, (uint8_t)135, (uint8_t)42, (uint8_t)129, (uint8_t)111, (uint8_t)232, (uint8_t)204, (uint8_t)49, (uint8_t)147, (uint8_t)148, (uint8_t)37, (uint8_t)5, (uint8_t)206, (uint8_t)199, (uint8_t)216, (uint8_t)162, (uint8_t)55, (uint8_t)242, (uint8_t)240, (uint8_t)139, (uint8_t)123, (uint8_t)164, (uint8_t)207, (uint8_t)178, (uint8_t)226, (uint8_t)158, (uint8_t)149, (uint8_t)227, (uint8_t)83, (uint8_t)15, (uint8_t)124, (uint8_t)6, (uint8_t)125, (uint8_t)60, (uint8_t)177, (uint8_t)157, (uint8_t)104, (uint8_t)125, (uint8_t)222, (uint8_t)243, (uint8_t)23, (uint8_t)169, (uint8_t)136, (uint8_t)234, (uint8_t)212, (uint8_t)68, (uint8_t)71, (uint8_t)8, (uint8_t)83, (uint8_t)160, (uint8_t)236, (uint8_t)96, (uint8_t)182, (uint8_t)178, (uint8_t)240, (uint8_t)159, (uint8_t)86, (uint8_t)240, (uint8_t)33, (uint8_t)205, (uint8_t)107, (uint8_t)214, (uint8_t)229, (uint8_t)105, (uint8_t)34, (uint8_t)140, (uint8_t)12, (uint8_t)23, (uint8_t)245, (uint8_t)104, (uint8_t)247, (uint8_t)156, (uint8_t)159, (uint8_t)225, (uint8_t)220, (uint8_t)59, (uint8_t)192, (uint8_t)195, (uint8_t)152, (uint8_t)246, (uint8_t)90, (uint8_t)34, (uint8_t)64, (uint8_t)254, (uint8_t)18, (uint8_t)130, (uint8_t)73, (uint8_t)37, (uint8_t)249, (uint8_t)80, (uint8_t)54, (uint8_t)96, (uint8_t)88, (uint8_t)41, (uint8_t)193, (uint8_t)249, (uint8_t)173, (uint8_t)11, (uint8_t)252, (uint8_t)205, (uint8_t)157, (uint8_t)247, (uint8_t)111, (uint8_t)202, (uint8_t)152, (uint8_t)251, (uint8_t)3, (uint8_t)62, (uint8_t)45, (uint8_t)142, (uint8_t)31, (uint8_t)159, (uint8_t)58, (uint8_t)229, (uint8_t)255, (uint8_t)127, (uint8_t)30, (uint8_t)21} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)40);
};


void c_LoopBackDemoChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    {
        int8_t exemplary[] =  {(int8_t) -61, (int8_t)122, (int8_t)26, (int8_t)112, (int8_t)20, (int8_t) -96, (int8_t)55, (int8_t) -87, (int8_t)10, (int8_t)76, (int8_t) -122, (int8_t)120, (int8_t) -55, (int8_t)105, (int8_t) -33, (int8_t) -35, (int8_t)6, (int8_t)71, (int8_t)72, (int8_t) -34, (int8_t) -55, (int8_t) -18, (int8_t)111, (int8_t)109, (int8_t)125, (int8_t) -44, (int8_t)13, (int8_t)30, (int8_t)97, (int8_t) -45, (int8_t)34, (int8_t)84} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)63);
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)11199);
};


void c_LoopBackDemoChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_name_LEN(ph) == 5);
    {
        char16_t * exemplary = u"ngllc";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p250_time_usec_GET(pack) == (uint64_t)3075084406157923064L);
    assert(p250_z_GET(pack) == (float)2.5154092E38F);
    assert(p250_y_GET(pack) == (float) -1.5848465E38F);
    assert(p250_x_GET(pack) == (float)2.6151695E38F);
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)3703266438L);
    assert(p251_value_GET(pack) == (float) -1.2046135E38F);
    assert(p251_name_LEN(ph) == 2);
    {
        char16_t * exemplary = u"ju";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_value_GET(pack) == (int32_t) -590492140);
    assert(p252_name_LEN(ph) == 10);
    {
        char16_t * exemplary = u"edmjWpsrlp";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)1812613599L);
};


void c_LoopBackDemoChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_DEBUG);
    assert(p253_text_LEN(ph) == 5);
    {
        char16_t * exemplary = u"rxpet";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)1676449526L);
    assert(p254_value_GET(pack) == (float) -1.825317E38F);
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)180);
};


void c_LoopBackDemoChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)1237436282984357039L);
    {
        uint8_t exemplary[] =  {(uint8_t)216, (uint8_t)29, (uint8_t)186, (uint8_t)12, (uint8_t)13, (uint8_t)18, (uint8_t)122, (uint8_t)114, (uint8_t)36, (uint8_t)111, (uint8_t)230, (uint8_t)210, (uint8_t)184, (uint8_t)127, (uint8_t)124, (uint8_t)60, (uint8_t)231, (uint8_t)172, (uint8_t)72, (uint8_t)30, (uint8_t)16, (uint8_t)97, (uint8_t)86, (uint8_t)226, (uint8_t)96, (uint8_t)183, (uint8_t)248, (uint8_t)243, (uint8_t)60, (uint8_t)88, (uint8_t)141, (uint8_t)168} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)171);
};


void c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_last_change_ms_GET(pack) == (uint32_t)3594042759L);
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)1985759935L);
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)212);
};


void c_LoopBackDemoChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_tune_LEN(ph) == 18);
    {
        char16_t * exemplary = u"bXLhKeFdcdtfuMcnfx";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)147);
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)145);
};


void c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)115, (uint8_t)232, (uint8_t)241, (uint8_t)188, (uint8_t)152, (uint8_t)224, (uint8_t)210, (uint8_t)116, (uint8_t)78, (uint8_t)190, (uint8_t)32, (uint8_t)209, (uint8_t)254, (uint8_t)54, (uint8_t)162, (uint8_t)150, (uint8_t)252, (uint8_t)100, (uint8_t)4, (uint8_t)76, (uint8_t)199, (uint8_t)153, (uint8_t)222, (uint8_t)209, (uint8_t)159, (uint8_t)89, (uint8_t)111, (uint8_t)95, (uint8_t)70, (uint8_t)41, (uint8_t)254, (uint8_t)101} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)37100);
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)890);
    assert(p259_flags_GET(pack) == e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)57256);
    assert(p259_sensor_size_h_GET(pack) == (float) -1.4896115E38F);
    assert(p259_focal_length_GET(pack) == (float) -1.6390109E38F);
    {
        uint8_t exemplary[] =  {(uint8_t)145, (uint8_t)78, (uint8_t)131, (uint8_t)211, (uint8_t)35, (uint8_t)255, (uint8_t)121, (uint8_t)63, (uint8_t)9, (uint8_t)10, (uint8_t)28, (uint8_t)181, (uint8_t)87, (uint8_t)42, (uint8_t)21, (uint8_t)171, (uint8_t)248, (uint8_t)128, (uint8_t)131, (uint8_t)37, (uint8_t)82, (uint8_t)34, (uint8_t)99, (uint8_t)49, (uint8_t)101, (uint8_t)231, (uint8_t)174, (uint8_t)242, (uint8_t)34, (uint8_t)24, (uint8_t)249, (uint8_t)240} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)2941703693L);
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)129);
    assert(p259_firmware_version_GET(pack) == (uint32_t)1331807499L);
    assert(p259_cam_definition_uri_LEN(ph) == 113);
    {
        char16_t * exemplary = u"yrqTerxepdsnluuynddeergjAcutsylbbctgbsspayLogkeauWymzbsCppviVYykdytqomxqyfbwisxwVeieyZTnTxBorvlgqjynzEqwzacgzhooq";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 226);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_sensor_size_v_GET(pack) == (float)1.2099008E36F);
};


void c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_IMAGE);
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)519995461L);
};


void c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)121);
    assert(p261_available_capacity_GET(pack) == (float) -2.7072228E38F);
    assert(p261_write_speed_GET(pack) == (float)5.916462E37F);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)2550129966L);
    assert(p261_used_capacity_GET(pack) == (float) -7.621143E37F);
    assert(p261_total_capacity_GET(pack) == (float) -5.5119623E37F);
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)112);
    assert(p261_read_speed_GET(pack) == (float) -2.0171698E38F);
};


void c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)9);
    assert(p262_available_capacity_GET(pack) == (float)1.1875125E38F);
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)2959207391L);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p262_image_interval_GET(pack) == (float)2.5692992E38F);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)3750280262L);
};


void c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_alt_GET(pack) == (int32_t) -2075375779);
    assert(p263_relative_alt_GET(pack) == (int32_t) -1532092643);
    assert(p263_time_utc_GET(pack) == (uint64_t)6883171120879632120L);
    assert(p263_file_url_LEN(ph) == 201);
    {
        char16_t * exemplary = u"fpfxZphqItzjejmddviHzoqoidbsIQsruxnzkisWuwoqimCthondskirwhzneWdMSvcPowbrxjhXcxvkiqbhiNedjsswsewotvsbzotiyamsyAykemoyipKirppmyfsofvctzpjlijfmoryWwuqFtvhnuAuekquNdVsftuxpsqODtafgoncbQemzyoxPcalzzrdxjuWzg";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 402);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)3582287015L);
    assert(p263_image_index_GET(pack) == (int32_t)1602444087);
    assert(p263_lon_GET(pack) == (int32_t) -1610112896);
    {
        float exemplary[] =  {-3.7829374E37F, -3.0871349E38F, 2.3994435E38F, -3.1774388E38F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t) -86);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p263_lat_GET(pack) == (int32_t) -2141530489);
};


void c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_flight_uuid_GET(pack) == (uint64_t)8928180109364118318L);
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)7256928325321178409L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)2524230066473017737L);
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)2036326861L);
};


void c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_yaw_GET(pack) == (float)5.1389577E37F);
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)2997417993L);
    assert(p265_roll_GET(pack) == (float)1.5182469E38F);
    assert(p265_pitch_GET(pack) == (float)2.6662027E38F);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)209);
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)74);
    {
        uint8_t exemplary[] =  {(uint8_t)62, (uint8_t)186, (uint8_t)141, (uint8_t)53, (uint8_t)145, (uint8_t)10, (uint8_t)16, (uint8_t)130, (uint8_t)218, (uint8_t)129, (uint8_t)50, (uint8_t)67, (uint8_t)58, (uint8_t)216, (uint8_t)6, (uint8_t)240, (uint8_t)90, (uint8_t)173, (uint8_t)90, (uint8_t)57, (uint8_t)184, (uint8_t)224, (uint8_t)93, (uint8_t)63, (uint8_t)243, (uint8_t)72, (uint8_t)248, (uint8_t)81, (uint8_t)43, (uint8_t)169, (uint8_t)5, (uint8_t)54, (uint8_t)181, (uint8_t)182, (uint8_t)100, (uint8_t)144, (uint8_t)138, (uint8_t)238, (uint8_t)0, (uint8_t)153, (uint8_t)178, (uint8_t)243, (uint8_t)3, (uint8_t)117, (uint8_t)232, (uint8_t)190, (uint8_t)18, (uint8_t)50, (uint8_t)48, (uint8_t)130, (uint8_t)222, (uint8_t)149, (uint8_t)113, (uint8_t)49, (uint8_t)103, (uint8_t)144, (uint8_t)37, (uint8_t)145, (uint8_t)185, (uint8_t)89, (uint8_t)14, (uint8_t)98, (uint8_t)153, (uint8_t)146, (uint8_t)102, (uint8_t)62, (uint8_t)139, (uint8_t)178, (uint8_t)153, (uint8_t)210, (uint8_t)171, (uint8_t)39, (uint8_t)163, (uint8_t)237, (uint8_t)11, (uint8_t)228, (uint8_t)106, (uint8_t)120, (uint8_t)115, (uint8_t)61, (uint8_t)73, (uint8_t)208, (uint8_t)194, (uint8_t)63, (uint8_t)168, (uint8_t)134, (uint8_t)255, (uint8_t)169, (uint8_t)85, (uint8_t)205, (uint8_t)41, (uint8_t)10, (uint8_t)180, (uint8_t)182, (uint8_t)47, (uint8_t)78, (uint8_t)63, (uint8_t)126, (uint8_t)128, (uint8_t)199, (uint8_t)35, (uint8_t)97, (uint8_t)192, (uint8_t)56, (uint8_t)95, (uint8_t)129, (uint8_t)226, (uint8_t)75, (uint8_t)67, (uint8_t)20, (uint8_t)16, (uint8_t)167, (uint8_t)113, (uint8_t)251, (uint8_t)233, (uint8_t)109, (uint8_t)253, (uint8_t)186, (uint8_t)16, (uint8_t)72, (uint8_t)98, (uint8_t)79, (uint8_t)105, (uint8_t)138, (uint8_t)60, (uint8_t)246, (uint8_t)247, (uint8_t)8, (uint8_t)206, (uint8_t)172, (uint8_t)200, (uint8_t)155, (uint8_t)220, (uint8_t)103, (uint8_t)123, (uint8_t)22, (uint8_t)244, (uint8_t)27, (uint8_t)199, (uint8_t)13, (uint8_t)16, (uint8_t)79, (uint8_t)70, (uint8_t)29, (uint8_t)249, (uint8_t)230, (uint8_t)214, (uint8_t)122, (uint8_t)68, (uint8_t)92, (uint8_t)232, (uint8_t)109, (uint8_t)119, (uint8_t)173, (uint8_t)192, (uint8_t)243, (uint8_t)7, (uint8_t)71, (uint8_t)154, (uint8_t)148, (uint8_t)217, (uint8_t)126, (uint8_t)189, (uint8_t)239, (uint8_t)44, (uint8_t)53, (uint8_t)9, (uint8_t)189, (uint8_t)153, (uint8_t)194, (uint8_t)84, (uint8_t)165, (uint8_t)120, (uint8_t)137, (uint8_t)223, (uint8_t)52, (uint8_t)212, (uint8_t)241, (uint8_t)213, (uint8_t)251, (uint8_t)55, (uint8_t)210, (uint8_t)161, (uint8_t)147, (uint8_t)84, (uint8_t)8, (uint8_t)209, (uint8_t)250, (uint8_t)157, (uint8_t)35, (uint8_t)165, (uint8_t)95, (uint8_t)178, (uint8_t)129, (uint8_t)188, (uint8_t)109, (uint8_t)136, (uint8_t)126, (uint8_t)24, (uint8_t)202, (uint8_t)221, (uint8_t)125, (uint8_t)246, (uint8_t)251, (uint8_t)90, (uint8_t)60, (uint8_t)155, (uint8_t)204, (uint8_t)96, (uint8_t)99, (uint8_t)77, (uint8_t)140, (uint8_t)247, (uint8_t)125, (uint8_t)247, (uint8_t)201, (uint8_t)34, (uint8_t)145, (uint8_t)12, (uint8_t)167, (uint8_t)231, (uint8_t)173, (uint8_t)102, (uint8_t)97, (uint8_t)114, (uint8_t)1, (uint8_t)242, (uint8_t)236, (uint8_t)181, (uint8_t)14, (uint8_t)83, (uint8_t)212, (uint8_t)191, (uint8_t)237, (uint8_t)159, (uint8_t)23, (uint8_t)97, (uint8_t)75, (uint8_t)144, (uint8_t)255, (uint8_t)43, (uint8_t)64, (uint8_t)141, (uint8_t)29, (uint8_t)248, (uint8_t)128, (uint8_t)229, (uint8_t)58, (uint8_t)77} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)5005);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)37);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)64693);
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)87);
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)103);
    {
        uint8_t exemplary[] =  {(uint8_t)48, (uint8_t)40, (uint8_t)235, (uint8_t)150, (uint8_t)114, (uint8_t)163, (uint8_t)113, (uint8_t)81, (uint8_t)199, (uint8_t)56, (uint8_t)100, (uint8_t)220, (uint8_t)38, (uint8_t)194, (uint8_t)29, (uint8_t)73, (uint8_t)200, (uint8_t)152, (uint8_t)173, (uint8_t)6, (uint8_t)194, (uint8_t)237, (uint8_t)208, (uint8_t)255, (uint8_t)68, (uint8_t)191, (uint8_t)104, (uint8_t)165, (uint8_t)137, (uint8_t)210, (uint8_t)58, (uint8_t)97, (uint8_t)209, (uint8_t)49, (uint8_t)216, (uint8_t)150, (uint8_t)228, (uint8_t)190, (uint8_t)128, (uint8_t)62, (uint8_t)0, (uint8_t)150, (uint8_t)23, (uint8_t)59, (uint8_t)224, (uint8_t)89, (uint8_t)101, (uint8_t)251, (uint8_t)179, (uint8_t)230, (uint8_t)167, (uint8_t)129, (uint8_t)197, (uint8_t)148, (uint8_t)70, (uint8_t)60, (uint8_t)43, (uint8_t)253, (uint8_t)128, (uint8_t)201, (uint8_t)157, (uint8_t)93, (uint8_t)78, (uint8_t)129, (uint8_t)174, (uint8_t)66, (uint8_t)239, (uint8_t)209, (uint8_t)156, (uint8_t)152, (uint8_t)108, (uint8_t)199, (uint8_t)193, (uint8_t)74, (uint8_t)85, (uint8_t)121, (uint8_t)31, (uint8_t)173, (uint8_t)203, (uint8_t)136, (uint8_t)218, (uint8_t)116, (uint8_t)36, (uint8_t)141, (uint8_t)110, (uint8_t)161, (uint8_t)126, (uint8_t)14, (uint8_t)10, (uint8_t)214, (uint8_t)135, (uint8_t)252, (uint8_t)206, (uint8_t)255, (uint8_t)74, (uint8_t)172, (uint8_t)196, (uint8_t)200, (uint8_t)77, (uint8_t)194, (uint8_t)182, (uint8_t)60, (uint8_t)37, (uint8_t)52, (uint8_t)140, (uint8_t)48, (uint8_t)96, (uint8_t)74, (uint8_t)44, (uint8_t)146, (uint8_t)202, (uint8_t)223, (uint8_t)51, (uint8_t)209, (uint8_t)9, (uint8_t)4, (uint8_t)167, (uint8_t)93, (uint8_t)202, (uint8_t)188, (uint8_t)77, (uint8_t)28, (uint8_t)103, (uint8_t)27, (uint8_t)123, (uint8_t)8, (uint8_t)118, (uint8_t)117, (uint8_t)164, (uint8_t)74, (uint8_t)87, (uint8_t)169, (uint8_t)254, (uint8_t)194, (uint8_t)156, (uint8_t)170, (uint8_t)53, (uint8_t)139, (uint8_t)179, (uint8_t)193, (uint8_t)169, (uint8_t)172, (uint8_t)4, (uint8_t)128, (uint8_t)129, (uint8_t)168, (uint8_t)116, (uint8_t)87, (uint8_t)76, (uint8_t)87, (uint8_t)61, (uint8_t)217, (uint8_t)183, (uint8_t)148, (uint8_t)194, (uint8_t)40, (uint8_t)186, (uint8_t)244, (uint8_t)51, (uint8_t)148, (uint8_t)30, (uint8_t)154, (uint8_t)159, (uint8_t)153, (uint8_t)83, (uint8_t)137, (uint8_t)45, (uint8_t)31, (uint8_t)126, (uint8_t)98, (uint8_t)15, (uint8_t)50, (uint8_t)88, (uint8_t)9, (uint8_t)65, (uint8_t)64, (uint8_t)119, (uint8_t)242, (uint8_t)48, (uint8_t)116, (uint8_t)249, (uint8_t)147, (uint8_t)23, (uint8_t)202, (uint8_t)64, (uint8_t)29, (uint8_t)238, (uint8_t)167, (uint8_t)217, (uint8_t)168, (uint8_t)239, (uint8_t)158, (uint8_t)119, (uint8_t)118, (uint8_t)229, (uint8_t)48, (uint8_t)102, (uint8_t)181, (uint8_t)66, (uint8_t)127, (uint8_t)43, (uint8_t)223, (uint8_t)150, (uint8_t)57, (uint8_t)117, (uint8_t)16, (uint8_t)44, (uint8_t)219, (uint8_t)70, (uint8_t)106, (uint8_t)236, (uint8_t)239, (uint8_t)126, (uint8_t)238, (uint8_t)242, (uint8_t)20, (uint8_t)250, (uint8_t)30, (uint8_t)19, (uint8_t)90, (uint8_t)216, (uint8_t)56, (uint8_t)203, (uint8_t)135, (uint8_t)125, (uint8_t)207, (uint8_t)150, (uint8_t)127, (uint8_t)13, (uint8_t)10, (uint8_t)0, (uint8_t)138, (uint8_t)55, (uint8_t)137, (uint8_t)221, (uint8_t)114, (uint8_t)170, (uint8_t)38, (uint8_t)187, (uint8_t)62, (uint8_t)77, (uint8_t)241, (uint8_t)80, (uint8_t)217, (uint8_t)85, (uint8_t)67, (uint8_t)20, (uint8_t)234, (uint8_t)45} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)145);
};


void c_LoopBackDemoChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)46751);
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)197);
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)239);
};


void c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)33565);
    assert(p269_uri_LEN(ph) == 127);
    {
        char16_t * exemplary = u"ilaltAtwsjuUvjotVmgcugmqswmaeshhqkqlyeexhnzvflkgbmxlptfttoiwryjzfjzyigrxelwxkwrjyizMujigBlLujyhruupbyerttoqypgfdsxtafrcRzeidnfv";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 254);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_framerate_GET(pack) == (float) -1.8199994E38F);
    assert(p269_bitrate_GET(pack) == (uint32_t)1224112268L);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)217);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)228);
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)14841);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)54401);
};


void c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_framerate_GET(pack) == (float)7.969138E37F);
    assert(p270_bitrate_GET(pack) == (uint32_t)78162329L);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)26607);
    assert(p270_uri_LEN(ph) == 141);
    {
        char16_t * exemplary = u"cbaynnTiodmalyErsmpeiuyygWyqguhpnpouxtjtllVwibyIgTeepHvojpaMrrigsexjqskkbcrcCxmhfmftrjtTitjhezvfotiyecdosaxfuiiuvMhhycbfXegeusuybhbccwoqovitQ";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 282);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)340);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)25233);
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)69);
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)53);
};


void c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_ssid_LEN(ph) == 15);
    {
        char16_t * exemplary = u"pjcjPImhvimukwx";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_password_LEN(ph) == 58);
    {
        char16_t * exemplary = u"xmzNtjmdwtktqesesyzgmygnhuqcLaopngQyHykbtxmysmwTeKtovJdpjP";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 116);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)12084);
    {
        uint8_t exemplary[] =  {(uint8_t)27, (uint8_t)117, (uint8_t)94, (uint8_t)60, (uint8_t)215, (uint8_t)144, (uint8_t)203, (uint8_t)54} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)190, (uint8_t)70, (uint8_t)126, (uint8_t)67, (uint8_t)112, (uint8_t)12, (uint8_t)154, (uint8_t)150} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)33216);
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)1004);
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_uptime_sec_GET(pack) == (uint32_t)3270841763L);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)30);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR);
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)56979);
    assert(p310_time_usec_GET(pack) == (uint64_t)9022781072169948119L);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)222);
    {
        uint8_t exemplary[] =  {(uint8_t)190, (uint8_t)174, (uint8_t)130, (uint8_t)85, (uint8_t)141, (uint8_t)204, (uint8_t)157, (uint8_t)185, (uint8_t)230, (uint8_t)251, (uint8_t)140, (uint8_t)127, (uint8_t)168, (uint8_t)62, (uint8_t)73, (uint8_t)94} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)3422544570L);
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p311_uptime_sec_GET(pack) == (uint32_t)1467979133L);
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)134);
    assert(p311_time_usec_GET(pack) == (uint64_t)4368929917618989823L);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)167);
    assert(p311_name_LEN(ph) == 53);
    {
        char16_t * exemplary = u"eaFShltsruoswLrxkneeiwaSjZhwioxtcmWtkyjishhcznZdzwJlc";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 106);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_param_id_LEN(ph) == 15);
    {
        char16_t * exemplary = u"scmbgoomgzCkyki";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 30);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)84);
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t)26385);
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)33);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)63);
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)149);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_value_LEN(ph) == 123);
    {
        char16_t * exemplary = u"gRojflvmswqhgboerjzlqzoTOoofobgBblZjrtwngxOwPrascexplruwcugvkSpzcttuHjcgmjzhvwqhzyWbgknjoiqofdpgfwgibwkMZCrzghfnHjiTknuvdXl";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 246);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)38434);
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)31005);
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32);
    assert(p322_param_id_LEN(ph) == 2);
    {
        char16_t * exemplary = u"oc";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)95);
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32);
    assert(p323_param_id_LEN(ph) == 2);
    {
        char16_t * exemplary = u"gf";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)118);
    assert(p323_param_value_LEN(ph) == 122);
    {
        char16_t * exemplary = u"DvcfaqjtVVTCpwdjdGdgvsgocigqghlviqitRuwzemcffrVposgTmuibphvxhgfnusyaszmkroviwkapZwhZloqodLhozvaymnzlugirkdvoWpnhdljmnvvfrj";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 244);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32);
    assert(p324_param_id_LEN(ph) == 6);
    {
        char16_t * exemplary = u"zSslis";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_value_LEN(ph) == 103);
    {
        char16_t * exemplary = u"odPoGeptocycvonryjtbgwxrmhtvWxbLdukpjmwblNqddaFrppqyHptufDglpkZglqvOesFjgUsNprqwemoiohelkekgImxspiljxaj";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 206);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_ACCEPTED);
};


void c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)18);
    {
        uint16_t exemplary[] =  {(uint16_t)12731, (uint16_t)34786, (uint16_t)28028, (uint16_t)38597, (uint16_t)10089, (uint16_t)59843, (uint16_t)36581, (uint16_t)5266, (uint16_t)23594, (uint16_t)53632, (uint16_t)46111, (uint16_t)27042, (uint16_t)41816, (uint16_t)15032, (uint16_t)21306, (uint16_t)29794, (uint16_t)64363, (uint16_t)45337, (uint16_t)56077, (uint16_t)33592, (uint16_t)36222, (uint16_t)46881, (uint16_t)55688, (uint16_t)14223, (uint16_t)32834, (uint16_t)61090, (uint16_t)60147, (uint16_t)49153, (uint16_t)44005, (uint16_t)6920, (uint16_t)18620, (uint16_t)16077, (uint16_t)55969, (uint16_t)36570, (uint16_t)5341, (uint16_t)37035, (uint16_t)19013, (uint16_t)49248, (uint16_t)2095, (uint16_t)25531, (uint16_t)42889, (uint16_t)57341, (uint16_t)14115, (uint16_t)22219, (uint16_t)42394, (uint16_t)52735, (uint16_t)1013, (uint16_t)45312, (uint16_t)5235, (uint16_t)28643, (uint16_t)49766, (uint16_t)3860, (uint16_t)10780, (uint16_t)53670, (uint16_t)5682, (uint16_t)62646, (uint16_t)15633, (uint16_t)45427, (uint16_t)33910, (uint16_t)48965, (uint16_t)3265, (uint16_t)49637, (uint16_t)50928, (uint16_t)7475, (uint16_t)19766, (uint16_t)27698, (uint16_t)23564, (uint16_t)2416, (uint16_t)55405, (uint16_t)6752, (uint16_t)31361, (uint16_t)34594} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_time_usec_GET(pack) == (uint64_t)8269500430103756781L);
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED);
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)24944);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)25196);
};


void c_LoopBackDemoChannel_on_UAVIONIX_ADSB_OUT_CFG_10001(Bounds_Inside * ph, Pack * pack)
{
    assert(p10001_aircraftSize_GET(pack) == e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_39P5M);
    assert(p10001_gpsOffsetLat_GET(pack) == e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_2M);
    assert(p10001_ICAO_GET(pack) == (uint32_t)1771962925L);
    assert(p10001_callsign_LEN(ph) == 3);
    {
        char16_t * exemplary = u"nmo";
        char16_t * sample = p10001_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p10001_stallSpeed_GET(pack) == (uint16_t)(uint16_t)59939);
    assert(p10001_gpsOffsetLon_GET(pack) == e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR);
    assert(p10001_rfSelect_GET(pack) == e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY);
    assert(p10001_emitterType_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_GLIDER);
};


void c_LoopBackDemoChannel_on_UAVIONIX_ADSB_OUT_DYNAMIC_10002(Bounds_Inside * ph, Pack * pack)
{
    assert(p10002_accuracyVel_GET(pack) == (uint16_t)(uint16_t)37539);
    assert(p10002_gpsFix_GET(pack) == e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_RTK);
    assert(p10002_gpsLon_GET(pack) == (int32_t)234310433);
    assert(p10002_accuracyVert_GET(pack) == (uint16_t)(uint16_t)3188);
    assert(p10002_velVert_GET(pack) == (int16_t)(int16_t)13633);
    assert(p10002_velNS_GET(pack) == (int16_t)(int16_t) -5642);
    assert(p10002_accuracyHor_GET(pack) == (uint32_t)3007745439L);
    assert(p10002_gpsAlt_GET(pack) == (int32_t)1127721829);
    assert(p10002_state_GET(pack) == e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT);
    assert(p10002_numSats_GET(pack) == (uint8_t)(uint8_t)145);
    assert(p10002_utcTime_GET(pack) == (uint32_t)3072621348L);
    assert(p10002_VelEW_GET(pack) == (int16_t)(int16_t) -28122);
    assert(p10002_baroAltMSL_GET(pack) == (int32_t) -443676971);
    assert(p10002_gpsLat_GET(pack) == (int32_t)1783223206);
    assert(p10002_emergencyStatus_GET(pack) == e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_UNLAWFUL_INTERFERANCE_EMERGENCY);
    assert(p10002_squawk_GET(pack) == (uint16_t)(uint16_t)43361);
};


void c_LoopBackDemoChannel_on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_10003(Bounds_Inside * ph, Pack * pack)
{
    assert(p10003_rfHealth_GET(pack) == e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_FAIL_TX);
};


void c_LoopBackDemoChannel_on_DEVICE_OP_READ_11000(Bounds_Inside * ph, Pack * pack)
{
    assert(p11000_busname_LEN(ph) == 13);
    {
        char16_t * exemplary = u"bsXtcikpKpnxt";
        char16_t * sample = p11000_busname_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p11000_target_system_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p11000_request_id_GET(pack) == (uint32_t)618767686L);
    assert(p11000_regstart_GET(pack) == (uint8_t)(uint8_t)55);
    assert(p11000_target_component_GET(pack) == (uint8_t)(uint8_t)47);
    assert(p11000_count_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p11000_address_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p11000_bustype_GET(pack) == e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI);
    assert(p11000_bus_GET(pack) == (uint8_t)(uint8_t)129);
};


void c_LoopBackDemoChannel_on_DEVICE_OP_READ_REPLY_11001(Bounds_Inside * ph, Pack * pack)
{
    assert(p11001_request_id_GET(pack) == (uint32_t)1855621796L);
    assert(p11001_count_GET(pack) == (uint8_t)(uint8_t)246);
    assert(p11001_regstart_GET(pack) == (uint8_t)(uint8_t)187);
    {
        uint8_t exemplary[] =  {(uint8_t)243, (uint8_t)157, (uint8_t)88, (uint8_t)57, (uint8_t)218, (uint8_t)197, (uint8_t)104, (uint8_t)218, (uint8_t)38, (uint8_t)76, (uint8_t)119, (uint8_t)89, (uint8_t)177, (uint8_t)20, (uint8_t)150, (uint8_t)254, (uint8_t)66, (uint8_t)80, (uint8_t)63, (uint8_t)208, (uint8_t)89, (uint8_t)93, (uint8_t)54, (uint8_t)39, (uint8_t)211, (uint8_t)201, (uint8_t)229, (uint8_t)214, (uint8_t)167, (uint8_t)57, (uint8_t)179, (uint8_t)129, (uint8_t)48, (uint8_t)30, (uint8_t)15, (uint8_t)158, (uint8_t)205, (uint8_t)73, (uint8_t)191, (uint8_t)248, (uint8_t)169, (uint8_t)253, (uint8_t)92, (uint8_t)243, (uint8_t)92, (uint8_t)235, (uint8_t)202, (uint8_t)99, (uint8_t)46, (uint8_t)94, (uint8_t)61, (uint8_t)185, (uint8_t)31, (uint8_t)124, (uint8_t)220, (uint8_t)13, (uint8_t)217, (uint8_t)88, (uint8_t)15, (uint8_t)247, (uint8_t)54, (uint8_t)36, (uint8_t)198, (uint8_t)31, (uint8_t)249, (uint8_t)229, (uint8_t)61, (uint8_t)165, (uint8_t)158, (uint8_t)173, (uint8_t)159, (uint8_t)57, (uint8_t)81, (uint8_t)74, (uint8_t)100, (uint8_t)164, (uint8_t)230, (uint8_t)9, (uint8_t)29, (uint8_t)38, (uint8_t)230, (uint8_t)98, (uint8_t)226, (uint8_t)174, (uint8_t)214, (uint8_t)209, (uint8_t)225, (uint8_t)77, (uint8_t)57, (uint8_t)216, (uint8_t)166, (uint8_t)24, (uint8_t)126, (uint8_t)93, (uint8_t)233, (uint8_t)19, (uint8_t)128, (uint8_t)188, (uint8_t)194, (uint8_t)12, (uint8_t)135, (uint8_t)210, (uint8_t)215, (uint8_t)44, (uint8_t)71, (uint8_t)73, (uint8_t)50, (uint8_t)253, (uint8_t)213, (uint8_t)26, (uint8_t)130, (uint8_t)8, (uint8_t)176, (uint8_t)247, (uint8_t)252, (uint8_t)24, (uint8_t)226, (uint8_t)31, (uint8_t)209, (uint8_t)143, (uint8_t)251, (uint8_t)138, (uint8_t)152, (uint8_t)216, (uint8_t)100, (uint8_t)212, (uint8_t)197, (uint8_t)28} ;
        uint8_t*  sample = p11001_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 128);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p11001_result_GET(pack) == (uint8_t)(uint8_t)186);
};


void c_LoopBackDemoChannel_on_DEVICE_OP_WRITE_11002(Bounds_Inside * ph, Pack * pack)
{
    assert(p11002_regstart_GET(pack) == (uint8_t)(uint8_t)43);
    assert(p11002_request_id_GET(pack) == (uint32_t)1779613397L);
    assert(p11002_busname_LEN(ph) == 38);
    {
        char16_t * exemplary = u"njgqfuuyjfutokXzyvfczfoqdxRmzakqjuJseh";
        char16_t * sample = p11002_busname_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 76);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p11002_target_component_GET(pack) == (uint8_t)(uint8_t)207);
    assert(p11002_count_GET(pack) == (uint8_t)(uint8_t)248);
    assert(p11002_bustype_GET(pack) == e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI);
    assert(p11002_bus_GET(pack) == (uint8_t)(uint8_t)96);
    assert(p11002_address_GET(pack) == (uint8_t)(uint8_t)175);
    {
        uint8_t exemplary[] =  {(uint8_t)125, (uint8_t)231, (uint8_t)128, (uint8_t)93, (uint8_t)87, (uint8_t)244, (uint8_t)248, (uint8_t)93, (uint8_t)92, (uint8_t)244, (uint8_t)236, (uint8_t)97, (uint8_t)175, (uint8_t)161, (uint8_t)141, (uint8_t)187, (uint8_t)121, (uint8_t)208, (uint8_t)208, (uint8_t)94, (uint8_t)225, (uint8_t)235, (uint8_t)229, (uint8_t)235, (uint8_t)161, (uint8_t)188, (uint8_t)221, (uint8_t)73, (uint8_t)96, (uint8_t)216, (uint8_t)13, (uint8_t)41, (uint8_t)125, (uint8_t)141, (uint8_t)179, (uint8_t)93, (uint8_t)135, (uint8_t)167, (uint8_t)94, (uint8_t)245, (uint8_t)157, (uint8_t)245, (uint8_t)19, (uint8_t)65, (uint8_t)86, (uint8_t)148, (uint8_t)47, (uint8_t)6, (uint8_t)30, (uint8_t)139, (uint8_t)86, (uint8_t)110, (uint8_t)226, (uint8_t)183, (uint8_t)197, (uint8_t)75, (uint8_t)134, (uint8_t)43, (uint8_t)193, (uint8_t)96, (uint8_t)131, (uint8_t)248, (uint8_t)126, (uint8_t)186, (uint8_t)71, (uint8_t)105, (uint8_t)66, (uint8_t)240, (uint8_t)226, (uint8_t)189, (uint8_t)35, (uint8_t)246, (uint8_t)226, (uint8_t)103, (uint8_t)228, (uint8_t)230, (uint8_t)233, (uint8_t)175, (uint8_t)233, (uint8_t)110, (uint8_t)210, (uint8_t)83, (uint8_t)52, (uint8_t)173, (uint8_t)243, (uint8_t)171, (uint8_t)37, (uint8_t)151, (uint8_t)51, (uint8_t)2, (uint8_t)100, (uint8_t)127, (uint8_t)54, (uint8_t)207, (uint8_t)98, (uint8_t)131, (uint8_t)153, (uint8_t)194, (uint8_t)229, (uint8_t)11, (uint8_t)209, (uint8_t)159, (uint8_t)224, (uint8_t)107, (uint8_t)52, (uint8_t)155, (uint8_t)30, (uint8_t)52, (uint8_t)102, (uint8_t)115, (uint8_t)158, (uint8_t)206, (uint8_t)194, (uint8_t)8, (uint8_t)136, (uint8_t)203, (uint8_t)175, (uint8_t)160, (uint8_t)232, (uint8_t)173, (uint8_t)32, (uint8_t)34, (uint8_t)122, (uint8_t)3, (uint8_t)207, (uint8_t)22, (uint8_t)150, (uint8_t)156} ;
        uint8_t*  sample = p11002_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 128);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p11002_target_system_GET(pack) == (uint8_t)(uint8_t)110);
};


void c_LoopBackDemoChannel_on_DEVICE_OP_WRITE_REPLY_11003(Bounds_Inside * ph, Pack * pack)
{
    assert(p11003_result_GET(pack) == (uint8_t)(uint8_t)7);
    assert(p11003_request_id_GET(pack) == (uint32_t)1887325500L);
};


void c_LoopBackDemoChannel_on_ADAP_TUNING_11010(Bounds_Inside * ph, Pack * pack)
{
    assert(p11010_error_GET(pack) == (float) -2.6518616E38F);
    assert(p11010_sigma_GET(pack) == (float) -7.4927125E37F);
    assert(p11010_omega_GET(pack) == (float)8.447634E37F);
    assert(p11010_achieved_GET(pack) == (float)2.3573397E38F);
    assert(p11010_sigma_dot_GET(pack) == (float)2.1316E38F);
    assert(p11010_omega_dot_GET(pack) == (float) -8.996182E37F);
    assert(p11010_theta_dot_GET(pack) == (float) -9.139858E37F);
    assert(p11010_f_dot_GET(pack) == (float)4.0589475E37F);
    assert(p11010_u_GET(pack) == (float) -1.5637594E38F);
    assert(p11010_desired_GET(pack) == (float)2.8714495E38F);
    assert(p11010_theta_GET(pack) == (float)2.8962885E38F);
    assert(p11010_f_GET(pack) == (float) -5.6073536E37F);
    assert(p11010_axis_GET(pack) == e_PID_TUNING_AXIS_PID_TUNING_STEER);
};


void c_LoopBackDemoChannel_on_VISION_POSITION_DELTA_11011(Bounds_Inside * ph, Pack * pack)
{
    assert(p11011_time_usec_GET(pack) == (uint64_t)2937697154875305446L);
    {
        float exemplary[] =  {2.0390608E38F, -7.187309E37F, 5.103497E37F} ;
        float*  sample = p11011_angle_delta_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p11011_confidence_GET(pack) == (float) -1.761705E38F);
    assert(p11011_time_delta_usec_GET(pack) == (uint64_t)782001265752966504L);
    {
        float exemplary[] =  {2.3189907E38F, 1.9596842E38F, -1.3826785E38F} ;
        float*  sample = p11011_position_delta_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
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
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_EMERGENCY, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_VTOL_TILTROTOR, PH.base.pack) ;
        p0_custom_mode_SET((uint32_t)2738671950L, PH.base.pack) ;
        p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED, PH.base.pack) ;
        p0_mavlink_version_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYS_STATUS_1(), &PH);
        p1_load_SET((uint16_t)(uint16_t)42862, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t) -25779, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL, PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)62410, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)6434, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_MAG, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)26031, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)16946, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t)118, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)44974, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)44177, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)27906, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_unix_usec_SET((uint64_t)5537790853574210705L, PH.base.pack) ;
        p2_time_boot_ms_SET((uint32_t)694755993L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p3_vy_SET((float)5.9839147E37F, PH.base.pack) ;
        p3_yaw_SET((float)1.5791067E38F, PH.base.pack) ;
        p3_vx_SET((float)2.1979907E38F, PH.base.pack) ;
        p3_vz_SET((float) -2.4121089E38F, PH.base.pack) ;
        p3_afx_SET((float) -4.544112E37F, PH.base.pack) ;
        p3_y_SET((float) -2.7512079E38F, PH.base.pack) ;
        p3_yaw_rate_SET((float)3.2824632E38F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)29723796L, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)3530, PH.base.pack) ;
        p3_afz_SET((float) -7.6385735E36F, PH.base.pack) ;
        p3_z_SET((float)7.410594E37F, PH.base.pack) ;
        p3_x_SET((float)1.4921827E36F, PH.base.pack) ;
        p3_afy_SET((float)4.496233E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PING_4(), &PH);
        p4_target_component_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        p4_time_usec_SET((uint64_t)295212837129797761L, PH.base.pack) ;
        p4_seq_SET((uint32_t)1079617963L, PH.base.pack) ;
        p4_target_system_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        p5_control_request_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        p5_target_system_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        p5_version_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
        {
            char16_t* passkey = u"duflvxubDGzuxnf";
            p5_passkey_SET_(passkey, &PH) ;
        }
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_ack_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        p6_control_request_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"xmomOxxegpxaiglhk";
            p7_key_SET_(key, &PH) ;
        }
        c_LoopBackDemoChannel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_MODE_11(), &PH);
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_TEST_ARMED, PH.base.pack) ;
        p11_target_system_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        p11_custom_mode_SET((uint32_t)4002579925L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_READ_20(), &PH);
        p20_param_index_SET((int16_t)(int16_t)11603, PH.base.pack) ;
        {
            char16_t* param_id = u"npWzjbpdwrp";
            p20_param_id_SET_(param_id, &PH) ;
        }
        p20_target_component_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        p20_target_system_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_component_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        p21_target_system_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_VALUE_22(), &PH);
        p22_param_value_SET((float) -4.541054E37F, PH.base.pack) ;
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL64, PH.base.pack) ;
        p22_param_index_SET((uint16_t)(uint16_t)35104, PH.base.pack) ;
        {
            char16_t* param_id = u"rtwbTRl";
            p22_param_id_SET_(param_id, &PH) ;
        }
        p22_param_count_SET((uint16_t)(uint16_t)31089, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_SET_23(), &PH);
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_REAL32, PH.base.pack) ;
        p23_target_system_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        {
            char16_t* param_id = u"xTNaPBjy";
            p23_param_id_SET_(param_id, &PH) ;
        }
        p23_target_component_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        p23_param_value_SET((float) -2.9286487E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RAW_INT_24(), &PH);
        p24_time_usec_SET((uint64_t)8726738046740334972L, PH.base.pack) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, PH.base.pack) ;
        p24_alt_SET((int32_t)1734640160, PH.base.pack) ;
        p24_cog_SET((uint16_t)(uint16_t)33232, PH.base.pack) ;
        p24_eph_SET((uint16_t)(uint16_t)15587, PH.base.pack) ;
        p24_alt_ellipsoid_SET((int32_t) -503660480, &PH) ;
        p24_epv_SET((uint16_t)(uint16_t)5309, PH.base.pack) ;
        p24_vel_acc_SET((uint32_t)1023747684L, &PH) ;
        p24_v_acc_SET((uint32_t)1834107273L, &PH) ;
        p24_hdg_acc_SET((uint32_t)2826814005L, &PH) ;
        p24_lat_SET((int32_t)139828968, PH.base.pack) ;
        p24_vel_SET((uint16_t)(uint16_t)48540, PH.base.pack) ;
        p24_lon_SET((int32_t) -1176944854, PH.base.pack) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p24_h_acc_SET((uint32_t)1731412571L, &PH) ;
        c_LoopBackDemoChannel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_used[] =  {(uint8_t)125, (uint8_t)46, (uint8_t)221, (uint8_t)228, (uint8_t)91, (uint8_t)110, (uint8_t)6, (uint8_t)161, (uint8_t)225, (uint8_t)55, (uint8_t)153, (uint8_t)76, (uint8_t)238, (uint8_t)185, (uint8_t)101, (uint8_t)92, (uint8_t)112, (uint8_t)49, (uint8_t)129, (uint8_t)72};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_snr[] =  {(uint8_t)197, (uint8_t)255, (uint8_t)55, (uint8_t)227, (uint8_t)12, (uint8_t)6, (uint8_t)181, (uint8_t)194, (uint8_t)123, (uint8_t)222, (uint8_t)234, (uint8_t)188, (uint8_t)1, (uint8_t)222, (uint8_t)185, (uint8_t)116, (uint8_t)226, (uint8_t)99, (uint8_t)1, (uint8_t)63};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)244, (uint8_t)168, (uint8_t)154, (uint8_t)97, (uint8_t)111, (uint8_t)23, (uint8_t)229, (uint8_t)173, (uint8_t)12, (uint8_t)160, (uint8_t)170, (uint8_t)252, (uint8_t)224, (uint8_t)151, (uint8_t)254, (uint8_t)227, (uint8_t)235, (uint8_t)124, (uint8_t)220, (uint8_t)221};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)25, (uint8_t)39, (uint8_t)66, (uint8_t)78, (uint8_t)205, (uint8_t)3, (uint8_t)153, (uint8_t)3, (uint8_t)209, (uint8_t)123, (uint8_t)62, (uint8_t)37, (uint8_t)253, (uint8_t)158, (uint8_t)241, (uint8_t)93, (uint8_t)96, (uint8_t)147, (uint8_t)154, (uint8_t)10};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)153, PH.base.pack) ;
        {
            uint8_t satellite_prn[] =  {(uint8_t)205, (uint8_t)119, (uint8_t)46, (uint8_t)111, (uint8_t)51, (uint8_t)169, (uint8_t)21, (uint8_t)40, (uint8_t)146, (uint8_t)113, (uint8_t)87, (uint8_t)18, (uint8_t)214, (uint8_t)159, (uint8_t)134, (uint8_t)230, (uint8_t)48, (uint8_t)6, (uint8_t)95, (uint8_t)156};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU_26(), &PH);
        p26_time_boot_ms_SET((uint32_t)3178484726L, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t) -21568, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t)7351, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t) -11455, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t)17639, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t) -1756, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t) -9004, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t) -19076, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t) -27123, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t)18231, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_IMU_27(), &PH);
        p27_xmag_SET((int16_t)(int16_t)1039, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t) -12330, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t) -8081, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t)15369, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t)21658, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t)30972, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)1660514289681552398L, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t) -17290, PH.base.pack) ;
        p27_zacc_SET((int16_t)(int16_t) -32221, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t)16203, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_PRESSURE_28(), &PH);
        p28_time_usec_SET((uint64_t)322954987939889448L, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t)5548, PH.base.pack) ;
        p28_press_diff2_SET((int16_t)(int16_t) -2807, PH.base.pack) ;
        p28_press_diff1_SET((int16_t)(int16_t)31337, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t)2081, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_press_abs_SET((float)3.3113036E38F, PH.base.pack) ;
        p29_temperature_SET((int16_t)(int16_t)2956, PH.base.pack) ;
        p29_press_diff_SET((float) -2.0278008E38F, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)4272831302L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_30(), &PH);
        p30_rollspeed_SET((float) -2.275626E38F, PH.base.pack) ;
        p30_yaw_SET((float) -1.3763289E38F, PH.base.pack) ;
        p30_pitchspeed_SET((float)1.1517027E38F, PH.base.pack) ;
        p30_roll_SET((float) -2.9523175E38F, PH.base.pack) ;
        p30_pitch_SET((float)3.5172857E37F, PH.base.pack) ;
        p30_yawspeed_SET((float) -2.4112037E38F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)686142457L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_q3_SET((float)2.7786903E38F, PH.base.pack) ;
        p31_q1_SET((float)3.1645586E37F, PH.base.pack) ;
        p31_q2_SET((float)1.6304298E38F, PH.base.pack) ;
        p31_yawspeed_SET((float) -3.3210433E37F, PH.base.pack) ;
        p31_rollspeed_SET((float) -3.0380153E38F, PH.base.pack) ;
        p31_pitchspeed_SET((float)1.2690295E38F, PH.base.pack) ;
        p31_q4_SET((float)6.218109E36F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)1114166402L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_vx_SET((float) -7.070674E37F, PH.base.pack) ;
        p32_vy_SET((float) -1.1324511E38F, PH.base.pack) ;
        p32_x_SET((float)3.2993804E37F, PH.base.pack) ;
        p32_vz_SET((float) -2.2068387E38F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)525889239L, PH.base.pack) ;
        p32_y_SET((float) -1.1857329E38F, PH.base.pack) ;
        p32_z_SET((float) -1.0667396E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_hdg_SET((uint16_t)(uint16_t)23299, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t) -22013, PH.base.pack) ;
        p33_alt_SET((int32_t)226553483, PH.base.pack) ;
        p33_relative_alt_SET((int32_t)319145427, PH.base.pack) ;
        p33_lon_SET((int32_t)1627215834, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)2471165527L, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t) -11924, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t)16709, PH.base.pack) ;
        p33_lat_SET((int32_t) -19907120, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan1_scaled_SET((int16_t)(int16_t) -16611, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)3708360954L, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t) -22726, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t) -17096, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t)30651, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t) -1718, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t) -27631, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t) -8791, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t) -30033, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_chan3_raw_SET((uint16_t)(uint16_t)36007, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)46015, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)35720, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)28923, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)24758, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)20432, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)1624542723L, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)52247, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)25318, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo1_raw_SET((uint16_t)(uint16_t)15104, PH.base.pack) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)26132, PH.base.pack) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)19097, &PH) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)6263, PH.base.pack) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)55878, &PH) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)32297, PH.base.pack) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)46534, &PH) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)22987, PH.base.pack) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)3225, &PH) ;
        p36_port_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)4498, &PH) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)17085, PH.base.pack) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)54789, PH.base.pack) ;
        p36_time_usec_SET((uint32_t)3776491599L, PH.base.pack) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)36637, &PH) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)64371, PH.base.pack) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)47221, &PH) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)37827, &PH) ;
        c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_start_index_SET((int16_t)(int16_t)3785, PH.base.pack) ;
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t) -15483, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_start_index_SET((int16_t)(int16_t)21117, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p38_target_system_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        p38_target_component_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
        p38_end_index_SET((int16_t)(int16_t) -7132, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_39(), &PH);
        p39_param4_SET((float) -1.4588676E38F, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p39_y_SET((float)2.7043695E37F, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_SET_FACTORY_TEST_MODE, PH.base.pack) ;
        p39_param2_SET((float) -2.717986E38F, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        p39_param3_SET((float) -3.1855453E38F, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p39_x_SET((float) -2.176817E38F, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)46645, PH.base.pack) ;
        p39_param1_SET((float) -6.999244E37F, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p39_z_SET((float) -2.8204723E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_40(), &PH);
        p40_target_component_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p40_seq_SET((uint16_t)(uint16_t)24011, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_target_system_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
        p41_seq_SET((uint16_t)(uint16_t)56896, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)27028, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_target_system_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p43_target_component_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_COUNT_44(), &PH);
        p44_count_SET((uint16_t)(uint16_t)17202, PH.base.pack) ;
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p44_target_component_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p45_target_component_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        p45_target_system_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)10861, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ACK_47(), &PH);
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID_SEQUENCE, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_time_usec_SET((uint64_t)3850265073922235744L, &PH) ;
        p48_latitude_SET((int32_t) -205468366, PH.base.pack) ;
        p48_target_system_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        p48_altitude_SET((int32_t)1216808069, PH.base.pack) ;
        p48_longitude_SET((int32_t) -1060847626, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_latitude_SET((int32_t)1686598451, PH.base.pack) ;
        p49_time_usec_SET((uint64_t)6713642608886351560L, &PH) ;
        p49_altitude_SET((int32_t) -983333754, PH.base.pack) ;
        p49_longitude_SET((int32_t)1354325720, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_MAP_RC_50(), &PH);
        {
            char16_t* param_id = u"g";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_target_component_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        p50_param_value0_SET((float) -1.5687017E38F, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t) -2458, PH.base.pack) ;
        p50_param_value_min_SET((float) -7.10308E37F, PH.base.pack) ;
        p50_param_value_max_SET((float)9.822185E37F, PH.base.pack) ;
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
        p50_scale_SET((float)2.203298E38F, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)4339, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
        p51_target_component_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_p2z_SET((float)9.710677E37F, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
        p54_p1z_SET((float) -2.2999342E38F, PH.base.pack) ;
        p54_p1x_SET((float)3.1360792E38F, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        p54_p1y_SET((float) -1.8485999E38F, PH.base.pack) ;
        p54_p2x_SET((float) -1.6307219E38F, PH.base.pack) ;
        p54_p2y_SET((float) -1.0273129E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT, PH.base.pack) ;
        p55_p1x_SET((float)9.332756E37F, PH.base.pack) ;
        p55_p1z_SET((float) -3.0465901E38F, PH.base.pack) ;
        p55_p2x_SET((float) -3.7479743E37F, PH.base.pack) ;
        p55_p2z_SET((float) -3.0495473E38F, PH.base.pack) ;
        p55_p2y_SET((float)1.7371462E37F, PH.base.pack) ;
        p55_p1y_SET((float) -2.1775747E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_time_usec_SET((uint64_t)8954629646871929114L, PH.base.pack) ;
        {
            float covariance[] =  {2.7418932E38F, -2.0236178E38F, 5.8848707E37F, 7.521846E37F, -1.5501494E38F, 2.4142602E38F, 2.4843589E38F, -6.225056E37F, -5.7039303E37F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p61_pitchspeed_SET((float)1.7580716E38F, PH.base.pack) ;
        {
            float q[] =  {2.6722955E38F, -1.2552885E38F, 1.7940227E38F, -2.1209194E38F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        p61_rollspeed_SET((float)1.9005847E38F, PH.base.pack) ;
        p61_yawspeed_SET((float)1.3639271E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_nav_roll_SET((float) -9.387065E37F, PH.base.pack) ;
        p62_aspd_error_SET((float)3.2367955E38F, PH.base.pack) ;
        p62_nav_pitch_SET((float)6.717252E36F, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t) -19908, PH.base.pack) ;
        p62_alt_error_SET((float)5.7469325E37F, PH.base.pack) ;
        p62_xtrack_error_SET((float) -2.1998015E38F, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)10267, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t)8406, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_vz_SET((float) -2.3772614E37F, PH.base.pack) ;
        p63_lat_SET((int32_t) -1623622009, PH.base.pack) ;
        p63_alt_SET((int32_t) -1534862069, PH.base.pack) ;
        p63_relative_alt_SET((int32_t)1080065771, PH.base.pack) ;
        p63_lon_SET((int32_t) -504838712, PH.base.pack) ;
        p63_vy_SET((float)2.0715894E36F, PH.base.pack) ;
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, PH.base.pack) ;
        {
            float covariance[] =  {2.5606228E38F, -2.307961E38F, 5.4891345E37F, 2.1177996E38F, -4.8552407E37F, -1.0163799E38F, 1.3329797E38F, -5.208861E37F, -1.4886549E38F, 1.4519405E38F, -1.469174E37F, 6.0758676E37F, 7.3372403E37F, 2.8506357E38F, 3.2207477E38F, 1.4757735E38F, 1.4200969E38F, 1.3198769E38F, 1.4131736E38F, 8.857364E37F, 5.4403573E37F, -1.0235026E38F, -2.818973E38F, 8.996177E37F, 2.7526445E38F, 1.7888211E38F, 2.0106995E38F, -1.065261E37F, 2.5724417E38F, -1.992602E38F, -1.8715593E38F, -3.062535E38F, 3.3508153E37F, -3.0475572E38F, 2.5526165E38F, 2.8315765E38F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_time_usec_SET((uint64_t)6729255855491189222L, PH.base.pack) ;
        p63_vx_SET((float)6.183623E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_x_SET((float) -1.8997894E38F, PH.base.pack) ;
        p64_vx_SET((float) -1.2809868E37F, PH.base.pack) ;
        p64_y_SET((float) -2.0582683E37F, PH.base.pack) ;
        p64_az_SET((float)1.4947249E38F, PH.base.pack) ;
        p64_z_SET((float)1.6027555E38F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS_INS, PH.base.pack) ;
        p64_vy_SET((float)1.8128783E38F, PH.base.pack) ;
        p64_ax_SET((float) -3.349672E38F, PH.base.pack) ;
        {
            float covariance[] =  {1.8606715E38F, -1.2011744E38F, 9.951099E37F, 3.0946533E38F, -2.4818682E38F, -2.6271178E38F, -1.5186705E38F, 2.8541664E38F, 3.5220503E37F, -8.99158E37F, -2.3841796E38F, 1.271449E38F, -1.5102609E38F, -2.1707734E38F, 2.5208891E38F, -2.480653E38F, 1.3881866E38F, 2.9458642E38F, -2.4458651E38F, -2.460553E38F, 5.0651586E37F, 3.3222751E38F, -1.635427E38F, 7.6055664E37F, 1.3094552E38F, 2.5690655E38F, -2.7075216E38F, -2.2721197E38F, -1.2924376E38F, 1.4971302E38F, -3.3300193E37F, -1.2358398E37F, 1.9181393E38F, 1.4504982E38F, -3.0959537E38F, 9.22801E37F, -2.9958997E38F, -2.6933494E38F, 9.316511E37F, 2.9019041E38F, 4.0905183E37F, 1.7252666E38F, 1.3998047E38F, -1.5735885E38F, 1.221211E37F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_ay_SET((float)2.217106E38F, PH.base.pack) ;
        p64_vz_SET((float)1.9953693E38F, PH.base.pack) ;
        p64_time_usec_SET((uint64_t)5283133235681849801L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan3_raw_SET((uint16_t)(uint16_t)1343, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)21304, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)13134, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)3764083324L, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)51730, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)52838, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)6719, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)3650, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)50663, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)12414, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)10130, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)32695, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)18988, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)58906, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)32635, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)27279, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)46395, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)35353, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)65129, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_req_stream_id_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        p66_req_message_rate_SET((uint16_t)(uint16_t)63508, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_STREAM_67(), &PH);
        p67_message_rate_SET((uint16_t)(uint16_t)19747, PH.base.pack) ;
        p67_stream_id_SET((uint8_t)(uint8_t)140, PH.base.pack) ;
        p67_on_off_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_y_SET((int16_t)(int16_t)13683, PH.base.pack) ;
        p69_z_SET((int16_t)(int16_t)2859, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t)24478, PH.base.pack) ;
        p69_buttons_SET((uint16_t)(uint16_t)34956, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t) -8231, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan6_raw_SET((uint16_t)(uint16_t)24273, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)51127, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)34422, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)56535, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)48914, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)57828, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)65027, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)8992, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_x_SET((int32_t)1588369050, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_NAV_LOITER_TURNS, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)9251, PH.base.pack) ;
        p73_param2_SET((float)1.3783983E38F, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p73_param4_SET((float)2.8016328E38F, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        p73_param1_SET((float) -3.2703964E38F, PH.base.pack) ;
        p73_z_SET((float)3.3374273E38F, PH.base.pack) ;
        p73_y_SET((int32_t)767398716, PH.base.pack) ;
        p73_param3_SET((float)2.1609066E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VFR_HUD_74(), &PH);
        p74_throttle_SET((uint16_t)(uint16_t)31910, PH.base.pack) ;
        p74_climb_SET((float) -1.4950729E38F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t)1209, PH.base.pack) ;
        p74_alt_SET((float)3.2069477E38F, PH.base.pack) ;
        p74_airspeed_SET((float) -1.5998549E38F, PH.base.pack) ;
        p74_groundspeed_SET((float)2.5356395E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_INT_75(), &PH);
        p75_z_SET((float)2.7057347E38F, PH.base.pack) ;
        p75_param3_SET((float)3.3703351E38F, PH.base.pack) ;
        p75_y_SET((int32_t)87216988, PH.base.pack) ;
        p75_param4_SET((float)2.195852E38F, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
        p75_param2_SET((float) -1.0598652E38F, PH.base.pack) ;
        p75_x_SET((int32_t)1216159528, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_SPATIAL_USER_5, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
        p75_param1_SET((float) -2.9364562E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_LONG_76(), &PH);
        p76_target_system_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p76_param2_SET((float) -3.1319247E38F, PH.base.pack) ;
        p76_param3_SET((float)2.861883E38F, PH.base.pack) ;
        p76_param5_SET((float) -1.9226787E38F, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)31, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE, PH.base.pack) ;
        p76_param4_SET((float)2.9928464E38F, PH.base.pack) ;
        p76_param6_SET((float) -2.8771945E38F, PH.base.pack) ;
        p76_param1_SET((float)1.4418545E38F, PH.base.pack) ;
        p76_param7_SET((float) -3.3321677E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_ACK_77(), &PH);
        p77_result_param2_SET((int32_t)1545741484, &PH) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_IN_PROGRESS, PH.base.pack) ;
        p77_target_system_SET((uint8_t)(uint8_t)62, &PH) ;
        p77_target_component_SET((uint8_t)(uint8_t)109, &PH) ;
        p77_progress_SET((uint8_t)(uint8_t)150, &PH) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_DO_REPEAT_SERVO, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_pitch_SET((float)3.1803447E38F, PH.base.pack) ;
        p81_roll_SET((float) -2.7895122E38F, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        p81_thrust_SET((float) -1.830613E37F, PH.base.pack) ;
        p81_yaw_SET((float)4.0076343E36F, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)3750877057L, PH.base.pack) ;
        p81_manual_override_switch_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_time_boot_ms_SET((uint32_t)3883848164L, PH.base.pack) ;
        p82_body_roll_rate_SET((float)3.3626189E38F, PH.base.pack) ;
        p82_body_pitch_rate_SET((float) -3.0549619E38F, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p82_thrust_SET((float) -2.9186544E38F, PH.base.pack) ;
        {
            float q[] =  {-1.4671242E38F, -8.58204E37F, 1.7484778E38F, -1.413136E37F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_body_yaw_rate_SET((float)1.9718532E38F, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_thrust_SET((float) -6.427242E37F, PH.base.pack) ;
        p83_body_roll_rate_SET((float)2.5926627E38F, PH.base.pack) ;
        p83_body_yaw_rate_SET((float) -1.9527042E38F, PH.base.pack) ;
        {
            float q[] =  {-9.643587E37F, 3.1130878E38F, 8.923623E37F, 3.3298433E38F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_body_pitch_rate_SET((float) -2.3883612E38F, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)641785841L, PH.base.pack) ;
        p83_type_mask_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_vy_SET((float) -1.5731545E38F, PH.base.pack) ;
        p84_z_SET((float) -2.0589157E38F, PH.base.pack) ;
        p84_vz_SET((float) -1.1845589E38F, PH.base.pack) ;
        p84_yaw_rate_SET((float) -2.7849231E38F, PH.base.pack) ;
        p84_afx_SET((float) -2.9063679E38F, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)53737, PH.base.pack) ;
        p84_vx_SET((float) -1.9269764E38F, PH.base.pack) ;
        p84_afy_SET((float)2.3537326E37F, PH.base.pack) ;
        p84_x_SET((float) -1.1328118E38F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)1416464694L, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        p84_y_SET((float) -1.086098E38F, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p84_yaw_SET((float)2.109506E38F, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        p84_afz_SET((float)5.043134E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_yaw_SET((float)7.8878717E37F, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p86_alt_SET((float) -3.3001401E38F, PH.base.pack) ;
        p86_afz_SET((float)1.877922E38F, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)1731633069L, PH.base.pack) ;
        p86_vx_SET((float) -1.8779907E38F, PH.base.pack) ;
        p86_yaw_rate_SET((float) -9.669672E37F, PH.base.pack) ;
        p86_afx_SET((float) -5.218414E37F, PH.base.pack) ;
        p86_vz_SET((float)1.973987E38F, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)38540, PH.base.pack) ;
        p86_afy_SET((float)1.474745E38F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p86_vy_SET((float) -1.9492552E38F, PH.base.pack) ;
        p86_lat_int_SET((int32_t)1829564625, PH.base.pack) ;
        p86_lon_int_SET((int32_t)1229895209, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_lat_int_SET((int32_t) -2133855171, PH.base.pack) ;
        p87_afx_SET((float) -2.4330955E38F, PH.base.pack) ;
        p87_afy_SET((float)1.5987702E38F, PH.base.pack) ;
        p87_vy_SET((float) -7.36924E36F, PH.base.pack) ;
        p87_afz_SET((float) -3.5804155E37F, PH.base.pack) ;
        p87_vz_SET((float) -3.122269E38F, PH.base.pack) ;
        p87_alt_SET((float) -1.1121075E38F, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)2764924049L, PH.base.pack) ;
        p87_lon_int_SET((int32_t)972280192, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)59220, PH.base.pack) ;
        p87_vx_SET((float) -1.3133995E38F, PH.base.pack) ;
        p87_yaw_SET((float) -2.3570772E38F, PH.base.pack) ;
        p87_yaw_rate_SET((float)2.1143937E37F, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_z_SET((float)2.635742E38F, PH.base.pack) ;
        p89_x_SET((float) -8.914448E37F, PH.base.pack) ;
        p89_yaw_SET((float)8.186212E37F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)3449120020L, PH.base.pack) ;
        p89_roll_SET((float)2.9606661E38F, PH.base.pack) ;
        p89_y_SET((float)2.4002461E38F, PH.base.pack) ;
        p89_pitch_SET((float)4.8518145E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_90(), &PH);
        p90_vx_SET((int16_t)(int16_t) -15297, PH.base.pack) ;
        p90_roll_SET((float)3.1443818E38F, PH.base.pack) ;
        p90_yaw_SET((float) -3.2473172E38F, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t)11631, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t)19271, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t) -6912, PH.base.pack) ;
        p90_lat_SET((int32_t) -613426522, PH.base.pack) ;
        p90_rollspeed_SET((float) -3.2657432E38F, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t)18640, PH.base.pack) ;
        p90_pitchspeed_SET((float)8.824346E37F, PH.base.pack) ;
        p90_alt_SET((int32_t) -1532746219, PH.base.pack) ;
        p90_yawspeed_SET((float) -7.8936157E37F, PH.base.pack) ;
        p90_lon_SET((int32_t) -1980839820, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t) -23917, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)2536275186215324673L, PH.base.pack) ;
        p90_pitch_SET((float)8.681347E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_CONTROLS_91(), &PH);
        p91_yaw_rudder_SET((float)1.2177409E38F, PH.base.pack) ;
        p91_aux1_SET((float) -5.359019E37F, PH.base.pack) ;
        p91_aux2_SET((float)1.9419781E36F, PH.base.pack) ;
        p91_throttle_SET((float)3.5207347E37F, PH.base.pack) ;
        p91_aux4_SET((float)3.2120467E38F, PH.base.pack) ;
        p91_pitch_elevator_SET((float) -2.4176093E38F, PH.base.pack) ;
        p91_roll_ailerons_SET((float)1.7811035E38F, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)7408040310355973957L, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)79, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_DISARMED, PH.base.pack) ;
        p91_aux3_SET((float)1.3995551E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan10_raw_SET((uint16_t)(uint16_t)13738, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)52166, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)47816, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)3915794727397469184L, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)27352, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)54775, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)4044, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)32853, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)4312, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)23339, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)51880, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)17334, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)244, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_mode_SET(e_MAV_MODE_MAV_MODE_TEST_ARMED, PH.base.pack) ;
        p93_flags_SET((uint64_t)3345631702855313753L, PH.base.pack) ;
        {
            float controls[] =  {-1.2365345E37F, 3.194105E38F, -2.491589E38F, 1.3174247E38F, 2.0227769E38F, -2.1508094E38F, 5.98522E37F, -2.627403E38F, -1.5292548E37F, 3.3163926E38F, -8.861006E37F, 2.6985736E38F, 1.1782844E38F, -2.6362545E38F, 2.9915023E38F, -1.1797941E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p93_time_usec_SET((uint64_t)3610378822110451169L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_flow_comp_m_x_SET((float)1.2318748E38F, PH.base.pack) ;
        p100_flow_rate_y_SET((float)9.212116E36F, &PH) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p100_flow_x_SET((int16_t)(int16_t)30938, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)2492431362765764245L, PH.base.pack) ;
        p100_quality_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        p100_ground_distance_SET((float)6.510126E37F, PH.base.pack) ;
        p100_flow_rate_x_SET((float)2.8145007E37F, &PH) ;
        p100_flow_y_SET((int16_t)(int16_t)2545, PH.base.pack) ;
        p100_flow_comp_m_y_SET((float) -2.4726239E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_z_SET((float)2.3854702E38F, PH.base.pack) ;
        p101_x_SET((float)3.2478097E38F, PH.base.pack) ;
        p101_pitch_SET((float) -9.180316E34F, PH.base.pack) ;
        p101_y_SET((float) -2.0894648E38F, PH.base.pack) ;
        p101_usec_SET((uint64_t)346127162980979691L, PH.base.pack) ;
        p101_yaw_SET((float)7.589145E37F, PH.base.pack) ;
        p101_roll_SET((float)2.4883297E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_yaw_SET((float) -3.1029334E38F, PH.base.pack) ;
        p102_pitch_SET((float)3.0887832E38F, PH.base.pack) ;
        p102_usec_SET((uint64_t)6982712676303463025L, PH.base.pack) ;
        p102_x_SET((float)1.1648626E38F, PH.base.pack) ;
        p102_z_SET((float)1.2230557E38F, PH.base.pack) ;
        p102_roll_SET((float) -5.49813E37F, PH.base.pack) ;
        p102_y_SET((float) -2.5966035E36F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_y_SET((float)1.1641661E38F, PH.base.pack) ;
        p103_usec_SET((uint64_t)642658115926641724L, PH.base.pack) ;
        p103_z_SET((float)4.0948854E37F, PH.base.pack) ;
        p103_x_SET((float)1.1976817E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_usec_SET((uint64_t)2848649985460158797L, PH.base.pack) ;
        p104_y_SET((float)1.6648662E38F, PH.base.pack) ;
        p104_yaw_SET((float)2.3019705E38F, PH.base.pack) ;
        p104_x_SET((float)3.037076E38F, PH.base.pack) ;
        p104_z_SET((float) -1.5407191E38F, PH.base.pack) ;
        p104_roll_SET((float)1.955726E38F, PH.base.pack) ;
        p104_pitch_SET((float)2.4871863E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGHRES_IMU_105(), &PH);
        p105_time_usec_SET((uint64_t)8035732459994930555L, PH.base.pack) ;
        p105_yacc_SET((float) -1.3589238E38F, PH.base.pack) ;
        p105_xgyro_SET((float)2.1181823E38F, PH.base.pack) ;
        p105_temperature_SET((float)6.1761656E37F, PH.base.pack) ;
        p105_pressure_alt_SET((float) -1.763626E38F, PH.base.pack) ;
        p105_xmag_SET((float) -4.3688057E37F, PH.base.pack) ;
        p105_abs_pressure_SET((float)9.383402E37F, PH.base.pack) ;
        p105_ygyro_SET((float)1.5737628E38F, PH.base.pack) ;
        p105_ymag_SET((float)2.3814648E38F, PH.base.pack) ;
        p105_xacc_SET((float)2.7754194E38F, PH.base.pack) ;
        p105_diff_pressure_SET((float)1.938417E38F, PH.base.pack) ;
        p105_zacc_SET((float)1.4714333E38F, PH.base.pack) ;
        p105_zmag_SET((float) -1.0142596E38F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)52405, PH.base.pack) ;
        p105_zgyro_SET((float) -1.1962299E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_distance_SET((float) -1.5792013E38F, PH.base.pack) ;
        p106_integrated_zgyro_SET((float)5.222451E37F, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)1080346057L, PH.base.pack) ;
        p106_integrated_x_SET((float) -1.7889487E37F, PH.base.pack) ;
        p106_integrated_y_SET((float)2.8276845E38F, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t)16115, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)6274787953676475057L, PH.base.pack) ;
        p106_integrated_ygyro_SET((float)2.4168254E38F, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)891935782L, PH.base.pack) ;
        p106_integrated_xgyro_SET((float)1.6676042E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_SENSOR_107(), &PH);
        p107_fields_updated_SET((uint32_t)4017465888L, PH.base.pack) ;
        p107_zgyro_SET((float) -2.4262745E38F, PH.base.pack) ;
        p107_xacc_SET((float)3.2411813E37F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)5766368874335532026L, PH.base.pack) ;
        p107_abs_pressure_SET((float)1.830222E38F, PH.base.pack) ;
        p107_diff_pressure_SET((float)3.1826766E38F, PH.base.pack) ;
        p107_zmag_SET((float)2.5152946E38F, PH.base.pack) ;
        p107_pressure_alt_SET((float)5.9387255E37F, PH.base.pack) ;
        p107_ymag_SET((float)6.05489E37F, PH.base.pack) ;
        p107_temperature_SET((float) -1.2685911E38F, PH.base.pack) ;
        p107_xmag_SET((float) -2.519297E38F, PH.base.pack) ;
        p107_yacc_SET((float)3.2596494E37F, PH.base.pack) ;
        p107_xgyro_SET((float)1.3213113E38F, PH.base.pack) ;
        p107_ygyro_SET((float)4.0846103E37F, PH.base.pack) ;
        p107_zacc_SET((float) -3.079331E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SIM_STATE_108(), &PH);
        p108_pitch_SET((float) -2.8306762E38F, PH.base.pack) ;
        p108_xgyro_SET((float)1.7057622E38F, PH.base.pack) ;
        p108_std_dev_horz_SET((float) -3.1721109E38F, PH.base.pack) ;
        p108_zgyro_SET((float) -6.008581E37F, PH.base.pack) ;
        p108_roll_SET((float) -3.1138229E38F, PH.base.pack) ;
        p108_q1_SET((float) -1.6243857E38F, PH.base.pack) ;
        p108_std_dev_vert_SET((float)5.989723E36F, PH.base.pack) ;
        p108_q4_SET((float)2.7459766E37F, PH.base.pack) ;
        p108_lat_SET((float) -6.3704747E37F, PH.base.pack) ;
        p108_lon_SET((float) -3.2346636E38F, PH.base.pack) ;
        p108_q3_SET((float) -3.7699934E37F, PH.base.pack) ;
        p108_vn_SET((float) -3.3265365E38F, PH.base.pack) ;
        p108_ve_SET((float)2.4364466E38F, PH.base.pack) ;
        p108_vd_SET((float)2.8343235E38F, PH.base.pack) ;
        p108_xacc_SET((float) -1.3477078E38F, PH.base.pack) ;
        p108_zacc_SET((float)7.2927503E37F, PH.base.pack) ;
        p108_yacc_SET((float)1.846219E38F, PH.base.pack) ;
        p108_q2_SET((float) -1.1490626E38F, PH.base.pack) ;
        p108_yaw_SET((float)2.4804916E38F, PH.base.pack) ;
        p108_alt_SET((float) -8.4081474E37F, PH.base.pack) ;
        p108_ygyro_SET((float)2.3939776E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RADIO_STATUS_109(), &PH);
        p109_rxerrors_SET((uint16_t)(uint16_t)32022, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)48456, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        p110_target_system_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        p110_target_component_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        p110_target_network_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)195, (uint8_t)132, (uint8_t)148, (uint8_t)28, (uint8_t)250, (uint8_t)89, (uint8_t)130, (uint8_t)101, (uint8_t)153, (uint8_t)184, (uint8_t)71, (uint8_t)138, (uint8_t)140, (uint8_t)130, (uint8_t)126, (uint8_t)118, (uint8_t)80, (uint8_t)41, (uint8_t)201, (uint8_t)1, (uint8_t)15, (uint8_t)213, (uint8_t)72, (uint8_t)147, (uint8_t)155, (uint8_t)226, (uint8_t)65, (uint8_t)30, (uint8_t)168, (uint8_t)205, (uint8_t)170, (uint8_t)58, (uint8_t)122, (uint8_t)102, (uint8_t)67, (uint8_t)64, (uint8_t)6, (uint8_t)61, (uint8_t)80, (uint8_t)139, (uint8_t)106, (uint8_t)98, (uint8_t)161, (uint8_t)152, (uint8_t)106, (uint8_t)99, (uint8_t)153, (uint8_t)68, (uint8_t)76, (uint8_t)78, (uint8_t)206, (uint8_t)2, (uint8_t)105, (uint8_t)3, (uint8_t)85, (uint8_t)62, (uint8_t)220, (uint8_t)251, (uint8_t)127, (uint8_t)205, (uint8_t)165, (uint8_t)99, (uint8_t)98, (uint8_t)19, (uint8_t)156, (uint8_t)161, (uint8_t)150, (uint8_t)34, (uint8_t)200, (uint8_t)146, (uint8_t)112, (uint8_t)104, (uint8_t)15, (uint8_t)77, (uint8_t)46, (uint8_t)81, (uint8_t)218, (uint8_t)125, (uint8_t)128, (uint8_t)20, (uint8_t)222, (uint8_t)237, (uint8_t)158, (uint8_t)12, (uint8_t)245, (uint8_t)158, (uint8_t)238, (uint8_t)51, (uint8_t)23, (uint8_t)193, (uint8_t)249, (uint8_t)186, (uint8_t)62, (uint8_t)74, (uint8_t)62, (uint8_t)128, (uint8_t)34, (uint8_t)225, (uint8_t)0, (uint8_t)79, (uint8_t)10, (uint8_t)179, (uint8_t)200, (uint8_t)131, (uint8_t)176, (uint8_t)125, (uint8_t)246, (uint8_t)80, (uint8_t)116, (uint8_t)48, (uint8_t)120, (uint8_t)93, (uint8_t)41, (uint8_t)15, (uint8_t)238, (uint8_t)201, (uint8_t)110, (uint8_t)43, (uint8_t)89, (uint8_t)232, (uint8_t)48, (uint8_t)153, (uint8_t)124, (uint8_t)155, (uint8_t)137, (uint8_t)78, (uint8_t)231, (uint8_t)44, (uint8_t)229, (uint8_t)163, (uint8_t)122, (uint8_t)243, (uint8_t)216, (uint8_t)219, (uint8_t)191, (uint8_t)99, (uint8_t)90, (uint8_t)90, (uint8_t)47, (uint8_t)145, (uint8_t)110, (uint8_t)118, (uint8_t)47, (uint8_t)19, (uint8_t)242, (uint8_t)57, (uint8_t)3, (uint8_t)5, (uint8_t)230, (uint8_t)119, (uint8_t)91, (uint8_t)45, (uint8_t)207, (uint8_t)47, (uint8_t)27, (uint8_t)200, (uint8_t)208, (uint8_t)43, (uint8_t)26, (uint8_t)229, (uint8_t)148, (uint8_t)173, (uint8_t)96, (uint8_t)113, (uint8_t)27, (uint8_t)83, (uint8_t)226, (uint8_t)10, (uint8_t)72, (uint8_t)230, (uint8_t)201, (uint8_t)72, (uint8_t)241, (uint8_t)202, (uint8_t)239, (uint8_t)57, (uint8_t)166, (uint8_t)76, (uint8_t)112, (uint8_t)186, (uint8_t)42, (uint8_t)197, (uint8_t)62, (uint8_t)12, (uint8_t)111, (uint8_t)3, (uint8_t)133, (uint8_t)69, (uint8_t)65, (uint8_t)164, (uint8_t)104, (uint8_t)20, (uint8_t)202, (uint8_t)188, (uint8_t)161, (uint8_t)206, (uint8_t)76, (uint8_t)171, (uint8_t)149, (uint8_t)2, (uint8_t)39, (uint8_t)214, (uint8_t)36, (uint8_t)173, (uint8_t)187, (uint8_t)232, (uint8_t)125, (uint8_t)19, (uint8_t)161, (uint8_t)180, (uint8_t)174, (uint8_t)30, (uint8_t)213, (uint8_t)167, (uint8_t)151, (uint8_t)148, (uint8_t)184, (uint8_t)188, (uint8_t)141, (uint8_t)160, (uint8_t)77, (uint8_t)149, (uint8_t)169, (uint8_t)20, (uint8_t)205, (uint8_t)82, (uint8_t)168, (uint8_t)207, (uint8_t)139, (uint8_t)249, (uint8_t)22, (uint8_t)21, (uint8_t)145, (uint8_t)78, (uint8_t)16, (uint8_t)87, (uint8_t)64, (uint8_t)80, (uint8_t)252, (uint8_t)111, (uint8_t)214, (uint8_t)194, (uint8_t)44, (uint8_t)161, (uint8_t)118, (uint8_t)7, (uint8_t)33, (uint8_t)46, (uint8_t)7, (uint8_t)84, (uint8_t)73};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TIMESYNC_111(), &PH);
        p111_ts1_SET((int64_t)3110380596920743801L, PH.base.pack) ;
        p111_tc1_SET((int64_t)8489164696769321262L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_time_usec_SET((uint64_t)4428343964171788555L, PH.base.pack) ;
        p112_seq_SET((uint32_t)2418821322L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_GPS_113(), &PH);
        p113_vd_SET((int16_t)(int16_t) -17259, PH.base.pack) ;
        p113_lon_SET((int32_t)888165502, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)52344, PH.base.pack) ;
        p113_alt_SET((int32_t) -1775594596, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)6268276013888803942L, PH.base.pack) ;
        p113_lat_SET((int32_t) -804956579, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t)2173, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)25131, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)39421, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)39585, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t) -29225, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_time_usec_SET((uint64_t)600192806215029821L, PH.base.pack) ;
        p114_integrated_ygyro_SET((float)1.5032397E38F, PH.base.pack) ;
        p114_integrated_zgyro_SET((float) -9.685447E37F, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)4185809387L, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)3846312628L, PH.base.pack) ;
        p114_distance_SET((float) -1.8865199E38F, PH.base.pack) ;
        p114_integrated_xgyro_SET((float)2.2546302E38F, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t) -27837, PH.base.pack) ;
        p114_integrated_x_SET((float)1.933978E38F, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p114_integrated_y_SET((float) -3.0095987E38F, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_pitchspeed_SET((float)1.7434455E38F, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)55959, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t)22823, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)41574, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t) -699, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t) -1297, PH.base.pack) ;
        p115_alt_SET((int32_t) -1265732710, PH.base.pack) ;
        p115_yawspeed_SET((float) -3.9748234E37F, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)3018348233812227960L, PH.base.pack) ;
        p115_rollspeed_SET((float)2.9922363E38F, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t) -3060, PH.base.pack) ;
        p115_lat_SET((int32_t)502577731, PH.base.pack) ;
        p115_lon_SET((int32_t)875146244, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t)4665, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {-8.116909E37F, -2.0489463E37F, 3.3970558E38F, 8.841517E36F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_xacc_SET((int16_t)(int16_t) -15495, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU2_116(), &PH);
        p116_yacc_SET((int16_t)(int16_t)5913, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t) -22006, PH.base.pack) ;
        p116_ymag_SET((int16_t)(int16_t) -9614, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t)29886, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t) -11758, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t) -17275, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t) -10307, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t) -8097, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t)27190, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)2282362795L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_start_SET((uint16_t)(uint16_t)39470, PH.base.pack) ;
        p117_target_component_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        p117_end_SET((uint16_t)(uint16_t)21467, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ENTRY_118(), &PH);
        p118_id_SET((uint16_t)(uint16_t)33501, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)1561019446L, PH.base.pack) ;
        p118_size_SET((uint32_t)590600547L, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)40149, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)36401, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_count_SET((uint32_t)3806372522L, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        p119_ofs_SET((uint32_t)880909687L, PH.base.pack) ;
        p119_id_SET((uint16_t)(uint16_t)21497, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_DATA_120(), &PH);
        p120_ofs_SET((uint32_t)2687092424L, PH.base.pack) ;
        p120_id_SET((uint16_t)(uint16_t)63599, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)118, (uint8_t)210, (uint8_t)172, (uint8_t)248, (uint8_t)147, (uint8_t)137, (uint8_t)85, (uint8_t)29, (uint8_t)98, (uint8_t)65, (uint8_t)39, (uint8_t)51, (uint8_t)65, (uint8_t)71, (uint8_t)129, (uint8_t)20, (uint8_t)142, (uint8_t)98, (uint8_t)185, (uint8_t)52, (uint8_t)204, (uint8_t)131, (uint8_t)191, (uint8_t)93, (uint8_t)195, (uint8_t)115, (uint8_t)79, (uint8_t)24, (uint8_t)248, (uint8_t)112, (uint8_t)125, (uint8_t)136, (uint8_t)96, (uint8_t)49, (uint8_t)17, (uint8_t)172, (uint8_t)227, (uint8_t)64, (uint8_t)233, (uint8_t)96, (uint8_t)176, (uint8_t)244, (uint8_t)173, (uint8_t)88, (uint8_t)170, (uint8_t)72, (uint8_t)110, (uint8_t)29, (uint8_t)72, (uint8_t)251, (uint8_t)244, (uint8_t)211, (uint8_t)47, (uint8_t)228, (uint8_t)163, (uint8_t)252, (uint8_t)77, (uint8_t)68, (uint8_t)169, (uint8_t)253, (uint8_t)196, (uint8_t)57, (uint8_t)46, (uint8_t)82, (uint8_t)246, (uint8_t)136, (uint8_t)87, (uint8_t)210, (uint8_t)34, (uint8_t)214, (uint8_t)12, (uint8_t)163, (uint8_t)192, (uint8_t)148, (uint8_t)11, (uint8_t)158, (uint8_t)51, (uint8_t)137, (uint8_t)68, (uint8_t)237, (uint8_t)225, (uint8_t)183, (uint8_t)24, (uint8_t)37, (uint8_t)237, (uint8_t)58, (uint8_t)176, (uint8_t)106, (uint8_t)57, (uint8_t)88};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        p120_count_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ERASE_121(), &PH);
        p121_target_system_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p121_target_component_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_component_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p122_target_system_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_len_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p123_target_component_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)208, (uint8_t)166, (uint8_t)182, (uint8_t)200, (uint8_t)209, (uint8_t)209, (uint8_t)245, (uint8_t)164, (uint8_t)234, (uint8_t)60, (uint8_t)76, (uint8_t)131, (uint8_t)71, (uint8_t)226, (uint8_t)23, (uint8_t)117, (uint8_t)29, (uint8_t)4, (uint8_t)173, (uint8_t)152, (uint8_t)42, (uint8_t)246, (uint8_t)158, (uint8_t)248, (uint8_t)141, (uint8_t)225, (uint8_t)172, (uint8_t)159, (uint8_t)75, (uint8_t)39, (uint8_t)8, (uint8_t)203, (uint8_t)201, (uint8_t)105, (uint8_t)41, (uint8_t)86, (uint8_t)162, (uint8_t)157, (uint8_t)212, (uint8_t)123, (uint8_t)237, (uint8_t)192, (uint8_t)94, (uint8_t)147, (uint8_t)139, (uint8_t)53, (uint8_t)74, (uint8_t)204, (uint8_t)193, (uint8_t)228, (uint8_t)238, (uint8_t)207, (uint8_t)22, (uint8_t)153, (uint8_t)215, (uint8_t)77, (uint8_t)102, (uint8_t)191, (uint8_t)80, (uint8_t)51, (uint8_t)84, (uint8_t)140, (uint8_t)17, (uint8_t)250, (uint8_t)242, (uint8_t)185, (uint8_t)247, (uint8_t)188, (uint8_t)93, (uint8_t)33, (uint8_t)69, (uint8_t)96, (uint8_t)16, (uint8_t)169, (uint8_t)122, (uint8_t)98, (uint8_t)160, (uint8_t)42, (uint8_t)171, (uint8_t)84, (uint8_t)225, (uint8_t)37, (uint8_t)129, (uint8_t)218, (uint8_t)179, (uint8_t)50, (uint8_t)73, (uint8_t)129, (uint8_t)120, (uint8_t)125, (uint8_t)62, (uint8_t)243, (uint8_t)198, (uint8_t)79, (uint8_t)124, (uint8_t)198, (uint8_t)123, (uint8_t)143, (uint8_t)73, (uint8_t)184, (uint8_t)237, (uint8_t)88, (uint8_t)192, (uint8_t)245, (uint8_t)201, (uint8_t)146, (uint8_t)127, (uint8_t)112, (uint8_t)7, (uint8_t)87};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_target_system_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RAW_124(), &PH);
        p124_cog_SET((uint16_t)(uint16_t)3343, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, PH.base.pack) ;
        p124_lon_SET((int32_t) -64322712, PH.base.pack) ;
        p124_lat_SET((int32_t) -1764239476, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)30532, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)6704756267007216384L, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)63598, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)37576, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)1235000715L, PH.base.pack) ;
        p124_alt_SET((int32_t) -612669741, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POWER_STATUS_125(), &PH);
        p125_Vcc_SET((uint16_t)(uint16_t)21552, PH.base.pack) ;
        p125_Vservo_SET((uint16_t)(uint16_t)7725, PH.base.pack) ;
        p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_USB_CONNECTED, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_count_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        p126_timeout_SET((uint16_t)(uint16_t)724, PH.base.pack) ;
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_TELEM1, PH.base.pack) ;
        p126_baudrate_SET((uint32_t)1141564324L, PH.base.pack) ;
        p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)93, (uint8_t)136, (uint8_t)117, (uint8_t)131, (uint8_t)38, (uint8_t)30, (uint8_t)121, (uint8_t)30, (uint8_t)129, (uint8_t)143, (uint8_t)163, (uint8_t)157, (uint8_t)147, (uint8_t)54, (uint8_t)130, (uint8_t)209, (uint8_t)205, (uint8_t)26, (uint8_t)15, (uint8_t)161, (uint8_t)181, (uint8_t)251, (uint8_t)141, (uint8_t)147, (uint8_t)110, (uint8_t)232, (uint8_t)44, (uint8_t)188, (uint8_t)69, (uint8_t)238, (uint8_t)166, (uint8_t)220, (uint8_t)107, (uint8_t)220, (uint8_t)92, (uint8_t)204, (uint8_t)121, (uint8_t)79, (uint8_t)155, (uint8_t)162, (uint8_t)31, (uint8_t)113, (uint8_t)36, (uint8_t)179, (uint8_t)185, (uint8_t)245, (uint8_t)138, (uint8_t)159, (uint8_t)114, (uint8_t)20, (uint8_t)163, (uint8_t)60, (uint8_t)38, (uint8_t)124, (uint8_t)68, (uint8_t)95, (uint8_t)183, (uint8_t)145, (uint8_t)203, (uint8_t)195, (uint8_t)174, (uint8_t)47, (uint8_t)107, (uint8_t)52, (uint8_t)235, (uint8_t)204, (uint8_t)55, (uint8_t)2, (uint8_t)37, (uint8_t)88};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTK_127(), &PH);
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)255, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t) -766641944, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t) -1640939192, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)15337, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)1816118331L, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)669787245L, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t) -1233024591, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)177, PH.base.pack) ;
        p127_tow_SET((uint32_t)4147896568L, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t)49578127, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RTK_128(), &PH);
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)39656706L, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t) -911973171, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t) -761973663, PH.base.pack) ;
        p128_tow_SET((uint32_t)4120914998L, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t) -61766415, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)51296, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)1179054331L, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t) -1470565959, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU3_129(), &PH);
        p129_zgyro_SET((int16_t)(int16_t)11713, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t)25704, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)1391269655L, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t) -14883, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t)16314, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t)23639, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t)12139, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t)19009, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t) -4747, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t)18382, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_height_SET((uint16_t)(uint16_t)44425, PH.base.pack) ;
        p130_size_SET((uint32_t)432989241L, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)12654, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)2022, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ENCAPSULATED_DATA_131(), &PH);
        p131_seqnr_SET((uint16_t)(uint16_t)12511, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)169, (uint8_t)200, (uint8_t)128, (uint8_t)87, (uint8_t)13, (uint8_t)127, (uint8_t)157, (uint8_t)7, (uint8_t)106, (uint8_t)66, (uint8_t)87, (uint8_t)90, (uint8_t)119, (uint8_t)32, (uint8_t)174, (uint8_t)37, (uint8_t)215, (uint8_t)69, (uint8_t)193, (uint8_t)21, (uint8_t)11, (uint8_t)63, (uint8_t)99, (uint8_t)41, (uint8_t)53, (uint8_t)81, (uint8_t)251, (uint8_t)253, (uint8_t)14, (uint8_t)253, (uint8_t)18, (uint8_t)195, (uint8_t)216, (uint8_t)165, (uint8_t)50, (uint8_t)141, (uint8_t)225, (uint8_t)64, (uint8_t)36, (uint8_t)64, (uint8_t)77, (uint8_t)130, (uint8_t)188, (uint8_t)134, (uint8_t)97, (uint8_t)174, (uint8_t)127, (uint8_t)90, (uint8_t)122, (uint8_t)76, (uint8_t)51, (uint8_t)90, (uint8_t)90, (uint8_t)242, (uint8_t)58, (uint8_t)107, (uint8_t)234, (uint8_t)74, (uint8_t)50, (uint8_t)225, (uint8_t)183, (uint8_t)77, (uint8_t)59, (uint8_t)160, (uint8_t)54, (uint8_t)148, (uint8_t)80, (uint8_t)169, (uint8_t)50, (uint8_t)153, (uint8_t)58, (uint8_t)78, (uint8_t)54, (uint8_t)125, (uint8_t)193, (uint8_t)146, (uint8_t)140, (uint8_t)19, (uint8_t)247, (uint8_t)249, (uint8_t)208, (uint8_t)123, (uint8_t)232, (uint8_t)29, (uint8_t)203, (uint8_t)188, (uint8_t)239, (uint8_t)59, (uint8_t)70, (uint8_t)59, (uint8_t)90, (uint8_t)203, (uint8_t)198, (uint8_t)233, (uint8_t)185, (uint8_t)169, (uint8_t)76, (uint8_t)209, (uint8_t)202, (uint8_t)39, (uint8_t)156, (uint8_t)222, (uint8_t)93, (uint8_t)208, (uint8_t)57, (uint8_t)78, (uint8_t)174, (uint8_t)36, (uint8_t)170, (uint8_t)214, (uint8_t)251, (uint8_t)155, (uint8_t)72, (uint8_t)62, (uint8_t)32, (uint8_t)248, (uint8_t)146, (uint8_t)133, (uint8_t)182, (uint8_t)37, (uint8_t)106, (uint8_t)173, (uint8_t)57, (uint8_t)221, (uint8_t)201, (uint8_t)204, (uint8_t)150, (uint8_t)6, (uint8_t)10, (uint8_t)137, (uint8_t)99, (uint8_t)250, (uint8_t)142, (uint8_t)156, (uint8_t)211, (uint8_t)210, (uint8_t)152, (uint8_t)143, (uint8_t)226, (uint8_t)56, (uint8_t)194, (uint8_t)2, (uint8_t)139, (uint8_t)235, (uint8_t)110, (uint8_t)215, (uint8_t)32, (uint8_t)211, (uint8_t)166, (uint8_t)161, (uint8_t)175, (uint8_t)10, (uint8_t)213, (uint8_t)129, (uint8_t)118, (uint8_t)231, (uint8_t)128, (uint8_t)170, (uint8_t)192, (uint8_t)244, (uint8_t)64, (uint8_t)148, (uint8_t)169, (uint8_t)31, (uint8_t)139, (uint8_t)146, (uint8_t)244, (uint8_t)50, (uint8_t)11, (uint8_t)222, (uint8_t)80, (uint8_t)32, (uint8_t)77, (uint8_t)69, (uint8_t)83, (uint8_t)247, (uint8_t)170, (uint8_t)166, (uint8_t)131, (uint8_t)22, (uint8_t)196, (uint8_t)17, (uint8_t)161, (uint8_t)191, (uint8_t)59, (uint8_t)241, (uint8_t)2, (uint8_t)199, (uint8_t)83, (uint8_t)44, (uint8_t)150, (uint8_t)31, (uint8_t)243, (uint8_t)71, (uint8_t)161, (uint8_t)10, (uint8_t)62, (uint8_t)181, (uint8_t)205, (uint8_t)176, (uint8_t)56, (uint8_t)218, (uint8_t)32, (uint8_t)17, (uint8_t)73, (uint8_t)126, (uint8_t)192, (uint8_t)196, (uint8_t)12, (uint8_t)200, (uint8_t)104, (uint8_t)194, (uint8_t)12, (uint8_t)84, (uint8_t)52, (uint8_t)40, (uint8_t)167, (uint8_t)26, (uint8_t)207, (uint8_t)200, (uint8_t)236, (uint8_t)128, (uint8_t)230, (uint8_t)12, (uint8_t)7, (uint8_t)250, (uint8_t)71, (uint8_t)2, (uint8_t)235, (uint8_t)203, (uint8_t)6, (uint8_t)69, (uint8_t)7, (uint8_t)75, (uint8_t)65, (uint8_t)189, (uint8_t)101, (uint8_t)164, (uint8_t)151, (uint8_t)164, (uint8_t)77, (uint8_t)168, (uint8_t)46, (uint8_t)246, (uint8_t)216, (uint8_t)10, (uint8_t)57, (uint8_t)121, (uint8_t)145, (uint8_t)118, (uint8_t)36, (uint8_t)28, (uint8_t)113};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_id_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)753725895L, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)35806, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_UNKNOWN, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)3838, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)44083, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_NONE, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_lon_SET((int32_t) -1825795948, PH.base.pack) ;
        p133_grid_spacing_SET((uint16_t)(uint16_t)24495, PH.base.pack) ;
        p133_lat_SET((int32_t) -1764712385, PH.base.pack) ;
        p133_mask_SET((uint64_t)6261140212422089752L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_DATA_134(), &PH);
        p134_lon_SET((int32_t) -1760890676, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t)27833, (int16_t)22673, (int16_t)14677, (int16_t)19443, (int16_t)30669, (int16_t) -29870, (int16_t)27554, (int16_t)18946, (int16_t)11349, (int16_t)2864, (int16_t) -1610, (int16_t) -30786, (int16_t) -25067, (int16_t)8214, (int16_t) -15260, (int16_t) -6677};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        p134_gridbit_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        p134_grid_spacing_SET((uint16_t)(uint16_t)43813, PH.base.pack) ;
        p134_lat_SET((int32_t)1590518892, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lat_SET((int32_t)140754526, PH.base.pack) ;
        p135_lon_SET((int32_t) -1113134223, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_pending_SET((uint16_t)(uint16_t)41490, PH.base.pack) ;
        p136_terrain_height_SET((float)3.2869115E38F, PH.base.pack) ;
        p136_lon_SET((int32_t) -640064180, PH.base.pack) ;
        p136_lat_SET((int32_t) -818639174, PH.base.pack) ;
        p136_spacing_SET((uint16_t)(uint16_t)38374, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)15156, PH.base.pack) ;
        p136_current_height_SET((float) -1.5538309E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_press_abs_SET((float)2.5038456E38F, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t) -30783, PH.base.pack) ;
        p137_press_diff_SET((float) -1.5110764E37F, PH.base.pack) ;
        p137_time_boot_ms_SET((uint32_t)1533155839L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATT_POS_MOCAP_138(), &PH);
        {
            float q[] =  {3.090613E38F, -2.5215609E38F, 1.9434485E38F, 2.857603E38F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_z_SET((float) -2.8343605E38F, PH.base.pack) ;
        p138_x_SET((float)3.3864838E38F, PH.base.pack) ;
        p138_y_SET((float)2.8904693E37F, PH.base.pack) ;
        p138_time_usec_SET((uint64_t)6130085898167468531L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        {
            float controls[] =  {-5.343814E37F, 2.4513756E38F, -3.862387E37F, -1.9150422E38F, -1.757943E38F, 6.4529404E37F, -6.4053153E37F, 1.954266E38F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_target_component_SET((uint8_t)(uint8_t)93, PH.base.pack) ;
        p139_group_mlx_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p139_time_usec_SET((uint64_t)7180790925433005027L, PH.base.pack) ;
        p139_target_system_SET((uint8_t)(uint8_t)193, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        p140_time_usec_SET((uint64_t)673306687967548736L, PH.base.pack) ;
        {
            float controls[] =  {-1.725789E38F, 1.362936E37F, 9.010594E37F, -1.5440354E38F, -3.2248465E38F, 1.4358472E37F, -4.661489E37F, -1.462023E38F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p140_group_mlx_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ALTITUDE_141(), &PH);
        p141_altitude_local_SET((float) -2.2252118E38F, PH.base.pack) ;
        p141_altitude_relative_SET((float)4.719326E37F, PH.base.pack) ;
        p141_altitude_monotonic_SET((float)1.063608E38F, PH.base.pack) ;
        p141_altitude_amsl_SET((float) -2.66418E38F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)6223877247300762799L, PH.base.pack) ;
        p141_altitude_terrain_SET((float)1.6658766E38F, PH.base.pack) ;
        p141_bottom_clearance_SET((float) -3.2258004E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RESOURCE_REQUEST_142(), &PH);
        p142_transfer_type_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        {
            uint8_t storage[] =  {(uint8_t)131, (uint8_t)181, (uint8_t)55, (uint8_t)15, (uint8_t)199, (uint8_t)199, (uint8_t)199, (uint8_t)201, (uint8_t)183, (uint8_t)198, (uint8_t)158, (uint8_t)173, (uint8_t)88, (uint8_t)144, (uint8_t)28, (uint8_t)126, (uint8_t)83, (uint8_t)23, (uint8_t)32, (uint8_t)52, (uint8_t)100, (uint8_t)176, (uint8_t)104, (uint8_t)220, (uint8_t)44, (uint8_t)142, (uint8_t)158, (uint8_t)193, (uint8_t)57, (uint8_t)82, (uint8_t)125, (uint8_t)132, (uint8_t)30, (uint8_t)116, (uint8_t)206, (uint8_t)163, (uint8_t)221, (uint8_t)156, (uint8_t)228, (uint8_t)22, (uint8_t)180, (uint8_t)129, (uint8_t)122, (uint8_t)110, (uint8_t)145, (uint8_t)92, (uint8_t)91, (uint8_t)200, (uint8_t)239, (uint8_t)192, (uint8_t)23, (uint8_t)206, (uint8_t)223, (uint8_t)199, (uint8_t)139, (uint8_t)177, (uint8_t)240, (uint8_t)46, (uint8_t)209, (uint8_t)80, (uint8_t)31, (uint8_t)150, (uint8_t)101, (uint8_t)122, (uint8_t)140, (uint8_t)139, (uint8_t)110, (uint8_t)187, (uint8_t)55, (uint8_t)85, (uint8_t)30, (uint8_t)215, (uint8_t)210, (uint8_t)42, (uint8_t)169, (uint8_t)44, (uint8_t)96, (uint8_t)241, (uint8_t)237, (uint8_t)92, (uint8_t)109, (uint8_t)30, (uint8_t)172, (uint8_t)200, (uint8_t)151, (uint8_t)225, (uint8_t)31, (uint8_t)248, (uint8_t)6, (uint8_t)59, (uint8_t)183, (uint8_t)11, (uint8_t)167, (uint8_t)39, (uint8_t)102, (uint8_t)221, (uint8_t)141, (uint8_t)113, (uint8_t)229, (uint8_t)213, (uint8_t)200, (uint8_t)187, (uint8_t)237, (uint8_t)138, (uint8_t)155, (uint8_t)36, (uint8_t)104, (uint8_t)200, (uint8_t)71, (uint8_t)254, (uint8_t)14, (uint8_t)154, (uint8_t)63, (uint8_t)43, (uint8_t)143, (uint8_t)136, (uint8_t)239, (uint8_t)9, (uint8_t)112, (uint8_t)34};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        p142_uri_type_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        p142_request_id_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        {
            uint8_t uri[] =  {(uint8_t)75, (uint8_t)221, (uint8_t)216, (uint8_t)38, (uint8_t)73, (uint8_t)59, (uint8_t)98, (uint8_t)134, (uint8_t)179, (uint8_t)24, (uint8_t)117, (uint8_t)110, (uint8_t)67, (uint8_t)160, (uint8_t)93, (uint8_t)122, (uint8_t)148, (uint8_t)138, (uint8_t)86, (uint8_t)76, (uint8_t)172, (uint8_t)229, (uint8_t)72, (uint8_t)36, (uint8_t)155, (uint8_t)153, (uint8_t)175, (uint8_t)242, (uint8_t)133, (uint8_t)24, (uint8_t)205, (uint8_t)22, (uint8_t)241, (uint8_t)177, (uint8_t)215, (uint8_t)149, (uint8_t)30, (uint8_t)168, (uint8_t)143, (uint8_t)80, (uint8_t)13, (uint8_t)142, (uint8_t)29, (uint8_t)70, (uint8_t)175, (uint8_t)210, (uint8_t)247, (uint8_t)227, (uint8_t)23, (uint8_t)138, (uint8_t)106, (uint8_t)222, (uint8_t)223, (uint8_t)39, (uint8_t)72, (uint8_t)29, (uint8_t)10, (uint8_t)241, (uint8_t)80, (uint8_t)239, (uint8_t)75, (uint8_t)14, (uint8_t)140, (uint8_t)86, (uint8_t)225, (uint8_t)47, (uint8_t)200, (uint8_t)8, (uint8_t)230, (uint8_t)37, (uint8_t)153, (uint8_t)246, (uint8_t)192, (uint8_t)125, (uint8_t)235, (uint8_t)18, (uint8_t)19, (uint8_t)87, (uint8_t)148, (uint8_t)204, (uint8_t)103, (uint8_t)114, (uint8_t)221, (uint8_t)189, (uint8_t)2, (uint8_t)71, (uint8_t)190, (uint8_t)222, (uint8_t)211, (uint8_t)241, (uint8_t)68, (uint8_t)29, (uint8_t)240, (uint8_t)189, (uint8_t)230, (uint8_t)174, (uint8_t)213, (uint8_t)55, (uint8_t)141, (uint8_t)102, (uint8_t)86, (uint8_t)13, (uint8_t)224, (uint8_t)241, (uint8_t)102, (uint8_t)211, (uint8_t)131, (uint8_t)60, (uint8_t)139, (uint8_t)145, (uint8_t)1, (uint8_t)134, (uint8_t)85, (uint8_t)27, (uint8_t)2, (uint8_t)233, (uint8_t)78, (uint8_t)186, (uint8_t)17, (uint8_t)143};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_temperature_SET((int16_t)(int16_t) -51, PH.base.pack) ;
        p143_press_abs_SET((float)2.52316E37F, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)66829004L, PH.base.pack) ;
        p143_press_diff_SET((float) -1.6845655E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FOLLOW_TARGET_144(), &PH);
        {
            float position_cov[] =  {3.0799354E38F, 1.6351668E38F, 1.6545734E38F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        p144_est_capabilities_SET((uint8_t)(uint8_t)250, PH.base.pack) ;
        {
            float acc[] =  {3.3329078E38F, -1.342158E38F, 1.4809308E37F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        {
            float vel[] =  {2.2025005E38F, -6.8749173E36F, -3.3925957E38F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        p144_custom_state_SET((uint64_t)2717888927284365146L, PH.base.pack) ;
        p144_alt_SET((float)1.5226047E37F, PH.base.pack) ;
        p144_lat_SET((int32_t)1556273008, PH.base.pack) ;
        p144_lon_SET((int32_t)1375766043, PH.base.pack) ;
        {
            float attitude_q[] =  {-3.1413248E38F, 2.6149843E38F, 1.0600463E38F, -1.0675729E37F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        p144_timestamp_SET((uint64_t)6792780101266041577L, PH.base.pack) ;
        {
            float rates[] =  {-1.5395566E37F, -2.6846333E36F, -8.607719E37F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_x_vel_SET((float) -3.0191914E38F, PH.base.pack) ;
        p146_x_acc_SET((float) -9.888169E37F, PH.base.pack) ;
        p146_pitch_rate_SET((float)2.7584817E38F, PH.base.pack) ;
        p146_y_acc_SET((float) -5.0228343E37F, PH.base.pack) ;
        p146_z_vel_SET((float)3.2853638E38F, PH.base.pack) ;
        p146_time_usec_SET((uint64_t)8459229483011361321L, PH.base.pack) ;
        p146_z_acc_SET((float) -6.153206E37F, PH.base.pack) ;
        {
            float q[] =  {-2.4306344E38F, -2.0323453E38F, -2.2395342E38F, 8.215653E37F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        p146_z_pos_SET((float) -1.6894234E38F, PH.base.pack) ;
        {
            float vel_variance[] =  {2.2286768E38F, 2.6500471E38F, -5.46416E37F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_roll_rate_SET((float)9.435573E37F, PH.base.pack) ;
        p146_airspeed_SET((float)5.704219E37F, PH.base.pack) ;
        p146_y_vel_SET((float)3.273235E38F, PH.base.pack) ;
        p146_yaw_rate_SET((float) -8.4477514E37F, PH.base.pack) ;
        p146_y_pos_SET((float) -3.0764189E38F, PH.base.pack) ;
        p146_x_pos_SET((float) -7.394305E37F, PH.base.pack) ;
        {
            float pos_variance[] =  {4.1987072E37F, 3.057614E38F, -2.8744436E38F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BATTERY_STATUS_147(), &PH);
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_NIMH, PH.base.pack) ;
        p147_energy_consumed_SET((int32_t) -1729670274, PH.base.pack) ;
        p147_current_consumed_SET((int32_t) -91129369, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t)15305, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)30069, (uint16_t)12518, (uint16_t)30842, (uint16_t)20782, (uint16_t)16283, (uint16_t)13262, (uint16_t)40865, (uint16_t)64318, (uint16_t)9993, (uint16_t)6006};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_id_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t) -100, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t)8174, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_ALL, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_148(), &PH);
        p148_middleware_sw_version_SET((uint32_t)2338041536L, PH.base.pack) ;
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)230, (uint8_t)83, (uint8_t)16, (uint8_t)95, (uint8_t)12, (uint8_t)22, (uint8_t)86, (uint8_t)200};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        {
            uint8_t uid2[] =  {(uint8_t)175, (uint8_t)84, (uint8_t)201, (uint8_t)1, (uint8_t)78, (uint8_t)239, (uint8_t)49, (uint8_t)205, (uint8_t)81, (uint8_t)95, (uint8_t)189, (uint8_t)11, (uint8_t)155, (uint8_t)94, (uint8_t)233, (uint8_t)55, (uint8_t)114, (uint8_t)23};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_flight_sw_version_SET((uint32_t)745415640L, PH.base.pack) ;
        p148_vendor_id_SET((uint16_t)(uint16_t)9999, PH.base.pack) ;
        p148_os_sw_version_SET((uint32_t)1271115204L, PH.base.pack) ;
        {
            uint8_t flight_custom_version[] =  {(uint8_t)90, (uint8_t)79, (uint8_t)30, (uint8_t)70, (uint8_t)158, (uint8_t)221, (uint8_t)191, (uint8_t)254};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MAVLINK2, PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)10, (uint8_t)107, (uint8_t)16, (uint8_t)108, (uint8_t)230, (uint8_t)78, (uint8_t)153, (uint8_t)5};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        p148_board_version_SET((uint32_t)2775529188L, PH.base.pack) ;
        p148_uid_SET((uint64_t)8599935115229991113L, PH.base.pack) ;
        p148_product_id_SET((uint16_t)(uint16_t)60905, PH.base.pack) ;
        c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LANDING_TARGET_149(), &PH);
        p149_distance_SET((float)2.936526E38F, PH.base.pack) ;
        p149_angle_y_SET((float)1.0874596E38F, PH.base.pack) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p149_angle_x_SET((float) -7.4662354E37F, PH.base.pack) ;
        p149_size_x_SET((float)1.8853116E38F, PH.base.pack) ;
        p149_time_usec_SET((uint64_t)4903025259323122880L, PH.base.pack) ;
        p149_target_num_SET((uint8_t)(uint8_t)214, PH.base.pack) ;
        p149_y_SET((float) -2.1549141E38F, &PH) ;
        p149_size_y_SET((float) -9.713978E37F, PH.base.pack) ;
        p149_z_SET((float)4.6078536E37F, &PH) ;
        p149_x_SET((float) -1.2569357E38F, &PH) ;
        p149_position_valid_SET((uint8_t)(uint8_t)30, &PH) ;
        {
            float q[] =  {-1.2245927E38F, 1.4668084E38F, 5.591384E37F, -1.171001E38F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SENSOR_OFFSETS_150(), &PH);
        p150_accel_cal_z_SET((float) -2.2330706E38F, PH.base.pack) ;
        p150_raw_press_SET((int32_t) -1579267925, PH.base.pack) ;
        p150_accel_cal_x_SET((float)2.860304E38F, PH.base.pack) ;
        p150_accel_cal_y_SET((float) -1.5397079E38F, PH.base.pack) ;
        p150_gyro_cal_y_SET((float)3.1044122E38F, PH.base.pack) ;
        p150_mag_ofs_x_SET((int16_t)(int16_t) -15522, PH.base.pack) ;
        p150_gyro_cal_x_SET((float)2.1309393E38F, PH.base.pack) ;
        p150_mag_declination_SET((float)1.2611883E38F, PH.base.pack) ;
        p150_gyro_cal_z_SET((float)7.012354E37F, PH.base.pack) ;
        p150_raw_temp_SET((int32_t) -594743805, PH.base.pack) ;
        p150_mag_ofs_z_SET((int16_t)(int16_t)20377, PH.base.pack) ;
        p150_mag_ofs_y_SET((int16_t)(int16_t) -16837, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SENSOR_OFFSETS_150(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_MAG_OFFSETS_151(), &PH);
        p151_target_component_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        p151_mag_ofs_y_SET((int16_t)(int16_t) -14157, PH.base.pack) ;
        p151_mag_ofs_z_SET((int16_t)(int16_t)16316, PH.base.pack) ;
        p151_mag_ofs_x_SET((int16_t)(int16_t) -4731, PH.base.pack) ;
        p151_target_system_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_MAG_OFFSETS_151(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MEMINFO_152(), &PH);
        p152_freemem_SET((uint16_t)(uint16_t)36412, PH.base.pack) ;
        p152_brkval_SET((uint16_t)(uint16_t)46559, PH.base.pack) ;
        p152_freemem32_SET((uint32_t)3485485193L, &PH) ;
        c_LoopBackDemoChannel_on_MEMINFO_152(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AP_ADC_153(), &PH);
        p153_adc2_SET((uint16_t)(uint16_t)18073, PH.base.pack) ;
        p153_adc4_SET((uint16_t)(uint16_t)44077, PH.base.pack) ;
        p153_adc1_SET((uint16_t)(uint16_t)45797, PH.base.pack) ;
        p153_adc5_SET((uint16_t)(uint16_t)42873, PH.base.pack) ;
        p153_adc3_SET((uint16_t)(uint16_t)32538, PH.base.pack) ;
        p153_adc6_SET((uint16_t)(uint16_t)21353, PH.base.pack) ;
        c_LoopBackDemoChannel_on_AP_ADC_153(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DIGICAM_CONFIGURE_154(), &PH);
        p154_extra_param_SET((uint8_t)(uint8_t)172, PH.base.pack) ;
        p154_iso_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
        p154_target_system_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        p154_command_id_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        p154_shutter_speed_SET((uint16_t)(uint16_t)30584, PH.base.pack) ;
        p154_mode_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        p154_exposure_type_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p154_aperture_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        p154_target_component_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
        p154_engine_cut_off_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        p154_extra_value_SET((float)1.6760428E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DIGICAM_CONFIGURE_154(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DIGICAM_CONTROL_155(), &PH);
        p155_shot_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        p155_command_id_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p155_target_component_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
        p155_zoom_step_SET((int8_t)(int8_t)102, PH.base.pack) ;
        p155_target_system_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p155_extra_param_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        p155_zoom_pos_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p155_focus_lock_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        p155_extra_value_SET((float) -2.9372049E38F, PH.base.pack) ;
        p155_session_SET((uint8_t)(uint8_t)56, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DIGICAM_CONTROL_155(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MOUNT_CONFIGURE_156(), &PH);
        p156_target_component_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        p156_mount_mode_SET(e_MAV_MOUNT_MODE_MAV_MOUNT_MODE_NEUTRAL, PH.base.pack) ;
        p156_stab_pitch_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        p156_stab_roll_SET((uint8_t)(uint8_t)70, PH.base.pack) ;
        p156_stab_yaw_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        p156_target_system_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MOUNT_CONFIGURE_156(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MOUNT_CONTROL_157(), &PH);
        p157_save_position_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p157_input_b_SET((int32_t) -2018012814, PH.base.pack) ;
        p157_input_a_SET((int32_t) -2035463648, PH.base.pack) ;
        p157_input_c_SET((int32_t) -312038060, PH.base.pack) ;
        p157_target_system_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
        p157_target_component_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MOUNT_CONTROL_157(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MOUNT_STATUS_158(), &PH);
        p158_pointing_a_SET((int32_t) -1590969205, PH.base.pack) ;
        p158_pointing_b_SET((int32_t)718597256, PH.base.pack) ;
        p158_pointing_c_SET((int32_t) -815885501, PH.base.pack) ;
        p158_target_component_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        p158_target_system_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MOUNT_STATUS_158(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FENCE_POINT_160(), &PH);
        p160_count_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        p160_lat_SET((float)1.9412977E37F, PH.base.pack) ;
        p160_target_component_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p160_idx_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        p160_target_system_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p160_lng_SET((float) -1.5820584E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FENCE_POINT_160(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FENCE_FETCH_POINT_161(), &PH);
        p161_target_system_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p161_target_component_SET((uint8_t)(uint8_t)252, PH.base.pack) ;
        p161_idx_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FENCE_FETCH_POINT_161(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FENCE_STATUS_162(), &PH);
        p162_breach_time_SET((uint32_t)479161813L, PH.base.pack) ;
        p162_breach_status_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p162_breach_count_SET((uint16_t)(uint16_t)9760, PH.base.pack) ;
        p162_breach_type_SET(e_FENCE_BREACH_FENCE_BREACH_MAXALT, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FENCE_STATUS_162(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AHRS_163(), &PH);
        p163_renorm_val_SET((float) -9.525016E37F, PH.base.pack) ;
        p163_accel_weight_SET((float) -1.1451615E38F, PH.base.pack) ;
        p163_omegaIy_SET((float) -1.5669243E38F, PH.base.pack) ;
        p163_omegaIx_SET((float)6.618985E37F, PH.base.pack) ;
        p163_error_yaw_SET((float) -9.424832E37F, PH.base.pack) ;
        p163_error_rp_SET((float)2.7917982E38F, PH.base.pack) ;
        p163_omegaIz_SET((float) -2.3994788E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_AHRS_163(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SIMSTATE_164(), &PH);
        p164_lng_SET((int32_t) -1179316840, PH.base.pack) ;
        p164_yaw_SET((float)9.282846E37F, PH.base.pack) ;
        p164_roll_SET((float)1.5977427E38F, PH.base.pack) ;
        p164_xacc_SET((float) -1.8808295E37F, PH.base.pack) ;
        p164_lat_SET((int32_t)117311869, PH.base.pack) ;
        p164_zacc_SET((float) -3.0120283E38F, PH.base.pack) ;
        p164_xgyro_SET((float) -1.7176615E38F, PH.base.pack) ;
        p164_ygyro_SET((float)3.1385443E38F, PH.base.pack) ;
        p164_zgyro_SET((float) -2.4914583E38F, PH.base.pack) ;
        p164_pitch_SET((float) -2.1396447E38F, PH.base.pack) ;
        p164_yacc_SET((float)1.6983207E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SIMSTATE_164(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HWSTATUS_165(), &PH);
        p165_Vcc_SET((uint16_t)(uint16_t)58304, PH.base.pack) ;
        p165_I2Cerr_SET((uint8_t)(uint8_t)127, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HWSTATUS_165(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RADIO_166(), &PH);
        p166_remnoise_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        p166_remrssi_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        p166_txbuf_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p166_fixed__SET((uint16_t)(uint16_t)4775, PH.base.pack) ;
        p166_noise_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p166_rssi_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
        p166_rxerrors_SET((uint16_t)(uint16_t)634, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RADIO_166(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LIMITS_STATUS_167(), &PH);
        p167_mods_required_SET(e_LIMIT_MODULE_LIMIT_GPSLOCK, PH.base.pack) ;
        p167_limits_state_SET(e_LIMITS_STATE_LIMITS_TRIGGERED, PH.base.pack) ;
        p167_last_action_SET((uint32_t)1278445368L, PH.base.pack) ;
        p167_mods_triggered_SET(e_LIMIT_MODULE_LIMIT_ALTITUDE, PH.base.pack) ;
        p167_last_clear_SET((uint32_t)2064376321L, PH.base.pack) ;
        p167_breach_count_SET((uint16_t)(uint16_t)36529, PH.base.pack) ;
        p167_mods_enabled_SET(e_LIMIT_MODULE_LIMIT_GPSLOCK, PH.base.pack) ;
        p167_last_trigger_SET((uint32_t)1054747665L, PH.base.pack) ;
        p167_last_recovery_SET((uint32_t)2342811393L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LIMITS_STATUS_167(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIND_168(), &PH);
        p168_speed_SET((float) -2.2988727E37F, PH.base.pack) ;
        p168_direction_SET((float) -2.9902588E38F, PH.base.pack) ;
        p168_speed_z_SET((float) -7.2069735E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_WIND_168(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA16_169(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)214, (uint8_t)104, (uint8_t)151, (uint8_t)241, (uint8_t)120, (uint8_t)97, (uint8_t)129, (uint8_t)96, (uint8_t)185, (uint8_t)245, (uint8_t)115, (uint8_t)235, (uint8_t)35, (uint8_t)49, (uint8_t)162, (uint8_t)213};
            p169_data__SET(&data_, 0, PH.base.pack) ;
        }
        p169_type_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p169_len_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA16_169(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA32_170(), &PH);
        p170_type_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        p170_len_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)229, (uint8_t)206, (uint8_t)197, (uint8_t)206, (uint8_t)99, (uint8_t)225, (uint8_t)48, (uint8_t)30, (uint8_t)53, (uint8_t)32, (uint8_t)123, (uint8_t)16, (uint8_t)95, (uint8_t)83, (uint8_t)93, (uint8_t)230, (uint8_t)40, (uint8_t)246, (uint8_t)40, (uint8_t)67, (uint8_t)150, (uint8_t)253, (uint8_t)188, (uint8_t)248, (uint8_t)202, (uint8_t)38, (uint8_t)71, (uint8_t)170, (uint8_t)218, (uint8_t)125, (uint8_t)228, (uint8_t)170};
            p170_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_DATA32_170(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA64_171(), &PH);
        p171_type_SET((uint8_t)(uint8_t)165, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)230, (uint8_t)91, (uint8_t)101, (uint8_t)36, (uint8_t)244, (uint8_t)16, (uint8_t)225, (uint8_t)68, (uint8_t)107, (uint8_t)114, (uint8_t)43, (uint8_t)48, (uint8_t)52, (uint8_t)104, (uint8_t)12, (uint8_t)190, (uint8_t)7, (uint8_t)117, (uint8_t)208, (uint8_t)0, (uint8_t)97, (uint8_t)105, (uint8_t)179, (uint8_t)142, (uint8_t)120, (uint8_t)205, (uint8_t)238, (uint8_t)168, (uint8_t)209, (uint8_t)81, (uint8_t)43, (uint8_t)63, (uint8_t)140, (uint8_t)67, (uint8_t)82, (uint8_t)177, (uint8_t)156, (uint8_t)116, (uint8_t)63, (uint8_t)217, (uint8_t)205, (uint8_t)174, (uint8_t)175, (uint8_t)144, (uint8_t)186, (uint8_t)6, (uint8_t)72, (uint8_t)126, (uint8_t)75, (uint8_t)207, (uint8_t)161, (uint8_t)190, (uint8_t)128, (uint8_t)182, (uint8_t)6, (uint8_t)154, (uint8_t)52, (uint8_t)138, (uint8_t)66, (uint8_t)97, (uint8_t)208, (uint8_t)139, (uint8_t)128, (uint8_t)13};
            p171_data__SET(&data_, 0, PH.base.pack) ;
        }
        p171_len_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA64_171(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA96_172(), &PH);
        p172_type_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p172_len_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)233, (uint8_t)89, (uint8_t)249, (uint8_t)137, (uint8_t)147, (uint8_t)81, (uint8_t)158, (uint8_t)173, (uint8_t)125, (uint8_t)230, (uint8_t)100, (uint8_t)40, (uint8_t)108, (uint8_t)238, (uint8_t)96, (uint8_t)5, (uint8_t)111, (uint8_t)53, (uint8_t)254, (uint8_t)167, (uint8_t)42, (uint8_t)221, (uint8_t)126, (uint8_t)5, (uint8_t)175, (uint8_t)104, (uint8_t)30, (uint8_t)215, (uint8_t)83, (uint8_t)79, (uint8_t)181, (uint8_t)123, (uint8_t)154, (uint8_t)8, (uint8_t)196, (uint8_t)8, (uint8_t)129, (uint8_t)84, (uint8_t)12, (uint8_t)75, (uint8_t)193, (uint8_t)151, (uint8_t)250, (uint8_t)249, (uint8_t)129, (uint8_t)203, (uint8_t)15, (uint8_t)131, (uint8_t)26, (uint8_t)34, (uint8_t)86, (uint8_t)64, (uint8_t)130, (uint8_t)175, (uint8_t)164, (uint8_t)251, (uint8_t)235, (uint8_t)47, (uint8_t)26, (uint8_t)10, (uint8_t)128, (uint8_t)212, (uint8_t)77, (uint8_t)248, (uint8_t)249, (uint8_t)64, (uint8_t)189, (uint8_t)66, (uint8_t)71, (uint8_t)221, (uint8_t)38, (uint8_t)60, (uint8_t)51, (uint8_t)245, (uint8_t)90, (uint8_t)60, (uint8_t)194, (uint8_t)163, (uint8_t)105, (uint8_t)138, (uint8_t)128, (uint8_t)189, (uint8_t)231, (uint8_t)103, (uint8_t)21, (uint8_t)206, (uint8_t)242, (uint8_t)44, (uint8_t)159, (uint8_t)53, (uint8_t)231, (uint8_t)135, (uint8_t)39, (uint8_t)29, (uint8_t)244, (uint8_t)77};
            p172_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_DATA96_172(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RANGEFINDER_173(), &PH);
        p173_voltage_SET((float)1.1511416E38F, PH.base.pack) ;
        p173_distance_SET((float)8.775278E36F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RANGEFINDER_173(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AIRSPEED_AUTOCAL_174(), &PH);
        p174_vy_SET((float)1.5481385E38F, PH.base.pack) ;
        p174_Pax_SET((float) -2.7244335E38F, PH.base.pack) ;
        p174_diff_pressure_SET((float)2.627316E38F, PH.base.pack) ;
        p174_Pby_SET((float) -2.5670368E37F, PH.base.pack) ;
        p174_Pcz_SET((float) -3.319359E37F, PH.base.pack) ;
        p174_state_x_SET((float)2.3604212E38F, PH.base.pack) ;
        p174_vx_SET((float) -2.699091E37F, PH.base.pack) ;
        p174_ratio_SET((float)2.2589408E38F, PH.base.pack) ;
        p174_state_z_SET((float)1.7863696E36F, PH.base.pack) ;
        p174_state_y_SET((float) -7.902479E37F, PH.base.pack) ;
        p174_EAS2TAS_SET((float) -2.6361409E38F, PH.base.pack) ;
        p174_vz_SET((float)1.5437915E36F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_AIRSPEED_AUTOCAL_174(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RALLY_POINT_175(), &PH);
        p175_target_component_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
        p175_alt_SET((int16_t)(int16_t) -5455, PH.base.pack) ;
        p175_idx_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p175_count_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p175_target_system_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        p175_lat_SET((int32_t)1520808089, PH.base.pack) ;
        p175_flags_SET(e_RALLY_FLAGS_LAND_IMMEDIATELY, PH.base.pack) ;
        p175_land_dir_SET((uint16_t)(uint16_t)49355, PH.base.pack) ;
        p175_lng_SET((int32_t) -1840716910, PH.base.pack) ;
        p175_break_alt_SET((int16_t)(int16_t)32264, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RALLY_POINT_175(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RALLY_FETCH_POINT_176(), &PH);
        p176_idx_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        p176_target_system_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        p176_target_component_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RALLY_FETCH_POINT_176(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMPASSMOT_STATUS_177(), &PH);
        p177_throttle_SET((uint16_t)(uint16_t)21490, PH.base.pack) ;
        p177_CompensationZ_SET((float)2.6876929E38F, PH.base.pack) ;
        p177_current_SET((float) -2.1091678E38F, PH.base.pack) ;
        p177_CompensationY_SET((float) -2.132742E37F, PH.base.pack) ;
        p177_CompensationX_SET((float) -4.8398406E36F, PH.base.pack) ;
        p177_interference_SET((uint16_t)(uint16_t)49008, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMPASSMOT_STATUS_177(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AHRS2_178(), &PH);
        p178_lng_SET((int32_t) -71042179, PH.base.pack) ;
        p178_lat_SET((int32_t)1953768174, PH.base.pack) ;
        p178_altitude_SET((float) -1.8755982E38F, PH.base.pack) ;
        p178_pitch_SET((float)6.0537035E37F, PH.base.pack) ;
        p178_yaw_SET((float) -3.2196327E38F, PH.base.pack) ;
        p178_roll_SET((float) -2.18424E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_AHRS2_178(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_STATUS_179(), &PH);
        p179_time_usec_SET((uint64_t)4706417044326555564L, PH.base.pack) ;
        p179_p2_SET((float)1.7823453E38F, PH.base.pack) ;
        p179_cam_idx_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        p179_p4_SET((float)2.8667513E38F, PH.base.pack) ;
        p179_img_idx_SET((uint16_t)(uint16_t)32291, PH.base.pack) ;
        p179_target_system_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
        p179_event_id_SET(e_CAMERA_STATUS_TYPES_CAMERA_STATUS_TYPE_TRIGGER, PH.base.pack) ;
        p179_p1_SET((float) -8.335899E37F, PH.base.pack) ;
        p179_p3_SET((float) -3.0889179E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_STATUS_179(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_FEEDBACK_180(), &PH);
        p180_yaw_SET((float) -1.1044116E38F, PH.base.pack) ;
        p180_lng_SET((int32_t)214634616, PH.base.pack) ;
        p180_flags_SET(e_CAMERA_FEEDBACK_FLAGS_CAMERA_FEEDBACK_CLOSEDLOOP, PH.base.pack) ;
        p180_alt_rel_SET((float)6.511096E37F, PH.base.pack) ;
        p180_time_usec_SET((uint64_t)2117520856037814619L, PH.base.pack) ;
        p180_foc_len_SET((float)3.3419738E38F, PH.base.pack) ;
        p180_alt_msl_SET((float)8.822345E37F, PH.base.pack) ;
        p180_cam_idx_SET((uint8_t)(uint8_t)2, PH.base.pack) ;
        p180_img_idx_SET((uint16_t)(uint16_t)24557, PH.base.pack) ;
        p180_roll_SET((float) -2.9591117E38F, PH.base.pack) ;
        p180_lat_SET((int32_t) -2045931032, PH.base.pack) ;
        p180_pitch_SET((float) -2.7764868E38F, PH.base.pack) ;
        p180_target_system_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_FEEDBACK_180(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BATTERY2_181(), &PH);
        p181_current_battery_SET((int16_t)(int16_t) -3636, PH.base.pack) ;
        p181_voltage_SET((uint16_t)(uint16_t)54847, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BATTERY2_181(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AHRS3_182(), &PH);
        p182_roll_SET((float)3.1158531E38F, PH.base.pack) ;
        p182_v4_SET((float) -1.6173555E38F, PH.base.pack) ;
        p182_v2_SET((float)1.0581919E38F, PH.base.pack) ;
        p182_altitude_SET((float)2.8746115E38F, PH.base.pack) ;
        p182_lat_SET((int32_t)1103231578, PH.base.pack) ;
        p182_v1_SET((float) -7.7223277E37F, PH.base.pack) ;
        p182_v3_SET((float) -3.113939E38F, PH.base.pack) ;
        p182_pitch_SET((float)3.8468594E37F, PH.base.pack) ;
        p182_yaw_SET((float) -9.890003E37F, PH.base.pack) ;
        p182_lng_SET((int32_t) -878298418, PH.base.pack) ;
        c_LoopBackDemoChannel_on_AHRS3_182(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_REQUEST_183(), &PH);
        p183_target_system_SET((uint8_t)(uint8_t)5, PH.base.pack) ;
        p183_target_component_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_REQUEST_183(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_REMOTE_LOG_DATA_BLOCK_184(), &PH);
        p184_seqno_SET(e_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_MAV_REMOTE_LOG_DATA_BLOCK_STOP, PH.base.pack) ;
        p184_target_component_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)57, (uint8_t)250, (uint8_t)12, (uint8_t)149, (uint8_t)9, (uint8_t)252, (uint8_t)179, (uint8_t)99, (uint8_t)190, (uint8_t)192, (uint8_t)232, (uint8_t)154, (uint8_t)212, (uint8_t)156, (uint8_t)140, (uint8_t)242, (uint8_t)94, (uint8_t)128, (uint8_t)124, (uint8_t)28, (uint8_t)63, (uint8_t)30, (uint8_t)86, (uint8_t)115, (uint8_t)250, (uint8_t)77, (uint8_t)165, (uint8_t)127, (uint8_t)67, (uint8_t)97, (uint8_t)178, (uint8_t)53, (uint8_t)215, (uint8_t)209, (uint8_t)168, (uint8_t)217, (uint8_t)56, (uint8_t)183, (uint8_t)229, (uint8_t)12, (uint8_t)197, (uint8_t)130, (uint8_t)137, (uint8_t)208, (uint8_t)85, (uint8_t)77, (uint8_t)248, (uint8_t)43, (uint8_t)110, (uint8_t)67, (uint8_t)249, (uint8_t)135, (uint8_t)93, (uint8_t)72, (uint8_t)100, (uint8_t)9, (uint8_t)230, (uint8_t)234, (uint8_t)102, (uint8_t)117, (uint8_t)52, (uint8_t)147, (uint8_t)206, (uint8_t)73, (uint8_t)51, (uint8_t)90, (uint8_t)61, (uint8_t)50, (uint8_t)114, (uint8_t)147, (uint8_t)213, (uint8_t)46, (uint8_t)230, (uint8_t)237, (uint8_t)27, (uint8_t)151, (uint8_t)8, (uint8_t)101, (uint8_t)148, (uint8_t)42, (uint8_t)221, (uint8_t)134, (uint8_t)135, (uint8_t)175, (uint8_t)12, (uint8_t)248, (uint8_t)140, (uint8_t)177, (uint8_t)66, (uint8_t)15, (uint8_t)80, (uint8_t)108, (uint8_t)33, (uint8_t)168, (uint8_t)84, (uint8_t)220, (uint8_t)228, (uint8_t)36, (uint8_t)23, (uint8_t)206, (uint8_t)204, (uint8_t)255, (uint8_t)94, (uint8_t)104, (uint8_t)59, (uint8_t)78, (uint8_t)245, (uint8_t)117, (uint8_t)121, (uint8_t)168, (uint8_t)138, (uint8_t)14, (uint8_t)186, (uint8_t)137, (uint8_t)76, (uint8_t)52, (uint8_t)157, (uint8_t)117, (uint8_t)144, (uint8_t)41, (uint8_t)111, (uint8_t)162, (uint8_t)25, (uint8_t)94, (uint8_t)246, (uint8_t)159, (uint8_t)153, (uint8_t)4, (uint8_t)239, (uint8_t)234, (uint8_t)205, (uint8_t)73, (uint8_t)237, (uint8_t)253, (uint8_t)105, (uint8_t)210, (uint8_t)229, (uint8_t)243, (uint8_t)129, (uint8_t)233, (uint8_t)114, (uint8_t)42, (uint8_t)26, (uint8_t)193, (uint8_t)247, (uint8_t)144, (uint8_t)225, (uint8_t)203, (uint8_t)162, (uint8_t)122, (uint8_t)59, (uint8_t)195, (uint8_t)215, (uint8_t)65, (uint8_t)58, (uint8_t)242, (uint8_t)122, (uint8_t)100, (uint8_t)117, (uint8_t)247, (uint8_t)127, (uint8_t)151, (uint8_t)124, (uint8_t)129, (uint8_t)131, (uint8_t)191, (uint8_t)166, (uint8_t)42, (uint8_t)150, (uint8_t)167, (uint8_t)247, (uint8_t)135, (uint8_t)107, (uint8_t)179, (uint8_t)119, (uint8_t)247, (uint8_t)161, (uint8_t)57, (uint8_t)61, (uint8_t)102, (uint8_t)115, (uint8_t)3, (uint8_t)236, (uint8_t)63, (uint8_t)110, (uint8_t)2, (uint8_t)19, (uint8_t)39, (uint8_t)255, (uint8_t)22, (uint8_t)77, (uint8_t)20, (uint8_t)159, (uint8_t)222, (uint8_t)93, (uint8_t)13, (uint8_t)183, (uint8_t)110, (uint8_t)119, (uint8_t)151};
            p184_data__SET(&data_, 0, PH.base.pack) ;
        }
        p184_target_system_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        c_LoopBackDemoChannel_on_REMOTE_LOG_DATA_BLOCK_184(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_REMOTE_LOG_BLOCK_STATUS_185(), &PH);
        p185_seqno_SET((uint32_t)4173146683L, PH.base.pack) ;
        p185_status_SET(e_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_MAV_REMOTE_LOG_DATA_BLOCK_ACK, PH.base.pack) ;
        p185_target_component_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
        p185_target_system_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        c_LoopBackDemoChannel_on_REMOTE_LOG_BLOCK_STATUS_185(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LED_CONTROL_186(), &PH);
        p186_custom_len_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        {
            uint8_t custom_bytes[] =  {(uint8_t)68, (uint8_t)130, (uint8_t)59, (uint8_t)92, (uint8_t)140, (uint8_t)36, (uint8_t)58, (uint8_t)111, (uint8_t)53, (uint8_t)74, (uint8_t)66, (uint8_t)215, (uint8_t)160, (uint8_t)29, (uint8_t)153, (uint8_t)127, (uint8_t)210, (uint8_t)236, (uint8_t)168, (uint8_t)23, (uint8_t)50, (uint8_t)60, (uint8_t)15, (uint8_t)99};
            p186_custom_bytes_SET(&custom_bytes, 0, PH.base.pack) ;
        }
        p186_pattern_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        p186_target_system_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        p186_target_component_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p186_instance_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LED_CONTROL_186(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MAG_CAL_PROGRESS_191(), &PH);
        p191_cal_status_SET(e_MAG_CAL_STATUS_MAG_CAL_RUNNING_STEP_TWO, PH.base.pack) ;
        p191_direction_z_SET((float) -2.3552013E37F, PH.base.pack) ;
        p191_cal_mask_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
        p191_direction_x_SET((float) -2.282132E38F, PH.base.pack) ;
        p191_direction_y_SET((float) -2.7257313E38F, PH.base.pack) ;
        p191_attempt_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        p191_compass_id_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        p191_completion_pct_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        {
            uint8_t completion_mask[] =  {(uint8_t)174, (uint8_t)89, (uint8_t)154, (uint8_t)35, (uint8_t)158, (uint8_t)203, (uint8_t)177, (uint8_t)155, (uint8_t)114, (uint8_t)57};
            p191_completion_mask_SET(&completion_mask, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_MAG_CAL_PROGRESS_191(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MAG_CAL_REPORT_192(), &PH);
        p192_cal_mask_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p192_offdiag_x_SET((float) -3.1873028E38F, PH.base.pack) ;
        p192_ofs_x_SET((float)1.405528E37F, PH.base.pack) ;
        p192_ofs_z_SET((float) -6.4722746E37F, PH.base.pack) ;
        p192_diag_z_SET((float)8.785531E37F, PH.base.pack) ;
        p192_offdiag_y_SET((float) -1.710795E38F, PH.base.pack) ;
        p192_diag_x_SET((float) -5.221766E37F, PH.base.pack) ;
        p192_compass_id_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p192_fitness_SET((float) -2.9418743E38F, PH.base.pack) ;
        p192_autosaved_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        p192_diag_y_SET((float)1.1972406E38F, PH.base.pack) ;
        p192_ofs_y_SET((float)5.6115454E37F, PH.base.pack) ;
        p192_offdiag_z_SET((float)2.8926476E38F, PH.base.pack) ;
        p192_cal_status_SET(e_MAG_CAL_STATUS_MAG_CAL_RUNNING_STEP_ONE, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MAG_CAL_REPORT_192(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_EKF_STATUS_REPORT_193(), &PH);
        p193_pos_vert_variance_SET((float) -3.2420274E38F, PH.base.pack) ;
        p193_compass_variance_SET((float) -2.8166561E38F, PH.base.pack) ;
        p193_terrain_alt_variance_SET((float) -1.5273746E38F, PH.base.pack) ;
        p193_velocity_variance_SET((float)1.7448637E36F, PH.base.pack) ;
        p193_flags_SET(e_EKF_STATUS_FLAGS_EKF_PRED_POS_HORIZ_REL, PH.base.pack) ;
        p193_pos_horiz_variance_SET((float) -2.6690665E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_EKF_STATUS_REPORT_193(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PID_TUNING_194(), &PH);
        p194_FF_SET((float)2.3683226E38F, PH.base.pack) ;
        p194_achieved_SET((float) -2.2209305E38F, PH.base.pack) ;
        p194_P_SET((float) -5.021555E37F, PH.base.pack) ;
        p194_desired_SET((float)2.656921E38F, PH.base.pack) ;
        p194_I_SET((float)4.897631E37F, PH.base.pack) ;
        p194_D_SET((float) -2.5154638E38F, PH.base.pack) ;
        p194_axis_SET(e_PID_TUNING_AXIS_PID_TUNING_ACCZ, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PID_TUNING_194(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GIMBAL_REPORT_200(), &PH);
        p200_target_component_SET((uint8_t)(uint8_t)83, PH.base.pack) ;
        p200_delta_velocity_y_SET((float) -6.456526E37F, PH.base.pack) ;
        p200_joint_roll_SET((float) -3.3034052E38F, PH.base.pack) ;
        p200_delta_angle_y_SET((float) -1.2333405E38F, PH.base.pack) ;
        p200_delta_angle_x_SET((float)1.7592719E38F, PH.base.pack) ;
        p200_delta_velocity_x_SET((float) -2.328791E38F, PH.base.pack) ;
        p200_delta_velocity_z_SET((float)2.0462215E38F, PH.base.pack) ;
        p200_delta_time_SET((float)2.1533662E38F, PH.base.pack) ;
        p200_joint_el_SET((float) -3.0479184E38F, PH.base.pack) ;
        p200_target_system_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p200_joint_az_SET((float) -1.767394E38F, PH.base.pack) ;
        p200_delta_angle_z_SET((float) -2.4934578E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GIMBAL_REPORT_200(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GIMBAL_CONTROL_201(), &PH);
        p201_demanded_rate_y_SET((float)3.197891E38F, PH.base.pack) ;
        p201_demanded_rate_z_SET((float) -5.561861E37F, PH.base.pack) ;
        p201_target_component_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
        p201_target_system_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p201_demanded_rate_x_SET((float)2.6927653E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GIMBAL_CONTROL_201(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GIMBAL_TORQUE_CMD_REPORT_214(), &PH);
        p214_az_torque_cmd_SET((int16_t)(int16_t) -8600, PH.base.pack) ;
        p214_el_torque_cmd_SET((int16_t)(int16_t) -291, PH.base.pack) ;
        p214_rl_torque_cmd_SET((int16_t)(int16_t)16847, PH.base.pack) ;
        p214_target_system_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        p214_target_component_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GIMBAL_TORQUE_CMD_REPORT_214(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GOPRO_HEARTBEAT_215(), &PH);
        p215_capture_mode_SET(e_GOPRO_CAPTURE_MODE_GOPRO_CAPTURE_MODE_VIDEO, PH.base.pack) ;
        p215_flags_SET(e_GOPRO_HEARTBEAT_FLAGS_GOPRO_FLAG_RECORDING, PH.base.pack) ;
        p215_status_SET(e_GOPRO_HEARTBEAT_STATUS_GOPRO_HEARTBEAT_STATUS_CONNECTED, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GOPRO_HEARTBEAT_215(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GOPRO_GET_REQUEST_216(), &PH);
        p216_target_system_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        p216_target_component_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p216_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_MODEL, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GOPRO_GET_REQUEST_216(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GOPRO_GET_RESPONSE_217(), &PH);
        {
            uint8_t value[] =  {(uint8_t)213, (uint8_t)95, (uint8_t)244, (uint8_t)204};
            p217_value_SET(&value, 0, PH.base.pack) ;
        }
        p217_status_SET(e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS, PH.base.pack) ;
        p217_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_PROTUNE_EXPOSURE, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GOPRO_GET_RESPONSE_217(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GOPRO_SET_REQUEST_218(), &PH);
        {
            uint8_t value[] =  {(uint8_t)138, (uint8_t)128, (uint8_t)8, (uint8_t)34};
            p218_value_SET(&value, 0, PH.base.pack) ;
        }
        p218_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_LOW_LIGHT, PH.base.pack) ;
        p218_target_component_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        p218_target_system_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GOPRO_SET_REQUEST_218(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GOPRO_SET_RESPONSE_219(), &PH);
        p219_cmd_id_SET(e_GOPRO_COMMAND_GOPRO_COMMAND_CHARGING, PH.base.pack) ;
        p219_status_SET(e_GOPRO_REQUEST_STATUS_GOPRO_REQUEST_SUCCESS, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GOPRO_SET_RESPONSE_219(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RPM_226(), &PH);
        p226_rpm1_SET((float)2.4958253E37F, PH.base.pack) ;
        p226_rpm2_SET((float)5.787931E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RPM_226(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_pos_horiz_accuracy_SET((float) -2.6168369E38F, PH.base.pack) ;
        p230_hagl_ratio_SET((float) -8.286331E37F, PH.base.pack) ;
        p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_ATTITUDE, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float) -2.4435578E38F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float) -3.225713E38F, PH.base.pack) ;
        p230_mag_ratio_SET((float)3.078284E38F, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float)3.8424503E37F, PH.base.pack) ;
        p230_vel_ratio_SET((float)1.1329453E38F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)6566160794537745771L, PH.base.pack) ;
        p230_tas_ratio_SET((float) -3.5839938E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIND_COV_231(), &PH);
        p231_vert_accuracy_SET((float) -1.3564092E38F, PH.base.pack) ;
        p231_var_horiz_SET((float) -1.2457691E38F, PH.base.pack) ;
        p231_wind_x_SET((float) -1.1693875E38F, PH.base.pack) ;
        p231_wind_z_SET((float) -4.8913343E37F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)1632632645745052918L, PH.base.pack) ;
        p231_var_vert_SET((float)6.7309737E37F, PH.base.pack) ;
        p231_wind_y_SET((float)1.9179276E38F, PH.base.pack) ;
        p231_wind_alt_SET((float) -1.5742209E38F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float)2.1439427E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INPUT_232(), &PH);
        p232_vn_SET((float)4.7901033E37F, PH.base.pack) ;
        p232_lon_SET((int32_t)573024533, PH.base.pack) ;
        p232_alt_SET((float) -3.3129977E38F, PH.base.pack) ;
        p232_vd_SET((float) -2.2507297E38F, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)234275877406942109L, PH.base.pack) ;
        p232_vdop_SET((float)1.2685448E38F, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)59543, PH.base.pack) ;
        p232_ve_SET((float) -1.5670992E38F, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)659730492L, PH.base.pack) ;
        p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HDOP, PH.base.pack) ;
        p232_lat_SET((int32_t)317830732, PH.base.pack) ;
        p232_horiz_accuracy_SET((float)1.6460375E38F, PH.base.pack) ;
        p232_speed_accuracy_SET((float)2.7299513E38F, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p232_hdop_SET((float) -1.879019E38F, PH.base.pack) ;
        p232_vert_accuracy_SET((float)6.7156067E37F, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTCM_DATA_233(), &PH);
        p233_len_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)188, (uint8_t)144, (uint8_t)145, (uint8_t)79, (uint8_t)223, (uint8_t)216, (uint8_t)248, (uint8_t)16, (uint8_t)209, (uint8_t)228, (uint8_t)133, (uint8_t)125, (uint8_t)63, (uint8_t)231, (uint8_t)47, (uint8_t)244, (uint8_t)21, (uint8_t)242, (uint8_t)155, (uint8_t)147, (uint8_t)225, (uint8_t)209, (uint8_t)205, (uint8_t)2, (uint8_t)27, (uint8_t)50, (uint8_t)160, (uint8_t)94, (uint8_t)114, (uint8_t)197, (uint8_t)89, (uint8_t)164, (uint8_t)240, (uint8_t)51, (uint8_t)30, (uint8_t)50, (uint8_t)133, (uint8_t)82, (uint8_t)144, (uint8_t)115, (uint8_t)191, (uint8_t)198, (uint8_t)189, (uint8_t)219, (uint8_t)30, (uint8_t)230, (uint8_t)13, (uint8_t)54, (uint8_t)189, (uint8_t)227, (uint8_t)231, (uint8_t)197, (uint8_t)146, (uint8_t)167, (uint8_t)182, (uint8_t)132, (uint8_t)23, (uint8_t)114, (uint8_t)125, (uint8_t)41, (uint8_t)38, (uint8_t)132, (uint8_t)180, (uint8_t)187, (uint8_t)129, (uint8_t)37, (uint8_t)134, (uint8_t)198, (uint8_t)118, (uint8_t)134, (uint8_t)16, (uint8_t)247, (uint8_t)77, (uint8_t)121, (uint8_t)72, (uint8_t)174, (uint8_t)107, (uint8_t)69, (uint8_t)114, (uint8_t)200, (uint8_t)193, (uint8_t)123, (uint8_t)215, (uint8_t)43, (uint8_t)19, (uint8_t)227, (uint8_t)21, (uint8_t)147, (uint8_t)70, (uint8_t)6, (uint8_t)211, (uint8_t)196, (uint8_t)34, (uint8_t)114, (uint8_t)47, (uint8_t)159, (uint8_t)255, (uint8_t)104, (uint8_t)197, (uint8_t)254, (uint8_t)106, (uint8_t)190, (uint8_t)216, (uint8_t)232, (uint8_t)139, (uint8_t)19, (uint8_t)202, (uint8_t)94, (uint8_t)201, (uint8_t)125, (uint8_t)246, (uint8_t)177, (uint8_t)59, (uint8_t)87, (uint8_t)187, (uint8_t)77, (uint8_t)135, (uint8_t)70, (uint8_t)7, (uint8_t)41, (uint8_t)228, (uint8_t)250, (uint8_t)154, (uint8_t)43, (uint8_t)40, (uint8_t)223, (uint8_t)249, (uint8_t)77, (uint8_t)21, (uint8_t)241, (uint8_t)222, (uint8_t)189, (uint8_t)99, (uint8_t)174, (uint8_t)197, (uint8_t)194, (uint8_t)14, (uint8_t)249, (uint8_t)226, (uint8_t)20, (uint8_t)31, (uint8_t)250, (uint8_t)183, (uint8_t)43, (uint8_t)31, (uint8_t)96, (uint8_t)0, (uint8_t)132, (uint8_t)125, (uint8_t)26, (uint8_t)61, (uint8_t)231, (uint8_t)223, (uint8_t)169, (uint8_t)58, (uint8_t)216, (uint8_t)117, (uint8_t)71, (uint8_t)202, (uint8_t)206, (uint8_t)182, (uint8_t)17, (uint8_t)99, (uint8_t)7, (uint8_t)27, (uint8_t)126, (uint8_t)4, (uint8_t)4, (uint8_t)3, (uint8_t)205, (uint8_t)45, (uint8_t)153, (uint8_t)195, (uint8_t)227, (uint8_t)97, (uint8_t)175, (uint8_t)211, (uint8_t)116, (uint8_t)182, (uint8_t)137};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        p233_flags_SET((uint8_t)(uint8_t)204, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGH_LATENCY_234(), &PH);
        p234_latitude_SET((int32_t) -1070663787, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_LANDING, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)23393, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t)22934, PH.base.pack) ;
        p234_longitude_SET((int32_t) -1678887264, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)844053704L, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)58, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_NO_GPS, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)76, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t) -31266, PH.base.pack) ;
        p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_TEST_ENABLED, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t)12708, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t) -14348, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t) -59, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)21577, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t) -58, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t) -61, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t) -119, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t)25829, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIBRATION_241(), &PH);
        p241_vibration_y_SET((float) -2.4237972E38F, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)3070810146L, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)3790782853L, PH.base.pack) ;
        p241_time_usec_SET((uint64_t)6337791997861692261L, PH.base.pack) ;
        p241_vibration_z_SET((float)3.3642585E38F, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)1016481609L, PH.base.pack) ;
        p241_vibration_x_SET((float) -7.302798E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HOME_POSITION_242(), &PH);
        p242_approach_z_SET((float)7.4888574E37F, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)8564977927090414023L, &PH) ;
        p242_z_SET((float)1.9928486E38F, PH.base.pack) ;
        p242_latitude_SET((int32_t) -410021199, PH.base.pack) ;
        p242_x_SET((float)2.740982E38F, PH.base.pack) ;
        p242_approach_y_SET((float) -2.8006787E38F, PH.base.pack) ;
        p242_altitude_SET((int32_t)1464979421, PH.base.pack) ;
        p242_approach_x_SET((float) -1.02680706E37F, PH.base.pack) ;
        p242_y_SET((float)2.457572E37F, PH.base.pack) ;
        {
            float q[] =  {1.1752303E38F, -1.8741703E38F, 4.426819E37F, 8.521899E37F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_longitude_SET((int32_t)992323133, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_HOME_POSITION_243(), &PH);
        p243_y_SET((float) -2.3047253E38F, PH.base.pack) ;
        {
            float q[] =  {2.0803254E38F, -5.998711E36F, 1.0976029E38F, 1.1206643E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_approach_z_SET((float) -7.8850413E37F, PH.base.pack) ;
        p243_z_SET((float)1.62176E38F, PH.base.pack) ;
        p243_altitude_SET((int32_t) -711633390, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        p243_x_SET((float) -3.1241395E38F, PH.base.pack) ;
        p243_longitude_SET((int32_t) -1381449306, PH.base.pack) ;
        p243_approach_x_SET((float)7.9038583E37F, PH.base.pack) ;
        p243_approach_y_SET((float)2.9522305E38F, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)8503561173903668787L, &PH) ;
        p243_latitude_SET((int32_t)1684008788, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_interval_us_SET((int32_t) -1204877428, PH.base.pack) ;
        p244_message_id_SET((uint16_t)(uint16_t)4984, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_MC, PH.base.pack) ;
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_ON_GROUND, PH.base.pack) ;
        c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ADSB_VEHICLE_246(), &PH);
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
        p246_lat_SET((int32_t) -330783888, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)27852, PH.base.pack) ;
        p246_altitude_SET((int32_t)1643306639, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)62652, PH.base.pack) ;
        {
            char16_t* callsign = u"ooistezy";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_lon_SET((int32_t)1553951816, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_SERVICE_SURFACE, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)30793, PH.base.pack) ;
        p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_SQUAWK, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t)12898, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)1721914629L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COLLISION_247(), &PH);
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_LOW, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float) -2.9769393E37F, PH.base.pack) ;
        p247_id_SET((uint32_t)4079285685L, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float)1.3347511E38F, PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float) -1.8626663E38F, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_NONE, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_V2_EXTENSION_248(), &PH);
        p248_target_component_SET((uint8_t)(uint8_t)40, PH.base.pack) ;
        p248_target_network_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        p248_message_type_SET((uint16_t)(uint16_t)57228, PH.base.pack) ;
        p248_target_system_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)160, (uint8_t)175, (uint8_t)168, (uint8_t)0, (uint8_t)7, (uint8_t)57, (uint8_t)148, (uint8_t)11, (uint8_t)134, (uint8_t)18, (uint8_t)6, (uint8_t)226, (uint8_t)96, (uint8_t)72, (uint8_t)125, (uint8_t)156, (uint8_t)39, (uint8_t)73, (uint8_t)8, (uint8_t)142, (uint8_t)144, (uint8_t)245, (uint8_t)31, (uint8_t)34, (uint8_t)71, (uint8_t)222, (uint8_t)180, (uint8_t)118, (uint8_t)2, (uint8_t)233, (uint8_t)113, (uint8_t)69, (uint8_t)37, (uint8_t)99, (uint8_t)91, (uint8_t)86, (uint8_t)232, (uint8_t)232, (uint8_t)165, (uint8_t)23, (uint8_t)14, (uint8_t)247, (uint8_t)241, (uint8_t)81, (uint8_t)196, (uint8_t)87, (uint8_t)188, (uint8_t)9, (uint8_t)150, (uint8_t)112, (uint8_t)230, (uint8_t)106, (uint8_t)225, (uint8_t)154, (uint8_t)50, (uint8_t)252, (uint8_t)30, (uint8_t)217, (uint8_t)31, (uint8_t)174, (uint8_t)156, (uint8_t)51, (uint8_t)179, (uint8_t)126, (uint8_t)6, (uint8_t)34, (uint8_t)201, (uint8_t)169, (uint8_t)91, (uint8_t)143, (uint8_t)253, (uint8_t)97, (uint8_t)81, (uint8_t)154, (uint8_t)75, (uint8_t)250, (uint8_t)6, (uint8_t)198, (uint8_t)70, (uint8_t)125, (uint8_t)253, (uint8_t)180, (uint8_t)176, (uint8_t)2, (uint8_t)196, (uint8_t)86, (uint8_t)22, (uint8_t)250, (uint8_t)211, (uint8_t)152, (uint8_t)227, (uint8_t)169, (uint8_t)163, (uint8_t)139, (uint8_t)47, (uint8_t)165, (uint8_t)171, (uint8_t)113, (uint8_t)100, (uint8_t)117, (uint8_t)151, (uint8_t)231, (uint8_t)35, (uint8_t)10, (uint8_t)128, (uint8_t)19, (uint8_t)38, (uint8_t)49, (uint8_t)182, (uint8_t)210, (uint8_t)174, (uint8_t)72, (uint8_t)178, (uint8_t)91, (uint8_t)212, (uint8_t)38, (uint8_t)202, (uint8_t)144, (uint8_t)150, (uint8_t)204, (uint8_t)88, (uint8_t)206, (uint8_t)141, (uint8_t)205, (uint8_t)24, (uint8_t)37, (uint8_t)214, (uint8_t)194, (uint8_t)28, (uint8_t)177, (uint8_t)180, (uint8_t)14, (uint8_t)135, (uint8_t)42, (uint8_t)129, (uint8_t)111, (uint8_t)232, (uint8_t)204, (uint8_t)49, (uint8_t)147, (uint8_t)148, (uint8_t)37, (uint8_t)5, (uint8_t)206, (uint8_t)199, (uint8_t)216, (uint8_t)162, (uint8_t)55, (uint8_t)242, (uint8_t)240, (uint8_t)139, (uint8_t)123, (uint8_t)164, (uint8_t)207, (uint8_t)178, (uint8_t)226, (uint8_t)158, (uint8_t)149, (uint8_t)227, (uint8_t)83, (uint8_t)15, (uint8_t)124, (uint8_t)6, (uint8_t)125, (uint8_t)60, (uint8_t)177, (uint8_t)157, (uint8_t)104, (uint8_t)125, (uint8_t)222, (uint8_t)243, (uint8_t)23, (uint8_t)169, (uint8_t)136, (uint8_t)234, (uint8_t)212, (uint8_t)68, (uint8_t)71, (uint8_t)8, (uint8_t)83, (uint8_t)160, (uint8_t)236, (uint8_t)96, (uint8_t)182, (uint8_t)178, (uint8_t)240, (uint8_t)159, (uint8_t)86, (uint8_t)240, (uint8_t)33, (uint8_t)205, (uint8_t)107, (uint8_t)214, (uint8_t)229, (uint8_t)105, (uint8_t)34, (uint8_t)140, (uint8_t)12, (uint8_t)23, (uint8_t)245, (uint8_t)104, (uint8_t)247, (uint8_t)156, (uint8_t)159, (uint8_t)225, (uint8_t)220, (uint8_t)59, (uint8_t)192, (uint8_t)195, (uint8_t)152, (uint8_t)246, (uint8_t)90, (uint8_t)34, (uint8_t)64, (uint8_t)254, (uint8_t)18, (uint8_t)130, (uint8_t)73, (uint8_t)37, (uint8_t)249, (uint8_t)80, (uint8_t)54, (uint8_t)96, (uint8_t)88, (uint8_t)41, (uint8_t)193, (uint8_t)249, (uint8_t)173, (uint8_t)11, (uint8_t)252, (uint8_t)205, (uint8_t)157, (uint8_t)247, (uint8_t)111, (uint8_t)202, (uint8_t)152, (uint8_t)251, (uint8_t)3, (uint8_t)62, (uint8_t)45, (uint8_t)142, (uint8_t)31, (uint8_t)159, (uint8_t)58, (uint8_t)229, (uint8_t)255, (uint8_t)127, (uint8_t)30, (uint8_t)21};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MEMORY_VECT_249(), &PH);
        p249_ver_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        {
            int8_t value[] =  {(int8_t) -61, (int8_t)122, (int8_t)26, (int8_t)112, (int8_t)20, (int8_t) -96, (int8_t)55, (int8_t) -87, (int8_t)10, (int8_t)76, (int8_t) -122, (int8_t)120, (int8_t) -55, (int8_t)105, (int8_t) -33, (int8_t) -35, (int8_t)6, (int8_t)71, (int8_t)72, (int8_t) -34, (int8_t) -55, (int8_t) -18, (int8_t)111, (int8_t)109, (int8_t)125, (int8_t) -44, (int8_t)13, (int8_t)30, (int8_t)97, (int8_t) -45, (int8_t)34, (int8_t)84};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_type_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        p249_address_SET((uint16_t)(uint16_t)11199, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEBUG_VECT_250(), &PH);
        p250_y_SET((float) -1.5848465E38F, PH.base.pack) ;
        {
            char16_t* name = u"ngllc";
            p250_name_SET_(name, &PH) ;
        }
        p250_z_SET((float)2.5154092E38F, PH.base.pack) ;
        p250_time_usec_SET((uint64_t)3075084406157923064L, PH.base.pack) ;
        p250_x_SET((float)2.6151695E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
        {
            char16_t* name = u"ju";
            p251_name_SET_(name, &PH) ;
        }
        p251_value_SET((float) -1.2046135E38F, PH.base.pack) ;
        p251_time_boot_ms_SET((uint32_t)3703266438L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_INT_252(), &PH);
        p252_time_boot_ms_SET((uint32_t)1812613599L, PH.base.pack) ;
        p252_value_SET((int32_t) -590492140, PH.base.pack) ;
        {
            char16_t* name = u"edmjWpsrlp";
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
            char16_t* text = u"rxpet";
            p253_text_SET_(text, &PH) ;
        }
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_DEBUG, PH.base.pack) ;
        c_LoopBackDemoChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEBUG_254(), &PH);
        p254_ind_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p254_value_SET((float) -1.825317E38F, PH.base.pack) ;
        p254_time_boot_ms_SET((uint32_t)1676449526L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SETUP_SIGNING_256(), &PH);
        p256_initial_timestamp_SET((uint64_t)1237436282984357039L, PH.base.pack) ;
        p256_target_component_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p256_target_system_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)216, (uint8_t)29, (uint8_t)186, (uint8_t)12, (uint8_t)13, (uint8_t)18, (uint8_t)122, (uint8_t)114, (uint8_t)36, (uint8_t)111, (uint8_t)230, (uint8_t)210, (uint8_t)184, (uint8_t)127, (uint8_t)124, (uint8_t)60, (uint8_t)231, (uint8_t)172, (uint8_t)72, (uint8_t)30, (uint8_t)16, (uint8_t)97, (uint8_t)86, (uint8_t)226, (uint8_t)96, (uint8_t)183, (uint8_t)248, (uint8_t)243, (uint8_t)60, (uint8_t)88, (uint8_t)141, (uint8_t)168};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BUTTON_CHANGE_257(), &PH);
        p257_time_boot_ms_SET((uint32_t)1985759935L, PH.base.pack) ;
        p257_last_change_ms_SET((uint32_t)3594042759L, PH.base.pack) ;
        p257_state_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PLAY_TUNE_258(), &PH);
        p258_target_component_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        p258_target_system_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
        {
            char16_t* tune = u"bXLhKeFdcdtfuMcnfx";
            p258_tune_SET_(tune, &PH) ;
        }
        c_LoopBackDemoChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_INFORMATION_259(), &PH);
        {
            uint8_t model_name[] =  {(uint8_t)145, (uint8_t)78, (uint8_t)131, (uint8_t)211, (uint8_t)35, (uint8_t)255, (uint8_t)121, (uint8_t)63, (uint8_t)9, (uint8_t)10, (uint8_t)28, (uint8_t)181, (uint8_t)87, (uint8_t)42, (uint8_t)21, (uint8_t)171, (uint8_t)248, (uint8_t)128, (uint8_t)131, (uint8_t)37, (uint8_t)82, (uint8_t)34, (uint8_t)99, (uint8_t)49, (uint8_t)101, (uint8_t)231, (uint8_t)174, (uint8_t)242, (uint8_t)34, (uint8_t)24, (uint8_t)249, (uint8_t)240};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_firmware_version_SET((uint32_t)1331807499L, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)115, (uint8_t)232, (uint8_t)241, (uint8_t)188, (uint8_t)152, (uint8_t)224, (uint8_t)210, (uint8_t)116, (uint8_t)78, (uint8_t)190, (uint8_t)32, (uint8_t)209, (uint8_t)254, (uint8_t)54, (uint8_t)162, (uint8_t)150, (uint8_t)252, (uint8_t)100, (uint8_t)4, (uint8_t)76, (uint8_t)199, (uint8_t)153, (uint8_t)222, (uint8_t)209, (uint8_t)159, (uint8_t)89, (uint8_t)111, (uint8_t)95, (uint8_t)70, (uint8_t)41, (uint8_t)254, (uint8_t)101};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE, PH.base.pack) ;
        p259_focal_length_SET((float) -1.6390109E38F, PH.base.pack) ;
        p259_time_boot_ms_SET((uint32_t)2941703693L, PH.base.pack) ;
        p259_resolution_v_SET((uint16_t)(uint16_t)37100, PH.base.pack) ;
        p259_lens_id_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        p259_sensor_size_v_SET((float)1.2099008E36F, PH.base.pack) ;
        p259_resolution_h_SET((uint16_t)(uint16_t)57256, PH.base.pack) ;
        p259_cam_definition_version_SET((uint16_t)(uint16_t)890, PH.base.pack) ;
        p259_sensor_size_h_SET((float) -1.4896115E38F, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"yrqTerxepdsnluuynddeergjAcutsylbbctgbsspayLogkeauWymzbsCppviVYykdytqomxqyfbwisxwVeieyZTnTxBorvlgqjynzEqwzacgzhooq";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_SETTINGS_260(), &PH);
        p260_time_boot_ms_SET((uint32_t)519995461L, PH.base.pack) ;
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_IMAGE, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STORAGE_INFORMATION_261(), &PH);
        p261_storage_id_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p261_total_capacity_SET((float) -5.5119623E37F, PH.base.pack) ;
        p261_used_capacity_SET((float) -7.621143E37F, PH.base.pack) ;
        p261_write_speed_SET((float)5.916462E37F, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)2550129966L, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)121, PH.base.pack) ;
        p261_available_capacity_SET((float) -2.7072228E38F, PH.base.pack) ;
        p261_read_speed_SET((float) -2.0171698E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_image_interval_SET((float)2.5692992E38F, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p262_available_capacity_SET((float)1.1875125E38F, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)2959207391L, PH.base.pack) ;
        p262_recording_time_ms_SET((uint32_t)3750280262L, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        {
            float q[] =  {-3.7829374E37F, -3.0871349E38F, 2.3994435E38F, -3.1774388E38F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_lat_SET((int32_t) -2141530489, PH.base.pack) ;
        p263_relative_alt_SET((int32_t) -1532092643, PH.base.pack) ;
        {
            char16_t* file_url = u"fpfxZphqItzjejmddviHzoqoidbsIQsruxnzkisWuwoqimCthondskirwhzneWdMSvcPowbrxjhXcxvkiqbhiNedjsswsewotvsbzotiyamsyAykemoyipKirppmyfsofvctzpjlijfmoryWwuqFtvhnuAuekquNdVsftuxpsqODtafgoncbQemzyoxPcalzzrdxjuWzg";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_time_utc_SET((uint64_t)6883171120879632120L, PH.base.pack) ;
        p263_camera_id_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p263_lon_SET((int32_t) -1610112896, PH.base.pack) ;
        p263_image_index_SET((int32_t)1602444087, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)3582287015L, PH.base.pack) ;
        p263_alt_SET((int32_t) -2075375779, PH.base.pack) ;
        p263_capture_result_SET((int8_t)(int8_t) -86, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_time_boot_ms_SET((uint32_t)2036326861L, PH.base.pack) ;
        p264_takeoff_time_utc_SET((uint64_t)2524230066473017737L, PH.base.pack) ;
        p264_arming_time_utc_SET((uint64_t)7256928325321178409L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)8928180109364118318L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_time_boot_ms_SET((uint32_t)2997417993L, PH.base.pack) ;
        p265_roll_SET((float)1.5182469E38F, PH.base.pack) ;
        p265_pitch_SET((float)2.6662027E38F, PH.base.pack) ;
        p265_yaw_SET((float)5.1389577E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_266(), &PH);
        p266_target_system_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        p266_length_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)62, (uint8_t)186, (uint8_t)141, (uint8_t)53, (uint8_t)145, (uint8_t)10, (uint8_t)16, (uint8_t)130, (uint8_t)218, (uint8_t)129, (uint8_t)50, (uint8_t)67, (uint8_t)58, (uint8_t)216, (uint8_t)6, (uint8_t)240, (uint8_t)90, (uint8_t)173, (uint8_t)90, (uint8_t)57, (uint8_t)184, (uint8_t)224, (uint8_t)93, (uint8_t)63, (uint8_t)243, (uint8_t)72, (uint8_t)248, (uint8_t)81, (uint8_t)43, (uint8_t)169, (uint8_t)5, (uint8_t)54, (uint8_t)181, (uint8_t)182, (uint8_t)100, (uint8_t)144, (uint8_t)138, (uint8_t)238, (uint8_t)0, (uint8_t)153, (uint8_t)178, (uint8_t)243, (uint8_t)3, (uint8_t)117, (uint8_t)232, (uint8_t)190, (uint8_t)18, (uint8_t)50, (uint8_t)48, (uint8_t)130, (uint8_t)222, (uint8_t)149, (uint8_t)113, (uint8_t)49, (uint8_t)103, (uint8_t)144, (uint8_t)37, (uint8_t)145, (uint8_t)185, (uint8_t)89, (uint8_t)14, (uint8_t)98, (uint8_t)153, (uint8_t)146, (uint8_t)102, (uint8_t)62, (uint8_t)139, (uint8_t)178, (uint8_t)153, (uint8_t)210, (uint8_t)171, (uint8_t)39, (uint8_t)163, (uint8_t)237, (uint8_t)11, (uint8_t)228, (uint8_t)106, (uint8_t)120, (uint8_t)115, (uint8_t)61, (uint8_t)73, (uint8_t)208, (uint8_t)194, (uint8_t)63, (uint8_t)168, (uint8_t)134, (uint8_t)255, (uint8_t)169, (uint8_t)85, (uint8_t)205, (uint8_t)41, (uint8_t)10, (uint8_t)180, (uint8_t)182, (uint8_t)47, (uint8_t)78, (uint8_t)63, (uint8_t)126, (uint8_t)128, (uint8_t)199, (uint8_t)35, (uint8_t)97, (uint8_t)192, (uint8_t)56, (uint8_t)95, (uint8_t)129, (uint8_t)226, (uint8_t)75, (uint8_t)67, (uint8_t)20, (uint8_t)16, (uint8_t)167, (uint8_t)113, (uint8_t)251, (uint8_t)233, (uint8_t)109, (uint8_t)253, (uint8_t)186, (uint8_t)16, (uint8_t)72, (uint8_t)98, (uint8_t)79, (uint8_t)105, (uint8_t)138, (uint8_t)60, (uint8_t)246, (uint8_t)247, (uint8_t)8, (uint8_t)206, (uint8_t)172, (uint8_t)200, (uint8_t)155, (uint8_t)220, (uint8_t)103, (uint8_t)123, (uint8_t)22, (uint8_t)244, (uint8_t)27, (uint8_t)199, (uint8_t)13, (uint8_t)16, (uint8_t)79, (uint8_t)70, (uint8_t)29, (uint8_t)249, (uint8_t)230, (uint8_t)214, (uint8_t)122, (uint8_t)68, (uint8_t)92, (uint8_t)232, (uint8_t)109, (uint8_t)119, (uint8_t)173, (uint8_t)192, (uint8_t)243, (uint8_t)7, (uint8_t)71, (uint8_t)154, (uint8_t)148, (uint8_t)217, (uint8_t)126, (uint8_t)189, (uint8_t)239, (uint8_t)44, (uint8_t)53, (uint8_t)9, (uint8_t)189, (uint8_t)153, (uint8_t)194, (uint8_t)84, (uint8_t)165, (uint8_t)120, (uint8_t)137, (uint8_t)223, (uint8_t)52, (uint8_t)212, (uint8_t)241, (uint8_t)213, (uint8_t)251, (uint8_t)55, (uint8_t)210, (uint8_t)161, (uint8_t)147, (uint8_t)84, (uint8_t)8, (uint8_t)209, (uint8_t)250, (uint8_t)157, (uint8_t)35, (uint8_t)165, (uint8_t)95, (uint8_t)178, (uint8_t)129, (uint8_t)188, (uint8_t)109, (uint8_t)136, (uint8_t)126, (uint8_t)24, (uint8_t)202, (uint8_t)221, (uint8_t)125, (uint8_t)246, (uint8_t)251, (uint8_t)90, (uint8_t)60, (uint8_t)155, (uint8_t)204, (uint8_t)96, (uint8_t)99, (uint8_t)77, (uint8_t)140, (uint8_t)247, (uint8_t)125, (uint8_t)247, (uint8_t)201, (uint8_t)34, (uint8_t)145, (uint8_t)12, (uint8_t)167, (uint8_t)231, (uint8_t)173, (uint8_t)102, (uint8_t)97, (uint8_t)114, (uint8_t)1, (uint8_t)242, (uint8_t)236, (uint8_t)181, (uint8_t)14, (uint8_t)83, (uint8_t)212, (uint8_t)191, (uint8_t)237, (uint8_t)159, (uint8_t)23, (uint8_t)97, (uint8_t)75, (uint8_t)144, (uint8_t)255, (uint8_t)43, (uint8_t)64, (uint8_t)141, (uint8_t)29, (uint8_t)248, (uint8_t)128, (uint8_t)229, (uint8_t)58, (uint8_t)77};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        p266_first_message_offset_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p266_sequence_SET((uint16_t)(uint16_t)5005, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_ACKED_267(), &PH);
        p267_target_component_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        p267_sequence_SET((uint16_t)(uint16_t)64693, PH.base.pack) ;
        p267_length_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        p267_target_system_SET((uint8_t)(uint8_t)87, PH.base.pack) ;
        p267_first_message_offset_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)48, (uint8_t)40, (uint8_t)235, (uint8_t)150, (uint8_t)114, (uint8_t)163, (uint8_t)113, (uint8_t)81, (uint8_t)199, (uint8_t)56, (uint8_t)100, (uint8_t)220, (uint8_t)38, (uint8_t)194, (uint8_t)29, (uint8_t)73, (uint8_t)200, (uint8_t)152, (uint8_t)173, (uint8_t)6, (uint8_t)194, (uint8_t)237, (uint8_t)208, (uint8_t)255, (uint8_t)68, (uint8_t)191, (uint8_t)104, (uint8_t)165, (uint8_t)137, (uint8_t)210, (uint8_t)58, (uint8_t)97, (uint8_t)209, (uint8_t)49, (uint8_t)216, (uint8_t)150, (uint8_t)228, (uint8_t)190, (uint8_t)128, (uint8_t)62, (uint8_t)0, (uint8_t)150, (uint8_t)23, (uint8_t)59, (uint8_t)224, (uint8_t)89, (uint8_t)101, (uint8_t)251, (uint8_t)179, (uint8_t)230, (uint8_t)167, (uint8_t)129, (uint8_t)197, (uint8_t)148, (uint8_t)70, (uint8_t)60, (uint8_t)43, (uint8_t)253, (uint8_t)128, (uint8_t)201, (uint8_t)157, (uint8_t)93, (uint8_t)78, (uint8_t)129, (uint8_t)174, (uint8_t)66, (uint8_t)239, (uint8_t)209, (uint8_t)156, (uint8_t)152, (uint8_t)108, (uint8_t)199, (uint8_t)193, (uint8_t)74, (uint8_t)85, (uint8_t)121, (uint8_t)31, (uint8_t)173, (uint8_t)203, (uint8_t)136, (uint8_t)218, (uint8_t)116, (uint8_t)36, (uint8_t)141, (uint8_t)110, (uint8_t)161, (uint8_t)126, (uint8_t)14, (uint8_t)10, (uint8_t)214, (uint8_t)135, (uint8_t)252, (uint8_t)206, (uint8_t)255, (uint8_t)74, (uint8_t)172, (uint8_t)196, (uint8_t)200, (uint8_t)77, (uint8_t)194, (uint8_t)182, (uint8_t)60, (uint8_t)37, (uint8_t)52, (uint8_t)140, (uint8_t)48, (uint8_t)96, (uint8_t)74, (uint8_t)44, (uint8_t)146, (uint8_t)202, (uint8_t)223, (uint8_t)51, (uint8_t)209, (uint8_t)9, (uint8_t)4, (uint8_t)167, (uint8_t)93, (uint8_t)202, (uint8_t)188, (uint8_t)77, (uint8_t)28, (uint8_t)103, (uint8_t)27, (uint8_t)123, (uint8_t)8, (uint8_t)118, (uint8_t)117, (uint8_t)164, (uint8_t)74, (uint8_t)87, (uint8_t)169, (uint8_t)254, (uint8_t)194, (uint8_t)156, (uint8_t)170, (uint8_t)53, (uint8_t)139, (uint8_t)179, (uint8_t)193, (uint8_t)169, (uint8_t)172, (uint8_t)4, (uint8_t)128, (uint8_t)129, (uint8_t)168, (uint8_t)116, (uint8_t)87, (uint8_t)76, (uint8_t)87, (uint8_t)61, (uint8_t)217, (uint8_t)183, (uint8_t)148, (uint8_t)194, (uint8_t)40, (uint8_t)186, (uint8_t)244, (uint8_t)51, (uint8_t)148, (uint8_t)30, (uint8_t)154, (uint8_t)159, (uint8_t)153, (uint8_t)83, (uint8_t)137, (uint8_t)45, (uint8_t)31, (uint8_t)126, (uint8_t)98, (uint8_t)15, (uint8_t)50, (uint8_t)88, (uint8_t)9, (uint8_t)65, (uint8_t)64, (uint8_t)119, (uint8_t)242, (uint8_t)48, (uint8_t)116, (uint8_t)249, (uint8_t)147, (uint8_t)23, (uint8_t)202, (uint8_t)64, (uint8_t)29, (uint8_t)238, (uint8_t)167, (uint8_t)217, (uint8_t)168, (uint8_t)239, (uint8_t)158, (uint8_t)119, (uint8_t)118, (uint8_t)229, (uint8_t)48, (uint8_t)102, (uint8_t)181, (uint8_t)66, (uint8_t)127, (uint8_t)43, (uint8_t)223, (uint8_t)150, (uint8_t)57, (uint8_t)117, (uint8_t)16, (uint8_t)44, (uint8_t)219, (uint8_t)70, (uint8_t)106, (uint8_t)236, (uint8_t)239, (uint8_t)126, (uint8_t)238, (uint8_t)242, (uint8_t)20, (uint8_t)250, (uint8_t)30, (uint8_t)19, (uint8_t)90, (uint8_t)216, (uint8_t)56, (uint8_t)203, (uint8_t)135, (uint8_t)125, (uint8_t)207, (uint8_t)150, (uint8_t)127, (uint8_t)13, (uint8_t)10, (uint8_t)0, (uint8_t)138, (uint8_t)55, (uint8_t)137, (uint8_t)221, (uint8_t)114, (uint8_t)170, (uint8_t)38, (uint8_t)187, (uint8_t)62, (uint8_t)77, (uint8_t)241, (uint8_t)80, (uint8_t)217, (uint8_t)85, (uint8_t)67, (uint8_t)20, (uint8_t)234, (uint8_t)45};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_ACK_268(), &PH);
        p268_target_system_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p268_sequence_SET((uint16_t)(uint16_t)46751, PH.base.pack) ;
        p268_target_component_SET((uint8_t)(uint8_t)197, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_rotation_SET((uint16_t)(uint16_t)14841, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)54401, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)33565, PH.base.pack) ;
        p269_bitrate_SET((uint32_t)1224112268L, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)217, PH.base.pack) ;
        {
            char16_t* uri = u"ilaltAtwsjuUvjotVmgcugmqswmaeshhqkqlyeexhnzvflkgbmxlptfttoiwryjzfjzyigrxelwxkwrjyizMujigBlLujyhruupbyerttoqypgfdsxtafrcRzeidnfv";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_framerate_SET((float) -1.8199994E38F, PH.base.pack) ;
        p269_status_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_framerate_SET((float)7.969138E37F, PH.base.pack) ;
        {
            char16_t* uri = u"cbaynnTiodmalyErsmpeiuyygWyqguhpnpouxtjtllVwibyIgTeepHvojpaMrrigsexjqskkbcrcCxmhfmftrjtTitjhezvfotiyecdosaxfuiiuvMhhycbfXegeusuybhbccwoqovitQ";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_target_system_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p270_target_component_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        p270_rotation_SET((uint16_t)(uint16_t)340, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)78162329L, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)25233, PH.base.pack) ;
        p270_resolution_h_SET((uint16_t)(uint16_t)26607, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* ssid = u"pjcjPImhvimukwx";
            p299_ssid_SET_(ssid, &PH) ;
        }
        {
            char16_t* password = u"xmzNtjmdwtktqesesyzgmygnhuqcLaopngQyHykbtxmysmwTeKtovJdpjP";
            p299_password_SET_(password, &PH) ;
        }
        c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PROTOCOL_VERSION_300(), &PH);
        p300_max_version_SET((uint16_t)(uint16_t)12084, PH.base.pack) ;
        p300_min_version_SET((uint16_t)(uint16_t)1004, PH.base.pack) ;
        p300_version_SET((uint16_t)(uint16_t)33216, PH.base.pack) ;
        {
            uint8_t library_version_hash[] =  {(uint8_t)190, (uint8_t)70, (uint8_t)126, (uint8_t)67, (uint8_t)112, (uint8_t)12, (uint8_t)154, (uint8_t)150};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        {
            uint8_t spec_version_hash[] =  {(uint8_t)27, (uint8_t)117, (uint8_t)94, (uint8_t)60, (uint8_t)215, (uint8_t)144, (uint8_t)203, (uint8_t)54};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_SOFTWARE_UPDATE, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)9022781072169948119L, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)3270841763L, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)56979, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_hw_version_minor_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        p311_sw_version_minor_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)190, (uint8_t)174, (uint8_t)130, (uint8_t)85, (uint8_t)141, (uint8_t)204, (uint8_t)157, (uint8_t)185, (uint8_t)230, (uint8_t)251, (uint8_t)140, (uint8_t)127, (uint8_t)168, (uint8_t)62, (uint8_t)73, (uint8_t)94};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        {
            char16_t* name = u"eaFShltsruoswLrxkneeiwaSjZhwioxtcmWtkyjishhcznZdzwJlc";
            p311_name_SET_(name, &PH) ;
        }
        p311_sw_version_major_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p311_time_usec_SET((uint64_t)4368929917618989823L, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)3422544570L, PH.base.pack) ;
        p311_uptime_sec_SET((uint32_t)1467979133L, PH.base.pack) ;
        p311_hw_version_major_SET((uint8_t)(uint8_t)167, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        p320_param_index_SET((int16_t)(int16_t)26385, PH.base.pack) ;
        p320_target_component_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        p320_target_system_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        {
            char16_t* param_id = u"scmbgoomgzCkyki";
            p320_param_id_SET_(param_id, &PH) ;
        }
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_system_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p321_target_component_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_VALUE_322(), &PH);
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32, PH.base.pack) ;
        p322_param_count_SET((uint16_t)(uint16_t)31005, PH.base.pack) ;
        {
            char16_t* param_value = u"gRojflvmswqhgboerjzlqzoTOoofobgBblZjrtwngxOwPrascexplruwcugvkSpzcttuHjcgmjzhvwqhzyWbgknjoiqofdpgfwgibwkMZCrzghfnHjiTknuvdXl";
            p322_param_value_SET_(param_value, &PH) ;
        }
        {
            char16_t* param_id = u"oc";
            p322_param_id_SET_(param_id, &PH) ;
        }
        p322_param_index_SET((uint16_t)(uint16_t)38434, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_SET_323(), &PH);
        {
            char16_t* param_id = u"gf";
            p323_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"DvcfaqjtVVTCpwdjdGdgvsgocigqghlviqitRuwzemcffrVposgTmuibphvxhgfnusyaszmkroviwkapZwhZloqodLhozvaymnzlugirkdvoWpnhdljmnvvfrj";
            p323_param_value_SET_(param_value, &PH) ;
        }
        p323_target_component_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32, PH.base.pack) ;
        p323_target_system_SET((uint8_t)(uint8_t)118, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_ACK_324(), &PH);
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32, PH.base.pack) ;
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_ACCEPTED, PH.base.pack) ;
        {
            char16_t* param_id = u"zSslis";
            p324_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"odPoGeptocycvonryjtbgwxrmhtvWxbLdukpjmwblNqddaFrppqyHptufDglpkZglqvOesFjgUsNprqwemoiohelkekgImxspiljxaj";
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
            uint16_t distances[] =  {(uint16_t)12731, (uint16_t)34786, (uint16_t)28028, (uint16_t)38597, (uint16_t)10089, (uint16_t)59843, (uint16_t)36581, (uint16_t)5266, (uint16_t)23594, (uint16_t)53632, (uint16_t)46111, (uint16_t)27042, (uint16_t)41816, (uint16_t)15032, (uint16_t)21306, (uint16_t)29794, (uint16_t)64363, (uint16_t)45337, (uint16_t)56077, (uint16_t)33592, (uint16_t)36222, (uint16_t)46881, (uint16_t)55688, (uint16_t)14223, (uint16_t)32834, (uint16_t)61090, (uint16_t)60147, (uint16_t)49153, (uint16_t)44005, (uint16_t)6920, (uint16_t)18620, (uint16_t)16077, (uint16_t)55969, (uint16_t)36570, (uint16_t)5341, (uint16_t)37035, (uint16_t)19013, (uint16_t)49248, (uint16_t)2095, (uint16_t)25531, (uint16_t)42889, (uint16_t)57341, (uint16_t)14115, (uint16_t)22219, (uint16_t)42394, (uint16_t)52735, (uint16_t)1013, (uint16_t)45312, (uint16_t)5235, (uint16_t)28643, (uint16_t)49766, (uint16_t)3860, (uint16_t)10780, (uint16_t)53670, (uint16_t)5682, (uint16_t)62646, (uint16_t)15633, (uint16_t)45427, (uint16_t)33910, (uint16_t)48965, (uint16_t)3265, (uint16_t)49637, (uint16_t)50928, (uint16_t)7475, (uint16_t)19766, (uint16_t)27698, (uint16_t)23564, (uint16_t)2416, (uint16_t)55405, (uint16_t)6752, (uint16_t)31361, (uint16_t)34594};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_min_distance_SET((uint16_t)(uint16_t)25196, PH.base.pack) ;
        p330_max_distance_SET((uint16_t)(uint16_t)24944, PH.base.pack) ;
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, PH.base.pack) ;
        p330_increment_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
        p330_time_usec_SET((uint64_t)8269500430103756781L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVIONIX_ADSB_OUT_CFG_10001(), &PH);
        {
            char16_t* callsign = u"nmo";
            p10001_callsign_SET_(callsign, &PH) ;
        }
        p10001_gpsOffsetLat_SET(e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_2M, PH.base.pack) ;
        p10001_aircraftSize_SET(e_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_39P5M, PH.base.pack) ;
        p10001_gpsOffsetLon_SET(e_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR, PH.base.pack) ;
        p10001_stallSpeed_SET((uint16_t)(uint16_t)59939, PH.base.pack) ;
        p10001_emitterType_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_GLIDER, PH.base.pack) ;
        p10001_rfSelect_SET(e_UAVIONIX_ADSB_OUT_RF_SELECT_UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY, PH.base.pack) ;
        p10001_ICAO_SET((uint32_t)1771962925L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVIONIX_ADSB_OUT_CFG_10001(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVIONIX_ADSB_OUT_DYNAMIC_10002(), &PH);
        p10002_accuracyVert_SET((uint16_t)(uint16_t)3188, PH.base.pack) ;
        p10002_gpsLat_SET((int32_t)1783223206, PH.base.pack) ;
        p10002_gpsFix_SET(e_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_RTK, PH.base.pack) ;
        p10002_state_SET(e_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT, PH.base.pack) ;
        p10002_squawk_SET((uint16_t)(uint16_t)43361, PH.base.pack) ;
        p10002_numSats_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        p10002_accuracyHor_SET((uint32_t)3007745439L, PH.base.pack) ;
        p10002_VelEW_SET((int16_t)(int16_t) -28122, PH.base.pack) ;
        p10002_baroAltMSL_SET((int32_t) -443676971, PH.base.pack) ;
        p10002_velNS_SET((int16_t)(int16_t) -5642, PH.base.pack) ;
        p10002_emergencyStatus_SET(e_UAVIONIX_ADSB_EMERGENCY_STATUS_UAVIONIX_ADSB_OUT_UNLAWFUL_INTERFERANCE_EMERGENCY, PH.base.pack) ;
        p10002_velVert_SET((int16_t)(int16_t)13633, PH.base.pack) ;
        p10002_gpsAlt_SET((int32_t)1127721829, PH.base.pack) ;
        p10002_accuracyVel_SET((uint16_t)(uint16_t)37539, PH.base.pack) ;
        p10002_gpsLon_SET((int32_t)234310433, PH.base.pack) ;
        p10002_utcTime_SET((uint32_t)3072621348L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVIONIX_ADSB_OUT_DYNAMIC_10002(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_10003(), &PH);
        p10003_rfHealth_SET(e_UAVIONIX_ADSB_RF_HEALTH_UAVIONIX_ADSB_RF_HEALTH_FAIL_TX, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_10003(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEVICE_OP_READ_11000(), &PH);
        p11000_regstart_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        p11000_request_id_SET((uint32_t)618767686L, PH.base.pack) ;
        p11000_target_component_SET((uint8_t)(uint8_t)47, PH.base.pack) ;
        p11000_count_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p11000_bustype_SET(e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI, PH.base.pack) ;
        p11000_bus_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        {
            char16_t* busname = u"bsXtcikpKpnxt";
            p11000_busname_SET_(busname, &PH) ;
        }
        p11000_address_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p11000_target_system_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEVICE_OP_READ_11000(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEVICE_OP_READ_REPLY_11001(), &PH);
        p11001_regstart_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)243, (uint8_t)157, (uint8_t)88, (uint8_t)57, (uint8_t)218, (uint8_t)197, (uint8_t)104, (uint8_t)218, (uint8_t)38, (uint8_t)76, (uint8_t)119, (uint8_t)89, (uint8_t)177, (uint8_t)20, (uint8_t)150, (uint8_t)254, (uint8_t)66, (uint8_t)80, (uint8_t)63, (uint8_t)208, (uint8_t)89, (uint8_t)93, (uint8_t)54, (uint8_t)39, (uint8_t)211, (uint8_t)201, (uint8_t)229, (uint8_t)214, (uint8_t)167, (uint8_t)57, (uint8_t)179, (uint8_t)129, (uint8_t)48, (uint8_t)30, (uint8_t)15, (uint8_t)158, (uint8_t)205, (uint8_t)73, (uint8_t)191, (uint8_t)248, (uint8_t)169, (uint8_t)253, (uint8_t)92, (uint8_t)243, (uint8_t)92, (uint8_t)235, (uint8_t)202, (uint8_t)99, (uint8_t)46, (uint8_t)94, (uint8_t)61, (uint8_t)185, (uint8_t)31, (uint8_t)124, (uint8_t)220, (uint8_t)13, (uint8_t)217, (uint8_t)88, (uint8_t)15, (uint8_t)247, (uint8_t)54, (uint8_t)36, (uint8_t)198, (uint8_t)31, (uint8_t)249, (uint8_t)229, (uint8_t)61, (uint8_t)165, (uint8_t)158, (uint8_t)173, (uint8_t)159, (uint8_t)57, (uint8_t)81, (uint8_t)74, (uint8_t)100, (uint8_t)164, (uint8_t)230, (uint8_t)9, (uint8_t)29, (uint8_t)38, (uint8_t)230, (uint8_t)98, (uint8_t)226, (uint8_t)174, (uint8_t)214, (uint8_t)209, (uint8_t)225, (uint8_t)77, (uint8_t)57, (uint8_t)216, (uint8_t)166, (uint8_t)24, (uint8_t)126, (uint8_t)93, (uint8_t)233, (uint8_t)19, (uint8_t)128, (uint8_t)188, (uint8_t)194, (uint8_t)12, (uint8_t)135, (uint8_t)210, (uint8_t)215, (uint8_t)44, (uint8_t)71, (uint8_t)73, (uint8_t)50, (uint8_t)253, (uint8_t)213, (uint8_t)26, (uint8_t)130, (uint8_t)8, (uint8_t)176, (uint8_t)247, (uint8_t)252, (uint8_t)24, (uint8_t)226, (uint8_t)31, (uint8_t)209, (uint8_t)143, (uint8_t)251, (uint8_t)138, (uint8_t)152, (uint8_t)216, (uint8_t)100, (uint8_t)212, (uint8_t)197, (uint8_t)28};
            p11001_data__SET(&data_, 0, PH.base.pack) ;
        }
        p11001_result_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        p11001_request_id_SET((uint32_t)1855621796L, PH.base.pack) ;
        p11001_count_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEVICE_OP_READ_REPLY_11001(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEVICE_OP_WRITE_11002(), &PH);
        p11002_target_system_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
        p11002_request_id_SET((uint32_t)1779613397L, PH.base.pack) ;
        p11002_address_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p11002_target_component_SET((uint8_t)(uint8_t)207, PH.base.pack) ;
        {
            char16_t* busname = u"njgqfuuyjfutokXzyvfczfoqdxRmzakqjuJseh";
            p11002_busname_SET_(busname, &PH) ;
        }
        p11002_count_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        p11002_bus_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
        p11002_bustype_SET(e_DEVICE_OP_BUSTYPE_DEVICE_OP_BUSTYPE_SPI, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)125, (uint8_t)231, (uint8_t)128, (uint8_t)93, (uint8_t)87, (uint8_t)244, (uint8_t)248, (uint8_t)93, (uint8_t)92, (uint8_t)244, (uint8_t)236, (uint8_t)97, (uint8_t)175, (uint8_t)161, (uint8_t)141, (uint8_t)187, (uint8_t)121, (uint8_t)208, (uint8_t)208, (uint8_t)94, (uint8_t)225, (uint8_t)235, (uint8_t)229, (uint8_t)235, (uint8_t)161, (uint8_t)188, (uint8_t)221, (uint8_t)73, (uint8_t)96, (uint8_t)216, (uint8_t)13, (uint8_t)41, (uint8_t)125, (uint8_t)141, (uint8_t)179, (uint8_t)93, (uint8_t)135, (uint8_t)167, (uint8_t)94, (uint8_t)245, (uint8_t)157, (uint8_t)245, (uint8_t)19, (uint8_t)65, (uint8_t)86, (uint8_t)148, (uint8_t)47, (uint8_t)6, (uint8_t)30, (uint8_t)139, (uint8_t)86, (uint8_t)110, (uint8_t)226, (uint8_t)183, (uint8_t)197, (uint8_t)75, (uint8_t)134, (uint8_t)43, (uint8_t)193, (uint8_t)96, (uint8_t)131, (uint8_t)248, (uint8_t)126, (uint8_t)186, (uint8_t)71, (uint8_t)105, (uint8_t)66, (uint8_t)240, (uint8_t)226, (uint8_t)189, (uint8_t)35, (uint8_t)246, (uint8_t)226, (uint8_t)103, (uint8_t)228, (uint8_t)230, (uint8_t)233, (uint8_t)175, (uint8_t)233, (uint8_t)110, (uint8_t)210, (uint8_t)83, (uint8_t)52, (uint8_t)173, (uint8_t)243, (uint8_t)171, (uint8_t)37, (uint8_t)151, (uint8_t)51, (uint8_t)2, (uint8_t)100, (uint8_t)127, (uint8_t)54, (uint8_t)207, (uint8_t)98, (uint8_t)131, (uint8_t)153, (uint8_t)194, (uint8_t)229, (uint8_t)11, (uint8_t)209, (uint8_t)159, (uint8_t)224, (uint8_t)107, (uint8_t)52, (uint8_t)155, (uint8_t)30, (uint8_t)52, (uint8_t)102, (uint8_t)115, (uint8_t)158, (uint8_t)206, (uint8_t)194, (uint8_t)8, (uint8_t)136, (uint8_t)203, (uint8_t)175, (uint8_t)160, (uint8_t)232, (uint8_t)173, (uint8_t)32, (uint8_t)34, (uint8_t)122, (uint8_t)3, (uint8_t)207, (uint8_t)22, (uint8_t)150, (uint8_t)156};
            p11002_data__SET(&data_, 0, PH.base.pack) ;
        }
        p11002_regstart_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEVICE_OP_WRITE_11002(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEVICE_OP_WRITE_REPLY_11003(), &PH);
        p11003_request_id_SET((uint32_t)1887325500L, PH.base.pack) ;
        p11003_result_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEVICE_OP_WRITE_REPLY_11003(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ADAP_TUNING_11010(), &PH);
        p11010_achieved_SET((float)2.3573397E38F, PH.base.pack) ;
        p11010_u_SET((float) -1.5637594E38F, PH.base.pack) ;
        p11010_omega_SET((float)8.447634E37F, PH.base.pack) ;
        p11010_sigma_dot_SET((float)2.1316E38F, PH.base.pack) ;
        p11010_theta_dot_SET((float) -9.139858E37F, PH.base.pack) ;
        p11010_axis_SET(e_PID_TUNING_AXIS_PID_TUNING_STEER, PH.base.pack) ;
        p11010_sigma_SET((float) -7.4927125E37F, PH.base.pack) ;
        p11010_theta_SET((float)2.8962885E38F, PH.base.pack) ;
        p11010_omega_dot_SET((float) -8.996182E37F, PH.base.pack) ;
        p11010_f_SET((float) -5.6073536E37F, PH.base.pack) ;
        p11010_f_dot_SET((float)4.0589475E37F, PH.base.pack) ;
        p11010_error_SET((float) -2.6518616E38F, PH.base.pack) ;
        p11010_desired_SET((float)2.8714495E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ADAP_TUNING_11010(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_POSITION_DELTA_11011(), &PH);
        p11011_time_delta_usec_SET((uint64_t)782001265752966504L, PH.base.pack) ;
        p11011_confidence_SET((float) -1.761705E38F, PH.base.pack) ;
        {
            float position_delta[] =  {2.3189907E38F, 1.9596842E38F, -1.3826785E38F};
            p11011_position_delta_SET(&position_delta, 0, PH.base.pack) ;
        }
        {
            float angle_delta[] =  {2.0390608E38F, -7.187309E37F, 5.103497E37F};
            p11011_angle_delta_SET(&angle_delta, 0, PH.base.pack) ;
        }
        p11011_time_usec_SET((uint64_t)2937697154875305446L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_POSITION_DELTA_11011(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
}

