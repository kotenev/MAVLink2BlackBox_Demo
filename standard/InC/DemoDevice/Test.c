
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
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_GCS);
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_BOOT);
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC);
    assert(p0_custom_mode_GET(pack) == (uint32_t)746907793L);
    assert(p0_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED);
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)95);
};


void c_LoopBackDemoChannel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)38054);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)4954);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)48580);
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)57976);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t)30304);
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)40641);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)57890);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)31357);
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t) -22);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)16841);
    assert(p1_onboard_control_sensors_health_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH);
    assert(p1_onboard_control_sensors_present_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING);
};


void c_LoopBackDemoChannel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)850154985L);
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)7940413277649479192L);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_z_GET(pack) == (float)2.4238118E37F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p3_afz_GET(pack) == (float) -1.773599E38F);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)23857);
    assert(p3_afx_GET(pack) == (float) -7.9932794E37F);
    assert(p3_afy_GET(pack) == (float)7.261288E37F);
    assert(p3_yaw_rate_GET(pack) == (float) -3.1946975E38F);
    assert(p3_y_GET(pack) == (float) -3.0857315E38F);
    assert(p3_vx_GET(pack) == (float)1.297936E38F);
    assert(p3_x_GET(pack) == (float)2.190926E38F);
    assert(p3_vy_GET(pack) == (float)1.9341914E38F);
    assert(p3_yaw_GET(pack) == (float) -7.727453E37F);
    assert(p3_vz_GET(pack) == (float)3.3848547E38F);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)2661330200L);
};


void c_LoopBackDemoChannel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)74);
    assert(p4_seq_GET(pack) == (uint32_t)595581859L);
    assert(p4_time_usec_GET(pack) == (uint64_t)4980911259099929723L);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)229);
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_passkey_LEN(ph) == 16);
    {
        char16_t * exemplary = u"tsttbgfeohxlnqfx";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)187);
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)169);
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)51);
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)155);
};


void c_LoopBackDemoChannel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 8);
    {
        char16_t * exemplary = u"zsIxqvat";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)251);
    assert(p11_custom_mode_GET(pack) == (uint32_t)2350801323L);
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_AUTO_ARMED);
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"goqfAejqguvna";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)12);
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t) -11298);
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)221);
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)245);
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)82);
};


void c_LoopBackDemoChannel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)8819);
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32);
    assert(p22_param_value_GET(pack) == (float)2.3736376E38F);
    assert(p22_param_id_LEN(ph) == 12);
    {
        char16_t * exemplary = u"wEOntrnvisvc";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)40100);
};


void c_LoopBackDemoChannel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16);
    assert(p23_param_id_LEN(ph) == 16);
    {
        char16_t * exemplary = u"bfHflMHthQvLbguq";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_param_value_GET(pack) == (float) -3.1535969E38F);
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)84);
};


void c_LoopBackDemoChannel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)14206);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT);
    assert(p24_v_acc_TRY(ph) == (uint32_t)4158275042L);
    assert(p24_h_acc_TRY(ph) == (uint32_t)3582498266L);
    assert(p24_alt_GET(pack) == (int32_t) -1765063902);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)44720);
    assert(p24_time_usec_GET(pack) == (uint64_t)905128649725603582L);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)18);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)62243);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)30953);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)2163871150L);
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t) -286335610);
    assert(p24_lon_GET(pack) == (int32_t) -138596825);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)1298678203L);
    assert(p24_lat_GET(pack) == (int32_t) -791806173);
};


void c_LoopBackDemoChannel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)177, (uint8_t)206, (uint8_t)240, (uint8_t)172, (uint8_t)44, (uint8_t)253, (uint8_t)212, (uint8_t)29, (uint8_t)202, (uint8_t)162, (uint8_t)109, (uint8_t)18, (uint8_t)21, (uint8_t)91, (uint8_t)129, (uint8_t)199, (uint8_t)121, (uint8_t)223, (uint8_t)81, (uint8_t)18} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)219);
    {
        uint8_t exemplary[] =  {(uint8_t)182, (uint8_t)158, (uint8_t)26, (uint8_t)34, (uint8_t)230, (uint8_t)187, (uint8_t)178, (uint8_t)88, (uint8_t)184, (uint8_t)210, (uint8_t)3, (uint8_t)7, (uint8_t)119, (uint8_t)24, (uint8_t)242, (uint8_t)121, (uint8_t)84, (uint8_t)45, (uint8_t)227, (uint8_t)134} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)23, (uint8_t)129, (uint8_t)104, (uint8_t)55, (uint8_t)200, (uint8_t)218, (uint8_t)190, (uint8_t)29, (uint8_t)231, (uint8_t)91, (uint8_t)92, (uint8_t)62, (uint8_t)231, (uint8_t)145, (uint8_t)21, (uint8_t)95, (uint8_t)89, (uint8_t)213, (uint8_t)115, (uint8_t)83} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)81, (uint8_t)4, (uint8_t)158, (uint8_t)175, (uint8_t)67, (uint8_t)220, (uint8_t)143, (uint8_t)96, (uint8_t)253, (uint8_t)224, (uint8_t)249, (uint8_t)24, (uint8_t)78, (uint8_t)242, (uint8_t)193, (uint8_t)56, (uint8_t)204, (uint8_t)20, (uint8_t)77, (uint8_t)94} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)73, (uint8_t)144, (uint8_t)155, (uint8_t)232, (uint8_t)92, (uint8_t)199, (uint8_t)104, (uint8_t)12, (uint8_t)28, (uint8_t)104, (uint8_t)86, (uint8_t)122, (uint8_t)206, (uint8_t)245, (uint8_t)33, (uint8_t)148, (uint8_t)105, (uint8_t)190, (uint8_t)132, (uint8_t)44} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t) -26484);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t) -25930);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t)32076);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t) -6022);
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t)13393);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t)30334);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t)25933);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t) -27760);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)975908389L);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t) -3616);
};


void c_LoopBackDemoChannel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t) -28565);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t)4865);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t) -29135);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t) -5381);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t)31896);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t) -9492);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t) -538);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t)7495);
    assert(p27_time_usec_GET(pack) == (uint64_t)4074352477166708791L);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t)27866);
};


void c_LoopBackDemoChannel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_time_usec_GET(pack) == (uint64_t)5161584472728574185L);
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t) -2127);
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t)3951);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t) -26296);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t) -16549);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t)2234);
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)3293051552L);
    assert(p29_press_diff_GET(pack) == (float)3.0333849E38F);
    assert(p29_press_abs_GET(pack) == (float) -2.1582181E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_pitch_GET(pack) == (float)1.7588532E38F);
    assert(p30_rollspeed_GET(pack) == (float) -2.6899653E38F);
    assert(p30_yawspeed_GET(pack) == (float) -6.296374E37F);
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)176238102L);
    assert(p30_pitchspeed_GET(pack) == (float) -3.3157582E37F);
    assert(p30_yaw_GET(pack) == (float) -2.279624E38F);
    assert(p30_roll_GET(pack) == (float) -8.2994023E37F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_q3_GET(pack) == (float)2.0179992E38F);
    assert(p31_pitchspeed_GET(pack) == (float)1.6670614E38F);
    assert(p31_rollspeed_GET(pack) == (float) -3.2483175E38F);
    assert(p31_yawspeed_GET(pack) == (float) -1.4434092E38F);
    assert(p31_q1_GET(pack) == (float) -9.645267E37F);
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)421225493L);
    assert(p31_q2_GET(pack) == (float)3.0786025E38F);
    assert(p31_q4_GET(pack) == (float) -2.0284744E38F);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_z_GET(pack) == (float)1.5706201E38F);
    assert(p32_vx_GET(pack) == (float) -1.9770617E38F);
    assert(p32_y_GET(pack) == (float) -2.3640734E38F);
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)3861791601L);
    assert(p32_vz_GET(pack) == (float)2.8581842E37F);
    assert(p32_x_GET(pack) == (float)1.3391524E38F);
    assert(p32_vy_GET(pack) == (float) -1.776045E38F);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_lon_GET(pack) == (int32_t)1906645972);
    assert(p33_lat_GET(pack) == (int32_t)809838976);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t) -27250);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t) -17795);
    assert(p33_alt_GET(pack) == (int32_t)1233241208);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t)9235);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)4059788384L);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)26907);
    assert(p33_relative_alt_GET(pack) == (int32_t)856551769);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)3691972021L);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t) -18502);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t)30477);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)36);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t) -15920);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t)9380);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)122);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t)23330);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t)22233);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t)6496);
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t) -24542);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)21242);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)42581);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)46311);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)20429);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)26268);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)15045);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)45569);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)942046927L);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)62270);
};


void c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)141);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)28194);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)2026);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)29813);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)49786);
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)36013);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)50673);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)8768);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)8033);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)43516);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)28875);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)33611);
    assert(p36_time_usec_GET(pack) == (uint32_t)1270565008L);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)342);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)64955);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)57721);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)13271);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)56726);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)32);
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t)31185);
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t)21592);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
};


void c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t)12819);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t) -16701);
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)158);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_x_GET(pack) == (float) -2.6381046E38F);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)116);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION);
    assert(p39_param4_GET(pack) == (float)1.1223545E38F);
    assert(p39_param2_GET(pack) == (float)3.0213906E38F);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)2064);
    assert(p39_z_GET(pack) == (float) -7.192847E37F);
    assert(p39_param1_GET(pack) == (float)2.4267682E38F);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)236);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)94);
    assert(p39_param3_GET(pack) == (float) -4.6586966E37F);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p39_y_GET(pack) == (float)2.3074478E38F);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)57);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)37106);
};


void c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)183);
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)52248);
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)136);
};


void c_LoopBackDemoChannel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)854);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)208);
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)180);
};


void c_LoopBackDemoChannel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)161);
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)31339);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)109);
};


void c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)104);
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)20);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)5213);
};


void c_LoopBackDemoChannel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)169);
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_INVALID);
};


void c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_latitude_GET(pack) == (int32_t) -114563872);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)162);
    assert(p48_time_usec_TRY(ph) == (uint64_t)5866821489129040147L);
    assert(p48_altitude_GET(pack) == (int32_t) -306749084);
    assert(p48_longitude_GET(pack) == (int32_t)2005871769);
};


void c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_time_usec_TRY(ph) == (uint64_t)8457574272267750069L);
    assert(p49_altitude_GET(pack) == (int32_t) -1539350010);
    assert(p49_latitude_GET(pack) == (int32_t)554305339);
    assert(p49_longitude_GET(pack) == (int32_t)1656970326);
};


void c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)106);
    assert(p50_scale_GET(pack) == (float)1.6821304E38F);
    assert(p50_param_value_min_GET(pack) == (float) -2.4446555E38F);
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)62);
    assert(p50_param_value_max_GET(pack) == (float) -2.833891E38F);
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)231);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t)5149);
    assert(p50_param_value0_GET(pack) == (float) -1.1532727E38F);
    assert(p50_param_id_LEN(ph) == 9);
    {
        char16_t * exemplary = u"JrnyKktwd";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)232);
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)143);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)40240);
};


void c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_p2z_GET(pack) == (float)1.774943E38F);
    assert(p54_p2x_GET(pack) == (float) -8.244991E37F);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)57);
    assert(p54_p1y_GET(pack) == (float)2.6070072E38F);
    assert(p54_p1z_GET(pack) == (float)5.166072E37F);
    assert(p54_p2y_GET(pack) == (float) -1.3577981E38F);
    assert(p54_p1x_GET(pack) == (float) -2.8704327E38F);
};


void c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p1y_GET(pack) == (float) -3.3730546E38F);
    assert(p55_p2y_GET(pack) == (float)2.7627137E38F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p55_p2x_GET(pack) == (float)1.6037819E38F);
    assert(p55_p1z_GET(pack) == (float)3.3092816E38F);
    assert(p55_p1x_GET(pack) == (float)2.2166905E38F);
    assert(p55_p2z_GET(pack) == (float) -1.9684863E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    assert(p61_yawspeed_GET(pack) == (float)2.2356225E38F);
    {
        float exemplary[] =  {-1.2852599E38F, -7.3858303E37F, -3.1682091E38F, 4.512943E37F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_pitchspeed_GET(pack) == (float) -9.964348E35F);
    {
        float exemplary[] =  {-2.249146E38F, -1.5480198E38F, 2.1029532E38F, 3.1823786E38F, 5.7450133E37F, -2.1660753E38F, -1.7790286E38F, -1.5131581E37F, 2.1428425E38F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_rollspeed_GET(pack) == (float) -4.7485324E37F);
    assert(p61_time_usec_GET(pack) == (uint64_t)4902443481483851250L);
};


void c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_nav_roll_GET(pack) == (float)2.614913E38F);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t) -9705);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t)517);
    assert(p62_aspd_error_GET(pack) == (float) -2.1761693E38F);
    assert(p62_nav_pitch_GET(pack) == (float)1.3456571E38F);
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)35433);
    assert(p62_alt_error_GET(pack) == (float)1.2449002E38F);
    assert(p62_xtrack_error_GET(pack) == (float)1.225728E38F);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_lon_GET(pack) == (int32_t) -977267411);
    assert(p63_vx_GET(pack) == (float) -8.666514E37F);
    assert(p63_relative_alt_GET(pack) == (int32_t)605387207);
    assert(p63_time_usec_GET(pack) == (uint64_t)1883987170271804026L);
    assert(p63_vz_GET(pack) == (float) -5.9422567E37F);
    assert(p63_vy_GET(pack) == (float) -2.438959E38F);
    assert(p63_alt_GET(pack) == (int32_t) -737571809);
    {
        float exemplary[] =  {1.9094605E38F, 1.6311393E38F, -2.6800385E38F, -3.2741076E38F, -4.3941795E37F, -3.3497588E38F, 2.6622695E38F, 2.904801E38F, 3.46457E37F, 2.9467824E38F, -3.5186132E37F, 2.576551E38F, 1.7121072E38F, -3.9323995E36F, -2.6324101E38F, -1.8259365E38F, -3.0289085E38F, -2.0484988E38F, -7.5949587E37F, 3.2795233E38F, -1.8636683E38F, -2.649916E38F, 7.033309E37F, 3.3660995E38F, 2.9013036E38F, -1.9544558E38F, -1.1131358E38F, 1.6554268E38F, 1.00363194E37F, -2.658712E38F, -2.1464803E38F, 2.421535E38F, -2.8677064E38F, 1.1664935E38F, -7.5847627E37F, 2.9857849E38F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO);
    assert(p63_lat_GET(pack) == (int32_t)122051686);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO);
    assert(p64_y_GET(pack) == (float) -1.3119562E38F);
    assert(p64_az_GET(pack) == (float) -7.1307456E37F);
    assert(p64_vz_GET(pack) == (float) -2.906276E38F);
    assert(p64_z_GET(pack) == (float)3.0521781E38F);
    assert(p64_ax_GET(pack) == (float)5.454039E37F);
    assert(p64_vx_GET(pack) == (float)1.3335146E38F);
    assert(p64_x_GET(pack) == (float)1.4173287E38F);
    assert(p64_vy_GET(pack) == (float) -1.7038849E38F);
    {
        float exemplary[] =  {-2.4422325E38F, -1.4630636E38F, -3.7148146E37F, 8.71808E37F, 3.0564995E38F, -1.6638921E38F, 2.3391275E38F, -8.813324E37F, 2.821655E38F, -1.3593637E38F, 4.6533385E37F, -1.5341846E38F, 2.946233E37F, -3.2457806E38F, -9.872926E37F, 8.4425114E37F, -3.2768176E38F, -3.3460723E38F, -2.8702289E38F, 2.2582305E38F, 3.3835175E38F, 6.0622576E37F, -8.4485906E37F, -5.3030667E37F, -2.8576169E38F, -3.3395251E38F, 2.2854964E38F, -2.8608943E38F, 9.012564E37F, -8.53777E36F, -1.9249597E38F, 3.322379E38F, -3.1845742E38F, -1.8013755E38F, -7.334434E36F, 2.282915E38F, -3.1725285E38F, -1.9302388E38F, 2.5759413E38F, -2.0123521E38F, -2.7038683E38F, 9.972482E37F, 1.1294098E38F, 2.2622537E38F, -1.714086E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_ay_GET(pack) == (float) -1.4376243E38F);
    assert(p64_time_usec_GET(pack) == (uint64_t)927273469140416714L);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)24496);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)11522);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)26160);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)133);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)30547);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)25255);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)3398520317L);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)11196);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)58404);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)21290);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)1398);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)39606);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)54400);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)25848);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)57030);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)208);
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)65174);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)57166);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)7197);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)56326);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)46880);
};


void c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)242);
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)222);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)6522);
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)41);
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)230);
};


void c_LoopBackDemoChannel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)61);
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)133);
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)40434);
};


void c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_x_GET(pack) == (int16_t)(int16_t)32099);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)21515);
    assert(p69_r_GET(pack) == (int16_t)(int16_t) -26257);
    assert(p69_y_GET(pack) == (int16_t)(int16_t) -12853);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)98);
    assert(p69_z_GET(pack) == (int16_t)(int16_t)30517);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)168);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)157);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)1278);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)28033);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)19407);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)29257);
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)49675);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)21379);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)52683);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)22745);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_param1_GET(pack) == (float)2.6316836E38F);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_PANORAMA_CREATE);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)61);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p73_param3_GET(pack) == (float)5.4893464E37F);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)60802);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)11);
    assert(p73_param4_GET(pack) == (float)6.853591E37F);
    assert(p73_x_GET(pack) == (int32_t)667471254);
    assert(p73_z_GET(pack) == (float)9.128472E37F);
    assert(p73_param2_GET(pack) == (float)2.6176942E38F);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)30);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
    assert(p73_y_GET(pack) == (int32_t)1786577647);
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
};


void c_LoopBackDemoChannel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_airspeed_GET(pack) == (float)1.0901303E38F);
    assert(p74_alt_GET(pack) == (float) -1.7880642E38F);
    assert(p74_climb_GET(pack) == (float)3.0346442E38F);
    assert(p74_groundspeed_GET(pack) == (float)1.601091E38F);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)44351);
    assert(p74_heading_GET(pack) == (int16_t)(int16_t) -26111);
};


void c_LoopBackDemoChannel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)234);
    assert(p75_param2_GET(pack) == (float) -2.0212226E37F);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)237);
    assert(p75_param3_GET(pack) == (float)1.8925975E38F);
    assert(p75_y_GET(pack) == (int32_t) -240960629);
    assert(p75_param1_GET(pack) == (float) -1.159435E38F);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p75_x_GET(pack) == (int32_t)730675154);
    assert(p75_z_GET(pack) == (float) -3.1335564E38F);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO);
    assert(p75_param4_GET(pack) == (float)9.050858E37F);
};


void c_LoopBackDemoChannel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)89);
    assert(p76_param6_GET(pack) == (float) -1.751839E38F);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_CONDITION_GATE);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)182);
    assert(p76_param3_GET(pack) == (float)1.2937754E38F);
    assert(p76_param1_GET(pack) == (float)2.6404622E37F);
    assert(p76_param7_GET(pack) == (float)1.6666054E38F);
    assert(p76_param4_GET(pack) == (float) -7.826144E36F);
    assert(p76_param5_GET(pack) == (float)2.7045242E38F);
    assert(p76_param2_GET(pack) == (float) -7.640786E37F);
};


void c_LoopBackDemoChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)84);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)54);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_IN_PROGRESS);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)47);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE);
    assert(p77_result_param2_TRY(ph) == (int32_t)1602185094);
};


void c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_pitch_GET(pack) == (float)2.0009218E38F);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p81_roll_GET(pack) == (float)1.701262E38F);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)1880885260L);
    assert(p81_yaw_GET(pack) == (float)2.6793304E38F);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)160);
    assert(p81_thrust_GET(pack) == (float) -1.5059035E38F);
};


void c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_body_pitch_rate_GET(pack) == (float) -1.2862223E38F);
    {
        float exemplary[] =  {-3.2035435E38F, -1.4750304E38F, 1.9951666E38F, -2.5166097E38F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_body_roll_rate_GET(pack) == (float)1.5863066E38F);
    assert(p82_body_yaw_rate_GET(pack) == (float) -1.8479952E38F);
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p82_thrust_GET(pack) == (float) -4.3332516E37F);
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)3994634529L);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)243);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)24);
};


void c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)3797340474L);
    assert(p83_thrust_GET(pack) == (float)2.7415995E38F);
    {
        float exemplary[] =  {2.5391188E38F, -3.3804402E38F, -3.179131E38F, -3.303055E38F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p83_body_roll_rate_GET(pack) == (float)2.6378551E38F);
    assert(p83_body_yaw_rate_GET(pack) == (float)3.054227E37F);
    assert(p83_body_pitch_rate_GET(pack) == (float) -8.731666E37F);
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)13);
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_y_GET(pack) == (float)2.4183901E38F);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)184);
    assert(p84_vz_GET(pack) == (float) -1.8783422E38F);
    assert(p84_x_GET(pack) == (float)3.0581614E38F);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)246);
    assert(p84_vy_GET(pack) == (float)1.2254845E38F);
    assert(p84_afx_GET(pack) == (float)2.8999838E38F);
    assert(p84_afz_GET(pack) == (float)2.263736E38F);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)19936);
    assert(p84_yaw_GET(pack) == (float) -1.2873433E38F);
    assert(p84_afy_GET(pack) == (float) -1.2011093E38F);
    assert(p84_vx_GET(pack) == (float) -8.731988E37F);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)324411328L);
    assert(p84_yaw_rate_GET(pack) == (float) -1.6841942E38F);
    assert(p84_z_GET(pack) == (float)1.0027836E38F);
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_alt_GET(pack) == (float) -1.4898584E38F);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)16);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p86_yaw_rate_GET(pack) == (float) -2.1700945E38F);
    assert(p86_lat_int_GET(pack) == (int32_t) -1152326183);
    assert(p86_vy_GET(pack) == (float)1.074402E37F);
    assert(p86_afz_GET(pack) == (float)1.1310283E38F);
    assert(p86_yaw_GET(pack) == (float)1.2824098E38F);
    assert(p86_lon_int_GET(pack) == (int32_t)336065083);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)23128);
    assert(p86_vx_GET(pack) == (float)4.3437564E37F);
    assert(p86_afy_GET(pack) == (float) -2.9924586E38F);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)2111373414L);
    assert(p86_vz_GET(pack) == (float)1.2446145E38F);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_NED);
    assert(p86_afx_GET(pack) == (float) -2.5147131E38F);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_lat_int_GET(pack) == (int32_t) -1584723288);
    assert(p87_lon_int_GET(pack) == (int32_t)733049159);
    assert(p87_yaw_GET(pack) == (float) -1.6508457E38F);
    assert(p87_yaw_rate_GET(pack) == (float) -3.2737456E38F);
    assert(p87_afx_GET(pack) == (float)2.6753733E37F);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)55699);
    assert(p87_vy_GET(pack) == (float)1.4873251E38F);
    assert(p87_alt_GET(pack) == (float) -3.120065E38F);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p87_afy_GET(pack) == (float) -2.498859E37F);
    assert(p87_afz_GET(pack) == (float)1.6933898E38F);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)671629333L);
    assert(p87_vz_GET(pack) == (float) -2.1526362E38F);
    assert(p87_vx_GET(pack) == (float)1.9668264E38F);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)647235651L);
    assert(p89_y_GET(pack) == (float) -1.2498022E38F);
    assert(p89_yaw_GET(pack) == (float)8.15647E37F);
    assert(p89_pitch_GET(pack) == (float) -7.9969136E36F);
    assert(p89_z_GET(pack) == (float) -1.8038037E38F);
    assert(p89_x_GET(pack) == (float) -1.2040753E38F);
    assert(p89_roll_GET(pack) == (float) -2.4595935E38F);
};


void c_LoopBackDemoChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_yawspeed_GET(pack) == (float)2.9028805E38F);
    assert(p90_lat_GET(pack) == (int32_t)2050340792);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t)18508);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t) -9733);
    assert(p90_roll_GET(pack) == (float)4.450275E37F);
    assert(p90_lon_GET(pack) == (int32_t)1022966771);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t)22312);
    assert(p90_pitchspeed_GET(pack) == (float) -8.696113E37F);
    assert(p90_pitch_GET(pack) == (float) -3.2167782E38F);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t)14466);
    assert(p90_rollspeed_GET(pack) == (float)1.7351014E37F);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t) -32524);
    assert(p90_alt_GET(pack) == (int32_t) -1306726532);
    assert(p90_yaw_GET(pack) == (float) -4.0911268E37F);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t)8987);
    assert(p90_time_usec_GET(pack) == (uint64_t)7754029865813712202L);
};


void c_LoopBackDemoChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_aux3_GET(pack) == (float)1.6017463E38F);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_MANUAL_ARMED);
    assert(p91_pitch_elevator_GET(pack) == (float)2.780021E38F);
    assert(p91_time_usec_GET(pack) == (uint64_t)496895142985832945L);
    assert(p91_aux1_GET(pack) == (float)1.4516475E38F);
    assert(p91_throttle_GET(pack) == (float) -8.4228684E37F);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)132);
    assert(p91_aux4_GET(pack) == (float) -1.9649028E38F);
    assert(p91_aux2_GET(pack) == (float)2.346589E38F);
    assert(p91_roll_ailerons_GET(pack) == (float)1.9149588E38F);
    assert(p91_yaw_rudder_GET(pack) == (float) -2.9663032E38F);
};


void c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)40286);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)9809);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)11664);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)2297);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)42568);
    assert(p92_time_usec_GET(pack) == (uint64_t)2084339140964312961L);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)9444);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)14975);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)27985);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)41312);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)13407);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)10513);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)47161);
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)208);
};


void c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    assert(p93_flags_GET(pack) == (uint64_t)4560366735783475588L);
    {
        float exemplary[] =  {-3.195126E38F, -1.5915898E37F, -2.2240114E38F, 1.9668976E38F, -1.4824636E38F, 2.0263243E38F, 1.1031579E38F, -1.0098248E38F, 1.4833578E38F, 1.8883102E38F, 3.5822739E37F, 3.3593745E37F, 3.3614632E38F, 3.247691E38F, 2.4371155E38F, -1.0000517E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_PREFLIGHT);
    assert(p93_time_usec_GET(pack) == (uint64_t)2567488371794832959L);
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t) -14798);
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t) -8275);
    assert(p100_flow_comp_m_y_GET(pack) == (float)5.7506407E37F);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)112);
    assert(p100_flow_comp_m_x_GET(pack) == (float) -2.6348791E38F);
    assert(p100_flow_rate_x_TRY(ph) == (float) -2.3237506E38F);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)64);
    assert(p100_ground_distance_GET(pack) == (float)2.8139557E38F);
    assert(p100_flow_rate_y_TRY(ph) == (float) -4.71217E37F);
    assert(p100_time_usec_GET(pack) == (uint64_t)224374894770255707L);
};


void c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_y_GET(pack) == (float) -1.1378702E38F);
    assert(p101_roll_GET(pack) == (float) -5.929979E37F);
    assert(p101_x_GET(pack) == (float) -8.828578E37F);
    assert(p101_yaw_GET(pack) == (float)1.7055835E38F);
    assert(p101_pitch_GET(pack) == (float) -2.222416E38F);
    assert(p101_z_GET(pack) == (float)1.6986359E38F);
    assert(p101_usec_GET(pack) == (uint64_t)7553001424720102822L);
};


void c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_z_GET(pack) == (float)2.1888509E38F);
    assert(p102_yaw_GET(pack) == (float) -2.6006246E38F);
    assert(p102_x_GET(pack) == (float)3.2330455E38F);
    assert(p102_roll_GET(pack) == (float) -1.4586123E37F);
    assert(p102_y_GET(pack) == (float) -2.4108682E38F);
    assert(p102_pitch_GET(pack) == (float) -1.1842903E38F);
    assert(p102_usec_GET(pack) == (uint64_t)6789320262507273684L);
};


void c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_x_GET(pack) == (float)1.198924E38F);
    assert(p103_y_GET(pack) == (float)1.2408964E38F);
    assert(p103_z_GET(pack) == (float) -1.6798537E38F);
    assert(p103_usec_GET(pack) == (uint64_t)1467612833915047019L);
};


void c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_yaw_GET(pack) == (float) -2.7334092E38F);
    assert(p104_pitch_GET(pack) == (float)1.2081625E38F);
    assert(p104_z_GET(pack) == (float) -1.6172556E38F);
    assert(p104_y_GET(pack) == (float) -1.5625364E38F);
    assert(p104_usec_GET(pack) == (uint64_t)4384105150636666823L);
    assert(p104_x_GET(pack) == (float)7.3203E37F);
    assert(p104_roll_GET(pack) == (float)6.657163E37F);
};


void c_LoopBackDemoChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_xmag_GET(pack) == (float) -2.7177632E38F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)5790);
    assert(p105_xacc_GET(pack) == (float)4.1363335E37F);
    assert(p105_yacc_GET(pack) == (float)1.5959531E37F);
    assert(p105_diff_pressure_GET(pack) == (float)1.2877676E37F);
    assert(p105_abs_pressure_GET(pack) == (float)2.6310174E37F);
    assert(p105_zmag_GET(pack) == (float)1.800658E37F);
    assert(p105_xgyro_GET(pack) == (float)1.1101039E38F);
    assert(p105_pressure_alt_GET(pack) == (float)1.734111E38F);
    assert(p105_time_usec_GET(pack) == (uint64_t)8549239154874540207L);
    assert(p105_zgyro_GET(pack) == (float) -5.7946043E37F);
    assert(p105_temperature_GET(pack) == (float) -2.6582674E38F);
    assert(p105_ymag_GET(pack) == (float)1.4276351E38F);
    assert(p105_ygyro_GET(pack) == (float)5.251862E37F);
    assert(p105_zacc_GET(pack) == (float)2.0557952E37F);
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_time_usec_GET(pack) == (uint64_t)1928384203348573103L);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)1993724259L);
    assert(p106_integrated_y_GET(pack) == (float)1.3874028E38F);
    assert(p106_integrated_ygyro_GET(pack) == (float)4.8436843E37F);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)84);
    assert(p106_integrated_zgyro_GET(pack) == (float) -2.6147208E38F);
    assert(p106_integrated_xgyro_GET(pack) == (float) -3.2413966E38F);
    assert(p106_distance_GET(pack) == (float) -1.2694026E38F);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)249566869L);
    assert(p106_integrated_x_GET(pack) == (float) -1.2229004E38F);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t) -11160);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)111);
};


void c_LoopBackDemoChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_time_usec_GET(pack) == (uint64_t)1156853484184566798L);
    assert(p107_xmag_GET(pack) == (float)1.7001716E38F);
    assert(p107_temperature_GET(pack) == (float) -2.667241E38F);
    assert(p107_ygyro_GET(pack) == (float)8.557706E36F);
    assert(p107_xgyro_GET(pack) == (float)6.67273E37F);
    assert(p107_abs_pressure_GET(pack) == (float)1.9825268E38F);
    assert(p107_zacc_GET(pack) == (float)1.6997747E38F);
    assert(p107_zmag_GET(pack) == (float)1.7889254E38F);
    assert(p107_diff_pressure_GET(pack) == (float) -1.8730957E37F);
    assert(p107_pressure_alt_GET(pack) == (float) -5.099245E37F);
    assert(p107_yacc_GET(pack) == (float)3.1234367E38F);
    assert(p107_zgyro_GET(pack) == (float)2.6672314E38F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)2827801811L);
    assert(p107_xacc_GET(pack) == (float) -4.960933E37F);
    assert(p107_ymag_GET(pack) == (float) -1.911557E37F);
};


void c_LoopBackDemoChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_pitch_GET(pack) == (float) -2.1460931E38F);
    assert(p108_std_dev_vert_GET(pack) == (float)2.7210439E38F);
    assert(p108_q4_GET(pack) == (float)2.5158098E38F);
    assert(p108_lon_GET(pack) == (float) -9.317673E37F);
    assert(p108_zgyro_GET(pack) == (float) -1.4597469E38F);
    assert(p108_vn_GET(pack) == (float)1.8383345E38F);
    assert(p108_std_dev_horz_GET(pack) == (float) -7.6393483E37F);
    assert(p108_lat_GET(pack) == (float)1.4763811E38F);
    assert(p108_ve_GET(pack) == (float) -4.369593E37F);
    assert(p108_q1_GET(pack) == (float) -2.92444E38F);
    assert(p108_roll_GET(pack) == (float) -2.824069E37F);
    assert(p108_q2_GET(pack) == (float) -2.0184752E38F);
    assert(p108_zacc_GET(pack) == (float) -1.0167349E38F);
    assert(p108_yacc_GET(pack) == (float)1.3322755E38F);
    assert(p108_xacc_GET(pack) == (float) -2.5644841E38F);
    assert(p108_ygyro_GET(pack) == (float) -3.7916682E37F);
    assert(p108_alt_GET(pack) == (float)1.2883949E38F);
    assert(p108_vd_GET(pack) == (float)3.1609762E38F);
    assert(p108_xgyro_GET(pack) == (float)7.688713E37F);
    assert(p108_yaw_GET(pack) == (float)4.998108E37F);
    assert(p108_q3_GET(pack) == (float)2.8864575E38F);
};


void c_LoopBackDemoChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)59);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)0);
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)30645);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)41708);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)226);
};


void c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)215);
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)192);
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)55);
    {
        uint8_t exemplary[] =  {(uint8_t)141, (uint8_t)187, (uint8_t)212, (uint8_t)128, (uint8_t)221, (uint8_t)1, (uint8_t)202, (uint8_t)204, (uint8_t)145, (uint8_t)124, (uint8_t)25, (uint8_t)14, (uint8_t)62, (uint8_t)221, (uint8_t)111, (uint8_t)178, (uint8_t)188, (uint8_t)13, (uint8_t)96, (uint8_t)182, (uint8_t)163, (uint8_t)72, (uint8_t)249, (uint8_t)28, (uint8_t)109, (uint8_t)171, (uint8_t)83, (uint8_t)1, (uint8_t)231, (uint8_t)112, (uint8_t)12, (uint8_t)7, (uint8_t)208, (uint8_t)182, (uint8_t)242, (uint8_t)63, (uint8_t)131, (uint8_t)62, (uint8_t)240, (uint8_t)84, (uint8_t)229, (uint8_t)101, (uint8_t)20, (uint8_t)175, (uint8_t)150, (uint8_t)111, (uint8_t)150, (uint8_t)88, (uint8_t)138, (uint8_t)162, (uint8_t)81, (uint8_t)229, (uint8_t)154, (uint8_t)17, (uint8_t)210, (uint8_t)116, (uint8_t)162, (uint8_t)30, (uint8_t)183, (uint8_t)33, (uint8_t)243, (uint8_t)93, (uint8_t)144, (uint8_t)227, (uint8_t)25, (uint8_t)1, (uint8_t)196, (uint8_t)24, (uint8_t)95, (uint8_t)180, (uint8_t)96, (uint8_t)36, (uint8_t)197, (uint8_t)182, (uint8_t)27, (uint8_t)166, (uint8_t)122, (uint8_t)118, (uint8_t)46, (uint8_t)43, (uint8_t)131, (uint8_t)152, (uint8_t)254, (uint8_t)72, (uint8_t)147, (uint8_t)108, (uint8_t)72, (uint8_t)214, (uint8_t)64, (uint8_t)243, (uint8_t)141, (uint8_t)231, (uint8_t)199, (uint8_t)169, (uint8_t)113, (uint8_t)254, (uint8_t)49, (uint8_t)216, (uint8_t)201, (uint8_t)135, (uint8_t)193, (uint8_t)216, (uint8_t)53, (uint8_t)80, (uint8_t)17, (uint8_t)177, (uint8_t)80, (uint8_t)161, (uint8_t)126, (uint8_t)132, (uint8_t)1, (uint8_t)135, (uint8_t)232, (uint8_t)183, (uint8_t)172, (uint8_t)56, (uint8_t)188, (uint8_t)20, (uint8_t)179, (uint8_t)104, (uint8_t)250, (uint8_t)197, (uint8_t)218, (uint8_t)203, (uint8_t)131, (uint8_t)15, (uint8_t)208, (uint8_t)144, (uint8_t)170, (uint8_t)62, (uint8_t)105, (uint8_t)6, (uint8_t)158, (uint8_t)38, (uint8_t)94, (uint8_t)44, (uint8_t)172, (uint8_t)172, (uint8_t)174, (uint8_t)185, (uint8_t)142, (uint8_t)115, (uint8_t)179, (uint8_t)22, (uint8_t)22, (uint8_t)1, (uint8_t)55, (uint8_t)241, (uint8_t)64, (uint8_t)89, (uint8_t)175, (uint8_t)26, (uint8_t)248, (uint8_t)253, (uint8_t)46, (uint8_t)242, (uint8_t)162, (uint8_t)109, (uint8_t)48, (uint8_t)20, (uint8_t)41, (uint8_t)136, (uint8_t)174, (uint8_t)70, (uint8_t)188, (uint8_t)142, (uint8_t)229, (uint8_t)119, (uint8_t)107, (uint8_t)4, (uint8_t)255, (uint8_t)58, (uint8_t)249, (uint8_t)150, (uint8_t)183, (uint8_t)254, (uint8_t)218, (uint8_t)151, (uint8_t)77, (uint8_t)1, (uint8_t)14, (uint8_t)145, (uint8_t)148, (uint8_t)5, (uint8_t)12, (uint8_t)183, (uint8_t)47, (uint8_t)130, (uint8_t)46, (uint8_t)247, (uint8_t)160, (uint8_t)32, (uint8_t)167, (uint8_t)57, (uint8_t)244, (uint8_t)41, (uint8_t)57, (uint8_t)15, (uint8_t)54, (uint8_t)110, (uint8_t)11, (uint8_t)14, (uint8_t)62, (uint8_t)237, (uint8_t)139, (uint8_t)40, (uint8_t)242, (uint8_t)111, (uint8_t)28, (uint8_t)160, (uint8_t)200, (uint8_t)92, (uint8_t)15, (uint8_t)225, (uint8_t)232, (uint8_t)31, (uint8_t)15, (uint8_t)234, (uint8_t)123, (uint8_t)87, (uint8_t)203, (uint8_t)41, (uint8_t)147, (uint8_t)69, (uint8_t)250, (uint8_t)64, (uint8_t)211, (uint8_t)225, (uint8_t)128, (uint8_t)122, (uint8_t)168, (uint8_t)62, (uint8_t)124, (uint8_t)152, (uint8_t)213, (uint8_t)236, (uint8_t)124, (uint8_t)0, (uint8_t)128, (uint8_t)5, (uint8_t)75, (uint8_t)239, (uint8_t)66, (uint8_t)158, (uint8_t)158, (uint8_t)4, (uint8_t)121, (uint8_t)174, (uint8_t)139, (uint8_t)247, (uint8_t)111} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_tc1_GET(pack) == (int64_t)9066531987965081276L);
    assert(p111_ts1_GET(pack) == (int64_t) -6820710341724442267L);
};


void c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_seq_GET(pack) == (uint32_t)349047401L);
    assert(p112_time_usec_GET(pack) == (uint64_t)3443581684130125324L);
};


void c_LoopBackDemoChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_time_usec_GET(pack) == (uint64_t)3392908150088189275L);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)61019);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)216);
    assert(p113_lon_GET(pack) == (int32_t)1751233755);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)13821);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t)2826);
    assert(p113_alt_GET(pack) == (int32_t) -720631257);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)2636);
    assert(p113_lat_GET(pack) == (int32_t)1526854778);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t)16464);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)25159);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t) -18683);
};


void c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_time_usec_GET(pack) == (uint64_t)4528934574512529667L);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t) -12361);
    assert(p114_integrated_xgyro_GET(pack) == (float)2.773049E38F);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)2596061684L);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)864484921L);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)163);
    assert(p114_integrated_ygyro_GET(pack) == (float)1.9288184E38F);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)73);
    assert(p114_integrated_y_GET(pack) == (float)1.0994803E38F);
    assert(p114_integrated_x_GET(pack) == (float) -2.5556063E38F);
    assert(p114_distance_GET(pack) == (float)4.834069E37F);
    assert(p114_integrated_zgyro_GET(pack) == (float)8.473334E37F);
};


void c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_lon_GET(pack) == (int32_t) -792468347);
    assert(p115_pitchspeed_GET(pack) == (float) -2.2947202E38F);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t) -19043);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t) -13655);
    {
        float exemplary[] =  {2.995407E38F, -2.2303807E38F, 3.0797527E38F, 1.942452E38F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_alt_GET(pack) == (int32_t)733224073);
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t) -30444);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t)5883);
    assert(p115_lat_GET(pack) == (int32_t) -1045094055);
    assert(p115_time_usec_GET(pack) == (uint64_t)1976304181846714897L);
    assert(p115_rollspeed_GET(pack) == (float)7.416648E37F);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t)367);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t)32747);
    assert(p115_yawspeed_GET(pack) == (float) -2.086654E38F);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)9970);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)46693);
};


void c_LoopBackDemoChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t) -8966);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t) -32006);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t) -29599);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t)28847);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)3183606508L);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t)25822);
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t)29657);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t)23735);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t) -5593);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t)43);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)29780);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)186);
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)43935);
};


void c_LoopBackDemoChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)31181);
    assert(p118_time_utc_GET(pack) == (uint32_t)2891848756L);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)17143);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)15658);
    assert(p118_size_GET(pack) == (uint32_t)1169602637L);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)7410);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)221);
    assert(p119_ofs_GET(pack) == (uint32_t)2027232737L);
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)154);
    assert(p119_count_GET(pack) == (uint32_t)4094678952L);
};


void c_LoopBackDemoChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)9325);
    {
        uint8_t exemplary[] =  {(uint8_t)60, (uint8_t)40, (uint8_t)240, (uint8_t)50, (uint8_t)17, (uint8_t)98, (uint8_t)97, (uint8_t)62, (uint8_t)124, (uint8_t)52, (uint8_t)112, (uint8_t)110, (uint8_t)20, (uint8_t)92, (uint8_t)199, (uint8_t)230, (uint8_t)229, (uint8_t)243, (uint8_t)173, (uint8_t)232, (uint8_t)40, (uint8_t)27, (uint8_t)254, (uint8_t)81, (uint8_t)36, (uint8_t)170, (uint8_t)150, (uint8_t)53, (uint8_t)75, (uint8_t)3, (uint8_t)199, (uint8_t)139, (uint8_t)66, (uint8_t)74, (uint8_t)134, (uint8_t)69, (uint8_t)87, (uint8_t)114, (uint8_t)226, (uint8_t)202, (uint8_t)228, (uint8_t)8, (uint8_t)43, (uint8_t)34, (uint8_t)192, (uint8_t)135, (uint8_t)35, (uint8_t)54, (uint8_t)161, (uint8_t)245, (uint8_t)52, (uint8_t)43, (uint8_t)74, (uint8_t)134, (uint8_t)196, (uint8_t)29, (uint8_t)10, (uint8_t)144, (uint8_t)139, (uint8_t)195, (uint8_t)168, (uint8_t)182, (uint8_t)251, (uint8_t)103, (uint8_t)59, (uint8_t)81, (uint8_t)162, (uint8_t)205, (uint8_t)109, (uint8_t)139, (uint8_t)110, (uint8_t)28, (uint8_t)242, (uint8_t)127, (uint8_t)239, (uint8_t)200, (uint8_t)134, (uint8_t)23, (uint8_t)125, (uint8_t)25, (uint8_t)130, (uint8_t)27, (uint8_t)32, (uint8_t)206, (uint8_t)14, (uint8_t)63, (uint8_t)76, (uint8_t)175, (uint8_t)65, (uint8_t)27} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p120_ofs_GET(pack) == (uint32_t)2310129942L);
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)174);
};


void c_LoopBackDemoChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)138);
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)183);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)54);
};


void c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)244);
    {
        uint8_t exemplary[] =  {(uint8_t)216, (uint8_t)206, (uint8_t)244, (uint8_t)12, (uint8_t)168, (uint8_t)165, (uint8_t)191, (uint8_t)210, (uint8_t)162, (uint8_t)251, (uint8_t)140, (uint8_t)192, (uint8_t)63, (uint8_t)182, (uint8_t)74, (uint8_t)193, (uint8_t)83, (uint8_t)210, (uint8_t)111, (uint8_t)164, (uint8_t)111, (uint8_t)26, (uint8_t)68, (uint8_t)8, (uint8_t)44, (uint8_t)72, (uint8_t)237, (uint8_t)144, (uint8_t)167, (uint8_t)63, (uint8_t)227, (uint8_t)35, (uint8_t)219, (uint8_t)127, (uint8_t)151, (uint8_t)189, (uint8_t)96, (uint8_t)171, (uint8_t)229, (uint8_t)83, (uint8_t)228, (uint8_t)220, (uint8_t)130, (uint8_t)137, (uint8_t)20, (uint8_t)141, (uint8_t)167, (uint8_t)107, (uint8_t)20, (uint8_t)103, (uint8_t)29, (uint8_t)115, (uint8_t)102, (uint8_t)156, (uint8_t)44, (uint8_t)88, (uint8_t)236, (uint8_t)235, (uint8_t)204, (uint8_t)226, (uint8_t)212, (uint8_t)85, (uint8_t)47, (uint8_t)22, (uint8_t)43, (uint8_t)100, (uint8_t)153, (uint8_t)29, (uint8_t)15, (uint8_t)10, (uint8_t)185, (uint8_t)83, (uint8_t)239, (uint8_t)82, (uint8_t)75, (uint8_t)93, (uint8_t)188, (uint8_t)144, (uint8_t)128, (uint8_t)86, (uint8_t)92, (uint8_t)240, (uint8_t)72, (uint8_t)68, (uint8_t)72, (uint8_t)148, (uint8_t)149, (uint8_t)45, (uint8_t)97, (uint8_t)14, (uint8_t)21, (uint8_t)166, (uint8_t)202, (uint8_t)255, (uint8_t)31, (uint8_t)3, (uint8_t)10, (uint8_t)133, (uint8_t)227, (uint8_t)238, (uint8_t)188, (uint8_t)236, (uint8_t)83, (uint8_t)133, (uint8_t)54, (uint8_t)224, (uint8_t)152, (uint8_t)247, (uint8_t)187, (uint8_t)95} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)71);
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)131);
};


void c_LoopBackDemoChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_lat_GET(pack) == (int32_t) -329473392);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)63793);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)21396);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)5258);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_3D_FIX);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)42416);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)35);
    assert(p124_time_usec_GET(pack) == (uint64_t)6145796054812901367L);
    assert(p124_dgps_age_GET(pack) == (uint32_t)3375734272L);
    assert(p124_alt_GET(pack) == (int32_t) -101596466);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p124_lon_GET(pack) == (int32_t) -1170909397);
};


void c_LoopBackDemoChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)52771);
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)65319);
    assert(p125_flags_GET(pack) == e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT);
};


void c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)142);
    assert(p126_baudrate_GET(pack) == (uint32_t)932605462L);
    {
        uint8_t exemplary[] =  {(uint8_t)136, (uint8_t)12, (uint8_t)147, (uint8_t)249, (uint8_t)6, (uint8_t)200, (uint8_t)7, (uint8_t)2, (uint8_t)15, (uint8_t)106, (uint8_t)242, (uint8_t)3, (uint8_t)18, (uint8_t)92, (uint8_t)60, (uint8_t)123, (uint8_t)56, (uint8_t)221, (uint8_t)165, (uint8_t)114, (uint8_t)62, (uint8_t)58, (uint8_t)226, (uint8_t)141, (uint8_t)253, (uint8_t)125, (uint8_t)9, (uint8_t)39, (uint8_t)245, (uint8_t)97, (uint8_t)50, (uint8_t)225, (uint8_t)26, (uint8_t)22, (uint8_t)128, (uint8_t)54, (uint8_t)72, (uint8_t)41, (uint8_t)193, (uint8_t)78, (uint8_t)80, (uint8_t)48, (uint8_t)19, (uint8_t)190, (uint8_t)42, (uint8_t)206, (uint8_t)169, (uint8_t)33, (uint8_t)83, (uint8_t)51, (uint8_t)110, (uint8_t)228, (uint8_t)158, (uint8_t)1, (uint8_t)210, (uint8_t)222, (uint8_t)129, (uint8_t)128, (uint8_t)82, (uint8_t)51, (uint8_t)149, (uint8_t)199, (uint8_t)174, (uint8_t)135, (uint8_t)41, (uint8_t)157, (uint8_t)129, (uint8_t)6, (uint8_t)200, (uint8_t)5} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2);
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)34380);
    assert(p126_flags_GET(pack) == e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY);
};


void c_LoopBackDemoChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)63);
    assert(p127_accuracy_GET(pack) == (uint32_t)2478974838L);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t)2109668784);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)4172940816L);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t)2025317357);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)195);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)54723);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)134);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t)617901110);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)52);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)232);
    assert(p127_tow_GET(pack) == (uint32_t)3135574598L);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t)1013415676);
};


void c_LoopBackDemoChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_baseline_b_mm_GET(pack) == (int32_t) -385074998);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)2384543303L);
    assert(p128_tow_GET(pack) == (uint32_t)804055881L);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t) -19308133);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)7);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)30376);
    assert(p128_accuracy_GET(pack) == (uint32_t)2703771080L);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)16);
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)244);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t) -597040842);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t)2074452508);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)35);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)52);
};


void c_LoopBackDemoChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t) -12880);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t)32532);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)4145135430L);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t) -18826);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t) -14533);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t) -16214);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t)31048);
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t) -19394);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t)10783);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t)20180);
};


void c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)21);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)44264);
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)12779);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)221);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)41126);
    assert(p130_size_GET(pack) == (uint32_t)3028665782L);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)254);
};


void c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)110, (uint8_t)35, (uint8_t)170, (uint8_t)90, (uint8_t)80, (uint8_t)244, (uint8_t)177, (uint8_t)164, (uint8_t)117, (uint8_t)29, (uint8_t)122, (uint8_t)244, (uint8_t)138, (uint8_t)34, (uint8_t)177, (uint8_t)159, (uint8_t)253, (uint8_t)87, (uint8_t)182, (uint8_t)134, (uint8_t)11, (uint8_t)228, (uint8_t)147, (uint8_t)171, (uint8_t)219, (uint8_t)223, (uint8_t)101, (uint8_t)141, (uint8_t)253, (uint8_t)42, (uint8_t)12, (uint8_t)109, (uint8_t)5, (uint8_t)137, (uint8_t)24, (uint8_t)70, (uint8_t)148, (uint8_t)63, (uint8_t)149, (uint8_t)106, (uint8_t)245, (uint8_t)156, (uint8_t)13, (uint8_t)205, (uint8_t)146, (uint8_t)149, (uint8_t)184, (uint8_t)26, (uint8_t)73, (uint8_t)226, (uint8_t)98, (uint8_t)136, (uint8_t)149, (uint8_t)229, (uint8_t)248, (uint8_t)198, (uint8_t)185, (uint8_t)71, (uint8_t)233, (uint8_t)242, (uint8_t)119, (uint8_t)47, (uint8_t)233, (uint8_t)226, (uint8_t)231, (uint8_t)55, (uint8_t)79, (uint8_t)1, (uint8_t)248, (uint8_t)148, (uint8_t)89, (uint8_t)86, (uint8_t)80, (uint8_t)131, (uint8_t)248, (uint8_t)227, (uint8_t)191, (uint8_t)114, (uint8_t)102, (uint8_t)118, (uint8_t)144, (uint8_t)227, (uint8_t)104, (uint8_t)100, (uint8_t)250, (uint8_t)197, (uint8_t)23, (uint8_t)61, (uint8_t)130, (uint8_t)103, (uint8_t)218, (uint8_t)49, (uint8_t)49, (uint8_t)45, (uint8_t)2, (uint8_t)220, (uint8_t)81, (uint8_t)28, (uint8_t)190, (uint8_t)168, (uint8_t)126, (uint8_t)127, (uint8_t)51, (uint8_t)9, (uint8_t)246, (uint8_t)4, (uint8_t)52, (uint8_t)187, (uint8_t)174, (uint8_t)191, (uint8_t)205, (uint8_t)207, (uint8_t)251, (uint8_t)255, (uint8_t)39, (uint8_t)253, (uint8_t)142, (uint8_t)156, (uint8_t)219, (uint8_t)85, (uint8_t)60, (uint8_t)102, (uint8_t)123, (uint8_t)58, (uint8_t)11, (uint8_t)176, (uint8_t)50, (uint8_t)185, (uint8_t)101, (uint8_t)16, (uint8_t)208, (uint8_t)186, (uint8_t)214, (uint8_t)243, (uint8_t)163, (uint8_t)187, (uint8_t)171, (uint8_t)1, (uint8_t)115, (uint8_t)166, (uint8_t)239, (uint8_t)229, (uint8_t)41, (uint8_t)127, (uint8_t)203, (uint8_t)183, (uint8_t)50, (uint8_t)149, (uint8_t)36, (uint8_t)4, (uint8_t)21, (uint8_t)17, (uint8_t)150, (uint8_t)251, (uint8_t)113, (uint8_t)33, (uint8_t)231, (uint8_t)129, (uint8_t)166, (uint8_t)39, (uint8_t)52, (uint8_t)217, (uint8_t)167, (uint8_t)193, (uint8_t)25, (uint8_t)208, (uint8_t)85, (uint8_t)149, (uint8_t)196, (uint8_t)122, (uint8_t)59, (uint8_t)95, (uint8_t)63, (uint8_t)243, (uint8_t)145, (uint8_t)167, (uint8_t)153, (uint8_t)104, (uint8_t)203, (uint8_t)39, (uint8_t)117, (uint8_t)190, (uint8_t)111, (uint8_t)246, (uint8_t)105, (uint8_t)132, (uint8_t)111, (uint8_t)82, (uint8_t)5, (uint8_t)137, (uint8_t)171, (uint8_t)27, (uint8_t)54, (uint8_t)158, (uint8_t)8, (uint8_t)60, (uint8_t)51, (uint8_t)210, (uint8_t)209, (uint8_t)138, (uint8_t)202, (uint8_t)56, (uint8_t)3, (uint8_t)155, (uint8_t)203, (uint8_t)24, (uint8_t)156, (uint8_t)110, (uint8_t)190, (uint8_t)63, (uint8_t)70, (uint8_t)70, (uint8_t)224, (uint8_t)213, (uint8_t)250, (uint8_t)49, (uint8_t)194, (uint8_t)140, (uint8_t)155, (uint8_t)114, (uint8_t)78, (uint8_t)27, (uint8_t)5, (uint8_t)66, (uint8_t)198, (uint8_t)246, (uint8_t)162, (uint8_t)127, (uint8_t)82, (uint8_t)228, (uint8_t)110, (uint8_t)157, (uint8_t)44, (uint8_t)202, (uint8_t)180, (uint8_t)143, (uint8_t)72, (uint8_t)162, (uint8_t)99, (uint8_t)97, (uint8_t)239, (uint8_t)164, (uint8_t)161, (uint8_t)206, (uint8_t)42, (uint8_t)224, (uint8_t)188, (uint8_t)98, (uint8_t)157, (uint8_t)185, (uint8_t)79, (uint8_t)227, (uint8_t)163} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)21163);
};


void c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)65504960L);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_315);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)5755);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)225);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)34121);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)21164);
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR);
};


void c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_lat_GET(pack) == (int32_t) -656045646);
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)21479);
    assert(p133_lon_GET(pack) == (int32_t)33139079);
    assert(p133_mask_GET(pack) == (uint64_t)4753543093664333295L);
};


void c_LoopBackDemoChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    assert(p134_lat_GET(pack) == (int32_t) -678615857);
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)18492);
    assert(p134_lon_GET(pack) == (int32_t) -763027854);
    {
        int16_t exemplary[] =  {(int16_t)9324, (int16_t)17822, (int16_t)28338, (int16_t)23813, (int16_t) -18382, (int16_t)27800, (int16_t)31417, (int16_t)25516, (int16_t) -9585, (int16_t) -29775, (int16_t) -2033, (int16_t)9108, (int16_t)28559, (int16_t) -29421, (int16_t)32112, (int16_t)13993} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lat_GET(pack) == (int32_t)1918579710);
    assert(p135_lon_GET(pack) == (int32_t)1412630928);
};


void c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_lat_GET(pack) == (int32_t) -99325165);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)36187);
    assert(p136_terrain_height_GET(pack) == (float) -2.2104123E38F);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)16403);
    assert(p136_current_height_GET(pack) == (float) -2.9392735E38F);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)52859);
    assert(p136_lon_GET(pack) == (int32_t)1151929247);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)323508292L);
    assert(p137_press_diff_GET(pack) == (float) -1.6043337E38F);
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t) -11297);
    assert(p137_press_abs_GET(pack) == (float) -2.1846997E38F);
};


void c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {4.150003E37F, 1.4922869E38F, -9.276017E37F, -2.6111317E38F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_x_GET(pack) == (float) -2.6874846E38F);
    assert(p138_y_GET(pack) == (float) -8.58887E37F);
    assert(p138_z_GET(pack) == (float)1.9977656E38F);
    assert(p138_time_usec_GET(pack) == (uint64_t)3595484720603252898L);
};


void c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)7);
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p139_time_usec_GET(pack) == (uint64_t)5316800483687197671L);
    {
        float exemplary[] =  {-3.7707715E37F, 3.9901975E37F, -1.3565633E38F, -9.989574E37F, -2.9452225E38F, -2.1560366E38F, 6.420881E36F, 8.3435383E37F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)159);
};


void c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-3.8085885E37F, -1.6549493E38F, 1.6059674E38F, -2.7815518E38F, 2.563886E38F, -2.450936E38F, -3.979575E36F, 3.3761543E38F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p140_time_usec_GET(pack) == (uint64_t)7630842519899568613L);
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)170);
};


void c_LoopBackDemoChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_altitude_terrain_GET(pack) == (float) -1.9743274E35F);
    assert(p141_time_usec_GET(pack) == (uint64_t)1642444103785961245L);
    assert(p141_altitude_amsl_GET(pack) == (float)2.091532E38F);
    assert(p141_altitude_relative_GET(pack) == (float) -9.699802E37F);
    assert(p141_altitude_monotonic_GET(pack) == (float) -2.6678275E38F);
    assert(p141_bottom_clearance_GET(pack) == (float)1.1445459E38F);
    assert(p141_altitude_local_GET(pack) == (float) -9.600342E37F);
};


void c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)215, (uint8_t)2, (uint8_t)9, (uint8_t)22, (uint8_t)21, (uint8_t)60, (uint8_t)52, (uint8_t)121, (uint8_t)40, (uint8_t)255, (uint8_t)149, (uint8_t)66, (uint8_t)223, (uint8_t)140, (uint8_t)156, (uint8_t)52, (uint8_t)173, (uint8_t)49, (uint8_t)106, (uint8_t)180, (uint8_t)185, (uint8_t)147, (uint8_t)61, (uint8_t)167, (uint8_t)153, (uint8_t)193, (uint8_t)35, (uint8_t)147, (uint8_t)159, (uint8_t)21, (uint8_t)234, (uint8_t)222, (uint8_t)243, (uint8_t)71, (uint8_t)181, (uint8_t)136, (uint8_t)30, (uint8_t)36, (uint8_t)152, (uint8_t)68, (uint8_t)193, (uint8_t)187, (uint8_t)129, (uint8_t)201, (uint8_t)91, (uint8_t)121, (uint8_t)88, (uint8_t)22, (uint8_t)21, (uint8_t)182, (uint8_t)212, (uint8_t)226, (uint8_t)7, (uint8_t)12, (uint8_t)148, (uint8_t)74, (uint8_t)140, (uint8_t)120, (uint8_t)41, (uint8_t)78, (uint8_t)111, (uint8_t)23, (uint8_t)166, (uint8_t)173, (uint8_t)146, (uint8_t)90, (uint8_t)59, (uint8_t)235, (uint8_t)208, (uint8_t)203, (uint8_t)74, (uint8_t)117, (uint8_t)19, (uint8_t)218, (uint8_t)57, (uint8_t)184, (uint8_t)207, (uint8_t)69, (uint8_t)102, (uint8_t)173, (uint8_t)190, (uint8_t)17, (uint8_t)44, (uint8_t)245, (uint8_t)19, (uint8_t)252, (uint8_t)224, (uint8_t)65, (uint8_t)99, (uint8_t)131, (uint8_t)84, (uint8_t)184, (uint8_t)215, (uint8_t)58, (uint8_t)196, (uint8_t)197, (uint8_t)79, (uint8_t)120, (uint8_t)169, (uint8_t)232, (uint8_t)62, (uint8_t)37, (uint8_t)208, (uint8_t)248, (uint8_t)176, (uint8_t)35, (uint8_t)93, (uint8_t)165, (uint8_t)88, (uint8_t)104, (uint8_t)207, (uint8_t)7, (uint8_t)173, (uint8_t)241, (uint8_t)22, (uint8_t)7, (uint8_t)188, (uint8_t)91, (uint8_t)193, (uint8_t)180} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)211);
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)129);
    {
        uint8_t exemplary[] =  {(uint8_t)56, (uint8_t)70, (uint8_t)112, (uint8_t)201, (uint8_t)65, (uint8_t)56, (uint8_t)63, (uint8_t)246, (uint8_t)123, (uint8_t)109, (uint8_t)240, (uint8_t)125, (uint8_t)214, (uint8_t)128, (uint8_t)90, (uint8_t)168, (uint8_t)147, (uint8_t)115, (uint8_t)192, (uint8_t)241, (uint8_t)89, (uint8_t)237, (uint8_t)102, (uint8_t)236, (uint8_t)173, (uint8_t)106, (uint8_t)97, (uint8_t)124, (uint8_t)190, (uint8_t)204, (uint8_t)10, (uint8_t)143, (uint8_t)131, (uint8_t)66, (uint8_t)27, (uint8_t)27, (uint8_t)241, (uint8_t)18, (uint8_t)219, (uint8_t)143, (uint8_t)73, (uint8_t)85, (uint8_t)2, (uint8_t)238, (uint8_t)208, (uint8_t)14, (uint8_t)46, (uint8_t)34, (uint8_t)29, (uint8_t)11, (uint8_t)22, (uint8_t)93, (uint8_t)56, (uint8_t)228, (uint8_t)126, (uint8_t)178, (uint8_t)98, (uint8_t)24, (uint8_t)4, (uint8_t)144, (uint8_t)52, (uint8_t)56, (uint8_t)213, (uint8_t)231, (uint8_t)73, (uint8_t)165, (uint8_t)159, (uint8_t)254, (uint8_t)111, (uint8_t)194, (uint8_t)224, (uint8_t)183, (uint8_t)74, (uint8_t)19, (uint8_t)114, (uint8_t)142, (uint8_t)6, (uint8_t)32, (uint8_t)104, (uint8_t)218, (uint8_t)131, (uint8_t)70, (uint8_t)146, (uint8_t)254, (uint8_t)108, (uint8_t)185, (uint8_t)154, (uint8_t)196, (uint8_t)223, (uint8_t)231, (uint8_t)179, (uint8_t)198, (uint8_t)79, (uint8_t)255, (uint8_t)223, (uint8_t)146, (uint8_t)214, (uint8_t)65, (uint8_t)174, (uint8_t)250, (uint8_t)245, (uint8_t)61, (uint8_t)224, (uint8_t)72, (uint8_t)84, (uint8_t)121, (uint8_t)33, (uint8_t)60, (uint8_t)134, (uint8_t)122, (uint8_t)57, (uint8_t)116, (uint8_t)121, (uint8_t)10, (uint8_t)52, (uint8_t)84, (uint8_t)212, (uint8_t)71, (uint8_t)53, (uint8_t)22} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)134);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t)14364);
    assert(p143_press_diff_GET(pack) == (float)2.4009726E38F);
    assert(p143_press_abs_GET(pack) == (float) -6.8153353E37F);
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)2498362883L);
};


void c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)191);
    assert(p144_timestamp_GET(pack) == (uint64_t)8547907477973938179L);
    {
        float exemplary[] =  {-3.121019E38F, -1.771192E38F, 2.2397447E38F, 3.2741663E38F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_custom_state_GET(pack) == (uint64_t)8877216893280063939L);
    assert(p144_alt_GET(pack) == (float) -2.281812E38F);
    {
        float exemplary[] =  {3.3749885E38F, 2.458964E38F, 8.891448E37F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-5.2713653E37F, -2.4599967E38F, -7.9903034E37F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-1.8304023E38F, -2.5965695E38F, 5.3772587E37F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-4.25672E37F, 2.2579036E38F, 2.831111E38F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lat_GET(pack) == (int32_t)493012254);
    assert(p144_lon_GET(pack) == (int32_t)983143444);
};


void c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_x_vel_GET(pack) == (float) -1.453971E38F);
    assert(p146_x_acc_GET(pack) == (float) -2.7363747E38F);
    {
        float exemplary[] =  {-7.804859E37F, -2.4251207E38F, -2.0166995E38F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_time_usec_GET(pack) == (uint64_t)8100699072674749972L);
    assert(p146_x_pos_GET(pack) == (float)1.4072873E38F);
    assert(p146_y_vel_GET(pack) == (float)1.1573261E38F);
    assert(p146_yaw_rate_GET(pack) == (float)2.010907E38F);
    assert(p146_roll_rate_GET(pack) == (float)6.859886E37F);
    assert(p146_z_pos_GET(pack) == (float) -8.363346E37F);
    assert(p146_z_vel_GET(pack) == (float)5.4560453E37F);
    {
        float exemplary[] =  {-1.4651292E38F, 1.5273652E38F, 3.831537E37F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-3.382749E38F, -2.0699702E38F, -3.4076945E36F, 2.9086022E38F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_airspeed_GET(pack) == (float) -1.205771E38F);
    assert(p146_pitch_rate_GET(pack) == (float)2.9532132E38F);
    assert(p146_y_acc_GET(pack) == (float)2.435616E38F);
    assert(p146_y_pos_GET(pack) == (float) -1.6415939E38F);
    assert(p146_z_acc_GET(pack) == (float) -5.33867E37F);
};


void c_LoopBackDemoChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)77);
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t) -18136);
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LION);
    assert(p147_energy_consumed_GET(pack) == (int32_t)921662843);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t)97);
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t) -17113);
    {
        uint16_t exemplary[] =  {(uint16_t)53762, (uint16_t)8505, (uint16_t)18593, (uint16_t)53910, (uint16_t)26292, (uint16_t)51565, (uint16_t)42924, (uint16_t)39094, (uint16_t)45958, (uint16_t)51872} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_UNKNOWN);
    assert(p147_current_consumed_GET(pack) == (int32_t) -980876098);
};


void c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    assert(p148_os_sw_version_GET(pack) == (uint32_t)3780009964L);
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)22012);
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)59339);
    {
        uint8_t exemplary[] =  {(uint8_t)143, (uint8_t)101, (uint8_t)132, (uint8_t)171, (uint8_t)12, (uint8_t)32, (uint8_t)191, (uint8_t)55} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)238, (uint8_t)173, (uint8_t)171, (uint8_t)41, (uint8_t)239, (uint8_t)21, (uint8_t)37, (uint8_t)135} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_capabilities_GET(pack) == e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FENCE);
    assert(p148_uid_GET(pack) == (uint64_t)7747904855314507675L);
    assert(p148_board_version_GET(pack) == (uint32_t)1223154979L);
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)3453251130L);
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)2806186920L);
    {
        uint8_t exemplary[] =  {(uint8_t)27, (uint8_t)209, (uint8_t)7, (uint8_t)30, (uint8_t)144, (uint8_t)245, (uint8_t)116, (uint8_t)220, (uint8_t)253, (uint8_t)236, (uint8_t)187, (uint8_t)144, (uint8_t)231, (uint8_t)18, (uint8_t)152, (uint8_t)241, (uint8_t)75, (uint8_t)230} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)139, (uint8_t)225, (uint8_t)30, (uint8_t)213, (uint8_t)224, (uint8_t)229, (uint8_t)178, (uint8_t)38} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER);
    {
        float exemplary[] =  {-1.3150781E38F, -1.4892152E38F, -7.5789924E37F, -2.2410807E38F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_size_y_GET(pack) == (float)1.4547794E36F);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)22);
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)112);
    assert(p149_time_usec_GET(pack) == (uint64_t)7802513781881485484L);
    assert(p149_y_TRY(ph) == (float) -2.2123766E38F);
    assert(p149_angle_x_GET(pack) == (float) -2.2618947E38F);
    assert(p149_angle_y_GET(pack) == (float)1.5788509E38F);
    assert(p149_size_x_GET(pack) == (float) -1.8292344E38F);
    assert(p149_x_TRY(ph) == (float)1.4784523E38F);
    assert(p149_z_TRY(ph) == (float) -1.1381937E38F);
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_INT);
    assert(p149_distance_GET(pack) == (float)3.2397742E38F);
};


void c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_time_usec_GET(pack) == (uint64_t)7123351080914805667L);
    assert(p230_flags_GET(pack) == e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL);
    assert(p230_tas_ratio_GET(pack) == (float)1.0647877E38F);
    assert(p230_vel_ratio_GET(pack) == (float) -8.664683E37F);
    assert(p230_pos_vert_accuracy_GET(pack) == (float) -3.0477509E38F);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float)3.2518238E38F);
    assert(p230_pos_horiz_ratio_GET(pack) == (float)1.8074436E38F);
    assert(p230_hagl_ratio_GET(pack) == (float)1.6442782E38F);
    assert(p230_pos_vert_ratio_GET(pack) == (float)5.99704E37F);
    assert(p230_mag_ratio_GET(pack) == (float)3.237829E38F);
};


void c_LoopBackDemoChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_wind_y_GET(pack) == (float)1.2343324E38F);
    assert(p231_vert_accuracy_GET(pack) == (float)2.3143706E38F);
    assert(p231_time_usec_GET(pack) == (uint64_t)4549712318110222512L);
    assert(p231_var_vert_GET(pack) == (float) -6.2388225E37F);
    assert(p231_var_horiz_GET(pack) == (float)5.3677574E37F);
    assert(p231_wind_x_GET(pack) == (float) -1.8396141E38F);
    assert(p231_wind_alt_GET(pack) == (float)3.0124502E38F);
    assert(p231_wind_z_GET(pack) == (float)8.999614E37F);
    assert(p231_horiz_accuracy_GET(pack) == (float) -3.014653E38F);
};


void c_LoopBackDemoChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)51);
    assert(p232_lon_GET(pack) == (int32_t)1607196787);
    assert(p232_vn_GET(pack) == (float)3.2158306E38F);
    assert(p232_vdop_GET(pack) == (float)1.5408743E38F);
    assert(p232_lat_GET(pack) == (int32_t) -572434130);
    assert(p232_time_usec_GET(pack) == (uint64_t)2143957542994889076L);
    assert(p232_ve_GET(pack) == (float) -1.0505313E38F);
    assert(p232_vert_accuracy_GET(pack) == (float)3.253345E38F);
    assert(p232_hdop_GET(pack) == (float) -2.9820492E37F);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)215);
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)43401);
    assert(p232_alt_GET(pack) == (float) -2.4569649E38F);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)1934282804L);
    assert(p232_horiz_accuracy_GET(pack) == (float) -2.1763E38F);
    assert(p232_vd_GET(pack) == (float) -9.500356E37F);
    assert(p232_ignore_flags_GET(pack) == e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT);
    assert(p232_speed_accuracy_GET(pack) == (float) -1.0689536E37F);
};


void c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)51, (uint8_t)7, (uint8_t)118, (uint8_t)43, (uint8_t)236, (uint8_t)160, (uint8_t)212, (uint8_t)41, (uint8_t)117, (uint8_t)236, (uint8_t)88, (uint8_t)171, (uint8_t)46, (uint8_t)220, (uint8_t)32, (uint8_t)70, (uint8_t)104, (uint8_t)182, (uint8_t)91, (uint8_t)210, (uint8_t)248, (uint8_t)55, (uint8_t)109, (uint8_t)89, (uint8_t)224, (uint8_t)237, (uint8_t)27, (uint8_t)49, (uint8_t)167, (uint8_t)237, (uint8_t)173, (uint8_t)56, (uint8_t)30, (uint8_t)221, (uint8_t)217, (uint8_t)253, (uint8_t)21, (uint8_t)9, (uint8_t)227, (uint8_t)247, (uint8_t)248, (uint8_t)80, (uint8_t)254, (uint8_t)78, (uint8_t)55, (uint8_t)207, (uint8_t)61, (uint8_t)30, (uint8_t)101, (uint8_t)136, (uint8_t)15, (uint8_t)53, (uint8_t)252, (uint8_t)221, (uint8_t)238, (uint8_t)85, (uint8_t)232, (uint8_t)190, (uint8_t)189, (uint8_t)137, (uint8_t)4, (uint8_t)83, (uint8_t)134, (uint8_t)201, (uint8_t)232, (uint8_t)169, (uint8_t)243, (uint8_t)166, (uint8_t)166, (uint8_t)239, (uint8_t)154, (uint8_t)53, (uint8_t)156, (uint8_t)53, (uint8_t)0, (uint8_t)83, (uint8_t)136, (uint8_t)157, (uint8_t)42, (uint8_t)140, (uint8_t)216, (uint8_t)115, (uint8_t)47, (uint8_t)195, (uint8_t)218, (uint8_t)152, (uint8_t)188, (uint8_t)74, (uint8_t)131, (uint8_t)8, (uint8_t)161, (uint8_t)161, (uint8_t)124, (uint8_t)0, (uint8_t)142, (uint8_t)198, (uint8_t)89, (uint8_t)63, (uint8_t)171, (uint8_t)153, (uint8_t)232, (uint8_t)112, (uint8_t)96, (uint8_t)41, (uint8_t)137, (uint8_t)182, (uint8_t)186, (uint8_t)212, (uint8_t)163, (uint8_t)90, (uint8_t)184, (uint8_t)89, (uint8_t)107, (uint8_t)92, (uint8_t)120, (uint8_t)41, (uint8_t)134, (uint8_t)214, (uint8_t)199, (uint8_t)81, (uint8_t)22, (uint8_t)41, (uint8_t)234, (uint8_t)95, (uint8_t)15, (uint8_t)91, (uint8_t)92, (uint8_t)1, (uint8_t)166, (uint8_t)125, (uint8_t)103, (uint8_t)113, (uint8_t)17, (uint8_t)214, (uint8_t)167, (uint8_t)175, (uint8_t)14, (uint8_t)222, (uint8_t)127, (uint8_t)91, (uint8_t)129, (uint8_t)243, (uint8_t)126, (uint8_t)12, (uint8_t)95, (uint8_t)224, (uint8_t)125, (uint8_t)47, (uint8_t)19, (uint8_t)165, (uint8_t)66, (uint8_t)253, (uint8_t)142, (uint8_t)161, (uint8_t)142, (uint8_t)239, (uint8_t)83, (uint8_t)159, (uint8_t)208, (uint8_t)133, (uint8_t)218, (uint8_t)47, (uint8_t)93, (uint8_t)185, (uint8_t)144, (uint8_t)192, (uint8_t)248, (uint8_t)181, (uint8_t)210, (uint8_t)87, (uint8_t)162, (uint8_t)159, (uint8_t)229, (uint8_t)74, (uint8_t)117, (uint8_t)142, (uint8_t)157, (uint8_t)232, (uint8_t)188, (uint8_t)240} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)10);
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)44);
};


void c_LoopBackDemoChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t)49);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)21);
    assert(p234_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED);
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t)21826);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)4853);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)247);
    assert(p234_latitude_GET(pack) == (int32_t)1163153918);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)42997);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)107);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t)47);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t)25481);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t)30152);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)188);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t)121);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t)116);
    assert(p234_custom_mode_GET(pack) == (uint32_t)3977809549L);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t) -3750);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)35);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t)32346);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p234_longitude_GET(pack) == (int32_t) -538188357);
};


void c_LoopBackDemoChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_clipping_1_GET(pack) == (uint32_t)125022799L);
    assert(p241_clipping_0_GET(pack) == (uint32_t)2482580071L);
    assert(p241_time_usec_GET(pack) == (uint64_t)1000070213894718602L);
    assert(p241_clipping_2_GET(pack) == (uint32_t)2529418196L);
    assert(p241_vibration_y_GET(pack) == (float) -1.7384765E38F);
    assert(p241_vibration_z_GET(pack) == (float)2.7602272E38F);
    assert(p241_vibration_x_GET(pack) == (float)7.563208E37F);
};


void c_LoopBackDemoChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_x_GET(pack) == (float)4.044369E37F);
    assert(p242_y_GET(pack) == (float) -2.6022922E38F);
    {
        float exemplary[] =  {1.676231E38F, 3.3991467E38F, 3.4882215E37F, -2.4813502E38F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_longitude_GET(pack) == (int32_t)501816747);
    assert(p242_approach_z_GET(pack) == (float)1.8496775E36F);
    assert(p242_approach_y_GET(pack) == (float)1.7044428E38F);
    assert(p242_approach_x_GET(pack) == (float)2.7410028E38F);
    assert(p242_altitude_GET(pack) == (int32_t) -841105729);
    assert(p242_latitude_GET(pack) == (int32_t)502866197);
    assert(p242_z_GET(pack) == (float) -2.3520833E38F);
    assert(p242_time_usec_TRY(ph) == (uint64_t)5967379375408375287L);
};


void c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_altitude_GET(pack) == (int32_t) -1643343410);
    assert(p243_latitude_GET(pack) == (int32_t) -1299964614);
    {
        float exemplary[] =  {3.2708485E38F, 3.1246281E38F, -1.7607592E38F, -2.2653736E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_time_usec_TRY(ph) == (uint64_t)5858762395317330072L);
    assert(p243_x_GET(pack) == (float) -3.253873E38F);
    assert(p243_y_GET(pack) == (float) -2.6454176E38F);
    assert(p243_longitude_GET(pack) == (int32_t) -1584831148);
    assert(p243_approach_y_GET(pack) == (float) -3.0426993E38F);
    assert(p243_z_GET(pack) == (float) -1.4541539E38F);
    assert(p243_approach_z_GET(pack) == (float) -1.1589811E37F);
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p243_approach_x_GET(pack) == (float)1.2117527E38F);
};


void c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_interval_us_GET(pack) == (int32_t) -1858382721);
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)23189);
};


void c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW);
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR);
};


void c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_flags_GET(pack) == e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY);
    assert(p246_lat_GET(pack) == (int32_t)1113435557);
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)24670);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)45432);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UNASSIGNED2);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t) -23766);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)3554927816L);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)48668);
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p246_altitude_GET(pack) == (int32_t) -1708736400);
    assert(p246_lon_GET(pack) == (int32_t)1032470985);
    assert(p246_callsign_LEN(ph) == 1);
    {
        char16_t * exemplary = u"h";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_id_GET(pack) == (uint32_t)159497008L);
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float)1.2359958E38F);
    assert(p247_time_to_minimum_delta_GET(pack) == (float) -1.8277834E38F);
    assert(p247_altitude_minimum_delta_GET(pack) == (float)3.2962114E38F);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_HOVER);
};


void c_LoopBackDemoChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)120);
    {
        uint8_t exemplary[] =  {(uint8_t)58, (uint8_t)115, (uint8_t)76, (uint8_t)95, (uint8_t)116, (uint8_t)251, (uint8_t)129, (uint8_t)33, (uint8_t)150, (uint8_t)180, (uint8_t)198, (uint8_t)217, (uint8_t)149, (uint8_t)22, (uint8_t)162, (uint8_t)147, (uint8_t)83, (uint8_t)200, (uint8_t)33, (uint8_t)163, (uint8_t)56, (uint8_t)120, (uint8_t)238, (uint8_t)246, (uint8_t)213, (uint8_t)88, (uint8_t)233, (uint8_t)235, (uint8_t)196, (uint8_t)107, (uint8_t)106, (uint8_t)90, (uint8_t)104, (uint8_t)85, (uint8_t)46, (uint8_t)251, (uint8_t)13, (uint8_t)255, (uint8_t)132, (uint8_t)37, (uint8_t)210, (uint8_t)41, (uint8_t)93, (uint8_t)223, (uint8_t)255, (uint8_t)121, (uint8_t)225, (uint8_t)149, (uint8_t)19, (uint8_t)255, (uint8_t)158, (uint8_t)71, (uint8_t)129, (uint8_t)34, (uint8_t)50, (uint8_t)177, (uint8_t)178, (uint8_t)137, (uint8_t)242, (uint8_t)184, (uint8_t)169, (uint8_t)178, (uint8_t)209, (uint8_t)1, (uint8_t)248, (uint8_t)214, (uint8_t)22, (uint8_t)18, (uint8_t)246, (uint8_t)167, (uint8_t)33, (uint8_t)77, (uint8_t)74, (uint8_t)156, (uint8_t)156, (uint8_t)35, (uint8_t)109, (uint8_t)200, (uint8_t)6, (uint8_t)107, (uint8_t)105, (uint8_t)62, (uint8_t)70, (uint8_t)153, (uint8_t)146, (uint8_t)137, (uint8_t)236, (uint8_t)62, (uint8_t)94, (uint8_t)71, (uint8_t)187, (uint8_t)199, (uint8_t)81, (uint8_t)30, (uint8_t)30, (uint8_t)175, (uint8_t)28, (uint8_t)249, (uint8_t)184, (uint8_t)58, (uint8_t)75, (uint8_t)181, (uint8_t)207, (uint8_t)226, (uint8_t)158, (uint8_t)253, (uint8_t)224, (uint8_t)81, (uint8_t)123, (uint8_t)209, (uint8_t)151, (uint8_t)153, (uint8_t)91, (uint8_t)8, (uint8_t)54, (uint8_t)120, (uint8_t)103, (uint8_t)10, (uint8_t)28, (uint8_t)128, (uint8_t)80, (uint8_t)76, (uint8_t)116, (uint8_t)127, (uint8_t)69, (uint8_t)173, (uint8_t)58, (uint8_t)153, (uint8_t)154, (uint8_t)55, (uint8_t)103, (uint8_t)182, (uint8_t)249, (uint8_t)107, (uint8_t)233, (uint8_t)25, (uint8_t)0, (uint8_t)219, (uint8_t)82, (uint8_t)43, (uint8_t)18, (uint8_t)191, (uint8_t)246, (uint8_t)183, (uint8_t)13, (uint8_t)147, (uint8_t)35, (uint8_t)248, (uint8_t)154, (uint8_t)222, (uint8_t)145, (uint8_t)127, (uint8_t)38, (uint8_t)221, (uint8_t)122, (uint8_t)0, (uint8_t)20, (uint8_t)187, (uint8_t)123, (uint8_t)65, (uint8_t)187, (uint8_t)101, (uint8_t)19, (uint8_t)25, (uint8_t)181, (uint8_t)37, (uint8_t)95, (uint8_t)212, (uint8_t)74, (uint8_t)123, (uint8_t)88, (uint8_t)122, (uint8_t)209, (uint8_t)58, (uint8_t)143, (uint8_t)38, (uint8_t)98, (uint8_t)178, (uint8_t)142, (uint8_t)157, (uint8_t)16, (uint8_t)128, (uint8_t)129, (uint8_t)59, (uint8_t)170, (uint8_t)200, (uint8_t)60, (uint8_t)245, (uint8_t)221, (uint8_t)233, (uint8_t)166, (uint8_t)240, (uint8_t)149, (uint8_t)5, (uint8_t)218, (uint8_t)233, (uint8_t)136, (uint8_t)13, (uint8_t)132, (uint8_t)74, (uint8_t)47, (uint8_t)96, (uint8_t)250, (uint8_t)207, (uint8_t)27, (uint8_t)179, (uint8_t)247, (uint8_t)123, (uint8_t)189, (uint8_t)6, (uint8_t)35, (uint8_t)91, (uint8_t)149, (uint8_t)4, (uint8_t)255, (uint8_t)233, (uint8_t)22, (uint8_t)85, (uint8_t)37, (uint8_t)184, (uint8_t)214, (uint8_t)153, (uint8_t)217, (uint8_t)122, (uint8_t)250, (uint8_t)76, (uint8_t)209, (uint8_t)178, (uint8_t)46, (uint8_t)116, (uint8_t)146, (uint8_t)249, (uint8_t)215, (uint8_t)63, (uint8_t)74, (uint8_t)201, (uint8_t)221, (uint8_t)171, (uint8_t)80, (uint8_t)51, (uint8_t)17, (uint8_t)178, (uint8_t)121, (uint8_t)123, (uint8_t)193, (uint8_t)74, (uint8_t)105, (uint8_t)108, (uint8_t)200} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)25112);
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)242);
};


void c_LoopBackDemoChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)14482);
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)200);
    {
        int8_t exemplary[] =  {(int8_t)35, (int8_t) -49, (int8_t)8, (int8_t) -113, (int8_t) -98, (int8_t) -66, (int8_t)118, (int8_t)46, (int8_t)21, (int8_t)25, (int8_t) -29, (int8_t)93, (int8_t) -126, (int8_t)101, (int8_t)46, (int8_t)73, (int8_t)92, (int8_t)84, (int8_t)115, (int8_t) -106, (int8_t) -62, (int8_t) -120, (int8_t)117, (int8_t) -115, (int8_t) -116, (int8_t)14, (int8_t)3, (int8_t) -92, (int8_t)27, (int8_t) -41, (int8_t)31, (int8_t) -89} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)130);
};


void c_LoopBackDemoChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_x_GET(pack) == (float) -1.4829207E38F);
    assert(p250_name_LEN(ph) == 9);
    {
        char16_t * exemplary = u"bzijZtbsa";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p250_z_GET(pack) == (float) -7.1027746E37F);
    assert(p250_y_GET(pack) == (float)1.3525078E38F);
    assert(p250_time_usec_GET(pack) == (uint64_t)8269817269184282550L);
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_name_LEN(ph) == 3);
    {
        char16_t * exemplary = u"mrs";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)3664150561L);
    assert(p251_value_GET(pack) == (float)1.0956553E36F);
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_name_LEN(ph) == 6);
    {
        char16_t * exemplary = u"zhnJpi";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p252_value_GET(pack) == (int32_t)927097636);
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)3792242384L);
};


void c_LoopBackDemoChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_EMERGENCY);
    assert(p253_text_LEN(ph) == 25);
    {
        char16_t * exemplary = u"livpfwlwLhCtejybwkupaadbz";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 50);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)94);
    assert(p254_value_GET(pack) == (float)1.2226327E38F);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)3098739729L);
};


void c_LoopBackDemoChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)3219416032820536588L);
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)43);
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)220);
    {
        uint8_t exemplary[] =  {(uint8_t)3, (uint8_t)123, (uint8_t)220, (uint8_t)133, (uint8_t)116, (uint8_t)132, (uint8_t)114, (uint8_t)139, (uint8_t)23, (uint8_t)182, (uint8_t)29, (uint8_t)108, (uint8_t)231, (uint8_t)246, (uint8_t)45, (uint8_t)83, (uint8_t)76, (uint8_t)240, (uint8_t)97, (uint8_t)164, (uint8_t)81, (uint8_t)172, (uint8_t)49, (uint8_t)145, (uint8_t)144, (uint8_t)21, (uint8_t)54, (uint8_t)34, (uint8_t)147, (uint8_t)70, (uint8_t)84, (uint8_t)128} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)888989796L);
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p257_last_change_ms_GET(pack) == (uint32_t)2937678060L);
};


void c_LoopBackDemoChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)209);
    assert(p258_tune_LEN(ph) == 25);
    {
        char16_t * exemplary = u"cwMmhpgeolCEwnpvalsddvnft";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 50);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)65);
};


void c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    assert(p259_firmware_version_GET(pack) == (uint32_t)604580272L);
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)51828);
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)19346);
    assert(p259_flags_GET(pack) == e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)39852);
    {
        uint8_t exemplary[] =  {(uint8_t)24, (uint8_t)230, (uint8_t)90, (uint8_t)32, (uint8_t)31, (uint8_t)226, (uint8_t)196, (uint8_t)225, (uint8_t)185, (uint8_t)17, (uint8_t)142, (uint8_t)119, (uint8_t)183, (uint8_t)35, (uint8_t)199, (uint8_t)4, (uint8_t)188, (uint8_t)153, (uint8_t)1, (uint8_t)118, (uint8_t)39, (uint8_t)129, (uint8_t)203, (uint8_t)207, (uint8_t)169, (uint8_t)149, (uint8_t)185, (uint8_t)51, (uint8_t)121, (uint8_t)250, (uint8_t)145, (uint8_t)193} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_cam_definition_uri_LEN(ph) == 11);
    {
        char16_t * exemplary = u"hxGyHeldalv";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 22);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_focal_length_GET(pack) == (float)2.4536994E38F);
    {
        uint8_t exemplary[] =  {(uint8_t)152, (uint8_t)186, (uint8_t)215, (uint8_t)32, (uint8_t)85, (uint8_t)76, (uint8_t)70, (uint8_t)70, (uint8_t)98, (uint8_t)228, (uint8_t)234, (uint8_t)2, (uint8_t)194, (uint8_t)3, (uint8_t)177, (uint8_t)3, (uint8_t)213, (uint8_t)229, (uint8_t)113, (uint8_t)209, (uint8_t)30, (uint8_t)226, (uint8_t)167, (uint8_t)3, (uint8_t)128, (uint8_t)238, (uint8_t)69, (uint8_t)4, (uint8_t)150, (uint8_t)35, (uint8_t)191, (uint8_t)237} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)1309907306L);
    assert(p259_sensor_size_h_GET(pack) == (float) -9.666031E37F);
    assert(p259_sensor_size_v_GET(pack) == (float)8.1888195E37F);
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)39);
};


void c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_IMAGE);
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)3994505774L);
};


void c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)3775668738L);
    assert(p261_total_capacity_GET(pack) == (float) -9.844026E37F);
    assert(p261_used_capacity_GET(pack) == (float)1.8202154E38F);
    assert(p261_write_speed_GET(pack) == (float) -1.9810716E38F);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)52);
    assert(p261_read_speed_GET(pack) == (float)2.0629405E38F);
    assert(p261_available_capacity_GET(pack) == (float) -1.6875967E38F);
};


void c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)157313535L);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)2360144998L);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)69);
    assert(p262_available_capacity_GET(pack) == (float)1.1271745E37F);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)85);
    assert(p262_image_interval_GET(pack) == (float) -1.3544878E38F);
};


void c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_time_utc_GET(pack) == (uint64_t)596422516538759598L);
    assert(p263_relative_alt_GET(pack) == (int32_t)1627064809);
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t) -103);
    assert(p263_image_index_GET(pack) == (int32_t)159098275);
    assert(p263_alt_GET(pack) == (int32_t)713523747);
    assert(p263_lon_GET(pack) == (int32_t)410349912);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)182);
    assert(p263_lat_GET(pack) == (int32_t) -184136594);
    {
        float exemplary[] =  {2.2445409E38F, -3.3703807E38F, -9.4415536E36F, -1.4890001E38F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)1092020896L);
    assert(p263_file_url_LEN(ph) == 127);
    {
        char16_t * exemplary = u"mbzzQgqfTkupvsxdqlyeMtlrlvFfuekreinhftuDjpogokPuCfusruifJqszyvmgjlomnobfznNnchnppLKmyipnnnaanwkhkbnJuxwkbhjgmufssGuImdztjbleksz";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 254);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)2762854935031538156L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)7956065181207754720L);
    assert(p264_flight_uuid_GET(pack) == (uint64_t)2129537382122781051L);
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)4059525250L);
};


void c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_roll_GET(pack) == (float) -1.9646647E38F);
    assert(p265_yaw_GET(pack) == (float)2.7789931E38F);
    assert(p265_pitch_GET(pack) == (float)1.9496485E37F);
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)3871428288L);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)55);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)101);
    {
        uint8_t exemplary[] =  {(uint8_t)203, (uint8_t)199, (uint8_t)141, (uint8_t)182, (uint8_t)104, (uint8_t)41, (uint8_t)30, (uint8_t)90, (uint8_t)138, (uint8_t)190, (uint8_t)190, (uint8_t)255, (uint8_t)178, (uint8_t)72, (uint8_t)66, (uint8_t)5, (uint8_t)223, (uint8_t)35, (uint8_t)222, (uint8_t)172, (uint8_t)68, (uint8_t)56, (uint8_t)77, (uint8_t)168, (uint8_t)26, (uint8_t)113, (uint8_t)200, (uint8_t)56, (uint8_t)165, (uint8_t)165, (uint8_t)66, (uint8_t)203, (uint8_t)157, (uint8_t)63, (uint8_t)169, (uint8_t)205, (uint8_t)240, (uint8_t)162, (uint8_t)200, (uint8_t)195, (uint8_t)170, (uint8_t)122, (uint8_t)191, (uint8_t)116, (uint8_t)40, (uint8_t)12, (uint8_t)187, (uint8_t)237, (uint8_t)177, (uint8_t)57, (uint8_t)205, (uint8_t)227, (uint8_t)68, (uint8_t)57, (uint8_t)241, (uint8_t)250, (uint8_t)74, (uint8_t)183, (uint8_t)244, (uint8_t)231, (uint8_t)209, (uint8_t)16, (uint8_t)83, (uint8_t)85, (uint8_t)157, (uint8_t)73, (uint8_t)177, (uint8_t)49, (uint8_t)205, (uint8_t)44, (uint8_t)167, (uint8_t)169, (uint8_t)17, (uint8_t)239, (uint8_t)226, (uint8_t)253, (uint8_t)229, (uint8_t)8, (uint8_t)14, (uint8_t)96, (uint8_t)136, (uint8_t)158, (uint8_t)186, (uint8_t)132, (uint8_t)131, (uint8_t)56, (uint8_t)115, (uint8_t)85, (uint8_t)123, (uint8_t)158, (uint8_t)186, (uint8_t)66, (uint8_t)153, (uint8_t)46, (uint8_t)154, (uint8_t)141, (uint8_t)211, (uint8_t)224, (uint8_t)17, (uint8_t)158, (uint8_t)181, (uint8_t)206, (uint8_t)88, (uint8_t)162, (uint8_t)124, (uint8_t)146, (uint8_t)170, (uint8_t)103, (uint8_t)198, (uint8_t)193, (uint8_t)41, (uint8_t)164, (uint8_t)153, (uint8_t)4, (uint8_t)164, (uint8_t)3, (uint8_t)121, (uint8_t)218, (uint8_t)215, (uint8_t)156, (uint8_t)151, (uint8_t)45, (uint8_t)10, (uint8_t)139, (uint8_t)46, (uint8_t)236, (uint8_t)6, (uint8_t)38, (uint8_t)35, (uint8_t)102, (uint8_t)45, (uint8_t)152, (uint8_t)171, (uint8_t)175, (uint8_t)147, (uint8_t)222, (uint8_t)229, (uint8_t)86, (uint8_t)104, (uint8_t)215, (uint8_t)220, (uint8_t)205, (uint8_t)127, (uint8_t)42, (uint8_t)3, (uint8_t)167, (uint8_t)96, (uint8_t)56, (uint8_t)38, (uint8_t)200, (uint8_t)216, (uint8_t)50, (uint8_t)216, (uint8_t)239, (uint8_t)105, (uint8_t)247, (uint8_t)56, (uint8_t)112, (uint8_t)178, (uint8_t)245, (uint8_t)159, (uint8_t)129, (uint8_t)197, (uint8_t)199, (uint8_t)20, (uint8_t)47, (uint8_t)177, (uint8_t)178, (uint8_t)159, (uint8_t)16, (uint8_t)186, (uint8_t)67, (uint8_t)125, (uint8_t)186, (uint8_t)179, (uint8_t)211, (uint8_t)200, (uint8_t)69, (uint8_t)74, (uint8_t)178, (uint8_t)42, (uint8_t)21, (uint8_t)150, (uint8_t)76, (uint8_t)236, (uint8_t)134, (uint8_t)158, (uint8_t)94, (uint8_t)101, (uint8_t)131, (uint8_t)120, (uint8_t)13, (uint8_t)106, (uint8_t)22, (uint8_t)151, (uint8_t)84, (uint8_t)244, (uint8_t)151, (uint8_t)163, (uint8_t)92, (uint8_t)252, (uint8_t)133, (uint8_t)108, (uint8_t)208, (uint8_t)42, (uint8_t)190, (uint8_t)103, (uint8_t)43, (uint8_t)143, (uint8_t)109, (uint8_t)213, (uint8_t)195, (uint8_t)144, (uint8_t)237, (uint8_t)160, (uint8_t)182, (uint8_t)54, (uint8_t)167, (uint8_t)205, (uint8_t)146, (uint8_t)174, (uint8_t)189, (uint8_t)163, (uint8_t)183, (uint8_t)36, (uint8_t)73, (uint8_t)235, (uint8_t)202, (uint8_t)79, (uint8_t)152, (uint8_t)192, (uint8_t)217, (uint8_t)94, (uint8_t)219, (uint8_t)69, (uint8_t)191, (uint8_t)226, (uint8_t)226, (uint8_t)206, (uint8_t)131, (uint8_t)0, (uint8_t)218, (uint8_t)96, (uint8_t)205, (uint8_t)189, (uint8_t)13, (uint8_t)25, (uint8_t)173, (uint8_t)168} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)53053);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)138, (uint8_t)250, (uint8_t)26, (uint8_t)215, (uint8_t)73, (uint8_t)204, (uint8_t)247, (uint8_t)215, (uint8_t)141, (uint8_t)245, (uint8_t)5, (uint8_t)234, (uint8_t)79, (uint8_t)212, (uint8_t)23, (uint8_t)130, (uint8_t)91, (uint8_t)224, (uint8_t)61, (uint8_t)24, (uint8_t)1, (uint8_t)57, (uint8_t)140, (uint8_t)201, (uint8_t)129, (uint8_t)201, (uint8_t)200, (uint8_t)61, (uint8_t)116, (uint8_t)169, (uint8_t)72, (uint8_t)32, (uint8_t)139, (uint8_t)66, (uint8_t)4, (uint8_t)135, (uint8_t)9, (uint8_t)68, (uint8_t)232, (uint8_t)162, (uint8_t)237, (uint8_t)164, (uint8_t)251, (uint8_t)104, (uint8_t)181, (uint8_t)3, (uint8_t)35, (uint8_t)236, (uint8_t)19, (uint8_t)133, (uint8_t)138, (uint8_t)152, (uint8_t)142, (uint8_t)161, (uint8_t)186, (uint8_t)191, (uint8_t)173, (uint8_t)52, (uint8_t)117, (uint8_t)172, (uint8_t)235, (uint8_t)234, (uint8_t)212, (uint8_t)163, (uint8_t)4, (uint8_t)103, (uint8_t)0, (uint8_t)29, (uint8_t)5, (uint8_t)10, (uint8_t)246, (uint8_t)227, (uint8_t)188, (uint8_t)233, (uint8_t)54, (uint8_t)168, (uint8_t)51, (uint8_t)133, (uint8_t)112, (uint8_t)184, (uint8_t)18, (uint8_t)239, (uint8_t)176, (uint8_t)83, (uint8_t)142, (uint8_t)253, (uint8_t)49, (uint8_t)150, (uint8_t)184, (uint8_t)155, (uint8_t)199, (uint8_t)127, (uint8_t)244, (uint8_t)93, (uint8_t)218, (uint8_t)88, (uint8_t)208, (uint8_t)89, (uint8_t)221, (uint8_t)120, (uint8_t)68, (uint8_t)101, (uint8_t)114, (uint8_t)57, (uint8_t)154, (uint8_t)208, (uint8_t)28, (uint8_t)10, (uint8_t)164, (uint8_t)54, (uint8_t)119, (uint8_t)234, (uint8_t)145, (uint8_t)151, (uint8_t)5, (uint8_t)201, (uint8_t)83, (uint8_t)126, (uint8_t)175, (uint8_t)62, (uint8_t)15, (uint8_t)249, (uint8_t)164, (uint8_t)58, (uint8_t)196, (uint8_t)75, (uint8_t)226, (uint8_t)33, (uint8_t)137, (uint8_t)205, (uint8_t)53, (uint8_t)232, (uint8_t)211, (uint8_t)249, (uint8_t)118, (uint8_t)41, (uint8_t)15, (uint8_t)13, (uint8_t)66, (uint8_t)203, (uint8_t)216, (uint8_t)238, (uint8_t)17, (uint8_t)164, (uint8_t)236, (uint8_t)180, (uint8_t)216, (uint8_t)189, (uint8_t)38, (uint8_t)139, (uint8_t)13, (uint8_t)46, (uint8_t)43, (uint8_t)38, (uint8_t)175, (uint8_t)98, (uint8_t)200, (uint8_t)18, (uint8_t)162, (uint8_t)203, (uint8_t)1, (uint8_t)70, (uint8_t)169, (uint8_t)253, (uint8_t)240, (uint8_t)177, (uint8_t)152, (uint8_t)17, (uint8_t)244, (uint8_t)173, (uint8_t)167, (uint8_t)91, (uint8_t)114, (uint8_t)255, (uint8_t)139, (uint8_t)158, (uint8_t)84, (uint8_t)248, (uint8_t)0, (uint8_t)255, (uint8_t)32, (uint8_t)95, (uint8_t)89, (uint8_t)185, (uint8_t)189, (uint8_t)98, (uint8_t)183, (uint8_t)48, (uint8_t)182, (uint8_t)181, (uint8_t)110, (uint8_t)224, (uint8_t)129, (uint8_t)179, (uint8_t)87, (uint8_t)186, (uint8_t)105, (uint8_t)86, (uint8_t)66, (uint8_t)181, (uint8_t)213, (uint8_t)57, (uint8_t)60, (uint8_t)3, (uint8_t)18, (uint8_t)202, (uint8_t)15, (uint8_t)207, (uint8_t)181, (uint8_t)230, (uint8_t)154, (uint8_t)121, (uint8_t)247, (uint8_t)10, (uint8_t)34, (uint8_t)120, (uint8_t)99, (uint8_t)58, (uint8_t)205, (uint8_t)9, (uint8_t)34, (uint8_t)182, (uint8_t)179, (uint8_t)180, (uint8_t)21, (uint8_t)97, (uint8_t)62, (uint8_t)21, (uint8_t)12, (uint8_t)195, (uint8_t)244, (uint8_t)249, (uint8_t)71, (uint8_t)254, (uint8_t)20, (uint8_t)221, (uint8_t)77, (uint8_t)147, (uint8_t)47, (uint8_t)243, (uint8_t)177, (uint8_t)128, (uint8_t)21, (uint8_t)240, (uint8_t)52, (uint8_t)129, (uint8_t)236, (uint8_t)179, (uint8_t)122} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)8);
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)148);
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)61);
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)24);
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)20529);
};


void c_LoopBackDemoChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)56912);
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)37);
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)180);
};


void c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_framerate_GET(pack) == (float) -5.248207E37F);
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)7049);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)14);
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)26772);
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)26);
    assert(p269_bitrate_GET(pack) == (uint32_t)2943415683L);
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)37754);
    assert(p269_uri_LEN(ph) == 45);
    {
        char16_t * exemplary = u"zwgnrspnixasggzrktasmjmlypgKqcysPyMdeQfbhiexc";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)69);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)61473);
    assert(p270_bitrate_GET(pack) == (uint32_t)34394L);
    assert(p270_uri_LEN(ph) == 51);
    {
        char16_t * exemplary = u"rtpknhnrqcjtztrdyZbxeQdukvutidlPRTLutnammjcbkurbycO";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 102);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)60374);
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)44776);
    assert(p270_framerate_GET(pack) == (float)1.0923621E38F);
};


void c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_ssid_LEN(ph) == 21);
    {
        char16_t * exemplary = u"faqaCfhohtftshfqcwbsn";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 42);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_password_LEN(ph) == 48);
    {
        char16_t * exemplary = u"hxayzwjqqXmKlmlyaaolbyhufphjbbgwyhmzkootdhlxXdsn";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 96);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)212, (uint8_t)94, (uint8_t)105, (uint8_t)59, (uint8_t)162, (uint8_t)218, (uint8_t)11, (uint8_t)79} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)18478);
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)6938);
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)6296);
    {
        uint8_t exemplary[] =  {(uint8_t)175, (uint8_t)84, (uint8_t)197, (uint8_t)172, (uint8_t)225, (uint8_t)135, (uint8_t)215, (uint8_t)170} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION);
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)25635);
    assert(p310_uptime_sec_GET(pack) == (uint32_t)1750481280L);
    assert(p310_time_usec_GET(pack) == (uint64_t)1383335241753802820L);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)21);
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)218);
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)176);
    {
        uint8_t exemplary[] =  {(uint8_t)70, (uint8_t)112, (uint8_t)13, (uint8_t)10, (uint8_t)54, (uint8_t)117, (uint8_t)74, (uint8_t)191, (uint8_t)88, (uint8_t)200, (uint8_t)101, (uint8_t)224, (uint8_t)91, (uint8_t)149, (uint8_t)107, (uint8_t)43} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)32);
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)44);
    assert(p311_uptime_sec_GET(pack) == (uint32_t)2583241550L);
    assert(p311_name_LEN(ph) == 4);
    {
        char16_t * exemplary = u"wxdz";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_time_usec_GET(pack) == (uint64_t)6509381760313854201L);
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)2912440542L);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)201);
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t)1771);
    assert(p320_param_id_LEN(ph) == 14);
    {
        char16_t * exemplary = u"fwcdiAcjxEaxek";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 28);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)21);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)102);
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)128);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_value_LEN(ph) == 28);
    {
        char16_t * exemplary = u"ifbbnqeltbPpkzobqupcqlfDYxvh";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 56);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_id_LEN(ph) == 4);
    {
        char16_t * exemplary = u"wqaf";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)59969);
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8);
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)12660);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_param_value_LEN(ph) == 41);
    {
        char16_t * exemplary = u"mNKumvftnuLlivAokzqpayacuRksccqkvtxhnpgml";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 82);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64);
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p323_param_id_LEN(ph) == 13);
    {
        char16_t * exemplary = u"lfTFIoUgwjjfC";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 26);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)148);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32);
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_ACCEPTED);
    assert(p324_param_value_LEN(ph) == 82);
    {
        char16_t * exemplary = u"FjpvmsidzhosmwpivmonyzOtxknrppggjoyanluiyVxtjspgqzwwihopVkyhalyertfznYYrqiafdklrpz";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 164);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_id_LEN(ph) == 6);
    {
        char16_t * exemplary = u"sWhqZP";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)51621);
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED);
    assert(p330_time_usec_GET(pack) == (uint64_t)4805460286946828669L);
    {
        uint16_t exemplary[] =  {(uint16_t)733, (uint16_t)51595, (uint16_t)31374, (uint16_t)38572, (uint16_t)23728, (uint16_t)47971, (uint16_t)41859, (uint16_t)9423, (uint16_t)47691, (uint16_t)46149, (uint16_t)58397, (uint16_t)29320, (uint16_t)23297, (uint16_t)46489, (uint16_t)22381, (uint16_t)11280, (uint16_t)39870, (uint16_t)503, (uint16_t)17841, (uint16_t)13023, (uint16_t)47512, (uint16_t)49349, (uint16_t)50381, (uint16_t)47027, (uint16_t)1706, (uint16_t)32564, (uint16_t)45945, (uint16_t)494, (uint16_t)135, (uint16_t)61009, (uint16_t)56914, (uint16_t)25610, (uint16_t)47494, (uint16_t)31413, (uint16_t)726, (uint16_t)55121, (uint16_t)23645, (uint16_t)41927, (uint16_t)16948, (uint16_t)54331, (uint16_t)58723, (uint16_t)5232, (uint16_t)39229, (uint16_t)20252, (uint16_t)50612, (uint16_t)57336, (uint16_t)62637, (uint16_t)64000, (uint16_t)35122, (uint16_t)37824, (uint16_t)15970, (uint16_t)19698, (uint16_t)8136, (uint16_t)58941, (uint16_t)57820, (uint16_t)48068, (uint16_t)12376, (uint16_t)16168, (uint16_t)267, (uint16_t)42962, (uint16_t)58852, (uint16_t)17237, (uint16_t)33070, (uint16_t)34205, (uint16_t)15845, (uint16_t)50719, (uint16_t)24497, (uint16_t)13037, (uint16_t)51674, (uint16_t)22550, (uint16_t)5385, (uint16_t)51354} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)247);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)41363);
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
        p0_mavlink_version_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_HIL_ENABLED, PH.base.pack) ;
        p0_custom_mode_SET((uint32_t)746907793L, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_GENERIC, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_GCS, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_BOOT, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYS_STATUS_1(), &PH);
        p1_current_battery_SET((int16_t)(int16_t)30304, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)31357, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)48580, PH.base.pack) ;
        p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)40641, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)57890, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)38054, PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)16841, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)57976, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)4954, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t) -22, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_LOGGING, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_boot_ms_SET((uint32_t)850154985L, PH.base.pack) ;
        p2_time_unix_usec_SET((uint64_t)7940413277649479192L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)2661330200L, PH.base.pack) ;
        p3_afz_SET((float) -1.773599E38F, PH.base.pack) ;
        p3_y_SET((float) -3.0857315E38F, PH.base.pack) ;
        p3_afx_SET((float) -7.9932794E37F, PH.base.pack) ;
        p3_yaw_SET((float) -7.727453E37F, PH.base.pack) ;
        p3_yaw_rate_SET((float) -3.1946975E38F, PH.base.pack) ;
        p3_vz_SET((float)3.3848547E38F, PH.base.pack) ;
        p3_z_SET((float)2.4238118E37F, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)23857, PH.base.pack) ;
        p3_vy_SET((float)1.9341914E38F, PH.base.pack) ;
        p3_x_SET((float)2.190926E38F, PH.base.pack) ;
        p3_vx_SET((float)1.297936E38F, PH.base.pack) ;
        p3_afy_SET((float)7.261288E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PING_4(), &PH);
        p4_target_component_SET((uint8_t)(uint8_t)74, PH.base.pack) ;
        p4_time_usec_SET((uint64_t)4980911259099929723L, PH.base.pack) ;
        p4_target_system_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p4_seq_SET((uint32_t)595581859L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        {
            char16_t* passkey = u"tsttbgfeohxlnqfx";
            p5_passkey_SET_(passkey, &PH) ;
        }
        p5_target_system_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        p5_version_SET((uint8_t)(uint8_t)187, PH.base.pack) ;
        p5_control_request_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_ack_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p6_control_request_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"zsIxqvat";
            p7_key_SET_(key, &PH) ;
        }
        c_LoopBackDemoChannel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_MODE_11(), &PH);
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_ARMED, PH.base.pack) ;
        p11_custom_mode_SET((uint32_t)2350801323L, PH.base.pack) ;
        p11_target_system_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_READ_20(), &PH);
        {
            char16_t* param_id = u"goqfAejqguvna";
            p20_param_id_SET_(param_id, &PH) ;
        }
        p20_param_index_SET((int16_t)(int16_t) -11298, PH.base.pack) ;
        p20_target_component_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        p20_target_system_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_component_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        p21_target_system_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_VALUE_22(), &PH);
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT32, PH.base.pack) ;
        {
            char16_t* param_id = u"wEOntrnvisvc";
            p22_param_id_SET_(param_id, &PH) ;
        }
        p22_param_value_SET((float)2.3736376E38F, PH.base.pack) ;
        p22_param_count_SET((uint16_t)(uint16_t)40100, PH.base.pack) ;
        p22_param_index_SET((uint16_t)(uint16_t)8819, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_SET_23(), &PH);
        p23_target_component_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        p23_target_system_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_UINT16, PH.base.pack) ;
        p23_param_value_SET((float) -3.1535969E38F, PH.base.pack) ;
        {
            char16_t* param_id = u"bfHflMHthQvLbguq";
            p23_param_id_SET_(param_id, &PH) ;
        }
        c_LoopBackDemoChannel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RAW_INT_24(), &PH);
        p24_satellites_visible_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
        p24_hdg_acc_SET((uint32_t)1298678203L, &PH) ;
        p24_eph_SET((uint16_t)(uint16_t)14206, PH.base.pack) ;
        p24_vel_acc_SET((uint32_t)2163871150L, &PH) ;
        p24_vel_SET((uint16_t)(uint16_t)44720, PH.base.pack) ;
        p24_h_acc_SET((uint32_t)3582498266L, &PH) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_RTK_FLOAT, PH.base.pack) ;
        p24_alt_ellipsoid_SET((int32_t) -286335610, &PH) ;
        p24_lon_SET((int32_t) -138596825, PH.base.pack) ;
        p24_lat_SET((int32_t) -791806173, PH.base.pack) ;
        p24_alt_SET((int32_t) -1765063902, PH.base.pack) ;
        p24_v_acc_SET((uint32_t)4158275042L, &PH) ;
        p24_cog_SET((uint16_t)(uint16_t)30953, PH.base.pack) ;
        p24_time_usec_SET((uint64_t)905128649725603582L, PH.base.pack) ;
        p24_epv_SET((uint16_t)(uint16_t)62243, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)182, (uint8_t)158, (uint8_t)26, (uint8_t)34, (uint8_t)230, (uint8_t)187, (uint8_t)178, (uint8_t)88, (uint8_t)184, (uint8_t)210, (uint8_t)3, (uint8_t)7, (uint8_t)119, (uint8_t)24, (uint8_t)242, (uint8_t)121, (uint8_t)84, (uint8_t)45, (uint8_t)227, (uint8_t)134};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_snr[] =  {(uint8_t)73, (uint8_t)144, (uint8_t)155, (uint8_t)232, (uint8_t)92, (uint8_t)199, (uint8_t)104, (uint8_t)12, (uint8_t)28, (uint8_t)104, (uint8_t)86, (uint8_t)122, (uint8_t)206, (uint8_t)245, (uint8_t)33, (uint8_t)148, (uint8_t)105, (uint8_t)190, (uint8_t)132, (uint8_t)44};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)23, (uint8_t)129, (uint8_t)104, (uint8_t)55, (uint8_t)200, (uint8_t)218, (uint8_t)190, (uint8_t)29, (uint8_t)231, (uint8_t)91, (uint8_t)92, (uint8_t)62, (uint8_t)231, (uint8_t)145, (uint8_t)21, (uint8_t)95, (uint8_t)89, (uint8_t)213, (uint8_t)115, (uint8_t)83};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)219, PH.base.pack) ;
        {
            uint8_t satellite_used[] =  {(uint8_t)177, (uint8_t)206, (uint8_t)240, (uint8_t)172, (uint8_t)44, (uint8_t)253, (uint8_t)212, (uint8_t)29, (uint8_t)202, (uint8_t)162, (uint8_t)109, (uint8_t)18, (uint8_t)21, (uint8_t)91, (uint8_t)129, (uint8_t)199, (uint8_t)121, (uint8_t)223, (uint8_t)81, (uint8_t)18};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_prn[] =  {(uint8_t)81, (uint8_t)4, (uint8_t)158, (uint8_t)175, (uint8_t)67, (uint8_t)220, (uint8_t)143, (uint8_t)96, (uint8_t)253, (uint8_t)224, (uint8_t)249, (uint8_t)24, (uint8_t)78, (uint8_t)242, (uint8_t)193, (uint8_t)56, (uint8_t)204, (uint8_t)20, (uint8_t)77, (uint8_t)94};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU_26(), &PH);
        p26_zmag_SET((int16_t)(int16_t) -26484, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t) -6022, PH.base.pack) ;
        p26_time_boot_ms_SET((uint32_t)975908389L, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t)32076, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t)13393, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t) -27760, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t) -3616, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t) -25930, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t)30334, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t)25933, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_IMU_27(), &PH);
        p27_ygyro_SET((int16_t)(int16_t)27866, PH.base.pack) ;
        p27_zacc_SET((int16_t)(int16_t)4865, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t)7495, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)4074352477166708791L, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t) -9492, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t) -538, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t) -5381, PH.base.pack) ;
        p27_xmag_SET((int16_t)(int16_t)31896, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t) -29135, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t) -28565, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_PRESSURE_28(), &PH);
        p28_press_diff2_SET((int16_t)(int16_t) -16549, PH.base.pack) ;
        p28_press_diff1_SET((int16_t)(int16_t) -2127, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t) -26296, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)5161584472728574185L, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t)3951, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_press_abs_SET((float) -2.1582181E38F, PH.base.pack) ;
        p29_press_diff_SET((float)3.0333849E38F, PH.base.pack) ;
        p29_temperature_SET((int16_t)(int16_t)2234, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)3293051552L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_30(), &PH);
        p30_roll_SET((float) -8.2994023E37F, PH.base.pack) ;
        p30_yawspeed_SET((float) -6.296374E37F, PH.base.pack) ;
        p30_time_boot_ms_SET((uint32_t)176238102L, PH.base.pack) ;
        p30_pitchspeed_SET((float) -3.3157582E37F, PH.base.pack) ;
        p30_pitch_SET((float)1.7588532E38F, PH.base.pack) ;
        p30_rollspeed_SET((float) -2.6899653E38F, PH.base.pack) ;
        p30_yaw_SET((float) -2.279624E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_pitchspeed_SET((float)1.6670614E38F, PH.base.pack) ;
        p31_q1_SET((float) -9.645267E37F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)421225493L, PH.base.pack) ;
        p31_q2_SET((float)3.0786025E38F, PH.base.pack) ;
        p31_q3_SET((float)2.0179992E38F, PH.base.pack) ;
        p31_yawspeed_SET((float) -1.4434092E38F, PH.base.pack) ;
        p31_rollspeed_SET((float) -3.2483175E38F, PH.base.pack) ;
        p31_q4_SET((float) -2.0284744E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_z_SET((float)1.5706201E38F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)3861791601L, PH.base.pack) ;
        p32_vx_SET((float) -1.9770617E38F, PH.base.pack) ;
        p32_y_SET((float) -2.3640734E38F, PH.base.pack) ;
        p32_vz_SET((float)2.8581842E37F, PH.base.pack) ;
        p32_vy_SET((float) -1.776045E38F, PH.base.pack) ;
        p32_x_SET((float)1.3391524E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_time_boot_ms_SET((uint32_t)4059788384L, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t) -17795, PH.base.pack) ;
        p33_lon_SET((int32_t)1906645972, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t) -27250, PH.base.pack) ;
        p33_relative_alt_SET((int32_t)856551769, PH.base.pack) ;
        p33_alt_SET((int32_t)1233241208, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t)9235, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)26907, PH.base.pack) ;
        p33_lat_SET((int32_t)809838976, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_rssi_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t) -18502, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t) -15920, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t)9380, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)122, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)3691972021L, PH.base.pack) ;
        p34_chan5_scaled_SET((int16_t)(int16_t)23330, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t) -24542, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t)30477, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t)6496, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t)22233, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_chan4_raw_SET((uint16_t)(uint16_t)20429, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)21242, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)45569, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)942046927L, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)62270, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)26268, PH.base.pack) ;
        p35_chan6_raw_SET((uint16_t)(uint16_t)42581, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)46311, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)15045, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_servo12_raw_SET((uint16_t)(uint16_t)43516, &PH) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)342, &PH) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)64955, &PH) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)28194, &PH) ;
        p36_time_usec_SET((uint32_t)1270565008L, PH.base.pack) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)8768, PH.base.pack) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)8033, &PH) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)33611, &PH) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)13271, PH.base.pack) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)56726, PH.base.pack) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)50673, PH.base.pack) ;
        p36_port_SET((uint8_t)(uint8_t)141, PH.base.pack) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)29813, PH.base.pack) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)49786, PH.base.pack) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)2026, &PH) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)28875, PH.base.pack) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)36013, &PH) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)57721, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_start_index_SET((int16_t)(int16_t)31185, PH.base.pack) ;
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p37_target_component_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t)21592, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_target_system_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p38_target_component_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p38_end_index_SET((int16_t)(int16_t) -16701, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t)12819, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_39(), &PH);
        p39_y_SET((float)2.3074478E38F, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)236, PH.base.pack) ;
        p39_param3_SET((float) -4.6586966E37F, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p39_z_SET((float) -7.192847E37F, PH.base.pack) ;
        p39_x_SET((float) -2.6381046E38F, PH.base.pack) ;
        p39_param4_SET((float)1.1223545E38F, PH.base.pack) ;
        p39_param1_SET((float)2.4267682E38F, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL, PH.base.pack) ;
        p39_param2_SET((float)3.0213906E38F, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_REQUEST_CAMERA_INFORMATION, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)2064, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_40(), &PH);
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p40_seq_SET((uint16_t)(uint16_t)37106, PH.base.pack) ;
        p40_target_component_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_target_system_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        p41_seq_SET((uint16_t)(uint16_t)52248, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)854, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_target_component_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p43_target_system_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_COUNT_44(), &PH);
        p44_target_component_SET((uint8_t)(uint8_t)109, PH.base.pack) ;
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)161, PH.base.pack) ;
        p44_count_SET((uint16_t)(uint16_t)31339, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_target_system_SET((uint8_t)(uint8_t)20, PH.base.pack) ;
        p45_target_component_SET((uint8_t)(uint8_t)104, PH.base.pack) ;
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)5213, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ACK_47(), &PH);
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_INVALID, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p47_target_component_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_altitude_SET((int32_t) -306749084, PH.base.pack) ;
        p48_time_usec_SET((uint64_t)5866821489129040147L, &PH) ;
        p48_longitude_SET((int32_t)2005871769, PH.base.pack) ;
        p48_target_system_SET((uint8_t)(uint8_t)162, PH.base.pack) ;
        p48_latitude_SET((int32_t) -114563872, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_time_usec_SET((uint64_t)8457574272267750069L, &PH) ;
        p49_altitude_SET((int32_t) -1539350010, PH.base.pack) ;
        p49_longitude_SET((int32_t)1656970326, PH.base.pack) ;
        p49_latitude_SET((int32_t)554305339, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_target_component_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
        p50_param_value0_SET((float) -1.1532727E38F, PH.base.pack) ;
        {
            char16_t* param_id = u"JrnyKktwd";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_param_value_min_SET((float) -2.4446555E38F, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)106, PH.base.pack) ;
        p50_scale_SET((float)1.6821304E38F, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t)5149, PH.base.pack) ;
        p50_param_value_max_SET((float) -2.833891E38F, PH.base.pack) ;
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)62, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_target_component_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)40240, PH.base.pack) ;
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_target_component_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p54_p1x_SET((float) -2.8704327E38F, PH.base.pack) ;
        p54_p2y_SET((float) -1.3577981E38F, PH.base.pack) ;
        p54_p1y_SET((float)2.6070072E38F, PH.base.pack) ;
        p54_p2x_SET((float) -8.244991E37F, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        p54_p2z_SET((float)1.774943E38F, PH.base.pack) ;
        p54_p1z_SET((float)5.166072E37F, PH.base.pack) ;
        p54_target_system_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p2z_SET((float) -1.9684863E38F, PH.base.pack) ;
        p55_p1x_SET((float)2.2166905E38F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        p55_p2y_SET((float)2.7627137E38F, PH.base.pack) ;
        p55_p1y_SET((float) -3.3730546E38F, PH.base.pack) ;
        p55_p2x_SET((float)1.6037819E38F, PH.base.pack) ;
        p55_p1z_SET((float)3.3092816E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_pitchspeed_SET((float) -9.964348E35F, PH.base.pack) ;
        {
            float covariance[] =  {-2.249146E38F, -1.5480198E38F, 2.1029532E38F, 3.1823786E38F, 5.7450133E37F, -2.1660753E38F, -1.7790286E38F, -1.5131581E37F, 2.1428425E38F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p61_yawspeed_SET((float)2.2356225E38F, PH.base.pack) ;
        p61_rollspeed_SET((float) -4.7485324E37F, PH.base.pack) ;
        {
            float q[] =  {-1.2852599E38F, -7.3858303E37F, -3.1682091E38F, 4.512943E37F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        p61_time_usec_SET((uint64_t)4902443481483851250L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_aspd_error_SET((float) -2.1761693E38F, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t) -9705, PH.base.pack) ;
        p62_nav_pitch_SET((float)1.3456571E38F, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)35433, PH.base.pack) ;
        p62_xtrack_error_SET((float)1.225728E38F, PH.base.pack) ;
        p62_target_bearing_SET((int16_t)(int16_t)517, PH.base.pack) ;
        p62_nav_roll_SET((float)2.614913E38F, PH.base.pack) ;
        p62_alt_error_SET((float)1.2449002E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_relative_alt_SET((int32_t)605387207, PH.base.pack) ;
        p63_lon_SET((int32_t) -977267411, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)1883987170271804026L, PH.base.pack) ;
        p63_vx_SET((float) -8.666514E37F, PH.base.pack) ;
        p63_vz_SET((float) -5.9422567E37F, PH.base.pack) ;
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, PH.base.pack) ;
        p63_alt_SET((int32_t) -737571809, PH.base.pack) ;
        {
            float covariance[] =  {1.9094605E38F, 1.6311393E38F, -2.6800385E38F, -3.2741076E38F, -4.3941795E37F, -3.3497588E38F, 2.6622695E38F, 2.904801E38F, 3.46457E37F, 2.9467824E38F, -3.5186132E37F, 2.576551E38F, 1.7121072E38F, -3.9323995E36F, -2.6324101E38F, -1.8259365E38F, -3.0289085E38F, -2.0484988E38F, -7.5949587E37F, 3.2795233E38F, -1.8636683E38F, -2.649916E38F, 7.033309E37F, 3.3660995E38F, 2.9013036E38F, -1.9544558E38F, -1.1131358E38F, 1.6554268E38F, 1.00363194E37F, -2.658712E38F, -2.1464803E38F, 2.421535E38F, -2.8677064E38F, 1.1664935E38F, -7.5847627E37F, 2.9857849E38F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p63_vy_SET((float) -2.438959E38F, PH.base.pack) ;
        p63_lat_SET((int32_t)122051686, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_az_SET((float) -7.1307456E37F, PH.base.pack) ;
        p64_vz_SET((float) -2.906276E38F, PH.base.pack) ;
        p64_time_usec_SET((uint64_t)927273469140416714L, PH.base.pack) ;
        p64_vx_SET((float)1.3335146E38F, PH.base.pack) ;
        p64_x_SET((float)1.4173287E38F, PH.base.pack) ;
        p64_ax_SET((float)5.454039E37F, PH.base.pack) ;
        p64_z_SET((float)3.0521781E38F, PH.base.pack) ;
        p64_vy_SET((float) -1.7038849E38F, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VIO, PH.base.pack) ;
        {
            float covariance[] =  {-2.4422325E38F, -1.4630636E38F, -3.7148146E37F, 8.71808E37F, 3.0564995E38F, -1.6638921E38F, 2.3391275E38F, -8.813324E37F, 2.821655E38F, -1.3593637E38F, 4.6533385E37F, -1.5341846E38F, 2.946233E37F, -3.2457806E38F, -9.872926E37F, 8.4425114E37F, -3.2768176E38F, -3.3460723E38F, -2.8702289E38F, 2.2582305E38F, 3.3835175E38F, 6.0622576E37F, -8.4485906E37F, -5.3030667E37F, -2.8576169E38F, -3.3395251E38F, 2.2854964E38F, -2.8608943E38F, 9.012564E37F, -8.53777E36F, -1.9249597E38F, 3.322379E38F, -3.1845742E38F, -1.8013755E38F, -7.334434E36F, 2.282915E38F, -3.1725285E38F, -1.9302388E38F, 2.5759413E38F, -2.0123521E38F, -2.7038683E38F, 9.972482E37F, 1.1294098E38F, 2.2622537E38F, -1.714086E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_y_SET((float) -1.3119562E38F, PH.base.pack) ;
        p64_ay_SET((float) -1.4376243E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan16_raw_SET((uint16_t)(uint16_t)21290, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)30547, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)26160, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)58404, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)7197, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)25255, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)46880, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)11196, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)54400, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)3398520317L, PH.base.pack) ;
        p65_chan17_raw_SET((uint16_t)(uint16_t)56326, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)57166, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)1398, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)65174, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)25848, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)39606, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)57030, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)24496, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)11522, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_start_stop_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)230, PH.base.pack) ;
        p66_req_message_rate_SET((uint16_t)(uint16_t)6522, PH.base.pack) ;
        p66_target_system_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_STREAM_67(), &PH);
        p67_on_off_SET((uint8_t)(uint8_t)133, PH.base.pack) ;
        p67_message_rate_SET((uint16_t)(uint16_t)40434, PH.base.pack) ;
        p67_stream_id_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_r_SET((int16_t)(int16_t) -26257, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t) -12853, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t)32099, PH.base.pack) ;
        p69_buttons_SET((uint16_t)(uint16_t)21515, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)98, PH.base.pack) ;
        p69_z_SET((int16_t)(int16_t)30517, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_target_system_SET((uint8_t)(uint8_t)168, PH.base.pack) ;
        p70_chan7_raw_SET((uint16_t)(uint16_t)22745, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)52683, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)49675, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)21379, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)19407, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)29257, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)28033, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)157, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)1278, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_autocontinue_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        p73_y_SET((int32_t)1786577647, PH.base.pack) ;
        p73_param4_SET((float)6.853591E37F, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p73_x_SET((int32_t)667471254, PH.base.pack) ;
        p73_param3_SET((float)5.4893464E37F, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)60802, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, PH.base.pack) ;
        p73_param1_SET((float)2.6316836E38F, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p73_param2_SET((float)2.6176942E38F, PH.base.pack) ;
        p73_command_SET(e_MAV_CMD_MAV_CMD_PANORAMA_CREATE, PH.base.pack) ;
        p73_z_SET((float)9.128472E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VFR_HUD_74(), &PH);
        p74_airspeed_SET((float)1.0901303E38F, PH.base.pack) ;
        p74_groundspeed_SET((float)1.601091E38F, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)44351, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t) -26111, PH.base.pack) ;
        p74_climb_SET((float)3.0346442E38F, PH.base.pack) ;
        p74_alt_SET((float) -1.7880642E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_INT_75(), &PH);
        p75_z_SET((float) -3.1335564E38F, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_OFFSET_NED, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p75_param3_SET((float)1.8925975E38F, PH.base.pack) ;
        p75_y_SET((int32_t) -240960629, PH.base.pack) ;
        p75_x_SET((int32_t)730675154, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        p75_command_SET(e_MAV_CMD_MAV_CMD_UAVCAN_GET_NODE_INFO, PH.base.pack) ;
        p75_param2_SET((float) -2.0212226E37F, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        p75_param4_SET((float)9.050858E37F, PH.base.pack) ;
        p75_param1_SET((float) -1.159435E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_LONG_76(), &PH);
        p76_param3_SET((float)1.2937754E38F, PH.base.pack) ;
        p76_param7_SET((float)1.6666054E38F, PH.base.pack) ;
        p76_param6_SET((float) -1.751839E38F, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        p76_param4_SET((float) -7.826144E36F, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_CONDITION_GATE, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        p76_param5_SET((float)2.7045242E38F, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)89, PH.base.pack) ;
        p76_param1_SET((float)2.6404622E37F, PH.base.pack) ;
        p76_param2_SET((float) -7.640786E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_ACK_77(), &PH);
        p77_target_system_SET((uint8_t)(uint8_t)54, &PH) ;
        p77_result_param2_SET((int32_t)1602185094, &PH) ;
        p77_progress_SET((uint8_t)(uint8_t)47, &PH) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_IN_PROGRESS, PH.base.pack) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_DO_CHANGE_ALTITUDE, PH.base.pack) ;
        p77_target_component_SET((uint8_t)(uint8_t)84, &PH) ;
        c_LoopBackDemoChannel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_manual_override_switch_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        p81_roll_SET((float)1.701262E38F, PH.base.pack) ;
        p81_thrust_SET((float) -1.5059035E38F, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)1880885260L, PH.base.pack) ;
        p81_pitch_SET((float)2.0009218E38F, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        p81_yaw_SET((float)2.6793304E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        {
            float q[] =  {-3.2035435E38F, -1.4750304E38F, 1.9951666E38F, -2.5166097E38F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_body_yaw_rate_SET((float) -1.8479952E38F, PH.base.pack) ;
        p82_body_pitch_rate_SET((float) -1.2862223E38F, PH.base.pack) ;
        p82_target_system_SET((uint8_t)(uint8_t)243, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p82_thrust_SET((float) -4.3332516E37F, PH.base.pack) ;
        p82_time_boot_ms_SET((uint32_t)3994634529L, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        p82_body_roll_rate_SET((float)1.5863066E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_body_roll_rate_SET((float)2.6378551E38F, PH.base.pack) ;
        p83_type_mask_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        p83_thrust_SET((float)2.7415995E38F, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)3797340474L, PH.base.pack) ;
        {
            float q[] =  {2.5391188E38F, -3.3804402E38F, -3.179131E38F, -3.303055E38F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_body_pitch_rate_SET((float) -8.731666E37F, PH.base.pack) ;
        p83_body_yaw_rate_SET((float)3.054227E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_target_component_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p84_z_SET((float)1.0027836E38F, PH.base.pack) ;
        p84_x_SET((float)3.0581614E38F, PH.base.pack) ;
        p84_afx_SET((float)2.8999838E38F, PH.base.pack) ;
        p84_afz_SET((float)2.263736E38F, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p84_y_SET((float)2.4183901E38F, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)184, PH.base.pack) ;
        p84_vy_SET((float)1.2254845E38F, PH.base.pack) ;
        p84_afy_SET((float) -1.2011093E38F, PH.base.pack) ;
        p84_vx_SET((float) -8.731988E37F, PH.base.pack) ;
        p84_vz_SET((float) -1.8783422E38F, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)324411328L, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)19936, PH.base.pack) ;
        p84_yaw_rate_SET((float) -1.6841942E38F, PH.base.pack) ;
        p84_yaw_SET((float) -1.2873433E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_afy_SET((float) -2.9924586E38F, PH.base.pack) ;
        p86_afz_SET((float)1.1310283E38F, PH.base.pack) ;
        p86_yaw_SET((float)1.2824098E38F, PH.base.pack) ;
        p86_type_mask_SET((uint16_t)(uint16_t)23128, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
        p86_lat_int_SET((int32_t) -1152326183, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)2111373414L, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        p86_vz_SET((float)1.2446145E38F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        p86_yaw_rate_SET((float) -2.1700945E38F, PH.base.pack) ;
        p86_vy_SET((float)1.074402E37F, PH.base.pack) ;
        p86_afx_SET((float) -2.5147131E38F, PH.base.pack) ;
        p86_alt_SET((float) -1.4898584E38F, PH.base.pack) ;
        p86_lon_int_SET((int32_t)336065083, PH.base.pack) ;
        p86_vx_SET((float)4.3437564E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p87_lat_int_SET((int32_t) -1584723288, PH.base.pack) ;
        p87_yaw_rate_SET((float) -3.2737456E38F, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)671629333L, PH.base.pack) ;
        p87_lon_int_SET((int32_t)733049159, PH.base.pack) ;
        p87_alt_SET((float) -3.120065E38F, PH.base.pack) ;
        p87_afz_SET((float)1.6933898E38F, PH.base.pack) ;
        p87_vz_SET((float) -2.1526362E38F, PH.base.pack) ;
        p87_vx_SET((float)1.9668264E38F, PH.base.pack) ;
        p87_vy_SET((float)1.4873251E38F, PH.base.pack) ;
        p87_type_mask_SET((uint16_t)(uint16_t)55699, PH.base.pack) ;
        p87_afy_SET((float) -2.498859E37F, PH.base.pack) ;
        p87_yaw_SET((float) -1.6508457E38F, PH.base.pack) ;
        p87_afx_SET((float)2.6753733E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_yaw_SET((float)8.15647E37F, PH.base.pack) ;
        p89_roll_SET((float) -2.4595935E38F, PH.base.pack) ;
        p89_z_SET((float) -1.8038037E38F, PH.base.pack) ;
        p89_pitch_SET((float) -7.9969136E36F, PH.base.pack) ;
        p89_x_SET((float) -1.2040753E38F, PH.base.pack) ;
        p89_y_SET((float) -1.2498022E38F, PH.base.pack) ;
        p89_time_boot_ms_SET((uint32_t)647235651L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_90(), &PH);
        p90_alt_SET((int32_t) -1306726532, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t)18508, PH.base.pack) ;
        p90_yaw_SET((float) -4.0911268E37F, PH.base.pack) ;
        p90_pitchspeed_SET((float) -8.696113E37F, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t)22312, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t) -9733, PH.base.pack) ;
        p90_yawspeed_SET((float)2.9028805E38F, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)7754029865813712202L, PH.base.pack) ;
        p90_roll_SET((float)4.450275E37F, PH.base.pack) ;
        p90_lat_SET((int32_t)2050340792, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t) -32524, PH.base.pack) ;
        p90_pitch_SET((float) -3.2167782E38F, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t)14466, PH.base.pack) ;
        p90_xacc_SET((int16_t)(int16_t)8987, PH.base.pack) ;
        p90_lon_SET((int32_t)1022966771, PH.base.pack) ;
        p90_rollspeed_SET((float)1.7351014E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_CONTROLS_91(), &PH);
        p91_throttle_SET((float) -8.4228684E37F, PH.base.pack) ;
        p91_aux3_SET((float)1.6017463E38F, PH.base.pack) ;
        p91_roll_ailerons_SET((float)1.9149588E38F, PH.base.pack) ;
        p91_pitch_elevator_SET((float)2.780021E38F, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)496895142985832945L, PH.base.pack) ;
        p91_aux4_SET((float) -1.9649028E38F, PH.base.pack) ;
        p91_aux1_SET((float)1.4516475E38F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_ARMED, PH.base.pack) ;
        p91_yaw_rudder_SET((float) -2.9663032E38F, PH.base.pack) ;
        p91_aux2_SET((float)2.346589E38F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_chan2_raw_SET((uint16_t)(uint16_t)11664, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)2297, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)2084339140964312961L, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)40286, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)10513, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)14975, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)41312, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)13407, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)9809, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)47161, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)42568, PH.base.pack) ;
        p92_rssi_SET((uint8_t)(uint8_t)208, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)27985, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)9444, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_time_usec_SET((uint64_t)2567488371794832959L, PH.base.pack) ;
        p93_mode_SET(e_MAV_MODE_MAV_MODE_PREFLIGHT, PH.base.pack) ;
        {
            float controls[] =  {-3.195126E38F, -1.5915898E37F, -2.2240114E38F, 1.9668976E38F, -1.4824636E38F, 2.0263243E38F, 1.1031579E38F, -1.0098248E38F, 1.4833578E38F, 1.8883102E38F, 3.5822739E37F, 3.3593745E37F, 3.3614632E38F, 3.247691E38F, 2.4371155E38F, -1.0000517E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p93_flags_SET((uint64_t)4560366735783475588L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_quality_SET((uint8_t)(uint8_t)64, PH.base.pack) ;
        p100_flow_comp_m_x_SET((float) -2.6348791E38F, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)224374894770255707L, PH.base.pack) ;
        p100_flow_rate_x_SET((float) -2.3237506E38F, &PH) ;
        p100_flow_comp_m_y_SET((float)5.7506407E37F, PH.base.pack) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)112, PH.base.pack) ;
        p100_flow_x_SET((int16_t)(int16_t) -14798, PH.base.pack) ;
        p100_ground_distance_SET((float)2.8139557E38F, PH.base.pack) ;
        p100_flow_rate_y_SET((float) -4.71217E37F, &PH) ;
        p100_flow_y_SET((int16_t)(int16_t) -8275, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_x_SET((float) -8.828578E37F, PH.base.pack) ;
        p101_yaw_SET((float)1.7055835E38F, PH.base.pack) ;
        p101_roll_SET((float) -5.929979E37F, PH.base.pack) ;
        p101_y_SET((float) -1.1378702E38F, PH.base.pack) ;
        p101_pitch_SET((float) -2.222416E38F, PH.base.pack) ;
        p101_usec_SET((uint64_t)7553001424720102822L, PH.base.pack) ;
        p101_z_SET((float)1.6986359E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_usec_SET((uint64_t)6789320262507273684L, PH.base.pack) ;
        p102_yaw_SET((float) -2.6006246E38F, PH.base.pack) ;
        p102_z_SET((float)2.1888509E38F, PH.base.pack) ;
        p102_pitch_SET((float) -1.1842903E38F, PH.base.pack) ;
        p102_roll_SET((float) -1.4586123E37F, PH.base.pack) ;
        p102_x_SET((float)3.2330455E38F, PH.base.pack) ;
        p102_y_SET((float) -2.4108682E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_x_SET((float)1.198924E38F, PH.base.pack) ;
        p103_z_SET((float) -1.6798537E38F, PH.base.pack) ;
        p103_usec_SET((uint64_t)1467612833915047019L, PH.base.pack) ;
        p103_y_SET((float)1.2408964E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_z_SET((float) -1.6172556E38F, PH.base.pack) ;
        p104_yaw_SET((float) -2.7334092E38F, PH.base.pack) ;
        p104_y_SET((float) -1.5625364E38F, PH.base.pack) ;
        p104_pitch_SET((float)1.2081625E38F, PH.base.pack) ;
        p104_x_SET((float)7.3203E37F, PH.base.pack) ;
        p104_roll_SET((float)6.657163E37F, PH.base.pack) ;
        p104_usec_SET((uint64_t)4384105150636666823L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGHRES_IMU_105(), &PH);
        p105_fields_updated_SET((uint16_t)(uint16_t)5790, PH.base.pack) ;
        p105_ygyro_SET((float)5.251862E37F, PH.base.pack) ;
        p105_diff_pressure_SET((float)1.2877676E37F, PH.base.pack) ;
        p105_xacc_SET((float)4.1363335E37F, PH.base.pack) ;
        p105_zacc_SET((float)2.0557952E37F, PH.base.pack) ;
        p105_xgyro_SET((float)1.1101039E38F, PH.base.pack) ;
        p105_ymag_SET((float)1.4276351E38F, PH.base.pack) ;
        p105_zmag_SET((float)1.800658E37F, PH.base.pack) ;
        p105_pressure_alt_SET((float)1.734111E38F, PH.base.pack) ;
        p105_xmag_SET((float) -2.7177632E38F, PH.base.pack) ;
        p105_temperature_SET((float) -2.6582674E38F, PH.base.pack) ;
        p105_yacc_SET((float)1.5959531E37F, PH.base.pack) ;
        p105_zgyro_SET((float) -5.7946043E37F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)8549239154874540207L, PH.base.pack) ;
        p105_abs_pressure_SET((float)2.6310174E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_integrated_xgyro_SET((float) -3.2413966E38F, PH.base.pack) ;
        p106_distance_SET((float) -1.2694026E38F, PH.base.pack) ;
        p106_integrated_zgyro_SET((float) -2.6147208E38F, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)249566869L, PH.base.pack) ;
        p106_integrated_y_SET((float)1.3874028E38F, PH.base.pack) ;
        p106_integrated_x_SET((float) -1.2229004E38F, PH.base.pack) ;
        p106_integrated_ygyro_SET((float)4.8436843E37F, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)1928384203348573103L, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t) -11160, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        p106_quality_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)1993724259L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_SENSOR_107(), &PH);
        p107_xacc_SET((float) -4.960933E37F, PH.base.pack) ;
        p107_zacc_SET((float)1.6997747E38F, PH.base.pack) ;
        p107_zmag_SET((float)1.7889254E38F, PH.base.pack) ;
        p107_diff_pressure_SET((float) -1.8730957E37F, PH.base.pack) ;
        p107_xgyro_SET((float)6.67273E37F, PH.base.pack) ;
        p107_xmag_SET((float)1.7001716E38F, PH.base.pack) ;
        p107_ymag_SET((float) -1.911557E37F, PH.base.pack) ;
        p107_yacc_SET((float)3.1234367E38F, PH.base.pack) ;
        p107_temperature_SET((float) -2.667241E38F, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)2827801811L, PH.base.pack) ;
        p107_ygyro_SET((float)8.557706E36F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)1156853484184566798L, PH.base.pack) ;
        p107_abs_pressure_SET((float)1.9825268E38F, PH.base.pack) ;
        p107_zgyro_SET((float)2.6672314E38F, PH.base.pack) ;
        p107_pressure_alt_SET((float) -5.099245E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SIM_STATE_108(), &PH);
        p108_vd_SET((float)3.1609762E38F, PH.base.pack) ;
        p108_std_dev_vert_SET((float)2.7210439E38F, PH.base.pack) ;
        p108_xgyro_SET((float)7.688713E37F, PH.base.pack) ;
        p108_vn_SET((float)1.8383345E38F, PH.base.pack) ;
        p108_ygyro_SET((float) -3.7916682E37F, PH.base.pack) ;
        p108_zacc_SET((float) -1.0167349E38F, PH.base.pack) ;
        p108_pitch_SET((float) -2.1460931E38F, PH.base.pack) ;
        p108_ve_SET((float) -4.369593E37F, PH.base.pack) ;
        p108_yacc_SET((float)1.3322755E38F, PH.base.pack) ;
        p108_q4_SET((float)2.5158098E38F, PH.base.pack) ;
        p108_yaw_SET((float)4.998108E37F, PH.base.pack) ;
        p108_zgyro_SET((float) -1.4597469E38F, PH.base.pack) ;
        p108_lat_SET((float)1.4763811E38F, PH.base.pack) ;
        p108_q3_SET((float)2.8864575E38F, PH.base.pack) ;
        p108_lon_SET((float) -9.317673E37F, PH.base.pack) ;
        p108_q2_SET((float) -2.0184752E38F, PH.base.pack) ;
        p108_xacc_SET((float) -2.5644841E38F, PH.base.pack) ;
        p108_roll_SET((float) -2.824069E37F, PH.base.pack) ;
        p108_std_dev_horz_SET((float) -7.6393483E37F, PH.base.pack) ;
        p108_q1_SET((float) -2.92444E38F, PH.base.pack) ;
        p108_alt_SET((float)1.2883949E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RADIO_STATUS_109(), &PH);
        p109_rxerrors_SET((uint16_t)(uint16_t)30645, PH.base.pack) ;
        p109_remnoise_SET((uint8_t)(uint8_t)226, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)0, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)59, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)41708, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        p110_target_network_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        p110_target_component_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        p110_target_system_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)141, (uint8_t)187, (uint8_t)212, (uint8_t)128, (uint8_t)221, (uint8_t)1, (uint8_t)202, (uint8_t)204, (uint8_t)145, (uint8_t)124, (uint8_t)25, (uint8_t)14, (uint8_t)62, (uint8_t)221, (uint8_t)111, (uint8_t)178, (uint8_t)188, (uint8_t)13, (uint8_t)96, (uint8_t)182, (uint8_t)163, (uint8_t)72, (uint8_t)249, (uint8_t)28, (uint8_t)109, (uint8_t)171, (uint8_t)83, (uint8_t)1, (uint8_t)231, (uint8_t)112, (uint8_t)12, (uint8_t)7, (uint8_t)208, (uint8_t)182, (uint8_t)242, (uint8_t)63, (uint8_t)131, (uint8_t)62, (uint8_t)240, (uint8_t)84, (uint8_t)229, (uint8_t)101, (uint8_t)20, (uint8_t)175, (uint8_t)150, (uint8_t)111, (uint8_t)150, (uint8_t)88, (uint8_t)138, (uint8_t)162, (uint8_t)81, (uint8_t)229, (uint8_t)154, (uint8_t)17, (uint8_t)210, (uint8_t)116, (uint8_t)162, (uint8_t)30, (uint8_t)183, (uint8_t)33, (uint8_t)243, (uint8_t)93, (uint8_t)144, (uint8_t)227, (uint8_t)25, (uint8_t)1, (uint8_t)196, (uint8_t)24, (uint8_t)95, (uint8_t)180, (uint8_t)96, (uint8_t)36, (uint8_t)197, (uint8_t)182, (uint8_t)27, (uint8_t)166, (uint8_t)122, (uint8_t)118, (uint8_t)46, (uint8_t)43, (uint8_t)131, (uint8_t)152, (uint8_t)254, (uint8_t)72, (uint8_t)147, (uint8_t)108, (uint8_t)72, (uint8_t)214, (uint8_t)64, (uint8_t)243, (uint8_t)141, (uint8_t)231, (uint8_t)199, (uint8_t)169, (uint8_t)113, (uint8_t)254, (uint8_t)49, (uint8_t)216, (uint8_t)201, (uint8_t)135, (uint8_t)193, (uint8_t)216, (uint8_t)53, (uint8_t)80, (uint8_t)17, (uint8_t)177, (uint8_t)80, (uint8_t)161, (uint8_t)126, (uint8_t)132, (uint8_t)1, (uint8_t)135, (uint8_t)232, (uint8_t)183, (uint8_t)172, (uint8_t)56, (uint8_t)188, (uint8_t)20, (uint8_t)179, (uint8_t)104, (uint8_t)250, (uint8_t)197, (uint8_t)218, (uint8_t)203, (uint8_t)131, (uint8_t)15, (uint8_t)208, (uint8_t)144, (uint8_t)170, (uint8_t)62, (uint8_t)105, (uint8_t)6, (uint8_t)158, (uint8_t)38, (uint8_t)94, (uint8_t)44, (uint8_t)172, (uint8_t)172, (uint8_t)174, (uint8_t)185, (uint8_t)142, (uint8_t)115, (uint8_t)179, (uint8_t)22, (uint8_t)22, (uint8_t)1, (uint8_t)55, (uint8_t)241, (uint8_t)64, (uint8_t)89, (uint8_t)175, (uint8_t)26, (uint8_t)248, (uint8_t)253, (uint8_t)46, (uint8_t)242, (uint8_t)162, (uint8_t)109, (uint8_t)48, (uint8_t)20, (uint8_t)41, (uint8_t)136, (uint8_t)174, (uint8_t)70, (uint8_t)188, (uint8_t)142, (uint8_t)229, (uint8_t)119, (uint8_t)107, (uint8_t)4, (uint8_t)255, (uint8_t)58, (uint8_t)249, (uint8_t)150, (uint8_t)183, (uint8_t)254, (uint8_t)218, (uint8_t)151, (uint8_t)77, (uint8_t)1, (uint8_t)14, (uint8_t)145, (uint8_t)148, (uint8_t)5, (uint8_t)12, (uint8_t)183, (uint8_t)47, (uint8_t)130, (uint8_t)46, (uint8_t)247, (uint8_t)160, (uint8_t)32, (uint8_t)167, (uint8_t)57, (uint8_t)244, (uint8_t)41, (uint8_t)57, (uint8_t)15, (uint8_t)54, (uint8_t)110, (uint8_t)11, (uint8_t)14, (uint8_t)62, (uint8_t)237, (uint8_t)139, (uint8_t)40, (uint8_t)242, (uint8_t)111, (uint8_t)28, (uint8_t)160, (uint8_t)200, (uint8_t)92, (uint8_t)15, (uint8_t)225, (uint8_t)232, (uint8_t)31, (uint8_t)15, (uint8_t)234, (uint8_t)123, (uint8_t)87, (uint8_t)203, (uint8_t)41, (uint8_t)147, (uint8_t)69, (uint8_t)250, (uint8_t)64, (uint8_t)211, (uint8_t)225, (uint8_t)128, (uint8_t)122, (uint8_t)168, (uint8_t)62, (uint8_t)124, (uint8_t)152, (uint8_t)213, (uint8_t)236, (uint8_t)124, (uint8_t)0, (uint8_t)128, (uint8_t)5, (uint8_t)75, (uint8_t)239, (uint8_t)66, (uint8_t)158, (uint8_t)158, (uint8_t)4, (uint8_t)121, (uint8_t)174, (uint8_t)139, (uint8_t)247, (uint8_t)111};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TIMESYNC_111(), &PH);
        p111_tc1_SET((int64_t)9066531987965081276L, PH.base.pack) ;
        p111_ts1_SET((int64_t) -6820710341724442267L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_time_usec_SET((uint64_t)3443581684130125324L, PH.base.pack) ;
        p112_seq_SET((uint32_t)349047401L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_GPS_113(), &PH);
        p113_vn_SET((int16_t)(int16_t)16464, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)13821, PH.base.pack) ;
        p113_lat_SET((int32_t)1526854778, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)3392908150088189275L, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)25159, PH.base.pack) ;
        p113_alt_SET((int32_t) -720631257, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t)2826, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t) -18683, PH.base.pack) ;
        p113_lon_SET((int32_t)1751233755, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)61019, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)2636, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_distance_SET((float)4.834069E37F, PH.base.pack) ;
        p114_integrated_zgyro_SET((float)8.473334E37F, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)4528934574512529667L, PH.base.pack) ;
        p114_integrated_ygyro_SET((float)1.9288184E38F, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t) -12361, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)163, PH.base.pack) ;
        p114_integrated_xgyro_SET((float)2.773049E38F, PH.base.pack) ;
        p114_integrated_y_SET((float)1.0994803E38F, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)864484921L, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)2596061684L, PH.base.pack) ;
        p114_integrated_x_SET((float) -2.5556063E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_yacc_SET((int16_t)(int16_t) -13655, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t) -30444, PH.base.pack) ;
        p115_alt_SET((int32_t)733224073, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {2.995407E38F, -2.2303807E38F, 3.0797527E38F, 1.942452E38F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_lon_SET((int32_t) -792468347, PH.base.pack) ;
        p115_yawspeed_SET((float) -2.086654E38F, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)46693, PH.base.pack) ;
        p115_rollspeed_SET((float)7.416648E37F, PH.base.pack) ;
        p115_vy_SET((int16_t)(int16_t) -19043, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)9970, PH.base.pack) ;
        p115_lat_SET((int32_t) -1045094055, PH.base.pack) ;
        p115_pitchspeed_SET((float) -2.2947202E38F, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t)367, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t)32747, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)1976304181846714897L, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t)5883, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU2_116(), &PH);
        p116_ymag_SET((int16_t)(int16_t) -8966, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)3183606508L, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t) -32006, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t)28847, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t)29657, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t)23735, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t)25822, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t) -29599, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t)43, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t) -5593, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_end_SET((uint16_t)(uint16_t)29780, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        p117_start_SET((uint16_t)(uint16_t)43935, PH.base.pack) ;
        p117_target_component_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ENTRY_118(), &PH);
        p118_size_SET((uint32_t)1169602637L, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)2891848756L, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)15658, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)31181, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)17143, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_id_SET((uint16_t)(uint16_t)7410, PH.base.pack) ;
        p119_count_SET((uint32_t)4094678952L, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)154, PH.base.pack) ;
        p119_ofs_SET((uint32_t)2027232737L, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_DATA_120(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)60, (uint8_t)40, (uint8_t)240, (uint8_t)50, (uint8_t)17, (uint8_t)98, (uint8_t)97, (uint8_t)62, (uint8_t)124, (uint8_t)52, (uint8_t)112, (uint8_t)110, (uint8_t)20, (uint8_t)92, (uint8_t)199, (uint8_t)230, (uint8_t)229, (uint8_t)243, (uint8_t)173, (uint8_t)232, (uint8_t)40, (uint8_t)27, (uint8_t)254, (uint8_t)81, (uint8_t)36, (uint8_t)170, (uint8_t)150, (uint8_t)53, (uint8_t)75, (uint8_t)3, (uint8_t)199, (uint8_t)139, (uint8_t)66, (uint8_t)74, (uint8_t)134, (uint8_t)69, (uint8_t)87, (uint8_t)114, (uint8_t)226, (uint8_t)202, (uint8_t)228, (uint8_t)8, (uint8_t)43, (uint8_t)34, (uint8_t)192, (uint8_t)135, (uint8_t)35, (uint8_t)54, (uint8_t)161, (uint8_t)245, (uint8_t)52, (uint8_t)43, (uint8_t)74, (uint8_t)134, (uint8_t)196, (uint8_t)29, (uint8_t)10, (uint8_t)144, (uint8_t)139, (uint8_t)195, (uint8_t)168, (uint8_t)182, (uint8_t)251, (uint8_t)103, (uint8_t)59, (uint8_t)81, (uint8_t)162, (uint8_t)205, (uint8_t)109, (uint8_t)139, (uint8_t)110, (uint8_t)28, (uint8_t)242, (uint8_t)127, (uint8_t)239, (uint8_t)200, (uint8_t)134, (uint8_t)23, (uint8_t)125, (uint8_t)25, (uint8_t)130, (uint8_t)27, (uint8_t)32, (uint8_t)206, (uint8_t)14, (uint8_t)63, (uint8_t)76, (uint8_t)175, (uint8_t)65, (uint8_t)27};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        p120_count_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        p120_ofs_SET((uint32_t)2310129942L, PH.base.pack) ;
        p120_id_SET((uint16_t)(uint16_t)9325, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ERASE_121(), &PH);
        p121_target_system_SET((uint8_t)(uint8_t)138, PH.base.pack) ;
        p121_target_component_SET((uint8_t)(uint8_t)183, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_component_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        p122_target_system_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INJECT_DATA_123(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)216, (uint8_t)206, (uint8_t)244, (uint8_t)12, (uint8_t)168, (uint8_t)165, (uint8_t)191, (uint8_t)210, (uint8_t)162, (uint8_t)251, (uint8_t)140, (uint8_t)192, (uint8_t)63, (uint8_t)182, (uint8_t)74, (uint8_t)193, (uint8_t)83, (uint8_t)210, (uint8_t)111, (uint8_t)164, (uint8_t)111, (uint8_t)26, (uint8_t)68, (uint8_t)8, (uint8_t)44, (uint8_t)72, (uint8_t)237, (uint8_t)144, (uint8_t)167, (uint8_t)63, (uint8_t)227, (uint8_t)35, (uint8_t)219, (uint8_t)127, (uint8_t)151, (uint8_t)189, (uint8_t)96, (uint8_t)171, (uint8_t)229, (uint8_t)83, (uint8_t)228, (uint8_t)220, (uint8_t)130, (uint8_t)137, (uint8_t)20, (uint8_t)141, (uint8_t)167, (uint8_t)107, (uint8_t)20, (uint8_t)103, (uint8_t)29, (uint8_t)115, (uint8_t)102, (uint8_t)156, (uint8_t)44, (uint8_t)88, (uint8_t)236, (uint8_t)235, (uint8_t)204, (uint8_t)226, (uint8_t)212, (uint8_t)85, (uint8_t)47, (uint8_t)22, (uint8_t)43, (uint8_t)100, (uint8_t)153, (uint8_t)29, (uint8_t)15, (uint8_t)10, (uint8_t)185, (uint8_t)83, (uint8_t)239, (uint8_t)82, (uint8_t)75, (uint8_t)93, (uint8_t)188, (uint8_t)144, (uint8_t)128, (uint8_t)86, (uint8_t)92, (uint8_t)240, (uint8_t)72, (uint8_t)68, (uint8_t)72, (uint8_t)148, (uint8_t)149, (uint8_t)45, (uint8_t)97, (uint8_t)14, (uint8_t)21, (uint8_t)166, (uint8_t)202, (uint8_t)255, (uint8_t)31, (uint8_t)3, (uint8_t)10, (uint8_t)133, (uint8_t)227, (uint8_t)238, (uint8_t)188, (uint8_t)236, (uint8_t)83, (uint8_t)133, (uint8_t)54, (uint8_t)224, (uint8_t)152, (uint8_t)247, (uint8_t)187, (uint8_t)95};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        p123_len_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p123_target_component_SET((uint8_t)(uint8_t)71, PH.base.pack) ;
        p123_target_system_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RAW_124(), &PH);
        p124_dgps_age_SET((uint32_t)3375734272L, PH.base.pack) ;
        p124_lon_SET((int32_t) -1170909397, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        p124_alt_SET((int32_t) -101596466, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)42416, PH.base.pack) ;
        p124_lat_SET((int32_t) -329473392, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)63793, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_3D_FIX, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)21396, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)6145796054812901367L, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)5258, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POWER_STATUS_125(), &PH);
        p125_Vcc_SET((uint16_t)(uint16_t)52771, PH.base.pack) ;
        p125_Vservo_SET((uint16_t)(uint16_t)65319, PH.base.pack) ;
        p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_CONTROL_126(), &PH);
        {
            uint8_t data_[] =  {(uint8_t)136, (uint8_t)12, (uint8_t)147, (uint8_t)249, (uint8_t)6, (uint8_t)200, (uint8_t)7, (uint8_t)2, (uint8_t)15, (uint8_t)106, (uint8_t)242, (uint8_t)3, (uint8_t)18, (uint8_t)92, (uint8_t)60, (uint8_t)123, (uint8_t)56, (uint8_t)221, (uint8_t)165, (uint8_t)114, (uint8_t)62, (uint8_t)58, (uint8_t)226, (uint8_t)141, (uint8_t)253, (uint8_t)125, (uint8_t)9, (uint8_t)39, (uint8_t)245, (uint8_t)97, (uint8_t)50, (uint8_t)225, (uint8_t)26, (uint8_t)22, (uint8_t)128, (uint8_t)54, (uint8_t)72, (uint8_t)41, (uint8_t)193, (uint8_t)78, (uint8_t)80, (uint8_t)48, (uint8_t)19, (uint8_t)190, (uint8_t)42, (uint8_t)206, (uint8_t)169, (uint8_t)33, (uint8_t)83, (uint8_t)51, (uint8_t)110, (uint8_t)228, (uint8_t)158, (uint8_t)1, (uint8_t)210, (uint8_t)222, (uint8_t)129, (uint8_t)128, (uint8_t)82, (uint8_t)51, (uint8_t)149, (uint8_t)199, (uint8_t)174, (uint8_t)135, (uint8_t)41, (uint8_t)157, (uint8_t)129, (uint8_t)6, (uint8_t)200, (uint8_t)5};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_REPLY, PH.base.pack) ;
        p126_baudrate_SET((uint32_t)932605462L, PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)142, PH.base.pack) ;
        p126_timeout_SET((uint16_t)(uint16_t)34380, PH.base.pack) ;
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_GPS2, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTK_127(), &PH);
        p127_nsats_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t)2025317357, PH.base.pack) ;
        p127_tow_SET((uint32_t)3135574598L, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)63, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t)2109668784, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t)617901110, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)195, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t)1013415676, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)2478974838L, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)54723, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)4172940816L, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RTK_128(), &PH);
        p128_baseline_b_mm_SET((int32_t) -385074998, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t) -19308133, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)244, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)2703771080L, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)2384543303L, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)30376, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t) -597040842, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t)2074452508, PH.base.pack) ;
        p128_tow_SET((uint32_t)804055881L, PH.base.pack) ;
        p128_rtk_health_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU3_129(), &PH);
        p129_zacc_SET((int16_t)(int16_t)32532, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t) -14533, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t)20180, PH.base.pack) ;
        p129_yacc_SET((int16_t)(int16_t) -12880, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t)31048, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)4145135430L, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t) -16214, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t) -19394, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t)10783, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t) -18826, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_type_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)12779, PH.base.pack) ;
        p130_jpg_quality_SET((uint8_t)(uint8_t)254, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)44264, PH.base.pack) ;
        p130_size_SET((uint32_t)3028665782L, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)221, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)41126, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ENCAPSULATED_DATA_131(), &PH);
        p131_seqnr_SET((uint16_t)(uint16_t)21163, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)110, (uint8_t)35, (uint8_t)170, (uint8_t)90, (uint8_t)80, (uint8_t)244, (uint8_t)177, (uint8_t)164, (uint8_t)117, (uint8_t)29, (uint8_t)122, (uint8_t)244, (uint8_t)138, (uint8_t)34, (uint8_t)177, (uint8_t)159, (uint8_t)253, (uint8_t)87, (uint8_t)182, (uint8_t)134, (uint8_t)11, (uint8_t)228, (uint8_t)147, (uint8_t)171, (uint8_t)219, (uint8_t)223, (uint8_t)101, (uint8_t)141, (uint8_t)253, (uint8_t)42, (uint8_t)12, (uint8_t)109, (uint8_t)5, (uint8_t)137, (uint8_t)24, (uint8_t)70, (uint8_t)148, (uint8_t)63, (uint8_t)149, (uint8_t)106, (uint8_t)245, (uint8_t)156, (uint8_t)13, (uint8_t)205, (uint8_t)146, (uint8_t)149, (uint8_t)184, (uint8_t)26, (uint8_t)73, (uint8_t)226, (uint8_t)98, (uint8_t)136, (uint8_t)149, (uint8_t)229, (uint8_t)248, (uint8_t)198, (uint8_t)185, (uint8_t)71, (uint8_t)233, (uint8_t)242, (uint8_t)119, (uint8_t)47, (uint8_t)233, (uint8_t)226, (uint8_t)231, (uint8_t)55, (uint8_t)79, (uint8_t)1, (uint8_t)248, (uint8_t)148, (uint8_t)89, (uint8_t)86, (uint8_t)80, (uint8_t)131, (uint8_t)248, (uint8_t)227, (uint8_t)191, (uint8_t)114, (uint8_t)102, (uint8_t)118, (uint8_t)144, (uint8_t)227, (uint8_t)104, (uint8_t)100, (uint8_t)250, (uint8_t)197, (uint8_t)23, (uint8_t)61, (uint8_t)130, (uint8_t)103, (uint8_t)218, (uint8_t)49, (uint8_t)49, (uint8_t)45, (uint8_t)2, (uint8_t)220, (uint8_t)81, (uint8_t)28, (uint8_t)190, (uint8_t)168, (uint8_t)126, (uint8_t)127, (uint8_t)51, (uint8_t)9, (uint8_t)246, (uint8_t)4, (uint8_t)52, (uint8_t)187, (uint8_t)174, (uint8_t)191, (uint8_t)205, (uint8_t)207, (uint8_t)251, (uint8_t)255, (uint8_t)39, (uint8_t)253, (uint8_t)142, (uint8_t)156, (uint8_t)219, (uint8_t)85, (uint8_t)60, (uint8_t)102, (uint8_t)123, (uint8_t)58, (uint8_t)11, (uint8_t)176, (uint8_t)50, (uint8_t)185, (uint8_t)101, (uint8_t)16, (uint8_t)208, (uint8_t)186, (uint8_t)214, (uint8_t)243, (uint8_t)163, (uint8_t)187, (uint8_t)171, (uint8_t)1, (uint8_t)115, (uint8_t)166, (uint8_t)239, (uint8_t)229, (uint8_t)41, (uint8_t)127, (uint8_t)203, (uint8_t)183, (uint8_t)50, (uint8_t)149, (uint8_t)36, (uint8_t)4, (uint8_t)21, (uint8_t)17, (uint8_t)150, (uint8_t)251, (uint8_t)113, (uint8_t)33, (uint8_t)231, (uint8_t)129, (uint8_t)166, (uint8_t)39, (uint8_t)52, (uint8_t)217, (uint8_t)167, (uint8_t)193, (uint8_t)25, (uint8_t)208, (uint8_t)85, (uint8_t)149, (uint8_t)196, (uint8_t)122, (uint8_t)59, (uint8_t)95, (uint8_t)63, (uint8_t)243, (uint8_t)145, (uint8_t)167, (uint8_t)153, (uint8_t)104, (uint8_t)203, (uint8_t)39, (uint8_t)117, (uint8_t)190, (uint8_t)111, (uint8_t)246, (uint8_t)105, (uint8_t)132, (uint8_t)111, (uint8_t)82, (uint8_t)5, (uint8_t)137, (uint8_t)171, (uint8_t)27, (uint8_t)54, (uint8_t)158, (uint8_t)8, (uint8_t)60, (uint8_t)51, (uint8_t)210, (uint8_t)209, (uint8_t)138, (uint8_t)202, (uint8_t)56, (uint8_t)3, (uint8_t)155, (uint8_t)203, (uint8_t)24, (uint8_t)156, (uint8_t)110, (uint8_t)190, (uint8_t)63, (uint8_t)70, (uint8_t)70, (uint8_t)224, (uint8_t)213, (uint8_t)250, (uint8_t)49, (uint8_t)194, (uint8_t)140, (uint8_t)155, (uint8_t)114, (uint8_t)78, (uint8_t)27, (uint8_t)5, (uint8_t)66, (uint8_t)198, (uint8_t)246, (uint8_t)162, (uint8_t)127, (uint8_t)82, (uint8_t)228, (uint8_t)110, (uint8_t)157, (uint8_t)44, (uint8_t)202, (uint8_t)180, (uint8_t)143, (uint8_t)72, (uint8_t)162, (uint8_t)99, (uint8_t)97, (uint8_t)239, (uint8_t)164, (uint8_t)161, (uint8_t)206, (uint8_t)42, (uint8_t)224, (uint8_t)188, (uint8_t)98, (uint8_t)157, (uint8_t)185, (uint8_t)79, (uint8_t)227, (uint8_t)163};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_max_distance_SET((uint16_t)(uint16_t)5755, PH.base.pack) ;
        p132_covariance_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)65504960L, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_YAW_315, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_RADAR, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)21164, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)34121, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)225, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_mask_SET((uint64_t)4753543093664333295L, PH.base.pack) ;
        p133_lat_SET((int32_t) -656045646, PH.base.pack) ;
        p133_lon_SET((int32_t)33139079, PH.base.pack) ;
        p133_grid_spacing_SET((uint16_t)(uint16_t)21479, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_DATA_134(), &PH);
        p134_gridbit_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p134_lon_SET((int32_t) -763027854, PH.base.pack) ;
        p134_lat_SET((int32_t) -678615857, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t)9324, (int16_t)17822, (int16_t)28338, (int16_t)23813, (int16_t) -18382, (int16_t)27800, (int16_t)31417, (int16_t)25516, (int16_t) -9585, (int16_t) -29775, (int16_t) -2033, (int16_t)9108, (int16_t)28559, (int16_t) -29421, (int16_t)32112, (int16_t)13993};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        p134_grid_spacing_SET((uint16_t)(uint16_t)18492, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lat_SET((int32_t)1918579710, PH.base.pack) ;
        p135_lon_SET((int32_t)1412630928, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_spacing_SET((uint16_t)(uint16_t)16403, PH.base.pack) ;
        p136_lon_SET((int32_t)1151929247, PH.base.pack) ;
        p136_terrain_height_SET((float) -2.2104123E38F, PH.base.pack) ;
        p136_current_height_SET((float) -2.9392735E38F, PH.base.pack) ;
        p136_lat_SET((int32_t) -99325165, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)52859, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)36187, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_time_boot_ms_SET((uint32_t)323508292L, PH.base.pack) ;
        p137_press_diff_SET((float) -1.6043337E38F, PH.base.pack) ;
        p137_press_abs_SET((float) -2.1846997E38F, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t) -11297, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATT_POS_MOCAP_138(), &PH);
        p138_x_SET((float) -2.6874846E38F, PH.base.pack) ;
        p138_z_SET((float)1.9977656E38F, PH.base.pack) ;
        p138_y_SET((float) -8.58887E37F, PH.base.pack) ;
        p138_time_usec_SET((uint64_t)3595484720603252898L, PH.base.pack) ;
        {
            float q[] =  {4.150003E37F, 1.4922869E38F, -9.276017E37F, -2.6111317E38F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        {
            float controls[] =  {-3.7707715E37F, 3.9901975E37F, -1.3565633E38F, -9.989574E37F, -2.9452225E38F, -2.1560366E38F, 6.420881E36F, 8.3435383E37F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p139_target_component_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p139_time_usec_SET((uint64_t)5316800483687197671L, PH.base.pack) ;
        p139_target_system_SET((uint8_t)(uint8_t)7, PH.base.pack) ;
        p139_group_mlx_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        p140_time_usec_SET((uint64_t)7630842519899568613L, PH.base.pack) ;
        p140_group_mlx_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        {
            float controls[] =  {-3.8085885E37F, -1.6549493E38F, 1.6059674E38F, -2.7815518E38F, 2.563886E38F, -2.450936E38F, -3.979575E36F, 3.3761543E38F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ALTITUDE_141(), &PH);
        p141_altitude_relative_SET((float) -9.699802E37F, PH.base.pack) ;
        p141_time_usec_SET((uint64_t)1642444103785961245L, PH.base.pack) ;
        p141_altitude_monotonic_SET((float) -2.6678275E38F, PH.base.pack) ;
        p141_bottom_clearance_SET((float)1.1445459E38F, PH.base.pack) ;
        p141_altitude_local_SET((float) -9.600342E37F, PH.base.pack) ;
        p141_altitude_terrain_SET((float) -1.9743274E35F, PH.base.pack) ;
        p141_altitude_amsl_SET((float)2.091532E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RESOURCE_REQUEST_142(), &PH);
        p142_request_id_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        {
            uint8_t uri[] =  {(uint8_t)56, (uint8_t)70, (uint8_t)112, (uint8_t)201, (uint8_t)65, (uint8_t)56, (uint8_t)63, (uint8_t)246, (uint8_t)123, (uint8_t)109, (uint8_t)240, (uint8_t)125, (uint8_t)214, (uint8_t)128, (uint8_t)90, (uint8_t)168, (uint8_t)147, (uint8_t)115, (uint8_t)192, (uint8_t)241, (uint8_t)89, (uint8_t)237, (uint8_t)102, (uint8_t)236, (uint8_t)173, (uint8_t)106, (uint8_t)97, (uint8_t)124, (uint8_t)190, (uint8_t)204, (uint8_t)10, (uint8_t)143, (uint8_t)131, (uint8_t)66, (uint8_t)27, (uint8_t)27, (uint8_t)241, (uint8_t)18, (uint8_t)219, (uint8_t)143, (uint8_t)73, (uint8_t)85, (uint8_t)2, (uint8_t)238, (uint8_t)208, (uint8_t)14, (uint8_t)46, (uint8_t)34, (uint8_t)29, (uint8_t)11, (uint8_t)22, (uint8_t)93, (uint8_t)56, (uint8_t)228, (uint8_t)126, (uint8_t)178, (uint8_t)98, (uint8_t)24, (uint8_t)4, (uint8_t)144, (uint8_t)52, (uint8_t)56, (uint8_t)213, (uint8_t)231, (uint8_t)73, (uint8_t)165, (uint8_t)159, (uint8_t)254, (uint8_t)111, (uint8_t)194, (uint8_t)224, (uint8_t)183, (uint8_t)74, (uint8_t)19, (uint8_t)114, (uint8_t)142, (uint8_t)6, (uint8_t)32, (uint8_t)104, (uint8_t)218, (uint8_t)131, (uint8_t)70, (uint8_t)146, (uint8_t)254, (uint8_t)108, (uint8_t)185, (uint8_t)154, (uint8_t)196, (uint8_t)223, (uint8_t)231, (uint8_t)179, (uint8_t)198, (uint8_t)79, (uint8_t)255, (uint8_t)223, (uint8_t)146, (uint8_t)214, (uint8_t)65, (uint8_t)174, (uint8_t)250, (uint8_t)245, (uint8_t)61, (uint8_t)224, (uint8_t)72, (uint8_t)84, (uint8_t)121, (uint8_t)33, (uint8_t)60, (uint8_t)134, (uint8_t)122, (uint8_t)57, (uint8_t)116, (uint8_t)121, (uint8_t)10, (uint8_t)52, (uint8_t)84, (uint8_t)212, (uint8_t)71, (uint8_t)53, (uint8_t)22};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        p142_transfer_type_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        p142_uri_type_SET((uint8_t)(uint8_t)134, PH.base.pack) ;
        {
            uint8_t storage[] =  {(uint8_t)215, (uint8_t)2, (uint8_t)9, (uint8_t)22, (uint8_t)21, (uint8_t)60, (uint8_t)52, (uint8_t)121, (uint8_t)40, (uint8_t)255, (uint8_t)149, (uint8_t)66, (uint8_t)223, (uint8_t)140, (uint8_t)156, (uint8_t)52, (uint8_t)173, (uint8_t)49, (uint8_t)106, (uint8_t)180, (uint8_t)185, (uint8_t)147, (uint8_t)61, (uint8_t)167, (uint8_t)153, (uint8_t)193, (uint8_t)35, (uint8_t)147, (uint8_t)159, (uint8_t)21, (uint8_t)234, (uint8_t)222, (uint8_t)243, (uint8_t)71, (uint8_t)181, (uint8_t)136, (uint8_t)30, (uint8_t)36, (uint8_t)152, (uint8_t)68, (uint8_t)193, (uint8_t)187, (uint8_t)129, (uint8_t)201, (uint8_t)91, (uint8_t)121, (uint8_t)88, (uint8_t)22, (uint8_t)21, (uint8_t)182, (uint8_t)212, (uint8_t)226, (uint8_t)7, (uint8_t)12, (uint8_t)148, (uint8_t)74, (uint8_t)140, (uint8_t)120, (uint8_t)41, (uint8_t)78, (uint8_t)111, (uint8_t)23, (uint8_t)166, (uint8_t)173, (uint8_t)146, (uint8_t)90, (uint8_t)59, (uint8_t)235, (uint8_t)208, (uint8_t)203, (uint8_t)74, (uint8_t)117, (uint8_t)19, (uint8_t)218, (uint8_t)57, (uint8_t)184, (uint8_t)207, (uint8_t)69, (uint8_t)102, (uint8_t)173, (uint8_t)190, (uint8_t)17, (uint8_t)44, (uint8_t)245, (uint8_t)19, (uint8_t)252, (uint8_t)224, (uint8_t)65, (uint8_t)99, (uint8_t)131, (uint8_t)84, (uint8_t)184, (uint8_t)215, (uint8_t)58, (uint8_t)196, (uint8_t)197, (uint8_t)79, (uint8_t)120, (uint8_t)169, (uint8_t)232, (uint8_t)62, (uint8_t)37, (uint8_t)208, (uint8_t)248, (uint8_t)176, (uint8_t)35, (uint8_t)93, (uint8_t)165, (uint8_t)88, (uint8_t)104, (uint8_t)207, (uint8_t)7, (uint8_t)173, (uint8_t)241, (uint8_t)22, (uint8_t)7, (uint8_t)188, (uint8_t)91, (uint8_t)193, (uint8_t)180};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_temperature_SET((int16_t)(int16_t)14364, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)2498362883L, PH.base.pack) ;
        p143_press_abs_SET((float) -6.8153353E37F, PH.base.pack) ;
        p143_press_diff_SET((float)2.4009726E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FOLLOW_TARGET_144(), &PH);
        {
            float attitude_q[] =  {-3.121019E38F, -1.771192E38F, 2.2397447E38F, 3.2741663E38F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        p144_custom_state_SET((uint64_t)8877216893280063939L, PH.base.pack) ;
        p144_lon_SET((int32_t)983143444, PH.base.pack) ;
        p144_est_capabilities_SET((uint8_t)(uint8_t)191, PH.base.pack) ;
        p144_lat_SET((int32_t)493012254, PH.base.pack) ;
        p144_timestamp_SET((uint64_t)8547907477973938179L, PH.base.pack) ;
        {
            float acc[] =  {-4.25672E37F, 2.2579036E38F, 2.831111E38F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        p144_alt_SET((float) -2.281812E38F, PH.base.pack) ;
        {
            float rates[] =  {-1.8304023E38F, -2.5965695E38F, 5.3772587E37F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        {
            float position_cov[] =  {-5.2713653E37F, -2.4599967E38F, -7.9903034E37F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        {
            float vel[] =  {3.3749885E38F, 2.458964E38F, 8.891448E37F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_y_vel_SET((float)1.1573261E38F, PH.base.pack) ;
        p146_pitch_rate_SET((float)2.9532132E38F, PH.base.pack) ;
        p146_y_acc_SET((float)2.435616E38F, PH.base.pack) ;
        p146_x_vel_SET((float) -1.453971E38F, PH.base.pack) ;
        p146_z_acc_SET((float) -5.33867E37F, PH.base.pack) ;
        p146_y_pos_SET((float) -1.6415939E38F, PH.base.pack) ;
        p146_yaw_rate_SET((float)2.010907E38F, PH.base.pack) ;
        p146_airspeed_SET((float) -1.205771E38F, PH.base.pack) ;
        {
            float pos_variance[] =  {-1.4651292E38F, 1.5273652E38F, 3.831537E37F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_x_acc_SET((float) -2.7363747E38F, PH.base.pack) ;
        p146_time_usec_SET((uint64_t)8100699072674749972L, PH.base.pack) ;
        {
            float vel_variance[] =  {-7.804859E37F, -2.4251207E38F, -2.0166995E38F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        p146_z_vel_SET((float)5.4560453E37F, PH.base.pack) ;
        p146_roll_rate_SET((float)6.859886E37F, PH.base.pack) ;
        p146_x_pos_SET((float)1.4072873E38F, PH.base.pack) ;
        p146_z_pos_SET((float) -8.363346E37F, PH.base.pack) ;
        {
            float q[] =  {-3.382749E38F, -2.0699702E38F, -3.4076945E36F, 2.9086022E38F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BATTERY_STATUS_147(), &PH);
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_UNKNOWN, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t)97, PH.base.pack) ;
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LION, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)53762, (uint16_t)8505, (uint16_t)18593, (uint16_t)53910, (uint16_t)26292, (uint16_t)51565, (uint16_t)42924, (uint16_t)39094, (uint16_t)45958, (uint16_t)51872};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        p147_energy_consumed_SET((int32_t)921662843, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t) -17113, PH.base.pack) ;
        p147_current_consumed_SET((int32_t) -980876098, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)77, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t) -18136, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_148(), &PH);
        p148_product_id_SET((uint16_t)(uint16_t)22012, PH.base.pack) ;
        p148_os_sw_version_SET((uint32_t)3780009964L, PH.base.pack) ;
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)238, (uint8_t)173, (uint8_t)171, (uint8_t)41, (uint8_t)239, (uint8_t)21, (uint8_t)37, (uint8_t)135};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        p148_vendor_id_SET((uint16_t)(uint16_t)59339, PH.base.pack) ;
        {
            uint8_t uid2[] =  {(uint8_t)27, (uint8_t)209, (uint8_t)7, (uint8_t)30, (uint8_t)144, (uint8_t)245, (uint8_t)116, (uint8_t)220, (uint8_t)253, (uint8_t)236, (uint8_t)187, (uint8_t)144, (uint8_t)231, (uint8_t)18, (uint8_t)152, (uint8_t)241, (uint8_t)75, (uint8_t)230};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_flight_sw_version_SET((uint32_t)3453251130L, PH.base.pack) ;
        p148_board_version_SET((uint32_t)1223154979L, PH.base.pack) ;
        {
            uint8_t flight_custom_version[] =  {(uint8_t)143, (uint8_t)101, (uint8_t)132, (uint8_t)171, (uint8_t)12, (uint8_t)32, (uint8_t)191, (uint8_t)55};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        p148_middleware_sw_version_SET((uint32_t)2806186920L, PH.base.pack) ;
        p148_uid_SET((uint64_t)7747904855314507675L, PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)139, (uint8_t)225, (uint8_t)30, (uint8_t)213, (uint8_t)224, (uint8_t)229, (uint8_t)178, (uint8_t)38};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FENCE, PH.base.pack) ;
        c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LANDING_TARGET_149(), &PH);
        p149_angle_y_SET((float)1.5788509E38F, PH.base.pack) ;
        p149_y_SET((float) -2.2123766E38F, &PH) ;
        p149_distance_SET((float)3.2397742E38F, PH.base.pack) ;
        p149_target_num_SET((uint8_t)(uint8_t)22, PH.base.pack) ;
        p149_z_SET((float) -1.1381937E38F, &PH) ;
        {
            float q[] =  {-1.3150781E38F, -1.4892152E38F, -7.5789924E37F, -2.2410807E38F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_VISION_OTHER, PH.base.pack) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
        p149_size_y_SET((float)1.4547794E36F, PH.base.pack) ;
        p149_position_valid_SET((uint8_t)(uint8_t)112, &PH) ;
        p149_time_usec_SET((uint64_t)7802513781881485484L, PH.base.pack) ;
        p149_size_x_SET((float) -1.8292344E38F, PH.base.pack) ;
        p149_angle_x_SET((float) -2.2618947E38F, PH.base.pack) ;
        p149_x_SET((float)1.4784523E38F, &PH) ;
        c_LoopBackDemoChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_POS_HORIZ_REL, PH.base.pack) ;
        p230_pos_horiz_ratio_SET((float)1.8074436E38F, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float) -3.0477509E38F, PH.base.pack) ;
        p230_vel_ratio_SET((float) -8.664683E37F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float)5.99704E37F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)7123351080914805667L, PH.base.pack) ;
        p230_tas_ratio_SET((float)1.0647877E38F, PH.base.pack) ;
        p230_hagl_ratio_SET((float)1.6442782E38F, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float)3.2518238E38F, PH.base.pack) ;
        p230_mag_ratio_SET((float)3.237829E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIND_COV_231(), &PH);
        p231_var_vert_SET((float) -6.2388225E37F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)4549712318110222512L, PH.base.pack) ;
        p231_var_horiz_SET((float)5.3677574E37F, PH.base.pack) ;
        p231_wind_y_SET((float)1.2343324E38F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float) -3.014653E38F, PH.base.pack) ;
        p231_wind_x_SET((float) -1.8396141E38F, PH.base.pack) ;
        p231_wind_z_SET((float)8.999614E37F, PH.base.pack) ;
        p231_wind_alt_SET((float)3.0124502E38F, PH.base.pack) ;
        p231_vert_accuracy_SET((float)2.3143706E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INPUT_232(), &PH);
        p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_VEL_VERT, PH.base.pack) ;
        p232_horiz_accuracy_SET((float) -2.1763E38F, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)1934282804L, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
        p232_lon_SET((int32_t)1607196787, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)43401, PH.base.pack) ;
        p232_vn_SET((float)3.2158306E38F, PH.base.pack) ;
        p232_vdop_SET((float)1.5408743E38F, PH.base.pack) ;
        p232_alt_SET((float) -2.4569649E38F, PH.base.pack) ;
        p232_speed_accuracy_SET((float) -1.0689536E37F, PH.base.pack) ;
        p232_hdop_SET((float) -2.9820492E37F, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p232_lat_SET((int32_t) -572434130, PH.base.pack) ;
        p232_vert_accuracy_SET((float)3.253345E38F, PH.base.pack) ;
        p232_vd_SET((float) -9.500356E37F, PH.base.pack) ;
        p232_ve_SET((float) -1.0505313E38F, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)215, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)2143957542994889076L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTCM_DATA_233(), &PH);
        p233_flags_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
        p233_len_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)51, (uint8_t)7, (uint8_t)118, (uint8_t)43, (uint8_t)236, (uint8_t)160, (uint8_t)212, (uint8_t)41, (uint8_t)117, (uint8_t)236, (uint8_t)88, (uint8_t)171, (uint8_t)46, (uint8_t)220, (uint8_t)32, (uint8_t)70, (uint8_t)104, (uint8_t)182, (uint8_t)91, (uint8_t)210, (uint8_t)248, (uint8_t)55, (uint8_t)109, (uint8_t)89, (uint8_t)224, (uint8_t)237, (uint8_t)27, (uint8_t)49, (uint8_t)167, (uint8_t)237, (uint8_t)173, (uint8_t)56, (uint8_t)30, (uint8_t)221, (uint8_t)217, (uint8_t)253, (uint8_t)21, (uint8_t)9, (uint8_t)227, (uint8_t)247, (uint8_t)248, (uint8_t)80, (uint8_t)254, (uint8_t)78, (uint8_t)55, (uint8_t)207, (uint8_t)61, (uint8_t)30, (uint8_t)101, (uint8_t)136, (uint8_t)15, (uint8_t)53, (uint8_t)252, (uint8_t)221, (uint8_t)238, (uint8_t)85, (uint8_t)232, (uint8_t)190, (uint8_t)189, (uint8_t)137, (uint8_t)4, (uint8_t)83, (uint8_t)134, (uint8_t)201, (uint8_t)232, (uint8_t)169, (uint8_t)243, (uint8_t)166, (uint8_t)166, (uint8_t)239, (uint8_t)154, (uint8_t)53, (uint8_t)156, (uint8_t)53, (uint8_t)0, (uint8_t)83, (uint8_t)136, (uint8_t)157, (uint8_t)42, (uint8_t)140, (uint8_t)216, (uint8_t)115, (uint8_t)47, (uint8_t)195, (uint8_t)218, (uint8_t)152, (uint8_t)188, (uint8_t)74, (uint8_t)131, (uint8_t)8, (uint8_t)161, (uint8_t)161, (uint8_t)124, (uint8_t)0, (uint8_t)142, (uint8_t)198, (uint8_t)89, (uint8_t)63, (uint8_t)171, (uint8_t)153, (uint8_t)232, (uint8_t)112, (uint8_t)96, (uint8_t)41, (uint8_t)137, (uint8_t)182, (uint8_t)186, (uint8_t)212, (uint8_t)163, (uint8_t)90, (uint8_t)184, (uint8_t)89, (uint8_t)107, (uint8_t)92, (uint8_t)120, (uint8_t)41, (uint8_t)134, (uint8_t)214, (uint8_t)199, (uint8_t)81, (uint8_t)22, (uint8_t)41, (uint8_t)234, (uint8_t)95, (uint8_t)15, (uint8_t)91, (uint8_t)92, (uint8_t)1, (uint8_t)166, (uint8_t)125, (uint8_t)103, (uint8_t)113, (uint8_t)17, (uint8_t)214, (uint8_t)167, (uint8_t)175, (uint8_t)14, (uint8_t)222, (uint8_t)127, (uint8_t)91, (uint8_t)129, (uint8_t)243, (uint8_t)126, (uint8_t)12, (uint8_t)95, (uint8_t)224, (uint8_t)125, (uint8_t)47, (uint8_t)19, (uint8_t)165, (uint8_t)66, (uint8_t)253, (uint8_t)142, (uint8_t)161, (uint8_t)142, (uint8_t)239, (uint8_t)83, (uint8_t)159, (uint8_t)208, (uint8_t)133, (uint8_t)218, (uint8_t)47, (uint8_t)93, (uint8_t)185, (uint8_t)144, (uint8_t)192, (uint8_t)248, (uint8_t)181, (uint8_t)210, (uint8_t)87, (uint8_t)162, (uint8_t)159, (uint8_t)229, (uint8_t)74, (uint8_t)117, (uint8_t)142, (uint8_t)157, (uint8_t)232, (uint8_t)188, (uint8_t)240};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGH_LATENCY_234(), &PH);
        p234_roll_SET((int16_t)(int16_t)30152, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t)21826, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t)116, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t)25481, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t)121, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)188, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p234_custom_mode_SET((uint32_t)3977809549L, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t)47, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t) -3750, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t)32346, PH.base.pack) ;
        p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_AUTO_ENABLED, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)42997, PH.base.pack) ;
        p234_longitude_SET((int32_t) -538188357, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_PPP, PH.base.pack) ;
        p234_latitude_SET((int32_t)1163153918, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)4853, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t)49, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIBRATION_241(), &PH);
        p241_time_usec_SET((uint64_t)1000070213894718602L, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)2482580071L, PH.base.pack) ;
        p241_vibration_x_SET((float)7.563208E37F, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)125022799L, PH.base.pack) ;
        p241_vibration_y_SET((float) -1.7384765E38F, PH.base.pack) ;
        p241_clipping_2_SET((uint32_t)2529418196L, PH.base.pack) ;
        p241_vibration_z_SET((float)2.7602272E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HOME_POSITION_242(), &PH);
        p242_latitude_SET((int32_t)502866197, PH.base.pack) ;
        p242_y_SET((float) -2.6022922E38F, PH.base.pack) ;
        p242_altitude_SET((int32_t) -841105729, PH.base.pack) ;
        p242_longitude_SET((int32_t)501816747, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)5967379375408375287L, &PH) ;
        {
            float q[] =  {1.676231E38F, 3.3991467E38F, 3.4882215E37F, -2.4813502E38F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_approach_y_SET((float)1.7044428E38F, PH.base.pack) ;
        p242_approach_x_SET((float)2.7410028E38F, PH.base.pack) ;
        p242_x_SET((float)4.044369E37F, PH.base.pack) ;
        p242_z_SET((float) -2.3520833E38F, PH.base.pack) ;
        p242_approach_z_SET((float)1.8496775E36F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_HOME_POSITION_243(), &PH);
        p243_target_system_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p243_approach_z_SET((float) -1.1589811E37F, PH.base.pack) ;
        p243_approach_x_SET((float)1.2117527E38F, PH.base.pack) ;
        p243_altitude_SET((int32_t) -1643343410, PH.base.pack) ;
        p243_y_SET((float) -2.6454176E38F, PH.base.pack) ;
        p243_longitude_SET((int32_t) -1584831148, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)5858762395317330072L, &PH) ;
        p243_approach_y_SET((float) -3.0426993E38F, PH.base.pack) ;
        p243_x_SET((float) -3.253873E38F, PH.base.pack) ;
        p243_latitude_SET((int32_t) -1299964614, PH.base.pack) ;
        p243_z_SET((float) -1.4541539E38F, PH.base.pack) ;
        {
            float q[] =  {3.2708485E38F, 3.1246281E38F, -1.7607592E38F, -2.2653736E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_message_id_SET((uint16_t)(uint16_t)23189, PH.base.pack) ;
        p244_interval_us_SET((int32_t) -1858382721, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_FW, PH.base.pack) ;
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_IN_AIR, PH.base.pack) ;
        c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ADSB_VEHICLE_246(), &PH);
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UNASSIGNED2, PH.base.pack) ;
        {
            char16_t* callsign = u"h";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_heading_SET((uint16_t)(uint16_t)48668, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t) -23766, PH.base.pack) ;
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_PRESSURE_QNH, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)3554927816L, PH.base.pack) ;
        p246_hor_velocity_SET((uint16_t)(uint16_t)24670, PH.base.pack) ;
        p246_lat_SET((int32_t)1113435557, PH.base.pack) ;
        p246_lon_SET((int32_t)1032470985, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)45432, PH.base.pack) ;
        p246_altitude_SET((int32_t) -1708736400, PH.base.pack) ;
        p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COLLISION_247(), &PH);
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_HOVER, PH.base.pack) ;
        p247_time_to_minimum_delta_SET((float) -1.8277834E38F, PH.base.pack) ;
        p247_id_SET((uint32_t)159497008L, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float)3.2962114E38F, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_ADSB, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float)1.2359958E38F, PH.base.pack) ;
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_NONE, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_V2_EXTENSION_248(), &PH);
        {
            uint8_t payload[] =  {(uint8_t)58, (uint8_t)115, (uint8_t)76, (uint8_t)95, (uint8_t)116, (uint8_t)251, (uint8_t)129, (uint8_t)33, (uint8_t)150, (uint8_t)180, (uint8_t)198, (uint8_t)217, (uint8_t)149, (uint8_t)22, (uint8_t)162, (uint8_t)147, (uint8_t)83, (uint8_t)200, (uint8_t)33, (uint8_t)163, (uint8_t)56, (uint8_t)120, (uint8_t)238, (uint8_t)246, (uint8_t)213, (uint8_t)88, (uint8_t)233, (uint8_t)235, (uint8_t)196, (uint8_t)107, (uint8_t)106, (uint8_t)90, (uint8_t)104, (uint8_t)85, (uint8_t)46, (uint8_t)251, (uint8_t)13, (uint8_t)255, (uint8_t)132, (uint8_t)37, (uint8_t)210, (uint8_t)41, (uint8_t)93, (uint8_t)223, (uint8_t)255, (uint8_t)121, (uint8_t)225, (uint8_t)149, (uint8_t)19, (uint8_t)255, (uint8_t)158, (uint8_t)71, (uint8_t)129, (uint8_t)34, (uint8_t)50, (uint8_t)177, (uint8_t)178, (uint8_t)137, (uint8_t)242, (uint8_t)184, (uint8_t)169, (uint8_t)178, (uint8_t)209, (uint8_t)1, (uint8_t)248, (uint8_t)214, (uint8_t)22, (uint8_t)18, (uint8_t)246, (uint8_t)167, (uint8_t)33, (uint8_t)77, (uint8_t)74, (uint8_t)156, (uint8_t)156, (uint8_t)35, (uint8_t)109, (uint8_t)200, (uint8_t)6, (uint8_t)107, (uint8_t)105, (uint8_t)62, (uint8_t)70, (uint8_t)153, (uint8_t)146, (uint8_t)137, (uint8_t)236, (uint8_t)62, (uint8_t)94, (uint8_t)71, (uint8_t)187, (uint8_t)199, (uint8_t)81, (uint8_t)30, (uint8_t)30, (uint8_t)175, (uint8_t)28, (uint8_t)249, (uint8_t)184, (uint8_t)58, (uint8_t)75, (uint8_t)181, (uint8_t)207, (uint8_t)226, (uint8_t)158, (uint8_t)253, (uint8_t)224, (uint8_t)81, (uint8_t)123, (uint8_t)209, (uint8_t)151, (uint8_t)153, (uint8_t)91, (uint8_t)8, (uint8_t)54, (uint8_t)120, (uint8_t)103, (uint8_t)10, (uint8_t)28, (uint8_t)128, (uint8_t)80, (uint8_t)76, (uint8_t)116, (uint8_t)127, (uint8_t)69, (uint8_t)173, (uint8_t)58, (uint8_t)153, (uint8_t)154, (uint8_t)55, (uint8_t)103, (uint8_t)182, (uint8_t)249, (uint8_t)107, (uint8_t)233, (uint8_t)25, (uint8_t)0, (uint8_t)219, (uint8_t)82, (uint8_t)43, (uint8_t)18, (uint8_t)191, (uint8_t)246, (uint8_t)183, (uint8_t)13, (uint8_t)147, (uint8_t)35, (uint8_t)248, (uint8_t)154, (uint8_t)222, (uint8_t)145, (uint8_t)127, (uint8_t)38, (uint8_t)221, (uint8_t)122, (uint8_t)0, (uint8_t)20, (uint8_t)187, (uint8_t)123, (uint8_t)65, (uint8_t)187, (uint8_t)101, (uint8_t)19, (uint8_t)25, (uint8_t)181, (uint8_t)37, (uint8_t)95, (uint8_t)212, (uint8_t)74, (uint8_t)123, (uint8_t)88, (uint8_t)122, (uint8_t)209, (uint8_t)58, (uint8_t)143, (uint8_t)38, (uint8_t)98, (uint8_t)178, (uint8_t)142, (uint8_t)157, (uint8_t)16, (uint8_t)128, (uint8_t)129, (uint8_t)59, (uint8_t)170, (uint8_t)200, (uint8_t)60, (uint8_t)245, (uint8_t)221, (uint8_t)233, (uint8_t)166, (uint8_t)240, (uint8_t)149, (uint8_t)5, (uint8_t)218, (uint8_t)233, (uint8_t)136, (uint8_t)13, (uint8_t)132, (uint8_t)74, (uint8_t)47, (uint8_t)96, (uint8_t)250, (uint8_t)207, (uint8_t)27, (uint8_t)179, (uint8_t)247, (uint8_t)123, (uint8_t)189, (uint8_t)6, (uint8_t)35, (uint8_t)91, (uint8_t)149, (uint8_t)4, (uint8_t)255, (uint8_t)233, (uint8_t)22, (uint8_t)85, (uint8_t)37, (uint8_t)184, (uint8_t)214, (uint8_t)153, (uint8_t)217, (uint8_t)122, (uint8_t)250, (uint8_t)76, (uint8_t)209, (uint8_t)178, (uint8_t)46, (uint8_t)116, (uint8_t)146, (uint8_t)249, (uint8_t)215, (uint8_t)63, (uint8_t)74, (uint8_t)201, (uint8_t)221, (uint8_t)171, (uint8_t)80, (uint8_t)51, (uint8_t)17, (uint8_t)178, (uint8_t)121, (uint8_t)123, (uint8_t)193, (uint8_t)74, (uint8_t)105, (uint8_t)108, (uint8_t)200};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p248_target_system_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        p248_target_component_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        p248_target_network_SET((uint8_t)(uint8_t)242, PH.base.pack) ;
        p248_message_type_SET((uint16_t)(uint16_t)25112, PH.base.pack) ;
        c_LoopBackDemoChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MEMORY_VECT_249(), &PH);
        p249_ver_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        {
            int8_t value[] =  {(int8_t)35, (int8_t) -49, (int8_t)8, (int8_t) -113, (int8_t) -98, (int8_t) -66, (int8_t)118, (int8_t)46, (int8_t)21, (int8_t)25, (int8_t) -29, (int8_t)93, (int8_t) -126, (int8_t)101, (int8_t)46, (int8_t)73, (int8_t)92, (int8_t)84, (int8_t)115, (int8_t) -106, (int8_t) -62, (int8_t) -120, (int8_t)117, (int8_t) -115, (int8_t) -116, (int8_t)14, (int8_t)3, (int8_t) -92, (int8_t)27, (int8_t) -41, (int8_t)31, (int8_t) -89};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_address_SET((uint16_t)(uint16_t)14482, PH.base.pack) ;
        p249_type_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEBUG_VECT_250(), &PH);
        p250_y_SET((float)1.3525078E38F, PH.base.pack) ;
        p250_x_SET((float) -1.4829207E38F, PH.base.pack) ;
        p250_z_SET((float) -7.1027746E37F, PH.base.pack) ;
        {
            char16_t* name = u"bzijZtbsa";
            p250_name_SET_(name, &PH) ;
        }
        p250_time_usec_SET((uint64_t)8269817269184282550L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
        p251_value_SET((float)1.0956553E36F, PH.base.pack) ;
        p251_time_boot_ms_SET((uint32_t)3664150561L, PH.base.pack) ;
        {
            char16_t* name = u"mrs";
            p251_name_SET_(name, &PH) ;
        }
        c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_INT_252(), &PH);
        p252_value_SET((int32_t)927097636, PH.base.pack) ;
        {
            char16_t* name = u"zhnJpi";
            p252_name_SET_(name, &PH) ;
        }
        p252_time_boot_ms_SET((uint32_t)3792242384L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STATUSTEXT_253(), &PH);
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_EMERGENCY, PH.base.pack) ;
        {
            char16_t* text = u"livpfwlwLhCtejybwkupaadbz";
            p253_text_SET_(text, &PH) ;
        }
        c_LoopBackDemoChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEBUG_254(), &PH);
        p254_time_boot_ms_SET((uint32_t)3098739729L, PH.base.pack) ;
        p254_value_SET((float)1.2226327E38F, PH.base.pack) ;
        p254_ind_SET((uint8_t)(uint8_t)94, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SETUP_SIGNING_256(), &PH);
        p256_initial_timestamp_SET((uint64_t)3219416032820536588L, PH.base.pack) ;
        p256_target_system_SET((uint8_t)(uint8_t)220, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)3, (uint8_t)123, (uint8_t)220, (uint8_t)133, (uint8_t)116, (uint8_t)132, (uint8_t)114, (uint8_t)139, (uint8_t)23, (uint8_t)182, (uint8_t)29, (uint8_t)108, (uint8_t)231, (uint8_t)246, (uint8_t)45, (uint8_t)83, (uint8_t)76, (uint8_t)240, (uint8_t)97, (uint8_t)164, (uint8_t)81, (uint8_t)172, (uint8_t)49, (uint8_t)145, (uint8_t)144, (uint8_t)21, (uint8_t)54, (uint8_t)34, (uint8_t)147, (uint8_t)70, (uint8_t)84, (uint8_t)128};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        p256_target_component_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BUTTON_CHANGE_257(), &PH);
        p257_state_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        p257_time_boot_ms_SET((uint32_t)888989796L, PH.base.pack) ;
        p257_last_change_ms_SET((uint32_t)2937678060L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PLAY_TUNE_258(), &PH);
        p258_target_component_SET((uint8_t)(uint8_t)65, PH.base.pack) ;
        p258_target_system_SET((uint8_t)(uint8_t)209, PH.base.pack) ;
        {
            char16_t* tune = u"cwMmhpgeolCEwnpvalsddvnft";
            p258_tune_SET_(tune, &PH) ;
        }
        c_LoopBackDemoChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_INFORMATION_259(), &PH);
        p259_resolution_h_SET((uint16_t)(uint16_t)39852, PH.base.pack) ;
        p259_resolution_v_SET((uint16_t)(uint16_t)51828, PH.base.pack) ;
        {
            uint8_t model_name[] =  {(uint8_t)152, (uint8_t)186, (uint8_t)215, (uint8_t)32, (uint8_t)85, (uint8_t)76, (uint8_t)70, (uint8_t)70, (uint8_t)98, (uint8_t)228, (uint8_t)234, (uint8_t)2, (uint8_t)194, (uint8_t)3, (uint8_t)177, (uint8_t)3, (uint8_t)213, (uint8_t)229, (uint8_t)113, (uint8_t)209, (uint8_t)30, (uint8_t)226, (uint8_t)167, (uint8_t)3, (uint8_t)128, (uint8_t)238, (uint8_t)69, (uint8_t)4, (uint8_t)150, (uint8_t)35, (uint8_t)191, (uint8_t)237};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_sensor_size_v_SET((float)8.1888195E37F, PH.base.pack) ;
        p259_firmware_version_SET((uint32_t)604580272L, PH.base.pack) ;
        p259_focal_length_SET((float)2.4536994E38F, PH.base.pack) ;
        p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE, PH.base.pack) ;
        p259_time_boot_ms_SET((uint32_t)1309907306L, PH.base.pack) ;
        p259_sensor_size_h_SET((float) -9.666031E37F, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)24, (uint8_t)230, (uint8_t)90, (uint8_t)32, (uint8_t)31, (uint8_t)226, (uint8_t)196, (uint8_t)225, (uint8_t)185, (uint8_t)17, (uint8_t)142, (uint8_t)119, (uint8_t)183, (uint8_t)35, (uint8_t)199, (uint8_t)4, (uint8_t)188, (uint8_t)153, (uint8_t)1, (uint8_t)118, (uint8_t)39, (uint8_t)129, (uint8_t)203, (uint8_t)207, (uint8_t)169, (uint8_t)149, (uint8_t)185, (uint8_t)51, (uint8_t)121, (uint8_t)250, (uint8_t)145, (uint8_t)193};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_lens_id_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"hxGyHeldalv";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_cam_definition_version_SET((uint16_t)(uint16_t)19346, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_SETTINGS_260(), &PH);
        p260_time_boot_ms_SET((uint32_t)3994505774L, PH.base.pack) ;
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_IMAGE, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STORAGE_INFORMATION_261(), &PH);
        p261_status_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p261_total_capacity_SET((float) -9.844026E37F, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        p261_used_capacity_SET((float)1.8202154E38F, PH.base.pack) ;
        p261_write_speed_SET((float) -1.9810716E38F, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)3775668738L, PH.base.pack) ;
        p261_available_capacity_SET((float) -1.6875967E38F, PH.base.pack) ;
        p261_read_speed_SET((float)2.0629405E38F, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_available_capacity_SET((float)1.1271745E37F, PH.base.pack) ;
        p262_image_interval_SET((float) -1.3544878E38F, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)85, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)157313535L, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
        p262_recording_time_ms_SET((uint32_t)2360144998L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        p263_image_index_SET((int32_t)159098275, PH.base.pack) ;
        {
            float q[] =  {2.2445409E38F, -3.3703807E38F, -9.4415536E36F, -1.4890001E38F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_lat_SET((int32_t) -184136594, PH.base.pack) ;
        p263_capture_result_SET((int8_t)(int8_t) -103, PH.base.pack) ;
        p263_lon_SET((int32_t)410349912, PH.base.pack) ;
        p263_camera_id_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        {
            char16_t* file_url = u"mbzzQgqfTkupvsxdqlyeMtlrlvFfuekreinhftuDjpogokPuCfusruifJqszyvmgjlomnobfznNnchnppLKmyipnnnaanwkhkbnJuxwkbhjgmufssGuImdztjbleksz";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_time_utc_SET((uint64_t)596422516538759598L, PH.base.pack) ;
        p263_relative_alt_SET((int32_t)1627064809, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)1092020896L, PH.base.pack) ;
        p263_alt_SET((int32_t)713523747, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_takeoff_time_utc_SET((uint64_t)7956065181207754720L, PH.base.pack) ;
        p264_arming_time_utc_SET((uint64_t)2762854935031538156L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)2129537382122781051L, PH.base.pack) ;
        p264_time_boot_ms_SET((uint32_t)4059525250L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_roll_SET((float) -1.9646647E38F, PH.base.pack) ;
        p265_yaw_SET((float)2.7789931E38F, PH.base.pack) ;
        p265_time_boot_ms_SET((uint32_t)3871428288L, PH.base.pack) ;
        p265_pitch_SET((float)1.9496485E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_266(), &PH);
        p266_length_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p266_first_message_offset_SET((uint8_t)(uint8_t)101, PH.base.pack) ;
        p266_sequence_SET((uint16_t)(uint16_t)53053, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        p266_target_system_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)203, (uint8_t)199, (uint8_t)141, (uint8_t)182, (uint8_t)104, (uint8_t)41, (uint8_t)30, (uint8_t)90, (uint8_t)138, (uint8_t)190, (uint8_t)190, (uint8_t)255, (uint8_t)178, (uint8_t)72, (uint8_t)66, (uint8_t)5, (uint8_t)223, (uint8_t)35, (uint8_t)222, (uint8_t)172, (uint8_t)68, (uint8_t)56, (uint8_t)77, (uint8_t)168, (uint8_t)26, (uint8_t)113, (uint8_t)200, (uint8_t)56, (uint8_t)165, (uint8_t)165, (uint8_t)66, (uint8_t)203, (uint8_t)157, (uint8_t)63, (uint8_t)169, (uint8_t)205, (uint8_t)240, (uint8_t)162, (uint8_t)200, (uint8_t)195, (uint8_t)170, (uint8_t)122, (uint8_t)191, (uint8_t)116, (uint8_t)40, (uint8_t)12, (uint8_t)187, (uint8_t)237, (uint8_t)177, (uint8_t)57, (uint8_t)205, (uint8_t)227, (uint8_t)68, (uint8_t)57, (uint8_t)241, (uint8_t)250, (uint8_t)74, (uint8_t)183, (uint8_t)244, (uint8_t)231, (uint8_t)209, (uint8_t)16, (uint8_t)83, (uint8_t)85, (uint8_t)157, (uint8_t)73, (uint8_t)177, (uint8_t)49, (uint8_t)205, (uint8_t)44, (uint8_t)167, (uint8_t)169, (uint8_t)17, (uint8_t)239, (uint8_t)226, (uint8_t)253, (uint8_t)229, (uint8_t)8, (uint8_t)14, (uint8_t)96, (uint8_t)136, (uint8_t)158, (uint8_t)186, (uint8_t)132, (uint8_t)131, (uint8_t)56, (uint8_t)115, (uint8_t)85, (uint8_t)123, (uint8_t)158, (uint8_t)186, (uint8_t)66, (uint8_t)153, (uint8_t)46, (uint8_t)154, (uint8_t)141, (uint8_t)211, (uint8_t)224, (uint8_t)17, (uint8_t)158, (uint8_t)181, (uint8_t)206, (uint8_t)88, (uint8_t)162, (uint8_t)124, (uint8_t)146, (uint8_t)170, (uint8_t)103, (uint8_t)198, (uint8_t)193, (uint8_t)41, (uint8_t)164, (uint8_t)153, (uint8_t)4, (uint8_t)164, (uint8_t)3, (uint8_t)121, (uint8_t)218, (uint8_t)215, (uint8_t)156, (uint8_t)151, (uint8_t)45, (uint8_t)10, (uint8_t)139, (uint8_t)46, (uint8_t)236, (uint8_t)6, (uint8_t)38, (uint8_t)35, (uint8_t)102, (uint8_t)45, (uint8_t)152, (uint8_t)171, (uint8_t)175, (uint8_t)147, (uint8_t)222, (uint8_t)229, (uint8_t)86, (uint8_t)104, (uint8_t)215, (uint8_t)220, (uint8_t)205, (uint8_t)127, (uint8_t)42, (uint8_t)3, (uint8_t)167, (uint8_t)96, (uint8_t)56, (uint8_t)38, (uint8_t)200, (uint8_t)216, (uint8_t)50, (uint8_t)216, (uint8_t)239, (uint8_t)105, (uint8_t)247, (uint8_t)56, (uint8_t)112, (uint8_t)178, (uint8_t)245, (uint8_t)159, (uint8_t)129, (uint8_t)197, (uint8_t)199, (uint8_t)20, (uint8_t)47, (uint8_t)177, (uint8_t)178, (uint8_t)159, (uint8_t)16, (uint8_t)186, (uint8_t)67, (uint8_t)125, (uint8_t)186, (uint8_t)179, (uint8_t)211, (uint8_t)200, (uint8_t)69, (uint8_t)74, (uint8_t)178, (uint8_t)42, (uint8_t)21, (uint8_t)150, (uint8_t)76, (uint8_t)236, (uint8_t)134, (uint8_t)158, (uint8_t)94, (uint8_t)101, (uint8_t)131, (uint8_t)120, (uint8_t)13, (uint8_t)106, (uint8_t)22, (uint8_t)151, (uint8_t)84, (uint8_t)244, (uint8_t)151, (uint8_t)163, (uint8_t)92, (uint8_t)252, (uint8_t)133, (uint8_t)108, (uint8_t)208, (uint8_t)42, (uint8_t)190, (uint8_t)103, (uint8_t)43, (uint8_t)143, (uint8_t)109, (uint8_t)213, (uint8_t)195, (uint8_t)144, (uint8_t)237, (uint8_t)160, (uint8_t)182, (uint8_t)54, (uint8_t)167, (uint8_t)205, (uint8_t)146, (uint8_t)174, (uint8_t)189, (uint8_t)163, (uint8_t)183, (uint8_t)36, (uint8_t)73, (uint8_t)235, (uint8_t)202, (uint8_t)79, (uint8_t)152, (uint8_t)192, (uint8_t)217, (uint8_t)94, (uint8_t)219, (uint8_t)69, (uint8_t)191, (uint8_t)226, (uint8_t)226, (uint8_t)206, (uint8_t)131, (uint8_t)0, (uint8_t)218, (uint8_t)96, (uint8_t)205, (uint8_t)189, (uint8_t)13, (uint8_t)25, (uint8_t)173, (uint8_t)168};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_ACKED_267(), &PH);
        p267_target_component_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
        p267_first_message_offset_SET((uint8_t)(uint8_t)61, PH.base.pack) ;
        p267_target_system_SET((uint8_t)(uint8_t)24, PH.base.pack) ;
        p267_length_SET((uint8_t)(uint8_t)8, PH.base.pack) ;
        p267_sequence_SET((uint16_t)(uint16_t)20529, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)138, (uint8_t)250, (uint8_t)26, (uint8_t)215, (uint8_t)73, (uint8_t)204, (uint8_t)247, (uint8_t)215, (uint8_t)141, (uint8_t)245, (uint8_t)5, (uint8_t)234, (uint8_t)79, (uint8_t)212, (uint8_t)23, (uint8_t)130, (uint8_t)91, (uint8_t)224, (uint8_t)61, (uint8_t)24, (uint8_t)1, (uint8_t)57, (uint8_t)140, (uint8_t)201, (uint8_t)129, (uint8_t)201, (uint8_t)200, (uint8_t)61, (uint8_t)116, (uint8_t)169, (uint8_t)72, (uint8_t)32, (uint8_t)139, (uint8_t)66, (uint8_t)4, (uint8_t)135, (uint8_t)9, (uint8_t)68, (uint8_t)232, (uint8_t)162, (uint8_t)237, (uint8_t)164, (uint8_t)251, (uint8_t)104, (uint8_t)181, (uint8_t)3, (uint8_t)35, (uint8_t)236, (uint8_t)19, (uint8_t)133, (uint8_t)138, (uint8_t)152, (uint8_t)142, (uint8_t)161, (uint8_t)186, (uint8_t)191, (uint8_t)173, (uint8_t)52, (uint8_t)117, (uint8_t)172, (uint8_t)235, (uint8_t)234, (uint8_t)212, (uint8_t)163, (uint8_t)4, (uint8_t)103, (uint8_t)0, (uint8_t)29, (uint8_t)5, (uint8_t)10, (uint8_t)246, (uint8_t)227, (uint8_t)188, (uint8_t)233, (uint8_t)54, (uint8_t)168, (uint8_t)51, (uint8_t)133, (uint8_t)112, (uint8_t)184, (uint8_t)18, (uint8_t)239, (uint8_t)176, (uint8_t)83, (uint8_t)142, (uint8_t)253, (uint8_t)49, (uint8_t)150, (uint8_t)184, (uint8_t)155, (uint8_t)199, (uint8_t)127, (uint8_t)244, (uint8_t)93, (uint8_t)218, (uint8_t)88, (uint8_t)208, (uint8_t)89, (uint8_t)221, (uint8_t)120, (uint8_t)68, (uint8_t)101, (uint8_t)114, (uint8_t)57, (uint8_t)154, (uint8_t)208, (uint8_t)28, (uint8_t)10, (uint8_t)164, (uint8_t)54, (uint8_t)119, (uint8_t)234, (uint8_t)145, (uint8_t)151, (uint8_t)5, (uint8_t)201, (uint8_t)83, (uint8_t)126, (uint8_t)175, (uint8_t)62, (uint8_t)15, (uint8_t)249, (uint8_t)164, (uint8_t)58, (uint8_t)196, (uint8_t)75, (uint8_t)226, (uint8_t)33, (uint8_t)137, (uint8_t)205, (uint8_t)53, (uint8_t)232, (uint8_t)211, (uint8_t)249, (uint8_t)118, (uint8_t)41, (uint8_t)15, (uint8_t)13, (uint8_t)66, (uint8_t)203, (uint8_t)216, (uint8_t)238, (uint8_t)17, (uint8_t)164, (uint8_t)236, (uint8_t)180, (uint8_t)216, (uint8_t)189, (uint8_t)38, (uint8_t)139, (uint8_t)13, (uint8_t)46, (uint8_t)43, (uint8_t)38, (uint8_t)175, (uint8_t)98, (uint8_t)200, (uint8_t)18, (uint8_t)162, (uint8_t)203, (uint8_t)1, (uint8_t)70, (uint8_t)169, (uint8_t)253, (uint8_t)240, (uint8_t)177, (uint8_t)152, (uint8_t)17, (uint8_t)244, (uint8_t)173, (uint8_t)167, (uint8_t)91, (uint8_t)114, (uint8_t)255, (uint8_t)139, (uint8_t)158, (uint8_t)84, (uint8_t)248, (uint8_t)0, (uint8_t)255, (uint8_t)32, (uint8_t)95, (uint8_t)89, (uint8_t)185, (uint8_t)189, (uint8_t)98, (uint8_t)183, (uint8_t)48, (uint8_t)182, (uint8_t)181, (uint8_t)110, (uint8_t)224, (uint8_t)129, (uint8_t)179, (uint8_t)87, (uint8_t)186, (uint8_t)105, (uint8_t)86, (uint8_t)66, (uint8_t)181, (uint8_t)213, (uint8_t)57, (uint8_t)60, (uint8_t)3, (uint8_t)18, (uint8_t)202, (uint8_t)15, (uint8_t)207, (uint8_t)181, (uint8_t)230, (uint8_t)154, (uint8_t)121, (uint8_t)247, (uint8_t)10, (uint8_t)34, (uint8_t)120, (uint8_t)99, (uint8_t)58, (uint8_t)205, (uint8_t)9, (uint8_t)34, (uint8_t)182, (uint8_t)179, (uint8_t)180, (uint8_t)21, (uint8_t)97, (uint8_t)62, (uint8_t)21, (uint8_t)12, (uint8_t)195, (uint8_t)244, (uint8_t)249, (uint8_t)71, (uint8_t)254, (uint8_t)20, (uint8_t)221, (uint8_t)77, (uint8_t)147, (uint8_t)47, (uint8_t)243, (uint8_t)177, (uint8_t)128, (uint8_t)21, (uint8_t)240, (uint8_t)52, (uint8_t)129, (uint8_t)236, (uint8_t)179, (uint8_t)122};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_ACK_268(), &PH);
        p268_target_component_SET((uint8_t)(uint8_t)180, PH.base.pack) ;
        p268_target_system_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p268_sequence_SET((uint16_t)(uint16_t)56912, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        {
            char16_t* uri = u"zwgnrspnixasggzrktasmjmlypgKqcysPyMdeQfbhiexc";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_framerate_SET((float) -5.248207E37F, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)37754, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)26, PH.base.pack) ;
        p269_resolution_h_SET((uint16_t)(uint16_t)7049, PH.base.pack) ;
        p269_bitrate_SET((uint32_t)2943415683L, PH.base.pack) ;
        p269_status_SET((uint8_t)(uint8_t)14, PH.base.pack) ;
        p269_rotation_SET((uint16_t)(uint16_t)26772, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_resolution_h_SET((uint16_t)(uint16_t)60374, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        {
            char16_t* uri = u"rtpknhnrqcjtztrdyZbxeQdukvutidlPRTLutnammjcbkurbycO";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_target_component_SET((uint8_t)(uint8_t)69, PH.base.pack) ;
        p270_framerate_SET((float)1.0923621E38F, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)34394L, PH.base.pack) ;
        p270_camera_id_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)61473, PH.base.pack) ;
        p270_rotation_SET((uint16_t)(uint16_t)44776, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* password = u"hxayzwjqqXmKlmlyaaolbyhufphjbbgwyhmzkootdhlxXdsn";
            p299_password_SET_(password, &PH) ;
        }
        {
            char16_t* ssid = u"faqaCfhohtftshfqcwbsn";
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
            uint8_t spec_version_hash[] =  {(uint8_t)175, (uint8_t)84, (uint8_t)197, (uint8_t)172, (uint8_t)225, (uint8_t)135, (uint8_t)215, (uint8_t)170};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        p300_max_version_SET((uint16_t)(uint16_t)18478, PH.base.pack) ;
        p300_version_SET((uint16_t)(uint16_t)6938, PH.base.pack) ;
        {
            uint8_t library_version_hash[] =  {(uint8_t)212, (uint8_t)94, (uint8_t)105, (uint8_t)59, (uint8_t)162, (uint8_t)218, (uint8_t)11, (uint8_t)79};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        p300_min_version_SET((uint16_t)(uint16_t)6296, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_ERROR, PH.base.pack) ;
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)25635, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)1383335241753802820L, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)1750481280L, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_INFO_311(), &PH);
        {
            char16_t* name = u"wxdz";
            p311_name_SET_(name, &PH) ;
        }
        p311_uptime_sec_SET((uint32_t)2583241550L, PH.base.pack) ;
        p311_time_usec_SET((uint64_t)6509381760313854201L, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)2912440542L, PH.base.pack) ;
        p311_sw_version_minor_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
        p311_hw_version_major_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        p311_hw_version_minor_SET((uint8_t)(uint8_t)176, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)70, (uint8_t)112, (uint8_t)13, (uint8_t)10, (uint8_t)54, (uint8_t)117, (uint8_t)74, (uint8_t)191, (uint8_t)88, (uint8_t)200, (uint8_t)101, (uint8_t)224, (uint8_t)91, (uint8_t)149, (uint8_t)107, (uint8_t)43};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        p311_sw_version_major_SET((uint8_t)(uint8_t)218, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        p320_target_system_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        p320_target_component_SET((uint8_t)(uint8_t)21, PH.base.pack) ;
        {
            char16_t* param_id = u"fwcdiAcjxEaxek";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_param_index_SET((int16_t)(int16_t)1771, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_component_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        p321_target_system_SET((uint8_t)(uint8_t)102, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_VALUE_322(), &PH);
        {
            char16_t* param_value = u"ifbbnqeltbPpkzobqupcqlfDYxvh";
            p322_param_value_SET_(param_value, &PH) ;
        }
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT8, PH.base.pack) ;
        {
            char16_t* param_id = u"wqaf";
            p322_param_id_SET_(param_id, &PH) ;
        }
        p322_param_index_SET((uint16_t)(uint16_t)59969, PH.base.pack) ;
        p322_param_count_SET((uint16_t)(uint16_t)12660, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_SET_323(), &PH);
        p323_target_system_SET((uint8_t)(uint8_t)148, PH.base.pack) ;
        {
            char16_t* param_id = u"lfTFIoUgwjjfC";
            p323_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"mNKumvftnuLlivAokzqpayacuRksccqkvtxhnpgml";
            p323_param_value_SET_(param_value, &PH) ;
        }
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64, PH.base.pack) ;
        p323_target_component_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_ACK_324(), &PH);
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL32, PH.base.pack) ;
        {
            char16_t* param_value = u"FjpvmsidzhosmwpivmonyzOtxknrppggjoyanluiyVxtjspgqzwwihopVkyhalyertfznYYrqiafdklrpz";
            p324_param_value_SET_(param_value, &PH) ;
        }
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_ACCEPTED, PH.base.pack) ;
        {
            char16_t* param_id = u"sWhqZP";
            p324_param_id_SET_(param_id, &PH) ;
        }
        c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OBSTACLE_DISTANCE_330(), &PH);
        p330_time_usec_SET((uint64_t)4805460286946828669L, PH.base.pack) ;
        {
            uint16_t distances[] =  {(uint16_t)733, (uint16_t)51595, (uint16_t)31374, (uint16_t)38572, (uint16_t)23728, (uint16_t)47971, (uint16_t)41859, (uint16_t)9423, (uint16_t)47691, (uint16_t)46149, (uint16_t)58397, (uint16_t)29320, (uint16_t)23297, (uint16_t)46489, (uint16_t)22381, (uint16_t)11280, (uint16_t)39870, (uint16_t)503, (uint16_t)17841, (uint16_t)13023, (uint16_t)47512, (uint16_t)49349, (uint16_t)50381, (uint16_t)47027, (uint16_t)1706, (uint16_t)32564, (uint16_t)45945, (uint16_t)494, (uint16_t)135, (uint16_t)61009, (uint16_t)56914, (uint16_t)25610, (uint16_t)47494, (uint16_t)31413, (uint16_t)726, (uint16_t)55121, (uint16_t)23645, (uint16_t)41927, (uint16_t)16948, (uint16_t)54331, (uint16_t)58723, (uint16_t)5232, (uint16_t)39229, (uint16_t)20252, (uint16_t)50612, (uint16_t)57336, (uint16_t)62637, (uint16_t)64000, (uint16_t)35122, (uint16_t)37824, (uint16_t)15970, (uint16_t)19698, (uint16_t)8136, (uint16_t)58941, (uint16_t)57820, (uint16_t)48068, (uint16_t)12376, (uint16_t)16168, (uint16_t)267, (uint16_t)42962, (uint16_t)58852, (uint16_t)17237, (uint16_t)33070, (uint16_t)34205, (uint16_t)15845, (uint16_t)50719, (uint16_t)24497, (uint16_t)13037, (uint16_t)51674, (uint16_t)22550, (uint16_t)5385, (uint16_t)51354};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_increment_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        p330_max_distance_SET((uint16_t)(uint16_t)51621, PH.base.pack) ;
        p330_min_distance_SET((uint16_t)(uint16_t)41363, PH.base.pack) ;
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
}

