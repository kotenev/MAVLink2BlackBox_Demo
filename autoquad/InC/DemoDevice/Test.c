
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
    assert(p0_autopilot_GET(pack) == e_MAV_AUTOPILOT_MAV_AUTOPILOT_PX4);
    assert(p0_type_GET(pack) == e_MAV_TYPE_MAV_TYPE_GCS);
    assert(p0_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED);
    assert(p0_mavlink_version_GET(pack) == (uint8_t)(uint8_t)126);
    assert(p0_system_status_GET(pack) == e_MAV_STATE_MAV_STATE_BOOT);
    assert(p0_custom_mode_GET(pack) == (uint32_t)1049918282L);
};


void c_LoopBackDemoChannel_on_SYS_STATUS_1(Bounds_Inside * ph, Pack * pack)
{
    assert(p1_load_GET(pack) == (uint16_t)(uint16_t)9147);
    assert(p1_errors_count3_GET(pack) == (uint16_t)(uint16_t)63080);
    assert(p1_onboard_control_sensors_present_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE);
    assert(p1_errors_count4_GET(pack) == (uint16_t)(uint16_t)15473);
    assert(p1_errors_count1_GET(pack) == (uint16_t)(uint16_t)47809);
    assert(p1_battery_remaining_GET(pack) == (int8_t)(int8_t)5);
    assert(p1_drop_rate_comm_GET(pack) == (uint16_t)(uint16_t)61491);
    assert(p1_errors_comm_GET(pack) == (uint16_t)(uint16_t)33807);
    assert(p1_voltage_battery_GET(pack) == (uint16_t)(uint16_t)32444);
    assert(p1_onboard_control_sensors_health_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE);
    assert(p1_current_battery_GET(pack) == (int16_t)(int16_t) -31306);
    assert(p1_onboard_control_sensors_enabled_GET(pack) == e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2);
    assert(p1_errors_count2_GET(pack) == (uint16_t)(uint16_t)39560);
};


void c_LoopBackDemoChannel_on_SYSTEM_TIME_2(Bounds_Inside * ph, Pack * pack)
{
    assert(p2_time_unix_usec_GET(pack) == (uint64_t)2866755814010049597L);
    assert(p2_time_boot_ms_GET(pack) == (uint32_t)1699224830L);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(Bounds_Inside * ph, Pack * pack)
{
    assert(p3_z_GET(pack) == (float)1.2844377E38F);
    assert(p3_afy_GET(pack) == (float) -1.120691E38F);
    assert(p3_vz_GET(pack) == (float)8.247461E37F);
    assert(p3_x_GET(pack) == (float) -1.520683E38F);
    assert(p3_yaw_rate_GET(pack) == (float) -2.4803243E38F);
    assert(p3_afx_GET(pack) == (float) -3.262818E38F);
    assert(p3_vx_GET(pack) == (float)3.227991E37F);
    assert(p3_y_GET(pack) == (float) -8.768712E37F);
    assert(p3_vy_GET(pack) == (float)1.2315801E37F);
    assert(p3_afz_GET(pack) == (float)3.050678E38F);
    assert(p3_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
    assert(p3_time_boot_ms_GET(pack) == (uint32_t)3038678426L);
    assert(p3_yaw_GET(pack) == (float) -3.0333384E38F);
    assert(p3_type_mask_GET(pack) == (uint16_t)(uint16_t)1531);
};


void c_LoopBackDemoChannel_on_PING_4(Bounds_Inside * ph, Pack * pack)
{
    assert(p4_seq_GET(pack) == (uint32_t)2576619764L);
    assert(p4_time_usec_GET(pack) == (uint64_t)712765046417073337L);
    assert(p4_target_component_GET(pack) == (uint8_t)(uint8_t)144);
    assert(p4_target_system_GET(pack) == (uint8_t)(uint8_t)44);
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(Bounds_Inside * ph, Pack * pack)
{
    assert(p5_version_GET(pack) == (uint8_t)(uint8_t)119);
    assert(p5_control_request_GET(pack) == (uint8_t)(uint8_t)60);
    assert(p5_target_system_GET(pack) == (uint8_t)(uint8_t)45);
    assert(p5_passkey_LEN(ph) == 1);
    {
        char16_t * exemplary = u"m";
        char16_t * sample = p5_passkey_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(Bounds_Inside * ph, Pack * pack)
{
    assert(p6_ack_GET(pack) == (uint8_t)(uint8_t)234);
    assert(p6_gcs_system_id_GET(pack) == (uint8_t)(uint8_t)55);
    assert(p6_control_request_GET(pack) == (uint8_t)(uint8_t)17);
};


void c_LoopBackDemoChannel_on_AUTH_KEY_7(Bounds_Inside * ph, Pack * pack)
{
    assert(p7_key_LEN(ph) == 5);
    {
        char16_t * exemplary = u"hottc";
        char16_t * sample = p7_key_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 10);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SET_MODE_11(Bounds_Inside * ph, Pack * pack)
{
    assert(p11_base_mode_GET(pack) == e_MAV_MODE_MAV_MODE_AUTO_ARMED);
    assert(p11_custom_mode_GET(pack) == (uint32_t)2339979983L);
    assert(p11_target_system_GET(pack) == (uint8_t)(uint8_t)53);
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(Bounds_Inside * ph, Pack * pack)
{
    assert(p20_param_index_GET(pack) == (int16_t)(int16_t)21305);
    assert(p20_target_system_GET(pack) == (uint8_t)(uint8_t)35);
    assert(p20_target_component_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p20_param_id_LEN(ph) == 12);
    {
        char16_t * exemplary = u"SqBkxkfabwwH";
        char16_t * sample = p20_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 24);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(Bounds_Inside * ph, Pack * pack)
{
    assert(p21_target_component_GET(pack) == (uint8_t)(uint8_t)192);
    assert(p21_target_system_GET(pack) == (uint8_t)(uint8_t)135);
};


void c_LoopBackDemoChannel_on_PARAM_VALUE_22(Bounds_Inside * ph, Pack * pack)
{
    assert(p22_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64);
    assert(p22_param_id_LEN(ph) == 11);
    {
        char16_t * exemplary = u"hkygDresnxh";
        char16_t * sample = p22_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 22);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p22_param_count_GET(pack) == (uint16_t)(uint16_t)60886);
    assert(p22_param_value_GET(pack) == (float)1.6750919E38F);
    assert(p22_param_index_GET(pack) == (uint16_t)(uint16_t)20142);
};


void c_LoopBackDemoChannel_on_PARAM_SET_23(Bounds_Inside * ph, Pack * pack)
{
    assert(p23_param_value_GET(pack) == (float) -1.436232E38F);
    assert(p23_param_type_GET(pack) == e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT16);
    assert(p23_param_id_LEN(ph) == 10);
    {
        char16_t * exemplary = u"hnykawjqtm";
        char16_t * sample = p23_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p23_target_component_GET(pack) == (uint8_t)(uint8_t)129);
    assert(p23_target_system_GET(pack) == (uint8_t)(uint8_t)171);
};


void c_LoopBackDemoChannel_on_GPS_RAW_INT_24(Bounds_Inside * ph, Pack * pack)
{
    assert(p24_alt_ellipsoid_TRY(ph) == (int32_t) -797997768);
    assert(p24_time_usec_GET(pack) == (uint64_t)3870957820941811675L);
    assert(p24_h_acc_TRY(ph) == (uint32_t)177513135L);
    assert(p24_lat_GET(pack) == (int32_t)676362669);
    assert(p24_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_3D_FIX);
    assert(p24_v_acc_TRY(ph) == (uint32_t)3377061317L);
    assert(p24_epv_GET(pack) == (uint16_t)(uint16_t)13371);
    assert(p24_hdg_acc_TRY(ph) == (uint32_t)1144430534L);
    assert(p24_alt_GET(pack) == (int32_t)1058443856);
    assert(p24_cog_GET(pack) == (uint16_t)(uint16_t)20831);
    assert(p24_lon_GET(pack) == (int32_t) -1058936255);
    assert(p24_vel_acc_TRY(ph) == (uint32_t)4120125257L);
    assert(p24_vel_GET(pack) == (uint16_t)(uint16_t)26147);
    assert(p24_satellites_visible_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p24_eph_GET(pack) == (uint16_t)(uint16_t)63128);
};


void c_LoopBackDemoChannel_on_GPS_STATUS_25(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)179, (uint8_t)118, (uint8_t)63, (uint8_t)125, (uint8_t)90, (uint8_t)128, (uint8_t)150, (uint8_t)235, (uint8_t)128, (uint8_t)49, (uint8_t)242, (uint8_t)99, (uint8_t)251, (uint8_t)109, (uint8_t)147, (uint8_t)77, (uint8_t)140, (uint8_t)6, (uint8_t)72, (uint8_t)246} ;
        uint8_t*  sample = p25_satellite_elevation_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p25_satellites_visible_GET(pack) == (uint8_t)(uint8_t)232);
    {
        uint8_t exemplary[] =  {(uint8_t)93, (uint8_t)111, (uint8_t)50, (uint8_t)226, (uint8_t)60, (uint8_t)156, (uint8_t)157, (uint8_t)61, (uint8_t)23, (uint8_t)251, (uint8_t)172, (uint8_t)39, (uint8_t)103, (uint8_t)13, (uint8_t)124, (uint8_t)71, (uint8_t)157, (uint8_t)188, (uint8_t)221, (uint8_t)212} ;
        uint8_t*  sample = p25_satellite_prn_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)219, (uint8_t)222, (uint8_t)74, (uint8_t)229, (uint8_t)63, (uint8_t)60, (uint8_t)122, (uint8_t)119, (uint8_t)164, (uint8_t)252, (uint8_t)97, (uint8_t)155, (uint8_t)196, (uint8_t)134, (uint8_t)234, (uint8_t)225, (uint8_t)134, (uint8_t)108, (uint8_t)143, (uint8_t)14} ;
        uint8_t*  sample = p25_satellite_azimuth_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)6, (uint8_t)197, (uint8_t)5, (uint8_t)131, (uint8_t)176, (uint8_t)171, (uint8_t)101, (uint8_t)95, (uint8_t)69, (uint8_t)42, (uint8_t)241, (uint8_t)210, (uint8_t)237, (uint8_t)119, (uint8_t)120, (uint8_t)61, (uint8_t)197, (uint8_t)6, (uint8_t)169, (uint8_t)85} ;
        uint8_t*  sample = p25_satellite_snr_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)241, (uint8_t)145, (uint8_t)17, (uint8_t)112, (uint8_t)74, (uint8_t)147, (uint8_t)177, (uint8_t)42, (uint8_t)151, (uint8_t)111, (uint8_t)111, (uint8_t)218, (uint8_t)111, (uint8_t)174, (uint8_t)47, (uint8_t)58, (uint8_t)192, (uint8_t)76, (uint8_t)115, (uint8_t)198} ;
        uint8_t*  sample = p25_satellite_used_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_SCALED_IMU_26(Bounds_Inside * ph, Pack * pack)
{
    assert(p26_ymag_GET(pack) == (int16_t)(int16_t) -8727);
    assert(p26_zacc_GET(pack) == (int16_t)(int16_t) -919);
    assert(p26_time_boot_ms_GET(pack) == (uint32_t)3554329856L);
    assert(p26_xmag_GET(pack) == (int16_t)(int16_t)7331);
    assert(p26_zgyro_GET(pack) == (int16_t)(int16_t)25902);
    assert(p26_ygyro_GET(pack) == (int16_t)(int16_t) -11714);
    assert(p26_yacc_GET(pack) == (int16_t)(int16_t) -31597);
    assert(p26_xacc_GET(pack) == (int16_t)(int16_t)12650);
    assert(p26_zmag_GET(pack) == (int16_t)(int16_t) -23423);
    assert(p26_xgyro_GET(pack) == (int16_t)(int16_t)29201);
};


void c_LoopBackDemoChannel_on_RAW_IMU_27(Bounds_Inside * ph, Pack * pack)
{
    assert(p27_time_usec_GET(pack) == (uint64_t)4262991439716535167L);
    assert(p27_xacc_GET(pack) == (int16_t)(int16_t)32154);
    assert(p27_xmag_GET(pack) == (int16_t)(int16_t)16911);
    assert(p27_ygyro_GET(pack) == (int16_t)(int16_t) -29201);
    assert(p27_zacc_GET(pack) == (int16_t)(int16_t)12139);
    assert(p27_zgyro_GET(pack) == (int16_t)(int16_t) -22460);
    assert(p27_zmag_GET(pack) == (int16_t)(int16_t)22928);
    assert(p27_xgyro_GET(pack) == (int16_t)(int16_t)30990);
    assert(p27_yacc_GET(pack) == (int16_t)(int16_t)20950);
    assert(p27_ymag_GET(pack) == (int16_t)(int16_t) -5916);
};


void c_LoopBackDemoChannel_on_RAW_PRESSURE_28(Bounds_Inside * ph, Pack * pack)
{
    assert(p28_press_diff1_GET(pack) == (int16_t)(int16_t)13304);
    assert(p28_time_usec_GET(pack) == (uint64_t)4199173096179369917L);
    assert(p28_press_abs_GET(pack) == (int16_t)(int16_t) -4922);
    assert(p28_press_diff2_GET(pack) == (int16_t)(int16_t)8052);
    assert(p28_temperature_GET(pack) == (int16_t)(int16_t)15327);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(Bounds_Inside * ph, Pack * pack)
{
    assert(p29_time_boot_ms_GET(pack) == (uint32_t)1026136409L);
    assert(p29_temperature_GET(pack) == (int16_t)(int16_t) -16337);
    assert(p29_press_abs_GET(pack) == (float) -1.981177E38F);
    assert(p29_press_diff_GET(pack) == (float)2.806208E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_30(Bounds_Inside * ph, Pack * pack)
{
    assert(p30_time_boot_ms_GET(pack) == (uint32_t)3230702190L);
    assert(p30_yaw_GET(pack) == (float) -1.1991395E38F);
    assert(p30_yawspeed_GET(pack) == (float)1.5575115E38F);
    assert(p30_pitchspeed_GET(pack) == (float) -1.3189831E38F);
    assert(p30_pitch_GET(pack) == (float)1.8977837E38F);
    assert(p30_roll_GET(pack) == (float) -3.2297646E38F);
    assert(p30_rollspeed_GET(pack) == (float) -9.429545E37F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(Bounds_Inside * ph, Pack * pack)
{
    assert(p31_time_boot_ms_GET(pack) == (uint32_t)521451827L);
    assert(p31_yawspeed_GET(pack) == (float) -2.9115855E38F);
    assert(p31_pitchspeed_GET(pack) == (float)1.588116E38F);
    assert(p31_rollspeed_GET(pack) == (float)4.455984E37F);
    assert(p31_q3_GET(pack) == (float) -2.6038704E37F);
    assert(p31_q2_GET(pack) == (float) -6.3507054E37F);
    assert(p31_q1_GET(pack) == (float) -2.6826912E38F);
    assert(p31_q4_GET(pack) == (float) -3.2979093E38F);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(Bounds_Inside * ph, Pack * pack)
{
    assert(p32_vy_GET(pack) == (float)3.8153177E37F);
    assert(p32_time_boot_ms_GET(pack) == (uint32_t)1460047116L);
    assert(p32_z_GET(pack) == (float) -2.2222982E38F);
    assert(p32_vx_GET(pack) == (float) -1.6260522E38F);
    assert(p32_y_GET(pack) == (float)2.524929E38F);
    assert(p32_vz_GET(pack) == (float) -1.569816E38F);
    assert(p32_x_GET(pack) == (float) -5.590345E37F);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(Bounds_Inside * ph, Pack * pack)
{
    assert(p33_lat_GET(pack) == (int32_t)71022867);
    assert(p33_relative_alt_GET(pack) == (int32_t) -320849642);
    assert(p33_lon_GET(pack) == (int32_t)1187969523);
    assert(p33_vz_GET(pack) == (int16_t)(int16_t) -349);
    assert(p33_alt_GET(pack) == (int32_t) -2055596479);
    assert(p33_vx_GET(pack) == (int16_t)(int16_t) -22381);
    assert(p33_time_boot_ms_GET(pack) == (uint32_t)3273940559L);
    assert(p33_hdg_GET(pack) == (uint16_t)(uint16_t)50326);
    assert(p33_vy_GET(pack) == (int16_t)(int16_t) -5430);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(Bounds_Inside * ph, Pack * pack)
{
    assert(p34_chan6_scaled_GET(pack) == (int16_t)(int16_t) -18018);
    assert(p34_chan2_scaled_GET(pack) == (int16_t)(int16_t)4646);
    assert(p34_port_GET(pack) == (uint8_t)(uint8_t)34);
    assert(p34_chan8_scaled_GET(pack) == (int16_t)(int16_t) -312);
    assert(p34_rssi_GET(pack) == (uint8_t)(uint8_t)50);
    assert(p34_chan1_scaled_GET(pack) == (int16_t)(int16_t)4463);
    assert(p34_chan5_scaled_GET(pack) == (int16_t)(int16_t) -27511);
    assert(p34_chan7_scaled_GET(pack) == (int16_t)(int16_t) -25389);
    assert(p34_time_boot_ms_GET(pack) == (uint32_t)1188757378L);
    assert(p34_chan3_scaled_GET(pack) == (int16_t)(int16_t) -18488);
    assert(p34_chan4_scaled_GET(pack) == (int16_t)(int16_t) -7111);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(Bounds_Inside * ph, Pack * pack)
{
    assert(p35_chan7_raw_GET(pack) == (uint16_t)(uint16_t)57296);
    assert(p35_time_boot_ms_GET(pack) == (uint32_t)1302107391L);
    assert(p35_chan1_raw_GET(pack) == (uint16_t)(uint16_t)49939);
    assert(p35_chan3_raw_GET(pack) == (uint16_t)(uint16_t)30874);
    assert(p35_chan8_raw_GET(pack) == (uint16_t)(uint16_t)33051);
    assert(p35_chan2_raw_GET(pack) == (uint16_t)(uint16_t)29717);
    assert(p35_port_GET(pack) == (uint8_t)(uint8_t)189);
    assert(p35_chan6_raw_GET(pack) == (uint16_t)(uint16_t)63730);
    assert(p35_rssi_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p35_chan5_raw_GET(pack) == (uint16_t)(uint16_t)27673);
    assert(p35_chan4_raw_GET(pack) == (uint16_t)(uint16_t)61073);
};


void c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(Bounds_Inside * ph, Pack * pack)
{
    assert(p36_servo10_raw_TRY(ph) == (uint16_t)(uint16_t)56959);
    assert(p36_servo15_raw_TRY(ph) == (uint16_t)(uint16_t)11477);
    assert(p36_servo9_raw_TRY(ph) == (uint16_t)(uint16_t)34337);
    assert(p36_servo1_raw_GET(pack) == (uint16_t)(uint16_t)41276);
    assert(p36_servo14_raw_TRY(ph) == (uint16_t)(uint16_t)10154);
    assert(p36_servo3_raw_GET(pack) == (uint16_t)(uint16_t)26079);
    assert(p36_servo6_raw_GET(pack) == (uint16_t)(uint16_t)58305);
    assert(p36_servo13_raw_TRY(ph) == (uint16_t)(uint16_t)63391);
    assert(p36_servo12_raw_TRY(ph) == (uint16_t)(uint16_t)37724);
    assert(p36_servo5_raw_GET(pack) == (uint16_t)(uint16_t)44698);
    assert(p36_port_GET(pack) == (uint8_t)(uint8_t)164);
    assert(p36_servo7_raw_GET(pack) == (uint16_t)(uint16_t)28326);
    assert(p36_servo2_raw_GET(pack) == (uint16_t)(uint16_t)32962);
    assert(p36_servo4_raw_GET(pack) == (uint16_t)(uint16_t)59338);
    assert(p36_servo11_raw_TRY(ph) == (uint16_t)(uint16_t)25061);
    assert(p36_time_usec_GET(pack) == (uint32_t)3062002973L);
    assert(p36_servo8_raw_GET(pack) == (uint16_t)(uint16_t)65280);
    assert(p36_servo16_raw_TRY(ph) == (uint16_t)(uint16_t)43522);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(Bounds_Inside * ph, Pack * pack)
{
    assert(p37_end_index_GET(pack) == (int16_t)(int16_t)20090);
    assert(p37_start_index_GET(pack) == (int16_t)(int16_t) -22054);
    assert(p37_target_system_GET(pack) == (uint8_t)(uint8_t)146);
    assert(p37_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p37_target_component_GET(pack) == (uint8_t)(uint8_t)11);
};


void c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(Bounds_Inside * ph, Pack * pack)
{
    assert(p38_target_system_GET(pack) == (uint8_t)(uint8_t)41);
    assert(p38_start_index_GET(pack) == (int16_t)(int16_t)8280);
    assert(p38_target_component_GET(pack) == (uint8_t)(uint8_t)66);
    assert(p38_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p38_end_index_GET(pack) == (int16_t)(int16_t)31157);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_39(Bounds_Inside * ph, Pack * pack)
{
    assert(p39_param3_GET(pack) == (float) -2.0781213E38F);
    assert(p39_param4_GET(pack) == (float) -1.1201141E38F);
    assert(p39_x_GET(pack) == (float) -2.3123584E38F);
    assert(p39_target_system_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p39_current_GET(pack) == (uint8_t)(uint8_t)38);
    assert(p39_param1_GET(pack) == (float)2.9931727E38F);
    assert(p39_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
    assert(p39_seq_GET(pack) == (uint16_t)(uint16_t)29244);
    assert(p39_command_GET(pack) == e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE);
    assert(p39_target_component_GET(pack) == (uint8_t)(uint8_t)194);
    assert(p39_z_GET(pack) == (float)1.2396276E38F);
    assert(p39_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p39_param2_GET(pack) == (float)2.9011494E38F);
    assert(p39_autocontinue_GET(pack) == (uint8_t)(uint8_t)51);
    assert(p39_y_GET(pack) == (float) -1.0301921E38F);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_40(Bounds_Inside * ph, Pack * pack)
{
    assert(p40_seq_GET(pack) == (uint16_t)(uint16_t)53438);
    assert(p40_target_system_GET(pack) == (uint8_t)(uint8_t)113);
    assert(p40_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY);
    assert(p40_target_component_GET(pack) == (uint8_t)(uint8_t)228);
};


void c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(Bounds_Inside * ph, Pack * pack)
{
    assert(p41_target_system_GET(pack) == (uint8_t)(uint8_t)23);
    assert(p41_seq_GET(pack) == (uint16_t)(uint16_t)36538);
    assert(p41_target_component_GET(pack) == (uint8_t)(uint8_t)139);
};


void c_LoopBackDemoChannel_on_MISSION_CURRENT_42(Bounds_Inside * ph, Pack * pack)
{
    assert(p42_seq_GET(pack) == (uint16_t)(uint16_t)18361);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(Bounds_Inside * ph, Pack * pack)
{
    assert(p43_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p43_target_system_GET(pack) == (uint8_t)(uint8_t)124);
    assert(p43_target_component_GET(pack) == (uint8_t)(uint8_t)245);
};


void c_LoopBackDemoChannel_on_MISSION_COUNT_44(Bounds_Inside * ph, Pack * pack)
{
    assert(p44_count_GET(pack) == (uint16_t)(uint16_t)59093);
    assert(p44_target_component_GET(pack) == (uint8_t)(uint8_t)43);
    assert(p44_target_system_GET(pack) == (uint8_t)(uint8_t)113);
    assert(p44_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
};


void c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(Bounds_Inside * ph, Pack * pack)
{
    assert(p45_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL);
    assert(p45_target_system_GET(pack) == (uint8_t)(uint8_t)150);
    assert(p45_target_component_GET(pack) == (uint8_t)(uint8_t)186);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(Bounds_Inside * ph, Pack * pack)
{
    assert(p46_seq_GET(pack) == (uint16_t)(uint16_t)5439);
};


void c_LoopBackDemoChannel_on_MISSION_ACK_47(Bounds_Inside * ph, Pack * pack)
{
    assert(p47_target_system_GET(pack) == (uint8_t)(uint8_t)145);
    assert(p47_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p47_target_component_GET(pack) == (uint8_t)(uint8_t)249);
    assert(p47_type_GET(pack) == e_MAV_MISSION_RESULT_MAV_MISSION_ERROR);
};


void c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(Bounds_Inside * ph, Pack * pack)
{
    assert(p48_latitude_GET(pack) == (int32_t)819597432);
    assert(p48_target_system_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p48_time_usec_TRY(ph) == (uint64_t)1560590423219178449L);
    assert(p48_altitude_GET(pack) == (int32_t)285354055);
    assert(p48_longitude_GET(pack) == (int32_t)2023987836);
};


void c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(Bounds_Inside * ph, Pack * pack)
{
    assert(p49_latitude_GET(pack) == (int32_t) -230262978);
    assert(p49_altitude_GET(pack) == (int32_t)2068706508);
    assert(p49_time_usec_TRY(ph) == (uint64_t)5341489511699149884L);
    assert(p49_longitude_GET(pack) == (int32_t)1694997785);
};


void c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(Bounds_Inside * ph, Pack * pack)
{
    assert(p50_scale_GET(pack) == (float)6.4341376E37F);
    assert(p50_param_index_GET(pack) == (int16_t)(int16_t)13257);
    assert(p50_param_value_max_GET(pack) == (float)2.1340624E38F);
    assert(p50_target_component_GET(pack) == (uint8_t)(uint8_t)152);
    assert(p50_param_id_LEN(ph) == 7);
    {
        char16_t * exemplary = u"wdgbatp";
        char16_t * sample = p50_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 14);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p50_target_system_GET(pack) == (uint8_t)(uint8_t)16);
    assert(p50_param_value_min_GET(pack) == (float) -2.5075824E38F);
    assert(p50_parameter_rc_channel_index_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p50_param_value0_GET(pack) == (float)2.9497071E38F);
};


void c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(Bounds_Inside * ph, Pack * pack)
{
    assert(p51_target_system_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p51_target_component_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p51_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE);
    assert(p51_seq_GET(pack) == (uint16_t)(uint16_t)10183);
};


void c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(Bounds_Inside * ph, Pack * pack)
{
    assert(p54_p2x_GET(pack) == (float) -2.2655082E38F);
    assert(p54_target_component_GET(pack) == (uint8_t)(uint8_t)9);
    assert(p54_p2y_GET(pack) == (float) -2.356907E38F);
    assert(p54_p1x_GET(pack) == (float) -2.458482E38F);
    assert(p54_p2z_GET(pack) == (float)1.280244E38F);
    assert(p54_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
    assert(p54_p1y_GET(pack) == (float) -1.5139381E38F);
    assert(p54_p1z_GET(pack) == (float)7.183131E37F);
    assert(p54_target_system_GET(pack) == (uint8_t)(uint8_t)241);
};


void c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(Bounds_Inside * ph, Pack * pack)
{
    assert(p55_p1y_GET(pack) == (float)1.8335525E36F);
    assert(p55_p1z_GET(pack) == (float) -1.3211999E38F);
    assert(p55_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_MISSION);
    assert(p55_p1x_GET(pack) == (float)3.1880812E38F);
    assert(p55_p2y_GET(pack) == (float)1.7684036E38F);
    assert(p55_p2z_GET(pack) == (float) -3.2079162E38F);
    assert(p55_p2x_GET(pack) == (float)1.0294272E38F);
};


void c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {2.5429765E38F, -3.0958062E38F, 1.4453513E38F, -2.0220005E38F} ;
        float*  sample = p61_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p61_rollspeed_GET(pack) == (float)1.5946163E38F);
    assert(p61_pitchspeed_GET(pack) == (float)1.4041678E38F);
    assert(p61_yawspeed_GET(pack) == (float)2.2078362E38F);
    assert(p61_time_usec_GET(pack) == (uint64_t)4557277817191516737L);
    {
        float exemplary[] =  {-7.9688436E37F, -1.995715E38F, -2.8615918E38F, 2.5187067E38F, 2.2813417E38F, -2.4655808E38F, 1.7365714E38F, -1.4099172E38F, 2.89197E37F} ;
        float*  sample = p61_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 36);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(Bounds_Inside * ph, Pack * pack)
{
    assert(p62_wp_dist_GET(pack) == (uint16_t)(uint16_t)21072);
    assert(p62_xtrack_error_GET(pack) == (float) -1.1641062E38F);
    assert(p62_nav_roll_GET(pack) == (float)1.0784313E37F);
    assert(p62_aspd_error_GET(pack) == (float)2.2851343E38F);
    assert(p62_nav_bearing_GET(pack) == (int16_t)(int16_t)23481);
    assert(p62_nav_pitch_GET(pack) == (float) -3.177426E38F);
    assert(p62_alt_error_GET(pack) == (float)1.8946113E36F);
    assert(p62_target_bearing_GET(pack) == (int16_t)(int16_t)16954);
};


void c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(Bounds_Inside * ph, Pack * pack)
{
    assert(p63_alt_GET(pack) == (int32_t)1784679152);
    assert(p63_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION);
    assert(p63_relative_alt_GET(pack) == (int32_t) -1114806170);
    {
        float exemplary[] =  {-4.4139756E37F, 4.95664E37F, -3.3115695E38F, -2.3735017E38F, -4.910613E37F, -7.549652E37F, -3.7433793E37F, 3.1568327E38F, -2.5738007E38F, 2.3172371E38F, 2.015421E38F, 3.0249184E38F, 1.0963625E38F, -8.1080945E37F, -1.5863199E38F, 3.2128602E38F, 7.696945E37F, -2.2344772E38F, 2.2865078E38F, 2.6473856E38F, -9.359695E37F, 2.2067546E38F, -7.073953E37F, 9.535361E37F, -5.8196287E37F, -3.2803595E38F, 9.60527E37F, -3.085046E38F, -1.5248177E38F, -3.0635016E38F, -1.3656424E38F, 2.9009456E38F, -2.869833E37F, 1.887977E38F, -2.5728796E38F, -1.2924686E38F} ;
        float*  sample = p63_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p63_vy_GET(pack) == (float) -1.290597E38F);
    assert(p63_vz_GET(pack) == (float) -3.206567E38F);
    assert(p63_time_usec_GET(pack) == (uint64_t)1696269504734581763L);
    assert(p63_lat_GET(pack) == (int32_t)254193721);
    assert(p63_vx_GET(pack) == (float) -3.2187066E38F);
    assert(p63_lon_GET(pack) == (int32_t) -1598349347);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(Bounds_Inside * ph, Pack * pack)
{
    assert(p64_vx_GET(pack) == (float)1.9360306E38F);
    assert(p64_az_GET(pack) == (float)3.2193117E38F);
    assert(p64_ay_GET(pack) == (float)9.831133E37F);
    assert(p64_time_usec_GET(pack) == (uint64_t)6729142783798613563L);
    assert(p64_y_GET(pack) == (float) -1.2412373E38F);
    assert(p64_ax_GET(pack) == (float) -8.0941174E37F);
    assert(p64_z_GET(pack) == (float)1.6203988E38F);
    {
        float exemplary[] =  {1.9100783E38F, 1.3856443E37F, -2.1717323E38F, 2.6390321E38F, -7.882638E37F, -1.3621049E37F, 2.5585536E38F, -2.2996652E38F, -1.1000763E38F, -2.5857519E38F, 2.7153737E38F, -7.108533E37F, -3.6102567E37F, 2.3102013E38F, -1.2287822E38F, -2.3875548E38F, 2.4230099E38F, -3.0173043E38F, 2.9549254E38F, -2.68777E38F, 2.4467168E38F, -3.2307873E38F, -3.2094611E38F, -1.2852244E38F, 4.4495124E37F, 2.4155863E38F, 1.452198E38F, 3.1149903E38F, 2.906713E38F, -3.1156223E38F, 7.8380114E37F, 2.9541378E37F, -1.4614942E38F, -6.571612E36F, -2.9302184E38F, -9.235883E37F, 1.0889615E38F, 2.2689374E38F, -1.1433816E38F, 3.3651008E38F, -3.3220924E38F, 1.3261823E38F, -1.2062161E38F, -1.5454036E38F, -1.3811962E38F} ;
        float*  sample = p64_covariance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p64_vz_GET(pack) == (float) -3.2733917E38F);
    assert(p64_x_GET(pack) == (float)2.235812E38F);
    assert(p64_estimator_type_GET(pack) == e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS);
    assert(p64_vy_GET(pack) == (float)1.2671179E38F);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_65(Bounds_Inside * ph, Pack * pack)
{
    assert(p65_chan7_raw_GET(pack) == (uint16_t)(uint16_t)32006);
    assert(p65_chan9_raw_GET(pack) == (uint16_t)(uint16_t)33767);
    assert(p65_chan3_raw_GET(pack) == (uint16_t)(uint16_t)28695);
    assert(p65_chan10_raw_GET(pack) == (uint16_t)(uint16_t)21759);
    assert(p65_chancount_GET(pack) == (uint8_t)(uint8_t)247);
    assert(p65_chan12_raw_GET(pack) == (uint16_t)(uint16_t)51595);
    assert(p65_chan6_raw_GET(pack) == (uint16_t)(uint16_t)6952);
    assert(p65_chan15_raw_GET(pack) == (uint16_t)(uint16_t)19192);
    assert(p65_chan18_raw_GET(pack) == (uint16_t)(uint16_t)31671);
    assert(p65_chan14_raw_GET(pack) == (uint16_t)(uint16_t)51829);
    assert(p65_chan17_raw_GET(pack) == (uint16_t)(uint16_t)11657);
    assert(p65_chan1_raw_GET(pack) == (uint16_t)(uint16_t)30573);
    assert(p65_rssi_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p65_chan16_raw_GET(pack) == (uint16_t)(uint16_t)1935);
    assert(p65_chan8_raw_GET(pack) == (uint16_t)(uint16_t)25936);
    assert(p65_chan4_raw_GET(pack) == (uint16_t)(uint16_t)24233);
    assert(p65_chan2_raw_GET(pack) == (uint16_t)(uint16_t)61640);
    assert(p65_chan13_raw_GET(pack) == (uint16_t)(uint16_t)42103);
    assert(p65_time_boot_ms_GET(pack) == (uint32_t)2076074457L);
    assert(p65_chan11_raw_GET(pack) == (uint16_t)(uint16_t)54972);
    assert(p65_chan5_raw_GET(pack) == (uint16_t)(uint16_t)10968);
};


void c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(Bounds_Inside * ph, Pack * pack)
{
    assert(p66_req_stream_id_GET(pack) == (uint8_t)(uint8_t)88);
    assert(p66_start_stop_GET(pack) == (uint8_t)(uint8_t)84);
    assert(p66_req_message_rate_GET(pack) == (uint16_t)(uint16_t)14964);
    assert(p66_target_component_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p66_target_system_GET(pack) == (uint8_t)(uint8_t)36);
};


void c_LoopBackDemoChannel_on_DATA_STREAM_67(Bounds_Inside * ph, Pack * pack)
{
    assert(p67_stream_id_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p67_message_rate_GET(pack) == (uint16_t)(uint16_t)62593);
    assert(p67_on_off_GET(pack) == (uint8_t)(uint8_t)23);
};


void c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(Bounds_Inside * ph, Pack * pack)
{
    assert(p69_z_GET(pack) == (int16_t)(int16_t) -27150);
    assert(p69_y_GET(pack) == (int16_t)(int16_t) -7474);
    assert(p69_x_GET(pack) == (int16_t)(int16_t)21695);
    assert(p69_buttons_GET(pack) == (uint16_t)(uint16_t)18541);
    assert(p69_r_GET(pack) == (int16_t)(int16_t)609);
    assert(p69_target_GET(pack) == (uint8_t)(uint8_t)55);
};


void c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(Bounds_Inside * ph, Pack * pack)
{
    assert(p70_chan3_raw_GET(pack) == (uint16_t)(uint16_t)42158);
    assert(p70_chan4_raw_GET(pack) == (uint16_t)(uint16_t)14068);
    assert(p70_chan6_raw_GET(pack) == (uint16_t)(uint16_t)49064);
    assert(p70_target_system_GET(pack) == (uint8_t)(uint8_t)135);
    assert(p70_target_component_GET(pack) == (uint8_t)(uint8_t)235);
    assert(p70_chan2_raw_GET(pack) == (uint16_t)(uint16_t)8266);
    assert(p70_chan5_raw_GET(pack) == (uint16_t)(uint16_t)38145);
    assert(p70_chan1_raw_GET(pack) == (uint16_t)(uint16_t)6223);
    assert(p70_chan7_raw_GET(pack) == (uint16_t)(uint16_t)21598);
    assert(p70_chan8_raw_GET(pack) == (uint16_t)(uint16_t)46873);
};


void c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(Bounds_Inside * ph, Pack * pack)
{
    assert(p73_mission_type_GET(pack) == e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION);
    assert(p73_target_component_GET(pack) == (uint8_t)(uint8_t)123);
    assert(p73_seq_GET(pack) == (uint16_t)(uint16_t)41133);
    assert(p73_param4_GET(pack) == (float) -1.1746958E38F);
    assert(p73_param3_GET(pack) == (float)8.1348394E37F);
    assert(p73_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_NED);
    assert(p73_param2_GET(pack) == (float)1.763342E38F);
    assert(p73_param1_GET(pack) == (float) -1.0318666E38F);
    assert(p73_current_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p73_y_GET(pack) == (int32_t) -384970639);
    assert(p73_autocontinue_GET(pack) == (uint8_t)(uint8_t)248);
    assert(p73_x_GET(pack) == (int32_t)457104418);
    assert(p73_command_GET(pack) == e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST);
    assert(p73_z_GET(pack) == (float) -2.261985E37F);
    assert(p73_target_system_GET(pack) == (uint8_t)(uint8_t)10);
};


void c_LoopBackDemoChannel_on_VFR_HUD_74(Bounds_Inside * ph, Pack * pack)
{
    assert(p74_heading_GET(pack) == (int16_t)(int16_t)6428);
    assert(p74_throttle_GET(pack) == (uint16_t)(uint16_t)25086);
    assert(p74_climb_GET(pack) == (float)2.7028922E37F);
    assert(p74_groundspeed_GET(pack) == (float)2.9130227E38F);
    assert(p74_airspeed_GET(pack) == (float)1.488647E38F);
    assert(p74_alt_GET(pack) == (float) -1.8107079E38F);
};


void c_LoopBackDemoChannel_on_COMMAND_INT_75(Bounds_Inside * ph, Pack * pack)
{
    assert(p75_param3_GET(pack) == (float) -2.6514046E38F);
    assert(p75_param2_GET(pack) == (float)2.1713186E38F);
    assert(p75_target_system_GET(pack) == (uint8_t)(uint8_t)125);
    assert(p75_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT);
    assert(p75_target_component_GET(pack) == (uint8_t)(uint8_t)144);
    assert(p75_current_GET(pack) == (uint8_t)(uint8_t)205);
    assert(p75_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_INT);
    assert(p75_param4_GET(pack) == (float) -3.1203885E38F);
    assert(p75_param1_GET(pack) == (float) -1.3446609E38F);
    assert(p75_x_GET(pack) == (int32_t)1324108846);
    assert(p75_y_GET(pack) == (int32_t)1075102028);
    assert(p75_z_GET(pack) == (float)2.3221515E38F);
    assert(p75_autocontinue_GET(pack) == (uint8_t)(uint8_t)232);
};


void c_LoopBackDemoChannel_on_COMMAND_LONG_76(Bounds_Inside * ph, Pack * pack)
{
    assert(p76_param2_GET(pack) == (float)1.1503348E38F);
    assert(p76_confirmation_GET(pack) == (uint8_t)(uint8_t)229);
    assert(p76_command_GET(pack) == e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE);
    assert(p76_target_system_GET(pack) == (uint8_t)(uint8_t)237);
    assert(p76_param7_GET(pack) == (float) -2.280676E38F);
    assert(p76_param1_GET(pack) == (float)5.073007E37F);
    assert(p76_target_component_GET(pack) == (uint8_t)(uint8_t)213);
    assert(p76_param6_GET(pack) == (float) -9.864486E37F);
    assert(p76_param4_GET(pack) == (float) -2.2863172E38F);
    assert(p76_param5_GET(pack) == (float) -2.283216E38F);
    assert(p76_param3_GET(pack) == (float)5.1063887E37F);
};


void c_LoopBackDemoChannel_on_COMMAND_ACK_77(Bounds_Inside * ph, Pack * pack)
{
    assert(p77_result_param2_TRY(ph) == (int32_t) -397415415);
    assert(p77_target_system_TRY(ph) == (uint8_t)(uint8_t)161);
    assert(p77_result_GET(pack) == e_MAV_RESULT_MAV_RESULT_DENIED);
    assert(p77_command_GET(pack) == e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE);
    assert(p77_progress_TRY(ph) == (uint8_t)(uint8_t)98);
    assert(p77_target_component_TRY(ph) == (uint8_t)(uint8_t)120);
};


void c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(Bounds_Inside * ph, Pack * pack)
{
    assert(p81_thrust_GET(pack) == (float)4.745377E37F);
    assert(p81_time_boot_ms_GET(pack) == (uint32_t)2819108460L);
    assert(p81_mode_switch_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p81_pitch_GET(pack) == (float)2.3060536E38F);
    assert(p81_roll_GET(pack) == (float)2.8737327E38F);
    assert(p81_yaw_GET(pack) == (float)1.2125277E37F);
    assert(p81_manual_override_switch_GET(pack) == (uint8_t)(uint8_t)205);
};


void c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(Bounds_Inside * ph, Pack * pack)
{
    assert(p82_time_boot_ms_GET(pack) == (uint32_t)1088010633L);
    assert(p82_body_yaw_rate_GET(pack) == (float)6.308009E37F);
    {
        float exemplary[] =  {-8.2529257E37F, -3.3775904E37F, -2.2504481E38F, 2.2121318E38F} ;
        float*  sample = p82_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p82_thrust_GET(pack) == (float) -2.8103286E38F);
    assert(p82_body_pitch_rate_GET(pack) == (float) -2.4320187E38F);
    assert(p82_target_component_GET(pack) == (uint8_t)(uint8_t)210);
    assert(p82_body_roll_rate_GET(pack) == (float) -8.2261245E37F);
    assert(p82_type_mask_GET(pack) == (uint8_t)(uint8_t)160);
    assert(p82_target_system_GET(pack) == (uint8_t)(uint8_t)73);
};


void c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {2.38169E38F, 3.368664E37F, 2.1720453E38F, -1.8085595E38F} ;
        float*  sample = p83_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p83_time_boot_ms_GET(pack) == (uint32_t)1918543860L);
    assert(p83_body_pitch_rate_GET(pack) == (float)2.973246E38F);
    assert(p83_body_roll_rate_GET(pack) == (float)2.069942E38F);
    assert(p83_type_mask_GET(pack) == (uint8_t)(uint8_t)66);
    assert(p83_thrust_GET(pack) == (float)1.6516483E38F);
    assert(p83_body_yaw_rate_GET(pack) == (float)4.6206113E37F);
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(Bounds_Inside * ph, Pack * pack)
{
    assert(p84_afy_GET(pack) == (float)2.7902158E38F);
    assert(p84_yaw_rate_GET(pack) == (float) -5.089831E37F);
    assert(p84_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_LOCAL_ENU);
    assert(p84_type_mask_GET(pack) == (uint16_t)(uint16_t)6474);
    assert(p84_target_system_GET(pack) == (uint8_t)(uint8_t)164);
    assert(p84_z_GET(pack) == (float)4.233198E37F);
    assert(p84_vz_GET(pack) == (float) -3.3074945E38F);
    assert(p84_vx_GET(pack) == (float) -1.1577264E38F);
    assert(p84_y_GET(pack) == (float) -1.2771864E38F);
    assert(p84_vy_GET(pack) == (float) -2.1639883E38F);
    assert(p84_yaw_GET(pack) == (float) -6.323792E37F);
    assert(p84_x_GET(pack) == (float) -1.7227414E38F);
    assert(p84_target_component_GET(pack) == (uint8_t)(uint8_t)239);
    assert(p84_time_boot_ms_GET(pack) == (uint32_t)2658475703L);
    assert(p84_afx_GET(pack) == (float) -1.7586983E38F);
    assert(p84_afz_GET(pack) == (float) -2.0274664E38F);
};


void c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(Bounds_Inside * ph, Pack * pack)
{
    assert(p86_vz_GET(pack) == (float)3.3494753E38F);
    assert(p86_lon_int_GET(pack) == (int32_t)499560938);
    assert(p86_afy_GET(pack) == (float) -2.7556426E38F);
    assert(p86_afz_GET(pack) == (float) -3.3710194E38F);
    assert(p86_target_component_GET(pack) == (uint8_t)(uint8_t)192);
    assert(p86_type_mask_GET(pack) == (uint16_t)(uint16_t)18752);
    assert(p86_yaw_rate_GET(pack) == (float) -8.7315355E36F);
    assert(p86_alt_GET(pack) == (float) -7.3626257E37F);
    assert(p86_target_system_GET(pack) == (uint8_t)(uint8_t)144);
    assert(p86_lat_int_GET(pack) == (int32_t) -1181322210);
    assert(p86_vy_GET(pack) == (float)1.405424E38F);
    assert(p86_afx_GET(pack) == (float)1.3053582E38F);
    assert(p86_vx_GET(pack) == (float) -7.5745835E37F);
    assert(p86_yaw_GET(pack) == (float) -1.6896969E38F);
    assert(p86_time_boot_ms_GET(pack) == (uint32_t)3627599026L);
    assert(p86_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_NED);
};


void c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(Bounds_Inside * ph, Pack * pack)
{
    assert(p87_lat_int_GET(pack) == (int32_t) -1211673698);
    assert(p87_time_boot_ms_GET(pack) == (uint32_t)200068824L);
    assert(p87_lon_int_GET(pack) == (int32_t)398218012);
    assert(p87_afy_GET(pack) == (float)1.0812786E38F);
    assert(p87_vz_GET(pack) == (float)2.795697E38F);
    assert(p87_coordinate_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT);
    assert(p87_afx_GET(pack) == (float) -7.802306E37F);
    assert(p87_type_mask_GET(pack) == (uint16_t)(uint16_t)63169);
    assert(p87_vx_GET(pack) == (float) -2.9204505E37F);
    assert(p87_vy_GET(pack) == (float) -1.3135944E38F);
    assert(p87_yaw_rate_GET(pack) == (float)1.411404E36F);
    assert(p87_alt_GET(pack) == (float) -4.5382317E37F);
    assert(p87_yaw_GET(pack) == (float) -1.4687577E38F);
    assert(p87_afz_GET(pack) == (float) -2.9063581E38F);
};


void c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(Bounds_Inside * ph, Pack * pack)
{
    assert(p89_yaw_GET(pack) == (float) -3.1550854E38F);
    assert(p89_z_GET(pack) == (float) -2.6272358E37F);
    assert(p89_time_boot_ms_GET(pack) == (uint32_t)3742540708L);
    assert(p89_pitch_GET(pack) == (float) -2.9035711E38F);
    assert(p89_x_GET(pack) == (float) -2.5437294E38F);
    assert(p89_roll_GET(pack) == (float)3.4227474E37F);
    assert(p89_y_GET(pack) == (float)2.4648E38F);
};


void c_LoopBackDemoChannel_on_HIL_STATE_90(Bounds_Inside * ph, Pack * pack)
{
    assert(p90_lat_GET(pack) == (int32_t) -69594771);
    assert(p90_pitch_GET(pack) == (float)2.47912E38F);
    assert(p90_vx_GET(pack) == (int16_t)(int16_t)3773);
    assert(p90_yawspeed_GET(pack) == (float)3.3615238E38F);
    assert(p90_rollspeed_GET(pack) == (float)2.118666E38F);
    assert(p90_yacc_GET(pack) == (int16_t)(int16_t)2787);
    assert(p90_pitchspeed_GET(pack) == (float)1.0225286E38F);
    assert(p90_roll_GET(pack) == (float)3.363573E38F);
    assert(p90_vy_GET(pack) == (int16_t)(int16_t)15914);
    assert(p90_lon_GET(pack) == (int32_t) -1030616578);
    assert(p90_time_usec_GET(pack) == (uint64_t)2574986204201894911L);
    assert(p90_yaw_GET(pack) == (float)2.6184335E38F);
    assert(p90_alt_GET(pack) == (int32_t) -400225197);
    assert(p90_vz_GET(pack) == (int16_t)(int16_t)4231);
    assert(p90_zacc_GET(pack) == (int16_t)(int16_t) -5715);
    assert(p90_xacc_GET(pack) == (int16_t)(int16_t) -13543);
};


void c_LoopBackDemoChannel_on_HIL_CONTROLS_91(Bounds_Inside * ph, Pack * pack)
{
    assert(p91_aux2_GET(pack) == (float)2.8563236E37F);
    assert(p91_nav_mode_GET(pack) == (uint8_t)(uint8_t)253);
    assert(p91_throttle_GET(pack) == (float)2.2216806E37F);
    assert(p91_time_usec_GET(pack) == (uint64_t)2734626562126311039L);
    assert(p91_roll_ailerons_GET(pack) == (float)4.224699E36F);
    assert(p91_mode_GET(pack) == e_MAV_MODE_MAV_MODE_MANUAL_DISARMED);
    assert(p91_aux4_GET(pack) == (float)2.948039E38F);
    assert(p91_yaw_rudder_GET(pack) == (float)1.5010709E38F);
    assert(p91_aux3_GET(pack) == (float)8.0259467E37F);
    assert(p91_pitch_elevator_GET(pack) == (float)4.3908887E37F);
    assert(p91_aux1_GET(pack) == (float)1.4066972E38F);
};


void c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(Bounds_Inside * ph, Pack * pack)
{
    assert(p92_rssi_GET(pack) == (uint8_t)(uint8_t)75);
    assert(p92_chan11_raw_GET(pack) == (uint16_t)(uint16_t)52924);
    assert(p92_chan12_raw_GET(pack) == (uint16_t)(uint16_t)12451);
    assert(p92_chan1_raw_GET(pack) == (uint16_t)(uint16_t)34224);
    assert(p92_chan3_raw_GET(pack) == (uint16_t)(uint16_t)42452);
    assert(p92_chan4_raw_GET(pack) == (uint16_t)(uint16_t)58865);
    assert(p92_chan8_raw_GET(pack) == (uint16_t)(uint16_t)28208);
    assert(p92_chan2_raw_GET(pack) == (uint16_t)(uint16_t)2936);
    assert(p92_chan7_raw_GET(pack) == (uint16_t)(uint16_t)4480);
    assert(p92_chan5_raw_GET(pack) == (uint16_t)(uint16_t)34055);
    assert(p92_chan9_raw_GET(pack) == (uint16_t)(uint16_t)6334);
    assert(p92_chan10_raw_GET(pack) == (uint16_t)(uint16_t)11679);
    assert(p92_chan6_raw_GET(pack) == (uint16_t)(uint16_t)46379);
    assert(p92_time_usec_GET(pack) == (uint64_t)7415270434051196649L);
};


void c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(Bounds_Inside * ph, Pack * pack)
{
    assert(p93_flags_GET(pack) == (uint64_t)4607800801507066774L);
    assert(p93_mode_GET(pack) == e_MAV_MODE_MAV_MODE_PREFLIGHT);
    {
        float exemplary[] =  {1.9394939E38F, -2.9320075E38F, 1.063194E38F, 2.8619427E38F, 2.3207466E38F, 7.906272E37F, 8.188366E37F, 1.286874E38F, 6.6889293E37F, 4.7776833E37F, -2.6134268E38F, 3.1612354E38F, 9.474035E37F, -2.9014766E38F, -3.8819515E36F, -2.657319E38F} ;
        float*  sample = p93_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p93_time_usec_GET(pack) == (uint64_t)1547823720855461510L);
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(Bounds_Inside * ph, Pack * pack)
{
    assert(p100_flow_y_GET(pack) == (int16_t)(int16_t)9536);
    assert(p100_flow_comp_m_x_GET(pack) == (float)8.530277E37F);
    assert(p100_ground_distance_GET(pack) == (float)9.116103E37F);
    assert(p100_flow_x_GET(pack) == (int16_t)(int16_t)32252);
    assert(p100_flow_rate_y_TRY(ph) == (float)2.181095E38F);
    assert(p100_flow_rate_x_TRY(ph) == (float)5.466814E37F);
    assert(p100_quality_GET(pack) == (uint8_t)(uint8_t)10);
    assert(p100_flow_comp_m_y_GET(pack) == (float) -1.2198762E38F);
    assert(p100_sensor_id_GET(pack) == (uint8_t)(uint8_t)222);
    assert(p100_time_usec_GET(pack) == (uint64_t)2243135110625647390L);
};


void c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(Bounds_Inside * ph, Pack * pack)
{
    assert(p101_usec_GET(pack) == (uint64_t)4656021596355750028L);
    assert(p101_x_GET(pack) == (float)1.0047654E38F);
    assert(p101_y_GET(pack) == (float)1.729738E38F);
    assert(p101_z_GET(pack) == (float) -1.334857E38F);
    assert(p101_roll_GET(pack) == (float) -5.2296895E37F);
    assert(p101_yaw_GET(pack) == (float)1.1315915E38F);
    assert(p101_pitch_GET(pack) == (float) -1.6403878E38F);
};


void c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(Bounds_Inside * ph, Pack * pack)
{
    assert(p102_pitch_GET(pack) == (float)1.0061344E38F);
    assert(p102_x_GET(pack) == (float)1.9799307E37F);
    assert(p102_roll_GET(pack) == (float)1.3800967E38F);
    assert(p102_y_GET(pack) == (float) -8.253037E37F);
    assert(p102_z_GET(pack) == (float)5.640685E37F);
    assert(p102_yaw_GET(pack) == (float) -2.2576754E38F);
    assert(p102_usec_GET(pack) == (uint64_t)2835164604771048530L);
};


void c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(Bounds_Inside * ph, Pack * pack)
{
    assert(p103_x_GET(pack) == (float) -4.1911642E37F);
    assert(p103_z_GET(pack) == (float)1.6068335E38F);
    assert(p103_y_GET(pack) == (float)2.5352097E38F);
    assert(p103_usec_GET(pack) == (uint64_t)2009143334684140421L);
};


void c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(Bounds_Inside * ph, Pack * pack)
{
    assert(p104_pitch_GET(pack) == (float) -3.2640796E38F);
    assert(p104_usec_GET(pack) == (uint64_t)6937791749803696301L);
    assert(p104_roll_GET(pack) == (float) -5.2660767E36F);
    assert(p104_x_GET(pack) == (float)2.2688287E38F);
    assert(p104_z_GET(pack) == (float)2.3344788E38F);
    assert(p104_yaw_GET(pack) == (float) -2.9904375E37F);
    assert(p104_y_GET(pack) == (float) -1.1264268E38F);
};


void c_LoopBackDemoChannel_on_HIGHRES_IMU_105(Bounds_Inside * ph, Pack * pack)
{
    assert(p105_zgyro_GET(pack) == (float) -4.5612305E37F);
    assert(p105_diff_pressure_GET(pack) == (float) -3.2082434E38F);
    assert(p105_time_usec_GET(pack) == (uint64_t)2161745166960261516L);
    assert(p105_abs_pressure_GET(pack) == (float) -1.0061351E38F);
    assert(p105_zmag_GET(pack) == (float)2.0084388E38F);
    assert(p105_yacc_GET(pack) == (float) -3.3677566E38F);
    assert(p105_pressure_alt_GET(pack) == (float)1.0832265E38F);
    assert(p105_xacc_GET(pack) == (float) -2.7348734E38F);
    assert(p105_temperature_GET(pack) == (float)1.215084E38F);
    assert(p105_xmag_GET(pack) == (float)2.5524727E38F);
    assert(p105_xgyro_GET(pack) == (float)1.0116622E38F);
    assert(p105_zacc_GET(pack) == (float) -2.6983282E38F);
    assert(p105_ymag_GET(pack) == (float) -9.302353E37F);
    assert(p105_fields_updated_GET(pack) == (uint16_t)(uint16_t)7520);
    assert(p105_ygyro_GET(pack) == (float) -1.013746E38F);
};


void c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(Bounds_Inside * ph, Pack * pack)
{
    assert(p106_integrated_x_GET(pack) == (float)1.0003097E38F);
    assert(p106_temperature_GET(pack) == (int16_t)(int16_t) -927);
    assert(p106_distance_GET(pack) == (float) -1.6147104E37F);
    assert(p106_time_usec_GET(pack) == (uint64_t)2299149947706698470L);
    assert(p106_integrated_ygyro_GET(pack) == (float)7.015535E37F);
    assert(p106_integrated_y_GET(pack) == (float)5.684749E37F);
    assert(p106_quality_GET(pack) == (uint8_t)(uint8_t)90);
    assert(p106_integrated_zgyro_GET(pack) == (float) -1.3117128E38F);
    assert(p106_sensor_id_GET(pack) == (uint8_t)(uint8_t)116);
    assert(p106_integration_time_us_GET(pack) == (uint32_t)2706554044L);
    assert(p106_time_delta_distance_us_GET(pack) == (uint32_t)1583334171L);
    assert(p106_integrated_xgyro_GET(pack) == (float) -3.2075059E38F);
};


void c_LoopBackDemoChannel_on_HIL_SENSOR_107(Bounds_Inside * ph, Pack * pack)
{
    assert(p107_temperature_GET(pack) == (float) -1.5614414E38F);
    assert(p107_abs_pressure_GET(pack) == (float)2.1201649E38F);
    assert(p107_ygyro_GET(pack) == (float)8.052983E37F);
    assert(p107_fields_updated_GET(pack) == (uint32_t)924085616L);
    assert(p107_xgyro_GET(pack) == (float)8.267211E37F);
    assert(p107_zacc_GET(pack) == (float)5.607017E37F);
    assert(p107_yacc_GET(pack) == (float)1.8484615E38F);
    assert(p107_diff_pressure_GET(pack) == (float)6.147163E36F);
    assert(p107_zgyro_GET(pack) == (float)3.8278175E37F);
    assert(p107_xmag_GET(pack) == (float)1.142581E38F);
    assert(p107_zmag_GET(pack) == (float) -1.3445148E38F);
    assert(p107_time_usec_GET(pack) == (uint64_t)386675838609781693L);
    assert(p107_xacc_GET(pack) == (float) -1.9102101E38F);
    assert(p107_pressure_alt_GET(pack) == (float) -1.0833513E38F);
    assert(p107_ymag_GET(pack) == (float)2.5613626E37F);
};


void c_LoopBackDemoChannel_on_SIM_STATE_108(Bounds_Inside * ph, Pack * pack)
{
    assert(p108_zacc_GET(pack) == (float) -2.735521E38F);
    assert(p108_ygyro_GET(pack) == (float) -2.3967841E38F);
    assert(p108_q1_GET(pack) == (float)1.06766E38F);
    assert(p108_xacc_GET(pack) == (float) -1.0856249E38F);
    assert(p108_xgyro_GET(pack) == (float) -2.9972438E38F);
    assert(p108_q3_GET(pack) == (float)1.6591408E38F);
    assert(p108_q4_GET(pack) == (float) -6.422069E37F);
    assert(p108_vd_GET(pack) == (float) -5.603438E37F);
    assert(p108_alt_GET(pack) == (float) -2.1835355E38F);
    assert(p108_lon_GET(pack) == (float) -2.8209361E38F);
    assert(p108_roll_GET(pack) == (float)4.6358363E37F);
    assert(p108_zgyro_GET(pack) == (float) -2.281411E38F);
    assert(p108_yaw_GET(pack) == (float)1.3542695E38F);
    assert(p108_pitch_GET(pack) == (float) -2.0083405E38F);
    assert(p108_q2_GET(pack) == (float)1.901965E38F);
    assert(p108_ve_GET(pack) == (float)1.6569638E38F);
    assert(p108_vn_GET(pack) == (float)2.5671137E37F);
    assert(p108_lat_GET(pack) == (float)1.800846E38F);
    assert(p108_std_dev_vert_GET(pack) == (float) -8.0053383E37F);
    assert(p108_std_dev_horz_GET(pack) == (float) -3.3226917E38F);
    assert(p108_yacc_GET(pack) == (float)1.6103025E38F);
};


void c_LoopBackDemoChannel_on_RADIO_STATUS_109(Bounds_Inside * ph, Pack * pack)
{
    assert(p109_rssi_GET(pack) == (uint8_t)(uint8_t)1);
    assert(p109_txbuf_GET(pack) == (uint8_t)(uint8_t)178);
    assert(p109_noise_GET(pack) == (uint8_t)(uint8_t)241);
    assert(p109_rxerrors_GET(pack) == (uint16_t)(uint16_t)59158);
    assert(p109_fixed__GET(pack) == (uint16_t)(uint16_t)54647);
    assert(p109_remnoise_GET(pack) == (uint8_t)(uint8_t)97);
    assert(p109_remrssi_GET(pack) == (uint8_t)(uint8_t)139);
};


void c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(Bounds_Inside * ph, Pack * pack)
{
    assert(p110_target_system_GET(pack) == (uint8_t)(uint8_t)78);
    assert(p110_target_component_GET(pack) == (uint8_t)(uint8_t)189);
    {
        uint8_t exemplary[] =  {(uint8_t)231, (uint8_t)138, (uint8_t)221, (uint8_t)199, (uint8_t)112, (uint8_t)173, (uint8_t)30, (uint8_t)21, (uint8_t)123, (uint8_t)193, (uint8_t)253, (uint8_t)114, (uint8_t)212, (uint8_t)149, (uint8_t)142, (uint8_t)176, (uint8_t)252, (uint8_t)24, (uint8_t)82, (uint8_t)102, (uint8_t)117, (uint8_t)156, (uint8_t)4, (uint8_t)201, (uint8_t)37, (uint8_t)27, (uint8_t)120, (uint8_t)74, (uint8_t)1, (uint8_t)119, (uint8_t)79, (uint8_t)237, (uint8_t)53, (uint8_t)129, (uint8_t)46, (uint8_t)18, (uint8_t)235, (uint8_t)9, (uint8_t)101, (uint8_t)44, (uint8_t)87, (uint8_t)12, (uint8_t)198, (uint8_t)181, (uint8_t)15, (uint8_t)106, (uint8_t)25, (uint8_t)203, (uint8_t)152, (uint8_t)198, (uint8_t)172, (uint8_t)142, (uint8_t)214, (uint8_t)70, (uint8_t)25, (uint8_t)111, (uint8_t)114, (uint8_t)137, (uint8_t)136, (uint8_t)213, (uint8_t)2, (uint8_t)157, (uint8_t)82, (uint8_t)200, (uint8_t)203, (uint8_t)97, (uint8_t)59, (uint8_t)13, (uint8_t)7, (uint8_t)152, (uint8_t)69, (uint8_t)246, (uint8_t)52, (uint8_t)202, (uint8_t)6, (uint8_t)101, (uint8_t)230, (uint8_t)129, (uint8_t)79, (uint8_t)247, (uint8_t)161, (uint8_t)139, (uint8_t)5, (uint8_t)201, (uint8_t)222, (uint8_t)111, (uint8_t)154, (uint8_t)65, (uint8_t)179, (uint8_t)167, (uint8_t)255, (uint8_t)122, (uint8_t)183, (uint8_t)201, (uint8_t)190, (uint8_t)173, (uint8_t)195, (uint8_t)179, (uint8_t)86, (uint8_t)183, (uint8_t)163, (uint8_t)106, (uint8_t)13, (uint8_t)97, (uint8_t)144, (uint8_t)115, (uint8_t)186, (uint8_t)49, (uint8_t)119, (uint8_t)168, (uint8_t)237, (uint8_t)43, (uint8_t)135, (uint8_t)190, (uint8_t)155, (uint8_t)54, (uint8_t)24, (uint8_t)186, (uint8_t)184, (uint8_t)34, (uint8_t)56, (uint8_t)19, (uint8_t)211, (uint8_t)186, (uint8_t)175, (uint8_t)71, (uint8_t)95, (uint8_t)184, (uint8_t)92, (uint8_t)50, (uint8_t)206, (uint8_t)152, (uint8_t)251, (uint8_t)34, (uint8_t)134, (uint8_t)69, (uint8_t)217, (uint8_t)249, (uint8_t)70, (uint8_t)237, (uint8_t)104, (uint8_t)79, (uint8_t)183, (uint8_t)14, (uint8_t)37, (uint8_t)56, (uint8_t)253, (uint8_t)58, (uint8_t)85, (uint8_t)157, (uint8_t)69, (uint8_t)24, (uint8_t)128, (uint8_t)188, (uint8_t)128, (uint8_t)153, (uint8_t)79, (uint8_t)103, (uint8_t)135, (uint8_t)151, (uint8_t)16, (uint8_t)102, (uint8_t)82, (uint8_t)123, (uint8_t)230, (uint8_t)162, (uint8_t)124, (uint8_t)98, (uint8_t)128, (uint8_t)32, (uint8_t)188, (uint8_t)236, (uint8_t)187, (uint8_t)37, (uint8_t)214, (uint8_t)245, (uint8_t)102, (uint8_t)213, (uint8_t)240, (uint8_t)85, (uint8_t)106, (uint8_t)50, (uint8_t)12, (uint8_t)138, (uint8_t)4, (uint8_t)217, (uint8_t)254, (uint8_t)27, (uint8_t)222, (uint8_t)210, (uint8_t)92, (uint8_t)65, (uint8_t)92, (uint8_t)115, (uint8_t)189, (uint8_t)27, (uint8_t)229, (uint8_t)207, (uint8_t)84, (uint8_t)163, (uint8_t)45, (uint8_t)195, (uint8_t)161, (uint8_t)15, (uint8_t)50, (uint8_t)48, (uint8_t)18, (uint8_t)86, (uint8_t)160, (uint8_t)122, (uint8_t)124, (uint8_t)71, (uint8_t)64, (uint8_t)77, (uint8_t)121, (uint8_t)49, (uint8_t)21, (uint8_t)247, (uint8_t)92, (uint8_t)122, (uint8_t)152, (uint8_t)154, (uint8_t)64, (uint8_t)98, (uint8_t)102, (uint8_t)170, (uint8_t)221, (uint8_t)102, (uint8_t)95, (uint8_t)155, (uint8_t)207, (uint8_t)121, (uint8_t)5, (uint8_t)126, (uint8_t)31, (uint8_t)188, (uint8_t)201, (uint8_t)202, (uint8_t)207, (uint8_t)52, (uint8_t)201, (uint8_t)58, (uint8_t)177, (uint8_t)63, (uint8_t)132, (uint8_t)13, (uint8_t)36, (uint8_t)21, (uint8_t)84, (uint8_t)175, (uint8_t)241} ;
        uint8_t*  sample = p110_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 251);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p110_target_network_GET(pack) == (uint8_t)(uint8_t)33);
};


void c_LoopBackDemoChannel_on_TIMESYNC_111(Bounds_Inside * ph, Pack * pack)
{
    assert(p111_ts1_GET(pack) == (int64_t) -9189301914699662759L);
    assert(p111_tc1_GET(pack) == (int64_t)277987972536978318L);
};


void c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(Bounds_Inside * ph, Pack * pack)
{
    assert(p112_seq_GET(pack) == (uint32_t)1824200230L);
    assert(p112_time_usec_GET(pack) == (uint64_t)8448793866722366849L);
};


void c_LoopBackDemoChannel_on_HIL_GPS_113(Bounds_Inside * ph, Pack * pack)
{
    assert(p113_lon_GET(pack) == (int32_t)507422942);
    assert(p113_vel_GET(pack) == (uint16_t)(uint16_t)55492);
    assert(p113_cog_GET(pack) == (uint16_t)(uint16_t)679);
    assert(p113_ve_GET(pack) == (int16_t)(int16_t) -11817);
    assert(p113_epv_GET(pack) == (uint16_t)(uint16_t)2243);
    assert(p113_lat_GET(pack) == (int32_t) -1828498453);
    assert(p113_vd_GET(pack) == (int16_t)(int16_t)17629);
    assert(p113_satellites_visible_GET(pack) == (uint8_t)(uint8_t)4);
    assert(p113_fix_type_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p113_alt_GET(pack) == (int32_t) -1928744369);
    assert(p113_time_usec_GET(pack) == (uint64_t)5456135387074893030L);
    assert(p113_eph_GET(pack) == (uint16_t)(uint16_t)27066);
    assert(p113_vn_GET(pack) == (int16_t)(int16_t) -24298);
};


void c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(Bounds_Inside * ph, Pack * pack)
{
    assert(p114_distance_GET(pack) == (float)2.4546673E38F);
    assert(p114_time_delta_distance_us_GET(pack) == (uint32_t)2514159745L);
    assert(p114_time_usec_GET(pack) == (uint64_t)5480081855421120786L);
    assert(p114_integrated_xgyro_GET(pack) == (float) -1.1588639E38F);
    assert(p114_sensor_id_GET(pack) == (uint8_t)(uint8_t)156);
    assert(p114_integration_time_us_GET(pack) == (uint32_t)1414189521L);
    assert(p114_quality_GET(pack) == (uint8_t)(uint8_t)166);
    assert(p114_integrated_ygyro_GET(pack) == (float) -1.6357859E38F);
    assert(p114_integrated_x_GET(pack) == (float) -2.0288184E38F);
    assert(p114_integrated_y_GET(pack) == (float) -1.1921217E38F);
    assert(p114_temperature_GET(pack) == (int16_t)(int16_t)9923);
    assert(p114_integrated_zgyro_GET(pack) == (float) -2.2725984E38F);
};


void c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(Bounds_Inside * ph, Pack * pack)
{
    assert(p115_yawspeed_GET(pack) == (float)1.6874685E37F);
    assert(p115_yacc_GET(pack) == (int16_t)(int16_t)17160);
    assert(p115_vz_GET(pack) == (int16_t)(int16_t) -17079);
    assert(p115_alt_GET(pack) == (int32_t) -1818877875);
    assert(p115_xacc_GET(pack) == (int16_t)(int16_t)10069);
    {
        float exemplary[] =  {-2.9684856E38F, -7.8258055E37F, 2.4524157E38F, 3.1416625E38F} ;
        float*  sample = p115_attitude_quaternion_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p115_zacc_GET(pack) == (int16_t)(int16_t)28063);
    assert(p115_time_usec_GET(pack) == (uint64_t)7229022435494494892L);
    assert(p115_true_airspeed_GET(pack) == (uint16_t)(uint16_t)4484);
    assert(p115_vx_GET(pack) == (int16_t)(int16_t)6689);
    assert(p115_rollspeed_GET(pack) == (float) -1.6897258E38F);
    assert(p115_vy_GET(pack) == (int16_t)(int16_t)30964);
    assert(p115_ind_airspeed_GET(pack) == (uint16_t)(uint16_t)16105);
    assert(p115_pitchspeed_GET(pack) == (float) -2.8958778E38F);
    assert(p115_lat_GET(pack) == (int32_t) -805924563);
    assert(p115_lon_GET(pack) == (int32_t) -1932020664);
};


void c_LoopBackDemoChannel_on_SCALED_IMU2_116(Bounds_Inside * ph, Pack * pack)
{
    assert(p116_xgyro_GET(pack) == (int16_t)(int16_t)11080);
    assert(p116_xmag_GET(pack) == (int16_t)(int16_t)31592);
    assert(p116_yacc_GET(pack) == (int16_t)(int16_t) -22272);
    assert(p116_zacc_GET(pack) == (int16_t)(int16_t) -6844);
    assert(p116_zgyro_GET(pack) == (int16_t)(int16_t) -908);
    assert(p116_xacc_GET(pack) == (int16_t)(int16_t)26120);
    assert(p116_time_boot_ms_GET(pack) == (uint32_t)4268121612L);
    assert(p116_ymag_GET(pack) == (int16_t)(int16_t) -4712);
    assert(p116_zmag_GET(pack) == (int16_t)(int16_t) -27671);
    assert(p116_ygyro_GET(pack) == (int16_t)(int16_t)31475);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(Bounds_Inside * ph, Pack * pack)
{
    assert(p117_start_GET(pack) == (uint16_t)(uint16_t)26088);
    assert(p117_target_component_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p117_end_GET(pack) == (uint16_t)(uint16_t)850);
    assert(p117_target_system_GET(pack) == (uint8_t)(uint8_t)247);
};


void c_LoopBackDemoChannel_on_LOG_ENTRY_118(Bounds_Inside * ph, Pack * pack)
{
    assert(p118_size_GET(pack) == (uint32_t)2422541542L);
    assert(p118_id_GET(pack) == (uint16_t)(uint16_t)14188);
    assert(p118_num_logs_GET(pack) == (uint16_t)(uint16_t)61900);
    assert(p118_time_utc_GET(pack) == (uint32_t)576976743L);
    assert(p118_last_log_num_GET(pack) == (uint16_t)(uint16_t)53828);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(Bounds_Inside * ph, Pack * pack)
{
    assert(p119_target_system_GET(pack) == (uint8_t)(uint8_t)103);
    assert(p119_target_component_GET(pack) == (uint8_t)(uint8_t)132);
    assert(p119_id_GET(pack) == (uint16_t)(uint16_t)60268);
    assert(p119_ofs_GET(pack) == (uint32_t)2285789909L);
    assert(p119_count_GET(pack) == (uint32_t)2120934409L);
};


void c_LoopBackDemoChannel_on_LOG_DATA_120(Bounds_Inside * ph, Pack * pack)
{
    assert(p120_count_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p120_ofs_GET(pack) == (uint32_t)431962935L);
    {
        uint8_t exemplary[] =  {(uint8_t)74, (uint8_t)7, (uint8_t)176, (uint8_t)149, (uint8_t)204, (uint8_t)171, (uint8_t)182, (uint8_t)228, (uint8_t)105, (uint8_t)1, (uint8_t)112, (uint8_t)149, (uint8_t)4, (uint8_t)151, (uint8_t)29, (uint8_t)0, (uint8_t)238, (uint8_t)128, (uint8_t)16, (uint8_t)144, (uint8_t)171, (uint8_t)164, (uint8_t)106, (uint8_t)123, (uint8_t)161, (uint8_t)64, (uint8_t)167, (uint8_t)64, (uint8_t)90, (uint8_t)160, (uint8_t)30, (uint8_t)191, (uint8_t)167, (uint8_t)66, (uint8_t)153, (uint8_t)71, (uint8_t)108, (uint8_t)243, (uint8_t)167, (uint8_t)33, (uint8_t)192, (uint8_t)254, (uint8_t)217, (uint8_t)110, (uint8_t)154, (uint8_t)250, (uint8_t)184, (uint8_t)54, (uint8_t)38, (uint8_t)106, (uint8_t)96, (uint8_t)136, (uint8_t)139, (uint8_t)38, (uint8_t)151, (uint8_t)109, (uint8_t)192, (uint8_t)18, (uint8_t)175, (uint8_t)147, (uint8_t)106, (uint8_t)192, (uint8_t)253, (uint8_t)214, (uint8_t)108, (uint8_t)45, (uint8_t)135, (uint8_t)121, (uint8_t)62, (uint8_t)1, (uint8_t)26, (uint8_t)0, (uint8_t)18, (uint8_t)56, (uint8_t)69, (uint8_t)188, (uint8_t)54, (uint8_t)132, (uint8_t)203, (uint8_t)143, (uint8_t)212, (uint8_t)21, (uint8_t)32, (uint8_t)180, (uint8_t)233, (uint8_t)122, (uint8_t)91, (uint8_t)239, (uint8_t)89, (uint8_t)242} ;
        uint8_t*  sample = p120_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 90);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p120_id_GET(pack) == (uint16_t)(uint16_t)40940);
};


void c_LoopBackDemoChannel_on_LOG_ERASE_121(Bounds_Inside * ph, Pack * pack)
{
    assert(p121_target_system_GET(pack) == (uint8_t)(uint8_t)159);
    assert(p121_target_component_GET(pack) == (uint8_t)(uint8_t)45);
};


void c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(Bounds_Inside * ph, Pack * pack)
{
    assert(p122_target_component_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p122_target_system_GET(pack) == (uint8_t)(uint8_t)131);
};


void c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(Bounds_Inside * ph, Pack * pack)
{
    assert(p123_target_component_GET(pack) == (uint8_t)(uint8_t)175);
    {
        uint8_t exemplary[] =  {(uint8_t)144, (uint8_t)55, (uint8_t)15, (uint8_t)175, (uint8_t)173, (uint8_t)248, (uint8_t)82, (uint8_t)206, (uint8_t)220, (uint8_t)10, (uint8_t)156, (uint8_t)195, (uint8_t)35, (uint8_t)31, (uint8_t)83, (uint8_t)157, (uint8_t)197, (uint8_t)10, (uint8_t)96, (uint8_t)146, (uint8_t)158, (uint8_t)149, (uint8_t)79, (uint8_t)143, (uint8_t)22, (uint8_t)69, (uint8_t)108, (uint8_t)128, (uint8_t)255, (uint8_t)55, (uint8_t)131, (uint8_t)128, (uint8_t)58, (uint8_t)100, (uint8_t)156, (uint8_t)196, (uint8_t)140, (uint8_t)138, (uint8_t)60, (uint8_t)164, (uint8_t)206, (uint8_t)190, (uint8_t)15, (uint8_t)125, (uint8_t)80, (uint8_t)17, (uint8_t)22, (uint8_t)187, (uint8_t)249, (uint8_t)72, (uint8_t)14, (uint8_t)248, (uint8_t)142, (uint8_t)120, (uint8_t)52, (uint8_t)138, (uint8_t)119, (uint8_t)231, (uint8_t)33, (uint8_t)57, (uint8_t)84, (uint8_t)30, (uint8_t)140, (uint8_t)132, (uint8_t)217, (uint8_t)72, (uint8_t)119, (uint8_t)164, (uint8_t)125, (uint8_t)73, (uint8_t)113, (uint8_t)212, (uint8_t)66, (uint8_t)203, (uint8_t)126, (uint8_t)50, (uint8_t)172, (uint8_t)101, (uint8_t)14, (uint8_t)114, (uint8_t)106, (uint8_t)159, (uint8_t)175, (uint8_t)127, (uint8_t)114, (uint8_t)26, (uint8_t)177, (uint8_t)62, (uint8_t)16, (uint8_t)216, (uint8_t)232, (uint8_t)142, (uint8_t)128, (uint8_t)119, (uint8_t)68, (uint8_t)248, (uint8_t)202, (uint8_t)213, (uint8_t)244, (uint8_t)110, (uint8_t)2, (uint8_t)235, (uint8_t)147, (uint8_t)69, (uint8_t)70, (uint8_t)150, (uint8_t)10, (uint8_t)162, (uint8_t)96, (uint8_t)201} ;
        uint8_t*  sample = p123_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 110);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p123_target_system_GET(pack) == (uint8_t)(uint8_t)143);
    assert(p123_len_GET(pack) == (uint8_t)(uint8_t)49);
};


void c_LoopBackDemoChannel_on_GPS2_RAW_124(Bounds_Inside * ph, Pack * pack)
{
    assert(p124_time_usec_GET(pack) == (uint64_t)3983800200700004307L);
    assert(p124_vel_GET(pack) == (uint16_t)(uint16_t)52693);
    assert(p124_alt_GET(pack) == (int32_t)2129727943);
    assert(p124_eph_GET(pack) == (uint16_t)(uint16_t)20056);
    assert(p124_dgps_numch_GET(pack) == (uint8_t)(uint8_t)185);
    assert(p124_epv_GET(pack) == (uint16_t)(uint16_t)34000);
    assert(p124_lat_GET(pack) == (int32_t) -1794078652);
    assert(p124_satellites_visible_GET(pack) == (uint8_t)(uint8_t)149);
    assert(p124_lon_GET(pack) == (int32_t)3282442);
    assert(p124_cog_GET(pack) == (uint16_t)(uint16_t)55507);
    assert(p124_dgps_age_GET(pack) == (uint32_t)2298113808L);
    assert(p124_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS);
};


void c_LoopBackDemoChannel_on_POWER_STATUS_125(Bounds_Inside * ph, Pack * pack)
{
    assert(p125_Vservo_GET(pack) == (uint16_t)(uint16_t)24982);
    assert(p125_flags_GET(pack) == e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT);
    assert(p125_Vcc_GET(pack) == (uint16_t)(uint16_t)43509);
};


void c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(Bounds_Inside * ph, Pack * pack)
{
    assert(p126_device_GET(pack) == e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL);
    {
        uint8_t exemplary[] =  {(uint8_t)26, (uint8_t)178, (uint8_t)187, (uint8_t)4, (uint8_t)82, (uint8_t)215, (uint8_t)198, (uint8_t)126, (uint8_t)71, (uint8_t)83, (uint8_t)164, (uint8_t)174, (uint8_t)112, (uint8_t)194, (uint8_t)200, (uint8_t)169, (uint8_t)219, (uint8_t)114, (uint8_t)189, (uint8_t)6, (uint8_t)235, (uint8_t)63, (uint8_t)126, (uint8_t)219, (uint8_t)225, (uint8_t)29, (uint8_t)212, (uint8_t)206, (uint8_t)135, (uint8_t)12, (uint8_t)24, (uint8_t)237, (uint8_t)9, (uint8_t)45, (uint8_t)199, (uint8_t)31, (uint8_t)116, (uint8_t)230, (uint8_t)185, (uint8_t)69, (uint8_t)87, (uint8_t)73, (uint8_t)31, (uint8_t)115, (uint8_t)202, (uint8_t)63, (uint8_t)82, (uint8_t)237, (uint8_t)164, (uint8_t)236, (uint8_t)102, (uint8_t)9, (uint8_t)215, (uint8_t)29, (uint8_t)95, (uint8_t)154, (uint8_t)54, (uint8_t)205, (uint8_t)23, (uint8_t)34, (uint8_t)51, (uint8_t)52, (uint8_t)108, (uint8_t)141, (uint8_t)39, (uint8_t)127, (uint8_t)206, (uint8_t)153, (uint8_t)18, (uint8_t)60} ;
        uint8_t*  sample = p126_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 70);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p126_baudrate_GET(pack) == (uint32_t)1196457077L);
    assert(p126_flags_GET(pack) == e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE);
    assert(p126_count_GET(pack) == (uint8_t)(uint8_t)30);
    assert(p126_timeout_GET(pack) == (uint16_t)(uint16_t)39841);
};


void c_LoopBackDemoChannel_on_GPS_RTK_127(Bounds_Inside * ph, Pack * pack)
{
    assert(p127_tow_GET(pack) == (uint32_t)2993390319L);
    assert(p127_rtk_rate_GET(pack) == (uint8_t)(uint8_t)45);
    assert(p127_baseline_c_mm_GET(pack) == (int32_t) -2066024894);
    assert(p127_wn_GET(pack) == (uint16_t)(uint16_t)39540);
    assert(p127_baseline_a_mm_GET(pack) == (int32_t)436377954);
    assert(p127_accuracy_GET(pack) == (uint32_t)4005293557L);
    assert(p127_time_last_baseline_ms_GET(pack) == (uint32_t)3405183735L);
    assert(p127_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)108);
    assert(p127_iar_num_hypotheses_GET(pack) == (int32_t)955910123);
    assert(p127_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)68);
    assert(p127_nsats_GET(pack) == (uint8_t)(uint8_t)246);
    assert(p127_baseline_b_mm_GET(pack) == (int32_t)751323678);
    assert(p127_rtk_health_GET(pack) == (uint8_t)(uint8_t)233);
};


void c_LoopBackDemoChannel_on_GPS2_RTK_128(Bounds_Inside * ph, Pack * pack)
{
    assert(p128_baseline_coords_type_GET(pack) == (uint8_t)(uint8_t)111);
    assert(p128_wn_GET(pack) == (uint16_t)(uint16_t)44824);
    assert(p128_rtk_receiver_id_GET(pack) == (uint8_t)(uint8_t)17);
    assert(p128_iar_num_hypotheses_GET(pack) == (int32_t) -1552447843);
    assert(p128_rtk_rate_GET(pack) == (uint8_t)(uint8_t)51);
    assert(p128_nsats_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p128_baseline_b_mm_GET(pack) == (int32_t) -1280685405);
    assert(p128_tow_GET(pack) == (uint32_t)1550556623L);
    assert(p128_baseline_a_mm_GET(pack) == (int32_t) -1782657947);
    assert(p128_accuracy_GET(pack) == (uint32_t)3542689821L);
    assert(p128_time_last_baseline_ms_GET(pack) == (uint32_t)270334891L);
    assert(p128_rtk_health_GET(pack) == (uint8_t)(uint8_t)151);
    assert(p128_baseline_c_mm_GET(pack) == (int32_t)974200798);
};


void c_LoopBackDemoChannel_on_SCALED_IMU3_129(Bounds_Inside * ph, Pack * pack)
{
    assert(p129_ymag_GET(pack) == (int16_t)(int16_t) -26198);
    assert(p129_zgyro_GET(pack) == (int16_t)(int16_t) -4896);
    assert(p129_time_boot_ms_GET(pack) == (uint32_t)2580467608L);
    assert(p129_zmag_GET(pack) == (int16_t)(int16_t)28687);
    assert(p129_ygyro_GET(pack) == (int16_t)(int16_t)25290);
    assert(p129_xmag_GET(pack) == (int16_t)(int16_t)26840);
    assert(p129_yacc_GET(pack) == (int16_t)(int16_t) -9963);
    assert(p129_xacc_GET(pack) == (int16_t)(int16_t)22296);
    assert(p129_xgyro_GET(pack) == (int16_t)(int16_t) -15274);
    assert(p129_zacc_GET(pack) == (int16_t)(int16_t) -12350);
};


void c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(Bounds_Inside * ph, Pack * pack)
{
    assert(p130_height_GET(pack) == (uint16_t)(uint16_t)45444);
    assert(p130_type_GET(pack) == (uint8_t)(uint8_t)54);
    assert(p130_size_GET(pack) == (uint32_t)3428630679L);
    assert(p130_width_GET(pack) == (uint16_t)(uint16_t)17743);
    assert(p130_jpg_quality_GET(pack) == (uint8_t)(uint8_t)96);
    assert(p130_payload_GET(pack) == (uint8_t)(uint8_t)211);
    assert(p130_packets_GET(pack) == (uint16_t)(uint16_t)32924);
};


void c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)244, (uint8_t)28, (uint8_t)240, (uint8_t)21, (uint8_t)221, (uint8_t)106, (uint8_t)121, (uint8_t)130, (uint8_t)55, (uint8_t)101, (uint8_t)44, (uint8_t)201, (uint8_t)159, (uint8_t)119, (uint8_t)20, (uint8_t)217, (uint8_t)95, (uint8_t)192, (uint8_t)186, (uint8_t)130, (uint8_t)120, (uint8_t)149, (uint8_t)251, (uint8_t)249, (uint8_t)88, (uint8_t)252, (uint8_t)121, (uint8_t)249, (uint8_t)140, (uint8_t)154, (uint8_t)189, (uint8_t)70, (uint8_t)37, (uint8_t)61, (uint8_t)138, (uint8_t)136, (uint8_t)206, (uint8_t)199, (uint8_t)87, (uint8_t)246, (uint8_t)244, (uint8_t)188, (uint8_t)185, (uint8_t)209, (uint8_t)218, (uint8_t)67, (uint8_t)123, (uint8_t)13, (uint8_t)157, (uint8_t)14, (uint8_t)84, (uint8_t)180, (uint8_t)155, (uint8_t)38, (uint8_t)11, (uint8_t)236, (uint8_t)245, (uint8_t)187, (uint8_t)64, (uint8_t)133, (uint8_t)254, (uint8_t)156, (uint8_t)45, (uint8_t)166, (uint8_t)73, (uint8_t)80, (uint8_t)172, (uint8_t)225, (uint8_t)80, (uint8_t)93, (uint8_t)83, (uint8_t)213, (uint8_t)12, (uint8_t)163, (uint8_t)210, (uint8_t)102, (uint8_t)82, (uint8_t)14, (uint8_t)239, (uint8_t)147, (uint8_t)144, (uint8_t)134, (uint8_t)0, (uint8_t)177, (uint8_t)158, (uint8_t)218, (uint8_t)14, (uint8_t)133, (uint8_t)254, (uint8_t)245, (uint8_t)135, (uint8_t)33, (uint8_t)236, (uint8_t)113, (uint8_t)238, (uint8_t)165, (uint8_t)148, (uint8_t)209, (uint8_t)142, (uint8_t)57, (uint8_t)172, (uint8_t)246, (uint8_t)140, (uint8_t)12, (uint8_t)148, (uint8_t)183, (uint8_t)166, (uint8_t)13, (uint8_t)170, (uint8_t)153, (uint8_t)130, (uint8_t)141, (uint8_t)215, (uint8_t)68, (uint8_t)87, (uint8_t)126, (uint8_t)18, (uint8_t)250, (uint8_t)168, (uint8_t)210, (uint8_t)179, (uint8_t)81, (uint8_t)41, (uint8_t)94, (uint8_t)209, (uint8_t)132, (uint8_t)180, (uint8_t)128, (uint8_t)27, (uint8_t)199, (uint8_t)179, (uint8_t)153, (uint8_t)52, (uint8_t)228, (uint8_t)239, (uint8_t)219, (uint8_t)112, (uint8_t)143, (uint8_t)145, (uint8_t)86, (uint8_t)210, (uint8_t)174, (uint8_t)69, (uint8_t)158, (uint8_t)223, (uint8_t)207, (uint8_t)60, (uint8_t)255, (uint8_t)198, (uint8_t)150, (uint8_t)167, (uint8_t)135, (uint8_t)72, (uint8_t)135, (uint8_t)230, (uint8_t)173, (uint8_t)169, (uint8_t)79, (uint8_t)196, (uint8_t)0, (uint8_t)210, (uint8_t)46, (uint8_t)47, (uint8_t)68, (uint8_t)63, (uint8_t)175, (uint8_t)131, (uint8_t)232, (uint8_t)26, (uint8_t)150, (uint8_t)206, (uint8_t)146, (uint8_t)12, (uint8_t)163, (uint8_t)178, (uint8_t)76, (uint8_t)244, (uint8_t)163, (uint8_t)51, (uint8_t)36, (uint8_t)195, (uint8_t)106, (uint8_t)77, (uint8_t)159, (uint8_t)107, (uint8_t)38, (uint8_t)25, (uint8_t)118, (uint8_t)174, (uint8_t)161, (uint8_t)150, (uint8_t)91, (uint8_t)80, (uint8_t)25, (uint8_t)154, (uint8_t)118, (uint8_t)245, (uint8_t)18, (uint8_t)210, (uint8_t)246, (uint8_t)56, (uint8_t)77, (uint8_t)210, (uint8_t)87, (uint8_t)135, (uint8_t)30, (uint8_t)229, (uint8_t)33, (uint8_t)96, (uint8_t)18, (uint8_t)125, (uint8_t)225, (uint8_t)16, (uint8_t)138, (uint8_t)152, (uint8_t)187, (uint8_t)113, (uint8_t)22, (uint8_t)146, (uint8_t)187, (uint8_t)186, (uint8_t)39, (uint8_t)90, (uint8_t)29, (uint8_t)188, (uint8_t)56, (uint8_t)22, (uint8_t)16, (uint8_t)86, (uint8_t)230, (uint8_t)43, (uint8_t)121, (uint8_t)246, (uint8_t)110, (uint8_t)223, (uint8_t)204, (uint8_t)114, (uint8_t)24, (uint8_t)22, (uint8_t)152, (uint8_t)203, (uint8_t)218, (uint8_t)150, (uint8_t)65, (uint8_t)71, (uint8_t)50, (uint8_t)77, (uint8_t)120, (uint8_t)82, (uint8_t)153, (uint8_t)173, (uint8_t)240, (uint8_t)242} ;
        uint8_t*  sample = p131_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 253);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p131_seqnr_GET(pack) == (uint16_t)(uint16_t)57183);
};


void c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(Bounds_Inside * ph, Pack * pack)
{
    assert(p132_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND);
    assert(p132_current_distance_GET(pack) == (uint16_t)(uint16_t)17023);
    assert(p132_max_distance_GET(pack) == (uint16_t)(uint16_t)16339);
    assert(p132_id_GET(pack) == (uint8_t)(uint8_t)145);
    assert(p132_orientation_GET(pack) == e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_PITCH_270);
    assert(p132_time_boot_ms_GET(pack) == (uint32_t)3816722028L);
    assert(p132_covariance_GET(pack) == (uint8_t)(uint8_t)224);
    assert(p132_min_distance_GET(pack) == (uint16_t)(uint16_t)38335);
};


void c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(Bounds_Inside * ph, Pack * pack)
{
    assert(p133_grid_spacing_GET(pack) == (uint16_t)(uint16_t)9218);
    assert(p133_lon_GET(pack) == (int32_t)890286506);
    assert(p133_lat_GET(pack) == (int32_t) -1395253698);
    assert(p133_mask_GET(pack) == (uint64_t)8229399769302156884L);
};


void c_LoopBackDemoChannel_on_TERRAIN_DATA_134(Bounds_Inside * ph, Pack * pack)
{
    assert(p134_gridbit_GET(pack) == (uint8_t)(uint8_t)48);
    assert(p134_lat_GET(pack) == (int32_t)1869757602);
    assert(p134_grid_spacing_GET(pack) == (uint16_t)(uint16_t)2841);
    {
        int16_t exemplary[] =  {(int16_t) -22027, (int16_t) -16529, (int16_t) -1672, (int16_t) -23142, (int16_t) -45, (int16_t) -3959, (int16_t) -29836, (int16_t)26291, (int16_t) -9066, (int16_t)23371, (int16_t) -13874, (int16_t)12712, (int16_t) -16784, (int16_t)29458, (int16_t) -16965, (int16_t)27288} ;
        int16_t*  sample = p134_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p134_lon_GET(pack) == (int32_t)813585548);
};


void c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(Bounds_Inside * ph, Pack * pack)
{
    assert(p135_lat_GET(pack) == (int32_t) -99258931);
    assert(p135_lon_GET(pack) == (int32_t) -731970747);
};


void c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(Bounds_Inside * ph, Pack * pack)
{
    assert(p136_current_height_GET(pack) == (float)3.2633519E38F);
    assert(p136_lon_GET(pack) == (int32_t) -2020262525);
    assert(p136_lat_GET(pack) == (int32_t) -441665359);
    assert(p136_loaded_GET(pack) == (uint16_t)(uint16_t)51748);
    assert(p136_terrain_height_GET(pack) == (float) -2.3669371E38F);
    assert(p136_pending_GET(pack) == (uint16_t)(uint16_t)12634);
    assert(p136_spacing_GET(pack) == (uint16_t)(uint16_t)38549);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(Bounds_Inside * ph, Pack * pack)
{
    assert(p137_time_boot_ms_GET(pack) == (uint32_t)978589286L);
    assert(p137_press_abs_GET(pack) == (float) -1.1975603E38F);
    assert(p137_temperature_GET(pack) == (int16_t)(int16_t)7287);
    assert(p137_press_diff_GET(pack) == (float) -1.2228719E38F);
};


void c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {5.595143E37F, 1.749373E38F, -8.2311804E37F, 3.1756173E38F} ;
        float*  sample = p138_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p138_x_GET(pack) == (float)2.2323384E38F);
    assert(p138_y_GET(pack) == (float) -2.3923425E38F);
    assert(p138_z_GET(pack) == (float) -1.3182828E38F);
    assert(p138_time_usec_GET(pack) == (uint64_t)5555185719086581326L);
};


void c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(Bounds_Inside * ph, Pack * pack)
{
    assert(p139_group_mlx_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p139_time_usec_GET(pack) == (uint64_t)7713081588506888055L);
    assert(p139_target_component_GET(pack) == (uint8_t)(uint8_t)173);
    {
        float exemplary[] =  {-2.4637777E38F, 1.5443764E38F, -3.1131562E38F, 1.9869435E38F, -2.2872948E38F, 8.750543E37F, 6.456044E37F, -7.9369916E37F} ;
        float*  sample = p139_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p139_target_system_GET(pack) == (uint8_t)(uint8_t)107);
};


void c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {8.797724E37F, 3.0240334E38F, 1.4697126E38F, 1.877357E38F, 1.0812021E37F, 2.2474605E38F, 2.9421388E38F, -3.7815754E37F} ;
        float*  sample = p140_controls_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p140_group_mlx_GET(pack) == (uint8_t)(uint8_t)202);
    assert(p140_time_usec_GET(pack) == (uint64_t)8433554591385714751L);
};


void c_LoopBackDemoChannel_on_ALTITUDE_141(Bounds_Inside * ph, Pack * pack)
{
    assert(p141_altitude_local_GET(pack) == (float) -1.0303086E38F);
    assert(p141_bottom_clearance_GET(pack) == (float) -2.4929817E37F);
    assert(p141_altitude_relative_GET(pack) == (float)9.389106E37F);
    assert(p141_altitude_monotonic_GET(pack) == (float)8.093497E37F);
    assert(p141_time_usec_GET(pack) == (uint64_t)6217323102206626655L);
    assert(p141_altitude_amsl_GET(pack) == (float)1.9199587E38F);
    assert(p141_altitude_terrain_GET(pack) == (float)2.4599103E38F);
};


void c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(Bounds_Inside * ph, Pack * pack)
{
    assert(p142_transfer_type_GET(pack) == (uint8_t)(uint8_t)130);
    assert(p142_uri_type_GET(pack) == (uint8_t)(uint8_t)253);
    {
        uint8_t exemplary[] =  {(uint8_t)103, (uint8_t)224, (uint8_t)55, (uint8_t)107, (uint8_t)80, (uint8_t)195, (uint8_t)18, (uint8_t)31, (uint8_t)21, (uint8_t)186, (uint8_t)142, (uint8_t)251, (uint8_t)116, (uint8_t)77, (uint8_t)55, (uint8_t)81, (uint8_t)162, (uint8_t)1, (uint8_t)55, (uint8_t)82, (uint8_t)53, (uint8_t)122, (uint8_t)239, (uint8_t)51, (uint8_t)228, (uint8_t)34, (uint8_t)70, (uint8_t)188, (uint8_t)117, (uint8_t)181, (uint8_t)41, (uint8_t)179, (uint8_t)7, (uint8_t)111, (uint8_t)236, (uint8_t)245, (uint8_t)64, (uint8_t)73, (uint8_t)172, (uint8_t)55, (uint8_t)92, (uint8_t)49, (uint8_t)82, (uint8_t)238, (uint8_t)179, (uint8_t)82, (uint8_t)223, (uint8_t)116, (uint8_t)192, (uint8_t)204, (uint8_t)80, (uint8_t)206, (uint8_t)30, (uint8_t)2, (uint8_t)107, (uint8_t)53, (uint8_t)30, (uint8_t)86, (uint8_t)12, (uint8_t)96, (uint8_t)30, (uint8_t)73, (uint8_t)11, (uint8_t)3, (uint8_t)116, (uint8_t)166, (uint8_t)64, (uint8_t)9, (uint8_t)120, (uint8_t)59, (uint8_t)71, (uint8_t)234, (uint8_t)232, (uint8_t)90, (uint8_t)23, (uint8_t)55, (uint8_t)210, (uint8_t)128, (uint8_t)174, (uint8_t)189, (uint8_t)248, (uint8_t)97, (uint8_t)186, (uint8_t)216, (uint8_t)185, (uint8_t)76, (uint8_t)47, (uint8_t)56, (uint8_t)104, (uint8_t)147, (uint8_t)158, (uint8_t)120, (uint8_t)109, (uint8_t)36, (uint8_t)68, (uint8_t)176, (uint8_t)156, (uint8_t)30, (uint8_t)16, (uint8_t)166, (uint8_t)246, (uint8_t)221, (uint8_t)9, (uint8_t)14, (uint8_t)136, (uint8_t)231, (uint8_t)65, (uint8_t)0, (uint8_t)109, (uint8_t)252, (uint8_t)243, (uint8_t)222, (uint8_t)250, (uint8_t)153, (uint8_t)210, (uint8_t)244, (uint8_t)180, (uint8_t)231, (uint8_t)197, (uint8_t)142} ;
        uint8_t*  sample = p142_uri_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)37, (uint8_t)11, (uint8_t)197, (uint8_t)34, (uint8_t)81, (uint8_t)36, (uint8_t)101, (uint8_t)203, (uint8_t)246, (uint8_t)165, (uint8_t)39, (uint8_t)178, (uint8_t)192, (uint8_t)248, (uint8_t)195, (uint8_t)210, (uint8_t)200, (uint8_t)134, (uint8_t)29, (uint8_t)46, (uint8_t)129, (uint8_t)71, (uint8_t)29, (uint8_t)191, (uint8_t)38, (uint8_t)215, (uint8_t)129, (uint8_t)105, (uint8_t)63, (uint8_t)128, (uint8_t)73, (uint8_t)7, (uint8_t)94, (uint8_t)47, (uint8_t)33, (uint8_t)195, (uint8_t)146, (uint8_t)217, (uint8_t)185, (uint8_t)43, (uint8_t)182, (uint8_t)88, (uint8_t)142, (uint8_t)232, (uint8_t)66, (uint8_t)55, (uint8_t)121, (uint8_t)78, (uint8_t)98, (uint8_t)55, (uint8_t)16, (uint8_t)141, (uint8_t)234, (uint8_t)237, (uint8_t)5, (uint8_t)192, (uint8_t)128, (uint8_t)195, (uint8_t)64, (uint8_t)131, (uint8_t)99, (uint8_t)230, (uint8_t)210, (uint8_t)13, (uint8_t)106, (uint8_t)178, (uint8_t)214, (uint8_t)172, (uint8_t)199, (uint8_t)224, (uint8_t)117, (uint8_t)242, (uint8_t)217, (uint8_t)165, (uint8_t)55, (uint8_t)122, (uint8_t)76, (uint8_t)178, (uint8_t)33, (uint8_t)9, (uint8_t)22, (uint8_t)7, (uint8_t)180, (uint8_t)9, (uint8_t)199, (uint8_t)252, (uint8_t)77, (uint8_t)205, (uint8_t)196, (uint8_t)150, (uint8_t)126, (uint8_t)43, (uint8_t)157, (uint8_t)122, (uint8_t)245, (uint8_t)211, (uint8_t)151, (uint8_t)183, (uint8_t)25, (uint8_t)126, (uint8_t)137, (uint8_t)238, (uint8_t)230, (uint8_t)2, (uint8_t)32, (uint8_t)179, (uint8_t)118, (uint8_t)2, (uint8_t)189, (uint8_t)226, (uint8_t)254, (uint8_t)126, (uint8_t)159, (uint8_t)210, (uint8_t)151, (uint8_t)249, (uint8_t)151, (uint8_t)193, (uint8_t)207, (uint8_t)86} ;
        uint8_t*  sample = p142_storage_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 120);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p142_request_id_GET(pack) == (uint8_t)(uint8_t)206);
};


void c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(Bounds_Inside * ph, Pack * pack)
{
    assert(p143_press_abs_GET(pack) == (float)3.3429764E38F);
    assert(p143_time_boot_ms_GET(pack) == (uint32_t)2112206153L);
    assert(p143_press_diff_GET(pack) == (float) -1.4023477E38F);
    assert(p143_temperature_GET(pack) == (int16_t)(int16_t) -9011);
};


void c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(Bounds_Inside * ph, Pack * pack)
{
    {
        float exemplary[] =  {-9.645967E37F, -3.117291E37F, 7.401917E37F} ;
        float*  sample = p144_vel_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lon_GET(pack) == (int32_t)1893696585);
    assert(p144_timestamp_GET(pack) == (uint64_t)1793977892703052353L);
    assert(p144_est_capabilities_GET(pack) == (uint8_t)(uint8_t)52);
    {
        float exemplary[] =  {-1.5079976E38F, -3.3120936E38F, -3.3805157E38F, 7.568284E37F} ;
        float*  sample = p144_attitude_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_lat_GET(pack) == (int32_t)107872323);
    {
        float exemplary[] =  {1.298003E38F, -3.2687813E38F, 1.3835711E38F} ;
        float*  sample = p144_acc_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_alt_GET(pack) == (float) -2.4912001E38F);
    {
        float exemplary[] =  {-2.9402243E38F, 3.1200412E38F, -3.0300413E38F} ;
        float*  sample = p144_rates_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {1.7667397E38F, 3.549423E36F, 7.1794335E37F} ;
        float*  sample = p144_position_cov_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p144_custom_state_GET(pack) == (uint64_t)4090244485321822172L);
};


void c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(Bounds_Inside * ph, Pack * pack)
{
    assert(p146_roll_rate_GET(pack) == (float) -1.6407169E38F);
    assert(p146_y_vel_GET(pack) == (float) -3.0856017E38F);
    assert(p146_airspeed_GET(pack) == (float) -1.383807E38F);
    assert(p146_z_acc_GET(pack) == (float)3.1760688E38F);
    assert(p146_z_vel_GET(pack) == (float) -1.2965553E38F);
    assert(p146_x_vel_GET(pack) == (float)7.404201E37F);
    assert(p146_pitch_rate_GET(pack) == (float)1.0414815E38F);
    assert(p146_z_pos_GET(pack) == (float)2.400677E38F);
    {
        float exemplary[] =  {6.521833E37F, -2.2146905E38F, 1.4690012E38F, 3.7554185E37F} ;
        float*  sample = p146_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        float exemplary[] =  {-1.5687349E38F, -2.2412927E38F, 2.221332E38F} ;
        float*  sample = p146_pos_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p146_yaw_rate_GET(pack) == (float) -1.9951496E38F);
    assert(p146_time_usec_GET(pack) == (uint64_t)2322542993496965825L);
    assert(p146_y_acc_GET(pack) == (float) -3.1861842E38F);
    assert(p146_y_pos_GET(pack) == (float) -2.7450404E38F);
    assert(p146_x_acc_GET(pack) == (float) -8.79271E37F);
    assert(p146_x_pos_GET(pack) == (float) -1.7321604E38F);
    {
        float exemplary[] =  {6.1187497E37F, -2.3755223E38F, 1.1106202E38F} ;
        float*  sample = p146_vel_variance_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 12);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_BATTERY_STATUS_147(Bounds_Inside * ph, Pack * pack)
{
    assert(p147_current_consumed_GET(pack) == (int32_t)1021715915);
    {
        uint16_t exemplary[] =  {(uint16_t)41251, (uint16_t)35445, (uint16_t)19370, (uint16_t)31949, (uint16_t)61573, (uint16_t)36238, (uint16_t)51019, (uint16_t)36270, (uint16_t)63186, (uint16_t)22193} ;
        uint16_t*  sample = p147_voltages_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p147_type_GET(pack) == e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIPO);
    assert(p147_energy_consumed_GET(pack) == (int32_t) -1666573051);
    assert(p147_temperature_GET(pack) == (int16_t)(int16_t)27934);
    assert(p147_battery_function_GET(pack) == e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION);
    assert(p147_id_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p147_current_battery_GET(pack) == (int16_t)(int16_t) -23971);
    assert(p147_battery_remaining_GET(pack) == (int8_t)(int8_t)49);
};


void c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)225, (uint8_t)168, (uint8_t)212, (uint8_t)56, (uint8_t)56, (uint8_t)105, (uint8_t)169, (uint8_t)231, (uint8_t)42, (uint8_t)212, (uint8_t)192, (uint8_t)33, (uint8_t)112, (uint8_t)254, (uint8_t)224, (uint8_t)228, (uint8_t)157, (uint8_t)215} ;
        uint8_t*  sample = p148_uid2_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 18);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)99, (uint8_t)132, (uint8_t)93, (uint8_t)144, (uint8_t)52, (uint8_t)34, (uint8_t)26, (uint8_t)7} ;
        uint8_t*  sample = p148_os_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_product_id_GET(pack) == (uint16_t)(uint16_t)59228);
    {
        uint8_t exemplary[] =  {(uint8_t)254, (uint8_t)94, (uint8_t)234, (uint8_t)234, (uint8_t)29, (uint8_t)120, (uint8_t)240, (uint8_t)97} ;
        uint8_t*  sample = p148_flight_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)226, (uint8_t)150, (uint8_t)175, (uint8_t)130, (uint8_t)253, (uint8_t)151, (uint8_t)62, (uint8_t)44} ;
        uint8_t*  sample = p148_middleware_custom_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p148_board_version_GET(pack) == (uint32_t)1453073520L);
    assert(p148_uid_GET(pack) == (uint64_t)7702990746507491038L);
    assert(p148_flight_sw_version_GET(pack) == (uint32_t)770410931L);
    assert(p148_vendor_id_GET(pack) == (uint16_t)(uint16_t)62951);
    assert(p148_capabilities_GET(pack) == e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT);
    assert(p148_middleware_sw_version_GET(pack) == (uint32_t)2251704304L);
    assert(p148_os_sw_version_GET(pack) == (uint32_t)1626150905L);
};


void c_LoopBackDemoChannel_on_LANDING_TARGET_149(Bounds_Inside * ph, Pack * pack)
{
    assert(p149_type_GET(pack) == e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON);
    assert(p149_angle_x_GET(pack) == (float) -1.3169938E38F);
    assert(p149_size_y_GET(pack) == (float)7.9611703E37F);
    assert(p149_target_num_GET(pack) == (uint8_t)(uint8_t)57);
    assert(p149_angle_y_GET(pack) == (float) -1.750509E38F);
    assert(p149_distance_GET(pack) == (float)1.9786227E38F);
    assert(p149_position_valid_TRY(ph) == (uint8_t)(uint8_t)209);
    assert(p149_size_x_GET(pack) == (float) -3.226967E38F);
    assert(p149_frame_GET(pack) == e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED);
    assert(p149_z_TRY(ph) == (float) -9.764795E37F);
    assert(p149_time_usec_GET(pack) == (uint64_t)8454174349278249267L);
    assert(p149_y_TRY(ph) == (float) -3.365397E38F);
    {
        float exemplary[] =  {7.082778E37F, 1.5458E38F, 1.5896906E38F, 4.1562235E37F} ;
        float*  sample = p149_q_TRY(ph);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p149_x_TRY(ph) == (float) -3.1930997E38F);
};


void c_LoopBackDemoChannel_on_AQ_TELEMETRY_F_150(Bounds_Inside * ph, Pack * pack)
{
    assert(p150_value9_GET(pack) == (float) -2.1015673E38F);
    assert(p150_value13_GET(pack) == (float)2.8382575E38F);
    assert(p150_value11_GET(pack) == (float)1.7632632E38F);
    assert(p150_value4_GET(pack) == (float) -1.5014304E38F);
    assert(p150_value18_GET(pack) == (float)1.1879066E38F);
    assert(p150_value2_GET(pack) == (float) -3.0606353E38F);
    assert(p150_value6_GET(pack) == (float) -3.492019E37F);
    assert(p150_value5_GET(pack) == (float) -1.5081113E38F);
    assert(p150_value17_GET(pack) == (float) -1.493297E38F);
    assert(p150_value16_GET(pack) == (float)2.6582501E38F);
    assert(p150_value12_GET(pack) == (float)2.7727482E38F);
    assert(p150_value1_GET(pack) == (float)2.1998571E38F);
    assert(p150_value19_GET(pack) == (float)2.3695941E38F);
    assert(p150_Index_GET(pack) == (uint16_t)(uint16_t)5194);
    assert(p150_value7_GET(pack) == (float)1.073884E38F);
    assert(p150_value8_GET(pack) == (float) -2.6958933E38F);
    assert(p150_value20_GET(pack) == (float)1.3320754E38F);
    assert(p150_value14_GET(pack) == (float) -3.1282923E38F);
    assert(p150_value15_GET(pack) == (float) -2.430555E38F);
    assert(p150_value3_GET(pack) == (float)5.605671E37F);
    assert(p150_value10_GET(pack) == (float) -4.994688E36F);
};


void c_LoopBackDemoChannel_on_AQ_ESC_TELEMETRY_152(Bounds_Inside * ph, Pack * pack)
{
    assert(p152_num_in_seq_GET(pack) == (uint8_t)(uint8_t)100);
    assert(p152_seq_GET(pack) == (uint8_t)(uint8_t)82);
    {
        uint32_t exemplary[] =  {106979466L, 3781519381L, 1082728225L, 3449363819L} ;
        uint32_t*  sample = p152_data0_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint32_t exemplary[] =  {2385176779L, 2306989917L, 823114701L, 2103296916L} ;
        uint32_t*  sample = p152_data1_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p152_time_boot_ms_GET(pack) == (uint32_t)108898985L);
    {
        uint16_t exemplary[] =  {(uint16_t)20018, (uint16_t)60974, (uint16_t)22882, (uint16_t)44388} ;
        uint16_t*  sample = p152_status_age_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)99, (uint8_t)34, (uint8_t)45, (uint8_t)148} ;
        uint8_t*  sample = p152_escid_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p152_num_motors_GET(pack) == (uint8_t)(uint8_t)232);
    {
        uint8_t exemplary[] =  {(uint8_t)103, (uint8_t)41, (uint8_t)228, (uint8_t)133} ;
        uint8_t*  sample = p152_data_version_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(Bounds_Inside * ph, Pack * pack)
{
    assert(p230_hagl_ratio_GET(pack) == (float)2.5537014E38F);
    assert(p230_time_usec_GET(pack) == (uint64_t)3550154252437264000L);
    assert(p230_pos_horiz_accuracy_GET(pack) == (float) -1.672426E37F);
    assert(p230_pos_vert_ratio_GET(pack) == (float) -1.5454398E38F);
    assert(p230_flags_GET(pack) == e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_HORIZ);
    assert(p230_pos_horiz_ratio_GET(pack) == (float)2.0984948E38F);
    assert(p230_pos_vert_accuracy_GET(pack) == (float) -2.8400578E38F);
    assert(p230_vel_ratio_GET(pack) == (float)1.948079E38F);
    assert(p230_tas_ratio_GET(pack) == (float) -2.3369297E38F);
    assert(p230_mag_ratio_GET(pack) == (float) -1.8288476E38F);
};


void c_LoopBackDemoChannel_on_WIND_COV_231(Bounds_Inside * ph, Pack * pack)
{
    assert(p231_var_vert_GET(pack) == (float)2.4534807E38F);
    assert(p231_wind_y_GET(pack) == (float)8.182787E37F);
    assert(p231_horiz_accuracy_GET(pack) == (float)9.056634E37F);
    assert(p231_var_horiz_GET(pack) == (float) -3.4415155E37F);
    assert(p231_wind_alt_GET(pack) == (float) -3.3286018E38F);
    assert(p231_wind_z_GET(pack) == (float) -3.3186815E38F);
    assert(p231_vert_accuracy_GET(pack) == (float)2.8652072E38F);
    assert(p231_wind_x_GET(pack) == (float) -5.0219715E36F);
    assert(p231_time_usec_GET(pack) == (uint64_t)814389179059969894L);
};


void c_LoopBackDemoChannel_on_GPS_INPUT_232(Bounds_Inside * ph, Pack * pack)
{
    assert(p232_hdop_GET(pack) == (float) -1.4369478E38F);
    assert(p232_time_usec_GET(pack) == (uint64_t)3964015191382715958L);
    assert(p232_vdop_GET(pack) == (float)2.851713E38F);
    assert(p232_vert_accuracy_GET(pack) == (float) -3.3278374E38F);
    assert(p232_fix_type_GET(pack) == (uint8_t)(uint8_t)13);
    assert(p232_lon_GET(pack) == (int32_t) -1998096099);
    assert(p232_ve_GET(pack) == (float) -1.4404676E38F);
    assert(p232_horiz_accuracy_GET(pack) == (float) -7.799927E37F);
    assert(p232_speed_accuracy_GET(pack) == (float)5.9887906E37F);
    assert(p232_time_week_GET(pack) == (uint16_t)(uint16_t)24101);
    assert(p232_alt_GET(pack) == (float)1.6053685E38F);
    assert(p232_vn_GET(pack) == (float)2.231187E38F);
    assert(p232_vd_GET(pack) == (float) -4.741962E37F);
    assert(p232_gps_id_GET(pack) == (uint8_t)(uint8_t)233);
    assert(p232_time_week_ms_GET(pack) == (uint32_t)438328789L);
    assert(p232_lat_GET(pack) == (int32_t) -844776903);
    assert(p232_ignore_flags_GET(pack) == e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY);
    assert(p232_satellites_visible_GET(pack) == (uint8_t)(uint8_t)119);
};


void c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(Bounds_Inside * ph, Pack * pack)
{
    assert(p233_len_GET(pack) == (uint8_t)(uint8_t)216);
    {
        uint8_t exemplary[] =  {(uint8_t)222, (uint8_t)244, (uint8_t)147, (uint8_t)159, (uint8_t)161, (uint8_t)100, (uint8_t)53, (uint8_t)167, (uint8_t)220, (uint8_t)113, (uint8_t)152, (uint8_t)200, (uint8_t)191, (uint8_t)57, (uint8_t)123, (uint8_t)60, (uint8_t)32, (uint8_t)127, (uint8_t)203, (uint8_t)187, (uint8_t)150, (uint8_t)221, (uint8_t)248, (uint8_t)91, (uint8_t)216, (uint8_t)5, (uint8_t)123, (uint8_t)35, (uint8_t)184, (uint8_t)45, (uint8_t)216, (uint8_t)107, (uint8_t)252, (uint8_t)51, (uint8_t)133, (uint8_t)115, (uint8_t)193, (uint8_t)55, (uint8_t)6, (uint8_t)148, (uint8_t)53, (uint8_t)224, (uint8_t)234, (uint8_t)163, (uint8_t)11, (uint8_t)49, (uint8_t)215, (uint8_t)31, (uint8_t)214, (uint8_t)239, (uint8_t)150, (uint8_t)173, (uint8_t)62, (uint8_t)197, (uint8_t)207, (uint8_t)114, (uint8_t)160, (uint8_t)176, (uint8_t)138, (uint8_t)10, (uint8_t)4, (uint8_t)241, (uint8_t)232, (uint8_t)203, (uint8_t)160, (uint8_t)231, (uint8_t)173, (uint8_t)206, (uint8_t)219, (uint8_t)14, (uint8_t)171, (uint8_t)10, (uint8_t)51, (uint8_t)223, (uint8_t)220, (uint8_t)79, (uint8_t)7, (uint8_t)137, (uint8_t)72, (uint8_t)189, (uint8_t)181, (uint8_t)66, (uint8_t)86, (uint8_t)154, (uint8_t)150, (uint8_t)139, (uint8_t)95, (uint8_t)76, (uint8_t)146, (uint8_t)146, (uint8_t)32, (uint8_t)23, (uint8_t)78, (uint8_t)30, (uint8_t)140, (uint8_t)204, (uint8_t)72, (uint8_t)229, (uint8_t)78, (uint8_t)147, (uint8_t)34, (uint8_t)160, (uint8_t)206, (uint8_t)160, (uint8_t)195, (uint8_t)190, (uint8_t)60, (uint8_t)99, (uint8_t)35, (uint8_t)236, (uint8_t)79, (uint8_t)52, (uint8_t)191, (uint8_t)191, (uint8_t)141, (uint8_t)10, (uint8_t)58, (uint8_t)239, (uint8_t)83, (uint8_t)156, (uint8_t)19, (uint8_t)187, (uint8_t)119, (uint8_t)69, (uint8_t)232, (uint8_t)61, (uint8_t)104, (uint8_t)76, (uint8_t)33, (uint8_t)243, (uint8_t)86, (uint8_t)112, (uint8_t)120, (uint8_t)239, (uint8_t)126, (uint8_t)210, (uint8_t)72, (uint8_t)232, (uint8_t)212, (uint8_t)47, (uint8_t)179, (uint8_t)165, (uint8_t)72, (uint8_t)126, (uint8_t)182, (uint8_t)11, (uint8_t)197, (uint8_t)11, (uint8_t)251, (uint8_t)248, (uint8_t)76, (uint8_t)5, (uint8_t)82, (uint8_t)15, (uint8_t)61, (uint8_t)37, (uint8_t)204, (uint8_t)74, (uint8_t)24, (uint8_t)117, (uint8_t)55, (uint8_t)123, (uint8_t)76, (uint8_t)21, (uint8_t)70, (uint8_t)112, (uint8_t)176, (uint8_t)179, (uint8_t)159, (uint8_t)7, (uint8_t)167, (uint8_t)156, (uint8_t)183, (uint8_t)5, (uint8_t)199, (uint8_t)36, (uint8_t)104, (uint8_t)35, (uint8_t)165, (uint8_t)50} ;
        uint8_t*  sample = p233_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 180);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p233_flags_GET(pack) == (uint8_t)(uint8_t)227);
};


void c_LoopBackDemoChannel_on_HIGH_LATENCY_234(Bounds_Inside * ph, Pack * pack)
{
    assert(p234_gps_fix_type_GET(pack) == e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC);
    assert(p234_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF);
    assert(p234_temperature_air_GET(pack) == (int8_t)(int8_t)27);
    assert(p234_custom_mode_GET(pack) == (uint32_t)2513242689L);
    assert(p234_failsafe_GET(pack) == (uint8_t)(uint8_t)231);
    assert(p234_longitude_GET(pack) == (int32_t)57510037);
    assert(p234_airspeed_sp_GET(pack) == (uint8_t)(uint8_t)120);
    assert(p234_gps_nsat_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p234_base_mode_GET(pack) == e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED);
    assert(p234_roll_GET(pack) == (int16_t)(int16_t)25891);
    assert(p234_pitch_GET(pack) == (int16_t)(int16_t)25906);
    assert(p234_airspeed_GET(pack) == (uint8_t)(uint8_t)201);
    assert(p234_heading_sp_GET(pack) == (int16_t)(int16_t) -28916);
    assert(p234_climb_rate_GET(pack) == (int8_t)(int8_t) -38);
    assert(p234_altitude_sp_GET(pack) == (int16_t)(int16_t) -19292);
    assert(p234_groundspeed_GET(pack) == (uint8_t)(uint8_t)169);
    assert(p234_altitude_amsl_GET(pack) == (int16_t)(int16_t)16427);
    assert(p234_wp_distance_GET(pack) == (uint16_t)(uint16_t)1674);
    assert(p234_wp_num_GET(pack) == (uint8_t)(uint8_t)203);
    assert(p234_heading_GET(pack) == (uint16_t)(uint16_t)36015);
    assert(p234_temperature_GET(pack) == (int8_t)(int8_t) -66);
    assert(p234_battery_remaining_GET(pack) == (uint8_t)(uint8_t)11);
    assert(p234_latitude_GET(pack) == (int32_t)1876731121);
    assert(p234_throttle_GET(pack) == (int8_t)(int8_t) -97);
};


void c_LoopBackDemoChannel_on_VIBRATION_241(Bounds_Inside * ph, Pack * pack)
{
    assert(p241_clipping_1_GET(pack) == (uint32_t)1204613084L);
    assert(p241_clipping_0_GET(pack) == (uint32_t)60729767L);
    assert(p241_clipping_2_GET(pack) == (uint32_t)443599353L);
    assert(p241_time_usec_GET(pack) == (uint64_t)1123013032334391021L);
    assert(p241_vibration_x_GET(pack) == (float)1.1212366E38F);
    assert(p241_vibration_y_GET(pack) == (float) -3.0223715E38F);
    assert(p241_vibration_z_GET(pack) == (float)1.4975082E38F);
};


void c_LoopBackDemoChannel_on_HOME_POSITION_242(Bounds_Inside * ph, Pack * pack)
{
    assert(p242_x_GET(pack) == (float)8.027735E36F);
    assert(p242_approach_x_GET(pack) == (float) -6.859255E37F);
    assert(p242_z_GET(pack) == (float) -1.9507076E38F);
    assert(p242_latitude_GET(pack) == (int32_t) -38364954);
    {
        float exemplary[] =  {2.993938E38F, -1.4180797E38F, 2.4258835E38F, -4.3149483E37F} ;
        float*  sample = p242_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p242_approach_z_GET(pack) == (float) -1.4027694E38F);
    assert(p242_time_usec_TRY(ph) == (uint64_t)5908498008748899177L);
    assert(p242_longitude_GET(pack) == (int32_t) -1650708290);
    assert(p242_y_GET(pack) == (float) -2.2185123E37F);
    assert(p242_approach_y_GET(pack) == (float)2.4004932E38F);
    assert(p242_altitude_GET(pack) == (int32_t)1252423298);
};


void c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(Bounds_Inside * ph, Pack * pack)
{
    assert(p243_target_system_GET(pack) == (uint8_t)(uint8_t)155);
    assert(p243_z_GET(pack) == (float) -3.669853E37F);
    assert(p243_y_GET(pack) == (float)1.88521E38F);
    assert(p243_latitude_GET(pack) == (int32_t)1519090844);
    assert(p243_time_usec_TRY(ph) == (uint64_t)6232826439787518417L);
    assert(p243_longitude_GET(pack) == (int32_t) -1351104907);
    {
        float exemplary[] =  {-2.1983384E38F, 7.1264477E37F, -1.1253487E37F, 1.7473251E38F} ;
        float*  sample = p243_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p243_approach_y_GET(pack) == (float) -3.1245468E38F);
    assert(p243_x_GET(pack) == (float)3.1022246E38F);
    assert(p243_approach_x_GET(pack) == (float) -2.8481854E38F);
    assert(p243_altitude_GET(pack) == (int32_t)638102777);
    assert(p243_approach_z_GET(pack) == (float)6.833373E37F);
};


void c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(Bounds_Inside * ph, Pack * pack)
{
    assert(p244_interval_us_GET(pack) == (int32_t) -1551090296);
    assert(p244_message_id_GET(pack) == (uint16_t)(uint16_t)18632);
};


void c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(Bounds_Inside * ph, Pack * pack)
{
    assert(p245_landed_state_GET(pack) == e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED);
    assert(p245_vtol_state_GET(pack) == e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_MC);
};


void c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(Bounds_Inside * ph, Pack * pack)
{
    assert(p246_lon_GET(pack) == (int32_t)337696588);
    assert(p246_lat_GET(pack) == (int32_t) -50572995);
    assert(p246_tslc_GET(pack) == (uint8_t)(uint8_t)29);
    assert(p246_ICAO_address_GET(pack) == (uint32_t)2374959065L);
    assert(p246_emitter_type_GET(pack) == e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UAV);
    assert(p246_altitude_type_GET(pack) == e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC);
    assert(p246_ver_velocity_GET(pack) == (int16_t)(int16_t)17245);
    assert(p246_callsign_LEN(ph) == 8);
    {
        char16_t * exemplary = u"hwnmCbws";
        char16_t * sample = p246_callsign_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p246_hor_velocity_GET(pack) == (uint16_t)(uint16_t)15217);
    assert(p246_altitude_GET(pack) == (int32_t)1828949709);
    assert(p246_squawk_GET(pack) == (uint16_t)(uint16_t)6991);
    assert(p246_flags_GET(pack) == e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY);
    assert(p246_heading_GET(pack) == (uint16_t)(uint16_t)7915);
};


void c_LoopBackDemoChannel_on_COLLISION_247(Bounds_Inside * ph, Pack * pack)
{
    assert(p247_id_GET(pack) == (uint32_t)3313692311L);
    assert(p247_horizontal_minimum_delta_GET(pack) == (float)2.2315283E38F);
    assert(p247_altitude_minimum_delta_GET(pack) == (float)1.2133445E38F);
    assert(p247_threat_level_GET(pack) == e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH);
    assert(p247_time_to_minimum_delta_GET(pack) == (float) -7.2788416E37F);
    assert(p247_src__GET(pack) == e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT);
    assert(p247_action_GET(pack) == e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_PERPENDICULAR);
};


void c_LoopBackDemoChannel_on_V2_EXTENSION_248(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)20, (uint8_t)86, (uint8_t)99, (uint8_t)167, (uint8_t)160, (uint8_t)226, (uint8_t)133, (uint8_t)217, (uint8_t)96, (uint8_t)221, (uint8_t)195, (uint8_t)227, (uint8_t)4, (uint8_t)167, (uint8_t)217, (uint8_t)118, (uint8_t)23, (uint8_t)192, (uint8_t)36, (uint8_t)18, (uint8_t)176, (uint8_t)224, (uint8_t)120, (uint8_t)109, (uint8_t)59, (uint8_t)105, (uint8_t)101, (uint8_t)21, (uint8_t)176, (uint8_t)40, (uint8_t)158, (uint8_t)11, (uint8_t)132, (uint8_t)164, (uint8_t)91, (uint8_t)134, (uint8_t)48, (uint8_t)216, (uint8_t)122, (uint8_t)64, (uint8_t)182, (uint8_t)22, (uint8_t)140, (uint8_t)106, (uint8_t)60, (uint8_t)1, (uint8_t)59, (uint8_t)3, (uint8_t)4, (uint8_t)48, (uint8_t)179, (uint8_t)149, (uint8_t)92, (uint8_t)80, (uint8_t)140, (uint8_t)215, (uint8_t)162, (uint8_t)188, (uint8_t)70, (uint8_t)3, (uint8_t)234, (uint8_t)177, (uint8_t)169, (uint8_t)55, (uint8_t)69, (uint8_t)37, (uint8_t)166, (uint8_t)208, (uint8_t)178, (uint8_t)166, (uint8_t)246, (uint8_t)165, (uint8_t)99, (uint8_t)146, (uint8_t)103, (uint8_t)141, (uint8_t)241, (uint8_t)60, (uint8_t)194, (uint8_t)129, (uint8_t)135, (uint8_t)43, (uint8_t)205, (uint8_t)158, (uint8_t)235, (uint8_t)209, (uint8_t)15, (uint8_t)119, (uint8_t)15, (uint8_t)99, (uint8_t)113, (uint8_t)180, (uint8_t)155, (uint8_t)70, (uint8_t)159, (uint8_t)182, (uint8_t)240, (uint8_t)181, (uint8_t)143, (uint8_t)246, (uint8_t)65, (uint8_t)94, (uint8_t)206, (uint8_t)50, (uint8_t)48, (uint8_t)213, (uint8_t)204, (uint8_t)83, (uint8_t)126, (uint8_t)136, (uint8_t)214, (uint8_t)58, (uint8_t)134, (uint8_t)146, (uint8_t)16, (uint8_t)82, (uint8_t)191, (uint8_t)5, (uint8_t)235, (uint8_t)166, (uint8_t)141, (uint8_t)53, (uint8_t)28, (uint8_t)43, (uint8_t)162, (uint8_t)8, (uint8_t)21, (uint8_t)122, (uint8_t)87, (uint8_t)233, (uint8_t)21, (uint8_t)140, (uint8_t)176, (uint8_t)28, (uint8_t)179, (uint8_t)150, (uint8_t)104, (uint8_t)60, (uint8_t)26, (uint8_t)73, (uint8_t)31, (uint8_t)209, (uint8_t)169, (uint8_t)100, (uint8_t)211, (uint8_t)72, (uint8_t)30, (uint8_t)26, (uint8_t)152, (uint8_t)0, (uint8_t)161, (uint8_t)244, (uint8_t)157, (uint8_t)163, (uint8_t)120, (uint8_t)57, (uint8_t)138, (uint8_t)252, (uint8_t)120, (uint8_t)253, (uint8_t)196, (uint8_t)75, (uint8_t)182, (uint8_t)173, (uint8_t)40, (uint8_t)34, (uint8_t)10, (uint8_t)153, (uint8_t)221, (uint8_t)51, (uint8_t)89, (uint8_t)30, (uint8_t)127, (uint8_t)236, (uint8_t)12, (uint8_t)209, (uint8_t)42, (uint8_t)252, (uint8_t)170, (uint8_t)150, (uint8_t)116, (uint8_t)14, (uint8_t)233, (uint8_t)45, (uint8_t)165, (uint8_t)253, (uint8_t)38, (uint8_t)172, (uint8_t)151, (uint8_t)82, (uint8_t)61, (uint8_t)59, (uint8_t)8, (uint8_t)38, (uint8_t)79, (uint8_t)192, (uint8_t)204, (uint8_t)159, (uint8_t)110, (uint8_t)83, (uint8_t)136, (uint8_t)153, (uint8_t)111, (uint8_t)242, (uint8_t)169, (uint8_t)162, (uint8_t)167, (uint8_t)119, (uint8_t)109, (uint8_t)178, (uint8_t)139, (uint8_t)255, (uint8_t)196, (uint8_t)159, (uint8_t)171, (uint8_t)36, (uint8_t)128, (uint8_t)99, (uint8_t)48, (uint8_t)69, (uint8_t)185, (uint8_t)57, (uint8_t)110, (uint8_t)125, (uint8_t)135, (uint8_t)155, (uint8_t)127, (uint8_t)239, (uint8_t)178, (uint8_t)19, (uint8_t)226, (uint8_t)215, (uint8_t)35, (uint8_t)86, (uint8_t)126, (uint8_t)105, (uint8_t)190, (uint8_t)235, (uint8_t)149, (uint8_t)172, (uint8_t)29, (uint8_t)166, (uint8_t)62, (uint8_t)36, (uint8_t)83, (uint8_t)105, (uint8_t)185, (uint8_t)63, (uint8_t)136} ;
        uint8_t*  sample = p248_payload_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p248_target_system_GET(pack) == (uint8_t)(uint8_t)110);
    assert(p248_message_type_GET(pack) == (uint16_t)(uint16_t)31460);
    assert(p248_target_network_GET(pack) == (uint8_t)(uint8_t)67);
    assert(p248_target_component_GET(pack) == (uint8_t)(uint8_t)251);
};


void c_LoopBackDemoChannel_on_MEMORY_VECT_249(Bounds_Inside * ph, Pack * pack)
{
    assert(p249_type_GET(pack) == (uint8_t)(uint8_t)223);
    assert(p249_address_GET(pack) == (uint16_t)(uint16_t)42256);
    {
        int8_t exemplary[] =  {(int8_t)42, (int8_t)64, (int8_t)47, (int8_t) -24, (int8_t)9, (int8_t)37, (int8_t)127, (int8_t) -54, (int8_t) -102, (int8_t)44, (int8_t) -74, (int8_t)34, (int8_t)20, (int8_t)3, (int8_t)46, (int8_t)8, (int8_t) -52, (int8_t)43, (int8_t)4, (int8_t)125, (int8_t)96, (int8_t)120, (int8_t) -125, (int8_t) -22, (int8_t)3, (int8_t)3, (int8_t) -118, (int8_t)101, (int8_t)89, (int8_t)105, (int8_t) -91, (int8_t) -36} ;
        int8_t*  sample = p249_value_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p249_ver_GET(pack) == (uint8_t)(uint8_t)28);
};


void c_LoopBackDemoChannel_on_DEBUG_VECT_250(Bounds_Inside * ph, Pack * pack)
{
    assert(p250_y_GET(pack) == (float) -2.8220634E38F);
    assert(p250_time_usec_GET(pack) == (uint64_t)5970517450518642378L);
    assert(p250_x_GET(pack) == (float)1.3761943E37F);
    assert(p250_name_LEN(ph) == 3);
    {
        char16_t * exemplary = u"ATt";
        char16_t * sample = p250_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 6);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p250_z_GET(pack) == (float) -3.2086602E38F);
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(Bounds_Inside * ph, Pack * pack)
{
    assert(p251_name_LEN(ph) == 10);
    {
        char16_t * exemplary = u"fapmwrddiP";
        char16_t * sample = p251_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 20);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p251_time_boot_ms_GET(pack) == (uint32_t)1365170760L);
    assert(p251_value_GET(pack) == (float)3.3286988E38F);
};


void c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(Bounds_Inside * ph, Pack * pack)
{
    assert(p252_time_boot_ms_GET(pack) == (uint32_t)673680035L);
    assert(p252_value_GET(pack) == (int32_t) -2129142694);
    assert(p252_name_LEN(ph) == 2);
    {
        char16_t * exemplary = u"Wd";
        char16_t * sample = p252_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 4);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_STATUSTEXT_253(Bounds_Inside * ph, Pack * pack)
{
    assert(p253_text_LEN(ph) == 44);
    {
        char16_t * exemplary = u"eVjWvigTsenncknkxortlvxjlkvipzAuvhbqMuuxmxfj";
        char16_t * sample = p253_text_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 88);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p253_severity_GET(pack) == e_MAV_SEVERITY_MAV_SEVERITY_NOTICE);
};


void c_LoopBackDemoChannel_on_DEBUG_254(Bounds_Inside * ph, Pack * pack)
{
    assert(p254_value_GET(pack) == (float) -2.0567544E38F);
    assert(p254_ind_GET(pack) == (uint8_t)(uint8_t)137);
    assert(p254_time_boot_ms_GET(pack) == (uint32_t)3582531714L);
};


void c_LoopBackDemoChannel_on_SETUP_SIGNING_256(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)102, (uint8_t)236, (uint8_t)41, (uint8_t)128, (uint8_t)0, (uint8_t)149, (uint8_t)202, (uint8_t)182, (uint8_t)45, (uint8_t)22, (uint8_t)18, (uint8_t)10, (uint8_t)71, (uint8_t)118, (uint8_t)220, (uint8_t)169, (uint8_t)24, (uint8_t)104, (uint8_t)65, (uint8_t)185, (uint8_t)255, (uint8_t)126, (uint8_t)57, (uint8_t)202, (uint8_t)132, (uint8_t)62, (uint8_t)74, (uint8_t)186, (uint8_t)178, (uint8_t)14, (uint8_t)105, (uint8_t)187} ;
        uint8_t*  sample = p256_secret_key_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p256_target_component_GET(pack) == (uint8_t)(uint8_t)105);
    assert(p256_target_system_GET(pack) == (uint8_t)(uint8_t)103);
    assert(p256_initial_timestamp_GET(pack) == (uint64_t)3481797220018744841L);
};


void c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(Bounds_Inside * ph, Pack * pack)
{
    assert(p257_state_GET(pack) == (uint8_t)(uint8_t)128);
    assert(p257_last_change_ms_GET(pack) == (uint32_t)1435655979L);
    assert(p257_time_boot_ms_GET(pack) == (uint32_t)1016514537L);
};


void c_LoopBackDemoChannel_on_PLAY_TUNE_258(Bounds_Inside * ph, Pack * pack)
{
    assert(p258_tune_LEN(ph) == 28);
    {
        char16_t * exemplary = u"qdkftxipeghdXdwdosKntrmhrllG";
        char16_t * sample = p258_tune_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 56);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p258_target_component_GET(pack) == (uint8_t)(uint8_t)181);
    assert(p258_target_system_GET(pack) == (uint8_t)(uint8_t)55);
};


void c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)92, (uint8_t)168, (uint8_t)64, (uint8_t)30, (uint8_t)91, (uint8_t)88, (uint8_t)93, (uint8_t)69, (uint8_t)15, (uint8_t)58, (uint8_t)159, (uint8_t)124, (uint8_t)105, (uint8_t)2, (uint8_t)212, (uint8_t)93, (uint8_t)36, (uint8_t)207, (uint8_t)67, (uint8_t)253, (uint8_t)240, (uint8_t)228, (uint8_t)188, (uint8_t)93, (uint8_t)57, (uint8_t)199, (uint8_t)249, (uint8_t)191, (uint8_t)213, (uint8_t)97, (uint8_t)73, (uint8_t)254} ;
        uint8_t*  sample = p259_vendor_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_cam_definition_uri_LEN(ph) == 76);
    {
        char16_t * exemplary = u"htiuObNlzzkdzszPdqowbfhbuwfxfvzpspibtjscrbGhjxkiistktnonrubySbliLgxqezuxxtvp";
        char16_t * sample = p259_cam_definition_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 152);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_flags_GET(pack) == e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
    assert(p259_firmware_version_GET(pack) == (uint32_t)173941620L);
    assert(p259_lens_id_GET(pack) == (uint8_t)(uint8_t)25);
    assert(p259_sensor_size_h_GET(pack) == (float) -2.2695404E38F);
    assert(p259_focal_length_GET(pack) == (float)3.3853729E38F);
    assert(p259_time_boot_ms_GET(pack) == (uint32_t)1821802759L);
    {
        uint8_t exemplary[] =  {(uint8_t)186, (uint8_t)234, (uint8_t)164, (uint8_t)104, (uint8_t)3, (uint8_t)195, (uint8_t)161, (uint8_t)209, (uint8_t)155, (uint8_t)223, (uint8_t)40, (uint8_t)149, (uint8_t)25, (uint8_t)61, (uint8_t)139, (uint8_t)239, (uint8_t)221, (uint8_t)122, (uint8_t)82, (uint8_t)36, (uint8_t)42, (uint8_t)132, (uint8_t)181, (uint8_t)141, (uint8_t)179, (uint8_t)187, (uint8_t)166, (uint8_t)102, (uint8_t)79, (uint8_t)241, (uint8_t)202, (uint8_t)74} ;
        uint8_t*  sample = p259_model_name_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p259_sensor_size_v_GET(pack) == (float)1.2195027E38F);
    assert(p259_resolution_v_GET(pack) == (uint16_t)(uint16_t)19651);
    assert(p259_cam_definition_version_GET(pack) == (uint16_t)(uint16_t)30039);
    assert(p259_resolution_h_GET(pack) == (uint16_t)(uint16_t)37625);
};


void c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(Bounds_Inside * ph, Pack * pack)
{
    assert(p260_mode_id_GET(pack) == e_CAMERA_MODE_CAMERA_MODE_VIDEO);
    assert(p260_time_boot_ms_GET(pack) == (uint32_t)2543545987L);
};


void c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(Bounds_Inside * ph, Pack * pack)
{
    assert(p261_total_capacity_GET(pack) == (float)2.5083615E38F);
    assert(p261_time_boot_ms_GET(pack) == (uint32_t)2120671094L);
    assert(p261_storage_id_GET(pack) == (uint8_t)(uint8_t)170);
    assert(p261_read_speed_GET(pack) == (float)2.3645215E38F);
    assert(p261_used_capacity_GET(pack) == (float) -1.5085219E38F);
    assert(p261_available_capacity_GET(pack) == (float) -1.8836227E38F);
    assert(p261_write_speed_GET(pack) == (float) -1.5468481E38F);
    assert(p261_status_GET(pack) == (uint8_t)(uint8_t)95);
    assert(p261_storage_count_GET(pack) == (uint8_t)(uint8_t)18);
};


void c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(Bounds_Inside * ph, Pack * pack)
{
    assert(p262_available_capacity_GET(pack) == (float) -2.2314922E38F);
    assert(p262_video_status_GET(pack) == (uint8_t)(uint8_t)212);
    assert(p262_time_boot_ms_GET(pack) == (uint32_t)3895567167L);
    assert(p262_image_status_GET(pack) == (uint8_t)(uint8_t)185);
    assert(p262_recording_time_ms_GET(pack) == (uint32_t)507410133L);
    assert(p262_image_interval_GET(pack) == (float) -4.4926627E37F);
};


void c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(Bounds_Inside * ph, Pack * pack)
{
    assert(p263_time_boot_ms_GET(pack) == (uint32_t)571428434L);
    {
        float exemplary[] =  {-5.818606E37F, 3.0544504E38F, 1.4490953E38F, -9.109118E37F} ;
        float*  sample = p263_q_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_alt_GET(pack) == (int32_t) -1368935894);
    assert(p263_image_index_GET(pack) == (int32_t) -385716810);
    assert(p263_capture_result_GET(pack) == (int8_t)(int8_t)79);
    assert(p263_file_url_LEN(ph) == 77);
    {
        char16_t * exemplary = u"ScsfrzukitEqkoUpedodkjrsvHHBgqqtnGomxctJymorPyVpKybwzqeskacxnxtbwyukdobqigiax";
        char16_t * sample = p263_file_url_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 154);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p263_lon_GET(pack) == (int32_t)261890399);
    assert(p263_relative_alt_GET(pack) == (int32_t)10281962);
    assert(p263_lat_GET(pack) == (int32_t) -193184169);
    assert(p263_camera_id_GET(pack) == (uint8_t)(uint8_t)99);
    assert(p263_time_utc_GET(pack) == (uint64_t)9107741541208605445L);
};


void c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(Bounds_Inside * ph, Pack * pack)
{
    assert(p264_flight_uuid_GET(pack) == (uint64_t)3432743130748652848L);
    assert(p264_arming_time_utc_GET(pack) == (uint64_t)8841255504274704073L);
    assert(p264_time_boot_ms_GET(pack) == (uint32_t)2961715268L);
    assert(p264_takeoff_time_utc_GET(pack) == (uint64_t)7984116458012481603L);
};


void c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(Bounds_Inside * ph, Pack * pack)
{
    assert(p265_time_boot_ms_GET(pack) == (uint32_t)2590057147L);
    assert(p265_roll_GET(pack) == (float) -1.4558429E38F);
    assert(p265_yaw_GET(pack) == (float) -3.2093696E37F);
    assert(p265_pitch_GET(pack) == (float)3.3184495E38F);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_266(Bounds_Inside * ph, Pack * pack)
{
    assert(p266_target_component_GET(pack) == (uint8_t)(uint8_t)224);
    {
        uint8_t exemplary[] =  {(uint8_t)86, (uint8_t)30, (uint8_t)53, (uint8_t)192, (uint8_t)242, (uint8_t)34, (uint8_t)157, (uint8_t)2, (uint8_t)136, (uint8_t)241, (uint8_t)232, (uint8_t)230, (uint8_t)79, (uint8_t)207, (uint8_t)71, (uint8_t)158, (uint8_t)248, (uint8_t)123, (uint8_t)34, (uint8_t)22, (uint8_t)134, (uint8_t)245, (uint8_t)25, (uint8_t)79, (uint8_t)35, (uint8_t)246, (uint8_t)211, (uint8_t)60, (uint8_t)26, (uint8_t)0, (uint8_t)35, (uint8_t)130, (uint8_t)230, (uint8_t)151, (uint8_t)100, (uint8_t)87, (uint8_t)14, (uint8_t)111, (uint8_t)38, (uint8_t)221, (uint8_t)168, (uint8_t)240, (uint8_t)103, (uint8_t)99, (uint8_t)253, (uint8_t)8, (uint8_t)230, (uint8_t)121, (uint8_t)240, (uint8_t)73, (uint8_t)106, (uint8_t)211, (uint8_t)18, (uint8_t)141, (uint8_t)207, (uint8_t)164, (uint8_t)3, (uint8_t)133, (uint8_t)212, (uint8_t)71, (uint8_t)223, (uint8_t)165, (uint8_t)14, (uint8_t)48, (uint8_t)229, (uint8_t)99, (uint8_t)62, (uint8_t)255, (uint8_t)113, (uint8_t)131, (uint8_t)60, (uint8_t)57, (uint8_t)230, (uint8_t)119, (uint8_t)234, (uint8_t)90, (uint8_t)54, (uint8_t)119, (uint8_t)217, (uint8_t)84, (uint8_t)206, (uint8_t)150, (uint8_t)187, (uint8_t)93, (uint8_t)237, (uint8_t)130, (uint8_t)102, (uint8_t)176, (uint8_t)231, (uint8_t)17, (uint8_t)116, (uint8_t)187, (uint8_t)20, (uint8_t)89, (uint8_t)255, (uint8_t)33, (uint8_t)120, (uint8_t)205, (uint8_t)204, (uint8_t)222, (uint8_t)158, (uint8_t)182, (uint8_t)143, (uint8_t)218, (uint8_t)6, (uint8_t)235, (uint8_t)238, (uint8_t)80, (uint8_t)167, (uint8_t)183, (uint8_t)228, (uint8_t)195, (uint8_t)15, (uint8_t)220, (uint8_t)87, (uint8_t)171, (uint8_t)63, (uint8_t)189, (uint8_t)62, (uint8_t)208, (uint8_t)203, (uint8_t)242, (uint8_t)208, (uint8_t)166, (uint8_t)73, (uint8_t)101, (uint8_t)144, (uint8_t)121, (uint8_t)70, (uint8_t)140, (uint8_t)86, (uint8_t)108, (uint8_t)7, (uint8_t)4, (uint8_t)153, (uint8_t)124, (uint8_t)197, (uint8_t)156, (uint8_t)99, (uint8_t)186, (uint8_t)24, (uint8_t)113, (uint8_t)228, (uint8_t)154, (uint8_t)232, (uint8_t)139, (uint8_t)189, (uint8_t)20, (uint8_t)28, (uint8_t)102, (uint8_t)211, (uint8_t)134, (uint8_t)240, (uint8_t)166, (uint8_t)176, (uint8_t)210, (uint8_t)154, (uint8_t)132, (uint8_t)88, (uint8_t)26, (uint8_t)88, (uint8_t)85, (uint8_t)25, (uint8_t)31, (uint8_t)24, (uint8_t)101, (uint8_t)225, (uint8_t)135, (uint8_t)46, (uint8_t)112, (uint8_t)70, (uint8_t)80, (uint8_t)101, (uint8_t)169, (uint8_t)104, (uint8_t)77, (uint8_t)172, (uint8_t)100, (uint8_t)129, (uint8_t)23, (uint8_t)43, (uint8_t)13, (uint8_t)203, (uint8_t)33, (uint8_t)18, (uint8_t)82, (uint8_t)68, (uint8_t)74, (uint8_t)224, (uint8_t)111, (uint8_t)79, (uint8_t)61, (uint8_t)237, (uint8_t)159, (uint8_t)144, (uint8_t)222, (uint8_t)136, (uint8_t)7, (uint8_t)43, (uint8_t)86, (uint8_t)216, (uint8_t)169, (uint8_t)154, (uint8_t)234, (uint8_t)34, (uint8_t)162, (uint8_t)103, (uint8_t)116, (uint8_t)5, (uint8_t)162, (uint8_t)180, (uint8_t)211, (uint8_t)33, (uint8_t)44, (uint8_t)199, (uint8_t)254, (uint8_t)103, (uint8_t)147, (uint8_t)83, (uint8_t)233, (uint8_t)203, (uint8_t)31, (uint8_t)92, (uint8_t)243, (uint8_t)28, (uint8_t)157, (uint8_t)149, (uint8_t)233, (uint8_t)163, (uint8_t)142, (uint8_t)67, (uint8_t)25, (uint8_t)191, (uint8_t)234, (uint8_t)191, (uint8_t)19, (uint8_t)34, (uint8_t)231, (uint8_t)241, (uint8_t)126, (uint8_t)214, (uint8_t)255, (uint8_t)215, (uint8_t)186, (uint8_t)204, (uint8_t)117, (uint8_t)96, (uint8_t)255, (uint8_t)165} ;
        uint8_t*  sample = p266_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p266_sequence_GET(pack) == (uint16_t)(uint16_t)15284);
    assert(p266_length_GET(pack) == (uint8_t)(uint8_t)19);
    assert(p266_first_message_offset_GET(pack) == (uint8_t)(uint8_t)42);
    assert(p266_target_system_GET(pack) == (uint8_t)(uint8_t)37);
};


void c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(Bounds_Inside * ph, Pack * pack)
{
    assert(p267_sequence_GET(pack) == (uint16_t)(uint16_t)52064);
    assert(p267_first_message_offset_GET(pack) == (uint8_t)(uint8_t)158);
    assert(p267_length_GET(pack) == (uint8_t)(uint8_t)32);
    assert(p267_target_component_GET(pack) == (uint8_t)(uint8_t)39);
    assert(p267_target_system_GET(pack) == (uint8_t)(uint8_t)166);
    {
        uint8_t exemplary[] =  {(uint8_t)213, (uint8_t)206, (uint8_t)155, (uint8_t)77, (uint8_t)175, (uint8_t)157, (uint8_t)194, (uint8_t)142, (uint8_t)15, (uint8_t)180, (uint8_t)19, (uint8_t)46, (uint8_t)171, (uint8_t)137, (uint8_t)71, (uint8_t)222, (uint8_t)43, (uint8_t)194, (uint8_t)21, (uint8_t)51, (uint8_t)198, (uint8_t)249, (uint8_t)46, (uint8_t)186, (uint8_t)205, (uint8_t)118, (uint8_t)39, (uint8_t)228, (uint8_t)14, (uint8_t)115, (uint8_t)212, (uint8_t)221, (uint8_t)133, (uint8_t)120, (uint8_t)46, (uint8_t)53, (uint8_t)64, (uint8_t)214, (uint8_t)47, (uint8_t)140, (uint8_t)155, (uint8_t)41, (uint8_t)106, (uint8_t)99, (uint8_t)174, (uint8_t)153, (uint8_t)224, (uint8_t)179, (uint8_t)228, (uint8_t)131, (uint8_t)22, (uint8_t)29, (uint8_t)196, (uint8_t)167, (uint8_t)23, (uint8_t)231, (uint8_t)6, (uint8_t)72, (uint8_t)9, (uint8_t)174, (uint8_t)251, (uint8_t)126, (uint8_t)254, (uint8_t)156, (uint8_t)220, (uint8_t)124, (uint8_t)57, (uint8_t)172, (uint8_t)34, (uint8_t)158, (uint8_t)194, (uint8_t)35, (uint8_t)151, (uint8_t)187, (uint8_t)216, (uint8_t)83, (uint8_t)194, (uint8_t)152, (uint8_t)239, (uint8_t)192, (uint8_t)158, (uint8_t)83, (uint8_t)252, (uint8_t)243, (uint8_t)47, (uint8_t)33, (uint8_t)144, (uint8_t)201, (uint8_t)144, (uint8_t)160, (uint8_t)130, (uint8_t)20, (uint8_t)237, (uint8_t)152, (uint8_t)17, (uint8_t)71, (uint8_t)183, (uint8_t)247, (uint8_t)209, (uint8_t)213, (uint8_t)207, (uint8_t)68, (uint8_t)236, (uint8_t)99, (uint8_t)203, (uint8_t)173, (uint8_t)25, (uint8_t)102, (uint8_t)248, (uint8_t)148, (uint8_t)183, (uint8_t)147, (uint8_t)26, (uint8_t)34, (uint8_t)28, (uint8_t)124, (uint8_t)177, (uint8_t)83, (uint8_t)219, (uint8_t)136, (uint8_t)64, (uint8_t)27, (uint8_t)123, (uint8_t)109, (uint8_t)211, (uint8_t)200, (uint8_t)170, (uint8_t)111, (uint8_t)70, (uint8_t)2, (uint8_t)218, (uint8_t)33, (uint8_t)12, (uint8_t)69, (uint8_t)223, (uint8_t)176, (uint8_t)178, (uint8_t)193, (uint8_t)224, (uint8_t)3, (uint8_t)196, (uint8_t)73, (uint8_t)122, (uint8_t)145, (uint8_t)33, (uint8_t)140, (uint8_t)91, (uint8_t)16, (uint8_t)248, (uint8_t)241, (uint8_t)88, (uint8_t)48, (uint8_t)73, (uint8_t)144, (uint8_t)173, (uint8_t)234, (uint8_t)102, (uint8_t)240, (uint8_t)114, (uint8_t)196, (uint8_t)68, (uint8_t)187, (uint8_t)30, (uint8_t)137, (uint8_t)176, (uint8_t)85, (uint8_t)246, (uint8_t)152, (uint8_t)157, (uint8_t)8, (uint8_t)181, (uint8_t)207, (uint8_t)200, (uint8_t)51, (uint8_t)188, (uint8_t)181, (uint8_t)186, (uint8_t)183, (uint8_t)21, (uint8_t)226, (uint8_t)97, (uint8_t)157, (uint8_t)108, (uint8_t)24, (uint8_t)116, (uint8_t)190, (uint8_t)10, (uint8_t)144, (uint8_t)127, (uint8_t)41, (uint8_t)97, (uint8_t)173, (uint8_t)208, (uint8_t)111, (uint8_t)38, (uint8_t)102, (uint8_t)94, (uint8_t)219, (uint8_t)134, (uint8_t)234, (uint8_t)56, (uint8_t)164, (uint8_t)211, (uint8_t)67, (uint8_t)103, (uint8_t)192, (uint8_t)47, (uint8_t)144, (uint8_t)45, (uint8_t)77, (uint8_t)159, (uint8_t)212, (uint8_t)14, (uint8_t)70, (uint8_t)26, (uint8_t)216, (uint8_t)10, (uint8_t)179, (uint8_t)91, (uint8_t)126, (uint8_t)112, (uint8_t)84, (uint8_t)107, (uint8_t)74, (uint8_t)33, (uint8_t)204, (uint8_t)30, (uint8_t)130, (uint8_t)74, (uint8_t)207, (uint8_t)187, (uint8_t)133, (uint8_t)210, (uint8_t)201, (uint8_t)147, (uint8_t)118, (uint8_t)249, (uint8_t)10, (uint8_t)38, (uint8_t)251, (uint8_t)54, (uint8_t)31, (uint8_t)251, (uint8_t)162, (uint8_t)219, (uint8_t)54, (uint8_t)184, (uint8_t)131, (uint8_t)62} ;
        uint8_t*  sample = p267_data__GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 249);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_LOGGING_ACK_268(Bounds_Inside * ph, Pack * pack)
{
    assert(p268_target_system_GET(pack) == (uint8_t)(uint8_t)95);
    assert(p268_sequence_GET(pack) == (uint16_t)(uint16_t)61111);
    assert(p268_target_component_GET(pack) == (uint8_t)(uint8_t)17);
};


void c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(Bounds_Inside * ph, Pack * pack)
{
    assert(p269_resolution_h_GET(pack) == (uint16_t)(uint16_t)25320);
    assert(p269_rotation_GET(pack) == (uint16_t)(uint16_t)8314);
    assert(p269_status_GET(pack) == (uint8_t)(uint8_t)190);
    assert(p269_uri_LEN(ph) == 8);
    {
        char16_t * exemplary = u"naYrwldy";
        char16_t * sample = p269_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p269_camera_id_GET(pack) == (uint8_t)(uint8_t)27);
    assert(p269_resolution_v_GET(pack) == (uint16_t)(uint16_t)15833);
    assert(p269_framerate_GET(pack) == (float) -2.5839477E38F);
    assert(p269_bitrate_GET(pack) == (uint32_t)1991029241L);
};


void c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(Bounds_Inside * ph, Pack * pack)
{
    assert(p270_target_system_GET(pack) == (uint8_t)(uint8_t)182);
    assert(p270_rotation_GET(pack) == (uint16_t)(uint16_t)59475);
    assert(p270_target_component_GET(pack) == (uint8_t)(uint8_t)46);
    assert(p270_framerate_GET(pack) == (float) -3.1695263E38F);
    assert(p270_camera_id_GET(pack) == (uint8_t)(uint8_t)15);
    assert(p270_uri_LEN(ph) == 147);
    {
        char16_t * exemplary = u"gqfvbLdvlwOklrmicvhrrbawxeptqkzwCyejpemcdyanemrcysntngkbidscafOnedxyjpqnUwareodifQlfhAVckornmvsgwhojwdsxjvuzsmdvqvqueodpelhvqzyNwycmdtMlsUpaXHgtuRb";
        char16_t * sample = p270_uri_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 294);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p270_bitrate_GET(pack) == (uint32_t)2452441953L);
    assert(p270_resolution_v_GET(pack) == (uint16_t)(uint16_t)55514);
    assert(p270_resolution_h_GET(pack) == (uint16_t)(uint16_t)36819);
};


void c_LoopBackDemoChannel_on_WIFI_CONFIG_AP_299(Bounds_Inside * ph, Pack * pack)
{
    assert(p299_ssid_LEN(ph) == 14);
    {
        char16_t * exemplary = u"kpbGkmtoflvoXk";
        char16_t * sample = p299_ssid_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 28);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p299_password_LEN(ph) == 1);
    {
        char16_t * exemplary = u"g";
        char16_t * sample = p299_password_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
};


void c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)217, (uint8_t)220, (uint8_t)208, (uint8_t)94, (uint8_t)233, (uint8_t)193, (uint8_t)84, (uint8_t)178} ;
        uint8_t*  sample = p300_library_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    {
        uint8_t exemplary[] =  {(uint8_t)139, (uint8_t)91, (uint8_t)15, (uint8_t)2, (uint8_t)19, (uint8_t)226, (uint8_t)122, (uint8_t)232} ;
        uint8_t*  sample = p300_spec_version_hash_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 8);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p300_min_version_GET(pack) == (uint16_t)(uint16_t)25249);
    assert(p300_version_GET(pack) == (uint16_t)(uint16_t)6140);
    assert(p300_max_version_GET(pack) == (uint16_t)(uint16_t)8464);
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(Bounds_Inside * ph, Pack * pack)
{
    assert(p310_uptime_sec_GET(pack) == (uint32_t)1322284972L);
    assert(p310_time_usec_GET(pack) == (uint64_t)1105401235972618759L);
    assert(p310_sub_mode_GET(pack) == (uint8_t)(uint8_t)200);
    assert(p310_mode_GET(pack) == e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION);
    assert(p310_vendor_specific_status_code_GET(pack) == (uint16_t)(uint16_t)27328);
    assert(p310_health_GET(pack) == e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_OK);
};


void c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(Bounds_Inside * ph, Pack * pack)
{
    {
        uint8_t exemplary[] =  {(uint8_t)203, (uint8_t)223, (uint8_t)123, (uint8_t)101, (uint8_t)162, (uint8_t)216, (uint8_t)121, (uint8_t)228, (uint8_t)123, (uint8_t)248, (uint8_t)141, (uint8_t)103, (uint8_t)29, (uint8_t)107, (uint8_t)159, (uint8_t)201} ;
        uint8_t*  sample = p311_hw_unique_id_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 16);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_time_usec_GET(pack) == (uint64_t)3775263283503279207L);
    assert(p311_uptime_sec_GET(pack) == (uint32_t)421995175L);
    assert(p311_sw_version_minor_GET(pack) == (uint8_t)(uint8_t)12);
    assert(p311_name_LEN(ph) == 32);
    {
        char16_t * exemplary = u"shdjhaazryLwrbjalxkePthcgpsdintq";
        char16_t * sample = p311_name_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 64);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p311_hw_version_minor_GET(pack) == (uint8_t)(uint8_t)81);
    assert(p311_sw_vcs_commit_GET(pack) == (uint32_t)3846086153L);
    assert(p311_hw_version_major_GET(pack) == (uint8_t)(uint8_t)136);
    assert(p311_sw_version_major_GET(pack) == (uint8_t)(uint8_t)80);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(Bounds_Inside * ph, Pack * pack)
{
    assert(p320_target_system_GET(pack) == (uint8_t)(uint8_t)43);
    assert(p320_target_component_GET(pack) == (uint8_t)(uint8_t)174);
    assert(p320_param_id_LEN(ph) == 16);
    {
        char16_t * exemplary = u"vcpruzznsnyowdra";
        char16_t * sample = p320_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 32);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p320_param_index_GET(pack) == (int16_t)(int16_t)7035);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(Bounds_Inside * ph, Pack * pack)
{
    assert(p321_target_system_GET(pack) == (uint8_t)(uint8_t)199);
    assert(p321_target_component_GET(pack) == (uint8_t)(uint8_t)240);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(Bounds_Inside * ph, Pack * pack)
{
    assert(p322_param_id_LEN(ph) == 11);
    {
        char16_t * exemplary = u"DjmyynrbaBj";
        char16_t * sample = p322_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 22);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_value_LEN(ph) == 79);
    {
        char16_t * exemplary = u"wgnvkeqqszhravndketozSkZzibsnteiDmjuVDhhozvzBcuwgHwopmkqejwytfjatvvwkVyhnsmbxVO";
        char16_t * sample = p322_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 158);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p322_param_count_GET(pack) == (uint16_t)(uint16_t)18228);
    assert(p322_param_index_GET(pack) == (uint16_t)(uint16_t)10487);
    assert(p322_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(Bounds_Inside * ph, Pack * pack)
{
    assert(p323_param_id_LEN(ph) == 14);
    {
        char16_t * exemplary = u"hwygjszfnzjvqd";
        char16_t * sample = p323_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 28);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_param_value_LEN(ph) == 69);
    {
        char16_t * exemplary = u"hdeIabyzgqlpLqKvYbolLYtxdxgcqcqdgayavgzxmniifafbIbsrcJljyvzxrjweqxyPq";
        char16_t * sample = p323_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 138);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p323_target_system_GET(pack) == (uint8_t)(uint8_t)179);
    assert(p323_target_component_GET(pack) == (uint8_t)(uint8_t)147);
    assert(p323_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64);
};


void c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(Bounds_Inside * ph, Pack * pack)
{
    assert(p324_param_id_LEN(ph) == 1);
    {
        char16_t * exemplary = u"o";
        char16_t * sample = p324_param_id_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 2);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_value_LEN(ph) == 101);
    {
        char16_t * exemplary = u"iaxvzmoeyaesqqrRonJxoktxzvotvnxjfswCjvjfysoddeezmnraqwwvbjfgoxJbigssyacdnztsQrsishdhpaezctduhAxXJzbvt";
        char16_t * sample = p324_param_value_TRY_(ph);
        int32_t result = Arrays_equals((uint8_t*)exemplary, (uint8_t*)sample, 202);
        assert(result == -1);
        free(sample);//do not forget to dispose
    }
    assert(p324_param_type_GET(pack) == e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32);
    assert(p324_param_result_GET(pack) == e_PARAM_ACK_PARAM_ACK_ACCEPTED);
};


void c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(Bounds_Inside * ph, Pack * pack)
{
    assert(p330_time_usec_GET(pack) == (uint64_t)6911494538107067992L);
    assert(p330_sensor_type_GET(pack) == e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED);
    assert(p330_max_distance_GET(pack) == (uint16_t)(uint16_t)7515);
    assert(p330_increment_GET(pack) == (uint8_t)(uint8_t)198);
    assert(p330_min_distance_GET(pack) == (uint16_t)(uint16_t)11235);
    {
        uint16_t exemplary[] =  {(uint16_t)44137, (uint16_t)22125, (uint16_t)17735, (uint16_t)30030, (uint16_t)30990, (uint16_t)24675, (uint16_t)3851, (uint16_t)21313, (uint16_t)64085, (uint16_t)42614, (uint16_t)62643, (uint16_t)24231, (uint16_t)41072, (uint16_t)22788, (uint16_t)15953, (uint16_t)43146, (uint16_t)56697, (uint16_t)40202, (uint16_t)55668, (uint16_t)9643, (uint16_t)28182, (uint16_t)28758, (uint16_t)11867, (uint16_t)14552, (uint16_t)63865, (uint16_t)30745, (uint16_t)24501, (uint16_t)24259, (uint16_t)56108, (uint16_t)7588, (uint16_t)17244, (uint16_t)59621, (uint16_t)38986, (uint16_t)18618, (uint16_t)33004, (uint16_t)27135, (uint16_t)6193, (uint16_t)52674, (uint16_t)6156, (uint16_t)27581, (uint16_t)48442, (uint16_t)31811, (uint16_t)41163, (uint16_t)60429, (uint16_t)7264, (uint16_t)47044, (uint16_t)12372, (uint16_t)17853, (uint16_t)37227, (uint16_t)16270, (uint16_t)978, (uint16_t)16221, (uint16_t)53054, (uint16_t)11867, (uint16_t)54521, (uint16_t)8797, (uint16_t)10609, (uint16_t)44340, (uint16_t)44575, (uint16_t)13799, (uint16_t)47023, (uint16_t)21845, (uint16_t)2869, (uint16_t)11312, (uint16_t)47512, (uint16_t)59432, (uint16_t)30195, (uint16_t)21719, (uint16_t)58219, (uint16_t)33219, (uint16_t)9277, (uint16_t)25077} ;
        uint16_t*  sample = p330_distances_GET_(pack);
        int32_t result = Arrays_equals(exemplary, sample, 144);
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
        p0_mavlink_version_SET((uint8_t)(uint8_t)126, PH.base.pack) ;
        p0_system_status_SET(e_MAV_STATE_MAV_STATE_BOOT, PH.base.pack) ;
        p0_type_SET(e_MAV_TYPE_MAV_TYPE_GCS, PH.base.pack) ;
        p0_autopilot_SET(e_MAV_AUTOPILOT_MAV_AUTOPILOT_PX4, PH.base.pack) ;
        p0_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_STABILIZE_ENABLED, PH.base.pack) ;
        p0_custom_mode_SET((uint32_t)1049918282L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HEARTBEAT_0(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYS_STATUS_1(), &PH);
        p1_onboard_control_sensors_health_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE, PH.base.pack) ;
        p1_errors_count3_SET((uint16_t)(uint16_t)63080, PH.base.pack) ;
        p1_errors_count4_SET((uint16_t)(uint16_t)15473, PH.base.pack) ;
        p1_load_SET((uint16_t)(uint16_t)9147, PH.base.pack) ;
        p1_onboard_control_sensors_present_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_GEOFENCE, PH.base.pack) ;
        p1_errors_count2_SET((uint16_t)(uint16_t)39560, PH.base.pack) ;
        p1_battery_remaining_SET((int8_t)(int8_t)5, PH.base.pack) ;
        p1_errors_comm_SET((uint16_t)(uint16_t)33807, PH.base.pack) ;
        p1_drop_rate_comm_SET((uint16_t)(uint16_t)61491, PH.base.pack) ;
        p1_current_battery_SET((int16_t)(int16_t) -31306, PH.base.pack) ;
        p1_errors_count1_SET((uint16_t)(uint16_t)47809, PH.base.pack) ;
        p1_voltage_battery_SET((uint16_t)(uint16_t)32444, PH.base.pack) ;
        p1_onboard_control_sensors_enabled_SET(e_MAV_SYS_STATUS_SENSOR_MAV_SYS_STATUS_SENSOR_3D_ACCEL2, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYS_STATUS_1(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SYSTEM_TIME_2(), &PH);
        p2_time_unix_usec_SET((uint64_t)2866755814010049597L, PH.base.pack) ;
        p2_time_boot_ms_SET((uint32_t)1699224830L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SYSTEM_TIME_2(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_LOCAL_NED_3(), &PH);
        p3_yaw_rate_SET((float) -2.4803243E38F, PH.base.pack) ;
        p3_afz_SET((float)3.050678E38F, PH.base.pack) ;
        p3_type_mask_SET((uint16_t)(uint16_t)1531, PH.base.pack) ;
        p3_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        p3_yaw_SET((float) -3.0333384E38F, PH.base.pack) ;
        p3_y_SET((float) -8.768712E37F, PH.base.pack) ;
        p3_afy_SET((float) -1.120691E38F, PH.base.pack) ;
        p3_x_SET((float) -1.520683E38F, PH.base.pack) ;
        p3_vx_SET((float)3.227991E37F, PH.base.pack) ;
        p3_time_boot_ms_SET((uint32_t)3038678426L, PH.base.pack) ;
        p3_z_SET((float)1.2844377E38F, PH.base.pack) ;
        p3_vy_SET((float)1.2315801E37F, PH.base.pack) ;
        p3_vz_SET((float)8.247461E37F, PH.base.pack) ;
        p3_afx_SET((float) -3.262818E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_LOCAL_NED_3(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PING_4(), &PH);
        p4_target_system_SET((uint8_t)(uint8_t)44, PH.base.pack) ;
        p4_target_component_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        p4_seq_SET((uint32_t)2576619764L, PH.base.pack) ;
        p4_time_usec_SET((uint64_t)712765046417073337L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PING_4(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_5(), &PH);
        p5_control_request_SET((uint8_t)(uint8_t)60, PH.base.pack) ;
        p5_version_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p5_target_system_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        {
            char16_t* passkey = u"m";
            p5_passkey_SET_(passkey, &PH) ;
        }
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_5(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CHANGE_OPERATOR_CONTROL_ACK_6(), &PH);
        p6_ack_SET((uint8_t)(uint8_t)234, PH.base.pack) ;
        p6_control_request_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p6_gcs_system_id_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CHANGE_OPERATOR_CONTROL_ACK_6(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTH_KEY_7(), &PH);
        {
            char16_t* key = u"hottc";
            p7_key_SET_(key, &PH) ;
        }
        c_LoopBackDemoChannel_on_AUTH_KEY_7(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_MODE_11(), &PH);
        p11_target_system_SET((uint8_t)(uint8_t)53, PH.base.pack) ;
        p11_base_mode_SET(e_MAV_MODE_MAV_MODE_AUTO_ARMED, PH.base.pack) ;
        p11_custom_mode_SET((uint32_t)2339979983L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_MODE_11(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_READ_20(), &PH);
        p20_target_component_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p20_param_index_SET((int16_t)(int16_t)21305, PH.base.pack) ;
        p20_target_system_SET((uint8_t)(uint8_t)35, PH.base.pack) ;
        {
            char16_t* param_id = u"SqBkxkfabwwH";
            p20_param_id_SET_(param_id, &PH) ;
        }
        c_LoopBackDemoChannel_on_PARAM_REQUEST_READ_20(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_REQUEST_LIST_21(), &PH);
        p21_target_system_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p21_target_component_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_REQUEST_LIST_21(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_VALUE_22(), &PH);
        p22_param_count_SET((uint16_t)(uint16_t)60886, PH.base.pack) ;
        p22_param_index_SET((uint16_t)(uint16_t)20142, PH.base.pack) ;
        p22_param_value_SET((float)1.6750919E38F, PH.base.pack) ;
        {
            char16_t* param_id = u"hkygDresnxh";
            p22_param_id_SET_(param_id, &PH) ;
        }
        p22_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT64, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_VALUE_22(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_SET_23(), &PH);
        p23_param_value_SET((float) -1.436232E38F, PH.base.pack) ;
        p23_target_system_SET((uint8_t)(uint8_t)171, PH.base.pack) ;
        {
            char16_t* param_id = u"hnykawjqtm";
            p23_param_id_SET_(param_id, &PH) ;
        }
        p23_target_component_SET((uint8_t)(uint8_t)129, PH.base.pack) ;
        p23_param_type_SET(e_MAV_PARAM_TYPE_MAV_PARAM_TYPE_INT16, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_SET_23(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RAW_INT_24(), &PH);
        p24_lat_SET((int32_t)676362669, PH.base.pack) ;
        p24_alt_ellipsoid_SET((int32_t) -797997768, &PH) ;
        p24_satellites_visible_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p24_hdg_acc_SET((uint32_t)1144430534L, &PH) ;
        p24_alt_SET((int32_t)1058443856, PH.base.pack) ;
        p24_vel_SET((uint16_t)(uint16_t)26147, PH.base.pack) ;
        p24_epv_SET((uint16_t)(uint16_t)13371, PH.base.pack) ;
        p24_v_acc_SET((uint32_t)3377061317L, &PH) ;
        p24_vel_acc_SET((uint32_t)4120125257L, &PH) ;
        p24_h_acc_SET((uint32_t)177513135L, &PH) ;
        p24_lon_SET((int32_t) -1058936255, PH.base.pack) ;
        p24_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_3D_FIX, PH.base.pack) ;
        p24_time_usec_SET((uint64_t)3870957820941811675L, PH.base.pack) ;
        p24_eph_SET((uint16_t)(uint16_t)63128, PH.base.pack) ;
        p24_cog_SET((uint16_t)(uint16_t)20831, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RAW_INT_24(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_STATUS_25(), &PH);
        {
            uint8_t satellite_used[] =  {(uint8_t)241, (uint8_t)145, (uint8_t)17, (uint8_t)112, (uint8_t)74, (uint8_t)147, (uint8_t)177, (uint8_t)42, (uint8_t)151, (uint8_t)111, (uint8_t)111, (uint8_t)218, (uint8_t)111, (uint8_t)174, (uint8_t)47, (uint8_t)58, (uint8_t)192, (uint8_t)76, (uint8_t)115, (uint8_t)198};
            p25_satellite_used_SET(&satellite_used, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_snr[] =  {(uint8_t)6, (uint8_t)197, (uint8_t)5, (uint8_t)131, (uint8_t)176, (uint8_t)171, (uint8_t)101, (uint8_t)95, (uint8_t)69, (uint8_t)42, (uint8_t)241, (uint8_t)210, (uint8_t)237, (uint8_t)119, (uint8_t)120, (uint8_t)61, (uint8_t)197, (uint8_t)6, (uint8_t)169, (uint8_t)85};
            p25_satellite_snr_SET(&satellite_snr, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_prn[] =  {(uint8_t)93, (uint8_t)111, (uint8_t)50, (uint8_t)226, (uint8_t)60, (uint8_t)156, (uint8_t)157, (uint8_t)61, (uint8_t)23, (uint8_t)251, (uint8_t)172, (uint8_t)39, (uint8_t)103, (uint8_t)13, (uint8_t)124, (uint8_t)71, (uint8_t)157, (uint8_t)188, (uint8_t)221, (uint8_t)212};
            p25_satellite_prn_SET(&satellite_prn, 0, PH.base.pack) ;
        }
        p25_satellites_visible_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        {
            uint8_t satellite_azimuth[] =  {(uint8_t)219, (uint8_t)222, (uint8_t)74, (uint8_t)229, (uint8_t)63, (uint8_t)60, (uint8_t)122, (uint8_t)119, (uint8_t)164, (uint8_t)252, (uint8_t)97, (uint8_t)155, (uint8_t)196, (uint8_t)134, (uint8_t)234, (uint8_t)225, (uint8_t)134, (uint8_t)108, (uint8_t)143, (uint8_t)14};
            p25_satellite_azimuth_SET(&satellite_azimuth, 0, PH.base.pack) ;
        }
        {
            uint8_t satellite_elevation[] =  {(uint8_t)179, (uint8_t)118, (uint8_t)63, (uint8_t)125, (uint8_t)90, (uint8_t)128, (uint8_t)150, (uint8_t)235, (uint8_t)128, (uint8_t)49, (uint8_t)242, (uint8_t)99, (uint8_t)251, (uint8_t)109, (uint8_t)147, (uint8_t)77, (uint8_t)140, (uint8_t)6, (uint8_t)72, (uint8_t)246};
            p25_satellite_elevation_SET(&satellite_elevation, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_GPS_STATUS_25(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU_26(), &PH);
        p26_time_boot_ms_SET((uint32_t)3554329856L, PH.base.pack) ;
        p26_zmag_SET((int16_t)(int16_t) -23423, PH.base.pack) ;
        p26_yacc_SET((int16_t)(int16_t) -31597, PH.base.pack) ;
        p26_zgyro_SET((int16_t)(int16_t)25902, PH.base.pack) ;
        p26_zacc_SET((int16_t)(int16_t) -919, PH.base.pack) ;
        p26_xacc_SET((int16_t)(int16_t)12650, PH.base.pack) ;
        p26_ymag_SET((int16_t)(int16_t) -8727, PH.base.pack) ;
        p26_ygyro_SET((int16_t)(int16_t) -11714, PH.base.pack) ;
        p26_xmag_SET((int16_t)(int16_t)7331, PH.base.pack) ;
        p26_xgyro_SET((int16_t)(int16_t)29201, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU_26(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_IMU_27(), &PH);
        p27_xmag_SET((int16_t)(int16_t)16911, PH.base.pack) ;
        p27_xacc_SET((int16_t)(int16_t)32154, PH.base.pack) ;
        p27_xgyro_SET((int16_t)(int16_t)30990, PH.base.pack) ;
        p27_zmag_SET((int16_t)(int16_t)22928, PH.base.pack) ;
        p27_ygyro_SET((int16_t)(int16_t) -29201, PH.base.pack) ;
        p27_zgyro_SET((int16_t)(int16_t) -22460, PH.base.pack) ;
        p27_yacc_SET((int16_t)(int16_t)20950, PH.base.pack) ;
        p27_zacc_SET((int16_t)(int16_t)12139, PH.base.pack) ;
        p27_ymag_SET((int16_t)(int16_t) -5916, PH.base.pack) ;
        p27_time_usec_SET((uint64_t)4262991439716535167L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_IMU_27(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RAW_PRESSURE_28(), &PH);
        p28_press_diff2_SET((int16_t)(int16_t)8052, PH.base.pack) ;
        p28_press_abs_SET((int16_t)(int16_t) -4922, PH.base.pack) ;
        p28_press_diff1_SET((int16_t)(int16_t)13304, PH.base.pack) ;
        p28_time_usec_SET((uint64_t)4199173096179369917L, PH.base.pack) ;
        p28_temperature_SET((int16_t)(int16_t)15327, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RAW_PRESSURE_28(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE_29(), &PH);
        p29_press_diff_SET((float)2.806208E38F, PH.base.pack) ;
        p29_press_abs_SET((float) -1.981177E38F, PH.base.pack) ;
        p29_time_boot_ms_SET((uint32_t)1026136409L, PH.base.pack) ;
        p29_temperature_SET((int16_t)(int16_t) -16337, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE_29(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_30(), &PH);
        p30_time_boot_ms_SET((uint32_t)3230702190L, PH.base.pack) ;
        p30_roll_SET((float) -3.2297646E38F, PH.base.pack) ;
        p30_rollspeed_SET((float) -9.429545E37F, PH.base.pack) ;
        p30_yawspeed_SET((float)1.5575115E38F, PH.base.pack) ;
        p30_pitch_SET((float)1.8977837E38F, PH.base.pack) ;
        p30_pitchspeed_SET((float) -1.3189831E38F, PH.base.pack) ;
        p30_yaw_SET((float) -1.1991395E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_30(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_31(), &PH);
        p31_q3_SET((float) -2.6038704E37F, PH.base.pack) ;
        p31_q2_SET((float) -6.3507054E37F, PH.base.pack) ;
        p31_rollspeed_SET((float)4.455984E37F, PH.base.pack) ;
        p31_yawspeed_SET((float) -2.9115855E38F, PH.base.pack) ;
        p31_q4_SET((float) -3.2979093E38F, PH.base.pack) ;
        p31_pitchspeed_SET((float)1.588116E38F, PH.base.pack) ;
        p31_q1_SET((float) -2.6826912E38F, PH.base.pack) ;
        p31_time_boot_ms_SET((uint32_t)521451827L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_31(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_32(), &PH);
        p32_vy_SET((float)3.8153177E37F, PH.base.pack) ;
        p32_vz_SET((float) -1.569816E38F, PH.base.pack) ;
        p32_y_SET((float)2.524929E38F, PH.base.pack) ;
        p32_z_SET((float) -2.2222982E38F, PH.base.pack) ;
        p32_x_SET((float) -5.590345E37F, PH.base.pack) ;
        p32_time_boot_ms_SET((uint32_t)1460047116L, PH.base.pack) ;
        p32_vx_SET((float) -1.6260522E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_32(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_33(), &PH);
        p33_lat_SET((int32_t)71022867, PH.base.pack) ;
        p33_relative_alt_SET((int32_t) -320849642, PH.base.pack) ;
        p33_time_boot_ms_SET((uint32_t)3273940559L, PH.base.pack) ;
        p33_hdg_SET((uint16_t)(uint16_t)50326, PH.base.pack) ;
        p33_vz_SET((int16_t)(int16_t) -349, PH.base.pack) ;
        p33_vx_SET((int16_t)(int16_t) -22381, PH.base.pack) ;
        p33_lon_SET((int32_t)1187969523, PH.base.pack) ;
        p33_vy_SET((int16_t)(int16_t) -5430, PH.base.pack) ;
        p33_alt_SET((int32_t) -2055596479, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_33(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_SCALED_34(), &PH);
        p34_chan5_scaled_SET((int16_t)(int16_t) -27511, PH.base.pack) ;
        p34_chan4_scaled_SET((int16_t)(int16_t) -7111, PH.base.pack) ;
        p34_chan2_scaled_SET((int16_t)(int16_t)4646, PH.base.pack) ;
        p34_chan1_scaled_SET((int16_t)(int16_t)4463, PH.base.pack) ;
        p34_chan8_scaled_SET((int16_t)(int16_t) -312, PH.base.pack) ;
        p34_chan6_scaled_SET((int16_t)(int16_t) -18018, PH.base.pack) ;
        p34_chan7_scaled_SET((int16_t)(int16_t) -25389, PH.base.pack) ;
        p34_chan3_scaled_SET((int16_t)(int16_t) -18488, PH.base.pack) ;
        p34_time_boot_ms_SET((uint32_t)1188757378L, PH.base.pack) ;
        p34_rssi_SET((uint8_t)(uint8_t)50, PH.base.pack) ;
        p34_port_SET((uint8_t)(uint8_t)34, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_SCALED_34(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_RAW_35(), &PH);
        p35_chan6_raw_SET((uint16_t)(uint16_t)63730, PH.base.pack) ;
        p35_port_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        p35_chan4_raw_SET((uint16_t)(uint16_t)61073, PH.base.pack) ;
        p35_chan1_raw_SET((uint16_t)(uint16_t)49939, PH.base.pack) ;
        p35_chan2_raw_SET((uint16_t)(uint16_t)29717, PH.base.pack) ;
        p35_chan5_raw_SET((uint16_t)(uint16_t)27673, PH.base.pack) ;
        p35_chan7_raw_SET((uint16_t)(uint16_t)57296, PH.base.pack) ;
        p35_rssi_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p35_time_boot_ms_SET((uint32_t)1302107391L, PH.base.pack) ;
        p35_chan8_raw_SET((uint16_t)(uint16_t)33051, PH.base.pack) ;
        p35_chan3_raw_SET((uint16_t)(uint16_t)30874, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_RAW_35(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERVO_OUTPUT_RAW_36(), &PH);
        p36_port_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        p36_servo8_raw_SET((uint16_t)(uint16_t)65280, PH.base.pack) ;
        p36_servo1_raw_SET((uint16_t)(uint16_t)41276, PH.base.pack) ;
        p36_time_usec_SET((uint32_t)3062002973L, PH.base.pack) ;
        p36_servo2_raw_SET((uint16_t)(uint16_t)32962, PH.base.pack) ;
        p36_servo15_raw_SET((uint16_t)(uint16_t)11477, &PH) ;
        p36_servo4_raw_SET((uint16_t)(uint16_t)59338, PH.base.pack) ;
        p36_servo12_raw_SET((uint16_t)(uint16_t)37724, &PH) ;
        p36_servo13_raw_SET((uint16_t)(uint16_t)63391, &PH) ;
        p36_servo11_raw_SET((uint16_t)(uint16_t)25061, &PH) ;
        p36_servo16_raw_SET((uint16_t)(uint16_t)43522, &PH) ;
        p36_servo9_raw_SET((uint16_t)(uint16_t)34337, &PH) ;
        p36_servo7_raw_SET((uint16_t)(uint16_t)28326, PH.base.pack) ;
        p36_servo6_raw_SET((uint16_t)(uint16_t)58305, PH.base.pack) ;
        p36_servo10_raw_SET((uint16_t)(uint16_t)56959, &PH) ;
        p36_servo14_raw_SET((uint16_t)(uint16_t)10154, &PH) ;
        p36_servo3_raw_SET((uint16_t)(uint16_t)26079, PH.base.pack) ;
        p36_servo5_raw_SET((uint16_t)(uint16_t)44698, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERVO_OUTPUT_RAW_36(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_PARTIAL_LIST_37(), &PH);
        p37_target_component_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        p37_target_system_SET((uint8_t)(uint8_t)146, PH.base.pack) ;
        p37_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p37_end_index_SET((int16_t)(int16_t)20090, PH.base.pack) ;
        p37_start_index_SET((int16_t)(int16_t) -22054, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_PARTIAL_LIST_37(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_WRITE_PARTIAL_LIST_38(), &PH);
        p38_target_component_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        p38_end_index_SET((int16_t)(int16_t)31157, PH.base.pack) ;
        p38_start_index_SET((int16_t)(int16_t)8280, PH.base.pack) ;
        p38_target_system_SET((uint8_t)(uint8_t)41, PH.base.pack) ;
        p38_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_WRITE_PARTIAL_LIST_38(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_39(), &PH);
        p39_param2_SET((float)2.9011494E38F, PH.base.pack) ;
        p39_param3_SET((float) -2.0781213E38F, PH.base.pack) ;
        p39_z_SET((float)1.2396276E38F, PH.base.pack) ;
        p39_target_system_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        p39_autocontinue_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
        p39_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        p39_command_SET(e_MAV_CMD_MAV_CMD_IMAGE_STOP_CAPTURE, PH.base.pack) ;
        p39_seq_SET((uint16_t)(uint16_t)29244, PH.base.pack) ;
        p39_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        p39_target_component_SET((uint8_t)(uint8_t)194, PH.base.pack) ;
        p39_current_SET((uint8_t)(uint8_t)38, PH.base.pack) ;
        p39_param4_SET((float) -1.1201141E38F, PH.base.pack) ;
        p39_y_SET((float) -1.0301921E38F, PH.base.pack) ;
        p39_param1_SET((float)2.9931727E38F, PH.base.pack) ;
        p39_x_SET((float) -2.3123584E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_39(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_40(), &PH);
        p40_seq_SET((uint16_t)(uint16_t)53438, PH.base.pack) ;
        p40_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_RALLY, PH.base.pack) ;
        p40_target_component_SET((uint8_t)(uint8_t)228, PH.base.pack) ;
        p40_target_system_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_40(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_SET_CURRENT_41(), &PH);
        p41_target_system_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p41_seq_SET((uint16_t)(uint16_t)36538, PH.base.pack) ;
        p41_target_component_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_SET_CURRENT_41(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CURRENT_42(), &PH);
        p42_seq_SET((uint16_t)(uint16_t)18361, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CURRENT_42(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_LIST_43(), &PH);
        p43_target_component_SET((uint8_t)(uint8_t)245, PH.base.pack) ;
        p43_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p43_target_system_SET((uint8_t)(uint8_t)124, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_LIST_43(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_COUNT_44(), &PH);
        p44_target_component_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        p44_target_system_SET((uint8_t)(uint8_t)113, PH.base.pack) ;
        p44_count_SET((uint16_t)(uint16_t)59093, PH.base.pack) ;
        p44_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_COUNT_44(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_CLEAR_ALL_45(), &PH);
        p45_target_component_SET((uint8_t)(uint8_t)186, PH.base.pack) ;
        p45_target_system_SET((uint8_t)(uint8_t)150, PH.base.pack) ;
        p45_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_ALL, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_CLEAR_ALL_45(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_REACHED_46(), &PH);
        p46_seq_SET((uint16_t)(uint16_t)5439, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_REACHED_46(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ACK_47(), &PH);
        p47_target_component_SET((uint8_t)(uint8_t)249, PH.base.pack) ;
        p47_target_system_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        p47_type_SET(e_MAV_MISSION_RESULT_MAV_MISSION_ERROR, PH.base.pack) ;
        p47_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ACK_47(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_GPS_GLOBAL_ORIGIN_48(), &PH);
        p48_time_usec_SET((uint64_t)1560590423219178449L, &PH) ;
        p48_latitude_SET((int32_t)819597432, PH.base.pack) ;
        p48_longitude_SET((int32_t)2023987836, PH.base.pack) ;
        p48_altitude_SET((int32_t)285354055, PH.base.pack) ;
        p48_target_system_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_GPS_GLOBAL_ORIGIN_48(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_GLOBAL_ORIGIN_49(), &PH);
        p49_latitude_SET((int32_t) -230262978, PH.base.pack) ;
        p49_longitude_SET((int32_t)1694997785, PH.base.pack) ;
        p49_altitude_SET((int32_t)2068706508, PH.base.pack) ;
        p49_time_usec_SET((uint64_t)5341489511699149884L, &PH) ;
        c_LoopBackDemoChannel_on_GPS_GLOBAL_ORIGIN_49(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_MAP_RC_50(), &PH);
        p50_param_value_min_SET((float) -2.5075824E38F, PH.base.pack) ;
        p50_param_index_SET((int16_t)(int16_t)13257, PH.base.pack) ;
        p50_target_system_SET((uint8_t)(uint8_t)16, PH.base.pack) ;
        p50_param_value0_SET((float)2.9497071E38F, PH.base.pack) ;
        {
            char16_t* param_id = u"wdgbatp";
            p50_param_id_SET_(param_id, &PH) ;
        }
        p50_target_component_SET((uint8_t)(uint8_t)152, PH.base.pack) ;
        p50_scale_SET((float)6.4341376E37F, PH.base.pack) ;
        p50_parameter_rc_channel_index_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        p50_param_value_max_SET((float)2.1340624E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_MAP_RC_50(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_REQUEST_INT_51(), &PH);
        p51_target_component_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p51_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_FENCE, PH.base.pack) ;
        p51_seq_SET((uint16_t)(uint16_t)10183, PH.base.pack) ;
        p51_target_system_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_REQUEST_INT_51(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_SET_ALLOWED_AREA_54(), &PH);
        p54_target_system_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p54_p2x_SET((float) -2.2655082E38F, PH.base.pack) ;
        p54_p1z_SET((float)7.183131E37F, PH.base.pack) ;
        p54_p1y_SET((float) -1.5139381E38F, PH.base.pack) ;
        p54_p2y_SET((float) -2.356907E38F, PH.base.pack) ;
        p54_target_component_SET((uint8_t)(uint8_t)9, PH.base.pack) ;
        p54_p1x_SET((float) -2.458482E38F, PH.base.pack) ;
        p54_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, PH.base.pack) ;
        p54_p2z_SET((float)1.280244E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_SET_ALLOWED_AREA_54(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SAFETY_ALLOWED_AREA_55(), &PH);
        p55_p2x_SET((float)1.0294272E38F, PH.base.pack) ;
        p55_p2y_SET((float)1.7684036E38F, PH.base.pack) ;
        p55_p1y_SET((float)1.8335525E36F, PH.base.pack) ;
        p55_p1z_SET((float) -1.3211999E38F, PH.base.pack) ;
        p55_frame_SET(e_MAV_FRAME_MAV_FRAME_MISSION, PH.base.pack) ;
        p55_p1x_SET((float)3.1880812E38F, PH.base.pack) ;
        p55_p2z_SET((float) -3.2079162E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SAFETY_ALLOWED_AREA_55(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_QUATERNION_COV_61(), &PH);
        p61_yawspeed_SET((float)2.2078362E38F, PH.base.pack) ;
        {
            float covariance[] =  {-7.9688436E37F, -1.995715E38F, -2.8615918E38F, 2.5187067E38F, 2.2813417E38F, -2.4655808E38F, 1.7365714E38F, -1.4099172E38F, 2.89197E37F};
            p61_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        {
            float q[] =  {2.5429765E38F, -3.0958062E38F, 1.4453513E38F, -2.0220005E38F};
            p61_q_SET(&q, 0, PH.base.pack) ;
        }
        p61_time_usec_SET((uint64_t)4557277817191516737L, PH.base.pack) ;
        p61_rollspeed_SET((float)1.5946163E38F, PH.base.pack) ;
        p61_pitchspeed_SET((float)1.4041678E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_QUATERNION_COV_61(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAV_CONTROLLER_OUTPUT_62(), &PH);
        p62_target_bearing_SET((int16_t)(int16_t)16954, PH.base.pack) ;
        p62_nav_pitch_SET((float) -3.177426E38F, PH.base.pack) ;
        p62_wp_dist_SET((uint16_t)(uint16_t)21072, PH.base.pack) ;
        p62_nav_bearing_SET((int16_t)(int16_t)23481, PH.base.pack) ;
        p62_nav_roll_SET((float)1.0784313E37F, PH.base.pack) ;
        p62_xtrack_error_SET((float) -1.1641062E38F, PH.base.pack) ;
        p62_aspd_error_SET((float)2.2851343E38F, PH.base.pack) ;
        p62_alt_error_SET((float)1.8946113E36F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAV_CONTROLLER_OUTPUT_62(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_POSITION_INT_COV_63(), &PH);
        p63_vx_SET((float) -3.2187066E38F, PH.base.pack) ;
        p63_vz_SET((float) -3.206567E38F, PH.base.pack) ;
        p63_alt_SET((int32_t)1784679152, PH.base.pack) ;
        p63_vy_SET((float) -1.290597E38F, PH.base.pack) ;
        p63_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_VISION, PH.base.pack) ;
        p63_lat_SET((int32_t)254193721, PH.base.pack) ;
        p63_time_usec_SET((uint64_t)1696269504734581763L, PH.base.pack) ;
        p63_relative_alt_SET((int32_t) -1114806170, PH.base.pack) ;
        p63_lon_SET((int32_t) -1598349347, PH.base.pack) ;
        {
            float covariance[] =  {-4.4139756E37F, 4.95664E37F, -3.3115695E38F, -2.3735017E38F, -4.910613E37F, -7.549652E37F, -3.7433793E37F, 3.1568327E38F, -2.5738007E38F, 2.3172371E38F, 2.015421E38F, 3.0249184E38F, 1.0963625E38F, -8.1080945E37F, -1.5863199E38F, 3.2128602E38F, 7.696945E37F, -2.2344772E38F, 2.2865078E38F, 2.6473856E38F, -9.359695E37F, 2.2067546E38F, -7.073953E37F, 9.535361E37F, -5.8196287E37F, -3.2803595E38F, 9.60527E37F, -3.085046E38F, -1.5248177E38F, -3.0635016E38F, -1.3656424E38F, 2.9009456E38F, -2.869833E37F, 1.887977E38F, -2.5728796E38F, -1.2924686E38F};
            p63_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_GLOBAL_POSITION_INT_COV_63(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_COV_64(), &PH);
        p64_ax_SET((float) -8.0941174E37F, PH.base.pack) ;
        p64_x_SET((float)2.235812E38F, PH.base.pack) ;
        p64_y_SET((float) -1.2412373E38F, PH.base.pack) ;
        p64_az_SET((float)3.2193117E38F, PH.base.pack) ;
        p64_time_usec_SET((uint64_t)6729142783798613563L, PH.base.pack) ;
        p64_estimator_type_SET(e_MAV_ESTIMATOR_TYPE_MAV_ESTIMATOR_TYPE_GPS, PH.base.pack) ;
        p64_vy_SET((float)1.2671179E38F, PH.base.pack) ;
        p64_z_SET((float)1.6203988E38F, PH.base.pack) ;
        p64_ay_SET((float)9.831133E37F, PH.base.pack) ;
        p64_vx_SET((float)1.9360306E38F, PH.base.pack) ;
        {
            float covariance[] =  {1.9100783E38F, 1.3856443E37F, -2.1717323E38F, 2.6390321E38F, -7.882638E37F, -1.3621049E37F, 2.5585536E38F, -2.2996652E38F, -1.1000763E38F, -2.5857519E38F, 2.7153737E38F, -7.108533E37F, -3.6102567E37F, 2.3102013E38F, -1.2287822E38F, -2.3875548E38F, 2.4230099E38F, -3.0173043E38F, 2.9549254E38F, -2.68777E38F, 2.4467168E38F, -3.2307873E38F, -3.2094611E38F, -1.2852244E38F, 4.4495124E37F, 2.4155863E38F, 1.452198E38F, 3.1149903E38F, 2.906713E38F, -3.1156223E38F, 7.8380114E37F, 2.9541378E37F, -1.4614942E38F, -6.571612E36F, -2.9302184E38F, -9.235883E37F, 1.0889615E38F, 2.2689374E38F, -1.1433816E38F, 3.3651008E38F, -3.3220924E38F, 1.3261823E38F, -1.2062161E38F, -1.5454036E38F, -1.3811962E38F};
            p64_covariance_SET(&covariance, 0, PH.base.pack) ;
        }
        p64_vz_SET((float) -3.2733917E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_COV_64(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_65(), &PH);
        p65_chan17_raw_SET((uint16_t)(uint16_t)11657, PH.base.pack) ;
        p65_chancount_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        p65_chan8_raw_SET((uint16_t)(uint16_t)25936, PH.base.pack) ;
        p65_chan14_raw_SET((uint16_t)(uint16_t)51829, PH.base.pack) ;
        p65_chan2_raw_SET((uint16_t)(uint16_t)61640, PH.base.pack) ;
        p65_chan4_raw_SET((uint16_t)(uint16_t)24233, PH.base.pack) ;
        p65_chan11_raw_SET((uint16_t)(uint16_t)54972, PH.base.pack) ;
        p65_chan18_raw_SET((uint16_t)(uint16_t)31671, PH.base.pack) ;
        p65_chan13_raw_SET((uint16_t)(uint16_t)42103, PH.base.pack) ;
        p65_chan16_raw_SET((uint16_t)(uint16_t)1935, PH.base.pack) ;
        p65_chan15_raw_SET((uint16_t)(uint16_t)19192, PH.base.pack) ;
        p65_chan3_raw_SET((uint16_t)(uint16_t)28695, PH.base.pack) ;
        p65_chan7_raw_SET((uint16_t)(uint16_t)32006, PH.base.pack) ;
        p65_chan9_raw_SET((uint16_t)(uint16_t)33767, PH.base.pack) ;
        p65_chan6_raw_SET((uint16_t)(uint16_t)6952, PH.base.pack) ;
        p65_chan5_raw_SET((uint16_t)(uint16_t)10968, PH.base.pack) ;
        p65_chan1_raw_SET((uint16_t)(uint16_t)30573, PH.base.pack) ;
        p65_time_boot_ms_SET((uint32_t)2076074457L, PH.base.pack) ;
        p65_chan12_raw_SET((uint16_t)(uint16_t)51595, PH.base.pack) ;
        p65_rssi_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        p65_chan10_raw_SET((uint16_t)(uint16_t)21759, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_65(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_REQUEST_DATA_STREAM_66(), &PH);
        p66_target_system_SET((uint8_t)(uint8_t)36, PH.base.pack) ;
        p66_start_stop_SET((uint8_t)(uint8_t)84, PH.base.pack) ;
        p66_req_stream_id_SET((uint8_t)(uint8_t)88, PH.base.pack) ;
        p66_req_message_rate_SET((uint16_t)(uint16_t)14964, PH.base.pack) ;
        p66_target_component_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        c_LoopBackDemoChannel_on_REQUEST_DATA_STREAM_66(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_STREAM_67(), &PH);
        p67_on_off_SET((uint8_t)(uint8_t)23, PH.base.pack) ;
        p67_message_rate_SET((uint16_t)(uint16_t)62593, PH.base.pack) ;
        p67_stream_id_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_STREAM_67(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_CONTROL_69(), &PH);
        p69_buttons_SET((uint16_t)(uint16_t)18541, PH.base.pack) ;
        p69_r_SET((int16_t)(int16_t)609, PH.base.pack) ;
        p69_target_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        p69_z_SET((int16_t)(int16_t) -27150, PH.base.pack) ;
        p69_y_SET((int16_t)(int16_t) -7474, PH.base.pack) ;
        p69_x_SET((int16_t)(int16_t)21695, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_CONTROL_69(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RC_CHANNELS_OVERRIDE_70(), &PH);
        p70_chan7_raw_SET((uint16_t)(uint16_t)21598, PH.base.pack) ;
        p70_chan5_raw_SET((uint16_t)(uint16_t)38145, PH.base.pack) ;
        p70_chan1_raw_SET((uint16_t)(uint16_t)6223, PH.base.pack) ;
        p70_chan8_raw_SET((uint16_t)(uint16_t)46873, PH.base.pack) ;
        p70_chan3_raw_SET((uint16_t)(uint16_t)42158, PH.base.pack) ;
        p70_chan2_raw_SET((uint16_t)(uint16_t)8266, PH.base.pack) ;
        p70_target_component_SET((uint8_t)(uint8_t)235, PH.base.pack) ;
        p70_chan6_raw_SET((uint16_t)(uint16_t)49064, PH.base.pack) ;
        p70_chan4_raw_SET((uint16_t)(uint16_t)14068, PH.base.pack) ;
        p70_target_system_SET((uint8_t)(uint8_t)135, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RC_CHANNELS_OVERRIDE_70(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MISSION_ITEM_INT_73(), &PH);
        p73_command_SET(e_MAV_CMD_MAV_CMD_DO_SET_CAM_TRIGG_DIST, PH.base.pack) ;
        p73_x_SET((int32_t)457104418, PH.base.pack) ;
        p73_target_component_SET((uint8_t)(uint8_t)123, PH.base.pack) ;
        p73_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_NED, PH.base.pack) ;
        p73_z_SET((float) -2.261985E37F, PH.base.pack) ;
        p73_seq_SET((uint16_t)(uint16_t)41133, PH.base.pack) ;
        p73_param3_SET((float)8.1348394E37F, PH.base.pack) ;
        p73_target_system_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        p73_mission_type_SET(e_MAV_MISSION_TYPE_MAV_MISSION_TYPE_MISSION, PH.base.pack) ;
        p73_y_SET((int32_t) -384970639, PH.base.pack) ;
        p73_param4_SET((float) -1.1746958E38F, PH.base.pack) ;
        p73_param1_SET((float) -1.0318666E38F, PH.base.pack) ;
        p73_current_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        p73_param2_SET((float)1.763342E38F, PH.base.pack) ;
        p73_autocontinue_SET((uint8_t)(uint8_t)248, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MISSION_ITEM_INT_73(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VFR_HUD_74(), &PH);
        p74_groundspeed_SET((float)2.9130227E38F, PH.base.pack) ;
        p74_heading_SET((int16_t)(int16_t)6428, PH.base.pack) ;
        p74_alt_SET((float) -1.8107079E38F, PH.base.pack) ;
        p74_climb_SET((float)2.7028922E37F, PH.base.pack) ;
        p74_throttle_SET((uint16_t)(uint16_t)25086, PH.base.pack) ;
        p74_airspeed_SET((float)1.488647E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VFR_HUD_74(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_INT_75(), &PH);
        p75_command_SET(e_MAV_CMD_MAV_CMD_NAV_RALLY_POINT, PH.base.pack) ;
        p75_y_SET((int32_t)1075102028, PH.base.pack) ;
        p75_x_SET((int32_t)1324108846, PH.base.pack) ;
        p75_param2_SET((float)2.1713186E38F, PH.base.pack) ;
        p75_param4_SET((float) -3.1203885E38F, PH.base.pack) ;
        p75_autocontinue_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        p75_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_INT, PH.base.pack) ;
        p75_z_SET((float)2.3221515E38F, PH.base.pack) ;
        p75_param3_SET((float) -2.6514046E38F, PH.base.pack) ;
        p75_param1_SET((float) -1.3446609E38F, PH.base.pack) ;
        p75_target_component_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        p75_target_system_SET((uint8_t)(uint8_t)125, PH.base.pack) ;
        p75_current_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_INT_75(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_LONG_76(), &PH);
        p76_param3_SET((float)5.1063887E37F, PH.base.pack) ;
        p76_param2_SET((float)1.1503348E38F, PH.base.pack) ;
        p76_target_component_SET((uint8_t)(uint8_t)213, PH.base.pack) ;
        p76_param5_SET((float) -2.283216E38F, PH.base.pack) ;
        p76_param7_SET((float) -2.280676E38F, PH.base.pack) ;
        p76_confirmation_SET((uint8_t)(uint8_t)229, PH.base.pack) ;
        p76_command_SET(e_MAV_CMD_MAV_CMD_NAV_GUIDED_ENABLE, PH.base.pack) ;
        p76_param6_SET((float) -9.864486E37F, PH.base.pack) ;
        p76_param4_SET((float) -2.2863172E38F, PH.base.pack) ;
        p76_param1_SET((float)5.073007E37F, PH.base.pack) ;
        p76_target_system_SET((uint8_t)(uint8_t)237, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COMMAND_LONG_76(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COMMAND_ACK_77(), &PH);
        p77_target_system_SET((uint8_t)(uint8_t)161, &PH) ;
        p77_result_SET(e_MAV_RESULT_MAV_RESULT_DENIED, PH.base.pack) ;
        p77_result_param2_SET((int32_t) -397415415, &PH) ;
        p77_command_SET(e_MAV_CMD_MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE, PH.base.pack) ;
        p77_progress_SET((uint8_t)(uint8_t)98, &PH) ;
        p77_target_component_SET((uint8_t)(uint8_t)120, &PH) ;
        c_LoopBackDemoChannel_on_COMMAND_ACK_77(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MANUAL_SETPOINT_81(), &PH);
        p81_yaw_SET((float)1.2125277E37F, PH.base.pack) ;
        p81_roll_SET((float)2.8737327E38F, PH.base.pack) ;
        p81_time_boot_ms_SET((uint32_t)2819108460L, PH.base.pack) ;
        p81_manual_override_switch_SET((uint8_t)(uint8_t)205, PH.base.pack) ;
        p81_pitch_SET((float)2.3060536E38F, PH.base.pack) ;
        p81_thrust_SET((float)4.745377E37F, PH.base.pack) ;
        p81_mode_switch_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MANUAL_SETPOINT_81(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ATTITUDE_TARGET_82(), &PH);
        p82_target_system_SET((uint8_t)(uint8_t)73, PH.base.pack) ;
        p82_body_yaw_rate_SET((float)6.308009E37F, PH.base.pack) ;
        p82_type_mask_SET((uint8_t)(uint8_t)160, PH.base.pack) ;
        {
            float q[] =  {-8.2529257E37F, -3.3775904E37F, -2.2504481E38F, 2.2121318E38F};
            p82_q_SET(&q, 0, PH.base.pack) ;
        }
        p82_body_roll_rate_SET((float) -8.2261245E37F, PH.base.pack) ;
        p82_thrust_SET((float) -2.8103286E38F, PH.base.pack) ;
        p82_time_boot_ms_SET((uint32_t)1088010633L, PH.base.pack) ;
        p82_target_component_SET((uint8_t)(uint8_t)210, PH.base.pack) ;
        p82_body_pitch_rate_SET((float) -2.4320187E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_ATTITUDE_TARGET_82(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATTITUDE_TARGET_83(), &PH);
        p83_body_roll_rate_SET((float)2.069942E38F, PH.base.pack) ;
        p83_thrust_SET((float)1.6516483E38F, PH.base.pack) ;
        p83_body_yaw_rate_SET((float)4.6206113E37F, PH.base.pack) ;
        p83_time_boot_ms_SET((uint32_t)1918543860L, PH.base.pack) ;
        {
            float q[] =  {2.38169E38F, 3.368664E37F, 2.1720453E38F, -1.8085595E38F};
            p83_q_SET(&q, 0, PH.base.pack) ;
        }
        p83_type_mask_SET((uint8_t)(uint8_t)66, PH.base.pack) ;
        p83_body_pitch_rate_SET((float)2.973246E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATTITUDE_TARGET_83(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_LOCAL_NED_84(), &PH);
        p84_vz_SET((float) -3.3074945E38F, PH.base.pack) ;
        p84_yaw_SET((float) -6.323792E37F, PH.base.pack) ;
        p84_y_SET((float) -1.2771864E38F, PH.base.pack) ;
        p84_afz_SET((float) -2.0274664E38F, PH.base.pack) ;
        p84_target_system_SET((uint8_t)(uint8_t)164, PH.base.pack) ;
        p84_z_SET((float)4.233198E37F, PH.base.pack) ;
        p84_yaw_rate_SET((float) -5.089831E37F, PH.base.pack) ;
        p84_x_SET((float) -1.7227414E38F, PH.base.pack) ;
        p84_afx_SET((float) -1.7586983E38F, PH.base.pack) ;
        p84_vx_SET((float) -1.1577264E38F, PH.base.pack) ;
        p84_target_component_SET((uint8_t)(uint8_t)239, PH.base.pack) ;
        p84_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_LOCAL_ENU, PH.base.pack) ;
        p84_vy_SET((float) -2.1639883E38F, PH.base.pack) ;
        p84_afy_SET((float)2.7902158E38F, PH.base.pack) ;
        p84_type_mask_SET((uint16_t)(uint16_t)6474, PH.base.pack) ;
        p84_time_boot_ms_SET((uint32_t)2658475703L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_LOCAL_NED_84(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_POSITION_TARGET_GLOBAL_INT_86(), &PH);
        p86_type_mask_SET((uint16_t)(uint16_t)18752, PH.base.pack) ;
        p86_time_boot_ms_SET((uint32_t)3627599026L, PH.base.pack) ;
        p86_lon_int_SET((int32_t)499560938, PH.base.pack) ;
        p86_afz_SET((float) -3.3710194E38F, PH.base.pack) ;
        p86_yaw_SET((float) -1.6896969E38F, PH.base.pack) ;
        p86_afy_SET((float) -2.7556426E38F, PH.base.pack) ;
        p86_vy_SET((float)1.405424E38F, PH.base.pack) ;
        p86_lat_int_SET((int32_t) -1181322210, PH.base.pack) ;
        p86_alt_SET((float) -7.3626257E37F, PH.base.pack) ;
        p86_target_system_SET((uint8_t)(uint8_t)144, PH.base.pack) ;
        p86_vz_SET((float)3.3494753E38F, PH.base.pack) ;
        p86_yaw_rate_SET((float) -8.7315355E36F, PH.base.pack) ;
        p86_vx_SET((float) -7.5745835E37F, PH.base.pack) ;
        p86_target_component_SET((uint8_t)(uint8_t)192, PH.base.pack) ;
        p86_afx_SET((float)1.3053582E38F, PH.base.pack) ;
        p86_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_NED, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_POSITION_TARGET_GLOBAL_INT_86(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POSITION_TARGET_GLOBAL_INT_87(), &PH);
        p87_type_mask_SET((uint16_t)(uint16_t)63169, PH.base.pack) ;
        p87_lat_int_SET((int32_t) -1211673698, PH.base.pack) ;
        p87_afz_SET((float) -2.9063581E38F, PH.base.pack) ;
        p87_afx_SET((float) -7.802306E37F, PH.base.pack) ;
        p87_yaw_rate_SET((float)1.411404E36F, PH.base.pack) ;
        p87_yaw_SET((float) -1.4687577E38F, PH.base.pack) ;
        p87_time_boot_ms_SET((uint32_t)200068824L, PH.base.pack) ;
        p87_alt_SET((float) -4.5382317E37F, PH.base.pack) ;
        p87_vx_SET((float) -2.9204505E37F, PH.base.pack) ;
        p87_lon_int_SET((int32_t)398218012, PH.base.pack) ;
        p87_afy_SET((float)1.0812786E38F, PH.base.pack) ;
        p87_vz_SET((float)2.795697E38F, PH.base.pack) ;
        p87_vy_SET((float) -1.3135944E38F, PH.base.pack) ;
        p87_coordinate_frame_SET(e_MAV_FRAME_MAV_FRAME_GLOBAL_TERRAIN_ALT, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POSITION_TARGET_GLOBAL_INT_87(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(), &PH);
        p89_time_boot_ms_SET((uint32_t)3742540708L, PH.base.pack) ;
        p89_yaw_SET((float) -3.1550854E38F, PH.base.pack) ;
        p89_z_SET((float) -2.6272358E37F, PH.base.pack) ;
        p89_roll_SET((float)3.4227474E37F, PH.base.pack) ;
        p89_pitch_SET((float) -2.9035711E38F, PH.base.pack) ;
        p89_x_SET((float) -2.5437294E38F, PH.base.pack) ;
        p89_y_SET((float)2.4648E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_89(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_90(), &PH);
        p90_xacc_SET((int16_t)(int16_t) -13543, PH.base.pack) ;
        p90_pitchspeed_SET((float)1.0225286E38F, PH.base.pack) ;
        p90_vy_SET((int16_t)(int16_t)15914, PH.base.pack) ;
        p90_yacc_SET((int16_t)(int16_t)2787, PH.base.pack) ;
        p90_yaw_SET((float)2.6184335E38F, PH.base.pack) ;
        p90_pitch_SET((float)2.47912E38F, PH.base.pack) ;
        p90_roll_SET((float)3.363573E38F, PH.base.pack) ;
        p90_yawspeed_SET((float)3.3615238E38F, PH.base.pack) ;
        p90_lat_SET((int32_t) -69594771, PH.base.pack) ;
        p90_alt_SET((int32_t) -400225197, PH.base.pack) ;
        p90_rollspeed_SET((float)2.118666E38F, PH.base.pack) ;
        p90_vz_SET((int16_t)(int16_t)4231, PH.base.pack) ;
        p90_vx_SET((int16_t)(int16_t)3773, PH.base.pack) ;
        p90_lon_SET((int32_t) -1030616578, PH.base.pack) ;
        p90_time_usec_SET((uint64_t)2574986204201894911L, PH.base.pack) ;
        p90_zacc_SET((int16_t)(int16_t) -5715, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_STATE_90(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_CONTROLS_91(), &PH);
        p91_aux4_SET((float)2.948039E38F, PH.base.pack) ;
        p91_pitch_elevator_SET((float)4.3908887E37F, PH.base.pack) ;
        p91_aux2_SET((float)2.8563236E37F, PH.base.pack) ;
        p91_roll_ailerons_SET((float)4.224699E36F, PH.base.pack) ;
        p91_mode_SET(e_MAV_MODE_MAV_MODE_MANUAL_DISARMED, PH.base.pack) ;
        p91_time_usec_SET((uint64_t)2734626562126311039L, PH.base.pack) ;
        p91_throttle_SET((float)2.2216806E37F, PH.base.pack) ;
        p91_nav_mode_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        p91_yaw_rudder_SET((float)1.5010709E38F, PH.base.pack) ;
        p91_aux1_SET((float)1.4066972E38F, PH.base.pack) ;
        p91_aux3_SET((float)8.0259467E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_CONTROLS_91(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_RC_INPUTS_RAW_92(), &PH);
        p92_rssi_SET((uint8_t)(uint8_t)75, PH.base.pack) ;
        p92_chan2_raw_SET((uint16_t)(uint16_t)2936, PH.base.pack) ;
        p92_chan6_raw_SET((uint16_t)(uint16_t)46379, PH.base.pack) ;
        p92_chan9_raw_SET((uint16_t)(uint16_t)6334, PH.base.pack) ;
        p92_chan3_raw_SET((uint16_t)(uint16_t)42452, PH.base.pack) ;
        p92_chan7_raw_SET((uint16_t)(uint16_t)4480, PH.base.pack) ;
        p92_chan1_raw_SET((uint16_t)(uint16_t)34224, PH.base.pack) ;
        p92_time_usec_SET((uint64_t)7415270434051196649L, PH.base.pack) ;
        p92_chan8_raw_SET((uint16_t)(uint16_t)28208, PH.base.pack) ;
        p92_chan5_raw_SET((uint16_t)(uint16_t)34055, PH.base.pack) ;
        p92_chan10_raw_SET((uint16_t)(uint16_t)11679, PH.base.pack) ;
        p92_chan11_raw_SET((uint16_t)(uint16_t)52924, PH.base.pack) ;
        p92_chan4_raw_SET((uint16_t)(uint16_t)58865, PH.base.pack) ;
        p92_chan12_raw_SET((uint16_t)(uint16_t)12451, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_RC_INPUTS_RAW_92(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_ACTUATOR_CONTROLS_93(), &PH);
        p93_time_usec_SET((uint64_t)1547823720855461510L, PH.base.pack) ;
        {
            float controls[] =  {1.9394939E38F, -2.9320075E38F, 1.063194E38F, 2.8619427E38F, 2.3207466E38F, 7.906272E37F, 8.188366E37F, 1.286874E38F, 6.6889293E37F, 4.7776833E37F, -2.6134268E38F, 3.1612354E38F, 9.474035E37F, -2.9014766E38F, -3.8819515E36F, -2.657319E38F};
            p93_controls_SET(&controls, 0, PH.base.pack) ;
        }
        p93_mode_SET(e_MAV_MODE_MAV_MODE_PREFLIGHT, PH.base.pack) ;
        p93_flags_SET((uint64_t)4607800801507066774L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_ACTUATOR_CONTROLS_93(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_100(), &PH);
        p100_quality_SET((uint8_t)(uint8_t)10, PH.base.pack) ;
        p100_sensor_id_SET((uint8_t)(uint8_t)222, PH.base.pack) ;
        p100_flow_rate_y_SET((float)2.181095E38F, &PH) ;
        p100_flow_rate_x_SET((float)5.466814E37F, &PH) ;
        p100_flow_comp_m_x_SET((float)8.530277E37F, PH.base.pack) ;
        p100_flow_y_SET((int16_t)(int16_t)9536, PH.base.pack) ;
        p100_flow_x_SET((int16_t)(int16_t)32252, PH.base.pack) ;
        p100_ground_distance_SET((float)9.116103E37F, PH.base.pack) ;
        p100_time_usec_SET((uint64_t)2243135110625647390L, PH.base.pack) ;
        p100_flow_comp_m_y_SET((float) -1.2198762E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_100(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GLOBAL_VISION_POSITION_ESTIMATE_101(), &PH);
        p101_y_SET((float)1.729738E38F, PH.base.pack) ;
        p101_z_SET((float) -1.334857E38F, PH.base.pack) ;
        p101_x_SET((float)1.0047654E38F, PH.base.pack) ;
        p101_yaw_SET((float)1.1315915E38F, PH.base.pack) ;
        p101_roll_SET((float) -5.2296895E37F, PH.base.pack) ;
        p101_usec_SET((uint64_t)4656021596355750028L, PH.base.pack) ;
        p101_pitch_SET((float) -1.6403878E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GLOBAL_VISION_POSITION_ESTIMATE_101(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_POSITION_ESTIMATE_102(), &PH);
        p102_y_SET((float) -8.253037E37F, PH.base.pack) ;
        p102_z_SET((float)5.640685E37F, PH.base.pack) ;
        p102_yaw_SET((float) -2.2576754E38F, PH.base.pack) ;
        p102_pitch_SET((float)1.0061344E38F, PH.base.pack) ;
        p102_usec_SET((uint64_t)2835164604771048530L, PH.base.pack) ;
        p102_x_SET((float)1.9799307E37F, PH.base.pack) ;
        p102_roll_SET((float)1.3800967E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_POSITION_ESTIMATE_102(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VISION_SPEED_ESTIMATE_103(), &PH);
        p103_z_SET((float)1.6068335E38F, PH.base.pack) ;
        p103_y_SET((float)2.5352097E38F, PH.base.pack) ;
        p103_usec_SET((uint64_t)2009143334684140421L, PH.base.pack) ;
        p103_x_SET((float) -4.1911642E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VISION_SPEED_ESTIMATE_103(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VICON_POSITION_ESTIMATE_104(), &PH);
        p104_y_SET((float) -1.1264268E38F, PH.base.pack) ;
        p104_yaw_SET((float) -2.9904375E37F, PH.base.pack) ;
        p104_z_SET((float)2.3344788E38F, PH.base.pack) ;
        p104_roll_SET((float) -5.2660767E36F, PH.base.pack) ;
        p104_usec_SET((uint64_t)6937791749803696301L, PH.base.pack) ;
        p104_x_SET((float)2.2688287E38F, PH.base.pack) ;
        p104_pitch_SET((float) -3.2640796E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VICON_POSITION_ESTIMATE_104(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGHRES_IMU_105(), &PH);
        p105_pressure_alt_SET((float)1.0832265E38F, PH.base.pack) ;
        p105_zacc_SET((float) -2.6983282E38F, PH.base.pack) ;
        p105_ymag_SET((float) -9.302353E37F, PH.base.pack) ;
        p105_temperature_SET((float)1.215084E38F, PH.base.pack) ;
        p105_abs_pressure_SET((float) -1.0061351E38F, PH.base.pack) ;
        p105_fields_updated_SET((uint16_t)(uint16_t)7520, PH.base.pack) ;
        p105_xgyro_SET((float)1.0116622E38F, PH.base.pack) ;
        p105_time_usec_SET((uint64_t)2161745166960261516L, PH.base.pack) ;
        p105_yacc_SET((float) -3.3677566E38F, PH.base.pack) ;
        p105_xmag_SET((float)2.5524727E38F, PH.base.pack) ;
        p105_zmag_SET((float)2.0084388E38F, PH.base.pack) ;
        p105_xacc_SET((float) -2.7348734E38F, PH.base.pack) ;
        p105_diff_pressure_SET((float) -3.2082434E38F, PH.base.pack) ;
        p105_ygyro_SET((float) -1.013746E38F, PH.base.pack) ;
        p105_zgyro_SET((float) -4.5612305E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGHRES_IMU_105(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OPTICAL_FLOW_RAD_106(), &PH);
        p106_quality_SET((uint8_t)(uint8_t)90, PH.base.pack) ;
        p106_temperature_SET((int16_t)(int16_t) -927, PH.base.pack) ;
        p106_time_delta_distance_us_SET((uint32_t)1583334171L, PH.base.pack) ;
        p106_integrated_ygyro_SET((float)7.015535E37F, PH.base.pack) ;
        p106_integrated_x_SET((float)1.0003097E38F, PH.base.pack) ;
        p106_distance_SET((float) -1.6147104E37F, PH.base.pack) ;
        p106_integration_time_us_SET((uint32_t)2706554044L, PH.base.pack) ;
        p106_integrated_xgyro_SET((float) -3.2075059E38F, PH.base.pack) ;
        p106_sensor_id_SET((uint8_t)(uint8_t)116, PH.base.pack) ;
        p106_integrated_y_SET((float)5.684749E37F, PH.base.pack) ;
        p106_time_usec_SET((uint64_t)2299149947706698470L, PH.base.pack) ;
        p106_integrated_zgyro_SET((float) -1.3117128E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OPTICAL_FLOW_RAD_106(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_SENSOR_107(), &PH);
        p107_temperature_SET((float) -1.5614414E38F, PH.base.pack) ;
        p107_xacc_SET((float) -1.9102101E38F, PH.base.pack) ;
        p107_diff_pressure_SET((float)6.147163E36F, PH.base.pack) ;
        p107_time_usec_SET((uint64_t)386675838609781693L, PH.base.pack) ;
        p107_zmag_SET((float) -1.3445148E38F, PH.base.pack) ;
        p107_zgyro_SET((float)3.8278175E37F, PH.base.pack) ;
        p107_xmag_SET((float)1.142581E38F, PH.base.pack) ;
        p107_yacc_SET((float)1.8484615E38F, PH.base.pack) ;
        p107_ymag_SET((float)2.5613626E37F, PH.base.pack) ;
        p107_abs_pressure_SET((float)2.1201649E38F, PH.base.pack) ;
        p107_pressure_alt_SET((float) -1.0833513E38F, PH.base.pack) ;
        p107_zacc_SET((float)5.607017E37F, PH.base.pack) ;
        p107_fields_updated_SET((uint32_t)924085616L, PH.base.pack) ;
        p107_ygyro_SET((float)8.052983E37F, PH.base.pack) ;
        p107_xgyro_SET((float)8.267211E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_SENSOR_107(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SIM_STATE_108(), &PH);
        p108_yaw_SET((float)1.3542695E38F, PH.base.pack) ;
        p108_q4_SET((float) -6.422069E37F, PH.base.pack) ;
        p108_lat_SET((float)1.800846E38F, PH.base.pack) ;
        p108_vn_SET((float)2.5671137E37F, PH.base.pack) ;
        p108_vd_SET((float) -5.603438E37F, PH.base.pack) ;
        p108_roll_SET((float)4.6358363E37F, PH.base.pack) ;
        p108_xgyro_SET((float) -2.9972438E38F, PH.base.pack) ;
        p108_q2_SET((float)1.901965E38F, PH.base.pack) ;
        p108_q1_SET((float)1.06766E38F, PH.base.pack) ;
        p108_std_dev_vert_SET((float) -8.0053383E37F, PH.base.pack) ;
        p108_pitch_SET((float) -2.0083405E38F, PH.base.pack) ;
        p108_zacc_SET((float) -2.735521E38F, PH.base.pack) ;
        p108_yacc_SET((float)1.6103025E38F, PH.base.pack) ;
        p108_ygyro_SET((float) -2.3967841E38F, PH.base.pack) ;
        p108_ve_SET((float)1.6569638E38F, PH.base.pack) ;
        p108_alt_SET((float) -2.1835355E38F, PH.base.pack) ;
        p108_q3_SET((float)1.6591408E38F, PH.base.pack) ;
        p108_lon_SET((float) -2.8209361E38F, PH.base.pack) ;
        p108_std_dev_horz_SET((float) -3.3226917E38F, PH.base.pack) ;
        p108_xacc_SET((float) -1.0856249E38F, PH.base.pack) ;
        p108_zgyro_SET((float) -2.281411E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SIM_STATE_108(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RADIO_STATUS_109(), &PH);
        p109_remnoise_SET((uint8_t)(uint8_t)97, PH.base.pack) ;
        p109_noise_SET((uint8_t)(uint8_t)241, PH.base.pack) ;
        p109_rssi_SET((uint8_t)(uint8_t)1, PH.base.pack) ;
        p109_remrssi_SET((uint8_t)(uint8_t)139, PH.base.pack) ;
        p109_rxerrors_SET((uint16_t)(uint16_t)59158, PH.base.pack) ;
        p109_txbuf_SET((uint8_t)(uint8_t)178, PH.base.pack) ;
        p109_fixed__SET((uint16_t)(uint16_t)54647, PH.base.pack) ;
        c_LoopBackDemoChannel_on_RADIO_STATUS_109(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FILE_TRANSFER_PROTOCOL_110(), &PH);
        p110_target_system_SET((uint8_t)(uint8_t)78, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)231, (uint8_t)138, (uint8_t)221, (uint8_t)199, (uint8_t)112, (uint8_t)173, (uint8_t)30, (uint8_t)21, (uint8_t)123, (uint8_t)193, (uint8_t)253, (uint8_t)114, (uint8_t)212, (uint8_t)149, (uint8_t)142, (uint8_t)176, (uint8_t)252, (uint8_t)24, (uint8_t)82, (uint8_t)102, (uint8_t)117, (uint8_t)156, (uint8_t)4, (uint8_t)201, (uint8_t)37, (uint8_t)27, (uint8_t)120, (uint8_t)74, (uint8_t)1, (uint8_t)119, (uint8_t)79, (uint8_t)237, (uint8_t)53, (uint8_t)129, (uint8_t)46, (uint8_t)18, (uint8_t)235, (uint8_t)9, (uint8_t)101, (uint8_t)44, (uint8_t)87, (uint8_t)12, (uint8_t)198, (uint8_t)181, (uint8_t)15, (uint8_t)106, (uint8_t)25, (uint8_t)203, (uint8_t)152, (uint8_t)198, (uint8_t)172, (uint8_t)142, (uint8_t)214, (uint8_t)70, (uint8_t)25, (uint8_t)111, (uint8_t)114, (uint8_t)137, (uint8_t)136, (uint8_t)213, (uint8_t)2, (uint8_t)157, (uint8_t)82, (uint8_t)200, (uint8_t)203, (uint8_t)97, (uint8_t)59, (uint8_t)13, (uint8_t)7, (uint8_t)152, (uint8_t)69, (uint8_t)246, (uint8_t)52, (uint8_t)202, (uint8_t)6, (uint8_t)101, (uint8_t)230, (uint8_t)129, (uint8_t)79, (uint8_t)247, (uint8_t)161, (uint8_t)139, (uint8_t)5, (uint8_t)201, (uint8_t)222, (uint8_t)111, (uint8_t)154, (uint8_t)65, (uint8_t)179, (uint8_t)167, (uint8_t)255, (uint8_t)122, (uint8_t)183, (uint8_t)201, (uint8_t)190, (uint8_t)173, (uint8_t)195, (uint8_t)179, (uint8_t)86, (uint8_t)183, (uint8_t)163, (uint8_t)106, (uint8_t)13, (uint8_t)97, (uint8_t)144, (uint8_t)115, (uint8_t)186, (uint8_t)49, (uint8_t)119, (uint8_t)168, (uint8_t)237, (uint8_t)43, (uint8_t)135, (uint8_t)190, (uint8_t)155, (uint8_t)54, (uint8_t)24, (uint8_t)186, (uint8_t)184, (uint8_t)34, (uint8_t)56, (uint8_t)19, (uint8_t)211, (uint8_t)186, (uint8_t)175, (uint8_t)71, (uint8_t)95, (uint8_t)184, (uint8_t)92, (uint8_t)50, (uint8_t)206, (uint8_t)152, (uint8_t)251, (uint8_t)34, (uint8_t)134, (uint8_t)69, (uint8_t)217, (uint8_t)249, (uint8_t)70, (uint8_t)237, (uint8_t)104, (uint8_t)79, (uint8_t)183, (uint8_t)14, (uint8_t)37, (uint8_t)56, (uint8_t)253, (uint8_t)58, (uint8_t)85, (uint8_t)157, (uint8_t)69, (uint8_t)24, (uint8_t)128, (uint8_t)188, (uint8_t)128, (uint8_t)153, (uint8_t)79, (uint8_t)103, (uint8_t)135, (uint8_t)151, (uint8_t)16, (uint8_t)102, (uint8_t)82, (uint8_t)123, (uint8_t)230, (uint8_t)162, (uint8_t)124, (uint8_t)98, (uint8_t)128, (uint8_t)32, (uint8_t)188, (uint8_t)236, (uint8_t)187, (uint8_t)37, (uint8_t)214, (uint8_t)245, (uint8_t)102, (uint8_t)213, (uint8_t)240, (uint8_t)85, (uint8_t)106, (uint8_t)50, (uint8_t)12, (uint8_t)138, (uint8_t)4, (uint8_t)217, (uint8_t)254, (uint8_t)27, (uint8_t)222, (uint8_t)210, (uint8_t)92, (uint8_t)65, (uint8_t)92, (uint8_t)115, (uint8_t)189, (uint8_t)27, (uint8_t)229, (uint8_t)207, (uint8_t)84, (uint8_t)163, (uint8_t)45, (uint8_t)195, (uint8_t)161, (uint8_t)15, (uint8_t)50, (uint8_t)48, (uint8_t)18, (uint8_t)86, (uint8_t)160, (uint8_t)122, (uint8_t)124, (uint8_t)71, (uint8_t)64, (uint8_t)77, (uint8_t)121, (uint8_t)49, (uint8_t)21, (uint8_t)247, (uint8_t)92, (uint8_t)122, (uint8_t)152, (uint8_t)154, (uint8_t)64, (uint8_t)98, (uint8_t)102, (uint8_t)170, (uint8_t)221, (uint8_t)102, (uint8_t)95, (uint8_t)155, (uint8_t)207, (uint8_t)121, (uint8_t)5, (uint8_t)126, (uint8_t)31, (uint8_t)188, (uint8_t)201, (uint8_t)202, (uint8_t)207, (uint8_t)52, (uint8_t)201, (uint8_t)58, (uint8_t)177, (uint8_t)63, (uint8_t)132, (uint8_t)13, (uint8_t)36, (uint8_t)21, (uint8_t)84, (uint8_t)175, (uint8_t)241};
            p110_payload_SET(&payload, 0, PH.base.pack) ;
        }
        p110_target_network_SET((uint8_t)(uint8_t)33, PH.base.pack) ;
        p110_target_component_SET((uint8_t)(uint8_t)189, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FILE_TRANSFER_PROTOCOL_110(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TIMESYNC_111(), &PH);
        p111_tc1_SET((int64_t)277987972536978318L, PH.base.pack) ;
        p111_ts1_SET((int64_t) -9189301914699662759L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TIMESYNC_111(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_TRIGGER_112(), &PH);
        p112_seq_SET((uint32_t)1824200230L, PH.base.pack) ;
        p112_time_usec_SET((uint64_t)8448793866722366849L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_TRIGGER_112(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_GPS_113(), &PH);
        p113_lon_SET((int32_t)507422942, PH.base.pack) ;
        p113_satellites_visible_SET((uint8_t)(uint8_t)4, PH.base.pack) ;
        p113_time_usec_SET((uint64_t)5456135387074893030L, PH.base.pack) ;
        p113_vel_SET((uint16_t)(uint16_t)55492, PH.base.pack) ;
        p113_vd_SET((int16_t)(int16_t)17629, PH.base.pack) ;
        p113_epv_SET((uint16_t)(uint16_t)2243, PH.base.pack) ;
        p113_cog_SET((uint16_t)(uint16_t)679, PH.base.pack) ;
        p113_lat_SET((int32_t) -1828498453, PH.base.pack) ;
        p113_fix_type_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        p113_vn_SET((int16_t)(int16_t) -24298, PH.base.pack) ;
        p113_ve_SET((int16_t)(int16_t) -11817, PH.base.pack) ;
        p113_eph_SET((uint16_t)(uint16_t)27066, PH.base.pack) ;
        p113_alt_SET((int32_t) -1928744369, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_GPS_113(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_OPTICAL_FLOW_114(), &PH);
        p114_integrated_x_SET((float) -2.0288184E38F, PH.base.pack) ;
        p114_temperature_SET((int16_t)(int16_t)9923, PH.base.pack) ;
        p114_time_usec_SET((uint64_t)5480081855421120786L, PH.base.pack) ;
        p114_integrated_y_SET((float) -1.1921217E38F, PH.base.pack) ;
        p114_integrated_zgyro_SET((float) -2.2725984E38F, PH.base.pack) ;
        p114_sensor_id_SET((uint8_t)(uint8_t)156, PH.base.pack) ;
        p114_time_delta_distance_us_SET((uint32_t)2514159745L, PH.base.pack) ;
        p114_integrated_ygyro_SET((float) -1.6357859E38F, PH.base.pack) ;
        p114_quality_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
        p114_integrated_xgyro_SET((float) -1.1588639E38F, PH.base.pack) ;
        p114_distance_SET((float)2.4546673E38F, PH.base.pack) ;
        p114_integration_time_us_SET((uint32_t)1414189521L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_OPTICAL_FLOW_114(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIL_STATE_QUATERNION_115(), &PH);
        p115_alt_SET((int32_t) -1818877875, PH.base.pack) ;
        p115_lat_SET((int32_t) -805924563, PH.base.pack) ;
        p115_xacc_SET((int16_t)(int16_t)10069, PH.base.pack) ;
        p115_true_airspeed_SET((uint16_t)(uint16_t)4484, PH.base.pack) ;
        p115_vz_SET((int16_t)(int16_t) -17079, PH.base.pack) ;
        p115_yacc_SET((int16_t)(int16_t)17160, PH.base.pack) ;
        p115_time_usec_SET((uint64_t)7229022435494494892L, PH.base.pack) ;
        p115_ind_airspeed_SET((uint16_t)(uint16_t)16105, PH.base.pack) ;
        p115_zacc_SET((int16_t)(int16_t)28063, PH.base.pack) ;
        p115_rollspeed_SET((float) -1.6897258E38F, PH.base.pack) ;
        p115_lon_SET((int32_t) -1932020664, PH.base.pack) ;
        p115_vx_SET((int16_t)(int16_t)6689, PH.base.pack) ;
        {
            float attitude_quaternion[] =  {-2.9684856E38F, -7.8258055E37F, 2.4524157E38F, 3.1416625E38F};
            p115_attitude_quaternion_SET(&attitude_quaternion, 0, PH.base.pack) ;
        }
        p115_vy_SET((int16_t)(int16_t)30964, PH.base.pack) ;
        p115_yawspeed_SET((float)1.6874685E37F, PH.base.pack) ;
        p115_pitchspeed_SET((float) -2.8958778E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIL_STATE_QUATERNION_115(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU2_116(), &PH);
        p116_ymag_SET((int16_t)(int16_t) -4712, PH.base.pack) ;
        p116_xacc_SET((int16_t)(int16_t)26120, PH.base.pack) ;
        p116_zmag_SET((int16_t)(int16_t) -27671, PH.base.pack) ;
        p116_zgyro_SET((int16_t)(int16_t) -908, PH.base.pack) ;
        p116_time_boot_ms_SET((uint32_t)4268121612L, PH.base.pack) ;
        p116_ygyro_SET((int16_t)(int16_t)31475, PH.base.pack) ;
        p116_xmag_SET((int16_t)(int16_t)31592, PH.base.pack) ;
        p116_xgyro_SET((int16_t)(int16_t)11080, PH.base.pack) ;
        p116_zacc_SET((int16_t)(int16_t) -6844, PH.base.pack) ;
        p116_yacc_SET((int16_t)(int16_t) -22272, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU2_116(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_LIST_117(), &PH);
        p117_end_SET((uint16_t)(uint16_t)850, PH.base.pack) ;
        p117_start_SET((uint16_t)(uint16_t)26088, PH.base.pack) ;
        p117_target_system_SET((uint8_t)(uint8_t)247, PH.base.pack) ;
        p117_target_component_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_LIST_117(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ENTRY_118(), &PH);
        p118_size_SET((uint32_t)2422541542L, PH.base.pack) ;
        p118_id_SET((uint16_t)(uint16_t)14188, PH.base.pack) ;
        p118_time_utc_SET((uint32_t)576976743L, PH.base.pack) ;
        p118_last_log_num_SET((uint16_t)(uint16_t)53828, PH.base.pack) ;
        p118_num_logs_SET((uint16_t)(uint16_t)61900, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ENTRY_118(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_DATA_119(), &PH);
        p119_count_SET((uint32_t)2120934409L, PH.base.pack) ;
        p119_target_component_SET((uint8_t)(uint8_t)132, PH.base.pack) ;
        p119_target_system_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        p119_id_SET((uint16_t)(uint16_t)60268, PH.base.pack) ;
        p119_ofs_SET((uint32_t)2285789909L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_DATA_119(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_DATA_120(), &PH);
        p120_ofs_SET((uint32_t)431962935L, PH.base.pack) ;
        p120_count_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p120_id_SET((uint16_t)(uint16_t)40940, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)74, (uint8_t)7, (uint8_t)176, (uint8_t)149, (uint8_t)204, (uint8_t)171, (uint8_t)182, (uint8_t)228, (uint8_t)105, (uint8_t)1, (uint8_t)112, (uint8_t)149, (uint8_t)4, (uint8_t)151, (uint8_t)29, (uint8_t)0, (uint8_t)238, (uint8_t)128, (uint8_t)16, (uint8_t)144, (uint8_t)171, (uint8_t)164, (uint8_t)106, (uint8_t)123, (uint8_t)161, (uint8_t)64, (uint8_t)167, (uint8_t)64, (uint8_t)90, (uint8_t)160, (uint8_t)30, (uint8_t)191, (uint8_t)167, (uint8_t)66, (uint8_t)153, (uint8_t)71, (uint8_t)108, (uint8_t)243, (uint8_t)167, (uint8_t)33, (uint8_t)192, (uint8_t)254, (uint8_t)217, (uint8_t)110, (uint8_t)154, (uint8_t)250, (uint8_t)184, (uint8_t)54, (uint8_t)38, (uint8_t)106, (uint8_t)96, (uint8_t)136, (uint8_t)139, (uint8_t)38, (uint8_t)151, (uint8_t)109, (uint8_t)192, (uint8_t)18, (uint8_t)175, (uint8_t)147, (uint8_t)106, (uint8_t)192, (uint8_t)253, (uint8_t)214, (uint8_t)108, (uint8_t)45, (uint8_t)135, (uint8_t)121, (uint8_t)62, (uint8_t)1, (uint8_t)26, (uint8_t)0, (uint8_t)18, (uint8_t)56, (uint8_t)69, (uint8_t)188, (uint8_t)54, (uint8_t)132, (uint8_t)203, (uint8_t)143, (uint8_t)212, (uint8_t)21, (uint8_t)32, (uint8_t)180, (uint8_t)233, (uint8_t)122, (uint8_t)91, (uint8_t)239, (uint8_t)89, (uint8_t)242};
            p120_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_LOG_DATA_120(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_ERASE_121(), &PH);
        p121_target_component_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        p121_target_system_SET((uint8_t)(uint8_t)159, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_ERASE_121(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOG_REQUEST_END_122(), &PH);
        p122_target_component_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        p122_target_system_SET((uint8_t)(uint8_t)131, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOG_REQUEST_END_122(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INJECT_DATA_123(), &PH);
        p123_target_component_SET((uint8_t)(uint8_t)175, PH.base.pack) ;
        p123_len_SET((uint8_t)(uint8_t)49, PH.base.pack) ;
        p123_target_system_SET((uint8_t)(uint8_t)143, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)144, (uint8_t)55, (uint8_t)15, (uint8_t)175, (uint8_t)173, (uint8_t)248, (uint8_t)82, (uint8_t)206, (uint8_t)220, (uint8_t)10, (uint8_t)156, (uint8_t)195, (uint8_t)35, (uint8_t)31, (uint8_t)83, (uint8_t)157, (uint8_t)197, (uint8_t)10, (uint8_t)96, (uint8_t)146, (uint8_t)158, (uint8_t)149, (uint8_t)79, (uint8_t)143, (uint8_t)22, (uint8_t)69, (uint8_t)108, (uint8_t)128, (uint8_t)255, (uint8_t)55, (uint8_t)131, (uint8_t)128, (uint8_t)58, (uint8_t)100, (uint8_t)156, (uint8_t)196, (uint8_t)140, (uint8_t)138, (uint8_t)60, (uint8_t)164, (uint8_t)206, (uint8_t)190, (uint8_t)15, (uint8_t)125, (uint8_t)80, (uint8_t)17, (uint8_t)22, (uint8_t)187, (uint8_t)249, (uint8_t)72, (uint8_t)14, (uint8_t)248, (uint8_t)142, (uint8_t)120, (uint8_t)52, (uint8_t)138, (uint8_t)119, (uint8_t)231, (uint8_t)33, (uint8_t)57, (uint8_t)84, (uint8_t)30, (uint8_t)140, (uint8_t)132, (uint8_t)217, (uint8_t)72, (uint8_t)119, (uint8_t)164, (uint8_t)125, (uint8_t)73, (uint8_t)113, (uint8_t)212, (uint8_t)66, (uint8_t)203, (uint8_t)126, (uint8_t)50, (uint8_t)172, (uint8_t)101, (uint8_t)14, (uint8_t)114, (uint8_t)106, (uint8_t)159, (uint8_t)175, (uint8_t)127, (uint8_t)114, (uint8_t)26, (uint8_t)177, (uint8_t)62, (uint8_t)16, (uint8_t)216, (uint8_t)232, (uint8_t)142, (uint8_t)128, (uint8_t)119, (uint8_t)68, (uint8_t)248, (uint8_t)202, (uint8_t)213, (uint8_t)244, (uint8_t)110, (uint8_t)2, (uint8_t)235, (uint8_t)147, (uint8_t)69, (uint8_t)70, (uint8_t)150, (uint8_t)10, (uint8_t)162, (uint8_t)96, (uint8_t)201};
            p123_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_GPS_INJECT_DATA_123(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RAW_124(), &PH);
        p124_lat_SET((int32_t) -1794078652, PH.base.pack) ;
        p124_satellites_visible_SET((uint8_t)(uint8_t)149, PH.base.pack) ;
        p124_eph_SET((uint16_t)(uint16_t)20056, PH.base.pack) ;
        p124_alt_SET((int32_t)2129727943, PH.base.pack) ;
        p124_time_usec_SET((uint64_t)3983800200700004307L, PH.base.pack) ;
        p124_cog_SET((uint16_t)(uint16_t)55507, PH.base.pack) ;
        p124_dgps_numch_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        p124_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_DGPS, PH.base.pack) ;
        p124_vel_SET((uint16_t)(uint16_t)52693, PH.base.pack) ;
        p124_epv_SET((uint16_t)(uint16_t)34000, PH.base.pack) ;
        p124_lon_SET((int32_t)3282442, PH.base.pack) ;
        p124_dgps_age_SET((uint32_t)2298113808L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RAW_124(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_POWER_STATUS_125(), &PH);
        p125_Vcc_SET((uint16_t)(uint16_t)43509, PH.base.pack) ;
        p125_Vservo_SET((uint16_t)(uint16_t)24982, PH.base.pack) ;
        p125_flags_SET(e_MAV_POWER_STATUS_MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT, PH.base.pack) ;
        c_LoopBackDemoChannel_on_POWER_STATUS_125(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SERIAL_CONTROL_126(), &PH);
        p126_baudrate_SET((uint32_t)1196457077L, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)26, (uint8_t)178, (uint8_t)187, (uint8_t)4, (uint8_t)82, (uint8_t)215, (uint8_t)198, (uint8_t)126, (uint8_t)71, (uint8_t)83, (uint8_t)164, (uint8_t)174, (uint8_t)112, (uint8_t)194, (uint8_t)200, (uint8_t)169, (uint8_t)219, (uint8_t)114, (uint8_t)189, (uint8_t)6, (uint8_t)235, (uint8_t)63, (uint8_t)126, (uint8_t)219, (uint8_t)225, (uint8_t)29, (uint8_t)212, (uint8_t)206, (uint8_t)135, (uint8_t)12, (uint8_t)24, (uint8_t)237, (uint8_t)9, (uint8_t)45, (uint8_t)199, (uint8_t)31, (uint8_t)116, (uint8_t)230, (uint8_t)185, (uint8_t)69, (uint8_t)87, (uint8_t)73, (uint8_t)31, (uint8_t)115, (uint8_t)202, (uint8_t)63, (uint8_t)82, (uint8_t)237, (uint8_t)164, (uint8_t)236, (uint8_t)102, (uint8_t)9, (uint8_t)215, (uint8_t)29, (uint8_t)95, (uint8_t)154, (uint8_t)54, (uint8_t)205, (uint8_t)23, (uint8_t)34, (uint8_t)51, (uint8_t)52, (uint8_t)108, (uint8_t)141, (uint8_t)39, (uint8_t)127, (uint8_t)206, (uint8_t)153, (uint8_t)18, (uint8_t)60};
            p126_data__SET(&data_, 0, PH.base.pack) ;
        }
        p126_flags_SET(e_SERIAL_CONTROL_FLAG_SERIAL_CONTROL_FLAG_EXCLUSIVE, PH.base.pack) ;
        p126_count_SET((uint8_t)(uint8_t)30, PH.base.pack) ;
        p126_timeout_SET((uint16_t)(uint16_t)39841, PH.base.pack) ;
        p126_device_SET(e_SERIAL_CONTROL_DEV_SERIAL_CONTROL_DEV_SHELL, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SERIAL_CONTROL_126(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTK_127(), &PH);
        p127_tow_SET((uint32_t)2993390319L, PH.base.pack) ;
        p127_time_last_baseline_ms_SET((uint32_t)3405183735L, PH.base.pack) ;
        p127_baseline_b_mm_SET((int32_t)751323678, PH.base.pack) ;
        p127_nsats_SET((uint8_t)(uint8_t)246, PH.base.pack) ;
        p127_baseline_a_mm_SET((int32_t)436377954, PH.base.pack) ;
        p127_rtk_health_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p127_iar_num_hypotheses_SET((int32_t)955910123, PH.base.pack) ;
        p127_wn_SET((uint16_t)(uint16_t)39540, PH.base.pack) ;
        p127_baseline_c_mm_SET((int32_t) -2066024894, PH.base.pack) ;
        p127_accuracy_SET((uint32_t)4005293557L, PH.base.pack) ;
        p127_rtk_rate_SET((uint8_t)(uint8_t)45, PH.base.pack) ;
        p127_rtk_receiver_id_SET((uint8_t)(uint8_t)108, PH.base.pack) ;
        p127_baseline_coords_type_SET((uint8_t)(uint8_t)68, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RTK_127(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS2_RTK_128(), &PH);
        p128_rtk_health_SET((uint8_t)(uint8_t)151, PH.base.pack) ;
        p128_tow_SET((uint32_t)1550556623L, PH.base.pack) ;
        p128_accuracy_SET((uint32_t)3542689821L, PH.base.pack) ;
        p128_rtk_rate_SET((uint8_t)(uint8_t)51, PH.base.pack) ;
        p128_wn_SET((uint16_t)(uint16_t)44824, PH.base.pack) ;
        p128_rtk_receiver_id_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p128_iar_num_hypotheses_SET((int32_t) -1552447843, PH.base.pack) ;
        p128_nsats_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p128_baseline_a_mm_SET((int32_t) -1782657947, PH.base.pack) ;
        p128_baseline_coords_type_SET((uint8_t)(uint8_t)111, PH.base.pack) ;
        p128_baseline_b_mm_SET((int32_t) -1280685405, PH.base.pack) ;
        p128_time_last_baseline_ms_SET((uint32_t)270334891L, PH.base.pack) ;
        p128_baseline_c_mm_SET((int32_t)974200798, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS2_RTK_128(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_IMU3_129(), &PH);
        p129_yacc_SET((int16_t)(int16_t) -9963, PH.base.pack) ;
        p129_xgyro_SET((int16_t)(int16_t) -15274, PH.base.pack) ;
        p129_time_boot_ms_SET((uint32_t)2580467608L, PH.base.pack) ;
        p129_ygyro_SET((int16_t)(int16_t)25290, PH.base.pack) ;
        p129_xmag_SET((int16_t)(int16_t)26840, PH.base.pack) ;
        p129_xacc_SET((int16_t)(int16_t)22296, PH.base.pack) ;
        p129_zgyro_SET((int16_t)(int16_t) -4896, PH.base.pack) ;
        p129_zmag_SET((int16_t)(int16_t)28687, PH.base.pack) ;
        p129_ymag_SET((int16_t)(int16_t) -26198, PH.base.pack) ;
        p129_zacc_SET((int16_t)(int16_t) -12350, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_IMU3_129(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DATA_TRANSMISSION_HANDSHAKE_130(), &PH);
        p130_jpg_quality_SET((uint8_t)(uint8_t)96, PH.base.pack) ;
        p130_type_SET((uint8_t)(uint8_t)54, PH.base.pack) ;
        p130_size_SET((uint32_t)3428630679L, PH.base.pack) ;
        p130_height_SET((uint16_t)(uint16_t)45444, PH.base.pack) ;
        p130_packets_SET((uint16_t)(uint16_t)32924, PH.base.pack) ;
        p130_width_SET((uint16_t)(uint16_t)17743, PH.base.pack) ;
        p130_payload_SET((uint8_t)(uint8_t)211, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DATA_TRANSMISSION_HANDSHAKE_130(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ENCAPSULATED_DATA_131(), &PH);
        p131_seqnr_SET((uint16_t)(uint16_t)57183, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)244, (uint8_t)28, (uint8_t)240, (uint8_t)21, (uint8_t)221, (uint8_t)106, (uint8_t)121, (uint8_t)130, (uint8_t)55, (uint8_t)101, (uint8_t)44, (uint8_t)201, (uint8_t)159, (uint8_t)119, (uint8_t)20, (uint8_t)217, (uint8_t)95, (uint8_t)192, (uint8_t)186, (uint8_t)130, (uint8_t)120, (uint8_t)149, (uint8_t)251, (uint8_t)249, (uint8_t)88, (uint8_t)252, (uint8_t)121, (uint8_t)249, (uint8_t)140, (uint8_t)154, (uint8_t)189, (uint8_t)70, (uint8_t)37, (uint8_t)61, (uint8_t)138, (uint8_t)136, (uint8_t)206, (uint8_t)199, (uint8_t)87, (uint8_t)246, (uint8_t)244, (uint8_t)188, (uint8_t)185, (uint8_t)209, (uint8_t)218, (uint8_t)67, (uint8_t)123, (uint8_t)13, (uint8_t)157, (uint8_t)14, (uint8_t)84, (uint8_t)180, (uint8_t)155, (uint8_t)38, (uint8_t)11, (uint8_t)236, (uint8_t)245, (uint8_t)187, (uint8_t)64, (uint8_t)133, (uint8_t)254, (uint8_t)156, (uint8_t)45, (uint8_t)166, (uint8_t)73, (uint8_t)80, (uint8_t)172, (uint8_t)225, (uint8_t)80, (uint8_t)93, (uint8_t)83, (uint8_t)213, (uint8_t)12, (uint8_t)163, (uint8_t)210, (uint8_t)102, (uint8_t)82, (uint8_t)14, (uint8_t)239, (uint8_t)147, (uint8_t)144, (uint8_t)134, (uint8_t)0, (uint8_t)177, (uint8_t)158, (uint8_t)218, (uint8_t)14, (uint8_t)133, (uint8_t)254, (uint8_t)245, (uint8_t)135, (uint8_t)33, (uint8_t)236, (uint8_t)113, (uint8_t)238, (uint8_t)165, (uint8_t)148, (uint8_t)209, (uint8_t)142, (uint8_t)57, (uint8_t)172, (uint8_t)246, (uint8_t)140, (uint8_t)12, (uint8_t)148, (uint8_t)183, (uint8_t)166, (uint8_t)13, (uint8_t)170, (uint8_t)153, (uint8_t)130, (uint8_t)141, (uint8_t)215, (uint8_t)68, (uint8_t)87, (uint8_t)126, (uint8_t)18, (uint8_t)250, (uint8_t)168, (uint8_t)210, (uint8_t)179, (uint8_t)81, (uint8_t)41, (uint8_t)94, (uint8_t)209, (uint8_t)132, (uint8_t)180, (uint8_t)128, (uint8_t)27, (uint8_t)199, (uint8_t)179, (uint8_t)153, (uint8_t)52, (uint8_t)228, (uint8_t)239, (uint8_t)219, (uint8_t)112, (uint8_t)143, (uint8_t)145, (uint8_t)86, (uint8_t)210, (uint8_t)174, (uint8_t)69, (uint8_t)158, (uint8_t)223, (uint8_t)207, (uint8_t)60, (uint8_t)255, (uint8_t)198, (uint8_t)150, (uint8_t)167, (uint8_t)135, (uint8_t)72, (uint8_t)135, (uint8_t)230, (uint8_t)173, (uint8_t)169, (uint8_t)79, (uint8_t)196, (uint8_t)0, (uint8_t)210, (uint8_t)46, (uint8_t)47, (uint8_t)68, (uint8_t)63, (uint8_t)175, (uint8_t)131, (uint8_t)232, (uint8_t)26, (uint8_t)150, (uint8_t)206, (uint8_t)146, (uint8_t)12, (uint8_t)163, (uint8_t)178, (uint8_t)76, (uint8_t)244, (uint8_t)163, (uint8_t)51, (uint8_t)36, (uint8_t)195, (uint8_t)106, (uint8_t)77, (uint8_t)159, (uint8_t)107, (uint8_t)38, (uint8_t)25, (uint8_t)118, (uint8_t)174, (uint8_t)161, (uint8_t)150, (uint8_t)91, (uint8_t)80, (uint8_t)25, (uint8_t)154, (uint8_t)118, (uint8_t)245, (uint8_t)18, (uint8_t)210, (uint8_t)246, (uint8_t)56, (uint8_t)77, (uint8_t)210, (uint8_t)87, (uint8_t)135, (uint8_t)30, (uint8_t)229, (uint8_t)33, (uint8_t)96, (uint8_t)18, (uint8_t)125, (uint8_t)225, (uint8_t)16, (uint8_t)138, (uint8_t)152, (uint8_t)187, (uint8_t)113, (uint8_t)22, (uint8_t)146, (uint8_t)187, (uint8_t)186, (uint8_t)39, (uint8_t)90, (uint8_t)29, (uint8_t)188, (uint8_t)56, (uint8_t)22, (uint8_t)16, (uint8_t)86, (uint8_t)230, (uint8_t)43, (uint8_t)121, (uint8_t)246, (uint8_t)110, (uint8_t)223, (uint8_t)204, (uint8_t)114, (uint8_t)24, (uint8_t)22, (uint8_t)152, (uint8_t)203, (uint8_t)218, (uint8_t)150, (uint8_t)65, (uint8_t)71, (uint8_t)50, (uint8_t)77, (uint8_t)120, (uint8_t)82, (uint8_t)153, (uint8_t)173, (uint8_t)240, (uint8_t)242};
            p131_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_ENCAPSULATED_DATA_131(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DISTANCE_SENSOR_132(), &PH);
        p132_covariance_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        p132_orientation_SET(e_MAV_SENSOR_ORIENTATION_MAV_SENSOR_ROTATION_ROLL_180_PITCH_270, PH.base.pack) ;
        p132_min_distance_SET((uint16_t)(uint16_t)38335, PH.base.pack) ;
        p132_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_ULTRASOUND, PH.base.pack) ;
        p132_time_boot_ms_SET((uint32_t)3816722028L, PH.base.pack) ;
        p132_id_SET((uint8_t)(uint8_t)145, PH.base.pack) ;
        p132_max_distance_SET((uint16_t)(uint16_t)16339, PH.base.pack) ;
        p132_current_distance_SET((uint16_t)(uint16_t)17023, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DISTANCE_SENSOR_132(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REQUEST_133(), &PH);
        p133_lon_SET((int32_t)890286506, PH.base.pack) ;
        p133_mask_SET((uint64_t)8229399769302156884L, PH.base.pack) ;
        p133_grid_spacing_SET((uint16_t)(uint16_t)9218, PH.base.pack) ;
        p133_lat_SET((int32_t) -1395253698, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REQUEST_133(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_DATA_134(), &PH);
        p134_lat_SET((int32_t)1869757602, PH.base.pack) ;
        p134_grid_spacing_SET((uint16_t)(uint16_t)2841, PH.base.pack) ;
        p134_gridbit_SET((uint8_t)(uint8_t)48, PH.base.pack) ;
        {
            int16_t data_[] =  {(int16_t) -22027, (int16_t) -16529, (int16_t) -1672, (int16_t) -23142, (int16_t) -45, (int16_t) -3959, (int16_t) -29836, (int16_t)26291, (int16_t) -9066, (int16_t)23371, (int16_t) -13874, (int16_t)12712, (int16_t) -16784, (int16_t)29458, (int16_t) -16965, (int16_t)27288};
            p134_data__SET(&data_, 0, PH.base.pack) ;
        }
        p134_lon_SET((int32_t)813585548, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_DATA_134(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_CHECK_135(), &PH);
        p135_lat_SET((int32_t) -99258931, PH.base.pack) ;
        p135_lon_SET((int32_t) -731970747, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_CHECK_135(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_TERRAIN_REPORT_136(), &PH);
        p136_lon_SET((int32_t) -2020262525, PH.base.pack) ;
        p136_lat_SET((int32_t) -441665359, PH.base.pack) ;
        p136_pending_SET((uint16_t)(uint16_t)12634, PH.base.pack) ;
        p136_terrain_height_SET((float) -2.3669371E38F, PH.base.pack) ;
        p136_spacing_SET((uint16_t)(uint16_t)38549, PH.base.pack) ;
        p136_current_height_SET((float)3.2633519E38F, PH.base.pack) ;
        p136_loaded_SET((uint16_t)(uint16_t)51748, PH.base.pack) ;
        c_LoopBackDemoChannel_on_TERRAIN_REPORT_136(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE2_137(), &PH);
        p137_press_abs_SET((float) -1.1975603E38F, PH.base.pack) ;
        p137_time_boot_ms_SET((uint32_t)978589286L, PH.base.pack) ;
        p137_temperature_SET((int16_t)(int16_t)7287, PH.base.pack) ;
        p137_press_diff_SET((float) -1.2228719E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE2_137(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ATT_POS_MOCAP_138(), &PH);
        p138_z_SET((float) -1.3182828E38F, PH.base.pack) ;
        {
            float q[] =  {5.595143E37F, 1.749373E38F, -8.2311804E37F, 3.1756173E38F};
            p138_q_SET(&q, 0, PH.base.pack) ;
        }
        p138_x_SET((float)2.2323384E38F, PH.base.pack) ;
        p138_time_usec_SET((uint64_t)5555185719086581326L, PH.base.pack) ;
        p138_y_SET((float) -2.3923425E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ATT_POS_MOCAP_138(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_ACTUATOR_CONTROL_TARGET_139(), &PH);
        p139_target_component_SET((uint8_t)(uint8_t)173, PH.base.pack) ;
        p139_target_system_SET((uint8_t)(uint8_t)107, PH.base.pack) ;
        p139_group_mlx_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        p139_time_usec_SET((uint64_t)7713081588506888055L, PH.base.pack) ;
        {
            float controls[] =  {-2.4637777E38F, 1.5443764E38F, -3.1131562E38F, 1.9869435E38F, -2.2872948E38F, 8.750543E37F, 6.456044E37F, -7.9369916E37F};
            p139_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_SET_ACTUATOR_CONTROL_TARGET_139(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ACTUATOR_CONTROL_TARGET_140(), &PH);
        p140_time_usec_SET((uint64_t)8433554591385714751L, PH.base.pack) ;
        p140_group_mlx_SET((uint8_t)(uint8_t)202, PH.base.pack) ;
        {
            float controls[] =  {8.797724E37F, 3.0240334E38F, 1.4697126E38F, 1.877357E38F, 1.0812021E37F, 2.2474605E38F, 2.9421388E38F, -3.7815754E37F};
            p140_controls_SET(&controls, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_ACTUATOR_CONTROL_TARGET_140(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ALTITUDE_141(), &PH);
        p141_time_usec_SET((uint64_t)6217323102206626655L, PH.base.pack) ;
        p141_altitude_amsl_SET((float)1.9199587E38F, PH.base.pack) ;
        p141_altitude_monotonic_SET((float)8.093497E37F, PH.base.pack) ;
        p141_altitude_relative_SET((float)9.389106E37F, PH.base.pack) ;
        p141_altitude_terrain_SET((float)2.4599103E38F, PH.base.pack) ;
        p141_altitude_local_SET((float) -1.0303086E38F, PH.base.pack) ;
        p141_bottom_clearance_SET((float) -2.4929817E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ALTITUDE_141(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_RESOURCE_REQUEST_142(), &PH);
        p142_transfer_type_SET((uint8_t)(uint8_t)130, PH.base.pack) ;
        {
            uint8_t storage[] =  {(uint8_t)37, (uint8_t)11, (uint8_t)197, (uint8_t)34, (uint8_t)81, (uint8_t)36, (uint8_t)101, (uint8_t)203, (uint8_t)246, (uint8_t)165, (uint8_t)39, (uint8_t)178, (uint8_t)192, (uint8_t)248, (uint8_t)195, (uint8_t)210, (uint8_t)200, (uint8_t)134, (uint8_t)29, (uint8_t)46, (uint8_t)129, (uint8_t)71, (uint8_t)29, (uint8_t)191, (uint8_t)38, (uint8_t)215, (uint8_t)129, (uint8_t)105, (uint8_t)63, (uint8_t)128, (uint8_t)73, (uint8_t)7, (uint8_t)94, (uint8_t)47, (uint8_t)33, (uint8_t)195, (uint8_t)146, (uint8_t)217, (uint8_t)185, (uint8_t)43, (uint8_t)182, (uint8_t)88, (uint8_t)142, (uint8_t)232, (uint8_t)66, (uint8_t)55, (uint8_t)121, (uint8_t)78, (uint8_t)98, (uint8_t)55, (uint8_t)16, (uint8_t)141, (uint8_t)234, (uint8_t)237, (uint8_t)5, (uint8_t)192, (uint8_t)128, (uint8_t)195, (uint8_t)64, (uint8_t)131, (uint8_t)99, (uint8_t)230, (uint8_t)210, (uint8_t)13, (uint8_t)106, (uint8_t)178, (uint8_t)214, (uint8_t)172, (uint8_t)199, (uint8_t)224, (uint8_t)117, (uint8_t)242, (uint8_t)217, (uint8_t)165, (uint8_t)55, (uint8_t)122, (uint8_t)76, (uint8_t)178, (uint8_t)33, (uint8_t)9, (uint8_t)22, (uint8_t)7, (uint8_t)180, (uint8_t)9, (uint8_t)199, (uint8_t)252, (uint8_t)77, (uint8_t)205, (uint8_t)196, (uint8_t)150, (uint8_t)126, (uint8_t)43, (uint8_t)157, (uint8_t)122, (uint8_t)245, (uint8_t)211, (uint8_t)151, (uint8_t)183, (uint8_t)25, (uint8_t)126, (uint8_t)137, (uint8_t)238, (uint8_t)230, (uint8_t)2, (uint8_t)32, (uint8_t)179, (uint8_t)118, (uint8_t)2, (uint8_t)189, (uint8_t)226, (uint8_t)254, (uint8_t)126, (uint8_t)159, (uint8_t)210, (uint8_t)151, (uint8_t)249, (uint8_t)151, (uint8_t)193, (uint8_t)207, (uint8_t)86};
            p142_storage_SET(&storage, 0, PH.base.pack) ;
        }
        p142_request_id_SET((uint8_t)(uint8_t)206, PH.base.pack) ;
        p142_uri_type_SET((uint8_t)(uint8_t)253, PH.base.pack) ;
        {
            uint8_t uri[] =  {(uint8_t)103, (uint8_t)224, (uint8_t)55, (uint8_t)107, (uint8_t)80, (uint8_t)195, (uint8_t)18, (uint8_t)31, (uint8_t)21, (uint8_t)186, (uint8_t)142, (uint8_t)251, (uint8_t)116, (uint8_t)77, (uint8_t)55, (uint8_t)81, (uint8_t)162, (uint8_t)1, (uint8_t)55, (uint8_t)82, (uint8_t)53, (uint8_t)122, (uint8_t)239, (uint8_t)51, (uint8_t)228, (uint8_t)34, (uint8_t)70, (uint8_t)188, (uint8_t)117, (uint8_t)181, (uint8_t)41, (uint8_t)179, (uint8_t)7, (uint8_t)111, (uint8_t)236, (uint8_t)245, (uint8_t)64, (uint8_t)73, (uint8_t)172, (uint8_t)55, (uint8_t)92, (uint8_t)49, (uint8_t)82, (uint8_t)238, (uint8_t)179, (uint8_t)82, (uint8_t)223, (uint8_t)116, (uint8_t)192, (uint8_t)204, (uint8_t)80, (uint8_t)206, (uint8_t)30, (uint8_t)2, (uint8_t)107, (uint8_t)53, (uint8_t)30, (uint8_t)86, (uint8_t)12, (uint8_t)96, (uint8_t)30, (uint8_t)73, (uint8_t)11, (uint8_t)3, (uint8_t)116, (uint8_t)166, (uint8_t)64, (uint8_t)9, (uint8_t)120, (uint8_t)59, (uint8_t)71, (uint8_t)234, (uint8_t)232, (uint8_t)90, (uint8_t)23, (uint8_t)55, (uint8_t)210, (uint8_t)128, (uint8_t)174, (uint8_t)189, (uint8_t)248, (uint8_t)97, (uint8_t)186, (uint8_t)216, (uint8_t)185, (uint8_t)76, (uint8_t)47, (uint8_t)56, (uint8_t)104, (uint8_t)147, (uint8_t)158, (uint8_t)120, (uint8_t)109, (uint8_t)36, (uint8_t)68, (uint8_t)176, (uint8_t)156, (uint8_t)30, (uint8_t)16, (uint8_t)166, (uint8_t)246, (uint8_t)221, (uint8_t)9, (uint8_t)14, (uint8_t)136, (uint8_t)231, (uint8_t)65, (uint8_t)0, (uint8_t)109, (uint8_t)252, (uint8_t)243, (uint8_t)222, (uint8_t)250, (uint8_t)153, (uint8_t)210, (uint8_t)244, (uint8_t)180, (uint8_t)231, (uint8_t)197, (uint8_t)142};
            p142_uri_SET(&uri, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_RESOURCE_REQUEST_142(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SCALED_PRESSURE3_143(), &PH);
        p143_press_diff_SET((float) -1.4023477E38F, PH.base.pack) ;
        p143_time_boot_ms_SET((uint32_t)2112206153L, PH.base.pack) ;
        p143_press_abs_SET((float)3.3429764E38F, PH.base.pack) ;
        p143_temperature_SET((int16_t)(int16_t) -9011, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SCALED_PRESSURE3_143(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FOLLOW_TARGET_144(), &PH);
        {
            float acc[] =  {1.298003E38F, -3.2687813E38F, 1.3835711E38F};
            p144_acc_SET(&acc, 0, PH.base.pack) ;
        }
        {
            float vel[] =  {-9.645967E37F, -3.117291E37F, 7.401917E37F};
            p144_vel_SET(&vel, 0, PH.base.pack) ;
        }
        p144_lon_SET((int32_t)1893696585, PH.base.pack) ;
        p144_lat_SET((int32_t)107872323, PH.base.pack) ;
        {
            float position_cov[] =  {1.7667397E38F, 3.549423E36F, 7.1794335E37F};
            p144_position_cov_SET(&position_cov, 0, PH.base.pack) ;
        }
        {
            float rates[] =  {-2.9402243E38F, 3.1200412E38F, -3.0300413E38F};
            p144_rates_SET(&rates, 0, PH.base.pack) ;
        }
        p144_timestamp_SET((uint64_t)1793977892703052353L, PH.base.pack) ;
        p144_custom_state_SET((uint64_t)4090244485321822172L, PH.base.pack) ;
        p144_est_capabilities_SET((uint8_t)(uint8_t)52, PH.base.pack) ;
        {
            float attitude_q[] =  {-1.5079976E38F, -3.3120936E38F, -3.3805157E38F, 7.568284E37F};
            p144_attitude_q_SET(&attitude_q, 0, PH.base.pack) ;
        }
        p144_alt_SET((float) -2.4912001E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FOLLOW_TARGET_144(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CONTROL_SYSTEM_STATE_146(), &PH);
        p146_yaw_rate_SET((float) -1.9951496E38F, PH.base.pack) ;
        p146_y_vel_SET((float) -3.0856017E38F, PH.base.pack) ;
        p146_z_vel_SET((float) -1.2965553E38F, PH.base.pack) ;
        p146_z_pos_SET((float)2.400677E38F, PH.base.pack) ;
        p146_x_pos_SET((float) -1.7321604E38F, PH.base.pack) ;
        {
            float pos_variance[] =  {-1.5687349E38F, -2.2412927E38F, 2.221332E38F};
            p146_pos_variance_SET(&pos_variance, 0, PH.base.pack) ;
        }
        p146_pitch_rate_SET((float)1.0414815E38F, PH.base.pack) ;
        p146_z_acc_SET((float)3.1760688E38F, PH.base.pack) ;
        p146_time_usec_SET((uint64_t)2322542993496965825L, PH.base.pack) ;
        {
            float vel_variance[] =  {6.1187497E37F, -2.3755223E38F, 1.1106202E38F};
            p146_vel_variance_SET(&vel_variance, 0, PH.base.pack) ;
        }
        {
            float q[] =  {6.521833E37F, -2.2146905E38F, 1.4690012E38F, 3.7554185E37F};
            p146_q_SET(&q, 0, PH.base.pack) ;
        }
        p146_y_pos_SET((float) -2.7450404E38F, PH.base.pack) ;
        p146_airspeed_SET((float) -1.383807E38F, PH.base.pack) ;
        p146_x_acc_SET((float) -8.79271E37F, PH.base.pack) ;
        p146_roll_rate_SET((float) -1.6407169E38F, PH.base.pack) ;
        p146_x_vel_SET((float)7.404201E37F, PH.base.pack) ;
        p146_y_acc_SET((float) -3.1861842E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CONTROL_SYSTEM_STATE_146(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BATTERY_STATUS_147(), &PH);
        p147_energy_consumed_SET((int32_t) -1666573051, PH.base.pack) ;
        p147_temperature_SET((int16_t)(int16_t)27934, PH.base.pack) ;
        p147_id_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p147_battery_remaining_SET((int8_t)(int8_t)49, PH.base.pack) ;
        p147_type_SET(e_MAV_BATTERY_TYPE_MAV_BATTERY_TYPE_LIPO, PH.base.pack) ;
        p147_battery_function_SET(e_MAV_BATTERY_FUNCTION_MAV_BATTERY_FUNCTION_PROPULSION, PH.base.pack) ;
        p147_current_battery_SET((int16_t)(int16_t) -23971, PH.base.pack) ;
        p147_current_consumed_SET((int32_t)1021715915, PH.base.pack) ;
        {
            uint16_t voltages[] =  {(uint16_t)41251, (uint16_t)35445, (uint16_t)19370, (uint16_t)31949, (uint16_t)61573, (uint16_t)36238, (uint16_t)51019, (uint16_t)36270, (uint16_t)63186, (uint16_t)22193};
            p147_voltages_SET(&voltages, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_BATTERY_STATUS_147(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AUTOPILOT_VERSION_148(), &PH);
        {
            uint8_t uid2[] =  {(uint8_t)225, (uint8_t)168, (uint8_t)212, (uint8_t)56, (uint8_t)56, (uint8_t)105, (uint8_t)169, (uint8_t)231, (uint8_t)42, (uint8_t)212, (uint8_t)192, (uint8_t)33, (uint8_t)112, (uint8_t)254, (uint8_t)224, (uint8_t)228, (uint8_t)157, (uint8_t)215};
            p148_uid2_SET(&uid2, 0, &PH) ;
        }
        p148_capabilities_SET(e_MAV_PROTOCOL_CAPABILITY_MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT, PH.base.pack) ;
        p148_board_version_SET((uint32_t)1453073520L, PH.base.pack) ;
        p148_vendor_id_SET((uint16_t)(uint16_t)62951, PH.base.pack) ;
        p148_os_sw_version_SET((uint32_t)1626150905L, PH.base.pack) ;
        {
            uint8_t middleware_custom_version[] =  {(uint8_t)226, (uint8_t)150, (uint8_t)175, (uint8_t)130, (uint8_t)253, (uint8_t)151, (uint8_t)62, (uint8_t)44};
            p148_middleware_custom_version_SET(&middleware_custom_version, 0, PH.base.pack) ;
        }
        p148_flight_sw_version_SET((uint32_t)770410931L, PH.base.pack) ;
        p148_middleware_sw_version_SET((uint32_t)2251704304L, PH.base.pack) ;
        {
            uint8_t flight_custom_version[] =  {(uint8_t)254, (uint8_t)94, (uint8_t)234, (uint8_t)234, (uint8_t)29, (uint8_t)120, (uint8_t)240, (uint8_t)97};
            p148_flight_custom_version_SET(&flight_custom_version, 0, PH.base.pack) ;
        }
        p148_product_id_SET((uint16_t)(uint16_t)59228, PH.base.pack) ;
        p148_uid_SET((uint64_t)7702990746507491038L, PH.base.pack) ;
        {
            uint8_t os_custom_version[] =  {(uint8_t)99, (uint8_t)132, (uint8_t)93, (uint8_t)144, (uint8_t)52, (uint8_t)34, (uint8_t)26, (uint8_t)7};
            p148_os_custom_version_SET(&os_custom_version, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_AUTOPILOT_VERSION_148(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LANDING_TARGET_149(), &PH);
        p149_angle_y_SET((float) -1.750509E38F, PH.base.pack) ;
        p149_size_x_SET((float) -3.226967E38F, PH.base.pack) ;
        p149_target_num_SET((uint8_t)(uint8_t)57, PH.base.pack) ;
        p149_size_y_SET((float)7.9611703E37F, PH.base.pack) ;
        {
            float q[] =  {7.082778E37F, 1.5458E38F, 1.5896906E38F, 4.1562235E37F};
            p149_q_SET(&q, 0, &PH) ;
        }
        p149_position_valid_SET((uint8_t)(uint8_t)209, &PH) ;
        p149_z_SET((float) -9.764795E37F, &PH) ;
        p149_y_SET((float) -3.365397E38F, &PH) ;
        p149_type_SET(e_LANDING_TARGET_TYPE_LANDING_TARGET_TYPE_LIGHT_BEACON, PH.base.pack) ;
        p149_angle_x_SET((float) -1.3169938E38F, PH.base.pack) ;
        p149_time_usec_SET((uint64_t)8454174349278249267L, PH.base.pack) ;
        p149_x_SET((float) -3.1930997E38F, &PH) ;
        p149_frame_SET(e_MAV_FRAME_MAV_FRAME_BODY_OFFSET_NED, PH.base.pack) ;
        p149_distance_SET((float)1.9786227E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LANDING_TARGET_149(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AQ_TELEMETRY_F_150(), &PH);
        p150_value12_SET((float)2.7727482E38F, PH.base.pack) ;
        p150_value14_SET((float) -3.1282923E38F, PH.base.pack) ;
        p150_value4_SET((float) -1.5014304E38F, PH.base.pack) ;
        p150_value15_SET((float) -2.430555E38F, PH.base.pack) ;
        p150_value19_SET((float)2.3695941E38F, PH.base.pack) ;
        p150_value16_SET((float)2.6582501E38F, PH.base.pack) ;
        p150_value6_SET((float) -3.492019E37F, PH.base.pack) ;
        p150_value10_SET((float) -4.994688E36F, PH.base.pack) ;
        p150_value8_SET((float) -2.6958933E38F, PH.base.pack) ;
        p150_value11_SET((float)1.7632632E38F, PH.base.pack) ;
        p150_value17_SET((float) -1.493297E38F, PH.base.pack) ;
        p150_value18_SET((float)1.1879066E38F, PH.base.pack) ;
        p150_value5_SET((float) -1.5081113E38F, PH.base.pack) ;
        p150_value20_SET((float)1.3320754E38F, PH.base.pack) ;
        p150_value7_SET((float)1.073884E38F, PH.base.pack) ;
        p150_value2_SET((float) -3.0606353E38F, PH.base.pack) ;
        p150_value13_SET((float)2.8382575E38F, PH.base.pack) ;
        p150_value9_SET((float) -2.1015673E38F, PH.base.pack) ;
        p150_value1_SET((float)2.1998571E38F, PH.base.pack) ;
        p150_value3_SET((float)5.605671E37F, PH.base.pack) ;
        p150_Index_SET((uint16_t)(uint16_t)5194, PH.base.pack) ;
        c_LoopBackDemoChannel_on_AQ_TELEMETRY_F_150(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_AQ_ESC_TELEMETRY_152(), &PH);
        {
            uint32_t data0[] =  {106979466L, 3781519381L, 1082728225L, 3449363819L};
            p152_data0_SET(&data0, 0, PH.base.pack) ;
        }
        {
            uint8_t data_version[] =  {(uint8_t)103, (uint8_t)41, (uint8_t)228, (uint8_t)133};
            p152_data_version_SET(&data_version, 0, PH.base.pack) ;
        }
        p152_seq_SET((uint8_t)(uint8_t)82, PH.base.pack) ;
        {
            uint8_t escid[] =  {(uint8_t)99, (uint8_t)34, (uint8_t)45, (uint8_t)148};
            p152_escid_SET(&escid, 0, PH.base.pack) ;
        }
        p152_num_in_seq_SET((uint8_t)(uint8_t)100, PH.base.pack) ;
        {
            uint32_t data1[] =  {2385176779L, 2306989917L, 823114701L, 2103296916L};
            p152_data1_SET(&data1, 0, PH.base.pack) ;
        }
        p152_num_motors_SET((uint8_t)(uint8_t)232, PH.base.pack) ;
        p152_time_boot_ms_SET((uint32_t)108898985L, PH.base.pack) ;
        {
            uint16_t status_age[] =  {(uint16_t)20018, (uint16_t)60974, (uint16_t)22882, (uint16_t)44388};
            p152_status_age_SET(&status_age, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_AQ_ESC_TELEMETRY_152(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ESTIMATOR_STATUS_230(), &PH);
        p230_pos_horiz_ratio_SET((float)2.0984948E38F, PH.base.pack) ;
        p230_flags_SET(e_ESTIMATOR_STATUS_FLAGS_ESTIMATOR_VELOCITY_HORIZ, PH.base.pack) ;
        p230_tas_ratio_SET((float) -2.3369297E38F, PH.base.pack) ;
        p230_time_usec_SET((uint64_t)3550154252437264000L, PH.base.pack) ;
        p230_pos_vert_accuracy_SET((float) -2.8400578E38F, PH.base.pack) ;
        p230_pos_horiz_accuracy_SET((float) -1.672426E37F, PH.base.pack) ;
        p230_mag_ratio_SET((float) -1.8288476E38F, PH.base.pack) ;
        p230_hagl_ratio_SET((float)2.5537014E38F, PH.base.pack) ;
        p230_vel_ratio_SET((float)1.948079E38F, PH.base.pack) ;
        p230_pos_vert_ratio_SET((float) -1.5454398E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ESTIMATOR_STATUS_230(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIND_COV_231(), &PH);
        p231_var_vert_SET((float)2.4534807E38F, PH.base.pack) ;
        p231_wind_alt_SET((float) -3.3286018E38F, PH.base.pack) ;
        p231_wind_y_SET((float)8.182787E37F, PH.base.pack) ;
        p231_wind_z_SET((float) -3.3186815E38F, PH.base.pack) ;
        p231_horiz_accuracy_SET((float)9.056634E37F, PH.base.pack) ;
        p231_time_usec_SET((uint64_t)814389179059969894L, PH.base.pack) ;
        p231_var_horiz_SET((float) -3.4415155E37F, PH.base.pack) ;
        p231_vert_accuracy_SET((float)2.8652072E38F, PH.base.pack) ;
        p231_wind_x_SET((float) -5.0219715E36F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_WIND_COV_231(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_INPUT_232(), &PH);
        p232_vert_accuracy_SET((float) -3.3278374E38F, PH.base.pack) ;
        p232_time_week_ms_SET((uint32_t)438328789L, PH.base.pack) ;
        p232_alt_SET((float)1.6053685E38F, PH.base.pack) ;
        p232_satellites_visible_SET((uint8_t)(uint8_t)119, PH.base.pack) ;
        p232_horiz_accuracy_SET((float) -7.799927E37F, PH.base.pack) ;
        p232_hdop_SET((float) -1.4369478E38F, PH.base.pack) ;
        p232_ignore_flags_SET(e_GPS_INPUT_IGNORE_FLAGS_GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY, PH.base.pack) ;
        p232_time_week_SET((uint16_t)(uint16_t)24101, PH.base.pack) ;
        p232_lat_SET((int32_t) -844776903, PH.base.pack) ;
        p232_lon_SET((int32_t) -1998096099, PH.base.pack) ;
        p232_time_usec_SET((uint64_t)3964015191382715958L, PH.base.pack) ;
        p232_gps_id_SET((uint8_t)(uint8_t)233, PH.base.pack) ;
        p232_speed_accuracy_SET((float)5.9887906E37F, PH.base.pack) ;
        p232_ve_SET((float) -1.4404676E38F, PH.base.pack) ;
        p232_vdop_SET((float)2.851713E38F, PH.base.pack) ;
        p232_vd_SET((float) -4.741962E37F, PH.base.pack) ;
        p232_vn_SET((float)2.231187E38F, PH.base.pack) ;
        p232_fix_type_SET((uint8_t)(uint8_t)13, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_INPUT_232(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_GPS_RTCM_DATA_233(), &PH);
        p233_len_SET((uint8_t)(uint8_t)216, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)222, (uint8_t)244, (uint8_t)147, (uint8_t)159, (uint8_t)161, (uint8_t)100, (uint8_t)53, (uint8_t)167, (uint8_t)220, (uint8_t)113, (uint8_t)152, (uint8_t)200, (uint8_t)191, (uint8_t)57, (uint8_t)123, (uint8_t)60, (uint8_t)32, (uint8_t)127, (uint8_t)203, (uint8_t)187, (uint8_t)150, (uint8_t)221, (uint8_t)248, (uint8_t)91, (uint8_t)216, (uint8_t)5, (uint8_t)123, (uint8_t)35, (uint8_t)184, (uint8_t)45, (uint8_t)216, (uint8_t)107, (uint8_t)252, (uint8_t)51, (uint8_t)133, (uint8_t)115, (uint8_t)193, (uint8_t)55, (uint8_t)6, (uint8_t)148, (uint8_t)53, (uint8_t)224, (uint8_t)234, (uint8_t)163, (uint8_t)11, (uint8_t)49, (uint8_t)215, (uint8_t)31, (uint8_t)214, (uint8_t)239, (uint8_t)150, (uint8_t)173, (uint8_t)62, (uint8_t)197, (uint8_t)207, (uint8_t)114, (uint8_t)160, (uint8_t)176, (uint8_t)138, (uint8_t)10, (uint8_t)4, (uint8_t)241, (uint8_t)232, (uint8_t)203, (uint8_t)160, (uint8_t)231, (uint8_t)173, (uint8_t)206, (uint8_t)219, (uint8_t)14, (uint8_t)171, (uint8_t)10, (uint8_t)51, (uint8_t)223, (uint8_t)220, (uint8_t)79, (uint8_t)7, (uint8_t)137, (uint8_t)72, (uint8_t)189, (uint8_t)181, (uint8_t)66, (uint8_t)86, (uint8_t)154, (uint8_t)150, (uint8_t)139, (uint8_t)95, (uint8_t)76, (uint8_t)146, (uint8_t)146, (uint8_t)32, (uint8_t)23, (uint8_t)78, (uint8_t)30, (uint8_t)140, (uint8_t)204, (uint8_t)72, (uint8_t)229, (uint8_t)78, (uint8_t)147, (uint8_t)34, (uint8_t)160, (uint8_t)206, (uint8_t)160, (uint8_t)195, (uint8_t)190, (uint8_t)60, (uint8_t)99, (uint8_t)35, (uint8_t)236, (uint8_t)79, (uint8_t)52, (uint8_t)191, (uint8_t)191, (uint8_t)141, (uint8_t)10, (uint8_t)58, (uint8_t)239, (uint8_t)83, (uint8_t)156, (uint8_t)19, (uint8_t)187, (uint8_t)119, (uint8_t)69, (uint8_t)232, (uint8_t)61, (uint8_t)104, (uint8_t)76, (uint8_t)33, (uint8_t)243, (uint8_t)86, (uint8_t)112, (uint8_t)120, (uint8_t)239, (uint8_t)126, (uint8_t)210, (uint8_t)72, (uint8_t)232, (uint8_t)212, (uint8_t)47, (uint8_t)179, (uint8_t)165, (uint8_t)72, (uint8_t)126, (uint8_t)182, (uint8_t)11, (uint8_t)197, (uint8_t)11, (uint8_t)251, (uint8_t)248, (uint8_t)76, (uint8_t)5, (uint8_t)82, (uint8_t)15, (uint8_t)61, (uint8_t)37, (uint8_t)204, (uint8_t)74, (uint8_t)24, (uint8_t)117, (uint8_t)55, (uint8_t)123, (uint8_t)76, (uint8_t)21, (uint8_t)70, (uint8_t)112, (uint8_t)176, (uint8_t)179, (uint8_t)159, (uint8_t)7, (uint8_t)167, (uint8_t)156, (uint8_t)183, (uint8_t)5, (uint8_t)199, (uint8_t)36, (uint8_t)104, (uint8_t)35, (uint8_t)165, (uint8_t)50};
            p233_data__SET(&data_, 0, PH.base.pack) ;
        }
        p233_flags_SET((uint8_t)(uint8_t)227, PH.base.pack) ;
        c_LoopBackDemoChannel_on_GPS_RTCM_DATA_233(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HIGH_LATENCY_234(), &PH);
        p234_custom_mode_SET((uint32_t)2513242689L, PH.base.pack) ;
        p234_longitude_SET((int32_t)57510037, PH.base.pack) ;
        p234_base_mode_SET(e_MAV_MODE_FLAG_MAV_MODE_FLAG_SAFETY_ARMED, PH.base.pack) ;
        p234_battery_remaining_SET((uint8_t)(uint8_t)11, PH.base.pack) ;
        p234_failsafe_SET((uint8_t)(uint8_t)231, PH.base.pack) ;
        p234_temperature_air_SET((int8_t)(int8_t)27, PH.base.pack) ;
        p234_wp_distance_SET((uint16_t)(uint16_t)1674, PH.base.pack) ;
        p234_roll_SET((int16_t)(int16_t)25891, PH.base.pack) ;
        p234_gps_fix_type_SET(e_GPS_FIX_TYPE_GPS_FIX_TYPE_STATIC, PH.base.pack) ;
        p234_wp_num_SET((uint8_t)(uint8_t)203, PH.base.pack) ;
        p234_altitude_amsl_SET((int16_t)(int16_t)16427, PH.base.pack) ;
        p234_airspeed_sp_SET((uint8_t)(uint8_t)120, PH.base.pack) ;
        p234_heading_sp_SET((int16_t)(int16_t) -28916, PH.base.pack) ;
        p234_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_TAKEOFF, PH.base.pack) ;
        p234_latitude_SET((int32_t)1876731121, PH.base.pack) ;
        p234_airspeed_SET((uint8_t)(uint8_t)201, PH.base.pack) ;
        p234_temperature_SET((int8_t)(int8_t) -66, PH.base.pack) ;
        p234_altitude_sp_SET((int16_t)(int16_t) -19292, PH.base.pack) ;
        p234_pitch_SET((int16_t)(int16_t)25906, PH.base.pack) ;
        p234_gps_nsat_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p234_climb_rate_SET((int8_t)(int8_t) -38, PH.base.pack) ;
        p234_groundspeed_SET((uint8_t)(uint8_t)169, PH.base.pack) ;
        p234_heading_SET((uint16_t)(uint16_t)36015, PH.base.pack) ;
        p234_throttle_SET((int8_t)(int8_t) -97, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HIGH_LATENCY_234(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIBRATION_241(), &PH);
        p241_clipping_2_SET((uint32_t)443599353L, PH.base.pack) ;
        p241_clipping_1_SET((uint32_t)1204613084L, PH.base.pack) ;
        p241_vibration_z_SET((float)1.4975082E38F, PH.base.pack) ;
        p241_vibration_y_SET((float) -3.0223715E38F, PH.base.pack) ;
        p241_clipping_0_SET((uint32_t)60729767L, PH.base.pack) ;
        p241_vibration_x_SET((float)1.1212366E38F, PH.base.pack) ;
        p241_time_usec_SET((uint64_t)1123013032334391021L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIBRATION_241(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_HOME_POSITION_242(), &PH);
        p242_x_SET((float)8.027735E36F, PH.base.pack) ;
        p242_approach_z_SET((float) -1.4027694E38F, PH.base.pack) ;
        p242_approach_y_SET((float)2.4004932E38F, PH.base.pack) ;
        p242_approach_x_SET((float) -6.859255E37F, PH.base.pack) ;
        p242_z_SET((float) -1.9507076E38F, PH.base.pack) ;
        p242_altitude_SET((int32_t)1252423298, PH.base.pack) ;
        p242_latitude_SET((int32_t) -38364954, PH.base.pack) ;
        {
            float q[] =  {2.993938E38F, -1.4180797E38F, 2.4258835E38F, -4.3149483E37F};
            p242_q_SET(&q, 0, PH.base.pack) ;
        }
        p242_longitude_SET((int32_t) -1650708290, PH.base.pack) ;
        p242_time_usec_SET((uint64_t)5908498008748899177L, &PH) ;
        p242_y_SET((float) -2.2185123E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_HOME_POSITION_242(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_HOME_POSITION_243(), &PH);
        p243_approach_x_SET((float) -2.8481854E38F, PH.base.pack) ;
        {
            float q[] =  {-2.1983384E38F, 7.1264477E37F, -1.1253487E37F, 1.7473251E38F};
            p243_q_SET(&q, 0, PH.base.pack) ;
        }
        p243_x_SET((float)3.1022246E38F, PH.base.pack) ;
        p243_latitude_SET((int32_t)1519090844, PH.base.pack) ;
        p243_target_system_SET((uint8_t)(uint8_t)155, PH.base.pack) ;
        p243_approach_z_SET((float)6.833373E37F, PH.base.pack) ;
        p243_y_SET((float)1.88521E38F, PH.base.pack) ;
        p243_approach_y_SET((float) -3.1245468E38F, PH.base.pack) ;
        p243_z_SET((float) -3.669853E37F, PH.base.pack) ;
        p243_time_usec_SET((uint64_t)6232826439787518417L, &PH) ;
        p243_altitude_SET((int32_t)638102777, PH.base.pack) ;
        p243_longitude_SET((int32_t) -1351104907, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_HOME_POSITION_243(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MESSAGE_INTERVAL_244(), &PH);
        p244_interval_us_SET((int32_t) -1551090296, PH.base.pack) ;
        p244_message_id_SET((uint16_t)(uint16_t)18632, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MESSAGE_INTERVAL_244(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_EXTENDED_SYS_STATE_245(), &PH);
        p245_vtol_state_SET(e_MAV_VTOL_STATE_MAV_VTOL_STATE_TRANSITION_TO_MC, PH.base.pack) ;
        p245_landed_state_SET(e_MAV_LANDED_STATE_MAV_LANDED_STATE_UNDEFINED, PH.base.pack) ;
        c_LoopBackDemoChannel_on_EXTENDED_SYS_STATE_245(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_ADSB_VEHICLE_246(), &PH);
        {
            char16_t* callsign = u"hwnmCbws";
            p246_callsign_SET_(callsign, &PH) ;
        }
        p246_hor_velocity_SET((uint16_t)(uint16_t)15217, PH.base.pack) ;
        p246_emitter_type_SET(e_ADSB_EMITTER_TYPE_ADSB_EMITTER_TYPE_UAV, PH.base.pack) ;
        p246_ver_velocity_SET((int16_t)(int16_t)17245, PH.base.pack) ;
        p246_flags_SET(e_ADSB_FLAGS_ADSB_FLAGS_VALID_VELOCITY, PH.base.pack) ;
        p246_squawk_SET((uint16_t)(uint16_t)6991, PH.base.pack) ;
        p246_tslc_SET((uint8_t)(uint8_t)29, PH.base.pack) ;
        p246_heading_SET((uint16_t)(uint16_t)7915, PH.base.pack) ;
        p246_altitude_SET((int32_t)1828949709, PH.base.pack) ;
        p246_ICAO_address_SET((uint32_t)2374959065L, PH.base.pack) ;
        p246_altitude_type_SET(e_ADSB_ALTITUDE_TYPE_ADSB_ALTITUDE_TYPE_GEOMETRIC, PH.base.pack) ;
        p246_lon_SET((int32_t)337696588, PH.base.pack) ;
        p246_lat_SET((int32_t) -50572995, PH.base.pack) ;
        c_LoopBackDemoChannel_on_ADSB_VEHICLE_246(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_COLLISION_247(), &PH);
        p247_time_to_minimum_delta_SET((float) -7.2788416E37F, PH.base.pack) ;
        p247_action_SET(e_MAV_COLLISION_ACTION_MAV_COLLISION_ACTION_MOVE_PERPENDICULAR, PH.base.pack) ;
        p247_horizontal_minimum_delta_SET((float)2.2315283E38F, PH.base.pack) ;
        p247_src__SET(e_MAV_COLLISION_SRC_MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT, PH.base.pack) ;
        p247_altitude_minimum_delta_SET((float)1.2133445E38F, PH.base.pack) ;
        p247_id_SET((uint32_t)3313692311L, PH.base.pack) ;
        p247_threat_level_SET(e_MAV_COLLISION_THREAT_LEVEL_MAV_COLLISION_THREAT_LEVEL_HIGH, PH.base.pack) ;
        c_LoopBackDemoChannel_on_COLLISION_247(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_V2_EXTENSION_248(), &PH);
        p248_target_component_SET((uint8_t)(uint8_t)251, PH.base.pack) ;
        p248_target_network_SET((uint8_t)(uint8_t)67, PH.base.pack) ;
        p248_target_system_SET((uint8_t)(uint8_t)110, PH.base.pack) ;
        p248_message_type_SET((uint16_t)(uint16_t)31460, PH.base.pack) ;
        {
            uint8_t payload[] =  {(uint8_t)20, (uint8_t)86, (uint8_t)99, (uint8_t)167, (uint8_t)160, (uint8_t)226, (uint8_t)133, (uint8_t)217, (uint8_t)96, (uint8_t)221, (uint8_t)195, (uint8_t)227, (uint8_t)4, (uint8_t)167, (uint8_t)217, (uint8_t)118, (uint8_t)23, (uint8_t)192, (uint8_t)36, (uint8_t)18, (uint8_t)176, (uint8_t)224, (uint8_t)120, (uint8_t)109, (uint8_t)59, (uint8_t)105, (uint8_t)101, (uint8_t)21, (uint8_t)176, (uint8_t)40, (uint8_t)158, (uint8_t)11, (uint8_t)132, (uint8_t)164, (uint8_t)91, (uint8_t)134, (uint8_t)48, (uint8_t)216, (uint8_t)122, (uint8_t)64, (uint8_t)182, (uint8_t)22, (uint8_t)140, (uint8_t)106, (uint8_t)60, (uint8_t)1, (uint8_t)59, (uint8_t)3, (uint8_t)4, (uint8_t)48, (uint8_t)179, (uint8_t)149, (uint8_t)92, (uint8_t)80, (uint8_t)140, (uint8_t)215, (uint8_t)162, (uint8_t)188, (uint8_t)70, (uint8_t)3, (uint8_t)234, (uint8_t)177, (uint8_t)169, (uint8_t)55, (uint8_t)69, (uint8_t)37, (uint8_t)166, (uint8_t)208, (uint8_t)178, (uint8_t)166, (uint8_t)246, (uint8_t)165, (uint8_t)99, (uint8_t)146, (uint8_t)103, (uint8_t)141, (uint8_t)241, (uint8_t)60, (uint8_t)194, (uint8_t)129, (uint8_t)135, (uint8_t)43, (uint8_t)205, (uint8_t)158, (uint8_t)235, (uint8_t)209, (uint8_t)15, (uint8_t)119, (uint8_t)15, (uint8_t)99, (uint8_t)113, (uint8_t)180, (uint8_t)155, (uint8_t)70, (uint8_t)159, (uint8_t)182, (uint8_t)240, (uint8_t)181, (uint8_t)143, (uint8_t)246, (uint8_t)65, (uint8_t)94, (uint8_t)206, (uint8_t)50, (uint8_t)48, (uint8_t)213, (uint8_t)204, (uint8_t)83, (uint8_t)126, (uint8_t)136, (uint8_t)214, (uint8_t)58, (uint8_t)134, (uint8_t)146, (uint8_t)16, (uint8_t)82, (uint8_t)191, (uint8_t)5, (uint8_t)235, (uint8_t)166, (uint8_t)141, (uint8_t)53, (uint8_t)28, (uint8_t)43, (uint8_t)162, (uint8_t)8, (uint8_t)21, (uint8_t)122, (uint8_t)87, (uint8_t)233, (uint8_t)21, (uint8_t)140, (uint8_t)176, (uint8_t)28, (uint8_t)179, (uint8_t)150, (uint8_t)104, (uint8_t)60, (uint8_t)26, (uint8_t)73, (uint8_t)31, (uint8_t)209, (uint8_t)169, (uint8_t)100, (uint8_t)211, (uint8_t)72, (uint8_t)30, (uint8_t)26, (uint8_t)152, (uint8_t)0, (uint8_t)161, (uint8_t)244, (uint8_t)157, (uint8_t)163, (uint8_t)120, (uint8_t)57, (uint8_t)138, (uint8_t)252, (uint8_t)120, (uint8_t)253, (uint8_t)196, (uint8_t)75, (uint8_t)182, (uint8_t)173, (uint8_t)40, (uint8_t)34, (uint8_t)10, (uint8_t)153, (uint8_t)221, (uint8_t)51, (uint8_t)89, (uint8_t)30, (uint8_t)127, (uint8_t)236, (uint8_t)12, (uint8_t)209, (uint8_t)42, (uint8_t)252, (uint8_t)170, (uint8_t)150, (uint8_t)116, (uint8_t)14, (uint8_t)233, (uint8_t)45, (uint8_t)165, (uint8_t)253, (uint8_t)38, (uint8_t)172, (uint8_t)151, (uint8_t)82, (uint8_t)61, (uint8_t)59, (uint8_t)8, (uint8_t)38, (uint8_t)79, (uint8_t)192, (uint8_t)204, (uint8_t)159, (uint8_t)110, (uint8_t)83, (uint8_t)136, (uint8_t)153, (uint8_t)111, (uint8_t)242, (uint8_t)169, (uint8_t)162, (uint8_t)167, (uint8_t)119, (uint8_t)109, (uint8_t)178, (uint8_t)139, (uint8_t)255, (uint8_t)196, (uint8_t)159, (uint8_t)171, (uint8_t)36, (uint8_t)128, (uint8_t)99, (uint8_t)48, (uint8_t)69, (uint8_t)185, (uint8_t)57, (uint8_t)110, (uint8_t)125, (uint8_t)135, (uint8_t)155, (uint8_t)127, (uint8_t)239, (uint8_t)178, (uint8_t)19, (uint8_t)226, (uint8_t)215, (uint8_t)35, (uint8_t)86, (uint8_t)126, (uint8_t)105, (uint8_t)190, (uint8_t)235, (uint8_t)149, (uint8_t)172, (uint8_t)29, (uint8_t)166, (uint8_t)62, (uint8_t)36, (uint8_t)83, (uint8_t)105, (uint8_t)185, (uint8_t)63, (uint8_t)136};
            p248_payload_SET(&payload, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_V2_EXTENSION_248(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MEMORY_VECT_249(), &PH);
        p249_ver_SET((uint8_t)(uint8_t)28, PH.base.pack) ;
        {
            int8_t value[] =  {(int8_t)42, (int8_t)64, (int8_t)47, (int8_t) -24, (int8_t)9, (int8_t)37, (int8_t)127, (int8_t) -54, (int8_t) -102, (int8_t)44, (int8_t) -74, (int8_t)34, (int8_t)20, (int8_t)3, (int8_t)46, (int8_t)8, (int8_t) -52, (int8_t)43, (int8_t)4, (int8_t)125, (int8_t)96, (int8_t)120, (int8_t) -125, (int8_t) -22, (int8_t)3, (int8_t)3, (int8_t) -118, (int8_t)101, (int8_t)89, (int8_t)105, (int8_t) -91, (int8_t) -36};
            p249_value_SET(&value, 0, PH.base.pack) ;
        }
        p249_type_SET((uint8_t)(uint8_t)223, PH.base.pack) ;
        p249_address_SET((uint16_t)(uint16_t)42256, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MEMORY_VECT_249(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEBUG_VECT_250(), &PH);
        {
            char16_t* name = u"ATt";
            p250_name_SET_(name, &PH) ;
        }
        p250_z_SET((float) -3.2086602E38F, PH.base.pack) ;
        p250_time_usec_SET((uint64_t)5970517450518642378L, PH.base.pack) ;
        p250_y_SET((float) -2.8220634E38F, PH.base.pack) ;
        p250_x_SET((float)1.3761943E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEBUG_VECT_250(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_FLOAT_251(), &PH);
        p251_time_boot_ms_SET((uint32_t)1365170760L, PH.base.pack) ;
        {
            char16_t* name = u"fapmwrddiP";
            p251_name_SET_(name, &PH) ;
        }
        p251_value_SET((float)3.3286988E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_NAMED_VALUE_FLOAT_251(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_NAMED_VALUE_INT_252(), &PH);
        p252_time_boot_ms_SET((uint32_t)673680035L, PH.base.pack) ;
        p252_value_SET((int32_t) -2129142694, PH.base.pack) ;
        {
            char16_t* name = u"Wd";
            p252_name_SET_(name, &PH) ;
        }
        c_LoopBackDemoChannel_on_NAMED_VALUE_INT_252(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STATUSTEXT_253(), &PH);
        p253_severity_SET(e_MAV_SEVERITY_MAV_SEVERITY_NOTICE, PH.base.pack) ;
        {
            char16_t* text = u"eVjWvigTsenncknkxortlvxjlkvipzAuvhbqMuuxmxfj";
            p253_text_SET_(text, &PH) ;
        }
        c_LoopBackDemoChannel_on_STATUSTEXT_253(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_DEBUG_254(), &PH);
        p254_ind_SET((uint8_t)(uint8_t)137, PH.base.pack) ;
        p254_value_SET((float) -2.0567544E38F, PH.base.pack) ;
        p254_time_boot_ms_SET((uint32_t)3582531714L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_DEBUG_254(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SETUP_SIGNING_256(), &PH);
        p256_target_component_SET((uint8_t)(uint8_t)105, PH.base.pack) ;
        p256_initial_timestamp_SET((uint64_t)3481797220018744841L, PH.base.pack) ;
        p256_target_system_SET((uint8_t)(uint8_t)103, PH.base.pack) ;
        {
            uint8_t secret_key[] =  {(uint8_t)102, (uint8_t)236, (uint8_t)41, (uint8_t)128, (uint8_t)0, (uint8_t)149, (uint8_t)202, (uint8_t)182, (uint8_t)45, (uint8_t)22, (uint8_t)18, (uint8_t)10, (uint8_t)71, (uint8_t)118, (uint8_t)220, (uint8_t)169, (uint8_t)24, (uint8_t)104, (uint8_t)65, (uint8_t)185, (uint8_t)255, (uint8_t)126, (uint8_t)57, (uint8_t)202, (uint8_t)132, (uint8_t)62, (uint8_t)74, (uint8_t)186, (uint8_t)178, (uint8_t)14, (uint8_t)105, (uint8_t)187};
            p256_secret_key_SET(&secret_key, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_SETUP_SIGNING_256(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_BUTTON_CHANGE_257(), &PH);
        p257_state_SET((uint8_t)(uint8_t)128, PH.base.pack) ;
        p257_time_boot_ms_SET((uint32_t)1016514537L, PH.base.pack) ;
        p257_last_change_ms_SET((uint32_t)1435655979L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_BUTTON_CHANGE_257(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PLAY_TUNE_258(), &PH);
        p258_target_system_SET((uint8_t)(uint8_t)55, PH.base.pack) ;
        {
            char16_t* tune = u"qdkftxipeghdXdwdosKntrmhrllG";
            p258_tune_SET_(tune, &PH) ;
        }
        p258_target_component_SET((uint8_t)(uint8_t)181, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PLAY_TUNE_258(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_INFORMATION_259(), &PH);
        {
            uint8_t model_name[] =  {(uint8_t)186, (uint8_t)234, (uint8_t)164, (uint8_t)104, (uint8_t)3, (uint8_t)195, (uint8_t)161, (uint8_t)209, (uint8_t)155, (uint8_t)223, (uint8_t)40, (uint8_t)149, (uint8_t)25, (uint8_t)61, (uint8_t)139, (uint8_t)239, (uint8_t)221, (uint8_t)122, (uint8_t)82, (uint8_t)36, (uint8_t)42, (uint8_t)132, (uint8_t)181, (uint8_t)141, (uint8_t)179, (uint8_t)187, (uint8_t)166, (uint8_t)102, (uint8_t)79, (uint8_t)241, (uint8_t)202, (uint8_t)74};
            p259_model_name_SET(&model_name, 0, PH.base.pack) ;
        }
        p259_firmware_version_SET((uint32_t)173941620L, PH.base.pack) ;
        p259_time_boot_ms_SET((uint32_t)1821802759L, PH.base.pack) ;
        p259_flags_SET(e_CAMERA_CAP_FLAGS_CAMERA_CAP_FLAGS_CAPTURE_VIDEO, PH.base.pack) ;
        {
            char16_t* cam_definition_uri = u"htiuObNlzzkdzszPdqowbfhbuwfxfvzpspibtjscrbGhjxkiistktnonrubySbliLgxqezuxxtvp";
            p259_cam_definition_uri_SET_(cam_definition_uri, &PH) ;
        }
        p259_resolution_v_SET((uint16_t)(uint16_t)19651, PH.base.pack) ;
        {
            uint8_t vendor_name[] =  {(uint8_t)92, (uint8_t)168, (uint8_t)64, (uint8_t)30, (uint8_t)91, (uint8_t)88, (uint8_t)93, (uint8_t)69, (uint8_t)15, (uint8_t)58, (uint8_t)159, (uint8_t)124, (uint8_t)105, (uint8_t)2, (uint8_t)212, (uint8_t)93, (uint8_t)36, (uint8_t)207, (uint8_t)67, (uint8_t)253, (uint8_t)240, (uint8_t)228, (uint8_t)188, (uint8_t)93, (uint8_t)57, (uint8_t)199, (uint8_t)249, (uint8_t)191, (uint8_t)213, (uint8_t)97, (uint8_t)73, (uint8_t)254};
            p259_vendor_name_SET(&vendor_name, 0, PH.base.pack) ;
        }
        p259_lens_id_SET((uint8_t)(uint8_t)25, PH.base.pack) ;
        p259_sensor_size_h_SET((float) -2.2695404E38F, PH.base.pack) ;
        p259_cam_definition_version_SET((uint16_t)(uint16_t)30039, PH.base.pack) ;
        p259_resolution_h_SET((uint16_t)(uint16_t)37625, PH.base.pack) ;
        p259_sensor_size_v_SET((float)1.2195027E38F, PH.base.pack) ;
        p259_focal_length_SET((float)3.3853729E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_INFORMATION_259(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_SETTINGS_260(), &PH);
        p260_mode_id_SET(e_CAMERA_MODE_CAMERA_MODE_VIDEO, PH.base.pack) ;
        p260_time_boot_ms_SET((uint32_t)2543545987L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_SETTINGS_260(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_STORAGE_INFORMATION_261(), &PH);
        p261_available_capacity_SET((float) -1.8836227E38F, PH.base.pack) ;
        p261_used_capacity_SET((float) -1.5085219E38F, PH.base.pack) ;
        p261_write_speed_SET((float) -1.5468481E38F, PH.base.pack) ;
        p261_total_capacity_SET((float)2.5083615E38F, PH.base.pack) ;
        p261_storage_id_SET((uint8_t)(uint8_t)170, PH.base.pack) ;
        p261_storage_count_SET((uint8_t)(uint8_t)18, PH.base.pack) ;
        p261_status_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        p261_time_boot_ms_SET((uint32_t)2120671094L, PH.base.pack) ;
        p261_read_speed_SET((float)2.3645215E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_STORAGE_INFORMATION_261(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_CAPTURE_STATUS_262(), &PH);
        p262_recording_time_ms_SET((uint32_t)507410133L, PH.base.pack) ;
        p262_image_status_SET((uint8_t)(uint8_t)185, PH.base.pack) ;
        p262_video_status_SET((uint8_t)(uint8_t)212, PH.base.pack) ;
        p262_available_capacity_SET((float) -2.2314922E38F, PH.base.pack) ;
        p262_time_boot_ms_SET((uint32_t)3895567167L, PH.base.pack) ;
        p262_image_interval_SET((float) -4.4926627E37F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_CAPTURE_STATUS_262(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_CAMERA_IMAGE_CAPTURED_263(), &PH);
        {
            char16_t* file_url = u"ScsfrzukitEqkoUpedodkjrsvHHBgqqtnGomxctJymorPyVpKybwzqeskacxnxtbwyukdobqigiax";
            p263_file_url_SET_(file_url, &PH) ;
        }
        p263_time_utc_SET((uint64_t)9107741541208605445L, PH.base.pack) ;
        p263_lon_SET((int32_t)261890399, PH.base.pack) ;
        p263_image_index_SET((int32_t) -385716810, PH.base.pack) ;
        p263_time_boot_ms_SET((uint32_t)571428434L, PH.base.pack) ;
        p263_relative_alt_SET((int32_t)10281962, PH.base.pack) ;
        p263_lat_SET((int32_t) -193184169, PH.base.pack) ;
        p263_capture_result_SET((int8_t)(int8_t)79, PH.base.pack) ;
        {
            float q[] =  {-5.818606E37F, 3.0544504E38F, 1.4490953E38F, -9.109118E37F};
            p263_q_SET(&q, 0, PH.base.pack) ;
        }
        p263_alt_SET((int32_t) -1368935894, PH.base.pack) ;
        p263_camera_id_SET((uint8_t)(uint8_t)99, PH.base.pack) ;
        c_LoopBackDemoChannel_on_CAMERA_IMAGE_CAPTURED_263(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_FLIGHT_INFORMATION_264(), &PH);
        p264_arming_time_utc_SET((uint64_t)8841255504274704073L, PH.base.pack) ;
        p264_takeoff_time_utc_SET((uint64_t)7984116458012481603L, PH.base.pack) ;
        p264_flight_uuid_SET((uint64_t)3432743130748652848L, PH.base.pack) ;
        p264_time_boot_ms_SET((uint32_t)2961715268L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_FLIGHT_INFORMATION_264(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_MOUNT_ORIENTATION_265(), &PH);
        p265_yaw_SET((float) -3.2093696E37F, PH.base.pack) ;
        p265_roll_SET((float) -1.4558429E38F, PH.base.pack) ;
        p265_time_boot_ms_SET((uint32_t)2590057147L, PH.base.pack) ;
        p265_pitch_SET((float)3.3184495E38F, PH.base.pack) ;
        c_LoopBackDemoChannel_on_MOUNT_ORIENTATION_265(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_266(), &PH);
        p266_target_system_SET((uint8_t)(uint8_t)37, PH.base.pack) ;
        p266_length_SET((uint8_t)(uint8_t)19, PH.base.pack) ;
        p266_first_message_offset_SET((uint8_t)(uint8_t)42, PH.base.pack) ;
        p266_sequence_SET((uint16_t)(uint16_t)15284, PH.base.pack) ;
        p266_target_component_SET((uint8_t)(uint8_t)224, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)86, (uint8_t)30, (uint8_t)53, (uint8_t)192, (uint8_t)242, (uint8_t)34, (uint8_t)157, (uint8_t)2, (uint8_t)136, (uint8_t)241, (uint8_t)232, (uint8_t)230, (uint8_t)79, (uint8_t)207, (uint8_t)71, (uint8_t)158, (uint8_t)248, (uint8_t)123, (uint8_t)34, (uint8_t)22, (uint8_t)134, (uint8_t)245, (uint8_t)25, (uint8_t)79, (uint8_t)35, (uint8_t)246, (uint8_t)211, (uint8_t)60, (uint8_t)26, (uint8_t)0, (uint8_t)35, (uint8_t)130, (uint8_t)230, (uint8_t)151, (uint8_t)100, (uint8_t)87, (uint8_t)14, (uint8_t)111, (uint8_t)38, (uint8_t)221, (uint8_t)168, (uint8_t)240, (uint8_t)103, (uint8_t)99, (uint8_t)253, (uint8_t)8, (uint8_t)230, (uint8_t)121, (uint8_t)240, (uint8_t)73, (uint8_t)106, (uint8_t)211, (uint8_t)18, (uint8_t)141, (uint8_t)207, (uint8_t)164, (uint8_t)3, (uint8_t)133, (uint8_t)212, (uint8_t)71, (uint8_t)223, (uint8_t)165, (uint8_t)14, (uint8_t)48, (uint8_t)229, (uint8_t)99, (uint8_t)62, (uint8_t)255, (uint8_t)113, (uint8_t)131, (uint8_t)60, (uint8_t)57, (uint8_t)230, (uint8_t)119, (uint8_t)234, (uint8_t)90, (uint8_t)54, (uint8_t)119, (uint8_t)217, (uint8_t)84, (uint8_t)206, (uint8_t)150, (uint8_t)187, (uint8_t)93, (uint8_t)237, (uint8_t)130, (uint8_t)102, (uint8_t)176, (uint8_t)231, (uint8_t)17, (uint8_t)116, (uint8_t)187, (uint8_t)20, (uint8_t)89, (uint8_t)255, (uint8_t)33, (uint8_t)120, (uint8_t)205, (uint8_t)204, (uint8_t)222, (uint8_t)158, (uint8_t)182, (uint8_t)143, (uint8_t)218, (uint8_t)6, (uint8_t)235, (uint8_t)238, (uint8_t)80, (uint8_t)167, (uint8_t)183, (uint8_t)228, (uint8_t)195, (uint8_t)15, (uint8_t)220, (uint8_t)87, (uint8_t)171, (uint8_t)63, (uint8_t)189, (uint8_t)62, (uint8_t)208, (uint8_t)203, (uint8_t)242, (uint8_t)208, (uint8_t)166, (uint8_t)73, (uint8_t)101, (uint8_t)144, (uint8_t)121, (uint8_t)70, (uint8_t)140, (uint8_t)86, (uint8_t)108, (uint8_t)7, (uint8_t)4, (uint8_t)153, (uint8_t)124, (uint8_t)197, (uint8_t)156, (uint8_t)99, (uint8_t)186, (uint8_t)24, (uint8_t)113, (uint8_t)228, (uint8_t)154, (uint8_t)232, (uint8_t)139, (uint8_t)189, (uint8_t)20, (uint8_t)28, (uint8_t)102, (uint8_t)211, (uint8_t)134, (uint8_t)240, (uint8_t)166, (uint8_t)176, (uint8_t)210, (uint8_t)154, (uint8_t)132, (uint8_t)88, (uint8_t)26, (uint8_t)88, (uint8_t)85, (uint8_t)25, (uint8_t)31, (uint8_t)24, (uint8_t)101, (uint8_t)225, (uint8_t)135, (uint8_t)46, (uint8_t)112, (uint8_t)70, (uint8_t)80, (uint8_t)101, (uint8_t)169, (uint8_t)104, (uint8_t)77, (uint8_t)172, (uint8_t)100, (uint8_t)129, (uint8_t)23, (uint8_t)43, (uint8_t)13, (uint8_t)203, (uint8_t)33, (uint8_t)18, (uint8_t)82, (uint8_t)68, (uint8_t)74, (uint8_t)224, (uint8_t)111, (uint8_t)79, (uint8_t)61, (uint8_t)237, (uint8_t)159, (uint8_t)144, (uint8_t)222, (uint8_t)136, (uint8_t)7, (uint8_t)43, (uint8_t)86, (uint8_t)216, (uint8_t)169, (uint8_t)154, (uint8_t)234, (uint8_t)34, (uint8_t)162, (uint8_t)103, (uint8_t)116, (uint8_t)5, (uint8_t)162, (uint8_t)180, (uint8_t)211, (uint8_t)33, (uint8_t)44, (uint8_t)199, (uint8_t)254, (uint8_t)103, (uint8_t)147, (uint8_t)83, (uint8_t)233, (uint8_t)203, (uint8_t)31, (uint8_t)92, (uint8_t)243, (uint8_t)28, (uint8_t)157, (uint8_t)149, (uint8_t)233, (uint8_t)163, (uint8_t)142, (uint8_t)67, (uint8_t)25, (uint8_t)191, (uint8_t)234, (uint8_t)191, (uint8_t)19, (uint8_t)34, (uint8_t)231, (uint8_t)241, (uint8_t)126, (uint8_t)214, (uint8_t)255, (uint8_t)215, (uint8_t)186, (uint8_t)204, (uint8_t)117, (uint8_t)96, (uint8_t)255, (uint8_t)165};
            p266_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_LOGGING_DATA_266(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_DATA_ACKED_267(), &PH);
        p267_sequence_SET((uint16_t)(uint16_t)52064, PH.base.pack) ;
        p267_target_component_SET((uint8_t)(uint8_t)39, PH.base.pack) ;
        p267_length_SET((uint8_t)(uint8_t)32, PH.base.pack) ;
        p267_first_message_offset_SET((uint8_t)(uint8_t)158, PH.base.pack) ;
        p267_target_system_SET((uint8_t)(uint8_t)166, PH.base.pack) ;
        {
            uint8_t data_[] =  {(uint8_t)213, (uint8_t)206, (uint8_t)155, (uint8_t)77, (uint8_t)175, (uint8_t)157, (uint8_t)194, (uint8_t)142, (uint8_t)15, (uint8_t)180, (uint8_t)19, (uint8_t)46, (uint8_t)171, (uint8_t)137, (uint8_t)71, (uint8_t)222, (uint8_t)43, (uint8_t)194, (uint8_t)21, (uint8_t)51, (uint8_t)198, (uint8_t)249, (uint8_t)46, (uint8_t)186, (uint8_t)205, (uint8_t)118, (uint8_t)39, (uint8_t)228, (uint8_t)14, (uint8_t)115, (uint8_t)212, (uint8_t)221, (uint8_t)133, (uint8_t)120, (uint8_t)46, (uint8_t)53, (uint8_t)64, (uint8_t)214, (uint8_t)47, (uint8_t)140, (uint8_t)155, (uint8_t)41, (uint8_t)106, (uint8_t)99, (uint8_t)174, (uint8_t)153, (uint8_t)224, (uint8_t)179, (uint8_t)228, (uint8_t)131, (uint8_t)22, (uint8_t)29, (uint8_t)196, (uint8_t)167, (uint8_t)23, (uint8_t)231, (uint8_t)6, (uint8_t)72, (uint8_t)9, (uint8_t)174, (uint8_t)251, (uint8_t)126, (uint8_t)254, (uint8_t)156, (uint8_t)220, (uint8_t)124, (uint8_t)57, (uint8_t)172, (uint8_t)34, (uint8_t)158, (uint8_t)194, (uint8_t)35, (uint8_t)151, (uint8_t)187, (uint8_t)216, (uint8_t)83, (uint8_t)194, (uint8_t)152, (uint8_t)239, (uint8_t)192, (uint8_t)158, (uint8_t)83, (uint8_t)252, (uint8_t)243, (uint8_t)47, (uint8_t)33, (uint8_t)144, (uint8_t)201, (uint8_t)144, (uint8_t)160, (uint8_t)130, (uint8_t)20, (uint8_t)237, (uint8_t)152, (uint8_t)17, (uint8_t)71, (uint8_t)183, (uint8_t)247, (uint8_t)209, (uint8_t)213, (uint8_t)207, (uint8_t)68, (uint8_t)236, (uint8_t)99, (uint8_t)203, (uint8_t)173, (uint8_t)25, (uint8_t)102, (uint8_t)248, (uint8_t)148, (uint8_t)183, (uint8_t)147, (uint8_t)26, (uint8_t)34, (uint8_t)28, (uint8_t)124, (uint8_t)177, (uint8_t)83, (uint8_t)219, (uint8_t)136, (uint8_t)64, (uint8_t)27, (uint8_t)123, (uint8_t)109, (uint8_t)211, (uint8_t)200, (uint8_t)170, (uint8_t)111, (uint8_t)70, (uint8_t)2, (uint8_t)218, (uint8_t)33, (uint8_t)12, (uint8_t)69, (uint8_t)223, (uint8_t)176, (uint8_t)178, (uint8_t)193, (uint8_t)224, (uint8_t)3, (uint8_t)196, (uint8_t)73, (uint8_t)122, (uint8_t)145, (uint8_t)33, (uint8_t)140, (uint8_t)91, (uint8_t)16, (uint8_t)248, (uint8_t)241, (uint8_t)88, (uint8_t)48, (uint8_t)73, (uint8_t)144, (uint8_t)173, (uint8_t)234, (uint8_t)102, (uint8_t)240, (uint8_t)114, (uint8_t)196, (uint8_t)68, (uint8_t)187, (uint8_t)30, (uint8_t)137, (uint8_t)176, (uint8_t)85, (uint8_t)246, (uint8_t)152, (uint8_t)157, (uint8_t)8, (uint8_t)181, (uint8_t)207, (uint8_t)200, (uint8_t)51, (uint8_t)188, (uint8_t)181, (uint8_t)186, (uint8_t)183, (uint8_t)21, (uint8_t)226, (uint8_t)97, (uint8_t)157, (uint8_t)108, (uint8_t)24, (uint8_t)116, (uint8_t)190, (uint8_t)10, (uint8_t)144, (uint8_t)127, (uint8_t)41, (uint8_t)97, (uint8_t)173, (uint8_t)208, (uint8_t)111, (uint8_t)38, (uint8_t)102, (uint8_t)94, (uint8_t)219, (uint8_t)134, (uint8_t)234, (uint8_t)56, (uint8_t)164, (uint8_t)211, (uint8_t)67, (uint8_t)103, (uint8_t)192, (uint8_t)47, (uint8_t)144, (uint8_t)45, (uint8_t)77, (uint8_t)159, (uint8_t)212, (uint8_t)14, (uint8_t)70, (uint8_t)26, (uint8_t)216, (uint8_t)10, (uint8_t)179, (uint8_t)91, (uint8_t)126, (uint8_t)112, (uint8_t)84, (uint8_t)107, (uint8_t)74, (uint8_t)33, (uint8_t)204, (uint8_t)30, (uint8_t)130, (uint8_t)74, (uint8_t)207, (uint8_t)187, (uint8_t)133, (uint8_t)210, (uint8_t)201, (uint8_t)147, (uint8_t)118, (uint8_t)249, (uint8_t)10, (uint8_t)38, (uint8_t)251, (uint8_t)54, (uint8_t)31, (uint8_t)251, (uint8_t)162, (uint8_t)219, (uint8_t)54, (uint8_t)184, (uint8_t)131, (uint8_t)62};
            p267_data__SET(&data_, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_LOGGING_DATA_ACKED_267(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_LOGGING_ACK_268(), &PH);
        p268_target_component_SET((uint8_t)(uint8_t)17, PH.base.pack) ;
        p268_sequence_SET((uint16_t)(uint16_t)61111, PH.base.pack) ;
        p268_target_system_SET((uint8_t)(uint8_t)95, PH.base.pack) ;
        c_LoopBackDemoChannel_on_LOGGING_ACK_268(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_VIDEO_STREAM_INFORMATION_269(), &PH);
        p269_resolution_h_SET((uint16_t)(uint16_t)25320, PH.base.pack) ;
        p269_camera_id_SET((uint8_t)(uint8_t)27, PH.base.pack) ;
        p269_bitrate_SET((uint32_t)1991029241L, PH.base.pack) ;
        p269_status_SET((uint8_t)(uint8_t)190, PH.base.pack) ;
        {
            char16_t* uri = u"naYrwldy";
            p269_uri_SET_(uri, &PH) ;
        }
        p269_framerate_SET((float) -2.5839477E38F, PH.base.pack) ;
        p269_resolution_v_SET((uint16_t)(uint16_t)15833, PH.base.pack) ;
        p269_rotation_SET((uint16_t)(uint16_t)8314, PH.base.pack) ;
        c_LoopBackDemoChannel_on_VIDEO_STREAM_INFORMATION_269(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_SET_VIDEO_STREAM_SETTINGS_270(), &PH);
        p270_camera_id_SET((uint8_t)(uint8_t)15, PH.base.pack) ;
        p270_framerate_SET((float) -3.1695263E38F, PH.base.pack) ;
        p270_resolution_h_SET((uint16_t)(uint16_t)36819, PH.base.pack) ;
        {
            char16_t* uri = u"gqfvbLdvlwOklrmicvhrrbawxeptqkzwCyejpemcdyanemrcysntngkbidscafOnedxyjpqnUwareodifQlfhAVckornmvsgwhojwdsxjvuzsmdvqvqueodpelhvqzyNwycmdtMlsUpaXHgtuRb";
            p270_uri_SET_(uri, &PH) ;
        }
        p270_rotation_SET((uint16_t)(uint16_t)59475, PH.base.pack) ;
        p270_resolution_v_SET((uint16_t)(uint16_t)55514, PH.base.pack) ;
        p270_bitrate_SET((uint32_t)2452441953L, PH.base.pack) ;
        p270_target_system_SET((uint8_t)(uint8_t)182, PH.base.pack) ;
        p270_target_component_SET((uint8_t)(uint8_t)46, PH.base.pack) ;
        c_LoopBackDemoChannel_on_SET_VIDEO_STREAM_SETTINGS_270(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_WIFI_CONFIG_AP_299(), &PH);
        {
            char16_t* password = u"g";
            p299_password_SET_(password, &PH) ;
        }
        {
            char16_t* ssid = u"kpbGkmtoflvoXk";
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
            uint8_t library_version_hash[] =  {(uint8_t)217, (uint8_t)220, (uint8_t)208, (uint8_t)94, (uint8_t)233, (uint8_t)193, (uint8_t)84, (uint8_t)178};
            p300_library_version_hash_SET(&library_version_hash, 0, PH.base.pack) ;
        }
        p300_min_version_SET((uint16_t)(uint16_t)25249, PH.base.pack) ;
        {
            uint8_t spec_version_hash[] =  {(uint8_t)139, (uint8_t)91, (uint8_t)15, (uint8_t)2, (uint8_t)19, (uint8_t)226, (uint8_t)122, (uint8_t)232};
            p300_spec_version_hash_SET(&spec_version_hash, 0, PH.base.pack) ;
        }
        p300_max_version_SET((uint16_t)(uint16_t)8464, PH.base.pack) ;
        p300_version_SET((uint16_t)(uint16_t)6140, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PROTOCOL_VERSION_300(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_STATUS_310(), &PH);
        p310_vendor_specific_status_code_SET((uint16_t)(uint16_t)27328, PH.base.pack) ;
        p310_sub_mode_SET((uint8_t)(uint8_t)200, PH.base.pack) ;
        p310_mode_SET(e_UAVCAN_NODE_MODE_UAVCAN_NODE_MODE_INITIALIZATION, PH.base.pack) ;
        p310_time_usec_SET((uint64_t)1105401235972618759L, PH.base.pack) ;
        p310_uptime_sec_SET((uint32_t)1322284972L, PH.base.pack) ;
        p310_health_SET(e_UAVCAN_NODE_HEALTH_UAVCAN_NODE_HEALTH_OK, PH.base.pack) ;
        c_LoopBackDemoChannel_on_UAVCAN_NODE_STATUS_310(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_UAVCAN_NODE_INFO_311(), &PH);
        p311_sw_version_minor_SET((uint8_t)(uint8_t)12, PH.base.pack) ;
        p311_sw_version_major_SET((uint8_t)(uint8_t)80, PH.base.pack) ;
        p311_sw_vcs_commit_SET((uint32_t)3846086153L, PH.base.pack) ;
        p311_uptime_sec_SET((uint32_t)421995175L, PH.base.pack) ;
        p311_hw_version_major_SET((uint8_t)(uint8_t)136, PH.base.pack) ;
        {
            char16_t* name = u"shdjhaazryLwrbjalxkePthcgpsdintq";
            p311_name_SET_(name, &PH) ;
        }
        p311_hw_version_minor_SET((uint8_t)(uint8_t)81, PH.base.pack) ;
        p311_time_usec_SET((uint64_t)3775263283503279207L, PH.base.pack) ;
        {
            uint8_t hw_unique_id[] =  {(uint8_t)203, (uint8_t)223, (uint8_t)123, (uint8_t)101, (uint8_t)162, (uint8_t)216, (uint8_t)121, (uint8_t)228, (uint8_t)123, (uint8_t)248, (uint8_t)141, (uint8_t)103, (uint8_t)29, (uint8_t)107, (uint8_t)159, (uint8_t)201};
            p311_hw_unique_id_SET(&hw_unique_id, 0, PH.base.pack) ;
        }
        c_LoopBackDemoChannel_on_UAVCAN_NODE_INFO_311(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_READ_320(), &PH);
        {
            char16_t* param_id = u"vcpruzznsnyowdra";
            p320_param_id_SET_(param_id, &PH) ;
        }
        p320_target_system_SET((uint8_t)(uint8_t)43, PH.base.pack) ;
        p320_param_index_SET((int16_t)(int16_t)7035, PH.base.pack) ;
        p320_target_component_SET((uint8_t)(uint8_t)174, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_READ_320(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_REQUEST_LIST_321(), &PH);
        p321_target_component_SET((uint8_t)(uint8_t)240, PH.base.pack) ;
        p321_target_system_SET((uint8_t)(uint8_t)199, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_REQUEST_LIST_321(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_VALUE_322(), &PH);
        {
            char16_t* param_id = u"DjmyynrbaBj";
            p322_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"wgnvkeqqszhravndketozSkZzibsnteiDmjuVDhhozvzBcuwgHwopmkqejwytfjatvvwkVyhnsmbxVO";
            p322_param_value_SET_(param_value, &PH) ;
        }
        p322_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_INT32, PH.base.pack) ;
        p322_param_index_SET((uint16_t)(uint16_t)10487, PH.base.pack) ;
        p322_param_count_SET((uint16_t)(uint16_t)18228, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_VALUE_322(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_SET_323(), &PH);
        p323_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_REAL64, PH.base.pack) ;
        {
            char16_t* param_value = u"hdeIabyzgqlpLqKvYbolLYtxdxgcqcqdgayavgzxmniifafbIbsrcJljyvzxrjweqxyPq";
            p323_param_value_SET_(param_value, &PH) ;
        }
        p323_target_component_SET((uint8_t)(uint8_t)147, PH.base.pack) ;
        p323_target_system_SET((uint8_t)(uint8_t)179, PH.base.pack) ;
        {
            char16_t* param_id = u"hwygjszfnzjvqd";
            p323_param_id_SET_(param_id, &PH) ;
        }
        c_LoopBackDemoChannel_on_PARAM_EXT_SET_323(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_PARAM_EXT_ACK_324(), &PH);
        {
            char16_t* param_id = u"o";
            p324_param_id_SET_(param_id, &PH) ;
        }
        {
            char16_t* param_value = u"iaxvzmoeyaesqqrRonJxoktxzvotvnxjfswCjvjfysoddeezmnraqwwvbjfgoxJbigssyacdnztsQrsishdhpaezctduhAxXJzbvt";
            p324_param_value_SET_(param_value, &PH) ;
        }
        p324_param_result_SET(e_PARAM_ACK_PARAM_ACK_ACCEPTED, PH.base.pack) ;
        p324_param_type_SET(e_MAV_PARAM_EXT_TYPE_MAV_PARAM_EXT_TYPE_UINT32, PH.base.pack) ;
        c_LoopBackDemoChannel_on_PARAM_EXT_ACK_324(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
    {
        setPack(c_LoopBackDemoChannel_new_OBSTACLE_DISTANCE_330(), &PH);
        p330_min_distance_SET((uint16_t)(uint16_t)11235, PH.base.pack) ;
        p330_increment_SET((uint8_t)(uint8_t)198, PH.base.pack) ;
        p330_max_distance_SET((uint16_t)(uint16_t)7515, PH.base.pack) ;
        {
            uint16_t distances[] =  {(uint16_t)44137, (uint16_t)22125, (uint16_t)17735, (uint16_t)30030, (uint16_t)30990, (uint16_t)24675, (uint16_t)3851, (uint16_t)21313, (uint16_t)64085, (uint16_t)42614, (uint16_t)62643, (uint16_t)24231, (uint16_t)41072, (uint16_t)22788, (uint16_t)15953, (uint16_t)43146, (uint16_t)56697, (uint16_t)40202, (uint16_t)55668, (uint16_t)9643, (uint16_t)28182, (uint16_t)28758, (uint16_t)11867, (uint16_t)14552, (uint16_t)63865, (uint16_t)30745, (uint16_t)24501, (uint16_t)24259, (uint16_t)56108, (uint16_t)7588, (uint16_t)17244, (uint16_t)59621, (uint16_t)38986, (uint16_t)18618, (uint16_t)33004, (uint16_t)27135, (uint16_t)6193, (uint16_t)52674, (uint16_t)6156, (uint16_t)27581, (uint16_t)48442, (uint16_t)31811, (uint16_t)41163, (uint16_t)60429, (uint16_t)7264, (uint16_t)47044, (uint16_t)12372, (uint16_t)17853, (uint16_t)37227, (uint16_t)16270, (uint16_t)978, (uint16_t)16221, (uint16_t)53054, (uint16_t)11867, (uint16_t)54521, (uint16_t)8797, (uint16_t)10609, (uint16_t)44340, (uint16_t)44575, (uint16_t)13799, (uint16_t)47023, (uint16_t)21845, (uint16_t)2869, (uint16_t)11312, (uint16_t)47512, (uint16_t)59432, (uint16_t)30195, (uint16_t)21719, (uint16_t)58219, (uint16_t)33219, (uint16_t)9277, (uint16_t)25077};
            p330_distances_SET(&distances, 0, PH.base.pack) ;
        }
        p330_sensor_type_SET(e_MAV_DISTANCE_SENSOR_MAV_DISTANCE_SENSOR_INFRARED, PH.base.pack) ;
        p330_time_usec_SET((uint64_t)6911494538107067992L, PH.base.pack) ;
        c_LoopBackDemoChannel_on_OBSTACLE_DISTANCE_330(&PH, PH.base.pack); //direct test.
        c_LoopBackDemoChannel_send(PH.base.pack); //put into the sender send-buffer
        for(uint32_t len; (len = c_LoopBackDemoChannel_input_bytes(buff, sizeof buff));) c_LoopBackDemoChannel_output_bytes(buff,  len);
        c_LoopBackDemoChannel_process_received();// process received pack on receiver side
    }
}

