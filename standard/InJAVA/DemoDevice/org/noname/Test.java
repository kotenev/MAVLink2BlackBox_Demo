
package org.noname;
import java.util.*;
import static org.unirail.BlackBox.BitUtils.*;
import org.unirail.BlackBox.Host.Pack.Meta.Field.Bounds;
import java.io.IOException;

public class Test extends DemoDevice
{




    static class TestChannel extends Channel
    {
        static final TestChannel instance = new TestChannel(); //test channel

        public final java.io.InputStream  inputStream          = new InputStream();
        public final java.io.OutputStream outputStream         = new OutputStream();
        public final java.io.InputStream  inputStreamAdvanced  = new AdvancedInputStream();
        public final java.io.OutputStream outputStreamAdvanced = new AdvancedOutputStream();

        @Override protected void failure(String reason)
        {
            super.failure(reason);
            assert(false);
        }



        static Pack testing_pack; //one pack send/receive buffer

        void send(Pack pack) { testing_pack = pack; }

        final Bounds.Inside ph = new Bounds.Inside();

        @Override protected Pack process(Pack pack, int id)
        {
            switch(id)
            {
                case Channel.PROCESS_CHANNEL_REQEST:
                    if(pack == null)
                    {
                        pack = testing_pack;
                        testing_pack = null;
                    }
                    else testing_pack = pack;
                    return pack;
                case Channel.PROCESS_RECEIVED:
                    if(testing_pack == null) return null;
                    ph.setPack(pack = testing_pack);
                    testing_pack = null;
                    id = pack.meta.id;
            }
            switch(id)
            {
            }
            return null;
        }
        static final byte[] buff = new byte[1024];
        static void transmission(java.io.InputStream src, java.io.OutputStream dst, Channel dst_ch)
        {
            try
            {
                if(src instanceof AdvancedInputStream && !(dst instanceof AdvancedOutputStream))
                {
                    for(int bytes; 0 < (bytes = src.read(buff, 0, buff.length));) TestChannel.instance.outputStreamAdvanced.write(buff, 0, bytes);
                    for(int bytes; 0 < (bytes = TestChannel.instance.inputStream.read(buff, 0, buff.length));) dst.write(buff, 0, bytes);
                }
                else if(!(src instanceof AdvancedInputStream) && dst instanceof AdvancedOutputStream)
                {
                    for(int bytes; 0 < (bytes = src.read(buff, 0, buff.length));) TestChannel.instance.outputStream.write(buff, 0, bytes);
                    for(int bytes; 0 < (bytes = TestChannel.instance.inputStreamAdvanced.read(buff, 0, buff.length));) dst.write(buff, 0, bytes);
                }
                else
                    for(int bytes; 0 < (bytes = src.read(buff, 0, buff.length));) dst.write(buff, 0, bytes);
                processReceived(dst_ch);
            }
            catch(IOException e)
            {
                e.printStackTrace();
                assert(false);
            }
        }
    }

    public static void main(String[] args)
    {
        final Bounds.Inside PH = new Bounds.Inside();
        LoopBackDemoChannel.instance.on_HEARTBEAT.add((src, ph, pack) ->
        {
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_FP);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_ACTIVE);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_VTOL_RESERVED4);
            assert(pack.custom_mode_GET() == 3805221701L);
            assert(pack.mavlink_version_GET() == (char)201);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
        });
        DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
        PH.setPack(p0);
        p0.custom_mode_SET(3805221701L) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_ACTIVE) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_VTOL_RESERVED4) ;
        p0.mavlink_version_SET((char)201) ;
        p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_FP) ;
        LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.onboard_control_sensors_health_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2);
            assert(pack.current_battery_GET() == (short) -7447);
            assert(pack.battery_remaining_GET() == (byte) - 58);
            assert(pack.load_GET() == (char)9645);
            assert(pack.errors_comm_GET() == (char)56591);
            assert(pack.errors_count3_GET() == (char)62001);
            assert(pack.errors_count1_GET() == (char)29566);
            assert(pack.errors_count2_GET() == (char)26467);
            assert(pack.onboard_control_sensors_enabled_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2);
            assert(pack.onboard_control_sensors_present_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH);
            assert(pack.errors_count4_GET() == (char)3302);
            assert(pack.drop_rate_comm_GET() == (char)51577);
            assert(pack.voltage_battery_GET() == (char)27498);
        });
        DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
        PH.setPack(p1);
        p1.errors_count2_SET((char)26467) ;
        p1.voltage_battery_SET((char)27498) ;
        p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH) ;
        p1.errors_count1_SET((char)29566) ;
        p1.errors_count3_SET((char)62001) ;
        p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_MAG2) ;
        p1.load_SET((char)9645) ;
        p1.battery_remaining_SET((byte) - 58) ;
        p1.errors_count4_SET((char)3302) ;
        p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_ACCEL2) ;
        p1.current_battery_SET((short) -7447) ;
        p1.drop_rate_comm_SET((char)51577) ;
        p1.errors_comm_SET((char)56591) ;
        LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2145861494L);
            assert(pack.time_unix_usec_GET() == 5275850431535498822L);
        });
        DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_boot_ms_SET(2145861494L) ;
        p2.time_unix_usec_SET(5275850431535498822L) ;
        LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.afy_GET() == 2.884754E38F);
            assert(pack.x_GET() == 1.914076E38F);
            assert(pack.yaw_GET() == -6.9166226E37F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.type_mask_GET() == (char)15852);
            assert(pack.z_GET() == -1.044241E38F);
            assert(pack.time_boot_ms_GET() == 2222079929L);
            assert(pack.afz_GET() == 8.967768E37F);
            assert(pack.vz_GET() == -2.806633E38F);
            assert(pack.yaw_rate_GET() == 2.8575562E38F);
            assert(pack.vy_GET() == -1.8676651E38F);
            assert(pack.y_GET() == 3.3051274E38F);
            assert(pack.afx_GET() == 1.713782E38F);
            assert(pack.vx_GET() == 2.7597855E38F);
        });
        DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.vz_SET(-2.806633E38F) ;
        p3.x_SET(1.914076E38F) ;
        p3.yaw_rate_SET(2.8575562E38F) ;
        p3.time_boot_ms_SET(2222079929L) ;
        p3.y_SET(3.3051274E38F) ;
        p3.type_mask_SET((char)15852) ;
        p3.afx_SET(1.713782E38F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p3.vx_SET(2.7597855E38F) ;
        p3.z_SET(-1.044241E38F) ;
        p3.afz_SET(8.967768E37F) ;
        p3.vy_SET(-1.8676651E38F) ;
        p3.afy_SET(2.884754E38F) ;
        p3.yaw_SET(-6.9166226E37F) ;
        LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 832013728L);
            assert(pack.target_system_GET() == (char)32);
            assert(pack.time_usec_GET() == 7575679969961943803L);
            assert(pack.target_component_GET() == (char)204);
        });
        DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
        PH.setPack(p4);
        p4.time_usec_SET(7575679969961943803L) ;
        p4.seq_SET(832013728L) ;
        p4.target_system_SET((char)32) ;
        p4.target_component_SET((char)204) ;
        LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.control_request_GET() == (char)97);
            assert(pack.version_GET() == (char)61);
            assert(pack.passkey_LEN(ph) == 9);
            assert(pack.passkey_TRY(ph).equals("aapcepMni"));
            assert(pack.target_system_GET() == (char)22);
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.passkey_SET("aapcepMni", PH) ;
        p5.target_system_SET((char)22) ;
        p5.control_request_SET((char)97) ;
        p5.version_SET((char)61) ;
        LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.gcs_system_id_GET() == (char)35);
            assert(pack.ack_GET() == (char)30);
            assert(pack.control_request_GET() == (char)143);
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.gcs_system_id_SET((char)35) ;
        p6.control_request_SET((char)143) ;
        p6.ack_SET((char)30) ;
        LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 23);
            assert(pack.key_TRY(ph).equals("cwnpVtnahzUmzuaaNkccanr"));
        });
        DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("cwnpVtnahzUmzuaaNkccanr", PH) ;
        LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.custom_mode_GET() == 26053649L);
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_AUTO_DISARMED);
            assert(pack.target_system_GET() == (char)200);
        });
        DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
        PH.setPack(p11);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_AUTO_DISARMED) ;
        p11.custom_mode_SET(26053649L) ;
        p11.target_system_SET((char)200) ;
        LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)96);
            assert(pack.param_index_GET() == (short)2066);
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("d"));
            assert(pack.target_component_GET() == (char)230);
        });
        DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_component_SET((char)230) ;
        p20.param_id_SET("d", PH) ;
        p20.target_system_SET((char)96) ;
        p20.param_index_SET((short)2066) ;
        LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)168);
            assert(pack.target_component_GET() == (char)147);
        });
        DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_system_SET((char)168) ;
        p21.target_component_SET((char)147) ;
        LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (char)48261);
            assert(pack.param_count_GET() == (char)13163);
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("ul"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
            assert(pack.param_value_GET() == 1.301755E38F);
        });
        DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
        PH.setPack(p22);
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16) ;
        p22.param_value_SET(1.301755E38F) ;
        p22.param_count_SET((char)13163) ;
        p22.param_index_SET((char)48261) ;
        p22.param_id_SET("ul", PH) ;
        LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)193);
            assert(pack.target_component_GET() == (char)0);
            assert(pack.param_value_GET() == -8.296227E37F);
            assert(pack.param_id_LEN(ph) == 12);
            assert(pack.param_id_TRY(ph).equals("DzzrvUvbtzvh"));
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
        });
        DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
        PH.setPack(p23);
        p23.target_system_SET((char)193) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16) ;
        p23.target_component_SET((char)0) ;
        p23.param_id_SET("DzzrvUvbtzvh", PH) ;
        p23.param_value_SET(-8.296227E37F) ;
        LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.epv_GET() == (char)36570);
            assert(pack.h_acc_TRY(ph) == 3217028659L);
            assert(pack.alt_ellipsoid_TRY(ph) == 2058099807);
            assert(pack.vel_GET() == (char)37013);
            assert(pack.alt_GET() == -1062481088);
            assert(pack.time_usec_GET() == 9063215223531905220L);
            assert(pack.eph_GET() == (char)24823);
            assert(pack.satellites_visible_GET() == (char)46);
            assert(pack.cog_GET() == (char)25230);
            assert(pack.lat_GET() == -336384331);
            assert(pack.hdg_acc_TRY(ph) == 2629007100L);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
            assert(pack.vel_acc_TRY(ph) == 1691530922L);
            assert(pack.lon_GET() == 1075405562);
            assert(pack.v_acc_TRY(ph) == 68892246L);
        });
        DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
        PH.setPack(p24);
        p24.satellites_visible_SET((char)46) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT) ;
        p24.alt_SET(-1062481088) ;
        p24.lat_SET(-336384331) ;
        p24.time_usec_SET(9063215223531905220L) ;
        p24.eph_SET((char)24823) ;
        p24.alt_ellipsoid_SET(2058099807, PH) ;
        p24.h_acc_SET(3217028659L, PH) ;
        p24.epv_SET((char)36570) ;
        p24.lon_SET(1075405562) ;
        p24.vel_SET((char)37013) ;
        p24.v_acc_SET(68892246L, PH) ;
        p24.vel_acc_SET(1691530922L, PH) ;
        p24.cog_SET((char)25230) ;
        p24.hdg_acc_SET(2629007100L, PH) ;
        LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)69, (char)97, (char)28, (char)56, (char)213, (char)226, (char)133, (char)198, (char)29, (char)211, (char)73, (char)85, (char)171, (char)137, (char)165, (char)46, (char)108, (char)208, (char)194, (char)86}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)54, (char)168, (char)76, (char)167, (char)43, (char)242, (char)109, (char)203, (char)163, (char)35, (char)204, (char)148, (char)209, (char)4, (char)74, (char)70, (char)19, (char)113, (char)78, (char)251}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)114, (char)87, (char)158, (char)38, (char)175, (char)207, (char)80, (char)90, (char)33, (char)117, (char)24, (char)116, (char)56, (char)67, (char)51, (char)11, (char)92, (char)151, (char)58, (char)118}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)136, (char)235, (char)108, (char)124, (char)207, (char)93, (char)119, (char)1, (char)70, (char)40, (char)80, (char)32, (char)125, (char)65, (char)150, (char)139, (char)9, (char)15, (char)246, (char)33}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)251, (char)163, (char)94, (char)222, (char)193, (char)161, (char)97, (char)205, (char)71, (char)227, (char)167, (char)132, (char)105, (char)251, (char)135, (char)242, (char)204, (char)114, (char)155, (char)185}));
            assert(pack.satellites_visible_GET() == (char)236);
        });
        DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_elevation_SET(new char[] {(char)114, (char)87, (char)158, (char)38, (char)175, (char)207, (char)80, (char)90, (char)33, (char)117, (char)24, (char)116, (char)56, (char)67, (char)51, (char)11, (char)92, (char)151, (char)58, (char)118}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)54, (char)168, (char)76, (char)167, (char)43, (char)242, (char)109, (char)203, (char)163, (char)35, (char)204, (char)148, (char)209, (char)4, (char)74, (char)70, (char)19, (char)113, (char)78, (char)251}, 0) ;
        p25.satellites_visible_SET((char)236) ;
        p25.satellite_used_SET(new char[] {(char)251, (char)163, (char)94, (char)222, (char)193, (char)161, (char)97, (char)205, (char)71, (char)227, (char)167, (char)132, (char)105, (char)251, (char)135, (char)242, (char)204, (char)114, (char)155, (char)185}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)136, (char)235, (char)108, (char)124, (char)207, (char)93, (char)119, (char)1, (char)70, (char)40, (char)80, (char)32, (char)125, (char)65, (char)150, (char)139, (char)9, (char)15, (char)246, (char)33}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)69, (char)97, (char)28, (char)56, (char)213, (char)226, (char)133, (char)198, (char)29, (char)211, (char)73, (char)85, (char)171, (char)137, (char)165, (char)46, (char)108, (char)208, (char)194, (char)86}, 0) ;
        LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.xgyro_GET() == (short) -6678);
            assert(pack.zmag_GET() == (short) -9852);
            assert(pack.ymag_GET() == (short) -6897);
            assert(pack.time_boot_ms_GET() == 2909515549L);
            assert(pack.zacc_GET() == (short)28294);
            assert(pack.yacc_GET() == (short) -900);
            assert(pack.zgyro_GET() == (short)12975);
            assert(pack.ygyro_GET() == (short) -2745);
            assert(pack.xmag_GET() == (short) -4291);
            assert(pack.xacc_GET() == (short) -15106);
        });
        DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
        PH.setPack(p26);
        p26.xmag_SET((short) -4291) ;
        p26.yacc_SET((short) -900) ;
        p26.zacc_SET((short)28294) ;
        p26.zmag_SET((short) -9852) ;
        p26.time_boot_ms_SET(2909515549L) ;
        p26.ygyro_SET((short) -2745) ;
        p26.xgyro_SET((short) -6678) ;
        p26.zgyro_SET((short)12975) ;
        p26.ymag_SET((short) -6897) ;
        p26.xacc_SET((short) -15106) ;
        LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short)29807);
            assert(pack.time_usec_GET() == 2540247634773735718L);
            assert(pack.zacc_GET() == (short) -27648);
            assert(pack.zmag_GET() == (short) -5511);
            assert(pack.zgyro_GET() == (short)22766);
            assert(pack.ygyro_GET() == (short)14771);
            assert(pack.ymag_GET() == (short)15296);
            assert(pack.yacc_GET() == (short)15405);
            assert(pack.xgyro_GET() == (short) -25970);
            assert(pack.xmag_GET() == (short) -25268);
        });
        DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
        PH.setPack(p27);
        p27.xmag_SET((short) -25268) ;
        p27.time_usec_SET(2540247634773735718L) ;
        p27.xgyro_SET((short) -25970) ;
        p27.zgyro_SET((short)22766) ;
        p27.zmag_SET((short) -5511) ;
        p27.yacc_SET((short)15405) ;
        p27.zacc_SET((short) -27648) ;
        p27.ymag_SET((short)15296) ;
        p27.xacc_SET((short)29807) ;
        p27.ygyro_SET((short)14771) ;
        LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5749347770934351370L);
            assert(pack.press_abs_GET() == (short)14287);
            assert(pack.press_diff2_GET() == (short)22533);
            assert(pack.temperature_GET() == (short)748);
            assert(pack.press_diff1_GET() == (short)28364);
        });
        DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
        PH.setPack(p28);
        p28.press_diff1_SET((short)28364) ;
        p28.press_diff2_SET((short)22533) ;
        p28.time_usec_SET(5749347770934351370L) ;
        p28.temperature_SET((short)748) ;
        p28.press_abs_SET((short)14287) ;
        LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == 1.6459424E38F);
            assert(pack.temperature_GET() == (short) -13034);
            assert(pack.press_diff_GET() == -2.2195327E38F);
            assert(pack.time_boot_ms_GET() == 889382766L);
        });
        DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_diff_SET(-2.2195327E38F) ;
        p29.temperature_SET((short) -13034) ;
        p29.time_boot_ms_SET(889382766L) ;
        p29.press_abs_SET(1.6459424E38F) ;
        LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.pitchspeed_GET() == -1.3019061E38F);
            assert(pack.pitch_GET() == 1.9665027E38F);
            assert(pack.time_boot_ms_GET() == 3026019434L);
            assert(pack.yawspeed_GET() == 7.5903156E37F);
            assert(pack.yaw_GET() == 3.385265E37F);
            assert(pack.roll_GET() == 5.562962E36F);
            assert(pack.rollspeed_GET() == -2.2361298E38F);
        });
        DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
        PH.setPack(p30);
        p30.pitchspeed_SET(-1.3019061E38F) ;
        p30.pitch_SET(1.9665027E38F) ;
        p30.yawspeed_SET(7.5903156E37F) ;
        p30.rollspeed_SET(-2.2361298E38F) ;
        p30.yaw_SET(3.385265E37F) ;
        p30.roll_SET(5.562962E36F) ;
        p30.time_boot_ms_SET(3026019434L) ;
        LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == 2.2110124E38F);
            assert(pack.q4_GET() == -2.0773145E38F);
            assert(pack.time_boot_ms_GET() == 133765639L);
            assert(pack.q2_GET() == -8.575923E37F);
            assert(pack.pitchspeed_GET() == 8.542965E37F);
            assert(pack.q1_GET() == -3.3882122E38F);
            assert(pack.rollspeed_GET() == 3.2185215E37F);
            assert(pack.q3_GET() == 1.3473564E38F);
        });
        DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.pitchspeed_SET(8.542965E37F) ;
        p31.q1_SET(-3.3882122E38F) ;
        p31.q2_SET(-8.575923E37F) ;
        p31.q4_SET(-2.0773145E38F) ;
        p31.q3_SET(1.3473564E38F) ;
        p31.time_boot_ms_SET(133765639L) ;
        p31.yawspeed_SET(2.2110124E38F) ;
        p31.rollspeed_SET(3.2185215E37F) ;
        LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 4240345149L);
            assert(pack.y_GET() == 1.5521299E38F);
            assert(pack.x_GET() == -9.13191E37F);
            assert(pack.vx_GET() == -2.5458846E37F);
            assert(pack.z_GET() == -1.5077686E38F);
            assert(pack.vz_GET() == 2.0878692E38F);
            assert(pack.vy_GET() == 2.9563506E38F);
        });
        DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.x_SET(-9.13191E37F) ;
        p32.vz_SET(2.0878692E38F) ;
        p32.vx_SET(-2.5458846E37F) ;
        p32.vy_SET(2.9563506E38F) ;
        p32.z_SET(-1.5077686E38F) ;
        p32.y_SET(1.5521299E38F) ;
        p32.time_boot_ms_SET(4240345149L) ;
        LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == (short)2131);
            assert(pack.lat_GET() == -1173727462);
            assert(pack.vy_GET() == (short) -23119);
            assert(pack.lon_GET() == -1541404097);
            assert(pack.vz_GET() == (short)619);
            assert(pack.hdg_GET() == (char)63353);
            assert(pack.alt_GET() == 131419372);
            assert(pack.relative_alt_GET() == 914880746);
            assert(pack.time_boot_ms_GET() == 2998708376L);
        });
        DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.lat_SET(-1173727462) ;
        p33.time_boot_ms_SET(2998708376L) ;
        p33.vz_SET((short)619) ;
        p33.vy_SET((short) -23119) ;
        p33.vx_SET((short)2131) ;
        p33.hdg_SET((char)63353) ;
        p33.alt_SET(131419372) ;
        p33.lon_SET(-1541404097) ;
        p33.relative_alt_SET(914880746) ;
        LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan8_scaled_GET() == (short)53);
            assert(pack.chan6_scaled_GET() == (short)20594);
            assert(pack.chan2_scaled_GET() == (short)17480);
            assert(pack.chan5_scaled_GET() == (short) -593);
            assert(pack.chan7_scaled_GET() == (short) -1451);
            assert(pack.rssi_GET() == (char)253);
            assert(pack.port_GET() == (char)154);
            assert(pack.chan1_scaled_GET() == (short)26954);
            assert(pack.chan4_scaled_GET() == (short) -3008);
            assert(pack.time_boot_ms_GET() == 1312703849L);
            assert(pack.chan3_scaled_GET() == (short)4567);
        });
        DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan8_scaled_SET((short)53) ;
        p34.chan2_scaled_SET((short)17480) ;
        p34.chan3_scaled_SET((short)4567) ;
        p34.rssi_SET((char)253) ;
        p34.chan7_scaled_SET((short) -1451) ;
        p34.chan4_scaled_SET((short) -3008) ;
        p34.chan6_scaled_SET((short)20594) ;
        p34.port_SET((char)154) ;
        p34.chan5_scaled_SET((short) -593) ;
        p34.time_boot_ms_SET(1312703849L) ;
        p34.chan1_scaled_SET((short)26954) ;
        LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan3_raw_GET() == (char)61623);
            assert(pack.chan1_raw_GET() == (char)55026);
            assert(pack.chan6_raw_GET() == (char)34304);
            assert(pack.time_boot_ms_GET() == 3983707863L);
            assert(pack.chan5_raw_GET() == (char)1132);
            assert(pack.port_GET() == (char)44);
            assert(pack.chan2_raw_GET() == (char)34038);
            assert(pack.chan4_raw_GET() == (char)62394);
            assert(pack.chan8_raw_GET() == (char)52304);
            assert(pack.chan7_raw_GET() == (char)43518);
            assert(pack.rssi_GET() == (char)252);
        });
        DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan1_raw_SET((char)55026) ;
        p35.rssi_SET((char)252) ;
        p35.chan2_raw_SET((char)34038) ;
        p35.chan4_raw_SET((char)62394) ;
        p35.chan5_raw_SET((char)1132) ;
        p35.chan6_raw_SET((char)34304) ;
        p35.chan7_raw_SET((char)43518) ;
        p35.port_SET((char)44) ;
        p35.chan8_raw_SET((char)52304) ;
        p35.chan3_raw_SET((char)61623) ;
        p35.time_boot_ms_SET(3983707863L) ;
        LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo11_raw_TRY(ph) == (char)42962);
            assert(pack.servo13_raw_TRY(ph) == (char)39099);
            assert(pack.servo16_raw_TRY(ph) == (char)2164);
            assert(pack.time_usec_GET() == 358481387L);
            assert(pack.servo5_raw_GET() == (char)50929);
            assert(pack.servo7_raw_GET() == (char)30896);
            assert(pack.servo3_raw_GET() == (char)48356);
            assert(pack.port_GET() == (char)132);
            assert(pack.servo1_raw_GET() == (char)60868);
            assert(pack.servo9_raw_TRY(ph) == (char)32285);
            assert(pack.servo15_raw_TRY(ph) == (char)47832);
            assert(pack.servo14_raw_TRY(ph) == (char)60890);
            assert(pack.servo8_raw_GET() == (char)32860);
            assert(pack.servo4_raw_GET() == (char)28540);
            assert(pack.servo10_raw_TRY(ph) == (char)33385);
            assert(pack.servo12_raw_TRY(ph) == (char)32564);
            assert(pack.servo2_raw_GET() == (char)23795);
            assert(pack.servo6_raw_GET() == (char)40619);
        });
        DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo6_raw_SET((char)40619) ;
        p36.servo1_raw_SET((char)60868) ;
        p36.servo14_raw_SET((char)60890, PH) ;
        p36.servo5_raw_SET((char)50929) ;
        p36.servo11_raw_SET((char)42962, PH) ;
        p36.time_usec_SET(358481387L) ;
        p36.servo7_raw_SET((char)30896) ;
        p36.servo13_raw_SET((char)39099, PH) ;
        p36.servo10_raw_SET((char)33385, PH) ;
        p36.servo2_raw_SET((char)23795) ;
        p36.servo8_raw_SET((char)32860) ;
        p36.servo12_raw_SET((char)32564, PH) ;
        p36.servo15_raw_SET((char)47832, PH) ;
        p36.servo9_raw_SET((char)32285, PH) ;
        p36.servo3_raw_SET((char)48356) ;
        p36.servo4_raw_SET((char)28540) ;
        p36.port_SET((char)132) ;
        p36.servo16_raw_SET((char)2164, PH) ;
        LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.end_index_GET() == (short) -3025);
            assert(pack.target_component_GET() == (char)3);
            assert(pack.target_system_GET() == (char)170);
            assert(pack.start_index_GET() == (short) -20048);
        });
        DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.start_index_SET((short) -20048) ;
        p37.end_index_SET((short) -3025) ;
        p37.target_component_SET((char)3) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p37.target_system_SET((char)170) ;
        LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_index_GET() == (short)23770);
            assert(pack.end_index_GET() == (short) -7154);
            assert(pack.target_system_GET() == (char)103);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_component_GET() == (char)155);
        });
        DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p38.start_index_SET((short)23770) ;
        p38.end_index_SET((short) -7154) ;
        p38.target_component_SET((char)155) ;
        p38.target_system_SET((char)103) ;
        LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 1.7714482E38F);
            assert(pack.param1_GET() == -3.2640614E38F);
            assert(pack.param3_GET() == 5.0103373E37F);
            assert(pack.param2_GET() == -1.5690046E38F);
            assert(pack.autocontinue_GET() == (char)23);
            assert(pack.z_GET() == 3.58761E37F);
            assert(pack.target_system_GET() == (char)248);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_SET_ROI);
            assert(pack.param4_GET() == -2.775834E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.current_GET() == (char)45);
            assert(pack.target_component_GET() == (char)34);
            assert(pack.seq_GET() == (char)43928);
            assert(pack.x_GET() == -1.0407065E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
        });
        DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
        PH.setPack(p39);
        p39.param1_SET(-3.2640614E38F) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p39.param2_SET(-1.5690046E38F) ;
        p39.target_component_SET((char)34) ;
        p39.y_SET(1.7714482E38F) ;
        p39.z_SET(3.58761E37F) ;
        p39.current_SET((char)45) ;
        p39.command_SET(MAV_CMD.MAV_CMD_DO_SET_ROI) ;
        p39.seq_SET((char)43928) ;
        p39.param3_SET(5.0103373E37F) ;
        p39.param4_SET(-2.775834E38F) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p39.target_system_SET((char)248) ;
        p39.x_SET(-1.0407065E38F) ;
        p39.autocontinue_SET((char)23) ;
        LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)88);
            assert(pack.seq_GET() == (char)45391);
            assert(pack.target_system_GET() == (char)178);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_system_SET((char)178) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p40.seq_SET((char)45391) ;
        p40.target_component_SET((char)88) ;
        LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)134);
            assert(pack.target_component_GET() == (char)206);
            assert(pack.seq_GET() == (char)40606);
        });
        DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.seq_SET((char)40606) ;
        p41.target_component_SET((char)206) ;
        p41.target_system_SET((char)134) ;
        LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)33013);
        });
        DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)33013) ;
        LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_component_GET() == (char)65);
            assert(pack.target_system_GET() == (char)177);
        });
        DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_component_SET((char)65) ;
        p43.target_system_SET((char)177) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)124);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)176);
            assert(pack.count_GET() == (char)61078);
        });
        DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
        PH.setPack(p44);
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p44.target_system_SET((char)176) ;
        p44.count_SET((char)61078) ;
        p44.target_component_SET((char)124) ;
        LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.target_system_GET() == (char)130);
            assert(pack.target_component_GET() == (char)156);
        });
        DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p45.target_component_SET((char)156) ;
        p45.target_system_SET((char)130) ;
        LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)33125);
        });
        DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)33125) ;
        LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_component_GET() == (char)143);
            assert(pack.target_system_GET() == (char)47);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM6_Y);
        });
        DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
        PH.setPack(p47);
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM6_Y) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p47.target_system_SET((char)47) ;
        p47.target_component_SET((char)143) ;
        LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 5084116729189478812L);
            assert(pack.altitude_GET() == -607279784);
            assert(pack.longitude_GET() == -701705904);
            assert(pack.target_system_GET() == (char)72);
            assert(pack.latitude_GET() == -1644685055);
        });
        DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.latitude_SET(-1644685055) ;
        p48.altitude_SET(-607279784) ;
        p48.target_system_SET((char)72) ;
        p48.time_usec_SET(5084116729189478812L, PH) ;
        p48.longitude_SET(-701705904) ;
        LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == -833445556);
            assert(pack.time_usec_TRY(ph) == 7132496243585069521L);
            assert(pack.altitude_GET() == -890694382);
            assert(pack.latitude_GET() == -351613274);
        });
        DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.longitude_SET(-833445556) ;
        p49.altitude_SET(-890694382) ;
        p49.latitude_SET(-351613274) ;
        p49.time_usec_SET(7132496243585069521L, PH) ;
        LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.scale_GET() == 8.889652E37F);
            assert(pack.param_value_min_GET() == -2.3283083E38F);
            assert(pack.target_component_GET() == (char)104);
            assert(pack.parameter_rc_channel_index_GET() == (char)28);
            assert(pack.param_value_max_GET() == 7.2222126E37F);
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("u"));
            assert(pack.target_system_GET() == (char)169);
            assert(pack.param_value0_GET() == 2.2249524E37F);
            assert(pack.param_index_GET() == (short) -29662);
        });
        DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
        PH.setPack(p50);
        p50.param_id_SET("u", PH) ;
        p50.target_system_SET((char)169) ;
        p50.target_component_SET((char)104) ;
        p50.param_value_max_SET(7.2222126E37F) ;
        p50.param_value0_SET(2.2249524E37F) ;
        p50.param_value_min_SET(-2.3283083E38F) ;
        p50.parameter_rc_channel_index_SET((char)28) ;
        p50.param_index_SET((short) -29662) ;
        p50.scale_SET(8.889652E37F) ;
        LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.seq_GET() == (char)40338);
            assert(pack.target_component_GET() == (char)70);
            assert(pack.target_system_GET() == (char)67);
        });
        DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.seq_SET((char)40338) ;
        p51.target_system_SET((char)67) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p51.target_component_SET((char)70) ;
        LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p1x_GET() == 1.2950081E38F);
            assert(pack.target_system_GET() == (char)189);
            assert(pack.p2x_GET() == -3.2402005E38F);
            assert(pack.p2y_GET() == 2.8426578E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.p1z_GET() == -1.8300423E38F);
            assert(pack.p2z_GET() == -2.5937204E38F);
            assert(pack.target_component_GET() == (char)75);
            assert(pack.p1y_GET() == 2.014674E38F);
        });
        DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p2z_SET(-2.5937204E38F) ;
        p54.p1z_SET(-1.8300423E38F) ;
        p54.target_system_SET((char)189) ;
        p54.p1x_SET(1.2950081E38F) ;
        p54.p2y_SET(2.8426578E37F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p54.p1y_SET(2.014674E38F) ;
        p54.target_component_SET((char)75) ;
        p54.p2x_SET(-3.2402005E38F) ;
        LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2z_GET() == 9.620406E37F);
            assert(pack.p1x_GET() == -7.6840304E37F);
            assert(pack.p2y_GET() == 1.956877E38F);
            assert(pack.p1y_GET() == 7.56777E37F);
            assert(pack.p2x_GET() == 1.9911434E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.p1z_GET() == -5.760847E37F);
        });
        DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p2x_SET(1.9911434E38F) ;
        p55.p2z_SET(9.620406E37F) ;
        p55.p1x_SET(-7.6840304E37F) ;
        p55.p1y_SET(7.56777E37F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p55.p1z_SET(-5.760847E37F) ;
        p55.p2y_SET(1.956877E38F) ;
        LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7866102858731892704L);
            assert(pack.pitchspeed_GET() == 8.1629765E36F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-1.0750015E38F, 1.1941628E38F, 1.6181462E38F, -1.2705332E38F, 6.536543E37F, 1.8307367E38F, 7.054211E37F, 4.4668377E37F, -3.6317117E37F}));
            assert(pack.yawspeed_GET() == -4.056484E36F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.1764172E38F, -3.3931376E37F, -1.805017E38F, -4.447952E37F}));
            assert(pack.rollspeed_GET() == -3.1810288E38F);
        });
        DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.yawspeed_SET(-4.056484E36F) ;
        p61.pitchspeed_SET(8.1629765E36F) ;
        p61.covariance_SET(new float[] {-1.0750015E38F, 1.1941628E38F, 1.6181462E38F, -1.2705332E38F, 6.536543E37F, 1.8307367E38F, 7.054211E37F, 4.4668377E37F, -3.6317117E37F}, 0) ;
        p61.rollspeed_SET(-3.1810288E38F) ;
        p61.q_SET(new float[] {3.1764172E38F, -3.3931376E37F, -1.805017E38F, -4.447952E37F}, 0) ;
        p61.time_usec_SET(7866102858731892704L) ;
        LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.xtrack_error_GET() == -2.4042724E38F);
            assert(pack.nav_roll_GET() == 5.22946E37F);
            assert(pack.aspd_error_GET() == -1.8785132E38F);
            assert(pack.alt_error_GET() == 3.2479433E38F);
            assert(pack.wp_dist_GET() == (char)56086);
            assert(pack.nav_bearing_GET() == (short) -2457);
            assert(pack.target_bearing_GET() == (short) -30213);
            assert(pack.nav_pitch_GET() == 1.5254577E38F);
        });
        DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.nav_bearing_SET((short) -2457) ;
        p62.nav_roll_SET(5.22946E37F) ;
        p62.alt_error_SET(3.2479433E38F) ;
        p62.aspd_error_SET(-1.8785132E38F) ;
        p62.target_bearing_SET((short) -30213) ;
        p62.xtrack_error_SET(-2.4042724E38F) ;
        p62.nav_pitch_SET(1.5254577E38F) ;
        p62.wp_dist_SET((char)56086) ;
        LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == 9.633983E37F);
            assert(pack.relative_alt_GET() == -1338183705);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
            assert(pack.vx_GET() == 9.865502E37F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.474589E38F, -3.2873794E38F, -1.0871189E38F, 2.824709E38F, -3.234249E38F, -1.7524728E38F, -4.0854084E37F, 5.7065356E37F, -4.0818588E37F, 1.7772326E38F, 2.1438663E38F, 7.041489E37F, 1.1993181E38F, 1.8429962E38F, 3.2996385E38F, -3.2890683E37F, -1.6042123E38F, 1.992648E38F, 8.156158E37F, 2.6173955E36F, 2.2010167E38F, 2.9081797E38F, 1.0319386E38F, 8.3155146E37F, -2.5200344E38F, 2.4195544E38F, -2.4808555E38F, -2.5002702E38F, 1.7410978E38F, 2.1196463E38F, 1.3040635E38F, -3.4595075E37F, -8.570173E37F, -2.0681851E38F, -1.0892068E38F, 1.8886158E38F}));
            assert(pack.lat_GET() == 1107191924);
            assert(pack.alt_GET() == 703774434);
            assert(pack.lon_GET() == 1620178725);
            assert(pack.time_usec_GET() == 4111721882208965842L);
            assert(pack.vz_GET() == 2.7409959E38F);
        });
        DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.vy_SET(9.633983E37F) ;
        p63.vz_SET(2.7409959E38F) ;
        p63.vx_SET(9.865502E37F) ;
        p63.lon_SET(1620178725) ;
        p63.relative_alt_SET(-1338183705) ;
        p63.lat_SET(1107191924) ;
        p63.time_usec_SET(4111721882208965842L) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS) ;
        p63.covariance_SET(new float[] {2.474589E38F, -3.2873794E38F, -1.0871189E38F, 2.824709E38F, -3.234249E38F, -1.7524728E38F, -4.0854084E37F, 5.7065356E37F, -4.0818588E37F, 1.7772326E38F, 2.1438663E38F, 7.041489E37F, 1.1993181E38F, 1.8429962E38F, 3.2996385E38F, -3.2890683E37F, -1.6042123E38F, 1.992648E38F, 8.156158E37F, 2.6173955E36F, 2.2010167E38F, 2.9081797E38F, 1.0319386E38F, 8.3155146E37F, -2.5200344E38F, 2.4195544E38F, -2.4808555E38F, -2.5002702E38F, 1.7410978E38F, 2.1196463E38F, 1.3040635E38F, -3.4595075E37F, -8.570173E37F, -2.0681851E38F, -1.0892068E38F, 1.8886158E38F}, 0) ;
        p63.alt_SET(703774434) ;
        LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 4534558806374177089L);
            assert(pack.ax_GET() == 2.2875954E38F);
            assert(pack.ay_GET() == -1.8679915E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
            assert(pack.vy_GET() == -3.1968453E38F);
            assert(pack.x_GET() == -3.3655935E38F);
            assert(pack.z_GET() == 3.6588425E37F);
            assert(pack.y_GET() == 1.8767048E38F);
            assert(pack.vz_GET() == -2.020407E38F);
            assert(pack.vx_GET() == -1.838898E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.0604328E38F, 6.1827465E36F, 3.026455E37F, 3.1238377E38F, 1.3776896E38F, -1.1482946E38F, 2.3542413E38F, 7.324904E37F, -2.9110971E38F, -3.0254258E38F, -2.2297467E37F, -2.5277492E38F, 3.357578E36F, 8.525289E37F, 1.7225958E37F, -2.905214E38F, 9.186909E37F, 2.5833545E38F, 4.4885094E37F, 1.5885547E38F, -5.7396446E37F, -3.8888313E37F, 1.0619061E38F, 2.6526147E36F, -2.4070156E38F, 1.5362767E38F, 2.7038022E38F, -1.4943574E38F, 2.4542884E38F, -2.3858111E38F, -2.1116414E38F, 3.8699E36F, -4.755743E37F, -1.0286408E38F, -3.114204E38F, -1.666509E38F, 1.9938144E38F, -2.846118E38F, 2.783031E38F, 8.5669E37F, 2.198905E37F, -2.6044547E38F, -2.9870846E38F, 2.5042162E38F, 2.161933E38F}));
            assert(pack.az_GET() == -1.0391909E38F);
        });
        DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.az_SET(-1.0391909E38F) ;
        p64.vz_SET(-2.020407E38F) ;
        p64.vx_SET(-1.838898E38F) ;
        p64.z_SET(3.6588425E37F) ;
        p64.vy_SET(-3.1968453E38F) ;
        p64.time_usec_SET(4534558806374177089L) ;
        p64.x_SET(-3.3655935E38F) ;
        p64.ay_SET(-1.8679915E38F) ;
        p64.ax_SET(2.2875954E38F) ;
        p64.y_SET(1.8767048E38F) ;
        p64.covariance_SET(new float[] {2.0604328E38F, 6.1827465E36F, 3.026455E37F, 3.1238377E38F, 1.3776896E38F, -1.1482946E38F, 2.3542413E38F, 7.324904E37F, -2.9110971E38F, -3.0254258E38F, -2.2297467E37F, -2.5277492E38F, 3.357578E36F, 8.525289E37F, 1.7225958E37F, -2.905214E38F, 9.186909E37F, 2.5833545E38F, 4.4885094E37F, 1.5885547E38F, -5.7396446E37F, -3.8888313E37F, 1.0619061E38F, 2.6526147E36F, -2.4070156E38F, 1.5362767E38F, 2.7038022E38F, -1.4943574E38F, 2.4542884E38F, -2.3858111E38F, -2.1116414E38F, 3.8699E36F, -4.755743E37F, -1.0286408E38F, -3.114204E38F, -1.666509E38F, 1.9938144E38F, -2.846118E38F, 2.783031E38F, 8.5669E37F, 2.198905E37F, -2.6044547E38F, -2.9870846E38F, 2.5042162E38F, 2.161933E38F}, 0) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS) ;
        LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan2_raw_GET() == (char)43986);
            assert(pack.chan16_raw_GET() == (char)57156);
            assert(pack.time_boot_ms_GET() == 974833978L);
            assert(pack.chan10_raw_GET() == (char)7974);
            assert(pack.rssi_GET() == (char)158);
            assert(pack.chan6_raw_GET() == (char)52556);
            assert(pack.chan7_raw_GET() == (char)65039);
            assert(pack.chan12_raw_GET() == (char)52665);
            assert(pack.chan17_raw_GET() == (char)26073);
            assert(pack.chan1_raw_GET() == (char)60067);
            assert(pack.chan18_raw_GET() == (char)14531);
            assert(pack.chan14_raw_GET() == (char)25414);
            assert(pack.chancount_GET() == (char)207);
            assert(pack.chan5_raw_GET() == (char)51392);
            assert(pack.chan11_raw_GET() == (char)27534);
            assert(pack.chan4_raw_GET() == (char)50417);
            assert(pack.chan3_raw_GET() == (char)33983);
            assert(pack.chan13_raw_GET() == (char)49725);
            assert(pack.chan15_raw_GET() == (char)10102);
            assert(pack.chan9_raw_GET() == (char)52534);
            assert(pack.chan8_raw_GET() == (char)4463);
        });
        DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
        PH.setPack(p65);
        p65.chan10_raw_SET((char)7974) ;
        p65.chan2_raw_SET((char)43986) ;
        p65.chan9_raw_SET((char)52534) ;
        p65.chan6_raw_SET((char)52556) ;
        p65.chan17_raw_SET((char)26073) ;
        p65.chan3_raw_SET((char)33983) ;
        p65.chan18_raw_SET((char)14531) ;
        p65.chan8_raw_SET((char)4463) ;
        p65.chan7_raw_SET((char)65039) ;
        p65.chan15_raw_SET((char)10102) ;
        p65.chan5_raw_SET((char)51392) ;
        p65.chan16_raw_SET((char)57156) ;
        p65.chan12_raw_SET((char)52665) ;
        p65.rssi_SET((char)158) ;
        p65.chan4_raw_SET((char)50417) ;
        p65.chan11_raw_SET((char)27534) ;
        p65.chan13_raw_SET((char)49725) ;
        p65.time_boot_ms_SET(974833978L) ;
        p65.chancount_SET((char)207) ;
        p65.chan14_raw_SET((char)25414) ;
        p65.chan1_raw_SET((char)60067) ;
        LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.req_stream_id_GET() == (char)245);
            assert(pack.start_stop_GET() == (char)20);
            assert(pack.req_message_rate_GET() == (char)61368);
            assert(pack.target_component_GET() == (char)182);
            assert(pack.target_system_GET() == (char)96);
        });
        DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.target_system_SET((char)96) ;
        p66.start_stop_SET((char)20) ;
        p66.req_stream_id_SET((char)245) ;
        p66.req_message_rate_SET((char)61368) ;
        p66.target_component_SET((char)182) ;
        LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.on_off_GET() == (char)131);
            assert(pack.stream_id_GET() == (char)243);
            assert(pack.message_rate_GET() == (char)11836);
        });
        DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
        PH.setPack(p67);
        p67.on_off_SET((char)131) ;
        p67.message_rate_SET((char)11836) ;
        p67.stream_id_SET((char)243) ;
        LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.buttons_GET() == (char)1355);
            assert(pack.target_GET() == (char)147);
            assert(pack.x_GET() == (short) -3151);
            assert(pack.z_GET() == (short)15254);
            assert(pack.r_GET() == (short)23696);
            assert(pack.y_GET() == (short) -476);
        });
        DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
        PH.setPack(p69);
        p69.r_SET((short)23696) ;
        p69.y_SET((short) -476) ;
        p69.x_SET((short) -3151) ;
        p69.target_SET((char)147) ;
        p69.buttons_SET((char)1355) ;
        p69.z_SET((short)15254) ;
        LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan1_raw_GET() == (char)23966);
            assert(pack.chan6_raw_GET() == (char)7483);
            assert(pack.chan8_raw_GET() == (char)39471);
            assert(pack.chan5_raw_GET() == (char)64475);
            assert(pack.target_system_GET() == (char)224);
            assert(pack.chan3_raw_GET() == (char)35717);
            assert(pack.chan4_raw_GET() == (char)34037);
            assert(pack.chan2_raw_GET() == (char)62034);
            assert(pack.chan7_raw_GET() == (char)50588);
            assert(pack.target_component_GET() == (char)230);
        });
        DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan8_raw_SET((char)39471) ;
        p70.chan7_raw_SET((char)50588) ;
        p70.target_system_SET((char)224) ;
        p70.chan5_raw_SET((char)64475) ;
        p70.target_component_SET((char)230) ;
        p70.chan2_raw_SET((char)62034) ;
        p70.chan3_raw_SET((char)35717) ;
        p70.chan4_raw_SET((char)34037) ;
        p70.chan6_raw_SET((char)7483) ;
        p70.chan1_raw_SET((char)23966) ;
        LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 1473101018);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST);
            assert(pack.current_GET() == (char)29);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.seq_GET() == (char)7117);
            assert(pack.z_GET() == -8.3736947E37F);
            assert(pack.y_GET() == 632901013);
            assert(pack.param4_GET() == -2.5783072E38F);
            assert(pack.target_system_GET() == (char)248);
            assert(pack.autocontinue_GET() == (char)47);
            assert(pack.target_component_GET() == (char)75);
            assert(pack.param2_GET() == -2.7097658E38F);
            assert(pack.param3_GET() == -2.2049407E38F);
            assert(pack.param1_GET() == -1.1390068E38F);
        });
        DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.param1_SET(-1.1390068E38F) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p73.target_system_SET((char)248) ;
        p73.current_SET((char)29) ;
        p73.autocontinue_SET((char)47) ;
        p73.x_SET(1473101018) ;
        p73.y_SET(632901013) ;
        p73.seq_SET((char)7117) ;
        p73.target_component_SET((char)75) ;
        p73.command_SET(MAV_CMD.MAV_CMD_ARM_AUTHORIZATION_REQUEST) ;
        p73.param3_SET(-2.2049407E38F) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p73.param4_SET(-2.5783072E38F) ;
        p73.z_SET(-8.3736947E37F) ;
        p73.param2_SET(-2.7097658E38F) ;
        LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.airspeed_GET() == 2.0614666E38F);
            assert(pack.heading_GET() == (short) -18707);
            assert(pack.groundspeed_GET() == -2.683585E38F);
            assert(pack.climb_GET() == -1.4096647E38F);
            assert(pack.throttle_GET() == (char)65209);
            assert(pack.alt_GET() == -2.3037534E38F);
        });
        DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
        PH.setPack(p74);
        p74.throttle_SET((char)65209) ;
        p74.alt_SET(-2.3037534E38F) ;
        p74.heading_SET((short) -18707) ;
        p74.airspeed_SET(2.0614666E38F) ;
        p74.groundspeed_SET(-2.683585E38F) ;
        p74.climb_SET(-1.4096647E38F) ;
        LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.param3_GET() == -2.5432334E38F);
            assert(pack.target_system_GET() == (char)236);
            assert(pack.z_GET() == -1.1263713E38F);
            assert(pack.param4_GET() == 1.1545577E38F);
            assert(pack.x_GET() == -1386080268);
            assert(pack.target_component_GET() == (char)205);
            assert(pack.param2_GET() == -2.362169E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE);
            assert(pack.current_GET() == (char)20);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_ENU);
            assert(pack.y_GET() == -106376261);
            assert(pack.param1_GET() == -3.3179968E38F);
            assert(pack.autocontinue_GET() == (char)41);
        });
        DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
        PH.setPack(p75);
        p75.command_SET(MAV_CMD.MAV_CMD_DO_CHANGE_ALTITUDE) ;
        p75.param3_SET(-2.5432334E38F) ;
        p75.target_component_SET((char)205) ;
        p75.param2_SET(-2.362169E38F) ;
        p75.current_SET((char)20) ;
        p75.target_system_SET((char)236) ;
        p75.autocontinue_SET((char)41) ;
        p75.x_SET(-1386080268) ;
        p75.param4_SET(1.1545577E38F) ;
        p75.param1_SET(-3.3179968E38F) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_ENU) ;
        p75.z_SET(-1.1263713E38F) ;
        p75.y_SET(-106376261) ;
        LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.param7_GET() == -2.2228418E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE);
            assert(pack.target_system_GET() == (char)182);
            assert(pack.param6_GET() == 1.8157097E38F);
            assert(pack.param3_GET() == -1.5712376E38F);
            assert(pack.param1_GET() == -1.71604E38F);
            assert(pack.target_component_GET() == (char)10);
            assert(pack.confirmation_GET() == (char)87);
            assert(pack.param4_GET() == -2.2393699E38F);
            assert(pack.param5_GET() == 2.5581298E36F);
            assert(pack.param2_GET() == -2.6108201E38F);
        });
        DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.target_system_SET((char)182) ;
        p76.target_component_SET((char)10) ;
        p76.param2_SET(-2.6108201E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_PREFLIGHT_STORAGE) ;
        p76.confirmation_SET((char)87) ;
        p76.param1_SET(-1.71604E38F) ;
        p76.param7_SET(-2.2228418E38F) ;
        p76.param5_SET(2.5581298E36F) ;
        p76.param3_SET(-1.5712376E38F) ;
        p76.param6_SET(1.8157097E38F) ;
        p76.param4_SET(-2.2393699E38F) ;
        LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.progress_TRY(ph) == (char)103);
            assert(pack.target_component_TRY(ph) == (char)130);
            assert(pack.result_param2_TRY(ph) == -1614300625);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_UNSUPPORTED);
            assert(pack.target_system_TRY(ph) == (char)46);
        });
        DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.target_system_SET((char)46, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_RESET_CAMERA_SETTINGS) ;
        p77.progress_SET((char)103, PH) ;
        p77.result_param2_SET(-1614300625, PH) ;
        p77.target_component_SET((char)130, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_UNSUPPORTED) ;
        LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.mode_switch_GET() == (char)56);
            assert(pack.yaw_GET() == 4.5054944E37F);
            assert(pack.manual_override_switch_GET() == (char)72);
            assert(pack.roll_GET() == -2.2257847E38F);
            assert(pack.time_boot_ms_GET() == 621674363L);
            assert(pack.thrust_GET() == 1.3749712E38F);
            assert(pack.pitch_GET() == 1.0248577E38F);
        });
        DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.yaw_SET(4.5054944E37F) ;
        p81.thrust_SET(1.3749712E38F) ;
        p81.pitch_SET(1.0248577E38F) ;
        p81.manual_override_switch_SET((char)72) ;
        p81.roll_SET(-2.2257847E38F) ;
        p81.mode_switch_SET((char)56) ;
        p81.time_boot_ms_SET(621674363L) ;
        LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_roll_rate_GET() == 4.3449135E37F);
            assert(pack.body_pitch_rate_GET() == 2.3188017E38F);
            assert(pack.target_component_GET() == (char)255);
            assert(pack.type_mask_GET() == (char)107);
            assert(pack.time_boot_ms_GET() == 1418559580L);
            assert(pack.thrust_GET() == -3.315378E38F);
            assert(pack.body_yaw_rate_GET() == 1.226455E38F);
            assert(pack.target_system_GET() == (char)230);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.3862488E38F, -5.769982E37F, 2.580788E38F, -1.3013324E38F}));
        });
        DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.time_boot_ms_SET(1418559580L) ;
        p82.thrust_SET(-3.315378E38F) ;
        p82.target_system_SET((char)230) ;
        p82.q_SET(new float[] {2.3862488E38F, -5.769982E37F, 2.580788E38F, -1.3013324E38F}, 0) ;
        p82.body_pitch_rate_SET(2.3188017E38F) ;
        p82.body_yaw_rate_SET(1.226455E38F) ;
        p82.type_mask_SET((char)107) ;
        p82.target_component_SET((char)255) ;
        p82.body_roll_rate_SET(4.3449135E37F) ;
        LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.thrust_GET() == 3.1554831E38F);
            assert(pack.type_mask_GET() == (char)39);
            assert(pack.body_roll_rate_GET() == 1.0901178E38F);
            assert(pack.body_pitch_rate_GET() == 9.108208E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.6948361E38F, 1.2004931E38F, 1.0603921E38F, -2.738322E38F}));
            assert(pack.time_boot_ms_GET() == 3110100722L);
            assert(pack.body_yaw_rate_GET() == 2.3313453E38F);
        });
        DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.q_SET(new float[] {1.6948361E38F, 1.2004931E38F, 1.0603921E38F, -2.738322E38F}, 0) ;
        p83.body_roll_rate_SET(1.0901178E38F) ;
        p83.body_pitch_rate_SET(9.108208E37F) ;
        p83.time_boot_ms_SET(3110100722L) ;
        p83.type_mask_SET((char)39) ;
        p83.body_yaw_rate_SET(2.3313453E38F) ;
        p83.thrust_SET(3.1554831E38F) ;
        LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.afx_GET() == 2.9100735E38F);
            assert(pack.yaw_GET() == 1.2510102E38F);
            assert(pack.vx_GET() == 1.455455E38F);
            assert(pack.y_GET() == -2.4084666E38F);
            assert(pack.afz_GET() == -1.3317656E38F);
            assert(pack.type_mask_GET() == (char)46563);
            assert(pack.target_component_GET() == (char)48);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
            assert(pack.x_GET() == 3.347843E38F);
            assert(pack.z_GET() == -1.7888188E37F);
            assert(pack.target_system_GET() == (char)18);
            assert(pack.afy_GET() == -2.8096494E37F);
            assert(pack.time_boot_ms_GET() == 2277937021L);
            assert(pack.vz_GET() == 1.4133031E38F);
            assert(pack.vy_GET() == 3.2882366E38F);
            assert(pack.yaw_rate_GET() == 1.204892E38F);
        });
        DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.y_SET(-2.4084666E38F) ;
        p84.vy_SET(3.2882366E38F) ;
        p84.time_boot_ms_SET(2277937021L) ;
        p84.target_component_SET((char)48) ;
        p84.x_SET(3.347843E38F) ;
        p84.yaw_rate_SET(1.204892E38F) ;
        p84.z_SET(-1.7888188E37F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) ;
        p84.afz_SET(-1.3317656E38F) ;
        p84.type_mask_SET((char)46563) ;
        p84.vx_SET(1.455455E38F) ;
        p84.afy_SET(-2.8096494E37F) ;
        p84.afx_SET(2.9100735E38F) ;
        p84.yaw_SET(1.2510102E38F) ;
        p84.target_system_SET((char)18) ;
        p84.vz_SET(1.4133031E38F) ;
        LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -2.7466434E37F);
            assert(pack.target_component_GET() == (char)143);
            assert(pack.lon_int_GET() == 1712342395);
            assert(pack.type_mask_GET() == (char)3350);
            assert(pack.afz_GET() == 2.8521607E38F);
            assert(pack.time_boot_ms_GET() == 1421824782L);
            assert(pack.lat_int_GET() == 1669126362);
            assert(pack.vx_GET() == -1.0286266E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.vy_GET() == -2.1950808E38F);
            assert(pack.yaw_rate_GET() == 2.995007E37F);
            assert(pack.target_system_GET() == (char)29);
            assert(pack.yaw_GET() == 6.6901097E37F);
            assert(pack.afx_GET() == -8.198399E36F);
            assert(pack.vz_GET() == -4.726871E37F);
            assert(pack.afy_GET() == 1.3122577E38F);
        });
        DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.yaw_rate_SET(2.995007E37F) ;
        p86.vx_SET(-1.0286266E38F) ;
        p86.time_boot_ms_SET(1421824782L) ;
        p86.lon_int_SET(1712342395) ;
        p86.afx_SET(-8.198399E36F) ;
        p86.target_system_SET((char)29) ;
        p86.vz_SET(-4.726871E37F) ;
        p86.target_component_SET((char)143) ;
        p86.type_mask_SET((char)3350) ;
        p86.vy_SET(-2.1950808E38F) ;
        p86.afz_SET(2.8521607E38F) ;
        p86.alt_SET(-2.7466434E37F) ;
        p86.lat_int_SET(1669126362) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p86.yaw_SET(6.6901097E37F) ;
        p86.afy_SET(1.3122577E38F) ;
        LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)16641);
            assert(pack.lat_int_GET() == -2008251976);
            assert(pack.vz_GET() == 2.9665864E38F);
            assert(pack.lon_int_GET() == 1188455983);
            assert(pack.alt_GET() == -2.6519397E38F);
            assert(pack.time_boot_ms_GET() == 2925621039L);
            assert(pack.vy_GET() == -1.737038E38F);
            assert(pack.afz_GET() == -2.6950102E38F);
            assert(pack.afy_GET() == 2.7779861E38F);
            assert(pack.yaw_rate_GET() == -1.9634806E38F);
            assert(pack.yaw_GET() == -2.1405288E38F);
            assert(pack.vx_GET() == -1.3231943E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.afx_GET() == -2.9483076E38F);
        });
        DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.alt_SET(-2.6519397E38F) ;
        p87.vy_SET(-1.737038E38F) ;
        p87.lat_int_SET(-2008251976) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p87.afx_SET(-2.9483076E38F) ;
        p87.afz_SET(-2.6950102E38F) ;
        p87.yaw_SET(-2.1405288E38F) ;
        p87.vx_SET(-1.3231943E38F) ;
        p87.lon_int_SET(1188455983) ;
        p87.vz_SET(2.9665864E38F) ;
        p87.yaw_rate_SET(-1.9634806E38F) ;
        p87.afy_SET(2.7779861E38F) ;
        p87.type_mask_SET((char)16641) ;
        p87.time_boot_ms_SET(2925621039L) ;
        LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 319171001L);
            assert(pack.z_GET() == -1.8476977E38F);
            assert(pack.yaw_GET() == 9.294042E37F);
            assert(pack.x_GET() == 2.2418107E38F);
            assert(pack.pitch_GET() == 2.310872E38F);
            assert(pack.roll_GET() == 8.4962035E37F);
            assert(pack.y_GET() == -1.0972805E38F);
        });
        DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.z_SET(-1.8476977E38F) ;
        p89.time_boot_ms_SET(319171001L) ;
        p89.y_SET(-1.0972805E38F) ;
        p89.x_SET(2.2418107E38F) ;
        p89.pitch_SET(2.310872E38F) ;
        p89.roll_SET(8.4962035E37F) ;
        p89.yaw_SET(9.294042E37F) ;
        LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == 297664025);
            assert(pack.yacc_GET() == (short)24337);
            assert(pack.yawspeed_GET() == 3.6027966E37F);
            assert(pack.lon_GET() == 1156598359);
            assert(pack.yaw_GET() == -1.8691555E38F);
            assert(pack.time_usec_GET() == 6974877291603186773L);
            assert(pack.xacc_GET() == (short) -10747);
            assert(pack.lat_GET() == 56330340);
            assert(pack.vx_GET() == (short)10796);
            assert(pack.roll_GET() == 1.8156154E38F);
            assert(pack.rollspeed_GET() == -2.7588635E38F);
            assert(pack.vz_GET() == (short) -22287);
            assert(pack.pitch_GET() == 1.7515749E38F);
            assert(pack.vy_GET() == (short) -1570);
            assert(pack.zacc_GET() == (short) -19698);
            assert(pack.pitchspeed_GET() == 1.7538388E38F);
        });
        DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.lon_SET(1156598359) ;
        p90.vx_SET((short)10796) ;
        p90.xacc_SET((short) -10747) ;
        p90.yacc_SET((short)24337) ;
        p90.yawspeed_SET(3.6027966E37F) ;
        p90.lat_SET(56330340) ;
        p90.roll_SET(1.8156154E38F) ;
        p90.pitchspeed_SET(1.7538388E38F) ;
        p90.vz_SET((short) -22287) ;
        p90.vy_SET((short) -1570) ;
        p90.zacc_SET((short) -19698) ;
        p90.alt_SET(297664025) ;
        p90.time_usec_SET(6974877291603186773L) ;
        p90.pitch_SET(1.7515749E38F) ;
        p90.rollspeed_SET(-2.7588635E38F) ;
        p90.yaw_SET(-1.8691555E38F) ;
        LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_ARMED);
            assert(pack.time_usec_GET() == 3906649915582302525L);
            assert(pack.aux2_GET() == 4.621019E37F);
            assert(pack.pitch_elevator_GET() == -6.4127554E37F);
            assert(pack.throttle_GET() == -1.9559204E38F);
            assert(pack.nav_mode_GET() == (char)13);
            assert(pack.aux4_GET() == -6.736188E37F);
            assert(pack.yaw_rudder_GET() == 2.7752942E38F);
            assert(pack.roll_ailerons_GET() == 2.6138565E37F);
            assert(pack.aux1_GET() == -1.0830576E38F);
            assert(pack.aux3_GET() == -1.0583161E38F);
        });
        DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.roll_ailerons_SET(2.6138565E37F) ;
        p91.aux3_SET(-1.0583161E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_TEST_ARMED) ;
        p91.aux4_SET(-6.736188E37F) ;
        p91.nav_mode_SET((char)13) ;
        p91.aux1_SET(-1.0830576E38F) ;
        p91.throttle_SET(-1.9559204E38F) ;
        p91.time_usec_SET(3906649915582302525L) ;
        p91.pitch_elevator_SET(-6.4127554E37F) ;
        p91.aux2_SET(4.621019E37F) ;
        p91.yaw_rudder_SET(2.7752942E38F) ;
        LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan2_raw_GET() == (char)1603);
            assert(pack.chan7_raw_GET() == (char)12605);
            assert(pack.chan3_raw_GET() == (char)60630);
            assert(pack.time_usec_GET() == 8678226101167970982L);
            assert(pack.chan12_raw_GET() == (char)2065);
            assert(pack.chan8_raw_GET() == (char)62142);
            assert(pack.chan5_raw_GET() == (char)12961);
            assert(pack.rssi_GET() == (char)224);
            assert(pack.chan4_raw_GET() == (char)15632);
            assert(pack.chan11_raw_GET() == (char)56010);
            assert(pack.chan10_raw_GET() == (char)63290);
            assert(pack.chan9_raw_GET() == (char)37577);
            assert(pack.chan6_raw_GET() == (char)36948);
            assert(pack.chan1_raw_GET() == (char)56441);
        });
        DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan1_raw_SET((char)56441) ;
        p92.chan6_raw_SET((char)36948) ;
        p92.chan10_raw_SET((char)63290) ;
        p92.rssi_SET((char)224) ;
        p92.chan8_raw_SET((char)62142) ;
        p92.chan12_raw_SET((char)2065) ;
        p92.time_usec_SET(8678226101167970982L) ;
        p92.chan3_raw_SET((char)60630) ;
        p92.chan7_raw_SET((char)12605) ;
        p92.chan2_raw_SET((char)1603) ;
        p92.chan4_raw_SET((char)15632) ;
        p92.chan9_raw_SET((char)37577) ;
        p92.chan5_raw_SET((char)12961) ;
        p92.chan11_raw_SET((char)56010) ;
        LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_DISARMED);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.560349E38F, -2.764918E38F, 2.5696657E38F, 2.8678824E38F, 2.5425424E38F, 1.6977072E38F, 2.0060324E36F, 3.1578866E38F, -7.710957E37F, 1.6946368E38F, -4.4217245E37F, -1.2160552E38F, -7.198665E37F, 2.42386E38F, -7.3290913E37F, 1.009883E38F}));
            assert(pack.time_usec_GET() == 3730562447192447992L);
            assert(pack.flags_GET() == 6148114610912849831L);
        });
        DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(3730562447192447992L) ;
        p93.flags_SET(6148114610912849831L) ;
        p93.controls_SET(new float[] {2.560349E38F, -2.764918E38F, 2.5696657E38F, 2.8678824E38F, 2.5425424E38F, 1.6977072E38F, 2.0060324E36F, 3.1578866E38F, -7.710957E37F, 1.6946368E38F, -4.4217245E37F, -1.2160552E38F, -7.198665E37F, 2.42386E38F, -7.3290913E37F, 1.009883E38F}, 0) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_TEST_DISARMED) ;
        LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.sensor_id_GET() == (char)206);
            assert(pack.flow_x_GET() == (short) -8085);
            assert(pack.ground_distance_GET() == 2.6130682E38F);
            assert(pack.time_usec_GET() == 2049607689669757786L);
            assert(pack.flow_rate_y_TRY(ph) == -9.000052E37F);
            assert(pack.quality_GET() == (char)38);
            assert(pack.flow_comp_m_x_GET() == -4.8220557E36F);
            assert(pack.flow_rate_x_TRY(ph) == -1.5569006E38F);
            assert(pack.flow_y_GET() == (short)13897);
            assert(pack.flow_comp_m_y_GET() == 9.913309E37F);
        });
        DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.flow_x_SET((short) -8085) ;
        p100.time_usec_SET(2049607689669757786L) ;
        p100.quality_SET((char)38) ;
        p100.flow_comp_m_y_SET(9.913309E37F) ;
        p100.flow_comp_m_x_SET(-4.8220557E36F) ;
        p100.sensor_id_SET((char)206) ;
        p100.flow_rate_x_SET(-1.5569006E38F, PH) ;
        p100.ground_distance_SET(2.6130682E38F) ;
        p100.flow_rate_y_SET(-9.000052E37F, PH) ;
        p100.flow_y_SET((short)13897) ;
        LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -1.4307109E38F);
            assert(pack.yaw_GET() == -5.4492684E37F);
            assert(pack.pitch_GET() == -2.6994977E38F);
            assert(pack.roll_GET() == 2.40009E37F);
            assert(pack.usec_GET() == 2155222519955478941L);
            assert(pack.x_GET() == 1.1438012E38F);
            assert(pack.y_GET() == -1.9863489E38F);
        });
        DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.roll_SET(2.40009E37F) ;
        p101.z_SET(-1.4307109E38F) ;
        p101.usec_SET(2155222519955478941L) ;
        p101.x_SET(1.1438012E38F) ;
        p101.y_SET(-1.9863489E38F) ;
        p101.pitch_SET(-2.6994977E38F) ;
        p101.yaw_SET(-5.4492684E37F) ;
        LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -2.2360507E38F);
            assert(pack.usec_GET() == 7602755656102912320L);
            assert(pack.y_GET() == -1.3140432E38F);
            assert(pack.pitch_GET() == 1.995774E38F);
            assert(pack.roll_GET() == -1.976763E38F);
            assert(pack.yaw_GET() == -1.7810966E38F);
            assert(pack.x_GET() == -1.8662547E38F);
        });
        DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.yaw_SET(-1.7810966E38F) ;
        p102.z_SET(-2.2360507E38F) ;
        p102.y_SET(-1.3140432E38F) ;
        p102.roll_SET(-1.976763E38F) ;
        p102.x_SET(-1.8662547E38F) ;
        p102.pitch_SET(1.995774E38F) ;
        p102.usec_SET(7602755656102912320L) ;
        LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -1.5196298E38F);
            assert(pack.usec_GET() == 4175133992565948097L);
            assert(pack.y_GET() == 2.429431E38F);
            assert(pack.z_GET() == 3.2094285E38F);
        });
        DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.y_SET(2.429431E38F) ;
        p103.x_SET(-1.5196298E38F) ;
        p103.z_SET(3.2094285E38F) ;
        p103.usec_SET(4175133992565948097L) ;
        LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == -3.229298E38F);
            assert(pack.usec_GET() == 2575788105037332333L);
            assert(pack.pitch_GET() == -1.2977557E38F);
            assert(pack.x_GET() == -1.4862055E38F);
            assert(pack.roll_GET() == -3.1709262E38F);
            assert(pack.z_GET() == 1.6297657E38F);
            assert(pack.yaw_GET() == -2.456799E38F);
        });
        DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.x_SET(-1.4862055E38F) ;
        p104.roll_SET(-3.1709262E38F) ;
        p104.pitch_SET(-1.2977557E38F) ;
        p104.y_SET(-3.229298E38F) ;
        p104.yaw_SET(-2.456799E38F) ;
        p104.usec_SET(2575788105037332333L) ;
        p104.z_SET(1.6297657E38F) ;
        LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == -7.9322227E37F);
            assert(pack.ygyro_GET() == -2.665952E38F);
            assert(pack.xacc_GET() == 2.254345E37F);
            assert(pack.xgyro_GET() == -3.2589228E38F);
            assert(pack.temperature_GET() == -2.3896439E38F);
            assert(pack.time_usec_GET() == 1704764607979590352L);
            assert(pack.fields_updated_GET() == (char)28919);
            assert(pack.diff_pressure_GET() == 5.2604716E37F);
            assert(pack.xmag_GET() == -2.2052969E38F);
            assert(pack.pressure_alt_GET() == 1.1383218E38F);
            assert(pack.zacc_GET() == -1.8405863E38F);
            assert(pack.ymag_GET() == 1.1485234E37F);
            assert(pack.zmag_GET() == -4.2922136E36F);
            assert(pack.abs_pressure_GET() == -3.0072844E38F);
            assert(pack.zgyro_GET() == -1.7382435E38F);
        });
        DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.zgyro_SET(-1.7382435E38F) ;
        p105.zacc_SET(-1.8405863E38F) ;
        p105.ygyro_SET(-2.665952E38F) ;
        p105.yacc_SET(-7.9322227E37F) ;
        p105.fields_updated_SET((char)28919) ;
        p105.pressure_alt_SET(1.1383218E38F) ;
        p105.temperature_SET(-2.3896439E38F) ;
        p105.ymag_SET(1.1485234E37F) ;
        p105.abs_pressure_SET(-3.0072844E38F) ;
        p105.xmag_SET(-2.2052969E38F) ;
        p105.zmag_SET(-4.2922136E36F) ;
        p105.diff_pressure_SET(5.2604716E37F) ;
        p105.xacc_SET(2.254345E37F) ;
        p105.time_usec_SET(1704764607979590352L) ;
        p105.xgyro_SET(-3.2589228E38F) ;
        LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.sensor_id_GET() == (char)156);
            assert(pack.distance_GET() == 2.723777E38F);
            assert(pack.quality_GET() == (char)22);
            assert(pack.integration_time_us_GET() == 1193903721L);
            assert(pack.time_delta_distance_us_GET() == 942241737L);
            assert(pack.integrated_y_GET() == 2.7884455E38F);
            assert(pack.integrated_ygyro_GET() == -1.6558018E38F);
            assert(pack.time_usec_GET() == 3904556293406132165L);
            assert(pack.integrated_x_GET() == -3.2688462E38F);
            assert(pack.integrated_zgyro_GET() == -1.1087803E38F);
            assert(pack.temperature_GET() == (short) -19968);
            assert(pack.integrated_xgyro_GET() == 2.0945072E38F);
        });
        DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.integrated_ygyro_SET(-1.6558018E38F) ;
        p106.temperature_SET((short) -19968) ;
        p106.quality_SET((char)22) ;
        p106.sensor_id_SET((char)156) ;
        p106.integration_time_us_SET(1193903721L) ;
        p106.time_usec_SET(3904556293406132165L) ;
        p106.integrated_x_SET(-3.2688462E38F) ;
        p106.integrated_y_SET(2.7884455E38F) ;
        p106.integrated_zgyro_SET(-1.1087803E38F) ;
        p106.integrated_xgyro_SET(2.0945072E38F) ;
        p106.distance_SET(2.723777E38F) ;
        p106.time_delta_distance_us_SET(942241737L) ;
        LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.zgyro_GET() == -1.5928753E38F);
            assert(pack.fields_updated_GET() == 279269654L);
            assert(pack.ygyro_GET() == -5.5373224E37F);
            assert(pack.xgyro_GET() == -2.8583827E37F);
            assert(pack.temperature_GET() == -5.7540714E37F);
            assert(pack.zmag_GET() == -6.6165495E37F);
            assert(pack.zacc_GET() == -1.726484E38F);
            assert(pack.time_usec_GET() == 8768181493589538039L);
            assert(pack.pressure_alt_GET() == 1.4623578E38F);
            assert(pack.xmag_GET() == -1.6808731E38F);
            assert(pack.xacc_GET() == 9.076387E37F);
            assert(pack.diff_pressure_GET() == -6.9541755E37F);
            assert(pack.yacc_GET() == -1.6269175E38F);
            assert(pack.ymag_GET() == 1.2951835E38F);
            assert(pack.abs_pressure_GET() == -8.797214E37F);
        });
        DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.zacc_SET(-1.726484E38F) ;
        p107.time_usec_SET(8768181493589538039L) ;
        p107.yacc_SET(-1.6269175E38F) ;
        p107.diff_pressure_SET(-6.9541755E37F) ;
        p107.pressure_alt_SET(1.4623578E38F) ;
        p107.xgyro_SET(-2.8583827E37F) ;
        p107.ygyro_SET(-5.5373224E37F) ;
        p107.zmag_SET(-6.6165495E37F) ;
        p107.fields_updated_SET(279269654L) ;
        p107.xmag_SET(-1.6808731E38F) ;
        p107.zgyro_SET(-1.5928753E38F) ;
        p107.temperature_SET(-5.7540714E37F) ;
        p107.ymag_SET(1.2951835E38F) ;
        p107.xacc_SET(9.076387E37F) ;
        p107.abs_pressure_SET(-8.797214E37F) ;
        LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == -1.6324389E37F);
            assert(pack.std_dev_vert_GET() == 2.5929558E38F);
            assert(pack.xacc_GET() == -3.2179231E38F);
            assert(pack.ve_GET() == -1.4357312E38F);
            assert(pack.yacc_GET() == -1.4554977E38F);
            assert(pack.pitch_GET() == 3.1198567E38F);
            assert(pack.roll_GET() == 7.9297143E37F);
            assert(pack.yaw_GET() == -3.023475E38F);
            assert(pack.vn_GET() == -6.859326E37F);
            assert(pack.xgyro_GET() == 3.1363104E38F);
            assert(pack.lon_GET() == 2.568511E38F);
            assert(pack.vd_GET() == 1.0407242E38F);
            assert(pack.zgyro_GET() == -1.083705E37F);
            assert(pack.std_dev_horz_GET() == -1.2905875E38F);
            assert(pack.q4_GET() == -2.1453508E38F);
            assert(pack.q2_GET() == 2.7323647E38F);
            assert(pack.q3_GET() == -1.7874051E37F);
            assert(pack.lat_GET() == 9.115065E37F);
            assert(pack.zacc_GET() == -3.2439436E38F);
            assert(pack.alt_GET() == -2.4811642E38F);
            assert(pack.q1_GET() == -3.0724968E38F);
        });
        DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.vn_SET(-6.859326E37F) ;
        p108.xacc_SET(-3.2179231E38F) ;
        p108.alt_SET(-2.4811642E38F) ;
        p108.zgyro_SET(-1.083705E37F) ;
        p108.zacc_SET(-3.2439436E38F) ;
        p108.q2_SET(2.7323647E38F) ;
        p108.pitch_SET(3.1198567E38F) ;
        p108.vd_SET(1.0407242E38F) ;
        p108.lon_SET(2.568511E38F) ;
        p108.q3_SET(-1.7874051E37F) ;
        p108.lat_SET(9.115065E37F) ;
        p108.yaw_SET(-3.023475E38F) ;
        p108.ygyro_SET(-1.6324389E37F) ;
        p108.std_dev_vert_SET(2.5929558E38F) ;
        p108.std_dev_horz_SET(-1.2905875E38F) ;
        p108.yacc_SET(-1.4554977E38F) ;
        p108.xgyro_SET(3.1363104E38F) ;
        p108.roll_SET(7.9297143E37F) ;
        p108.q4_SET(-2.1453508E38F) ;
        p108.q1_SET(-3.0724968E38F) ;
        p108.ve_SET(-1.4357312E38F) ;
        LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.remrssi_GET() == (char)34);
            assert(pack.fixed__GET() == (char)38293);
            assert(pack.rxerrors_GET() == (char)63639);
            assert(pack.rssi_GET() == (char)196);
            assert(pack.txbuf_GET() == (char)142);
            assert(pack.remnoise_GET() == (char)117);
            assert(pack.noise_GET() == (char)216);
        });
        DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.remrssi_SET((char)34) ;
        p109.noise_SET((char)216) ;
        p109.txbuf_SET((char)142) ;
        p109.rssi_SET((char)196) ;
        p109.rxerrors_SET((char)63639) ;
        p109.fixed__SET((char)38293) ;
        p109.remnoise_SET((char)117) ;
        LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_network_GET() == (char)131);
            assert(pack.target_component_GET() == (char)254);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)106, (char)25, (char)178, (char)251, (char)150, (char)15, (char)128, (char)220, (char)92, (char)8, (char)85, (char)29, (char)125, (char)208, (char)242, (char)3, (char)230, (char)109, (char)185, (char)81, (char)185, (char)213, (char)127, (char)218, (char)185, (char)98, (char)196, (char)207, (char)47, (char)114, (char)182, (char)80, (char)106, (char)176, (char)134, (char)67, (char)106, (char)233, (char)205, (char)45, (char)193, (char)159, (char)30, (char)23, (char)230, (char)141, (char)197, (char)132, (char)80, (char)97, (char)79, (char)248, (char)113, (char)192, (char)117, (char)232, (char)69, (char)29, (char)39, (char)181, (char)37, (char)67, (char)126, (char)253, (char)166, (char)113, (char)212, (char)50, (char)67, (char)84, (char)5, (char)209, (char)63, (char)200, (char)203, (char)85, (char)69, (char)238, (char)150, (char)146, (char)96, (char)86, (char)6, (char)131, (char)189, (char)182, (char)76, (char)192, (char)186, (char)103, (char)41, (char)249, (char)248, (char)33, (char)44, (char)71, (char)116, (char)28, (char)142, (char)122, (char)0, (char)59, (char)100, (char)69, (char)100, (char)180, (char)52, (char)137, (char)97, (char)73, (char)0, (char)249, (char)118, (char)63, (char)173, (char)55, (char)243, (char)196, (char)89, (char)11, (char)16, (char)48, (char)174, (char)142, (char)87, (char)75, (char)144, (char)81, (char)125, (char)77, (char)80, (char)211, (char)208, (char)52, (char)255, (char)197, (char)6, (char)18, (char)108, (char)44, (char)237, (char)20, (char)231, (char)121, (char)6, (char)248, (char)203, (char)95, (char)123, (char)147, (char)24, (char)133, (char)234, (char)46, (char)174, (char)208, (char)231, (char)129, (char)207, (char)41, (char)121, (char)220, (char)124, (char)109, (char)254, (char)102, (char)129, (char)122, (char)45, (char)224, (char)41, (char)76, (char)176, (char)145, (char)242, (char)208, (char)203, (char)31, (char)100, (char)246, (char)249, (char)11, (char)92, (char)91, (char)87, (char)185, (char)227, (char)105, (char)125, (char)24, (char)174, (char)84, (char)80, (char)76, (char)238, (char)90, (char)123, (char)41, (char)37, (char)120, (char)129, (char)167, (char)30, (char)8, (char)10, (char)96, (char)107, (char)217, (char)149, (char)107, (char)39, (char)212, (char)172, (char)137, (char)56, (char)15, (char)130, (char)116, (char)25, (char)101, (char)50, (char)161, (char)149, (char)232, (char)49, (char)232, (char)15, (char)214, (char)16, (char)99, (char)8, (char)150, (char)103, (char)203, (char)56, (char)44, (char)128, (char)116, (char)180, (char)80, (char)18, (char)205, (char)14, (char)107, (char)14, (char)193, (char)218, (char)2, (char)89, (char)13, (char)69}));
            assert(pack.target_system_GET() == (char)175);
        });
        DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.payload_SET(new char[] {(char)106, (char)25, (char)178, (char)251, (char)150, (char)15, (char)128, (char)220, (char)92, (char)8, (char)85, (char)29, (char)125, (char)208, (char)242, (char)3, (char)230, (char)109, (char)185, (char)81, (char)185, (char)213, (char)127, (char)218, (char)185, (char)98, (char)196, (char)207, (char)47, (char)114, (char)182, (char)80, (char)106, (char)176, (char)134, (char)67, (char)106, (char)233, (char)205, (char)45, (char)193, (char)159, (char)30, (char)23, (char)230, (char)141, (char)197, (char)132, (char)80, (char)97, (char)79, (char)248, (char)113, (char)192, (char)117, (char)232, (char)69, (char)29, (char)39, (char)181, (char)37, (char)67, (char)126, (char)253, (char)166, (char)113, (char)212, (char)50, (char)67, (char)84, (char)5, (char)209, (char)63, (char)200, (char)203, (char)85, (char)69, (char)238, (char)150, (char)146, (char)96, (char)86, (char)6, (char)131, (char)189, (char)182, (char)76, (char)192, (char)186, (char)103, (char)41, (char)249, (char)248, (char)33, (char)44, (char)71, (char)116, (char)28, (char)142, (char)122, (char)0, (char)59, (char)100, (char)69, (char)100, (char)180, (char)52, (char)137, (char)97, (char)73, (char)0, (char)249, (char)118, (char)63, (char)173, (char)55, (char)243, (char)196, (char)89, (char)11, (char)16, (char)48, (char)174, (char)142, (char)87, (char)75, (char)144, (char)81, (char)125, (char)77, (char)80, (char)211, (char)208, (char)52, (char)255, (char)197, (char)6, (char)18, (char)108, (char)44, (char)237, (char)20, (char)231, (char)121, (char)6, (char)248, (char)203, (char)95, (char)123, (char)147, (char)24, (char)133, (char)234, (char)46, (char)174, (char)208, (char)231, (char)129, (char)207, (char)41, (char)121, (char)220, (char)124, (char)109, (char)254, (char)102, (char)129, (char)122, (char)45, (char)224, (char)41, (char)76, (char)176, (char)145, (char)242, (char)208, (char)203, (char)31, (char)100, (char)246, (char)249, (char)11, (char)92, (char)91, (char)87, (char)185, (char)227, (char)105, (char)125, (char)24, (char)174, (char)84, (char)80, (char)76, (char)238, (char)90, (char)123, (char)41, (char)37, (char)120, (char)129, (char)167, (char)30, (char)8, (char)10, (char)96, (char)107, (char)217, (char)149, (char)107, (char)39, (char)212, (char)172, (char)137, (char)56, (char)15, (char)130, (char)116, (char)25, (char)101, (char)50, (char)161, (char)149, (char)232, (char)49, (char)232, (char)15, (char)214, (char)16, (char)99, (char)8, (char)150, (char)103, (char)203, (char)56, (char)44, (char)128, (char)116, (char)180, (char)80, (char)18, (char)205, (char)14, (char)107, (char)14, (char)193, (char)218, (char)2, (char)89, (char)13, (char)69}, 0) ;
        p110.target_system_SET((char)175) ;
        p110.target_network_SET((char)131) ;
        p110.target_component_SET((char)254) ;
        LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.ts1_GET() == -7059692884396382671L);
            assert(pack.tc1_GET() == 4941472233167114252L);
        });
        DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(-7059692884396382671L) ;
        p111.tc1_SET(4941472233167114252L) ;
        LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 1430936832L);
            assert(pack.time_usec_GET() == 3981924238054727064L);
        });
        DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.time_usec_SET(3981924238054727064L) ;
        p112.seq_SET(1430936832L) ;
        LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.cog_GET() == (char)23226);
            assert(pack.ve_GET() == (short)5781);
            assert(pack.fix_type_GET() == (char)53);
            assert(pack.satellites_visible_GET() == (char)132);
            assert(pack.time_usec_GET() == 2046728676740033867L);
            assert(pack.vd_GET() == (short)25785);
            assert(pack.eph_GET() == (char)43156);
            assert(pack.epv_GET() == (char)64959);
            assert(pack.lon_GET() == 801702038);
            assert(pack.vn_GET() == (short) -7715);
            assert(pack.vel_GET() == (char)43348);
            assert(pack.lat_GET() == 1682766695);
            assert(pack.alt_GET() == 686996407);
        });
        DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.vel_SET((char)43348) ;
        p113.time_usec_SET(2046728676740033867L) ;
        p113.alt_SET(686996407) ;
        p113.fix_type_SET((char)53) ;
        p113.vn_SET((short) -7715) ;
        p113.epv_SET((char)64959) ;
        p113.cog_SET((char)23226) ;
        p113.eph_SET((char)43156) ;
        p113.ve_SET((short)5781) ;
        p113.lat_SET(1682766695) ;
        p113.vd_SET((short)25785) ;
        p113.lon_SET(801702038) ;
        p113.satellites_visible_SET((char)132) ;
        LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integration_time_us_GET() == 290137477L);
            assert(pack.temperature_GET() == (short)9490);
            assert(pack.sensor_id_GET() == (char)208);
            assert(pack.distance_GET() == 5.501684E37F);
            assert(pack.quality_GET() == (char)43);
            assert(pack.time_delta_distance_us_GET() == 3925327314L);
            assert(pack.integrated_x_GET() == 5.522625E37F);
            assert(pack.integrated_xgyro_GET() == -4.476406E37F);
            assert(pack.integrated_ygyro_GET() == 1.1117676E38F);
            assert(pack.integrated_y_GET() == 2.4534673E37F);
            assert(pack.integrated_zgyro_GET() == 1.2428273E38F);
            assert(pack.time_usec_GET() == 9222686473486824631L);
        });
        DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.temperature_SET((short)9490) ;
        p114.integrated_zgyro_SET(1.2428273E38F) ;
        p114.integrated_y_SET(2.4534673E37F) ;
        p114.integrated_ygyro_SET(1.1117676E38F) ;
        p114.integrated_x_SET(5.522625E37F) ;
        p114.sensor_id_SET((char)208) ;
        p114.integrated_xgyro_SET(-4.476406E37F) ;
        p114.distance_SET(5.501684E37F) ;
        p114.time_usec_SET(9222686473486824631L) ;
        p114.time_delta_distance_us_SET(3925327314L) ;
        p114.integration_time_us_SET(290137477L) ;
        p114.quality_SET((char)43) ;
        LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 1609581748);
            assert(pack.ind_airspeed_GET() == (char)31667);
            assert(pack.pitchspeed_GET() == 2.5009132E38F);
            assert(pack.vy_GET() == (short)30783);
            assert(pack.true_airspeed_GET() == (char)52927);
            assert(pack.vz_GET() == (short)19070);
            assert(pack.zacc_GET() == (short)26421);
            assert(pack.vx_GET() == (short)17736);
            assert(pack.rollspeed_GET() == 1.5110093E38F);
            assert(pack.xacc_GET() == (short)21989);
            assert(pack.yacc_GET() == (short) -23165);
            assert(pack.time_usec_GET() == 8122995045757720708L);
            assert(pack.lat_GET() == 674070673);
            assert(pack.alt_GET() == 1182614377);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {-8.3883447E37F, 9.223951E37F, -8.121488E37F, -1.430757E38F}));
            assert(pack.yawspeed_GET() == 6.848776E36F);
        });
        DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.pitchspeed_SET(2.5009132E38F) ;
        p115.yacc_SET((short) -23165) ;
        p115.true_airspeed_SET((char)52927) ;
        p115.zacc_SET((short)26421) ;
        p115.lat_SET(674070673) ;
        p115.ind_airspeed_SET((char)31667) ;
        p115.rollspeed_SET(1.5110093E38F) ;
        p115.vz_SET((short)19070) ;
        p115.alt_SET(1182614377) ;
        p115.yawspeed_SET(6.848776E36F) ;
        p115.time_usec_SET(8122995045757720708L) ;
        p115.vy_SET((short)30783) ;
        p115.lon_SET(1609581748) ;
        p115.vx_SET((short)17736) ;
        p115.xacc_SET((short)21989) ;
        p115.attitude_quaternion_SET(new float[] {-8.3883447E37F, 9.223951E37F, -8.121488E37F, -1.430757E38F}, 0) ;
        LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short)1262);
            assert(pack.zmag_GET() == (short)3027);
            assert(pack.xacc_GET() == (short) -17728);
            assert(pack.ymag_GET() == (short)17816);
            assert(pack.zacc_GET() == (short)8286);
            assert(pack.xmag_GET() == (short) -13955);
            assert(pack.zgyro_GET() == (short) -6376);
            assert(pack.time_boot_ms_GET() == 1604099671L);
            assert(pack.xgyro_GET() == (short) -13711);
            assert(pack.ygyro_GET() == (short)3842);
        });
        DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.zgyro_SET((short) -6376) ;
        p116.zmag_SET((short)3027) ;
        p116.yacc_SET((short)1262) ;
        p116.zacc_SET((short)8286) ;
        p116.xmag_SET((short) -13955) ;
        p116.ymag_SET((short)17816) ;
        p116.time_boot_ms_SET(1604099671L) ;
        p116.xacc_SET((short) -17728) ;
        p116.ygyro_SET((short)3842) ;
        p116.xgyro_SET((short) -13711) ;
        LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)29);
            assert(pack.end_GET() == (char)18446);
            assert(pack.target_system_GET() == (char)138);
            assert(pack.start_GET() == (char)64178);
        });
        DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.end_SET((char)18446) ;
        p117.target_system_SET((char)138) ;
        p117.start_SET((char)64178) ;
        p117.target_component_SET((char)29) ;
        LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.time_utc_GET() == 1419444528L);
            assert(pack.id_GET() == (char)36064);
            assert(pack.last_log_num_GET() == (char)32673);
            assert(pack.num_logs_GET() == (char)30202);
            assert(pack.size_GET() == 3618388607L);
        });
        DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)36064) ;
        p118.num_logs_SET((char)30202) ;
        p118.size_SET(3618388607L) ;
        p118.last_log_num_SET((char)32673) ;
        p118.time_utc_SET(1419444528L) ;
        LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.ofs_GET() == 466380427L);
            assert(pack.id_GET() == (char)5197);
            assert(pack.target_system_GET() == (char)207);
            assert(pack.target_component_GET() == (char)206);
            assert(pack.count_GET() == 2863679044L);
        });
        DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.id_SET((char)5197) ;
        p119.ofs_SET(466380427L) ;
        p119.target_system_SET((char)207) ;
        p119.count_SET(2863679044L) ;
        p119.target_component_SET((char)206) ;
        LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)39486);
            assert(pack.ofs_GET() == 1038185612L);
            assert(pack.count_GET() == (char)12);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)6, (char)211, (char)246, (char)40, (char)252, (char)162, (char)90, (char)139, (char)157, (char)170, (char)133, (char)107, (char)173, (char)162, (char)76, (char)68, (char)34, (char)90, (char)114, (char)53, (char)60, (char)252, (char)119, (char)231, (char)32, (char)31, (char)56, (char)251, (char)34, (char)141, (char)34, (char)203, (char)10, (char)227, (char)219, (char)95, (char)142, (char)138, (char)132, (char)178, (char)230, (char)183, (char)139, (char)13, (char)110, (char)88, (char)178, (char)246, (char)130, (char)51, (char)77, (char)199, (char)209, (char)250, (char)228, (char)142, (char)177, (char)53, (char)102, (char)232, (char)164, (char)227, (char)123, (char)239, (char)78, (char)97, (char)142, (char)226, (char)105, (char)69, (char)33, (char)68, (char)88, (char)131, (char)60, (char)122, (char)62, (char)123, (char)135, (char)84, (char)30, (char)54, (char)203, (char)94, (char)161, (char)140, (char)255, (char)232, (char)247, (char)250}));
        });
        DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.count_SET((char)12) ;
        p120.id_SET((char)39486) ;
        p120.ofs_SET(1038185612L) ;
        p120.data__SET(new char[] {(char)6, (char)211, (char)246, (char)40, (char)252, (char)162, (char)90, (char)139, (char)157, (char)170, (char)133, (char)107, (char)173, (char)162, (char)76, (char)68, (char)34, (char)90, (char)114, (char)53, (char)60, (char)252, (char)119, (char)231, (char)32, (char)31, (char)56, (char)251, (char)34, (char)141, (char)34, (char)203, (char)10, (char)227, (char)219, (char)95, (char)142, (char)138, (char)132, (char)178, (char)230, (char)183, (char)139, (char)13, (char)110, (char)88, (char)178, (char)246, (char)130, (char)51, (char)77, (char)199, (char)209, (char)250, (char)228, (char)142, (char)177, (char)53, (char)102, (char)232, (char)164, (char)227, (char)123, (char)239, (char)78, (char)97, (char)142, (char)226, (char)105, (char)69, (char)33, (char)68, (char)88, (char)131, (char)60, (char)122, (char)62, (char)123, (char)135, (char)84, (char)30, (char)54, (char)203, (char)94, (char)161, (char)140, (char)255, (char)232, (char)247, (char)250}, 0) ;
        LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)178);
            assert(pack.target_system_GET() == (char)21);
        });
        DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)21) ;
        p121.target_component_SET((char)178) ;
        LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)14);
            assert(pack.target_system_GET() == (char)103);
        });
        DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_component_SET((char)14) ;
        p122.target_system_SET((char)103) ;
        LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)23);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)82, (char)145, (char)89, (char)57, (char)173, (char)191, (char)101, (char)21, (char)164, (char)190, (char)130, (char)113, (char)108, (char)48, (char)204, (char)253, (char)253, (char)86, (char)20, (char)8, (char)56, (char)147, (char)144, (char)82, (char)2, (char)173, (char)115, (char)103, (char)249, (char)158, (char)54, (char)56, (char)99, (char)241, (char)232, (char)130, (char)150, (char)204, (char)195, (char)100, (char)147, (char)0, (char)93, (char)8, (char)221, (char)250, (char)245, (char)237, (char)210, (char)122, (char)206, (char)189, (char)21, (char)226, (char)153, (char)248, (char)37, (char)15, (char)6, (char)196, (char)187, (char)148, (char)103, (char)219, (char)26, (char)142, (char)82, (char)36, (char)130, (char)22, (char)173, (char)36, (char)206, (char)189, (char)36, (char)252, (char)194, (char)5, (char)91, (char)83, (char)145, (char)220, (char)154, (char)160, (char)228, (char)163, (char)255, (char)222, (char)9, (char)247, (char)156, (char)174, (char)222, (char)5, (char)38, (char)82, (char)104, (char)0, (char)34, (char)94, (char)204, (char)8, (char)215, (char)76, (char)58, (char)253, (char)78, (char)127, (char)53, (char)43}));
            assert(pack.target_system_GET() == (char)85);
            assert(pack.len_GET() == (char)211);
        });
        DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.len_SET((char)211) ;
        p123.target_component_SET((char)23) ;
        p123.data__SET(new char[] {(char)82, (char)145, (char)89, (char)57, (char)173, (char)191, (char)101, (char)21, (char)164, (char)190, (char)130, (char)113, (char)108, (char)48, (char)204, (char)253, (char)253, (char)86, (char)20, (char)8, (char)56, (char)147, (char)144, (char)82, (char)2, (char)173, (char)115, (char)103, (char)249, (char)158, (char)54, (char)56, (char)99, (char)241, (char)232, (char)130, (char)150, (char)204, (char)195, (char)100, (char)147, (char)0, (char)93, (char)8, (char)221, (char)250, (char)245, (char)237, (char)210, (char)122, (char)206, (char)189, (char)21, (char)226, (char)153, (char)248, (char)37, (char)15, (char)6, (char)196, (char)187, (char)148, (char)103, (char)219, (char)26, (char)142, (char)82, (char)36, (char)130, (char)22, (char)173, (char)36, (char)206, (char)189, (char)36, (char)252, (char)194, (char)5, (char)91, (char)83, (char)145, (char)220, (char)154, (char)160, (char)228, (char)163, (char)255, (char)222, (char)9, (char)247, (char)156, (char)174, (char)222, (char)5, (char)38, (char)82, (char)104, (char)0, (char)34, (char)94, (char)204, (char)8, (char)215, (char)76, (char)58, (char)253, (char)78, (char)127, (char)53, (char)43}, 0) ;
        p123.target_system_SET((char)85) ;
        LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 8542656424603279359L);
            assert(pack.vel_GET() == (char)7846);
            assert(pack.lat_GET() == -325687661);
            assert(pack.satellites_visible_GET() == (char)109);
            assert(pack.dgps_age_GET() == 1307199837L);
            assert(pack.eph_GET() == (char)64622);
            assert(pack.dgps_numch_GET() == (char)193);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
            assert(pack.epv_GET() == (char)29400);
            assert(pack.lon_GET() == 70307628);
            assert(pack.cog_GET() == (char)34829);
            assert(pack.alt_GET() == -7630081);
        });
        DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.alt_SET(-7630081) ;
        p124.lat_SET(-325687661) ;
        p124.eph_SET((char)64622) ;
        p124.lon_SET(70307628) ;
        p124.epv_SET((char)29400) ;
        p124.vel_SET((char)7846) ;
        p124.dgps_numch_SET((char)193) ;
        p124.cog_SET((char)34829) ;
        p124.time_usec_SET(8542656424603279359L) ;
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED) ;
        p124.satellites_visible_SET((char)109) ;
        p124.dgps_age_SET(1307199837L) ;
        LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED);
            assert(pack.Vservo_GET() == (char)21738);
            assert(pack.Vcc_GET() == (char)38852);
        });
        DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_CHANGED) ;
        p125.Vcc_SET((char)38852) ;
        p125.Vservo_SET((char)21738) ;
        LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)14, (char)244, (char)152, (char)114, (char)207, (char)203, (char)201, (char)223, (char)63, (char)160, (char)19, (char)76, (char)216, (char)54, (char)253, (char)50, (char)217, (char)245, (char)184, (char)219, (char)25, (char)89, (char)136, (char)111, (char)189, (char)194, (char)219, (char)183, (char)21, (char)16, (char)68, (char)189, (char)231, (char)47, (char)71, (char)212, (char)72, (char)165, (char)30, (char)173, (char)192, (char)120, (char)80, (char)214, (char)34, (char)86, (char)236, (char)174, (char)199, (char)18, (char)103, (char)199, (char)30, (char)140, (char)59, (char)53, (char)69, (char)119, (char)187, (char)16, (char)82, (char)197, (char)182, (char)251, (char)182, (char)89, (char)187, (char)221, (char)72, (char)205}));
            assert(pack.timeout_GET() == (char)4832);
            assert(pack.count_GET() == (char)99);
            assert(pack.baudrate_GET() == 2480300243L);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2);
        });
        DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.baudrate_SET(2480300243L) ;
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_MULTI) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM2) ;
        p126.timeout_SET((char)4832) ;
        p126.data__SET(new char[] {(char)14, (char)244, (char)152, (char)114, (char)207, (char)203, (char)201, (char)223, (char)63, (char)160, (char)19, (char)76, (char)216, (char)54, (char)253, (char)50, (char)217, (char)245, (char)184, (char)219, (char)25, (char)89, (char)136, (char)111, (char)189, (char)194, (char)219, (char)183, (char)21, (char)16, (char)68, (char)189, (char)231, (char)47, (char)71, (char)212, (char)72, (char)165, (char)30, (char)173, (char)192, (char)120, (char)80, (char)214, (char)34, (char)86, (char)236, (char)174, (char)199, (char)18, (char)103, (char)199, (char)30, (char)140, (char)59, (char)53, (char)69, (char)119, (char)187, (char)16, (char)82, (char)197, (char)182, (char)251, (char)182, (char)89, (char)187, (char)221, (char)72, (char)205}, 0) ;
        p126.count_SET((char)99) ;
        LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_b_mm_GET() == 1567194764);
            assert(pack.rtk_receiver_id_GET() == (char)156);
            assert(pack.baseline_a_mm_GET() == -395888746);
            assert(pack.time_last_baseline_ms_GET() == 2436267358L);
            assert(pack.wn_GET() == (char)1927);
            assert(pack.baseline_coords_type_GET() == (char)4);
            assert(pack.iar_num_hypotheses_GET() == 1563754814);
            assert(pack.tow_GET() == 325615171L);
            assert(pack.rtk_rate_GET() == (char)34);
            assert(pack.rtk_health_GET() == (char)211);
            assert(pack.baseline_c_mm_GET() == -1071096720);
            assert(pack.accuracy_GET() == 2933106676L);
            assert(pack.nsats_GET() == (char)167);
        });
        DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.wn_SET((char)1927) ;
        p127.rtk_receiver_id_SET((char)156) ;
        p127.baseline_b_mm_SET(1567194764) ;
        p127.tow_SET(325615171L) ;
        p127.nsats_SET((char)167) ;
        p127.iar_num_hypotheses_SET(1563754814) ;
        p127.baseline_coords_type_SET((char)4) ;
        p127.baseline_c_mm_SET(-1071096720) ;
        p127.baseline_a_mm_SET(-395888746) ;
        p127.rtk_rate_SET((char)34) ;
        p127.rtk_health_SET((char)211) ;
        p127.accuracy_SET(2933106676L) ;
        p127.time_last_baseline_ms_SET(2436267358L) ;
        LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.baseline_a_mm_GET() == -1611108275);
            assert(pack.iar_num_hypotheses_GET() == 595134404);
            assert(pack.baseline_c_mm_GET() == 349931196);
            assert(pack.nsats_GET() == (char)109);
            assert(pack.time_last_baseline_ms_GET() == 3398405137L);
            assert(pack.rtk_health_GET() == (char)205);
            assert(pack.wn_GET() == (char)58073);
            assert(pack.accuracy_GET() == 348732314L);
            assert(pack.tow_GET() == 4262139047L);
            assert(pack.baseline_b_mm_GET() == 1785254580);
            assert(pack.baseline_coords_type_GET() == (char)61);
            assert(pack.rtk_rate_GET() == (char)64);
            assert(pack.rtk_receiver_id_GET() == (char)174);
        });
        DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.baseline_a_mm_SET(-1611108275) ;
        p128.baseline_coords_type_SET((char)61) ;
        p128.nsats_SET((char)109) ;
        p128.tow_SET(4262139047L) ;
        p128.baseline_c_mm_SET(349931196) ;
        p128.baseline_b_mm_SET(1785254580) ;
        p128.rtk_rate_SET((char)64) ;
        p128.time_last_baseline_ms_SET(3398405137L) ;
        p128.rtk_receiver_id_SET((char)174) ;
        p128.wn_SET((char)58073) ;
        p128.rtk_health_SET((char)205) ;
        p128.accuracy_SET(348732314L) ;
        p128.iar_num_hypotheses_SET(595134404) ;
        LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.zacc_GET() == (short)4453);
            assert(pack.xmag_GET() == (short) -28756);
            assert(pack.ygyro_GET() == (short)24552);
            assert(pack.yacc_GET() == (short) -16614);
            assert(pack.zmag_GET() == (short) -6442);
            assert(pack.zgyro_GET() == (short)24102);
            assert(pack.ymag_GET() == (short)32121);
            assert(pack.xacc_GET() == (short)27955);
            assert(pack.xgyro_GET() == (short)21582);
            assert(pack.time_boot_ms_GET() == 2083718837L);
        });
        DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.xgyro_SET((short)21582) ;
        p129.ymag_SET((short)32121) ;
        p129.xmag_SET((short) -28756) ;
        p129.ygyro_SET((short)24552) ;
        p129.yacc_SET((short) -16614) ;
        p129.time_boot_ms_SET(2083718837L) ;
        p129.zgyro_SET((short)24102) ;
        p129.zmag_SET((short) -6442) ;
        p129.zacc_SET((short)4453) ;
        p129.xacc_SET((short)27955) ;
        LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.height_GET() == (char)40673);
            assert(pack.type_GET() == (char)183);
            assert(pack.payload_GET() == (char)216);
            assert(pack.packets_GET() == (char)64686);
            assert(pack.jpg_quality_GET() == (char)131);
            assert(pack.width_GET() == (char)12492);
            assert(pack.size_GET() == 4018874233L);
        });
        DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.packets_SET((char)64686) ;
        p130.size_SET(4018874233L) ;
        p130.height_SET((char)40673) ;
        p130.type_SET((char)183) ;
        p130.jpg_quality_SET((char)131) ;
        p130.payload_SET((char)216) ;
        p130.width_SET((char)12492) ;
        LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)103, (char)230, (char)214, (char)187, (char)6, (char)195, (char)32, (char)3, (char)60, (char)222, (char)113, (char)119, (char)249, (char)151, (char)9, (char)213, (char)36, (char)37, (char)113, (char)136, (char)250, (char)216, (char)91, (char)251, (char)140, (char)94, (char)95, (char)179, (char)101, (char)163, (char)216, (char)139, (char)217, (char)202, (char)145, (char)189, (char)71, (char)233, (char)149, (char)21, (char)219, (char)220, (char)41, (char)184, (char)19, (char)215, (char)208, (char)55, (char)156, (char)10, (char)251, (char)114, (char)53, (char)140, (char)120, (char)232, (char)70, (char)33, (char)105, (char)168, (char)131, (char)146, (char)105, (char)157, (char)43, (char)123, (char)81, (char)17, (char)36, (char)206, (char)158, (char)113, (char)233, (char)222, (char)209, (char)127, (char)82, (char)226, (char)19, (char)10, (char)37, (char)95, (char)65, (char)222, (char)225, (char)141, (char)8, (char)85, (char)52, (char)98, (char)224, (char)181, (char)98, (char)67, (char)196, (char)200, (char)134, (char)45, (char)8, (char)100, (char)151, (char)191, (char)4, (char)18, (char)212, (char)179, (char)199, (char)248, (char)128, (char)40, (char)81, (char)139, (char)62, (char)156, (char)1, (char)140, (char)76, (char)11, (char)51, (char)243, (char)172, (char)175, (char)58, (char)78, (char)232, (char)131, (char)37, (char)156, (char)177, (char)18, (char)160, (char)132, (char)118, (char)18, (char)159, (char)98, (char)94, (char)38, (char)91, (char)151, (char)75, (char)226, (char)227, (char)195, (char)86, (char)137, (char)84, (char)91, (char)98, (char)124, (char)108, (char)14, (char)166, (char)100, (char)12, (char)200, (char)34, (char)245, (char)194, (char)206, (char)230, (char)43, (char)21, (char)249, (char)59, (char)63, (char)137, (char)119, (char)17, (char)134, (char)73, (char)200, (char)74, (char)52, (char)47, (char)244, (char)27, (char)210, (char)116, (char)140, (char)6, (char)93, (char)242, (char)198, (char)34, (char)93, (char)195, (char)27, (char)231, (char)143, (char)254, (char)233, (char)100, (char)136, (char)43, (char)105, (char)83, (char)40, (char)131, (char)242, (char)153, (char)52, (char)178, (char)167, (char)135, (char)197, (char)51, (char)68, (char)157, (char)69, (char)23, (char)20, (char)9, (char)217, (char)183, (char)19, (char)193, (char)154, (char)184, (char)224, (char)42, (char)117, (char)245, (char)121, (char)213, (char)208, (char)243, (char)213, (char)176, (char)53, (char)189, (char)79, (char)245, (char)246, (char)179, (char)1, (char)214, (char)169, (char)37, (char)110, (char)31, (char)61, (char)169, (char)102, (char)163, (char)81, (char)251, (char)22, (char)126, (char)36, (char)6, (char)87, (char)55}));
            assert(pack.seqnr_GET() == (char)50353);
        });
        DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.data__SET(new char[] {(char)103, (char)230, (char)214, (char)187, (char)6, (char)195, (char)32, (char)3, (char)60, (char)222, (char)113, (char)119, (char)249, (char)151, (char)9, (char)213, (char)36, (char)37, (char)113, (char)136, (char)250, (char)216, (char)91, (char)251, (char)140, (char)94, (char)95, (char)179, (char)101, (char)163, (char)216, (char)139, (char)217, (char)202, (char)145, (char)189, (char)71, (char)233, (char)149, (char)21, (char)219, (char)220, (char)41, (char)184, (char)19, (char)215, (char)208, (char)55, (char)156, (char)10, (char)251, (char)114, (char)53, (char)140, (char)120, (char)232, (char)70, (char)33, (char)105, (char)168, (char)131, (char)146, (char)105, (char)157, (char)43, (char)123, (char)81, (char)17, (char)36, (char)206, (char)158, (char)113, (char)233, (char)222, (char)209, (char)127, (char)82, (char)226, (char)19, (char)10, (char)37, (char)95, (char)65, (char)222, (char)225, (char)141, (char)8, (char)85, (char)52, (char)98, (char)224, (char)181, (char)98, (char)67, (char)196, (char)200, (char)134, (char)45, (char)8, (char)100, (char)151, (char)191, (char)4, (char)18, (char)212, (char)179, (char)199, (char)248, (char)128, (char)40, (char)81, (char)139, (char)62, (char)156, (char)1, (char)140, (char)76, (char)11, (char)51, (char)243, (char)172, (char)175, (char)58, (char)78, (char)232, (char)131, (char)37, (char)156, (char)177, (char)18, (char)160, (char)132, (char)118, (char)18, (char)159, (char)98, (char)94, (char)38, (char)91, (char)151, (char)75, (char)226, (char)227, (char)195, (char)86, (char)137, (char)84, (char)91, (char)98, (char)124, (char)108, (char)14, (char)166, (char)100, (char)12, (char)200, (char)34, (char)245, (char)194, (char)206, (char)230, (char)43, (char)21, (char)249, (char)59, (char)63, (char)137, (char)119, (char)17, (char)134, (char)73, (char)200, (char)74, (char)52, (char)47, (char)244, (char)27, (char)210, (char)116, (char)140, (char)6, (char)93, (char)242, (char)198, (char)34, (char)93, (char)195, (char)27, (char)231, (char)143, (char)254, (char)233, (char)100, (char)136, (char)43, (char)105, (char)83, (char)40, (char)131, (char)242, (char)153, (char)52, (char)178, (char)167, (char)135, (char)197, (char)51, (char)68, (char)157, (char)69, (char)23, (char)20, (char)9, (char)217, (char)183, (char)19, (char)193, (char)154, (char)184, (char)224, (char)42, (char)117, (char)245, (char)121, (char)213, (char)208, (char)243, (char)213, (char)176, (char)53, (char)189, (char)79, (char)245, (char)246, (char)179, (char)1, (char)214, (char)169, (char)37, (char)110, (char)31, (char)61, (char)169, (char)102, (char)163, (char)81, (char)251, (char)22, (char)126, (char)36, (char)6, (char)87, (char)55}, 0) ;
        p131.seqnr_SET((char)50353) ;
        LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN);
            assert(pack.max_distance_GET() == (char)57216);
            assert(pack.current_distance_GET() == (char)34484);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_YAW_135);
            assert(pack.time_boot_ms_GET() == 2219521690L);
            assert(pack.id_GET() == (char)200);
            assert(pack.covariance_GET() == (char)60);
            assert(pack.min_distance_GET() == (char)28780);
        });
        DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.current_distance_SET((char)34484) ;
        p132.id_SET((char)200) ;
        p132.min_distance_SET((char)28780) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN) ;
        p132.max_distance_SET((char)57216) ;
        p132.covariance_SET((char)60) ;
        p132.time_boot_ms_SET(2219521690L) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_90_YAW_135) ;
        LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.mask_GET() == 7104305831336716890L);
            assert(pack.grid_spacing_GET() == (char)5029);
            assert(pack.lon_GET() == -2056669947);
            assert(pack.lat_GET() == -248999280);
        });
        DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.lon_SET(-2056669947) ;
        p133.grid_spacing_SET((char)5029) ;
        p133.mask_SET(7104305831336716890L) ;
        p133.lat_SET(-248999280) ;
        LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short)19284, (short) -23534, (short) -24632, (short)30705, (short)10566, (short)13320, (short) -1321, (short)8690, (short)5268, (short)31100, (short)18876, (short)25576, (short) -32019, (short) -12665, (short)18327, (short) -14085}));
            assert(pack.lat_GET() == 681076898);
            assert(pack.gridbit_GET() == (char)103);
            assert(pack.grid_spacing_GET() == (char)23030);
            assert(pack.lon_GET() == -966710320);
        });
        DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.gridbit_SET((char)103) ;
        p134.data__SET(new short[] {(short)19284, (short) -23534, (short) -24632, (short)30705, (short)10566, (short)13320, (short) -1321, (short)8690, (short)5268, (short)31100, (short)18876, (short)25576, (short) -32019, (short) -12665, (short)18327, (short) -14085}, 0) ;
        p134.lat_SET(681076898) ;
        p134.lon_SET(-966710320) ;
        p134.grid_spacing_SET((char)23030) ;
        LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 544989945);
            assert(pack.lat_GET() == 2015301234);
        });
        DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lon_SET(544989945) ;
        p135.lat_SET(2015301234) ;
        LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == 554917081);
            assert(pack.spacing_GET() == (char)21795);
            assert(pack.loaded_GET() == (char)32053);
            assert(pack.pending_GET() == (char)39453);
            assert(pack.terrain_height_GET() == 7.855981E37F);
            assert(pack.lat_GET() == -1606812200);
            assert(pack.current_height_GET() == 2.9134258E38F);
        });
        DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.terrain_height_SET(7.855981E37F) ;
        p136.pending_SET((char)39453) ;
        p136.loaded_SET((char)32053) ;
        p136.current_height_SET(2.9134258E38F) ;
        p136.lat_SET(-1606812200) ;
        p136.lon_SET(554917081) ;
        p136.spacing_SET((char)21795) ;
        LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == 3.1365428E38F);
            assert(pack.temperature_GET() == (short) -25741);
            assert(pack.press_diff_GET() == -3.366809E38F);
            assert(pack.time_boot_ms_GET() == 4282553114L);
        });
        DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.time_boot_ms_SET(4282553114L) ;
        p137.temperature_SET((short) -25741) ;
        p137.press_diff_SET(-3.366809E38F) ;
        p137.press_abs_SET(3.1365428E38F) ;
        LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.5092753E38F, -3.2331984E38F, 1.2158082E38F, 3.2952431E38F}));
            assert(pack.z_GET() == -3.021538E38F);
            assert(pack.time_usec_GET() == 8338516195720956552L);
            assert(pack.x_GET() == 2.8120786E38F);
            assert(pack.y_GET() == 7.3378335E37F);
        });
        DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.q_SET(new float[] {-1.5092753E38F, -3.2331984E38F, 1.2158082E38F, 3.2952431E38F}, 0) ;
        p138.x_SET(2.8120786E38F) ;
        p138.z_SET(-3.021538E38F) ;
        p138.time_usec_SET(8338516195720956552L) ;
        p138.y_SET(7.3378335E37F) ;
        LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)54);
            assert(pack.target_system_GET() == (char)35);
            assert(pack.time_usec_GET() == 6557518364604442698L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.0289271E38F, 1.2194496E38F, 4.2668343E37F, 6.2363816E37F, -1.2808267E38F, -3.04652E38F, 3.376018E38F, -2.0688313E38F}));
            assert(pack.group_mlx_GET() == (char)142);
        });
        DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.target_system_SET((char)35) ;
        p139.controls_SET(new float[] {1.0289271E38F, 1.2194496E38F, 4.2668343E37F, 6.2363816E37F, -1.2808267E38F, -3.04652E38F, 3.376018E38F, -2.0688313E38F}, 0) ;
        p139.target_component_SET((char)54) ;
        p139.time_usec_SET(6557518364604442698L) ;
        p139.group_mlx_SET((char)142) ;
        LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {2.9289747E38F, -3.3047148E38F, -1.6243053E38F, 2.9553645E38F, 1.0746581E37F, -7.387177E37F, 2.185508E38F, 8.017657E37F}));
            assert(pack.time_usec_GET() == 5484443516116760406L);
            assert(pack.group_mlx_GET() == (char)110);
        });
        DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.group_mlx_SET((char)110) ;
        p140.controls_SET(new float[] {2.9289747E38F, -3.3047148E38F, -1.6243053E38F, 2.9553645E38F, 1.0746581E37F, -7.387177E37F, 2.185508E38F, 8.017657E37F}, 0) ;
        p140.time_usec_SET(5484443516116760406L) ;
        LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_terrain_GET() == 4.898797E37F);
            assert(pack.altitude_relative_GET() == 1.576011E37F);
            assert(pack.time_usec_GET() == 2286723944922725085L);
            assert(pack.altitude_local_GET() == -2.386881E38F);
            assert(pack.altitude_monotonic_GET() == -2.8166933E38F);
            assert(pack.altitude_amsl_GET() == -1.4479911E38F);
            assert(pack.bottom_clearance_GET() == -1.5254394E38F);
        });
        DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_local_SET(-2.386881E38F) ;
        p141.altitude_terrain_SET(4.898797E37F) ;
        p141.altitude_relative_SET(1.576011E37F) ;
        p141.time_usec_SET(2286723944922725085L) ;
        p141.bottom_clearance_SET(-1.5254394E38F) ;
        p141.altitude_monotonic_SET(-2.8166933E38F) ;
        p141.altitude_amsl_SET(-1.4479911E38F) ;
        LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.request_id_GET() == (char)30);
            assert(pack.uri_type_GET() == (char)11);
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)155, (char)5, (char)174, (char)187, (char)2, (char)223, (char)36, (char)175, (char)204, (char)156, (char)10, (char)176, (char)85, (char)238, (char)20, (char)189, (char)67, (char)242, (char)210, (char)20, (char)55, (char)84, (char)10, (char)234, (char)204, (char)223, (char)185, (char)233, (char)178, (char)18, (char)128, (char)235, (char)228, (char)37, (char)217, (char)27, (char)119, (char)251, (char)75, (char)1, (char)76, (char)78, (char)177, (char)9, (char)18, (char)222, (char)212, (char)244, (char)131, (char)239, (char)44, (char)189, (char)144, (char)48, (char)243, (char)73, (char)82, (char)64, (char)165, (char)149, (char)95, (char)55, (char)92, (char)237, (char)206, (char)217, (char)33, (char)30, (char)77, (char)11, (char)252, (char)244, (char)231, (char)204, (char)251, (char)55, (char)235, (char)77, (char)234, (char)255, (char)72, (char)224, (char)109, (char)116, (char)21, (char)90, (char)230, (char)14, (char)66, (char)16, (char)89, (char)85, (char)173, (char)242, (char)157, (char)8, (char)165, (char)105, (char)236, (char)148, (char)13, (char)174, (char)199, (char)144, (char)101, (char)17, (char)25, (char)164, (char)94, (char)45, (char)60, (char)174, (char)42, (char)157, (char)109, (char)8, (char)33, (char)19, (char)65, (char)252}));
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)21, (char)123, (char)71, (char)240, (char)108, (char)227, (char)156, (char)36, (char)150, (char)172, (char)23, (char)191, (char)110, (char)87, (char)60, (char)158, (char)177, (char)207, (char)77, (char)94, (char)118, (char)152, (char)110, (char)127, (char)95, (char)126, (char)8, (char)107, (char)22, (char)193, (char)59, (char)6, (char)150, (char)122, (char)109, (char)29, (char)46, (char)195, (char)22, (char)62, (char)174, (char)181, (char)74, (char)124, (char)226, (char)52, (char)134, (char)6, (char)157, (char)133, (char)207, (char)179, (char)193, (char)247, (char)217, (char)183, (char)120, (char)168, (char)167, (char)188, (char)47, (char)236, (char)172, (char)248, (char)72, (char)164, (char)201, (char)202, (char)252, (char)80, (char)113, (char)244, (char)28, (char)13, (char)73, (char)31, (char)118, (char)162, (char)142, (char)67, (char)65, (char)142, (char)53, (char)137, (char)162, (char)212, (char)166, (char)80, (char)241, (char)124, (char)212, (char)183, (char)1, (char)105, (char)44, (char)151, (char)211, (char)180, (char)204, (char)46, (char)175, (char)194, (char)15, (char)212, (char)54, (char)115, (char)197, (char)5, (char)176, (char)64, (char)253, (char)138, (char)33, (char)222, (char)57, (char)153, (char)173, (char)241, (char)178, (char)135}));
            assert(pack.transfer_type_GET() == (char)144);
        });
        DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.storage_SET(new char[] {(char)21, (char)123, (char)71, (char)240, (char)108, (char)227, (char)156, (char)36, (char)150, (char)172, (char)23, (char)191, (char)110, (char)87, (char)60, (char)158, (char)177, (char)207, (char)77, (char)94, (char)118, (char)152, (char)110, (char)127, (char)95, (char)126, (char)8, (char)107, (char)22, (char)193, (char)59, (char)6, (char)150, (char)122, (char)109, (char)29, (char)46, (char)195, (char)22, (char)62, (char)174, (char)181, (char)74, (char)124, (char)226, (char)52, (char)134, (char)6, (char)157, (char)133, (char)207, (char)179, (char)193, (char)247, (char)217, (char)183, (char)120, (char)168, (char)167, (char)188, (char)47, (char)236, (char)172, (char)248, (char)72, (char)164, (char)201, (char)202, (char)252, (char)80, (char)113, (char)244, (char)28, (char)13, (char)73, (char)31, (char)118, (char)162, (char)142, (char)67, (char)65, (char)142, (char)53, (char)137, (char)162, (char)212, (char)166, (char)80, (char)241, (char)124, (char)212, (char)183, (char)1, (char)105, (char)44, (char)151, (char)211, (char)180, (char)204, (char)46, (char)175, (char)194, (char)15, (char)212, (char)54, (char)115, (char)197, (char)5, (char)176, (char)64, (char)253, (char)138, (char)33, (char)222, (char)57, (char)153, (char)173, (char)241, (char)178, (char)135}, 0) ;
        p142.request_id_SET((char)30) ;
        p142.uri_type_SET((char)11) ;
        p142.transfer_type_SET((char)144) ;
        p142.uri_SET(new char[] {(char)155, (char)5, (char)174, (char)187, (char)2, (char)223, (char)36, (char)175, (char)204, (char)156, (char)10, (char)176, (char)85, (char)238, (char)20, (char)189, (char)67, (char)242, (char)210, (char)20, (char)55, (char)84, (char)10, (char)234, (char)204, (char)223, (char)185, (char)233, (char)178, (char)18, (char)128, (char)235, (char)228, (char)37, (char)217, (char)27, (char)119, (char)251, (char)75, (char)1, (char)76, (char)78, (char)177, (char)9, (char)18, (char)222, (char)212, (char)244, (char)131, (char)239, (char)44, (char)189, (char)144, (char)48, (char)243, (char)73, (char)82, (char)64, (char)165, (char)149, (char)95, (char)55, (char)92, (char)237, (char)206, (char)217, (char)33, (char)30, (char)77, (char)11, (char)252, (char)244, (char)231, (char)204, (char)251, (char)55, (char)235, (char)77, (char)234, (char)255, (char)72, (char)224, (char)109, (char)116, (char)21, (char)90, (char)230, (char)14, (char)66, (char)16, (char)89, (char)85, (char)173, (char)242, (char)157, (char)8, (char)165, (char)105, (char)236, (char)148, (char)13, (char)174, (char)199, (char)144, (char)101, (char)17, (char)25, (char)164, (char)94, (char)45, (char)60, (char)174, (char)42, (char)157, (char)109, (char)8, (char)33, (char)19, (char)65, (char)252}, 0) ;
        LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3182131947L);
            assert(pack.temperature_GET() == (short) -6177);
            assert(pack.press_abs_GET() == -2.2148805E38F);
            assert(pack.press_diff_GET() == 9.667187E37F);
        });
        DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(3182131947L) ;
        p143.temperature_SET((short) -6177) ;
        p143.press_abs_SET(-2.2148805E38F) ;
        p143.press_diff_SET(9.667187E37F) ;
        LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.timestamp_GET() == 7989895909891695783L);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {2.389886E38F, -2.7508288E38F, 7.1768356E36F}));
            assert(pack.lon_GET() == -865134274);
            assert(pack.est_capabilities_GET() == (char)37);
            assert(pack.alt_GET() == -9.109032E37F);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {1.2600623E38F, -3.5568012E37F, -2.6678794E38F}));
            assert(pack.custom_state_GET() == 8432411981395000406L);
            assert(Arrays.equals(pack.vel_GET(),  new float[] {2.1227827E38F, 2.263293E38F, -2.4935509E38F}));
            assert(pack.lat_GET() == -1476250418);
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {1.4636857E38F, -1.2251747E38F, 9.977319E37F, 2.2856716E38F}));
            assert(Arrays.equals(pack.rates_GET(),  new float[] {-2.2021724E38F, 1.1845921E38F, -4.773656E37F}));
        });
        DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.rates_SET(new float[] {-2.2021724E38F, 1.1845921E38F, -4.773656E37F}, 0) ;
        p144.attitude_q_SET(new float[] {1.4636857E38F, -1.2251747E38F, 9.977319E37F, 2.2856716E38F}, 0) ;
        p144.est_capabilities_SET((char)37) ;
        p144.custom_state_SET(8432411981395000406L) ;
        p144.acc_SET(new float[] {2.389886E38F, -2.7508288E38F, 7.1768356E36F}, 0) ;
        p144.lon_SET(-865134274) ;
        p144.timestamp_SET(7989895909891695783L) ;
        p144.alt_SET(-9.109032E37F) ;
        p144.vel_SET(new float[] {2.1227827E38F, 2.263293E38F, -2.4935509E38F}, 0) ;
        p144.position_cov_SET(new float[] {1.2600623E38F, -3.5568012E37F, -2.6678794E38F}, 0) ;
        p144.lat_SET(-1476250418) ;
        LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == 7.3836403E37F);
            assert(pack.y_vel_GET() == 3.7074077E37F);
            assert(pack.x_vel_GET() == -2.9673719E38F);
            assert(pack.roll_rate_GET() == 1.4973384E38F);
            assert(pack.z_acc_GET() == 1.1599703E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.4512848E38F, -4.351558E37F, 3.0809465E38F, -2.5512426E38F}));
            assert(pack.y_pos_GET() == 1.7485834E38F);
            assert(pack.x_acc_GET() == -3.17412E38F);
            assert(pack.airspeed_GET() == 7.7075677E37F);
            assert(pack.x_pos_GET() == -2.0098746E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {-1.8683537E38F, 1.0905699E38F, 5.893286E37F}));
            assert(pack.time_usec_GET() == 6357063104624122947L);
            assert(pack.z_vel_GET() == -4.517699E37F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {2.9580925E38F, -3.613822E37F, 1.5702386E38F}));
            assert(pack.y_acc_GET() == -1.2952485E38F);
            assert(pack.pitch_rate_GET() == 1.6315275E38F);
            assert(pack.z_pos_GET() == 1.5806568E38F);
        });
        DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.pos_variance_SET(new float[] {-1.8683537E38F, 1.0905699E38F, 5.893286E37F}, 0) ;
        p146.y_vel_SET(3.7074077E37F) ;
        p146.vel_variance_SET(new float[] {2.9580925E38F, -3.613822E37F, 1.5702386E38F}, 0) ;
        p146.roll_rate_SET(1.4973384E38F) ;
        p146.airspeed_SET(7.7075677E37F) ;
        p146.z_pos_SET(1.5806568E38F) ;
        p146.x_vel_SET(-2.9673719E38F) ;
        p146.x_pos_SET(-2.0098746E38F) ;
        p146.z_vel_SET(-4.517699E37F) ;
        p146.q_SET(new float[] {-2.4512848E38F, -4.351558E37F, 3.0809465E38F, -2.5512426E38F}, 0) ;
        p146.yaw_rate_SET(7.3836403E37F) ;
        p146.time_usec_SET(6357063104624122947L) ;
        p146.x_acc_SET(-3.17412E38F) ;
        p146.pitch_rate_SET(1.6315275E38F) ;
        p146.y_acc_SET(-1.2952485E38F) ;
        p146.y_pos_SET(1.7485834E38F) ;
        p146.z_acc_SET(1.1599703E38F) ;
        LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)55);
            assert(pack.current_consumed_GET() == 199462476);
            assert(pack.temperature_GET() == (short)22632);
            assert(pack.energy_consumed_GET() == -1486753113);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)58551, (char)25678, (char)45951, (char)56125, (char)41983, (char)64319, (char)61351, (char)4215, (char)53615, (char)32228}));
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS);
            assert(pack.current_battery_GET() == (short)31771);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE);
            assert(pack.battery_remaining_GET() == (byte) - 42);
        });
        DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.current_battery_SET((short)31771) ;
        p147.temperature_SET((short)22632) ;
        p147.battery_remaining_SET((byte) - 42) ;
        p147.id_SET((char)55) ;
        p147.current_consumed_SET(199462476) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_AVIONICS) ;
        p147.energy_consumed_SET(-1486753113) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LIFE) ;
        p147.voltages_SET(new char[] {(char)58551, (char)25678, (char)45951, (char)56125, (char)41983, (char)64319, (char)61351, (char)4215, (char)53615, (char)32228}, 0) ;
        LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.board_version_GET() == 3039895794L);
            assert(pack.flight_sw_version_GET() == 4287306698L);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)53, (char)125, (char)34, (char)237, (char)131, (char)154, (char)179, (char)133}));
            assert(pack.vendor_id_GET() == (char)3375);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)135, (char)160, (char)66, (char)94, (char)179, (char)83, (char)169, (char)36}));
            assert(pack.uid_GET() == 259789244326309824L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)65, (char)152, (char)174, (char)127, (char)127, (char)76, (char)145, (char)17, (char)238, (char)32, (char)65, (char)204, (char)207, (char)124, (char)68, (char)81, (char)73, (char)219}));
            assert(pack.middleware_sw_version_GET() == 84205084L);
            assert(pack.product_id_GET() == (char)28231);
            assert(pack.capabilities_GET() == MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN);
            assert(pack.os_sw_version_GET() == 4164119218L);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)7, (char)72, (char)163, (char)139, (char)253, (char)17, (char)56, (char)142}));
        });
        DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.uid2_SET(new char[] {(char)65, (char)152, (char)174, (char)127, (char)127, (char)76, (char)145, (char)17, (char)238, (char)32, (char)65, (char)204, (char)207, (char)124, (char)68, (char)81, (char)73, (char)219}, 0, PH) ;
        p148.os_custom_version_SET(new char[] {(char)53, (char)125, (char)34, (char)237, (char)131, (char)154, (char)179, (char)133}, 0) ;
        p148.board_version_SET(3039895794L) ;
        p148.flight_custom_version_SET(new char[] {(char)135, (char)160, (char)66, (char)94, (char)179, (char)83, (char)169, (char)36}, 0) ;
        p148.os_sw_version_SET(4164119218L) ;
        p148.uid_SET(259789244326309824L) ;
        p148.flight_sw_version_SET(4287306698L) ;
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_TERRAIN) ;
        p148.middleware_custom_version_SET(new char[] {(char)7, (char)72, (char)163, (char)139, (char)253, (char)17, (char)56, (char)142}, 0) ;
        p148.product_id_SET((char)28231) ;
        p148.middleware_sw_version_SET(84205084L) ;
        p148.vendor_id_SET((char)3375) ;
        LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.distance_GET() == -4.7052853E37F);
            assert(pack.angle_x_GET() == -2.410523E38F);
            assert(pack.z_TRY(ph) == 2.4241658E38F);
            assert(pack.size_x_GET() == -1.1828178E38F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {2.0859318E38F, -5.2444303E37F, -4.3439176E37F, -1.0701732E37F}));
            assert(pack.y_TRY(ph) == 3.185223E37F);
            assert(pack.angle_y_GET() == -5.69512E37F);
            assert(pack.size_y_GET() == -9.065532E37F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL);
            assert(pack.target_num_GET() == (char)199);
            assert(pack.x_TRY(ph) == -5.0111785E37F);
            assert(pack.time_usec_GET() == 4896839558874568322L);
            assert(pack.position_valid_TRY(ph) == (char)158);
        });
        DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.size_x_SET(-1.1828178E38F) ;
        p149.time_usec_SET(4896839558874568322L) ;
        p149.x_SET(-5.0111785E37F, PH) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_VISION_FIDUCIAL) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p149.size_y_SET(-9.065532E37F) ;
        p149.angle_y_SET(-5.69512E37F) ;
        p149.y_SET(3.185223E37F, PH) ;
        p149.q_SET(new float[] {2.0859318E38F, -5.2444303E37F, -4.3439176E37F, -1.0701732E37F}, 0, PH) ;
        p149.position_valid_SET((char)158, PH) ;
        p149.angle_x_SET(-2.410523E38F) ;
        p149.z_SET(2.4241658E38F, PH) ;
        p149.distance_SET(-4.7052853E37F) ;
        p149.target_num_SET((char)199) ;
        LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.mag_ratio_GET() == 2.1024428E37F);
            assert(pack.tas_ratio_GET() == 2.7172162E38F);
            assert(pack.pos_vert_accuracy_GET() == -3.0246131E38F);
            assert(pack.flags_GET() == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ);
            assert(pack.hagl_ratio_GET() == -2.618288E38F);
            assert(pack.vel_ratio_GET() == -2.1706892E38F);
            assert(pack.time_usec_GET() == 5136236725681260815L);
            assert(pack.pos_horiz_ratio_GET() == -2.5034396E38F);
            assert(pack.pos_horiz_accuracy_GET() == 3.064125E38F);
            assert(pack.pos_vert_ratio_GET() == 2.0565329E38F);
        });
        DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.pos_horiz_accuracy_SET(3.064125E38F) ;
        p230.hagl_ratio_SET(-2.618288E38F) ;
        p230.pos_horiz_ratio_SET(-2.5034396E38F) ;
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_VELOCITY_HORIZ) ;
        p230.pos_vert_ratio_SET(2.0565329E38F) ;
        p230.pos_vert_accuracy_SET(-3.0246131E38F) ;
        p230.mag_ratio_SET(2.1024428E37F) ;
        p230.time_usec_SET(5136236725681260815L) ;
        p230.tas_ratio_SET(2.7172162E38F) ;
        p230.vel_ratio_SET(-2.1706892E38F) ;
        LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_z_GET() == 2.9657899E38F);
            assert(pack.vert_accuracy_GET() == -2.9310175E38F);
            assert(pack.var_vert_GET() == -9.995999E37F);
            assert(pack.horiz_accuracy_GET() == 1.430476E37F);
            assert(pack.wind_alt_GET() == -1.6510262E38F);
            assert(pack.var_horiz_GET() == -7.810207E37F);
            assert(pack.wind_y_GET() == 2.9293476E37F);
            assert(pack.wind_x_GET() == -2.6433229E38F);
            assert(pack.time_usec_GET() == 2609472524012010296L);
        });
        DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.wind_x_SET(-2.6433229E38F) ;
        p231.var_vert_SET(-9.995999E37F) ;
        p231.vert_accuracy_SET(-2.9310175E38F) ;
        p231.horiz_accuracy_SET(1.430476E37F) ;
        p231.time_usec_SET(2609472524012010296L) ;
        p231.var_horiz_SET(-7.810207E37F) ;
        p231.wind_y_SET(2.9293476E37F) ;
        p231.wind_alt_SET(-1.6510262E38F) ;
        p231.wind_z_SET(2.9657899E38F) ;
        LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.vdop_GET() == 1.2661588E38F);
            assert(pack.hdop_GET() == -1.3182798E38F);
            assert(pack.satellites_visible_GET() == (char)75);
            assert(pack.gps_id_GET() == (char)191);
            assert(pack.alt_GET() == 3.0750755E38F);
            assert(pack.lon_GET() == 411098182);
            assert(pack.time_week_ms_GET() == 3182944201L);
            assert(pack.vert_accuracy_GET() == -1.1480007E38F);
            assert(pack.speed_accuracy_GET() == -3.1131842E38F);
            assert(pack.vd_GET() == 2.3650249E38F);
            assert(pack.vn_GET() == -1.1389516E38F);
            assert(pack.lat_GET() == -1736565676);
            assert(pack.ve_GET() == -2.2608987E38F);
            assert(pack.ignore_flags_GET() == GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP);
            assert(pack.fix_type_GET() == (char)159);
            assert(pack.horiz_accuracy_GET() == -6.512137E36F);
            assert(pack.time_week_GET() == (char)52223);
            assert(pack.time_usec_GET() == 1818120262192707404L);
        });
        DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.horiz_accuracy_SET(-6.512137E36F) ;
        p232.time_week_ms_SET(3182944201L) ;
        p232.lat_SET(-1736565676) ;
        p232.vdop_SET(1.2661588E38F) ;
        p232.vert_accuracy_SET(-1.1480007E38F) ;
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_HDOP) ;
        p232.vn_SET(-1.1389516E38F) ;
        p232.alt_SET(3.0750755E38F) ;
        p232.gps_id_SET((char)191) ;
        p232.hdop_SET(-1.3182798E38F) ;
        p232.time_usec_SET(1818120262192707404L) ;
        p232.time_week_SET((char)52223) ;
        p232.fix_type_SET((char)159) ;
        p232.lon_SET(411098182) ;
        p232.speed_accuracy_SET(-3.1131842E38F) ;
        p232.vd_SET(2.3650249E38F) ;
        p232.satellites_visible_SET((char)75) ;
        p232.ve_SET(-2.2608987E38F) ;
        LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (char)137);
            assert(pack.len_GET() == (char)7);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)142, (char)253, (char)131, (char)205, (char)146, (char)102, (char)126, (char)36, (char)217, (char)98, (char)228, (char)164, (char)57, (char)184, (char)107, (char)28, (char)201, (char)200, (char)208, (char)2, (char)45, (char)228, (char)98, (char)21, (char)224, (char)138, (char)65, (char)18, (char)60, (char)247, (char)170, (char)209, (char)217, (char)13, (char)127, (char)225, (char)10, (char)228, (char)52, (char)9, (char)48, (char)182, (char)25, (char)113, (char)203, (char)188, (char)190, (char)131, (char)27, (char)196, (char)80, (char)51, (char)153, (char)76, (char)156, (char)168, (char)47, (char)62, (char)208, (char)222, (char)193, (char)16, (char)62, (char)198, (char)126, (char)104, (char)145, (char)147, (char)1, (char)71, (char)110, (char)183, (char)197, (char)134, (char)179, (char)177, (char)201, (char)213, (char)82, (char)242, (char)188, (char)137, (char)136, (char)145, (char)204, (char)139, (char)27, (char)190, (char)112, (char)41, (char)30, (char)86, (char)173, (char)62, (char)237, (char)81, (char)121, (char)74, (char)129, (char)187, (char)212, (char)156, (char)199, (char)87, (char)202, (char)73, (char)41, (char)149, (char)218, (char)65, (char)134, (char)30, (char)128, (char)32, (char)159, (char)63, (char)37, (char)90, (char)97, (char)95, (char)232, (char)211, (char)152, (char)142, (char)1, (char)24, (char)8, (char)32, (char)121, (char)186, (char)21, (char)175, (char)96, (char)211, (char)103, (char)151, (char)160, (char)192, (char)211, (char)23, (char)96, (char)111, (char)197, (char)35, (char)241, (char)235, (char)11, (char)174, (char)33, (char)248, (char)165, (char)162, (char)213, (char)25, (char)4, (char)252, (char)147, (char)134, (char)184, (char)70, (char)186, (char)172, (char)206, (char)151, (char)97, (char)23, (char)120, (char)3, (char)77, (char)56, (char)185, (char)95, (char)118, (char)237, (char)75, (char)60, (char)92, (char)34, (char)243, (char)91}));
        });
        DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.data__SET(new char[] {(char)142, (char)253, (char)131, (char)205, (char)146, (char)102, (char)126, (char)36, (char)217, (char)98, (char)228, (char)164, (char)57, (char)184, (char)107, (char)28, (char)201, (char)200, (char)208, (char)2, (char)45, (char)228, (char)98, (char)21, (char)224, (char)138, (char)65, (char)18, (char)60, (char)247, (char)170, (char)209, (char)217, (char)13, (char)127, (char)225, (char)10, (char)228, (char)52, (char)9, (char)48, (char)182, (char)25, (char)113, (char)203, (char)188, (char)190, (char)131, (char)27, (char)196, (char)80, (char)51, (char)153, (char)76, (char)156, (char)168, (char)47, (char)62, (char)208, (char)222, (char)193, (char)16, (char)62, (char)198, (char)126, (char)104, (char)145, (char)147, (char)1, (char)71, (char)110, (char)183, (char)197, (char)134, (char)179, (char)177, (char)201, (char)213, (char)82, (char)242, (char)188, (char)137, (char)136, (char)145, (char)204, (char)139, (char)27, (char)190, (char)112, (char)41, (char)30, (char)86, (char)173, (char)62, (char)237, (char)81, (char)121, (char)74, (char)129, (char)187, (char)212, (char)156, (char)199, (char)87, (char)202, (char)73, (char)41, (char)149, (char)218, (char)65, (char)134, (char)30, (char)128, (char)32, (char)159, (char)63, (char)37, (char)90, (char)97, (char)95, (char)232, (char)211, (char)152, (char)142, (char)1, (char)24, (char)8, (char)32, (char)121, (char)186, (char)21, (char)175, (char)96, (char)211, (char)103, (char)151, (char)160, (char)192, (char)211, (char)23, (char)96, (char)111, (char)197, (char)35, (char)241, (char)235, (char)11, (char)174, (char)33, (char)248, (char)165, (char)162, (char)213, (char)25, (char)4, (char)252, (char)147, (char)134, (char)184, (char)70, (char)186, (char)172, (char)206, (char)151, (char)97, (char)23, (char)120, (char)3, (char)77, (char)56, (char)185, (char)95, (char)118, (char)237, (char)75, (char)60, (char)92, (char)34, (char)243, (char)91}, 0) ;
        p233.len_SET((char)7) ;
        p233.flags_SET((char)137) ;
        LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.heading_sp_GET() == (short) -27672);
            assert(pack.roll_GET() == (short) -31169);
            assert(pack.wp_num_GET() == (char)17);
            assert(pack.airspeed_GET() == (char)131);
            assert(pack.battery_remaining_GET() == (char)195);
            assert(pack.longitude_GET() == -846003135);
            assert(pack.altitude_amsl_GET() == (short)16697);
            assert(pack.temperature_air_GET() == (byte) - 14);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
            assert(pack.custom_mode_GET() == 2553950247L);
            assert(pack.latitude_GET() == 649779370);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED);
            assert(pack.groundspeed_GET() == (char)155);
            assert(pack.airspeed_sp_GET() == (char)25);
            assert(pack.heading_GET() == (char)411);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS);
            assert(pack.altitude_sp_GET() == (short)23264);
            assert(pack.throttle_GET() == (byte)70);
            assert(pack.climb_rate_GET() == (byte) - 44);
            assert(pack.wp_distance_GET() == (char)54789);
            assert(pack.gps_nsat_GET() == (char)94);
            assert(pack.pitch_GET() == (short) -12579);
            assert(pack.temperature_GET() == (byte)40);
            assert(pack.failsafe_GET() == (char)41);
        });
        DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.roll_SET((short) -31169) ;
        p234.heading_sp_SET((short) -27672) ;
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED) ;
        p234.temperature_SET((byte)40) ;
        p234.failsafe_SET((char)41) ;
        p234.airspeed_sp_SET((char)25) ;
        p234.airspeed_SET((char)131) ;
        p234.altitude_sp_SET((short)23264) ;
        p234.longitude_SET(-846003135) ;
        p234.custom_mode_SET(2553950247L) ;
        p234.latitude_SET(649779370) ;
        p234.throttle_SET((byte)70) ;
        p234.wp_distance_SET((char)54789) ;
        p234.battery_remaining_SET((char)195) ;
        p234.wp_num_SET((char)17) ;
        p234.climb_rate_SET((byte) - 44) ;
        p234.groundspeed_SET((char)155) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_DGPS) ;
        p234.heading_SET((char)411) ;
        p234.gps_nsat_SET((char)94) ;
        p234.altitude_amsl_SET((short)16697) ;
        p234.temperature_air_SET((byte) - 14) ;
        p234.pitch_SET((short) -12579) ;
        LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.vibration_z_GET() == 3.6239986E37F);
            assert(pack.clipping_0_GET() == 2259709559L);
            assert(pack.vibration_y_GET() == -1.9579421E38F);
            assert(pack.vibration_x_GET() == 2.1879467E38F);
            assert(pack.time_usec_GET() == 3356665083313621926L);
            assert(pack.clipping_2_GET() == 2534062837L);
            assert(pack.clipping_1_GET() == 2901056308L);
        });
        DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.clipping_1_SET(2901056308L) ;
        p241.vibration_x_SET(2.1879467E38F) ;
        p241.clipping_0_SET(2259709559L) ;
        p241.clipping_2_SET(2534062837L) ;
        p241.time_usec_SET(3356665083313621926L) ;
        p241.vibration_y_SET(-1.9579421E38F) ;
        p241.vibration_z_SET(3.6239986E37F) ;
        LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == -566144476);
            assert(pack.latitude_GET() == 431581215);
            assert(pack.altitude_GET() == 1644109081);
            assert(pack.y_GET() == -4.29241E37F);
            assert(pack.z_GET() == 2.2821921E38F);
            assert(pack.x_GET() == 1.1850329E37F);
            assert(pack.time_usec_TRY(ph) == 7607682967090471537L);
            assert(pack.approach_y_GET() == 1.1861791E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-4.908021E37F, -2.0660733E38F, 2.1436787E38F, 1.5381951E38F}));
            assert(pack.approach_x_GET() == -2.9635657E38F);
            assert(pack.approach_z_GET() == -7.4439354E37F);
        });
        DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.longitude_SET(-566144476) ;
        p242.altitude_SET(1644109081) ;
        p242.x_SET(1.1850329E37F) ;
        p242.latitude_SET(431581215) ;
        p242.approach_x_SET(-2.9635657E38F) ;
        p242.z_SET(2.2821921E38F) ;
        p242.time_usec_SET(7607682967090471537L, PH) ;
        p242.q_SET(new float[] {-4.908021E37F, -2.0660733E38F, 2.1436787E38F, 1.5381951E38F}, 0) ;
        p242.y_SET(-4.29241E37F) ;
        p242.approach_z_SET(-7.4439354E37F) ;
        p242.approach_y_SET(1.1861791E38F) ;
        LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {6.785715E37F, -1.0485465E37F, -8.506952E37F, 1.4904825E38F}));
            assert(pack.z_GET() == -2.5175474E38F);
            assert(pack.approach_x_GET() == -1.0564414E38F);
            assert(pack.latitude_GET() == 432910923);
            assert(pack.x_GET() == 1.4063171E38F);
            assert(pack.approach_y_GET() == -7.5302096E37F);
            assert(pack.longitude_GET() == -394752470);
            assert(pack.target_system_GET() == (char)73);
            assert(pack.altitude_GET() == 1245274155);
            assert(pack.y_GET() == 2.7056887E38F);
            assert(pack.time_usec_TRY(ph) == 8457823576842269541L);
            assert(pack.approach_z_GET() == 2.476285E38F);
        });
        DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.altitude_SET(1245274155) ;
        p243.approach_x_SET(-1.0564414E38F) ;
        p243.longitude_SET(-394752470) ;
        p243.target_system_SET((char)73) ;
        p243.y_SET(2.7056887E38F) ;
        p243.approach_y_SET(-7.5302096E37F) ;
        p243.q_SET(new float[] {6.785715E37F, -1.0485465E37F, -8.506952E37F, 1.4904825E38F}, 0) ;
        p243.time_usec_SET(8457823576842269541L, PH) ;
        p243.latitude_SET(432910923) ;
        p243.approach_z_SET(2.476285E38F) ;
        p243.z_SET(-2.5175474E38F) ;
        p243.x_SET(1.4063171E38F) ;
        LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.interval_us_GET() == -199872568);
            assert(pack.message_id_GET() == (char)34221);
        });
        DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)34221) ;
        p244.interval_us_SET(-199872568) ;
        LoopBackDemoChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_MC);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING);
        });
        DemoDevice.EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_LANDING) ;
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_MC) ;
        LoopBackDemoChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -449489143);
            assert(pack.heading_GET() == (char)38263);
            assert(pack.hor_velocity_GET() == (char)39768);
            assert(pack.altitude_GET() == -2050791923);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_NO_INFO);
            assert(pack.callsign_LEN(ph) == 8);
            assert(pack.callsign_TRY(ph).equals("Zrtjgthh"));
            assert(pack.lat_GET() == 1608328278);
            assert(pack.tslc_GET() == (char)94);
            assert(pack.squawk_GET() == (char)37281);
            assert(pack.flags_GET() == ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH);
            assert(pack.ver_velocity_GET() == (short)19891);
            assert(pack.ICAO_address_GET() == 3798516906L);
        });
        DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.altitude_SET(-2050791923) ;
        p246.ver_velocity_SET((short)19891) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_NO_INFO) ;
        p246.hor_velocity_SET((char)39768) ;
        p246.lat_SET(1608328278) ;
        p246.heading_SET((char)38263) ;
        p246.squawk_SET((char)37281) ;
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_SQUAWK) ;
        p246.ICAO_address_SET(3798516906L) ;
        p246.callsign_SET("Zrtjgthh", PH) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_PRESSURE_QNH) ;
        p246.lon_SET(-449489143) ;
        p246.tslc_SET((char)94) ;
        LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT);
            assert(pack.time_to_minimum_delta_GET() == 3.1381233E37F);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
            assert(pack.altitude_minimum_delta_GET() == -5.695994E37F);
            assert(pack.horizontal_minimum_delta_GET() == 2.4170485E38F);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
            assert(pack.id_GET() == 3296005506L);
        });
        DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
        PH.setPack(p247);
        p247.altitude_minimum_delta_SET(-5.695994E37F) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_REPORT) ;
        p247.time_to_minimum_delta_SET(3.1381233E37F) ;
        p247.horizontal_minimum_delta_SET(2.4170485E38F) ;
        p247.id_SET(3296005506L) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)27, (char)7, (char)187, (char)13, (char)205, (char)219, (char)194, (char)58, (char)160, (char)221, (char)231, (char)234, (char)30, (char)38, (char)78, (char)207, (char)62, (char)242, (char)195, (char)3, (char)74, (char)151, (char)74, (char)248, (char)175, (char)48, (char)58, (char)221, (char)216, (char)122, (char)185, (char)51, (char)214, (char)138, (char)130, (char)153, (char)230, (char)219, (char)220, (char)154, (char)90, (char)15, (char)178, (char)180, (char)255, (char)182, (char)20, (char)210, (char)72, (char)8, (char)8, (char)248, (char)79, (char)124, (char)43, (char)237, (char)87, (char)144, (char)19, (char)2, (char)59, (char)63, (char)144, (char)251, (char)212, (char)78, (char)242, (char)146, (char)147, (char)62, (char)116, (char)10, (char)64, (char)173, (char)238, (char)204, (char)215, (char)148, (char)8, (char)39, (char)57, (char)119, (char)215, (char)149, (char)4, (char)126, (char)223, (char)223, (char)184, (char)90, (char)222, (char)250, (char)215, (char)78, (char)179, (char)91, (char)122, (char)240, (char)188, (char)43, (char)170, (char)144, (char)204, (char)33, (char)131, (char)139, (char)87, (char)164, (char)145, (char)35, (char)105, (char)130, (char)223, (char)91, (char)148, (char)184, (char)14, (char)241, (char)185, (char)98, (char)154, (char)137, (char)102, (char)4, (char)146, (char)66, (char)57, (char)175, (char)161, (char)134, (char)205, (char)109, (char)138, (char)101, (char)28, (char)89, (char)182, (char)104, (char)248, (char)2, (char)208, (char)63, (char)47, (char)190, (char)156, (char)40, (char)101, (char)214, (char)22, (char)242, (char)48, (char)114, (char)133, (char)245, (char)206, (char)204, (char)198, (char)187, (char)10, (char)76, (char)253, (char)225, (char)231, (char)135, (char)130, (char)215, (char)51, (char)30, (char)157, (char)33, (char)54, (char)35, (char)85, (char)215, (char)218, (char)219, (char)202, (char)84, (char)23, (char)102, (char)216, (char)134, (char)100, (char)36, (char)47, (char)105, (char)202, (char)36, (char)23, (char)213, (char)82, (char)83, (char)170, (char)198, (char)31, (char)168, (char)119, (char)240, (char)170, (char)144, (char)9, (char)241, (char)29, (char)251, (char)151, (char)202, (char)34, (char)44, (char)26, (char)21, (char)252, (char)71, (char)210, (char)188, (char)107, (char)26, (char)119, (char)2, (char)161, (char)114, (char)96, (char)59, (char)83, (char)224, (char)115, (char)253, (char)31, (char)123, (char)87, (char)39, (char)73, (char)218, (char)222, (char)226, (char)113, (char)42, (char)155, (char)43, (char)19, (char)199, (char)63, (char)254, (char)152, (char)213, (char)198, (char)11, (char)31, (char)36, (char)5}));
            assert(pack.target_component_GET() == (char)165);
            assert(pack.target_network_GET() == (char)0);
            assert(pack.target_system_GET() == (char)183);
            assert(pack.message_type_GET() == (char)34502);
        });
        DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_component_SET((char)165) ;
        p248.target_system_SET((char)183) ;
        p248.target_network_SET((char)0) ;
        p248.message_type_SET((char)34502) ;
        p248.payload_SET(new char[] {(char)27, (char)7, (char)187, (char)13, (char)205, (char)219, (char)194, (char)58, (char)160, (char)221, (char)231, (char)234, (char)30, (char)38, (char)78, (char)207, (char)62, (char)242, (char)195, (char)3, (char)74, (char)151, (char)74, (char)248, (char)175, (char)48, (char)58, (char)221, (char)216, (char)122, (char)185, (char)51, (char)214, (char)138, (char)130, (char)153, (char)230, (char)219, (char)220, (char)154, (char)90, (char)15, (char)178, (char)180, (char)255, (char)182, (char)20, (char)210, (char)72, (char)8, (char)8, (char)248, (char)79, (char)124, (char)43, (char)237, (char)87, (char)144, (char)19, (char)2, (char)59, (char)63, (char)144, (char)251, (char)212, (char)78, (char)242, (char)146, (char)147, (char)62, (char)116, (char)10, (char)64, (char)173, (char)238, (char)204, (char)215, (char)148, (char)8, (char)39, (char)57, (char)119, (char)215, (char)149, (char)4, (char)126, (char)223, (char)223, (char)184, (char)90, (char)222, (char)250, (char)215, (char)78, (char)179, (char)91, (char)122, (char)240, (char)188, (char)43, (char)170, (char)144, (char)204, (char)33, (char)131, (char)139, (char)87, (char)164, (char)145, (char)35, (char)105, (char)130, (char)223, (char)91, (char)148, (char)184, (char)14, (char)241, (char)185, (char)98, (char)154, (char)137, (char)102, (char)4, (char)146, (char)66, (char)57, (char)175, (char)161, (char)134, (char)205, (char)109, (char)138, (char)101, (char)28, (char)89, (char)182, (char)104, (char)248, (char)2, (char)208, (char)63, (char)47, (char)190, (char)156, (char)40, (char)101, (char)214, (char)22, (char)242, (char)48, (char)114, (char)133, (char)245, (char)206, (char)204, (char)198, (char)187, (char)10, (char)76, (char)253, (char)225, (char)231, (char)135, (char)130, (char)215, (char)51, (char)30, (char)157, (char)33, (char)54, (char)35, (char)85, (char)215, (char)218, (char)219, (char)202, (char)84, (char)23, (char)102, (char)216, (char)134, (char)100, (char)36, (char)47, (char)105, (char)202, (char)36, (char)23, (char)213, (char)82, (char)83, (char)170, (char)198, (char)31, (char)168, (char)119, (char)240, (char)170, (char)144, (char)9, (char)241, (char)29, (char)251, (char)151, (char)202, (char)34, (char)44, (char)26, (char)21, (char)252, (char)71, (char)210, (char)188, (char)107, (char)26, (char)119, (char)2, (char)161, (char)114, (char)96, (char)59, (char)83, (char)224, (char)115, (char)253, (char)31, (char)123, (char)87, (char)39, (char)73, (char)218, (char)222, (char)226, (char)113, (char)42, (char)155, (char)43, (char)19, (char)199, (char)63, (char)254, (char)152, (char)213, (char)198, (char)11, (char)31, (char)36, (char)5}, 0) ;
        LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == (char)180);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte) - 21, (byte)65, (byte) - 42, (byte)1, (byte)91, (byte) - 23, (byte) - 84, (byte)8, (byte)47, (byte)122, (byte) - 90, (byte)106, (byte) - 72, (byte)27, (byte) - 44, (byte)22, (byte) - 50, (byte) - 45, (byte) - 121, (byte)14, (byte) - 11, (byte) - 33, (byte)108, (byte) - 98, (byte)33, (byte) - 122, (byte)80, (byte)60, (byte)88, (byte) - 48, (byte)108, (byte)68}));
            assert(pack.ver_GET() == (char)47);
            assert(pack.address_GET() == (char)34906);
        });
        DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)34906) ;
        p249.ver_SET((char)47) ;
        p249.value_SET(new byte[] {(byte) - 21, (byte)65, (byte) - 42, (byte)1, (byte)91, (byte) - 23, (byte) - 84, (byte)8, (byte)47, (byte)122, (byte) - 90, (byte)106, (byte) - 72, (byte)27, (byte) - 44, (byte)22, (byte) - 50, (byte) - 45, (byte) - 121, (byte)14, (byte) - 11, (byte) - 33, (byte)108, (byte) - 98, (byte)33, (byte) - 122, (byte)80, (byte)60, (byte)88, (byte) - 48, (byte)108, (byte)68}, 0) ;
        p249.type_SET((char)180) ;
        LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -9.61042E37F);
            assert(pack.y_GET() == 3.238271E38F);
            assert(pack.name_LEN(ph) == 4);
            assert(pack.name_TRY(ph).equals("burl"));
            assert(pack.x_GET() == -2.0924575E38F);
            assert(pack.time_usec_GET() == 2119473155890719179L);
        });
        DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.y_SET(3.238271E38F) ;
        p250.z_SET(-9.61042E37F) ;
        p250.time_usec_SET(2119473155890719179L) ;
        p250.x_SET(-2.0924575E38F) ;
        p250.name_SET("burl", PH) ;
        LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 8);
            assert(pack.name_TRY(ph).equals("dfyitujy"));
            assert(pack.value_GET() == -5.1230933E37F);
            assert(pack.time_boot_ms_GET() == 263196077L);
        });
        DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.name_SET("dfyitujy", PH) ;
        p251.value_SET(-5.1230933E37F) ;
        p251.time_boot_ms_SET(263196077L) ;
        LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == 2124311857);
            assert(pack.time_boot_ms_GET() == 2374716771L);
            assert(pack.name_LEN(ph) == 5);
            assert(pack.name_TRY(ph).equals("ymqyp"));
        });
        DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.value_SET(2124311857) ;
        p252.time_boot_ms_SET(2374716771L) ;
        p252.name_SET("ymqyp", PH) ;
        LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 30);
            assert(pack.text_TRY(ph).equals("ktykIhkYweeWovddHlzgsurjvohfsb"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_CRITICAL);
        });
        DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_CRITICAL) ;
        p253.text_SET("ktykIhkYweeWovddHlzgsurjvohfsb", PH) ;
        LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == -2.886786E38F);
            assert(pack.ind_GET() == (char)56);
            assert(pack.time_boot_ms_GET() == 2930581069L);
        });
        DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(2930581069L) ;
        p254.value_SET(-2.886786E38F) ;
        p254.ind_SET((char)56) ;
        LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)170);
            assert(pack.initial_timestamp_GET() == 6421664445507924866L);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)18, (char)238, (char)211, (char)49, (char)243, (char)135, (char)235, (char)207, (char)75, (char)53, (char)231, (char)156, (char)161, (char)49, (char)222, (char)61, (char)185, (char)137, (char)63, (char)119, (char)125, (char)128, (char)137, (char)27, (char)254, (char)100, (char)161, (char)152, (char)119, (char)170, (char)102, (char)48}));
            assert(pack.target_component_GET() == (char)250);
        });
        DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.target_component_SET((char)250) ;
        p256.target_system_SET((char)170) ;
        p256.initial_timestamp_SET(6421664445507924866L) ;
        p256.secret_key_SET(new char[] {(char)18, (char)238, (char)211, (char)49, (char)243, (char)135, (char)235, (char)207, (char)75, (char)53, (char)231, (char)156, (char)161, (char)49, (char)222, (char)61, (char)185, (char)137, (char)63, (char)119, (char)125, (char)128, (char)137, (char)27, (char)254, (char)100, (char)161, (char)152, (char)119, (char)170, (char)102, (char)48}, 0) ;
        LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.last_change_ms_GET() == 2225109678L);
            assert(pack.time_boot_ms_GET() == 1925032775L);
            assert(pack.state_GET() == (char)65);
        });
        DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(1925032775L) ;
        p257.state_SET((char)65) ;
        p257.last_change_ms_SET(2225109678L) ;
        LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)18);
            assert(pack.target_component_GET() == (char)103);
            assert(pack.tune_LEN(ph) == 22);
            assert(pack.tune_TRY(ph).equals("cpzknRkyvkeecaluhjbswj"));
        });
        DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_system_SET((char)18) ;
        p258.tune_SET("cpzknRkyvkeecaluhjbswj", PH) ;
        p258.target_component_SET((char)103) ;
        LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 2096496834L);
            assert(pack.lens_id_GET() == (char)85);
            assert(pack.resolution_v_GET() == (char)40193);
            assert(pack.sensor_size_v_GET() == -1.0578223E38F);
            assert(pack.firmware_version_GET() == 187758981L);
            assert(pack.resolution_h_GET() == (char)29050);
            assert(pack.flags_GET() == CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)93, (char)217, (char)176, (char)247, (char)234, (char)5, (char)34, (char)78, (char)176, (char)100, (char)46, (char)195, (char)33, (char)51, (char)13, (char)132, (char)119, (char)134, (char)31, (char)231, (char)88, (char)226, (char)90, (char)246, (char)34, (char)18, (char)251, (char)42, (char)154, (char)150, (char)64, (char)193}));
            assert(pack.cam_definition_version_GET() == (char)61959);
            assert(pack.cam_definition_uri_LEN(ph) == 113);
            assert(pack.cam_definition_uri_TRY(ph).equals("QfulowocxrcxrcmpnubkzrGotxxadpygpNwjmwbvykzptdxxrqpsihuzThyzhdavtshitrgUnvOmhggmeqzlbwvgxfeofkbbhwzretDrjaowlLlhf"));
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)98, (char)65, (char)182, (char)123, (char)28, (char)224, (char)121, (char)111, (char)204, (char)175, (char)116, (char)44, (char)255, (char)51, (char)225, (char)58, (char)33, (char)86, (char)180, (char)102, (char)138, (char)41, (char)84, (char)210, (char)194, (char)39, (char)74, (char)16, (char)53, (char)76, (char)160, (char)21}));
            assert(pack.focal_length_GET() == -4.7932587E37F);
            assert(pack.sensor_size_h_GET() == 6.3717327E37F);
        });
        DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.cam_definition_version_SET((char)61959) ;
        p259.model_name_SET(new char[] {(char)98, (char)65, (char)182, (char)123, (char)28, (char)224, (char)121, (char)111, (char)204, (char)175, (char)116, (char)44, (char)255, (char)51, (char)225, (char)58, (char)33, (char)86, (char)180, (char)102, (char)138, (char)41, (char)84, (char)210, (char)194, (char)39, (char)74, (char)16, (char)53, (char)76, (char)160, (char)21}, 0) ;
        p259.cam_definition_uri_SET("QfulowocxrcxrcmpnubkzrGotxxadpygpNwjmwbvykzptdxxrqpsihuzThyzhdavtshitrgUnvOmhggmeqzlbwvgxfeofkbbhwzretDrjaowlLlhf", PH) ;
        p259.lens_id_SET((char)85) ;
        p259.time_boot_ms_SET(2096496834L) ;
        p259.focal_length_SET(-4.7932587E37F) ;
        p259.sensor_size_h_SET(6.3717327E37F) ;
        p259.vendor_name_SET(new char[] {(char)93, (char)217, (char)176, (char)247, (char)234, (char)5, (char)34, (char)78, (char)176, (char)100, (char)46, (char)195, (char)33, (char)51, (char)13, (char)132, (char)119, (char)134, (char)31, (char)231, (char)88, (char)226, (char)90, (char)246, (char)34, (char)18, (char)251, (char)42, (char)154, (char)150, (char)64, (char)193}, 0) ;
        p259.resolution_v_SET((char)40193) ;
        p259.resolution_h_SET((char)29050) ;
        p259.firmware_version_SET(187758981L) ;
        p259.sensor_size_v_SET(-1.0578223E38F) ;
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE) ;
        LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_IMAGE);
            assert(pack.time_boot_ms_GET() == 2824532309L);
        });
        DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(2824532309L) ;
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_IMAGE) ;
        LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.total_capacity_GET() == -3.3718397E38F);
            assert(pack.status_GET() == (char)240);
            assert(pack.read_speed_GET() == 1.969562E38F);
            assert(pack.available_capacity_GET() == 3.508187E37F);
            assert(pack.time_boot_ms_GET() == 2207628171L);
            assert(pack.storage_count_GET() == (char)44);
            assert(pack.used_capacity_GET() == 4.6398436E37F);
            assert(pack.storage_id_GET() == (char)201);
            assert(pack.write_speed_GET() == 3.0451024E37F);
        });
        DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.status_SET((char)240) ;
        p261.used_capacity_SET(4.6398436E37F) ;
        p261.storage_id_SET((char)201) ;
        p261.time_boot_ms_SET(2207628171L) ;
        p261.total_capacity_SET(-3.3718397E38F) ;
        p261.available_capacity_SET(3.508187E37F) ;
        p261.read_speed_SET(1.969562E38F) ;
        p261.write_speed_SET(3.0451024E37F) ;
        p261.storage_count_SET((char)44) ;
        LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 120928738L);
            assert(pack.video_status_GET() == (char)216);
            assert(pack.image_status_GET() == (char)120);
            assert(pack.available_capacity_GET() == 1.6840187E38F);
            assert(pack.image_interval_GET() == -1.7926178E38F);
            assert(pack.recording_time_ms_GET() == 2330714781L);
        });
        DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.image_interval_SET(-1.7926178E38F) ;
        p262.image_status_SET((char)120) ;
        p262.available_capacity_SET(1.6840187E38F) ;
        p262.recording_time_ms_SET(2330714781L) ;
        p262.video_status_SET((char)216) ;
        p262.time_boot_ms_SET(120928738L) ;
        LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1548934379L);
            assert(pack.file_url_LEN(ph) == 192);
            assert(pack.file_url_TRY(ph).equals("tewrskdxcxapxuvdhmHhubbltrrbslgUtQxrthFRsxzalbhlbBbtytsmyApdkqthjgfwzjzWyiflrsozsdoKcwnwPCNjygiovhtxpEqlipidbgLzwkasoddmffigmydwrqDhwyycwedhthfkvhpLavbVeyqmimtmohenquyHudbenwxwMdzamhgnozniadOt"));
            assert(pack.alt_GET() == 25801467);
            assert(pack.image_index_GET() == 1249299038);
            assert(pack.time_utc_GET() == 3694194144387787241L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.529512E38F, 2.8638864E38F, -2.057082E38F, 8.7841246E36F}));
            assert(pack.relative_alt_GET() == -272343898);
            assert(pack.lon_GET() == -1654913912);
            assert(pack.camera_id_GET() == (char)240);
            assert(pack.lat_GET() == 1956959198);
            assert(pack.capture_result_GET() == (byte) - 30);
        });
        DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.lat_SET(1956959198) ;
        p263.time_boot_ms_SET(1548934379L) ;
        p263.lon_SET(-1654913912) ;
        p263.alt_SET(25801467) ;
        p263.q_SET(new float[] {-1.529512E38F, 2.8638864E38F, -2.057082E38F, 8.7841246E36F}, 0) ;
        p263.capture_result_SET((byte) - 30) ;
        p263.time_utc_SET(3694194144387787241L) ;
        p263.relative_alt_SET(-272343898) ;
        p263.camera_id_SET((char)240) ;
        p263.file_url_SET("tewrskdxcxapxuvdhmHhubbltrrbslgUtQxrthFRsxzalbhlbBbtytsmyApdkqthjgfwzjzWyiflrsozsdoKcwnwPCNjygiovhtxpEqlipidbgLzwkasoddmffigmydwrqDhwyycwedhthfkvhpLavbVeyqmimtmohenquyHudbenwxwMdzamhgnozniadOt", PH) ;
        p263.image_index_SET(1249299038) ;
        LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 4170630829L);
            assert(pack.arming_time_utc_GET() == 1868264157183561531L);
            assert(pack.flight_uuid_GET() == 1722791272429726501L);
            assert(pack.takeoff_time_utc_GET() == 7665537753292701198L);
        });
        DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.arming_time_utc_SET(1868264157183561531L) ;
        p264.time_boot_ms_SET(4170630829L) ;
        p264.takeoff_time_utc_SET(7665537753292701198L) ;
        p264.flight_uuid_SET(1722791272429726501L) ;
        LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 1.5370516E38F);
            assert(pack.roll_GET() == -2.1679413E38F);
            assert(pack.yaw_GET() == 5.108595E37F);
            assert(pack.time_boot_ms_GET() == 2058782326L);
        });
        DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.roll_SET(-2.1679413E38F) ;
        p265.time_boot_ms_SET(2058782326L) ;
        p265.pitch_SET(1.5370516E38F) ;
        p265.yaw_SET(5.108595E37F) ;
        LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)224, (char)216, (char)197, (char)33, (char)254, (char)41, (char)236, (char)35, (char)244, (char)221, (char)88, (char)141, (char)126, (char)230, (char)81, (char)93, (char)81, (char)171, (char)214, (char)137, (char)146, (char)34, (char)142, (char)13, (char)122, (char)14, (char)104, (char)3, (char)64, (char)175, (char)3, (char)43, (char)51, (char)107, (char)210, (char)46, (char)164, (char)139, (char)166, (char)32, (char)88, (char)182, (char)213, (char)124, (char)153, (char)246, (char)97, (char)13, (char)111, (char)13, (char)46, (char)40, (char)251, (char)110, (char)164, (char)46, (char)36, (char)250, (char)138, (char)21, (char)64, (char)25, (char)223, (char)73, (char)120, (char)86, (char)186, (char)53, (char)156, (char)91, (char)63, (char)166, (char)95, (char)70, (char)230, (char)240, (char)87, (char)32, (char)155, (char)169, (char)120, (char)197, (char)181, (char)231, (char)173, (char)2, (char)33, (char)173, (char)174, (char)112, (char)33, (char)67, (char)180, (char)210, (char)138, (char)179, (char)30, (char)198, (char)123, (char)8, (char)92, (char)245, (char)82, (char)65, (char)25, (char)85, (char)101, (char)144, (char)221, (char)225, (char)140, (char)219, (char)169, (char)125, (char)37, (char)227, (char)78, (char)114, (char)179, (char)135, (char)10, (char)105, (char)191, (char)36, (char)76, (char)65, (char)240, (char)65, (char)186, (char)63, (char)79, (char)209, (char)2, (char)151, (char)56, (char)217, (char)93, (char)145, (char)130, (char)248, (char)245, (char)131, (char)29, (char)104, (char)121, (char)179, (char)143, (char)30, (char)242, (char)185, (char)61, (char)7, (char)198, (char)120, (char)200, (char)225, (char)245, (char)65, (char)102, (char)32, (char)177, (char)166, (char)91, (char)71, (char)117, (char)219, (char)111, (char)46, (char)135, (char)165, (char)179, (char)220, (char)161, (char)62, (char)51, (char)139, (char)104, (char)232, (char)138, (char)244, (char)151, (char)206, (char)104, (char)247, (char)110, (char)10, (char)220, (char)95, (char)118, (char)237, (char)162, (char)34, (char)4, (char)247, (char)230, (char)70, (char)49, (char)117, (char)10, (char)220, (char)113, (char)165, (char)128, (char)77, (char)116, (char)141, (char)7, (char)10, (char)120, (char)46, (char)74, (char)17, (char)246, (char)101, (char)9, (char)189, (char)249, (char)30, (char)99, (char)132, (char)194, (char)76, (char)207, (char)189, (char)4, (char)97, (char)65, (char)226, (char)65, (char)80, (char)197, (char)14, (char)165, (char)10, (char)165, (char)159, (char)248, (char)109, (char)199, (char)143, (char)177, (char)106, (char)235, (char)240, (char)212, (char)40, (char)176, (char)161, (char)75}));
            assert(pack.sequence_GET() == (char)19371);
            assert(pack.length_GET() == (char)200);
            assert(pack.target_component_GET() == (char)217);
            assert(pack.target_system_GET() == (char)58);
            assert(pack.first_message_offset_GET() == (char)100);
        });
        DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.sequence_SET((char)19371) ;
        p266.first_message_offset_SET((char)100) ;
        p266.length_SET((char)200) ;
        p266.target_system_SET((char)58) ;
        p266.target_component_SET((char)217) ;
        p266.data__SET(new char[] {(char)224, (char)216, (char)197, (char)33, (char)254, (char)41, (char)236, (char)35, (char)244, (char)221, (char)88, (char)141, (char)126, (char)230, (char)81, (char)93, (char)81, (char)171, (char)214, (char)137, (char)146, (char)34, (char)142, (char)13, (char)122, (char)14, (char)104, (char)3, (char)64, (char)175, (char)3, (char)43, (char)51, (char)107, (char)210, (char)46, (char)164, (char)139, (char)166, (char)32, (char)88, (char)182, (char)213, (char)124, (char)153, (char)246, (char)97, (char)13, (char)111, (char)13, (char)46, (char)40, (char)251, (char)110, (char)164, (char)46, (char)36, (char)250, (char)138, (char)21, (char)64, (char)25, (char)223, (char)73, (char)120, (char)86, (char)186, (char)53, (char)156, (char)91, (char)63, (char)166, (char)95, (char)70, (char)230, (char)240, (char)87, (char)32, (char)155, (char)169, (char)120, (char)197, (char)181, (char)231, (char)173, (char)2, (char)33, (char)173, (char)174, (char)112, (char)33, (char)67, (char)180, (char)210, (char)138, (char)179, (char)30, (char)198, (char)123, (char)8, (char)92, (char)245, (char)82, (char)65, (char)25, (char)85, (char)101, (char)144, (char)221, (char)225, (char)140, (char)219, (char)169, (char)125, (char)37, (char)227, (char)78, (char)114, (char)179, (char)135, (char)10, (char)105, (char)191, (char)36, (char)76, (char)65, (char)240, (char)65, (char)186, (char)63, (char)79, (char)209, (char)2, (char)151, (char)56, (char)217, (char)93, (char)145, (char)130, (char)248, (char)245, (char)131, (char)29, (char)104, (char)121, (char)179, (char)143, (char)30, (char)242, (char)185, (char)61, (char)7, (char)198, (char)120, (char)200, (char)225, (char)245, (char)65, (char)102, (char)32, (char)177, (char)166, (char)91, (char)71, (char)117, (char)219, (char)111, (char)46, (char)135, (char)165, (char)179, (char)220, (char)161, (char)62, (char)51, (char)139, (char)104, (char)232, (char)138, (char)244, (char)151, (char)206, (char)104, (char)247, (char)110, (char)10, (char)220, (char)95, (char)118, (char)237, (char)162, (char)34, (char)4, (char)247, (char)230, (char)70, (char)49, (char)117, (char)10, (char)220, (char)113, (char)165, (char)128, (char)77, (char)116, (char)141, (char)7, (char)10, (char)120, (char)46, (char)74, (char)17, (char)246, (char)101, (char)9, (char)189, (char)249, (char)30, (char)99, (char)132, (char)194, (char)76, (char)207, (char)189, (char)4, (char)97, (char)65, (char)226, (char)65, (char)80, (char)197, (char)14, (char)165, (char)10, (char)165, (char)159, (char)248, (char)109, (char)199, (char)143, (char)177, (char)106, (char)235, (char)240, (char)212, (char)40, (char)176, (char)161, (char)75}, 0) ;
        LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)14, (char)42, (char)224, (char)11, (char)210, (char)59, (char)9, (char)198, (char)83, (char)101, (char)114, (char)140, (char)161, (char)234, (char)220, (char)170, (char)201, (char)26, (char)254, (char)224, (char)31, (char)159, (char)145, (char)181, (char)127, (char)184, (char)193, (char)89, (char)195, (char)10, (char)37, (char)202, (char)56, (char)58, (char)143, (char)223, (char)155, (char)107, (char)125, (char)182, (char)179, (char)187, (char)28, (char)36, (char)131, (char)174, (char)238, (char)118, (char)91, (char)227, (char)170, (char)155, (char)149, (char)51, (char)194, (char)11, (char)251, (char)113, (char)190, (char)213, (char)165, (char)190, (char)1, (char)253, (char)101, (char)107, (char)171, (char)234, (char)96, (char)219, (char)43, (char)51, (char)197, (char)16, (char)96, (char)165, (char)89, (char)252, (char)80, (char)34, (char)57, (char)220, (char)5, (char)93, (char)84, (char)71, (char)201, (char)239, (char)197, (char)116, (char)80, (char)251, (char)248, (char)179, (char)141, (char)140, (char)42, (char)235, (char)162, (char)188, (char)217, (char)176, (char)58, (char)227, (char)36, (char)111, (char)153, (char)162, (char)155, (char)102, (char)56, (char)18, (char)8, (char)122, (char)169, (char)180, (char)89, (char)137, (char)212, (char)76, (char)235, (char)139, (char)164, (char)102, (char)156, (char)80, (char)76, (char)149, (char)210, (char)236, (char)71, (char)120, (char)190, (char)18, (char)74, (char)48, (char)243, (char)141, (char)239, (char)62, (char)114, (char)253, (char)239, (char)43, (char)154, (char)110, (char)52, (char)171, (char)254, (char)56, (char)170, (char)109, (char)177, (char)99, (char)145, (char)235, (char)5, (char)10, (char)151, (char)12, (char)106, (char)50, (char)190, (char)61, (char)107, (char)147, (char)179, (char)52, (char)215, (char)113, (char)92, (char)85, (char)40, (char)253, (char)33, (char)139, (char)7, (char)125, (char)131, (char)214, (char)56, (char)158, (char)205, (char)54, (char)196, (char)187, (char)72, (char)117, (char)82, (char)78, (char)206, (char)62, (char)77, (char)27, (char)94, (char)129, (char)61, (char)24, (char)169, (char)80, (char)170, (char)166, (char)221, (char)97, (char)146, (char)172, (char)119, (char)160, (char)223, (char)60, (char)71, (char)17, (char)87, (char)236, (char)233, (char)41, (char)58, (char)109, (char)157, (char)142, (char)29, (char)234, (char)1, (char)193, (char)238, (char)121, (char)156, (char)95, (char)189, (char)26, (char)108, (char)27, (char)111, (char)7, (char)166, (char)177, (char)87, (char)158, (char)103, (char)84, (char)34, (char)133, (char)182, (char)203, (char)73, (char)194, (char)225, (char)83, (char)43}));
            assert(pack.target_component_GET() == (char)10);
            assert(pack.sequence_GET() == (char)72);
            assert(pack.length_GET() == (char)75);
            assert(pack.target_system_GET() == (char)204);
            assert(pack.first_message_offset_GET() == (char)249);
        });
        DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.target_component_SET((char)10) ;
        p267.sequence_SET((char)72) ;
        p267.length_SET((char)75) ;
        p267.data__SET(new char[] {(char)14, (char)42, (char)224, (char)11, (char)210, (char)59, (char)9, (char)198, (char)83, (char)101, (char)114, (char)140, (char)161, (char)234, (char)220, (char)170, (char)201, (char)26, (char)254, (char)224, (char)31, (char)159, (char)145, (char)181, (char)127, (char)184, (char)193, (char)89, (char)195, (char)10, (char)37, (char)202, (char)56, (char)58, (char)143, (char)223, (char)155, (char)107, (char)125, (char)182, (char)179, (char)187, (char)28, (char)36, (char)131, (char)174, (char)238, (char)118, (char)91, (char)227, (char)170, (char)155, (char)149, (char)51, (char)194, (char)11, (char)251, (char)113, (char)190, (char)213, (char)165, (char)190, (char)1, (char)253, (char)101, (char)107, (char)171, (char)234, (char)96, (char)219, (char)43, (char)51, (char)197, (char)16, (char)96, (char)165, (char)89, (char)252, (char)80, (char)34, (char)57, (char)220, (char)5, (char)93, (char)84, (char)71, (char)201, (char)239, (char)197, (char)116, (char)80, (char)251, (char)248, (char)179, (char)141, (char)140, (char)42, (char)235, (char)162, (char)188, (char)217, (char)176, (char)58, (char)227, (char)36, (char)111, (char)153, (char)162, (char)155, (char)102, (char)56, (char)18, (char)8, (char)122, (char)169, (char)180, (char)89, (char)137, (char)212, (char)76, (char)235, (char)139, (char)164, (char)102, (char)156, (char)80, (char)76, (char)149, (char)210, (char)236, (char)71, (char)120, (char)190, (char)18, (char)74, (char)48, (char)243, (char)141, (char)239, (char)62, (char)114, (char)253, (char)239, (char)43, (char)154, (char)110, (char)52, (char)171, (char)254, (char)56, (char)170, (char)109, (char)177, (char)99, (char)145, (char)235, (char)5, (char)10, (char)151, (char)12, (char)106, (char)50, (char)190, (char)61, (char)107, (char)147, (char)179, (char)52, (char)215, (char)113, (char)92, (char)85, (char)40, (char)253, (char)33, (char)139, (char)7, (char)125, (char)131, (char)214, (char)56, (char)158, (char)205, (char)54, (char)196, (char)187, (char)72, (char)117, (char)82, (char)78, (char)206, (char)62, (char)77, (char)27, (char)94, (char)129, (char)61, (char)24, (char)169, (char)80, (char)170, (char)166, (char)221, (char)97, (char)146, (char)172, (char)119, (char)160, (char)223, (char)60, (char)71, (char)17, (char)87, (char)236, (char)233, (char)41, (char)58, (char)109, (char)157, (char)142, (char)29, (char)234, (char)1, (char)193, (char)238, (char)121, (char)156, (char)95, (char)189, (char)26, (char)108, (char)27, (char)111, (char)7, (char)166, (char)177, (char)87, (char)158, (char)103, (char)84, (char)34, (char)133, (char)182, (char)203, (char)73, (char)194, (char)225, (char)83, (char)43}, 0) ;
        p267.target_system_SET((char)204) ;
        p267.first_message_offset_SET((char)249) ;
        LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)221);
            assert(pack.sequence_GET() == (char)30992);
            assert(pack.target_system_GET() == (char)14);
        });
        DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)14) ;
        p268.target_component_SET((char)221) ;
        p268.sequence_SET((char)30992) ;
        LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.status_GET() == (char)139);
            assert(pack.resolution_h_GET() == (char)27361);
            assert(pack.rotation_GET() == (char)59937);
            assert(pack.framerate_GET() == -2.6536521E38F);
            assert(pack.camera_id_GET() == (char)115);
            assert(pack.bitrate_GET() == 2601977035L);
            assert(pack.uri_LEN(ph) == 174);
            assert(pack.uri_TRY(ph).equals("gdcmyneEaqaznrmlmlryyxzyoVaLecotlcimbnbhMcxwjkwbbgolrjmsyunLionEjsxhYohxypoziyquhdtjrvjhxkcnLocwvduizzmwNuoYchwkYAecGefqturnkhpynpwauisrlmdXrpqJdBzlqqaqayhmpnigguORuyubaqixrm"));
            assert(pack.resolution_v_GET() == (char)54429);
        });
        DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.uri_SET("gdcmyneEaqaznrmlmlryyxzyoVaLecotlcimbnbhMcxwjkwbbgolrjmsyunLionEjsxhYohxypoziyquhdtjrvjhxkcnLocwvduizzmwNuoYchwkYAecGefqturnkhpynpwauisrlmdXrpqJdBzlqqaqayhmpnigguORuyubaqixrm", PH) ;
        p269.framerate_SET(-2.6536521E38F) ;
        p269.rotation_SET((char)59937) ;
        p269.resolution_h_SET((char)27361) ;
        p269.status_SET((char)139) ;
        p269.bitrate_SET(2601977035L) ;
        p269.resolution_v_SET((char)54429) ;
        p269.camera_id_SET((char)115) ;
        LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.resolution_h_GET() == (char)16166);
            assert(pack.resolution_v_GET() == (char)53190);
            assert(pack.target_component_GET() == (char)99);
            assert(pack.rotation_GET() == (char)46040);
            assert(pack.uri_LEN(ph) == 52);
            assert(pack.uri_TRY(ph).equals("efaedtybkwgufkdlOpivekrawqkxuiceCicQkklrxdccfvdqlsqe"));
            assert(pack.bitrate_GET() == 410357091L);
            assert(pack.camera_id_GET() == (char)183);
            assert(pack.target_system_GET() == (char)50);
            assert(pack.framerate_GET() == -3.1694196E38F);
        });
        DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.bitrate_SET(410357091L) ;
        p270.camera_id_SET((char)183) ;
        p270.rotation_SET((char)46040) ;
        p270.uri_SET("efaedtybkwgufkdlOpivekrawqkxuiceCicQkklrxdccfvdqlsqe", PH) ;
        p270.resolution_v_SET((char)53190) ;
        p270.framerate_SET(-3.1694196E38F) ;
        p270.target_component_SET((char)99) ;
        p270.resolution_h_SET((char)16166) ;
        p270.target_system_SET((char)50) ;
        LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.password_LEN(ph) == 12);
            assert(pack.password_TRY(ph).equals("icnBogpVezhl"));
            assert(pack.ssid_LEN(ph) == 32);
            assert(pack.ssid_TRY(ph).equals("okkewrvtfjSpbfIZfmBpglcxjqtcLwmv"));
        });
        DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("icnBogpVezhl", PH) ;
        p299.ssid_SET("okkewrvtfjSpbfIZfmBpglcxjqtcLwmv", PH) ;
        LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.version_GET() == (char)34071);
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)219, (char)141, (char)180, (char)119, (char)234, (char)120, (char)153, (char)238}));
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)109, (char)220, (char)219, (char)92, (char)19, (char)137, (char)28, (char)189}));
            assert(pack.max_version_GET() == (char)60190);
            assert(pack.min_version_GET() == (char)29132);
        });
        DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.max_version_SET((char)60190) ;
        p300.version_SET((char)34071) ;
        p300.library_version_hash_SET(new char[] {(char)219, (char)141, (char)180, (char)119, (char)234, (char)120, (char)153, (char)238}, 0) ;
        p300.spec_version_hash_SET(new char[] {(char)109, (char)220, (char)219, (char)92, (char)19, (char)137, (char)28, (char)189}, 0) ;
        p300.min_version_SET((char)29132) ;
        LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
            assert(pack.sub_mode_GET() == (char)77);
            assert(pack.uptime_sec_GET() == 1714501877L);
            assert(pack.time_usec_GET() == 7168529335837649705L);
            assert(pack.vendor_specific_status_code_GET() == (char)9871);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE);
        });
        DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OFFLINE) ;
        p310.uptime_sec_SET(1714501877L) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING) ;
        p310.vendor_specific_status_code_SET((char)9871) ;
        p310.time_usec_SET(7168529335837649705L) ;
        p310.sub_mode_SET((char)77) ;
        LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.uptime_sec_GET() == 3400939653L);
            assert(pack.sw_version_major_GET() == (char)229);
            assert(pack.sw_vcs_commit_GET() == 2799458155L);
            assert(pack.hw_version_major_GET() == (char)118);
            assert(pack.time_usec_GET() == 7619222160494438354L);
            assert(pack.hw_version_minor_GET() == (char)7);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)94, (char)57, (char)169, (char)161, (char)136, (char)231, (char)208, (char)137, (char)251, (char)50, (char)176, (char)38, (char)38, (char)65, (char)121, (char)133}));
            assert(pack.sw_version_minor_GET() == (char)109);
            assert(pack.name_LEN(ph) == 52);
            assert(pack.name_TRY(ph).equals("lsfStcxeiyqhyzqipsuaHiUuCrudLotxlpimkcqvqczpzguwrmzc"));
        });
        DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.hw_version_major_SET((char)118) ;
        p311.time_usec_SET(7619222160494438354L) ;
        p311.uptime_sec_SET(3400939653L) ;
        p311.sw_vcs_commit_SET(2799458155L) ;
        p311.sw_version_minor_SET((char)109) ;
        p311.hw_unique_id_SET(new char[] {(char)94, (char)57, (char)169, (char)161, (char)136, (char)231, (char)208, (char)137, (char)251, (char)50, (char)176, (char)38, (char)38, (char)65, (char)121, (char)133}, 0) ;
        p311.name_SET("lsfStcxeiyqhyzqipsuaHiUuCrudLotxlpimkcqvqczpzguwrmzc", PH) ;
        p311.sw_version_major_SET((char)229) ;
        p311.hw_version_minor_SET((char)7) ;
        LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("r"));
            assert(pack.target_component_GET() == (char)4);
            assert(pack.param_index_GET() == (short)29765);
            assert(pack.target_system_GET() == (char)126);
        });
        DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.param_index_SET((short)29765) ;
        p320.param_id_SET("r", PH) ;
        p320.target_system_SET((char)126) ;
        p320.target_component_SET((char)4) ;
        LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)37);
            assert(pack.target_component_GET() == (char)99);
        });
        DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)37) ;
        p321.target_component_SET((char)99) ;
        LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_count_GET() == (char)42886);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64);
            assert(pack.param_index_GET() == (char)2563);
            assert(pack.param_value_LEN(ph) == 118);
            assert(pack.param_value_TRY(ph).equals("hktkxbhtmzqzejlpzWvNxsuigsbYbFkOKxmitqszkydowuetjuilmvvogbkCpmmzfrecuulhpdryitrpHtnknMtctnwnxvaaviByfwXmfjtlcwjqdyvwtq"));
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("gxsa"));
        });
        DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_index_SET((char)2563) ;
        p322.param_value_SET("hktkxbhtmzqzejlpzWvNxsuigsbYbFkOKxmitqszkydowuetjuilmvvogbkCpmmzfrecuulhpdryitrpHtnknMtctnwnxvaaviByfwXmfjtlcwjqdyvwtq", PH) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT64) ;
        p322.param_count_SET((char)42886) ;
        p322.param_id_SET("gxsa", PH) ;
        LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("PZ"));
            assert(pack.target_component_GET() == (char)10);
            assert(pack.target_system_GET() == (char)245);
            assert(pack.param_value_LEN(ph) == 33);
            assert(pack.param_value_TRY(ph).equals("oTarcwoQlyzmhhuiVlbiolexbcsdoirlh"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8);
        });
        DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_component_SET((char)10) ;
        p323.param_id_SET("PZ", PH) ;
        p323.param_value_SET("oTarcwoQlyzmhhuiVlbiolexbcsdoirlh", PH) ;
        p323.target_system_SET((char)245) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT8) ;
        LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 6);
            assert(pack.param_id_TRY(ph).equals("rybokg"));
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_IN_PROGRESS);
            assert(pack.param_value_LEN(ph) == 64);
            assert(pack.param_value_TRY(ph).equals("hOlwobuuzbpxbrfjhmEqpwczwkRRoxihraitvHatttvQrduqmeifcgzFcnmihnnU"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
        });
        DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16) ;
        p324.param_value_SET("hOlwobuuzbpxbrfjhmEqpwczwkRRoxihraitvHatttvQrduqmeifcgzFcnmihnnU", PH) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_IN_PROGRESS) ;
        p324.param_id_SET("rybokg", PH) ;
        LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.increment_GET() == (char)35);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)5006, (char)25217, (char)52582, (char)4490, (char)43722, (char)54247, (char)23665, (char)56705, (char)37386, (char)31835, (char)7853, (char)63037, (char)23114, (char)20965, (char)59925, (char)29665, (char)29403, (char)48486, (char)33914, (char)28254, (char)18386, (char)38195, (char)40640, (char)4746, (char)59789, (char)54275, (char)26567, (char)56980, (char)44837, (char)47751, (char)55896, (char)61121, (char)42291, (char)65175, (char)16979, (char)21054, (char)61451, (char)44996, (char)57649, (char)64217, (char)38792, (char)60616, (char)44248, (char)59653, (char)59384, (char)42651, (char)23804, (char)34905, (char)221, (char)50243, (char)39656, (char)59191, (char)2220, (char)25149, (char)7556, (char)61412, (char)51210, (char)61063, (char)19888, (char)37344, (char)15254, (char)56718, (char)28071, (char)40856, (char)49270, (char)14267, (char)64000, (char)50571, (char)50835, (char)12006, (char)44107, (char)49754}));
            assert(pack.min_distance_GET() == (char)25563);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
            assert(pack.max_distance_GET() == (char)41202);
            assert(pack.time_usec_GET() == 4352877862863124406L);
        });
        DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.max_distance_SET((char)41202) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER) ;
        p330.time_usec_SET(4352877862863124406L) ;
        p330.increment_SET((char)35) ;
        p330.distances_SET(new char[] {(char)5006, (char)25217, (char)52582, (char)4490, (char)43722, (char)54247, (char)23665, (char)56705, (char)37386, (char)31835, (char)7853, (char)63037, (char)23114, (char)20965, (char)59925, (char)29665, (char)29403, (char)48486, (char)33914, (char)28254, (char)18386, (char)38195, (char)40640, (char)4746, (char)59789, (char)54275, (char)26567, (char)56980, (char)44837, (char)47751, (char)55896, (char)61121, (char)42291, (char)65175, (char)16979, (char)21054, (char)61451, (char)44996, (char)57649, (char)64217, (char)38792, (char)60616, (char)44248, (char)59653, (char)59384, (char)42651, (char)23804, (char)34905, (char)221, (char)50243, (char)39656, (char)59191, (char)2220, (char)25149, (char)7556, (char)61412, (char)51210, (char)61063, (char)19888, (char)37344, (char)15254, (char)56718, (char)28071, (char)40856, (char)49270, (char)14267, (char)64000, (char)50571, (char)50835, (char)12006, (char)44107, (char)49754}, 0) ;
        p330.min_distance_SET((char)25563) ;
        LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
    }

}