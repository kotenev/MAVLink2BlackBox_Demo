
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
            assert(pack.mavlink_version_GET() == (char)232);
            assert(pack.custom_mode_GET() == 3634135182L);
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_HEXAROTOR);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_UNINIT);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY);
        });
        DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
        PH.setPack(p0);
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY) ;
        p0.mavlink_version_SET((char)232) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_HEXAROTOR) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_UNINIT) ;
        p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED) ;
        p0.custom_mode_SET(3634135182L) ;
        LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.errors_comm_GET() == (char)35661);
            assert(pack.errors_count4_GET() == (char)22212);
            assert(pack.errors_count1_GET() == (char)33353);
            assert(pack.current_battery_GET() == (short) -437);
            assert(pack.drop_rate_comm_GET() == (char)46090);
            assert(pack.errors_count2_GET() == (char)57891);
            assert(pack.onboard_control_sensors_health_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN);
            assert(pack.load_GET() == (char)7345);
            assert(pack.onboard_control_sensors_present_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE);
            assert(pack.errors_count3_GET() == (char)34261);
            assert(pack.voltage_battery_GET() == (char)7979);
            assert(pack.onboard_control_sensors_enabled_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO);
            assert(pack.battery_remaining_GET() == (byte)52);
        });
        DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
        PH.setPack(p1);
        p1.errors_comm_SET((char)35661) ;
        p1.current_battery_SET((short) -437) ;
        p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_3D_GYRO) ;
        p1.drop_rate_comm_SET((char)46090) ;
        p1.errors_count4_SET((char)22212) ;
        p1.load_SET((char)7345) ;
        p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_TERRAIN) ;
        p1.errors_count2_SET((char)57891) ;
        p1.voltage_battery_SET((char)7979) ;
        p1.errors_count1_SET((char)33353) ;
        p1.battery_remaining_SET((byte)52) ;
        p1.errors_count3_SET((char)34261) ;
        p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_GEOFENCE) ;
        LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1685809646L);
            assert(pack.time_unix_usec_GET() == 7350234657238334688L);
        });
        DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(7350234657238334688L) ;
        p2.time_boot_ms_SET(1685809646L) ;
        LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == -9.675256E37F);
            assert(pack.afy_GET() == -1.2789063E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.x_GET() == -1.8275577E38F);
            assert(pack.y_GET() == 2.4457146E38F);
            assert(pack.type_mask_GET() == (char)19871);
            assert(pack.yaw_GET() == 2.2048868E38F);
            assert(pack.vy_GET() == -1.00734204E37F);
            assert(pack.afz_GET() == 2.0774528E38F);
            assert(pack.vx_GET() == -8.46921E37F);
            assert(pack.time_boot_ms_GET() == 1588672513L);
            assert(pack.afx_GET() == -1.594078E38F);
            assert(pack.vz_GET() == -3.0936962E38F);
            assert(pack.z_GET() == -1.274485E38F);
        });
        DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p3.yaw_SET(2.2048868E38F) ;
        p3.afx_SET(-1.594078E38F) ;
        p3.afy_SET(-1.2789063E38F) ;
        p3.yaw_rate_SET(-9.675256E37F) ;
        p3.x_SET(-1.8275577E38F) ;
        p3.afz_SET(2.0774528E38F) ;
        p3.z_SET(-1.274485E38F) ;
        p3.type_mask_SET((char)19871) ;
        p3.y_SET(2.4457146E38F) ;
        p3.vy_SET(-1.00734204E37F) ;
        p3.vz_SET(-3.0936962E38F) ;
        p3.time_boot_ms_SET(1588672513L) ;
        p3.vx_SET(-8.46921E37F) ;
        LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7035492684251032139L);
            assert(pack.seq_GET() == 2062974050L);
            assert(pack.target_component_GET() == (char)37);
            assert(pack.target_system_GET() == (char)196);
        });
        DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
        PH.setPack(p4);
        p4.seq_SET(2062974050L) ;
        p4.target_component_SET((char)37) ;
        p4.target_system_SET((char)196) ;
        p4.time_usec_SET(7035492684251032139L) ;
        LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)60);
            assert(pack.passkey_LEN(ph) == 17);
            assert(pack.passkey_TRY(ph).equals("ftenrsbybybongrNs"));
            assert(pack.version_GET() == (char)140);
            assert(pack.control_request_GET() == (char)141);
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.control_request_SET((char)141) ;
        p5.target_system_SET((char)60) ;
        p5.version_SET((char)140) ;
        p5.passkey_SET("ftenrsbybybongrNs", PH) ;
        LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.ack_GET() == (char)176);
            assert(pack.control_request_GET() == (char)109);
            assert(pack.gcs_system_id_GET() == (char)159);
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.ack_SET((char)176) ;
        p6.control_request_SET((char)109) ;
        p6.gcs_system_id_SET((char)159) ;
        LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 29);
            assert(pack.key_TRY(ph).equals("sfjtzlkzvjtyqxHagpyttxgtrsmwr"));
        });
        DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("sfjtzlkzvjtyqxHagpyttxgtrsmwr", PH) ;
        LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.custom_mode_GET() == 4152793860L);
            assert(pack.target_system_GET() == (char)173);
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_AUTO_DISARMED);
        });
        DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
        PH.setPack(p11);
        p11.base_mode_SET(MAV_MODE.MAV_MODE_AUTO_DISARMED) ;
        p11.custom_mode_SET(4152793860L) ;
        p11.target_system_SET((char)173) ;
        LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)8);
            assert(pack.target_component_GET() == (char)18);
            assert(pack.param_index_GET() == (short)24954);
            assert(pack.param_id_LEN(ph) == 13);
            assert(pack.param_id_TRY(ph).equals("mUptndbemcwte"));
        });
        DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.param_id_SET("mUptndbemcwte", PH) ;
        p20.target_component_SET((char)18) ;
        p20.param_index_SET((short)24954) ;
        p20.target_system_SET((char)8) ;
        LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)158);
            assert(pack.target_component_GET() == (char)223);
        });
        DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_component_SET((char)223) ;
        p21.target_system_SET((char)158) ;
        LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 3);
            assert(pack.param_id_TRY(ph).equals("ykm"));
            assert(pack.param_index_GET() == (char)47234);
            assert(pack.param_count_GET() == (char)59583);
            assert(pack.param_value_GET() == -5.740538E36F);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16);
        });
        DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
        PH.setPack(p22);
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT16) ;
        p22.param_index_SET((char)47234) ;
        p22.param_count_SET((char)59583) ;
        p22.param_id_SET("ykm", PH) ;
        p22.param_value_SET(-5.740538E36F) ;
        LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)159);
            assert(pack.target_component_GET() == (char)211);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32);
            assert(pack.param_value_GET() == -3.157439E38F);
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("W"));
        });
        DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
        PH.setPack(p23);
        p23.param_id_SET("W", PH) ;
        p23.param_value_SET(-3.157439E38F) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT32) ;
        p23.target_system_SET((char)159) ;
        p23.target_component_SET((char)211) ;
        LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 2028627516415625040L);
            assert(pack.alt_GET() == -1809695823);
            assert(pack.satellites_visible_GET() == (char)138);
            assert(pack.vel_acc_TRY(ph) == 2704536960L);
            assert(pack.v_acc_TRY(ph) == 3810031313L);
            assert(pack.lon_GET() == 1896795156);
            assert(pack.epv_GET() == (char)16993);
            assert(pack.hdg_acc_TRY(ph) == 330434057L);
            assert(pack.cog_GET() == (char)1480);
            assert(pack.vel_GET() == (char)13194);
            assert(pack.h_acc_TRY(ph) == 3912520585L);
            assert(pack.lat_GET() == -1623876608);
            assert(pack.alt_ellipsoid_TRY(ph) == -202309234);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
            assert(pack.eph_GET() == (char)42666);
        });
        DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
        PH.setPack(p24);
        p24.v_acc_SET(3810031313L, PH) ;
        p24.lon_SET(1896795156) ;
        p24.cog_SET((char)1480) ;
        p24.epv_SET((char)16993) ;
        p24.alt_ellipsoid_SET(-202309234, PH) ;
        p24.time_usec_SET(2028627516415625040L) ;
        p24.h_acc_SET(3912520585L, PH) ;
        p24.hdg_acc_SET(330434057L, PH) ;
        p24.vel_SET((char)13194) ;
        p24.lat_SET(-1623876608) ;
        p24.satellites_visible_SET((char)138) ;
        p24.alt_SET(-1809695823) ;
        p24.vel_acc_SET(2704536960L, PH) ;
        p24.eph_SET((char)42666) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED) ;
        LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.satellites_visible_GET() == (char)158);
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)113, (char)68, (char)123, (char)135, (char)39, (char)148, (char)1, (char)18, (char)145, (char)103, (char)70, (char)162, (char)99, (char)128, (char)97, (char)225, (char)179, (char)12, (char)233, (char)160}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)90, (char)5, (char)237, (char)116, (char)12, (char)246, (char)51, (char)9, (char)231, (char)111, (char)214, (char)241, (char)184, (char)89, (char)8, (char)131, (char)168, (char)111, (char)45, (char)246}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)170, (char)122, (char)190, (char)109, (char)189, (char)41, (char)86, (char)183, (char)42, (char)70, (char)79, (char)9, (char)54, (char)47, (char)227, (char)250, (char)133, (char)251, (char)65, (char)71}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)167, (char)2, (char)196, (char)198, (char)58, (char)55, (char)214, (char)253, (char)227, (char)74, (char)168, (char)181, (char)27, (char)218, (char)65, (char)165, (char)89, (char)152, (char)73, (char)154}));
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)241, (char)74, (char)0, (char)144, (char)214, (char)201, (char)145, (char)153, (char)23, (char)233, (char)126, (char)172, (char)222, (char)56, (char)130, (char)210, (char)141, (char)184, (char)39, (char)42}));
        });
        DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_prn_SET(new char[] {(char)113, (char)68, (char)123, (char)135, (char)39, (char)148, (char)1, (char)18, (char)145, (char)103, (char)70, (char)162, (char)99, (char)128, (char)97, (char)225, (char)179, (char)12, (char)233, (char)160}, 0) ;
        p25.satellite_azimuth_SET(new char[] {(char)241, (char)74, (char)0, (char)144, (char)214, (char)201, (char)145, (char)153, (char)23, (char)233, (char)126, (char)172, (char)222, (char)56, (char)130, (char)210, (char)141, (char)184, (char)39, (char)42}, 0) ;
        p25.satellite_used_SET(new char[] {(char)90, (char)5, (char)237, (char)116, (char)12, (char)246, (char)51, (char)9, (char)231, (char)111, (char)214, (char)241, (char)184, (char)89, (char)8, (char)131, (char)168, (char)111, (char)45, (char)246}, 0) ;
        p25.satellites_visible_SET((char)158) ;
        p25.satellite_elevation_SET(new char[] {(char)167, (char)2, (char)196, (char)198, (char)58, (char)55, (char)214, (char)253, (char)227, (char)74, (char)168, (char)181, (char)27, (char)218, (char)65, (char)165, (char)89, (char)152, (char)73, (char)154}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)170, (char)122, (char)190, (char)109, (char)189, (char)41, (char)86, (char)183, (char)42, (char)70, (char)79, (char)9, (char)54, (char)47, (char)227, (char)250, (char)133, (char)251, (char)65, (char)71}, 0) ;
        LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.zmag_GET() == (short)16153);
            assert(pack.xgyro_GET() == (short)25830);
            assert(pack.time_boot_ms_GET() == 914980387L);
            assert(pack.zacc_GET() == (short)9931);
            assert(pack.zgyro_GET() == (short)6529);
            assert(pack.ymag_GET() == (short) -29467);
            assert(pack.yacc_GET() == (short) -10002);
            assert(pack.xacc_GET() == (short)3240);
            assert(pack.ygyro_GET() == (short) -7568);
            assert(pack.xmag_GET() == (short)7533);
        });
        DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
        PH.setPack(p26);
        p26.zgyro_SET((short)6529) ;
        p26.zmag_SET((short)16153) ;
        p26.yacc_SET((short) -10002) ;
        p26.xacc_SET((short)3240) ;
        p26.xmag_SET((short)7533) ;
        p26.xgyro_SET((short)25830) ;
        p26.zacc_SET((short)9931) ;
        p26.time_boot_ms_SET(914980387L) ;
        p26.ymag_SET((short) -29467) ;
        p26.ygyro_SET((short) -7568) ;
        LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.xgyro_GET() == (short)29187);
            assert(pack.ygyro_GET() == (short) -9072);
            assert(pack.ymag_GET() == (short) -21343);
            assert(pack.time_usec_GET() == 4023680742860563418L);
            assert(pack.xmag_GET() == (short)22);
            assert(pack.zacc_GET() == (short)12571);
            assert(pack.zgyro_GET() == (short)20522);
            assert(pack.xacc_GET() == (short) -23810);
            assert(pack.zmag_GET() == (short)11269);
            assert(pack.yacc_GET() == (short) -28137);
        });
        DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
        PH.setPack(p27);
        p27.zgyro_SET((short)20522) ;
        p27.ygyro_SET((short) -9072) ;
        p27.xgyro_SET((short)29187) ;
        p27.xacc_SET((short) -23810) ;
        p27.time_usec_SET(4023680742860563418L) ;
        p27.zmag_SET((short)11269) ;
        p27.xmag_SET((short)22) ;
        p27.yacc_SET((short) -28137) ;
        p27.zacc_SET((short)12571) ;
        p27.ymag_SET((short) -21343) ;
        LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_diff2_GET() == (short)12964);
            assert(pack.time_usec_GET() == 1614959983093557129L);
            assert(pack.press_diff1_GET() == (short) -21171);
            assert(pack.press_abs_GET() == (short)1368);
            assert(pack.temperature_GET() == (short) -9423);
        });
        DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
        PH.setPack(p28);
        p28.temperature_SET((short) -9423) ;
        p28.time_usec_SET(1614959983093557129L) ;
        p28.press_diff2_SET((short)12964) ;
        p28.press_abs_SET((short)1368) ;
        p28.press_diff1_SET((short) -21171) ;
        LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short)6795);
            assert(pack.time_boot_ms_GET() == 3092453331L);
            assert(pack.press_diff_GET() == -2.7131457E38F);
            assert(pack.press_abs_GET() == -4.9344126E37F);
        });
        DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
        PH.setPack(p29);
        p29.press_abs_SET(-4.9344126E37F) ;
        p29.press_diff_SET(-2.7131457E38F) ;
        p29.time_boot_ms_SET(3092453331L) ;
        p29.temperature_SET((short)6795) ;
        LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -3.2226435E38F);
            assert(pack.rollspeed_GET() == 8.623131E37F);
            assert(pack.pitchspeed_GET() == -7.458963E37F);
            assert(pack.yawspeed_GET() == 8.3670066E37F);
            assert(pack.yaw_GET() == 2.263314E38F);
            assert(pack.roll_GET() == -1.3518611E38F);
            assert(pack.time_boot_ms_GET() == 3277505353L);
        });
        DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
        PH.setPack(p30);
        p30.time_boot_ms_SET(3277505353L) ;
        p30.pitchspeed_SET(-7.458963E37F) ;
        p30.pitch_SET(-3.2226435E38F) ;
        p30.roll_SET(-1.3518611E38F) ;
        p30.yaw_SET(2.263314E38F) ;
        p30.rollspeed_SET(8.623131E37F) ;
        p30.yawspeed_SET(8.3670066E37F) ;
        LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.q1_GET() == -4.8953127E37F);
            assert(pack.q3_GET() == -7.5440225E37F);
            assert(pack.rollspeed_GET() == 3.1242712E37F);
            assert(pack.yawspeed_GET() == 2.6259865E38F);
            assert(pack.q2_GET() == 1.4678318E38F);
            assert(pack.pitchspeed_GET() == -2.027708E37F);
            assert(pack.q4_GET() == 2.8501947E38F);
            assert(pack.time_boot_ms_GET() == 1162676473L);
        });
        DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.pitchspeed_SET(-2.027708E37F) ;
        p31.rollspeed_SET(3.1242712E37F) ;
        p31.yawspeed_SET(2.6259865E38F) ;
        p31.q4_SET(2.8501947E38F) ;
        p31.q2_SET(1.4678318E38F) ;
        p31.time_boot_ms_SET(1162676473L) ;
        p31.q1_SET(-4.8953127E37F) ;
        p31.q3_SET(-7.5440225E37F) ;
        LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3039385646L);
            assert(pack.y_GET() == 2.2025583E38F);
            assert(pack.vz_GET() == -9.063064E36F);
            assert(pack.vx_GET() == 3.381187E38F);
            assert(pack.vy_GET() == -4.4410724E37F);
            assert(pack.x_GET() == -1.8837929E38F);
            assert(pack.z_GET() == -1.8820344E38F);
        });
        DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.vx_SET(3.381187E38F) ;
        p32.y_SET(2.2025583E38F) ;
        p32.vz_SET(-9.063064E36F) ;
        p32.z_SET(-1.8820344E38F) ;
        p32.x_SET(-1.8837929E38F) ;
        p32.vy_SET(-4.4410724E37F) ;
        p32.time_boot_ms_SET(3039385646L) ;
        LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -939489518);
            assert(pack.hdg_GET() == (char)3289);
            assert(pack.lat_GET() == -1641982918);
            assert(pack.time_boot_ms_GET() == 3285188109L);
            assert(pack.vy_GET() == (short)2655);
            assert(pack.lon_GET() == -627905362);
            assert(pack.relative_alt_GET() == -1517593708);
            assert(pack.vz_GET() == (short) -7272);
            assert(pack.vx_GET() == (short)27473);
        });
        DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.vx_SET((short)27473) ;
        p33.vy_SET((short)2655) ;
        p33.alt_SET(-939489518) ;
        p33.lat_SET(-1641982918) ;
        p33.vz_SET((short) -7272) ;
        p33.time_boot_ms_SET(3285188109L) ;
        p33.hdg_SET((char)3289) ;
        p33.lon_SET(-627905362) ;
        p33.relative_alt_SET(-1517593708) ;
        LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan3_scaled_GET() == (short) -12500);
            assert(pack.chan8_scaled_GET() == (short) -21277);
            assert(pack.chan2_scaled_GET() == (short)6122);
            assert(pack.chan7_scaled_GET() == (short) -6475);
            assert(pack.time_boot_ms_GET() == 1761605517L);
            assert(pack.rssi_GET() == (char)9);
            assert(pack.port_GET() == (char)177);
            assert(pack.chan6_scaled_GET() == (short)31965);
            assert(pack.chan1_scaled_GET() == (short) -14743);
            assert(pack.chan5_scaled_GET() == (short)3213);
            assert(pack.chan4_scaled_GET() == (short)31428);
        });
        DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.chan8_scaled_SET((short) -21277) ;
        p34.chan3_scaled_SET((short) -12500) ;
        p34.port_SET((char)177) ;
        p34.time_boot_ms_SET(1761605517L) ;
        p34.rssi_SET((char)9) ;
        p34.chan1_scaled_SET((short) -14743) ;
        p34.chan4_scaled_SET((short)31428) ;
        p34.chan7_scaled_SET((short) -6475) ;
        p34.chan6_scaled_SET((short)31965) ;
        p34.chan5_scaled_SET((short)3213) ;
        p34.chan2_scaled_SET((short)6122) ;
        LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan4_raw_GET() == (char)48938);
            assert(pack.chan2_raw_GET() == (char)60980);
            assert(pack.chan1_raw_GET() == (char)19944);
            assert(pack.chan8_raw_GET() == (char)34640);
            assert(pack.rssi_GET() == (char)241);
            assert(pack.port_GET() == (char)114);
            assert(pack.chan7_raw_GET() == (char)33224);
            assert(pack.chan5_raw_GET() == (char)38842);
            assert(pack.chan3_raw_GET() == (char)13000);
            assert(pack.time_boot_ms_GET() == 1214045050L);
            assert(pack.chan6_raw_GET() == (char)27273);
        });
        DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan8_raw_SET((char)34640) ;
        p35.chan7_raw_SET((char)33224) ;
        p35.chan6_raw_SET((char)27273) ;
        p35.chan4_raw_SET((char)48938) ;
        p35.time_boot_ms_SET(1214045050L) ;
        p35.chan3_raw_SET((char)13000) ;
        p35.chan1_raw_SET((char)19944) ;
        p35.port_SET((char)114) ;
        p35.chan5_raw_SET((char)38842) ;
        p35.chan2_raw_SET((char)60980) ;
        p35.rssi_SET((char)241) ;
        LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo1_raw_GET() == (char)41509);
            assert(pack.servo2_raw_GET() == (char)2303);
            assert(pack.port_GET() == (char)73);
            assert(pack.servo5_raw_GET() == (char)44351);
            assert(pack.servo11_raw_TRY(ph) == (char)9738);
            assert(pack.servo13_raw_TRY(ph) == (char)31721);
            assert(pack.servo9_raw_TRY(ph) == (char)5515);
            assert(pack.servo14_raw_TRY(ph) == (char)46124);
            assert(pack.servo12_raw_TRY(ph) == (char)45960);
            assert(pack.servo10_raw_TRY(ph) == (char)53115);
            assert(pack.servo15_raw_TRY(ph) == (char)57866);
            assert(pack.servo7_raw_GET() == (char)19294);
            assert(pack.time_usec_GET() == 3157241568L);
            assert(pack.servo8_raw_GET() == (char)13785);
            assert(pack.servo16_raw_TRY(ph) == (char)4001);
            assert(pack.servo3_raw_GET() == (char)20566);
            assert(pack.servo4_raw_GET() == (char)59098);
            assert(pack.servo6_raw_GET() == (char)48834);
        });
        DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo6_raw_SET((char)48834) ;
        p36.servo12_raw_SET((char)45960, PH) ;
        p36.servo15_raw_SET((char)57866, PH) ;
        p36.servo5_raw_SET((char)44351) ;
        p36.servo8_raw_SET((char)13785) ;
        p36.servo11_raw_SET((char)9738, PH) ;
        p36.port_SET((char)73) ;
        p36.time_usec_SET(3157241568L) ;
        p36.servo4_raw_SET((char)59098) ;
        p36.servo7_raw_SET((char)19294) ;
        p36.servo13_raw_SET((char)31721, PH) ;
        p36.servo3_raw_SET((char)20566) ;
        p36.servo10_raw_SET((char)53115, PH) ;
        p36.servo9_raw_SET((char)5515, PH) ;
        p36.servo1_raw_SET((char)41509) ;
        p36.servo2_raw_SET((char)2303) ;
        p36.servo16_raw_SET((char)4001, PH) ;
        p36.servo14_raw_SET((char)46124, PH) ;
        LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.start_index_GET() == (short)11165);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.end_index_GET() == (short) -17377);
            assert(pack.target_system_GET() == (char)43);
            assert(pack.target_component_GET() == (char)163);
        });
        DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p37.target_component_SET((char)163) ;
        p37.end_index_SET((short) -17377) ;
        p37.target_system_SET((char)43) ;
        p37.start_index_SET((short)11165) ;
        LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_index_GET() == (short)31981);
            assert(pack.target_system_GET() == (char)29);
            assert(pack.start_index_GET() == (short)4212);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_component_GET() == (char)136);
        });
        DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.start_index_SET((short)4212) ;
        p38.target_component_SET((char)136) ;
        p38.target_system_SET((char)29) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p38.end_index_SET((short)31981) ;
        LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)218);
            assert(pack.param4_GET() == 1.5804682E38F);
            assert(pack.param2_GET() == -2.4679423E38F);
            assert(pack.current_GET() == (char)231);
            assert(pack.y_GET() == 2.810002E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_CONDITION_DELAY);
            assert(pack.target_system_GET() == (char)97);
            assert(pack.param3_GET() == -8.7982317E36F);
            assert(pack.x_GET() == 2.2508791E38F);
            assert(pack.seq_GET() == (char)50730);
            assert(pack.autocontinue_GET() == (char)133);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.param1_GET() == 2.8903281E38F);
            assert(pack.z_GET() == 6.162382E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
        });
        DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
        PH.setPack(p39);
        p39.x_SET(2.2508791E38F) ;
        p39.param3_SET(-8.7982317E36F) ;
        p39.seq_SET((char)50730) ;
        p39.param4_SET(1.5804682E38F) ;
        p39.z_SET(6.162382E37F) ;
        p39.target_system_SET((char)97) ;
        p39.target_component_SET((char)218) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p39.command_SET(MAV_CMD.MAV_CMD_CONDITION_DELAY) ;
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p39.y_SET(2.810002E38F) ;
        p39.autocontinue_SET((char)133) ;
        p39.param2_SET(-2.4679423E38F) ;
        p39.param1_SET(2.8903281E38F) ;
        p39.current_SET((char)231) ;
        LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)111);
            assert(pack.target_component_GET() == (char)47);
            assert(pack.seq_GET() == (char)44768);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
        });
        DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_component_SET((char)47) ;
        p40.seq_SET((char)44768) ;
        p40.target_system_SET((char)111) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)53933);
            assert(pack.target_system_GET() == (char)95);
            assert(pack.target_component_GET() == (char)142);
        });
        DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.target_component_SET((char)142) ;
        p41.seq_SET((char)53933) ;
        p41.target_system_SET((char)95) ;
        LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)33381);
        });
        DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)33381) ;
        LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_component_GET() == (char)123);
            assert(pack.target_system_GET() == (char)239);
        });
        DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p43.target_system_SET((char)239) ;
        p43.target_component_SET((char)123) ;
        LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)439);
            assert(pack.target_component_GET() == (char)156);
            assert(pack.target_system_GET() == (char)17);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
        PH.setPack(p44);
        p44.count_SET((char)439) ;
        p44.target_system_SET((char)17) ;
        p44.target_component_SET((char)156) ;
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)60);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)178);
        });
        DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)178) ;
        p45.target_component_SET((char)60) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)47001);
        });
        DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)47001) ;
        LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM6_Y);
            assert(pack.target_system_GET() == (char)147);
            assert(pack.target_component_GET() == (char)126);
        });
        DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
        PH.setPack(p47);
        p47.target_component_SET((char)126) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_INVALID_PARAM6_Y) ;
        p47.target_system_SET((char)147) ;
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == -967621561);
            assert(pack.longitude_GET() == -1577674883);
            assert(pack.altitude_GET() == -1962419505);
            assert(pack.target_system_GET() == (char)250);
            assert(pack.time_usec_TRY(ph) == 5143605272836672803L);
        });
        DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.time_usec_SET(5143605272836672803L, PH) ;
        p48.altitude_SET(-1962419505) ;
        p48.target_system_SET((char)250) ;
        p48.latitude_SET(-967621561) ;
        p48.longitude_SET(-1577674883) ;
        LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == -195748886);
            assert(pack.time_usec_TRY(ph) == 5587172624550593514L);
            assert(pack.latitude_GET() == -204178549);
            assert(pack.altitude_GET() == 1604756660);
        });
        DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.time_usec_SET(5587172624550593514L, PH) ;
        p49.longitude_SET(-195748886) ;
        p49.altitude_SET(1604756660) ;
        p49.latitude_SET(-204178549) ;
        LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.param_index_GET() == (short) -5952);
            assert(pack.target_component_GET() == (char)190);
            assert(pack.param_id_LEN(ph) == 5);
            assert(pack.param_id_TRY(ph).equals("roavm"));
            assert(pack.target_system_GET() == (char)217);
            assert(pack.param_value_min_GET() == -1.6538367E38F);
            assert(pack.param_value0_GET() == -7.4851477E37F);
            assert(pack.parameter_rc_channel_index_GET() == (char)208);
            assert(pack.scale_GET() == -1.9314776E38F);
            assert(pack.param_value_max_GET() == 1.8184981E38F);
        });
        DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
        PH.setPack(p50);
        p50.parameter_rc_channel_index_SET((char)208) ;
        p50.scale_SET(-1.9314776E38F) ;
        p50.param_value0_SET(-7.4851477E37F) ;
        p50.param_id_SET("roavm", PH) ;
        p50.param_value_min_SET(-1.6538367E38F) ;
        p50.target_system_SET((char)217) ;
        p50.target_component_SET((char)190) ;
        p50.param_index_SET((short) -5952) ;
        p50.param_value_max_SET(1.8184981E38F) ;
        LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)47);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.seq_GET() == (char)29959);
            assert(pack.target_system_GET() == (char)48);
        });
        DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p51.seq_SET((char)29959) ;
        p51.target_component_SET((char)47) ;
        p51.target_system_SET((char)48) ;
        LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)42);
            assert(pack.p2y_GET() == 3.1222107E38F);
            assert(pack.p1y_GET() == -1.3851196E38F);
            assert(pack.p1z_GET() == -1.1473651E38F);
            assert(pack.target_component_GET() == (char)102);
            assert(pack.p1x_GET() == 3.0155773E38F);
            assert(pack.p2x_GET() == 3.0630941E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.p2z_GET() == -3.063018E38F);
        });
        DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.p2y_SET(3.1222107E38F) ;
        p54.p2z_SET(-3.063018E38F) ;
        p54.p1x_SET(3.0155773E38F) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p54.p1y_SET(-1.3851196E38F) ;
        p54.p1z_SET(-1.1473651E38F) ;
        p54.target_system_SET((char)42) ;
        p54.p2x_SET(3.0630941E38F) ;
        p54.target_component_SET((char)102) ;
        LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2z_GET() == -1.031396E38F);
            assert(pack.p1x_GET() == 3.4018913E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED);
            assert(pack.p2y_GET() == -1.4384409E38F);
            assert(pack.p2x_GET() == 2.3114874E38F);
            assert(pack.p1z_GET() == 2.7266678E38F);
            assert(pack.p1y_GET() == 2.4784697E38F);
        });
        DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p2y_SET(-1.4384409E38F) ;
        p55.p1x_SET(3.4018913E38F) ;
        p55.p1z_SET(2.7266678E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_BODY_OFFSET_NED) ;
        p55.p1y_SET(2.4784697E38F) ;
        p55.p2z_SET(-1.031396E38F) ;
        p55.p2x_SET(2.3114874E38F) ;
        LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.yawspeed_GET() == 9.514887E37F);
            assert(pack.time_usec_GET() == 5646394443710543643L);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {1.9924044E38F, -2.6254664E38F, -1.0025305E38F, 3.3669366E38F, -3.1105884E38F, -2.532522E38F, -1.6486503E36F, 2.7677697E38F, 1.5107757E38F}));
            assert(pack.rollspeed_GET() == -1.4758521E38F);
            assert(pack.pitchspeed_GET() == 1.9454143E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.2622126E38F, 2.4946788E38F, 1.974342E38F, -3.3464907E38F}));
        });
        DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.yawspeed_SET(9.514887E37F) ;
        p61.rollspeed_SET(-1.4758521E38F) ;
        p61.pitchspeed_SET(1.9454143E38F) ;
        p61.q_SET(new float[] {-1.2622126E38F, 2.4946788E38F, 1.974342E38F, -3.3464907E38F}, 0) ;
        p61.time_usec_SET(5646394443710543643L) ;
        p61.covariance_SET(new float[] {1.9924044E38F, -2.6254664E38F, -1.0025305E38F, 3.3669366E38F, -3.1105884E38F, -2.532522E38F, -1.6486503E36F, 2.7677697E38F, 1.5107757E38F}, 0) ;
        LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.target_bearing_GET() == (short) -17731);
            assert(pack.aspd_error_GET() == 9.275455E37F);
            assert(pack.nav_roll_GET() == 7.1357726E37F);
            assert(pack.xtrack_error_GET() == 1.7751786E38F);
            assert(pack.wp_dist_GET() == (char)22427);
            assert(pack.nav_pitch_GET() == 2.3774586E38F);
            assert(pack.nav_bearing_GET() == (short)17571);
            assert(pack.alt_error_GET() == 2.5192469E38F);
        });
        DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.nav_roll_SET(7.1357726E37F) ;
        p62.wp_dist_SET((char)22427) ;
        p62.aspd_error_SET(9.275455E37F) ;
        p62.nav_pitch_SET(2.3774586E38F) ;
        p62.target_bearing_SET((short) -17731) ;
        p62.nav_bearing_SET((short)17571) ;
        p62.xtrack_error_SET(1.7751786E38F) ;
        p62.alt_error_SET(2.5192469E38F) ;
        LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1561976862);
            assert(pack.vy_GET() == -3.2749232E38F);
            assert(pack.vz_GET() == 6.706338E37F);
            assert(pack.vx_GET() == -4.2404066E37F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.0452709E38F, 1.0573726E37F, -2.6215103E37F, -2.763435E38F, -2.4268096E38F, 1.5496205E38F, -1.5726055E38F, 2.787976E38F, -3.3930119E38F, 5.0401083E37F, -2.7886871E38F, 2.0293133E38F, 1.3447744E38F, -2.879642E38F, 2.9177273E38F, -6.2045407E37F, 3.3095534E38F, -1.776455E38F, 1.792704E38F, 4.7868936E37F, -2.1737283E38F, 7.072103E37F, -3.0552288E38F, -2.843021E38F, 1.2530737E38F, -1.6177126E38F, -1.1200444E38F, -3.1030814E37F, 8.0073143E37F, 2.4678478E38F, 1.1991627E38F, -2.3774363E38F, -9.513187E37F, -2.3870102E38F, -2.842177E38F, 5.047361E37F}));
            assert(pack.lon_GET() == -1213772796);
            assert(pack.alt_GET() == 192549134);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS);
            assert(pack.time_usec_GET() == 1801906784737667644L);
            assert(pack.relative_alt_GET() == -727986157);
        });
        DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.lon_SET(-1213772796) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS) ;
        p63.vy_SET(-3.2749232E38F) ;
        p63.lat_SET(-1561976862) ;
        p63.covariance_SET(new float[] {2.0452709E38F, 1.0573726E37F, -2.6215103E37F, -2.763435E38F, -2.4268096E38F, 1.5496205E38F, -1.5726055E38F, 2.787976E38F, -3.3930119E38F, 5.0401083E37F, -2.7886871E38F, 2.0293133E38F, 1.3447744E38F, -2.879642E38F, 2.9177273E38F, -6.2045407E37F, 3.3095534E38F, -1.776455E38F, 1.792704E38F, 4.7868936E37F, -2.1737283E38F, 7.072103E37F, -3.0552288E38F, -2.843021E38F, 1.2530737E38F, -1.6177126E38F, -1.1200444E38F, -3.1030814E37F, 8.0073143E37F, 2.4678478E38F, 1.1991627E38F, -2.3774363E38F, -9.513187E37F, -2.3870102E38F, -2.842177E38F, 5.047361E37F}, 0) ;
        p63.vx_SET(-4.2404066E37F) ;
        p63.time_usec_SET(1801906784737667644L) ;
        p63.vz_SET(6.706338E37F) ;
        p63.alt_SET(192549134) ;
        p63.relative_alt_SET(-727986157) ;
        LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -6.401479E37F);
            assert(pack.vx_GET() == -2.9772464E38F);
            assert(pack.ax_GET() == -1.7128836E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {-2.1532833E37F, 5.1345234E37F, 4.8526426E37F, -3.3219822E38F, 2.1677278E38F, 1.6898297E38F, 5.440473E37F, -1.3995106E38F, -2.0536526E38F, -3.3362189E38F, 2.6077615E38F, 2.1034822E38F, 1.7954873E38F, 3.3070246E38F, -1.4771894E38F, -2.4128358E37F, -1.3903584E36F, 2.3189686E38F, 1.4930225E38F, -2.3435892E38F, 1.3063132E38F, -2.7126473E38F, -3.2320659E38F, -4.717023E37F, -2.5134065E38F, 2.4604374E38F, 1.650256E38F, -2.0797086E38F, 2.7240008E38F, 1.4659194E37F, -2.5660879E38F, 1.7562274E37F, 3.3371125E38F, 3.2381508E38F, -1.8611364E38F, -2.8161245E38F, -2.8292085E38F, -2.24737E38F, -5.1784866E37F, 5.4868984E37F, 1.7242784E38F, 2.3840386E38F, 4.683955E37F, -1.4682738E38F, 3.0780546E38F}));
            assert(pack.ay_GET() == 6.4110344E35F);
            assert(pack.vz_GET() == 2.4577161E38F);
            assert(pack.vy_GET() == 1.58214E38F);
            assert(pack.y_GET() == 6.484228E37F);
            assert(pack.az_GET() == 9.439116E37F);
            assert(pack.z_GET() == 3.3404279E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
            assert(pack.time_usec_GET() == 8543489862977121954L);
        });
        DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.ax_SET(-1.7128836E38F) ;
        p64.vz_SET(2.4577161E38F) ;
        p64.y_SET(6.484228E37F) ;
        p64.covariance_SET(new float[] {-2.1532833E37F, 5.1345234E37F, 4.8526426E37F, -3.3219822E38F, 2.1677278E38F, 1.6898297E38F, 5.440473E37F, -1.3995106E38F, -2.0536526E38F, -3.3362189E38F, 2.6077615E38F, 2.1034822E38F, 1.7954873E38F, 3.3070246E38F, -1.4771894E38F, -2.4128358E37F, -1.3903584E36F, 2.3189686E38F, 1.4930225E38F, -2.3435892E38F, 1.3063132E38F, -2.7126473E38F, -3.2320659E38F, -4.717023E37F, -2.5134065E38F, 2.4604374E38F, 1.650256E38F, -2.0797086E38F, 2.7240008E38F, 1.4659194E37F, -2.5660879E38F, 1.7562274E37F, 3.3371125E38F, 3.2381508E38F, -1.8611364E38F, -2.8161245E38F, -2.8292085E38F, -2.24737E38F, -5.1784866E37F, 5.4868984E37F, 1.7242784E38F, 2.3840386E38F, 4.683955E37F, -1.4682738E38F, 3.0780546E38F}, 0) ;
        p64.z_SET(3.3404279E38F) ;
        p64.vy_SET(1.58214E38F) ;
        p64.az_SET(9.439116E37F) ;
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS) ;
        p64.vx_SET(-2.9772464E38F) ;
        p64.x_SET(-6.401479E37F) ;
        p64.time_usec_SET(8543489862977121954L) ;
        p64.ay_SET(6.4110344E35F) ;
        LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan13_raw_GET() == (char)28527);
            assert(pack.chan7_raw_GET() == (char)37852);
            assert(pack.chancount_GET() == (char)252);
            assert(pack.chan6_raw_GET() == (char)52394);
            assert(pack.chan17_raw_GET() == (char)13161);
            assert(pack.chan9_raw_GET() == (char)10293);
            assert(pack.chan2_raw_GET() == (char)34570);
            assert(pack.chan18_raw_GET() == (char)8105);
            assert(pack.chan8_raw_GET() == (char)15873);
            assert(pack.chan3_raw_GET() == (char)43741);
            assert(pack.chan16_raw_GET() == (char)18859);
            assert(pack.chan10_raw_GET() == (char)61304);
            assert(pack.chan4_raw_GET() == (char)53661);
            assert(pack.time_boot_ms_GET() == 2779590043L);
            assert(pack.chan5_raw_GET() == (char)23686);
            assert(pack.chan15_raw_GET() == (char)33940);
            assert(pack.chan14_raw_GET() == (char)21602);
            assert(pack.rssi_GET() == (char)52);
            assert(pack.chan11_raw_GET() == (char)4029);
            assert(pack.chan12_raw_GET() == (char)26335);
            assert(pack.chan1_raw_GET() == (char)60359);
        });
        DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
        PH.setPack(p65);
        p65.rssi_SET((char)52) ;
        p65.chan16_raw_SET((char)18859) ;
        p65.chan7_raw_SET((char)37852) ;
        p65.chan5_raw_SET((char)23686) ;
        p65.chan4_raw_SET((char)53661) ;
        p65.time_boot_ms_SET(2779590043L) ;
        p65.chan8_raw_SET((char)15873) ;
        p65.chan9_raw_SET((char)10293) ;
        p65.chan3_raw_SET((char)43741) ;
        p65.chan17_raw_SET((char)13161) ;
        p65.chan2_raw_SET((char)34570) ;
        p65.chancount_SET((char)252) ;
        p65.chan12_raw_SET((char)26335) ;
        p65.chan6_raw_SET((char)52394) ;
        p65.chan13_raw_SET((char)28527) ;
        p65.chan11_raw_SET((char)4029) ;
        p65.chan15_raw_SET((char)33940) ;
        p65.chan10_raw_SET((char)61304) ;
        p65.chan18_raw_SET((char)8105) ;
        p65.chan1_raw_SET((char)60359) ;
        p65.chan14_raw_SET((char)21602) ;
        LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)235);
            assert(pack.start_stop_GET() == (char)141);
            assert(pack.req_message_rate_GET() == (char)61099);
            assert(pack.req_stream_id_GET() == (char)3);
            assert(pack.target_system_GET() == (char)8);
        });
        DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.req_message_rate_SET((char)61099) ;
        p66.target_system_SET((char)8) ;
        p66.req_stream_id_SET((char)3) ;
        p66.start_stop_SET((char)141) ;
        p66.target_component_SET((char)235) ;
        LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.stream_id_GET() == (char)74);
            assert(pack.message_rate_GET() == (char)3258);
            assert(pack.on_off_GET() == (char)218);
        });
        DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
        PH.setPack(p67);
        p67.message_rate_SET((char)3258) ;
        p67.stream_id_SET((char)74) ;
        p67.on_off_SET((char)218) ;
        LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == (short)5013);
            assert(pack.z_GET() == (short) -15385);
            assert(pack.buttons_GET() == (char)3799);
            assert(pack.target_GET() == (char)87);
            assert(pack.y_GET() == (short) -18176);
            assert(pack.r_GET() == (short) -21522);
        });
        DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
        PH.setPack(p69);
        p69.y_SET((short) -18176) ;
        p69.x_SET((short)5013) ;
        p69.buttons_SET((char)3799) ;
        p69.z_SET((short) -15385) ;
        p69.r_SET((short) -21522) ;
        p69.target_SET((char)87) ;
        LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan4_raw_GET() == (char)22532);
            assert(pack.chan6_raw_GET() == (char)60597);
            assert(pack.target_system_GET() == (char)235);
            assert(pack.chan7_raw_GET() == (char)39371);
            assert(pack.chan1_raw_GET() == (char)15729);
            assert(pack.target_component_GET() == (char)220);
            assert(pack.chan3_raw_GET() == (char)65381);
            assert(pack.chan5_raw_GET() == (char)37177);
            assert(pack.chan8_raw_GET() == (char)21283);
            assert(pack.chan2_raw_GET() == (char)62367);
        });
        DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan6_raw_SET((char)60597) ;
        p70.target_component_SET((char)220) ;
        p70.chan7_raw_SET((char)39371) ;
        p70.target_system_SET((char)235) ;
        p70.chan2_raw_SET((char)62367) ;
        p70.chan3_raw_SET((char)65381) ;
        p70.chan4_raw_SET((char)22532) ;
        p70.chan5_raw_SET((char)37177) ;
        p70.chan8_raw_SET((char)21283) ;
        p70.chan1_raw_SET((char)15729) ;
        LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)53790);
            assert(pack.target_component_GET() == (char)77);
            assert(pack.target_system_GET() == (char)81);
            assert(pack.y_GET() == 393925220);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_WAYPOINT_USER_4);
            assert(pack.x_GET() == -1654697826);
            assert(pack.param1_GET() == -4.8017175E37F);
            assert(pack.z_GET() == -3.0593672E38F);
            assert(pack.autocontinue_GET() == (char)196);
            assert(pack.param3_GET() == 3.1892872E38F);
            assert(pack.current_GET() == (char)157);
            assert(pack.param2_GET() == -3.0072074E38F);
            assert(pack.param4_GET() == 1.6742091E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
        });
        DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.param4_SET(1.6742091E38F) ;
        p73.param1_SET(-4.8017175E37F) ;
        p73.param2_SET(-3.0072074E38F) ;
        p73.x_SET(-1654697826) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p73.z_SET(-3.0593672E38F) ;
        p73.command_SET(MAV_CMD.MAV_CMD_WAYPOINT_USER_4) ;
        p73.y_SET(393925220) ;
        p73.target_system_SET((char)81) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION) ;
        p73.current_SET((char)157) ;
        p73.seq_SET((char)53790) ;
        p73.param3_SET(3.1892872E38F) ;
        p73.target_component_SET((char)77) ;
        p73.autocontinue_SET((char)196) ;
        LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.heading_GET() == (short) -12009);
            assert(pack.climb_GET() == -1.7375679E38F);
            assert(pack.throttle_GET() == (char)11507);
            assert(pack.groundspeed_GET() == -5.6527223E37F);
            assert(pack.alt_GET() == -1.0391809E38F);
            assert(pack.airspeed_GET() == -2.689611E38F);
        });
        DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
        PH.setPack(p74);
        p74.climb_SET(-1.7375679E38F) ;
        p74.alt_SET(-1.0391809E38F) ;
        p74.heading_SET((short) -12009) ;
        p74.airspeed_SET(-2.689611E38F) ;
        p74.throttle_SET((char)11507) ;
        p74.groundspeed_SET(-5.6527223E37F) ;
        LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.param4_GET() == 7.8762225E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
            assert(pack.x_GET() == -2031864995);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION);
            assert(pack.target_component_GET() == (char)189);
            assert(pack.param1_GET() == 2.038467E38F);
            assert(pack.param2_GET() == -3.3889158E38F);
            assert(pack.autocontinue_GET() == (char)39);
            assert(pack.y_GET() == 2040855310);
            assert(pack.target_system_GET() == (char)112);
            assert(pack.param3_GET() == -3.2714825E38F);
            assert(pack.current_GET() == (char)125);
            assert(pack.z_GET() == -1.8464718E37F);
        });
        DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
        PH.setPack(p75);
        p75.command_SET(MAV_CMD.MAV_CMD_REQUEST_STORAGE_INFORMATION) ;
        p75.target_component_SET((char)189) ;
        p75.z_SET(-1.8464718E37F) ;
        p75.y_SET(2040855310) ;
        p75.param3_SET(-3.2714825E38F) ;
        p75.param4_SET(7.8762225E37F) ;
        p75.target_system_SET((char)112) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p75.current_SET((char)125) ;
        p75.autocontinue_SET((char)39) ;
        p75.param2_SET(-3.3889158E38F) ;
        p75.param1_SET(2.038467E38F) ;
        p75.x_SET(-2031864995) ;
        LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)187);
            assert(pack.param3_GET() == 1.8294642E38F);
            assert(pack.param1_GET() == -1.913148E35F);
            assert(pack.param5_GET() == 5.0517535E37F);
            assert(pack.confirmation_GET() == (char)98);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION);
            assert(pack.target_system_GET() == (char)219);
            assert(pack.param4_GET() == 1.2712837E38F);
            assert(pack.param6_GET() == -2.2372806E38F);
            assert(pack.param7_GET() == -4.636765E37F);
            assert(pack.param2_GET() == 1.3363531E38F);
        });
        DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.target_component_SET((char)187) ;
        p76.param4_SET(1.2712837E38F) ;
        p76.param2_SET(1.3363531E38F) ;
        p76.target_system_SET((char)219) ;
        p76.param5_SET(5.0517535E37F) ;
        p76.param3_SET(1.8294642E38F) ;
        p76.command_SET(MAV_CMD.MAV_CMD_AIRFRAME_CONFIGURATION) ;
        p76.param1_SET(-1.913148E35F) ;
        p76.param7_SET(-4.636765E37F) ;
        p76.param6_SET(-2.2372806E38F) ;
        p76.confirmation_SET((char)98) ;
        LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_TRY(ph) == (char)94);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_LAND);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED);
            assert(pack.result_param2_TRY(ph) == -1538114994);
            assert(pack.target_component_TRY(ph) == (char)114);
            assert(pack.progress_TRY(ph) == (char)66);
        });
        DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.result_SET(MAV_RESULT.MAV_RESULT_TEMPORARILY_REJECTED) ;
        p77.result_param2_SET(-1538114994, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_NAV_LAND) ;
        p77.target_system_SET((char)94, PH) ;
        p77.target_component_SET((char)114, PH) ;
        p77.progress_SET((char)66, PH) ;
        LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -2.3581275E38F);
            assert(pack.thrust_GET() == -2.6393558E37F);
            assert(pack.time_boot_ms_GET() == 1029471221L);
            assert(pack.roll_GET() == 1.7557963E38F);
            assert(pack.mode_switch_GET() == (char)242);
            assert(pack.manual_override_switch_GET() == (char)78);
            assert(pack.pitch_GET() == 3.0811098E38F);
        });
        DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.time_boot_ms_SET(1029471221L) ;
        p81.manual_override_switch_SET((char)78) ;
        p81.yaw_SET(-2.3581275E38F) ;
        p81.pitch_SET(3.0811098E38F) ;
        p81.mode_switch_SET((char)242) ;
        p81.thrust_SET(-2.6393558E37F) ;
        p81.roll_SET(1.7557963E38F) ;
        LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_roll_rate_GET() == -2.3266443E38F);
            assert(pack.target_system_GET() == (char)32);
            assert(Arrays.equals(pack.q_GET(),  new float[] {5.7582663E37F, 2.1350475E38F, -1.056438E38F, 5.434048E37F}));
            assert(pack.body_yaw_rate_GET() == -1.2000825E38F);
            assert(pack.thrust_GET() == -2.2010787E38F);
            assert(pack.type_mask_GET() == (char)140);
            assert(pack.time_boot_ms_GET() == 2246219312L);
            assert(pack.body_pitch_rate_GET() == 1.4126204E36F);
            assert(pack.target_component_GET() == (char)81);
        });
        DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.type_mask_SET((char)140) ;
        p82.target_component_SET((char)81) ;
        p82.body_pitch_rate_SET(1.4126204E36F) ;
        p82.target_system_SET((char)32) ;
        p82.body_yaw_rate_SET(-1.2000825E38F) ;
        p82.time_boot_ms_SET(2246219312L) ;
        p82.q_SET(new float[] {5.7582663E37F, 2.1350475E38F, -1.056438E38F, 5.434048E37F}, 0) ;
        p82.body_roll_rate_SET(-2.3266443E38F) ;
        p82.thrust_SET(-2.2010787E38F) ;
        LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.type_mask_GET() == (char)176);
            assert(pack.body_yaw_rate_GET() == -2.146837E38F);
            assert(pack.thrust_GET() == 9.719561E37F);
            assert(pack.body_roll_rate_GET() == 3.1039342E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {2.3015541E38F, 3.3915404E38F, 3.3812627E38F, 3.072774E38F}));
            assert(pack.body_pitch_rate_GET() == 2.3260731E38F);
            assert(pack.time_boot_ms_GET() == 937629010L);
        });
        DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.q_SET(new float[] {2.3015541E38F, 3.3915404E38F, 3.3812627E38F, 3.072774E38F}, 0) ;
        p83.body_pitch_rate_SET(2.3260731E38F) ;
        p83.body_roll_rate_SET(3.1039342E38F) ;
        p83.type_mask_SET((char)176) ;
        p83.body_yaw_rate_SET(-2.146837E38F) ;
        p83.thrust_SET(9.719561E37F) ;
        p83.time_boot_ms_SET(937629010L) ;
        LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.vy_GET() == -1.9972103E38F);
            assert(pack.yaw_rate_GET() == -2.798588E38F);
            assert(pack.y_GET() == 8.0489926E37F);
            assert(pack.target_component_GET() == (char)80);
            assert(pack.target_system_GET() == (char)97);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.afy_GET() == -2.7999915E38F);
            assert(pack.type_mask_GET() == (char)33707);
            assert(pack.z_GET() == -2.4651413E38F);
            assert(pack.afx_GET() == -1.1904275E38F);
            assert(pack.vz_GET() == 7.4822316E37F);
            assert(pack.vx_GET() == -2.8557162E38F);
            assert(pack.yaw_GET() == -7.9551337E37F);
            assert(pack.x_GET() == -5.7482134E37F);
            assert(pack.afz_GET() == 2.85627E37F);
            assert(pack.time_boot_ms_GET() == 3423792535L);
        });
        DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.target_system_SET((char)97) ;
        p84.type_mask_SET((char)33707) ;
        p84.afx_SET(-1.1904275E38F) ;
        p84.yaw_SET(-7.9551337E37F) ;
        p84.time_boot_ms_SET(3423792535L) ;
        p84.y_SET(8.0489926E37F) ;
        p84.vz_SET(7.4822316E37F) ;
        p84.target_component_SET((char)80) ;
        p84.vy_SET(-1.9972103E38F) ;
        p84.yaw_rate_SET(-2.798588E38F) ;
        p84.z_SET(-2.4651413E38F) ;
        p84.afy_SET(-2.7999915E38F) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p84.x_SET(-5.7482134E37F) ;
        p84.vx_SET(-2.8557162E38F) ;
        p84.afz_SET(2.85627E37F) ;
        LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.afz_GET() == 9.324773E37F);
            assert(pack.lat_int_GET() == 821366387);
            assert(pack.target_system_GET() == (char)37);
            assert(pack.vy_GET() == -2.8272216E37F);
            assert(pack.vz_GET() == -2.3420545E38F);
            assert(pack.lon_int_GET() == -2104046237);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.yaw_rate_GET() == -1.2320412E38F);
            assert(pack.afx_GET() == 2.7885796E38F);
            assert(pack.vx_GET() == -1.0796554E38F);
            assert(pack.type_mask_GET() == (char)17699);
            assert(pack.afy_GET() == -2.2971103E38F);
            assert(pack.alt_GET() == -6.4786345E36F);
            assert(pack.time_boot_ms_GET() == 1132332270L);
            assert(pack.yaw_GET() == -3.0547913E38F);
            assert(pack.target_component_GET() == (char)34);
        });
        DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.target_component_SET((char)34) ;
        p86.yaw_rate_SET(-1.2320412E38F) ;
        p86.vz_SET(-2.3420545E38F) ;
        p86.afz_SET(9.324773E37F) ;
        p86.lon_int_SET(-2104046237) ;
        p86.afx_SET(2.7885796E38F) ;
        p86.alt_SET(-6.4786345E36F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p86.time_boot_ms_SET(1132332270L) ;
        p86.lat_int_SET(821366387) ;
        p86.target_system_SET((char)37) ;
        p86.type_mask_SET((char)17699) ;
        p86.yaw_SET(-3.0547913E38F) ;
        p86.vx_SET(-1.0796554E38F) ;
        p86.vy_SET(-2.8272216E37F) ;
        p86.afy_SET(-2.2971103E38F) ;
        LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.vx_GET() == 2.3311113E38F);
            assert(pack.yaw_rate_GET() == -2.1110766E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_INT);
            assert(pack.afx_GET() == -2.9291899E38F);
            assert(pack.afz_GET() == 1.5698391E38F);
            assert(pack.yaw_GET() == 3.364019E38F);
            assert(pack.type_mask_GET() == (char)31803);
            assert(pack.lon_int_GET() == 2113407509);
            assert(pack.time_boot_ms_GET() == 252095188L);
            assert(pack.vz_GET() == -7.759574E37F);
            assert(pack.afy_GET() == -7.370383E37F);
            assert(pack.vy_GET() == 1.8233984E38F);
            assert(pack.lat_int_GET() == 2118447352);
            assert(pack.alt_GET() == -2.9991477E38F);
        });
        DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.vz_SET(-7.759574E37F) ;
        p87.vx_SET(2.3311113E38F) ;
        p87.type_mask_SET((char)31803) ;
        p87.time_boot_ms_SET(252095188L) ;
        p87.afz_SET(1.5698391E38F) ;
        p87.yaw_rate_SET(-2.1110766E38F) ;
        p87.afy_SET(-7.370383E37F) ;
        p87.yaw_SET(3.364019E38F) ;
        p87.afx_SET(-2.9291899E38F) ;
        p87.lon_int_SET(2113407509) ;
        p87.vy_SET(1.8233984E38F) ;
        p87.alt_SET(-2.9991477E38F) ;
        p87.lat_int_SET(2118447352) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_INT) ;
        LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 1.9478376E38F);
            assert(pack.z_GET() == 1.7711563E38F);
            assert(pack.pitch_GET() == 1.4657461E38F);
            assert(pack.y_GET() == -2.076744E38F);
            assert(pack.x_GET() == 5.825416E37F);
            assert(pack.roll_GET() == -3.1106696E37F);
            assert(pack.time_boot_ms_GET() == 1547497247L);
        });
        DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.x_SET(5.825416E37F) ;
        p89.pitch_SET(1.4657461E38F) ;
        p89.z_SET(1.7711563E38F) ;
        p89.time_boot_ms_SET(1547497247L) ;
        p89.y_SET(-2.076744E38F) ;
        p89.yaw_SET(1.9478376E38F) ;
        p89.roll_SET(-3.1106696E37F) ;
        LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == 5.3086565E37F);
            assert(pack.pitchspeed_GET() == -1.6754516E37F);
            assert(pack.zacc_GET() == (short)28521);
            assert(pack.yacc_GET() == (short)9802);
            assert(pack.yawspeed_GET() == -2.0381836E38F);
            assert(pack.xacc_GET() == (short)15568);
            assert(pack.alt_GET() == 1647069462);
            assert(pack.rollspeed_GET() == 7.574949E37F);
            assert(pack.lat_GET() == -1178766426);
            assert(pack.time_usec_GET() == 6282050948783911538L);
            assert(pack.lon_GET() == 834707010);
            assert(pack.vx_GET() == (short) -2567);
            assert(pack.yaw_GET() == 2.3015846E38F);
            assert(pack.pitch_GET() == 1.2334264E38F);
            assert(pack.vy_GET() == (short)27623);
            assert(pack.vz_GET() == (short) -9386);
        });
        DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.pitchspeed_SET(-1.6754516E37F) ;
        p90.zacc_SET((short)28521) ;
        p90.time_usec_SET(6282050948783911538L) ;
        p90.vy_SET((short)27623) ;
        p90.yawspeed_SET(-2.0381836E38F) ;
        p90.lat_SET(-1178766426) ;
        p90.yaw_SET(2.3015846E38F) ;
        p90.vz_SET((short) -9386) ;
        p90.lon_SET(834707010) ;
        p90.alt_SET(1647069462) ;
        p90.pitch_SET(1.2334264E38F) ;
        p90.roll_SET(5.3086565E37F) ;
        p90.yacc_SET((short)9802) ;
        p90.xacc_SET((short)15568) ;
        p90.vx_SET((short) -2567) ;
        p90.rollspeed_SET(7.574949E37F) ;
        LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.throttle_GET() == -2.0998776E38F);
            assert(pack.aux4_GET() == 1.9154515E38F);
            assert(pack.aux2_GET() == -2.2616781E38F);
            assert(pack.roll_ailerons_GET() == 2.105888E38F);
            assert(pack.nav_mode_GET() == (char)251);
            assert(pack.aux1_GET() == 2.1154663E38F);
            assert(pack.time_usec_GET() == 3723713971888888274L);
            assert(pack.pitch_elevator_GET() == 2.8877336E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_TEST_ARMED);
            assert(pack.yaw_rudder_GET() == 1.3166987E38F);
            assert(pack.aux3_GET() == -4.1171286E37F);
        });
        DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.roll_ailerons_SET(2.105888E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_TEST_ARMED) ;
        p91.aux4_SET(1.9154515E38F) ;
        p91.aux1_SET(2.1154663E38F) ;
        p91.aux3_SET(-4.1171286E37F) ;
        p91.aux2_SET(-2.2616781E38F) ;
        p91.nav_mode_SET((char)251) ;
        p91.throttle_SET(-2.0998776E38F) ;
        p91.pitch_elevator_SET(2.8877336E38F) ;
        p91.time_usec_SET(3723713971888888274L) ;
        p91.yaw_rudder_SET(1.3166987E38F) ;
        LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.chan2_raw_GET() == (char)60081);
            assert(pack.chan10_raw_GET() == (char)16518);
            assert(pack.rssi_GET() == (char)180);
            assert(pack.chan8_raw_GET() == (char)14821);
            assert(pack.chan9_raw_GET() == (char)8999);
            assert(pack.time_usec_GET() == 6332990934270375748L);
            assert(pack.chan12_raw_GET() == (char)35234);
            assert(pack.chan11_raw_GET() == (char)33740);
            assert(pack.chan5_raw_GET() == (char)51530);
            assert(pack.chan4_raw_GET() == (char)11327);
            assert(pack.chan7_raw_GET() == (char)46837);
            assert(pack.chan6_raw_GET() == (char)28072);
            assert(pack.chan3_raw_GET() == (char)62692);
            assert(pack.chan1_raw_GET() == (char)33769);
        });
        DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan8_raw_SET((char)14821) ;
        p92.chan11_raw_SET((char)33740) ;
        p92.chan12_raw_SET((char)35234) ;
        p92.chan10_raw_SET((char)16518) ;
        p92.chan9_raw_SET((char)8999) ;
        p92.chan5_raw_SET((char)51530) ;
        p92.chan4_raw_SET((char)11327) ;
        p92.chan3_raw_SET((char)62692) ;
        p92.chan6_raw_SET((char)28072) ;
        p92.chan1_raw_SET((char)33769) ;
        p92.time_usec_SET(6332990934270375748L) ;
        p92.chan7_raw_SET((char)46837) ;
        p92.chan2_raw_SET((char)60081) ;
        p92.rssi_SET((char)180) ;
        LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == 509161684874860082L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-2.1259012E38F, -1.6440645E37F, 1.8136484E38F, -3.0772153E38F, 6.3085893E36F, 1.0746806E38F, -1.4002627E38F, 1.9482774E38F, -1.6692313E38F, -4.206497E36F, 1.0881413E38F, -6.839544E37F, 1.9390957E38F, 1.5717466E38F, 2.504607E38F, 6.3256297E37F}));
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_AUTO_ARMED);
            assert(pack.time_usec_GET() == 4616415233921135944L);
        });
        DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.time_usec_SET(4616415233921135944L) ;
        p93.controls_SET(new float[] {-2.1259012E38F, -1.6440645E37F, 1.8136484E38F, -3.0772153E38F, 6.3085893E36F, 1.0746806E38F, -1.4002627E38F, 1.9482774E38F, -1.6692313E38F, -4.206497E36F, 1.0881413E38F, -6.839544E37F, 1.9390957E38F, 1.5717466E38F, 2.504607E38F, 6.3256297E37F}, 0) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED) ;
        p93.flags_SET(509161684874860082L) ;
        LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_rate_x_TRY(ph) == -9.386883E36F);
            assert(pack.flow_x_GET() == (short)12344);
            assert(pack.flow_comp_m_x_GET() == -5.163843E37F);
            assert(pack.ground_distance_GET() == -2.1454995E38F);
            assert(pack.sensor_id_GET() == (char)208);
            assert(pack.flow_comp_m_y_GET() == 1.6249379E38F);
            assert(pack.flow_y_GET() == (short) -7797);
            assert(pack.time_usec_GET() == 7088088433816244002L);
            assert(pack.flow_rate_y_TRY(ph) == -2.009944E38F);
            assert(pack.quality_GET() == (char)153);
        });
        DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.sensor_id_SET((char)208) ;
        p100.flow_x_SET((short)12344) ;
        p100.flow_comp_m_x_SET(-5.163843E37F) ;
        p100.time_usec_SET(7088088433816244002L) ;
        p100.flow_rate_y_SET(-2.009944E38F, PH) ;
        p100.flow_comp_m_y_SET(1.6249379E38F) ;
        p100.flow_y_SET((short) -7797) ;
        p100.quality_SET((char)153) ;
        p100.ground_distance_SET(-2.1454995E38F) ;
        p100.flow_rate_x_SET(-9.386883E36F, PH) ;
        LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -1.7741239E38F);
            assert(pack.y_GET() == -3.1702185E37F);
            assert(pack.usec_GET() == 5838447124263248550L);
            assert(pack.yaw_GET() == 4.685348E37F);
            assert(pack.z_GET() == -2.3204137E38F);
            assert(pack.x_GET() == -9.710522E37F);
            assert(pack.roll_GET() == 1.4767218E38F);
        });
        DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.z_SET(-2.3204137E38F) ;
        p101.usec_SET(5838447124263248550L) ;
        p101.x_SET(-9.710522E37F) ;
        p101.roll_SET(1.4767218E38F) ;
        p101.pitch_SET(-1.7741239E38F) ;
        p101.yaw_SET(4.685348E37F) ;
        p101.y_SET(-3.1702185E37F) ;
        LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == -2.9060146E38F);
            assert(pack.usec_GET() == 6454426981247882181L);
            assert(pack.z_GET() == -5.3372476E37F);
            assert(pack.x_GET() == -1.6354106E38F);
            assert(pack.y_GET() == -2.3374497E38F);
            assert(pack.pitch_GET() == 8.598642E37F);
            assert(pack.roll_GET() == 2.1457246E38F);
        });
        DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.pitch_SET(8.598642E37F) ;
        p102.roll_SET(2.1457246E38F) ;
        p102.yaw_SET(-2.9060146E38F) ;
        p102.y_SET(-2.3374497E38F) ;
        p102.x_SET(-1.6354106E38F) ;
        p102.usec_SET(6454426981247882181L) ;
        p102.z_SET(-5.3372476E37F) ;
        LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 3.2142336E38F);
            assert(pack.usec_GET() == 8780521763036716239L);
            assert(pack.y_GET() == 2.0385834E38F);
            assert(pack.z_GET() == 2.117022E38F);
        });
        DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.y_SET(2.0385834E38F) ;
        p103.z_SET(2.117022E38F) ;
        p103.usec_SET(8780521763036716239L) ;
        p103.x_SET(3.2142336E38F) ;
        LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.z_GET() == -1.7307009E38F);
            assert(pack.yaw_GET() == 1.937978E38F);
            assert(pack.usec_GET() == 7429537837165040229L);
            assert(pack.y_GET() == -3.3570132E38F);
            assert(pack.roll_GET() == 2.4625407E38F);
            assert(pack.x_GET() == 7.135731E37F);
            assert(pack.pitch_GET() == -1.3124257E38F);
        });
        DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.yaw_SET(1.937978E38F) ;
        p104.x_SET(7.135731E37F) ;
        p104.y_SET(-3.3570132E38F) ;
        p104.roll_SET(2.4625407E38F) ;
        p104.z_SET(-1.7307009E38F) ;
        p104.pitch_SET(-1.3124257E38F) ;
        p104.usec_SET(7429537837165040229L) ;
        LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == -1.6085196E38F);
            assert(pack.zacc_GET() == -2.426524E38F);
            assert(pack.zmag_GET() == -2.5121744E38F);
            assert(pack.temperature_GET() == 1.3496968E38F);
            assert(pack.fields_updated_GET() == (char)26132);
            assert(pack.zgyro_GET() == -6.4764933E37F);
            assert(pack.diff_pressure_GET() == -1.4544338E38F);
            assert(pack.xmag_GET() == 1.3010247E38F);
            assert(pack.xacc_GET() == 1.3926914E38F);
            assert(pack.abs_pressure_GET() == 7.2168246E37F);
            assert(pack.time_usec_GET() == 6309291689645687552L);
            assert(pack.ygyro_GET() == -1.6569417E37F);
            assert(pack.pressure_alt_GET() == -7.2343496E37F);
            assert(pack.ymag_GET() == -3.2357126E38F);
            assert(pack.xgyro_GET() == -3.1677264E38F);
        });
        DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.fields_updated_SET((char)26132) ;
        p105.ygyro_SET(-1.6569417E37F) ;
        p105.yacc_SET(-1.6085196E38F) ;
        p105.zacc_SET(-2.426524E38F) ;
        p105.xmag_SET(1.3010247E38F) ;
        p105.ymag_SET(-3.2357126E38F) ;
        p105.zmag_SET(-2.5121744E38F) ;
        p105.abs_pressure_SET(7.2168246E37F) ;
        p105.temperature_SET(1.3496968E38F) ;
        p105.zgyro_SET(-6.4764933E37F) ;
        p105.xacc_SET(1.3926914E38F) ;
        p105.pressure_alt_SET(-7.2343496E37F) ;
        p105.time_usec_SET(6309291689645687552L) ;
        p105.xgyro_SET(-3.1677264E38F) ;
        p105.diff_pressure_SET(-1.4544338E38F) ;
        LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.integration_time_us_GET() == 1708206938L);
            assert(pack.quality_GET() == (char)23);
            assert(pack.integrated_y_GET() == 2.168419E38F);
            assert(pack.integrated_x_GET() == 2.4540381E38F);
            assert(pack.temperature_GET() == (short)27482);
            assert(pack.distance_GET() == -6.1752E37F);
            assert(pack.integrated_ygyro_GET() == -1.1458706E36F);
            assert(pack.sensor_id_GET() == (char)150);
            assert(pack.integrated_xgyro_GET() == -7.7557774E37F);
            assert(pack.integrated_zgyro_GET() == 1.1774011E38F);
            assert(pack.time_usec_GET() == 8846840844137867876L);
            assert(pack.time_delta_distance_us_GET() == 4077646362L);
        });
        DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.temperature_SET((short)27482) ;
        p106.integrated_zgyro_SET(1.1774011E38F) ;
        p106.integrated_xgyro_SET(-7.7557774E37F) ;
        p106.integrated_y_SET(2.168419E38F) ;
        p106.time_usec_SET(8846840844137867876L) ;
        p106.sensor_id_SET((char)150) ;
        p106.quality_SET((char)23) ;
        p106.time_delta_distance_us_SET(4077646362L) ;
        p106.integrated_x_SET(2.4540381E38F) ;
        p106.distance_SET(-6.1752E37F) ;
        p106.integrated_ygyro_SET(-1.1458706E36F) ;
        p106.integration_time_us_SET(1708206938L) ;
        LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == 2.5811987E38F);
            assert(pack.xmag_GET() == 5.796794E37F);
            assert(pack.zmag_GET() == -3.2756432E38F);
            assert(pack.time_usec_GET() == 2038134026821818961L);
            assert(pack.fields_updated_GET() == 4209462584L);
            assert(pack.temperature_GET() == -1.190089E38F);
            assert(pack.pressure_alt_GET() == -3.302512E38F);
            assert(pack.xgyro_GET() == 2.5592028E38F);
            assert(pack.zacc_GET() == 2.8727096E38F);
            assert(pack.ymag_GET() == -2.8327687E38F);
            assert(pack.ygyro_GET() == 2.4547616E38F);
            assert(pack.abs_pressure_GET() == -2.5850552E38F);
            assert(pack.diff_pressure_GET() == -2.886694E38F);
            assert(pack.yacc_GET() == -1.4228003E38F);
            assert(pack.zgyro_GET() == 5.963243E36F);
        });
        DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.xgyro_SET(2.5592028E38F) ;
        p107.ygyro_SET(2.4547616E38F) ;
        p107.pressure_alt_SET(-3.302512E38F) ;
        p107.fields_updated_SET(4209462584L) ;
        p107.temperature_SET(-1.190089E38F) ;
        p107.xmag_SET(5.796794E37F) ;
        p107.zgyro_SET(5.963243E36F) ;
        p107.abs_pressure_SET(-2.5850552E38F) ;
        p107.yacc_SET(-1.4228003E38F) ;
        p107.zacc_SET(2.8727096E38F) ;
        p107.time_usec_SET(2038134026821818961L) ;
        p107.ymag_SET(-2.8327687E38F) ;
        p107.diff_pressure_SET(-2.886694E38F) ;
        p107.zmag_SET(-3.2756432E38F) ;
        p107.xacc_SET(2.5811987E38F) ;
        LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.std_dev_vert_GET() == -2.5957152E38F);
            assert(pack.q3_GET() == -3.822362E37F);
            assert(pack.q4_GET() == -6.473326E37F);
            assert(pack.ygyro_GET() == -3.1769975E38F);
            assert(pack.ve_GET() == -2.3236593E38F);
            assert(pack.yacc_GET() == 1.8885603E38F);
            assert(pack.roll_GET() == -2.6685505E38F);
            assert(pack.yaw_GET() == 2.2087615E38F);
            assert(pack.std_dev_horz_GET() == 1.9352907E38F);
            assert(pack.lat_GET() == -2.1579437E38F);
            assert(pack.xgyro_GET() == 2.8465999E38F);
            assert(pack.q2_GET() == 1.5430967E38F);
            assert(pack.zgyro_GET() == 1.3813871E38F);
            assert(pack.alt_GET() == 3.2293774E38F);
            assert(pack.zacc_GET() == 1.6300122E38F);
            assert(pack.q1_GET() == -7.538609E37F);
            assert(pack.vd_GET() == 3.2847725E38F);
            assert(pack.pitch_GET() == -3.3843052E38F);
            assert(pack.lon_GET() == 2.2925104E38F);
            assert(pack.xacc_GET() == -1.2116112E37F);
            assert(pack.vn_GET() == 3.4077455E37F);
        });
        DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.vn_SET(3.4077455E37F) ;
        p108.std_dev_vert_SET(-2.5957152E38F) ;
        p108.yacc_SET(1.8885603E38F) ;
        p108.q3_SET(-3.822362E37F) ;
        p108.ve_SET(-2.3236593E38F) ;
        p108.xacc_SET(-1.2116112E37F) ;
        p108.vd_SET(3.2847725E38F) ;
        p108.yaw_SET(2.2087615E38F) ;
        p108.alt_SET(3.2293774E38F) ;
        p108.lat_SET(-2.1579437E38F) ;
        p108.ygyro_SET(-3.1769975E38F) ;
        p108.pitch_SET(-3.3843052E38F) ;
        p108.lon_SET(2.2925104E38F) ;
        p108.q1_SET(-7.538609E37F) ;
        p108.zgyro_SET(1.3813871E38F) ;
        p108.xgyro_SET(2.8465999E38F) ;
        p108.zacc_SET(1.6300122E38F) ;
        p108.roll_SET(-2.6685505E38F) ;
        p108.q2_SET(1.5430967E38F) ;
        p108.q4_SET(-6.473326E37F) ;
        p108.std_dev_horz_SET(1.9352907E38F) ;
        LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.fixed__GET() == (char)42511);
            assert(pack.remrssi_GET() == (char)100);
            assert(pack.remnoise_GET() == (char)108);
            assert(pack.rxerrors_GET() == (char)51053);
            assert(pack.txbuf_GET() == (char)140);
            assert(pack.noise_GET() == (char)196);
            assert(pack.rssi_GET() == (char)222);
        });
        DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.fixed__SET((char)42511) ;
        p109.remnoise_SET((char)108) ;
        p109.rssi_SET((char)222) ;
        p109.remrssi_SET((char)100) ;
        p109.noise_SET((char)196) ;
        p109.rxerrors_SET((char)51053) ;
        p109.txbuf_SET((char)140) ;
        LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)243, (char)8, (char)74, (char)79, (char)123, (char)110, (char)181, (char)162, (char)231, (char)108, (char)151, (char)4, (char)234, (char)97, (char)47, (char)124, (char)230, (char)154, (char)4, (char)136, (char)242, (char)216, (char)126, (char)142, (char)1, (char)175, (char)1, (char)157, (char)140, (char)86, (char)188, (char)226, (char)57, (char)12, (char)209, (char)224, (char)93, (char)22, (char)118, (char)178, (char)248, (char)225, (char)128, (char)116, (char)101, (char)212, (char)72, (char)102, (char)242, (char)165, (char)253, (char)251, (char)114, (char)198, (char)124, (char)46, (char)163, (char)103, (char)42, (char)181, (char)190, (char)243, (char)56, (char)39, (char)50, (char)216, (char)125, (char)173, (char)93, (char)121, (char)229, (char)89, (char)94, (char)104, (char)122, (char)223, (char)31, (char)28, (char)107, (char)180, (char)159, (char)140, (char)50, (char)100, (char)119, (char)80, (char)80, (char)180, (char)105, (char)128, (char)25, (char)188, (char)191, (char)147, (char)18, (char)61, (char)89, (char)240, (char)168, (char)77, (char)32, (char)240, (char)203, (char)50, (char)229, (char)229, (char)131, (char)0, (char)32, (char)119, (char)143, (char)56, (char)64, (char)84, (char)75, (char)100, (char)233, (char)139, (char)30, (char)199, (char)210, (char)43, (char)182, (char)195, (char)216, (char)116, (char)82, (char)76, (char)168, (char)194, (char)185, (char)94, (char)147, (char)219, (char)214, (char)145, (char)51, (char)67, (char)117, (char)132, (char)181, (char)49, (char)43, (char)174, (char)197, (char)241, (char)15, (char)183, (char)183, (char)164, (char)236, (char)155, (char)105, (char)248, (char)183, (char)145, (char)181, (char)235, (char)216, (char)64, (char)106, (char)46, (char)70, (char)31, (char)123, (char)232, (char)120, (char)1, (char)181, (char)4, (char)245, (char)37, (char)108, (char)253, (char)138, (char)235, (char)21, (char)12, (char)31, (char)67, (char)66, (char)239, (char)106, (char)89, (char)8, (char)138, (char)176, (char)245, (char)26, (char)103, (char)41, (char)70, (char)101, (char)137, (char)78, (char)26, (char)122, (char)119, (char)94, (char)100, (char)57, (char)207, (char)201, (char)151, (char)101, (char)232, (char)207, (char)67, (char)14, (char)55, (char)78, (char)247, (char)181, (char)177, (char)57, (char)40, (char)166, (char)155, (char)63, (char)131, (char)102, (char)56, (char)152, (char)23, (char)157, (char)95, (char)242, (char)85, (char)9, (char)161, (char)105, (char)143, (char)230, (char)240, (char)192, (char)131, (char)191, (char)169, (char)161, (char)89, (char)42, (char)202, (char)113, (char)132, (char)48, (char)10, (char)12, (char)174, (char)127, (char)135, (char)243}));
            assert(pack.target_system_GET() == (char)146);
            assert(pack.target_component_GET() == (char)222);
            assert(pack.target_network_GET() == (char)104);
        });
        DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.payload_SET(new char[] {(char)243, (char)8, (char)74, (char)79, (char)123, (char)110, (char)181, (char)162, (char)231, (char)108, (char)151, (char)4, (char)234, (char)97, (char)47, (char)124, (char)230, (char)154, (char)4, (char)136, (char)242, (char)216, (char)126, (char)142, (char)1, (char)175, (char)1, (char)157, (char)140, (char)86, (char)188, (char)226, (char)57, (char)12, (char)209, (char)224, (char)93, (char)22, (char)118, (char)178, (char)248, (char)225, (char)128, (char)116, (char)101, (char)212, (char)72, (char)102, (char)242, (char)165, (char)253, (char)251, (char)114, (char)198, (char)124, (char)46, (char)163, (char)103, (char)42, (char)181, (char)190, (char)243, (char)56, (char)39, (char)50, (char)216, (char)125, (char)173, (char)93, (char)121, (char)229, (char)89, (char)94, (char)104, (char)122, (char)223, (char)31, (char)28, (char)107, (char)180, (char)159, (char)140, (char)50, (char)100, (char)119, (char)80, (char)80, (char)180, (char)105, (char)128, (char)25, (char)188, (char)191, (char)147, (char)18, (char)61, (char)89, (char)240, (char)168, (char)77, (char)32, (char)240, (char)203, (char)50, (char)229, (char)229, (char)131, (char)0, (char)32, (char)119, (char)143, (char)56, (char)64, (char)84, (char)75, (char)100, (char)233, (char)139, (char)30, (char)199, (char)210, (char)43, (char)182, (char)195, (char)216, (char)116, (char)82, (char)76, (char)168, (char)194, (char)185, (char)94, (char)147, (char)219, (char)214, (char)145, (char)51, (char)67, (char)117, (char)132, (char)181, (char)49, (char)43, (char)174, (char)197, (char)241, (char)15, (char)183, (char)183, (char)164, (char)236, (char)155, (char)105, (char)248, (char)183, (char)145, (char)181, (char)235, (char)216, (char)64, (char)106, (char)46, (char)70, (char)31, (char)123, (char)232, (char)120, (char)1, (char)181, (char)4, (char)245, (char)37, (char)108, (char)253, (char)138, (char)235, (char)21, (char)12, (char)31, (char)67, (char)66, (char)239, (char)106, (char)89, (char)8, (char)138, (char)176, (char)245, (char)26, (char)103, (char)41, (char)70, (char)101, (char)137, (char)78, (char)26, (char)122, (char)119, (char)94, (char)100, (char)57, (char)207, (char)201, (char)151, (char)101, (char)232, (char)207, (char)67, (char)14, (char)55, (char)78, (char)247, (char)181, (char)177, (char)57, (char)40, (char)166, (char)155, (char)63, (char)131, (char)102, (char)56, (char)152, (char)23, (char)157, (char)95, (char)242, (char)85, (char)9, (char)161, (char)105, (char)143, (char)230, (char)240, (char)192, (char)131, (char)191, (char)169, (char)161, (char)89, (char)42, (char)202, (char)113, (char)132, (char)48, (char)10, (char)12, (char)174, (char)127, (char)135, (char)243}, 0) ;
        p110.target_system_SET((char)146) ;
        p110.target_component_SET((char)222) ;
        p110.target_network_SET((char)104) ;
        LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.tc1_GET() == -2732927443723377252L);
            assert(pack.ts1_GET() == -7304150227805712633L);
        });
        DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.ts1_SET(-7304150227805712633L) ;
        p111.tc1_SET(-2732927443723377252L) ;
        LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 1468618473L);
            assert(pack.time_usec_GET() == 1915259244566929012L);
        });
        DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.seq_SET(1468618473L) ;
        p112.time_usec_SET(1915259244566929012L) ;
        LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.vn_GET() == (short)4856);
            assert(pack.fix_type_GET() == (char)101);
            assert(pack.cog_GET() == (char)61699);
            assert(pack.lon_GET() == -1241696547);
            assert(pack.lat_GET() == 1844009517);
            assert(pack.alt_GET() == -329457214);
            assert(pack.satellites_visible_GET() == (char)60);
            assert(pack.eph_GET() == (char)54380);
            assert(pack.time_usec_GET() == 5812481061809013598L);
            assert(pack.vd_GET() == (short) -8490);
            assert(pack.epv_GET() == (char)15661);
            assert(pack.ve_GET() == (short)7954);
            assert(pack.vel_GET() == (char)49834);
        });
        DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.lat_SET(1844009517) ;
        p113.vd_SET((short) -8490) ;
        p113.eph_SET((char)54380) ;
        p113.alt_SET(-329457214) ;
        p113.fix_type_SET((char)101) ;
        p113.satellites_visible_SET((char)60) ;
        p113.ve_SET((short)7954) ;
        p113.time_usec_SET(5812481061809013598L) ;
        p113.vel_SET((char)49834) ;
        p113.lon_SET(-1241696547) ;
        p113.vn_SET((short)4856) ;
        p113.cog_SET((char)61699) ;
        p113.epv_SET((char)15661) ;
        LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integrated_x_GET() == -3.1028138E38F);
            assert(pack.integrated_ygyro_GET() == 2.6632723E38F);
            assert(pack.distance_GET() == 3.226791E38F);
            assert(pack.sensor_id_GET() == (char)66);
            assert(pack.integrated_xgyro_GET() == 1.1910841E38F);
            assert(pack.integration_time_us_GET() == 610665585L);
            assert(pack.time_delta_distance_us_GET() == 1092060183L);
            assert(pack.integrated_y_GET() == 2.2318108E38F);
            assert(pack.integrated_zgyro_GET() == -3.1797332E38F);
            assert(pack.temperature_GET() == (short)19840);
            assert(pack.time_usec_GET() == 1798586136375374771L);
            assert(pack.quality_GET() == (char)101);
        });
        DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.time_usec_SET(1798586136375374771L) ;
        p114.integrated_xgyro_SET(1.1910841E38F) ;
        p114.sensor_id_SET((char)66) ;
        p114.integrated_y_SET(2.2318108E38F) ;
        p114.integrated_x_SET(-3.1028138E38F) ;
        p114.quality_SET((char)101) ;
        p114.integration_time_us_SET(610665585L) ;
        p114.time_delta_distance_us_SET(1092060183L) ;
        p114.integrated_ygyro_SET(2.6632723E38F) ;
        p114.distance_SET(3.226791E38F) ;
        p114.temperature_SET((short)19840) ;
        p114.integrated_zgyro_SET(-3.1797332E38F) ;
        LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == -3.2318876E38F);
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {2.738803E38F, 6.0495036E35F, -5.511696E37F, -4.208945E36F}));
            assert(pack.lon_GET() == 1216914970);
            assert(pack.pitchspeed_GET() == -2.7449603E38F);
            assert(pack.vz_GET() == (short)22976);
            assert(pack.ind_airspeed_GET() == (char)3823);
            assert(pack.xacc_GET() == (short)11473);
            assert(pack.time_usec_GET() == 7333925203737805407L);
            assert(pack.alt_GET() == 861920798);
            assert(pack.yacc_GET() == (short) -25643);
            assert(pack.lat_GET() == 112034423);
            assert(pack.vy_GET() == (short)31760);
            assert(pack.zacc_GET() == (short)5143);
            assert(pack.vx_GET() == (short) -1856);
            assert(pack.yawspeed_GET() == 7.164862E36F);
            assert(pack.true_airspeed_GET() == (char)18583);
        });
        DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.time_usec_SET(7333925203737805407L) ;
        p115.vz_SET((short)22976) ;
        p115.true_airspeed_SET((char)18583) ;
        p115.xacc_SET((short)11473) ;
        p115.vy_SET((short)31760) ;
        p115.yacc_SET((short) -25643) ;
        p115.ind_airspeed_SET((char)3823) ;
        p115.lon_SET(1216914970) ;
        p115.vx_SET((short) -1856) ;
        p115.pitchspeed_SET(-2.7449603E38F) ;
        p115.alt_SET(861920798) ;
        p115.yawspeed_SET(7.164862E36F) ;
        p115.zacc_SET((short)5143) ;
        p115.rollspeed_SET(-3.2318876E38F) ;
        p115.attitude_quaternion_SET(new float[] {2.738803E38F, 6.0495036E35F, -5.511696E37F, -4.208945E36F}, 0) ;
        p115.lat_SET(112034423) ;
        LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short)11540);
            assert(pack.xgyro_GET() == (short)15835);
            assert(pack.zgyro_GET() == (short) -26941);
            assert(pack.ymag_GET() == (short) -12610);
            assert(pack.zacc_GET() == (short) -25016);
            assert(pack.xacc_GET() == (short)12117);
            assert(pack.zmag_GET() == (short) -2379);
            assert(pack.time_boot_ms_GET() == 1385754972L);
            assert(pack.xmag_GET() == (short) -30746);
            assert(pack.yacc_GET() == (short)26218);
        });
        DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.xgyro_SET((short)15835) ;
        p116.xacc_SET((short)12117) ;
        p116.zgyro_SET((short) -26941) ;
        p116.xmag_SET((short) -30746) ;
        p116.zacc_SET((short) -25016) ;
        p116.yacc_SET((short)26218) ;
        p116.zmag_SET((short) -2379) ;
        p116.time_boot_ms_SET(1385754972L) ;
        p116.ygyro_SET((short)11540) ;
        p116.ymag_SET((short) -12610) ;
        LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_GET() == (char)56960);
            assert(pack.start_GET() == (char)60076);
            assert(pack.target_component_GET() == (char)216);
            assert(pack.target_system_GET() == (char)133);
        });
        DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.target_component_SET((char)216) ;
        p117.end_SET((char)56960) ;
        p117.target_system_SET((char)133) ;
        p117.start_SET((char)60076) ;
        LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == (char)49351);
            assert(pack.num_logs_GET() == (char)27918);
            assert(pack.time_utc_GET() == 851305146L);
            assert(pack.last_log_num_GET() == (char)35742);
            assert(pack.size_GET() == 1914113081L);
        });
        DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)49351) ;
        p118.last_log_num_SET((char)35742) ;
        p118.num_logs_SET((char)27918) ;
        p118.size_SET(1914113081L) ;
        p118.time_utc_SET(851305146L) ;
        LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == 44360461L);
            assert(pack.ofs_GET() == 2725354911L);
            assert(pack.target_component_GET() == (char)126);
            assert(pack.target_system_GET() == (char)12);
            assert(pack.id_GET() == (char)37631);
        });
        DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)12) ;
        p119.target_component_SET((char)126) ;
        p119.ofs_SET(2725354911L) ;
        p119.id_SET((char)37631) ;
        p119.count_SET(44360461L) ;
        LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)217, (char)190, (char)207, (char)98, (char)57, (char)200, (char)16, (char)143, (char)135, (char)122, (char)49, (char)53, (char)68, (char)126, (char)31, (char)150, (char)161, (char)243, (char)148, (char)137, (char)250, (char)250, (char)128, (char)4, (char)44, (char)152, (char)11, (char)11, (char)204, (char)12, (char)191, (char)77, (char)156, (char)93, (char)219, (char)153, (char)149, (char)157, (char)15, (char)32, (char)181, (char)130, (char)70, (char)130, (char)161, (char)102, (char)218, (char)189, (char)24, (char)205, (char)50, (char)130, (char)110, (char)177, (char)46, (char)115, (char)172, (char)247, (char)222, (char)122, (char)254, (char)148, (char)105, (char)91, (char)223, (char)6, (char)116, (char)207, (char)158, (char)95, (char)150, (char)52, (char)244, (char)221, (char)79, (char)160, (char)250, (char)32, (char)227, (char)193, (char)229, (char)20, (char)80, (char)51, (char)238, (char)246, (char)129, (char)235, (char)221, (char)177}));
            assert(pack.ofs_GET() == 2173808933L);
            assert(pack.id_GET() == (char)8567);
            assert(pack.count_GET() == (char)96);
        });
        DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.ofs_SET(2173808933L) ;
        p120.data__SET(new char[] {(char)217, (char)190, (char)207, (char)98, (char)57, (char)200, (char)16, (char)143, (char)135, (char)122, (char)49, (char)53, (char)68, (char)126, (char)31, (char)150, (char)161, (char)243, (char)148, (char)137, (char)250, (char)250, (char)128, (char)4, (char)44, (char)152, (char)11, (char)11, (char)204, (char)12, (char)191, (char)77, (char)156, (char)93, (char)219, (char)153, (char)149, (char)157, (char)15, (char)32, (char)181, (char)130, (char)70, (char)130, (char)161, (char)102, (char)218, (char)189, (char)24, (char)205, (char)50, (char)130, (char)110, (char)177, (char)46, (char)115, (char)172, (char)247, (char)222, (char)122, (char)254, (char)148, (char)105, (char)91, (char)223, (char)6, (char)116, (char)207, (char)158, (char)95, (char)150, (char)52, (char)244, (char)221, (char)79, (char)160, (char)250, (char)32, (char)227, (char)193, (char)229, (char)20, (char)80, (char)51, (char)238, (char)246, (char)129, (char)235, (char)221, (char)177}, 0) ;
        p120.id_SET((char)8567) ;
        p120.count_SET((char)96) ;
        LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)165);
            assert(pack.target_system_GET() == (char)84);
        });
        DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_component_SET((char)165) ;
        p121.target_system_SET((char)84) ;
        LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)114);
            assert(pack.target_component_GET() == (char)97);
        });
        DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_component_SET((char)97) ;
        p122.target_system_SET((char)114) ;
        LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)118, (char)102, (char)234, (char)165, (char)13, (char)145, (char)135, (char)115, (char)19, (char)216, (char)152, (char)181, (char)71, (char)84, (char)43, (char)85, (char)201, (char)94, (char)84, (char)228, (char)239, (char)43, (char)54, (char)249, (char)109, (char)174, (char)148, (char)179, (char)3, (char)19, (char)39, (char)98, (char)81, (char)55, (char)130, (char)237, (char)71, (char)79, (char)127, (char)129, (char)220, (char)32, (char)175, (char)95, (char)145, (char)7, (char)202, (char)208, (char)183, (char)251, (char)70, (char)1, (char)40, (char)226, (char)243, (char)5, (char)140, (char)101, (char)45, (char)163, (char)0, (char)225, (char)29, (char)100, (char)126, (char)62, (char)156, (char)23, (char)23, (char)173, (char)175, (char)63, (char)106, (char)185, (char)65, (char)174, (char)229, (char)9, (char)215, (char)201, (char)168, (char)57, (char)241, (char)18, (char)196, (char)252, (char)217, (char)228, (char)216, (char)30, (char)44, (char)113, (char)187, (char)254, (char)135, (char)231, (char)166, (char)171, (char)81, (char)169, (char)180, (char)217, (char)139, (char)18, (char)233, (char)211, (char)232, (char)55, (char)206, (char)252}));
            assert(pack.target_component_GET() == (char)75);
            assert(pack.len_GET() == (char)69);
            assert(pack.target_system_GET() == (char)51);
        });
        DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.len_SET((char)69) ;
        p123.target_component_SET((char)75) ;
        p123.data__SET(new char[] {(char)118, (char)102, (char)234, (char)165, (char)13, (char)145, (char)135, (char)115, (char)19, (char)216, (char)152, (char)181, (char)71, (char)84, (char)43, (char)85, (char)201, (char)94, (char)84, (char)228, (char)239, (char)43, (char)54, (char)249, (char)109, (char)174, (char)148, (char)179, (char)3, (char)19, (char)39, (char)98, (char)81, (char)55, (char)130, (char)237, (char)71, (char)79, (char)127, (char)129, (char)220, (char)32, (char)175, (char)95, (char)145, (char)7, (char)202, (char)208, (char)183, (char)251, (char)70, (char)1, (char)40, (char)226, (char)243, (char)5, (char)140, (char)101, (char)45, (char)163, (char)0, (char)225, (char)29, (char)100, (char)126, (char)62, (char)156, (char)23, (char)23, (char)173, (char)175, (char)63, (char)106, (char)185, (char)65, (char)174, (char)229, (char)9, (char)215, (char)201, (char)168, (char)57, (char)241, (char)18, (char)196, (char)252, (char)217, (char)228, (char)216, (char)30, (char)44, (char)113, (char)187, (char)254, (char)135, (char)231, (char)166, (char)171, (char)81, (char)169, (char)180, (char)217, (char)139, (char)18, (char)233, (char)211, (char)232, (char)55, (char)206, (char)252}, 0) ;
        p123.target_system_SET((char)51) ;
        LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.cog_GET() == (char)35265);
            assert(pack.lon_GET() == 1403280176);
            assert(pack.time_usec_GET() == 8193300585817214260L);
            assert(pack.dgps_age_GET() == 3651367518L);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
            assert(pack.epv_GET() == (char)30198);
            assert(pack.alt_GET() == -1042091888);
            assert(pack.dgps_numch_GET() == (char)139);
            assert(pack.satellites_visible_GET() == (char)103);
            assert(pack.lat_GET() == -1972636178);
            assert(pack.vel_GET() == (char)23493);
            assert(pack.eph_GET() == (char)22966);
        });
        DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX) ;
        p124.cog_SET((char)35265) ;
        p124.dgps_numch_SET((char)139) ;
        p124.epv_SET((char)30198) ;
        p124.lon_SET(1403280176) ;
        p124.vel_SET((char)23493) ;
        p124.time_usec_SET(8193300585817214260L) ;
        p124.satellites_visible_SET((char)103) ;
        p124.dgps_age_SET(3651367518L) ;
        p124.eph_SET((char)22966) ;
        p124.alt_SET(-1042091888) ;
        p124.lat_SET(-1972636178) ;
        LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vservo_GET() == (char)62883);
            assert(pack.Vcc_GET() == (char)29261);
            assert(pack.flags_GET() == MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID);
        });
        DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vservo_SET((char)62883) ;
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID) ;
        p125.Vcc_SET((char)29261) ;
        LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.baudrate_GET() == 2138291633L);
            assert(pack.count_GET() == (char)137);
            assert(pack.flags_GET() == SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY);
            assert(pack.timeout_GET() == (char)50304);
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)172, (char)16, (char)83, (char)93, (char)139, (char)238, (char)49, (char)180, (char)173, (char)114, (char)71, (char)166, (char)231, (char)25, (char)74, (char)131, (char)151, (char)226, (char)9, (char)240, (char)207, (char)51, (char)173, (char)228, (char)16, (char)209, (char)180, (char)173, (char)154, (char)173, (char)87, (char)16, (char)212, (char)180, (char)21, (char)78, (char)143, (char)144, (char)200, (char)144, (char)54, (char)37, (char)65, (char)87, (char)58, (char)208, (char)43, (char)113, (char)38, (char)71, (char)97, (char)238, (char)211, (char)183, (char)217, (char)66, (char)115, (char)48, (char)41, (char)252, (char)43, (char)90, (char)110, (char)155, (char)14, (char)226, (char)41, (char)185, (char)200, (char)123}));
        });
        DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.count_SET((char)137) ;
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_REPLY) ;
        p126.baudrate_SET(2138291633L) ;
        p126.data__SET(new char[] {(char)172, (char)16, (char)83, (char)93, (char)139, (char)238, (char)49, (char)180, (char)173, (char)114, (char)71, (char)166, (char)231, (char)25, (char)74, (char)131, (char)151, (char)226, (char)9, (char)240, (char)207, (char)51, (char)173, (char)228, (char)16, (char)209, (char)180, (char)173, (char)154, (char)173, (char)87, (char)16, (char)212, (char)180, (char)21, (char)78, (char)143, (char)144, (char)200, (char)144, (char)54, (char)37, (char)65, (char)87, (char)58, (char)208, (char)43, (char)113, (char)38, (char)71, (char)97, (char)238, (char)211, (char)183, (char)217, (char)66, (char)115, (char)48, (char)41, (char)252, (char)43, (char)90, (char)110, (char)155, (char)14, (char)226, (char)41, (char)185, (char)200, (char)123}, 0) ;
        p126.timeout_SET((char)50304) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_TELEM1) ;
        LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.rtk_receiver_id_GET() == (char)241);
            assert(pack.tow_GET() == 551343228L);
            assert(pack.baseline_a_mm_GET() == 1060249534);
            assert(pack.baseline_coords_type_GET() == (char)41);
            assert(pack.time_last_baseline_ms_GET() == 2711834231L);
            assert(pack.baseline_c_mm_GET() == -1560325607);
            assert(pack.nsats_GET() == (char)120);
            assert(pack.wn_GET() == (char)14388);
            assert(pack.rtk_health_GET() == (char)69);
            assert(pack.accuracy_GET() == 638158692L);
            assert(pack.baseline_b_mm_GET() == 2023118566);
            assert(pack.rtk_rate_GET() == (char)160);
            assert(pack.iar_num_hypotheses_GET() == 1380322870);
        });
        DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.iar_num_hypotheses_SET(1380322870) ;
        p127.baseline_a_mm_SET(1060249534) ;
        p127.baseline_b_mm_SET(2023118566) ;
        p127.accuracy_SET(638158692L) ;
        p127.wn_SET((char)14388) ;
        p127.rtk_health_SET((char)69) ;
        p127.rtk_receiver_id_SET((char)241) ;
        p127.nsats_SET((char)120) ;
        p127.time_last_baseline_ms_SET(2711834231L) ;
        p127.tow_SET(551343228L) ;
        p127.baseline_c_mm_SET(-1560325607) ;
        p127.baseline_coords_type_SET((char)41) ;
        p127.rtk_rate_SET((char)160) ;
        LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.tow_GET() == 1549002301L);
            assert(pack.rtk_receiver_id_GET() == (char)168);
            assert(pack.rtk_health_GET() == (char)122);
            assert(pack.nsats_GET() == (char)44);
            assert(pack.wn_GET() == (char)22992);
            assert(pack.accuracy_GET() == 87873402L);
            assert(pack.rtk_rate_GET() == (char)87);
            assert(pack.baseline_b_mm_GET() == -658277418);
            assert(pack.baseline_c_mm_GET() == 1628775737);
            assert(pack.baseline_coords_type_GET() == (char)244);
            assert(pack.baseline_a_mm_GET() == -1271441761);
            assert(pack.time_last_baseline_ms_GET() == 1348881990L);
            assert(pack.iar_num_hypotheses_GET() == -1578291244);
        });
        DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.tow_SET(1549002301L) ;
        p128.time_last_baseline_ms_SET(1348881990L) ;
        p128.iar_num_hypotheses_SET(-1578291244) ;
        p128.rtk_receiver_id_SET((char)168) ;
        p128.rtk_rate_SET((char)87) ;
        p128.wn_SET((char)22992) ;
        p128.nsats_SET((char)44) ;
        p128.accuracy_SET(87873402L) ;
        p128.baseline_b_mm_SET(-658277418) ;
        p128.baseline_a_mm_SET(-1271441761) ;
        p128.rtk_health_SET((char)122) ;
        p128.baseline_coords_type_SET((char)244) ;
        p128.baseline_c_mm_SET(1628775737) ;
        LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short)18208);
            assert(pack.zacc_GET() == (short)15699);
            assert(pack.zmag_GET() == (short) -24236);
            assert(pack.zgyro_GET() == (short) -12129);
            assert(pack.xgyro_GET() == (short) -26287);
            assert(pack.xmag_GET() == (short) -20080);
            assert(pack.xacc_GET() == (short) -6518);
            assert(pack.time_boot_ms_GET() == 2054688546L);
            assert(pack.ymag_GET() == (short) -18258);
            assert(pack.yacc_GET() == (short)23725);
        });
        DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.zgyro_SET((short) -12129) ;
        p129.zmag_SET((short) -24236) ;
        p129.ymag_SET((short) -18258) ;
        p129.xmag_SET((short) -20080) ;
        p129.time_boot_ms_SET(2054688546L) ;
        p129.xacc_SET((short) -6518) ;
        p129.zacc_SET((short)15699) ;
        p129.yacc_SET((short)23725) ;
        p129.ygyro_SET((short)18208) ;
        p129.xgyro_SET((short) -26287) ;
        LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.height_GET() == (char)57145);
            assert(pack.jpg_quality_GET() == (char)12);
            assert(pack.type_GET() == (char)122);
            assert(pack.width_GET() == (char)59532);
            assert(pack.payload_GET() == (char)10);
            assert(pack.packets_GET() == (char)17467);
            assert(pack.size_GET() == 3647216503L);
        });
        DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.size_SET(3647216503L) ;
        p130.type_SET((char)122) ;
        p130.width_SET((char)59532) ;
        p130.packets_SET((char)17467) ;
        p130.height_SET((char)57145) ;
        p130.jpg_quality_SET((char)12) ;
        p130.payload_SET((char)10) ;
        LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)55493);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)23, (char)61, (char)175, (char)7, (char)101, (char)217, (char)211, (char)127, (char)122, (char)104, (char)101, (char)195, (char)59, (char)245, (char)84, (char)219, (char)69, (char)170, (char)160, (char)144, (char)153, (char)252, (char)251, (char)218, (char)177, (char)50, (char)205, (char)97, (char)44, (char)58, (char)34, (char)62, (char)106, (char)189, (char)74, (char)110, (char)36, (char)174, (char)100, (char)24, (char)32, (char)87, (char)32, (char)106, (char)136, (char)238, (char)157, (char)201, (char)52, (char)158, (char)169, (char)6, (char)89, (char)166, (char)59, (char)97, (char)52, (char)186, (char)97, (char)223, (char)26, (char)231, (char)163, (char)112, (char)214, (char)47, (char)126, (char)72, (char)99, (char)55, (char)134, (char)240, (char)225, (char)184, (char)152, (char)155, (char)83, (char)139, (char)179, (char)190, (char)143, (char)62, (char)97, (char)206, (char)58, (char)153, (char)186, (char)205, (char)200, (char)44, (char)24, (char)156, (char)178, (char)72, (char)169, (char)83, (char)47, (char)252, (char)202, (char)95, (char)150, (char)60, (char)236, (char)121, (char)185, (char)183, (char)52, (char)37, (char)155, (char)230, (char)66, (char)26, (char)250, (char)202, (char)209, (char)198, (char)109, (char)70, (char)231, (char)114, (char)29, (char)73, (char)155, (char)157, (char)203, (char)82, (char)1, (char)124, (char)49, (char)154, (char)50, (char)91, (char)95, (char)52, (char)137, (char)8, (char)117, (char)173, (char)90, (char)176, (char)52, (char)84, (char)36, (char)82, (char)80, (char)132, (char)100, (char)80, (char)244, (char)128, (char)38, (char)232, (char)32, (char)56, (char)30, (char)77, (char)208, (char)144, (char)198, (char)4, (char)100, (char)112, (char)168, (char)219, (char)9, (char)141, (char)204, (char)92, (char)241, (char)109, (char)229, (char)6, (char)202, (char)24, (char)89, (char)95, (char)252, (char)172, (char)28, (char)156, (char)122, (char)65, (char)158, (char)188, (char)205, (char)160, (char)25, (char)125, (char)47, (char)30, (char)203, (char)203, (char)196, (char)88, (char)76, (char)57, (char)45, (char)22, (char)75, (char)210, (char)67, (char)11, (char)155, (char)175, (char)231, (char)189, (char)16, (char)181, (char)144, (char)2, (char)222, (char)109, (char)59, (char)135, (char)210, (char)94, (char)165, (char)74, (char)193, (char)80, (char)183, (char)213, (char)79, (char)181, (char)80, (char)187, (char)172, (char)244, (char)164, (char)204, (char)50, (char)87, (char)76, (char)159, (char)81, (char)63, (char)35, (char)106, (char)104, (char)0, (char)202, (char)174, (char)229, (char)42, (char)178, (char)43, (char)85, (char)110, (char)43, (char)138, (char)101, (char)18, (char)79}));
        });
        DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.seqnr_SET((char)55493) ;
        p131.data__SET(new char[] {(char)23, (char)61, (char)175, (char)7, (char)101, (char)217, (char)211, (char)127, (char)122, (char)104, (char)101, (char)195, (char)59, (char)245, (char)84, (char)219, (char)69, (char)170, (char)160, (char)144, (char)153, (char)252, (char)251, (char)218, (char)177, (char)50, (char)205, (char)97, (char)44, (char)58, (char)34, (char)62, (char)106, (char)189, (char)74, (char)110, (char)36, (char)174, (char)100, (char)24, (char)32, (char)87, (char)32, (char)106, (char)136, (char)238, (char)157, (char)201, (char)52, (char)158, (char)169, (char)6, (char)89, (char)166, (char)59, (char)97, (char)52, (char)186, (char)97, (char)223, (char)26, (char)231, (char)163, (char)112, (char)214, (char)47, (char)126, (char)72, (char)99, (char)55, (char)134, (char)240, (char)225, (char)184, (char)152, (char)155, (char)83, (char)139, (char)179, (char)190, (char)143, (char)62, (char)97, (char)206, (char)58, (char)153, (char)186, (char)205, (char)200, (char)44, (char)24, (char)156, (char)178, (char)72, (char)169, (char)83, (char)47, (char)252, (char)202, (char)95, (char)150, (char)60, (char)236, (char)121, (char)185, (char)183, (char)52, (char)37, (char)155, (char)230, (char)66, (char)26, (char)250, (char)202, (char)209, (char)198, (char)109, (char)70, (char)231, (char)114, (char)29, (char)73, (char)155, (char)157, (char)203, (char)82, (char)1, (char)124, (char)49, (char)154, (char)50, (char)91, (char)95, (char)52, (char)137, (char)8, (char)117, (char)173, (char)90, (char)176, (char)52, (char)84, (char)36, (char)82, (char)80, (char)132, (char)100, (char)80, (char)244, (char)128, (char)38, (char)232, (char)32, (char)56, (char)30, (char)77, (char)208, (char)144, (char)198, (char)4, (char)100, (char)112, (char)168, (char)219, (char)9, (char)141, (char)204, (char)92, (char)241, (char)109, (char)229, (char)6, (char)202, (char)24, (char)89, (char)95, (char)252, (char)172, (char)28, (char)156, (char)122, (char)65, (char)158, (char)188, (char)205, (char)160, (char)25, (char)125, (char)47, (char)30, (char)203, (char)203, (char)196, (char)88, (char)76, (char)57, (char)45, (char)22, (char)75, (char)210, (char)67, (char)11, (char)155, (char)175, (char)231, (char)189, (char)16, (char)181, (char)144, (char)2, (char)222, (char)109, (char)59, (char)135, (char)210, (char)94, (char)165, (char)74, (char)193, (char)80, (char)183, (char)213, (char)79, (char)181, (char)80, (char)187, (char)172, (char)244, (char)164, (char)204, (char)50, (char)87, (char)76, (char)159, (char)81, (char)63, (char)35, (char)106, (char)104, (char)0, (char)202, (char)174, (char)229, (char)42, (char)178, (char)43, (char)85, (char)110, (char)43, (char)138, (char)101, (char)18, (char)79}, 0) ;
        LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.max_distance_GET() == (char)34525);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_90);
            assert(pack.min_distance_GET() == (char)63125);
            assert(pack.covariance_GET() == (char)243);
            assert(pack.id_GET() == (char)72);
            assert(pack.time_boot_ms_GET() == 1909286717L);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER);
            assert(pack.current_distance_GET() == (char)46202);
        });
        DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_90) ;
        p132.min_distance_SET((char)63125) ;
        p132.time_boot_ms_SET(1909286717L) ;
        p132.covariance_SET((char)243) ;
        p132.current_distance_SET((char)46202) ;
        p132.id_SET((char)72) ;
        p132.max_distance_SET((char)34525) ;
        LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.grid_spacing_GET() == (char)11893);
            assert(pack.lon_GET() == 1812784320);
            assert(pack.lat_GET() == -684484357);
            assert(pack.mask_GET() == 1121196357219748499L);
        });
        DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.mask_SET(1121196357219748499L) ;
        p133.lat_SET(-684484357) ;
        p133.lon_SET(1812784320) ;
        p133.grid_spacing_SET((char)11893) ;
        LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.grid_spacing_GET() == (char)21319);
            assert(pack.lon_GET() == -1943705705);
            assert(pack.gridbit_GET() == (char)136);
            assert(pack.lat_GET() == -1380906918);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short)25285, (short)32057, (short) -27109, (short) -28178, (short)21812, (short) -16963, (short)3782, (short)20738, (short) -21104, (short)10411, (short) -19666, (short)2946, (short) -18952, (short)31362, (short) -21617, (short) -27013}));
        });
        DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.lon_SET(-1943705705) ;
        p134.gridbit_SET((char)136) ;
        p134.grid_spacing_SET((char)21319) ;
        p134.lat_SET(-1380906918) ;
        p134.data__SET(new short[] {(short)25285, (short)32057, (short) -27109, (short) -28178, (short)21812, (short) -16963, (short)3782, (short)20738, (short) -21104, (short)10411, (short) -19666, (short)2946, (short) -18952, (short)31362, (short) -21617, (short) -27013}, 0) ;
        LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1826177374);
            assert(pack.lon_GET() == -1923495320);
        });
        DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(-1826177374) ;
        p135.lon_SET(-1923495320) ;
        LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.current_height_GET() == 6.045667E37F);
            assert(pack.lon_GET() == -928564883);
            assert(pack.loaded_GET() == (char)57433);
            assert(pack.terrain_height_GET() == -2.1391296E38F);
            assert(pack.spacing_GET() == (char)54414);
            assert(pack.lat_GET() == -1776004626);
            assert(pack.pending_GET() == (char)2063);
        });
        DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.terrain_height_SET(-2.1391296E38F) ;
        p136.current_height_SET(6.045667E37F) ;
        p136.lon_SET(-928564883) ;
        p136.spacing_SET((char)54414) ;
        p136.pending_SET((char)2063) ;
        p136.loaded_SET((char)57433) ;
        p136.lat_SET(-1776004626) ;
        LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 22414934L);
            assert(pack.press_abs_GET() == 1.2153313E38F);
            assert(pack.temperature_GET() == (short) -16063);
            assert(pack.press_diff_GET() == 2.9742243E38F);
        });
        DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.press_diff_SET(2.9742243E38F) ;
        p137.press_abs_SET(1.2153313E38F) ;
        p137.time_boot_ms_SET(22414934L) ;
        p137.temperature_SET((short) -16063) ;
        LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -9.722194E37F);
            assert(pack.z_GET() == -1.4543789E38F);
            assert(pack.y_GET() == 2.4588623E38F);
            assert(pack.time_usec_GET() == 5028073576876518265L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {3.2186935E38F, 2.3055607E38F, -2.8167533E38F, -6.535061E37F}));
        });
        DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.x_SET(-9.722194E37F) ;
        p138.q_SET(new float[] {3.2186935E38F, 2.3055607E38F, -2.8167533E38F, -6.535061E37F}, 0) ;
        p138.y_SET(2.4588623E38F) ;
        p138.z_SET(-1.4543789E38F) ;
        p138.time_usec_SET(5028073576876518265L) ;
        LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)151);
            assert(pack.target_system_GET() == (char)27);
            assert(pack.time_usec_GET() == 5811721579400267742L);
            assert(pack.target_component_GET() == (char)22);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {1.7483585E38F, -2.7479907E38F, 3.000381E38F, 2.5462857E38F, -2.0631036E38F, -2.501003E37F, 3.3990712E38F, 2.299641E38F}));
        });
        DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.time_usec_SET(5811721579400267742L) ;
        p139.target_system_SET((char)27) ;
        p139.target_component_SET((char)22) ;
        p139.controls_SET(new float[] {1.7483585E38F, -2.7479907E38F, 3.000381E38F, 2.5462857E38F, -2.0631036E38F, -2.501003E37F, 3.3990712E38F, 2.299641E38F}, 0) ;
        p139.group_mlx_SET((char)151) ;
        LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.group_mlx_GET() == (char)28);
            assert(pack.time_usec_GET() == 3527332487005093002L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-2.6639073E38F, -9.946054E37F, -1.1189247E38F, -2.0170025E38F, 5.8400673E37F, 2.9609606E38F, 2.0558997E38F, 1.6325572E38F}));
        });
        DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.group_mlx_SET((char)28) ;
        p140.controls_SET(new float[] {-2.6639073E38F, -9.946054E37F, -1.1189247E38F, -2.0170025E38F, 5.8400673E37F, 2.9609606E38F, 2.0558997E38F, 1.6325572E38F}, 0) ;
        p140.time_usec_SET(3527332487005093002L) ;
        LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_monotonic_GET() == 3.336045E38F);
            assert(pack.altitude_terrain_GET() == -2.523067E38F);
            assert(pack.time_usec_GET() == 3678244811529845657L);
            assert(pack.altitude_amsl_GET() == -5.2705707E37F);
            assert(pack.bottom_clearance_GET() == -2.9205473E38F);
            assert(pack.altitude_local_GET() == -1.1880973E38F);
            assert(pack.altitude_relative_GET() == 1.5624091E38F);
        });
        DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.altitude_relative_SET(1.5624091E38F) ;
        p141.altitude_monotonic_SET(3.336045E38F) ;
        p141.bottom_clearance_SET(-2.9205473E38F) ;
        p141.altitude_local_SET(-1.1880973E38F) ;
        p141.altitude_amsl_SET(-5.2705707E37F) ;
        p141.altitude_terrain_SET(-2.523067E38F) ;
        p141.time_usec_SET(3678244811529845657L) ;
        LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.request_id_GET() == (char)118);
            assert(pack.transfer_type_GET() == (char)3);
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)32, (char)219, (char)153, (char)146, (char)156, (char)90, (char)179, (char)228, (char)61, (char)61, (char)244, (char)233, (char)170, (char)85, (char)125, (char)243, (char)219, (char)118, (char)234, (char)214, (char)199, (char)163, (char)39, (char)83, (char)37, (char)253, (char)96, (char)220, (char)15, (char)51, (char)74, (char)84, (char)178, (char)254, (char)85, (char)204, (char)28, (char)132, (char)63, (char)172, (char)133, (char)93, (char)55, (char)175, (char)43, (char)115, (char)217, (char)105, (char)93, (char)141, (char)179, (char)34, (char)208, (char)98, (char)116, (char)29, (char)76, (char)64, (char)9, (char)150, (char)173, (char)181, (char)122, (char)178, (char)135, (char)161, (char)200, (char)131, (char)135, (char)152, (char)126, (char)153, (char)168, (char)210, (char)134, (char)27, (char)83, (char)59, (char)22, (char)93, (char)78, (char)71, (char)100, (char)223, (char)130, (char)24, (char)252, (char)121, (char)142, (char)246, (char)182, (char)128, (char)176, (char)182, (char)40, (char)67, (char)105, (char)211, (char)94, (char)121, (char)97, (char)121, (char)4, (char)208, (char)208, (char)155, (char)23, (char)186, (char)74, (char)124, (char)184, (char)137, (char)27, (char)121, (char)178, (char)18, (char)17, (char)153, (char)114, (char)181}));
            assert(pack.uri_type_GET() == (char)240);
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)111, (char)6, (char)233, (char)166, (char)133, (char)91, (char)16, (char)115, (char)211, (char)152, (char)241, (char)118, (char)139, (char)86, (char)143, (char)244, (char)74, (char)1, (char)47, (char)115, (char)108, (char)136, (char)218, (char)178, (char)216, (char)219, (char)225, (char)45, (char)112, (char)70, (char)239, (char)246, (char)38, (char)13, (char)222, (char)73, (char)247, (char)236, (char)238, (char)126, (char)51, (char)75, (char)66, (char)173, (char)118, (char)76, (char)107, (char)78, (char)103, (char)203, (char)82, (char)68, (char)142, (char)56, (char)81, (char)181, (char)3, (char)187, (char)172, (char)113, (char)191, (char)146, (char)101, (char)248, (char)134, (char)203, (char)141, (char)65, (char)16, (char)86, (char)65, (char)64, (char)80, (char)178, (char)253, (char)221, (char)184, (char)84, (char)83, (char)195, (char)85, (char)164, (char)76, (char)27, (char)155, (char)185, (char)22, (char)141, (char)73, (char)140, (char)49, (char)217, (char)111, (char)137, (char)124, (char)177, (char)71, (char)170, (char)41, (char)252, (char)188, (char)167, (char)254, (char)50, (char)98, (char)149, (char)167, (char)114, (char)166, (char)150, (char)109, (char)73, (char)162, (char)12, (char)234, (char)105, (char)127, (char)204, (char)240, (char)109}));
        });
        DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.transfer_type_SET((char)3) ;
        p142.uri_SET(new char[] {(char)32, (char)219, (char)153, (char)146, (char)156, (char)90, (char)179, (char)228, (char)61, (char)61, (char)244, (char)233, (char)170, (char)85, (char)125, (char)243, (char)219, (char)118, (char)234, (char)214, (char)199, (char)163, (char)39, (char)83, (char)37, (char)253, (char)96, (char)220, (char)15, (char)51, (char)74, (char)84, (char)178, (char)254, (char)85, (char)204, (char)28, (char)132, (char)63, (char)172, (char)133, (char)93, (char)55, (char)175, (char)43, (char)115, (char)217, (char)105, (char)93, (char)141, (char)179, (char)34, (char)208, (char)98, (char)116, (char)29, (char)76, (char)64, (char)9, (char)150, (char)173, (char)181, (char)122, (char)178, (char)135, (char)161, (char)200, (char)131, (char)135, (char)152, (char)126, (char)153, (char)168, (char)210, (char)134, (char)27, (char)83, (char)59, (char)22, (char)93, (char)78, (char)71, (char)100, (char)223, (char)130, (char)24, (char)252, (char)121, (char)142, (char)246, (char)182, (char)128, (char)176, (char)182, (char)40, (char)67, (char)105, (char)211, (char)94, (char)121, (char)97, (char)121, (char)4, (char)208, (char)208, (char)155, (char)23, (char)186, (char)74, (char)124, (char)184, (char)137, (char)27, (char)121, (char)178, (char)18, (char)17, (char)153, (char)114, (char)181}, 0) ;
        p142.request_id_SET((char)118) ;
        p142.storage_SET(new char[] {(char)111, (char)6, (char)233, (char)166, (char)133, (char)91, (char)16, (char)115, (char)211, (char)152, (char)241, (char)118, (char)139, (char)86, (char)143, (char)244, (char)74, (char)1, (char)47, (char)115, (char)108, (char)136, (char)218, (char)178, (char)216, (char)219, (char)225, (char)45, (char)112, (char)70, (char)239, (char)246, (char)38, (char)13, (char)222, (char)73, (char)247, (char)236, (char)238, (char)126, (char)51, (char)75, (char)66, (char)173, (char)118, (char)76, (char)107, (char)78, (char)103, (char)203, (char)82, (char)68, (char)142, (char)56, (char)81, (char)181, (char)3, (char)187, (char)172, (char)113, (char)191, (char)146, (char)101, (char)248, (char)134, (char)203, (char)141, (char)65, (char)16, (char)86, (char)65, (char)64, (char)80, (char)178, (char)253, (char)221, (char)184, (char)84, (char)83, (char)195, (char)85, (char)164, (char)76, (char)27, (char)155, (char)185, (char)22, (char)141, (char)73, (char)140, (char)49, (char)217, (char)111, (char)137, (char)124, (char)177, (char)71, (char)170, (char)41, (char)252, (char)188, (char)167, (char)254, (char)50, (char)98, (char)149, (char)167, (char)114, (char)166, (char)150, (char)109, (char)73, (char)162, (char)12, (char)234, (char)105, (char)127, (char)204, (char)240, (char)109}, 0) ;
        p142.uri_type_SET((char)240) ;
        LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.press_diff_GET() == 2.450865E38F);
            assert(pack.time_boot_ms_GET() == 28623220L);
            assert(pack.temperature_GET() == (short)30228);
            assert(pack.press_abs_GET() == 7.1374296E37F);
        });
        DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.time_boot_ms_SET(28623220L) ;
        p143.temperature_SET((short)30228) ;
        p143.press_diff_SET(2.450865E38F) ;
        p143.press_abs_SET(7.1374296E37F) ;
        LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(pack.alt_GET() == -7.17267E37F);
            assert(Arrays.equals(pack.acc_GET(),  new float[] {1.299826E38F, 3.980901E36F, -3.0611563E38F}));
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {3.203994E38F, 3.3815342E38F, -1.9509554E38F}));
            assert(pack.est_capabilities_GET() == (char)106);
            assert(Arrays.equals(pack.vel_GET(),  new float[] {-9.813136E37F, -4.0843862E37F, 3.311909E38F}));
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {1.1435105E37F, 1.8982321E38F, -2.2786924E38F, 3.1024655E38F}));
            assert(pack.custom_state_GET() == 9039647676070511802L);
            assert(Arrays.equals(pack.rates_GET(),  new float[] {2.3228333E37F, 2.648394E37F, -1.5897825E38F}));
            assert(pack.lat_GET() == -1291834333);
            assert(pack.timestamp_GET() == 4123619064808673952L);
            assert(pack.lon_GET() == 1028082611);
        });
        DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.lat_SET(-1291834333) ;
        p144.lon_SET(1028082611) ;
        p144.alt_SET(-7.17267E37F) ;
        p144.rates_SET(new float[] {2.3228333E37F, 2.648394E37F, -1.5897825E38F}, 0) ;
        p144.vel_SET(new float[] {-9.813136E37F, -4.0843862E37F, 3.311909E38F}, 0) ;
        p144.timestamp_SET(4123619064808673952L) ;
        p144.custom_state_SET(9039647676070511802L) ;
        p144.attitude_q_SET(new float[] {1.1435105E37F, 1.8982321E38F, -2.2786924E38F, 3.1024655E38F}, 0) ;
        p144.position_cov_SET(new float[] {3.203994E38F, 3.3815342E38F, -1.9509554E38F}, 0) ;
        p144.acc_SET(new float[] {1.299826E38F, 3.980901E36F, -3.0611563E38F}, 0) ;
        p144.est_capabilities_SET((char)106) ;
        LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.y_pos_GET() == -4.3233442E36F);
            assert(pack.airspeed_GET() == 8.842584E37F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {7.4402343E37F, -5.3118155E37F, -1.1671994E38F}));
            assert(pack.y_vel_GET() == -3.2323435E38F);
            assert(pack.pitch_rate_GET() == 1.4450615E38F);
            assert(pack.z_pos_GET() == 4.554088E37F);
            assert(pack.x_pos_GET() == 2.047895E38F);
            assert(pack.x_acc_GET() == -2.6124768E38F);
            assert(pack.roll_rate_GET() == 1.4251242E38F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {-7.0851725E37F, 2.2856272E38F, -2.8209307E38F}));
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.6795753E38F, 1.7580505E38F, 2.5070965E37F, 1.2483102E38F}));
            assert(pack.x_vel_GET() == -2.7051376E38F);
            assert(pack.z_vel_GET() == -2.5660313E38F);
            assert(pack.time_usec_GET() == 4945723050938909874L);
            assert(pack.yaw_rate_GET() == 2.0938432E38F);
            assert(pack.y_acc_GET() == -7.4913323E37F);
            assert(pack.z_acc_GET() == 2.4627016E37F);
        });
        DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.y_vel_SET(-3.2323435E38F) ;
        p146.z_vel_SET(-2.5660313E38F) ;
        p146.z_acc_SET(2.4627016E37F) ;
        p146.x_vel_SET(-2.7051376E38F) ;
        p146.x_acc_SET(-2.6124768E38F) ;
        p146.y_acc_SET(-7.4913323E37F) ;
        p146.vel_variance_SET(new float[] {-7.0851725E37F, 2.2856272E38F, -2.8209307E38F}, 0) ;
        p146.yaw_rate_SET(2.0938432E38F) ;
        p146.z_pos_SET(4.554088E37F) ;
        p146.airspeed_SET(8.842584E37F) ;
        p146.pos_variance_SET(new float[] {7.4402343E37F, -5.3118155E37F, -1.1671994E38F}, 0) ;
        p146.y_pos_SET(-4.3233442E36F) ;
        p146.x_pos_SET(2.047895E38F) ;
        p146.time_usec_SET(4945723050938909874L) ;
        p146.q_SET(new float[] {-2.6795753E38F, 1.7580505E38F, 2.5070965E37F, 1.2483102E38F}, 0) ;
        p146.roll_rate_SET(1.4251242E38F) ;
        p146.pitch_rate_SET(1.4450615E38F) ;
        LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.current_battery_GET() == (short) -14633);
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION);
            assert(pack.battery_remaining_GET() == (byte) - 109);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)16948, (char)42586, (char)23722, (char)18175, (char)51398, (char)25398, (char)62296, (char)51723, (char)31402, (char)52154}));
            assert(pack.current_consumed_GET() == 1165247339);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD);
            assert(pack.energy_consumed_GET() == 2006016130);
            assert(pack.temperature_GET() == (short) -7877);
            assert(pack.id_GET() == (char)123);
        });
        DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.current_battery_SET((short) -14633) ;
        p147.id_SET((char)123) ;
        p147.temperature_SET((short) -7877) ;
        p147.battery_remaining_SET((byte) - 109) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_TYPE_PAYLOAD) ;
        p147.current_consumed_SET(1165247339) ;
        p147.energy_consumed_SET(2006016130) ;
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_LION) ;
        p147.voltages_SET(new char[] {(char)16948, (char)42586, (char)23722, (char)18175, (char)51398, (char)25398, (char)62296, (char)51723, (char)31402, (char)52154}, 0) ;
        LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(pack.board_version_GET() == 4100456880L);
            assert(pack.vendor_id_GET() == (char)37403);
            assert(pack.product_id_GET() == (char)15367);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)81, (char)46, (char)209, (char)61, (char)98, (char)22, (char)93, (char)203}));
            assert(pack.capabilities_GET() == MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT);
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)102, (char)9, (char)157, (char)232, (char)126, (char)24, (char)184, (char)211}));
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)235, (char)126, (char)192, (char)17, (char)84, (char)99, (char)125, (char)149, (char)95, (char)103, (char)64, (char)9, (char)173, (char)105, (char)253, (char)52, (char)190, (char)54}));
            assert(pack.flight_sw_version_GET() == 1902555924L);
            assert(pack.uid_GET() == 5088354382561347301L);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)252, (char)72, (char)80, (char)244, (char)49, (char)155, (char)236, (char)159}));
            assert(pack.middleware_sw_version_GET() == 2764192278L);
            assert(pack.os_sw_version_GET() == 729854224L);
        });
        DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.flight_custom_version_SET(new char[] {(char)252, (char)72, (char)80, (char)244, (char)49, (char)155, (char)236, (char)159}, 0) ;
        p148.board_version_SET(4100456880L) ;
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT) ;
        p148.middleware_custom_version_SET(new char[] {(char)81, (char)46, (char)209, (char)61, (char)98, (char)22, (char)93, (char)203}, 0) ;
        p148.product_id_SET((char)15367) ;
        p148.uid_SET(5088354382561347301L) ;
        p148.uid2_SET(new char[] {(char)235, (char)126, (char)192, (char)17, (char)84, (char)99, (char)125, (char)149, (char)95, (char)103, (char)64, (char)9, (char)173, (char)105, (char)253, (char)52, (char)190, (char)54}, 0, PH) ;
        p148.os_sw_version_SET(729854224L) ;
        p148.middleware_sw_version_SET(2764192278L) ;
        p148.vendor_id_SET((char)37403) ;
        p148.os_custom_version_SET(new char[] {(char)102, (char)9, (char)157, (char)232, (char)126, (char)24, (char)184, (char)211}, 0) ;
        p148.flight_sw_version_SET(1902555924L) ;
        LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.size_x_GET() == -7.277971E36F);
            assert(pack.z_TRY(ph) == 1.0076397E38F);
            assert(pack.angle_x_GET() == -4.0078209E37F);
            assert(pack.y_TRY(ph) == -2.1266999E38F);
            assert(pack.distance_GET() == 3.152293E38F);
            assert(pack.size_y_GET() == -1.168936E38F);
            assert(pack.time_usec_GET() == 4335539563086172452L);
            assert(pack.position_valid_TRY(ph) == (char)109);
            assert(pack.angle_y_GET() == 2.2624476E38F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {-1.2410696E38F, -2.3850996E38F, 2.9635876E38F, -1.3635475E38F}));
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON);
            assert(pack.target_num_GET() == (char)130);
            assert(pack.x_TRY(ph) == -1.3021236E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_NED);
        });
        DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.distance_SET(3.152293E38F) ;
        p149.z_SET(1.0076397E38F, PH) ;
        p149.angle_x_SET(-4.0078209E37F) ;
        p149.size_y_SET(-1.168936E38F) ;
        p149.q_SET(new float[] {-1.2410696E38F, -2.3850996E38F, 2.9635876E38F, -1.3635475E38F}, 0, PH) ;
        p149.position_valid_SET((char)109, PH) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_NED) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_RADIO_BEACON) ;
        p149.angle_y_SET(2.2624476E38F) ;
        p149.size_x_SET(-7.277971E36F) ;
        p149.time_usec_SET(4335539563086172452L) ;
        p149.y_SET(-2.1266999E38F, PH) ;
        p149.target_num_SET((char)130) ;
        p149.x_SET(-1.3021236E38F, PH) ;
        LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCRIPT_ITEM.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)93);
            assert(pack.name_LEN(ph) == 11);
            assert(pack.name_TRY(ph).equals("yqvcadjoiiv"));
            assert(pack.seq_GET() == (char)15402);
            assert(pack.target_system_GET() == (char)125);
        });
        DemoDevice.SCRIPT_ITEM p180 = LoopBackDemoChannel.new_SCRIPT_ITEM();
        PH.setPack(p180);
        p180.seq_SET((char)15402) ;
        p180.target_system_SET((char)125) ;
        p180.target_component_SET((char)93) ;
        p180.name_SET("yqvcadjoiiv", PH) ;
        LoopBackDemoChannel.instance.send(p180);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCRIPT_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)64107);
            assert(pack.target_component_GET() == (char)106);
            assert(pack.target_system_GET() == (char)53);
        });
        DemoDevice.SCRIPT_REQUEST p181 = LoopBackDemoChannel.new_SCRIPT_REQUEST();
        PH.setPack(p181);
        p181.target_system_SET((char)53) ;
        p181.target_component_SET((char)106) ;
        p181.seq_SET((char)64107) ;
        LoopBackDemoChannel.instance.send(p181);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCRIPT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)167);
            assert(pack.target_component_GET() == (char)85);
        });
        DemoDevice.SCRIPT_REQUEST_LIST p182 = LoopBackDemoChannel.new_SCRIPT_REQUEST_LIST();
        PH.setPack(p182);
        p182.target_component_SET((char)85) ;
        p182.target_system_SET((char)167) ;
        LoopBackDemoChannel.instance.send(p182);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCRIPT_COUNT.add((src, ph, pack) ->
        {
            assert(pack.count_GET() == (char)4105);
            assert(pack.target_system_GET() == (char)214);
            assert(pack.target_component_GET() == (char)203);
        });
        DemoDevice.SCRIPT_COUNT p183 = LoopBackDemoChannel.new_SCRIPT_COUNT();
        PH.setPack(p183);
        p183.target_component_SET((char)203) ;
        p183.count_SET((char)4105) ;
        p183.target_system_SET((char)214) ;
        LoopBackDemoChannel.instance.send(p183);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCRIPT_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)18590);
        });
        DemoDevice.SCRIPT_CURRENT p184 = LoopBackDemoChannel.new_SCRIPT_CURRENT();
        PH.setPack(p184);
        p184.seq_SET((char)18590) ;
        LoopBackDemoChannel.instance.send(p184);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL);
            assert(pack.pos_vert_ratio_GET() == 7.216982E37F);
            assert(pack.pos_vert_accuracy_GET() == 1.586466E37F);
            assert(pack.time_usec_GET() == 5129265395808343298L);
            assert(pack.pos_horiz_ratio_GET() == 4.9041715E37F);
            assert(pack.tas_ratio_GET() == -8.508575E37F);
            assert(pack.mag_ratio_GET() == -2.9563212E38F);
            assert(pack.hagl_ratio_GET() == -1.7165914E38F);
            assert(pack.vel_ratio_GET() == -1.2542499E38F);
            assert(pack.pos_horiz_accuracy_GET() == -2.3169584E38F);
        });
        DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.pos_vert_ratio_SET(7.216982E37F) ;
        p230.time_usec_SET(5129265395808343298L) ;
        p230.hagl_ratio_SET(-1.7165914E38F) ;
        p230.pos_vert_accuracy_SET(1.586466E37F) ;
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_POS_VERT_AGL) ;
        p230.tas_ratio_SET(-8.508575E37F) ;
        p230.pos_horiz_accuracy_SET(-2.3169584E38F) ;
        p230.vel_ratio_SET(-1.2542499E38F) ;
        p230.pos_horiz_ratio_SET(4.9041715E37F) ;
        p230.mag_ratio_SET(-2.9563212E38F) ;
        LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.var_horiz_GET() == 1.652399E38F);
            assert(pack.horiz_accuracy_GET() == 2.5566561E38F);
            assert(pack.wind_y_GET() == 6.3048225E37F);
            assert(pack.wind_z_GET() == 1.0427003E37F);
            assert(pack.var_vert_GET() == 9.062857E37F);
            assert(pack.vert_accuracy_GET() == 6.356375E37F);
            assert(pack.wind_x_GET() == 1.4896539E38F);
            assert(pack.time_usec_GET() == 3743606105701124773L);
            assert(pack.wind_alt_GET() == 1.1587277E38F);
        });
        DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.wind_y_SET(6.3048225E37F) ;
        p231.horiz_accuracy_SET(2.5566561E38F) ;
        p231.wind_alt_SET(1.1587277E38F) ;
        p231.var_vert_SET(9.062857E37F) ;
        p231.time_usec_SET(3743606105701124773L) ;
        p231.vert_accuracy_SET(6.356375E37F) ;
        p231.wind_z_SET(1.0427003E37F) ;
        p231.wind_x_SET(1.4896539E38F) ;
        p231.var_horiz_SET(1.652399E38F) ;
        LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.satellites_visible_GET() == (char)93);
            assert(pack.gps_id_GET() == (char)175);
            assert(pack.horiz_accuracy_GET() == -2.7477686E38F);
            assert(pack.hdop_GET() == -2.0047065E38F);
            assert(pack.vn_GET() == -2.7918E38F);
            assert(pack.speed_accuracy_GET() == 3.2331611E38F);
            assert(pack.fix_type_GET() == (char)255);
            assert(pack.lat_GET() == 1261950487);
            assert(pack.alt_GET() == 1.8829806E38F);
            assert(pack.vd_GET() == 1.3797962E38F);
            assert(pack.lon_GET() == 1050518426);
            assert(pack.time_week_GET() == (char)54773);
            assert(pack.time_usec_GET() == 1272123214746006263L);
            assert(pack.time_week_ms_GET() == 985598901L);
            assert(pack.ve_GET() == -3.0372125E38F);
            assert(pack.vdop_GET() == 7.9351905E37F);
            assert(pack.vert_accuracy_GET() == 2.2154803E38F);
            assert(pack.ignore_flags_GET() == GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT);
        });
        DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.satellites_visible_SET((char)93) ;
        p232.time_week_SET((char)54773) ;
        p232.gps_id_SET((char)175) ;
        p232.lon_SET(1050518426) ;
        p232.ve_SET(-3.0372125E38F) ;
        p232.vn_SET(-2.7918E38F) ;
        p232.fix_type_SET((char)255) ;
        p232.vd_SET(1.3797962E38F) ;
        p232.time_usec_SET(1272123214746006263L) ;
        p232.time_week_ms_SET(985598901L) ;
        p232.speed_accuracy_SET(3.2331611E38F) ;
        p232.vdop_SET(7.9351905E37F) ;
        p232.horiz_accuracy_SET(-2.7477686E38F) ;
        p232.hdop_SET(-2.0047065E38F) ;
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VEL_VERT) ;
        p232.alt_SET(1.8829806E38F) ;
        p232.vert_accuracy_SET(2.2154803E38F) ;
        p232.lat_SET(1261950487) ;
        LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.len_GET() == (char)125);
            assert(pack.flags_GET() == (char)86);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)29, (char)193, (char)113, (char)135, (char)89, (char)143, (char)93, (char)160, (char)168, (char)226, (char)57, (char)91, (char)234, (char)34, (char)151, (char)239, (char)244, (char)98, (char)192, (char)238, (char)82, (char)27, (char)132, (char)157, (char)80, (char)137, (char)236, (char)138, (char)68, (char)96, (char)127, (char)14, (char)159, (char)21, (char)82, (char)124, (char)128, (char)144, (char)189, (char)161, (char)168, (char)139, (char)159, (char)247, (char)84, (char)243, (char)82, (char)232, (char)178, (char)125, (char)62, (char)50, (char)226, (char)142, (char)208, (char)152, (char)183, (char)159, (char)124, (char)15, (char)204, (char)4, (char)80, (char)146, (char)7, (char)220, (char)32, (char)99, (char)139, (char)207, (char)255, (char)195, (char)233, (char)178, (char)13, (char)173, (char)175, (char)54, (char)107, (char)179, (char)145, (char)198, (char)161, (char)143, (char)102, (char)61, (char)229, (char)118, (char)162, (char)71, (char)132, (char)183, (char)237, (char)174, (char)106, (char)89, (char)153, (char)193, (char)188, (char)120, (char)170, (char)36, (char)92, (char)143, (char)102, (char)27, (char)231, (char)2, (char)248, (char)73, (char)214, (char)112, (char)39, (char)38, (char)184, (char)162, (char)208, (char)160, (char)35, (char)103, (char)33, (char)110, (char)26, (char)99, (char)74, (char)239, (char)136, (char)71, (char)50, (char)59, (char)241, (char)130, (char)47, (char)36, (char)80, (char)69, (char)60, (char)227, (char)185, (char)144, (char)49, (char)158, (char)71, (char)119, (char)170, (char)160, (char)138, (char)180, (char)65, (char)19, (char)117, (char)225, (char)121, (char)220, (char)196, (char)200, (char)21, (char)21, (char)13, (char)10, (char)143, (char)108, (char)86, (char)106, (char)201, (char)144, (char)185, (char)135, (char)108, (char)232, (char)57, (char)46, (char)253, (char)93, (char)165, (char)53, (char)234, (char)226, (char)170, (char)20}));
        });
        DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.len_SET((char)125) ;
        p233.flags_SET((char)86) ;
        p233.data__SET(new char[] {(char)29, (char)193, (char)113, (char)135, (char)89, (char)143, (char)93, (char)160, (char)168, (char)226, (char)57, (char)91, (char)234, (char)34, (char)151, (char)239, (char)244, (char)98, (char)192, (char)238, (char)82, (char)27, (char)132, (char)157, (char)80, (char)137, (char)236, (char)138, (char)68, (char)96, (char)127, (char)14, (char)159, (char)21, (char)82, (char)124, (char)128, (char)144, (char)189, (char)161, (char)168, (char)139, (char)159, (char)247, (char)84, (char)243, (char)82, (char)232, (char)178, (char)125, (char)62, (char)50, (char)226, (char)142, (char)208, (char)152, (char)183, (char)159, (char)124, (char)15, (char)204, (char)4, (char)80, (char)146, (char)7, (char)220, (char)32, (char)99, (char)139, (char)207, (char)255, (char)195, (char)233, (char)178, (char)13, (char)173, (char)175, (char)54, (char)107, (char)179, (char)145, (char)198, (char)161, (char)143, (char)102, (char)61, (char)229, (char)118, (char)162, (char)71, (char)132, (char)183, (char)237, (char)174, (char)106, (char)89, (char)153, (char)193, (char)188, (char)120, (char)170, (char)36, (char)92, (char)143, (char)102, (char)27, (char)231, (char)2, (char)248, (char)73, (char)214, (char)112, (char)39, (char)38, (char)184, (char)162, (char)208, (char)160, (char)35, (char)103, (char)33, (char)110, (char)26, (char)99, (char)74, (char)239, (char)136, (char)71, (char)50, (char)59, (char)241, (char)130, (char)47, (char)36, (char)80, (char)69, (char)60, (char)227, (char)185, (char)144, (char)49, (char)158, (char)71, (char)119, (char)170, (char)160, (char)138, (char)180, (char)65, (char)19, (char)117, (char)225, (char)121, (char)220, (char)196, (char)200, (char)21, (char)21, (char)13, (char)10, (char)143, (char)108, (char)86, (char)106, (char)201, (char)144, (char)185, (char)135, (char)108, (char)232, (char)57, (char)46, (char)253, (char)93, (char)165, (char)53, (char)234, (char)226, (char)170, (char)20}, 0) ;
        LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.airspeed_sp_GET() == (char)55);
            assert(pack.roll_GET() == (short) -21308);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED);
            assert(pack.altitude_amsl_GET() == (short)30970);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
            assert(pack.wp_distance_GET() == (char)6639);
            assert(pack.custom_mode_GET() == 3337399406L);
            assert(pack.climb_rate_GET() == (byte)26);
            assert(pack.heading_GET() == (char)49542);
            assert(pack.latitude_GET() == 142015712);
            assert(pack.altitude_sp_GET() == (short)4730);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND);
            assert(pack.longitude_GET() == -2000140249);
            assert(pack.gps_nsat_GET() == (char)220);
            assert(pack.throttle_GET() == (byte) - 58);
            assert(pack.wp_num_GET() == (char)63);
            assert(pack.airspeed_GET() == (char)242);
            assert(pack.groundspeed_GET() == (char)133);
            assert(pack.heading_sp_GET() == (short)919);
            assert(pack.battery_remaining_GET() == (char)3);
            assert(pack.failsafe_GET() == (char)157);
            assert(pack.temperature_air_GET() == (byte)122);
            assert(pack.temperature_GET() == (byte) - 40);
            assert(pack.pitch_GET() == (short)11505);
        });
        DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.heading_sp_SET((short)919) ;
        p234.gps_nsat_SET((char)220) ;
        p234.custom_mode_SET(3337399406L) ;
        p234.climb_rate_SET((byte)26) ;
        p234.wp_distance_SET((char)6639) ;
        p234.groundspeed_SET((char)133) ;
        p234.failsafe_SET((char)157) ;
        p234.altitude_amsl_SET((short)30970) ;
        p234.roll_SET((short) -21308) ;
        p234.temperature_SET((byte) - 40) ;
        p234.heading_SET((char)49542) ;
        p234.battery_remaining_SET((char)3) ;
        p234.latitude_SET(142015712) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_ON_GROUND) ;
        p234.temperature_air_SET((byte)122) ;
        p234.altitude_sp_SET((short)4730) ;
        p234.airspeed_SET((char)242) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED) ;
        p234.longitude_SET(-2000140249) ;
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_TEST_ENABLED) ;
        p234.pitch_SET((short)11505) ;
        p234.airspeed_sp_SET((char)55) ;
        p234.throttle_SET((byte) - 58) ;
        p234.wp_num_SET((char)63) ;
        LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.vibration_y_GET() == 2.7996498E38F);
            assert(pack.vibration_x_GET() == -2.1672477E38F);
            assert(pack.clipping_0_GET() == 2949048476L);
            assert(pack.time_usec_GET() == 5316674329797476782L);
            assert(pack.vibration_z_GET() == 2.6522492E38F);
            assert(pack.clipping_2_GET() == 2482842870L);
            assert(pack.clipping_1_GET() == 1245969523L);
        });
        DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.clipping_2_SET(2482842870L) ;
        p241.time_usec_SET(5316674329797476782L) ;
        p241.clipping_1_SET(1245969523L) ;
        p241.vibration_z_SET(2.6522492E38F) ;
        p241.vibration_x_SET(-2.1672477E38F) ;
        p241.vibration_y_SET(2.7996498E38F) ;
        p241.clipping_0_SET(2949048476L) ;
        LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.approach_x_GET() == -2.8141541E38F);
            assert(pack.y_GET() == 3.369563E38F);
            assert(pack.approach_z_GET() == -2.9897154E38F);
            assert(pack.approach_y_GET() == 1.5238217E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.5892646E38F, 8.468918E37F, -3.316375E38F, -2.5282833E38F}));
            assert(pack.z_GET() == -1.7286294E38F);
            assert(pack.latitude_GET() == -680109370);
            assert(pack.time_usec_TRY(ph) == 2458761240988167342L);
            assert(pack.x_GET() == 2.9664147E37F);
            assert(pack.longitude_GET() == -1493243422);
            assert(pack.altitude_GET() == 1042657608);
        });
        DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.approach_y_SET(1.5238217E38F) ;
        p242.y_SET(3.369563E38F) ;
        p242.x_SET(2.9664147E37F) ;
        p242.approach_x_SET(-2.8141541E38F) ;
        p242.time_usec_SET(2458761240988167342L, PH) ;
        p242.q_SET(new float[] {1.5892646E38F, 8.468918E37F, -3.316375E38F, -2.5282833E38F}, 0) ;
        p242.z_SET(-1.7286294E38F) ;
        p242.altitude_SET(1042657608) ;
        p242.longitude_SET(-1493243422) ;
        p242.latitude_SET(-680109370) ;
        p242.approach_z_SET(-2.9897154E38F) ;
        LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.approach_x_GET() == 3.2029553E38F);
            assert(pack.target_system_GET() == (char)113);
            assert(pack.latitude_GET() == -355165564);
            assert(pack.altitude_GET() == 1259124647);
            assert(pack.approach_y_GET() == 7.622444E37F);
            assert(pack.longitude_GET() == 2121799931);
            assert(pack.time_usec_TRY(ph) == 5904779958913902769L);
            assert(pack.y_GET() == -2.5507434E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.5021601E38F, -2.3227111E38F, 2.7917903E38F, -2.8222563E38F}));
            assert(pack.z_GET() == 6.776188E37F);
            assert(pack.approach_z_GET() == 1.6418756E38F);
            assert(pack.x_GET() == -1.2834074E38F);
        });
        DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.approach_z_SET(1.6418756E38F) ;
        p243.time_usec_SET(5904779958913902769L, PH) ;
        p243.target_system_SET((char)113) ;
        p243.z_SET(6.776188E37F) ;
        p243.altitude_SET(1259124647) ;
        p243.latitude_SET(-355165564) ;
        p243.approach_y_SET(7.622444E37F) ;
        p243.longitude_SET(2121799931) ;
        p243.x_SET(-1.2834074E38F) ;
        p243.y_SET(-2.5507434E38F) ;
        p243.q_SET(new float[] {-1.5021601E38F, -2.3227111E38F, 2.7917903E38F, -2.8222563E38F}, 0) ;
        p243.approach_x_SET(3.2029553E38F) ;
        LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.message_id_GET() == (char)20317);
            assert(pack.interval_us_GET() == 1390158586);
        });
        DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.message_id_SET((char)20317) ;
        p244.interval_us_SET(1390158586) ;
        LoopBackDemoChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_FW);
        });
        DemoDevice.EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_FW) ;
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        LoopBackDemoChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            assert(pack.lat_GET() == 960999930);
            assert(pack.ver_velocity_GET() == (short) -15438);
            assert(pack.altitude_GET() == 2090478945);
            assert(pack.lon_GET() == -1277439361);
            assert(pack.flags_GET() == ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_EMERGENCY_SURFACE);
            assert(pack.heading_GET() == (char)34733);
            assert(pack.callsign_LEN(ph) == 8);
            assert(pack.callsign_TRY(ph).equals("oozogakp"));
            assert(pack.ICAO_address_GET() == 1316947286L);
            assert(pack.tslc_GET() == (char)7);
            assert(pack.squawk_GET() == (char)105);
            assert(pack.hor_velocity_GET() == (char)41648);
        });
        DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.heading_SET((char)34733) ;
        p246.tslc_SET((char)7) ;
        p246.hor_velocity_SET((char)41648) ;
        p246.callsign_SET("oozogakp", PH) ;
        p246.ver_velocity_SET((short) -15438) ;
        p246.altitude_SET(2090478945) ;
        p246.lon_SET(-1277439361) ;
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_COORDS) ;
        p246.ICAO_address_SET(1316947286L) ;
        p246.squawk_SET((char)105) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_EMERGENCY_SURFACE) ;
        p246.lat_SET(960999930) ;
        LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
            assert(pack.horizontal_minimum_delta_GET() == -2.277521E37F);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER);
            assert(pack.id_GET() == 4258083571L);
            assert(pack.time_to_minimum_delta_GET() == 6.6701625E37F);
            assert(pack.altitude_minimum_delta_GET() == 2.5667588E38F);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
        });
        DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
        PH.setPack(p247);
        p247.time_to_minimum_delta_SET(6.6701625E37F) ;
        p247.altitude_minimum_delta_SET(2.5667588E38F) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE) ;
        p247.id_SET(4258083571L) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_HOVER) ;
        p247.horizontal_minimum_delta_SET(-2.277521E37F) ;
        LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)197);
            assert(pack.target_network_GET() == (char)5);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)56, (char)127, (char)29, (char)229, (char)121, (char)144, (char)239, (char)68, (char)131, (char)245, (char)76, (char)202, (char)149, (char)211, (char)93, (char)249, (char)126, (char)189, (char)14, (char)78, (char)51, (char)63, (char)171, (char)120, (char)23, (char)206, (char)26, (char)64, (char)28, (char)64, (char)188, (char)175, (char)61, (char)197, (char)159, (char)254, (char)37, (char)182, (char)87, (char)135, (char)26, (char)205, (char)142, (char)19, (char)80, (char)124, (char)116, (char)180, (char)96, (char)218, (char)156, (char)80, (char)142, (char)161, (char)189, (char)163, (char)94, (char)208, (char)80, (char)26, (char)249, (char)136, (char)62, (char)52, (char)186, (char)29, (char)22, (char)158, (char)242, (char)204, (char)221, (char)194, (char)164, (char)49, (char)253, (char)148, (char)144, (char)104, (char)39, (char)93, (char)121, (char)45, (char)226, (char)254, (char)87, (char)149, (char)245, (char)84, (char)198, (char)173, (char)71, (char)133, (char)138, (char)199, (char)127, (char)66, (char)57, (char)195, (char)204, (char)0, (char)76, (char)231, (char)17, (char)100, (char)205, (char)96, (char)175, (char)11, (char)201, (char)72, (char)182, (char)44, (char)246, (char)24, (char)213, (char)210, (char)46, (char)75, (char)47, (char)241, (char)170, (char)37, (char)97, (char)216, (char)83, (char)162, (char)152, (char)126, (char)33, (char)199, (char)35, (char)18, (char)250, (char)202, (char)153, (char)89, (char)160, (char)206, (char)232, (char)110, (char)185, (char)124, (char)227, (char)52, (char)159, (char)67, (char)180, (char)157, (char)22, (char)244, (char)119, (char)58, (char)165, (char)58, (char)159, (char)90, (char)235, (char)167, (char)146, (char)186, (char)213, (char)138, (char)174, (char)230, (char)46, (char)57, (char)75, (char)250, (char)168, (char)182, (char)14, (char)169, (char)214, (char)102, (char)6, (char)247, (char)91, (char)79, (char)100, (char)183, (char)47, (char)64, (char)122, (char)168, (char)201, (char)73, (char)48, (char)173, (char)82, (char)221, (char)83, (char)177, (char)158, (char)166, (char)79, (char)28, (char)87, (char)0, (char)174, (char)108, (char)46, (char)83, (char)81, (char)185, (char)98, (char)170, (char)210, (char)109, (char)139, (char)205, (char)75, (char)247, (char)147, (char)131, (char)181, (char)160, (char)150, (char)49, (char)237, (char)152, (char)105, (char)65, (char)18, (char)55, (char)211, (char)249, (char)126, (char)203, (char)84, (char)228, (char)192, (char)167, (char)31, (char)155, (char)240, (char)140, (char)34, (char)124, (char)117, (char)6, (char)98, (char)209, (char)7, (char)76, (char)114, (char)222, (char)249, (char)245, (char)40}));
            assert(pack.message_type_GET() == (char)46006);
            assert(pack.target_component_GET() == (char)216);
        });
        DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)5) ;
        p248.payload_SET(new char[] {(char)56, (char)127, (char)29, (char)229, (char)121, (char)144, (char)239, (char)68, (char)131, (char)245, (char)76, (char)202, (char)149, (char)211, (char)93, (char)249, (char)126, (char)189, (char)14, (char)78, (char)51, (char)63, (char)171, (char)120, (char)23, (char)206, (char)26, (char)64, (char)28, (char)64, (char)188, (char)175, (char)61, (char)197, (char)159, (char)254, (char)37, (char)182, (char)87, (char)135, (char)26, (char)205, (char)142, (char)19, (char)80, (char)124, (char)116, (char)180, (char)96, (char)218, (char)156, (char)80, (char)142, (char)161, (char)189, (char)163, (char)94, (char)208, (char)80, (char)26, (char)249, (char)136, (char)62, (char)52, (char)186, (char)29, (char)22, (char)158, (char)242, (char)204, (char)221, (char)194, (char)164, (char)49, (char)253, (char)148, (char)144, (char)104, (char)39, (char)93, (char)121, (char)45, (char)226, (char)254, (char)87, (char)149, (char)245, (char)84, (char)198, (char)173, (char)71, (char)133, (char)138, (char)199, (char)127, (char)66, (char)57, (char)195, (char)204, (char)0, (char)76, (char)231, (char)17, (char)100, (char)205, (char)96, (char)175, (char)11, (char)201, (char)72, (char)182, (char)44, (char)246, (char)24, (char)213, (char)210, (char)46, (char)75, (char)47, (char)241, (char)170, (char)37, (char)97, (char)216, (char)83, (char)162, (char)152, (char)126, (char)33, (char)199, (char)35, (char)18, (char)250, (char)202, (char)153, (char)89, (char)160, (char)206, (char)232, (char)110, (char)185, (char)124, (char)227, (char)52, (char)159, (char)67, (char)180, (char)157, (char)22, (char)244, (char)119, (char)58, (char)165, (char)58, (char)159, (char)90, (char)235, (char)167, (char)146, (char)186, (char)213, (char)138, (char)174, (char)230, (char)46, (char)57, (char)75, (char)250, (char)168, (char)182, (char)14, (char)169, (char)214, (char)102, (char)6, (char)247, (char)91, (char)79, (char)100, (char)183, (char)47, (char)64, (char)122, (char)168, (char)201, (char)73, (char)48, (char)173, (char)82, (char)221, (char)83, (char)177, (char)158, (char)166, (char)79, (char)28, (char)87, (char)0, (char)174, (char)108, (char)46, (char)83, (char)81, (char)185, (char)98, (char)170, (char)210, (char)109, (char)139, (char)205, (char)75, (char)247, (char)147, (char)131, (char)181, (char)160, (char)150, (char)49, (char)237, (char)152, (char)105, (char)65, (char)18, (char)55, (char)211, (char)249, (char)126, (char)203, (char)84, (char)228, (char)192, (char)167, (char)31, (char)155, (char)240, (char)140, (char)34, (char)124, (char)117, (char)6, (char)98, (char)209, (char)7, (char)76, (char)114, (char)222, (char)249, (char)245, (char)40}, 0) ;
        p248.target_system_SET((char)197) ;
        p248.target_component_SET((char)216) ;
        p248.message_type_SET((char)46006) ;
        LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.address_GET() == (char)55114);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)85, (byte)67, (byte)42, (byte) - 73, (byte)3, (byte)84, (byte)21, (byte)93, (byte)43, (byte)5, (byte)3, (byte)93, (byte)110, (byte) - 88, (byte)71, (byte)124, (byte)60, (byte)86, (byte)71, (byte)48, (byte)15, (byte)44, (byte) - 121, (byte) - 74, (byte)124, (byte)35, (byte) - 22, (byte)94, (byte) - 119, (byte) - 113, (byte)34, (byte) - 107}));
            assert(pack.ver_GET() == (char)206);
            assert(pack.type_GET() == (char)86);
        });
        DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.ver_SET((char)206) ;
        p249.value_SET(new byte[] {(byte)85, (byte)67, (byte)42, (byte) - 73, (byte)3, (byte)84, (byte)21, (byte)93, (byte)43, (byte)5, (byte)3, (byte)93, (byte)110, (byte) - 88, (byte)71, (byte)124, (byte)60, (byte)86, (byte)71, (byte)48, (byte)15, (byte)44, (byte) - 121, (byte) - 74, (byte)124, (byte)35, (byte) - 22, (byte)94, (byte) - 119, (byte) - 113, (byte)34, (byte) - 107}, 0) ;
        p249.address_SET((char)55114) ;
        p249.type_SET((char)86) ;
        LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 2.0623783E38F);
            assert(pack.time_usec_GET() == 1439382108194288329L);
            assert(pack.y_GET() == 2.0125298E38F);
            assert(pack.name_LEN(ph) == 2);
            assert(pack.name_TRY(ph).equals("tu"));
            assert(pack.z_GET() == 1.2858724E38F);
        });
        DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.y_SET(2.0125298E38F) ;
        p250.x_SET(2.0623783E38F) ;
        p250.time_usec_SET(1439382108194288329L) ;
        p250.z_SET(1.2858724E38F) ;
        p250.name_SET("tu", PH) ;
        LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3353032132L);
            assert(pack.value_GET() == 1.4138586E38F);
            assert(pack.name_LEN(ph) == 10);
            assert(pack.name_TRY(ph).equals("vcltlreyyh"));
        });
        DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.time_boot_ms_SET(3353032132L) ;
        p251.name_SET("vcltlreyyh", PH) ;
        p251.value_SET(1.4138586E38F) ;
        LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == -1460684479);
            assert(pack.time_boot_ms_GET() == 439865401L);
            assert(pack.name_LEN(ph) == 9);
            assert(pack.name_TRY(ph).equals("nmowsxtsm"));
        });
        DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.name_SET("nmowsxtsm", PH) ;
        p252.time_boot_ms_SET(439865401L) ;
        p252.value_SET(-1460684479) ;
        LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_DEBUG);
            assert(pack.text_LEN(ph) == 31);
            assert(pack.text_TRY(ph).equals("ulibrcxceuektdrwuzzeeazenlgqXdl"));
        });
        DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_DEBUG) ;
        p253.text_SET("ulibrcxceuektdrwuzzeeazenlgqXdl", PH) ;
        LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.value_GET() == -6.584011E36F);
            assert(pack.ind_GET() == (char)51);
            assert(pack.time_boot_ms_GET() == 1325905819L);
        });
        DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
        PH.setPack(p254);
        p254.ind_SET((char)51) ;
        p254.value_SET(-6.584011E36F) ;
        p254.time_boot_ms_SET(1325905819L) ;
        LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.initial_timestamp_GET() == 4168437712960223379L);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)124, (char)100, (char)180, (char)74, (char)55, (char)9, (char)111, (char)116, (char)247, (char)0, (char)239, (char)102, (char)145, (char)110, (char)76, (char)232, (char)245, (char)203, (char)253, (char)155, (char)47, (char)147, (char)4, (char)161, (char)236, (char)210, (char)222, (char)65, (char)221, (char)4, (char)153, (char)96}));
            assert(pack.target_component_GET() == (char)219);
            assert(pack.target_system_GET() == (char)172);
        });
        DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.initial_timestamp_SET(4168437712960223379L) ;
        p256.target_system_SET((char)172) ;
        p256.secret_key_SET(new char[] {(char)124, (char)100, (char)180, (char)74, (char)55, (char)9, (char)111, (char)116, (char)247, (char)0, (char)239, (char)102, (char)145, (char)110, (char)76, (char)232, (char)245, (char)203, (char)253, (char)155, (char)47, (char)147, (char)4, (char)161, (char)236, (char)210, (char)222, (char)65, (char)221, (char)4, (char)153, (char)96}, 0) ;
        p256.target_component_SET((char)219) ;
        LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3941788878L);
            assert(pack.last_change_ms_GET() == 399796020L);
            assert(pack.state_GET() == (char)60);
        });
        DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.time_boot_ms_SET(3941788878L) ;
        p257.last_change_ms_SET(399796020L) ;
        p257.state_SET((char)60) ;
        LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)176);
            assert(pack.target_system_GET() == (char)163);
            assert(pack.tune_LEN(ph) == 12);
            assert(pack.tune_TRY(ph).equals("coaxsmpghpcz"));
        });
        DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.target_component_SET((char)176) ;
        p258.target_system_SET((char)163) ;
        p258.tune_SET("coaxsmpghpcz", PH) ;
        LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.resolution_v_GET() == (char)17772);
            assert(pack.cam_definition_version_GET() == (char)14945);
            assert(pack.firmware_version_GET() == 3240591260L);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)201, (char)147, (char)144, (char)120, (char)201, (char)174, (char)236, (char)44, (char)3, (char)150, (char)83, (char)50, (char)204, (char)8, (char)230, (char)214, (char)11, (char)133, (char)249, (char)210, (char)249, (char)8, (char)116, (char)209, (char)227, (char)31, (char)238, (char)177, (char)68, (char)113, (char)32, (char)24}));
            assert(pack.lens_id_GET() == (char)190);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)159, (char)209, (char)110, (char)212, (char)153, (char)10, (char)219, (char)108, (char)133, (char)87, (char)48, (char)225, (char)57, (char)64, (char)163, (char)152, (char)168, (char)8, (char)0, (char)209, (char)114, (char)245, (char)51, (char)224, (char)152, (char)42, (char)16, (char)34, (char)206, (char)204, (char)185, (char)95}));
            assert(pack.flags_GET() == CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE);
            assert(pack.cam_definition_uri_LEN(ph) == 118);
            assert(pack.cam_definition_uri_TRY(ph).equals("dfUBpsvyqngknMvdptbrfponqbgxvanxpgvppmxpxrxfhfjvkhkUkguupbemnagzfKpskkxRbxkurdVpknnciNnjcjCjjoaepkXiytooWdooNjviylrmha"));
            assert(pack.time_boot_ms_GET() == 2438421983L);
            assert(pack.sensor_size_h_GET() == -7.091457E36F);
            assert(pack.sensor_size_v_GET() == -7.404778E37F);
            assert(pack.focal_length_GET() == 1.5182689E38F);
            assert(pack.resolution_h_GET() == (char)37763);
        });
        DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE) ;
        p259.resolution_h_SET((char)37763) ;
        p259.cam_definition_version_SET((char)14945) ;
        p259.model_name_SET(new char[] {(char)159, (char)209, (char)110, (char)212, (char)153, (char)10, (char)219, (char)108, (char)133, (char)87, (char)48, (char)225, (char)57, (char)64, (char)163, (char)152, (char)168, (char)8, (char)0, (char)209, (char)114, (char)245, (char)51, (char)224, (char)152, (char)42, (char)16, (char)34, (char)206, (char)204, (char)185, (char)95}, 0) ;
        p259.lens_id_SET((char)190) ;
        p259.cam_definition_uri_SET("dfUBpsvyqngknMvdptbrfponqbgxvanxpgvppmxpxrxfhfjvkhkUkguupbemnagzfKpskkxRbxkurdVpknnciNnjcjCjjoaepkXiytooWdooNjviylrmha", PH) ;
        p259.focal_length_SET(1.5182689E38F) ;
        p259.sensor_size_h_SET(-7.091457E36F) ;
        p259.firmware_version_SET(3240591260L) ;
        p259.time_boot_ms_SET(2438421983L) ;
        p259.resolution_v_SET((char)17772) ;
        p259.vendor_name_SET(new char[] {(char)201, (char)147, (char)144, (char)120, (char)201, (char)174, (char)236, (char)44, (char)3, (char)150, (char)83, (char)50, (char)204, (char)8, (char)230, (char)214, (char)11, (char)133, (char)249, (char)210, (char)249, (char)8, (char)116, (char)209, (char)227, (char)31, (char)238, (char)177, (char)68, (char)113, (char)32, (char)24}, 0) ;
        p259.sensor_size_v_SET(-7.404778E37F) ;
        LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_VIDEO);
            assert(pack.time_boot_ms_GET() == 2202000429L);
        });
        DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_VIDEO) ;
        p260.time_boot_ms_SET(2202000429L) ;
        LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.storage_id_GET() == (char)230);
            assert(pack.time_boot_ms_GET() == 3108878518L);
            assert(pack.available_capacity_GET() == 1.6800207E38F);
            assert(pack.write_speed_GET() == -3.2479585E38F);
            assert(pack.status_GET() == (char)46);
            assert(pack.total_capacity_GET() == -2.7761258E38F);
            assert(pack.used_capacity_GET() == 1.2632608E38F);
            assert(pack.read_speed_GET() == -7.5076805E37F);
            assert(pack.storage_count_GET() == (char)229);
        });
        DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.time_boot_ms_SET(3108878518L) ;
        p261.available_capacity_SET(1.6800207E38F) ;
        p261.total_capacity_SET(-2.7761258E38F) ;
        p261.storage_id_SET((char)230) ;
        p261.status_SET((char)46) ;
        p261.read_speed_SET(-7.5076805E37F) ;
        p261.write_speed_SET(-3.2479585E38F) ;
        p261.used_capacity_SET(1.2632608E38F) ;
        p261.storage_count_SET((char)229) ;
        LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.image_interval_GET() == -1.3112747E38F);
            assert(pack.video_status_GET() == (char)237);
            assert(pack.image_status_GET() == (char)45);
            assert(pack.recording_time_ms_GET() == 3184311963L);
            assert(pack.time_boot_ms_GET() == 3191186052L);
            assert(pack.available_capacity_GET() == -2.1442655E37F);
        });
        DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.image_interval_SET(-1.3112747E38F) ;
        p262.image_status_SET((char)45) ;
        p262.video_status_SET((char)237) ;
        p262.recording_time_ms_SET(3184311963L) ;
        p262.time_boot_ms_SET(3191186052L) ;
        p262.available_capacity_SET(-2.1442655E37F) ;
        LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.q_GET(),  new float[] {-7.222496E37F, -3.125761E38F, 3.32127E38F, -1.3587719E38F}));
            assert(pack.capture_result_GET() == (byte)48);
            assert(pack.relative_alt_GET() == -1752247079);
            assert(pack.alt_GET() == -1777353877);
            assert(pack.time_boot_ms_GET() == 2200989159L);
            assert(pack.lon_GET() == -1075284320);
            assert(pack.image_index_GET() == 1749954551);
            assert(pack.file_url_LEN(ph) == 9);
            assert(pack.file_url_TRY(ph).equals("zFobaaJgj"));
            assert(pack.time_utc_GET() == 5278052899831191133L);
            assert(pack.lat_GET() == -904255327);
            assert(pack.camera_id_GET() == (char)12);
        });
        DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.image_index_SET(1749954551) ;
        p263.q_SET(new float[] {-7.222496E37F, -3.125761E38F, 3.32127E38F, -1.3587719E38F}, 0) ;
        p263.relative_alt_SET(-1752247079) ;
        p263.time_boot_ms_SET(2200989159L) ;
        p263.capture_result_SET((byte)48) ;
        p263.lon_SET(-1075284320) ;
        p263.time_utc_SET(5278052899831191133L) ;
        p263.file_url_SET("zFobaaJgj", PH) ;
        p263.alt_SET(-1777353877) ;
        p263.camera_id_SET((char)12) ;
        p263.lat_SET(-904255327) ;
        LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3258608922L);
            assert(pack.takeoff_time_utc_GET() == 4329950523814478358L);
            assert(pack.flight_uuid_GET() == 4237973854340348541L);
            assert(pack.arming_time_utc_GET() == 2553753961515672657L);
        });
        DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.takeoff_time_utc_SET(4329950523814478358L) ;
        p264.time_boot_ms_SET(3258608922L) ;
        p264.flight_uuid_SET(4237973854340348541L) ;
        p264.arming_time_utc_SET(2553753961515672657L) ;
        LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 2.9261154E38F);
            assert(pack.time_boot_ms_GET() == 2303848219L);
            assert(pack.yaw_GET() == 1.8092177E37F);
            assert(pack.roll_GET() == -1.971815E38F);
        });
        DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.roll_SET(-1.971815E38F) ;
        p265.yaw_SET(1.8092177E37F) ;
        p265.time_boot_ms_SET(2303848219L) ;
        p265.pitch_SET(2.9261154E38F) ;
        LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.first_message_offset_GET() == (char)212);
            assert(pack.target_component_GET() == (char)211);
            assert(pack.target_system_GET() == (char)31);
            assert(pack.length_GET() == (char)109);
            assert(pack.sequence_GET() == (char)51042);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)73, (char)220, (char)23, (char)24, (char)163, (char)61, (char)152, (char)248, (char)126, (char)197, (char)87, (char)71, (char)181, (char)13, (char)218, (char)170, (char)5, (char)76, (char)167, (char)203, (char)171, (char)242, (char)189, (char)29, (char)83, (char)174, (char)160, (char)114, (char)193, (char)15, (char)220, (char)120, (char)184, (char)134, (char)203, (char)42, (char)177, (char)108, (char)172, (char)39, (char)1, (char)207, (char)245, (char)104, (char)49, (char)249, (char)253, (char)158, (char)176, (char)31, (char)220, (char)236, (char)90, (char)178, (char)120, (char)123, (char)125, (char)23, (char)115, (char)26, (char)192, (char)109, (char)215, (char)218, (char)113, (char)224, (char)71, (char)22, (char)150, (char)5, (char)98, (char)52, (char)108, (char)31, (char)210, (char)150, (char)226, (char)142, (char)88, (char)31, (char)234, (char)152, (char)217, (char)153, (char)70, (char)166, (char)129, (char)158, (char)128, (char)116, (char)204, (char)52, (char)149, (char)163, (char)1, (char)77, (char)0, (char)218, (char)133, (char)110, (char)133, (char)191, (char)15, (char)228, (char)69, (char)251, (char)143, (char)83, (char)255, (char)204, (char)25, (char)180, (char)130, (char)29, (char)13, (char)145, (char)40, (char)125, (char)127, (char)203, (char)84, (char)54, (char)218, (char)74, (char)141, (char)157, (char)100, (char)54, (char)75, (char)76, (char)51, (char)117, (char)94, (char)73, (char)36, (char)114, (char)197, (char)98, (char)192, (char)66, (char)61, (char)144, (char)16, (char)2, (char)217, (char)125, (char)202, (char)167, (char)227, (char)211, (char)72, (char)132, (char)213, (char)58, (char)182, (char)95, (char)93, (char)53, (char)103, (char)255, (char)172, (char)174, (char)233, (char)231, (char)5, (char)111, (char)80, (char)6, (char)89, (char)1, (char)199, (char)198, (char)88, (char)183, (char)251, (char)73, (char)162, (char)91, (char)220, (char)100, (char)168, (char)215, (char)165, (char)113, (char)10, (char)67, (char)253, (char)25, (char)103, (char)137, (char)48, (char)21, (char)49, (char)221, (char)125, (char)16, (char)118, (char)170, (char)166, (char)200, (char)137, (char)95, (char)202, (char)20, (char)96, (char)125, (char)204, (char)237, (char)87, (char)203, (char)88, (char)110, (char)38, (char)84, (char)94, (char)99, (char)21, (char)140, (char)67, (char)163, (char)12, (char)30, (char)14, (char)224, (char)61, (char)11, (char)45, (char)69, (char)84, (char)110, (char)206, (char)0, (char)2, (char)118, (char)197, (char)253, (char)4, (char)223, (char)221, (char)191, (char)204, (char)89, (char)158, (char)87, (char)184, (char)125, (char)65, (char)117, (char)82}));
        });
        DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.sequence_SET((char)51042) ;
        p266.data__SET(new char[] {(char)73, (char)220, (char)23, (char)24, (char)163, (char)61, (char)152, (char)248, (char)126, (char)197, (char)87, (char)71, (char)181, (char)13, (char)218, (char)170, (char)5, (char)76, (char)167, (char)203, (char)171, (char)242, (char)189, (char)29, (char)83, (char)174, (char)160, (char)114, (char)193, (char)15, (char)220, (char)120, (char)184, (char)134, (char)203, (char)42, (char)177, (char)108, (char)172, (char)39, (char)1, (char)207, (char)245, (char)104, (char)49, (char)249, (char)253, (char)158, (char)176, (char)31, (char)220, (char)236, (char)90, (char)178, (char)120, (char)123, (char)125, (char)23, (char)115, (char)26, (char)192, (char)109, (char)215, (char)218, (char)113, (char)224, (char)71, (char)22, (char)150, (char)5, (char)98, (char)52, (char)108, (char)31, (char)210, (char)150, (char)226, (char)142, (char)88, (char)31, (char)234, (char)152, (char)217, (char)153, (char)70, (char)166, (char)129, (char)158, (char)128, (char)116, (char)204, (char)52, (char)149, (char)163, (char)1, (char)77, (char)0, (char)218, (char)133, (char)110, (char)133, (char)191, (char)15, (char)228, (char)69, (char)251, (char)143, (char)83, (char)255, (char)204, (char)25, (char)180, (char)130, (char)29, (char)13, (char)145, (char)40, (char)125, (char)127, (char)203, (char)84, (char)54, (char)218, (char)74, (char)141, (char)157, (char)100, (char)54, (char)75, (char)76, (char)51, (char)117, (char)94, (char)73, (char)36, (char)114, (char)197, (char)98, (char)192, (char)66, (char)61, (char)144, (char)16, (char)2, (char)217, (char)125, (char)202, (char)167, (char)227, (char)211, (char)72, (char)132, (char)213, (char)58, (char)182, (char)95, (char)93, (char)53, (char)103, (char)255, (char)172, (char)174, (char)233, (char)231, (char)5, (char)111, (char)80, (char)6, (char)89, (char)1, (char)199, (char)198, (char)88, (char)183, (char)251, (char)73, (char)162, (char)91, (char)220, (char)100, (char)168, (char)215, (char)165, (char)113, (char)10, (char)67, (char)253, (char)25, (char)103, (char)137, (char)48, (char)21, (char)49, (char)221, (char)125, (char)16, (char)118, (char)170, (char)166, (char)200, (char)137, (char)95, (char)202, (char)20, (char)96, (char)125, (char)204, (char)237, (char)87, (char)203, (char)88, (char)110, (char)38, (char)84, (char)94, (char)99, (char)21, (char)140, (char)67, (char)163, (char)12, (char)30, (char)14, (char)224, (char)61, (char)11, (char)45, (char)69, (char)84, (char)110, (char)206, (char)0, (char)2, (char)118, (char)197, (char)253, (char)4, (char)223, (char)221, (char)191, (char)204, (char)89, (char)158, (char)87, (char)184, (char)125, (char)65, (char)117, (char)82}, 0) ;
        p266.target_system_SET((char)31) ;
        p266.first_message_offset_SET((char)212) ;
        p266.target_component_SET((char)211) ;
        p266.length_SET((char)109) ;
        LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)239, (char)147, (char)148, (char)77, (char)194, (char)34, (char)19, (char)247, (char)137, (char)118, (char)132, (char)131, (char)185, (char)179, (char)155, (char)162, (char)123, (char)216, (char)90, (char)66, (char)239, (char)15, (char)138, (char)52, (char)106, (char)96, (char)220, (char)15, (char)19, (char)239, (char)56, (char)57, (char)18, (char)89, (char)35, (char)252, (char)54, (char)210, (char)5, (char)182, (char)47, (char)245, (char)48, (char)132, (char)47, (char)137, (char)247, (char)27, (char)42, (char)52, (char)239, (char)232, (char)18, (char)119, (char)156, (char)214, (char)4, (char)3, (char)221, (char)246, (char)117, (char)96, (char)163, (char)177, (char)46, (char)127, (char)205, (char)103, (char)149, (char)25, (char)179, (char)11, (char)139, (char)1, (char)50, (char)54, (char)195, (char)251, (char)196, (char)151, (char)183, (char)146, (char)7, (char)254, (char)158, (char)62, (char)182, (char)172, (char)72, (char)220, (char)90, (char)42, (char)218, (char)203, (char)152, (char)193, (char)90, (char)14, (char)245, (char)0, (char)28, (char)81, (char)156, (char)28, (char)130, (char)212, (char)60, (char)231, (char)111, (char)121, (char)115, (char)139, (char)47, (char)98, (char)142, (char)44, (char)12, (char)223, (char)162, (char)229, (char)55, (char)145, (char)143, (char)64, (char)102, (char)225, (char)246, (char)27, (char)191, (char)36, (char)182, (char)204, (char)72, (char)163, (char)164, (char)100, (char)76, (char)53, (char)206, (char)248, (char)95, (char)225, (char)161, (char)108, (char)120, (char)134, (char)231, (char)83, (char)87, (char)209, (char)75, (char)102, (char)61, (char)139, (char)250, (char)0, (char)174, (char)84, (char)52, (char)2, (char)64, (char)135, (char)83, (char)127, (char)101, (char)0, (char)20, (char)233, (char)192, (char)240, (char)203, (char)240, (char)92, (char)93, (char)0, (char)111, (char)71, (char)150, (char)72, (char)51, (char)52, (char)67, (char)219, (char)102, (char)167, (char)247, (char)57, (char)218, (char)152, (char)233, (char)78, (char)21, (char)151, (char)24, (char)20, (char)125, (char)96, (char)78, (char)112, (char)105, (char)80, (char)49, (char)66, (char)180, (char)147, (char)86, (char)93, (char)56, (char)185, (char)123, (char)163, (char)25, (char)126, (char)128, (char)173, (char)13, (char)136, (char)236, (char)123, (char)96, (char)212, (char)206, (char)67, (char)51, (char)40, (char)124, (char)168, (char)164, (char)49, (char)109, (char)143, (char)247, (char)137, (char)177, (char)90, (char)208, (char)100, (char)217, (char)24, (char)114, (char)122, (char)35, (char)199, (char)4, (char)38, (char)13, (char)194, (char)140, (char)164}));
            assert(pack.target_component_GET() == (char)128);
            assert(pack.first_message_offset_GET() == (char)127);
            assert(pack.length_GET() == (char)110);
            assert(pack.target_system_GET() == (char)99);
            assert(pack.sequence_GET() == (char)40261);
        });
        DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.first_message_offset_SET((char)127) ;
        p267.length_SET((char)110) ;
        p267.target_component_SET((char)128) ;
        p267.data__SET(new char[] {(char)239, (char)147, (char)148, (char)77, (char)194, (char)34, (char)19, (char)247, (char)137, (char)118, (char)132, (char)131, (char)185, (char)179, (char)155, (char)162, (char)123, (char)216, (char)90, (char)66, (char)239, (char)15, (char)138, (char)52, (char)106, (char)96, (char)220, (char)15, (char)19, (char)239, (char)56, (char)57, (char)18, (char)89, (char)35, (char)252, (char)54, (char)210, (char)5, (char)182, (char)47, (char)245, (char)48, (char)132, (char)47, (char)137, (char)247, (char)27, (char)42, (char)52, (char)239, (char)232, (char)18, (char)119, (char)156, (char)214, (char)4, (char)3, (char)221, (char)246, (char)117, (char)96, (char)163, (char)177, (char)46, (char)127, (char)205, (char)103, (char)149, (char)25, (char)179, (char)11, (char)139, (char)1, (char)50, (char)54, (char)195, (char)251, (char)196, (char)151, (char)183, (char)146, (char)7, (char)254, (char)158, (char)62, (char)182, (char)172, (char)72, (char)220, (char)90, (char)42, (char)218, (char)203, (char)152, (char)193, (char)90, (char)14, (char)245, (char)0, (char)28, (char)81, (char)156, (char)28, (char)130, (char)212, (char)60, (char)231, (char)111, (char)121, (char)115, (char)139, (char)47, (char)98, (char)142, (char)44, (char)12, (char)223, (char)162, (char)229, (char)55, (char)145, (char)143, (char)64, (char)102, (char)225, (char)246, (char)27, (char)191, (char)36, (char)182, (char)204, (char)72, (char)163, (char)164, (char)100, (char)76, (char)53, (char)206, (char)248, (char)95, (char)225, (char)161, (char)108, (char)120, (char)134, (char)231, (char)83, (char)87, (char)209, (char)75, (char)102, (char)61, (char)139, (char)250, (char)0, (char)174, (char)84, (char)52, (char)2, (char)64, (char)135, (char)83, (char)127, (char)101, (char)0, (char)20, (char)233, (char)192, (char)240, (char)203, (char)240, (char)92, (char)93, (char)0, (char)111, (char)71, (char)150, (char)72, (char)51, (char)52, (char)67, (char)219, (char)102, (char)167, (char)247, (char)57, (char)218, (char)152, (char)233, (char)78, (char)21, (char)151, (char)24, (char)20, (char)125, (char)96, (char)78, (char)112, (char)105, (char)80, (char)49, (char)66, (char)180, (char)147, (char)86, (char)93, (char)56, (char)185, (char)123, (char)163, (char)25, (char)126, (char)128, (char)173, (char)13, (char)136, (char)236, (char)123, (char)96, (char)212, (char)206, (char)67, (char)51, (char)40, (char)124, (char)168, (char)164, (char)49, (char)109, (char)143, (char)247, (char)137, (char)177, (char)90, (char)208, (char)100, (char)217, (char)24, (char)114, (char)122, (char)35, (char)199, (char)4, (char)38, (char)13, (char)194, (char)140, (char)164}, 0) ;
        p267.target_system_SET((char)99) ;
        p267.sequence_SET((char)40261) ;
        LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)7460);
            assert(pack.target_component_GET() == (char)245);
            assert(pack.target_system_GET() == (char)100);
        });
        DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)100) ;
        p268.target_component_SET((char)245) ;
        p268.sequence_SET((char)7460) ;
        LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.uri_LEN(ph) == 158);
            assert(pack.uri_TRY(ph).equals("psumhRilgpcWUqpcjbpekwylscptlbdNbqgxbrfbjiicramiwfysszoGxRhhdphokyigfoummeAYagihudllvzisyxHqirctNstzlGhsEljfnHoCecazfbqclwedqNeobmlafverjiisiAtiguuzktggvinnJt"));
            assert(pack.status_GET() == (char)66);
            assert(pack.resolution_h_GET() == (char)20288);
            assert(pack.bitrate_GET() == 1974640573L);
            assert(pack.resolution_v_GET() == (char)2433);
            assert(pack.camera_id_GET() == (char)188);
            assert(pack.rotation_GET() == (char)43607);
            assert(pack.framerate_GET() == -1.7215107E38F);
        });
        DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.bitrate_SET(1974640573L) ;
        p269.status_SET((char)66) ;
        p269.resolution_v_SET((char)2433) ;
        p269.resolution_h_SET((char)20288) ;
        p269.framerate_SET(-1.7215107E38F) ;
        p269.uri_SET("psumhRilgpcWUqpcjbpekwylscptlbdNbqgxbrfbjiicramiwfysszoGxRhhdphokyigfoummeAYagihudllvzisyxHqirctNstzlGhsEljfnHoCecazfbqclwedqNeobmlafverjiisiAtiguuzktggvinnJt", PH) ;
        p269.camera_id_SET((char)188) ;
        p269.rotation_SET((char)43607) ;
        LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.camera_id_GET() == (char)201);
            assert(pack.rotation_GET() == (char)48259);
            assert(pack.resolution_v_GET() == (char)52297);
            assert(pack.target_system_GET() == (char)22);
            assert(pack.framerate_GET() == 3.012108E38F);
            assert(pack.uri_LEN(ph) == 4);
            assert(pack.uri_TRY(ph).equals("rdcq"));
            assert(pack.resolution_h_GET() == (char)59283);
            assert(pack.target_component_GET() == (char)38);
            assert(pack.bitrate_GET() == 698461972L);
        });
        DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.resolution_h_SET((char)59283) ;
        p270.target_component_SET((char)38) ;
        p270.resolution_v_SET((char)52297) ;
        p270.target_system_SET((char)22) ;
        p270.camera_id_SET((char)201) ;
        p270.framerate_SET(3.012108E38F) ;
        p270.bitrate_SET(698461972L) ;
        p270.rotation_SET((char)48259) ;
        p270.uri_SET("rdcq", PH) ;
        LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.ssid_LEN(ph) == 13);
            assert(pack.ssid_TRY(ph).equals("aovqgltKffkbX"));
            assert(pack.password_LEN(ph) == 45);
            assert(pack.password_TRY(ph).equals("zkxvgTkTsnarckldZllQwelknlndhkrkxxvpuaXedmjme"));
        });
        DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.ssid_SET("aovqgltKffkbX", PH) ;
        p299.password_SET("zkxvgTkTsnarckldZllQwelknlndhkrkxxvpuaXedmjme", PH) ;
        LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(pack.max_version_GET() == (char)55303);
            assert(pack.version_GET() == (char)42911);
            assert(pack.min_version_GET() == (char)30239);
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)148, (char)173, (char)219, (char)185, (char)119, (char)149, (char)146, (char)68}));
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)206, (char)115, (char)108, (char)83, (char)93, (char)30, (char)151, (char)93}));
        });
        DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.library_version_hash_SET(new char[] {(char)148, (char)173, (char)219, (char)185, (char)119, (char)149, (char)146, (char)68}, 0) ;
        p300.version_SET((char)42911) ;
        p300.min_version_SET((char)30239) ;
        p300.spec_version_hash_SET(new char[] {(char)206, (char)115, (char)108, (char)83, (char)93, (char)30, (char)151, (char)93}, 0) ;
        p300.max_version_SET((char)55303) ;
        LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING);
            assert(pack.sub_mode_GET() == (char)205);
            assert(pack.vendor_specific_status_code_GET() == (char)47004);
            assert(pack.time_usec_GET() == 6799107126650236257L);
            assert(pack.uptime_sec_GET() == 2429230019L);
        });
        DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_WARNING) ;
        p310.time_usec_SET(6799107126650236257L) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_SOFTWARE_UPDATE) ;
        p310.vendor_specific_status_code_SET((char)47004) ;
        p310.uptime_sec_SET(2429230019L) ;
        p310.sub_mode_SET((char)205) ;
        LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)226, (char)188, (char)103, (char)84, (char)97, (char)51, (char)178, (char)107, (char)206, (char)91, (char)20, (char)83, (char)191, (char)96, (char)188, (char)117}));
            assert(pack.hw_version_minor_GET() == (char)14);
            assert(pack.time_usec_GET() == 2449366577626671521L);
            assert(pack.sw_vcs_commit_GET() == 305571967L);
            assert(pack.sw_version_major_GET() == (char)45);
            assert(pack.uptime_sec_GET() == 3046438971L);
            assert(pack.name_LEN(ph) == 11);
            assert(pack.name_TRY(ph).equals("xvnpwRqaedr"));
            assert(pack.sw_version_minor_GET() == (char)177);
            assert(pack.hw_version_major_GET() == (char)203);
        });
        DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.name_SET("xvnpwRqaedr", PH) ;
        p311.hw_unique_id_SET(new char[] {(char)226, (char)188, (char)103, (char)84, (char)97, (char)51, (char)178, (char)107, (char)206, (char)91, (char)20, (char)83, (char)191, (char)96, (char)188, (char)117}, 0) ;
        p311.time_usec_SET(2449366577626671521L) ;
        p311.hw_version_minor_SET((char)14) ;
        p311.sw_version_minor_SET((char)177) ;
        p311.hw_version_major_SET((char)203) ;
        p311.sw_vcs_commit_SET(305571967L) ;
        p311.sw_version_major_SET((char)45) ;
        p311.uptime_sec_SET(3046438971L) ;
        LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 16);
            assert(pack.param_id_TRY(ph).equals("uhWovakibhngtzBe"));
            assert(pack.target_system_GET() == (char)48);
            assert(pack.param_index_GET() == (short) -18860);
            assert(pack.target_component_GET() == (char)31);
        });
        DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.param_id_SET("uhWovakibhngtzBe", PH) ;
        p320.target_component_SET((char)31) ;
        p320.target_system_SET((char)48) ;
        p320.param_index_SET((short) -18860) ;
        LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)214);
            assert(pack.target_system_GET() == (char)146);
        });
        DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_system_SET((char)146) ;
        p321.target_component_SET((char)214) ;
        LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 127);
            assert(pack.param_value_TRY(ph).equals("nrkklmdanFmyHgqwtqeopeziavkTOnykqrBamevfooOfximtwsMaegvllveetmmTtSvgSpjjuuzgguoiCoxjfWLvtVyurwMyzvgigyrufYASzlrxtkxaebscaitjSDw"));
            assert(pack.param_index_GET() == (char)37749);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16);
            assert(pack.param_count_GET() == (char)5433);
            assert(pack.param_id_LEN(ph) == 14);
            assert(pack.param_id_TRY(ph).equals("vmoonhsmxCxhga"));
        });
        DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT16) ;
        p322.param_index_SET((char)37749) ;
        p322.param_value_SET("nrkklmdanFmyHgqwtqeopeziavkTOnykqrBamevfooOfximtwsMaegvllveetmmTtSvgSpjjuuzgguoiCoxjfWLvtVyurwMyzvgigyrufYASzlrxtkxaebscaitjSDw", PH) ;
        p322.param_count_SET((char)5433) ;
        p322.param_id_SET("vmoonhsmxCxhga", PH) ;
        LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.param_value_LEN(ph) == 84);
            assert(pack.param_value_TRY(ph).equals("xbuiysyMsjxmbpxwaanhjfgTSsxxrpcsRqlwxjbclqdaubeuPnwYqagjJmkjrbmrvfWorjzxgttmfbcnkfvw"));
            assert(pack.param_id_LEN(ph) == 6);
            assert(pack.param_id_TRY(ph).equals("wstohV"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16);
            assert(pack.target_system_GET() == (char)116);
            assert(pack.target_component_GET() == (char)81);
        });
        DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_component_SET((char)81) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT16) ;
        p323.param_id_SET("wstohV", PH) ;
        p323.target_system_SET((char)116) ;
        p323.param_value_SET("xbuiysyMsjxmbpxwaanhjfgTSsxxrpcsRqlwxjbclqdaubeuPnwYqagjJmkjrbmrvfWorjzxgttmfbcnkfvw", PH) ;
        LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
            assert(pack.param_id_LEN(ph) == 13);
            assert(pack.param_id_TRY(ph).equals("lakzoxlajCnpy"));
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_FAILED);
            assert(pack.param_value_LEN(ph) == 8);
            assert(pack.param_value_TRY(ph).equals("wdmuxnlo"));
        });
        DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8) ;
        p324.param_id_SET("lakzoxlajCnpy", PH) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_FAILED) ;
        p324.param_value_SET("wdmuxnlo", PH) ;
        LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.min_distance_GET() == (char)65299);
            assert(pack.increment_GET() == (char)65);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)12042, (char)9348, (char)12154, (char)63636, (char)30418, (char)21926, (char)26631, (char)17225, (char)4438, (char)36319, (char)30855, (char)59913, (char)14673, (char)42653, (char)42098, (char)5979, (char)62039, (char)23892, (char)60413, (char)59083, (char)33744, (char)18096, (char)34319, (char)55402, (char)42481, (char)8558, (char)3672, (char)39377, (char)29770, (char)56328, (char)27357, (char)23755, (char)6806, (char)45446, (char)4871, (char)50831, (char)51134, (char)53590, (char)12887, (char)30351, (char)15169, (char)28521, (char)6291, (char)22617, (char)23011, (char)56928, (char)55606, (char)43198, (char)30997, (char)46423, (char)2522, (char)8502, (char)27131, (char)19882, (char)23085, (char)48995, (char)49724, (char)35963, (char)30586, (char)57420, (char)19876, (char)6470, (char)40265, (char)24134, (char)20514, (char)27932, (char)9553, (char)12928, (char)38693, (char)51106, (char)52160, (char)10394}));
            assert(pack.max_distance_GET() == (char)10795);
            assert(pack.time_usec_GET() == 777964684834370166L);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
        });
        DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.distances_SET(new char[] {(char)12042, (char)9348, (char)12154, (char)63636, (char)30418, (char)21926, (char)26631, (char)17225, (char)4438, (char)36319, (char)30855, (char)59913, (char)14673, (char)42653, (char)42098, (char)5979, (char)62039, (char)23892, (char)60413, (char)59083, (char)33744, (char)18096, (char)34319, (char)55402, (char)42481, (char)8558, (char)3672, (char)39377, (char)29770, (char)56328, (char)27357, (char)23755, (char)6806, (char)45446, (char)4871, (char)50831, (char)51134, (char)53590, (char)12887, (char)30351, (char)15169, (char)28521, (char)6291, (char)22617, (char)23011, (char)56928, (char)55606, (char)43198, (char)30997, (char)46423, (char)2522, (char)8502, (char)27131, (char)19882, (char)23085, (char)48995, (char)49724, (char)35963, (char)30586, (char)57420, (char)19876, (char)6470, (char)40265, (char)24134, (char)20514, (char)27932, (char)9553, (char)12928, (char)38693, (char)51106, (char)52160, (char)10394}, 0) ;
        p330.increment_SET((char)65) ;
        p330.max_distance_SET((char)10795) ;
        p330.time_usec_SET(777964684834370166L) ;
        p330.min_distance_SET((char)65299) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR) ;
        LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
    }

}