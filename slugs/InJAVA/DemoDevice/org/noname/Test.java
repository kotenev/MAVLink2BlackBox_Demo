
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
            assert(pack.type_GET() == MAV_TYPE.MAV_TYPE_ONBOARD_CONTROLLER);
            assert(pack.custom_mode_GET() == 2360489556L);
            assert(pack.system_status_GET() == MAV_STATE.MAV_STATE_CALIBRATING);
            assert(pack.mavlink_version_GET() == (char)80);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED);
            assert(pack.autopilot_GET() == MAV_AUTOPILOT.MAV_AUTOPILOT_OPENPILOT);
        });
        DemoDevice.HEARTBEAT p0 = LoopBackDemoChannel.new_HEARTBEAT();
        PH.setPack(p0);
        p0.custom_mode_SET(2360489556L) ;
        p0.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED) ;
        p0.system_status_SET(MAV_STATE.MAV_STATE_CALIBRATING) ;
        p0.type_SET(MAV_TYPE.MAV_TYPE_ONBOARD_CONTROLLER) ;
        p0.mavlink_version_SET((char)80) ;
        p0.autopilot_SET(MAV_AUTOPILOT.MAV_AUTOPILOT_OPENPILOT) ;
        LoopBackDemoChannel.instance.send(p0);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYS_STATUS.add((src, ph, pack) ->
        {
            assert(pack.errors_count2_GET() == (char)45280);
            assert(pack.current_battery_GET() == (short)16899);
            assert(pack.voltage_battery_GET() == (char)48848);
            assert(pack.onboard_control_sensors_health_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR);
            assert(pack.errors_count1_GET() == (char)54699);
            assert(pack.load_GET() == (char)26843);
            assert(pack.onboard_control_sensors_enabled_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION);
            assert(pack.onboard_control_sensors_present_GET() == MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY);
            assert(pack.errors_count3_GET() == (char)59052);
            assert(pack.errors_comm_GET() == (char)25506);
            assert(pack.errors_count4_GET() == (char)32693);
            assert(pack.battery_remaining_GET() == (byte)29);
            assert(pack.drop_rate_comm_GET() == (char)56987);
        });
        DemoDevice.SYS_STATUS p1 = LoopBackDemoChannel.new_SYS_STATUS();
        PH.setPack(p1);
        p1.errors_count4_SET((char)32693) ;
        p1.battery_remaining_SET((byte)29) ;
        p1.onboard_control_sensors_present_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_BATTERY) ;
        p1.onboard_control_sensors_health_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_REVERSE_MOTOR) ;
        p1.errors_comm_SET((char)25506) ;
        p1.errors_count3_SET((char)59052) ;
        p1.errors_count2_SET((char)45280) ;
        p1.voltage_battery_SET((char)48848) ;
        p1.current_battery_SET((short)16899) ;
        p1.drop_rate_comm_SET((char)56987) ;
        p1.onboard_control_sensors_enabled_SET(MAV_SYS_STATUS_SENSOR.MAV_SYS_STATUS_SENSOR_VISION_POSITION) ;
        p1.errors_count1_SET((char)54699) ;
        p1.load_SET((char)26843) ;
        LoopBackDemoChannel.instance.send(p1);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SYSTEM_TIME.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 535887935L);
            assert(pack.time_unix_usec_GET() == 1002851252872246024L);
        });
        DemoDevice.SYSTEM_TIME p2 = LoopBackDemoChannel.new_SYSTEM_TIME();
        PH.setPack(p2);
        p2.time_unix_usec_SET(1002851252872246024L) ;
        p2.time_boot_ms_SET(535887935L) ;
        LoopBackDemoChannel.instance.send(p2);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.afy_GET() == -5.1379E36F);
            assert(pack.time_boot_ms_GET() == 697978508L);
            assert(pack.x_GET() == -2.1924695E38F);
            assert(pack.y_GET() == -2.3372033E38F);
            assert(pack.yaw_GET() == 1.1680623E38F);
            assert(pack.vx_GET() == -2.2831843E37F);
            assert(pack.afx_GET() == -1.1258786E37F);
            assert(pack.yaw_rate_GET() == 2.0857221E37F);
            assert(pack.type_mask_GET() == (char)24400);
            assert(pack.z_GET() == -1.6074336E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.vz_GET() == 1.1364556E38F);
            assert(pack.afz_GET() == -2.4730306E38F);
            assert(pack.vy_GET() == -4.6597065E36F);
        });
        DemoDevice.POSITION_TARGET_LOCAL_NED p3 = LoopBackDemoChannel.new_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p3);
        p3.afy_SET(-5.1379E36F) ;
        p3.vz_SET(1.1364556E38F) ;
        p3.y_SET(-2.3372033E38F) ;
        p3.afz_SET(-2.4730306E38F) ;
        p3.yaw_rate_SET(2.0857221E37F) ;
        p3.time_boot_ms_SET(697978508L) ;
        p3.yaw_SET(1.1680623E38F) ;
        p3.vy_SET(-4.6597065E36F) ;
        p3.x_SET(-2.1924695E38F) ;
        p3.afx_SET(-1.1258786E37F) ;
        p3.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p3.z_SET(-1.6074336E38F) ;
        p3.vx_SET(-2.2831843E37F) ;
        p3.type_mask_SET((char)24400) ;
        LoopBackDemoChannel.instance.send(p3);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PING.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == 3138263510L);
            assert(pack.time_usec_GET() == 3835687370668589813L);
            assert(pack.target_system_GET() == (char)173);
            assert(pack.target_component_GET() == (char)37);
        });
        DemoDevice.PING p4 = LoopBackDemoChannel.new_PING();
        PH.setPack(p4);
        p4.seq_SET(3138263510L) ;
        p4.target_component_SET((char)37) ;
        p4.time_usec_SET(3835687370668589813L) ;
        p4.target_system_SET((char)173) ;
        LoopBackDemoChannel.instance.send(p4);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.version_GET() == (char)243);
            assert(pack.passkey_LEN(ph) == 1);
            assert(pack.passkey_TRY(ph).equals("k"));
            assert(pack.control_request_GET() == (char)140);
            assert(pack.target_system_GET() == (char)186);
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL p5 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL();
        PH.setPack(p5);
        p5.passkey_SET("k", PH) ;
        p5.version_SET((char)243) ;
        p5.target_system_SET((char)186) ;
        p5.control_request_SET((char)140) ;
        LoopBackDemoChannel.instance.send(p5);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CHANGE_OPERATOR_CONTROL_ACK.add((src, ph, pack) ->
        {
            assert(pack.gcs_system_id_GET() == (char)106);
            assert(pack.control_request_GET() == (char)127);
            assert(pack.ack_GET() == (char)247);
        });
        DemoDevice.CHANGE_OPERATOR_CONTROL_ACK p6 = LoopBackDemoChannel.new_CHANGE_OPERATOR_CONTROL_ACK();
        PH.setPack(p6);
        p6.gcs_system_id_SET((char)106) ;
        p6.control_request_SET((char)127) ;
        p6.ack_SET((char)247) ;
        LoopBackDemoChannel.instance.send(p6);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTH_KEY.add((src, ph, pack) ->
        {
            assert(pack.key_LEN(ph) == 4);
            assert(pack.key_TRY(ph).equals("iLus"));
        });
        DemoDevice.AUTH_KEY p7 = LoopBackDemoChannel.new_AUTH_KEY();
        PH.setPack(p7);
        p7.key_SET("iLus", PH) ;
        LoopBackDemoChannel.instance.send(p7);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_MODE.add((src, ph, pack) ->
        {
            assert(pack.base_mode_GET() == MAV_MODE.MAV_MODE_AUTO_ARMED);
            assert(pack.custom_mode_GET() == 3326064763L);
            assert(pack.target_system_GET() == (char)66);
        });
        DemoDevice.SET_MODE p11 = LoopBackDemoChannel.new_SET_MODE();
        PH.setPack(p11);
        p11.custom_mode_SET(3326064763L) ;
        p11.target_system_SET((char)66) ;
        p11.base_mode_SET(MAV_MODE.MAV_MODE_AUTO_ARMED) ;
        LoopBackDemoChannel.instance.send(p11);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)4);
            assert(pack.param_index_GET() == (short)7792);
            assert(pack.param_id_LEN(ph) == 2);
            assert(pack.param_id_TRY(ph).equals("kd"));
            assert(pack.target_component_GET() == (char)159);
        });
        DemoDevice.PARAM_REQUEST_READ p20 = LoopBackDemoChannel.new_PARAM_REQUEST_READ();
        PH.setPack(p20);
        p20.target_system_SET((char)4) ;
        p20.target_component_SET((char)159) ;
        p20.param_index_SET((short)7792) ;
        p20.param_id_SET("kd", PH) ;
        LoopBackDemoChannel.instance.send(p20);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)21);
            assert(pack.target_component_GET() == (char)32);
        });
        DemoDevice.PARAM_REQUEST_LIST p21 = LoopBackDemoChannel.new_PARAM_REQUEST_LIST();
        PH.setPack(p21);
        p21.target_component_SET((char)32) ;
        p21.target_system_SET((char)21) ;
        LoopBackDemoChannel.instance.send(p21);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8);
            assert(pack.param_count_GET() == (char)9997);
            assert(pack.param_value_GET() == -1.3371064E38F);
            assert(pack.param_id_LEN(ph) == 9);
            assert(pack.param_id_TRY(ph).equals("qmzvffnhm"));
            assert(pack.param_index_GET() == (char)31544);
        });
        DemoDevice.PARAM_VALUE p22 = LoopBackDemoChannel.new_PARAM_VALUE();
        PH.setPack(p22);
        p22.param_id_SET("qmzvffnhm", PH) ;
        p22.param_value_SET(-1.3371064E38F) ;
        p22.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT8) ;
        p22.param_index_SET((char)31544) ;
        p22.param_count_SET((char)9997) ;
        LoopBackDemoChannel.instance.send(p22);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_SET.add((src, ph, pack) ->
        {
            assert(pack.param_value_GET() == -6.7253397E37F);
            assert(pack.param_id_LEN(ph) == 11);
            assert(pack.param_id_TRY(ph).equals("iTrvKacauba"));
            assert(pack.target_system_GET() == (char)166);
            assert(pack.target_component_GET() == (char)184);
            assert(pack.param_type_GET() == MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64);
        });
        DemoDevice.PARAM_SET p23 = LoopBackDemoChannel.new_PARAM_SET();
        PH.setPack(p23);
        p23.param_id_SET("iTrvKacauba", PH) ;
        p23.target_system_SET((char)166) ;
        p23.param_type_SET(MAV_PARAM_TYPE.MAV_PARAM_TYPE_UINT64) ;
        p23.target_component_SET((char)184) ;
        p23.param_value_SET(-6.7253397E37F) ;
        LoopBackDemoChannel.instance.send(p23);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RAW_INT.add((src, ph, pack) ->
        {
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT);
            assert(pack.epv_GET() == (char)44820);
            assert(pack.lon_GET() == -1737545172);
            assert(pack.satellites_visible_GET() == (char)180);
            assert(pack.vel_acc_TRY(ph) == 3725695498L);
            assert(pack.lat_GET() == 1945723704);
            assert(pack.h_acc_TRY(ph) == 932693487L);
            assert(pack.v_acc_TRY(ph) == 2220966318L);
            assert(pack.vel_GET() == (char)1941);
            assert(pack.alt_GET() == 776457787);
            assert(pack.alt_ellipsoid_TRY(ph) == 256466578);
            assert(pack.eph_GET() == (char)64307);
            assert(pack.hdg_acc_TRY(ph) == 2312932658L);
            assert(pack.time_usec_GET() == 426068003389491236L);
            assert(pack.cog_GET() == (char)6571);
        });
        DemoDevice.GPS_RAW_INT p24 = LoopBackDemoChannel.new_GPS_RAW_INT();
        PH.setPack(p24);
        p24.vel_SET((char)1941) ;
        p24.hdg_acc_SET(2312932658L, PH) ;
        p24.satellites_visible_SET((char)180) ;
        p24.time_usec_SET(426068003389491236L) ;
        p24.eph_SET((char)64307) ;
        p24.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FLOAT) ;
        p24.lat_SET(1945723704) ;
        p24.alt_ellipsoid_SET(256466578, PH) ;
        p24.lon_SET(-1737545172) ;
        p24.epv_SET((char)44820) ;
        p24.v_acc_SET(2220966318L, PH) ;
        p24.cog_SET((char)6571) ;
        p24.h_acc_SET(932693487L, PH) ;
        p24.vel_acc_SET(3725695498L, PH) ;
        p24.alt_SET(776457787) ;
        LoopBackDemoChannel.instance.send(p24);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_STATUS.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.satellite_azimuth_GET(),  new char[] {(char)75, (char)186, (char)33, (char)42, (char)205, (char)167, (char)39, (char)99, (char)156, (char)136, (char)45, (char)37, (char)225, (char)135, (char)222, (char)26, (char)147, (char)150, (char)65, (char)150}));
            assert(Arrays.equals(pack.satellite_prn_GET(),  new char[] {(char)128, (char)92, (char)139, (char)45, (char)170, (char)3, (char)137, (char)253, (char)48, (char)108, (char)149, (char)109, (char)198, (char)55, (char)51, (char)146, (char)248, (char)249, (char)159, (char)25}));
            assert(Arrays.equals(pack.satellite_snr_GET(),  new char[] {(char)249, (char)61, (char)157, (char)56, (char)179, (char)87, (char)179, (char)228, (char)107, (char)13, (char)230, (char)72, (char)31, (char)182, (char)179, (char)228, (char)182, (char)220, (char)222, (char)39}));
            assert(Arrays.equals(pack.satellite_elevation_GET(),  new char[] {(char)16, (char)122, (char)1, (char)246, (char)71, (char)65, (char)140, (char)242, (char)58, (char)83, (char)35, (char)71, (char)9, (char)2, (char)175, (char)252, (char)90, (char)30, (char)54, (char)31}));
            assert(Arrays.equals(pack.satellite_used_GET(),  new char[] {(char)179, (char)248, (char)235, (char)84, (char)246, (char)17, (char)138, (char)44, (char)237, (char)205, (char)222, (char)227, (char)36, (char)75, (char)235, (char)147, (char)62, (char)150, (char)165, (char)125}));
            assert(pack.satellites_visible_GET() == (char)6);
        });
        DemoDevice.GPS_STATUS p25 = LoopBackDemoChannel.new_GPS_STATUS();
        PH.setPack(p25);
        p25.satellite_azimuth_SET(new char[] {(char)75, (char)186, (char)33, (char)42, (char)205, (char)167, (char)39, (char)99, (char)156, (char)136, (char)45, (char)37, (char)225, (char)135, (char)222, (char)26, (char)147, (char)150, (char)65, (char)150}, 0) ;
        p25.satellite_snr_SET(new char[] {(char)249, (char)61, (char)157, (char)56, (char)179, (char)87, (char)179, (char)228, (char)107, (char)13, (char)230, (char)72, (char)31, (char)182, (char)179, (char)228, (char)182, (char)220, (char)222, (char)39}, 0) ;
        p25.satellite_elevation_SET(new char[] {(char)16, (char)122, (char)1, (char)246, (char)71, (char)65, (char)140, (char)242, (char)58, (char)83, (char)35, (char)71, (char)9, (char)2, (char)175, (char)252, (char)90, (char)30, (char)54, (char)31}, 0) ;
        p25.satellite_prn_SET(new char[] {(char)128, (char)92, (char)139, (char)45, (char)170, (char)3, (char)137, (char)253, (char)48, (char)108, (char)149, (char)109, (char)198, (char)55, (char)51, (char)146, (char)248, (char)249, (char)159, (char)25}, 0) ;
        p25.satellite_used_SET(new char[] {(char)179, (char)248, (char)235, (char)84, (char)246, (char)17, (char)138, (char)44, (char)237, (char)205, (char)222, (char)227, (char)36, (char)75, (char)235, (char)147, (char)62, (char)150, (char)165, (char)125}, 0) ;
        p25.satellites_visible_SET((char)6) ;
        LoopBackDemoChannel.instance.send(p25);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short)1118);
            assert(pack.time_boot_ms_GET() == 2257950469L);
            assert(pack.ymag_GET() == (short)22014);
            assert(pack.zgyro_GET() == (short) -20388);
            assert(pack.zmag_GET() == (short) -18407);
            assert(pack.xacc_GET() == (short) -27049);
            assert(pack.xmag_GET() == (short) -19128);
            assert(pack.xgyro_GET() == (short) -29767);
            assert(pack.zacc_GET() == (short) -30234);
            assert(pack.ygyro_GET() == (short)20465);
        });
        DemoDevice.SCALED_IMU p26 = LoopBackDemoChannel.new_SCALED_IMU();
        PH.setPack(p26);
        p26.yacc_SET((short)1118) ;
        p26.xacc_SET((short) -27049) ;
        p26.xmag_SET((short) -19128) ;
        p26.time_boot_ms_SET(2257950469L) ;
        p26.ygyro_SET((short)20465) ;
        p26.ymag_SET((short)22014) ;
        p26.xgyro_SET((short) -29767) ;
        p26.zacc_SET((short) -30234) ;
        p26.zgyro_SET((short) -20388) ;
        p26.zmag_SET((short) -18407) ;
        LoopBackDemoChannel.instance.send(p26);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_IMU.add((src, ph, pack) ->
        {
            assert(pack.xacc_GET() == (short)28146);
            assert(pack.yacc_GET() == (short)28062);
            assert(pack.time_usec_GET() == 5079255678799706806L);
            assert(pack.xmag_GET() == (short)8814);
            assert(pack.ymag_GET() == (short) -27307);
            assert(pack.zgyro_GET() == (short) -23830);
            assert(pack.zmag_GET() == (short) -12469);
            assert(pack.ygyro_GET() == (short) -5458);
            assert(pack.zacc_GET() == (short) -27432);
            assert(pack.xgyro_GET() == (short)18660);
        });
        DemoDevice.RAW_IMU p27 = LoopBackDemoChannel.new_RAW_IMU();
        PH.setPack(p27);
        p27.ygyro_SET((short) -5458) ;
        p27.time_usec_SET(5079255678799706806L) ;
        p27.yacc_SET((short)28062) ;
        p27.ymag_SET((short) -27307) ;
        p27.xgyro_SET((short)18660) ;
        p27.zmag_SET((short) -12469) ;
        p27.zacc_SET((short) -27432) ;
        p27.xacc_SET((short)28146) ;
        p27.zgyro_SET((short) -23830) ;
        p27.xmag_SET((short)8814) ;
        LoopBackDemoChannel.instance.send(p27);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RAW_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == (short)9250);
            assert(pack.time_usec_GET() == 7373229928475280976L);
            assert(pack.press_diff1_GET() == (short)5536);
            assert(pack.press_diff2_GET() == (short)30111);
            assert(pack.temperature_GET() == (short)11807);
        });
        DemoDevice.RAW_PRESSURE p28 = LoopBackDemoChannel.new_RAW_PRESSURE();
        PH.setPack(p28);
        p28.temperature_SET((short)11807) ;
        p28.time_usec_SET(7373229928475280976L) ;
        p28.press_diff1_SET((short)5536) ;
        p28.press_diff2_SET((short)30111) ;
        p28.press_abs_SET((short)9250) ;
        LoopBackDemoChannel.instance.send(p28);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE.add((src, ph, pack) ->
        {
            assert(pack.temperature_GET() == (short)24376);
            assert(pack.press_abs_GET() == -9.197664E37F);
            assert(pack.press_diff_GET() == 7.722411E36F);
            assert(pack.time_boot_ms_GET() == 1209283190L);
        });
        DemoDevice.SCALED_PRESSURE p29 = LoopBackDemoChannel.new_SCALED_PRESSURE();
        PH.setPack(p29);
        p29.time_boot_ms_SET(1209283190L) ;
        p29.press_abs_SET(-9.197664E37F) ;
        p29.temperature_SET((short)24376) ;
        p29.press_diff_SET(7.722411E36F) ;
        LoopBackDemoChannel.instance.send(p29);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == -1.2957054E38F);
            assert(pack.pitchspeed_GET() == -3.343016E38F);
            assert(pack.roll_GET() == 1.4469474E38F);
            assert(pack.time_boot_ms_GET() == 1487273366L);
            assert(pack.yawspeed_GET() == 1.2189027E38F);
            assert(pack.rollspeed_GET() == 1.88843E38F);
            assert(pack.yaw_GET() == 2.0451853E38F);
        });
        DemoDevice.ATTITUDE p30 = LoopBackDemoChannel.new_ATTITUDE();
        PH.setPack(p30);
        p30.yawspeed_SET(1.2189027E38F) ;
        p30.rollspeed_SET(1.88843E38F) ;
        p30.pitchspeed_SET(-3.343016E38F) ;
        p30.roll_SET(1.4469474E38F) ;
        p30.yaw_SET(2.0451853E38F) ;
        p30.pitch_SET(-1.2957054E38F) ;
        p30.time_boot_ms_SET(1487273366L) ;
        LoopBackDemoChannel.instance.send(p30);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION.add((src, ph, pack) ->
        {
            assert(pack.q1_GET() == 1.3840218E38F);
            assert(pack.yawspeed_GET() == 1.1907788E38F);
            assert(pack.q2_GET() == 2.8989806E38F);
            assert(pack.pitchspeed_GET() == -1.154099E38F);
            assert(pack.q3_GET() == -5.61669E37F);
            assert(pack.q4_GET() == 5.253847E37F);
            assert(pack.time_boot_ms_GET() == 2729055212L);
            assert(pack.rollspeed_GET() == 3.2772366E38F);
        });
        DemoDevice.ATTITUDE_QUATERNION p31 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION();
        PH.setPack(p31);
        p31.yawspeed_SET(1.1907788E38F) ;
        p31.q4_SET(5.253847E37F) ;
        p31.q3_SET(-5.61669E37F) ;
        p31.pitchspeed_SET(-1.154099E38F) ;
        p31.q2_SET(2.8989806E38F) ;
        p31.q1_SET(1.3840218E38F) ;
        p31.time_boot_ms_SET(2729055212L) ;
        p31.rollspeed_SET(3.2772366E38F) ;
        LoopBackDemoChannel.instance.send(p31);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3279217957L);
            assert(pack.y_GET() == 2.9432788E38F);
            assert(pack.vx_GET() == -8.963329E36F);
            assert(pack.vy_GET() == 5.7101443E37F);
            assert(pack.x_GET() == -4.192506E36F);
            assert(pack.z_GET() == -2.5131782E38F);
            assert(pack.vz_GET() == 1.1192749E38F);
        });
        DemoDevice.LOCAL_POSITION_NED p32 = LoopBackDemoChannel.new_LOCAL_POSITION_NED();
        PH.setPack(p32);
        p32.vx_SET(-8.963329E36F) ;
        p32.time_boot_ms_SET(3279217957L) ;
        p32.x_SET(-4.192506E36F) ;
        p32.y_SET(2.9432788E38F) ;
        p32.z_SET(-2.5131782E38F) ;
        p32.vy_SET(5.7101443E37F) ;
        p32.vz_SET(1.1192749E38F) ;
        LoopBackDemoChannel.instance.send(p32);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -1715594940);
            assert(pack.hdg_GET() == (char)7815);
            assert(pack.vz_GET() == (short) -21130);
            assert(pack.relative_alt_GET() == 242192510);
            assert(pack.vx_GET() == (short)26200);
            assert(pack.time_boot_ms_GET() == 1636465219L);
            assert(pack.vy_GET() == (short) -19614);
            assert(pack.lon_GET() == -899747907);
            assert(pack.alt_GET() == -913079133);
        });
        DemoDevice.GLOBAL_POSITION_INT p33 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT();
        PH.setPack(p33);
        p33.vx_SET((short)26200) ;
        p33.lat_SET(-1715594940) ;
        p33.relative_alt_SET(242192510) ;
        p33.vz_SET((short) -21130) ;
        p33.time_boot_ms_SET(1636465219L) ;
        p33.alt_SET(-913079133) ;
        p33.hdg_SET((char)7815) ;
        p33.lon_SET(-899747907) ;
        p33.vy_SET((short) -19614) ;
        LoopBackDemoChannel.instance.send(p33);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_SCALED.add((src, ph, pack) ->
        {
            assert(pack.chan3_scaled_GET() == (short) -5888);
            assert(pack.chan6_scaled_GET() == (short)20675);
            assert(pack.rssi_GET() == (char)160);
            assert(pack.chan2_scaled_GET() == (short) -16594);
            assert(pack.chan4_scaled_GET() == (short)25402);
            assert(pack.chan5_scaled_GET() == (short)8299);
            assert(pack.time_boot_ms_GET() == 1757307584L);
            assert(pack.port_GET() == (char)202);
            assert(pack.chan1_scaled_GET() == (short)13437);
            assert(pack.chan7_scaled_GET() == (short)8967);
            assert(pack.chan8_scaled_GET() == (short) -6324);
        });
        DemoDevice.RC_CHANNELS_SCALED p34 = LoopBackDemoChannel.new_RC_CHANNELS_SCALED();
        PH.setPack(p34);
        p34.time_boot_ms_SET(1757307584L) ;
        p34.chan3_scaled_SET((short) -5888) ;
        p34.chan4_scaled_SET((short)25402) ;
        p34.chan8_scaled_SET((short) -6324) ;
        p34.chan1_scaled_SET((short)13437) ;
        p34.rssi_SET((char)160) ;
        p34.chan7_scaled_SET((short)8967) ;
        p34.port_SET((char)202) ;
        p34.chan6_scaled_SET((short)20675) ;
        p34.chan2_scaled_SET((short) -16594) ;
        p34.chan5_scaled_SET((short)8299) ;
        LoopBackDemoChannel.instance.send(p34);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_RAW.add((src, ph, pack) ->
        {
            assert(pack.rssi_GET() == (char)223);
            assert(pack.chan3_raw_GET() == (char)58575);
            assert(pack.chan5_raw_GET() == (char)56724);
            assert(pack.chan6_raw_GET() == (char)10830);
            assert(pack.time_boot_ms_GET() == 2910170084L);
            assert(pack.chan2_raw_GET() == (char)41532);
            assert(pack.chan1_raw_GET() == (char)57077);
            assert(pack.chan7_raw_GET() == (char)30023);
            assert(pack.chan8_raw_GET() == (char)14042);
            assert(pack.chan4_raw_GET() == (char)63189);
            assert(pack.port_GET() == (char)87);
        });
        DemoDevice.RC_CHANNELS_RAW p35 = LoopBackDemoChannel.new_RC_CHANNELS_RAW();
        PH.setPack(p35);
        p35.chan3_raw_SET((char)58575) ;
        p35.chan8_raw_SET((char)14042) ;
        p35.chan2_raw_SET((char)41532) ;
        p35.chan1_raw_SET((char)57077) ;
        p35.chan5_raw_SET((char)56724) ;
        p35.port_SET((char)87) ;
        p35.time_boot_ms_SET(2910170084L) ;
        p35.chan4_raw_SET((char)63189) ;
        p35.chan6_raw_SET((char)10830) ;
        p35.rssi_SET((char)223) ;
        p35.chan7_raw_SET((char)30023) ;
        LoopBackDemoChannel.instance.send(p35);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERVO_OUTPUT_RAW.add((src, ph, pack) ->
        {
            assert(pack.servo13_raw_TRY(ph) == (char)7939);
            assert(pack.servo8_raw_GET() == (char)44665);
            assert(pack.servo12_raw_TRY(ph) == (char)38188);
            assert(pack.servo14_raw_TRY(ph) == (char)7559);
            assert(pack.servo6_raw_GET() == (char)47154);
            assert(pack.servo7_raw_GET() == (char)53928);
            assert(pack.servo4_raw_GET() == (char)6201);
            assert(pack.time_usec_GET() == 4268835031L);
            assert(pack.servo1_raw_GET() == (char)4166);
            assert(pack.servo10_raw_TRY(ph) == (char)6402);
            assert(pack.servo9_raw_TRY(ph) == (char)50342);
            assert(pack.port_GET() == (char)135);
            assert(pack.servo16_raw_TRY(ph) == (char)61423);
            assert(pack.servo2_raw_GET() == (char)23696);
            assert(pack.servo11_raw_TRY(ph) == (char)21537);
            assert(pack.servo15_raw_TRY(ph) == (char)61276);
            assert(pack.servo3_raw_GET() == (char)26851);
            assert(pack.servo5_raw_GET() == (char)8036);
        });
        DemoDevice.SERVO_OUTPUT_RAW p36 = LoopBackDemoChannel.new_SERVO_OUTPUT_RAW();
        PH.setPack(p36);
        p36.servo5_raw_SET((char)8036) ;
        p36.time_usec_SET(4268835031L) ;
        p36.servo8_raw_SET((char)44665) ;
        p36.port_SET((char)135) ;
        p36.servo15_raw_SET((char)61276, PH) ;
        p36.servo14_raw_SET((char)7559, PH) ;
        p36.servo3_raw_SET((char)26851) ;
        p36.servo2_raw_SET((char)23696) ;
        p36.servo7_raw_SET((char)53928) ;
        p36.servo16_raw_SET((char)61423, PH) ;
        p36.servo1_raw_SET((char)4166) ;
        p36.servo4_raw_SET((char)6201) ;
        p36.servo13_raw_SET((char)7939, PH) ;
        p36.servo10_raw_SET((char)6402, PH) ;
        p36.servo12_raw_SET((char)38188, PH) ;
        p36.servo9_raw_SET((char)50342, PH) ;
        p36.servo6_raw_SET((char)47154) ;
        p36.servo11_raw_SET((char)21537, PH) ;
        LoopBackDemoChannel.instance.send(p36);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)78);
            assert(pack.target_system_GET() == (char)72);
            assert(pack.start_index_GET() == (short) -12201);
            assert(pack.end_index_GET() == (short) -16830);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        DemoDevice.MISSION_REQUEST_PARTIAL_LIST p37 = LoopBackDemoChannel.new_MISSION_REQUEST_PARTIAL_LIST();
        PH.setPack(p37);
        p37.target_system_SET((char)72) ;
        p37.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p37.target_component_SET((char)78) ;
        p37.end_index_SET((short) -16830) ;
        p37.start_index_SET((short) -12201) ;
        LoopBackDemoChannel.instance.send(p37);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_WRITE_PARTIAL_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)77);
            assert(pack.start_index_GET() == (short) -6267);
            assert(pack.target_system_GET() == (char)61);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.end_index_GET() == (short) -10633);
        });
        DemoDevice.MISSION_WRITE_PARTIAL_LIST p38 = LoopBackDemoChannel.new_MISSION_WRITE_PARTIAL_LIST();
        PH.setPack(p38);
        p38.start_index_SET((short) -6267) ;
        p38.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p38.end_index_SET((short) -10633) ;
        p38.target_system_SET((char)61) ;
        p38.target_component_SET((char)77) ;
        LoopBackDemoChannel.instance.send(p38);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM.add((src, ph, pack) ->
        {
            assert(pack.param2_GET() == 3.0318152E38F);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT);
            assert(pack.seq_GET() == (char)175);
            assert(pack.param1_GET() == 5.858891E37F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.current_GET() == (char)255);
            assert(pack.param3_GET() == -2.3118544E38F);
            assert(pack.y_GET() == 1.3667055E38F);
            assert(pack.autocontinue_GET() == (char)133);
            assert(pack.z_GET() == -2.015254E38F);
            assert(pack.target_component_GET() == (char)42);
            assert(pack.param4_GET() == 2.9346046E38F);
            assert(pack.x_GET() == 1.8143512E38F);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL);
            assert(pack.target_system_GET() == (char)15);
        });
        DemoDevice.MISSION_ITEM p39 = LoopBackDemoChannel.new_MISSION_ITEM();
        PH.setPack(p39);
        p39.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_ALL) ;
        p39.autocontinue_SET((char)133) ;
        p39.x_SET(1.8143512E38F) ;
        p39.param4_SET(2.9346046E38F) ;
        p39.seq_SET((char)175) ;
        p39.z_SET(-2.015254E38F) ;
        p39.target_component_SET((char)42) ;
        p39.target_system_SET((char)15) ;
        p39.param1_SET(5.858891E37F) ;
        p39.y_SET(1.3667055E38F) ;
        p39.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p39.current_SET((char)255) ;
        p39.command_SET(MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT) ;
        p39.param3_SET(-2.3118544E38F) ;
        p39.param2_SET(3.0318152E38F) ;
        LoopBackDemoChannel.instance.send(p39);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)14053);
            assert(pack.target_component_GET() == (char)199);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_system_GET() == (char)16);
        });
        DemoDevice.MISSION_REQUEST p40 = LoopBackDemoChannel.new_MISSION_REQUEST();
        PH.setPack(p40);
        p40.target_component_SET((char)199) ;
        p40.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p40.seq_SET((char)14053) ;
        p40.target_system_SET((char)16) ;
        LoopBackDemoChannel.instance.send(p40);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_SET_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)105);
            assert(pack.target_component_GET() == (char)197);
            assert(pack.seq_GET() == (char)35739);
        });
        DemoDevice.MISSION_SET_CURRENT p41 = LoopBackDemoChannel.new_MISSION_SET_CURRENT();
        PH.setPack(p41);
        p41.seq_SET((char)35739) ;
        p41.target_component_SET((char)197) ;
        p41.target_system_SET((char)105) ;
        LoopBackDemoChannel.instance.send(p41);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CURRENT.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)15761);
        });
        DemoDevice.MISSION_CURRENT p42 = LoopBackDemoChannel.new_MISSION_CURRENT();
        PH.setPack(p42);
        p42.seq_SET((char)15761) ;
        LoopBackDemoChannel.instance.send(p42);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)82);
            assert(pack.target_component_GET() == (char)84);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
        });
        DemoDevice.MISSION_REQUEST_LIST p43 = LoopBackDemoChannel.new_MISSION_REQUEST_LIST();
        PH.setPack(p43);
        p43.target_system_SET((char)82) ;
        p43.target_component_SET((char)84) ;
        p43.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        LoopBackDemoChannel.instance.send(p43);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_COUNT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)213);
            assert(pack.target_component_GET() == (char)79);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.count_GET() == (char)18469);
        });
        DemoDevice.MISSION_COUNT p44 = LoopBackDemoChannel.new_MISSION_COUNT();
        PH.setPack(p44);
        p44.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p44.target_component_SET((char)79) ;
        p44.target_system_SET((char)213) ;
        p44.count_SET((char)18469) ;
        LoopBackDemoChannel.instance.send(p44);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_CLEAR_ALL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)239);
            assert(pack.target_system_GET() == (char)8);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
        });
        DemoDevice.MISSION_CLEAR_ALL p45 = LoopBackDemoChannel.new_MISSION_CLEAR_ALL();
        PH.setPack(p45);
        p45.target_system_SET((char)8) ;
        p45.target_component_SET((char)239) ;
        p45.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        LoopBackDemoChannel.instance.send(p45);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_REACHED.add((src, ph, pack) ->
        {
            assert(pack.seq_GET() == (char)60888);
        });
        DemoDevice.MISSION_ITEM_REACHED p46 = LoopBackDemoChannel.new_MISSION_ITEM_REACHED();
        PH.setPack(p46);
        p46.seq_SET((char)60888) ;
        LoopBackDemoChannel.instance.send(p46);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ACK.add((src, ph, pack) ->
        {
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE);
            assert(pack.target_system_GET() == (char)30);
            assert(pack.target_component_GET() == (char)247);
            assert(pack.type_GET() == MAV_MISSION_RESULT.MAV_MISSION_ERROR);
        });
        DemoDevice.MISSION_ACK p47 = LoopBackDemoChannel.new_MISSION_ACK();
        PH.setPack(p47);
        p47.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_FENCE) ;
        p47.target_component_SET((char)247) ;
        p47.type_SET(MAV_MISSION_RESULT.MAV_MISSION_ERROR) ;
        p47.target_system_SET((char)30) ;
        LoopBackDemoChannel.instance.send(p47);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.time_usec_TRY(ph) == 353642757189065688L);
            assert(pack.target_system_GET() == (char)27);
            assert(pack.altitude_GET() == -1194421451);
            assert(pack.longitude_GET() == -418501506);
            assert(pack.latitude_GET() == -1534884155);
        });
        DemoDevice.SET_GPS_GLOBAL_ORIGIN p48 = LoopBackDemoChannel.new_SET_GPS_GLOBAL_ORIGIN();
        PH.setPack(p48);
        p48.target_system_SET((char)27) ;
        p48.latitude_SET(-1534884155) ;
        p48.time_usec_SET(353642757189065688L, PH) ;
        p48.longitude_SET(-418501506) ;
        p48.altitude_SET(-1194421451) ;
        LoopBackDemoChannel.instance.send(p48);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_GLOBAL_ORIGIN.add((src, ph, pack) ->
        {
            assert(pack.altitude_GET() == 1162669736);
            assert(pack.latitude_GET() == 1872451738);
            assert(pack.time_usec_TRY(ph) == 7651813971056988678L);
            assert(pack.longitude_GET() == -1898238274);
        });
        DemoDevice.GPS_GLOBAL_ORIGIN p49 = LoopBackDemoChannel.new_GPS_GLOBAL_ORIGIN();
        PH.setPack(p49);
        p49.longitude_SET(-1898238274) ;
        p49.latitude_SET(1872451738) ;
        p49.time_usec_SET(7651813971056988678L, PH) ;
        p49.altitude_SET(1162669736) ;
        LoopBackDemoChannel.instance.send(p49);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_MAP_RC.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)146);
            assert(pack.param_id_LEN(ph) == 1);
            assert(pack.param_id_TRY(ph).equals("k"));
            assert(pack.scale_GET() == -2.5907004E38F);
            assert(pack.param_value_max_GET() == -6.5415963E37F);
            assert(pack.parameter_rc_channel_index_GET() == (char)141);
            assert(pack.param_value_min_GET() == 6.444997E37F);
            assert(pack.param_index_GET() == (short)12398);
            assert(pack.param_value0_GET() == 3.1939707E37F);
            assert(pack.target_system_GET() == (char)243);
        });
        DemoDevice.PARAM_MAP_RC p50 = LoopBackDemoChannel.new_PARAM_MAP_RC();
        PH.setPack(p50);
        p50.scale_SET(-2.5907004E38F) ;
        p50.param_id_SET("k", PH) ;
        p50.parameter_rc_channel_index_SET((char)141) ;
        p50.param_value_min_SET(6.444997E37F) ;
        p50.target_component_SET((char)146) ;
        p50.param_value_max_SET(-6.5415963E37F) ;
        p50.target_system_SET((char)243) ;
        p50.param_index_SET((short)12398) ;
        p50.param_value0_SET(3.1939707E37F) ;
        LoopBackDemoChannel.instance.send(p50);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_REQUEST_INT.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)242);
            assert(pack.seq_GET() == (char)64224);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.target_component_GET() == (char)219);
        });
        DemoDevice.MISSION_REQUEST_INT p51 = LoopBackDemoChannel.new_MISSION_REQUEST_INT();
        PH.setPack(p51);
        p51.target_system_SET((char)242) ;
        p51.seq_SET((char)64224) ;
        p51.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p51.target_component_SET((char)219) ;
        LoopBackDemoChannel.instance.send(p51);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_SET_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.p2y_GET() == 3.00948E38F);
            assert(pack.p1y_GET() == -2.9919179E38F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT);
            assert(pack.p2z_GET() == -5.6436895E37F);
            assert(pack.p1x_GET() == -1.6815952E38F);
            assert(pack.p1z_GET() == -2.9186745E37F);
            assert(pack.p2x_GET() == -1.2385847E38F);
            assert(pack.target_component_GET() == (char)123);
            assert(pack.target_system_GET() == (char)170);
        });
        DemoDevice.SAFETY_SET_ALLOWED_AREA p54 = LoopBackDemoChannel.new_SAFETY_SET_ALLOWED_AREA();
        PH.setPack(p54);
        p54.target_component_SET((char)123) ;
        p54.p1z_SET(-2.9186745E37F) ;
        p54.p1y_SET(-2.9919179E38F) ;
        p54.p2x_SET(-1.2385847E38F) ;
        p54.p1x_SET(-1.6815952E38F) ;
        p54.target_system_SET((char)170) ;
        p54.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT) ;
        p54.p2y_SET(3.00948E38F) ;
        p54.p2z_SET(-5.6436895E37F) ;
        LoopBackDemoChannel.instance.send(p54);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SAFETY_ALLOWED_AREA.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT);
            assert(pack.p2z_GET() == 2.5934937E37F);
            assert(pack.p2x_GET() == -1.9125986E36F);
            assert(pack.p1z_GET() == -1.0535076E38F);
            assert(pack.p1y_GET() == -1.4928188E38F);
            assert(pack.p1x_GET() == 3.1443824E38F);
            assert(pack.p2y_GET() == 3.2776035E38F);
        });
        DemoDevice.SAFETY_ALLOWED_AREA p55 = LoopBackDemoChannel.new_SAFETY_ALLOWED_AREA();
        PH.setPack(p55);
        p55.p1y_SET(-1.4928188E38F) ;
        p55.p2z_SET(2.5934937E37F) ;
        p55.p1x_SET(3.1443824E38F) ;
        p55.p2y_SET(3.2776035E38F) ;
        p55.frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_TERRAIN_ALT) ;
        p55.p1z_SET(-1.0535076E38F) ;
        p55.p2x_SET(-1.9125986E36F) ;
        LoopBackDemoChannel.instance.send(p55);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_QUATERNION_COV.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == 3.1227514E36F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.8790793E38F, -2.9644253E38F, -1.6070306E38F, -2.35182E38F}));
            assert(pack.yawspeed_GET() == -3.312715E37F);
            assert(pack.pitchspeed_GET() == 1.2828158E38F);
            assert(pack.time_usec_GET() == 5228618632716516760L);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.2266106E38F, -1.3507015E38F, 2.1819587E38F, -3.043819E38F, -3.0424884E38F, 8.3083514E37F, -5.1060236E37F, -1.0934447E38F, -1.2192054E38F}));
        });
        DemoDevice.ATTITUDE_QUATERNION_COV p61 = LoopBackDemoChannel.new_ATTITUDE_QUATERNION_COV();
        PH.setPack(p61);
        p61.covariance_SET(new float[] {2.2266106E38F, -1.3507015E38F, 2.1819587E38F, -3.043819E38F, -3.0424884E38F, 8.3083514E37F, -5.1060236E37F, -1.0934447E38F, -1.2192054E38F}, 0) ;
        p61.q_SET(new float[] {-1.8790793E38F, -2.9644253E38F, -1.6070306E38F, -2.35182E38F}, 0) ;
        p61.time_usec_SET(5228618632716516760L) ;
        p61.yawspeed_SET(-3.312715E37F) ;
        p61.pitchspeed_SET(1.2828158E38F) ;
        p61.rollspeed_SET(3.1227514E36F) ;
        LoopBackDemoChannel.instance.send(p61);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAV_CONTROLLER_OUTPUT.add((src, ph, pack) ->
        {
            assert(pack.wp_dist_GET() == (char)63914);
            assert(pack.alt_error_GET() == 3.984674E36F);
            assert(pack.nav_roll_GET() == -1.1835026E38F);
            assert(pack.nav_pitch_GET() == 1.1161171E38F);
            assert(pack.xtrack_error_GET() == -1.2799132E38F);
            assert(pack.nav_bearing_GET() == (short)1251);
            assert(pack.aspd_error_GET() == -2.6439326E38F);
            assert(pack.target_bearing_GET() == (short) -26564);
        });
        DemoDevice.NAV_CONTROLLER_OUTPUT p62 = LoopBackDemoChannel.new_NAV_CONTROLLER_OUTPUT();
        PH.setPack(p62);
        p62.alt_error_SET(3.984674E36F) ;
        p62.xtrack_error_SET(-1.2799132E38F) ;
        p62.nav_bearing_SET((short)1251) ;
        p62.nav_pitch_SET(1.1161171E38F) ;
        p62.aspd_error_SET(-2.6439326E38F) ;
        p62.nav_roll_SET(-1.1835026E38F) ;
        p62.target_bearing_SET((short) -26564) ;
        p62.wp_dist_SET((char)63914) ;
        LoopBackDemoChannel.instance.send(p62);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_POSITION_INT_COV.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == 1804969017);
            assert(pack.vz_GET() == 2.0974301E38F);
            assert(pack.time_usec_GET() == 5009000491704547156L);
            assert(pack.vx_GET() == 1.0049418E38F);
            assert(pack.relative_alt_GET() == -1330212854);
            assert(pack.lon_GET() == -1171601774);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.3630725E38F, 2.8458113E38F, 2.5219673E38F, 2.491172E38F, -1.1746311E38F, 1.525644E38F, 1.4566946E38F, -1.6981194E38F, 2.2868902E38F, -9.014598E37F, -2.420766E38F, 3.0581468E38F, 1.3730277E38F, 2.678186E38F, -1.3319224E37F, 1.5055037E38F, -2.1573594E38F, 3.0200553E38F, 3.049982E38F, 3.2633286E38F, 1.2551813E38F, 2.3864634E38F, 3.5023812E37F, 2.0964627E38F, 1.8780294E38F, -4.2173952E37F, 2.6440492E38F, 4.3652197E37F, -2.765626E38F, -3.1950063E37F, -2.6802908E38F, 1.2325058E38F, -1.0641616E38F, -5.0860142E36F, -3.7401378E36F, -2.9606053E38F}));
            assert(pack.vy_GET() == -4.498239E37F);
            assert(pack.alt_GET() == 1382115622);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS);
        });
        DemoDevice.GLOBAL_POSITION_INT_COV p63 = LoopBackDemoChannel.new_GLOBAL_POSITION_INT_COV();
        PH.setPack(p63);
        p63.covariance_SET(new float[] {2.3630725E38F, 2.8458113E38F, 2.5219673E38F, 2.491172E38F, -1.1746311E38F, 1.525644E38F, 1.4566946E38F, -1.6981194E38F, 2.2868902E38F, -9.014598E37F, -2.420766E38F, 3.0581468E38F, 1.3730277E38F, 2.678186E38F, -1.3319224E37F, 1.5055037E38F, -2.1573594E38F, 3.0200553E38F, 3.049982E38F, 3.2633286E38F, 1.2551813E38F, 2.3864634E38F, 3.5023812E37F, 2.0964627E38F, 1.8780294E38F, -4.2173952E37F, 2.6440492E38F, 4.3652197E37F, -2.765626E38F, -3.1950063E37F, -2.6802908E38F, 1.2325058E38F, -1.0641616E38F, -5.0860142E36F, -3.7401378E36F, -2.9606053E38F}, 0) ;
        p63.alt_SET(1382115622) ;
        p63.vz_SET(2.0974301E38F) ;
        p63.time_usec_SET(5009000491704547156L) ;
        p63.lat_SET(1804969017) ;
        p63.lon_SET(-1171601774) ;
        p63.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_GPS_INS) ;
        p63.vy_SET(-4.498239E37F) ;
        p63.vx_SET(1.0049418E38F) ;
        p63.relative_alt_SET(-1330212854) ;
        LoopBackDemoChannel.instance.send(p63);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_COV.add((src, ph, pack) ->
        {
            assert(pack.ax_GET() == -1.6442691E37F);
            assert(pack.vz_GET() == -2.9835398E38F);
            assert(pack.y_GET() == 2.5184003E38F);
            assert(Arrays.equals(pack.covariance_GET(),  new float[] {2.2108173E38F, 2.470409E37F, 3.257324E38F, 1.0823068E38F, -2.0494272E38F, -2.1828386E38F, -2.2814441E38F, 6.114021E37F, 1.3378097E38F, 2.2180036E38F, -2.4626993E38F, -6.9214635E37F, 1.8871665E38F, 5.0928476E37F, -1.5384755E38F, -5.2295283E37F, 1.2615482E38F, 2.9446071E38F, -3.3808824E38F, 1.5489407E38F, 3.1879697E38F, -4.0055196E37F, -3.076769E38F, -5.772085E37F, 2.241296E38F, 2.5908866E38F, -3.325305E38F, -9.249014E37F, -2.1270147E38F, 3.2253573E38F, -5.1603647E37F, 2.94352E38F, -1.0425372E37F, 2.9168468E38F, -6.86889E37F, 9.505479E37F, 1.4423567E38F, 1.5794434E38F, 4.603859E37F, 2.138281E38F, 3.7438136E37F, -1.5629802E38F, 1.861251E38F, -3.017938E38F, -1.3155185E38F}));
            assert(pack.z_GET() == -1.345251E38F);
            assert(pack.estimator_type_GET() == MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION);
            assert(pack.vy_GET() == 1.308178E38F);
            assert(pack.vx_GET() == 4.316456E37F);
            assert(pack.x_GET() == 2.7485204E38F);
            assert(pack.ay_GET() == -3.1716095E38F);
            assert(pack.time_usec_GET() == 4313946956746769319L);
            assert(pack.az_GET() == -3.1919992E38F);
        });
        DemoDevice.LOCAL_POSITION_NED_COV p64 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_COV();
        PH.setPack(p64);
        p64.estimator_type_SET(MAV_ESTIMATOR_TYPE.MAV_ESTIMATOR_TYPE_VISION) ;
        p64.vy_SET(1.308178E38F) ;
        p64.ay_SET(-3.1716095E38F) ;
        p64.x_SET(2.7485204E38F) ;
        p64.vz_SET(-2.9835398E38F) ;
        p64.covariance_SET(new float[] {2.2108173E38F, 2.470409E37F, 3.257324E38F, 1.0823068E38F, -2.0494272E38F, -2.1828386E38F, -2.2814441E38F, 6.114021E37F, 1.3378097E38F, 2.2180036E38F, -2.4626993E38F, -6.9214635E37F, 1.8871665E38F, 5.0928476E37F, -1.5384755E38F, -5.2295283E37F, 1.2615482E38F, 2.9446071E38F, -3.3808824E38F, 1.5489407E38F, 3.1879697E38F, -4.0055196E37F, -3.076769E38F, -5.772085E37F, 2.241296E38F, 2.5908866E38F, -3.325305E38F, -9.249014E37F, -2.1270147E38F, 3.2253573E38F, -5.1603647E37F, 2.94352E38F, -1.0425372E37F, 2.9168468E38F, -6.86889E37F, 9.505479E37F, 1.4423567E38F, 1.5794434E38F, 4.603859E37F, 2.138281E38F, 3.7438136E37F, -1.5629802E38F, 1.861251E38F, -3.017938E38F, -1.3155185E38F}, 0) ;
        p64.ax_SET(-1.6442691E37F) ;
        p64.y_SET(2.5184003E38F) ;
        p64.time_usec_SET(4313946956746769319L) ;
        p64.vx_SET(4.316456E37F) ;
        p64.az_SET(-3.1919992E38F) ;
        p64.z_SET(-1.345251E38F) ;
        LoopBackDemoChannel.instance.send(p64);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS.add((src, ph, pack) ->
        {
            assert(pack.chan8_raw_GET() == (char)53152);
            assert(pack.chan5_raw_GET() == (char)10708);
            assert(pack.chan7_raw_GET() == (char)36029);
            assert(pack.chan14_raw_GET() == (char)52061);
            assert(pack.chan17_raw_GET() == (char)809);
            assert(pack.time_boot_ms_GET() == 263667640L);
            assert(pack.chan4_raw_GET() == (char)28688);
            assert(pack.chancount_GET() == (char)137);
            assert(pack.chan15_raw_GET() == (char)56335);
            assert(pack.chan9_raw_GET() == (char)7822);
            assert(pack.chan2_raw_GET() == (char)42011);
            assert(pack.chan13_raw_GET() == (char)20778);
            assert(pack.chan16_raw_GET() == (char)21329);
            assert(pack.chan11_raw_GET() == (char)62649);
            assert(pack.chan12_raw_GET() == (char)39348);
            assert(pack.chan1_raw_GET() == (char)32499);
            assert(pack.chan10_raw_GET() == (char)20288);
            assert(pack.chan6_raw_GET() == (char)24887);
            assert(pack.chan3_raw_GET() == (char)4446);
            assert(pack.chan18_raw_GET() == (char)46701);
            assert(pack.rssi_GET() == (char)252);
        });
        DemoDevice.RC_CHANNELS p65 = LoopBackDemoChannel.new_RC_CHANNELS();
        PH.setPack(p65);
        p65.chan7_raw_SET((char)36029) ;
        p65.chancount_SET((char)137) ;
        p65.rssi_SET((char)252) ;
        p65.chan6_raw_SET((char)24887) ;
        p65.chan18_raw_SET((char)46701) ;
        p65.chan13_raw_SET((char)20778) ;
        p65.chan10_raw_SET((char)20288) ;
        p65.chan9_raw_SET((char)7822) ;
        p65.chan16_raw_SET((char)21329) ;
        p65.chan3_raw_SET((char)4446) ;
        p65.chan5_raw_SET((char)10708) ;
        p65.chan2_raw_SET((char)42011) ;
        p65.time_boot_ms_SET(263667640L) ;
        p65.chan12_raw_SET((char)39348) ;
        p65.chan14_raw_SET((char)52061) ;
        p65.chan8_raw_SET((char)53152) ;
        p65.chan15_raw_SET((char)56335) ;
        p65.chan17_raw_SET((char)809) ;
        p65.chan1_raw_SET((char)32499) ;
        p65.chan11_raw_SET((char)62649) ;
        p65.chan4_raw_SET((char)28688) ;
        LoopBackDemoChannel.instance.send(p65);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_REQUEST_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)255);
            assert(pack.target_system_GET() == (char)18);
            assert(pack.start_stop_GET() == (char)251);
            assert(pack.req_message_rate_GET() == (char)59250);
            assert(pack.req_stream_id_GET() == (char)18);
        });
        DemoDevice.REQUEST_DATA_STREAM p66 = LoopBackDemoChannel.new_REQUEST_DATA_STREAM();
        PH.setPack(p66);
        p66.req_stream_id_SET((char)18) ;
        p66.req_message_rate_SET((char)59250) ;
        p66.target_system_SET((char)18) ;
        p66.target_component_SET((char)255) ;
        p66.start_stop_SET((char)251) ;
        LoopBackDemoChannel.instance.send(p66);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_STREAM.add((src, ph, pack) ->
        {
            assert(pack.on_off_GET() == (char)60);
            assert(pack.message_rate_GET() == (char)48171);
            assert(pack.stream_id_GET() == (char)213);
        });
        DemoDevice.DATA_STREAM p67 = LoopBackDemoChannel.new_DATA_STREAM();
        PH.setPack(p67);
        p67.message_rate_SET((char)48171) ;
        p67.stream_id_SET((char)213) ;
        p67.on_off_SET((char)60) ;
        LoopBackDemoChannel.instance.send(p67);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.r_GET() == (short) -7225);
            assert(pack.z_GET() == (short)10225);
            assert(pack.x_GET() == (short) -70);
            assert(pack.target_GET() == (char)14);
            assert(pack.buttons_GET() == (char)52163);
            assert(pack.y_GET() == (short) -22537);
        });
        DemoDevice.MANUAL_CONTROL p69 = LoopBackDemoChannel.new_MANUAL_CONTROL();
        PH.setPack(p69);
        p69.buttons_SET((char)52163) ;
        p69.y_SET((short) -22537) ;
        p69.r_SET((short) -7225) ;
        p69.x_SET((short) -70) ;
        p69.z_SET((short)10225) ;
        p69.target_SET((char)14) ;
        LoopBackDemoChannel.instance.send(p69);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RC_CHANNELS_OVERRIDE.add((src, ph, pack) ->
        {
            assert(pack.chan3_raw_GET() == (char)39229);
            assert(pack.chan7_raw_GET() == (char)25347);
            assert(pack.chan8_raw_GET() == (char)1188);
            assert(pack.chan1_raw_GET() == (char)22843);
            assert(pack.chan5_raw_GET() == (char)61242);
            assert(pack.target_component_GET() == (char)203);
            assert(pack.chan4_raw_GET() == (char)2861);
            assert(pack.chan6_raw_GET() == (char)37726);
            assert(pack.target_system_GET() == (char)22);
            assert(pack.chan2_raw_GET() == (char)29255);
        });
        DemoDevice.RC_CHANNELS_OVERRIDE p70 = LoopBackDemoChannel.new_RC_CHANNELS_OVERRIDE();
        PH.setPack(p70);
        p70.chan4_raw_SET((char)2861) ;
        p70.chan5_raw_SET((char)61242) ;
        p70.chan7_raw_SET((char)25347) ;
        p70.chan3_raw_SET((char)39229) ;
        p70.chan8_raw_SET((char)1188) ;
        p70.target_component_SET((char)203) ;
        p70.chan1_raw_SET((char)22843) ;
        p70.target_system_SET((char)22) ;
        p70.chan2_raw_SET((char)29255) ;
        p70.chan6_raw_SET((char)37726) ;
        LoopBackDemoChannel.instance.send(p70);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MISSION_ITEM_INT.add((src, ph, pack) ->
        {
            assert(pack.param2_GET() == 2.0134648E38F);
            assert(pack.z_GET() == 7.262217E36F);
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.param1_GET() == -2.1624983E37F);
            assert(pack.param4_GET() == 2.0768072E38F);
            assert(pack.seq_GET() == (char)13843);
            assert(pack.param3_GET() == 4.9176903E37F);
            assert(pack.x_GET() == 533927324);
            assert(pack.target_system_GET() == (char)76);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_SET_HOME);
            assert(pack.y_GET() == 496936054);
            assert(pack.mission_type_GET() == MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY);
            assert(pack.current_GET() == (char)209);
            assert(pack.target_component_GET() == (char)56);
            assert(pack.autocontinue_GET() == (char)179);
        });
        DemoDevice.MISSION_ITEM_INT p73 = LoopBackDemoChannel.new_MISSION_ITEM_INT();
        PH.setPack(p73);
        p73.param1_SET(-2.1624983E37F) ;
        p73.command_SET(MAV_CMD.MAV_CMD_DO_SET_HOME) ;
        p73.target_component_SET((char)56) ;
        p73.frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p73.seq_SET((char)13843) ;
        p73.param2_SET(2.0134648E38F) ;
        p73.param3_SET(4.9176903E37F) ;
        p73.mission_type_SET(MAV_MISSION_TYPE.MAV_MISSION_TYPE_RALLY) ;
        p73.y_SET(496936054) ;
        p73.current_SET((char)209) ;
        p73.target_system_SET((char)76) ;
        p73.param4_SET(2.0768072E38F) ;
        p73.z_SET(7.262217E36F) ;
        p73.x_SET(533927324) ;
        p73.autocontinue_SET((char)179) ;
        LoopBackDemoChannel.instance.send(p73);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VFR_HUD.add((src, ph, pack) ->
        {
            assert(pack.airspeed_GET() == 1.988863E38F);
            assert(pack.climb_GET() == -3.3350374E38F);
            assert(pack.throttle_GET() == (char)45372);
            assert(pack.heading_GET() == (short)9908);
            assert(pack.groundspeed_GET() == -2.602128E37F);
            assert(pack.alt_GET() == -1.4620697E38F);
        });
        DemoDevice.VFR_HUD p74 = LoopBackDemoChannel.new_VFR_HUD();
        PH.setPack(p74);
        p74.alt_SET(-1.4620697E38F) ;
        p74.groundspeed_SET(-2.602128E37F) ;
        p74.climb_SET(-3.3350374E38F) ;
        p74.heading_SET((short)9908) ;
        p74.throttle_SET((char)45372) ;
        p74.airspeed_SET(1.988863E38F) ;
        LoopBackDemoChannel.instance.send(p74);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_INT.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_BODY_NED);
            assert(pack.z_GET() == 1.0423811E37F);
            assert(pack.target_system_GET() == (char)109);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_CONDITION_YAW);
            assert(pack.target_component_GET() == (char)217);
            assert(pack.param3_GET() == -6.6950926E37F);
            assert(pack.param1_GET() == 2.2198454E38F);
            assert(pack.param2_GET() == 3.0859254E38F);
            assert(pack.y_GET() == 545631432);
            assert(pack.autocontinue_GET() == (char)139);
            assert(pack.current_GET() == (char)12);
            assert(pack.x_GET() == 1544912838);
            assert(pack.param4_GET() == 2.793334E38F);
        });
        DemoDevice.COMMAND_INT p75 = LoopBackDemoChannel.new_COMMAND_INT();
        PH.setPack(p75);
        p75.target_system_SET((char)109) ;
        p75.x_SET(1544912838) ;
        p75.y_SET(545631432) ;
        p75.param2_SET(3.0859254E38F) ;
        p75.z_SET(1.0423811E37F) ;
        p75.param1_SET(2.2198454E38F) ;
        p75.param3_SET(-6.6950926E37F) ;
        p75.param4_SET(2.793334E38F) ;
        p75.frame_SET(MAV_FRAME.MAV_FRAME_BODY_NED) ;
        p75.current_SET((char)12) ;
        p75.target_component_SET((char)217) ;
        p75.command_SET(MAV_CMD.MAV_CMD_CONDITION_YAW) ;
        p75.autocontinue_SET((char)139) ;
        LoopBackDemoChannel.instance.send(p75);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_LONG.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)22);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_DO_FENCE_ENABLE);
            assert(pack.param3_GET() == -2.025439E38F);
            assert(pack.param5_GET() == -2.4209705E38F);
            assert(pack.param2_GET() == 1.0872729E36F);
            assert(pack.param7_GET() == 2.6287682E38F);
            assert(pack.confirmation_GET() == (char)116);
            assert(pack.target_system_GET() == (char)180);
            assert(pack.param4_GET() == 1.921682E38F);
            assert(pack.param6_GET() == 2.4964328E38F);
            assert(pack.param1_GET() == -8.246963E37F);
        });
        DemoDevice.COMMAND_LONG p76 = LoopBackDemoChannel.new_COMMAND_LONG();
        PH.setPack(p76);
        p76.command_SET(MAV_CMD.MAV_CMD_DO_FENCE_ENABLE) ;
        p76.param7_SET(2.6287682E38F) ;
        p76.param1_SET(-8.246963E37F) ;
        p76.param4_SET(1.921682E38F) ;
        p76.target_system_SET((char)180) ;
        p76.param3_SET(-2.025439E38F) ;
        p76.confirmation_SET((char)116) ;
        p76.param2_SET(1.0872729E36F) ;
        p76.param5_SET(-2.4209705E38F) ;
        p76.param6_SET(2.4964328E38F) ;
        p76.target_component_SET((char)22) ;
        LoopBackDemoChannel.instance.send(p76);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COMMAND_ACK.add((src, ph, pack) ->
        {
            assert(pack.result_param2_TRY(ph) == -1748637814);
            assert(pack.command_GET() == MAV_CMD.MAV_CMD_USER_1);
            assert(pack.result_GET() == MAV_RESULT.MAV_RESULT_DENIED);
            assert(pack.target_system_TRY(ph) == (char)44);
            assert(pack.progress_TRY(ph) == (char)170);
            assert(pack.target_component_TRY(ph) == (char)212);
        });
        DemoDevice.COMMAND_ACK p77 = LoopBackDemoChannel.new_COMMAND_ACK();
        PH.setPack(p77);
        p77.target_component_SET((char)212, PH) ;
        p77.target_system_SET((char)44, PH) ;
        p77.result_SET(MAV_RESULT.MAV_RESULT_DENIED) ;
        p77.result_param2_SET(-1748637814, PH) ;
        p77.command_SET(MAV_CMD.MAV_CMD_USER_1) ;
        p77.progress_SET((char)170, PH) ;
        LoopBackDemoChannel.instance.send(p77);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MANUAL_SETPOINT.add((src, ph, pack) ->
        {
            assert(pack.manual_override_switch_GET() == (char)211);
            assert(pack.pitch_GET() == -1.8164109E38F);
            assert(pack.time_boot_ms_GET() == 2786220467L);
            assert(pack.thrust_GET() == 8.793935E37F);
            assert(pack.yaw_GET() == -1.7502393E38F);
            assert(pack.mode_switch_GET() == (char)172);
            assert(pack.roll_GET() == -2.1762476E38F);
        });
        DemoDevice.MANUAL_SETPOINT p81 = LoopBackDemoChannel.new_MANUAL_SETPOINT();
        PH.setPack(p81);
        p81.mode_switch_SET((char)172) ;
        p81.manual_override_switch_SET((char)211) ;
        p81.pitch_SET(-1.8164109E38F) ;
        p81.yaw_SET(-1.7502393E38F) ;
        p81.time_boot_ms_SET(2786220467L) ;
        p81.thrust_SET(8.793935E37F) ;
        p81.roll_SET(-2.1762476E38F) ;
        LoopBackDemoChannel.instance.send(p81);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_roll_rate_GET() == -2.2364932E38F);
            assert(pack.time_boot_ms_GET() == 654467170L);
            assert(pack.type_mask_GET() == (char)164);
            assert(pack.body_pitch_rate_GET() == 2.399943E38F);
            assert(pack.thrust_GET() == 2.5521364E38F);
            assert(pack.body_yaw_rate_GET() == 1.1945531E38F);
            assert(pack.target_component_GET() == (char)197);
            assert(Arrays.equals(pack.q_GET(),  new float[] {4.517124E37F, 1.8872383E38F, -3.035244E38F, -1.9297534E38F}));
            assert(pack.target_system_GET() == (char)189);
        });
        DemoDevice.SET_ATTITUDE_TARGET p82 = LoopBackDemoChannel.new_SET_ATTITUDE_TARGET();
        PH.setPack(p82);
        p82.body_yaw_rate_SET(1.1945531E38F) ;
        p82.body_roll_rate_SET(-2.2364932E38F) ;
        p82.target_system_SET((char)189) ;
        p82.target_component_SET((char)197) ;
        p82.time_boot_ms_SET(654467170L) ;
        p82.type_mask_SET((char)164) ;
        p82.body_pitch_rate_SET(2.399943E38F) ;
        p82.q_SET(new float[] {4.517124E37F, 1.8872383E38F, -3.035244E38F, -1.9297534E38F}, 0) ;
        p82.thrust_SET(2.5521364E38F) ;
        LoopBackDemoChannel.instance.send(p82);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATTITUDE_TARGET.add((src, ph, pack) ->
        {
            assert(pack.body_pitch_rate_GET() == 3.2518651E38F);
            assert(pack.body_roll_rate_GET() == -3.0678193E38F);
            assert(pack.thrust_GET() == 1.680383E38F);
            assert(pack.body_yaw_rate_GET() == 9.911556E37F);
            assert(pack.type_mask_GET() == (char)45);
            assert(pack.time_boot_ms_GET() == 3297381379L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.9117562E37F, -9.063308E37F, -3.2710223E38F, 4.1909923E37F}));
        });
        DemoDevice.ATTITUDE_TARGET p83 = LoopBackDemoChannel.new_ATTITUDE_TARGET();
        PH.setPack(p83);
        p83.type_mask_SET((char)45) ;
        p83.thrust_SET(1.680383E38F) ;
        p83.body_yaw_rate_SET(9.911556E37F) ;
        p83.body_pitch_rate_SET(3.2518651E38F) ;
        p83.q_SET(new float[] {-1.9117562E37F, -9.063308E37F, -3.2710223E38F, 4.1909923E37F}, 0) ;
        p83.body_roll_rate_SET(-3.0678193E38F) ;
        p83.time_boot_ms_SET(3297381379L) ;
        LoopBackDemoChannel.instance.send(p83);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_LOCAL_NED.add((src, ph, pack) ->
        {
            assert(pack.afy_GET() == 2.0544708E38F);
            assert(pack.yaw_rate_GET() == 3.1279942E38F);
            assert(pack.z_GET() == 2.4904274E37F);
            assert(pack.type_mask_GET() == (char)37619);
            assert(pack.y_GET() == 2.5674338E38F);
            assert(pack.yaw_GET() == -3.2144088E38F);
            assert(pack.vz_GET() == 1.1807022E38F);
            assert(pack.vx_GET() == 1.9235882E38F);
            assert(pack.target_system_GET() == (char)115);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT);
            assert(pack.afx_GET() == 9.990389E37F);
            assert(pack.vy_GET() == -2.5009716E38F);
            assert(pack.time_boot_ms_GET() == 1064662458L);
            assert(pack.x_GET() == -7.0316173E37F);
            assert(pack.target_component_GET() == (char)229);
            assert(pack.afz_GET() == 2.6812611E38F);
        });
        DemoDevice.SET_POSITION_TARGET_LOCAL_NED p84 = LoopBackDemoChannel.new_SET_POSITION_TARGET_LOCAL_NED();
        PH.setPack(p84);
        p84.yaw_rate_SET(3.1279942E38F) ;
        p84.vy_SET(-2.5009716E38F) ;
        p84.afy_SET(2.0544708E38F) ;
        p84.x_SET(-7.0316173E37F) ;
        p84.yaw_SET(-3.2144088E38F) ;
        p84.target_component_SET((char)229) ;
        p84.afx_SET(9.990389E37F) ;
        p84.y_SET(2.5674338E38F) ;
        p84.afz_SET(2.6812611E38F) ;
        p84.time_boot_ms_SET(1064662458L) ;
        p84.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) ;
        p84.vx_SET(1.9235882E38F) ;
        p84.target_system_SET((char)115) ;
        p84.z_SET(2.4904274E37F) ;
        p84.type_mask_SET((char)37619) ;
        p84.vz_SET(1.1807022E38F) ;
        LoopBackDemoChannel.instance.send(p84);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)189);
            assert(pack.time_boot_ms_GET() == 285820706L);
            assert(pack.vy_GET() == 2.7410644E38F);
            assert(pack.afz_GET() == -8.2935554E37F);
            assert(pack.target_system_GET() == (char)220);
            assert(pack.afy_GET() == 3.3103915E38F);
            assert(pack.lon_int_GET() == -1189850668);
            assert(pack.afx_GET() == -7.646733E37F);
            assert(pack.lat_int_GET() == -241977595);
            assert(pack.vz_GET() == -1.3701432E38F);
            assert(pack.alt_GET() == 2.4822287E37F);
            assert(pack.yaw_rate_GET() == -2.9016577E38F);
            assert(pack.vx_GET() == 2.8193367E38F);
            assert(pack.yaw_GET() == 2.9589622E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED);
            assert(pack.type_mask_GET() == (char)49641);
        });
        DemoDevice.SET_POSITION_TARGET_GLOBAL_INT p86 = LoopBackDemoChannel.new_SET_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p86);
        p86.vz_SET(-1.3701432E38F) ;
        p86.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_LOCAL_OFFSET_NED) ;
        p86.yaw_SET(2.9589622E38F) ;
        p86.yaw_rate_SET(-2.9016577E38F) ;
        p86.alt_SET(2.4822287E37F) ;
        p86.afx_SET(-7.646733E37F) ;
        p86.lat_int_SET(-241977595) ;
        p86.time_boot_ms_SET(285820706L) ;
        p86.afz_SET(-8.2935554E37F) ;
        p86.vy_SET(2.7410644E38F) ;
        p86.target_system_SET((char)220) ;
        p86.type_mask_SET((char)49641) ;
        p86.vx_SET(2.8193367E38F) ;
        p86.afy_SET(3.3103915E38F) ;
        p86.lon_int_SET(-1189850668) ;
        p86.target_component_SET((char)189) ;
        LoopBackDemoChannel.instance.send(p86);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POSITION_TARGET_GLOBAL_INT.add((src, ph, pack) ->
        {
            assert(pack.yaw_rate_GET() == 2.516976E38F);
            assert(pack.coordinate_frame_GET() == MAV_FRAME.MAV_FRAME_GLOBAL);
            assert(pack.lat_int_GET() == -1139847961);
            assert(pack.afx_GET() == -2.5563394E37F);
            assert(pack.vx_GET() == -2.5845017E38F);
            assert(pack.type_mask_GET() == (char)54157);
            assert(pack.time_boot_ms_GET() == 4052015607L);
            assert(pack.yaw_GET() == 2.3530956E38F);
            assert(pack.lon_int_GET() == 705620897);
            assert(pack.vz_GET() == -1.0655469E38F);
            assert(pack.afz_GET() == 3.2634079E38F);
            assert(pack.vy_GET() == -8.92924E37F);
            assert(pack.afy_GET() == 1.3648426E38F);
            assert(pack.alt_GET() == -9.588677E37F);
        });
        DemoDevice.POSITION_TARGET_GLOBAL_INT p87 = LoopBackDemoChannel.new_POSITION_TARGET_GLOBAL_INT();
        PH.setPack(p87);
        p87.time_boot_ms_SET(4052015607L) ;
        p87.lat_int_SET(-1139847961) ;
        p87.type_mask_SET((char)54157) ;
        p87.vx_SET(-2.5845017E38F) ;
        p87.afz_SET(3.2634079E38F) ;
        p87.afy_SET(1.3648426E38F) ;
        p87.afx_SET(-2.5563394E37F) ;
        p87.yaw_rate_SET(2.516976E38F) ;
        p87.vz_SET(-1.0655469E38F) ;
        p87.vy_SET(-8.92924E37F) ;
        p87.alt_SET(-9.588677E37F) ;
        p87.lon_int_SET(705620897) ;
        p87.coordinate_frame_SET(MAV_FRAME.MAV_FRAME_GLOBAL) ;
        p87.yaw_SET(2.3530956E38F) ;
        LoopBackDemoChannel.instance.send(p87);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 1.6773405E38F);
            assert(pack.time_boot_ms_GET() == 1153423554L);
            assert(pack.roll_GET() == -1.7627047E38F);
            assert(pack.pitch_GET() == -3.6376702E37F);
            assert(pack.z_GET() == 2.5254539E38F);
            assert(pack.x_GET() == -4.0420256E37F);
            assert(pack.yaw_GET() == -1.5270037E38F);
        });
        DemoDevice.LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET p89 = LoopBackDemoChannel.new_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET();
        PH.setPack(p89);
        p89.x_SET(-4.0420256E37F) ;
        p89.y_SET(1.6773405E38F) ;
        p89.z_SET(2.5254539E38F) ;
        p89.pitch_SET(-3.6376702E37F) ;
        p89.time_boot_ms_SET(1153423554L) ;
        p89.roll_SET(-1.7627047E38F) ;
        p89.yaw_SET(-1.5270037E38F) ;
        LoopBackDemoChannel.instance.send(p89);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE.add((src, ph, pack) ->
        {
            assert(pack.rollspeed_GET() == 8.830417E37F);
            assert(pack.vx_GET() == (short)29216);
            assert(pack.yawspeed_GET() == 5.8422756E37F);
            assert(pack.zacc_GET() == (short)30641);
            assert(pack.alt_GET() == -632645325);
            assert(pack.pitchspeed_GET() == 1.999073E37F);
            assert(pack.yacc_GET() == (short) -1537);
            assert(pack.pitch_GET() == 2.1464838E38F);
            assert(pack.vz_GET() == (short)14335);
            assert(pack.xacc_GET() == (short)8746);
            assert(pack.lat_GET() == -279136646);
            assert(pack.yaw_GET() == -1.2846869E38F);
            assert(pack.vy_GET() == (short)23388);
            assert(pack.roll_GET() == 1.7259884E37F);
            assert(pack.time_usec_GET() == 2337119038132268489L);
            assert(pack.lon_GET() == 2146020401);
        });
        DemoDevice.HIL_STATE p90 = LoopBackDemoChannel.new_HIL_STATE();
        PH.setPack(p90);
        p90.yacc_SET((short) -1537) ;
        p90.vx_SET((short)29216) ;
        p90.pitch_SET(2.1464838E38F) ;
        p90.vy_SET((short)23388) ;
        p90.xacc_SET((short)8746) ;
        p90.lon_SET(2146020401) ;
        p90.yaw_SET(-1.2846869E38F) ;
        p90.lat_SET(-279136646) ;
        p90.time_usec_SET(2337119038132268489L) ;
        p90.rollspeed_SET(8.830417E37F) ;
        p90.zacc_SET((short)30641) ;
        p90.pitchspeed_SET(1.999073E37F) ;
        p90.vz_SET((short)14335) ;
        p90.yawspeed_SET(5.8422756E37F) ;
        p90.roll_SET(1.7259884E37F) ;
        p90.alt_SET(-632645325) ;
        LoopBackDemoChannel.instance.send(p90);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.aux2_GET() == -2.9238961E38F);
            assert(pack.time_usec_GET() == 4068945578262664434L);
            assert(pack.nav_mode_GET() == (char)227);
            assert(pack.aux1_GET() == -3.2663249E38F);
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_ARMED);
            assert(pack.pitch_elevator_GET() == 2.282091E38F);
            assert(pack.throttle_GET() == 3.0565768E38F);
            assert(pack.aux3_GET() == 3.144545E38F);
            assert(pack.aux4_GET() == -1.2702386E38F);
            assert(pack.roll_ailerons_GET() == 4.1240576E37F);
            assert(pack.yaw_rudder_GET() == 8.920731E37F);
        });
        DemoDevice.HIL_CONTROLS p91 = LoopBackDemoChannel.new_HIL_CONTROLS();
        PH.setPack(p91);
        p91.aux1_SET(-3.2663249E38F) ;
        p91.aux4_SET(-1.2702386E38F) ;
        p91.throttle_SET(3.0565768E38F) ;
        p91.roll_ailerons_SET(4.1240576E37F) ;
        p91.time_usec_SET(4068945578262664434L) ;
        p91.aux2_SET(-2.9238961E38F) ;
        p91.nav_mode_SET((char)227) ;
        p91.yaw_rudder_SET(8.920731E37F) ;
        p91.pitch_elevator_SET(2.282091E38F) ;
        p91.mode_SET(MAV_MODE.MAV_MODE_STABILIZE_ARMED) ;
        p91.aux3_SET(3.144545E38F) ;
        LoopBackDemoChannel.instance.send(p91);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_RC_INPUTS_RAW.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 6339215495265077309L);
            assert(pack.chan10_raw_GET() == (char)47302);
            assert(pack.chan8_raw_GET() == (char)51804);
            assert(pack.rssi_GET() == (char)36);
            assert(pack.chan11_raw_GET() == (char)53233);
            assert(pack.chan4_raw_GET() == (char)37747);
            assert(pack.chan12_raw_GET() == (char)14710);
            assert(pack.chan5_raw_GET() == (char)40047);
            assert(pack.chan2_raw_GET() == (char)36116);
            assert(pack.chan1_raw_GET() == (char)8445);
            assert(pack.chan3_raw_GET() == (char)38998);
            assert(pack.chan9_raw_GET() == (char)51036);
            assert(pack.chan7_raw_GET() == (char)22850);
            assert(pack.chan6_raw_GET() == (char)47174);
        });
        DemoDevice.HIL_RC_INPUTS_RAW p92 = LoopBackDemoChannel.new_HIL_RC_INPUTS_RAW();
        PH.setPack(p92);
        p92.chan8_raw_SET((char)51804) ;
        p92.chan11_raw_SET((char)53233) ;
        p92.chan4_raw_SET((char)37747) ;
        p92.chan12_raw_SET((char)14710) ;
        p92.chan10_raw_SET((char)47302) ;
        p92.chan9_raw_SET((char)51036) ;
        p92.chan3_raw_SET((char)38998) ;
        p92.chan7_raw_SET((char)22850) ;
        p92.rssi_SET((char)36) ;
        p92.chan6_raw_SET((char)47174) ;
        p92.chan1_raw_SET((char)8445) ;
        p92.time_usec_SET(6339215495265077309L) ;
        p92.chan2_raw_SET((char)36116) ;
        p92.chan5_raw_SET((char)40047) ;
        LoopBackDemoChannel.instance.send(p92);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_ACTUATOR_CONTROLS.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 43285444428839384L);
            assert(pack.flags_GET() == 1786432054896214599L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {4.2650977E37F, -3.3824796E38F, -2.7748732E38F, 1.2588387E38F, 2.9826291E38F, -5.22205E37F, -7.9077525E37F, 1.3123312E38F, -5.811204E37F, -1.7675674E38F, 6.176729E36F, 1.7848141E38F, -1.1418276E38F, -6.587217E37F, -1.5707424E38F, -7.678632E37F}));
            assert(pack.mode_GET() == MAV_MODE.MAV_MODE_STABILIZE_DISARMED);
        });
        DemoDevice.HIL_ACTUATOR_CONTROLS p93 = LoopBackDemoChannel.new_HIL_ACTUATOR_CONTROLS();
        PH.setPack(p93);
        p93.controls_SET(new float[] {4.2650977E37F, -3.3824796E38F, -2.7748732E38F, 1.2588387E38F, 2.9826291E38F, -5.22205E37F, -7.9077525E37F, 1.3123312E38F, -5.811204E37F, -1.7675674E38F, 6.176729E36F, 1.7848141E38F, -1.1418276E38F, -6.587217E37F, -1.5707424E38F, -7.678632E37F}, 0) ;
        p93.time_usec_SET(43285444428839384L) ;
        p93.flags_SET(1786432054896214599L) ;
        p93.mode_SET(MAV_MODE.MAV_MODE_STABILIZE_DISARMED) ;
        LoopBackDemoChannel.instance.send(p93);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.flow_comp_m_y_GET() == -2.8836587E38F);
            assert(pack.flow_x_GET() == (short)23876);
            assert(pack.time_usec_GET() == 4443489889678025567L);
            assert(pack.flow_rate_x_TRY(ph) == -1.6338675E37F);
            assert(pack.quality_GET() == (char)25);
            assert(pack.flow_y_GET() == (short)928);
            assert(pack.flow_rate_y_TRY(ph) == 1.0298912E37F);
            assert(pack.sensor_id_GET() == (char)153);
            assert(pack.flow_comp_m_x_GET() == -2.909833E38F);
            assert(pack.ground_distance_GET() == -2.2155667E38F);
        });
        DemoDevice.OPTICAL_FLOW p100 = LoopBackDemoChannel.new_OPTICAL_FLOW();
        PH.setPack(p100);
        p100.sensor_id_SET((char)153) ;
        p100.quality_SET((char)25) ;
        p100.ground_distance_SET(-2.2155667E38F) ;
        p100.time_usec_SET(4443489889678025567L) ;
        p100.flow_rate_y_SET(1.0298912E37F, PH) ;
        p100.flow_y_SET((short)928) ;
        p100.flow_x_SET((short)23876) ;
        p100.flow_comp_m_y_SET(-2.8836587E38F) ;
        p100.flow_rate_x_SET(-1.6338675E37F, PH) ;
        p100.flow_comp_m_x_SET(-2.909833E38F) ;
        LoopBackDemoChannel.instance.send(p100);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GLOBAL_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.yaw_GET() == 1.5352174E37F);
            assert(pack.z_GET() == -1.7972693E38F);
            assert(pack.pitch_GET() == 1.0755628E38F);
            assert(pack.roll_GET() == 2.5142128E38F);
            assert(pack.y_GET() == -2.942081E38F);
            assert(pack.x_GET() == -5.985858E37F);
            assert(pack.usec_GET() == 4940180743259617261L);
        });
        DemoDevice.GLOBAL_VISION_POSITION_ESTIMATE p101 = LoopBackDemoChannel.new_GLOBAL_VISION_POSITION_ESTIMATE();
        PH.setPack(p101);
        p101.y_SET(-2.942081E38F) ;
        p101.yaw_SET(1.5352174E37F) ;
        p101.z_SET(-1.7972693E38F) ;
        p101.roll_SET(2.5142128E38F) ;
        p101.pitch_SET(1.0755628E38F) ;
        p101.usec_SET(4940180743259617261L) ;
        p101.x_SET(-5.985858E37F) ;
        LoopBackDemoChannel.instance.send(p101);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == 2.254764E38F);
            assert(pack.z_GET() == 3.158713E38F);
            assert(pack.pitch_GET() == 3.4000846E37F);
            assert(pack.x_GET() == 1.2317941E38F);
            assert(pack.yaw_GET() == 1.8320601E37F);
            assert(pack.usec_GET() == 3972100535123514698L);
            assert(pack.y_GET() == 1.2683625E38F);
        });
        DemoDevice.VISION_POSITION_ESTIMATE p102 = LoopBackDemoChannel.new_VISION_POSITION_ESTIMATE();
        PH.setPack(p102);
        p102.roll_SET(2.254764E38F) ;
        p102.z_SET(3.158713E38F) ;
        p102.usec_SET(3972100535123514698L) ;
        p102.x_SET(1.2317941E38F) ;
        p102.yaw_SET(1.8320601E37F) ;
        p102.y_SET(1.2683625E38F) ;
        p102.pitch_SET(3.4000846E37F) ;
        LoopBackDemoChannel.instance.send(p102);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VISION_SPEED_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.usec_GET() == 4710686769722192883L);
            assert(pack.y_GET() == -2.8769136E38F);
            assert(pack.x_GET() == -6.7708053E37F);
            assert(pack.z_GET() == 4.0926858E37F);
        });
        DemoDevice.VISION_SPEED_ESTIMATE p103 = LoopBackDemoChannel.new_VISION_SPEED_ESTIMATE();
        PH.setPack(p103);
        p103.usec_SET(4710686769722192883L) ;
        p103.y_SET(-2.8769136E38F) ;
        p103.x_SET(-6.7708053E37F) ;
        p103.z_SET(4.0926858E37F) ;
        LoopBackDemoChannel.instance.send(p103);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VICON_POSITION_ESTIMATE.add((src, ph, pack) ->
        {
            assert(pack.roll_GET() == 3.005012E38F);
            assert(pack.z_GET() == 6.9556115E37F);
            assert(pack.y_GET() == -2.0480202E38F);
            assert(pack.yaw_GET() == -1.6670365E37F);
            assert(pack.pitch_GET() == -1.1203757E38F);
            assert(pack.x_GET() == 2.7504793E38F);
            assert(pack.usec_GET() == 6669023869848296454L);
        });
        DemoDevice.VICON_POSITION_ESTIMATE p104 = LoopBackDemoChannel.new_VICON_POSITION_ESTIMATE();
        PH.setPack(p104);
        p104.x_SET(2.7504793E38F) ;
        p104.y_SET(-2.0480202E38F) ;
        p104.pitch_SET(-1.1203757E38F) ;
        p104.roll_SET(3.005012E38F) ;
        p104.z_SET(6.9556115E37F) ;
        p104.usec_SET(6669023869848296454L) ;
        p104.yaw_SET(-1.6670365E37F) ;
        LoopBackDemoChannel.instance.send(p104);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGHRES_IMU.add((src, ph, pack) ->
        {
            assert(pack.xmag_GET() == -1.015623E38F);
            assert(pack.diff_pressure_GET() == -5.67736E37F);
            assert(pack.ygyro_GET() == 1.0392139E38F);
            assert(pack.xgyro_GET() == 1.0217696E38F);
            assert(pack.zgyro_GET() == 2.4151825E38F);
            assert(pack.temperature_GET() == -1.1136667E38F);
            assert(pack.ymag_GET() == -1.4260097E38F);
            assert(pack.abs_pressure_GET() == 1.9424522E38F);
            assert(pack.pressure_alt_GET() == -1.8143309E37F);
            assert(pack.time_usec_GET() == 3936159153294674851L);
            assert(pack.yacc_GET() == -3.2102753E38F);
            assert(pack.zmag_GET() == -4.204192E37F);
            assert(pack.zacc_GET() == -1.9156379E38F);
            assert(pack.xacc_GET() == 8.31933E36F);
            assert(pack.fields_updated_GET() == (char)23859);
        });
        DemoDevice.HIGHRES_IMU p105 = LoopBackDemoChannel.new_HIGHRES_IMU();
        PH.setPack(p105);
        p105.pressure_alt_SET(-1.8143309E37F) ;
        p105.ygyro_SET(1.0392139E38F) ;
        p105.xacc_SET(8.31933E36F) ;
        p105.fields_updated_SET((char)23859) ;
        p105.abs_pressure_SET(1.9424522E38F) ;
        p105.time_usec_SET(3936159153294674851L) ;
        p105.xgyro_SET(1.0217696E38F) ;
        p105.xmag_SET(-1.015623E38F) ;
        p105.ymag_SET(-1.4260097E38F) ;
        p105.zmag_SET(-4.204192E37F) ;
        p105.diff_pressure_SET(-5.67736E37F) ;
        p105.temperature_SET(-1.1136667E38F) ;
        p105.zacc_SET(-1.9156379E38F) ;
        p105.zgyro_SET(2.4151825E38F) ;
        p105.yacc_SET(-3.2102753E38F) ;
        LoopBackDemoChannel.instance.send(p105);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OPTICAL_FLOW_RAD.add((src, ph, pack) ->
        {
            assert(pack.quality_GET() == (char)48);
            assert(pack.distance_GET() == 6.449759E35F);
            assert(pack.integrated_xgyro_GET() == -1.0739802E38F);
            assert(pack.integrated_y_GET() == -8.3133464E37F);
            assert(pack.integration_time_us_GET() == 836563121L);
            assert(pack.time_usec_GET() == 4948639802715106602L);
            assert(pack.integrated_zgyro_GET() == -2.1748468E37F);
            assert(pack.temperature_GET() == (short)889);
            assert(pack.sensor_id_GET() == (char)217);
            assert(pack.integrated_x_GET() == -1.1658348E38F);
            assert(pack.time_delta_distance_us_GET() == 4254297067L);
            assert(pack.integrated_ygyro_GET() == -3.1732231E38F);
        });
        DemoDevice.OPTICAL_FLOW_RAD p106 = LoopBackDemoChannel.new_OPTICAL_FLOW_RAD();
        PH.setPack(p106);
        p106.integrated_x_SET(-1.1658348E38F) ;
        p106.integration_time_us_SET(836563121L) ;
        p106.quality_SET((char)48) ;
        p106.sensor_id_SET((char)217) ;
        p106.time_delta_distance_us_SET(4254297067L) ;
        p106.time_usec_SET(4948639802715106602L) ;
        p106.integrated_zgyro_SET(-2.1748468E37F) ;
        p106.distance_SET(6.449759E35F) ;
        p106.integrated_ygyro_SET(-3.1732231E38F) ;
        p106.integrated_xgyro_SET(-1.0739802E38F) ;
        p106.integrated_y_SET(-8.3133464E37F) ;
        p106.temperature_SET((short)889) ;
        LoopBackDemoChannel.instance.send(p106);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.diff_pressure_GET() == 1.8398498E38F);
            assert(pack.xgyro_GET() == 1.6741165E38F);
            assert(pack.zgyro_GET() == 9.253382E37F);
            assert(pack.fields_updated_GET() == 1219141459L);
            assert(pack.pressure_alt_GET() == -1.1752813E38F);
            assert(pack.yacc_GET() == 2.6405618E38F);
            assert(pack.ygyro_GET() == -2.9822306E38F);
            assert(pack.temperature_GET() == 1.9611916E38F);
            assert(pack.time_usec_GET() == 892707251548070989L);
            assert(pack.xacc_GET() == -1.5829436E38F);
            assert(pack.xmag_GET() == -8.278402E37F);
            assert(pack.abs_pressure_GET() == -2.4170343E38F);
            assert(pack.zacc_GET() == -3.0463246E38F);
            assert(pack.zmag_GET() == -1.630693E38F);
            assert(pack.ymag_GET() == -8.948354E37F);
        });
        DemoDevice.HIL_SENSOR p107 = LoopBackDemoChannel.new_HIL_SENSOR();
        PH.setPack(p107);
        p107.ymag_SET(-8.948354E37F) ;
        p107.diff_pressure_SET(1.8398498E38F) ;
        p107.yacc_SET(2.6405618E38F) ;
        p107.zmag_SET(-1.630693E38F) ;
        p107.fields_updated_SET(1219141459L) ;
        p107.pressure_alt_SET(-1.1752813E38F) ;
        p107.time_usec_SET(892707251548070989L) ;
        p107.zgyro_SET(9.253382E37F) ;
        p107.xacc_SET(-1.5829436E38F) ;
        p107.ygyro_SET(-2.9822306E38F) ;
        p107.xgyro_SET(1.6741165E38F) ;
        p107.zacc_SET(-3.0463246E38F) ;
        p107.temperature_SET(1.9611916E38F) ;
        p107.xmag_SET(-8.278402E37F) ;
        p107.abs_pressure_SET(-2.4170343E38F) ;
        LoopBackDemoChannel.instance.send(p107);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SIM_STATE.add((src, ph, pack) ->
        {
            assert(pack.vd_GET() == -3.3825933E37F);
            assert(pack.q2_GET() == -9.097917E37F);
            assert(pack.ygyro_GET() == -1.1599747E37F);
            assert(pack.alt_GET() == -2.4760224E38F);
            assert(pack.roll_GET() == -2.572524E38F);
            assert(pack.q3_GET() == -2.3203281E38F);
            assert(pack.pitch_GET() == 1.2524691E38F);
            assert(pack.zacc_GET() == 3.1602548E38F);
            assert(pack.std_dev_vert_GET() == 2.7022798E38F);
            assert(pack.lat_GET() == -2.2972253E38F);
            assert(pack.vn_GET() == 3.1652055E38F);
            assert(pack.xacc_GET() == 1.4879841E38F);
            assert(pack.q4_GET() == -2.8457663E38F);
            assert(pack.std_dev_horz_GET() == -3.1284605E38F);
            assert(pack.zgyro_GET() == -4.4926627E37F);
            assert(pack.ve_GET() == 1.6823734E38F);
            assert(pack.q1_GET() == 2.475472E38F);
            assert(pack.yacc_GET() == -1.4887353E38F);
            assert(pack.yaw_GET() == -3.36517E38F);
            assert(pack.lon_GET() == 2.9956218E38F);
            assert(pack.xgyro_GET() == 3.9951796E37F);
        });
        DemoDevice.SIM_STATE p108 = LoopBackDemoChannel.new_SIM_STATE();
        PH.setPack(p108);
        p108.q3_SET(-2.3203281E38F) ;
        p108.xgyro_SET(3.9951796E37F) ;
        p108.q4_SET(-2.8457663E38F) ;
        p108.yaw_SET(-3.36517E38F) ;
        p108.lon_SET(2.9956218E38F) ;
        p108.q1_SET(2.475472E38F) ;
        p108.roll_SET(-2.572524E38F) ;
        p108.std_dev_horz_SET(-3.1284605E38F) ;
        p108.vn_SET(3.1652055E38F) ;
        p108.ygyro_SET(-1.1599747E37F) ;
        p108.vd_SET(-3.3825933E37F) ;
        p108.ve_SET(1.6823734E38F) ;
        p108.alt_SET(-2.4760224E38F) ;
        p108.q2_SET(-9.097917E37F) ;
        p108.zacc_SET(3.1602548E38F) ;
        p108.yacc_SET(-1.4887353E38F) ;
        p108.xacc_SET(1.4879841E38F) ;
        p108.pitch_SET(1.2524691E38F) ;
        p108.std_dev_vert_SET(2.7022798E38F) ;
        p108.zgyro_SET(-4.4926627E37F) ;
        p108.lat_SET(-2.2972253E38F) ;
        LoopBackDemoChannel.instance.send(p108);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RADIO_STATUS.add((src, ph, pack) ->
        {
            assert(pack.remnoise_GET() == (char)183);
            assert(pack.remrssi_GET() == (char)109);
            assert(pack.fixed__GET() == (char)5516);
            assert(pack.txbuf_GET() == (char)111);
            assert(pack.rssi_GET() == (char)114);
            assert(pack.noise_GET() == (char)227);
            assert(pack.rxerrors_GET() == (char)30446);
        });
        DemoDevice.RADIO_STATUS p109 = LoopBackDemoChannel.new_RADIO_STATUS();
        PH.setPack(p109);
        p109.noise_SET((char)227) ;
        p109.fixed__SET((char)5516) ;
        p109.rssi_SET((char)114) ;
        p109.remnoise_SET((char)183) ;
        p109.rxerrors_SET((char)30446) ;
        p109.remrssi_SET((char)109) ;
        p109.txbuf_SET((char)111) ;
        LoopBackDemoChannel.instance.send(p109);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FILE_TRANSFER_PROTOCOL.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)115);
            assert(pack.target_network_GET() == (char)136);
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)100, (char)255, (char)81, (char)4, (char)215, (char)176, (char)242, (char)131, (char)53, (char)29, (char)253, (char)84, (char)21, (char)32, (char)40, (char)230, (char)229, (char)34, (char)234, (char)200, (char)166, (char)183, (char)0, (char)237, (char)179, (char)106, (char)187, (char)236, (char)140, (char)122, (char)13, (char)194, (char)150, (char)240, (char)253, (char)10, (char)64, (char)153, (char)148, (char)212, (char)92, (char)0, (char)191, (char)99, (char)172, (char)162, (char)64, (char)239, (char)212, (char)139, (char)110, (char)195, (char)49, (char)193, (char)21, (char)131, (char)8, (char)73, (char)178, (char)39, (char)19, (char)150, (char)231, (char)90, (char)17, (char)214, (char)31, (char)93, (char)151, (char)173, (char)160, (char)158, (char)91, (char)139, (char)18, (char)60, (char)156, (char)90, (char)74, (char)204, (char)74, (char)111, (char)0, (char)240, (char)112, (char)94, (char)37, (char)98, (char)166, (char)250, (char)131, (char)56, (char)244, (char)92, (char)111, (char)45, (char)242, (char)45, (char)214, (char)26, (char)240, (char)83, (char)147, (char)119, (char)222, (char)132, (char)131, (char)96, (char)111, (char)38, (char)56, (char)8, (char)113, (char)28, (char)66, (char)196, (char)84, (char)213, (char)141, (char)140, (char)5, (char)8, (char)128, (char)55, (char)203, (char)141, (char)68, (char)41, (char)23, (char)83, (char)178, (char)218, (char)57, (char)233, (char)174, (char)175, (char)88, (char)187, (char)123, (char)49, (char)149, (char)127, (char)64, (char)143, (char)130, (char)69, (char)149, (char)183, (char)153, (char)240, (char)156, (char)13, (char)121, (char)27, (char)161, (char)99, (char)135, (char)226, (char)246, (char)116, (char)234, (char)16, (char)80, (char)216, (char)208, (char)109, (char)199, (char)188, (char)27, (char)126, (char)255, (char)208, (char)158, (char)131, (char)54, (char)78, (char)34, (char)81, (char)173, (char)196, (char)65, (char)100, (char)154, (char)44, (char)201, (char)93, (char)126, (char)33, (char)223, (char)168, (char)3, (char)99, (char)129, (char)112, (char)241, (char)45, (char)122, (char)108, (char)152, (char)68, (char)57, (char)108, (char)227, (char)17, (char)223, (char)104, (char)169, (char)108, (char)52, (char)125, (char)196, (char)92, (char)165, (char)252, (char)116, (char)10, (char)41, (char)118, (char)123, (char)104, (char)224, (char)240, (char)212, (char)177, (char)88, (char)121, (char)127, (char)152, (char)190, (char)186, (char)63, (char)173, (char)250, (char)196, (char)62, (char)225, (char)122, (char)253, (char)189, (char)9, (char)194, (char)127, (char)181, (char)149, (char)35, (char)128, (char)187, (char)235, (char)116, (char)159, (char)80}));
            assert(pack.target_system_GET() == (char)24);
        });
        DemoDevice.FILE_TRANSFER_PROTOCOL p110 = LoopBackDemoChannel.new_FILE_TRANSFER_PROTOCOL();
        PH.setPack(p110);
        p110.target_component_SET((char)115) ;
        p110.target_system_SET((char)24) ;
        p110.target_network_SET((char)136) ;
        p110.payload_SET(new char[] {(char)100, (char)255, (char)81, (char)4, (char)215, (char)176, (char)242, (char)131, (char)53, (char)29, (char)253, (char)84, (char)21, (char)32, (char)40, (char)230, (char)229, (char)34, (char)234, (char)200, (char)166, (char)183, (char)0, (char)237, (char)179, (char)106, (char)187, (char)236, (char)140, (char)122, (char)13, (char)194, (char)150, (char)240, (char)253, (char)10, (char)64, (char)153, (char)148, (char)212, (char)92, (char)0, (char)191, (char)99, (char)172, (char)162, (char)64, (char)239, (char)212, (char)139, (char)110, (char)195, (char)49, (char)193, (char)21, (char)131, (char)8, (char)73, (char)178, (char)39, (char)19, (char)150, (char)231, (char)90, (char)17, (char)214, (char)31, (char)93, (char)151, (char)173, (char)160, (char)158, (char)91, (char)139, (char)18, (char)60, (char)156, (char)90, (char)74, (char)204, (char)74, (char)111, (char)0, (char)240, (char)112, (char)94, (char)37, (char)98, (char)166, (char)250, (char)131, (char)56, (char)244, (char)92, (char)111, (char)45, (char)242, (char)45, (char)214, (char)26, (char)240, (char)83, (char)147, (char)119, (char)222, (char)132, (char)131, (char)96, (char)111, (char)38, (char)56, (char)8, (char)113, (char)28, (char)66, (char)196, (char)84, (char)213, (char)141, (char)140, (char)5, (char)8, (char)128, (char)55, (char)203, (char)141, (char)68, (char)41, (char)23, (char)83, (char)178, (char)218, (char)57, (char)233, (char)174, (char)175, (char)88, (char)187, (char)123, (char)49, (char)149, (char)127, (char)64, (char)143, (char)130, (char)69, (char)149, (char)183, (char)153, (char)240, (char)156, (char)13, (char)121, (char)27, (char)161, (char)99, (char)135, (char)226, (char)246, (char)116, (char)234, (char)16, (char)80, (char)216, (char)208, (char)109, (char)199, (char)188, (char)27, (char)126, (char)255, (char)208, (char)158, (char)131, (char)54, (char)78, (char)34, (char)81, (char)173, (char)196, (char)65, (char)100, (char)154, (char)44, (char)201, (char)93, (char)126, (char)33, (char)223, (char)168, (char)3, (char)99, (char)129, (char)112, (char)241, (char)45, (char)122, (char)108, (char)152, (char)68, (char)57, (char)108, (char)227, (char)17, (char)223, (char)104, (char)169, (char)108, (char)52, (char)125, (char)196, (char)92, (char)165, (char)252, (char)116, (char)10, (char)41, (char)118, (char)123, (char)104, (char)224, (char)240, (char)212, (char)177, (char)88, (char)121, (char)127, (char)152, (char)190, (char)186, (char)63, (char)173, (char)250, (char)196, (char)62, (char)225, (char)122, (char)253, (char)189, (char)9, (char)194, (char)127, (char)181, (char)149, (char)35, (char)128, (char)187, (char)235, (char)116, (char)159, (char)80}, 0) ;
        LoopBackDemoChannel.instance.send(p110);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TIMESYNC.add((src, ph, pack) ->
        {
            assert(pack.ts1_GET() == -6445961183180254128L);
            assert(pack.tc1_GET() == -7866401472738887507L);
        });
        DemoDevice.TIMESYNC p111 = LoopBackDemoChannel.new_TIMESYNC();
        PH.setPack(p111);
        p111.tc1_SET(-7866401472738887507L) ;
        p111.ts1_SET(-6445961183180254128L) ;
        LoopBackDemoChannel.instance.send(p111);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_TRIGGER.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 5492455397194988633L);
            assert(pack.seq_GET() == 3246112548L);
        });
        DemoDevice.CAMERA_TRIGGER p112 = LoopBackDemoChannel.new_CAMERA_TRIGGER();
        PH.setPack(p112);
        p112.seq_SET(3246112548L) ;
        p112.time_usec_SET(5492455397194988633L) ;
        LoopBackDemoChannel.instance.send(p112);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_GPS.add((src, ph, pack) ->
        {
            assert(pack.satellites_visible_GET() == (char)97);
            assert(pack.cog_GET() == (char)61759);
            assert(pack.eph_GET() == (char)9895);
            assert(pack.ve_GET() == (short)32165);
            assert(pack.time_usec_GET() == 864694541184980915L);
            assert(pack.lat_GET() == -2011545035);
            assert(pack.vd_GET() == (short) -5091);
            assert(pack.lon_GET() == 1038909358);
            assert(pack.vn_GET() == (short)18292);
            assert(pack.fix_type_GET() == (char)111);
            assert(pack.vel_GET() == (char)46452);
            assert(pack.alt_GET() == 712568006);
            assert(pack.epv_GET() == (char)27409);
        });
        DemoDevice.HIL_GPS p113 = LoopBackDemoChannel.new_HIL_GPS();
        PH.setPack(p113);
        p113.alt_SET(712568006) ;
        p113.epv_SET((char)27409) ;
        p113.ve_SET((short)32165) ;
        p113.lat_SET(-2011545035) ;
        p113.eph_SET((char)9895) ;
        p113.vel_SET((char)46452) ;
        p113.fix_type_SET((char)111) ;
        p113.cog_SET((char)61759) ;
        p113.vd_SET((short) -5091) ;
        p113.lon_SET(1038909358) ;
        p113.satellites_visible_SET((char)97) ;
        p113.vn_SET((short)18292) ;
        p113.time_usec_SET(864694541184980915L) ;
        LoopBackDemoChannel.instance.send(p113);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_OPTICAL_FLOW.add((src, ph, pack) ->
        {
            assert(pack.integrated_ygyro_GET() == -2.2313997E38F);
            assert(pack.temperature_GET() == (short) -15217);
            assert(pack.distance_GET() == -2.3432325E38F);
            assert(pack.integrated_zgyro_GET() == 2.019756E38F);
            assert(pack.integration_time_us_GET() == 132141683L);
            assert(pack.integrated_y_GET() == 4.26102E37F);
            assert(pack.sensor_id_GET() == (char)62);
            assert(pack.quality_GET() == (char)237);
            assert(pack.time_delta_distance_us_GET() == 3686475706L);
            assert(pack.integrated_x_GET() == -7.689507E36F);
            assert(pack.integrated_xgyro_GET() == 2.4241802E38F);
            assert(pack.time_usec_GET() == 5181743791999443601L);
        });
        DemoDevice.HIL_OPTICAL_FLOW p114 = LoopBackDemoChannel.new_HIL_OPTICAL_FLOW();
        PH.setPack(p114);
        p114.integrated_ygyro_SET(-2.2313997E38F) ;
        p114.quality_SET((char)237) ;
        p114.sensor_id_SET((char)62) ;
        p114.distance_SET(-2.3432325E38F) ;
        p114.time_usec_SET(5181743791999443601L) ;
        p114.time_delta_distance_us_SET(3686475706L) ;
        p114.integrated_y_SET(4.26102E37F) ;
        p114.integrated_zgyro_SET(2.019756E38F) ;
        p114.integrated_xgyro_SET(2.4241802E38F) ;
        p114.integrated_x_SET(-7.689507E36F) ;
        p114.temperature_SET((short) -15217) ;
        p114.integration_time_us_SET(132141683L) ;
        LoopBackDemoChannel.instance.send(p114);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIL_STATE_QUATERNION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.attitude_quaternion_GET(),  new float[] {-2.7087038E38F, -1.9520578E38F, -1.0361692E37F, 2.8321582E38F}));
            assert(pack.xacc_GET() == (short)12673);
            assert(pack.yawspeed_GET() == -1.981582E38F);
            assert(pack.alt_GET() == -1780846790);
            assert(pack.true_airspeed_GET() == (char)63892);
            assert(pack.zacc_GET() == (short) -14127);
            assert(pack.vy_GET() == (short)21146);
            assert(pack.lat_GET() == -768413536);
            assert(pack.ind_airspeed_GET() == (char)63594);
            assert(pack.lon_GET() == -957795084);
            assert(pack.vx_GET() == (short)24147);
            assert(pack.pitchspeed_GET() == -2.2708608E38F);
            assert(pack.time_usec_GET() == 1500359300990954568L);
            assert(pack.yacc_GET() == (short) -29618);
            assert(pack.rollspeed_GET() == -2.8562411E38F);
            assert(pack.vz_GET() == (short) -24043);
        });
        DemoDevice.HIL_STATE_QUATERNION p115 = LoopBackDemoChannel.new_HIL_STATE_QUATERNION();
        PH.setPack(p115);
        p115.vy_SET((short)21146) ;
        p115.lon_SET(-957795084) ;
        p115.yacc_SET((short) -29618) ;
        p115.zacc_SET((short) -14127) ;
        p115.attitude_quaternion_SET(new float[] {-2.7087038E38F, -1.9520578E38F, -1.0361692E37F, 2.8321582E38F}, 0) ;
        p115.true_airspeed_SET((char)63892) ;
        p115.ind_airspeed_SET((char)63594) ;
        p115.xacc_SET((short)12673) ;
        p115.pitchspeed_SET(-2.2708608E38F) ;
        p115.lat_SET(-768413536) ;
        p115.vz_SET((short) -24043) ;
        p115.alt_SET(-1780846790) ;
        p115.yawspeed_SET(-1.981582E38F) ;
        p115.vx_SET((short)24147) ;
        p115.time_usec_SET(1500359300990954568L) ;
        p115.rollspeed_SET(-2.8562411E38F) ;
        LoopBackDemoChannel.instance.send(p115);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU2.add((src, ph, pack) ->
        {
            assert(pack.ygyro_GET() == (short) -9980);
            assert(pack.xacc_GET() == (short) -16474);
            assert(pack.ymag_GET() == (short) -22839);
            assert(pack.zacc_GET() == (short) -18496);
            assert(pack.xmag_GET() == (short)19897);
            assert(pack.yacc_GET() == (short) -15917);
            assert(pack.time_boot_ms_GET() == 2751357070L);
            assert(pack.zmag_GET() == (short)2900);
            assert(pack.zgyro_GET() == (short)19037);
            assert(pack.xgyro_GET() == (short)27698);
        });
        DemoDevice.SCALED_IMU2 p116 = LoopBackDemoChannel.new_SCALED_IMU2();
        PH.setPack(p116);
        p116.xgyro_SET((short)27698) ;
        p116.zgyro_SET((short)19037) ;
        p116.time_boot_ms_SET(2751357070L) ;
        p116.xacc_SET((short) -16474) ;
        p116.xmag_SET((short)19897) ;
        p116.zmag_SET((short)2900) ;
        p116.yacc_SET((short) -15917) ;
        p116.ymag_SET((short) -22839) ;
        p116.ygyro_SET((short) -9980) ;
        p116.zacc_SET((short) -18496) ;
        LoopBackDemoChannel.instance.send(p116);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.end_GET() == (char)63155);
            assert(pack.target_system_GET() == (char)82);
            assert(pack.start_GET() == (char)18477);
            assert(pack.target_component_GET() == (char)167);
        });
        DemoDevice.LOG_REQUEST_LIST p117 = LoopBackDemoChannel.new_LOG_REQUEST_LIST();
        PH.setPack(p117);
        p117.start_SET((char)18477) ;
        p117.target_system_SET((char)82) ;
        p117.end_SET((char)63155) ;
        p117.target_component_SET((char)167) ;
        LoopBackDemoChannel.instance.send(p117);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ENTRY.add((src, ph, pack) ->
        {
            assert(pack.size_GET() == 387981772L);
            assert(pack.time_utc_GET() == 2502414005L);
            assert(pack.id_GET() == (char)12735);
            assert(pack.num_logs_GET() == (char)5973);
            assert(pack.last_log_num_GET() == (char)50461);
        });
        DemoDevice.LOG_ENTRY p118 = LoopBackDemoChannel.new_LOG_ENTRY();
        PH.setPack(p118);
        p118.id_SET((char)12735) ;
        p118.last_log_num_SET((char)50461) ;
        p118.num_logs_SET((char)5973) ;
        p118.time_utc_SET(2502414005L) ;
        p118.size_SET(387981772L) ;
        LoopBackDemoChannel.instance.send(p118);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_DATA.add((src, ph, pack) ->
        {
            assert(pack.ofs_GET() == 2244547106L);
            assert(pack.target_component_GET() == (char)41);
            assert(pack.target_system_GET() == (char)232);
            assert(pack.id_GET() == (char)63150);
            assert(pack.count_GET() == 3059762491L);
        });
        DemoDevice.LOG_REQUEST_DATA p119 = LoopBackDemoChannel.new_LOG_REQUEST_DATA();
        PH.setPack(p119);
        p119.target_system_SET((char)232) ;
        p119.id_SET((char)63150) ;
        p119.ofs_SET(2244547106L) ;
        p119.target_component_SET((char)41) ;
        p119.count_SET(3059762491L) ;
        LoopBackDemoChannel.instance.send(p119);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_DATA.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)145, (char)69, (char)217, (char)127, (char)136, (char)132, (char)44, (char)186, (char)50, (char)17, (char)212, (char)127, (char)143, (char)98, (char)174, (char)154, (char)251, (char)132, (char)40, (char)35, (char)132, (char)3, (char)210, (char)66, (char)70, (char)86, (char)143, (char)107, (char)52, (char)250, (char)122, (char)236, (char)191, (char)178, (char)210, (char)47, (char)23, (char)162, (char)0, (char)227, (char)206, (char)111, (char)17, (char)137, (char)181, (char)47, (char)94, (char)60, (char)222, (char)197, (char)195, (char)88, (char)153, (char)119, (char)221, (char)139, (char)65, (char)121, (char)213, (char)191, (char)58, (char)94, (char)32, (char)91, (char)122, (char)13, (char)175, (char)97, (char)0, (char)110, (char)133, (char)158, (char)129, (char)86, (char)80, (char)116, (char)27, (char)30, (char)64, (char)219, (char)192, (char)47, (char)224, (char)227, (char)205, (char)46, (char)196, (char)246, (char)125, (char)221}));
            assert(pack.id_GET() == (char)35486);
            assert(pack.ofs_GET() == 1740400033L);
            assert(pack.count_GET() == (char)206);
        });
        DemoDevice.LOG_DATA p120 = LoopBackDemoChannel.new_LOG_DATA();
        PH.setPack(p120);
        p120.count_SET((char)206) ;
        p120.id_SET((char)35486) ;
        p120.data__SET(new char[] {(char)145, (char)69, (char)217, (char)127, (char)136, (char)132, (char)44, (char)186, (char)50, (char)17, (char)212, (char)127, (char)143, (char)98, (char)174, (char)154, (char)251, (char)132, (char)40, (char)35, (char)132, (char)3, (char)210, (char)66, (char)70, (char)86, (char)143, (char)107, (char)52, (char)250, (char)122, (char)236, (char)191, (char)178, (char)210, (char)47, (char)23, (char)162, (char)0, (char)227, (char)206, (char)111, (char)17, (char)137, (char)181, (char)47, (char)94, (char)60, (char)222, (char)197, (char)195, (char)88, (char)153, (char)119, (char)221, (char)139, (char)65, (char)121, (char)213, (char)191, (char)58, (char)94, (char)32, (char)91, (char)122, (char)13, (char)175, (char)97, (char)0, (char)110, (char)133, (char)158, (char)129, (char)86, (char)80, (char)116, (char)27, (char)30, (char)64, (char)219, (char)192, (char)47, (char)224, (char)227, (char)205, (char)46, (char)196, (char)246, (char)125, (char)221}, 0) ;
        p120.ofs_SET(1740400033L) ;
        LoopBackDemoChannel.instance.send(p120);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_ERASE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)18);
            assert(pack.target_component_GET() == (char)5);
        });
        DemoDevice.LOG_ERASE p121 = LoopBackDemoChannel.new_LOG_ERASE();
        PH.setPack(p121);
        p121.target_system_SET((char)18) ;
        p121.target_component_SET((char)5) ;
        LoopBackDemoChannel.instance.send(p121);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOG_REQUEST_END.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)187);
            assert(pack.target_component_GET() == (char)46);
        });
        DemoDevice.LOG_REQUEST_END p122 = LoopBackDemoChannel.new_LOG_REQUEST_END();
        PH.setPack(p122);
        p122.target_component_SET((char)46) ;
        p122.target_system_SET((char)187) ;
        LoopBackDemoChannel.instance.send(p122);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INJECT_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)100);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)16, (char)22, (char)185, (char)255, (char)26, (char)137, (char)44, (char)129, (char)19, (char)24, (char)56, (char)16, (char)214, (char)1, (char)184, (char)123, (char)201, (char)157, (char)4, (char)220, (char)215, (char)217, (char)93, (char)152, (char)28, (char)149, (char)59, (char)216, (char)23, (char)2, (char)45, (char)64, (char)251, (char)196, (char)10, (char)223, (char)255, (char)126, (char)244, (char)78, (char)142, (char)61, (char)99, (char)202, (char)179, (char)127, (char)102, (char)129, (char)235, (char)172, (char)185, (char)233, (char)5, (char)27, (char)26, (char)4, (char)46, (char)187, (char)116, (char)11, (char)149, (char)93, (char)128, (char)59, (char)38, (char)244, (char)236, (char)21, (char)63, (char)71, (char)29, (char)21, (char)113, (char)115, (char)209, (char)151, (char)134, (char)22, (char)251, (char)202, (char)203, (char)86, (char)33, (char)70, (char)219, (char)44, (char)175, (char)213, (char)170, (char)54, (char)241, (char)69, (char)185, (char)119, (char)120, (char)145, (char)137, (char)115, (char)104, (char)32, (char)166, (char)16, (char)75, (char)99, (char)247, (char)220, (char)24, (char)69, (char)57, (char)81}));
            assert(pack.len_GET() == (char)68);
            assert(pack.target_system_GET() == (char)55);
        });
        DemoDevice.GPS_INJECT_DATA p123 = LoopBackDemoChannel.new_GPS_INJECT_DATA();
        PH.setPack(p123);
        p123.len_SET((char)68) ;
        p123.data__SET(new char[] {(char)16, (char)22, (char)185, (char)255, (char)26, (char)137, (char)44, (char)129, (char)19, (char)24, (char)56, (char)16, (char)214, (char)1, (char)184, (char)123, (char)201, (char)157, (char)4, (char)220, (char)215, (char)217, (char)93, (char)152, (char)28, (char)149, (char)59, (char)216, (char)23, (char)2, (char)45, (char)64, (char)251, (char)196, (char)10, (char)223, (char)255, (char)126, (char)244, (char)78, (char)142, (char)61, (char)99, (char)202, (char)179, (char)127, (char)102, (char)129, (char)235, (char)172, (char)185, (char)233, (char)5, (char)27, (char)26, (char)4, (char)46, (char)187, (char)116, (char)11, (char)149, (char)93, (char)128, (char)59, (char)38, (char)244, (char)236, (char)21, (char)63, (char)71, (char)29, (char)21, (char)113, (char)115, (char)209, (char)151, (char)134, (char)22, (char)251, (char)202, (char)203, (char)86, (char)33, (char)70, (char)219, (char)44, (char)175, (char)213, (char)170, (char)54, (char)241, (char)69, (char)185, (char)119, (char)120, (char)145, (char)137, (char)115, (char)104, (char)32, (char)166, (char)16, (char)75, (char)99, (char)247, (char)220, (char)24, (char)69, (char)57, (char)81}, 0) ;
        p123.target_component_SET((char)100) ;
        p123.target_system_SET((char)55) ;
        LoopBackDemoChannel.instance.send(p123);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RAW.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 8687439832554607386L);
            assert(pack.fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX);
            assert(pack.lon_GET() == 1720550117);
            assert(pack.epv_GET() == (char)21703);
            assert(pack.dgps_numch_GET() == (char)193);
            assert(pack.dgps_age_GET() == 3724544759L);
            assert(pack.eph_GET() == (char)56006);
            assert(pack.lat_GET() == 865110985);
            assert(pack.vel_GET() == (char)14123);
            assert(pack.alt_GET() == -1816583403);
            assert(pack.satellites_visible_GET() == (char)90);
            assert(pack.cog_GET() == (char)11647);
        });
        DemoDevice.GPS2_RAW p124 = LoopBackDemoChannel.new_GPS2_RAW();
        PH.setPack(p124);
        p124.fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX) ;
        p124.satellites_visible_SET((char)90) ;
        p124.time_usec_SET(8687439832554607386L) ;
        p124.dgps_age_SET(3724544759L) ;
        p124.cog_SET((char)11647) ;
        p124.eph_SET((char)56006) ;
        p124.dgps_numch_SET((char)193) ;
        p124.lat_SET(865110985) ;
        p124.epv_SET((char)21703) ;
        p124.lon_SET(1720550117) ;
        p124.alt_SET(-1816583403) ;
        p124.vel_SET((char)14123) ;
        LoopBackDemoChannel.instance.send(p124);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_POWER_STATUS.add((src, ph, pack) ->
        {
            assert(pack.Vservo_GET() == (char)27256);
            assert(pack.flags_GET() == MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID);
            assert(pack.Vcc_GET() == (char)47149);
        });
        DemoDevice.POWER_STATUS p125 = LoopBackDemoChannel.new_POWER_STATUS();
        PH.setPack(p125);
        p125.Vservo_SET((char)27256) ;
        p125.flags_SET(MAV_POWER_STATUS.MAV_POWER_STATUS_BRICK_VALID) ;
        p125.Vcc_SET((char)47149) ;
        LoopBackDemoChannel.instance.send(p125);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SERIAL_CONTROL.add((src, ph, pack) ->
        {
            assert(pack.baudrate_GET() == 758397549L);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)87, (char)26, (char)183, (char)61, (char)191, (char)26, (char)210, (char)255, (char)243, (char)70, (char)69, (char)173, (char)37, (char)190, (char)154, (char)81, (char)133, (char)21, (char)42, (char)51, (char)3, (char)250, (char)225, (char)75, (char)158, (char)30, (char)179, (char)173, (char)17, (char)117, (char)21, (char)98, (char)114, (char)97, (char)229, (char)142, (char)34, (char)155, (char)122, (char)90, (char)132, (char)57, (char)197, (char)73, (char)93, (char)109, (char)35, (char)103, (char)154, (char)240, (char)79, (char)5, (char)20, (char)42, (char)54, (char)32, (char)45, (char)138, (char)19, (char)136, (char)229, (char)227, (char)182, (char)79, (char)76, (char)183, (char)31, (char)22, (char)148, (char)230}));
            assert(pack.device_GET() == SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL);
            assert(pack.count_GET() == (char)146);
            assert(pack.timeout_GET() == (char)15633);
            assert(pack.flags_GET() == SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE);
        });
        DemoDevice.SERIAL_CONTROL p126 = LoopBackDemoChannel.new_SERIAL_CONTROL();
        PH.setPack(p126);
        p126.baudrate_SET(758397549L) ;
        p126.data__SET(new char[] {(char)87, (char)26, (char)183, (char)61, (char)191, (char)26, (char)210, (char)255, (char)243, (char)70, (char)69, (char)173, (char)37, (char)190, (char)154, (char)81, (char)133, (char)21, (char)42, (char)51, (char)3, (char)250, (char)225, (char)75, (char)158, (char)30, (char)179, (char)173, (char)17, (char)117, (char)21, (char)98, (char)114, (char)97, (char)229, (char)142, (char)34, (char)155, (char)122, (char)90, (char)132, (char)57, (char)197, (char)73, (char)93, (char)109, (char)35, (char)103, (char)154, (char)240, (char)79, (char)5, (char)20, (char)42, (char)54, (char)32, (char)45, (char)138, (char)19, (char)136, (char)229, (char)227, (char)182, (char)79, (char)76, (char)183, (char)31, (char)22, (char)148, (char)230}, 0) ;
        p126.device_SET(SERIAL_CONTROL_DEV.SERIAL_CONTROL_DEV_SHELL) ;
        p126.flags_SET(SERIAL_CONTROL_FLAG.SERIAL_CONTROL_FLAG_EXCLUSIVE) ;
        p126.timeout_SET((char)15633) ;
        p126.count_SET((char)146) ;
        LoopBackDemoChannel.instance.send(p126);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTK.add((src, ph, pack) ->
        {
            assert(pack.nsats_GET() == (char)185);
            assert(pack.iar_num_hypotheses_GET() == 655607200);
            assert(pack.baseline_coords_type_GET() == (char)125);
            assert(pack.baseline_a_mm_GET() == 844086358);
            assert(pack.accuracy_GET() == 3222138647L);
            assert(pack.rtk_receiver_id_GET() == (char)125);
            assert(pack.time_last_baseline_ms_GET() == 612158897L);
            assert(pack.tow_GET() == 1600420492L);
            assert(pack.baseline_c_mm_GET() == 487943321);
            assert(pack.wn_GET() == (char)35957);
            assert(pack.baseline_b_mm_GET() == 2080264277);
            assert(pack.rtk_health_GET() == (char)186);
            assert(pack.rtk_rate_GET() == (char)204);
        });
        DemoDevice.GPS_RTK p127 = LoopBackDemoChannel.new_GPS_RTK();
        PH.setPack(p127);
        p127.baseline_coords_type_SET((char)125) ;
        p127.baseline_a_mm_SET(844086358) ;
        p127.time_last_baseline_ms_SET(612158897L) ;
        p127.baseline_c_mm_SET(487943321) ;
        p127.nsats_SET((char)185) ;
        p127.rtk_rate_SET((char)204) ;
        p127.iar_num_hypotheses_SET(655607200) ;
        p127.accuracy_SET(3222138647L) ;
        p127.rtk_receiver_id_SET((char)125) ;
        p127.wn_SET((char)35957) ;
        p127.rtk_health_SET((char)186) ;
        p127.baseline_b_mm_SET(2080264277) ;
        p127.tow_SET(1600420492L) ;
        LoopBackDemoChannel.instance.send(p127);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS2_RTK.add((src, ph, pack) ->
        {
            assert(pack.accuracy_GET() == 1937972881L);
            assert(pack.time_last_baseline_ms_GET() == 27600837L);
            assert(pack.rtk_receiver_id_GET() == (char)16);
            assert(pack.iar_num_hypotheses_GET() == 1325419022);
            assert(pack.baseline_b_mm_GET() == -508015294);
            assert(pack.baseline_c_mm_GET() == -109831213);
            assert(pack.tow_GET() == 2660227256L);
            assert(pack.nsats_GET() == (char)29);
            assert(pack.rtk_rate_GET() == (char)238);
            assert(pack.baseline_a_mm_GET() == 1222932106);
            assert(pack.baseline_coords_type_GET() == (char)117);
            assert(pack.rtk_health_GET() == (char)112);
            assert(pack.wn_GET() == (char)53355);
        });
        DemoDevice.GPS2_RTK p128 = LoopBackDemoChannel.new_GPS2_RTK();
        PH.setPack(p128);
        p128.baseline_c_mm_SET(-109831213) ;
        p128.baseline_b_mm_SET(-508015294) ;
        p128.rtk_rate_SET((char)238) ;
        p128.accuracy_SET(1937972881L) ;
        p128.baseline_a_mm_SET(1222932106) ;
        p128.nsats_SET((char)29) ;
        p128.rtk_receiver_id_SET((char)16) ;
        p128.rtk_health_SET((char)112) ;
        p128.wn_SET((char)53355) ;
        p128.tow_SET(2660227256L) ;
        p128.baseline_coords_type_SET((char)117) ;
        p128.iar_num_hypotheses_SET(1325419022) ;
        p128.time_last_baseline_ms_SET(27600837L) ;
        LoopBackDemoChannel.instance.send(p128);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_IMU3.add((src, ph, pack) ->
        {
            assert(pack.yacc_GET() == (short) -10039);
            assert(pack.xacc_GET() == (short)18838);
            assert(pack.zgyro_GET() == (short)1170);
            assert(pack.ymag_GET() == (short)13591);
            assert(pack.xmag_GET() == (short)32644);
            assert(pack.zacc_GET() == (short)28410);
            assert(pack.zmag_GET() == (short)7250);
            assert(pack.time_boot_ms_GET() == 2640943174L);
            assert(pack.xgyro_GET() == (short)17333);
            assert(pack.ygyro_GET() == (short)1924);
        });
        DemoDevice.SCALED_IMU3 p129 = LoopBackDemoChannel.new_SCALED_IMU3();
        PH.setPack(p129);
        p129.ygyro_SET((short)1924) ;
        p129.xmag_SET((short)32644) ;
        p129.ymag_SET((short)13591) ;
        p129.zacc_SET((short)28410) ;
        p129.xacc_SET((short)18838) ;
        p129.xgyro_SET((short)17333) ;
        p129.time_boot_ms_SET(2640943174L) ;
        p129.zgyro_SET((short)1170) ;
        p129.zmag_SET((short)7250) ;
        p129.yacc_SET((short) -10039) ;
        LoopBackDemoChannel.instance.send(p129);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_TRANSMISSION_HANDSHAKE.add((src, ph, pack) ->
        {
            assert(pack.width_GET() == (char)34067);
            assert(pack.size_GET() == 1673961005L);
            assert(pack.packets_GET() == (char)64966);
            assert(pack.height_GET() == (char)29017);
            assert(pack.jpg_quality_GET() == (char)0);
            assert(pack.type_GET() == (char)6);
            assert(pack.payload_GET() == (char)250);
        });
        DemoDevice.DATA_TRANSMISSION_HANDSHAKE p130 = LoopBackDemoChannel.new_DATA_TRANSMISSION_HANDSHAKE();
        PH.setPack(p130);
        p130.height_SET((char)29017) ;
        p130.width_SET((char)34067) ;
        p130.payload_SET((char)250) ;
        p130.type_SET((char)6) ;
        p130.packets_SET((char)64966) ;
        p130.size_SET(1673961005L) ;
        p130.jpg_quality_SET((char)0) ;
        LoopBackDemoChannel.instance.send(p130);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ENCAPSULATED_DATA.add((src, ph, pack) ->
        {
            assert(pack.seqnr_GET() == (char)3936);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)4, (char)44, (char)209, (char)160, (char)69, (char)45, (char)202, (char)158, (char)125, (char)75, (char)202, (char)187, (char)176, (char)23, (char)155, (char)220, (char)246, (char)161, (char)18, (char)59, (char)11, (char)73, (char)79, (char)193, (char)113, (char)101, (char)80, (char)12, (char)14, (char)109, (char)152, (char)187, (char)139, (char)43, (char)116, (char)154, (char)250, (char)253, (char)233, (char)141, (char)225, (char)148, (char)80, (char)93, (char)168, (char)46, (char)8, (char)224, (char)99, (char)77, (char)154, (char)144, (char)243, (char)80, (char)58, (char)237, (char)205, (char)139, (char)52, (char)235, (char)178, (char)88, (char)128, (char)156, (char)216, (char)62, (char)126, (char)151, (char)213, (char)109, (char)140, (char)40, (char)229, (char)226, (char)227, (char)0, (char)134, (char)33, (char)61, (char)220, (char)138, (char)14, (char)123, (char)95, (char)103, (char)100, (char)162, (char)152, (char)66, (char)210, (char)13, (char)111, (char)17, (char)173, (char)248, (char)203, (char)111, (char)225, (char)170, (char)6, (char)16, (char)203, (char)226, (char)156, (char)6, (char)80, (char)253, (char)47, (char)125, (char)44, (char)245, (char)12, (char)7, (char)36, (char)240, (char)247, (char)51, (char)37, (char)159, (char)236, (char)50, (char)225, (char)217, (char)141, (char)193, (char)199, (char)43, (char)33, (char)246, (char)203, (char)45, (char)248, (char)160, (char)130, (char)150, (char)60, (char)76, (char)111, (char)212, (char)34, (char)113, (char)233, (char)205, (char)223, (char)133, (char)66, (char)80, (char)7, (char)126, (char)222, (char)238, (char)233, (char)249, (char)162, (char)143, (char)147, (char)211, (char)101, (char)90, (char)32, (char)184, (char)207, (char)177, (char)95, (char)85, (char)212, (char)183, (char)238, (char)213, (char)147, (char)9, (char)66, (char)172, (char)84, (char)59, (char)93, (char)114, (char)218, (char)2, (char)135, (char)38, (char)73, (char)66, (char)52, (char)153, (char)37, (char)208, (char)25, (char)127, (char)57, (char)218, (char)184, (char)153, (char)215, (char)170, (char)107, (char)57, (char)54, (char)202, (char)150, (char)70, (char)37, (char)232, (char)111, (char)153, (char)44, (char)87, (char)147, (char)17, (char)164, (char)184, (char)31, (char)211, (char)33, (char)88, (char)77, (char)96, (char)202, (char)233, (char)132, (char)34, (char)51, (char)232, (char)181, (char)189, (char)125, (char)169, (char)153, (char)142, (char)101, (char)214, (char)4, (char)44, (char)66, (char)244, (char)4, (char)144, (char)190, (char)190, (char)69, (char)75, (char)123, (char)250, (char)109, (char)211, (char)81, (char)255, (char)131, (char)204, (char)78, (char)88, (char)160, (char)192}));
        });
        DemoDevice.ENCAPSULATED_DATA p131 = LoopBackDemoChannel.new_ENCAPSULATED_DATA();
        PH.setPack(p131);
        p131.data__SET(new char[] {(char)4, (char)44, (char)209, (char)160, (char)69, (char)45, (char)202, (char)158, (char)125, (char)75, (char)202, (char)187, (char)176, (char)23, (char)155, (char)220, (char)246, (char)161, (char)18, (char)59, (char)11, (char)73, (char)79, (char)193, (char)113, (char)101, (char)80, (char)12, (char)14, (char)109, (char)152, (char)187, (char)139, (char)43, (char)116, (char)154, (char)250, (char)253, (char)233, (char)141, (char)225, (char)148, (char)80, (char)93, (char)168, (char)46, (char)8, (char)224, (char)99, (char)77, (char)154, (char)144, (char)243, (char)80, (char)58, (char)237, (char)205, (char)139, (char)52, (char)235, (char)178, (char)88, (char)128, (char)156, (char)216, (char)62, (char)126, (char)151, (char)213, (char)109, (char)140, (char)40, (char)229, (char)226, (char)227, (char)0, (char)134, (char)33, (char)61, (char)220, (char)138, (char)14, (char)123, (char)95, (char)103, (char)100, (char)162, (char)152, (char)66, (char)210, (char)13, (char)111, (char)17, (char)173, (char)248, (char)203, (char)111, (char)225, (char)170, (char)6, (char)16, (char)203, (char)226, (char)156, (char)6, (char)80, (char)253, (char)47, (char)125, (char)44, (char)245, (char)12, (char)7, (char)36, (char)240, (char)247, (char)51, (char)37, (char)159, (char)236, (char)50, (char)225, (char)217, (char)141, (char)193, (char)199, (char)43, (char)33, (char)246, (char)203, (char)45, (char)248, (char)160, (char)130, (char)150, (char)60, (char)76, (char)111, (char)212, (char)34, (char)113, (char)233, (char)205, (char)223, (char)133, (char)66, (char)80, (char)7, (char)126, (char)222, (char)238, (char)233, (char)249, (char)162, (char)143, (char)147, (char)211, (char)101, (char)90, (char)32, (char)184, (char)207, (char)177, (char)95, (char)85, (char)212, (char)183, (char)238, (char)213, (char)147, (char)9, (char)66, (char)172, (char)84, (char)59, (char)93, (char)114, (char)218, (char)2, (char)135, (char)38, (char)73, (char)66, (char)52, (char)153, (char)37, (char)208, (char)25, (char)127, (char)57, (char)218, (char)184, (char)153, (char)215, (char)170, (char)107, (char)57, (char)54, (char)202, (char)150, (char)70, (char)37, (char)232, (char)111, (char)153, (char)44, (char)87, (char)147, (char)17, (char)164, (char)184, (char)31, (char)211, (char)33, (char)88, (char)77, (char)96, (char)202, (char)233, (char)132, (char)34, (char)51, (char)232, (char)181, (char)189, (char)125, (char)169, (char)153, (char)142, (char)101, (char)214, (char)4, (char)44, (char)66, (char)244, (char)4, (char)144, (char)190, (char)190, (char)69, (char)75, (char)123, (char)250, (char)109, (char)211, (char)81, (char)255, (char)131, (char)204, (char)78, (char)88, (char)160, (char)192}, 0) ;
        p131.seqnr_SET((char)3936) ;
        LoopBackDemoChannel.instance.send(p131);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DISTANCE_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.max_distance_GET() == (char)290);
            assert(pack.id_GET() == (char)37);
            assert(pack.type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR);
            assert(pack.orientation_GET() == MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_45);
            assert(pack.time_boot_ms_GET() == 3008667792L);
            assert(pack.min_distance_GET() == (char)54891);
            assert(pack.covariance_GET() == (char)241);
            assert(pack.current_distance_GET() == (char)24828);
        });
        DemoDevice.DISTANCE_SENSOR p132 = LoopBackDemoChannel.new_DISTANCE_SENSOR();
        PH.setPack(p132);
        p132.max_distance_SET((char)290) ;
        p132.time_boot_ms_SET(3008667792L) ;
        p132.current_distance_SET((char)24828) ;
        p132.id_SET((char)37) ;
        p132.type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR) ;
        p132.min_distance_SET((char)54891) ;
        p132.covariance_SET((char)241) ;
        p132.orientation_SET(MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_ROLL_180_YAW_45) ;
        LoopBackDemoChannel.instance.send(p132);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.grid_spacing_GET() == (char)39868);
            assert(pack.lon_GET() == -1160726739);
            assert(pack.lat_GET() == -2048966120);
            assert(pack.mask_GET() == 3318453228253752847L);
        });
        DemoDevice.TERRAIN_REQUEST p133 = LoopBackDemoChannel.new_TERRAIN_REQUEST();
        PH.setPack(p133);
        p133.mask_SET(3318453228253752847L) ;
        p133.grid_spacing_SET((char)39868) ;
        p133.lon_SET(-1160726739) ;
        p133.lat_SET(-2048966120) ;
        LoopBackDemoChannel.instance.send(p133);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_DATA.add((src, ph, pack) ->
        {
            assert(pack.gridbit_GET() == (char)84);
            assert(pack.lat_GET() == 576316597);
            assert(Arrays.equals(pack.data__GET(),  new short[] {(short) -14026, (short)20947, (short) -12817, (short) -18732, (short) -467, (short)26918, (short) -6978, (short)32458, (short)20394, (short)9492, (short)30269, (short)2103, (short)28017, (short) -3487, (short)16676, (short)31196}));
            assert(pack.lon_GET() == 1388910559);
            assert(pack.grid_spacing_GET() == (char)22150);
        });
        DemoDevice.TERRAIN_DATA p134 = LoopBackDemoChannel.new_TERRAIN_DATA();
        PH.setPack(p134);
        p134.gridbit_SET((char)84) ;
        p134.grid_spacing_SET((char)22150) ;
        p134.lon_SET(1388910559) ;
        p134.lat_SET(576316597) ;
        p134.data__SET(new short[] {(short) -14026, (short)20947, (short) -12817, (short) -18732, (short) -467, (short)26918, (short) -6978, (short)32458, (short)20394, (short)9492, (short)30269, (short)2103, (short)28017, (short) -3487, (short)16676, (short)31196}, 0) ;
        LoopBackDemoChannel.instance.send(p134);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_CHECK.add((src, ph, pack) ->
        {
            assert(pack.lon_GET() == -1676662649);
            assert(pack.lat_GET() == -1787905659);
        });
        DemoDevice.TERRAIN_CHECK p135 = LoopBackDemoChannel.new_TERRAIN_CHECK();
        PH.setPack(p135);
        p135.lat_SET(-1787905659) ;
        p135.lon_SET(-1676662649) ;
        LoopBackDemoChannel.instance.send(p135);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_TERRAIN_REPORT.add((src, ph, pack) ->
        {
            assert(pack.current_height_GET() == -1.7186085E38F);
            assert(pack.lat_GET() == -1467496546);
            assert(pack.pending_GET() == (char)15974);
            assert(pack.spacing_GET() == (char)9606);
            assert(pack.lon_GET() == 1215379377);
            assert(pack.loaded_GET() == (char)56533);
            assert(pack.terrain_height_GET() == -1.986206E38F);
        });
        DemoDevice.TERRAIN_REPORT p136 = LoopBackDemoChannel.new_TERRAIN_REPORT();
        PH.setPack(p136);
        p136.current_height_SET(-1.7186085E38F) ;
        p136.lon_SET(1215379377) ;
        p136.pending_SET((char)15974) ;
        p136.spacing_SET((char)9606) ;
        p136.lat_SET(-1467496546) ;
        p136.terrain_height_SET(-1.986206E38F) ;
        p136.loaded_SET((char)56533) ;
        LoopBackDemoChannel.instance.send(p136);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE2.add((src, ph, pack) ->
        {
            assert(pack.press_abs_GET() == 1.8171303E38F);
            assert(pack.press_diff_GET() == 2.1266066E38F);
            assert(pack.time_boot_ms_GET() == 2823114849L);
            assert(pack.temperature_GET() == (short) -24846);
        });
        DemoDevice.SCALED_PRESSURE2 p137 = LoopBackDemoChannel.new_SCALED_PRESSURE2();
        PH.setPack(p137);
        p137.press_diff_SET(2.1266066E38F) ;
        p137.time_boot_ms_SET(2823114849L) ;
        p137.temperature_SET((short) -24846) ;
        p137.press_abs_SET(1.8171303E38F) ;
        LoopBackDemoChannel.instance.send(p137);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ATT_POS_MOCAP.add((src, ph, pack) ->
        {
            assert(pack.y_GET() == 3.1058363E38F);
            assert(pack.z_GET() == 3.3532032E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-3.1138573E38F, 1.4975644E38F, 2.1245605E38F, -4.247455E37F}));
            assert(pack.time_usec_GET() == 8604224409295909873L);
            assert(pack.x_GET() == 1.6229759E38F);
        });
        DemoDevice.ATT_POS_MOCAP p138 = LoopBackDemoChannel.new_ATT_POS_MOCAP();
        PH.setPack(p138);
        p138.z_SET(3.3532032E38F) ;
        p138.y_SET(3.1058363E38F) ;
        p138.x_SET(1.6229759E38F) ;
        p138.q_SET(new float[] {-3.1138573E38F, 1.4975644E38F, 2.1245605E38F, -4.247455E37F}, 0) ;
        p138.time_usec_SET(8604224409295909873L) ;
        LoopBackDemoChannel.instance.send(p138);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)172);
            assert(pack.target_system_GET() == (char)156);
            assert(pack.group_mlx_GET() == (char)142);
            assert(pack.time_usec_GET() == 4807004888738069618L);
            assert(Arrays.equals(pack.controls_GET(),  new float[] {3.0165985E38F, -4.3606486E37F, 1.8114303E38F, 2.6178666E38F, 2.7014127E38F, 3.0046712E38F, -3.3145191E38F, 2.2741009E38F}));
        });
        DemoDevice.SET_ACTUATOR_CONTROL_TARGET p139 = LoopBackDemoChannel.new_SET_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p139);
        p139.group_mlx_SET((char)142) ;
        p139.time_usec_SET(4807004888738069618L) ;
        p139.target_system_SET((char)156) ;
        p139.target_component_SET((char)172) ;
        p139.controls_SET(new float[] {3.0165985E38F, -4.3606486E37F, 1.8114303E38F, 2.6178666E38F, 2.7014127E38F, 3.0046712E38F, -3.3145191E38F, 2.2741009E38F}, 0) ;
        LoopBackDemoChannel.instance.send(p139);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ACTUATOR_CONTROL_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.controls_GET(),  new float[] {-2.5972589E38F, -3.1356333E38F, -1.4555501E37F, 1.0737961E37F, 1.2681807E38F, 6.002389E37F, -1.0945764E38F, 1.8155077E38F}));
            assert(pack.group_mlx_GET() == (char)113);
            assert(pack.time_usec_GET() == 6824708813212510305L);
        });
        DemoDevice.ACTUATOR_CONTROL_TARGET p140 = LoopBackDemoChannel.new_ACTUATOR_CONTROL_TARGET();
        PH.setPack(p140);
        p140.group_mlx_SET((char)113) ;
        p140.controls_SET(new float[] {-2.5972589E38F, -3.1356333E38F, -1.4555501E37F, 1.0737961E37F, 1.2681807E38F, 6.002389E37F, -1.0945764E38F, 1.8155077E38F}, 0) ;
        p140.time_usec_SET(6824708813212510305L) ;
        LoopBackDemoChannel.instance.send(p140);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ALTITUDE.add((src, ph, pack) ->
        {
            assert(pack.altitude_local_GET() == -3.4002478E38F);
            assert(pack.time_usec_GET() == 1161000585713273123L);
            assert(pack.bottom_clearance_GET() == -2.9924594E38F);
            assert(pack.altitude_relative_GET() == 3.0881855E38F);
            assert(pack.altitude_amsl_GET() == -2.8554475E38F);
            assert(pack.altitude_monotonic_GET() == -2.6762278E38F);
            assert(pack.altitude_terrain_GET() == -1.1460838E38F);
        });
        DemoDevice.ALTITUDE p141 = LoopBackDemoChannel.new_ALTITUDE();
        PH.setPack(p141);
        p141.bottom_clearance_SET(-2.9924594E38F) ;
        p141.altitude_amsl_SET(-2.8554475E38F) ;
        p141.altitude_terrain_SET(-1.1460838E38F) ;
        p141.altitude_monotonic_SET(-2.6762278E38F) ;
        p141.altitude_local_SET(-3.4002478E38F) ;
        p141.altitude_relative_SET(3.0881855E38F) ;
        p141.time_usec_SET(1161000585713273123L) ;
        LoopBackDemoChannel.instance.send(p141);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_RESOURCE_REQUEST.add((src, ph, pack) ->
        {
            assert(pack.transfer_type_GET() == (char)24);
            assert(Arrays.equals(pack.uri_GET(),  new char[] {(char)75, (char)21, (char)5, (char)94, (char)72, (char)151, (char)38, (char)159, (char)147, (char)125, (char)98, (char)76, (char)239, (char)254, (char)227, (char)4, (char)119, (char)239, (char)159, (char)227, (char)26, (char)134, (char)103, (char)139, (char)127, (char)84, (char)34, (char)176, (char)97, (char)78, (char)252, (char)65, (char)34, (char)85, (char)147, (char)255, (char)191, (char)255, (char)68, (char)127, (char)44, (char)233, (char)203, (char)128, (char)62, (char)92, (char)62, (char)26, (char)171, (char)250, (char)82, (char)96, (char)18, (char)220, (char)52, (char)214, (char)248, (char)55, (char)163, (char)216, (char)22, (char)14, (char)117, (char)248, (char)119, (char)194, (char)222, (char)51, (char)106, (char)189, (char)248, (char)118, (char)54, (char)152, (char)110, (char)29, (char)208, (char)115, (char)38, (char)65, (char)197, (char)159, (char)151, (char)241, (char)47, (char)16, (char)26, (char)141, (char)64, (char)74, (char)146, (char)143, (char)114, (char)73, (char)72, (char)240, (char)192, (char)199, (char)171, (char)94, (char)135, (char)221, (char)184, (char)192, (char)173, (char)164, (char)84, (char)124, (char)119, (char)205, (char)93, (char)48, (char)149, (char)26, (char)26, (char)202, (char)190, (char)109, (char)122, (char)81}));
            assert(pack.uri_type_GET() == (char)139);
            assert(pack.request_id_GET() == (char)53);
            assert(Arrays.equals(pack.storage_GET(),  new char[] {(char)81, (char)95, (char)77, (char)100, (char)178, (char)55, (char)88, (char)96, (char)119, (char)55, (char)99, (char)231, (char)163, (char)1, (char)181, (char)112, (char)180, (char)139, (char)27, (char)36, (char)173, (char)99, (char)82, (char)39, (char)114, (char)26, (char)156, (char)231, (char)144, (char)227, (char)90, (char)183, (char)58, (char)222, (char)214, (char)11, (char)62, (char)101, (char)134, (char)152, (char)105, (char)29, (char)82, (char)37, (char)104, (char)100, (char)50, (char)130, (char)7, (char)49, (char)253, (char)174, (char)99, (char)186, (char)79, (char)177, (char)60, (char)73, (char)17, (char)229, (char)203, (char)12, (char)104, (char)68, (char)180, (char)236, (char)204, (char)83, (char)205, (char)185, (char)18, (char)122, (char)77, (char)14, (char)181, (char)235, (char)138, (char)252, (char)84, (char)200, (char)81, (char)55, (char)114, (char)193, (char)138, (char)19, (char)189, (char)98, (char)194, (char)18, (char)57, (char)208, (char)1, (char)202, (char)228, (char)106, (char)42, (char)150, (char)134, (char)196, (char)199, (char)153, (char)119, (char)200, (char)157, (char)175, (char)22, (char)53, (char)110, (char)185, (char)42, (char)47, (char)90, (char)22, (char)177, (char)153, (char)124, (char)234, (char)56, (char)78}));
        });
        DemoDevice.RESOURCE_REQUEST p142 = LoopBackDemoChannel.new_RESOURCE_REQUEST();
        PH.setPack(p142);
        p142.transfer_type_SET((char)24) ;
        p142.request_id_SET((char)53) ;
        p142.uri_type_SET((char)139) ;
        p142.storage_SET(new char[] {(char)81, (char)95, (char)77, (char)100, (char)178, (char)55, (char)88, (char)96, (char)119, (char)55, (char)99, (char)231, (char)163, (char)1, (char)181, (char)112, (char)180, (char)139, (char)27, (char)36, (char)173, (char)99, (char)82, (char)39, (char)114, (char)26, (char)156, (char)231, (char)144, (char)227, (char)90, (char)183, (char)58, (char)222, (char)214, (char)11, (char)62, (char)101, (char)134, (char)152, (char)105, (char)29, (char)82, (char)37, (char)104, (char)100, (char)50, (char)130, (char)7, (char)49, (char)253, (char)174, (char)99, (char)186, (char)79, (char)177, (char)60, (char)73, (char)17, (char)229, (char)203, (char)12, (char)104, (char)68, (char)180, (char)236, (char)204, (char)83, (char)205, (char)185, (char)18, (char)122, (char)77, (char)14, (char)181, (char)235, (char)138, (char)252, (char)84, (char)200, (char)81, (char)55, (char)114, (char)193, (char)138, (char)19, (char)189, (char)98, (char)194, (char)18, (char)57, (char)208, (char)1, (char)202, (char)228, (char)106, (char)42, (char)150, (char)134, (char)196, (char)199, (char)153, (char)119, (char)200, (char)157, (char)175, (char)22, (char)53, (char)110, (char)185, (char)42, (char)47, (char)90, (char)22, (char)177, (char)153, (char)124, (char)234, (char)56, (char)78}, 0) ;
        p142.uri_SET(new char[] {(char)75, (char)21, (char)5, (char)94, (char)72, (char)151, (char)38, (char)159, (char)147, (char)125, (char)98, (char)76, (char)239, (char)254, (char)227, (char)4, (char)119, (char)239, (char)159, (char)227, (char)26, (char)134, (char)103, (char)139, (char)127, (char)84, (char)34, (char)176, (char)97, (char)78, (char)252, (char)65, (char)34, (char)85, (char)147, (char)255, (char)191, (char)255, (char)68, (char)127, (char)44, (char)233, (char)203, (char)128, (char)62, (char)92, (char)62, (char)26, (char)171, (char)250, (char)82, (char)96, (char)18, (char)220, (char)52, (char)214, (char)248, (char)55, (char)163, (char)216, (char)22, (char)14, (char)117, (char)248, (char)119, (char)194, (char)222, (char)51, (char)106, (char)189, (char)248, (char)118, (char)54, (char)152, (char)110, (char)29, (char)208, (char)115, (char)38, (char)65, (char)197, (char)159, (char)151, (char)241, (char)47, (char)16, (char)26, (char)141, (char)64, (char)74, (char)146, (char)143, (char)114, (char)73, (char)72, (char)240, (char)192, (char)199, (char)171, (char)94, (char)135, (char)221, (char)184, (char)192, (char)173, (char)164, (char)84, (char)124, (char)119, (char)205, (char)93, (char)48, (char)149, (char)26, (char)26, (char)202, (char)190, (char)109, (char)122, (char)81}, 0) ;
        LoopBackDemoChannel.instance.send(p142);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SCALED_PRESSURE3.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 3900296808L);
            assert(pack.temperature_GET() == (short)30871);
            assert(pack.press_diff_GET() == -1.2676736E38F);
            assert(pack.press_abs_GET() == 2.5387669E38F);
        });
        DemoDevice.SCALED_PRESSURE3 p143 = LoopBackDemoChannel.new_SCALED_PRESSURE3();
        PH.setPack(p143);
        p143.temperature_SET((short)30871) ;
        p143.time_boot_ms_SET(3900296808L) ;
        p143.press_diff_SET(-1.2676736E38F) ;
        p143.press_abs_SET(2.5387669E38F) ;
        LoopBackDemoChannel.instance.send(p143);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FOLLOW_TARGET.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.vel_GET(),  new float[] {1.6831482E38F, -6.897659E37F, 3.3867907E36F}));
            assert(Arrays.equals(pack.rates_GET(),  new float[] {7.8758635E37F, 2.1770753E38F, 1.4908334E38F}));
            assert(pack.alt_GET() == -3.9876992E37F);
            assert(pack.timestamp_GET() == 6272616057409579298L);
            assert(pack.est_capabilities_GET() == (char)252);
            assert(pack.custom_state_GET() == 7781049409930469908L);
            assert(Arrays.equals(pack.position_cov_GET(),  new float[] {-1.3533844E38F, 5.4282873E37F, 2.4027628E37F}));
            assert(pack.lat_GET() == 267184791);
            assert(Arrays.equals(pack.attitude_q_GET(),  new float[] {2.8880975E38F, 1.01204806E37F, 2.753487E38F, -1.9714243E38F}));
            assert(Arrays.equals(pack.acc_GET(),  new float[] {2.813903E37F, 2.9734536E38F, 2.3308472E38F}));
            assert(pack.lon_GET() == 863057646);
        });
        DemoDevice.FOLLOW_TARGET p144 = LoopBackDemoChannel.new_FOLLOW_TARGET();
        PH.setPack(p144);
        p144.custom_state_SET(7781049409930469908L) ;
        p144.timestamp_SET(6272616057409579298L) ;
        p144.rates_SET(new float[] {7.8758635E37F, 2.1770753E38F, 1.4908334E38F}, 0) ;
        p144.lat_SET(267184791) ;
        p144.est_capabilities_SET((char)252) ;
        p144.vel_SET(new float[] {1.6831482E38F, -6.897659E37F, 3.3867907E36F}, 0) ;
        p144.attitude_q_SET(new float[] {2.8880975E38F, 1.01204806E37F, 2.753487E38F, -1.9714243E38F}, 0) ;
        p144.lon_SET(863057646) ;
        p144.alt_SET(-3.9876992E37F) ;
        p144.position_cov_SET(new float[] {-1.3533844E38F, 5.4282873E37F, 2.4027628E37F}, 0) ;
        p144.acc_SET(new float[] {2.813903E37F, 2.9734536E38F, 2.3308472E38F}, 0) ;
        LoopBackDemoChannel.instance.send(p144);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CONTROL_SYSTEM_STATE.add((src, ph, pack) ->
        {
            assert(pack.y_pos_GET() == 2.8795639E38F);
            assert(pack.z_pos_GET() == 1.9112153E38F);
            assert(pack.roll_rate_GET() == -7.7628347E37F);
            assert(pack.x_vel_GET() == 1.6554402E38F);
            assert(Arrays.equals(pack.pos_variance_GET(),  new float[] {7.5311715E37F, -1.984417E38F, 1.655589E38F}));
            assert(pack.x_pos_GET() == -3.0804646E38F);
            assert(pack.pitch_rate_GET() == -8.558584E36F);
            assert(Arrays.equals(pack.vel_variance_GET(),  new float[] {-2.229419E37F, 3.163455E38F, 9.613691E37F}));
            assert(pack.yaw_rate_GET() == -1.1164879E38F);
            assert(pack.z_acc_GET() == 4.5824225E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {1.7007903E38F, -2.8964557E38F, 3.2281875E38F, 2.0730752E38F}));
            assert(pack.x_acc_GET() == 8.156463E37F);
            assert(pack.y_vel_GET() == 1.6022517E38F);
            assert(pack.time_usec_GET() == 4678619893339946867L);
            assert(pack.airspeed_GET() == -4.7990514E37F);
            assert(pack.y_acc_GET() == 1.5019405E38F);
            assert(pack.z_vel_GET() == -4.455467E37F);
        });
        DemoDevice.CONTROL_SYSTEM_STATE p146 = LoopBackDemoChannel.new_CONTROL_SYSTEM_STATE();
        PH.setPack(p146);
        p146.y_vel_SET(1.6022517E38F) ;
        p146.z_vel_SET(-4.455467E37F) ;
        p146.yaw_rate_SET(-1.1164879E38F) ;
        p146.roll_rate_SET(-7.7628347E37F) ;
        p146.z_acc_SET(4.5824225E37F) ;
        p146.x_acc_SET(8.156463E37F) ;
        p146.x_pos_SET(-3.0804646E38F) ;
        p146.vel_variance_SET(new float[] {-2.229419E37F, 3.163455E38F, 9.613691E37F}, 0) ;
        p146.q_SET(new float[] {1.7007903E38F, -2.8964557E38F, 3.2281875E38F, 2.0730752E38F}, 0) ;
        p146.pitch_rate_SET(-8.558584E36F) ;
        p146.pos_variance_SET(new float[] {7.5311715E37F, -1.984417E38F, 1.655589E38F}, 0) ;
        p146.z_pos_SET(1.9112153E38F) ;
        p146.y_pos_SET(2.8795639E38F) ;
        p146.airspeed_SET(-4.7990514E37F) ;
        p146.x_vel_SET(1.6554402E38F) ;
        p146.y_acc_SET(1.5019405E38F) ;
        p146.time_usec_SET(4678619893339946867L) ;
        LoopBackDemoChannel.instance.send(p146);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BATTERY_STATUS.add((src, ph, pack) ->
        {
            assert(pack.type_GET() == MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN);
            assert(pack.id_GET() == (char)101);
            assert(Arrays.equals(pack.voltages_GET(),  new char[] {(char)54385, (char)50002, (char)22678, (char)21109, (char)13234, (char)65065, (char)19487, (char)41205, (char)24857, (char)18482}));
            assert(pack.battery_remaining_GET() == (byte) - 7);
            assert(pack.current_consumed_GET() == -2100660288);
            assert(pack.temperature_GET() == (short) -13602);
            assert(pack.battery_function_GET() == MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION);
            assert(pack.energy_consumed_GET() == 1819392505);
            assert(pack.current_battery_GET() == (short)30808);
        });
        DemoDevice.BATTERY_STATUS p147 = LoopBackDemoChannel.new_BATTERY_STATUS();
        PH.setPack(p147);
        p147.type_SET(MAV_BATTERY_TYPE.MAV_BATTERY_TYPE_UNKNOWN) ;
        p147.current_battery_SET((short)30808) ;
        p147.temperature_SET((short) -13602) ;
        p147.battery_function_SET(MAV_BATTERY_FUNCTION.MAV_BATTERY_FUNCTION_PROPULSION) ;
        p147.energy_consumed_SET(1819392505) ;
        p147.voltages_SET(new char[] {(char)54385, (char)50002, (char)22678, (char)21109, (char)13234, (char)65065, (char)19487, (char)41205, (char)24857, (char)18482}, 0) ;
        p147.current_consumed_SET(-2100660288) ;
        p147.battery_remaining_SET((byte) - 7) ;
        p147.id_SET((char)101) ;
        LoopBackDemoChannel.instance.send(p147);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_AUTOPILOT_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.os_custom_version_GET(),  new char[] {(char)176, (char)220, (char)215, (char)48, (char)109, (char)235, (char)167, (char)191}));
            assert(pack.board_version_GET() == 3262758138L);
            assert(Arrays.equals(pack.middleware_custom_version_GET(),  new char[] {(char)194, (char)126, (char)113, (char)118, (char)205, (char)202, (char)249, (char)227}));
            assert(pack.product_id_GET() == (char)33727);
            assert(pack.flight_sw_version_GET() == 3700905361L);
            assert(Arrays.equals(pack.flight_custom_version_GET(),  new char[] {(char)80, (char)18, (char)210, (char)100, (char)193, (char)187, (char)48, (char)188}));
            assert(pack.uid_GET() == 1627364656524852959L);
            assert(Arrays.equals(pack.uid2_TRY(ph),  new char[] {(char)82, (char)239, (char)24, (char)224, (char)186, (char)28, (char)182, (char)211, (char)236, (char)93, (char)90, (char)65, (char)134, (char)212, (char)167, (char)242, (char)202, (char)82}));
            assert(pack.capabilities_GET() == MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION);
            assert(pack.middleware_sw_version_GET() == 440101423L);
            assert(pack.vendor_id_GET() == (char)43812);
            assert(pack.os_sw_version_GET() == 288976233L);
        });
        DemoDevice.AUTOPILOT_VERSION p148 = LoopBackDemoChannel.new_AUTOPILOT_VERSION();
        PH.setPack(p148);
        p148.middleware_custom_version_SET(new char[] {(char)194, (char)126, (char)113, (char)118, (char)205, (char)202, (char)249, (char)227}, 0) ;
        p148.uid2_SET(new char[] {(char)82, (char)239, (char)24, (char)224, (char)186, (char)28, (char)182, (char)211, (char)236, (char)93, (char)90, (char)65, (char)134, (char)212, (char)167, (char)242, (char)202, (char)82}, 0, PH) ;
        p148.product_id_SET((char)33727) ;
        p148.uid_SET(1627364656524852959L) ;
        p148.vendor_id_SET((char)43812) ;
        p148.middleware_sw_version_SET(440101423L) ;
        p148.flight_custom_version_SET(new char[] {(char)80, (char)18, (char)210, (char)100, (char)193, (char)187, (char)48, (char)188}, 0) ;
        p148.os_sw_version_SET(288976233L) ;
        p148.board_version_SET(3262758138L) ;
        p148.os_custom_version_SET(new char[] {(char)176, (char)220, (char)215, (char)48, (char)109, (char)235, (char)167, (char)191}, 0) ;
        p148.capabilities_SET(MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION) ;
        p148.flight_sw_version_SET(3700905361L) ;
        LoopBackDemoChannel.instance.send(p148);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LANDING_TARGET.add((src, ph, pack) ->
        {
            assert(pack.frame_GET() == MAV_FRAME.MAV_FRAME_MISSION);
            assert(pack.angle_x_GET() == -2.475001E38F);
            assert(pack.type_GET() == LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON);
            assert(pack.angle_y_GET() == 3.5694483E37F);
            assert(pack.x_TRY(ph) == 1.4777502E38F);
            assert(Arrays.equals(pack.q_TRY(ph),  new float[] {2.2650357E38F, 3.1730684E38F, -2.8856342E38F, -2.9148496E38F}));
            assert(pack.time_usec_GET() == 5628055670458525285L);
            assert(pack.target_num_GET() == (char)233);
            assert(pack.position_valid_TRY(ph) == (char)217);
            assert(pack.y_TRY(ph) == 2.952223E38F);
            assert(pack.z_TRY(ph) == -2.648604E38F);
            assert(pack.size_y_GET() == -1.2082552E38F);
            assert(pack.distance_GET() == 3.178482E38F);
            assert(pack.size_x_GET() == 2.38772E38F);
        });
        DemoDevice.LANDING_TARGET p149 = LoopBackDemoChannel.new_LANDING_TARGET();
        PH.setPack(p149);
        p149.position_valid_SET((char)217, PH) ;
        p149.size_x_SET(2.38772E38F) ;
        p149.z_SET(-2.648604E38F, PH) ;
        p149.distance_SET(3.178482E38F) ;
        p149.angle_y_SET(3.5694483E37F) ;
        p149.type_SET(LANDING_TARGET_TYPE.LANDING_TARGET_TYPE_LIGHT_BEACON) ;
        p149.target_num_SET((char)233) ;
        p149.size_y_SET(-1.2082552E38F) ;
        p149.time_usec_SET(5628055670458525285L) ;
        p149.y_SET(2.952223E38F, PH) ;
        p149.q_SET(new float[] {2.2650357E38F, 3.1730684E38F, -2.8856342E38F, -2.9148496E38F}, 0, PH) ;
        p149.x_SET(1.4777502E38F, PH) ;
        p149.frame_SET(MAV_FRAME.MAV_FRAME_MISSION) ;
        p149.angle_x_SET(-2.475001E38F) ;
        LoopBackDemoChannel.instance.send(p149);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CPU_LOAD.add((src, ph, pack) ->
        {
            assert(pack.ctrlLoad_GET() == (char)110);
            assert(pack.sensLoad_GET() == (char)196);
            assert(pack.batVolt_GET() == (char)40006);
        });
        DemoDevice.CPU_LOAD p170 = LoopBackDemoChannel.new_CPU_LOAD();
        PH.setPack(p170);
        p170.sensLoad_SET((char)196) ;
        p170.batVolt_SET((char)40006) ;
        p170.ctrlLoad_SET((char)110) ;
        LoopBackDemoChannel.instance.send(p170);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SENSOR_BIAS.add((src, ph, pack) ->
        {
            assert(pack.gzBias_GET() == -2.407354E38F);
            assert(pack.gyBias_GET() == -1.2110544E38F);
            assert(pack.axBias_GET() == -2.1834716E38F);
            assert(pack.ayBias_GET() == -3.1130527E38F);
            assert(pack.azBias_GET() == -1.5167027E37F);
            assert(pack.gxBias_GET() == 1.5004325E38F);
        });
        DemoDevice.SENSOR_BIAS p172 = LoopBackDemoChannel.new_SENSOR_BIAS();
        PH.setPack(p172);
        p172.gzBias_SET(-2.407354E38F) ;
        p172.gyBias_SET(-1.2110544E38F) ;
        p172.gxBias_SET(1.5004325E38F) ;
        p172.azBias_SET(-1.5167027E37F) ;
        p172.axBias_SET(-2.1834716E38F) ;
        p172.ayBias_SET(-3.1130527E38F) ;
        LoopBackDemoChannel.instance.send(p172);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DIAGNOSTIC.add((src, ph, pack) ->
        {
            assert(pack.diagSh1_GET() == (short) -16738);
            assert(pack.diagSh3_GET() == (short)11073);
            assert(pack.diagSh2_GET() == (short)8647);
            assert(pack.diagFl1_GET() == -1.1166036E38F);
            assert(pack.diagFl2_GET() == -1.4071842E38F);
            assert(pack.diagFl3_GET() == 2.858547E37F);
        });
        DemoDevice.DIAGNOSTIC p173 = LoopBackDemoChannel.new_DIAGNOSTIC();
        PH.setPack(p173);
        p173.diagFl1_SET(-1.1166036E38F) ;
        p173.diagFl3_SET(2.858547E37F) ;
        p173.diagFl2_SET(-1.4071842E38F) ;
        p173.diagSh2_SET((short)8647) ;
        p173.diagSh3_SET((short)11073) ;
        p173.diagSh1_SET((short) -16738) ;
        LoopBackDemoChannel.instance.send(p173);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SLUGS_NAVIGATION.add((src, ph, pack) ->
        {
            assert(pack.theta_c_GET() == 1.5497391E38F);
            assert(pack.fromWP_GET() == (char)244);
            assert(pack.psiDot_c_GET() == 1.9934205E38F);
            assert(pack.dist2Go_GET() == -8.744757E37F);
            assert(pack.ay_body_GET() == -1.8653049E38F);
            assert(pack.u_m_GET() == 2.277463E38F);
            assert(pack.phi_c_GET() == -2.922075E38F);
            assert(pack.toWP_GET() == (char)28);
            assert(pack.totalDist_GET() == 1.5162452E38F);
            assert(pack.h_c_GET() == (char)30724);
        });
        DemoDevice.SLUGS_NAVIGATION p176 = LoopBackDemoChannel.new_SLUGS_NAVIGATION();
        PH.setPack(p176);
        p176.theta_c_SET(1.5497391E38F) ;
        p176.psiDot_c_SET(1.9934205E38F) ;
        p176.h_c_SET((char)30724) ;
        p176.ay_body_SET(-1.8653049E38F) ;
        p176.phi_c_SET(-2.922075E38F) ;
        p176.fromWP_SET((char)244) ;
        p176.dist2Go_SET(-8.744757E37F) ;
        p176.toWP_SET((char)28) ;
        p176.totalDist_SET(1.5162452E38F) ;
        p176.u_m_SET(2.277463E38F) ;
        LoopBackDemoChannel.instance.send(p176);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DATA_LOG.add((src, ph, pack) ->
        {
            assert(pack.fl_3_GET() == 5.291593E37F);
            assert(pack.fl_1_GET() == 8.925018E37F);
            assert(pack.fl_5_GET() == -1.8367472E38F);
            assert(pack.fl_4_GET() == -1.5915606E38F);
            assert(pack.fl_2_GET() == 1.5523233E38F);
            assert(pack.fl_6_GET() == 3.0275632E38F);
        });
        DemoDevice.DATA_LOG p177 = LoopBackDemoChannel.new_DATA_LOG();
        PH.setPack(p177);
        p177.fl_4_SET(-1.5915606E38F) ;
        p177.fl_2_SET(1.5523233E38F) ;
        p177.fl_1_SET(8.925018E37F) ;
        p177.fl_3_SET(5.291593E37F) ;
        p177.fl_6_SET(3.0275632E38F) ;
        p177.fl_5_SET(-1.8367472E38F) ;
        LoopBackDemoChannel.instance.send(p177);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_DATE_TIME.add((src, ph, pack) ->
        {
            assert(pack.percentUsed_GET() == (char)113);
            assert(pack.sec_GET() == (char)48);
            assert(pack.visSat_GET() == (char)228);
            assert(pack.day_GET() == (char)255);
            assert(pack.sigUsedMask_GET() == (char)196);
            assert(pack.year_GET() == (char)25);
            assert(pack.GppGl_GET() == (char)215);
            assert(pack.useSat_GET() == (char)17);
            assert(pack.min_GET() == (char)34);
            assert(pack.hour_GET() == (char)131);
            assert(pack.clockStat_GET() == (char)55);
            assert(pack.month_GET() == (char)105);
        });
        DemoDevice.GPS_DATE_TIME p179 = LoopBackDemoChannel.new_GPS_DATE_TIME();
        PH.setPack(p179);
        p179.sec_SET((char)48) ;
        p179.useSat_SET((char)17) ;
        p179.month_SET((char)105) ;
        p179.visSat_SET((char)228) ;
        p179.min_SET((char)34) ;
        p179.sigUsedMask_SET((char)196) ;
        p179.GppGl_SET((char)215) ;
        p179.percentUsed_SET((char)113) ;
        p179.clockStat_SET((char)55) ;
        p179.year_SET((char)25) ;
        p179.day_SET((char)255) ;
        p179.hour_SET((char)131) ;
        LoopBackDemoChannel.instance.send(p179);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MID_LVL_CMDS.add((src, ph, pack) ->
        {
            assert(pack.rCommand_GET() == -2.486737E38F);
            assert(pack.uCommand_GET() == -1.5698951E38F);
            assert(pack.hCommand_GET() == 2.3924577E38F);
            assert(pack.target_GET() == (char)205);
        });
        DemoDevice.MID_LVL_CMDS p180 = LoopBackDemoChannel.new_MID_LVL_CMDS();
        PH.setPack(p180);
        p180.uCommand_SET(-1.5698951E38F) ;
        p180.target_SET((char)205) ;
        p180.hCommand_SET(2.3924577E38F) ;
        p180.rCommand_SET(-2.486737E38F) ;
        LoopBackDemoChannel.instance.send(p180);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CTRL_SRFC_PT.add((src, ph, pack) ->
        {
            assert(pack.target_GET() == (char)48);
            assert(pack.bitfieldPt_GET() == (char)61457);
        });
        DemoDevice.CTRL_SRFC_PT p181 = LoopBackDemoChannel.new_CTRL_SRFC_PT();
        PH.setPack(p181);
        p181.target_SET((char)48) ;
        p181.bitfieldPt_SET((char)61457) ;
        LoopBackDemoChannel.instance.send(p181);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SLUGS_CAMERA_ORDER.add((src, ph, pack) ->
        {
            assert(pack.tilt_GET() == (byte) - 48);
            assert(pack.zoom_GET() == (byte)23);
            assert(pack.target_GET() == (char)213);
            assert(pack.pan_GET() == (byte)97);
            assert(pack.moveHome_GET() == (byte)2);
        });
        DemoDevice.SLUGS_CAMERA_ORDER p184 = LoopBackDemoChannel.new_SLUGS_CAMERA_ORDER();
        PH.setPack(p184);
        p184.zoom_SET((byte)23) ;
        p184.target_SET((char)213) ;
        p184.pan_SET((byte)97) ;
        p184.tilt_SET((byte) - 48) ;
        p184.moveHome_SET((byte)2) ;
        LoopBackDemoChannel.instance.send(p184);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CONTROL_SURFACE.add((src, ph, pack) ->
        {
            assert(pack.mControl_GET() == 2.0819476E38F);
            assert(pack.target_GET() == (char)245);
            assert(pack.idSurface_GET() == (char)163);
            assert(pack.bControl_GET() == 2.3036035E38F);
        });
        DemoDevice.CONTROL_SURFACE p185 = LoopBackDemoChannel.new_CONTROL_SURFACE();
        PH.setPack(p185);
        p185.idSurface_SET((char)163) ;
        p185.target_SET((char)245) ;
        p185.mControl_SET(2.0819476E38F) ;
        p185.bControl_SET(2.3036035E38F) ;
        LoopBackDemoChannel.instance.send(p185);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SLUGS_MOBILE_LOCATION.add((src, ph, pack) ->
        {
            assert(pack.latitude_GET() == -4.822796E37F);
            assert(pack.longitude_GET() == 2.9076576E37F);
            assert(pack.target_GET() == (char)161);
        });
        DemoDevice.SLUGS_MOBILE_LOCATION p186 = LoopBackDemoChannel.new_SLUGS_MOBILE_LOCATION();
        PH.setPack(p186);
        p186.longitude_SET(2.9076576E37F) ;
        p186.latitude_SET(-4.822796E37F) ;
        p186.target_SET((char)161) ;
        LoopBackDemoChannel.instance.send(p186);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SLUGS_CONFIGURATION_CAMERA.add((src, ph, pack) ->
        {
            assert(pack.idOrder_GET() == (char)63);
            assert(pack.target_GET() == (char)153);
            assert(pack.order_GET() == (char)142);
        });
        DemoDevice.SLUGS_CONFIGURATION_CAMERA p188 = LoopBackDemoChannel.new_SLUGS_CONFIGURATION_CAMERA();
        PH.setPack(p188);
        p188.idOrder_SET((char)63) ;
        p188.order_SET((char)142) ;
        p188.target_SET((char)153) ;
        LoopBackDemoChannel.instance.send(p188);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ISR_LOCATION.add((src, ph, pack) ->
        {
            assert(pack.option3_GET() == (char)186);
            assert(pack.option2_GET() == (char)48);
            assert(pack.height_GET() == -7.2437135E37F);
            assert(pack.option1_GET() == (char)11);
            assert(pack.latitude_GET() == -2.757968E38F);
            assert(pack.longitude_GET() == 2.8547649E37F);
            assert(pack.target_GET() == (char)43);
        });
        DemoDevice.ISR_LOCATION p189 = LoopBackDemoChannel.new_ISR_LOCATION();
        PH.setPack(p189);
        p189.height_SET(-7.2437135E37F) ;
        p189.latitude_SET(-2.757968E38F) ;
        p189.option1_SET((char)11) ;
        p189.option2_SET((char)48) ;
        p189.target_SET((char)43) ;
        p189.longitude_SET(2.8547649E37F) ;
        p189.option3_SET((char)186) ;
        LoopBackDemoChannel.instance.send(p189);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VOLT_SENSOR.add((src, ph, pack) ->
        {
            assert(pack.r2Type_GET() == (char)114);
            assert(pack.reading2_GET() == (char)22735);
            assert(pack.voltage_GET() == (char)33501);
        });
        DemoDevice.VOLT_SENSOR p191 = LoopBackDemoChannel.new_VOLT_SENSOR();
        PH.setPack(p191);
        p191.reading2_SET((char)22735) ;
        p191.r2Type_SET((char)114) ;
        p191.voltage_SET((char)33501) ;
        LoopBackDemoChannel.instance.send(p191);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PTZ_STATUS.add((src, ph, pack) ->
        {
            assert(pack.tilt_GET() == (short) -8941);
            assert(pack.pan_GET() == (short)26540);
            assert(pack.zoom_GET() == (char)37);
        });
        DemoDevice.PTZ_STATUS p192 = LoopBackDemoChannel.new_PTZ_STATUS();
        PH.setPack(p192);
        p192.tilt_SET((short) -8941) ;
        p192.zoom_SET((char)37) ;
        p192.pan_SET((short)26540) ;
        LoopBackDemoChannel.instance.send(p192);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAV_STATUS.add((src, ph, pack) ->
        {
            assert(pack.target_GET() == (char)83);
            assert(pack.course_GET() == 1.97711E38F);
            assert(pack.speed_GET() == -1.7686352E38F);
            assert(pack.latitude_GET() == 2.5144338E38F);
            assert(pack.longitude_GET() == -4.033733E37F);
            assert(pack.altitude_GET() == 5.7321674E37F);
        });
        DemoDevice.UAV_STATUS p193 = LoopBackDemoChannel.new_UAV_STATUS();
        PH.setPack(p193);
        p193.longitude_SET(-4.033733E37F) ;
        p193.speed_SET(-1.7686352E38F) ;
        p193.course_SET(1.97711E38F) ;
        p193.altitude_SET(5.7321674E37F) ;
        p193.target_SET((char)83) ;
        p193.latitude_SET(2.5144338E38F) ;
        LoopBackDemoChannel.instance.send(p193);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STATUS_GPS.add((src, ph, pack) ->
        {
            assert(pack.msgsType_GET() == (char)23);
            assert(pack.modeInd_GET() == (char)141);
            assert(pack.csFails_GET() == (char)44818);
            assert(pack.magVar_GET() == 1.3323823E38F);
            assert(pack.magDir_GET() == (byte) - 25);
            assert(pack.posStatus_GET() == (char)47);
            assert(pack.gpsQuality_GET() == (char)170);
        });
        DemoDevice.STATUS_GPS p194 = LoopBackDemoChannel.new_STATUS_GPS();
        PH.setPack(p194);
        p194.msgsType_SET((char)23) ;
        p194.posStatus_SET((char)47) ;
        p194.modeInd_SET((char)141) ;
        p194.magVar_SET(1.3323823E38F) ;
        p194.magDir_SET((byte) - 25) ;
        p194.gpsQuality_SET((char)170) ;
        p194.csFails_SET((char)44818) ;
        LoopBackDemoChannel.instance.send(p194);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NOVATEL_DIAG.add((src, ph, pack) ->
        {
            assert(pack.receiverStatus_GET() == 3437312301L);
            assert(pack.velType_GET() == (char)21);
            assert(pack.posSolAge_GET() == 3.1904514E38F);
            assert(pack.solStatus_GET() == (char)83);
            assert(pack.timeStatus_GET() == (char)246);
            assert(pack.posType_GET() == (char)215);
            assert(pack.csFails_GET() == (char)138);
        });
        DemoDevice.NOVATEL_DIAG p195 = LoopBackDemoChannel.new_NOVATEL_DIAG();
        PH.setPack(p195);
        p195.timeStatus_SET((char)246) ;
        p195.receiverStatus_SET(3437312301L) ;
        p195.velType_SET((char)21) ;
        p195.csFails_SET((char)138) ;
        p195.posSolAge_SET(3.1904514E38F) ;
        p195.solStatus_SET((char)83) ;
        p195.posType_SET((char)215) ;
        LoopBackDemoChannel.instance.send(p195);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SENSOR_DIAG.add((src, ph, pack) ->
        {
            assert(pack.char1_GET() == (byte)24);
            assert(pack.float1_GET() == 3.1622242E38F);
            assert(pack.int1_GET() == (short) -22946);
            assert(pack.float2_GET() == 1.2773043E38F);
        });
        DemoDevice.SENSOR_DIAG p196 = LoopBackDemoChannel.new_SENSOR_DIAG();
        PH.setPack(p196);
        p196.char1_SET((byte)24) ;
        p196.float1_SET(3.1622242E38F) ;
        p196.float2_SET(1.2773043E38F) ;
        p196.int1_SET((short) -22946) ;
        LoopBackDemoChannel.instance.send(p196);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BOOT.add((src, ph, pack) ->
        {
            assert(pack.version_GET() == 2808109032L);
        });
        DemoDevice.BOOT p197 = LoopBackDemoChannel.new_BOOT();
        PH.setPack(p197);
        p197.version_SET(2808109032L) ;
        LoopBackDemoChannel.instance.send(p197);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ESTIMATOR_STATUS.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE);
            assert(pack.pos_vert_accuracy_GET() == 1.7125343E38F);
            assert(pack.pos_horiz_accuracy_GET() == -3.1618728E37F);
            assert(pack.pos_vert_ratio_GET() == 4.0684136E37F);
            assert(pack.time_usec_GET() == 9115499228701975869L);
            assert(pack.vel_ratio_GET() == 9.895298E37F);
            assert(pack.mag_ratio_GET() == 1.0794903E38F);
            assert(pack.tas_ratio_GET() == 9.661729E37F);
            assert(pack.pos_horiz_ratio_GET() == 1.603225E38F);
            assert(pack.hagl_ratio_GET() == 1.3614417E38F);
        });
        DemoDevice.ESTIMATOR_STATUS p230 = LoopBackDemoChannel.new_ESTIMATOR_STATUS();
        PH.setPack(p230);
        p230.vel_ratio_SET(9.895298E37F) ;
        p230.pos_horiz_accuracy_SET(-3.1618728E37F) ;
        p230.time_usec_SET(9115499228701975869L) ;
        p230.tas_ratio_SET(9.661729E37F) ;
        p230.hagl_ratio_SET(1.3614417E38F) ;
        p230.flags_SET(ESTIMATOR_STATUS_FLAGS.ESTIMATOR_CONST_POS_MODE) ;
        p230.pos_vert_accuracy_SET(1.7125343E38F) ;
        p230.pos_vert_ratio_SET(4.0684136E37F) ;
        p230.pos_horiz_ratio_SET(1.603225E38F) ;
        p230.mag_ratio_SET(1.0794903E38F) ;
        LoopBackDemoChannel.instance.send(p230);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIND_COV.add((src, ph, pack) ->
        {
            assert(pack.wind_alt_GET() == 5.593711E37F);
            assert(pack.var_horiz_GET() == -2.461313E38F);
            assert(pack.var_vert_GET() == 1.2771102E38F);
            assert(pack.horiz_accuracy_GET() == 2.1384355E38F);
            assert(pack.vert_accuracy_GET() == -3.2484441E38F);
            assert(pack.time_usec_GET() == 7149792340954581339L);
            assert(pack.wind_y_GET() == 2.4167373E38F);
            assert(pack.wind_x_GET() == 3.1513275E38F);
            assert(pack.wind_z_GET() == -2.3890245E38F);
        });
        DemoDevice.WIND_COV p231 = LoopBackDemoChannel.new_WIND_COV();
        PH.setPack(p231);
        p231.wind_alt_SET(5.593711E37F) ;
        p231.time_usec_SET(7149792340954581339L) ;
        p231.horiz_accuracy_SET(2.1384355E38F) ;
        p231.wind_x_SET(3.1513275E38F) ;
        p231.wind_y_SET(2.4167373E38F) ;
        p231.var_horiz_SET(-2.461313E38F) ;
        p231.wind_z_SET(-2.3890245E38F) ;
        p231.var_vert_SET(1.2771102E38F) ;
        p231.vert_accuracy_SET(-3.2484441E38F) ;
        LoopBackDemoChannel.instance.send(p231);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_INPUT.add((src, ph, pack) ->
        {
            assert(pack.vdop_GET() == 6.59185E37F);
            assert(pack.lon_GET() == 170369997);
            assert(pack.satellites_visible_GET() == (char)144);
            assert(pack.hdop_GET() == -2.7099155E38F);
            assert(pack.vert_accuracy_GET() == -2.9569123E38F);
            assert(pack.lat_GET() == 9977659);
            assert(pack.alt_GET() == 5.861716E37F);
            assert(pack.time_usec_GET() == 8127325439644908670L);
            assert(pack.ve_GET() == -2.1350893E38F);
            assert(pack.speed_accuracy_GET() == -4.5346606E36F);
            assert(pack.time_week_ms_GET() == 3055409492L);
            assert(pack.horiz_accuracy_GET() == 1.0014816E38F);
            assert(pack.ignore_flags_GET() == GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP);
            assert(pack.fix_type_GET() == (char)92);
            assert(pack.vd_GET() == 7.4128435E37F);
            assert(pack.gps_id_GET() == (char)63);
            assert(pack.time_week_GET() == (char)43363);
            assert(pack.vn_GET() == -6.8207776E37F);
        });
        DemoDevice.GPS_INPUT p232 = LoopBackDemoChannel.new_GPS_INPUT();
        PH.setPack(p232);
        p232.ignore_flags_SET(GPS_INPUT_IGNORE_FLAGS.GPS_INPUT_IGNORE_FLAG_VDOP) ;
        p232.hdop_SET(-2.7099155E38F) ;
        p232.time_usec_SET(8127325439644908670L) ;
        p232.fix_type_SET((char)92) ;
        p232.lon_SET(170369997) ;
        p232.time_week_SET((char)43363) ;
        p232.vd_SET(7.4128435E37F) ;
        p232.speed_accuracy_SET(-4.5346606E36F) ;
        p232.alt_SET(5.861716E37F) ;
        p232.lat_SET(9977659) ;
        p232.vert_accuracy_SET(-2.9569123E38F) ;
        p232.vdop_SET(6.59185E37F) ;
        p232.horiz_accuracy_SET(1.0014816E38F) ;
        p232.vn_SET(-6.8207776E37F) ;
        p232.gps_id_SET((char)63) ;
        p232.time_week_ms_SET(3055409492L) ;
        p232.ve_SET(-2.1350893E38F) ;
        p232.satellites_visible_SET((char)144) ;
        LoopBackDemoChannel.instance.send(p232);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_GPS_RTCM_DATA.add((src, ph, pack) ->
        {
            assert(pack.flags_GET() == (char)72);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)206, (char)160, (char)210, (char)116, (char)204, (char)227, (char)74, (char)115, (char)77, (char)165, (char)200, (char)105, (char)247, (char)129, (char)31, (char)203, (char)153, (char)254, (char)129, (char)15, (char)105, (char)28, (char)240, (char)193, (char)171, (char)159, (char)34, (char)66, (char)170, (char)171, (char)132, (char)19, (char)23, (char)148, (char)223, (char)188, (char)225, (char)251, (char)136, (char)31, (char)247, (char)241, (char)138, (char)56, (char)168, (char)130, (char)96, (char)195, (char)191, (char)67, (char)82, (char)49, (char)78, (char)199, (char)185, (char)92, (char)185, (char)238, (char)247, (char)12, (char)224, (char)222, (char)13, (char)145, (char)210, (char)109, (char)170, (char)125, (char)82, (char)43, (char)45, (char)73, (char)180, (char)4, (char)222, (char)6, (char)50, (char)71, (char)231, (char)125, (char)196, (char)3, (char)42, (char)182, (char)38, (char)92, (char)143, (char)173, (char)24, (char)239, (char)187, (char)251, (char)202, (char)128, (char)112, (char)197, (char)157, (char)146, (char)228, (char)221, (char)90, (char)175, (char)86, (char)93, (char)60, (char)145, (char)111, (char)245, (char)233, (char)191, (char)229, (char)61, (char)86, (char)18, (char)35, (char)137, (char)183, (char)75, (char)222, (char)80, (char)106, (char)143, (char)184, (char)60, (char)159, (char)65, (char)199, (char)227, (char)239, (char)155, (char)91, (char)50, (char)18, (char)224, (char)220, (char)120, (char)137, (char)47, (char)1, (char)93, (char)180, (char)208, (char)78, (char)44, (char)29, (char)223, (char)24, (char)220, (char)121, (char)108, (char)242, (char)244, (char)159, (char)252, (char)65, (char)92, (char)213, (char)70, (char)76, (char)149, (char)70, (char)174, (char)20, (char)183, (char)233, (char)210, (char)60, (char)125, (char)109, (char)119, (char)93, (char)23, (char)248, (char)133, (char)121, (char)120, (char)229, (char)20, (char)195, (char)217}));
            assert(pack.len_GET() == (char)237);
        });
        DemoDevice.GPS_RTCM_DATA p233 = LoopBackDemoChannel.new_GPS_RTCM_DATA();
        PH.setPack(p233);
        p233.len_SET((char)237) ;
        p233.flags_SET((char)72) ;
        p233.data__SET(new char[] {(char)206, (char)160, (char)210, (char)116, (char)204, (char)227, (char)74, (char)115, (char)77, (char)165, (char)200, (char)105, (char)247, (char)129, (char)31, (char)203, (char)153, (char)254, (char)129, (char)15, (char)105, (char)28, (char)240, (char)193, (char)171, (char)159, (char)34, (char)66, (char)170, (char)171, (char)132, (char)19, (char)23, (char)148, (char)223, (char)188, (char)225, (char)251, (char)136, (char)31, (char)247, (char)241, (char)138, (char)56, (char)168, (char)130, (char)96, (char)195, (char)191, (char)67, (char)82, (char)49, (char)78, (char)199, (char)185, (char)92, (char)185, (char)238, (char)247, (char)12, (char)224, (char)222, (char)13, (char)145, (char)210, (char)109, (char)170, (char)125, (char)82, (char)43, (char)45, (char)73, (char)180, (char)4, (char)222, (char)6, (char)50, (char)71, (char)231, (char)125, (char)196, (char)3, (char)42, (char)182, (char)38, (char)92, (char)143, (char)173, (char)24, (char)239, (char)187, (char)251, (char)202, (char)128, (char)112, (char)197, (char)157, (char)146, (char)228, (char)221, (char)90, (char)175, (char)86, (char)93, (char)60, (char)145, (char)111, (char)245, (char)233, (char)191, (char)229, (char)61, (char)86, (char)18, (char)35, (char)137, (char)183, (char)75, (char)222, (char)80, (char)106, (char)143, (char)184, (char)60, (char)159, (char)65, (char)199, (char)227, (char)239, (char)155, (char)91, (char)50, (char)18, (char)224, (char)220, (char)120, (char)137, (char)47, (char)1, (char)93, (char)180, (char)208, (char)78, (char)44, (char)29, (char)223, (char)24, (char)220, (char)121, (char)108, (char)242, (char)244, (char)159, (char)252, (char)65, (char)92, (char)213, (char)70, (char)76, (char)149, (char)70, (char)174, (char)20, (char)183, (char)233, (char)210, (char)60, (char)125, (char)109, (char)119, (char)93, (char)23, (char)248, (char)133, (char)121, (char)120, (char)229, (char)20, (char)195, (char)217}, 0) ;
        LoopBackDemoChannel.instance.send(p233);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HIGH_LATENCY.add((src, ph, pack) ->
        {
            assert(pack.battery_remaining_GET() == (char)231);
            assert(pack.wp_distance_GET() == (char)52017);
            assert(pack.base_mode_GET() == MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED);
            assert(pack.latitude_GET() == -609340865);
            assert(pack.altitude_amsl_GET() == (short)8840);
            assert(pack.throttle_GET() == (byte)15);
            assert(pack.heading_GET() == (char)22355);
            assert(pack.pitch_GET() == (short) -11413);
            assert(pack.failsafe_GET() == (char)105);
            assert(pack.temperature_GET() == (byte)3);
            assert(pack.heading_sp_GET() == (short)1066);
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED);
            assert(pack.climb_rate_GET() == (byte) - 62);
            assert(pack.gps_fix_type_GET() == GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED);
            assert(pack.custom_mode_GET() == 3207868027L);
            assert(pack.altitude_sp_GET() == (short)18582);
            assert(pack.airspeed_sp_GET() == (char)81);
            assert(pack.roll_GET() == (short)29729);
            assert(pack.gps_nsat_GET() == (char)74);
            assert(pack.airspeed_GET() == (char)173);
            assert(pack.wp_num_GET() == (char)125);
            assert(pack.temperature_air_GET() == (byte) - 26);
            assert(pack.longitude_GET() == -1758004954);
            assert(pack.groundspeed_GET() == (char)80);
        });
        DemoDevice.HIGH_LATENCY p234 = LoopBackDemoChannel.new_HIGH_LATENCY();
        PH.setPack(p234);
        p234.groundspeed_SET((char)80) ;
        p234.throttle_SET((byte)15) ;
        p234.base_mode_SET(MAV_MODE_FLAG.MAV_MODE_FLAG_STABILIZE_ENABLED) ;
        p234.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_UNDEFINED) ;
        p234.wp_num_SET((char)125) ;
        p234.wp_distance_SET((char)52017) ;
        p234.custom_mode_SET(3207868027L) ;
        p234.temperature_SET((byte)3) ;
        p234.battery_remaining_SET((char)231) ;
        p234.longitude_SET(-1758004954) ;
        p234.heading_SET((char)22355) ;
        p234.gps_fix_type_SET(GPS_FIX_TYPE.GPS_FIX_TYPE_RTK_FIXED) ;
        p234.climb_rate_SET((byte) - 62) ;
        p234.gps_nsat_SET((char)74) ;
        p234.heading_sp_SET((short)1066) ;
        p234.failsafe_SET((char)105) ;
        p234.altitude_amsl_SET((short)8840) ;
        p234.altitude_sp_SET((short)18582) ;
        p234.pitch_SET((short) -11413) ;
        p234.temperature_air_SET((byte) - 26) ;
        p234.latitude_SET(-609340865) ;
        p234.roll_SET((short)29729) ;
        p234.airspeed_SET((char)173) ;
        p234.airspeed_sp_SET((char)81) ;
        LoopBackDemoChannel.instance.send(p234);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIBRATION.add((src, ph, pack) ->
        {
            assert(pack.time_usec_GET() == 7235224783413243587L);
            assert(pack.clipping_2_GET() == 3488870258L);
            assert(pack.clipping_1_GET() == 2782210078L);
            assert(pack.vibration_y_GET() == -2.756601E38F);
            assert(pack.vibration_x_GET() == -5.358948E37F);
            assert(pack.vibration_z_GET() == -2.8167526E37F);
            assert(pack.clipping_0_GET() == 125580350L);
        });
        DemoDevice.VIBRATION p241 = LoopBackDemoChannel.new_VIBRATION();
        PH.setPack(p241);
        p241.vibration_x_SET(-5.358948E37F) ;
        p241.vibration_y_SET(-2.756601E38F) ;
        p241.clipping_0_SET(125580350L) ;
        p241.clipping_2_SET(3488870258L) ;
        p241.time_usec_SET(7235224783413243587L) ;
        p241.clipping_1_SET(2782210078L) ;
        p241.vibration_z_SET(-2.8167526E37F) ;
        LoopBackDemoChannel.instance.send(p241);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == 2.8789255E37F);
            assert(pack.z_GET() == -2.5147421E38F);
            assert(pack.altitude_GET() == 344082635);
            assert(pack.latitude_GET() == -443686237);
            assert(pack.longitude_GET() == -1231992123);
            assert(pack.approach_x_GET() == -6.4524343E37F);
            assert(pack.time_usec_TRY(ph) == 7755691779320943505L);
            assert(pack.approach_y_GET() == 2.2711723E38F);
            assert(pack.approach_z_GET() == -3.6835761E37F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-8.923968E37F, 1.4091863E38F, 2.831836E38F, 2.2272232E38F}));
            assert(pack.y_GET() == 1.4539381E38F);
        });
        DemoDevice.HOME_POSITION p242 = LoopBackDemoChannel.new_HOME_POSITION();
        PH.setPack(p242);
        p242.approach_y_SET(2.2711723E38F) ;
        p242.altitude_SET(344082635) ;
        p242.approach_z_SET(-3.6835761E37F) ;
        p242.x_SET(2.8789255E37F) ;
        p242.time_usec_SET(7755691779320943505L, PH) ;
        p242.longitude_SET(-1231992123) ;
        p242.z_SET(-2.5147421E38F) ;
        p242.y_SET(1.4539381E38F) ;
        p242.q_SET(new float[] {-8.923968E37F, 1.4091863E38F, 2.831836E38F, 2.2272232E38F}, 0) ;
        p242.approach_x_SET(-6.4524343E37F) ;
        p242.latitude_SET(-443686237) ;
        LoopBackDemoChannel.instance.send(p242);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_HOME_POSITION.add((src, ph, pack) ->
        {
            assert(pack.longitude_GET() == 278712870);
            assert(pack.y_GET() == 3.0850867E38F);
            assert(pack.latitude_GET() == -334604001);
            assert(pack.altitude_GET() == 434946938);
            assert(pack.approach_z_GET() == 1.2214491E38F);
            assert(pack.x_GET() == -1.409836E38F);
            assert(pack.approach_x_GET() == -2.052427E38F);
            assert(pack.time_usec_TRY(ph) == 283309714987754162L);
            assert(pack.target_system_GET() == (char)252);
            assert(pack.approach_y_GET() == -2.1209713E38F);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-2.7851272E37F, -1.7247995E38F, 1.4487343E38F, -3.239215E38F}));
            assert(pack.z_GET() == 2.759389E38F);
        });
        DemoDevice.SET_HOME_POSITION p243 = LoopBackDemoChannel.new_SET_HOME_POSITION();
        PH.setPack(p243);
        p243.longitude_SET(278712870) ;
        p243.x_SET(-1.409836E38F) ;
        p243.latitude_SET(-334604001) ;
        p243.time_usec_SET(283309714987754162L, PH) ;
        p243.altitude_SET(434946938) ;
        p243.approach_x_SET(-2.052427E38F) ;
        p243.approach_z_SET(1.2214491E38F) ;
        p243.approach_y_SET(-2.1209713E38F) ;
        p243.q_SET(new float[] {-2.7851272E37F, -1.7247995E38F, 1.4487343E38F, -3.239215E38F}, 0) ;
        p243.z_SET(2.759389E38F) ;
        p243.y_SET(3.0850867E38F) ;
        p243.target_system_SET((char)252) ;
        LoopBackDemoChannel.instance.send(p243);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MESSAGE_INTERVAL.add((src, ph, pack) ->
        {
            assert(pack.interval_us_GET() == 1427802605);
            assert(pack.message_id_GET() == (char)63194);
        });
        DemoDevice.MESSAGE_INTERVAL p244 = LoopBackDemoChannel.new_MESSAGE_INTERVAL();
        PH.setPack(p244);
        p244.interval_us_SET(1427802605) ;
        p244.message_id_SET((char)63194) ;
        LoopBackDemoChannel.instance.send(p244);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_EXTENDED_SYS_STATE.add((src, ph, pack) ->
        {
            assert(pack.landed_state_GET() == MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR);
            assert(pack.vtol_state_GET() == MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC);
        });
        DemoDevice.EXTENDED_SYS_STATE p245 = LoopBackDemoChannel.new_EXTENDED_SYS_STATE();
        PH.setPack(p245);
        p245.landed_state_SET(MAV_LANDED_STATE.MAV_LANDED_STATE_IN_AIR) ;
        p245.vtol_state_SET(MAV_VTOL_STATE.MAV_VTOL_STATE_TRANSITION_TO_MC) ;
        LoopBackDemoChannel.instance.send(p245);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_ADSB_VEHICLE.add((src, ph, pack) ->
        {
            assert(pack.lat_GET() == -691222509);
            assert(pack.callsign_LEN(ph) == 4);
            assert(pack.callsign_TRY(ph).equals("faep"));
            assert(pack.heading_GET() == (char)34542);
            assert(pack.ver_velocity_GET() == (short) -30060);
            assert(pack.squawk_GET() == (char)3757);
            assert(pack.tslc_GET() == (char)246);
            assert(pack.lon_GET() == 178624540);
            assert(pack.flags_GET() == ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE);
            assert(pack.hor_velocity_GET() == (char)22009);
            assert(pack.altitude_type_GET() == ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC);
            assert(pack.altitude_GET() == 587680906);
            assert(pack.ICAO_address_GET() == 1761967855L);
            assert(pack.emitter_type_GET() == ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SMALL);
        });
        DemoDevice.ADSB_VEHICLE p246 = LoopBackDemoChannel.new_ADSB_VEHICLE();
        PH.setPack(p246);
        p246.emitter_type_SET(ADSB_EMITTER_TYPE.ADSB_EMITTER_TYPE_SMALL) ;
        p246.heading_SET((char)34542) ;
        p246.tslc_SET((char)246) ;
        p246.ICAO_address_SET(1761967855L) ;
        p246.flags_SET(ADSB_FLAGS.ADSB_FLAGS_VALID_ALTITUDE) ;
        p246.hor_velocity_SET((char)22009) ;
        p246.ver_velocity_SET((short) -30060) ;
        p246.squawk_SET((char)3757) ;
        p246.lon_SET(178624540) ;
        p246.altitude_SET(587680906) ;
        p246.callsign_SET("faep", PH) ;
        p246.altitude_type_SET(ADSB_ALTITUDE_TYPE.ADSB_ALTITUDE_TYPE_GEOMETRIC) ;
        p246.lat_SET(-691222509) ;
        LoopBackDemoChannel.instance.send(p246);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_COLLISION.add((src, ph, pack) ->
        {
            assert(pack.id_GET() == 3779330762L);
            assert(pack.threat_level_GET() == MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE);
            assert(pack.horizontal_minimum_delta_GET() == 3.3089553E38F);
            assert(pack.time_to_minimum_delta_GET() == 1.1188062E37F);
            assert(pack.altitude_minimum_delta_GET() == 5.0374254E37F);
            assert(pack.action_GET() == MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE);
            assert(pack.src__GET() == MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB);
        });
        DemoDevice.COLLISION p247 = LoopBackDemoChannel.new_COLLISION();
        PH.setPack(p247);
        p247.action_SET(MAV_COLLISION_ACTION.MAV_COLLISION_ACTION_NONE) ;
        p247.horizontal_minimum_delta_SET(3.3089553E38F) ;
        p247.time_to_minimum_delta_SET(1.1188062E37F) ;
        p247.src__SET(MAV_COLLISION_SRC.MAV_COLLISION_SRC_ADSB) ;
        p247.altitude_minimum_delta_SET(5.0374254E37F) ;
        p247.threat_level_SET(MAV_COLLISION_THREAT_LEVEL.MAV_COLLISION_THREAT_LEVEL_NONE) ;
        p247.id_SET(3779330762L) ;
        LoopBackDemoChannel.instance.send(p247);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_V2_EXTENSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.payload_GET(),  new char[] {(char)119, (char)192, (char)244, (char)250, (char)126, (char)204, (char)72, (char)68, (char)120, (char)6, (char)34, (char)21, (char)13, (char)123, (char)4, (char)187, (char)141, (char)92, (char)69, (char)30, (char)127, (char)227, (char)89, (char)69, (char)226, (char)255, (char)192, (char)83, (char)199, (char)4, (char)219, (char)183, (char)142, (char)167, (char)201, (char)35, (char)130, (char)181, (char)19, (char)47, (char)17, (char)89, (char)170, (char)163, (char)169, (char)77, (char)253, (char)240, (char)193, (char)138, (char)199, (char)211, (char)14, (char)223, (char)145, (char)34, (char)18, (char)6, (char)176, (char)205, (char)236, (char)169, (char)128, (char)144, (char)2, (char)89, (char)195, (char)55, (char)212, (char)85, (char)123, (char)108, (char)89, (char)90, (char)81, (char)173, (char)73, (char)75, (char)175, (char)58, (char)77, (char)127, (char)97, (char)56, (char)186, (char)132, (char)230, (char)154, (char)169, (char)49, (char)57, (char)191, (char)116, (char)7, (char)239, (char)237, (char)73, (char)196, (char)112, (char)195, (char)152, (char)164, (char)31, (char)206, (char)104, (char)77, (char)151, (char)200, (char)29, (char)92, (char)93, (char)122, (char)162, (char)63, (char)20, (char)35, (char)85, (char)144, (char)115, (char)58, (char)54, (char)225, (char)198, (char)34, (char)47, (char)164, (char)195, (char)136, (char)200, (char)142, (char)210, (char)232, (char)71, (char)131, (char)16, (char)55, (char)65, (char)157, (char)224, (char)249, (char)41, (char)94, (char)28, (char)117, (char)45, (char)69, (char)89, (char)222, (char)44, (char)212, (char)119, (char)188, (char)252, (char)92, (char)33, (char)224, (char)209, (char)53, (char)145, (char)252, (char)222, (char)105, (char)220, (char)252, (char)91, (char)21, (char)38, (char)217, (char)123, (char)170, (char)99, (char)49, (char)92, (char)4, (char)214, (char)194, (char)3, (char)19, (char)89, (char)231, (char)168, (char)76, (char)136, (char)103, (char)67, (char)213, (char)58, (char)0, (char)101, (char)125, (char)105, (char)174, (char)74, (char)238, (char)84, (char)183, (char)99, (char)96, (char)221, (char)198, (char)17, (char)134, (char)126, (char)64, (char)187, (char)97, (char)231, (char)152, (char)141, (char)57, (char)64, (char)247, (char)252, (char)255, (char)86, (char)40, (char)54, (char)211, (char)169, (char)187, (char)93, (char)112, (char)200, (char)188, (char)68, (char)195, (char)133, (char)51, (char)80, (char)172, (char)244, (char)210, (char)183, (char)57, (char)252, (char)141, (char)253, (char)48, (char)112, (char)64, (char)77, (char)70, (char)182, (char)96, (char)122, (char)161, (char)74, (char)17, (char)143}));
            assert(pack.message_type_GET() == (char)15184);
            assert(pack.target_system_GET() == (char)255);
            assert(pack.target_component_GET() == (char)155);
            assert(pack.target_network_GET() == (char)201);
        });
        DemoDevice.V2_EXTENSION p248 = LoopBackDemoChannel.new_V2_EXTENSION();
        PH.setPack(p248);
        p248.target_network_SET((char)201) ;
        p248.message_type_SET((char)15184) ;
        p248.target_system_SET((char)255) ;
        p248.target_component_SET((char)155) ;
        p248.payload_SET(new char[] {(char)119, (char)192, (char)244, (char)250, (char)126, (char)204, (char)72, (char)68, (char)120, (char)6, (char)34, (char)21, (char)13, (char)123, (char)4, (char)187, (char)141, (char)92, (char)69, (char)30, (char)127, (char)227, (char)89, (char)69, (char)226, (char)255, (char)192, (char)83, (char)199, (char)4, (char)219, (char)183, (char)142, (char)167, (char)201, (char)35, (char)130, (char)181, (char)19, (char)47, (char)17, (char)89, (char)170, (char)163, (char)169, (char)77, (char)253, (char)240, (char)193, (char)138, (char)199, (char)211, (char)14, (char)223, (char)145, (char)34, (char)18, (char)6, (char)176, (char)205, (char)236, (char)169, (char)128, (char)144, (char)2, (char)89, (char)195, (char)55, (char)212, (char)85, (char)123, (char)108, (char)89, (char)90, (char)81, (char)173, (char)73, (char)75, (char)175, (char)58, (char)77, (char)127, (char)97, (char)56, (char)186, (char)132, (char)230, (char)154, (char)169, (char)49, (char)57, (char)191, (char)116, (char)7, (char)239, (char)237, (char)73, (char)196, (char)112, (char)195, (char)152, (char)164, (char)31, (char)206, (char)104, (char)77, (char)151, (char)200, (char)29, (char)92, (char)93, (char)122, (char)162, (char)63, (char)20, (char)35, (char)85, (char)144, (char)115, (char)58, (char)54, (char)225, (char)198, (char)34, (char)47, (char)164, (char)195, (char)136, (char)200, (char)142, (char)210, (char)232, (char)71, (char)131, (char)16, (char)55, (char)65, (char)157, (char)224, (char)249, (char)41, (char)94, (char)28, (char)117, (char)45, (char)69, (char)89, (char)222, (char)44, (char)212, (char)119, (char)188, (char)252, (char)92, (char)33, (char)224, (char)209, (char)53, (char)145, (char)252, (char)222, (char)105, (char)220, (char)252, (char)91, (char)21, (char)38, (char)217, (char)123, (char)170, (char)99, (char)49, (char)92, (char)4, (char)214, (char)194, (char)3, (char)19, (char)89, (char)231, (char)168, (char)76, (char)136, (char)103, (char)67, (char)213, (char)58, (char)0, (char)101, (char)125, (char)105, (char)174, (char)74, (char)238, (char)84, (char)183, (char)99, (char)96, (char)221, (char)198, (char)17, (char)134, (char)126, (char)64, (char)187, (char)97, (char)231, (char)152, (char)141, (char)57, (char)64, (char)247, (char)252, (char)255, (char)86, (char)40, (char)54, (char)211, (char)169, (char)187, (char)93, (char)112, (char)200, (char)188, (char)68, (char)195, (char)133, (char)51, (char)80, (char)172, (char)244, (char)210, (char)183, (char)57, (char)252, (char)141, (char)253, (char)48, (char)112, (char)64, (char)77, (char)70, (char)182, (char)96, (char)122, (char)161, (char)74, (char)17, (char)143}, 0) ;
        LoopBackDemoChannel.instance.send(p248);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MEMORY_VECT.add((src, ph, pack) ->
        {
            assert(pack.ver_GET() == (char)8);
            assert(Arrays.equals(pack.value_GET(),  new byte[] {(byte)9, (byte) - 29, (byte) - 72, (byte) - 2, (byte) - 51, (byte)64, (byte)108, (byte) - 42, (byte) - 91, (byte) - 88, (byte) - 63, (byte) - 97, (byte) - 23, (byte)97, (byte)99, (byte) - 20, (byte)95, (byte) - 26, (byte) - 74, (byte)78, (byte) - 105, (byte)5, (byte)116, (byte)103, (byte) - 22, (byte) - 50, (byte) - 106, (byte)115, (byte)114, (byte) - 120, (byte)87, (byte) - 116}));
            assert(pack.address_GET() == (char)40381);
            assert(pack.type_GET() == (char)102);
        });
        DemoDevice.MEMORY_VECT p249 = LoopBackDemoChannel.new_MEMORY_VECT();
        PH.setPack(p249);
        p249.address_SET((char)40381) ;
        p249.ver_SET((char)8) ;
        p249.type_SET((char)102) ;
        p249.value_SET(new byte[] {(byte)9, (byte) - 29, (byte) - 72, (byte) - 2, (byte) - 51, (byte)64, (byte)108, (byte) - 42, (byte) - 91, (byte) - 88, (byte) - 63, (byte) - 97, (byte) - 23, (byte)97, (byte)99, (byte) - 20, (byte)95, (byte) - 26, (byte) - 74, (byte)78, (byte) - 105, (byte)5, (byte)116, (byte)103, (byte) - 22, (byte) - 50, (byte) - 106, (byte)115, (byte)114, (byte) - 120, (byte)87, (byte) - 116}, 0) ;
        LoopBackDemoChannel.instance.send(p249);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG_VECT.add((src, ph, pack) ->
        {
            assert(pack.x_GET() == -7.149212E37F);
            assert(pack.z_GET() == -3.1892639E38F);
            assert(pack.name_LEN(ph) == 7);
            assert(pack.name_TRY(ph).equals("haryitd"));
            assert(pack.y_GET() == 3.365709E38F);
            assert(pack.time_usec_GET() == 5760202230471622725L);
        });
        DemoDevice.DEBUG_VECT p250 = LoopBackDemoChannel.new_DEBUG_VECT();
        PH.setPack(p250);
        p250.name_SET("haryitd", PH) ;
        p250.y_SET(3.365709E38F) ;
        p250.time_usec_SET(5760202230471622725L) ;
        p250.z_SET(-3.1892639E38F) ;
        p250.x_SET(-7.149212E37F) ;
        LoopBackDemoChannel.instance.send(p250);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_FLOAT.add((src, ph, pack) ->
        {
            assert(pack.name_LEN(ph) == 5);
            assert(pack.name_TRY(ph).equals("zzmje"));
            assert(pack.value_GET() == -3.3834233E37F);
            assert(pack.time_boot_ms_GET() == 1564325924L);
        });
        DemoDevice.NAMED_VALUE_FLOAT p251 = LoopBackDemoChannel.new_NAMED_VALUE_FLOAT();
        PH.setPack(p251);
        p251.name_SET("zzmje", PH) ;
        p251.value_SET(-3.3834233E37F) ;
        p251.time_boot_ms_SET(1564325924L) ;
        LoopBackDemoChannel.instance.send(p251);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_NAMED_VALUE_INT.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1474886489L);
            assert(pack.value_GET() == -1127716190);
            assert(pack.name_LEN(ph) == 5);
            assert(pack.name_TRY(ph).equals("Mwxme"));
        });
        DemoDevice.NAMED_VALUE_INT p252 = LoopBackDemoChannel.new_NAMED_VALUE_INT();
        PH.setPack(p252);
        p252.value_SET(-1127716190) ;
        p252.name_SET("Mwxme", PH) ;
        p252.time_boot_ms_SET(1474886489L) ;
        LoopBackDemoChannel.instance.send(p252);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STATUSTEXT.add((src, ph, pack) ->
        {
            assert(pack.text_LEN(ph) == 20);
            assert(pack.text_TRY(ph).equals("jewmhmhochzqFzwstlzm"));
            assert(pack.severity_GET() == MAV_SEVERITY.MAV_SEVERITY_INFO);
        });
        DemoDevice.STATUSTEXT p253 = LoopBackDemoChannel.new_STATUSTEXT();
        PH.setPack(p253);
        p253.text_SET("jewmhmhochzqFzwstlzm", PH) ;
        p253.severity_SET(MAV_SEVERITY.MAV_SEVERITY_INFO) ;
        LoopBackDemoChannel.instance.send(p253);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_DEBUG.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 727639433L);
            assert(pack.value_GET() == -1.2731438E38F);
            assert(pack.ind_GET() == (char)71);
        });
        DemoDevice.DEBUG p254 = LoopBackDemoChannel.new_DEBUG();
        PH.setPack(p254);
        p254.time_boot_ms_SET(727639433L) ;
        p254.value_SET(-1.2731438E38F) ;
        p254.ind_SET((char)71) ;
        LoopBackDemoChannel.instance.send(p254);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SETUP_SIGNING.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)18);
            assert(pack.initial_timestamp_GET() == 1508103241211761460L);
            assert(pack.target_component_GET() == (char)184);
            assert(Arrays.equals(pack.secret_key_GET(),  new char[] {(char)87, (char)141, (char)206, (char)49, (char)60, (char)41, (char)31, (char)44, (char)182, (char)232, (char)13, (char)149, (char)73, (char)138, (char)41, (char)101, (char)206, (char)122, (char)42, (char)222, (char)173, (char)194, (char)120, (char)35, (char)19, (char)182, (char)191, (char)191, (char)75, (char)227, (char)86, (char)39}));
        });
        DemoDevice.SETUP_SIGNING p256 = LoopBackDemoChannel.new_SETUP_SIGNING();
        PH.setPack(p256);
        p256.secret_key_SET(new char[] {(char)87, (char)141, (char)206, (char)49, (char)60, (char)41, (char)31, (char)44, (char)182, (char)232, (char)13, (char)149, (char)73, (char)138, (char)41, (char)101, (char)206, (char)122, (char)42, (char)222, (char)173, (char)194, (char)120, (char)35, (char)19, (char)182, (char)191, (char)191, (char)75, (char)227, (char)86, (char)39}, 0) ;
        p256.target_component_SET((char)184) ;
        p256.initial_timestamp_SET(1508103241211761460L) ;
        p256.target_system_SET((char)18) ;
        LoopBackDemoChannel.instance.send(p256);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_BUTTON_CHANGE.add((src, ph, pack) ->
        {
            assert(pack.state_GET() == (char)62);
            assert(pack.last_change_ms_GET() == 2312684595L);
            assert(pack.time_boot_ms_GET() == 2175466060L);
        });
        DemoDevice.BUTTON_CHANGE p257 = LoopBackDemoChannel.new_BUTTON_CHANGE();
        PH.setPack(p257);
        p257.state_SET((char)62) ;
        p257.time_boot_ms_SET(2175466060L) ;
        p257.last_change_ms_SET(2312684595L) ;
        LoopBackDemoChannel.instance.send(p257);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PLAY_TUNE.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)207);
            assert(pack.tune_LEN(ph) == 10);
            assert(pack.tune_TRY(ph).equals("lgswypdeco"));
            assert(pack.target_component_GET() == (char)10);
        });
        DemoDevice.PLAY_TUNE p258 = LoopBackDemoChannel.new_PLAY_TUNE();
        PH.setPack(p258);
        p258.tune_SET("lgswypdeco", PH) ;
        p258.target_component_SET((char)10) ;
        p258.target_system_SET((char)207) ;
        LoopBackDemoChannel.instance.send(p258);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.cam_definition_version_GET() == (char)20056);
            assert(Arrays.equals(pack.model_name_GET(),  new char[] {(char)12, (char)252, (char)193, (char)102, (char)121, (char)199, (char)81, (char)19, (char)253, (char)83, (char)134, (char)218, (char)202, (char)140, (char)212, (char)77, (char)223, (char)77, (char)31, (char)111, (char)239, (char)138, (char)55, (char)1, (char)207, (char)46, (char)198, (char)175, (char)6, (char)229, (char)82, (char)72}));
            assert(pack.focal_length_GET() == -2.4149985E38F);
            assert(pack.time_boot_ms_GET() == 1684337859L);
            assert(pack.lens_id_GET() == (char)14);
            assert(pack.sensor_size_h_GET() == -3.211932E38F);
            assert(Arrays.equals(pack.vendor_name_GET(),  new char[] {(char)48, (char)249, (char)155, (char)161, (char)128, (char)26, (char)117, (char)42, (char)201, (char)160, (char)126, (char)220, (char)40, (char)128, (char)116, (char)100, (char)18, (char)6, (char)113, (char)179, (char)213, (char)85, (char)61, (char)174, (char)20, (char)107, (char)212, (char)146, (char)110, (char)75, (char)203, (char)161}));
            assert(pack.firmware_version_GET() == 1854014690L);
            assert(pack.flags_GET() == CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE);
            assert(pack.resolution_v_GET() == (char)25839);
            assert(pack.resolution_h_GET() == (char)53361);
            assert(pack.cam_definition_uri_LEN(ph) == 52);
            assert(pack.cam_definition_uri_TRY(ph).equals("IyaQnlaqctfplcqrjuqunpgaepvwpArwanTlqtbyfwoaanlgkmfi"));
            assert(pack.sensor_size_v_GET() == -2.2690153E38F);
        });
        DemoDevice.CAMERA_INFORMATION p259 = LoopBackDemoChannel.new_CAMERA_INFORMATION();
        PH.setPack(p259);
        p259.model_name_SET(new char[] {(char)12, (char)252, (char)193, (char)102, (char)121, (char)199, (char)81, (char)19, (char)253, (char)83, (char)134, (char)218, (char)202, (char)140, (char)212, (char)77, (char)223, (char)77, (char)31, (char)111, (char)239, (char)138, (char)55, (char)1, (char)207, (char)46, (char)198, (char)175, (char)6, (char)229, (char)82, (char)72}, 0) ;
        p259.sensor_size_v_SET(-2.2690153E38F) ;
        p259.cam_definition_version_SET((char)20056) ;
        p259.lens_id_SET((char)14) ;
        p259.sensor_size_h_SET(-3.211932E38F) ;
        p259.vendor_name_SET(new char[] {(char)48, (char)249, (char)155, (char)161, (char)128, (char)26, (char)117, (char)42, (char)201, (char)160, (char)126, (char)220, (char)40, (char)128, (char)116, (char)100, (char)18, (char)6, (char)113, (char)179, (char)213, (char)85, (char)61, (char)174, (char)20, (char)107, (char)212, (char)146, (char)110, (char)75, (char)203, (char)161}, 0) ;
        p259.resolution_v_SET((char)25839) ;
        p259.flags_SET(CAMERA_CAP_FLAGS.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE) ;
        p259.cam_definition_uri_SET("IyaQnlaqctfplcqrjuqunpgaepvwpArwanTlqtbyfwoaanlgkmfi", PH) ;
        p259.firmware_version_SET(1854014690L) ;
        p259.time_boot_ms_SET(1684337859L) ;
        p259.focal_length_SET(-2.4149985E38F) ;
        p259.resolution_h_SET((char)53361) ;
        LoopBackDemoChannel.instance.send(p259);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.mode_id_GET() == CAMERA_MODE.CAMERA_MODE_VIDEO);
            assert(pack.time_boot_ms_GET() == 1428634149L);
        });
        DemoDevice.CAMERA_SETTINGS p260 = LoopBackDemoChannel.new_CAMERA_SETTINGS();
        PH.setPack(p260);
        p260.time_boot_ms_SET(1428634149L) ;
        p260.mode_id_SET(CAMERA_MODE.CAMERA_MODE_VIDEO) ;
        LoopBackDemoChannel.instance.send(p260);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_STORAGE_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.storage_count_GET() == (char)157);
            assert(pack.read_speed_GET() == -1.6731048E38F);
            assert(pack.available_capacity_GET() == 1.3819138E37F);
            assert(pack.status_GET() == (char)117);
            assert(pack.used_capacity_GET() == -1.3365888E38F);
            assert(pack.write_speed_GET() == -5.2753675E37F);
            assert(pack.time_boot_ms_GET() == 666053627L);
            assert(pack.storage_id_GET() == (char)13);
            assert(pack.total_capacity_GET() == 2.066071E38F);
        });
        DemoDevice.STORAGE_INFORMATION p261 = LoopBackDemoChannel.new_STORAGE_INFORMATION();
        PH.setPack(p261);
        p261.write_speed_SET(-5.2753675E37F) ;
        p261.used_capacity_SET(-1.3365888E38F) ;
        p261.storage_id_SET((char)13) ;
        p261.storage_count_SET((char)157) ;
        p261.total_capacity_SET(2.066071E38F) ;
        p261.status_SET((char)117) ;
        p261.time_boot_ms_SET(666053627L) ;
        p261.read_speed_SET(-1.6731048E38F) ;
        p261.available_capacity_SET(1.3819138E37F) ;
        LoopBackDemoChannel.instance.send(p261);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_CAPTURE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.time_boot_ms_GET() == 1676998989L);
            assert(pack.video_status_GET() == (char)124);
            assert(pack.available_capacity_GET() == 2.0684776E38F);
            assert(pack.image_status_GET() == (char)25);
            assert(pack.recording_time_ms_GET() == 1840762169L);
            assert(pack.image_interval_GET() == -9.568485E37F);
        });
        DemoDevice.CAMERA_CAPTURE_STATUS p262 = LoopBackDemoChannel.new_CAMERA_CAPTURE_STATUS();
        PH.setPack(p262);
        p262.image_interval_SET(-9.568485E37F) ;
        p262.recording_time_ms_SET(1840762169L) ;
        p262.time_boot_ms_SET(1676998989L) ;
        p262.image_status_SET((char)25) ;
        p262.video_status_SET((char)124) ;
        p262.available_capacity_SET(2.0684776E38F) ;
        LoopBackDemoChannel.instance.send(p262);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_CAMERA_IMAGE_CAPTURED.add((src, ph, pack) ->
        {
            assert(pack.capture_result_GET() == (byte)124);
            assert(pack.time_boot_ms_GET() == 2246278990L);
            assert(pack.file_url_LEN(ph) == 65);
            assert(pack.file_url_TRY(ph).equals("wbAvcyioNpemsogfkuarmkiwmxArJxpCqzclrmzqmhywHcqsTmmcdcyCelqjCbpfy"));
            assert(pack.alt_GET() == -590512356);
            assert(pack.relative_alt_GET() == -374143277);
            assert(pack.camera_id_GET() == (char)155);
            assert(pack.time_utc_GET() == 5418250048822946073L);
            assert(Arrays.equals(pack.q_GET(),  new float[] {-1.972193E38F, -1.9261543E38F, 1.9909467E38F, -2.8021114E38F}));
            assert(pack.lon_GET() == -1170072280);
            assert(pack.image_index_GET() == -1612567671);
            assert(pack.lat_GET() == 1186490518);
        });
        DemoDevice.CAMERA_IMAGE_CAPTURED p263 = LoopBackDemoChannel.new_CAMERA_IMAGE_CAPTURED();
        PH.setPack(p263);
        p263.lon_SET(-1170072280) ;
        p263.file_url_SET("wbAvcyioNpemsogfkuarmkiwmxArJxpCqzclrmzqmhywHcqsTmmcdcyCelqjCbpfy", PH) ;
        p263.time_boot_ms_SET(2246278990L) ;
        p263.q_SET(new float[] {-1.972193E38F, -1.9261543E38F, 1.9909467E38F, -2.8021114E38F}, 0) ;
        p263.time_utc_SET(5418250048822946073L) ;
        p263.lat_SET(1186490518) ;
        p263.relative_alt_SET(-374143277) ;
        p263.capture_result_SET((byte)124) ;
        p263.alt_SET(-590512356) ;
        p263.image_index_SET(-1612567671) ;
        p263.camera_id_SET((char)155) ;
        LoopBackDemoChannel.instance.send(p263);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_FLIGHT_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.arming_time_utc_GET() == 6406458908928029423L);
            assert(pack.flight_uuid_GET() == 8845428658296486795L);
            assert(pack.time_boot_ms_GET() == 2791282208L);
            assert(pack.takeoff_time_utc_GET() == 7802859151771674217L);
        });
        DemoDevice.FLIGHT_INFORMATION p264 = LoopBackDemoChannel.new_FLIGHT_INFORMATION();
        PH.setPack(p264);
        p264.takeoff_time_utc_SET(7802859151771674217L) ;
        p264.flight_uuid_SET(8845428658296486795L) ;
        p264.time_boot_ms_SET(2791282208L) ;
        p264.arming_time_utc_SET(6406458908928029423L) ;
        LoopBackDemoChannel.instance.send(p264);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_MOUNT_ORIENTATION.add((src, ph, pack) ->
        {
            assert(pack.pitch_GET() == 1.660593E38F);
            assert(pack.time_boot_ms_GET() == 2617069943L);
            assert(pack.roll_GET() == -3.375475E38F);
            assert(pack.yaw_GET() == 6.616658E37F);
        });
        DemoDevice.MOUNT_ORIENTATION p265 = LoopBackDemoChannel.new_MOUNT_ORIENTATION();
        PH.setPack(p265);
        p265.roll_SET(-3.375475E38F) ;
        p265.time_boot_ms_SET(2617069943L) ;
        p265.pitch_SET(1.660593E38F) ;
        p265.yaw_SET(6.616658E37F) ;
        LoopBackDemoChannel.instance.send(p265);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA.add((src, ph, pack) ->
        {
            assert(pack.target_component_GET() == (char)28);
            assert(pack.length_GET() == (char)192);
            assert(pack.target_system_GET() == (char)0);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)204, (char)195, (char)95, (char)232, (char)174, (char)232, (char)190, (char)133, (char)171, (char)212, (char)193, (char)173, (char)162, (char)207, (char)50, (char)244, (char)147, (char)115, (char)214, (char)40, (char)136, (char)32, (char)174, (char)174, (char)173, (char)153, (char)18, (char)124, (char)91, (char)105, (char)16, (char)223, (char)87, (char)208, (char)36, (char)253, (char)35, (char)182, (char)190, (char)230, (char)131, (char)192, (char)106, (char)32, (char)163, (char)164, (char)216, (char)224, (char)6, (char)144, (char)7, (char)163, (char)31, (char)116, (char)102, (char)22, (char)19, (char)197, (char)187, (char)110, (char)154, (char)10, (char)253, (char)212, (char)82, (char)95, (char)157, (char)251, (char)67, (char)141, (char)122, (char)204, (char)191, (char)128, (char)114, (char)24, (char)171, (char)241, (char)3, (char)54, (char)128, (char)239, (char)36, (char)87, (char)62, (char)126, (char)189, (char)58, (char)176, (char)122, (char)110, (char)103, (char)11, (char)36, (char)167, (char)236, (char)207, (char)197, (char)116, (char)27, (char)243, (char)7, (char)96, (char)155, (char)20, (char)75, (char)239, (char)238, (char)128, (char)36, (char)248, (char)159, (char)34, (char)158, (char)145, (char)94, (char)77, (char)22, (char)125, (char)142, (char)154, (char)107, (char)33, (char)240, (char)12, (char)177, (char)58, (char)60, (char)116, (char)53, (char)148, (char)77, (char)229, (char)198, (char)245, (char)151, (char)32, (char)141, (char)192, (char)186, (char)32, (char)99, (char)178, (char)88, (char)11, (char)47, (char)174, (char)11, (char)205, (char)140, (char)8, (char)144, (char)92, (char)249, (char)71, (char)175, (char)170, (char)59, (char)68, (char)120, (char)79, (char)223, (char)129, (char)177, (char)48, (char)129, (char)93, (char)132, (char)160, (char)99, (char)39, (char)190, (char)63, (char)15, (char)165, (char)98, (char)20, (char)53, (char)31, (char)162, (char)235, (char)223, (char)235, (char)119, (char)248, (char)241, (char)43, (char)4, (char)236, (char)137, (char)179, (char)114, (char)88, (char)186, (char)51, (char)11, (char)134, (char)141, (char)82, (char)253, (char)212, (char)10, (char)15, (char)18, (char)85, (char)12, (char)189, (char)178, (char)218, (char)80, (char)160, (char)126, (char)76, (char)1, (char)128, (char)204, (char)86, (char)233, (char)222, (char)228, (char)69, (char)253, (char)143, (char)130, (char)124, (char)203, (char)46, (char)46, (char)227, (char)195, (char)150, (char)154, (char)224, (char)203, (char)26, (char)56, (char)144, (char)223, (char)26, (char)185, (char)109, (char)8, (char)22, (char)86, (char)145, (char)254, (char)254, (char)60, (char)188}));
            assert(pack.first_message_offset_GET() == (char)175);
            assert(pack.sequence_GET() == (char)31186);
        });
        DemoDevice.LOGGING_DATA p266 = LoopBackDemoChannel.new_LOGGING_DATA();
        PH.setPack(p266);
        p266.data__SET(new char[] {(char)204, (char)195, (char)95, (char)232, (char)174, (char)232, (char)190, (char)133, (char)171, (char)212, (char)193, (char)173, (char)162, (char)207, (char)50, (char)244, (char)147, (char)115, (char)214, (char)40, (char)136, (char)32, (char)174, (char)174, (char)173, (char)153, (char)18, (char)124, (char)91, (char)105, (char)16, (char)223, (char)87, (char)208, (char)36, (char)253, (char)35, (char)182, (char)190, (char)230, (char)131, (char)192, (char)106, (char)32, (char)163, (char)164, (char)216, (char)224, (char)6, (char)144, (char)7, (char)163, (char)31, (char)116, (char)102, (char)22, (char)19, (char)197, (char)187, (char)110, (char)154, (char)10, (char)253, (char)212, (char)82, (char)95, (char)157, (char)251, (char)67, (char)141, (char)122, (char)204, (char)191, (char)128, (char)114, (char)24, (char)171, (char)241, (char)3, (char)54, (char)128, (char)239, (char)36, (char)87, (char)62, (char)126, (char)189, (char)58, (char)176, (char)122, (char)110, (char)103, (char)11, (char)36, (char)167, (char)236, (char)207, (char)197, (char)116, (char)27, (char)243, (char)7, (char)96, (char)155, (char)20, (char)75, (char)239, (char)238, (char)128, (char)36, (char)248, (char)159, (char)34, (char)158, (char)145, (char)94, (char)77, (char)22, (char)125, (char)142, (char)154, (char)107, (char)33, (char)240, (char)12, (char)177, (char)58, (char)60, (char)116, (char)53, (char)148, (char)77, (char)229, (char)198, (char)245, (char)151, (char)32, (char)141, (char)192, (char)186, (char)32, (char)99, (char)178, (char)88, (char)11, (char)47, (char)174, (char)11, (char)205, (char)140, (char)8, (char)144, (char)92, (char)249, (char)71, (char)175, (char)170, (char)59, (char)68, (char)120, (char)79, (char)223, (char)129, (char)177, (char)48, (char)129, (char)93, (char)132, (char)160, (char)99, (char)39, (char)190, (char)63, (char)15, (char)165, (char)98, (char)20, (char)53, (char)31, (char)162, (char)235, (char)223, (char)235, (char)119, (char)248, (char)241, (char)43, (char)4, (char)236, (char)137, (char)179, (char)114, (char)88, (char)186, (char)51, (char)11, (char)134, (char)141, (char)82, (char)253, (char)212, (char)10, (char)15, (char)18, (char)85, (char)12, (char)189, (char)178, (char)218, (char)80, (char)160, (char)126, (char)76, (char)1, (char)128, (char)204, (char)86, (char)233, (char)222, (char)228, (char)69, (char)253, (char)143, (char)130, (char)124, (char)203, (char)46, (char)46, (char)227, (char)195, (char)150, (char)154, (char)224, (char)203, (char)26, (char)56, (char)144, (char)223, (char)26, (char)185, (char)109, (char)8, (char)22, (char)86, (char)145, (char)254, (char)254, (char)60, (char)188}, 0) ;
        p266.target_system_SET((char)0) ;
        p266.first_message_offset_SET((char)175) ;
        p266.length_SET((char)192) ;
        p266.sequence_SET((char)31186) ;
        p266.target_component_SET((char)28) ;
        LoopBackDemoChannel.instance.send(p266);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_DATA_ACKED.add((src, ph, pack) ->
        {
            assert(pack.sequence_GET() == (char)8962);
            assert(Arrays.equals(pack.data__GET(),  new char[] {(char)149, (char)213, (char)155, (char)127, (char)235, (char)27, (char)118, (char)171, (char)71, (char)160, (char)82, (char)179, (char)219, (char)32, (char)82, (char)174, (char)196, (char)144, (char)211, (char)165, (char)23, (char)136, (char)40, (char)121, (char)158, (char)200, (char)177, (char)9, (char)181, (char)249, (char)185, (char)207, (char)1, (char)148, (char)20, (char)96, (char)244, (char)42, (char)76, (char)158, (char)118, (char)245, (char)132, (char)123, (char)37, (char)255, (char)141, (char)117, (char)33, (char)244, (char)45, (char)236, (char)149, (char)211, (char)219, (char)180, (char)9, (char)67, (char)54, (char)198, (char)83, (char)189, (char)207, (char)202, (char)58, (char)138, (char)180, (char)13, (char)121, (char)14, (char)141, (char)88, (char)203, (char)18, (char)27, (char)228, (char)158, (char)232, (char)23, (char)21, (char)219, (char)199, (char)160, (char)157, (char)17, (char)178, (char)20, (char)182, (char)162, (char)45, (char)17, (char)61, (char)33, (char)30, (char)100, (char)87, (char)246, (char)167, (char)79, (char)207, (char)167, (char)53, (char)185, (char)215, (char)235, (char)197, (char)34, (char)113, (char)230, (char)103, (char)226, (char)85, (char)47, (char)129, (char)150, (char)58, (char)226, (char)213, (char)70, (char)27, (char)203, (char)43, (char)9, (char)63, (char)246, (char)176, (char)32, (char)145, (char)85, (char)59, (char)95, (char)77, (char)60, (char)90, (char)250, (char)218, (char)132, (char)91, (char)79, (char)156, (char)25, (char)210, (char)170, (char)106, (char)70, (char)190, (char)202, (char)130, (char)113, (char)62, (char)29, (char)93, (char)158, (char)85, (char)196, (char)102, (char)21, (char)169, (char)182, (char)40, (char)159, (char)154, (char)179, (char)5, (char)193, (char)201, (char)213, (char)190, (char)179, (char)212, (char)237, (char)29, (char)102, (char)40, (char)72, (char)165, (char)13, (char)137, (char)59, (char)62, (char)114, (char)135, (char)18, (char)61, (char)215, (char)168, (char)105, (char)41, (char)104, (char)237, (char)138, (char)118, (char)81, (char)143, (char)153, (char)4, (char)221, (char)110, (char)105, (char)197, (char)179, (char)69, (char)80, (char)112, (char)155, (char)172, (char)220, (char)170, (char)20, (char)144, (char)244, (char)251, (char)158, (char)224, (char)62, (char)208, (char)23, (char)247, (char)140, (char)25, (char)127, (char)84, (char)90, (char)77, (char)23, (char)185, (char)146, (char)15, (char)208, (char)11, (char)49, (char)36, (char)119, (char)36, (char)72, (char)87, (char)14, (char)123, (char)60, (char)92, (char)29, (char)61, (char)93, (char)13, (char)148, (char)135, (char)15, (char)169, (char)42}));
            assert(pack.first_message_offset_GET() == (char)113);
            assert(pack.target_system_GET() == (char)101);
            assert(pack.length_GET() == (char)207);
            assert(pack.target_component_GET() == (char)125);
        });
        DemoDevice.LOGGING_DATA_ACKED p267 = LoopBackDemoChannel.new_LOGGING_DATA_ACKED();
        PH.setPack(p267);
        p267.data__SET(new char[] {(char)149, (char)213, (char)155, (char)127, (char)235, (char)27, (char)118, (char)171, (char)71, (char)160, (char)82, (char)179, (char)219, (char)32, (char)82, (char)174, (char)196, (char)144, (char)211, (char)165, (char)23, (char)136, (char)40, (char)121, (char)158, (char)200, (char)177, (char)9, (char)181, (char)249, (char)185, (char)207, (char)1, (char)148, (char)20, (char)96, (char)244, (char)42, (char)76, (char)158, (char)118, (char)245, (char)132, (char)123, (char)37, (char)255, (char)141, (char)117, (char)33, (char)244, (char)45, (char)236, (char)149, (char)211, (char)219, (char)180, (char)9, (char)67, (char)54, (char)198, (char)83, (char)189, (char)207, (char)202, (char)58, (char)138, (char)180, (char)13, (char)121, (char)14, (char)141, (char)88, (char)203, (char)18, (char)27, (char)228, (char)158, (char)232, (char)23, (char)21, (char)219, (char)199, (char)160, (char)157, (char)17, (char)178, (char)20, (char)182, (char)162, (char)45, (char)17, (char)61, (char)33, (char)30, (char)100, (char)87, (char)246, (char)167, (char)79, (char)207, (char)167, (char)53, (char)185, (char)215, (char)235, (char)197, (char)34, (char)113, (char)230, (char)103, (char)226, (char)85, (char)47, (char)129, (char)150, (char)58, (char)226, (char)213, (char)70, (char)27, (char)203, (char)43, (char)9, (char)63, (char)246, (char)176, (char)32, (char)145, (char)85, (char)59, (char)95, (char)77, (char)60, (char)90, (char)250, (char)218, (char)132, (char)91, (char)79, (char)156, (char)25, (char)210, (char)170, (char)106, (char)70, (char)190, (char)202, (char)130, (char)113, (char)62, (char)29, (char)93, (char)158, (char)85, (char)196, (char)102, (char)21, (char)169, (char)182, (char)40, (char)159, (char)154, (char)179, (char)5, (char)193, (char)201, (char)213, (char)190, (char)179, (char)212, (char)237, (char)29, (char)102, (char)40, (char)72, (char)165, (char)13, (char)137, (char)59, (char)62, (char)114, (char)135, (char)18, (char)61, (char)215, (char)168, (char)105, (char)41, (char)104, (char)237, (char)138, (char)118, (char)81, (char)143, (char)153, (char)4, (char)221, (char)110, (char)105, (char)197, (char)179, (char)69, (char)80, (char)112, (char)155, (char)172, (char)220, (char)170, (char)20, (char)144, (char)244, (char)251, (char)158, (char)224, (char)62, (char)208, (char)23, (char)247, (char)140, (char)25, (char)127, (char)84, (char)90, (char)77, (char)23, (char)185, (char)146, (char)15, (char)208, (char)11, (char)49, (char)36, (char)119, (char)36, (char)72, (char)87, (char)14, (char)123, (char)60, (char)92, (char)29, (char)61, (char)93, (char)13, (char)148, (char)135, (char)15, (char)169, (char)42}, 0) ;
        p267.sequence_SET((char)8962) ;
        p267.target_component_SET((char)125) ;
        p267.first_message_offset_SET((char)113) ;
        p267.length_SET((char)207) ;
        p267.target_system_SET((char)101) ;
        LoopBackDemoChannel.instance.send(p267);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_LOGGING_ACK.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)150);
            assert(pack.sequence_GET() == (char)61797);
            assert(pack.target_component_GET() == (char)23);
        });
        DemoDevice.LOGGING_ACK p268 = LoopBackDemoChannel.new_LOGGING_ACK();
        PH.setPack(p268);
        p268.target_system_SET((char)150) ;
        p268.target_component_SET((char)23) ;
        p268.sequence_SET((char)61797) ;
        LoopBackDemoChannel.instance.send(p268);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_VIDEO_STREAM_INFORMATION.add((src, ph, pack) ->
        {
            assert(pack.resolution_h_GET() == (char)64729);
            assert(pack.uri_LEN(ph) == 219);
            assert(pack.uri_TRY(ph).equals("cwuoIyryPdfubgfunufuqvatwqwbtfjmuakzgmwfSegbdDRjtcolvEdpaqwaguabvwsrazrVppzcuhEnhiWdbytpbjnwfxjiTegvmlxzPsIudhOewwynhtXrtuqaXRrahfascIxjtjypvYkapaBahmvEKrpkrpDlnrtnttxdztpjpcwjztbQeozFRswakYpejZfnjdrhwvugiahjgdbzuefmpjg"));
            assert(pack.rotation_GET() == (char)33324);
            assert(pack.camera_id_GET() == (char)102);
            assert(pack.bitrate_GET() == 2993531472L);
            assert(pack.framerate_GET() == -4.546559E37F);
            assert(pack.resolution_v_GET() == (char)51512);
            assert(pack.status_GET() == (char)50);
        });
        DemoDevice.VIDEO_STREAM_INFORMATION p269 = LoopBackDemoChannel.new_VIDEO_STREAM_INFORMATION();
        PH.setPack(p269);
        p269.resolution_v_SET((char)51512) ;
        p269.uri_SET("cwuoIyryPdfubgfunufuqvatwqwbtfjmuakzgmwfSegbdDRjtcolvEdpaqwaguabvwsrazrVppzcuhEnhiWdbytpbjnwfxjiTegvmlxzPsIudhOewwynhtXrtuqaXRrahfascIxjtjypvYkapaBahmvEKrpkrpDlnrtnttxdztpjpcwjztbQeozFRswakYpejZfnjdrhwvugiahjgdbzuefmpjg", PH) ;
        p269.rotation_SET((char)33324) ;
        p269.resolution_h_SET((char)64729) ;
        p269.camera_id_SET((char)102) ;
        p269.bitrate_SET(2993531472L) ;
        p269.framerate_SET(-4.546559E37F) ;
        p269.status_SET((char)50) ;
        LoopBackDemoChannel.instance.send(p269);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_SET_VIDEO_STREAM_SETTINGS.add((src, ph, pack) ->
        {
            assert(pack.bitrate_GET() == 1200784007L);
            assert(pack.uri_LEN(ph) == 185);
            assert(pack.uri_TRY(ph).equals("lwsczjvukxyrugmyyEkeqJeefiyumaqrawkttkfnrqzBhktaxcsjcnihvsthIIvkgxkkoniAkxnrdxmBayokfdeeKksamvwotrbjtcodypXYisdycnwjyisiewozfAKknmNoymzscttnlUpveyYocwrqwiexsmtqkuvdndozajbgggxlkifQwuuxy"));
            assert(pack.target_system_GET() == (char)225);
            assert(pack.target_component_GET() == (char)186);
            assert(pack.camera_id_GET() == (char)236);
            assert(pack.resolution_h_GET() == (char)18791);
            assert(pack.resolution_v_GET() == (char)32617);
            assert(pack.framerate_GET() == 1.6967558E38F);
            assert(pack.rotation_GET() == (char)50260);
        });
        DemoDevice.SET_VIDEO_STREAM_SETTINGS p270 = LoopBackDemoChannel.new_SET_VIDEO_STREAM_SETTINGS();
        PH.setPack(p270);
        p270.resolution_v_SET((char)32617) ;
        p270.bitrate_SET(1200784007L) ;
        p270.rotation_SET((char)50260) ;
        p270.resolution_h_SET((char)18791) ;
        p270.target_system_SET((char)225) ;
        p270.target_component_SET((char)186) ;
        p270.uri_SET("lwsczjvukxyrugmyyEkeqJeefiyumaqrawkttkfnrqzBhktaxcsjcnihvsthIIvkgxkkoniAkxnrdxmBayokfdeeKksamvwotrbjtcodypXYisdycnwjyisiewozfAKknmNoymzscttnlUpveyYocwrqwiexsmtqkuvdndozajbgggxlkifQwuuxy", PH) ;
        p270.framerate_SET(1.6967558E38F) ;
        p270.camera_id_SET((char)236) ;
        LoopBackDemoChannel.instance.send(p270);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_WIFI_CONFIG_AP.add((src, ph, pack) ->
        {
            assert(pack.password_LEN(ph) == 41);
            assert(pack.password_TRY(ph).equals("mdfzusygkpglCenkdcynXrfjzgumfuzzuuihmiizb"));
            assert(pack.ssid_LEN(ph) == 25);
            assert(pack.ssid_TRY(ph).equals("bsceofmHdlnasiMzrkhlriWse"));
        });
        DemoDevice.WIFI_CONFIG_AP p299 = LoopBackDemoChannel.new_WIFI_CONFIG_AP();
        PH.setPack(p299);
        p299.password_SET("mdfzusygkpglCenkdcynXrfjzgumfuzzuuihmiizb", PH) ;
        p299.ssid_SET("bsceofmHdlnasiMzrkhlriWse", PH) ;
        LoopBackDemoChannel.instance.send(p299);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PROTOCOL_VERSION.add((src, ph, pack) ->
        {
            assert(Arrays.equals(pack.library_version_hash_GET(),  new char[] {(char)43, (char)85, (char)190, (char)179, (char)20, (char)9, (char)0, (char)211}));
            assert(pack.version_GET() == (char)53026);
            assert(pack.min_version_GET() == (char)54149);
            assert(pack.max_version_GET() == (char)17808);
            assert(Arrays.equals(pack.spec_version_hash_GET(),  new char[] {(char)144, (char)34, (char)98, (char)232, (char)47, (char)14, (char)5, (char)91}));
        });
        DemoDevice.PROTOCOL_VERSION p300 = LoopBackDemoChannel.new_PROTOCOL_VERSION();
        PH.setPack(p300);
        p300.max_version_SET((char)17808) ;
        p300.spec_version_hash_SET(new char[] {(char)144, (char)34, (char)98, (char)232, (char)47, (char)14, (char)5, (char)91}, 0) ;
        p300.version_SET((char)53026) ;
        p300.min_version_SET((char)54149) ;
        p300.library_version_hash_SET(new char[] {(char)43, (char)85, (char)190, (char)179, (char)20, (char)9, (char)0, (char)211}, 0) ;
        LoopBackDemoChannel.instance.send(p300);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_STATUS.add((src, ph, pack) ->
        {
            assert(pack.uptime_sec_GET() == 1495198384L);
            assert(pack.mode_GET() == UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL);
            assert(pack.sub_mode_GET() == (char)64);
            assert(pack.time_usec_GET() == 1806198320069379414L);
            assert(pack.health_GET() == UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR);
            assert(pack.vendor_specific_status_code_GET() == (char)25267);
        });
        DemoDevice.UAVCAN_NODE_STATUS p310 = LoopBackDemoChannel.new_UAVCAN_NODE_STATUS();
        PH.setPack(p310);
        p310.uptime_sec_SET(1495198384L) ;
        p310.mode_SET(UAVCAN_NODE_MODE.UAVCAN_NODE_MODE_OPERATIONAL) ;
        p310.vendor_specific_status_code_SET((char)25267) ;
        p310.health_SET(UAVCAN_NODE_HEALTH.UAVCAN_NODE_HEALTH_ERROR) ;
        p310.sub_mode_SET((char)64) ;
        p310.time_usec_SET(1806198320069379414L) ;
        LoopBackDemoChannel.instance.send(p310);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_UAVCAN_NODE_INFO.add((src, ph, pack) ->
        {
            assert(pack.sw_version_minor_GET() == (char)239);
            assert(pack.sw_vcs_commit_GET() == 1543016685L);
            assert(pack.hw_version_minor_GET() == (char)212);
            assert(pack.sw_version_major_GET() == (char)50);
            assert(pack.uptime_sec_GET() == 1962890588L);
            assert(Arrays.equals(pack.hw_unique_id_GET(),  new char[] {(char)239, (char)182, (char)190, (char)215, (char)98, (char)229, (char)198, (char)76, (char)163, (char)85, (char)54, (char)18, (char)204, (char)214, (char)174, (char)116}));
            assert(pack.hw_version_major_GET() == (char)31);
            assert(pack.name_LEN(ph) == 5);
            assert(pack.name_TRY(ph).equals("rbWsv"));
            assert(pack.time_usec_GET() == 3320563238221346365L);
        });
        DemoDevice.UAVCAN_NODE_INFO p311 = LoopBackDemoChannel.new_UAVCAN_NODE_INFO();
        PH.setPack(p311);
        p311.hw_version_major_SET((char)31) ;
        p311.hw_version_minor_SET((char)212) ;
        p311.sw_version_minor_SET((char)239) ;
        p311.uptime_sec_SET(1962890588L) ;
        p311.time_usec_SET(3320563238221346365L) ;
        p311.sw_version_major_SET((char)50) ;
        p311.name_SET("rbWsv", PH) ;
        p311.sw_vcs_commit_SET(1543016685L) ;
        p311.hw_unique_id_SET(new char[] {(char)239, (char)182, (char)190, (char)215, (char)98, (char)229, (char)198, (char)76, (char)163, (char)85, (char)54, (char)18, (char)204, (char)214, (char)174, (char)116}, 0) ;
        LoopBackDemoChannel.instance.send(p311);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_READ.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)26);
            assert(pack.param_id_LEN(ph) == 11);
            assert(pack.param_id_TRY(ph).equals("btMfoxxxhoz"));
            assert(pack.target_component_GET() == (char)59);
            assert(pack.param_index_GET() == (short)29571);
        });
        DemoDevice.PARAM_EXT_REQUEST_READ p320 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_READ();
        PH.setPack(p320);
        p320.param_id_SET("btMfoxxxhoz", PH) ;
        p320.param_index_SET((short)29571) ;
        p320.target_system_SET((char)26) ;
        p320.target_component_SET((char)59) ;
        LoopBackDemoChannel.instance.send(p320);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_REQUEST_LIST.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)140);
            assert(pack.target_component_GET() == (char)136);
        });
        DemoDevice.PARAM_EXT_REQUEST_LIST p321 = LoopBackDemoChannel.new_PARAM_EXT_REQUEST_LIST();
        PH.setPack(p321);
        p321.target_component_SET((char)136) ;
        p321.target_system_SET((char)140) ;
        LoopBackDemoChannel.instance.send(p321);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_VALUE.add((src, ph, pack) ->
        {
            assert(pack.param_id_LEN(ph) == 4);
            assert(pack.param_id_TRY(ph).equals("quha"));
            assert(pack.param_index_GET() == (char)14867);
            assert(pack.param_value_LEN(ph) == 25);
            assert(pack.param_value_TRY(ph).equals("zhdnmkmxfnbgnialcymgYekif"));
            assert(pack.param_count_GET() == (char)39984);
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64);
        });
        DemoDevice.PARAM_EXT_VALUE p322 = LoopBackDemoChannel.new_PARAM_EXT_VALUE();
        PH.setPack(p322);
        p322.param_count_SET((char)39984) ;
        p322.param_id_SET("quha", PH) ;
        p322.param_index_SET((char)14867) ;
        p322.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_UINT64) ;
        p322.param_value_SET("zhdnmkmxfnbgnialcymgYekif", PH) ;
        LoopBackDemoChannel.instance.send(p322);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_SET.add((src, ph, pack) ->
        {
            assert(pack.target_system_GET() == (char)2);
            assert(pack.param_id_LEN(ph) == 16);
            assert(pack.param_id_TRY(ph).equals("txistHtTophjsjmq"));
            assert(pack.param_value_LEN(ph) == 78);
            assert(pack.param_value_TRY(ph).equals("zwujorsdutbhvasojqfmivxspsbjdvhcimlPqzcvigojornvgpgorthorcwxnzbbshntkytitkitnh"));
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64);
            assert(pack.target_component_GET() == (char)228);
        });
        DemoDevice.PARAM_EXT_SET p323 = LoopBackDemoChannel.new_PARAM_EXT_SET();
        PH.setPack(p323);
        p323.target_system_SET((char)2) ;
        p323.target_component_SET((char)228) ;
        p323.param_id_SET("txistHtTophjsjmq", PH) ;
        p323.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_REAL64) ;
        p323.param_value_SET("zwujorsdutbhvasojqfmivxspsbjdvhcimlPqzcvigojornvgpgorthorcwxnzbbshntkytitkitnh", PH) ;
        LoopBackDemoChannel.instance.send(p323);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_PARAM_EXT_ACK.add((src, ph, pack) ->
        {
            assert(pack.param_type_GET() == MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8);
            assert(pack.param_result_GET() == PARAM_ACK.PARAM_ACK_IN_PROGRESS);
            assert(pack.param_value_LEN(ph) == 91);
            assert(pack.param_value_TRY(ph).equals("cempgszbmxmftfzysdBwjoLlPannYijxmliojewotUimiLkRibaqmawzaxsgbsxvcQblwbfYxjhhbqffxnksdsotlPm"));
            assert(pack.param_id_LEN(ph) == 6);
            assert(pack.param_id_TRY(ph).equals("vdvaya"));
        });
        DemoDevice.PARAM_EXT_ACK p324 = LoopBackDemoChannel.new_PARAM_EXT_ACK();
        PH.setPack(p324);
        p324.param_value_SET("cempgszbmxmftfzysdBwjoLlPannYijxmliojewotUimiLkRibaqmawzaxsgbsxvcQblwbfYxjhhbqffxnksdsotlPm", PH) ;
        p324.param_type_SET(MAV_PARAM_EXT_TYPE.MAV_PARAM_EXT_TYPE_INT8) ;
        p324.param_result_SET(PARAM_ACK.PARAM_ACK_IN_PROGRESS) ;
        p324.param_id_SET("vdvaya", PH) ;
        LoopBackDemoChannel.instance.send(p324);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
        LoopBackDemoChannel.instance.on_OBSTACLE_DISTANCE.add((src, ph, pack) ->
        {
            assert(pack.max_distance_GET() == (char)31364);
            assert(Arrays.equals(pack.distances_GET(),  new char[] {(char)41215, (char)51431, (char)62339, (char)11963, (char)19660, (char)47855, (char)64120, (char)38445, (char)25385, (char)26786, (char)11875, (char)62078, (char)36124, (char)57056, (char)54840, (char)9482, (char)47311, (char)5041, (char)26395, (char)27232, (char)41846, (char)17330, (char)55165, (char)30184, (char)7501, (char)22379, (char)63110, (char)61984, (char)57356, (char)15030, (char)24381, (char)18221, (char)55640, (char)26515, (char)49814, (char)22192, (char)18078, (char)37297, (char)27780, (char)18704, (char)21722, (char)20028, (char)40001, (char)61923, (char)53823, (char)52085, (char)57919, (char)16422, (char)13113, (char)23558, (char)20339, (char)56222, (char)20220, (char)42525, (char)39944, (char)47488, (char)36636, (char)47382, (char)9365, (char)12056, (char)59786, (char)6238, (char)58058, (char)29817, (char)50570, (char)4510, (char)36994, (char)16907, (char)14992, (char)10144, (char)35125, (char)49913}));
            assert(pack.increment_GET() == (char)130);
            assert(pack.time_usec_GET() == 6715974181559269861L);
            assert(pack.sensor_type_GET() == MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND);
            assert(pack.min_distance_GET() == (char)64001);
        });
        DemoDevice.OBSTACLE_DISTANCE p330 = LoopBackDemoChannel.new_OBSTACLE_DISTANCE();
        PH.setPack(p330);
        p330.max_distance_SET((char)31364) ;
        p330.time_usec_SET(6715974181559269861L) ;
        p330.sensor_type_SET(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND) ;
        p330.distances_SET(new char[] {(char)41215, (char)51431, (char)62339, (char)11963, (char)19660, (char)47855, (char)64120, (char)38445, (char)25385, (char)26786, (char)11875, (char)62078, (char)36124, (char)57056, (char)54840, (char)9482, (char)47311, (char)5041, (char)26395, (char)27232, (char)41846, (char)17330, (char)55165, (char)30184, (char)7501, (char)22379, (char)63110, (char)61984, (char)57356, (char)15030, (char)24381, (char)18221, (char)55640, (char)26515, (char)49814, (char)22192, (char)18078, (char)37297, (char)27780, (char)18704, (char)21722, (char)20028, (char)40001, (char)61923, (char)53823, (char)52085, (char)57919, (char)16422, (char)13113, (char)23558, (char)20339, (char)56222, (char)20220, (char)42525, (char)39944, (char)47488, (char)36636, (char)47382, (char)9365, (char)12056, (char)59786, (char)6238, (char)58058, (char)29817, (char)50570, (char)4510, (char)36994, (char)16907, (char)14992, (char)10144, (char)35125, (char)49913}, 0) ;
        p330.min_distance_SET((char)64001) ;
        p330.increment_SET((char)130) ;
        LoopBackDemoChannel.instance.send(p330);//put test pack to the  channel send buffer
        TestChannel.transmission(LoopBackDemoChannel.instance.inputStream, LoopBackDemoChannel.instance.outputStream, LoopBackDemoChannel.instance);
    }

}